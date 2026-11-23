#!/usr/bin/env python3
"""contract_verify.py — 把**實際生效值**讀回來對照契約表,不合就擋下航次。

⚠️⚠️ 與 `preflight_check.py` 的分工(兩支都要跑,不是二選一):

    preflight_check.py   飛控 PARM 讀回 + 幾何/死區關係檢查(已存在)
    contract_verify.py   **節點層生效值** 讀回 + 泡棉禁區 + 跨層不變式(本檔)

先前的缺口是:節點層的 `vmin/vmax/dive_speed/dive_pace/gate_r/formation_alt/
patrol_alt` **完全沒有任何檢查覆蓋** —— 它們不在飛控裡,`preflight_check.py`
的 pymavlink 讀回根本構不到。而 `tot_node.py` 的 declare_parameter 預設值
全部是泡棉時代的,任何一條餵值路徑斷掉都會**靜靜退回**去。

## 三種取值模式

    contract_verify.py                     # 現場:ros2 param + 飛控 PARM + env
    contract_verify.py --from-log <log>    # 事後:從產物 log 的契約行重建
    contract_verify.py --snapshot <path>   # 併同寫出生效值快照(JSON)

`--from-log` 是為了回答「**那一趟到底跑的是什麼**」——事後判讀時不必靠記憶
或猜。它讀 `tot_node.py` 的 `CONTRACT_SENTINEL` 那一行。

## ⚠️ 讀不到 = 違規

不是跳過。「沒讀到所以算過」正是這一整類 bug 的形狀:
`preflight_check.py` 檔頭第 1 條(`JET_CDP` 為空 ⇒ 整段改裝被跳過、
幾十趟跑原廠阻力)就是這樣來的。

## 降級

    STRICT=0 contract_verify.py     # 只警告不擋(**不建議**)

⚠️ 降級本身會被大聲印出來 —— 靜默降級是這一整類 bug 的成因。
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import re
import subprocess
import sys
import time

# ⚠️⚠️ 兩種佈局下都要 import 得到,而且**優先用部署版**:
#   repo  scripts/wsl/contract_verify.py + scripts/wsl/sar_swarm_patches/contract_table.py
#   WSL   ~/fixwing_ws/sitl_run/contract_verify.py + 已安裝的 sar_swarm.contract_table
# 部署後兩者**不在同一層** —— 只寫相對路徑會在 WSL 端 ImportError,
# 而那正是 deploy_build.sh 檔頭記的「防靜默失敗的工具自己踩靜默失敗」。
# 先試已安裝的 package:它是 deploy_build.sh 用 md5 驗過的那一份;
# 相對路徑只是開發機直跑的退路,不可以蓋過部署版。
try:
    from sar_swarm.contract_table import (  # noqa: E402
        SOURCE_FC, SOURCE_NODE, SOURCE_SHELL, TABLE, evaluate, rerun_hint,
    )
except ImportError:
    sys.path.insert(0,
                    str(pathlib.Path(__file__).resolve().parent / "sar_swarm_patches"))
    from contract_table import (  # noqa: E402
        SOURCE_FC, SOURCE_NODE, SOURCE_SHELL, TABLE, evaluate, rerun_hint,
    )

#: 與 tot_node.py 的 CONTRACT_SENTINEL 逐字相同(刻意不帶括號,見該處註解)。
#: ⚠️ 改一邊沒改另一邊 = 契約行永遠讀不到 ⇒ 全部判 missing,
#:    而「全部讀不到」看起來像「節點沒起來」—— 歸因會歪掉。
#:    tests/test_contract_table.py::TestSentinelsAgree 比對兩邊。
CONTRACT_SENTINEL = "生效值契約"

#: 節點 basename → 要從它身上讀回的參數名。
#: ⚠️ 分散在多個節點是事實而不是設計失誤:formation_alt 由編隊層擁有、
#:    vmin 由 tot_node 擁有。契約層必須**跨節點**取值,只問 tot_node 會漏掉
#:    formation-layer-cannot-fly-the-jet 那一整類。
NODE_PARAMS = {
    "tot_node": ["vmin", "vmax", "dive_speed", "dive_pace", "gate_r",
                 "perch_alt", "r_fan", "dive_angle", "reach", "perch_reach",
                 "impact_reach", "half_angle", "hold_radius", "goto_radius",
                 "gate_r_min", "gate_r_max",
                 # ⭐ 2026-08-14:契約表加了這兩項卻忘了加到讀取映射 ⇒
                 #    一律判 missing,而 missing 的訊息長得像「節點沒起來」。
                 #    表與讀取映射**必須同步**,tests 有鎖。
                 "pn_los_floor", "pn_nz_min",
                 "gate_theta_max", "gate_phi", "hold_alt", "hold_v",
                 "tecs_pitch_min", "state_max_age", "att_eas_floor",
                 "att_agl_floor", "predict_hz",
                 # ⭐ 2026-08-14 RPY3:這兩項改導引行為卻完全不在契約表內 ⇒
                 #    閘門對「開著預測器致動、卻沒開影子觀測」完全瞎。
                 "tgo_from_predictor", "tgo_predict_dt", "dive_angle_max",
                 "tgo_ring_transit",
                 "itcg_on", "itcg_gain", "itcg_a_max_g", "pn_vmax",
                 # ⭐ 2026-08-19 B1:契約表加了 mpng_on,讀取映射**必須同步** ——
                 #    否則一律判 missing,而 missing 的訊息長得像「節點沒起來」。
                 "mpng_on",
                 # ⚠️ 連**增益**一起讀回:開關有不變式守著,增益沒有。
                 #    `MPNG_K_EPS` 打錯字 ⇒ 節點吃 0.0 ⇒ 自動走建議式,而
                 #    log / 契約行 / CSV 全部看起來正常 ⇒ 掃增益的那一批會
                 #    得到「怎麼調都一樣」而完全不知道根本沒調到。
                 "mpng_k_eps", "mpng_k_s",
                 # ⭐ 2026-08-18 S1:契約表加了列,讀取映射**必須同步加** ——
                 #    否則一律判 missing,而 missing 的訊息長得像「節點沒起來」,
                 #    整批會在第一趟被自己的閘門擋掉(RPZ1 就是這個形狀)。
                 "release_at_ring", "dive_from_runup", "wait_goto_radius",
                 "perch_settle", "perch_band"],
    "wingman_node": ["formation_alt", "cruise_alt_threshold", "climbout_hold_alt"],
    "master_node": ["patrol_alt", "formation_alt"],
    "backup_node": ["formation_alt"],
}

#: 飛控 PARM(從表裡自動取,這裡只列出讀取所需的連線常數)
FC_BASE = int(os.environ.get("SAR_NODE_CMD_BASE", "5762"))
FC_STRIDE = int(os.environ.get("SAR_NODE_CMD_STRIDE", "10"))
N = int(os.environ.get("COUNT") or os.environ.get("SWARM_COUNT") or "2")
STRICT = os.environ.get("STRICT", "1") != "0"

#: ⚠️ 暫態讀取失敗的重試次數。**只放寬讀取,不放寬判定** ——
#: 重試完仍讀不到就照樣算違規。放寬判定會退化成「沒讀到所以算過」,
#: 那正是這一整層要治的病(preflight_check 檔頭第 1 條)。
#: 代價證據:RPW5 一整批 + RQD4 + RQG3/RQG4 共四趟,被一次性讀取抖動殺掉,
#: 而四次的組態全是對的。
RETRY_READS = int(os.environ.get("CONTRACT_RETRY_READS", "3"))
RETRY_SLEEP_S = float(os.environ.get("CONTRACT_RETRY_SLEEP", "1.0"))

_NUM = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=(-?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?|nan)")


# ══ 取值:log(事後)══════════════════════════════════════════════════
def from_log(path) -> dict:
    """從產物 log 的契約行重建生效值。回傳 {} 表示那一趟沒有契約行。

    ⚠️ 舊產物(2026-08-13 之前)不會有這一行 —— 那是**事實**,不是解析失敗:
       那些航次的節點層生效值從來沒有被記錄下來,事後無法重建。
    """
    text = pathlib.Path(path).read_text(encoding="utf-8", errors="replace")
    hit = None
    for line in text.splitlines():
        if CONTRACT_SENTINEL in line:
            hit = line          # 取最後一行:第二發目標會重印
    if hit is None:
        return {}
    return {k: v for k, v in _NUM.findall(hit.split(CONTRACT_SENTINEL, 1)[1])}


# ══ 取值:ros2 param(現場)════════════════════════════════════════════
def _run(cmd, timeout=20):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return p.stdout if p.returncode == 0 else None


def absent_owner_params(running_nodes) -> set:
    """回傳「擁有它的節點不在這個組態裡」的契約參數。

    ⚠️⚠️ 實例:`cruise_alt_threshold` **只在 wingman_node 宣告**,而 n=2 是
    「一長一備長」沒有僚機 ⇒ 該參數在 n=2 下**不存在**,不是讀失敗。
    (RPO1 實測:這個區別搞錯會讓整批在第一趟被誤擋。)

    ⚠️ 判定依據是 `ros2 node list` 的**實際查證**,不是「讀不到就算 N/A」——
       後者會退化成「沒讀到所以算過」,正是這一層要治的病。
    """
    bases = {n.rsplit("/", 1)[-1] for n in running_nodes}
    out = set()
    for name in {c.name for c in TABLE if c.source == SOURCE_NODE}:
        owners = {node for node, params in NODE_PARAMS.items() if name in params}
        if owners and not (owners & bases):
            out.add(name)
    return out


def parse_param_value(raw):
    """把 `ros2 param get` 的一行輸出變成契約表用的數值字串;認不得回 None。

    ⚠️⚠️ bool 一定要先攔。`ros2 param get` 對布林參數回的是
       「Boolean value is: True」—— **結尾沒有數字**,純數字正規式完全不匹配
       ⇒ 判成「讀不到」⇒ 契約中止整批,而訊息長得像「節點沒起來」。
       RPZ1(2026-08-15)整批就是這樣被自己擋掉的:參數其實好好的,
       壞的是讀取端。契約表內部用 0.0/1.0 表示 bool,這裡把兩種表示接起來。

    ⚠️ 抽成純函式是為了**測得到**。留在 `from_ros2()` 裡就得起 ROS2 才能驗,
       等於這條路永遠沒有迴歸鎖 —— 而它已經擋掉一整批了。
    """
    if raw is None:
        return None
    s = raw.strip()
    mb = re.search(r"\b(True|False)\s*$", s)
    if mb:
        return "1.0" if mb.group(1) == "True" else "0.0"
    m = re.search(r"(-?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)\s*$", s)
    return m.group(1) if m else None


def from_ros2() -> tuple[dict, list]:
    """`ros2 param get` 逐項讀回。回傳 (值, 註記)。

    ⚠️ 用 `get` 而不是 `dump`:dump 走 YAML,一旦某個參數型別不對(本專案踩過
       `.nan` 被解析成字串)整份就崩掉,而崩掉的表現是**全部讀不到**——
       那會把「一個參數壞了」誤報成「節點沒起來」。
    """
    out, notes, nodes = {}, [], []
    listing = _run(["ros2", "node", "list"])
    if listing is None:
        return out, ["`ros2 node list` 失敗 —— ROS2 環境沒 source?節點沒起來?"], nodes
    nodes = [n.strip() for n in listing.splitlines() if n.strip()]
    for full in nodes:
        base = full.rsplit("/", 1)[-1]
        for want in NODE_PARAMS.get(base, []):
            # ⚠️⚠️ **暫態讀取失敗要重試**,不可以一次讀不到就判違規。
            #    `ros2 param get` 偶發逾時/空回,而契約層把 missing 當硬違規、
            #    直接中止整批。累計代價:RPW5 一整批 + RQD4 + RQG3/RQG4,
            #    四趟報廢,而四次的組態**都是對的** —— 純粹是讀取抖動。
            #    ⚠️ 重試只放寬「讀取」,不放寬「判定」:重試完仍讀不到就照樣違規。
            #    這條界線很重要 —— 放寬判定就會退化成「沒讀到所以算過」。
            got = None
            for attempt in range(RETRY_READS):
                raw = _run(["ros2", "param", "get", full, want], timeout=10)
                got = parse_param_value(raw)
                if got is not None:
                    if attempt:
                        notes.append(f"ℹ️ {want} 第 {attempt + 1} 次才讀到"
                                     f"({full})—— 鏈路有抖動,不是組態問題")
                    break
                time.sleep(RETRY_SLEEP_S)
            if got is not None:
                # 多節點同名參數(formation_alt 在三個節點都有):不一致要吵。
                if want in out and abs(float(out[want]) - float(got)) > 1e-9:
                    notes.append(f"⚠️ {want} 跨節點不一致:{out[want]} vs {got}"
                                 f"({full})—— 編隊層與打擊層對同一個量的認知不同")
                out[want] = got
    if not out:
        notes.append("一個節點參數都讀不到 —— 節點還沒起來?")
    return out, notes, nodes


# ══ 取值:飛控 PARM(現場)═════════════════════════════════════════════
def from_fc(names) -> tuple[dict, list]:
    """從直連鏈路讀回飛控參數。**讀回**,不是相信我們送出的值。"""
    try:
        from pymavlink import mavutil
    except ImportError:
        return {}, ["pymavlink 不可用 ⇒ 飛控 PARM 無法讀回"]
    out, notes = {}, []
    for uav in range(1, N + 1):
        port = FC_BASE + FC_STRIDE * (uav - 1)
        try:
            c = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}")
            c.wait_heartbeat(timeout=8)
        except Exception as e:                                  # noqa: BLE001
            notes.append(f"UAV{uav} 連線失敗 {port}: {e!r}")
            continue
        for name in names:
            try:
                c.mav.param_request_read_send(c.target_system, 1, name.encode(), -1)
                m = c.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
            except Exception:                                   # noqa: BLE001
                continue
            if m is None:
                continue
            got = float(m.param_value)
            key = m.param_id.strip("\x00")
            if key in out and abs(float(out[key]) - got) > 1e-6:
                notes.append(f"⚠️ {key} 逐機不一致:{out[key]} vs {got}(UAV{uav})")
            out[key] = got
        try:
            c.close()
        except Exception:                                       # noqa: BLE001
            pass
    return out, notes


# ══ 主流程 ════════════════════════════════════════════════════════════
def gather(args):
    values, notes, srcs, na = {}, [], {}, set()

    if args.from_log:
        got = from_log(args.from_log)
        if not got:
            notes.append(f"{args.from_log} 沒有契約行 —— 該航次跑的是加入契約行"
                         f"之前的節點版本,節點層生效值**無法事後重建**")
        values.update(got)
        srcs.update({k: "log" for k in got})
    else:
        got, n, nodes = from_ros2()
        values.update(got); notes += n
        srcs.update({k: "ros2" for k in got})
        # ⚠️ 只有在**真的問到節點清單**時才敢判 N/A。清單拿不到就一律照常檢查,
        #    否則「ros2 掛了」會被誤判成「這些參數本來就不適用」而全部放行。
        if nodes:
            na = absent_owner_params(nodes)

        fc_names = [c.name for c in TABLE if c.source == SOURCE_FC]
        got, n = from_fc(fc_names)
        values.update(got); notes += n
        srcs.update({k: "fc" for k in got})

    # shell 來源一律讀 env(--from-log 時也讀,因為批次腳本的變數不進節點 log)
    for c in TABLE:
        if c.source == SOURCE_SHELL and os.environ.get(c.name):
            values[c.name] = os.environ[c.name]
            srcs[c.name] = "env"
    return values, notes, srcs, na


def _derived(values):
    """算得出來但**不阻擋**的量,寫進快照供事後判讀。

    ⚠️ 需求俯角刻意留在這裡而不是 INVARIANTS:TWO1 與 RPK/RPL 的
       γ_nominal 都是 32.9°,一個 2/2 一個 2/6 ⇒ 零鑑別力,做成門檻只會
       誤擋已知可行的組態。理由完整寫在 contract_table.py 對應段落。
       但事後回頭比對「那一趟的 γ 是多少」時,這個數字非常有用。
    """
    try:
        from sar_swarm.contract_table import required_gamma_deg
    except ImportError:
        from contract_table import required_gamma_deg
    out = {}
    perch, gate = _num(values.get("perch_alt")), _num(values.get("gate_r"))
    if perch is not None and gate is not None:
        out["gamma_req_worst_deg"] = round(required_gamma_deg(perch, gate), 2)
        out["gate_over_ring"] = None
    ang = _num(values.get("dive_angle"))
    if perch is not None and gate is not None and ang and 0.0 < ang < 89.9:
        import math as _m
        ring = perch / _m.tan(_m.radians(ang))
        out["ring_radius_m"] = round(ring)
        out["gate_over_ring"] = round(gate / ring, 3)
    return out


def _num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser(description="生效值契約驗證")
    ap.add_argument("--from-log", metavar="LOG",
                    help="事後模式:從產物 log 的契約行重建生效值")
    ap.add_argument("--snapshot", metavar="PATH",
                    help="把生效值快照寫成 JSON(與 CSV/log/geom 同級的產物)")
    ap.add_argument("--json", action="store_true", help="機器可讀輸出")
    ap.add_argument("--require", metavar="NAMES",
                    help="只檢查這些參數(逗號分隔);預設全表")
    args = ap.parse_args()

    values, notes, srcs, na = gather(args)
    require = args.require.split(",") if args.require else None
    rep = evaluate(values, require=require, not_applicable=na)

    if args.snapshot:
        snap = {"values": values, "sources": srcs,
                "not_applicable": sorted(na),
                "derived": _derived(values),
                "violations": [v.__dict__ for v in rep.violations],
                "ok": rep.ok}
        p = pathlib.Path(args.snapshot)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(snap, ensure_ascii=False, indent=1), encoding="utf-8")

    if args.json:
        print(json.dumps({"values": values, "sources": srcs, "notes": notes,
                          "violations": [v.__dict__ for v in rep.violations],
                          "ok": rep.ok}, ensure_ascii=False, indent=1))
    else:
        print(rep.text())
        for nt in notes:
            print(f"  ⚠️ {nt}")
        if rep.violations:
            hint = rerun_hint(rep.violations)
            if hint:
                print("\n" + hint)

    if rep.ok:
        return 0
    if not STRICT:
        # ⚠️ 降級必須大聲。靜默降級正是這一整類 bug 的成因。
        print("\n⚠️⚠️ STRICT=0:**已知有違規仍繼續執行** —— "
              "這一趟的數字不可用來下結論。", file=sys.stderr)
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
