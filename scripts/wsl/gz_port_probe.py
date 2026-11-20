#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gz_port_probe.py — VTOL 群機的埠位前置檢查(**真的送封包**,不是看 ss)。

═══════════════════════════════════════════════════════════════════════════
為什麼不能只看 `ss -lunp`
═══════════════════════════════════════════════════════════════════════════

`ArduPilotPlugin` 的 FDM socket 是 `SocketUDP sock = SocketUDP(true, true)`
(ardupilot_gazebo/src/ArduPilotPlugin.cc:227),第二個 true 走
`SocketUDP::set_reuseaddress()`,也就是 `setsockopt(SO_REUSEADDR)`
(ardupilot_gazebo/src/SocketUDP.cc:71-75)。

Linux 對 UDP 的規則是:**只要新舊 socket 都設了 SO_REUSEADDR,重複 bind 同一個
`addr:port` 就會成功**,而封包只送給**最後 bind 的那一個**。本機 2026-09-09 實測
(P=19003,兩個 SO_REUSEADDR socket 都 bind 127.0.0.1:19003):

    squatter bind 127.0.0.1 REUSE=1 -> ok
      probe reuse=True  addr=127.0.0.1 -> BIND OK      ← 第二個也成功
      recv first(squatter)  -> TIMEOUT (zero packets)  ← 先 bind 的收到 **0** 封包
      recv second(probe)    -> b'PROBE'                ← 後 bind 的把封包全吃走

整段過程 **沒有任何錯誤、沒有任何 log**。plugin 那邊的 `gzerr << "failed to bind"`
(ArduPilotPlugin.cc:1289-1293)也**永遠不會印**,因為 bind 根本沒失敗。

⇒ `ss -lunp` 會誠實地告訴你「這個埠有人 bind」,但它**分辨不出**
   「那是我剛起的 gz」還是「別人的 gz 在偷我的封包」——兩者長得一模一樣。
   而我們的每一層既有檢查(gz 的 log、sim_time 前進、CPU 使用率)在這個失效上
   **全部是綠燈**。這正是本專案定義的那類「驗證器壞掉時表徵一律是綠燈」。

═══════════════════════════════════════════════════════════════════════════
本檔採用的判別法(三層,只有第一層有投票權)
═══════════════════════════════════════════════════════════════════════════

**Layer A —— 獨占 bind(唯一的判準)**
    開一個**不設 SO_REUSEADDR** 的 socket 去 bind 目標埠。
    Linux 的規則是「衝突的 socket 們 **和** 新 socket **全部**都要有 SO_REUSEADDR
    才放行」,所以只要我這一邊不設,任何既有 binder(不管它有沒有設)都會讓我
    拿到 `EADDRINUSE`。實測(同上那一輪):

        probe reuse=False addr=127.0.0.1 -> EADDRINUSE   ← 抓得到 REUSE 的 squatter
        probe reuse=False addr=0.0.0.0   -> EADDRINUSE   ← 萬用位址也抓得到

    ⇒ **EADDRINUSE = 髒**。這是唯一能穿透 SO_REUSEADDR 偽裝的檢查。

**Layer B —— 送探測封包(證明這條路真的通)**
    Layer A 的 socket **不關掉**,直接用它收自己送出的一個含亂數 nonce 的封包。
    這證明的是「送到 127.0.0.1:P 的 datagram 真的會落到 bind 在 P 的人手上」——
    排除掉 nftables/防火牆規則、loopback 異常、位址族(v4/v6)假設錯誤這幾種
    「bind 得到但收不到」的情形。

    ⚠️⚠️ **為什麼用同一個 socket、而不是照 plugin 的方式另開一個 SO_REUSEADDR 的?**
    因為那樣會做出一個**必定綠燈的假驗證器**:squatter 先 bind、我後 bind,
    依照上面的實測「後 bind 的收到全部封包」⇒ **我一定收得到,一定報乾淨**。
    這個假綠燈在 `--self-test` 的 A2 臂被實際演示出來(見下面)。
    同一個 socket 還順帶消掉 Layer A 與 Layer B 之間的 TOCTOU 窗口。

**Layer C —— `ss` 只負責「指認兇手」,不負責判決**
    Layer A 報紅之後才去問 `ss`「是誰」,把 pid/cmdline 印出來給人看。
    ⚠️ `ss -lunp` 的欄位:第 4 欄才是 `Local Address:Port`,第 5 欄是 Peer。
       本次撰寫時我自己先寫成 `$5` ⇒ **9002/9012/9022/9032 全部沒被列出來**,
       看起來像「埠是乾淨的」。連「用 ss 指認」都有一個安靜的失敗模式。
    ⚠️ 非本人的行程,`users:(...)` 會是空的。**「查不到行程名」不等於「沒人佔用」**。

═══════════════════════════════════════════════════════════════════════════
埠位表(單一真相來源 —— 不要在 bash 那邊重算一次)
═══════════════════════════════════════════════════════════════════════════

事實:ArduPilot 的埠是從 **instance 號** 推出來的,不是我們給幾號就是幾號:
  * `SITL_cmdline.cpp:258-259`  SIM_IN_PORT=9003 / SIM_OUT_PORT=9002 / IRLOCK_PORT=9005
  * `SITL_cmdline.cpp:435-452`  `-I` 的處理:**凡是還等於預設值的**,一律 `+= instance*10`
    (base_port / rcin / fg_view / sim_in / sim_out / irlock 六個都是)
  * `SITL_State.h:29-30`        `_serial_path[0] = "tcp:0:wait"`
  * `UARTDriver.cpp:368-372`    `tcp:N` 的 N ≤ 1000 時是 **base_port + N**,>1000 才是字面埠

  ⇒ SDF 的 `<fdm_port_in>` **必須等於** 該架 SITL 的 `9002 + 10*instance`。
     對不上的症狀是 SITL 一直印 `No JSON sensor message received`,
     **和撞名/被偷封包的症狀一模一樣**,肉眼分不出來。

⚠️⚠️ `--base-port` 的陷阱:那個 `if (_base_port == BASE_PORT)` 守衛是**比值**的。
   顯式傳 `--base-port 5760` 之後再給 `-I10`,守衛成立 ⇒ 還是會被加成 5860。
   所以我們給的 TCP 基準值刻意**永遠不等於 5760**(預設 6360 起)。

⚠️ `9003 + 10*i`(SIM_IN_PORT)**不要檢查**:`SIM_JSON::set_interface_ports()`
   (ardupilot/libraries/SITL/SIM_JSON.cpp:88-99)只做了 set_blocking + reuseaddress,
   **從頭到尾沒有 bind**,來源埠是 kernel 給的臨時埠。實測 `ss -lunp` 上
   9003/9013/… 一個都不存在。把它列進必檢清單只會製造假紅燈。

═══════════════════════════════════════════════════════════════════════════
實測紀錄(2026-09-09)
═══════════════════════════════════════════════════════════════════════════

`--self-test` **6/6 通過**。除此之外還在**真實的**髒埠段上對抗過:當時同一台機器
有另一個 session 的四機 Gazebo 在跑,

    python3 gz_port_probe.py --i0 0 -n 1 --tcp-base 5761

Layer A 對 UDP 9002 給出 EADDRINUSE,Layer C 指名兇手:

    ss: 127.0.0.1:9002  state=UNCONN  users:(("ruby",pid=153122,fd=31))

—— 那正是 `gz sim server`(ruby wrapper)用 SO_REUSEADDR 佔著的 FDM 埠。
同一輪對乾淨的 I=10..13 段(24 個必檢埠)全綠。

═══════════════════════════════════════════════════════════════════════════
用法
═══════════════════════════════════════════════════════════════════════════

    python3 gz_port_probe.py --i0 10 -n 4                 # 檢查 I=10..13
    python3 gz_port_probe.py --instances 10,11,14         # 指定 instance 清單
    python3 gz_port_probe.py --i0 10 -n 4 --print-map     # 印埠位表給 shell eval
    python3 gz_port_probe.py --i0 10 -n 4 --json          # 機器可讀結果
    python3 gz_port_probe.py --self-test                  # ⚠️ 驗「這支檢查真的會響」

離開碼:
    0  乾淨(所有必檢埠都通過 Layer A + Layer B)
    1  髒(至少一個埠被佔,或封包送不到)
    2  用法/內部錯誤
    3  自檢失敗(= 這支檢查本身壞了,**不要**相信它任何一次綠燈)
"""
from __future__ import annotations

import argparse
import errno
import json
import os
import random
import socket
import subprocess
import sys

# ── 這些常數全部來自 ArduPilot 原始碼,不是猜的 ────────────────────────────
SIM_OUT_PORT = 9002   # SITL_cmdline.cpp:259 — gz 的 ArduPilotPlugin bind 這個
IRLOCK_PORT = 9005    # SITL_cmdline.cpp:260 — SITL 自己 bind
RCIN_PORT = 5501      # SITL_cmdline.cpp:252 — SITL 自己 bind
FG_VIEW_PORT = 5503   # SITL_cmdline.cpp:253 — 只有 --enable-fgview 才 bind
INSTANCE_STRIDE = 10  # SITL_cmdline.cpp:435-452 全部都是 instance*10

DEFAULT_I0 = 10       # 事實【13】:0..3 被別的 session 的四機 Gazebo 佔著
DEFAULT_N = 4
DEFAULT_TCP_BASE = 6360

PROBE_TIMEOUT_S = 0.40


# ═════════════════════════════════════════════════════════════════════════
# 埠位表
# ═════════════════════════════════════════════════════════════════════════
def port_map(instance: int, tcp_base: int, fgview: bool = False) -> list:
    """回傳一架的必檢埠位。

    ⚠️ `tcp_base` 是**這一架的**基準值(已經含了機序偏移),不是整批的起點。
       ArduPilot 那邊我們是用 `--base-port <tcp_base>` 顯式指定的,所以
       `-I` 的 `+= instance*10` 不會再作用(值 ≠ 5760,守衛不成立)。

    回傳 [(proto, port, role, fatal)],fatal=False 的只警告不擋。
    """
    off = INSTANCE_STRIDE * instance
    rows = [
        ("udp", SIM_OUT_PORT + off, "fdm_in(gz plugin bind;SITL 送 servo 到這)", True),
        ("udp", IRLOCK_PORT + off, "irlock(SITL bind)", True),
        ("udp", RCIN_PORT + off, "rcin(SITL bind)", True),
        # SERIAL0/1/2 = _serial_path[0..2] = "tcp:0:wait" / "tcp:2" / "tcp:3"
        # ⇒ base_port+0 / +2 / +3(UARTDriver.cpp:368-372)
        ("tcp", tcp_base + 0, "SERIAL0(GCS / coordinator)", True),
        ("tcp", tcp_base + 2, "SERIAL1", True),
        ("tcp", tcp_base + 3, "SERIAL2", True),
    ]
    if fgview:
        # 只有加 --enable-fgview 才會 bind;預設不檢查,免得製造假紅燈。
        rows.append(("udp", FG_VIEW_PORT + off, "fgview(僅 --enable-fgview)", False))
    # ⚠️ **這張表刻意不是完整的**,不要把「全綠」讀成「這一架用到的埠全都乾淨」:
    #   _serial_path[5..8] = "tcp:5".."tcp:8"(SITL_State.h:29-37)⇒ base_port+5..+8
    #   也可能被 bind,取決於該機種的 SERIALn_PROTOCOL 有沒有啟用。
    #   不列進來的理由:啟不啟用是機種/參數決定的,列成必檢會製造假紅燈;
    #   而它們落在**本架自己的** base_port+0..+9 區塊內(每架間距 10),
    #   不會跨機互撞,被外人佔走時 UARTDriver.cpp:396-402 會 exit(1)——
    #   那是**大聲**的失敗,不是靜默的,所以留給 gz_vtol_up.sh 的 G0 去接。
    return rows


BASE_PORT_DEFAULT = 5760   # SITL_cmdline.cpp:251 const int BASE_PORT = 5760


def build_plan(instances: list, tcp_base: int, fgview: bool = False) -> list:
    """整批的計畫。每架的 TCP 基準 = tcp_base + 10*機序(不是 10*instance)。

    為什麼用機序而不是 instance:instance 可能不連號(--instances 10,11,14),
    但 TCP 基準只要彼此不重疊即可,用機序才不會在跳號時挖出無用的空洞。
    """
    plan = []
    for k, inst in enumerate(instances):
        base = tcp_base + INSTANCE_STRIDE * k
        # ⚠️⚠️ 守衛要下在**每一架**的基準上,不是只看第一架。
        #    main() 那道 `--tcp-base == 5760` 只擋得掉 k=0;`--tcp-base 5700 -n 10`
        #    的第 7 架剛好是 5760,而 SITL_cmdline.cpp:435 的守衛是**比值**的
        #    ⇒ 那一架的 `-I` 會再把 base_port 加成 5760+10*instance,
        #      於是 probe 檢查 5760、SITL 其實 listen 在別的埠。
        #    (gz_vtol_up.sh 的 G2 讀回抓得到,但那是起飛之後才知道;
        #     這裡擋掉才不用先弄髒別人的埠段。)
        if base == BASE_PORT_DEFAULT:
            # ⚠️ 離開碼用 2(用法/內部錯誤),**不可以**讓 SystemExit(str) 走預設的 1
            #    —— 1 在本檔的契約裡是「埠髒」,呼叫端會去找不存在的佔用者。
            print("✗ 第 %d 架的 SITL TCP 基準算出來是 %d,正好等於韌體預設 BASE_PORT\n"
                "  ⇒ SITL_cmdline.cpp:435 的 `if (_base_port == BASE_PORT)` 會成立,\n"
                "     `-I%d` 之後它會變成 %d,而不是我們檢查的 %d。換一個 --tcp-base。"
                % (k, base, inst, base + INSTANCE_STRIDE * inst, base),
                file=sys.stderr)
            raise SystemExit(2)
        rows = port_map(inst, base, fgview)
        for proto, port, role, _fatal in rows:
            # instance 太大時 9002+10*I 會超出 16 bit,socket.bind 丟的是
            # OverflowError 而**不是** OSError ⇒ 直接 traceback,看起來像 probe 壞了。
            if not (1 <= port <= 65535):
                print("✗ 第 %d 架(instance=%d)推出來的 %s 埠 %d 超出 1-65535 —— "
                      "instance 或 --tcp-base 給得太大" % (k, inst, proto, port),
                      file=sys.stderr)
                raise SystemExit(2)
        plan.append({"k": k, "instance": inst, "tcp_base": base, "ports": rows})
    return plan


# ═════════════════════════════════════════════════════════════════════════
# Layer C — 指認兇手(不參與判決)
# ═════════════════════════════════════════════════════════════════════════
def _ss_rows(proto: str) -> list:
    """跑 ss 撈出 listening/bound 的清單。回傳 [(local, state, procs)]。"""
    flag = "-lunp" if proto == "udp" else "-ltnp"
    try:
        out = subprocess.run(["ss", flag], capture_output=True, text=True,
                             timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    rows = []
    for line in out.splitlines():
        parts = line.split()
        # ⚠️ 欄位:0=State 1=Recv-Q 2=Send-Q 3=Local 4=Peer 5=Process
        #    我第一版寫成用第 4 欄(Peer)過濾,結果被佔用的 9002/9012/9022/9032
        #    一個都沒列出來 —— 連指認工具都有安靜的失敗模式。
        if len(parts) < 5 or parts[0] in ("State", "Netid"):
            continue
        rows.append((parts[3], parts[0], " ".join(parts[5:]) if len(parts) > 5 else ""))
    return rows


def blame(proto: str, port: int) -> list:
    """把佔用這個埠的人找出來,盡量講清楚。找不到不代表沒人(見檔頭說明)。"""
    hits = []
    suffix = ":%d" % port
    for local, state, procs in _ss_rows(proto):
        if not local.endswith(suffix):
            continue
        hits.append("ss: %-22s state=%-11s %s" % (local, state, procs or "(行程不明)"))
    # ss 拿不到行程名(不同使用者)時再退一步問 fuser。
    if not any("pid=" in h for h in hits):
        try:
            r = subprocess.run(["fuser", "-v", "-n", proto, str(port)],
                               capture_output=True, text=True, timeout=5)
            extra = (r.stderr or "").strip() or (r.stdout or "").strip()
            if extra:
                hits.append("fuser: " + extra.replace("\n", " | "))
        except (OSError, subprocess.SubprocessError):
            pass
    if not hits:
        hits.append("(ss/fuser 都指認不出來 —— 可能是別的使用者或別的 netns。"
                    "⚠️ 這**不是**『沒人佔用』的證據,判決以 Layer A 為準。)")
    return hits


# ═════════════════════════════════════════════════════════════════════════
# Layer A + B — 判決
# ═════════════════════════════════════════════════════════════════════════
def _bind_exclusive_udp(addr: str, port: int):
    """不設 SO_REUSEADDR 去 bind。回傳 (sock, None) 或 (None, errno)。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind((addr, port))
        return s, None
    except OSError as e:
        s.close()
        return None, e.errno


def check_udp(port: int) -> dict:
    """UDP 的 Layer A + Layer B。"""
    res = {"proto": "udp", "port": port, "ok": False, "level": "FAIL",
           "layer_a": "", "layer_b": "", "addr": None}

    # Layer A:先試萬用位址(最敏感 —— 實測它和 127.0.0.1 的 binder 也會衝突,
    # 而且和 IPv6 `::` 的 dual-stack binder 也會衝突)。
    sock, err = _bind_exclusive_udp("0.0.0.0", port)
    addr = "0.0.0.0"
    if sock is None and err == errno.EADDRINUSE:
        # 有可能佔用者只 bind 在某張實體網卡上(例如 172.17.x.x),那對我們的
        # loopback FDM 其實無害。退一步只 bind 127.0.0.1 分辨這件事。
        sock, err2 = _bind_exclusive_udp("127.0.0.1", port)
        addr = "127.0.0.1"
        if sock is not None:
            res.update(ok=True, level="WARN", addr=addr,
                       layer_a="0.0.0.0 被佔(EADDRINUSE)但 127.0.0.1 可獨占 —— "
                               "佔用者綁在別張網卡上,對 loopback FDM 無害")
        else:
            res["layer_a"] = ("EADDRINUSE:0.0.0.0 與 127.0.0.1 都無法獨占 bind "
                              "⇒ 真的有人在用(即使對方設了 SO_REUSEADDR 也躲不掉)")
            res["blame"] = blame("udp", port)
            return res
    elif sock is None:
        res["layer_a"] = "bind 失敗:%s(errno=%s)" % (
            errno.errorcode.get(err, err), err)
        res["blame"] = blame("udp", port)
        return res
    else:
        res.update(addr=addr, layer_a="0.0.0.0 獨占 bind 成功(無 SO_REUSEADDR)")

    # Layer B:用**同一個** socket 收自己送的封包。
    # 同一個 socket ⇒ 沒有 TOCTOU 窗口,也不會複製出 plugin 那種「後 bind 者
    # 必勝」的假綠燈(見檔頭與 --self-test 的 A2 臂)。
    try:
        nonce = ("GZVTOL-%08x" % random.getrandbits(32)).encode()
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            tx.sendto(nonce, ("127.0.0.1", port))
            sock.settimeout(PROBE_TIMEOUT_S)
            got, src = sock.recvfrom(256)
        finally:
            tx.close()
        if got != nonce:
            res.update(ok=False, level="FAIL",
                       layer_b="收到的不是自己的 nonce(收到 %r)—— 這條路上有別人"
                               % got[:32])
            res["blame"] = blame("udp", port)
            return res
        res["layer_b"] = "nonce 從 %s:%d 送出後原樣收回" % (src[0], src[1])
    except socket.timeout:
        res.update(ok=False, level="FAIL",
                   layer_b="送出 nonce 後 %.2fs 內收不到 —— bind 得到但封包進不來"
                           "(防火牆 / netns / loopback 異常)" % PROBE_TIMEOUT_S)
        res["blame"] = blame("udp", port)
        return res
    except OSError as e:
        res.update(ok=False, level="FAIL", layer_b="送/收 例外:%s" % e)
        return res
    finally:
        sock.close()

    if res["level"] != "WARN":
        res["level"] = "OK"
    res["ok"] = True
    return res


def check_tcp(port: int) -> dict:
    """TCP 的 Layer A(bind + listen)。

    ⚠️ TCP 這邊 SO_REUSEADDR **不會**讓兩個 LISTEN 並存(那是 SO_REUSEPORT 的事),
       所以獨占 bind 本來就抓得到 listener。但它會多抓到一種**假紅燈**:
       上一輪留下的 TIME_WAIT。ArduPilot 的 listen socket 有設 SO_REUSEADDR
       (UARTDriver.cpp:387),所以那種情況它其實 bind 得起來。
       ⇒ bind 失敗時再問 ss:有 LISTEN 才判髒,只有 TIME_WAIT 就降成 WARN。
    """
    res = {"proto": "tcp", "port": port, "ok": False, "level": "FAIL",
           "layer_a": "", "layer_b": "(TCP 不做封包層測試:listen 本身就是通道證據)",
           "addr": "0.0.0.0"}
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("0.0.0.0", port))
        s.listen(1)
        res.update(ok=True, level="OK",
                   layer_a="0.0.0.0 獨占 bind + listen 成功(無 SO_REUSEADDR)")
    except OSError as e:
        states = [st for local, st, _ in _ss_rows("tcp")
                  if local.endswith(":%d" % port)]
        if e.errno == errno.EADDRINUSE and "LISTEN" not in states:
            res.update(ok=True, level="WARN",
                       layer_a="EADDRINUSE 但 ss 看不到 LISTEN(可能是殘留 TIME_WAIT)"
                               " —— ArduPilot 的 listen socket 有 SO_REUSEADDR"
                               "(UARTDriver.cpp:387)所以它起得來")
        else:
            res["layer_a"] = "bind/listen 失敗:%s(errno=%s)" % (
                errno.errorcode.get(e.errno, e.errno), e.errno)
            res["blame"] = blame("tcp", port)
    finally:
        s.close()
    return res


def check_one(proto: str, port: int) -> dict:
    return check_udp(port) if proto == "udp" else check_tcp(port)


# ═════════════════════════════════════════════════════════════════════════
# 自檢:證明「這支檢查真的會響」
# ═════════════════════════════════════════════════════════════════════════
def self_test(base_port: int = 19700) -> int:
    """⚠️⚠️⚠️ 本專案的教訓:驗證器本身壞掉時,表徵一律是綠燈。

    所以每一趟開跑前都跑這一段,在**明知被佔**的埠上必須報紅;報不出來就
    整支不可信,直接擋掉航次。

    六個臂:
      A1  UDP 被 SO_REUSEADDR squatter 佔住   ⇒ 必須 FAIL
      A2  同一個 squatter,但只跑「plugin 式」的送收測試 ⇒ **必須綠燈**
          (刻意演示那個假綠燈,證明 Layer A 才是有投票權的那一層;
           如果 A2 也紅了,代表我對 kernel 語意的理解錯了 ⇒ 一樣擋掉)
      A3  乾淨的 UDP 埠                       ⇒ 必須 OK(否則是卡死紅燈,一樣沒用)
      A4  Layer C 在 A1 的埠上要指認得出兇手(拿不到行程名只警告,不擋)
      A5  TCP 被 LISTEN 佔住                  ⇒ 必須 FAIL
      A6  乾淨的 TCP 埠                       ⇒ 必須 OK
    """
    arms, bad = [], 0

    def arm(name, want, got, note=""):
        nonlocal bad
        ok = (want == got)
        if not ok:
            bad += 1
        arms.append((name, want, got, ok, note))
        print("  [%s] %-58s 期望=%-4s 實得=%-4s%s"
              % ("✔" if ok else "✗", name, want, got,
                 ("  " + note) if note else ""))

    print("[probe] ── 自檢:證明這支檢查會響 ──────────────────────────────")

    # ---- A1 / A2 / A4:UDP,被 SO_REUSEADDR squatter 佔住 -----------------
    p = base_port
    sq = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sq.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sq.bind(("127.0.0.1", p))   # 與 ArduPilotPlugin.cc:227 的 SocketUDP(true,true) 同款
    except OSError as e:
        print("[probe] ✗ 自檢佈置失敗:連 squatter 都 bind 不上 %d(%s)" % (p, e),
              file=sys.stderr)
        return 3

    r1 = check_udp(p)
    arm("A1 UDP 被 REUSEADDR 佔住 → Layer A 報紅", "FAIL", r1["level"],
        r1["layer_a"][:70])

    # A2:刻意造一個「只做送收、且照 plugin 方式 bind」的假驗證器。
    #     按照實測語意(後 bind 者收全部),它一定收得到 ⇒ 一定綠燈。
    fake = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fake.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    a2 = "FAIL"
    try:
        fake.bind(("127.0.0.1", p))      # 第二個 binder —— 依實測會把封包全偷走
        n = b"FAKE-VERIFIER"
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tx.sendto(n, ("127.0.0.1", p))
        tx.close()
        fake.settimeout(PROBE_TIMEOUT_S)
        a2 = "OK" if fake.recvfrom(64)[0] == n else "FAIL"
    except (OSError, socket.timeout):
        a2 = "FAIL"
    finally:
        fake.close()
    arm("A2 只送收的假驗證器 → 必須給出假綠燈", "OK", a2,
        "← 這就是為什麼 Layer B 不能單獨當判準")

    hits = blame("udp", p)
    arm("A4 Layer C 指認得出兇手", "OK",
        "OK" if any("pid=" in h or "fuser" in h for h in hits) else "WARN",
        hits[0][:70])
    sq.close()

    # ---- A3:乾淨的 UDP 埠 -----------------------------------------------
    r3 = check_udp(base_port + 1)
    arm("A3 乾淨 UDP 埠 → 必須綠燈(不能是卡死紅燈)", "OK", r3["level"],
        (r3["layer_b"] or r3["layer_a"])[:70])

    # ---- A5 / A6:TCP -----------------------------------------------------
    p5 = base_port + 2
    lst = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    lst.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        lst.bind(("0.0.0.0", p5))
        lst.listen(1)
        arm("A5 TCP 被 LISTEN 佔住 → 報紅", "FAIL", check_tcp(p5)["level"])
    except OSError as e:
        print("[probe] ✗ 自檢佈置失敗:TCP squatter bind 不上 %d(%s)" % (p5, e),
              file=sys.stderr)
        return 3
    finally:
        lst.close()
    arm("A6 乾淨 TCP 埠 → 必須綠燈", "OK", check_tcp(base_port + 3)["level"])

    print("[probe] ── 自檢結果:%d/%d 臂通過 ──" % (len(arms) - bad, len(arms)))
    if bad:
        print("[probe] ✗✗✗ 自檢失敗 —— **不要相信這支檢查的任何一次綠燈**。\n"
              "        A2 若失敗,代表 kernel 的 SO_REUSEADDR 語意與檔頭記載不符,\n"
              "        整份判別法的前提要重驗(重跑檔頭那段實測)。", file=sys.stderr)
        return 3
    return 0


# ═════════════════════════════════════════════════════════════════════════
# CLI
# ═════════════════════════════════════════════════════════════════════════
def parse_instances(args) -> list:
    if args.instances:
        try:
            xs = [int(x) for x in args.instances.replace(" ", "").split(",") if x]
        except ValueError:
            print("✗ --instances 只能是逗號分隔的整數", file=sys.stderr)
            raise SystemExit(2)
        if len(set(xs)) != len(xs):
            print("✗ --instances 有重複的 instance 號 —— 埠會撞在一起", file=sys.stderr)
            raise SystemExit(2)
        return xs
    return [args.i0 + k for k in range(args.count)]


def main() -> int:
    ap = argparse.ArgumentParser(
        description="VTOL 群機埠位前置檢查(真的送封包,不是只看 ss)",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--i0", type=int, default=int(os.environ.get("I0", DEFAULT_I0)),
                    help="起始 SITL instance 號(預設 %d;0..3 被別人佔著)" % DEFAULT_I0)
    ap.add_argument("-n", "--count", type=int,
                    default=int(os.environ.get("N", DEFAULT_N)), help="機數")
    ap.add_argument("--instances", default=os.environ.get("INSTANCES", ""),
                    help="直接給 instance 清單(逗號分隔),會蓋掉 --i0/-n")
    ap.add_argument("--tcp-base", type=int,
                    default=int(os.environ.get("TCP_BASE", DEFAULT_TCP_BASE)),
                    help="第一架的 SITL TCP 基準埠(預設 %d)" % DEFAULT_TCP_BASE)
    ap.add_argument("--fgview", action="store_true",
                    help="連 fgview 埠(5503+10i)一起檢查(只在 --enable-fgview 時需要)")
    ap.add_argument("--print-map", action="store_true",
                    help="只印埠位表(KEY=VALUE,給 shell eval),不做檢查")
    ap.add_argument("--json", action="store_true", help="結果輸出 JSON")
    ap.add_argument("--self-test", action="store_true",
                    help="⚠️ 驗這支檢查本身會不會響(在明知被佔的埠上必須報紅)")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    if args.count < 1:
        print("✗ -n 至少 1", file=sys.stderr)
        return 2
    if args.tcp_base == 5760:
        # SITL_cmdline.cpp:435 的守衛是比值的:--base-port 5760 之後 -I 還是會
        # 再加 instance*10,我們就再也不知道它到底 listen 在哪。
        print("✗ --tcp-base 不可以是 5760(ArduPilot 會把它當成『沒指定』"
              "而再加上 instance*10,見 SITL_cmdline.cpp:435-436)", file=sys.stderr)
        return 2

    instances = parse_instances(args)
    plan = build_plan(instances, args.tcp_base, args.fgview)

    if args.print_map:
        # 給 gz_vtol_up.sh eval 用 —— 埠算術只有這一份,bash 那邊不重算。
        print("VTOL_N=%d" % len(plan))
        print("VTOL_INSTANCES=\"%s\"" % " ".join(str(p["instance"]) for p in plan))
        print("VTOL_FDM_PORTS=\"%s\""
              % " ".join(str(SIM_OUT_PORT + INSTANCE_STRIDE * p["instance"])
                         for p in plan))
        print("VTOL_TCP_BASES=\"%s\"" % " ".join(str(p["tcp_base"]) for p in plan))
        print("VTOL_IRLOCK_PORTS=\"%s\""
              % " ".join(str(IRLOCK_PORT + INSTANCE_STRIDE * p["instance"])
                         for p in plan))
        print("VTOL_RCIN_PORTS=\"%s\""
              % " ".join(str(RCIN_PORT + INSTANCE_STRIDE * p["instance"])
                         for p in plan))
        return 0

    results, n_fail, n_warn = [], 0, 0
    print("[probe] 檢查 %d 架:instance=%s,TCP 基準 %d 起"
          % (len(plan), ",".join(str(i) for i in instances), args.tcp_base))
    for entry in plan:
        print("[probe] ── uav%d (I=%d, tcp_base=%d) ──"
              % (entry["k"], entry["instance"], entry["tcp_base"]))
        for proto, port, role, fatal in entry["ports"]:
            r = check_one(proto, port)
            r.update(role=role, fatal=fatal, instance=entry["instance"], k=entry["k"])
            results.append(r)
            mark = {"OK": "✔", "WARN": "!", "FAIL": "✗"}[r["level"]]
            print("  [%s] %-4s %-6d %-46s %s"
                  % (mark, proto.upper(), port, role,
                     r["layer_b"] if r["level"] == "OK" else r["layer_a"]))
            if r["level"] == "FAIL":
                for line in r.get("blame", []):
                    print("        └─ %s" % line)
                if fatal:
                    n_fail += 1
            elif r["level"] == "WARN":
                n_warn += 1

    if args.json:
        print(json.dumps({"instances": instances, "tcp_base": args.tcp_base,
                          "fail": n_fail, "warn": n_warn, "results": results},
                         ensure_ascii=False, indent=2))

    print("[probe] ── 總計:%d 個必檢埠,%d 髒,%d 警告 ──"
          % (sum(1 for r in results if r["fatal"]), n_fail, n_warn))
    if n_fail:
        print("[probe] ✗ 有埠不乾淨。**不要**直接換一個埠段就跑 ——\n"
              "        先確認那是不是別人正在跑的航次(上面的 ss/fuser 指認),\n"
              "        再用 --i0 / --tcp-base 挪開。", file=sys.stderr)
        return 1
    print("[probe] ✔ 全部乾淨。\n"
          "        ⚠️ 這是**當下**的檢查,不是租約:probe 結束到 gz/SITL 真的 bind\n"
          "           之間仍有窗口。真正的閉環是啟動之後的逐機值層級讀回\n"
          "           (gz_vtol_up.sh 會比對 SITL 印出的 "
          "`JSON control interface set to 127.0.0.1:<port>`)。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
