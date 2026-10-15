# -*- coding: utf-8 -*-
"""編隊巡航段的逐拍記錄器 —— master_node 與 backup_node 共用。

═══════════════════════════════════════════════════════════════════════════
 為什麼要有這個檔
═══════════════════════════════════════════════════════════════════════════
2026-08-24:編隊巡航段**完全沒有落盤資料**。

  * `tot_node` 的 CSV 從「收到目標」才開始 ⇒ 編隊段整個在它之前。
  * 唯一的指令記錄器 `_cmd_log_open` 在 `wingman_node.py:424`,
    而 **n=2 不生 wingman_node** ⇒ 那份 CSV 恆不存在。
  * `master_node` / `backup_node` 兩個檔 grep `csv` / `writerow` **零命中**。
  * 節點 log 去 `/tmp/nodes.log`,不進產物目錄,下一趟直接覆寫。

後果:兩個當天才查出來的失效,**在資料上完全看不見**——
  ① 互動路徑的編隊速度指令全部低於 `AIRSPEED_MIN` 而被飛控整發拒收
  ② 備長的 slot 方位寫死正後方,雁型從來沒有被命令過

═══════════════════════════════════════════════════════════════════════════
 設計原則(每一條都是今天換來的)
═══════════════════════════════════════════════════════════════════════════
⭐ **指令值與實得值兩個都要記。** ① 之所以隱形,正是因為只要有人把
   `cmd_speed` 印出來就會發現它是 14;而只看實得地速則看不出它被拒收
   (飛機仍在飛,只是吃著別的速度)。兩個並排才問得出「指令生效了嗎」。

⭐ **slot 誤差與實際間距是兩個不同的問題,都要記。**
     `d_slot / along / cross`  → 僚機**跟不跟得上** slot
     `sep_*`                   → 雁型**有沒有真的出來**(對長機的實際相對位置)
   只記前者的話,一個「完美跟上一個錯的 slot」的航次看起來是滿分。

⚠️ **絕不阻塞主迴圈。** 開檔失敗就靜默停用(照抄 wingman 的安全形狀);
   寫入例外全部吞掉。飛行迴圈比記錄重要。

⚠️ **儀表要能自證有在動。** `stats()` 回傳每一欄的填充率,呼叫端在收工時
   印出來 —— 一欄恆空會被讀成「量到零」,而那正是我們想驗證的假說
   ([[instrument-that-cannot-fire]])。
"""
import io
import math
import os

try:                                   # 節點內走套件路徑
    from sar_swarm.geodesy import EARTH_M_PER_DEG, eas_to_tas
except ImportError:                    # 測試把 patches 目錄放在 sys.path 上
    from geodesy import EARTH_M_PER_DEG, eas_to_tas

#: ⚠️ 欄位順序即契約。**只准在尾端加**,不准插入或改名 —— 下游分析用位置讀。
COLUMNS = (
    "t", "role", "uav_id", "phase",
    # ── 速度:指令 vs 實得 ────────────────────────────────────────────
    # cmd_speed_eas   節點下的指令(EAS)
    # cmd_gs_equiv    同一個指令換算成這個高度的地速 = 「本來應該飛多快」
    # gs              實得地速
    # gs_err          實得 − 應該 ⇒ **指令有沒有生效的直接判準**
    #                 ⚠️ 指令被飛控拒收時飛機照樣在飛,只有這一欄看得出來。
    "cmd_speed_eas", "cmd_gs_equiv", "gs", "gs_err",
    # ── 能量分配:TECS 到底在拿什麼換什麼 ────────────────────────────
    # eas        實得空速(**不是地速** —— 3100 m 差 1.167 倍)
    # eas_err    實得空速 − 指令空速 ⇒ 指令有沒有被守住
    # throttle   0-1。⚠️ 飽和在 1.0 = **已經沒有剩餘推力可以拿來爬升**
    # pitch_deg  俯仰角。⚠️ 與 gam_deg(航跡角)不是同一個量:
    #            pitch = 航跡角 + 攻角。TECS 是靠俯仰換高度的。
    # climb_fc   **飛控自己算的**爬升率。用它,不要對 alt 做逐拍差分 ——
    #            差分的中位數 −13 m/s 而整段平均 +3 m/s(2026-08-25 實測),
    #            兩者不可能同時為真。
    "eas", "eas_err", "throttle", "pitch_deg", "climb_fc",
    # ── 自己的狀態 ──────────────────────────────────────────────────
    "lat", "lon", "alt", "hdg",
    # ── 長機 ────────────────────────────────────────────────────────
    "leader_id", "leader_lat", "leader_lon", "leader_hdg", "leader_speed",
    # ── slot(指令的位置)與對它的誤差 ────────────────────────────────
    "slot_lat", "slot_lon", "d_slot",
    # ── 對長機的**實際**相對位置(雁型有沒有出來)────────────────────
    # sep_m      直線距離
    # sep_along  沿長機航向的分量(負 = 在後方)
    # sep_cross  垂直長機航向的分量(正 = 在左,與 v_tail_side=+1 同號)
    # sep_up     高度差（正 = 在長機之上）。
    # ⭐⭐ 這一欄與 along/cross 合起來就是**長機座標系的完整三元組**，
    #    而三個都是**差量** ⇒ 兩邊的 datum 一起消掉。
    #    這正是使用者提的「長機 = (0,0,0)、僚機 = (0, 2500, 50)」：
    #    只要只量相對量，前面那一堆高度基準的麻煩全部消失。
    "sep_m", "sep_along", "sep_cross", "sep_up",
    # ── 指令的側後方偏置(雁型的「指令側」)──────────────────────────
    # tail_brg   送進 polar_slot 的方位偏置;180 = 正後方
    # eff_angle  展開斜坡之後的實際偏角
    "tail_brg", "eff_angle",
    # ── 編隊判定 ────────────────────────────────────────────────────
    "formed", "worst_err",
)


def _brg_components(me, leader, hdg_deg):
    """(直線距離, 沿航向分量, 垂直航向分量) —— 全部以公尺計。

    沿航向為負 = 在長機後方。垂直為正 = 在長機**左側**(與 v_tail_side +1 同號)。
    ⚠️ 基準是**長機航向**,不是到紅心的方位 —— 編隊幾何是繞著長機定義的。
       (2026-08-24 我在另一個分析裡把這兩個基準搞混過,得到一個
        「比指令多轉三倍」的物理上不可能的數字。)
    """
    if me is None or leader is None:
        return None, None, None
    dy = (me[0] - leader[0]) * EARTH_M_PER_DEG
    dx = (me[1] - leader[1]) * EARTH_M_PER_DEG * math.cos(math.radians(leader[0]))
    h = math.radians(hdg_deg or 0.0)
    along = dx * math.sin(h) + dy * math.cos(h)
    # ⚠️⚠️ **正 = 左**，而不是標準的 cross-track（正 = 右）。
    #    理由：`v_tail_side` 的 +1 是**後左**，而讀資料的人一定會把
    #    這兩欄並排比。兩個系統相反的話，一個「指令後左、實得正值」
    #    的健康航次看起來會像飛錯邊 —— 而那是一個很難反驗的錯覺。
    #    （本 repo 已經有 tests/test_gamma_sign_conventions.py 在防同型問題。）
    cross = dy * math.sin(h) - dx * math.cos(h)
    return math.hypot(dx, dy), along, cross


class FormationLog:
    """一個節點一份 CSV。⚠️ 開檔失敗就停用,永遠不拋。"""

    def __init__(self, role, uav_id, logger=None, hz=2.0):
        self.role = role
        self.uav_id = int(uav_id)
        self._log = logger
        self._period = 1.0 / float(hz) if hz > 0 else 0.5
        # ⚠️初值 None 不是 0.0：`now` 剛好是 0.0 時，`(now - 0.0) < period`
        #   成立 ⇒ **第一列被節流后掉**。生產上 `now` 是牆鐘不會是 0，
        #   但測試與任何以 t=0 起算的回放都會少一列，而那一列正好是
        #   「初始狀態」—— 最想看的那一拍。
        self._t = None
        self._fh = None
        self.n_rows = 0
        #: 每一欄寫過幾次非空值 —— 給 stats() 用(儀表自證有在動)
        self._filled = dict.fromkeys(COLUMNS, 0)
        try:
            d = os.environ.get(
                "FORMATION_LOG_DIR",
                os.path.join(os.path.expanduser("~"), "fixwing_ws", "runs",
                             "formation"))
            os.makedirs(d, exist_ok=True)
            path = os.path.join(d, "formation_%s_uav%d.csv" % (role, self.uav_id))
            self._fh = io.open(path, "w", encoding="utf-8", newline="",
                               buffering=1)
            self._fh.write(",".join(COLUMNS) + "\n")
            self.path = path
            if logger is not None:
                logger.info("編隊記錄器 → %s" % path)
        except Exception as e:                     # noqa: BLE001
            self._fh = None
            self.path = None
            if logger is not None:
                logger.warn("編隊記錄器停用:%r" % (e,))

    # ------------------------------------------------------------------
    def write(self, now, force=False, **kw):
        """節流寫一行。未提供的欄位留空(**不是 0** —— 0 是合法的角度/速度)。

        ⚠️ 留空與寫 0 的差別在這個專案裡吃過虧:一個恆為 0 的欄位會被讀成
           「量到零」,而空欄至少誠實地說「沒量到」。
        """
        # ⚠️⚠️ `force` 是給**邊界標記**用的（讓權 / 恢復）。
        #    那種列一輩子只寫一次，而節流很可能剛好把它吞掉
        #    ⇒ 一個**永遠不會寫出的儀表**，而它的缺席正好會被讀成
        #    「沒有發生讓權」。同型：[[instrument-that-cannot-fire]]。
        if self._fh is None or (not force and self._t is not None
                                and (now - self._t) < self._period):
            return
        self._t = now
        try:
            row = self._row(now, kw)
            self._fh.write(",".join(row) + "\n")
            self.n_rows += 1
        except Exception:                          # noqa: BLE001
            pass                                   # 記錄永遠不可以拖垮飛行迴圈

    def _row(self, now, kw):
        me = kw.get("me")
        leader = kw.get("leader")                  # dict: lat/lon/heading/speed/id
        alt = kw.get("alt")
        cmd = kw.get("cmd_speed_eas")
        gs = kw.get("gs")
        # ⭐ 指令換算成「本來應該飛多快」的地速 —— 與實得並排,
        #    差很多就表示指令沒生效(最常見:被飛控整發拒收)。
        cmd_gs = None
        if cmd is not None and alt is not None:
            try:
                cmd_gs = eas_to_tas(float(cmd), float(alt))
            except Exception:                      # noqa: BLE001
                cmd_gs = None
        gs_err = (float(gs) - cmd_gs) if (gs is not None and cmd_gs is not None) \
            else None
        lpos = None
        lhdg = None
        if leader is not None:
            lpos = (leader.get("lat"), leader.get("lon"))
            lhdg = leader.get("heading")
            if lpos[0] is None or lpos[1] is None:
                lpos = None
        sep, along, cross = _brg_components(me, lpos, lhdg)
        slot = kw.get("slot")
        vals = {
            "t": now, "role": self.role, "uav_id": self.uav_id,
            "phase": kw.get("phase"),
            "cmd_speed_eas": cmd, "cmd_gs_equiv": cmd_gs,
            "gs": gs, "gs_err": gs_err,
            "eas": kw.get("eas"),
            # ⚠️ 指令端是 EAS,實得端也取 EAS ⇒ **同一個量綱直接相減**。
            #    拿 gs 去減指令會系統性差 17%(3100 m 的 eas2tas)。
            "eas_err": (None if (kw.get("eas") is None or cmd is None)
                        else float(kw["eas"]) - float(cmd)),
            "throttle": kw.get("throttle"),
            "pitch_deg": kw.get("pitch_deg"),
            "climb_fc": kw.get("climb_fc"),
            "lat": me[0] if me else None, "lon": me[1] if me else None,
            "alt": alt, "hdg": kw.get("hdg"),
            "leader_id": (leader or {}).get("id"),
            "leader_lat": lpos[0] if lpos else None,
            "leader_lon": lpos[1] if lpos else None,
            "leader_hdg": lhdg,
            "leader_speed": (leader or {}).get("speed"),
            "slot_lat": slot[0] if slot else None,
            "slot_lon": slot[1] if slot else None,
            "d_slot": kw.get("d_slot"),
            "sep_m": sep, "sep_along": along, "sep_cross": cross,
            # ⚠️ 兩邊都要有值才算 —— 缺一邊就留空，**不要寫 0**。
            "sep_up": (None if (alt is None or (leader or {}).get("alt") is None)
                       else float(alt) - float(leader["alt"])),
            "tail_brg": kw.get("tail_brg"), "eff_angle": kw.get("eff_angle"),
            "formed": kw.get("formed"), "worst_err": kw.get("worst_err"),
        }
        out = []
        for c in COLUMNS:
            v = vals.get(c)
            if v is None:
                out.append("")
                continue
            self._filled[c] += 1
            if isinstance(v, bool):
                out.append("1" if v else "0")
            elif isinstance(v, float):
                out.append("%.7f" % v if c.endswith(("lat", "lon")) else "%.3f" % v)
            else:
                out.append(str(v))
        return out

    # ------------------------------------------------------------------
    def stats(self):
        """每一欄的填充率。⚠️ 恆空的欄位會被讀成「量到零」,必須自己講出來。"""
        if not self.n_rows:
            return "編隊記錄器:0 列(沒有寫過任何東西)"
        dead = [c for c in COLUMNS if self._filled[c] == 0]
        s = "編隊記錄器:%d 列 → %s" % (self.n_rows, self.path)
        if dead:
            s += ("\n  ⚠️⚠️ 這些欄位**一列都沒寫過**:%s\n"
                  "     恆空的欄位在下游會被讀成「量到零」,而那通常正好是"
                  "我們想驗證的假說。" % ", ".join(dead))
        return s

    def close(self):
        try:
            if self._fh is not None:
                self._fh.close()
        except Exception:                          # noqa: BLE001
            pass
        self._fh = None
