"""AUTO 預規劃航線:任務項建構 + MISSION 上傳協商狀態機(純邏輯)。

【這個檔存在的理由】
2026-08-25 之前,本專案**完全沒有 mission 上傳能力**:
  * `mavros_interface.send_guided_waypoint()` 的 docstring 說它送
    `MISSION_ITEM_INT current=2`,那是 ArduPilot 的 legacy「飛到這裡」,
    不進任務清單、AUTO 模式永遠不會飛它;而且實作早就改成
    `COMMAND_INT / MAV_CMD_DO_REPOSITION` 了(docstring 沒跟上)。
  * `tot_node` 整份檔案沒有任何 MISSION_COUNT / MISSION_ITEM 的處理。
⇒ 「事前規劃航線走 AUTO」需要的是**完整的 mission 協商**,不是再送一顆
   航點。這個檔負責協商的**邏輯**,不負責把位元塞進 MAVLink。

⚠️⚠️ 本檔**刻意不 import pymavlink、也不 import rclpy**。
   本專案已經確立的規矩是「跑不到的鎖等於沒有鎖」
   (tests/test_mpng_value_locks.py:36-38),而 Windows 開發機上
   **沒有 pymavlink** —— tests/test_reposition_change_mode.py 的常數檢查
   因此只能 `importorskip`,在開發機上等於不存在。
   偏偏任務項的欄位順序、序號協商、逾時重送這三件事正是
   「最容易寫錯、又最不會有人發現」的部分:寫錯了飛控只會回一個
   MISSION_ACK 錯誤碼,而沒有人在讀那個碼。
   ⇒ 這裡只產生「要送什麼」的純資料 + 純狀態機,`mav.mission_item_int_send()`
     那一行留在 tot_node.py。這樣每一條鎖在開發機上**每次都跑得到**。

⚠️ 幾何(lat/lon 怎麼算)**不在這裡**:tot_node 已經有 `dest()` / `bearing()`
   / `ring_point_of()` / `_fan_off()` 那一整套,再抄一份就會有兩份會分歧的
   幾何。本檔收「已經算好的點」。
"""
import collections

#: MAVLink 常數。⚠️ **不從 pymavlink import** —— 見檔頭。數值取自 MAVLink
#  common.xml,並由 tests/test_auto_mission.py 在有 pymavlink 的機器上
#  逐項比對真值(沒有 pymavlink 時那一條 skip,其餘的鎖照跑)。
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LOITER_UNLIM = 17
MAV_MISSION_TYPE_MISSION = 0
MAV_MISSION_ACCEPTED = 0

#: 座標的定點編碼倍率(MISSION_ITEM_INT 的 x/y 是 int32 的 1e-7 度)。
COORD_SCALE = 1e7

#: MISSION_ITEM_INT 的欄位。⚠️ **順序就是 MAVLink 的順序**,而 tot_node
#  是用**位置引數**把它餵進 `mission_item_int_send` 的 ⇒ 這個 namedtuple
#  的欄位順序是介面的一部分,不是內部細節。tests 有鎖。
MissionItem = collections.namedtuple(
    "MissionItem",
    ("seq", "frame", "command", "current", "autocontinue",
     "param1", "param2", "param3", "param4", "x", "y", "z", "mission_type"))


def _ll(pt):
    """(lat, lon) → 定點 (x, y)。⚠️ 用 round() 不是 int():
    int() 會**向零截斷**,南半球/西半球因此系統性偏一個 1e-7 度的方向,
    而那是 1.1 cm —— 小到永遠不會有人發現,也小到不影響任何結論。
    用 round() 純粹是因為「對稱」比「碰巧夠小」更好維護。"""
    return int(round(float(pt[0]) * COORD_SCALE)), int(round(float(pt[1]) * COORD_SCALE))


def build_strike_mission(here, ip, ip_alt, loiter_r, transit=None,
                         transit_alt=None):
    """打擊航線的任務項清單:[home 佔位, (選配)進場點, 攻擊發起點盤旋]。

    參數都是**已經算好的**點:
        here        (lat, lon)  本機當下位置 —— seq 0 的 home 佔位
        ip          (lat, lon)  攻擊發起點(= 本機自己的 pushover 環點)
        ip_alt      float       盤旋高度(m,相對 HOME)
        loiter_r    float       盤旋半徑(m)
        transit     (lat, lon)  選配的進場航點;None = 直接飛 IP
        transit_alt float       進場航點高度;None ⇒ 用 ip_alt

    ⚠️⚠️ **seq 0 一定要有而且一定是佔位**。ArduPilot 的任務清單第 0 項是
       HOME,飛控會用自己的 home 覆蓋它;但**協商時 count 必須把它算進去**,
       否則飛控要 seq N-1 而我們手上只有 N-1 項 ⇒ 送不出去 ⇒ 逾時。
       這是 MAVLink mission protocol 最常見的離一錯誤。

    ⚠️ 盤旋用 `NAV_LOITER_UNLIM` 而不是 `NAV_LOITER_TIME`:
       「盤旋到大家都到」的判定在**節點**手上(`_gate_all`),不是在飛控手上。
       給飛控一個會自己結束的盤旋 = 兩個地方各有一份「等多久」的定義,
       而它們一定會分歧。飛控只負責「無限期繞在這裡」,何時離開由我們
       切 GUIDED 決定。

    ⚠️⚠️ `loiter_r` 傳 0 或負值是**災難**,和 `_reposition` 的 param3 同一個病:
       ArduPlane 會退回 `WP_LOITER_RAD`(現行 1488)再乘 eas2tas²
       ⇒ 3100 m 高度實際 ~2030 m,而我們**以為**自己指定了半徑。
       ⇒ 這裡直接拒絕,不做「善意的」預設值。
    """
    if not (float(loiter_r) > 0.0):
        raise ValueError(
            f"loiter_r 必須 > 0(收到 {loiter_r})—— 傳 0 會讓 ArduPlane 退回 "
            f"WP_LOITER_RAD 再乘 eas2tas²,而我們會以為半徑是自己指定的")
    items = []

    def _add(pt, alt, command, param3=0.0):
        x, y = _ll(pt)
        items.append(MissionItem(
            seq=len(items),
            frame=MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            command=command,
            # ⚠️ current 只有 seq 0 是 1(MAVLink 慣例),不是「我們想從這裡開始」。
            #    真正的「從哪一項開始」由 MISSION_SET_CURRENT 決定,不在這裡。
            current=1 if not items else 0,
            autocontinue=1,
            param1=0.0, param2=0.0, param3=float(param3), param4=0.0,
            x=x, y=y, z=float(alt), mission_type=MAV_MISSION_TYPE_MISSION))

    _add(here, 0.0, MAV_CMD_NAV_WAYPOINT)              # seq 0:home 佔位
    if transit is not None:
        _add(transit, ip_alt if transit_alt is None else transit_alt,
             MAV_CMD_NAV_WAYPOINT)
    # ⚠️ 盤旋半徑的**正負號**在 ArduPilot 代表繞行方向(正=順時針)。
    #    這裡一律取正:方向由誰決定不該藏在一個半徑的符號裡。
    _add(ip, ip_alt, MAV_CMD_NAV_LOITER_UNLIM, param3=abs(float(loiter_r)))
    return items


#: 上傳狀態機的終態。⚠️ 終態是**黏著**的:進去就不會再出來,
#  否則一則遲到的 MISSION_REQUEST 會把已經失敗的上傳「復活」,
#  而復活之後的狀態沒有任何人在追。
TERMINAL = ("done", "failed")


class MissionUploader:
    """一架飛機的 MISSION 上傳協商(純狀態機,不做 I/O、不看牆鐘)。

    用法(呼叫端每 tick 給它現在時刻,拿回「要送什麼」):

        up = MissionUploader(items)
        act = up.begin(now)                 # → {"send": "count", ...}
        act = up.on_request(seq, now)       # 收到 MISSION_REQUEST(_INT)
        act = up.on_ack(ack_type, now)      # 收到 MISSION_ACK
        act = up.poll(now)                  # 沒收到東西時的逾時/重送

    每個方法回傳一個 **action dict**,可能是:
        {"send": "count", "count": n}
        {"send": "item", "seq": k, "item": MissionItem}
        {"done": True}
        {"failed": "<理由>"}
        None                                 (這一拍不必做事)

    ⚠️⚠️ **時間一律由外面傳進來**。本檔不 import time ——
       本專案的 conftest.py 有一整段在講「測試凍結了 stdlib time 沒還原,
       八十分鐘 CPU 被吃掉」的事故。狀態機自己讀時鐘 = 逾時邏輯只能靠
       凍結全域時鐘來測,而那正是那個事故的形狀。
    """

    def __init__(self, items, item_timeout=3.0, total_timeout=30.0,
                 max_retry=4):
        if not items:
            raise ValueError("空的任務清單不可上傳 —— 那會讓飛控清掉現有任務"
                             "而且看起來像成功")
        self.items = list(items)
        self.item_timeout = float(item_timeout)
        self.total_timeout = float(total_timeout)
        self.max_retry = int(max_retry)
        self.state = "idle"
        self.reason = ""
        #: 已經**送出過**的 seq。⚠️ 用集合不是計數器:飛控會重複要同一個
        #  seq(丟包重試),用計數器會以為進度比實際快 —— 而「以為送完了」
        #  的下一步是接受一則早到的 ACK,那就是一個假綠燈。
        self.served = set()
        self.retries = 0
        self.last_seq = None
        self._t0 = None
        self._t_send = None

    # ── 查詢 ──────────────────────────────────────────────────────────────
    @property
    def finished(self):
        return self.state in TERMINAL

    @property
    def ok(self):
        return self.state == "done"

    def progress(self):
        """(已送項數, 總項數) —— 給 CSV / log 用,不參與判定。"""
        return len(self.served), len(self.items)

    # ── 驅動 ──────────────────────────────────────────────────────────────
    def begin(self, now):
        """開始上傳:送 MISSION_COUNT。重入是安全的(已開始就回 None)。"""
        if self.state != "idle":
            return None
        self._t0 = float(now)
        self.state = "count"
        return self._mark_count(now)

    def on_request(self, seq, now):
        """飛控要第 seq 項。⚠️ **不假設 seq 是遞增的**:MAVLink 允許
        飛控重要同一項(丟包)或跳回去,照著要的那一項給就對了。"""
        if self.finished:
            return None
        seq = int(seq)
        if not (0 <= seq < len(self.items)):
            # ⚠️ 越界不是「忽略就好」:它表示我們送的 count 與飛控理解的
            #    不一致(離一錯誤的典型表徵),而繼續下去只會逾時 ——
            #    逾時的訊息長得像「鏈路壞了」,會把人帶去查錯的地方。
            return self._fail(f"飛控要 seq={seq},但清單只有 "
                              f"{len(self.items)} 項(0..{len(self.items) - 1})"
                              f" ⇒ count 與飛控理解不一致")
        self.state = "items"
        self.retries = 0
        self.served.add(seq)
        self.last_seq = seq
        self._t_send = float(now)
        return {"send": "item", "seq": seq, "item": self.items[seq]}

    def on_ack(self, ack_type, now):
        """收到 MISSION_ACK。0 = ACCEPTED,其餘一律是失敗。"""
        if self.finished:
            return None
        if int(ack_type) != MAV_MISSION_ACCEPTED:
            return self._fail(f"飛控回 MISSION_ACK type={int(ack_type)}"
                              f"(0 才是 ACCEPTED)")
        if len(self.served) < len(self.items):
            # ⚠️⚠️ **早到的 ACCEPTED 不可以採信。** 它有兩個來源:
            #   ① 上一次上傳的遲到 ACK(換目標重打時真的會發生);
            #   ② 我們的 count 比實際項數少(離一錯誤)。
            #   兩者都會讓「上傳成功」變成假綠燈,而假綠燈的下游是
            #   一架飛在錯誤航線上的飛機。寧可等到逾時再大聲失敗。
            return {"ignored": "premature_ack",
                    "served": len(self.served), "count": len(self.items)}
        self.state = "done"
        return {"done": True}

    def poll(self, now):
        """沒有收到任何東西時的逾時處理:重送最後一次送的東西。"""
        if self.finished or self.state == "idle":
            return None
        now = float(now)
        if now - self._t0 > self.total_timeout:
            return self._fail(
                f"上傳逾時 {self.total_timeout:.0f}s"
                f"(已送 {len(self.served)}/{len(self.items)} 項,"
                f"重送 {self.retries} 次)")
        if self._t_send is None or now - self._t_send <= self.item_timeout:
            return None
        if self.retries >= self.max_retry:
            return self._fail(
                f"重送 {self.retries} 次仍無回應"
                f"(已送 {len(self.served)}/{len(self.items)} 項)")
        self.retries += 1
        if self.state == "count":
            return self._mark_count(now)
        self._t_send = now
        return {"send": "item", "seq": self.last_seq,
                "item": self.items[self.last_seq]}

    # ── 內部 ──────────────────────────────────────────────────────────────
    def _mark_count(self, now):
        self._t_send = float(now)
        return {"send": "count", "count": len(self.items)}

    def _fail(self, reason):
        self.state = "failed"
        self.reason = reason
        return {"failed": reason}
