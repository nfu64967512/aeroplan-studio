# -*- coding: utf-8 -*-
"""編隊巡航段記錄器(2026-08-24)。

═══════════════════════════════════════════════════════════════════════════
 為什麼需要它,以及它為什麼要這樣設計
═══════════════════════════════════════════════════════════════════════════
編隊巡航段先前**完全沒有落盤資料**:
  * `tot_node` 的 CSV 從「收到目標」才開始 ⇒ 編隊段整個在它之前
  * 唯一的指令記錄器在 `wingman_node.py:424`,而 **n=2 不生 wingman_node**
  * `master_node` / `backup_node` 兩檔 grep `csv` / `writerow` **零命中**
  * 節點 log 去 `/tmp/nodes.log`,不進產物目錄,下一趟直接覆寫

後果:當天查出來的兩個失效**在資料上完全看不見** ——
  ① 編隊速度指令全部低於 `AIRSPEED_MIN` 而被飛控整發拒收
  ② 備長的 slot 方位寫死正後方,雁型從來沒有被命令過

⇒ 這個檔守的是「下一次同型失效**會**在資料裡留下痕跡」。
"""
import io
import math
import os
import sys
import tempfile
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts" / "wsl" / "sar_swarm_patches"))

from formation_log import COLUMNS, FormationLog, _brg_components  # noqa: E402


@pytest.fixture()
def flog(tmp_path, monkeypatch):
    monkeypatch.setenv("FORMATION_LOG_DIR", str(tmp_path))
    f = FormationLog("backup", 2, logger=None, hz=1000.0)   # hz 高 = 不節流
    yield f
    f.close()


def rows_of(f):
    f.close()
    with io.open(f.path, encoding="utf-8", newline="") as fh:
        return [ln.rstrip("\n").split(",") for ln in fh]


# ══════════════════════════════════════════════════════════════════════
#  紅線 1:指令值與實得值**兩個都要記**
# ══════════════════════════════════════════════════════════════════════

def test_it_records_both_the_command_and_what_was_achieved(flog):
    """⭐⭐⭐ 這是整個記錄器存在的理由。

    ① 之所以隱形,正是因為只要有人把 `cmd_speed` 印出來就會發現它是 14;
    而只看實得地速則看不出指令被拒收 —— 飛機仍在飛,只是吃著別的速度。
    **兩個並排**才問得出「指令生效了嗎」。
    """
    for c in ("cmd_speed_eas", "cmd_gs_equiv", "gs", "gs_err"):
        assert c in COLUMNS, "少了 %s ⇒ 指令有沒有生效就變成不可觀測" % c


def test_gs_err_is_the_direct_verdict_on_whether_the_command_took(flog):
    """`gs_err = 實得地速 − 指令換算的地速`。

    指令 135 EAS 在 3100 m 是地速 157。若飛機實際在飛 90(指令被拒收、
    吃著別的速度),gs_err 應該是 −67 —— 一眼就看得出來。
    """
    flog.write(1.0, cmd_speed_eas=135.0, alt=3100.0, gs=90.0)
    r = rows_of(flog)
    assert len(r) == 2, r
    hdr, row = r[0], r[1]
    d = dict(zip(hdr, row))
    cgs = float(d["cmd_gs_equiv"])
    assert 155.0 < cgs < 160.0, "135 EAS @3100 應該是地速 ~157,實得 %.1f" % cgs
    assert abs(float(d["gs_err"]) - (90.0 - cgs)) < 0.01


def test_a_healthy_command_shows_a_small_gs_err(flog):
    flog.write(1.0, cmd_speed_eas=135.0, alt=3100.0, gs=157.0)
    d = dict(zip(*rows_of(flog)[:2]))
    assert abs(float(d["gs_err"])) < 2.0, d["gs_err"]


# ══════════════════════════════════════════════════════════════════════
#  紅線 2:slot 誤差與實際間距是兩個不同的問題
# ══════════════════════════════════════════════════════════════════════

def test_slot_error_and_actual_separation_are_separate_columns():
    """⚠️ 只記 `d_slot` 的話,一個「完美跟上一個**錯的** slot」的航次
    看起來是滿分 —— 而 2026-08-24 之前 slot 方位就是寫死正後方的。
    """
    assert "d_slot" in COLUMNS
    for c in ("sep_m", "sep_along", "sep_cross"):
        assert c in COLUMNS, "少了 %s ⇒ 「雁型有沒有真的出來」不可觀測" % c


def test_separation_is_measured_against_the_leader_heading():
    """⚠️⚠️ 基準是**長機航向**,不是到紅心的方位。

    2026-08-24 我在另一個分析裡把這兩個基準搞混,得到「實得比指令多轉
    2.99 倍」這個物理上不可能的數字,而它看起來很像一個追蹤異常。
    """
    E = 111320.0
    leader = (25.0, 121.0)
    # 長機朝正北(0°);僚機在正南 1000 m
    me = (25.0 - 1000.0 / E, 121.0)
    sep, along, cross = _brg_components(me, leader, 0.0)
    assert abs(sep - 1000.0) < 1.0
    assert along < -990.0, "正後方應該是**負的**沿航向分量:%.1f" % along
    assert abs(cross) < 1.0, "正後方不該有垂航向分量:%.1f" % cross

    # 同一個位置,但長機朝東(90°)⇒ 僚機變成在右側
    sep2, along2, cross2 = _brg_components(me, leader, 90.0)
    assert abs(along2) < 1.0, along2
    # ⚠️ 慣例是**正 = 左**(與 v_tail_side=+1「後左」同號),
    #    不是標準 cross-track 的「正 = 右」。朝東時正南是右側 ⇒ 負。
    assert cross2 < -990.0, "朝東時正南應為負(右側):%.1f" % cross2


def test_echelon_offset_shows_up_in_both_components():
    """雁型 = 後方**且**側向。2500 m @ 25° ⇒ 落後 2266、側偏 1057。"""
    E = 111320.0
    leader = (25.0, 121.0)
    d, th = 2500.0, math.radians(25.0)
    # 長機朝北;slot 在後左 ⇒ south 2266、west 1057
    me = (25.0 - d * math.cos(th) / E,
          121.0 - d * math.sin(th) / (E * math.cos(math.radians(25.0))))
    sep, along, cross = _brg_components(me, leader, 0.0)
    assert abs(sep - 2500.0) < 5.0, sep
    assert abs(along + 2266.0) < 5.0, along
    assert abs(cross - 1057.0) < 5.0, cross   # 後左 ⇒ 正


# ══════════════════════════════════════════════════════════════════════
#  紅線 3:空 ≠ 0
# ══════════════════════════════════════════════════════════════════════

def test_missing_values_stay_empty_not_zero(flog):
    """⚠️⚠️ 一個恆為 0 的欄位會被讀成「量到零」,而空欄至少誠實地說
    「沒量到」。這個專案已經因為分不清這兩件事吃過虧。
    """
    flog.write(1.0, gs=100.0)                 # 只給 gs,其他都沒有
    hdr, row = rows_of(flog)[:2]
    d = dict(zip(hdr, row))
    for c in ("cmd_speed_eas", "sep_m", "tail_brg", "d_slot", "formed"):
        assert d[c] == "", "%s 應該留空,實得 %r" % (c, d[c])
    assert float(d["gs"]) == 100.0


def test_zero_is_written_as_zero_not_empty(flog):
    """⚠️ 反向:真的量到 0 就要寫 0。把 0 寫成空會讓「正後方(偏角 0)」
    與「沒算過」變成同一件事。
    """
    flog.write(1.0, eff_angle=0.0, tail_brg=180.0)
    d = dict(zip(*rows_of(flog)[:2]))
    assert d["eff_angle"] == "0.000", d["eff_angle"]
    assert float(d["tail_brg"]) == 180.0


# ══════════════════════════════════════════════════════════════════════
#  紅線 4:儀表要能自證有在動
# ══════════════════════════════════════════════════════════════════════

def test_stats_names_the_columns_that_never_got_a_value(flog):
    """⭐⭐ 恆空的欄位在下游會被讀成「量到零」,而那通常正好是我們想
    驗證的假說([[instrument-that-cannot-fire]])。記錄器要自己講出來。
    """
    for t in range(5):
        flog.write(float(t), gs=100.0 + t)
    s = flog.stats()
    assert "5 列" in s, s
    assert "一列都沒寫過" in s, s
    assert "sep_m" in s and "tail_brg" in s, s


def test_stats_is_quiet_when_everything_is_filled(flog):
    kw = dict(cmd_speed_eas=135.0, alt=3100.0, gs=157.0, hdg=90.0,
              # ⚠️ 新增欄位要一起補 —— 這條測試就是在抓「有欄位恆空」,
              #    它已經連續兩次咬到我自己新加的欄。
              eas=134.0, throttle=0.62, pitch_deg=3.4, climb_fc=4.1,
              me=(25.0, 121.0), slot=(25.01, 121.01), d_slot=42.0,
              tail_brg=205.0, eff_angle=25.0, formed=True, worst_err=30.0,
              phase="CRUISE",
              # ⚠️ `alt` 不可省 —— `sep_up` 需要它。
              #    少了那一項 sep_up 恆空，而這條測試就是在拓
              #    「有欄位恆空」—— 它確實咬到了我新加的那一欄。
              leader={"id": 1, "lat": 25.02, "lon": 121.02, "alt": 3050.0,
                      "heading": 90.0, "speed": 135.0})
    for t in range(3):
        flog.write(float(t), **kw)
    s = flog.stats()
    assert "一列都沒寫過" not in s, s


def test_zero_rows_is_reported_as_zero_rows(flog):
    assert "0 列" in flog.stats()


# ══════════════════════════════════════════════════════════════════════
#  紅線 5:絕不拖垮飛行迴圈
# ══════════════════════════════════════════════════════════════════════

def test_open_failure_disables_silently_and_never_raises(monkeypatch, tmp_path):
    """⚠️ 飛行迴圈比記錄重要。開檔失敗就停用,不可以拋。"""
    bad = tmp_path / "nope"
    bad.write_text("我是檔案不是目錄", encoding="utf-8")
    monkeypatch.setenv("FORMATION_LOG_DIR", str(bad))
    f = FormationLog("backup", 2, logger=None)
    f.write(1.0, gs=100.0)                    # 不可以拋
    assert f.path is None
    assert "0 列" in f.stats()
    f.close()


def test_write_swallows_exceptions(flog):
    class Boom:
        def __getitem__(self, _):
            raise RuntimeError("炸")

    flog.write(1.0, me=Boom())                # 不可以拋


def test_throttling_limits_the_row_rate(tmp_path, monkeypatch):
    monkeypatch.setenv("FORMATION_LOG_DIR", str(tmp_path))
    f = FormationLog("backup", 2, logger=None, hz=2.0)     # 每 0.5 s 一列
    for k in range(100):
        f.write(k * 0.05, gs=100.0)           # 5 秒份、100 次呼叫
    n = f.n_rows
    f.close()
    assert 8 <= n <= 12, "5 秒 @2 Hz 應該約 10 列,實得 %d" % n


# ══════════════════════════════════════════════════════════════════════
#  欄位順序是契約
# ══════════════════════════════════════════════════════════════════════

def test_column_order_is_a_contract(flog):
    """⚠️ 下游分析用位置讀。只准在尾端加,不准插入或改名。"""
    assert COLUMNS[:4] == ("t", "role", "uav_id", "phase"), COLUMNS[:4]
    flog.write(1.0, gs=1.0)
    assert rows_of(flog)[0] == list(COLUMNS)


# ══════════════════════════════════════════════════════════════════════
#  紅線 6:單位與座標系 —— 欄位名說的必須就是欄位裡的東西
# ══════════════════════════════════════════════════════════════════════

def test_nodes_do_not_write_odom_metres_into_the_latlon_columns():
    """⚠️⚠️⚠️ `mavros_interface.position()` 回傳的是 odometry 的**區域 ENU
    公尺**(x, y, z),**不是 lat/lon**。

    FLOG1 實測(2026-08-24):master 那一欄寫出
        lat = -57659.4123834,  lon = 13652.2297218
    數字看起來像座標、欄位名說它是座標,而它是公尺 —— 而且**沒有任何
    東西會報錯**。下游任何幾何都會靜靜地算出垃圾。

    ⇒ 真的經緯度:master 走 `self.master.lat/lon`(GeoPose 那一份),
      backup 走 `self.my_fix`(NavSatFix)。
    """
    src = (REPO / "scripts" / "wsl" / "sar_swarm_patches"
           / "master_node.py").read_text(encoding="utf-8")
    i = src.index("def _flog_row")
    j = src.index("def _leader_speed_control", i)
    body = src[i:j]
    assert "me=me" in body or "m.lat" in body, body[-400:]
    assert "p[:2]" not in body, (
        "⚠️⚠️ master 的 _flog_row 又用 `position()[:2]` 當經緯度了 —— "
        "那是區域 ENU 公尺,不是 lat/lon。")


def test_heading_is_degrees_from_north_not_enu_radians():
    """⚠️⚠️ `mav.heading()` 是 **ENU 弧度、從東起算**;`hdg` 欄與
    `_brg_components` 吃的是**度、從北起算**。

    直接寫進去會得到一個看起來完全合理的小數(FLOG1 實測 0.664),
    而單位全錯。0.664 「像」一個角度,所以不會有人多看一眼。
    """
    src = (REPO / "scripts" / "wsl" / "sar_swarm_patches"
           / "backup_node.py").read_text(encoding="utf-8")
    i = src.index("self._flog.write(")
    body = src[i:i + 1400]
    assert "self.mav.heading()" not in body or "math.degrees" in body, (
        "⚠️ backup 把 mav.heading() 的 ENU 弧度直接寫進 hdg 欄")
    assert "90.0 - math.degrees" in body, (
        "⚠️ 沒有做 ENU弧度→北向度 的換算:(90 - degrees(rad)) mod 360")


@pytest.mark.parametrize("enu_rad,want_deg", [
    (0.0, 90.0),                      # 正東 -> 090
    (math.pi / 2, 0.0),               # 正北 -> 000
    (math.pi, 270.0),                 # 正西 -> 270
    (-math.pi / 2, 180.0),            # 正南 -> 180
])
def test_the_enu_to_compass_conversion_itself(enu_rad, want_deg):
    """換算式本身。⚠️ 方向搞反的話每個值仍然落在 0-360,看不出來。"""
    got = (90.0 - math.degrees(enu_rad)) % 360.0
    assert abs(got - want_deg) < 1e-9, (enu_rad, got, want_deg)


# ══════════════════════════════════════════════════════════════════════
#  紅線 7:能量分配的四個量(2026-08-25)
# ══════════════════════════════════════════════════════════════════════

def test_the_energy_columns_exist():
    """⭐⭐⭐ 四趟實測的形態是「**毛爬升多、保留率低**」:

        僚機毛爬升 7273-8132 m(比長機的 5154-5874 還多),保留率 5-23%
        長機                                                 保留率 52-60%

    而我的兩個機制假說(坡度太大、目標不可達)都已被實測推翻。
    ⇒ 少了 pitch / throttle / 空速就**分不出 TECS 是拿空速換高度、
      還是拿高度換空速** —— 而那是唯一剩下的候選。
    """
    for c in ("eas", "eas_err", "throttle", "pitch_deg", "climb_fc"):
        assert c in COLUMNS, "少了 %s ⇒ 能量分配不可觀測" % c


def test_eas_err_compares_like_with_like(flog):
    """⚠️⚠️ 指令端是 **EAS**,所以實得端也要取 EAS。

    拿地速去減 EAS 指令會系統性差 17%(3100 m 的 eas2tas = 1.167)——
    而那個差看起來就像一個真實的追蹤誤差。本專案已經因為混用這兩個
    量綱吃過虧([[eas-vs-groundspeed-157]])。
    """
    flog.write(1.0, cmd_speed_eas=135.0, eas=130.0, alt=3100.0, gs=157.0)
    d = dict(zip(*rows_of(flog)[:2]))
    assert abs(float(d["eas_err"]) - (-5.0)) < 0.01, d["eas_err"]
    # ⭐ 同一列的 gs_err 用的是**地速**基準,兩者必須並存而且不同值
    assert abs(float(d["gs_err"])) < 2.0, (
        "gs_err 應該對地速比(157 vs 157),實得 %s" % d["gs_err"])


def test_climb_fc_is_recorded_because_differencing_alt_is_garbage(flog):
    """⚠️⚠️ 不要對 alt 做逐拍差分。

    2026-08-25 實測:同一段資料,逐拍差分的中位數是 **−13 m/s**,
    而整段平均是 **+3 m/s** —— 兩者不可能同時為真。差分被遙測抖動主導。
    ⇒ 記飛控自己算的 `climb`,那是一個**量測**不是一個推導。
    """
    flog.write(1.0, climb_fc=4.2)
    d = dict(zip(*rows_of(flog)[:2]))
    assert abs(float(d["climb_fc"]) - 4.2) < 1e-6


def test_energy_columns_stay_empty_when_not_supplied(flog):
    """⚠️ 沒量到就留空。throttle=0 是**合法值**(慢車),不可以與「沒量到」同形。"""
    flog.write(1.0, gs=100.0)
    d = dict(zip(*rows_of(flog)[:2]))
    for c in ("eas", "eas_err", "throttle", "pitch_deg", "climb_fc"):
        assert d[c] == "", "%s 應該留空,實得 %r" % (c, d[c])
    # 真的量到 0 就要寫 0
    f2 = FormationLog("backup", 3, logger=None, hz=1000.0)
    f2.write(1.0, throttle=0.0, pitch_deg=0.0)
    d2 = dict(zip(*rows_of(f2)[:2]))
    assert d2["throttle"] == "0.000" and d2["pitch_deg"] == "0.000", d2


def test_the_mavros_message_type_is_the_ros_camelcase_one():
    """⚠️⚠️ 型別名是 **VfrHud**(駝峰),不是 MAVLink 那邊的 `VFR_HUD`。

    寫錯會在 import 就死 ⇒ **節點整個起不來**,而症狀是「沒有進入 ready」。
    實測:`ros2 topic type /uav2/mavros/vfr_hud` 回 mavros_msgs/msg/VfrHud,
    而 `from mavros_msgs.msg import VFR_HUD` 直接 ImportError。
    我第一版就是寫成 VFR_HUD,靠在 WSL 裡先試 import 才發現。
    """
    src = (REPO / "scripts" / "wsl" / "sar_swarm_patches"
           / "mavros_interface.py").read_text(encoding="utf-8")
    # ⚠️⚠️ 只看 **import 與訂閱那兩行**,不要整檔 grep:
    #    我在旁邊寫了「不是 MAVLink 那邊的 VFR_HUD」的解說註解,
    #    整檔 grep 會被自己的說明文字觸發 —— 這是本 session 第三次
    #    做出「會對自己的註解開火」的守衛,而那種守衛下一個人只會刪掉。
    import re as _re
    imp = _re.search(r"from mavros_msgs\.msg import \(([^)]*)\)", src)
    assert imp, "找不到 mavros_msgs.msg 的 import"
    assert "VfrHud" in imp.group(1), "import 的不是 VfrHud:%r" % imp.group(1)
    assert "VFR_HUD" not in imp.group(1), (
        "⚠️ import 用了 MAVLink 的 VFR_HUD 名字 —— mavros_msgs 裡沒有它,"
        "會直接 ImportError,而節點起不來的症狀是「沒有進入 ready」")
    sub = [ln for ln in src.splitlines() if "vfr_hud" in ln and "create_subscription" in ln]
    assert sub and "VfrHud" in sub[0], "訂閱用的型別不對:%r" % (sub or "<沒有訂閱>")


def test_pitch_is_not_the_same_as_flight_path_angle():
    """⚠️ pitch = 航跡角 + 攻角。要分辨「拿空速換高度」還是「拿高度換空速」
    就必須看 pitch —— 只看 gam_deg 看不出 TECS 在做什麼。
    """
    assert "pitch_deg" in COLUMNS and "climb_fc" in COLUMNS
