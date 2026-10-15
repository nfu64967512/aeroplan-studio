"""`split_area_for_fleet` 的三道新守衛（2026-09-08）。

⚠⚠⚠ 這個檔存在的理由：既有的 `tests/test_fleet_area_split.py` 18 條**全部**以兩個
位置參數呼叫（`split_area_for_fleet(areas, n)`），所以任何加在關鍵字參數上的新行為
都會讓那 18 條**維持全綠**，而生產路徑（`mission/swarm_coordinator.py` 的 HV 分支
一定會傳 `min_piece_width_m`）等於零覆蓋 —— 典型的假綠燈形狀。

因此本檔每一條都遵守同一個規矩：**先斷言「這個設定真的能讓 X 發生」，再斷言守衛
擋住了 X**。少了前置斷言，守衛就可能是「不可能觸發的儀表」（見專案筆記）。
"""

from __future__ import annotations

import logging

import pytest

from mission.swarm_coordinator import split_area_for_fleet

pytest.importorskip("shapely")

M_LAT = 111132.0


def _rect(area_id, *, half_e_m, half_n_m, lat0=23.95, lon0=120.90):
    """軸對齊矩形（東西 2·half_e、南北 2·half_n 公尺）。"""
    import math

    m_lon = 111412.0 * math.cos(math.radians(lat0))
    dlat, dlon = half_n_m / M_LAT, half_e_m / m_lon
    return {
        "area_id": area_id,
        "polygon": [
            (lat0 - dlat, lon0 - dlon),
            (lat0 - dlat, lon0 + dlon),
            (lat0 + dlat, lon0 + dlon),
            (lat0 + dlat, lon0 - dlon),
        ],
    }


def _widths(out):
    """每塊子區沿南北（切割軸）的寬度，公尺。"""
    return sorted((max(p[0] for p in a["polygon"]) - min(p[0] for p in a["polygon"])) * M_LAT
                  for a in out)


# ── 守衛一：子區最小寬度 ────────────────────────────────────────────────

def test_narrow_pieces_really_happen_without_the_guard():
    """前置斷言：不給守衛時，1560×800 配 8 台**真的**切出窄於刈幅的子區。

    沒有這一條，下一個測試就可能在「本來就不會發生」的情境上宣稱守衛有效。
    """
    out = split_area_for_fleet([_rect(1, half_e_m=780.0, half_n_m=400.0)], 8)
    w = _widths(out)
    assert len(out) == 8
    assert max(w) == pytest.approx(100.0, abs=1.0), w
    # 120 m 高度、HFOV 60° 的刈幅是 138.6 m ⇒ 100 m 的子區確實窄於一個刈幅
    assert max(w) < 138.6


def test_min_piece_width_caps_the_share_count():
    """守衛：給了最小寬度就把份數往下夾，不切出窄於刈幅的子區。"""
    area = _rect(1, half_e_m=780.0, half_n_m=400.0)
    out = split_area_for_fleet([area], 8, min_piece_width_m=138.6)

    # 800 / 138.6 = 5.77 ⇒ 最多 5 塊
    assert len(out) == 5, [a["area_id"] for a in out]
    assert min(_widths(out)) >= 138.6

    # 面積守恆仍成立（守衛只改份數，不改切法）
    import math

    from shapely.geometry import Polygon

    m_lon = 111412.0 * math.cos(math.radians(23.95))
    to_xy = [(lambda p: ((p[1] - 120.90) * m_lon, (p[0] - 23.95) * M_LAT))]
    tot = sum(Polygon([to_xy[0](p) for p in a["polygon"]]).area for a in out)
    src = Polygon([to_xy[0](p) for p in area["polygon"]]).area
    assert tot == pytest.approx(src, rel=1e-9)


def test_min_piece_width_logs_loudly_when_it_bites():
    """夾制必須**大聲**：靜靜少切幾塊 = 機隊有人沒工作而沒人知道。"""
    logger = logging.getLogger("test_fleet_split_guards.cap")
    with pytest.MonkeyPatch.context():
        records = []
        handler = logging.Handler()
        handler.emit = records.append          # type: ignore[method-assign]
        logger.addHandler(handler)
        logger.setLevel(logging.WARNING)
        try:
            split_area_for_fleet([_rect(1, half_e_m=780.0, half_n_m=400.0)], 8,
                                 logger=logger, min_piece_width_m=138.6)
        finally:
            logger.removeHandler(handler)
    msgs = [r.getMessage() for r in records]
    assert any("fleet_split_width_capped" in m for m in msgs), msgs
    assert any("8 → 5" in m for m in msgs), msgs


def test_guard_is_inert_when_pieces_are_already_wide_enough():
    """反向臂：HV4SITL9 的組態（1560×800 配 4 台 ⇒ 200 m）不該被這道守衛動到。

    ⚠ 少了這一條，守衛可能是「一律夾到 1」的壞掉版本而測試照樣綠。
    """
    area = _rect(1, half_e_m=780.0, half_n_m=400.0)
    plain = split_area_for_fleet([area], 4)
    guarded = split_area_for_fleet([area], 4, min_piece_width_m=138.6)
    assert len(plain) == len(guarded) == 4
    assert _widths(plain) == pytest.approx(_widths(guarded))
    assert min(_widths(guarded)) == pytest.approx(200.0, abs=1.0)


def test_default_is_bit_identical_to_before():
    """不給 `min_piece_width_m` 時必須與舊行為逐位相同（既有航次不可被動到）。"""
    area = _rect(1, half_e_m=780.0, half_n_m=400.0)
    for n in (2, 4, 8, 16):
        out = split_area_for_fleet([area], n)
        assert len(out) == n
        assert min(_widths(out)) == pytest.approx(800.0 / n, abs=1.0)


# ── 守衛二：最大餘數的平手要確定，不可由浮點雜訊決定 ────────────────────

def test_remainder_tie_break_is_deterministic_not_float_noise():
    """三塊**完全等面積**的母區配 4 台，多的那一份必須固定給 area_id 最小的那塊。

    ⚠ 反向臂就在輸入順序裡：id 依序是 3 / 1 / 2。舊的鍵（直接比浮點小數部分）
    在完全平手時靠 `sorted` 的穩定性選中**輸入第一個**（id=3）；修好之後應該是
    id=1。所以這條測試在還原舊鍵時會紅。
    """
    areas = [_rect(3, half_e_m=300.0, half_n_m=300.0, lon0=120.90),
             _rect(1, half_e_m=300.0, half_n_m=300.0, lon0=120.92),
             _rect(2, half_e_m=300.0, half_n_m=300.0, lon0=120.94)]

    # 前置斷言：三塊真的是平手（差 < 1e-9 相對），否則這條測不到平手邏輯
    import math

    from shapely.geometry import Polygon

    m_lon = 111412.0 * math.cos(math.radians(23.95))
    ar = [Polygon([((p[1] - 120.90) * m_lon, (p[0] - 23.95) * M_LAT)
                   for p in a["polygon"]]).area for a in areas]
    assert max(ar) - min(ar) < max(ar) * 1e-9, ar

    out = split_area_for_fleet(areas, 4)
    assert len(out) == 4, [a["area_id"] for a in out]

    # 被切開的那一塊會產生兩個帶 `_src_area_id` 的子區
    split_src = {a["_src_area_id"] for a in out if a.get("_src_area_id") is not None}
    assert split_src == {1}, [dict(area_id=a["area_id"],
                                   src=a.get("_src_area_id")) for a in out]


def test_tie_break_survives_input_reordering():
    """同一組母區換輸入順序，被切開的仍是同一塊 ⇒ 計畫可重現。"""
    mk = lambda: [_rect(3, half_e_m=300.0, half_n_m=300.0, lon0=120.90),   # noqa: E731
                  _rect(1, half_e_m=300.0, half_n_m=300.0, lon0=120.92),
                  _rect(2, half_e_m=300.0, half_n_m=300.0, lon0=120.94)]
    a, b = mk(), list(reversed(mk()))
    src_a = {x["_src_area_id"] for x in split_area_for_fleet(a, 4)
             if x.get("_src_area_id") is not None}
    src_b = {x["_src_area_id"] for x in split_area_for_fleet(b, 4)
             if x.get("_src_area_id") is not None}
    assert src_a == src_b == {1}


# ── 守衛三：溯源要跨得過第二層切割 ──────────────────────────────────────

def test_src_area_id_points_at_the_user_drawn_area_after_two_splits():
    """切過的輸入再切一次，`_src_area_id` 必須仍指向使用者畫的那塊。

    ⚠ 修好之前這裡會拿到**上一層子區**的 id（例如 2/3），使用者在統計面板上
    就對不回自己畫的那塊區。目前生產路徑靠「機隊切區先於禁航區切區」的順序
    僥倖沒事，順序一換就壞。
    """
    once = split_area_for_fleet([_rect(1, half_e_m=780.0, half_n_m=400.0)], 2)
    assert len(once) == 2
    assert {a["_src_area_id"] for a in once} == {1}

    twice = split_area_for_fleet(once, 4)
    assert len(twice) >= 4
    assert {a["_src_area_id"] for a in twice} == {1}, \
        [(a["area_id"], a.get("_src_area_id")) for a in twice]
    # id 仍是唯一整數
    ids = [a["area_id"] for a in twice]
    assert len(set(ids)) == len(ids)
    assert all(isinstance(i, int) for i in ids)
