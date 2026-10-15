"""守住 HARVEST 改名之後的 **vendored 命名邊界**（ADR-0010）。

2026-09-09 起演算法一律叫 **HARVEST**，但兩樣東西**刻意維持原名**：

1. vendored 套件目錄 / import 路徑 `hv_dccpp`（改它等於從上游分叉，
   `docs/hv_dccpp/VENDOR.md` 的重新同步程序就作廢了）。
2. **跨進 vendored 轉接層的參數鍵**。

⚠⚠⚠ 第 2 點是本檔存在的真正理由：`hv_dccpp/integration/aeroplan_adapter.py`
只認得少數幾個 `hv_*` 名字。AeroPlan 這一側若在某次「全面改名」時把它們一起改掉，
轉接層會拿不到值而**靜靜退回預設**——不會拋例外、不會有 log、規劃照樣跑完，
只是所有參數都不是你設的那些。那正是本專案「有效值契約」明文禁止的靜默失敗。

所以這裡用**值層級**驗證：不是檢查某一行長什麼樣，而是餵一份用該鍵的輸入進去，
斷言產物真的跟著變；再把鍵改名，斷言產物**不再**跟著變（刪除實驗）。
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

pytest.importorskip("shapely")

ROOT = Path(__file__).resolve().parent.parent
ADAPTER = ROOT / "hv_dccpp" / "integration" / "aeroplan_adapter.py"

# 這四個是 2026-09-09 實測 `grep -o 'hv_[a-z_0-9]*'` 在轉接層裡的**全部**命中。
FROZEN_BOUNDARY_KEYS = {"hv_cfg", "hv_class", "hv_dccpp", "hv_uav_id"}


def test_vendored_package_keeps_its_name():
    """套件目錄與 import 路徑維持 `hv_dccpp`。"""
    import hv_dccpp
    assert (ROOT / "hv_dccpp").is_dir()
    assert Path(hv_dccpp.__file__).parent.name == "hv_dccpp"
    assert ADAPTER.is_file()


def test_vendor_doc_records_the_naming_split():
    """VENDOR.md 必須寫著這個分層，否則下一個人不會知道為什麼不能改。"""
    txt = (ROOT / "docs" / "hv_dccpp" / "VENDOR.md").read_text(encoding="utf-8")
    assert "HARVEST" in txt and "hv_dccpp" in txt
    assert "名稱對照" in txt, "VENDOR.md 沒有名稱對照段落"


def test_boundary_key_set_has_not_drifted():
    """轉接層裡的 `hv_*` 名字集合必須維持已知的那四個。

    ⚠ 這條紅了**不一定是壞事**：它可能只代表上游同步進了新版轉接層。
    但它一定代表「AeroPlan 這一側要重新確認有沒有人送這些鍵」——
    這正是我們要被提醒的時刻。
    """
    found = set(re.findall(r"\bhv_[a-z_0-9]+\b", ADAPTER.read_text(encoding="utf-8")))
    assert found == FROZEN_BOUNDARY_KEYS, (
        f"轉接層的 hv_* 名字集合變了：多了 {sorted(found - FROZEN_BOUNDARY_KEYS)}、"
        f"少了 {sorted(FROZEN_BOUNDARY_KEYS - found)}。請重新確認 AeroPlan 這一側送的鍵。")


# ── 值層級：`hv_class` 真的是載重的那個鍵 ────────────────────────────────

_HOME = (23.95, 120.90)
_AREAS = [{"area_id": 1, "priority": 1.0,
           "polygon": [(23.9470, 120.8930), (23.9470, 120.9070),
                       (23.9530, 120.9070), (23.9530, 120.8930)]}]


def _classes(class_key: str):
    """用 `class_key` 當構型鍵建一份輸入，回傳 {uav_id: 構型字母}。"""
    from hv_dccpp.integration.aeroplan_adapter import to_mission_inputs
    drones = [{"drone_id": 1, "name": "u1", class_key: "A", "position": _HOME,
               "speed": 21.5, "turn_radius": 200.0, "vehicle_type": "vtol"},
              {"drone_id": 2, "name": "u2", class_key: "B", "position": _HOME,
               "speed": 19.0, "turn_radius": 120.0, "vehicle_type": "vtol"}]
    ins, _m = to_mission_inputs(_AREAS, drones, coverage_width_m=150.0,
                                overlap_rate=0.30, coverage_altitude=120.0,
                                home=_HOME, hfov_deg=60.0, vfov_deg=45.0)
    return {u.uav_id: u.spec.cls for u in ins.fleet}


def test_hv_class_key_is_load_bearing():
    """⭐ 前置斷言：用 `hv_class` 時構型真的照它走（一 A 一 B）。"""
    assert set(_classes("hv_class").values()) == {"A", "B"}


def test_renaming_the_class_key_silently_loses_the_class():
    """⭐⭐ 刪除實驗：把鍵改名 ⇒ 轉接層讀不到，**不會報錯**，構型靜靜退回預設。

    這條就是那個靜默失敗的證據。它通過表示「改名確實會壞掉」，
    所以上面 `test_boundary_key_set_has_not_drifted` 那道鎖是有意義的。

    ⚠ 注意 `default_class_of` 還會從 `name` 猜構型，所以刻意把名字取成
    `u1`/`u2`（不含「字母後面緊接數字」的 A/B 樣式）讓它猜不到。
    """
    got = _classes("harvest_class")
    assert set(got.values()) == {"A"}, (
        f"改名之後居然還讀得到構型（{got}）—— 那表示這條刪除實驗沒有測到東西，"
        f"邊界鎖的前提要重新檢查")


def test_our_side_still_sends_the_boundary_keys():
    """AeroPlan 這一側（情境／面板）送出的 drone dict 必須仍用 `hv_class`。"""
    from scripts.hv_sitl_scenarios import SCENARIOS
    sc = SCENARIOS["gui_legal_120_split_resched"]
    for d in sc.drones():
        assert "hv_class" in d, f"情境送出的 drone dict 少了 hv_class：{sorted(d)}"
