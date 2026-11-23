"""SITL 參數讀回閘門的值層級守衛（`scripts/hv_sitl_run.expected_params`）。

⚠⚠⚠ 2026-09-14 實際缺陷。舊版的閘門印出「51/51 相符」而且**兩個缺陷同時存在**：

1. **涵蓋率**：比對的是一份寫死的白名單 ``CHECK_PARAMS``（17 個名字），
   而匯出的 ``plan/<uav>.param`` 有 **32** 個 ⇒ 17 個從來沒被比對過，其中包含
   ``WP_RADIUS``（影響航跡）、``FENCE_*``、``FS_LONG_*``、``BATT_FS_*``（失效保護）、
   ``Q_LAND_FINAL_ALT``（``NAV_VTOL_LAND`` 的 param7=0 時由它決定最終進場高度）。
   而 51 = 17 × 3 台 ⇒ 「51/51」看起來像全數通過，其實是 53% 的涵蓋率。

2. **不可見**：``want = 檔案值 | EXPECT_EXTRA`` 讓常數蓋掉檔案值。
   ``Q_RTL_MODE`` 匯出 **1**、飛機實際 **0**，而 CSV 只寫
   ``expected=0.0, actual=0.0, ok=True, note=""`` ⇒ 這個分歧在產物裡看不見。

   ⚠ 訂正：那個覆寫**本身是對的**（ArduPlane 4.8.0-dev 的 PreArm 不接受
   ``RTL_AUTOLAND`` 與 ``Q_RTL_MODE`` 同時開，而任務含 ``DO_LAND_START``），
   缺陷在於它**不可見**，不在於它錯。所以修法是讓它現形，不是讓它失敗。

修好之後實測（HV4SITL16）：``102/102 相符（34 個參數 × 3 台；匯出檔 32 個全部涵蓋）``，
且 ``actual`` 沒有任何一列是空的 ⇒ 這版韌體認得全部 32 個匯出參數名。

為什麼鎖的是**值**不是源碼
--------------------------
只斷言「那一行長什麼樣」的鎖，改上游、改下游、繞過去都免疫。這一檔一律斷言
:func:`expected_params` 的**回傳內容**：涵蓋了哪些名字、期望值取自哪裡、note 寫了什麼。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

hv_sitl_run = pytest.importorskip("hv_sitl_run")

expected_params = hv_sitl_run.expected_params
EXPECT_FALLBACK = hv_sitl_run.EXPECT_FALLBACK
FIRMWARE_OVERRIDE = hv_sitl_run.FIRMWARE_OVERRIDE

# 一份**代表性**的匯出檔內容：含 EXPECT_FALLBACK 也有的鍵（Q_ENABLE/RTL_AUTOLAND）、
# 含刻意覆寫的鍵（Q_RTL_MODE），以及舊白名單漏掉的那幾個。
SAMPLE_FILE: dict[str, float] = {
    "AIRSPEED_CRUISE": 21.5, "AIRSPEED_MIN": 18.0, "AIRSPEED_MAX": 35.0,
    "Q_ENABLE": 1.0, "RTL_AUTOLAND": 2.0, "Q_RTL_MODE": 1.0, "Q_RTL_ALT": 100.0,
    "WP_RADIUS": 14.0, "Q_LAND_FINAL_ALT": 10.0, "FENCE_ACTION": 1.0,
    "FS_LONG_ACTN": 0.0, "BATT_FS_LOW_ACT": 1.0, "TERRAIN_FOLLOW": 0.0,
}


# ---------------------------------------------------------------------------
# 涵蓋率：匯出檔的每一個都要進比對集合
# ---------------------------------------------------------------------------

def test_every_exported_parameter_is_compared() -> None:
    """這一條直接鎖住舊版那個 17/32 的白名單缺陷。"""
    got = expected_params(SAMPLE_FILE)
    missing = sorted(set(SAMPLE_FILE) - set(got))
    assert not missing, f"匯出檔有參數沒被比對：{missing}"


def test_fallback_only_names_are_also_compared() -> None:
    """不在匯出檔、但由 default_params/identity 設定的那些也要驗，否則改名會靜默。"""
    got = expected_params(SAMPLE_FILE)
    for name in EXPECT_FALLBACK:
        assert name in got, f"{name} 在 EXPECT_FALLBACK 卻沒進比對集合"


def test_the_comparison_set_is_exactly_the_union() -> None:
    """不多也不少：多出來的名字代表憑空冒出期望值，少的代表漏驗。"""
    got = expected_params(SAMPLE_FILE)
    assert set(got) == set(SAMPLE_FILE) | set(EXPECT_FALLBACK)


# ---------------------------------------------------------------------------
# 優先序：檔案值必須贏過常數
# ---------------------------------------------------------------------------

def test_file_value_wins_over_fallback_constant() -> None:
    """舊版聯集寫反了（`檔案值 | EXPECT_EXTRA`）⇒ 常數蓋掉檔案值。

    這裡用一個**與常數不同**的檔案值來測，否則兩者相等時這條是空語句。
    """
    shared = sorted(set(SAMPLE_FILE) & set(EXPECT_FALLBACK) - set(FIRMWARE_OVERRIDE))
    assert shared, "樣本沒有同時出現在匯出檔與 EXPECT_FALLBACK 的鍵 ⇒ 這條測不到東西"
    name = shared[0]
    distinct = EXPECT_FALLBACK[name] + 7.0            # 保證與常數不同
    got = expected_params({**SAMPLE_FILE, name: distinct})
    exp, src, _note = got[name]
    assert exp == pytest.approx(distinct), (
        f"{name} 的期望值取到常數 {EXPECT_FALLBACK[name]} 而不是檔案值 {distinct}"
        f" ⇒ 聯集順序寫反了")
    assert src == "匯出檔"


def test_fallback_is_used_only_when_absent_from_file() -> None:
    without = {k: v for k, v in SAMPLE_FILE.items() if k not in EXPECT_FALLBACK}
    got = expected_params(without)
    for name, const in EXPECT_FALLBACK.items():
        exp, src, note = got[name]
        if name in FIRMWARE_OVERRIDE:
            continue
        assert exp == pytest.approx(const)
        assert src == "EXPECT_FALLBACK"
        assert note, f"{name} 用了常數備援卻沒有在 note 說明"


# ---------------------------------------------------------------------------
# 刻意覆寫必須現形
# ---------------------------------------------------------------------------

def test_deliberate_override_is_visible_with_its_reason() -> None:
    """覆寫可以存在，但**被蓋掉的匯出值**與**理由**都必須出現在 note。"""
    assert FIRMWARE_OVERRIDE, "沒有任何覆寫 ⇒ 下面的斷言是空語句"
    got = expected_params(SAMPLE_FILE)
    for name, (ov, why) in FIRMWARE_OVERRIDE.items():
        if name not in SAMPLE_FILE:
            continue
        exp, src, note = got[name]
        assert exp == pytest.approx(ov)
        assert src == "FIRMWARE_OVERRIDE", f"{name} 被覆寫卻標成 {src} ⇒ 分歧看不出來"
        assert note, f"{name} 的 note 是空的 ⇒ 這正是 2026-09-14 修掉的缺陷"
        assert f"{SAMPLE_FILE[name]:g}" in note, (
            f"{name} 的 note 沒有帶出被蓋掉的匯出值 {SAMPLE_FILE[name]:g}")
        assert why[:20] in note, f"{name} 的 note 沒有帶出理由"


def test_override_without_a_reason_is_rejected() -> None:
    """沒有理由的覆寫＝缺陷本身，必須大聲停下來而不是靜靜生效。"""
    orig = dict(FIRMWARE_OVERRIDE)
    FIRMWARE_OVERRIDE["AIRSPEED_CRUISE"] = (9.0, "   ")
    try:
        with pytest.raises(SystemExit):
            expected_params(SAMPLE_FILE)
    finally:
        FIRMWARE_OVERRIDE.clear()
        FIRMWARE_OVERRIDE.update(orig)


def test_overrides_only_cover_names_that_are_actually_exported() -> None:
    """覆寫一個根本沒匯出的名字＝憑空生出期望值，沒有東西會發現它是死的。"""
    for name in FIRMWARE_OVERRIDE:
        assert name in SAMPLE_FILE, (
            f"FIRMWARE_OVERRIDE 有 {name}，但代表性匯出檔沒有它 ——"
            f" 若真的不再匯出，這筆覆寫已經是死碼，請刪掉或更新樣本")
