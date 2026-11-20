# -*- coding: utf-8 -*-
"""KPI 的「不通過的關鍵項」必須涵蓋 ok 的每一個條件。

═══ 為什麼要鎖 ═══
`stot_repeat.sh` 的 `ok` 由**七個**條件相乘,但「不通過的關鍵項」清單原本只有
三個分支(機數 / 首航次 / rc)。後果不是「少講一句」,而是**指錯方向**:

KIL3(2026-08-06,N=2)實測 —— 真正沒過的是「誤差 5.25 > 5.0 m」,
印出來的卻是「觸地 2/6」:
  * 分母寫死 6(該檔自己的註解就寫著「分母必須是本趟的機數」),兩機航次恆為假陽性
  * 而真因(誤差超標)在清單裡**根本沒有對應分支**

診斷訊息指錯方向比沒有訊息更貴 —— 它會讓下一輪去修一個不存在的問題。
"""
import pathlib
import re

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = (ROOT / "scripts" / "wsl" / "stot_repeat.sh").read_text(encoding="utf-8")


def _why_block():
    m = re.search(r"if not ok:\s*\n(.*?)\n(?=\S|\Z)", SRC, re.S)
    assert m, "找不到 `if not ok:` 區塊"
    return m.group(1)


class TestDenominatorIsNotHardcoded:
    def test_no_hardcoded_six_in_touchdown_display(self):
        """`觸地 %d/6` 這種寫法在兩機航次是恆假的假陽性。"""
        assert "觸地 %d/6" not in SRC, "觸地 顯示仍寫死 6"

    def test_uses_want_n(self):
        assert '_want_n = int(os.environ.get("SWARM_COUNT"' in SRC
        assert "n_t != _want_n" in SRC, "守衛仍在比 6 而不是 _want_n"


class TestReasonCoversEveryCondition:
    """ok 的每一個條件都要有對應的 _why 分支。"""

    def test_miss_distance(self):
        assert re.search(r"mx\s*>\s*miss_req", _why_block()), "誤差超標沒有對應分支"

    def test_spread(self):
        assert re.search(r"spread\s*>\s*3\.0", _why_block()), "散布超標沒有對應分支"

    def test_hit_altitude(self):
        assert re.search(r"alt\s*>\s*30\.0", _why_block()), "命中高度超標沒有對應分支"

    def test_ground_speed(self):
        assert re.search(r"gmean\s*<\s*vmin_req", _why_block()), "地速不足沒有對應分支"

    def test_count_and_first_pass_and_rc_still_there(self):
        b = _why_block()
        assert "n_t != _want_n" in b
        assert "first_pass_ok" in b
        assert 'trial_rc != "0"' in b

    def test_missing_value_branches(self):
        """缺值和超標要分開講 —— `mx is None` 與 `mx > miss_req` 的處置不同。"""
        b = _why_block()
        assert "mx is None" in b
        assert "alt is None" in b
        assert "gmean is None" in b
