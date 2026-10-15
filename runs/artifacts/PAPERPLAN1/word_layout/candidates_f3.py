"""F3：圖3 圖說縮為兩行（caveat 保留），以及 D2 文字的說明。由 candidates_extra 匯入後註冊。"""
from __future__ import annotations

import re

import paper_model as pm
from candidates import CANDIDATES

# 圖內已有分組標籤（單變因對照／歷史對照）與圖例（規劃值（上界）／SITL 飛行值／平面近似值），
# 圖說只保留 caveat：單變因對照、未設風之歷史對照、2 s 網格上界、WGS84 連續時間最小值、平面近似值。
FIG3_CAP_SHORT = (
    "圖3 高度合規與 SITL 飛行間距（(a) HV4W55R／HV4W55 為單變因對照，HV4SITL15／16 為 120.0 m 天花板、"
    "模擬未設風之歷史對照；(b) 規劃值為 2 s 網格上界，SITL 飛行值為 WGS84 橢球連續時間最小值，× 為既有平面近似值）")


def c_cap_short(bl):
    ks = [k for k, b in enumerate(bl) if b["kind"] == "fcap"
          and "高度合規與 SITL 飛行間距" in pm.text_of(b["runs"])]
    assert len(ks) == 1
    n = re.match(r"圖(\d)", pm.text_of(bl[ks[0]]["runs"])).group(1)
    bl[ks[0]]["runs"] = pm.inline(FIG3_CAP_SHORT.replace("圖3", f"圖{n}", 1))
    return bl


def register(order: list) -> None:
    CANDIDATES["F3"] = {"fn": c_cap_short, "kind": "改寫", "title": "圖3 圖說縮為兩行",
                        "lost": "無 caveat；(a)(b) 之量名改由圖內標題、分組標籤與圖例交代",
                        "text": FIG3_CAP_SHORT}
    if "F3" not in order:
        order.insert(order.index("F2") + 1, "F3")


# ── R3：參考文獻 [14] 刪網站名稱「ArduPilot Plane Documentation, 」（URL 與存取日保留）──
def c_ref14(bl):
    for b in bl:
        if b["kind"] == "ref" and pm.text_of(b["runs"]).startswith("[14]"):
            t = pm.text_of(b["runs"]).replace("ArduPilot Plane Documentation, ", "")
            b["runs"] = [(t, {})]
    return bl


def register_r3(order: list) -> None:
    CANDIDATES["R3"] = {"fn": c_ref14, "kind": "內容", "title": "文獻 [14] 刪網站名稱",
                        "lost": "「ArduPilot Plane Documentation」字樣（URL、存取日保留）",
                        "text": '[14] ArduPilot Dev Team, "QuadPlane Overview", '
                                'https://ardupilot.org/plane/docs/quadplane-overview.html, accessed September 14, 2026.'}
    if "R3" not in order:
        order.append("R3")
