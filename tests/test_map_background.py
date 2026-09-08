"""2D 地圖黑底：三道防線都要在。

成因：全域 QSS 的第一條是 `QWidget { background: {{BG_PRIMARY}} }`，
而 `BG_PRIMARY` 是 **#000000**。那條規則沒有排除 `QWebEngineView`，
所以 web view 這個 QWidget 被漆黑，網頁只要有任何一層透明（Leaflet 容器、
圖磚空隙、folium 的外層 div）黑色就透上來，整張圖像蒙了一層黑。

三道防線各自獨立，任何一道被拿掉都可能復發，所以三條都鎖。
"""
from __future__ import annotations

import inspect
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def test_bg_primary_really_is_black():
    """前置斷言：如果 BG_PRIMARY 不是純黑，這整組測試在防的東西就不存在了。"""
    from ui.resources.aeroplan_theme.tokens import BG_PRIMARY
    assert BG_PRIMARY.lower() in ("#000000", "#000"), BG_PRIMARY


def test_global_qss_paints_every_qwidget():
    """前置斷言：通用 QWidget 規則確實存在（成因的另一半）。"""
    qss = (ROOT / "ui/resources/aeroplan_theme/aeroplan_theme.qss").read_text(
        encoding="utf-8")
    assert "QWidget {" in qss and "background: {{BG_PRIMARY}}" in qss


def test_qss_exempts_the_web_view():
    """防線一：QSS 要把 QWebEngineView 從通用規則豁免。"""
    qss = (ROOT / "ui/resources/aeroplan_theme/aeroplan_theme.qss").read_text(
        encoding="utf-8")
    assert "QWebEngineView {" in qss, "web view 沒有被豁免 ⇒ 會被漆成 #000000"
    block = qss.split("QWebEngineView {", 1)[1].split("}", 1)[0]
    assert "background" in block and "#000" not in block, block


def test_page_background_is_pinned_opaque():
    """防線二：網頁本身的底色要釘成不透明白色。"""
    src = inspect.getsource(
        __import__("ui.widgets.map_widget", fromlist=["MapWidget"]).MapWidget)
    assert "setBackgroundColor(QColor(255, 255, 255))" in src, \
        "沒有釘 page 底色 ⇒ 圖磚還沒載完的地方會露出 Qt 的黑"


def test_html_pins_body_and_leaflet_background():
    """防線三：HTML 自己也要有不透明底色（body 與 .leaflet-container）。"""
    src = (ROOT / "ui/widgets/map_widget.py").read_text(encoding="utf-8")
    assert "background:#ffffff; }}" in src, "body 沒釘底色"
    assert ".leaflet-container {{ background:#ffffff !important; }}" in src, \
        "Leaflet 容器沒釘底色（它預設 #ddd，會讓黑透上來）"
