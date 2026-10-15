# -*- coding: utf-8 -*-
"""把飛控下載回來的任務轉成「畫得出來」的形狀。

輸入是 `SITLLink.mission_downloaded` 送出的那種 item dict（見
`mission/sitl_link.py::_recv_mission`）：飛控給什麼就是什麼，沒有經過任何
列舉驗證，所以這裡的每個函式都必須**假設 command 可能是它沒看過的數字**。

═══ 為什麼這是獨立模組而不是寫在 main_window 裡 ═══
因為 2D 那條路的失效是**靜默且全域**的：

  * `ui/widgets/map_widget.py:253-292` 把 path 原封不動交給
    `folium.PolyLine(locations=path)` / `Marker(location=path[0])`，
    完全沒有截斷。
  * 對實際安裝的 folium 0.20.0 實測：三元組會丟
    `ValueError: Expected two (lat, lon) values for location,
     instead got: (25.0, 121.0, 50.0)`。
  * 而 `_render_map()` 用**一個** try/except 包住整個 `_build_map()`
    （map_widget.py:542-584），例外在 `web_view.setUrl(...)` **之前**中止 ⇒
    角點、多邊形、NFZ、圍籬、home 點**全部一起消失**，WebView 停在上一頁，
    唯一症狀是一行 log。

一個高度欄就能讓整張地圖靜靜消失。所以「剝掉高度」這件事要有自己的名字、
自己的測試，而不是散落在呼叫端的 list comprehension 裡 —— repo 裡已經有一份
只保護蜂群分支的 `_ll`（map_widget.py:394-400），這就是它沒有被重用的下場。
"""
from typing import Any, Dict, List, Sequence, Tuple

__all__ = [
    'NAV_CMD_MIN', 'NAV_CMD_MAX', 'LOITER_COMMANDS',
    'is_navigational', 'mission_to_path_2d', 'mission_to_path_3d',
    'mission_home', 'mission_loiters', 'mission_summary',
]

# MAVLink 規範：MAV_CMD_NAV_* 佔 16..95（MAV_CMD_NAV_LAST = 95）。
# 只有這個區間的指令帶真實的 x/y/z；DO_*(176+) 與 CONDITION_*(112+) 的
# 座標欄是參數，不是位置。
#
# ⚠️ 用**區間**判斷而不是白名單，是刻意的：白名單漏掉一個 NAV 指令，
#    航線就會少一段而且不會有人發現（少一段仍然畫得出來）。區間的失效
#    方向相反 —— 寧可多畫一個點，也不要少畫一段。
NAV_CMD_MIN = 16
NAV_CMD_MAX = 95

# param3 = 盤旋半徑（公尺，帶號：負 = 逆時針）的指令。
#   17 NAV_LOITER_UNLIM / 18 NAV_LOITER_TURNS / 19 NAV_LOITER_TIME
#   31 NAV_LOITER_TO_ALT
LOITER_COMMANDS = frozenset({17, 18, 19, 31})

# 經緯度都在這個值以內就視為「沒有座標」。飛控對純指令項回 0/0，
# 而 (0, 0) 在幾內亞灣外海 —— 畫出去會把整張地圖的縮放拉爛。
_ZERO_EPS = 1e-9


def is_navigational(item: Dict[str, Any]) -> bool:
    """這一項有沒有真實的 lat/lon 可以畫。"""
    cmd = int(item.get('command', 0))
    if not (NAV_CMD_MIN <= cmd <= NAV_CMD_MAX):
        return False
    lat = float(item.get('lat', 0.0))
    lon = float(item.get('lon', 0.0))
    return abs(lat) > _ZERO_EPS or abs(lon) > _ZERO_EPS


def mission_to_path_2d(items: Sequence[Dict[str, Any]],
                       skip_home: bool = True) -> List[Tuple[float, float]]:
    """→ `[(lat, lon), ...]`，**保證是二元組**。給 folium 那條 2D 路用。

    Args:
        items: 下載回來的 item dict 序列。
        skip_home: 跳過 seq 0。ArduPilot 的 seq 0 永遠是 home（它會用自己的
            home 覆蓋掉上傳時放在那裡的任何東西），把它接進航線會讓路線從
            起飛點拉一條假的長線出去。

    ⚠️ 回傳值刻意是 tuple 不是 list：`[lat, lon]` 這種兩元素 list 在
       folium 那邊也能用，於是「忘了剝高度」的 bug 會晚一步才炸，
       而且炸在別人的呼叫端。固定形狀讓型別檢查與測試抓得到。
    """
    out: List[Tuple[float, float]] = []
    for it in items:
        if skip_home and int(it.get('seq', -1)) == 0:
            continue
        if not is_navigational(it):
            continue
        out.append((float(it['lat']), float(it['lon'])))
    return out


def mission_to_path_3d(items: Sequence[Dict[str, Any]],
                       skip_home: bool = True) -> List[Tuple[float, float, float]]:
    """→ `[(lat, lon, alt), ...]`。給 Cesium 那條 3D 路用。

    ⚠️ **不可以**把這個結果交給 `map_widget.display_path/display_paths`
       —— 見本模組 docstring。要 2D 請用 `mission_to_path_2d`。
    """
    out: List[Tuple[float, float, float]] = []
    for it in items:
        if skip_home and int(it.get('seq', -1)) == 0:
            continue
        if not is_navigational(it):
            continue
        out.append((float(it['lat']), float(it['lon']), float(it.get('alt', 0.0))))
    return out


def mission_home(items: Sequence[Dict[str, Any]]):
    """seq 0（飛控的 home）→ `{'lat','lon','alt'}`，沒有就 None。

    ArduPilot 的 seq 0 **永遠**是 home：不管上傳時放了什麼，飛控都會用自己的
    home 覆蓋掉。所以每個 GCS（MP / QGC）都把它畫成獨立的 H 標記、
    **但不接進航線折線** —— 任務本體從 seq 1 開始。

    ⚠️ 2026-08-26 實測（-I5，真 ArduPilot，尚未定位）：seq 0 回的是
       `lat=0 lon=0 alt=0 frame=0` —— **不是**一個真的座標。假飛控在那裡
       放了真值，所以只靠單元測試看不到這個形狀。回 None 而不是 (0,0)：
       (0,0) 在幾內亞灣外海，畫出去會把整張地圖的縮放拉爛。
    """
    for it in items:
        if int(it.get('seq', -1)) != 0:
            continue
        lat = float(it.get('lat', 0.0))
        lon = float(it.get('lon', 0.0))
        if abs(lat) <= _ZERO_EPS and abs(lon) <= _ZERO_EPS:
            return None
        return {'lat': lat, 'lon': lon, 'alt': float(it.get('alt', 0.0))}
    return None


def mission_loiters(items: Sequence[Dict[str, Any]]) -> List[Dict[str, float]]:
    """抽出盤旋圈 → `[{'lat','lon','radius_m','alt','seq'}, ...]`。

    這是 AUTO 架構裡**最想看到的一樣東西**：攻擊發起點的決斷圈。
    `tot_node` 上傳的 `NAV_LOITER_UNLIM` 把半徑放在 param3
    （見 `scripts/wsl/sar_swarm_patches/mission_plan.py::build_strike_mission`）。

    半徑取絕對值 —— param3 帶號只表示旋轉方向，畫圈時負半徑會讓
    Cesium 的 ellipse 直接不顯示（而且不會報錯）。
    """
    out: List[Dict[str, float]] = []
    for it in items:
        if int(it.get('command', 0)) not in LOITER_COMMANDS:
            continue
        if not is_navigational(it):
            continue
        out.append({
            'seq':      int(it.get('seq', -1)),
            'lat':      float(it['lat']),
            'lon':      float(it['lon']),
            'alt':      float(it.get('alt', 0.0)),
            'radius_m': abs(float(it.get('param3', 0.0))),
        })
    return out


def mission_summary(items: Sequence[Dict[str, Any]]) -> str:
    """一行給狀態列的摘要。

    ⚠️ 明確區分「總項數」與「畫得出來的點數」。只報一個數字的話，
       一份全是 DO_* 的任務會顯示「12 個航點」但地圖上什麼都沒有，
       而使用者會去查地圖 —— 查錯地方。
    """
    n = len(items)
    if n == 0:
        return '飛控目前沒有任務（0 項）'
    drawable = len(mission_to_path_2d(items))
    rings = len(mission_loiters(items))
    parts = [f'{n} 項', f'可繪製 {drawable} 點']
    if rings:
        parts.append(f'盤旋圈 {rings}')
    skipped = n - drawable - (1 if any(int(i.get('seq', -1)) == 0 for i in items) else 0)
    if skipped > 0:
        parts.append(f'非導航 {skipped}')
    return '任務下載完成：' + '、'.join(parts)
