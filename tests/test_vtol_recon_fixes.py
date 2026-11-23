"""VTOL 偵察頁 2026-08-25 實跑後修掉的五個缺陷 —— 值層級鎖。

這五條都是在 App 內把整條鏈跑過一遍（圈選→規劃→驗算→熱力圖→匯出→起 SITL→
上傳飛控）之後量到的，不是靜態讀出來的：

  1. 生成點間距被設成**剛好等於**避撞門檻，再被兩個不同的每度公尺常數
     （111320.0 vs EARTH_RADIUS_M 的 111319.4908）推到門檻底下 ⇒ t=0 每一組
     相鄰機都被誤判成衝突。
  2. `_resolve_altitude` 先無條件記動作、之後才對 TAKEOFF/LANDING 跳過，而 t=0
     的衝突正好都在那些地面點上 ⇒ `run()` 的停滯保險絲永遠燒不斷 ⇒ 8 輪跑滿，
     每輪 +30 m 全堆到作業航點（實測把 120 m 推到 360 m）。
  3. 上傳飛控讀的是**基本演算法**分頁的 speed/turn_radius，不是這份規劃用的值。
  4. 匯出檔的 DO_CHANGE_SPEED param1=1（**地速**）而上傳是 0（空速），型別不一致；
     且原始碼註解把 1 寫成「空速」。
  5. `_get_cesium_widget()` 不管現在顯示 2D 還 3D 都回傳物件 ⇒
     `if cesium is None: 請切換到 3D` 是死碼，熱力圖畫進看不見的圖台還報成功。

每一條都配一個「這個設定真的能讓它發生」的對照組，避免變成永遠不會紅的儀表。
"""

import math
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from core.dccpp.collision_avoidance import CollisionAvoidance, _haversine
from core.dccpp.dccpp_path_builder import (
    BuiltPath, BuiltWaypoint, DCCPPPathBuilder, SegmentLabel,
)
from mission.vtol_mission_exporter import (
    VTOLMissionExporter, _SPEED_TYPE_AIRSPEED, _SPEED_TYPE_GROUNDSPEED,
)

try:
    from ui.main_window import (            # noqa: F401  先載入 QtWebEngine
        MainWindow, resolve_spawn_spacing_m, spawn_positions,
        resolve_upload_speed_radius, SPAWN_SPACING_MARGIN,
    )
    from ui.widgets.dual_map_widget import DualMapWidget
    _QT_OK = True
except Exception as _e:                     # pragma: no cover
    _QT_OK = False
    _QT_ERR = _e

qt_only = pytest.mark.skipif(
    not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}"
)

HOME_LAT, HOME_LON = 25.077923, 121.231959

#: VTOL 偵察頁 get_params() 實際會送出的東西（實跑那趟量到的）：
#: 有 min_separation_m，**沒有** takeoff_spacing、也沒有 alt_offset_m。
VTOL_PANEL_PARAMS = {
    'num_drones': 3,
    'min_separation_m': 50.0,
    'collision_avoidance': True,
    'altitude': 120.0,
    'speed': 18.0,
    'turn_radius': 60.0,
}


def _legacy_spawn_positions(base_lat, base_lon, n, bearing_deg, spacing_m):
    """修好之前的算法：手刻 111320.0 m/deg。當對照組用，證明測試真的會咬。"""
    out = []
    for i in range(n):
        offset_m = (i - (n - 1) / 2.0) * spacing_m
        perp = math.radians((bearing_deg + 90.0) % 360.0)
        dN = offset_m * math.cos(perp)
        dE = offset_m * math.sin(perp)
        cos_lat = max(math.cos(math.radians(base_lat)), 1e-6)
        out.append((base_lat + dN / 111320.0,
                    base_lon + dE / (111320.0 * cos_lat)))
    return out


def _adjacent_gaps(pts):
    return [_haversine(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1])
            for i in range(len(pts) - 1)]


# ═════════════════════════════════════════════════════════════════════
#  修正 1：生成點間距 —— 同一個地球模型，而且要高過門檻
# ═════════════════════════════════════════════════════════════════════
@qt_only
def test_spacing_defaults_above_separation_threshold():
    """面板沒送 takeoff_spacing 時，退回值必須高過 min_separation_m。"""
    spacing = resolve_spawn_spacing_m(VTOL_PANEL_PARAMS)
    assert spacing > VTOL_PANEL_PARAMS['min_separation_m']
    assert spacing == pytest.approx(50.0 * SPAWN_SPACING_MARGIN)


@qt_only
def test_measured_spacing_clears_threshold_on_panel_defaults():
    """用面板預設排出來的生成點，**量出來**要 >= 門檻（這才是避撞真正看的量）。"""
    spacing = resolve_spawn_spacing_m(VTOL_PANEL_PARAMS)
    pts = spawn_positions(HOME_LAT, HOME_LON, 3, 0.0, spacing)
    gaps = _adjacent_gaps(pts)
    assert len(gaps) == 2
    for g in gaps:
        assert g >= VTOL_PANEL_PARAMS['min_separation_m'], gaps


@qt_only
def test_legacy_arithmetic_would_have_failed():
    """對照組：舊算法 + 舊退回值真的會落在門檻底下，所以上面那條測得到東西。"""
    pts = _legacy_spawn_positions(HOME_LAT, HOME_LON, 3, 0.0, 50.0)
    gaps = _adjacent_gaps(pts)
    assert all(g < 50.0 for g in gaps), gaps
    assert gaps[0] == pytest.approx(49.999771, abs=1e-5)


@qt_only
def test_requested_spacing_is_what_gets_measured():
    """要 D 公尺就要量到 D 公尺 —— 排點與量距必須用同一個地球模型。"""
    for d in (50.0, 60.0, 137.5, 500.0):
        pts = spawn_positions(HOME_LAT, HOME_LON, 2, 0.0, d)
        assert _adjacent_gaps(pts)[0] == pytest.approx(d, rel=1e-9)


@qt_only
@pytest.mark.parametrize('lat', [0.0, 23.7, 25.077923, 45.0, 60.0])
def test_spacing_holds_at_every_latitude(lat):
    """cos φ 會約掉 ⇒ 舊 bug 與緯度無關；修法也必須與緯度無關。"""
    spacing = resolve_spawn_spacing_m(VTOL_PANEL_PARAMS)
    pts = spawn_positions(lat, HOME_LON, 3, 0.0, spacing)
    for g in _adjacent_gaps(pts):
        assert g >= VTOL_PANEL_PARAMS['min_separation_m']


@qt_only
@pytest.mark.parametrize('bearing', [0.0, 45.0, 90.0, 217.0, 359.0])
def test_spacing_holds_at_every_takeoff_bearing(bearing):
    spacing = resolve_spawn_spacing_m(VTOL_PANEL_PARAMS)
    pts = spawn_positions(HOME_LAT, HOME_LON, 4, bearing, spacing)
    for g in _adjacent_gaps(pts):
        assert g >= VTOL_PANEL_PARAMS['min_separation_m']


@qt_only
def test_explicit_too_small_spacing_is_raised():
    """使用者明講的間距若壓在門檻上，要被放大（而不是安靜地產生假衝突）。"""
    p = dict(VTOL_PANEL_PARAMS, takeoff_spacing=50.0)
    assert resolve_spawn_spacing_m(p) > 50.0


@qt_only
def test_explicit_generous_spacing_is_respected():
    """對照組：夠大的間距不可以被偷改。"""
    p = dict(VTOL_PANEL_PARAMS, takeoff_spacing=250.0)
    assert resolve_spawn_spacing_m(p) == pytest.approx(250.0)


@qt_only
def test_single_drone_stays_at_base():
    assert spawn_positions(HOME_LAT, HOME_LON, 1, 0.0, 60.0) == [(HOME_LAT, HOME_LON)]


# ═════════════════════════════════════════════════════════════════════
#  修正 2：停滯保險絲要燒得斷
# ═════════════════════════════════════════════════════════════════════
def _ground_only_paths():
    """衝突只落在起降段 —— 那些點被守衛保護，高度錯開對它們無能為力。"""
    out = {}
    for uid in (1, 2):
        out[uid] = BuiltPath(uav_id=uid, waypoints=[
            BuiltWaypoint(lat=HOME_LAT, lon=HOME_LON, alt=0.0,
                          heading_compass_deg=0.0, segment_type=SegmentLabel.TAKEOFF),
            BuiltWaypoint(lat=HOME_LAT, lon=HOME_LON, alt=40.0,
                          heading_compass_deg=0.0, segment_type=SegmentLabel.TAKEOFF),
            BuiltWaypoint(lat=HOME_LAT, lon=HOME_LON, alt=0.0,
                          heading_compass_deg=0.0, segment_type=SegmentLabel.LANDING),
        ])
    return out


def _operation_paths():
    """一般作業段衝突 —— 高度錯開真的動得了。"""
    out = {}
    for uid in (1, 2):
        out[uid] = BuiltPath(uav_id=uid, waypoints=[
            BuiltWaypoint(lat=HOME_LAT + i * 0.0009, lon=HOME_LON, alt=120.0,
                          heading_compass_deg=0.0, segment_type=SegmentLabel.OPERATION)
            for i in range(6)
        ])
    return out


def _realistic_paths():
    """實跑那趟的形狀：同一條航跡上既有起降地面點、也有作業航點。

    t=0 的衝突在**地面點**（守衛保護、動不了），但偏移窗會延伸到後面的
    **作業航點** —— 舊行為就是把 +30 m 一輪一輪堆到那些作業點上，
    把 120 m 的偵察航高推到 360 m。純起降段的夾具測不到這件事。
    """
    out = {}
    for uid in (1, 2):
        wps = [
            BuiltWaypoint(lat=HOME_LAT, lon=HOME_LON, alt=0.0,
                          heading_compass_deg=0.0, segment_type=SegmentLabel.TAKEOFF),
            BuiltWaypoint(lat=HOME_LAT, lon=HOME_LON, alt=40.0,
                          heading_compass_deg=0.0, segment_type=SegmentLabel.TAKEOFF),
        ]
        wps += [
            BuiltWaypoint(lat=HOME_LAT + 0.002 + i * 0.0009,
                          lon=HOME_LON + 0.004 * uid, alt=120.0,
                          heading_compass_deg=0.0, segment_type=SegmentLabel.OPERATION)
            for i in range(5)
        ]
        wps.append(BuiltWaypoint(lat=HOME_LAT, lon=HOME_LON, alt=0.0,
                                 heading_compass_deg=0.0,
                                 segment_type=SegmentLabel.LANDING))
        out[uid] = BuiltPath(uav_id=uid, waypoints=wps)
    return out


def _operation_alts(paths):
    return [w.alt for p in paths.values() for w in p.waypoints
            if getattr(w.segment_type, 'name', '') == 'OPERATION']


def _ca(**kw):
    kw.setdefault('min_separation_m', 50.0)
    kw.setdefault('alt_offset_m', 30.0)
    kw.setdefault('time_step_s', 1.0)
    kw.setdefault('cruise_speed_mps', 18.0)
    kw.setdefault('strategy', 'altitude')
    kw.setdefault('max_rounds', 8)
    return CollisionAvoidance(**kw)


def test_unmovable_conflict_reports_no_actions():
    """動不了的衝突不可以回報成「已執行避撞動作」。"""
    paths = _ground_only_paths()
    conflicts, actions = _ca().run(paths)
    assert conflicts, '這個夾具本來就該偵測到衝突，否則下面什麼都沒測到'
    assert actions == []


def test_unmovable_conflict_does_not_stack_altitude():
    """保險絲燒斷 ⇒ 不會跑滿 8 輪，高度一點都不該被抬。"""
    paths = _ground_only_paths()
    before = [w.alt for p in paths.values() for w in p.waypoints]
    _ca().run(paths)
    after = [w.alt for p in paths.values() for w in p.waypoints]
    assert after == before


def test_movable_conflict_still_reports_actions():
    """對照組：真的動得了的衝突仍要回報動作，否則上面兩條是假綠燈。"""
    paths = _operation_paths()
    conflicts, actions = _ca().run(paths)
    assert conflicts
    assert len(actions) > 0


def test_movable_conflict_still_changes_altitude():
    paths = _operation_paths()
    before = max(w.alt for p in paths.values() for w in p.waypoints)
    _ca().run(paths)
    after = max(w.alt for p in paths.values() for w in p.waypoints)
    assert after > before


@pytest.mark.parametrize('rounds', [1, 2, 4, 8])
def test_unmovable_conflict_is_round_count_independent(rounds):
    """關鍵不變式：解不開時，高度**不隨輪數增加**。舊行為是每輪再加一次。"""
    paths = _ground_only_paths()
    _ca(max_rounds=rounds).run(paths)
    assert max(w.alt for p in paths.values() for w in p.waypoints) == pytest.approx(40.0)


def test_recon_altitude_is_not_inflated_by_unmovable_conflicts():
    """⭐ 這條才是實跑那趟真正壞掉的東西：偵察航高被推到 3 倍。

    衝突在動不了的地面點上，但偏移窗會蓋到作業航點。修好之後作業航高
    必須**原封不動**停在 120 m。
    """
    paths = _realistic_paths()
    conflicts, _ = _ca().run(paths)
    assert conflicts, '夾具沒產生衝突就什麼都沒測到'
    assert _operation_alts(paths) == pytest.approx([120.0] * 10)


@pytest.mark.parametrize('rounds', [1, 4, 8])
def test_recon_altitude_does_not_grow_with_rounds(rounds):
    """舊行為下高度隨輪數線性增加（8 輪 = +240 m）。鎖死這個關係。"""
    paths = _realistic_paths()
    _ca(max_rounds=rounds).run(paths)
    assert max(_operation_alts(paths)) == pytest.approx(120.0)


def test_realistic_fixture_really_conflicts_on_the_ground():
    """對照組：確認這個夾具的衝突真的落在起降段，否則上面兩條測的是別的東西。"""
    paths = _realistic_paths()
    conflicts = _ca().detect(paths)
    assert conflicts
    names = set()
    for cf in conflicts:
        for uid, idx in ((cf.uav_a, cf.wp_idx_a), (cf.uav_b, cf.wp_idx_b)):
            seg = paths[uid].waypoints[idx].segment_type
            names.add(getattr(seg, 'name', ''))
    assert names & {'TAKEOFF', 'LANDING'}, names


def _ramp_reaches_ground_paths():
    """衝突在**作業段第一點**（動得了），偏移窗的全偏移區會蓋到起飛的地面點。

    這是唯一能驗到「起降段守衛」的形狀：
      * 衝突若本身在地面點，上游就整個跳過了，守衛根本不會被執行到；
      * 衝突若落在後段，地面點只會落進漸升段的**起點**，那裡 frac=0、
        加的是 +0 —— 拿掉守衛也看不出差別。
    所以兩機的起飛點要對稱分開、到作業段第一點的距離相同，讓它們同時抵達，
    衝突就會落在 index 2，全偏移區 [s,e] 才會蓋住 index 0/1。
    """
    out = {}
    for uid, sign in ((1, -1.0), (2, +1.0)):
        wps = [
            # 起飛點左右對稱分開（t=0 不衝突），但到 index 2 的距離一樣
            BuiltWaypoint(lat=HOME_LAT, lon=HOME_LON + sign * 0.005, alt=0.0,
                          heading_compass_deg=0.0, segment_type=SegmentLabel.TAKEOFF),
            BuiltWaypoint(lat=HOME_LAT, lon=HOME_LON + sign * 0.005, alt=40.0,
                          heading_compass_deg=0.0, segment_type=SegmentLabel.TAKEOFF),
        ]
        # 兩機在作業段完全重疊 ⇒ 最接近點就在作業段第一點
        wps += [
            BuiltWaypoint(lat=HOME_LAT + 0.0009 * i, lon=HOME_LON, alt=120.0,
                          heading_compass_deg=0.0, segment_type=SegmentLabel.OPERATION)
            for i in range(4)
        ]
        out[uid] = BuiltPath(uav_id=uid, waypoints=wps)
    return out


def test_ramp_fixture_really_produces_a_movable_conflict():
    """對照組：先確認這個夾具真的有**動得了**的衝突，否則守衛照樣不會被執行到。"""
    paths = _ramp_reaches_ground_paths()
    conflicts, actions = _ca().run(paths)
    assert conflicts, '夾具沒衝突'
    assert actions, '衝突沒動作 ⇒ 偏移迴圈沒跑 ⇒ 守衛沒被執行到'


def test_ramp_window_really_covers_a_takeoff_waypoint():
    """對照組：確認偏移窗的**全偏移區**真的蓋到起飛段航點。

    沒有這條，下面那條可能只是在測一個永遠不會被碰到的點 —— 漸升段的起點
    frac=0、加的是 +0，拿掉守衛也看不出任何差別。
    """
    paths = _ramp_reaches_ground_paths()
    covered = False
    for cf in _ca().detect(paths):
        tgt = max(cf.uav_a, cf.uav_b)
        ti = cf.wp_idx_a if tgt == cf.uav_a else cf.wp_idx_b
        lo, hi = max(0, ti - 2), min(len(paths[tgt].waypoints) - 1, ti + 2)
        for i in range(lo, hi + 1):
            seg = getattr(paths[tgt].waypoints[i].segment_type, 'name', '')
            if seg in ('TAKEOFF', 'LANDING'):
                covered = True
    assert covered, '偏移窗沒蓋到任何起降段航點 ⇒ 守衛不會被執行到'


def test_takeoff_waypoint_survives_a_covering_offset_window():
    """偏移窗蓋住起飛段航點時，那些點的高度必須紋風不動。

    起降段被抬高 = 「在半空中起飛、在半空中降落」，是安全問題不是顯示問題。
    """
    paths = _ramp_reaches_ground_paths()

    def _takeoff_alts(ps):
        return {uid: [w.alt for w in bp.waypoints
                      if getattr(w.segment_type, 'name', '') == 'TAKEOFF']
                for uid, bp in ps.items()}

    before = _takeoff_alts(paths)
    _ca().run(paths)
    after = _takeoff_alts(paths)
    assert after == before, f'起飛段高度被動到了：{before} -> {after}'


def test_ground_waypoints_are_never_lifted():
    """地面點被抬高 = 「在半空中降落」，這是安全問題。"""
    paths = _ground_only_paths()
    _ca().run(paths)
    for p in paths.values():
        assert p.waypoints[0].alt == pytest.approx(0.0)
        assert p.waypoints[-1].alt == pytest.approx(0.0)


# ═════════════════════════════════════════════════════════════════════
#  修正 3：上傳要用「這份規劃當時用的」值
# ═════════════════════════════════════════════════════════════════════
@qt_only
def test_upload_prefers_planned_values_over_panel():
    """實跑那趟的數字：面板停在初始預設 3.0/50，規劃用的是 18/60。"""
    panel = {'speed': 3.0, 'turn_radius': 50.0}
    planned = {'cruise_speed_mps': 18.0, 'turn_radius_m': 60.0}
    assert resolve_upload_speed_radius(planned, panel) == (18.0, 60.0)


@qt_only
def test_upload_falls_back_to_panel_when_not_planned():
    """對照組：沒有規劃值時才可以用面板值。"""
    panel = {'speed': 12.0, 'turn_radius': 80.0}
    assert resolve_upload_speed_radius({}, panel) == (12.0, 80.0)
    assert resolve_upload_speed_radius(None, panel) == (12.0, 80.0)


@qt_only
def test_upload_ignores_zero_planned_values():
    """0 / None 不算「有規劃值」，要退回面板，不可以送出 0 m/s。"""
    panel = {'speed': 12.0, 'turn_radius': 80.0}
    planned = {'cruise_speed_mps': 0.0, 'turn_radius_m': None}
    assert resolve_upload_speed_radius(planned, panel) == (12.0, 80.0)


@qt_only
def test_planner_records_the_keys_the_uploader_reads():
    """⚠ 源碼層級妥協：規劃端在 MainWindow 的巨大方法裡，沒有可呼叫的接縫。
    這裡只確認寫入端與讀取端用的是**同一組鍵名**——改一邊沒改另一邊要紅。
    """
    from pathlib import Path
    import ui.main_window as mw
    src = Path(mw.__file__).read_text(encoding='utf-8')
    assert "dccpp_result['cruise_speed_mps']" in src
    assert "dccpp_result['turn_radius_m']" in src


# ═════════════════════════════════════════════════════════════════════
#  修正 4：DO_CHANGE_SPEED 型別統一成空速
# ═════════════════════════════════════════════════════════════════════
def test_speed_type_constants_match_mavlink():
    """MAVLink common.xml `enum SPEED_TYPE`：0=Airspeed、1=Groundspeed。"""
    assert _SPEED_TYPE_AIRSPEED == 0
    assert _SPEED_TYPE_GROUNDSPEED == 1


def _vtol_path():
    path = BuiltPath(uav_id=1, waypoints=[
        BuiltWaypoint(lat=HOME_LAT + 0.006, lon=HOME_LON - 0.011, alt=120.0,
                      heading_compass_deg=90.0, segment_type=SegmentLabel.OPERATION),
        BuiltWaypoint(lat=HOME_LAT + 0.008, lon=HOME_LON + 0.018, alt=120.0,
                      heading_compass_deg=90.0, segment_type=SegmentLabel.OPERATION),
    ])
    b = DCCPPPathBuilder(min_turn_radius=100.0, default_altitude=120.0)
    b.prepend_vtol_takeoff(path, HOME_LAT, HOME_LON, 0.0, 60.0, 18.0)
    b.append_vtol_landing(path, HOME_LAT, HOME_LON, 180.0, 60.0, 18.0)
    return path


def _change_speed_item(cruise=18.0):
    lines = VTOLMissionExporter(transition_alt_m=60.0, cruise_speed_ms=cruise,
                                turn_radius_m=100.0).generate_waypoint_lines(
        _vtol_path().waypoints, altitude=120.0, uav_id=1)
    for ln in lines[1:]:
        f = ln.rstrip('\n').split('\t')
        if int(f[3]) == 178:
            return {'p1': float(f[4]), 'p2': float(f[5]), 'p3': float(f[6])}
    raise AssertionError('匯出檔裡找不到 DO_CHANGE_SPEED')


def test_exported_speed_type_is_airspeed():
    assert _change_speed_item()['p1'] == pytest.approx(float(_SPEED_TYPE_AIRSPEED))


def test_exported_speed_type_is_not_groundspeed():
    """舊值是 1（地速），註解卻寫「空速」—— 鎖住不許回去。"""
    assert _change_speed_item()['p1'] != pytest.approx(float(_SPEED_TYPE_GROUNDSPEED))


def test_exported_speed_value_is_the_cruise_speed():
    """對照組：改巡航速度，p2 要跟著動（證明這條測得到東西）。"""
    assert _change_speed_item(18.0)['p2'] == pytest.approx(18.0)
    assert _change_speed_item(24.0)['p2'] == pytest.approx(24.0)


def test_exported_throttle_is_unchanged():
    assert _change_speed_item()['p3'] == pytest.approx(-1.0)


# ═════════════════════════════════════════════════════════════════════
#  修正 5：3D 圖台守衛要真的擋得住
# ═════════════════════════════════════════════════════════════════════
class _FakeDualMap:
    """只帶 ensure_3d / is_3d 真正會碰到的狀態。

    ⚠ 不建真的 DualMapWidget：它會把 QtWebEngine（Leaflet + Cesium）整個拉起來，
    在無頭環境會卡住。這裡用未綁定方法跑**同一份函式本體**，測的仍是產品程式碼。
    """

    def __init__(self, mode):
        self._mode = mode
        self.switch_calls = []

    def _switch_mode(self, mode):
        self.switch_calls.append(mode)
        self._mode = mode


class _StuckIn2D(_FakeDualMap):
    """切不過去的情況（例如 3D 圖台建不起來）。"""

    def _switch_mode(self, mode):
        self.switch_calls.append(mode)     # 收到請求，但模式沒變


@qt_only
def test_dual_map_default_mode_is_2d():
    """這是 bug 的前提：開機顯示 2D。

    ⚠ 源碼層級：要拿到這個預設值就得建真的 widget（會拉起 QtWebEngine 而卡住），
    所以只鎖 __init__ 裡那一行。下面的行為測試才是值層級的。
    """
    from pathlib import Path
    import ui.widgets.dual_map_widget as dm
    src = Path(dm.__file__).read_text(encoding='utf-8')
    assert 'self._mode = _MODE_2D' in src


@qt_only
def test_ensure_3d_switches_from_2d():
    from ui.widgets.dual_map_widget import DualMapWidget, _MODE_2D, _MODE_3D
    w = _FakeDualMap(_MODE_2D)
    assert DualMapWidget.is_3d(w) is False
    assert DualMapWidget.ensure_3d(w) is True
    assert w.switch_calls == [_MODE_3D]
    assert DualMapWidget.is_3d(w) is True


@qt_only
def test_ensure_3d_does_not_switch_when_already_3d():
    """對照組：已經在 3D 就不該再切一次（切換會重載圖台）。"""
    from ui.widgets.dual_map_widget import DualMapWidget, _MODE_3D
    w = _FakeDualMap(_MODE_3D)
    assert DualMapWidget.ensure_3d(w) is True
    assert w.switch_calls == []


@qt_only
def test_ensure_3d_reports_failure_when_switch_does_not_take():
    """切不過去要回 False —— 呼叫端才有機會擋下來，而不是畫進看不見的圖台。"""
    from ui.widgets.dual_map_widget import DualMapWidget, _MODE_2D, _MODE_3D
    w = _StuckIn2D(_MODE_2D)
    assert DualMapWidget.ensure_3d(w) is False
    assert w.switch_calls == [_MODE_3D], '該試著切過去，只是沒成功'


@qt_only
def test_visible_cesium_guard_blocks_when_switch_fails(monkeypatch):
    """切不過去就必須回 None（並提示），不可以讓呼叫端畫進看不見的圖台。"""
    import ui.main_window as mw

    shown = []
    monkeypatch.setattr(mw.QMessageBox, 'information',
                        lambda *a, **k: shown.append(a[-1]))

    class _Map:
        def ensure_3d(self):
            return False

    class _Fake:
        map_widget = _Map()

        def _get_cesium_widget(self):
            return object()

    got = mw.MainWindow._get_visible_cesium_widget(_Fake(), "呈現熱力圖")
    assert got is None
    assert shown, '擋下來卻沒告訴使用者，等於另一種靜默失敗'
    assert '呈現熱力圖' in shown[0]


@qt_only
def test_visible_cesium_guard_passes_when_3d_available(monkeypatch):
    """對照組：切得過去就要回傳 widget，而且不可以彈提示。"""
    import ui.main_window as mw

    shown = []
    monkeypatch.setattr(mw.QMessageBox, 'information',
                        lambda *a, **k: shown.append(a[-1]))
    sentinel = object()

    class _Map:
        def ensure_3d(self):
            return True

    class _Fake:
        map_widget = _Map()

        def _get_cesium_widget(self):
            return sentinel

    assert mw.MainWindow._get_visible_cesium_widget(_Fake(), "呈現熱力圖") is sentinel
    assert shown == []


@qt_only
def test_heatmap_and_missed_use_the_visible_guard():
    """⚠ 源碼層級妥協：兩個 handler 在 MainWindow 裡，沒有便宜的呼叫接縫。
    至少鎖住它們用的是會擋的那個函式，而不是不會擋的 _get_cesium_widget()。
    """
    from pathlib import Path
    import ui.main_window as mw
    src = Path(mw.__file__).read_text(encoding='utf-8')
    assert src.count('_get_visible_cesium_widget("呈現熱力圖")') == 1
    assert src.count('_get_visible_cesium_widget("標示漏掃格")') == 1


# ═════════════════════════════════════════════════════════════════════
#  2026-08-25 追加：3D 顯示 — 繭型移除 + VTOL 專屬模型
# ═════════════════════════════════════════════════════════════════════
def _cesium_html():
    from pathlib import Path
    return (Path(__file__).resolve().parents[1] /
            'ui' / 'widgets' / 'cesium_templates' / 'main.html'
            ).read_text(encoding='utf-8')


@qt_only
def test_transition_cocoons_are_gone():
    """使用者要求移除 FW/MC 轉換點的閃爍繭型橢球（每架 ×2，六顆很吵）。"""
    html = _cesium_html()
    assert '_addVTOLTransitionMarker' not in html
    assert "modeName + ' 轉換'" not in html


@qt_only
def test_vtol_model_channel_is_wired():
    """VTOL 要用自己的 vtol.glb，不再借固定翼的模型。"""
    html = _cesium_html()
    assert '_VTOL_MODEL_URI' in html
    assert 'isVtolGlb' in html
    # VTOL 有專屬檔時優先；沒有才退回固定翼（isPlaneGlb 要排除 isVtolGlb）
    assert '!isVtolGlb && isPlane' in html


@qt_only
def test_vtol_template_vars_all_substituted():
    """widget 端必須把每一個 __VTOL_*__ 樣板變數都代入 —— 漏一個，
    JS 端就是 SyntaxError，整張 3D 圖台白屏。"""
    from pathlib import Path
    import ui.widgets.cesium_map_widget as cmw
    src = Path(cmw.__file__).read_text(encoding='utf-8')
    html = _cesium_html()
    import re
    for var in sorted(set(re.findall(r'__VTOL_[A-Z_]+__', html))):
        assert f"'{var}'" in src, f'{var} 在 widget 端沒有 .replace 代入'


@qt_only
def test_vtol_glb_exists_and_is_the_chosen_model():
    """使用者 2026-08-25 確認：VTOL 模型 = P25-ID.glb（第一輪選的 900 STEP
    渲染出來是六軸旋翼機，不是 VTOL）。鎖住檔案存在且內容一致 ——
    換檔要連 cesium_map_widget 的校正常數一起改。"""
    import hashlib
    from pathlib import Path
    res = Path(__file__).resolve().parents[1] / 'ui' / 'resources'
    vtol = res / 'vtol.glb'
    assert vtol.exists(), 'vtol.glb 不在 ⇒ VTOL 會退回固定翼模型'
    h = lambda p: hashlib.md5(p.read_bytes()).hexdigest()
    assert h(vtol) == h(res / 'P25-ID.glb'), \
        'vtol.glb 內容變了 —— 校正常數 (scale/offsets) 綁定 P25-ID，要一起重校'


@qt_only
def test_quadplane_telemetry_reports_vtol():
    """QuadPlane 的 MAV_TYPE=1 光看 heartbeat 分不出 VTOL；連線層要用
    vehicle_hint / Q_ENABLE 把它升格，3D 圖台才選得到 vtol.glb。"""
    from mission.sitl_link import SITLLink
    lk = SITLLink.__new__(SITLLink)
    lk._vehicle_hint, lk._q_enable_confirmed = 'VTOL', False
    assert lk._effective_vehicle_type(1) == 'VTOL'
    lk._vehicle_hint, lk._q_enable_confirmed = '', True
    assert lk._effective_vehicle_type(1) == 'VTOL'


@qt_only
def test_pure_plane_and_copter_are_not_mislabeled():
    """對照組：沒有任何 VTOL 證據時，1 還是 PLANE、2 還是 COPTER。"""
    from mission.sitl_link import SITLLink
    lk = SITLLink.__new__(SITLLink)
    lk._vehicle_hint, lk._q_enable_confirmed = '', False
    assert lk._effective_vehicle_type(1) == 'PLANE'
    assert lk._effective_vehicle_type(2) == 'COPTER'


# ═════════════════════════════════════════════════════════════════════
#  2026-08-25：2D 群飛圖例把整張地圖變黑（black background）
# ═════════════════════════════════════════════════════════════════════
@qt_only
def test_swarm_legend_cannot_stretch_fullscreen():
    """群飛圖例是 position:fixed 且只設 bottom/right。它用 folium.Element 直接
    掛在 <body>，而 Leaflet/branca 的全域樣式會把 left/top 也解析成非 auto ⇒
    一個 fixed 且 left+right（與 top+bottom）都有值的元素會**撐滿視窗**，
    於是右下角小圖例變成蓋住全圖的 88% 不透明黑幕。

    修法是顯式釘 left/top/width/height = auto。這條鎖住那四個 auto 都在，
    以及有 max-width/max-height 收邊 —— 少任何一個都可能再度全螢幕變黑。
    """
    from ui.widgets.map_widget import MapWidget
    sd = {'drones': [{'drone_id': 1}, {'drone_id': 2}],
          'stats': {'total_drones': 2, 'total_waypoints': 30,
                    'total_distance': 1234.0, 'estimated_time': 5.0}}
    w = MapWidget.__new__(MapWidget)
    html = MapWidget._build_swarm_legend(w, sd)
    outer = html.split('>', 1)[0]  # 第一個 div 的開標籤
    for must in ('left:auto', 'top:auto', 'width:auto', 'height:auto'):
        assert must in outer, f'圖例外框缺 {must} ⇒ 可能被撐滿變黑幕：{outer!r}'
    assert 'max-width' in outer and 'max-height' in outer
    assert 'position:fixed' in outer  # 仍是浮動角落框


@qt_only
def test_swarm_legend_still_shows_content():
    """對照組：修版面別把內容弄不見了 —— 圖例標題與統計仍要在。"""
    from ui.widgets.map_widget import MapWidget
    sd = {'drones': [{'drone_id': 1}],
          'stats': {'total_drones': 1, 'total_waypoints': 15,
                    'total_distance': 500.0, 'estimated_time': 2.0}}
    w = MapWidget.__new__(MapWidget)
    html = MapWidget._build_swarm_legend(w, sd)
    assert '群飛路徑圖例' in html
