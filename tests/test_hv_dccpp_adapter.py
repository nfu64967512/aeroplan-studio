"""tests/test_hv_dccpp_adapter.py — HV-DCCPP（異構雙構型 VTOL）接縫測試。

被測的是 **AeroPlan 這一側的接線**，不是 hv_dccpp 本身（那邊自己有 809 項測試）：

  A. `SwarmCoordinator.plan_coverage_dccpp` 的 hv 分支：預設關閉、打開後回傳同形的
     6 鍵字典、下游 duck typing 欄位齊全、`dem_margin_m` 沒有退回會產生 55% nan 的預設值。
  B. `ui/main_window.py` 的閘門與 `ui/workers/planning_worker.py` 的背景執行緒。

⚠ 這裡**不用** `tests/test_dccpp_pipeline.py` 的 golden 當基準：hv 全程走 WGS84 橢球
（AeroPlan 部分模組用 111111.0 m/deg，差約 0.1%），而且 `total_makespan` 的語意不同
（hv 含起降、轉換、盤旋與錯開起飛，必然較大）。本檔用自己的容忍值。

寫法遵循本專案的「值層級鎖」：斷言規劃器**輸出的數字**，而且每條旗標測試都先證明
「這個設定真的改變得了答案」，避免出現一條永遠不可能變紅的測試。
"""
from __future__ import annotations

import ast
import inspect
import math
import os
from pathlib import Path
from types import MethodType as types_MethodType

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from mission.swarm_coordinator import SwarmCoordinator  # noqa: E402

# ⚠ 不要在模組層級 importorskip：這個檔裡有一半的測試（閘門、背景執行緒、
#   以及**既有 DCCPP 顯示路徑**的回歸）根本不需要 hv_dccpp。整檔一起 skip 的話，
#   沒裝 hv 的環境會連「預設路徑有沒有被切壞」都不再被檢查，而畫面上是一片綠。
try:
    from hv_dccpp.integration import aeroplan_adapter as hv_adapter
    _HV_OK = True
except ImportError:                                  # pragma: no cover - 環境相依
    hv_adapter = None
    _HV_OK = False

hv_only = pytest.mark.skipif(
    not _HV_OK, reason='hv_dccpp 未安裝；見 docs/AEROPLAN_接入手冊.md §2')

SIX_KEYS = {'results', 'allocation', 'coverage_paths', 'uav_states',
            'total_makespan', 'assembled_paths'}
SEGMENT_NAMES = {'TAKEOFF', 'ENTRY', 'OPERATION', 'TRANSFER', 'LANDING'}

HOME = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME[0]))


def _ll(east_m: float, north_m: float) -> tuple[float, float]:
    return (HOME[0] + north_m / _M_LAT, HOME[1] + east_m / _M_LON)


def _areas() -> list[dict]:
    """3 km × 1.2 km 作業區，形狀同 main_window 組出來的 areas。"""
    return [{'area_id': 1, 'priority': 1.0, 'scan_angle_deg': 0.0,
             'polygon': [_ll(4000, -600), _ll(7000, -600),
                         _ll(7000, 600), _ll(4000, 600)]}]


def _drones() -> list[dict]:
    """兩台，欄位與 main_window._on_dccpp_coverage_requested 組出來的完全一致。

    ★ 刻意不放任何構型線索（沒有 hv_class / payload / platform），
      因為 UI 真的組不出那些欄位 —— 構型只能靠 class_of 回呼決定。
    """
    return [
        {'drone_id': 1, 'name': 'UAV-1', 'position': HOME, 'speed': 21.5,
         'heading': 0.0, 'turn_radius': 200.0, 'vehicle_type': 'vtol',
         'fov_width': 160.0},
        {'drone_id': 2, 'name': 'UAV-2', 'position': HOME, 'speed': 19.0,
         'heading': 0.0, 'turn_radius': 120.0, 'vehicle_type': 'vtol',
         'fov_width': 160.0},
    ]


def _plan_kwargs() -> dict:
    return dict(areas=_areas(), drones=_drones(), coverage_width_m=160.0,
                overlap_rate=0.25, coverage_altitude=160.0)


def _hv_coordinator(**overrides) -> SwarmCoordinator:
    c = SwarmCoordinator()
    c.use_hv_dccpp = True
    c.hv_class_of = lambda d: 'A' if int(d['drone_id']) == 1 else 'B'
    c.hv_home = HOME
    # ⚠ 一定要明給豁免：沒有豁免時 AGL 上限就是法規的 120 m，而 Pioneer 的
    #   最低固定翼作業高度也是 120 ⇒ 可用高度帶 0 m ⇒ 整趟 preflight NO_GO。
    #   這些測試量的是別的東西，所以把空域條件固定在「已取得 200 m 豁免」。
    c.hv_params = {'min_separation_m': 100.0, 'alt_offset_m': 50.0,
                   'hv_waiver_max_agl_m': 200.0}
    for k, v in overrides.items():
        setattr(c, k, v)
    return c


@pytest.fixture(scope='module')
def hv_plan():
    """跑一次 hv 規劃（約 1.5 s），整個模組共用。"""
    coord = _hv_coordinator()
    result = coord.plan_coverage_dccpp(**_plan_kwargs())
    return coord, result


# ══════════════════════════════════════════════════════════════════════
#  A. 接縫：旗標、回傳形狀、下游欄位
# ══════════════════════════════════════════════════════════════════════
def test_flag_is_off_by_default():
    """預設不得走 hv —— 否則 test_dccpp_pipeline 的 golden 會整組變紅。"""
    c = SwarmCoordinator()
    assert c.use_hv_dccpp is False
    assert c.hv_coordinator is None
    assert c.hv_class_of is None


def test_gui_kwarg_vtol_transition_alt_m_is_accepted():
    """`ui/main_window.py` 每次規劃都會送這個關鍵字。

    它一度不在方法簽章上 ⇒ 每一次 GUI 規劃都丟 TypeError 進那個大 except，
    使用者只看到「DCCPP 失敗」。這條把「呼叫端與被呼叫端對得起來」鎖住。
    """
    params = inspect.signature(SwarmCoordinator.plan_coverage_dccpp).parameters
    assert 'vtol_transition_alt_m' in params
    # 可觸發性：真的呼叫得動（簽章對但實作炸掉一樣沒用）
    c = SwarmCoordinator()
    out = c.plan_coverage_dccpp(vtol_transition_alt_m=100.0, **_plan_kwargs())
    # ⚠ 既有 v2 管線會另外附上自己的鍵（例如 effective_scan_angles），
    #   所以這裡只要求「6 鍵都在」。HV 分支的**恰好 6 鍵**由
    #   test_hv_branch_returns_the_same_six_keys 負責。
    assert SIX_KEYS <= set(out)


@hv_only
def test_hv_branch_returns_the_same_six_keys(hv_plan):
    _coord, result = hv_plan
    assert set(result) == SIX_KEYS
    assert isinstance(result['total_makespan'], float)
    assert result['allocation'] == {1: [1, 2]}, '兩個構型都要被分配到工作'


@hv_only
def test_hv_branch_actually_changes_the_plan():
    """可觸發性前置斷言：旗標真的換掉了規劃器，不是一條惰性開關。

    比的是「同一組輸入、兩條路徑」的輸出數字。makespan 語意不同（hv 含起降與
    錯開起飛，必然較大），所以只斷言「顯著不同」而不是某個定值。
    """
    off = SwarmCoordinator().plan_coverage_dccpp(**_plan_kwargs())
    on_coord = _hv_coordinator()
    on = on_coord.plan_coverage_dccpp(**_plan_kwargs())
    assert on['total_makespan'] > off['total_makespan'] * 1.2
    assert on_coord.hv_coordinator is not None
    assert on_coord.hv_coordinator.last_result is on


@hv_only
def test_class_of_decides_the_two_platforms(hv_plan):
    """★ 不給 class_of 的話整隊會被當成構型 A，而且完全不會報錯。"""
    coord, _result = hv_plan
    assert coord.hv_coordinator.id_map == {1: 'A1', 2: 'B2'}

    # 刪除實驗：把回呼拿掉，兩台都會變成 A
    bare = SwarmCoordinator()
    bare.use_hv_dccpp = True
    bare.hv_home = HOME
    bare.hv_params = {'hv_waiver_max_agl_m': 200.0}   # 見 _hv_coordinator 的註解
    bare_result = bare.plan_coverage_dccpp(**_plan_kwargs())
    assert set(bare_result) == SIX_KEYS
    assert set(bare.hv_coordinator.id_map.values()) == {'A1', 'A2'}, (
        '沒有 class_of 時應退化成整隊構型 A —— 這正是必須提供回呼的理由')


@hv_only
def test_downstream_duck_typing(hv_plan):
    """`ui/main_window.py` 的後處理只靠 getattr 讀這些欄位。"""
    _coord, result = hv_plan
    for bp in result['assembled_paths'].values():
        lengths = [bp.takeoff_length_m, bp.entry_length_m, bp.operation_length_m,
                   bp.transfer_length_m, bp.landing_length_m]
        assert all(isinstance(x, float) for x in lengths)
        assert bp.total_length_m == pytest.approx(sum(lengths), rel=1e-9)
        assert {w.segment_type.name for w in bp.waypoints} <= SEGMENT_NAMES
        # 兩個航向欄位名都要在（live 讀 heading_compass_deg、legacy 讀 heading_deg）
        w0 = bp.waypoints[0]
        assert w0.heading_compass_deg == w0.heading_deg
        assert len(bp.to_latlon()) == len(bp.waypoints)
        assert len(bp.to_latlon_list()) == len(bp.waypoints)
        # 首航點是起飛點，alt 是「相對 home」而非 AMSL
        assert w0.alt == pytest.approx(0.0, abs=2.0)


@hv_only
def test_upload_tuples_start_with_speed_then_vtol_takeoff(hv_plan):
    """SITL 上傳必須走轉接層的 8-tuple，不能走 `_build_wps_for_path`。

    後者的 TAKEOFF 分支先 `if idx == 0: continue` 才檢查 takeoff_emitted，
    而 hv 縮點後的起飛段**只有一個航點且就在 idx 0** ⇒ 整份任務沒有起飛指令。
    """
    coord, result = hv_plan
    # 先證明觸發條件成立：起飛段真的只有一個航點，而且在索引 0
    for bp in result['assembled_paths'].values():
        idx = [i for i, w in enumerate(bp.waypoints)
               if w.segment_type.name == 'TAKEOFF']
        assert idx == [0], f'起飛段航點索引 {idx}，觸發條件不成立，本測試失去意義'

    tuples_by_drone = coord.hv_coordinator.upload_tuples(vtol_transition_alt_m=100.0)
    assert set(tuples_by_drone) == {1, 2}
    for wps in tuples_by_drone.values():
        cmds = [t[3] for t in wps]
        assert cmds[0] == 178, 'DO_CHANGE_SPEED'
        assert cmds[1] == 84, 'NAV_VTOL_TAKEOFF —— 沒有它飛控會從第一個航點直接開始'
        assert 85 in cmds, 'NAV_VTOL_LAND'
        assert all(len(t) == 8 for t in wps)


@hv_only
def test_dem_margin_is_3000_not_the_1500_default(monkeypatch):
    """`dem_margin_m` 若退回轉接層預設 1500，ENU 地形網格會有 55% 取樣成 nan。

    平地情境看不出差別（makespan 一樣），所以只能鎖傳進去的**值**本身。
    """
    seen = {}
    real_ctor = hv_adapter.HvDccppCoordinator

    class _Spy(real_ctor):
        def __init__(self, **kw):
            seen.update(kw)
            super().__init__(**kw)

    monkeypatch.setattr(hv_adapter, 'HvDccppCoordinator', _Spy)
    _hv_coordinator().plan_coverage_dccpp(**_plan_kwargs())
    assert seen['dem_margin_m'] == 3000.0
    # 順便鎖住另外兩個「不給就 NO_GO」的欄位有被傳下去
    assert seen['c2_range_km'] == 25.0
    assert seen['waiver_max_agl_m'] == 200.0


@hv_only
def test_ui_params_reach_the_planner_config(monkeypatch):
    """面板的 min_separation_m / alt_offset_m 是 25 鍵裡少數真的生效的兩個。"""
    seen = {}
    real_ctor = hv_adapter.HvDccppCoordinator

    class _Spy(real_ctor):
        def __init__(self, **kw):
            seen.update(kw)
            super().__init__(**kw)

    monkeypatch.setattr(hv_adapter, 'HvDccppCoordinator', _Spy)
    coord = _hv_coordinator()
    coord.hv_params = {'min_separation_m': 250.0, 'alt_offset_m': 80.0,
                       'hv_c2_range_km': 40.0, 'hv_waiver_max_agl_m': 200.0}
    coord.plan_coverage_dccpp(**_plan_kwargs())
    assert seen['cfg'].d_sep_m == 250.0
    assert seen['cfg'].dh_sep_m == 80.0
    assert seen['c2_range_km'] == 40.0
    # dem_step_m 不可退回 PlannerConfig 的 10.0（地形節點會變 6.25 倍）
    assert seen['cfg'].dem_step_m == 25.0


# ══════════════════════════════════════════════════════════════════════
#  B. UI 側：閘門、背景執行緒、同步讀回
# ══════════════════════════════════════════════════════════════════════
try:
    from ui.main_window import MainWindow          # 先匯入以載入 QtWebEngine
    from PyQt6.QtCore import QCoreApplication
    from PyQt6.QtWidgets import QApplication
    from ui.workers.planning_worker import HvPlanningWorker
    _QT_APP = QApplication.instance() or QApplication([])
    _QT_OK = True
except Exception as _e:      # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

qt_only = pytest.mark.skipif(not _QT_OK, reason='ui.main_window / PyQt6 匯入失敗')


class _FakeWindow:
    """給未綁定方法用的假 self（house style，見 test_dccpp_reroute_altitude.py）。"""

    def __init__(self, home=None):
        import types
        self.home_point = home
        self._hv_classes = {}
        self._hv_alternates = None
        # _configure_hv_dccpp 會用到這兩支，所以假 self 也要有
        self._hv_class_of = types.MethodType(MainWindow._hv_class_of, self)
        self._hv_fence_zones = types.MethodType(MainWindow._hv_fence_zones, self)


@qt_only
@pytest.mark.parametrize('vehicle_type,hv_flag,expected', [
    ('multirotor', True, False),      # hv 只支援 4+1 複合翼 VTOL
    ('fixed_wing', True, False),
    ('vtol', False, False),           # 面板沒勾選
    ('vtol', True, True),
])
def test_hv_gate(vehicle_type, hv_flag, expected):
    win = _FakeWindow(home=HOME)
    coord = SwarmCoordinator()
    params = {'hv_dccpp': hv_flag, 'num_drones': 2, 'hv_num_a': 1}
    got = MainWindow._configure_hv_dccpp(win, coord, params, vehicle_type)
    assert got is expected
    assert coord.use_hv_dccpp is expected


@qt_only
def test_hv_gate_builds_the_class_map():
    win = _FakeWindow(home=HOME)
    coord = SwarmCoordinator()
    params = {'hv_dccpp': True, 'num_drones': 5, 'hv_num_a': 2,
              'hv_wind': (6.0, 90.0)}
    assert MainWindow._configure_hv_dccpp(win, coord, params, 'vtol') is True
    assert win._hv_classes == {1: 'A', 2: 'A', 3: 'B', 4: 'B', 5: 'B'}
    assert coord.hv_wind == (6.0, 90.0)
    assert coord.hv_home == HOME
    # 禁飛區要在 GUI 執行緒取好再交出去（FenceZoneRegistry 是主執行緒的 QObject 單例）
    assert coord.hv_fence_zones is not None
    # 回呼真的照這張表回答
    assert MainWindow._hv_class_of(win, {'drone_id': 2}) == 'A'
    assert MainWindow._hv_class_of(win, {'drone_id': 3}) == 'B'
    # A 台數超過總台數時夾住，不會產生 B = 負數
    params['hv_num_a'] = 99
    MainWindow._configure_hv_dccpp(win, coord, params, 'vtol')
    assert set(win._hv_classes.values()) == {'A'}
    assert len(win._hv_classes) == 5


@qt_only
def test_worker_emits_result_on_success():
    """背景執行緒把 6 鍵字典送回主執行緒，progress / report 一併轉發。"""
    class _StubCoordinator:
        hv_progress = None
        hv_on_report = None

        def plan_coverage_dccpp(self, **kw):
            self.hv_progress(0.4, '規劃中')
            self.hv_on_report({'status': 'GO', 'warnings': [], 'uncovered': []})
            return {'total_makespan': 12.5, 'assembled_paths': {1: object()},
                    'results': {}, 'allocation': {}, 'coverage_paths': [],
                    'uav_states': []}

    seen = {'progress': [], 'report': [], 'ok': [], 'failed': []}
    w = HvPlanningWorker(_StubCoordinator(), {})
    w.progress.connect(lambda f, m: seen['progress'].append((f, m)))
    w.report.connect(lambda p: seen['report'].append(p))
    w.finished_ok.connect(lambda r: seen['ok'].append(r))
    w.failed.connect(lambda m: seen['failed'].append(m))
    w.start()
    assert w.wait(20000), 'worker 未在 20 s 內結束'
    QCoreApplication.processEvents()

    assert seen['failed'] == []
    assert len(seen['ok']) == 1
    assert seen['ok'][0]['total_makespan'] == 12.5
    assert seen['progress'] == [(0.4, '規劃中')]
    assert seen['report'][0]['status'] == 'GO'
    assert w.result is seen['ok'][0]


@qt_only
def test_worker_reports_failure_on_its_own_signal():
    """失敗必須走 `failed`，不可以用「成功但空結果」代表失敗。

    空的 assembled_paths 是一個合法答案（真的沒有可覆蓋區域），兩者混在同一條路上
    UI 就分不出「規劃失敗」與「沒有東西可飛」，而那兩者要做的事相反。
    """
    class _Boom:
        hv_progress = None
        hv_on_report = None

        def plan_coverage_dccpp(self, **kw):
            err = RuntimeError('preflight NO_GO')
            err.errors = ['c2_link_unresolved(A)', 'alternates_missing(A,B)']
            raise err

    seen = {'ok': [], 'failed': []}
    w = HvPlanningWorker(_Boom(), {})
    w.finished_ok.connect(lambda r: seen['ok'].append(r))
    w.failed.connect(lambda m: seen['failed'].append(m))
    w.start()
    assert w.wait(20000)
    QCoreApplication.processEvents()

    assert seen['ok'] == []
    assert len(seen['failed']) == 1
    assert 'preflight NO_GO' in seen['failed'][0]
    assert 'c2_link_unresolved(A)' in seen['failed'][0], '錯誤清單要一起給操作員'


@qt_only
def test_worker_clears_its_callbacks_from_the_coordinator():
    """執行緒結束後不該還留著 worker 的 bound method 在 coordinator 上。"""
    class _Stub:
        hv_progress = None
        hv_on_report = None

        def plan_coverage_dccpp(self, **kw):
            return {'results': {}, 'allocation': {}, 'coverage_paths': [],
                    'uav_states': [], 'total_makespan': 0.0, 'assembled_paths': {}}

    stub = _Stub()
    w = HvPlanningWorker(stub, {})
    w.start()
    assert w.wait(20000)
    assert stub.hv_progress is None and stub.hv_on_report is None


class _StubWidget:
    """記錄呼叫的假 widget（不建立真的 MainWindow：本機 QtWebEngine 一 setHtml 就 abort）。"""

    def __init__(self):
        self.calls = []
        self.text_value = ''

    def __getattr__(self, name):
        def _rec(*a, **k):
            self.calls.append((name, a, k))
            return self
        return _rec

    def setText(self, t):
        self.text_value = str(t)
        self.calls.append(('setText', (t,), {}))

    def text(self):
        return self.text_value


class _DisplayWindow:
    """跑 `_display_dccpp_result` 用的假 self —— 只提供它真的會碰的 11 個屬性。"""

    def __init__(self, home=None):
        self.home_point = home
        self.flight_params = {}
        self.map_widget = _StubWidget()
        self.vtol_recon_panel = _StubWidget()
        self.mission_panel = _StubWidget()
        self.waypoint_label = _StubWidget()
        self.distance_label = _StubWidget()
        self._status = _StubWidget()
        self._dccpp_result = None
        self._dccpp_metrics = None
        self.fence_called = []

    def statusBar(self):
        return self._status

    def _apply_fence_avoidance_to_dccpp_result(self, *a, **k):
        # 真的那支會碰 NFZ registry 與 Dubins fillet；這裡只記一筆
        self.fence_called.append((a, k))


@hv_only
@qt_only
def test_display_dccpp_result_consumes_a_real_hv_plan(hv_plan):
    """★ 最關鍵的一條：後處理是從 435 行的 handler 裡切出來的，

    切壞了不會有語法錯誤，只會在按下按鈕時丟進那個大 except 變成一句
    「DCCPP 失敗」。這裡拿**真的 hv 規劃結果**餵進去，斷言它一路跑到底：
    地圖畫了、統計更新了、`_dccpp_result` 與 `_dccpp_metrics` 都寫進去了。
    """
    _coord, result = hv_plan
    win = _DisplayWindow(home=HOME)
    corners = [p for p in _areas()[0]['polygon']]

    MainWindow._display_dccpp_result(
        win, result, params={'collision_avoidance': False, 'auto_landing': False,
                             'vehicle_type': 'vtol', 'transition_alt': 100.0},
        corners_snapshot=corners,
        centroid_lat=sum(c[0] for c in corners) / 4,
        centroid_lon=sum(c[1] for c in corners) / 4,
        altitude=160.0, speed=21.5, turn_radius=200.0,
        vehicle_type='vtol', auto_scan=True)

    assert win._dccpp_result is result, '結果沒有存回去 ⇒ 匯出／上傳／SITL 全部拿不到'
    assert win._dccpp_metrics is not None
    assert win._dccpp_metrics['num_drones'] == 2
    assert win._dccpp_metrics['total_waypoints'] > 0
    assert win._dccpp_metrics['total_distance'] > 0.0
    # 地圖真的被畫了
    drawn = [c for c in win.map_widget.calls if c[0] == 'display_swarm_raw']
    assert len(drawn) == 1, f'display_swarm_raw 呼叫 {len(drawn)} 次'
    swarm_data = drawn[0][1][0]
    assert len(swarm_data['drones']) == 2
    assert len(swarm_data['areas'][0]['polygon']) == 4
    # 權威值有蓋章（SITL 上傳與 spawn 都讀這幾個鍵）
    assert result['vehicle_type'] == 'vtol'
    assert result['vtol_transition_alt_m'] == 100.0
    # 統計面板
    assert win.waypoint_label.text_value
    assert win.distance_label.text_value


@qt_only
def test_display_still_consumes_a_legacy_plan():
    """後處理是**共用**的：既有 DCCPP（預設路徑）的結果也必須照跑。

    切出這支方法是為了讓背景執行緒完成後有東西可接，但預設路徑天天走它，
    只驗 hv 不驗舊的話，等於把主要路徑放生。
    """
    result = SwarmCoordinator().plan_coverage_dccpp(**_plan_kwargs())
    win = _DisplayWindow(home=HOME)
    corners = list(_areas()[0]['polygon'])
    MainWindow._display_dccpp_result(
        win, result, params={'collision_avoidance': False, 'auto_landing': False,
                             'vehicle_type': 'vtol'},
        corners_snapshot=corners,
        centroid_lat=sum(c[0] for c in corners) / 4,
        centroid_lon=sum(c[1] for c in corners) / 4,
        altitude=160.0, speed=21.5, turn_radius=200.0,
        vehicle_type='vtol', auto_scan=True)
    assert win._dccpp_result is result
    assert win._dccpp_metrics['num_drones'] == 2
    assert [c for c in win.map_widget.calls if c[0] == 'display_swarm_raw']


@hv_only
@qt_only
def test_display_uses_the_snapshot_not_live_corners(hv_plan):
    """可觸發性：畫出來的多邊形要跟著快照走，不是跟著使用者當下的邊界點。"""
    _coord, result = hv_plan
    win = _DisplayWindow(home=HOME)
    snapshot = [(23.0, 120.0), (23.0, 120.1), (23.1, 120.1), (23.1, 120.0), (23.05, 120.05)]
    MainWindow._display_dccpp_result(
        win, result, params={'collision_avoidance': False, 'auto_landing': False},
        corners_snapshot=snapshot, centroid_lat=23.05, centroid_lon=120.05,
        altitude=160.0, speed=21.5, turn_radius=200.0,
        vehicle_type='vtol', auto_scan=True)
    drawn = [c for c in win.map_widget.calls if c[0] == 'display_swarm_raw'][0]
    assert drawn[1][0]['areas'][0]['polygon'] == snapshot


class _JoinWindow(_DisplayWindow):
    """把 _start_hv_planning → worker → _on_hv_finished → _display_dccpp_result

    這條**真正的**接線跑一遍用的假 self。整個改動的重點就在這條線上，
    而它原本沒有任何測試覆蓋：worker 測試餵的是替身 coordinator，
    顯示測試是直接呼叫 _display_dccpp_result，兩者中間那一段沒人走過。
    """

    def __init__(self, home=None):
        import types
        super().__init__(home=home)
        for name in ('_hv_planning_in_flight', '_hv_disconnect_worker',
                     '_start_hv_planning', '_on_hv_progress', '_on_hv_report',
                     '_on_hv_failed', '_on_hv_cancelled', '_on_hv_finished',
                     '_hv_notify_done', '_hv_show_report_banner',
                     '_display_dccpp_result'):
            setattr(self, name, types.MethodType(getattr(MainWindow, name), self))
        self._HV_LOUD_NOTES = MainWindow._HV_LOUD_NOTES
        self._hv_worker = None
        self._hv_coordinator = None
        self._hv_display_ctx = {}
        self._hv_last_report = {}
        self._hv_on_done = None
        self.failures = []
        self._on_hv_failed = lambda msg: self.failures.append(msg)


class _RealishCoordinator:
    """行為像 SwarmCoordinator 的替身：回傳指定結果，並暴露 hv_coordinator。"""

    def __init__(self, result, hv_obj=None, ticks=((0.4, '規劃中'),)):
        self._result, self.hv_coordinator, self._ticks = result, hv_obj, ticks
        self.hv_progress = None
        self.hv_on_report = None
        self.seen_kwargs = None

    def plan_coverage_dccpp(self, **kw):
        self.seen_kwargs = kw
        for f, m in self._ticks:
            self.hv_progress(f, m)
        self.hv_on_report({'status': 'GO_WITH_WARNINGS', 'errors': [], 'warnings': [],
                           'notes': ['coverage_altitude_clamped(100.0->120.0)',
                                     'fov_width_mismatch(cls=A: 224 m)'],
                           'uncovered': [], 'timing_ms': {'total': 900.0}})
        return self._result


@qt_only
@hv_only
def test_background_path_reaches_the_display(hv_plan):
    """★ 背景規劃這條線（本次改動的全部理由）從頭到尾跑一次。"""
    _coord, result = hv_plan
    win = _JoinWindow(home=HOME)
    corners = list(_areas()[0]['polygon'])
    ctx = dict(params={'collision_avoidance': False, 'auto_landing': False,
                       'vehicle_type': 'vtol'},
               corners_snapshot=corners,
               centroid_lat=sum(c[0] for c in corners) / 4,
               centroid_lon=sum(c[1] for c in corners) / 4,
               altitude=160.0, speed=21.5, turn_radius=200.0,
               vehicle_type='vtol', auto_scan=True)
    hv_obj = object()
    coordinator = _RealishCoordinator(result, hv_obj=hv_obj)

    win._start_hv_planning(coordinator, _plan_kwargs(), ctx)
    worker = win._hv_worker
    assert worker is not None, 'worker 必須存成成員，否則會被 GC 收掉'
    assert worker.parent() is None, (
        'worker 不可以掛在視窗底下：關視窗時 Qt 會在還在跑的 QThread 上呼叫解構子 ⇒ abort')
    assert win._hv_planning_in_flight() or worker.isFinished()
    assert worker.wait(20000)
    for _ in range(50):
        QCoreApplication.processEvents()
        if win._dccpp_result is not None:
            break

    assert win.failures == [], f'不該失敗：{win.failures}'
    assert win._dccpp_result is result, 'finished_ok 沒有把結果送到 _display_dccpp_result'
    assert win._hv_coordinator is hv_obj, '匯出／上傳要用的轉接器沒有被存下來'
    assert win._dccpp_metrics['num_drones'] == 2
    assert [c for c in win.map_widget.calls if c[0] == 'display_swarm_raw']
    # plan_kwargs 真的原封不動送進 coordinator
    assert coordinator.seen_kwargs['coverage_altitude'] == 160.0
    # display_ctx 的鍵必須與 _display_dccpp_result 的簽章對得起來（改名就會爆）
    assert set(ctx) == set(inspect.signature(MainWindow._display_dccpp_result)
                           .parameters) - {'self', 'dccpp_result'}


@qt_only
def test_second_plan_is_refused_while_one_is_running():
    """連按兩次不可以同時跑兩個 solver（舊版是靠 UI 凍結擋住的）。"""
    import threading
    gate = threading.Event()
    empty = {'results': {}, 'allocation': {}, 'coverage_paths': [],
             'uav_states': [], 'total_makespan': 0.0, 'assembled_paths': {}}

    class _Slow(_RealishCoordinator):
        def plan_coverage_dccpp(self, **kw):
            gate.wait(20)
            return empty

    win = _JoinWindow(home=HOME)
    # ⚠ 假 self 不是 QObject ⇒ PyQt 把 slot 當 direct connection，會在 **worker 執行緒**
    #   裡跑；正式環境的 MainWindow 是 QObject，Qt 會排回主執行緒。這裡把顯示換掉，
    #   免得空的 display_ctx 在背景執行緒裡彈出 modal 而整組測試卡死。
    done = []
    win._on_hv_finished = lambda r: done.append(r)
    win._start_hv_planning(_Slow(empty), {}, {})
    first = win._hv_worker
    try:
        assert win._hv_planning_in_flight()
        # 第二次請求：被擋下，而且**不可以**換掉正在跑的 worker
        win._start_hv_planning(_RealishCoordinator(empty), {}, {})
        assert win._hv_worker is first
    finally:
        gate.set()
        assert first.wait(20000)
    QCoreApplication.processEvents()


@qt_only
def test_closing_mid_plan_parks_the_thread_instead_of_killing_it():
    """關視窗時規劃還在跑 —— 不可以讓 QThread 被銷毀（那是 qFatal，整支 abort）。

    規劃停不下來（hv 沒有取消點），所以唯一乾淨的做法是：拆掉所有連到即將消失的
    視窗的信號，把執行緒寄放到模組層級，程式結束前再等它收尾。
    """
    import threading
    import ui.main_window as mw

    gate = threading.Event()
    empty = {'results': {}, 'allocation': {}, 'coverage_paths': [],
             'uav_states': [], 'total_makespan': 0.0, 'assembled_paths': {}}

    class _Slow(_RealishCoordinator):
        def plan_coverage_dccpp(self, **kw):
            gate.wait(20)
            return empty

    win = _JoinWindow(home=HOME)
    win._on_hv_finished = lambda r: None
    win._start_hv_planning(_Slow(empty), {}, {})
    worker = win._hv_worker
    try:
        assert worker.isRunning()
        assert worker.parent() is None
        before = len(mw._HV_ORPHAN_WORKERS)
        mw._hv_park_running_worker(win, worker)
        assert worker in mw._HV_ORPHAN_WORKERS
        assert len(mw._HV_ORPHAN_WORKERS) == before + 1
        # 信號已拆：視窗死了也不會有 slot 被呼叫
        with pytest.raises(TypeError):
            worker.finished_ok.disconnect()
    finally:
        gate.set()
        assert worker.wait(20000)
    QCoreApplication.processEvents()
    mw._hv_wait_for_orphan_workers(5000)
    assert worker not in mw._HV_ORPHAN_WORKERS


@qt_only
def test_report_notes_are_surfaced_not_just_counted():
    """`notes` 裡有「你的作業高度被夾了」這種話，不能只記個數量。"""
    win = _JoinWindow(home=HOME)
    win._on_hv_report({'status': 'GO_WITH_WARNINGS', 'errors': [], 'warnings': [],
                       'notes': ['coverage_altitude_clamped(100.0->120.0; min=120)',
                                 'provisional_params(A: a_tr_mps2)'],
                       'uncovered': [], 'timing_ms': {'total': 1.0}})
    loud = win._hv_last_report['loud_notes']
    assert any('coverage_altitude_clamped' in n for n in loud)
    assert not any('provisional_params' in n for n in loud), (
        'provisional_params 是正常的雜訊，不該吵到操作員')
    win._hv_show_report_banner()
    assert 'coverage_altitude_clamped' in win._status.calls[-1][1][0]


@qt_only
def test_uncovered_warning_survives_the_success_message():
    """未覆蓋警示必須在後處理**之後**才顯示，否則會被「完成」訊息蓋掉。"""
    src = inspect.getsource(MainWindow._on_hv_finished)
    i_display = src.index('_display_dccpp_result')
    i_banner = src.index('_hv_show_report_banner')
    assert i_banner > i_display, '警示要排在後處理之後'


@qt_only
def test_stop_does_not_pretend_to_cancel():
    """轉接層把 progress 回呼的例外吞掉 ⇒ 從回呼拋例外中止規劃是死路。

    這條鎖住「我們沒有假裝取消得掉」：stop() 之後結果不採用（emit cancelled），
    但規劃本身仍然跑完。
    """
    import hv_dccpp.integration.aeroplan_adapter as ad
    tick_src = inspect.getsource(ad.HvDccppCoordinator._tick)
    assert 'except Exception' in tick_src, (
        '如果 hv 哪天不再吞回呼例外，就可以改回真正的取消，這條會提醒你')

    empty = {'results': {}, 'allocation': {}, 'coverage_paths': [],
             'uav_states': [], 'total_makespan': 0.0, 'assembled_paths': {}}

    class _TicksThenReturns:
        hv_progress = None
        hv_on_report = None
        completed = False

        def plan_coverage_dccpp(self, **kw):
            self.hv_progress(0.4, 'x')      # stop() 之後這一 tick 不該再 emit
            self.completed = True           # 但規劃仍然跑完
            return empty

    coord = _TicksThenReturns()
    w = HvPlanningWorker(coord, {})
    seen = {'progress': [], 'ok': [], 'cancelled': []}
    w.progress.connect(lambda f, m: seen['progress'].append((f, m)))
    w.finished_ok.connect(lambda r: seen['ok'].append(r))
    w.cancelled.connect(lambda: seen['cancelled'].append(True))
    w.stop()
    w.start()
    assert w.wait(20000)
    QCoreApplication.processEvents()
    assert coord.completed, '規劃本身停不下來（誠實面）'
    assert seen['progress'] == [], 'stop() 之後不該再送進度'
    assert seen['ok'] == [], '被放棄的結果不可以送去顯示'
    assert seen['cancelled'] == [True]


@qt_only
@hv_only
def test_panel_keys_reach_the_planner(monkeypatch):
    """VTOL 偵察面板 → coordinator.hv_params → HvDccppCoordinator 這一整跳。

    `_configure_hv_dccpp` 裡的 `coordinator.hv_params = dict(params)` 是那幾個面板鍵
    唯一的通路；先前的測試都是自己手寫 hv_params，等於跳過它。
    """
    from ui.widgets.vtol_recon_panel import VTOLReconPanel
    panel = VTOLReconPanel()
    panel.algo_combo.setCurrentIndex(panel.algo_combo.findData('hv_dccpp'))
    panel.hv_num_a_spin.setValue(1)      # Pioneer
    panel.hv_num_b_spin.setValue(1)      # HERO —— 總台數由這兩格相加
    panel.hv_c2_spin.setValue(42.0)
    # 豁免值只有勾了核取方塊才會送出（不勾 ＝ 沒有豁免 ＝ 法規 120 m）
    panel.hv_waiver_check.setChecked(True)
    panel.hv_waiver_spin.setValue(300.0)
    panel.hv_clearance_spin.setValue(55.0)
    panel.hv_wind_speed_spin.setValue(3.0)   # 太大會撞到陣風站點檢查 ⇒ NO_GO
    panel.hv_wind_dir_spin.setValue(90.0)
    params = panel.get_params()
    assert params['vehicle_type'] == 'vtol' and params['hv_dccpp'] is True

    win = _FakeWindow(home=HOME)
    coord = SwarmCoordinator()
    assert MainWindow._configure_hv_dccpp(win, coord, params, 'vtol') is True

    seen = {}
    real = hv_adapter.HvDccppCoordinator

    class _Spy(real):
        def __init__(self, **kw):
            seen.update(kw)
            super().__init__(**kw)

    monkeypatch.setattr(hv_adapter, 'HvDccppCoordinator', _Spy)
    coord.plan_coverage_dccpp(**_plan_kwargs())
    assert seen['c2_range_km'] == 42.0
    assert seen['waiver_max_agl_m'] == 300.0
    assert seen['terrain_clearance_m'] == 55.0
    assert seen['wind'] == (3.0, 90.0)
    assert seen['dem_margin_m'] == 3000.0
    assert coord.hv_class_of({'drone_id': 1}) == 'A'
    assert coord.hv_class_of({'drone_id': 2}) == 'B'


@qt_only
def test_recon_panel_offers_both_planners():
    """VTOL 偵察頁要能選演算法，而且選了才算數（`hv_dccpp` 是唯一的旗標來源）。"""
    from ui.widgets.vtol_recon_panel import VTOLReconPanel
    panel = VTOLReconPanel()
    data = [panel.algo_combo.itemData(i) for i in range(panel.algo_combo.count())]
    assert data == ['dccpp', 'hv_dccpp'], f'演算法選項變了：{data}'

    # 預設是既有 DCCPP，hv 旗標關著
    p0 = panel.get_params()
    assert p0['planner'] == 'dccpp' and p0['hv_dccpp'] is False
    assert 'DCCPP' in panel.btn_plan.text() and 'HV' not in panel.btn_plan.text()

    # 切到 HV：旗標打開、按鈕字樣跟著改、HV 專屬欄位一起送出
    panel.algo_combo.setCurrentIndex(panel.algo_combo.findData('hv_dccpp'))
    p1 = panel.get_params()
    assert p1['planner'] == 'hv_dccpp' and p1['hv_dccpp'] is True
    assert 'HV-DCCPP' in panel.btn_plan.text()
    for key in ('hv_num_a', 'hv_waiver_max_agl_m', 'hv_c2_range_km',
                'hv_terrain_clearance_m', 'hv_wind', 'hv_reduce_waypoints'):
        assert key in p1, f'少送 {key}'
    # 對地淨空 0 要送 None（送 0.0 會被當成「淨空 0 公尺」）
    assert p1['hv_terrain_clearance_m'] is None
    panel.hv_clearance_spin.setValue(40.0)
    assert panel.get_params()['hv_terrain_clearance_m'] == 40.0

    # 兩個構型各自獨立設定，總台數是相加的結果（不是再去別的地方調）
    panel.hv_num_a_spin.setValue(3)
    panel.hv_num_b_spin.setValue(4)
    p2 = panel.get_params()
    assert (p2['hv_num_a'], p2['hv_num_b'], p2['num_drones']) == (3, 4, 7)
    assert 'Pioneer' in panel.hv_fleet_label.text()
    assert 'HERO' in panel.hv_fleet_label.text()
    # 上面「4. 群飛編組」那一格在 HV 模式是唯讀的鏡子，不可以變成第二個真相來源
    assert panel.num_drones_spin.value() == 7
    assert not panel.num_drones_spin.isEnabled()
    # 空機隊要當場擋下來，不要送給規劃器
    panel.hv_num_a_spin.setValue(0)
    panel.hv_num_b_spin.setValue(0)
    assert panel.get_params()['num_drones'] == 0
    sent = []
    panel.plan_requested.connect(sent.append)
    panel._emit_plan()
    assert sent == [] and '空' in panel.plan_status.text()

    # 既有 DCCPP 的參數鍵沒有被動到
    for key in ('fov_width_m', 'altitude', 'speed', 'turn_radius',
                'min_separation_m', 'alt_offset_m', 'transition_alt'):
        assert key in p1


@qt_only
def test_platform_photos_show_when_present_and_degrade_when_not(tmp_path, monkeypatch):
    """兩個構型各配一張機體照片；檔案不在時不可以留破圖。

    ⚠ 這條同時鎖住「路徑對不對」：檔名或資料夾改掉的話，畫面只會安靜地
    退回文字佔位，沒有任何錯誤 —— 正是看不出來的那種壞法。
    """
    from PyQt6.QtGui import QPixmap
    from PyQt6.QtWidgets import QLabel
    import ui.widgets.vtol_recon_panel as vrp

    # ① 檔案不在：顯示該放的檔名，面板照常運作
    monkeypatch.setattr(vrp, '_IMAGE_DIR', tmp_path)
    panel = vrp.VTOLReconPanel()
    texts = [w.text() for w in panel.hv_group.findChildren(QLabel) if w.text()]
    assert any(vrp._HV_PLATFORM_IMAGES['A'] in t for t in texts)
    assert any(vrp._HV_PLATFORM_IMAGES['B'] in t for t in texts)
    assert not any(w.pixmap() and not w.pixmap().isNull()
                   for w in panel.hv_group.findChildren(QLabel))

    # ② 檔案在：真的畫出照片（可觸發性 —— 證明這條測試量得到差別）
    for name in vrp._HV_PLATFORM_IMAGES.values():
        pm = QPixmap(320, 180)
        pm.fill()
        assert pm.save(str(tmp_path / name))
    panel2 = vrp.VTOLReconPanel()
    shown = [w for w in panel2.hv_group.findChildren(QLabel)
             if w.pixmap() and not w.pixmap().isNull()]
    assert len(shown) == 2, f'應該有兩張機體照片，實際 {len(shown)}'
    assert all(w.pixmap().height() == vrp._PLATFORM_THUMB_H for w in shown)
    texts2 = [w.text() for w in panel2.hv_group.findChildren(QLabel) if w.text()]
    assert not any(vrp._HV_PLATFORM_IMAGES['A'] in t for t in texts2), '照片在了還顯示檔名'

    # 正式路徑必須指到打包收得到的地方（ui/resources 整包會進 PyInstaller）
    assert vrp.__file__.endswith('vtol_recon_panel.py')
    real = Path(vrp.__file__).resolve().parent.parent / 'resources' / 'images'
    assert real.is_dir(), f'機體照片資料夾不存在：{real}'


@qt_only
def test_recon_plan_waits_for_the_background_result():
    """偵察頁改走非同步後，摘要表必須由 `on_done` 觸發，不能當場讀 `_dccpp_result`。

    這是最容易靜默壞掉的地方：直接讀的話會拿到**上一次**的規劃結果，
    畫面上是一張看起來很正常、但屬於別次規劃的表。
    """
    src = inspect.getsource(MainWindow._on_vtol_recon_plan)
    assert 'on_done' in src, '沒有把結果接到 on_done'
    tree = ast.parse(src.lstrip())
    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and n.func.attr == '_on_dccpp_coverage_requested']
    assert calls, '找不到規劃呼叫'
    kw = {k.arg for c in calls for k in c.keywords}
    assert {'run_async', 'on_done'} <= kw
    # 呼叫之後不可以再自己讀 self._dccpp_result（那是舊的同步寫法）。
    # ⚠ 用 AST 比對，不要對原始碼做子字串搜尋：註解裡就寫著這個名字，
    #   直接搜字串的話這條會恆紅（而且是假紅）。
    reads = [n for n in ast.walk(tree)
             if isinstance(n, ast.Attribute) and n.attr == '_dccpp_result']
    assert not reads, '偵察頁又自己去讀 _dccpp_result 了'
    # 而接手的那一支要吃 (params, ok) 兩個參數
    params = inspect.signature(MainWindow._on_vtol_recon_planned).parameters
    assert list(params)[1:] == ['params', 'ok']


@qt_only
@hv_only
def test_recon_page_drives_either_planner(hv_plan, monkeypatch):
    """VTOL 偵察頁按下規劃：兩種演算法都要送出正確的參數並回填摘要表。

    這裡把 `_on_dccpp_coverage_requested` 換成替身（規劃本身另有測試），
    只驗這一頁自己的接線：狀態字樣、run_async 的選擇、以及結果回表格。
    """
    import types
    _coord, result = hv_plan

    class _Panel:
        def __init__(self):
            self.status = []
            self.rows = None
            self.summary = None

        def set_plan_status(self, t):
            self.status.append(t)

        def set_plan_result(self, rows, summary):
            self.rows, self.summary = rows, summary

    class _Win:
        def __init__(self):
            self.vtol_recon_panel = _Panel()
            self.corners = [(23.0, 120.0), (23.0, 120.1), (23.1, 120.1)]
            self._dccpp_result = result
            self.seen = []
            self._on_vtol_recon_plan = types.MethodType(
                MainWindow._on_vtol_recon_plan, self)
            self._on_vtol_recon_planned = types.MethodType(
                MainWindow._on_vtol_recon_planned, self)
            self._polyline_length_m = MainWindow._polyline_length_m
            # 多區模式（2026-09-07）：閘門改成問 survey_area_polygons()，
            # 替身也要有它，否則測的是「屬性不存在」而不是規劃分派。
            self.survey_areas = []
            self.survey_area_polygons = types.MethodType(
                MainWindow.survey_area_polygons, self)

        def _get_cesium_widget(self):
            return None            # 3D 視覺化不在這條測試的範圍

        def _on_dccpp_coverage_requested(self, params, *, run_async=True, on_done=None):
            self.seen.append((params.get('planner'), run_async))
            if on_done:
                on_done(True)      # 替身：當場完成

    # ① 既有 DCCPP → 同步
    win = _Win()
    win._on_vtol_recon_plan({'planner': 'dccpp', 'hv_dccpp': False,
                             'fov_width_m': 120.0, 'speed': 18.0})
    assert win.seen == [('dccpp', False)]
    assert any('DCCPP 規劃中' in t for t in win.vtol_recon_panel.status)

    # ② HV-DCCPP → 背景執行緒
    win = _Win()
    win._on_vtol_recon_plan({'planner': 'hv_dccpp', 'hv_dccpp': True,
                             'fov_width_m': 120.0, 'speed': 18.0})
    assert win.seen == [('hv_dccpp', True)], 'HV 必須走背景執行緒'
    assert any('HV-DCCPP' in t for t in win.vtol_recon_panel.status)
    # 摘要表：兩架、航程與時間都算得出來，而且看得出構型
    rows = win.vtol_recon_panel.rows
    assert len(rows) == 2
    assert all(r[1] > 0 and r[2] > 0.0 and r[3] > 0.0 for r in rows)
    assert {r[4] for r in rows} == {'Pioneer PNP', 'HERO PNP'}, (
        f'摘要表沒有帶構型：{rows}')
    assert 'makespan' in win.vtol_recon_panel.summary

    # 要了 5 台但只排得下 2 台 ⇒ 摘要必須說出來（規劃器自己不會講）
    win2 = _Win()
    win2._on_vtol_recon_plan({'planner': 'hv_dccpp', 'hv_dccpp': True,
                              'fov_width_m': 120.0, 'speed': 18.0,
                              'num_drones': 5})
    assert '2/5' in win2.vtol_recon_panel.summary
    assert '沒有分到工作' in win2.vtol_recon_panel.summary

    # ③ 規劃失敗（on_done(False)）：面板要說話，不能停在「規劃中 …」
    win = _Win()
    win._on_dccpp_coverage_requested = types.MethodType(
        lambda self, params, *, run_async=True, on_done=None: on_done(False), win)
    win._on_vtol_recon_plan({'planner': 'hv_dccpp', 'hv_dccpp': True,
                             'fov_width_m': 120.0, 'speed': 18.0})
    assert win.vtol_recon_panel.rows == []
    assert '未產生結果' in win.vtol_recon_panel.status[-1]


@qt_only
def test_on_done_fires_on_every_path():
    """成功、失敗、取消三條路都要叫到 `on_done`，否則面板永遠停在「規劃中 …」。"""
    win = _JoinWindow(home=HOME)
    seen = []
    win._hv_on_done = lambda ok: seen.append(('failed', ok))
    win._on_hv_failed = types_MethodType(MainWindow._on_hv_failed, win)
    import PyQt6.QtWidgets as _QtW
    _orig = _QtW.QMessageBox.critical
    _QtW.QMessageBox.critical = staticmethod(lambda *a, **k: 0)
    try:
        win._on_hv_failed('boom')
    finally:
        _QtW.QMessageBox.critical = _orig
    assert seen == [('failed', False)]

    seen.clear()
    win._hv_on_done = lambda ok: seen.append(('cancelled', ok))
    win._on_hv_cancelled()
    assert seen == [('cancelled', False)]

    # 回呼自己爆掉不可以連累規劃流程
    win._hv_on_done = lambda ok: 1 / 0
    win._on_hv_cancelled()          # 不應拋出


@qt_only
def test_display_is_separated_from_planning():
    """後處理必須是一支可獨立呼叫的方法，否則背景執行緒完成後沒有東西可以接。"""
    assert hasattr(MainWindow, '_display_dccpp_result')
    params = inspect.signature(MainWindow._display_dccpp_result).parameters
    for name in ('dccpp_result', 'params', 'corners_snapshot', 'centroid_lat',
                 'centroid_lon', 'altitude', 'speed', 'turn_radius',
                 'vehicle_type', 'auto_scan'):
        assert name in params, f'_display_dccpp_result 少了 {name}'
    # 邊界點必須來自規劃前拍的快照，不可以在顯示階段重讀 self.corners
    # （背景規劃期間使用者仍可改邊界，重讀會畫出與實飛不符的多邊形且不會報錯）
    # ⚠ 比對前先把 docstring 拿掉：它本身就提到 list(self.corners) 這個字串，
    #   直接對整份原始碼做子字串比對的話，這條測試會恆紅（而且是假紅）。
    fn = ast.parse(inspect.getsource(MainWindow._display_dccpp_result).lstrip()).body[0]
    if (fn.body and isinstance(fn.body[0], ast.Expr)
            and isinstance(fn.body[0].value, ast.Constant)
            and isinstance(fn.body[0].value.value, str)):
        fn.body = fn.body[1:]
    body_src = ast.unparse(fn)
    assert 'list(self.corners)' not in body_src
    assert 'list(corners_snapshot)' in body_src


@qt_only
def test_real_platform_photos_are_committed_and_loadable():
    """兩張機體照真的在 repo 裡而且讀得出來。

    ⚠ 上面那條用 tmp_path 測「有 / 沒有」兩種行為，**測不到正式檔案在不在**。
    照片缺席時面板只會安靜地退回顯示檔名，畫面上看起來像設計就是這樣。
    """
    import ui.widgets.vtol_recon_panel as vrp
    from PyQt6.QtGui import QPixmap

    for cls, name in vrp._HV_PLATFORM_IMAGES.items():
        path = vrp._IMAGE_DIR / name
        assert path.exists(), f'構型 {cls} 的機體照沒進 repo：{path}'
        pm = QPixmap(str(path))
        assert not pm.isNull(), f'{name} 讀不出來（檔壞了或格式不對）'
        # 縮圖高 74、tooltip 以 width=260 呈現 ⇒ 至少要有 260 寬才不糊
        assert pm.width() >= 260, f'{name} 只有 {pm.width()} px 寬，tooltip 會糊'
        # 也不要無謂地大：repo 裡放原始 1500 px 照片沒有意義
        assert path.stat().st_size < 300 * 1024, \
            f'{name} 有 {path.stat().st_size / 1024:.0f} KB，縮一下再進 repo'
        thumb = vrp._platform_pixmap(name, vrp._PLATFORM_THUMB_H)
        assert thumb is not None and thumb.width() <= vrp._PLATFORM_THUMB_W, \
            f'{name} 縮圖 {thumb.width() if thumb else 0} px 超出框寬 {vrp._PLATFORM_THUMB_W}'
