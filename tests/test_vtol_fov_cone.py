# -*- coding: utf-8 -*-
"""VTOL 偵察頁 FOV 光錐的值層級守衛（2026-08-25）。

需求:VTOL 頁要有跟戰術分析頁一樣的 3D FOV 光錐,但**參數吃 VTOL 這一頁的**。

⚠️⚠️ 這件事最危險的地方不是畫不出來,而是**畫得出來但吃錯來源**:
兩頁的 HFOV 預設都是 60、VFOV 都是 45 ⇒ 接錯來源時畫面**完全一樣**,
只有當使用者去改其中一頁的值才會發現「改了沒用」——
和掃描角那次(參數送到了、log 也印了、但被死掉的管線讀走)是同一個形狀。
⇒ 所有斷言都用**互不相同**的值,不用預設值。

參見 [[two-pipelines-one-is-dead]]、[[verifying-the-verifier]]。
"""

import pytest

from PyQt6.QtWidgets import QApplication

from ui.main_window import (FOV_SOURCE_TACTICAL, FOV_SOURCE_VTOL,
                            resolve_fov_param_source)
from ui.widgets.vtol_recon_panel import VTOLReconPanel


@pytest.fixture(scope="module")
def qt_app():
    yield QApplication.instance() or QApplication([])


@pytest.fixture()
def panel(qt_app):
    return VTOLReconPanel()


class _Panel:
    """最小面板替身:只要提供那兩個同名取值方法。"""

    def __init__(self, tag, radius, mount, hfov, vfov):
        self.tag = tag
        self._r, self._m, self._h, self._v = radius, mount, hfov, vfov

    def get_fov_radius(self):
        return self._r

    def get_fov_visual_params(self):
        return (self._m, self._h, self._v)


# ⚠ 每個值都不一樣 —— 用預設值(60/45)測等於沒測
TAC = _Panel('tactical', 111.0, 11.0, 61.0, 46.0)
VTOL = _Panel('vtol', 222.0, 22.0, 77.0, 33.0)


# ══════════════════════════════════════════════════════════════
# 接縫:誰提供參數
# ══════════════════════════════════════════════════════════════

def test_vtol_source_uses_vtol_panel():
    """⚠⚠⚠ 正主守衛:來源是 VTOL 時,參數必須來自 VTOL 面板。"""
    src = resolve_fov_param_source(FOV_SOURCE_VTOL, TAC, VTOL)
    assert src is VTOL
    assert src.get_fov_visual_params() == (22.0, 77.0, 33.0)
    assert src.get_fov_radius() == pytest.approx(222.0)


def test_tactical_source_uses_tactical_panel():
    src = resolve_fov_param_source(FOV_SOURCE_TACTICAL, TAC, VTOL)
    assert src is TAC
    assert src.get_fov_visual_params() == (11.0, 61.0, 46.0)


@pytest.mark.parametrize("source", [None, FOV_SOURCE_TACTICAL, 'unknown', ''])
def test_non_vtol_sources_fall_back_to_tactical(source):
    """只有明確是 VTOL 才切過去;其餘一律戰術頁(不猜)。"""
    assert resolve_fov_param_source(source, TAC, VTOL) is TAC


def test_missing_vtol_panel_falls_back_not_crashes():
    """VTOL 面板還沒建立時要退回戰術頁,不能丟例外。

    ⚠ 光錐開關可能在面板建立前就被還原(設定持久化),丟例外會炸掉啟動。
    """
    assert resolve_fov_param_source(FOV_SOURCE_VTOL, TAC, None) is TAC


def test_two_sources_are_actually_distinguishable():
    """前提自檢:兩個來源給的值必須不同,否則上面全是假綠燈。

    ⚠ 這條擋的是「拿預設值當測資」——兩頁 HFOV 預設都是 60,
    用預設值時接錯來源也會過。
    """
    a = resolve_fov_param_source(FOV_SOURCE_VTOL, TAC, VTOL)
    b = resolve_fov_param_source(FOV_SOURCE_TACTICAL, TAC, VTOL)
    assert a.get_fov_visual_params() != b.get_fov_visual_params()
    assert a.get_fov_radius() != b.get_fov_radius()


# ══════════════════════════════════════════════════════════════
# 面板:VTOL 面板真的提供那兩個同名方法
# ══════════════════════════════════════════════════════════════

def test_vtol_panel_exposes_the_same_interface(panel):
    """兩個面板必須有**同名同語意**的取值方法,接縫才不用分支。"""
    assert hasattr(panel, 'get_fov_visual_params')
    assert hasattr(panel, 'get_fov_radius')
    assert hasattr(panel, 'is_fov_cone_enabled')
    mount, hfov, vfov = panel.get_fov_visual_params()
    assert (mount, hfov, vfov) == (
        pytest.approx(panel.mount_spin.value()),
        pytest.approx(panel.hfov_spin.value()),
        pytest.approx(panel.vfov_spin.value()))


def test_vtol_params_track_the_panel_spinboxes(panel):
    """改面板數字 ⇒ 取值跟著變(不是快照、不是預設值)。"""
    panel.hfov_spin.setValue(77.0)
    panel.vfov_spin.setValue(33.0)
    panel.mount_spin.setValue(22.0)
    assert panel.get_fov_visual_params() == (
        pytest.approx(22.0), pytest.approx(77.0), pytest.approx(33.0))


def test_fov_radius_comes_from_the_swath(panel):
    """本頁沒有「FOV 半徑」輸入 ⇒ 用感測器掃描寬度的一半(同一個幾何量)。"""
    swath = getattr(panel, '_raw_swath_width_m', 0.0)
    assert swath > 0, "掃描寬度沒算出來,這條測不到東西"
    assert panel.get_fov_radius() == pytest.approx(swath / 2.0)


def test_fov_radius_survives_broken_camera_geometry(panel):
    """相機幾何無解時 swath=0 ⇒ 半徑不可以是 0 或負數(Cesium 會畫不出來)。"""
    panel._raw_swath_width_m = 0.0
    assert panel.get_fov_radius() > 0


def test_toggle_emits_signal(panel, qt_app):
    """勾選框要真的發出訊號 —— 不然是個不會動的裝飾。"""
    seen = []
    panel.fov_cone_toggle_requested.connect(seen.append)
    panel.fov_cone_check.setChecked(True)
    panel.fov_cone_check.setChecked(False)
    assert seen == [True, False]


def test_angle_change_emits_visual_signal(panel, qt_app):
    """三個角度任一改變都要發 fov_visual_changed,否則要等下一筆遙測才更新。"""
    seen = []
    panel.fov_visual_changed.connect(lambda *a: seen.append(a))
    panel.hfov_spin.setValue(70.0)
    assert seen, "改 HFOV 沒有發出 fov_visual_changed"
    assert seen[-1] == (pytest.approx(panel.mount_spin.value()),
                        pytest.approx(70.0),
                        pytest.approx(panel.vfov_spin.value()))


def test_status_label_is_writable(panel):
    """狀態標籤是使用者唯一能看出「吃哪一頁參數」的地方。"""
    panel.set_fov_cone_status('使用本頁參數：掛載 1.0°', ok=True)
    assert '本頁' in panel.fov_cone_status.text()


def test_no_consumer_hardcodes_the_tactical_panel():
    """⚠ 結構性守衛:取光錐參數不得直接寫 `self.parameter_panel.get_fov_*`。

    為什麼**必須**是 AST:值層級鎖得住「接縫選對來源了嗎」,但鎖不住
    「消費端有沒有用接縫」。寫死 `self.parameter_panel` 時,接縫的測試照樣全綠,
    而 VTOL 頁改參數會**完全沒有反應且沒有錯誤**
    (實測突變臂 M4 就是這樣活下來的)。

    ⚠ 用 AST 而不是 `SRC.count(...)` 字串比對 —— 後者對換行/改名/註解全都誤判,
    見 [[verifying-the-verifier]]。
    """
    import ast
    import pathlib

    src = pathlib.Path('ui/main_window.py').read_text(encoding='utf-8')
    tree = ast.parse(src)

    GETTERS = {'get_fov_radius', 'get_fov_visual_params'}
    # 白名單:SAR 熱力圖半徑**故意**永遠吃戰術頁(它是戰術頁的功能),
    # 見 _heatmap_radius_m 的註解。除此之外一律要走接縫。
    ALLOWED_IN = {'_heatmap_radius_m'}
    allowed_lines = set()
    for fn in ast.walk(tree):
        if isinstance(fn, ast.FunctionDef) and fn.name in ALLOWED_IN:
            allowed_lines.update(
                n.lineno for n in ast.walk(fn) if hasattr(n, 'lineno'))

    offenders = []
    for node in ast.walk(tree):
        if node.lineno in allowed_lines if hasattr(node, 'lineno') else False:
            continue
        if not (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr in GETTERS):
            continue
        recv = node.func.value
        # 允許:接縫回傳值(某個區域變數/函式呼叫結果)
        # 禁止:self.parameter_panel.get_fov_*  /  self.vtol_recon_panel.get_fov_*
        if (isinstance(recv, ast.Attribute)
                and isinstance(recv.value, ast.Name)
                and recv.value.id == 'self'
                and recv.attr in ('parameter_panel', 'vtol_recon_panel')):
            offenders.append((node.lineno, recv.attr, node.func.attr))

    assert not offenders, (
        "這些呼叫繞過了 resolve_fov_param_source(),VTOL 頁的參數會被忽略且無錯誤:"
        + "; ".join(f"main_window.py:{ln} self.{p}.{g}()" for ln, p, g in offenders))


def test_the_ast_guard_can_actually_see_the_getters():
    """前提自檢:main_window 裡真的有這兩個取值呼叫。

    ⚠ 沒有這條的話,上面那條在「取值方法被改名」時會靜靜變成永遠通過 ——
    一個永遠不可能失敗的守衛([[instrument-that-cannot-fire]])。
    """
    import ast
    import pathlib

    tree = ast.parse(pathlib.Path('ui/main_window.py').read_text(encoding='utf-8'))
    found = [n.func.attr for n in ast.walk(tree)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and n.func.attr in {'get_fov_radius', 'get_fov_visual_params'}]
    assert 'get_fov_radius' in found, "找不到 get_fov_radius() 呼叫 ⇒ 上面的守衛是空的"
    assert 'get_fov_visual_params' in found


# ══════════════════════════════════════════════════════════════
# Cesium 端硬夾制:不可以靜靜地讓面板值與畫面分家
# ══════════════════════════════════════════════════════════════

def test_negative_mount_angle_is_reported_not_silent():
    """⚠⚠⚠ VTOL 面板允許 -84°,而 Cesium 把掛載角夾成 max(0, ...)。

    戰術頁範圍是 0~85,永遠碰不到這個夾制;把 VTOL 頁接上光錐,等於**第一次**
    讓使用者碰得到「設 -30、畫出來是 0」而且面板足跡仍用 -30 算 ⇒ 兩邊對不上,
    兩邊都不報錯。這條要求它必須被講出來。
    """
    from ui.main_window import fov_cone_clamp_warning
    w = fov_cone_clamp_warning(-30.0, 60.0, 45.0)
    assert w, "負掛載角被靜靜夾掉卻沒有任何說明"
    assert '-30' in w or '−30' in w
    assert '0' in w


def test_in_range_values_produce_no_warning():
    """已知合法值不可以誤報(否則警告會被當雜訊忽略)。"""
    from ui.main_window import fov_cone_clamp_warning
    assert fov_cone_clamp_warning(0.0, 60.0, 45.0) == ''
    assert fov_cone_clamp_warning(30.0, 120.0, 90.0) == ''
    assert fov_cone_clamp_warning(85.0, 170.0, 170.0) == ''


@pytest.mark.parametrize("mount,hfov,vfov", [
    (-1.0, 60.0, 45.0),      # 剛好越界的負角
    (90.0, 60.0, 45.0),      # 超過 89.5
    (0.0, 179.0, 45.0),      # 半角 89.5 > 85
    (0.0, 60.0, 179.0),
])
def test_every_out_of_range_input_is_reported(mount, hfov, vfov):
    from ui.main_window import fov_cone_clamp_warning
    assert fov_cone_clamp_warning(mount, hfov, vfov) != ''


def test_panel_range_can_actually_reach_the_clamp(panel):
    """前提自檢:VTOL 面板真的設得到會被夾掉的值。

    ⚠ 沒有這條的話,上面幾條可能在測一個**面板根本到不了**的區域 ——
    那就是 [[instrument-that-cannot-fire]]。
    """
    from ui.main_window import fov_cone_clamp_warning
    panel.mount_spin.setValue(-30.0)
    assert panel.mount_spin.value() == pytest.approx(-30.0), \
        "面板夾住了負角 ⇒ 上面那些測試碰不到真實情境"
    mount, hfov, vfov = panel.get_fov_visual_params()
    assert fov_cone_clamp_warning(mount, hfov, vfov) != ''


def test_heatmap_radius_is_pinned_to_the_tactical_panel():
    """⚠ 熱力圖是戰術頁的功能,半徑不可以跟著光錐來源跑。

    我改光錐來源時一度讓它跟著跑 ⇒ VTOL 頁一開光錐就把戰術頁的熱力圖
    掃描半徑從 50 改成 69.3,而使用者完全不知道自己動到另一頁的東西。
    """
    import ast
    import pathlib
    tree = ast.parse(pathlib.Path('ui/main_window.py').read_text(encoding='utf-8'))
    fn = next((n for n in ast.walk(tree)
               if isinstance(n, ast.FunctionDef) and n.name == '_heatmap_radius_m'), None)
    assert fn is not None, "_heatmap_radius_m 不見了 ⇒ 熱力圖半徑可能又跟著光錐跑"
    # 它必須直接讀戰術頁,而不是走光錐的來源接縫
    names = {n.func.value.attr for n in ast.walk(fn)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and isinstance(n.func.value, ast.Attribute)}
    assert 'parameter_panel' in names
    # 而 update_heatmap 的呼叫必須用它
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Attribute) and n.func.attr == 'update_heatmap']
    assert calls, "找不到 update_heatmap 呼叫 ⇒ 這條守衛是空的"
    for c in calls:
        used = [a for a in c.args if isinstance(a, ast.Call)
                and isinstance(a.func, ast.Attribute)
                and a.func.attr == '_heatmap_radius_m']
        assert used, (
            f"main_window.py:{c.lineno} 的 update_heatmap 沒有用 _heatmap_radius_m()"
            f" ⇒ 熱力圖半徑會跟著光錐來源變")


# ══════════════════════════════════════════════════════════════
# 兩頁擁有權交接 + 「畫不出來」要講出來
# ══════════════════════════════════════════════════════════════

def test_both_panels_expose_a_fov_checkbox(qt_app):
    """⚠ 兩個面板的勾選框屬性名**不一樣**(歷史因素)。

    查找失敗時 `_uncheck_fov_source` 只會 return ⇒ 兩個勾選框都留在「已勾選」,
    而 `_fov_cone_source` 只認得其中一個 ⇒ 使用者看到兩個開關都亮著,
    光錐卻吃另一頁的數字,**沒有任何錯誤**。這條把名字釘住。
    """
    from PyQt6.QtWidgets import QCheckBox

    from ui.main_window import FOV_CHECKBOX_ATTRS
    from ui.widgets.parameter_panel import ParameterPanel
    from ui.widgets.vtol_recon_panel import VTOLReconPanel

    for cls in (VTOLReconPanel, ParameterPanel):
        panel = cls()
        found = [getattr(panel, n) for n in FOV_CHECKBOX_ATTRS
                 if getattr(panel, n, None) is not None]
        assert found, (
            f"{cls.__name__} 找不到光錐勾選框（試過 {FOV_CHECKBOX_ATTRS}）"
            f" ⇒ 擁有權交接會靜靜失效")
        assert isinstance(found[0], QCheckBox)


def test_ground_level_uavs_are_reported_not_silent():
    """⚠⚠ 貼地的 UAV 在 Cesium 端會被刪掉光錐(main.html 對 !(alt>0) 直接 remove)。

    VTOL 停在起降場正是使用者第一次勾這個框的時刻 ⇒ 什麼都不會出現。
    這條要求「有姿態」和「真的離地」必須分開數,狀態列才講得出差別。
    """
    from ui.main_window import fov_pose_summary
    ground = {1: (25.0, 121.0, 0.0, 0, 0, 0), 2: (25.0, 121.0, 0.2, 0, 0, 0)}
    assert fov_pose_summary(ground) == (2, 0)


def test_airborne_uavs_are_counted():
    from ui.main_window import fov_pose_summary
    mixed = {1: (25.0, 121.0, 0.0, 0, 0, 0), 2: (25.0, 121.0, 120.0, 0, 0, 0)}
    assert fov_pose_summary(mixed) == (2, 1)


def test_empty_pose_cache_is_not_confused_with_grounded():
    """「沒收到姿態」和「收到了但都在地上」是兩件事,訊息不能一樣。"""
    from ui.main_window import fov_pose_summary
    assert fov_pose_summary({}) == (0, 0)
