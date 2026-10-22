# -*- coding: utf-8 -*-
"""tests/test_attitude_fidelity_probe.py — tools/attitude_fidelity_probe.py（3D 模型姿態忠實度量測）本身量得準、也抓得到錯。

使用者 2026-10-22：「模型要忠實反映 SITL 的姿態」。探針走真的那一條路（SITLLink → MainWindow 合幀 → 真 CesiumMapWidget），
這裡證明它：
  A. 純數學／資料處理（不開視窗）：3-2-1 與 ENU 機體軸互換、跨北 slerp、每列誤差只落在該落的軸、統計只算 ok 列、
     飛行任務的形狀（且過得了 SITLLink 自己的寫入前檢查）、頁面 JS 用到的名字 main.html 真的有。
  B. 假飛控 × 真 SITLLink（真 socket、不開視窗）：VTOL／AUTO／GPS、20 Hz 串流要求有效、送出去的姿態一字不差到達、
     任務寫入＋讀回驗證、FORCE ARM 的狀態文字確實是探針在等的那句。
  C. 真的 QtWebEngine 視窗（WA_DontShowOnScreen；SAFE_ALLOW_BROWSER=1 才跑、一次一個）：
     (a) 機鼻跟航跡關 ⇒ 誤差 ≈ 0；(b) 開、航跡比機頭偏 10° ⇒ 偏航誤差 = 10°，滾轉／俯仰仍 ≈ 0；
     (c) 把頁面換回舊的角度相加寫法 ⇒ 滾轉誤差很大（探針有牙齒）；--fly 飛完正方形落地自己停；PreArm 擋下被 FORCE ⇒ 中止不飛。
  2026-10-22 審查後補（探針原本量不到的東西）：
     A：開機時間軸真值（下包絡擬合、成批偵測）、SITLLink 換算 vs 獨立真值（同一個飛控時刻）、每幀列、相位直方圖、
        閘門是「全部同時成立」、校正寫死在探針裡、萬向鎖、讀回的「現在」用 JS 的時間、設定在任何 ui import 之前換掉
        （子行程＋稽核掛鉤擋 settings.json 的開檔，對照組證明掛鉤抓得到）。
     B：一直在彎的劇本＋姿態串流錯相＋成批送 × 真 SITLLink；PLANE／COPTER／沒有機型提示的 QuadPlane（PLANE → VTOL）。
     C：(d) 一直在彎＋錯相 ⇒ 閘門全過、讀回的相位散開；(d2) 不插值的突變 ⇒ 閘門擋下；(e) 成批到 ⇒ 開機時間軸看得到、
        閘門擋下；(f) 飛行中 PLANE → VTOL 換模型 ⇒ 兩種模型各自的校正都對；(g) 固定翼、多旋翼。
     視窗測試的 GLB 跟 app 一樣用合併檔（AEROPLAN_GLB_MERGE=1；快取放 tmp，有 repo 的現成合併檔就複製過去）。
"""
from __future__ import annotations

import bisect
import json
import math
import os
import pathlib
import random
import re
import subprocess
import sys
import time

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('AEROPLAN_NO_FILE_LOG', '1')
os.environ.setdefault('MAVLINK20', '1')

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools import attitude_fidelity_probe as afp  # noqa: E402

HTML = (ROOT / 'ui' / 'widgets' / 'cesium_templates' / 'main.html').read_text(encoding='utf-8')
_ENV = dict(AEROPLAN_NO_FILE_LOG='1', PYTHONIOENCODING='utf-8', MAVLINK20='1')


@pytest.fixture(autouse=True, scope='module')
def _no_real_settings_file(tmp_path_factory):
    """這個行程裡 import ui.main_window（它在模組層 get_settings()）之前，全域設定先換成指向 tmp 裡不存在的檔
    ⇒ 測試不讀 config/settings.json（2026-10-22 審查：test_gui_stub… 以前會讀一次）。"""
    import config.settings as cs
    old = cs._global_settings
    cs._global_settings = cs.GlobalSettings(config_file=str(tmp_path_factory.mktemp('settings') / 'none.json'))
    yield
    cs._global_settings = old


# ═══════════════════════════════════════════════════════════════════════
#  獨立參考：逐一乘旋轉矩陣（不是探針用的閉式），ENU 機體軸
# ═══════════════════════════════════════════════════════════════════════
def _mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _ref_dcm(yaw, pitch, roll):
    ps, th, ph = (math.radians(v) for v in (yaw, pitch, roll))
    rz = [[math.cos(ps), -math.sin(ps), 0], [math.sin(ps), math.cos(ps), 0], [0, 0, 1]]
    ry = [[math.cos(th), 0, math.sin(th)], [0, 1, 0], [-math.sin(th), 0, math.cos(th)]]
    rx = [[1, 0, 0], [0, math.cos(ph), -math.sin(ph)], [0, math.sin(ph), math.cos(ph)]]
    return _mm(_mm(rz, ry), rx)


def _ref_axes(yaw, pitch, roll):
    """ENU 下的 (機鼻, 右翼, 機背)。"""
    c = _ref_dcm(yaw, pitch, roll)
    enu = lambda j: [c[1][j], c[0][j], -c[2][j]]   # noqa: E731  NED → ENU
    down = enu(2)
    return {'nose': enu(0), 'right': enu(1), 'up': [-v for v in down]}


# ═══════════════════════════════════════════════════════════════════════
#  A. 純數學／資料處理
# ═══════════════════════════════════════════════════════════════════════
def test_euler_dcm_quaternion_and_enu_axes_agree_with_the_independent_reference():
    rnd = random.Random(20261022)
    for _ in range(400):
        y, p, r = rnd.uniform(0, 360), rnd.uniform(-85, 85), rnd.uniform(-180, 180)
        ref = _ref_dcm(y, p, r)
        assert afp.rotation_angle_deg(ref, afp.dcm_from_euler(y, p, r)) < 1e-5
        ax = _ref_axes(y, p, r)
        c = afp.dcm_from_enu_axes(ax['nose'], ax['right'], ax['up'])
        assert afp.rotation_angle_deg(ref, c) < 1e-5
        ey, ep, er = afp.euler_from_dcm(c)
        assert abs(afp.wrap180(ey - y)) < 1e-6 and abs(ep - p) < 1e-6 and abs(afp.wrap180(er - r)) < 1e-6
        assert afp.rotation_angle_deg(ref, afp.dcm_from_quat(afp.quat_from_dcm(ref))) < 1e-5
    # 慣例本身（不靠參考）：航向 0、右滾 30 ⇒ 右翼往下；抬頭 20 ⇒ 機鼻往上；航向 90 ⇒ 機鼻朝東
    c = afp.dcm_from_euler(0, 0, 30)
    assert c[2][1] == pytest.approx(0.5)                     # 右翼（第 2 欄）的 Down 分量 > 0
    assert afp.dcm_from_euler(0, 20, 0)[2][0] == pytest.approx(-math.sin(math.radians(20)))
    assert afp.dcm_from_euler(90, 0, 0)[1][0] == pytest.approx(1.0)


def test_rotation_angle_is_independent_of_how_the_error_is_split_into_euler_angles():
    a, b = afp.dcm_from_euler(10, 5, 30), afp.dcm_from_euler(10, 5, 37)
    assert afp.rotation_angle_deg(a, b) == pytest.approx(7.0, abs=1e-6)
    assert afp.rotation_angle_deg(afp.dcm_from_euler(359, 0, 0), afp.dcm_from_euler(1, 0, 0)) == pytest.approx(2.0)


def test_truth_series_slerps_across_north_and_reports_before_hold_and_duplicates():
    ts = afp.TruthSeries([(10.0, 0, 0, 350), (10.0, 99, 99, 99), (11.0, 0, 0, 10), (0.0, 1, 1, 1)])
    assert len(ts) == 2, '同一個時間戳只留第一筆、t ≤ 0（沒收過）不要'
    q, st, gap = ts.at(10.5)
    y, p, r = afp.euler_from_dcm(afp.dcm_from_quat(q))
    assert st == 'ok' and gap == pytest.approx(1.0)
    assert abs(afp.wrap180(y)) < 1e-6, f'350→10 的中點是 0，不是 180：{y}'
    q, st, _ = ts.at(10.25)
    assert afp.wrap180(afp.euler_from_dcm(afp.dcm_from_quat(q))[0]) == pytest.approx(-5.0, abs=1e-6)
    assert ts.at(9.0)[1] == 'before' and ts.at(11.5)[1] == 'hold' and ts.at(11.0)[1] == 'ok'
    assert afp.TruthSeries([]).at(1.0)[1] == 'none'


def _frames(sid=1, t0=1000.0, n=40, dt=0.05, mode='AUTO', speed=18.0, att=None):
    """FRAME_FIELDS 元組：姿態 att(t) → (r, p, y)。"""
    att = att or (lambda t: (30.0 * math.sin(t), 10.0 * math.cos(t), (350.0 + 20.0 * (t - t0)) % 360.0))
    out = []
    for i in range(n):
        t = t0 + i * dt
        r, p, y = att(t)
        out.append((t + 0.002, sid, mode, True, speed, 'VTOL', t, r, p, y, 23.45, 120.28, 100.0, 3, 18.0, 0.0))
    return out, att


def _read(t_read, t_disp, ax, **kw):
    u = {'disp': t_disp * 1000.0, 'delay': t_read - t_disp, 'kind': 'vtol', 'nAtt': 10, 'lastAttMs': (t_disp + 0.04) * 1000,
         'yaw': 0.0, 'nose': 0.0, 'crs': 0.0, 'pitch': 0.0, 'roll': 0.0, 'speed': 18.0}
    if ax is not None:
        u['ax'] = ax
    u.update(kw)
    return (t_read, {'flag': True, 'uav': {'1': u}})


def test_compute_rows_puts_each_injected_error_on_its_own_axis_and_labels_statuses():
    frames, att = _frames()
    reads = []
    for k, t_disp in enumerate((1000.02, 1000.5, 1001.03, 1001.5, 1001.8, 999.0, 1003.0)):
        r, p, y = att(t_disp)
        if k == 2:
            r += 4.0                                         # 只在滾轉加 4°
        if k == 3:
            y += 7.0                                         # 只在偏航加 7°（像機鼻改用航跡）
        reads.append(_read(t_disp + 0.13, t_disp, _ref_axes(y, p, r)))
    reads.append(_read(1001.2, 1001.1, None))                # 頁面還沒有姿態
    rows = afp.compute_rows(reads, frames, warmup_s=0.3)
    st = [row['status'] for row in rows]
    assert st == ['warmup', 'ok', 'ok', 'ok', 'ok', 'before', 'hold', 'no_render'], st
    ok = [row for row in rows if row['status'] == 'ok']
    assert ok[0]['err_total'] < 0.02 and ok[3]['err_total'] < 0.02, \
        '同一個姿態 vs 20 Hz 取樣後 slerp 的真值：只剩內插誤差（< 0.02°）'
    assert ok[1]['err_roll'] == pytest.approx(4.0, abs=0.02) and abs(ok[1]['err_yaw']) < 0.02 \
        and abs(ok[1]['err_pitch']) < 0.02 and ok[1]['err_total'] == pytest.approx(4.0, abs=0.02)
    assert ok[2]['err_yaw'] == pytest.approx(7.0, abs=0.02) and abs(ok[2]['err_roll']) < 0.02 \
        and abs(ok[2]['err_pitch']) < 0.02
    assert all(row['mode'] == 'AUTO' and row['vt'] == 'VTOL' and row['model'] == 'vtol' for row in ok)
    assert ok[0]['truth_gap_s'] == pytest.approx(0.05, abs=1e-6)
    assert ok[0]['page_att_age_s'] == pytest.approx(-0.04, abs=1e-6)
    # 牆鐘落後：讀回當下（t_disp + 0.13 s）的真值 vs 畫面 ＝ 姿態在 0.13 s 內轉了多少
    for row in (ok[0], ok[3]):
        td, tn = row['t_disp'], row['t_read']
        want = afp.rotation_angle_deg(_ref_dcm(*(lambda r, p, y: (y, p, r))(*att(td))),
                                      _ref_dcm(*(lambda r, p, y: (y, p, r))(*att(tn))))
        assert want > 1.0 and row['lag_err_total'] == pytest.approx(want, abs=0.05), (row['lag_err_total'], want)


def test_compute_rows_raw_truth_drawn_frames_and_mode_timeline():
    frames, att = _frames(n=60)
    # 模式時間軸：t ≥ 1001.5 起變 QLOITER
    frames = [f if f[0] < 1001.5 else f[:2] + ('QLOITER',) + f[3:] for f in frames]
    raw = [(t, 1, 0, *att(t)) for t in (1000.0 + 0.02 * i for i in range(140))]
    reads = []
    for t_disp, tick in ((1001.0, 7), (1001.2, 7), (1002.0, 8)):
        r, p, y = att(t_disp)
        ax = _ref_axes(y, p, r)
        drawn = {'disp': (t_disp - 0.01) * 1000, 'tick': tick, 'tickMs': 0,
                 'ax': _ref_axes(*(lambda rr, pp, yy: (yy, pp, rr + 2.0))(*att(t_disp - 0.01)))}
        reads.append(_read(t_disp + 0.13, t_disp, ax, drawn=drawn))
    rows = afp.compute_rows(reads, frames, raw, warmup_s=0.0)
    assert [row['mode'] for row in rows] == ['AUTO', 'AUTO', 'QLOITER']
    assert all(row['err_raw_total'] is not None and row['err_raw_total'] < 0.05 for row in rows)
    assert rows[0]['drawn_err_roll'] == pytest.approx(2.0, abs=0.02)
    assert rows[1]['drawn_err_total'] is None, '同一幀（tick 相同）的 drawn 只算一次'
    assert rows[2]['drawn_err_roll'] == pytest.approx(2.0, abs=0.02)
    s = afp.link_vs_raw(frames, raw)
    assert s['n'] > 50 and s['max'] < 0.05


def test_summarize_counts_only_ok_rows_and_groups_by_mode_and_speed_band():
    def row(status, mode, speed, e):
        r = {c: None for c in afp.COLUMNS}
        r.update(status=status, mode=mode, speed=speed, sysid=1, nose_flag=True, model='vtol', delay_s=0.13,
                 err_yaw=e, err_pitch=-e, err_roll=2 * e, err_total=3 * e)
        return r
    rows = [row('ok', 'AUTO', 20.0, 1.0), row('ok', 'AUTO', 20.0, -1.0), row('ok', 'QLOITER', 1.0, 0.5),
            row('ok', 'AUTO', 4.0, 0.0), row('warmup', 'AUTO', 20.0, 50.0), row('hold', 'AUTO', 20.0, 50.0)]
    s = afp.summarize(rows, {'extra': 1})
    a = s['errors_deg']['all']
    assert a['n'] == 4 and a['roll']['max'] == 2.0 and a['total']['max'] == 3.0, 'warmup／hold 不進統計'
    assert a['yaw']['bias'] == pytest.approx(0.125) and a['yaw']['p50'] == pytest.approx(0.75)
    assert s['errors_deg']['page_not_holding']['n'] == 4 and s['page_hold_rows'] == 0
    stall = dict(row('ok', 'AUTO', 20.0, 3.0), page_att_age_s=0.08)          # 頁面在 HOLD（批次晚到）
    s2 = afp.summarize(rows + [stall])
    assert s2['errors_deg']['all']['total']['max'] == 9.0 and s2['page_hold_rows'] == 1
    assert s2['errors_deg']['page_not_holding']['n'] == 4 and s2['errors_deg']['page_not_holding']['total']['max'] == 3.0
    assert set(s['errors_deg']['by_mode']) == {'AUTO', 'QLOITER'} and s['errors_deg']['by_mode']['QLOITER']['n'] == 1
    assert set(s['errors_deg']['by_speed_band']) == {'lt3', '3to6', 'ge6'}
    assert s['counts']['by_status'] == {'ok': 4, 'warmup': 1, 'hold': 1} and s['extra'] == 1
    assert afp.speed_band(2.99) == 'lt3' and afp.speed_band(3.0) == '3to6' and afp.speed_band(6.0) == 'ge6'
    assert afp.percentile([1, 2, 3, 4], 0.5) == 2.5 and afp.stats([])['n'] == 0


def test_timeseries_plot_is_written_only_when_there_are_ok_rows(tmp_path):
    pytest.importorskip('matplotlib')
    frames, att = _frames(n=60)
    reads = []
    for i in range(30):
        td = 1000.1 + 0.05 * i
        r, p, y = att(td)
        reads.append(_read(td + 0.13, td, _ref_axes(y, p, r)))
    rows = afp.compute_rows(reads, frames, warmup_s=0.0)
    assert afp.write_plot(rows, tmp_path / 'p.png') and (tmp_path / 'p.png').stat().st_size > 10_000
    assert afp.write_plot([dict(r, status='warmup') for r in rows], tmp_path / 'q.png') is False
    assert not (tmp_path / 'q.png').exists()


def test_square_mission_shape_passes_sitllinks_own_preflight_and_banks_through_90_degree_corners():
    from mission.sitl_link import SITLLink
    items = afp.build_square_mission(23.4518, 120.2863, 60.0, 600.0)
    assert [it['command'] for it in items] == [16, 84, 16, 16, 16, 85]
    assert [it['seq'] for it in items] == list(range(6)) and all(it['frame'] == 3 for it in items[1:])
    norm, err = SITLLink._preflight_mission_items(items)
    assert err is None and len(norm) == 6
    ne = [(math.radians(it['lat'] - 23.4518) * 6378137.0,
           math.radians(it['lon'] - 120.2863) * 6378137.0 * math.cos(math.radians(23.4518))) for it in items]
    assert ne[2] == pytest.approx((600.0, 0.0), abs=0.01) and ne[3] == pytest.approx((600.0, 600.0), abs=0.01)
    assert ne[4] == pytest.approx((0.0, 600.0), abs=0.01) and ne[5] == pytest.approx((0.0, 0.0), abs=1e-6)
    assert math.hypot(*ne[4]) >= 150.0, '最後航點離 VTOL 降落點太近會 PreArm「VTOL land too short」'
    assert items[1]['alt'] == 60.0 and items[-1]['alt'] == 0.0
    plane = afp.build_square_mission(23.4518, 120.2863, 60.0, 600.0, 'PLANE')
    assert [it['command'] for it in plane] == [16, 22, 16, 16, 16, 21] and plane[1]['param1'] == 15.0


def test_page_js_only_uses_names_that_main_html_really_defines():
    """頁面一改名，探針就會靜默讀不到東西 ⇒ 鎖住它依賴的每一個名字；突變的簽名要跟現在的函式一樣。"""
    names = set(re.findall(r'\b(_[A-Za-z][A-Za-z0-9_]*|uavEntities|viewer)\b',
                           afp.READ_JS + afp.INSTALL_JS + afp.PAGE_INFO_JS + afp.READY_JS
                           + afp.NO_IMAGERY_JS + ''.join(afp.NOSE_JS.values()) + ''.join(afp.MUTATIONS.values())))
    names -= {'_afp', '_times'}             # _times：Cesium SampledProperty 的內部陣列（不是頁面的名字）
    defined = {n for n in names if re.search(r'(?:const|let|var|function)\s+' + re.escape(n) + r'\b', HTML)
               or re.search(r'\b' + re.escape(n) + r'\s*[,=]', HTML)}
    assert names - defined == set(), names - defined
    for must in ('_uavDisplayMs', '_UAV_STATE', '_VTOL_HEADING_OFFSET', '_PLANE_ROLL_OFFSET', '_COPTER_PITCH_OFFSET',
                 '_UAV_NOSE_FOLLOWS_TRACK', '_VTOL_MODEL_URI', '_UAV_NOSE_BLEND_LO', '_uavSampledProps'):
        assert must in names, must
    assert 'ori: lin(new Cesium.SampledProperty(Cesium.Quaternion))' in HTML, 'hold-previous 突變改的是 _uavSampledProps().ori'
    sig = re.search(r'function _uavModelOrientation\(([^)]*)\)', HTML).group(1)
    mut = re.search(r'_uavModelOrientation = function \(([^)]*)\)', afp.MUTATIONS['old-angle-addition']).group(1)
    assert sig.replace(' ', '') == mut.replace(' ', '')
    assert 'let _UAV_NOSE_FOLLOWS_TRACK' in HTML, '旗標要是 let（探針要能改）'
    assert '_uavModelOrientation(pos, noseDeg, pitchDeg, rollDeg, headingOffset, pitchOffset, rollOffset)' in HTML, \
        '突變靠 _updateUAVOne 用名字呼叫 _uavModelOrientation'


def test_probe_replaces_the_settings_before_any_ui_import_of_any_kind():
    """不讀 config/settings.json：全域設定在**任何** ui／mission import 之前就換掉（2026-10-22 審查：以前
    `from ui.render_setup import …` 在換設定之前 ⇒ ui/__init__ → ui.main_window 的模組層 get_settings() 已經讀了檔）。
    行為見下一個測試（子行程＋稽核掛鉤）；這裡鎖原始碼順序。"""
    import inspect
    boot = inspect.getsource(afp.bootstrap_modules)
    swap = boot.index("cs._global_settings = cs.GlobalSettings(config_file=str(out / '_no_settings.json'))")
    first_ui = min(m.start() for m in re.finditer(r'^\s*(?:from|import)\s+(?:ui|mission)\b', boot, re.M))
    assert swap < first_ui, '換設定要在第一個 ui／mission import 之前'
    assert boot.index("tc._TEMP_HTML_DIR = out / '_page'") < first_ui
    main = inspect.getsource(afp.main)
    call = main.index('mods = bootstrap_modules(out, args.model_cache)')
    for m in re.finditer(r'^\s*(?:from|import)\s+(?:ui|mission|config|utils)\b', main, re.M):
        assert m.start() > call, f'main() 裡不可以在 bootstrap_modules 之前 import 專案模組：{m.group(0)!r}'
    assert not re.search(r'^\s*(?:from|import)\s+(?:ui|mission|config)\b', inspect.getsource(afp).split('def main(')[0]
                         .split('def bootstrap_modules(')[0], re.M), '模組層不可以 import ui／mission／config'
    src = (ROOT / 'tools' / 'attitude_fidelity_probe.py').read_text(encoding='utf-8')
    assert "_resolve_token = lambda self: ''" in src and 'get_settings(' not in src


_AUDIT_CHILD = r'''
import json, os, pathlib, sys
root, out, mode = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2]), sys.argv[3]
sys.path.insert(0, str(root))
target = os.path.normcase(os.path.abspath(root / 'config' / 'settings.json'))
hits = []
def hook(ev, args):
    if ev == 'open' and args and isinstance(args[0], (str, bytes, os.PathLike)):
        try:
            p = os.path.normcase(os.path.abspath(os.fsdecode(args[0])))
        except Exception:
            return
        if p == target:
            hits.append(p)
            raise PermissionError('test: settings.json blocked')
sys.addaudithook(hook)
res = {}
if mode == 'probe':
    from tools import attitude_fidelity_probe as afp
    mods = afp.bootstrap_modules(out)
    import config.settings as cs
    res.update(same=mods.uimw.settings is cs._global_settings, cfg=cs._global_settings.config_file,
               page_dir=str(mods.tc._TEMP_HTML_DIR))
else:                                   # 對照：舊的順序（先 import ui.render_setup）
    from ui.render_setup import configure_qt_presentation  # noqa: F401
res['hits'] = len(hits)
print('RESULT ' + json.dumps(res))
'''


def test_probe_bootstrap_never_opens_the_real_settings_file_and_the_old_order_did(tmp_path):
    """子行程＋稽核掛鉤（open 到 config/settings.json 就記下並擋掉，內容一個字都不讀）：bootstrap_modules ⇒ 0 次，
    而且 ui.main_window.settings 就是換上去的那一份；對照組（舊順序：先 import ui.render_setup）⇒ 掛鉤抓得到
    （這台機器有那個檔才比得了 —— 只看存不存在，不讀）。"""
    child = tmp_path / 'child.py'
    child.write_text(_AUDIT_CHILD, encoding='utf-8')

    def run(mode):
        r = subprocess.run([sys.executable, str(child), str(ROOT), str(tmp_path / 'out'), mode],
                           env=dict(os.environ, QT_QPA_PLATFORM='offscreen', **_ENV), cwd=str(tmp_path),
                           capture_output=True, text=True, encoding='utf-8', errors='replace', timeout=180)
        line = [x for x in r.stdout.splitlines() if x.startswith('RESULT ')]
        assert r.returncode == 0 and line, r.stdout[-2000:] + r.stderr[-2000:]
        return json.loads(line[-1][len('RESULT '):])
    got = run('probe')
    assert got['hits'] == 0, '探針不可以去開 config/settings.json'
    assert got['same'] is True, 'ui.main_window 模組層的 settings 要是換上去的那一份'
    assert pathlib.Path(got['cfg']) == tmp_path / 'out' / '_no_settings.json'
    assert pathlib.Path(got['page_dir']) == tmp_path / 'out' / '_page'
    if (ROOT / 'config' / 'settings.json').exists():
        assert run('control')['hits'] >= 1, '對照組（舊順序）要被掛鉤抓到，否則上面的 0 次不代表什麼'


def test_gui_stub_uses_the_real_mainwindow_methods_and_produces_the_real_batch(monkeypatch):
    import ui.main_window as mw
    from ui.main_window import MainWindow
    from mission.sitl_link import TelemetryFrame
    fired = []
    monkeypatch.setattr(mw.QTimer, 'singleShot', staticmethod(lambda ms, *a: fired.append((ms, a[-1]))))
    monkeypatch.setattr(mw, 'TELEMETRY_FLUSH_HZ', 30.0)
    sink, logs = [], []

    class _W:
        def update_uav_positions(self, batch):
            sink.append(('widget', list(batch)))
    rec = afp._MapRecorder(_W(), [])
    link = type('L', (), {'sysid_label': 1})()
    gui = afp.make_gui_stub(MainWindow, rec, [link], logs.append)
    for name in afp._BOUND_METHODS:
        assert getattr(gui, name).__func__ is getattr(MainWindow, name), name
    assert gui._uav_batch_item is MainWindow._uav_batch_item
    f = TelemetryFrame(sysid=1, lat=23.45, lon=120.28, alt_rel=80.0, heading=0.0, yaw=-30.0, ground_speed=18.0,
                       vel_n=10.0, vel_e=10.0, gps_fix=3, mode='AUTO', armed=True, vehicle_type='VTOL', roll=12.5,
                       pitch=-3.25, pos_t=1_800_000_000.123, att_t=1_800_000_000.456)
    gui._on_sitl_telemetry(f)
    gui._on_sitl_telemetry(f)
    assert len(fired) == 1, '合幀：一個窗口只排一次'
    fired.pop()[1]()                                          # 計時到 ⇒ 真的 _flush_telemetry
    assert len(sink) == 1 and len(rec.sink) == 1
    it = sink[0][1][0]
    assert it == MainWindow._uav_batch_item(f)
    assert it['hdg'] == pytest.approx(330.0) and it['crs'] == pytest.approx(45.0) and it['roll'] == 12.5
    assert it['at'] == 1_800_000_000_456 and it['pt'] == 1_800_000_000_123
    gui._on_sitl_telemetry(TelemetryFrame(sysid=1, lat=23.45, lon=120.28, gps_fix=3, armed=False))
    fired.pop()[1]()
    assert any('已上鎖' in m for m in logs), '解鎖→上鎖：真的 _note_armed_transition（樁 HUD 收到）'


# ── 2026-10-22 審查後：開機時間軸、每幀、閘門、校正、萬向鎖 ─────────────────────────────
def _sine(t):
    """(r, p, y)：一直在彎（和 tests/fake_mav_vehicle.sine_attitude 同形）。"""
    w = 2 * math.pi * 0.5
    return 45.0 * math.sin(w * t), 20.0 * math.sin(w * t + 1.0), (350.0 + 40.0 * math.sin(2 * math.pi * 0.2 * t)) % 360


def _link_frames(vehicle_t, recv_t, att=_sine, t_base=1000.0, sid=1, with_boot=True, roll_sign=1.0):
    """FRAME_FIELDS 元組（17 欄）：飛控在 vehicle_t 取樣（time_boot_ms = vehicle_t − t_base）、鏈路在 recv_t 收到。"""
    out = []
    for tv, tr in zip(vehicle_t, recv_t):
        r, p, y = att(tv)
        out.append((tr, sid, 'AUTO', True, 18.0, 'VTOL', tr, roll_sign * r, p, afp.wrap180(y), 23.45, 120.28, 100.0, 3,
                    18.0, 0.0, int(round((tv - t_base) * 1000)) if with_boot else None))
    return out


def _page_like_link(frames, t_disps, sid=1):
    """模擬一個「完全照鏈路收到時刻內插」的頁面：每個顯示時間畫鏈路真值（= 頁面拿 att_t 當樣本時間的行為）。"""
    ts = afp.TruthSeries((f[6], f[7], f[8], f[9]) for f in frames)
    recs = []
    for td in t_disps:
        q, st, _ = ts.at(td)
        y, p, r = afp.euler_from_dcm(afp.dcm_from_quat(q))
        recs.append({'ms': (td + 0.13) * 1000.0, 'disp': td * 1000.0, 'tick': len(recs), 'delay': 0.13, 'kind': 'vtol',
                     'lastAttMs': (td + 0.05) * 1000.0, 'ax': _ref_axes(y, p, r)})
    return recs


def test_lower_envelope_recovers_the_vehicle_clock_through_jitter_bursts_and_speedup():
    rnd = random.Random(20261022)
    for speedup in (1.0, 4.0):
        boot = [10.0 + 0.05 * i for i in range(400)]
        recv = [1000.0 + b / speedup + 0.002 + rnd.expovariate(1 / 0.003) for b in boot]
        for i in range(150, 156):                          # 成批：6 則一起到
            recv[i] = recv[155] + 0.0001 * (i - 150)
        for i in range(0, 30):                             # 連線那一下的積壓
            recv[i] = recv[30]
        a, b = afp.fit_lower_envelope(boot, recv)
        assert b == pytest.approx(1 / speedup, rel=2e-4), (speedup, b)
        for x in (boot[0], boot[200], boot[-1]):
            assert a + b * x == pytest.approx(1000.0 + x / speedup + 0.002, abs=5e-4), '線 = 最快那幾則（最小延遲）'
        assert all(r >= a + b * x - 1e-9 for x, r in zip(boot, recv)), '每一點都在線上或上面'
    assert afp.fit_lower_envelope([1.0], [2.0]) is None and afp.fit_lower_envelope([1.0, 1.0], [2.0, 3.0]) is None


def test_boot_clock_series_counts_compressed_gaps_and_bursts_after_the_start():
    vt = [1000.0 + 0.05 * i for i in range(200)]                       # 飛控 20 Hz、10 s
    rt = [t + 0.001 for t in vt]
    for i in range(0, 30):                                             # 連線積壓：前 1.5 s 一起到
        rt[i] = vt[30] + 0.001
    for i in range(100, 105):                                          # 5.0–5.2 s 被扣住，5.25 s 一起到
        rt[i] = vt[105] + 0.0005 * (i - 100)
    samples = [(r_, v - 1000.0, *_sine(v)) for v, r_ in zip(vt, rt)]
    _ts, st = afp.boot_clock_series(samples, skip_first_s=2.0)
    assert st['compressed_gaps'] == 4 and st['bursts'] == 1, st
    assert st['latency_above_min_s']['max'] == pytest.approx(0.25, abs=0.003)
    assert st['latency_above_min_s']['p50'] < 0.0005
    assert st['latency_above_min_s_incl_start']['max'] == pytest.approx(1.5, abs=0.003), '連線積壓另外記'
    _ts, st0 = afp.boot_clock_series(samples)
    assert st0['compressed_gaps'] == 4 + 30 and st0['bursts'] == 2, '積壓的 30 則和第 31 則同時到 ⇒ 30 個間隔'
    assert len(st0['segments']) == 1 and st0['segments'][0]['b'] == pytest.approx(1.0, abs=1e-6)
    reboot = samples + [(rt[-1] + 1.0 + 0.05 * k, 0.5 + 0.05 * k, 0.0, 0.0, 0.0) for k in range(20)]
    assert len(afp.boot_clock_series(reboot)[1]['segments']) == 2, '開機時間倒退（飛控重開機）⇒ 分段各自擬合'


def test_boot_clock_truth_sees_bursts_that_receipt_time_truth_cannot():
    """2026-10-22 審查：真值的時間就是頁面的樣本時間（收到時刻）⇒ 一個「照收到時刻內插」的頁面對鏈路真值永遠 0 誤差，
    成批到也一樣。開機時間軸的真值才看得到。"""
    vt = [1000.0 + 0.05 * i for i in range(200)]
    rt = [t + 0.001 for t in vt]
    for i in range(100, 105):
        rt[i] = vt[105] + 0.0005 * (i - 100)
    frames = _link_frames(vt, rt)
    t_disps = [1004.6 + k / 60.0 for k in range(60)]                  # 4.6–5.6 s，每秒 60 幀
    reads = [(1006.0, {'flag': False, 'uav': {'1': dict(_read(1006.0, 1005.87, None)[1]['uav']['1'],
                                                        frames=_page_like_link(frames, t_disps))}})]
    rows = afp.compute_rows(reads, frames, warmup_s=0.0)
    fr = [r for r in rows if r['row_kind'] == 'frame' and r['status'] == 'ok']
    assert len(fr) == 60
    assert max(r['err_total'] for r in fr) < 1e-3, '對收到時刻的真值：看不到成批'
    assert max(r['err_boot_total'] for r in fr) > 10.0, '對開機時間軸的真值：成批那 0.25 s 畫的是舊姿態'
    calm = [r for r in fr if r['t_disp'] < 1004.95]
    assert calm and max(r['err_boot_total'] for r in calm) < 0.1, '沒成批的地方兩者一致（只差 1 ms 的最小延遲）'
    s = afp.summarize(rows)
    assert s['errors_per_frame_deg']['all']['total']['max'] < 1e-3 and s['errors_vs_boot_clock_deg']['total']['max'] > 10


def test_raw_boot_truth_sees_a_slow_attitude_stream_that_link_truth_cannot():
    """鏈路只送 4 Hz 的姿態（例如序列埠數傳）：頁面照鏈路樣本內插 ⇒ 對鏈路真值（同一組樣本）0 誤差；
    獨立鏈路 50 Hz 的開機時間軸真值看到弦誤差（一直在彎 ⇒ 幾度）。"""
    vt = [1000.0 + 0.25 * i for i in range(40)]
    frames = _link_frames(vt, [t + 0.001 for t in vt])
    raw = [(1000.0 + 0.02 * i + 0.001, 1, int(round(0.02 * i * 1000)), *_sine(1000.0 + 0.02 * i)) for i in range(500)]
    t_disps = [1002.0 + k / 60.0 for k in range(240)]
    reads = [(1007.0, {'flag': False, 'uav': {'1': dict(_read(1007.0, 1006.87, None)[1]['uav']['1'],
                                                        frames=_page_like_link(frames, t_disps))}})]
    rows = [r for r in afp.compute_rows(reads, frames, raw, warmup_s=0.0) if r['row_kind'] == 'frame']
    assert max(r['err_total'] for r in rows) < 1e-3 and max(r['err_boot_total'] for r in rows) < 1e-3
    assert max(r['err_rawboot_total'] for r in rows) > 2.0, '獨立 50 Hz 真值看得到 4 Hz 的弦誤差'


def test_link_vs_raw_boot_isolates_a_sitllink_decode_error_from_all_clock_effects():
    vt = [1000.0 + 0.05 * i for i in range(100)]
    raw = [(1000.0 + 0.02 * i + 0.0007, 1, int(round(0.02 * i * 1000)), *_sine(1000.0 + 0.02 * i)) for i in range(260)]
    good = _link_frames(vt, [t + 0.004 for t in vt])                    # 不同的延遲：開機時間軸上不影響
    assert afp.link_vs_raw_boot(good, raw)['p95'] < 0.2
    bad = _link_frames(vt, [t + 0.004 for t in vt], roll_sign=-1.0)    # SITLLink 滾轉正負號錯
    s = afp.link_vs_raw_boot(bad, raw)
    assert s['p95'] > 60.0 and s['n'] > 80
    assert afp.link_vs_raw_boot(_link_frames(vt, vt, with_boot=False), raw)['n'] == 0


def test_frames_become_frame_rows_with_phases_and_the_drawn_matrix_of_that_same_frame():
    frames, att = _frames(n=80)
    recs = []
    for k in range(40):
        td = 1001.0 + 0.0123 * k
        r, p, y = att(td)
        recs.append({'ms': (td + 0.13) * 1000, 'disp': td * 1000, 'tick': k, 'delay': 0.13, 'kind': 'vtol',
                     'lastAttMs': (td + 0.04) * 1000, 'ax': _ref_axes(y, p, r), 'dax': _ref_axes(y, p, r + 1.5)})
    reads = [(1002.0, {'ms': 1001.9e3, 'flag': False,
                       'uav': {'1': dict(_read(1001.9, 1001.77, _ref_axes(*(lambda r, p, y: (y, p, r))(*att(1001.77))))[1]
                                         ['uav']['1'], ms=1001.9e3, frames=recs)}})]
    rows = afp.compute_rows(reads, frames, warmup_s=0.0)
    assert [r['row_kind'] for r in rows] == ['frame'] * 40 + ['read'], '那次讀回之前的幀先、讀回那一列最後'
    fr = rows[:40]
    assert all(r['status'] == 'ok' and r['err_total'] < 0.02 for r in fr)
    assert all(r['drawn_err_roll'] == pytest.approx(1.5, abs=0.02) and r['drawn_tick'] == i for i, r in enumerate(fr))
    assert rows[-1]['t_read'] == pytest.approx(1001.9), '讀回的「現在」= JS 執行當下（u.ms），不是 Python 收到回呼的時刻'
    h = afp.summarize(rows)['phase_histogram']
    assert sum(h['frames']) == 40 and sum(1 for c in h['frames'] if c) >= 8, h
    assert afp.summarize(rows)['errors_drawn_frame_deg']['n'] == 40


def test_read_now_is_the_js_time_for_wallclock_lag():
    frames, att = _frames(n=80)
    td, js_now = 1001.5, 1001.63
    r, p, y = att(td)
    data = {'ms': js_now * 1000.0, 'flag': False, 'uav': {'1': dict(_read(0, td, _ref_axes(y, p, r))[1]['uav']['1'])}}
    data['uav']['1'].pop('delay')
    row = afp.compute_rows([(js_now + 0.4, data)], frames, warmup_s=0.0)[0]       # Python 收到回呼晚了 0.4 s
    want = afp.rotation_angle_deg(_ref_dcm(*(lambda r, p, y: (y, p, r))(*att(td))),
                                  _ref_dcm(*(lambda r, p, y: (y, p, r))(*att(js_now))))
    assert row['t_read'] == pytest.approx(js_now) and row['lag_err_total'] == pytest.approx(want, abs=0.05)
    assert afp._read_time(5.0, {}) == 5.0 and afp._read_time(5.0, {'ms': 4000.0}) == 4.0


def test_gimbal_lock_rows_are_left_out_of_the_yaw_and_roll_stats_only():
    def row(tp, e):
        r = {c: None for c in afp.COLUMNS}
        r.update(status='ok', truth_pitch=tp, err_yaw=e, err_pitch=0.1, err_roll=e, err_total=0.2)
        return r
    b = afp._err_block([row(10.0, 1.0), row(89.0, 170.0), row(-88.0, 160.0)])
    assert b['yaw']['max'] == 1.0 and b['roll']['n'] == 1 and b['gimbal_rows_skipped'] == 2
    assert b['total']['n'] == 3 and b['pitch']['n'] == 3


def test_calibration_offsets_are_pinned_in_the_probe_and_a_changed_page_constant_is_reported():
    assert afp.PINNED_OFFSETS == {'vtol': (-90.0, 90.0, 180.0), 'plane': (0.0, 0.0, 0.0),
                                  'copter': (-90.0, 90.0, 180.0)}, '使用者目視確認、GLB 幾何稽核證實的那幾組'
    assert json.dumps({k: list(v) for k, v in afp.PINNED_OFFSETS.items()}) in afp.INSTALL_JS
    assert 'offs: A.pinned[K[i][0]]' in afp.INSTALL_JS, '去校正用寫死的，不用頁面常數（頁面常數只回報）'
    ok = afp.calibration_check({'offsets': {'vtol': [-90, 90, 180], 'plane': [0, 0, 0], 'copter': [-90, 90, 180]}})
    assert ok['mismatch'] == []
    bad = afp.calibration_check({'offsets': {'vtol': [-90, 90, 0], 'plane': [0, 0, 0], 'copter': [-90, 90, 180]}})
    assert bad['mismatch'] == [{'kind': 'vtol', 'page': [-90, 90, 0], 'pinned': [-90.0, 90.0, 180.0]}]
    assert afp.calibration_check({})['mismatch'] != [], '沒讀到頁面常數也算不過'
    # main.html 本身：VTOL 的偏移由 widget 常數填，固定翼／多旋翼是頁面常數 —— 兩邊都要等於寫死的
    from ui.widgets.cesium_map_widget import CesiumMapWidget as W
    assert (W.VTOL_HEADING_OFFSET, W.VTOL_PITCH_OFFSET, W.VTOL_ROLL_OFFSET) == afp.PINNED_OFFSETS['vtol']
    for kind, name in (('plane', 'PLANE'), ('copter', 'COPTER')):
        got = tuple(float(re.search(rf'const _{name}_{ax}_OFFSET\s*=\s*(-?\d+(?:\.\d+)?);', HTML).group(1))
                    for ax in ('HEADING', 'PITCH', 'ROLL'))
        assert got == afp.PINNED_OFFSETS[kind], (kind, got)


def _gate_summary(**over):
    blk = lambda p95, n=100: {'n': n, 'total': {'p95': p95}}           # noqa: E731
    s = {'errors_deg': {'all': blk(0.05)}, 'errors_per_frame_deg': {'all': blk(0.05)},
         'errors_vs_boot_clock_deg': blk(0.1), 'errors_drawn_frame_deg': blk(0.1),
         'errors_vs_raw_truth_deg': blk(0.1), 'errors_vs_raw_boot_clock_deg': blk(0.1),
         'raw_truth': {'link_vs_raw_boot_deg': {'n': 100, 'p95': 0.01}},
         'counts': {'by_status': {'ok': 200, 'warmup': 50, 'before': 2}, 'frames_by_status': {'ok': 600, 'warmup': 100}},
         'models': ['vtol'], 'page': {'uris': {'vtol': 'file:///v.glb', 'plane': '', 'copter': ''}},
         'calibration': {'mismatch': []}, 'display_delay_s': {'p95': 0.14}}
    for k, v in over.items():
        cur = s
        *path, last = k.split('__')
        for p in path:
            cur = cur[p]
        cur[last] = v
    return s


def _failed(g):
    return sorted(c['name'] for c in g['checks'] if not c['ok'])


def test_gate_is_a_conjunction_of_every_error_coverage_model_and_calibration_check():
    """2026-10-22 審查：以前 --fail-over 只看 errors_deg.all 的 p95。"""
    g = afp.evaluate_gate(_gate_summary(), 0.5)
    assert g['ok'] and _failed(g) == [], g
    cases = {
        'link_vs_raw_boot_deg.p95': _gate_summary(raw_truth={'link_vs_raw_boot_deg': {'n': 100, 'p95': 90.0}}),
        'errors_per_frame_deg.all.p95': _gate_summary(errors_per_frame_deg={'all': {'n': 100, 'total': {'p95': 2.9}}}),
        'errors_vs_boot_clock_deg.p95': _gate_summary(errors_vs_boot_clock_deg={'n': 100, 'total': {'p95': 13.7}}),
        'errors_drawn_frame_deg.p95': _gate_summary(errors_drawn_frame_deg={'n': 0}),
        'coverage.frames.ok_frac': _gate_summary(counts__frames_by_status={'ok': 100, 'hold': 400, 'warmup': 10}),
        'coverage.reads.n_ok': _gate_summary(counts__by_status={'ok': 10}),
        'models': _gate_summary(models=['unknown', 'vtol']),
        'calibration.mismatch': _gate_summary(calibration={'mismatch': [{'kind': 'vtol'}]}),
    }
    for name, s in cases.items():
        g = afp.evaluate_gate(s, 0.5)
        assert not g['ok'] and _failed(g) == [name], (name, _failed(g))
    # 姿態串流斷掉：之後全是 hold（以前被排除在統計外 ⇒ 照樣過）
    assert 'coverage.reads.ok_frac' in _failed(afp.evaluate_gate(
        _gate_summary(counts__by_status={'ok': 200, 'hold': 300}), 0.5))
    # 沒有獨立真值卻要閘門 ⇒ 不過；不要求時那幾項不算
    no_raw = _gate_summary(errors_vs_raw_truth_deg={'n': 0}, errors_vs_raw_boot_clock_deg={'n': 0}, raw_truth=None)
    assert _failed(afp.evaluate_gate(no_raw, 0.5)) == ['errors_vs_raw_boot_clock_deg.p95', 'link_vs_raw_boot_deg.p95']
    assert afp.evaluate_gate(no_raw, 0.5, need_raw=False)['ok']
    # 2026-10-22：獨立真值放在它自己的收到時刻上（0～32 ms 抖動）只報告、不進閘門；判準是飛控時間軸，而且逐階段
    assert afp.evaluate_gate(_gate_summary(errors_vs_raw_truth_deg={'n': 100, 'total': {'p95': 3.4}}), 0.5)['ok']
    blk = lambda p95, n=100: {'n': n, 'total': {'p95': p95}}           # noqa: E731
    ph = _gate_summary(errors_vs_raw_boot_clock_by_phase={'hover': blk(0.9), 'transition': blk(2.0, n=10),
                                                           'cruise': blk(0.2)})
    g = afp.evaluate_gate(ph, 0.5)
    assert _failed(g) == ['errors_vs_raw_boot_clock_deg.hover.p95'], _failed(g)
    assert 'errors_vs_raw_boot_clock_deg.cruise.p95' in [c['name'] for c in g['checks']]
    assert 'errors_vs_raw_boot_clock_deg.transition.p95' not in [c['name'] for c in g['checks']], \
        '列數不到 --min-rows 的階段只報告'
    # 沒有 GLB（退回方塊）⇒ 找不到 primitive、沒有 drawn 也可以；有 GLB 卻是 none ⇒ 不過
    box = _gate_summary(models=['none'], errors_drawn_frame_deg={'n': 0}, page={'uris': {'vtol': '', 'plane': ''}})
    assert afp.evaluate_gate(box, 0.5)['ok']
    assert _failed(afp.evaluate_gate(_gate_summary(models=['none', 'vtol']), 0.5)) == ['models']
    assert _failed(afp.evaluate_gate(_gate_summary(), 0.5, max_delay_s=0.1)) == ['display_delay_s.p95']


def test_fail_over_requires_independent_truth():
    with pytest.raises(SystemExit):
        afp.parse_args(['--conn', 'tcp:127.0.0.1:1', '--fail-over', '1.0'])
    a = afp.parse_args(['--conn', 'tcp:127.0.0.1:1', '--truth', 'tcp:127.0.0.1:2', '--fail-over', '1.0'])
    assert a.fail_over == 1.0 and a.period_jitter == 0.4 and a.min_rows == 50 and a.model_cache == ''
    with pytest.raises(SystemExit):
        afp.parse_args(['--conn', 'x', '--period-jitter', '1.5'])


def test_truth_tap_records_boot_ms_only_for_the_attitude_message_that_produced_the_frame():
    from mission.sitl_link import TelemetryFrame

    class _Att:
        def __init__(self, src, r, p, y, boot):
            self.src, self.roll, self.pitch, self.yaw, self.time_boot_ms = src, r, p, y, boot

        def get_srcSystem(self):
            return self.src
    msgs = {}
    link = type('L', (), {})()
    link._mav = type('M', (), {'messages': msgs})()
    tap = afp._TruthTap(link)
    r, p, y = 0.3, -0.1, 2.0
    msgs['ATTITUDE'] = _Att(1, r, p, y, 12345)
    f = TelemetryFrame(sysid=1, roll=math.degrees(r), pitch=math.degrees(p), yaw=math.degrees(y), att_t=1000.5)
    tap.on_frame(f)
    tap.on_frame(f)                                    # 同一個 att_t（GLOBAL_POSITION_INT 那一幀）⇒ 不記
    tap.on_frame(TelemetryFrame(sysid=1, roll=1.0, pitch=2.0, yaw=3.0, att_t=1000.55))   # 值對不上 ⇒ None
    msgs['ATTITUDE'] = _Att(2, r, p, y, 999)
    tap.on_frame(TelemetryFrame(sysid=1, roll=math.degrees(r), pitch=math.degrees(p), yaw=math.degrees(y),
                                att_t=1000.6))        # 別架的 ATTITUDE ⇒ None
    assert [fr[16] for fr in tap.frames] == [12345, None, None, None]
    assert len(tap.frames[0]) == len(afp.FRAME_FIELDS)
    plain = afp._TruthTap()                            # 沒有 link（舊用法）⇒ 照記、開機時間 None
    plain.on_frame(f)
    assert len(plain.frames) == 1 and plain.frames[0][16] is None


def test_link_truth_sits_on_the_sample_time_the_page_was_given():
    """2026-10-22：頁面的姿態樣本時間 = SITLLink 的 att_ts（飛控時間換到牆鐘）⇒ 鏈路真值也放在 att_ts 上（只驗畫法＋插值
    ＋合幀）；舊資料（17 欄、沒有 att_ts）退回收到時刻。"""
    vt = [1000.0 + 0.02 * i for i in range(200)]
    rt = [t + 0.004 + 0.03 * ((i * 7) % 11) / 11 for i, t in enumerate(vt)]          # 收到時刻：0～30 ms 抖動
    frames = [f + (tv + 0.004,) for f, tv in zip(_link_frames(vt, rt), vt)]           # att_ts = 最小延遲線
    assert len(frames[0]) == len(afp.FRAME_FIELDS) and afp.FRAME_FIELDS[-1] == 'att_ts'
    assert afp.page_sample_t(dict(zip(afp.FRAME_FIELDS, frames[0]))) == pytest.approx(vt[0] + 0.004)
    assert afp.page_sample_t(dict(zip(afp.FRAME_FIELDS, frames[0][:17]))) == rt[0], '沒有 att_ts ⇒ 收到時刻'
    assert afp.page_sample_t({'att_t': 5.0, 'att_ts': 0.0}) == 5.0
    t_disps = [1001.0 + k / 60.0 for k in range(120)]
    page = []                                         # 頁面照 att_ts 內插（新的 SITLLink＋合幀）
    ts = afp.TruthSeries((f[17], f[7], f[8], f[9]) for f in frames)
    for td in t_disps:
        q, _st, _ = ts.at(td)
        y, p_, r = afp.euler_from_dcm(afp.dcm_from_quat(q))
        page.append({'ms': (td + 0.13) * 1000.0, 'disp': td * 1000.0, 'tick': len(page), 'delay': 0.13, 'kind': 'vtol',
                     'lastAttMs': (td + 0.05) * 1000.0, 'ax': _ref_axes(y, p_, r)})
    reads = [(1004.0, {'flag': False, 'uav': {'1': dict(_read(1004.0, 1003.87, None)[1]['uav']['1'], frames=page)}})]
    rows = [r for r in afp.compute_rows(reads, frames, warmup_s=0.0) if r['row_kind'] == 'frame' and r['status'] == 'ok']
    assert len(rows) == 120
    assert max(r['err_total'] for r in rows) < 1e-3, '鏈路真值和頁面同一個時間軸'
    assert max(r['err_boot_total'] for r in rows) < 0.05, '飛控時間軸也對（att_ts 就在最小延遲線上）'


def test_phase_offset_and_common_boot_samples():
    assert afp.phase_offset_ms([4, 24, 44, 64, 84], [104, 124, 144, 164, 184]) == (20, 0)
    assert afp.phase_offset_ms([14, 34, 54, 74, 94], [104, 124, 144, 164, 184]) == (20, 10)
    assert afp.phase_offset_ms([3, 24, 44, 64, 84, 104], [104, 124, 144, 164, 184]) == (20, 0), '間隔用中位數'
    assert afp.phase_offset_ms([4, 54, 104, 154, 204], [104, 124, 144, 164, 184]) is None, '間隔不同（20 Hz 對 50 Hz）'
    assert afp.phase_offset_ms([4, 24], [104, 124, 144, 164, 184]) is None, '樣本不夠'
    vt = [1000.0 + 0.02 * i for i in range(100)]
    link = _link_frames(vt, vt)                                    # 開機 0、20、40… ms
    same = [(t, 1, int(round((t - 1000.0) * 1000)), 0.0, 0.0, 0.0) for t in vt]
    shifted = [(t + 0.01, 1, int(round((t - 1000.0) * 1000)) + 10, 0.0, 0.0, 0.0) for t in vt]
    assert afp.common_boot_fraction(link, same)['1']['frac'] == 1.0
    c = afp.common_boot_fraction(link, shifted)['1']
    assert c['frac'] == 0.0 and c['raw_in_range'] == 99 and c['link_n'] == 100
    assert afp.common_boot_fraction(link, same, skip_first_s=1.0)['1']['raw_in_range'] == 50
    assert afp.common_boot_fraction(_link_frames(vt, vt, with_boot=False), same) == {}


def test_raw_boot_errors_are_also_reported_and_gated_per_flight_phase():
    def row(speed, e, mode='QLOITER'):
        r = {c: None for c in afp.COLUMNS}
        r.update(row_kind='frame', status='ok', speed=speed, mode=mode, model='vtol', err_rawboot_total=e,
                 err_rawboot_yaw=0.0, err_rawboot_pitch=e, err_rawboot_roll=0.0, err_boot_total=e / 2,
                 err_boot_yaw=0.0, err_boot_pitch=e / 2, err_boot_roll=0.0, truth_pitch=0.0)
        return r
    rows = [row(0.5, 0.3)] * 60 + [row(4.0, 2.0, 'AUTO')] * 5 + [row(20.0, 0.1, 'AUTO')] * 60
    s = afp.summarize(rows)
    ph = s['errors_vs_raw_boot_clock_by_phase']
    assert set(ph) == {'hover', 'transition', 'cruise'}
    assert ph['hover']['n'] == 60 and ph['hover']['total']['p95'] == pytest.approx(0.3)
    assert ph['transition']['n'] == 5 and ph['cruise']['total']['p95'] == pytest.approx(0.1)
    assert s['errors_vs_raw_boot_clock_by_mode']['AUTO']['n'] == 65
    assert s['errors_vs_boot_clock_by_phase']['hover']['total']['p95'] == pytest.approx(0.15)
    assert afp.PHASE_OF_BAND == {'lt3': 'hover', '3to6': 'transition', 'ge6': 'cruise'}


# ═══════════════════════════════════════════════════════════════════════
#  B. 假飛控 × 真 SITLLink（真 socket、不開視窗）
# ═══════════════════════════════════════════════════════════════════════
@pytest.fixture
def qcore():
    from PyQt6.QtCore import QCoreApplication
    return QCoreApplication.instance() or QCoreApplication([])


def _wait(cond, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


def _link_to(fake, qcore, hint='VTOL'):
    from PyQt6.QtCore import Qt
    from mission.sitl_link import SITLLink
    link = SITLLink(f'tcp:127.0.0.1:{fake.ports[0]}', sysid_label=1, vehicle_hint=hint)
    tap, texts, uploads = afp._TruthTap(link), [], []
    link.telemetry.connect(tap.on_frame, Qt.ConnectionType.DirectConnection)
    link.status_text.connect(lambda s, t: texts.append(t), Qt.ConnectionType.DirectConnection)
    link.mission_uploaded.connect(lambda s, ok, m: uploads.append((ok, m)), Qt.ConnectionType.DirectConnection)
    link.start()
    return link, tap, texts, uploads


def _stop(link, fake):
    link.stop()
    link.wait(5000)
    fake.stop()


def test_fake_vehicle_through_the_real_sitllink_attitude_arrives_exactly_at_the_requested_rate(qcore):
    from tests.fake_mav_vehicle import FakeVehicle, scripted_attitude
    fake = FakeVehicle(ports=(0,), crab_deg=10.0).start()
    link, tap, texts, _ = _link_to(fake, qcore)
    try:
        # 50 Hz（2026-10-22 起）：等 ≥ 200 則 ≈ 4 s，扣掉第一秒還有 ~150 則量頻率
        assert _wait(lambda: len({fr[6] for fr in tap.frames if fr[6] > 0}) >= 200, 20.0), '沒有收到姿態'
        t_req = next(fr[0] for fr in tap.frames if fr[6] > 0)
        f = tap.latest[1]
        assert f.vehicle_type == 'VTOL' and f.mode == 'AUTO' and f.armed and f.gps_fix == 3
        assert any('Q_ENABLE=1' in t for t in texts)
        att_t = sorted({fr[6] for fr in tap.frames if fr[6] > 0 and fr[0] > t_req + 1.0})
        rate = (len(att_t) - 1) / (att_t[-1] - att_t[0])
        assert 45.0 < rate < 55.0, \
            f'SITLLink 另外用 SET_MESSAGE_INTERVAL 要 50 Hz 姿態（ATTITUDE_HZ），假飛控有照做：{rate:.1f} Hz'
        # 送出的每一則（主鏈路）與 SITLLink 解出來的值逐一相同（float32 弧度 ⇒ 1e-4° 以內）
        sent = [(r, p, y) for idx, _t, _b, r, p, y in list(fake.att_log) if idx == 0]
        got = {fr[6]: fr[7:10] for fr in tap.frames if fr[6] > 0}
        for r, p, y in got.values():
            err = min(max(abs(r - a), abs(p - b), abs(afp.wrap180(y - c))) for a, b, c in sent)
            assert err < 1e-4, f'收到的姿態要是某一則送出去的（float32 弧度 ⇒ < 1e-4°）：{(r, p, y)} 差 {err}'
            assert -180.0 <= y <= 180.0, 'ATTITUDE yaw 是 ±180（_uav_batch_item 才轉成 0–360）'
        assert len(got) > 60
        # 劇本跨北：收到的偏航有 > 340 也有 < 20
        ys = [v[2] % 360.0 for v in got.values()]
        assert any(y > 340 for y in ys) and any(y < 20 for y in ys)
        assert scripted_attitude(3.25) == pytest.approx((45.0, 0.0, 20.0))
        # 航跡 = 偏航 + 10°（側風偏流）：由 NED 速度算（中位數：位置與姿態是兩則訊息，轉彎中差一點）
        d = sorted(afp.wrap180(math.degrees(math.atan2(fr[15], fr[14])) - fr[9]) for fr in tap.frames
                   if fr[6] > 0 and fr[0] > t_req + 1.0)
        assert d[len(d) // 2] == pytest.approx(10.0, abs=0.2)
    finally:
        _stop(link, fake)


def test_raw_truth_realigns_its_sampling_phase_and_gives_up_after_its_tries(qcore):
    """真值鏈路的相位對齊（理由見 RawTruth）：參考就是自己的取樣時刻 ⇒ 不重抽；參考永遠錯開 7 ms ⇒ 每次停掉再設回
    （SET_MESSAGE_INTERVAL −1、再 20000 µs）、試滿就放棄；串流照樣是 50 Hz（對齊模式不拉 EXTRA1 串流）。"""
    from tests.fake_mav_vehicle import FakeVehicle
    fake = FakeVehicle(ports=(0,)).start()
    try:
        raw = afp.RawTruth(f'tcp:127.0.0.1:{fake.ports[0]}', 50.0, align=lambda: [x[2] for x in raw.samples[-40:]])
        raw.start()
        assert _wait(lambda: len(raw.samples) >= 100 and raw.aligned is not None, 20.0), raw.error
        raw.stop(); raw.join(5.0)
        assert raw.aligned is True and raw.align_tries == 0 and raw.align_log[-1][2] == 'aligned', raw.align_log
        bad = afp.RawTruth(f'tcp:127.0.0.1:{fake.ports[0]}', 50.0, align=lambda: [x[2] + 7 for x in bad.samples[-40:]])
        bad.ALIGN_MAX_TRIES, bad.ALIGN_RETRY_S = 3, 0.3
        bad.start()
        assert _wait(lambda: bad.align_tries >= 3, 20.0), (bad.align_tries, bad.error)
        n0 = len(bad.samples)
        time.sleep(1.0)
        bad.stop(); bad.join(5.0)
        assert bad.aligned is False and bad.align_tries == 3 and [x[2] for x in bad.align_log] == ['retry'] * 3
        # 錯開 7 ms ⇒ 自己比參考晚 13 ms（模間隔）；假飛控在牆鐘上排程，間隔中位數偶爾是 19／21 ms ⇒ 12～14
        assert all(abs(x[1] - 13) <= 1 for x in bad.align_log), bad.align_log
        assert 40 <= len(bad.samples) - n0 <= 60, '試完之後串流照樣 50 Hz'
    finally:
        fake.stop()


def test_fake_vehicle_mission_upload_with_readback_and_the_force_arm_text_the_probe_waits_for(qcore):
    from tests.fake_mav_vehicle import FakeVehicle
    fake = FakeVehicle(ports=(0,), script='fly', ready_after_s=0.0, refuse_normal_arm=True).start()
    link, tap, texts, uploads = _link_to(fake, qcore)
    try:
        assert _wait(lambda: 1 in tap.latest and tap.latest[1].gps_fix >= 3, 15.0)
        assert afp.prearm_state(link) in (True, None)
        assert _wait(lambda: afp.prearm_state(link) is True, 5.0), 'SYS_STATUS 的 PreArm 位元讀得到'
        assert afp.boot_seconds(link) is not None
        items = afp.build_square_mission(fake.home[0], fake.home[1], 30.0, 200.0)
        link.upload_mission_items(items, verify=True)
        assert _wait(lambda: uploads, 20.0)
        ok, msg = uploads[0]
        assert ok and '讀回驗證' in msg, msg
        assert [it['command'] for it in fake.mission] == [16, 84, 16, 16, 16, 85]
        link.arm()
        assert _wait(lambda: any('FORCE ARM' in t for t in texts), 15.0), \
            '正常解鎖被拒 ⇒ SITLLink 走 FORCE，狀態文字含「FORCE ARM」（探針靠這句中止）'
        assert any('PreArm 擋住解鎖' in t for t in texts)
        # SITLLink 先發「FORCE ARM」狀態文字、再送 force 解鎖指令 ⇒ 假飛控收到要再等一下（以前直接 assert，約 1/15 次失敗）
        assert _wait(lambda: any('FORCE' in e for _t, e in list(fake.events)), 5.0)
    finally:
        _stop(link, fake)


def _att_rows(tap):
    return [fr for fr in tap.frames if fr[6] > 0 and fr[16] is not None]


def test_fake_vehicle_sine_and_phase_offset_through_the_real_sitllink_carry_the_boot_clock(qcore):
    """一直在彎＋姿態串流比位置晚 23 ms：真 SITLLink 收到的每一則 ATTITUDE 都記到它自己的 time_boot_ms（值逐一對得上
    送出去的那一則）；本機 TCP 的抖動很小、沒有成批；假飛控兩條串流真的錯相。"""
    from tests.fake_mav_vehicle import FakeVehicle, sine_attitude
    fake = FakeVehicle(ports=(0,), script='sine', att_phase_s=0.023).start()
    link, tap, _texts, _ = _link_to(fake, qcore)
    try:
        # 50 Hz：≥ 300 則 ≈ 6 s（boot_clock_series 要扣掉前 2.5 s）
        assert _wait(lambda: len(_att_rows(tap)) >= 300, 25.0), '沒有收到帶開機時間的姿態'
    finally:
        _stop(link, fake)
    att = _att_rows(tap)
    n_att = len({fr[6] for fr in tap.frames if fr[6] > 0})
    assert len(att) >= 0.95 * n_att, (len(att), n_att)
    sent = {b: (r, p, y) for idx, _t, b, r, p, y in list(fake.att_log) if idx == 0}
    for fr in att:
        r, p, y = sent[fr[16]]
        assert max(abs(r - fr[7]), abs(p - fr[8]), abs(afp.wrap180(y - fr[9]))) < 1e-4, '開機時間配對到送出去的那一則'
    for fr in att:                                     # 真的是「一直在彎」的劇本，而且姿態就取在 time_boot_ms 那一刻
        r, p, y = sine_attitude(fr[16] / 1000.0)
        assert max(abs(r - fr[7]), abs(p - fr[8]), abs(afp.wrap180(y - fr[9]))) < 1e-4, (fr[16], fr[7:10], (r, p, y))
    assert max(abs(fr[7]) for fr in att) > 40.0
    _ts, st = afp.boot_clock_series([(fr[6], fr[16] / 1000.0, fr[7], fr[8], fr[9]) for fr in att], skip_first_s=2.5)
    assert st['bursts'] == 0 and st['latency_above_min_s']['p95'] < 0.01, st
    assert st['segments'][0]['b'] == pytest.approx(1.0, abs=2e-3)
    pos_t = [t for idx, t, _b in fake.pos_log if idx == 0]
    lag = sorted(t - pos_t[bisect.bisect_right(pos_t, t) - 1] for idx, t, *_ in fake.att_log
                 if idx == 0 and t > pos_t[0] + 2.0)
    assert lag[len(lag) // 2] == pytest.approx(0.023, abs=0.006), '姿態比位置晚 att_phase_s 送'


def test_fake_vehicle_bursts_show_up_on_the_boot_clock_through_the_real_sitllink(qcore):
    from tests.fake_mav_vehicle import FakeVehicle
    fake = FakeVehicle(ports=(0,), script='sine', burst_every_s=1.0, burst_hold_s=0.25, burst_start_s=2.0).start()
    link, tap, _texts, _ = _link_to(fake, qcore)
    try:
        assert _wait(lambda: fake.bursts() >= 5, 25.0)
        time.sleep(0.6)
    finally:
        _stop(link, fake)
    att = _att_rows(tap)
    _ts, st = afp.boot_clock_series([(fr[6], fr[16] / 1000.0, fr[7], fr[8], fr[9]) for fr in att], skip_first_s=2.5)
    assert st['bursts'] >= 2 and st['compressed_gaps'] >= 8, st
    assert st['latency_above_min_s']['max'] == pytest.approx(0.25, abs=0.06), st['latency_above_min_s']


@pytest.mark.parametrize('vehicle, hint, want', [('copter', 'COPTER', ['COPTER']), ('plane', 'PLANE', ['PLANE']),
                                                 ('quadplane', '', ['PLANE', 'VTOL'])],
                         ids=['copter', 'plane', 'quadplane-without-hint'])
def test_fake_vehicle_kinds_through_the_real_sitllink(qcore, vehicle, hint, want):
    """多旋翼／固定翼各自的心跳與模式表；QuadPlane 沒有機型提示時，SITLLink 第一筆帶 PLANE（第一個心跳預先建的 frame），
    Q_ENABLE 讀回後的下一個心跳才變 VTOL ⇒ 3D 頁面會在飛行中換模型（探針 (f) 就是量這個）。"""
    from tests.fake_mav_vehicle import FakeVehicle
    fake = FakeVehicle(ports=(0,), script='sine', vehicle=vehicle, hb_period_s=2.0).start()
    link, tap, texts, _ = _link_to(fake, qcore, hint=hint)
    try:
        assert _wait(lambda: 1 in tap.latest and tap.latest[1].vehicle_type == want[-1] and tap.latest[1].mode == 'AUTO'
                     and sum(1 for fr in tap.frames if fr[13] >= 3) > 20, 20.0), [fr[5] for fr in tap.frames][-5:]
    finally:
        _stop(link, fake)
    seq = []
    for fr in tap.frames:
        if fr[13] >= 3 and (not seq or seq[-1] != fr[5]):
            seq.append(fr[5])
    assert seq == want, seq
    assert tap.latest[1].mode == 'AUTO'
    if vehicle == 'quadplane':
        assert any('Q_ENABLE=1' in t for t in texts)


# ═══════════════════════════════════════════════════════════════════════
#  C. 真的 QtWebEngine 視窗（一次一個；SAFE_ALLOW_BROWSER=1 才跑）
# ═══════════════════════════════════════════════════════════════════════
needs_window = pytest.mark.skipif(
    os.environ.get('SAFE_ALLOW_BROWSER') != '1' or sys.platform != 'win32'
    or not (ROOT / 'assets' / 'cesium' / 'Build' / 'Cesium' / 'Cesium.js').is_file(),
    reason='要開真的 QtWebEngine 視窗（WebGL）：SAFE_ALLOW_BROWSER=1、Windows、本機 assets/cesium')


@pytest.fixture(scope='module')
def glb_cache(tmp_path_factory):
    """GLB 合併快取放 tmp（app 是 data/cache/models/；測試不寫 repo）。repo 裡已經有現成的合併檔就複製過來（只讀 repo），
    鍵對不上（原檔換了）探針會自己在這裡重做。"""
    import shutil
    d = tmp_path_factory.mktemp('glb_cache')
    for f in (ROOT / 'data' / 'cache' / 'models').glob('*.merged-*.glb'):
        shutil.copy2(f, d / f.name)
    return d


def _run_probe(tmp_path, fake_kw, args, timeout=240, cache=None):
    from tests.fake_mav_vehicle import FakeVehicle
    tmp_path.mkdir(parents=True, exist_ok=True)
    fake = FakeVehicle(ports=(0, 0), **fake_kw).start()
    out = tmp_path / 'out'
    # AEROPLAN_GLB_MERGE=1：tests/conftest.py 設成 0（建 widget 的測試別寫 repo 快取）會漏進子行程 ⇒ 畫原檔、
    # 不是 app 畫的合併檔（2026-10-22 審查）；快取指到 tmp
    env = dict(os.environ, QT_QPA_PLATFORM='offscreen', AEROPLAN_GLB_MERGE='1', **_ENV)   # 探針自己會改成 windows（WebGL）
    cmd = [sys.executable, str(ROOT / 'tools' / 'attitude_fidelity_probe.py'),
           '--conn', f'tcp:127.0.0.1:{fake.ports[0]}', '--truth', f'tcp:127.0.0.1:{fake.ports[1]}',
           '--out', str(out), *(['--model-cache', str(cache)] if cache else []), *args]
    try:
        r = subprocess.run(cmd, env=env, cwd=str(tmp_path), capture_output=True, text=True, encoding='utf-8',
                           errors='replace', timeout=timeout)
    finally:
        fake.stop()
    s = json.loads((out / 'summary.json').read_text(encoding='utf-8')) if (out / 'summary.json').exists() else None
    return r, s, fake, out


def _failed_checks(s):
    return sorted(c['name'] for c in s['gate']['checks'] if not c['ok'])


def _uri_path(u):
    from urllib.parse import unquote, urlparse
    return pathlib.Path(unquote(urlparse(u).path).lstrip('/'))


def _all(s):
    return s['errors_deg']['all']


@needs_window
def test_a_nose_follow_off_rendered_attitude_equals_the_sitl_attitude(tmp_path, glb_cache):
    r, s, fake, out = _run_probe(tmp_path, {}, ['--seconds', '17', '--nose-follows-track', 'off', '--shot-every', '100',
                                                '--fail-over', '0.5'], cache=glb_cache)
    assert r.returncode == 0 and s is not None, r.stdout[-3000:] + r.stderr[-2000:]
    assert s['gate']['enforced'] and s['gate']['ok'] and _failed_checks(s) == [], _failed_checks(s)
    a, nh = _all(s), s['errors_deg']['page_not_holding']
    assert a['n'] > 150, a
    for ax in ('yaw', 'pitch', 'roll', 'total'):
        assert a[ax]['p95'] < 0.5, (ax, a[ax])
        assert nh[ax]['max'] < 1.0, ('頁面沒在 HOLD 時只剩畫法＋插值', ax, nh[ax])
    # 頁面 HOLD 的列（主執行緒卡住、批次晚到）：誤差要能用「角速度 × 落後」解釋（劇本角速度 ≤ ~61°/s），而且少見
    import csv
    rows = [x for x in csv.DictReader(open(out / 'samples.csv', encoding='utf-8')) if x['status'] == 'ok']
    for x in rows:
        age = float(x['page_att_age_s'])
        if age > 0:
            assert float(x['err_total']) <= 65.0 * age + 0.5, x
    assert s['page_hold_rows'] <= 0.1 * a['n'], s['page_hold_rows']
    assert s['nose_follows_track_observed'] == ['False'] and s['models'] == ['vtol']
    assert s['page']['offsets']['vtol'] == [-90, 90, 180]
    assert s['errors_vs_raw_truth_deg']['n'] > 150 and s['errors_vs_raw_truth_deg']['total']['p95'] < 0.5
    assert s['errors_drawn_frame_deg']['n'] > 50 and s['errors_drawn_frame_deg']['total']['p95'] < 0.5, \
        '真的畫進那一幀的模型矩陣也對'
    assert 0.08 <= s['display_delay_s']['p50'] <= 0.3
    # 劇本真的有 ±45° 滾轉、±20° 俯仰、跨北的偏航 —— 不是在量一架水平飛的飛機
    tr = [float(x['truth_roll']) for x in rows]
    tp = [float(x['truth_pitch']) for x in rows]
    ty = [float(x['truth_yaw']) for x in rows]
    assert max(tr) > 40 and min(tr) < -40 and max(tp) > 15 and min(tp) < -15
    assert any(y > 345 for y in ty) and any(y < 15 for y in ty)
    assert list((out / 'shots').glob('*.png')), '截圖'
    assert s['plot'] == 'attitude_timeseries.png' and (out / 'attitude_timeseries.png').is_file()
    lag = s['errors_vs_wallclock_now_deg']['roll']['max']
    assert 4.0 < lag < 12.0, f'牆鐘落後：60°/s 滾轉 × 顯示延遲 ~0.13 s ≈ 8°（{lag}）'
    # 2026-10-22 審查後：每一個算圖幀、開機時間軸、獨立真值的開機時間軸、SITLLink 換算都量了，而且都對
    f = s['errors_per_frame_deg']['all']
    assert f['n'] > 1.5 * a['n'] and f['total']['p95'] < 0.5, f
    for k in ('errors_vs_boot_clock_deg', 'errors_vs_raw_boot_clock_deg'):
        assert s[k]['n'] > 300 and s[k]['total']['p95'] < 0.5, (k, s[k])
    # 同一個飛控時刻：SITLLink 換算 = 獨立鏈路。剩下的只是獨立鏈路 50 Hz 的內插誤差：這個劇本是分段線性、轉角處角速度
    # 一下跳 60～120°/s ⇒ 一個 20 ms 間隔裡最多 ~0.3°（換算錯 —— 正負號、軸、單位 —— 是幾十度）
    assert s['raw_truth']['link_vs_raw_boot_deg']['p95'] < 0.25, s['raw_truth']['link_vs_raw_boot_deg']
    assert s['boot_clock']['link']['1']['bursts'] == 0 and s['boot_clock']['link']['1']['latency_above_min_s']['p95'] < 0.01
    assert s['samples_in']['truth_link_with_boot_ms'] > 250
    assert sum(1 for c in s['phase_histogram']['reads'] if c) >= 7, s['phase_histogram']
    assert s['calibration']['mismatch'] == []
    # 跟 app 一樣畫合併過的 GLB（快取在 tmp）
    vt = s['page']['uris']['vtol']
    assert '.merged-' in vt and os.path.samefile(_uri_path(vt).parent, glb_cache), vt


@needs_window
def test_b_nose_follow_on_with_a_10_degree_crab_shows_up_as_exactly_that_yaw_error(tmp_path, glb_cache):
    r, s, fake, out = _run_probe(tmp_path, {'crab_deg': 10.0}, ['--seconds', '12', '--nose-follows-track', 'on'],
                                 cache=glb_cache)
    assert r.returncode == 0 and s is not None, r.stdout[-3000:] + r.stderr[-2000:]
    a = _all(s)
    assert a['yaw']['bias'] == pytest.approx(10.0, abs=0.3) and a['yaw']['p50'] == pytest.approx(10.0, abs=0.3), a['yaw']
    assert a['roll']['p95'] < 0.5 and a['pitch']['p95'] < 0.5, '機鼻換成航跡只動偏航'
    assert s['nose_follows_track_observed'] == ['True']
    assert s['crs_minus_yaw_deg']['p50'] == pytest.approx(10.0, abs=0.1)


@needs_window
def test_c_the_old_angle_addition_formula_is_caught_as_a_large_roll_error(tmp_path, glb_cache):
    r, s, fake, out = _run_probe(tmp_path, {}, ['--seconds', '12', '--nose-follows-track', 'off',
                                                '--mutate', 'old-angle-addition'], cache=glb_cache)
    assert r.returncode == 0 and s is not None, r.stdout[-3000:] + r.stderr[-2000:]
    a = _all(s)
    assert a['roll']['p95'] > 20.0 and a['total']['p95'] > 20.0, a
    assert a['yaw']['p95'] > 20.0, '舊寫法在 VTOL 校正下：滾轉畫成偏航'
    r2, s2, *_ = _run_probe(tmp_path / 'fail', {}, ['--seconds', '8', '--nose-follows-track', 'off',
                                                    '--mutate', 'old-angle-addition', '--fail-over', '1.0'],
                            cache=glb_cache)
    assert r2.returncode == 4 and s2['exit_code'] == 4, '--fail-over：p95 超過門檻 ⇒ 結束碼 4'
    bad = _failed_checks(s2)
    assert {'errors_deg.all.p95', 'errors_per_frame_deg.all.p95', 'errors_vs_boot_clock_deg.p95',
            'errors_drawn_frame_deg.p95'} <= set(bad), bad
    assert not any(n.startswith(('link_vs_raw', 'coverage', 'calibration', 'models')) for n in bad),         '錯在頁面的畫法，不是 SITLLink 換算、覆蓋率或校正常數：閘門要指得出是哪一項'


@needs_window
def test_fly_mode_uploads_arms_without_force_flies_the_square_and_stops_at_landing(tmp_path, glb_cache):
    r, s, fake, out = _run_probe(tmp_path, {'script': 'fly', 'speed_ms': 25.0, 'climb_ms': 6.0, 'sink_ms': 6.0,
                                            'ready_after_s': 2.0},
                                 ['--fly', '--fly-side', '150', '--fly-alt', '20', '--seconds', '150'], timeout=300,
                                 cache=glb_cache)
    assert r.returncode == 0 and s is not None, r.stdout[-3000:] + r.stderr[-2000:]
    assert s['end_reason'] == '落地上鎖' and s['fly']['final_state'] == 'done'
    states = [x[1] for x in s['fly']['log']]
    assert states[:3] == ['wait_ready', 'upload', 'qmode'] and states[-3:] == ['auto', 'flying', 'done'], states
    ev = [e for _t, e in fake.events]
    assert '解鎖' in ev and not any('FORCE' in e for e in ev), '真的 PreArm 解鎖，不是 force'
    assert sum(e.startswith('到達航點') for e in ev) == 3 and '觸地、自動上鎖' in ev
    assert 'AUTO' in s['errors_deg']['by_mode'] and s['errors_deg']['by_mode']['AUTO']['total']['p95'] < 0.5
    assert 'ge6' in s['errors_deg']['by_speed_band'] and 'lt3' in s['errors_deg']['by_speed_band']


@needs_window
def test_fly_mode_aborts_and_disarms_instead_of_flying_when_sitllink_falls_back_to_force_arm(tmp_path, glb_cache):
    r, s, fake, out = _run_probe(tmp_path, {'script': 'fly', 'ready_after_s': 0.0, 'refuse_normal_arm': True},
                                 ['--fly', '--fly-side', '150', '--fly-alt', '20', '--seconds', '60'], cache=glb_cache)
    assert r.returncode == 3 and s is not None, r.stdout[-3000:] + r.stderr[-2000:]
    assert s['fly']['final_state'] == 'abort' and 'FORCE' in s['fly']['log'][-1][2]
    ev = [e for _t, e in fake.events]
    assert '解鎖（FORCE）' in ev and '上鎖' in ev[ev.index('解鎖（FORCE）'):], '被 force 解鎖後立刻上鎖'
    assert not any('AUTO' in e for e in ev), '沒有切 AUTO、沒有起飛'


# ── 2026-10-22 審查後：讓假飛控變難（一直在彎、錯相、成批、換機型、固定翼／多旋翼）────────────────────────
@needs_window
def test_d_curving_attitude_with_phase_offset_streams_passes_the_full_gate(tmp_path, glb_cache):
    r, s, fake, out = _run_probe(tmp_path, {'script': 'sine', 'att_phase_s': 0.023},
                                 ['--seconds', '12', '--fail-over', '0.5'], cache=glb_cache)
    assert r.returncode == 0 and s is not None, r.stdout[-3000:] + r.stderr[-2000:]
    assert s['gate']['ok'], _failed_checks(s)
    f = s['errors_per_frame_deg']['all']
    assert f['n'] > 300 and f['total']['p95'] < 0.5
    hr = s['phase_histogram']['reads']
    assert sum(1 for c in hr if c) >= 7 and max(hr) <= 0.35 * sum(hr), f'讀回不和樣本鎖相：{hr}'
    import csv
    rows = [x for x in csv.DictReader(open(out / 'samples.csv', encoding='utf-8'))
            if x['status'] == 'ok' and x['row_kind'] == 'frame']
    tr = [float(x['truth_roll']) for x in rows]
    assert max(tr) > 40 and min(tr) < -40, '真的在彎'


@needs_window
def test_d2_drawing_the_previous_sample_without_interpolation_is_caught_on_every_frame(tmp_path, glb_cache):
    """突變：模型姿態不插值（顯示時間落在兩筆之間就畫前一筆）。以前讀回和樣本鎖相 ⇒ 相位剛好在樣本上就量不到。"""
    r, s, fake, out = _run_probe(tmp_path, {'script': 'sine', 'att_phase_s': 0.023},
                                 ['--seconds', '8', '--mutate', 'hold-previous', '--fail-over', '0.5'], cache=glb_cache)
    assert r.returncode == 4 and s is not None, r.stdout[-3000:] + r.stderr[-2000:]
    assert s['errors_per_frame_deg']['all']['total']['p95'] > 2.0, s['errors_per_frame_deg']['all']
    bad = _failed_checks(s)
    assert {'errors_per_frame_deg.all.p95', 'errors_vs_boot_clock_deg.p95', 'errors_deg.all.p95'} <= set(bad), bad
    assert not any(n.startswith(('link_vs_raw', 'coverage', 'calibration', 'models')) for n in bad), bad


@needs_window
def test_e_a_bursty_link_is_seen_on_the_boot_clock_and_fails_the_gate(tmp_path, glb_cache):
    """主鏈路每秒被扣住 0.25 s 再一口氣送（獨立真值鏈路不受影響）：開機時間軸看得到成批、延遲，閘門擋下。"""
    r, s, fake, out = _run_probe(tmp_path, {'script': 'sine', 'burst_every_s': 1.0, 'burst_hold_s': 0.25},
                                 ['--seconds', '10', '--fail-over', '0.5'], cache=glb_cache)
    assert r.returncode == 4 and s is not None, r.stdout[-3000:] + r.stderr[-2000:]
    bl, br = s['boot_clock']['link']['1'], s['boot_clock']['raw']['1']
    assert bl['bursts'] >= 3 and bl['latency_above_min_s']['max'] == pytest.approx(0.25, abs=0.08), bl
    assert br['bursts'] == 0 and br['latency_above_min_s']['p95'] < 0.01, '獨立真值鏈路沒有被扣住'
    assert s['errors_vs_boot_clock_deg']['total']['p95'] > 2.0
    assert 'errors_vs_boot_clock_deg.p95' in _failed_checks(s)
    assert s['raw_truth']['link_vs_raw_boot_deg']['p95'] < 0.05, '換算本身沒錯：錯的是時間'


@needs_window
def test_f_vehicle_type_change_in_flight_rebuilds_the_model_with_its_own_calibration(tmp_path, glb_cache):
    """沒有機型提示的 QuadPlane：SITLLink 先送 PLANE（Barracuda GLB、校正 0/0/0），下一個心跳變 VTOL（vtol GLB、
    −90/90/180）⇒ 頁面在飛行中換模型；兩段各自用自己的校正都要對（探針用寫死的校正、依實際載入的 URI 選）。"""
    r, s, fake, out = _run_probe(tmp_path, {'script': 'sine', 'hb_period_s': 4.0},
                                 ['--seconds', '10', '--vehicle', '', '--warmup', '1.0', '--fail-over', '0.5'],
                                 cache=glb_cache)
    assert r.returncode == 0 and s is not None, r.stdout[-3000:] + r.stderr[-2000:]
    assert s['gate']['ok'], _failed_checks(s)
    assert s['models'] == ['plane', 'vtol'], s['models']
    import csv
    seq = []
    for x in csv.DictReader(open(out / 'samples.csv', encoding='utf-8')):
        if x['row_kind'] == 'frame' and (not seq or seq[-1] != x['model']):
            seq.append(x['model'])
    assert seq == ['plane', 'vtol'], seq
    bm = s['errors_per_frame_deg']['by_model']
    assert bm['plane']['n'] >= 20 and bm['plane']['total']['p95'] < 0.5, bm['plane']
    assert bm['vtol']['n'] >= 200 and bm['vtol']['total']['p95'] < 0.5, bm['vtol']


@needs_window
@pytest.mark.parametrize('vehicle, hint, kind', [('plane', 'PLANE', 'plane'), ('copter', 'COPTER', 'copter')])
def test_g_fixed_wing_and_multirotor_models_follow_the_sitl_attitude(tmp_path, glb_cache, vehicle, hint, kind):
    r, s, fake, out = _run_probe(tmp_path, {'script': 'sine', 'vehicle': vehicle, 'speed_ms': 12.0},
                                 ['--seconds', '8', '--vehicle', hint, '--fail-over', '0.5'], cache=glb_cache)
    assert r.returncode == 0 and s is not None, r.stdout[-3000:] + r.stderr[-2000:]
    assert s['gate']['ok'], _failed_checks(s)
    assert s['models'] == [kind] and s['calibration']['mismatch'] == []
    assert s['errors_per_frame_deg']['all']['total']['p95'] < 0.5 and s['errors_drawn_frame_deg']['n'] > 100
