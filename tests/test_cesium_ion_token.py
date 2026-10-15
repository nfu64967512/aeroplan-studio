"""Cesium ion access token 的接線（2026-10-09 使用者提供 token 要放進 3D 地圖）。

設計：token 是祕密 ⇒ 不進原始碼，放被 .gitignore 的 ``config/settings.json`` 的 ``map.cesium_ion_token``，
環境變數 ``CESIUM_ION_TOKEN`` 優先。token 會被 ``.replace('__CESIUM_TOKEN__', …)`` 塞進 JS 字串字面量，
所以只放行 JWT 字元集（含引號／空白／換行的值會讓整份 3D 頁的 script 語法錯誤、3D 地圖靜默變黑）。

這裡一律用**假 token**，測試檔裡不放任何真 token。
"""
from __future__ import annotations

import dataclasses
import re
import types

import pytest

from config.settings import MapSettings
from ui.widgets import cesium_map_widget as cmw
from ui.widgets.cesium_map_widget import CesiumMapWidget, resolve_cesium_ion_token

TOK_A = "aaaa.bbbb.cccc"          # 類別屬性
TOK_E = "eeee.ffff.gggg"          # 環境變數
TOK_S = "ssss.tttt.uuuu"          # settings.json


# ── 純函式：優先序、清理、拒絕壞值 ───────────────────────────────────────────
def test_precedence_class_then_env_then_settings():
    assert resolve_cesium_ion_token(TOK_A, TOK_E, TOK_S)[0] == TOK_A
    assert resolve_cesium_ion_token('', TOK_E, TOK_S)[0] == TOK_E
    assert resolve_cesium_ion_token('', None, TOK_S)[0] == TOK_S
    assert resolve_cesium_ion_token('', '', '') == ('', '')


def test_source_names_say_where_the_value_came_from():
    assert 'CESIUM_TOKEN' in resolve_cesium_ion_token(TOK_A, TOK_E, TOK_S)[1]
    assert 'CESIUM_ION_TOKEN' in resolve_cesium_ion_token('', TOK_E, TOK_S)[1]
    assert 'settings.json' in resolve_cesium_ion_token('', None, TOK_S)[1]


def test_surrounding_whitespace_and_newlines_are_stripped():
    """貼上 token 時常帶換行／空白。"""
    assert resolve_cesium_ion_token('', None, f"  {TOK_S}\r\n")[0] == TOK_S
    assert resolve_cesium_ion_token('   ', None, TOK_S)[0] == TOK_S      # 全空白 = 沒設


@pytest.mark.parametrize("bad", [
    "abc.def.ghi'; alert(1); //",        # 跳出字串
    'abc.def."ghi',                      # 雙引號
    "abc def.ghi.jkl",                   # 中間有空白
    "abc.def.ghi\nxyz",                  # 中間換行
    "abc.def.ghi</script>",              # 提早結束 script
    "abc.def.ghi\\",                     # 反斜線
    "token with spaces",
    "權杖.abc.def",                       # 非 ASCII
])
def test_unsafe_values_are_rejected(bad):
    tok, src = resolve_cesium_ion_token(bad, None, '')
    assert tok == '' and src.startswith('invalid:'), (tok, src)


def test_an_invalid_value_does_not_fall_back_to_a_lower_priority_source():
    """壞值不可靜默換成別的來源的值生效 ⇒ 回 invalid，由呼叫端記 log。"""
    tok, src = resolve_cesium_ion_token("bad token", TOK_E, TOK_S)
    assert tok == '' and src.startswith('invalid:')
    tok, src = resolve_cesium_ion_token('', "bad'token", TOK_S)
    assert tok == '' and 'CESIUM_ION_TOKEN' in src


def test_a_real_looking_jwt_is_accepted():
    jwt = "eyJhbGciOiJIUzI1NiJ9.eyJzdWIiOiIxIn0.abc_DEF-123"
    assert resolve_cesium_ion_token('', None, jwt) == (jwt, 'settings.json map.cesium_ion_token')


# ── 設定檔欄位 ───────────────────────────────────────────────────────────────
def test_map_settings_has_the_field_and_defaults_to_empty():
    assert MapSettings().cesium_ion_token == ''
    assert 'cesium_ion_token' in dataclasses.asdict(MapSettings())


def test_old_settings_files_without_the_key_still_load():
    """既有的 settings.json 沒有這個鍵：MapSettings(**dict) 要照樣載得進來（預設空）。"""
    old = {k: v for k, v in dataclasses.asdict(MapSettings()).items() if k != 'cesium_ion_token'}
    assert MapSettings(**old).cesium_ion_token == ''
    assert MapSettings(**{**old, 'cesium_ion_token': TOK_S}).cesium_ion_token == TOK_S


# ── 元件：_resolve_token / _load_cesium（假 self，不建 QtWebEngine）────────────
class _Capture:
    def __init__(self):
        self.lines = []

    def info(self, m, *a, **k):
        self.lines.append(str(m))

    warning = error = debug = info


@pytest.fixture
def fake(monkeypatch, tmp_path):
    """一個夠 `_load_cesium` 跑完的假 self + 假設定 + 假 logger；輸出頁寫到 tmp。"""
    cap = _Capture()
    monkeypatch.setattr(cmw, 'logger', cap)
    monkeypatch.delenv('CESIUM_ION_TOKEN', raising=False)
    holder = types.SimpleNamespace(token='')
    settings = types.SimpleNamespace(map=types.SimpleNamespace(
        default_lat=25.0, default_lon=121.0,
        cesium_ion_token=''))
    monkeypatch.setattr('config.settings.get_settings', lambda: settings)

    out = tmp_path / 'cesium_test.html'
    import utils.temp_cache as tc
    monkeypatch.setattr(tc, 'new_temp_html_path', lambda **k: out)

    shown = {}
    self_ = types.SimpleNamespace(
        CESIUM_TOKEN='', VTOL_MODEL_SCALE=1.0, VTOL_HEADING_OFFSET=0, VTOL_PITCH_OFFSET=0,
        VTOL_ROLL_OFFSET=0, web_view=types.SimpleNamespace(setUrl=lambda u: shown.setdefault('url', u)))
    self_._resolve_token = types.MethodType(CesiumMapWidget._resolve_token, self_)
    return types.SimpleNamespace(self=self_, settings=settings, log=cap, out=out, shown=shown, holder=holder)


def _page(fake_):
    CesiumMapWidget._load_cesium(fake_.self)
    return fake_.out.read_text(encoding='utf-8')


def _token_line(html):
    m = re.search(r"^Cesium\.Ion\.defaultAccessToken = '([^']*)';$", html, flags=re.M)
    assert m, 'ion token 那一行不見了或語法不對'
    return m.group(1)


def test_settings_token_ends_up_in_the_generated_page(fake):
    fake.settings.map.cesium_ion_token = TOK_S
    assert _token_line(_page(fake)) == TOK_S


def test_env_beats_settings_and_class_beats_env(fake, monkeypatch):
    fake.settings.map.cesium_ion_token = TOK_S
    monkeypatch.setenv('CESIUM_ION_TOKEN', TOK_E)
    assert _token_line(_page(fake)) == TOK_E
    fake.self.CESIUM_TOKEN = TOK_A
    assert _token_line(_page(fake)) == TOK_A


def test_no_token_means_an_empty_literal_and_a_valid_script(fake):
    html = _page(fake)
    assert _token_line(html) == ''
    assert '__CESIUM_TOKEN__' not in html


def test_unsafe_token_never_reaches_the_page(fake):
    fake.settings.map.cesium_ion_token = "x'; alert(1); //"
    html = _page(fake)
    assert _token_line(html) == ''
    assert 'alert(1)' not in html
    assert any('格式不合' in m for m in fake.log.lines), '壞 token 要在 log 說出來，不能靜默吞掉'


def test_the_token_is_never_written_to_the_log(fake):
    fake.settings.map.cesium_ion_token = TOK_S
    _page(fake)
    assert fake.log.lines, '前提：有記 log'
    joined = '\n'.join(fake.log.lines)
    assert TOK_S not in joined and 'ssss' not in joined
    assert f'長度 {len(TOK_S)}' in joined and 'settings.json' in joined


def test_page_is_still_written_when_settings_cannot_be_read(fake, monkeypatch):
    """設定讀不到（例外）⇒ 照樣出頁、沒有 token，不能讓 3D 地圖起不來。"""
    def _boom():
        raise RuntimeError('settings broken')
    monkeypatch.setattr('config.settings.get_settings', _boom)
    html = _page(fake)
    assert _token_line(html) == ''


# ── utils.cesium_ion.configured_ion_token（規劃核心與面板用）──────────────────
from utils.cesium_ion import configured_ion_token  # noqa: E402


def test_configured_token_reads_the_environment_then_settings(monkeypatch):
    settings = types.SimpleNamespace(map=types.SimpleNamespace(cesium_ion_token=TOK_S))
    monkeypatch.setattr('config.settings.get_settings', lambda: settings)
    monkeypatch.delenv('CESIUM_ION_TOKEN', raising=False)
    assert configured_ion_token() == (TOK_S, 'settings.json map.cesium_ion_token')
    monkeypatch.setenv('CESIUM_ION_TOKEN', TOK_E)
    assert configured_ion_token()[0] == TOK_E
    assert configured_ion_token(TOK_A)[0] == TOK_A


def test_configured_token_survives_broken_settings(monkeypatch):
    def _boom():
        raise RuntimeError('settings broken')
    monkeypatch.setattr('config.settings.get_settings', _boom)
    monkeypatch.delenv('CESIUM_ION_TOKEN', raising=False)
    assert configured_ion_token() == ('', '')


def test_configured_token_rejects_an_unsafe_value(monkeypatch):
    settings = types.SimpleNamespace(map=types.SimpleNamespace(cesium_ion_token="x'; alert(1); //"))
    monkeypatch.setattr('config.settings.get_settings', lambda: settings)
    monkeypatch.delenv('CESIUM_ION_TOKEN', raising=False)
    tok, src = configured_ion_token()
    assert tok == '' and src.startswith('invalid:')
