# -*- coding: utf-8 -*-
"""tests/test_stream_hz.py — 網路鏈路把位置／姿態／VFR 串流拉高到 STREAM_HZ（2026-10-17：畫面要流暢、SITL Hz 要高）。"""
from __future__ import annotations

import ast
import os
import pathlib

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('AEROPLAN_NO_FILE_LOG', '1')

import mission.sitl_link as sl  # noqa: E402
from mission.sitl_link import SITLLink  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[1]


def _link(conn):
    lk = SITLLink.__new__(SITLLink)
    lk.conn_str = conn
    return lk


@pytest.mark.parametrize('conn, serial', [
    ('tcp:127.0.0.1:5760', False), ('udpin:0.0.0.0:14550', False), ('udp:127.0.0.1:14550', False),
    ('COM7,57600', True), ('com36,115200', True), ('/dev/ttyACM0', True), ('serial:/dev/ttyUSB0:57600', True),
    ('command_center', False),
])
def test_serial_detection(conn, serial):
    assert _link(conn).is_serial_link() is serial


def test_network_links_get_stream_hz_and_serial_stays_at_default(monkeypatch):
    monkeypatch.setattr(sl, 'STREAM_HZ', 20.0)
    assert _link('tcp:127.0.0.1:5760').telemetry_stream_hz() == 20.0
    assert _link('COM7,57600').telemetry_stream_hz() == 0.0, '數傳 57600 塞不下，不拉高'
    monkeypatch.setenv('AEROPLAN_STREAM_HZ_SERIAL', '8')
    assert _link('COM7,57600').telemetry_stream_hz() == 8.0
    monkeypatch.setattr(sl, 'STREAM_HZ', 0.0)
    assert _link('tcp:127.0.0.1:5760').telemetry_stream_hz() == 0.0


def test_default_is_20_hz_and_env_overrides():
    assert sl._env_float('AEROPLAN_STREAM_HZ_NOPE', 20.0) == 20.0
    src = (ROOT / 'mission' / 'sitl_link.py').read_text(encoding='utf-8', errors='replace')
    assert "_env_float('AEROPLAN_STREAM_HZ', 20.0)" in src


def test_run_link_requests_position_attitude_and_vfr_streams_at_stream_hz():
    """AST 鎖：_run_link 裡對 (6, 10, 11) 三條串流各送一次 request_data_stream_send，頻率來自 telemetry_stream_hz()。"""
    src = (ROOT / 'mission' / 'sitl_link.py').read_text(encoding='utf-8', errors='replace')
    tree = ast.parse(src)
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_run_link')
    body = ast.unparse(fn)
    assert 'for stream_id in (6, 10, 11)' in body
    assert 'self.telemetry_stream_hz()' in body
    assert 'request_data_stream_send(self._mav.target_system, self._mav.target_component, stream_id, int(hz), 1)' in body


# ── ATTITUDE 另外拉到 50 Hz（2026-10-22：3D 姿態忠實度；懸停 ~17 Hz 振盪 20 Hz 畫不出來）──────────
def test_attitude_gets_its_own_rate_on_network_links_and_serial_keeps_the_stream_rate(monkeypatch):
    monkeypatch.setattr(sl, 'ATTITUDE_HZ', 50.0)
    assert _link('tcp:127.0.0.1:5760').attitude_stream_hz() == 50.0
    assert _link('udpin:0.0.0.0:14550').attitude_stream_hz() == 50.0
    assert _link('COM7,57600').attitude_stream_hz() == 0.0, '數傳不另外拉高'
    monkeypatch.setenv('AEROPLAN_ATTITUDE_HZ_SERIAL', '10')
    assert _link('COM7,57600').attitude_stream_hz() == 10.0
    monkeypatch.setattr(sl, 'ATTITUDE_HZ', -3.0)
    assert _link('tcp:127.0.0.1:5760').attitude_stream_hz() == 0.0
    src = (ROOT / 'mission' / 'sitl_link.py').read_text(encoding='utf-8', errors='replace')
    assert "ATTITUDE_HZ: float = _env_float('AEROPLAN_ATTITUDE_HZ', 50.0)" in src
    assert sl._MAVLINK_MSG_ID_ATTITUDE == 30 and sl._MAV_CMD_SET_MESSAGE_INTERVAL == 511


def test_run_link_sets_the_attitude_interval_after_the_stream_requests():
    """AST：SET_MESSAGE_INTERVAL(ATTITUDE, 1e6/att_hz) 在 (6, 10, 11) 串流要求**之後**（ArduPilot 收到 EXTRA1 的串流要求會把
    ATTITUDE 改回串流頻率），而且只在比串流頻率高時才送。"""
    src = (ROOT / 'mission' / 'sitl_link.py').read_text(encoding='utf-8', errors='replace')
    tree = ast.parse(src)
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_run_link')
    body = ast.unparse(fn)
    i_streams = body.index('for stream_id in (6, 10, 11)')
    i_att = body.index('att_hz = self.attitude_stream_hz()')
    assert i_streams < i_att
    tail = body[i_att:]
    assert 'if att_hz > max(hz, 0.0):' in tail
    assert ('command_long_send(self._mav.target_system, self._mav.target_component, _MAV_CMD_SET_MESSAGE_INTERVAL, 0, '
            '_MAVLINK_MSG_ID_ATTITUDE, 1000000.0 / att_hz, 0, 0, 0, 0, 0)') in tail
