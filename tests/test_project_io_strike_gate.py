# -*- coding: utf-8 -*-
"""讀取 .aeroplan 專案檔時，攻擊那一系的狀態（打擊目標／STOT 基地）只在開關打開時還原。

審查 2026-10-17 抓到：開關關著、工具列鈕與分頁都拿掉了，但「讀取專案」仍會把檔案裡的目標畫到 3D 圖上，
而 UI 上沒有任何入口能清掉它。
"""
from __future__ import annotations

import logging
import types

import pytest

from config import features
from mission.project_io import AeroPlanProject, apply_project_to_main_window


class _Cesium:
    def __init__(self):
        self.added = []

    def strike_add_target(self, lat, lon, idx):
        self.added.append((lat, lon, idx))


class _Panel:
    def __init__(self):
        self.counts, self.bases = [], []

    def update_strike_target_count(self, n):
        self.counts.append(n)

    def update_strike_base_label(self, lat, lon):
        self.bases.append((lat, lon))


def _win():
    w = types.SimpleNamespace()
    w._strike_targets = []
    w._strike_launch_base = None
    w._cesium = _Cesium()
    w._get_cesium_widget = lambda: w._cesium
    w.parameter_panel = _Panel()
    w.flight_params = {}
    return w


def _project():
    return AeroPlanProject(strike_targets=[(23.45, 120.28), (23.46, 120.29)], strike_launch_base=(23.44, 120.27))


def test_flag_off_ignores_targets_and_base_but_says_so(monkeypatch, caplog):
    monkeypatch.setattr(features, 'STRIKE_ENABLED', False)
    w = _win()
    with caplog.at_level(logging.INFO, logger='mission.project_io'):
        apply_project_to_main_window(_project(), w)
    assert w._strike_targets == [] and w._strike_launch_base is None
    assert w._cesium.added == [] and w.parameter_panel.counts == [] and w.parameter_panel.bases == []
    assert any('2 個打擊目標' in r.getMessage() and '略過' in r.getMessage() for r in caplog.records), \
        [r.getMessage() for r in caplog.records]


def test_flag_on_restores_them(monkeypatch):
    monkeypatch.setattr(features, 'STRIKE_ENABLED', True)
    w = _win()
    apply_project_to_main_window(_project(), w)
    assert w._strike_targets == [(23.45, 120.28), (23.46, 120.29)] and w._strike_launch_base == (23.44, 120.27)
    assert len(w._cesium.added) == 2 and w.parameter_panel.counts == [2] and w.parameter_panel.bases == [(23.44, 120.27)]


def test_file_without_targets_logs_nothing(monkeypatch, caplog):
    monkeypatch.setattr(features, 'STRIKE_ENABLED', False)
    w = _win()
    with caplog.at_level(logging.INFO, logger='mission.project_io'):
        apply_project_to_main_window(AeroPlanProject(), w)
    assert not [r for r in caplog.records if '打擊目標' in r.getMessage()]
