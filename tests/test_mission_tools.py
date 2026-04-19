"""mission/ 新工具的單元測試

覆蓋：
    - mission.mock_sitl_link
    - mission.mission_validator
    - mission.project_io
"""
import os
import tempfile

import pytest

from mission.mock_sitl_link import MockSITLLink, make_mock_fleet
from mission.mission_validator import validate_mission, ValidationResult
from mission.project_io import (
    AeroPlanProject, save_project, load_project,
)


# ═══════════════════════════════════════════════════════════════════════
#  MockSITLLink
# ═══════════════════════════════════════════════════════════════════════

class TestMockSITLLink:
    def test_basic_lifecycle(self):
        link = MockSITLLink(sysid_label='TEST-1')
        assert not link.is_connected()
        assert link.connect()
        assert link.is_connected()
        link.disconnect()
        assert not link.is_connected()

    def test_upload_recorded(self):
        link = MockSITLLink()
        link.connect()
        wps = [(25.0, 121.5, 100.0, 16, 0, 150, 0, 0)]
        link.upload_mission(wps)
        assert link.upload_count == 1
        assert link.last_uploaded_waypoints == wps

    def test_multiple_uploads_tracked(self):
        link = MockSITLLink()
        for i in range(5):
            link.upload_mission([(25.0, 121.5, 100, 16, 0, 0, 0, 0)])
        assert link.upload_count == 5
        assert len(link.all_uploaded_missions) == 5

    def test_simulated_failure(self):
        link = MockSITLLink()
        link.simulate_upload_failure = True
        with pytest.raises(RuntimeError, match='模擬'):
            link.upload_mission([(25.0, 121.5, 100, 16, 0, 0, 0, 0)])

    def test_command_log(self):
        link = MockSITLLink()
        link.arm()
        link.set_mode('AUTO')
        link.takeoff(100)
        assert 'ARM' in link.cmd_log
        assert any('AUTO' in c for c in link.cmd_log)
        assert any('TAKEOFF' in c for c in link.cmd_log)

    def test_fleet_factory(self):
        fleet = make_mock_fleet(3)
        assert len(fleet) == 3
        for i, link in enumerate(fleet):
            assert link.sysid_label == f'MOCK-{i + 1}'


# ═══════════════════════════════════════════════════════════════════════
#  Mission Validator
# ═══════════════════════════════════════════════════════════════════════

class TestMissionValidator:
    def test_empty_mission_fails(self):
        r = validate_mission([])
        assert not r.ok
        assert '空序列' in r.errors[0]

    def test_valid_mission_passes(self):
        wps = [
            (25.0, 121.5, 0, 179, 0, 0, 0, 0),      # DO_SET_HOME
            (25.0, 121.5, 0, 178, 0, 25, -1, 0),    # DO_CHANGE_SPEED
            (25.001, 121.501, 100, 22, 10, 0, 0, 0),  # TAKEOFF
            (25.01, 121.51, 100, 16, 0, 150, 0, 0),   # WAYPOINT
        ]
        r = validate_mission(wps)
        assert r.ok

    def test_unknown_cmd_rejected(self):
        wps = [(25.0, 121.5, 100, 9999, 0, 0, 0, 0)]
        r = validate_mission(wps)
        assert not r.ok
        assert any('9999' in e for e in r.errors)

    def test_invalid_latlon_rejected(self):
        wps = [
            (25.0, 121.5, 0, 179, 0, 0, 0, 0),
            (91.0, 121.5, 100, 16, 0, 0, 0, 0),     # lat > 90
            (25.0, 181.0, 100, 16, 0, 0, 0, 0),     # lon > 180
        ]
        r = validate_mission(wps)
        assert not r.ok
        assert any('lat' in e for e in r.errors)
        assert any('lon' in e for e in r.errors)

    def test_negative_altitude_rejected(self):
        wps = [
            (25.0, 121.5, 0, 179, 0, 0, 0, 0),
            (25.0, 121.5, -50.0, 16, 0, 0, 0, 0),
        ]
        r = validate_mission(wps)
        assert not r.ok
        assert any('alt' in e for e in r.errors)

    def test_warning_for_zero_latlon_on_nav(self):
        wps = [
            (25.0, 121.5, 0, 179, 0, 0, 0, 0),
            (0.0, 0.0, 100, 16, 0, 0, 0, 0),   # 赤道外海 WAYPOINT
        ]
        r = validate_mission(wps)
        assert r.ok   # 只是警告，非致命
        assert any('lat/lon' in w for w in r.warnings)

    def test_strict_mode_treats_warning_as_fail(self):
        wps = [
            (25.0, 121.5, 0, 179, 0, 0, 0, 0),
            (0.0, 0.0, 100, 16, 0, 0, 0, 0),
        ]
        r = validate_mission(wps, strict=True)
        assert not r.ok


# ═══════════════════════════════════════════════════════════════════════
#  Project I/O
# ═══════════════════════════════════════════════════════════════════════

class TestProjectIO:
    def test_save_load_roundtrip(self):
        proj = AeroPlanProject(
            corners=[(25.05, 121.55), (25.06, 121.56), (25.05, 121.57)],
            strike_targets=[(25.033, 121.5654)],
            strike_launch_base=(25.0, 121.5),
            vehicle_type='固定翼',
            notes='test',
        )
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'test.aeroplan')
            save_project(proj, path)
            assert os.path.exists(path)
            loaded = load_project(path)
        assert loaded.corners == proj.corners
        assert loaded.strike_targets == proj.strike_targets
        assert loaded.strike_launch_base == proj.strike_launch_base
        assert loaded.vehicle_type == proj.vehicle_type
        assert loaded.notes == proj.notes

    def test_auto_extension(self):
        proj = AeroPlanProject(corners=[(25, 121)])
        with tempfile.TemporaryDirectory() as td:
            # 不給副檔名 → 應自動補 .aeroplan
            path = os.path.join(td, 'noext')
            save_project(proj, path)
            assert os.path.exists(path + '.aeroplan')

    def test_version_mismatch_raises(self):
        # 寫入一個 version 超前的檔案 → load 時應報錯
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'future.aeroplan')
            import json
            with open(path, 'w', encoding='utf-8') as f:
                json.dump({'version': 999}, f)
            with pytest.raises(ValueError, match='版本'):
                load_project(path)
