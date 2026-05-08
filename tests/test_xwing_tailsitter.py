"""XWingSITLParamGenerator 單元測試

X-Wing Tailsitter 的 SITL 參數若有錯會直接失事墜毀，
因此此測試驗證「關鍵物理約束」不被意外修改。
"""
import os
import re
import tempfile

import pytest

from mission.xwing_tailsitter_params import (
    XWingSITLParamGenerator, XWingTailsitterConfig,
)


class TestXWingConfig:
    """設定資料類的驗證"""

    def test_defaults_are_safe(self):
        c = XWingTailsitterConfig()
        # 關鍵三項必須正確
        assert c.q_tailsit_enable == 2, 'Q_TAILSIT_ENABLE 必須 = 2 (Copter Motor Tailsitter)'
        assert c.q_tailsit_motmx == 15, 'Q_TAILSIT_MOTMX 必須 = 15 (0b1111 全馬達)'
        assert c.q_frame_type == 17, 'Q_FRAME_TYPE 必須 = 17 (NYT QUAD X)'
        # Safety bit
        assert c.q_options & 262144, 'Q_OPTIONS 必須含 bit 18 (Only Arm in QMode)'

    def test_transition_angle_safe_range(self):
        c = XWingTailsitterConfig()
        assert 30 <= c.q_tailsit_angle <= 80, (
            'Q_TAILSIT_ANGLE 應在 30~80° 之間 (< 30 太早, > 80 太晚)'
        )

    def test_transition_rates_positive(self):
        c = XWingTailsitterConfig()
        assert c.q_tailsit_rat_fw > 0
        assert c.q_tailsit_rat_vt > 0
        # VTOL 方向 (抬頭) 應 >= FW 方向 (緊急減速)
        assert c.q_tailsit_rat_vt >= c.q_tailsit_rat_fw

    def test_airspeed_ordering(self):
        c = XWingTailsitterConfig()
        assert c.airspeed_min_mps < c.airspeed_cruise_mps < c.airspeed_max_mps


class TestXWingExport:
    """檔案匯出測試"""

    def test_export_creates_file(self):
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'xwing.parm')
            out = XWingSITLParamGenerator().export(path)
            assert os.path.exists(out)
            assert out.endswith('.parm')

    def test_auto_extension(self):
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'xwing_noext')
            out = XWingSITLParamGenerator().export(path)
            assert out.endswith('.parm')

    def test_critical_params_in_output(self):
        """產出檔必須含三個關鍵物理約束參數"""
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'x.parm')
            XWingSITLParamGenerator().export(path)
            with open(path, 'r', encoding='utf-8') as f:
                content = f.read()

        # 三大關鍵參數 (會破壞飛行的那些)
        assert re.search(r'^Q_TAILSIT_ENABLE\s+2\b', content, re.MULTILINE)
        assert re.search(r'^Q_TAILSIT_MOTMX\s+15\b', content, re.MULTILINE)
        assert re.search(r'^Q_FRAME_TYPE\s+17\b', content, re.MULTILINE)
        assert re.search(r'^Q_ENABLE\s+1\b', content, re.MULTILINE)

    def test_safety_banner_present(self):
        """安全性警告註解必須在檔頭"""
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'x.parm')
            XWingSITLParamGenerator().export(path)
            with open(path, 'r', encoding='utf-8') as f:
                head = f.read(2000)   # 讀前 2KB
        assert '勁蜂三型' in head or 'X-Wing' in head
        assert 'ArduPlane.exe -f tailsitter' in head

    def test_q_options_bit18_in_output(self):
        """Q_OPTIONS 必須含 bit 18 (262144) — 防地面暴衝的最後防線"""
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'x.parm')
            XWingSITLParamGenerator().export(path)
            with open(path, 'r', encoding='utf-8') as f:
                content = f.read()
        # 找 Q_OPTIONS 那行，確認值 & 262144 != 0
        m = re.search(r'^Q_OPTIONS\s+(\d+)', content, re.MULTILINE)
        assert m, '產出檔未找到 Q_OPTIONS'
        val = int(m.group(1))
        assert val & 262144, f'Q_OPTIONS={val} 未設 bit 18 (Only Arm in QMode)'

    def test_all_4_servos_configured(self):
        """必須 SERVO1~4 全數配置 (X-Wing 4 馬達)"""
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'x.parm')
            XWingSITLParamGenerator().export(path)
            with open(path, 'r', encoding='utf-8') as f:
                content = f.read()
        for i in range(1, 5):
            assert re.search(rf'^SERVO{i}_FUNCTION\s+\d+', content, re.MULTILINE)
            assert re.search(rf'^SERVO{i}_MIN', content, re.MULTILINE)
            assert re.search(rf'^SERVO{i}_MAX', content, re.MULTILINE)


class TestCustomConfig:
    """客製化配置的驗證"""

    def test_extra_params_written(self):
        cfg = XWingTailsitterConfig(extra_params={
            'SYSID_THISMAV': 42,
            'LOG_BITMASK': 65535,
        })
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'x.parm')
            XWingSITLParamGenerator(cfg).export(path)
            with open(path, 'r', encoding='utf-8') as f:
                content = f.read()
        assert 'SYSID_THISMAV' in content
        assert 'LOG_BITMASK' in content

    def test_custom_transition_angle(self):
        cfg = XWingTailsitterConfig(q_tailsit_angle=55.0)
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'x.parm')
            XWingSITLParamGenerator(cfg).export(path)
            with open(path, 'r', encoding='utf-8') as f:
                content = f.read()
        assert re.search(r'^Q_TAILSIT_ANGLE\s+55\b', content, re.MULTILINE)


class TestSummary:
    def test_summary_shows_critical_params(self):
        s = XWingSITLParamGenerator().summary()
        assert 'Q_TAILSIT_ENABLE' in s
        assert 'Q_TAILSIT_MOTMX' in s
        assert 'NYT QUAD X' in s
