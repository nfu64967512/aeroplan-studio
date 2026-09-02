"""幽靈 UAV 卡片：只有真的是飛機的來源才配一張卡片。

⚠️ 這是 2026-09-02 實測到的：AeroPlan 接 WSL 蜂群時，三條鏈路上各冒出一張
   `UAV191` 卡片（MODE_0 / 0 sat / 0.0 V / 座標 0,0）。那不是飛機 —— 是我們
   自己的 MAVROS（swarm_strike/launch_helpers.py: ``GCS_SYSTEM_ID = 191``），
   飛控把它的心跳轉發到 SERIAL0，地面站就替一台機載電腦開了卡片。

測試策略：**值層級**，不鎖原始碼長什麼樣。每條都直接餵一包合成 HEARTBEAT
給真正的分派函式，然後看「有沒有生出 TelemetryFrame」這個外顯結果。
改上游、改下游、換寫法都免疫；把守衛拆掉就一定轉紅。

順帶：每一條「應該被擋掉」的測試旁邊都有一條「同一條路徑放行真飛機」的
對照，證明這個儀器**真的會動** —— 否則一個永遠回 False 的守衛也會全綠。
"""
import pytest

from mission.sitl_link import SITLLink

# ArduPilot SITL 實測值（2026-09-02，ArduCopter tcp:127.0.0.1:5790）
REAL_TYPE, REAL_AUTOPILOT = 2, 3          # MAV_TYPE_QUADROTOR / ARDUPILOTMEGA
# MAVROS 機載電腦節點
MAVROS_TYPE, MAVROS_AUTOPILOT = 18, 8     # ONBOARD_CONTROLLER / AUTOPILOT_INVALID


class FakeMsg:
    """最小可用的 MAVLink 訊息替身。"""

    def __init__(self, mtype, src, **fields):
        self._mtype = mtype
        self._src = src
        # HEARTBEAT 的必要欄位給合理預設，個別測試再覆寫
        self.type = REAL_TYPE
        self.autopilot = REAL_AUTOPILOT
        self.base_mode = 0
        self.custom_mode = 0
        self.system_status = 3
        for k, v in fields.items():
            setattr(self, k, v)

    def get_type(self):
        return self._mtype

    def get_srcSystem(self):
        return self._src

    def get_srcComponent(self):
        return 1


def make_link(sysid_label=1):
    """建一條不啟動執行緒的 SITLLink（只用它的解析邏輯）。"""
    return SITLLink(conn_str='tcp:127.0.0.1:5760', sysid_label=sysid_label)


def heartbeat(src, mtype=REAL_TYPE, autopilot=REAL_AUTOPILOT):
    return FakeMsg('HEARTBEAT', src, type=mtype, autopilot=autopilot)


# ── 分類函式本身 ────────────────────────────────────────────────────
def test_real_ardupilot_heartbeat_is_a_vehicle():
    """對照組：真飛機一定要放行，否則整個地面站瞎掉。"""
    link = make_link()
    assert link._heartbeat_is_vehicle(heartbeat(1)) is True


def test_mavros_heartbeat_is_not_a_vehicle():
    link = make_link()
    assert link._heartbeat_is_vehicle(
        heartbeat(191, MAVROS_TYPE, MAVROS_AUTOPILOT)) is False


@pytest.mark.parametrize('mav_type', [5, 6, 18, 26, 27, 30, 31, 32,
                                      36, 37, 38, 39, 40])
def test_non_vehicle_types_are_rejected_even_with_valid_autopilot(mav_type):
    """只靠 autopilot 一條會漏 —— 節點若填了合法 autopilot 仍要靠 type 擋下。"""
    link = make_link()
    assert link._heartbeat_is_vehicle(
        heartbeat(191, mav_type, REAL_AUTOPILOT)) is False


def test_invalid_autopilot_rejected_even_with_vehicle_type():
    """只靠 type 一條也會漏 —— 節點若自稱 GENERIC(0) 要靠 autopilot 擋下。"""
    link = make_link()
    assert link._heartbeat_is_vehicle(heartbeat(191, 0, 8)) is False


@pytest.mark.parametrize('mav_type', [1, 2, 3, 4, 13, 14, 19, 20, 21, 22])
def test_real_vehicle_types_all_pass(mav_type):
    """固定翼/多旋翼/直升機/VTOL 全部都要過。"""
    link = make_link()
    assert link._heartbeat_is_vehicle(heartbeat(1, mav_type, 3)) is True


# ── 端到端：卡片到底有沒有生出來 ─────────────────────────────────────
def test_mavros_source_creates_no_telemetry_frame():
    """核心不變式：191 進來，不可以有 frame。

    把守衛拆掉這條就轉紅 —— frame 會被建立，HUD 就會多一張 UAV191。
    """
    link = make_link(sysid_label=1)
    link._handle_message(heartbeat(191, MAVROS_TYPE, MAVROS_AUTOPILOT))
    assert 191 not in link._frames
    assert link._frames == {}


def test_real_vehicle_source_does_create_a_frame():
    """儀器能不能動的證明：同一條路徑，真飛機**必須**生出 frame。

    沒有這一條的話，一個「什麼都擋」的壞守衛會讓上面那條測試全綠。
    """
    link = make_link(sysid_label=1)
    link._handle_message(heartbeat(1))
    assert 1 in link._frames


def test_extra_real_vehicle_on_same_link_still_gets_a_frame():
    """mavproxy 把多架 fan-in 到同一條 --out 的既有行為不可以被打壞。"""
    link = make_link(sysid_label=1)
    link._handle_message(heartbeat(1))
    link._handle_message(heartbeat(7))
    assert {1, 7} <= set(link._frames)


def test_telemetry_without_heartbeat_creates_no_frame():
    """只送遙測不送心跳的節點，一樣不可以生出卡片。

    分類資訊只在心跳裡；先收到 GLOBAL_POSITION_INT 就建 frame 的話，
    守衛等於被繞過。
    """
    link = make_link(sysid_label=1)
    link._handle_message(FakeMsg('GLOBAL_POSITION_INT', 191,
                                 lat=0, lon=0, alt=0, relative_alt=0,
                                 hdg=0, vx=0, vy=0, vz=0))
    assert 191 not in link._frames


def test_labelled_sysid_is_always_allowed():
    """本鏈路 UI 標示的機號永遠放行 —— 不能因為分類把正主擋掉。"""
    link = make_link(sysid_label=191)
    link._handle_message(heartbeat(191, MAVROS_TYPE, MAVROS_AUTOPILOT))
    assert 191 in link._frames


def test_non_vehicle_is_logged_once_not_every_heartbeat():
    """1 Hz 心跳會把日誌洗掉 —— 只記一次。"""
    link = make_link()
    for _ in range(10):
        link._handle_message(heartbeat(191, MAVROS_TYPE, MAVROS_AUTOPILOT))
    assert link._nonvehicle_srcs == {191}


def test_gcs_echo_and_zero_source_still_filtered():
    """既有的 0 / 255 過濾不可以被新守衛取代掉。"""
    link = make_link()
    link._handle_message(heartbeat(255))
    link._handle_message(heartbeat(0))
    assert link._frames == {}
