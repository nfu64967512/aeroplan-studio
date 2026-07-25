#!/usr/bin/env python3
"""jet_catapult_launch.py — 6 架 plane-jet-catapult 彈射起飛。

彈射鏈(SIM_Plane.cpp:410 `launch_triggered = input.servos[6] > 1700`):
  SERVO7_FUNCTION=0(Disabled=手動)→ MAV_CMD_DO_SET_SERVO(183) ch7=2000
  → 伺服輸出 7 拉高 → SITL 施加 launch_accel 15 m/s²(2 秒)
  → 前向加速度超過 TKOFF_THR_MINACC(10)→ ArduPlane TAKEOFF 解除馬達抑制 → 爬升。

⚠️ 不能用 RC_CHANNELS_OVERRIDE:那是灌「RC 輸入」,要 SERVO7 設成 RCIN7 直通才會
   傳到輸出;而且沒有 RC 幀時通道維持 trim。實測 override 期間 servo7 仍是 1000。
   DO_SET_SERVO 直接寫輸出,但通道必須是 Disabled,否則每迴圈被指派功能蓋回去。

用法: python3 jet_catapult_launch.py [stagger_s] [climb_wait_s]
"""
import sys, time, math
from pymavlink import mavutil

BASE = 6960          # 走節點分流(AeroPlan 佔 6760 家族)
STRIDE = 10
N = 6
TAKEOFF_MODE = 13    # ArduPlane TAKEOFF
GUIDED_MODE = 15


def conn(i):
    c = mavutil.mavlink_connection(f"tcp:127.0.0.1:{BASE + STRIDE * (i - 1)}",
                                   source_system=180 + i, dialect="ardupilotmega")
    c.wait_heartbeat(timeout=10)
    return c


def setp(c, name, val):
    c.mav.param_set_send(c.target_system, 1, name.encode(), float(val),
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)


def fire_catapult(c, ch=7, pwm=2000):
    """MAV_CMD_DO_SET_SERVO(183):直接把伺服輸出通道 ch 壓到 pwm。"""
    c.mav.command_long_send(c.target_system, 1,
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                            float(ch), float(pwm), 0, 0, 0, 0, 0)


def servo7(c):
    """讀回伺服輸出 7,驗證彈射器真的被觸發(>1700)。"""
    t0 = time.time(); last = None
    while time.time() - t0 < 1.5:
        m = c.recv_match(type="SERVO_OUTPUT_RAW", blocking=False)
        if m:
            last = m
        else:
            time.sleep(0.02)
    return last.servo7_raw if last else None


def alt_of(c):
    last = None; t0 = time.time()
    while time.time() - t0 < 1.2:
        m = c.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if m:
            last = m
        else:
            time.sleep(0.02)
    return last.relative_alt / 1000.0 if last else 0.0


def main():
    stagger = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
    climb_wait = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    links = {i: conn(i) for i in range(1, N + 1)}

    # 0) 確保彈射觸發鏈接好(SITL 已在跑時 .parm 不會重載 → 這裡補設)
    for i, c in links.items():
        setp(c, "SERVO7_FUNCTION", 0)
    time.sleep(2.0)

    # 1) 逐機:TAKEOFF 模式 → ARM → 點火彈射
    for i in range(1, N + 1):
        c = links[i]
        c.mav.set_mode_send(c.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            TAKEOFF_MODE)
        time.sleep(0.5)
        c.mav.command_long_send(c.target_system, 1,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                1, 0, 0, 0, 0, 0, 0)
        time.sleep(0.8)
        # 點火:壓住 ch7 = 2000 維持 3 秒(彈射推力 2 秒)
        t0 = time.time()
        while time.time() - t0 < 3.0:
            fire_catapult(c)
            time.sleep(0.2)
        s7 = servo7(c)
        fire_catapult(c, pwm=1000)          # 收回彈射臂
        print(f"UAV{i} 彈射! (servo7={s7} {'✓觸發' if (s7 or 0) > 1700 else '✗未觸發'})",
              flush=True)
        time.sleep(stagger)

    # 2) 等爬升
    t0 = time.time()
    while time.time() - t0 < climb_wait:
        alts = {i: round(alt_of(links[i])) for i in links}
        print(f"climb… {alts}", flush=True)
        if all(a > 90 for a in alts.values()):
            print("全員爬升過 90m", flush=True)
            break
        time.sleep(3)

    # 3) 轉 GUIDED 交給協同層
    for i, c in links.items():
        c.mav.set_mode_send(c.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            GUIDED_MODE)
    print("全員 GUIDED", flush=True)
    for i, c in links.items():
        print(f"UAV{i} alt={alt_of(c):.0f}m", flush=True)


if __name__ == "__main__":
    main()
