#!/usr/bin/env python3
"""jet_show_ros2.py — 用 **ROS2 編隊層** 做長程編隊巡航(取代 jet_show_cruise.py)。

分工照 docs/swarm_architecture.md:
  ① MAVLink / ArduPilot(SITL)  單機飛控 —— 每架自己飛到被指定的點
  ② ROS2 / sar_swarm           編隊協同 —— wingman_node 依長機航向算 slot 並下航點

本腳本只做「② 管不到、但表演需要」的一件事:**驅動長機沿航線持續前進**。
`swarm_ros2_up.sh PATROL=0` 之下長機不自動巡邏,所以航路由這裡給。

⚠️ **長機一定要持續前進,不能給定點。**固定翼停不住,一給定點就會繞圈 ——
   那正是先前 jet_show_cruise 的 GATHER 段產生繞圈的原因。這裡的長機航點永遠
   放在「當前位置往目標方向再推 lead_ahead 公尺」,所以它一路直線飛,僚機追移動
   中的 slot 也就不會繞。

⚠️ 分流分家:5860+10i 給 MAVROS(ROS2 層要用)、6960+10i 給 tot_node,
   所以本腳本走 6760+10i(AeroPlan 那條,本組態沒用到)。搶同一條會讓先連者
   獨佔遙測,後連的讀不到位置。

用法(環境變數):
  HANDOFF_D   交棒距離(m,距紅心);到達就結束,讓 stot_trial 接著跑 tot_node
  LEAD_AHEAD  長機航點放在前方多遠
  CRUISE_ALT / CRUISE_V
"""
import math
import os
import sys
import time

from pymavlink import mavutil

EARTH = 111320.0
T = (float(os.environ.get("TGT_LAT", "25.1406")),
     float(os.environ.get("TGT_LON", "121.2325")))
BASE = int(os.environ.get("SHOW_LINK_BASE", "6760"))
STRIDE = int(os.environ.get("SHOW_LINK_STRIDE", "10"))
# ⚠️⚠️ N 先前寫死 6,而同目錄的 jet_catapult_launch.py:20 是讀環境變數的。
#    後果(2026-08-06 實測):兩機航次它照樣去連 UAV3–6 的 6780/6790/6800/6810
#    ⇒ `ConnectionRefusedError: [Errno 111]` 當場崩在 :120 的
#    `links = {i: conn(i) for i in range(1, N + 1)}`。
#    而 stot_trial.sh 呼叫本檔時 **rc 是丟掉的** ⇒ 整個巡航段靜默跳過,
#    機隊維持彈射後的正北航點,直到 tot_node 的 ALIGN 接手才被帶去扇形。
#    ⇒ TWO1 / KIL2 / KIL3 三趟的「巡航編隊」從未執行過,ALIGN 跑了 963 s 做完全部。
#    名字用 SWARM_COUNT 優先(批次鏈的正式名),N 是相容別名。
N = int(os.environ.get("SWARM_COUNT") or os.environ.get("N") or "6")
LEADER = 1

HANDOFF_D = float(os.environ.get("HANDOFF_D", "15000"))
LEAD_AHEAD = float(os.environ.get("LEAD_AHEAD", "6000"))
CRUISE_ALT = float(os.environ.get("SHOW_CRUISE_ALT", "3100"))
CRUISE_V = float(os.environ.get("SHOW_CRUISE_V", "75"))
CLIMB_V = float(os.environ.get("SHOW_CLIMB_V", "62"))
                               # 爬升階段的長機速度。要慢到讓僚機**同時**追得上又爬得動。
ALT_TOL = float(os.environ.get("SHOW_ALT_TOL", "250"))
# 手風琴:落後超過這個距離,長機降到 WAIT_V 等。門檻要略大於編隊縱深
# (rank2 的 slot 本來就在長機後方 ~865 m),否則長機會被自己的隊形拖住。
FORM_LAG_MAX = float(os.environ.get("SHOW_FORM_LAG_MAX", "1500"))
WAIT_V = float(os.environ.get("SHOW_WAIT_V", "80"))
# ── 垂直通道差速協同(Vertical Channel Speed Decoupling)──
# ⭐ 長機讓出的速度不要用「收油門」丟掉,要用「爬升」存成位能。
#    物理:長機在 90 EAS 的剩餘推力可爬 25.6 m/s。平飛時 TECS 只能收油門把這份
#    功率丟掉;讓它爬,同樣的速度差(地速 105 vs 僚機 183 ⇒ 閉合 78 m/s)照樣拿到,
#    而且 60 s 存下 ~1540 m 位能,終端俯衝再換回速度。
#    ⚠️ 垂直機動**不佔水平航跡**,不需要在終端繞圈 —— 與「全程直線」的約束正交。
#    ⚠️ 僚機維持原巡航高度全力平飛衝刺(它們的高度由 wingman_node 的 formation_alt
#      決定,本腳本不碰)⇒ 自然形成「長機高 / 僚機平」的非對稱幾何。
VERT_COORD = int(os.environ.get("SHOW_VERT_COORD", "1"))
VERT_DZ_MAX = float(os.environ.get("SHOW_VERT_DZ_MAX", "1500"))  # 最多存多少高度
VERT_DZ_RATE = float(os.environ.get("SHOW_VERT_DZ_RATE", "20"))  # 每秒抬高多少(m/s)

# 1 = 本脚本發長機速度；0 = 交給 master_node 的 max-consensus（見下方說明）
# ⚠️⚠️ **預設 0 = 本腳本不發速度**。編隊模式下速度的唯一權威是 master_node
#    的 max-consensus —— 它拿得到僚機對自己 slot 的真實誤差,比本腳本從位置
#    估的 lag 準得多。兩個控制器同時發 DO_CHANGE_SPEED 會讓執行器收到
#    非連續指令、產生極限環抖動。只有在**沒有 ROS2 編隊層**的獨立測試才設 1。
LEAD_SPEED = int(os.environ.get("SHOW_LEAD_SPEED", "0"))
                               # 全隊最低高度離巡航高度多近才算爬完
TIMEOUT = float(os.environ.get("SHOW_TIMEOUT", "900"))
REC = os.environ.get("SHOW_REC", os.path.expanduser(
    "~/fixwing_ws/runs/show_%s.csv" % os.environ.get("TRIAL", "x")))


def mlon(lat):
    return EARTH * math.cos(math.radians(lat))


def dist_m(a, b):
    return math.hypot((b[0] - a[0]) * EARTH, (b[1] - a[1]) * mlon(a[0]))


def dest(lat, lon, brg, d):
    b = math.radians(brg)
    return (lat + d * math.cos(b) / EARTH, lon + d * math.sin(b) / mlon(lat))


def bearing(a, b):
    return math.degrees(math.atan2((b[1] - a[1]) * mlon(a[0]),
                                   (b[0] - a[0]) * EARTH)) % 360.0


def conn(i):
    c = mavutil.mavlink_connection(f"tcp:127.0.0.1:{BASE + STRIDE * (i - 1)}",
                                   source_system=170 + i, dialect="ardupilotmega")
    c.wait_heartbeat(timeout=10)
    c.mav.request_data_stream_send(c.target_system or i, 1,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    return c


_pos = {}


def pos_of(c, i, window=0.3):
    last = None
    t0 = time.time()
    while time.time() - t0 < window:
        m = c.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if m:
            last = m
        elif last:
            break
        else:
            time.sleep(0.01)
    if last:
        _pos[i] = (last.lat / 1e7, last.lon / 1e7, last.relative_alt / 1000.0)
    return _pos.get(i)


def main():
    links = {i: conn(i) for i in range(1, N + 1)}
    print("[ros2-show] 連線就緒(分流 %d+)" % BASE, flush=True)
    print("[ros2-show] 控制權威:航點=本腳本;速度=%s"
          % ("本腳本(無編隊層)" if LEAD_SPEED else "master_node(max-consensus)"),
          flush=True)
    if not LEAD_SPEED:
        print("[ros2-show] ⚠️ 已封鎖本腳本的 DO_CHANGE_SPEED —— "
              "避免與 master_node 爭速度權威", flush=True)
    rec = open(REC, "w", newline="")
    rec.write("t,sysid,lat,lon,alt,phase\n")

    t0 = time.time()
    last_wp = None
    phase = "CLIMB"
    vert_dz = 0.0            # 垂直通道目前存了多少高度(m)
    t_prev = time.time()
    while time.time() - t0 < TIMEOUT:
        p = pos_of(links[LEADER], LEADER)
        if p is None:
            time.sleep(0.5)
            continue
        now_t = time.time()
        dt = min(2.0, max(0.0, now_t - t_prev))
        t_prev = now_t
        d = dist_m(p[:2], T)
        # ⚠️⚠️ 爬升階段長機必須放慢。低推重比機體的僚機沒辦法同時「追長機」和「爬升」
        #    —— 追趕時速度飽和,那個速度下剩餘功率接近零。長機自己沒有追趕負擔,
        #    所以會一路爬到巡航高度而把僚機丟在低空(實測 P1:長機 3625 m、
        #    僚機只有 1063–1475 m,一架還落地)。拉長航程救不了,要把剩餘功率讓出來。
        #    全程仍是直線,不需要任何盤旋。
        lo_alt = min((q[2] for q in _pos.values()), default=0.0)
        if phase == "CLIMB" and lo_alt >= CRUISE_ALT - ALT_TOL:
            phase = "CRUISE"
            print("[ros2-show] 全隊抵巡航高度(最低 %.0f m)→ 加速巡航" % lo_alt,
                  flush=True)
        v_cmd = CLIMB_V if phase == "CLIMB" else CRUISE_V
        # ⭐⭐ 手風琴:隊形沒成形之前,長機不准全速前進。
        #
        # ⚠️⚠️ CRUISE_MODE=ros2 之下長機是由本腳本驅動,**不是 master_node** ——
        #    所以 sar_swarm 內建的「隊形沒成形就降到 wait_speed 等」那套機制
        #    被整個繞過了。wait_speed 參數有傳進去,但沒有任何東西會去用它。
        #    後果:長機一路加速,僚機永遠在追一個從彈射就存在的缺口。
        #    實測 AJ1:六機依序彈射,UAV6 離地時長機已跑掉 3.2 km;爬升段長機
        #    又持續拉開,t=339 s 時 UAV6 落後 **11.3 km**。而追趕權限只有 26 m/s
        #    地速 ⇒ 追平要 435 s,交棒前根本來不及,隊形因此永遠是一條縱列。
        #    ⚠️ 這不是「在目標上空盤旋」—— 全程仍是直線,只是長機把速度讓出來。
        lag = 0.0
        for q in _pos.values():
            # 沿「長機→目標」方向的落後量(只算落後,超前不計)
            lag = max(lag, dist_m(q[:2], p[:2])
                      if dist_m(q[:2], T) > d else 0.0)
        if lag > FORM_LAG_MAX:
            v_cmd = min(v_cmd, WAIT_V)
        # ⚠️⚠️ 兩個控制器不能同時對長機發 DO_CHANGE_SPEED。master_node 的
        #    `_leader_speed_control` 在 operator 模式(auto_patrol=False)下**仍會執行**
        #    —— 它在那個 early return 之前(master_node.py:252)。而它拿得到僚機
        #    對自己 slot 的真實誤差(_formed 的 worst),比本腳本只能從位置估的
        #    lag 準得多,而且已經改成 max-consensus 的連續律。
        #    ⇒ 有 master_node 在場時,本腳本只負責**航點**,速度交給它。
        #    LEAD_SPEED=0 就是這個模式;=1 才由本腳本發速度(無 ROS2 層時用)。
        if not LEAD_SPEED:
            v_cmd = None
        # ⭐ 垂直通道:落後時把長機的航點高度往上抬,讓它「爬著等」而不是「收油門等」。
        #    抬升速率刻意設得比可達爬升率(25.6 m/s)慢一點,讓 TECS 追得上、
        #    不會為了追高度而犧牲空速(那正是 UAV6 失速的機制)。
        if VERT_COORD and lag > FORM_LAG_MAX:
            vert_dz = min(VERT_DZ_MAX, vert_dz + VERT_DZ_RATE * dt)
        elif vert_dz > 0.0:
            vert_dz = max(0.0, vert_dz - VERT_DZ_RATE * dt)   # 收攏後緩慢還原
        # 長機航點:永遠放在「往目標方向再推 LEAD_AHEAD」,所以它不會停下來繞圈
        brg = bearing(p[:2], T)
        aim = dest(p[0], p[1], brg, min(LEAD_AHEAD, max(500.0, d - HANDOFF_D * 0.5)))
        if last_wp is None or dist_m(last_wp, aim) > 400.0:
            c = links[LEADER]
            if v_cmd is not None:
                c.mav.command_long_send(c.target_system or LEADER, 1,
                                        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                                        0, v_cmd, -1, 0, 0, 0, 0)
            c.mav.command_int_send(
                c.target_system or LEADER, 1,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1.0, 0.0, 0.0,
                float("nan"), int(aim[0] * 1e7), int(aim[1] * 1e7),
                CRUISE_ALT + vert_dz)
            last_wp = aim
        # 記錄全隊
        el = time.time() - t0
        for i in links:
            q = pos_of(links[i], i, window=0.05)
            if q:
                rec.write("%.1f,%d,%.7f,%.7f,%.1f,CRUISE\n"
                          % (el, i, q[0], q[1], q[2]))
        rec.flush()
        if int(el) % 20 == 0:
            spread = 0.0
            pts = [_pos[i][:2] for i in _pos]
            for a in pts:
                for b in pts:
                    spread = max(spread, dist_m(a, b))
            print("  t+%4.0fs [%s] 長機距T %6.0f m  alt %4.0f  "
                  "全隊最低 alt %4.0f  最大間距 %5.0f m"
                  % (el, phase, d, p[2], lo_alt, spread), flush=True)
            if vert_dz > 1.0:
                print("       ↑ 垂直協同:長機已存 %.0f m 位能(落後 %.0f m)"
                      % (vert_dz, lag), flush=True)
            time.sleep(1.0)
        if d <= HANDOFF_D:
            print("[ros2-show] 抵達交棒距離 %.0f m —— 交給 tot_node" % d, flush=True)
            break
        time.sleep(0.5)
    rec.close()
    for i in sorted(_pos):
        print("  UAV%d 距T=%.0f m alt=%.0f m"
              % (i, dist_m(_pos[i][:2], T), _pos[i][2]), flush=True)


if __name__ == "__main__":
    main()
