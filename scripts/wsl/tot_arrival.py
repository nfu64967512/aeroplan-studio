#!/usr/bin/env python3
"""蜂群同時抵達(TOT / Time-on-Target)編排器 — 獨立驅動 6 機(6760 分流)。

概念:
  目標 T 前方一個「扇形閘門」= 以 T 為圓心、半徑 R_fan 的弧,跨 ±half_angle,
  上面均佈 N 個 slot(徑向切線進場、彼此橫向分離→防撞)。六機各綁一個 slot,
  在共同時刻 T_arr 同時抵達各自 slot。速度控制做大部分時間對齊(慢的加速、
  快的放慢);快到連 v_min 都會早到的,做 S 型機動燒掉多餘時間。

不需要 ROS2 節點(TOT 是與編隊跟隨不同的任務模式)——執行前請先停 sar_swarm
節點層,由本編排器獨佔 6 機導引。

用法:
  python3 tot_arrival.py --tlat 25.1585 --tlon 121.2325 --talt 90 \
      --approach-from 180 --r-fan 2000 --half-angle 30 \
      --vmin 14 --vmax 31 --margin 25 --out /tmp/tot
產出:<out>.csv(逐機軌跡+指令+模式)、抵達時間散布印到 stdout。
"""
import argparse, csv, math, time
from pymavlink import mavutil

EARTH = 111320.0


def m_per_deg_lon(lat):
    return EARTH * math.cos(math.radians(lat))


def dest(lat, lon, brg_deg, dist_m):
    b = math.radians(brg_deg)
    return (lat + dist_m * math.cos(b) / EARTH,
            lon + dist_m * math.sin(b) / m_per_deg_lon(lat))


def dist_m(a, b):
    dn = (b[0] - a[0]) * EARTH
    de = (b[1] - a[1]) * m_per_deg_lon(a[0])
    return math.hypot(dn, de)


def bearing(a, b):
    dn = (b[0] - a[0]) * EARTH
    de = (b[1] - a[1]) * m_per_deg_lon(a[0])
    return math.degrees(math.atan2(de, dn)) % 360.0


class Plane:
    def __init__(self, sysid):
        self.sysid = sysid
        port = 6760 + 10 * (sysid - 1)
        self.c = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}",
                                            source_system=234,
                                            dialect="ardupilotmega")
        self.c.wait_heartbeat(timeout=6)
        self.tgt = self.c.target_system or sysid
        self.pos = None          # (lat, lon, relalt)
        self.gs = 0.0
        self.slot = None
        self.arrived_t = None
        self.mode = "init"
        self.last_v = 0.0
        self.last_v_t = 0.0

    def pump(self):
        while True:
            m = self.c.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
            if m is None:
                break
            self.pos = (m.lat / 1e7, m.lon / 1e7, m.relative_alt / 1000.0)
            self.gs = math.hypot(m.vx, m.vy) / 100.0

    def set_mode_guided(self):
        self.c.mav.command_long_send(
            self.tgt, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15, 0, 0, 0, 0, 0)

    def goto(self, lat, lon, alt, v):
        now = time.time()
        if abs(v - self.last_v) > 0.5 or (now - self.last_v_t) > 2.0:
            self.c.mav.command_long_send(
                self.tgt, 1, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                0, float(v), -1, 0, 0, 0, 0)
            self.last_v, self.last_v_t = v, now
        self.c.mav.command_int_send(
            self.tgt, 1, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0,
            -1.0, 0.0, 0.0, float("nan"),
            int(lat * 1e7), int(lon * 1e7), float(alt))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--tlat", type=float, required=True)
    p.add_argument("--tlon", type=float, required=True)
    p.add_argument("--talt", type=float, default=90.0)
    p.add_argument("--approach-from", type=float, default=180.0,
                   help="蜂群從哪個方位進場(度,正北CW);扇形在 T 的這一側")
    p.add_argument("--r-fan", type=float, default=2000.0)
    p.add_argument("--half-angle", type=float, default=30.0)
    p.add_argument("--n", type=int, default=6)
    p.add_argument("--vmin", type=float, default=14.0)
    p.add_argument("--vmax", type=float, default=31.0)
    p.add_argument("--margin", type=float, default=25.0, help="T_arr 額外餘裕 s")
    p.add_argument("--plan-frac", type=float, default=0.9,
                   help="規劃速度=此比例×vmax(給約束機速度餘裕,收緊散布)")
    p.add_argument("--reach", type=float, default=120.0, help="判定抵達 slot 半徑 m")
    p.add_argument("--rate", type=float, default=3.0, help="控制迴圈 Hz")
    p.add_argument("--timeout", type=float, default=600.0)
    p.add_argument("--out", default="/tmp/tot")
    a = p.parse_args()

    T = (a.tlat, a.tlon)
    # ── 扇形 slot:以 T 為圓心、R_fan 半徑,bearing-from-T = approach_from ± θ ──
    slots = []
    for k in range(a.n):
        off = -a.half_angle + (2 * a.half_angle) * k / (a.n - 1)
        brg = (a.approach_from + off) % 360.0
        slots.append(dest(T[0], T[1], brg, a.r_fan))       # slot 在 T 的進場側
    spacing = a.r_fan * math.radians(2 * a.half_angle) / (a.n - 1)
    print(f"[tot] 扇形:R={a.r_fan:.0f}m ±{a.half_angle:.0f}° {a.n} slot,"
          f"相鄰間隔 {spacing:.0f} m", flush=True)

    planes = [Plane(i) for i in range(1, a.n + 1)]
    for pl in planes:
        pl.set_mode_guided()
    time.sleep(1.0)
    for _ in range(20):
        for pl in planes:
            pl.pump()
        if all(pl.pos for pl in planes):
            break
        time.sleep(0.2)

    # ── slot 指派:依「相對進場軸的橫向位置」排序,左機配左 slot(航跡不交叉)──
    axis = a.approach_from                     # 進場軸方位
    def cross(pl):                              # 橫向座標(對進場軸的垂直分量)
        b = bearing(T, pl.pos[:2])             # T→機 的方位
        return math.sin(math.radians(b - axis)) * dist_m(T, pl.pos[:2])
    order = sorted(planes, key=cross)          # 左→右
    slot_order = sorted(range(a.n),
                        key=lambda k: math.sin(math.radians(
                            (a.approach_from - a.half_angle
                             + 2 * a.half_angle * k / (a.n - 1)) - axis)))
    for pl, k in zip(order, slot_order):
        pl.slot = slots[k]
        pl.slot_id = k

    # ── T_arr:最慢(最遠)那台定義最早可行同時刻,其餘皆有餘裕 ──
    # 用「規劃速度」v_plan = plan_frac·vmax(< vmax)給約束機留爬升/機動的
    # 速度餘裕,否則它得全程貼 vmax、稍有損耗就晚到(iter1 UAV3 晚 4.1s)。
    t0 = time.time()
    v_plan = a.plan_frac * a.vmax
    tmax = max(dist_m(pl.pos[:2], pl.slot) / v_plan for pl in planes)
    T_arr = t0 + tmax + a.margin
    print(f"[tot] 各機到 slot 距離: "
          f"{ {pl.sysid: round(dist_m(pl.pos[:2], pl.slot)) for pl in planes} }",
          flush=True)
    print(f"[tot] T_arr = +{T_arr - t0:.0f} s(最慢全速 {tmax:.0f}s + margin {a.margin:.0f})",
          flush=True)

    rows = []
    dt = 1.0 / a.rate
    while time.time() < t0 + a.timeout:
        now = time.time()
        for pl in planes:
            pl.pump()
            if pl.pos is None:
                continue
            d = dist_m(pl.pos[:2], pl.slot)
            t_rem = T_arr - now
            if pl.arrived_t is None and d < a.reach:
                pl.arrived_t = now
                pl.mode = "arrived"
                print(f"[tot] UAV{pl.sysid} 抵達 slot{pl.slot_id} "
                      f"@ +{now - t0:.1f}s (T_arr 偏差 {now - T_arr:+.1f}s)", flush=True)
            if pl.arrived_t is not None:
                pl.goto(pl.slot[0], pl.slot[1], a.talt, a.vmin)   # 到位:低速守 slot
            else:
                v_req = d / max(t_rem, 0.1)
                if v_req > a.vmax:
                    pl.mode = "max"; v = a.vmax
                    aim = pl.slot
                elif v_req >= a.vmin:
                    pl.mode = "straight"; v = v_req
                    aim = pl.slot
                else:                       # 太早 → S 型機動燒時間
                    pl.mode = "weave"; v = a.vmin
                    excess = a.vmin * t_rem - d          # 多餘距離預算(>0)
                    A = max(0.0, min(0.45 * excess, 600.0))  # 側擺幅
                    ph = 2 * math.pi * 0.06 * (now - t0)     # ~16.7s 週期
                    brg_slot = bearing(pl.pos[:2], pl.slot)
                    # 前向只推進 slot 方向一小段,側向大擺 → 慢速逼近的 S
                    fwd = pl.pos[:2]
                    fp = dest(fwd[0], fwd[1], brg_slot, min(d, a.vmin * 4.0))
                    aim = dest(fp[0], fp[1], brg_slot + 90.0, A * math.sin(ph))
                pl.goto(aim[0], aim[1], a.talt, v)
            rows.append((round(now - t0, 1), pl.sysid, pl.pos[0], pl.pos[1],
                         pl.pos[2], round(pl.gs, 1), round(d), pl.mode,
                         round(pl.last_v, 1), pl.slot[0], pl.slot[1]))
        if all(pl.arrived_t for pl in planes):
            print("[tot] 全員抵達扇形 slot。", flush=True)
            break
        time.sleep(dt)

    arr = {pl.sysid: (pl.arrived_t - t0 if pl.arrived_t else None) for pl in planes}
    print(f"[tot] 抵達時刻(s): {arr}", flush=True)
    got = [v for v in arr.values() if v is not None]
    if len(got) >= 2:
        print(f"[tot] ★ 抵達時間散布 = {max(got) - min(got):.1f} s "
              f"({len(got)}/{a.n} 抵達)", flush=True)
    with open(a.out + ".csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "sysid", "lat", "lon", "alt", "gs", "d_slot",
                    "mode", "v_cmd", "slot_lat", "slot_lon"])
        w.writerows(rows)
    # 存扇形/目標幾何供作圖
    with open(a.out + "_geom.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["kind", "lat", "lon"])
        w.writerow(["target", T[0], T[1]])
        for k, s in enumerate(slots):
            w.writerow([f"slot{k}", s[0], s[1]])
    print(f"[tot] CSV: {a.out}.csv + {a.out}_geom.csv", flush=True)


if __name__ == "__main__":
    main()
