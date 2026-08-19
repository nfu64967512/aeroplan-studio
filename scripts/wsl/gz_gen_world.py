#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gz_gen_world.py — 產生 N 機的 Gazebo 世界與逐機模型(展示用路徑)。

⚠️ 定位:**Gazebo 只做表演,量測留在 SITL**(使用者 2026-08-11 定調)。
   所以本腳本刻意**不碰** swarm_sitl_up.sh / swarm_ros2_up.sh ——
   那條是已驗證的量測平台,必須逐位元穩定。

## 為什麼需要產生器

`ArduPilotPlugin` 的 `fdm_port_in` 是**寫死在模型檔裡**的(原廠 9002),
而 SDF 的 `<include>` 不能覆寫巢狀 plugin 的參數
⇒ 多機只能**逐機複製模型目錄再改埠**。這就是本腳本做的事。

SITL 那一端用 `--sim-port-out` 對上(`SITL_cmdline.cpp:365` 有這個選項);
`swarm_sitl_up.sh` 本來就直接叫 `arduplane` 而非 sim_vehicle.py,所以給得進去。

## 埠位

    UAV i (1-based)  fdm_port_in = 9002 + 10*(i-1)

⚠️ 與既有埠位表不衝突:SITL master 5760+10i、直連 5762+10i、MAVROS 5860+10i、
   AeroPlan 6760+10i、tot_node 6860+10i、編隊 WP 6960+10i。9002 起是新的一段。

## 隊形

與 `swarm_sitl_up.sh:360` 相同:沿跑道航向的**反方向**排成縱列,間距 SPACING。
    bearing = hdg + 180;  east = d·sin(bearing),north = d·cos(bearing)
"""
from __future__ import annotations

import argparse
import math
import os
import re
import shutil
import sys
from pathlib import Path

FDM_PORT_BASE = 9002
FDM_PORT_STRIDE = 10

TEMPLATE_MODEL = "zephyr_with_ardupilot"


def read_swarm_state(path: Path):
    """讀 swarm_sitl_up.sh 產生的 swarm_state.env,讓兩邊原點一致。"""
    if not path.exists():
        return None
    txt = path.read_text(encoding="utf-8", errors="replace")
    m = re.search(r'SWARM_LOC="([^"]+)"', txt)
    if not m:
        return None
    parts = [p.strip() for p in m.group(1).split(",")]
    if len(parts) < 4:
        return None
    return tuple(float(p) for p in parts[:4])   # lat, lon, alt, hdg


def make_model(src_dir: Path, dst_dir: Path, name: str, port: int) -> None:
    if dst_dir.exists():
        shutil.rmtree(dst_dir)
    shutil.copytree(src_dir, dst_dir)

    sdf = dst_dir / "model.sdf"
    txt = sdf.read_text(encoding="utf-8")

    new, n_name = re.subn(r'<model name="%s">' % re.escape(TEMPLATE_MODEL),
                          '<model name="%s">' % name, txt, count=1)
    if n_name != 1:
        raise SystemExit("✗ %s 找不到 <model name=\"%s\">" % (sdf, TEMPLATE_MODEL))

    new, n_port = re.subn(r"<fdm_port_in>\s*\d+\s*</fdm_port_in>",
                          "<fdm_port_in>%d</fdm_port_in>" % port, new)
    if n_port != 1:
        raise SystemExit(
            "✗ %s 的 <fdm_port_in> 出現 %d 次(預期 1)—— 模板版面變了" % (sdf, n_port))

    sdf.write_text(new, encoding="utf-8", newline="\n")

    cfg = dst_dir / "model.config"
    if cfg.exists():
        c = cfg.read_text(encoding="utf-8")
        c = re.sub(r"<name>[^<]*</name>", "<name>%s</name>" % name, c, count=1)
        cfg.write_text(c, encoding="utf-8", newline="\n")


def build_world(base_world: Path, n: int, spacing: float, hdg: float,
                model_names: list) -> str:
    txt = base_world.read_text(encoding="utf-8")

    # 拿掉模板裡的佔位機體,換成 N 架
    txt, n_cut = re.subn(
        r"\n[ \t]*<include>\s*<uri>model://%s</uri>.*?</include>\n"
        % re.escape(TEMPLATE_MODEL),
        "\n", txt, flags=re.S)
    if n_cut != 1:
        raise SystemExit(
            "✗ 基礎世界檔裡的佔位機體 <include> 找到 %d 個(預期 1)" % n_cut)

    bearing = math.radians(hdg + 180.0)
    blocks = []
    for i, name in enumerate(model_names):
        d = i * spacing
        east = d * math.sin(bearing)
        north = d * math.cos(bearing)
        # roll −90 / yaw 是模板的座標慣例,見 taoyuan_runway.sdf 的說明
        blocks.append(
            "    <include>\n"
            "      <uri>model://%s</uri>\n"
            "      <name>%s</name>\n"
            "      <pose degrees=\"true\">%.3f %.3f 0.422 -90 0 -145</pose>\n"
            "    </include>\n" % (name, name, east, north))

    marker = "  </world>"
    if marker not in txt:
        raise SystemExit("✗ 基礎世界檔找不到 </world>")
    return txt.replace(marker, "".join(blocks) + marker, 1)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-n", "--count", type=int, default=2)
    ap.add_argument("--spacing", type=float, default=30.0,
                    help="縱列間距 m(對齊 swarm_sitl_up.sh 的 SPACING 預設 30)")
    ap.add_argument("--gazebo-src", default=str(Path.home() / "ardupilot_gazebo"))
    ap.add_argument("--out", default=str(Path.home() / "fixwing_ws" / "gz_show"),
                    help="產生的模型與世界檔放這裡(不寫進 repo)")
    ap.add_argument("--base-world", default=None)
    args = ap.parse_args()

    if args.count < 1:
        print("✗ N 至少 1", file=sys.stderr)
        return 1

    here = Path(__file__).resolve().parent
    base_world = Path(args.base_world) if args.base_world \
        else here / "gazebo" / "taoyuan_runway.sdf"
    src_model = Path(args.gazebo_src) / "models" / TEMPLATE_MODEL
    if not src_model.exists():
        print("✗ 找不到模板模型 %s" % src_model, file=sys.stderr)
        return 1
    if not base_world.exists():
        print("✗ 找不到基礎世界檔 %s" % base_world, file=sys.stderr)
        return 1

    out = Path(args.out)
    (out / "models").mkdir(parents=True, exist_ok=True)

    # 原點以 swarm_state.env 為準,對不上就用世界檔自己的值
    state = read_swarm_state(Path.home() / "fixwing_ws" / "sitl_run" / "swarm_state.env")
    hdg = state[3] if state else 52.0
    if state:
        print("[gen] 原點取自 swarm_state.env:lat=%.6f lon=%.6f alt=%.0f hdg=%.0f"
              % state)
    else:
        print("[gen] ⚠️ 沒有 swarm_state.env,航向用預設 %.0f°" % hdg)

    names = []
    for i in range(1, args.count + 1):
        name = "uav%d" % i
        port = FDM_PORT_BASE + FDM_PORT_STRIDE * (i - 1)
        make_model(src_model, out / "models" / name, name, port)
        names.append(name)
        print("[gen] %-6s fdm_port_in=%d" % (name, port))

    world_txt = build_world(base_world, args.count, args.spacing, hdg, names)
    world_path = out / "taoyuan_show.sdf"
    world_path.write_text(world_txt, encoding="utf-8", newline="\n")
    print("[gen] 世界檔 → %s" % world_path)

    print()
    print("[gen] 起 Gazebo:")
    print("    GZ_EXTRA_RESOURCE=%s/models WORLD=%s bash gz_up.sh" % (out, world_path))
    print("[gen] 起 SITL(逐機,i 從 0 起):")
    print("    arduplane --model JSON --sim-port-out $((%d + %d*i)) -I$i --sysid $((i+1)) \\"
          % (FDM_PORT_BASE, FDM_PORT_STRIDE))
    print("        --home <lat>,<lon>,<alt>,<hdg>")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
