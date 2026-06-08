"""cli/sitl_cmd.py — `aeroplan sitl` 子指令。

對齊 ADOS `cli/commands/sitl.ts`：spawn ArduPilot SITL，
支援 --drones / --vehicle / --lat / --lon / --speedup / --wind / --fanout / --firewall。

無 GUI；阻塞於 SIGINT 時呼叫 SITLLauncher.stop()。
"""
from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from typing import List

from mission.sitl_launcher import SITLLauncher


logger = logging.getLogger(__name__)


def add_subparser(subparsers: argparse._SubParsersAction) -> argparse.ArgumentParser:
    """註冊 `sitl` 子指令到主 argparse。"""
    p: argparse.ArgumentParser = subparsers.add_parser(
        "sitl",
        help="Spawn ArduPilot SITL instance(s) headless (no GUI).",
    )
    p.add_argument("--vehicle", choices=["COPTER", "PLANE", "VTOL", "ROVER"], default="COPTER")
    p.add_argument("--drones", type=int, default=None, help="Number of instances (1-16). Interactive prompt if omitted on a TTY.")
    p.add_argument("--lat", type=float, default=23.7)
    p.add_argument("--lon", type=float, default=121.0)
    p.add_argument("--alt", type=float, default=100.0)
    p.add_argument("--heading", type=float, default=0.0)
    p.add_argument("--speedup", type=int, choices=[1, 2, 5, 10], default=1)
    p.add_argument("--wind", type=str, default=None, help="speed,dir (m/s,deg)")
    p.add_argument("--no-fanout", action="store_true", help="Disable embedded fan-out.")
    p.add_argument("--firewall", action="store_true", help="Auto add Windows Firewall rule.")
    p.add_argument("--no-dashboard", action="store_true", help="Suppress headless dashboard output.")
    p.add_argument("--print-connstr", action="store_true", help="Print comma-separated connection strings and exit.")
    p.set_defaults(func=run)
    return p


def _prompt_drones() -> int:
    """互動式提示：1 / 2 / 3 / 5（對齊 ADOS）。"""
    print("[SITL] How many drones? (1 / 2 / 3 / 5) [1]: ", end="", flush=True)
    try:
        line = sys.stdin.readline().strip()
    except KeyboardInterrupt:
        sys.exit(130)
    if not line:
        return 1
    try:
        n = int(line)
    except ValueError:
        return 1
    return max(1, min(16, n))


def run(args: argparse.Namespace) -> int:
    """CLI 進入點。"""
    if not SITLLauncher.is_available():
        print("[ERROR] SITL binaries not found. Please install ArduPilot SITL.", file=sys.stderr)
        return 2

    drones = args.drones
    if drones is None:
        if sys.stdin.isatty():
            drones = _prompt_drones()
        else:
            drones = 1
    drones = max(1, min(16, int(drones)))

    instance_configs: List[dict] = []
    fanout = not args.no_fanout
    if fanout:
        # 0-8 去重：instance 預設設定統一由 SITLLauncher 產生
        instance_configs = SITLLauncher.default_instance_configs(drones)

    launcher = SITLLauncher()
    if not args.no_dashboard:
        print(f"[SITL] vehicle={args.vehicle}  drones={drones}  "
              f"home=({args.lat}, {args.lon})  alt={args.alt}m  "
              f"heading={args.heading}°  speedup={args.speedup}x")
        if args.wind:
            print(f"[SITL] wind={args.wind}")
        if fanout:
            print(f"[SITL] fan-out enabled — ports {', '.join(str(c['embedded_port']) for c in instance_configs)}")

    try:
        results = launcher.start_multi(
            vehicle=args.vehicle,
            count=drones,
            lat=args.lat,
            lon=args.lon,
            alt=args.alt,
            heading=args.heading,
            instance_configs=instance_configs if fanout else None,
            auto_firewall=bool(args.firewall),
        )
    except Exception as exc:
        print(f"[ERROR] failed to start SITL: {exc}", file=sys.stderr)
        return 3

    conn_strs: List[str] = []
    for inst, conn in results:
        print(f"[SITL] i{inst}  sys={inst + 1}  {conn}")
        conn_strs.append(conn)

    if args.print_connstr:
        # 印一行 comma-joined 後立刻離開（不阻塞）
        print(",".join(conn_strs))
        return 0

    # Block 直到 Ctrl+C
    print("[SITL] Press Ctrl+C to stop all instances.")
    stop_requested = {"flag": False}

    def _handle_sigint(signum, frame):
        stop_requested["flag"] = True

    signal.signal(signal.SIGINT, _handle_sigint)
    try:
        while not stop_requested["flag"] and launcher.is_running():
            time.sleep(0.5)
    finally:
        print("[SITL] stopping all instances …")
        launcher.stop()
    return 0


__all__ = ["add_subparser", "run"]
