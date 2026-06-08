"""cli/demo_cmd.py — `aeroplan demo` 子指令。

對齊 ADOS `cli/commands/demo.ts`：一鍵 spawn 5 機 demo fleet，
home 在 Bangalore（12.9716, 77.5946），sysid 1..N、ports 14550 + 10·i。

可加 --gui 同時開啟 milstd shell（透過 os.execv 重啟自身）。
"""
from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import time
from typing import List

from mission.sitl_launcher import SITLLauncher


logger = logging.getLogger(__name__)

# Demo Fleet 固定參數
_DEMO_LAT = 12.9716
_DEMO_LON = 77.5946


def add_subparser(subparsers: argparse._SubParsersAction) -> argparse.ArgumentParser:
    """註冊 `demo` 子指令到主 argparse。"""
    p: argparse.ArgumentParser = subparsers.add_parser(
        "demo",
        help="Spawn a fleet of SITL drones for demo purposes (ADOS-equivalent).",
    )
    p.add_argument("--drones", type=int, choices=[1, 3, 5, 10], default=5)
    p.add_argument("--vehicle", choices=["COPTER", "PLANE", "VTOL"], default="COPTER")
    p.add_argument("--gui", action="store_true", help="Also launch milstd GUI pre-connected.")
    p.set_defaults(func=run)
    return p


def run(args: argparse.Namespace) -> int:
    """CLI 進入點。"""
    if not SITLLauncher.is_available():
        print("[ERROR] SITL binaries not found.", file=sys.stderr)
        return 2

    drones = max(1, min(10, int(args.drones)))
    # 0-8 去重：instance 預設設定統一由 SITLLauncher 產生
    instance_configs: List[dict] = SITLLauncher.default_instance_configs(drones)

    print(f"[DEMO] vehicle={args.vehicle}  drones={drones}  "
          f"home=({_DEMO_LAT}, {_DEMO_LON})  ports={[c['embedded_port'] for c in instance_configs]}")

    launcher = SITLLauncher()
    try:
        results = launcher.start_multi(
            vehicle=args.vehicle,
            count=drones,
            lat=_DEMO_LAT,
            lon=_DEMO_LON,
            instance_configs=instance_configs,
        )
    except Exception as exc:
        print(f"[ERROR] failed to start demo SITL: {exc}", file=sys.stderr)
        return 3

    conn_strs: List[str] = []
    for inst, conn in results:
        print(f"[DEMO] i{inst}  sys={inst + 1}  {conn}")
        conn_strs.append(conn)

    if args.gui:
        # 重啟自身為 GUI 模式（milstd shell），並把 connstr 透過環境變數帶入
        os.environ["AEROPLAN_AUTOCONNECT"] = ",".join(conn_strs)
        python = sys.executable
        # 不停 SITL，子行程僅取代當前進程（os.execv 接管）
        argv = [python, sys.argv[0], "--shell", "milstd"]
        print(f"[DEMO] exec → {' '.join(argv)}")
        os.execv(python, argv)
        return 0  # unreachable

    # 否則 Ctrl+C 阻塞
    print("[DEMO] Press Ctrl+C to stop all instances.")
    stop_requested = {"flag": False}

    def _handle_sigint(signum, frame):
        stop_requested["flag"] = True

    signal.signal(signal.SIGINT, _handle_sigint)
    try:
        while not stop_requested["flag"] and launcher.is_running():
            time.sleep(0.5)
    finally:
        print("[DEMO] stopping …")
        launcher.stop()
    return 0


__all__ = ["add_subparser", "run"]
