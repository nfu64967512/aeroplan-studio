#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mgs_record.py — 獨立的 NDJSON 錄製 client(不需要 MATLAB)。

連到 `mgs_bridge_node.py`,把收到的每一則訊息**原樣**寫成一行 JSON。
`mgs.parseStream` / `mgs.shadowDiff` 直接吃這個檔。

為什麼不用 `mgs.recordStream`(MATLAB 端):
  一趟完整航次是好幾分鐘,MATLAB 端錄會把整個 session 卡住;而且 MCP 的
  eval 是同步的,長時間阻塞會逾時。這支放背景跑,錄完再讓 MATLAB 讀檔。
  順帶證明這個介面是**第三方可接的**,不是只有 MATLAB 進得來。

⚠️ **先開錄再開打。** 錄製只收它啟動之後到達的訊息,而一趟航次不能重播。

用法:
    python3 matlab/mgs/bridge/mgs_record.py --out /tmp/base.ndjson --seconds 1200

停止條件(任一):
    --seconds 到期
    tot_status 的 phase 變成 DONE(且已經看過非 STANDBY 的相位)
    Ctrl-C
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import time

SCHEMA = 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5799)
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=1800.0)
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()

    s = socket.create_connection((a.host, a.port), timeout=10)
    s.settimeout(1.0)
    s.sendall((json.dumps({"type": "hello", "schema": SCHEMA, "t": time.time(),
                           "peer": "mgs_record", "mode": "shadow"}) + "\n").encode())

    fh = open(a.out, "w", encoding="utf-8", newline="\n")
    fh.write(json.dumps({"type": "_meta", "t": time.time(), "schema": SCHEMA,
                         "recorder": "mgs_record.py"}, ensure_ascii=False) + "\n")

    t0 = time.time()
    buf = b""
    n = {"total": 0, "state": 0, "status": 0, "gate": 0}
    phases_seen = set()
    last_report = 0.0
    saw_flight = False

    try:
        while time.time() - t0 < a.seconds:
            try:
                chunk = s.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue
                fh.write(line.decode("utf-8", "replace") + "\n")
                n["total"] += 1
                try:
                    m = json.loads(line)
                except Exception:
                    continue
                k = m.get("type")
                if k in n:
                    n[k] += 1
                if k == "status":
                    ph = (m.get("data") or {}).get("phase")
                    if ph:
                        phases_seen.add(ph)
                        if ph not in ("STANDBY", "INIT"):
                            saw_flight = True
                        if ph == "DONE" and saw_flight:
                            fh.flush()
                            if not a.quiet:
                                sys.stdout.write(
                                    "phase DONE after %.0f s -- stopping\n"
                                    % (time.time() - t0))
                            raise SystemExit(0)
            fh.flush()

            if not a.quiet and time.time() - last_report > 15:
                last_report = time.time()
                sys.stdout.write(
                    "%6.0f s | %7d msgs | state %6d | phases %s\n"
                    % (time.time() - t0, n["total"], n["state"],
                       ",".join(sorted(phases_seen)) or "-"))
                sys.stdout.flush()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        fh.flush()
        fh.close()
        try:
            s.close()
        except OSError:
            pass

    sys.stdout.write("recorded %d messages (%d state) to %s over %.0f s\n"
                     % (n["total"], n["state"], a.out, time.time() - t0))
    if n["state"] == 0:
        sys.stdout.write("!! NOT ONE per-UAV state message -- the recording is "
                         "useless for a shadow diff.  Check the bridge log.\n")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
