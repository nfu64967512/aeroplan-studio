# -*- coding: utf-8 -*-
"""SITL 端對端：飛完（這裡用 LOG_DISARMED=1 讓板子一開機就寫日誌）→ 列表 → 下載最新 .bin → 逐位元組比對。

    python scripts/log_download_e2e.py                 # 起一台本機 ArduPlane.exe（instance 200、埠 7760）
    python scripts/log_download_e2e.py --grow-s 30     # 多等一點讓日誌長大
    python scripts/log_download_e2e.py --out runs/artifacts/LOGDL_E2E_<日期>

閘門（全部要過才 PASS，結果寫到 <out>/summary.json 與 README.md）：
  G1 列表回來 ≥ 1 份，每份有 id / size / time_utc。
  G2 下載最新一份：done 的 size == 列表 size == 檔案大小，sha256 相符。
  G3 下載下來的位元組 == SITL 自己寫在 sitl/vtol/i<N>/logs/<LASTLOG>.BIN 的前 size 個位元組
     （板子那邊是從同一個檔讀出來送的；不相等就是傳輸層丟／錯塊）。
  G4 下載期間遙測沒停（telemetry 信號持續到達：非阻塞狀態機的證據）。
  G5 同一份再下載一次 ⇒ 另存 `_2`，第一份的雜湊不變（永不覆蓋）。
  G6 下載中取消 ⇒ failed('取消…')、沒有 .part 殘留、沒有目的檔。

⚠ 只會殺**自己起的那個** PID，不會 taskkill 全部 ArduPlane.exe（共用機器）。
⚠ 不送 LOG_ERASE、不用 MAVFTP。測的是 mission/sitl_link.py 的真實程式碼，沒有任何替身。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

os.environ.setdefault('AEROPLAN_NO_FILE_LOG', '1')
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from PyQt6.QtCore import QCoreApplication  # noqa: E402

from mission.sitl_link import SITLLink  # noqa: E402

SITL_DIR = ROOT / 'sitl'
BINARY = SITL_DIR / 'ArduPlane.exe'
HOME = (23.451785, 120.286324, 8.0, 0)       # 嘉義亞創中心跑道北端（config/spawn_presets.py）


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def _instance_dir(i: int) -> Path:
    d = SITL_DIR / 'vtol' / f'i{i}'
    d.mkdir(parents=True, exist_ok=True)
    return d


def launch_sitl(instance: int, speedup: float) -> tuple[subprocess.Popen, Path, int]:
    work = _instance_dir(instance)
    for junk in ('eeprom.bin', 'mav.parm'):
        (work / junk).unlink(missing_ok=True)
    parm = work / 'e2e_logdl.parm'
    parm.write_text(
        '# 由 scripts/log_download_e2e.py 產生：一開機就寫日誌，才有東西可下載\n'
        'SERIAL0_PROTOCOL 2\nMAV_SYSID 1\n'
        'SIM_TERRAIN 0\nTERRAIN_ENABLE 0\nSCHED_LOOP_RATE 200\nSIM_RATE_HZ 400\n'
        # ⚠ plane.parm 是 ARMING_REQUIRE 0 ⇒ 開機就是 armed ⇒ 飛控拒絕日誌（第一次實跑就撞到）
        'ARMING_REQUIRE 1\n'
        'LOG_DISARMED 1\nLOG_BITMASK 65535\n', encoding='utf-8')
    defaults = [SITL_DIR / 'default_params' / 'plane.parm',
                SITL_DIR / 'default_params' / 'vtol_default.parm', parm]
    missing = [str(d) for d in defaults if not d.exists()]
    if missing:
        raise SystemExit(f'缺參數檔：{missing}')
    if not BINARY.exists():
        raise SystemExit(f'找不到 {BINARY}')
    port = 5760 + 10 * instance
    cmd = [str(BINARY), '--model', 'quadplane',
           '--home', f'{HOME[0]},{HOME[1]},{HOME[2]},{HOME[3]}',
           '--speedup', str(speedup), '--instance', str(instance), '--sysid', '1',
           '--defaults', ','.join(str(d) for d in defaults)]
    proc = subprocess.Popen(cmd, cwd=str(work), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return proc, work, port


class Collector:
    """把 link 的信號收成清單（主執行緒；QCoreApplication 把跨執行緒信號排進來）。"""

    def __init__(self, link: SITLLink):
        self.telemetry: list[float] = []
        self.lists: list[tuple[int, list]] = []
        self.list_failed: list[tuple[int, str]] = []
        self.progress: list[dict] = []
        self.done: list[dict] = []
        self.failed: list[str] = []
        self.status: list[tuple[int, str]] = []
        link.telemetry.connect(lambda f: self.telemetry.append(time.monotonic()))
        link.log_list.connect(lambda s, e: self.lists.append((s, list(e))))
        link.log_list_failed.connect(lambda s, r: self.list_failed.append((s, r)))
        link.log_download_progress.connect(lambda s, p: self.progress.append(dict(p)))
        link.log_download_done.connect(lambda s, p: self.done.append(dict(p)))
        link.log_download_failed.connect(lambda s, r: self.failed.append(r))
        link.status_text.connect(lambda sev, t: self.status.append((sev, t)))


def pump(app: QCoreApplication, seconds: float):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        app.processEvents()
        time.sleep(0.02)


def wait_until(app: QCoreApplication, pred, timeout: float, what: str) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        app.processEvents()
        if pred():
            return True
        time.sleep(0.02)
    print(f'  ⏱ 等 {what} 逾時（{timeout:.0f} s）')
    return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--instance', type=int, default=200)
    ap.add_argument('--speedup', type=float, default=5.0)
    ap.add_argument('--grow-s', type=float, default=20.0, help='連上後再等幾秒讓日誌長大')
    ap.add_argument('--out', default='')
    ap.add_argument('--keep', action='store_true', help='結束不殺 SITL')
    args = ap.parse_args()

    out = Path(args.out) if args.out else ROOT / 'runs' / 'artifacts' / f'LOGDL_E2E_{time.strftime("%Y-%m-%d_%H%M%S")}'
    out.mkdir(parents=True, exist_ok=True)
    gates: dict[str, dict] = {}
    summary: dict = {'instance': args.instance, 'out': str(out), 'gates': gates}

    proc, work, port = launch_sitl(args.instance, args.speedup)
    summary['sitl'] = {'pid': proc.pid, 'port': port, 'work': str(work)}
    print(f'SITL 起了：pid {proc.pid}、埠 {port}、工作目錄 {work}')
    app = QCoreApplication.instance() or QCoreApplication([])
    link = SITLLink(f'tcp:127.0.0.1:{port}', sysid_label=1, vehicle_hint='VTOL')
    col = Collector(link)
    rc = 1
    try:
        time.sleep(2.0)                        # 讓 exe 把 TCP 埠開起來
        link.start()
        if not wait_until(app, lambda: len(col.telemetry) > 0, 90, '第一筆遙測'):
            gates['G0_connect'] = {'ok': False, 'reason': '連不上 SITL'}
            return 1
        gates['G0_connect'] = {'ok': True}
        print(f'已連線；再等 {args.grow_s:.0f} s 讓日誌長大…')
        pump(app, args.grow_s)

        # ── G1 列表 ───────────────────────────────────────────────
        link.request_log_list()
        ok = wait_until(app, lambda: col.lists or col.list_failed, 60, '日誌清單')
        entries = col.lists[0][1] if col.lists else []
        g1 = ok and bool(entries) and all({'id', 'size', 'time_utc'} <= set(e) for e in entries)
        gates['G1_list'] = {'ok': g1, 'entries': entries, 'failed': col.list_failed}
        print(f'G1 列表：{"OK" if g1 else "FAIL"} {entries[:5]} {col.list_failed}')
        if not g1:
            return 1
        latest = max(entries, key=lambda e: (int(e['id']), int(e.get('time_utc', 0))))
        log_id, size = int(latest['id']), int(latest['size'])

        # ── G2/G4 下載最新 ───────────────────────────────────────
        dest = out / f'e2e_log{log_id}.bin'
        n_tel0 = len(col.telemetry)
        t0 = time.monotonic()
        link.download_log(log_id, str(dest), size)
        ok = wait_until(app, lambda: col.done or col.failed, max(120, size / 2048), '下載完成')
        secs = time.monotonic() - t0
        done = col.done[0] if col.done else None
        g2 = bool(done) and dest.exists() and dest.stat().st_size == size == int(done['size']) \
            and _sha256(dest) == done.get('sha256')
        gates['G2_download'] = {'ok': g2, 'id': log_id, 'size': size, 'seconds': round(secs, 2),
                                'bytes_per_s': round(size / secs, 1) if secs > 0 else None,
                                'done': done, 'failed': list(col.failed),
                                'progress_events': len(col.progress),
                                'refills': (done or {}).get('refills')}
        print(f'G2 下載：{"OK" if g2 else "FAIL"} id={log_id} {size} B {secs:.1f} s '
              f'{size / max(secs, 1e-6) / 1024:.0f} KiB/s refills={(done or {}).get("refills")} {col.failed}')
        tel_during = len(col.telemetry) - n_tel0
        g4 = tel_during > 0 and secs > 0 and (tel_during / secs) > 0.5
        gates['G4_telemetry_alive'] = {'ok': g4, 'frames_during_download': tel_during,
                                       'hz': round(tel_during / secs, 2) if secs > 0 else None}
        print(f'G4 遙測沒停：{"OK" if g4 else "FAIL"} {tel_during} 筆 / {secs:.1f} s')
        if not g2:
            return 1

        # ── G3 與 SITL 自己寫的檔比對 ───────────────────────────
        # ⚠ 第一次實跑學到的兩件事（2026-10-17）：
        #   (a) LOG_ENTRY.id 是「從最舊一份起算的序號」⇒ 檔名 = oldest + id - 1（不是 LASTLOG）。
        #   (b) ArduPilot 一開始傳日誌就把**正在寫的那份關掉、另開新檔**（LASTLOG 2→3）：
        #       關掉前還會再寫入一個 flush 區塊（實測多 8,192 B）⇒ 磁碟檔 ≥ 列表 size，
        #       下載到的必須等於磁碟檔的**前 size 個位元組**（LOG_DISARMED=0 的實飛，落地上鎖後
        #       日誌已關閉、size 就是最終大小，沒有這個差）。
        g3 = False
        info3: dict = {}
        try:
            logs_dir = work / 'logs'
            nums = sorted(int(p.stem) for p in logs_dir.glob('*.BIN') if p.stem.isdigit())
            lastlog = int((logs_dir / 'LASTLOG.TXT').read_text(encoding='utf-8').strip())
            disk = logs_dir / f'{nums[0] + log_id - 1:08d}.BIN'
            on_disk = disk.read_bytes()
            got = dest.read_bytes()
            g3 = (len(on_disk) >= size) and (len(got) == size) and (on_disk[:size] == got)
            first_diff = next((i for i, (a, b) in enumerate(zip(on_disk, got)) if a != b), None)
            info3 = {'disk': str(disk), 'oldest_file': nums[0], 'lastlog': lastlog,
                     'disk_size': len(on_disk), 'listed_size': size,
                     'disk_extra_bytes_after_listing': len(on_disk) - size,
                     'first_diff_offset': first_diff,
                     'rotated_new_log_after_transfer': lastlog > nums[0] + log_id - 1}
        except Exception as e:                      # noqa: BLE001
            info3 = {'error': repr(e)}
        gates['G3_bytes_equal_sitl_file'] = {'ok': g3, **info3}
        print(f'G3 位元組相等：{"OK" if g3 else "FAIL"} {info3}')

        # ── G5 再下載一次：另存、不覆蓋 ────────────────────────
        h1 = _sha256(dest)
        col.done.clear(); col.failed.clear()
        link.download_log(log_id, str(dest), size)
        wait_until(app, lambda: col.done or col.failed, max(120, size / 2048), '第二次下載')
        d2 = col.done[0] if col.done else None
        p2 = Path(d2['path']) if d2 else None
        g5 = bool(d2) and p2 is not None and p2.exists() and p2.resolve() != dest.resolve() \
            and _sha256(dest) == h1 and p2.name.startswith('e2e_log') and '_2' in p2.name
        gates['G5_never_overwrite'] = {'ok': g5, 'first': str(dest), 'second': str(p2),
                                       'first_hash_unchanged': _sha256(dest) == h1, 'failed': list(col.failed)}
        print(f'G5 不覆蓋：{"OK" if g5 else "FAIL"} 第二份={p2}')

        # ── G6 下載中取消 ───────────────────────────────────────
        col.done.clear(); col.failed.clear(); col.progress.clear()
        dest3 = out / f'e2e_cancel{log_id}.bin'
        link.download_log(log_id, str(dest3), size)
        wait_until(app, lambda: len(col.progress) >= 2 or col.done or col.failed, 30, '第一批進度')
        link.cancel_log_download()
        wait_until(app, lambda: col.done or col.failed, 30, '取消回報')
        part = Path(str(dest3) + '.part')
        pump(app, 1.0)
        g6 = bool(col.failed) and ('取消' in col.failed[0]) and not col.done \
            and not dest3.exists() and not part.exists()
        gates['G6_cancel'] = {'ok': g6, 'failed': list(col.failed), 'done': list(col.done),
                              'dest_exists': dest3.exists(), 'part_exists': part.exists()}
        print(f'G6 取消：{"OK" if g6 else "FAIL"} {col.failed[:1]}')

        rc = 0 if all(g['ok'] for g in gates.values()) else 1
        return rc
    finally:
        try:
            link.stop()
            link.wait(3000)
        except Exception:
            pass
        if not args.keep:
            try:
                proc.kill()
            except Exception:
                pass
        summary['status_text'] = col.status[-20:]
        summary['result'] = 'PASS' if rc == 0 else 'FAIL'
        (out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2, default=str),
                                          encoding='utf-8')
        lines = [f'# 日誌下載端對端 {summary["result"]}', '', f'SITL instance {args.instance}、埠 {port}、'
                 f'speedup {args.speedup}；產物在 {out}', '']
        for k, g in gates.items():
            lines.append(f'- {k}: {"✅" if g.get("ok") else "❌"} ' +
                         '、'.join(f'{a}={b}' for a, b in g.items() if a not in ('ok', 'entries', 'done')))
        (out / 'README.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
        print(f'→ {summary["result"]}；{out / "summary.json"}')


if __name__ == '__main__':
    sys.exit(main())
