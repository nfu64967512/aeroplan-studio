# -*- coding: utf-8 -*-
"""Pre-flight check: are HV4SITL15 / HV4SITL16 still byte-identical to the
backup taken by the prep_runner label?

Read-only.  Exits 1 and names every differing file if anything moved.
Run it BEFORE every batch of sorties; the sortie artifacts are overwrite-style
and those two trials are committed-quality paper evidence.
"""
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
OUT = ROOT / "runs/artifacts/PAPERPLAN1/prep_runner"
BACKUP = OUT / "evidence_backup"
TRIALS = ("HV4SITL15", "HV4SITL16")


def tree(p: Path) -> dict:
    return {f.relative_to(p).as_posix():
            (f.stat().st_size, hashlib.sha256(f.read_bytes()).hexdigest())
            for f in sorted(p.rglob("*")) if f.is_file()}


def main() -> int:
    man_p = OUT / "evidence_backup_manifest.json"
    man = json.loads(man_p.read_text(encoding="utf-8")) if man_p.exists() else {}
    bad = []
    for t in TRIALS:
        live_d, back_d = ROOT / "runs/artifacts" / t, BACKUP / t
        if not back_d.exists():
            bad.append(f"{t}: BACKUP MISSING at {back_d}")
            continue
        if not live_d.exists():
            bad.append(f"{t}: LIVE MISSING at {live_d} (restore from the backup)")
            continue
        live, back = tree(live_d), tree(back_d)
        ref = {k: tuple(v) for k, v in (man.get(t, {}).get("sha256") or {}).items()}
        for name in sorted(set(live) | set(back)):
            a, b = live.get(name), back.get(name)
            if a != b:
                bad.append(f"{t}/{name}: live={a} backup={b}")
            elif ref and name in ref and tuple(ref[name]) != a:
                bad.append(f"{t}/{name}: differs from the manifest taken at backup time")
        print(f"{t}: {len(live)} files live / {len(back)} backed up / "
              f"{len(ref)} in manifest -> {'OK' if not bad else 'SEE BELOW'}")
    if bad:
        print("\n*** PAPER EVIDENCE CHANGED — DO NOT FLY UNTIL THIS IS UNDERSTOOD ***")
        for b in bad[:50]:
            print("  ", b)
        if len(bad) > 50:
            print(f"   ... and {len(bad) - 50} more")
        return 1
    print("\nOK: paper evidence intact and identical to the prep_runner backup.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
