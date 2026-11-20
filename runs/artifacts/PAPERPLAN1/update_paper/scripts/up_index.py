# -*- coding: utf-8 -*-
"""label update_paper / stage 8 -- INDEX.json of this label's own products.

Same discipline as analysis/INDEX.json: every non-regenerable product gets a
size + sha256 so a third party can tell whether what they are reading is what
was produced.  cache/*.npz are excluded: they are pure derivatives of the .BIN
files and are already excluded by .gitignore's `*.npz` rule.
"""
from __future__ import annotations

import hashlib
from pathlib import Path

from up_lib import jdump

HERE = Path(__file__).resolve().parent.parent
ROOT = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
SKIP_DIRS = {"cache", "__pycache__"}


def main() -> None:
    files = {}
    for p in sorted(HERE.rglob("*")):
        if not p.is_file() or p.name == "INDEX.json":
            continue
        if SKIP_DIRS & set(p.relative_to(HERE).parts):
            continue
        files[p.relative_to(ROOT).as_posix()] = {
            "bytes": p.stat().st_size,
            "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
    jdump(HERE / "INDEX.json", {
        "label": "update_paper",
        "note": "cache/*.npz are excluded: regenerable from the .BIN files with "
                "scripts/up_extract.py, and already matched by .gitignore's `*.npz`.",
        "reproduce": ["python scripts/up_extract.py",
                      "python scripts/up_metrics.py",
                      "python scripts/up_wind.py",
                      "python scripts/up_spawn.py",
                      "python scripts/up_corrections.py",
                      "python scripts/up_numbers.py",
                      "python scripts/up_corrected_copies.py",
                      "python scripts/up_index.py"],
        "n_files": len(files),
        "total_bytes": sum(v["bytes"] for v in files.values()),
        "files": files})
    print(f"{len(files)} files, {sum(v['bytes'] for v in files.values())} bytes")


if __name__ == "__main__":
    main()
