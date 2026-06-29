# -*- coding: utf-8 -*-
"""煙霧測試：逐一匯入專案內所有模組，回報 ImportError / 例外。

用法: python scripts/_smoke_import_all.py
輸出: 每行 "OK <module>" 或 "FAIL <module> :: <錯誤摘要>"
"""
import importlib
import os
import sys
import traceback

# 避免 GUI 視窗彈出
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

SCAN_DIRS = ["config", "core", "mission", "sensors", "sitl", "ui", "utils", "cli", "data", "scripts"]
ROOT_MODULES = ["main", "download_huwei_dem", "download_taiwan_dem"]

# 已知具外部相依/重副作用、不適合直接 import 的檔案（ArduPilot 工具）
SKIP = {
    "sitl.sim_vehicle",          # ArduPilot 啟動器，頂層即執行
    "scripts._smoke_import_all", # 本腳本自身
}

def collect_modules():
    mods = []
    for d in SCAN_DIRS:
        base = os.path.join(ROOT, d)
        if not os.path.isdir(base):
            continue
        for dirpath, dirnames, filenames in os.walk(base):
            dirnames[:] = [x for x in dirnames if x != "__pycache__"]
            for fn in sorted(filenames):
                if not fn.endswith(".py"):
                    continue
                rel = os.path.relpath(os.path.join(dirpath, fn), ROOT)
                mod = rel[:-3].replace(os.sep, ".")
                if mod.endswith(".__init__"):
                    mod = mod[: -len(".__init__")]
                mods.append(mod)
    mods.extend(ROOT_MODULES)
    return mods

def main():
    ok, fail, skipped = 0, [], 0
    for mod in collect_modules():
        if mod in SKIP:
            print(f"SKIP {mod}")
            skipped += 1
            continue
        try:
            importlib.import_module(mod)
            print(f"OK   {mod}")
            ok += 1
        except BaseException as e:  # noqa: BLE001 — 需攔截 SystemExit 等
            tb = traceback.format_exc().strip().splitlines()
            summary = f"{type(e).__name__}: {e}"
            print(f"FAIL {mod} :: {summary}")
            # 印出最後 6 行 traceback 方便定位
            for line in tb[-6:]:
                print(f"     | {line}")
            fail.append(mod)
    print()
    print(f"RESULT ok={ok} fail={len(fail)} skip={skipped}")
    if fail:
        print("FAILED_MODULES: " + ", ".join(fail))
    sys.exit(1 if fail else 0)

if __name__ == "__main__":
    main()
