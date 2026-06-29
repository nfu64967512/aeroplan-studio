# -*- coding: utf-8 -*-
"""比對 git HEAD 與工作區，列出每個檔案被移除的「專案內部模組」import。

輸出 JSON: [{"file": ..., "module": ..., "name": ...}, ...]
僅列出來源為專案套件 (config/core/mission/...) 的移除項——
stdlib / 第三方 (typing, PyQt6, numpy...) 的移除不可能破壞其他檔案。
"""
import ast
import json
import subprocess
import sys

PROJECT_PKGS = ("config", "core", "mission", "sensors", "sitl",
                "ui", "utils", "cli", "scripts", "data", "main")


def import_pairs(src: str):
    """回傳 {(module, name)}；name='*' 代表整模組 import"""
    pairs = set()
    try:
        tree = ast.parse(src)
    except SyntaxError:
        return pairs
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module:
            for a in node.names:
                # bound = 該檔案命名空間中實際綁定的名稱（re-export 檢查用）
                pairs.add((node.module, a.name, a.asname or a.name))
        elif isinstance(node, ast.Import):
            for a in node.names:
                pairs.add((a.name, "*", a.asname or a.name))
    return pairs


def main():
    files = subprocess.run(
        ["git", "diff", "--name-only", "HEAD", "--", "*.py"],
        capture_output=True, text=True, encoding="utf-8",
    ).stdout.split()
    removed = []
    for f in files:
        try:
            old = subprocess.run(
                ["git", "show", f"HEAD:{f}"],
                capture_output=True, encoding="utf-8",
            ).stdout
            new = open(f, encoding="utf-8").read()
        except (OSError, UnicodeDecodeError):
            continue
        for mod, name, bound in sorted(import_pairs(old) - import_pairs(new)):
            if mod.split(".")[0] in PROJECT_PKGS:
                removed.append({"file": f, "module": mod,
                                "name": name, "bound": bound})
    json.dump(removed, sys.stdout, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
