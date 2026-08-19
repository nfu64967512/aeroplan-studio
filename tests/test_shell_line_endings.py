# -*- coding: utf-8 -*-
"""WSL 端執行的檔案一律必須是純 LF。

═══ 為什麼要一條測試,而不是「記得就好」═══

⚠️⚠️ 這個坑今天(2026-08-13)踩了**第二次**,而且第二次是在同一個 session 裡
引用過該記憶之後。失效形狀:

    Windows 端用 `pathlib.Path.write_text()` 改 .sh
      ⇒ Python 預設把 `\\n` 譯成 `\\r\\n`
      ⇒ WSL bash 讀到 `set -o pipefail\\r`
      ⇒ 報 `line 40: set: pipefail: invalid option name`

**錯誤訊息指向的行號是錯的**(第 40 行沒有問題,壞的是整個檔),
而且症狀是「tot_node 沒有進入 ready_for_target」—— 看起來像節點問題,
不像編碼問題。RPP 整批三趟因此報廢,零產物。

⇒ 這是 effective-value-contract 那一整類的變種:
   檔案「改成功了」、md5 部署也「一致」,但語意壞了而沒有東西在檢查。

修法:改 .sh 一律用 `write_bytes()`(不做行尾轉換)或 Edit 工具,
      並靠這條測試把關。
"""
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]

#: WSL 端會被 bash / python3 直接執行的目錄。
WSL_DIRS = [ROOT / "scripts" / "wsl", ROOT / "scripts" / "wsl" / "sar_swarm_patches"]

TARGETS = sorted(
    {p for d in WSL_DIRS if d.is_dir()
     for pat in ("*.sh", "*.py") for p in d.glob(pat)}
)


def _ids(p):
    return str(p.relative_to(ROOT)).replace("\\", "/")


@pytest.mark.parametrize("path", TARGETS, ids=_ids)
def test_no_carriage_returns(path):
    """⚠️ 一個 CR 就足以讓整層起不來,所以門檻是零而不是「大部分」。"""
    data = path.read_bytes()
    n = data.count(b"\r")
    assert n == 0, (
        f"{_ids(path)} 含 {n} 個 CR ⇒ WSL bash 會失敗,而且**錯誤訊息會指向錯的行**。\n"
        f"    成因幾乎一定是 Windows 端用 pathlib.write_text() 或 open(..,'w') 改檔。\n"
        f"    修法:用 write_bytes() 或 Edit 工具;修復指令見本檔 docstring。")


def test_there_is_something_to_check():
    """⚠️ 反向鎖:若 glob 因為路徑改動而抓不到檔,上面那條會**全部 skip 而變綠** ——
    一條靜默通過的測試比沒有測試更糟。"""
    assert len(TARGETS) > 20, f"只掃到 {len(TARGETS)} 個檔,glob 可能失效了"


def test_shell_scripts_are_covered():
    """特別確認幾個關鍵驅動腳本在掃描範圍內。"""
    names = {p.name for p in TARGETS}
    for must in ("swarm_ros2_up.sh", "run_west.sh", "rep_interactive.sh",
                 "deploy_build.sh", "stot_trial.sh", "swarm_sitl_up.sh"):
        assert must in names, f"{must} 不在行尾檢查範圍內"
