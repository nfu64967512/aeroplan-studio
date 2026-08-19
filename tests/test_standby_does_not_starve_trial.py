# -*- coding: utf-8 -*-
"""待命 tot_node 不可以餓死批次航次的 tot_node。

═══ 這條迴歸鎖擋的是什麼 ═══
2026-08-05 起 `swarm_ros2_up.sh` 會一併拉起一個 **STANDBY tot_node**,給
「AeroPlan 地圖右鍵指定紅心」那條互動路徑用。但 `stot_trial.sh` 也會呼叫
`swarm_ros2_up.sh`(為了 MAVROS + master/wingman/backup),然後**自己**再啟動
一個航次用的 tot_node。

兩個都起,就會撞上分流埠的**先連者得**:
MAVProxy 的 `--out tcpin` 只餵**第一個**連上的 client。

實測(KIL1,2026-08-06):
    14:44:14  待命 tot_node 啟動  → 連上 6960/6970,成為第一個 client
    14:44:30  「STANDBY 待命就緒」
    14:44:55  航次 tot_node 啟動  → 第二個 client
  ⇒ 航次節點每 30 s 印「⚠️ 等機體定位中:0/2 台有 GLOBAL_POSITION_INT,缺 [1, 2]」
    整趟逾時,tot_KIL1.csv 只有表頭 70 bytes。
  ⇒ `ss -tnp` 當場看到:
        CLOSE-WAIT 29851  127.0.0.1:6960   ← 被殺掉的待命節點,29851 bytes 沒人讀
        CLOSE-WAIT 29851  127.0.0.1:6970
    `pkill -x tot_node` 殺得掉行程,**殺不掉那條 socket 佔著的第一 client 名額**。

⚠️ 這是靜默失敗:SITL 正常、MAVROS 正常、log 沒有任何 ERROR 指向根因。
   修法是在批次路徑用 TOT_ENABLE=0 讓待命節點根本不要起。
"""
import pathlib
import re

ROOT = pathlib.Path(__file__).resolve().parents[1]
TRIAL = (ROOT / "scripts" / "wsl" / "stot_trial.sh").read_text(encoding="utf-8")
UP = (ROOT / "scripts" / "wsl" / "swarm_ros2_up.sh").read_text(encoding="utf-8")


def _code_only(src):
    """去掉註解再掃 —— 否則說明文字裡的 TOT_ENABLE 會造成假通過。"""
    return "\n".join(ln.split("#", 1)[0] for ln in src.splitlines())


class TestBatchPathDisablesStandby:
    def test_trial_passes_tot_enable_zero(self):
        """批次航次呼叫 swarm_ros2_up.sh 時必須帶 TOT_ENABLE=0。"""
        code = _code_only(TRIAL)
        assert re.search(r"TOT_ENABLE=0", code), \
            "stot_trial.sh 沒有把 TOT_ENABLE=0 傳給 swarm_ros2_up.sh"

    def test_tot_enable_is_on_the_same_invocation(self):
        """必須綁在**那一次呼叫**上,不能只是檔案裡某處有這個字串。

        bash 的前綴賦值只對緊接著的那個命令生效;寫在別的地方等於沒寫。
        """
        code = _code_only(TRIAL)
        m = re.search(r'TOT_ENABLE=0\s*\\?\s*\n?\s*bash\s+"\$SR/swarm_ros2_up\.sh"', code)
        assert m, "TOT_ENABLE=0 必須緊接在 `bash $SR/swarm_ros2_up.sh` 之前"

    def test_up_script_honours_the_switch(self):
        """swarm_ros2_up.sh 這一端要真的讀得懂 TOT_ENABLE=0。"""
        assert 'TOT_ENABLE="${TOT_ENABLE:-1}"' in UP
        code = _code_only(UP)
        # 0 要能擋掉啟動:必須有一個以 TOT_ENABLE 為條件的分支
        assert re.search(r'\[\s*"?\$\{?TOT_ENABLE\}?"?\s*(=|-eq)\s*"?1"?\s*\]', code) \
            or re.search(r'\[\s*"?\$\{?TOT_ENABLE\}?"?\s*(!=|-ne)\s*"?0"?\s*\]', code), \
            "swarm_ros2_up.sh 沒有用 TOT_ENABLE 當守門條件"

    def test_trial_still_starts_its_own_tot_node(self):
        """關掉待命節點之後,批次路徑仍必須自己起一個 —— 否則就從餓死變成沒有。"""
        code = _code_only(TRIAL)
        assert re.search(r"ros2\s+run\s+sar_swarm\s+tot_node", code), \
            "stot_trial.sh 應該自己啟動航次用的 tot_node"
