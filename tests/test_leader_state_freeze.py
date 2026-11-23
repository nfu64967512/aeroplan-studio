# -*- coding: utf-8 -*-
"""長機狀態「凍結而不中斷」—— 容錯鏈防的是斷線,這次是說謊。

═══ 2026-08-07 ANM1 事故 ═══
UAV1 的 `mavros_node` 在飛行中 SIGABRT 死亡:
    /tmp/mavros_uav1.log:151  std::future_error: Promise already satisfied
    /tmp/mavros_uav1.log:153  process has died [exit code -6, __ns:=/uav1/mavros]

而 `master_role.MasterPublisher` 對定位快取 `_fix` **沒有任何年齡檢查**
(`if self._fix is None` 只擋「從沒收過第一筆」,擋不住「收過但停了」),
於是長機位姿凍成 ghost,仍以 10 Hz 廣播 **448 秒**。

更惡劣的是 whoLeader 的 `speed` / `omega` 兩欄不經 MAVROS(來自 master_node 內部),
全程還在變 ⇒ **訊息看起來是活的**,連「內容有沒有變」的粗檢查都騙得過。

僚機端 `on_wholeader` **無條件**刷新 `_leader_t`,量的是「訊息有沒有到」而不是
「內容有沒有更新」⇒ `age ≈ 0.1 s` ⇒ `_leader_estimate` 的 5 s(啟動 DR)與
45 s(放棄外推)兩道告警**結構上永遠進不去**。

後果:僚機忠實地飛向一個永遠不動的 slot,飛到 **68 km** 外,
整趟飛行的 WARN/ERROR 計數 = **0**,然後航次逾時。

⭐ 最諷刺的一點:**停發是有告警的,凍結發布沒有。**
   如果長機乾脆停止廣播,僚機 5 秒內就會出聲。
"""
import pathlib
import re

import pytest

P = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "wsl" / "sar_swarm_patches"
ROLE = (P / "master_role.py").read_text(encoding="utf-8")
WING = (P / "wingman_node.py").read_text(encoding="utf-8")


def stale(age, max_age):
    """發布端閘門的純述詞。max_age <= 0 = 關閉(回到事故前行為)。"""
    return max_age > 0.0 and age > max_age


def effective_age(msg_age, content_age, watchdog=True):
    """消費端:取兩者的大者,讓既有的 5s/45s 告警對「說謊」也生效。"""
    return max(msg_age, content_age) if watchdog else msg_age


class TestPublisherGate:
    def test_fresh_fix_publishes(self):
        assert not stale(0.1, 1.5)

    def test_stale_fix_stops_publishing(self):
        """ANM1 的凍結持續 448 s,遠超任何合理門檻。"""
        assert stale(448.0, 1.5)

    def test_disabled_by_env(self):
        """FIX_MAX_AGE=0 必須完全回到事故前行為(回退開關)。"""
        assert not stale(448.0, 0.0)

    def test_first_fix_never_gates(self):
        """`_fix is None` 的路徑先擋掉,年齡閘門不該影響「還沒收到第一筆」。"""
        assert "if self._fix is None:" in ROLE

    def test_role_tracks_fix_time(self):
        assert "self._fix_t" in ROLE
        assert re.search(r"self\._fix_t\s*=\s*self\.node\.get_clock\(\)", ROLE)

    def test_role_has_env_fallback(self):
        assert 'os.environ.get("FIX_MAX_AGE"' in ROLE

    def test_threshold_aligned_with_consumer_dr_hold(self):
        """⚠️ 發布端的門檻不可以比消費端自己的容忍度(`dr_hold_s`)還嚴。

        NAG1(2026-08-07)實測:設 1.5 s ⇒ 巡航 150 s 內誤觸發 3 次,
        而 MAVROS 兩台都活著 —— 那是傳遞抖動。後果不是中性的:
        編隊 worst 從 53 m 跳到 167→187 m,因為停發把「稍微過期」換成
        「完全沒有」,短間隙下後者更糟。
        """
        m = re.search(r'os\.environ\.get\("FIX_MAX_AGE",\s*"([\d.]+)"\)', ROLE)
        assert m, "找不到 FIX_MAX_AGE 預設"
        pub = float(m.group(1))
        w = re.search(r'declare_parameter\("dr_hold_s",\s*([\d.]+)\)', WING)
        assert w, "找不到 dr_hold_s"
        assert pub >= float(w.group(1)), \
            "發布端門檻 %.1fs 比消費端 dr_hold %.1fs 還嚴 —— 會製造消費端本來不會察覺的空窗" \
            % (pub, float(w.group(1)))

    def test_threshold_still_catches_the_real_failure(self):
        """ANM1 的真實凍結是 448 s,門檻放寬到 5 s 仍然遠遠擋得住。"""
        m = re.search(r'os\.environ\.get\("FIX_MAX_AGE",\s*"([\d.]+)"\)', ROLE)
        assert float(m.group(1)) < 60.0

    def test_gate_releases_on_recovery(self):
        """⚠️ 必須是「閘門 + 永遠放行」:唯一復原條件是再收到一筆 fix。
        不可以 latch,否則長機一次 GPS 抖動就永遠不再發布。"""
        m = re.search(r"if self\._fix_max_age > 0\.0 and age > self\._fix_max_age:(.*?)\n            self\.lat",
                      ROLE, re.S)
        assert m, "找不到年齡閘門"
        body = m.group(1)
        assert "return" in body
        assert "latch" not in body.lower()
        # 復原是靠 _on_fix 重寫 _fix_t,不是靠閘門內部狀態
        assert "_fix_t" not in body.replace("self._fix_max_age", "")

    def test_zero_fix_is_no_longer_silent(self):
        """(0,0) 定位是同一個病的第二個入口,原本是裸 return。"""
        assert "_zero_fix_n" in ROLE
        assert re.search(r"收到 \(0,0\) 定位", ROLE)


class TestConsumerWatchdog:
    def test_frozen_content_is_treated_as_lost(self):
        """訊息一直到(0.1 s)但內容凍了 448 s ⇒ 有效年齡必須是 448。"""
        assert effective_age(0.1, 448.0) == pytest.approx(448.0)

    def test_normal_flight_unaffected(self):
        """正常飛行時內容每筆都在動 ⇒ 兩者相同,不影響既有行為。"""
        assert effective_age(0.1, 0.1) == pytest.approx(0.1)

    def test_watchdog_disabled_reverts(self):
        assert effective_age(0.1, 448.0, watchdog=False) == pytest.approx(0.1)

    @pytest.mark.parametrize("content_age,expect_dr,expect_giveup", [
        (0.1, False, False),
        (6.0, True, False),      # > dr_hold 5 s
        (60.0, True, True),      # > dr_max 45 s
    ])
    def test_reuses_existing_thresholds(self, content_age, expect_dr, expect_giveup):
        """重用既有的 5 s / 45 s,不新增分支。"""
        age = effective_age(0.1, content_age)
        assert (age > 5.0) is expect_dr
        assert (age > 45.0) is expect_giveup

    def test_wingman_tracks_content_movement(self):
        assert "_leader_move_t" in WING
        assert "_leader_prev" in WING

    def test_move_threshold_is_above_gps_jitter(self):
        """門檻要大於 GPS 抖動、小於任何真實移動(巡航 100 m/s ⇒ 0.1 s 走 10 m)。"""
        m = re.search(r"dist_m\(self\._leader_prev, \(lat, lon\)\) > ([\d.]+)", WING)
        assert m, "找不到位移門檻"
        assert 0.1 <= float(m.group(1)) <= 5.0

    def test_init_uses_startup_time_not_negative_infinity(self):
        """⚠️ 初值用 -1e9 的話,開機第一筆進來之前 c_age = 1e9,當場誤判凍結。"""
        m = re.search(r"self\._leader_move_t\s*=\s*(.+)", WING)
        assert m
        assert "get_clock" in m.group(1), "初值必須是節點啟動時刻"

    def test_has_env_fallback(self):
        assert 'os.environ.get("LEADER_CONTENT_WATCHDOG"' in WING

    def test_warns_distinctly_from_disconnect(self):
        """兩種失效模式要分得開 —— 訊息說的是「仍在到達但內容凍結」。"""
        assert "內容已凍結" in WING


class TestOrthogonality:
    def test_two_independent_layers(self):
        """發布端與消費端各自獨立:任一端單獨生效都能擋住這個事故。
        (發布端停發 ⇒ 消費端既有的訊息年齡就會跳;
         發布端沒改 ⇒ 消費端的內容年齡照樣會跳。)"""
        assert "_fix_max_age" in ROLE
        assert "_content_watchdog" in WING
        assert "_fix_max_age" not in WING, "消費端不該依賴發布端的實作"
