"""
ui.controllers.stot_ros2_controller — 地圖點擊 → WSL ROS2 STOT 全流程
=====================================================================

使用者在地圖上點一個紅心,按一下按鈕,WSL 那條 ROS2 管線就從彈射起飛一路跑到
同步俯衝命中,AeroPlan 全程顯示相位、隨時可中止。

⚠️ 這是**第二條、平行的**打擊實作,不是取代。
   AeroPlan 自有的 MAVLink 上傳路徑(strike_controller.py 的 EXECUTE /
   TERMINAL SYNC / 上傳 SITL)完全沒有被動到。兩條路同時跑會搶控制權,
   所以啟動時會呼叫 ``StrikeTTTDashboard.set_strike_running(True)`` 互鎖。

分層:
    integrations/stot_ros2_bridge.py   純邏輯(組指令、解析 log、前置判斷)
                                       —— 零 Qt、零 WSL,可在 WSL 的 python3 單測
    本檔                                subprocess / QTimer / QMessageBox

⚠️⚠️ 副作用預告(這不是 bug,是上游腳本的設計):
   run_west.sh 末行 ``exec bash stot_repeat.sh``,而 stot_repeat.sh:51/66/67 是
   swarm_sitl_down → swarm_sitl_up → sleep 150。按下執行鈕之後幾秒內,
   **WSL 的 6 架 SITL 會被拆掉重建**,AeroPlan 在 6760+10i 的連線全部斷開。
   所以流程裡有一個明確的 Yes/Cancel 警告,按 Yes 之後會主動斷線 ——
   否則會留下一堆對著死埠重試的 SITLLink QThread。

進度來源是**檔案輪詢**,不是 PIPE:
   * mission/sitl_launcher.py:658-660 已記下 PIPE 會 deadlock 的教訓
   * 相位有一半在 runs/trial_<T>.log 裡,PIPE 本來就拿不到那半邊
   * \\\\wsl.localhost\\<distro>\\... 這條 UNC 路徑從 Windows 直接可讀,
     所以輪詢**不需要每秒 spawn 一次 wsl.exe**

絕對不會做的事:
   * 不建立任何 MAVLink 連線去「順手測一下埠通不通」(會把 AeroPlan 餓死)
   * 不啟動 SITL、不跑 deploy_build.sh
   * 中止時不殺 arduplane / mavproxy(留著讓使用者能立刻重連)
"""
from __future__ import annotations

import os
import subprocess
import time

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QApplication, QMessageBox

from integrations import stot_ros2_bridge as bridge
from utils.logger import get_logger

logger = get_logger()

#: Windows 端 subprocess 旗標 —— 不要彈出黑窗,進度全部走 UI。
_NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)

#: UI 端的硬上限。**只警示,不自動殺。**
#: stot_trial.sh:435-447 檢討過 CA1(被砍在 t+900 s,實際需要 1129 s)與
#: D451(殺在俯衝中,差 2.5 秒)—— 自動殺會製造「不可歸因的失敗」,
#: 那是這個專案代價最高的一類錯誤。單趟最壞約 65 分鐘。
_SOFT_TIMEOUT_S = 75 * 60

#: 節流:輪詢 1 Hz,但輸出最多每 2 s 一次(沿用 _tick_terminal_sync 的慣例)。
_LOG_THROTTLE_S = 2.0


class StotRos2ControllerMixin:
    """ROS2 STOT 全流程 Controller — 供 MainWindow 繼承。

    Mixin 不佔用 MainWindow.__init__ —— 狀態透過 ``_stot()`` 延遲建立,
    這樣 main_window.py 只需要多兩行 connect。
    """

    # ═════════════════════════════════════════════════════════════════
    #  狀態
    # ═════════════════════════════════════════════════════════════════
    def _stot(self) -> dict:
        st = getattr(self, "_stot_state", None)
        if st is None:
            st = {
                "proc": None,          # subprocess.Popen(wsl.exe …)
                "prefix": "",
                "trial": "",
                "timer": None,         # QTimer
                "t0": 0.0,
                "tracker": None,       # bridge.PhaseTracker
                "offsets": {},         # {windows path: 已讀 byte 數}
                "last_emit": 0.0,
                "runs_unc": "",        # \\wsl.localhost\…\fixwing_ws\runs
                "sent": None,          # (lat, lon) 送出去的值
                "geom_checked": False,
                "warned_timeout": False,
                "closed_at": 0.0,     # 結案哨兵出現的時刻(仍等驅動收尾)
            }
            self._stot_state = st
        return st

    def _stot_running(self) -> bool:
        st = self._stot()
        p = st["proc"]
        return bool(st["timer"]) or (p is not None and p.poll() is None)

    # ═════════════════════════════════════════════════════════════════
    #  UI 小工具
    # ═════════════════════════════════════════════════════════════════
    def _stot_set_status(self, text: str) -> None:
        panel = getattr(self, "parameter_panel", None)
        lbl = getattr(panel, "_stot_status_label", None) if panel else None
        if lbl is not None:
            lbl.setText(text)

    def _stot_set_buttons(self, running: bool) -> None:
        panel = getattr(self, "parameter_panel", None)
        if panel is None:
            return
        run_btn = getattr(panel, "_stot_run_btn", None)
        abort_btn = getattr(panel, "_stot_abort_btn", None)
        if run_btn is not None:
            run_btn.setEnabled(not running)
        if abort_btn is not None:
            abort_btn.setEnabled(running)

    def _stot_say(self, msg: str, ms: int = 4000) -> None:
        """相位訊息的三個去處:常駐標籤、狀態列、log —— 三者一致才好對帳。"""
        self._stot_set_status(msg)
        try:
            self.statusBar().showMessage(msg, ms)
        except Exception:
            pass
        logger.info("[STOT-ROS2] " + msg)

    def _stot_error(self, title: str, body: str) -> None:
        """大聲失敗。這個專案最大的敵人是靜默失敗,所以一律彈窗 + log。"""
        logger.error("[STOT-ROS2] %s | %s" % (title, body))
        QMessageBox.critical(self, title, body)

    # ═════════════════════════════════════════════════════════════════
    #  執行
    # ═════════════════════════════════════════════════════════════════
    def _on_stot_ros2_run(self) -> None:
        """『▶ 以此點執行 ROS2 STOT 打擊』的 slot。"""
        st = self._stot()

        if self._stot_running():
            self._stot_error(
                "ROS2 STOT — 已有航次在跑",
                "AeroPlan 這邊已經有一趟在跑(%s)。\n\n"
                "⚠️ 絕對不能平行跑兩趟:stot_repeat.sh:50 與 swarm_sitl_down.sh:4 的\n"
                "   pkill 都是無差別的,第二趟會**靜默殺掉**第一趟。\n\n"
                "請先按 ABORT。" % (st["trial"] or "未命名"))
            return

        # ── 1) 紅心座標 ────────────────────────────────────────────
        targets = getattr(self, "_strike_targets", None) or []
        target = None
        if targets:
            try:
                target = bridge.validate_target(targets[0][0], targets[0][1])
            except bridge.StotBridgeError as exc:
                self._stot_error("ROS2 STOT — 目標座標不合法", str(exc))
                return

        # ── 2) WSL 前置檢查(單次呼叫,不碰任何 MAVLink 埠)──────────
        self._stot_say("前置檢查中…")
        out = err = ""
        fatal = None       # (標題, 內文);彈窗一律等游標還原之後再開
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        try:
            probe = subprocess.run(
                bridge.build_preflight_command(),
                capture_output=True, text=True, encoding="utf-8",
                errors="replace", timeout=40, creationflags=_NO_WINDOW)
            out = probe.stdout or ""
            err = probe.stderr or ""
        except FileNotFoundError:
            fatal = ("ROS2 STOT — 找不到 wsl.exe",
                     "無法執行 wsl.exe。這台機器上的 WSL 沒有安裝或不在 PATH。")
        except subprocess.TimeoutExpired:
            fatal = ("ROS2 STOT — WSL 沒有回應",
                     "前置檢查 40 秒沒有回應。請確認 WSL 發行版起得來:\n"
                     "  wsl.exe -d %s -e bash -lc 'echo ok'" % bridge.WSL_DISTRO_DEFAULT)
        except Exception as exc:                      # noqa: BLE001
            fatal = ("ROS2 STOT — 前置檢查失敗",
                     "%s: %s" % (type(exc).__name__, exc))
        finally:
            # 只還原一次 —— restoreOverrideCursor 是堆疊式的,多 pop 會弄壞別人的游標
            try:
                QApplication.restoreOverrideCursor()
            except Exception:
                pass
        if fatal is not None:
            self._stot_error(*fatal)
            self._stot_set_status("待命")
            return

        report = bridge.parse_preflight(out)
        if err.strip():
            logger.warning("[STOT-ROS2] 前置檢查 stderr: %s" % err.strip()[:500])

        issues = bridge.evaluate_preflight(
            report, target=target, local_run_active=False)
        blockers = bridge.blocking_issues(issues)
        if blockers:
            self._stot_error(
                "ROS2 STOT — 前置檢查未通過(%d 項)" % len(blockers),
                "\n\n".join(str(i) for i in blockers))
            self._stot_set_status("前置檢查未通過:" +
                                  "、".join(i.code for i in blockers))
            return

        # ── 3) 航次代號(撞名 = 靜默刪掉舊航次的原始資料)──────────
        try:
            prefix = bridge.make_prefix(report.taken)
        except bridge.StotBridgeError as exc:
            self._stot_error("ROS2 STOT — 無法產生航次代號", str(exc))
            return
        trial = bridge.trial_name(prefix)

        sea = True
        panel = getattr(self, "parameter_panel", None)
        chk = getattr(panel, "_stot_sea_target_chk", None) if panel else None
        if chk is not None:
            sea = bool(chk.isChecked())

        env = bridge.build_launch_env(target[0], target[1], prefix,
                                      sea_level_target=sea)
        cmd = bridge.build_run_command(target[0], target[1], prefix,
                                       sea_level_target=sea)

        # ── 4) 確認 —— 把**實際送出的值**原樣攤開給使用者看 ────────
        env_txt = "\n".join("    %-14s = %s" % (k, v) for k, v in env.items())
        notices = [str(i) for i in issues if not i.blocking]
        n_links = len(getattr(self, "_sitl_links", []) or [])
        msg = (
            "航次代號:%s\n\n"
            "實際送進 WSL 的值(逐字):\n%s\n\n"
            "%s\n\n"
            "⚠️ AeroPlan 目前的 %d 條 SITL 連線會被斷開(本流程會重建 SITL)。\n"
            "⚠️⚠️ 航次進行中請**不要**重新連線嵌入式蜂群:階段 2 的 jet_show_ros2.py\n"
            "    走的是同一組 6760+10i 分流,而 MAVProxy 的 tcpin 只接受先連上的\n"
            "    那一個 client。AeroPlan 先佔住的話,階段 2 會「印出連線就緒」然後\n"
            "    空轉滿 1300 秒、一行進度都不印 —— 白燒 20 幾分鐘且看起來像正常跑完。\n"
            "⚠️ 單趟典型 25–30 分鐘,最壞約 65 分鐘。全程可按 ABORT 中止。\n\n"
            "要開始嗎?" % (trial, env_txt, "\n\n".join(notices), n_links))
        if QMessageBox.question(
                self, "ROS2 STOT 全流程 — 確認", msg,
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
                QMessageBox.StandardButton.Cancel) != QMessageBox.StandardButton.Yes:
            self._stot_set_status("已取消")
            return

        # ── 5) 主動斷線 —— SITL 幾秒後就會被 pkill 掉 ───────────────
        if n_links:
            try:
                self.on_sitl_disconnect()
                logger.info("[STOT-ROS2] 已主動斷開 %d 條 SITL 連線(SITL 即將被重建)" % n_links)
            except Exception as exc:                  # noqa: BLE001
                logger.warning("[STOT-ROS2] 斷線時出錯(不阻擋):%s" % exc)

        # ── 6) 互斥:鎖住 AeroPlan 自有的 MAVLink 打擊路徑 ──────────
        dash = getattr(self, "strike_ttt_dashboard", None)
        if dash is not None:
            try:
                dash.set_strike_running(True)
            except Exception:
                pass

        # ── 7) 啟動 ────────────────────────────────────────────────
        logger.info("[STOT-ROS2] 啟動 %s:%s" % (trial, " ".join(cmd)))
        try:
            proc = subprocess.Popen(
                cmd, stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                creationflags=_NO_WINDOW)
        except Exception as exc:                      # noqa: BLE001
            if dash is not None:
                try:
                    dash.set_strike_running(False)
                except Exception:
                    pass
            self._stot_error("ROS2 STOT — 啟動失敗", "%s: %s" % (type(exc).__name__, exc))
            return

        st["proc"] = proc
        st["prefix"] = prefix
        st["trial"] = trial
        st["t0"] = time.time()
        st["tracker"] = bridge.PhaseTracker()
        st["offsets"] = {}
        st["last_emit"] = 0.0
        st["sent"] = target
        st["geom_checked"] = False
        st["warned_timeout"] = False
        # ⚠️ 每一個 per-run 欄位都要重設。漏一個就會用上一趟的殘值判這一趟 ——
        #    closed_at 沒歸零的話,第二趟一開始就被當成「已結案」。
        st["closed_at"] = 0.0
        st["runs_unc"] = self._stot_runs_unc(report.wsl_home)

        self._stot_set_buttons(True)
        self._stot_say("%s 已啟動 — TGT_LAT=%s TGT_LON=%s APPROACH_FROM=%s "
                       "TARGET_AMSL=%s" % (trial, env["TGT_LAT"], env["TGT_LON"],
                                           env["APPROACH_FROM"], env["TARGET_AMSL"]),
                       ms=8000)

        timer = QTimer(self)
        timer.timeout.connect(self._tick_stot_ros2)
        timer.start(1000)
        st["timer"] = timer

    # ═════════════════════════════════════════════════════════════════
    #  進度輪詢
    # ═════════════════════════════════════════════════════════════════
    @staticmethod
    def _stot_runs_unc(wsl_home: str) -> str:
        """把 WSL 的 $HOME 換成 Windows 看得到的 UNC 路徑。

        \\\\wsl.localhost\\<distro>\\home\\<user>\\fixwing_ws\\runs
        —— 已實測從 Windows 可直接列檔,所以輪詢不必 spawn wsl.exe。
        """
        home = (wsl_home or "").strip().rstrip("/")
        if not home.startswith("/"):
            home = "/home"
        return "\\\\wsl.localhost\\%s%s\\%s" % (
            bridge.WSL_DISTRO_DEFAULT, home.replace("/", "\\"),
            bridge.RUNS_SUBPATH.replace("/", "\\"))

    def _stot_read_new(self, path: str) -> str:
        """只讀增量。**不完整的最後一行留到下一 tick** —— log 是邊寫邊讀的,
        半行餵進 parser 會漏掉相位標記。"""
        st = self._stot()
        try:
            with open(path, "rb") as fh:
                fh.seek(st["offsets"].get(path, 0))
                buf = fh.read()
        except OSError:
            return ""
        if not buf:
            return ""
        cut = buf.rfind(b"\n")
        if cut < 0:
            return ""
        st["offsets"][path] = st["offsets"].get(path, 0) + cut + 1
        return buf[:cut + 1].decode("utf-8", errors="replace")

    def _tick_stot_ros2(self) -> None:
        st = self._stot()
        tracker = st["tracker"]
        if tracker is None:
            return
        runs = st["runs_unc"]
        trial = st["trial"]
        driver_log = os.path.join(runs, "driver_%s.log" % st["prefix"])
        trial_log = os.path.join(runs, "trial_%s.log" % trial)

        changed = False
        for path in (driver_log, trial_log):
            chunk = self._stot_read_new(path)
            if chunk and tracker.feed(chunk):
                changed = True

        # ── 回讀驗證:節點自己寫下的目標 vs 我們送出去的值 ──────────
        if not st["geom_checked"]:
            self._stot_verify_target(tracker)

        now = time.time()
        elapsed = now - st["t0"]
        proc = st["proc"]
        rc = proc.poll() if proc is not None else None

        if changed or (now - st["last_emit"]) >= _LOG_THROTTLE_S:
            st["last_emit"] = now
            self._stot_say("%s · t+%ds · %s"
                           % (trial, int(elapsed), tracker.status_text()), ms=3000)

        # ── 唯一的終點:驅動行程自己結束 ────────────────────────────
        if rc is not None:
            self._stot_finish(rc)
            return

        # ── UI 端軟上限 —— 只警示,絕不自動殺 ───────────────────────
        if elapsed > _SOFT_TIMEOUT_S and not st["warned_timeout"]:
            st["warned_timeout"] = True
            logger.warning("[STOT-ROS2] %s 已跑 %.0f 分鐘,超過軟上限 %.0f 分鐘"
                           % (trial, elapsed / 60.0, _SOFT_TIMEOUT_S / 60.0))
            QMessageBox.warning(
                self, "ROS2 STOT — 超過預期時間",
                "%s 已經跑了 %.0f 分鐘,超過預期上限。\n\n"
                "**沒有自動中止**:自動殺會製造不可歸因的失敗\n"
                "(stot_trial.sh:435-447 對 CA1/D451 的檢討就是這個教訓)。\n\n"
                "要停就請自己按 ABORT。" % (trial, elapsed / 60.0))

        # ── 結案哨兵已出現,但驅動還在收尾 ──────────────────────────
        # ⚠️ **不要在這裡收工。** tot_node 印完 ★★ 俯衝結案 之後,
        #    stot_trial.sh 還要跑 make_artifacts.sh 產素材、stot_score.py 計分,
        #    runs/artifacts/<T>/ 與 stot_repeat_v2.csv 都在那之後才落地。
        #    提早收工會同時失去摘要數字,並把還在跑的驅動變成沒人管的孤兒。
        if tracker.finished and st["closed_at"] == 0.0:
            st["closed_at"] = now
            self._stot_say("%s · %s —— 等待素材與計分落地"
                           % (trial, tracker.status_text()), ms=10000)

    def _stot_verify_target(self, tracker) -> None:
        """『不要相信自己送出的值』—— 拿節點回報的目標跟送出值對帳。"""
        st = self._stot()
        sent = st["sent"]
        if sent is None:
            return
        echoed = tracker.state.echoed_target_node
        if echoed is None:
            geom = os.path.join(st["runs_unc"], "tot_%s_geom.csv" % st["trial"])
            try:
                with open(geom, "r", encoding="utf-8", errors="replace") as fh:
                    echoed = bridge.parse_geom_target(fh.read())
            except OSError:
                echoed = None
        if echoed is None:
            return
        st["geom_checked"] = True
        if bridge.target_matches(sent, echoed):
            logger.info("[STOT-ROS2] 目標回讀驗證通過:送出 (%s, %s) = 節點 (%s, %s)"
                        % (bridge.fmt_coord(sent[0]), bridge.fmt_coord(sent[1]),
                           bridge.fmt_coord(echoed[0]), bridge.fmt_coord(echoed[1])))
            return
        self._stot_error(
            "ROS2 STOT — ⚠️⚠️ 目標回讀不符",
            "送出的紅心和 tot_node 實際採用的不是同一點:\n\n"
            "    送出   %s, %s\n"
            "    節點   %s, %s\n\n"
            "這趟打的不是你點的那一點。建議立刻 ABORT。"
            % (bridge.fmt_coord(sent[0]), bridge.fmt_coord(sent[1]),
               bridge.fmt_coord(echoed[0]), bridge.fmt_coord(echoed[1])))

    # ═════════════════════════════════════════════════════════════════
    #  結束
    # ═════════════════════════════════════════════════════════════════
    def _stot_stop_timer(self) -> None:
        st = self._stot()
        timer = st["timer"]
        if timer is not None:
            try:
                timer.stop()
                timer.deleteLater()
            except Exception:
                pass
        st["timer"] = None

    def _stot_finish(self, rc) -> None:
        st = self._stot()
        tracker = st["tracker"]
        trial = st["trial"]
        self._stot_stop_timer()

        # 最後再掃一次尾巴 —— 結案摘要常常就寫在行程結束的前一瞬間
        if tracker is not None:
            for path in (os.path.join(st["runs_unc"], "driver_%s.log" % st["prefix"]),
                         os.path.join(st["runs_unc"], "trial_%s.log" % trial)):
                chunk = self._stot_read_new(path)
                if chunk:
                    tracker.feed(chunk)

        summary = self._stot_read_summary(trial)
        parts = ["%s 結束" % trial, bridge.describe_rc(rc)]
        if tracker is not None:
            parts.append(tracker.status_text())
        if summary:
            parts.append(
                "pass=%s 最大誤差=%s m 觸地散布=%s s 平均觸地地速=%s m/s"
                % (summary.get("pass", "?"), summary.get("max_impact_miss_m", "?"),
                   summary.get("touch_spread_s", "?"), summary.get("gs_touch_mean", "?")))
        parts.append("素材:~/%s/artifacts/%s/" % (bridge.RUNS_SUBPATH, trial))
        text = " · ".join(parts)

        logger.info("[STOT-ROS2] " + text)
        self._stot_set_status(text)
        try:
            self.statusBar().showMessage(text, 15000)
        except Exception:
            pass

        st["proc"] = None
        self._stot_set_buttons(False)
        dash = getattr(self, "strike_ttt_dashboard", None)
        if dash is not None:
            try:
                dash.set_strike_running(False)
            except Exception:
                pass

        # ⚠️ 刻意**不**自動重連 SITL。使用者的 SITL 由使用者決定何時連 ——
        #    在程式裡假設 SITL 是我們起的,正是本專案踩過的坑。
        if tracker is not None and tracker.state.closed_out:
            QMessageBox.information(self, "ROS2 STOT — 結案", text)
        else:
            QMessageBox.warning(
                self, "ROS2 STOT — 未結案",
                text + "\n\n(沒有偵測到結案哨兵。WSL 上的 SITL 已重建,"
                       "可從 SITL 分頁重新連線。)")

    def _stot_read_summary(self, trial: str):
        path = os.path.join(self._stot()["runs_unc"], "stot_repeat_v2.csv")
        try:
            with open(path, "r", encoding="utf-8", errors="replace") as fh:
                return bridge.parse_summary_row(fh.read(), trial)
        except OSError:
            return None

    # ═════════════════════════════════════════════════════════════════
    #  關閉視窗時的去留
    # ═════════════════════════════════════════════════════════════════
    def _stot_close_decision(self) -> bool:
        """MainWindow.closeEvent 呼叫。回傳 False 代表使用者選擇不關閉。

        兩種靜默都不可接受:靜默中止會製造不可歸因的失敗;靜默留著則變成一趟
        沒人監看、也沒有 UI 可以中止的孤兒航次(而它正在指揮六架飛機)。
        """
        if not self._stot_running():
            return True
        trial = self._stot()["trial"] or "(未命名)"
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Icon.Warning)
        box.setWindowTitle("ROS2 STOT 航次仍在執行")
        box.setText(
            "航次 %s 還在 WSL 上跑。\n\n"
            "關掉 AeroPlan 之後就沒有 UI 可以中止它了 —— "
            "tot_node 是 setsid nohup 起的,會繼續指揮六架飛機。" % trial)
        b_abort = box.addButton("中止航次再關閉", QMessageBox.ButtonRole.DestructiveRole)
        b_keep = box.addButton("留著繼續跑", QMessageBox.ButtonRole.ActionRole)
        b_cancel = box.addButton("不要關閉", QMessageBox.ButtonRole.RejectRole)
        box.setDefaultButton(b_cancel)
        box.exec()
        clicked = box.clickedButton()
        if clicked is b_cancel:
            return False
        if clicked is b_abort:
            self._on_stot_ros2_abort()
        else:
            self._stot_stop_timer()
            logger.warning(
                "[STOT-ROS2] %s 在 AeroPlan 關閉後繼續執行 —— "
                "要中止請在 WSL 端自己來:pkill -f '[t]ot_node'; "
                "bash ~/%s/swarm_ros2_down.sh" % (trial, bridge.SITL_RUN_SUBPATH))
        return True

    # ═════════════════════════════════════════════════════════════════
    #  中止
    # ═════════════════════════════════════════════════════════════════
    def _on_stot_ros2_abort(self) -> None:
        """兩段式中止。第二段(WSL 端拆除)**無條件執行**。

        殺掉 Windows 端的 wsl.exe 收不乾淨:tot_node 是 setsid nohup、
        SITL 在背景子殼、MAVProxy 也是 setsid,而 stot_trial.sh:195 的 TERM trap
        跳過了 PGID 收尾。殘留的 tot_node 會繼續指揮六架飛機,而且完全靜默。
        """
        st = self._stot()
        if not self._stot_running() and st["proc"] is None:
            # 冪等:沒在跑就當清場,仍然跑一次拆除比較安全
            logger.info("[STOT-ROS2] ABORT:本地沒有航次在跑,仍執行一次 WSL 端清場")

        trial = st["trial"] or "(未命名)"
        tracker = st["tracker"]
        pgid = tracker.state.tot_pgid if tracker is not None else None

        self._stot_stop_timer()
        self._stot_say("%s 中止中…" % trial, ms=6000)

        # 1) 先收 Windows 端的驅動行程
        proc = st["proc"]
        if proc is not None and proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    proc.kill()
                except Exception:
                    pass
            except Exception as exc:                  # noqa: BLE001
                logger.warning("[STOT-ROS2] terminate 失敗:%s" % exc)
        st["proc"] = None

        # 2) WSL 端拆除 —— 無條件
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        out = err = ""
        failed = None
        try:
            res = subprocess.run(
                bridge.build_abort_command(pgid),
                capture_output=True, text=True, encoding="utf-8",
                errors="replace", timeout=60, creationflags=_NO_WINDOW)
            out, err = res.stdout or "", res.stderr or ""
        except Exception as exc:                      # noqa: BLE001
            failed = "%s: %s" % (type(exc).__name__, exc)
        finally:
            try:
                QApplication.restoreOverrideCursor()
            except Exception:
                pass

        self._stot_set_buttons(False)
        dash = getattr(self, "strike_ttt_dashboard", None)
        if dash is not None:
            try:
                dash.set_strike_running(False)
            except Exception:
                pass

        # 3) 讀回來驗證 —— 拆完不驗證等於沒拆
        if failed is not None:
            self._stot_error(
                "ROS2 STOT — 中止指令沒跑完",
                "WSL 端的拆除指令失敗:%s\n\n"
                "無法確認殘留。請在自己的終端機執行:\n"
                "  pgrep -af '[t]ot_node'\n"
                "  bash ~/%s/swarm_ros2_down.sh" % (failed, bridge.SITL_RUN_SUBPATH))
            self._stot_set_status("%s 中止:狀態未知(拆除指令失敗)" % trial)
            return

        logger.info("[STOT-ROS2] abort stdout: %s | stderr: %s"
                    % (out.strip()[:400], err.strip()[:200]))
        try:
            resid = bridge.parse_resid(out)
        except bridge.StotBridgeError as exc:
            self._stot_error("ROS2 STOT — 中止後無法確認殘留", str(exc))
            self._stot_set_status("%s 中止:殘留未確認" % trial)
            return

        left = bridge.resid_leftovers(resid)
        resid_txt = " ".join("%s=%d" % (k, v) for k, v in resid.items())
        if left:
            self._stot_error(
                "ROS2 STOT — ⚠️⚠️ 中止後仍有殘留",
                "以下行程在 TERM+KILL 之後還活著:\n    %s\n\n"
                "完整計數:%s\n\n"
                "殘留的 tot_node 會跟下一趟搶控制,而且完全靜默。\n"
                "請在自己的終端機確認:\n  pgrep -af '[t]ot_node'"
                % (" ".join("%s=%d" % kv for kv in left.items()), resid_txt))
            self._stot_set_status("%s 已中止,但有殘留:%s" % (trial, resid_txt))
        else:
            msg = ("%s 已中止,WSL 端無殘留(%s)。arduplane 保留著,"
                   "可從 SITL 分頁重新連線。" % (trial, resid_txt))
            self._stot_say(msg, ms=12000)
