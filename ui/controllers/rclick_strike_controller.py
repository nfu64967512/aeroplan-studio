"""
ui.controllers.rclick_strike_controller — 地圖右鍵「以此點執行 STOT 打擊」
=========================================================================

使用者在 3D 地圖上右鍵一點,選「攻擊」,那一點就被送給**已經待命中**的
tot_node,六架立刻進入 ALIGN → PERCH → DIVE。AeroPlan 全程顯示相位。

⚠️ 這是**第三條**路徑,前兩條一個位元都沒動:
      strike_controller.py         AeroPlan 自有的 MAVLink 上傳打擊
      stot_ros2_controller.py      批次:整包 shell out 到 run_west.sh
                                   (會拆掉重建 SITL,跑完一趟才結束)
      本檔                          互動:SITL / ROS2 層都是常駐的,
                                   只丟一個座標給待命中的 tot_node
   實體分成三個檔案,review diff 時一眼就能確認舊路徑零改動。

分層:
    integrations/rclick_strike.py   純邏輯(組指令、前置判斷、座標格式化、
                                    相位解析)—— 零 Qt、零 WSL,WSL python3 可單測
    本檔                             subprocess / QTimer / QMessageBox

進度來源是**檔案輪詢**,不是 PIPE:
   * WSL 端起一個取樣器把 /swarm/tot_status 與 /swarm/tot_gate 週期性寫進
     ~/fixwing_ws/runs/rclick_strike_<stamp>.log
   * Windows 端從 \\\\wsl.localhost\\<distro>\\… 直接讀增量(已實測可讀)
   * PIPE 會 deadlock(mission/sitl_launcher.py:658-660 的教訓),而且
     每秒 spawn 一次 wsl.exe 去問狀態太貴

絕對不會做的事:
   * 不建立任何 MAVLink 連線去「順手測一下埠通不通」——MAVProxy 的
     ``--out tcpin`` 只餵先連上的 client,測一下就會把 AeroPlan 自己餓死
   * 不啟動 SITL、不跑 deploy_build.sh、不碰 scripts/wsl/
   * 不殺 tot_node ——殺了就沒有待命模式了。這條路只送目標與看狀態,
     真的要中止請走「蜂群打擊」分頁的 ABORT,或自己在 WSL 端收。
"""
from __future__ import annotations

import os
import subprocess
import time
from datetime import datetime

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QApplication, QMessageBox

from integrations import rclick_strike as rcs
from utils.logger import get_logger

logger = get_logger()

#: Windows 端 subprocess 旗標 —— 不要彈黑窗。
_NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)

#: 前置檢查的整體上限。裡面有三個 ros2 CLI 呼叫,每個都自帶 timeout,
#: 這是最外層的保險(WSL 整個沒回應時才會用到)。
_PREFLIGHT_TIMEOUT_S = 60
#: 送目標:pub 自己有 timeout(rcs.PUB_TIMEOUT_S),外層再加緩衝。
_PUBLISH_TIMEOUT_S = rcs.PUB_TIMEOUT_S + 20
_WATCH_START_TIMEOUT_S = 30
_WATCH_STOP_TIMEOUT_S = 30

#: 輪詢節流(沿用批次那條的慣例:1 Hz 讀檔,最多每 2 s 更新一次文字)。
_LOG_THROTTLE_S = 2.0

#: 監看的軟上限。**只停監看,不動飛機** —— 自動殺會製造不可歸因的失敗。
_WATCH_SOFT_LIMIT_S = 75 * 60

#: 節點還沒把新目標寫進 gate 之前,回讀對帳一定會不符。給它這麼久才報警。
_READBACK_GRACE_S = 30.0


class RClickStrikeControllerMixin:
    """右鍵打擊 Controller — 供 MainWindow 繼承。

    Mixin 不佔用 MainWindow.__init__:狀態由 ``_rcs()`` 延遲建立。
    """

    # ═════════════════════════════════════════════════════════════════
    #  狀態
    # ═════════════════════════════════════════════════════════════════
    def _rcs(self) -> dict:
        st = getattr(self, "_rcs_state", None)
        if st is None:
            st = {
                "timer": None,          # QTimer
                "watch_pid": None,      # WSL 端取樣器 PID
                "watch_unc": "",        # Windows 看得到的取樣檔
                "offset": 0,            # 已讀 byte 數
                "sent": None,           # (lat, lon) 送出去的值
                "t0": 0.0,
                "last_emit": 0.0,
                "phase": "",
                "verified": False,      # gate 回讀是否已對上
                "warned_mismatch": False,
                "warned_limit": False,
            }
            self._rcs_state = st
        return st

    def rclick_strike_active(self) -> bool:
        return self._rcs()["timer"] is not None

    # ═════════════════════════════════════════════════════════════════
    #  UI 小工具
    # ═════════════════════════════════════════════════════════════════
    def _rcs_say(self, msg: str, ms: int = 6000) -> None:
        """相位訊息的兩個去處:狀態列 + log。兩邊一致才好對帳。"""
        try:
            self.statusBar().showMessage(msg, ms)
        except Exception:
            pass
        logger.info("[RCLICK-STRIKE] " + msg)

    def _rcs_error(self, title: str, body: str) -> None:
        """大聲失敗。這個專案最大的敵人是靜默失敗 ⇒ 一律彈窗 + log。"""
        logger.error("[RCLICK-STRIKE] %s | %s" % (title, body))
        QMessageBox.critical(self, title, body)

    @staticmethod
    def _rcs_issue_text(issues) -> str:
        return "\n\n".join(str(i) for i in issues)

    # ═════════════════════════════════════════════════════════════════
    #  對外入口
    # ═════════════════════════════════════════════════════════════════
    def rclick_strike_at(self, lat: float, lon: float, *,
                         via_alt_zero: bool = False) -> bool:
        """把右鍵的那一點送給待命中的 tot_node。回傳有沒有真的送出去。

        流程:前置檢查 → 確認框(逐字攤開座標)→ 送出 → 起監看 → 輪詢相位。
        任何一關沒過都**不會送**,而且一定看得到原因。
        """
        st = self._rcs()

        # ── 0) 座標本身 ────────────────────────────────────────────
        try:
            la, lo = rcs.validate_target(lat, lon)
        except rcs.StotBridgeError as exc:
            self._rcs_error("右鍵打擊 — 座標不合法",
                            "%s\n\n(台灣附近 lat≈25、lon≈121;"
                            "lat/lon 顛倒是最常見的原因)" % exc)
            return False

        # ── 1) 前置檢查 ────────────────────────────────────────────
        self._rcs_say("右鍵打擊:前置檢查中…(WSL / ROS2 圖 / tot_node 待命狀態)", ms=8000)
        report = self._rcs_preflight()
        if report is None:
            return False
        issues = rcs.evaluate_preflight(
            report, target=(la, lo), local_strike_active=self.rclick_strike_active())
        blocking = rcs.blocking_issues(issues)
        if blocking:
            self._rcs_error(
                "右鍵打擊 — 前置檢查未通過(沒有送出任何東西)",
                self._rcs_issue_text(blocking + [i for i in issues if not i.blocking]))
            return False

        # ── 2) 確認框 —— 逐字攤開要送出去的座標,並且**明示高度基準** ────
        #
        # ⚠️⚠️ 高度基準是「海」還是「陸」不可以猜。
        #    海上:SRTM 在水域沒有資料,tot_node 查回來的是垃圾 ——
        #          實測散布 17 m,而驗收門檻是 3 s(sea-level-target-datum)。
        #    陸上:反過來,釘 amsl=0 會讓紅心沉到地面以下幾百公尺。
        #    兩個方向都錯得很安靜,所以這裡做成**兩顆並列的動作鈕**,
        #    使用者一定要選一個;預設鈕仍然是取消。
        #    (批次路徑對應的是 TARGET_AMSL 這個參數,那邊是命令列明示。)
        approach = rcs.approach_from_for((la, lo), rcs.HOME_BY_LOC[rcs.DEFAULT_LOC])
        advisories = [i for i in issues if not i.blocking]
        box = QMessageBox(self)
        box.setWindowTitle("右鍵打擊 — 確認目標")
        box.setIcon(QMessageBox.Icon.Question)
        b_sea = box.addButton("🌊 海上目標(amsl = 0)",
                              QMessageBox.ButtonRole.AcceptRole)
        b_land = box.addButton("⛰ 陸上目標(查地形庫)",
                               QMessageBox.ButtonRole.AcceptRole)
        b_cancel = box.addButton("取消", QMessageBox.ButtonRole.RejectRole)
        box.setDefaultButton(b_cancel)              # 預設**取消**
        box.setEscapeButton(b_cancel)

        def _body(amsl):
            # 機數用 tot_node **生效的 n**(拿不到就講「全部機體」),不寫死 6
            t = rcs.confirm_text(la, lo, approach_from=approach, amsl=amsl,
                                 n_aircraft=report.swarm_n,
                                 via_alt_zero=via_alt_zero)
            if advisories:
                t += "\n\n── 注意(不擋,但要知道)──\n" + self._rcs_issue_text(advisories)
            return t

        # 文案先顯示海上版(台灣海峽的紅心是本專案的主要用例),但**兩顆鈕都在**,
        # 使用者按哪顆就送哪一種,不會有「看到 A 送出 B」。
        box.setText(_body(rcs.SEA_LEVEL_AMSL))
        box.setDetailedText("陸上目標會改送:\n\n" + _body(None))
        box.exec()
        picked = box.clickedButton()
        if picked is b_cancel or picked is None:
            self._rcs_say("右鍵打擊:已取消,沒有送出任何東西", ms=4000)
            return False
        amsl = rcs.SEA_LEVEL_AMSL if picked is b_sea else None

        # ── 3) 送出 ────────────────────────────────────────────────
        if not self._rcs_publish(la, lo, approach, amsl=amsl):
            return False

        # ── 4) 監看 ────────────────────────────────────────────────
        stamp = datetime.now().strftime("%m%d_%H%M%S")
        st["sent"] = (la, lo)
        st["t0"] = time.time()
        st["offset"] = 0
        st["last_emit"] = 0.0
        st["phase"] = ""
        st["verified"] = False
        st["warned_mismatch"] = False
        st["warned_limit"] = False
        st["watch_pid"] = None
        st["watch_unc"] = ""
        self._rcs_start_watch(stamp, report.wsl_home)

        self._rcs_say("右鍵打擊:已送出 %s, %s → %s"
                      % (rcs.fmt_coord(la), rcs.fmt_coord(lo), rcs.TARGET_TOPIC),
                      ms=10000)
        return True

    def rclick_strike_stop_watch(self) -> None:
        """停止監看。

        ⚠️ **這不會叫飛機停下來**。tot_node 收到目標之後就自己跑完整條
           ALIGN → PERCH → DIVE,AeroPlan 這邊只是在看。真的要停,
           請走「蜂群打擊」分頁的 ABORT,或在 WSL 端自己收 tot_node。
        """
        st = self._rcs()
        self._rcs_stop_timer()
        pid = st["watch_pid"]
        st["watch_pid"] = None
        if pid is None:
            return
        try:
            res = subprocess.run(
                rcs.build_stop_watch_command(pid), capture_output=True, text=True,
                encoding="utf-8", errors="replace", timeout=_WATCH_STOP_TIMEOUT_S,
                creationflags=_NO_WINDOW)
            logger.info("[RCLICK-STRIKE] 收監看:%s" % (res.stdout or "").strip()[:200])
        except Exception as exc:                      # noqa: BLE001
            logger.warning(
                "[RCLICK-STRIKE] 收監看失敗(%s: %s)—— 取樣器會在 %d 秒後自我到期,"
                "要立刻收就自己來:kill %s"
                % (type(exc).__name__, exc, rcs.WATCH_MAX_S, pid))

    # ═════════════════════════════════════════════════════════════════
    #  三個 wsl.exe 呼叫
    # ═════════════════════════════════════════════════════════════════
    def _rcs_run(self, argv, timeout: int, what: str):
        """跑一次 wsl.exe。失敗一律回 None 並且已經彈過窗。"""
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        try:
            return subprocess.run(
                argv, capture_output=True, text=True, encoding="utf-8",
                errors="replace", timeout=timeout, creationflags=_NO_WINDOW)
        except FileNotFoundError:
            self._rcs_error(
                "右鍵打擊 — 找不到 wsl.exe",
                "無法執行 wsl.exe:這台機器上的 WSL 沒有安裝或不在 PATH。\n\n"
                "自己確認:wsl.exe -d %s -e bash -lc 'echo ok'" % rcs.WSL_DISTRO_DEFAULT)
            return None
        except subprocess.TimeoutExpired:
            self._rcs_error(
                "右鍵打擊 — %s 逾時" % what,
                "WSL 在 %d 秒內沒有回應。\n\n"
                "ROS2 的 CLI 卡住時常常是 daemon 壞了,處置:\n"
                "  wsl.exe -d %s -e bash -lc 'ros2 daemon stop; ros2 daemon start'"
                % (timeout, rcs.WSL_DISTRO_DEFAULT))
            return None
        except Exception as exc:                      # noqa: BLE001
            self._rcs_error("右鍵打擊 — %s 失敗" % what,
                            "%s: %s" % (type(exc).__name__, exc))
            return None
        finally:
            try:
                QApplication.restoreOverrideCursor()
            except Exception:
                pass

    def _rcs_preflight(self):
        res = self._rcs_run(rcs.build_preflight_command(),
                            _PREFLIGHT_TIMEOUT_S, "前置檢查")
        if res is None:
            return None
        out = res.stdout or ""
        logger.info("[RCLICK-STRIKE] preflight stdout: %s" % out.strip()[:600])
        if (res.stderr or "").strip():
            logger.warning("[RCLICK-STRIKE] preflight stderr: %s"
                           % res.stderr.strip()[:300])
        return rcs.parse_preflight(out)

    def _rcs_publish(self, lat: float, lon: float, approach_from: float,
                     amsl=None) -> bool:
        try:
            argv = rcs.build_publish_command(lat, lon, approach_from=approach_from,
                                             amsl=amsl)
        except rcs.StotBridgeError as exc:
            # 組指令時的自我對帳失敗 —— 寧可什麼都不送
            self._rcs_error("右鍵打擊 — 指令組裝失敗(沒有送出任何東西)", str(exc))
            return False

        res = self._rcs_run(argv, _PUBLISH_TIMEOUT_S, "送出目標")
        if res is None:
            return False
        out = res.stdout or ""
        logger.info("[RCLICK-STRIKE] publish: %s | stderr: %s"
                    % (out.strip()[:300], (res.stderr or "").strip()[:300]))
        rc = rcs.parse_pub_rc(out)
        if rc != 0:
            self._rcs_error(
                "右鍵打擊 — 目標沒有送出去",
                "%s\n\nros2 topic pub 的輸出:\n%s\n%s"
                % (rcs.describe_pub_rc(rc), out.strip()[:400],
                   (res.stderr or "").strip()[:400]))
            return False
        return True

    def _rcs_start_watch(self, stamp: str, wsl_home: str) -> None:
        """起取樣器 + 開輪詢。監看起不來**不影響已經送出去的目標**,
        只是看不到相位 —— 這種時候要說清楚,不能讓使用者以為沒打出去。"""
        st = self._rcs()
        res = self._rcs_run(rcs.build_watch_command(stamp),
                            _WATCH_START_TIMEOUT_S, "啟動狀態監看")
        if res is None:
            QMessageBox.warning(
                self, "右鍵打擊 — 目標已送出,但監看沒起來",
                "目標**已經送出去了**(pub rc=0),飛機會照常執行。\n"
                "只是 AeroPlan 這邊看不到相位。\n\n"
                "想自己看:wsl.exe -d %s -e bash -lc "
                "'ros2 topic echo %s'" % (rcs.WSL_DISTRO_DEFAULT, rcs.STATUS_TOPIC))
            return
        pid, wsl_log = rcs.parse_watch_start(res.stdout or "")
        st["watch_pid"] = pid
        if wsl_log:
            st["watch_unc"] = os.path.join(rcs.runs_unc(wsl_home),
                                           os.path.basename(wsl_log))
        logger.info("[RCLICK-STRIKE] 監看 pid=%s log=%s unc=%s"
                    % (pid, wsl_log, st["watch_unc"]))
        if not st["watch_unc"]:
            logger.warning("[RCLICK-STRIKE] 取樣器沒有回報 log 路徑 —— 相位顯示不會更新")
            return

        timer = QTimer(self)
        timer.timeout.connect(self._tick_rclick_strike)
        timer.start(1000)
        st["timer"] = timer

    # ═════════════════════════════════════════════════════════════════
    #  輪詢
    # ═════════════════════════════════════════════════════════════════
    def _rcs_read_new(self) -> str:
        """只讀增量,**不完整的最後一行留到下一 tick**(檔案是邊寫邊讀的)。"""
        st = self._rcs()
        path = st["watch_unc"]
        if not path:
            return ""
        try:
            with open(path, "rb") as fh:
                fh.seek(st["offset"])
                buf = fh.read()
        except OSError:
            return ""
        if not buf:
            return ""
        cut = buf.rfind(b"\n")
        if cut < 0:
            return ""
        st["offset"] += cut + 1
        return buf[:cut + 1].decode("utf-8", errors="replace")

    def _tick_rclick_strike(self) -> None:
        st = self._rcs()
        chunk = self._rcs_read_new()
        payloads = rcs.parse_echo_payloads(chunk) if chunk else []
        status = rcs.latest_status(payloads)
        gate = rcs.latest_gate(payloads)

        if status is not None:
            st["phase"] = rcs.status_phase(status)
        now = time.time()
        elapsed = now - st["t0"]

        # ── 回讀對帳:節點自己寫下的目標 vs 我們送出去的值 ──────────
        if gate is not None and not st["verified"]:
            self._rcs_verify(gate, elapsed)

        if status is not None or (now - st["last_emit"]) >= _LOG_THROTTLE_S:
            st["last_emit"] = now
            self._rcs_say("右鍵打擊 · t+%ds · %s%s"
                          % (int(elapsed), rcs.status_text(status),
                             "" if st["verified"] else " · 目標回讀待確認"),
                          ms=3000)

        if status is not None and rcs.strike_finished(status):
            self._rcs_finish()
            return

        if elapsed > _WATCH_SOFT_LIMIT_S and not st["warned_limit"]:
            st["warned_limit"] = True
            logger.warning("[RCLICK-STRIKE] 已監看 %.0f 分鐘,超過軟上限"
                           % (elapsed / 60.0))
            QMessageBox.warning(
                self, "右鍵打擊 — 監看超過預期時間",
                "已經 %.0f 分鐘還沒看到 DONE。\n\n"
                "**沒有自動中止任何東西**:自動殺會製造不可歸因的失敗。\n"
                "要停止監看請自己來;要停飛機請走「蜂群打擊」分頁的 ABORT。"
                % (elapsed / 60.0))

    def _rcs_verify(self, gate: dict, elapsed: float) -> None:
        """『不要相信自己送出的值』—— 拿節點回報的 gate 跟送出值對帳。"""
        st = self._rcs()
        sent = st["sent"]
        echoed = rcs.gate_target(gate)
        if sent is None or echoed is None:
            return
        if rcs.target_matches(sent, echoed):
            st["verified"] = True
            logger.info("[RCLICK-STRIKE] 目標回讀驗證通過:%s"
                        % rcs.sent_vs_gate_text(sent, echoed))
            return
        # gate 是 latched 的,節點還沒處理完新目標時會先回舊值 ⇒ 給一段寬限期
        if elapsed < _READBACK_GRACE_S or st["warned_mismatch"]:
            return
        st["warned_mismatch"] = True
        self._rcs_error(
            "右鍵打擊 — ⚠️⚠️ 目標回讀不符",
            "送出的點和 tot_node 實際採用的不是同一點:\n\n"
            "    %s\n\n"
            "節點可能根本沒收到(還在用上一個目標),也可能收到了但解錯。\n"
            "這一發打的不是你點的那一點 —— 建議立刻在「蜂群打擊」分頁按 ABORT。"
            % rcs.sent_vs_gate_text(sent, echoed))

    def _rcs_stop_timer(self) -> None:
        st = self._rcs()
        timer = st["timer"]
        if timer is not None:
            try:
                timer.stop()
                timer.deleteLater()
            except Exception:
                pass
        st["timer"] = None

    def _rcs_finish(self) -> None:
        st = self._rcs()
        elapsed = time.time() - st["t0"]
        sent = st["sent"]
        self.rclick_strike_stop_watch()
        text = ("右鍵打擊結束(DONE)· 全程 %.0f 分鐘 · 目標 %s%s"
                % (elapsed / 60.0,
                   "%s, %s" % (rcs.fmt_coord(sent[0]), rcs.fmt_coord(sent[1]))
                   if sent else "(未知)",
                   "" if st["verified"] else " · ⚠️ 目標回讀始終沒對上"))
        self._rcs_say(text, ms=20000)
        QMessageBox.information(
            self, "右鍵打擊 — 結束",
            "%s\n\n命中結果請看 tot_node 的 log 與 runs/ 底下的 CSV;\n"
            "這條互動路徑不會自己產素材(那是批次路徑 make_artifacts.sh 的事)。"
            % text)
