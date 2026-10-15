"""AeroPlan 完整流程的腳本化跑法 + 全程錄幀 + 逐階段截圖。

驅動的是**真的 MainWindow**（不是替身），每一步呼叫真正會被按鈕觸發的那個
方法，所以截出來的是真實 UI 的真實狀態。

    python scripts/_full_flow_capture.py

產物：runs/artifacts/aeroplan_flow/
    frames/     全程錄幀（2 Hz），之後編成影片
    shots/      逐階段截圖
    flow.txt    每一階段的結果與耗時

⚠️⚠️ 模態對話框會讓腳本永久卡住
--------------------------------
`on_sitl_launch` 在「SITL 分頁選的載具」與「任務規劃的載具」不符時會跳
QMessageBox 等人按。腳本沒有人可以按 ⇒ 事件迴圈卡在那裡不會回來。
這裡兩層處理：
  1. 讓載具一致（COPTER + multirotor 任務），對話框根本不該出現；
  2. 仍然裝一道自動回答的守衛 —— 但**它一觸發就大聲記錄**。
     靜默的自動回答會讓「跳過了一個本來要人確認的決定」看起來像成功。
"""
from __future__ import annotations

import os
import sys
import time
import traceback
from pathlib import Path

_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_ROOT))

OUT = _ROOT / "runs" / "artifacts" / "aeroplan_flow"
FRAMES = OUT / "frames"
SHOTS = OUT / "shots"
for d in (OUT, FRAMES, SHOTS):
    d.mkdir(parents=True, exist_ok=True)
LOG = OUT / "flow.txt"

# 虎尾科大一帶 —— 專案預設作業區（config/spawn_presets.py HUWEI），
# 對應 huwei_dem.tif，所以地形跟隨與 3D 有真的高程可用。
HOME_LAT, HOME_LON = 23.702732, 120.419333
AREA = [                      # 約 600 x 450 m 的作業區
    (HOME_LAT + 0.0020, HOME_LON - 0.0030),
    (HOME_LAT + 0.0020, HOME_LON + 0.0030),
    (HOME_LAT - 0.0020, HOME_LON + 0.0030),
    (HOME_LAT - 0.0020, HOME_LON - 0.0030),
]
TARGET = (HOME_LAT + 0.0035, HOME_LON + 0.0045)   # 打擊目標（作業區東北）
N_DRONES = 3

_lines: list[str] = []
_modal_hits: list[str] = []


def say(msg: str = "") -> None:
    print(msg, flush=True)
    _lines.append(msg)
    LOG.write_text("\n".join(_lines) + "\n", encoding="utf-8")


# ── 事件迴圈 ─────────────────────────────────────────────────────────
_app = None
_win = None
_frame_n = 0


def pump(seconds: float) -> None:
    """轉事件迴圈並持續錄幀。

    ⚠️ 不能用 time.sleep()：事件迴圈不轉的話 WebEngine 不會畫任何東西，
       截出來會是上一幀的殘影。
    """
    global _frame_n
    end = time.time() + seconds
    last = 0.0
    while time.time() < end:
        _app.processEvents()
        now = time.time()
        if now - last >= 0.5:                     # 2 Hz 錄幀
            last = now
            try:
                _win.grab().save(str(FRAMES / ("f%05d.png" % _frame_n)))
                _frame_n += 1
            except Exception:
                pass
        time.sleep(0.01)


def shot(name: str) -> None:
    pump(1.2)
    p = SHOTS / (name + ".png")
    _win.grab().save(str(p))
    say("    截圖 %s.png" % name)


def stage(idx: int, title: str, fn, settle: float = 3.0) -> bool:
    """跑一個階段。失敗不中斷 —— 後面的階段照跑，最後誠實列出哪些沒過。

    ⚠️⚠️ 「沒有丟例外」**不等於**成功。這個 App 大量把錯誤轉成 QMessageBox
       就 return，腳本這邊完全看不到。所以這一段期間只要跳出 warning 或
       critical，就算這個階段沒過 —— 實測踩過：蜂群打擊跳「目標未設定」直接
       return，而階段還是記成「完成」。
    """
    say("")
    say("── [%d] %s" % (idx, title))
    t0 = time.time()
    n_before = len(_modal_hits)
    ok = True
    try:
        fn()
    except Exception as e:
        ok = False
        say("    !! 失敗：%s: %s" % (type(e).__name__, e))
        for ln in traceback.format_exc().splitlines()[-4:]:
            say("       " + ln)
    pump(settle)
    bad = [m for m in _modal_hits[n_before:]
           if m.startswith("warning") or m.startswith("critical")]
    if bad and ok:
        ok = False
        say("    !! 期間跳出 %d 個 warning/critical -> 這一階段不算過" % len(bad))
    say("    %s（%.1f s）" % ("完成" if ok else "未完成", time.time() - t0))
    return ok


# ── 主流程 ───────────────────────────────────────────────────────────
def main() -> int:
    global _app, _win

    from PyQt6.QtCore import Qt
    from PyQt6.QtWidgets import QApplication, QMessageBox
    from PyQt6.QtWebEngineWidgets import QWebEngineView  # noqa: F401

    QApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts, True)
    _app = QApplication(sys.argv)
    try:
        from ui.resources.aeroplan_theme import apply_theme
        apply_theme(_app)
    except Exception:
        pass

    # ⚠️ 模態守衛：一觸發就記錄。看不見的自動回答比卡住更糟。
    _orig_q = QMessageBox.question
    _orig_i = QMessageBox.information
    _orig_w = QMessageBox.warning

    def _guard(kind, orig_default):
        def f(*a, **k):
            txt = " / ".join(str(x)[:70] for x in a[1:3])
            _modal_hits.append("%s: %s" % (kind, txt))
            say("    ⚠️ 自動回答了一個 %s 對話框：%s" % (kind, txt))
            return orig_default
        return f

    # ⚠️⚠️ 真正的卡點不是 QMessageBox，是 `SITLLaunchDialog.get_config()` ——
    #    一個**自訂**模態，我第一版的 QMessageBox 守衛完全蓋不到它，於是
    #    腳本在「啟動 SITL」那一步永久卡住（實測卡了 8 分鐘沒有任何輸出）。
    #    這裡照它的真實契約（collect_result，見 ui/dialogs/sitl_launch_dialog.py:371）
    #    回一份等價設定：sysid 1..N、port 14550+10i、不覆寫生成點
    #    （讓 DCCPP 的每架獨立起飛點優先）。
    from ui.dialogs import sitl_launch_dialog as _sld

    def _fake_get_config(vehicle, count, parent=None):
        cfg = {
            "enable_fanout": False,
            "auto_firewall": False,
            "save_as_default": False,
            "home_preset": "",
            "home_override": None,        # None = 不覆寫，DCCPP 起飛點優先
            "instances": [
                {"sysid": i + 1, "embedded_ip": "", "embedded_port": 14550 + 10 * i}
                for i in range(count)
            ],
        }
        _modal_hits.append("SITLLaunchDialog: %s x%d" % (vehicle, count))
        say("    ⚠️ 自動填了 SITL 啟動對話框：%s x%d，port %s"
            % (vehicle, count, [d["embedded_port"] for d in cfg["instances"]]))
        return cfg

    _orig_cfg = _sld.SITLLaunchDialog.get_config
    _sld.SITLLaunchDialog.get_config = staticmethod(_fake_get_config)

    # ⚠️ 第三個模態來源：QFileDialog.getSaveFileName（匯出航點）。
    #    main_window.py 有 5 個呼叫點。給它一個固定路徑，並記錄匯到哪。
    from PyQt6.QtWidgets import QFileDialog
    _orig_save = QFileDialog.getSaveFileName
    _export_path = OUT / "exported_mission.waypoints"

    def _fake_save(*a, **k):
        _modal_hits.append("QFileDialog: %s" % _export_path.name)
        say("    ⚠️ 自動填了另存對話框 -> %s" % _export_path)
        return (str(_export_path), "QGC WPL (*.waypoints)")

    QFileDialog.getSaveFileName = staticmethod(_fake_save)

    QMessageBox.question = _guard("question", QMessageBox.StandardButton.No)
    QMessageBox.information = _guard("information", QMessageBox.StandardButton.Ok)
    QMessageBox.warning = _guard("warning", QMessageBox.StandardButton.Ok)

    from ui.main_window import MainWindow
    say("=" * 62)
    say(" AeroPlan 完整流程  %s" % time.strftime("%F %T"))
    say("=" * 62)

    t0 = time.time()
    _win = MainWindow()
    _win.resize(1920, 1080)
    _win.show()
    say("視窗建構 %.1f s" % (time.time() - t0))
    pump(12.0)                      # 等 Leaflet / Cesium 載完
    shot("00_startup")

    pp = _win.parameter_panel
    results: list[tuple[str, bool]] = []

    # ① 標定作業區 ---------------------------------------------------
    def s1():
        for i, (la, lo) in enumerate(AREA, 1):
            _win.on_manual_corner_added(la, lo)
            say("    角點 %d: %.6f, %.6f" % (i, la, lo))
            pump(0.6)
        say("    共 %d 個角點" % len(_win.corners))
        assert len(_win.corners) >= 3, "角點不足 3 個，後面全部不會動"
    results.append(("標定作業區", stage(1, "標定作業區（4 個角點）", s1)))
    shot("01_area")

    # ② 飛行參數 -----------------------------------------------------
    def s2():
        before = pp.get_parameters()
        say("    目前載具 = %s" % before.get("vehicle_type"))
        pp.set_parameters({
            "vehicle_type": "multirotor",   # 與稍後的 SITL COPTER 一致
            "altitude": 80.0,
            "speed": 12.0,
        })
        # ⚠️ 舊版這裡送 {"overlap": 30.0}。這個 key 在整個 repo 沒有定義：
        # ParameterPanel.parameters 沒有它，set_parameters 認不得就整條跳過，
        # 然後 get_parameters().get("overlap") 回 None，整趟流程照樣報成功
        # ——一支不可能響的儀器。
        # 也**不要**順手改成寫進 dccpp_overlap_spin：那顆是 overlap_rate
        # （**橫向**條帶間距、比例 0.0–0.5），而依本 repo 命名，帶百分比的
        # overlap_percent 是**航向**重疊（camera_model.trigger_dist）。
        # 沒定義的 key 沒有軸向可言，硬塞一個值等於自己決定它是哪一個。
        # 所以這裡只**讀回**規劃真正會用到的值，而且用中括號——key 名再改就直接炸。
        after = pp.get_parameters()
        say("    設定後 載具=%s 高度=%s 速度=%s 重疊率=%s"
            % (after.get("vehicle_type"), after.get("altitude"),
               after.get("speed"), pp.get_dccpp_params()["overlap_rate"]))
    results.append(("飛行參數", stage(2, "設定飛行參數", s2, settle=1.5)))

    # ③ 預覽路徑 -----------------------------------------------------
    def s3():
        _win.on_preview_paths()
    results.append(("預覽路徑", stage(3, "預覽覆蓋路徑", s3, settle=6.0)))
    shot("02_coverage_path")

    # ④ 3D Cesium ----------------------------------------------------
    def s4():
        # ⚠️ 屬性叫 map_widget（DualMapWidget），不是 dual_map。
        dm = _win.map_widget
        ok = dm.ensure_3d()
        say("    ensure_3d() -> %s，is_3d=%s" % (ok, dm.is_3d()))
        if not ok:
            raise RuntimeError("切不到 3D Cesium")
    results.append(("3D 檢視", stage(4, "切到 3D Cesium", s4, settle=9.0)))
    shot("03_cesium_3d")
    # ⚠️ QWidget.grab() 對 Cesium 的 WebGL 畫布**不可信**：內容由另一個 GPU
    #    行程合成，視窗不在前景時 Chromium 會節流，抓到的是舊幀 —— 而且看起來
    #    像成功。Cesium 自己的 capture_png 會推時鐘 + 強制 render 再取畫布。
    try:
        done = {}
        _win.map_widget.map_3d.capture_png(
            str(SHOTS / "03b_cesium_canvas.png"),
            lambda ok, *a: done.update(ok=ok))
        for _ in range(40):
            pump(0.5)
            if "ok" in done:
                break
        pth = SHOTS / "03b_cesium_canvas.png"
        sz = pth.stat().st_size if pth.exists() else 0
        say("    Cesium capture_png -> ok=%s, %d bytes" % (done.get("ok"), sz))
        if sz == 0:
            say("    ⚠️ 0 bytes —— 畫布擷取失敗（3D 那張以 grab() 為準）")
    except Exception as e:
        say("    ⚠️ capture_png 不可用：%s" % e)

    # ⑤ DCCPP --------------------------------------------------------
    def s5():
        params = pp.get_dccpp_params()
        params["drone_count"] = N_DRONES
        params["vehicle_type"] = "multirotor"
        say("    DCCPP 參數 keys = %s" % sorted(params)[:10])
        _win._on_dccpp_coverage_requested(params)
        pump(8.0)
        # ⚠️ _on_dccpp_coverage_requested 把**每一個**例外都接住轉成對話框，
        #    所以「沒有丟例外」完全不代表成功。要驗真正的產物。
        res = getattr(_win, "_dccpp_result", None)
        if not res:
            raise RuntimeError("_dccpp_result 是空的 —— DCCPP 實際上沒有產出")
        # ⚠️ 第一版查的是 res["paths"] / res["sub_paths"] —— 那兩個 key
        #    **根本不存在**，所以不管 DCCPP 成不成功都印「0 條子路徑」。
        #    真正的欄位見 ui/main_window.py:5220-5223。恆為 0 的儀器比沒有
        #    儀器更糟：它看起來像量到了。
        say("    _dccpp_result keys = %s" % sorted(res))
        assembled = res.get("assembled_paths") or {}
        results_d = res.get("results") or {}
        alloc = res.get("allocation") or {}
        say("    assembled_paths %d 架、results %d 架、allocation %d 架"
            % (len(assembled), len(results_d), len(alloc)))
        say("    makespan=%s  cruise=%s m/s"
            % (res.get("total_makespan"), res.get("cruise_speed_mps")))
        if not (assembled or results_d):
            raise RuntimeError("DCCPP 沒有產出任何一架的路徑")
    results.append(("DCCPP", stage(5, "DCCPP %d 機協同最佳化" % N_DRONES,
                                   s5, settle=12.0)))
    shot("04_dccpp")

    # ⑥ 啟動 SITL ----------------------------------------------------
    def s6():
        _win.on_sitl_launch("COPTER", N_DRONES)
    results.append(("啟動 SITL", stage(6, "啟動 %d 台 SITL" % N_DRONES,
                                       s6, settle=25.0)))
    shot("05_sitl_launched")

    # ⑦ 等遙測 -------------------------------------------------------
    def s7():
        deadline = time.time() + 60
        seen = 0
        while time.time() < deadline:
            pump(2.0)
            # ⚠️ 是 _sitl_links（list），不是 sitl_links
            seen = len(getattr(_win, "_sitl_links", None) or [])
            if seen >= N_DRONES:
                break
        say("    已連線 %d / %d" % (seen, N_DRONES))
        if seen == 0:
            raise RuntimeError("沒有任何 SITL 連上（看 flow.txt 上面的錯誤）")
    results.append(("SITL 連線", stage(7, "等待 SITL 遙測", s7, settle=6.0)))
    shot("06_telemetry")

    # ⑧ 上傳任務 -----------------------------------------------------
    def s8():
        _win.on_sitl_upload_mission()
    results.append(("上傳任務", stage(8, "上傳任務到各機", s8, settle=12.0)))
    shot("07_mission_uploaded")

    # ⑨ 蜂群打擊 -----------------------------------------------------
    def s9():
        # ⚠️ pp.set_strike_target_coord() 只動面板的 spin box。真正被檢查的是
        #    **主視窗**上的 _strike_targets / _strike_launch_base（見
        #    ui/controllers/strike_controller.py:190-199）。第一版只設了面板，
        #    於是執行時跳「目標未設定」warning 就 return —— 而我的階段檢查
        #    只看有沒有丟例外，把它記成「完成」。
        pp.set_strike_target_coord(*TARGET)
        _win._on_strike_target_coord_input(*TARGET)          # -> _strike_targets
        _win._strike_launch_base = (HOME_LAT, HOME_LON)      # 共用發射基地
        pump(2.0)
        say("    目標 %d 個 = %s" % (len(_win._strike_targets),
                                     _win._strike_targets))
        say("    發射基地 = %s" % (_win._strike_launch_base,))
        assert len(_win._strike_targets) == 1, "目標沒有進到 _strike_targets"
        params = pp.get_strike_formation_params()
        say("    編隊參數 = %s" % sorted(params)[:8])
        pp.strike_execute_requested.emit(params)
    results.append(("蜂群打擊", stage(9, "設定目標並執行蜂群打擊",
                                      s9, settle=15.0)))
    shot("08_swarm_strike")

    # ⑩ 匯出航點 -----------------------------------------------------
    def s10():
        _win.on_export_waypoints()
        pump(2.0)
        if _export_path.exists():
            n = sum(1 for _ in _export_path.open(encoding="utf-8", errors="replace"))
            say("    匯出檔 %d 行，%d bytes"
                % (n, _export_path.stat().st_size))
        else:
            raise RuntimeError("匯出對話框回了路徑，但檔案沒有產生")
    results.append(("匯出航點", stage(10, "匯出 QGC WPL 航點", s10, settle=4.0)))
    shot("09_export")

    # ── 收尾 ────────────────────────────────────────────────────────
    say("")
    say("=" * 62)
    say(" 結果")
    say("=" * 62)
    for nm, ok in results:
        say("  %s  %s" % ("✅" if ok else "❌", nm))
    n_ok = sum(1 for _, o in results if o)
    say("")
    say("  %d / %d 階段完成" % (n_ok, len(results)))
    say("  錄幀 %d 張 -> %s" % (_frame_n, FRAMES))
    say("  截圖 -> %s" % SHOTS)
    if _modal_hits:
        say("")
        say("  ⚠️ 這一趟自動回答了 %d 個對話框（本來需要人決定）："
            % len(_modal_hits))
        for m in _modal_hits[:8]:
            say("     - " + m)

    _sld.SITLLaunchDialog.get_config = _orig_cfg
    QFileDialog.getSaveFileName = _orig_save
    QMessageBox.question = _orig_q
    QMessageBox.information = _orig_i
    QMessageBox.warning = _orig_w
    try:
        _win.close()
    except Exception:
        pass
    say("FLOW_DONE")
    return 0 if n_ok == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
