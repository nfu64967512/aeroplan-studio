"""情境驅動的 HARVEST SITL 實跑（規劃 → 匯出 → 起 SITL → 上傳 → 飛 → 收資料）。

    python scripts/hv_sitl_run.py list                 # 列出情境
    python scripts/hv_sitl_run.py plan  gui_uniform_1a3b
    python scripts/hv_sitl_run.py fly   gui_uniform_1a3b

    # SITL S1（航測建圖）：旋鈕全部預設關閉，見 `survey_knobs`；飛完用 survey_sitl_s1.py 分析
    HV_TRIAL=S1_xxx HV_MAPPING=1 HV_CAMERA=hq6 HV_SHUTTER_HZ=2 HV_SIDE_OVERLAP=0.7 \\
        HV_FWD_OVERLAP=0.8 python scripts/hv_sitl_run.py fly gui_legal_120_1a2b

與 `scripts/hv_sitl_flight.py` 的關係
------------------------------------
那一支是 HV4SITL1/2 用的，機隊與作業區寫死在模組層，**刻意不動它**，
好留下一條可重現的基準線。這一支把「情境」抽出來（見 `hv_sitl_scenarios.py`），
並支援兩條規劃路徑；低階的 SITL 參數檔產生器與行程收拾直接沿用那一支。

每一步的守則（都是實測踩出來的，不要省）
--------------------------------------
1. `--defaults` 鏈最後兩份必須是 `vtol_enforced.parm`（ARMING_REQUIRE=1，否則
   四軸馬達停在 PWM 1000）與該機自己的 `<UAV>.param`。
2. **讀回**每一個關鍵參數再比對：名字打錯或韌體改名時 ArduPilot 靜默忽略。
3. 上傳任務後**下載回來**比對項數。
4. ⚠⚠ ArduPlane 4.8 **不執行** NAV_DELAY ⇒ 錯開起飛必須由地面站做：
   任務檔用 `takeoff_delay="operator"`，每架各自等到自己的 t_takeoff_s 才切 AUTO。
5. ⚠⚠ 先解鎖再切 AUTO。在 AUTO 但未解鎖時任務狀態機已經在跑。
6. 時間一律取 `GLOBAL_POSITION_INT.time_boot_ms`（模擬時鐘），不是牆鐘。
"""
from __future__ import annotations

import contextlib
import csv
import dataclasses
import json
import logging
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from scripts.hv_sitl_flight import (  # noqa: E402
    BINARY, HOME_ALT_MSL, SITL_DIR, _instance_dir, _write_enforced,
    _write_firmware_override, _write_identity,
)
from scripts.hv_sitl_scenarios import HOME, SCENARIOS, Scenario  # noqa: E402

SPEEDUP = float(os.environ.get("HV_SPEEDUP", "5"))
#: 每條鏈路每輪的遙測輪詢逾時（s）。這是 flight.csv 取樣率的**硬上限**：
#: 四架時迴圈週期 ≈ 4×_POLL_S，再乘上 SPEEDUP 就是模擬時間的取樣間隔。
#: ⚠ 0.25（舊值）⇒ 5.00 s 模擬 ⇒ HERO 一格飛 95 m，而水平門檻是 100 m
#:   ⇒ 兩機可以從取樣縫隙穿過去，「違規 0 筆」與真的安全長得一樣。
_POLL_S = float(os.environ.get("HV_POLL_S", "0.05"))
# ── SITL 風場（C2）──────────────────────────────────────────────────
#   ⚠⚠⚠ 規劃層的風在 `Scenario.wind`（預設 5 m/s／270°），而 SITL 端到 2026-09-14
#   為止**從來沒有設過 SIM_WIND_***（見 runs/artifacts/*/sitl_cmdlines.txt）⇒
#   「含風規劃」是被「無風模擬」驗證的。這兩個量必須分開給、分開讀回。
#   方向約定：SIM_WIND_DIR 與 hv_dccpp 的 `dir_from_deg` 同一個約定，都是氣象
#   「來向」（ArduPilot SITL.cpp: "Direction simulated wind is coming from"）
#   ⇒ 270 = 西風，吹向東。
#   SIM_WIND_T=1（none）：風不隨高度衰減，與規劃的單一 0–3000 m AGL 風層一致；
#   預設 0（square law）會讓 AGL < SIM_WIND_T_ALT(60 m) 的風乘上 sqrt(agl/60)。
#   SIM_WIND_TURB 一律 0：湍流走 rand() ⇒ 會打掉 SITL 的決定性。
SIM_WIND_SPD = float(os.environ.get("HV_SIM_WIND_SPD", "0"))
SIM_WIND_DIR = float(os.environ.get("HV_SIM_WIND_DIR", "270"))
SIM_WIND_TURB = float(os.environ.get("HV_SIM_WIND_TURB", "0"))
SIM_WIND_T = float(os.environ.get("HV_SIM_WIND_T", "1"))
# ── 生成點容差（HV_SPAWN_TOL_M）──────────────────────────────────────
#   ⚠⚠⚠ 這道閘門要抓的是「規劃分了逐機起降站、SITL 卻全部生在 HOME」。那種不一致
#   是 **10–20 m 級、而且會破壞兩兩間距**的（間距直接塌成 0）。2.0 m 是它的預設值，
#   **不可以為了讓某一趟過關而改掉預設**（RUNBOOK §8 明列為禁止事項）。
#   `HV_SPAWN_TOL_M` 是一個**必須明寫在指令列上**的覆寫，不是新的預設：
#     不給 ⇒ 2.0 m，行為與覆寫前逐位元相同；
#     給了 ⇒ 終端印大聲警告，且 `spawn_readback.json` 留下
#            `tol_m` / `tol_default_m` / `tol_overridden` / `tol_source` 四欄，
#            事後看產物就知道這一趟是放寬過的，不會靜靜地過。
#   ⚠ 覆寫容差**不會**放掉閘門真正的功能：逐機位移向量、兩兩間距（規劃 vs 實得）
#     與剛體平移判定是**另一組、與容差無關**的量，永遠會被計算並寫進產物；
#     「規劃分站、SITL 沒分」會讓間距誤差爆掉，放寬容差擋不住它。
SPAWN_TOL_DEFAULT_M = 2.0
_SPAWN_TOL_ENV = os.environ.get("HV_SPAWN_TOL_M")
SPAWN_TOL_M = float(_SPAWN_TOL_ENV) if _SPAWN_TOL_ENV else SPAWN_TOL_DEFAULT_M
if not (SPAWN_TOL_M > 0.0):
    raise SystemExit(f"HV_SPAWN_TOL_M 必須 > 0，收到 {_SPAWN_TOL_ENV!r}")
#: 剛體平移判定：兩兩間距（實得）與規劃間距的最大差。**與 `HV_SPAWN_TOL_M` 無關**，
#: 不提供環境變數覆寫 —— 這一項才是閘門存在的理由。
SPAWN_RIGID_TOL_M = 0.5
# ── SITL 起始埠 ──────────────────────────────────────────────────────
#   ⚠⚠⚠ ArduPlane 的 `--instance N` 是「所有埠都加 10·N」，起始 5760。
#   這台機器上另一個 session 會跑 WSL 的 SITL 群，`wslrelay.exe` 把
#   5760/5770/5780/5790 從 WSL 轉發到 Windows localhost ⇒ 撞到時我們的
#   ArduPlane 起得來，但 client 連到的是**別人的載具**，症狀是
#   「90 s 內無載具心跳」，看起來像我們的 SITL 壞了（HV4SITL10 第一次就這樣）。
#   ⇒ 預設整段錯開到 **6760**（instance 100），與標準埠段完全分離。
_SITL_PORT0 = 5760                       # ArduPlane 的內建起始埠，不可改
BASE_PORT = int(os.environ.get("HV_BASE_PORT", "6760"))
if BASE_PORT < _SITL_PORT0 or (BASE_PORT - _SITL_PORT0) % 10:
    raise SystemExit(f"HV_BASE_PORT 必須 ≥ {_SITL_PORT0} 且為 10 的倍數偏移，收到 {BASE_PORT}")
INSTANCE_OFFSET = int(os.environ.get("HV_INSTANCE_OFFSET",
                                     str((BASE_PORT - _SITL_PORT0) // 10)))
ALTERNATES = [(23.972571, 120.90)]

# ⚠⚠⚠ 2026-09-14：這裡原本是一份**寫死的 17 個名字**的白名單 `CHECK_PARAMS`，
#   加上一個放在聯集**右邊**的 `EXPECT_EXTRA`。兩個缺陷同時存在：
#
#   (1) 涵蓋率：匯出的 `plan/<uav>.param` 有 **32** 個參數，白名單只點名其中 15 個
#       （另外 2 個白名單項不在匯出檔）。17 個從來沒被比對過，其中包含
#       `WP_RADIUS`（影響航跡）、`FENCE_*`、`FS_LONG_*`、`BATT_FS_*`（失效保護）、
#       `Q_LAND_FINAL_ALT`（NAV_VTOL_LAND 的 param7=0 時由它決定最終進場高度）。
#   (2) 不可見：`want = 檔案值 | EXPECT_EXTRA` ⇒ 常數蓋掉檔案值，而 CSV 只寫
#       `expected=0.0, actual=0.0, ok=True, note=""`。`Q_RTL_MODE` 匯出的是 **1**，
#       這個「匯出 1、實飛 0」的分歧在產物裡**完全看不見**，讀的人會以為單純相符。
#       ⚠ 訂正（2026-09-14）：這個覆寫**本身是對的**，不是隨手寫死的值 ——
#       `hv_sitl_flight._write_firmware_override` 記著 ArduPlane 4.8.0-dev 的 PreArm
#       會擋「RTL_AUTOLAND 與 Q_RTL_MODE 同時開」，而任務含 DO_LAND_START 所以
#       必須保留 RTL_AUTOLAND=2、只能關掉 Q_RTL_MODE。缺陷在於它**不可見**，不在於它錯。
#
#   ⇒ 現在改成三件事：
#     (a) 比對**匯出檔的每一個**參數，不再有白名單；
#     (b) 檔案值優先於 `EXPECT_FALLBACK`（後者只對不經匯出檔的參數生效）；
#     (c) 刻意的覆寫集中在 `FIRMWARE_OVERRIDE`，**每筆都必須帶理由**，
#         理由與被覆寫掉的匯出值一起寫進 CSV 的 note 欄 ⇒ 分歧看得見。
EXPECT_FALLBACK = {"Q_ENABLE": 1.0, "ARMING_REQUIRE": 1.0, "SCHED_LOOP_RATE": 200.0,
                   "RTL_AUTOLAND": 2.0}

#: 刻意的韌體相容性覆寫：匯出檔說 A，我們**明知故犯**讓飛機跑 B。
#: 值是 (實際期望值, 理由)。理由是必填 —— 沒有理由的覆寫就是上面那個缺陷本身。
FIRMWARE_OVERRIDE: dict[str, tuple[float, str]] = {
    "Q_RTL_MODE": (0.0,
                   "ArduPlane 4.8.0-dev PreArm 不接受 RTL_AUTOLAND 與 Q_RTL_MODE 同時開；"
                   "任務含 DO_LAND_START ⇒ 必須保留 RTL_AUTOLAND=2 而關掉 Q_RTL_MODE。"
                   "失效保護的 RTL 改走 DO_LAND_START 序列（本任務該段即垂直降落）。"
                   "見 hv_sitl_flight._write_firmware_override"),
}


def expected_params(from_file: dict[str, float]) -> dict[str, tuple[float, str, str]]:
    """參數閘門的**唯一**決策點：{參數名: (期望值, 來源, note)}。

    抽成獨立函式是為了能在**沒有載具**的情況下測試它 —— 這一段的歷史缺陷
    （白名單涵蓋率、常數蓋掉檔案值）都是決策邏輯的錯，不是讀回通訊的錯。

    規則：
    * 比對集合 = 匯出檔的每一個 + :data:`EXPECT_FALLBACK` 裡不在匯出檔的那些。
    * **檔案值優先**（``EXPECT_FALLBACK`` 在聯集左邊）。
    * :data:`FIRMWARE_OVERRIDE` 的項目改用覆寫值，並把「被蓋掉的匯出值 + 理由」寫進 note。

    :raises SystemExit: 有匯出參數沒進比對集合（＝白名單缺陷復發）。
    """
    want = EXPECT_FALLBACK | from_file          # ⚠ 順序不可顛倒：檔案值必須在右邊
    missed = sorted(set(from_file) - set(want))
    if missed:
        raise SystemExit(f"有匯出參數沒進比對集合 {missed}")
    out: dict[str, tuple[float, str, str]] = {}
    for name in sorted(want):
        exp = want[name]
        src = "匯出檔" if name in from_file else "EXPECT_FALLBACK"
        note = "" if name in from_file else "不在匯出檔，用常數備援"
        if name in FIRMWARE_OVERRIDE:
            ov, why = FIRMWARE_OVERRIDE[name]
            if not why.strip():
                raise SystemExit(f"FIRMWARE_OVERRIDE[{name}] 沒有理由 —— 沒有理由的覆寫"
                                 f"就是 2026-09-14 修掉的那個缺陷本身")
            note = f"刻意覆寫：匯出 {exp:g} → 期望 {ov:g}；{why}"
            exp, src = ov, "FIRMWARE_OVERRIDE"
        out[name] = (exp, src, note)
    return out


# ══════════════════════════════════════════════════════════════════════
#  航測建圖／相機旋鈕（SITL S1）
# ══════════════════════════════════════════════════════════════════════
#   ⚠⚠ 全部預設關閉：一個都沒給 ⇒ `survey_knobs()` 回 None ⇒ 規劃、匯出、上傳與舊版逐位元相同
#     （export_all／plan_to_mission_items 不帶 camera_cfg、不寫 survey_knobs.json）。
#   任何一個給了 ⇒ 整組 S1 相機設定生效：
#     HV_MAPPING=1     建圖模式（MappingModeConfig；引入段 HV_LEAD_IN_M，預設 50 m）
#     HV_CAMERA        hq6｜hq8｜cm3｜gs8｜panel（預設 panel ＝ 面板 HFOV/VFOV 反推的相機）
#     HV_SIDE_OVERLAP  側向重疊（覆寫情境的 overlap_rate）
#     HV_FWD_OVERLAP   前向重疊（覆寫寫死的 0.70；注入點與 phase-1 planner_experiment 相同）
#     HV_SHUTTER_HZ    相機最高拍照率：規劃器 max_shutter_rate_hz 與 CAM1_INTRVAL_MIN 由同一個數而來
#     HV_CAM_MAX_ROLL  CAM_MAX_ROLL（度，預設 15）
#     HV_CROSS=1       交叉航帶（MappingModeConfig.cross_strips；**需要 HV_MAPPING=1**；預設關）
#       HV_CROSS_FRACTION    面積比例 (0, 1]（預設 0.2）；HV_CROSS_ALT_OFFSET  高度差 m（預設 0）
#       ⚠ 只給 HV_CROSS=0（或都不給）＝ 沒有交叉航帶，與舊版逐位元相同；比例／高度差沒開 HV_CROSS=1 卻給了 ⇒ 拒絕
#   匯出用 CameraTriggerConfig(servo, 0.1 s, INTRVAL_MIN = ceil_0.02(1/HV_SHUTTER_HZ), AUTO_ONLY,
#   MAX_ROLL, camera_lead_in_m = 引入段)。SITL 沒有 relay GPIO；servo 後端在 SITL 已實測會寫 CAM。
#   ⚠ 只支援 path='gui' 的情境（direct 路徑的酬載由 payload_reqs 決定，不注入）。
#   ⚠ 有效值契約：規劃後**讀回**規劃器真正用的重疊／拍照率／相機／建圖設定，與旋鈕不一致就停，
#     全部寫進 `survey_knobs.json`；飛完再補 SITL 日誌路徑（給 scripts/survey_sitl_s1.py 用）。
#: (焦距 mm, 感測器寬 mm, 感測器高 mm, 像素 µm)；與 phase-1 planner_experiment 的 CAMS 同值。
#: 拍照率不在這裡：一律由 HV_SHUTTER_HZ 給（不給 ⇒ CameraHardware 預設），與 INTRVAL_MIN 同源。
SURVEY_CAMERAS: dict[str, tuple[float, float, float, float] | None] = {
    "hq6": (6.0, 4056 * 1.55e-3, 3040 * 1.55e-3, 1.55),
    "hq8": (8.0, 4056 * 1.55e-3, 3040 * 1.55e-3, 1.55),
    "cm3": (4.74, 4608 * 1.4e-3, 2592 * 1.4e-3, 1.40),
    "gs8": (8.0, 1456 * 3.45e-3, 1088 * 3.45e-3, 3.45),
    "panel": None,
}
SURVEY_ENV = ("HV_MAPPING", "HV_LEAD_IN_M", "HV_CAMERA", "HV_SIDE_OVERLAP", "HV_FWD_OVERLAP",
              "HV_SHUTTER_HZ", "HV_CAM_MAX_ROLL")
SURVEY_LEAD_IN_DEFAULT_M = 50.0
#: 交叉航帶旋鈕（另列：只給 HV_CROSS=0 不算「有給旋鈕」⇒ survey_knobs() 仍回 None）
SURVEY_CROSS_ENV = ("HV_CROSS", "HV_CROSS_FRACTION", "HV_CROSS_ALT_OFFSET")
SURVEY_CROSS_FRACTION_DEFAULT = 0.2
SURVEY_CROSS_ALT_OFFSET_DEFAULT_M = 0.0
SURVEY_CAM_MAX_ROLL_DEFAULT_DEG = 15.0
#: LOG_* 刻意不動（寫進 survey_knobs.json，讀產物的人才知道）。
SURVEY_LOGGING_NOTE = ("LOG_* 參數沒有改：沿用 SITL 預設（LOG_DISARMED=0 ⇒ 未解鎖時的拍照不寫 CAM；"
                       "S1 的照片全在 AUTO 解鎖中拍，不受影響。地面試拍 F0 要另外處理）")


def _ceil_002(x: float) -> float:
    """向上取整到 0.02 s（評估 §4：CAM1_INTRVAL_MIN = 1/實測頻率，向上取整到 0.02 s）。"""
    return round(math.ceil(round(x / 0.02, 9)) * 0.02, 2)


#: SITL 匯出用的 CAM1_DURATION（servo 後端）
SURVEY_TRIGGER_DURATION_S = 0.1

#: 首飛建圖風速上限（2026-10-15 使用者定調 ≤ 3 m/s）；與 mission.swarm_coordinator.MAPPING_MAX_WIND_DEFAULT_MPS
#: 同值（tests/test_survey_sitl_s1.py 鎖住）。SITL 只拿來印 GUI 閘門的結果，不擋。
MAPPING_FIRST_FLIGHT_MAX_WIND_MPS = 3.0


def _min_trigger_spacing_s(duration_s: float) -> float:
    """servo／relay 後端兩次觸發之間的最短間隔（秒）＝ (floor(DURATION/0.02) + 2) × 0.02。

    與 mission.swarm_coordinator.trigger_min_spacing_s 同一條公式（GUI 路徑用那支；這裡不匯入它，免得
    SITL 腳本把整個協調器拖進來）。tests/test_survey_sitl_s1.py 鎖住兩者相等。"""
    n = math.floor(float(duration_s) / 0.02 + 1e-6)
    return round((n + 2) * 0.02, 6)


@dataclasses.dataclass(frozen=True)
class SurveyKnobs:
    """解析後的旋鈕（皆為有效值；預設值的來源記在 ``shutter_source`` 與 ``env``）。"""
    env: dict                    # 有給的環境變數原字串
    mapping: bool
    lead_in_m: float             # 建圖模式關閉時為 0
    camera: str
    side_overlap: float | None   # None ＝ 沿用情境的 overlap_rate
    fwd_overlap: float | None    # None ＝ 沿用規劃器（0.70）
    shutter_hz: float
    shutter_source: str
    cam_max_roll_deg: float
    interval_min_s: float        # ceil_0.02(1/shutter_hz)
    cross: bool = False                      # HV_CROSS=1（交叉航帶；需要建圖模式）
    cross_fraction: float | None = None      # 只在 cross 時有值
    cross_alt_offset_m: float | None = None  # 只在 cross 時有值

    def mapping_config(self):
        if not self.mapping:
            return None
        from hv_dccpp.mission import MappingModeConfig
        if self.cross:
            return MappingModeConfig(lead_in_m=self.lead_in_m, cross_strips=True,
                                     cross_fraction=self.cross_fraction, cross_alt_offset_m=self.cross_alt_offset_m)
        return MappingModeConfig(lead_in_m=self.lead_in_m)

    def camera_hw(self):
        """具名相機 → CameraHardware；panel → None（沿用面板反推的相機，只改拍照率）。"""
        spec = SURVEY_CAMERAS[self.camera]
        if spec is None:
            return None
        from hv_dccpp.platform import CameraHardware
        f, sw, sh, pp = spec
        return CameraHardware(focal_length_mm=f, sensor_width_mm=sw, sensor_height_mm=sh,
                              pixel_pitch_um=pp, max_shutter_rate_hz=self.shutter_hz,
                              is_default_example=False)

    def fov_deg(self) -> tuple[float, float] | None:
        spec = SURVEY_CAMERAS[self.camera]
        if spec is None:
            return None
        f, sw, sh, _pp = spec
        return (math.degrees(2.0 * math.atan(sw / 2.0 / f)), math.degrees(2.0 * math.atan(sh / 2.0 / f)))

    def trigger_config(self):
        from hv_dccpp.ardupilot import CameraTriggerConfig
        try:
            # first_shot_immediate=False：與 GUI 建圖模式相同（build_mapping_setup）；理由見那裡
            return CameraTriggerConfig(backend="servo", duration_s=SURVEY_TRIGGER_DURATION_S,
                                       interval_min_s=self.interval_min_s, auto_only=True,
                                       max_roll_deg=self.cam_max_roll_deg,
                                       camera_lead_in_m=self.lead_in_m, first_shot_immediate=False)
        except ValueError as e:
            raise SystemExit(f"相機觸發設定被拒絕：{e}") from None

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        if not self.cross:
            # 沒開交叉航帶 ⇒ 不列 cross_* 鍵（survey_knobs.json 與加入交叉航帶前逐位元相同）
            for k in ("cross", "cross_fraction", "cross_alt_offset_m"):
                d.pop(k, None)
        fc_hz = 1.0 / self.interval_min_s
        d["camera_spec"] = (None if SURVEY_CAMERAS[self.camera] is None else dict(zip(
            ("focal_length_mm", "sensor_width_mm", "sensor_height_mm", "pixel_pitch_um"),
            SURVEY_CAMERAS[self.camera])))
        d["fov_deg"] = self.fov_deg()
        d["fc_max_rate_hz"] = fc_hz
        d["notes"] = ([] if fc_hz >= self.shutter_hz - 1e-9 else [
            f"CAM1_INTRVAL_MIN 向上取整成 {self.interval_min_s:g} s ⇒ 飛控最高 {fc_hz:.4f} Hz "
            f"< 規劃用 {self.shutter_hz:g} Hz：規劃速度上限處的觸發會被飛控延後，"
            f"間距最多長 {(self.shutter_hz * self.interval_min_s - 1.0) * 100.0:.2f}%"])
        return d


def survey_knobs(env=None) -> SurveyKnobs | None:
    """讀旋鈕。全部沒給（或只給 HV_MAPPING=0）⇒ None ＝ 與舊版逐位元相同。

    :raises SystemExit: 值不合法、或給了卻不會生效（有效值契約：拒絕，不靜默忽略）。
    """
    env = os.environ if env is None else env
    given = {k: str(env[k]).strip() for k in SURVEY_ENV if str(env.get(k) or "").strip()}
    cross_given = {k: str(env[k]).strip() for k in SURVEY_CROSS_ENV if str(env.get(k) or "").strip()}
    mapping_s = given.get("HV_MAPPING", "0")
    if mapping_s not in ("0", "1"):
        raise SystemExit(f"HV_MAPPING 只能是 0 或 1，收到 {mapping_s!r}")
    mapping = mapping_s == "1"
    cross_s = cross_given.get("HV_CROSS", "0")
    if cross_s not in ("0", "1"):
        raise SystemExit(f"HV_CROSS 只能是 0 或 1，收到 {cross_s!r}")
    cross = cross_s == "1"
    if not cross and any(k != "HV_CROSS" for k in cross_given):
        raise SystemExit(f"{sorted(k for k in cross_given if k != 'HV_CROSS')} 只在 HV_CROSS=1 時有作用；"
                         "沒開交叉航帶卻給了 ⇒ 拒絕，不靜默忽略")
    if cross and not mapping:
        raise SystemExit("HV_CROSS=1 需要 HV_MAPPING=1（交叉航帶是建圖模式的一部分）")
    if not mapping and not any(k != "HV_MAPPING" for k in given):
        return None

    def cross_num(name: str, default: float, *, lo: float | None = None, hi: float | None = None) -> float:
        if name not in cross_given:
            return default
        s = cross_given[name]
        try:
            v = float(s)
        except ValueError:
            raise SystemExit(f"{name} 必須是數字，收到 {s!r}") from None
        if not math.isfinite(v) or (lo is not None and v <= lo) or (hi is not None and v > hi):
            raise SystemExit(f"{name}={s} 超出範圍 ({lo if lo is not None else '-inf'}, {hi if hi is not None else 'inf'}]")
        return v

    cross_frac = cross_num("HV_CROSS_FRACTION", SURVEY_CROSS_FRACTION_DEFAULT, lo=0.0, hi=1.0) if cross else None
    cross_dh = cross_num("HV_CROSS_ALT_OFFSET", SURVEY_CROSS_ALT_OFFSET_DEFAULT_M) if cross else None

    def num(name: str, lo: float, hi: float, *, lo_open: bool = False, hi_open: bool = False) -> float:
        s = given[name]
        try:
            v = float(s)
        except ValueError:
            raise SystemExit(f"{name} 必須是數字，收到 {s!r}") from None
        if not (math.isfinite(v) and (v > lo if lo_open else v >= lo) and (v < hi if hi_open else v <= hi)):
            raise SystemExit(f"{name}={s} 超出範圍 {'(' if lo_open else '['}{lo:g}, {hi:g}{')' if hi_open else ']'}")
        return v

    if "HV_LEAD_IN_M" in given and not mapping:
        raise SystemExit("HV_LEAD_IN_M 只在 HV_MAPPING=1 時有作用（引入段要靠建圖模式把條帶外伸）；"
                         "沒開建圖模式卻給了 ⇒ 拒絕，不靜默忽略")
    lead = (num("HV_LEAD_IN_M", 0.0, 1000.0) if "HV_LEAD_IN_M" in given
            else (SURVEY_LEAD_IN_DEFAULT_M if mapping else 0.0))
    camera = given.get("HV_CAMERA", "panel").lower()
    if camera not in SURVEY_CAMERAS:
        raise SystemExit(f"HV_CAMERA 只能是 {sorted(SURVEY_CAMERAS)}，收到 {given['HV_CAMERA']!r}")
    side = num("HV_SIDE_OVERLAP", 0.0, 1.0, lo_open=True, hi_open=True) if "HV_SIDE_OVERLAP" in given else None
    fwd = num("HV_FWD_OVERLAP", 0.0, 1.0, lo_open=True, hi_open=True) if "HV_FWD_OVERLAP" in given else None
    if "HV_SHUTTER_HZ" in given:
        # 下限 0.1 Hz ⇐ CAM1_INTRVAL_MIN 上限 10 s；上限 50 Hz ⇐ AP_Camera 50 Hz 更新
        hz, hz_src = num("HV_SHUTTER_HZ", 0.1, 50.0), "HV_SHUTTER_HZ"
    else:
        from hv_dccpp.platform import CameraHardware
        hz, hz_src = float(CameraHardware().max_shutter_rate_hz), "CameraHardware 預設"
    roll = (num("HV_CAM_MAX_ROLL", 0.0, 180.0) if "HV_CAM_MAX_ROLL" in given
            else SURVEY_CAM_MAX_ROLL_DEFAULT_DEG)
    interval = _ceil_002(1.0 / hz)
    spacing = _min_trigger_spacing_s(SURVEY_TRIGGER_DURATION_S)
    if interval < spacing - 1e-9:
        # 有效值契約：servo 後端在 CAM1_DURATION 內拒絕新觸發，再要一個週期放回 OFF ⇒ 飛控給不出這個頻率，
        # 只會靜默拉長照片間距（前向重疊不足）。拒絕，不靜默照飛
        raise SystemExit(
            f"{hz_src}={hz:g} Hz ⇒ CAM1_INTRVAL_MIN {interval:.2f} s，但 CAM1_DURATION "
            f"{SURVEY_TRIGGER_DURATION_S:g} s 時 servo 後端兩次觸發至少要隔 {spacing:.2f} s ⇒ 上限 "
            f"{1.0 / spacing:.3f} Hz；請把 HV_SHUTTER_HZ 設到 ≤ {1.0 / spacing:.3f}")
    env_rec = dict(given)
    env_rec.update(cross_given)                     # 交叉航帶旋鈕也記進 survey_knobs.json（只在有給時）
    return SurveyKnobs(env=env_rec, mapping=mapping, lead_in_m=lead, camera=camera, side_overlap=side,
                       fwd_overlap=fwd, shutter_hz=hz, shutter_source=hz_src, cam_max_roll_deg=roll,
                       interval_min_s=interval, cross=cross, cross_fraction=cross_frac,
                       cross_alt_offset_m=cross_dh)


def apply_survey_knobs(sc: Scenario, knobs: SurveyKnobs) -> Scenario:
    """情境層的旋鈕：具名相機的 FOV、側向重疊。只接受 GUI 路徑。"""
    if sc.path != "gui":
        raise SystemExit(f"建圖／相機旋鈕 {sorted(knobs.env)} 只支援 path='gui' 的情境"
                         f"（{sc.key} 是 {sc.path!r}；direct 路徑的酬載由 payload_reqs 決定，不注入）")
    rep: dict = {}
    fov = knobs.fov_deg()
    if fov is not None:
        rep.update(hfov_deg=fov[0], vfov_deg=fov[1])
    if knobs.side_overlap is not None:
        rep["overlap_rate"] = knobs.side_overlap
    return dataclasses.replace(sc, **rep) if rep else sc


@contextlib.contextmanager
def _inject_survey(knobs: SurveyKnobs | None):
    """只在這個 with 裡把相機、前向重疊、建圖模式注入 GUI 路徑（離開即還原）。

    注入點與 phase-1 planner_experiment 相同：``swarm_coordinator._uniform_camera`` 與
    ``aeroplan_adapter.HvDccppCoordinator``（swarm_coordinator 在規劃當下才 import 它）。
    """
    if knobs is None:
        yield
        return
    import hv_dccpp.integration.aeroplan_adapter as ad
    import mission.swarm_coordinator as swc
    base_coord, base_cam = ad.HvDccppCoordinator, swc._uniform_camera
    mm, fwd, hw, hz = knobs.mapping_config(), knobs.fwd_overlap, knobs.camera_hw(), knobs.shutter_hz

    class _SurveyCoordinator(base_coord):            # type: ignore[misc,valid-type]
        def __init__(self, *a, **kw):
            if mm is not None:
                if kw.get("mapping_mode") is not None and kw["mapping_mode"] != mm:
                    raise SystemExit(f"呼叫端已給 mapping_mode={kw['mapping_mode']!r}，與 HV_MAPPING 的 {mm!r} 不同")
                kw["mapping_mode"] = mm
            super().__init__(*a, **kw)
            if fwd is not None and self.payload_reqs:
                self.payload_reqs = {k: (dataclasses.replace(r, forward_overlap=fwd) if r.kind == "rgb" else r)
                                     for k, r in self.payload_reqs.items()}

    ad.HvDccppCoordinator = _SurveyCoordinator
    if hw is not None:
        swc._uniform_camera = lambda **_kw: hw
    else:
        swc._uniform_camera = lambda **kw: dataclasses.replace(base_cam(**kw), max_shutter_rate_hz=hz)
    try:
        yield
    finally:
        ad.HvDccppCoordinator, swc._uniform_camera = base_coord, base_cam


def survey_effective(hv, knobs: SurveyKnobs, cam_cfg, sc: Scenario) -> dict:
    """規劃後**讀回**規劃器真正用的值，與旋鈕逐項比對；任何一項沒到 ⇒ SystemExit（不飛）。"""
    from hv_dccpp.ardupilot import trigger_distance_m
    want_hw = knobs.camera_hw()
    bad, sensors = [], {}
    for (cls, rid), s in sorted(hv.last_ctx.sensors.items()):
        hw, r = s.hw, s.r
        if getattr(hw, "kind", None) != "rgb":
            continue
        tag = f"{cls}:{rid}"
        geom = tuple(getattr(hw, k, None) for k in
                     ("focal_length_mm", "sensor_width_mm", "sensor_height_mm", "pixel_pitch_um"))
        sensors[tag] = {"forward_overlap": r.forward_overlap, "side_overlap": r.side_overlap,
                        "shutter_rate_hz": r.shutter_rate_hz,
                        "max_shutter_rate_hz": getattr(hw, "max_shutter_rate_hz", None),
                        "camera": dict(zip(("focal_length_mm", "sensor_width_mm", "sensor_height_mm",
                                            "pixel_pitch_um"), geom)),
                        "trigger_distance_m": trigger_distance_m(s, float(sc.coverage_altitude_m)),
                        "at_agl_m": float(sc.coverage_altitude_m)}
        if knobs.fwd_overlap is not None and not math.isclose(r.forward_overlap, knobs.fwd_overlap, abs_tol=1e-9):
            bad.append(f"{tag} 前向重疊 {r.forward_overlap} ≠ HV_FWD_OVERLAP {knobs.fwd_overlap}"
                       f"（情境沒有 target_gsd 時轉接層自己合成需求，前向重疊注入不到）")
        if knobs.side_overlap is not None and (r.side_overlap is None or not math.isclose(
                r.side_overlap, knobs.side_overlap, abs_tol=1e-9)):
            bad.append(f"{tag} 側向重疊 {r.side_overlap} ≠ HV_SIDE_OVERLAP {knobs.side_overlap}")
        if r.shutter_rate_hz is None or not math.isclose(r.shutter_rate_hz, knobs.shutter_hz, abs_tol=1e-9):
            bad.append(f"{tag} 拍照率 {r.shutter_rate_hz} Hz ≠ {knobs.shutter_hz} Hz（{knobs.shutter_source}）")
        if want_hw is not None and geom != (want_hw.focal_length_mm, want_hw.sensor_width_mm,
                                            want_hw.sensor_height_mm, want_hw.pixel_pitch_um):
            bad.append(f"{tag} 相機 {geom} ≠ HV_CAMERA={knobs.camera}")
    if not sensors:
        bad.append("規劃器沒有任何 rgb 相機 ⇒ 相機旋鈕無處生效")
    mm = getattr(hv, "mapping_mode", None)
    if mm != knobs.mapping_config():
        bad.append(f"規劃器的建圖模式 {mm!r} ≠ HV_MAPPING 要的 {knobs.mapping_config()!r}")
    lead_plan = float(mm.lead_in_m) if mm is not None else 0.0
    if not math.isclose(float(cam_cfg.camera_lead_in_m), lead_plan, abs_tol=1e-9):
        bad.append(f"匯出引入段 {cam_cfg.camera_lead_in_m} m ≠ 規劃條帶外伸的 lead_in {lead_plan} m")
    if bad:
        raise SystemExit("建圖／相機旋鈕沒有照設定到達規劃器（不匯出、不飛）：\n  " + "\n  ".join(bad))
    return {"sensors": sensors, "mapping_mode": None if mm is None else mm.to_dict(),
            "mapping_effective": getattr(hv, "mapping_effective", None)}


def out_dir(sc: Scenario) -> Path:
    return ROOT / "runs" / "artifacts" / os.environ.get("HV_TRIAL", sc.trial)


# ── 只收拾自己的行程 ──────────────────────────────────────────────────
#   ⚠ `hv_sitl_flight.stop_all()` 是 `taskkill` 掉**機器上每一個** ArduPlane.exe。
#     這台機器同時有別的 session 在跑東西，一趟 SITL 不該去殺別人的。
#     所以這裡改成：只殺命令列裡帶著我們 instance 目錄的那幾個。
_INSTANCE_TAG = str((SITL_DIR / "vtol").resolve())


def _sitl_pids(tag: str = _INSTANCE_TAG) -> list[int]:
    """命令列含 `tag` 的 ArduPlane.exe PID。查不到就回空清單（不要猜）。"""
    ps = ("Get-CimInstance Win32_Process -Filter \"Name='ArduPlane.exe'\" | "
          "ForEach-Object { \"$($_.ProcessId)`t$($_.CommandLine)\" }")
    try:
        r = subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-Command", ps],
                           capture_output=True, text=True, timeout=30)
    except Exception as e:      # noqa: BLE001
        print(f"  查行程失敗（{e}），不做任何收拾")
        return []
    pids = []
    for line in r.stdout.splitlines():
        pid, _, cmd = line.partition("\t")
        if pid.strip().isdigit() and tag.lower() in cmd.lower():
            pids.append(int(pid))
    return pids


def stop_ours(pids: list[int] | None = None) -> int:
    """終止我們自己起的 SITL。`pids` 不給時用命令列比對找。"""
    before_all = set(_sitl_pids(tag="ArduPlane.exe"))     # 每一個 ArduPlane 的 PID
    targets = pids if pids is not None else _sitl_pids()
    n = 0
    for pid in targets:
        r = subprocess.run(["taskkill", "/PID", str(pid), "/F"], capture_output=True)
        n += (r.returncode == 0)
    left = len(before_all - set(targets))
    print(f"已終止 {n} 個本次的 ArduPlane 行程"
          + (f"（機器上另有 {left} 個不是我們起的，沒有動）" if left else ""))
    return n


# ══════════════════════════════════════════════════════════════════════
#  1. 規劃（兩條路徑）
# ══════════════════════════════════════════════════════════════════════
def _fence_zones(sc: Scenario):
    """情境的禁航區 → FenceZone。空的就回 [] （代表「確定沒有禁航區」而不是「沒查」）。"""
    if not sc.nfz:
        return []
    from mission.fence_zone import FenceZone, ZoneCategory, ZoneShape
    return [FenceZone(name=f"NFZ{i + 1}", category=ZoneCategory.NFZ,
                      shape=ZoneShape.POLYGON, vertices=list(v),
                      alt_min_m=0.0, alt_max_m=500.0)
            for i, v in enumerate(sc.nfz)]


def _plan_gui(sc: Scenario, knobs: SurveyKnobs | None = None):
    """AeroPlan 介面按下去真正會走的那一條。兩型共用單光相機。``knobs`` 見 :func:`survey_knobs`。"""
    from mission.swarm_coordinator import SwarmCoordinator
    cls_of = {i + 1: c for i, (_n, c, _s, _v, _r) in enumerate(sc.fleet)}
    report: dict = {}
    c = SwarmCoordinator()
    c.use_harvest = True
    c.hv_uniform_sensor = True
    c.hv_class_of = lambda d: cls_of[int(d["drone_id"])]
    c.hv_home = HOME
    c.hv_alternates = ALTERNATES
    c.hv_wind = sc.wind
    c.hv_fence_zones = _fence_zones(sc)
    c.hv_on_report = report.update
    # ⚠⚠⚠ 回收站自動指派會**遮蔽**上游的差異：PAD8 那組 A/B 裡，修好與沒修的
    #   原始計畫最小間距是 51.5 m vs **5.0 m**，但自動指派把 B3/B4 都移到 ALT1
    #   之後兩者都變成同一份計畫（889.5 s、148.2 m）⇒ 實飛完全分辨不出來。
    #   要做「豁免判準」的受控實驗就得關掉它，飛原始計畫。
    c.hv_recovery_auto = os.environ.get("HV_RECOVERY_AUTO", "1") != "0"
    if not c.hv_recovery_auto:
        print("⚠ HV_RECOVERY_AUTO=0：已關閉回收站自動指派，這一趟飛的是原始計畫")
    # ⚠ 這四個是情境自己的，不可以再寫死。`hv_waiver_max_agl_m=None`
    #   ＝ 沒有豁免 ⇒ 上游用民航法規的 120 m 當 h_ceil。
    c.harvest_params = {"hfov_deg": sc.hfov_deg, "vfov_deg": sc.vfov_deg,
                   "hv_c2_range_km": 25.0,
                   "hv_waiver_max_agl_m": sc.waiver_max_agl_m,
                   "hv_dem_margin_m": 3000.0, "hv_terrain_clearance_m": 36.0}
    if sc.min_fw_agl_m:
        c.harvest_params["hv_min_fw_agl_m"] = dict(sc.min_fw_agl_m)
    if sc.dh_band_min_m is not None:
        c.harvest_params["hv_dh_band_min_m"] = float(sc.dh_band_min_m)
    if sc.target_gsd is not None:
        c.harvest_params["target_gsd"] = float(sc.target_gsd)
    with _inject_survey(knobs):
        c.plan_coverage_dccpp(areas=sc.areas, drones=sc.drones(),
                              coverage_width_m=sc.coverage_width_m,
                              overlap_rate=sc.overlap_rate,
                              coverage_altitude=sc.coverage_altitude_m,
                              mounting_angle_deg=0.0)
    return c.hv_coordinator, report


def _plan_direct(sc: Scenario):
    """HV4SITL1/2 用的那一條：自己給 payload_reqs。"""
    from hv_dccpp.integration.aeroplan_adapter import HvDccppCoordinator
    from hv_dccpp.mission import PayloadRequirement, PlannerConfig
    reqs = {rid: PayloadRequirement(rid, **kw) for rid, kw in sc.payload_reqs.items()}
    report: dict = {}
    coord = HvDccppCoordinator(
        class_of=lambda d: str(d["hv_class"]),
        cfg=PlannerConfig(dem_step_m=25.0, t_sep_ldg_s=30.0),
        home=HOME, alternates=ALTERNATES, payload_reqs=reqs,
        wind=sc.wind, c2_range_km=25.0, terrain_clearance_m=36.0,
        waiver_max_agl_m=200.0, dem_margin_m=3000.0,
        on_report=report.update, raise_on_no_go=True)
    coord.plan_coverage_dccpp(areas=sc.areas, drones=sc.drones(),
                              coverage_width_m=sc.coverage_width_m,
                              overlap_rate=sc.overlap_rate,
                              coverage_altitude=sc.coverage_altitude_m)
    return coord, report


def plan_and_export(sc: Scenario, knobs: SurveyKnobs | None = None):
    import hv_dccpp as hvlib
    if knobs is not None:
        sc = apply_survey_knobs(sc, knobs)
    out = out_dir(sc)
    out.mkdir(parents=True, exist_ok=True)

    t0 = time.perf_counter()
    hv, report = _plan_gui(sc, knobs) if sc.path == "gui" else _plan_direct(sc)
    dt = time.perf_counter() - t0
    plan, ctx, cfg = hv.last_plan, hv.last_ctx, hv.cfg

    # ★ 閘門：規劃器可能只用機隊的一部分，而且**不會有任何訊息**。
    working = {u: len(p.strips) for u, p in plan.plans.items()}
    n_fleet = len(sc.fleet)
    if len(plan.plans) != n_fleet or not all(working.values()):
        raise SystemExit(f"不是 {n_fleet} 架都有工作：{working}"
                         f"（用 scripts/hv_sitl_scout.py 換幾何）")

    cache = hvlib.build_validation_cache(plan, ctx, budget_only=False)
    v = hvlib.validate(plan, ctx, cache, cfg)

    print(f"情境 {sc.key}（路徑 {sc.path}）：規劃 {dt:.2f}s  "
          f"makespan {plan.makespan_s:.1f}s  status {report.get('status')}")
    print(f"validate passed={v.passed} conflicts={len(v.conflicts)} "
          f"uncovered={len(plan.uncovered)}")
    vrec = {"passed": v.passed, "conflicts": len(v.conflicts), "coverage_ok": v.coverage_ok,
            "energy_ok": v.energy_ok, "min_sep_h_m": v.min_sep_h_m, "min_sep_v_m": v.min_sep_v_m}
    if knobs is not None and knobs.mapping:
        # GUI 的建圖上傳閘門（含首飛風速上限 3 m/s）在這裡**只印不擋**：SITL 情境刻意用 5 m/s 規劃風
        gate = hv.mapping_gate(max_wind_mps=MAPPING_FIRST_FLIGHT_MAX_WIND_MPS)
        vrec["gui_mapping_gate"] = {"max_wind_mps": MAPPING_FIRST_FLIGHT_MAX_WIND_MPS, **gate}
        print(f"GUI 建圖上傳閘門（風速上限 {MAPPING_FIRST_FLIGHT_MAX_WIND_MPS:g} m/s；SITL 只提示不擋）："
              f"{'可上傳' if gate['uploadable'] else '✘ ' + '；'.join(gate['reasons'])}")
    allow_conflicts = os.environ.get("HV_ALLOW_CONFLICTS", "0") == "1"
    if knobs is not None and knobs.cross and not v.passed:
        # 交叉航帶會飛進鄰機獨占區塊（同高度、只靠 4D 時間錯開）⇒ validate 不過就不飛（GUI 也會擋）
        vrec["allow_conflicts"] = allow_conflicts
        if not allow_conflicts:
            (out / "validation.json").write_text(json.dumps(vrec, ensure_ascii=False, indent=1, default=str),
                                                 encoding="utf-8")
            raise SystemExit(f"交叉航帶計畫 validate 不過（殘留衝突 {len(v.conflicts)}）⇒ 不匯出、不飛；"
                             "真的要飛請明確設 HV_ALLOW_CONFLICTS=1（會記在 validation.json）")
        print(f"⚠⚠ HV_ALLOW_CONFLICTS=1：validate 不過（衝突 {len(v.conflicts)}）仍照飛")
    kinds = {cls: sen.hw.kind for (cls, _rid), sen in ctx.sensors.items()}
    print(f"感測器 {kinds}   酬載需求 "
          f"{ {r.req_id: r.kind for r in hv.last_inputs.payload_reqs.values()} }")
    for u, p in sorted(plan.plans.items()):
        print(f"  {u} {p.cls} strips={len(p.strips)} wp={len(p.waypoints)} "
              f"{p.distance_m / 1000:.2f}km  t_off={p.t_takeoff_s:.0f}s "
              f"t_land={p.t_land_s:.0f}s  E={p.energy_used_wh:.0f}/{p.energy_margin_wh:.0f}Wh")

    cam_cfg = survey_eff = None
    if knobs is not None:
        cam_cfg = knobs.trigger_config()
        survey_eff = survey_effective(hv, knobs, cam_cfg, sc)     # 讀回；沒到就停（不匯出）
    plan_dir = out / "plan"
    if plan_dir.exists():
        shutil.rmtree(plan_dir)
    summary = hvlib.export_all(plan, ctx, cfg, plan_dir, alt_frame="relative",
                               camera_trigger=True, takeoff_delay="operator", camera_cfg=cam_cfg)
    (out / "validation.json").write_text(json.dumps(vrec, ensure_ascii=False, indent=1, default=str),
                                         encoding="utf-8")
    (out / "report.json").write_text(json.dumps(
        {k: report.get(k) for k in ("status", "errors", "warnings", "notes",
                                    "uncovered", "timing_ms")},
        ensure_ascii=False, indent=1, default=str), encoding="utf-8")
    (out / "scenario.json").write_text(json.dumps(
        {"key": sc.key, "path": sc.path, "n_a": sc.n_a, "n_b": sc.n_b,
         "altitude_m": sc.coverage_altitude_m, "hfov_deg": sc.hfov_deg,
         "vfov_deg": sc.vfov_deg, "overlap_rate": sc.overlap_rate,
         "wind": list(sc.wind), "areas": sc.areas, "nfz": sc.nfz, "note": sc.note,
         # ★ 規劃風（上一行）與**模擬風**是兩件事，產物上必須分得出來
         "sim_wind": {"spd_mps": SIM_WIND_SPD, "dir_from_deg": SIM_WIND_DIR,
                      "turb": SIM_WIND_TURB, "profile_type": SIM_WIND_T},
         "sensor_kinds": kinds, "speedup": SPEEDUP,
         # ⚠ 沒有這一欄的話，分散起降點的航次與共點航次在產物上**分不出來**
         #   （唯一的痕跡是 sitl_cmdlines.txt 裡的 --home）。
         "spawn_spread_m": sc.spawn_spread_m,
         "takeoff_sites": {u: getattr(p_, "takeoff_site", "") for u, p_ in plan.plans.items()},
         # ★ 這一趟真正生效的空域／平台條件（讀回，不是抄情境定義）
         "waiver_max_agl_m": sc.waiver_max_agl_m,
         "h_ceil_m": (120.0 if sc.waiver_max_agl_m is None else sc.waiver_max_agl_m),
         "min_fw_agl_m": {"A": hv.platform_a.min_fw_operation_agl_m,
                          "B": hv.platform_b.min_fw_operation_agl_m},
         "dh_band_min_m": cfg.dh_band_min_m,
         "target_gsd": sc.target_gsd},
        ensure_ascii=False, indent=1), encoding="utf-8")
    if knobs is not None:
        if summary.get("camera_cfg") != dataclasses.asdict(cam_cfg):
            raise SystemExit(f"匯出器讀回的 camera_cfg {summary.get('camera_cfg')} ≠ 送進去的 "
                             f"{dataclasses.asdict(cam_cfg)}")
        (out / "survey_knobs.json").write_text(json.dumps(
            {"knobs": knobs.to_dict(), "camera_trigger_config": dataclasses.asdict(cam_cfg),
             "scenario_effective": {"key": sc.key, "hfov_deg": sc.hfov_deg, "vfov_deg": sc.vfov_deg,
                                    "overlap_rate": sc.overlap_rate,
                                    "coverage_altitude_m": sc.coverage_altitude_m},
             "planner_readback": survey_eff, "logging": SURVEY_LOGGING_NOTE,
             **({"cross_uncovered": [list(u_) for u_ in plan.uncovered if "_cross_" in str(u_[0])]}
                if knobs.cross else {})},
            ensure_ascii=False, indent=1, default=str), encoding="utf-8")
        print(f"⚠ 航測旋鈕生效（有效值見 survey_knobs.json）：建圖 {knobs.mapping}"
              f"（引入段 {knobs.lead_in_m:g} m）、相機 {knobs.camera}、側向重疊 {sc.overlap_rate:g}、"
              f"前向重疊 {sorted({v['forward_overlap'] for v in survey_eff['sensors'].values()})}、"
              f"拍照率 {knobs.shutter_hz:g} Hz（{knobs.shutter_source}）→ CAM1_INTRVAL_MIN "
              f"{knobs.interval_min_s:g} s、CAM_MAX_ROLL {knobs.cam_max_roll_deg:g}°")
        for n in knobs.to_dict()["notes"]:
            print("  ⚠", n)
        if knobs.cross:
            _cx = [w for w in (report.get("warnings") or []) if str(w).startswith("mapping_cross")]
            _cu = [u_ for u_ in plan.uncovered if "_cross_" in str(u_[0])]
            print(f"⚠ 交叉航帶：面積比例 {knobs.cross_fraction:g}、高度差 {knobs.cross_alt_offset_m:+g} m；"
                  f"{len(_cx)} 則摘要／略過／退回（見 report.json warnings）"
                  + (f"；⚠⚠ {len(_cu)} 條交叉帶沒排進去（{sorted({r for _s, r in _cu})}），GUI 會擋上傳" if _cu else ""))
    print("匯出:", {u: summary["uavs"][u]["n_items"] for u in summary["uavs"]})
    return hv, plan, ctx, cfg, summary


# ══════════════════════════════════════════════════════════════════════
#  2. 起 SITL
# ══════════════════════════════════════════════════════════════════════
def _home_of(hv, uav: str) -> tuple[float, float]:
    """該機 SITL 的生成點 = **規劃真正登記的起飛站**之座標。

    來源順序（每一步都讀回，讀不到就往下，全部落空才丟例外）：
    1. ``UAVPlan.takeoff_site`` → ``ctx.inputs.sites[<id>]`` 的 lat/lon（vendored #8）。
    2. ``ctx.inputs.home`` 的 lat/lon。

    ⚠⚠⚠ **不要**在這裡退回模組層的 ``HOME`` 常數。那樣做的話「規劃分了站、
    SITL 沒分」會靜靜通過：任務照上傳、飛機照飛，只有第一段航線歪掉，
    而航線歪掉在報表上跟正常長得一樣。寧可停下來。
    """
    plan = hv.last_plan.plans[uav]
    ctx = hv.last_ctx
    sid = str(getattr(plan, "takeoff_site", "") or "")
    sites = getattr(getattr(ctx, "inputs", None), "sites", None) or {}
    site = sites.get(sid) if sid else None
    if site is None:
        site = getattr(getattr(ctx, "inputs", None), "home", None)
    if site is None or not hasattr(site, "lat_deg"):
        raise SystemExit(f"{uav}：查不到起飛站座標（takeoff_site={sid!r}）⇒ 停止，"
                         f"不要用預設 HOME 蓋過去")
    return (float(site.lat_deg), float(site.lon_deg))


def _write_wind(i: int) -> Path:
    """SITL 風場參數檔，排在 ``--defaults`` 最後（後蓋前）。

    ⚠ 這一份是**模擬器**的風，跟規劃層的 `Scenario.wind` 是兩件事：前者決定飛機
      真的遭遇什麼風，後者決定規劃器**以為**會遭遇什麼風。刻意不綁在一起，
      因為「規劃 0 飛 5」與「規劃 5 飛 5」的對照才是論文要的那個證據。
    """
    p = _instance_dir(i) / "wind.parm"
    p.write_text(
        "# 由 scripts/hv_sitl_run.py 產生；SIM_WIND_DIR 是氣象「來向」（270 = 西風）\n"
        f"SIM_WIND_SPD  {SIM_WIND_SPD:g}\n"
        f"SIM_WIND_DIR  {SIM_WIND_DIR:g}\n"
        f"SIM_WIND_TURB {SIM_WIND_TURB:g}\n"
        f"SIM_WIND_T    {SIM_WIND_T:g}\n"
        "SIM_WIND_DIR_Z 0\n", encoding="utf-8")
    return p


def launch(sc: Scenario, hv) -> list[dict]:
    """★ uav_id → sysid 用 `hv.id_map`（drone_id → uav_id）反查，不要靠排序 zip。"""
    out = out_dir(sc)
    sysid_of = {}
    for drone_id, uav_id in hv.id_map.items():
        _n, cls, sysid, *_ = sc.fleet[int(drone_id) - 1]
        sysid_of[uav_id] = (sysid, cls)
    procs, lines = [], []
    # 埠段錯開的理由見檔頭 BASE_PORT。
    for j, uav in enumerate(sorted(hv.last_plan.plans)):
        i = j + INSTANCE_OFFSET
        sysid, cls = sysid_of[uav]
        work = _instance_dir(i)
        for junk in ("eeprom.bin", "mav.parm"):
            (work / junk).unlink(missing_ok=True)
        defaults = [
            SITL_DIR / "default_params" / "plane.parm",
            SITL_DIR / "default_params" / "vtol_default.parm",
            _write_identity(i, sysid),
            _write_enforced(i),
            out / "plan" / f"{uav}.param",
            _write_firmware_override(i),
            _write_wind(i),          # ★ 最後蓋：SITL 風場（C2）
        ]
        missing = [str(d) for d in defaults if not d.exists()]
        if missing:
            raise SystemExit(f"缺參數檔：{missing}")
        # ⚠⚠⚠ 起飛點必須**讀回規劃真正用的那個站**，不可以寫死 HOME。
        #   規劃層在 spawn_spread_m > 0 時會生出逐機的 TO_<uav> 站；若 SITL 仍然
        #   全部生在 HOME，就變成「計畫說從 TO_A1 起飛、飛機在別的地方」——
        #   而這個不一致**沒有任何徵狀**：任務照上傳、照飛，只是第一段航線歪掉。
        #   `_home_of` 從 ctx 讀站點座標，讀不到就大聲停下來而不是默默用 HOME。
        hlat, hlon = _home_of(hv, uav)
        cmd = [str(BINARY), "--model", "quadplane",
               "--home", f"{hlat},{hlon},{HOME_ALT_MSL},0",
               "--speedup", str(SPEEDUP), "--instance", str(i),
               "--sysid", str(sysid),
               "--defaults", ",".join(str(d) for d in defaults)]
        lines.append(f"[{uav}] " + " ".join(cmd))
        proc = subprocess.Popen(cmd, cwd=str(work),
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append({"uav": uav, "cls": cls, "sysid": sysid, "instance": i,
                      "port": _SITL_PORT0 + 10 * i, "pid": proc.pid})
        time.sleep(1.0)
    (out / "sitl_cmdlines.txt").write_text("\n".join(lines), encoding="utf-8")
    return procs


def _lastlog(i: int) -> int | None:
    """該 instance 的 logs/LASTLOG.TXT（最新 .BIN 編號）。讀不到回 None（不要猜）。"""
    try:
        return int((_instance_dir(i) / "logs" / "LASTLOG.TXT").read_text(encoding="utf-8").strip())
    except (OSError, ValueError):
        return None


def _record_sitl_logs(out: Path, procs: list[dict], before: dict) -> dict:
    """把這一趟每架的 .BIN 路徑補進 survey_knobs.json（只在旋鈕開著時呼叫）。

    LASTLOG 起飛前後沒變 ⇒ 這一趟沒有新日誌 ⇒ ``bin`` 記 None，不拿舊檔充數。
    """
    rows = {}
    for p in procs:
        logs = _instance_dir(p["instance"]) / "logs"
        b, a = before.get(p["uav"]), _lastlog(p["instance"])
        rows[p["uav"]] = {"instance": p["instance"], "log_dir": str(logs), "lastlog_before": b,
                          "lastlog_after": a,
                          "bin": str(logs / f"{a:08d}.BIN") if a is not None and a != b else None}
    path = out / "survey_knobs.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    data["sitl_logs"] = rows
    path.write_text(json.dumps(data, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
    print("SITL 日誌（給 scripts/survey_sitl_s1.py）:", {u: r["bin"] for u, r in rows.items()})
    return rows


# ══════════════════════════════════════════════════════════════════════
#  3. 飛
# ══════════════════════════════════════════════════════════════════════
def _param_file_values(sc: Scenario, uav: str) -> dict[str, float]:
    vals = {}
    for ln in (out_dir(sc) / "plan" / f"{uav}.param").read_text(encoding="utf-8").splitlines():
        ln = ln.strip()
        if ln and not ln.startswith("#") and "," in ln:
            k, v = ln.split(",", 1)
            vals[k.strip()] = float(v)
    return vals


def _mission_for_upload(plan, uav: str, ctx, cfg, cam_cfg=None, summary: dict | None = None):
    """上傳用的任務項目：與匯出同一份 camera_cfg（上傳＝匯出）。給了 ``summary`` 就比對項數，不一致就停。"""
    from hv_dccpp import ardupilot as ap
    items = ap.plan_to_mission_items(plan.plans[uav], ctx, cfg, alt_frame="relative", camera_trigger=True,
                                     camera_cfg=cam_cfg)
    if summary is not None and len(items) != summary["uavs"][uav]["n_items"]:
        raise SystemExit(f"{uav} 上傳 {len(items)} 項 ≠ 匯出檔 {summary['uavs'][uav]['n_items']} 項")
    return items


def fly(sc: Scenario, knobs: SurveyKnobs | None = None) -> int:
    from hv_dccpp.mavlink_link import VehicleLink

    out = out_dir(sc)
    hv, plan, ctx, cfg, summary = plan_and_export(sc, knobs)
    cam_cfg = knobs.trigger_config() if knobs is not None else None     # 與匯出同一份（上傳＝匯出）
    uavs = sorted(plan.plans)
    print(f"\n機隊 {uavs}  makespan {plan.makespan_s:.0f}s  speedup {SPEEDUP}"
          f"  → 牆鐘約 {plan.makespan_s / SPEEDUP / 60:.1f} 分鐘\n")

    stop_ours()                       # 只收拾上一趟殘留的「我們的」
    procs = launch(sc, hv)
    print("SITL 已啟動:", [(p["uav"], p["port"], p["pid"]) for p in procs])
    log0 = {p["uav"]: _lastlog(p["instance"]) for p in procs} if knobs is not None else {}

    links: dict[str, VehicleLink] = {}
    try:
        for p in procs:
            lk = VehicleLink(f"tcp:127.0.0.1:{p['port']}", sysid=p["sysid"],
                             timeout_s=60.0, uav_id=p["uav"], time_source="boot")
            lk.wait_heartbeat(timeout_s=90.0)
            links[p["uav"]] = lk
            print(f"  {p['uav']} 心跳 OK (sysid {p['sysid']}, port {p['port']})")

        # ── 遙測速率：把位置串流拉高，否則間距量測**點不著** ────────
        #   ⚠⚠⚠ 預設 SR*_POSITION = 1 Hz 是**牆鐘**頻率，而 --speedup 5 讓模擬時鐘
        #   跑 5 倍 ⇒ flight.csv 的取樣間隔是 **5.00 s 模擬時間**（實測中位數）。
        #   HERO 19 m/s 在一個間隔飛 95 m，而水平門檻是 100 m ⇒ 兩機可以整段
        #   從彼此的取樣縫隙穿過去，量到的「違規 0 筆」跟真的安全長得一模一樣。
        #   ⇒ 拉到 10 Hz（模擬 0.5 s、HERO 9.5 m）並**讀回**確認真的生效。
        #   ⚠⚠⚠ 參數名是 **MAV1_/MAV2_/MAV3_**，不是 SR0_/SR1_。這版韌體把整組
        #     串流速率參數改名了（見 sitl/default_params/firmware_param_names.json：
        #     只有 MAV1_POSITION…MAV3_POSITION，沒有任何 SR* 串流參數）。
        #     第一版寫 SR0_/SR1_，四架讀回**全是 None**、速率一點都沒動 ——
        #     這正是「ArduPilot 對未知參數名靜默略過」那條老陷阱。
        #     ⇒ 名字錯了唯一的徵狀就是讀回 None，所以**一定要讀回**。
        #   ⚠⚠ 只調 POSITION 還是沒用（實測：參數讀回 10.0，flight.csv 仍是 5.00 s）。
        #     `read_telemetry` 的提前結束條件要**同時**有 EXTENDED_SYS_STATE 與
        #     BATTERY_STATUS/SYS_STATUS 在快取裡；拿不到就每次都跑滿 0.25 s 逾時，
        #     四條鏈路 ⇒ 迴圈固定 ~1 Hz 牆鐘 ⇒ 5 s 模擬。⇒ EXT_STAT／EXTRA2 也要調。
        #     判準是 flight.csv 的**實際中位間隔**，不是參數讀回值。
        want_hz = float(os.environ.get("HV_TELEM_HZ", "10"))
        _sr = {f"MAV{c}_{k}": want_hz for c in (1, 2, 3)
               for k in ("POSITION", "EXTRA1", "EXTRA2", "EXTRA3", "EXT_STAT")}
        for uav, lk in links.items():
            lk.set_params(_sr, timeout_s=5.0)
        got_hz = {u: lk.get_param("MAV1_POSITION", timeout_s=10.0) for u, lk in links.items()}
        print(f"\n位置串流速率讀回: {got_hz}（要求 {want_hz} Hz）")
        if any(v is None or abs(float(v) - want_hz) > 0.51 for v in got_hz.values()):
            print("  ⚠ 有機體沒吃到速率設定 ⇒ 該機的間距量測仍是 5 s 解析度，"
                  "低於數百公尺的結論不可信")

        # ── 風場讀回（C2）─────────────────────────────────────────
        #   ⚠⚠ 與 params_readback.csv **分開寫**：SIM_* 不屬於「匯出參數」那一組，
        #   混進去會把論文的 34 × 3 = 102 變成 38 × 3，把 C4 的數字攪亂。
        #   ⚠ 讀回 None ＝ 韌體沒有這個參數名（非 SITL build）⇒ 當成失敗，不可放行。
        #   ⚠⚠⚠ MIN_GROUNDSPEED 也在這一組：它不是 SIM_ 參數，但只要 > 0，TECS 會為了
        #   維持地速而在逆風腿**自己加空速** ⇒ 風的影響被飛控吃掉，整個對照作廢，
        #   而且沒有任何徵狀。匯出檔沒有設它（韌體預設 0），所以這裡是讀回確認。
        want_wind = {"SIM_WIND_SPD": SIM_WIND_SPD, "SIM_WIND_DIR": SIM_WIND_DIR,
                     "SIM_WIND_TURB": SIM_WIND_TURB, "SIM_WIND_T": SIM_WIND_T,
                     "MIN_GROUNDSPEED": 0.0}
        wind_rows, wind_bad = [], []
        for uav, lk in links.items():
            for name, exp in want_wind.items():
                got = lk.get_param(name, timeout_s=10.0)
                ok = got is not None and abs(float(got) - exp) <= 0.01
                wind_rows.append({"uav": uav, "param": name, "expected": exp,
                                  "actual": got, "ok": bool(ok)})
                if not ok:
                    wind_bad.append(f"{uav}.{name} 期望 {exp} 實得 {got}")
        (out / "wind_readback.json").write_text(json.dumps(
            {"sim_wind": want_wind, "plan_wind": list(sc.wind), "rows": wind_rows,
             "dir_convention": "氣象「來向」（270 = 西風，吹向東），與 hv_dccpp dir_from_deg 同約定"},
            ensure_ascii=False, indent=1), encoding="utf-8")
        n_ok = sum(r["ok"] for r in wind_rows)
        print(f"\nSITL 風場讀回：{n_ok}/{len(wind_rows)} 相符"
              f"（SIM_WIND_SPD={SIM_WIND_SPD:g} DIR={SIM_WIND_DIR:g}）"
              f"；規劃假設風 {sc.wind}")
        if wind_bad:
            for b in wind_bad:
                print("   ✗", b)
            raise SystemExit("風場參數沒有生效，停止起飛（見 wind_readback.json）")

        # ── 生成點讀回：飛機真的生在規劃說的那個起飛站嗎 ────────────
        #   ⚠⚠⚠ 這是 vendored #8 之後必要的一道。規劃層在 spawn_spread_m > 0 時
        #   會逐機生出 TO_<uav> 站；若 SITL 仍全部生在 HOME，任務照上傳、照飛，
        #   只有第一段航線歪掉 —— **完全沒有徵狀**。所以讀回實際位置比對。
        #   ⚠ 起飛前的 GLOBAL_POSITION_INT 就是 `--home`（還沒動）。
        home_rows, home_bad = [], []
        for uav, lk in links.items():
            want = _home_of(hv, uav)
            # ⚠⚠⚠ 判準是**資料穩定**，不是數值範圍。EKF 還在初始化時回報的位置
            #   會從 0 附近往上漂：實測第一架（心跳後立刻被問）先回 (0, 0)，接著
            #   回 (2.975e-4, 2.37e-5)。兩版守衛都被它騙過去 ——
            #     v1「la or lo 非零」→ (0,0) 之後那一筆過關；
            #     v2「兩個分量 > 1e-6」→ 2.975e-4 也過關。
            #   兩次都量出 **12 594 400 m** 的「誤差」，看起來像規劃與 SITL 天差地遠。
            #   ⚠ 不可以改成「離我們送出的 --home 夠近才算數」：那正好把本檢查要抓的
            #     「規劃分了站、SITL 沒分」整個遮掉（拿被測物自己的輸入當判準）。
            #   ⇒ 改成等**連續兩筆相隔 ≥1.5 s 且相距 ≤0.5 m**，收斂了才比對。
            #     真正的不一致是**穩定**的，照樣會被抓出來。
            got, keep, t0_wait, t_end = None, None, time.time(), time.time() + 90.0
            trace: list[dict] = []
            while got is None and time.time() < t_end:
                tel = lk.read_telemetry(timeout_s=1.0, require_new_position=False)
                la, lo = getattr(tel, "lat_deg", None), getattr(tel, "lon_deg", None)
                if la is None or lo is None:
                    continue
                cur = (float(la), float(lo), time.time())
                # ★ 收斂過程的原始軌跡：**只記錄、不參與判定**。用來分辨
                #   「機體真的在地面上移動」與「只是狀態估計在漂」——前者地速會
                #   不為零、機首可能轉，後者兩者都會是 0。位移調查需要這個。
                def _r(v, n=3):
                    return None if v is None else round(float(v), n)
                trace.append({
                    "dt_s": round(cur[2] - t0_wait, 2),
                    "d_north_m": round((cur[0] - want[0]) * 111132.0, 3),
                    "d_east_m": round((cur[1] - want[1]) * 111412.0
                                      * math.cos(math.radians(want[0])), 3),
                    "heading_deg": _r(getattr(tel, "heading_deg", None), 1),
                    "groundspeed_mps": _r(getattr(tel, "groundspeed_mps", None)),
                    "airspeed_mps": _r(getattr(tel, "airspeed_mps", None)),
                    "wind_spd_est": _r(getattr(tel, "wind_speed_mps", None), 2),
                    "wind_dir_from_est": _r(getattr(tel, "wind_dir_from_deg", None), 1),
                })
                if keep is not None and cur[2] - keep[2] >= 1.5:
                    dd = math.hypot((cur[0] - keep[0]) * 111132.0,
                                    (cur[1] - keep[1]) * 111412.0
                                    * math.cos(math.radians(cur[0])))
                    if dd <= 0.5:
                        got = (cur[0], cur[1])
                    keep = cur
                elif keep is None:
                    keep = cur
            if got is None:
                raise SystemExit(f"{uav}：起飛前 90 s 內位置沒有收斂 ⇒ 無法確認生成點")
            settle_s = round(time.time() - t0_wait, 1)
            dy = (got[0] - want[0]) * 111132.0
            dx = (got[1] - want[1]) * 111412.0 * math.cos(math.radians(want[0]))
            err = math.hypot(dx, dy)
            site = str(getattr(plan.plans[uav], "takeoff_site", "") or "")
            home_rows.append({"uav": uav, "takeoff_site": site,
                              "want_lat": want[0], "want_lon": want[1],
                              "got_lat": got[0], "got_lon": got[1],
                              "err_m": round(err, 2), "ok": err <= SPAWN_TOL_M,
                              "settle_s": settle_s,
                              # ★ 把 err_m 拆成可檢驗的兩軸 + 方位角：一個純量擋不住
                              #   「量值對、機制錯」的解釋（180° 順風只能解釋北向分量）。
                              "d_north_m": round(dy, 3), "d_east_m": round(dx, 3),
                              "err_m_full": round(err, 4),
                              "bearing_deg": round(math.degrees(math.atan2(dx, dy)) % 360.0, 2),
                              "settle_trace": trace})
            print(f"  {uav} 生成點 {site or '(home)'} 誤差 {err:5.2f} m "
                  f"{'OK' if err <= SPAWN_TOL_M else '✗'}（收斂等了 {settle_s}s；"
                  f"dN {dy:+.2f} dE {dx:+.2f} m）")
            if err > SPAWN_TOL_M:
                home_bad.append(f"{uav} 差 {err:.2f} m（站 {site!r}）")

        # ── 剛體平移檢查：閘門真正的理由 ──────────────────────────
        #   「規劃分了站、SITL 沒分」會讓**兩兩間距塌掉**（10 m → 0 m）。逐機位移
        #   容差可以被 HV_SPAWN_TOL_M 明示放寬，但這一項不行：它是這道閘門存在的
        #   目的本身。整隊一起平移（間距守住）⇒ 閘門要抓的事沒有發生。
        def _pair_m(alat, alon, blat, blon):
            return math.hypot((alat - blat) * 111132.0,
                              (alon - blon) * 111412.0 * math.cos(math.radians(alat)))

        pairs = []
        for _i, _a in enumerate(home_rows):
            for _b in home_rows[_i + 1:]:
                _pw = _pair_m(_a["want_lat"], _a["want_lon"], _b["want_lat"], _b["want_lon"])
                _pg = _pair_m(_a["got_lat"], _a["got_lon"], _b["got_lat"], _b["got_lon"])
                pairs.append({"pair": f'{_a["uav"]}-{_b["uav"]}',
                              "planned_m": round(_pw, 4), "actual_m": round(_pg, 4),
                              "delta_m": round(_pg - _pw, 4)})
        rigid_max = max((abs(p["delta_m"]) for p in pairs), default=0.0)
        rigid_ok = rigid_max <= SPAWN_RIGID_TOL_M
        (out / "spawn_readback.json").write_text(
            json.dumps({"rows": home_rows, "spawn_spread_m": sc.spawn_spread_m,
                        "tol_m": SPAWN_TOL_M,
                        "tol_default_m": SPAWN_TOL_DEFAULT_M,
                        "tol_overridden": bool(_SPAWN_TOL_ENV),
                        "tol_source": "HV_SPAWN_TOL_M" if _SPAWN_TOL_ENV else "default",
                        "pairwise": pairs,
                        "rigid_translation": {
                            "max_abs_delta_m": round(rigid_max, 4),
                            "tol_m": SPAWN_RIGID_TOL_M, "ok": bool(rigid_ok),
                            "enforced": bool(_SPAWN_TOL_ENV),
                            "note": "兩兩間距是否守住＝『規劃分站、SITL 沒分』有沒有發生。"
                                    "不吃 HV_SPAWN_TOL_M；容差被明示放寬時改由這一項把關"}},
                       ensure_ascii=False, indent=1), encoding="utf-8")
        if pairs:
            print(f"  生成點兩兩最小間距 {min(p['actual_m'] for p in pairs):.2f} m"
                  f"（情境設定 {sc.spawn_spread_m:.1f} m）；"
                  f"與規劃間距最大差 {rigid_max:.3f} m "
                  f"{'（剛體平移）' if rigid_ok else '✗ 間距沒守住'}")
        if _SPAWN_TOL_ENV:
            print(f"⚠⚠⚠ HV_SPAWN_TOL_M={_SPAWN_TOL_ENV}：生成點容差已由預設 "
                  f"{SPAWN_TOL_DEFAULT_M:g} m 明示放寬為 {SPAWN_TOL_M:g} m。"
                  f"這一趟的 spawn_readback.json 會標記 tol_overridden=true。")
            if not rigid_ok:
                raise SystemExit(
                    f"生成點兩兩間距與規劃差 {rigid_max:.3f} m > {SPAWN_RIGID_TOL_M:g} m"
                    f"⇒ 不是整隊平移，放寬容差不適用（這正是本閘門要抓的不一致）")
        if home_bad:
            raise SystemExit("生成點與規劃的起飛站不一致：" + "；".join(home_bad))

        # ── 參數讀回 ───────────────────────────────────────────────
        rows, bad = [], []
        for uav, lk in links.items():
            from_file = _param_file_values(sc, uav)
            for name, (exp, src, note) in expected_params(from_file).items():
                got = lk.get_param(name, timeout_s=10.0)
                ok = (got is not None and exp is not None
                      and abs(got - exp) <= max(0.01, abs(exp) * 0.001))
                # 韌體端整數型別會截掉小數（實測 AIRSPEED_MAX 是 INT16 ⇒ 26.6→26.0）。
                if not ok and got is not None and exp is not None and \
                        got in (float(int(exp)), float(round(exp))):
                    ok, note = True, f"韌體整數型別，{exp} → {got}"
                # 讀回 None＝韌體沒有這個名字（改名或打錯）。這是靜默失敗的唯一徵狀，
                # 必須當成失敗，不可以因為「沒讀到就跳過」而放行。見 memory: sitl-param-renames-silent
                if got is None:
                    note = (note + "；" if note else "") + "讀回 None＝韌體沒有這個參數名"
                rows.append({"uav": uav, "param": name, "expected": exp, "actual": got,
                             "ok": bool(ok), "source": src, "note": note})
                if exp is not None and not ok:
                    bad.append(f"{uav}.{name} 期望 {exp}（{src}）實得 {got}")
        with (out / "params_readback.csv").open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=["uav", "param", "expected", "actual",
                                              "ok", "source", "note"])
            w.writeheader()
            w.writerows(rows)
        _n_file = len(_param_file_values(sc, uavs[0]))
        _n_par = len({r["param"] for r in rows})
        print(f"\n參數讀回：{sum(r['ok'] for r in rows)}/{len(rows)} 相符"
              f"（{_n_par} 個參數 × {len(links)} 台；匯出檔 {_n_file} 個全部涵蓋）")
        if bad:
            for b in bad:
                print("   ✗", b)
            raise SystemExit("參數沒有生效，停止起飛（見 params_readback.csv）")

        # ── 上傳 + 下載比對 ────────────────────────────────────────
        upload_rows = []
        for uav, lk in links.items():
            items = _mission_for_upload(plan, uav, ctx, cfg, cam_cfg,
                                        summary if knobs is not None else None)
            lk.upload_mission(items, timeout_s=60.0)
            back = lk.download_mission(timeout_s=60.0)
            same = len(back) == len(items)
            upload_rows.append({"uav": uav, "sent": len(items), "read_back": len(back),
                                "ok": same, "first_cmds": [i.command for i in back[:3]]})
            print(f"  {uav} 上傳 {len(items)} → 讀回 {len(back)} "
                  f"{'OK' if same else '✗ 不一致'}")
            if not same:
                raise SystemExit(f"{uav} 任務讀回不一致")
        (out / "mission_upload.json").write_text(
            json.dumps(upload_rows, ensure_ascii=False, indent=1), encoding="utf-8")

        # ── 解鎖（先解鎖再切 AUTO）─────────────────────────────────
        for uav, lk in links.items():
            armed, reasons, t0 = False, [], time.time()
            while not armed and time.time() - t0 < 120.0:
                armed = lk.arm(True, timeout_s=10.0)
                if armed:
                    break
                deadline = time.time() + 5.0
                while time.time() < deadline:
                    msg = lk.conn.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
                    if msg is not None and msg.text not in reasons:
                        reasons.append(msg.text)
            if not armed:
                print(f"  {uav} 解鎖失敗，飛控說：")
                for r in reasons[-6:]:
                    print("     ", r)
                raise SystemExit(f"{uav} 解鎖失敗")
            print(f"  {uav} armed（等 {time.time() - t0:.0f}s）")

        # ── 依規劃起飛時刻逐機切 AUTO ─────────────────────────────
        t_off = {u: float(plan.plans[u].t_takeoff_s) for u in links}
        t_min = min(t_off.values())
        release = {u: (t_off[u] - t_min) / SPEEDUP for u in links}
        print("\n放行時刻（牆鐘秒）:",
              {u: round(v, 1) for u, v in sorted(release.items(), key=lambda kv: kv[1])})

        writers, files, peak = {}, {}, {u: 0.0 for u in links}
        for uav in links:
            d = out / uav
            d.mkdir(parents=True, exist_ok=True)
            f = (d / "flight.csv").open("w", newline="", encoding="utf-8")
            w = csv.writer(f)
            w.writerow(["t_s", "lat", "lon", "rel_alt_m", "airspeed",
                        "groundspeed", "wp_seq", "mode", "armed",
                        "wind_spd_est", "wind_dir_from_est"])
            files[uav], writers[uav] = f, w

        budget_s = plan.makespan_s / SPEEDUP + 420.0
        t_start = time.time()
        done: set[str] = set()
        released: dict[str, float] = {}
        dead: set[str] = set()          # 連續拿不到遙測 ⇒ 不再輪詢（見迴圈內說明）
        rejected: set[str] = set()      # 切 AUTO 三次都被拒 ⇒ 這一架不會起飛
        miss: dict[str, int] = {}
        DEAD_AFTER = 60
        while (time.time() - t_start < budget_s and len(done) < len(links)
               and len(dead) < len(links)):
            el = time.time() - t_start
            for uav, lk in links.items():
                if uav not in released and el >= release[uav] and uav not in dead:
                    # ⚠ 切 AUTO 被拒就是「這一架不會起飛」，而且不會有別的徵狀。
                    #   HV4SITL6（speedup 10）就是這樣：兩架被拒、全程 0.0 m，
                    #   腳本卻照跑到逾時。所以重試，仍失敗就記成 rejected。
                    ok = False
                    for _try in range(3):
                        ok = lk.set_mode("AUTO", timeout_s=10.0)
                        if ok:
                            break
                    released[uav] = time.time() - t_start
                    if not ok:
                        rejected.add(uav)
                    print(f"  > {uav} 放行切 AUTO（牆鐘 {released[uav]:.1f}s，"
                          f"規劃 t_off={t_off[uav]:.0f}s）"
                          f"{'' if ok else ' ✗ 三次都被拒，這一架不會起飛'}")
            for uav, lk in links.items():
                if uav in dead:
                    continue
                # ⚠⚠⚠ 這個逾時**就是**取樣率的上限：`read_telemetry` 的提前結束
                #   條件要 EXTENDED_SYS_STATE 在快取裡，拿不到就每次跑滿逾時
                #   ⇒ 四條鏈路 × 0.25 s = 迴圈固定 1 Hz 牆鐘 = **5 s 模擬**。
                #   三趟不同串流設定都拿到**剛好 164 筆**，就是這裡卡的，不是載具。
                tel = lk.read_telemetry(timeout_s=_POLL_S, require_new_position=False)
                if tel is None:
                    # ⚠⚠ 連線斷掉時 pymavlink 會**每次呼叫都印一行** socket 錯誤。
                    #   HV4SITL6 那一趟因此產生 1744 萬行 / 832 MB 的 log。
                    #   所以連續拿不到遙測就把這一架標成 dead，不再輪詢它。
                    miss[uav] = miss.get(uav, 0) + 1
                    if miss[uav] >= DEAD_AFTER and uav not in dead:
                        dead.add(uav)
                        print(f"  ✗ {uav} 連續 {DEAD_AFTER} 次拿不到遙測 ⇒ "
                              f"視為斷線，停止輪詢（SITL 掛了？）")
                        if len(dead) == len(links):
                            print("  ✗ 四架全部斷線，提前結束這一趟")
                    continue
                miss[uav] = 0
                alt = float(getattr(tel, "rel_alt_m", 0.0) or 0.0)
                peak[uav] = max(peak[uav], alt)
                writers[uav].writerow([
                    round(float(getattr(tel, "t_s", 0.0) or 0.0), 2),
                    getattr(tel, "lat_deg", None), getattr(tel, "lon_deg", None),
                    round(alt, 2),
                    round(float(getattr(tel, "airspeed_mps", 0.0) or 0.0), 2),
                    round(float(getattr(tel, "groundspeed_mps", 0.0) or 0.0), 2),
                    getattr(tel, "mission_seq", None), getattr(tel, "mode", ""),
                    getattr(tel, "armed", None),
                    # ★ 飛控自己的風估計（MAVLink WIND）—— 風真的有作用的獨立證據。
                    #   跟「groundspeed − airspeed」互為對照：兩者都指向同一個 W 才算數。
                    getattr(tel, "wind_speed_mps", None),
                    getattr(tel, "wind_dir_from_deg", None)])
                if peak[uav] > 20.0 and alt < 2.0 and not getattr(tel, "armed", True):
                    if uav not in done:
                        done.add(uav)
                        print(f"  {uav} 已降落 (sim t={getattr(tel, 't_s', 0):.0f}s)")
            if int(el) % 20 == 0:
                print(f"    [{el:5.0f}s] " + "  ".join(
                    f"{u}:{peak[u]:5.1f}m" for u in sorted(links)))
                time.sleep(1.0)
        for f in files.values():
            f.close()

        (out / "release_schedule.json").write_text(json.dumps(
            {"speedup": SPEEDUP,
             "planned_t_takeoff_s": {u: round(t_off[u], 1) for u in t_off},
             "release_wall_s": {u: round(release[u], 1) for u in release},
             "actual_release_wall_s": {u: round(v, 1) for u, v in released.items()},
             "mode_change_rejected": sorted(rejected), "link_lost": sorted(dead)},
            ensure_ascii=False, indent=1), encoding="utf-8")
        if knobs is not None:
            _record_sitl_logs(out, procs, log0)
        print(f"\n落地 {len(done)}/{len(links)}：{sorted(done)}")
        if rejected:
            print(f"⚠ 切 AUTO 三次都被拒（沒起飛）：{sorted(rejected)}")
        if dead:
            print(f"⚠ 中途斷線：{sorted(dead)}")
        print("最高高度:", {u: round(v, 1) for u, v in peak.items()})
        return 0 if len(done) == len(links) else 2
    finally:
        for lk in links.values():
            try:
                lk.close()
            except Exception:      # noqa: BLE001
                pass
        stop_ours([p["pid"] for p in procs])


def main() -> int:
    cmd = sys.argv[1] if len(sys.argv) > 1 else "list"
    if cmd == "list":
        for k, s in SCENARIOS.items():
            print(f"{k:22s} trial={s.trial:10s} path={s.path:7s} "
                  f"{s.n_a}A+{s.n_b}B alt={s.coverage_altitude_m:.0f}m  {s.note}")
        return 0
    key = sys.argv[2] if len(sys.argv) > 2 else ""
    if key not in SCENARIOS:
        print(f"沒有這個情境：{key!r}。可用：{list(SCENARIOS)}")
        return 1
    sc = SCENARIOS[key]
    # ★ 規劃風的覆寫：做「規劃 0 飛 5」這種不匹配格子用，不用新增情境。
    if os.environ.get("HV_PLAN_WIND_SPD"):
        import dataclasses
        sc = dataclasses.replace(sc, wind=(float(os.environ["HV_PLAN_WIND_SPD"]),
                                           float(os.environ.get("HV_PLAN_WIND_DIR", "270"))))
        print(f"⚠ HV_PLAN_WIND_SPD：規劃假設風改為 {sc.wind}（SITL 風是另一個旋鈕）")
    # ★ 規劃高度上限／航高的覆寫（C1／D2）：把天花板壓到 119.5 m 再飛，
    #   量到的最高高度才會真的留在 120.000 m 以下，不用改情境表。
    #
    #   ⚠⚠⚠ 為什麼需要它：規劃器內部的最大 AGL **剛好** 120.000000 m，可是
    #     `hv_dccpp/ardupilot.py:388-395` 的 `reduce_waypoints()` 把每段圓弧重新
    #     離散成外接多邊形，新頂點的 z 是在弧兩端**線性內插**出來的；而等 AGL 面
    #     在本地 ENU 是拋物面（z = h − d²/2R_earth），所以新頂點會落在該面之上
    #     （匯出 +0.004…+0.075 m，加上追蹤誤差 +0.021…+0.127 m）⇒ 30/30 架次的
    #     實飛最高點都超過 120.000 m（最大 120.18 m）。法規上限 400 ft = 121.92 m，
    #     所以不違法，但「不超過 120 m」這句話在產物上是假的。
    #   ⚠ 送審前**不修**那個內插缺陷（會動到每一份既有產物）⇒ 改成壓天花板。
    #     代價實測幾乎為零：makespan +0.02 s、航程 +1 m、最小間距還略好
    #     （runs/artifacts/PAPERPLAN1/c1_altitude/ablation_cap.json）。
    #
    #   兩個旋鈕刻意分開，與風的那兩個同一個道理：
    #     HV_PLAN_CEIL_M  → 空域 AGL 上限 h_ceil（走 `waiver_max_agl_m` 這一支，
    #                        見 hv_dccpp/context.py:418 `resolve_ceiling` 的優先序）。
    #     HV_PLAN_ALT_M   → 目標航高 `coverage_altitude_m`。不給時自動壓到不超過
    #                        新天花板（＝ ablation_cap.py 驗過的那個組合），
    #                        因為轉接層本來就會 clamp，只是會多一條 warning。
    #   ⚠ 兩個都不給 ⇒ **完全不動情境**，與今天逐位元相同（120.0 參考格就是這樣跑）。
    #   ⚠ `waiver_max_agl_m` 這個名字在這裡是「h_ceil 旋鈕」而不是「真的有豁免」：
    #     它唯一的另一個作用是 `decompose.py:401` / `preflight.py:537` 在高度帶
    #     **不可行時**挑 `A_needs_waiver` 還是 `band_lt_min` 當理由字串。本情境
    #     （min_fw 100、dh_band_min 15、119.5 − 100 = 19.5 ≥ 15）可行，不受影響。
    #   ⚠ 產物上看得見：scenario.json 的 `waiver_max_agl_m` 與 `h_ceil_m` 會寫 119.5。
    if os.environ.get("HV_PLAN_CEIL_M") or os.environ.get("HV_PLAN_ALT_M"):
        import dataclasses
        _ceil_s = os.environ.get("HV_PLAN_CEIL_M")
        _alt_s = os.environ.get("HV_PLAN_ALT_M")
        _ceil = sc.waiver_max_agl_m if _ceil_s is None else float(_ceil_s)
        if _alt_s:
            _alt = float(_alt_s)
        else:
            _alt = min(float(sc.coverage_altitude_m),
                       math.inf if _ceil is None else float(_ceil))
        sc = dataclasses.replace(sc, waiver_max_agl_m=_ceil, coverage_altitude_m=_alt)
        print(f"⚠ HV_PLAN_CEIL_M／HV_PLAN_ALT_M：規劃天花板 {_ceil} m、"
              f"目標航高 {_alt} m（情境原值 {SCENARIOS[key].waiver_max_agl_m} / "
              f"{SCENARIOS[key].coverage_altitude_m} m）")
    logging.basicConfig(level=logging.WARNING,
                        format="%(levelname)s %(name)s: %(message)s")
    # ★ 航測建圖／相機旋鈕（SITL S1）。都不給 ⇒ None ⇒ 與舊版逐位元相同。
    knobs = survey_knobs() if cmd in ("plan", "fly") else None
    if cmd == "plan":
        plan_and_export(sc, knobs)
        return 0
    if cmd == "fly":
        return fly(sc, knobs)
    if cmd == "stop":
        stop_ours()
        return 0
    print(f"未知指令 {cmd!r}（list/plan/fly/stop）")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
