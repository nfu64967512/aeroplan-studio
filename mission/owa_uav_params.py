"""
OWA-UAV (One-Way Attack) 長程打擊機 SITL 飛控參數生成器
========================================================

戰術背景
--------
長程單向攻擊無人機 (One-Way Attack UAV，例如 Shahed-136 / Geran-2)
具有以下作戰特徵：

    1. **內燃機 (ICE) 動力**：通常使用 50~100 cc 雙缸汽油引擎驅動後置螺旋槳
    2. **低空地形跟隨突防**：貼地 50~200m 飛行，利用地形遮蔽敵方雷達視距
    3. **一去不復返 (kamikaze)**：預先裝訂打擊座標，命中即完成任務
    4. **電戰強韌性**：遭遇 GPS 干擾 / GCS 訊號中斷時「絕對不能 RTL」，
       必須死板地沿任務序列繼續前進直至命中座標或耗盡燃料

因此 SITL 驗證 OWA 任務時，預設 ArduPlane 的「回家 / 暫停」失效保護
必須被強制覆寫為「繼續自動任務」。本模組自動產生符合上述戰術的 `.parm` 檔。

使用方式
--------
    from mission.owa_uav_params import generate_owa_uav_parm, OWAParamConfig

    # 最簡用法：全部取預設
    generate_owa_uav_parm('sitl/default_params/owa_uav_default.parm')

    # 客製化：根據巡航速度調整 WP_RADIUS 等
    cfg = OWAParamConfig(cruise_airspeed_mps=38.0, ice_idle_rpm=1300)
    generate_owa_uav_parm('sitl/default_params/shahed_like.parm', cfg)

之後 SITL 啟動時掛上 `--defaults plane.parm,owa_uav_default.parm`
即可載入這些戰術覆寫。
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional, Tuple

from utils.logger import get_logger

logger = get_logger()


# ═══════════════════════════════════════════════════════════════════════
#  OWA-UAV 參數配置
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class OWAParamConfig:
    """OWA-UAV 戰術參數配置

    所有預設值均參考公開的 ArduPlane 4.5+ 文件與 Shahed-136 類武器
    的公開資料。可依實際機型客製化後傳入 ``generate_owa_uav_parm()``。

    ─── 進階失效保護 (絕對不能 RTL) ─────────────────────────────
    fs_long_actn : int
        長時 failsafe 動作：**0 = Continue if in AUTO** (關鍵戰術覆寫)
        其他值 1=RTL / 2=Glide / 3=Parachute 都會使 OWA 任務失敗
    fs_long_timeout : float
        長時 failsafe 觸發秒數（秒）
    fs_short_actn : int
        短時 failsafe 動作：**0 = Continue if in AUTO**
    fs_short_timeout : float
        短時 failsafe 觸發秒數
    fs_gcs_enabl : int
        GCS failsafe 啟用：1 = 啟用（但配合 actn=0 讓它繼續）
    thr_failsafe : int
        油門 RC failsafe：1 = 啟用
    rtl_autoland : int
        禁用 RTL-autoland（0 = 關閉，不進行回場降落動作）

    ─── 內燃機 (ICE) 動力 ────────────────────────────────────────
    ice_enable : int
        啟用內燃機邏輯（1 = 啟用）
    ice_start_chan : int
        啟動通道：0 = 任務自動啟動（不需 RC 觸發）
    ice_start_pct : int
        引擎啟動油門百分比（Shahed 類約 15~25%）
    ice_idle_rpm : int
        怠速 RPM（地面 armed 後的保持值）
    ice_idle_pct : int
        怠速油門 %
    ice_redline_rpm : int
        紅線 RPM（超過此值保護觸發）
    ice_rpm_thresh : int
        引擎「已啟動」的 RPM 判定閾值
    rpm_type : int
        RPM 感測器類型：1 = PWM (常用於 ArduPlane SITL)
    rpm_pin : int
        RPM 輸入腳位 (Pixhawk AUX5 = 54)
    rpm_scaling : float
        RPM 訊號倍率

    ─── 地形跟隨 / 航點邏輯 ──────────────────────────────────────
    terrain_enable : int
        啟用地形資料庫
    terrain_follow : int
        巡航時貼地飛行（1 = 啟用；WP/RTL 均貼地）
    terrain_spacing : int
        地形柵格解析度（m，100 = 標準 SRTM 近似）
    wp_radius : int
        航點通過半徑 (m)；戰術建議 = 巡航速度 × 1.5
    wp_loiter_rad : int
        盤旋半徑 (m)，用於 NAV_LOITER_*
    wp_max_radius : int
        任意航點最大可接受半徑上限
    navl1_period : float
        L1 控制器週期；越小越緊貼航線 (預設 20，OWA 戰術可調至 15)
    cruise_airspeed_mps : float
        巡航空速 (m/s)，會同步寫入 AIRSPEED_CRUISE / TRIM_ARSPD_CM
    stall_airspeed_mps : float
        失速空速 (m/s) → ARSPD_FBW_MIN
    max_airspeed_mps : float
        最大空速 (m/s) → ARSPD_FBW_MAX
    cruise_alt_m : int
        預設巡航高度 (m)
    mis_done_behave : int
        任務完成後行為：0 = HOLD（禁止 RTL，符合 OWA 哲學）
    mis_restart : int
        重啟 mission 行為：0 = 繼續從中斷點飛
    """

    # ── 失效保護 ──────────────────────────────────────────────
    fs_long_actn:     int   = 0
    fs_long_timeout:  float = 5.0
    fs_short_actn:    int   = 0
    fs_short_timeout: float = 1.5
    fs_gcs_enabl:     int   = 1
    thr_failsafe:     int   = 1
    rtl_autoland:     int   = 0

    # ── 內燃機 ──────────────────────────────────────────────
    ice_enable:       int   = 1
    ice_start_chan:   int   = 0
    ice_start_pct:    int   = 20
    ice_idle_rpm:     int   = 1300
    ice_idle_pct:     int   = 5
    ice_redline_rpm:  int   = 9000
    ice_rpm_thresh:   int   = 100
    rpm_type:         int   = 1
    rpm_pin:          int   = 54
    rpm_scaling:      float = 1.0

    # ── 地形 / 航點 ───────────────────────────────────────────
    terrain_enable:   int   = 1
    terrain_follow:   int   = 1
    terrain_spacing:  int   = 100
    wp_radius:        int   = 0   # 0 = 自動依 cruise_speed 計算
    wp_loiter_rad:    int   = 120
    wp_max_radius:    int   = 0   # 0 = 自動
    navl1_period:     float = 18.0

    # ── 空速 / 高度 ───────────────────────────────────────────
    cruise_airspeed_mps: float = 38.0     # Shahed-136 約 35-50 m/s
    stall_airspeed_mps:  float = 22.0
    max_airspeed_mps:    float = 55.0
    cruise_alt_m:        int   = 120      # 低空突防 (100~200m)

    # ── 任務行為 ─────────────────────────────────────────────
    mis_done_behave:  int   = 0
    mis_restart:      int   = 0

    # ── 附加 overrides ────────────────────────────────────────
    extra_params: Dict[str, float] = field(default_factory=dict)

    def __post_init__(self):
        # WP_RADIUS 自動估算：巡航速度 × 1.5 (保留最小 60m)
        if self.wp_radius <= 0:
            self.wp_radius = max(60, int(self.cruise_airspeed_mps * 1.5))
        if self.wp_max_radius <= 0:
            self.wp_max_radius = self.wp_radius * 3

        # 邊界檢查
        if not (self.stall_airspeed_mps < self.cruise_airspeed_mps < self.max_airspeed_mps):
            raise ValueError(
                f'空速設定不合理：stall({self.stall_airspeed_mps}) < '
                f'cruise({self.cruise_airspeed_mps}) < max({self.max_airspeed_mps})'
            )
        if self.fs_long_actn != 0 or self.fs_short_actn != 0:
            logger.warning(
                '[OWA] 警告：fs_*_actn 非 0，OWA-UAV 戰術要求此值必須為 0 (Continue in AUTO)。'
                '若你不確定請恢復預設。'
            )


# ═══════════════════════════════════════════════════════════════════════
#  核心：產生 .parm 檔內容
# ═══════════════════════════════════════════════════════════════════════

def _build_parm_sections(cfg: OWAParamConfig) -> List[Tuple[str, List[Tuple[str, float, str]]]]:
    """組織分區的 (section_title, [(param, value, comment), ...]) 結構

    之所以拆成區塊而非一次性 dict，是為了在輸出的 .parm 檔中保留
    可讀性高的分節註解，方便飛控工程師手動檢查。
    """
    return [
        # ─── 區塊 1：絕對不能 RTL ─────────────────────────────
        ('1. 失效保護 (FAILSAFE) — OWA 戰術要求「訊號中斷 ≠ RTL」',
         [
             ('FS_LONG_ACTN',     cfg.fs_long_actn,
              '0=Continue AUTO (OWA 戰術必須=0，禁止 RTL)'),
             ('FS_LONG_TIMEOUT',  cfg.fs_long_timeout,
              '長時 failsafe 觸發秒數'),
             ('FS_SHORT_ACTN',    cfg.fs_short_actn,
              '0=Continue AUTO (短時也必須=0)'),
             ('FS_SHORT_TIMEOUT', cfg.fs_short_timeout,
              '短時 failsafe 觸發秒數'),
             ('FS_GCS_ENABL',     cfg.fs_gcs_enabl,
              '1=啟用 GCS failsafe (配合 actn=0 仍會繼續)'),
             ('THR_FAILSAFE',     cfg.thr_failsafe,
              '1=啟用油門 RC failsafe'),
             ('RTL_AUTOLAND',     cfg.rtl_autoland,
              '0=禁用自動降落 (絕不回場)'),
             ('MIS_DONE_BEHAVE',  cfg.mis_done_behave,
              '0=任務完成 HOLD (禁止 RTL)'),
             ('MIS_RESTART',      cfg.mis_restart,
              '0=從中斷點繼續，不重置 mission'),
         ]),

        # ─── 區塊 2：內燃機 (ICE) ────────────────────────────
        ('2. 內燃機動力系統 (ICE) — Shahed 類 50~100cc 雙缸汽油機模擬',
         [
             ('ICE_ENABLE',        cfg.ice_enable,
              '1=啟用 ICE 控制邏輯'),
             ('ICE_START_CHAN',    cfg.ice_start_chan,
              '0=任務自動啟動 (無需 RC)'),
             ('ICE_START_PCT',     cfg.ice_start_pct,
              '啟動油門 % (20~25 為典型 Shahed 類值)'),
             ('ICE_IDLE_RPM',      cfg.ice_idle_rpm,
              '地面怠速 RPM'),
             ('ICE_IDLE_PCT',      cfg.ice_idle_pct,
              '怠速油門 %'),
             ('ICE_REDLINE_RPM',   cfg.ice_redline_rpm,
              '紅線保護 RPM'),
             ('ICE_RPM_THRESH',    cfg.ice_rpm_thresh,
              '判定「引擎已點火」的 RPM 閾值'),
             ('RPM_TYPE',          cfg.rpm_type,
              '1=PWM 輸入 (SITL 預設)'),
             ('RPM_PIN',           cfg.rpm_pin,
              'RPM 訊號腳位 (AUX5=54)'),
             ('RPM_SCALING',       cfg.rpm_scaling,
              '訊號倍率 (SITL 模擬用 1.0)'),
         ]),

        # ─── 區塊 3：地形跟隨 / 航點 ─────────────────────────
        ('3. 地形跟隨 & 航點邏輯 — 低空突防、WP 半徑配合巡航速度',
         [
             ('TERRAIN_ENABLE',   cfg.terrain_enable,
              '1=啟用地形資料庫'),
             ('TERRAIN_FOLLOW',   cfg.terrain_follow,
              '1=AUTO/RTL 貼地飛 (低空突防)'),
             ('TERRAIN_SPACING',  cfg.terrain_spacing,
              '地形柵格解析度 (m)'),
             ('WP_RADIUS',        cfg.wp_radius,
              f'航點通過半徑 (建議 = cruise × 1.5 = {cfg.wp_radius}m)'),
             ('WP_LOITER_RAD',    cfg.wp_loiter_rad,
              '盤旋半徑 (m)'),
             ('WP_MAX_RADIUS',    cfg.wp_max_radius,
              '最大航點半徑上限 (m)'),
             ('NAVL1_PERIOD',     cfg.navl1_period,
              'L1 導航週期 (越小越緊貼航線)'),
         ]),

        # ─── 區塊 4：空速 / 高度 ─────────────────────────────
        ('4. 空速與高度 — Shahed 類典型值 (35~50 m/s 巡航)',
         [
             ('AIRSPEED_CRUISE',  cfg.cruise_airspeed_mps,
              '巡航空速 (m/s)'),
             ('ARSPD_FBW_MIN',    cfg.stall_airspeed_mps,
              'FBW 最低空速 (失速保護)'),
             ('ARSPD_FBW_MAX',    cfg.max_airspeed_mps,
              'FBW 最大空速'),
             # TRIM_ARSPD_CM 單位為 cm/s
             ('TRIM_ARSPD_CM',    cfg.cruise_airspeed_mps * 100.0,
              '平飛目標空速 (cm/s)'),
             ('ALT_HOLD_RTL',     -1,
              '-1=RTL 維持當前高度 (禁用自動爬升)'),
             ('STALL_PREVENTION', 1,
              '1=啟用失速保護 (仍不會 RTL)'),
         ]),
    ]


def _format_parm_line(name: str, value: float, comment: str = '',
                      name_width: int = 20) -> str:
    """格式化單行 .parm

    ArduPilot 的 .parm 格式：``NAME    VALUE`` (空白/tab 分隔)，
    # 之後為註解。為了與 Mission Planner 的 ``.param`` 格式相容，
    採用「空白對齊 + 行尾註解」。
    """
    # 整數型直接不帶小數；浮點保留必要精度
    if isinstance(value, int) or value == int(value):
        val_str = f'{int(value)}'
    else:
        val_str = f'{value:g}'
    line = f'{name:<{name_width}} {val_str}'
    if comment:
        # 對齊到欄位 40 附近，避免視覺凌亂
        pad = max(1, 40 - len(line))
        line = f'{line}{" " * pad}# {comment}'
    return line


def generate_owa_uav_parm(
    output_path: str,
    config: Optional[OWAParamConfig] = None,
    include_header: bool = True,
) -> str:
    """為 ArduPlane SITL 產生 OWA-UAV 戰術 .parm 檔。

    Parameters
    ----------
    output_path : str
        輸出檔路徑 (含檔名)，會自動建立父目錄
    config : OWAParamConfig, optional
        參數覆寫；None 表示全部使用預設值
    include_header : bool
        是否寫入區塊註解與時間戳 Header (預設 True，SITL 必須 False 時仍可讀)

    Returns
    -------
    str
        實際寫出的絕對路徑
    """
    if config is None:
        config = OWAParamConfig()

    sections = _build_parm_sections(config)

    os.makedirs(os.path.dirname(os.path.abspath(output_path)) or '.', exist_ok=True)

    lines: List[str] = []
    if include_header:
        lines += [
            '# ══════════════════════════════════════════════════════════════════',
            '# !!! WARNING / 警告 !!!',
            '# 本參數檔 "禁用 failsafe + 禁用 RTL"，僅供 SITL 模擬驗證用。',
            '# 嚴禁直接上傳至實機飛控 (ArduPlane) — 會導致訊號中斷時',
            '# 無人機「繼續朝預設座標飛行且無法召回」，造成重大安全事故。',
            '# 實機部署請使用 plane.parm 預設值 + FS_LONG_ACTN=1(RTL)。',
            '# ══════════════════════════════════════════════════════════════════',
            '# OWA-UAV (One-Way Attack) 長程打擊機戰術參數檔',
            '# 用途：ArduPlane SITL --defaults 疊加於 plane.parm 之上',
            f'# 生成時間：{datetime.now().isoformat(timespec="seconds")}',
            '# ',
            '# 核心戰術假設：',
            '#   - 內燃機 (ICE) 動力',
            '#   - 低空地形跟隨突防',
            '#   - 遭 GCS/GPS 失效時「繼續任務，絕不 RTL」 ← 安全風險',
            '# ══════════════════════════════════════════════════════════════════',
            '',
        ]
    # 執行階段也在 logger 警告一次
    logger.warning(
        '[OWA] 產生 SITL 戰術參數檔 (禁 RTL / 禁 failsafe)，'
        '請確認僅用於模擬，切勿上傳實機！'
    )

    for title, params in sections:
        lines.append(f'# ── {title} ' + '─' * max(0, 60 - len(title)))
        for name, value, comment in params:
            lines.append(_format_parm_line(name, value, comment))
        lines.append('')

    # ── Extra overrides ──────────────────────────────────────────
    if config.extra_params:
        lines.append('# ── 使用者自訂覆寫 ─────────────────────────────────────')
        for name, value in config.extra_params.items():
            lines.append(_format_parm_line(name, value, '(custom override)'))
        lines.append('')

    content = '\n'.join(lines) + '\n'

    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(content)

    abs_path = os.path.abspath(output_path)
    logger.info(
        f'[OWA] .parm 已輸出: {abs_path} '
        f'({sum(len(p) for _, p in sections)} 筆參數)'
    )
    return abs_path


# ═══════════════════════════════════════════════════════════════════════
#  Example — 直接執行即可產出預設與客製化兩份 .parm
# ═══════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    # ① 預設 OWA-UAV 參數檔 (通用)
    default_path = os.path.join('sitl', 'default_params', 'owa_uav_default.parm')
    generate_owa_uav_parm(default_path)

    # ② 客製：模擬 Shahed-136 (巡航 45 m/s，RPM 較高)
    shahed_cfg = OWAParamConfig(
        cruise_airspeed_mps=45.0,
        stall_airspeed_mps=25.0,
        max_airspeed_mps=62.0,
        cruise_alt_m=150,
        ice_idle_rpm=1400,
        ice_redline_rpm=9500,
        ice_start_pct=22,
        wp_radius=70,                # 高速機體用較大接受半徑
        extra_params={
            'SYSID_THISMAV': 201,    # 保留給多機 SITL 的獨立 ID
            'SIM_DRIFT_SPEED': 0,    # 關閉地面滑行漂移
        },
    )
    shahed_path = os.path.join('sitl', 'default_params', 'shahed136_like.parm')
    generate_owa_uav_parm(shahed_path, shahed_cfg)

    print('\n已產生 2 份 OWA-UAV 戰術參數檔：')
    print(f'  • {os.path.abspath(default_path)}')
    print(f'  • {os.path.abspath(shahed_path)}')
    print('\nSITL 啟動範例：')
    print('  ArduPlane.exe --model plane --defaults '
          '"sitl/default_params/plane.parm,sitl/default_params/owa_uav_default.parm"')
