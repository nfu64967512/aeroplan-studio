"""
XWingSITLParamGenerator — X-Wing 尾座式 VTOL SITL 參數生成器
===============================================================

適用機型：中科院「勁蜂三型」類 **Copter Motor Tailsitter (X-Wing)**
    * 4 顆馬達共用 VTOL + Fixed-Wing 模式 (無後推馬達)
    * 無任何控制面 (無副翼/升降舵/方向舵)
    * 前飛時整機低頭 90° 轉換為水平姿態
    * 所有姿態控制 (roll/pitch/yaw) 完全靠馬達差速推力

物理約束推導
------------
    Q_TAILSIT_ENABLE = 2   → Copter Motor Tailsitter (純馬達差速)
                             選 1 會驅動不存在的伺服 → 翻滾墜毀

    Q_TAILSIT_MOTMX = 15   → 4 馬達全開 (bit 0|1|2|3 = 1+2+4+8)
                             X-Wing 沒有獨立後推馬達，FW 模式仍需全 4 顆

    Q_FRAME_TYPE = 17      → NYT QUAD X (No Yaw Torque)
                             X-Wing 於 FW 姿態時偏航靠機身風舵 (body yaw)
                             不靠馬達反扭矩，必須關閉 yaw torque mixing

    Q_TAILSIT_ANGLE = 60°  → 低頭到 60° 時移交 ArduPlane 固定翼控制
                             機翼升力 ≈ sin(60°) ≈ 87%，馬達負擔合理
                             若等到 90° 才轉場，馬達推力差距過大會掉高

SITL 啟動方式
-------------
    ArduPlane.exe -f tailsitter \
        --defaults sitl/default_params/plane.parm,xwing_tailsitter.parm

MAVLink 對應參數 (ArduPilot 4.5+)
---------------------------------
    https://ardupilot.org/plane/docs/quadplane-tailsitters.html
    https://ardupilot.org/plane/docs/parameters.html#q-tailsit-enable
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional, Tuple

from utils.logger import get_logger

logger = get_logger()


# ═══════════════════════════════════════════════════════════════════════
#  參數配置
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class XWingTailsitterConfig:
    """X-Wing Tailsitter 戰術/飛行參數配置

    所有預設值均參考 ArduPilot 官方 tailsitter 文件 + 勁蜂三型類機型的
    實際飛行參數區間。可依實機特性覆寫。
    """

    # ═══════════════════ 1. 機架與 Tailsitter 啟用 ═══════════════════

    # QuadPlane 總開關 — 必須 1，否則 ArduPlane 完全不認得 Q_* 參數
    q_enable: int = 1

    # Tailsitter 類型
    #   0 = Disabled
    #   1 = Normal Tailsitter (有副翼/升降舵)
    #   2 = Copter Motor Tailsitter (純馬達差速) ← X-Wing 唯一選項
    q_tailsit_enable: int = 2

    # 機架 Class
    #   1 = Quad, 2 = Hexa, 3 = Octa, 4 = OctaQuad, 5 = Y6
    q_frame_class: int = 1

    # 機架 Type (對 Quad)
    #   0 = Plus, 1 = X, 2 = V, 3 = H, 4 = V-Tail, 5 = A-Tail, 17 = NYT QUAD X
    # 17 (NYT QUAD X) = No Yaw Torque Quad X:
    #   X-Wing 於 FW 水平姿態時，馬達反扭矩差會造成不希望的偏航
    #   NYT 關閉 yaw torque mixing，讓 yaw 只靠 body yaw (機身風舵效應)
    q_frame_type: int = 17

    # 前飛馬達遮罩 (bitmask)
    #   bit 0 = Motor 1, bit 1 = Motor 2, bit 2 = Motor 3, bit 3 = Motor 4
    #   15 (0b1111) = 全 4 顆馬達在 FW 模式仍運轉
    #   X-Wing 無獨立後推，必須 = 15
    q_tailsit_motmx: int = 15

    # ═══════════════════ 2. 轉換動態與角度 ═══════════════════

    # 前飛轉換角度閾值 (度)：pitch 低頭達此值即移交 ArduPlane FW 控制
    # 60° 是安全區間：機翼升力夠 + 留緩衝，避免 90° 瞬間斷層
    q_tailsit_angle: float = 60.0

    # 退場 (VTOL 轉) 轉換角度 (度)
    #   0 = 沿用 Q_TAILSIT_ANGLE (推薦)
    #   正值 = 另行指定 VT 模式的抬頭角閾值
    q_tailsit_ang_vt: float = 0.0

    # 前飛方向俯仰角速率 (度/秒)：轉 FW 時的低頭速度
    # 值越大越快進入水平姿態，但風險是推力突變
    q_tailsit_rat_fw: float = 30.0

    # VTOL 方向俯仰角速率 (度/秒)：轉 VTOL 時的抬頭速度
    # 抬頭比低頭需更快 (減速 + 重回懸停更敏捷)
    q_tailsit_rat_vt: float = 40.0

    # FW 過渡失敗超時 (秒)：若 q_tailsit_rat_fw 下仍未達 Q_TAILSIT_ANGLE，
    # 自動放棄 FW 過渡並回 VTOL
    q_trans_fail: float = 5.0

    # ═══════════════════ 3. 安全與 Q_OPTIONS ═══════════════════

    # Q_OPTIONS bitmask (重要 bit 說明)：
    #   bit 0 (1)      = Level Transition (水平轉場，不抬頭)
    #   bit 2 (4)      = Motors in FW Mode (FW 模式仍讓馬達轉)
    #   bit 3 (8)      = Airmode (低油門仍全權控制姿態)
    #   bit 5 (32)     = Disable Synthetic Airspeed
    #   bit 7 (128)    = Disarmed Yaw Tilt (關閉地面馬達扭轉 yaw)
    #   bit 8 (256)    = Delay Spoolup
    #   bit 14 (16384) = Enable FW VTOL (FW mode 可呼叫 VTOL motors)
    #   bit 16 (65536) = Disable Approach (停用固定翼盤旋進場)
    #                    → X-Wing 強制飛到目標上空垂直降落
    #   bit 18 (262144)= Only Arm in QMode (僅允許 VTOL 模式解鎖)
    #                    → 防止水平姿態誤解鎖造成地面暴衝
    q_options: int = (
        65536 +     # bit 16: Disable Approach
        262144      # bit 18: Only Arm in QMode (最關鍵安全鎖)
    )

    # ═══════════════════ 4. SERVO 通道 (4 顆馬達) ═══════════════════

    # X-Wing NYT QUAD X 的標準 SERVO function 映射：
    #   SERVO1 = Motor 1 (33)  右前
    #   SERVO2 = Motor 2 (34)  左後
    #   SERVO3 = Motor 3 (35)  左前
    #   SERVO4 = Motor 4 (36)  右後
    servo_motor_map: Dict[int, int] = field(default_factory=lambda: {
        1: 33, 2: 34, 3: 35, 4: 36,
    })

    # PWM 範圍 (1000~2000us 是 ESC 標準)
    pwm_min: int = 1000
    pwm_max: int = 2000
    pwm_trim: int = 1500

    # ═══════════════════ 5. 姿態控制增益 (X-Wing 專用) ═══════════════════

    # 多旋翼 (VTOL) 模式姿態 PID
    q_a_rat_rll_p: float = 0.15
    q_a_rat_rll_i: float = 0.10
    q_a_rat_rll_d: float = 0.004
    q_a_rat_pit_p: float = 0.15
    q_a_rat_pit_i: float = 0.10
    q_a_rat_pit_d: float = 0.004
    q_a_rat_yaw_p: float = 0.30
    q_a_rat_yaw_i: float = 0.02
    q_a_rat_yaw_d: float = 0.000

    # 最大傾角 (度)：VTOL 模式最多能傾多少 (過渡期間會暫時超過)
    q_angle_max_deg: float = 30.0

    # VTOL 爬降率上限 (m/s)
    q_velz_max: float = 3.0
    q_pilot_accel_z: float = 250.0    # cm/s²

    # ═══════════════════ 6. 空速與推力 ═══════════════════

    # 固定翼巡航空速 (m/s)
    airspeed_cruise_mps: float = 18.0
    airspeed_min_mps: float = 12.0
    airspeed_max_mps: float = 25.0
    trim_throttle_pct: int = 50
    stall_prevention: int = 1

    # 失速後仍有控制力 (X-Wing 全靠馬達，所以此值可關)
    use_reverse_thrust: int = 0

    # ═══════════════════ 7. 自訂 overrides ═══════════════════

    extra_params: Dict[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        # 邏輯檢查
        if self.q_tailsit_enable != 2:
            logger.warning(
                '[XWing] Q_TAILSIT_ENABLE != 2 會使 ArduPlane 驅動不存在的'
                ' 副翼伺服 → 翻滾墜毀！請勿更改。'
            )
        if self.q_tailsit_motmx != 15:
            logger.warning(
                '[XWing] Q_TAILSIT_MOTMX != 15 (0b1111) '
                '可能導致 FW 模式馬達關閉 → 失控。'
            )
        if self.q_frame_type != 17:
            logger.warning(
                '[XWing] Q_FRAME_TYPE != 17 (NYT QUAD X) 可能在 FW 姿態下'
                '馬達反扭矩造成意外偏航。'
            )
        # bit 18 檢查
        if not (self.q_options & 262144):
            logger.warning(
                '[XWing] Q_OPTIONS 未啟用 bit 18 (Only Arm in QMode)！'
                '水平姿態可能誤解鎖造成地面暴衝。'
            )


# ═══════════════════════════════════════════════════════════════════════
#  XWingSITLParamGenerator — 核心類別
# ═══════════════════════════════════════════════════════════════════════

class XWingSITLParamGenerator:
    """X-Wing Copter-Motor Tailsitter SITL 參數檔生成器

    典型用法::

        from mission.xwing_tailsitter_params import (
            XWingSITLParamGenerator, XWingTailsitterConfig,
        )

        # 預設配置 (勁蜂三型類)
        gen = XWingSITLParamGenerator()
        gen.export('sitl/default_params/xwing_tailsitter.parm')

        # 客製化
        cfg = XWingTailsitterConfig(
            q_tailsit_angle=55.0,            # 更保守的轉場角
            airspeed_cruise_mps=20.0,
        )
        gen = XWingSITLParamGenerator(cfg)
        gen.export('sitl/default_params/xwing_v2.parm')

    Parameters
    ----------
    config :
        XWingTailsitterConfig 實例；None = 使用預設
    """

    # ─────────────────────────────────────────────────────────────────
    #  建構
    # ─────────────────────────────────────────────────────────────────
    def __init__(self, config: Optional[XWingTailsitterConfig] = None):
        self.config = config or XWingTailsitterConfig()

    # ─────────────────────────────────────────────────────────────────
    #  核心：分區組合所有參數 (section, [(name, value, comment), ...])
    # ─────────────────────────────────────────────────────────────────
    def _build_sections(self) -> List[Tuple[str, List[Tuple[str, float, str]]]]:
        """組織為分區結構便於輸出可讀的 .parm 檔"""
        c = self.config
        return [
            # ─────────────────────────────────────────────────────
            ('1. QuadPlane 總開關 + Tailsitter 啟用',
             [
                 ('Q_ENABLE',         c.q_enable,
                  '1=啟用 QuadPlane (必要)'),
                 ('Q_TAILSIT_ENABLE', c.q_tailsit_enable,
                  '2=Copter Motor Tailsitter (純馬達差速，X-Wing 必須=2)'),
             ]),

            # ─────────────────────────────────────────────────────
            ('2. 機架型態 (NYT QUAD X — 無偏航扭矩)',
             [
                 ('Q_FRAME_CLASS',    c.q_frame_class,
                  '1=Quad'),
                 ('Q_FRAME_TYPE',     c.q_frame_type,
                  '17=NYT QUAD X (No Yaw Torque，FW 姿態靠 body yaw)'),
                 ('Q_TAILSIT_MOTMX',  c.q_tailsit_motmx,
                  '15 (0b1111) = 4 馬達全開於 FW 模式 (X-Wing 必須=15)'),
             ]),

            # ─────────────────────────────────────────────────────
            ('3. 轉換角度與角速率 (Transition Dynamics)',
             [
                 ('Q_TAILSIT_ANGLE',  c.q_tailsit_angle,
                  '低頭達此角度 (度) 移交 FW 控制，推薦 60° (機翼升力充分+緩衝)'),
                 ('Q_TAILSIT_ANG_VT', c.q_tailsit_ang_vt,
                  '0=VTOL 轉場沿用 Q_TAILSIT_ANGLE'),
                 ('Q_TAILSIT_RAT_FW', c.q_tailsit_rat_fw,
                  '轉 FW 的低頭角速度 (°/s)'),
                 ('Q_TAILSIT_RAT_VT', c.q_tailsit_rat_vt,
                  '轉 VTOL 的抬頭角速度 (°/s；比 FW 快以利急停)'),
                 ('Q_TRANS_FAIL',     c.q_trans_fail,
                  'FW 過渡超時 (秒)，逾時回 VTOL'),
             ]),

            # ─────────────────────────────────────────────────────
            ('4. Q_OPTIONS 安全/行為 bitmask',
             [
                 ('Q_OPTIONS', c.q_options,
                  f'{c.q_options} = bit18(262144, 僅 VTOL 解鎖, 防地面暴衝)'
                  f' + bit16(65536, 停用 FW 盤旋進場, 強制上空垂直降落)'),
             ]),

            # ─────────────────────────────────────────────────────
            ('5. SERVO 輸出 (4 顆共用馬達)',
             self._servo_params()),

            # ─────────────────────────────────────────────────────
            ('6. 多旋翼姿態 PID (VTOL + 過渡期用)',
             [
                 ('Q_A_RAT_RLL_P',   c.q_a_rat_rll_p,   'Roll  rate P'),
                 ('Q_A_RAT_RLL_I',   c.q_a_rat_rll_i,   'Roll  rate I'),
                 ('Q_A_RAT_RLL_D',   c.q_a_rat_rll_d,   'Roll  rate D'),
                 ('Q_A_RAT_PIT_P',   c.q_a_rat_pit_p,   'Pitch rate P'),
                 ('Q_A_RAT_PIT_I',   c.q_a_rat_pit_i,   'Pitch rate I'),
                 ('Q_A_RAT_PIT_D',   c.q_a_rat_pit_d,   'Pitch rate D'),
                 ('Q_A_RAT_YAW_P',   c.q_a_rat_yaw_p,   'Yaw   rate P'),
                 ('Q_A_RAT_YAW_I',   c.q_a_rat_yaw_i,   'Yaw   rate I'),
                 ('Q_A_RAT_YAW_D',   c.q_a_rat_yaw_d,   'Yaw   rate D'),
                 ('Q_ANGLE_MAX',     c.q_angle_max_deg * 100.0,
                  'VTOL 最大傾角 (centi-degrees)'),
                 ('Q_VELZ_MAX',      c.q_velz_max * 100.0,
                  'VTOL 爬降率上限 (cm/s)'),
                 ('Q_PILOT_ACCEL_Z', c.q_pilot_accel_z,
                  'Pilot 垂直加速度上限 (cm/s²)'),
             ]),

            # ─────────────────────────────────────────────────────
            ('7. 固定翼空速與推力',
             [
                 ('AIRSPEED_CRUISE', c.airspeed_cruise_mps,
                  'FW 巡航空速 (m/s)'),
                 ('ARSPD_FBW_MIN',   c.airspeed_min_mps,
                  'FW 最低空速 (m/s，低於會拉高度)'),
                 ('ARSPD_FBW_MAX',   c.airspeed_max_mps,
                  'FW 最大空速 (m/s)'),
                 ('TRIM_THROTTLE',   c.trim_throttle_pct,
                  '平飛油門 %'),
                 ('STALL_PREVENTION', c.stall_prevention,
                  '1=啟用失速保護'),
                 ('USE_REV_THRUST',  c.use_reverse_thrust,
                  '0=不反推 (X-Wing 靠抬頭減速)'),
             ]),
        ]

    def _servo_params(self) -> List[Tuple[str, float, str]]:
        """SERVO1~4 function + PWM 上下限"""
        c = self.config
        rows: List[Tuple[str, float, str]] = []
        for srv_idx, motor_func in sorted(c.servo_motor_map.items()):
            rows.append((
                f'SERVO{srv_idx}_FUNCTION', motor_func,
                f'Motor {srv_idx} (func {motor_func})',
            ))
            rows.append((
                f'SERVO{srv_idx}_MIN',  c.pwm_min,   'PWM min'
            ))
            rows.append((
                f'SERVO{srv_idx}_MAX',  c.pwm_max,   'PWM max'
            ))
            rows.append((
                f'SERVO{srv_idx}_TRIM', c.pwm_trim,  'PWM trim'
            ))
        return rows

    # ─────────────────────────────────────────────────────────────────
    #  格式化
    # ─────────────────────────────────────────────────────────────────
    @staticmethod
    def _format_value(v: float) -> str:
        """整數不帶小數，浮點保留必要精度"""
        if isinstance(v, int) or float(v).is_integer():
            return f'{int(v)}'
        return f'{v:g}'

    @staticmethod
    def _format_line(name: str, value: float, comment: str = '',
                     name_width: int = 20) -> str:
        """格式化單行：`NAME    VALUE    # comment`"""
        val_str = XWingSITLParamGenerator._format_value(value)
        line = f'{name:<{name_width}} {val_str}'
        if comment:
            pad = max(1, 40 - len(line))
            line = f'{line}{" " * pad}# {comment}'
        return line

    # ─────────────────────────────────────────────────────────────────
    #  Public API: 匯出
    # ─────────────────────────────────────────────────────────────────
    def export(self, output_path: str, include_header: bool = True) -> str:
        """將參數檔寫出至 output_path

        Parameters
        ----------
        output_path :
            輸出路徑 (若無副檔名，自動補 .parm)
        include_header :
            是否寫入區塊註解 + 時間戳記 (除錯/版本控制用)

        Returns
        -------
        str : 實際寫出的絕對路徑
        """
        if not output_path.endswith('.parm'):
            output_path = output_path + '.parm'

        parent = os.path.dirname(os.path.abspath(output_path))
        if parent:
            os.makedirs(parent, exist_ok=True)

        lines: List[str] = []

        # ── Header ─────────────────────────────────────────────
        if include_header:
            lines += [
                '# ══════════════════════════════════════════════════════════════════',
                '# AeroPlan Studio — X-Wing Copter Motor Tailsitter',
                '# ',
                '# 適用機型：中科院 勁蜂三型 類純馬達差速 X-Wing 尾座式 VTOL',
                '#   * 4 顆馬達共用 VTOL + FW 模式 (無後推馬達)',
                '#   * 無副翼/升降舵/方向舵，所有姿態全靠馬達差速',
                '#   * 前飛時整機低頭 90° 水平',
                '# ',
                '# 用途：ArduPlane SITL --defaults 疊加載入',
                '# ',
                '# SITL 啟動指令：',
                '#   ArduPlane.exe -f tailsitter \\',
                '#       --defaults "sitl/default_params/plane.parm,',
                '#                   sitl/default_params/xwing_tailsitter.parm"',
                '# ',
                '# 關鍵物理約束：',
                '#   Q_TAILSIT_ENABLE = 2  (純馬達差速，禁動不存在的副翼)',
                '#   Q_TAILSIT_MOTMX  = 15 (FW 模式仍 4 馬達全開)',
                '#   Q_FRAME_TYPE     = 17 (NYT QUAD X，無 yaw 扭矩)',
                '#   Q_TAILSIT_ANGLE  = 60 (低頭 60° 即移交 FW，留 30° 緩衝)',
                '#   Q_OPTIONS bit18  = 262144 (僅 VTOL 解鎖，防地面暴衝)',
                '# ',
                f'# 生成時間：{datetime.now().isoformat(timespec="seconds")}',
                '# ══════════════════════════════════════════════════════════════════',
                '',
            ]

        # ── Sections ──────────────────────────────────────────
        for title, params in self._build_sections():
            lines.append(f'# ── {title} '
                          + '─' * max(1, 60 - len(title)))
            for name, val, comment in params:
                lines.append(self._format_line(name, val, comment))
            lines.append('')

        # ── 自訂 overrides ─────────────────────────────────────
        if self.config.extra_params:
            lines.append('# ── 使用者自訂覆寫 ─────────────────────────────────')
            for name, val in self.config.extra_params.items():
                lines.append(self._format_line(name, val, '(override)'))
            lines.append('')

        # 寫檔
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines) + '\n')

        abs_path = os.path.abspath(output_path)
        n_params = sum(len(p) for _, p in self._build_sections()) + \
                   len(self.config.extra_params)
        logger.info(
            f'[XWing] X-Wing Tailsitter 參數檔已輸出: {abs_path} '
            f'({n_params} 筆參數)'
        )
        return abs_path

    def summary(self) -> str:
        """回傳關鍵參數摘要 (除錯/UI 顯示用)"""
        c = self.config
        return (
            f'X-Wing Tailsitter Config:\n'
            f'  Q_TAILSIT_ENABLE = {c.q_tailsit_enable}  (須=2)\n'
            f'  Q_TAILSIT_MOTMX  = {c.q_tailsit_motmx}  (須=15)\n'
            f'  Q_FRAME_TYPE     = {c.q_frame_type}  (NYT QUAD X)\n'
            f'  Q_TAILSIT_ANGLE  = {c.q_tailsit_angle}°\n'
            f'  Q_TAILSIT_RAT_FW = {c.q_tailsit_rat_fw}°/s\n'
            f'  Q_TAILSIT_RAT_VT = {c.q_tailsit_rat_vt}°/s\n'
            f'  Q_OPTIONS        = {c.q_options} (bit18={bool(c.q_options&262144)})\n'
            f'  Airspeed cruise  = {c.airspeed_cruise_mps} m/s\n'
        )


# ═══════════════════════════════════════════════════════════════════════
#  main() 執行範例
# ═══════════════════════════════════════════════════════════════════════

def main() -> int:
    """生成預設 + 客製化兩份 X-Wing tailsitter .parm 檔"""
    print('═' * 72)
    print(' X-Wing Copter Motor Tailsitter SITL 參數生成器')
    print('═' * 72)

    # ── ① 預設配置 (勁蜂三型類) ──────────────────────────
    default_path = os.path.join('sitl', 'default_params', 'xwing_tailsitter.parm')
    gen = XWingSITLParamGenerator()
    print()
    print(gen.summary())
    abs_default = gen.export(default_path)

    # ── ② 客製化：較保守轉場 + 較高巡航速 ─────────────────
    from dataclasses import replace  # 方便複製後微調
    custom_cfg = XWingTailsitterConfig(
        q_tailsit_angle=55.0,            # 55° 更保守
        q_tailsit_rat_fw=25.0,           # 低頭慢一點
        q_tailsit_rat_vt=45.0,           # 抬頭更快 (緊急減速)
        airspeed_cruise_mps=22.0,
        airspeed_max_mps=30.0,
        extra_params={
            'SYSID_THISMAV': 101,        # SITL 實例識別
            'SIM_DRIFT_SPEED': 0,        # 停止地面漂移
            'LOG_BITMASK':   65535,      # 全 log 用於除錯
        },
    )
    custom_path = os.path.join('sitl', 'default_params', 'xwing_conservative.parm')
    abs_custom = XWingSITLParamGenerator(custom_cfg).export(custom_path)

    # ── 輸出摘要 ─────────────────────────────────────
    print()
    print('═' * 72)
    print(' 已生成 2 份 X-Wing Tailsitter SITL 參數檔')
    print('═' * 72)
    print(f'  [1] 預設 (勁蜂三型)  : {abs_default}')
    print(f'  [2] 保守調校版        : {abs_custom}')
    print()
    print('SITL 啟動指令範例：')
    print('  # Windows')
    print('  ArduPlane.exe -f tailsitter ^')
    print('      --defaults "sitl/default_params/plane.parm,^')
    print('                  sitl/default_params/xwing_tailsitter.parm"')
    print()
    print('  # Linux / WSL')
    print('  sim_vehicle.py -v ArduPlane -f tailsitter \\')
    print('      --add-param-file=xwing_tailsitter.parm')
    print()
    print('Mission Planner 連線後驗證項目：')
    print('  1. QSTABILIZE 模式下 ARM → 4 馬達同步怠速 (應可解鎖)')
    print('  2. 水平姿態下嘗試 ARM → 應被拒絕 (Q_OPTIONS bit18)')
    print('  3. VTOL takeoff 至 50m → 切 FBWA 觀察低頭過渡')
    print('  4. Pitch 達 60° 時 log 應顯示 "transitioned to FW"')
    print('  5. FW 模式下 Q_TAILSIT_MOTMX=15 確保 4 馬達持續轉')

    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
