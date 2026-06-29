"""終端同步打擊協調器（closed-loop STOT executor 的純邏輯核心）。

把 [[terminal_sync]] 幾何 + [[tot_controller]] 速度修正組成一個「每 tick 吃**全機**現況、
吐出各機指令」的狀態機。本檔**無 Qt、無 MAVLink** → 可單元測試；GCS 端薄驅動
（ui/controllers/strike_controller.py）只負責讀 FleetRegistry 餵 step()、把回傳指令發到 SITLLink。

── 各機之間的資訊互通（核心保證）─────────────────────────────────────────────
step() 的唯一輸入是「**全機**快照」FleetSnapshot；每架的指令都是**全機**狀態的函數：
  • 同步釋放閘    release gate = 全員都就位（all_staged）才放行 —— 任一機沒到，全體等
  • ToT 速度修正  每機目標速度由「最遠機的 ETA」決定（compute_speeds 看所有機）
  • 防撞分離監看  min separation = 全機兩兩 3D 比對，gating_sysid 指出誰在拖時間
亦即沒有任何一架是「只看自己」飛 —— 每架的行為都被其他所有機的即時狀態影響。
這就是經 GCS hub（FleetRegistry 共享黑板）中介的 inter-aircraft 資訊互通。

── 三段式機制（實測 38.7s → ~4s，見 [[terminal_sync]] docstring）──────────────
  STAGE    各機 GUIDED 飛到等距 push point（不同方位＝繞開），先到者盤旋等待
  RELEASE  全員就位後「同一刻」放行，平飛(分層高度)直撲目標
  CORRECT  飛行中持續以 ToT 修正各機空速 → 收斂同一命中時刻；偵測命中
  DONE     全機命中（或驅動端逾時）
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

import math

from core.strike.geometry import haversine, bearing_deg, destination
from core.strike.tot_controller import TimeOnTargetController
from core.strike.tgo_coordination import (
    negotiate_tgo, ROLE_SPRINT, ROLE_CRUISE, ROLE_BURN, ROLE_WEAVE,
)

# ── 階段常數 ──────────────────────────────────────────────────────────────
PHASE_STAGE = 'STAGE'
PHASE_RELEASE = 'RELEASE'
PHASE_CORRECT = 'CORRECT'
PHASE_DONE = 'DONE'


@dataclass
class FleetState:
    """單機即時狀態（由 GCS 端從 FleetRegistry 遙測轉入）。"""
    sysid: int
    lat: float
    lon: float
    alt_rel: float = 0.0
    ground_speed: float = 0.0
    ts: float = 0.0           # 此筆遙測的時間戳（s），供陳舊判斷（可選用）


@dataclass
class StrikeCommand:
    """協調器要驅動端對某機下達的指令（驅動端轉成 SITLLink 呼叫）。"""
    sysid: int
    kind: str                 # 'goto'（GUIDED 飛往 lat/lon/alt）| 'speed'（DO_CHANGE_SPEED）
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    speed: float = 0.0
    reason: str = ''          # 'stage' | 'release' | 'tot' —— 供日誌


@dataclass
class StepResult:
    """一個 tick 的協調輸出 + 共享態勢摘要（驅動端據此下令 + 顯示）。"""
    phase: str
    commands: List[StrikeCommand] = field(default_factory=list)
    staged: Set[int] = field(default_factory=set)
    impacted: Set[int] = field(default_factory=set)
    lost: Set[int] = field(default_factory=set)        # 久未回報遙測（視為失聯，不阻塞收尾）
    min_separation_m: float = float('inf')
    gating_sysid: Optional[int] = None     # 目前在拖累釋放/命中時刻的機（態勢互通的證據）
    seen_sysids: Set[int] = field(default_factory=set)
    note: str = ''
    # ── 牧羊犬網格 / t_go 協商態勢（供顯示）──
    tgo_s: float = 0.0                                   # 當前共同剩餘時間 τ = t* − t_now
    impact_time_s: Optional[float] = None               # 協商鎖定的共同命中時刻 t*
    roles: Dict[int, str] = field(default_factory=dict)  # sysid → sprint/cruise/burn
    energy: float = 0.0                                  # 協商 τ* 的總能量（相對）


class TerminalSyncCoordinator:
    """終端同步打擊狀態機。純邏輯、可重播、可單元測試。

    參數
    ----
    target_lat, target_lon : 目標座標
    push_points  : {sysid: (lat, lon)}  各機等距 push point（通常取 plan.pre_strike_lat/lon）
    approach_alt : {sysid: alt_rel}      各機分層平飛/命中高度（避撞）
    v_min, v_max : ToT 修正可用空速範圍（需 0 < v_min < v_max）
    arrive_m     : 視為「已到 push point」的半徑（越小釋放越等距，但需 > GUIDED 盤旋半徑）
    target_radius_m : 視為「命中」的水平半徑
    impact_buffer_s : 共同命中時刻緩衝（放大 → 修正餘裕大）
    stage_timeout_s : 進站逾時 → 強制釋放（避免單機卡住拖死全體）
    resend_goto_s   : GUIDED goto 重送間隔（防丟包；同點重送為冪等）
    speed_interval_s: ToT 速度下令節流間隔
    min_air_alt_m   : 高於此高度才納入防撞/修正（濾掉地面/剛起飛）
    """

    def __init__(
        self,
        target_lat: float, target_lon: float,
        push_points: Dict[int, Tuple[float, float]],
        approach_alt: Dict[int, float],
        *,
        v_min: float = 12.0, v_max: float = 22.0,
        v_eff: float = 0.0,                  # 最佳續航速度（能耗最小點）；0=自動取 (v_min+v_max)/2
        arrive_m: float = 250.0,
        target_radius_m: float = 150.0,
        final_lock_m: float = 350.0,         # 進入此半徑即鎖定直撲（不再耗時機動，避免擺進目標）
        time_burn_mode: str = 'WEAVE',       # 耗時手段：'WEAVE'=S 型機動（預設）/ 'LOITER'=原地盤旋
        weave_period_s: float = 10.0,        # S 機動左右換邊週期（成 zigzag）
        weave_lookahead_m: float = 350.0,    # S 機動 carrot 前置距離
        weave_resend_s: float = 2.0,         # S 機動 carrot 重送間隔（航向需較勤更新）
        impact_buffer_s: float = 4.0,
        stage_timeout_s: float = 240.0,
        resend_goto_s: float = 5.0,
        speed_interval_s: float = 1.0,
        min_air_alt_m: float = 15.0,
        lost_timeout_s: float = 20.0,
    ) -> None:
        if not push_points:
            raise ValueError("push_points 不可為空")
        missing = set(push_points) - set(approach_alt)
        if missing:
            raise ValueError(f"approach_alt 缺少 sysid {sorted(missing)} 的高度")
        if target_radius_m <= 0 or arrive_m <= 0:
            raise ValueError("arrive_m / target_radius_m 需 > 0")

        self.tlat, self.tlon = float(target_lat), float(target_lon)
        self.push = {int(s): (float(p[0]), float(p[1])) for s, p in push_points.items()}
        self.alt = {int(s): float(a) for s, a in approach_alt.items()}
        self.sysids: Set[int] = set(self.push)
        self.arrive_m = float(arrive_m)
        self.target_radius_m = float(target_radius_m)
        self.final_lock_m = float(final_lock_m)
        self.time_burn_mode = str(time_burn_mode).upper()
        self.weave_period_s = float(weave_period_s)
        self.weave_lookahead_m = float(weave_lookahead_m)
        self.weave_resend_s = float(weave_resend_s)
        self.resend_goto_s = float(resend_goto_s)
        self.speed_interval_s = float(speed_interval_s)
        self.stage_timeout_s = float(stage_timeout_s)
        self.min_air_alt_m = float(min_air_alt_m)
        self.lost_timeout_s = float(lost_timeout_s)
        if not (0 < v_min < v_max):
            raise ValueError(f"need 0 < v_min < v_max, got {v_min}, {v_max}")
        self.v_min = float(v_min)
        self.v_max = float(v_max)
        self.v_eff = float(v_eff) if v_eff > 0 else 0.5 * (v_min + v_max)
        self.impact_buffer_s = float(impact_buffer_s)

        # 保留 ToT 控制器（向後相容；驅動端讀 coord.tot.v_min/v_max）。終端制導已改用
        # t_go 協商 + 牧羊犬網格（見 _step_release/_step_correct）。
        self.tot = TimeOnTargetController(
            self.tlat, self.tlon, v_min=v_min, v_max=v_max,
            terminal_range_m=self._stage_radius() + 600.0,
            impact_buffer_s=impact_buffer_s,
        )

        # ── 可變狀態 ──
        self.phase: str = PHASE_STAGE
        self.staged: Set[int] = set()
        self.impacted: Set[int] = set()
        self._stage_start_t: Optional[float] = None
        self._last_goto_t: Dict[int, float] = {}
        self._released: Set[int] = set()
        self._last_speed_t: float = -1e9
        self._last_seen: Dict[int, float] = {}   # sysid → 最近一次有遙測的 t（失聯判斷）
        self._t_star: Optional[float] = None     # 協商鎖定的共同命中時刻 t*（絕對秒）
        self._energy: float = 0.0                # 協商 τ* 的總能量（顯示用）
        self._first_tlm_t: Optional[float] = None  # 首次收到任一機遙測的 t（失聯寬限基準）

    # ── 幾何小工具 ────────────────────────────────────────────────────────
    def _stage_radius(self) -> float:
        """push point 距目標的代表距離（取各機平均；通常皆等距）。"""
        if not self.push:
            return 1500.0
        ds = [haversine(self.tlat, self.tlon, p[0], p[1]) for p in self.push.values()]
        return sum(ds) / len(ds)

    def _dist_to_target(self, st: FleetState) -> float:
        return haversine(st.lat, st.lon, self.tlat, self.tlon)

    def _min_separation(self, states: Dict[int, FleetState]) -> float:
        """全機兩兩 3D 最小間隔（只看仍在空中、未命中者）。防撞態勢的核心指標。"""
        air = [s for sid, s in states.items()
               if sid not in self.impacted and s.alt_rel > self.min_air_alt_m]
        best = float('inf')
        for i in range(len(air)):
            for j in range(i + 1, len(air)):
                a, b = air[i], air[j]
                dh = haversine(a.lat, a.lon, b.lat, b.lon)
                d3 = (dh * dh + (a.alt_rel - b.alt_rel) ** 2) ** 0.5
                if d3 < best:
                    best = d3
        return best

    def _want_goto(self, sysid: int, t_now: float) -> bool:
        """是否該（重）送此機的 GUIDED goto（首次或超過重送間隔）。"""
        last = self._last_goto_t.get(sysid)
        return last is None or (t_now - last) >= self.resend_goto_s

    def _lost(self, t_now: float) -> Set[int]:
        """久未回報遙測（> lost_timeout_s）而尚未命中的機 → 視為失聯。

        失聯機不得「永久阻塞」釋放/收尾（否則單機掉線會卡死全隊到驅動端 900s 逾時）。
        從未出現過遙測的機也算失聯（避免規劃含未連線 sysid 時整隊卡在 STAGE）。
        """
        out: Set[int] = set()
        for sid in self.sysids:
            if sid in self.impacted:
                continue
            seen = self._last_seen.get(sid)
            if seen is None:
                # 啟動寬限：以「首次收到任一機遙測」為基準計時（非固定 t=0），
                # 兼容較慢的任務初始化；完全沒有遙測時退回以 t=0 起算，避免永久等待。
                base = self._first_tlm_t if self._first_tlm_t is not None else 0.0
                if t_now - base > self.lost_timeout_s:
                    out.add(sid)
            elif (t_now - seen) > self.lost_timeout_s:
                out.add(sid)
        return out

    # ── 主步進 ────────────────────────────────────────────────────────────
    def step(self, states: Dict[int, FleetState], t_now: float) -> StepResult:
        """吃全機快照，回傳本 tick 指令 + 共享態勢。states 鍵為 sysid。

        單次 step 內會把已觸發的階段轉換「鏈接」走完（STAGE→RELEASE→CORRECT），
        讓「全員就位」當下就送出釋放 goto + 首輪 ToT 修正，不浪費一個 tick。
        """
        # 更新「最近有遙測」時戳（失聯判斷的依據）+ 記首次遙測時刻（失聯寬限基準）
        if states and self._first_tlm_t is None:
            self._first_tlm_t = t_now
        for sid in states:
            if sid in self.sysids:
                self._last_seen[sid] = t_now

        res = StepResult(phase=self.phase, seen_sysids=set(states))
        res.min_separation_m = self._min_separation(states)

        visited: Set[str] = set()
        while self.phase not in visited and self.phase != PHASE_DONE:
            visited.add(self.phase)
            if self.phase == PHASE_STAGE:
                self._step_stage(states, t_now, res)
            elif self.phase == PHASE_RELEASE:
                self._step_release(states, t_now, res)
            elif self.phase == PHASE_CORRECT:
                self._step_correct(states, t_now, res)

        res.phase = self.phase
        res.staged = set(self.staged)
        res.impacted = set(self.impacted)
        res.lost = self._lost(t_now)
        return res

    # ── STAGE：各機飛往等距 push point，先到者就位 ──────────────────────────
    def _step_stage(self, states, t_now, res: StepResult) -> None:
        if self._stage_start_t is None:
            self._stage_start_t = t_now

        worst_sid, worst_d = None, -1.0
        for sid in self.sysids:
            st = states.get(sid)
            if st is None:
                worst_sid, worst_d = sid, float('inf')   # 沒遙測 = 最該等的（態勢互通）
                continue
            pp = self.push[sid]
            d_push = haversine(st.lat, st.lon, pp[0], pp[1])
            if d_push <= self.arrive_m:
                self.staged.add(sid)
            else:
                if d_push > worst_d:
                    worst_sid, worst_d = sid, d_push
                if self._want_goto(sid, t_now):
                    res.commands.append(StrikeCommand(
                        sysid=sid, kind='goto', lat=pp[0], lon=pp[1],
                        alt=self.alt[sid], reason='stage'))
                    self._last_goto_t[sid] = t_now

        # 失聯機不阻塞釋放：全員就位 = 已就位 ∪ 失聯 ⊇ 全體
        lost = self._lost(t_now)
        all_in = (self.staged | lost) >= self.sysids
        timed_out = (t_now - self._stage_start_t) >= self.stage_timeout_s
        if all_in or timed_out:
            self.phase = PHASE_RELEASE
            res.note = ('全員就位 → 同步釋放' if all_in
                        else f'進站逾時 {self.stage_timeout_s:.0f}s → 強制釋放({len(self.staged)}/{len(self.sysids)})')
        else:
            res.note = f'進站中 {len(self.staged)}/{len(self.sysids)}，等待 UAV{worst_sid}'

    # ── t_go 協商小工具 ────────────────────────────────────────────────────
    def _remaining_distances(self, states) -> Dict[int, float]:
        """非命中、有遙測機到目標的剩餘距離 {sysid: r}（t_go 協商輸入）。"""
        out: Dict[int, float] = {}
        for sid in self.sysids:
            if sid in self.impacted:
                continue
            st = states.get(sid)
            if st is not None:
                out[sid] = self._dist_to_target(st)
        return out

    def _negotiate_lock(self, states, t_now) -> None:
        """協商「能量最省」的共同剩餘時間 τ*（協同變數），鎖定 t* = t_now + τ*。"""
        rs = self._remaining_distances(states)
        if not rs:
            return
        plan = negotiate_tgo(rs, self.v_min, self.v_max, self.v_eff)
        self._t_star = t_now + plan.tgo_common_s
        self._energy = plan.energy

    # ── RELEASE：以 t_go 協商出能量最省的共同命中時刻 t*（協同變數），鎖定後入網格 ──
    def _step_release(self, states, t_now, res: StepResult) -> None:
        self._negotiate_lock(states, t_now)
        self.phase = PHASE_CORRECT
        if self._t_star is not None:
            res.note = (f'已協商能量最省的共同命中 t*（τ={self._t_star - t_now:.0f}s）'
                        f'→ 牧羊犬網格收斂')
        else:
            res.note = '尚無遙測可協商，待 CORRECT 階段補算'

    # ── CORRECT：牧羊犬網格 —— 遠機全速直線衝刺、近機盤旋耗時，全機收斂到協商 t* ──
    def _step_correct(self, states, t_now, res: StepResult) -> None:
        lost = self._lost(t_now)
        # 命中偵測 + gating（最遠、未命中、未失聯者）
        worst_sid, worst_d = None, -1.0
        hits = []
        for sid in self.sysids:
            if sid in self.impacted:
                continue
            st = states.get(sid)
            if st is None:
                continue
            d = self._dist_to_target(st)
            if d <= self.target_radius_m:
                self.impacted.add(sid)
                hits.append(f'UAV{sid} 命中 (d={d:.0f}m)')
            elif sid not in lost and d > worst_d:
                worst_sid, worst_d = sid, d
        if hits:
            res.note = '；'.join(hits)            # 同一 tick 多機命中皆保留，不互相覆蓋
        res.gating_sysid = worst_sid

        # t* 鎖定（首次補算）。鎖定後不再每 tick 以樂觀 v_max 外推 —— 那會讓 t* 微幅漂移
        # 累積、鬆散命中散度；逾時(t_now>t*)時 τ 自然夾到 0.5 → 全機 sprint 直撲（見下）。
        if self._t_star is None:
            self._negotiate_lock(states, t_now)

        do_speed = (t_now - self._last_speed_t) >= self.speed_interval_s
        if do_speed:
            self._last_speed_t = t_now

        if self._t_star is not None:
            tau = max(self._t_star - t_now, 0.5)
            res.tgo_s = tau
            res.impact_time_s = self._t_star
            res.energy = self._energy
            for sid in self.sysids:
                if sid in self.impacted or sid in lost:
                    continue
                st = states.get(sid)
                if st is None or st.alt_rel <= self.min_air_alt_m:
                    continue
                r = self._dist_to_target(st)
                # v_close = 準時抵達 t* 所需的「對目標接近速率」(closing rate)。
                # 統一制導：以速度 v 與航向偏角 θ 控制 closing = v·cosθ = v_close。
                #   dr/dt = -v_close = -r/(t*-t_now) → r = K·(t*-t_now) → t→t* 時 r→0（精準對時）
                v_close = r / tau
                if r <= self.final_lock_m:
                    # 終端鎖定：直撲，速度盡力（不再 S 機動，避免擺幅掃進目標）
                    self._released.add(sid)
                    if self._want_goto(sid, t_now):
                        res.commands.append(StrikeCommand(
                            sysid=sid, kind='goto', lat=self.tlat, lon=self.tlon,
                            alt=self.alt[sid], reason='release'))
                        self._last_goto_t[sid] = t_now
                    if do_speed:
                        v = min(self.v_max, max(self.v_min, v_close))
                        res.commands.append(StrikeCommand(
                            sysid=sid, kind='speed', speed=v, reason='tot'))
                    res.roles[sid] = ROLE_SPRINT if v_close > self.v_eff else ROLE_CRUISE
                elif v_close < self.v_min:
                    # 超前到「直線即使飛最慢 v_min 仍會早到」→ 需耗時。weave 僅在真正需要時啟用，
                    # 故等距環(v_close≈v_eff>v_min)全程直線、不抖動。
                    self._released.discard(sid)
                    if self.time_burn_mode == 'LOITER':
                        # 原地盤旋耗時（守在當前位置；ArduPlane 盤旋行為穩定，極端落差下較穩健）
                        if self._want_goto(sid, t_now):
                            res.commands.append(StrikeCommand(
                                sysid=sid, kind='goto', lat=st.lat, lon=st.lon,
                                alt=self.alt[sid], reason='loiter'))
                            self._last_goto_t[sid] = t_now
                        if do_speed:
                            res.commands.append(StrikeCommand(
                                sysid=sid, kind='speed', speed=self.v_min, reason='loiter'))
                    else:
                        # S 型機動精修耗時：航向偏角 θ=acos(v_close/v_act)，對目標接近速率
                        # = v_act·cosθ = v_close（精準對時）；左右交替換邊成 zigzag S。
                        # 偏角用「實際對地速度」算 → 即使 SITL 未真正慢到 v_min 仍成立（保真穩健）。
                        v_act = max(st.ground_speed, self.v_min)
                        cos_t = max(0.0, min(1.0, v_close / v_act))
                        theta = math.degrees(math.acos(cos_t))
                        side = 1.0 if (int(t_now / self.weave_period_s) % 2 == 0) else -1.0
                        brg = bearing_deg(st.lat, st.lon, self.tlat, self.tlon)
                        carrot = destination(st.lat, st.lon, brg + side * theta,
                                             self.weave_lookahead_m)
                        if (t_now - self._last_goto_t.get(sid, -1e9)) >= self.weave_resend_s:
                            res.commands.append(StrikeCommand(
                                sysid=sid, kind='goto', lat=carrot[0], lon=carrot[1],
                                alt=self.alt[sid], reason='weave'))
                            self._last_goto_t[sid] = t_now
                        if do_speed:
                            res.commands.append(StrikeCommand(
                                sysid=sid, kind='speed', speed=self.v_min, reason='weave'))
                    res.roles[sid] = ROLE_WEAVE
                else:
                    # v_close ≥ v_min → 直線撲向目標，速度 = 接近速率（落後 sprint、適中 cruise）
                    self._released.add(sid)
                    if self._want_goto(sid, t_now):
                        res.commands.append(StrikeCommand(
                            sysid=sid, kind='goto', lat=self.tlat, lon=self.tlon,
                            alt=self.alt[sid], reason='release'))
                        self._last_goto_t[sid] = t_now
                    if do_speed:
                        v = min(self.v_max, max(self.v_min, v_close))
                        res.commands.append(StrikeCommand(
                            sysid=sid, kind='speed', speed=v, reason='tot'))
                    res.roles[sid] = ROLE_SPRINT if v_close > self.v_eff * 1.02 else ROLE_CRUISE

        # 收尾：命中 ∪ 失聯 ⊇ 全體（失聯機不得卡死整隊；命中數另由驅動端統計散度）
        if (self.impacted | lost) >= self.sysids:
            self.phase = PHASE_DONE
            res.note = (f'全機收尾：命中 {len(self.impacted)}/{len(self.sysids)}'
                        + (f'，失聯 {sorted(lost)}' if lost else '')) or res.note

    # ── 對外摘要（驅動端日誌/狀態列用）────────────────────────────────────
    def progress(self) -> str:
        return (f'phase={self.phase} staged={len(self.staged)}/{len(self.sysids)} '
                f'impacted={len(self.impacted)}/{len(self.sysids)}')
