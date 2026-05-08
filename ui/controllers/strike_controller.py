"""
ui.controllers.strike_controller — Swarm Strike 相關所有 handlers
=================================================================

本模組將原 main_window.py 中 1240 行蜂群打擊邏輯抽出為 Mixin。
MainWindow 透過多重繼承 `class MainWindow(QMainWindow, StrikeControllerMixin)`
取得所有 `_on_strike_*` 方法，保持 100% 向下相容。

Mixin 不持有自身狀態，所有 self.* 屬性皆由 MainWindow.__init__ 設定：
    _strike_marking_mode, _strike_base_marking_mode, _strike_launch_base
    _strike_targets, _strike_result, _vtol_strike_planner, _vtol_strike_plans
    _recon_strike_manager, _recon_strike_report
    _sitl_links, _dccpp_result, parameter_panel, map_widget

涵蓋 signal slots：
    strike_mark_targets_requested       → _on_strike_mark_targets
    strike_mark_base_requested           → _on_strike_mark_base
    strike_mode_changed                   → _on_strike_mode_changed
    strike_execute_requested              → _on_strike_execute (+ _vtol 分流)
    strike_clear_requested                → _on_strike_clear
    strike_export_requested               → _on_strike_export
    strike_dtot_export_requested          → _on_strike_dtot_export
    strike_owa_parm_requested             → _on_strike_owa_parm
    strike_sitl_upload_requested          → _on_strike_sitl_upload
    strike_recon_trigger_requested        → _on_strike_recon_trigger
    strike_vtol_toggle_changed            → _on_strike_vtol_toggled
"""
from __future__ import annotations

import math
from PyQt6.QtWidgets import QMessageBox, QFileDialog

from utils.logger import get_logger

logger = get_logger()


class StrikeControllerMixin:
    """蜂群打擊 Controller — 供 MainWindow 繼承"""

    # ─────────────────────────────────────────────────────────────────
    # 蜂群打擊模組 (Swarm Strike) 處理
    # ─────────────────────────────────────────────────────────────────
    def _on_strike_mark_targets(self):
        """切換打擊目標標記模式"""
        self._strike_marking_mode = not self._strike_marking_mode

        # 取得 3D 地圖 widget（通過 DualMapWidget）
        cesium = self._get_cesium_widget()
        if cesium:
            cesium.strike_set_marking_mode(self._strike_marking_mode)

        self.parameter_panel.set_strike_marking_mode(self._strike_marking_mode)

        if self._strike_marking_mode:
            # 啟動標記模式：連接地圖點擊信號
            self.map_widget.strike_target_added.connect(self._on_strike_target_clicked)
            self.statusBar().showMessage(
                '🎯 打擊目標標記模式：左鍵點擊 3D 地圖新增地面目標', 0
            )
        else:
            # 關閉標記模式
            try:
                self.map_widget.strike_target_added.disconnect(self._on_strike_target_clicked)
            except TypeError:
                pass
            self.statusBar().showMessage(
                f'已結束目標標記，共 {len(self._strike_targets)} 個目標', 3000
            )

    def _on_strike_target_clicked(self, lat: float, lon: float):
        """地圖點擊路由 → 依模式決定是「新增目標」還是「設定 STOT 基地」

        兩個操作都透過 QUndoStack 紀錄，使用者可 Ctrl+Z 撤銷。
        """
        # ── STOT 基地標記模式：單點設定、設完自動退出 ─────────────
        if self._strike_base_marking_mode:
            try:
                from ui.undo_commands import SetStrikeLaunchBaseCommand
                cmd = SetStrikeLaunchBaseCommand(self, (lat, lon))
                if hasattr(self, 'undo_stack'):
                    self.undo_stack.push(cmd)
                else:
                    cmd.redo()
            except Exception:
                # fallback: 直接設定
                self._strike_launch_base = (lat, lon)
                self.parameter_panel.update_strike_base_label(lat, lon)

            self._toggle_strike_base_marking(False)
            self.statusBar().showMessage(
                f'📍 STOT 發射基地已設定: ({lat:.6f}, {lon:.6f}) (Ctrl+Z 可撤銷)', 4000
            )
            logger.info(f'[Strike/STOT] 設定共用發射基地: ({lat:.6f}, {lon:.6f})')
            return

        # ── 預設：新增打擊目標 (透過 Undo Stack 紀錄) ─────────────
        try:
            from ui.undo_commands import AddStrikeTargetCommand
            cmd = AddStrikeTargetCommand(self, lat, lon)
            if hasattr(self, 'undo_stack'):
                self.undo_stack.push(cmd)
            else:
                cmd.redo()
        except Exception:
            # fallback: 直接操作（若 Undo 系統未初始化）
            idx = len(self._strike_targets) + 1
            self._strike_targets.append((lat, lon))
            cesium = self._get_cesium_widget()
            if cesium:
                cesium.strike_add_target(lat, lon, idx)
            self.parameter_panel.update_strike_target_count(len(self._strike_targets))

        idx = len(self._strike_targets)
        self.statusBar().showMessage(
            f'🎯 TGT-{idx} 已標記: ({lat:.6f}, {lon:.6f}) (Ctrl+Z 可撤銷)', 3000
        )
        logger.info(f'[Strike] 新增打擊目標 TGT-{idx}: ({lat:.6f}, {lon:.6f})')

    # ─────────────────────────────────────────────────────────────────
    #  共用發射基地 (SAME 模式才需要) — 標記切換 / 模式切換
    # ─────────────────────────────────────────────────────────────────
    def _on_strike_mode_changed(self, mode: str):
        """發射位置切換 (SAME 同地 / DIFF 異地)

        2026 重構：接收 SAME/DIFF，也相容舊值 STOT/DTOT。
        """
        from core.strike.time_coordination import normalize_launch_mode
        try:
            norm_mode = normalize_launch_mode(mode)
        except ValueError:
            norm_mode = mode

        # 切換到異地 (DIFF) 時，清掉可能進行中的基地標記狀態
        if norm_mode == 'DIFF' and self._strike_base_marking_mode:
            self._toggle_strike_base_marking(False)
        desc = ('同地 (SAME)' if norm_mode == 'SAME' else '異地 (DIFF)')
        self.statusBar().showMessage(f'發射位置切換為 {desc}', 2500)
        logger.info(f'[Strike] 發射位置 → {norm_mode} ({desc})')

    def _on_strike_mark_base(self):
        """切換共用發射基地標記模式 (僅同地 SAME 模式需要；與目標標記互斥)"""
        # 若目標標記模式正在運作，先關閉它
        if self._strike_marking_mode:
            self._on_strike_mark_targets()
        self._toggle_strike_base_marking(not self._strike_base_marking_mode)

    def _toggle_strike_base_marking(self, enabled: bool):
        """實際切換基地標記模式的狀態與地圖訊號連接"""
        self._strike_base_marking_mode = enabled
        self.parameter_panel.set_strike_base_marking_mode(enabled)

        # 基地標記沿用 cesium 的 strike_target_added 信號（click 由 _on_strike_target_clicked 路由）
        cesium = self._get_cesium_widget()
        if cesium:
            cesium.strike_set_marking_mode(enabled)

        if enabled:
            try:
                self.map_widget.strike_target_added.connect(self._on_strike_target_clicked)
            except Exception:
                pass
            self.statusBar().showMessage(
                '📍 STOT 基地標記模式：左鍵點擊 3D 地圖設定共用起飛點', 0
            )
        else:
            try:
                self.map_widget.strike_target_added.disconnect(self._on_strike_target_clicked)
            except TypeError:
                pass

    def _on_strike_execute(self, params: dict):
        """執行蜂群打擊 — 依發射位置 + 載具類型分流

        分流邏輯：
          - VTOL 模式啟用 → 呼叫 _on_strike_execute_vtol() 使用 VTOLSwarmStrikePlanner
          - 固定翼模式：按發射位置走 plan_auto (異地) / plan_stot (同地)

        兩種路徑都會接著執行 STOT/DTOT 時空協同。
        """
        # ── Schema 驗證：在進入規劃邏輯前先確認參數合法 ────────
        # (2026 重構：以 config.schemas 做 fail-fast 驗證，取代散落的 if 檢查)
        try:
            from config.schemas import StrikeParameters
            schema_params = {
                'launch_mode':   params.get('mode', 'DTOT'),
                'timing_mode':   params.get('timing_mode', 'STOT'),
                'interval_sec':  params.get('interval_sec', 0.0),
                'cruise_alt':    params.get('cruise_alt', 500.0),
                'cruise_speed':  params.get('cruise_speed', 60.0),
                'altitude_step': params.get('altitude_step', 30.0),
                'max_dive_angle': params.get('max_dive_angle', 45.0),
                'dive_initiation_dist': params.get('dive_initiation_dist', 800.0),
                'max_speed':     params.get('max_speed', 85.0),
                'stall_speed':   params.get('stall_speed', 25.0),
                'min_turn_radius': params.get('min_turn_radius', 150.0),
            }
            sp = StrikeParameters.from_dict(schema_params)
            sp.validate()
        except ValueError as e:
            QMessageBox.warning(
                self, '蜂群打擊 — 參數錯誤',
                f'參數驗證失敗，請檢查設定:\n\n{e}'
            )
            logger.warning(f'[Strike] 參數驗證失敗: {e}')
            return
        except Exception as e:
            logger.warning(f'[Strike] Schema 檢查異常 (忽略): {e}')

        # ── VTOL 模式分流 ──────────────────────────────────────
        vtol_params = self.parameter_panel.get_strike_vtol_params()
        if vtol_params.get('enabled', False):
            self._on_strike_execute_vtol(params, vtol_params)
            return

        import json
        from core.strike.terminal_strike_planner import (
            TerminalStrikePlanner, StrikeTarget,
        )

        if not self._strike_targets:
            QMessageBox.warning(self, '蜂群打擊', '請先標記至少一個打擊目標')
            return

        # ── 讀取「發射位置」模式 (2026 重構：SAME/DIFF 取代 STOT/DTOT) ─
        # normalize_launch_mode 同時處理新值 (SAME/DIFF) 與 legacy (STOT/DTOT/中文)
        from core.strike.time_coordination import normalize_launch_mode
        launch_mode = normalize_launch_mode(
            params.get('launch_mode') or params.get('mode', 'DIFF')
        )

        # ── 同地發射必須先設定共用發射基地 ──────────────────────
        if launch_mode == 'SAME' and self._strike_launch_base is None:
            QMessageBox.warning(
                self, '同地發射 (SAME) 所需基地未設定',
                '請先使用「📍 從地圖標記共用發射基地」按鈕設定起飛點。'
            )
            return

        # 關閉任何進行中的標記模式
        if self._strike_marking_mode:
            self._on_strike_mark_targets()
        if self._strike_base_marking_mode:
            self._toggle_strike_base_marking(False)

        cruise_alt = params.get('cruise_alt', 500.0)
        cruise_speed = params.get('cruise_speed', 60.0)
        max_dive = params.get('max_dive_angle', 45.0)
        dive_dist = params.get('dive_initiation_dist', 800.0)
        alt_step = params.get('altitude_step', 30.0)
        anim_speed = params.get('anim_speed', 3.0)
        turn_radius = params.get('min_turn_radius',
                                 max(cruise_speed ** 2 / (9.81 * math.tan(math.radians(30.0))), 80.0))

        # 建立目標列表
        targets = []
        for i, (lat, lon) in enumerate(self._strike_targets):
            targets.append(StrikeTarget(target_id=i + 1, lat=lat, lon=lon))

        # 建立規劃器（含起飛爬升 + Dubins 巡航 + 末端俯衝 三段）
        planner = TerminalStrikePlanner(
            max_dive_angle_deg=max_dive,
            dive_initiation_dist_m=dive_dist,
            cruise_alt_m=cruise_alt,
            cruise_speed_mps=cruise_speed,
            altitude_step_m=alt_step,
            takeoff_alt_m=0.0,
            climb_angle_deg=8.0,
            min_turn_radius_m=turn_radius,
            use_dubins_cruise=True,
        )

        # ── 依發射位置分流規劃 ──────────────────────────────────
        if launch_mode == 'SAME':
            base_lat, base_lon = self._strike_launch_base
            trajectories = planner.plan_stot(
                targets,
                launch_lat=base_lat,
                launch_lon=base_lon,
                launch_alt=0.0,
            )
            logger.info(
                f'[Strike/SAME] 同地發射：base=({base_lat:.6f}, {base_lon:.6f}), '
                f'{len(targets)} 目標 → {len(trajectories)} 架 UCAV'
            )
        else:
            # DIFF (異地)：各 UCAV 依目標分散起飛
            trajectories = planner.plan_auto(targets, spawn_dist_m=1500.0)
            logger.info(
                f'[Strike/DIFF] 異地發射：{len(targets)} 目標 → {len(trajectories)} 架 UCAV'
            )

        # 保留舊變數名以利後面程式碼最小修改 (mode == launch_mode)
        mode = launch_mode

        if not trajectories:
            QMessageBox.warning(self, '蜂群打擊', '軌跡規劃失敗：無有效分配')
            return

        # 轉換為 Cesium JSON 並送往前端
        cesium_data = planner.trajectories_to_cesium_data(trajectories, targets)
        data_json = json.dumps(cesium_data)

        cesium = self._get_cesium_widget()
        if cesium:
            # 直接渲染靜態打擊路徑（起飛=黃 / 巡航=綠 / 俯衝=紅），
            # 不再播放飛行動畫，讓操作員可檢視整條預定航線
            cesium.strike_render_path(data_json)

        # 快取結果 → 供「匯出打擊任務」按鈕使用 (含模式與基地資訊)
        # 使用 TypedDict 確保 schema 一致性 (core.strike.strike_result)
        from core.strike.strike_result import make_empty_strike_result
        self._strike_result = make_empty_strike_result(
            mode=mode,                                         # 'SAME' 同地 / 'DIFF' 異地
            launch_base=self._strike_launch_base,              # (lat, lon) 或 None
            trajectories=trajectories,
            targets=targets,
            params=dict(params),
        )
        self.parameter_panel.set_strike_export_enabled(True)

        # ── 時空協同即時預算：依 timing_mode 計算各機 TOT 速度預覽 ──────
        # 架構說明：
        #   mode         = 發射位置 ('SAME' 同地 / 'DIFF' 異地) → 已於空間規劃階段處理
        #   timing_mode  = 時間協同 ('STOT' 同時 / 'DTOT' 間隔) → 此處決定 TOT 排程
        timing_mode = params.get('timing_mode', 'STOT')
        interval_sec = float(params.get('interval_sec', 0.0))
        try:
            from core.strike.dtot_coordinator import DTOTCoordinator
            stall_v = params.get('stall_speed',
                                 self.parameter_panel._strike_stall_speed.value())
            max_v = params.get('max_speed',
                               self.parameter_panel._strike_max_speed.value())
            turn_r = params.get('min_turn_radius',
                                self.parameter_panel._strike_turn_radius.value())
            coord = DTOTCoordinator(
                cruise_speed_mps=cruise_speed,
                max_speed_mps=max_v,
                stall_speed_mps=stall_v,
                min_turn_radius_m=turn_r,
            )
            plans = coord.coordinate(
                trajectories,
                mode=timing_mode,
                interval_sec=interval_sec if timing_mode == 'DTOT' else 0.0,
            )
            if plans:
                # STOT: 所有機同秒命中；DTOT: slot k 於 T+k·Δ 命中
                if timing_mode == 'STOT':
                    tot = plans[0].tot_sec
                    pv = [f'STOT — TOT = {tot:.1f}s (同秒命中)']
                else:
                    tots = sorted(p.tot_sec for p in plans)
                    pv = [f'DTOT Δ={interval_sec:.1f}s → 命中時刻 '
                          + ', '.join(f'{t:.1f}' for t in tots) + 's']
                for p in plans:
                    hold = f' +Loiter {p.holding_time_sec:.0f}s' if p.holding_time_sec > 0.5 else ''
                    flag = '' if p.feasible else ' [NG]'
                    pv.append(
                        f'{p.uav_name}: V={p.required_speed_mps:.1f}m/s '
                        f'@t={p.tot_sec:.0f}s{hold}{flag}'
                    )
                self.parameter_panel.update_dtot_preview('\n'.join(pv))
                # 快取計畫結果，供匯出/SITL 上傳時免重算
                self._strike_result['coord_plans'] = plans
                self._strike_result['timing_mode'] = timing_mode
                self._strike_result['interval_sec'] = interval_sec
        except Exception as e:
            logger.warning(f'[Strike] 時空協同預算失敗: {e}')

        # 顯示結果摘要
        summary_lines = [
            f'蜂群打擊規劃完成 ({mode})：{len(trajectories)} 架 UCAV → '
            f'{len(targets)} 個目標（高度錯層 {alt_step:.0f}m）'
        ]
        for tr in trajectories:
            summary_lines.append(
                f'  {tr.uav_name} → {tr.target_name}: '
                f'{tr.total_distance_m:.0f}m, alt={tr.cruise_alt_m:.0f}m, '
                f'dive@WP{tr.dive_start_index}'
            )
        self.statusBar().showMessage(summary_lines[0], 5000)
        logger.info('\n'.join(summary_lines))

    # ═════════════════════════════════════════════════════════════════
    #  VTOL 蜂群打擊 — 使用 VTOLSwarmStrikePlanner
    # ═════════════════════════════════════════════════════════════════
    def _on_strike_execute_vtol(self, params: dict, vtol_params: dict):
        """VTOL 模式專用打擊執行路徑。

        使用 `VTOLSwarmStrikePlanner`：
          - NAV_VTOL_TAKEOFF 垂直起飛
          - Phase 2 (cruise) / Phase 3 (2km 邊界後衝刺) 雙段空速
          - 可選 IMAGE_START_CAPTURE AI 尋標
          - CEP ring 分佈命中點
          - 以「抵達 2km 邊界時刻」為 TOT 基準

        此路徑為獨立規劃（不共用 TerminalStrikePlanner），以避免混淆固定翼 vs VTOL
        的任務序列格式。
        """
        from core.strike.vtol_swarm_strike_planner import (
            VTOLSwarmStrikePlanner, AttackMode as VTOLAttackMode,
            Target as VTOLTarget, VTOLUAV,
        )

        if not self._strike_targets:
            QMessageBox.warning(self, 'VTOL 蜂群打擊', '請先標記至少一個打擊目標')
            return

        # 2026 重構：launch_mode = SAME/DIFF; legacy STOT/DTOT 自動轉換
        from core.strike.time_coordination import normalize_launch_mode
        launch_mode = normalize_launch_mode(
            params.get('launch_mode') or params.get('mode', 'DIFF')
        )
        timing_mode = params.get('timing_mode', 'STOT')
        interval_sec = float(params.get('interval_sec', 0.0))

        # 同地發射需要基地點
        if launch_mode == 'SAME' and self._strike_launch_base is None:
            QMessageBox.warning(
                self, 'VTOL 同地發射 (SAME) 所需基地未設定',
                '請先使用「📍 從地圖標記共用發射基地」按鈕設定起飛點。'
            )
            return

        # 關閉標記模式
        if self._strike_marking_mode:
            self._on_strike_mark_targets()
        if self._strike_base_marking_mode:
            self._toggle_strike_base_marking(False)

        # ── 建立 VTOL Target (CEP 從 VTOL 參數讀) ───────────────
        tgt_lat, tgt_lon = self._strike_targets[0]
        vtol_target = VTOLTarget(
            lat=tgt_lat, lon=tgt_lon, alt=0.0,
            cep_m=float(vtol_params.get('cep_m', 12.0)),
            name='TGT-1',
        )

        # ── 建立 VTOL UAV 列表 ──────────────────────────────────
        vtol_uavs: list = []
        if launch_mode == 'SAME':
            # 同地發射：全部 UCAV 共用一個基地
            base_lat, base_lon = self._strike_launch_base
            n = max(1, len(self._strike_targets))
            for i in range(n):
                vtol_uavs.append(VTOLUAV(
                    uav_id=i + 1, lat=base_lat, lon=base_lon, alt=0.0,
                    name=f'VTOL-{i + 1}',
                ))
        else:
            # 異地發射：使用「目標周邊分散點」近似（真實場景應用實際起飛點）
            import math as _math
            n = len(self._strike_targets)
            for i in range(n):
                # 目標外圍 1.5 km 方圓產生起飛點
                bearing = (360.0 / n * i) % 360.0
                from core.strike.swarm_strike_planner import _destination as _dst
                lat, lon = _dst(tgt_lat, tgt_lon,
                                (bearing + 180.0) % 360.0, 1500.0)
                vtol_uavs.append(VTOLUAV(
                    uav_id=i + 1, lat=lat, lon=lon, alt=0.0,
                    name=f'VTOL-{i + 1}',
                ))

        # ── VTOL Planner ─────────────────────────────────────────
        attack_mode = (VTOLAttackMode.STOT if timing_mode == 'STOT'
                       else VTOLAttackMode.DTOT)
        try:
            planner = VTOLSwarmStrikePlanner(
                target=vtol_target,
                uavs=vtol_uavs,
                mode=attack_mode,
                interval_sec=interval_sec,
                cruise_speed_kts=float(vtol_params.get('cruise_kts', 50.0)),
                terminal_speed_kts=float(vtol_params.get('terminal_kts', 90.0)),
                stall_speed_mps=float(params.get('stall_speed', 18.0)),
                turn_radius_m=float(params.get('min_turn_radius', 150.0)),
                base_cruise_alt_m=float(params.get('cruise_alt', 400.0)),
                altitude_step_m=float(params.get('altitude_step', 30.0)),
                terminal_boundary_m=float(vtol_params.get('boundary_m', 2000.0)),
                vtol_transition_alt_m=80.0,
                vtol_climb_rate_mps=5.0,
                enable_ai_seeker=bool(vtol_params.get('ai_seeker', True)),
            )
            vtol_plans = planner.plan()
        except Exception as e:
            logger.error(f'[VTOL Strike] 規劃失敗: {e}', exc_info=True)
            QMessageBox.critical(self, 'VTOL 蜂群打擊', f'規劃失敗:\n{e}')
            return

        # ── 快取 VTOL 結果 (不同於固定翼 self._strike_result) ────
        self._vtol_strike_planner = planner
        self._vtol_strike_plans = vtol_plans
        # 同時設定基本的 _strike_result 以讓匯出/SITL 按鈕可運作
        from core.strike.strike_result import make_empty_strike_result
        self._strike_result = make_empty_strike_result(
            mode=launch_mode,                       # 'SAME' 同地 / 'DIFF' 異地
            timing_mode=timing_mode,
            interval_sec=interval_sec,
            is_vtol=True,
            vtol_plans=vtol_plans,
            vtol_planner=planner,
            target=vtol_target,
            params=dict(params),
        )
        self.parameter_panel.set_strike_export_enabled(True)

        # ── 更新 UI 預覽 ────────────────────────────────────────
        if timing_mode == 'STOT':
            pv = [f'VTOL STOT — 同秒突破 2km 邊界 @ t={planner.base_t_boundary_sec:.1f}s']
        else:
            pv = [f'VTOL DTOT Δ={interval_sec:.1f}s → 邊界突破依序']

        for p in vtol_plans:
            loi = f' +Loiter {p.loiter_time_sec:.0f}s' if p.loiter_time_sec > 0.5 else ''
            pv.append(
                f'{p.uav.name}: V_c={p.cruise_speed_mps:.1f}→V_t={p.terminal_speed_mps:.1f} m/s, '
                f'alt={p.cruise_alt_m:.0f}m, t_bnd={p.t_boundary_sec:.0f}s, '
                f't_hit={p.t_impact_sec:.0f}s{loi}'
            )
        self.parameter_panel.update_dtot_preview('\n'.join(pv))

        # ── 結果摘要 ────────────────────────────────────────────
        summary = (f'VTOL 蜂群打擊規劃完成：{len(vtol_plans)} 架 UCAV → '
                   f'{mode} 發射 / {timing_mode} 時間協同')
        self.statusBar().showMessage(summary, 5000)
        logger.info(summary)
        for p in vtol_plans:
            logger.info(
                f'  {p.uav.name}: psi={p.attack_heading_deg:.1f}°, '
                f'Vc={p.cruise_speed_mps:.1f}m/s, Vt={p.terminal_speed_mps:.1f}m/s, '
                f'alt={p.cruise_alt_m:.0f}m, t_bnd={p.t_boundary_sec:.1f}s'
            )

    def _on_strike_clear(self):
        """清除打擊視覺化 (包含 STOT 基地、VTOL、Recon 快取)"""
        self._strike_targets.clear()
        self._strike_marking_mode = False
        self._strike_launch_base = None
        if self._strike_base_marking_mode:
            self._toggle_strike_base_marking(False)
        self._strike_result = None
        self._vtol_strike_planner = None
        self._vtol_strike_plans = None
        self._recon_strike_manager = None
        self._recon_strike_report = None

        cesium = self._get_cesium_widget()
        if cesium:
            cesium.strike_clear_all()

        self.parameter_panel.update_strike_target_count(0)
        self.parameter_panel.set_strike_marking_mode(False)
        self.parameter_panel.set_strike_export_enabled(False)
        self.parameter_panel.update_dtot_preview('')
        self.parameter_panel.update_strike_base_label(None, None)
        self.statusBar().showMessage('已清除打擊視覺化', 3000)

    # ─────────────────────────────────────────────────────────────────
    #  蜂群打擊 — 匯出 QGC WPL 航點（比照 DCCPP 匯出架構）
    # ─────────────────────────────────────────────────────────────────
    def _on_strike_export(self):
        """匯出當前蜂群打擊路徑為 QGC WPL 110 航點檔案。

        固定翼模式：
            DO_SET_HOME (179) → DO_CHANGE_SPEED (178) → NAV_TAKEOFF (22)
            → NAV_WAYPOINT (16, 巡航) × N → NAV_WAYPOINT (16, 俯衝) × M

        VTOL 模式（VTOLSwarmStrikePlanner 自帶 export_qgc_wpl）：
            DO_SET_HOME → NAV_VTOL_TAKEOFF (84) → DO_VTOL_TRANSITION (3000)
            → DO_CHANGE_SPEED (Phase 2) → NAV_WAYPOINT (2km 邊界)
            → DO_CHANGE_SPEED (Phase 3) → IMAGE_START_CAPTURE → NAV_WAYPOINT (IMPACT)
        """
        import os, math
        from PyQt6.QtWidgets import QFileDialog, QMessageBox
        from utils.file_io import create_waypoint_line, write_waypoints

        if not self._strike_result:
            QMessageBox.warning(
                self, "無打擊路徑",
                "請先點擊 EXECUTE SWARM STRIKE 完成規劃，再匯出任務。"
            )
            return

        # ── VTOL 模式：委派給 VTOLSwarmStrikePlanner 內建匯出 ────
        if self._strike_result.get('is_vtol'):
            export_dir = QFileDialog.getExistingDirectory(
                self, "選擇 VTOL 蜂群打擊匯出目錄", ""
            )
            if not export_dir:
                return
            try:
                planner = self._strike_result['vtol_planner']
                files = planner.export_qgc_wpl(export_dir)
                QMessageBox.information(
                    self, "VTOL 匯出完成",
                    f"已匯出 {len(files)} 架 VTOL UCAV 到:\n{export_dir}\n\n"
                    + "\n".join(f'  • {os.path.basename(f)}' for f in files)
                )
                self.statusBar().showMessage(
                    f'VTOL 蜂群打擊匯出完成 ({len(files)} 檔)', 6000,
                )
            except Exception as e:
                logger.error(f'[VTOL Strike] 匯出失敗: {e}', exc_info=True)
                QMessageBox.critical(self, 'VTOL 匯出失敗', str(e))
            return

        # ── 固定翼流程 ──────────────────────────────────────────
        if not self._strike_result.get('trajectories'):
            QMessageBox.warning(
                self, "無打擊路徑",
                "固定翼模式需要 trajectories；請重新 EXECUTE。"
            )
            return

        export_dir = QFileDialog.getExistingDirectory(
            self, "選擇蜂群打擊任務匯出目錄", ""
        )
        if not export_dir:
            return

        trajectories = self._strike_result['trajectories']
        targets = self._strike_result['targets']
        params = self._strike_result['params']

        cruise_speed = params.get('cruise_speed', 60.0)
        max_dive = params.get('max_dive_angle', 45.0)
        dive_dist = params.get('dive_initiation_dist', 800.0)
        turn_radius = max(
            cruise_speed ** 2 / (9.81 * math.tan(math.radians(30.0))), 80.0
        )
        # 俯衝段 acceptance_radius 較小避免跳點；巡航段用 turn_radius
        dive_accept_r = max(10.0, turn_radius * 0.25)

        exported_files = []
        for tr in trajectories:
            if not tr.waypoints:
                continue

            waypoint_lines = ['QGC WPL 110']
            seq = 0

            # ── seq 0: DO_SET_HOME ──────────────────────────────────
            waypoint_lines.append(create_waypoint_line(
                seq=seq, command=179,
                lat=tr.takeoff_lat, lon=tr.takeoff_lon, alt=0.0,
                current=1, autocontinue=1,
            ))
            seq += 1

            # ── seq 1: DO_CHANGE_SPEED (巡航空速) ───────────────────
            waypoint_lines.append(create_waypoint_line(
                seq=seq, command=178,
                param1=1.0, param2=cruise_speed, param3=0.0,
                current=0, autocontinue=1,
            ))
            seq += 1

            # ── seq 2: NAV_TAKEOFF — 上升至巡航高度 ─────────────────
            # 使用「巡航段起點」的經緯度作為 NAV_TAKEOFF 目標點
            cs_idx = max(tr.cruise_start_index, 0)
            if cs_idx < len(tr.waypoints):
                t0 = tr.waypoints[cs_idx]
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=22,                # NAV_TAKEOFF
                    param1=math.radians(8.0),           # 最小爬升俯仰 (rad)
                    param4=0.0,                         # 使用當前航向
                    lat=t0.lat, lon=t0.lon, alt=tr.cruise_alt_m,
                    current=0, autocontinue=1,
                ))
                seq += 1

            # ── 巡航段 NAV_WAYPOINT (seg='cruise')──────────────────
            for wp in tr.waypoints[cs_idx + 1:tr.dive_start_index + 1]:
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=16,                # NAV_WAYPOINT
                    lat=wp.lat, lon=wp.lon, alt=wp.alt,
                    param2=turn_radius,                 # 提前轉彎
                    current=0, autocontinue=1,
                ))
                seq += 1

            # ── 俯衝段 NAV_WAYPOINT (seg='dive')─────────────────────
            # acceptance_radius 縮小，確保緊貼目標；命中點 current=0 保持續飛
            for wp in tr.waypoints[tr.dive_start_index + 1:]:
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=16,
                    lat=wp.lat, lon=wp.lon, alt=wp.alt,
                    param2=dive_accept_r,
                    current=0, autocontinue=1,
                ))
                seq += 1

            # ── 寫檔 ────────────────────────────────────────────────
            filename = (
                f"STRIKE_{tr.uav_name}_→{tr.target_name}_"
                f"alt{tr.cruise_alt_m:.0f}m_θ{tr.actual_dive_angle_deg:.0f}°.waypoints"
            )
            # 清洗非法檔名字元
            filename = filename.replace('→', '-').replace('°', 'deg')
            filepath = os.path.join(export_dir, filename)
            if write_waypoints(filepath, waypoint_lines):
                exported_files.append(filename)
                logger.info(
                    f"匯出蜂群打擊 {tr.uav_name}→{tr.target_name}: "
                    f"{filepath} ({seq} 航點)"
                )

        # ── 匯出任務簡報 ───────────────────────────────────────────
        briefing_path = os.path.join(export_dir, "SWARM_STRIKE_briefing.txt")
        try:
            with open(briefing_path, 'w', encoding='utf-8') as f:
                f.write("蜂群協同打擊任務簡報 (Swarm Strike Mission)\n")
                f.write("=" * 48 + "\n\n")
                f.write(f"UCAV 數量：{len(trajectories)}\n")
                f.write(f"目標數量：{len(targets)}\n")
                f.write(f"巡航空速：{cruise_speed:.1f} m/s\n")
                f.write(f"最大俯衝角 θ_max：{max_dive:.1f}°\n")
                f.write(f"俯衝起始距離：{dive_dist:.0f} m\n")
                f.write(f"最小轉彎半徑 R_min：{turn_radius:.0f} m\n\n")
                f.write("─" * 48 + "\n")
                f.write("打擊分配與軌跡參數：\n")
                f.write("─" * 48 + "\n")
                for tr in trajectories:
                    f.write(
                        f"  {tr.uav_name:10s} → {tr.target_name:10s}  "
                        f"alt={tr.cruise_alt_m:4.0f}m  "
                        f"θ={tr.actual_dive_angle_deg:5.1f}°  "
                        f"dist={tr.total_distance_m:6.0f}m  "
                        f"WPs={len(tr.waypoints)}  "
                        f"dive@WP{tr.dive_start_index}\n"
                    )
                f.write("\n目標座標：\n")
                for t in targets:
                    f.write(f"  {t.name}: ({t.lat:.6f}, {t.lon:.6f}, {t.alt:.1f}m)\n")
        except Exception as e:
            logger.warning(f"寫入簡報失敗: {e}")

        QMessageBox.information(
            self, "匯出完成",
            f"已匯出 {len(exported_files)} 架 UCAV 任務到:\n{export_dir}\n\n"
            + "\n".join(exported_files)
        )
        self.statusBar().showMessage(
            f"已匯出 {len(exported_files)} 架蜂群打擊任務 → {export_dir}", 6000
        )

    # ─────────────────────────────────────────────────────────────────
    #  DTOT / STOT 飽和攻擊時空協同匯出
    # ─────────────────────────────────────────────────────────────────
    def _on_strike_dtot_export(self, params: dict):
        """執行 DTOT/STOT 時空協同 → 反推各機空速 → 匯出 QGC WPL。

        模式以「快取的 _strike_result['mode']」為準（即 EXECUTE 時選的發射模式），
        避免使用者在不同介面設定不一致。
        """
        import os
        from core.strike.dtot_coordinator import DTOTCoordinator

        if not self._strike_result:
            QMessageBox.warning(
                self, 'DTOT/STOT',
                '請先完成 EXECUTE SWARM STRIKE 再進行飽和攻擊協同匯出。',
            )
            return

        # VTOL 模式無需重新協同（VTOLSwarmStrikePlanner 已內建 TOT）
        # → 直接委派給一般匯出 (VTOL 路徑)
        if self._strike_result.get('is_vtol'):
            self._on_strike_export()
            return

        if not self._strike_result.get('trajectories'):
            QMessageBox.warning(
                self, 'DTOT/STOT',
                '固定翼模式需要 trajectories；請重新 EXECUTE。',
            )
            return

        export_dir = QFileDialog.getExistingDirectory(
            self, '選擇 DTOT/STOT 匯出目錄', '',
        )
        if not export_dir:
            return

        # 以 EXECUTE 時的 timing_mode 為準 (發射位置 mode 只影響空間規劃)
        # 優先順序：params > cached result > 預設 STOT
        timing_mode = (
            params.get('timing_mode')
            or self._strike_result.get('timing_mode')
            or params.get('mode', 'STOT')
        )
        interval_sec = float(
            params.get('interval_sec',
                       self._strike_result.get('interval_sec', 0.0))
        )
        cruise_speed = params.get('cruise_speed', 60.0)
        max_speed = params.get('max_speed', 85.0)
        stall_speed = params.get('stall_speed', 25.0)
        min_turn_r = params.get('min_turn_radius', 150.0)

        coord = DTOTCoordinator(
            cruise_speed_mps=cruise_speed,
            max_speed_mps=max_speed,
            stall_speed_mps=stall_speed,
            min_turn_radius_m=min_turn_r,
        )

        trajectories = self._strike_result['trajectories']
        plans = coord.coordinate(
            trajectories,
            mode=timing_mode,
            interval_sec=interval_sec if timing_mode == 'DTOT' else 0.0,
        )
        # 將最終 timing_mode 回寫給匯出檔名用
        mode = timing_mode

        if not plans:
            QMessageBox.warning(self, 'DTOT/STOT', '協同演算失敗：無有效結果')
            return

        files = coord.export_qgc_wpl(
            plans,
            export_dir=export_dir,
            cruise_accept_radius_m=min_turn_r,
            dive_accept_radius_m=max(10.0, min_turn_r * 0.25),
            mode=mode,
        )

        # 更新面板 DTOT 預覽 (依模式顯示 TOT 資訊)
        if mode == 'STOT':
            tot = plans[0].tot_sec if plans else 0.0
            preview_lines = [f'STOT — TOT = {tot:.1f}s (同秒命中)']
            tot_info = f'基準 TOT = {tot:.1f}s (全體同秒命中)'
        else:
            tots = sorted(p.tot_sec for p in plans)
            preview_lines = [
                f'DTOT Δ={interval_sec:.1f}s → 命中時刻 '
                + ', '.join(f'{t:.1f}' for t in tots) + 's'
            ]
            tot_info = (f'DTOT 間隔 Δ={interval_sec:.1f}s\n'
                        f'命中時刻: '
                        + ', '.join(f'{t:.1f}s' for t in tots))

        for p in plans:
            hold = f' +Loiter {p.holding_time_sec:.0f}s' if p.holding_time_sec > 0.5 else ''
            flag = '' if p.feasible else ' [INFEASIBLE]'
            preview_lines.append(
                f'{p.uav_name}: V={p.required_speed_mps:.1f}m/s '
                f'@t={p.tot_sec:.0f}s{hold}{flag}'
            )
        self.parameter_panel.update_dtot_preview('\n'.join(preview_lines))

        # ── Task 3: 衝突檢測 (與 AdvancedSwarmStrikePlanner 風格一致) ──
        # 簡易版：檢查高度錯層 + 盤旋圈重疊
        conflicts = self._strike_collision_check(plans, trajectories, min_turn_r)
        if conflicts:
            self.parameter_panel.update_collision_report(
                safe=False, conflicts=conflicts,
            )
        else:
            self.parameter_panel.update_collision_report(safe=True, conflicts=[])

        QMessageBox.information(
            self, f'{mode} 匯出完成',
            f'已匯出 {len(files)} 架 UCAV ({mode}) 到:\n{export_dir}\n\n'
            f'{tot_info}\n\n'
            f'避障檢查: {"SAFE" if not conflicts else f"{len(conflicts)} 項警告"}',
        )
        self.statusBar().showMessage(
            f'{mode} 飽和攻擊匯出完成 ({len(files)} 檔, '
            f'{"SAFE" if not conflicts else f"{len(conflicts)} 項衝突"})', 6000,
        )

    def _strike_collision_check(self, plans, trajectories, turn_radius_m):
        """簡易版避障檢查（借鑑 AdvancedSwarmStrikePlanner.CollisionReport）

        檢查：
          1. 兩兩巡航高度差 (step < 25m 視為違規)
          2. 盤旋中心兩兩 2D 距離 (若 < 2R 且同高度層則衝突)
        """
        import math as _math
        conflicts = []
        min_alt_sep = 25.0

        # 取出每機的 (alt, loiter_lat, loiter_lon, holding_time)
        info_list = []
        for p in plans:
            tr = p.trajectory
            alt = tr.cruise_alt_m if tr else 0.0
            info_list.append((p.uav_name, alt,
                              p.holding_center_lat, p.holding_center_lon,
                              p.holding_time_sec))

        for i in range(len(info_list)):
            for j in range(i + 1, len(info_list)):
                ni, ai, lat_i, lon_i, ti = info_list[i]
                nj, aj, lat_j, lon_j, tj = info_list[j]
                alt_diff = abs(ai - aj)
                if alt_diff < min_alt_sep:
                    conflicts.append(
                        f'{ni} vs {nj}: 高度差僅 {alt_diff:.0f}m '
                        f'< {min_alt_sep:.0f}m'
                    )
                # 盤旋圈檢查：兩者都有 loiter 且同高度層
                if ti > 0.5 and tj > 0.5 and alt_diff < min_alt_sep:
                    from core.strike.swarm_strike_planner import _haversine
                    d = _haversine(lat_i, lon_i, lat_j, lon_j)
                    if d < 2.0 * turn_radius_m:
                        conflicts.append(
                            f'{ni} vs {nj}: 盤旋圈重疊 (d={d:.0f}m < 2R, '
                            f'同層 {ai:.0f}m)'
                        )
        return conflicts

    # ─────────────────────────────────────────────────────────────────
    #  OWA-UAV 戰術 .parm 參數檔生成
    # ─────────────────────────────────────────────────────────────────
    def _on_strike_owa_parm(self):
        """產生 OWA-UAV 戰術 .parm 檔至 sitl/default_params/。"""
        import os
        from mission.owa_uav_params import generate_owa_uav_parm, OWAParamConfig

        # 從面板取用當前巡航速度相關值
        cruise_speed = self.parameter_panel._strike_cruise_speed.value()
        stall_speed = self.parameter_panel._strike_stall_speed.value()
        max_speed = self.parameter_panel._strike_max_speed.value()

        cfg = OWAParamConfig(
            cruise_airspeed_mps=cruise_speed,
            stall_airspeed_mps=stall_speed,
            max_airspeed_mps=max_speed,
        )

        output_path = os.path.join('sitl', 'default_params', 'owa_uav_default.parm')
        try:
            abs_path = generate_owa_uav_parm(output_path, cfg)
        except Exception as e:
            QMessageBox.critical(self, 'OWA-UAV', f'.parm 生成失敗:\n{e}')
            return

        QMessageBox.information(
            self, 'OWA-UAV .parm 已生成',
            f'已寫出至:\n{abs_path}\n\n'
            'SITL 啟動指令:\n'
            f'ArduPlane.exe --model plane --defaults\n'
            f'  "sitl/default_params/plane.parm,{output_path}"\n\n'
            '核心覆寫:\n'
            f'  FS_LONG_ACTN=0, FS_SHORT_ACTN=0 (繼續 AUTO)\n'
            f'  ICE_ENABLE=1, TERRAIN_FOLLOW=1\n'
            f'  AIRSPEED_CRUISE={cruise_speed:.0f} m/s',
        )
        self.statusBar().showMessage(
            f'OWA-UAV .parm 已生成: {output_path}', 5000,
        )

    # ─────────────────────────────────────────────────────────────────
    #  蜂群打擊 → SITL 上傳（每架 UCAV 綁定一條 SITL link）
    # ─────────────────────────────────────────────────────────────────
    def _on_strike_sitl_upload(self, use_dtot_speed: bool = True):
        """把當前蜂群打擊任務分派到所有連線中的 SITL 實例。

        每架 UCAV 產生一份獨立的 MAVLink 任務序列：
            DO_CHANGE_SPEED (178) — 若 use_dtot_speed=True 則用各機專屬 V_i
            NAV_TAKEOFF      (22) — 爬升至巡航高度
            NAV_LOITER_TIME  (19) — (可選) S-turn 補時盤旋 (僅 DTOT 協同結果)
            NAV_WAYPOINT     (16) × 巡航段 (param2 = turn_radius)
            NAV_WAYPOINT     (16) × 俯衝段 (param2 = 0.25 × turn_radius)

        分派規則同 DCCPP：uav_id 排序後對應 SITL link 0/1/2...；
        若 SITL 實例少於 UCAV 數，超出者共用最後一份路徑。
        """
        # ── 前置檢查 ───────────────────────────────────────────────
        if not self._sitl_links:
            QMessageBox.information(
                self, '蜂群打擊 → SITL',
                '尚未連線任何 SITL 實例。\n'
                '請先切換到「🛰 SITL」分頁，啟動 N 台 ArduPlane。'
            )
            return

        if not self._strike_result:
            QMessageBox.warning(
                self, '蜂群打擊 → SITL',
                '請先完成 EXECUTE SWARM STRIKE 再上傳任務至 SITL。'
            )
            return

        # ── VTOL 模式：直接上傳 VTOLSwarmStrikePlanner 的 mission ──
        if self._strike_result.get('is_vtol'):
            return self._on_strike_sitl_upload_vtol()

        if not self._strike_result.get('trajectories'):
            QMessageBox.warning(
                self, '蜂群打擊 → SITL',
                '固定翼模式需要 trajectories；請重新 EXECUTE。'
            )
            return

        trajectories = self._strike_result['trajectories']
        params = self._strike_result.get('params', {})
        cached_plans = self._strike_result.get('coord_plans', [])  # 協同結果
        mode = self._strike_result.get('mode', 'DTOT')                  # 發射位置
        timing_mode = self._strike_result.get('timing_mode', 'STOT')    # 時間協同
        interval_sec = self._strike_result.get('interval_sec', 0.0)

        cruise_speed = float(params.get('cruise_speed', 60.0))
        turn_radius = float(params.get('min_turn_radius',
                                       max(cruise_speed ** 2 /
                                           (9.81 * math.tan(math.radians(30.0))), 80.0)))
        dive_accept_r = max(10.0, turn_radius * 0.25)

        # 建 uav_id → TimingPlan 對照表 (供查 V_i 與 Loiter)
        plan_by_uav = {p.uav_id: p for p in cached_plans} if cached_plans else {}

        # ── MAVLink 命令常數 ──────────────────────────────────────
        CMD_WP, CMD_TO, CMD_SPD, CMD_LOITER = 16, 22, 178, 19

        # ── 逐架 UCAV 建構任務序列 ─────────────────────────────────
        sorted_trajs = sorted(trajectories, key=lambda t: t.uav_id)
        per_link_wps: dict = {}

        for link_idx, tr in enumerate(sorted_trajs):
            wp_list: list = []

            # 決定這架 UCAV 的巡航空速
            if use_dtot_speed and tr.uav_id in plan_by_uav:
                v_i = plan_by_uav[tr.uav_id].required_speed_mps
            else:
                v_i = cruise_speed

            # 1) DO_CHANGE_SPEED(178): param1=0 空速, param2=V, param3=-1 不改油門
            wp_list.append((0.0, 0.0, 0.0, CMD_SPD, 0.0, float(v_i), -1.0, 0.0))

            # 2) NAV_TAKEOFF(22): 爬升到巡航高度 (以巡航段起點為目標)
            cs_idx = max(tr.cruise_start_index, 0)
            if cs_idx < len(tr.waypoints):
                t0 = tr.waypoints[cs_idx]
                wp_list.append((
                    float(t0.lat), float(t0.lon), float(tr.cruise_alt_m),
                    CMD_TO, 10.0, 0.0, 0.0, 0.0,   # param1=pitch 10°
                ))

            # 3) (可選) NAV_LOITER_TIME(19): DTOT 補時盤旋
            if (use_dtot_speed and tr.uav_id in plan_by_uav
                    and plan_by_uav[tr.uav_id].holding_time_sec > 0.5):
                p = plan_by_uav[tr.uav_id]
                wp_list.append((
                    float(p.holding_center_lat), float(p.holding_center_lon),
                    float(tr.cruise_alt_m),
                    CMD_LOITER,
                    float(p.holding_time_sec),     # param1 = 盤旋秒數
                    0.0,
                    float(p.holding_radius_m),     # param3 = 盤旋半徑
                    1.0,                           # param4 = 達高度方結束
                ))

            # 4) 巡航段 NAV_WAYPOINT(16)
            dive_idx = tr.dive_start_index if tr.dive_start_index > 0 else len(tr.waypoints) - 1
            for wp in tr.waypoints[cs_idx + 1:dive_idx + 1]:
                wp_list.append((
                    float(wp.lat), float(wp.lon), float(wp.alt),
                    CMD_WP, 0.0, float(turn_radius), 0.0, 0.0,
                ))

            # 5) 俯衝段 NAV_WAYPOINT(16)：acceptance 縮小以確保貼近目標
            for wp in tr.waypoints[dive_idx + 1:]:
                wp_list.append((
                    float(wp.lat), float(wp.lon), float(wp.alt),
                    CMD_WP, 0.0, float(dive_accept_r), 0.0, 0.0,
                ))

            per_link_wps[link_idx] = (tr.uav_id, tr.uav_name, wp_list, v_i)
            logger.info(
                f'[SITL/Strike] UAV-{tr.uav_id} ({tr.uav_name}) → link#{link_idx}: '
                f'{len(wp_list)} MAVLink 指令 (V={v_i:.1f} m/s)'
            )

        # ── 分派到所有連線中的 SITL ────────────────────────────────
        from mission.mission_validator import validate_mission
        n_links = len(self._sitl_links)
        n_ucav = len(per_link_wps)
        uploaded: list = []
        for link_idx, link in enumerate(self._sitl_links):
            key = link_idx if link_idx in per_link_wps else max(per_link_wps.keys())
            uav_id, uav_name, path_wps, v_i = per_link_wps[key]

            # 上傳前驗證 — 防止明顯無效的任務發到飛控
            val = validate_mission(path_wps)
            if not val.ok:
                logger.error(
                    f'[SITL/Strike] {uav_name} 任務驗證失敗，跳過上傳: '
                    + ', '.join(val.errors[:3])
                )
                continue
            if val.warnings:
                logger.warning(
                    f'[SITL/Strike] {uav_name} 任務有 {len(val.warnings)} 項警告 '
                    f'(第一項: {val.warnings[0]})'
                )
            try:
                link.upload_mission(path_wps)
                uploaded.append((link_idx, uav_name, v_i, len(path_wps)))
                logger.info(
                    f'[SITL/Strike] link#{link_idx} ← {uav_name} '
                    f'({len(path_wps)} 點, V={v_i:.1f} m/s)'
                )
            except Exception as e:
                logger.error(f'[SITL/Strike] link#{link_idx} 上傳失敗: {e}',
                             exc_info=True)

        # ── 使用者回饋 ─────────────────────────────────────────────
        if not uploaded:
            QMessageBox.critical(
                self, '蜂群打擊 → SITL',
                '所有 SITL 上傳皆失敗，請查看日誌。'
            )
            return

        speed_mode = ('協同空速 (各機 V_i)' if use_dtot_speed and plan_by_uav
                      else '統一 cruise_speed')
        timing_desc = (f'STOT 同秒命中' if timing_mode == 'STOT'
                       else f'DTOT 間隔命中 (Δ={interval_sec:.1f}s)')
        summary = '\n'.join(
            f'  link#{idx}: {name} — V={v:.1f} m/s ({n} 點)'
            for idx, name, v, n in uploaded
        )
        QMessageBox.information(
            self, '蜂群打擊 → SITL 上傳完成',
            f'發射位置: {mode}\n'
            f'時間協同: {timing_desc}\n'
            f'空速策略: {speed_mode}\n'
            f'SITL 實例: {n_links} 台 | UCAV: {n_ucav} 架\n\n'
            f'{summary}\n\n'
            '接下來：\n'
            '  1. 在 Mission Planner 確認任務已收到\n'
            '  2. 切換飛行模式至 AUTO\n'
            '  3. ARM 後自動執行蜂群打擊'
        )
        self.statusBar().showMessage(
            f'[{timing_desc}] 已上傳 {len(uploaded)} 架 UCAV 任務至 SITL',
            6000,
        )

    # ═════════════════════════════════════════════════════════════════
    #  VTOL 模式 SITL 上傳
    # ═════════════════════════════════════════════════════════════════
    def _on_strike_sitl_upload_vtol(self):
        """VTOL 模式專用 SITL 上傳：使用 VTOLSwarmStrikePlanner 的 mission 序列"""
        if not self._sitl_links:
            QMessageBox.information(self, 'VTOL SITL', '尚未連線任何 SITL 實例')
            return

        plans = self._strike_result.get('vtol_plans', [])
        if not plans:
            QMessageBox.warning(self, 'VTOL SITL', '無 VTOL 規劃結果')
            return

        uploaded = []
        for p in sorted(plans, key=lambda x: x.uav.uav_id):
            link_idx = p.uav.uav_id - 1
            if not (0 <= link_idx < len(self._sitl_links)):
                continue
            # MissionItem → 8-tuple
            wps_tuples = [
                (float(m.lat), float(m.lon), float(m.alt),
                 int(m.cmd),
                 float(m.param1), float(m.param2),
                 float(m.param3), float(m.param4))
                for m in p.mission
            ]
            try:
                self._sitl_links[link_idx].upload_mission(wps_tuples)
                uploaded.append((link_idx, p.uav.name, p.cruise_speed_mps, len(wps_tuples)))
                logger.info(
                    f'[VTOL/SITL] {p.uav.name} → link#{link_idx}: '
                    f'{len(wps_tuples)} 指令 (Vc={p.cruise_speed_mps:.1f}/'
                    f'Vt={p.terminal_speed_mps:.1f}m/s)'
                )
            except Exception as e:
                logger.error(f'[VTOL/SITL] link#{link_idx} 失敗: {e}')

        summary = '\n'.join(
            f'  link#{idx}: {name} — V_cruise={v:.1f} m/s ({n} 指令)'
            for idx, name, v, n in uploaded
        )
        QMessageBox.information(
            self, 'VTOL 蜂群打擊 → SITL 上傳完成',
            f'VTOL 模式任務 (NAV_VTOL_TAKEOFF + Phase 2/3)\n'
            f'SITL 實例: {len(self._sitl_links)} 台 | VTOL: {len(plans)} 架\n\n'
            f'{summary}\n\n'
            '接下來：\n'
            '  1. Mission Planner 確認收到 VTOL 任務\n'
            '  2. 切換 AUTO 模式\n'
            '  3. ARM 後執行垂直起飛 → 轉固定翼 → Phase 2 巡航 → Phase 3 衝刺'
        )
        self.statusBar().showMessage(
            f'[VTOL] 已上傳 {len(uploaded)} 架 VTOL 任務至 SITL', 6000,
        )

    # ═════════════════════════════════════════════════════════════════
    #  動態偵打切換 (ReconToStrikeManager) — DCCPP 掃描中觸發打擊
    # ═════════════════════════════════════════════════════════════════
    def _on_strike_recon_trigger(self, params: dict):
        """DCCPP → Strike 動態切換

        流程：
          1. 從 DCCPP 結果 + SITL 即時遙測建構 UAV 當前狀態
          2. 取第一個已標記的打擊目標作為偵測事件
          3. 呼叫 ReconToStrikeManager → 平滑銜接 + IAPF + STOT
          4. 若有 SITL 連線，上傳新任務到對應 link
          5. 在 Cesium 上標示新任務路徑
        """
        from core.strike.recon_to_strike_manager import (
            ReconToStrikeManager, UAVState, TaskMode,
        )

        # ── 前置檢查 ────────────────────────────────────────────
        if not self._strike_targets:
            QMessageBox.warning(
                self, '動態切換',
                '請先在地圖上標記至少 1 個打擊目標 (🎯 標記目標按鈕)'
            )
            return

        dccpp = getattr(self, '_dccpp_result', None)
        if not dccpp or not dccpp.get('assembled_paths'):
            QMessageBox.warning(
                self, '動態切換',
                '需先執行 DCCPP 規劃（DCCPP 分頁 → 生成協同覆蓋路徑）\n'
                '以取得 UAV 群的當前位置資訊。'
            )
            return

        # ── 建構 UAV 狀態 (優先用 SITL live 遙測，fallback 用 DCCPP 起飛點)
        uav_states: dict = {}
        assembled = dccpp['assembled_paths']

        for uid, bpath in assembled.items():
            wps = getattr(bpath, 'waypoints', None) or []
            if not wps:
                continue
            w0 = wps[0]

            # 優先：若有 SITL link 對應，讀即時位置
            live_pos = None
            link_idx = uid - 1 if isinstance(uid, int) else -1
            if 0 <= link_idx < len(self._sitl_links):
                lk = self._sitl_links[link_idx]
                frame = getattr(lk, 'last_frame', None)
                if frame and getattr(frame, 'lat', None) is not None:
                    live_pos = (frame.lat, frame.lon, frame.alt,
                                getattr(frame, 'heading_deg', 0.0),
                                getattr(frame, 'groundspeed_mps', 25.0))

            if live_pos:
                lat, lon, alt, hdg, gs = live_pos
            else:
                # fallback: 用 DCCPP 第一個航點作為當前位置
                lat = float(getattr(w0, 'lat', 0.0))
                lon = float(getattr(w0, 'lon', 0.0))
                alt = float(getattr(w0, 'alt', 300.0))
                hdg = float(getattr(w0, 'heading_compass_deg', 0.0) or 0.0)
                gs = 25.0

            uav_states[int(uid)] = UAVState(
                uav_id=int(uid),
                lat=lat, lon=lon, alt=alt,
                heading_deg=hdg,
                ground_speed_mps=gs,
                task_mode=TaskMode.COVERAGE,
                name=f'UAV-{int(uid)}',
            )

        if len(uav_states) < 1:
            QMessageBox.warning(self, '動態切換', 'DCCPP 結果中未找到有效的 UAV 位置')
            return

        # ── 建立 Manager 並觸發切換 ──────────────────────────────
        tgt_lat, tgt_lon = self._strike_targets[0]
        try:
            mgr = ReconToStrikeManager(
                uav_states=uav_states,
                coalition_size=int(params.get('coalition_size', 3)),
                base_strike_alt_m=float(params.get('base_alt', 500.0)),
                altitude_step_m=float(params.get('alt_step', 30.0)),
                turn_radius_m=float(params.get('turn_radius', 150.0)),
                cruise_speed_mps=float(params.get('cruise_speed', 25.0)),
                stall_speed_mps=float(params.get('stall_speed', 18.0)),
                iapf_min_safe_dist_m=float(params.get('iapf_safe_dist', 1500.0)),
                transition_lookahead_m=800.0,
            )
            report = mgr.target_detected_callback(tgt_lat, tgt_lon, 0.0)
        except Exception as e:
            logger.error(f'[ReconToStrike] 切換失敗: {e}', exc_info=True)
            QMessageBox.critical(self, '動態切換失敗', str(e))
            return

        # ── 上傳至 SITL (若有連線) ───────────────────────────────
        sitl_uploaded = 0
        if self._sitl_links:
            for a in mgr.assignments.values():
                link_idx = a.uav_id - 1
                if 0 <= link_idx < len(self._sitl_links):
                    try:
                        wps_tuples = mgr.mission_to_tuples(a)
                        self._sitl_links[link_idx].upload_mission(wps_tuples)
                        sitl_uploaded += 1
                        logger.info(
                            f'[ReconToStrike] UAV-{a.uav_id} → SITL link#{link_idx}: '
                            f'{len(wps_tuples)} 指令'
                        )
                    except Exception as e:
                        logger.warning(
                            f'[ReconToStrike] UAV-{a.uav_id} SITL 上傳失敗: {e}'
                        )

        # ── 快取結果 + 渲染 ────────────────────────────────────
        self._recon_strike_manager = mgr
        self._recon_strike_report = report

        # ── 提示對話框 ────────────────────────────────────────
        selected = [uav_states[uid].name for uid in report.selected_uav_ids]
        rejected = [uav_states[uid].name for uid in report.rejected_uav_ids]
        conflict_txt = ''
        if report.iapf_conflicts:
            conflict_txt = f'\n\nIAPF 衝突 ({len(report.iapf_conflicts)}):\n' + '\n'.join(
                f'  ! {c}' for c in report.iapf_conflicts[:3]
            )
        if report.iapf_adjustments:
            conflict_txt += f'\n\nIAPF 避障調整:\n' + '\n'.join(
                f'  + {a}' for a in report.iapf_adjustments[:3]
            )

        QMessageBox.information(
            self, '⚡ 動態切換完成',
            f'偵測事件: 目標 ({tgt_lat:.6f}, {tgt_lon:.6f})\n\n'
            f'打擊聯盟 ({len(selected)} 架): {", ".join(selected)}\n'
            f'繼續 DCCPP ({len(rejected)} 架): {", ".join(rejected) or "-"}\n\n'
            f'基準 TOT: {report.tot_sec:.2f} s\n'
            f'高度層: {min(report.altitude_layers.values()):.0f}m ~ '
            f'{max(report.altitude_layers.values()):.0f}m\n\n'
            f'SITL 上傳: {sitl_uploaded} / {len(mgr.assignments)} 架'
            + conflict_txt
        )
        self.statusBar().showMessage(
            f'[ReconToStrike] {len(selected)} 架轉打擊, TOT={report.tot_sec:.1f}s, '
            f'SITL 上傳={sitl_uploaded}', 6000,
        )

        # 更新面板預覽 (Task 3: CollisionReport UI)
        self.parameter_panel.update_dtot_preview(
            f'[偵打切換] 聯盟={len(selected)} 架, TOT={report.tot_sec:.1f}s'
        )
        self.parameter_panel.update_collision_report(
            safe=not report.iapf_conflicts,
            conflicts=report.iapf_conflicts,
        )

    # ═════════════════════════════════════════════════════════════════
    #  VTOL 模式切換處理
    # ═════════════════════════════════════════════════════════════════
    def _on_strike_vtol_toggled(self, enabled: bool):
        """VTOL 模式切換回饋（UI 狀態提示）"""
        msg = ('🛩 VTOL 模式已啟用 — 下次 EXECUTE 將使用 NAV_VTOL_TAKEOFF + Phase 2/3'
               if enabled else
               '固定翼模式 — 使用標準 NAV_TAKEOFF + 單段巡航')
        self.statusBar().showMessage(msg, 4000)
        logger.info(f'[Strike] VTOL 模式 = {enabled}')

