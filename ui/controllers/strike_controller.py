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

import json
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
                # fallback: 直接設定 + 在 3D 地圖打基地標記
                self._strike_launch_base = (lat, lon)
                self.parameter_panel.update_strike_base_label(lat, lon)
                try:
                    if hasattr(self.map_widget, 'set_home_point_overlay'):
                        self.map_widget.set_home_point_overlay(lat, lon)
                except Exception:
                    pass

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
        # 把最新點擊的座標同步回 ⓿ 區 lat/lon spinbox（單目標模式）
        if hasattr(self.parameter_panel, 'set_strike_target_coord'):
            self.parameter_panel.set_strike_target_coord(lat, lon)
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
        """執行蜂群打擊 — Diamond 4 機菱形編隊 + STOT 飽和打擊（唯一流程）。

        2026 重構：移除舊的 TerminalStrikePlanner / VTOL 分支
                 與其產生的垂直螺旋俯衝視覺。所有蜂群打擊統一走
                 DiamondSwarmStrikePlanner（直線爬升、平緩水平巡航、
                 4 向同時俯衝）。
        """
        # ── 前置檢查：必須有 1 個目標 + 已標基地 ─────────────────
        if len(self._strike_targets) != 1:
            QMessageBox.warning(
                self, '蜂群打擊 — 目標未設定',
                '菱形 STOT 飽和打擊需要恰好 1 個目標。\n'
                '請於 ⓿ 區直接輸入目標座標，或啟用「在 3D 地圖上點選目標」後左鍵點擊地圖。'
            )
            return
        if self._strike_launch_base is None:
            QMessageBox.warning(
                self, '蜂群打擊 — 發射基地未設定',
                '需先標記共用發射基地（4 機都從此點起飛）。\n'
                '請於 ① 區點擊「📍 從 3D 地圖標記發射基地」，再回地圖左鍵點擊位置。'
            )
            return
        self._on_diamond_strike_execute(params)
        return  # ★ 不再 fallthrough 到舊 TerminalStrikePlanner（避免螺旋俯衝視覺殘留）

    # ═════════════════════════════════════════════════════════════════
    #  Diamond Swarm Strike — 4 機菱形編隊 + STOT 飽和打擊
    # ═════════════════════════════════════════════════════════════════
    def _on_diamond_strike_execute(self, params: dict):
        """
        執行菱形編隊蜂群打擊（單一目標、4 機 4 方位同時俯衝）。
        參考 core.strike.diamond_swarm_planner.DiamondSwarmStrikePlanner。
        """
        from core.strike.diamond_swarm_planner import (
            DiamondSwarmStrikePlanner, FormationConfig, Target,
        )

        # 關閉進行中的標記模式
        if self._strike_marking_mode:
            self._on_strike_mark_targets()
        if self._strike_base_marking_mode:
            self._toggle_strike_base_marking(False)

        tgt_lat, tgt_lon = self._strike_targets[0]
        target = Target(lat=tgt_lat, lon=tgt_lon, alt=0.0, name='TGT')

        cruise_speed = float(params.get('cruise_speed', 60.0))
        # 固定翼最小轉彎半徑：依 30° 滾轉角推導 R = V² / (g·tan φ)
        turn_r_default = max(cruise_speed**2 / (9.81 * math.tan(math.radians(30.0))), 80.0)

        # 從面板讀新增的編隊與四散參數（N、縱向/橫向間距、決斷圈、攻擊方位）
        fp = self.parameter_panel.get_strike_formation_params()
        n_uavs  = max(2, min(int(fp.get('n_uavs', 4)), 12))
        long_m  = max(fp.get('longitudinal_m', 300.0), 100.0)
        lat_m   = max(fp.get('lateral_m', 400.0), 100.0)   # 總跨距 → 半跨距 = lat_m/2
        disp_r  = max(fp.get('dispersion_radius_m', 3000.0), 500.0)

        # 攻擊方位處理：
        # 一律傳 None 讓 planner 用「同側半圈扇面均分」自動公式：
        #   bearings[i] = back_bearing - 90° + i × 180°/(N-1)
        # 不論基地朝目標是哪個方向，都會自動以 cruise_heading 為基準軸
        # 將 N 個方位平均分布在 base 那一側 180° 半圈內（端點含 ±90°）。
        # 這比舊版「N=4 用 UI 上 [0,90,180,270] 全方位包圍」戰術更佳：
        #   ① 不繞過目標、不在目標上空交會 ② 飛行距離最短 ③ STOT 補時更小
        # 若需自訂方位，可改成讀 UI 上的非預設值（保留註解的 user_bearings 邏輯）。
        bearings = None
        # （保留：若使用者明確改過 UI 4 個方位 spinbox 為非預設值，可自訂）
        # if n_uavs == 4:
        #     user_bearings = fp.get('attack_bearings', [0.0, 90.0, 180.0, 270.0])
        #     # 偵測：若 4 值仍為預設 [0, 90, 180, 270]，視為「未自訂」用自動公式
        #     defaults = [0.0, 90.0, 180.0, 270.0]
        #     if any(abs(float(b) - d) > 0.5 for b, d in zip(user_bearings[:4], defaults)):
        #         bearings = tuple(float(b) % 360.0 for b in user_bearings[:4])

        # 機間錯開參數（避免空中碰撞 + 波次攻擊）
        # timing_mode: 'STOT' (同秒命中) / 'DTOT' (依方位順序間隔命中)
        # altitude_step: 各機巡航高度差 (m)；建議 ≥ 50m
        # interval_sec : DTOT 模式機間命中時間間隔 (s)
        timing_mode_raw = str(params.get('timing_mode', 'STOT')).upper()
        cfg = FormationConfig(
            n_uavs               = n_uavs,
            front_back_spacing_m = long_m,
            left_right_half_m    = lat_m / 2.0,
            cruise_alt_m         = float(params.get('cruise_alt', 500.0)),
            takeoff_alt_m        = float(params.get('takeoff_alt', 80.0)),
            takeoff_pitch_deg    = float(params.get('takeoff_pitch', 12.0)),
            cruise_speed_mps     = cruise_speed,
            decision_ring_m      = disp_r,
            pre_strike_radius_m  = max(disp_r * 0.5, 800.0),
            loiter_radius_m      = 120.0,
            dive_initiation_m    = float(params.get('dive_initiation_dist', 800.0)),
            max_dive_angle_deg   = float(params.get('max_dive_angle', 45.0)),
            min_turn_radius_m    = float(params.get('min_turn_radius', turn_r_default)),
            altitude_step_m      = float(params.get('altitude_step', 0.0)),
            timing_mode          = timing_mode_raw if timing_mode_raw in ('STOT', 'DTOT') else 'STOT',
            time_interval_s      = float(params.get('interval_sec', 0.0)),
            attack_bearings_deg  = bearings,
        )

        planner = DiamondSwarmStrikePlanner(cfg)
        try:
            plans = planner.plan(target=target, base_latlon=self._strike_launch_base)
        except Exception as e:
            logger.error(f'[DiamondStrike] 規劃失敗: {e}', exc_info=True)
            QMessageBox.warning(self, '蜂群打擊失敗',
                f'菱形編隊規劃發生錯誤:\n{e}')
            return

        # 快取結果供匯出 / SITL 上傳使用
        self._diamond_plans = plans
        self._diamond_target = target
        self._diamond_cfg = cfg

        # ── 更新底部 STOT TIME-ON-TARGET 同步狀態儀表板 ─────────
        # 顯示 N 張 UAV 卡片：TTT / Loiter / Delay 補償 / 距離 / 方位 / 高度
        # 4 機 TTT 應顯示為一致數字（STOT 命中時刻同步證明）
        try:
            dash = getattr(self, 'strike_ttt_dashboard', None)
            if dash is not None:
                dash.set_plans(
                    plans=plans,
                    target_lat=target.lat, target_lon=target.lon,
                    timing_mode=cfg.timing_mode,
                    delay_strategy=cfg.delay_strategy,
                )
                dash.setVisible(True)
        except Exception as e:
            logger.warning(f'[TTT Dashboard] 更新失敗: {e}', exc_info=True)

        # ── 3D 地圖視覺化：每架 UAV 一條完整連續軌跡（不同顏色）──
        # 為避免「N 機編隊段在 km 級地圖上重疊看起來像 1 條線」的視覺問題，
        # 改成「每機一條從基地→目標的連續 polyline」用 UAV_PALETTE 不同色繪製，
        # 再額外疊加：決斷圈、盤旋圓、俯衝段（紅色發光線）強調戰術元素。
        try:
            from ui.resources.tactical_theme import TacticalColors as TC
            uav_palette = list(TC.UAV_PALETTE) + list(TC.REGION_PALETTE)
        except Exception:
            uav_palette = ['#FFB703', '#FF003C', '#00E676', '#00B4D8',
                            '#C77DFF', '#80FFDB', '#FF8500', '#7CB9E8',
                            '#08EC91', '#FFB703', '#00B4D8', '#FF003C']

        if hasattr(self.map_widget, 'map_3d'):
            cm = self.map_widget.map_3d
            cm._js('clearPaths()')
            cm._js('clearTacticalOverlay()')
            # 清除舊版（TerminalStrikePlanner / VTOL）視覺殘留
            cm._js('if (typeof strikeClearAll === "function") strikeClearAll();')
            cm._js('if (typeof clearAllTerminalDives === "function") clearAllTerminalDives();')

            # ── 蜂群打擊不使用 Geofence ────────────────────────
            # 蜂群末端飽和打擊 = 自殺撞擊任務，不能讓 fence 攔截俯衝。
            # 清除 3D 地圖上的 fence 視覺、清除 dual_map_widget 的 fence
            # bundle 快取，並避免 SITL 上傳階段被 fence 干擾。
            try:
                cm.clear_geofence()
            except Exception:
                pass
            try:
                # 清除 dual_map_widget 內部 fence bundle 快取
                if hasattr(self.map_widget, '_last_fence_bundle'):
                    self.map_widget._last_fence_bundle = None
                # 通知 fence_built 信號清空（若有 listener）
                if hasattr(self.map_widget, 'fence_built'):
                    self.map_widget.fence_built.emit(None)
            except Exception:
                pass

            # 1) 每架 UAV 一條完整 polyline（formation + dispersion + dive 串接）
            for idx, plan in enumerate(plans):
                color = uav_palette[idx % len(uav_palette)]
                # 拼接全程路徑，避免重複起終點
                full_path = list(plan.formation_path)
                if plan.dispersion_path:
                    # 跳過第一點（與 formation_path 末端重複）
                    full_path.extend(plan.dispersion_path[1:])
                if plan.dive_path:
                    # 注意：dive 第一點 = pre_strike 的延伸；若與前段末端重複就跳過
                    if (full_path and len(plan.dive_path) >= 1
                            and abs(full_path[-1][0] - plan.dive_path[0][0]) < 1e-6
                            and abs(full_path[-1][1] - plan.dive_path[0][1]) < 1e-6):
                        full_path.extend(plan.dive_path[1:])
                    else:
                        full_path.extend(plan.dive_path)

                if len(full_path) >= 2:
                    pts_json = cm._pts_to_json(full_path,
                                                default_alt=cfg.cruise_alt_m)
                    label = (f'UAV{plan.sysid} {plan.role} '
                             f'@{plan.attack_bearing_deg:.0f}° '
                             f'(loiter {plan.loiter_time_s:.0f}s)')
                    cm._js(
                        f'addPath({json.dumps(pts_json)},'
                        f'{json.dumps(color)},5,'
                        f'{json.dumps(label)},false,"off")'
                    )

                # 預備點盤旋圓 — 用該 UAV 的色，標示 STOT 補時位置
                cm._js(
                    f'addLoiterCircle({plan.pre_strike_lat},{plan.pre_strike_lon},'
                    f'{cfg.loiter_radius_m},{cfg.cruise_alt_m},'
                    f'{json.dumps(color)})'
                )
                # Phase 3 俯衝紅線（醒目，與 UAV 色獨立 — 強調戰術終端）
                cm._js(
                    f'addDiveLine({plan.dive_lat},{plan.dive_lon},{cfg.cruise_alt_m},'
                    f'{tgt_lat},{tgt_lon},{target.alt})'
                )

            # 2) 決斷圈（琥珀虛線圓，目標為圓心）
            cm._js(
                f'setDispersionRing({tgt_lat},{tgt_lon},{cfg.decision_ring_m})'
            )
            # 3) 紅色目標標記
            cm._js(f'strikeAddTarget({tgt_lat},{tgt_lon},1)')

        # 啟用匯出 / 上傳按鈕（用 N 替換固定的 1）
        self.parameter_panel.set_strike_export_enabled(True)
        self.parameter_panel.update_strike_target_count(1)

        # 狀態列摘要 — 改成依實際 N 顯示
        arrivals = [p.final_leg_time_s + p.loiter_time_s for p in plans]
        spread_ms = (max(arrivals) - min(arrivals)) * 1000.0
        msg = (f'🎯 菱形 STOT 規劃完成: {len(plans)} 機 → '
               f'同秒命中誤差 {spread_ms:.0f} ms '
               f'| 巡航 {cfg.cruise_speed_mps:.0f}m/s | '
               f'最大俯衝 {cfg.max_dive_angle_deg:.0f}°')
        self.statusBar().showMessage(msg, 8000)
        logger.info(msg)
        for p in plans:
            logger.info(
                f'  UAV{p.sysid} [{p.role}] '
                f'attack@{p.attack_bearing_deg:.0f}° '
                f'leg={p.final_leg_length_m:.0f}m '
                f't={p.final_leg_time_s:.1f}s '
                f'loiter={p.loiter_time_s:.1f}s '
                f'WPs={len(p.mission)}'
            )

    def _on_param_tab_changed(self, tab_name: str):
        """
        參數面板分頁切換 → 調整地圖點擊路由。
        進入「蜂群打擊」自動啟用目標標記模式；離開時關閉。
        這樣使用者切到蜂群打擊分頁後，左鍵點擊地圖立即新增 / 設定打擊目標，
        而不會誤觸成「新增邊界角點」。
        """
        is_strike = ('蜂群' in (tab_name or '') or 'Strike' in (tab_name or ''))
        # 進入打擊頁 → 啟用 marking（若尚未啟用）
        if is_strike and not self._strike_marking_mode:
            self._on_strike_mark_targets()
        # 離開打擊頁 → 關閉所有 strike 標記模式（避免到別的頁面誤點）
        elif not is_strike:
            if self._strike_marking_mode:
                self._on_strike_mark_targets()
            if self._strike_base_marking_mode:
                self._toggle_strike_base_marking(False)

    def _on_strike_target_coord_input(self, lat: float, lon: float):
        """使用者在 ⓿ 區手動輸入目標經緯 → 同步到 _strike_targets + 3D marker。"""
        self._strike_targets = [(float(lat), float(lon))]
        self.parameter_panel.update_strike_target_count(1)
        # 同步在 3D 地圖打紅色目標標記
        if hasattr(self.map_widget, 'map_3d'):
            try:
                self.map_widget.map_3d._js(
                    f'strikeClearTargets(); strikeAddTarget({lat},{lon},1);'
                )
            except Exception:
                pass

    # ─────────────────────────────────────────────────────────────────
    #  ArduPlane SITL — 蜂群末端飽和打擊「KAMIKAZE 全限制解除」參數
    # ─────────────────────────────────────────────────────────────────
    # ⚠️  警告：此參數組僅供 SITL 自殺攻擊模擬使用，**禁止部署到實機**。
    #     所有 fence / failsafe / 角度 / 速度 / 電量 / 墜機偵測等限制
    #     均已解除，目的是讓 4 機能完整執行「並排起飛 → 12° 斜爬升 →
    #     菱形編隊 → 解散 → 同秒命中」全流程，不被任何安全機制中斷。
    #
    # 解除類別：
    #   ① Pre-arm / Arming check：完全跳過
    #   ② Failsafe：油門/GCS/電池/EKF/長短失聯 全關
    #   ③ Geofence：FENCE_ENABLE=0 + FENCE_TYPE=0 + FENCE_ACTION=0
    #   ④ 角度極限：俯仰 -90°~+45°、滾轉 ±85°
    #   ⑤ 空速：5 ~ 120 m/s（自由俯衝衝刺）
    #   ⑥ TECS：爬升 15 m/s、下沉 80 m/s
    #   ⑦ 節流：0~100% / slewrate 200%/s
    #   ⑧ 墜機偵測：完全關閉（避免命中時被識為墜毀觸發 disarm）
    #   ⑨ 起飛 / 速度 / GPS HDOP / RC failsafe 全部跳過
    # ─────────────────────────────────────────────────────────────────
    DIAMOND_SITL_PRESET_PARAMS = [
        # ── ① ARM / Pre-arm 全部跳過 ──────────────────────────
        ('ARMING_CHECK',     0,    'INT32'),  # bitmask 0 = 不檢查任何項
        ('ARMING_REQUIRE',   0,    'INT8'),   # 0 = 不需要 arm 即可動作

        # ── ② Failsafe 全部關閉 ───────────────────────────────
        ('FS_THR_ENABLE',    0,    'INT32'),
        ('FS_GCS_ENABLE',    0,    'INT32'),
        ('FS_BATT_ENABLE',   0,    'INT32'),
        ('FS_EKF_ACTION',    1,    'INT32'),  # 1 = 僅報警不動作（0 會 disable EKF）
        ('FS_EKF_THRESH',    1.0,  'REAL32'), # EKF innovation 容忍度放寬
        ('FS_LONG_ACTN',     0,    'INT8'),
        ('FS_SHORT_ACTN',    0,    'INT8'),
        ('FS_LONG_TIMEOUT',  300.0,'REAL32'), # 5 分鐘才觸發長失聯
        ('FS_SHORT_TIMEOUT', 60.0, 'REAL32'),
        ('THR_FAILSAFE',     0,    'INT8'),

        # ── ③ 電池 failsafe 全部關（自殺機不會回家） ─────────
        ('BATT_LOW_VOLT',    0.0,  'REAL32'),
        ('BATT_CRT_VOLT',    0.0,  'REAL32'),
        ('BATT_LOW_MAH',     0,    'INT32'),
        ('BATT_CRT_MAH',     0,    'INT32'),
        ('BATT_FS_LOW_ACT',  0,    'INT8'),
        ('BATT_FS_CRT_ACT',  0,    'INT8'),

        # ── ④ Geofence 全部關（允許飛任何高度任何位置）────────
        # 蜂群打擊就是要俯衝撞地，不能讓 fence 攔截
        ('FENCE_ENABLE',     0,    'INT8'),
        ('FENCE_AUTOENABLE', 0,    'INT8'),
        ('FENCE_ACTION',     0,    'INT8'),
        ('FENCE_TYPE',       0,    'INT8'),    # bitmask 0 = 不啟用任何 fence 類型
        ('FENCE_ALT_MAX',    99999.0, 'REAL32'),
        ('FENCE_ALT_MIN',    -1000.0, 'REAL32'),
        ('FENCE_RADIUS',     999999.0,'REAL32'),
        ('FENCE_MARGIN',     0.0,  'REAL32'),
        ('FENCE_TOTAL',      0,    'INT8'),    # 清空 polygon fence

        # ── ⑤ GPS / EKF / 感測器健康檢查放寬 ──────────────────
        ('GPS_HDOP_GOOD',    900,  'INT16'),   # HDOP=9.0 也視為可用
        ('EK3_GLITCH_RAD',   1000, 'INT16'),   # GPS glitch 容忍 1000m
        ('EK3_CHECK_SCALE',  500,  'INT16'),   # EKF 健康檢查門檻放寬

        # ── ⑥ 起飛立即點火，跳過所有起飛條件檢查 ─────────────
        ('TKOFF_THR_MINACC', 0.0,  'REAL32'),
        ('TKOFF_THR_MINSPD', 0.0,  'REAL32'),
        ('TKOFF_THR_MAX',    100,  'INT16'),
        ('TKOFF_THR_DELAY',  0,    'INT8'),
        ('TKOFF_LVL_PITCH',  10.0, 'REAL32'),
        ('TKOFF_TDRAG_ELEV', 0,    'INT8'),
        ('TKOFF_TDRAG_SPD1', 0.0,  'REAL32'),
        ('TKOFF_ROTATE_SPD', 0.0,  'REAL32'),
        ('TKOFF_FLAP_PCNT',  0,    'INT8'),
        ('GROUND_STEER_ALT', -1.0, 'REAL32'),

        # ── ⑦ 空速完全放開（5 ~ 120 m/s） ─────────────────────
        ('AIRSPEED_MIN',     5,    'INT8'),    # 從 8 降到 5
        ('AIRSPEED_CRUISE',  18,   'INT8'),
        ('AIRSPEED_MAX',     120,  'INT8'),    # 從 60 提到 120 m/s（俯衝衝刺極限）
        ('STALL_PREVENTION', 0,    'INT8'),    # 完全關失速保護

        # ── ⑧ 俯仰角度完全解除（-90° ~ +45°） ──────────────────
        # 允許接近垂直俯衝撞擊；centidegrees 單位
        ('LIM_PITCH_MIN',   -9000, 'INT16'),   # -90° 完全垂直俯衝
        ('LIM_PITCH_MAX',    4500, 'INT16'),   # +45° 急上仰
        ('TECS_PITCH_MIN',  -89.0, 'REAL32'),  # 接近垂直
        ('TECS_PITCH_MAX',   45.0, 'REAL32'),

        # ── ⑨ 滾轉角放寬（±85°） ──────────────────────────────
        ('LIM_ROLL_CD',      8500, 'INT16'),   # 85° 急轉彎
        ('ACRO_PITCH_RATE',   720, 'INT16'),
        ('ACRO_ROLL_RATE',    720, 'INT16'),

        # ── ⑩ TECS 完全鬆綁 ───────────────────────────────────
        ('TECS_CLMB_MAX',    15.0, 'REAL32'),  # 從 8 提到 15 m/s
        ('TECS_SINK_MAX',    80.0, 'REAL32'),  # 從 50 提到 80 m/s
        ('TECS_SPDWEIGHT',    0.0, 'REAL32'),  # 0 = 完全用高度，不顧空速
        ('TECS_LAND_SPDWGT',  0.0, 'REAL32'),

        # ── ⑪ 節流完全放開（0 ~ 100% / 200%/s） ───────────────
        ('THR_MAX',          100,  'INT8'),
        ('THR_MIN',            0,  'INT8'),
        ('THR_SLEWRATE',     200,  'INT8'),    # 從 100 提到 200%/s（俯衝瞬間響應）
        ('THR_PASS_STAB',      0,  'INT8'),

        # ── ⑫ 墜機偵測完全關閉 ────────────────────────────────
        # 自殺撞擊時 ArduPlane 會偵測到 G 力過大「以為墜毀」自動 disarm
        # 馬達 → 飛機在最後關頭失去控制力。必須關閉。
        ('CRASH_ACC_THRESH', 0.0,  'REAL32'),
        ('CRASH_DETECT',     0,    'INT8'),

        # ── ⑬ Mission 行為 ────────────────────────────────────
        ('MIS_DONE_BEHAVE',  0,    'INT8'),    # 0=HOLD（不 RTL）
        ('MIS_RESTART',      0,    'INT8'),    # 不要重啟 mission
        ('LAND_DISARMDELAY', 0,    'INT8'),
        # ── ⑬b RTL 完全失效 — 即便被觸發 RTL 也不要返航 ──────
        # mission 已加 NAV_LOITER_UNLIM 防止「mission done」進 RTL，
        # 但仍可能被 fence breach / failsafe 觸發 RTL。把 RTL 設成
        # 「就地停留 + 不爬升 + 不降落」確保即便進 RTL 也不離開目標區。
        ('RTL_ALT',           0,   'INT16'),   # RTL 不爬升
        ('RTL_AUTOLAND',      0,   'INT8'),    # RTL 不自動降落
        ('RTL_RADIUS',        0,   'INT16'),
        ('RTL_CLIMB_MIN',     0,   'INT16'),
        ('AFS_ENABLE',        0,   'INT8'),    # Advanced Failsafe 完全關閉

        # ── ⑭ Waypoint approach 半徑（避免鋸齒軌跡） ─────────
        # 固定翼 min_turn_radius ≈ 150m，若 WP_RADIUS 太小（如 10m）飛機會
        # 「飛到 wp 才開始轉」 → overshoot → U-turn 回頭 → 鋸齒之字形軌跡。
        # WP_RADIUS = 80m 讓飛機在 wp 前 80m 就 advance，預留轉彎空間。
        # 但 mission 內 TARGET / OVERSHOOT 用 param2 個別覆蓋為 3-5m 確保精準命中。
        ('WP_RADIUS',         80,  'INT16'),   # approach radius (m) — 全局
        ('WP_LOITER_RAD',    150,  'INT16'),   # 盤旋半徑與 cfg.loiter_radius 一致
        ('WP_MAX_RADIUS',    100,  'INT16'),   # 大角度轉彎時最大 advance 半徑

        # ── ⑭b L1 路徑追蹤控制器（解決「不按路線飛」）─────────
        # ArduPlane 用 L1 演算法追蹤 wp-to-wp 直線。預設 NAV_L1_PERIOD=20s
        # 在 60 m/s 巡航下 = 1200m 追蹤距離 → 飛機可大幅偏離規劃路線。
        # 降到 12s = 720m，飛機緊貼路線飛，但會略增滾轉率。
        ('NAV_L1_PERIOD',    12.0, 'REAL32'),  # 從 20 降到 12（緊跟路線）
        ('NAV_L1_DAMPING',   0.75, 'REAL32'),  # 阻尼比（0.75 略震盪、追隨快）
        ('NAV_L1_XTRACK_I',  0.05, 'REAL32'),  # cross-track 積分項（消除穩態誤差）
        ('PTCH2SRV_RLL',      1.0, 'REAL32'),  # 滾轉時自動補俯仰（保高度）
        # ── ⑭c TECS 響應加快（速度/高度同步控制） ────────────
        ('TECS_TIME_CONST',   4.0, 'REAL32'),  # 從預設 5 降到 4（更積極）
        ('TECS_THR_DAMP',     0.5, 'REAL32'),  # 節流阻尼
        ('TECS_INTEG_GAIN',   0.5, 'REAL32'),  # TECS 積分增益
        ('TECS_PTCH_DAMP',    0.5, 'REAL32'),  # 俯仰阻尼

        # ── ⑮ MAVLink 串流 ────────────────────────────────────
        ('SR0_POSITION',     5,    'INT16'),
        ('SR0_EXTRA1',       5,    'INT16'),
        ('SR0_EXTRA2',       5,    'INT16'),
        ('SR0_EXTRA3',       2,    'INT16'),
    ]

    def _on_diamond_strike_sitl_upload(self):
        """
        將菱形 STOT 任務上傳到所有 SITL：
          UAV1 → links[0],  UAV2 → links[1],  UAV3 → links[2],  UAV4 → links[3]
        若 SITL 實例少於 4 台 → 重複使用最後一個 link（並警告）
        每個 link 收到：
          1) DIAMOND_SITL_PRESET_PARAMS（友善起飛 + 終端俯衝參數）
          2) 自己 UAV 的完整 MissionItem 序列
        """
        plans = self._diamond_plans
        n_links = len(self._sitl_links)
        if n_links < len(plans):
            self.statusBar().showMessage(
                f'⚠ SITL 連線數 ({n_links}) 少於 UAV 數 ({len(plans)})，'
                f'多餘 UAV 會與最後一台共用 link', 6000
            )

        upload_count = 0
        for i, plan in enumerate(plans):
            link = self._sitl_links[min(i, n_links - 1)]
            # 0) 蜂群打擊不使用 fence — 主動清除飛控 EEPROM 中任何殘留 polygon
            #    fence 與 FENCE_ENABLE=0，避免上次任務的 fence 攔截俯衝。
            try:
                link.clear_fence()
            except Exception as e:
                logger.warning(f'[DiamondStrike] UAV{plan.sysid} 清除 fence 失敗: {e}')

            # 1) 先寫入起飛 + 終端俯衝友善參數（每架都需要）
            try:
                link.set_params(self.DIAMOND_SITL_PRESET_PARAMS)
            except Exception as e:
                logger.warning(f'[DiamondStrike] UAV{plan.sysid} 參數寫入失敗: {e}')

            # 2) 上傳任務 — MissionItem → (lat, lon, alt, cmd, p1, p2, p3, p4)
            #    跳過 plan.mission[0]（HOME） — sitl_link._send_mission 會自動
            #    在 seq=0 補一份 HOME。若不跳過會出現雙 HOME，使 mission 第一個
            #    NAV 命令變成 NAV_WAYPOINT @ alt=0 → AUTO 模式卡在地面不起飛。
            wps = []
            mission_for_upload = plan.mission[1:] if plan.mission else []
            for it in mission_for_upload:
                wps.append((it.lat, it.lon, it.alt, it.cmd,
                             it.param1, it.param2, it.param3, it.param4))
            try:
                link.upload_mission(wps)
                upload_count += 1
            except Exception as e:
                logger.error(
                    f'[DiamondStrike] UAV{plan.sysid} 上傳失敗: {e}', exc_info=True
                )

        # 顯示「上傳完成」對話框，包含啟動策略說明
        delay_summary = ', '.join(
            f'UAV{p.sysid}={p.takeoff_delay_s:.0f}s'
            for p in plans if p.takeoff_delay_s > 0.5
        ) or '無'
        loiter_summary = ', '.join(
            f'UAV{p.sysid}={p.loiter_time_s:.0f}s'
            for p in plans if p.loiter_time_s > 0.5
        ) or '無'
        QMessageBox.information(
            self, '✅ 菱形 STOT 任務上傳完成',
            f'已將 {len(plans)} 機任務送出到 {upload_count} 條 SITL 連線。\n\n'
            f'⚠ 重要：請按 [KAMIKAZE LAUNCH] 按鈕同步啟動，\n'
            f'   而非手動逐一切 AUTO（GCS 端會依規劃延遲送命令給較近的 UAV）。\n\n'
            f'各機地面延遲: {delay_summary}\n'
            f'各機空中盤旋: {loiter_summary}'
        )
        self.statusBar().showMessage(
            f'🚀 菱形 STOT 上傳完成 → {upload_count} 台 SITL，'
            f'請按 KAMIKAZE LAUNCH 同步啟動', 8000
        )

    # ─────────────────────────────────────────────────────────────────
    #  GCS 端排程啟動 — 解決 ArduPlane NAV_DELAY 在地面不生效問題
    # ─────────────────────────────────────────────────────────────────
    def launch_kamikaze_synchronized(self):
        """同步啟動所有 SITL 蜂群打擊任務，依 plan.takeoff_delay_s 在 GCS 端錯開。

        為什麼用 GCS 端排程而非 mission 內 NAV_DELAY？
            ArduPlane 在 NAV_TAKEOFF 之前的 NAV_DELAY 會被忽略
            （官方文件：「飛機需在空中才會 wait at current location」）。
            因此改在 GCS 端用 QTimer 對「較近的 UAV」延後送 AUTO+ARM 命令，
            飛機會停在跑道上直到收到 mode 切換指令才解鎖起飛。

        排程策略 (假設 STOT, 各機 takeoff_delay_s 為 0/8.8/17.6/66s)：
            t=0s    UAV3 (基準機 delay=0) 立即收到 AUTO+ARM
            t=8.8s  UAV2 收到 AUTO+ARM
            t=17.6s UAV4 收到 AUTO+ARM
            t=66s   UAV1 收到 AUTO+ARM

        呼叫時機：
            蜂群打擊規劃 + 上傳 SITL 完成後，由 TTT Dashboard
            「KAMIKAZE LAUNCH」按鈕觸發。
        """
        from PyQt6.QtCore import QTimer

        plans = getattr(self, '_diamond_plans', None)
        if not plans:
            QMessageBox.information(
                self, '無蜂群打擊任務',
                '尚未規劃蜂群打擊或已被清除，請先按「執行打擊」生成計畫'
            )
            return

        n_links = len(self._sitl_links)
        if n_links == 0:
            QMessageBox.warning(
                self, 'SITL 未連線',
                '尚未連線任何 SITL，請先在 SITL 分頁啟動 + 連線'
            )
            return

        scheduled = []
        for i, plan in enumerate(plans):
            if i >= n_links:
                break  # link 數少於 UAV 數時，超出的 plan 不啟動
            link = self._sitl_links[i]
            delay_s = max(float(plan.takeoff_delay_s), 0.0)
            delay_ms = int(round(delay_s * 1000.0))

            # 用 default 參數綁定 link 避免 lambda late-binding bug
            def _launch(L=link, sid=plan.sysid, ds=delay_s):
                try:
                    L.auto_start()  # AUTO + ARM + MISSION_START 三步驟
                    logger.info(
                        f'[KAMIKAZE] UAV{sid} 已送出 AUTO+ARM (delay={ds:.1f}s)'
                    )
                except Exception as e:
                    logger.error(f'[KAMIKAZE] UAV{sid} 啟動失敗: {e}', exc_info=True)

            if delay_ms <= 0:
                _launch()
            else:
                QTimer.singleShot(delay_ms, _launch)
            scheduled.append((plan.sysid, delay_s))

        # 同步啟動 TTT Dashboard 倒數
        try:
            dash = getattr(self, 'strike_ttt_dashboard', None)
            if dash is not None:
                dash.start_countdown()
        except Exception:
            pass

        sched_text = ', '.join(
            f'UAV{sid}@+{ds:.1f}s' for sid, ds in scheduled
        )
        self.statusBar().showMessage(
            f'🚀 KAMIKAZE LAUNCH ⚠ 已排程 {len(scheduled)} 機: {sched_text}',
            10000,
        )
        logger.info(f'[KAMIKAZE] 同步啟動排程: {sched_text}')

        # ─────────────────────────────────────────────────────────────
        #  Strike Visual Overlay — 啟動即時視覺化 + 訂閱 telemetry
        # ─────────────────────────────────────────────────────────────
        self._start_strike_visualization(plans, scheduled)

    # ═════════════════════════════════════════════════════════════════
    #  閉環終端同步打擊 — FleetRegistry 共享黑板 + 同步釋放 + ToT 速度修正
    # ═════════════════════════════════════════════════════════════════
    def launch_terminal_sync_strike(self):
        """閉環終端同步打擊：以 FleetRegistry 共享黑板為態勢源，分段同步釋放 + 飛行中 ToT
        速度修正 → 確保多機同時命中（實測命中散度 ~4s，遠優於開環 ~38s）。

        與 launch_kamikaze_synchronized（開環：mission 內 S 機動/盤旋補時 + 排程起飛）互補：
        建議流程「先 KAMIKAZE LAUNCH 起飛巡航 → 各機接近目標區後按 TERMINAL SYNC 收尾」。
        本流程每個 tick 讀「全機」即時態勢（snapshot），每架的指令都是全機狀態的函數
        （同步釋放閘＝全員就位、ToT 速度＝最遠機 ETA、防撞＝兩兩間隔）→ 各機資訊互通。
        """
        from PyQt6.QtCore import QTimer
        plans = getattr(self, '_diamond_plans', None)
        target = getattr(self, '_diamond_target', None)
        cfg = getattr(self, '_diamond_cfg', None)
        if not plans or target is None:
            QMessageBox.information(
                self, '無蜂群打擊任務',
                '尚未規劃蜂群打擊，請先按「執行打擊」生成菱形 STOT 計畫')
            return
        if not getattr(self, '_sitl_links', None):
            QMessageBox.warning(self, 'SITL 未連線', '尚未連線任何 SITL')
            return

        from mission.fleet_registry import FleetRegistry
        from core.strike.terminal_sync_coordinator import TerminalSyncCoordinator
        from core.strike.terminal_sync import equidistant_push_points, layered_altitudes

        reg = FleetRegistry.instance()
        snap = reg.snapshot()                       # ★ 共享黑板：全機即時態勢
        seen = [p.sysid for p in plans if p.sysid in snap]
        if len(seen) < len(plans):
            missing = [p.sysid for p in plans if p.sysid not in snap]
            ans = QMessageBox.question(
                self, '部分機尚未回報遙測',
                f'FleetRegistry 共享黑板目前只看到 {len(seen)}/{len(plans)} 機'
                f'（缺 UAV {missing}）。\n協調器會等到全機就位才同步釋放；仍要啟動嗎？',
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            if ans != QMessageBox.StandardButton.Yes:
                return

        # ── 由 plan 建構等距 push points + 分層平飛高度（缺值則用幾何回退）──
        ordered = sorted(plans, key=lambda x: x.sysid)
        bearings = [float(p.attack_bearing_deg) for p in ordered]
        stage_m = float(getattr(cfg, 'pre_strike_radius_m', 1500.0)) or 1500.0
        fallback_pp = equidistant_push_points(target.lat, target.lon, bearings, stage_m)
        base_alt = float(getattr(cfg, 'cruise_alt_m', 200.0)) or 200.0
        step = max(float(getattr(cfg, 'altitude_step_m', 0.0)), 16.0)
        fallback_alt = layered_altitudes(len(plans), base_alt, step)

        push, approach = {}, {}
        for i, p in enumerate(ordered):
            if abs(p.pre_strike_lat) > 1e-9 and abs(p.pre_strike_lon) > 1e-9:
                push[p.sysid] = (float(p.pre_strike_lat), float(p.pre_strike_lon))
            else:
                push[p.sysid] = fallback_pp[i]
            approach[p.sysid] = float(p.cruise_alt) if p.cruise_alt > 1.0 else fallback_alt[i]

        v_cruise = float(getattr(cfg, 'cruise_speed_mps', 20.0))
        if v_cruise <= 1.0:
            v_cruise = 20.0
        v_min = max(v_cruise * 0.6, 10.0)
        v_max = max(v_cruise * 1.15, v_min + 4.0)   # 保證 v_max > v_min（小 cruise 時不退化）
        try:
            coord = TerminalSyncCoordinator(
                target.lat, target.lon, push, approach,
                v_min=v_min, v_max=v_max,
                arrive_m=max(stage_m * 0.18, 200.0),
                target_radius_m=150.0, impact_buffer_s=4.0,
            )
        except Exception as e:
            QMessageBox.critical(self, '終端同步協調器初始化失敗', f'參數無效：{e}')
            logger.error(f'[TSYNC] coordinator init failed: {e}', exc_info=True)
            return
        self._tsync = {'coord': coord, 'plans': plans, 'n': len(plans),
                       't0': None, 'last_log': -1e9, 'impact_t': {}}
        if getattr(self, '_tsync_timer', None) is None:
            self._tsync_timer = QTimer(self)
            self._tsync_timer.timeout.connect(self._tick_terminal_sync)
        self._tsync_timer.start(1000)

        # 鎖住儀表板兩個啟動鈕（與開環 KAMIKAZE 互斥），開 ABORT
        dash = getattr(self, 'strike_ttt_dashboard', None)
        if dash is not None and hasattr(dash, 'set_strike_running'):
            dash.set_strike_running(True)

        # 共用 3D 攻擊視覺化（cesium 不可用時自動略過）
        try:
            self._start_strike_visualization(plans, [(p.sysid, 0.0) for p in plans])
        except Exception:
            pass

        self.statusBar().showMessage(
            f'🎯 閉環終端同步打擊啟動：{len(plans)} 機（共享黑板看到 {len(seen)} 機）'
            f'→ 等距同步釋放 + ToT 修正中…', 8000)
        logger.info(
            f'[TSYNC] launch n={len(plans)} seen={len(seen)} stage={stage_m:.0f}m '
            f'v=[{coord.tot.v_min:.0f},{coord.tot.v_max:.0f}] arrive={coord.arrive_m:.0f}m')

    def _tick_terminal_sync(self):
        """1Hz：讀 FleetRegistry 全機快照 → 協調器 step() → 派送指令到各 SITLLink。"""
        import time as _t
        from mission.fleet_registry import FleetRegistry
        from core.strike.terminal_sync_coordinator import FleetState, PHASE_DONE
        ts = getattr(self, '_tsync', None)
        if not ts:
            if getattr(self, '_tsync_timer', None):
                self._tsync_timer.stop()
            return
        now = _t.monotonic()
        if ts['t0'] is None:
            ts['t0'] = now
        t = now - ts['t0']

        reg = FleetRegistry.instance()
        snap = reg.snapshot()                       # ★ 全機共享態勢（資訊互通讀取點）
        now_wall = _t.time()
        MAX_AGE_S = 5.0                              # 過期遙測不採用（交給協調器失聯邏輯）
        states = {}
        for p in ts['plans']:
            fr = snap.get(p.sysid)
            if fr is None:
                continue
            if now_wall - float(getattr(fr, 'last_update', now_wall)) > MAX_AGE_S:
                continue                            # 陳舊：連線掉/卡 → 視為缺，不據此下令
            states[p.sysid] = FleetState(
                p.sysid, float(fr.lat), float(fr.lon),
                float(fr.alt_rel), float(fr.ground_speed), t)
        res = ts['coord'].step(states, t)

        # 派送協調器算出的各機指令（每架命令皆為全機態勢函數）
        for cmd in res.commands:
            link = reg.get_link_by_sysid(cmd.sysid)
            if link is None:
                continue
            try:
                if cmd.kind == 'goto':
                    link.guided_goto(cmd.lat, cmd.lon, cmd.alt)
                elif cmd.kind == 'speed':
                    link.change_speed(cmd.speed)
            except Exception as e:
                logger.warning(f'[TSYNC] UAV{cmd.sysid} 指令 {cmd.kind} 失敗: {e}')

        for sid in res.impacted:
            ts['impact_t'].setdefault(sid, t)

        if t - ts['last_log'] >= 2.0:
            ts['last_log'] = t
            gate = f'UAV{res.gating_sysid}' if res.gating_sysid else '─'
            sep = ('%.0f' % res.min_separation_m) if res.min_separation_m < 1e8 else '∞'
            msg = (f'[TSYNC t={t:5.1f}] {res.phase} 就位{len(res.staged)}/{ts["n"]} '
                   f'命中{len(res.impacted)}/{ts["n"]} 見{len(res.seen_sysids)}機 '
                   f'minSep={sep}m 等待{gate} {res.note}')
            logger.info(msg)
            self.statusBar().showMessage(msg, 3000)

        if res.phase == PHASE_DONE or t > 900.0:
            self._tsync_timer.stop()
            its = ts['impact_t']
            if its and res.phase == PHASE_DONE:
                spread = max(its.values()) - min(its.values())
                summary = (f'✅ 終端同步打擊完成：{len(its)}/{ts["n"]} 機命中，'
                           f'命中散度 {spread:.1f}s（同時命中）')
            elif its:
                summary = f'⚠ 終端同步逾時：{len(its)}/{ts["n"]} 機命中'
            else:
                summary = '⚠ 終端同步結束：尚無命中紀錄'
            self.statusBar().showMessage(summary, 12000)
            logger.info(f'[TSYNC] {summary}')
            self._tsync = None
            dash = getattr(self, 'strike_ttt_dashboard', None)
            if dash is not None and hasattr(dash, 'set_strike_running'):
                dash.set_strike_running(False)

    def abort_terminal_sync(self):
        """中止進行中的閉環終端同步打擊（ABORT 按鈕也會呼叫）。

        停掉 tick timer、清狀態；不對已起飛的 UAV 發 RTL（與 KAMIKAZE ABORT 一致，
        避免誤把自殺機召回）。使用者可重新 TERMINAL SYNC 啟動。
        """
        if getattr(self, '_tsync_timer', None) is not None:
            try:
                self._tsync_timer.stop()
            except Exception:
                pass
        # 收掉本流程啟動的 3D 視覺化 telemetry 訂閱（避免懸空 handler）
        try:
            if getattr(self, '_strike_viz_state', None) is not None:
                self._finalize_strike_visualization()
        except Exception:
            pass
        if getattr(self, '_tsync', None) is not None:
            self._tsync = None
            self.statusBar().showMessage('⏹ 閉環終端同步打擊已中止（不召回已起飛 UAV）', 6000)
            logger.info('[TSYNC] aborted by user')
        # 還原儀表板按鈕（結束 running 狀態）
        dash = getattr(self, 'strike_ttt_dashboard', None)
        if dash is not None and hasattr(dash, 'set_strike_running'):
            dash.set_strike_running(False)

    def _start_strike_visualization(self, plans, scheduled):
        """KAMIKAZE LAUNCH 後啟動 3D 攻擊視覺化（軌跡 / HUD / 爆炸 / BDA）。

        流程：
            1) strike_viz_begin(target_lat, target_lon) 重置 JS 端 overlay
            2) 訂閱 FleetRegistry.telemetry_updated → _on_strike_viz_telemetry
            3) 每筆 telemetry 投到 strike_viz_update_uav
            4) 全機 IMPACT 或 timeout 600s → 拆除 + 開 BDA 對話框
        """
        if not self._strike_targets:
            return
        # 防重複訂閱：若已有進行中的視覺化（開環或上一輪），先收掉再重啟，
        # 避免 FleetRegistry.telemetry_updated 累積多個 handler。
        if getattr(self, '_strike_viz_state', None) is not None:
            try:
                self._finalize_strike_visualization()
            except Exception:
                pass
        cesium = self._get_cesium_widget()
        if cesium is None or not hasattr(cesium, 'strike_viz_begin'):
            return

        tgt_lat, tgt_lon = self._strike_targets[0]

        # 預建 sysid → 計畫 / ETA / loiter / callsign 對照表
        from mission.fleet_registry import FleetRegistry
        reg = FleetRegistry.instance()
        viz_state = {
            'plans_by_sysid': {},
            'sysid_to_callsign': {},
            'impacted': set(),
            'target_lat': tgt_lat,
            'target_lon': tgt_lon,
            'started_at': None,  # 第一筆 telemetry 進來才設
            'timeout_timer': None,
        }
        for p in plans:
            eta_s = (float(p.takeoff_delay_s)
                     + float(p.final_leg_time_s)
                     + float(p.loiter_time_s))
            viz_state['plans_by_sysid'][int(p.sysid)] = {
                'eta_s': eta_s,
                'loiter_lat': float(p.pre_strike_lat),
                'loiter_lon': float(p.pre_strike_lon),
                'loiter_radius_m': 120.0 if p.loiter_time_s > 0 else 0.0,
            }

        # sysid → callsign mapping（從 FleetRegistry 反查）
        try:
            for cs in reg.callsigns():
                link = reg.get_link(cs)
                if link is None:
                    continue
                sysid_label = int(getattr(link, 'sysid_label', 0) or 0)
                if sysid_label > 0:
                    viz_state['sysid_to_callsign'][sysid_label] = cs
        except Exception:
            pass

        cesium.strike_viz_begin(tgt_lat, tgt_lon, 0.0)
        self._strike_viz_state = viz_state

        # 訂閱 telemetry — 用 instance method 才好斷開
        reg.telemetry_updated.connect(self._on_strike_viz_telemetry)
        viz_state['_handler'] = self._on_strike_viz_telemetry

        # 600s 安全超時 → 強制收尾（避免某機卡住整個 BDA）
        from PyQt6.QtCore import QTimer
        t = QTimer(self)
        t.setSingleShot(True)
        t.timeout.connect(self._finalize_strike_visualization)
        t.start(600_000)
        viz_state['timeout_timer'] = t

        logger.info(
            f'[Strike VIZ] 啟動 — target=({tgt_lat:.5f}, {tgt_lon:.5f}), '
            f'n_uavs={len(viz_state["plans_by_sysid"])}'
        )

    def _on_strike_viz_telemetry(self, callsign: str, frame):
        """FleetRegistry.telemetry_updated 訂閱 — 過濾 strike 參與機後送往 3D。"""
        state = getattr(self, '_strike_viz_state', None)
        if state is None:
            return
        sysid = int(getattr(frame, 'sysid', 0) or 0)
        plan_info = state['plans_by_sysid'].get(sysid)
        if plan_info is None:
            return  # 非 strike 參與機

        cesium = self._get_cesium_widget()
        if cesium is None:
            return

        # 第一筆 telemetry → 標記實際 t0
        if state['started_at'] is None:
            import time as _t
            state['started_at'] = _t.monotonic()

        # 推 JS：階段判定 + 軌跡 + label
        cs = state['sysid_to_callsign'].get(sysid, f'UAV-{sysid}')
        cesium.strike_viz_update_uav(
            sysid=sysid,
            lat=float(frame.lat),
            lon=float(frame.lon),
            alt=float(frame.alt_rel),
            heading_deg=float(frame.heading),
            callsign=cs,
            planned_eta_s=plan_info['eta_s'],
            loiter_lat=plan_info['loiter_lat'],
            loiter_lon=plan_info['loiter_lon'],
            loiter_radius_m=plan_info['loiter_radius_m'],
        )

        # IMPACT 偵測（與 JS 端閾值對齊：d<60m AND alt<30m）
        import math as _m
        try:
            dx = (float(frame.lat) - state['target_lat'])
            dy = (float(frame.lon) - state['target_lon'])
            dist_m = _m.sqrt(
                (dx * 111320.0) ** 2
                + (dy * 111320.0 * _m.cos(_m.radians(state['target_lat']))) ** 2
            )
        except Exception:
            dist_m = 1e9
        if dist_m < 100.0 and float(frame.alt_rel) < 40.0:
            state['impacted'].add(sysid)

        # 全機 IMPACT → 立刻收尾
        if state['impacted'] >= set(state['plans_by_sysid'].keys()):
            self._finalize_strike_visualization()

    def _finalize_strike_visualization(self):
        """拆除 telemetry 訂閱 + 取 BDA 統計 + 開對話框。"""
        state = getattr(self, '_strike_viz_state', None)
        if state is None:
            return

        # 斷開 telemetry 訂閱（避免事件累積）
        try:
            from mission.fleet_registry import FleetRegistry
            FleetRegistry.instance().telemetry_updated.disconnect(
                self._on_strike_viz_telemetry
            )
        except Exception:
            pass

        # 停超時計時器
        t = state.get('timeout_timer')
        if t is not None:
            try: t.stop()
            except Exception: pass

        cesium = self._get_cesium_widget()
        if cesium is None or not hasattr(cesium, 'strike_viz_end'):
            self._strike_viz_state = None
            return

        def _on_stats(stats: dict):
            try:
                from ui.dialogs.strike_bda_dialog import StrikeBDADialog
                dlg = StrikeBDADialog(stats, parent=self)
                dlg.show()
            except Exception as e:
                logger.error(f'[Strike VIZ] BDA dialog 開啟失敗: {e}', exc_info=True)

        cesium.strike_viz_end(callback=_on_stats)
        self._strike_viz_state = None
        logger.info('[Strike VIZ] 結束 — BDA 統計已請求')

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
                   f'{launch_mode} 發射 / {timing_mode} 時間協同')
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
        # 清除菱形 STOT 快取
        self._diamond_plans = None
        self._diamond_target = None
        self._diamond_cfg = None
        # 隱藏底部 TTT Dashboard
        try:
            dash = getattr(self, 'strike_ttt_dashboard', None)
            if dash is not None:
                dash.clear()
                dash.setVisible(False)
        except Exception:
            pass
        self.statusBar().showMessage('已清除打擊視覺化', 3000)

    # ─────────────────────────────────────────────────────────────────
    #  蜂群打擊 — 匯出 QGC WPL 航點（比照 DCCPP 匯出架構）
    # ─────────────────────────────────────────────────────────────────
    def _on_strike_export(self):
        """匯出當前蜂群打擊路徑為 QGC WPL 110 航點檔案。"""
        # ── 新流程：菱形編隊 STOT 飽和打擊 ─────────────────────
        if getattr(self, '_diamond_plans', None):
            from pathlib import Path
            from datetime import datetime
            from core.strike.diamond_swarm_planner import DiamondSwarmStrikePlanner
            default_dir = Path('data/exports') / f'diamond_strike_{datetime.now():%Y%m%d_%H%M%S}'
            chosen = QFileDialog.getExistingDirectory(
                self, '選擇匯出資料夾（4 個 .waypoints + 簡報 .txt）',
                str(default_dir.parent),
            )
            if not chosen:
                return
            out_dir = Path(chosen) / default_dir.name
            files = DiamondSwarmStrikePlanner.export_qgc_wpl(
                self._diamond_plans, out_dir, prefix='diamond_strike_uav'
            )
            QMessageBox.information(
                self, '✅ 菱形編隊打擊匯出完成',
                f'匯出資料夾：{out_dir}\n\n共 {len(files)} 個檔案：\n' +
                '\n'.join(f'  • {f.name}' for f in files)
            )
            self.statusBar().showMessage(
                f'🛡️ 菱形 STOT 任務已匯出 ({len(files)} 個檔案)', 6000
            )
            return

        # ── 舊流程（多目標固定翼 / VTOL） ──────────────────────
        # （下方原有邏輯保留供其他模式使用）
        # 固定翼模式：
        #     DO_SET_HOME (179) → DO_CHANGE_SPEED (178) → NAV_TAKEOFF (22)
        #     → NAV_WAYPOINT (16, 巡航) × N → NAV_WAYPOINT (16, 俯衝) × M
        # VTOL 模式（VTOLSwarmStrikePlanner 自帶 export_qgc_wpl）：
        #     DO_SET_HOME → NAV_VTOL_TAKEOFF (84) → DO_VTOL_TRANSITION (3000)
        #     → DO_CHANGE_SPEED (Phase 2) → NAV_WAYPOINT (2km 邊界)
        #     → DO_CHANGE_SPEED (Phase 3) → IMAGE_START_CAPTURE → NAV_WAYPOINT (IMPACT)
        import os
        # 不再 re-import QFileDialog/QMessageBox（會把模組層級匯入遮蔽成 local），
        # 已於檔案頂端 from PyQt6.QtWidgets import QMessageBox, QFileDialog
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

        分流：
          - 若有快取的 _diamond_plans → 上傳菱形 4 機 STOT 任務
          - 否則 → 沿用舊版 _strike_result 流程
        """
        # ── 前置檢查 ───────────────────────────────────────────────
        if not self._sitl_links:
            QMessageBox.information(
                self, '蜂群打擊 → SITL',
                '尚未連線任何 SITL 實例。\n'
                '請先切換到「🛰 SITL」分頁，啟動 N 台 ArduPlane。'
            )
            return

        # ── 新流程：菱形 STOT 任務 ────────────────────────────
        if getattr(self, '_diamond_plans', None):
            return self._on_diamond_strike_sitl_upload()

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
        timing_desc = ('STOT 同秒命中' if timing_mode == 'STOT'
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
                # SITLLink 沒有 last_frame；即時遙測經 telemetry signal 存進
                # FleetRegistry，依註冊 callsign（UAV-<sysid_label>）反查最新 frame。
                # 注意：欄位用真實 TelemetryFrame 名稱（alt_rel/heading/ground_speed），
                # 而非 Mock 的 alt/heading_deg/groundspeed_mps。
                sysid = int(getattr(lk, 'sysid_label', 0) or 0)
                frame = None
                if sysid:
                    from mission.fleet_registry import FleetRegistry
                    frame = FleetRegistry.instance().latest(f'UAV-{sysid}')
                if frame is not None and frame.is_valid_gps():
                    live_pos = (frame.lat, frame.lon, frame.alt_rel,
                                frame.heading, frame.ground_speed)

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
            conflict_txt += '\n\nIAPF 避障調整:\n' + '\n'.join(
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

