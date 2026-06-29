"""mission/param_service.py — ArduPilot 參數服務。

包裝 PARAM_REQUEST_LIST / PARAM_VALUE / PARAM_SET，
透過 `FleetRegistry` 取對應 UAV 的 `SITLLink`，**不開第二條 MAVLink**。

緩存：模組層 `_param_cache: dict[sysid, (timestamp, params)]` TTL 5min。
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

from PyQt6.QtCore import QObject, pyqtSignal

from mission.fleet_registry import FleetRegistry

logger = logging.getLogger(__name__)


_CACHE_TTL_SEC: float = 5 * 60


@dataclass
class ParamRecord:
    """單筆參數紀錄。"""
    name: str
    value: float
    type_code: int = 9      # MAV_PARAM_TYPE_REAL32
    index: int = -1


@dataclass
class _ParamSet:
    """一台 UAV 的全參數表 + 時間戳。"""
    timestamp: float = 0.0
    expected_count: int = 0
    records: Dict[str, ParamRecord] = field(default_factory=dict)


class ParamService(QObject):
    """全機隊共用的參數服務單例。"""

    # ── Signals ──────────────────────────────────────────────
    param_list_progress = pyqtSignal(int, int, int)   # sysid, total, received
    param_list_complete = pyqtSignal(int, dict)       # sysid, dict[name, ParamRecord]
    param_set_ack       = pyqtSignal(int, str, bool)  # sysid, name, accepted
    param_error         = pyqtSignal(int, str)        # sysid, message

    _instance: Optional["ParamService"] = None

    def __init__(self, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._sets: Dict[int, _ParamSet] = {}
        self._connected_sysids: set[int] = set()

    @classmethod
    def instance(cls) -> "ParamService":
        if cls._instance is None:
            cls._instance = ParamService()
        return cls._instance

    # ── 公開介面 ──────────────────────────────────────────────
    def request_all(self, sysid: int) -> bool:
        """送 PARAM_REQUEST_LIST；回傳是否成功送出。"""
        link = FleetRegistry.instance().get_link_by_sysid(sysid)
        if link is None:
            logger.warning("ParamService: no link for sysid=%d", sysid)
            return False
        # 接 SITLLink 的 param_value 訊號（每 sysid 只接一次）
        # 注意：原始 SITLLink 沒有 param_value signal，本服務便無法逐 PARAM_VALUE
        # 更新；於缺席時降級為「只發送 PARAM_REQUEST_LIST，不主動 watch 回應」，
        # 避免因 AttributeError 阻擋連線流程。
        if sysid not in self._connected_sysids:
            if hasattr(link, "param_value"):
                link.param_value.connect(
                    lambda name, value, type_code, pidx, pcount, _sys=sysid:
                    self._on_param_value(_sys, name, value, type_code, pidx, pcount)
                )
            else:
                logger.info(
                    "ParamService: sysid=%d SITLLink has no param_value signal; "
                    "live param updates disabled", sysid,
                )
            self._connected_sysids.add(sysid)
        # 重置紀錄
        self._sets[sysid] = _ParamSet(timestamp=time.monotonic())
        # 透過 SITLLink._mav 發送
        try:
            mav = getattr(link, "_mav", None)
            if mav is None:
                self.param_error.emit(sysid, "MAVLink not connected")
                return False
            mav.mav.param_request_list_send(mav.target_system, mav.target_component)
            return True
        except Exception as exc:
            self.param_error.emit(sysid, f"PARAM_REQUEST_LIST failed: {exc}")
            return False

    def set_param(
        self,
        sysid: int,
        name: str,
        value: float,
        type_code: int = 9,
    ) -> bool:
        """發 PARAM_SET。沿用 SITLLink 既有 `_send_param_set` 內部封裝。"""
        link = FleetRegistry.instance().get_link_by_sysid(sysid)
        if link is None:
            self.param_error.emit(sysid, "no link")
            return False
        try:
            mav = getattr(link, "_mav", None)
            if mav is None:
                self.param_error.emit(sysid, "MAVLink not connected")
                return False
            pid = name.encode("utf-8")[:16]
            mav.mav.param_set_send(
                mav.target_system, mav.target_component,
                pid, float(value), type_code,
            )
            # 樂觀標記，待下一筆 PARAM_VALUE 確認
            return True
        except Exception as exc:
            self.param_error.emit(sysid, f"PARAM_SET failed: {exc}")
            return False

    def cached(self, sysid: int) -> Optional[Dict[str, ParamRecord]]:
        """取快取（TTL 內）；過期或無資料回 None。"""
        ps = self._sets.get(sysid)
        if ps is None:
            return None
        if time.monotonic() - ps.timestamp > _CACHE_TTL_SEC:
            return None
        return ps.records

    # ── 內部：處理進來的 PARAM_VALUE ──────────────────────────
    def _on_param_value(
        self,
        sysid: int,
        name: str,
        value: float,
        type_code: int,
        param_index: int,
        param_count: int,
    ) -> None:
        ps = self._sets.setdefault(sysid, _ParamSet(timestamp=time.monotonic()))
        old = ps.records.get(name)
        ps.records[name] = ParamRecord(name=name, value=value, type_code=type_code, index=param_index)
        if param_count > 0:
            ps.expected_count = param_count
        # 進度
        if param_count > 0:
            self.param_list_progress.emit(sysid, param_count, len(ps.records))
        # 完成偵測
        if param_count > 0 and len(ps.records) >= param_count:
            ps.timestamp = time.monotonic()
            self.param_list_complete.emit(sysid, dict(ps.records))
        # 寫入回 ACK：若 value 不同於先前 → 視為 set 成功
        if old is not None and old.value != value:
            self.param_set_ack.emit(sysid, name, True)


__all__ = ["ParamService", "ParamRecord"]
