"""ui/widgets/sitl_messages.py — SITL 頁的飛控／地面站訊息緩衝（純 Python，不碰 Qt）。

2026-10-22 使用者：「servo output 跟 message 應該是我選哪一台飛機就列出哪一台的」⇒ SITL 頁分「群控／單機」
兩種模式。訊息要能：
  * 群控：全部 UAV 合併成一條時間線（每行標 UAVn）；
  * 單機：只列選定那一台的（加上不屬於任何一台的本機訊息，例如「已連線」）；切換對象時**立刻**列出那台的歷史。

所以每則訊息帶 sysid 進來、在這裡分桶保存，畫面要哪一份就從這裡重畫。

上限（每一桶都是 deque，滿了丟最舊的）：
  合併 500 ／ 每台 300 ／ 本機 100；PARAM_VALUE 回報另外每台只留 50 —— 一次 PARAM_REQUEST_LIST
  會灌進約 900 則 ``PARAM X = v``，放同一桶會把飛控訊息全部擠掉，所以分開放、讀的時候依序號合併。
斷線後該台的歷史**保留**（有上限），重連選回來還看得到斷線前的訊息。

兩種「不是某一台自己的訊息」（2026-10-22 審查後補）：
  * link ＝某一台**斷線**：存進那台的歷史，**也**存進全隊警示桶 —— 單機看任何一台都列出來（標 UAVn），
    不然單機看 UAV1 時 UAV2 在空中斷線，畫面上只是一顆 chip 不見了。
  * ext ＝來源不是「已確認的飛機的自駕儀」（MAVROS／伴隨電腦／相機元件／別的地面站 255／來源不明 0）：
    只進合併時間線（群控看得到、標 [EXT n]），**不**進任何一台的單機訊息 —— 以前被當成「本機訊息」，
    在每一台的單機畫面裡都出現，看起來像是選定那台說的。
"""
from __future__ import annotations

import heapq
import itertools
import time
from collections import deque
from dataclasses import dataclass
from operator import attrgetter
from typing import Dict, List, Optional

#: 訊息種類：fc＝飛控 STATUSTEXT／gcs＝鏈路自己產生的／param＝PARAM_VALUE 回報／log＝日誌下載／
#: strip＝航測條帶監測／local＝地面站本機（連線、啟動、參數預設…，不屬於任何一台）／
#: link＝某一台斷線（全隊警示）／ext＝非飛機自駕儀來源的 STATUSTEXT（見模組說明）
KINDS = ('fc', 'gcs', 'param', 'log', 'strip', 'local', 'link', 'ext')

_SEQ_KEY = attrgetter('seq')


@dataclass(frozen=True)
class HudMsg:
    seq: int                    # 全域遞增序號（排序與合併用）
    t: float                    # 主機牆鐘時間（time.time()）
    sysid: Optional[int]        # None ＝ 不屬於任何一台（本機訊息／來源 0、255）
    severity: Optional[int]     # MAVLink severity（0=EMERG … 7=DEBUG）；本機訊息為 None
    text: str
    kind: str
    color: Optional[str] = None  # 本機訊息沿用 lbl_status.setStyleSheet 設的顏色


class SitlMessageLog:
    MERGED_MAX = 500
    PER_UAV_MAX = 300
    LOCAL_MAX = 100
    PARAM_MAX = 50

    def __init__(self) -> None:
        self._seq = itertools.count(1)
        self._merged: deque = deque(maxlen=self.MERGED_MAX)        # 非 param（含本機）
        self._merged_param: deque = deque(maxlen=self.PARAM_MAX)   # 全部 sysid 的 param 回報
        self._per: Dict[int, deque] = {}                           # sysid → 非 param
        self._per_param: Dict[int, deque] = {}                     # sysid → param 回報
        self._local: deque = deque(maxlen=self.LOCAL_MAX)          # sysid None 的非 param
        self._fleet: deque = deque(maxlen=self.LOCAL_MAX)          # link（斷線）：全隊警示

    @staticmethod
    def _norm_sysid(sysid) -> Optional[int]:
        try:
            s = int(sysid)
        except (TypeError, ValueError):
            return None
        return s if s > 0 else None

    def add(self, sysid, severity, text, kind, color=None, t=None) -> HudMsg:
        """存一則；sysid ≤ 0 或 None ⇒ 當成不屬於任何一台（None）。回傳存進去的那一則。"""
        sid = self._norm_sysid(sysid)
        kind = kind if kind in KINDS else 'gcs'
        sev = None if severity is None else int(severity)
        e = HudMsg(next(self._seq), time.time() if t is None else float(t), sid, sev,
                   str(text), kind, color)
        if kind == 'param':
            self._merged_param.append(e)
            if sid is not None:
                self._per_param.setdefault(sid, deque(maxlen=self.PARAM_MAX)).append(e)
            return e
        self._merged.append(e)
        if kind == 'ext':                     # 只進合併時間線（不屬於任何一台的單機訊息）
            return e
        if sid is None:
            self._local.append(e)
        else:
            self._per.setdefault(sid, deque(maxlen=self.PER_UAV_MAX)).append(e)
            if kind == 'link':
                self._fleet.append(e)
        return e

    def merged(self) -> List[HudMsg]:
        """全部（含本機訊息、param 回報、ext），依序號。"""
        return list(heapq.merge(self._merged, self._merged_param, key=_SEQ_KEY))

    def for_uav(self, sysid) -> List[HudMsg]:
        """這一台（fc／gcs／log／strip／link ＋ 它的 param 回報）＋ 本機訊息 ＋ **別台的斷線警示**（link）；
        其他 sysid 的一般訊息、ext 一律不含。sysid None ⇒ 本機訊息 ＋ 全部斷線警示。"""
        sid = self._norm_sysid(sysid)
        if sid is None:
            return list(heapq.merge(self._local, self._fleet, key=_SEQ_KEY))
        others = [e for e in self._fleet if e.sysid != sid]   # 自己的斷線已在 _per[sid] 裡（不重複）
        return list(heapq.merge(self._per.get(sid, ()), self._per_param.get(sid, ()), self._local, others,
                                key=_SEQ_KEY))

    def sysids(self) -> List[int]:
        return sorted(set(self._per) | set(self._per_param))

    def counts(self) -> dict:
        """各桶目前筆數（測試／除錯用）。"""
        return {'merged': len(self._merged), 'merged_param': len(self._merged_param),
                'local': len(self._local), 'fleet': len(self._fleet),
                'per': {s: len(q) for s, q in self._per.items()},
                'per_param': {s: len(q) for s, q in self._per_param.items()}}


__all__ = ['HudMsg', 'SitlMessageLog', 'KINDS']
