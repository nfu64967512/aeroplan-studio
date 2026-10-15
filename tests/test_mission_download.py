# -*- coding: utf-8 -*-
"""AeroPlan 從飛控讀回任務航線 —— 值層級鎖。

═══ 這個檔在防哪三件已經發生過的事 ═══

① **送得出去、收不回來**（[[ParamService]] 那一類）
   `mission/param_service.py:66-99` 送 PARAM_REQUEST_LIST，但 `SITLLink`
   從來沒有 `param_value` signal ⇒ 回覆全部丟掉，`ParametersBrowser` 接在
   一個結構上不可能完成的服務上，唯一症狀是 line 82-85 的一行 INFO。
   ⇒ `TestEveryLinkSignalMainWindowConnectsExists` 用**交叉比對**擋這一類：
      把 main_window 裡每個 `link.X.connect(` 的 X 抓出來，逐一確認
      `SITLLink` 真的有那個 signal。這是唯一擋得住「未來又漏一個」的形狀。

② **三元組讓整張 2D 圖靜靜消失**
   `map_widget.py:253-292` 把 path 原封不動交給 folium；三元組會丟
   ValueError，而 `_render_map()` 用一個 try/except 包住整個 `_build_map()`
   （:542-584）⇒ 例外在 `setUrl()` 之前中止，角點/多邊形/NFZ/圍籬/home
   **全部一起消失**，WebView 停在上一頁，只留一行 log。
   ⇒ 這裡不只斷言「長度是 2」，而是**真的餵給 folium**。

③ **失敗畫成空航線**
   空清單是合法答案（飛控裡真的沒有任務，`auto_mission=False` 時的常態）。
   把失敗也畫成空的，使用者會去查上傳側 —— 查錯地方。
   ⇒ `TestFailureIsNotAnEmptyRoute`。
"""
from __future__ import annotations

import ast
import pathlib
import re

import pytest
from PyQt6.QtCore import QCoreApplication

from mission import mission_view as mv
from mission.sitl_link import SITLLink

_app = QCoreApplication.instance() or QCoreApplication([])

ROOT = pathlib.Path(__file__).resolve().parents[1]


def _calls_in(path: pathlib.Path, func_name: str) -> set:
    """`path` 裡名為 `func_name` 的函式**呼叫了**哪些東西（名稱集合）。

    回傳 None 代表找不到那個函式。

    ⚠️ 存在的理由：字串比對法（`'x' not in 原始碼片段`）會被註解**和
       docstring** 觸發誤判。本 repo 已經記錄過五次，而通用修法
       `tests/test_auto_mission.py::_code_only` 只剝得掉 `#` 註解 ——
       剝不掉 docstring。AST 兩者都免疫，而且問的是真正要鎖的性質。
    """
    tree = ast.parse(path.read_text(encoding='utf-8'))
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) \
                and node.name == func_name:
            out = set()
            for sub in ast.walk(node):
                if isinstance(sub, ast.Call):
                    f = sub.func
                    if isinstance(f, ast.Attribute):
                        out.add(f.attr)
                    elif isinstance(f, ast.Name):
                        out.add(f.id)
            return out
    return None

NAV_WAYPOINT = 16
NAV_LOITER_UNLIM = 17
NAV_TAKEOFF = 22
DO_CHANGE_SPEED = 178
DO_JUMP = 177            # ⚠️ 不在 mission.waypoint.MAVCommand 裡
DO_SET_MODE = 176        # ⚠️ 同上


# ══════════════════════════════════════════════════════════════════
# 假飛控：實作 mission 協定的伺服端
# ══════════════════════════════════════════════════════════════════
class _Msg:
    def __init__(self, mtype, **fields):
        self._mtype = mtype
        for k, v in fields.items():
            setattr(self, k, v)

    def get_type(self):
        return self._mtype

    def get_srcSystem(self):
        return 1


class _FakeMavApi:
    def __init__(self, owner):
        self._o = owner

    def mission_request_list_send(self, ts, tc, mission_type=0):
        self._o.sent.append(('request_list', mission_type))
        if self._o.count_reply is not False:
            n = (self._o.count_reply
                 if self._o.count_reply is not None else len(self._o.items))
            self._o.inbox.append(_Msg('MISSION_COUNT', count=n))

    def mission_request_int_send(self, ts, tc, seq, mission_type=0):
        self._o.sent.append(('request_int', seq))
        self._o.serve_item(seq)

    def mission_ack_send(self, ts, tc, type, mission_type=0):
        self._o.sent.append(('ack', type))

    def heartbeat_send(self, *a, **kw):
        self._o.sent.append(('heartbeat',))


class FakeMav:
    """假 pymavlink 連線：對 mission 讀取協定做出反應。

    ⚠️ 這個替身**自己也要被驗證**。`test_the_fake_can_actually_misbehave`
       確認每一種故障模式真的產生得出來 —— 否則「測到了」只是「什麼都沒發生」，
       而那正好長得像通過（見 [[instrument-that-cannot-fire]]）。
    """

    def __init__(self, items, *, count_reply=None, drop_seqs=(),
                 wrong_seq_for=None, item_type='MISSION_ITEM_INT',
                 stale_inbox=()):
        self.target_system = 1
        self.target_component = 1
        self.items = list(items)
        self.count_reply = count_reply      # None=真值, False=不回, int=謊報
        self.drop_seqs = set(drop_seqs)     # 這些 seq 完全不回
        self.wrong_seq_for = wrong_seq_for  # {被問的seq: 回傳的seq}
        self.item_type = item_type
        # 模擬「上一筆已經放棄的交易」留在接收緩衝裡、還沒被讀走的回覆
        # （例如上一次讀取逾時離開時，飛控的回覆其實還在路上）。
        self.inbox = list(stale_inbox)
        self.sent = []
        self.mav = _FakeMavApi(self)

    def serve_item(self, seq):
        if seq in self.drop_seqs:
            return
        reply_seq = (self.wrong_seq_for or {}).get(seq, seq)
        if reply_seq >= len(self.items):
            return
        it = self.items[reply_seq]
        if self.item_type == 'MISSION_ITEM_INT':
            x, y = int(it['lat'] * 1e7), int(it['lon'] * 1e7)
        else:
            x, y = float(it['lat']), float(it['lon'])
        self.inbox.append(_Msg(
            self.item_type, seq=reply_seq, command=it['command'],
            frame=it.get('frame', 3), x=x, y=y, z=float(it.get('alt', 0.0)),
            param1=float(it.get('param1', 0.0)),
            param2=float(it.get('param2', 0.0)),
            param3=float(it.get('param3', 0.0)),
            param4=float(it.get('param4', 0.0)),
            current=0, autocontinue=1))

    def recv_match(self, type=None, blocking=False, timeout=None):
        want = [type] if isinstance(type, str) else (type or [])
        for i, m in enumerate(self.inbox):
            if not want or m.get_type() in want:
                return self.inbox.pop(i)
        return None


def mk_link(fake):
    link = SITLLink('udpin:0.0.0.0:1', sysid_label=7)
    link._mav = fake
    return link


def capture(link):
    got, failed = [], []
    link.mission_downloaded.connect(lambda s, i: got.append((s, i)))
    link.mission_download_failed.connect(lambda s, r: failed.append((s, r)))
    return got, failed


STRIKE = [
    # seq 0 = home（ArduPilot 一定會用自己的 home 蓋掉這一項）
    dict(seq=0, command=NAV_WAYPOINT, lat=25.0777, lon=121.2325, alt=32.1),
    dict(seq=1, command=NAV_WAYPOINT, lat=25.0800, lon=121.2000, alt=3100.0),
    dict(seq=2, command=NAV_LOITER_UNLIM, lat=25.0777, lon=121.1705,
         alt=3100.0, param3=3300.0),
]


# ══════════════════════════════════════════════════════════════════
class TestTheFakeCanActuallyMisbehave:
    """⚠️⚠️ 先證明儀器打得出每一種故障。

    「守衛沒咬」和「故障根本沒發生」在報表上長得一模一樣，而後者會被讀成
    前者的證據。每一種故障模式在這裡都要先確認**真的能發生**。
    """

    def test_normal_path_serves_everything(self):
        f = FakeMav(STRIKE)
        f.mav.mission_request_list_send(1, 1)
        assert f.inbox and f.inbox[0].count == 3
        f.inbox.clear()
        f.mav.mission_request_int_send(1, 1, 1)
        assert len(f.inbox) == 1 and f.inbox[0].seq == 1

    def test_count_reply_false_really_stays_silent(self):
        f = FakeMav(STRIKE, count_reply=False)
        f.mav.mission_request_list_send(1, 1)
        assert f.inbox == [], '故障模式沒生效 ⇒ 對應的測試證明不了任何事'

    def test_drop_seqs_really_drops(self):
        f = FakeMav(STRIKE, drop_seqs={2})
        f.mav.mission_request_int_send(1, 1, 2)
        assert f.inbox == []

    def test_wrong_seq_really_answers_wrong(self):
        f = FakeMav(STRIKE, wrong_seq_for={1: 2})
        f.mav.mission_request_int_send(1, 1, 1)
        assert f.inbox[0].seq == 2, '亂序故障沒生效'


class TestDownloadHappyPath:
    def test_emits_every_item(self):
        link = mk_link(FakeMav(STRIKE))
        got, failed = capture(link)
        link._recv_mission()
        assert not failed
        assert len(got) == 1
        sysid, items = got[0]
        assert sysid == 7
        assert [i['seq'] for i in items] == [0, 1, 2]

    def test_lat_lon_survive_the_1e7_round_trip(self):
        link = mk_link(FakeMav(STRIKE))
        got, _ = capture(link)
        link._recv_mission()
        loiter = got[0][1][2]
        assert loiter['lat'] == pytest.approx(25.0777, abs=1e-7)
        assert loiter['lon'] == pytest.approx(121.1705, abs=1e-7)
        assert loiter['param3'] == pytest.approx(3300.0)

    def test_plain_mission_item_float_degrees_also_work(self):
        """舊飛控回 MISSION_ITEM（float 度）而不是 _INT（int×1e7）。
        兩者用同一個除以 1e7 會把座標打到外海，而且不會報錯。"""
        link = mk_link(FakeMav(STRIKE, item_type='MISSION_ITEM'))
        got, _ = capture(link)
        link._recv_mission()
        assert got[0][1][2]['lat'] == pytest.approx(25.0777, abs=1e-7)

    def test_acks_at_the_end(self):
        f = FakeMav(STRIKE)
        link = mk_link(f)
        capture(link)
        link._recv_mission()
        assert ('ack', 0) in f.sent, '沒送收尾 ACK ⇒ 部分飛控會卡在傳輸狀態'

    def test_unknown_command_does_not_explode(self):
        """⚠️⚠️ 這是本檔最重要的一條。

        `mission.waypoint.Waypoint.__post_init__` 會做 `MAVCommand(command)`，
        而那個列舉**沒有** DO_JUMP(177) / DO_SET_MODE(176) ⇒ 只要下載端用
        Waypoint 當資料模型，飛控回一筆這種指令就會整趟消失在 ValueError 裡。
        """
        odd = [dict(seq=0, command=NAV_WAYPOINT, lat=25.0, lon=121.0, alt=0.0),
               dict(seq=1, command=DO_JUMP, lat=0.0, lon=0.0, alt=0.0, param1=3),
               dict(seq=2, command=DO_SET_MODE, lat=0.0, lon=0.0, alt=0.0)]
        link = mk_link(FakeMav(odd))
        got, failed = capture(link)
        link._recv_mission()
        assert not failed
        assert [i['command'] for i in got[0][1]] == [16, 177, 176]

    def test_the_enum_really_would_have_rejected_it(self):
        """證明上一條擋的是真的東西，不是想像的東西。"""
        from mission.waypoint import Waypoint
        with pytest.raises(ValueError):
            Waypoint(seq=1, command=DO_JUMP)


class TestFailureIsNotAnEmptyRoute:
    def test_count_timeout_fails_loudly(self):
        link = mk_link(FakeMav(STRIKE, count_reply=False))
        got, failed = capture(link)
        link._recv_mission()
        assert got == [], '失敗**不可以**走成功信號 —— 空航線是合法答案'
        assert len(failed) == 1 and failed[0][0] == 7
        assert 'MISSION_COUNT' in failed[0][1]

    def test_missing_item_fails_loudly(self):
        link = mk_link(FakeMav(STRIKE, drop_seqs={1}))
        got, failed = capture(link)
        link._recv_mission()
        assert got == []
        assert len(failed) == 1 and 'seq 1' in failed[0][1]

    def test_zero_items_is_success_not_failure(self):
        """auto_mission=False 時飛控真的是空的。那**不是**錯誤。"""
        link = mk_link(FakeMav([], count_reply=0))
        got, failed = capture(link)
        link._recv_mission()
        assert failed == []
        assert got == [(7, [])]

    def test_out_of_order_reply_is_rejected(self):
        """⚠️ 「收到就算數」會產出一張畫得出來、但**錯位**的地圖 ——
        那比畫不出來更糟，因為不會有人去查。"""
        link = mk_link(FakeMav(STRIKE, wrong_seq_for={1: 2}))
        got, failed = capture(link)
        link._recv_mission()
        assert got == []
        assert 'seq 1' in failed[0][1]


class TestStaleTransactionDoesNotLeakIntoANewRead:
    """⚠️⚠️ _mission_msg_is_for_us() 只濾「哪一架飛控」，不濾「哪一筆交易」。

    上一次已經放棄的讀取（使用者按「讀取」又逾時離開、或上一次讀回驗證被
    中斷）留下的 MISSION_COUNT / MISSION_ITEM_INT，跟這次全新請求的回覆長得
    一模一樣 —— 沒有交易代號/nonce 可以分辨。這裡證明：這種過期回覆不會被
    當成這次讀取的答案（會被當成「目前」的任務顯示成功、零錯誤文字，正是
    `_recv_mission` 自己的文件開頭第 ③ 條要擋的「失敗畫成空航線」的相反面
    ——「過期畫成當前」）。
    """

    def test_stale_count_and_item_are_not_taken_for_the_new_read(self):
        stale = [_Msg('MISSION_COUNT', count=1),
                 _Msg('MISSION_ITEM_INT', seq=0, command=16, frame=3,
                      x=int(1.0 * 1e7), y=int(2.0 * 1e7), z=3.0,
                      param1=0.0, param2=0.0, param3=0.0, param4=0.0,
                      current=1, autocontinue=1)]
        link = mk_link(FakeMav(STRIKE, stale_inbox=stale))
        got, failed = capture(link)
        link._recv_mission()
        assert failed == [], failed
        assert len(got) == 1
        sysid, items = got[0]
        # 真正的（新）任務有 3 項；如果吃到舊資料只會有 1 項、且座標會是
        # (1.0, 2.0) 而不是 STRIKE 裡的座標。
        assert [i['seq'] for i in items] == [0, 1, 2]
        assert items[0]['lat'] != pytest.approx(1.0)

    def test_stale_messages_are_drained_before_the_new_request_is_sent(self):
        """清乾淨之後才送 mission_request_list_send —— 不會把清出來的舊
        MISSION_COUNT 誤算進『收到幾次』或讓舊資料混進統計。"""
        stale = [_Msg('MISSION_COUNT', count=99)]
        f = FakeMav(STRIKE, stale_inbox=stale)
        link = mk_link(f)
        capture(link)
        link._recv_mission()
        # 清空後才真正 request_list；真正收到的 MISSION_COUNT 是 3（STRIKE 的長度）。
        assert f.sent[0] == ('request_list', 0)


class TestPublicApiStaysOffTheGuiThread:
    def test_download_mission_only_enqueues(self):
        """⚠️ `param_service.py:91-95` 從 GUI 執行緒直接戳 `link._mav`，
        和接收執行緒搶同一個 pymavlink 物件。這條鎖住不要再犯。"""
        f = FakeMav(STRIKE)
        link = mk_link(f)
        link.download_mission()
        assert f.sent == [], '公開方法直接送了 MAVLink ⇒ 跨執行緒競爭'
        assert link._cmd_queue.get_nowait() == ('download_mission', None)

    def test_dispatch_routes_it(self):
        f = FakeMav(STRIKE)
        link = mk_link(f)
        got, _ = capture(link)
        link._dispatch_command('download_mission', None)
        assert len(got) == 1 and len(got[0][1]) == 3


class TestEveryLinkSignalMainWindowConnectsExists:
    """⭐ 交叉比對：main_window 接的每個 link signal 都要真的存在。

    這是唯一擋得住「ParamService 那一類」的形狀 —— 不是鎖住某一行長什麼樣，
    而是鎖住「送出端與接收端對得上」這個性質，所以未來新增的 signal
    也自動被涵蓋。
    """

    SRC = (ROOT / 'ui' / 'main_window.py').read_text(encoding='utf-8')

    def test_all_connected_signals_exist(self):
        names = set(re.findall(r'\blink\.(\w+)\.connect\(', self.SRC))
        assert names, '一個都沒抓到 ⇒ 這條測試自己壞了'
        missing = [n for n in sorted(names) if not hasattr(SITLLink, n)]
        assert not missing, (
            'main_window 接了 SITLLink 沒有的 signal：%s\n'
            '症狀會是「送得出去但畫面永遠不動」，而且完全靜默。' % missing)

    @pytest.mark.parametrize('sig', ['mission_downloaded',
                                     'mission_download_failed'])
    def test_download_signals_are_connected(self, sig):
        assert f'link.{sig}.connect(' in self.SRC, (
            f'{sig} 沒有接 ⇒ 下載會完成，但沒有人在聽（ParamService 形狀）')

    def test_signals_carry_a_payload(self):
        """零參數的 signal 會逼呼叫端反查狀態，而反查的來源就是競爭。"""
        f = FakeMav(STRIKE)
        link = mk_link(f)
        seen = []
        link.mission_downloaded.connect(lambda s, i: seen.append((s, i)))
        link._recv_mission()
        assert seen[0][0] == 7 and isinstance(seen[0][1], list)


# ══════════════════════════════════════════════════════════════════
# mission_view：繪製轉接層
# ══════════════════════════════════════════════════════════════════
class TestTwoDimensionalPathIsReallyTwoDimensional:
    def test_every_element_is_a_two_tuple(self):
        path = mv.mission_to_path_2d(STRIKE)
        assert path, '沒有點 ⇒ 下面的斷言全都空轉'
        for p in path:
            assert isinstance(p, tuple) and len(p) == 2

    def test_folium_actually_accepts_it(self):
        """⭐ 不只斷言長度 —— **真的餵給 folium**。

        長度斷言鎖的是我對 folium 的理解；這一條鎖的是 folium 本身。
        兩者不同時，會壞的是使用者的地圖，不是我的理解。
        """
        folium = pytest.importorskip('folium')
        path = mv.mission_to_path_2d(STRIKE)
        folium.PolyLine(locations=path)
        folium.Marker(location=path[0])
        folium.CircleMarker(location=path[-1])

    def test_folium_really_rejects_three_tuples(self):
        """證明上一條在防的是真的東西。這也是整個 bug 的機制本身。"""
        folium = pytest.importorskip('folium')
        with pytest.raises(ValueError):
            folium.PolyLine(locations=mv.mission_to_path_3d(STRIKE))

    def test_three_d_keeps_altitude(self):
        p3 = mv.mission_to_path_3d(STRIKE)
        assert all(len(p) == 3 for p in p3)
        assert p3[-1][2] == pytest.approx(3100.0)


class TestWhatGetsDrawn:
    def test_home_is_skipped(self):
        """seq 0 是 ArduPilot 的 home（它會蓋掉上傳的內容）。接進航線
        會從起飛點拉一條假的長線出去。"""
        assert len(mv.mission_to_path_2d(STRIKE)) == 2
        assert len(mv.mission_to_path_2d(STRIKE, skip_home=False)) == 3

    def test_do_commands_are_not_drawn(self):
        items = STRIKE + [dict(seq=3, command=DO_CHANGE_SPEED,
                               lat=0.0, lon=0.0, alt=0.0, param2=95.0)]
        assert len(mv.mission_to_path_2d(items)) == 2

    def test_zero_coordinates_are_dropped(self):
        """(0,0) 在幾內亞灣外海 —— 畫出去會把整張圖的縮放拉爛。"""
        items = [dict(seq=1, command=NAV_WAYPOINT, lat=0.0, lon=0.0, alt=0.0)]
        assert mv.mission_to_path_2d(items) == []

    def test_unknown_nav_command_is_still_drawn(self):
        """⚠️ 用區間而不是白名單：失效方向要是「多畫一點」，
        不是「少畫一段」—— 少一段仍然畫得出來，不會有人發現。"""
        items = [dict(seq=1, command=31, lat=25.0, lon=121.0, alt=100.0)]
        assert len(mv.mission_to_path_2d(items)) == 1


class TestHomeIsMarkedNotConnected:
    """seq 0 是飛控的 home。MP / QGC 都畫成獨立 H 標記、不接進折線。

    ⚠️ 只「跳過」它是不夠的:AUTO 架構上傳的任務只有 2 項,跳掉 home 之後
       折線只剩 1 個點 —— L.polyline 對單點**什麼都不畫**,地圖上就只剩
       一個圈,而那看起來像「航線沒讀到」。
    """

    def test_home_is_returned_separately(self):
        h = mv.mission_home(STRIKE)
        assert h is not None
        assert h['lat'] == pytest.approx(25.0777)
        assert h['lon'] == pytest.approx(121.2325)

    def test_home_is_not_in_the_route(self):
        assert (25.0777, 121.2325) not in mv.mission_to_path_2d(STRIKE)

    def test_unset_home_is_none_not_origin(self):
        """⚠️⚠️ 2026-08-26 對真 ArduPilot(-I5,尚未定位)實測:
        seq 0 回的是 `lat=0 lon=0 alt=0 frame=0`,**不是**真座標。
        假飛控在那裡放了真值 ⇒ 只靠單元測試看不到這個形狀。
        回 (0,0) 會把整張地圖的縮放拉到幾內亞灣外海。"""
        items = [dict(seq=0, command=NAV_WAYPOINT, lat=0.0, lon=0.0, alt=0.0),
                 dict(seq=1, command=NAV_LOITER_UNLIM, lat=25.0, lon=121.0,
                      alt=3100.0, param3=3300.0)]
        assert mv.mission_home(items) is None

    def test_no_seq_zero_at_all(self):
        assert mv.mission_home([dict(seq=1, command=NAV_WAYPOINT,
                                     lat=25.0, lon=121.0, alt=0.0)]) is None

    def test_cesium_has_a_home_function(self):
        html = (ROOT / 'ui' / 'widgets' / 'cesium_templates' /
                'main.html').read_text(encoding='utf-8')
        assert 'function addMissionHome(' in html
        m = re.search(r'function addMissionHome\(.*?\n\}', html, re.S)
        assert '_missionEntities' in m.group(0)
        assert '_tacticalEntities' not in m.group(0)

    @pytest.mark.parametrize('mod,fn', [
        ('ui/widgets/dual_map_widget.py', 'display_fc_mission'),
        ('ui/widgets/cesium_map_widget.py', 'display_fc_mission'),
        ('ui/widgets/map_widget.py', 'display_fc_mission'),
    ])
    def test_home_is_threaded_through_every_layer(self, mod, fn):
        """⚠️ 少接一層 = home 靜靜不見。三層都要有 `home` 這個參數 ——
        中間任何一層漏掉,上游算對了也畫不出來,而且不會報錯。"""
        src = (ROOT / mod).read_text(encoding='utf-8')
        m = re.search(r'def %s\(([^)]*)\)' % fn, src, re.S)
        assert m, f'{mod} 找不到 {fn}'
        assert 'home' in m.group(1), f'{mod}::{fn} 沒有 home 參數'

    def test_dual_layer_actually_forwards_it(self):
        """⚠️⚠️ **宣告了參數不等於傳下去了。**

        第一版只斷言「`display_fc_mission` 有 `home` 這個參數」,於是
        「收下 home 然後丟掉」完全通過 —— 突變掃描裡這一條是唯一的存活者。
        同一個形狀在 `test_link_port_exclusivity.py::
        test_formation_launcher_actually_routes_it` 已經記過一次:
        宣告 `gp=` 但少了 `--out` 就是純 no-op。

        這裡用 AST 確認**每一個**下游 display_fc_mission 呼叫都真的把
        home 當關鍵字傳出去。
        """
        src = (ROOT / 'ui' / 'widgets' / 'dual_map_widget.py').read_text(
            encoding='utf-8')
        tree = ast.parse(src)
        fn = next((n for n in ast.walk(tree)
                   if isinstance(n, ast.FunctionDef)
                   and n.name == 'display_fc_mission'), None)
        assert fn is not None
        calls = [c for c in ast.walk(fn)
                 if isinstance(c, ast.Call)
                 and isinstance(c.func, ast.Attribute)
                 and c.func.attr == 'display_fc_mission']
        assert len(calls) == 2, f'應該轉發給 2D 與 3D 兩層,實際 {len(calls)} 個'
        for c in calls:
            kw = {k.arg for k in c.keywords}
            assert 'home' in kw, (
                'DualMapWidget 收下了 home 但沒傳給下游 ⇒ 上游算對了也畫不出來,'
                '而且完全不報錯')

    def test_main_window_actually_computes_it(self):
        src = (ROOT / 'ui' / 'main_window.py').read_text(encoding='utf-8')
        assert '_mv.mission_home(items)' in src
        assert 'home=home' in src


class TestLoiterRings:
    def test_finds_the_decision_ring(self):
        rings = mv.mission_loiters(STRIKE)
        assert len(rings) == 1
        assert rings[0]['radius_m'] == pytest.approx(3300.0)
        assert rings[0]['alt'] == pytest.approx(3100.0)

    def test_negative_radius_becomes_positive(self):
        """param3 帶號只表示旋轉方向。負半徑會讓 Cesium 的 ellipse
        **靜靜不畫**（不報錯）。"""
        items = [dict(seq=1, command=NAV_LOITER_UNLIM, lat=25.0, lon=121.0,
                      alt=3100.0, param3=-3300.0)]
        assert mv.mission_loiters(items)[0]['radius_m'] == pytest.approx(3300.0)

    def test_plain_waypoints_are_not_rings(self):
        items = [dict(seq=1, command=NAV_WAYPOINT, lat=25.0, lon=121.0,
                      alt=100.0, param3=999.0)]
        assert mv.mission_loiters(items) == []


class TestSummaryDistinguishesItemsFromDrawablePoints:
    def test_reports_both_numbers(self):
        s = mv.mission_summary(STRIKE)
        assert '3 項' in s and '2 點' in s and '盤旋圈 1' in s

    def test_all_do_commands_says_zero_drawable(self):
        """⚠️ 只報「12 個航點」而地圖上空無一物，使用者會去查地圖。"""
        items = [dict(seq=i, command=DO_CHANGE_SPEED, lat=0.0, lon=0.0,
                      alt=0.0) for i in range(1, 4)]
        s = mv.mission_summary(items)
        assert '3 項' in s and '0 點' in s

    def test_empty_is_stated_plainly(self):
        assert '沒有任務' in mv.mission_summary([])


# ══════════════════════════════════════════════════════════════════
class TestDrawingDoesNotRebuildTheGeofence:
    """⚠️⚠️ 讀回任務是**觀測**，不可以有副作用。

    `DualMapWidget.display_path/display_paths/display_fw_paths` 每一個都跟著
    跑 `_auto_build_fence`（dual_map_widget.py:520-539），而它重建的
    `_last_fence_bundle` **正是「上傳電子圍籬」上傳的那一份**
    （由 `last_fence_bundle` 屬性讀出，main_window.py:6504）。
    看一眼讀回的 AUTO 航線 ⇒ 圍籬被悄悄換成包住 AUTO 航線的矩形 ⇒ 上傳 ⇒
    飛行中觸發 RTL。錯的圍籬不會當場出事。
    """

    DUAL = ROOT / 'ui' / 'widgets' / 'dual_map_widget.py'

    def test_display_fc_mission_exists(self):
        assert _calls_in(self.DUAL, 'display_fc_mission') is not None

    def test_it_does_not_call_auto_build_fence(self):
        """⚠️ 用 AST 問「有沒有**呼叫**」，不是用字串問「有沒有出現」。

        第一版寫成 `'_auto_build_fence' not in 原始碼片段`，結果對
        `display_fc_mission` 自己的 docstring 開火 —— 那段 docstring 正是在
        解釋「為什麼不可以呼叫它」。這是本 repo 記錄在案的第**六**次同型誤判
        （前五次見 `tests/test_auto_mission.py::_code_only`），而前幾次的修法
        `_code_only` 只剝 `#` 註解、**剝不掉 docstring**，所以這次得換工具。
        一道會對正確程式碼開火的守衛，下一個人只會把它刪掉。
        """
        assert '_auto_build_fence' not in _calls_in(self.DUAL,
                                                    'display_fc_mission'), (
            '讀回任務重建了圍籬 ⇒ 「上傳電子圍籬」會上傳包住 AUTO 航線的那個')

    @pytest.mark.parametrize('name', ['display_path', 'display_paths',
                                      'display_fw_paths'])
    def test_the_other_display_methods_still_do(self, name):
        """⚠️ 反向鎖：既有的規劃路徑**必須**維持強制圍籬政策。
        只鎖「新的不做」，會讓「把舊的也一起拿掉」變成通過。"""
        assert '_auto_build_fence' in _calls_in(self.DUAL, name), (
            f'{name} 不再建構圍籬 ⇒ 強制飛安政策被拿掉了')


class TestCesiumOverlayIsSeparateFromTactical:
    """⚠️ 3D 疊加層不可以共用 `_tacticalEntities`。

    右鍵打擊每次重畫都先呼叫 `clearTacticalOverlay()`
    （ui/controllers/strike_controller.py:326）⇒ 共用的話，做一次打擊規劃
    就會把讀回來的航線靜靜洗掉。
    """

    HTML = (ROOT / 'ui' / 'widgets' / 'cesium_templates' / 'main.html'
            ).read_text(encoding='utf-8')

    def test_mission_overlay_has_its_own_array(self):
        assert 'let _missionEntities = []' in self.HTML
        assert 'function clearMissionOverlay()' in self.HTML

    @pytest.mark.parametrize('fn', ['addMissionRoute', 'addMissionLoiter'])
    def test_mission_functions_do_not_touch_tactical(self, fn):
        m = re.search(r'function %s\(.*?\n\}' % fn, self.HTML, re.S)
        assert m, f'找不到 {fn}'
        assert '_tacticalEntities' not in m.group(0)

    def test_tactical_functions_do_not_touch_mission(self):
        """反向：打擊那組也不可以推進 _missionEntities，
        否則 clearMissionOverlay 會反過來洗掉打擊圖層。"""
        for fn in ('addLoiterCircle', 'addDiveLine', 'setDispersionRing'):
            m = re.search(r'function %s\(.*?\n\}' % fn, self.HTML, re.S)
            assert m and '_missionEntities' not in m.group(0)


class TestLoggerCallsArePreFormatted:
    """⚠️⚠️ `utils.logger.Logger` 的簽章是 ``(message: str, exc_info: bool)``
    —— **不吃 %-args**。stdlib 慣用的 ``logger.info("… %s", x)`` 在這裡有
    兩種失效，而且兩種都不像失效：

      * 傳 2 個以上額外參數 → 執行期 TypeError（實測抓到，就在這條路徑上）
      * 傳**剛好 1 個**額外參數 → **不會當掉**：字面的 `%s` 被寫進 log，
        而那個參數被當成 `exc_info` ⇒ 悄悄打開堆疊輸出。第二種完全靜默。

    ⚠️ repo 裡已經有一條同型鎖（`tests/test_click_to_strike.py:744`），
       但它只掃 `ui/controllers/stot_ros2_controller.py` **一個檔**。
       這就是「鎖綁在一個檔、缺陷住在整個 repo」的形狀 —— 2026-08-26 實測
       另有 6 個檔在犯同樣的錯（fleet_registry / param_service /
       alert_engine / parameters_browser 等），那條鎖一個都咬不到。
       這裡先把**本次改動碰到的檔**鎖起來；全 repo 掃除另案處理。
    """

    FILES = [
        ROOT / 'mission' / 'sitl_link.py',
        ROOT / 'mission' / 'mission_view.py',
        ROOT / 'ui' / 'widgets' / 'map_widget.py',
        ROOT / 'ui' / 'widgets' / 'cesium_map_widget.py',
        ROOT / 'ui' / 'widgets' / 'dual_map_widget.py',
        ROOT / 'ui' / 'widgets' / 'drone_detail_panel.py',
    ]

    @staticmethod
    def _bad_calls(path):
        tree = ast.parse(path.read_text(encoding='utf-8'))
        bad = []
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and isinstance(node.func.value, ast.Name)
                    and node.func.value.id == 'logger'
                    and len(node.args) > 1):
                bad.append((node.lineno, node.func.attr, len(node.args)))
        return bad

    @pytest.mark.parametrize('path', FILES, ids=lambda p: p.name)
    def test_no_percent_args(self, path):
        bad = self._bad_calls(path)
        assert not bad, (
            f'{path.name} 的 logger 呼叫傳了多個位置參數：{bad}\n'
            'Logger 只吃一個已格式化的字串 —— 用 f-string。')

    def test_the_check_can_actually_fire(self):
        """⚠️ 證明這個檢查抓得到東西。一個永遠回空清單的檢查
        和一個「全部通過」的檢查在報表上一模一樣
        （見 [[instrument-that-cannot-fire]]）。"""
        import tempfile
        with tempfile.NamedTemporaryFile('w', suffix='.py', delete=False,
                                         encoding='utf-8') as fh:
            fh.write('logger.info("x %s", y)\n')
            tmp = pathlib.Path(fh.name)
        try:
            assert self._bad_calls(tmp) == [(1, 'info', 2)]
        finally:
            tmp.unlink(missing_ok=True)

    def test_main_window_download_path_only(self):
        """main_window 太大且有既有違規 —— 只鎖本次新增的三個方法。"""
        mw = ROOT / 'ui' / 'main_window.py'
        tree = ast.parse(mw.read_text(encoding='utf-8'))
        targets = {'on_sitl_download_mission', '_on_mission_downloaded',
                   '_on_mission_download_failed'}
        found = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef) and node.name in targets:
                found.add(node.name)
                for sub in ast.walk(node):
                    if (isinstance(sub, ast.Call)
                            and isinstance(sub.func, ast.Attribute)
                            and isinstance(sub.func.value, ast.Name)
                            and sub.func.value.id == 'logger'):
                        assert len(sub.args) == 1, (
                            f'{node.name} 第 {sub.lineno} 行的 logger 傳了 '
                            f'{len(sub.args)} 個位置參數')
        assert found == targets, f'少了這些方法：{targets - found}'
