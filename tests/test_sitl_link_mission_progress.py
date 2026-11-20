"""mission.sitl_link：建圖條帶監測（A5）用的任務進度信號 —— MISSION_CURRENT 與 STATUSTEXT「Mission: N <type>」。

只收**已確認是飛機的自駕儀元件**送的（同 sysid 的相機、地面站 255、沒發過心跳的 sysid 都不能偽造進度）；
MISSION_CURRENT 只在任一欄位變了才送。訊息用 pymavlink MAVLink2 編碼再解碼（帶真的 sysid/compid），
直接餵 _handle_message，不開任何連線。
"""
from __future__ import annotations

from tests.test_sitl_link_camera import cam_hb, hb, make_link, mk


def mc(src, comp, seq, total=181, state=3, mode=1):
    return mk(src, comp, 'mission_current', seq, total, state, mode)


def stxt(src, comp, text, sev=6):
    return mk(src, comp, 'statustext', sev, text.encode('ascii'))


def _wire(link):
    link._got_mc, link._got_item = [], []
    link.mission_current.connect(lambda s, d: link._got_mc.append((s, d)))
    link.mission_item_started.connect(lambda s, q, t: link._got_item.append((s, q, t)))
    return link


def test_mission_current_is_emitted_on_change_only_and_kept_per_sysid():
    link = _wire(make_link())
    link._handle_message(hb(1, 1))
    for seq in (17, 17, 19, 19, 21):
        link._handle_message(mc(1, 1, seq))
    assert [(s, d['seq']) for s, d in link._got_mc] == [(1, 17), (1, 19), (1, 21)]
    assert link._got_mc[0][1] == {'seq': 17, 'total': 181, 'mission_state': 3, 'mission_mode': 1}
    link._handle_message(mc(1, 1, 21, mode=0))                       # 只有 mission_mode 變（離開 AUTO）也要送
    assert link._got_mc[-1][1]['mission_mode'] == 0 and len(link._got_mc) == 4
    assert link.get_mission_current(1) == {'seq': 21, 'total': 181, 'mission_state': 3, 'mission_mode': 0}
    assert link.get_mission_current() == link.get_mission_current(1)   # 預設 sysid_label
    assert link.get_mission_current(2) is None
    snap = link.get_mission_current(1)
    snap['seq'] = 999
    assert link.get_mission_current(1)['seq'] == 21                    # 回的是複本


def test_mission_current_from_camera_gcs_or_unknown_sysid_is_ignored():
    link = _wire(make_link())
    link._handle_message(hb(1, 1))
    link._handle_message(cam_hb(1, 100))
    link._handle_message(mc(1, 100, 55))                               # 同 sysid 的相機元件
    link._handle_message(mc(255, 190, 55))                             # 地面站
    link._handle_message(mc(7, 1, 55))                                 # 沒發過心跳的 sysid
    assert link._got_mc == [] and link.get_mission_current(1) is None and link.get_mission_current(7) is None


def test_mission_text_becomes_item_started_only_from_the_autopilot():
    link = _wire(make_link())
    link._handle_message(hb(1, 1))
    link._handle_message(cam_hb(1, 100))
    link._handle_message(stxt(1, 1, 'Mission: 18 SetCamTrigDst'))
    link._handle_message(stxt(1, 1, 'Mission: 179 LandStart'))
    link._handle_message(stxt(1, 100, 'Mission: 5 WP'))                # 相機元件偽造
    link._handle_message(stxt(255, 190, 'Mission: 6 WP'))              # 地面站
    link._handle_message(stxt(7, 1, 'Mission: 8 WP'))                  # 沒發過心跳的 sysid
    link._handle_message(stxt(1, 1, 'Reached waypoint #18'))           # 不是 Mission: 格式
    assert link._got_item == [(1, 18, 'SetCamTrigDst'), (1, 179, 'LandStart')]
    # 原本的 status_text 照舊：每一則都轉出（不過濾、不帶 sysid）
    assert [t for _sev, t in link._got_status if t.startswith(('Mission', 'Reached'))] == [
        'Mission: 18 SetCamTrigDst', 'Mission: 179 LandStart', 'Mission: 5 WP', 'Mission: 6 WP', 'Mission: 8 WP',
        'Reached waypoint #18']


def test_sysid_label_is_accepted_before_its_first_heartbeat():
    """本鏈路明確標示的機號永遠放行（與機卡規則相同）；自駕儀元件 compid 1。"""
    link = _wire(make_link(sysid_label=3))
    link._handle_message(mc(3, 1, 40))
    link._handle_message(stxt(3, 1, 'Mission: 41 SetCamTrigDst'))
    assert [(s, d['seq']) for s, d in link._got_mc] == [(3, 40)] and link._got_item == [(3, 41, 'SetCamTrigDst')]
