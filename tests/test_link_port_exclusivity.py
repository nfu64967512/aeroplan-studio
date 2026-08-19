# -*- coding: utf-8 -*-
"""分流埠必須獨佔 —— 「同時跑的兩個消費者共用一個 tcpin」是靜默死局。

═══ 2026-08-07 事故(AeroPlan 互動路徑)═══
MAVProxy 的 `--out=tcpin:` 只餵**第一個**連上的 client。而:

    master_node  → 6960   (UAV1 的 WP 鏈路)   先連
    backup_node  → 6970   (UAV2 的 WP 鏈路)   先連
    tot_node     → 6960 且 6970(它要全機)    後連 ⇒ 收到 0 bytes

實測(`ss -tnpi`,兩機 AeroPlan 航次):

    master_node → 6960   bytes_received:847965      ← 有餵
    tot_node    → 6960   (ss 省略該欄 = 0)          ← 餓死
    tot_node    → 6970   (ss 省略該欄 = 0)          ← 餓死

後果:`ready_for_target` 恆 false ⇒ AeroPlan 右鍵「以此點執行 STOT 打擊」
按下去毫無反應。而且 **完全靜默**:tot_node 的「等機體定位中」告警只在
`INIT` 印,`_tick()` 對 STANDBY 走的是它**前面**那條 early return。
待命節點從啟動那三行之後,log 再也沒有任何一行。

⭐ 批次路徑(stot_trial.sh)為什麼一直沒踩到:它在交棒前先
`swarm_ros2_down.sh` 把整個編隊層收掉(stot_trial.sh:425),6960/6970
空出來,航次用的 tot_node 才變成第一 client。互動路徑是**兩層同時跑**,
沒有這個機會 —— 這就是「批次會成功、產品路徑在失敗帶」的機制。

修法:給 tot_node 專屬的第四路 `6860+10i`。
"""
import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
W = ROOT / "scripts" / "wsl"
SITL = (W / "swarm_sitl_up.sh").read_text(encoding="utf-8")
UP = (W / "swarm_ros2_up.sh").read_text(encoding="utf-8")
TRIAL = (W / "stot_trial.sh").read_text(encoding="utf-8")
TOT = (W / "sar_swarm_patches" / "tot_node.py").read_text(encoding="utf-8")
LAUNCH = (W / "sar_swarm_patches" /
          "sar_failover_nodes.launch.py").read_text(encoding="utf-8")


def _out_ports():
    """從 _launch_bridge 抽出所有 --out=tcpin 的埠基底。"""
    m = re.search(r"_launch_bridge\(\).*?\n\}", SITL, re.S)
    assert m, "找不到 _launch_bridge"
    body = m.group(0)
    base = {}
    # ⚠️ mp/ap/np 是同一行 `local m=… mp=… ap=… np=…` 宣告的,
    #    不可以要求 `local` 緊貼變數名。
    for name, expr in re.findall(r"(\w+)=\$\(\((\d+) \+ 10 \* i\)\)", body):
        base[name] = int(expr)
    outs = re.findall(r"--out=tcpin:0\.0\.0\.0:\$(\w+)", body)
    return base, outs


class TestSitlOpensADedicatedTotPort:
    def test_four_tcpin_outputs(self):
        """MAVROS / AeroPlan / tot_node / 編隊層 —— 四個獨立消費者,四個埠。"""
        _base, outs = _out_ports()
        assert len(outs) == 4, "只有 %d 路 tcpin:%s" % (len(outs), outs)

    def test_tot_port_is_6860(self):
        base, outs = _out_ports()
        assert "tp" in base and base["tp"] == 6860
        assert "tp" in outs, "6860 這一路沒有真的接到 --out"

    def test_every_consumer_has_its_own_base(self):
        """⚠️ 核心不變式:沒有任何兩個消費者共用同一個埠基底。"""
        base, outs = _out_ports()
        used = [base[o] for o in outs]
        assert len(set(used)) == len(used), "有埠基底重複:%s" % sorted(used)

    def test_bases_are_stride_safe(self):
        """埠基底之間至少差 100 ⇒ 撐得住 N=10(stride 10)不互相踩。"""
        base, outs = _out_ports()
        used = sorted(base[o] for o in outs)
        for a, b in zip(used, used[1:]):
            assert b - a >= 100, "%d 與 %d 只差 %d,N>10 就會重疊" % (a, b, b - a)

    def test_existing_three_unchanged(self):
        """既有三路一個位元都不能動 —— MAVROS / AeroPlan / 編隊層都在用。"""
        base, _outs = _out_ports()
        assert base["mp"] == 5860 and base["ap"] == 6760 and base["np"] == 6960


class TestStandbyUsesTheDedicatedPort:
    def test_default_link_base_is_6860(self):
        m = re.search(r'TOT_LINK_BASE="\$\{TOT_LINK_BASE:-(\d+)\}"', UP)
        assert m, "找不到 TOT_LINK_BASE 預設"
        assert int(m.group(1)) == 6860

    def test_not_the_formation_port(self):
        """⚠️ 退回 6960 會讓病無聲地復發 —— 那是編隊層的埠。"""
        m = re.search(r'TOT_LINK_BASE="\$\{TOT_LINK_BASE:-(\d+)\}"', UP)
        assert int(m.group(1)) != 6960

    def test_formation_nodes_still_on_6960(self):
        """編隊層不動:master/backup 各連自己那一台的 6960+10i,本來就不衝突。"""
        assert "6960" in LAUNCH or "6960" in UP

    def test_batch_path_left_alone(self):
        """⚠️ stot_trial.sh 仍走 6960 —— 那是 TWO1 唯一達標組態驗證過的埠,
        而且它交棒前會收掉編隊層,6960 是空的。不要「順手統一」。"""
        assert "-p link_base:=6960" in TRIAL

    def test_batch_path_still_tears_down_formation_first(self):
        """批次路徑的正確性**依賴**這一步:不收編隊層,6960 就被佔著。"""
        i_down = TRIAL.index("swarm_ros2_down.sh")
        i_tot = TRIAL.index("-p link_base:=6960")
        assert i_down < i_tot, "tot_node 在收掉編隊層之前就起了 —— 會被餓死"


class TestStandbyFailureIsNoLongerSilent:
    def test_standby_tick_warns(self):
        assert "self._standby_warn_pos(now)" in TOT

    def test_warn_helper_exists(self):
        assert "def _standby_warn_pos(self, now):" in TOT

    def test_warning_names_the_root_cause(self):
        """告警要講得出「查什麼」,不然和沒講一樣。"""
        m = re.search(r"def _standby_warn_pos\(self, now\):(.*?)\n    def ", TOT, re.S)
        assert m
        body = m.group(1)
        assert "ready_for_target=false" in body
        assert "bytes_received" in body, "沒有給查證方法"
        assert "self._link_desc" in body, "沒有印出是哪些埠"

    def test_throttled_not_spammy(self):
        """待命是長時間狀態,每秒印會把 log 洗掉。"""
        m = re.search(r"def _standby_warn_pos\(self, now\):(.*?)\n    def ", TOT, re.S)
        assert "30.0" in m.group(1)

    def test_reports_recovery_too(self):
        """由缺到齊要補一行正向回報,否則使用者不知道何時可以按。"""
        m = re.search(r"def _standby_warn_pos\(self, now\):(.*?)\n    def ", TOT, re.S)
        assert "_standby_pos_ok" in m.group(1)
        assert "ready_for_target=true" in m.group(1)

    @pytest.mark.parametrize("attr,default", [
        ("_standby_pos_warn_t", "0.0"),
        ("_standby_pos_ok", "True"),
    ])
    def test_class_level_defaults_for_stub_nodes(self, attr, default):
        """本 repo 的測試樁不跑 __init__ —— 少了 class 預設會 AttributeError。"""
        assert re.search(r"^    %s = %s$" % (attr, default), TOT, re.M)

    def test_instance_init_starts_not_ok(self):
        """⚠️ class 預設 True 是給樁用的;真實節點必須從 False 起算,
        才會在第一次湊滿時印那行正向回報。"""
        assert re.search(r"self\._standby_pos_ok = False", TOT)

    def test_link_desc_built_from_actual_params(self):
        """訊息裡的埠號要來自實際參數,不可以寫死 —— 寫死就會在改埠後說謊。"""
        m = re.search(r"self\._link_desc = \((.*?)\)\n", TOT, re.S)
        assert m
        assert "lbase" in m.group(1) and "lstride" in m.group(1)


class TestBringUpVerifiesReadiness:
    def test_checks_ready_for_target_not_just_sentinel(self):
        """⚠️「進了 STANDBY」不等於「接得了目標」—— 事故當下那行 ✔ 照樣印。"""
        assert '"ready_for_target": true' in UP

    def test_failure_message_points_at_the_port(self):
        i = UP.index('"ready_for_target": true')
        tail = UP[i:i + 2000]
        assert "bytes_received" in tail
        assert "6860" in tail

    def test_readiness_check_is_after_the_sentinel_check(self):
        """順序有意義:節點沒起來時不該再去 echo topic 空等 40 s。"""
        assert UP.index("STANDBY 待命就緒") < UP.index('"ready_for_target": true')
