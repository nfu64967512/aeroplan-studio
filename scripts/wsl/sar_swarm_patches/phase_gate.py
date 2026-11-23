"""爬升/巡航相位閘門 —— 帶遲滯與最短駐留的 latch。

⚠️⚠️ 為什麼需要這個模組(P91 實測,不是設計潔癖):

  wingman_node 原本用裸比較 `alt < cruise_alt_threshold` 判定爬升段,
  而且在三個地方各自比一次。P91 的節點日誌顯示 UAV5 在 200 s 內
  來回穿越 3000 m 閘門 **10 次**(cmdlog 重放後是 18 次),
  每次只低於閘門 1–23 m —— 實測最低點 2977 / 2983 / 2986 / 2987 /
  2992 / 2994 / 2997 / 2999 m。同一趟 UAV2 翻 1 次、UAV3/UAV4 各 0 次。
  **翻轉次數與末端 slot 誤差完全對應**:UAV5 發散 33 km,其餘停在 1400–1600 m。

  每次翻轉同時做三件破壞性的事:
    (a) 速度指令從追趕 157 EAS 掉回 best_climb 96 EAS
        —— 追趕通道當場關掉,而縱向就只有這一條有權限的通道;
    (b) 側向展開角瞬間塌回 0°,rank2 的 slot 橫向跳 4589 m,
        再用 expand_time_s(120 s)重新展開;下一次下沉在 5–40 s 後就來,
        **永遠展不完**;
    (c) bearing_off 被強制收成 180°(純 trail),橫向站位放棄。

  這是**閘門自己造出來的極限環**:飛機為了追趕而拿高度換速度 →
  下沉觸發閘門 → 閘門砍掉追趕速度 → 爬回高度 → 再加速 → 再下沉。
  巡航窗口從 168 s 拉到 630 s 末端誤差不會變好,正是因為時間都花在這裡。

  巡航高度 3100 m、閘門 3000 m,餘裕只有 100 m,而追趕中的飛機本來
  就會拿高度換速度 ⇒ 沒有遲滯就必然抖動。

⇒ 規則:**進入巡航段用 threshold(第一次到達就算數);退出要同時滿足
  (1) 低於 threshold − hysteresis、(2) 連續 dwell 秒。**
  3100 巡航 / 3000 閘門 / 150 遲滯 ⇒ 退出線 2850 m,
  比 P91 觀測到的最低點(重放值 2911 m)還低 61 m ⇒ 那些抖動全部被吃掉,
  但真的掉高度(連續 5 s 低於 2850)仍然會誠實地退回爬升段。

本模組刻意不依賴 rclpy / MAVROS,時間與高度都由呼叫端注入,
所以可以在 pytest 裡用真實航次的高度序列重放。
"""


class LatchedAltGate:
    """爬升(True)/巡航(False)的單一真相來源。

    用法:每個控制 tick 呼叫一次 `update(alt, now)`,其餘所有消費者
    讀 `climbing` 屬性 —— **不要各自重新比較高度**,否則同一個 tick 內
    不同消費者可能看到不同相位。
    """

    def __init__(self, threshold_m, hysteresis_m=150.0, dwell_s=5.0,
                 start_climbing=True):
        self.threshold_m = float(threshold_m)
        self.hysteresis_m = max(0.0, float(hysteresis_m))
        self.dwell_s = max(0.0, float(dwell_s))
        self.climbing = bool(start_climbing)
        self.flips = 0              # 翻轉次數:遲滯是否生效的直接驗收量
        self._sag_t0 = None         # 連續低於退出線的起算時刻
        self._last_exit_alt = None  # 最近一次退出巡航段時的高度(給日誌用)

    @property
    def exit_alt_m(self):
        """低於此高度才**可能**退回爬升段(還要撐滿 dwell 秒)。"""
        return self.threshold_m - self.hysteresis_m

    @property
    def enabled(self):
        return self.threshold_m > 0.0

    def update(self, alt_m, now_s):
        """推進狀態機。alt_m=None(沒有定位)時維持現狀,不盲翻。

        回傳 (climbing, changed)。
        """
        if not self.enabled:
            changed, self.climbing = self.climbing, False
            return False, bool(changed)
        if alt_m is None:
            return self.climbing, False
        alt, now = float(alt_m), float(now_s)
        before = self.climbing
        if self.climbing:
            if alt >= self.threshold_m:
                self.climbing, self._sag_t0 = False, None
        elif alt < self.exit_alt_m:
            if self._sag_t0 is None:
                self._sag_t0 = now          # 這一拍開始計時
            # ⚠️ 這裡要用 if 而不是 elif:dwell_s=0 時必須**當拍**就翻,
            #    否則 hyst=0/dwell=0 不會退化成原本的裸比較,A/B 基準線就不對齊了。
            if (now - self._sag_t0) >= self.dwell_s:
                self.climbing, self._sag_t0 = True, None
                self._last_exit_alt = alt
        else:
            self._sag_t0 = None             # 爬回遲滯帶內 → 取消計時
        changed = self.climbing != before
        if changed:
            self.flips += 1
        return self.climbing, changed


class MonotonicRamp:
    """單調不減的 0→1 斜坡,只在「允許」的 tick 累積,否則**凍結**。

    ⚠️ 側向展開專用。原實作用起算時刻 `t0`,一回到爬升段就把 t0 歸零
       ⇒ 展開角塌回 0°。rank2 的側偏是 ±4589 m,塌回再展開等於命令 slot
       橫向瞬移 4.6 km —— wingman_node 本來就有「不可以在門檻上硬切換」
       的註解,但只防了「往上展開」那一邊,沒防「往下塌回」。
    ⇒ 改成累積器:爬升段凍結(保住已展開的角度),巡航段才繼續前進。
       就算遲滯被真實的掉高度突破,slot 也只是停在原地等,不會橫向瞬移。
    """

    def __init__(self, duration_s, max_step_s=3.0):
        self.duration_s = float(duration_s)
        self.max_step_s = float(max_step_s)   # 夾住排程抖動/暫停
        self.accumulated_s = 0.0
        self._last_t = None

    def update(self, now_s, advance):
        """advance=True 才累積時間。回傳 0.0–1.0 的比例。"""
        last, self._last_t = self._last_t, float(now_s)
        if self.duration_s <= 0.0:
            return 1.0
        if advance and last is not None:
            self.accumulated_s += max(0.0, min(float(now_s) - last,
                                               self.max_step_s))
        return max(0.0, min(1.0, self.accumulated_s / self.duration_s))

    @property
    def fraction(self):
        if self.duration_s <= 0.0:
            return 1.0
        return max(0.0, min(1.0, self.accumulated_s / self.duration_s))
