"""preflight_contracts.py — 執行期契約檢查(runtime assertions)。

【任務 D2】把本輪踩過的 7 類**靜默失敗**變成開跑就會叫的錯誤。

為什麼不能只靠靜態檢查:這 7 個 bug **`bash -n`、`python -c ast.parse`、
`colcon build` 全部都通過**。它們的共同結構是「值合法但語意錯」或
「名字不存在所以被忽略」—— 只有在**使用前對照實際生效值**才抓得到。

設計原則(Design by Contract):
  1. **前置條件在使用點檢查**,不是在文件裡寫「請記得設」。
  2. **失敗要吵**:預設 raise。要降級必須顯式 `level="warn"`,
     而且降級本身也要印出來 —— 靜默降級就是這一整類 bug 的成因。
  3. **檢查的是「實際生效值」**,不是「我們送出去的值」。
     指令記錄器記 157 但飛控拒收,就是因為只記了前者。

用法(節點啟動時):
    from sar_swarm.preflight_contracts import ContractSet
    c = ContractSet("wingman_node", logger=self.get_logger())
    c.param_exists("ROLL_LIMIT_DEG", fc_params)          # 名字打錯會被靜默忽略
    c.in_range("catch_speed", self.catch_speed, self.airspeed_min,
               self.airspeed_max, why="超出區間會被飛控整發拒收")
    c.require_env("CLIMB_TO_ALT", min_value=1000.0)
    c.report()          # 一次印出全部結果;有 fail 就 raise
"""
from __future__ import annotations

import os


class ContractViolation(RuntimeError):
    pass


class ContractSet:
    """收集一組契約檢查,最後一次回報。分開收集是為了讓使用者**一次看到全部
    問題**,而不是修一個、重跑、再撞下一個。"""

    def __init__(self, owner: str, logger=None, level: str = "raise"):
        self.owner = owner
        self.logger = logger
        self.level = level          # "raise" | "warn"
        self.results = []           # [(ok, name, detail)]

    # ── 基礎 ──────────────────────────────────────────────────────────
    def _add(self, ok, name, detail):
        self.results.append((bool(ok), name, detail))
        return bool(ok)

    def check(self, ok, name, detail=""):
        return self._add(ok, name, detail)

    # ── 1) 參數名存在性 ───────────────────────────────────────────────
    def param_exists(self, name, available):
        """⚠️ 失敗模式#1:**參數名在這個 build 不存在 ⇒ 被靜默忽略**。
        本輪實例:`ARSPD_STRICT`、`SIM_ARSPD_FAIL`、`LIM_ROLL_CD`
        (正確的是 `ARSPD_OPTIONS` / `ROLL_LIMIT_DEG`,而且單位是度不是 centidegree)。
        `available` 應該是**從飛控讀回來**的參數名集合,不是我們自己的清單。"""
        return self._add(name in available, f"param_exists({name})",
                         "" if name in available
                         else "此 build 沒有這個參數 —— 設了會被靜默忽略")

    # ── 2) 生效值與期望值一致 ─────────────────────────────────────────
    def param_equals(self, name, actual, expected, tol=1e-6):
        """⚠️ 失敗模式#2:**送出去了但沒生效**。
        本輪實例:`CLIMB_THR_MAX=35` 從未下發,飛控整場 `THR_MAX=100`;
        `JET_THRMAX` 為空 ⇒ 巡航設計值 21 也沒設過。
        ⇒ 一律**讀回來比對**,不要相信「我有送」。"""
        ok = actual is not None and abs(float(actual) - float(expected)) <= tol
        return self._add(ok, f"param_equals({name})",
                         "" if ok else f"期望 {expected} 實得 {actual}")

    # ── 3) 指令值落在飛控接受區間 ─────────────────────────────────────
    def in_range(self, name, value, lo, hi, why=""):
        """⚠️ 失敗模式#3:**值超出飛控接受區間 ⇒ 整發拒收(不是夾住)**。
        本輪實例:`catch_speed=157` > `AIRSPEED_MAX=150`
        ⇒ 每一發追趕指令都被丟掉,而追趕是縱向唯一有權限的通道。"""
        ok = value is not None and lo <= float(value) <= hi
        return self._add(ok, f"in_range({name})",
                         "" if ok else f"{value} 不在 [{lo}, {hi}];{why}")

    # ── 4) 必要環境變數 ───────────────────────────────────────────────
    def require_env(self, name, min_value=None, why=""):
        """⚠️ 失敗模式#4:**必要環境變數沒設 ⇒ 整段功能被跳過**。
        本輪實例:`CLIMB_TO_ALT` 沒設 ⇒ 只爬到 ~100 m 就交棒協同層,
        階段 2 整段在爬升中跑,末端誤差 44 km(與導引邏輯完全無關)。"""
        raw = os.environ.get(name)
        ok = raw is not None and raw != ""
        if ok and min_value is not None:
            try:
                ok = float(raw) >= float(min_value)
            except ValueError:
                ok = False
        return self._add(ok, f"require_env({name})",
                         "" if ok else f"值={raw!r} 期望 ≥{min_value};{why}")

    # ── 5) 逾時/預算與工作量相稱 ──────────────────────────────────────
    def budget_covers(self, name, budget_s, need_s, why=""):
        """⚠️ 失敗模式#5:**修好一個一直沒生效的保護之後,它的參數已經過期**。
        本輪實例:`timeout` 因為續行裡夾註解而從未生效,所以
        `LAUNCH_TIMEOUT=260` 再小都無所謂;bug 修好後它第一次真的計時,
        當場把爬升砍在 2347 m(爬到 3100 m 需要 ~500 s)。
        ⇒ **修 bug 會讓死參數突然復活**,要同時重新標定。"""
        ok = budget_s is None or float(budget_s) >= float(need_s)
        return self._add(ok, f"budget_covers({name})",
                         "" if ok else f"預算 {budget_s}s < 需要 {need_s}s;{why}")

    # ── 6) 輸出路徑不會覆蓋既有證據 ───────────────────────────────────
    def output_is_fresh(self, path, why=""):
        """⚠️ 失敗模式#6:**覆寫式記錄檔吃掉上一趟的證據**。
        本輪實例:`runs/cmdlog/uav{id}_*.csv` 路徑寫死又用 "w" 開檔,
        每趟覆蓋;P91 的資料是靠開跑前手動備份才留下來的。
        「先量不要猜」的前提是**量到的東西還在**。"""
        exists = os.path.exists(path)
        return self._add(not exists, f"output_is_fresh({os.path.basename(path)})",
                         "" if not exists else f"{path} 已存在,會被覆蓋;{why}")

    # ── 7) 指令的前置狀態(模式閘)────────────────────────────────────
    def mode_allows(self, cmd_name, mode, allowed):
        """⚠️ 失敗模式#7:**不在允許的飛行模式 ⇒ 指令被靜默丟棄**。
        本輪實例:`DO_CHANGE_SPEED` 只在 GUIDED/AUTO 被接受
        (`GCS_MAVLink_Plane.cpp::handle_command_DO_CHANGE_SPEED` 開頭就擋),
        模式轉換空窗期送的指令全部丟失,而 CSV 照樣記 —— log 與實際行為不一致。"""
        ok = mode in allowed
        return self._add(ok, f"mode_allows({cmd_name})",
                         "" if ok else f"目前模式 {mode} 不在 {sorted(allowed)}")

    # ── 回報 ──────────────────────────────────────────────────────────
    def report(self, level=None):
        level = level or self.level
        bad = [r for r in self.results if not r[0]]
        lines = [f"[contract:{self.owner}] {len(self.results)-len(bad)}/"
                 f"{len(self.results)} 通過"]
        for ok, name, detail in self.results:
            if not ok:
                lines.append(f"  ✗ {name}: {detail}")
        msg = "\n".join(lines)
        if not bad:
            self._log("info", msg)
            return True
        # ⚠️ 降級本身也要印出來 —— 靜默降級正是這一整類 bug 的成因
        if level == "warn":
            self._log("error", msg + "\n  ⚠️ level=warn:**已知有違規仍繼續執行**")
            return False
        self._log("error", msg)
        raise ContractViolation(msg)

    def _log(self, lvl, msg):
        if self.logger is None:
            print(msg)
            return
        getattr(self.logger, {"info": "info", "error": "error"}[lvl])(msg)
