# -*- coding: utf-8 -*-
"""`scripts/wsl/s2_readout.py` 的迴歸鎖(2026-08-18)。

═══ 為什麼需要這一支 ═══

`s2_readout.py` 是 S2(`wait_goto_radius`)這條線的**唯一判讀工具** —— RRD 分成
「硬彎 5 架 0/5 / 直進 11 架 10/11」兩個母體、RRE 之後要證明「S2 生效」,結論全部
由它產出。它兩輪下來被修了 15+ 條缺陷,但**一條鎖都沒有**:全部只靠沙盒實跑驗證,
明天任何一次「順手簡化」都可能把它改回去,而且**不會有任何測試變紅**。

⚠️⚠️ 這支腳本的失效模式全部是**靜默**的:它不會當掉,它會印出一份看起來很正常、
   數字卻是假的報表。本檔鎖的就是那些「看起來正常」的形狀:

     · 空的一群印成裸的 `0/0` ——「S2 生效(硬彎母體消失)」與「全面回歸」
       長得一模一樣,而且最好的結果與最壞的結果都印成同一個 `0/0`;
     · 雜訊等級的最大間隙被講成「把 11 架切成 3 / 8」的偽分群;
     · 兩個空集合互相比對後蓋一個「互相佐證 ✅」的章;
     · 契約說開、CSV 卻沒有儀表欄的**混合批次**完全不報警;
     · 判定量取到 `cmd_radius_m`(hold-last-value ⇒ 開關開關都是 30)⇒ 假綠燈;
     · 契約缺 key 時靜默套上別批次的常數 30/1300 當尺規。

═══ 本檔的鎖與對應的真實缺陷 ═══

  1. RRD 端到端讀數不變(硬彎 5 / 直進 11 / 0/5 / 10/11 / 10/16)—— 唯一一條
     「判讀層整體沒壞」的鎖。RRD 不在就 skip,不 fail。
  2. 答案不可硬編:餵 RRD1–3 必須得到**不同**的數字(3/3/0-3/3-3/6)。
  3. 單一母體(= S2 成功的形狀):不得裸 `0/0`、不得偽分群、誤差中位不得消失、
     必須有正面的生效結論,而且結論要**殿後**。
  4. 混合批次:契約說開但 CSV 沒有儀表欄 ⇒ 必須報警 + 進保留欄。
  5. 契約 vs 致動一律**逐趟**計數:被污染的對照組併批後仍要報警;A/B 併批不得假紅。
  6. 判定量是 `req_radius_m` 不是 `cmd_radius_m`。
  7. `req_radius_m` 覆蓋率 < 90% ⇒ 不可判,不得印「✅ 全段」。
  8. σ_pre 全部算不出來 ⇒ **中止**(非零退出),不得印 0/0。
  9. 契約缺 goto_radius / hold_radius ⇒ 警語 + 保留,不得靜默套 30/1300。
 10. 判準 C 與 σ_pre 兩邊都是空集合 ⇒ 不得蓋「互相佐證 ✅」。

═══ 沙盒紀律 ═══

⚠️ `s2_readout.ART` 是 `ROOT/runs/artifacts`(模組層常數,`ROOT = HERE.parents[2]`)
   ⇒ 合成批次一律寫進 pytest 的 `tmp_path` 並 monkeypatch `ART` 重導,
   **絕對不寫進 repo 的 runs/artifacts**(那會污染真實航次資料)。
⚠️ 合成批次把 `score_trial()`(= 跑 `stot_score.py` 子行程)換成樁:本檔鎖的是
   **判讀/分群/報表**那一層,不是計分。計分的真實路徑由第 1 條 RRD 端到端鎖覆蓋。
"""
import contextlib
import csv
import io
import json
import os
import re
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts", "wsl"))
import s2_readout  # noqa: E402


# ══════════════════════════════════════════════════════════════════════════
#  執行輔助
# ══════════════════════════════════════════════════════════════════════════
class _Cap(io.StringIO):
    """可當 stdout 用的緩衝區。

    ⚠️ `s2_readout._enc_guard()` 會讀 `sys.stdout.encoding` —— 純 `io.StringIO`
       沒有這個屬性,會噴 AttributeError(它只捕捉 UnicodeEncodeError/LookupError)。
       ⇒ 補上 `encoding` 與 `reconfigure`,讓 CLI 入口(`main()`)也能被鎖。
    """

    encoding = "utf-8"

    def reconfigure(self, **_kw):        # pragma: no cover - 只是讓護欄跑得過
        return None


def run_cli(argv):
    """跑 `s2_readout.main(argv)`,回傳 stdout 全文。"""
    buf = _Cap()
    with contextlib.redirect_stdout(buf):
        s2_readout.main(list(argv))
    return buf.getvalue()


def run_report(trials, split=None):
    """跑 `report()`,回傳 stdout 全文。"""
    buf = _Cap()
    with contextlib.redirect_stdout(buf):
        s2_readout.report(list(trials),
                          s2_readout.SIG_PRE_SPLIT_DEG if split is None else split)
    return buf.getvalue()


def run_report_expect_abort(trials, split=None):
    """跑 `report()` 並要求它**中止**。回傳 (已印出的 stdout, SystemExit 實例)。"""
    buf = _Cap()
    with contextlib.redirect_stdout(buf):
        with pytest.raises(SystemExit) as ei:
            s2_readout.report(list(trials),
                              s2_readout.SIG_PRE_SPLIT_DEG if split is None else split)
    return buf.getvalue(), ei.value


def last_line(out):
    """報表的最後一句話(去掉尾端空行)。"""
    lines = [ln for ln in out.splitlines() if ln.strip()]
    return lines[-1] if lines else ""


def section(out, title, until=None):
    """截出某一段(從 `title` 那一行到下一個 `【` 標題或 `until`)。

    ⚠️ 標題必須**整行相符**:報表內文自己會引用別段的標題
       (例:「要證明 S2 生效,看【S2 機制生效證明】那一段」),
       用 `in` 比對會截到那一句上,拿到一段空的內文然後靜靜地全綠。
    """
    lines = out.splitlines()
    try:
        i = next(k for k, ln in enumerate(lines) if ln.strip() == title)
    except StopIteration:
        return ""
    body = []
    for ln in lines[i + 1:]:
        if until is not None and until in ln:
            break
        if until is None and ln.lstrip().startswith("【"):
            break
        body.append(ln)
    return "\n".join(body)


def bare_zero_over_zero(out):
    """回傳所有印出**裸** `0/0` 的行。

    ⚠️ 報表自己會**明講**「命中率無從計算,不是 0/0」—— 那是在解釋為什麼不印,
       不算違規。只有在沒有這句否定語的行上出現的 `0/0` 才是本檔要抓的東西。
    """
    return [ln for ln in out.splitlines()
            if "0/0" in ln and "不是 0/0" not in ln]


def warn_line(out):
    """【本次判讀的保留】那一行。"""
    for ln in out.splitlines():
        if "【本次判讀的保留】" in ln:
            return ln
    return ""


# ══════════════════════════════════════════════════════════════════════════
#  合成批次產生器(寫進 tmp_path,不碰 repo)
# ══════════════════════════════════════════════════════════════════════════
TGT = (25.0000000, 120.0000000)
PPS = {"pp0": (25.1000000, 120.0500000), "pp1": (25.0500000, 120.1000000)}

DT = 0.2            # 取樣間隔(s)。> 1.0 s 才會觸發資料品質的空檔段 ⇒ 這裡不觸發
T_END = 40.0
T_WAIT = 10.0       # 進 wait 的時刻
T_REL = 30.0        # 放行(mode 進 dive)的時刻;σ_pre 窗 = [20, 30) 全落在 wait 段內

BASE_FIELDS = ["t", "sysid", "lat", "lon", "alt", "gs", "mode", "agl",
               "wp_lat", "wp_lon"]
LEAD_FIELD = "lead_angle_deg"
S2_FIELDS = ["req_radius_m", "cmd_radius_m"]


class Sandbox:
    """一棵假的 `runs/artifacts` 目錄樹 + 一份假的計分結果。"""

    def __init__(self, root):
        self.art = root / "runs" / "artifacts"
        self.art.mkdir(parents=True, exist_ok=True)
        self.miss = {}          # trial -> {sysid: {"impact_miss_m": ..., ...}}

    # ── 建趟次 ────────────────────────────────────────────────────────────
    def add_trial(self, name, aircraft, *, s2_cols=True, contract=True,
                  wait_goto_radius="1.0", goto_radius="30.0",
                  hold_radius="1300.0", drop_contract_keys=()):
        """`aircraft` 是一串 dict,每個描述一架:

            lead      wait 段的 |lead_angle_deg|(決定 σ_pre ⇒ 決定硬彎/直進)
            miss      stot_score 的觸地誤差(None = 不可判)
            req       wait 段每拍要求的半徑(寫進 req_radius_m)
            cmd       wait 段每拍送出的半徑(寫進 cmd_radius_m)
            req_cov   req_radius_m 的覆蓋率(0.0–1.0;其餘列寫空字串)
            lead_col  False ⇒ 整個 lead_angle_deg 欄不存在(σ_pre 算不出來)
            land      False ⇒ agl 一路是正的(掃不到觸地)
        """
        d = self.art / name
        d.mkdir(parents=True, exist_ok=True)

        fields = list(BASE_FIELDS)
        if any(a.get("lead_col", True) for a in aircraft):
            fields.insert(BASE_FIELDS.index("agl") + 1, LEAD_FIELD)
        if s2_cols:
            fields += S2_FIELDS

        rows = []
        pp_names = sorted(PPS)
        for k, a in enumerate(aircraft):
            rows += self._rows(a, sysid=a.get("sysid", k + 1),
                               pp=PPS[pp_names[k % len(pp_names)]],
                               fields=fields, s2_cols=s2_cols)
        with open(d / f"tot_{name}.csv", "w", encoding="utf-8", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=fields, restval="")
            w.writeheader()
            w.writerows(rows)

        with open(d / f"tot_{name}_geom.csv", "w", encoding="utf-8", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["kind", "lat", "lon"])
            w.writerow(["target", "%.7f" % TGT[0], "%.7f" % TGT[1]])
            for n2 in pp_names:
                w.writerow([n2, "%.7f" % PPS[n2][0], "%.7f" % PPS[n2][1]])

        if contract:
            values = {"goto_radius": goto_radius, "hold_radius": hold_radius}
            if wait_goto_radius is not None:
                values["wait_goto_radius"] = wait_goto_radius
            for k2 in drop_contract_keys:
                values.pop(k2, None)
            (d / f"contract_{name}.json").write_text(
                json.dumps({"values": values}, ensure_ascii=False),
                encoding="utf-8")

        self.miss[name] = {}
        for k, a in enumerate(aircraft):
            sid = a.get("sysid", k + 1)
            if a.get("miss") is not None:
                self.miss[name][sid] = {"impact_miss_m": float(a["miss"]),
                                        "impact_t_s": 35.0}
        return name

    def _rows(self, a, *, sysid, pp, fields, s2_cols):
        lead_col = a.get("lead_col", True)
        land = a.get("land", True)
        req_cov = float(a.get("req_cov", 1.0))
        out, n = [], int(round(T_END / DT)) + 1
        wait_i = 0
        for i in range(n):
            t = round(i * DT, 3)
            if t < T_WAIT:
                mode = "straight"
            elif t < T_REL:
                mode = "wait"
            else:
                mode = "dive"
            f = min(t / T_END, 1.0)
            lat = pp[0] + (TGT[0] - pp[0]) * f
            lon = pp[1] + (TGT[1] - pp[1]) * f
            if t < T_REL:
                agl = 3000.0
            elif land:
                agl = 3000.0 - (t - T_REL) * 333.0      # t=39.1 ⇒ −30.3 m(真觸地)
            else:
                agl = 3000.0 - (t - T_REL) * 10.0       # 一路正的 ⇒ 掃不到觸地
            r = {"t": "%.1f" % t, "sysid": str(sysid),
                 "lat": "%.7f" % lat, "lon": "%.7f" % lon,
                 "alt": "3100.0", "gs": "150.0", "mode": mode,
                 "agl": "%.1f" % agl,
                 "wp_lat": "%.7f" % pp[0], "wp_lon": "%.7f" % pp[1]}
            if lead_col and LEAD_FIELD in fields:
                r[LEAD_FIELD] = "%.2f" % (float(a["lead"]) if mode == "wait" else 1.0)
            if s2_cols:
                r["cmd_radius_m"] = "%.1f" % float(a.get("cmd", 30.0))
                if mode == "wait":
                    # 覆蓋率:每 10 拍寫 round(req_cov×10) 拍,其餘留**空字串**
                    # ⚠️ 空字串是這份 CSV 的正常公民(節點用它區分「沒啟用」與
                    #    「值就是 0」)⇒ 不可以寫成 0。
                    quota = max(0, min(10, int(round(req_cov * 10))))
                    r["req_radius_m"] = ("%.1f" % float(a.get("req", 30.0))) \
                        if (wait_i % 10) < quota else ""
                    wait_i += 1
                else:
                    r["req_radius_m"] = "%.1f" % float(a.get("req", 30.0))
            out.append(r)
        return out

    # ── 掛上去 ────────────────────────────────────────────────────────────
    def install(self, monkeypatch):
        """把 `ART` 重導到沙盒,並把 `score_trial` 換成樁。

        ⚠️ 換掉 `score_trial` 是刻意的:本檔鎖的是判讀/分群/報表那一層,
           真的跑 `stot_score.py` 那條路徑由 RRD 端到端鎖覆蓋。
        """
        monkeypatch.setattr(s2_readout, "ART", self.art)
        monkeypatch.setattr(s2_readout, "score_trial",
                            lambda tr: (dict(self.miss.get(tr, {})), None))
        assert s2_readout.ART != s2_readout.ROOT / "runs" / "artifacts", (
            "沙盒沒有生效 —— ART 還指著 repo 的 runs/artifacts,"
            "再跑下去會把假資料寫進真實航次目錄")
        return self


@pytest.fixture()
def sandbox(tmp_path, monkeypatch):
    return Sandbox(tmp_path).install(monkeypatch)


def straight_batch(sb, prefix="SNG", n_trials=6, **kw):
    """單一母體(全部直進)—— S2 生效之後**應該**長的樣子。

    σ_pre 給 0.5, 1.0, … 遞增 ⇒ 排序後相鄰間隙全部 0.5°(遠小於 GAP_MIN_DEG=10°)
    ⇒ 這批**沒有**群內結構,報表不可以從雜訊讀出「切成 N 架 / M 架」。
    """
    trials, lead = [], 0.5
    for i in range(1, n_trials + 1):
        acs = []
        for j in range(2):
            acs.append({"lead": lead, "miss": 0.5 + lead, "req": 30.0, "cmd": 30.0})
            lead += 0.5
        trials.append(sb.add_trial(f"{prefix}{i}", acs, **kw))
    return trials


# ══════════════════════════════════════════════════════════════════════════
#  0. 沙盒自身的前提(前提壞掉要紅,不要靜靜地變成一堆空跑的綠燈)
# ══════════════════════════════════════════════════════════════════════════
class TestHarnessSanity:

    def test_sandbox_never_touches_the_repo_artifacts(self, sandbox, tmp_path):
        """沙盒必須在 tmp_path 底下,而且 repo 的 runs/artifacts 一個檔都不能多。"""
        straight_batch(sandbox)
        assert str(tmp_path) in str(s2_readout.ART)
        real = s2_readout.ROOT / "runs" / "artifacts"
        assert not str(s2_readout.ART).startswith(str(real)), (
            "合成批次寫到了 repo 的 runs/artifacts —— 那是真實航次資料")

    def test_synthetic_batch_really_reads_as_one_population(self, sandbox):
        """前提:straight_batch 真的只產出「直進」一群(12 架)。"""
        out = run_report(straight_batch(sandbox))
        assert "直進組  n = 12" in out
        assert "硬彎組  n =  0" in out


# ══════════════════════════════════════════════════════════════════════════
#  1 & 2. RRD 端到端:讀數不變,而且答案不是硬編的
# ══════════════════════════════════════════════════════════════════════════
RRD_TRIALS = [f"RRD{i}" for i in range(1, 9)]
_CLI_CACHE = {}


def _rrd_available(names):
    """RRD 不在就 skip(不要 fail):artifacts 是本機資料,不進版控。"""
    art = s2_readout.ROOT / "runs" / "artifacts"
    for t in names:
        d = art / t
        if not (d / f"tot_{t}.csv").exists() or not (d / f"tot_{t}_geom.csv").exists():
            return False
    return True


def cli_cached(argv):
    """跑 CLI 並快取 —— 每次都會叫起 `stot_score.py` 子行程(每趟一次)。"""
    key = tuple(argv)
    if key not in _CLI_CACHE:
        _CLI_CACHE[key] = run_cli(argv)
    return _CLI_CACHE[key]


@pytest.mark.slow
class TestRrdReadoutIsUnchanged:
    """⭐ 唯一一條「判讀層整體沒壞」的端到端鎖。

    ⚠️ 它會跑 `stot_score.py` 8 次(實測全批 ~2.5 s)。慢不是不做的理由:
       上面所有沙盒鎖用的都是**假的** score_trial,只有這一條走真實計分路徑。
    """

    @pytest.mark.parametrize("needle", [
        "硬彎組  n =  5",
        "直進組  n = 11",
        "**0/5**",
        "**10/11**",
        "**10/16**",
    ])
    def test_rrd_decisive_readout(self, needle):
        """RRD 必須自吐 硬彎 5 / 直進 11 / 0/5 / 10/11 / 10/16。

        ⚠️ 這五個數字是 S2 這條線的**地基**(根因診斷、交接文件、後續 RRE 對照組
           全部引用它們)。判讀腳本任何一處改動只要動到分群、命中判準、計分外包
           或掃描窗,這裡就必須紅。
        """
        if not _rrd_available(RRD_TRIALS):
            pytest.skip("runs/artifacts/RRD1..RRD8 不在本機 ⇒ 端到端鎖無從執行")
        out = cli_cached(["RRD"])
        assert needle in out, (
            "RRD 的決定性讀數變了(找不到 %r)—— 這五個數字是 S2 診斷的地基,"
            "改判讀腳本前先確認是不是真的想翻掉它們" % needle)

    def test_rrd_true_touchdown_rate(self):
        """16/16 真觸地(自掃 AGL≤0)。⚠️ 落地 ≠ 命中,兩個量分開鎖。"""
        if not _rrd_available(RRD_TRIALS):
            pytest.skip("runs/artifacts/RRD1..RRD8 不在本機")
        out = cli_cached(["RRD"])
        assert "逐機真觸地率 = 16/16" in out, (
            "真觸地率變了 —— 若改用 stot_score 的「最低AGL」欄會憑空造出"
            "一個不存在的「高空掠過」族群(見腳本 docstring 紀律 2)")

    def test_rrd_hard_group_median_error(self):
        """誤差中位:硬彎 42.18 m vs 直進 0.98 m —— 兩群的**尺度差**是結論本身。"""
        if not _rrd_available(RRD_TRIALS):
            pytest.skip("runs/artifacts/RRD1..RRD8 不在本機")
        out = cli_cached(["RRD"])
        assert "誤差中位:硬彎 42.18 m  vs  直進 0.98 m" in out

    def test_rrd_never_claims_s2_works(self):
        """⚠️⚠️ RRD 是 S2 部署**之前**的批次(CSV 沒有 req_radius_m 欄)⇒
        它**不可以**被讀成「機制沒問題」,只代表這個問題在這批資料上量不出來。"""
        if not _rrd_available(RRD_TRIALS):
            pytest.skip("runs/artifacts/RRD1..RRD8 不在本機")
        out = cli_cached(["RRD"])
        assert "**這批沒有 S2 儀表**" in out
        assert "但**不可以**當成「機制沒問題」" in out
        assert "✅✅ 【S2 生效】" not in out, (
            "沒有 S2 儀表欄的批次被宣告成「S2 生效」—— 那是憑空生出來的證據")
        assert not last_line(out).startswith("── 結論:"), (
            "無從判定的批次卻蓋了殿後結論 ⇒ 「量不出來」被印成「已證明」")


@pytest.mark.slow
class TestAnswersAreNotHardcoded:
    """⚠️ 上面那條鎖最危險的作弊法:把 5 / 11 / 10 直接寫死在腳本裡。

    ⇒ 餵**子集**必須得到**不同**的數字。RRD1–3 是 6 架次(硬彎 3 / 直進 3、3/6)。
    """

    def test_subset_gives_its_own_numbers(self):
        if not _rrd_available(["RRD1", "RRD2", "RRD3"]):
            pytest.skip("runs/artifacts/RRD1..RRD3 不在本機")
        out = cli_cached(["RRD1", "RRD2", "RRD3"])
        assert "硬彎組  n =  3" in out and "**0/3**" in out
        assert "直進組  n =  3" in out and "**3/3**" in out
        assert "整批    n =  6" in out and "**3/6**" in out
        for hard_coded in ("硬彎組  n =  5", "**10/16**", "**10/11**"):
            assert hard_coded not in out, (
                "餵 RRD1–3 卻印出整批 RRD 的數字(%r)⇒ 讀數是硬編的,"
                "不是從資料算出來的" % hard_coded)


# ══════════════════════════════════════════════════════════════════════════
#  3 & 10. 單一母體 —— S2 成功時報表**應該**長的樣子
# ══════════════════════════════════════════════════════════════════════════
class TestSinglePopulationIsReadable:
    """⭐ S2 生效的指紋是「指令的**缺席**」+ 單一母體 ⇒ 成功在報表上天生會長成
    一排警語。若成功長得像故障,下一個人會去修一個沒有壞的東西。"""

    @pytest.fixture()
    def out(self, sandbox):
        return run_report(straight_batch(sandbox))

    def test_no_bare_zero_over_zero(self, out):
        """⚠️⚠️ 空的一群**絕對不可以**印裸的 `0/0`。

        兩種「空」的意義完全相反:硬彎組空 = S2 生效(預期);直進組空 = 全面回歸。
        印成同一個 `0/0` 會讓**最好**的結果與**最壞**的結果長得一模一樣,
        而且與腳本自己「σ_pre 全算不出來就中止、不印 0/0」的規則自相矛盾。
        """
        assert bare_zero_over_zero(out) == [], (
            "報表裡出現裸的 0/0 ⇒ 「這一群沒有飛機」被印成「命中 0」:%s"
            % bare_zero_over_zero(out))

    def test_the_empty_group_says_why_it_is_empty(self, out):
        """硬彎組空要明講是 S2 生效**預期**的形狀,不是命中 0。"""
        assert "本批**沒有這一群**" in out
        assert "這正是 S2 生效時**預期**的形狀" in out
        assert "這不是「命中 0」" in out

    def test_empty_hard_and_empty_straight_read_differently(self, sandbox):
        """⚠️ 反向:直進組空 = **全面回歸**,措辭必須與硬彎組空完全不同。"""
        acs = [{"lead": 60.0 + 5 * i, "miss": 42.0 + i, "req": 1300.0, "cmd": 30.0}
               for i in range(2)]
        trials = [sandbox.add_trial(f"SRG{i}", acs) for i in range(1, 4)]
        out = run_report(trials)
        assert "直進組  n =  0" in out
        assert "這是**全面回歸**" in out, (
            "直進組空印成了跟硬彎組空一樣的文字 —— 最壞的消息被印成最好的消息")
        assert "這正是 S2 生效時**預期**的形狀" not in out, (
            "全面回歸被套上了「S2 生效預期形狀」的措辭 —— 兩種空的意義完全相反")
        assert bare_zero_over_zero(out) == []

    def test_no_pseudo_grouping_from_noise(self, out):
        """⚠️⚠️ 最大間隙沒過 GAP_MIN_DEG 就是**沒有群內結構**,不可以照著雜訊講故事。

        舊版正是在「跨門檻間隙算不出來」的警語正下方,拿雜訊等級的最大間隙
        (實測 1.60°)印出「把 11 架切成 3 / 8」的兩群敘述 —— 而**真的**有兩群時
        (RRD gap=32.70°)這段反而不印。
        """
        assert "沒有群內結構" in out
        assert "這是一段**連續分布**" in out
        assert not re.search(r"切成 \d+ 架", out), (
            "最大間隙只有雜訊等級,報表卻印出了「切成 N 架 / M 架」的偽分群結構")

    def test_a_real_gap_is_still_reported(self, sandbox):
        """⚠️ 反向鎖:防雜訊的護欄不可以把**真的** ≥10° 間隙一起吃掉。

        單一母體**內部**確實有大間隙時要照印,但措辭要講清楚那不是分群門檻兩側的間隙。
        """
        acs = [{"lead": 60.0 + 5 * i, "miss": 42.0 + i, "req": 1300.0, "cmd": 30.0}
               for i in range(2)]
        t1 = sandbox.add_trial("SRG1", acs)
        acs2 = [{"lead": 100.0 + 5 * i, "miss": 50.0 + i, "req": 1300.0, "cmd": 30.0}
                for i in range(2)]
        t2 = sandbox.add_trial("SRG2", acs2)
        out = run_report([t1, t2])
        assert "**單一母體內部**的結構" in out, (
            "單一母體內部有 ≥10° 的真間隙,卻被當成雜訊吃掉了")
        assert "不可以拿它當「這批其實有兩個母體」的證據" in out

    def test_median_error_line_survives_a_one_sided_batch(self, out):
        """⚠️⚠️ 舊版 `if mh and ms:` ⇒ **任何**單邊母體都會讓誤差中位整條靜默消失
        (沒有替代輸出、沒有警語、也不進保留欄)。這一行必須無條件存在。"""
        assert "誤差中位:" in out, "單邊母體時「誤差中位」整條消失了"
        assert "硬彎 (本批沒有這一群)" in out
        assert "全距 硬彎 — /" in out
        assert "誤差中位只有單邊" in warn_line(out)

    def test_consistency_check_does_not_stamp_two_empty_sets(self, out):
        """⚠️⚠️ 空集合 == 空集合 也成立 ⇒ 舊版會對兩個空集合蓋「互相佐證 ✅」。

        而單一母體正是 S2 修好之後**應該**看到的樣子 ⇒ 決定性讀數正上方會蓋一個
        零證據力的 ✅。這是交接文件警告過的假綠燈形狀,只是換成了判準一致性。
        """
        assert "✅ 與 σ_pre 判準指認**完全同一組**" not in out, (
            "兩個判準都指認 0 架,報表卻蓋了「互相佐證 ✅」的章(零證據力的假綠燈)")
        assert "**沒有鑑別力**" in out
        assert "空集合對空集合,不是互相佐證" in out
        assert "一致性檢查無鑑別力" in warn_line(out)

    def test_there_is_a_positive_verdict(self, out):
        """⭐ 沒有任何一句**正面**斷言的話,S2 成功會被讀成「判讀壞了」。"""
        assert "✅✅ 【S2 生效】" in out, (
            "契約宣告開 + 實測致動全過,報表卻沒有任何一句正面結論 ⇒ "
            "成功長得像故障,下一個人會去修一個沒壞的東西")
        assert "實測致動 wait 段全程只要求 goto_radius **12/12 架**" in out
        assert "**不是判讀壞掉**" in out

    def test_the_verdict_comes_last(self, out):
        """⚠️⚠️ 保留欄不可以是報表的最後一句話 —— 讀者掃到最後只會看到一排警語。"""
        tail = last_line(out)
        assert tail.startswith("── 結論:**S2 生效**"), (
            "報表的最後一句不是結論而是 %r ⇒ 保留欄殿後會讓成功被讀成失敗" % tail[:40])
        assert out.index("【本次判讀的保留】") < out.index("── 結論:")


# ══════════════════════════════════════════════════════════════════════════
#  4. 混合批次:契約說開,CSV 卻沒有 S2 儀表欄
# ══════════════════════════════════════════════════════════════════════════
class TestMixedBatchRaisesTheAlarm:
    """⭐ 舊版把這個對帳關在 `if not any(have_s2.values()):`(= **整批**都沒儀表)
    的分支裡 ⇒ **混合批次完全不觸發**。而 RRE 是 8 趟 = 正是混合批次,
    「其中一趟跑到舊 binary / 參數沒吃到」是本專案反覆踩到的形狀。"""

    @pytest.fixture()
    def out(self, sandbox):
        acs = [{"lead": 1.0 + i, "miss": 1.0 + i, "req": 30.0, "cmd": 30.0}
               for i in range(2)]
        t1 = sandbox.add_trial("SMX1", acs)
        t2 = sandbox.add_trial("SMX2", acs, s2_cols=False)   # 契約仍宣告開
        t3 = sandbox.add_trial("SMX3", acs)
        return run_report([t1, t2, t3])

    def test_ghost_trial_is_named_and_loud(self, out):
        assert "契約說開關**已開**,但這些趟次的 CSV **沒有 S2 儀表欄**" in out, (
            "混合批次沒有觸發「契約宣告了、節點沒寫出來」的對帳 —— "
            "這正是 RRE 最可能的部分失效")
        assert "SMX2" in section(out, "【S2 機制生效證明】")
        assert "這幾趟**不可以**與其餘趟次合併統計" in out

    def test_ghost_trial_enters_the_reservations(self, out):
        w = warn_line(out)
        assert "契約說開關已開但 CSV 沒有 S2 儀表欄" in w and "SMX2" in w, (
            "報警了但沒有進保留欄 ⇒ 只讀最後兩行的人會漏掉它")
        assert "批次混了有/沒有 S2 儀表的趟次" in w

    def test_deployment_failure_is_not_worded_as_a_bad_sortie(self, out):
        """⚠️⚠️ 「沒有儀表欄」= 部署沒生效;「有儀表但沒放行」= 航次問題。
        舊版把兩者併成同一句良性措辭 ⇒ 前者被後者的語氣稀釋掉。"""
        assert "這是**部署沒生效**的形狀" in out
        assert "不是「這一趟飛得不好」。先查部署。" in out

    def test_the_batch_is_not_declared_effective(self, out):
        """有幽靈趟次時**不可以**下「S2 生效」的結論,也不可以拿通過的幾趟代表整批。"""
        assert "❌ 【S2 生效未獲證明】" in out
        assert "**不可以**拿通過的那幾趟代表整批" in out
        assert last_line(out).startswith("── 結論:**S2 生效未獲證明**")


# ══════════════════════════════════════════════════════════════════════════
#  5. 契約 vs 致動一律逐趟計數
# ══════════════════════════════════════════════════════════════════════════
class TestContractCheckIsPerTrial:
    """⚠️⚠️ 舊版用**整批**累加的 n_on/n_off 去對**逐趟**的契約,兩個方向都致命:

      · 假綠燈 —— 被污染的對照組(契約說關、資料卻顯示開關生效)單獨跑會報警,
        一旦與別趟併批就完全靜音(連保留欄都乾淨)。而對照組是整個宣稱的地基。
      · 假紅   —— 最自然的 A/B 批次(開關組 + 關閉組一起跑)必定對每個開關組趟次
        誤報「契約與實際致動不一致」,把人送去查不存在的問題。

    而腳本自己的紀律 5 寫著「這支腳本吃的是批次」⇒ 失效的正是**唯一被授權的用法**。
    """

    def test_contaminated_control_still_screams_after_merging(self, sandbox):
        """契約說**關**、資料卻全段 goto(= 被污染的對照組)⇒ 併批之後仍然要報警。

        ⚠️ 這裡刻意與一個**乾淨**的對照組(PC2:契約關、資料真的要求了 hold_radius)
           併批 —— 只有這樣才會踩到舊版的病灶:整批累加的 n_off 被 PC2 撐成非零,
           於是 PC1 的「契約說關卻看不到 hold_radius」整條靜音,連保留欄都乾淨。
        """
        dirty = [{"lead": 1.0 + i, "miss": 1.0, "req": 30.0, "cmd": 30.0}
                 for i in range(2)]
        clean = [{"lead": 60.0 + 5 * i, "miss": 42.0, "req": 1300.0, "cmd": 30.0}
                 for i in range(2)]
        t_bad = sandbox.add_trial("PC1", dirty, wait_goto_radius="0.0")   # 被污染
        t_ok = sandbox.add_trial("PC2", clean, wait_goto_radius="0.0")    # 乾淨對照
        out = run_report([t_bad, t_ok])
        assert "契約說**關**,但**本趟**沒有任何機體要求過 hold_radius" in out, (
            "被污染的對照組與別趟併批之後就靜音了 —— "
            "而對照組正是「直進 10/11 vs 硬彎 0/5」整個宣稱的地基")
        assert "PC1 契約說關但資料看不到 hold_radius" in warn_line(out)

    def test_a_clean_ab_batch_is_not_falsely_red(self, sandbox):
        """A/B 併批(開關組全 goto + 對照組全 hold)**不得**產生任何契約不一致警語。"""
        on_acs = [{"lead": 1.0 + i, "miss": 1.0, "req": 30.0, "cmd": 30.0}
                  for i in range(2)]
        off_acs = [{"lead": 60.0 + 5 * i, "miss": 42.0, "req": 1300.0, "cmd": 30.0}
                   for i in range(2)]
        t_on = sandbox.add_trial("AB1", on_acs, wait_goto_radius="1.0")
        t_off = sandbox.add_trial("AB2", off_acs, wait_goto_radius="0.0")
        out = run_report([t_on, t_off])
        assert "契約與實際致動**不一致**" not in out, (
            "最自然的 A/B 批次被誤報成契約不一致 ⇒ 把人送去查不存在的問題")
        assert "契約說**關**,但**本趟**沒有任何機體要求過 hold_radius" not in out
        assert "契約說開但資料顯示未生效" not in warn_line(out)

    def test_per_trial_tally_is_printed_for_every_trial(self, sandbox):
        """逐趟計數必須逐趟印出來,而且整批總計要自我聲明「不可拿來對任一趟的契約」。"""
        on_acs = [{"lead": 1.0 + i, "miss": 1.0, "req": 30.0, "cmd": 30.0}
                  for i in range(2)]
        off_acs = [{"lead": 60.0 + 5 * i, "miss": 42.0, "req": 1300.0, "cmd": 30.0}
                   for i in range(2)]
        out = run_report([sandbox.add_trial("AB1", on_acs, wait_goto_radius="1.0"),
                          sandbox.add_trial("AB2", off_acs, wait_goto_radius="0.0")])
        assert "AB1: goto 2 / hold 0 / 不可判 0" in out
        assert "AB2: goto 0 / hold 2 / 不可判 0" in out
        assert "整批總計**只是總計**,不可拿來對任何一趟的契約" in out

    def test_a_pure_control_batch_cannot_prove_anything(self, sandbox):
        """全部契約都說**關**的批次 ⇒ 它最多是對照組,不可以拿來證明 S2 生效。"""
        off_acs = [{"lead": 60.0 + 5 * i, "miss": 42.0, "req": 1300.0, "cmd": 30.0}
                   for i in range(2)]
        out = run_report([sandbox.add_trial(f"CTL{i}", off_acs,
                                            wait_goto_radius="0.0")
                          for i in range(1, 3)])
        assert "沒有任何一趟的契約宣告 wait_goto_radius 為**開**" in out
        assert "✅✅ 【S2 生效】" not in out
        assert "無從證明生效" in warn_line(out)

    def test_s2_instrument_without_a_contract_value_is_flagged(self, sandbox):
        """⚠️ 有儀表卻沒有契約值 ⇒ 分不出它是開關組還是對照組,不可以拿來做 A/B。"""
        acs = [{"lead": 1.0 + i, "miss": 1.0, "req": 30.0, "cmd": 30.0}
               for i in range(2)]
        out = run_report([sandbox.add_trial("NCV1", acs, wait_goto_radius=None)])
        assert "卻**沒有契約值**可對帳" in out
        assert "不可以拿來做 A/B" in out
        assert "有 S2 儀表但契約缺 wait_goto_radius:NCV1" in warn_line(out)


# ══════════════════════════════════════════════════════════════════════════
#  6. 判定量是 req_radius_m,不是 cmd_radius_m
# ══════════════════════════════════════════════════════════════════════════
class TestVerdictReadsReqNotCmd:
    """⚠️⚠️ 成功的指紋是「指令的**缺席**」⇒ `cmd_radius_m` 是 hold-last-value,
    開關**開與關同樣都是 30** ⇒ 拿它當證明是假綠燈。"""

    @staticmethod
    def _wait_rows(req, cmd, n=50):
        return {"wait_rows": [{"req_radius_m": "%.1f" % req,
                               "cmd_radius_m": "%.1f" % cmd} for _ in range(n)]}

    def test_identical_cmd_gives_opposite_verdicts(self):
        """cmd 兩邊逐位元相同(都是 30),req 一個 30 一個 1300 ⇒ 判定必須相反。"""
        on_v, on_d = s2_readout.s2_probe(self._wait_rows(30.0, 30.0), 30.0, 1300.0)
        off_v, off_d = s2_readout.s2_probe(self._wait_rows(1300.0, 30.0), 30.0, 1300.0)
        assert "cmd=[30.0]" in on_d and "cmd=[30.0]" in off_d, (
            "前提壞了:兩邊的 cmd_radius_m 應該逐位元相同,假綠燈才成立")
        assert on_v == "✅ 全段 = goto_radius"
        assert off_v == "❌ 要求過 hold_radius", (
            "req_radius_m 跳到 hold_radius 卻沒被判成未生效 ⇒ "
            "判定量取到了 cmd_radius_m(hold-last-value)⇒ 假綠燈")
        assert on_v != off_v

    def test_end_to_end_the_same_cmd_column_does_not_flip_the_batch(self, sandbox):
        """整份報表層級的同一條鎖:兩批的 cmd 全是 30,結論必須一開一關。"""
        on_acs = [{"lead": 1.0 + i, "miss": 1.0, "req": 30.0, "cmd": 30.0}
                  for i in range(2)]
        off_acs = [{"lead": 1.0 + i, "miss": 1.0, "req": 1300.0, "cmd": 30.0}
                   for i in range(2)]
        out_on = run_report([sandbox.add_trial("RQ1", on_acs)])
        out_off = run_report([sandbox.add_trial("RQ2", off_acs)])
        assert "✅✅ 【S2 生效】" in out_on
        assert "✅✅ 【S2 生效】" not in out_off, (
            "wait 段要求的是 hold_radius,報表卻宣告 S2 生效 —— "
            "cmd_radius_m 在兩批裡都是 30,它沒有鑑別力")
        assert "❌ 【S2 生效未獲證明】" in out_off


# ══════════════════════════════════════════════════════════════════════════
#  7. req_radius_m 覆蓋率不足 ⇒ 不可判
# ══════════════════════════════════════════════════════════════════════════
class TestCoverageShortfallIsNotEffective:
    """⚠️⚠️ 「全段」是本腳本對 S2 生效的**核心斷言**,而它可能只有一個樣本在撐:
    wait 段 1013 拍裡只要有 1 拍寫了 30.0、其餘全空,舊版照樣印「✅ 全段」,
    而且細節字串 `req=[30.0]` 與 100% 覆蓋時**一模一樣**,看不出差別。"""

    def test_probe_refuses_below_the_threshold(self):
        seg = [{"req_radius_m": "30.0", "cmd_radius_m": "30.0"} for _ in range(5)]
        seg += [{"req_radius_m": "", "cmd_radius_m": "30.0"} for _ in range(95)]
        v, d = s2_readout.s2_probe({"wait_rows": seg}, 30.0, 1300.0)
        assert v.startswith("⚠️"), (
            "wait 段 100 拍只有 5 拍寫了 req_radius_m,卻宣稱「全段 = goto_radius」")
        assert "覆蓋不足" in v
        assert "全段" not in v
        assert "「全段」無從斷言" in d

    def test_single_sample_is_not_a_green_light(self):
        """只有 1 拍寫了 30.0 —— 舊版的細節字串與 100% 覆蓋時一模一樣。"""
        seg = [{"req_radius_m": "30.0", "cmd_radius_m": "30.0"}]
        seg += [{"req_radius_m": "", "cmd_radius_m": "30.0"} for _ in range(999)]
        v, d = s2_readout.s2_probe({"wait_rows": seg}, 30.0, 1300.0)
        assert not v.startswith("✅")
        assert "0.1%" in d, "覆蓋率沒有印出來 ⇒ 假綠燈與真綠燈的細節字串無從區分"

    def test_threshold_is_ninety_percent(self):
        assert s2_readout.REQ_COV_MIN == pytest.approx(0.90), (
            "覆蓋率門檻被改掉了 —— 這個數字決定「全段」這條核心斷言的可信度")

    def test_report_does_not_declare_effect_on_a_sparse_batch(self, sandbox):
        """報表層級:覆蓋率不足的批次不得出現「✅ 全段」,也不得宣告 S2 生效。"""
        acs = [{"lead": 1.0 + i, "miss": 1.0, "req": 30.0, "cmd": 30.0,
                "req_cov": 0.4} for i in range(2)]
        out = run_report([sandbox.add_trial("COV1", acs)])
        assert "✅ 全段 = goto_radius" not in out
        assert "覆蓋不足,不可判" in out
        assert "✅✅ 【S2 生效】" not in out, (
            "req_radius_m 覆蓋率只有 40%,報表卻宣告 S2 生效")


# ══════════════════════════════════════════════════════════════════════════
#  8. σ_pre 全部算不出來 ⇒ 中止,不印 0/0
# ══════════════════════════════════════════════════════════════════════════
class TestSigmaPreAllNaNAborts:
    """⚠️⚠️ σ_pre 算不出來的架次會被**靜默剔除**,成因包含「整個 lead_angle_deg
    欄不存在」。那時【★決定性讀數★】會印「硬彎 0/0 / 直進 0/0」——
    標題掛著★決定性讀數★卻在說「這批根本沒量到」,正是本專案定義的靜默失敗。"""

    @pytest.fixture()
    def blind_batch(self, sandbox):
        acs = [{"lead": 1.0, "miss": 1.0, "req": 30.0, "cmd": 30.0,
                "lead_col": False} for _ in range(2)]
        return [sandbox.add_trial(f"NAN{i}", acs) for i in range(1, 3)]

    def test_it_aborts_instead_of_printing_zero_over_zero(self, blind_batch):
        out, _exc = run_report_expect_abort(blind_batch)
        assert bare_zero_over_zero(out) == [], (
            "lead_angle_deg 整欄不存在,報表卻印出了 0/0 ——「這批沒量到」"
            "被印成「一架都沒打中」")
        assert "★決定性讀數★" not in out, "已經沒有量到任何 σ_pre,卻還印了決定性讀數"

    def test_the_abort_is_a_non_zero_exit(self, blind_batch):
        _out, exc = run_report_expect_abort(blind_batch)
        assert exc.code not in (0, None), (
            "中止卻是零退出 ⇒ 上游腳本(rep_interactive.sh / make_artifacts.sh)"
            "會把它當成功,靜默失敗")
        msg = str(exc)
        assert "σ_pre **全部**算不出來" in msg
        assert "那不是「命中 0」,是「這批沒量到」" in msg
        assert "本次判讀的保留" in msg, "中止訊息沒有帶上保留欄 ⇒ 線索全丟"

    def test_partial_loss_is_counted_not_swallowed(self, sandbox):
        """⚠️ 剔除幾架就要吵幾架:部分架次算不出 σ_pre 時不可以靜默縮小母體。"""
        acs = [{"lead": 1.0, "miss": 1.0, "req": 30.0, "cmd": 30.0},
               {"lead": 2.0, "miss": 2.0, "req": 30.0, "cmd": 30.0}]
        blind = [{"lead": 1.0, "miss": 1.0, "req": 30.0, "cmd": 30.0,
                  "lead_col": False} for _ in range(2)]
        out = run_report([sandbox.add_trial("MIX1", acs),
                          sandbox.add_trial("MIX2", blind)])
        assert "架放行了但 σ_pre **算不出來**" in out
        assert "MIX2/U1" in out and "MIX2/U2" in out
        assert "σ_pre 算不出來(未分群)" in warn_line(out)
        assert "整批    n =  4" in out, (
            "被剔出分群的架次必須仍留在整批分母內,否則命中率會被灌水")


# ══════════════════════════════════════════════════════════════════════════
#  9. 契約缺半徑 key ⇒ 警語 + 保留,不得靜默套 30/1300
# ══════════════════════════════════════════════════════════════════════════
class TestMissingContractRadiusIsLoud:
    """⚠️⚠️ 「讀不到」包含**契約檔存在但缺 key**:`con.get("goto_radius", FALLBACK)`
    那種寫法會靜默套一組來自**別批次**的常數(30/1300 是 RRD 的實測值),
    等於用錯尺規量新資料 —— 而 hold_radius 遲早要重調。"""

    @pytest.fixture()
    def out(self, sandbox):
        acs = [{"lead": 1.0 + i, "miss": 1.0, "req": 30.0, "cmd": 30.0}
               for i in range(2)]
        t = sandbox.add_trial("NOC1", acs, drop_contract_keys=("goto_radius",))
        return run_report([t])

    def test_missing_key_is_announced(self, out):
        assert "契約表缺 goto_radius" in out, (
            "契約檔存在但缺 goto_radius,腳本卻沒吭聲 ⇒ 靜默套上別批次的常數當尺規")
        assert "這一趟沒有可信的判定尺規" in out

    def test_the_fallback_itself_is_announced(self, out):
        assert "讀不到契約半徑 ⇒ 退回 goto_radius=30" in out
        assert "若這一趟不是這組值,它的 S2 判定會錯" in out

    def test_both_go_into_the_reservations(self, out):
        w = warn_line(out)
        assert "NOC1 契約缺 goto_radius" in w
        assert "NOC1 契約半徑讀不到" in w

    def test_a_missing_contract_file_is_also_loud(self, sandbox):
        """整份契約檔不存在時同樣要吵(退回值一樣是別批次的常數)。"""
        acs = [{"lead": 1.0 + i, "miss": 1.0, "req": 30.0, "cmd": 30.0}
               for i in range(2)]
        out = run_report([sandbox.add_trial("NOC2", acs, contract=False)])
        assert "讀不到契約半徑 ⇒ 退回" in out
        assert "NOC2 契約半徑讀不到" in warn_line(out)

    def test_inconsistent_rulers_across_trials_are_flagged(self, sandbox):
        """⚠️⚠️ 各趟 (goto, hold) 不一致時要吵:半徑不同的兩批本來就不是同一個實驗。"""
        acs = [{"lead": 1.0 + i, "miss": 1.0, "req": 30.0, "cmd": 30.0}
               for i in range(2)]
        t1 = sandbox.add_trial("RUL1", acs, goto_radius="30.0", hold_radius="1300.0")
        t2 = sandbox.add_trial("RUL2", acs, goto_radius="30.0", hold_radius="800.0")
        out = run_report([t1, t2])
        assert "各趟的 goto/hold 半徑**不一致**" in out
        assert "各趟契約半徑不一致" in warn_line(out)
