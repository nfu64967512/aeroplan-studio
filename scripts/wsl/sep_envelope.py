#!/usr/bin/env python3
"""sep_envelope.py — 用實飛 dataflash 反推機體的**比剩餘功率(SEP)包絡**。

【任務 A1】回答一個問題,而且只用實測資料回答:
    「這具機體在 3100 m 平飛,到底能穩定飛多快?」

方法:能量狀態近似(Energy-State Approximation)
    能量高度   h_e = h + V_TAS² / (2g)          [m]
    比剩餘功率 SEP = d(h_e)/dt = V(T−D)/W        [m/s]
    SEP > 0 → 還有餘力加速或爬升;SEP = 0 → 該速度是**穩態平飛極限**。

⚠️⚠️ 關鍵方法學:**只取接近滿油門的樣本**。
    巡航中的 SEP≈0 只代表「當下油門下的平衡點」,不是**能力上限**。
    要量能力就得取 T≈T_max 的樣本,所以預設只收 throttle ≥ 90% × THR_MAX。
    覆蓋率不足時腳本會明講,不會硬給一條曲線。

⚠️ 用 **TAS** 不是地速也不是 EAS:能量是動能,動能吃真空速。
    σ 由高度算(ISA),EAS = TAS·√σ。3100 m 時 √σ ≈ 0.857。

⚠️ 統計紀律:每個 (高度帶, 速度) 格子都回報 n。n < MIN_N 的格子標成
    「樣本不足」而不是給一個看起來很確定的中位數。

用法:
    python3 sep_envelope.py --uav 1 --alt 3100 --band 200
    python3 sep_envelope.py --bin ~/fixwing_ws/sitl_run/uav1/logs/00000277.BIN
    python3 sep_envelope.py --uav 1 --thr-frac 0.0     # 不篩油門(看實際平衡點)
"""
import argparse
import bisect
import glob
import math
import os
import statistics as st
import sys

G = 9.80665
MIN_N = 8            # 格子樣本數下限;低於此只報 n,不報數值結論


# ── ISA 密度比 ─────────────────────────────────────────────────────────
def sigma(alt_m):
    """ISA 對流層密度比 ρ/ρ0。EAS = TAS·√σ。"""
    if alt_m < 11000.0:
        return max(1e-3, (1.0 - 2.25577e-5 * alt_m) ** 4.25588)
    return max(1e-3, 0.29708 * math.exp(-(alt_m - 11000.0) / 6341.6))


def eas_of(tas, alt_m):
    return tas * math.sqrt(sigma(alt_m))


def tas_of(eas, alt_m):
    return eas / math.sqrt(sigma(alt_m))


# ── dataflash 讀取 ────────────────────────────────────────────────────
def _first_attr(rec, names):
    for n in names:
        v = getattr(rec, n, None)
        if v is not None:
            return float(v)
    return None


def read_bin(path, verbose=True):
    """→ [(t, alt_m, tas, thr_frac)]，t 為秒。

    ⚠️⚠️ 本 build 的實際 schema(已對照 dataflash 確認,不要憑印象寫):
        CTUN: TimeUS NavRoll Roll NavPitch Pitch ThO RdO ThD **As AsT E2T** GU
              → **沒有 Alt**。ThO 是 0–1;E2T 是飛控自己算的 EAS→TAS 比。
        POS : TimeUS Lat Lng Alt **RelHomeAlt** RelOriginAlt
              → 高度用 RelHomeAlt(與節點的 position()[2] 同基準)。
    ⇒ 兩條訊息**時間對齊**後才組得出一筆樣本。
    ⚠️ 用 CTUN.E2T 而不是 ISA 公式:那是飛控當下實際採用的換算,
       和 TECS 的內部一致;自己套 ISA 會和飛控差一截而不自知。
    """
    from pymavlink import DFReader
    r = (DFReader.DFReader_binary(path) if path.endswith(".BIN")
         else DFReader.DFReader_text(path))
    pos, ctun = [], []
    while True:
        m = r.recv_match(type=["CTUN", "POS"])
        if m is None:
            break
        t = getattr(m, "_timestamp", None)
        if t is None:
            continue
        if m.get_type() == "POS":
            alt = _first_attr(m, ["RelHomeAlt", "Alt"])
            if alt is not None:
                pos.append((float(t), alt))
        else:
            aspd = _first_attr(m, ["As"])
            thr = _first_attr(m, ["ThO"])
            e2t = _first_attr(m, ["E2T"]) or 1.0
            if aspd is None or thr is None:
                continue
            ctun.append((float(t), aspd, thr if thr <= 1.5 else thr / 100.0,
                         e2t))
    if not pos or not ctun:
        if verbose:
            print(f"  {os.path.basename(path)}: POS {len(pos)} / CTUN "
                  f"{len(ctun)} —— 缺一不可", file=sys.stderr)
        return []
    pos.sort()
    pts = [p[0] for p in pos]
    out = []
    for t, aspd, thr, e2t in ctun:
        i = bisect.bisect_left(pts, t)
        i = min(max(i, 0), len(pos) - 1)
        if abs(pos[i][0] - t) > 0.5:      # 對不上就丟,不要內插硬湊
            continue
        out.append((t, pos[i][1], aspd * e2t, thr))
    if verbose:
        print(f"  {os.path.basename(path)}: POS {len(pos)} + CTUN {len(ctun)}"
              f" → 對齊 {len(out)} 筆", file=sys.stderr)
    return out


def sep_series(rows, smooth_s=3.0):
    """→ [(t, alt, tas, thr, sep)]。SEP = d(h + V²/2g)/dt,用 smooth_s 秒中央差分。

    ⚠️ 不要用逐筆差分:CTUN 是 10–25 Hz,逐筆差分的雜訊會淹掉訊號。
    """
    rows = sorted(rows)
    ts = [r[0] for r in rows]
    he = [r[1] + r[2] * r[2] / (2.0 * G) for r in rows]
    out = []
    for i, (t, alt, tas, thr) in enumerate(rows):
        j = bisect.bisect_left(ts, t + smooth_s)
        k = bisect.bisect_left(ts, t - smooth_s)
        if j >= len(ts) or k < 0 or j <= k:
            continue
        dt = ts[j] - ts[k]
        if dt < 0.5:
            continue
        out.append((t, alt, tas, thr, (he[j] - he[k]) / dt))
    return out


# ── 包絡萃取 ──────────────────────────────────────────────────────────
def envelope(samples, alt_c, band, thr_min, v_bin=5.0):
    """在 [alt_c−band, alt_c+band] 內,依 EAS 分箱統計 SEP。"""
    sel = [s for s in samples
           if abs(s[1] - alt_c) <= band and s[3] >= thr_min]
    bins = {}
    for t, alt, tas, thr, sep in sel:
        e = eas_of(tas, alt)
        bins.setdefault(int(e // v_bin) * v_bin, []).append((sep, thr))
    return sel, bins


def zero_crossing(rows):
    """由 (EAS, SEP中位數) 序列線性內插出 SEP=0 的速度。回傳 None 表示沒有跨零。"""
    rows = sorted(rows)
    for i in range(1, len(rows)):
        (v0, s0), (v1, s1) = rows[i - 1], rows[i]
        if s0 > 0.0 >= s1:
            return v0 + (v1 - v0) * (s0 / (s0 - s1))
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", action="append", default=[],
                    help="dataflash 檔;可重複。未給則用 --uav 取最新一份")
    ap.add_argument("--uav", type=int, default=1)
    ap.add_argument("--alt", type=float, default=3100.0, help="分析高度中心 (m)")
    ap.add_argument("--band", type=float, default=250.0, help="高度帶半寬 (m)")
    ap.add_argument("--thr-max", type=float, default=35.0,
                    help="機體的 THR_MAX(%%);用來定義「接近滿油門」")
    ap.add_argument("--thr-frac", type=float, default=0.90,
                    help="只取 throttle ≥ 此比例×THR_MAX 的樣本;0 = 不篩")
    ap.add_argument("--v-bin", type=float, default=5.0, help="速度分箱寬 (EAS m/s)")
    a = ap.parse_args()

    paths = a.bin
    if not paths:
        pat = os.path.expanduser(
            f"~/fixwing_ws/sitl_run/uav{a.uav}/logs/*.BIN")
        found = sorted(glob.glob(pat), key=os.path.getmtime)
        if not found:
            raise SystemExit(f"✗ 找不到 dataflash:{pat}")
        paths = [found[-1]]

    print(f"══ SEP 能量包絡 ══  高度 {a.alt:.0f} ± {a.band:.0f} m", file=sys.stderr)
    rows = []
    for p in paths:
        rows += read_bin(p)
    if not rows:
        raise SystemExit("✗ 沒有可用的 CTUN 樣本(欄位名可能不符,見 read_bin)")

    samples = sep_series(rows)
    thr_min = a.thr_frac * (a.thr_max / 100.0)
    sel, bins = envelope(samples, a.alt, a.band, thr_min, a.v_bin)

    # ── 覆蓋率誠實回報 ────────────────────────────────────────────────
    in_band = [s for s in samples if abs(s[1] - a.alt) <= a.band]
    print(f"\n樣本:總 {len(samples)}  高度帶內 {len(in_band)}  "
          f"其中 throttle ≥ {thr_min*100:.0f}%: {len(sel)}")
    if in_band:
        thrs = [s[3] * 100 for s in in_band]
        print(f"高度帶內油門分布(%%): p10={sorted(thrs)[len(thrs)//10]:.0f} "
              f"p50={st.median(thrs):.0f} p90={sorted(thrs)[9*len(thrs)//10]:.0f} "
              f"max={max(thrs):.0f}   (THR_MAX={a.thr_max:.0f})")
    if not sel:
        print("\n⚠️ 高度帶內沒有接近滿油門的樣本 —— **無法反推能力上限**。")
        print("   這一趟的資料只能給「當下油門的平衡點」,不能當成包絡。")
        print("   要量包絡請跑一段刻意全油門加速的測試,或用 --thr-frac 0 看平衡點。")
        return

    print(f"\n{'EAS':>7} {'TAS':>7} {'地速':>7} {'SEP_p50':>9} {'SEP_p90':>9} "
          f"{'thr_p50':>8} {'n':>5}")
    curve = []
    for v in sorted(bins):
        vals = bins[v]
        if len(vals) < MIN_N:
            print(f"{v:>7.0f} {'—':>7} {'—':>7} {'樣本不足':>9} {'':>9} "
                  f"{'':>8} {len(vals):>5}")
            continue
        seps = sorted(x[0] for x in vals)
        p50 = st.median(seps)
        p90 = seps[int(0.9 * (len(seps) - 1))]
        thr = st.median(x[1] for x in vals) * 100
        tas = tas_of(v + a.v_bin / 2, a.alt)
        print(f"{v:>7.0f} {tas:>7.1f} {tas:>7.1f} {p50:>9.2f} {p90:>9.2f} "
              f"{thr:>8.0f} {len(vals):>5}")
        curve.append((v + a.v_bin / 2, p50))

    print("\n(平飛時地速 = TAS;有風才會不同,SITL 預設無風)")
    vz = zero_crossing(curve)
    if vz is None:
        print("⚠️ SEP 在取樣範圍內**沒有跨零** —— 包絡落在資料涵蓋之外,不外推。")
        if curve:
            vbest, sbest = max(curve, key=lambda r: r[1])
            print(f"   目前最大 SEP {sbest:+.2f} m/s @ EAS {vbest:.0f}"
                  f"(TAS {tas_of(vbest, a.alt):.0f});SEP 仍為正 = 還有餘力加速。")
    else:
        print(f"⭐ **SEP=0 交點:EAS {vz:.1f} → TAS {tas_of(vz, a.alt):.1f} m/s** "
              f"@ {a.alt:.0f} m")
        print(f"   = 該高度的穩態平飛極限(此油門條件下)。")
    if curve:
        vbest, sbest = max(curve, key=lambda r: r[1])
        print(f"   最佳能量速度(SEP 最大):EAS {vbest:.0f},SEP {sbest:+.2f} m/s"
              f" —— 可與 best_climb_eas 對照。")

    print("\n⚠️ 單趟資料只能當**初步觀察**;要下設計結論請跨 ≥3 趟重跑本腳本"
          "並比對交點的離散度。")


if __name__ == "__main__":
    main()
