#!/usr/bin/env python3
"""stot_covariate.py — 跨航次比較前的共變量標準化。

【任務 D1】把「窗口長度 / 速度 / 高度 / 側偏」不同的航次,轉成可比的殘差指標。

問題:各航次的巡航窗口 604–1300 s、全隊地速 84–176 m/s、長機高度 3127–4601 m
都不同,而這些都會直接影響末端 slot 誤差。直接比末端誤差等於在比混雜因子。
更糟的是 P91 的「好成績」有一部分是閘門 bug 的副作用(側偏被壓在 ~0,
誘導阻力低)—— 那正是一個沒被記錄下來的共變量。

做法:
  1. 每趟抽出應變量 y 與共變量 X(全部從 show_*.csv / cmdlog 實測,不從設定檔抄)
  2. 用最小平方擬合 y ~ X(無外部相依,純 Python 正規方程 + 嶺回歸穩定化)
  3. 回報**殘差** = 實測 − 模型預測。殘差才是「排除混雜後這一趟的好壞」

⚠️⚠️ 統計紀律(硬性):
  - n 趟 < COVARIATE_MIN_N(共變量數 + 3)時**拒絕輸出係數**,只列原始值。
    5 個共變量配 6 趟資料是在硬湊,那種係數沒有意義。
  - 一律同時印 n。任何結論都標「初步觀察,待多次驗證」。
  - 殘差**只能用來排序**,不要當成「校正後的絕對誤差」。

用法:
    python3 stot_covariate.py P91 T1 U1 X1 Y1 Z1 BL1
    python3 stot_covariate.py --auto          # 掃 runs/ 內所有 show_*.csv
"""
import argparse
import csv
import glob
import math
import os
import statistics as st

EARTH = 111320.0
LEADER = 1
V_RANK_SIDE = {2: (1, +1), 3: (2, +1), 4: (1, -1), 5: (2, -1)}


def mlon(lat):
    return EARTH * math.cos(math.radians(lat))


def load_show(trial, runs):
    p = os.path.join(runs, f"show_{trial}.csv")
    if not os.path.exists(p):
        return None
    fr = {}
    with open(p, newline="") as fh:
        for r in csv.reader(fh):
            try:
                fr.setdefault(round(float(r[0]), 1), {})[int(r[1])] = (
                    float(r[2]), float(r[3]), float(r[4]))
            except (ValueError, IndexError):
                continue
    return dict(sorted(fr.items())) or None


def features(trial, runs, v_spacing=4000.0, v_angle=35.0):
    """→ dict(y=末端worst誤差, 共變量...) 全部由實測反推。"""
    fr = load_show(trial, runs)
    if not fr:
        return None
    ts = [t for t in fr if LEADER in fr[t]]
    if len(ts) < 20:
        return None
    span = ts[-1] - ts[0]

    # 長機航向(平滑)與累計轉角
    H = {}
    for i, t in enumerate(ts):
        j = i
        while j + 1 < len(ts) and ts[j + 1] - t < 6.0:
            j += 1
        if j == i:
            continue
        a, b = fr[ts[i]][LEADER], fr[ts[j]][LEADER]
        de, dn = (b[1] - a[1]) * mlon(a[0]), (b[0] - a[0]) * EARTH
        if math.hypot(de, dn) >= 1.0:
            H[t] = math.degrees(math.atan2(de, dn)) % 360.0
    if len(H) < 10:
        return None
    ht = sorted(H)
    turn = sum(abs((H[ht[k]] - H[ht[k - 1]] + 540.0) % 360.0 - 180.0)
               for k in range(1, len(ht)))

    # 每機:地速、航向、與長機航向的夾角、高度
    gs, dpsi, alt = {}, {}, {}
    for u in range(1, 7):
        s = sorted((t, fr[t][u]) for t in fr if u in fr[t])
        if len(s) < 10:
            continue
        v, d, al = [], [], []
        for i in range(1, len(s)):
            dt = s[i][0] - s[i - 1][0]
            if not (0.2 < dt < 5.0):
                continue
            dn = (s[i][1][0] - s[i - 1][1][0]) * EARTH
            de = (s[i][1][1] - s[i - 1][1][1]) * mlon(s[i][1][0])
            v.append(math.hypot(dn, de) / dt)
            hh = math.degrees(math.atan2(de, dn)) % 360.0
            k = min(H, key=lambda x: abs(x - s[i][0])) if H else None
            if k is not None:
                d.append(abs((hh - H[k] + 540.0) % 360.0 - 180.0))
            al.append(s[i][1][2])
        if v:
            gs[u], alt[u] = st.median(v), st.median(al)
            dpsi[u] = st.median(d) if d else float("nan")

    # 應變量:末端 60 s 的 worst slot 誤差(僚機 2–5)
    t_cut = ts[-1] - 60.0
    worst = 0.0
    for u in V_RANK_SIDE:
        errs = []
        for t in ts:
            if t < t_cut or u not in fr[t] or t not in H:
                continue
            L, p, h = fr[t][LEADER], fr[t][u], H[t]
            rank, side = V_RANK_SIDE[u]
            brg = math.radians(h + 180.0 + side * v_angle)
            s = (L[0] + rank * v_spacing * math.cos(brg) / EARTH,
                 L[1] + rank * v_spacing * math.sin(brg) / mlon(L[0]))
            errs.append(math.hypot((p[0] - s[0]) * EARTH,
                                   (p[1] - s[1]) * mlon(s[0])))
        if errs:
            worst = max(worst, st.median(errs))

    wing = [u for u in (2, 3, 4, 5) if u in gs]
    if not wing or LEADER not in gs:
        return None
    return {
        "trial": trial,
        "y_worst_err_m": worst,
        # ── 共變量 ────────────────────────────────────────────────────
        "x_window_s": span,
        "x_lead_gs": gs[LEADER],
        "x_wing_gs": st.median(gs[u] for u in wing),
        "x_lead_alt": alt.get(LEADER, float("nan")),
        # ⭐ 這一項是 P91「假好成績」的關鍵共變量:僚機實際偏離長機航向多少。
        #    偏航角直接決定橫向漂移率 v·sinΔ,不放進模型就會把它算成「隊形好」。
        "x_dpsi_deg": st.median(dpsi[u] for u in wing if dpsi.get(u) == dpsi.get(u)),
        "x_turn_deg": turn,
        "n_samples": len(ts),
    }


# ── 嶺回歸(純 Python,無外部相依)────────────────────────────────────
def ridge(X, y, lam=1e-3):
    n, m = len(X), len(X[0])
    A = [[sum(X[k][i] * X[k][j] for k in range(n)) + (lam if i == j else 0.0)
          for j in range(m)] for i in range(m)]
    b = [sum(X[k][i] * y[k] for k in range(n)) for i in range(m)]
    for i in range(m):                      # 高斯消去
        p = max(range(i, m), key=lambda r: abs(A[r][i]))
        if abs(A[p][i]) < 1e-12:
            return None
        A[i], A[p] = A[p], A[i]
        b[i], b[p] = b[p], b[i]
        for r in range(i + 1, m):
            f = A[r][i] / A[i][i]
            for c in range(i, m):
                A[r][c] -= f * A[i][c]
            b[r] -= f * b[i]
    w = [0.0] * m
    for i in range(m - 1, -1, -1):
        w[i] = (b[i] - sum(A[i][j] * w[j] for j in range(i + 1, m))) / A[i][i]
    return w


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trials", nargs="*")
    ap.add_argument("--runs", default=os.path.expanduser("~/fixwing_ws/runs"))
    ap.add_argument("--auto", action="store_true")
    a = ap.parse_args()

    trials = a.trials
    if a.auto or not trials:
        trials = sorted(os.path.basename(p)[5:-4]
                        for p in glob.glob(os.path.join(a.runs, "show_*.csv")))

    rows = [r for r in (features(t, a.runs) for t in trials) if r]
    if not rows:
        raise SystemExit("✗ 沒有可用的航次")

    cov = [k for k in rows[0] if k.startswith("x_")]
    print(f"══ 共變量標準化 ══  n = {len(rows)} 趟,共變量 {len(cov)} 個\n")
    hdr = f"{'trial':>6} {'worst_err':>10} " + " ".join(f"{c[2:]:>10}" for c in cov)
    print(hdr)
    for r in rows:
        print(f"{r['trial']:>6} {r['y_worst_err_m']:>10.0f} "
              + " ".join(f"{r[c]:>10.1f}" for c in cov))

    need = len(cov) + 3
    if len(rows) < need:
        print(f"\n⚠️ **拒絕擬合**:{len(rows)} 趟 < {need}(共變量 {len(cov)} + 3)。")
        print("   這麼少的樣本配這麼多共變量,係數只會在擬合雜訊。")
        print("   上表可以看,但**不要**從它推因果。先補航次數。")
        print("\n   可做的替代:固定其他條件只變一個變數的成對比較"
              "(A/B),那不需要回歸。")
        return

    # y 取 log(誤差是正值且跨數量級),共變量標準化
    ys = [math.log(max(r["y_worst_err_m"], 1.0)) for r in rows]
    mu = {c: st.mean(r[c] for r in rows) for c in cov}
    sd = {c: (st.pstdev([r[c] for r in rows]) or 1.0) for c in cov}
    X = [[1.0] + [(r[c] - mu[c]) / sd[c] for c in cov] for r in rows]
    w = ridge(X, ys)
    if w is None:
        print("\n⚠️ 設計矩陣退化(共變量共線),不輸出係數。")
        return
    print(f"\n標準化係數(對 log 誤差;每 +1 標準差的效果):")
    for c, coef in zip(cov, w[1:]):
        print(f"  {c:>14}  {coef:+.3f}   (1σ = {sd[c]:.1f})")
    print(f"\n{'trial':>6} {'實測':>10} {'模型預測':>10} {'殘差(log)':>11}  判讀")
    for r, xr, yv in zip(rows, X, ys):
        pred = sum(a_ * b_ for a_, b_ in zip(w, xr))
        res = yv - pred
        tag = "優於同條件" if res < -0.2 else ("劣於同條件" if res > 0.2 else "符合預期")
        print(f"{r['trial']:>6} {math.exp(yv):>10.0f} {math.exp(pred):>10.0f} "
              f"{res:>+11.3f}  {tag}")
    print("\n⚠️ 殘差只用來**排序**,不是「校正後的絕對誤差」。"
          "所有結論標為初步觀察,待多次驗證。")


if __name__ == "__main__":
    main()
