#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""build_table.py —— Guo (2020) DD-ITCG 的 5 維 FSV 查表建構器 + 查表器。

離線階段(Guo §III):掃 `(r, z, V, gamma, eps)` 的格點,每一格用 `sim3d.fly`
正向積分到觸地,存下 `t_go^PNG`。線上階段用
`e_tgo = t_go^PNG - t_go^req` 去驅動橫向偏置(§IV.C:**只加在橫向**)。

用法
----
    python build_table.py --preset small            # 6^5 = 7,776 點,秒級
    python build_table.py --preset ours             # 我們實測帶,17x15x14x19x19
    python build_table.py --preset small --out /path/tbl.npz
    python build_table.py --check /path/tbl.npz     # 重載 + 查表自洽檢查

═══════════════════════════════════════════════════════════════════════
⚠️⚠️ 三件事必須先講清楚,否則這張表會靜靜地騙人
═══════════════════════════════════════════════════════════════════════

### (1) 不能沿用 Guo 的邊界

| 維度 | 我們(32 架次放行實測) | Guo | 落在他界內 |
|---|---|---|---|
| r    | 6134 - 6147 m       | 0 - 62 km      | 32/32 |
| z    | 2274 - 3449 m       | 0.1 - 34.1 km  | 32/32 |
| **V**| **142 - 254 m/s**   | **400 - 2000** | **0/32** |
| gamma| -21.3 - +16.9 deg   | -90 - +30      | 32/32 |
| eps  | 1.3 - 81.0 deg      | 0 - 60         | 30/32 |

V **整批低於他的下界 2.8 倍**,而且他的機體是**無動力滑翔**(1600→781 m/s
一路衰減),我們是**有動力加速**(150→215)。⇒ 邊界、密度、油門律全部自己訂。

### (2) 「PNG」在我們身上必須先定義,否則 5 維不變性不成立

Guo §III.B 的不變性要求 `t_go^PNG` 只是 FSV 的函數。我們的
`v_req = clip(45, 210, R/t_rem)`、`t_rem = dive_T - now` **引用絕對時鐘**
⇒ 同一個 FSV 會因 t_rem 不同而有不同 t_go。
**本表把 PNG 定義成「PN + v_req 釘死 = pn_vmax = 210」**
(`sim3d.SimParams.throttle_law = "vmax"`),理由與後果見 `sim3d.py` 檔頭。
一句話:t_go^PNG = **最快可達時間** ⇒ `e_tgo = t_go^req - t_go^PNG >= 0`,
控制方向只有「拖慢/繞遠」,與 `tot_node._itcg_bias`「只能延後不能提前」一致。
⚠️ 這個選擇寫在 npz 的 `meta` 裡。**換了它整張表作廢。**

### (3) eps 用**帶號水平方位差**,並且只存 |eps|

`t_go` 對 eps 鏡像對稱(左偏 30 度與右偏 30 度航程一樣長)⇒ 表只存
`|eps| in [0, 90] deg`,線上查表取絕對值,符號只用在 `de_tgo/deps` 的方向。
⚠️ A2 曾提出「eps = arcsin(cos(gamma) sin(sigma)) 讓朝向/背對塌成同一格」的
   分支歧義。**改用運動學定義(eps = 帶號水平方位差)之後這個歧義消失**:
   |eps| < 90 = 趨近、|eps| > 90 = 遠離,是兩個不同的格子,不會相撞。
   本表**不涵蓋 |eps| > 90**(放行點 0/32 落在那裡),查表會回 NaN 而不是外插。

### (4) ⚠️⚠️ 失敗樣本一律標「不可判」,**絕不讓內插靜靜蓋過去**

論文對「模擬失敗怎麼辦」是 **NOT STATED**(§III.C 只說「有些樣本 PNG 模擬
會失敗」)。而我們的洞正好落在**陡俯衝區** —— 也就是實際工作的地方。
本表的作法:
  * `t_go` 存 NaN,另存 `valid`(bool)與 `fail_code`(0/1/2/3/4)。
  * `lookup()` 做多線性內插時,**只要 32 個角點裡有任何一個 invalid,
    整次查詢就回 NaN**,並在 `reason` 回報是哪一種失敗。
  * 查詢落在格點外一律回 NaN,**不外插**。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim3d                                                        # noqa: E402

#: ⚠️ 建表用的積分步長。`sim3d.selfcheck()` 已驗 dt = 0.05/0.02/0.005 的
#:    t_go 全距只有 **0.0143 s**,而閘門是 3.0 s ⇒ 數值雜訊小 200 倍,
#:    取 0.05(= 節點的 tick,零階保持與實機逐拍對齊)最快也最忠實。
TABLE_DT = 0.05

#: 格點預設。⚠️ 先跑 small 通,不要一開始就 10^7 點。
PRESETS = {
    # 每維 6 格 = 7,776 點。用來跑通管線與看失敗樣本落在哪。
    "small": {
        "r_m": np.linspace(1000.0, 8000.0, 6),
        "z_m": np.linspace(500.0, 3500.0, 6),
        "V_ms": np.linspace(120.0, 220.0, 6),
        "gamma_deg": np.linspace(-60.0, 15.0, 6),
        "eps_deg": np.linspace(0.0, 75.0, 6),
    },
    # 我們**實測帶**的生產格點 = 17 x 15 x 14 x 19 x 19 = 1,288,860 點。
    # 每一維的下界/上界都覆蓋 32 架次的實測值再各留一格餘裕:
    #   r 放行 6134-6147(release_at_ring ⇒ 幾乎是常數),但俯衝全程要走到 0
    #   z 放行 2274-3449;V 俯衝 124.8-206.5(取 100-230)
    #   gamma 放行 -21.3…+16.9、俯衝最陡 -52.0(取 -70…+20)
    #   |eps| 放行 1.3-81.0(取 0-90;>90 = 遠離,刻意不涵蓋)
    "ours": {
        "r_m": np.arange(0.0, 8001.0, 500.0),
        "z_m": np.arange(100.0, 3601.0, 250.0),
        "V_ms": np.arange(100.0, 231.0, 10.0),
        "gamma_deg": np.arange(-70.0, 21.0, 5.0),
        "eps_deg": np.arange(0.0, 91.0, 5.0),
    },
}
AXES = ("r_m", "z_m", "V_ms", "gamma_deg", "eps_deg")


def build(grid: dict, *, pn=None, sp=None, chunk=200_000, verbose=True):
    """掃格點,回傳 (t_go 5D, valid 5D, fail 5D, meta dict)。"""
    pn = pn or sim3d.PNParams()
    sp = sp or sim3d.SimParams(throttle_law="vmax", dt=TABLE_DT)
    if sp.throttle_law != "vmax":
        raise ValueError(
            "建表只准用 throttle_law='vmax' —— 其餘政策會打破 Guo §III.B 的 "
            "5 維不變性(v_req 引用絕對時鐘 dive_T)。見 sim3d.py 檔頭。")

    axes = [np.asarray(grid[k], dtype=np.float64) for k in AXES]
    shape = tuple(len(a) for a in axes)
    total = int(np.prod(shape))
    if verbose:
        print(f"格點 {' x '.join(str(s) for s in shape)} = {total:,} 點")
        for k, a in zip(AXES, axes):
            print(f"   {k:<10s} {len(a):3d} 格  {a[0]:8.1f} … {a[-1]:8.1f}"
                  f"  (間距 {a[1]-a[0] if len(a) > 1 else 0:.1f})")

    mesh = np.meshgrid(*axes, indexing="ij")
    flat = [m.ravel() for m in mesh]
    t_go = np.empty(total)
    valid = np.empty(total, dtype=bool)
    fail = np.empty(total, dtype=np.int8)

    t0 = time.time()
    for a in range(0, total, chunk):
        b = min(a + chunk, total)
        o = sim3d.fly(flat[0][a:b], flat[1][a:b], flat[2][a:b],
                      np.radians(flat[3][a:b]), np.radians(flat[4][a:b]),
                      pn=pn, sp=sp)
        t_go[a:b] = o["t_go"]
        valid[a:b] = o["valid"]
        fail[a:b] = o["fail_code"]
        if verbose:
            el = time.time() - t0
            print(f"   {b:>9,}/{total:,}  {100.0*b/total:5.1f}%  "
                  f"{el:6.1f}s  (剩約 {el*(total-b)/max(b,1):6.1f}s)")

    md = sim3d.meta(pn, sp)
    md.update({
        "axes": {k: list(map(float, a)) for k, a in zip(AXES, axes)},
        "axis_order": list(AXES),
        "table_dt": sp.dt,
        "png_definition": (
            "PN(N_lat=4, N_vert=5, pn_lead=1.0, LOS地板=1.0, nz in [-1,3]) "
            "+ 油門律 thr=clip(0,1, 0.36+0.030*(v_req-V)) 且 **v_req 釘死 210** "
            "⇒ t_go^PNG = 最快可達時間 ⇒ e_tgo = t_go^req - t_go^PNG >= 0"),
        "eps_convention": (
            "存 |eps|(deg),0-90。t_go 對 eps 鏡像對稱;"
            "|eps|>90(遠離)刻意不涵蓋,查表回 NaN 不外插"),
        "invalid_policy": (
            "失敗樣本 t_go = NaN,valid=False,fail_code 1=逾時 2=打過頭 "
            "3=速度掉到地板 4=NaN。lookup() 的 32 個角點只要有一個 invalid "
            "整次查詢就回 NaN —— 不讓內插蓋過去(論文對此 NOT STATED)"),
        "built_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "build_seconds": round(time.time() - t0, 1),
        "n_points": total,
        "source": "scripts/research/ditcg/build_table.py",
    })
    return (t_go.reshape(shape), valid.reshape(shape), fail.reshape(shape), md)


def save(path, t_go, valid, fail, md):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    np.savez_compressed(path, t_go=t_go, valid=valid, fail_code=fail,
                        meta=np.array(json.dumps(md, ensure_ascii=False)),
                        **{f"axis_{k}": np.asarray(md["axes"][k]) for k in AXES})
    return path


def load(path):
    z = np.load(path, allow_pickle=False)
    md = json.loads(str(z["meta"]))
    return {"t_go": z["t_go"], "valid": z["valid"], "fail_code": z["fail_code"],
            "axes": [z[f"axis_{k}"] for k in AXES], "meta": md}


def lookup(tbl, r_m, z_m, V_ms, gamma_deg, eps_deg):
    """5 維多線性內插。回傳 (t_go, reason)。

    ⚠️⚠️ 兩種情況一律回 **NaN**,不外插也不用最近鄰頂替:
        "out_of_range" —— 查詢點落在格點外
        "invalid_corner" —— 32 個角點裡有 invalid(模擬失敗的洞)
    Guo §III.C 只說「有些樣本會失敗」,**沒說怎麼處理**;我們的洞落在陡俯衝區,
    也就是實際工作的地方 ⇒ 讓內插蓋過去 = 在最需要誠實的地方說謊。
    """
    axes = tbl["axes"]
    q = [float(r_m), float(z_m), float(V_ms), float(gamma_deg),
         abs(float(eps_deg))]                     # ⭐ eps 取絕對值(鏡像對稱)
    lo_i, w = [], []
    for a, x in zip(axes, q):
        if x < a[0] or x > a[-1]:
            return float("nan"), "out_of_range"
        j = int(np.clip(np.searchsorted(a, x) - 1, 0, len(a) - 2))
        span = a[j + 1] - a[j]
        lo_i.append(j)
        w.append(0.0 if span <= 0 else (x - a[j]) / span)

    acc, wsum = 0.0, 0.0
    for c in range(32):
        idx, ww = [], 1.0
        for d in range(5):
            bit = (c >> d) & 1
            idx.append(lo_i[d] + bit)
            ww *= (w[d] if bit else 1.0 - w[d])
        if ww == 0.0:
            continue
        t = tbl["t_go"][tuple(idx)]
        if not tbl["valid"][tuple(idx)] or not math.isfinite(float(t)):
            return float("nan"), "invalid_corner"
        acc += ww * float(t)
        wsum += ww
    return (acc / wsum if wsum > 0 else float("nan")), "ok"


def report_failures(t_go, valid, fail, md, top=12):
    """失敗樣本比例 + **它們落在哪一個角落**(這比比例本身重要)。"""
    axes = [np.asarray(md["axes"][k]) for k in AXES]
    n = valid.size
    nbad = int((~valid).sum())
    print(f"\n== 失敗樣本 {nbad:,}/{n:,} = {100.0*nbad/n:.2f}% ==")
    names = {1: "逾時(t>t_max)", 2: "打過頭(r<-5000)",
             3: "速度掉到地板", 4: "NaN/Inf"}
    for c, nm in names.items():
        k = int((fail == c).sum())
        if k:
            print(f"   fail_code {c} {nm:<18s} {k:,} ({100.0*k/n:.2f}%)")
    if nbad == 0:
        print("   (無失敗樣本)")
        return
    bad = ~valid
    print("\n   失敗樣本沿各維的分佈(該維每個格點值的失敗率):")
    for d, (k, a) in enumerate(zip(AXES, axes)):
        ax = tuple(i for i in range(5) if i != d)
        fr = bad.sum(axis=ax) / np.prod([bad.shape[i] for i in ax])
        hot = np.argsort(fr)[::-1][:top]
        s = "  ".join(f"{a[i]:.0f}:{100*fr[i]:.0f}%" for i in sorted(hot)
                      if fr[i] > 0)
        print(f"   {k:<10s} {s if s else '(全零)'}")
    # 最極端的角落
    ii = np.argwhere(bad)
    print(f"\n   失敗樣本的座標包絡(min…max):")
    for d, (k, a) in enumerate(zip(AXES, axes)):
        v = a[ii[:, d]]
        print(f"   {k:<10s} {v.min():8.1f} … {v.max():8.1f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--preset", default="small", choices=sorted(PRESETS))
    ap.add_argument("--out", default=None)
    ap.add_argument("--check", default=None,
                    help="重載既有 npz 做查表自洽檢查,不建表")
    ap.add_argument("--chunk", type=int, default=200_000)
    a = ap.parse_args()

    if a.check:
        tbl = load(a.check)
        print(f"載入 {a.check}")
        print(json.dumps({k: v for k, v in tbl["meta"].items()
                          if k not in ("axes",)},
                         ensure_ascii=False, indent=1)[:2500])
        selfcheck_lookup(tbl)
        return

    out = a.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        f"ditcg_table_{a.preset}.npz")
    t_go, valid, fail, md = build(PRESETS[a.preset], chunk=a.chunk)
    report_failures(t_go, valid, fail, md)
    v = t_go[valid]
    print(f"\n== t_go 統計(有效樣本)==")
    print(f"   n {v.size:,}  中位 {np.median(v):.2f}  全距 {v.min():.2f}"
          f"…{v.max():.2f}  sd {v.std():.2f}")
    save(out, t_go, valid, fail, md)
    sz = os.path.getsize(out) / 1e6
    print(f"\n寫出 {out}  ({sz:.2f} MB,建表 {md['build_seconds']} s)")
    selfcheck_lookup(load(out))


def selfcheck_lookup(tbl):
    """查表自洽:格點上要精確命中;格點外要回 NaN;失敗洞要傳染。"""
    print("\n== 查表自洽檢查 ==")
    axes = tbl["axes"]
    ok = bad = 0
    rng = np.random.default_rng(0)
    for _ in range(300):
        idx = [int(rng.integers(0, len(a))) for a in axes]
        q = [float(a[i]) for a, i in zip(axes, idx)]
        got, why = lookup(tbl, *q)
        want = tbl["t_go"][tuple(idx)]
        if not tbl["valid"][tuple(idx)]:
            (ok if (math.isnan(got) and why == "invalid_corner") else bad)
            if math.isnan(got):
                ok += 1
            else:
                bad += 1
            continue
        if abs(got - float(want)) < 1e-9:
            ok += 1
        else:
            bad += 1
    print(f"   格點命中 {ok}/300 精確,{bad} 不合")
    lo = [float(a[0]) for a in axes]
    got, why = lookup(tbl, lo[0] - 1.0, lo[1], lo[2], lo[3], lo[4])
    print(f"   格點外查詢 → {got} / {why} "
          f"({'OK' if math.isnan(got) and why == 'out_of_range' else 'FAIL'})")
    # 中點內插要落在兩端之間(單調性抽查)
    a0 = axes
    mid = [(float(x[0]) + float(x[-1])) / 2 for x in a0]
    g, why = lookup(tbl, *mid)
    print(f"   中心點查詢 → t_go {g:.3f} s / {why}")


if __name__ == "__main__":
    main()
