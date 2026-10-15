# `truth32.csv` / `traj/` —— RRD1-8 + RRE1-8 共 32 架次的實測真值

產生方式(可重跑,只讀 `runs/artifacts/`、只寫本目錄):

```
PYTHONIOENCODING=utf-8 python matlab/export_truth32.py
```

MATLAB 讀法:

```matlab
T = readtable('matlab/data/truth32.csv');          % 32×46
J = readtable('matlab/data/traj/RRE1_U1.csv');     % 逐拍
```

---

## 〇 三個來源,以及為什麼不是同一支

| 段落 | 來源 | 為什麼是它 |
|---|---|---|
| 放行狀態、兩個時鐘 | `scripts/research/ditcg/truth32.py` | 已對 `runs/_rrd_rows.json` 逐位元驗證 |
| **觸地量**(誤差/殘餘高度/觸地速度/判定) | `scripts/wsl/stot_score.py --json` | 在 AGL=0 **內插**穿越點;是 `batch_tally.py` 印的那個數字 |
| nz 夾制比例 | `scripts/research/ditcg/fidelity_long.py::replay_nz` | CSV 沒有這一欄,必須逐列重放 |

### ⚠️⚠️ 觸地誤差有兩個版本,`err_touch_m` 才是 canonical

`truth32.py` 的 `r_touch_m` 是「**首次 `agl<=0` 那一列**的水平距」,沒有內插。
在 155 m/s × 0.05 s tick 下,一格就走 7.8 m ⇒ 它量到的是**量化殘渣**:

| | RRE 16 架次中位 |
|---|---|
| `row_r_touch_m`(取樣值,無內插) | **4.79 m** |
| `err_touch_m`(AGL=0 內插,canonical) | **0.80 m** |

本檔兩個都留:`err_*` / `z_res_m` / `d1_min_m` 是內插值,`row_*` 前綴是取樣值。
**做任何精度結論一律用 `err_touch_m`**;`row_*` 只在你要複現舊分析時才用。

這個差別不是學術的 —— 它會改變結論:

```
spearman(nz_clip_frac, row_r_touch_m) = +0.577      (n=30, err<=100 m 主母體)
spearman(nz_clip_frac, err_touch_m)   = +0.935      (同一組 30 架)
```

`fidelity_long.py` 檔頭記的「本檔量到 +0.581,而交辦寫的是 +0.914,差異未解」
就是這一條:它拿的是**沒內插的** `r_touch_m`。用 canonical 值重算即與 +0.914 一致。

### ⚠️ `stot_score` 取的是「離紅心最近的那個」AGL=0 穿越點,不是第一個

`truth32.py` 取**首次**過零。兩者在本批 32 架次上都指向同一次穿越
(`closed_by` 全 = `touchdown`),但定義不同,所以 `t_touch_row_s` 與
`t_touch_interp_s` 不保證同一點 —— 實測差 ≤ 0.09 s(32 架次最大值)。

---

## 一 `truth32.csv` 欄位(32 列 × 46 欄)

### 身分

| 欄 | 單位 | 定義 |
|---|---|---|
| `trial` | — | `RRD1`…`RRE8` |
| `sysid` | — | MAVLink sysid(本批每趟 n=2 ⇒ 1 或 2) |
| `tag` | — | `<trial>-U<sysid>`,唯一鍵 |

### 放行狀態(取放行**當拍**那一列)

放行 = `mode` 首次 == `"dive"`。

| 欄 | 單位 | 定義 |
|---|---|---|
| `t_release_s` | s | 放行時刻,CSV 的 `t`(= `time.time()` 牆鐘,每趟自己的原點) |
| `r_rel_m` | m | 到紅心的**水平**距(等距圓柱投影,`EARTH=111320`,與 `tot_node.py:51` 同值) |
| `z_rel_m` | m | 放行高度 AGL(CSV `agl` 欄;海上目標 ⇒ 等同 AMSL) |
| `V_rel_ms` | m/s | **3D 地速** = `gs / cos(gamma)` |
| `gs_rel_ms` | m/s | 水平地速(CSV `gs`) |
| `gamma_rel_deg` | deg | 航跡角 γ,**上正下負**(CSV `gam_deg`) |
| `eps_rel_deg` | deg | **帶號**水平方位差 σ:速度水平投影 vs 水平 LOS。>0 = 速度在 LOS 右側 |
| `eps_spoken_rel_deg` | deg | Guo §II.A 口語版 `asin(cos γ · sin σ)`,**只當對照**(見 `truth32.py` 檔頭的自洽性判決:一律用 `eps_rel_deg`) |
| `lead_angle_csv_deg` | deg | CSV 自帶的 `lead_angle_deg`(節點算的,**無號**)。與 `abs(eps_rel_deg)` 對照用 |
| `R_slant_rel_m` | m | 斜距 `hypot(r, z)` |
| `dive_T_s` | s | 排定的觸地時刻(同 `t_release_s` 的牆鐘軸)= `t_release_s + t_rem0_s` |
| `t_rem0_s` | s | 放行當拍的 `dive_T − now`,由 `early_by_s + secs_to_target` 反推(油門律吃的就是它) |

⚠️ `eps_rel_deg` 的航向來自**位置有限差分**(窗半寬 4 列,不跨凍結空檔),
只取位移**方向**不除 dt ⇒ 免疫 `t` 的 0.1 s 量化。

### 真值 —— 時間

| 欄 | 單位 | 定義 |
|---|---|---|
| `t_touch_row_s` | s | 首次 `agl<=0` **那一列**的 `t`(取樣值) |
| `t_dive_wall_s` | s | 俯衝耗時(牆鐘)= `t_touch_row_s − t_release_s`。**32 架中位 41.20 s** |
| `t_freeze_s` | s | 該段內相鄰列時距 > 0.3 s 的總和 = 節點凍結時間 |
| `t_dive_sim_s` | s | 俯衝耗時(模擬鐘)= `t_dive_wall_s − t_freeze_s`。**中位 37.90 s** |
| `t_touch_interp_s` | s | AGL=0 內插出來的觸地時刻(`stot_score.impact_t_s`) |
| `t_dive_interp_s` | s | `t_touch_interp_s − t_release_s`(牆鐘;中位 41.20 s) |

⚠️⚠️ **報任何耗時/散布都必須說明用哪一個鐘。** 每趟俯衝夾著 1–2 次
> 0.5 s 的空檔,空檔內機體只移動 3.4–24.2 m(真飛 3.7 s @190 m/s 應該 ~700 m)
⇒ 模擬時鐘實質停住。`t_dive_sim_s` 是「飛機真的飛了多久」,
`t_dive_wall_s` 是「牆上的鐘走了多久」。RRE7/RRE8 四架 `t_freeze_s = 0`。

### 真值 —— 幾何(canonical,內插)

| 欄 | 單位 | 定義 |
|---|---|---|
| `err_touch_m` | m | ⭐ **觸地誤差**:軌跡穿過 AGL=0 的那一點到紅心的水平距 |
| `z_res_m` | m | ⭐ **殘餘高度**:水平最近通過點**當下**的 AGL(可為負)。⚠️ 它不是「最低高度」 |
| `d1_min_m` | m | 首航次窗內最近水平通過距(線段內插)。判 HIT 的量(門檻 300 m) |
| `agl_min_m` | m | ⭐ **真最低 AGL**:放行 → 首次過零(含)那一段的 `min(agl)`。本批 **32/32 都 ≤ 0 ⇒ 全部真的落水**(全距 −9.5 … 0.0 m) |

`z_res_m` 與 `agl_min_m` 是**不同的東西**,混用會憑空造出一個「高空掠過、
沒落地」的族群:RRD1-U2 的 `z_res_m = 42.84`(通過紅心時還高 42.8 m)
但 `agl_min_m = −4.1`(它落水了,只是落在下游 46 m)。
**判「有沒有命中」只看 `err_touch_m`;判「有沒有落地」只看 `agl_min_m`。**

### 真值 —— 幾何(取樣值,無內插;複現舊分析用)

| 欄 | 單位 | 定義 |
|---|---|---|
| `row_r_touch_m` | m | 首次 `agl<=0` 那一列的水平距(見 §〇 的警告) |
| `row_r_min_m` | m | 放行→過零段內**取樣點**的最小水平距 |
| `row_z_at_rmin_m` | m | 上式那一點的 AGL |
| `path_len_m` | m | 放行→過零的 3D 航跡長(逐列位移積分)。與 `R_slant_rel_m` 的比值 = 航跡比 |

### 真值 —— 速度

| 欄 | 單位 | 定義 |
|---|---|---|
| `V_touch_ms` | m/s | ⭐ **觸地總速**:內插觸地點的 `hypot(gs, vz)`,`vz` 由觸地那一對相鄰列的高度差推 |
| `gs_touch_ms` | m/s | 內插觸地點的水平地速 |
| `row_gs_touch_ms` | m/s | 過零**前**最後一列的 `gs`(取樣值) |
| `row_V_touch_ms` | m/s | 同上那一列的 `gs / cos(gamma)` |
| `row_gamma_touch_deg` | deg | 同上那一列的 γ |
| `gs_peak_dive_ms` | m/s | 只取 `mode == "dive"` 的取樣集合裡的地速峰值(排除 `expended` 的彈道自由墜落) |

⚠️ 驗收第④項(終端速度)用 `V_touch_ms`(沿航跡總速),不是地速。
⚠️ 已知 `dt = 0` ⇒ `vz` 被釘 0 的靜默退化(見記憶 `impact-speed-reads-groundspeed`);
本批 **18/32 架**的 `V_touch_ms` 與 `gs_touch_ms` 逐位元相同,那就是退化的指紋
(`vz` 被釘 0),**不是**真的水平觸地 ⇒ 這 18 架的 `V_touch_ms` 是**低估值**,
只毀驗收④,不影響 `err_touch_m`。要真總速請改用 `row_*` 那一組自行重算,
或把觸地區間的 `dt` 用 `traj/` 的 `t_dq`(去量化)重推。

### 真值 —— 重放重建(CSV 沒有這些欄)

`tot_*.csv` 不記 nz。以下四欄逐列重放 `tot_node.py::_pn_cmd` 2617-2626 的
**指令側** nz,取樣集合 = 放行索引 → 首次過零索引(不含過零列):

```
d      = hypot(dn, de)                    dn,de = 由本機指向紅心的水平向量
h      = agl                              R = hypot(d, h)
vh     = gs                               hdot = vh · tan(gamma)
V      = hypot(vh, hdot)
chi    = 位置有限差分的航跡方位角          vn,ve = vh·(cos chi, sin chi)
ddot   = −(dn·vn + de·ve)/d
sig_dot= (hdot·d − h·ddot)/R²
nz_raw = cos(gamma) − n_vert · sig_dot · V / g        n_vert = 5.0, g = 9.80665
夾制   = nz_raw < −1.0(記 lo)或 nz_raw > 3.0(記 hi)
```

這與 `sim3d.fly` 的 `nz_sat_frac` 是**同一條式子、同一個計數點**,兩側才有資格比。
⚠️ 這是**指令側**的重建;用實得 γ̇ 反推 nz 的版本 spearman 是 **−0.399**(方向就錯)。

| 欄 | 單位 | 定義 |
|---|---|---|
| `nz_clip_frac` | 比例 0-1 | ⭐ **nz 夾制比例** = (lo+hi)/n。主母體(`err_touch_m` ≤ 100 m,n=30)全距 **0.11 %–14.23 %**;兩架真脫靶是 33.90 %(RRE7-U2)與 51.25 %(RRD3-U2) |
| `nz_lo_frac` | 比例 0-1 | 只算下側(`nz_raw < −1.0`,要求更陡但權限不夠)。23/32 架非零,佔全部夾制的 **58.4 %** |
| `nz_hi_frac` | 比例 0-1 | 只算上側(`nz_raw > 3.0`,要求拉起但權限不夠)。19/32 架非零,佔 **41.6 %**。⚠️ 上側不可忽略 —— 誤差最大的 RRD3-U2 反而是**上側** 35.1 % vs 下側 16.1 %;主母體裡前三大(RRD1-U2 / RRD7-U1 / RRD4-U1)才是下側主導(11–13 % vs ≤0.8 %) |
| `nz_n` | 列 | 重放的取樣數(分母) |
| `climb_after_rel_m` | m | 放行後仍**爬升**的最大量。⚠️ 這是「模擬缺了什麼物理」的直接指紋:`sim3d.py` 一放行就開始下降,結構上生不出非零值 |
| `gam_min_deg` | deg | 放行→過零段內最負的 γ。⚠️ **不可拿它當判準**(終端俯角已騙過我們兩次) |

### 判定

| 欄 | 單位 | 定義 |
|---|---|---|
| `status` | — | `stot_score` 的首航次判定:`HIT`(`d1_min_m` ≤ 300 m)/ `MISS` / `ESCAPE` / `UNKNOWN` |
| `hit_10m` | 0/1 | `err_touch_m <= 10`。⚠️ **這才是 `batch_tally.py` 報的命中率**,與 `status` 的 300 m 門檻不同 |
| `escaped` | 0/1 | 放行後曾離目標 > 20 km |
| `closed_by` | — | 首航次窗的收窗規則:本批 32/32 全是 `touchdown` |
| `n_rows_dive` | 列 | 放行→過零(含)的列數 |

---

## 二 `traj/<trial>_U<sid>.csv` —— 逐拍軌跡

32 個檔,**共 31,192 列 / 2.50 MB ⇒ 未抽樣,保留原始 ~18-20 Hz 全取**
(`export_truth32.py` 的預算是 5 MB;超過才會改成每 0.2 s 一筆並在此註明)。

窗:**放行那一列** → 首次水平距 > 20,000 m(逃逸半徑)或資料末端。
⚠️ 本批 32 架**逃逸半徑從未觸發**(放行時就只有 ~6,140 m,之後單調遞減)
⇒ 實際都收在資料末端,因此**每個檔都含觸地後的尾巴**(41–725 列)。
要只看俯衝段就用 `post_touch == 0` 過濾。

| 欄 | 單位 | 定義 |
|---|---|---|
| `t` | s | CSV 原始 `t`(牆鐘,**量化到 0.1 s**,約 50% 相鄰列同戳) |
| `t_dq` | s | 去量化時間:同戳第 k 筆補 `+0.05k`(k 從 0 起)。⭐ 做微分/內插用這個 |
| `t_rel` | s | `t − t_release`(原始 `t`,未去量化) |
| `r` | m | 到紅心的水平距 |
| `z` | m | AGL(= CSV `agl`;海上目標 ⇒ 等同 AMSL) |
| `V` | m/s | 3D 地速 = `gs / cos(gamma)` |
| `gamma_deg` | deg | 航跡角,上正下負 |
| `gs` | m/s | 水平地速(CSV 原欄,**逐位元就是節點看到的**) |
| `mode` | — | `dive` / `expended`(本批窗內只有這兩種) |
| `lat`,`lon` | deg | WGS84 |
| `post_touch` | 0/1 | 1 = 已在首次 `agl<=0` **之後**(second-pass 污染區,見記憶 `second-pass-contamination`) |

⚠️ 幅值一律用 `gs` / `gamma_deg`,**不要自行對 lat/lon 差分**:CSV 的 `t`
量化 0.1 s,差分出來的速度會有 ±100% 的假抖動。要方向就用位移方向(不除 dt)。

---

## 三 自我驗證(`export_truth32.py` 每次執行都重算,不過就 exit 1)

```
RRE 命中率(err_touch_m <= 10 m) = 15/16          期望 15/16          ✅
RRE 觸地誤差中位                = 0.80 m          期望 0.79–0.80 m    ✅
  (只看命中的 15 架 = 0.79 m)
[對照] RRD 命中率 = 10/16、中位 3.25 m           與記憶 s2-fixed-hard-turns「10/16 → 15/16」一致
[對照] row_r_touch_m 的 RRE 中位 = 4.79 m        未內插,勿用
nz 夾制比例 0.11–51.25 %(中位 1.41 %)、全為 0 的 0/32
真最低 AGL −9.5 … 0.0 m、<= 0 的 32/32
```

順帶對上閘門那兩個數(這是本批**實測側**的離散度,模擬要打敗的就是它):

```
殘餘高度 sd(主母體 n=30) = 11.76 m       vs 交辦記的實測 11.773 m
spearman(nz_clip_frac, err_touch_m) = +0.935 (n=30)   vs 記憶的 +0.914
```
