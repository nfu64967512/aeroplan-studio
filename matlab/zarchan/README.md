# `matlab/zarchan/` — reference implementations from Zarchan

Ports of the guidance listings in Paul Zarchan, *Tactical and Strategic Missile
Guidance*. Their value is that **they are known to be right**. They are
transcribed line by line and are not to be "improved"; every deviation is
listed in the header of the file it happens in, and again in the table below.

Source of truth for every line number quoted here is the local full text:

```
matlab/zarchan/_source/zarchan.txt      15,120 lines / 204,137 words
```

**Moved 2026-08-22, and it mattered.** This used to point at a session
scratchpad (`<scratchpad>/zarchan.txt`), which is temporary and gets collected
— every line number in this file was one garbage-collection away from being
unverifiable. It now lives next to the code that cites it. Four spot checks
(409, 650, 10887, 10930) resolve correctly at the new location.

`_source/` is **gitignored**: it is a copyrighted book, kept locally so the
citations stay checkable, not redistributed. Re-extract it from the PDF if it
goes missing. (The earlier "1,552,668 words" figure was the character count.)

Not NotebookLM. The notebook carries six sources with this book's title and
only that local file is the real book — the others are a Goodreads listing, SEO
pages, and one apparently LM-generated "Treatise". There is a recorded incident
of the notebook citing *our own project notes* back as external literature and
inventing numbers to go with them.

Run everything:

```
"C:/Program Files/MATLAB/R2025b/bin/matlab.exe" -batch ^
  "cd('C:/Users/joy46/Documents/Projects/aeroplan-studio/matlab/zarchan'); demo_reproduce"
```

Current status: **21 PASS, 2 expected FAIL, 0 unexpected FAIL**, ~1 s wall.
The two expected failures are pre-registered thresholds that were missed and
then *left failing on purpose* — see "Where the book does not reproduce" below.

---

## Files

| file | book location | line nos. in `zarchan.txt` | what it is |
|---|---|---|---|
| `pn2d.m` | Listing 2.1 | title 409, code 410–427, prose 428, Fig. 2.2 caption 431 | 2-D engagement, **True PN** `nc = N'·Vc·λ̇`, command **⊥ LOS** |
| `purepn2d.m` | Listing 30.1, `IGUID=1` branch | prose 10930, code from 10932, `ICHOICE` 10949, Figs. 30.2–30.5 at 10961 / 10980 | **Pure PN** `nc = VM·N'·λ̇`, command **⊥ velocity** (optionally ⊥ LOS), optional bias |
| `zem3d.m` | Listing 28.1 | title 10296, code 10297–10324, results 10326; law at 10231–10252 | 3-D **ZEM** law against a spiraling target |
| `tshape3d.m` | Listing A.6 | 2-D form 14414, title 14416, code 14417–14442, law 14446, ⊥V 14452, terminal angle 14454, cost 14463 | 3-D **trajectory shaping** with terminal pitch/yaw angle |
| `closedform.m` | Ch. 2 (+ Ch. 3 lag-miss set) | Ch.2 533–539, 549–559, 578–583; Ch.3 830, 883–921, 925–926 | closed-form `nc(t)` and miss, with and without a lag |
| `adjoint_miss.m` | Listing 3.1 | rules 651–659, method 682, result 751, title 756, code 757–763 | **adjoint**: miss vs *every* flight time in one run |
| `demo_reproduce.m` | — | — | reproduces the book's stated results and grades them |

`pn2d`, `zem3d`, `tshape3d`, `adjoint_miss`, `closedform` are in **feet /
ft·s**, exactly as the book. `purepn2d` takes and returns metric but its loop
runs in feet — see below.

---

## Applicability — which file may be used for what

Read this before quoting any number out of this directory at anyone.

### Assumptions, per file

| | lag | accel limit | speed constant | geometry |
|---|---|---|---|---|
| `pn2d` | **none** | **none** | no (⊥ LOS ⇒ \|V\| drifts) | nonlinear 2-D |
| `purepn2d` | **none** | ±`XNCLIM`, symmetric, on the command magnitude | yes when `ICHOICE=1`, by construction | nonlinear 2-D |
| `zem3d` | one first-order `TAU` | `XNCLIM` = 1e10, i.e. **none** in the nominal case | no | nonlinear 3-D |
| `tshape3d` | **none** | **none** | yes, by the ⊥V projection | nonlinear 3-D |
| `closedform` | none (Ch.2) / one `T` (Ch.3) | **none** | assumed | **linearised**, constant `Vc` |
| `adjoint_miss` | one `T` | **none** | assumed | **linearised**, constant `Vc` |

No file has gravity, drag, or thrust. Every one of them is a constant- or
drifting-speed point mass.

### How our vehicle differs, and what that voids

* **We saturate; none of these files do.** Our `nz = clip(cos γ + V·ġ/g, −1.0,
  3.0)` is asymmetric, is in load-factor space, and fires constantly. Measured:
  nz-clamp fraction vs touchdown error, Spearman **+0.538 … +0.914** depending
  on definition. Lateral roll clip is only 1.58% and is 0.000% on 29 of 32
  aircraft — the bottleneck is **longitudinal authority**.
* **We have a one-sided LOS floor and the book has no such thing.**
  `gam_cmd = min(gam_cmd, −atan2(h,d)·1.0)`, applied *after* the nz clamp.
  Zarchan imposes terminal angle as an **additive** term in all four places it
  appears — Ch. 24 line 8109, Ch. 28, Ch. 30 line 10887, App. A line 14446.
  A one-sided `min()` has **zero gradient on the active side**, so the loop is
  open there; an additive term keeps a signed error and pulls back. The floor
  fires on **43.1%** of PN samples overall and **89.8–99.7%** on the four
  failing aircraft.
* **Zarchan clamps the achieved state; we clamp the command.** `zem3d` line
  10310 drives the lag from the *unclamped* demand `AMiP` and only then clamps
  the state `AMi`. Our loop never sees the excess demand. Not interchangeable.
* **Two channels with different gains has no precedent here.** We fly
  `n_lat = 4.0` and `n_vert = 5.0`. Listing 28.1 is one vector law: line 10252
  says N′ is *a* constant (singular), line 10297 declares one `XNP=3.`, and
  `ZEMDOTRTM` is a single scalar over all three axes, so the axes are not
  separable at all.
* **Our lag is not one lag.** τ ≈ 1.50 s while the LOS floor is quiet; when it
  fires the identified lag inflates to 2.5–8.0 s. On top: a 1.75 s α lowpass,
  1.0 s PN lead, 20 Hz commands.

### Judge by residual altitude and touchdown error

Not by terminal dive angle — that has misled this project twice. The standing
relation is `err = 1.124 × residual_altitude + 1.99`, R² = 0.906, n = 30.
Nothing in this directory predicts that.

---

# ⚠️ `adjoint_miss.m` — READ THIS BEFORE YOU BELIEVE ITS CURVE ⚠️

> ## THE ADJOINT ASSUMES:
> ## 1. A **LINEAR** SYSTEM (line 650: *"For every linear deterministic system, there exists an adjoint system"*)
> ## 2. EXACTLY **ONE FIRST-ORDER LAG**
> ## 3. **NO SATURATION ANYWHERE**
>
> ## OUR SYSTEM VIOLATES **ALL THREE**.
>
> ## Saturation is not a correction to our behaviour — it **IS** our behaviour.

This is the single most misusable file here, because it produces a clean,
persuasive curve for a system that does not resemble ours.

**Which assumption we break, and how badly:**

* **No saturation — broken worst.** See the nz clamp and LOS floor above. An
  adjoint of a loop that is open 43% of the time (89.8–99.7% on the failures)
  is not an approximation of that loop; it is a different loop.
* **One lag — broken.** τ switches between ≈1.50 s and 2.5–8.0 s depending on
  whether the floor is firing. That is a structure change, not a time constant.
* **Linear — broken.** Superposition is the first thing saturation destroys,
  and superposition is the entire selling point of the adjoint (line 751:
  doubling the maneuver doubles the miss).
* **Linearised geometry — stretched.** Large-angle dive, stationary target,
  strong terminal-angle demand.

**The specific trap, with our numbers.** Our `tF/τ` looks comfortable — batch
median 16.4 apparent, 27.4 on the airframe τ. But the four aircraft that missed
by 37–46 m sit at **15.1 / 7.6 / 5.5 / 4.7**, three of them under 8. Read the
adjoint curve naively and you conclude "short flights, of course they miss."
The measured cause is that the LOS floor was firing on ~90–100% of their PN
samples, so they were barely doing PN at all. Same symptom, different
mechanism — and the adjoint **cannot represent the second one**, so it cannot
distinguish them. Do not let it adjudicate that question.

**What it is legitimately good for:**

1. A **floor / lower bound** — if the unsaturated linear loop already misses at
   some `tF/τ`, we certainly do.
2. **Trend and scaling** — miss ∝ τ² for target maneuver, ∝ τ for heading
   error (lines 925–926, verbatim: *"doubling the guidance system time constant
   quadruples the miss!"* vs *"only doubles the heading error miss"*). These
   survive as shape arguments.
3. **Validating other code** — it is a known-correct linear reference, which is
   exactly how `demo_reproduce` check B uses it.

---

## Errata — the source code wins, twice

### 1. Ch. 30 biased-PN bias sign (already known)

The prose prints `γF` **positive**; Listing 30.1 prints it **negative**:

```
BIASDEG=(-GAMFDEG*(XNP-1.)+XNP*XLAM*57.3-GAMDEG)/DELT
         ^ leading minus
```

The **listing** is authoritative (re-confirmed against the PDF page image). On
the nominal numbers the listing gives `BIAS = +5 deg/s`, the prose `−7 deg/s`;
+5 is the one that lofts the missile so it has altitude to dive out of, which
is the ~3 km apogee Fig. 30.2 actually draws. `purepn2d('BIASFORM','text')`
runs the prose variant if you want to see it fail.

### 2. Ch. 3, N′=5 heading-error miss, last term — **new**

Lines 919–921 print the last term as `+tF³/24T³`. It should be **minus**:

```
Miss/(VM·HE) |N'=5 = tF·e^(−tF/T)·(1 − 1.5·tF/T + tF²/(2T²) − tF³/(24T³))
```

Confirmed two independent ways, and `demo_reproduce` check B5 re-runs the
second one every time:

1. Partial-fraction expansion of the book's own `1 − H(s) = [s/(s+1/T)]^N'`
   (line 830) for `s³/(s+a)⁵` gives `t·e^(−at)·[1 − 1.5at + 0.5(at)² − (at)³/24]`.
2. Numerically against the verbatim Listing 3.1 adjoint at `XNP=5`:
   **minus** version max deviation **2.572e−3**, **plus** version **3.901e−1**
   — two orders of magnitude apart. The other five formulas all land at
   1.797e−3 … 3.874e−3, i.e. the integrator's own noise floor.

Unlike erratum 1 we have **no PDF page image** for this one, so we cannot say
whether it is an OCR fault or a typesetting fault in the book. The formula
itself is confirmed either way.

---

## Deviations from the listings

Complete list. Each is also flagged in its own file.

**Common to all ports**: wrapped in a function with name/value options whose
defaults equal the book's data statement; arrays preallocated then trimmed;
a runaway guard so a mis-specified case cannot hang MATLAB. None of these
touch a value.

**Operator repairs.** The PDF text layer destroys MATLAB comparisons — `,` is
`<` and `.` is `>`. So `while VC .= 0` → `while VC >= 0`, `if RTM , 1000` →
`if RTM < 1000`, and so on. Each file lists its own set. **These restorations
are inference, not source**, though they are forced by context.

**Per file, beyond the above:**

* `pn2d.m` — adds `GAMDEG = atan2(VM2,VM1)·57.3` to the 0.1 s log block, so the
  Fig. 2.2 claim can be measured. Instrumentation only; never fed back.
* `purepn2d.m` — keeps only the `IGUID=1` branch (`IGUID=0` lives in
  `../ref/zarchan_listing30_1.m`); adds `BIASFORM='none'` as the default,
  because "pure PN" in this project means unbiased; adds `RT2M` so the target
  can sit below the launch altitude (default 0 = the book exactly); logs `VMM`,
  which the printed listing drops even though the prose says missile velocity
  is written to the file and Fig. 30.5 plots it.
* `zem3d.m` — exposes the `RT3IC` sweep as a vector so it can be coarsened;
  default is the book's full `40000:-500:500`. The book's 0.1 s block computes
  `RT1K…RM3K` and stores nothing — dead code in the original, kept as such.
* `tshape3d.m` — adds `HSCALE` to coarsen the book's `H = 1e-4`; `LAUNCHLOGIC`
  is transcribed unchanged including its 0.1 s search grid and its stated
  10 s validity limit.
* `closedform.m` — not a listing; the formulas typeset in Ch. 2 and Ch. 3, with
  erratum 2 applied and `SIGN5='text'` retained to reproduce the printed form.
* `adjoint_miss.m` — nothing beyond the common items. Note Listing 3.1's
  corrector is written `X = (XOLD+X)/2 + .5*H*XD` where Listing 2.1 writes
  `.5*(XOLD+X+H*XD)`; algebraically identical, kept as each was printed.

### Units warning for `purepn2d.m`

Listing 30.1's prose (line 10930) says the units "have been converted to the
metric system". **The loop is in feet.** The initial conditions are written
`10000.*3.28` and `250.*3.28`, and `XNCLIM = 32.2*XNCLIMG`; only the 0.1 s log
block converts back. This is kept verbatim, because `3.28` (not `3.28084`) is
the book's constant and round-tripping through it is not the identity.
Arguments and returned fields are metric; the integration is not.

---

## `demo_reproduce.m` — what it checks and what it found

Each check states its acceptance rule **before** it runs. Nothing in the file
was tuned to make a check pass.

| check | book claim | result |
|---|---|---|
| **A** | Fig. 2.2 (line 431): larger N′ removes heading error faster; line 541: larger N′ needs more g at the start | **PASS ×3.** \|nc(0)\| 11.93 g → 14.77 g; peak excursion 1702.7 → 1444.2 ft; time to 10% of nc(0) 7.3 → 5.7 s |
| **B** | line 751: 3-g maneuver at tF = 1 s gives "nearly 12 ft"; long flights give "virtually zero miss" | **PASS ×5.** 12.0712 ft at tF = 1 s. Adjoint vs closed form ≤ 3.9e−3 for all six formulas. Erratum 2 confirmed at 150× separation |
| **C** | lines 10961 / 10980: ⊥LOS vs ⊥V — trajectories nearly identical, command profiles vastly different, ⊥LOS severe speed loss, ⊥V none | **PASS ×5, 1 expected FAIL.** tf 92.92 vs 48.64 s; peak \|nc\| 0.688 vs 2.698 g; vmin/vic **0.2631 vs 1.0000**; both γ_f = −90.00° |
| **D** | — | **PASS.** `purepn2d` and `../ref/zarchan_listing30_1` agree **bit for bit** (Δγ = 0.000e+00, Δmiss = 0.000e+00, 639 samples each) |
| **E** | line 10326: crossrange miss virtually zero; total miss virtually constant vs flight time | **PASS ×3, 1 expected FAIL.** median total miss 18.65 ft, std/mean **0.012**; crossrange 1.23 ft = 4.1% of total |
| **F** | Figs. A.16–A.18: terminal angles met, at the price of very large commanded acceleration | **PASS ×3.** terminal pitch −30.606° vs −30, yaw 20.437° vs +20, miss 0.033 ft |

### Where the book does **not** reproduce

Both are thresholds *we* invented, missed, and then left failing rather than
relaxing. Neither refutes the book; both qualify a claim the book made in prose
without a number.

**C1 — "the missile trajectories are nearly identical" (line 10961).**
Three metrics agree that they are *close but not that close*:

| metric | value | vs apogee 3.0019 km |
|---|---|---|
| max \|ΔAlt\| at equal downrange | 0.4728 km | 15.8% |
| symmetric Hausdorff distance | 0.3474 km | 11.6% |
| apogee difference (3.0019 vs 2.7978 km) | 0.2041 km | 6.8% |

Pre-registered bound was 10%. The vs-downrange metric is ill-conditioned in the
terminal dive (downrange nearly stationary while altitude collapses), which is
why Hausdorff is also reported — and it misses too. **Verdict: Fig. 30.2 is a
qualified reproduction.** Its substantive content is confirmed (both loft to a
~3 km apogee, both arrive at γ_f = −90.00°, tested as C1b); the literal phrase
"nearly identical" is not confirmed at 10%. Do not relax the bound to turn this
green.

**E1 — "the miss is virtually zero in the cross-range direction" (line 10326).**
Measured median 1.2336 ft against a pre-registered 1 ft. In hindsight the 1 ft
bound had no basis in the book at all — Zarchan attaches no number to
"virtually zero". The only scale the book supplies is its own Fig. 28.3 axis,
`axis([0 10 -30 30])`, against which 1.23 ft is **4.1%**. That source-grounded
version is stated separately as E1b (PASS). E1 itself is not relaxed.

### Two things worth knowing that the checks turned up

**The "long flight ⇒ zero miss" claim has a subtlety.** An earlier draft of
check B2 required the miss at `tF/τ = 10` to be under 1% of the miss at
`tF/τ = 1`. It failed: −0.5152 ft against 12.0712 ft, i.e. 4.27%. That is
**not** integration error — refining `H` from 0.01 to 0.0002 moves it only to
−0.5047 ft, and the exact closed form gives −0.5117 ft. The reason is that the
N′=4 maneuver-miss curve `x²e^(−x)(0.5 − x/6)` **crosses zero at x = tF/τ = 3**
and then has a negative lobe; `x = 10` is still on that lobe. The decay is real
but is not complete by `tF/τ = 10` — it needs ~20. If you are sizing against
"make the flight long compared to the time constant", 10 is not enough.

**App. A's 60 g is a plot clip, not a measurement.** Line 14436 is
`axis([00,10,00,60])`. Measured peak in `tshape3d` is **573,779 g** raw —
which is the `1/tgo²` singularity as `tgo → 0`, not the shaping demand. Peak
excluding the last 0.5 s is **53.62 g**. Quote the second number, and only
alongside line 14463's prose ("a very large commanded missile acceleration"),
which is what actually carries the claim.

---

## Traps that will bite a port into our code

* **⊥LOS vs ⊥V look identical in code and are not.** Ch. 24 nonlinear (line
  8352) projects ⊥ **LOS**. App. A (line 14452) projects ⊥ **velocity**.
  `zem3d` is ⊥ LOS; `tshape3d` is ⊥ velocity; `purepn2d` switches with
  `ICHOICE`. **We fly ⊥ velocity.** Check C measures what the choice costs:
  ⊥ LOS lost **74% of its speed** in the book's own nominal case.
* **`tshape3d`'s terminal-speed constant.** `VM1F/VM2F/VM3F` are computed
  **once**, before the loop, from the initial `VM` (line 14420). That is only
  self-consistent because the ⊥V projection makes `|VM|` exactly constant in a
  constant-speed point mass. We have thrust and drag, so `|VM|` at impact is
  not `|VM|` at release; copying it as-is biases the additive term
  systematically — the failure already on record as
  `predictor-underestimates-dive`. Recompute `VM*F` every step from a
  *predicted* terminal speed.
* **"No velocity loss at all" (line 10980) is a kinematic identity, not a
  physical result.** With `ICHOICE=1` the acceleration is perpendicular to the
  velocity by construction, so speed cannot change. Check C4 measures
  `vmin/vic = 1.0000`, which confirms the port, not the airframe. It does not
  transfer to a vehicle with drag.
* **`1/tgo` and `1/tgo²` singularities.** `tshape3d` and `zem3d` both diverge as
  `tgo → 0` whenever any residual remains. Any port must gate or floor `tgo`.
* **Miss resolution is `Vc × H_final`.** In `pn2d`, `H = 0.0002` s and
  `Vc ≈ 4000 ft/s` gives ~0.8 ft. The 0.460 / 0.251 ft misses check A reports
  are that floor, not physics. Do not read a sub-foot miss out of these files
  as real.
* **`N' = 2` is singular in the Ch. 2 maneuver formula.** L'Hôpital gives
  `nc = −2·ln((tF−t)/tF)·nT` (lines 555–557), but lines 558–559 say setting
  `N' = 2.01` or `1.99` gives approximately the same answer — so a port needs
  no special case. `closedform` returns both so you can check that yourself.

## Relationship to the rest of `matlab/`

This directory is self-contained except that `demo_reproduce` checks C and D
read `../ref/zarchan_listing30_1.m`, which holds the **full** Listing 30.1
(both `IGUID` branches) and was transcribed by a different workstream. It is
read, never written. If it is absent, C and D report SKIP and the rest still
runs. Check D exists precisely because two independent transcriptions of the
same listing ought to agree — if they ever stop agreeing, one of them is wrong
and neither should be fixed by eye.

Nothing here writes anywhere outside `matlab/zarchan/`.
