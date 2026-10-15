function R = optimalGainsSingleLag(opts)
%MGS.ZARCHAN.OPTIMALGAINSSINGLELAG  Zarchan Listing 26.1 -- integrating the
%   matrix Riccati equation for the OPTIMAL CONTROL GAINS of a single-lag
%   flight control system.  Ch.26 is the one chapter that DERIVES a guidance
%   law rather than testing one; this is its entry point.
%
%   R = mgs.zarchan.optimalGainsSingleLag()
%   R = mgs.zarchan.optimalGainsSingleLag('H', 1e-3)            10x cheaper
%   R = mgs.zarchan.optimalGainsSingleLag('GAM', 1e-6, 'assert', false)
%
%   Line-by-line port of zarchan.txt:9359-9367.
%
%   UNITS -- the book's unit-free listing; no length ever appears.  tgo, TAU,
%   TF in SECONDS.  C1 [1/s^2], C2 [1/s]; C3, C4, N' DIMENSIONLESS.  GAM is
%   the weight in J = y(tF)^2 + GAM*int(nc^2)dt, hence s^3, which is why TAU^3
%   divides it everywhere below.
%
%   WHAT IT SOLVES.  x = [y, ydot, nT, nL]', u = nc, target maneuver assumed
%   CONSTANT; SF = diag([1 0 0 0]), B = GAM.  The change of variable t -> tgo
%   (:9263) turns the boundary-value Riccati into an initial-value one,
%   Sdot = F'S + SF - C'BC with S(0) = SF and C = B^-1 G'S, integrated forward
%   in tgo by the book's own second-order Runge-Kutta (Heun; :9356 says
%   "Second-order Runge-Kutta integration is used", and SELF-CHECK 6 measures
%   the order at 3.95-4.00 per step doubling, so that claim is the book's and
%   ours).  Gains are Ci = -C(1,i).
%
%   ⚠️ CITE LINE NUMBERS FROM zarchan.txt, NOT FROM zarchan_decoded.txt.  Every
%   ":NNNN" in this file is a RAW-file line, verified 2026-08-24.  The decoded
%   file is no longer line-aligned with the raw one (15307 lines against 15120),
%   so :9341 lands on the N' formula in the raw file and on a running head in
%   Ch.25 in the decoded one.
%
%   *** WHAT IT IS NOT. ***  Linearised, planar, constant closing velocity,
%   FIRST-ORDER lag, no gravity/drag/acceleration limit/LOS floor, constant
%   target maneuver.  Our gamma-tracking gain is 0.063 against a model 1.385
%   and the lag SATURATES; this is the clean baseline our failures are measured
%   against, not a drop-in gain table.
%
% --- TRANSCRIPTION.  No unresolved variable-to-variable comparisons: every
%   comparison in 26.1 is against a literal.  Three OCR repairs, all forced by
%   shape ("." is this book's ">", "," its "<"):  "while ~(T .= (TF-.0001))"
%   -> ">=" (T only increases from 0, so "<=" makes the negated test false on
%   entry and the loop never runs); "while STEP,=1" -> "<=" (the body sets
%   STEP=2 to leave); "if S1.=.009999" -> ">=" (S1 resets on every record, so
%   "<=" records forever).  Four book quirks: "save datfil.txt output /ascii"
%   is a typo for -ascii, and nothing is written to disk here, so it is
%   dropped; "W1=1./TAU" is computed and never read, dropped; the "FLAG=0"
%   after the inner loop is dead because the outer loop re-initialises it,
%   dropped; and the literal ".009999" pairs with the listing's own H=.0001 to
%   mean "record every 0.01 s", so this port tests against 'samp' -- keeping
%   the literal would silently change MEANING at any other H.  GT and GAMINV
%   are hoisted out of the inner loop; both are loop-invariant there, so this
%   is a no-op, stated so it is not mistaken for a restructuring.
%
% --- THE ONE THING TO KNOW BEFORE READING THE OUTPUT.  The book's "Formula"
%   curves are NOT the same function as its numerical curves, and the gap is
%   closed-form.  Both are
%
%       N'(x) = x^2 (e^-x + x - 1)/(Q(x) + d),   x = tgo/TAU
%       Q(x)  = int_0^x (e^-s+s-1)^2 ds
%             = .5 - .5e^-2x - 2x e^-x - x^2 + x + x^3/3
%
%   and 6*Q(x) is EXACTLY the denominator printed at :9341.  Only d differs:
%   the exact law has d = 0; the listing's XNPP has d = .0001/6 = 1.667e-5 (its
%   singularity guard, :9356); the INTEGRATION converges to d = GAM/TAU^3 =
%   1.000e-5.  That last is not a guess -- SF is rank one, so S stays rank one,
%   S(tgo) = [GAM/(W+GAM)] v v' with v = [1, tgo, tgo^2/2, -TAU^2(e^-x+x-1)]'
%   the zero-effort-miss vector and W = TAU^3 Q(x), giving C = -h v'/(W+GAM),
%   h = TAU(e^-x+x-1).  Three consequences, all used below:
%     a  The guard that WOULD match the integration is 6*GAM/TAU^3 = 6e-5, not
%        the 1e-4 the book chose.  The two agree to 0.022% at tgo = TAU and
%        diverge to a factor 5/3 as tgo -> 0 -- invisible on five semilogy
%        decades, which is why Figs. 26.2-26.6 look exact.
%     b  Below x ~ 0.2 the guard, not the physics, sets the curve: XNPP turns
%        over and collapses while the true N' diverges like 10/x.  Everything
%        here is scored on x >= x_lo (default 1).
%     c  C2/C1 = tgo, C3/C1 = tgo^2/2 and C4/C1 = -TAU^2(e^-x+x-1) hold for ANY
%        GAM and any guard, being ratios of v's components.  Sharpest rows here.
%   The printed denominator is also unusable for small x: it is 0.3x^5 computed
%   as a difference of terms of size 3, and it returns a plausible small number,
%   never a NaN.  Q is evaluated by series below x = 1; SELF-CHECK 5 pins the
%   two forms together on x >= 1.
%
% --- THE TRUNCATION ENVELOPE.  *** MEASURED 2026-08-24 on R2025b. ***  Every
%   identity in this file -- the gain ratios of consequence (c), S rank one,
%   the S11 closed form -- is exact for the EXACT Riccati solution and only
%   approximate for the integrated one.  So each needs a band, and the band is
%   this listing's own truncation error.  That error was measured, not assumed:
%     * ORDER 2, confirmed.  Doubling H multiplies every one of the five
%       residuals by 3.947-3.990 at the listing's own H = 1e-4, and by
%       3.999-4.000 at H = 1e-5.  (3.57-3.91 at H = 3e-4 and 1.98-3.26 at
%       1e-3, as the asymptotic regime is left; the low end of each spread is
%       the rank-one row.)  SELF-CHECK 6 re-runs the integration at 2H
%       and scores this, which is what turns "the residual is numerical" from a
%       claim into a measurement.
%     * It depends on H/TAU, GAM/TAU^3 and x ONLY.  Holding H/TAU = 1e-4 and
%       GAM/TAU^3 = 1e-5 while TAU = 0.5, 1, 2 gives 6.638e-4, 6.647e-4,
%       6.647e-4 on the C1 row.
%     * It is INVERSE in GAM across four decades: the C1 residual is 6.58e-6,
%       6.62e-5, 6.65e-4, 6.59e-3 at GAM = 1e-3, 1e-4, 1e-5, 1e-6.  That is the
%       1/GAM amplification in the right-hand side, and also the fact that the
%       SIGNAL these ratios are measured against, s = GAM/(TAU^3 Q + GAM),
%       decays like GAM/x^3 while the off-manifold error does not.
%     * It grows LINEARLY in x, and is a function of x alone -- TF only decides
%       how far the sweep goes.  A run to TF = 3 s and the first 3 s of a run
%       to TF = 100 s both give 9.3e-5.
%   So, with nd = (H/TAU)^2 * (TAU^3/GAM) and xh = max(1, max(x)),
%
%       C1, C3 residual <= 0.082 nd xh      rank-one minor  <= 0.165 nd xh
%       C4 residual     <= 0.083 nd         S11 closed form <= 0.372 nd xh
%
%   Those five constants are the SUPREMA over x >= 1 of the normalised residual
%   on sweeps to x = 100 at GAM = 1e-3, 1e-4, 1e-5, 1e-6 crossed with H = 3e-5,
%   1e-4, 3e-4; worst observed 0.0816, 0.0812, 0.0825, 0.1649, 0.3712.  Bands
%   are TWICE that, then capped (0.5% on the book rows, 1% on the structural
%   ones) and floored at 1e-6 -- see truncBand for why that floor is the
%   NO-CONTROL PLATEAU and not roundoff, and why it had to exist.
%   ⚠️ The C4 constant carries no
%   xh because that residual SATURATES -- 0.0812 nd at x = 10 and still
%   0.0812 nd at x = 100 -- which is why C4 was the one row of the four that
%   was already green.
%
%   ⭐ THE BAND THIS REPLACES WAS A GUESS, AND A WRONG ONE.  It read
%   max(0.01%, 100*(H/TAU)^2), its comment calling 100 "the square of the
%   fastest eigenvalue on window".  At the book's own H that expression returns
%   1e-6, the 0.01% floor takes over, and the port measures 6.6e-4 -- so two
%   identity rows and both structural rows were RED at the listing's own
%   settings, and had been since the port was written.  The effective constant
%   is not 100 but 0.082*(TAU^3/GAM)*xh = 8.2e3 at nominal; the factor no
%   eigenvalue argument was ever going to supply is 1/GAM.
%
%   ⚠️ THE LOOSER BAND STILL BITES, and that was checked by DELETION -- on THIS
%   FILE, edited in place and run, not on a paraphrase of it.  Four mutations,
%   each applied to riccatiRun below and then reverted (md5 checked back to the
%   byte):
%     F(2,4) -1 -> +1     red on C4 and BM6.  The other rows do not move: they
%                         are ratios that this sign does not enter.
%     F(4,4) -1 -> +1     red on rank one, S11, C1, C3, C4, BM6 and check 6.
%                         rank-one minor 0.9916 against a 3.3e-3 band.
%     SD = SFSFT - CTBC   red on SELF-CHECK 3 (S11, 2.438e+07 against 7.44e-3)
%       -> SD = SFSFT     AND on BM6 (2.4e+09% against 0.5%).  ⚠️⚠️ EVERY
%                         OTHER ROW GOES GREENER: dropping the quadratic term
%                         makes S exactly rank one, so C1/C3/C4 fall to 2e-8
%                         and check 6 reads a perfect 4.000 on all four.
%                         ⭐ CORRECTED 2026-08-24 (audit re-ran the mutation on
%                         this file): an earlier revision of this block said
%                         "red on S11 alone" and concluded that deleting
%                         SELF-CHECK 3 would let the mutation ship undetected.
%                         It would not -- BM6 also goes red, by nine orders.
%                         Keep check 3 anyway: it is the only row that names
%                         WHICH term went missing, and BM6 red alone reads as
%                         "the N' curve is wrong" rather than "the quadratic
%                         term is gone".
%     SFT = SF' -> SF     red on FOUR of the six structural checks (1, 2, 3, 6)
%                         plus four benchmarks; symmetry catches it first at
%                         |S - S'| = 200.  ⭐ CORRECTED 2026-08-24: an earlier
%                         revision said "all six".  Checks 4 (S11 monotone) and
%                         5 (printed denominator == 6Q) stay GREEN under it --
%                         4 because S11 still falls, 5 because it never touches
%                         S at all.
%   The old 1e-5 / 1e-4 tolerances bought no detection power over these bands
%   whatsoever -- they only bought a false red at the book's own settings.
%   SELF-CHECK 6 pins the same faults from the other side: truncation gives a
%   2H ratio of 4.0, a wiring fault gives 1.0, NaN, or a residual so large the
%   check refuses to score it at all.
%
%   ⚠️⚠️ WHAT NO BAND IN THIS FILE CAN SEE, MEASURED 2026-08-24.  The bands
%   are the listing's own truncation error, so at the book's own H = 1e-4 they
%   are ~1.6e-3 wide, and NOTHING structurally smaller than that shows up in
%   them.  Deletion, on this file:
%     F(2,3) 1 -> 1.0002    (0.02%)  every band GREEN -- the C3 row even gets
%                           GREENER, 6.379e-4 -> 4.380e-4.  Caught ONLY by
%                           SELF-CHECK 6, at a 2H ratio of 5.348 against the
%                           [1.8, 5.0] band.  That row is not decoration.
%     F(2,3) 1 -> 1.00005   (0.005%) the WHOLE PORT goes green, check 6
%                           included (ratio 4.239).
%   So the honest detection floor for a coefficient typo at the listing's own
%   step size is between 5e-5 and 2e-4 relative, and self-check 6 is the last
%   thing standing before it.  A tighter floor is not available by tuning a
%   band -- it needs a smaller H, and the CAP in truncBand is what forces that
%   choice to be made deliberately.
%
% --- BENCHMARKS.  Every book value was grepped; the line is in .source.  Figs.
%   26.2-26.6 became image links and publish NO number, so :9373's "near
%   perfect agreement" is scored as agreement between two things this port
%   computes -- the honest version of the claim, and a sharper one.
%     1  N' -> 3 for long tgo (:2526).  A LIMIT claim, so scored on the formula
%        at x = 1e4 (3.0006); at the sweep's own x = 10 it is 3.69.  Band 0.5%.
%     2  N' grows as tgo -> 0 (:2526).  SHAPE: strictly decreasing on [0.1,10]
%        with N'(0.1)/N'(10) >= 10; the closed form gives 27.7.
%     3  C1 = N'/tgo^2 on the INTEGRATED gains (:9347 with :9349).
%     4  C3 = 0.5 N' on them (:9351).
%     5  C4 = -N'(e^-x+x-1)/x^2 on them (:9352).  ** The printed C4 at :9352
%        and again at :2540 has NO leading minus, but this book's OCR eats
%        leading minus signs (:9341 prints e^-x as "ex").  The listing settles
%        it: C4TH = -XNPP*(...), plotted as -C4 on a log axis.  OCR loss, not a
%        contradiction in the book. **  Bands for 3-5 come from THE TRUNCATION
%        ENVELOPE above: twice the measured envelope, capped at 0.5%.  At the
%        book's own settings that is 1.640e-3 for 3 and 4 and 1.660e-4 for 5,
%        against measured 6.647e-4, 6.379e-4, 7.381e-5.  The cap is what stops
%        an H-scaled band from buying a green at any step size: at H = 1e-3 the
%        envelope is 0.12 but the cap holds at 0.005 and the rows go red on a
%        measured 5.9e-2, which is what a 6% disagreement deserves.
%     6  Integrated N' vs the closed form of :9341, band 0.5%.  The residual is
%        PREDICTED as -GAM/(TAU^3 Q(x)+GAM) = -0.033% at x = 1.  If that
%        prediction exceeds half the band the row SKIPS with the arithmetic
%        rather than the band being widened to absorb it.
%   NOT SCORED: every number in Figs. 26.2-26.6 (image links, and the
%   "axis(...)" calls are plot ranges the author chose); the nominal parameter
%   set at :9356, reported as INFO against what the run actually used.
%
% --- WHAT IT PRINTS.  *** MEASURED 2026-08-24 on R2025b. ***  Everything in
%   this block was run.  At the book's settings (TAU 1, GAM 1e-5, TF 10,
%   H 1e-4):
%       tgo      N'         C1         C2         C3        C4
%        1 s   12.29678   12.2968    12.2968     6.14838  -4.52372
%       10 s    3.69269    0.0369514  0.369269   1.84517  -0.332319
%   ⚠️ The 10 s row is the INTEGRATED gain schedule, not the closed form.  The
%   closed form there is N' = 3.691086 and the sweep prints 3.69269 -- the
%   +0.044% that BENCHMARK 6 scores.  An earlier version of this header printed
%   the closed-form numbers in this table as if they were the sweep's; they are
%   not.  At tgo = 1 s the two agree to six figures only because the residual
%   there is 4.7e-6.
%   BM1 3.00060 vs 3 (+0.020%); BM2 ratio 27.7; BM3/4/5 residuals 6.647e-4,
%   6.379e-4, 7.381e-5 relative -- *** NOT "order 1e-6", which is what this
%   header claimed before anyone ran it *** -- against bands 1.640e-3,
%   1.640e-3, 1.660e-4; BM6 worst +0.0437% against 0.5%, and it is worst at
%   tgo = 10 s, not at 1 s: the residual is largest at the FAR end and it is
%   POSITIVE, where the old header predicted -0.0334% at 1 s.
%   The sweep records 1000 samples, the last at tgo = 9.99999999999 s.  The
%   loop guard is T >= TF - .0001, which nominally stops one step short at
%   9.9999 s -- but the accumulated T lands a few ulp below that, one more step
%   runs, and record 1000 sits on 10.00 s.  The old header's "999 samples,
%   last record 9.99 s" is the nominal arithmetic, not what the loop does.
%   COST: 100,000 outer steps and 200,000 4x4 right-hand-side evaluations, plus
%   SELF-CHECK 6's 2H re-run at half that again.  Measured on R2025b, median of
%   five: 0.19 s before check 6 existed, 0.30 s now.  Check 6 is 37% of this
%   port's runtime and it is the only thing here that measures whether the
%   bands mean anything, so it is not offered as optional.  (An earlier header
%   called this "the most expensive thing in this package".  That was never
%   measured and is not claimed here.)
%   ⚠️⚠️ 'H', 1e-3 IS NOT A CHEAP SUBSTITUTE.  It is ten times faster and
%   wrong: measured 5.9e-2, 5.7e-2, 6.6e-3 on BM3/4/5 and 4.04% on BM6 against
%   its 0.5% band, plus 0.103 and 0.309 on the two structural rows.  The old
%   header said it "still lands inside every band"; it lands outside five of
%   them.  H = 1e-4 is the listing's own choice and it is the right one.
%   A residual near a band EDGE is a fault, not a pass; do not widen a band.
%
% --- OUTPUTS.  R.tgo, R.x (= tgo/TAU); R.C1..R.C4 and R.NP (the INTEGRATED
%   gains -C(1,i), and C2*tgo); R.S (4x4xN); R.C1TH..R.C4TH and R.XNPP (the
%   listing's "Formula" curves, guard 1e-4); R.NP_exact (the same, guard
%   removed, i.e. :9341 as printed); R.bench, R.selfcheck (SIX checks -- 6 is
%   the step-size one and it costs a second integration at 2H), R.pass, R.p,
%   R.units, R.summary; and R.verdict, which is ALWAYS FORMED -- 'assert'
%   chooses the reaction and nothing else, false WARNS and still returns R
%   complete, because a port that refuses to return is one nobody can diagnose.
%
%   See also MGS.ZARCHAN.SHAPINGFILTERADJOINT, MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 26.1 line 1, verbatim.  Seconds; GAM in s^3.
    opts.TAU (1,1) double {mustBeFinite, mustBePositive} = 1.0
    opts.GAM (1,1) double {mustBeFinite, mustBePositive} = 1.0e-5
    opts.TF  (1,1) double {mustBeFinite, mustBePositive} = 10.0
    opts.H   (1,1) double {mustBeFinite, mustBePositive} = 1.0e-4
    % [] means the listing's own 0.01 s (its ".009999" against its own H).
    opts.samp double {mustBeReal} = []
    % The book's singularity guard, added to the PRINTED denominator 6*Q(x)
    % at zarchan.txt:9356.  Exposed only so the central claim in the header --
    % that 6*GAM/TAU^3 is the guard that would match the integration -- can be
    % tested rather than merely asserted.
    opts.guard (1,1) double {mustBeFinite, mustBeNonnegative} = 1.0e-4
    % Scored window, in units of TAU.  Below x ~ 0.2 the guard rather than the
    % physics sets the "Formula" curve, so a benchmark there scores the guard.
    % 1.0 keeps a full decade of margin on that.
    opts.x_lo (1,1) double {mustBeFinite, mustBePositive} = 1.0
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so an "if nargin == 0" mode here would never fire.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

TAU = opts.TAU;  GAM = opts.GAM;  TF = opts.TF;  H = opts.H;

samp = opts.samp;
if isempty(samp)
    samp = 0.01;                 % the listing's ".009999" at its own H
end
if ~isscalar(samp) || ~isfinite(samp) || samp <= 0
    error('mgs:zarchan:optimalGainsSingleLag:samp', ...
        'samp must be a positive finite scalar or [] (the listing''s 0.01 s); got %s.', ...
        mat2str(samp));
end
if H > samp
    error('mgs:zarchan:optimalGainsSingleLag:step', ...
        ['H = %g s exceeds the record interval samp = %g s, so the curve ' ...
         'would be sampled at the step rate while claiming the coarser one ' ...
         'in its own time vector.'], H, samp);
end
if H >= 0.05*TAU
    error('mgs:zarchan:optimalGainsSingleLag:stiff', ...
        ['H = %g s is not small against TAU = %g s.  The Riccati right-hand ' ...
         'side carries a 1/GAM amplification and this scheme returns a ' ...
         'smooth, entirely wrong gain schedule rather than blowing up, so ' ...
         'this is refused rather than warned about.'], H, TAU);
end
if TF <= 2*samp
    error('mgs:zarchan:optimalGainsSingleLag:short', ...
        'TF = %g s records fewer than two samples at samp = %g s.', TF, samp);
end

% =========================================================================
%   LISTING 26.1.  Transcribed from zarchan.txt:9359-9367.
% =========================================================================
[tgo, Cs, NP, S, asym, nEval] = riccatiRun(TAU, GAM, TF, H, samp);
if numel(tgo) < 2
    error('mgs:zarchan:optimalGainsSingleLag:empty', ...
        'the sweep recorded %d sample(s); at least two are needed.', numel(tgo));
end

x  = tgo / TAU;
C1 = Cs(:,1);  C2 = Cs(:,2);  C3 = Cs(:,3);  C4 = Cs(:,4);

% ---- the listing's "Formula" curves, and the unguarded book formula --------
XNPP     = npRatio(x, opts.guard/6);        % the listing's XNPP
NP_exact = npRatio(x, 0);                   % zarchan.txt:9341 as printed
gx       = expm1(-x) + x;                   % e^-x + x - 1, stably
C1TH = XNPP ./ (tgo.^2);
C2TH = XNPP ./ tgo;
C3TH = 0.5 * XNPP;
C4TH = -XNPP .* gx ./ (x.^2);

R = struct();
R.tgo = tgo;  R.x = x;
R.C1 = C1;  R.C2 = C2;  R.C3 = C3;  R.C4 = C4;
R.NP = NP;  R.S = S;
R.C1TH = C1TH;  R.C2TH = C2TH;  R.C3TH = C3TH;  R.C4TH = C4TH;
R.XNPP = XNPP;  R.NP_exact = NP_exact;
R.units = ['seconds and dimensionless: tgo/TAU/TF [s], GAM [s^3], ' ...
           'C1 [1/s^2], C2 [1/s], C3/C4/N'' [-].  No length appears.'];
R.p = struct('TAU',TAU, 'GAM',GAM, 'TF',TF, 'H',H, 'samp',samp, ...
             'guard',opts.guard, 'x_lo',opts.x_lo, 'n_rhs',nEval, ...
             'n_samples',numel(tgo), 'max_asymmetry',asym, ...
             'units',R.units);

R.selfcheck = structuralChecks(R, TAU, GAM, asym, opts.x_lo);

if opts.benchmark
    R.bench = benchmarks(R, TAU, GAM, H, opts.guard, opts.x_lo);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine".
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R); end

msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated at TAU = %g s, ' ...
        'GAM = %g s^3, TF = %g s, x_lo = %g.  An unvalidated port must not ' ...
        'be handed back silently.'], TAU, GAM, TF, opts.x_lo);
elseif ~R.selfcheck.all_ok
    % ⚠️ The old wording here read "a wiring fault, not a tolerance question",
    %    full stop.  That was only true while checks 2 and 3 carried
    %    roundoff-level tolerances -- and they were failing on truncation, so
    %    the sentence was accusing the wiring of the integrator's step size.
    %    Now the bands ARE the measured truncation envelope, so a failure means
    %    one of two things and the message says which two.
    msg = sprintf(['%d structural invariant(s) violated -- the port ' ...
        'disagrees with ITSELF beyond its band.  READ THE BAND''S OWN ' ...
        'EXPLANATION BEFORE HUNTING A BUG: if the band was CAP-limited the ' ...
        'step-size model held and only the absolute agreement is poor (refine ' ...
        'H); if it was ENVELOPE-limited the residual is bigger than the step ' ...
        'size explains, and self-check 6 separates truncation (2H ratio near ' ...
        '4) from a wiring fault (ratio 1).  First: %s.'], ...
        sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf(['%d of %d evaluated book values are outside tolerance ' ...
        '(first: %s, book %s, ours %s, band %s).'], numel(bad), sum(ev), ...
        R.bench(bad(1)).what, R.bench(bad(1)).book_str, ...
        R.bench(bad(1)).ours_str, R.bench(bad(1)).band);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:optimalGainsSingleLag:benchmark', '%s\n\n%s', ...
            msg, R.summary);
    else
        warning('mgs:zarchan:optimalGainsSingleLag:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function [tArr, Cs, NP, Ssamp, asym, nEval] = riccatiRun(TAU, GAM, TF, H, samp)
%RICCATIRUN  Listing 26.1, zarchan.txt:9359-9367.
%
%   Every constant this needs is an ARGUMENT.  MATLAB local functions cannot
%   see the main function's workspace, and a previous wave in this package
%   threw "Unrecognized function or variable" on the first line of its first
%   run for exactly that reason.

F = zeros(4,4);
S = zeros(4,4);
G = zeros(4,1);
G(1,1) = 0;  G(2,1) = 0;  G(3,1) = 0;  G(4,1) = 1./TAU;
F(1,2) = 1;  F(2,3) = 1;  F(2,4) = -1;  F(4,4) = -1./TAU;
S(1,1) = 1;                       % SF = diag([1 0 0 0]); miss = y(tF)
T  = 0;
S1 = 0;

% Loop-invariant in the listing, recomputed there every pass.  Hoisting is a
% no-op; see the header.
GT     = G';
GAMINV = 1./GAM;

% Initialised before the loop so the FLAG==1 branch cannot read a leftover.
SD = zeros(4,4);
C  = zeros(1,4);

nmax  = ceil(TF/samp) + 4;
tArr  = zeros(nmax,1);
Cs    = zeros(nmax,4);
NP    = zeros(nmax,1);
Ssamp = zeros(4,4,nmax);
n     = 0;
nEval = 0;
asym  = 0;

% The literal ".0001" is the book's, and equals its own H.  For any other H
% the loop still stops within one step of TF, which is all it is for.
while ~(T >= (TF - .0001))
    S1   = S1 + H;
    SOLD = S;
    STEP = 1;
    FLAG = 0;

    % Zarchan's two-pass predictor loop.  Pass 1 evaluates at the current
    % state; pass 2 takes a full Euler step, advances T by H, and re-evaluates
    % there.  T advances INSIDE this loop, exactly once -- move that line out
    % and the scheme quietly degrades to first order while still running.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            HSD  = H*SD;
            S    = S + HSD;
            T    = T + H;
        end
        SF     = S*F;
        GTS    = GT*S;
        C      = GAMINV*GTS;
        SFT    = SF';
        CT     = C';
        CTC    = CT*C;
        CTBC   = GAM*CTC;
        SFSFT  = SF + SFT;
        SD     = SFSFT - CTBC;
        FLAG   = 1;
        nEval  = nEval + 1;
    end
    % The listing writes "FLAG=0" here.  Dead -- the outer loop re-initialises
    % it.  Kept as a comment so nobody re-adds it thinking a state was lost.

    H2 = .5*H;  HSDP = H2*SD;  SS = SOLD + S;  SSP = .5*SS;  S = SSP + HSDP;

    % ONE increment, tested against samp.  The listing's literal ".009999"
    % pairs with its own H = .0001 to mean 0.01 s; reproducing the literal
    % would silently change meaning at any other H.
    if S1 >= samp - 0.5*H
        S1 = 0;
        n  = n + 1;
        tArr(n)   = T;
        % The listing's own sign convention: nc = C1 y + C2 ydot + C3 nT +
        % C4 nL with Ci = -C(1,i).  C4 comes out NEGATIVE; the book plots -C4.
        Cs(n,:)   = [-C(1,1), -C(1,2), -C(1,3), -C(1,4)];
        NP(n)     = Cs(n,2)*T;              % NP = C2*T, T being tgo
        Ssamp(:,:,n) = S;
        asym = max(asym, max(max(abs(S - S.'))));
    end
end

tArr = tArr(1:n);  Cs = Cs(1:n,:);  NP = NP(1:n);  Ssamp = Ssamp(:,:,1:n);
end


% =============================================================================
function q = qKernel(x)
%QKERNEL  Q(x) = int_0^x (e^-s + s - 1)^2 ds, so that 6*Q(x) is EXACTLY the
%   denominator printed at zarchan.txt:9341:
%
%       6Q = 2x^3 + 3 + 6x - 6x^2 - 12x e^-x - 3 e^-2x
%
%   As printed it is unusable below x ~ 0.05 (see the header), so below x = 1
%   this uses the exact series, whose coefficients follow from
%   (e^-s+s-1)^2 = sum_m (-1)^m (2^m - 2m - 2) s^m/m!  (m >= 4):
%
%       Q(x) = sum_{m>=4} (-1)^m (2^m - 2m - 2) x^(m+1) / (m! (m+1))
%
%   At x = 1 the m-th term is below 1e-13 by m = 20, so 45 terms is exact to
%   machine precision everywhere the series is used.
q   = zeros(size(x));
big = x >= 1;

% Both branches are guarded with any().  An all-false logical index on the
% left of an assignment whose right side is empty is a DELETION in MATLAB,
% not a no-op, and this is called with scalars as well as with whole sweeps.
if any(big(:))
    xb = x(big);
    q(big) = 0.5 - 0.5*exp(-2*xb) - 2*xb.*exp(-xb) - xb.^2 + xb + xb.^3/3;
end

if any(~big(:))
    xs   = x(~big);
    s    = zeros(size(xs));
    fact = 24;                 % 4!
    pow2 = 16;                 % 2^4
    xp   = xs.^5;              % x^(m+1) at m = 4
    for m = 4:45
        s    = s + ((-1)^m) * ((pow2 - 2*m - 2)/(fact*(m+1))) * xp;
        fact = fact*(m+1);
        pow2 = pow2*2;
        xp   = xp .* xs;
    end
    q(~big) = s;
end

% Q is an integral of a non-negative integrand.  This clamp exists only to
% stop a rounding-level negative at the very first sample; it cannot mask a
% real error, because at x >= 1 the value is never below 0.0299.
q = max(q, 0);
end


% =============================================================================
function np = npRatio(x, d)
%NPRATIO  N'(x) = x^2 (e^-x + x - 1) / (Q(x) + d).
%
%   d = 0                 -> the closed form printed at zarchan.txt:9341
%   d = guard/6           -> the listing's XNPP (its ".0001" sits on 6Q)
%   d = GAM/TAU^3         -> what the integration actually converges to
%
%   See the header for why those are the same function with three different d.
%   expm1 is used for e^-x + x - 1: the naive form loses digits below x ~ 1e-3
%   in exactly the same silent way the printed denominator does.
g  = expm1(-x) + x;
np = (x.^2 .* g) ./ (qKernel(x) + d);
np(x <= 0) = Inf;              % N' ~ 10 TAU/tgo; the limit is a divergence
end


% =============================================================================
function E = truncEnv(TAU, GAM, H, x_hi)
%TRUNCENV  This listing's OWN truncation error, as a MEASURED envelope.
%
%   *** THE FIVE CONSTANTS BELOW ARE MEASUREMENTS, NOT ESTIMATES. ***  See THE
%   TRUNCATION ENVELOPE in the header for the sweeps.  Each is the supremum
%   over x >= 1 of  residual / (nd * xh)  -- for C4, of residual / nd -- taken
%   on runs to x = 100 at GAM = 1e-3, 1e-4, 1e-5, 1e-6 crossed with H = 3e-5,
%   1e-4, 3e-4.  Worst observed 0.0816 / 0.0812 / 0.0825 / 0.1649 / 0.3712.
%
%   nd is the only dimensionless group the residual depends on.  GAM enters as
%   1/GAM, verified across four decades; TAU only through H/TAU and GAM/TAU^3,
%   verified at TAU = 0.5, 1, 2; and TF enters ONLY through how far the sweep
%   reaches, which is why the argument is x_hi and not TF.
%
%   ⚠️ C4 carries no x factor.  Its residual SATURATES (0.0812 nd at x = 10,
%   0.0812 nd at x = 100) while the other four grow linearly in x.  Giving C4
%   an xh it does not have would loosen a row that was already passing.
nd  = (H/TAU)^2 * (TAU^3/GAM);
xh  = max(1, x_hi);
E.nd  = nd;
E.xh  = xh;
E.C1  = 0.082 * nd * xh;
E.C3  = 0.082 * nd * xh;
E.C4  = 0.083 * nd;
E.rk1 = 0.165 * nd * xh;
E.S11 = 0.372 * nd * xh;
end


% =============================================================================
function [tol, why] = truncBand(pred, cap)
%TRUNCBAND  Band = TWICE the measured envelope, floored and capped.
%
%   The factor 2 is headroom, not slack: at the book's own settings the C1 row
%   lands at 41% of its band, the rank-one row at 47%, the S11 row at 45%.  A
%   row sitting at 95% would be a fault under this file's own rule.
%
%   ⭐ THE CAP IS THE PART THAT MATTERS.  A band proportional to H^2 passes at
%   EVERY step size by construction -- that is the failure mode of any
%   self-scaling tolerance, and it would have turned "C1 == N'/tgo^2" green at
%   H = 1e-3 while the actual agreement was 6%.  The cap makes a row assert two
%   things at once: the residual behaves like truncation AND the absolute
%   agreement is good enough to be called agreement.
%
%   ⚠️ THE FLOOR IS NOT A ROUNDOFF GUARD, IT IS THE NO-CONTROL PLATEAU, and it
%   was put here because the envelope alone produced a SPURIOUS RED.  The
%   TAU^3/GAM factor says the residual keeps shrinking as GAM grows.  It does
%   not: measured at H = 1e-4, TF = 10, the C1/C3/C4 residuals fall as 1/GAM
%   down to 6.5e-7 / 5.3e-7 / 7.2e-8 at GAM = 1e-2 and then SATURATE at
%   2.2e-8 / 4.8e-8 / 2.7e-8 for every GAM from 1 to 100 -- because at a large
%   control penalty there is barely any control, S is close to the linear flow
%   e^(F'tau) SF e^(F tau), and the 1/GAM amplification the envelope is built
%   on has gone away.  The plateau is still truncation (it is 2.1e-10 /
%   4.8e-10 / 2.7e-10 at H = 1e-5, a clean factor 100), it is simply not what
%   nd describes.  Rather than widen the constants -- which would have loosened
%   the C4 band 32x at the settings that matter -- the band stops descending at
%   1e-6.  This port claims no agreement finer than one part in a million; the
%   plateau is fifty times below that, and every mutation is millions of times
%   above it.
%
%   ⭐ WHY IT ALSO RETURNS "why".  Three different things can set this number,
%   and a row that fails means three different things depending on which one
%   did.  Envelope-limited: the residual is bigger than the step size explains,
%   so look for a wiring fault.  CAP-limited: the step-size model is fine and
%   the ABSOLUTE agreement is what is too poor -- refine H, do not go looking
%   for a bug.  Floor-limited: the row is measuring arithmetic.  A reader who
%   cannot tell those apart will file the second one under the first, which is
%   the failure mode this whole exercise was about.
raw = 2*pred;
if raw < 1e-6
    tol = 1e-6;
    why = sprintf('1e-6 no-control floor binds (2 x envelope = %.3e)', raw);
elseif raw > cap
    tol = cap;
    why = sprintf(['the %.3g%% CAP binds (2 x envelope = %.3e), so what ' ...
        'fails here is absolute agreement, NOT the step-size model'], ...
        100*cap, raw);
else
    tol = raw;
    why = sprintf('2 x measured envelope %.3e', pred);
end
end


% =============================================================================
function e = residSet(tgo, TAU, GAM, Cs, NP, S, x_lo)
%RESIDSET  The five residuals this file bands, in one place, so that the
%   convergence check measures EXACTLY the quantities the other checks score.
%   Order: [C1 ratio, C3 ratio, C4 ratio, rank-one minor, S11 closed form].
x   = tgo / TAU;
sel = x >= x_lo;
if ~any(sel), sel = true(size(x)); end
S11 = reshape(S(1,1,:), [], 1);
S14 = reshape(S(1,4,:), [], 1);
S44 = reshape(S(4,4,:), [], 1);
g   = expm1(-x) + x;

e = zeros(1,5);
e(1) = maxRel(Cs(sel,1).*tgo(sel).^2, NP(sel));
e(2) = maxRel(Cs(sel,3), 0.5*NP(sel));
e(3) = maxRel(Cs(sel,4), -NP(sel).*g(sel)./(x(sel).^2));
den  = max(S11(sel).*S44(sel), realmin);
e(4) = max(abs(S11(sel).*S44(sel) - S14(sel).^2) ./ den);
p    = GAM ./ (TAU^3*qKernel(x(sel)) + GAM);
e(5) = max(abs(S11(sel) - p) ./ max(p, realmin));
end


% =============================================================================
function K = structuralChecks(R, TAU, GAM, asym, x_lo)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.  They are the weaker
%   claim that it agrees with itself, which is what catches wiring faults.
K = struct();
names = {};  oks = [];  detail = {};

sel = R.x >= x_lo;
if ~any(sel), sel = true(size(R.x)); end
S11 = reshape(R.S(1,1,:), [], 1);
S14 = reshape(R.S(1,4,:), [], 1);
S44 = reshape(R.S(4,4,:), [], 1);

% ⚠️ CHECKS 2 AND 3 ARE NOT BIT-EXACT INVARIANTS, AND CHECK 1 IS.  Symmetry
%   below is preserved by the arithmetic itself and measures 0.000e+00, so it
%   keeps a roundoff-level tolerance.  Rank one and the S11 closed form are
%   properties of the EXACT Riccati flow that an explicit second-order scheme
%   does not inherit -- the rank-one minor is a quadratic invariant, and Heun
%   is not a method that preserves those -- so they are truncation-limited and
%   their tolerances come from the MEASURED envelope.  Writing them as
%   roundoff-level constants (they were 1e-5 and 1e-4) made both of them red at
%   the listing's own step size, which is the report this file used to hand
%   back: "the port disagrees with ITSELF" when it did not.
env = truncEnv(TAU, GAM, R.p.H, max(R.x));

% 1  S stays SYMMETRIC.  SD = (S*F) + (S*F)' - GAM*C'*C is symmetric to the
%    bit (both terms are), so this should be identically zero.  It is the
%    cheapest thing that notices a transposed F or a C'C built the wrong way.
nrm = max(max(abs(S11)), realmin);
names{end+1}  = 'S symmetric throughout';
oks(end+1)    = asym <= 1e-12*nrm;
detail{end+1} = sprintf('max |S - S''| = %.3e (scale %.3e)', asym, nrm);

% 2  S stays RANK ONE.  SF = diag([1 0 0 0]) is rank one and the Riccati flow
%    preserves that (S = [GAM/(W+GAM)] v v'), so S11*S44 - S14^2 must vanish.
%    This is the single strongest wiring check in the file: a wrong entry in
%    F, or CTBC dropped, breaks it immediately while every curve still plots.
den  = max(S11(sel).*S44(sel), realmin);
r1   = max(abs(S11(sel).*S44(sel) - S14(sel).^2) ./ den);
[tol1, why1] = truncBand(env.rk1, 0.01);
names{end+1}  = 'S rank one (S11*S44 == S14^2)';
oks(end+1)    = r1 <= tol1;
detail{end+1} = sprintf('max relative 2x2 minor %.3e vs band %.3e on x >= %g; %s', ...
                        r1, tol1, x_lo, why1);

% 3  S(1,1) == GAM/(TAU^3 Q(x) + GAM).  DERIVED here, not printed in the book
%    (see the header); it is the scalar form of check 2 and it is what pins
%    the GAM dependence that BENCHMARK 6's predicted residual relies on.
s11p = GAM ./ (TAU^3*qKernel(R.x(sel)) + GAM);
r3   = max(abs(S11(sel) - s11p) ./ max(s11p, realmin));
[tol3, why3] = truncBand(env.S11, 0.01);
names{end+1}  = 'S11 == GAM/(TAU^3 Q(x) + GAM)';
oks(end+1)    = r3 <= tol3;
detail{end+1} = sprintf('max relative deviation %.3e vs band %.3e on x >= %g; %s', ...
                        r3, tol3, x_lo, why3);

% 4  S(1,1) starts at SF(1,1) = 1 and only decreases: it is the residual
%    weight on the miss, and control can only reduce it.  An integration that
%    has gone unstable shows up here before it shows up anywhere else.
dS11 = diff(S11);
names{end+1}  = 'S11 non-increasing and <= 1';
oks(end+1)    = all(dS11 <= 1e-14) && max(S11) <= 1 + 1e-12;
detail{end+1} = sprintf('S11: %.4g -> %.4g, worst rise %+.3e', ...
                        S11(1), S11(end), max([dS11; -Inf]));

% 5  The PRINTED denominator equals 6*Q(x) where the printed form is still
%    numerically usable (x >= 1).  This is what licenses using the series
%    below x = 1: without it, "we replaced the book's expression" would be an
%    assertion rather than a measurement.
xb   = R.x(R.x >= 1);
if numel(xb) >= 2
    bot  = 2*xb.^3 + 3 + 6*xb - 6*xb.^2 - 12*xb.*exp(-xb) - 3*exp(-2*xb);
    r5   = max(abs(bot - 6*qKernel(xb)) ./ max(abs(bot), realmin));
    ok5  = r5 <= 1e-9;
    d5   = sprintf('max relative %.3e over %d samples at x >= 1', r5, numel(xb));
else
    ok5 = true;
    d5  = 'not evaluated: fewer than two samples at x >= 1';
end
names{end+1}  = 'printed denominator == 6*Q(x) for x >= 1';
oks(end+1)    = ok5;
detail{end+1} = d5;

% 6  THE RESIDUALS ARE TRUNCATION.  ⭐⭐ This is the check that makes checks 2
%    and 3 legitimate rather than convenient.  Bands 2 and 3 and the C1/C3/C4
%    benchmark bands all assert "what is left over is the integrator's step
%    size".  That is a claim about how the leftover BEHAVES, and the only way
%    to score it is to change the step size and look, so this re-runs the whole
%    integration at 2H and measures every one of the five residuals again.
%    Second order demands a factor (2H/H)^2 = 4.
%
%    ⚠️ THE BAND IS WIDE ON PURPOSE, AND IT IS STILL DECISIVE.  Measured 2H/H
%    ratios: 3.947-3.990 at the listing's own H, 3.999-4.000 at H = 1e-5,
%    3.57-3.91 at 3e-4, and 1.980-3.263 at 1e-3, where the asymptotic regime is
%    being left and the rank-one row is the first to go.  The 1.980 is what
%    sets the lower edge at 1.8.  Against that, a WIRING fault pins the ratio
%    at 1: the residual is then structural and does not care about H at all --
%    measured, F(2,4) flipped gives exactly 1.000 on the C4 row, CTBC dropped
%    gives 1.000 on the S11 row, F transposed gives NaN.  So [1.8, 5.0] around
%    4 separates the two populations by the whole gap between 4 and 1, and
%    tightening it would only catch coarse steps the ABSOLUTE bands catch
%    already.
%
%    A residual already above 5% carries no order information (the leading-term
%    expansion that defines "order 2" has stopped holding), and one below 1e-12
%    is roundoff.  Both are excluded, and if that leaves nothing the check says
%    which of the two it was instead of quietly passing.
Hc = 2*R.p.H;
if Hc > R.p.samp || Hc >= 0.05*TAU
    % 2H is refused by this port's own guards, so refine instead of coarsen.
    % H/2 is always legal here and costs twice the base run rather than half.
    Hc = 0.5*R.p.H;
end
[tc, Csc, NPc, Sc] = riccatiRun(TAU, GAM, R.p.TF, Hc, R.p.samp);
base = residSet(R.tgo, TAU, GAM, [R.C1 R.C2 R.C3 R.C4], R.NP, R.S, x_lo);
if numel(tc) >= 2
    comp = residSet(tc, TAU, GAM, Csc, NPc, Sc, x_lo);
else
    comp = nan(1,5);
end
rho   = (Hc/R.p.H)^2;
band6 = [0.45*rho, 1.25*rho];
scor  = base > 1e-12 & base <= 0.05 & isfinite(base);
rat   = comp ./ base;
if ~any(scor)
    if all(base <= 1e-12)
        ok6 = true;
        d6  = sprintf(['every residual is at roundoff (max %.3e); there is ' ...
            'nothing left for a step-size test to resolve'], max(base));
    else
        ok6 = false;
        d6  = sprintf(['H = %g s leaves every residual above 5%% (max %.3g); ' ...
            'that is outside the asymptotic regime, so the order test carries ' ...
            'no information and cannot be reported as a pass'], R.p.H, max(base));
    end
else
    ok6 = all(isfinite(rat(scor))) && ...
          all(rat(scor) >= band6(1) & rat(scor) <= band6(2));
    lbl = {'C1','C3','C4','rank1','S11'};
    parts = cell(1, sum(scor));
    idx = find(scor);
    for q = 1:numel(idx)
        parts{q} = sprintf('%s %.3f', lbl{idx(q)}, rat(idx(q)));
    end
    d6 = sprintf('H %g -> %g s, ratios [%s]; order 2 needs %.2f, band [%.2f %.2f]', ...
                 R.p.H, Hc, strjoin(parts, ' '), rho, band6(1), band6(2));
end
names{end+1}  = 'residuals scale as H^2 (re-run at 2H)';
oks(end+1)    = ok6;
detail{end+1} = d6;

K.names  = names;
K.oks    = logical(oks(:).');
K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    j = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{j}, detail{j});
end
end


% =============================================================================
function B = benchmarks(R, TAU, GAM, H, guard, x_lo)
%BENCHMARKS  Computed vs published, with the PROVENANCE of every band.
%
%   *** EVERY BOOK VALUE HERE WAS GREPPED OUT OF zarchan.txt AND THE LINE
%   NUMBER IS IN THE .source FIELD. ***  Bands and their provenance are in the
%   header; do NOT widen one to turn a row green.

B    = emptyBench();
sel  = R.x >= x_lo;
nsel = sum(sel);

% Bands 3-5.  See THE TRUNCATION ENVELOPE in the header: twice the MEASURED
% envelope, capped at the same 0.5% this file already uses as its reading of
% ":9373 near perfect agreement", floored at roundoff.  The cap is load-bearing
% -- without it an H^2 band would pass at every step size, including the ones
% where the port disagrees with the book by 6%.
envB = truncEnv(TAU, GAM, H, max(R.x));
[tolC1, whyC1] = truncBand(envB.C1, 0.005);
[tolC3, whyC3] = truncBand(envB.C3, 0.005);
[tolC4, whyC4] = truncBand(envB.C4, 0.005);

% ---- 1.  N' -> 3 for long time to go ---------------------------------------
% A LIMIT claim, so it is scored on the closed form at x = 1e4, not on the
% sweep: at the sweep's own x = 10 the ratio is 3.69, 23% above the asymptote.
nInf = npRatio(1e4, 0);
B = addVal(B, 'N'' -> 3 for long time to go', 3, nInf, 'text', 0.005, ...
    'Ch.8 p.179 text (zarchan.txt:2526)', '-', ...
    sprintf(['evaluated at x = 1e4.  At the sweep''s own longest x = %.3g ' ...
             'the closed form gives %.4f, %.0f%% above the asymptote -- ' ...
             'which is why the limit is scored on the formula, not the run'], ...
             max(R.x), npRatio(max(R.x),0), 100*(npRatio(max(R.x),0)/3 - 1)));

% ---- 2.  N' grows considerably as tgo -> 0 (SHAPE) --------------------------
xs   = logspace(-1, 1, 200).';
ns   = npRatio(xs, 0);
mono = all(diff(ns) < 0);
rat  = ns(1)/ns(end);
B = addShape(B, 'N'' grows considerably as time to go shrinks', ...
    'strictly decreasing in x over [0.1, 10], with N''(0.1)/N''(10) >= 10', ...
    sprintf('monotone: %d; N''(0.1) = %.2f, N''(10) = %.4f, ratio %.1f', ...
            mono, ns(1), ns(end), rat), ...
    mono && rat >= 10, 'Ch.8 p.179 text (zarchan.txt:2526)', ...
    ['scored on the UNGUARDED formula.  The listing''s XNPP turns over and ' ...
     'collapses below x ~ 0.2 because the .0001 guard, not the physics, ' ...
     'sets the denominator there']);

% ---- 3-5.  the published gain STRUCTURE, on the integrated gains ------------
% Nothing in the integration forces these: they are ratios of the components of
% the zero-effort-miss vector, and they hold at any GAM and any guard, which is
% what makes them the sharpest rows here.  See the header, consequence (c).
if nsel >= 2
    NPs = R.NP(sel);
    r3  = maxRel(R.C1(sel).*R.tgo(sel).^2, NPs);
    B = addBound(B, 'C1 == N''/tgo^2 on the integrated gains', ...
        sprintf('<= %.4f%%', 100*tolC1), ...
        sprintf('%.3e relative (%.0f%% of band)', r3, 100*r3/tolC1), ...
        r3 <= tolC1, 'p.653 (zarchan.txt:9347 with :9349)', '1/s^2', ...
        ['with NP = C2*tgo this is C1*tgo == C2 -- a property of the ' ...
         'Riccati solution, not an identity of the listing''s arithmetic.  ' ...
         'Band: ' whyC1 '.  ⚠️ It is a ratio of two components of the ' ...
         'zero-effort-miss vector, so it does NOT see the quadratic term: ' ...
         'deleting CTBC entirely leaves this row at 2.2e-8, greener than the ' ...
         'correct code.  SELF-CHECK 3 catches that (and so, by nine orders, ' ...
         'does BM6 -- measured, not assumed)']);

    r4 = maxRel(R.C3(sel), 0.5*NPs);
    B = addBound(B, 'C3 == 0.5 N'' on the integrated gains', ...
        sprintf('<= %.4f%%', 100*tolC3), ...
        sprintf('%.3e relative (%.0f%% of band)', r4, 100*r4/tolC3), ...
        r4 <= tolC3, 'p.653 (zarchan.txt:9351)', '-', ...
        ['the target-acceleration gain; independent of GAM and of the guard, ' ...
         'though its RESIDUAL is not -- that goes as 1/GAM, which is why the ' ...
         'band carries TAU^3/GAM.  Band: ' whyC3]);

    pred5 = -NPs .* (expm1(-R.x(sel)) + R.x(sel)) ./ (R.x(sel).^2);
    r5    = maxRel(R.C4(sel), pred5);
    B = addBound(B, 'C4 == -N''(e^-x + x - 1)/x^2 on the integrated gains', ...
        sprintf('<= %.4f%%', 100*tolC4), ...
        sprintf('%.3e relative (%.0f%% of band)', r5, 100*r5/tolC4), ...
        r5 <= tolC4, 'p.653 (zarchan.txt:9352; sign from the listing)', '-', ...
        ['the printed C4 at :9352 and :2540 has no leading minus, but this ' ...
         'book''s OCR eats leading minus signs; the listing computes ' ...
         'C4TH = -XNPP*(...) and plots -C4 on a log axis, so C4 is negative.  ' ...
         'Band carries NO x factor: this residual SATURATES at 0.083 nd ' ...
         '(measured 0.0812 nd at x = 10 and at x = 100) while the other four ' ...
         'grow linearly in x.  Band: ' whyC4]);
else
    whyWin = sprintf(['only %d sample(s) at x >= %g (TF = %g s, TAU = %g s, ' ...
        'samp = %g s); the scored window is empty'], nsel, x_lo, R.p.TF, ...
        TAU, R.p.samp);
    B = addSkip(B, 'C1 == N''/tgo^2 on the integrated gains', 'N''/tgo^2', ...
        whyWin, 'p.653 (zarchan.txt:9347 with :9349)', '1/s^2');
    B = addSkip(B, 'C3 == 0.5 N'' on the integrated gains', '0.5 N''', ...
        whyWin, 'p.653 (zarchan.txt:9351)', '-');
    B = addSkip(B, 'C4 == -N''(e^-x + x - 1)/x^2 on the integrated gains', ...
        '-N''(e^-x+x-1)/x^2', whyWin, 'p.653 (zarchan.txt:9352)', '-');
end

% ---- 6.  "near perfect agreement" between integration and closed form -------
% Band 0.5%, and the residual is PREDICTED exactly by the rank-one solution:
% N'_num/N'_exact = 1/(1 + GAM/(TAU^3 Q(x))).  If that prediction already eats
% half the band the row SKIPS with the arithmetic -- it would be scoring GAM,
% not the integration -- rather than the band being widened to absorb it.
if nsel >= 2
    % "/" and not "./": x_lo is declared (1,1) so this IS a scalar, and the
    % "if devPred <= .0025" below would silently become an all() over a vector
    % if it ever stopped being one.
    devPred = GAM / (TAU^3*qKernel(x_lo) + GAM);
    if devPred <= 0.0025
        r6 = maxRel(R.NP(sel), R.NP_exact(sel));
        % ⚠️ SIGNED, at the window's two ends, because the row's own prediction
        %    only applies at ONE of them.  The finite-GAM offset is what sets
        %    the residual at x_lo; by the far end, truncation has overtaken it
        %    AND FLIPPED THE SIGN.  Reporting only max|.| let this file's
        %    header claim for years that BM6 was "worst -0.0334% at tgo = 1 s"
        %    when the worst is +0.044% at tgo = 10 s.
        jLo = find(sel, 1, 'first');
        jHi = find(sel, 1, 'last');
        sLo = R.NP(jLo)/R.NP_exact(jLo) - 1;
        sHi = R.NP(jHi)/R.NP_exact(jHi) - 1;
        B = addBound(B, 'integrated N'' vs the closed form of :9341', ...
            '<= 0.5000%', ...
            sprintf('%.4f%% worst on x >= %g (%+.4f%% at x = %g, %+.4f%% at x = %g)', ...
                    100*r6, x_lo, 100*sLo, R.x(jLo), 100*sHi, R.x(jHi)), ...
            r6 <= 0.005, ...
            'p.653 formula + p.657 text (zarchan.txt:9341, :9373)', '-', ...
            sprintf(['the quantitative form of "near perfect agreement".  ' ...
                'At the near end the residual is the finite-GAM offset ' ...
                '-GAM/(TAU^3 Q(%.4g)+GAM) = %.4f%% and it lands there.  It ' ...
                'does NOT stay there: truncation grows like nd*x and takes ' ...
                'over by the far end, so the worst value is at max(x) and is ' ...
                'POSITIVE.  Anything near the band edge is a fault, not a pass'], ...
                R.x(jLo), -100*(GAM/(TAU^3*qKernel(R.x(jLo)) + GAM))));
    else
        B = addSkip(B, 'integrated N'' vs the closed form of :9341', ...
            'N'' = 6x^2(e^-x-1+x)/(2x^3+3+6x-6x^2-12x e^-x-3e^-2x)', ...
            sprintf(['at x_lo = %g the finite-GAM offset is already ' ...
                'GAM/(TAU^3 Q + GAM) = %.3f%%, over half the 0.5%% band, so ' ...
                'the row would be measuring GAM = %g s^3 rather than the ' ...
                'integration.  Raise x_lo or lower GAM'], ...
                x_lo, 100*devPred, GAM), ...
            'p.653 formula (zarchan.txt:9341)', '-');
    end
else
    B = addSkip(B, 'integrated N'' vs the closed form of :9341', 'N''(x)', ...
        sprintf('only %d sample(s) at x >= %g', nsel, x_lo), ...
        'p.653 formula (zarchan.txt:9341)', '-');
end

% ---- INFO / SKIP rows.  Reported, never scored. -----------------------------
B = addSkip(B, 'the gain values plotted in Figs. 26.2-26.6', '--', ...
    ['the figures became image links in extraction and the chapter prints ' ...
     'no number from them; the "axis([0 10 .01 1000])" calls in the listing ' ...
     'are plot ranges the author chose, not measurements'], ...
    'Figs. 26.2-26.6 (zarchan.txt:9373-9382)', '-');

B = addInfo(B, 'the listing''s nominal parameter set', ...
    'TAU 1 s, TF 10 s, GAM .00001, H .0001 s', ...
    sprintf('TAU %g s, TF %g s, GAM %g s^3, H %g s, samp %g s, %d RHS evals', ...
            TAU, R.p.TF, GAM, H, R.p.samp, R.p.n_rhs), ...
    'p.654 text (zarchan.txt:9356)', ...
    'NOT SCORED: these are the run''s inputs, not a result to adjudicate');

B = addInfo(B, 'the guard that would match the integration', ...
    sprintf('the book chose %g (zarchan.txt:9356)', guard), ...
    sprintf('6*GAM/TAU^3 = %g; ratio %.3f, and 5/3 as tgo -> 0', ...
            6*GAM/TAU^3, guard/max(6*GAM/TAU^3, realmin)), ...
    'derived here from the rank-one Riccati solution', ...
    ['NOT SCORED, and not a book claim.  It is why Figs. 26.2-26.6 look ' ...
     'exact: on five semilogy decades a 5/3 factor confined to the last ' ...
     'tenth of a second is invisible']);
end


% =============================================================================
function r = maxRel(a, b)
%MAXREL  Worst relative deviation of a from b, ignoring samples where b is
%   zero (there are none on the scored window, but a silent Inf here would
%   read as a failure of the port rather than of the window).
a = a(:);  b = b(:);
ok = isfinite(a) & isfinite(b) & b ~= 0;
if ~any(ok)
    r = Inf;
else
    r = max(abs(a(ok)./b(ok) - 1));
end
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw "Subscripted assignment between dissimilar
%   structures" -- which is the behaviour we want: a struct array that quietly
%   reordered fields would make [B.ok] mean something else.  Schema is the one
%   mgs.zarchan.benchSuite's shared benchRows() consumes.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.4g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.5g', ours);
B(i).band = band;  B(i).tol = tol;
B(i).units = units;  B(i).source = source;  B(i).detail = detail;
if ~isfinite(ours) || book == 0
    B(i).pct = NaN;  B(i).ok = false;  B(i).skipped = true;
    B(i).why = 'the computed value is not finite (or the book value is zero)';
    return
end
B(i).pct = 100*(ours/book - 1);
B(i).ok  = abs(ours/book - 1) <= tol;
B(i).skipped = false;  B(i).why = '';
end

function B = addShape(B, what, bookStr, oursStr, ok, source, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'shape';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'shape';  B(i).tol = NaN;
B(i).ok   = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = '-';  B(i).detail = detail;
end

function B = addBound(B, what, bookStr, oursStr, ok, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'bound';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'bound';  B(i).tol = NaN;
B(i).ok   = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = units;  B(i).detail = detail;
end

function B = addInfo(B, what, bookStr, oursStr, source, why)
%ADDINFO  Reported, never scored.  skipped = true keeps it out of the pass
%   count in BOTH directions -- an INFO row can neither pass nor fail.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'info';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'info';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = '-';  B(i).detail = '';
end

function B = addSkip(B, what, bookStr, why, source, units)
%ADDSKIP  A check that COULD NOT RUN.  Never a pass: a check that did not
%   happen is a different thing from one that succeeded.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'skip';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = '--';
B(i).pct  = NaN;   B(i).band = 'skip';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end


% =============================================================================
function s = render(R, ev)
L = {};
p = R.p;
L{end+1} = 'Zarchan Listing 26.1 -- RICCATI EQUATIONS FOR OPTIMAL CONTROL GAINS';
L{end+1} = '   (single-lag flight control system; Ch.26 derives, it does not test)';
L{end+1} = ['UNITS: ' R.units];
L{end+1} = sprintf(['   TAU = %g s   GAM = %g s^3   TF = %g s   H = %g s   ' ...
    'samp = %g s   guard = %g'], p.TAU, p.GAM, p.TF, p.H, p.samp, p.guard);
L{end+1} = sprintf('   %d samples, %d right-hand-side evaluations, scored on x >= %g', ...
    p.n_samples, p.n_rhs, p.x_lo);
L{end+1} = '';

L{end+1} = 'CONTROL GAINS  (integrated | the listing''s "Formula" curve)';
L{end+1} = sprintf('%8s %10s %19s %19s %19s %19s', 'tgo [s]', 'N''', ...
    'C1 [1/s^2]', 'C2 [1/s]', 'C3 [-]', 'C4 [-]');
% max(R.tgo), not p.TF: the loop stops at TF - .0001, so the last record is
% one samp short of TF and a table keyed on TF would silently omit its own
% endpoint row.
want = [1 2 3 5 max(R.tgo)];
for k = 1:numel(want)
    if want(k) < min(R.tgo) - 1e-9 || want(k) > max(R.tgo) + 1e-9, continue, end
    [~, j] = min(abs(R.tgo - want(k)));
    L{end+1} = sprintf('%8.2f %10.4f %9.4g|%9.4g %9.4g|%9.4g %9.4g|%9.4g %9.4g|%9.4g', ...
        R.tgo(j), R.NP(j), R.C1(j), R.C1TH(j), R.C2(j), R.C2TH(j), ...
        R.C3(j), R.C3TH(j), R.C4(j), R.C4TH(j)); %#ok<AGROW>
end
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL SELF-CHECKS  (self-consistency, NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-42s %s', vstr(K.oks(i)), K.names{i}, ...
        K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

L{end+1} = 'BENCHMARKS vs THE BOOK';
if isempty(R.bench)
    L{end+1} = '   (none: ''benchmark'' is false -- NOT CHECKED, not "fine")';
else
    for i = 1:numel(R.bench)
        b = R.bench(i);
        if b.skipped
            tag = upper(b.kind);
            L{end+1} = sprintf('   %-6s %-52s %s', tag, b.what, b.why); %#ok<AGROW>
        else
            L{end+1} = sprintf('   %-6s %-52s book %-16s ours %s', ...
                vstr(b.ok), b.what, b.book_str, b.ours_str); %#ok<AGROW>
        end
        L{end+1} = sprintf('          src: %s', b.source); %#ok<AGROW>
    end
end
L{end+1} = '';
if any(ev)
    L{end+1} = sprintf('   %d row(s) scored, %d skipped/info.', sum(ev), sum(~ev));
end
s = strjoin(L, newline);
end

function v = vstr(ok)
if ok, v = 'ok'; else, v = 'FAIL'; end
end


% =============================================================================
function drawCurves(R)
%DRAWCURVES  The listing's five semilogy figures, with its own axis ranges.
%   Off by default; plotting is not a check.
pairs = {R.C1, R.C1TH, 'C1',  [0 10 .01 1000]; ...
         R.C2, R.C2TH, 'C2',  [0 10 .1 200]; ...
         R.C3, R.C3TH, 'C3',  [0 10 1 20]; ...
         -R.C4, -R.C4TH, '-C4', [0 10 .1 20]; ...
         R.NP, R.XNPP, 'Effective Navigation Ratio', [0 10 .1 40]};
for k = 1:size(pairs,1)
    figure;
    semilogy(R.tgo, pairs{k,1}, R.tgo, pairs{k,2}); grid on
    xlabel('Time to go (s)'); ylabel(pairs{k,3});
    legend('Riccati', 'Formula', 'Location', 'best');
    a = pairs{k,4};  a(2) = max(a(2), max(R.tgo));  axis(a);
end
end
