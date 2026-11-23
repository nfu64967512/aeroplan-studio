function R = mmaeWeaveFilters(opts)
%MGS.ZARCHAN.MMAEWEAVEFILTERS  Zarchan Listing 27.1 -- three-filter fixed MMAE
%   (multiple model adaptive estimation) against a weaving target.
%
%   R = mgs.zarchan.mmaeWeaveFilters()
%   R = mgs.zarchan.mmaeWeaveFilters('WBANK', [1 2.5 3])   Fig. 27.14's bank
%   R = mgs.zarchan.mmaeWeaveFilters('assert', false)      warn, still return R
%   R = mgs.zarchan.mmaeWeaveFilters('FSCALE', [1 2 1])    scale f(1..3); row 4b
%
%   Three independent four-state linear weave Kalman filters, tuned to 1, 2 and
%   4 r/s, all fed the SAME seeker measurement.  Bayes' rule on each filter's
%   likelihood gives the probability that it is the correct one; frequency and
%   states are the probability-weighted blend, and that blend drives the
%   compensated weave guidance law.  Truth is 2 r/s (zarchan.txt:10075), so the
%   scoreable content of this chapter is WHICH FILTER WINS PROBABILITY AND WHEN.
%
%   UNITS ARE THE BOOK'S: ft, ft/s, ft/s^2 with g = 32.2 ft/s^2; r/s; s.
%   Nothing is converted, so every number printed lies straight against a
%   published one.  Convert at the CALL SITE and say so there.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_27_01.m does not parse.  Its 70 code lines were
%   byte-compared against zarchan.txt:10077-10151 with the running page headers
%   removed: they agree except for the four OCR repairs below.  So the
%   transcription header's "prose leaked in x1" IS those page headers, already
%   dropped by the extractor -- nothing was invented and nothing is missing.
%
%     1  "while T ,= (TF-.0001)" -> "<=".  "," is this book's OCR of "<";
%        shape-forced, T only increases.
%     2  "while STEP ,=1" -> "<=".  The body sets STEP=2 to leave.
%     3  "if S .= (TS-.00001)" -> ">=".  "." is the OCR of ">"; S is reset to 0
%        on each record, so "<=" would record every step.
%     4  THE ONE UNRESOLVED VARIABLE-TO-VARIABLE COMPARISON, "if XNC.XLIM"
%        (zarchan.txt:10145):
%        resolved as "if XNC > XLIM, XNC = XLIM".  Not a judgement call -- the
%        NEXT line of the book (zarchan.txt:10146) is its mirror, "if XNC,-XLIM
%        XNC=-XLIM", the same "<" spelled with the other OCR glyph.  The pair is
%        a symmetric +/-XLIM saturation; ">" the other way would pin XNC to the
%        limit permanently.
%
%   BOOK QUIRKS REPRODUCED, NOT TIDIED (load-bearing, or provably not):
%     a  THE PROBABILITY RECURSION IS SEQUENTIAL AND IN PLACE, AND THE BOOK'S
%        OWN IDENTITY DOES NOT HOLD FOR IT.  The listing (zarchan.txt:10138)
%        overwrites PROB1 before PROB2's denominator reads it, so its three
%        lines are NOT the simultaneous update printed at zarchan.txt:10063-
%        10065, and the identity p1+p2+p3 = 1 that the book draws from that
%        update at zarchan.txt:10068 is FALSE OF THE CODE.  Measured, default
%        seed: the sum ranges 0.882277..1.096623, worst |sum-1| = 1.177e-01 at
%        t = 3.20 s.  This is not drift and not a missing constant factor --
%        the excursion is TWO-SIDED (259 samples below 1, 245 above), and the
%        same run's simultaneous shadow holds the identity to 2.2e-16.  THE
%        PORT REPRODUCES THE CODE, NOT THE EQUATION, and says so on the board.
%
%        WHAT THE RECURSION DOES CONSERVE.  Write a_i = p_i*F_i and D = sum a.
%        Line 1 divides by the all-old D, so p1' = a1/D is EXACTLY the printed
%        equation's p1 -- bitwise, not to a tolerance (selfcheck 5).  Line i
%        then divides by D_i = D + sum_{j<i} (p_j'F_j - p_j F_j), so lines 2
%        and 3 are NOT the printed equation (selfcheck 5 requires them to
%        differ: measured 2.43e-02 and 5.34e-02 on its fixed inputs).  The
%        identity is recovered exactly when every D_i collapses back onto D,
%        which happens in two cases and only those two: when the likelihoods
%        coincide, and when the posterior concentrates on one filter, because
%        p_j(F_j - Fbar) -> 0 for every j.  So the violation is a TRANSIENT OF
%        THE IDENTIFICATION EPOCH, not a standing normalisation error:
%        |sum-1| falls to 1.33e-05 by t = 3.5 s (the book's own "almost
%        certain" time, zarchan.txt:10154), 3.98e-09 by 4 s, 9.54e-12 by 5 s,
%        and 0 at t = 10 s.
%
%        ⚠️ WHAT IS *NOT* SCOREABLE HERE, AND WHY ROW 4 IS NOW UNSCORED.  The
%        printed equation is p_i <- p_i f_i / sum_j p_j f_j, i.e. a vector
%        divided by its own sum, so p1+p2+p3 = 1 holds BY ALGEBRA -- which is
%        precisely what the book says at :10067, "we can see FROM THE PRECEDING
%        SET OF EQUATIONS".  A row scoring that shadow's sum against 1 measures
%        floating-point round-off (2.2e-16) and NOTHING ELSE; it cannot fail.
%        It was scored green until 2026-08-24 and is now an INFO row: the
%        number still prints, the :10068-vs-:10138 split still prints, but it
%        buys no credit.  What replaced it on the board is ROW 4b, an
%        invariance of the recursion that a real defect can break -- see there.
%        Recovery (row 5) is scored against the CODE, over a fixed seed
%        ensemble, at 1e-12 rather than the old 1e-9.
%
%        The blend is barely affected: WHPZ is a sum of un-normalised weights,
%        yet max |w_hat - w| is 0.7883 r/s raw against 0.7867 r/s normalised.
%     b  PROB0 = .333, not 1/3 (eq. at :10069-10073).  BENCHMARK 3 scores the gap.
%     c  The likelihood uses 6.28, not 2*pi (zarchan.txt:10137).  Same constant
%        in all three f's, so it cancels in Bayes' rule and cannot move a
%        probability.  That used to be an ASSERTION in this comment; it is now
%        BENCHMARK 4b, and measured: a common factor that is a power of two
%        leaves the whole flight BITWISE identical (max |dp| = 0 exactly), and
%        the actual 6.28 -> 2*pi substitution moves p by 1.887e-15 and the
%        miss by 3.74e-14 ft.  ⚠️ The reach of that row is exactly the class of
%        faults that break HOMOGENEITY -- a dropped denominator, an additive
%        epsilon in it, a factor applied to some f's and not others.  It does
%        NOT catch a per-filter constant baked into all runs (that stays
%        homogeneous), which is why selfcheck 1 checks PHI element by element.
%     d  Q(1,1) uses .333 for 1/3 -- which is why no selfcheck compares Q against
%        the exact integral; that check would be red by construction.
%     e  XNP = 3 is set and never used (the weave law computes its own N' as
%        XNPP).  Not exposed as an option: a switch nothing reads fails silently.
%     f  RES adds +.5*Ts^2*nL where the state update subtracts it.  NOT a typo:
%        RES = z - H*PHI*x - H*G*u and H*G*u = -.5*Ts^2*nL.  Both signs right.
%     g  randn is replaced by a MINSTD + Box-Muller stream inside this file.  A
%        benchmark that moves when someone else calls rng() is not a benchmark,
%        and setting the global stream would reach into the caller's session
%        mid-suite.  The two uniforms per draw come from SEPARATE BLOCKS:
%        consecutive values of a multiplicative generator lie on a 2-D lattice.
%
%   NOT A MODEL OF OUR AIRCRAFT: linearised planar homing, constant closing
%   velocity, one first-order lag, no gravity/drag/thrust.  It is the machinery.
%
%   BENCHMARKS carry their zarchan.txt line in .source.  Nothing was read off a
%   figure -- extraction turned every figure into an image link -- so figure
%   CLAIMS are scored as orderings and bounds.
%
%   ⚠️⚠️ THE 2-s/3-s CONTRADICTION IS NOT ADJUDICABLE, AND ROW 6 NO LONGER
%   CLAIMS IT IS.  The book contradicts itself on timing (text "after 2 s" at
%   zarchan.txt:10152 vs caption "after 3 s" at zarchan.txt:10157).  Until
%   2026-08-24 row 6 SCORED the caption and this comment said "the run
%   adjudicates rather than the editor".  It does not.  Over a fixed 12-seed
%   ensemble the caption's 3 s reading holds on 6 seeds and FAILS ON 6 (555,
%   31337, 42, 8675309, 2718281, 161803, with 14, 22, 40, 2, 12 and 21
%   violating samples), and the prose's 2 s reading holds on exactly one
%   (seed 2024, zero violations).  The direction of the verdict is a coin
%   flip on the draw, so scoring either reading scores the coin.  ⇒ Row 6 is
%   now an INFO row: both readings are measured, the whole per-seed tally
%   prints, and neither is scored.
%
%   ⭐ WHAT SURVIVES ACROSS DRAWS is the book's OWN third timing statement,
%   "after 3.5 s the algorithm is almost certain" (zarchan.txt:10154).  That
%   one holds on every seed in the ensemble, and it is where the identification
%   claim is now scored (row 7): worst-over-seeds min p(correct) for t >= 3.5 s
%   is 0.9571 against the 0.95 band, and the latest sample at which the correct
%   filter is not the strict leader, over all 12 seeds, is t = 3.40 s.  A
%   0.0071 margin and a 0.10 s margin -- thin on purpose, and both measured.
%
%   The 5-ft rms miss is a SKIP: it needs the 20 x 50 = 1000-flight Monte Carlo
%   of Fig. 27.10.
%
%   *** EVERY NUMBER IN THIS HEADER IS MEASURED, not asserted: this file was
%   executed on 2026-08-24 at the default seed 20260824 and over the fixed
%   12-seed ensemble hard-coded in seedEnsemble().  Rows 1-3, 4b, 5, 7, 8 and 9
%   are scored (8 evaluated); rows 4 and 6 are INFO and row 10 is a SKIP; 5
%   structural checks.  The ROWS state what the BOOK claims; the numbers beside
%   them come from the run.  Rows 8 and 9 are still SINGLE-DRAW rows scored on
%   the default seed, as the book's own figures are -- their ensemble spread is
%   printed in .detail precisely so nobody quotes them as population results
%   (row 8's 1 r/s reading fails on 4 of the 12 seeds, and row 9's ordering on
%   1). ***
%
%   ⭐⭐ FAULT-INJECTION LEDGER, 2026-08-24.  A row nobody has made fail is a
%   row nobody has verified, so every scored row below was turned red on
%   purpose and the reading recorded.  Options first, then code patches:
%     row 1   XNT 96.6 -> 96.5             2.996894 g   vs 3, band 1e-9
%     row 2   XLIM 322 -> 321              9.968944 g   vs 10, band 1e-9
%     row 3   PROB0 .333 -> .34            0.340000     vs 1/3, band 0.5%
%     row 4b  delete the F.*FSCALE line    unequal moves p by 0.000e+00 (0.830 needed)
%     row 4b  +1e-16 in the denominator    common 2^10 moves p by 2.331e-12 (0 or
%                                          subnormal needed)
%     row 5   bank -> Fig. 27.14's [1 2.5 3]  final |sum-1| 5.152e-09 vs 1e-12
%     row 7   SIGRIN 1e-4 -> 2e-4          ensemble worst p 0.2028 vs 0.95
%     row 8   WREAL 2 -> 3 (off the bank)  max |w_hat-w| 1.6250 vs 1 r/s
%     row 9   seed -> 42                   rms ratio 0.9506/0.9478 vs 1/3
%     sc 1    PHI(3,4) *= 1.0002           max |PHI-expm| 2.000e-06 vs 1e-12
%     sc 3    hoist the in-flight loop     0 below, 0 above of 1000 (both > 0 needed)
%     sc 5    seqBayes normalised whole-   line 1 delta 3.518e-02 (bitwise 0 needed)
%             vector inside the 3-loop
%     sc 5    seqBayes hoisted outright    lines 2,3 delta 0.000e+00, 0.000e+00
%             (loop removed)               (both > 0 needed); line 1 reads 0, green
%   ⚠️ THE TWO sc-5 PATCHES FIRE DIFFERENT CLAUSES, and re-verified 2026-08-24
%   because the first line above used to be labelled "hoist" and is not one.
%   A literal hoist leaves line 1 EXACTLY the printed equation (that is the
%   row's own first clause, and it stays green); what it destroys is lines 2
%   and 3, which is the second clause.  Ditto one precomputed shared
%   denominator: line 1 delta 0.000e+00, lines 2 and 3 both 0.000e+00.  Only
%   an ordering change that reaches line 1 -- the loop run backwards
%   (1.218e-02), or the vector normalisation applied three times (3.518e-02)
%   -- turns the FIRST clause red.  Both clauses are needed; neither alone
%   covers the other.
%   ⚠️ Row 4b survives three things BY DESIGN, and each is written into its
%   'why' rather than left for a reader to discover: a 1e-30 denominator
%   epsilon (it rounds away below the denominator's own eps and changes no
%   answer), a per-filter constant present in every run (still homogeneous --
%   selfcheck 1 is what covers that), and differences confined to the
%   SUBNORMAL range, where scaling by a power of two stops being exact in IEEE
%   binary64.  The book's own case underflows nowhere and reads exactly 0.
%
%   OUTPUTS
%     R.t R.prob R.prob_bayes R.whpz R.ytddg R.ytddhpzg R.miss_ft R.units
%     R.bench (shared schema) R.selfcheck (.names/.oks) R.pass R.verdict R.p
%
%   See also MGS.ZARCHAN.WEAVEKALMANFILTER (Listing 25.2, the single filter this
%   is built from), MGS.ZARCHAN.WEAVEEXTENDEDKALMAN, MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 27.1 line 1 and Table 27.1 (zarchan.txt:9937-9946), verbatim.
    opts.WBANK    (1,3) double {mustBeFinite, mustBePositive} = [1 2 4]
    opts.WREAL    (1,1) double {mustBeFinite, mustBePositive} = 2
    opts.TAU      (1,1) double {mustBeFinite, mustBePositive} = 0.5
    opts.VC       (1,1) double {mustBeFinite, mustBePositive} = 9000
    opts.XLIM     (1,1) double {mustBeFinite, mustBePositive} = 322
    opts.XNT      (1,1) double {mustBeFinite, mustBePositive} = 96.6
    opts.XNTREAL  (1,1) double {mustBeFinite}                 = 96.6
    opts.TS       (1,1) double {mustBeFinite, mustBePositive} = 0.01
    opts.TF       (1,1) double {mustBeFinite, mustBePositive} = 10
    opts.H        (1,1) double {mustBeFinite, mustBePositive} = 0.001
    opts.VM       (1,1) double {mustBeFinite, mustBePositive} = 3000
    opts.YIC      (1,1) double {mustBeFinite}                 = 0
    opts.HEDEG    (1,1) double {mustBeFinite}                 = 0
    opts.HEDEGFIL (1,1) double {mustBeFinite}                 = 20
    opts.SIGRIN   (1,1) double {mustBeFinite, mustBePositive} = 1e-4
    opts.PROB0    (1,1) double {mustBeFinite, mustBePositive} = 0.333
    opts.seed     (1,1) double {mustBeFinite, mustBeNonnegative} = 20260824
    % NOT IN THE LISTING.  A multiplicative factor on the three likelihoods,
    % default [1 1 1] so the nominal flight is bit-for-bit the listing's.  It
    % exists so BENCHMARK 4b can run the invariance experiment IN FLIGHT rather
    % than on a hand-made pair of vectors, and that row's second clause
    % requires an UNEQUAL setting to actually move the answer -- so this knob
    % cannot rot into quirk (e)'s dead switch without turning the row red.
    opts.FSCALE   (1,3) double {mustBeFinite, mustBePositive} = [1 1 1]
    % THE ONLY SWITCHES.  Not nargin: with an arguments block declaring only
    % name-value options MATLAB reports nargin = 0 however many were passed,
    % so an "if nargin == 0" mode here would never fire.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.assert    (1,1) logical = true
end

if opts.H >= opts.TS
    error('mgs:zarchan:mmaeWeaveFilters:step', ...
        ['H = %g s is not smaller than the filter sampling time TS = %g s. ' ...
         'The Kalman update would fire on every integration step and the run ' ...
         'would return a smooth, entirely wrong curve.'], opts.H, opts.TS);
end
if opts.TS >= opts.TF
    error('mgs:zarchan:mmaeWeaveFilters:short', ...
        'TS = %g s is not smaller than TF = %g s; no sample would be taken.', ...
        opts.TS, opts.TF);
end
if numel(unique(opts.WBANK)) < 3
    error('mgs:zarchan:mmaeWeaveFilters:bank', ...
        ['WBANK = %s has a repeated frequency.  Two identical filters share a ' ...
         'likelihood, so the probability split between them is arbitrary and ' ...
         'every identification row below would be reporting a coin toss.'], ...
        mat2str(opts.WBANK));
end

S = runFlight(opts, opts.WREAL, opts.seed);

R = struct();
R.t          = S.t;
R.prob       = S.prob;        % the listing's SEQUENTIAL recursion -- drives the law
R.prob_bayes = S.probb;       % the book's printed equation, same residuals
R.whpz       = S.whpz;        % blended weave frequency [r/s]
R.ytddg      = S.ytddg;       % true target acceleration [g]
R.ytddhpzg   = S.ytddhpzg;    % blended estimate of it [g]
R.miss_ft    = S.miss_ft;
R.units      = 'ft, ft/s, ft/s^2 (g = 32.2 ft/s^2); r/s; s; probabilities dimensionless';
R.p = struct('WBANK',opts.WBANK, 'WREAL',opts.WREAL, 'TF',opts.TF, ...
             'TS',opts.TS, 'H',opts.H, 'TAU',opts.TAU, 'VC',opts.VC, ...
             'XLIM',opts.XLIM, 'XNT',opts.XNT, 'SIGRIN',opts.SIGRIN, ...
             'PROB0',opts.PROB0, 'seed',opts.seed, 'n_samples',numel(S.t), ...
             'units',R.units);

R.selfcheck = structuralChecks(opts, S);

if opts.benchmark
    R.bench = benchmarks(R, opts);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  "Not checked" must never render as "fine".
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false; nothing was compared. R.pass = NOT CHECKED.';
elseif ~any(ev)
    msg = 'no row could be evaluated; an unvalidated port must not pass silently.';
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, a wiring fault rather than a tolerance question. First: %s.'], ...
        sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf('%d of %d evaluated rows outside tolerance (first: %s; book %s, ours %s).', ...
        numel(bad), sum(ev), R.bench(bad(1)).what, ...
        R.bench(bad(1)).book_str, R.bench(bad(1)).ours_str);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:mmaeWeaveFilters:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN COMPLETE.  benchSuite runs every port with assert off
        % so one red row cannot withhold the evidence for all the others.
        warning('mgs:zarchan:mmaeWeaveFilters:benchmark', ...
            '%s  (''assert'' false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function S = runFlight(P, WREAL, seed)
%RUNFLIGHT  Listing 27.1, zarchan.txt:10077-10151.  Every constant arrives as an
%   argument (P is the options struct): MATLAB local functions cannot see the
%   main function's workspace, and a previous wave here threw on line 1 for it.
TS = P.TS;  H = P.H;  TF = P.TF;  TAU = P.TAU;  VC = P.VC;  XLIM = P.XLIM;
XNT = P.XNT;  XNTREAL = P.XNTREAL;  SIGNOISE = P.SIGRIN;
W = P.WBANK(:).';  X = W*TS;                    % the listing's X1, X2, X3
PHIS = W.^2 * XNT^2 / TF;                       % PHIS1..PHIS3

PHI = cell(1,3);  Q = cell(1,3);  Pc = cell(1,3);
for i = 1:3
    [PHI{i}, Q{i}] = weaveMatrices(W(i), TS, PHIS(i));
    Pc{i} = diag([ (VC*TF*SIGNOISE)^2, (P.VM*P.HEDEGFIL/57.3)^2, ...
                   XNT^2, W(i)^2*XNT^2 ]);
end
HMAT = [1 0 0 0];  ID = eye(4);

T = 0;  Sacc = 0;  XNC = 0;  XNL = 0;
Y = P.YIC;  YD = -P.VM*P.HEDEG/57.3;
XLAM = Y/(VC*TF);  YDD = 0;  XNLD = 0;          % preset so the FLAG branch cannot
YH = zeros(1,3);  YDH = zeros(1,3);             % read a leftover on step 1
YTDDH = zeros(1,3);  YTDDDH = zeros(1,3);
PROB = repmat(P.PROB0, 1, 3);  PROBB = PROB;

nmax = ceil(TF/TS) + 8;
t = zeros(nmax,1);  prob = zeros(nmax,3);  probb = zeros(nmax,3);
whpz = zeros(nmax,1);  ytddg = zeros(nmax,1);  ytddhpzg = zeros(nmax,1);
zn = lcgNormal(seed, nmax);
n = 0;

while T <= (TF - 0.0001)
    Sacc = Sacc + H;
    YOLD = Y;  YDOLD = YD;  XNLOLD = XNL;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor.  T advances INSIDE this loop, exactly once,
    % which is what puts the second derivative evaluation at t+H.  Moving that
    % line out degrades the scheme to first order while still running smoothly.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            Y = Y + H*YD;  YD = YD + H*YDD;  XNL = XNL + H*XNLD;  T = T + H;
        end
        TGO  = TF - T + 0.000001;
        XLAM = Y/(VC*TGO);
        YTDD = XNTREAL*sin(WREAL*T);
        XNLD = (XNC - XNL)/TAU;
        YDD  = YTDD - XNL;
        FLAG = 1;
    end
    Y   = 0.5*(YOLD   + Y   + H*YD);
    YD  = 0.5*(YDOLD  + YD  + H*YDD);
    XNL = 0.5*(XNLOLD + XNL + H*XNLD);
    % XLAM is deliberately NOT recomputed from the corrected Y: the listing
    % measures the predictor value.  The difference is O(H^2), and keeping it
    % means the port cannot be blamed for a discrepancy the book owns.

    if Sacc >= (TS - 0.00001)
        Sacc = 0;
        TGO = TF - T + 0.000001;  RTM = VC*TGO;
        SIGN2 = (RTM*SIGNOISE)^2;

        % ---- Riccati, one per filter.  HMAT = [1 0 0 0], so H*M*H' is M(1,1)
        %      and M*H' is M(:,1); written that way, not as matrix products,
        %      because the products are the identical arithmetic at 16x the cost.
        GAIN = zeros(4,3);  CPZ = zeros(1,3);
        for i = 1:3
            M     = PHI{i}*Pc{i}*PHI{i}.' + Q{i};
            HMHTR = M(1,1) + SIGN2;             % a priori residual covariance
            K     = M(:,1)/HMHTR;
            Pc{i} = (ID - K*HMAT)*M;
            GAIN(:,i) = K;  CPZ(i) = HMHTR;
        end

        YTDD  = XNTREAL*sin(WREAL*T);
        YTDDD = XNTREAL*WREAL*cos(WREAL*T); %#ok<NASGU> % listing's dead line, quirk (e)-like
        n = n + 1;
        YSTAR = RTM*(XLAM + SIGNOISE*zn(n));    % RTM*XLAM is Y exactly

        % ---- the three filters.  ALL FOUR updates read the OLD estimates;
        %      the listing's YTDDHNEW temporary exists to stop YTDDH being
        %      clobbered before the jerk line reads it.
        RES = zeros(1,3);
        for i = 1:3
            xi = X(i);  wi = W(i);  s = sin(xi);  c = cos(xi);
            RES(i) = YSTAR - YH(i) - TS*YDH(i) - (1-c)*YTDDH(i)/(wi*wi) ...
                     - (xi-s)*YTDDDH(i)/(wi*wi*wi) + 0.5*TS*TS*XNL;
            nY  = YH(i) + TS*YDH(i) + (1-c)*YTDDH(i)/(wi*wi) ...
                  + (xi-s)*YTDDDH(i)/(wi*wi*wi) + GAIN(1,i)*RES(i) - 0.5*TS*TS*XNL;
            nYD = YDH(i) + s*YTDDH(i)/wi + (1-c)*YTDDDH(i)/(wi*wi) ...
                  + GAIN(2,i)*RES(i) - TS*XNL;
            nA  =  c*YTDDH(i) + s*YTDDDH(i)/wi + GAIN(3,i)*RES(i);
            nJ  = -wi*s*YTDDH(i) + c*YTDDDH(i) + GAIN(4,i)*RES(i);
            YH(i) = nY;  YDH(i) = nYD;  YTDDH(i) = nA;  YTDDDH(i) = nJ;
        end

        % ---- likelihood and Bayes.  6.28, not 2*pi: quirk (c).  The three
        %      lines are zarchan.txt:10137.
        F = exp(-0.5*RES.^2./CPZ)./sqrt(6.28*CPZ);
        F = F .* P.FSCALE;                      % [1 1 1] by default: exact no-op
        % The listing's THREE LINES, in place and sequential (quirk a): the
        % denominator is recomputed after each assignment, so PROB2's reads the
        % ALREADY-UPDATED PROB1.  Do not hoist the sum out of this loop.
        for i = 1:3
            PROB(i) = PROB(i)*F(i)/sum(PROB.*F);
        end
        % The book's printed equation on the same residuals, carried alongside
        % so selfcheck 3 can isolate the ordering.  Never drives the law.
        PROBB = PROBB.*F/sum(PROBB.*F);

        WHPZ = sum(W.*PROB);        YHPZ     = sum(YH.*PROB);
        YDHPZ = sum(YDH.*PROB);     YTDDHPZ  = sum(YTDDH.*PROB);
        YTDDDHPZ = sum(YTDDDH.*PROB);

        % ---- compensated weave guidance law, zarchan.txt:10141 and :10144.
        %      :10142 is that line's continuation and :10143 is a running page
        %      header ('FILTER BANK APPROACH ... 707'), not code -- the old
        %      citation of :10143-10144 started on the header and dropped the
        %      line that defines XS, XNPP and C1..C5.  Spot-checked 2026-08-24.
        XS = TGO/TAU;
        TOP  = 6.*XS*XS*(exp(-XS)-1.+XS);
        BOT1 = 2*XS*XS*XS + 3. + 6.*XS - 6.*XS*XS;
        BOT2 = -12.*XS*exp(-XS) - 3.*exp(-2.*XS);
        XNPP = TOP/(.0001 + BOT1 + BOT2);       % .0001 is the listing's own 0/0 guard
        C1 = XNPP/(TGO*TGO);
        C2 = XNPP/TGO;
        C3 = XNPP*(1.-cos(WHPZ*TGO))/(WHPZ*WHPZ*TGO*TGO);
        C4 = -XNPP*(exp(-XS)+XS-1.)/(XS*XS);
        C5 = XNPP*(WHPZ*TGO-sin(WHPZ*TGO))/(WHPZ*WHPZ*WHPZ*TGO*TGO);
        XNC = C1*YHPZ + C2*YDHPZ + C3*YTDDHPZ + C4*XNL + C5*YTDDDHPZ;
        if XNC >  XLIM, XNC =  XLIM; end        % repair 4: the symmetric pair
        if XNC < -XLIM, XNC = -XLIM; end

        t(n) = T;  prob(n,:) = PROB;  probb(n,:) = PROBB;  whpz(n) = WHPZ;
        ytddg(n) = YTDD/32.2;  ytddhpzg(n) = YTDDHPZ/32.2;
    end
end

% Built field by field, NOT with struct(...): struct('PHI', PHI) with PHI a
% 1x3 cell silently returns a 1x3 STRUCT ARRAY, and every later S.field would
% then be a comma-separated list rather than the vector it looks like.
S = struct();
S.t = t(1:n);  S.prob = prob(1:n,:);  S.probb = probb(1:n,:);
S.whpz = whpz(1:n);  S.ytddg = ytddg(1:n);  S.ytddhpzg = ytddhpzg(1:n);
S.miss_ft = abs(Y);
S.PHI = PHI;
end


% =============================================================================
function [PHI, Q] = weaveMatrices(w, Ts, phis)
%WEAVEMATRICES  The fundamental and process-noise matrices of the four-state
%   linear weave filter, zarchan.txt:10080-10105.  Q(1,1) keeps the book's .333
%   for 1/3 (quirk d).  States: [y, ydot, yTddot, yTdddot].
x = w*Ts;  s = sin(x);  c = cos(x);  s2 = sin(2*x);

PHI = eye(4);
PHI(1,2) = Ts;        PHI(1,3) = (1-c)/(w*w);   PHI(1,4) = (x-s)/(w*w*w);
PHI(2,3) = s/w;       PHI(2,4) = (1-c)/(w*w);
PHI(3,3) = c;         PHI(3,4) = s/w;
PHI(4,3) = -w*s;      PHI(4,4) = c;

Q = zeros(4,4);
Q(1,1) = phis*(.333*x^3 - 2*s + 2*x*c + .5*x - .25*s2)/(w^5);
Q(1,2) = phis*(.5*x*x - x*s + .5*s*s)/(w^4);              Q(2,1) = Q(1,2);
Q(1,3) = phis*(s - x*c - .5*x + .25*s2)/(w^3);            Q(3,1) = Q(1,3);
Q(1,4) = phis*(c + x*s - .5*s*s - 1)/(w*w);               Q(4,1) = Q(1,4);
Q(2,2) = phis*(1.5*x - 2*s + .25*s2)/(w^3);
Q(2,3) = phis*(1 - c - .5*s*s)/(w*w);                     Q(3,2) = Q(2,3);
Q(2,4) = phis*(s - .5*x - .25*s2)/w;                      Q(4,2) = Q(2,4);
Q(3,3) = phis*(.5*x - .25*s2)/w;
Q(3,4) = .5*phis*s*s;                                     Q(4,3) = Q(3,4);
Q(4,4) = w*phis*(.5*x + .25*s2);
end


% =============================================================================
function z = lcgNormal(seed, n)
%LCGNORMAL  n standard normals from a MINSTD (Lehmer) stream + Box-Muller.
%   Base MATLAB, deterministic, and it does not touch the caller's global
%   random stream -- see quirk (g).  x <- 16807*x mod (2^31-1); 16807*x < 2^53,
%   so every product is exact in double.  x = 0 is absorbing, hence the +1.
m = 2147483647;
x = mod(floor(abs(seed)), m-1) + 1;
u = zeros(1, 2*n);
for k = 1:2*n
    x = mod(16807*x, m);
    u(k) = x/m;                          % in (0,1): x is never 0 and never m
end
u1 = u(1:n);  u2 = u(n+1:2*n);           % SEPARATE BLOCKS, not consecutive pairs
z  = sqrt(-2*log(u1)) .* cos(2*pi*u2);
end


% =============================================================================
function K = structuralChecks(opts, S)
%STRUCTURALCHECKS  Self-consistency.  These are NOT book values and must never
%   be quoted as evidence that the port reproduces Zarchan; they are the weaker
%   claim that it agrees with itself, which is what catches wiring faults.
names = {};  oks = [];  detail = {};

% 1  PHI_i is exactly the matrix exponential of the weave dynamics.  The book
%    prints PHI element by element; if a transcribed element is wrong the run
%    is still smooth and still plausible, and only this notices.
dmax = 0;
for i = 1:3
    w = opts.WBANK(i);
    F = [0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 -w*w 0];
    dmax = max(dmax, max(max(abs(S.PHI{i} - expm(F*opts.TS)))));
end
names{end+1} = 'PHI_i == expm(F_i*TS), all three filters';
oks(end+1)   = dmax <= 1e-12;
detail{end+1} = sprintf('max |PHI - expm| = %.3e', dmax);

% 2  Probabilities finite and in [0,1].  If every likelihood underflows to
%    zero the listing divides 0/0 and NaN spreads silently through the
%    guidance law; nothing else in this file would report it.
p = S.prob;
nbad = sum(~isfinite(p(:)) | p(:) < -1e-12 | p(:) > 1+1e-12);
names{end+1} = 'probabilities finite and within [0,1] at every sample';
oks(end+1)   = nbad == 0 && ~isempty(p);
detail{end+1} = sprintf('%d of %d entries out of range or non-finite', nbad, numel(p));

% 3  DELETE EXPERIMENT on quirk (a), BOTH WAYS.  Same residuals, two
%    recursions: the book's printed simultaneous equation must sum to 1, and
%    the listing's in-place ordering must NOT.
%
%    ⚠️ THE SECOND CLAUSE IS THE POINT, and it was missing until 2026-08-24.
%    DELETE EXPERIMENT, RUN: replace the in-flight loop with the hoisted
%    PROB = PROB.*F/sum(PROB.*F) -- i.e. make the port implement the printed
%    equation instead of the listing -- and under the pre-2026-08-24 scoring
%    EVERY ROW STAYED GREEN.  Measured on that patched file: old selfcheck 3
%    (shadow only) 2.220e-16, pass; old benchmark 4 (shadow vs 1, band 1e-12)
%    2.220e-16, pass; old benchmark 5 (FINAL |sum-1|, band 1e-9) 0.000e+00,
%    pass; selfchecks 4 and 5 untouched, pass.  The port would have stopped
%    reproducing zarchan.txt:10138 with a full green board.  With the second
%    clause the same patch reads "0 below, 0 above of 1000" and goes red.
%    1e-12 is round-off, so the clause says "different algebra", not "a bigger
%    number" -- and the listing's worst |sum-1| falls from 1.177e-01 to
%    2.220e-16 the moment the ordering is tidied away.
sb    = sum(S.probb, 2);
sl    = sum(S.prob,  2);
dev_b = max(abs(sb - 1));
dev_l = max(abs(sl - 1));
nlo   = sum(sl < 1 - 1e-12);
nhi   = sum(sl > 1 + 1e-12);
names{end+1} = 'shadow sums to 1 AND the in-flight listing does not (:10063-10065 vs :10138)';
oks(end+1)   = dev_b <= 1e-12 && nlo > 0 && nhi > 0;
detail{end+1} = sprintf(['shadow %.3e; listing %.3e, leaving 1+/-1e-12 on ' ...
    'both sides (%d below, %d above of %d)'], dev_b, dev_l, nlo, nhi, numel(sl));

% 4  DELETE EXPERIMENT on identification.  Move the truth onto a DIFFERENT
%    member of the bank and the winner must move with it.  Without this the
%    identification rows below pass for a port that always favours filter 2.
[~, iw] = min(abs(opts.WBANK - opts.WREAL));
j  = find(1:3 ~= iw, 1, 'last');
S2 = runFlight(opts, opts.WBANK(j), opts.seed);
[~, win2] = max(S2.prob(end,:));
names{end+1} = 'identification follows the truth (WREAL moved onto another bank member)';
oks(end+1)   = win2 == j;
detail{end+1} = sprintf('truth %.3g r/s -> filter %d wins with p = %.4f', ...
    opts.WBANK(j), win2, max(S2.prob(end,:)));

% 5  WHICH OF THE TWO BOOK TEXTS THE ORDERING IMPLEMENTS, stated exactly and
%    isolated from the flight.  The listing's line 1 divides by the ALL-OLD
%    denominator, so p1' is the printed equation's p1 BITWISE -- not to a
%    tolerance, the same two floating-point operations in the same order.
%    Lines 2 and 3 divide by partially-updated denominators, so they must
%    DIFFER from it.  Both clauses are threshold-free, which is why this
%    replaced the old "spread F: |sum-1| > 1e-3" form on 2026-08-24: 1e-3 was
%    an invented edge, and equality/inequality needs none.
%
%    What each clause catches, RE-MEASURED 2026-08-24 (the previous version of
%    this paragraph put one of the three faults under the wrong clause):
%      * hoisting the sum out of the loop -- and equally, precomputing ONE
%        shared denominator -- makes lines 2 and 3 agree with the equation
%        (clause 2 red, both reading 0.000e+00).  Line 1 in both cases is
%        STILL exactly the printed equation, so clause 1 stays green; it is
%        not what catches these.
%      * running the loop BACKWARDS is what reaches line 1: it then divides by
%        a denominator already updated by lines 3 and 2, giving a line-1 delta
%        of 1.218e-02 (clause 1 red).
%    The third clause is benchmark 5's mechanism -- coincident likelihoods
%    collapse every denominator onto the same D, so the sum is exactly 1.
p0   = [0.2 0.5 0.3];
Fsp  = [1 9 3];
pl   = seqBayes(p0, Fsp);                 % the listing's three lines
pb   = p0.*Fsp/sum(p0.*Fsp);              % the printed equation, same inputs
d_eq = abs(sum(seqBayes(p0, [7 7 7])) - 1);
names{end+1} = 'line 1 is bitwise the printed equation, lines 2-3 are not (:10138)';
oks(end+1)   = pl(1) == pb(1) && pl(2) ~= pb(2) && pl(3) ~= pb(3) && d_eq == 0;
detail{end+1} = sprintf(['line1 delta %.3e (bitwise equal: %d); lines 2,3 ' ...
    'differ by %.3e, %.3e; equal-F |sum-1| = %.3e'], ...
    abs(pl(1)-pb(1)), pl(1) == pb(1), abs(pl(2)-pb(2)), abs(pl(3)-pb(3)), d_eq);

K = struct();
K.names  = names;
K.oks    = logical(oks);
K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    k = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{k}, detail{k});
end
end


% =============================================================================
function p = seqBayes(p, F)
%SEQBAYES  The listing's three probability lines (zarchan.txt:10138), in place
%   and sequential, on given likelihoods.  Same arithmetic as the loop inside
%   runFlight -- written out here so selfcheck 5 exercises the ORDERING rather
%   than re-calling the code it is meant to characterise.
for i = 1:numel(p)
    p(i) = p(i)*F(i)/sum(p.*F);
end
end


% =============================================================================
function B = benchmarks(R, opts)
%BENCHMARKS  Ten rows.  Book values carry their zarchan.txt line; figure
%   CLAIMS become orderings and bounds because no figure survived extraction.
B = emptyBench();
t = R.t;  Pr = R.prob;
[~, iw] = min(abs(opts.WBANK - opts.WREAL));      % index of the correct filter
oth = find(1:3 ~= iw);

% 1-2  Table 27.1 against the listing's ft/s^2 constants.  Cheap, and they are
%      what catches a mistyped 96.6 or 322 before any shape row notices.
v = opts.XNT/32.2;
B = addRow(B, 'value', 'target acceleration nT', 3, '3', v, sprintf('%.6f',v), ...
    '1e-9', 1e-9, abs(v-3) <= 1e-9, false, ...
    'the listing sets XNT = 96.6 ft/s^2 and 96.6/32.2 = 3 exactly', ...
    'Table 27.1 p.691 (zarchan.txt:9939)', 'g', '');
v = opts.XLIM/32.2;
B = addRow(B, 'value', 'missile acceleration limit nLIM', 10, '10', v, sprintf('%.6f',v), ...
    '1e-9', 1e-9, abs(v-10) <= 1e-9, false, ...
    'XLIM = 322 ft/s^2 and 322/32.2 = 10 exactly', ...
    'Table 27.1 p.691 (zarchan.txt:9940)', 'g', '');

% 3  p0(i) = 1/r against the listing's rounded .333 (quirk b).
tol3 = (0.5/100)/3;
B = addRow(B, 'value', 'initial probability p0(i) = 1/3', 1/3, '0.333333', ...
    opts.PROB0, sprintf('%.6f',opts.PROB0), '0.5%', tol3, ...
    abs(opts.PROB0 - 1/3) <= tol3, false, ...
    'the equation says 1/r; the code rounds it, so the three start at 0.999 not 1', ...
    'eq. p.699 (zarchan.txt:10069-10073); code at zarchan.txt:10107', '-', '');

% ---- the fixed seed ensemble that rows 5, 6 and 7 stand on.  Run once.
E  = seedEnsemble(opts, opts.WBANK);
E2 = seedEnsemble(opts, [1 2.5 3]);   % Fig. 27.14's off-truth bank (:10191)

% 4  INFO, NOT SCORED -- AND THAT IS THE FINDING.  The printed equation at
%    zarchan.txt:10063-10065 is p_i <- p_i f_i / sum_j p_j f_j: a vector
%    divided by its own sum.  Its sum is 1 BY ALGEBRA, which is exactly what
%    the book says at :10067 -- "we can see FROM THE PRECEDING SET OF
%    EQUATIONS".  Scoring our shadow of it against 1 measures floating-point
%    round-off and nothing else, so the row is on the board for the number and
%    the citation, not for credit.  It was scored green until 2026-08-24.
s     = sum(Pr,2);
dev_b = maxOr(abs(sum(R.prob_bayes,2) - 1));
dev_l = maxOr(abs(s - 1));
B = addRow(B, 'info', 'p(1)+p(2)+p(3) == 1 under the printed EQUATION', ...
    1, '1 (by algebra, not by measurement)', 1+dev_b, ...
    sprintf('%.16f (dev %.3e = round-off)', 1+dev_b, dev_b), ...
    'not scored', NaN, false, true, ...
    sprintf(['UNSCORED ON PURPOSE.  The shadow recursion is ' ...
     'PROBB = PROBB.*F/sum(PROBB.*F); a vector divided by its own sum has ' ...
     'sum 1 for every input, so this row CANNOT FAIL and reproduces ' ...
     'nothing -- the %.3e is round-off.  It keeps a board line because the ' ...
     'comparison beside it is the real finding: the LISTING at ' ...
     'zarchan.txt:10138 is a DIFFERENT recursion and its sum on this same ' ...
     'flight ranges %.6f..%.6f, worst |sum-1| = %.3e.  So :10068 and :10138 ' ...
     'disagree with each other; this port reproduces the CODE, and row 5 ' ...
     'scores it there.  Demoted from a scored 1e-12 row on 2026-08-24: it ' ...
     'was green, it made the port look checked, and it certified nothing.'], ...
     dev_b, minOr(s), maxOr(s), dev_l), ...
    'eq. p.699 (zarchan.txt:10068); the equations it follows from at zarchan.txt:10063-10065', ...
    '-', '');

% 4b THE ROW THAT REPLACED IT, AND IT CAN GO RED.  Bayes' rule divides by a
%    sum over ALL THREE likelihoods, so the update is homogeneous of degree
%    zero in f: a factor common to f(1), f(2), f(3) must cancel.  That is
%    precisely why quirk (c) -- the book's 6.28 where 2*pi belongs, at
%    zarchan.txt:10137 -- is harmless, and until 2026-08-24 that was an
%    assertion in this file's comment rather than a measurement.
%
%    Two clauses, both threshold-free:
%      (i)  a common factor that is a POWER OF TWO is exact in IEEE double, so
%           the whole flight must come back BITWISE identical -- band 0, not a
%           tolerance.  The 6.28 -> 2*pi substitution itself is not a power of
%           two, so it is scored at round-off (1e-12) alongside.
%      (ii) an UNEQUAL factor must MOVE the answer.  Without this clause a
%           FSCALE that nothing read would pass clause (i) trivially -- quirk
%           (e)'s dead switch, which this file has been bitten by before.
%    ⚠️ THE THREE EXPERIMENTS COMPOSE WITH opts.FSCALE, they do not replace it.
%    Written the other way the row read RED for any caller who set FSCALE at
%    all -- it would have been comparing that caller's flight against an
%    unscaled one and calling the difference a broken invariance.  Composing
%    means the row asks the right question at every setting: does ANOTHER
%    common factor on top of yours move anything.
kpow = 1024;                                   % 2^10: exact in binary64
o    = opts;  o.render = false;  o.benchmark = false;  o.assert = false;
o.FSCALE = opts.FSCALE * kpow;      Spow = runFlight(o, opts.WREAL, opts.seed);
o.FSCALE = opts.FSCALE * sqrt(6.28/(2*pi));   % algebraically the 2*pi likelihood
Spi  = runFlight(o, opts.WREAL, opts.seed);
o.FSCALE = opts.FSCALE .* [1 2 1];  Ssplit = runFlight(o, opts.WREAL, opts.seed);
d_pow   = maxOr(abs(Spow.prob(:)   - Pr(:)));
d_pi    = maxOr(abs(Spi.prob(:)    - Pr(:)));
d_split = maxOr(abs(Ssplit.prob(:) - Pr(:)));
% SUBNORMALS ARE THE ONE EXEMPTION, and it is IEEE's, not this port's.  Scaling
% by a power of two is exact only while the operand stays normalised; once a
% probability underflows past realmin, gradual underflow makes it inexact.  On
% the book's own case nothing underflows and d_pow is 0.000e+00 -- measured, and
% that is what the row reports.  A caller who drives one filter to ~1e-320 (try
% 'FSCALE', [1 2 1]) gets d_pow = 7.905e-323, and calling that a broken
% invariance would be blaming the recursion for binary64.  The exemption cannot
% hide a real defect: the smallest injected fault this row is claimed to catch,
% a 1e-16 epsilon in the denominator, reads 2.331e-12 -- 290 orders above the
% line.
pow_ok = d_pow == 0 || d_pow < realmin;
B = addRow(B, 'shape', 'a common factor on f(1..3) cannot move a probability', ...
    NaN, 'p identical under a common scale, different under an unequal one', ...
    d_pow, sprintf(['common 2^10: %.3e (0, or subnormal, required); 2*pi: ' ...
        '%.3e (<= 1e-12); unequal: %.3e (> 0 required)'], ...
        d_pow, d_pi, d_split), 'invariance', NaN, ...
    pow_ok && d_pi <= 1e-12 && d_split > 0, false, ...
    ['scores quirk (c) instead of asserting it. WHAT IT CATCHES, measured by ' ...
     'injection on 2026-08-24: deleting the FSCALE line entirely (clause ii ' ...
     'reads 0.000e+00 instead of 8.30e-01) and an additive epsilon in the ' ...
     'Bayes denominator, which breaks homogeneity -- 1e-16 gives 2.33e-12 ' ...
     'against a required 0, 1e-12 gives 2.33e-08. WHAT IT DOES NOT: an ' ...
     'epsilon of 1e-30 rounds away below the denominator''s own eps and this ' ...
     'row stays green, correctly -- it changes no answer; and a per-filter ' ...
     'constant baked into every run stays homogeneous and passes here, which ' ...
     'is selfcheck 1''s job. Clause (ii) is the anti-dead-switch clause: it ' ...
     'is what fails if FSCALE stops being read.'], ...
    'likelihoods at zarchan.txt:10137; the Bayes denominator at zarchan.txt:10063-10065', ...
    '-', sprintf(['the 6.28 -> 2*pi substitution moves the miss by %.2e ft ' ...
        '(%.3f ft -> %.3f ft), so the book''s rounded constant is harmless ' ...
        'to the flight, not just to the algebra'], ...
        abs(Spi.miss_ft - R.miss_ft), R.miss_ft, Spi.miss_ft));

% 5  WHAT THE LISTING CONSERVES INSTEAD, SCORED OVER THE SEED ENSEMBLE.  The
%    in-place recursion breaks the identity during identification and recovers
%    it exactly once one filter dominates: every denominator collapses back
%    onto the common D when p_j*(F_j - Fbar) -> 0.  The RECOVERY is the
%    scoreable invariant; the transient is printed beside it, not tuned away.
%
%    TWO CHANGES ON 2026-08-24.  (i) It was a single-draw row; it now takes the
%    worst final deviation over the 12 fixed seeds.  (ii) The band went from
%    1e-9 to 1e-12 -- TIGHTENED, never widened.  The claim is EXACT recovery
%    and the ensemble worst is eps/2 = 1.11e-16, so 1e-9 was seven orders of
%    slack; 1e-12 is this file's round-off idiom and still leaves four.
[~, ix]  = max(abs(s - 1));
dev_endE = maxOr(E.findev);
nlo = sum(s < 1 - 1e-12);  nhi = sum(s > 1 + 1e-12);
B = addRow(B, 'value', 'the LISTING recovers that identity once the bank identifies', ...
    1, '1', 1+dev_endE, sprintf('%.15f', 1+dev_endE), '1e-12', 1e-12, ...
    dev_endE <= 1e-12 && ~isempty(s), false, ...
    ['the listing overwrites PROB1 before PROB2''s denominator reads it, so ' ...
     'the sum is not 1 while the three probabilities are comparable. The ' ...
     'excursion is TWO-SIDED, which rules out drift and a missing constant ' ...
     'factor, and it is exact in the two cases where all three denominators ' ...
     'coincide: equal likelihoods (selfcheck 5 forces it) and a concentrated ' ...
     'posterior. Line 1 is exact Bayes throughout, so p(1) is never the ' ...
     'guilty term. PROB0 = .333 also starts the sum at 0.999 (row 3). ' ...
     'RECOVERY TRACKS CONCENTRATION, and the band has a real edge: on Fig. ' ...
     '27.14''s off-truth bank [1 2.5 3], where 2.5 and 3 keep competing, the ' ...
     'worst final deviation over the same seeds is 5.15e-09 -- this row ' ...
     'would be RED there. That is the mechanism, measured, not a caveat.'], ...
    'code at zarchan.txt:10138, against the equation at zarchan.txt:10068', '-', ...
    sprintf(['worst over %d seeds; on the default draw the transient worst ' ...
             'is %.3e at t = %.2f s, range %.6f..%.6f (%d below 1, %d above), ' ...
             'decaying to %.2e by t = 3.5 s and %.2e by 5 s. Off-truth bank ' ...
             '[1 2.5 3]: final %.3e, transient %.3e'], ...
             numel(E.seeds), dev_l, tOr(t, ix), minOr(s), maxOr(s), nlo, nhi, ...
             maxOr(abs(s(t >= 3.5) - 1)), maxOr(abs(s(t >= 5) - 1)), ...
             maxOr(E2.findev), maxOr(E2.maxdev)));

% 6  INFO, NOT SCORED.  Fig. 27.7's timing, where the book contradicts itself:
%    text "after 2 s" (:10152) against caption "after 3 s" (:10157).
%
%    ⚠️⚠️ THIS ROW USED TO SCORE THE CAPTION and this file used to say the run
%    "adjudicates rather than the editor".  It does not.  Over the 12 fixed
%    seeds the caption's reading holds on 6 and fails on 6; the prose's reading
%    holds on 1.  Scoring either one scores the draw, so NEITHER is scored.
%    Both are measured and printed.  The identification claim that DOES hold
%    across the ensemble is the book's own 3.5 s (row 7).
late  = t >= 3.0;
lead  = Pr(:,iw) > max(Pr(:,oth), [], 2);
nbad  = sum(late & ~lead);
early = t >= 2.0;
nbad2 = sum(early & ~lead);
tb    = t(early & ~lead);
nseed = numel(E.seeds);
B = addRow(B, 'info', 'Fig. 27.7 timing: text "after 2 s" vs caption "after 3 s"', ...
    NaN, 'the book states both; neither is scored', NaN, ...
    sprintf(['3 s reading holds on %d of %d seeds, 2 s reading on %d of %d; ' ...
        'default draw %d and %d violations'], ...
        sum(E.bad3 == 0), nseed, sum(E.bad2 == 0), nseed, nbad, nbad2), ...
    'not scored', NaN, false, true, ...
    sprintf(['UNSCORED ON PURPOSE.  The two book lines disagree and the run ' ...
     'cannot settle it: over the fixed ensemble the caption''s 3 s reading ' ...
     'holds on %d of %d seeds (violating-sample counts %s) and the prose''s ' ...
     '2 s reading holds on %d (seed(s) %s).  The DIRECTION of the verdict ' ...
     'flips with the draw, so a scored row here would be scoring a coin ' ...
     'flip -- it was scored green on the default seed until 2026-08-24.  On ' ...
     'THIS draw: t >= 3 s has %d violations and min p(filter %d) = %.4f; ' ...
     't >= 2 s has %d of %d violations, confined to %.2f..%.2f s, min p ' ...
     '= %.4f.  Fig. 27.7 is one nominal run and so is each of ours.  The ' ...
     'latest sample anywhere in the ensemble at which the correct filter is ' ...
     'not the strict leader is t = %.2f s.'], ...
     sum(E.bad3 == 0), nseed, mat2str(E.bad3), sum(E.bad2 == 0), ...
     mat2str(E.seeds(E.bad2 == 0)), nbad, iw, minOr(Pr(late,iw)), nbad2, ...
     sum(early), minOr(tb), maxOr(tb), minOr(Pr(early,iw)), maxOr(E.tlast)), ...
    'text p.708 (zarchan.txt:10152) vs caption Fig. 27.7 p.708 (zarchan.txt:10157)', ...
    '-', '');

% 7  "After 3.5 s the algorithm is almost certain", zarchan.txt:10154 -- the
%    book's third and only draw-robust timing statement.  0.95 is THIS PORT'S
%    rendering of "almost certain", chosen once and unchanged since; the book
%    prints no number.  SCORED OVER THE ENSEMBLE as of 2026-08-24, which is
%    where the identification claim now lives: worst-over-seeds, not best-case.
pminE = minOr(E.pmin35);
pmin  = minOr(Pr(t >= 3.5, iw));
B = addRow(B, 'bound', '"almost certain" after 3.5 s, on every seed', ...
    0.95, 'p(correct) >= 0.95', pminE, sprintf('%.4f (worst of %d seeds)', pminE, nseed), ...
    '>= 0.95', NaN, pminE >= 0.95 && isfinite(pminE), false, ...
    ['0.95 is this port''s rendering of "almost certain"; the book prints no ' ...
     'figure for it, and the band has not moved. What changed on 2026-08-24 ' ...
     'is the SAMPLE: the row was scored on one draw and is now scored on the ' ...
     'worst of 12 fixed seeds, so it cannot be green by luck. Margin is ' ...
     'thin on purpose -- doubling the seeker noise to SIGRIN = 2e-4 takes ' ...
     'the ensemble worst to 0.2028 and turns this row red.'], ...
    'text p.708 (zarchan.txt:10154)', '-', ...
    sprintf(['per-seed min p(correct) for t >= 3.5 s spans %.4f..%.4f; the ' ...
        'default draw gives %.4f. Latest sample in the ensemble at which the ' ...
        'correct filter is not the strict leader: t = %.2f s, i.e. inside ' ...
        'the book''s own 3.5 s by %.2f s'], ...
        minOr(E.pmin35), maxOr(E.pmin35), pmin, maxOr(E.tlast), 3.5 - maxOr(E.tlast)));

% 8  Fig. 27.8: "always quite close ... for the entire flight".  1 r/s is the
%    book's OWN yardstick for a small frequency error (zarchan.txt:10172), and
%    the bank spans +/-2 r/s about the truth, so it is a real narrowing.
%    ⚠️ SINGLE DRAW, like the book's own figure -- and unlike row 7 it does NOT
%    survive the ensemble.  Kept scored on the nominal run because that is the
%    book's protocol, but the spread is printed so it cannot be quoted as a
%    population result.
werr = max(abs(R.whpz - opts.WREAL));
B = addRow(B, 'bound', 'estimated weave frequency tracks truth all flight', ...
    1.0, 'max |w_hat - w| <= 1 r/s', werr, sprintf('%.4f', werr), '<= 1', NaN, ...
    werr <= 1.0, false, ...
    sprintf(['1 r/s is the book''s own yardstick for "less uncertainty" ' ...
     '(zarchan.txt:10172); the bank spans %.3g..%.3g about a truth of %.3g. ' ...
     '⚠️ NOT A POPULATION RESULT: over the 12 fixed seeds this bound fails ' ...
     'on %d of them.'], ...
     min(opts.WBANK), max(opts.WBANK), opts.WREAL, sum(E.werr > 1.0)), ...
    'text and caption Fig. 27.8 (zarchan.txt:10154,10156)', 'r/s', ...
    sprintf('over the ensemble max |w_hat - w| spans %.4f..%.4f r/s, exceeding 1 r/s on %d of %d seeds', ...
        minOr(E.werr), maxOr(E.werr), sum(E.werr > 1.0), nseed));

% 9  Fig. 27.9: "excellent estimates of the target acceleration after only
%    2 s" (caption says 3 s -- the second contradiction).  Scored as an
%    ORDERING against the same run's own start-up error, so no threshold has
%    to be invented for the word "excellent".
e    = R.ytddhpzg - R.ytddg;
rEar = rmsOf(e(t > 0 & t <= 1));
rLat = rmsOf(e(t >= 3));
B = addRow(B, 'shape', 'target acceleration estimate settles after 3 s', ...
    NaN, 'rms error over t >= 3 s at most 1/3 of the rms over the first 1 s', ...
    rLat, sprintf('%.4f vs %.4f g', rLat, rEar), 'ordering', NaN, ...
    isfinite(rEar) && isfinite(rLat) && rLat <= rEar/3, false, ...
    sprintf(['an ordering against the run''s own transient; "excellent" is an ' ...
     'adjective. ⚠️ SINGLE DRAW and CLOSE TO THE EDGE: the default ratio is ' ...
     '%.4f against the 1/3 = 0.3333 ordering, and over the 12 fixed seeds ' ...
     'the ratio spans %.4f..%.4f and breaks the ordering on %d of them. Like ' ...
     'row 8 this is kept on the nominal run because that is the book''s own ' ...
     'protocol, not because it is a population result.'], ...
     rLat/max(rEar,realmin), minOr(E.rmsrat), maxOr(E.rmsrat), sum(E.rmsrat > 1/3)), ...
    'text p.708 (zarchan.txt:10154); caption Fig. 27.9 says "after 3 s" (zarchan.txt:10166)', ...
    'g', sprintf(['target amplitude %.4g g, so the late rms is %.1f%% of it; ' ...
        'ratio %.4f vs the 0.3333 edge, ensemble %.4f..%.4f'], ...
        abs(opts.XNTREAL)/32.2, 100*rLat/max(abs(opts.XNTREAL)/32.2, realmin), ...
        rLat/max(rEar,realmin), minOr(E.rmsrat), maxOr(E.rmsrat)));

% 10 SKIP.  Not scoreable from this listing at any tolerance.
B = addRow(B, 'skip', 'rms miss distance usually less than 5 ft', ...
    5, '< 5 ft', NaN, 'n/a', 'n/a', NaN, false, true, ...
    ['needs the Monte Carlo of Fig. 27.10: flight times 0.5..10 s in 0.5 s ' ...
     'steps x 50 runs = 20 x 50 = 1000 complete flights with a fresh noise ' ...
     'draw each. Listing 27.1 is ONE flight at tF = 10 s and publishes no ' ...
     'miss, so there is nothing here to lay against 5 ft; "usually" is not a ' ...
     'threshold either. This run''s single miss was ' ...
     sprintf('%.3f ft', R.miss_ft) ' -- a draw, not an rms.'], ...
    'text p.709 (zarchan.txt:10163)', 'ft', '');
end


% =============================================================================
function E = seedEnsemble(opts, wbank)
%SEEDENSEMBLE  The same flight on a FIXED list of seeds, for the rows that must
%   not be green by luck.
%
%   ⚠️ THE SEED LIST IS HARD-CODED AND IS NOT AN OPTION.  Listing 27.1 is one
%   nominal flight and every figure in Chapter 27 is one draw, so single-draw
%   rows here are green or red on a coin.  Measured on 2026-08-24: the caption
%   reading of Fig. 27.7 holds on 6 of these 12 seeds and fails on 6, and the
%   Fig. 27.8 bound (row 8) fails on 4 -- the count row 8 itself computes and
%   prints; this comment said 5 until it was checked against the run.
%   An ensemble whose membership is a caller
%   argument is an ensemble that can be re-rolled until the board is green, so
%   the list is a constant of this file.  It is the eight seeds of the earlier
%   sweep plus 2024, 8675309, 2718281 and 161803; the first is the default.
%
%   Everything EXCEPT the seed and the bank follows opts, so a caller who
%   changes the flight sees the ensemble follow it.
%
%   Cost is 12 flights, about 0.17 s -- the price of rows 5, 6 and 7 meaning
%   something.  Two banks are run, so the port is about 26 flights in all.
E = struct();
E.seeds = [20260824 1 7 12345 999983 2024 555 31337 42 8675309 2718281 161803];
n = numel(E.seeds);
E.bank   = wbank;
E.bad3   = zeros(1,n);   % samples at t >= 3.0 s where the correct filter is not the leader
E.bad2   = zeros(1,n);   % the same at t >= 2.0 s
E.tlast  = zeros(1,n);   % latest sample at which it is not the leader (0 = never lost it)
E.pmin35 = zeros(1,n);   % min p(correct) over t >= 3.5 s
E.findev = zeros(1,n);   % |sum(p) - 1| at the FINAL sample -- the recovery claim
E.maxdev = zeros(1,n);   % worst |sum(p) - 1| over the flight -- the transient
E.werr   = zeros(1,n);   % max |w_hat - w| over the flight
E.rmsrat = zeros(1,n);   % rms accel error over t >= 3 s, relative to the first 1 s

o = opts;  o.WBANK = wbank;  o.FSCALE = [1 1 1];
[~, iw] = min(abs(wbank - opts.WREAL));
oth = find(1:3 ~= iw);
for q = 1:n
    S = runFlight(o, opts.WREAL, E.seeds(q));
    tq = S.t;  Pq = S.prob;
    lead = Pq(:,iw) > max(Pq(:,oth), [], 2);
    E.bad3(q)   = sum(tq >= 3.0 & ~lead);
    E.bad2(q)   = sum(tq >= 2.0 & ~lead);
    tl          = tq(~lead);
    if isempty(tl), E.tlast(q) = 0; else, E.tlast(q) = max(tl); end
    E.pmin35(q) = minOr(Pq(tq >= 3.5, iw));
    sq          = sum(Pq, 2);
    if isempty(sq)
        E.findev(q) = NaN;  E.maxdev(q) = NaN;
    else
        E.findev(q) = abs(sq(end) - 1);  E.maxdev(q) = max(abs(sq - 1));
    end
    E.werr(q)   = maxOr(abs(S.whpz - opts.WREAL));
    eq          = S.ytddhpzg - S.ytddg;
    E.rmsrat(q) = rmsOf(eq(tq >= 3)) / rmsOf(eq(tq > 0 & tq <= 1));
end
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  The shared schema mgs.zarchan.benchSuite consumes.  FIELD ORDER IS
%   PART OF THE CONTRACT -- adaptGeneric is strict and a reordered struct array
%   would make [B.ok] mean something else.  Every row goes through addRow so
%   the order cannot drift.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addRow(B, kind, what, book, book_str, ours, ours_str, band, tol, ...
                    ok, skipped, why, source, units, detail)
i = numel(B) + 1;
B(i).what = what;          B(i).kind = kind;
B(i).book = book;          B(i).book_str = book_str;
B(i).ours = ours;          B(i).ours_str = ours_str;
if isnumeric(book) && isscalar(book) && isnumeric(ours) && isscalar(ours) ...
        && isfinite(book) && isfinite(ours) && book ~= 0
    B(i).pct = 100*(ours - book)/book;
else
    B(i).pct = NaN;
end
B(i).band = band;          B(i).tol = tol;
B(i).ok = logical(ok);     B(i).skipped = logical(skipped);
B(i).why = why;            B(i).source = source;
B(i).units = units;        B(i).detail = detail;
end

function v = minOr(x)
if isempty(x), v = NaN; else, v = min(x); end
end

function v = maxOr(x)
if isempty(x), v = NaN; else, v = max(x); end
end

function v = tOr(t, i)
%TOR  t(i) guarded, so a caller who shortens the flight to nothing gets NaN in
%   a detail string rather than an index error out of a benchmark row.
if isempty(t) || i < 1 || i > numel(t), v = NaN; else, v = t(i); end
end

function v = rmsOf(x)
if isempty(x), v = NaN; else, v = sqrt(mean(x.^2)); end
end


% =============================================================================
function s = render(R, ev)
L = {'mgs.zarchan.mmaeWeaveFilters -- Zarchan Listing 27.1, three-filter fixed MMAE'};
L{end+1} = sprintf('  units: %s', R.units);
L{end+1} = sprintf('  bank %s r/s, truth %.3g r/s, tF %.3g s, TS %.3g s, seed %d, %d samples', ...
    mat2str(R.p.WBANK), R.p.WREAL, R.p.TF, R.p.TS, R.p.seed, R.p.n_samples);
if ~isempty(R.t)
    L{end+1} = sprintf('  final p = [%.4f %.4f %.4f], w_hat = %.4f r/s, miss = %.3f ft', ...
        R.prob(end,1), R.prob(end,2), R.prob(end,3), R.whpz(end), R.miss_ft);
end
L{end+1} = '';
for i = 1:numel(R.bench)
    L{end+1} = sprintf('  %s %-52s book %-14s ours %s', tick(R.bench(i).ok, ...
        R.bench(i).skipped), R.bench(i).what, R.bench(i).book_str, R.bench(i).ours_str); %#ok<AGROW>
end
L{end+1} = '';
for i = 1:numel(R.selfcheck.oks)
    L{end+1} = sprintf('  %s %-62s %s', tick(R.selfcheck.oks(i), false), ...
        R.selfcheck.names{i}, R.selfcheck.detail{i}); %#ok<AGROW>
end
if any(ev)
    L{end+1} = sprintf('\nEVALUATED %d rows, %d passed; %d structural checks, %d passed.', ...
        sum(ev), sum([R.bench(ev).ok]), numel(R.selfcheck.oks), sum(R.selfcheck.oks));
else
    L{end+1} = sprintf('\nNOTHING WAS EVALUATED.');
end
s = strjoin(L, newline);
end

function m = tick(ok, skipped)
if skipped, m = '..'; elseif ok, m = 'ok'; else, m = 'XX'; end
end
