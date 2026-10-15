function R = engage3d(opts)
%MGS.ZARCHAN.ENGAGE3D  Zarchan Listing 28.1 -- THREE-DIMENSIONAL tactical
%   missile-target engagement against a spiraling target, ported line for line.
%
%   R = mgs.zarchan.engage3d()                        % book run + benchmarks
%   R = mgs.zarchan.engage3d('benchmark', false, 'qpn', 0)   % one law, one sweep
%   R = mgs.zarchan.engage3d('benchmark', false, 'qpn', 1, 'rt3ic', 40000)
%
%   Book source in this repo:
%       matlab/zarchan/_source/zarchan.txt:10296   the LISTING 28.1 header
%       matlab/zarchan/_source/zarchan.txt:10297   .. 10323, the code (pp.719-723)
%       matlab/zarchan/_source/zarchan.txt:10211   .. 10295, the derivation
%       matlab/zarchan/_source/zarchan.txt:10325   the proportional-navigation result
%       matlab/zarchan/_source/zarchan.txt:10331   the compensated-weave result
%   Transcription this port was made from:
%       matlab/zarchan/_listings/listing_28_01.m
%
% ===========================================================================
%   *** UNITS ARE THE BOOK'S.  FEET, ft/s, ft/s^2, g = 32.2, deg via 57.3. ***
% ===========================================================================
%   Positions ft, speeds ft/s, accelerations ft/s^2, weave frequency rad/s,
%   time constants and flight times seconds.  The rest of mgs is metres and
%   this file is DELIBERATELY not, for the reason engage2d gives at length: a
%   miss distance of "18.3" is plausible in feet and in metres, so a silent
%   conversion does not crash, it produces a number that is wrong by 3.28x and
%   still reads fine.  Convert AT THE CALL SITE, visibly.
%
% ===========================================================================
%   WHY THIS PORT, OUT OF ALL OF THEM
% ===========================================================================
%   Every one of the ten Zarchan ports that came before this one is PLANAR.
%   We do not fly planar.  Measured over 23,954 ticks of real flight data, our
%   guidance kernel matches canonical three-dimensional pure PN with a
%   vertical-channel gamma-dot ratio of 1.000 and a lateral chi-dot ratio of
%   1.020 -- i.e. the thing we actually fly is a 3D law, and until now we had
%   no external check on the 3D part of it at all, only on a 1D shadow of it.
%
%   Listing 28.1 is the only place in the book where a THREE-DIMENSIONAL
%   guidance kernel is built and checked against published results.  Its
%   kernel is exactly the one we run:
%
%       ZEM       = RTM + VTM*tgo                         (zarchan.txt:10217)
%       ZEMPER    = ZEM - (ZEM . 1_RTM) 1_RTM             (perp to the LOS)
%       a_M       = N' * ZEMPER / tgo^2                   (zarchan.txt:10240ish)
%
%   That is PPN written in zero-effort-miss form.  It is not "PN done three
%   times in three planes"; the projection onto the plane perpendicular to the
%   line of sight is what makes it three-dimensional, and it is the step a
%   per-axis implementation silently omits.  If our lateral and vertical
%   channels were really two independent planar laws bolted together, THIS is
%   the listing that would show the difference.
%
%   WHAT THE CHAPTER ACTUALLY TEACHES, and it is not a number.
%   The target spirals: a_T1 = a_T sin(wt) in downrange, a_T2 = a_T cos(wt) in
%   altitude, a_T3 = 0 in cross-range (zarchan.txt:10285-10289).  The two
%   driven channels are therefore in QUADRATURE.  Each single-plane miss
%   oscillates with flight time -- which is the Ch.20 result, and the result
%   that makes single-plane weave analysis feel like a lottery -- but the
%   TOTAL, sqrt(m1^2 + m2^2), is CONSTANT.  Zarchan's own words:
%
%       "the peak miss distance for the single plane results will define the
%        radius of a circle or the three-dimensional miss distance for the
%        weaving target"                                  (zarchan.txt:10325)
%
%   For us that is a direct warning.  Every spread and miss statistic this
%   project has published against a maneuvering red heart has been computed in
%   ONE plane.  If the target's evasion has any circular component, our
%   single-plane numbers are sampling a circle at one phase angle and the
%   flight-time dependence we would read off them is an artifact.
%
% ===========================================================================
%   WHAT THIS DOES NOT MODEL, AND MUST NOT BE REPORTED AS IF IT DID
% ===========================================================================
%   No gravity.  No drag.  No thrust.  No angle-of-attack, no airframe, no
%   seeker, no noise, no estimator.  The missile is a POINT MASS whose
%   commanded acceleration is passed through ONE first-order lag per channel
%   (TAU = 1 s) and then added straight to the velocity vector.  In
%   particular:
%
%     * the acceleration is NOT constrained perpendicular to the missile
%       velocity.  It is perpendicular to the LINE OF SIGHT, which is a
%       different vector.  Missile SPEED is therefore not conserved; it drifts.
%       R.selfcheck.vm_drift measures how much (it is small here only because
%       the engagement is nearly head-on, so the LOS is nearly along the
%       velocity -- do not assume that in a crossing geometry).
%     * "6 g" is a demand.  There is no airframe that has to meet it, and the
%       book sets the limit to 9999999999 so nothing ever clips.
%     * our own vehicle's dominant failure -- 8 s of gamma-tracking lag with a
%       measured loop gain of 0.063 against a modelled 1.385 -- is represented
%       here by a single clean 1-s pole.  A miss distance computed by this file
%       is NOT a sortie prediction and never was.
%
% ===========================================================================
%   THE THREE UNRESOLVED COMPARISONS, RESOLVED, AND HOW WE KNOW
% ===========================================================================
%   The transcription header declares 3 variable-to-variable comparisons it
%   would not guess at.  They are these, and only these:
%
%       if AM1.XNCLIM, AM1=XNCLIM; end          (zarchan.txt:10310)
%       if AM2.XNCLIM, AM2=XNCLIM; end          (zarchan.txt:10314)
%       if AM3.XNCLIM, AM3=XNCLIM; end          (zarchan.txt:10316)
%
%   They are the ONLY three -- grep the transcription for a bare '.' between
%   two identifiers and nothing else comes back.  All three resolve to '>'.
%   Three independent reasons, any one sufficient:
%
%   (1) THE SUBSTITUTION TABLE IS FIXED, AND NOT ONLY IN THIS LISTING.  The OCR
%       maps '>' -> '.' and '<' -> ','.  Four comparisons in Listing 28.1 have
%       a LITERAL on the right and so were shape-resolvable, and all four
%       confirm the table:
%           while VC.=0      -> while VC >= 0        (:10298)
%           if RTM,1000      -> if RTM < 1000        (:10300)
%           while STEP,=1    -> while STEP <= 1      (:10300)
%           if S.=.09999     -> if S >= .09999       (:10319)
%       The same two forms recur across the whole book -- 'while STEP,=1'
%       appears in ~40 listings, 'if S.=.09999' and 'while VC.=0.' in a dozen
%       more -- and the reading is forced in every one of them by the
%       surrounding code.  Under that table 'AM1.XNCLIM' is 'AM1 > XNCLIM'
%       with nothing left to decide.  The companion line immediately after,
%       'if AM1,-XNCLIM' -> 'AM1 < -XNCLIM', completes the ordinary symmetric
%       clamp -- and note that the companion is NOT damaged in the same way,
%       so the pair would be asymmetric under any other reading.
%
%   (2) THE INVERTED READING DESTROYS THE BOOK RUN ON STEP ONE.  With the
%       book's XNCLIM = 9999999999 and '<' in place of '>', the very first
%       derivative evaluation (AM1 = 0) would set AM1 = 9999999999, and the
%       second line would then set it to -9999999999.  Every channel would be
%       pinned at -1e10 ft/s^2 forever and the simulation would return
%       garbage, not a 20-ft miss.  A run that produces sane numbers has
%       already ruled the inversion out.
%
%   (3) A DELETE-EXPERIMENT DISCRIMINATES IT, and it is wired into this file
%       rather than asserted in a comment.  R.selfcheck.clamp_alive_* reruns
%       one engagement with XNCLIM set to 2 g and reports what fraction of
%       derivative evaluations left AM strictly INSIDE the limit.  Under '>'
%       that fraction is large; under '<' it is zero by construction, because
%       every inside sample is exactly what the inverted test would overwrite.
%       R.selfcheck.clamp_inert_* is the matching control at the book's limit,
%       where the fraction must be exactly 1 and the number of modifications
%       exactly 0.  See [[promised-guards-that-do-not-exist]]: a comment
%       claiming a guard is not evidence that the guard exists.
%
%   NOTHING ELSE in the listing needed a decision.  Two further pieces of
%   transcription noise are handled and are NOT resolutions:
%     * '+. . .' at the ends of the ZEM lines is MATLAB's line continuation
%       '...' with spaces inserted by the extractor.  Read as '+ ...'.  Read
%       naively as '+.' followed by '. .' it is a syntax error, not a subtly
%       wrong model, so it cannot survive into a running port unnoticed.
%     * the very long semicolon-separated lines are the book's own layout.
%
% ===========================================================================
%   THE .0001 IN THE COMPENSATED-WEAVE DENOMINATOR IS LOAD-BEARING
% ===========================================================================
%       XNP = TOP / (.0001 + BOT1 + BOT2)                 (zarchan.txt:10306)
%
%   That guard looks like a lazy divide-by-zero dodge.  It is not, and this is
%   the single most mis-portable line in the listing.  With
%
%       TOP  = 6 x^2 (e^-x - 1 + x)
%       BOT  = 2x^3 + 3 + 6x - 6x^2 - 12x e^-x - 3 e^-2x
%
%   the series for BOT cancels to FIFTH order: every coefficient through x^4
%   is identically zero and BOT -> 0.3 x^5, while TOP -> 3 x^4.  Two separate
%   things go wrong as x -> 0, and the guard is aimed at both:
%
%   (1) THE LAW IS SINGULAR THERE.  TOP/BOT -> 10/x.  This is the ordinary
%       optimal-guidance blow-up at zero time to go, and it is real algebra,
%       not numerical trouble.
%   (2) BOT CANNOT BE COMPUTED THERE.  The terms being summed are of size 3,
%       so the roundoff floor is ~4e-16.  MEASURED: at x = 1e-3 the computed
%       BOT is 4.44e-16 against a true 3.00e-16 -- 48% wrong -- and at
%       x = 1e-4 it is EXACTLY 0.0.  Below x ~ 1e-3 the denominator is noise
%       with an arbitrary sign, so the unguarded ratio is a random number that
%       can be NEGATIVE: guidance that steers away from the target.
%
%   The .0001 handles both at once by FLOORING the denominator.  0.3 x^5 only
%   overtakes 1e-4 at x = 0.2016, so for the last 0.2*TAU seconds of every
%   flight the denominator is effectively the constant 1e-4 and XNP ~ 3e4 x^4
%   -> 0.  The compensated weave law does not saturate near intercept, it
%   TURNS ITSELF OFF.  Measured against the unguarded expression:
%
%       x       0.5     0.2      0.1      0.05     0.001
%       guarded 21.96   24.16     2.82     0.184    3e-8
%       raw     22.26   52.24   102.2    202.2    6753        (and Inf at 1e-4)
%
%   -- so the guard is inert where the law is meaningful and total where it is
%   not.  That is a real behaviour of the published code, not an artifact of
%   this port, and R.selfcheck.guard_* scores both halves of it (the roundoff
%   below, and the change in XNP above) so that a future reader who "cleans
%   up" the .0001 finds a failing row instead of a plausible answer.
%   R.weave_law tabulates XNP(x) guarded and unguarded across the whole range.
%
% ===========================================================================
%   WHERE THE BENCHMARK NUMBER COMES FROM.  IT IS NOT ON A FIGURE.
% ===========================================================================
%   Chapter 28 publishes its Listing 28.1 results as Figs. 28.1-28.5, and
%   every figure in the extracted text is an image link -- there are no digits
%   to read.  Two greppable things survive, and this port scores those:
%
%   (a) PROSE, zarchan.txt:10331 (p.724):
%           "the miss is reduced from nearly 20 ft to virtually zero"
%       That is the proportional-navigation total 3D miss ("nearly 20 ft") and
%       the compensated-weave total 3D miss ("virtually zero").  The 20 is a
%       number and is scored as a bound.  "Virtually zero" is an ADJECTIVE and
%       is NOT scored -- it is emitted as INFO, and the ORDERING it implies
%       (weave miss << PN miss) is scored instead.  Widening "virtually zero"
%       into a numeric tolerance would be inventing a book value.
%
%   (b) A CLOSED FORM in Ch.20, which the Ch.28 prose explicitly points at:
%           "the amplitude of the miss agrees with the single plane results of
%            Fig. 20.7 ... because identical inputs were used" (:10325)
%       Fig. 20.7's curve is unreadable but the formula behind it is printed:
%
%           Peak Miss / (nT T^2) |N'=3 = 32.2 x / (1 + x^2)^1.5   (:6093-6096)
%           x = wT * T                                            (:6091)
%
%       equivalently Peak Miss = a_T * w / (w^2 + 1/T^2)^1.5, which removes the
%       32.2 (it is only there to turn nT from g into ft/s^2) and is how this
%       file computes it.  At the Ch.28 nominal -- a_T = 193.2 ft/s^2, w = 3
%       rad/s, T = 1 s, N' = 3 -- that is
%
%           x = 3,   32.2*3/10^1.5 = 3.0548,   3.0548 * 6 * 1^2 = 18.33 ft
%
%       and 18.33 ft is "nearly 20 ft".  Two independent greppable routes to
%       the same number, which is why this port scores it as a value and not
%       merely as a bound.
%
%       THE FORMULA ITSELF IS VALIDATED, not assumed.  The book works three
%       numerical examples off it in prose (:6122-6131) and this port scores
%       all five published figures from them, so if the transcription of the
%       closed form were wrong the board would go red HERE, in Ch.20
%       provenance rows, rather than silently mis-scoring Ch.28.
%
%   THE BANDS, and what each one means:
%       'exact'   1e-12   a parameter echoed from prose; an identity
%       'text'    2%      printed in prose to 2-3 figures
%       'approx'  5%      the book says "approximately"
%       'graph'   15%     the author read it off Fig. 20.8 ("approximately 5.5")
%       'xchap'   15%     a NONLINEAR run of Listing 28.1 compared against the
%                         LINEAR closed form of Ch.20.  The book asserts they
%                         "agree" and quotes no digits, so 15% is the book's own
%                         precision, not a fitted tolerance.  The floor under it
%                         is the terminal step quantum, HFINE*VC = 0.8 ft on an
%                         18 ft miss = 4.4%, which no band can go below.
%   DO NOT WIDEN A BAND TO TURN A ROW GREEN.
%
% ===========================================================================
%   WHAT THE BOOK NOMINAL PRODUCES.  Regression reference.
% ===========================================================================
%   These were obtained from a bit-faithful transcription of the loop below,
%   run outside MATLAB before this file was written, so they are a check ON
%   this file rather than a record of it.  A default call should land here.
%
%     PROPORTIONAL NAVIGATION, tF in [4, 10] s
%       total 3D miss           18.659 ft   vs closed form 18.329   (+1.80%)
%       spread of the total      0.955 ft   = 5.12% of the mean
%                                            (the 0.8 ft quantum alone is 4.36%)
%       downrange amplitude     18.609 ft   frequency 2.9997 rad/s   r2 0.99999
%       altitude  amplitude     18.609 ft   frequency 2.9998 rad/s   r2 0.99979
%       phase difference        +90.11 deg  <- the quadrature the chapter is about
%       cross-range |miss3|max   1.803 ft   (NOT zero: closing-axis geometry)
%       peak demand              351 g      (there is no limit, by construction)
%     COMPENSATED WEAVE, same window
%       total 3D miss            0.731 ft   = 3.9% of the PN miss, and AT the
%                                            0.8 ft terminal quantum
%       peak demand               17.5 g    <- 20x less than PN, unremarked in
%                                            the text
%       N' range               2e-20 .. 31.0
%     DELETE EXPERIMENTS
%       aT = 0                   0.800 ft   exactly one closing step
%       XNCLIM = 2 g            58.3% of evaluations clamp, 41.7% strictly
%                               inside -> the '>' resolution is confirmed
%
%   THE THREE THINGS WORTH NOTICING IN THAT TABLE:
%     * the two component amplitudes agree to 4 significant figures and the
%       phase difference is 90.11 deg.  The book's circle is not approximate.
%     * the total miss is flat to 5.12% against a 4.36% numerical floor, so
%       Fig. 28.4's "approaches a constant" is exact to the resolution of the
%       integrator, not merely a trend.
%     * the cross-range miss is 1.8 ft, NOT zero.  It is 10% of the total and
%       is pure closest-approach geometry (axis 3 is the closing axis).  A
%       reader who scores "virtually zero" at one step quantum fails a correct
%       port; the row below scores it as a ratio and says why.
%
%   THREE THINGS THIS PORT REFUSES TO SCORE, and says so on the board:
%     * "virtually zero" (:10331, :10345) -- adjective.  INFO.
%     * "an initial transient period" (:10325) -- the book never says how long,
%       so the steady-state window is OUR choice (opts.tfss, default 4 s).  The
%       window is printed and its sensitivity is reported at 4 s and 5 s; a row
%       that only passes in one of them is not a book agreement.
%     * anything from Figs. 28.1-28.5 read as a magnitude.  There is nothing
%       there to read.
%
% ===========================================================================
%   BENCHMARK MODE IS A FLAG.  IT IS NOT nargin.
% ===========================================================================
%   In a function whose arguments block declares ONLY name-value options,
%   MATLAB reports nargin = 0 NO MATTER HOW MANY were passed (measured,
%   R2025b).  Three ports in the first wave used `if nargin == 0`: one
%   infinite-recursed and two silently replaced the caller's arguments with the
%   benchmark run.  'benchmark' is the ONLY switch here and it defaults TRUE,
%   so a bare call is a CHECKED call.
%
%   Benchmark mode does not override anything the caller passed.  Every book
%   value that depends on a parameter the caller moved is SKIPPED with the
%   reason printed.  That is why the arguments-block defaults are [] and the
%   book constants live in BOOKNOMINAL(): "the caller supplied this" is then
%   exactly "the field is non-empty", with no second copy of the constants to
%   drift out of agreement.
%
%   In benchmark mode BOTH guidance laws are swept (QPN = 1 and QPN = 0),
%   because the headline claim at :10331 is a comparison between them.  A
%   'qpn' passed by the caller therefore does NOT restrict the benchmark run;
%   it selects which sweep lands in R.run for a non-benchmark call.
%
%   COST: 2 laws x 80 engagements x ~2200 predictor/corrector steps, plus the
%   structural experiments.  Expect single-digit seconds, not milliseconds.
%
% ===========================================================================
%   OUTPUT
% ===========================================================================
%   R.pn, R.weave   one sweep each (QPN = 1, QPN = 0):
%                   .rt3ic .tf .miss1 .miss2 .miss3 .miss (sorted by flight
%                   time ASCENDING -- the book's loop counts range DOWN),
%                   .peak_g .peak_am .steps .vm_end .trace (longest run only)
%   R.fit           sinusoid fits of miss1(tf) and miss2(tf) over the
%                   steady-state window, FOR THE SWEEP THE CALLER SELECTED:
%                   .c1 .c2 .c3 each with .amp .omega .phase_deg .r2, plus
%                   .dphase_deg (the quadrature test), .window, .alt
%   R.fit_pn        the same, PINNED to the proportional-navigation sweep.
%                   This is the only one the benchmark board reads, so that
%                   'qpn', 0 cannot silently retitle the weave results as PN
%                   ones.  Identical to R.fit on a default call.
%   R.ch20          the closed-form reference: .x .norm .peak_miss_ft, the
%                   N'=3/4/5 formulas, and the book's own worked examples
%   R.weave_law     XNP(x) for the compensated law, guarded and unguarded
%   R.bench         computed vs book, one row per published value, each with
%                   .source (figure/page + zarchan.txt line) and .band
%   R.pass          true iff every EVALUATED row passed
%   R.selfcheck     structural invariants and their measured residuals,
%                   including the two clamp delete-experiments
%   R.cfg, R.offbook, R.summary, R.entry
%
%   Bench rows carry BOTH numeric (.book/.ours, NaN on shape rows) and text
%   (.book_str/.ours_str, always populated), matching the row schema
%   mgs.zarchan.benchSuite consumes via its adaptAutopilot path.
%
%   See also MGS.ZARCHAN.ENGAGE2D (Listing 2.1, the planar counterpart),
%   MGS.ZARCHAN.WEAVEDESIGNCURVE (Listing 20.3, the same weave family),
%   MGS.ZARCHAN.BENCHSUITE.

arguments
    % Defaults are [] ON PURPOSE -- see BOOKNOMINAL().  Empty means "use the
    % book value"; non-empty means "the caller chose this", which is what
    % gates the benchmark rows.  Do NOT replace these with literals: that puts
    % a second copy of every book constant in the file and destroys the only
    % mechanism this port has for telling an off-book run apart from a broken
    % one.
    opts.qpn     double {mustBeReal, mustBeFinite} = []  % QPN    1 = PN, 0 = compensated weave
    opts.tau     double {mustBeReal, mustBeFinite} = []  % TAU    [s]      one lag per channel
    opts.w       double {mustBeReal, mustBeFinite} = []  % W      [rad/s]  target weave frequency
    opts.at      double {mustBeReal, mustBeFinite} = []  % AT     [ft/s^2] target maneuver, 6 g
    opts.vt      double {mustBeReal, mustBeFinite} = []  % VT     [ft/s]   target speed (cross-range)
    opts.vm      double {mustBeReal, mustBeFinite} = []  % VM     [ft/s]   missile speed
    opts.xnp     double {mustBeReal, mustBeFinite} = []  % XNP    N' for PN
    opts.xnclim  double {mustBeReal, mustBeFinite} = []  % XNCLIM [ft/s^2] per-channel accel limit
    opts.rt3ic   double {mustBeReal, mustBeFinite} = []  % RT3IC  [ft]     initial separations swept
    % Discretization.  NOT model parameters, so they are NOT in the off-book
    % gate list -- except HFINE, which sets the terminal quantum HFINE*VC that
    % the cross-range and miss-floor rows are stated in.  That one row IS
    % gated on hfine, because moving hfine moves the criterion, not the answer.
    opts.hcoarse double {mustBeReal, mustBeFinite} = []  % H  [s] far from intercept
    opts.hfine   double {mustBeReal, mustBeFinite} = []  % H  [s] inside RFINE
    opts.rfine   double {mustBeReal, mustBeFinite} = []  % [ft] range at which H drops
    % The steady-state window.  OURS, not the book's -- see the header.  It is
    % printed on the board and its sensitivity is reported.
    opts.tfss    double {mustBeReal, mustBeFinite} = []  % [s] fit/statistics start
    opts.benchmark (1,1) logical = true    % THE ONLY SWITCH.  Not nargin.
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
    opts.trace     (1,1) logical = true    % keep the book's 0.1 s print cadence
end

GRAV_FT = 32.2;    % g, the book's value.  Not 9.80665, not 9.81.

N = bookNominal();
[cfg, offbook] = resolveConfig(opts, N, GRAV_FT);

R = struct();
R.units   = 'ft, ft/s, ft/s^2, rad/s, deg via 57.3, g = 32.2 -- Zarchan''s units, NOT SI';
R.listing = 'Zarchan Listing 28.1 (zarchan.txt:10296, pp.719-723)';
R.entry   = 'engage3d(''benchmark'',true,''assert'',false,''render'',false)';
R.opts    = opts;
R.cfg     = cfg;
R.offbook = offbook;
R.nominal = N;

% ---- the reference closed form, and the Ch.20 numbers that validate it ----
R.ch20 = ch20Reference(cfg, GRAV_FT);

% ---- the sweeps ----------------------------------------------------------
% Benchmark mode runs BOTH laws because the headline claim (:10331) is a
% comparison between them.  Off-benchmark runs only what was asked for, which
% is the difference between "check the port" and "use the port".
if cfg.benchmark
    R.pn    = sweep(setLaw(cfg, 1));
    R.weave = sweep(setLaw(cfg, 0));
    R.run   = pickLaw(R, cfg.qpn);
else
    R.run   = sweep(cfg);
    if cfg.qpn == 1
        R.pn = R.run;  R.weave = [];
    else
        R.weave = R.run;  R.pn = [];
    end
end

% ---- the shape analysis the chapter is actually about --------------------
% R.fit describes WHATEVER SWEEP THE CALLER ASKED TO LOOK AT.  R.fit_pn is
% pinned to the proportional-navigation sweep and is the ONLY one the board
% reads.  They are the same object on a default call, and that is exactly why
% they have to be separate names: with 'qpn', 0 and 'benchmark', true, a board
% built from R.fit would report the compensated-weave sweep under every
% proportional-navigation row heading and every number would still look
% reasonable.  A wrong answer that reads fine is the failure this package
% exists to stop.
R.fit       = fitSweep(R.run, cfg);
R.weave_law = weaveLawTable(cfg);

if cfg.benchmark
    if cfg.qpn == 1
        R.fit_pn = R.fit;          % same sweep; do not fit it twice
    else
        R.fit_pn = fitSweep(R.pn, cfg);
    end
    R.selfcheck = structuralCheck(R, cfg, GRAV_FT);
    R.bench     = benchmarks(R, cfg, offbook, GRAV_FT);
    ev          = ~[R.bench.skipped];
    R.pass      = any(ev) && all([R.bench(ev).ok]);
else
    R.fit_pn    = [];
    R.selfcheck = struct('skipped', true);
    R.bench     = emptyBench();
    ev          = false;
    R.pass      = false;   % NOT true.  Nothing was checked, and the summary says so.
end

R.summary = render(R, cfg, offbook);
if cfg.render, fprintf('%s\n', R.summary); end
if cfg.plot,   drawCurves(R, cfg); end

if cfg.assert && cfg.benchmark
    % "Nothing was evaluated" must NOT read as "everything passed".  A run
    % whose parameters put every book value out of reach has checked NOTHING,
    % and that is precisely the shape of green light this repo has been burned
    % by before ([[promised-guards-that-do-not-exist]]).
    if ~any(ev)
        error('mgs:zarchan:engage3d:noBenchmark', ...
            ['not one book value could be evaluated.  The published results ' ...
             'live at the Listing 28.1 nominal (w = 3 rad/s, aT = 193.2 ft/s^2, ' ...
             'VT = 1000, VM = 3000, TAU = 1 s, N'' = 3); this run moved %s.  ' ...
             'Refusing to hand back an unvalidated port silently -- pass ' ...
             '''benchmark'', false if you meant to run off-book.'], ...
            offbookStr(offbook));
    end
    if ~R.pass
        bad = find(~[R.bench.ok] & ev);
        error('mgs:zarchan:engage3d:benchmark', ...
            ['%d of %d evaluated book values are outside tolerance ' ...
             '(first: %s -- book %s, ours %s).  The full board is in ' ...
             'R.summary.  Pass ''assert'', false to inspect instead of throw.'], ...
            numel(bad), sum(ev), R.bench(bad(1)).what, ...
            R.bench(bad(1)).book_str, R.bench(bad(1)).ours_str);
    end
    if isfield(R.selfcheck, 'ok') && ~R.selfcheck.ok
        error('mgs:zarchan:engage3d:structural', ...
            ['a STRUCTURAL self-check failed: %s.  These are wiring ' ...
             'invariants, not book values -- a failure here voids every ' ...
             'benchmark row above it.'], R.selfcheck.first_fail);
    end
end
end


% =============================================================================
%   THE BOOK CONSTANTS.  One copy, here.
% =============================================================================
function N = bookNominal()
%BOOKNOMINAL  Every literal from the first line of Listing 28.1
%   (zarchan.txt:10297) and from the prose that describes it (:10293).
N.qpn     = 1;              % QPN=1 -> proportional navigation; QPN=0 -> compensated weave
N.tau     = 1.0;            % TAU    [s]      "a single-lag time constant of 1 s"      :10293
N.w       = 3.0;            % W      [rad/s]  "spiraling at 3 r/s"                     :10293
N.at      = 193.2;          % AT     [ft/s^2] "an acceleration level of 6 g" = 6*32.2  :10293
                            %        the book writes 193.2 as a literal, NOT 6*32.2, so
                            %        193.2/32.2 is 5.99999999999999928 in double and an
                            %        "exact" check on 6 g needs 1e-12, not 0.
N.vt      = 1000.0;         % VT     [ft/s]   "the target is traveling at 1000 ft/s"   :10293
N.vm      = 3000.0;         % VM     [ft/s]   "the missile is traveling at 3000 ft/s"  :10293
N.xnp     = 3.0;            % XNP    N' = 3   "set equal to three for proportional
                            %                  navigation"                            :10252
N.xnclim  = 9999999999.0;   % XNCLIM [ft/s^2] "has an infinite acceleration limit"     :10293
N.rt3ic   = 40000:-500:500; % the book's sweep, counting DOWN in range.  80 cases.
                            % At VC = VT + VM = 4000 ft/s this is tF = 10 s down to
                            % 0.125 s, which is why the listing's own plots are
                            % axis([0 10 ...]) (:10323).
N.hcoarse = 0.01;           % H  [s]  far out                                          :10300
N.hfine   = 0.0002;         % H  [s]  inside RFINE.  0.0002*4000 = 0.8 ft of closing
                            %         per step -- the floor under every miss here.
N.rfine   = 1000.0;         % [ft]    "if RTM < 1000"                                  :10300
N.tfss    = 4.0;            % [s]  OURS.  The book says "after an initial transient
                            %      period" and never quantifies it.  4 s is ~4 TAU and
                            %      leaves 49 of the 80 cases, ~2.9 weave periods, for
                            %      the sinusoid fit.  Sensitivity at 5 s is reported.
end


function [cfg, offbook] = resolveConfig(opts, N, GRAV_FT)
% ⚠️⚠️ GRAV_FT 必須**當引數傳進來**。第一版讓它讀主函式的區域變數 ——
%    MATLAB 的 local function **看不到**主函式的工作區,所以那是
%    `Unrecognized function or variable 'GRAV_FT'`,而且只在真的呼叫到
%    這一行時才炸。靜態檢查抓不到(它是個合法的識別字)。
%    ⚠️ 同一類的第二次:tot_node.py 的 `_weave_theta_of` 也用了一個
%    只存在於別的函式裡的 `GRAV`,而那一行從沒執行過所以從沒炸過。
%RESOLVECONFIG  Book value where the caller passed nothing; caller's value
%   otherwise; and a LIST of what they moved, which is what gates the board.
f = fieldnames(N);
cfg = struct();
offbook = {};
for i = 1:numel(f)
    k = f{i};
    if isempty(opts.(k))
        cfg.(k) = N.(k);
    else
        cfg.(k) = opts.(k);
        if ~isequal(cfg.(k), N.(k))
            offbook{end+1} = k; %#ok<AGROW>
        end
    end
end
cfg.benchmark = opts.benchmark;
cfg.render    = opts.render;
cfg.plot      = opts.plot;
cfg.assert    = opts.assert;
cfg.trace     = opts.trace;
cfg.grav      = GRAV_FT;

% ---- guards.  A function that cannot do what it was asked ERRORS. --------
if ~isscalar(cfg.qpn) || ~any(cfg.qpn == [0 1])
    error('mgs:zarchan:engage3d:qpn', ...
        ['qpn must be 1 (proportional navigation) or 0 (compensated weave); ' ...
         'got %s.  The listing branches on ''if QPN==1'' (:10304), so any ' ...
         'other value silently selects the WEAVE law -- a wrong guidance law ' ...
         'that still produces a plausible miss distance.'], mat2str(cfg.qpn));
end
if cfg.tau <= 0
    error('mgs:zarchan:engage3d:tau', ...
        'tau must be > 0; the guidance lag AM1D = (AM1P-AM1)/TAU divides by it.');
end
if cfg.w <= 0
    error('mgs:zarchan:engage3d:w', ...
        ['w must be > 0; the target initial condition VT1 = -AT/W (:10297) and ' ...
         'the weave ZEM terms divide by W^2 and W^3.']);
end
if cfg.vm + cfg.vt <= 0
    error('mgs:zarchan:engage3d:closing', ...
        ['VM + VT = %g ft/s is not a closing engagement, so the ''while VC >= 0'' ' ...
         'loop (:10298) never runs and every miss comes back as the initial ' ...
         'separation.'], cfg.vm + cfg.vt);
end
if cfg.vm < 400
    % Same guard, and the same reason, as engage2d: our own vehicle cruises
    % 157-210 m/s, and that bare number handed in here simulates a 200 ft/s
    % missile and still returns a plausible-looking miss distance.
    error('mgs:zarchan:engage3d:slowMissile', ...
        ['vm = %g is below 400 ft/s.  Listing 28.1 is in FEET; the likely ' ...
         'cause is an SI speed passed straight in.  Convert at the call site.'], ...
        cfg.vm);
end
if cfg.hfine <= 0 || cfg.hcoarse <= 0 || cfg.hfine > cfg.hcoarse
    error('mgs:zarchan:engage3d:step', ...
        ['need 0 < hfine <= hcoarse; got hfine = %g, hcoarse = %g.  The ' ...
         'listing drops the step INSIDE rfine (:10300) and getting that ' ...
         'backwards coarsens the only part of the flight where the miss is set.'], ...
        cfg.hfine, cfg.hcoarse);
end
if isempty(cfg.rt3ic)
    error('mgs:zarchan:engage3d:rt3ic', 'rt3ic is empty; there is nothing to fly.');
end
if any(cfg.rt3ic <= 0)
    error('mgs:zarchan:engage3d:rt3icSign', ...
        'rt3ic entries must be > 0; got a minimum of %g ft.', min(cfg.rt3ic));
end
if cfg.xnclim <= 0
    error('mgs:zarchan:engage3d:xnclim', ...
        ['xnclim must be > 0.  It is a MAGNITUDE limit applied symmetrically ' ...
         '(:10310-10318); a non-positive value pins every channel at the ' ...
         'limit instead of leaving it inert.']);
end
% The steady-state window has to leave enough of the sweep to fit a sinusoid
% to.  Silently fitting 3 points would produce an amplitude with r2 = 1 and no
% meaning at all.
cfg.tf_est = cfg.rt3ic ./ (cfg.vm + cfg.vt);
if sum(cfg.tf_est >= cfg.tfss) < 8
    error('mgs:zarchan:engage3d:window', ...
        ['the steady-state window tfss = %g s leaves only %d of %d cases, and ' ...
         'a 3-parameter sinusoid fit to that many points is not a measurement. ' ...
         'Lower tfss or extend rt3ic (the book sweeps to %g ft = %.3g s).'], ...
        cfg.tfss, sum(cfg.tf_est >= cfg.tfss), numel(cfg.rt3ic), ...
        max(cfg.rt3ic), max(cfg.tf_est));
end
end


function cfg = setLaw(cfg, qpn)
cfg.qpn = qpn;
end


function S = pickLaw(R, qpn)
if qpn == 1, S = R.pn; else, S = R.weave; end
end


% =============================================================================
%   LISTING 28.1, TRANSCRIBED.  This is the part that must not be improved.
% =============================================================================
function E = flyOne(RT3IC, C, wantTrace)
%FLYONE  One pass of the book's outer 'for RT3IC' loop: a single engagement,
%   flown to closest approach, returning the miss vector.
%
%   The variable names are the book's, in the book's CASE, deliberately.  A
%   reader with the page open should be able to run a finger down both.
%
%   THE INTEGRATOR IS ZARCHAN'S TWO-PASS PREDICTOR/CORRECTOR (Heun), written
%   in the listing with an unusual STEP/FLAG encoding that is worth spelling
%   out because it looks like a bug:
%
%       STEP=1; FLAG=0;
%       while STEP <= 1
%           if FLAG==1                 % <- skipped on the FIRST pass
%               STEP=2;                % <- the only thing that ends the loop
%               <Euler-advance every state by H using the derivatives just
%                computed, and advance T>
%           end
%           <evaluate all derivatives at the current state>
%           FLAG=1;
%       end
%       <trapezoidal corrector: x = .5*(xOLD + xPredicted + H*xdotPredicted)>
%
%   Pass 1 evaluates derivatives at t.  Pass 2 Euler-steps to t+H and
%   re-evaluates there.  The corrector then averages.  It is ordinary Heun;
%   only the loop control is odd.  Do NOT "simplify" it into a single pass --
%   the corrector's H*VT1 and H*AM1D terms are the SECOND pass's values, and a
%   one-pass rewrite silently becomes forward Euler with a halved step.
%
%   ONE ORDERING DETAIL THAT MATTERS AND LOOKS ARBITRARY.  The lag derivative
%   is computed BEFORE the clamp:
%       AM1D = (AM1P - AM1)/TAU;   then   if AM1 > XNCLIM, AM1 = XNCLIM; end
%   so the clamp acts on the STATE that has already been used to form its own
%   derivative, and the corrector afterwards averages a CLAMPED value with an
%   UNCLAMPED old one -- which means |AM| can finish a step slightly OUTSIDE
%   XNCLIM.  That is the book's code and this port keeps it.  It is the same
%   family of ordering trap as our own nz_clamp_after_floor
%   ([[nz-clamp-after-floor-refuted]]): the fix that "obviously" belongs there
%   changes the answer, and the honest thing is to measure the overshoot
%   rather than quietly remove it.  E.clamp_overshoot reports it.

QPN    = C.qpn;
TAU    = C.tau;
W      = C.w;
AT     = C.at;
VT     = C.vt;
VM     = C.vm;
XNCLIM = C.xnclim;
GRAV   = C.grav;

% XNP is set ONCE before the book's for-loop (:10297) and then overwritten on
% every derivative evaluation in the QPN==0 branch.  Re-initialising it PER
% ENGAGEMENT, as here, is not cosmetic: in the book only the leading `clear`
% stops a stale weave-law XNP from leaking into a later PN run, and a port
% that hoists this out of the loop reproduces exactly that contamination --
% a PN sweep silently flown at whatever navigation ratio the last weave step
% happened to leave behind.
XNP = C.xnp;

% ---- initial conditions, zarchan.txt:10297 -------------------------------
% Axis 1 = downrange, axis 2 = altitude, axis 3 = cross-range (:10289).
% The missile sits at 10,000 ft altitude flying +3 at VM; the target is
% RT3IC ahead on axis 3 flying -3 at VT, so the engagement is head-on along
% the cross-range axis and the spiral is entirely in the 1-2 plane.
RM1 = 0.;  RM2 = 10000.;  RM3 = 0.;
RT1 = 0.;  RT2 = 10000.;  RT3 = RT3IC;
% VT1 = -AT/W is not a heading, it is the phase-correct start of the circle:
% integrating AT1 = AT sin(Wt) from this value gives VT1(t) = -(AT/W) cos(Wt),
% a ZERO-MEAN oscillation.  Start it at 0 instead and the target acquires a
% steady -AT/W = -64.4 ft/s downrange drift, which is a completely different
% (and non-published) engagement that still looks like a spiral on a plot.
VT1 = -AT/W;  VT2 = 0.;  VT3 = -VT;
T = 0.;  S = 0.;
RTM1 = RT1 - RM1;  RTM2 = RT2 - RM2;  RTM3 = RT3 - RM3;
RTM  = sqrt(RTM1^2 + RTM2^2 + RTM3^2);
VM1 = 0.;  VM2 = 0.;  VM3 = VM;
VTM1 = VT1 - VM1;  VTM2 = VT2 - VM2;  VTM3 = VT3 - VM3;
VC = -(RTM1*VTM1 + RTM2*VTM2 + RTM3*VTM3)/RTM;
AM1 = 0.;  AM2 = 0.;  AM3 = 0.;

% Declared so the QPN==1 branch is not merely relying on them never being
% read.  In the book they are undefined in that branch and simply never
% referenced; making that explicit costs nothing and removes a trap for
% anyone who later moves a line into the wrong branch.
AT1D = 0.;  AT2D = 0.;  AT3D = 0.;
AM1D = 0.;  AM2D = 0.;  AM3D = 0.;
AT1  = 0.;  AT2  = 0.;  AT3  = 0.;
XNCG = 0.;  H = C.hcoarse;

VM0 = sqrt(VM1^2 + VM2^2 + VM3^2);

% ---- instrumentation.  None of this is in the book. ----------------------
nStep = 0;
nEval = 0;
% peakAM is sampled INSIDE the derivative loop, i.e. AFTER the clamp, so it is
% bounded by XNCLIM by construction.  The state can end a step higher than
% that -- see clampOver below and the INFO row it feeds.  Do not read peakAM
% as "the largest acceleration this missile ever had".
peakAM = 0;      % max over channels and time of |AM_i|, post-clamp
peakG  = 0;      % max of XNCG, the book's own total-g display
clampHits   = 0; % derivative evaluations where a clamp line actually fired
clampInside = 0; % derivative evaluations where all three |AM_i| < XNCLIM
clampOver   = 0; % worst |AM_i| - XNCLIM left behind by the corrector
xnpMin =  Inf;   % XNP excursion, so a PN run can PROVE the weave branch is cold
xnpMax = -Inf;
MAXSTEP = 5e6;   % a hang is a failure mode too; it must ERROR, not spin

if wantTrace
    nTrace = 0;
    cap = 4096;
    trT = nan(1, cap);  trRTM = nan(1, cap);  trG = nan(1, cap);
    trXNP = nan(1, cap); trTGO = nan(1, cap);
    trR = nan(3, cap);   trM = nan(3, cap);
end

% ---- the flight, zarchan.txt:10298-10321 ---------------------------------
while VC >= 0
    if RTM < C.rfine
        H = C.hfine;
    else
        H = C.hcoarse;
    end
    RT1OLD = RT1; RT2OLD = RT2; RT3OLD = RT3;
    RM1OLD = RM1; RM2OLD = RM2; RM3OLD = RM3;
    VM1OLD = VM1; VM2OLD = VM2; VM3OLD = VM3;
    VT1OLD = VT1; VT2OLD = VT2; VT3OLD = VT3;
    AM1OLD = AM1; AM2OLD = AM2; AM3OLD = AM3;
    STEP = 1;  FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            RT1 = RT1 + H*VT1;  RT2 = RT2 + H*VT2;  RT3 = RT3 + H*VT3;
            RM1 = RM1 + H*VM1;  RM2 = RM2 + H*VM2;  RM3 = RM3 + H*VM3;
            VM1 = VM1 + H*AM1;  VM2 = VM2 + H*AM2;  VM3 = VM3 + H*AM3;
            VT1 = VT1 + H*AT1;  VT2 = VT2 + H*AT2;  VT3 = VT3 + H*AT3;
            AM1 = AM1 + H*AM1D; AM2 = AM2 + H*AM2D; AM3 = AM3 + H*AM3D;
            T = T + H;
        end
        RTM1 = RT1 - RM1;  RTM2 = RT2 - RM2;  RTM3 = RT3 - RM3;
        RTM  = sqrt(RTM1^2 + RTM2^2 + RTM3^2);
        VTM1 = VT1 - VM1;  VTM2 = VT2 - VM2;  VTM3 = VT3 - VM3;
        % RTM == 0 is a literal direct hit and makes VC = 0/0 = NaN, which
        % makes 'while VC >= 0' false and ends the run with a NaN miss that
        % prints as a blank rather than as a failure.  Refuse it out loud.
        if ~(RTM > 0)
            error('mgs:zarchan:engage3d:degenerate', ...
                ['RTM reached %g ft at T = %g s: the LOS unit vector and VC ' ...
                 'are both 0/0 from here on.  This is a genuine direct hit ' ...
                 'at the step size in use, not a miss of zero.'], RTM, T);
        end
        VC  = -(RTM1*VTM1 + RTM2*VTM2 + RTM3*VTM3)/RTM;
        TGO = RTM/VC;
        % TGO feeds 1/TGO^2 in the guidance command.  VC == 0 exactly gives
        % Inf, and a NaN or Inf here propagates into AM and then into every
        % state without ever throwing.
        if ~isfinite(TGO)
            error('mgs:zarchan:engage3d:tgo', ...
                ['TGO = RTM/VC is not finite at T = %g s (RTM = %g ft, ' ...
                 'VC = %g ft/s).  The guidance command divides by TGO^2, so ' ...
                 'continuing would fill the states with NaN silently.'], ...
                T, RTM, VC);
        end
        % ---- target spiral, zarchan.txt:10285-10289 ----------------------
        AT1 = AT*sin(W*T);  AT2 = AT*cos(W*T);  AT3 = 0.;
        if QPN == 1
            % Proportional navigation.  ZEM = RTM + VTM*tgo, :10217.
            ZEM1 = RTM1 + VTM1*TGO;
            ZEM2 = RTM2 + VTM2*TGO;
            ZEM3 = RTM3 + VTM3*TGO;
        else
            % Compensated weave.  Target JERK is exact here because the
            % maneuver is analytic (:10289-10292); in the field it is not,
            % and that is the term that makes this law hard to fly.
            AT1D = AT*W*cos(W*T);  AT2D = -AT*W*sin(W*T);  AT3D = 0.;
            X    = TGO/TAU;
            TOP  = 6.*X*X*(exp(-X) - 1. + X);
            BOT1 = 2*X*X*X + 3. + 6.*X - 6.*X*X;
            BOT2 = -12.*X*exp(-X) - 3.*exp(-2.*X);
            % The .0001 is LOAD-BEARING, not a tidy-up.  BOT1+BOT2 cancels to
            % fifth order (-> 0.3*X^5) out of terms of size 3, so below
            % X ~ 1e-3 it is pure double-precision roundoff WITH RANDOM SIGN
            % (measured: 4.44e-16 against a true 3.00e-16 at X = 1e-3, and
            % EXACTLY 0.0 at X = 1e-4), while the law it approximates is
            % genuinely singular there (TOP/BOT -> 10/X).  Below X ~ 0.2016
            % the .0001 dominates outright and drives XNP -> 0.  Removing it
            % does not produce a divide-by-zero you would notice; it produces
            % NEGATIVE navigation ratios in the last fraction of a second.
            % See the header table and R.selfcheck.guard_*.
            XNP  = TOP/(.0001 + BOT1 + BOT2);
            % '+. . .' in the transcription is MATLAB's '...' continuation.
            ZEM1 = RTM1 + VTM1*TGO + AT1*(1. - cos(W*TGO))/W^2 + ...
                   (AT1D*(W*TGO - sin(W*TGO))/W^3) - ...
                   AM1*TAU*TAU*(exp(-X) + X - 1.);
            ZEM2 = RTM2 + VTM2*TGO + AT2*(1. - cos(W*TGO))/W^2 + ...
                   (AT2D*(W*TGO - sin(W*TGO))/W^3) - ...
                   AM2*TAU*TAU*(exp(-X) + X - 1.);
            ZEM3 = RTM3 + VTM3*TGO + AT3*(1. - cos(W*TGO))/W^2 + ...
                   (AT3D*(W*TGO - sin(W*TGO))/W^3) - ...
                   AM3*TAU*TAU*(exp(-X) + X - 1.);
        end
        % ---- the 3D step: project ZEM perpendicular to the LOS -----------
        % THIS is what makes the law three-dimensional rather than three
        % planar laws.  Drop it and each channel would chase its own ZEM
        % including the component ALONG the line of sight, which no
        % acceleration can remove and which therefore just burns g.
        ZEMDOTRTM = (ZEM1*RTM1 + ZEM2*RTM2 + ZEM3*RTM3)/RTM;
        ZEMPER1 = ZEM1 - ZEMDOTRTM*RTM1/RTM;
        ZEMPER2 = ZEM2 - ZEMDOTRTM*RTM2/RTM;
        ZEMPER3 = ZEM3 - ZEMDOTRTM*RTM3/RTM;
        AM1P = XNP*ZEMPER1/(TGO*TGO);
        AM2P = XNP*ZEMPER2/(TGO*TGO);
        AM3P = XNP*ZEMPER3/(TGO*TGO);
        AM1D = (AM1P - AM1)/TAU;
        AM2D = (AM2P - AM2)/TAU;
        AM3D = (AM3P - AM3)/TAU;
        % ---- the three OCR-damaged comparisons, resolved to '>' ----------
        % See the header for the three independent reasons.  Instrumented so
        % the resolution is a MEASUREMENT on the board, not a claim in a
        % comment.
        preIn = (abs(AM1) < XNCLIM) && (abs(AM2) < XNCLIM) && (abs(AM3) < XNCLIM);
        hit = false;
        if AM1 > XNCLIM,  AM1 =  XNCLIM; hit = true; end
        if AM1 < -XNCLIM, AM1 = -XNCLIM; hit = true; end
        if AM2 > XNCLIM,  AM2 =  XNCLIM; hit = true; end
        if AM2 < -XNCLIM, AM2 = -XNCLIM; hit = true; end
        if AM3 > XNCLIM,  AM3 =  XNCLIM; hit = true; end
        if AM3 < -XNCLIM, AM3 = -XNCLIM; hit = true; end
        XNCG = sqrt(AM1^2 + AM2^2 + AM3^2)/32.2;
        FLAG = 1;

        % Scalar comparisons, not max([...]).  This runs ~550,000 times over
        % the two sweeps and building a 4-element array each time is most of
        % the instrumentation's cost.
        nEval = nEval + 1;
        if preIn, clampInside = clampInside + 1; end
        if hit,   clampHits   = clampHits + 1;   end
        a1 = abs(AM1);  a2 = abs(AM2);  a3 = abs(AM3);
        if a1 > peakAM, peakAM = a1; end
        if a2 > peakAM, peakAM = a2; end
        if a3 > peakAM, peakAM = a3; end
        if XNCG > peakG,  peakG  = XNCG; end
        if XNP  < xnpMin, xnpMin = XNP;  end
        if XNP  > xnpMax, xnpMax = XNP;  end
    end
    FLAG = 0;
    RT1 = .5*(RT1OLD + RT1 + H*VT1);  RT2 = .5*(RT2OLD + RT2 + H*VT2);
    RT3 = .5*(RT3OLD + RT3 + H*VT3);
    RM1 = .5*(RM1OLD + RM1 + H*VM1);  RM2 = .5*(RM2OLD + RM2 + H*VM2);
    RM3 = .5*(RM3OLD + RM3 + H*VM3);
    VM1 = .5*(VM1OLD + VM1 + H*AM1);  VM2 = .5*(VM2OLD + VM2 + H*AM2);
    VM3 = .5*(VM3OLD + VM3 + H*AM3);
    VT1 = .5*(VT1OLD + VT1 + H*AT1);  VT2 = .5*(VT2OLD + VT2 + H*AT2);
    VT3 = .5*(VT3OLD + VT3 + H*AT3);
    AM1 = .5*(AM1OLD + AM1 + H*AM1D); AM2 = .5*(AM2OLD + AM2 + H*AM2D);
    AM3 = .5*(AM3OLD + AM3 + H*AM3D);
    % How far outside XNCLIM the CORRECTOR left the state.  It can be
    % positive: the corrector averages the clamped predictor value with the
    % unclamped old one.  See the note at the top of FLYONE.
    o1 = abs(AM1) - XNCLIM;  o2 = abs(AM2) - XNCLIM;  o3 = abs(AM3) - XNCLIM;
    if o1 > clampOver, clampOver = o1; end
    if o2 > clampOver, clampOver = o2; end
    if o3 > clampOver, clampOver = o3; end
    S = S + H;
    % '.09999' is NOT a typo for '.1'.  Neither 0.01 nor 0.0002 is exactly
    % representable in binary, so ten 0.01 steps accumulate to
    % 0.09999999999999999 and 'S >= .1' would miss samples at irregular
    % intervals.  The book's slack is the fix.
    if S >= .09999
        S = 0.;
        RT1K = RT1/1000.;  RT2K = RT2/1000.;  RT3K = RT3/1000.;
        RM1K = RM1/1000.;  RM2K = RM2/1000.;  RM3K = RM3/1000.;
        if wantTrace
            nTrace = nTrace + 1;
            if nTrace > cap
                cap = 2*cap;
                trT(cap) = NaN; trRTM(cap) = NaN; trG(cap) = NaN;
                trXNP(cap) = NaN; trTGO(cap) = NaN;
                trR(3, cap) = NaN; trM(3, cap) = NaN;
            end
            % The trace holds the CORRECTED states but the derivative-pass
            % RTM/XNCG/XNP/TGO, exactly as the book's own display block does.
            trT(nTrace)   = T;    trRTM(nTrace) = RTM;
            trG(nTrace)   = XNCG; trXNP(nTrace) = XNP;
            trTGO(nTrace) = TGO;
            trR(:, nTrace) = [RT1K; RT2K; RT3K];
            trM(:, nTrace) = [RM1K; RM2K; RM3K];
        end
    end
    nStep = nStep + 1;
    if nStep > MAXSTEP
        error('mgs:zarchan:engage3d:runaway', ...
            ['engagement from RT3IC = %g ft did not close after %d steps ' ...
             '(T = %g s, RTM = %g ft, VC = %g ft/s).  VC has stayed >= 0 ' ...
             'without intercept, which means the geometry is diverging; ' ...
             'returning the current separation as a "miss" would be a ' ...
             'plausible wrong answer.'], RT3IC, nStep, T, RTM, VC);
    end
end

% The loop exits on the FIRST step past closest approach, so the reported miss
% carries a floor of one step of closing: HFINE*VC = 0.0002*4000 = 0.8 ft.
% A miss below that floor is indistinguishable from zero and MUST NOT be
% quoted as a trend.  E.quantum_ft reports it for the run actually done.
E.rt3ic  = RT3IC;
E.T      = T;
E.RTM1   = RTM1;  E.RTM2 = RTM2;  E.RTM3 = RTM3;  E.RTM = RTM;
E.peak_g = peakG;
E.peak_am = peakAM;
E.steps  = nStep;
E.evals  = nEval;
E.vm_end = sqrt(VM1^2 + VM2^2 + VM3^2);
E.vm_0   = VM0;
E.clamp_hits   = clampHits;
E.clamp_inside = clampInside;
E.clamp_overshoot = max(clampOver, 0);
E.xnp_min = xnpMin;
E.xnp_max = xnpMax;
E.quantum_ft = C.hfine*max(VC, C.vm + C.vt);
if wantTrace
    E.trace = struct('t', trT(1:nTrace), 'rtm', trRTM(1:nTrace), ...
        'g', trG(1:nTrace), 'xnp', trXNP(1:nTrace), 'tgo', trTGO(1:nTrace), ...
        'rt_kft', trR(:, 1:nTrace), 'rm_kft', trM(:, 1:nTrace));
else
    E.trace = [];
end
end


function S = sweep(C)
%SWEEP  The book's outer loop: one engagement per initial separation, then the
%   miss components tabulated against FLIGHT TIME.
%
%   The book's grid counts DOWN in range (40000:-500:500) and therefore down
%   in flight time.  Everything downstream of here -- the sinusoid fit, the
%   steady-state window, the plots -- assumes ASCENDING flight time, so the
%   sort is done once, here, and the sorted arrays are the only ones exported.
%   Sorting late instead would let a reversed frequency estimate look fine.
n = numel(C.rt3ic);
rt3 = C.rt3ic(:).';
T   = nan(1, n);  M1 = nan(1, n);  M2 = nan(1, n);  M3 = nan(1, n);
MM  = nan(1, n);  PG = nan(1, n);  PA = nan(1, n);
ST  = nan(1, n);  VE = nan(1, n);
CH  = zeros(1, n); CI = zeros(1, n); CO = zeros(1, n);
NEV = zeros(1, n);
XLO =  inf(1, n);  XHI = -inf(1, n);
trace = [];

% The longest engagement carries the time-history trace: it is the one whose
% transient is fully resolved and the only one worth plotting.
[~, iLong] = max(rt3);
for k = 1:n
    E = flyOne(rt3(k), C, C.trace && k == iLong);
    T(k) = E.T;  M1(k) = E.RTM1;  M2(k) = E.RTM2;  M3(k) = E.RTM3;
    MM(k) = E.RTM;  PG(k) = E.peak_g;  PA(k) = E.peak_am;
    ST(k) = E.steps;  VE(k) = E.vm_end;
    CH(k) = E.clamp_hits;  CI(k) = E.clamp_inside;  CO(k) = E.clamp_overshoot;
    NEV(k) = E.evals;
    XLO(k) = E.xnp_min;  XHI(k) = E.xnp_max;
    if k == iLong && ~isempty(E.trace), trace = E.trace; end
end

[tf, ord] = sort(T);
S.qpn     = C.qpn;
S.rt3ic   = rt3(ord);
S.tf      = tf;
S.miss1   = M1(ord);
S.miss2   = M2(ord);
S.miss3   = M3(ord);
S.miss    = MM(ord);
S.peak_g  = PG(ord);
S.peak_am = PA(ord);
S.steps   = ST(ord);
S.vm_end  = VE(ord);
S.evals   = NEV(ord);
S.xnp_min = min(XLO);
S.xnp_max = max(XHI);
S.clamp_hits    = sum(CH);
S.clamp_inside  = sum(CI);
S.clamp_evals   = sum(NEV);
S.clamp_overshoot = max(CO);
S.trace   = trace;
S.quantum_ft = C.hfine*(C.vm + C.vt);
S.vc0     = C.vm + C.vt;
end


% =============================================================================
%   THE SHAPE ANALYSIS.  This is where the chapter's claims become checkable.
% =============================================================================
function F = fitSweep(S, C)
%FITSWEEP  Fit miss_i(tF) = A sin(w tF + phi) + c on the steady-state window.
%
%   WHY A FIT AND NOT A max().  The book's claims about Figs. 28.1/28.2 are
%   about an AMPLITUDE and a FREQUENCY, and both are read off a curve sampled
%   at 0.125 s.  max(|miss1|) over the grid is a biased estimate of the
%   amplitude (it can only ever land at or below a true peak that falls
%   between samples) and says nothing at all about frequency.  A three-
%   parameter least-squares fit with the frequency scanned gives both, plus an
%   r2 that says whether the sinusoid claim is even true -- which is the part
%   a max() can never fail.
%
%   The frequency scan is a plain grid search on w with a linear solve for
%   (a, b, c) at each w, because the problem is linear in everything except w.
%   No toolbox, no fminsearch, no local minimum to fall into.
w = (S.tf >= C.tfss);
F.window   = [C.tfss, max(S.tf)];
F.n_points = sum(w);
F.periods  = C.w*(max(S.tf) - C.tfss)/(2*pi);
F.tf       = S.tf(w);

F.c1 = fitSinusoid(S.tf(w), S.miss1(w));
F.c2 = fitSinusoid(S.tf(w), S.miss2(w));
F.c3 = fitSinusoid(S.tf(w), S.miss3(w));

% Quadrature.  The target drives axis 1 with sin(wt) and axis 2 with cos(wt)
% (:10285-10289), so the two miss channels must come out 90 deg apart.  That
% -- not the amplitudes -- is the mechanism behind the constant total miss,
% and it is the one thing a per-axis implementation could get wrong while
% still producing two plausible oscillations.
d = F.c2.phase_deg - F.c1.phase_deg;
F.dphase_deg = mod(d + 180, 360) - 180;
F.quadrature_err_deg = abs(abs(F.dphase_deg) - 90);

% Total miss statistics on the same window.
m = S.miss(w);
F.total_mean   = mean(m);
F.total_std    = std(m);
F.total_min    = min(m);
F.total_max    = max(m);
F.total_spread = (max(m) - min(m));
if F.total_mean > 0
    F.total_rel_spread = F.total_spread/F.total_mean;
else
    F.total_rel_spread = NaN;
end
% Peak-to-peak of a component, for the "oscillates in the plane, constant in
% 3D" claim (:10583).  Comparing spreads is the whole content of that claim.
F.comp_pp = max(max(S.miss1(w)) - min(S.miss1(w)), ...
                max(S.miss2(w)) - min(S.miss2(w)));
if F.comp_pp > 0
    F.flatness_ratio = F.total_spread/F.comp_pp;
else
    F.flatness_ratio = NaN;
end
F.cross_max = max(abs(S.miss3));       % over the WHOLE sweep, not the window
F.total_max_all = max(S.miss);
F.total_min_all = min(S.miss);
F.comp_max_all  = max(max(abs(S.miss1)), max(abs(S.miss2)));

% Sensitivity of the window choice.  The book says "an initial transient
% period" and never quantifies it, so a result that only survives at one
% window is a result about the window.
w5 = (S.tf >= C.tfss + 1);
if sum(w5) >= 8
    F.alt = fitSinusoid(S.tf(w5), S.miss1(w5));
    F.alt.window_start = C.tfss + 1;
    F.alt_total_mean = mean(S.miss(w5));
else
    F.alt = [];
    F.alt_total_mean = NaN;
end
end


function Q = fitSinusoid(t, y)
%FITSINUSOID  Least squares y ~ a sin(w t) + b cos(w t) + c, w scanned.
t = t(:);  y = y(:);
n = numel(t);
if n < 8
    error('mgs:zarchan:engage3d:fit', ...
        ['a 3-parameter sinusoid fit to %d points is not a measurement. ' ...
         'This is reached only if the steady-state window guard in ' ...
         'resolveConfig was bypassed.'], n);
end
span = max(t) - min(t);
% Search band: from one full period inside the window up to well past the
% target weave frequency.  Deliberately WIDE, so that a wrong answer shows up
% as a wrong frequency rather than being clamped to the expected one.
wLo = 2*pi/span;
wHi = pi/median(diff(sort(t)));         % the sampling Nyquist
wLo = max(wLo, 0.2);
if wHi <= wLo
    error('mgs:zarchan:engage3d:fitBand', ...
        ['the flight-time grid supports no frequency band: span %.4g s, ' ...
         'median spacing %.4g s.'], span, median(diff(sort(t))));
end
best = struct('sse', Inf, 'w', NaN, 'p', [0;0;0]);
on = ones(n, 1);                        % hoisted: the scan runs ~4800 times
best = scanBand(t, y, linspace(wLo, wHi, 4000), best, on);
% Refine around the winner.  The coarse grid step is (wHi-wLo)/4000; two
% refinement passes take the resolution to ~1e-4 rad/s, well below the 2%
% band the frequency rows are scored at.
for pass = 1:2
    dw = (wHi - wLo)/4000/(10^(pass-1));
    best = scanBand(t, y, best.w + linspace(-2*dw, 2*dw, 401), best, on);
end
a = best.p(1);  b = best.p(2);  c = best.p(3);
Q.omega     = best.w;
Q.amp       = hypot(a, b);
Q.phase_deg = atan2(b, a)*57.3;         % the book's 57.3, not 180/pi
Q.offset    = c;
Q.rms_resid = sqrt(best.sse/n);
sst = sum((y - mean(y)).^2);
if sst > 0
    Q.r2 = 1 - best.sse/sst;
else
    Q.r2 = NaN;
end
Q.n = n;
end


function best = scanBand(t, y, ws, best, on)
for w = ws(:).'
    if ~isfinite(w) || w <= 0, continue; end
    A = [sin(w*t), cos(w*t), on];
    p = A\y;
    r = y - A*p;
    sse = r.'*r;
    if sse < best.sse
        best.sse = sse;  best.w = w;  best.p = p;
    end
end
end


% =============================================================================
%   THE CH.20 CLOSED FORM -- the only greppable magnitude in reach
% =============================================================================
function Z = ch20Reference(C, GRAV_FT)
%CH20REFERENCE  The steady-state peak weave miss, and the book's own numerical
%   examples that validate this transcription of it.
%
%   zarchan.txt:6093-6096, p.449:
%       Peak Miss / (nT T^2) |N'=3 = 32.2 x / (1 + x^2)^1.5,   x = wT*T
%   zarchan.txt:6104-6111, p.450:
%       |N'=4 = 32.2 x^2 / (1 + x^2)^2
%       |N'=5 = 32.2 x^3 / (1 + x^2)^2.5
%
%   The 32.2 is ONLY the g -> ft/s^2 conversion on nT.  This file therefore
%   computes the miss as aT*w/(w^2 + 1/T^2)^1.5 for N'=3 (the equivalent form
%   printed at :6056-6057) so the constant is not applied twice, and reports
%   the normalized number separately for comparison with Fig. 20.8's ordinate.
Z.norm3 = @(x) 32.2*x./(1 + x.^2).^1.5;
Z.norm4 = @(x) 32.2*x.^2./(1 + x.^2).^2;
Z.norm5 = @(x) 32.2*x.^3./(1 + x.^2).^2.5;

Z.x     = C.w*C.tau;
Z.nt_g  = C.at/GRAV_FT;
switch round(C.xnp)
case 3, Z.norm = Z.norm3(Z.x); Z.have = true; Z.law = 'N''=3';
case 4, Z.norm = Z.norm4(Z.x); Z.have = true; Z.law = 'N''=4';
case 5, Z.norm = Z.norm5(Z.x); Z.have = true; Z.law = 'N''=5';
otherwise
    Z.norm = NaN;  Z.have = false;
    Z.law  = sprintf('N''=%g', C.xnp);
end
Z.have = Z.have && abs(C.xnp - round(C.xnp)) < 1e-12;
Z.peak_miss_ft = Z.norm*Z.nt_g*C.tau^2;
% The same number without the round trip through 32.2, for N'=3 only.
Z.peak_miss_direct_ft = C.at*C.w/(C.w^2 + 1/C.tau^2)^1.5;
Z.direct_vs_norm = Z.peak_miss_direct_ft - Z.norm3(Z.x)*Z.nt_g*C.tau^2;

% The book's own worked examples, :6122-6131 (p.451).  These validate the
% FORMULA, not this port.  If they go red the reference constant is wrong and
% every Ch.28 row scored against it is meaningless -- which is why they are on
% the board rather than in a comment.
Z.ex = struct( ...
  'label', {'x=2 (wT=2, T=1)', 'x=1 (wT=2, T=0.5)', 'x=2 (wT=4, T=0.5)'}, ...
  'x',     {2, 1, 2}, ...
  'norm_book', {5.5, 11.5, 5.5}, ...
  'norm_line', {6122, 6126, 6130}, ...
  'miss_book', {33, 17, 8}, ...
  'miss_line', {6123, 6127, 6131}, ...
  'nt',    {6, 6, 6}, ...
  'tau',   {1, 0.5, 0.5});
for i = 1:numel(Z.ex)
    Z.ex(i).norm_ours = Z.norm3(Z.ex(i).x);
    Z.ex(i).miss_ours = Z.ex(i).norm_ours*Z.ex(i).nt*Z.ex(i).tau^2;
end
end


function L = weaveLawTable(C)
%WEAVELAWTABLE  XNP(x) for the compensated weave law, guarded and unguarded.
%   Exists so the .0001 finding in the header is a TABLE the reader can check,
%   not an assertion.  x = tgo/TAU, so small x is the last moments of flight.
L.x = [1e-4 1e-3 1e-2 0.05 0.1 0.2 0.5 1 2 3 5 10 100 1000 10000];
n = numel(L.x);
L.top = nan(1, n);  L.bot_raw = nan(1, n);
L.xnp = nan(1, n);  L.xnp_raw = nan(1, n);
for i = 1:n
    X = L.x(i);
    TOP  = 6.*X*X*(exp(-X) - 1. + X);
    BOT1 = 2*X*X*X + 3. + 6.*X - 6.*X*X;
    BOT2 = -12.*X*exp(-X) - 3.*exp(-2.*X);
    L.top(i)     = TOP;
    L.bot_raw(i) = BOT1 + BOT2;
    L.xnp(i)     = TOP/(.0001 + BOT1 + BOT2);   % the book's line, :10306
    L.xnp_raw(i) = TOP/(BOT1 + BOT2);           % what "cleaning up" gives you
end
% The analytic series, for comparison with the computed BOT: every coefficient
% through x^4 cancels identically and the leading survivor is 0.3 x^5.
L.bot_series = 0.3*L.x.^5;
L.tau = C.tau;
L.t_remaining_s = L.x*C.tau;   % x is tgo/TAU, so this is literally time to go
end


% =============================================================================
%   STRUCTURAL SELF-CHECKS.  Wiring, not the book.  A failure here voids
%   everything above it, so these are separated from the benchmark board.
% =============================================================================
function K = structuralCheck(R, C, GRAV_FT)
K = struct();
K.skipped = false;
names = {};  oks = [];

% ---- S1. closing velocity is an identity --------------------------------
% VC(0) = VT + VM exactly, because the initial geometry is purely along axis 3.
% If this drifts, the initial conditions were mis-transcribed.
K.vc0_ours = R.pn.vc0;
K.vc0_book = C.vt + C.vm;
K.vc0_err  = abs(K.vc0_ours - K.vc0_book);
K.vc0_tol  = 1e-9;
K.vc0_ok   = K.vc0_err <= K.vc0_tol;
names{end+1} = 'initial VC = VT + VM';  oks(end+1) = K.vc0_ok; %#ok<AGROW>

% ---- S2. the PN branch never touches the weave law ----------------------
% XNP must be EXACTLY cfg.xnp for every derivative evaluation of every PN
% engagement.  This is a delete-experiment on branch selection: if 'QPN==1'
% were mis-transcribed the weave branch would run, XNP would vary from ~3e4
% down to ~0, and the miss would still look like a number.
K.xnp_pn_min = R.pn.xnp_min;
K.xnp_pn_max = R.pn.xnp_max;
K.xnp_pn_err = max(abs(R.pn.xnp_min - C.xnp), abs(R.pn.xnp_max - C.xnp));
K.xnp_pn_tol = 0;                       % exact: XNP is never assigned in that branch
K.xnp_pn_ok  = K.xnp_pn_err <= K.xnp_pn_tol;
names{end+1} = 'PN run leaves XNP untouched';  oks(end+1) = K.xnp_pn_ok; %#ok<AGROW>

% ---- S3. the weave law's own limit -------------------------------------
% N' -> 3 as x -> infinity: the compensated law degenerates to plain PN when
% there is far more time to go than lag.  A transcription error anywhere in
% TOP/BOT1/BOT2 breaks this.
% The approach is O(1/x) -- measured, the error is 2.003/x almost exactly -- so
% the sample points and the tolerance have to be chosen together.  x = 100 is
% only 2.03% converged and would fail a 2% band on a CORRECT transcription,
% which is why it is not in the list: a tolerance that a right answer cannot
% meet is worse than no check.
xi = [1e3 1e4];
K.xnp_inf = arrayfun(@(x) weaveXNP(x, true), xi);
K.xnp_inf_x = xi;
K.xnp_inf_err = max(abs(K.xnp_inf/3 - 1));
K.xnp_inf_tol = 0.005;                  % 3.00601 at x=1e3, 3.00060 at x=1e4
K.xnp_inf_ok  = K.xnp_inf_err <= K.xnp_inf_tol;
names{end+1} = 'weave N'' -> 3 as x -> inf';  oks(end+1) = K.xnp_inf_ok; %#ok<AGROW>

% ---- S4. the .0001 guard, both halves ----------------------------------
% (a) below x ~ 1e-3 the UNGUARDED denominator is roundoff: its magnitude is
%     far below the guard AND far from the analytic 0.3 x^5.
xg = 1e-3;
botRaw = weaveBot(xg);
K.guard_x        = xg;
K.guard_bot_raw  = botRaw;
K.guard_bot_true = 0.3*xg^5;
K.guard_roundoff_ok = abs(botRaw) < 1e-6;   % the guard is 1e-4; roundoff is ~1e-16
names{end+1} = 'unguarded denominator is roundoff at x=1e-3';
oks(end+1) = K.guard_roundoff_ok;
% (b) somewhere in the terminal band the guard CHANGES the answer by >= 2x.
%     If it did not, removing it would be harmless and the header's claim
%     would be wrong.
xb = logspace(-3, log10(0.25), 60);
gv = arrayfun(@(x) weaveXNP(x, true),  xb);
rv = arrayfun(@(x) weaveXNP(x, false), xb);
ratio = gv./rv;
ok = isfinite(ratio) & rv ~= 0;
if any(ok)
    K.guard_max_change = max(abs(1 - ratio(ok)));
else
    K.guard_max_change = NaN;
end
K.guard_change_tol = 0.5;
K.guard_change_ok  = K.guard_max_change >= K.guard_change_tol;
names{end+1} = 'guard changes XNP by >= 50% below x = 0.25';
oks(end+1) = K.guard_change_ok;
% Where the guard takes over, in seconds of time to go.
K.guard_takeover_x = (1e-4/0.3)^(1/5);
K.guard_takeover_s = K.guard_takeover_x*C.tau;

% ---- S5. DELETE EXPERIMENT: no maneuver, no miss ------------------------
% "The only source of error in the simulation is the spiraling target
% maneuver" (:10295).  Set AT = 0 and the miss must collapse to the terminal
% quantum.  Without this row every miss-magnitude row above would also pass if
% the miss were being produced by the geometry or by the integrator.
C0 = C;  C0.at = 0;  C0.qpn = 1;  C0.trace = false;
C0.rt3ic = C.rt3ic(round(linspace(1, numel(C.rt3ic), 5)));
S0 = sweep(C0);
K.nomaneuver_miss = max(S0.miss);
K.nomaneuver_tol  = 1.5*S0.quantum_ft;   % one closing step, with slack for the corrector
K.nomaneuver_ok   = K.nomaneuver_miss <= K.nomaneuver_tol;
names{end+1} = 'delete experiment: aT = 0 collapses the miss';
oks(end+1) = K.nomaneuver_ok;

% ---- S6. the clamp is INERT at the book's limit -------------------------
K.clamp_inert_hits   = R.pn.clamp_hits;
K.clamp_inert_frac   = R.pn.clamp_inside/max(R.pn.clamp_evals, 1);
K.clamp_inert_margin = R.pn.clamp_overshoot;
K.clamp_inert_ok = (K.clamp_inert_hits == 0) && (K.clamp_inert_frac == 1);
names{end+1} = 'clamp inert at XNCLIM = 9999999999';
oks(end+1) = K.clamp_inert_ok;
K.peak_am_over_limit = max(R.pn.peak_am)/C.xnclim;

% ---- S7. DELETE EXPERIMENT: the clamp is ALIVE, and '>' is the reading ---
% Rerun one engagement with a 2-g limit.  Under the resolved '>' reading the
% clamp fires on some evaluations and leaves others strictly inside.  Under
% the inverted '<' reading NOTHING is ever strictly inside -- every inside
% sample is exactly what the inverted test overwrites -- so inside_frac would
% be 0 and hits_frac 1.  The two fractions together are the discriminator.
C1 = C;  C1.xnclim = 2*GRAV_FT;  C1.qpn = 1;  C1.trace = false;
C1.rt3ic = max(C.rt3ic);
S1 = sweep(C1);
K.clamp_alive_hits_frac   = S1.clamp_hits/max(S1.clamp_evals, 1);
K.clamp_alive_inside_frac = S1.clamp_inside/max(S1.clamp_evals, 1);
K.clamp_alive_peak_am     = max(S1.peak_am);
K.clamp_alive_limit       = C1.xnclim;
K.clamp_alive_overshoot   = S1.clamp_overshoot;
K.clamp_alive_ok = (K.clamp_alive_inside_frac > 0.05) && ...
                   (K.clamp_alive_hits_frac   > 0.0);
names{end+1} = 'delete experiment: clamp fires AND releases at 2 g';
oks(end+1) = K.clamp_alive_ok;
% The overshoot the clamp-before-corrector ordering leaves behind, reported as
% a fraction of the limit.  Not scored -- it is a PROPERTY of the book's code,
% and pinning a tolerance on it would be inventing a book claim.
K.clamp_alive_overshoot_frac = K.clamp_alive_overshoot/C1.xnclim;

% ---- S8. the point mass is not speed-normalized ------------------------
% Reported and bounded, not asserted to be zero: the model genuinely does not
% conserve speed (see the header).  It stays small ONLY because this
% engagement is nearly head-on.
% Measured at 1.58% on the book nominal.  The tolerance is 5% rather than
% something snug because the drift is dominated by the LAST FEW STEPS, where
% the command spikes to ~350 g and the line of sight swings away from the
% velocity vector -- so it is a function of the terminal step size, and a 2%
% band would turn a step-size change into a "structural failure".
K.vm_drift = max(abs(R.pn.vm_end/C.vm - 1));
K.vm_drift_tol = 0.05;
K.vm_drift_ok = K.vm_drift <= K.vm_drift_tol;
names{end+1} = 'missile speed drift under un-normalized accel';
oks(end+1) = K.vm_drift_ok;

% ---- S9. the sinusoid claim is supported at all ------------------------
% Every amplitude and frequency row is meaningless if the fit is poor.  This
% row is what stops a bad fit from being reported as a confident number.
K.fit_r2 = min([R.fit_pn.c1.r2, R.fit_pn.c2.r2]);
K.fit_r2_tol = 0.90;
K.fit_r2_ok  = K.fit_r2 >= K.fit_r2_tol;
names{end+1} = 'miss components really are sinusoidal in tF';
oks(end+1) = K.fit_r2_ok;

% ---- S10. window sensitivity -------------------------------------------
% The steady-state window is OURS.  If the answer moves when it moves, the
% answer is about the window.
if ~isempty(R.fit_pn.alt)
    K.window_shift = abs(R.fit_pn.alt.amp/R.fit_pn.c1.amp - 1);
    K.window_ok = K.window_shift <= 0.10;
else
    K.window_shift = NaN;
    K.window_ok = false;
end
names{end+1} = 'amplitude stable across a 1 s window shift';
oks(end+1) = K.window_ok;

K.names = names;
K.oks   = logical(oks);
K.ok    = all(K.oks);
bad = find(~K.oks, 1);
if isempty(bad)
    K.first_fail = '';
else
    K.first_fail = names{bad};
end
end


function v = weaveXNP(X, guarded)
%WEAVEXNP  The listing's compensated-weave navigation ratio, :10306.
TOP  = 6.*X*X*(exp(-X) - 1. + X);
BOT1 = 2*X*X*X + 3. + 6.*X - 6.*X*X;
BOT2 = -12.*X*exp(-X) - 3.*exp(-2.*X);
if guarded
    v = TOP/(.0001 + BOT1 + BOT2);
else
    v = TOP/(BOT1 + BOT2);
end
end


function b = weaveBot(X)
BOT1 = 2*X*X*X + 3. + 6.*X - 6.*X*X;
BOT2 = -12.*X*exp(-X) - 3.*exp(-2.*X);
b = BOT1 + BOT2;
end


% =============================================================================
%   THE BOARD
% =============================================================================
function B = benchmarks(R, C, offbook, GRAV_FT)
%BENCHMARKS  Computed vs published, with a band, a reason for the band, and
%   the figure/page plus the zarchan.txt line each value came from.
B = emptyBench();

% Gates.  A row is SKIPPED, with the reason printed, when the caller moved a
% parameter the book value depends on.  It is never silently re-scored.
GATE_TGT  = {'w', 'at', 'tau'};
GATE_ALL  = [GATE_TGT, {'vt', 'vm', 'xnp', 'xnclim'}];
GATE_GRID = [GATE_ALL, {'rt3ic'}];
gT = gateWhy(offbook, GATE_TGT);
gA = gateWhy(offbook, GATE_ALL);
gG = gateWhy(offbook, GATE_GRID);
gQ = gateWhy(offbook, [GATE_GRID, {'hfine'}]);

SRC_CFG  = 'p.718-719 prose (zarchan.txt:10293)';
SRC_NP   = 'p.716 (zarchan.txt:10252)';
SRC_PN   = 'Figs. 28.1-28.4 discussion, p.723 (zarchan.txt:10325)';
SRC_WV   = 'p.724 (zarchan.txt:10331)';
SRC_F1   = 'Fig. 28.1 caption, p.724 (zarchan.txt:10332)';
SRC_F2   = 'Fig. 28.2 caption, p.724 (zarchan.txt:10333)';
SRC_F3   = 'Fig. 28.3 caption, p.725 (zarchan.txt:10345)';
SRC_F4   = 'Fig. 28.4 caption, p.725 (zarchan.txt:10346)';
SRC_F5   = 'Fig. 28.5 caption, p.726 (zarchan.txt:10365)';
SRC_SUM  = 'Ch.28 summary (zarchan.txt:10583)';
SRC_CF   = 'Ch.20 closed form, p.449 (zarchan.txt:6093-6096)';
SRC_AXIS = 'Listing 28.1 own plot limits, p.723 (zarchan.txt:10323)';

% -------------------------------------------------------------------------
% A. CONFIGURATION ECHO.  Cheap, and they lock the transcription: every one
%    of these is a literal on the listing's first line cross-checked against
%    a sentence of prose.  If the transcription of that line drifted, the
%    magnitude rows below would move too and there would be nothing to say
%    which had happened.
% -------------------------------------------------------------------------
B = addVal(B, 'target speed VT', 1000, C.vt, 'exact', SRC_CFG, 'ft/s', ...
    '"the target is traveling at 1000 ft/s" -- this is the CROSS-RANGE component only');
B = addVal(B, 'weave frequency W', 3, C.w, 'exact', SRC_CFG, 'rad/s', ...
    '"spiraling at 3 r/s"');
B = addVal(B, 'target maneuver AT', 6, C.at/GRAV_FT, 'exact', SRC_CFG, 'g', ...
    ['"an acceleration level of 6 g".  The listing writes the literal 193.2, ' ...
     'and 193.2/32.2 is 5.99999999999999928 in double -- which is why the ' ...
     '''exact'' band here is 1e-12 and not 0']);
B = addVal(B, 'missile speed VM', 3000, C.vm, 'exact', SRC_CFG, 'ft/s', ...
    '"the missile is traveling at 3000 ft/s"');
B = addVal(B, 'FCS time constant TAU', 1, C.tau, 'exact', SRC_CFG, 's', ...
    '"a single-lag time constant of 1 s ... in each of the three channels"');
B = addVal(B, 'navigation ratio N''', 3, C.xnp, 'exact', SRC_NP, '-', ...
    '"a constant set equal to three for proportional navigation"');
B = addBound(B, 'acceleration limit is "infinite"', 'peak |a_M| / XNCLIM < 1e-3', ...
    sprintf('%.3g', R.selfcheck.peak_am_over_limit), ...
    R.selfcheck.peak_am_over_limit < 1e-3, SRC_CFG, '-', ...
    ['"has an infinite acceleration limit".  Scored as a bound because ' ...
     '"infinite" is a modelling statement, not a measurement: what is ' ...
     'checkable is that nothing ever came near XNCLIM.  The threshold is ' ...
     '1e-3 and not something tighter because the demand SPIKES at intercept ' ...
     '(N''*ZEMPER/tgo^2 with tgo -> 0 reaches ~350 g here), so the peak is ' ...
     'set by the terminal step size; 1e-3 still leaves three orders of margin']);

% -------------------------------------------------------------------------
% B. CH.20 PROVENANCE.  These validate the 18.33 ft reference constant, NOT
%    this port.  They are on the board because if the closed form were
%    mis-transcribed, every Ch.28 magnitude row would be scored against a
%    wrong number and would go red for the wrong reason.
% -------------------------------------------------------------------------
Z = R.ch20;
for i = 1:numel(Z.ex)
    e = Z.ex(i);
    B = addVal(B, sprintf('Ch.20 ref: norm miss %s', e.label), ...
        e.norm_book, e.norm_ours, 'graph', ...
        sprintf('p.451 (zarchan.txt:%d)', e.norm_line), '-', ...
        'the author READ this off Fig. 20.8''s ordinate, hence the 15% band');
    B = addVal(B, sprintf('Ch.20 ref: peak miss %s', e.label), ...
        e.miss_book, e.miss_ours, 'graph', ...
        sprintf('p.451 (zarchan.txt:%d)', e.miss_line), 'ft', ...
        'the book''s own arithmetic on the line above; carried so the units close');
end
B = addBound(B, 'Ch.20 ref: 32.2 applied exactly once', ...
    'norm*nT*T^2 == aT*w/(w^2+1/T^2)^1.5', ...
    sprintf('%.3g ft apart', abs(Z.direct_vs_norm)), ...
    abs(Z.direct_vs_norm) <= 1e-9*max(1, abs(Z.peak_miss_direct_ft)), ...
    'zarchan.txt:6056 vs :6093', 'ft', ...
    ['an identity, not a book value: the 32.2 in the normalized form is ' ...
     'purely nT[g] -> ft/s^2, so applying it twice would inflate every ' ...
     'reference miss by 32.2x and still look like a plausible foot-scale number']);

% -------------------------------------------------------------------------
% C. THE CH.28 RESULT ITSELF
% -------------------------------------------------------------------------
% PINNED to the proportional-navigation sweep, never to R.run -- see the note
% where R.fit_pn is built.  Every row below is headed "(PN)" and must mean it.
F  = R.fit_pn;
ref = Z.peak_miss_direct_ft;      % 18.33 ft at the book nominal

% C1. the headline: total 3D miss vs the Ch.20 closed form.
if ~Z.have
    B = addSkip(B, 'total 3D miss (PN) vs Ch.20 closed form', ...
        '32.2 x/(1+x^2)^1.5 * nT * T^2', ...
        sprintf(['the closed form is published only for N'' = 3, 4 and 5 ' ...
                 '(zarchan.txt:6093-6111); this run has N'' = %g'], C.xnp), ...
        SRC_CF, 'ft');
elseif ~isempty(gT)
    B = addSkip(B, 'total 3D miss (PN) vs Ch.20 closed form', ...
        sprintf('%.4g', ref), gT, SRC_CF, 'ft');
else
    B = addVal(B, 'total 3D miss (PN) vs Ch.20 closed form', ref, F.total_mean, ...
        'xchap', [SRC_CF ' via ' SRC_PN], 'ft', ...
        ['"the amplitude of the miss agrees with the single plane results of ' ...
         'Fig. 20.7 ... identical inputs were used".  A NONLINEAR run against ' ...
         'a LINEAR closed form; the book quotes no digits, so 15% is its own ' ...
         'precision.  The floor is the 0.8 ft terminal quantum = 4.4% of 18.3']);
end

% C2. the only Ch.28 magnitude actually printed in the book's prose.
if ~isempty(gA)
    B = addSkip(B, 'total 3D miss (PN) is "nearly 20 ft"', '15 <= miss < 20 ft', ...
        gA, SRC_WV, 'ft');
else
    B = addBound(B, 'total 3D miss (PN) is "nearly 20 ft"', '15 <= miss < 20 ft', ...
        sprintf('%.3f ft', F.total_mean), ...
        F.total_mean >= 15 && F.total_mean < 20, SRC_WV, 'ft', ...
        ['"the miss is reduced from nearly 20 ft to virtually zero".  The ' ...
         '20 is a NUMBER and is scored; "nearly" is read as just under.  ' ...
         'This is INDEPENDENT of the Ch.20 row above -- it tests the ' ...
         'simulation against Ch.28''s own prose, not against Ch.20''s algebra']);
end

% C3/C4. the two driven channels' amplitudes are the circle's radius.
for ch = 1:2
    if ch == 1
        nm = 'downrange'; q = F.c1; src = SRC_F1;
    else
        nm = 'altitude';  q = F.c2; src = SRC_F2;
    end
    what = sprintf('%s miss amplitude vs closed form', nm);
    if ~Z.have || ~isempty(gT)
        B = addSkip(B, what, sprintf('%.4g', ref), firstNonEmpty(gT, ...
            'the closed form is published only for N'' = 3, 4, 5'), SRC_CF, 'ft');
    else
        B = addVal(B, what, ref, q.amp, 'xchap', [SRC_CF ' via ' SRC_PN], 'ft', ...
            ['least-squares amplitude of the miss-vs-flight-time sinusoid; ' ...
             'a max() over the 0.125 s grid would be biased LOW and could ' ...
             'not fail the sinusoid claim at all']);
    end
    what = sprintf('%s miss oscillation frequency', nm);
    if ~isempty(gT)
        B = addSkip(B, what, sprintf('%.4g', C.w), gT, src, 'rad/s');
    else
        B = addVal(B, what, C.w, q.omega, 'text', src, 'rad/s', ...
            ['"oscillate[s] at same frequency as 3-r/s weaving target ' ...
             'maneuver" -- a figure CAPTION, which is prose and survives ' ...
             'extraction even though the curve did not']);
    end
end

% C5. cross-range: the untouched channel.
%
%   "Virtually zero" is an adjective, so the scored criterion has to be built,
%   and it is built from the geometry rather than picked.  Two terms, both
%   derived before the file was run:
%     (i) at closest approach RTM.VTM = 0, and VTM3 = -(VT+VM), so
%           |miss3| = |m1*VTM1 + m2*VTM2| / (VT+VM)
%         With the two driven misses ~18 ft and the lateral closing rates at
%         most a few times the target's own circular speed AT/W = 64.4 ft/s,
%         that is order 1 ft -- NOT zero, and a threshold of "one quantum"
%         would fail a correct port.
%    (ii) the run stops one step PAST closest approach, adding up to another
%         HFINE*VC = 0.8 ft along axis 3.
%   Expected worst case is therefore ~2 ft against an 18 ft total, so the row
%   is scored as a RATIO at 15% (2.75 ft).  That is what "virtually zero"
%   means when the other two channels are carrying 18 ft each; it is not a
%   tolerance widened to fit an answer.
CROSS_FRAC = 0.15;
if ~isempty(gQ)
    B = addSkip(B, 'cross-range miss is "virtually zero"', ...
        sprintf('|miss3| <= %g%% of the total miss', 100*CROSS_FRAC), ...
        gQ, SRC_F3, 'ft');
else
    thr = CROSS_FRAC*F.total_mean;
    B = addBound(B, 'cross-range miss is "virtually zero"', ...
        sprintf('|miss3| <= %.2f ft (%g%% of total)', thr, 100*CROSS_FRAC), ...
        sprintf('%.4f ft (quantum %.2f)', F.cross_max, R.pn.quantum_ft), ...
        F.cross_max <= thr, SRC_F3, 'ft', ...
        ['"virtually zero because there is no target maneuver in that ' ...
         'channel".  Axis 3 is the CLOSING axis, so a small residual is ' ...
         'geometry (see the derivation above the row), not a guidance ' ...
         'failure -- scoring this against a single closing step would fail a ' ...
         'correct port']);
end

% C6. the chapter's actual discovery.
if ~isempty(gG)
    B = addSkip(B, 'total miss constant vs flight time', ...
        'relative spread <= 15%', gG, SRC_F4, '-');
else
    B = addShape(B, 'total miss constant vs flight time', ...
        'approaches a constant in steady state', ...
        sprintf('spread %.1f%% of mean over tF in [%g, %g] s', ...
            100*F.total_rel_spread, F.window(1), F.window(2)), ...
        F.total_rel_spread <= 0.15, SRC_F4, ...
        ['"Total three-dimensional miss caused by 3-r/s weaving target ' ...
         'approaches a constant in steady state".  The 15% cannot go lower: ' ...
         'the 0.8 ft terminal quantum alone is 4.4% of an 18 ft miss']);
end
if ~isempty(gG)
    B = addSkip(B, 'components oscillate, total does not', ...
        'spread(total) << peak-to-peak(component)', gG, SRC_SUM, '-');
else
    B = addShape(B, 'components oscillate, total does not', ...
        'does not oscillate in 3D as it does in 1D or 2D', ...
        sprintf('spread(total)/pp(component) = %.3f', F.flatness_ratio), ...
        F.flatness_ratio <= 0.20, SRC_SUM, ...
        ['the chapter summary in one number.  Scored as a RATIO because ' ...
         'that is what the sentence compares; an absolute spread would pass ' ...
         'trivially for any miss that happened to be small']);
end

% C7. the mechanism, not the result.
if ~isempty(gT)
    B = addSkip(B, 'downrange/altitude miss in quadrature', ...
        '|phase difference| = 90 deg', gT, SRC_PN, 'deg');
else
    B = addShape(B, 'downrange/altitude miss in quadrature', ...
        'sine in one direction, cosine in the other -> circular motion', ...
        sprintf('phase difference %+.1f deg (err %.1f)', ...
            F.dphase_deg, F.quadrature_err_deg), ...
        F.quadrature_err_deg <= 15, SRC_PN, ...
        ['THE MECHANISM.  The constant total miss is a consequence of this, ' ...
         'not an independent fact -- and this is the row a per-axis (rather ' ...
         'than perpendicular-to-LOS) implementation would break while still ' ...
         'producing two plausible oscillations.  Sign is not scored: which ' ...
         'channel leads depends on the RTM = target - missile convention']);
end
if ~isempty(gT)
    B = addSkip(B, 'the two component amplitudes are equal', ...
        'they are one circle radius', gT, SRC_PN, 'ft');
else
    ampErr = abs(F.c2.amp/F.c1.amp - 1);
    B = addShape(B, 'the two component amplitudes are equal', ...
        'peak single-plane miss = the radius of a circle', ...
        sprintf('%.3f vs %.3f ft (%.1f%% apart)', F.c1.amp, F.c2.amp, 100*ampErr), ...
        ampErr <= 0.10, SRC_PN, ...
        '"the total miss is also along a circle in the downrange and altitude direction"');
end

% C8. the book's own plot limits, used as a bound.  Not a claim about the
%     data -- a claim about what Zarchan expected the data to fit inside.
if ~isempty(gG)
    B = addSkip(B, 'miss inside the listing''s own axis limits', ...
        'axis([0 10 -30 30]) / ([0 10 0 30])', gG, SRC_AXIS, 'ft');
else
    inAxis = F.comp_max_all <= 30 && F.total_max_all <= 30 && F.total_min_all >= 0;
    B = addBound(B, 'miss inside the listing''s own axis limits', ...
        'components within +/-30 ft, total within [0, 30] ft', ...
        sprintf('comp %.2f, total [%.2f, %.2f]', ...
            F.comp_max_all, F.total_min_all, F.total_max_all), ...
        inAxis, SRC_AXIS, 'ft', ...
        ['the listing hard-codes these limits, so they are the author''s own ' ...
         'statement of the range he expected -- weak, free, and the only ' ...
         'quantitative thing the figures left behind']);
end

% -------------------------------------------------------------------------
% D. THE COMPENSATED WEAVE LAW
% -------------------------------------------------------------------------
if isempty(R.weave)
    B = addSkip(B, 'compensated weave reduces the miss', 'ratio <= 0.15', ...
        'the weave sweep was not run (benchmark = false)', SRC_WV, '-');
else
    wv = R.weave;
    wsel = wv.tf >= C.tfss;
    wMean = mean(wv.miss(wsel));
    pMean = F.total_mean;
    if pMean > 0, ratio = wMean/pMean; else, ratio = NaN; end
    if ~isempty(gA)
        B = addSkip(B, 'compensated weave reduces the miss', 'ratio <= 0.15', ...
            gA, SRC_WV, '-');
    else
        B = addBound(B, 'compensated weave reduces the miss', ...
            'weave miss / PN miss <= 0.15', ...
            sprintf('%.4f ft / %.4f ft = %.4f', wMean, pMean, ratio), ...
            isfinite(ratio) && ratio <= 0.15, [SRC_WV ' + ' SRC_F5], '-', ...
            ['"reduced from nearly 20 ft to virtually zero" and "dramatically ' ...
             'reduces the total miss".  The ORDERING is scored; the absolute ' ...
             '"virtually zero" is not, and is the INFO row below.  0.15 is ' ...
             'the reduction the word "dramatically" has to mean at minimum, ' ...
             'not a fit to what this port produced']);
    end
    B = addInfo(B, 'compensated weave absolute miss', 'virtually zero', ...
        sprintf('%.4f ft vs terminal quantum %.3f ft (x%.2f)', ...
            wMean, wv.quantum_ft, wMean/max(wv.quantum_ft, eps)), ...
        SRC_WV, ...
        ['NOT SCORED: "virtually zero" is an adjective, and turning it into ' ...
         'a numeric tolerance would be inventing a book value.  Note WHY it ' ...
         'cannot be scored even in principle here: the measured weave miss ' ...
         'sits AT the terminal step quantum HFINE*VC, so any tolerance ' ...
         'tighter than that scores the step size rather than the guidance ' ...
         'law.  The law has driven the miss below what this integrator can ' ...
         'resolve, which is the strongest statement available and is not a ' ...
         'number']);
    B = addInfo(B, 'compensated weave peak acceleration', 'not published', ...
        sprintf('%.1f g vs %.1f g for PN', max(wv.peak_g), max(R.pn.peak_g)), ...
        SRC_WV, ...
        ['NOT SCORED: no published value.  Carried because it is the part of ' ...
         'the weave law''s advantage the book does not mention -- it buys the ' ...
         'miss reduction while ALSO demanding far less peak g, because the ' ...
         '.0001 guard collapses N'' at intercept instead of letting the ' ...
         'ZEMPER/tgo^2 term run away']);
    B = addInfo(B, 'compensated weave XNP range over the sweep', ...
        'N'' is not a constant for this law', ...
        sprintf('%.4g .. %.4g', wv.xnp_min, wv.xnp_max), ...
        'p.717-718 (zarchan.txt:10262)', ...
        ['reported because it is the clearest way to see the .0001 guard at ' ...
         'work: N'' collapses toward 0 in the last 0.2*TAU seconds rather ' ...
         'than diverging']);
end

% -------------------------------------------------------------------------
% E. REPORTED, NEVER SCORED
% -------------------------------------------------------------------------
B = addInfo(B, 'peak commanded acceleration (PN)', 'no published value', ...
    sprintf('%.2f g (max over the sweep)', max(R.pn.peak_g)), SRC_CFG, ...
    ['NOT SCORED: the book sets an infinite limit precisely so that this ' ...
     'number never constrains anything, and never prints it.  Carried ' ...
     'because it is what a real airframe would have to deliver']);
B = addInfo(B, 'transient peak of the total miss', ...
    'the book claims agreement only AFTER the transient', ...
    sprintf('%.2f ft at tF = %.3f s (window starts at %g s)', ...
        max(R.pn.miss), R.pn.tf(find(R.pn.miss == max(R.pn.miss), 1)), C.tfss), ...
    SRC_PN, ...
    ['NOT SCORED: "after an initial transient period" is never quantified, ' ...
     'so any number taken from inside the transient would be scoring OUR ' ...
     'window choice.  Printed so the choice is visible']);
B = addInfo(B, 'the .0001 guard turns the weave law off near intercept', ...
    'not discussed in the text', ...
    sprintf('guard dominates below x = %.3f, i.e. the last %.3f s', ...
        R.selfcheck.guard_takeover_x, R.selfcheck.guard_takeover_s), ...
    'Listing 28.1 line, p.721 (zarchan.txt:10306)', ...
    ['NOT SCORED: this is a finding ABOUT the listing, not a claim IN the ' ...
     'book.  R.weave_law tabulates it and R.selfcheck.guard_* scores the ' ...
     'two halves that are checkable']);
B = addInfo(B, 'clamp-before-corrector overshoot', ...
    'a property of the listing, not a book claim', ...
    sprintf('reaches %.2fx the limit at XNCLIM = 2 g', ...
        1 + R.selfcheck.clamp_alive_overshoot_frac), ...
    'Listing 28.1 lines, p.722 (zarchan.txt:10310-10318)', ...
    ['NOT SCORED (no book claim to score against), but READ IT.  The clamp ' ...
     'runs inside the derivative loop and the corrector afterwards averages ' ...
     'the CLAMPED predictor value with the UNCLAMPED old one, so the state ' ...
     'ends the step well outside XNCLIM -- measured at ~2.5x a 2 g limit, ' ...
     'i.e. the "2 g limit" actually flies about 5 g.  Anyone who lifts this ' ...
     'clamp into their own integrator inherits that.  Same ordering trap as ' ...
     'our own nz_clamp_after_floor; measured here rather than quietly fixed, ' ...
     'because fixing it changes the answer and the book''s answer is the ' ...
     'thing being reproduced']);
end


% =============================================================================
%   BENCH ROW HELPERS.  Schema matches what benchSuite consumes.
% =============================================================================
function B = emptyBench()
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end


function B = addVal(B, what, book, ours, band, source, units, detail)
i = numel(B) + 1;
tol = bandTol(band);
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.4g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.5g', ours);
B(i).band = band;  B(i).tol = tol;
B(i).units = units;  B(i).source = source;  B(i).detail = detail;
if ~isfinite(ours) || book == 0
    B(i).pct = NaN;
    B(i).ok  = false;
    B(i).skipped = true;
    B(i).why = 'the computed value is not finite (or the book value is zero)';
    return;
end
B(i).pct = 100*(ours/book - 1);
B(i).ok  = abs(ours/book - 1) <= tol;
B(i).skipped = false;
B(i).why = '';
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
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'skip';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = '--';
B(i).pct  = NaN;   B(i).band = 'skip';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end


function t = bandTol(band)
%BANDTOL  Bands encode the PROVENANCE of the book value.  Inventing one
%   silently sets an arbitrary pass criterion, so an unknown band ERRORS.
switch lower(band)
case 'exact',  t = 1e-12;  % a parameter echoed from prose, or an identity
case 'text',   t = 0.02;   % printed in prose to 2-3 figures
case 'approx', t = 0.05;   % the book says "approximately"
case 'graph',  t = 0.15;   % the author read it off a figure's ordinate
case 'xchap',  t = 0.15;   % nonlinear Ch.28 run vs linear Ch.20 closed form;
                           % the book asserts agreement and quotes no digits
otherwise
    error('mgs:zarchan:engage3d:band', ...
        ['unknown tolerance band "%s".  Bands encode the PROVENANCE of the ' ...
         'book value; inventing one silently sets an arbitrary pass ' ...
         'criterion.'], band);
end
end


function w = gateWhy(offbook, names)
%GATEWHY  '' if the caller left all of NAMES at the book value; otherwise the
%   reason the row cannot be compared against the book.
hit = intersect(offbook, names);
if isempty(hit)
    w = '';
else
    w = sprintf('off-book: %s', strjoin(sort(hit), ', '));
end
end


function s = offbookStr(offbook)
if isempty(offbook)
    s = 'nothing';
else
    s = strjoin(sort(offbook), ', ');
end
end


function s = firstNonEmpty(a, b)
if ~isempty(a), s = a; else, s = b; end
end


% =============================================================================
%   RENDER
% =============================================================================
function s = render(R, C, offbook)
W = 100;
L = {};
L{end+1} = 'Zarchan Listing 28.1 -- THREE-DIMENSIONAL TACTICAL ENGAGEMENT, SPIRALING TARGET';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['target: %g ft/s cross-range, spiral %g rad/s at %.4g g   |   ' ...
                    'missile: %g ft/s, N'' = %g, TAU = %g s'], ...
    C.vt, C.w, C.at/C.grav, C.vm, C.xnp, C.tau);
L{end+1} = sprintf(['sweep: RT3IC %g -> %g ft (%d cases), H = %g / %g s inside %g ft, ' ...
                    'VC(0) = %g ft/s'], ...
    max(C.rt3ic), min(C.rt3ic), numel(C.rt3ic), C.hcoarse, C.hfine, C.rfine, ...
    C.vt + C.vm);
L{end+1} = sprintf(['terminal quantum HFINE*VC = %.3f ft -- NO miss below this is ' ...
                    'distinguishable from zero'], C.hfine*(C.vt + C.vm));
L{end+1} = 'integrator: Zarchan''s two-pass predictor/corrector (Heun), transcribed';
L{end+1} = sprintf('axes: 1 = downrange, 2 = altitude, 3 = cross-range (zarchan.txt:10289)');
if isempty(offbook)
    L{end+1} = 'configuration: BOOK NOMINAL (nothing moved)';
else
    L{end+1} = sprintf('configuration: OFF-BOOK -- caller moved %s', offbookStr(offbook));
end
L{end+1} = repmat('-', 1, W);

% ---- structural ----------------------------------------------------------
if isfield(R.selfcheck, 'ok')
    K = R.selfcheck;
    L{end+1} = 'STRUCTURAL SELF-CHECK  (wiring, not the book -- a failure here voids everything)';
    L{end+1} = sprintf('  %-52s %-26s %s', 'initial VC = VT + VM', ...
        sprintf('err %.2e (tol %.0e)', K.vc0_err, K.vc0_tol), tf2s(K.vc0_ok));
    L{end+1} = sprintf('  %-52s %-26s %s', 'PN run leaves XNP untouched', ...
        sprintf('XNP in [%.6g, %.6g]', K.xnp_pn_min, K.xnp_pn_max), tf2s(K.xnp_pn_ok));
    L{end+1} = sprintf('  %-52s %-26s %s', 'weave N'' -> 3 as x -> inf', ...
        sprintf('%.4f at x = 1e4', K.xnp_inf(end)), tf2s(K.xnp_inf_ok));
    L{end+1} = sprintf('  %-52s %-26s %s', 'unguarded denominator at x = 1e-3 is roundoff', ...
        sprintf('%.2e vs true %.2e', K.guard_bot_raw, K.guard_bot_true), ...
        tf2s(K.guard_roundoff_ok));
    L{end+1} = sprintf('  %-52s %-26s %s', 'the .0001 guard is load-bearing', ...
        sprintf('changes XNP by %.0f%%', 100*K.guard_max_change), tf2s(K.guard_change_ok));
    L{end+1} = sprintf('  %-52s %-26s %s', 'DELETE EXPT: aT = 0 collapses the miss', ...
        sprintf('%.4f ft (tol %.3f)', K.nomaneuver_miss, K.nomaneuver_tol), ...
        tf2s(K.nomaneuver_ok));
    L{end+1} = sprintf('  %-52s %-26s %s', 'clamp inert at the book''s XNCLIM', ...
        sprintf('%d hits, %.4f inside', K.clamp_inert_hits, K.clamp_inert_frac), ...
        tf2s(K.clamp_inert_ok));
    L{end+1} = sprintf('  %-52s %-26s %s', 'DELETE EXPT: clamp fires AND releases at 2 g', ...
        sprintf('%.3f fire / %.3f inside', K.clamp_alive_hits_frac, ...
                K.clamp_alive_inside_frac), tf2s(K.clamp_alive_ok));
    L{end+1} = '      ^ this is the row that DISCRIMINATES the three OCR-damaged comparisons.';
    L{end+1} = '        Under the inverted reading "inside" would be 0.000 by construction.';
    L{end+1} = sprintf('  %-52s %-26s %s', 'missile speed drift (accel not normalized)', ...
        sprintf('%.4f%%', 100*K.vm_drift), tf2s(K.vm_drift_ok));
    L{end+1} = sprintf('  %-52s %-26s %s', 'miss components really are sinusoidal in tF', ...
        sprintf('min r2 = %.4f', K.fit_r2), tf2s(K.fit_r2_ok));
    L{end+1} = sprintf('  %-52s %-26s %s', 'amplitude stable across a 1 s window shift', ...
        sprintf('%.2f%% change', 100*K.window_shift), tf2s(K.window_ok));
else
    L{end+1} = 'STRUCTURAL SELF-CHECK  ***SKIPPED*** (benchmark = false) -- nothing is verified';
end
L{end+1} = repmat('-', 1, W);

% ---- the shape analysis --------------------------------------------------
F = R.fit;
L{end+1} = sprintf('SHAPE ANALYSIS  law = %s   steady-state window tF in [%g, %g] s (%d cases, %.2f weave periods)', ...
    lawName(R.run.qpn), F.window(1), F.window(2), F.n_points, F.periods);
L{end+1} = sprintf('  %-12s %10s %10s %10s %8s', 'channel', 'amp [ft]', 'w [rad/s]', 'phase[deg]', 'r2');
L{end+1} = sprintf('  %-12s %10.4f %10.4f %10.2f %8.4f', 'downrange', ...
    F.c1.amp, F.c1.omega, F.c1.phase_deg, F.c1.r2);
L{end+1} = sprintf('  %-12s %10.4f %10.4f %10.2f %8.4f', 'altitude', ...
    F.c2.amp, F.c2.omega, F.c2.phase_deg, F.c2.r2);
L{end+1} = sprintf('  %-12s %10.4f %10.4f %10.2f %8.4f', 'cross-range', ...
    F.c3.amp, F.c3.omega, F.c3.phase_deg, F.c3.r2);
L{end+1} = sprintf('  quadrature: phase difference %+.2f deg (target drives sin in 1, cos in 2)', ...
    F.dphase_deg);
L{end+1} = sprintf('  total miss: mean %.4f ft, spread %.4f ft (%.1f%%), range [%.4f, %.4f]', ...
    F.total_mean, F.total_spread, 100*F.total_rel_spread, F.total_min, F.total_max);
L{end+1} = sprintf('  component peak-to-peak %.3f ft -> flatness ratio %.4f', ...
    F.comp_pp, F.flatness_ratio);
L{end+1} = sprintf('  cross-range |miss3| max over the WHOLE sweep: %.4f ft', F.cross_max);
Z = R.ch20;
L{end+1} = sprintf('  Ch.20 closed form: x = w*T = %.3g -> normalized %.4f -> peak miss %.4f ft', ...
    Z.x, Z.norm, Z.peak_miss_direct_ft);
L{end+1} = '    (zarchan.txt:6093-6096; the ONLY greppable magnitude for this case -- Figs.';
L{end+1} = '     28.1-28.5 survived extraction as image links with no digits on them)';
L{end+1} = repmat('-', 1, W);

% ---- the weave-law table -------------------------------------------------
Lw = R.weave_law;
L{end+1} = 'COMPENSATED-WEAVE N''(x), GUARDED vs UNGUARDED   (x = tgo/TAU; zarchan.txt:10306)';
L{end+1} = sprintf('  %10s %12s %14s %14s %14s', 'x', 'tgo [s]', 'BOT computed', 'N'' guarded', 'N'' raw');
for i = 1:numel(Lw.x)
    L{end+1} = sprintf('  %10.4g %12.4g %14.3e %14.6g %14.6g', ...
        Lw.x(i), Lw.t_remaining_s(i), Lw.bot_raw(i), Lw.xnp(i), Lw.xnp_raw(i)); %#ok<AGROW>
end
L{end+1} = '  BOT cancels to 5th order (-> 0.3x^5) out of terms of size 3, so below x ~ 1e-3 the';
L{end+1} = '  raw denominator is double-precision NOISE, sign included (exactly 0.0 at x = 1e-4).';
L{end+1} = '  The law itself is also singular there: TOP/BOT -> 10/x.  The .0001 floors the';
L{end+1} = '  denominator below x = 0.2016, so N'' -> 0 and the weave law TURNS ITSELF OFF for the';
L{end+1} = '  last 0.2*TAU seconds rather than diverging.  It is inert where the law is meaningful.';
L{end+1} = repmat('-', 1, W);

% ---- the board -----------------------------------------------------------
if isempty(R.bench)
    L{end+1} = 'BOOK BENCHMARKS  ***NOT RUN*** (benchmark = false).  R.pass is false, meaning';
    L{end+1} = '  "nothing was checked", NOT "everything is fine".';
else
    L{end+1} = 'BOOK BENCHMARKS   computed vs published, with the zarchan.txt line each came from';
    L{end+1} = sprintf('  %-44s %-22s %-22s %-9s %s', 'what', 'book', 'ours', 'band', 'verdict');
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch b.kind
        case 'info'
            L{end+1} = sprintf('  %-44s %-22s %-22s %-9s INFO', ...
                trunc(b.what, 44), trunc(b.book_str, 22), trunc(b.ours_str, 22), 'info'); %#ok<AGROW>
        case 'skip'
            L{end+1} = sprintf('  %-44s %-22s %-22s %-9s SKIP (%s)', ...
                trunc(b.what, 44), trunc(b.book_str, 22), '--', 'skip', b.why); %#ok<AGROW>
        case {'shape', 'bound'}
            L{end+1} = sprintf('  %-44s %-22s %-22s %-9s %s', ...
                trunc(b.what, 44), trunc(b.book_str, 22), trunc(b.ours_str, 22), ...
                b.kind, tf2s(b.ok)); %#ok<AGROW>
        otherwise
            if b.skipped
                L{end+1} = sprintf('  %-44s %-22s %-22s %-9s SKIP (%s)', ...
                    trunc(b.what, 44), [b.book_str ' ' b.units], '--', b.band, b.why); %#ok<AGROW>
            else
                L{end+1} = sprintf('  %-44s %-22s %-22s %-9s %s  (%+.2f%%)', ...
                    trunc(b.what, 44), [b.book_str ' ' b.units], ...
                    [b.ours_str ' ' b.units], ...
                    sprintf('%s %.3g%%', b.band, 100*b.tol), tf2s(b.ok), b.pct); %#ok<AGROW>
            end
        end
        L{end+1} = sprintf('      src: %s', b.source); %#ok<AGROW>
    end
    ev = ~[R.bench.skipped];
    L{end+1} = repmat('-', 1, W);
    L{end+1} = sprintf('  %d rows: %d evaluated, %d passed, %d failed, %d skipped/INFO', ...
        numel(R.bench), sum(ev), sum(ev & [R.bench.ok]), ...
        sum(ev & ~[R.bench.ok]), sum(~ev));
    L{end+1} = '  bands: exact = echoed parameter/identity (1e-12);  text = prose to 2-3 figs (2%);';
    L{end+1} = '         graph = read off Fig. 20.8''s ordinate (15%);  xchap = nonlinear Ch.28 run';
    L{end+1} = '         vs linear Ch.20 closed form, the book asserting agreement without digits (15%).';
    L{end+1} = '  shape/bound rows carry no tolerance; INFO rows are neither passed nor failed.';
end
L{end+1} = repmat('=', 1, W);
L{end+1} = 'READ THIS BEFORE QUOTING ANYTHING ABOVE IN A REPORT:';
L{end+1} = '  * this is a POINT MASS with one lag per channel.  No gravity, drag, thrust,';
L{end+1} = '    airframe, seeker or noise.  A miss here is not a sortie prediction.';
L{end+1} = '  * the chapter''s transferable result is a WARNING, not a number: against a';
L{end+1} = '    circularly evading target the single-plane miss oscillates with flight time';
L{end+1} = '    while the 3D total is CONSTANT and equal to the single-plane PEAK.  Every';
L{end+1} = '    single-plane statistic we publish is one phase sample of that circle.';
s = strjoin(L, newline);
end


function s = lawName(qpn)
if qpn == 1
    s = 'proportional navigation (QPN = 1)';
else
    s = 'compensated weave (QPN = 0)';
end
end


function s = tf2s(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end


function s = trunc(t, n)
if isstring(t), t = char(t); end
t = regexprep(t, '\s+', ' ');
if numel(t) > n
    s = [t(1:n-1) '~'];
else
    s = t;
end
end


% =============================================================================
%   PLOTS.  The listing's own four figures (zarchan.txt:10323), plus the one
%   the book does not draw: the miss in the downrange/altitude PLANE, which is
%   where the circle it describes in prose is actually visible.
% =============================================================================
function drawCurves(R, C)
S = R.run;
figure('Name', 'Listing 28.1 -- miss components vs flight time');
nm = {'RTM1 (ft)  downrange', 'RTM2 (ft)  altitude', 'RTM3 (ft)  cross-range'};
dat = {S.miss1, S.miss2, S.miss3};
for k = 1:3
    subplot(2, 2, k);
    plot(S.tf, dat{k}, '-o', 'MarkerSize', 3); grid on;
    xlabel('Time (s)'); ylabel(nm{k});
    axis([0 max(S.tf) -30 30]);          % the book's own limits
end
subplot(2, 2, 4);
plot(S.tf, S.miss, '-o', 'MarkerSize', 3); grid on;
hold on;
if R.ch20.have
    plot([0 max(S.tf)], R.ch20.peak_miss_direct_ft*[1 1], 'r--', 'LineWidth', 1.2);
    legend('Listing 28.1 total miss', 'Ch.20 closed-form peak', 'Location', 'best');
end
xlabel('Time (s)'); ylabel('RTM (ft)  total');
axis([0 max(S.tf) 0 30]);

figure('Name', 'Listing 28.1 -- the circle the book describes in prose');
plot(S.miss1, S.miss2, 'o-'); grid on; axis equal;
xlabel('downrange miss (ft)'); ylabel('altitude miss (ft)');
title(sprintf(['miss locus over flight time  (Ch.20 peak = %.2f ft)\n' ...
               'the book states this is a circle (zarchan.txt:10325); it draws' ...
               ' the components, not this'], R.ch20.peak_miss_direct_ft));
if R.ch20.have
    th = linspace(0, 2*pi, 200);
    hold on;
    plot(R.ch20.peak_miss_direct_ft*cos(th), R.ch20.peak_miss_direct_ft*sin(th), 'r--');
end
end
