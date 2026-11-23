function R = adjointOptimal(opts)
%MGS.ZARCHAN.ADJOINTOPTIMAL  Zarchan Listing 8.2 -- adjoint of the OPTIMAL
%   (lag-compensating) single-time-constant guidance system, ported line for
%   line, with the Listing A.1 time-to-go-error extension.
%
%   R = mgs.zarchan.adjointOptimal()                  % book case, prints the table
%   R = mgs.zarchan.adjointOptimal('APN', 2)          % optimal law only
%   R = mgs.zarchan.adjointOptimal('TAU', 8.0)        % our measured lag
%   R = mgs.zarchan.adjointOptimal('SF', 0.9)         % 10% error on time to go
%   R = mgs.zarchan.adjointOptimal('render', false, 'assert', false)
%
%   The MGS.ZARCHAN.BENCHSUITE entry point is
%       mgs.zarchan.adjointOptimal('assert',false,'render',false,'plot',false)
%   which returns R.bench in the same shape MGS.ZARCHAN.ADJOINTBINOMIAL5 does
%   (what/book/ours/pct/band/tol/ok/skipped/why) plus source/units/detail.
%
%   Transcribed from zarchan.txt:2562-2573 (Listing 8.2, p.182-183) and
%   zarchan.txt:14103-14117 (Listing A.1, p.950-952, "MODIFIED LISTING 8.2").
%   Adjoint of Fig. 8.17, which is the adjoint of the Fig. 8.15 homing loop.
%
%   APN selects the guidance law, exactly as the book's own parameter does
%   (zarchan.txt:2560):
%       APN = 0   proportional navigation                    C3 = C4 = 0
%       APN = 1   augmented proportional navigation          C3 = 0.5 N', C4 = 0
%       APN = 2   OPTIMAL guidance                           all four gains live
%
% ===========================================================================
%   *** UNITS ARE THE BOOK'S.  FEET, ft/s, ft/s^2, g = 32.2, deg via 57.3. ***
%
%   The rest of mgs is metres.  This file is DELIBERATELY not converted so the
%   published numbers can be laid against it digit for digit with no factor in
%   between to be got wrong.  A miss distance of "12" is plausible in either
%   unit, so a silent ft->m here would produce a number that looks right and is
%   wrong by 3.28x.  Convert AT THE CALL SITE, visibly.
% ===========================================================================
%
%   WHY THIS ONE, AND WHY IT IS DIFFERENT FROM EVERY OTHER ZARCHAN PORT WE HAVE
%
%   Every other port in this package PRICES lag.  Ch.3 and Ch.6 answer "what
%   does a time constant T cost you in miss distance".  This listing is the
%   first one that COMPENSATES for it: the optimal law is derived knowing T,
%   and its gains contain T and time-to-go explicitly.  The book's claim about
%   Fig. 8.18 is blunt: the optimal law "always yields zero miss distance", and
%   it does so by making the lagged system behave like a ZERO-LAG augmented-PN
%   system with an effective navigation ratio of 3 (zarchan.txt:2574).
%
%   That matters to us because our measured problem IS a lag we cannot remove:
%   flight-path-angle tracking gain 0.063 against the model's 1.385, roughly
%   8 s of it, saturating on 26 of 32 sorties.
%
%   *** BEFORE ANYONE PROPOSES FLYING THIS LAW, READ THE NEXT PARAGRAPH. ***
%
%   The optimal law needs time-to-go, and the book measures how fast it rots
%   when time-to-go is wrong (zarchan.txt:14101, Fig. A.1, p.950).  A 0.9 scale
%   factor on tgo is, in the book's words, "worse than that of proportional
%   navigation for flight times greater than 8 s"; a 0.1 s bias crosses over at
%   5.5 s.
%
%   A TEN PERCENT scale error on tgo already loses to plain PN.  Our own dive
%   predictor is measured to under-estimate dive duration by a median of
%   +18.7 s, one-sided positive on 27 of 27 samples, against dive times of
%   35-47 s -- a scale error around 1.4-1.5, i.e. four to five times past the
%   point where this law is beaten by the PN we already fly.  Those two rows
%   are SCORED benchmarks below (bench rows "A.1 SF=0.9 crossover" and
%   "A.1 BIAS=0.1 crossover") precisely so this file cannot be quoted as
%   evidence for adopting the law while the predictor is in that state.
%
%   Two further limits, both structural:
%     * the lag here is LINEAR and never saturates.  Ours saturates on 26 of
%       32 sorties.  A law that cancels a linear lag does not cancel a
%       saturated one; it commands harder and saturates sooner.
%     * the only disturbances in this adjoint are a STEP TARGET MANEUVER and
%       an INITIAL HEADING ERROR.  Our bullseye is stationary.  The heading-
%       error channel is the one that maps onto our measured discriminant
%       (release-side |eps0|), and it is REPORTED here but the book publishes
%       no value for it, so it is not gated.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION, AND THE ONE THING I HAD TO RESOLVE BY HAND
%
%   The extractor's header for listing_08_02.m claims "unresolved: (none)".
%   IT IS WRONG.  Line 10 of that file still reads
%
%       while TP,=(TF-1e-5)
%
%   The ',' is an OCR'd '<'.  The extractor's repair regex requires a word
%   character after the '=' and this one is followed by '(', so it was skipped
%   AND NOT REPORTED.  The same line is unrepaired in listing_08_01.m for the
%   same reason.  Resolved here as  while TP <= (TF - 1e-5)  from context: it
%   is the outer sweep over flight time and TF is its end, the identical line
%   appears in Listings 3.1, 8.1 and A.1, and the already-repaired sibling
%   `while STEP,=1` -> `while STEP <= 1` fixes the reading of ',' as '<='.
%   Every other ',='/'.=' in this listing was repaired by the extractor.
%
%   The prose equations for N' and C4 (zarchan.txt:2525, :2534-2540) have lost
%   their minus signs to the same OCR damage -- the text prints "e^x" where the
%   listing has exp(-X), and "C4 = N'(...)" where the listing has -XNPP*(...).
%   THE LISTING IS AUTHORITATIVE and is what is transcribed.  The two agree
%   once the dropped minus signs are restored, which is the check that this is
%   OCR damage and not a genuine discrepancy.
%
% ---------------------------------------------------------------------------
%   THE 1e-4 IN XNPP IS NOT NOISE.  IT CHANGES THE ANSWER AT SHORT FLIGHT TIME.
%
%   The book computes the optimal navigation ratio as
%
%       XNPP = TOP / (.0001 + BOT1 + BOT2)
%
%   and that .0001 is a REGULARIZER, not a typo.  Series-expand the exact
%   expression about x = tgo/T = 0:
%
%       TOP = 3x^4 - x^5 + ...          BOT1+BOT2 = 0.3x^5 - x^6/6 + ...
%
%   -- the denominator's first FIVE Taylor coefficients cancel identically, so
%   the unregularized N' behaves as 10/x and diverges at intercept.  The .0001
%   dominates the denominator for x below about 0.4, which drags N' back DOWN
%   and then to zero:
%
%       x        0.1      0.2      0.3      0.5      1.0
%       guarded  2.82    24.16    30.63    21.96    12.29
%       exact   102.2    52.24    35.58    22.26    12.30
%
%   So the book's own implementation does NOT fly the optimal law for
%   tgo/T < ~0.4, and the "always yields zero miss" claim cannot be tested as
%   an absolute zero there.  Whatever miss accumulates inside that window
%   persists as a CONSTANT OFFSET on the miss curve for every longer flight
%   time, because x1 is an integral and nothing later removes it.  The zero-
%   miss benchmark below is therefore scored in two pieces -- an offset piece
%   and a growth piece -- and the growth piece is the sharp one.  Do not
%   "improve" the .0001 away: it is the book's code, the published figure is
%   what it produces, and a different regularizer produces a different figure.
%
% ---------------------------------------------------------------------------
%   LISTING 8.2 WITH APN=0 IS ALGEBRAICALLY THE SAME SYSTEM AS LISTING 3.1
%
%   This is used by two benchmarks, so here is the derivation rather than an
%   assertion.  With APN = 0 the four equations are
%
%       x1' = x2                       x2' = x3 + N' x4 /(T tgo)
%       x3' = N' x4 /(T tgo^2)         x4' = -x4/T - x2
%
%   Define  v = x2' = x3 + N' x4/(T tgo)  and  w = x4' = -(x4/T + x2).  In
%   adjoint time tgo IS the running time, so d(tgo)/dt = +1 and
%
%       v' = N' x4/(T tgo^2) + (N'/T)[x4'/tgo - x4/tgo^2] = N' w /(T tgo)
%       w' = -(x4'/T + x2')                               = -w/T - v
%
%   which is Listing 3.1 term for term (its Y1 = (x4-x2)/T plays the role of
%   -w, its x3 the role of v).  The initial conditions map too: both start
%   from x1 = x2 = x4 = 0, x3 = 1, giving v(0) = 1 and w(0) = 0 in each.
%   Hence Fig. 3.13's published "nearly 12 ft" is a legitimate benchmark for
%   THIS listing's APN = 0 branch, and the Ch.3 port's analytic closed form is
%   a legitimate second opinion on this integrator.  Both are wired below.
%
%   The two are DIFFERENT STATE REALIZATIONS, so they carry independent
%   discretization error; they agree in exact arithmetic, not bit for bit.
%
% ---------------------------------------------------------------------------
%   APN = 0 AND APN = 1 SHARE x2, x3 AND x4 EXACTLY
%
%   The augmentation term enters through C3, C3 appears only in x1', and x1
%   feeds nothing.  So switching APN between 0 and 1 CANNOT move x2/x3/x4, and
%   the heading-error miss -VM*HE*x2 is identical for the two laws.  Only the
%   target-maneuver miss differs, by  XNT * 0.5*N'/T * integral(x4).  That is
%   an exact structural property and it is checked at 1e-12 below; if it ever
%   fails, the C3 term has been wired into the wrong derivative.
%
% ---------------------------------------------------------------------------
%   THE OUTPUTS ARE SENSITIVITIES.  YOU MUST MULTIPLY THEM.
%
%       x1  [s^2]  ->  miss from a step target maneuver   = x1 * nT
%       x2  [s]    ->  miss from an initial heading error = x2 * (-VM*HE)
%
%   Both products are formed for you as .miss_nt_ft and .miss_he_ft.  The
%   scale factor on the heading-error channel is -VM*HE and NOT 2*VM*HE: the
%   "2" in the p.46 prose is an OCR'd minus sign, established in detail in
%   MGS.ZARCHAN.ADJOINTSINGLELAG's header.  This listing's own line
%   ArrayXMHE(n) = -VM*HE*X2 settles it.
%
% ---------------------------------------------------------------------------
%   OUTPUT
%
%   R.t             1xM adjoint time [s] = original-system FLIGHT TIME tF,
%                   equally readable as "time to go at which the step lands"
%   R.law(k)        one per APN value requested, in the order given
%                     .apn .name .x1 .x2 .x3 .x4
%                     .miss_nt_ft   XNT*x1     [ft]
%                     .miss_he_ft   -VM*HE*x2  [ft]
%                     .peak_nt_ft   max|miss_nt_ft|
%   R.err(j)        the two Listing A.1 degraded-tgo cases (SF=0.9 / BIAS=0.1)
%                     .name .sf .bias .miss_nt_ft .crossover_s
%   R.nav           the Fig. 8.14 kernel: .x .np (guarded) .np_exact
%   R.gains         @(tgo) -> struct with C1..C4, XNPP, TGOH, x.  Exposed so
%                   callers never retype the kernel; it IS the loop's kernel.
%   R.bench         struct array, computed vs book, with verdicts
%   R.selfcheck     structural invariants and their measured residuals
%   R.pass          true iff every evaluable benchmark row passed
%   R.summary       the printed text
%
%   See also MGS.ZARCHAN.ADJOINTSINGLELAG, MGS.ZARCHAN.ADJOINTBINOMIAL5,
%   MGS.ZARCHAN.BENCHSUITE, MGS.ZARCHAN.CROSSCHECK.

arguments
    % Book names are UPPERCASE in Listing 8.2; kept, so a reader can diff the
    % option list against zarchan.txt:2562 without translating anything.
    opts.APN    (1,:) double {mustBeMember(opts.APN,[0 1 2])} = [0 1 2]
    opts.XNT    (1,1) double {mustBeFinite}                 = 96.6    % ft/s^2 = 3 g
    opts.XNP    (1,1) double {mustBeFinite, mustBeNonnegative} = 4.0  % N', APN 0/1 only
    opts.TAU    (1,1) double {mustBeFinite, mustBePositive} = 1.0     % T [s]
    opts.TF     (1,1) double {mustBeFinite, mustBePositive} = 10.0    % s
    opts.VM     (1,1) double {mustBeFinite, mustBePositive} = 3000.0  % ft/s
    opts.HEDEG  (1,1) double {mustBeFinite}                 = -20.0   % deg
    opts.H      (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s, RK2 step
    opts.samp   (1,1) double {mustBeFinite, mustBePositive} = 0.1     % s, record every
    % Listing A.1 extension.  SF = 1, BIAS = 0 reduces the code below to
    % Listing 8.2 EXACTLY (TGOH == TGO bit for bit), which is why the FLYING
    % code has one path and not two: a second copy of the gain kernel in the
    % loop is a copy that drifts.  (STRUCTURALCHECK does keep a second copy,
    % deliberately, as a test oracle pinned to the un-extended listing -- see
    % item 3 there.)  These affect the APN = 2 branch only: the book does not put
    % tgo errors on PN "because in practice this guidance law does not require
    % time to go but works directly on the line-of-sight rate" (:14100).
    opts.SF     (1,1) double {mustBeFinite, mustBePositive} = 1.0     % tgo scale factor
    opts.BIAS   (1,1) double {mustBeFinite}                 = 0.0     % tgo bias [s]
    opts.ttgo   (1,1) logical = true    % also run the two Fig. A.1 degraded cases
    opts.render (1,1) logical = true
    opts.plot   (1,1) logical = false
    opts.assert (1,1) logical = true
    % Internal.  The open-loop invariant re-runs the integrator with XNP = 0;
    % that inner run must not self-check again.  There is no `nargin` switch
    % anywhere in this file: in a function whose arguments block declares only
    % name-value options MATLAB reports nargin = 0 however many were passed
    % (measured, R2025b), so `if nargin == 0` is ALWAYS TRUE and silently
    % replaces the caller's parameters.  Explicit flags only.
    opts.selfcheck (1,1) logical = true
    opts.crosscheck (1,1) logical = true   % compare APN=0 against the Ch.3 port
end

if numel(unique(opts.APN)) ~= numel(opts.APN)
    error('mgs:zarchan:adjointOptimal:dupAPN', ...
        ['''APN'' contains a repeat: %s.  Two rows of the same law would ' ...
         'print as if two laws had been compared.'], mat2str(opts.APN));
end
if opts.H > opts.samp
    error('mgs:zarchan:adjointOptimal:step', ...
        ['H = %g s exceeds the sample interval samp = %g s, so the curve ' ...
         'would be sampled at the step rate and silently disagree with its ' ...
         'own time vector.'], opts.H, opts.samp);
end
if opts.H >= opts.TAU
    error('mgs:zarchan:adjointOptimal:stiff', ...
        ['H = %g s is not smaller than the guidance time constant TAU = %g s. ' ...
         'RK2 does not blow up here, it returns a smooth and entirely wrong ' ...
         'curve, so this is refused rather than warned about.'], opts.H, opts.TAU);
end
if opts.TF <= 2*opts.samp
    error('mgs:zarchan:adjointOptimal:horizon', ...
        'TF = %g s leaves fewer than three samples; there is no curve to return.', ...
        opts.TF);
end

HE = opts.HEDEG / 57.3;      % book's own conversion; 57.3 is not 180/pi and
                             % swapping in the exact value moves HE in the 4th
                             % digit for no gain.  Kept as the book has it.
HE_SCALE = -opts.VM * HE;    % ft/s, +1047.120 at the nominal case

R = struct();
R.units   = 'feet, ft/s, ft/s^2; g = 32.2; deg via 57.3 -- Zarchan''s units, NOT SI';
R.listing = ['Zarchan Listing 8.2 (zarchan.txt:2562), adjoint of Fig. 8.17; ' ...
             'tgo-error terms from Listing A.1 (zarchan.txt:14111)'];
R.opts     = opts;
R.he_scale = HE_SCALE;
R.gains    = @(tgo) gainStruct(tgo, opts.TAU, opts.SF, opts.BIAS);
R.book_case = isBookCase(opts);

% ---- the runs -------------------------------------------------------------
K = numel(opts.APN);
for k = 1:K
    A = adjointRun(opts.APN(k), opts.XNP, opts.TAU, opts.TF, opts.H, ...
                   opts.samp, opts.SF, opts.BIAS);
    R.law(k).apn  = opts.APN(k);
    R.law(k).name = lawName(opts.APN(k));
    R.law(k).sf   = opts.SF;
    R.law(k).bias = opts.BIAS;
    R.law(k).x1   = A.x1;   R.law(k).x2 = A.x2;
    R.law(k).x3   = A.x3;   R.law(k).x4 = A.x4;
    R.law(k).miss_nt_ft = opts.XNT * A.x1;
    R.law(k).miss_he_ft = HE_SCALE  * A.x2;
    R.law(k).peak_nt_ft = max(abs(R.law(k).miss_nt_ft));
    if k == 1
        R.t = A.tp;
    elseif numel(A.tp) ~= numel(R.t) || max(abs(A.tp - R.t)) > 1e-9
        % Same TF/H/samp, so the grids are the same sequence of float adds.
        % If they are not, something below is not deterministic and every
        % law-to-law comparison in the benchmark table is comparing curves
        % sampled at different times.
        error('mgs:zarchan:adjointOptimal:grid', ...
            ['the APN = %g run landed on a different time grid from the ' ...
             'APN = %g run.  Law-to-law comparisons would be meaningless.'], ...
            opts.APN(k), opts.APN(1));
    end
end

% ---- the two Listing A.1 degraded-time-to-go cases -------------------------
% Run unconditionally of opts.APN: they are APN = 2 by definition and their
% benchmark is a comparison against the APN = 0 curve, not against opts.
R.err = emptyErr();
if opts.ttgo
    R.err = runTtgoCases(opts, HE_SCALE);
end

% The crossover flight times are computed HERE and not inside BENCHMARKS.
% BENCHMARKS receives a COPY of R; anything it assigns into R.err would be
% discarded on return and the printed table would report "none found" for a
% crossover it had just computed correctly.  That is a silent-wrong-answer
% shape, so the computation lives where the result can escape.
j0_ = find(opts.APN == 0, 1);
for jj = 1:numel(R.err)
    if numel(R.err(jj).miss_nt_ft) ~= numel(R.t)
        error('mgs:zarchan:adjointOptimal:errGrid', ...
            ['the Listing A.1 case "%s" landed on a %d-sample grid against ' ...
             'the main run''s %d.  Comparing them pointwise would compare ' ...
             'different flight times.'], R.err(jj).name, ...
            numel(R.err(jj).miss_nt_ft), numel(R.t));
    end
    if isempty(j0_)
        R.err(jj).crossover_s = NaN;    % no PN curve to be worse than
    else
        R.err(jj).crossover_s = lastCrossAbove(R.t, ...
            abs(R.err(jj).miss_nt_ft), abs(R.law(j0_).miss_nt_ft));
    end
end

% ---- the Fig. 8.14 kernel, evaluated analytically --------------------------
R.nav = navTable(opts.TAU);

% ---- structural self-check ------------------------------------------------
if opts.selfcheck
    R.selfcheck = structuralCheck(opts);
else
    R.selfcheck = struct('skipped', true);
end

% ---- book benchmarks ------------------------------------------------------
R.bench = benchmarks(R, opts);
ev = ~[R.bench.skipped];
R.pass = any(ev) && all([R.bench(ev).ok]);

R.summary = render(R, opts);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawFigures(R, opts); end

if opts.assert
    % "Nothing was evaluated" must NOT read as "everything passed".
    if ~any(ev)
        error('mgs:zarchan:adjointOptimal:noBenchmark', ...
            ['not one book benchmark could be evaluated.  The gain-kernel rows ' ...
             'need nothing at all, so reaching this message means the kernel ' ...
             'rows were skipped too -- refusing to hand back an unvalidated ' ...
             'port silently.  Pass ''assert'', false if you meant to run off-book.']);
    end
    if ~R.pass
        bad = find(~[R.bench.ok] & ev);
        error('mgs:zarchan:adjointOptimal:benchmark', ...
            ['%d of %d book benchmarks are outside tolerance (first: %s, book ' ...
             '%.4g, ours %.4g, band %s).  Either the port broke or the ' ...
             'parameters left the region the book covers.  Full table:\n\n%s'], ...
            numel(bad), sum(ev), R.bench(bad(1)).what, R.bench(bad(1)).book, ...
            R.bench(bad(1)).ours, R.bench(bad(1)).band, R.summary);
    end
end
end


% =============================================================================
function s = lawName(apn)
switch apn
case 0, s = 'PN';
case 1, s = 'APN';
case 2, s = 'OPTIMAL';
otherwise
    error('mgs:zarchan:adjointOptimal:apn', ...
        ['APN = %g is not a guidance law the book defines.  Listing 8.2 has ' ...
         'exactly three: 0 = PN, 1 = augmented PN, 2 = optimal (:2560).'], apn);
end
end


% =============================================================================
function G = gainStruct(tgo, TAU, SF, BIAS)
%GAINSTRUCT  The guidance gains C1..C4 at one time-to-go.  THE loop uses this
%   function, so a benchmark on it is a benchmark on what actually flew.
%
%   The book prints (zarchan.txt:2534-2540), minus signs restored:
%       C1 = N'/tgo^2      C2 = N'/tgo      C3 = 0.5 N'
%       C4 = -N' (e^-x + x - 1) / x^2 ,      x = tgo/T
%   with N' itself a function of x (:2525).  For APN 0/1 the book substitutes
%   the constant XNP for N' and zeroes C3/C4 as appropriate; that substitution
%   lives in ADJOINTRUN, not here, because it is a branch of the listing.
[C1, C2, C3, C4, XNPP, TGOH, X] = optimalGains(tgo, TAU, SF, BIAS);
G = struct('C1',C1, 'C2',C2, 'C3',C3, 'C4',C4, 'XNPP',XNPP, ...
           'TGOH',TGOH, 'x',X, ...
           'note','optimal-law gains; APN 0/1 substitute the constant XNP');
end


% -----------------------------------------------------------------------------
function [C1, C2, C3, C4, XNPP, TGOH, X] = optimalGains(TGO, TAU, SF, BIAS)
%OPTIMALGAINS  The APN == 2 branch of Listing 8.2, with Listing A.1's TGOH.
%   Transcribed verbatim; the variable names are the book's.
%
%   TWO THINGS THAT LOOK LIKE BUGS AND ARE NOT:
%     * the .0001 added to the denominator of XNPP.  See the file header --
%       the denominator's first five Taylor coefficients cancel, so without it
%       N' diverges as 10/x at intercept.  It is a regularizer and it changes
%       the published answer at short flight time.
%     * the gains use TGOH, the ESTIMATED time to go, everywhere -- including
%       in C1 = XNPP/(TGOH*TGOH).  That is the whole point of Listing A.1: the
%       error propagates into the gains, not just into x.  With SF = 1 and
%       BIAS = 0, TGOH == TGO bit for bit and this is Listing 8.2 unchanged.
TGOH = SF*TGO + BIAS;
if TGOH < 0                       % book's clamp, `if TGOH,0` at :14111.
    TGOH = 0.0001;                % NOTE it is `< 0`, not `<= 0`: an exact zero
end                               % falls through.  ADJOINTRUN refuses inputs
                                  % that can produce one -- see there.
X    = TGOH/TAU;
TOP  = 6.*X*X*(exp(-X)-1.+X);
BOT1 = 2*X*X*X+3.+6.*X-6.*X*X;
BOT2 = -12.*X*exp(-X)-3.*exp(-2.*X);
XNPP = TOP/(.0001+BOT1+BOT2);
C1   = XNPP/(TGOH*TGOH);
C2   = XNPP/TGOH;
C3   = .5*XNPP;
C4   = -XNPP*(exp(-X)+X-1.)/(X*X);
end


% -----------------------------------------------------------------------------
function np = optimalNavExact(x)
%OPTIMALNAVEXACT  The same N' WITHOUT the .0001 regularizer, vectorized.
%   Used only to report how much the regularizer moves the answer.  It is not
%   what the loop flies and must never be substituted for it.
top  = 6.*x.^2.*(exp(-x)-1.+x);
bot  = 2*x.^3+3.+6.*x-6.*x.^2-12.*x.*exp(-x)-3.*exp(-2.*x);
np   = top./bot;
end


% =============================================================================
function A = adjointRun(APN, XNP, TAU, TF, H, samp, SF, BIAS)
%ADJOINTRUN  Listing 8.2 itself (zarchan.txt:2562-2569), with A.1's SF/BIAS.
%
%   Fig. 8.17's four adjoint states.  Only x1 and x2 have meaning:
%       x1  MNT node -- miss sensitivity to a STEP target maneuver   [s^2]
%       x2  MHE node -- miss sensitivity to a HEADING ERROR          [s]
%       x3  carries the unit impulse that makes this a MISS adjoint  [-]
%       x4  the guidance lag state
%
%   x3 STARTS AT 1.  That unity initial condition IS the impulse (:2452 for
%   the identical construction in Listing 8.1).  Zero it and every curve comes
%   out flat zero, which reads as "no miss" rather than "no simulation".
%
%   TGO = TP + 1e-5 and TP itself starts at 1e-5.  Both offsets are the book's
%   dodge for the 1/tgo and 1/tgo^2 gains at intercept.  Remove either and the
%   first derivative evaluation divides by zero.
%
%   Do not replace the hand-rolled predictor/corrector with ode45.  The
%   published curves are what THIS scheme at THIS step size produces.

% The book's clamp is `if TGOH < 0`, so an exactly-zero TGOH survives it and
% divides by zero in C2 -- giving Inf, then NaN, then a curve of NaNs that
% plots as nothing rather than as an error.  Refuse the inputs that can do it
% instead of hardening the book's line.
if APN == 2 && BIAS < 0
    tgo_hits_zero = (-BIAS/SF);
    if tgo_hits_zero >= 0 && tgo_hits_zero <= TF
        error('mgs:zarchan:adjointOptimal:tgohZero', ...
            ['SF = %g with BIAS = %g makes the estimated time to go pass ' ...
             'through zero at tgo = %g s, inside the 0..%g s sweep.  The ' ...
             'book''s guard is `if TGOH < 0`, which does not catch an exact ' ...
             'zero: C2 = XNPP/TGOH would be Inf and the whole curve would ' ...
             'come back NaN.  Refusing rather than patching the book''s line.'], ...
            SF, BIAS, tgo_hits_zero, TF);
    end
end

nMax = ceil(TF/samp) + 4;
tpA  = zeros(1, nMax);
x1A  = zeros(1, nMax);  x2A = zeros(1, nMax);
x3A  = zeros(1, nMax);  x4A = zeros(1, nMax);

T0 = 0.0;
S  = 0.0;
TP = T0 + 0.00001;                 % TP=T+.00001
X1 = 0.0;  X2 = 0.0;  X3 = 1.0;  X4 = 0.0;   % X3=1 -- the adjoint impulse
n  = 0;

% Initialised before the loop so the FLAG==1 branch cannot read a stale value.
X1D = 0.0;  X2D = 0.0;  X3D = 0.0;  X4D = 0.0;

while TP <= (TF - 1e-5)            % book: `while TP,=(TF-1e-5)`, see header
    X1OLD = X1;  X2OLD = X2;  X3OLD = X3;  X4OLD = X4;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  Pass 1 evaluates at the current
    % state; pass 2 takes a full Euler step, ADVANCES TP BY H, and re-evaluates
    % there.  TP is advanced inside this loop, exactly once, and is not rolled
    % back -- so the 1/tgo gains are evaluated at both times.  Move that line
    % out and the scheme quietly degrades to first order while still running.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            X1 = X1 + H*X1D;  X2 = X2 + H*X2D;
            X3 = X3 + H*X3D;  X4 = X4 + H*X4D;
            TP = TP + H;
        end

        TGO = TP + 0.00001;
        if APN == 0
            C1 = XNP/(TGO*TGO);  C2 = XNP/TGO;  C3 = 0.;      C4 = 0.;
        elseif APN == 1
            C1 = XNP/(TGO*TGO);  C2 = XNP/TGO;  C3 = .5*XNP;  C4 = 0.;
        else
            [C1, C2, C3, C4] = optimalGains(TGO, TAU, SF, BIAS);
        end

        X1D = X2 + C3*X4/TAU;
        X2D = X3 + C2*X4/TAU;
        X3D = C1*X4/TAU;
        X4D = -X4/TAU - X2 + C4*X4/TAU;
        FLAG = 1;
    end
    FLAG = 0;

    % Heun corrector.  X1D..X4D here are the SECOND-pass derivatives;
    % averaging with X1OLD is what reintroduces the first.
    X1 = (X1OLD + X1)/2 + .5*H*X1D;
    X2 = (X2OLD + X2)/2 + .5*H*X2D;
    X3 = (X3OLD + X3)/2 + .5*H*X3D;
    X4 = (X4OLD + X4)/2 + .5*H*X4D;

    % Book writes `if S >= .0999`, a threshold hard-wired to H = 0.01 and
    % samp = 0.1.  The form below is bit-identical at the book's values (both
    % fire on the 10th step and not the 9th) and stays correct elsewhere.
    S = S + H;
    if S >= samp - 0.5*H
        S = 0.0;
        n = n + 1;
        tpA(n) = TP;
        x1A(n) = X1;  x2A(n) = X2;  x3A(n) = X3;  x4A(n) = X4;
    end
end

if n < 3
    error('mgs:zarchan:adjointOptimal:noSamples', ...
        ['the adjoint produced %d samples (TF = %g, samp = %g).  Returning ' ...
         'that would hand the caller a curve that plots as nothing at all.'], ...
        n, TF, samp);
end

A.tp = tpA(1:n);
A.x1 = x1A(1:n);  A.x2 = x2A(1:n);
A.x3 = x3A(1:n);  A.x4 = x4A(1:n);

% A NaN or Inf anywhere means a gain divided by zero somewhere in the sweep.
% Caught here rather than downstream, where a NaN curve silently makes every
% max() and every comparison come back NaN and every benchmark come back false
% for the wrong reason.
if ~all(isfinite([A.x1 A.x2 A.x3 A.x4]))
    error('mgs:zarchan:adjointOptimal:nonFinite', ...
        ['the APN = %g run produced non-finite states (TAU = %g, SF = %g, ' ...
         'BIAS = %g).  Something divided by zero inside the gain kernel; the ' ...
         'curve is not a curve and is not being returned.'], APN, TAU, SF, BIAS);
end
end


% =============================================================================
function E = emptyErr()
E = struct('name',{}, 'sf',{}, 'bias',{}, 'miss_nt_ft',{}, 'miss_he_ft',{}, ...
           'crossover_s',{}, 'book_crossover_s',{});
end


function E = runTtgoCases(opts, HE_SCALE)
%RUNTTGOCASES  Listing A.1's two published degraded-time-to-go cases.
%
%   The book reran three cases against each other (zarchan.txt:14101): plain PN
%   (APN = 0); optimal guidance with a tgo SCALE FACTOR error, SF = 0.9 and
%   BIAS = 0; and optimal guidance with a tgo BIAS error, SF = 1 and BIAS = 0.1
%   s.  The two parameter sets below are those, verbatim.
%
%   Their benchmark is the crossover flight time past which each is WORSE than
%   plain PN: 8 s and 5.5 s respectively, both stated in the running text and
%   therefore greppable -- unlike anything that lives only on Fig. A.1 itself.
E = emptyErr();
spec = {'SF=0.9  (10% tgo scale error)', 0.9, 0.0, 8.0
        'BIAS=0.1 s (tgo bias error)',   1.0, 0.1, 5.5};
for j = 1:size(spec,1)
    A = adjointRun(2, opts.XNP, opts.TAU, opts.TF, opts.H, opts.samp, ...
                   spec{j,2}, spec{j,3});
    E(j).name = spec{j,1};
    E(j).sf   = spec{j,2};
    E(j).bias = spec{j,3};
    E(j).miss_nt_ft = opts.XNT * A.x1;
    E(j).miss_he_ft = HE_SCALE  * A.x2;
    E(j).book_crossover_s = spec{j,4};
    % Filled in by the CALLER, which has the PN curve to compare against.
    % Left NaN here rather than 0 so a caller that forgets gets "none found"
    % printed instead of a crossover at the origin.
    E(j).crossover_s = NaN;
end
end


% =============================================================================
function N = navTable(TAU)
%NAVTABLE  Fig. 8.14 -- the optimal effective navigation ratio against x=tgo/T.
%   Computed from the SAME kernel the loop runs, so the printed table is not a
%   second implementation that can disagree with the simulation.
x = [0.1 0.2 0.3 0.5 1 2 3 5 10 20 50 1000];
np = zeros(size(x));
for i = 1:numel(x)
    [~,~,~,~,np(i)] = optimalGains(x(i)*TAU, TAU, 1.0, 0.0);
end
N.x        = x;
N.np       = np;                        % what the book's code uses
N.np_exact = optimalNavExact(x);        % without the .0001 regularizer
N.tau      = TAU;
end


% =============================================================================
function C = structuralCheck(opts)
%STRUCTURALCHECK  Invariants that must hold or every curve above is garbage.
%
%   These are NOT benchmarks.  They test the WIRING using properties the loop
%   has for structural reasons, and they are the only checks in this file that
%   still bite when the caller runs off-book parameters.
%
%   1. OPEN LOOP.  With APN = 0 and XNP = 0 all four gains are zero, so the
%      loop is cut and the equations collapse to x1' = x2, x2' = x3, x3' = 0
%      with x3(0) = 1.  Then x2 = t and x1 = t^2/2 EXACTLY -- Heun integrates
%      a linear integrand exactly, so this is a machine-precision test, not an
%      approximate one.  It pins the two output integrators and the corrector
%      against analytic truth with no book reading anywhere in it.  Referenced
%      to (TP - 1e-5) rather than TP because the sweep starts at +1e-5.
%
%   2. C3 CANNOT REACH x2, x3 OR x4.  The augmentation gain appears only in
%      x1', and x1 feeds nothing, so APN = 0 and APN = 1 must produce
%      BIT-IDENTICAL x2/x3/x4 and therefore an identical heading-error miss.
%      This is the check that catches C3 being wired into the wrong derivative
%      -- a fault that leaves both curves smooth and plausible.
%
%   3. SF = 1, BIAS = 0 IS LISTING 8.2.  The A.1 extension must be inert at
%      its defaults, bit for bit, or the "port" is not a port of Listing 8.2.

C = struct();
C.skipped = false;

% -- 1. open loop against the analytic answer -------------------------------
A0  = adjointRun(0, 0.0, opts.TAU, opts.TF, opts.H, opts.samp, 1.0, 0.0);
te  = A0.tp - 1e-5;
sel = te >= max(opts.TAU, 10*opts.samp);
if ~any(sel)
    error('mgs:zarchan:adjointOptimal:openLoopHorizon', ...
        ['TF = %g s is too short for the open-loop invariant to be evaluated ' ...
         '(it needs samples past t = %g s).  An unevaluated invariant is not ' ...
         'a passing one, so this is an error rather than a skip.'], ...
        opts.TF, max(opts.TAU, 10*opts.samp));
end
ref1 = te.^2/2;
C.openloop_rel = max(abs(A0.x1(sel) - ref1(sel)) ./ abs(ref1(sel)));
% 1e-8, not 1e-12.  Heun is exact on this integrand, so the only residual is
% the float accumulation of TP (n adds of H) against the float accumulation of
% x2 (the same n adds, from a different start) -- of order n*eps*TF, i.e. a
% few 1e-13 at the book's settings.  Any structural break here is O(1), so
% there is no daylight between passing and being right.
C.openloop_tol = 1e-8;
C.openloop_ok  = C.openloop_rel <= C.openloop_tol;

% -- 2. C3 is confined to x1' -----------------------------------------------
% Only evaluable when XNP > 0; at XNP = 0 the two laws are the same code.
if opts.XNP > 0
    Ap = adjointRun(0, opts.XNP, opts.TAU, opts.TF, opts.H, opts.samp, 1.0, 0.0);
    Aa = adjointRun(1, opts.XNP, opts.TAU, opts.TF, opts.H, opts.samp, 1.0, 0.0);
    sc = max(abs([Ap.x2 Ap.x3 Ap.x4]));
    if sc <= 0
        error('mgs:zarchan:adjointOptimal:degenerate', ...
            ['x2/x3/x4 are identically zero, so any difference compares as ' ...
             'perfect.  Refusing to report a pass on an empty comparison.']);
    end
    C.c3_rel = max(abs([Ap.x2 Ap.x3 Ap.x4] - [Aa.x2 Aa.x3 Aa.x4])) / sc;
    C.c3_tol = 1e-12;                 % expected to be exactly 0.0
    C.c3_ok  = C.c3_rel <= C.c3_tol;
    C.c3_evaluated = true;
else
    C.c3_rel = 0;  C.c3_tol = 1e-12;  C.c3_ok = true;  C.c3_evaluated = false;
end

% -- 3. the A.1 extension is inert at SF = 1, BIAS = 0 ----------------------
% Compared against an INDEPENDENT transcription of Listing 8.2's own APN == 2
% branch -- the one WITHOUT TGOH.  This is the one place in the file where
% writing the kernel twice is right: the second copy is a test oracle pinned
% to the un-extended listing, and its whole job is to fail if the SF/BIAS
% plumbing ever changes what Listing 8.2 computes.  Delete it and there is no
% evidence left that this file still ports Listing 8.2 rather than only A.1.
%
% Required to agree EXACTLY.  With SF = 1 and BIAS = 0, TGOH = 1*TGO + 0 is
% TGO bit for bit in IEEE arithmetic, so every subsequent operation is the
% same operation on the same bits.  A tolerance here would let a real change
% hide inside it.
% Fixed grid, deliberately NOT tied to TF: this compares two EXPRESSIONS, so
% it needs coverage of x = tgo/T from the regularized end to the asymptotic
% one, not coverage of the caller's sweep.  Tying it to TF would make a long
% run pay for thousands of pointless evaluations.
tg = [0.05:0.05:5, 6:1:20, 50, 100] * opts.TAU;
d_gain = 0;
for i = 1:numel(tg)
    TGO = tg(i);
    [g1, g2, g3, g4] = optimalGains(TGO, opts.TAU, 1.0, 0.0);
    % ---- Listing 8.2, zarchan.txt:2566, transcribed with no TGOH ----
    Xb    = TGO/opts.TAU;
    TOPb  = 6.*Xb*Xb*(exp(-Xb)-1.+Xb);
    BOT1b = 2*Xb*Xb*Xb+3.+6.*Xb-6.*Xb*Xb;
    BOT2b = -12.*Xb*exp(-Xb)-3.*exp(-2.*Xb);
    XNPPb = TOPb/(.0001+BOT1b+BOT2b);
    b1 = XNPPb/(TGO*TGO);  b2 = XNPPb/TGO;  b3 = .5*XNPPb;
    b4 = -XNPPb*(exp(-Xb)+Xb-1.)/(Xb*Xb);
    % ----------------------------------------------------------------
    d_gain = max(d_gain, max(abs([g1 g2 g3 g4] - [b1 b2 b3 b4]) ./ ...
                             max(abs([b1 b2 b3 b4]), eps)));
end
C.a1_rel = d_gain;
C.a1_tol = 0;                          % bit-identical or it is not Listing 8.2
C.a1_ok  = C.a1_rel <= C.a1_tol;

C.ok = C.openloop_ok && C.c3_ok && C.a1_ok;
if ~C.ok
    error('mgs:zarchan:adjointOptimal:selfcheck', ...
        ['a STRUCTURAL invariant of the adjoint failed, so every curve this ' ...
         'function computed is wrong:\n' ...
         '  open loop (XNP=0) vs analytic t^2/2 : rel %.3e (tol %.1e) %s\n' ...
         '  C3 confined to x1'' (APN 0 vs 1)     : rel %.3e (tol %.1e) %s\n' ...
         '  A.1 extension inert at SF=1,BIAS=0  : rel %.3e (tol %.1e) %s\n' ...
         'These test the wiring, not the book.  A failure here means the ' ...
         'transcription of Listing 8.2 has been damaged.'], ...
        C.openloop_rel, C.openloop_tol, tf2s(C.openloop_ok), ...
        C.c3_rel, C.c3_tol, tf2s(C.c3_ok), ...
        C.a1_rel, C.a1_tol, tf2s(C.a1_ok));
end
end


% =============================================================================
function B = benchmarks(R, opts)
%BENCHMARKS  Computed vs published, with a band and a REASON for the band.
%
%   FOUR TOLERANCE CLASSES, and the difference is provenance, never
%   convenience.  Do not widen a band to turn a row green.
%
%   'kernel'  -- the row tests a CLOSED-FORM limit of the gain kernel against
%                a number the book states in words ("approaching 3").  No
%                simulation is involved, so the band is tight and absolute.
%   'graph'   -- a value the author read off a printed figure and quoted in
%                the text.  One or two significant figures.
%   'order'   -- the book states a DIRECTION or an ORDERING, not a number.
%                Scored as a boolean.  These are often stronger than a value
%                read off a plot, because no digit was lost in the reading.
%   'ratio'   -- the book states "zero" and zero cannot be tested at floating
%                point.  Scored as a fraction of the PN peak on the same run,
%                which is the comparison the figure itself makes.
%   'xport'   -- compared against an ANALYTIC second opinion carried by another
%                port.  The band here is discretization, not provenance: the
%                book value is exactly zero and the only question is how much
%                two integrators may disagree with the same closed form.

B = emptyBench();

TAU = opts.TAU;
j0  = find(opts.APN == 0, 1);
j1  = find(opts.APN == 1, 1);
j2  = find(opts.APN == 2, 1);
t   = R.t;

% -------------------------------------------------------------------------
% GAIN-KERNEL ROWS.  Pure algebra on the same kernel the loop calls, so these
% are exact, cheap, and evaluable no matter what parameters the caller passed.
% They are the rows that survive when everything else is skipped.
% -------------------------------------------------------------------------
XBIG = 1000;                                  % "long time to go"
[~,~,c3big,c4big,npbig] = optimalGains(XBIG*TAU, TAU, 1.0, 0.0);

% "at the beginning of the flight (long time to go) the effective navigation
%  ratio is approximately constant and is approaching 3"      (:2526, Fig 8.14)
B = addRow(B, 'N'' at tgo/T = 1000 -> 3', 3.0, npbig, 0.02, 'kernel', ...
    'Fig. 8.14 / p.179 prose (:2526)', '-', ...
    'closed form, no simulation: N'' -> 3 + 6/x as x -> inf, so 3.006 at x = 1000');

% "We can see that the 'optimal' guidance law is simply augmented
%  proportional navigation with an effective navigation ratio of 3"  (:2435)
% -- i.e. as the lag vanishes (x = tgo/T -> inf) the four gains must collapse
% onto APN with N' = 3:  C3 -> 1.5 and C4 -> 0 like -3/x.
B = addRow(B, 'C3 at tgo/T = 1000 -> 1.5', 1.5, c3big, 0.01, 'kernel', ...
    'p.173 prose (:2435), C3 = 0.5 N''', '-', ...
    'zero-lag limit is APN with N''=3, so C3 = 0.5*3');
B = addRow(B, 'x*C4 at tgo/T = 1000 -> -3', -3.0, XBIG*c4big, 0.02, 'kernel', ...
    'p.173 prose (:2435) + C4 defn (:2540)', '-', ...
    'C4 -> -N''/x, so the lag feedforward vanishes as the lag does');

% "As we get closer to intercept (small time to go), the effective navigation
%  ratio grows considerably."                                 (:2526, Fig 8.14)
% Scored as an ORDERING over the range where the book's own .0001 regularizer
% is inert (x >= 0.5, where it costs 1.4% and falling).  Below that the
% regularizer turns the curve back down -- see the file header -- and asserting
% monotonicity there would be asserting something the book's code does not do.
xg = 0.5:0.01:20;
npg = zeros(size(xg));
for i = 1:numel(xg)
    [~,~,~,~,npg(i)] = optimalGains(xg(i)*TAU, TAU, 1.0, 0.0);
end
nviol = nnz(diff(npg) >= 0);
mono  = (nviol == 0);
B = addRow(B, 'N'' falls monotonically 0.5<=x<=20', 1, double(mono), 0, 'order', ...
    'Fig. 8.14 / p.179 prose (:2526)', '-', ...
    sprintf('N''(0.5) = %.2f, N''(20) = %.2f, ratio %.2f; %d non-decreasing steps of %d', ...
            npg(1), npg(end), npg(1)/npg(end), nviol, numel(npg)-1));

% -------------------------------------------------------------------------
% SIMULATION ROWS
% -------------------------------------------------------------------------

% Fig. 3.13 / p.47 (:751): "a 3-g maneuver causes nearly 12 ft of miss if the
% flight time is only 1 s".  Legitimate for THIS listing's APN = 0 branch by
% the Listing 8.2 == Listing 3.1 equivalence proved in the file header.
if ~isempty(j0) && isBookCase(opts) && t(1) <= 1.0 && t(end) >= 1.0
    ours = readAt(t, R.law(j0).miss_nt_ft, 1.0);
    B = addRow(B, 'PN miss at tF = 1 s [ft]', 12.0, ours, 1.0, 'graph', ...
        'Fig. 3.13 / p.47 prose (:751), via the Ch.3 equivalence', 'ft', ...
        'absolute band: 12 is a value read off a printed graph');
else
    B = addSkip(B, 'PN miss at tF = 1 s [ft]', 12.0, 'graph', ...
        'Fig. 3.13 / p.47 prose (:751), via the Ch.3 equivalence', 'ft', ...
        ['needs APN = 0 in the run, the book''s nominal parameters, and a ' ...
         'sample grid that brackets tF = 1 s']);
end

% Cross-port second opinion: APN = 0 against the ANALYTIC closed form that
% MGS.ZARCHAN.ADJOINTSINGLELAG carries (p.55, Figs 3.16/3.18).  Shares no code
% with this integrator -- it is an inverse Laplace transform -- so it catches a
% mis-placed impulse, a swapped derivative or a botched corrector, none of
% which the 12 ft row alone would.
if opts.crosscheck && ~isempty(j0)
    [cf_resid, cf_tol, cf_why] = crossCheckCh3(R, opts, j0);
    if isnan(cf_resid)
        B = addSkip(B, 'APN=0 vs p.55 closed form', 0.0, 'xport', ...
            'p.55 closed form via MGS.ZARCHAN.ADJOINTSINGLELAG', 'of peak', cf_why);
    else
        B = addRow(B, 'APN=0 vs p.55 closed form', 0.0, cf_resid, cf_tol, ...
            'xport', 'p.55 closed form via MGS.ZARCHAN.ADJOINTSINGLELAG', ...
            'of peak', ['band = max(1%, 8H): TWO realizations, each carrying ' ...
            'its own O(h^2) error against the same analytic truth.  The Ch.3 ' ...
            'port measures 1.88% of peak on its own; every structural break ' ...
            'measured there lands at 100% or more.']);
    end
else
    B = addSkip(B, 'APN=0 vs p.55 closed form', 0.0, 'xport', ...
        'p.55 closed form via MGS.ZARCHAN.ADJOINTSINGLELAG', 'of peak', ...
        'crosscheck disabled, or APN = 0 not in the run');
end

% Fig. 8.12 / p.176 (:2461): "Augmented proportional navigation yields smaller
% miss distances for shorter flight times, whereas proportional navigation
% yields smaller miss distances for longer flight times."
% An ORDERING at the two ends of the sweep.  Where the crossover sits is NOT
% published, so it is reported and not scored.
if ~isempty(j0) && ~isempty(j1) && t(1) <= 1.0 && t(end) >= 1.0
    pn  = abs(R.law(j0).miss_nt_ft);
    apn = abs(R.law(j1).miss_nt_ft);
    short_ok = readAt(t, apn, 1.0) < readAt(t, pn, 1.0);
    long_ok  = apn(end) > pn(end);
    xo = firstCross(t, apn - pn);
    B = addRow(B, 'APN < PN at short tF (1 s)', 1, double(short_ok), 0, 'order', ...
        'Fig. 8.12 / p.176 prose (:2461)', '-', ...
        sprintf('|APN| = %.3f ft vs |PN| = %.3f ft at tF = 1 s', ...
                readAt(t, apn, 1.0), readAt(t, pn, 1.0)));
    B = addRow(B, 'APN > PN at long tF (end)', 1, double(long_ok), 0, 'order', ...
        'Fig. 8.12 / p.176 prose (:2461)', '-', ...
        sprintf('|APN| = %.3f ft vs |PN| = %.3f ft at tF = %.1f s; crossover %s', ...
                apn(end), pn(end), t(end), fmtNaN(xo, 's')));
else
    B = addSkip(B, 'APN < PN at short tF (1 s)', 1, 'order', ...
        'Fig. 8.12 / p.176 prose (:2461)', '-', ...
        'needs both APN = 0 and APN = 1, and a sample grid that brackets tF = 1 s');
    B = addSkip(B, 'APN > PN at long tF (end)', 1, 'order', ...
        'Fig. 8.12 / p.176 prose (:2461)', '-', ...
        'needs both APN = 0 and APN = 1, and a sample grid that brackets tF = 1 s');
end

% Fig. 8.18 / p.183 (:2574): "the optimal guidance law always yields zero miss
% distance."  Zero is not testable at floating point, and the book's own .0001
% regularizer defeats the law for tgo/T below about 0.4 (see the file header),
% so this is scored in TWO pieces:
%   (a) the whole curve, as a fraction of the PN peak -- the comparison the
%       figure itself makes, since Fig. 8.18 plots all three laws on one axis;
%   (b) the GROWTH after tF = 1 s, which the regularizer cannot reach, because
%       "the law nulls the miss" is exactly the statement that the miss curve
%       stops accumulating.  (b) is the sharp one.
if ~isempty(j2) && ~isempty(j0)
    pnpk = max(abs(R.law(j0).miss_nt_ft));
    mopt = R.law(j2).miss_nt_ft;
    if pnpk <= 0
        B = addSkip(B, 'OPTIMAL peak / PN peak', 0.0, 'ratio', ...
            'Fig. 8.18 / p.183 prose (:2574)', 'of PN peak', ...
            'the PN curve is identically zero, so a ratio to it means nothing');
        B = addSkip(B, 'OPTIMAL growth after 1 s / PN peak', 0.0, 'ratio', ...
            'Fig. 8.18 / p.183 prose (:2574)', 'of PN peak', ...
            'the PN curve is identically zero, so a ratio to it means nothing');
    else
        % The detail carries the SPLIT, not just the verdict.  If this row
        % ever runs close to its band the first question is "is the residual
        % all regularizer-window offset, or is the law failing?", and that
        % question is answerable only if the 0.5 s value is printed beside
        % the peak.  Without it a near-miss is an afternoon of re-deriving.
        m_half = readAt(t, mopt, 0.5);
        B = addRow(B, 'OPTIMAL peak / PN peak', 0.0, max(abs(mopt))/pnpk, ...
            0.15, 'ratio', 'Fig. 8.18 / p.183 prose (:2574)', 'of PN peak', ...
            sprintf(['band 15%%: the book''s .0001 regularizer stops the law ' ...
             'working below tgo/T ~ 0.4 and whatever accumulates there rides ' ...
             'along as a constant offset.  The band separates "nulls the ' ...
             'miss" from "does not", which is the claim; it certifies no ' ...
             'digit.  Split: %.4f ft at tF = 0.5 s (end of the regularized ' ...
             'window), peak %.4f ft, PN peak %.4f ft.'], ...
             m_half, max(abs(mopt)), pnpk));
        if t(end) > 1.0 && t(1) <= 1.0
            sel = t >= 1.0;
            m0  = readAt(t, mopt, 1.0);
            grow = max(abs(mopt(sel) - m0));
            B = addRow(B, 'OPTIMAL growth after 1 s / PN peak', 0.0, grow/pnpk, ...
                0.03, 'ratio', 'Fig. 8.18 / p.183 prose (:2574)', 'of PN peak', ...
                ['THE SHARP ONE.  Past tF = 1 s the regularizer is inert ' ...
                 '(0.06% at tgo/T = 1), so a law that nulls the miss adds ' ...
                 'nothing further while PN and APN swing over the full range.']);
        else
            B = addSkip(B, 'OPTIMAL growth after 1 s / PN peak', 0.0, 'ratio', ...
                'Fig. 8.18 / p.183 prose (:2574)', 'of PN peak', ...
                sprintf(['the sampled grid %g..%g s does not bracket the 1 s ' ...
                         'reference point'], t(1), t(end)));
        end
    end
else
    B = addSkip(B, 'OPTIMAL peak / PN peak', 0.0, 'ratio', ...
        'Fig. 8.18 / p.183 prose (:2574)', 'of PN peak', 'needs APN = 0 and APN = 2');
    B = addSkip(B, 'OPTIMAL growth after 1 s / PN peak', 0.0, 'ratio', ...
        'Fig. 8.18 / p.183 prose (:2574)', 'of PN peak', 'needs APN = 0 and APN = 2');
end

% Listing A.1 / Fig. A.1 / p.950 (:14101).  The only two NUMBERS the book
% publishes about optimal guidance's miss performance, and both are printed in
% the running text rather than read off the plot by us.
if isempty(R.err)
    % ttgo = false.  These two rows must still APPEAR, as skips.  A board that
    % silently loses two benchmarks looks exactly like a board that never had
    % them -- the same failure shape MGS.ZARCHAN.BENCHSUITE's manifest guard
    % exists to prevent one level up.
    B = addSkip(B, 'A.1 SF=0.9 crossover [s]',   8.0, 'graph', ...
        'Fig. A.1 / p.950 prose (:14101)', 's', '''ttgo'' is false: the case was not run');
    B = addSkip(B, 'A.1 BIAS=0.1 crossover [s]', 5.5, 'graph', ...
        'Fig. A.1 / p.950 prose (:14101)', 's', '''ttgo'' is false: the case was not run');
else
    for jj = 1:numel(R.err)
        e = R.err(jj);
        nm = sprintf('A.1 %s crossover [s]', ternary(e.bias == 0, 'SF=0.9', 'BIAS=0.1'));
        if isempty(j0) || ~isBookCase(opts) || opts.TF < 10
            B = addSkip(B, nm, e.book_crossover_s, 'graph', ...
                'Fig. A.1 / p.950 prose (:14101)', 's', ...
                'needs APN = 0 in the run, the book''s nominal parameters, and TF >= 10 s');
            continue;
        end
        % Precomputed in the main body; see the comment there for why it is
        % not computed in this function.
        xo = e.crossover_s;
        if isnan(xo)
            % j0 is non-empty here, so a NaN can only mean the degraded law
            % never becomes worse than PN inside the sweep.  The book says it
            % does.  That is a FAIL, not a skip.
            B = addRow(B, nm, e.book_crossover_s, NaN, 1.0, 'graph', ...
                'Fig. A.1 / p.950 prose (:14101)', 's', ...
                'no crossover found: the degraded law never becomes worse than PN here');
        else
            B = addRow(B, nm, e.book_crossover_s, xo, 1.0, 'graph', ...
                'Fig. A.1 / p.950 prose (:14101)', 's', ...
                ['last tF at which |miss| crosses above the PN curve; band +/-1 s ' ...
                 'because the book quotes a flight time read off Fig. A.1']);
        end
    end
end
end


% -----------------------------------------------------------------------------
function [resid, tol, why] = crossCheckCh3(R, opts, j0)
%CROSSCHECKCH3  Compare our APN = 0 maneuver curve against the ANALYTIC closed
%   form carried by MGS.ZARCHAN.ADJOINTSINGLELAG (p.55).
%
%   The sibling ERRORS BY DESIGN when its own benchmarks fail.  If that
%   happens this returns NaN and the caller records a SKIP with the reason --
%   never a pass.  "Could not be evaluated" and "passed" must not look alike.
resid = NaN;  tol = NaN;  why = '';
try
    A = mgs.zarchan.adjointSingleLag('VM', opts.VM, 'HEDEG', opts.HEDEG, ...
        'XNT', opts.XNT, 'TAU', opts.TAU, 'XNP', opts.XNP, 'TF', opts.TF, ...
        'H', opts.H, 'samp', opts.samp, 'render', false);
catch ME
    why = sprintf('MGS.ZARCHAN.ADJOINTSINGLELAG could not run: %s', ...
                  oneLine(ME.message));
    return;
end
if ~any(isfinite(A.cf_nt_ft))
    why = ['the Ch.3 port has no closed form for these parameters ' ...
           '(it needs an integer N'' >= 3), so there is nothing analytic to ' ...
           'compare against'];
    return;
end
% Grids: both start at TP = 1e-5 and advance by the same H, so they coincide.
% Interpolate anyway rather than assume, and refuse if there is no overlap.
tA = A.t(:).';
cf = A.cf_nt_ft(:).';
lo = max(R.t(1), tA(1));  hi = min(R.t(end), tA(end));
sel = R.t >= lo & R.t <= hi;
if nnz(sel) < 3
    why = 'the two ports'' time grids do not overlap; nothing to compare';
    return;
end
cfi = interp1(tA, cf, R.t(sel), 'linear');
scale = max(abs(cfi));
if scale <= 0
    why = 'the closed-form curve is identically zero over the overlap';
    return;
end
% Normalized by the curve's own PEAK, not pointwise: the closed form crosses
% zero near tF = 3T at N' = 4 and a pointwise relative error there divides a
% small number by a smaller one and manufactures a failure out of nothing.
resid = max(abs(R.law(j0).miss_nt_ft(sel) - cfi)) / scale;
tol   = max(0.01, 8.0*opts.H);
end


% =============================================================================
%   BENCH-ROW PLUMBING.  Field names and order match what
%   MGS.ZARCHAN.BENCHSUITE's adapters read off MGS.ZARCHAN.ADJOINTBINOMIAL5
%   (what/book/ours/pct/band/tol/ok/skipped/why), plus source/units/detail.
% =============================================================================
function B = emptyBench()
B = struct('what',{}, 'book',{}, 'ours',{}, 'pct',{}, 'band',{}, 'tol',{}, ...
           'ok',{}, 'skipped',{}, 'why',{}, 'source',{}, 'units',{}, 'detail',{});
end


function B = addRow(B, what, book, ours, tol, band, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).book = book;  B(i).ours = ours;
B(i).band = band;  B(i).tol = tol;    B(i).source = source;
B(i).units = units;  B(i).detail = detail;  B(i).why = '';
B(i).skipped = false;
if ~isfinite(ours)
    B(i).pct = NaN;  B(i).ok = false;  return;
end
switch lower(band)
case {'kernel','graph','xport'}
    % ABSOLUTE band, in the row's own units.  These rows include values near
    % zero and a relative band there is meaningless.
    B(i).ok  = abs(ours - book) <= tol;
    B(i).pct = pctOf(ours, book);
case 'order'
    % Boolean.  book = 1 means "the book asserts this direction holds".
    B(i).ok  = (ours == book);
    B(i).pct = NaN;
case 'ratio'
    % book is 0 (the book says "zero"); ours is already a fraction.
    B(i).ok  = abs(ours - book) <= tol;
    B(i).pct = 100*ours;
otherwise
    error('mgs:zarchan:adjointOptimal:band', ...
        ['unknown tolerance band "%s".  Bands encode the PROVENANCE of the ' ...
         'book value; inventing one silently sets an arbitrary pass criterion.'], band);
end
end


function B = addSkip(B, what, book, band, source, units, why)
i = numel(B) + 1;
B(i).what = what;  B(i).book = book;  B(i).ours = NaN;  B(i).pct = NaN;
B(i).band = band;  B(i).tol = NaN;   B(i).ok = false;  B(i).skipped = true;
B(i).why = why;    B(i).source = source;  B(i).units = units;  B(i).detail = why;
end


function p = pctOf(ours, book)
if book == 0
    p = NaN;
else
    p = 100*(ours/book - 1);
end
end


% =============================================================================
%   SMALL HELPERS
% =============================================================================
function y = readAt(t, v, x)
%READAT  v(x) by linear interpolation.  NEVER extrapolates -- out of horizon
%   returns NaN so the row fails loudly instead of quoting an invented value.
if x < t(1) || x > t(end)
    y = NaN;
    return;
end
y = interp1(t, v, x, 'linear');
end


function tc = firstCross(t, d)
%FIRSTCROSS  First t where d changes sign, by linear interpolation.  NaN if it
%   never does.  Used for REPORTING only -- the book publishes no crossover
%   time for Fig. 8.12, so nothing scores against this.
k = find(d(1:end-1).*d(2:end) < 0, 1, 'first');
if isempty(k)
    tc = NaN;
else
    tc = t(k) + (t(k+1)-t(k)) * (-d(k)) / (d(k+1)-d(k));
end
end


function tc = lastCrossAbove(t, a, b)
%LASTCROSSABOVE  The smallest t0 such that a > b for EVERY t >= t0.
%
%   This is the quantity the book states -- "worse than proportional
%   navigation for flight times greater than 8 s" -- and it is deliberately
%   the LAST crossing, not the first.  |PN| passes through zero near tF = 3T,
%   and any curve is momentarily "worse" than a curve that is instantaneously
%   zero, so a first-crossing definition would report ~3 s for both cases and
%   both book rows would fail for a reason that has nothing to do with the
%   port.
d = a - b;
if d(end) <= 0
    tc = NaN;                 % never ends up worse: no such t0 exists
    return;
end
k = find(d <= 0, 1, 'last');
if isempty(k)
    tc = t(1);                % worse across the whole sweep
    return;
end
tc = t(k) + (t(k+1)-t(k)) * (-d(k)) / (d(k+1)-d(k));
end


function tf = isBookCase(opts)
%ISBOOKCASE  True only for Listing 8.2's own data statement (:2562).  The
%   published figures describe THIS engagement; scoring a graph-read number
%   against different parameters is a false red light and sometimes a false
%   green one.
tf = (opts.XNT == 96.6) && (opts.XNP == 4.0) && (opts.TAU == 1.0) && ...
     (opts.VM == 3000.0) && (opts.HEDEG == -20.0) && ...
     (opts.SF == 1.0) && (opts.BIAS == 0.0);
end


function s = fmtNaN(v, unit)
if isnan(v)
    s = 'none found';
else
    s = sprintf('%.2f %s', v, unit);
end
end


function v = ternary(c, a, b)
if c, v = a; else, v = b; end
end


function s = oneLine(msg)
s = regexprep(msg, '\s+', ' ');
if numel(s) > 160, s = [s(1:157) '...']; end
end


function s = tf2s(b)
if b, s = 'PASS'; else, s = 'FAIL'; end
end


% =============================================================================
function s = render(R, opts)
W = 100;
L = {};
L{end+1} = 'Zarchan Listing 8.2 -- ADJOINT OF THE OPTIMAL SINGLE-LAG GUIDANCE SYSTEM';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf('TAU = %g s   TF = %g s   H = %g s   XNT = %g ft/s^2 (%.2f g)   VM = %g ft/s   HE = %g deg', ...
    opts.TAU, opts.TF, opts.H, opts.XNT, opts.XNT/32.2, opts.VM, opts.HEDEG);
L{end+1} = sprintf('laws: %s      XNP = %g  (APN 0/1 ONLY -- the optimal law computes its own N'')', ...
    strjoin(arrayfun(@(k) sprintf('%g=%s', R.law(k).apn, R.law(k).name), ...
                     1:numel(R.law), 'UniformOutput', false), '  '), opts.XNP);
L{end+1} = sprintf('heading-error scale -VM*HE = %+.3f ft/s   (NOT 2*VM*HE -- see ADJOINTSINGLELAG)', ...
    R.he_scale);
if opts.SF ~= 1 || opts.BIAS ~= 0
    L{end+1} = sprintf(['*** time-to-go ERROR ACTIVE: SF = %g, BIAS = %g s.  This is Listing A.1, ' ...
        'not Listing 8.2, and the Fig. 8.18 rows do not describe it. ***'], opts.SF, opts.BIAS);
end
L{end+1} = 'integrator: Zarchan''s two-pass predictor/corrector, transcribed';
L{end+1} = repmat('-', 1, W);

% --- structural self-check ---
if isfield(R.selfcheck, 'ok')
    C = R.selfcheck;
    L{end+1} = 'STRUCTURAL SELF-CHECK  (wiring, not the book -- a failure here voids everything)';
    L{end+1} = sprintf('  open loop XNP=0 vs analytic t^2/2 (Heun is exact here) : rel %9.2e  tol %.0e  %s', ...
        C.openloop_rel, C.openloop_tol, tf2s(C.openloop_ok));
    if C.c3_evaluated
        L{end+1} = sprintf('  C3 confined to x1'' -> APN 0 and 1 share x2,x3,x4     : rel %9.2e  tol %.0e  %s', ...
            C.c3_rel, C.c3_tol, tf2s(C.c3_ok));
    else
        L{end+1} = '  C3 confined to x1''                                    : not evaluable at XNP = 0';
    end
    L{end+1} = sprintf('  Listing A.1 extension inert at SF=1, BIAS=0           : rel %9.2e  tol %.0e  %s', ...
        C.a1_rel, C.a1_tol, tf2s(C.a1_ok));
else
    L{end+1} = 'STRUCTURAL SELF-CHECK  ***SKIPPED*** (selfcheck=false) -- nothing here is verified';
end
L{end+1} = repmat('-', 1, W);

% --- benchmark table ---
L{end+1} = 'BOOK BENCHMARKS';
L{end+1} = sprintf('  %-34s %10s %10s %9s  %-7s %s', ...
    'what', 'book', 'ours', 'units', 'band', 'verdict');
for i = 1:numel(R.bench)
    b = R.bench(i);
    if b.skipped
        L{end+1} = sprintf('  %-34s %10.4g %10s %9s  %-7s SKIP (%s)', ...
            b.what, b.book, '--', b.units, b.band, b.why); %#ok<AGROW>
    else
        L{end+1} = sprintf('  %-34s %10.4g %10.4f %9s  %-7s %s', ...
            b.what, b.book, b.ours, b.units, b.band, tf2s(b.ok)); %#ok<AGROW>
    end
end
L{end+1} = '  bands: kernel = closed-form limit of the gain kernel, absolute;';
L{end+1} = '         graph  = a value the author read off a figure and quoted, absolute;';
L{end+1} = '         order  = the book states a direction, not a number: boolean;';
L{end+1} = '         ratio  = fraction of the PN peak, because the book''s value is "zero";';
L{end+1} = '         xport  = against an analytic second opinion in another port.';
L{end+1} = repmat('-', 1, W);

% --- Fig. 8.14 kernel ---
L{end+1} = 'OPTIMAL EFFECTIVE NAVIGATION RATIO (Fig. 8.14)   x = tgo/T';
L{end+1} = sprintf('    %8s %12s %12s %10s', 'x', 'N'' (book)', 'N'' (exact)', 'guard');
for i = 1:numel(R.nav.x)
    if R.nav.np_exact(i) ~= 0
        g = sprintf('%8.2f%%', 100*(R.nav.np(i)/R.nav.np_exact(i) - 1));
    else
        g = '      --';
    end
    L{end+1} = sprintf('    %8.4g %12.4f %12.4f %10s', ...
        R.nav.x(i), R.nav.np(i), R.nav.np_exact(i), g); %#ok<AGROW>
end
L{end+1} = '  "N'' (book)" is what the loop flies -- TOP/(.0001+BOT).  "exact" drops the .0001.';
L{end+1} = '  The .0001 is a REGULARIZER: the exact denominator''s first five Taylor terms';
L{end+1} = '  cancel, so N'' would diverge as 10/x at intercept.  It costs 1.4% at x = 0.5 and';
L{end+1} = '  0.06% at x = 1, but it takes over completely below x ~ 0.4.  Do not remove it:';
L{end+1} = '  it is the book''s code and the published figure is what it produces.';
L{end+1} = repmat('-', 1, W);

% --- miss curves ---
L{end+1} = 'MISS SENSITIVITY TO A STEP TARGET MANEUVER (Fig. 8.18)   [ft]';
hdr = sprintf('    %7s', 'tF [s]');
for k = 1:numel(R.law)
    hdr = [hdr sprintf('  %11s', R.law(k).name)]; %#ok<AGROW>
end
for jj = 1:numel(R.err)
    hdr = [hdr sprintf('  %11s', ternary(R.err(jj).bias == 0, 'OPT SF.9', 'OPT BIAS.1'))]; %#ok<AGROW>
end
hdr = [hdr sprintf('  %11s', 'MHE(law1)')];
L{end+1} = hdr;
for x = 1:floor(R.t(end))
    row = sprintf('    %7.1f', x);
    for k = 1:numel(R.law)
        row = [row sprintf('  %11.4f', readAt(R.t, R.law(k).miss_nt_ft, x))]; %#ok<AGROW>
    end
    for jj = 1:numel(R.err)
        row = [row sprintf('  %11.4f', readAt(R.t, R.err(jj).miss_nt_ft, x))]; %#ok<AGROW>
    end
    row = [row sprintf('  %11.4f', readAt(R.t, R.law(1).miss_he_ft, x))];
    L{end+1} = row; %#ok<AGROW>
end
L{end+1} = '  MHE column is the HEADING-ERROR channel of the first law listed.  The book';
L{end+1} = '  publishes no value for it here, so it is reported and NOT scored -- but it is';
L{end+1} = '  the channel that maps onto our measured discriminant, release-side |eps0|.';
L{end+1} = repmat('-', 1, W);

% --- time to go errors ---
if ~isempty(R.err)
    L{end+1} = 'TIME-TO-GO ERROR SENSITIVITY (Listing A.1 / Fig. A.1, p.950)';
    for jj = 1:numel(R.err)
        L{end+1} = sprintf('  %-32s crossover with PN: ours %s   book %.1f s', ...
            R.err(jj).name, fmtNaN(R.err(jj).crossover_s, 's'), ...
            R.err(jj).book_crossover_s); %#ok<AGROW>
    end
    L{end+1} = '  Past that flight time the optimal law is WORSE than plain proportional';
    L{end+1} = '  navigation.  A 10% scale error on tgo is enough.  Read the next block.';
    L{end+1} = repmat('-', 1, W);
end

L{end+1} = 'READ THIS BEFORE QUOTING ANY NUMBER ABOVE';
L{end+1} = '  1. This law needs TIME TO GO and it is brittle in it: a 0.9 scale factor loses';
L{end+1} = '     to plain PN beyond 8 s of flight, a 0.1 s bias beyond 5.5 s (p.950).  Our own';
L{end+1} = '     dive predictor under-estimates dive duration by a median +18.7 s, one-sided';
L{end+1} = '     positive on 27 of 27 samples, against 35-47 s dives -- a scale error of about';
L{end+1} = '     1.4-1.5.  On this evidence the law would be worse than what we already fly.';
L{end+1} = '  2. The lag here is LINEAR and never saturates.  Ours saturates on 26 of 32';
L{end+1} = '     sorties.  Cancelling a linear lag is not cancelling a saturated one.';
L{end+1} = '  3. The disturbances are a STEP TARGET MANEUVER and a HEADING ERROR.  Our';
L{end+1} = '     bullseye is stationary.  A miss distance computed here is not a prediction';
L{end+1} = '     about a sortie and must not be reported as if it were.';
if ~isBookCase(opts)
    L{end+1} = '  4. PARAMETERS ARE NOT THE BOOK''S NOMINAL SET, so every figure-derived row';
    L{end+1} = '     above is skipped.  Only the gain-kernel rows and the structural checks';
    L{end+1} = '     still have any force.';
end
s = strjoin(L, newline);
end


% =============================================================================
function drawFigures(R, opts)
%DRAWFIGURES  Base MATLAB only -- no toolbox anywhere in this package.
figure('Name', 'Zarchan Fig. 8.18 -- miss due to step target maneuver');
hold on; grid on;
names = {};
for k = 1:numel(R.law)
    plot(R.t, R.law(k).miss_nt_ft, 'LineWidth', 1.3);
    names{end+1} = R.law(k).name; %#ok<AGROW>
end
for jj = 1:numel(R.err)
    plot(R.t, R.err(jj).miss_nt_ft, '--', 'LineWidth', 1.1);
    names{end+1} = R.err(jj).name; %#ok<AGROW>
end
hold off;
xlabel('Flight Time (Sec)');                       % the listing's own labels
ylabel('Target Maneuver Miss (Ft)');
title(sprintf('Single-lag adjoint, TAU = %g s, %.1f g target', ...
      opts.TAU, opts.XNT/32.2));
legend(names, 'Location', 'best');

figure('Name', 'Zarchan Fig. 8.14 -- optimal effective navigation ratio');
xx = logspace(-1, 1.5, 400);
np = zeros(size(xx));
for i = 1:numel(xx)
    [~,~,~,~,np(i)] = optimalGains(xx(i)*opts.TAU, opts.TAU, 1.0, 0.0);
end
semilogx(xx, np, 'LineWidth', 1.3); hold on;
semilogx(xx, optimalNavExact(xx), '--', 'LineWidth', 1.0);
semilogx([xx(1) xx(end)], [3 3], ':', 'LineWidth', 1.0); hold off;
grid on;
xlabel('Normalized time to go  t_{go}/T');
ylabel('Effective navigation ratio  N''');
legend({'book (.0001 regularizer)', 'exact', 'asymptote N'' = 3'}, 'Location', 'best');
title('N'' approaches 3 at long time to go and grows as intercept nears');
end
