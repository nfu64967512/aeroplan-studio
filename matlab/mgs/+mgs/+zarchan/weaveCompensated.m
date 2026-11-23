function R = weaveCompensated(opts)
%MGS.ZARCHAN.WEAVECOMPENSATED  Zarchan Listing 20.4 -- brute-force evaluation of
%   THREE guidance laws against a WEAVING (sinusoidal) target.
%
%   R = mgs.zarchan.weaveCompensated()
%   R = mgs.zarchan.weaveCompensated('WH', 1.6)        law mis-estimates omega
%   R = mgs.zarchan.weaveCompensated('XNCLIM', 322)    10 g acceleration limit
%   R = mgs.zarchan.weaveCompensated('assert', false, 'render', false)
%
%   Port of Listing 20.4 (zarchan.txt:6377-6399, pp.468-469): one forward run
%   per flight time, tF = 0.1:0.1:10 s, of a LINEARISED planar homing loop
%   whose target accelerates as nT*sin(w*t) from t = 0.  The missile carries a
%   FIFTH-ORDER BINOMIAL guidance lag -- five first-order stages of TAU/5, two
%   on the line-of-sight path and three on the command path.  Three laws share
%   that loop, selected by the listing's own APN switch:
%
%     APN = 1  PROPORTIONAL NAVIGATION      nc = N'*Vc*lamdot_hat
%     APN = 2  WEAVE guidance (p.465)       PN plus a target-ACCELERATION term
%              and a target-JERK term, weighted by kernels in w*tgo.  Optimal
%              against a sinusoid for a LAG-FREE missile.
%     APN = 3  COMPENSATED WEAVE (p.466)    the same three terms with N' made
%              time-varying, plus a fourth that subtracts the acceleration
%              already committed to the airframe, nL*T^2*(e^-x + x - 1)/tgo^2.
%              Optimal for a SINGLE-lag missile -- so it is SUBOPTIMAL here,
%              which the book says explicitly at zarchan.txt:6374.
%
%   ⭐ WHY THIS ONE MATTERS HERE.  It is the only place in the book where a law
%   is DESIGNED against a sinusoid, and this project is building a weave
%   manoeuvre.  Read the three curves as a price list: what PN pays, what
%   knowing the target's acceleration and jerk buys, and what additionally
%   knowing your own lag buys.
%
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, g = 32.2 ft/s^2.  NOT SI.  Angles
%   are radians, w and WH rad/s.  Nothing is converted -- a silent ft->m would
%   turn 20.1 ft of miss into "6.13" and it would still read as a plausible
%   miss.  Convert AT THE CALL SITE and say so there.
%
%   *** WHAT IT IS NOT. ***  Linearised planar geometry, constant closing
%   velocity, no gravity, drag, thrust or seeker noise, and a target whose
%   weave starts at t = 0 with KNOWN amplitude, phase and frequency.  The only
%   concession to ignorance is WH, the frequency the law ASSUMES, which this
%   port separates from the true W so the two can be made to disagree.  The
%   book always sets them equal.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_20_04.m does not parse.  Five repairs:
%
%     1  "while T , =(TF-.0001)" -> "<=".  The OCR renders "<" as ",".  T only
%        increases, so ">=" never terminates.
%     2  "while STEP , =1" -> "<=".  Same substitution; the body sets STEP=2 to
%        leave, which only makes sense against "<=".
%     3  "if XNC . XNCLIM" -> "if XNC > XNCLIM".  THIS IS THE ONE THE
%        TRANSCRIPTION FLAGS AS UNRESOLVED (variable vs variable).  Settled by
%        the NEXT line, "if XNC , -XNCLIM" -> "if XNC < -XNCLIM": the two are a
%        symmetric limiter, so "." must be ">".  Resolving it the other way
%        pins XNC to +XNCLIM whenever the command is small -- a constant
%        hard-over.  STRUCTURAL row 2 is what would catch that.
%     4  "...*(1.cos(XP))/XP^2...." -> "(1.-cos(XP))" and a "..." continuation.
%        Forced twice over: the APN==2 branch four lines earlier prints the same
%        kernel correctly, and p.466 prints it as (1-cos(w*tgo))/w^2.
%     5  The paragraph beginning "Although weave guidance has small miss
%        distances" trailing the transcription is the book's next PROSE block
%        (zarchan.txt:6405), not code.  Dropped, not "fixed" -- it is the
%        sentence supplying BENCHMARK 6, which is how it was spotted.
%
%   Reproduced deliberately: the dead "FLAG=0" after the inner while (recorded,
%   not coded); TGO's +1e-5 alongside the loop test's -1e-4 (different
%   constants doing different jobs -- keeping XLAM finite on the last
%   evaluation, and a margin against drift in T); and no file is written.
%
%   ⚠️ THE 0.0001 IN THE DENOMINATOR OF XNPP IS NOT COSMETIC.  The listing
%   computes the time-varying navigation ratio as TOP/(.0001+BOT1+BOT2), and
%   BOT1+BOT2 -> (3/10)x^5 as x -> 0.  The guard therefore DOMINATES below
%   x ~ 0.5, so the gain the listing actually flies TURNS OVER and collapses
%   toward zero near intercept while Fig. 20.17 shows the formula growing --
%   opposite directions.  Both are returned (R.nprime.flown, R.nprime.book) and
%   an INFO row names the crossover.  BENCHMARKS 2/3 score the PRINTED formula,
%   because that is what the figure plots.
%
%   VECTORISED ACROSS FLIGHT TIME, and that is exact: the book's 100 runs all
%   start at T = 0 and advance by the same H, so T is common and the runs are
%   an array index, each element frozen once T > tF_i - 1e-4.  Freezing uses
%   INDEXED ASSIGNMENT, not a mask -- a frozen element's tgo goes negative,
%   exp(-x) overflows to Inf, and Inf*0 is NaN.  STRUCTURAL row 1 runs the
%   listing verbatim and scalar for all three laws and requires a match.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  ⚠️ CHAPTER 20 PUBLISHES ALMOST NO NUMBERS FOR THIS LISTING.
%   Figs. 20.17/20.18/20.19 became image links in the extraction, so their axes
%   are gone; what survives is PROSE ABOUT them, and prose about a figure is an
%   ordering, not a value.  Most of the teeth are therefore in shape rows, and
%   that is the correct outcome for this chapter, not a shortfall.  A SKIP row
%   says so on the board.  Every book value below was grepped from zarchan.txt
%   and its line is in the row's .source.
%
%     1 value  nT = 193.2 ft/s^2 IS 6 g (":6375" calls the target 6-g).
%              Skips off-book: a row whose "book value" is really the caller's
%              parameter passes by construction and adjudicates nothing.
%     2 value  N' -> 3 at long time-to-go, ":6373" -- "approximately constant
%              and is approaching 3".  Evaluated at x = 1000; the approach is
%              from ABOVE, so a band and not an equality.
%     3 shape  N' GROWS as tgo shrinks (same sentence).  Strict monotone
%              decrease of the PRINTED formula over x in [0.5, 20].  The lower
%              cut is 0.5 for two independent reasons: below it the 1e-4 guard
%              has taken over, and the printed denominator is a difference of
%              terms of order 3 cancelling to order x^5, so it loses its last
%              digit near x ~ 0.002 -- silently, as a plausible small number.
%     4 value  the WEAVE law at ZERO FREQUENCY is augmented PN with N' = 3 plus
%              a jerk term (":6341-6346"): (1-cos u)/u^2 -> 1/2 and
%              (u-sin u)/(u^2 w) -> tgo/6, so nc -> 3*Vc*lamdot + 1.5*ydd_T +
%              0.5*tgo*yddd_T.  The port's own command assembly is evaluated at
%              w = 1e-3, chosen because the residual there is pure Taylor
%              truncation (~1e-6) and (1-cos u) has not yet cancelled.
%     5 shape  Fig. 20.18 (":6375"): uncompensated weave guidance
%              "substantially reduce[s] the miss" vs PN.  Scored as
%              peak|miss|_PN >= 2 * peak|miss|_weave.  The factor 2 is this
%              port's reading of an adverb and is stated on the row.
%     6 shape  Fig. 20.19 (":6405"): compensating for the lag "reduces the miss
%              distance even more".  Scored as an ORDERING; no magnitude is
%              published anywhere in this section.
%     7 shape  the PN miss vs flight time is a SINUSOID AT THE TARGET WEAVE
%              FREQUENCY.  ":5959" -- "the miss is sinusoidal in nature with
%              frequency 3 rad/s, which is identical to the target weave
%              frequency".  ⚠️ THAT SENTENCE IS ABOUT LISTING 20.1 (w = 3,
%              TAU = 1, ONE lag) and is applied here at w = 2 with five.  The
%              row says so.  Transferred rather than dropped because it is by a
%              wide margin the sharpest check in this chapter -- a
%              least-squares sine fit recovers w to four decimals -- and it is
%              the only row that would notice W and WH swapped, or the target
%              model driven by the wrong clock.
%
%   NOT SCORED (INFO/SKIP):  the figures' axis values (image links -- SKIP, with
%   the arithmetic left undone rather than a band widened to absorb it); how
%   large "substantial" is (an adjective, reported beside it); the fifth-order
%   binomial claim (a chain this file builds and a closed form this file writes
%   agree by construction -- it is adjudicated by STRUCTURAL row 3, which
%   drives the listing's OWN lag lines); and the SENSITIVITY TO THE FREQUENCY
%   ESTIMATE, which the book never explores and so publishes nothing to score.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED. ***  MATLAB is the
%   caller's; this file has never been executed here.  The numbers come from an
%   independent replica of the same transcription run outside MATLAB, so a
%   first run can be compared against an expectation instead of merely admired.
%   No runtime and no pass count is claimed.
%
%     peak |miss| over tF = 0.1..10 s, at the listing's own settings
%        APN 1  PN                  20.128 ft  at tF = 2.3 s
%        APN 2  weave                6.337 ft  at tF = 0.9 s
%        APN 3  compensated weave    0.894 ft  at tF = 0.6 s
%        ratios: PN/weave = 3.18,  weave/compensated = 7.09
%     settled PN miss over tF in [3,10] s: amplitude 20.123 ft, fitted
%        w = 2.0000 rad/s, R^2 = 1.000000, 71 points.  Insensitive to the
%        window: [4,10] and [5,10] give the same w and amplitude.
%     N'(1000) = 3.0060 (+0.20% on the book's 3);  N'(0.5)/N'(20) = 6.700
%     the flown gain departs from the printed formula by >1% below x = 0.541,
%        i.e. tgo < 0.135 s at TAU = 0.25 s (the INFO row)
%     structural residuals: row 1 exactly 0; row 2 rel 3.65e-5 (band 4.0e-4);
%        row 3 rel 2.2e-6 (band 1e-4); row 4 7.3e-7 (band 1e-5)
%     WH/W sensitivity, peak compensated-weave miss.  The DEFAULT 'wh_ratio'
%        computes only the two outer points; the middle three are recorded here
%        so the shape of the curve is on file:
%        0.8 -> 1.008 ft   0.9 -> 0.933   1.0 -> 0.894   1.1 -> 0.875   1.2 -> 0.854
%        A 20% frequency error costs about 13% of miss, and OVER-estimating is
%        the cheaper error.  FLAT either way.  That is the most
%        decision-relevant number in the file and it is unscored, because the
%        book publishes nothing to score it against.
%
%   If a residual comes back near a band EDGE that is not a pass to be
%   accepted.  Do not widen a band to make a row green.
%
% ---------------------------------------------------------------------------
%   OUTPUTS
%     R.tf        flight times [s]; R.miss  3 x n miss [ft], rows PN / weave /
%                 compensated weave, also as R.miss_pn/_weave/_comp.  Sign is
%                 the listing's: + means the target was above at intercept.
%     R.peak      .pn .weave .comp [ft] and .tf_* where they occur
%     R.nprime    .x .book (printed) .flown (with the guard) .depart_x
%     R.fit       .omega .r2 .amp .lo .hi .n  -- [] if not evaluable
%     R.wh_scan   .ratio .wh .peak_ft -- [] if 'wh_ratio' is empty
%     R.bench     benchmark rows, schema shared with the other ports
%     R.selfcheck .names .oks .detail .all_ok .first_bad -- STRUCTURAL, never
%                 evidence that this port reproduces Zarchan
%     R.pass      true only if at least one row was SCORED and all passed
%     R.verdict   '' when everything passed, else the one-line reason.  ALWAYS
%                 FORMED.  'assert' chooses the reaction and nothing else: true
%                 errors, false WARNS and still returns R complete.  benchSuite
%                 runs every port with assert off precisely so that one red row
%                 cannot hide the board.
%     R.units     one-line statement of the units;  R.p  parameters used
%     R.summary   the rendered text block
%
%   NO RANDOM NUMBERS ARE DRAWN anywhere in this file, so there is no seed to
%   record and no dependence on the caller's random stream.
%
%   See also MGS.ZARCHAN.WEAVEDESIGNCURVE (Listing 20.3, the normalised design
%   curves these laws are measured against), MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 20.4 line 1, verbatim.  Feet, seconds, radians.
    opts.XNT    (1,1) double {mustBeFinite, mustBePositive}    = 193.2      % ft/s^2 = 6 g
    opts.XNP    (1,1) double {mustBeFinite, mustBePositive}    = 3.0        % N' for APN 1 and 2
    opts.XNCLIM (1,1) double {mustBeFinite, mustBeNonnegative} = 99999999.0 % ft/s^2
    opts.TAU    (1,1) double {mustBeFinite, mustBePositive}    = 0.25       % s, TOTAL guidance lag
    opts.W      (1,1) double {mustBeFinite, mustBePositive}    = 2.0        % rad/s, the TRUE weave
    % WH is the frequency the GUIDANCE LAW assumes.  The book only ever runs
    % WH == W.  It is separated here because "how well must we know it" is the
    % question this chapter raises and never answers; see the INFO row.
    opts.WH     (1,1) double {mustBeFinite, mustBePositive}    = 2.0        % rad/s
    opts.VC     (1,1) double {mustBeFinite, mustBePositive}    = 4000.0     % ft/s
    opts.H      (1,1) double {mustBeFinite, mustBePositive}    = 0.01       % s, RK2 step
    % [] means the listing's own sweep, tF = 0.1:0.1:10 s.
    opts.tf       double {mustBeReal, mustBeFinite} = []
    % WH/W ratios for the unscored sensitivity row.  [] switches it off.  Two
    % points by default: each one is a full extra sweep.
    opts.wh_ratio double {mustBeReal, mustBePositive} = [0.8 1.2]
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so an "if nargin == 0" here would be a mode that never fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

XNT = opts.XNT;  XNP = opts.XNP;  XNCLIM = opts.XNCLIM;  TAU = opts.TAU;
W   = opts.W;    WH  = opts.WH;   VC     = opts.VC;      H   = opts.H;

tf = opts.tf(:).';
if isempty(tf)
    tf = 0.1:0.1:10;                 % the listing's own outer loop
end
tf = sort(tf);
if any(tf <= 0)
    error('mgs:zarchan:weaveCompensated:tf', ...
        'every flight time must be positive; got a minimum of %g s.', min(tf));
end
if min(tf) < 2*H
    error('mgs:zarchan:weaveCompensated:short', ...
        ['the shortest flight time, %g s, is not long enough for two steps ' ...
         'at H = %g s.  A one-step run returns its initial condition, which ' ...
         'is a perfectly smooth zero miss.'], min(tf), H);
end
if H > TAU/10
    error('mgs:zarchan:weaveCompensated:stiff', ...
        ['H = %g s is coarser than a tenth of the guidance time constant ' ...
         'TAU = %g s.  Each of the FIVE lags in this loop carries TAU/5 = ' ...
         '%g s, so this would integrate the lag chain at fewer than two ' ...
         'samples per lag.  RK2 does not blow up there -- it returns a ' ...
         'smooth, entirely wrong miss curve -- so this is refused rather ' ...
         'than warned about.'], H, TAU, TAU/5);
end

% =========================================================================
%   LISTING 20.4.  Three laws over the whole flight-time sweep.
% =========================================================================
miss = zeros(3, numel(tf));
for law = 1:3
    miss(law,:) = sweepLaw(law, tf, XNT, XNP, XNCLIM, TAU, W, WH, VC, H, NaN);
end

R = struct();
R.tf         = tf;
R.miss       = miss;
R.miss_pn    = miss(1,:);
R.miss_weave = miss(2,:);
R.miss_comp  = miss(3,:);

pk = struct();
[pk.pn,    pk.tf_pn]    = peakOf(miss(1,:), tf);
[pk.weave, pk.tf_weave] = peakOf(miss(2,:), tf);
[pk.comp,  pk.tf_comp]  = peakOf(miss(3,:), tf);
R.peak = pk;

% ---- the time-varying navigation ratio, both versions of it ---------------
xg = logspace(log10(0.05), log10(1000), 600);
np = struct();
np.x     = xg;
np.book  = nppBook(xg);            % the formula Fig. 20.17 plots
np.flown = nppListing(xg);         % the gain the listing actually applies
np.depart_x = departureX(xg, np.book, np.flown, 0.01);
R.nprime = np;

% ---- BENCHMARK 7's sine fit, over the settled part of the sweep -----------
R.fit = sineFitWindow(tf, miss(1,:), W);

% ---- the unscored frequency-estimate sensitivity --------------------------
R.wh_scan = [];
wr = opts.wh_ratio(:).';
if ~isempty(wr)
    ws = struct('ratio', wr, 'wh', WH*wr, 'peak_ft', zeros(1,numel(wr)));
    for i = 1:numel(wr)
        y = sweepLaw(3, tf, XNT, XNP, XNCLIM, TAU, W, WH*wr(i), VC, H, NaN);
        ws.peak_ft(i) = max(abs(y));
    end
    R.wh_scan = ws;
end

R.units = 'feet, ft/s, ft/s^2 (g = 32.2 ft/s^2); time s; W and WH rad/s';
R.p = struct('XNT',XNT, 'XNP',XNP, 'XNCLIM',XNCLIM, 'TAU',TAU, 'W',W, ...
             'WH',WH, 'VC',VC, 'H',H, 'tf',tf, 'wh_ratio',wr, ...
             'units',R.units);

R.selfcheck = structuralChecks(tf, miss, XNT, XNP, XNCLIM, TAU, W, WH, VC, H, np);

if opts.benchmark
    R.bench = benchmarks(R, XNT, TAU, W);
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
    msg = 'not one book value could be evaluated; an unvalidated port must not be handed back silently.';
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question.  ' ...
        'First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
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
        error('mgs:zarchan:weaveCompensated:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN.  R comes back complete so the caller can see WHICH
        % side of a comparison moved; a port that errors here hands back
        % nothing and cannot be diagnosed.
        warning('mgs:zarchan:weaveCompensated:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function Y = sweepLaw(APN, tf, XNT, XNP, XNCLIM, TAU, W, WH, VC, H, ncHold)
%SWEEPLAW  Listing 20.4's whole outer loop, vectorised across flight time.
%
%   Every constant arrives as an ARGUMENT.  MATLAB local functions cannot see
%   the main function's workspace; a previous wave in this package read a
%   constant from the caller and threw on the first line of the first run,
%   which static checking could not see beforehand.
%
%   ncHold is NaN in every book case.  When finite it BYPASSES the guidance law
%   and drives the lag chain with a constant command -- SELFCHECK 4 uses it to
%   put the listing's own X4/X5/XNL lines against a closed form.
n   = numel(tf);
Y   = zeros(1,n);  YD  = zeros(1,n);  XNL = zeros(1,n);
D   = zeros(1,n);  EL  = zeros(1,n);  X4  = zeros(1,n);  X5 = zeros(1,n);
% Derivatives initialised before the loop so the phase-2 branch cannot read a
% value left over from anything else.
YDD = zeros(1,n);  XNLD = zeros(1,n);  ELD = zeros(1,n);
DD  = zeros(1,n);  X4D  = zeros(1,n);  X5D = zeros(1,n);

T   = 0.0;
lim = tf - 0.0001;      % the listing's own loop bound, formed the same way
held = isfinite(ncHold);

while true
    a = find(T <= lim);          % INDICES, not a mask; see the header
    if isempty(a), break; end

    YO = Y(a); YDO = YD(a); XNLO = XNL(a); DO = D(a);
    ELO = EL(a); X4O = X4(a); X5O = X5(a);

    % Zarchan's two-pass predictor.  Pass 1 evaluates the derivatives where we
    % are; pass 2 takes a full Euler step, advances T by H exactly once, and
    % re-evaluates there.  Moving the T advance out of pass 2 degrades the
    % scheme to first order while still running smoothly.
    for phase = 1:2
        if phase == 2
            Y(a)   = Y(a)   + H*YD(a);
            YD(a)  = YD(a)  + H*YDD(a);
            XNL(a) = XNL(a) + H*XNLD(a);
            EL(a)  = EL(a)  + H*ELD(a);
            D(a)   = D(a)   + H*DD(a);
            X4(a)  = X4(a)  + H*X4D(a);
            X5(a)  = X5(a)  + H*X5D(a);
            T      = T + H;
        end

        YTDD  = XNT*sin(W*T);            % target acceleration
        YTDDD = W*XNT*cos(W*T);          % target JERK -- the law's third term
        TGO   = tf(a) - T + 0.00001;
        XLAM  = Y(a)./(VC*TGO);

        % Two lags on the line-of-sight path: D is filtered lambda, its
        % derivative DD is the filtered LOS RATE, and EL lags that once more.
        DD(a)  = 5.*(XLAM - D(a))/TAU;
        ELD(a) = 5.*(DD(a) - EL(a))/TAU;

        if held
            XNC = ncHold*ones(1,numel(a));
        else
            XNC = guidanceCmd(APN, EL(a), XNL(a), YTDD, YTDDD, TGO, ...
                              XNP, VC, TAU, WH);
        end
        XNC(XNC >  XNCLIM) =  XNCLIM;
        XNC(XNC < -XNCLIM) = -XNCLIM;

        % Three more lags on the command path.  Five in all, each TAU/5.
        X4D(a)  = 5.*(XNC   - X4(a))/TAU;
        X5D(a)  = 5.*(X4(a) - X5(a))/TAU;
        XNLD(a) = 5.*(X5(a) - XNL(a))/TAU;
        YDD(a)  = YTDD - XNL(a);
    end

    % Heun corrector, in the listing's own algebra.
    Y(a)   = .5*(YO   + Y(a)   + H*YD(a));
    YD(a)  = .5*(YDO  + YD(a)  + H*YDD(a));
    XNL(a) = .5*(XNLO + XNL(a) + H*XNLD(a));
    D(a)   = .5*(DO   + D(a)   + H*DD(a));
    EL(a)  = .5*(ELO  + EL(a)  + H*ELD(a));
    X4(a)  = .5*(X4O  + X4(a)  + H*X4D(a));
    X5(a)  = .5*(X5O  + X5(a)  + H*X5D(a));
end
end


% =============================================================================
function XNC = guidanceCmd(APN, EL, XNL, YTDD, YTDDD, TGO, XNP, VC, TAU, WH)
%GUIDANCECMD  The three laws, transcribed with the LISTING'S OWN OPERATOR
%   ASSOCIATION so that the scalar reference in scalarRun can match it to the
%   last bit.  EL, XNL and TGO are row vectors; YTDD and YTDDD are scalars.
%
%   BENCHMARK 4 calls this directly at w -> 0.  That is the only reason it is a
%   function rather than three lines inside the sweep: a limit you can evaluate
%   is a limit you can score.
switch APN
case 1                                          % proportional navigation
    XNC = XNP*VC*EL;
case 2                                          % weave guidance, p.465
    XP  = WH*TGO;
    XNC = XNP*VC*EL + XNP*YTDD*(1.-cos(XP))./XP.^2 + ...
          XNP*YTDDD*(XP-sin(XP))./(XP.*XP*WH);
otherwise                                       % compensated weave, p.466
    X    = TGO/TAU;
    XP   = WH*TGO;
    XNPP = nppListing(X);
    XNC  = XNPP*VC.*EL + XNPP*YTDD.*(1.-cos(XP))./XP.^2 + ...
           XNPP*YTDDD.*(XP-sin(XP))./(XP.*XP*WH) - ...
           XNPP.*XNL*TAU*TAU.*(exp(-X)+X-1.)./TGO.^2;
end
end


% =============================================================================
function [Y, XNLf] = scalarRun(APN, TF, XNT, XNP, XNCLIM, TAU, W, WH, VC, H, ncHold)
%SCALARRUN  Listing 20.4 VERBATIM, scalar, one flight time.  Nothing here is
%   shared with sweepLaw or guidanceCmd -- it is typed out again from
%   zarchan.txt:6377-6399 on purpose, so that SELFCHECK 1 compares two
%   transcriptions rather than one implementation with itself.
Y=0.; YD=0.; XNL=0.; D=0.; ELAMDH=0.; X4=0.; X5=0.; T=0.;
YDD=0.; XNLD=0.; ELAMDHD=0.; DD=0.; X4D=0.; X5D=0.;
while T <= (TF-.0001)
    YOLD=Y; YDOLD=YD; XNLOLD=XNL; DOLD=D;
    ELAMDHOLD=ELAMDH; X4OLD=X4; X5OLD=X5; STEP=1; FLAG=0;
    while STEP <= 1
        if FLAG==1
            STEP=2; Y=Y+H*YD; YD=YD+H*YDD; XNL=XNL+H*XNLD;
            ELAMDH=ELAMDH+H*ELAMDHD; D=D+H*DD; X4=X4+H*X4D; X5=X5+H*X5D; T=T+H;
        end
        YTDD=XNT*sin(W*T); YTDDD=W*XNT*cos(W*T); TGO=TF-T+.00001;
        XLAM=Y/(VC*TGO); DD=5.*(XLAM-D)/TAU; ELAMDHD=5.*(DD-ELAMDH)/TAU;
        if isfinite(ncHold)
            XNC=ncHold;
        elseif APN==1
            XNC=XNP*VC*ELAMDH;
        elseif APN==2
            XP=WH*TGO;
            XNC=XNP*VC*ELAMDH+XNP*YTDD*(1.-cos(XP))/XP^2+ ...
                XNP*YTDDD*(XP-sin(XP))/(XP*XP*WH);
        else
            X=TGO/TAU; XP=WH*TGO;
            TOP=6.*X*X*(exp(-X)-1.+X); BOT1=2*X*X*X+3.+6.*X-6.*X*X;
            BOT2=-12.*X*exp(-X)-3.*exp(-2.*X); XNPP=TOP/(.0001+BOT1+BOT2);
            XNC=XNPP*VC*ELAMDH+XNPP*YTDD*(1.-cos(XP))/XP^2+ ...
                XNPP*YTDDD*(XP-sin(XP))/(XP*XP*WH)- ...
                XNPP*XNL*TAU*TAU*(exp(-X)+X-1.)/TGO^2;
        end
        if XNC > XNCLIM,  XNC= XNCLIM; end
        if XNC < -XNCLIM, XNC=-XNCLIM; end
        X4D=5.*(XNC-X4)/TAU; X5D=5.*(X4-X5)/TAU; XNLD=5.*(X5-XNL)/TAU;
        YDD=YTDD-XNL; FLAG=1;
    end
    % The listing's "FLAG=0" belongs here and is dead; see the header.
    Y=.5*(YOLD+Y+H*YD); YD=.5*(YDOLD+YD+H*YDD); XNL=.5*(XNLOLD+XNL+H*XNLD);
    D=.5*(DOLD+D+H*DD); ELAMDH=.5*(ELAMDHOLD+ELAMDH+H*ELAMDHD);
    X4=.5*(X4OLD+X4+H*X4D); X5=.5*(X5OLD+X5+H*X5D);
end
XNLf = XNL;
end


% =============================================================================
function v = nppListing(x)
%NPPLISTING  The time-varying navigation ratio AS THE LISTING COMPUTES IT,
%   including the 0.0001 added to the denominator.  x = tgo/T.
%
%   ⚠️ THIS IS NOT THE FUNCTION FIG. 20.17 PLOTS.  The true denominator tends
%   to (3/10)*x^5, so 0.0001 dominates it below x ~ 0.5 and this gain turns
%   over and collapses toward zero near intercept.  Kept because it is what the
%   book flew; nppBook is the printed formula, and the two are returned side by
%   side in R.nprime.
TOP  = 6.*x.*x.*(exp(-x)-1.+x);
BOT1 = 2*x.*x.*x+3.+6.*x-6.*x.*x;
BOT2 = -12.*x.*exp(-x)-3.*exp(-2.*x);
v    = TOP./(.0001+BOT1+BOT2);
end


% =============================================================================
function v = nppBook(x)
%NPPBOOK  zarchan.txt:6364, p.466, printed exactly as the book prints it:
%
%     N' = 6x^2(e^-x - 1 + x) / (2x^3 + 3 + 6x - 6x^2 - 12x e^-x - 3 e^-2x)
%
%   No guard.  For x >~ 0.5 the denominator is well conditioned; below that it
%   is a difference of terms of order 3 cancelling to order (3/10)x^5 and it
%   loses its last significant digit near x ~ 0.002, returning a plausible
%   number rather than a NaN.  BENCHMARK 5 is therefore scored on [0.5, 20].
TOP = 6.*x.*x.*(exp(-x)-1.+x);
BOT = 2*x.*x.*x+3.+6.*x-6.*x.*x-12.*x.*exp(-x)-3.*exp(-2.*x);
v   = TOP./BOT;
end


% =============================================================================
function xd = departureX(x, a, b, frac)
%DEPARTUREX  Smallest x above which |b/a - 1| stays under frac.  NaN if it
%   never does.  Used only for the INFO row about the 1e-4 guard.
ok = abs(b./a - 1) <= frac;
i  = find(~ok, 1, 'last');
if isempty(i)
    xd = x(1);
elseif i >= numel(x)
    xd = NaN;
else
    xd = x(i+1);
end
end


% =============================================================================
function y = binomialStep(n, t, TAU)
%BINOMIALSTEP  Unit-step response of n identical lags of TAU/n each, so the
%   cascade carries total time constant TAU:
%       y(t) = 1 - e^-u * sum_{k=0}^{n-1} u^k/k! ,   u = n*t/TAU
u = n*t/TAU;
s = zeros(size(u));
c = ones(size(u));                    % u^k/k!, starting at k = 0
for k = 0:(n-1)
    s = s + c;
    c = c .* u / (k+1);
end
y = 1 - exp(-u).*s;
end


% =============================================================================
function F = sineFitWindow(tf, y, W)
%SINEFITWINDOW  Least-squares fit of a + b*sin(w t) + c*cos(w t) over the
%   settled part of the sweep, scanning w.  Base MATLAB only: a coarse scan
%   then one refinement, each a 3-column backslash.
%
%   Returns [] when the window cannot support the question -- fewer than 20
%   samples, or less than 1.5 full weave periods.  That is a SKIP on the board,
%   never a pass: a sine fitted to under a period recovers almost any frequency.
%
%   ⚠️ THE WINDOW IS 0.3*max(tF), NOT 0.4.  At the book's settings the PN miss
%   has settled by tF ~ 2.3 s, so both would work -- but 0.4 leaves a 6.0 s
%   window against a 2-period requirement of 6.28 s, and the row would SKIP on
%   the book's OWN case.  A gate that refuses the nominal run is not a
%   conservative gate, it is a broken one.
F = [];
lo = 0.3*max(tf);
m  = tf >= lo;
t  = tf(m).';  z = y(m).';
if numel(t) < 20 || (max(t) - min(t)) < 1.5*(2*pi/W)
    return
end
one  = ones(numel(t),1);
best = inf;  wf = NaN;  sol = [0;0;0];
grid = max(0.05, 0.25*W) : 0.01 : 3*W;
for pass = 1:2
    for w = grid
        A = [sin(w*t), cos(w*t), one];
        c = A\z;
        r = sum((z - A*c).^2);
        if r < best, best = r; wf = w; sol = c; end
    end
    grid = (wf - 0.01) : 0.0002 : (wf + 0.01);
end
tot = sum((z - mean(z)).^2);
F   = struct('omega', wf, 'r2', 1 - best/max(tot, realmin), ...
             'amp', hypot(sol(1), sol(2)), 'lo', min(t), 'hi', max(t), ...
             'n', numel(t));
end


% =============================================================================
function [pk, tpk] = peakOf(y, tf)
[pk, i] = max(abs(y));
tpk     = tf(i);
end


% =============================================================================
function K = structuralChecks(tf, miss, XNT, XNP, XNCLIM, TAU, W, WH, VC, H, np)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.  They are the weaker
%   claim that it agrees with itself -- which is what catches wiring faults.
names = {};  oks = [];  detail = {};

% 1  The vectorised sweep IS the listing, run scalar -- the only thing standing
%    between the array rewrite and a silently wrong board.  EXPECTED RESIDUAL
%    IS EXACTLY ZERO (same operator association both sides).  The band is 1e-7
%    of the peak only because MATLAB may contract a*b+c into an FMA in the array
%    path and not the scalar one; that random-walks to ~1e-14 relative over
%    ~1000 steps.  Nothing is lost: freezing an element one step late moves the
%    miss by H*ydot, O(1) feet.  No real wiring fault lands near 1e-8.
TFr  = tf(end);
scal = zeros(1,3);
for law = 1:3
    scal(law) = scalarRun(law, TFr, XNT, XNP, XNCLIM, TAU, W, WH, VC, H, NaN);
end
resid  = max(abs(scal - miss(:,end).'));
scale  = max(max(abs(miss(:))), realmin);
names{end+1}  = 'vectorised sweep == verbatim scalar listing (3 laws)';
oks(end+1)    = resid <= 1e-7*scale;
detail{end+1} = sprintf('max |vector - scalar| = %.3e ft at tF = %g s (peak %.4g ft)', ...
                        resid, TFr, scale);

% 2  Clamp the command shut (XNCLIM = 0, which fires BOTH limiter lines) and
%    the missile never manoeuvres, so the miss is the target's own double
%    integral, in closed form:
%        y(tF) = nT*( tF/w - sin(w*tF)/w^2 )
%    This one check exercises the target model, the integrator, the sign of
%    YDD = YTDD - XNL, and both halves of the limiter -- including whether the
%    unresolved "if XNC . XNCLIM" was resolved as ">" rather than "<".
%    THE BAND 0.5*H^2*W^3 IS DERIVED, NOT PICKED.  Clamped shut this is
%    quadrature of a sinusoid: Heun's global error in y is ~(H^2/12)*tF*nT*w^2
%    against y(tF) ~ nT*tF/w, so the RELATIVE error is ~H^2*w^3/12.  At the
%    book's point that predicts 6.7e-5; the residual is 3.65e-5 and falls
%    exactly 100x when H falls 10x.  A band flat in H would have been wrong the
%    first time anyone raised w -- the error goes as its CUBE.
yFree  = scalarRun(1, TFr, XNT, XNP, 0.0, TAU, W, WH, VC, H, NaN);
yExact = XNT*(TFr/W - sin(W*TFr)/W^2);
rFree  = abs(yFree/yExact - 1);
tolF   = max(1e-8, 0.5*H^2*W^3);
names{end+1}  = 'clamp shut => miss is the target''s own double integral';
oks(end+1)    = rFree <= tolF;
detail{end+1} = sprintf('%.5f vs %.5f ft, rel %.3e (band %.3e ~ H^2 w^3/12)', ...
                        yFree, yExact, rFree, tolF);

% 3  Hold the command constant and the listing's OWN X4/X5/XNL lines must be a
%    three-lag binomial of TAU/5 each.  This is where the book's "fifth-order
%    binomial, time constant 0.25 s" is actually adjudicated: it drives the
%    simulation, so a tidied-up lag gain shows here and nowhere else.
Hc     = min(H, TAU/500);
TFh    = TAU;
[~, xnlGot] = scalarRun(1, TFh, XNT, XNP, XNCLIM, TAU, W, WH, VC, Hc, 100.0);
xnlWant = 100.0*binomialStep(3, TFh, 3*TAU/5);
rHold   = abs(xnlGot/xnlWant - 1);
names{end+1}  = 'held command => nL is a 3-lag binomial of TAU/5';
oks(end+1)    = rHold <= 1e-4;
detail{end+1} = sprintf('nL(%.3f s) = %.6f vs %.6f ft/s^2, rel %.3e (H = %g s)', ...
                        TFh, xnlGot, xnlWant, rHold, Hc);
holdMeas = xnlGot;  holdWant = xnlWant;  holdRel = rHold;

% 4  Above the guard's reach the listing's gain and the printed formula are the
%    same function.  If they are not, one of the two was mistyped -- and the
%    printed one is what BENCHMARKS 2 and 3 score.
%
%    ⚠️ CUT x >= 5, BAND 1e-5, BOTH ARITHMETIC.  The two differ BY CONSTRUCTION
%    by exactly the guard: flown/book = BOT/(1e-4+BOT), and BOT ~ 2x^3.  At
%    x = 1 that designed-in difference is 5.2e-4; at x = 5 (BOT = 136) it is
%    7.3e-7.  This check first said "x >= 1, band 1e-6" -- which would have
%    failed a CORRECT port by 500x, reading as a mistyped formula.  A real
%    mistype moves these by O(1), so 1e-5 at x >= 5 keeps every tooth.
big   = np.x >= 5;
rGain = max(abs(np.flown(big)./np.book(big) - 1));
names{end+1}  = 'listing N'' == printed N'' where the 1e-4 guard is negligible';
oks(end+1)    = rGain <= 1e-5;
detail{end+1} = sprintf(['max rel difference over x >= 5 is %.3e; the guard ' ...
                         'alone accounts for 1e-4/(1e-4+BOT) = 7.3e-7 there'], rGain);

K = struct();
K.names  = names;
K.oks    = logical(oks);
K.detail = detail;
% Exported so the INFO row about the fifth-order binomial can quote the number
% this row measured instead of measuring a second, weaker one of its own.
K.hold_meas = holdMeas;  K.hold_want = holdWant;  K.hold_rel = holdRel;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    i = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', K.names{i}, K.detail{i});
end
end


% =============================================================================
function B = benchmarks(R, XNT, TAU, W)
%BENCHMARKS  See the header for what each row is and where it was grepped from.
B = emptyBench();
P = R.peak;

% ---- 1  the listing's 193.2 is the prose's 6 g ----------------------------
%   GATED ON THE BOOK'S OWN VALUE.  A row whose "book value" is really the
%   caller's parameter passes by construction and is worse than no row: it
%   occupies a slot on the board while adjudicating nothing.
if abs(XNT - 193.2) < 1e-9
    B = addVal(B, 'target maneuver amplitude', 6.0, XNT/32.2, 'prose', 1e-9, ...
        'p.467 text, Fig. 20.18 discussion (zarchan.txt:6375)', 'g', ...
        'the listing writes 193.2 ft/s^2; the prose calls the target 6-g');
else
    B = addSkip(B, 'target maneuver amplitude', '6', ...
        sprintf(['the book''s sentence is about ITS target, nT = 193.2 ' ...
                 'ft/s^2; this run uses %g, so there is nothing of the ' ...
                 'book''s to compare against'], XNT), ...
        'p.467 text (zarchan.txt:6375)', 'g');
end

% ---- 2  "fifth-order binomial ... time constant of 0.25 s" ----------------
%   INFO, NOT A VALUE ROW, AND THE DEMOTION IS THE POINT.  The obvious value
%   row here -- integrate a five-lag chain of TAU/5 and check it comes to TAU
%   -- compares two things this file wrote against each other and passes by
%   construction whatever the port does.  The claim is adjudicated where it can
%   actually fail: STRUCTURAL row 3 drives the LISTING'S OWN X4/X5/XNL lines
%   with a held command and lands them on the three-lag closed form.
B = addInfo(B, 'fifth-order binomial, total time constant TAU', ...
    sprintf('5 lags, %g s total', TAU), ...
    sprintf(['5 x %g s; the command chain''s own 3 lags measure %.4f vs ' ...
             '%.4f ft/s^2 (rel %.1e) -- see STRUCTURAL row 3'], ...
            TAU/5, R.selfcheck.hold_meas, R.selfcheck.hold_want, ...
            R.selfcheck.hold_rel), ...
    'p.467 text (zarchan.txt:6375); five-lag model p.453 (zarchan.txt:6140)', ...
    ['NOT SCORED HERE: a chain this file builds and a closed form this file ' ...
     'writes agree by construction.  The structural row is the one that can go red']);

% ---- 2/3  Fig. 20.17, the time-varying navigation ratio -------------------
B = addVal(B, 'N'' at long time-to-go (x = 1000)', 3.0, nppBook(1000), ...
    'prose "approaching 3"', 0.02, ...
    'Fig. 20.17 / p.467 text (zarchan.txt:6373); formula p.466 (zarchan.txt:6364)', '-', ...
    'the limit is exact; at a finite x the approach is from ABOVE, so a band and not an equality');

xs   = R.nprime.x;
mono = xs >= 0.5 & xs <= 20;
vv   = R.nprime.book(mono);
isDec = all(diff(vv) < 0);
B = addShape(B, 'N'' grows as time-to-go shrinks', ...
    '"as we get closer to intercept ... grows considerably"', ...
    sprintf('strictly decreasing in x over [0.5,20]: %s; N''(0.5)/N''(20) = %.2f', ...
            tf2str(isDec), nppBook(0.5)/nppBook(20)), ...
    isDec && nppBook(0.5)/nppBook(20) > 3, ...
    'Fig. 20.17 / p.467 text (zarchan.txt:6373)', ...
    'x = tgo/T; scored on the PRINTED formula, which is what the figure plots');

% ---- 4  the weave law at zero frequency ----------------------------------
% Representative, non-degenerate operating point.  Nothing about it is tuned:
% the identity holds at any of them, because BENCHMARK 4 is a LIMIT check on
% the two KERNELS and the kernels carry no parameters but w and tgo.
%
% N' IS PINNED AT THE BOOK'S 3 HERE, not taken from the caller.  The printed
% limit on p.465 is 3/tgo^2*[...]; evaluating it at some other N' would still
% be a true statement about this port's own law but it would no longer be the
% book's equation, and this row exists to reproduce the book's equation.  The
% kernels are shared with the flown law either way, so the row keeps its teeth
% at any XNP -- and it never has to skip.
elT = 1e-4;  ytdd = 150.0;  ytddd = 250.0;  tgo = 4.0;  vcT = 4000.0;
wSml = 1e-3;  nBook = 3.0;
got  = guidanceCmd(2, elT, 0.0, ytdd, ytddd, tgo, nBook, vcT, TAU, wSml);
want = nBook*vcT*elT + (nBook/2)*ytdd + (nBook/6)*tgo*ytddd;
B = addVal(B, 'weave law at w -> 0 is augmented PN (N''=3) + jerk', ...
    want, got, 'analytic limit', 1e-5, ...
    'p.465 text (zarchan.txt:6341-6346)', 'ft/s^2', ...
    sprintf(['(1-cos u)/u^2 -> 1/2 and (u-sin u)/(u^2 w) -> tgo/6, evaluated ' ...
             'at w = %g rad/s where the residual is still pure Taylor truncation'], wSml));

% ---- 5/6  Figs. 20.18 and 20.19, the two orderings the chapter turns on ---
B = addShape(B, 'weave guidance substantially reduces the miss vs PN', ...
    'peak|miss| PN >= 2 x peak|miss| weave', ...
    sprintf('PN %.3f ft (tF %.1f s) vs weave %.3f ft (tF %.1f s), ratio %.2f', ...
            P.pn, P.tf_pn, P.weave, P.tf_weave, P.pn/max(P.weave, realmin)), ...
    P.pn >= 2*P.weave, ...
    'Fig. 20.18 / p.467 text (zarchan.txt:6375)', ...
    '"substantially" is an adverb; the factor 2 is this port''s reading of it and is stated, not hidden');

B = addShape(B, 'compensating for the lag reduces the miss further', ...
    'peak|miss| compensated < peak|miss| weave', ...
    sprintf('compensated %.3f ft (tF %.1f s) vs weave %.3f ft, ratio %.2f', ...
            P.comp, P.tf_comp, P.weave, P.weave/max(P.comp, realmin)), ...
    P.comp < P.weave, ...
    'Fig. 20.19 / p.470 text (zarchan.txt:6405)', ...
    'the compensation is SUBOPTIMAL here -- it assumes one lag and the loop has five (zarchan.txt:6374)');

% ---- 7  the miss is a sinusoid at the TARGET weave frequency --------------
if isempty(R.fit)
    B = addSkip(B, 'settled miss oscillates at the target weave frequency', ...
        sprintf('%.4g rad/s', W), ...
        ['the settled part of the flight-time sweep does not carry 20 samples ' ...
         'over 1.5 weave periods, so a fitted frequency would not be ' ...
         'identified.  Widening the fit window to make this evaluable would ' ...
         'be widening a band.'], ...
        'p.441 text (zarchan.txt:5959)', 'rad/s');
else
    F  = R.fit;
    ok = abs(F.omega - W) <= 0.01*W && F.r2 >= 0.99;
    B  = addShape(B, 'settled miss oscillates at the target weave frequency', ...
        sprintf('w = %.4g rad/s (within 1%%), R^2 >= 0.99', W), ...
        sprintf('fitted w = %.4f rad/s, R^2 = %.5f, amplitude %.3f ft over tF in [%.1f, %.1f] s (%d pts)', ...
                F.omega, F.r2, F.amp, F.lo, F.hi, F.n), ok, ...
        'p.441 text (zarchan.txt:5959)', ...
        ['⚠️ TRANSFERRED: that sentence is about Listing 20.1 (w = 3 rad/s, ' ...
         'ONE lag).  Applied here at w = 2 with five lags.  Kept because it ' ...
         'is the only row that would notice W and WH swapped']);
end

% ---- INFO / SKIP ----------------------------------------------------------
B = addInfo(B, 'how large "substantial" is for PN', ...
    '"substantial miss distances"', ...
    sprintf('%.3f ft peak at tF = %.1f s', P.pn, P.tf_pn), ...
    'p.467 text (zarchan.txt:6375)', ...
    'NOT SCORED: the book''s word is an adjective and Fig. 20.18''s axes did not survive extraction');

B = addInfo(B, 'the 1e-4 denominator guard on XNPP', ...
    'not discussed in the book', ...
    sprintf('flown gain departs from the printed formula by >1%% below x = %.3f (tgo = %.4f s at TAU = %g s)', ...
            R.nprime.depart_x, R.nprime.depart_x*TAU, TAU), ...
    'listing line (zarchan.txt:6390)', ...
    ['NOT SCORED: it is the listing''s own numerical guard, not a claim.  ' ...
     'Below the crossover the flown gain TURNS OVER toward zero while ' ...
     'Fig. 20.17 shows the formula growing -- opposite directions']);

if ~isempty(R.wh_scan)
    S = R.wh_scan;
    B = addInfo(B, 'sensitivity to the assumed weave frequency WH', ...
        'the book always sets WH = W', ...
        strjoin(arrayfun(@(i) sprintf('%.2fx -> %.3f ft', S.ratio(i), S.peak_ft(i)), ...
                1:numel(S.ratio), 'UniformOutput', false), ',  '), ...
        'project question, not Zarchan', ...
        ['NOT SCORED: nothing is published to score it against.  It is here ' ...
         'because it is the number this project would actually need before ' ...
         'flying a law that assumes it knows a weave frequency']);
end

B = addSkip(B, 'miss-distance values off Figs. 20.17/20.18/20.19', ...
    'no number is printed', ...
    ['every figure in this chapter became an image link in the extraction, ' ...
     'and the surrounding prose gives orderings and adverbs only.  There is ' ...
     'no published miss distance for Listing 20.4 to score against, so the ' ...
     'value rows above are parameters and limits and the teeth are in the ' ...
     'shape rows.  Inventing a band to absorb a figure-read number is the ' ...
     'failure this row exists to name'], ...
    'Figs. 20.17-20.19 (zarchan.txt:6366-6367, 6401)', 'ft');
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want: a struct array
%   whose fields reordered would make [B.ok] mean something else.  Schema is
%   the one mgs.zarchan.benchSuite's shared benchRows() consumes.
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
%ADDSKIP  A check that COULD NOT RUN.  Never a pass: it counts as a check that
%   did not happen, which is a different thing from one that succeeded.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'skip';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = '--';
B(i).pct  = NaN;   B(i).band = 'skip';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end

function s = tf2str(v)
if v, s = 'yes'; else, s = 'NO'; end
end


% =============================================================================
function s = render(R, ev)
L = {};
p = R.p;
L{end+1} = 'Zarchan Listing 20.4 -- GUIDANCE LAWS AGAINST A WEAVING TARGET';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['   nT = %g ft/s^2 (%.4g g)   w = %g rad/s   WH = %g rad/s   ' ...
    'N'' = %g   TAU = %g s (5 lags of %g s)'], p.XNT, p.XNT/32.2, p.W, p.WH, ...
    p.XNP, p.TAU, p.TAU/5);
L{end+1} = sprintf('   Vc = %g ft/s   H = %g s   nc limit = %g ft/s^2   %d flight times, %g..%g s', ...
    p.VC, p.H, p.XNCLIM, numel(p.tf), min(p.tf), max(p.tf));
L{end+1} = '';

L{end+1} = 'MISS vs FLIGHT TIME  [ft]   (+ = target above the missile at intercept)';
L{end+1} = sprintf('%9s %14s %14s %14s', 'tF [s]', 'PN', 'weave', 'comp. weave');
step = max(1, round(numel(p.tf)/12));
for i = 1:step:numel(p.tf)
    L{end+1} = sprintf('%9.2f %14.4f %14.4f %14.4f', p.tf(i), ...
        R.miss(1,i), R.miss(2,i), R.miss(3,i)); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = sprintf('   peak |miss|:  PN %.3f ft (tF %.2f)   weave %.3f ft (tF %.2f)   comp %.3f ft (tF %.2f)', ...
    R.peak.pn, R.peak.tf_pn, R.peak.weave, R.peak.tf_weave, R.peak.comp, R.peak.tf_comp);
if ~isempty(R.fit)
    L{end+1} = sprintf('   settled PN miss: amplitude %.3f ft at w = %.4f rad/s (R^2 %.5f, tF %.1f-%.1f s)', ...
        R.fit.amp, R.fit.omega, R.fit.r2, R.fit.lo, R.fit.hi);
end
if ~isempty(R.wh_scan)
    L{end+1} = ['   WH/W sensitivity (compensated weave peak): ' ...
        strjoin(arrayfun(@(i) sprintf('%.2fx %.3f ft', R.wh_scan.ratio(i), ...
        R.wh_scan.peak_ft(i)), 1:numel(R.wh_scan.ratio), ...
        'UniformOutput', false), '  ')];
end
L{end+1} = '';

L{end+1} = 'BENCHMARKS';
for i = 1:numel(R.bench)
    b = R.bench(i);
    switch lower(b.kind)
    case 'info'
        tag = 'INFO';
    case 'skip'
        tag = 'SKIP';
    otherwise
        if b.ok, tag = 'PASS'; else, tag = 'FAIL'; end
    end
    L{end+1} = sprintf('  [%s] %s', tag, b.what); %#ok<AGROW>
    L{end+1} = sprintf('         book %s | ours %s', b.book_str, b.ours_str); %#ok<AGROW>
    L{end+1} = sprintf('         %s', b.source); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = 'STRUCTURAL (self-consistency -- NOT book values)';
for i = 1:numel(R.selfcheck.names)
    if R.selfcheck.oks(i), tag = 'ok '; else, tag = 'BAD'; end
    L{end+1} = sprintf('  [%s] %s', tag, R.selfcheck.names{i}); %#ok<AGROW>
    L{end+1} = sprintf('         %s', R.selfcheck.detail{i}); %#ok<AGROW>
end
L{end+1} = '';
if ~any(ev)
    L{end+1} = 'VERDICT: nothing was adjudicated.';
elseif R.pass
    L{end+1} = sprintf('VERDICT: all %d evaluated rows pass.', sum(ev));
else
    L{end+1} = 'VERDICT: see R.verdict.';
end
s = strjoin(L, newline);
end


% =============================================================================
function drawCurves(R)
figure('Name','Zarchan Listing 20.4 -- weave-compensated guidance');
subplot(2,1,1);
plot(R.tf, R.miss(1,:), '-', R.tf, R.miss(2,:), '-', R.tf, R.miss(3,:), '-');
grid on; xlabel('Flight Time (Sec)'); ylabel('Miss (Ft)');
legend('proportional navigation','weave','compensated weave','Location','best');
title(sprintf('nT = %g ft/s^2, w = %g rad/s, TAU = %g s (5th-order binomial)', ...
      R.p.XNT, R.p.W, R.p.TAU));
subplot(2,1,2);
semilogx(R.nprime.x, R.nprime.book, '-', R.nprime.x, R.nprime.flown, '--');
grid on; xlabel('t_{go}/T'); ylabel('effective N''');
legend('printed formula (Fig. 20.17)','as the listing computes it','Location','best');
ylim([0 40]);
end
