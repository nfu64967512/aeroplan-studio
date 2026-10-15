function R = stepAdjointSingleLag(opts)
%MGS.ZARCHAN.STEPADJOINTSINGLELAG  Zarchan Listing 19.3 -- adjoint for a STEP
%   IN TARGET DISPLACEMENT through a single time constant guidance system.
%
%   R = mgs.zarchan.stepAdjointSingleLag()
%   R = mgs.zarchan.stepAdjointSingleLag('XNP', 4)
%   R = mgs.zarchan.stepAdjointSingleLag('TAU', 0.2, 'assert', false)
%
%   Line-by-line port of Listing 19.3 (zarchan.txt:5700-5708), the adjoint of
%   Fig. 19.10, integrated by the book's own second-order Runge-Kutta (Heun).
%
%   *** UNITS ARE THE BOOK'S: FEET, ft/s, seconds; g = 32.2 ft/s^2.  NOT SI. ***
%   The rest of mgs is metres.  Nothing is converted here, so that every number
%   printed can be laid straight against a published one.  Convert AT THE CALL
%   SITE and say so there.
%
%   WHAT THE CHAPTER IS ABOUT.  A seeker resolves a formation: the apparent
%   target jumps from the power centroid to one aircraft, i.e. the target
%   POSITION steps by DISP with tF seconds of homing left.  Two error sources
%   arrive together and the listing tracks both -- the step itself (XMY) and
%   the initial condition it forces on the seeker dish angle (XIC).
%   TOT = XMY + XIC is the answer, in feet.
%
%   *** WHAT IT IS NOT. ***  A LINEARISED, PLANAR loop, constant closing
%   velocity, ONE first-order lag, no gravity, drag, thrust, acceleration limit
%   or LOS floor.  The chapter itself says a single lag "can lead to serious
%   underestimates of the miss distance": the fifth-order version needs
%   tF/T > 8 where this one needs > 3 (zarchan.txt:5815).  And the adjoint
%   identity is a LINEARITY identity, so nothing here represents a SATURATING
%   lag, which is our own dominant error source.  Clean-loop baseline, not a
%   prediction.
%
% ---------------------------------------------------------------------------
%   TRANSCRIPTION.  listing_19_03.m does not parse as MATLAB.  It reports no
%   unresolved variable-to-variable comparisons, which is right -- every
%   comparison in 19.3 is against a literal.  Three repairs:
%
%     1  "while ~(TP.(TF-.00001))" -> "while TP <= (TF - 1e-5)".  "." is this
%        book's OCR of ">".  Shape-forced anyway: TP only increases from 1e-5.
%     2  "while STEP,=1" -> "while STEP <= 1".  "," is the OCR of "<"; the body
%        sets STEP=2 to leave, which only makes sense against "<=".
%     3  *** THE EXTRACTOR GOT THIS ONE WRONG AND IT IS RESTORED HERE. ***
%        "if S,.09999" was auto-repaired to "if S < .09999".  It is ">=":
%          a  The record test appears ~38 times in this book and EVERY other
%             occurrence OCRs as ".=" or " . =", i.e. ">=" -- zarchan.txt:421,
%             1529, 4702, 6006, 8147, 10319, all "if S.=.09999".  Only :5707
%             and :5810 (Listings 19.3 and 19.4) lost the "=" and became ",".
%          b  With "<" the threshold is dead: S resets to 0 on every record and
%             one increment is H = 0.01 < 0.09999, so it records EVERY step and
%             .09999 could have been any number at all.
%          c  DECISIVE.  Fig. 19.11 overlays this listing on Listing 19.2,
%             whose sweep is "for THOM = 0.1:0.1:5" -- 50 points at 0.1 s.
%             ">=" with H = 0.01 records every 10th step: 0.1 s, 50 points.
%             The two curves only line up under ">=".
%        Cost of the wrong reading: 500 samples instead of 50, and a plot that
%        looks IDENTICAL.  That is why it is worth writing down.
%
%   Book quirks kept deliberately: TP starts at +1e-5 and TGO carries a SECOND
%   +1e-5 (the gain is 1/(VC*tgo); an exact zero divides by zero on the first
%   evaluation); XMHEPZ divides by TP while XIC divides by TGO -- mismatch and
%   all, because it is the entire residual in BENCHMARK 5 and is quantified
%   there; X1 is integrated and NEVER READ, and is kept, because deleting a
%   state changes nothing observable and that is exactly when a diff stops
%   being trustworthy.  Dropped: "save datfil.txt" writes nothing here.  The
%   literal 0.09999 is replaced by 'samp' (default 0.1, the listing's own
%   interval), since the literal silently changes meaning at other H.
%
% ---------------------------------------------------------------------------
%   THE EXACT INVARIANT -- this is what makes Fig. 19.12 a theorem, not a demo.
%   With Z4 = X4/VC and u = Z4 - N'*X2 the transcribed ODEs are
%       X2' = X3,   X3' = u/(TAU*tgo),   Z4' = -u/TAU
%   so d/dt(tgo*X3 + Z4) = X3 + u/TAU - u/TAU = X3 = X2'.  Hence
%
%       tgo*X3 + X4/VC - X2  ==  tgo(0)  ==  2e-5     FOR ALL t, ALL N'
%
%   TOT/DISP = X3 + X4/(VC*tgo) = X2/tgo and XMHEPZ/DISP = X2/TP, so the book's
%   "postulate" (zarchan.txt:5710) that the heading-error miss divided by
%   homing time EQUALS the target-displacement miss is an algebraic identity,
%   exact but for the 1e-5 between TP and TGO.
%   It is preserved EXACTLY by Heun, not merely to O(H^2): c(t) = [0,-1,tgo,
%   1/VC] is AFFINE in t (c'' = 0) and satisfies A(t)'*c(t) = -e3 at every t,
%   and those two facts kill the H, H^2 and H^3 terms of the Heun step
%   identically.  SELF-CHECK 2 is therefore banded at 1e-9 of peak -- a
%   ROUNDING check, and the sharpest thing in this file.
%   VC and VM both CANCEL out of every output (self-checks 3, 4); DISP scales
%   everything linearly.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS -- every book value grepped from zarchan.txt; the line number is
%   in each row's .source and the band's provenance is in its .detail, so this
%   is only the index.  Nothing was read off a figure by ME: the extraction
%   turned every figure into an image link.  Where the BOOK reports a figure
%   reading in prose, the row says so and is banded as one.
%
%   1-3  The three printed closed forms, zarchan.txt:5747-5760, x = tF/T:
%          Miss/yTIC |N'=3 = e^-x (1 - 0.5x)
%          Miss/yTIC |N'=4 = e^-x (1 - x + x^2/6)
%          Miss/yTIC |N'=5 = e^-x (1 - 1.5x + x^2/2 - x^3/24)
%        The OCR drops the minus signs; they are restored by SUBSTITUTION, not
%        guesswork -- the adjoint implies x*f'' + (2+x)*f' + N'*f = 0, and all
%        three vanish in it term for term.  Nothing else does.  Each row runs
%        its OWN sweep at that N' with the caller's other parameters, so all
%        three are scorable at ANY settings (the formulas depend on x alone).
%        BAND = max(0.5%, N'*H) of peak = TWICE the predicted Heun startup
%        error.  Mechanism: on the first step Y1 = (X4 - N'*VC*X2)/TAU is
%        exactly 0 while the true X3'(0+) is -N'/TAU, so Heun takes half the
%        correct first slope; the one-time offset lands on X3, and since the
%        whole solution IS the response to X3(0) = 1 it SCALES every state by
%        about (1 + 0.5*N'*H) -- +1.5/2.0/2.5% at H = 0.01.  The same
%        prediction gives +2.0% for mgs.zarchan.adjointSingleLag (N' = 4)
%        against its MEASURED +1.88%, so the mechanism is not a story.
%   4    zarchan.txt:5762, "MissN'=3 = 0.18 x 200 = 36 ft" at tF/T = 1, READ OFF
%        Fig. 19.13 by the book.  A BOUND, not a percentage: "0.18" to two
%        decimals is [35, 37] ft.  A percentage row would report a 2.2% "error"
%        that is entirely the book's own rounding of 0.1839.
%   5    SHAPE, zarchan.txt:5710-5712 (Fig. 19.12), max|XMHEPZ - TOT|.  Exact by
%        the invariant above; the residual is only the TP-vs-TGO 1e-5.
%   6    SHAPE, zarchan.txt:5684-5689 (Fig. 19.8/19.11), large miss at small
%        homing time and ~0 at long.  Scored as the RATIO the sentence states,
%        NOT as an absolute fraction of DISP early -- the first sample sits at
%        x = samp/TAU where the curve is both N'- and samp-dependent, so an
%        80%-of-DISP rule would red a correct N' = 5 run.
%
%   NOT SCORED (INFO), and why:
%     * zarchan.txt:5763-5764, "0.061 x 200 = 12 ft" and "0.015 x 200 = 3 ft".
%       The N'=5 one is a place THE BOOK CONTRADICTS ITSELF: its own formula
%       gives -3.07 ft at x = 1, and :5766 says negative means "below aircraft
%       1" -- yet Fig. 19.13's axis runs 0.0 to 1.0 and the prose prints "3 ft"
%       unsigned.  Scoring a sign we cannot source would be inventing a
%       benchmark, so the row reports both and adjudicates neither.
%     * zarchan.txt:5815, "greater than three [time constants] for the miss
%       distance to be negligible".  "Negligible" is an adjective.  Reported as
%       max|TOT| for x > 3, which the N'=3 formula pins at exactly
%       0.5*exp(-3)*DISP = 4.98 ft (the curve's minimum is AT x = 3).
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED. ***  This file has
%   never been executed -- MATLAB is the caller's, not this author's.
%     50 samples at TP = 0.10001, 0.20001, ... 5.00001 s (defaults)
%     TOT/DISP, closed form N'=3, at x = 0.1 / 1 / 2 / 3 / 5:
%          0.85960   0.18394   0.00000  -0.02489  -0.01011
%     in feet (DISP = 200):  171.92   36.79   0.00   -4.98   -2.02
%     the adjoint should sit about +1.5% above each, so about 37.3 ft at
%          tF/T = 1 against the book's 36 ft
%     N'=4 at x=1: 0.06131 (12.26 ft);  N'=5 at x=1: -0.01533 (-3.07 ft)
%     rows 1/2/3 residuals about 1.5/2.0/2.5% of peak, bands 3/4/5%
%     row 5 residual about 0.023 ft;  self-check 2 about 1e-14 of peak
%   A residual near a band EDGE is not a pass to be accepted -- the mechanism
%   above says where it should land.  Do not widen a band to make a row green.
%
% ---------------------------------------------------------------------------
%   OUTPUTS
%     R.t         adjoint time = the original system's HOMING TIME tF [s]
%     R.tgo       the TGO the listing divides by (= R.t + 1e-5)
%     R.x1..R.x4  the four adjoint states (x1 is integrated and never read)
%     R.xmy       DISP*x3 -- miss from the step alone [ft]
%     R.xic       x4*DISP/(VC*tgo) -- miss from the dish-angle IC [ft]
%     R.tot       xmy + xic -- THE ANSWER [ft];  R.tot_norm = tot/DISP
%     R.tot_cf    closed form counterpart [ft] (NaN unless N' is 3, 4 or 5)
%     R.xmhepz    heading-error miss / homing time [ft] -- Fig. 19.12's claim
%     R.units     one-line unit string
%     R.bench     benchmark rows, schema shared with the other ports
%     R.selfcheck structural residuals (self-consistency, NOT book values)
%     R.pass      true only if at least one row was SCORED and all passed
%     R.verdict   '' when everything passed, else the one-line reason.  ALWAYS
%                 FORMED.  'assert' picks the reaction and nothing else: true
%                 errors, false WARNS and still returns R complete.  benchSuite
%                 runs every port with assert off so one red row cannot hide
%                 the board.
%     R.p         the parameters actually used;  R.summary the rendered text
%
%   See also MGS.ZARCHAN.ADJOINTSINGLELAG (Listing 3.1, the heading-error and
%   step-maneuver adjoint of the same loop), MGS.ZARCHAN.ADJOINTBINOMIAL5,
%   MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 19.3 line 1, verbatim.  Feet and seconds.
    opts.XNP  (1,1) double {mustBeFinite, mustBePositive} = 3.0     % N-prime
    opts.TAU  (1,1) double {mustBeFinite, mustBePositive} = 1.0     % s
    opts.TF   (1,1) double {mustBeFinite, mustBePositive} = 5.0     % s, sweep end
    opts.VC   (1,1) double {mustBeFinite, mustBePositive} = 4000.0  % ft/s
    opts.DISP (1,1) double {mustBeFinite, mustBePositive} = 200.0   % ft
    opts.VM   (1,1) double {mustBeFinite, mustBePositive} = 3000.0  % ft/s
    opts.H    (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s, RK2 step
    % [] means "the listing's own 0.1 s" -- see repair 3 in the header.
    opts.samp double {mustBeReal, mustBeFinite} = []
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options, MATLAB reports nargin = 0 no matter how
    % many were passed, so an `if nargin == 0` here would be a mode that never
    % fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

XNP = opts.XNP;   TAU = opts.TAU;   TF  = opts.TF;
VC  = opts.VC;    DISP = opts.DISP; VM  = opts.VM;   H = opts.H;

samp = opts.samp;
if isempty(samp)
    samp = 0.1;                      % the listing's ".09999" with H = 0.01
end
if ~isscalar(samp) || samp <= 0
    error('mgs:zarchan:stepAdjointSingleLag:samp', ...
        'samp must be a positive scalar or [] (meaning the listing''s 0.1 s); got %s.', ...
        mat2str(samp));
end
if H > samp
    error('mgs:zarchan:stepAdjointSingleLag:step', ...
        ['integration step H = %g s is larger than the sample interval samp = ' ...
         '%g s, so the returned curve would be sampled at the step rate while ' ...
         'claiming the coarser interval in its own time vector.'], H, samp);
end
if H >= TAU
    error('mgs:zarchan:stepAdjointSingleLag:stiff', ...
        ['H = %g s is not smaller than the guidance time constant TAU = %g s. ' ...
         'RK2 does not blow up here -- it returns a smooth, entirely wrong ' ...
         'curve -- so this is refused rather than warned about.'], H, TAU);
end
if TF <= 2*H
    error('mgs:zarchan:stepAdjointSingleLag:short', ...
        'TF = %g s is not long enough to take two steps at H = %g s.', TF, H);
end

% =========================================================================
%   LISTING 19.3.  Transcribed from zarchan.txt:5700-5708.
% =========================================================================
[t, X, tgo, nEval] = adjointRun(XNP, TAU, TF, VC, H, samp);
if numel(t) < 2
    % Refused, not warned about.  An empty curve returned quietly looks like a
    % converged result, and a ONE-sample curve is worse: diff() on it is empty
    % and several checks below would then append an empty verdict to a
    % pass/fail vector -- a "null assignment" error thrown far from the samp
    % value that caused it.
    error('mgs:zarchan:stepAdjointSingleLag:empty', ...
        ['the sweep recorded %d sample(s) at TF = %g s, samp = %g s.  At least ' ...
         'two are needed for any curve comparison to mean anything.'], ...
        numel(t), TF, samp);
end

x1 = X(:,1);  x2 = X(:,2);  x3 = X(:,3);  x4 = X(:,4);

% The listing's sampling block, verbatim.  HE and the -VM in XMHE cancel; both
% are kept so that SELF-CHECK 4 can hold the transcription to it.
HE     = -DISP/VM;
xmy    = DISP*x3;
xic    = x4*DISP./(VC*tgo);
tot    = xmy + xic;
xmhe   = -VM*HE*x2;
xmhepz = xmhe./t;

tot_norm = tot/DISP;
cf_norm  = normMiss(t/TAU, XNP);      % NaN vector unless N' is 3, 4 or 5

R = struct();
R.t = t;  R.tgo = tgo;
R.x1 = x1;  R.x2 = x2;  R.x3 = x3;  R.x4 = x4;
R.xmy = xmy;  R.xic = xic;  R.tot = tot;
R.tot_norm = tot_norm;
R.tot_cf   = DISP*cf_norm;
R.xmhepz   = xmhepz;
R.units    = 'ft, ft/s, s; g = 32.2 ft/s^2 (Ch.19 is a foot-second chapter)';
R.p = struct('XNP',XNP, 'TAU',TAU, 'TF',TF, 'VC',VC, 'DISP',DISP, 'VM',VM, ...
             'H',H, 'samp',samp, 'HE',HE, 'n_rhs',nEval, 'units',R.units);

R.selfcheck = structuralChecks(R, XNP, TAU, TF, VC, DISP, VM, H, samp);

if opts.benchmark
    R.bench = benchmarks(R, XNP, TAU, TF, VC, DISP, H, samp);
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

% ---- adjudication.  The verdict is FORMED either way; only the reaction to it
%      depends on 'assert'.  A port that errors unconditionally cannot be
%      diagnosed, because the caller never gets R.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the book. ' ...
           'R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = ['not one book value could be evaluated.  An unvalidated port must ' ...
           'not be handed back silently.'];
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question.  ' ...
        'First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf(['%d of %d evaluated book values are outside tolerance ' ...
        '(first: %s, want %s, ours %s).'], numel(bad), sum(ev), ...
        R.bench(bad(1)).what, R.bench(bad(1)).book_str, R.bench(bad(1)).ours_str);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:stepAdjointSingleLag:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN, with R complete, so the caller can see WHICH side of
        % a comparison moved.
        warning('mgs:zarchan:stepAdjointSingleLag:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [t, X, tgo, nEval] = adjointRun(XNP, TAU, TF, VC, H, samp)
%ADJOINTRUN  Listing 19.3, zarchan.txt:5700-5708.
%
%   Every constant this needs is an ARGUMENT.  MATLAB local functions cannot
%   see the main function's workspace; a previous wave in this package read a
%   constant from the caller and threw on the first line of the first run.

S  = 0.0;
TP = 0.0 + 0.00001;      % NOT 0: the adjoint gain is 1/(VC*tgo).
X1 = 0.0;
X2 = 0.0;
X3 = 1.0;                % <<< THE IMPULSE, on the miss node.
X4 = 0.0;

% Derivatives initialised before the loop so the FLAG==1 branch inside it
% cannot read a value left over from anything else.
X1D = 0.0;  X2D = 0.0;  X3D = 0.0;  X4D = 0.0;
TGO = TP + 0.00001;

nmax = ceil(TF/min(samp, H)) + 8;
t   = zeros(nmax,1);
X   = zeros(nmax,4);
tgo = zeros(nmax,1);
n   = 0;
nEval = 0;

while TP <= (TF - 1e-5)
    % The listing increments S HERE, at the top of the outer body, once per
    % step -- unlike most listings in the book, which do it in the corrector.
    S = S + H;
    X1OLD = X1;  X2OLD = X2;  X3OLD = X3;  X4OLD = X4;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  TP advances INSIDE it, exactly once,
    % which is what puts the second derivative evaluation at t+H.  Move that
    % line out and the scheme quietly degrades to first order -- smoothly.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            X1 = X1 + H*X1D;
            X2 = X2 + H*X2D;
            X3 = X3 + H*X3D;
            X4 = X4 + H*X4D;
            TP = TP + H;
        end
        X1D = X2;
        X2D = X3;
        Y1  = (X4 - XNP*VC*X2)/TAU;   % the single guidance lag
        TGO = TP + 0.00001;           % adjoint time IS time-to-go
        X3D = Y1/(VC*TGO);
        X4D = -Y1;
        FLAG = 1;
        nEval = nEval + 1;
    end
    % The listing writes "FLAG = 0" here too.  Dropped: the outer loop
    % re-initialises FLAG on its next pass, so the assignment cannot be read.
    % Kept as a comment so nobody re-adds it thinking a state machine was lost.

    % Heun corrector, in the listing's own algebra: (x_old + x_pred)/2 +
    % (H/2)*f(t+H, x_pred), which equals x_old + (H/2)*(f_old + f_pred).
    X1 = (X1OLD + X1)/2 + 0.5*H*X1D;
    X2 = (X2OLD + X2)/2 + 0.5*H*X2D;
    X3 = (X3OLD + X3)/2 + 0.5*H*X3D;
    X4 = (X4OLD + X4)/2 + 0.5*H*X4D;

    % "if S >= .09999" -- see repair 3 in the header for why this is ">=" and
    % not the extractor's "<".  Tested against samp so a caller's interval is
    % honoured rather than the literal.
    if S >= samp - 0.5*H
        S = 0.0;
        n = n + 1;
        t(n)   = TP;
        X(n,:) = [X1 X2 X3 X4];
        tgo(n) = TGO;
    end
end

t = t(1:n);  X = X(1:n,:);  tgo = tgo(1:n);
end


% =============================================================================
function f = normMiss(x, N)
%NORMMISS  The book's printed normalized miss, zarchan.txt:5747-5760, for
%   x = tF/T.  Returns NaN for any other N' -- there is no formula for one.
%
%   The OCR lost the minus signs ("ex" for e^-x, and the signs inside the
%   brackets).  They are restored by SUBSTITUTION, not by guesswork: the
%   adjoint implies x*f'' + (2+x)*f' + N'*f = 0 for f = Miss/yTIC, and all
%   three forms below vanish in it term for term.  Nothing else does.
if abs(N - 3) < 1e-12
    f = exp(-x).*(1 - 0.5*x);
elseif abs(N - 4) < 1e-12
    f = exp(-x).*(1 - x + x.^2/6);
elseif abs(N - 5) < 1e-12
    f = exp(-x).*(1 - 1.5*x + x.^2/2 - x.^3/24);
else
    f = nan(size(x));
end
end


% =============================================================================
function r = maxRel(a, b)
%MAXREL  Max absolute difference NORMALISED BY THE REFERENCE CURVE'S OWN PEAK.
%   Not pointwise relative: every one of these curves crosses zero (at x = 2
%   for N' = 3), and a pointwise relative error there divides a small number by
%   a smaller one and manufactures a failure out of nothing.
den = max(abs(b));
if ~(den > 0)
    r = Inf;                 % refuse to report "perfect" against a zero curve
    return
end
r = max(abs(a - b))/den;
end


% =============================================================================
function K = structuralChecks(R, XNP, TAU, TF, VC, DISP, VM, H, samp)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.  They are the weaker
%   claim that it agrees with itself -- which is what catches wiring faults.
K = struct();
names = {};  oks = [];  detail = {};

% 1  TGO really is the recorded time plus 1e-5.  The only thing that would
%    notice someone "cleaning up" the divisor in XIC to R.t.
K.tgo_resid = max(abs(R.tgo - (R.t + 1e-5)));
names{end+1} = 'tgo == t + 1e-5';
oks(end+1)   = K.tgo_resid <= 1e-12;
detail{end+1} = sprintf('max |tgo - (t+1e-5)| = %.3e s', K.tgo_resid);

% 2  THE ADJOINT INVARIANT.  tgo*X3 + X4/VC - X2 == tgo(0) == 2e-5 exactly --
%    see the header for why Heun preserves this to ROUNDING and not merely to
%    O(H^2).  This is the sharp one: a swapped derivative, a dropped XNP in Y1
%    or a sign flip on X4D moves it by order 1 while leaving TOT looking like a
%    plausible miss curve.
tgo0  = 2e-5;
peak2 = max(abs(R.x2));
K.invariant = max(abs(R.tgo.*R.x3 + R.x4/VC - R.x2 - tgo0))/max(peak2, realmin);
names{end+1} = 'tgo*x3 + x4/VC - x2 == 2e-5  (exact adjoint invariant)';
oks(end+1)   = K.invariant <= 1e-9;
detail{end+1} = sprintf('%.3e of peak |x2| (band 1e-09; expect rounding, ~1e-14)', ...
    K.invariant);

% 3  VC CANCELS out of every output.  It enters Y1 and X3D and must leave no
%    trace.  Two full sweeps at VC and 3*VC; if a future edit puts a real
%    closing-velocity dependence in, nothing else here notices.
[tb, Xb, tgob] = adjointRun(XNP, TAU, TF, 3*VC, H, samp);
if numel(tb) == numel(R.t)
    totb = DISP*Xb(:,3) + Xb(:,4)*DISP./(3*VC*tgob);
    K.vc_resid = max(abs(totb - R.tot))/max(max(abs(R.tot)), realmin);
else
    K.vc_resid = Inf;
end
names{end+1} = 'TOT independent of VC';
% 1e-9, not 1e-14: VC cancels ALGEBRAICALLY, not bit-for-bit -- X4 carries a
% factor VC through a step that divides by VC*TGO with TGO ~ 1e-5, so the two
% runs take different rounding paths.  Every plausible way to BREAK the
% cancellation changes TOT by order 1, so this still discriminates by eight
% orders of magnitude.
oks(end+1)   = K.vc_resid <= 1e-9;
detail{end+1} = sprintf('|dTOT|/peak = %.3e between VC = %g and %g ft/s', ...
    K.vc_resid, VC, 3*VC);

% 4  HE = -DISP/VM and XMHE = -VM*HE*X2 must round-trip to DISP*X2.  Both halves
%    are carried verbatim from the listing precisely so that dropping one is
%    visible; without this row VM could vanish and nothing would change.
HE = -DISP/VM;
K.vm_resid = max(abs((-VM*HE*R.x2) - DISP*R.x2))/max(DISP*max(abs(R.x2)), realmin);
names{end+1} = 'XMHE == DISP*x2  (HE = -DISP/VM round trip)';
oks(end+1)   = K.vm_resid <= 1e-12;
detail{end+1} = sprintf('%.3e of peak (VM = %g ft/s cancels)', K.vm_resid, VM);

K.names    = names;
K.oks      = logical(oks);
K.detail   = detail;
K.all_ok   = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    j = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{j}, detail{j});
end
end


% =============================================================================
function B = benchmarks(R, XNP, TAU, TF, VC, DISP, H, samp)
%BENCHMARKS  Computed vs published.  EVERY book value was grepped out of
%   zarchan.txt and the line number is in the .source field.  Bands are set by
%   the provenance of the book value and by the O(H) mechanism in the header,
%   never by what we happen to compute.  Do NOT widen one to turn a row green.

B = emptyBench();

% ---- 1-3.  the three printed closed forms --------------------------------
srcN = {'p.427 closed form N''=3 (zarchan.txt:5747-5749, x at :5760)', ...
        'p.427 closed form N''=4 (zarchan.txt:5750-5753, x at :5760)', ...
        'p.427 closed form N''=5 (zarchan.txt:5754-5759, x at :5760)'};
formN = {'exp(-x)*(1 - x/2)', ...
         'exp(-x)*(1 - x + x^2/6)', ...
         'exp(-x)*(1 - 1.5x + x^2/2 - x^3/24)'};

for k = 1:3
    Nk  = k + 2;
    tol = max(0.005, Nk*H);       % 2x the predicted 0.5*N'*H startup scaling
    if abs(XNP - Nk) < 1e-12
        tk = R.t;  totk = R.tot_norm;      % the main run already IS this case
        note = 'the caller''s own run';
    else
        [tk, Xk, tgok] = adjointRun(Nk, TAU, TF, VC, H, samp);
        totk = Xk(:,3) + Xk(:,4)./(VC*tgok);
        note = sprintf('own sweep at N''=%d (caller ran N''=%g)', Nk, XNP);
    end
    cfk = normMiss(tk/TAU, Nk);
    rk  = maxRel(totk, cfk);
    B = addBound(B, sprintf('TOT/DISP == %s   [N''=%d]', formN{k}, Nk), ...
        sprintf('<= %.4g%% of peak', 100*tol), ...
        sprintf('%.4g%% of peak', 100*rk), rk <= tol, srcN{k}, '-', ...
        sprintf(['%s; band = max(0.5%%, N''*H) = 2x the predicted +%.2g%% ' ...
                 'Heun startup scaling'], note, 100*0.5*Nk*H));
end

% ---- 4.  the book's own reading off Fig. 19.13 ---------------------------
src4 = 'p.427 prose, read off Fig. 19.13 (zarchan.txt:5762)';
[dx, j] = min(abs(R.t/TAU - 1));
if abs(DISP - 200) > 1e-9
    B = addSkip(B, 'miss at tF/T = 1, N''=3, 200 ft displacement', '36 ft', ...
        sprintf(['the book''s 36 ft is 0.18 x 200 ft; this run used DISP = ' ...
                 '%g ft, so the comparable value would be 0.18 x %g = %.4g ft ' ...
                 '-- a different number, not a rescaled benchmark'], ...
                DISP, DISP, 0.18*DISP), src4, 'ft');
elseif abs(XNP - 3) > 1e-12
    B = addSkip(B, 'miss at tF/T = 1, N''=3, 200 ft displacement', '36 ft', ...
        sprintf('quoted for N''=3; this run used N''=%g', XNP), src4, 'ft');
elseif dx > 0.01
    B = addSkip(B, 'miss at tF/T = 1, N''=3, 200 ft displacement', '36 ft', ...
        sprintf(['no sample lands within 1%% of tF/T = 1: nearest is ' ...
                 'tF/T = %.4g at TF = %g s, TAU = %g s, samp = %g s'], ...
                R.t(j)/TAU, TF, TAU, samp), src4, 'ft');
else
    % "0.18" to two decimals is the interval [0.175, 0.185] -> [35, 37] ft,
    % widened by the same Heun scaling as rows 1-3.  Scored as a BOUND: a
    % percentage row would report a 2.2% "error" that is entirely the book's
    % own rounding of 0.1839.
    scheme = max(0.005, 3*H);
    lo = 0.175*DISP*(1 - scheme);
    hi = 0.185*DISP*(1 + scheme);
    v  = R.tot(j);
    B = addBound(B, 'miss at tF/T = 1, N''=3, 200 ft displacement', ...
        sprintf('in [%.4g, %.4g] ft  (0.18 read to 2 dp, +/- scheme)', lo, hi), ...
        sprintf('%.4g ft', v), v >= lo && v <= hi, src4, 'ft', ...
        sprintf(['the formula at :5747 gives %.4g ft exactly; the book prints ' ...
                 '36 because it read 0.18 off the figure'], DISP*normMiss(1,3)));
end

% ---- 5.  Fig. 19.12: heading-error miss / homing time == step miss -------
% Expected residual is NOT zero and is known: DISP*[x2*1e-5 - 2e-5*TP]/(TP*TGO),
% largest at the FIRST sample.  The band tracks t(1) so a finer samp cannot
% manufacture a red light out of the 1e-5 offset.
tol5 = max(1e-3, 4*2e-5/R.t(1))*DISP;
r5   = max(abs(R.xmhepz - R.tot));
B = addShape(B, 'XMHEPZ == TOT  (heading-error miss / homing time)', ...
    sprintf('max |XMHEPZ - TOT| <= %.4g ft', tol5), ...
    sprintf('%.4g ft', r5), r5 <= tol5, ...
    'p.426 prose about Fig. 19.12 (zarchan.txt:5710-5712)', ...
    ['exact identity of the adjoint (see header); the residual is only the ' ...
     'TP-vs-TGO 1e-5, about 0.023 ft at samp = 0.1 s']);

% ---- 6.  Fig. 19.8/19.11: large miss at short homing time, ~0 at long ----
src6 = 'p.423 prose about Fig. 19.8 (zarchan.txt:5684-5689)';
if TF/TAU < 4
    B = addSkip(B, 'miss large at small tF, ~0 at large tF', ...
        'first >= 10x last, and last <= 0.05*DISP', ...
        sprintf(['the claim is about LONG homing times; TF/TAU = %.4g here, ' ...
                 'and the curve has not decayed by then (the N''=3 formula ' ...
                 'is still %.3g of DISP at x = %.4g)'], TF/TAU, ...
                abs(normMiss(TF/TAU,3)), TF/TAU), src6, 'ft');
elseif R.t(1)/TAU > 1
    B = addSkip(B, 'miss large at small tF, ~0 at large tF', ...
        'first >= 10x last, and last <= 0.05*DISP', ...
        sprintf(['the claim is about SMALL homing times, and the first sample ' ...
                 'is already at tF/T = %.4g (samp = %g s, TAU = %g s)'], ...
                R.t(1)/TAU, samp, TAU), src6, 'ft');
else
    % A RATIO, not an absolute fraction of DISP -- see the header for why an
    % 80%-of-DISP early threshold would red a correct N' = 5 run.
    early = abs(R.tot(1));
    late  = abs(R.tot(end));
    ok6   = early >= 10*late && late <= 0.05*DISP;
    B = addShape(B, 'miss large at small tF, ~0 at large tF', ...
        sprintf('first >= 10x last, and last <= %.4g ft', 0.05*DISP), ...
        sprintf('first %.4g ft (tF/T=%.4g), last %.4g ft (tF/T=%.4g), ratio %.4g', ...
                early, R.t(1)/TAU, late, R.t(end)/TAU, ...
                early/max(late, realmin)), ok6, src6, ...
        ['scores the PROSE; the book publishes no number for Fig. 19.8.  NOT ' ...
         'the sharp detector -- dropping XIC still passes it -- rows 1-3 pin ' ...
         'the curve']);
end

% ---- INFO ----------------------------------------------------------------
cf4 = normMiss(1,4);  cf5 = normMiss(1,5);
B = addInfo(B, 'the other two figure reads at tF/T = 1', '', ...
    sprintf(['book 0.061*200 = 12 ft (N''=4), 0.015*200 = 3 ft (N''=5); ' ...
             'the printed formulas give %.4g ft and %+.4g ft'], ...
            200*cf4, 200*cf5), ...
    'p.427 prose, read off Fig. 19.13 (zarchan.txt:5763-5764)', ...
    ['reported, never scored -- and the N''=5 entry is a place THE BOOK ' ...
     'CONTRADICTS ITSELF: its own formula at :5754 gives a NEGATIVE value at ' ...
     'x = 1, and :5766 says negative means "below aircraft 1", yet Fig. 19.13''s ' ...
     'axis runs 0.0 to 1.0 and the prose prints "3 ft" unsigned.  Scoring a ' ...
     'sign we cannot source would be inventing a benchmark.']);

xs = R.t/TAU;
if any(xs > 3)
    tail = max(abs(R.tot(xs > 3)));
else
    tail = NaN;
end
B = addInfo(B, '"> 3 guidance time constants for negligible miss"', '', ...
    sprintf('max |TOT| for tF/T > 3 is %.4g ft (N''=3 formula pins it at %.4g ft)', ...
            tail, 0.5*exp(-3)*DISP), ...
    'p.430 prose (zarchan.txt:5815)', ...
    ['"negligible" is an adjective, so this is reported, not adjudicated.  ' ...
     'The N''=3 curve has its MINIMUM exactly at x = 3, value -exp(-3)/2.']);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want: a struct array
%   that quietly reordered fields would make [B.ok] mean something else.
%   Schema is the one mgs.zarchan.benchSuite's shared benchRows() consumes.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
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
%ADDSKIP  A check that COULD NOT RUN.  Never a pass: it is counted as a check
%   that did not happen, which is a different thing from one that succeeded.
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
L{end+1} = 'Zarchan Listing 19.3 -- ADJOINT FOR STEP IN TARGET DISPLACEMENT,';
L{end+1} = '   SINGLE TIME CONSTANT GUIDANCE SYSTEM';
L{end+1} = '*** UNITS ARE FEET, ft/s, s.  g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['   N'' = %g   TAU = %g s   TF = %g s   VC = %g ft/s   ' ...
    'DISP = %g ft   VM = %g ft/s'], p.XNP, p.TAU, p.TF, p.VC, p.DISP, p.VM);
L{end+1} = sprintf('   H = %g s   samp = %g s   -> %d samples, %d RHS evaluations', ...
    p.H, p.samp, numel(R.t), p.n_rhs);
L{end+1} = '';

L{end+1} = 'MISS vs HOMING TIME  (the listing''s TOT, XMY, XIC, XMHEPZ)';
L{end+1} = sprintf('%9s %9s %11s %11s %11s %11s %11s', ...
    'tF [s]', 'tF/T', 'XMY [ft]', 'XIC [ft]', 'TOT [ft]', 'XMHEPZ', 'closed form');
stride = max(1, round(0.5/p.samp));
rows = unique([1:stride:numel(R.t), numel(R.t)]);
for r = rows
    L{end+1} = sprintf('%9.4f %9.4f %11.4f %11.4f %11.4f %11.4f %11s', ...
        R.t(r), R.t(r)/p.TAU, R.xmy(r), R.xic(r), R.tot(r), R.xmhepz(r), ...
        numOrDash(R.tot_cf(r))); %#ok<AGROW>
end
L{end+1} = '';
if ~any(isfinite(R.tot_cf))
    L{end+1} = sprintf(['NOTE: N'' = %g, so the "closed form" column is empty. ' ...
        'Chapter 19 prints'], p.XNP);
    L{end+1} = '      formulas for N'' = 3, 4 and 5 ONLY (zarchan.txt:5747-5760).';
    L{end+1} = '      The closed-form rows below still run, each at its own N''.';
    L{end+1} = '';
end

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-56s %s', vstr(K.oks(i)), K.names{i}, ...
        K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning';
    L{end+1} = '            NOT CHECKED, which is not the same thing as "fine".';
else
    L{end+1} = 'BENCHMARKS  (book values grepped from zarchan.txt; line numbers in .source)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
            case 'info'
                L{end+1} = sprintf('   INFO %-52s', b.what); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.why); %#ok<AGROW>
            case 'skip'
                L{end+1} = sprintf('   SKIP %-52s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.why); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf('   %-4s %-52s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped/info -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: TOT is a MISS IN FEET for a step of DISP ft in apparent target';
L{end+1} = '   position -- it is not a maneuver miss and not a sensitivity.  A single';
L{end+1} = '   lag UNDERSTATES it (the book''s own warning, zarchan.txt:5815: the';
L{end+1} = '   fifth-order loop needs tF/T > 8 where this one needs > 3), and the';
L{end+1} = '   adjoint identity is a LINEARITY identity, so a saturating lag -- ours --';
L{end+1} = '   is outside what any of this can say.';
s = strjoin(L, newline);
end


function s = numOrDash(v)
if isfinite(v)
    s = sprintf('%11.4f', v);
else
    s = sprintf('%11s', '--');
end
end

function s = vstr(tf)
if tf
    s = 'PASS';
else
    s = 'FAIL';
end
end


% =============================================================================
function drawCurves(R)
%DRAWCURVES  Base MATLAB graphics only.  Reproduces the listing's own
%   plot(ArrayTP, ArrayTOT, ArrayTP, ArrayXMHEPZ) -- i.e. Fig. 19.12.
figure;
hold on; grid on;
plot(R.t, R.tot, '-');
lg = {'TOT (step in target displacement)'};
plot(R.t, R.xmhepz, '--');
lg{end+1} = 'XMHEPZ (heading-error miss / homing time)';
if any(isfinite(R.tot_cf))
    plot(R.t, R.tot_cf, ':');
    lg{end+1} = 'Ch.19 closed form';
end
xlabel('Homing Time (s)');
ylabel('Miss (Ft)');
title(sprintf(['Adjoint, step in target displacement, single lag ' ...
    '(N''=%g, T=%g s, %g ft step)'], R.p.XNP, R.p.TAU, R.p.DISP));
legend(lg{:}, 'Location', 'best');
hold off;
end
