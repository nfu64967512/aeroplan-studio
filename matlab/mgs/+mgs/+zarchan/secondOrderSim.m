function R = secondOrderSim(opts)
%MGS.ZARCHAN.SECONDORDERSIM  Zarchan Listing 1.1 -- a second-order network
%   stepped by the book's own second-order Runge-Kutta (Heun) integrator.
%
%   R = mgs.zarchan.secondOrderSim()
%   R = mgs.zarchan.secondOrderSim('H', 0.01)               coarser step
%   R = mgs.zarchan.secondOrderSim('W', 10, 'assert', false)
%
%   Line-by-line port of Listing 1.1 (zarchan.txt:255-270), the twelve-line
%   program the book opens with -- and the integrator the REST OF THE BOOK RUNS
%   ON (zarchan.txt:1415, "In all of the systems simulated in the text, the
%   second-order Runge-Kutta numerical integration method is used").  Every mgs
%   port that says "the book's own second-order Runge-Kutta scheme" means THIS
%   loop, so a fault here moves all of them the same way and they would still
%   agree with each other.  That is why this one is checked against an EXACT
%   solution rather than against a sibling.
%
%   *** NO FEET AND NO g HERE. ***  Y/X = w/(s^2 + w^2) has dimensions of TIME,
%   so y is in (input units)*seconds.  Do not carry the ft, g = 32.2 convention
%   of Chapters 2-8 into this listing.
%
%   THE TRANSCRIPTION.  Plant, zarchan.txt:236-239: ydd = w*x - w^2*y.  Scheme,
%   :231-232: x(K+1) = x(K) + h*f(x,t)/2 + h*f(x,t+h)/2 (the extraction lost the
%   fraction bars -- ":231" reads "hf(x; t) 2" and that 2 is a DENOMINATOR; the
%   listing's own corrector settles it).  listing_01_01.m reports NO unresolved
%   variable-to-variable comparisons -- correct, every comparison is against a
%   literal -- and one prose leak.  Four repairs, none a judgement call:
%
%     1  "while T , = (1.-1e-5)" -> "<=".  The OCR renders "<" as "," in this
%        book.  Shape-forced: T starts at 0, so ">=" is false immediately and
%        the program produces NOTHING, and "==" never fires.
%     2  "while STEP , = 1"      -> "<=".  Same substitution; the body sets
%        STEP = 2 to leave, which only makes sense against "<=".
%     3  "if S . = .000999"      -> ">=".  "." is the OCR of ">".  S resets to 0
%        on each record and the increment is H = 0.001, so "<=" would record
%        nothing at all after the first step.
%     4  The trailing "Z TRANSFORMS AND DIFFERENCE EQUATIONS" line and its
%        paragraph are the NEXT SECTION leaking in (the listing ends at :270,
%        that heading is :271).  Dropped, not "fixed" into code.
%
%   *** THE LISTING NEVER ASSIGNS W. ***  YDD = W*X - W*W*Y reads a variable the
%   printed program does not set.  w = 20 rad/s comes from PROSE (:252) and is
%   corroborated by Listing 5.1, the same network, which does print it:
%   "XIN=1.; W=20." (:1448).  That is an INFO row rather than a silent default,
%   because a port that invents a missing constant looks exactly like one that
%   transcribed a printed one.  Also: "clc" dropped (it would blank the caller's
%   window mid-suite), "save datfil output -ascii" recorded and dropped, and the
%   sampling threshold 0.000999 kept LITERAL -- see below.
%
%   THE BOOK CONTRADICTS ITSELF ON SAMPLING.  :252 says "every 0.01 s we print
%   out the output" (= 100 samples); the listing tests "S >= .000999" against an
%   increment of H = 0.001, so it fires EVERY step and records 1000.  The
%   LISTING is scored and the prose is cited beside it.  A threshold of .00999 --
%   one digit -- would satisfy the prose and is the likeliest OCR casualty on
%   that page, but the transcription is what gets ported; the book is not edited
%   to agree with itself.
%
%   BANDS.  The residual is DISCRETIZATION and its size is PREDICTABLE, so the
%   band is derived, not fitted.  Heun on this plant is exactly I + hA +
%   (hA)^2/2, eigenvalues (1 - q^2/2) +/- i*q with q = w*H: per step it runs the
%   phase FAST by q^3/6 and grows the amplitude by sqrt(1 + q^4/4).  Over TSTOP
%   that is a phase error of TSTOP*w^3*H^2/6 rad and a curve error of
%   TSTOP*w^3*H^2/12 of peak-to-peak.  tolCurve = max(0.5%, TWICE that).
%
%   CROSS-CHECK vs THE RK4 COUNTERPART.  mgs.zarchan.rk4SecondOrder is Listing
%   5.1: same network, same w and unit step, fourth-order.  Structural check 4
%   runs it here and requires both curves inside tolCurve and the RK4 residual
%   no larger than ours.  A self-check, not a bench row -- the book states no
%   number for it -- and it appends NO row at all when that port is off the
%   path.  The traffic already runs the other way too: rk4SecondOrder's row 6
%   scores this listing's 2000 passes from its own local RK2 comparator, so a
%   disagreement lights a row in both files.
%
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED -- this file has never
%   been executed; MATLAB is the caller's. ***  At the defaults:
%     1000 samples, 2000 derivative evaluations
%     y(1.000) exact 0.0295959, ours about 0.0296568 = about +0.21% (band 0.5%,
%       ~2.4x of margin; if it lands nearer 0.5%, something moved -- do not
%       widen the band)
%     max |err| about 6.6e-5 = 0.066% of peak-to-peak (0.1); order fit about 1.98
%     sweep, % of peak-to-peak: h = 0.001 -> 0.066, 0.0025 -> 0.42, 0.005 -> 1.7,
%       0.01 -> 6.6, 0.02 -> about 30.  RK4 at h = 0.001 near 1e-6 %.
%
%   OUTPUTS.  R.t R.y R.yd R.y_cf R.err; R.sweep (.h .rk2, Fig. 5.2); R.bench;
%   R.selfcheck (.names/.oks/.detail -- structural, NOT book values); R.pass
%   (true only if at least one row was SCORED and all passed); R.verdict, ALWAYS
%   FORMED -- 'assert' picks the reaction and nothing else, true errors, false
%   WARNS and still returns R complete, because benchSuite runs every port with
%   assert off and one red row must not be able to hide the board; R.p R.units
%   R.summary.
%
%   See also MGS.ZARCHAN.RK4SECONDORDER (Listing 5.1), MGS.ZARCHAN.BENCHSUITE.
arguments
    opts.XIN   (1,1) double {mustBeFinite} = 1.0                   % step input
    % NOT in Listing 1.1 -- see the header.  Prose value, zarchan.txt:252.
    opts.W     (1,1) double {mustBeFinite, mustBePositive} = 20.0  % rad/s
    opts.TSTOP (1,1) double {mustBeFinite, mustBePositive} = 1.0   % s
    opts.H     (1,1) double {mustBeFinite, mustBePositive} = 0.001 % s, RK2 step
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so an "if nargin == 0" mode here would never fire.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

XIN = opts.XIN;  W = opts.W;  TSTOP = opts.TSTOP;  H = opts.H;

% The listing's own sampling threshold, kept literal.  Passed as an ARGUMENT
% wherever it is used: MATLAB local functions cannot see this workspace.
SREC = 0.000999;

if XIN == 0
    error('mgs:zarchan:secondOrderSim:zeroInput', ...
        ['XIN = 0 makes the exact step response identically zero, so every ' ...
         'row below would divide a residual by zero and report either NaN ' ...
         'or a perfect score.  Refused rather than returned.']);
end
if TSTOP <= 2*H
    error('mgs:zarchan:secondOrderSim:short', ...
        'TSTOP = %g s is not long enough to take two steps at H = %g s.', ...
        TSTOP, H);
end
% Heun has NO stable region on the imaginary axis -- this undamped oscillator
% grows by sqrt(1 + (w*H)^4/4) every step no matter how small H is.  There is
% therefore no "stability limit" to quote; what matters is the TOTAL growth
% over TSTOP.  Refused rather than warned about, because past this point the
% curve is smooth and steadily swelling and reads as a physical resonance
% rather than as the integrator eating the answer.
growth = (1 + (W*H)^4/4)^(TSTOP/H);
if growth > 10
    error('mgs:zarchan:secondOrderSim:unstable', ...
        ['at w = %g rad/s, H = %g s the Heun map grows the oscillation by ' ...
         'a factor %.3g over %g s.  The output would be smooth and entirely ' ...
         'wrong.'], W, H, growth, TSTOP);
end

% =========================================================================
%   LISTING 1.1.  Transcribed from zarchan.txt:255-270.
% =========================================================================
[t, y, yd, nEval] = rk2Run(XIN, W, TSTOP, H, 0.0, 0.0, SREC);
if numel(t) < 2
    % Refused, not warned about.  An empty curve returned quietly looks like a
    % converged result, and a ONE-sample curve is worse: diff() on it is empty
    % and the checks below would append an empty verdict to a logical vector,
    % throwing a "null assignment" error far from the H that caused it.
    error('mgs:zarchan:secondOrderSim:empty', ...
        ['the run recorded %d sample(s) at TSTOP = %g s, H = %g s against ' ...
         'the listing''s literal sampling threshold %g s.'], ...
        numel(t), TSTOP, H, SREC);
end
y_cf = stepResponse(t, XIN, W);

% ---- the Fig. 5.2 step-size sweep ---------------------------------------
% Scaled by w so the sweep means the same thing at any natural frequency: at
% the book's w = 20 this is exactly 0.001 0.0025 0.005 0.01 0.02 s, which
% brackets both step sizes named at zarchan.txt:1417 and :1424.
hs = [0.02 0.05 0.1 0.2 0.4]/W;
hs = hs(hs < TSTOP/4 & (1 + (W*hs).^4/4).^(TSTOP./hs) <= 10);
SW = struct('h', hs, 'rk2', nan(size(hs)));
for i = 1:numel(hs)
    SW.rk2(i) = errFrac(XIN, W, TSTOP, hs(i), SREC);
end

R = struct();
R.t = t;  R.y = y;  R.yd = yd;  R.y_cf = y_cf;  R.err = y - y_cf;
R.sweep = SW;
R.units = ['t [s], w [rad/s]; y is in (input units)*seconds -- w/(s^2+w^2) ' ...
           'has dimensions of time.  NO feet and NO g in this listing.'];
R.p = struct('XIN',XIN, 'W',W, 'TSTOP',TSTOP, 'H',H, 'SREC',SREC, ...
             'n_samples',numel(t), 'n_rhs',nEval, 'growth',growth, ...
             'units',R.units);

R.selfcheck = structuralChecks(R, XIN, W, TSTOP, H, SREC);

if opts.benchmark
    R.bench = benchmarks(R, XIN, W, TSTOP, H, SREC);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine" -- that is the shape of green light this repo has been burned
    % by more than once.
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R); end

% ---- adjudication.  The verdict is FORMED either way; only the reaction to
%      it depends on 'assert'.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated at XIN = %g, ' ...
        'w = %g rad/s, TSTOP = %g s, H = %g s.  An unvalidated port must ' ...
        'not be handed back silently.'], XIN, W, TSTOP, H);
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port ' ...
        'disagrees with ITSELF, which is a wiring fault, not a tolerance ' ...
        'question.  First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
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
        error('mgs:zarchan:secondOrderSim:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN.  R comes back complete so the caller can see WHICH
        % side of a comparison moved; a port that errors here hands back
        % nothing and nobody can tell the book value from the integrator.
        warning('mgs:zarchan:secondOrderSim:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function [t, y, yd, nEval] = rk2Run(XIN, W, TSTOP, H, y0, yd0, SREC)
%RK2RUN  Listing 1.1, zarchan.txt:255-270, verbatim apart from the plotting.
%
%   Every constant this needs is an ARGUMENT.  MATLAB local functions cannot
%   see the main function's workspace; an earlier wave in this package read a
%   constant from the caller and threw on the first line of the first run,
%   which static checking could not see beforehand.
%
%   SREC is the listing's own 0.000999, a parameter only so the one-step
%   structural checks can pass 0 and be sure of their sample: at the book's
%   H = 0.001 the literal threshold fires every step, but a single step at any
%   H < 0.000999 would record nothing at all.

T  = 0.0;
S  = 0.0;
Y  = y0;
YD = yd0;
X  = XIN;
YDD = 0.0;                 % initialised so the FLAG==1 branch cannot read a
                           % value left over from anything else
n = 0;
nmax = ceil(TSTOP/H) + 4;
t = zeros(nmax,1);  y = zeros(nmax,1);  yd = zeros(nmax,1);
nEval = 0;

while T <= (TSTOP - 1e-5)
    YOLD = Y;  YDOLD = YD;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  Pass 1 evaluates the derivative at
    % the current state; pass 2 takes a full Euler step, advances T by H, and
    % re-evaluates there.  T advances INSIDE this loop, exactly once, which is
    % what makes the second evaluation happen at t+H.  Move that line out and
    % the scheme quietly degrades to first order while still running smoothly.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            Y  = Y  + H*YD;
            YD = YD + H*YDD;
            T  = T  + H;
        end
        YDD = W*X - W*W*Y;        % the plant, zarchan.txt:239
        FLAG = 1;
        nEval = nEval + 1;
    end
    % The listing writes "FLAG = 0" here as well.  Dropped: the outer loop
    % re-initialises FLAG on its next pass, so the assignment cannot be read.
    % Kept as a comment rather than as code so nobody re-adds it thinking a
    % state machine was lost.

    % The corrector in the listing's own algebra.  YD on the right of the Y
    % line is the PREDICTED YD, and YDD is the SECOND-pass derivative; this is
    % x_old + (H/2)*(f_old + f_pred) rearranged, i.e. zarchan.txt:231-232.
    Y  = 0.5*(YOLD  + Y  + H*YD);
    YD = 0.5*(YDOLD + YD + H*YDD);

    S = S + H;
    if S >= SREC
        S = 0.0;
        n = n + 1;
        t(n) = T;  y(n) = Y;  yd(n) = YD;
    end
end

t = t(1:n);  y = y(1:n);  yd = yd(1:n);
end


% =============================================================================
function y = stepResponse(t, XIN, W)
%STEPRESPONSE  zarchan.txt:251, the exact step response of w/(s^2 + w^2):
%   y = (XIN/w)*(1 - cos(w*t)).  Exact for ANY XIN and w, which is why rows 2
%   and 3 survive off-book parameters while rows 1, 4 and 6 do not.
y = (XIN/W)*(1 - cos(W*t));
end


% =============================================================================
function f = errFrac(XIN, W, TSTOP, H, SREC)
%ERRFRAC  Max |numeric - exact| over a whole run, as a FRACTION of the exact
%   peak-to-peak 2*XIN/w.
%
%   Normalised by peak-to-peak and not pointwise: the exact solution touches
%   zero once per period, and a pointwise relative error there divides a small
%   number by a smaller one and manufactures a failure out of nothing.
[tt, yy] = rk2Run(XIN, W, TSTOP, H, 0.0, 0.0, SREC);
if isempty(tt)
    f = Inf;                  % refuse to report "perfect" against an empty run
    return
end
f = max(abs(yy - stepResponse(tt, XIN, W))) / (2*abs(XIN)/W);
end


% =============================================================================
function K = structuralChecks(R, XIN, W, TSTOP, H, SREC)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never evidence that
%   this port reproduces Zarchan -- only the weaker claim that it agrees with
%   itself, which is what catches wiring faults.
K = struct();
names = {};  oks = [];  detail = {};

q        = W*H;
tolCurve = max(0.005, TSTOP*W^3*H^2/6);     % see BANDS in the header
pp       = 2*abs(XIN)/W;

% 1  ONE STEP == THE EXACT HEUN MAP.  The sharp one.  At XIN = 0 the plant is
%    the pure oscillator ydd = -w^2*y, for which Heun is EXACTLY I + hA +
%    (hA)^2/2 with A = [0 1; -w^2 0].  (hA)^2 = -q^2*I collapses that to
%    [1-q^2/2, h; -w^2*h, 1-q^2/2]: one step from (1,0) must land on
%    (1-q^2/2, -w^2*h) and one from (0,1) on (h, 1-q^2/2).
%    ALSO THE DELETE EXPERIMENT for the second pass -- drop it and the map is
%    plain Euler, I + hA, which differs in the (1,1) entry by q^2/2 = 2e-4 at
%    the book's settings against a band of 1e-12.
%    TSTOP1 = max(H, 1e-5), not H: the loop test is "T <= TSTOP - 1e-5", so at
%    H <= 1e-5 a TSTOP of H takes no step and this check would report Inf on a
%    good run.  SREC = 0 makes the single step certain to be recorded.
TSTOP1 = max(H, 1e-5);
[~, ya, yda] = rk2Run(0.0, W, TSTOP1, H, 1.0, 0.0, 0.0);
[~, yb, ydb] = rk2Run(0.0, W, TSTOP1, H, 0.0, 1.0, 0.0);
if isscalar(ya) && isscalar(yb)
    e11 = abs(ya(1)  - (1 - q^2/2));
    e21 = abs(yda(1) - (-W*W*H))/max(W*W*H, realmin);
    e12 = abs(yb(1)  - H)/max(H, realmin);
    e22 = abs(ydb(1) - (1 - q^2/2));
    K.heun_map_resid = max([e11 e21 e12 e22]);
else
    K.heun_map_resid = Inf;
end
names{end+1}  = 'one step == I + hA + (hA)^2/2 (all four entries)';
oks(end+1)    = K.heun_map_resid <= 1e-12;
detail{end+1} = sprintf('max residual %.3e (band 1e-12)', K.heun_map_resid);

% 2  THE EQUILIBRIUM IS EXACTLY PRESERVED.  y = XIN/w, yd = 0 makes
%    ydd = w*XIN - w^2*(XIN/w) = 0 identically, so a correct integrator sits
%    there forever.  Catches a w vs w^2 swap and a sign flip on the w^2*y term,
%    both of which otherwise leave a smooth oscillation at the wrong frequency.
scale = abs(XIN)/W;
[~, yE, ydE] = rk2Run(XIN, W, TSTOP, H, XIN/W, 0.0, SREC);
K.equil_resid = max(max(abs(yE - XIN/W))/scale, max(abs(ydE))/(scale*W));
names{end+1}  = 'equilibrium y = XIN/w is a fixed point';
oks(end+1)    = K.equil_resid <= 1e-9;
detail{end+1} = sprintf('max normalised drift %.3e (band 1e-9)', K.equil_resid);

% 3  THE RECORDED CURVE IS THE MAP'S OWN ORBIT.  In (u,v) = (w*(y - XIN/w), yd)
%    the Heun map is exactly a scaled rotation, so the discrete energy u^2+v^2
%    grows by EXACTLY (1 + q^4/4) per step: E(n) = E(0)*(1+q^4/4)^n, n = t/H.
%    Check 1 pins the MAP; this pins the RECORD to it -- an off-by-one in the
%    sampling block, or storing YOLD instead of Y, moves this and nothing else.
u    = W*(R.y - XIN/W);
E    = u.^2 + R.yd.^2;
nIdx = round(R.t/H);
% E(0) = (w*(y0 - XIN/w))^2 + yd0^2 = XIN^2, since the listing starts from
% y = yd = 0.  The forcing drops out exactly: Heun is affine, so z = y - XIN/w
% obeys the homogeneous oscillator step for step.
Epred = XIN^2 * (1 + q^4/4).^nIdx;
K.energy_resid = max(abs(E./Epred - 1));
names{end+1}  = 'discrete energy tracks (1 + q^4/4)^n exactly';
oks(end+1)    = K.energy_resid <= 1e-9;
detail{end+1} = sprintf('max |E/Epred - 1| = %.3e over %d samples (band 1e-9)', ...
    K.energy_resid, numel(E));

% 4  CROSS-CHECK AGAINST THE RK4 COUNTERPART, Listing 5.1.  Same network, same
%    settings, an independent file and an independent scheme.  Appended only
%    when that port is on the path -- a check that could not run must not
%    occupy a row that reads as a pass.
K.rk4_ours = max(abs(R.err))/pp;
K.rk4_sib  = NaN;
K.rk4_err  = '';
if ~isempty(which('mgs.zarchan.rk4SecondOrder'))
    % 'benchmark' false keeps this cheap (no step-size sweep of its own) and
    % keeps a failure of THAT port's rows out of THIS port's verdict.  It warns
    % when benchmarking is off, by design, so the warning is silenced for the
    % duration and the caller's warning state is put back exactly as found.
    ws = warning('off', 'mgs:zarchan:rk4SecondOrder:benchmark');
    try
        S4 = mgs.zarchan.rk4SecondOrder('XIN',XIN, 'W',W, 'TSTOP',TSTOP, ...
            'H',H, 'benchmark',false, 'render',false, 'plot',false, ...
            'assert',false);
        K.rk4_sib = max(abs(S4.err))/pp;
    catch ME
        K.rk4_sib = NaN;
        K.rk4_err = ME.message;
    end
    warning(ws);
    names{end+1}  = 'agrees with mgs.zarchan.rk4SecondOrder (Listing 5.1)';
    oks(end+1)    = isfinite(K.rk4_sib) && K.rk4_sib <= K.rk4_ours && ...
                    K.rk4_ours <= tolCurve;
    detail{end+1} = sprintf(['RK2 %.4g%% of p-p, RK4 %.4g%% (band: RK4 no ' ...
        'worse than RK2, RK2 within %.4g%%)'], 100*K.rk4_ours, ...
        100*K.rk4_sib, 100*tolCurve);
end

% 5  THE TIME GRID.  S accumulates by H and records at S >= 0.000999, so the
%    interval is the smallest multiple of H reaching the threshold -- H itself
%    at the book's H = 0.001.  This is the row that notices the DOUBLE
%    "S = S + H" the same author wrote into Listing 4.6, which halves the
%    interval while plotting perfectly smoothly.  Normalised by the interval so
%    the band means the same thing at any H.
recStep     = ceil(SREC/H)*H;
K.rec_step  = recStep;
K.dt_resid  = max(abs(diff(R.t) - recStep))/recStep;
K.end_resid = abs(R.t(end) - TSTOP);
names{end+1}  = 'time grid uniform at the listing''s record interval';
oks(end+1)    = K.dt_resid <= 1e-9 && K.end_resid <= recStep + 1e-12;
detail{end+1} = sprintf(['%d samples, interval %g s (= %g x H), max |dt-int|/int ' ...
    '= %.2e, |t(end)-TSTOP| = %.2e s'], numel(R.t), recStep, ...
    round(recStep/H), K.dt_resid, K.end_resid);

K.names  = names;
K.oks    = logical(oks);
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
function B = benchmarks(R, XIN, W, TSTOP, H, SREC)
%BENCHMARKS  Computed vs published, with the PROVENANCE of every band.
%
%   *** EVERY BOOK VALUE WAS GREPPED OUT OF zarchan.txt AND THE LINE NUMBER IS
%   IN .source. ***  Nothing was read off a figure -- the extraction turned
%   Figs. 1.2 and 5.2 into image links -- but prose ABOUT a figure is greppable,
%   and an ordering check is stronger than a plotted value.  Bands: 'analytic'
%   = the derived Heun error (header); 'exact' = a printed integer; 'shape' =
%   an ordering from prose.  Do NOT widen one to turn a row green.

B = emptyBench();
tolCurve = max(0.005, TSTOP*W^3*H^2/6);
pp       = 2*abs(XIN)/W;

% Rows 1 and 4 depend only on the clock (TSTOP, H); row 6 quotes "this
% example", which is w = 20 rad/s over 1 s with a unit step.  Off those cases
% they SKIP, never pass.
stepCase = abs(TSTOP - 1) < 1e-12 && abs(H - 0.001) < 1e-15;
bookCase = stepCase && abs(W - 20) < 1e-12 && abs(XIN - 1) < 1e-12;
whyStep  = sprintf(['off the book''s own clock -- ":253" is 1 s at ' ...
    'h = 0.001 s; this run used TSTOP = %g s, H = %g s'], TSTOP, H);
whyCase  = sprintf(['off the book''s own example -- Fig. 5.2 is w = 20 ' ...
    'rad/s, unit step, 1 s at h = 0.001 s; this run used w = %g rad/s, ' ...
    'XIN = %g, TSTOP = %g s, H = %g s'], W, XIN, TSTOP, H);

% ---- 1.  the printed derivative-pass count ------------------------------
if stepCase
    B = addVal(B, 'derivative-equation passes', 2000, R.p.n_rhs, 'exact', ...
        1e-9, 'p.7 text (zarchan.txt:253)', 'evaluations', ...
        ['"2000 passes are made to the differential equations" = 1000 steps ' ...
         'x 2.  Exact integer, no tolerance to set.  The -1e-5 in the loop ' ...
         'test is what makes it exact and not 1000-or-1001; Listing 5.1 ' ...
         'drops that guard and pays for it']);
else
    B = addSkip(B, 'derivative-equation passes', '2000', whyStep, ...
        'p.7 text (zarchan.txt:253)', 'evaluations');
end

% ---- 2.  the endpoint against the printed closed form -------------------
cfEnd = stepResponse(R.t(end), XIN, W);
if abs(cfEnd) < 0.05*pp
    B = addSkip(B, 'y at t = TSTOP vs (XIN/w)(1 - cos w t)', ...
        sprintf('%.6g', cfEnd), ...
        sprintf(['exact solution %.3g at t = %.6f s, under 5%% of peak-to-' ...
                 'peak: it crosses zero once per period and a pointwise ' ...
                 'relative error there invents a failure.  Row 3 covers the ' ...
                 'whole curve and is unaffected'], cfEnd, R.t(end)), ...
        'p.5 closed form (zarchan.txt:251, w at :252)', '(input units)*s');
else
    B = addVal(B, 'y at t = TSTOP vs (XIN/w)(1 - cos w t)', cfEnd, ...
        R.y(end), 'analytic', tolCurve, ...
        'p.5 closed form (zarchan.txt:251, w at :252)', '(input units)*s', ...
        sprintf(['sampled at the ACTUAL t = %.6f s.  Heun runs the phase ' ...
                 'FAST by TSTOP*w^3*H^2/6 = %.4g rad, so this residual is ' ...
                 'expected POSITIVE; band max(0.5%%, %.4g%%)'], ...
                R.t(end), TSTOP*W^3*H^2/6, 100*tolCurve));
end

% ---- 3.  "agrees exactly with the closed-form solution" -----------------
r3 = max(abs(R.err))/pp;
B = addBound(B, 'whole curve vs the closed form', ...
    sprintf('<= %.4f%% of peak-to-peak', 100*tolCurve), ...
    sprintf('%.5g%% of peak-to-peak', 100*r3), r3 <= tolCurve, ...
    'p.7 text (zarchan.txt:253) with :251', '(input units)*s', ...
    ['row 2 is one sample; this is every sample.  "Agrees exactly" is the ' ...
     'book''s phrase for Fig. 1.2, so the band has to come from the scheme ' ...
     'rather than from the sentence -- see BANDS in the header']);

% ---- 4.  the recorded sample count, and the book against itself ---------
if stepCase
    B = addVal(B, 'recorded samples', 1000, numel(R.t), 'exact', 1e-9, ...
        'Listing 1.1''s own sampling test (zarchan.txt:264) vs prose :252', ...
        'samples', ...
        ['*** THE BOOK CONTRADICTS ITSELF. ***  :252 says "every 0.01 s we ' ...
         'print out the output" = 100 samples; the listing tests "S >= ' ...
         '.000999" against an increment of H = 0.001, fires every step and ' ...
         'records 1000.  The LISTING is scored -- it is what gets ported.  ' ...
         'See the header for why .00999 is not silently substituted']);
else
    B = addSkip(B, 'recorded samples', '1000', whyStep, ...
        'Listing 1.1''s own sampling test (zarchan.txt:264) vs prose :252', ...
        'samples');
end

% ---- 5.  measured order of accuracy = 2 ---------------------------------
% The defining property of the recursion printed at zarchan.txt:231-232.
% Fit points scaled by w so they sit in the same regime at any frequency.
% Assigned unconditionally: a variable that exists only inside an "if" is a
% variable that is undefined on the branch nobody tested.
ho = [0.05 0.1 0.2]/W;
ho = ho(ho < TSTOP/4 & (1 + (W*ho).^4/4).^(TSTOP./ho) <= 10);
if W*TSTOP >= 6 && numel(ho) == 3
    eo = zeros(size(ho));
    for i = 1:numel(ho)
        eo(i) = errFrac(XIN, W, TSTOP, ho(i), SREC);
    end
    if all(eo > 0) && all(isfinite(eo))
        lx = log(ho);  ly = log(eo);
        ordr = sum((lx-mean(lx)).*(ly-mean(ly)))/sum((lx-mean(lx)).^2);
    else
        ordr = NaN;
    end
    B = addVal(B, 'measured order of accuracy', 2.0, ordr, 'analytic', 0.075, ...
        'p.4 recursion (zarchan.txt:231-232)', 'decades/decade', ...
        sprintf(['least-squares slope of log(max error) on log(h) at h = ' ...
                 '%.4g, %.4g, %.4g s.  Band 0.075 relative = +/-0.15 ' ...
                 'absolute; expect a shade under 2, the O(h^3) amplitude ' ...
                 'error drags the fit down'], ho(1), ho(2), ho(3)));
else
    B = addSkip(B, 'measured order of accuracy', '2', ...
        sprintf(['w*TSTOP = %.3g rad over %g s leaves no room for three fit ' ...
                 'points inside TSTOP/4; a slope fitted here describes the ' ...
                 'transient, not the scheme'], W*TSTOP, TSTOP), ...
        'p.4 recursion (zarchan.txt:231-232)', 'decades/decade');
end

% ---- 6.  "h = 0.01 s is about the largest ... 0.02 s misses the bandwidth"
if bookCase && numel(R.sweep.h) >= 4
    e = R.sweep.rk2;
    mono = all(diff(e) > 0);
    [~, i01] = min(abs(R.sweep.h - 0.01));
    [~, i02] = min(abs(R.sweep.h - 0.02));
    ratio = e(i02)/max(e(i01), realmin);
    B = addShape(B, 'error grows with h; h = 0.02 s is past the ceiling', ...
        ['max error monotone increasing over h = 0.001 .. 0.02 s, and ' ...
         'h = 0.02 s at least twice as bad as the stated ceiling h = 0.01 s'], ...
        sprintf('%s; 0.02 s is %.1fx the error at 0.01 s (%.3g%% vs %.3g%% of p-p)', ...
                monoStr(mono), ratio, 100*e(i02), 100*e(i01)), ...
        mono && ratio >= 2, 'p.91 text (zarchan.txt:1417 and :1424)', ...
        ['an ORDERING, because the magnitudes behind those sentences live ' ...
         'on Fig. 5.2 and became an image link.  The book''s own rule of ' ...
         'thumb -- interval five times under the 0.05 s time constant ' ...
         '(1/20), :1417 -- is the same statement; h^2 puts the ratio near 4']);
else
    B = addSkip(B, 'error grows with h; h = 0.02 s is past the ceiling', ...
        '"missing the effect of the system''s high bandwidth"', whyCase, ...
        'p.91 text (zarchan.txt:1417 and :1424)', '-');
end

% ---- INFO rows.  Reported, never scored. --------------------------------
B = addInfo(B, 'w is NOT printed in Listing 1.1', '--', ...
    sprintf('w = %g rad/s taken from prose', W), ...
    'prose zarchan.txt:252, corroborated by Listing 5.1 at :1448', ...
    ['NOT SCORED: YDD = W*X - W*W*Y reads a variable the printed program ' ...
     'never assigns, so our w IS the book''s w by construction and there is ' ...
     'nothing to compare.  On the board so the gap is visible instead of ' ...
     'buried in a default.  :252 gives 20 rad/s; Listing 5.1, the same ' ...
     'network, prints "XIN=1.; W=20." at :1448']);

B = addInfo(B, 'step-size sweep behind Fig. 5.2', '--', sweepStr(R.sweep), ...
    'Fig. 5.2 -- shape only (zarchan.txt:1416)', ...
    ['NOT SCORED: Fig. 5.2 is a curve and the book prints no value off it.  ' ...
     'Row 6 scores the ORDERING the surrounding prose states; a magnitude ' ...
     'read off the figure would be invented']);
end


function s = monoStr(tf)
if tf
    s = 'monotone increasing';
else
    s = 'NOT monotone';
end
end

function s = sweepStr(S)
s = '';
for i = 1:numel(S.h)
    s = [s sprintf('h=%.4g: %.3g%% p-p;  ', S.h(i), 100*S.rk2(i))]; %#ok<AGROW>
end
if isempty(s)
    s = '(no step size in the sweep was usable at this TSTOP)';
end
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw "Subscripted assignment between dissimilar
%   structures" -- which is the behaviour we want.  This is the schema
%   mgs.zarchan.benchSuite's adaptGeneric consumes, and it is STRICT.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
% Tolerance arrives EXPLICITLY rather than through a band->tol lookup, because
% the analytic band is built from H at the call site and a lookup table would
% need a second copy of that arithmetic.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.6g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.7g', ours);
B(i).pct  = NaN;   B(i).band = band;  B(i).tol = tol;
B(i).ok   = false; B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = units;  B(i).detail = detail;
if ~isfinite(ours) || book == 0
    B(i).skipped = true;
    B(i).why = 'the computed value is not finite (or the book value is zero)';
    return
end
B(i).pct = 100*(ours/book - 1);
B(i).ok  = abs(ours/book - 1) <= tol;
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
B(i).ok   = false; B(i).skipped = true;  B(i).why = why;
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
B(i).ok   = false; B(i).skipped = true;  B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end


% =============================================================================
function s = render(R, ev)
L = {};
p = R.p;
L{end+1} = 'Zarchan Listing 1.1 -- SECOND-ORDER SYSTEM, SECOND-ORDER RUNGE-KUTTA';
L{end+1} = sprintf('   %s', R.units);
L{end+1} = sprintf(['   XIN = %g   w = %g rad/s   TSTOP = %g s   H = %g s   ' ...
    '(w*H = %.3g)   %d samples, %d derivative evaluations'], ...
    p.XIN, p.W, p.TSTOP, p.H, p.W*p.H, p.n_samples, p.n_rhs);
L{end+1} = sprintf(['   vs exact y = (XIN/w)(1-cos w t):  y(end) = %.8f, ' ...
    'exact %.8f, max |err| = %.3e'], R.y(end), R.y_cf(end), max(abs(R.err)));
L{end+1} = sprintf('   w NOT printed in Listing 1.1 -- prose value, zarchan.txt:252');
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-50s %s', vstr(K.oks(i)), K.names{i}, ...
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
                L{end+1} = sprintf('   INFO %-50s %s', b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case 'skip'
                L{end+1} = sprintf('   SKIP %-50s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case {'shape','bound'}
                L{end+1} = sprintf('   %-4s %-50s %s', vstr(b.ok), b.what, ...
                    b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf(['   %-4s %-50s book %s %s / ours %s %s ' ...
                    '(%+.4f%%, band %.4g%%)'], vstr(b.ok), b.what, ...
                    b.book_str, b.units, b.ours_str, b.units, b.pct, ...
                    100*b.tol); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped/info -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: this is the integrator the REST OF THE BOOK runs on';
L{end+1} = '   (zarchan.txt:1415).  A fault here would move every other port in';
L{end+1} = '   this package the same way, and they would still agree with each';
L{end+1} = '   other.  That is why this listing gets checked against an EXACT';
L{end+1} = '   solution rather than against a sibling.';
s = strjoin(L, newline);
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
%DRAWCURVES  Base MATLAB graphics only.  The listing's own figure, plus the
%   step-size sweep that is the actual point of Fig. 5.2.
figure;
plot(R.t, R.y, '-', R.t, R.y_cf, '--');  grid on;
xlabel('Time (Sec)');  ylabel('y');
title('Listing 1.1: second-order Runge-Kutta, second-order network');
legend('Listing 1.1 RK2', 'exact (XIN/w)(1-cos wt)', 'Location', 'best');

S = R.sweep;
if numel(S.h) >= 2
    figure;
    loglog(S.h, 100*S.rk2, 'o-');  grid on;
    xlabel('Integration step size h (s)');
    ylabel('Max error (% of peak-to-peak)');
    title('Fig. 5.2: increasing the step size degrades accuracy');
end
end
