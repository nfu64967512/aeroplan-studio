function R = rk4SecondOrder(opts)
%MGS.ZARCHAN.RK4SECONDORDER  Zarchan Listing 5.1 -- the second-order network of
%   Fig. 1.1, integrated with FOURTH-order Runge-Kutta.
%
%   R = mgs.zarchan.rk4SecondOrder()
%   R = mgs.zarchan.rk4SecondOrder('H', 0.02)          the book's Fig. 5.3 case
%   R = mgs.zarchan.rk4SecondOrder('W', 10, 'assert', false)
%
%   Line-by-line port of Listing 5.1 (zarchan.txt:1442-1456).  The plant is the
%   sinusoidal network Y/X = w/(s^2 + w^2), i.e. ydd = w*x - w^2*y with
%   x = XIN (a step) and y(0) = yd(0) = 0, whose EXACT step response the book
%   prints at zarchan.txt:251 as  y = (XIN/w)*(1 - cos(w*t)).  So every row
%   below is scored against a closed form, never off a figure.
%
%   UNITS: t in SECONDS, w in RAD/S, y in (input units)*seconds -- w/(s^2+w^2)
%   has dimensions of time.  This listing carries NONE of the book's ft/g
%   machinery: no gravity, no length, no 32.2.  There is nothing to convert.
%
%   WHY THIS ONE IS HERE.  Chapter 5 switches integrators.  Every other listing
%   uses the second-order Runge-Kutta of Listing 1.1, and zarchan.txt:1415 says
%   why that is not enough: when the covariance equations are solved
%   numerically, higher order methods are required.  Listing 5.1 is the
%   demonstration, on a plant whose answer is known exactly, that buys the
%   right to use RK4 in Listing 5.2.  This file is therefore the INTEGRATOR
%   REFERENCE for mgs.zarchan.covarianceHoming; its job is to pin the scheme.
%
%   *** WHAT IT CANNOT SAY. ***  The right-hand side has NO EXPLICIT TIME
%   DEPENDENCE -- ydd depends on y alone, because XIN is constant, and the
%   listing computes TNEW = T + 0.5*H at the K1/K2 stages then never passes it
%   to anything.  So this port exercises RK4's STATE path four times per step
%   and its TIME path zero times.  Listing 5.2's covariance right-hand side IS
%   time-varying (Vc*(tF-t) in a denominator), so a pass here is necessary but
%   NOT sufficient for that one.  The unused stage times are kept in the code,
%   exactly as the book has them, so the gap stays visible.
%
%   THE TRANSCRIPTION.  listing_05_01.m reports no unresolved
%   variable-to-variable comparisons, and that is right: every comparison in
%   5.1 is against a literal.  Four notes, none of them a judgement call:
%     1  zarchan.txt:1447 reads "while ~(T .= 1.)"; the extractor already
%        repaired "." -> ">=".  Shape-forced: T only increases, so "<=" never
%        terminates and "==" never fires.
%     2  5.1 has NO -1e-5 stop guard, unlike Listing 1.1 ("while T ,=(1.-1e-5)"
%        -> "<=").  FRAGILE: whether the last step is taken depends on float
%        accumulation.  Kept verbatim and DEFUSED rather than tidied -- every
%        row evaluates the closed form at the ACTUAL recorded t, so an extra or
%        missing final sample cannot move a residual, and a structural row
%        reports the count and the spacing.
%     3  "S=S+H" is DEAD CODE in 5.1: it records every step via "count" and
%        never reads S.  Dropped, and said so, so nobody re-adds a sampling
%        test this listing does not have.
%     4  "save datfil.txt output /ascii" is the -ascii typo Listing 4.6 also
%        carries.  Nothing is written to disk by this port at all.
%   Deliberate: K0..K3 stay zeros([1,6]) with only columns 1 and 2 touched --
%   Listing 5.2's six-state machinery around a two-state problem, which is the
%   evidence that 5.1 is 5.2's reduced case.
%
%   BENCHMARKS -- every book value grepped from zarchan.txt, line number in the
%   row's .source.  Figs. 5.2 and 5.3 became image links in extraction, so the
%   only thing they can supply is the PROSE about them, scored as an ordering.
%     1  y(TSTOP) vs the printed closed form (:251, w = 20 rad/s at :252).
%        Band max(0.5%, (w*H)^4): the second term IS this scheme's error order,
%        so the band means the same thing at every H; the floor keeps a refined
%        H from eventually punishing a correct run.
%     2  the WHOLE curve against it, as a fraction of peak-to-peak 2*XIN/w.
%     3  RK4 at h = 0.02 s still tracks it -- :1438, "a step size of 0.02 s was
%        adequate for getting the correct solution".
%     4  the book's OWN second-order scheme at that same h does not -- :1424,
%        at or above 0.02 s "we will be missing the effect of the system's high
%        bandwidth".  A COMPARISON, so scored as one (RK2 >= 10x RK4).
%     5  measured order of accuracy is 4 -- the defining property of the
%        recursion printed at :1427-1429.
%     6  Listing 1.1 makes 2000 derivative passes at h = 0.001 s over 1 s
%        (:253).  Exact integer, and the only row tying the RK2 comparator that
%        rows 3 and 4 lean on to a printed number.
%   NOT SCORED (INFO): the step-size sweeps behind Figs. 5.2/5.3.  The book
%   publishes those as curves and prints no value off either one.
%
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED. ***  This file has
%   never been executed -- MATLAB is the caller's.  From the two schemes'
%   amplification factors on an undamped oscillator, R_RK4(iq) = 1 + iq - q^2/2
%   - iq^3/6 + q^4/24 and R_RK2(iq) = 1 + iq - q^2/2, q = w*h:
%     closed form at w = 20, XIN = 1, t = 1 s:  y = 0.02959590
%     row 1 at H = 0.01: about -0.041%  (band 0.5%)
%     row 2 at H = 0.01: about 0.014% of peak-to-peak  (band 0.2%)
%     row 3 at h = 0.02: about 0.22% of peak-to-peak   (band 2%)
%     row 4: RK2 at h = 0.02 near 29% of peak-to-peak, about 130x row 3
%     row 5: order 4.00 to two decimals
%     max error, % of peak-to-peak, at h = 0.0025 0.005 0.01 0.02 0.05 s:
%        RK4   1.3e-6   8.3e-4   0.0135   0.225   7.3
%        RK2   0.067    1.67     6.8      29      ~500 (diverged)
%   A residual near a band EDGE is not a pass to be accepted; the gap between
%   "about 0.014%" and "0.2%" is the entire safety margin.  Do not widen a band
%   to make a row green.  No random numbers here, so there is no seed.
%
%   OUTPUTS: R.t R.y R.yd (the listing's ArrayT/ArrayY), R.y_cf the exact step
%   response at those same t, R.err, R.sweep (.h .rk4 .rk2), R.bench,
%   R.selfcheck, R.pass (true only if a row was SCORED and all passed),
%   R.verdict (ALWAYS formed; 'assert' picks the reaction and nothing else --
%   true errors, false WARNS and still returns R complete, which is how
%   benchSuite runs every port), R.p, R.units, R.summary.
%
%   See also MGS.ZARCHAN.COVARIANCEHOMING (Listing 5.2, the reason this
%   integrator was adopted), MGS.ZARCHAN.BENCHSUITE.

arguments
    opts.XIN   (1,1) double {mustBeFinite} = 1.0                   % step input
    opts.W     (1,1) double {mustBeFinite, mustBePositive} = 20.0  % rad/s
    opts.TSTOP (1,1) double {mustBeFinite, mustBePositive} = 1.0   % s
    opts.H     (1,1) double {mustBeFinite, mustBePositive} = 0.01  % s, RK4 step
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so an "if nargin == 0" mode here would never fire.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

XIN = opts.XIN;  W = opts.W;  TSTOP = opts.TSTOP;  H = opts.H;

if XIN == 0
    error('mgs:zarchan:rk4SecondOrder:zeroInput', ...
        ['XIN = 0 makes the exact step response identically zero, so every ' ...
         'row below would divide a residual by zero and report either NaN ' ...
         'or a perfect score.  Refused rather than returned.']);
end
if TSTOP <= 2*H
    error('mgs:zarchan:rk4SecondOrder:short', ...
        'TSTOP = %g s is not long enough to take two steps at H = %g s.', ...
        TSTOP, H);
end
if W*H >= 2.8284271
    % Past RK4's imaginary-axis stability limit |R(i*q)| <= 1 at q = 2*sqrt(2).
    % Refused rather than warned about: beyond it the scheme AMPLIFIES this
    % undamped oscillator every step and returns a smooth, steadily growing
    % curve that reads as a physical instability rather than a numerical one.
    error('mgs:zarchan:rk4SecondOrder:unstable', ...
        ['w*H = %g is at or past RK4''s imaginary-axis stability limit ' ...
         '2*sqrt(2) = 2.8284.  The output would be smooth and entirely ' ...
         'wrong.'], W*H);
end

% =========================================================================
%   LISTING 5.1.  Transcribed from zarchan.txt:1444-1456.
% =========================================================================
[t, y, yd, nEvalRK4] = rk4Run(XIN, W, TSTOP, H, 0.0, 0.0);
if numel(t) < 2
    error('mgs:zarchan:rk4SecondOrder:empty', ...
        ['the run recorded %d sample(s) at TSTOP = %g s, H = %g s.  At ' ...
         'least two are needed for any curve comparison to mean anything.'], ...
        numel(t), TSTOP, H);
end
y_cf = stepResponse(t, XIN, W);

% ---- the Fig. 5.2 / Fig. 5.3 step-size sweep, both schemes ---------------
% Scaled by w so the sweep means the same thing at any natural frequency: at
% the listing's w = 20 this is exactly 0.0025 0.005 0.01 0.02 0.05 s.
hs = [0.05 0.1 0.2 0.4 1.0]/W;
hs = hs(hs < TSTOP/4);
SW = struct('h', hs, 'rk4', nan(size(hs)), 'rk2', nan(size(hs)));
for i = 1:numel(hs)
    SW.rk4(i) = peakFrac(XIN, W, TSTOP, hs(i), true);
    SW.rk2(i) = peakFrac(XIN, W, TSTOP, hs(i), false);
end

R = struct();
R.t = t;  R.y = y;  R.yd = yd;  R.y_cf = y_cf;  R.err = y - y_cf;
R.sweep = SW;
R.units = ['t [s], w [rad/s]; y is in (input units)*seconds -- w/(s^2+w^2) ' ...
           'has dimensions of time.  NO feet and NO g in this listing.'];
R.p = struct('XIN',XIN, 'W',W, 'TSTOP',TSTOP, 'H',H, 'n_steps',numel(t), ...
             'n_rhs',nEvalRK4, 'units',R.units);

R.selfcheck = structuralChecks(R, XIN, W, TSTOP, H);

if opts.benchmark
    R.bench = benchmarks(R, XIN, W, TSTOP, H);
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
        'w = %g rad/s, TSTOP = %g s.  An unvalidated port must not be ' ...
        'handed back silently.'], XIN, W, TSTOP);
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
        error('mgs:zarchan:rk4SecondOrder:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN.  R comes back complete so the caller can see WHICH
        % side of a comparison moved.  A port that errors here hands back
        % nothing, and then nobody can tell the book value from the integrator.
        warning('mgs:zarchan:rk4SecondOrder:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function [t, y, yd, nEval] = rk4Run(XIN, W, TSTOP, H, y0, yd0)
%RK4RUN  Listing 5.1, zarchan.txt:1444-1456.
%
%   Every constant this needs is an ARGUMENT.  MATLAB local functions cannot
%   see the main function's workspace; an earlier wave in this package read a
%   constant from the caller and threw on the first line of the first run,
%   which static checking could not see beforehand.
%
%   K0..K3 are the listing's zeros([1,6]) with only columns 1 and 2 used.

K0 = zeros([1,6]);  K1 = zeros([1,6]);  K2 = zeros([1,6]);  K3 = zeros([1,6]);
count = 0;
Y = y0;  YD = yd0;  T = 0.0;  TNEW = 0.0;
nmax = ceil(TSTOP/H) + 4;
t = zeros(nmax,1);  y = zeros(nmax,1);  yd = zeros(nmax,1);
nEval = 0;

% The listing's own stop test, WITHOUT Listing 1.1's -1e-5 guard.  Verbatim,
% fragility included; the header says how that fragility is defused.
while ~(T >= TSTOP)
    YOLD = Y;  YDOLD = YD;  STEP = 1;
    while ((STEP == 1) || (STEP == 2) || (STEP == 3))
        YDD = W*XIN - W*W*Y;          % the plant.  No explicit t dependence.
        nEval = nEval + 1;
        if (STEP == 1)
            STEP = 2;
            K0(1,1) = YD;  K0(1,2) = YDD;
            TNEW = T + 0.5*H;         % computed by the book, never fed to f
            Y  = YOLD  + 0.5.*H.*K0(1,1);
            YD = YDOLD + 0.5.*H.*K0(1,2);
        elseif (STEP == 2)
            STEP = 3;
            K1(1,1) = YD;  K1(1,2) = YDD;
            TNEW = T + 0.5*H;
            Y  = YOLD  + 0.5.*H.*K1(1,1);
            YD = YDOLD + 0.5.*H.*K1(1,2);
        else
            STEP = 4;
            K2(1,1) = YD;  K2(1,2) = YDD;
            TNEW = T + H;
            Y  = YOLD  + H.*K2(1,1);
            YD = YDOLD + H.*K2(1,2);
        end
    end
    YDD = W*XIN - W*W*Y;              % the fourth evaluation, at t + H
    nEval = nEval + 1;
    K3(1,1) = YD;  K3(1,2) = YDD;
    T  = TNEW;
    Y  = YOLD  + H.*(K0(1,1) + 2.*(K1(1,1)+K2(1,1)) + K3(1,1))./6;
    YD = YDOLD + H.*(K0(1,2) + 2.*(K1(1,2)+K2(1,2)) + K3(1,2))./6;
    count = count + 1;
    t(count) = T;  y(count) = Y;  yd(count) = YD;
end

t = t(1:count);  y = y(1:count);  yd = yd(1:count);
end


% =============================================================================
function [t, y, yd, nEval] = rk2Run(XIN, W, TSTOP, H, y0, yd0)
%RK2RUN  Listing 1.1, zarchan.txt:256-258 -- the SECOND-order Runge-Kutta this
%   chapter is arguing against.  It is here only as the comparator for the
%   Fig. 5.2/5.3 rows; without it, "0.02 s was adequate" has nothing to be
%   adequate compared to.
%
%   Its stop test DOES carry the -1e-5 guard, which is why its step count is
%   exact and Listing 5.1's is not.  Listing 1.1's sampling test
%   ("if S >= .000999" at H = 0.001) fires every step, so every step is kept.

Y = y0;  YD = yd0;  T = 0.0;
count = 0;
nmax = ceil(TSTOP/H) + 4;
t = zeros(nmax,1);  y = zeros(nmax,1);  yd = zeros(nmax,1);
nEval = 0;
YDD = W*XIN - W*W*Y;

while T <= (TSTOP - 1e-5)
    YOLD = Y;  YDOLD = YD;  STEP = 1;  FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            Y  = Y  + H*YD;
            YD = YD + H*YDD;
            T  = T  + H;
        end
        YDD = W*XIN - W*W*Y;
        nEval = nEval + 1;
        FLAG = 1;
    end
    % ORDER MATTERS: Y's corrector uses the PREDICTED YD, and YD's uses the
    % derivative at the predicted Y.  Swapping these two lines is a silent
    % change of scheme that still runs smoothly.
    Y  = 0.5*(YOLD  + Y  + H*YD);
    YD = 0.5*(YDOLD + YD + H*YDD);
    count = count + 1;
    t(count) = T;  y(count) = Y;  yd(count) = YD;
end

t = t(1:count);  y = y(1:count);  yd = yd(1:count);
end


% =============================================================================
function y = stepResponse(t, XIN, W)
%STEPRESPONSE  zarchan.txt:251, the exact step response of w/(s^2 + w^2):
%   y = (XIN/w)*(1 - cos(w*t)).  Exact for ANY XIN and w, which is why rows 1,
%   2 and 5 survive off-book parameters while rows 3 and 4 do not.
y = (XIN/W)*(1 - cos(W*t));
end


% =============================================================================
function f = peakFrac(XIN, W, TSTOP, H, useRK4)
%PEAKFRAC  Max |numeric - exact| over a whole run, as a FRACTION of the exact
%   peak-to-peak 2*XIN/w.
%
%   Normalised by peak-to-peak and not pointwise: the exact solution touches
%   zero once per period, and a pointwise relative error there divides a small
%   number by a smaller one and manufactures a failure out of nothing.
%
%   useRK4 is an EXPLICIT flag.  Never nargin: see the arguments block.
if useRK4
    [tt, yy] = rk4Run(XIN, W, TSTOP, H, 0.0, 0.0);
else
    [tt, yy] = rk2Run(XIN, W, TSTOP, H, 0.0, 0.0);
end
if isempty(tt)
    f = Inf;                  % refuse to report "perfect" against an empty run
    return
end
f = max(abs(yy - stepResponse(tt, XIN, W))) / (2*abs(XIN)/W);
end


% =============================================================================
function K = structuralChecks(R, XIN, W, TSTOP, H)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.  They are the weaker
%   claim that it agrees with itself -- which is what catches wiring faults.
K = struct();
names = {};  oks = [];  detail = {};

% 1  ONE STEP == THE EXACT RK4 AMPLIFICATION MATRIX.  The sharp one.  With
%    XIN = 0 the plant is the pure oscillator ydd = -w^2*y, for which RK4 is
%    EXACTLY sum_{k=0..4} (h*A)^k/k! with A = [0 1; -w^2 0].  (h*A)^2 = -q^2*I
%    (q = w*h) collapses that, so from (y,yd) = (1,0) one step must land on
%    y1 = 1 - q^2/2 + q^4/24 and yd1 = -w^2*h*(1 - q^2/6).  A wrong weight in
%    (K0 + 2*(K1+K2) + K3)/6 moves these at the fourth digit while leaving the
%    plotted curve entirely reasonable.  Nothing else here would notice.
q = W*H;
[~, y1, yd1] = rk4Run(0.0, W, 1.5*H, H, 1.0, 0.0);
eY  = abs(y1(1)  - (1 - q^2/2 + q^4/24));
eYD = abs(yd1(1) - (-W*W*H*(1 - q^2/6)))/max(W*W*H, realmin);
K.rk4_map_resid = max(eY, eYD);
names{end+1}  = 'one RK4 step == sum (hA)^k/k!, k = 0..4';
oks(end+1)    = K.rk4_map_resid <= 1e-12;
detail{end+1} = sprintf('max residual %.3e (band 1e-12)', K.rk4_map_resid);

% 2  THE EQUILIBRIUM IS EXACTLY PRESERVED.  y = XIN/w, yd = 0 makes
%    ydd = w*XIN - w^2*(XIN/w) = 0 identically, so a correct integrator sits
%    there forever.  Catches a w vs w^2 swap and a sign flip on the w^2*y term,
%    both of which leave a smooth oscillation at the wrong frequency.
scale = abs(XIN)/W;
[~, yE, ydE] = rk4Run(XIN, W, TSTOP, H, XIN/W, 0.0);
K.equil_resid = max(max(abs(yE - XIN/W))/scale, max(abs(ydE))/(scale*W));
names{end+1}  = 'equilibrium y = XIN/w is a fixed point';
oks(end+1)    = K.equil_resid <= 1e-9;
detail{end+1} = sprintf('max normalised drift %.3e (band 1e-9)', K.equil_resid);

% 3  THE CURVE STAYS INSIDE ITS ANALYTIC ENVELOPE [0, 2*XIN/w].  The exact
%    solution has exactly that range.  Catches divergence, which at these step
%    sizes is the failure mode that still plots smoothly.
pp = 2*abs(XIN)/W;
K.envelope_over = max([max(R.y) - pp, -min(R.y), 0])/pp;
names{end+1}  = 'y stays within [0, 2*XIN/w]';
oks(end+1)    = K.envelope_over <= 1e-3;
detail{end+1} = sprintf('worst excursion %.3e of peak-to-peak (band 1e-3)', ...
    K.envelope_over);

% 4  THE TIME GRID.  Uniform at H, ending within one step of TSTOP.  This row
%    REPORTS Listing 5.1's unguarded stop test rather than hiding it: the
%    sample count is allowed to be off by one, the SPACING is not.
K.dt_resid  = max(abs(diff(R.t) - H));
K.end_resid = abs(R.t(end) - TSTOP);
names{end+1}  = 'time grid uniform at H, ends within one step of TSTOP';
oks(end+1)    = K.dt_resid <= 1e-12 && K.end_resid <= H + 1e-12;
detail{end+1} = sprintf('%d samples, max |dt-H| = %.2e s, |t(end)-TSTOP| = %.2e s', ...
    numel(R.t), K.dt_resid, K.end_resid);

% 5  RK4 WINS AT EQUAL COST, not merely at equal step size.  The book compares
%    the schemes at the SAME h, which flatters RK4 -- it buys accuracy with
%    four derivative evaluations per step against RK2's two.  Holding the
%    evaluation count fixed instead: RK4 at 2H vs RK2 at H, 200 each at the
%    listing's settings.  STRUCTURAL -- the book never makes this claim.
K.cost_ratio = NaN;
if 2*H < TSTOP/4
    e4 = peakFrac(XIN, W, TSTOP, 2*H, true);
    e2 = peakFrac(XIN, W, TSTOP, H,   false);
    K.cost_ratio  = e2/max(e4, realmin);
    names{end+1}  = 'RK4 wins at EQUAL derivative-evaluation count';
    oks(end+1)    = K.cost_ratio >= 5;
    detail{end+1} = sprintf(['RK4 at h=%g: %.4g%% of p-p; RK2 at h=%g: ' ...
        '%.4g%% of p-p; ratio %.1fx (band 5x)'], 2*H, 100*e4, H, 100*e2, ...
        K.cost_ratio);
end

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
function B = benchmarks(R, XIN, W, TSTOP, H)
%BENCHMARKS  Computed vs published, with the PROVENANCE of every band.
%
%   *** EVERY BOOK VALUE HERE WAS GREPPED OUT OF zarchan.txt AND THE LINE
%   NUMBER IS IN THE .source FIELD. ***  Nothing was read off a figure: the
%   extraction turned Figs. 5.2 and 5.3 into image links, and earlier waves in
%   this package were handed "benchmarks" that were exactly such phantom
%   readings.  Where the book states a SHAPE about a figure that prose IS
%   greppable, and an ordering check is stronger than a plotted value.
%
%   Bands: 'analytic' = max(0.5%, (w*H)^4) against the exact step response;
%   'text' = a bound or ordering from prose about a figure; 'exact' = an
%   integer the book prints.  Bands are set by the PROVENANCE of the book
%   value, not by what we happen to compute.  Do NOT widen one to turn a row
%   green.

B = emptyBench();
tolY = max(0.005, (W*H)^4);
pp   = 2*abs(XIN)/W;
% Rows 3 and 4 quote "this example", which is w = 20 rad/s over 1 s with a unit
% step (zarchan.txt:252, :253).  Off that case they SKIP, never pass.
bookCase = abs(W - 20) < 1e-12 && abs(TSTOP - 1) < 1e-12 && abs(XIN - 1) < 1e-12;
whyCase  = sprintf(['off the book''s own example -- Figs. 5.2/5.3 are ' ...
    'w = 20 rad/s, unit step, 1 s; this run used w = %g rad/s, XIN = %g, ' ...
    'TSTOP = %g s'], W, XIN, TSTOP);

% ---- 1.  the endpoint against the printed closed form ---------------------
cfEnd = stepResponse(R.t(end), XIN, W);
B = addVal(B, 'y at t = TSTOP vs (XIN/w)(1 - cos w t)', cfEnd, R.y(end), ...
    'analytic', tolY, 'p.5 closed form (zarchan.txt:251, w at :252)', ...
    '(input units)*s', ...
    sprintf(['sampled at the ACTUAL t = %.6f s, so Listing 5.1''s unguarded ' ...
             'stop test cannot move this residual.  Band = max(0.5%%, ' ...
             '(w*H)^4 = %.4g%%)'], R.t(end), 100*(W*H)^4));

% ---- 2.  the whole curve, not one sample ---------------------------------
r2 = max(abs(R.err))/pp;
B = addBound(B, 'whole curve vs the closed form', ...
    sprintf('<= %.4f%% of peak-to-peak', 100*tolY), ...
    sprintf('%.5g%% of peak-to-peak', 100*r2), r2 <= tolY, ...
    'p.5 closed form (zarchan.txt:251)', '(input units)*s', ...
    ['row 1 is one sample; this is every sample.  Normalised by the exact ' ...
     'peak-to-peak 2*XIN/w because the exact solution touches zero once per ' ...
     'period, where a pointwise relative error would invent a failure']);

% ---- 3.  "0.02 s was adequate" (fourth order) -----------------------------
if bookCase
    e4 = peakFrac(XIN, W, TSTOP, 0.02, true);
    B = addBound(B, 'RK4 at h = 0.02 s still gets the correct solution', ...
        '<= 2% of peak-to-peak', sprintf('%.4g%% of peak-to-peak', 100*e4), ...
        e4 <= 0.02, 'p.92 text (zarchan.txt:1438), Fig. 5.3', ...
        '(input units)*s', ...
        ['the chapter''s headline claim.  2% is a BOUND on "correct", not a ' ...
         'measurement: the book prints no number off Fig. 5.3.  It is chosen ' ...
         'to sit an order of magnitude clear on both sides -- the sweep below ' ...
         'puts h = 0.02 near 0.22% and the next step size, 0.05 s, near 7%']);
else
    B = addSkip(B, 'RK4 at h = 0.02 s still gets the correct solution', ...
        '"adequate for getting the correct solution"', whyCase, ...
        'p.92 text (zarchan.txt:1438), Fig. 5.3', '(input units)*s');
end

% ---- 4.  and the second-order scheme at that same h does not --------------
if bookCase
    e4b   = peakFrac(XIN, W, TSTOP, 0.02, true);
    e2b   = peakFrac(XIN, W, TSTOP, 0.02, false);
    ratio = e2b/max(e4b, realmin);
    B = addShape(B, 'RK2 at h = 0.02 s misses the high bandwidth, RK4 does not', ...
        'second-order error at least 10x the fourth-order error at h = 0.02 s', ...
        sprintf('RK2 %.4g%% vs RK4 %.4g%% of peak-to-peak (%.0fx)', ...
                100*e2b, 100*e4b, ratio), ratio >= 10, ...
        'p.91 text (zarchan.txt:1424) with :1438', ...
        ['the sentence is a comparison, so it is scored as one.  This is the ' ...
         'row that fails if a stage weight is wrong in a way that quietly ' ...
         'degrades the scheme toward second order -- rows 1 and 2 would still ' ...
         'pass at their 0.5% floor band']);
else
    B = addSkip(B, 'RK2 at h = 0.02 s misses the high bandwidth, RK4 does not', ...
        '"missing the effect of the system''s high bandwidth"', whyCase, ...
        'p.91 text (zarchan.txt:1424)', '(input units)*s');
end

% ---- 5.  measured order of accuracy = 4 ----------------------------------
% The defining property of the recursion printed at zarchan.txt:1427-1429.
% Fitted where the h^4 phase term dominates the h^5 amplitude term, which is
% why the fit points are scaled by w rather than fixed in seconds.
if W*TSTOP >= 6
    ho = [0.05 0.1 0.2]/W;
    eo = zeros(size(ho));
    for i = 1:numel(ho)
        eo(i) = peakFrac(XIN, W, TSTOP, ho(i), true);
    end
    if all(eo > 0) && all(isfinite(eo))
        lx = log(ho);  ly = log(eo);
        ordr = sum((lx-mean(lx)).*(ly-mean(ly)))/sum((lx-mean(lx)).^2);
    else
        ordr = NaN;
    end
    B = addVal(B, 'measured order of accuracy', 4.0, ordr, 'analytic', 0.075, ...
        'p.91 recursion (zarchan.txt:1427-1429)', 'decades/decade', ...
        sprintf(['least-squares slope of log(max error) on log(h) at h = ' ...
                 '%.4g, %.4g, %.4g s.  Band 0.075 relative = +/-0.3 ' ...
                 'absolute'], ho(1), ho(2), ho(3)));
else
    B = addSkip(B, 'measured order of accuracy', '4', ...
        sprintf(['w*TSTOP = %.3g rad is under one full oscillation, so the ' ...
                 'error is not yet in its asymptotic regime and a fitted ' ...
                 'slope would be a statement about the transient rather ' ...
                 'than about the scheme'], W*TSTOP), ...
        'p.91 recursion (zarchan.txt:1427-1429)', 'decades/decade');
end

% ---- 6.  Listing 1.1's printed derivative-pass count ----------------------
% "the ratio of the simulation time to the step size is 1000.  This means that
% 2000 passes are made to the differential equations."  Independent of this
% run's own parameters on purpose: it validates the comparator rows 3 and 4
% lean on, and it is the only integer the chapter prints.
[~, ~, ~, n11] = rk2Run(1.0, W, 1.0, 0.001, 0.0, 0.0);
B = addVal(B, 'Listing 1.1 derivative passes at h = 0.001 s over 1 s', ...
    2000, n11, 'exact', 1e-9, 'p.7 text (zarchan.txt:253)', 'evaluations', ...
    '1000 steps x 2 evaluations per step.  Exact integer, no tolerance to set');

% ---- INFO rows.  Reported, never scored. ---------------------------------
B = addInfo(B, 'step-size sweep behind Figs. 5.2 and 5.3', '--', ...
    sweepStr(R.sweep), 'Figs. 5.2/5.3 -- shape only', ...
    ['NOT SCORED: the book publishes those two figures as curves and prints ' ...
     'no value off either one.  The extraction turned every figure into an ' ...
     'image link, so any number read off them would be invented']);

B = addInfo(B, 'RK4''s time path is never exercised here', '--', ...
    sprintf('%d derivative evaluations, every one of them f(y) with no t', ...
            R.p.n_rhs), 'the listing itself (zarchan.txt:1447-1456)', ...
    ['NOT SCORED: XIN is constant, so ydd depends on y alone and the stage ' ...
     'times TNEW = T+0.5*H are computed and never used.  Listing 5.2''s ' ...
     'covariance right-hand side IS time-varying, so a pass here is ' ...
     'necessary but not sufficient for that one']);
end


function s = sweepStr(S)
%SWEEPSTR  The Fig. 5.2/5.3 sweep as one line for the INFO row.
s = '';
for i = 1:numel(S.h)
    s = [s sprintf('h=%.4g: RK4 %.3g%% / RK2 %.3g%% p-p;  ', ...
        S.h(i), 100*S.rk4(i), 100*S.rk2(i))]; %#ok<AGROW>
end
if isempty(s)
    s = '(no step size in the sweep was small enough for this TSTOP)';
end
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw "Subscripted assignment between dissimilar
%   structures" -- which is the behaviour we want.  A struct array whose fields
%   quietly reordered would make [B.ok] mean something else, and it would read
%   as a typo rather than as a schema break.  This is the schema
%   mgs.zarchan.benchSuite's adaptGeneric consumes, and it is STRICT.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
% Tolerance arrives EXPLICITLY rather than through a band->tol lookup, because
% one band here is built from H at the call site and a lookup table would need
% a second copy of that arithmetic.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.4g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.5g', ours);
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
L{end+1} = 'Zarchan Listing 5.1 -- SECOND-ORDER NETWORK, FOURTH-ORDER RUNGE-KUTTA';
L{end+1} = sprintf('   %s', R.units);
L{end+1} = sprintf(['   XIN = %g   w = %g rad/s   TSTOP = %g s   H = %g s   ' ...
    '(w*H = %.3g)   %d steps, %d derivative evaluations'], ...
    p.XIN, p.W, p.TSTOP, p.H, p.W*p.H, p.n_steps, p.n_rhs);
L{end+1} = sprintf(['   vs exact y = (XIN/w)(1-cos w t):  y(end) = %.8f, ' ...
    'exact %.8f, max |err| = %.3e over %d samples'], R.y(end), ...
    R.y_cf(end), max(abs(R.err)), numel(R.t));
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-48s %s', vstr(K.oks(i)), K.names{i}, ...
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
L{end+1} = 'REMINDER: f here has NO explicit t dependence, so the stage times';
L{end+1} = '   TNEW = T + 0.5*H are computed and never used.  Listing 5.2''s';
L{end+1} = '   covariance right-hand side IS time-varying; a pass here is';
L{end+1} = '   necessary but not sufficient for that one.';
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
%   step-size sweep that is the actual point of Figs. 5.2 and 5.3.
figure;
plot(R.t, R.y, '-', R.t, R.y_cf, '--');  grid on;
xlabel('Time (s)');  ylabel('Y');
title('Forth-order Runge-Kutta: Second Order Network');
legend('Listing 5.1 RK4', 'exact (XIN/w)(1-cos wt)', 'Location', 'best');

S = R.sweep;
if numel(S.h) >= 2
    figure;
    loglog(S.h, 100*S.rk4, 'o-', S.h, 100*S.rk2, 's--');  grid on;
    xlabel('Integration step size h (s)');
    ylabel('Max error (% of peak-to-peak)');
    title('Figs. 5.2/5.3: fourth order tolerates a larger step size');
    legend('fourth-order (Listing 5.1)', 'second-order (Listing 1.1)', ...
        'Location', 'best');
end
end
