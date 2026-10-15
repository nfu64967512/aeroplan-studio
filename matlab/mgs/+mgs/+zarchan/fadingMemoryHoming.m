function R = fadingMemoryHoming(opts)
%MGS.ZARCHAN.FADINGMEMORYHOMING  Zarchan Listing 7.1 -- homing engagement with
%   a SECOND-ORDER (two-state) DIGITAL FADING MEMORY FILTER estimating the
%   line-of-sight rate from a noisy line-of-sight angle measurement.
%
%   R = mgs.zarchan.fadingMemoryHoming()
%   R = mgs.zarchan.fadingMemoryHoming('BETA', 0.5, 'assert', false)
%
%   Line-by-line port of Listing 7.1 (zarchan.txt:2007-2057): Heun integration
%   at H = 0.01 s, filter difference equations every TS = 0.1 s.
%
%   UNITS ARE THE BOOK'S: ft, ft/s, ft/s^2, rad, rad/s; g = 32.2 ft/s^2.  NOT
%   SI.  The rest of mgs is metres and nothing is converted here, so that every
%   number printed lies straight against a published one.  Convert AT THE CALL
%   SITE.  *** WHAT IT IS NOT: *** linearised, planar, constant closing
%   velocity, UNITY-GAIN flight control (:2004) -- the ONLY lag in the loop is
%   the filter.  No gravity, drag, thrust, acceleration limit or LOS floor.
%   Relevance here: our tgo predictor's scale error is 1.4-1.5 where Appendix
%   A.1 says 0.9 already loses to plain PN, and a fading memory filter is one
%   candidate fix.  This is the smallest honest statement of that machinery.
%
%   TRANSCRIPTION.  listing_07_01.m does not parse.  Its header reports no
%   unresolved variable-to-variable comparisons, and that is right -- both
%   surviving comparisons are against expressions in constants.  Three repairs,
%   all shape-forced:
%     1 "while T ,= (TF - 1e-5)" -> "<=".  "," is this book's OCR of "<"; T only
%       increases, so ">=" never terminates.
%     2 "if S . =(TS - 1e-5)" -> ">=".  "." is the OCR of ">"; S is reset to 0
%       on every record, so "<=" would record every step and kill the sampler.
%     3 The trailing paragraph (with the "b 1/4 0.8" ligature damage) is PROSE
%       at :2064 leaking in.  Dropped, not "fixed" into code -- and it is what
%       supplies BENCHMARKS 1-2 and the nominal case, which is how it was
%       identified.
%   Kept: TGO's +1e-5, so the LAST sample has TGO = 1e-5 and an XLAMD of order
%   Y/(VC*1e-10).  That is the listing's own behaviour; the book hides it behind
%   axis([0 10 -.01 .06]), a plot range and not a measurement.  Every windowed
%   measurement below excludes that sample explicitly.  Dropped: "save
%   datfil.txt output /ascii" (typo for -ascii; Listing 7.3 has it right at
%   :2205) -- nothing is written to disk.  Replaced: randn, by a Lehmer
%   generator + Box-Muller local to this file, so the benchmark cannot move
%   when someone else calls rng().
%
%   ** THE BOOK CONTRADICTS ITSELF ON b. **  The printed listing sets BETA=0.3
%   (:2044) and its plot title is Fig. 7.3's caption; the NOMINAL case in prose
%   is b = 0.8 (:2027, :2036, :2064) and Listings 7.2/7.3 hard-code .8 (:2071,
%   :2196).  Default here is the nominal 0.8, with 0.3 run alongside as
%   BETA_FAST -- the two scored figure claims are a comparison BETWEEN them.
%   Both lines are cited on the INFO row.
%
%   THE ONE CLOSED FORM, DERIVED HERE (not in the book).  Feed the filter a
%   quadratic angle lam = 0.5*c*t^2, so the true LOS rate is the ramp c*t that
%   a constant target maneuver produces.  The two-state filter models the angle
%   as degree 1, so by the book's truncation-error prose (:2001) it must lag.
%   Solving Table 7.1's recursion (:2011-2012) at steady state gives exactly
%        residual = c*Ts^2/H,   rate error = c*Ts*(G/H - 0.5),
%        position error = c*Ts^2*(1-G)/H  (CONSTANT),
%        equivalent LAG = Ts*(G/H - 0.5) = Ts*[(1+b)/(1-b) - 0.5]  seconds,
%   since G/H = (1-b^2)/(1-b)^2 = (1+b)/(1-b).  One expression predicts all
%   three book figure claims:  b=.8/Ts=.1 -> 0.850 s (Fig. 7.2 "sluggish");
%   b=.3/Ts=.1 -> 0.136 s (Fig. 7.3 "no longer lags"); b=.8/Ts=.05 -> 0.425 s
%   (Fig. 7.20).  It is scored STRUCTURALLY (open loop, exact); the book's
%   claims are scored as ORDERINGS on the CLOSED-loop measurement, a weaker
%   quantity -- the loop closes through the estimate, so the measured shift
%   need not equal 0.850 s and is not asked to.
%
%   THE OBVIOUS DELETE EXPERIMENT DOES NOT FIRE, which is why it is written
%   down.  Deleting the TS*XLAMDH prediction term from the residual looks
%   fatal.  It is not: the deleted recursion stays STABLE (poles move to
%   |z| = sqrt(1-G+H) = 0.825 at b = 0.8, still inside the unit circle) and on
%   a RAMP still converges to the correct rate with only a one-sample position
%   offset -- residual, rate and stability all look fine.  The signal appears
%   only on a QUADRATIC, where the intact position error is constant and the
%   deleted one grows by exactly c*Ts per second.  Structural check 4 watches
%   that; a delete experiment aimed at the residual would have passed broken.
%
%   BENCHMARKS.  Every book value was grepped from zarchan_decoded.txt; the
%   line is in each row's .source.  NOTHING was read off a figure -- the
%   extraction turned every figure into an image link.
%     1-2  nT = 96.6 ft/s^2 = 3 g, SIGNOISE = .001 rad = 1 mr.  Stated in words
%          at :2064 and in code at :2044, so the row checks the two agree.
%     3-4  G = 1-b^2 = 0.36, H = (1-b)^2 = 0.04 at b = 0.8 (Table 7.1, :2011-12).
%     5    BOUND: Ts/H "must be a large integer" (:2026, repeated :2177).
%     6    INFO: the b contradiction, both lines cited.
%     7-10 SHAPE, from prose about Figs. 7.2, 7.3, 7.20 (:2036, :2037, :2209).
%          Lags are measured NOISE-FREE (lag is deterministic; one noise draw
%          would be an anecdote); noise transmission is std(noisy - clean) on
%          the SAME seed, the paired form of "the price paid for reducing b".
%     11   SKIP: Figs. 7.4/7.5 publish curves, not numbers.
%   Chapter 7 publishes almost no scoreable NUMBERS -- it is a chapter of
%   figures.  Four config values, one bound and four orderings is the honest
%   yield, not a short measurement.
%
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED -- this file has never
%   been executed; MATLAB is the caller's. ***  G = 0.36 and H = 0.04 exactly;
%   Ts/H = 10; the open-loop lag closed form matched to ~1e-10; closed-loop
%   measured lag at b = 0.8 of order several tenths of a second and POSITIVE;
%   noise-transmission ratio (b=.3 over b=.8) of order 10.  The terminal true
%   LOS rate approaches nT/VC = 96.6/4000 = 0.024 rad/s, which is why the
%   book's 0.06 rad/s plot ceiling accommodates it.  A residual landing near a
%   band EDGE is not a pass to be accepted.  Do not widen a band to make a row
%   green.
%
%   OUTPUTS
%     R.nom R.nom_clean R.fast R.fast_clean R.fine_clean   the five runs, each
%          .t .y .yd .xnc .xlam .xlamd .xlamdh .xlamh .res .tgo .miss_ft .nstep
%          recorded at the SAMPLE instants, as the listing's ArrayT does
%     R.lag       measured closed-loop lag [s] and its closed-form twin
%     R.noise_tx  std(noisy - clean) of the rate estimate, and the ratio
%     R.bench     benchmark rows, schema shared with the other ports
%     R.selfcheck .names/.oks/.detail/.all_ok/.first_bad -- structural only
%     R.pass      true only if at least one row was SCORED and all passed
%     R.verdict   '' when everything passed, else the one-line reason
%     R.p         parameters actually used;  R.summary  the rendered text
%
%   See also MGS.ZARCHAN.SHAPINGFILTERADJOINT, MGS.ZARCHAN.BENCHSUITE.
%
arguments
    % Listing 7.1 line 1, verbatim (:2044).  Feet, seconds, radians.
    opts.VC       (1,1) double {mustBeFinite, mustBePositive} = 4000    % ft/s
    opts.XNT      (1,1) double {mustBeFinite} = 96.6                    % ft/s^2 = 3 g
    opts.VM       (1,1) double {mustBeFinite, mustBePositive} = 3000    % ft/s
    opts.HEDEG    (1,1) double {mustBeFinite} = 0                       % deg
    opts.XNP      (1,1) double {mustBeFinite, mustBePositive} = 3       % N-prime
    opts.SIGNOISE (1,1) double {mustBeFinite, mustBeNonnegative} = .001 % rad = 1 mr
    opts.TF       (1,1) double {mustBeFinite, mustBePositive} = 10      % s
    opts.TS       (1,1) double {mustBeFinite, mustBePositive} = .1      % s, sampler
    opts.H        (1,1) double {mustBeFinite, mustBePositive} = .01     % s, RK2 step
    % b.  DEFAULT IS THE PROSE NOMINAL 0.8, NOT the listing's printed 0.3 --
    % see "THE BOOK CONTRADICTS ITSELF ON b" in the header.
    opts.BETA      (1,1) double {mustBeFinite, mustBeNonnegative, mustBeLessThan(opts.BETA,1)} = 0.8
    % The listing's own printed value, run alongside so Fig. 7.3's comparison
    % can be scored.  It is a comparison BETWEEN two b, not a property of one.
    opts.BETA_FAST (1,1) double {mustBeFinite, mustBeNonnegative, mustBeLessThan(opts.BETA_FAST,1)} = 0.3
    % Fig. 7.20's case: same b, halved sampling time (:2209).
    opts.TS_FINE  (1,1) double {mustBeFinite, mustBePositive} = .05     % s
    opts.seed     (1,1) double {mustBeFinite, mustBeNonnegative} = 20260824
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so an `if nargin == 0` here would be a mode that never fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.assert    (1,1) logical = true
end

VC = opts.VC;  XNT = opts.XNT;  VM = opts.VM;  HEDEG = opts.HEDEG;
XNP = opts.XNP;  SIG = opts.SIGNOISE;  TF = opts.TF;  TS = opts.TS;
H = opts.H;  B0 = opts.BETA;  BF = opts.BETA_FAST;  TSF = opts.TS_FINE;

% The book states this as a REQUIREMENT, twice (:2026, :2177), so a violation
% is refused rather than warned about: at a non-integral ratio the sampler
% drifts against the integration grid and the run still looks perfectly smooth.
checkRatio(TS, H, 'TS');
if TF <= 4*TS
    error('mgs:zarchan:fadingMemoryHoming:short', ...
        'TF = %g s gives fewer than five filter samples at TS = %g s.', TF, TS);
end

% Fig. 7.20's run halves TS.  H IS HALVED WITH IT, so that Ts/H stays at the
% listing's 10 rather than dropping to 5: the book requires that ratio to be a
% LARGE integer (:2026), and letting it fall would change TWO things at once
% and make the lag comparison uninterpretable.
HF = H*TSF/TS;
checkRatio(TSF, HF, 'TS_FINE');

% ---- the five runs.  Each is ~TF/H steps, so all five together are cheap. ---
nomC  = engage(VC, XNT, XNP, VM, HEDEG, 0.0, B0, TS,  TF, H,  opts.seed);
nomN  = engage(VC, XNT, XNP, VM, HEDEG, SIG, B0, TS,  TF, H,  opts.seed);
fastC = engage(VC, XNT, XNP, VM, HEDEG, 0.0, BF, TS,  TF, H,  opts.seed);
fastN = engage(VC, XNT, XNP, VM, HEDEG, SIG, BF, TS,  TF, H,  opts.seed);
fineC = engage(VC, XNT, XNP, VM, HEDEG, 0.0, B0, TSF, TF, HF, opts.seed);

% ---- lag, measured on the CLEAN runs (see header) --------------------------
L = struct();
L.nom_s     = measureLag(nomC,  TS,  TF);
L.fast_s    = measureLag(fastC, TS,  TF);
L.fine_s    = measureLag(fineC, TSF, TF);
L.nom_cf_s  = lagClosedForm(B0, TS);
L.fast_cf_s = lagClosedForm(BF, TS);
L.fine_cf_s = lagClosedForm(B0, TSF);

% ---- noise transmission: paired, same seed, same window --------------------
N = struct();
N.nom  = pairedNoiseStd(nomC,  nomN);
N.fast = pairedNoiseStd(fastC, fastN);
N.ratio = N.fast / max(N.nom, realmin);

R = struct();
R.nom = nomN;  R.nom_clean = nomC;
R.fast = fastN;  R.fast_clean = fastC;
R.fine_clean = fineC;
R.lag = L;
R.noise_tx = N;
R.p = struct('VC',VC, 'XNT',XNT, 'VM',VM, 'HEDEG',HEDEG, 'XNP',XNP, ...
             'SIGNOISE',SIG, 'TF',TF, 'TS',TS, 'H',H, 'BETA',B0, ...
             'BETA_FAST',BF, 'TS_FINE',TSF, 'seed',opts.seed, ...
             'GFILTER',1-B0^2, 'HFILTER',(1-B0)^2, ...
             'units','ft, ft/s, ft/s^2, rad, rad/s, s; g = 32.2 ft/s^2');

R.selfcheck = structuralChecks(VC, XNT, XNP, VM, HEDEG, B0, TS, TF, H);

if opts.benchmark
    R.bench = benchmarks(R, B0, BF, TS, TSF, H, XNT, SIG, L, N);
    ev = ~[R.bench.skipped];
    R.pass = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev = false;
    % NOT true.  "Not checked" must never render as "fine".
    R.pass = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false, so nothing was compared against the book. R.pass is false meaning NOT CHECKED.';
elseif ~any(ev)
    msg = 'not one book value could be evaluated -- every row skipped on off-book parameters.';
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question. First: %s.'], ...
        sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf('%d of %d evaluated book values are outside tolerance (first: %s, book %s, ours %s).', ...
        numel(bad), sum(ev), R.bench(bad(1)).what, R.bench(bad(1)).book_str, ...
        R.bench(bad(1)).ours_str);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:fadingMemoryHoming:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN, complete.  benchSuite runs every port with assert
        % off precisely so one red row cannot hide the board.
        warning('mgs:zarchan:fadingMemoryHoming:benchmark', ...
            '%s  (''assert'' is false; see R.verdict)', msg);
    end
end
end


% =============================================================================
function checkRatio(TSx, H, name)
%CHECKRATIO  zarchan.txt:1990 -- "the ratio Ts/H must be a large integer".
r = TSx/H;
if abs(r - round(r)) > 1e-9*max(1,r) || round(r) < 2
    error('mgs:zarchan:fadingMemoryHoming:ratio', ...
        ['%s/H = %g/%g = %.6g.  The book requires a LARGE INTEGER here ' ...
         '(zarchan.txt:1990, repeated :2177).  A non-integral ratio lets the ' ...
         'sampler drift against the integration grid and still returns a ' ...
         'smooth, entirely wrong curve, so it is refused not warned about.'], ...
        name, TSx, H, r);
end
end


% =============================================================================
function S = engage(VC, XNT, XNP, VM, HEDEG, SIGNOISE, BETA, TS, TF, H, seed)
%ENGAGE  Listing 7.1, zarchan.txt:2008-2056, verbatim except for the plotting.
%   Recorded at the SAMPLE instants, as the listing's ArrayT does.  Every
%   constant is an ARGUMENT: MATLAB local functions cannot see the main
%   function's workspace, and a previous wave here lost a run to exactly that.

Y    = 0;                       % YIC
YD   = -VM*HEDEG/57.3;          % the listing's initial heading error; VM is
                                % used HERE AND NOWHERE ELSE, so at the
                                % listing's own HEDEG = 0 it is dead weight.
T    = 0;
S_   = 0;
GFILTER = 1 - BETA^2;           % Table 7.1, zarchan.txt:1975
HFILTER = (1 - BETA)^2;         % Table 7.1, zarchan.txt:1976
XLAMH  = 0;
XLAMDH = 0;
XNC    = 0;

% Initialised before the loop so the FLAG==1 branch cannot read a stale value.
YDD = 0;  XLAM = 0;  XLAMD = 0;  TGO = TF;

nmax = ceil(TF/TS) + 4;
t = zeros(nmax,1);  y = t;  yd = t;  xnc = t;  xlam = t;
xlamd = t;  xlamdh = t;  xlamh = t;  res = t;  tgo = t;
count = 0;
nstep = 0;

% One Gaussian per SAMPLE, drawn from a generator local to this file so the
% benchmark cannot move when someone else calls rng().
g = lcgGauss(seed, nmax);

while T <= (TF - 1e-5)
    YOLD = Y;  YDOLD = YD;  STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  Pass 1 evaluates at the current
    % state; pass 2 takes a full Euler step, advances T by H, and re-evaluates
    % there.  T advances INSIDE this loop, exactly once -- move that line out
    % and the scheme quietly degrades to first order while still running.
    while STEP <= 1
        if FLAG == 1
            Y  = Y  + H*YD;
            YD = YD + H*YDD;
            T  = T  + H;
            STEP = 2;
        end
        TGO   = TF - T + 1e-5;      % +1e-5: never divide by exactly zero
        RTM   = VC*TGO;
        XLAM  = Y/(VC*TGO);
        XLAMD = (RTM*YD + Y*VC)/(RTM^2);
        YDD   = XNT - XNC;          % UNITY-GAIN flight control: no autopilot lag
        FLAG  = 1;
    end
    % The listing writes "FLAG=0" here too.  Dropped: the outer loop
    % re-initialises FLAG, so the assignment cannot be read.  Noted rather than
    % coded so nobody re-adds it thinking a state machine was lost.

    % Corrector, in the listing's own algebra AND ORDER: Y is corrected FIRST,
    % using the PREDICTED YD; YD is corrected after, using the pass-2 YDD.
    % Swapping these two lines is a silent change of scheme.
    Y  = .5*(YOLD  + Y  + H*YD);
    YD = .5*(YDOLD + YD + H*YDD);

    nstep = nstep + 1;
    S_ = S_ + H;
    if S_ >= (TS - 1e-5)
        S_ = 0;
        count = count + 1;
        if SIGNOISE > 0
            XLAMNOISE = SIGNOISE*g(count);
        else
            XLAMNOISE = 0;
        end
        % Table 7.1's second-order recursion.  The prediction term TS*XLAMDH
        % inside the residual is what bounds the POSITION error on a quadratic
        % angle; deleting it leaves the loop stable and the rate roughly right,
        % which is exactly why the delete experiment in structuralChecks has to
        % watch the position error and not the residual.
        RES    = XLAM - (XLAMH + TS*XLAMDH) + XLAMNOISE;
        XLAMH  = GFILTER*RES + XLAMH + TS*XLAMDH;
        XLAMDH = HFILTER*RES/TS + XLAMDH;
        XNC    = XNP*VC*XLAMDH;     % held constant until the next sample

        t(count) = T;        y(count) = Y;          yd(count) = YD;
        xnc(count) = XNC;    xlam(count) = XLAM;    xlamd(count) = XLAMD;
        xlamdh(count) = XLAMDH;  xlamh(count) = XLAMH;
        res(count) = RES;    tgo(count) = TGO;
    end
end

k = 1:count;
S = struct('t',t(k), 'y',y(k), 'yd',yd(k), 'xnc',xnc(k), 'xlam',xlam(k), ...
           'xlamd',xlamd(k), 'xlamdh',xlamdh(k), 'xlamh',xlamh(k), ...
           'res',res(k), 'tgo',tgo(k), 'miss_ft',Y, 'nstep',nstep, ...
           'beta',BETA, 'ts',TS, 'signoise',SIGNOISE);
end


% =============================================================================
function [w, lo, hi] = scoreWindow(S, TS, TF)
%SCOREWINDOW  The samples any windowed measurement is allowed to use.
%
%   Excludes (a) the first 3.5 s, where the filter is still settling from zero
%   estimates -- its poles are both at z = b, so at b = 0.8 the transient is
%   0.8^35 ~ 4e-4 by then -- and (b) the last 1.5 s, where TGO -> 1e-5 drives
%   the TRUE XLAMD to order Y/(VC*TGO^2).  That last sample is a real feature
%   of the listing, not a bug, and the book hides it behind axis([0 10 ...]).
lo = min(3.5, 0.35*TF);
hi = TF - max(1.5, 3*TS);
w  = find(S.t >= lo & S.t <= hi);
end


% =============================================================================
function lag = measureLag(S, TS, TF)
%MEASURELAG  Best whole-plus-fractional time shift making the ESTIMATE
%   xlamdh(t) coincide with the TRUE xlamd(t - lag), by minimum SSE.  Plain SSE
%   (no gain fit) is correct here: the filter has unity DC gain, so the shift
%   is the only degree of freedom worth searching.  NEGATIVE shifts are
%   searched too -- BENCHMARK 8 has to be able to see a filter that LEADS,
%   rather than have it clipped to zero.

[w, ~, ~] = scoreWindow(S, TS, TF);
if numel(w) < 8
    lag = NaN;  return
end
% Both ends of the shift search are clamped to the recorded array: a shift that
% ran off either end would either error or (worse, with a stray transpose)
% silently wrap.
kmax = min(round(3/TS),   w(1) - 1);
kmin = max(-round(0.5/TS), w(end) - numel(S.xlamd));
if kmax < max(kmin, 1)
    lag = NaN;  return
end
ks   = kmin:kmax;
sse  = zeros(size(ks));
est  = S.xlamdh(w);
for j = 1:numel(ks)
    d      = S.xlamd(w - ks(j)) - est;
    sse(j) = sum(d.*d);
end
[~, jb] = min(sse);
lag = ks(jb)*TS;

% Sub-sample parabolic refinement, guarded against a flat or boundary minimum.
if jb > 1 && jb < numel(ks)
    den = sse(jb-1) - 2*sse(jb) + sse(jb+1);
    if den > 0
        lag = lag + 0.5*(sse(jb-1) - sse(jb+1))/den * TS;
    end
end
end


% =============================================================================
function lag = lagClosedForm(BETA, TS)
%LAGCLOSEDFORM  Ts*(G/H - 0.5) = Ts*[(1+b)/(1-b) - 0.5].  DERIVED HERE from
%   Table 7.1 (zarchan.txt:1975-2012) for a quadratic angle / ramp rate; see
%   the header.  It is NOT a published number and is never scored as one.
lag = TS*((1 + BETA)/(1 - BETA) - 0.5);
end


% =============================================================================
function s = pairedNoiseStd(clean, noisy)
%PAIREDNOISESTD  std of the noise-induced part of the rate estimate.
%   PAIRED: same seed, same b, same window, so the difference isolates the
%   noise response of the filter from the deterministic engagement.
n = min(numel(clean.t), numel(noisy.t));
[w, ~, ~] = scoreWindow(clean, clean.ts, clean.t(n));
w = w(w <= n);
if numel(w) < 4
    s = NaN;  return
end
d = noisy.xlamdh(w) - clean.xlamdh(w);
s = sqrt(sum((d - sum(d)/numel(d)).^2)/(numel(d) - 1));
end


% =============================================================================
function [xh, xdh, res] = fmfOpen(z, TS, G, Hg, usePredict)
%FMFOPEN  Table 7.1's two-state recursion, OPEN LOOP, on a supplied stream z.
%   usePredict = false DELETES the TS*xdh term from the residual only -- that
%   is the delete experiment behind structural check 3.
n = numel(z);
xh = zeros(n,1);  xdh = zeros(n,1);  res = zeros(n,1);
a = 0;  v = 0;
for i = 1:n
    if usePredict
        pred = a + TS*v;
    else
        pred = a;
    end
    r = z(i) - pred;
    a = G*r + a + TS*v;
    v = Hg*r/TS + v;
    xh(i) = a;  xdh(i) = v;  res(i) = r;
end
end


% =============================================================================
function u = lcgUniform(seed, n)
%LCGUNIFORM  MINSTD (Lehmer) uniforms in (0,1).  Base MATLAB, deterministic,
%   and it does not touch the global random stream.  16807*x < 3.61e13 < 2^53
%   so every product is EXACT in double precision.  x = 0 is absorbing, so the
%   seed is mapped into 1..m-1 and u never reaches 0 (log(0) below).
m = 2147483647;
x = mod(floor(abs(seed)), m - 1) + 1;
u = zeros(n,1);
for k = 1:n
    x = mod(16807*x, m);
    u(k) = x/m;
end
end


% =============================================================================
function g = lcgGauss(seed, n)
%LCGGAUSS  Box-Muller on lcgUniform, standing in for the listing's randn.
%   The two uniforms of each pair are taken from SEPARATE BLOCKS of the stream:
%   consecutive values of a multiplicative generator lie on a lattice in 2-D,
%   and pairing them consecutively would put visible structure into the noise.
m = max(1, ceil(n/2));
u = lcgUniform(seed, 2*m);
u1 = u(1:m);  u2 = u(m+1:2*m);
r  = sqrt(-2*log(u1));
g  = reshape([r.*cos(2*pi*u2), r.*sin(2*pi*u2)].', [], 1);
g  = g(1:n);
end


% =============================================================================
function K = structuralChecks(VC, XNT, XNP, VM, HEDEG, BETA, TS, TF, H)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.  They are the weaker
%   claim that it agrees with ITSELF -- which is what catches wiring faults.
K = struct();
names = {};  oks = [];  detail = {};

G  = 1 - BETA^2;
Hg = (1 - BETA)^2;

% 1  VC CANCELS when the noise is off.  XLAM ~ 1/VC and XNC = XNP*VC*XLAMDH,
%    so the commanded acceleration -- and therefore the whole trajectory -- is
%    independent of closing velocity.  (With noise ON it does NOT cancel: the
%    angle noise is not scaled by 1/VC, so noise-induced acceleration is
%    proportional to VC.  That asymmetry is real and is why this check is
%    stated for the noise-free case only.)
A = engage(VC,   XNT, XNP, VM, HEDEG, 0, BETA, TS, TF, H, 1);
Bv = engage(3*VC, XNT, XNP, VM, HEDEG, 0, BETA, TS, TF, H, 1);
nAB = min(numel(A.y), numel(Bv.y));
relVC = max(abs(A.y(1:nAB) - Bv.y(1:nAB))) / max(max(abs(A.y(1:nAB))), realmin);
names{end+1} = 'VC cancels (noise off): y identical at VC and 3*VC';
oks(end+1)   = relVC <= 1e-8;
detail{end+1} = sprintf('max relative |dy| = %.3e', relVC);

% 2  b = 0 makes the filter a PASS-THROUGH.  Table 7.1 gives G = 1 - 0 = 1 and
%    H = 1, and the recursion then returns xh(n) = z(n) exactly.  Catches a
%    swapped or mis-transcribed gain formula without needing a tolerance.
z0  = cumsum(ones(60,1))*0.003 + sin((1:60)'*0.7)*0.01;
xh0 = fmfOpen(z0, TS, 1-0^2, (1-0)^2, true);
passErr = max(abs(xh0 - z0));
names{end+1} = 'b = 0 => filter output equals measurement exactly';
oks(end+1)   = passErr <= 1e-14;
detail{end+1} = sprintf('max |xh - z| = %.3e rad', passErr);

% 3  DEGREE-1 EXACTNESS.  A RAMP angle is degree 1, which the two-state filter
%    models exactly, so its steady-state residual AND rate error are both ZERO
%    -- the concrete form of the book's truncation-error prose (:2001).  The
%    filter's poles are both at z = b, so 0.8^400 has long since vanished; the
%    only floor left is rounding.
tR   = (1:400)'*TS;
aR   = 0.02;
[~, vI, rI] = fmfOpen(aR*tR, TS, G, Hg, true);
rampRes  = abs(rI(end));
rampRate = abs(vI(end) - aR);
names{end+1} = 'ramp input: residual and rate error both exactly 0';
oks(end+1)   = rampRes <= 1e-12 && rampRate <= 1e-12;
detail{end+1} = sprintf('|res| = %.3e rad, |rate err| = %.3e rad/s (true rate %.3g)', ...
                        rampRes, rampRate, aR);

% 4  THE CLOSED FORM the three shape rows rest on, AND THE DELETE EXPERIMENT.
%    A QUADRATIC angle lam = 0.5*c*t^2 (true rate a ramp, which is what a
%    constant target maneuver produces) gives, exactly:
%        residual   = c*Ts^2/H
%        rate error = c*Ts*(G/H - 0.5)          <- the lag closed form
%        pos error  = c*Ts^2*(1-G)/H, CONSTANT
%    Derived in the header from Table 7.1.  Open loop, so these are exact.
%
%    DELETE EXPERIMENT: remove the TS*xdh prediction term from the residual.
%    The deleted recursion stays stable and still gets the RATE roughly right,
%    so residual- or divergence-based tests pass on it (see the header).  What
%    it cannot do is hold the POSITION error: that grows by exactly c*Ts per
%    second.  Measured over the second half of the record, intact growth must
%    be at rounding level and deleted growth must equal c*Ts*dt.
cQ = 0.005;
zQ = 0.5*cQ*tR.^2;
[aQ, vQ, rQ] = fmfOpen(zQ, TS, G, Hg, true);
aD           = fmfOpen(zQ, TS, G, Hg, false);
resPred  = cQ*TS^2/Hg;
ratePred = cQ*TS*(G/Hg - 0.5);
posPred  = cQ*TS^2*(1 - G)/Hg;
i1 = 200;  i2 = 400;  dt = tR(i2) - tR(i1);
eI = [abs(rQ(i2)/resPred - 1), ...
      abs((cQ*tR(i2) - vQ(i2))/ratePred - 1), ...
      abs((zQ(i2) - aQ(i2))/posPred - 1)];
growI = abs((zQ(i2)-aQ(i2)) - (zQ(i1)-aQ(i1)));
growD = abs((zQ(i2)-aD(i2)) - (zQ(i1)-aD(i1)));
growPred = cQ*TS*dt;
names{end+1} = 'quadratic closed form + DELETE prediction term';
oks(end+1)   = all(eI <= 1e-9) && growI <= 1e-12 && ...
               abs(growD/growPred - 1) <= 1e-6;
detail{end+1} = sprintf(['res %.5g/%.5g, rate err %.5g/%.5g, pos err %.5g/%.5g ' ...
                         '(max rel %.1e); pos drift over %.4g s: intact %.3e, ' ...
                         'deleted %.5g vs predicted %.5g'], ...
                        rQ(i2), resPred, cQ*tR(i2)-vQ(i2), ratePred, ...
                        zQ(i2)-aQ(i2), posPred, max(eI), dt, growI, growD, growPred);

% 5  ZERO FORCING => ZERO MOTION.  With XNT = 0, HEDEG = 0 and no noise there
%    is nothing to drive the engagement, so y, xlamd and xnc must be EXACTLY
%    zero -- not small.  Catches a stray constant or a sign-flipped bias that a
%    tolerance-based check would absorb.
Z = engage(VC, 0, XNP, VM, 0, 0, BETA, TS, TF, H, 1);
zeroMax = max([max(abs(Z.y)), max(abs(Z.xlamd)), max(abs(Z.xnc))]);
names{end+1} = 'zero forcing (nT = 0, HE = 0, no noise) => y, lamdot, nc all 0';
oks(end+1)   = zeroMax == 0;
detail{end+1} = sprintf('max |y|,|lamdot|,|nc| = %.3e', zeroMax);

K.names = names;
K.oks   = logical(oks(:)).';
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
function B = benchmarks(R, B0, BF, TS, TSF, H, XNT, SIG, L, N)
%BENCHMARKS  Book values only.  Line numbers are valid in either zarchan.txt
%   or zarchan_decoded.txt -- the two files share length and numbering.
B = emptyBench();
G = 32.2;

% ---- 1, 2: the two nominal-case quantities stated in WORDS and in CODE -----
if abs(XNT - 96.6) < 1e-12
    B = addVal(B, 'target maneuver nT', 3.0, XNT/G, '1e-9', 1e-9, ...
        'prose "constant 3-g target maneuver", zarchan.txt:2028; listing XNT=96.6, :2044', ...
        'g', 'checks the word and the code agree: 96.6/32.2 is exactly 3');
else
    B = addSkip(B, 'target maneuver nT', '3 g', ...
        sprintf('XNT was changed from the listing''s 96.6 to %g ft/s^2', XNT), ...
        'zarchan.txt:2028, :2044', 'g');
end

if abs(SIG - .001) < 1e-15
    B = addVal(B, 'LOS angle measurement noise', 1.0, SIG*1000, '1e-9', 1e-9, ...
        'prose "1 milliradian (mr) of measurement noise", zarchan.txt:2028; listing SIGNOISE=.001, :2044', ...
        'mr', 'one sigma, added to the sampled LOS angle every TS');
else
    B = addSkip(B, 'LOS angle measurement noise', '1 mr', ...
        sprintf('SIGNOISE was changed from the listing''s .001 to %g rad', SIG), ...
        'zarchan.txt:2028, :2044', 'mr');
end

% ---- 3, 4: Table 7.1's two gain formulas, at the nominal b -----------------
if abs(B0 - 0.8) < 1e-12
    B = addVal(B, 'fading memory gain G at b = 0.8', 0.36, 1-B0^2, '1e-12', 1e-12, ...
        'Table 7.1 "G = 1 - b^2", zarchan.txt:1975; b = 0.8 from :2064', ...
        '-', 'listing: GFILTER=1.-BETA^2 (:2044)');
    B = addVal(B, 'fading memory gain H at b = 0.8', 0.04, (1-B0)^2, '1e-12', 1e-12, ...
        'Table 7.1 "H = (1 - b)^2", zarchan.txt:1976; b = 0.8 from :2064', ...
        '-', 'listing: HFILTER=(1.-BETA)^2 (:2044)');
else
    B = addSkip(B, 'fading memory gain G at b = 0.8', '0.36', ...
        sprintf('BETA = %g, not the nominal 0.8', B0), 'zarchan.txt:1975', '-');
    B = addSkip(B, 'fading memory gain H at b = 0.8', '0.04', ...
        sprintf('BETA = %g, not the nominal 0.8', B0), 'zarchan.txt:1976', '-');
end

% ---- 5: the book's stated REQUIREMENT on the two step sizes ----------------
ratio = TS/H;
B = addBound(B, 'Ts/H is a large integer', 'integer, "large"', ...
    sprintf('%.6g', ratio), ...
    abs(ratio-round(ratio)) <= 1e-9*max(1,ratio) && ratio >= 10, ...
    'prose "the ratio Ts/H must be a large integer", zarchan.txt:1990 (repeated :2177)', ...
    '-', 'listing''s own TS=.1, H=.01 (:2044) gives 10');

% ---- 6: the contradiction -------------------------------------------------
B = addInfo(B, 'value of b (the book states two)', ...
    'listing prints BETA=0.3 (:2044); prose nominal case is b = 0.8 (:2027, :2036, :2064)', ...
    sprintf('this run: BETA = %g (default 0.8), BETA_FAST = %g', B0, BF), ...
    'zarchan.txt:2008 vs :2064; Listings 7.2/7.3 also use .8 (:2071, :2196)', ...
    ['the printed listing''s plot title IS Fig. 7.3''s caption, so the 0.3 ' ...
     'source is the Fig. 7.3 run; 0.8 is the nominal case the rest of the ' ...
     'chapter analyses.  Both are run here, so neither is discarded.']);

% ---- 7-10: the three figure claims, as orderings ---------------------------
lagOK = isfinite(L.nom_s) && isfinite(L.fast_s) && isfinite(L.fine_s);

B = addShape(B, 'b = 0.8: rate estimate LAGS the true LOS rate', ...
    '"smooth but lags the actual line-of-sight rate ... the filter is sluggish"', ...
    sprintf('measured lag %+.3f s (closed form %.3f s)', L.nom_s, L.nom_cf_s), ...
    lagOK && L.nom_s >= TS, ...
    'prose about Fig. 7.2, zarchan.txt:2000; caption :2027', ...
    'measured on the NOISE-FREE run; at least one whole sample of lag required');

B = addShape(B, 'b 0.8 -> 0.3 removes the lag', ...
    '"the line-of-sight rate estimate no longer lags the actual signal when b is reduced from 0.8 to 0.3"', ...
    sprintf('lag %+.3f s at b=%.2g vs %+.3f s at b=%.2g', L.fast_s, BF, L.nom_s, B0), ...
    lagOK && L.fast_s < 0.5*L.nom_s, ...
    'prose about Fig. 7.3, zarchan.txt:2001', ...
    'ordering only -- the book publishes no lag number, Fig. 7.3 is a figure');

if SIG > 0 && isfinite(N.ratio)
    B = addShape(B, 'decreasing b increases noise transmission', ...
        '"the noisiness of the rate estimate is the price paid for reducing b"', ...
        sprintf('std(noisy-clean) %.3e at b=%.2g vs %.3e at b=%.2g, ratio %.2f', ...
                N.fast, BF, N.nom, B0, N.ratio), ...
        N.ratio > 1, ...
        'prose about Fig. 7.3, zarchan.txt:2001', ...
        'PAIRED: same seed, same window, so the difference is the noise response alone');
else
    B = addSkip(B, 'decreasing b increases noise transmission', 'ratio > 1', ...
        'SIGNOISE = 0, so there is no noise response to compare', ...
        'zarchan.txt:2001', '-');
end

B = addShape(B, 'halving Ts removes the lag at b = 0.8', ...
    '"reduce Ts from 0.1 s to 0.05 s removes the previously noted lag ... when b is 0.8"', ...
    sprintf('lag %+.3f s at Ts=%.3g vs %+.3f s at Ts=%.3g', L.fine_s, TSF, L.nom_s, TS), ...
    lagOK && L.fine_s < L.nom_s, ...
    'prose about Fig. 7.20, zarchan.txt:2173', ...
    'same lag metric, noise-free, on the halved sampling time');

% ---- 11: the chapter's miss-distance results are not scoreable -------------
B = addSkip(B, 'sigma and mean of miss vs flight time', 'Figs. 7.4, 7.5', ...
    ['Figs. 7.4/7.5 publish CURVES; the extraction turned every figure into ' ...
     'an image link, so no number survives.  The prose that accompanies them ' ...
     '(:2064) makes only an ORDERING claim -- noise-induced miss small vs ' ...
     'maneuver-induced miss -- and that ordering is the output of LISTING 7.2: ' ...
     '50 runs at each of 20 flight times, 1000 sorties.  This listing flies ' ...
     'ONE (miss here: ' sprintf('%.2f', R.nom.miss_ft) ' ft), and a one-draw ' ...
     'estimate of a standard deviation is not a check.  Widening a band to ' ...
     'absorb it would be worse than skipping it.'], ...
    'zarchan.txt:2028; Listing 7.2 at :2065-2082', 'ft');
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want.  Schema is the
%   one mgs.zarchan.benchSuite's shared benchRows() consumes.
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
p = R.p;
L = {};
L{end+1} = '';
L{end+1} = 'mgs.zarchan.fadingMemoryHoming -- Zarchan Listing 7.1 (zarchan.txt:2007-2057)';
L{end+1} = '  homing loop with a two-state digital FADING MEMORY FILTER on the LOS angle';
L{end+1} = sprintf('  UNITS: %s', p.units);
L{end+1} = sprintf('  VC=%g ft/s  nT=%g ft/s^2 (%.3g g)  N''=%g  sigma=%g rad (%.3g mr)', ...
                   p.VC, p.XNT, p.XNT/32.2, p.XNP, p.SIGNOISE, p.SIGNOISE*1000);
L{end+1} = sprintf('  b=%g (G=%.4g, H=%.4g)   b_fast=%g   Ts=%g s   Ts_fine=%g s   H=%g s   Ts/H=%g', ...
                   p.BETA, p.GFILTER, p.HFILTER, p.BETA_FAST, p.TS, p.TS_FINE, p.H, p.TS/p.H);
L{end+1} = sprintf('  TF=%g s   seed=%d (Lehmer + Box-Muller, local to this file)', p.TF, p.seed);
L{end+1} = '';
L{end+1} = sprintf('  miss (single sortie, noisy): b=%g -> %8.3f ft    b=%g -> %8.3f ft', ...
                   p.BETA, R.nom.miss_ft, p.BETA_FAST, R.fast.miss_ft);
L{end+1} = sprintf('  LOS-rate lag, measured / closed form [s]:');
L{end+1} = sprintf('     b=%-4g Ts=%-5g   %+7.3f / %6.3f', p.BETA,      p.TS,      R.lag.nom_s,  R.lag.nom_cf_s);
L{end+1} = sprintf('     b=%-4g Ts=%-5g   %+7.3f / %6.3f', p.BETA_FAST, p.TS,      R.lag.fast_s, R.lag.fast_cf_s);
L{end+1} = sprintf('     b=%-4g Ts=%-5g   %+7.3f / %6.3f', p.BETA,      p.TS_FINE, R.lag.fine_s, R.lag.fine_cf_s);
L{end+1} = sprintf('  noise transmission std(noisy-clean) [rad/s]: b=%g %.3e, b=%g %.3e, ratio %.2f', ...
                   p.BETA, R.noise_tx.nom, p.BETA_FAST, R.noise_tx.fast, R.noise_tx.ratio);
L{end+1} = '';
L{end+1} = '  BENCHMARKS (book values; source line numbers valid in zarchan.txt)';
for i = 1:numel(R.bench)
    b = R.bench(i);
    switch b.kind
        case 'value'
            L{end+1} = sprintf('   %-4s %-46s book %-10s ours %-10s (%+.3g%%) [%s]', ...
                vstr(b.ok), b.what, b.book_str, b.ours_str, b.pct, b.units); %#ok<AGROW>
        case {'shape','bound'}
            L{end+1} = sprintf('   %-4s %-46s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
            L{end+1} = sprintf('        book: %s', b.book_str); %#ok<AGROW>
        case 'info'
            L{end+1} = sprintf('   INFO %-46s %s', b.what, b.ours_str); %#ok<AGROW>
        case 'skip'
            L{end+1} = sprintf('   SKIP %-46s %s', b.what, b.book_str); %#ok<AGROW>
    end
    L{end+1} = sprintf('        src : %s', b.source); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = '  STRUCTURAL (self-consistency, NOT book values)';
for i = 1:numel(R.selfcheck.names)
    L{end+1} = sprintf('   %-4s %-58s %s', vstr(R.selfcheck.oks(i)), ...
        R.selfcheck.names{i}, R.selfcheck.detail{i}); %#ok<AGROW>
end
L{end+1} = '';
if any(ev)
    L{end+1} = sprintf('  scored %d row(s); %d passed.', sum(ev), sum([R.bench(ev).ok]));
else
    L{end+1} = '  scored 0 rows -- NOTHING was compared against the book.';
end
s = strjoin(L, newline);
end

function v = vstr(ok)
if ok, v = 'ok'; else, v = 'FAIL'; end
end
