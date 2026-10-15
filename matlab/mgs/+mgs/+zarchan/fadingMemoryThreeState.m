function R = fadingMemoryThreeState(opts)
%MGS.ZARCHAN.FADINGMEMORYTHREESTATE  Zarchan Listing 7.4 -- engagement
%   simulation with a THREE-STATE fading memory filter, plus the Listing 7.1
%   TWO-STATE filter it is meant to be contrasted against.
%
%   R = mgs.zarchan.fadingMemoryThreeState()
%   R = mgs.zarchan.fadingMemoryThreeState('BETA', 0.3)
%   R = mgs.zarchan.fadingMemoryThreeState('noise', false, 'assert', false)
%
%   Line-by-line port of Listing 7.4 (zarchan.txt:2216-2263).  Listing 7.1's
%   two-state filter (zarchan.txt:2008-2057) is in this file too, run on the
%   SAME engagement at the SAME b, because the only interesting thing Listing
%   7.4 publishes is the contrast: the book states at :2250 that "the
%   three-state filter gains are different from the two-state filter gains",
%   and the whole payoff is Fig. 7.2 ("smooth but LAGS", :2036) against
%   Fig. 7.24 ("FOLLOWS the geometric line-of-sight rate", :2265) at the same
%   b = 0.8.  A port of 7.4 alone could not check that sentence at all.
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, g = 32.2 ft/s^2; ANGLES IN
%   RADIANS (SIGNOISE = 0.001 rad = 1 mr).  NOT SI.  The rest of mgs is
%   metres -- convert AT THE CALL SITE and say so there.
% ===========================================================================
%
%   THE TRANSCRIPTION, AND WHAT WAS DONE TO IT
%
%   matlab/zarchan/_listings/listing_07_04.m does not parse.  Its own header
%   reports NO unresolved variable-to-variable comparisons, and that is right:
%   every comparison in 7.4 is against a literal.  Three repairs, none of them
%   a judgement call:
%
%     1  "while T ,= (TF - 1e-5)"  -> "while T <= (TF - 1e-5)".  The OCR maps
%        "<" to "," throughout this book.  Shape-forced: T only increases.
%     2  "while STEP ,=1"          -> "while STEP <= 1" (already repaired by
%        the extractor).  The body sets STEP = 2 to leave.
%     3  "if S.=(TS - 1e-5)"       -> "if S >= (TS - 1e-5)".  "." is the OCR
%        of ">".  With "<=" the test fires on the very first step (S = H =
%        0.01 <= 0.0999), so the DIFFERENCE equations would run every H, not
%        every Ts -- contradicting :2026, "the ratio Ts/H must be a large
%        integer".  SELF-CHECK 1 is that repair made measurable: it counts
%        the records, which is TF/TS = 100 one way and TF/H = 1000 the other.
%     * The trailing plotting and "save datfil.txt output /ascii" are dropped.
%       ("/ascii" is the book's own typo for -ascii; nothing is written here.)
%
%   The third damage class -- a minus sign read as the digit 2 -- appears in
%   this chapter's PROSE (":2029" writes the delay operator z^-1 as "z21") but
%   nowhere in Listing 7.4's numbers: every constant on the listing's first
%   line is a plain positive literal, checked against the prose at :2064.
%
%   TWO QUIRKS REPRODUCED DELIBERATELY -- "tidying" either changes the answer
%   while leaving a perfectly smooth plot:
%     * RTM, XLAM and XLAMD reaching the filter block are left over from the
%       SECOND derivative pass, i.e. formed from the Euler-PREDICTED Y and YD,
%       not from the corrected ones a few lines later.  O(H), no symptom.
%     * The four filter assignments update IN PLACE and their ORDER is
%       load-bearing: YH reads the old YDH and old XNTH, YDH reads the old
%       XNTH, XNTH is updated last.  Reordering them is a different filter.
%
%   ENDGAME.  At the last record T = TF, TGO = 1e-5, so both XLAMD and
%   XLAMDH = (YH + YDH*TGO)/(VC*TGO^2) run to 1e6-1e10.  The listing's own
%   behaviour (the book hides it behind "axis([0 10 0 .05])"); NOT clipped
%   here, so every scored window stops at t = 9 s and says so in .detail.
%
%   NOISE.  randn is replaced by Box-Muller on a Lehmer stream inside this
%   file: a benchmark that moves when somebody else calls rng() is not a
%   benchmark, and seeding the global stream would reach into the caller's
%   session.  Seed in R.p.seed.  The lag contrast (row 8) is scored with the
%   noise OFF, so the ordering is about the filters, not about one draw.
%
%   WHAT IS AND IS NOT SCOREABLE.  Chapter 7's target-maneuver section
%   publishes NO number of its own -- Figs. 7.23-7.25 are pictures, and the
%   extraction turned every figure into an image link.  So the scored rows are
%   the filter GAINS (Table 7.1, :2011-:2020, which the listing's own first
%   line independently confirms), the two nominal constants stated in prose at
%   :2064, and two ORDERING claims the prose makes about the figures.  A small
%   claim, honestly sized; the INFO rows name what was refused and why.
%
%   OUTPUTS
%     R.t R.y R.xlamd R.xlamdh R.xnc_g R.xnt_g R.xnth_g   Listing 7.4, noise on
%     R.clean R.two R.two_noisy  the noise-free run and the Listing 7.1
%                                two-state runs used for the contrast
%     R.gains .G .H .K and .two = [G H];  R.bench;  R.selfcheck (.names/.oks/
%     .detail -- structural, NOT book values);  R.pass R.verdict R.p
%     R.summary R.units
%
%   See also MGS.ZARCHAN.BENCHSUITE, MGS.ZARCHAN.SHAPINGFILTERADJOINT.

arguments
    % Listing 7.4 line 1, verbatim (zarchan.txt:2216).
    opts.VC       (1,1) double {mustBeFinite, mustBePositive} = 4000.0  % ft/s
    opts.XNT      (1,1) double {mustBeFinite}                 = 96.6    % ft/s^2
    opts.VM       (1,1) double {mustBeFinite, mustBePositive} = 3000.0  % ft/s
    opts.HEDEG    (1,1) double {mustBeFinite}                 = 0.0     % deg
    opts.BETA     (1,1) double {mustBeFinite, mustBeNonnegative} = 0.8
    opts.XNP      (1,1) double {mustBeFinite, mustBePositive} = 3.0
    opts.SIGNOISE (1,1) double {mustBeFinite, mustBeNonnegative} = 0.001  % rad
    opts.TF       (1,1) double {mustBeFinite, mustBePositive} = 10.0    % s
    opts.TS       (1,1) double {mustBeFinite, mustBePositive} = 0.1     % s
    opts.H        (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s
    opts.YIC      (1,1) double {mustBeFinite}                 = 0.0     % ft
    % The listing's NOISE = 1.
    opts.noise    (1,1) logical = true
    opts.seed     (1,1) double {mustBeFinite, mustBeNonnegative} = 20260824
    % THE ONLY SWITCHES.  Not nargin: with an arguments block that declares
    % only name-value options MATLAB reports nargin = 0 however many were
    % passed, so an "if nargin == 0" mode here would never fire.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

if opts.BETA >= 1
    error('mgs:zarchan:fadingMemoryThreeState:beta', ...
        ['BETA = %g.  Table 7.1 (zarchan.txt:1964) defines b as "a constant ' ...
         'between zero and unity"; at b = 1 all three gains are zero and the ' ...
         'filter never updates -- it would run, smoothly, and estimate nothing.'], ...
        opts.BETA);
end
ratio = opts.TS/opts.H;
if abs(ratio - round(ratio)) > 1e-9 || round(ratio) < 2
    error('mgs:zarchan:fadingMemoryThreeState:ratio', ...
        ['TS/H = %g.  zarchan.txt:1990: "the ratio Ts/H must be a large ' ...
         'integer so that effects in between sampling instants are treated ' ...
         'properly".  A non-integer ratio drifts the sampling instants against ' ...
         'the integration grid and the run still completes, smoothly.'], ratio);
end
if opts.TF <= 2*opts.TS
    error('mgs:zarchan:fadingMemoryThreeState:short', ...
        'TF = %g s gives fewer than two filter samples at TS = %g s.', ...
        opts.TF, opts.TS);
end

b  = opts.BETA;
g3 = [1 - b^3, 1.5*((1 - b)^2)*(1 + b), 0.5*((1 - b)^3)];   % Table 7.1, :2017-:2020
g2 = [1 - b^2, (1 - b)^2];                                  % Table 7.1, :2011-:2012

% ---- the four runs.  Each is TF/H outer steps; they are cheap. -------------
main  = engageRun(3, g3, opts, opts.noise, opts.seed);   % Listing 7.4 as printed
clean = engageRun(3, g3, opts, false,      opts.seed);   % same, noise off
two   = engageRun(2, g2, opts, false,      opts.seed);   % Listing 7.1, noise off
twoN  = engageRun(2, g2, opts, opts.noise, opts.seed);   % Listing 7.1, noise on

R = struct();
R.t = main.t;  R.y = main.y;  R.xlamd = main.xlamd;  R.xlamdh = main.xlamdh;
R.xnc_g = main.xnc_g;  R.xnt_g = main.xnt_g;  R.xnth_g = main.xnth_g;
R.clean = clean;  R.two = two;  R.two_noisy = twoN;
R.gains = struct('G', g3(1), 'H', g3(2), 'K', g3(3), 'two', g2);
R.units = 'ft, ft/s, ft/s^2 (g = 32.2); LOS angle in rad, LOS rate in rad/s';
R.p = struct('VC',opts.VC, 'XNT',opts.XNT, 'VM',opts.VM, 'HEDEG',opts.HEDEG, ...
             'BETA',b, 'XNP',opts.XNP, 'SIGNOISE',opts.SIGNOISE, 'TF',opts.TF, ...
             'TS',opts.TS, 'H',opts.H, 'YIC',opts.YIC, 'noise',opts.noise, ...
             'seed',opts.seed, 'nsamp',numel(main.t), 'units',R.units);

R.selfcheck = structuralChecks(opts, g3, g2, main, clean);

if opts.benchmark
    R.bench = benchmarks(R, opts, g3, g2, main, clean, two);
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
if opts.plot,   drawCurves(R, opts); end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false: nothing was compared. R.pass = false means NOT CHECKED.';
elseif ~any(ev)
    msg = 'not one book value could be evaluated; an unvalidated port must not be handed back silently.';
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question. First: %s.'], ...
        sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf(['%d of %d evaluated book values are outside tolerance ' ...
        '(first: %s, book %s, ours %s).'], numel(bad), sum(ev), ...
        R.bench(bad(1)).what, R.bench(bad(1)).book_str, R.bench(bad(1)).ours_str);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:fadingMemoryThreeState:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN, complete.  benchSuite runs every port with assert
        % off precisely so that one red row cannot hide the board.
        warning('mgs:zarchan:fadingMemoryThreeState:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function out = engageRun(order, gains, opts, useNoise, seed)
%ENGAGERUN  Listing 7.4 (order 3, zarchan.txt:2216-2263) and Listing 7.1
%   (order 2, zarchan.txt:2008-2057).  The differential-equation half is
%   IDENTICAL in the two listings; only the filter block differs, which is why
%   they share one loop here.
%
%   Every constant arrives as an ARGUMENT.  MATLAB local functions cannot see
%   the caller's workspace, and a previous wave in this package lost a whole
%   run to exactly that.

VC = opts.VC;  XNT = opts.XNT;  XNP = opts.XNP;  TF = opts.TF;
TS = opts.TS;  H  = opts.H;     SIGNOISE = opts.SIGNOISE;

GFILTER = gains(1);  HFILTER = gains(2);
if order == 3
    KFILTER = gains(3);
else
    KFILTER = 0;
end

% The listing's line 1, verbatim.
Y   = opts.YIC;
YD  = -opts.VM*opts.HEDEG/57.3;       % the book's own 57.3 deg/rad
T   = 0.0;  S = 0.0;
YH  = 0.0;  YDH = 0.0;  XNTH = 0.0;   % three-state filter states
XLAMH = 0.0;  XLAMDH = 0.0;           % two-state filter states
XNC = 0.0;
% Initialised before the loop so the FLAG==1 branch cannot read a leftover.
YDD = 0.0;  XLAM = 0.0;  XLAMD = 0.0;  RTM = 0.0;  TGO = TF;

nmax = ceil(TF/TS) + 4;
tA = zeros(nmax,1);  yA = zeros(nmax,1);  ldA = zeros(nmax,1);
lhA = zeros(nmax,1); ncA = zeros(nmax,1); nhA = zeros(nmax,1);
n = 0;

zn = lcgNormal(seed, nmax);      % one draw per SAMPLE, not per integration step
kz = 0;

while T <= (TF - 1e-5)
    YOLD = Y;  YDOLD = YD;  STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor.  T advances INSIDE this loop, exactly
    % once; move that line out and the scheme quietly drops to first order.
    while STEP <= 1
        if FLAG == 1
            Y  = Y + H*YD;
            YD = YD + H*YDD;
            T  = T + H;
            STEP = 2;
        end
        TGO   = TF - T + 0.00001;
        RTM   = VC*TGO;
        XLAM  = Y/(VC*TGO);
        XLAMD = (RTM*YD + Y*VC)/(RTM^2);
        YDD   = XNT - XNC;
        FLAG  = 1;
    end

    % Corrector, in the listing's own algebra.
    Y  = 0.5*(YOLD  + Y  + H*YD);
    YD = 0.5*(YDOLD + YD + H*YDD);

    S = S + H;
    if S >= (TS - 1e-5)              % OCR repair 3; see the header
        S = 0.0;
        kz = kz + 1;
        if useNoise
            XLAMNOISE = SIGNOISE*zn(kz);
        else
            XLAMNOISE = 0.0;
        end

        if order == 3
            % Listing 7.4.  YSTAR = RTM*(XLAM + noise) = Y + VC*TGO*noise:
            % the angle noise is multiplied by RANGE, so unlike Listing 4.5
            % VC does NOT cancel out of this measurement.  RTM and XLAM here
            % are the SECOND-PASS (predicted) values -- see the header.
            YSTAR = RTM*(XLAM + XLAMNOISE);
            RES   = YSTAR - YH - TS*YDH - 0.5*TS*TS*(XNTH - XNC);
            % ORDER IS LOAD-BEARING: YH reads the old YDH and old XNTH, YDH
            % reads the old XNTH, and XNTH is updated last.
            YH    = GFILTER*RES + YH + TS*YDH + 0.5*TS*TS*(XNTH - XNC);
            YDH   = HFILTER*RES/TS + YDH + TS*(XNTH - XNC);
            XNTH  = 2.0*KFILTER*RES/(TS*TS) + XNTH;
            XLAMDH = (YH + YDH*TGO)/(VC*TGO*TGO);
        else
            % Listing 7.1.  The two-state filter runs on the ANGLE itself and
            % has no acceleration state at all.
            RES    = XLAM - (XLAMH + TS*XLAMDH) + XLAMNOISE;
            XLAMH  = GFILTER*RES + XLAMH + TS*XLAMDH;
            XLAMDH = HFILTER*RES/TS + XLAMDH;
        end
        XNC = XNP*VC*XLAMDH;

        n = n + 1;
        tA(n) = T;  yA(n) = Y;  ldA(n) = XLAMD;  lhA(n) = XLAMDH;
        ncA(n) = XNC/32.2;      % the listing's ArrayXNCG
        nhA(n) = XNTH/32.2;     % the listing's ArrayXNTHG
    end
end

out = struct('order',order, 't',tA(1:n), 'y',yA(1:n), 'xlamd',ldA(1:n), ...
             'xlamdh',lhA(1:n), 'xnc_g',ncA(1:n), 'xnth_g',nhA(1:n), ...
             'xnt_g',repmat(XNT/32.2, n, 1), 'nsamp',n, 'ndraw',kz);
if order == 2
    out.xnth_g = nan(n,1);      % Listing 7.1 estimates no target maneuver
end
end


% =============================================================================
function z = lcgNormal(seed, n)
%LCGNORMAL  Box-Muller on a MINSTD (Lehmer) stream.  Base MATLAB,
%   deterministic, and it never touches the caller's global random stream.
%   16807*x < 3.61e13 < 2^53, so every product is exact in double precision;
%   x = 0 is absorbing, so the seed is mapped into 1..m-1.
m = 2147483647;
x = mod(floor(abs(seed)), m - 1) + 1;
np = 2*ceil(max(n,1)/2);
u = zeros(1, np);
for k = 1:np
    x = mod(16807*x, m);
    u(k) = x/m;
end
u1 = max(u(1:2:end), realmin);      % log(0) guard
u2 = u(2:2:end);
r  = sqrt(-2*log(u1));
z  = zeros(1, np);
z(1:2:end) = r.*cos(2*pi*u2);
z(2:2:end) = r.*sin(2*pi*u2);
z = z(1:n);
end


% =============================================================================
function c = charInvariants(gains)
%CHARINVARIANTS  Coefficients of the characteristic polynomial of the
%   three-state filter's ESTIMATION-ERROR transition matrix, as
%   c = [trace, sum of principal 2x2 minors, determinant].  With Zarchan's
%   Table 7.1 gains these are exactly [3b, 3b^2, b^3] -- the polynomial is
%   (z - b)^3, which is what "fading memory factor b" MEANS.
%
%   The matrix is A = (I - Gv*C)*F with F = [1 Ts Ts^2/2; 0 1 Ts; 0 0 1],
%   Gv = [G; H/Ts; 2K/Ts^2] and C = [1 0 0].  Working that out by hand, EVERY
%   Ts CANCELS -- e.g. the (1,2) minor is (1-G)(1-H) - (1-G)Ts*(-H/Ts) = 1-G:
%
%        c1 = 3 - G - H - K
%        c2 = 3 - 2G - H + K
%        c3 = 1 - G
%
%   so the closed form is used and no matrix is built.  Two reasons, and the
%   second is the load-bearing one:
%     * Ts-independence is itself part of the claim; deriving it makes the
%       check say something about the GAINS rather than about a sample rate.
%     * *** NOT eig(). ***  A triple eigenvalue on a companion-like matrix is
%       DEFECTIVE, so eig() perturbs it by about eps^(1/3) ~ 6e-6, and a
%       CORRECT port would fail a 1e-9 tolerance for purely numerical
%       reasons.  These three combinations are exact to a few ulp.
%
%   The map (G,H,K) -> (c1,c2,c3) is invertible (its matrix has determinant
%   2), so matching all three is equivalent to having all three gains right.
G = gains(1);  Hf = gains(2);  K = gains(3);
c = [3 - G - Hf - K, 3 - 2*G - Hf + K, 1 - G];
end


% =============================================================================
function r = winRms(t, v, t0, t1)
%WINRMS  RMS of v over t in [t0, t1].  Inf when the window is empty, so a
%   window that does not exist reports a FAILURE rather than a perfect zero.
w = t >= t0 & t <= t1;
if ~any(w)
    r = Inf;
    return
end
r = sqrt(mean(v(w).^2));
end

function m = winMean(t, v, t0, t1)
w = t >= t0 & t <= t1;
if ~any(w)
    m = NaN;
    return
end
m = mean(v(w));
end


% =============================================================================
function K = structuralChecks(opts, g3, g2, main, clean)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never to be quoted
%   as evidence that this port reproduces Zarchan.  Two of the five are DELETE
%   EXPERIMENTS, and all five are in .names/.oks so that they are actually
%   VISIBLE on the board -- a delete experiment nobody can see is one that
%   did not run.
K = struct();
names = {};  oks = [];  detail = {};
b = opts.BETA;
cWant = [3*b, 3*b^2, b^3];

% 1  The "if S >= TS - 1e-5" repair, made measurable.  Under the other reading
%    of the OCR'd "." the difference equations would run every H, and this
%    count would be TF/H rather than TF/TS.
want = round(opts.TF/opts.TS);
K.nsamp = main.nsamp;
names{end+1} = 'records TF/TS samples (the "S >= TS" repair)';
oks(end+1)   = (main.nsamp == want) && (main.ndraw == want);
detail{end+1} = sprintf('%d records, %d noise draws; want %d (TF/H would be %d)', ...
    main.nsamp, main.ndraw, want, round(opts.TF/opts.H));

% 2  Table 7.1's three-state gains make the error dynamics (z - b)^3.  Exact
%    algebraic identity, sensitive to G, H and K SEPARATELY.
K.char_resid = max(abs(charInvariants(g3) - cWant));
names{end+1} = 'three-state error dynamics == (z - BETA)^3';
oks(end+1)   = K.char_resid <= 1e-12;
detail{end+1} = sprintf('max |c - [3b, 3b^2, b^3]| = %.3e (b = %.3f)', K.char_resid, b);

% 3  DELETE EXPERIMENT: substitute the TWO-STATE H = (1-b)^2 into the
%    three-state filter -- precisely the slip the book warns about at :2250.
%    The characteristic polynomial must MOVE.  Algebraically forced: it is
%    G + H + K = 3(1-b) that makes the trace 3b, and the swap breaks that sum.
K.hswap_resid = max(abs(charInvariants([g3(1) g2(2) g3(3)]) - cWant));
names{end+1} = 'DELETE: two-state H substituted => char. poly moves off (z-b)^3';
oks(end+1)   = K.hswap_resid > 1e-3;
detail{end+1} = sprintf('max |c - want| = %.4f with H = (1-b)^2 = %.4g (correct H = %.4g)', ...
    K.hswap_resid, g2(2), g3(2));

% 4  DELETE EXPERIMENT: KFILTER = 0.  The third state then never moves, the
%    maneuver estimate is identically zero, and the late-window estimation
%    error must rise to the full target maneuver.  This is what shows the
%    third state does the work rather than decorating the loop.
z     = engageRun(3, [g3(1) g3(2) 0], opts, opts.noise, opts.seed);
errK0 = winRms(z.t,    z.xnth_g    - z.xnt_g,    5, 9);
errK  = winRms(main.t, main.xnth_g - main.xnt_g, 5, 9);
K.k0_err = errK0;  K.k_err = errK;
names{end+1} = 'DELETE: KFILTER = 0 => no maneuver estimate at all';
oks(end+1)   = (max(abs(z.xnth_g)) == 0) && (errK0 > errK);
detail{end+1} = sprintf('max |nT_hat| = %.3g g; late rms err %.4f g (K=0) vs %.4f g (K=%.4g)', ...
    max(abs(z.xnth_g)), errK0, errK, g3(3));

% 5  With the noise off the seed is unreachable.  Catches a stray rand/randn
%    leaking into a run that is supposed to be deterministic.
alt = engageRun(3, g3, opts, false, opts.seed + 977);
K.seed_resid = max(abs(alt.y - clean.y));
names{end+1} = 'noise off => bit-identical across seeds';
oks(end+1)   = K.seed_resid == 0;
detail{end+1} = sprintf('max |dY| = %.3e ft between seeds %g and %g', ...
    K.seed_resid, opts.seed, opts.seed + 977);

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
function B = benchmarks(R, opts, g3, g2, main, clean, two)
%BENCHMARKS  Computed vs published.  EVERY book value here was grepped out of
%   zarchan_decoded.txt and its line number is in .source.  Nothing was read
%   off a figure: the extraction turned every figure into an image link, and
%   phantom "figure readings" have been caught in this package before.
%
%   Bands:
%     'table'  1e-9  an algebraic identity from Table 7.1 evaluated at the b
%                    the listing's own first line sets.  Nothing to round.
%     'text'   1e-9  a constant printed in prose AND in the listing.
%     'shape'        an ordering; no tolerance exists or is possible.
%     'bound'        an inequality, with the threshold declared in the row's
%                    own .detail as OURS, not the book's.
B = emptyBench();
b   = opts.BETA;
tbl = 'Table 7.1 (zarchan.txt:1981-2020) + Listing 7.4 line 1 (zarchan.txt:2216)';
tb2 = 'Table 7.1 (zarchan.txt:1975-2012) + Listing 7.1 line 1 (zarchan.txt:2008)';
atB = abs(b - 0.8) < 1e-12;
whyB = sprintf(['the gain VALUES are quoted for the nominal b = 0.8 ' ...
    '(zarchan.txt:2229); this run used b = %g.  The FORMULA is still the ' ...
    'book''s -- only the arithmetic below is off-case, and SELF-CHECK 2 ' ...
    'still holds it to (z - b)^3 at whatever b was asked for'], b);

% ---- 1-3  the three-state gains --------------------------------------------
if atB
    B = addVal(B, 'GFILTER = 1 - b^3  at b = 0.8', 0.488, g3(1), 'table', 1e-9, tbl, '-', ...
        ['Table 7.1 and the listing''s own first line agree, so the OCR''s ' ...
         'lost superscript in "G = 1 - b3" is confirmed, not guessed']);
    B = addVal(B, 'HFILTER = 1.5(1-b)^2(1+b)  at b = 0.8', 0.108, g3(2), 'table', 1e-9, tbl, '-', ...
        ['THE row the book''s warning at :2250 is about: the two-state H at ' ...
         'the same b is 0.04, a factor of 2.7 away, and both are plausible ' ...
         'small numbers that would plot smoothly']);
    B = addVal(B, 'KFILTER = 0.5(1-b)^3  at b = 0.8', 0.004, g3(3), 'table', 1e-9, tbl, '-', ...
        'the gain that exists ONLY in the three-state filter; SELF-CHECK 4 deletes it');
else
    B = addSkip(B, 'GFILTER = 1 - b^3  at b = 0.8', '0.488', whyB, tbl, '-');
    B = addSkip(B, 'HFILTER = 1.5(1-b)^2(1+b)  at b = 0.8', '0.108', whyB, tbl, '-');
    B = addSkip(B, 'KFILTER = 0.5(1-b)^3  at b = 0.8', '0.004', whyB, tbl, '-');
end

% ---- 4-5  the two-state gains, i.e. the thing being contrasted against -----
if atB
    B = addVal(B, 'two-state GFILTER = 1 - b^2  at b = 0.8', 0.36, g2(1), 'table', 1e-9, tb2, '-', ...
        ['Listing 7.1''s filter, run here at b = 0.8 -- Fig. 7.2''s nominal ' ...
         'case (:2036) -- rather than at its own listing''s 0.3, so that the ' ...
         'contrast row below is apples-to-apples']);
    B = addVal(B, 'two-state HFILTER = (1-b)^2  at b = 0.8', 0.04, g2(2), 'table', 1e-9, tb2, '-', ...
        ['if this ever equalled 0.108 the contrast row would be comparing ' ...
         'the three-state filter with itself and would pass forever']);
else
    B = addSkip(B, 'two-state GFILTER = 1 - b^2  at b = 0.8', '0.36', whyB, tb2, '-');
    B = addSkip(B, 'two-state HFILTER = (1-b)^2  at b = 0.8', '0.04', whyB, tb2, '-');
end

% ---- 6-7  the nominal case, stated in prose AND in the listing -------------
B = addVal(B, 'target maneuver XNT', 3.0, opts.XNT/32.2, 'text', 1e-9, ...
    'p.142 text (zarchan.txt:2028) "a constant 3-g target maneuver" + listing (zarchan.txt:2216)', 'g', ...
    ['XNT = 96.6 ft/s^2 is exactly 3 g at the book''s g = 32.2.  A silent ' ...
     'ft->m would make this 0.93 and it would still read as a number']);
B = addVal(B, 'LOS-angle measurement noise SIGNOISE', 1.0, 1000*opts.SIGNOISE, 'text', 1e-9, ...
    'p.142 text (zarchan.txt:2028) "1 milliradian (mr) of measurement noise" + listing (zarchan.txt:2216)', 'mr', ...
    ['RADIANS in the code.  YSTAR multiplies this by RANGE, so it is 40 ft ' ...
     'of pseudo-position noise at t = 0 and only 4 ft at t = 9 s']);

% ---- 8  THE ROW THIS FILE EXISTS FOR: three-state vs two-state -------------
% :2036 (Fig. 7.2, TWO-state, b = 0.8): "smooth but LAGS the actual
%   line-of-sight rate, indicating that the filter is sluggish".
% :2265 (Fig. 7.24, THREE-state, same b): "the filter estimate FOLLOWS the
%   geometric line-of-sight rate".
% Scored NOISE-FREE, so the ordering is about the filters and not about one
% draw, and over t = 1..8 s: before 1 s neither filter has started, and past
% 9 s TGO -> 1e-5 makes estimate and truth both diverge (see the header).
e3 = winRms(clean.t, clean.xlamdh - clean.xlamd, 1, 8) / ...
     winRms(clean.t, clean.xlamd, 1, 8);
e2 = winRms(two.t,   two.xlamdh   - two.xlamd,   1, 8) / ...
     winRms(two.t,   two.xlamd,   1, 8);
B = addShape(B, 'three-state tracks LOS rate; two-state at same b lags', ...
    'three-state relative tracking error STRICTLY smaller than two-state at the same b', ...
    sprintf('%.4f (3-state) vs %.4f (2-state), %.2fx', e3, e2, e2/max(e3, realmin)), ...
    e3 < e2, 'p.140/p.161 text (zarchan.txt:2000 vs :2265)', ...
    ['each filter is scored against ITS OWN run''s geometric XLAMD, which ' ...
     'is what Figs. 7.2 and 7.24 plot; noise off, window t = 1..8 s']);

% ---- 9  the maneuver estimate converges ------------------------------------
% :2266, about Fig. 7.25: "it takes the filter about 5 s to get a reasonable
% estimate of the maneuver level".  Scored on the LISTING'S OWN case (noise
% on), as a pure ordering -- no invented constant.
errEarly = winRms(main.t, main.xnth_g - main.xnt_g, 0, 4);
errLate  = winRms(main.t, main.xnth_g - main.xnt_g, 5, 9);
B = addShape(B, 'maneuver estimate converges: late error < early error', ...
    '"about 5 s to get a reasonable estimate" => rms error over 5..9 s below that over 0..4 s', ...
    sprintf('%.4f g late vs %.4f g early (%.2fx)', errLate, errEarly, ...
            errEarly/max(errLate, realmin)), ...
    errLate < errEarly, 'p.161 text (zarchan.txt:2230)', ...
    ['noise ON -- the listing''s own NOISE = 1, seed in R.p.seed.  Pure ' ...
     'ordering: the book gives no number for "reasonable"']);

% ---- 10  and it converges to the RIGHT level -------------------------------
mNoisy = winMean(main.t,  main.xnth_g,  5, 9);
mClean = winMean(clean.t, clean.xnth_g, 5, 9);
nTg    = opts.XNT/32.2;
okLvl  = isfinite(mNoisy) && nTg ~= 0 && abs(mNoisy/nTg - 1) <= 0.25;
B = addBound(B, 'maneuver estimate reaches the right level by t = 5 s', ...
    'mean nT_hat over t = 5..9 s within 25% of the true 3 g', ...
    sprintf('%.4f g (%+.2f%%); noise-free counterpart %.4f g (%+.2f%%)', ...
            mNoisy, 100*(mNoisy/nTg - 1), mClean, 100*(mClean/nTg - 1)), ...
    okLvl, 'p.161 text (zarchan.txt:2230)', 'g', ...
    ['*** THE 25% IS OURS ***, not the book''s: "reasonable" is an adjective. ' ...
     'Averaging the 40 samples in 5..9 s is what makes this a statement ' ...
     'about the estimate rather than about the draw, and the noise-free ' ...
     'counterpart is printed beside it.  Never widen it to turn the row green']);

% ---- INFO.  Reported, never scored. ---------------------------------------
nt3 = winRms(main.t, main.xlamdh - clean.xlamdh, 1, 8);
nt2 = winRms(R.two_noisy.t, R.two_noisy.xlamdh - two.xlamdh, 1, 8);
B = addInfo(B, 'noise transmission of the LOS-rate estimate', ...
    '"without excessive noise transmission"', ...
    sprintf('rms(noisy - clean) = %.3e (3-state) vs %.3e rad/s (2-state), t = 1..8 s', nt3, nt2), ...
    'p.161 text (zarchan.txt:2229); cf. :2037', ...
    ['NOT SCORED: "excessive" has no published threshold, and :2037 says ' ...
     'the comparison runs the other way as b is lowered, so a direction ' ...
     'fixed at b = 0.8 would not generalise']);

idx = find(main.t <= 9);
[pk, ip] = max(abs(main.xlamd(idx)));
B = addInfo(B, 'the listing''s axis([0 10 0 .05]) plot box', '0 .05 rad/s', ...
    sprintf('max |XLAMD| over t <= 9 s is %.4g rad/s at t = %.1f s; the final record at t = TF is %.3g', ...
            pk, main.t(idx(ip)), main.xlamd(end)), ...
    'Listing 7.4 plotting lines (zarchan.txt:2227)', ...
    ['NOT SCORED: an axis range is a plot box the author chose, not a ' ...
     'measurement.  The final record blows up because TGO -> 1e-5; the ' ...
     'book''s axis merely hides it']);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER must match the adders exactly; this is the
%   schema mgs.zarchan.benchSuite's adaptGeneric consumes, and it is STRICT.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.4g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.6g', ours);
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
%ADDINFO  Reported, never scored: an INFO row can neither pass nor fail.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'info';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'info';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = '-';  B(i).detail = '';
end

function B = addSkip(B, what, bookStr, why, source, units)
%ADDSKIP  A check that COULD NOT RUN.  Never a pass.
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
L{end+1} = 'Zarchan Listing 7.4 -- ENGAGEMENT WITH A THREE-STATE FADING MEMORY FILTER';
L{end+1} = '   (with Listing 7.1''s two-state filter, the thing it is contrasted against)';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2; ANGLES IN RADIANS.  NOT SI. ***';
L{end+1} = sprintf(['   VC = %g ft/s   nT = %g ft/s^2 (%.4g g)   N'' = %g   ' ...
    'b = %g   TS = %g s   H = %g s   TF = %g s'], p.VC, p.XNT, p.XNT/32.2, ...
    p.XNP, p.BETA, p.TS, p.H, p.TF);
L{end+1} = sprintf(['   gains: three-state G = %.6g  H = %.6g  K = %.6g   |   ' ...
    'two-state G = %.6g  H = %.6g'], R.gains.G, R.gains.H, R.gains.K, ...
    R.gains.two(1), R.gains.two(2));
if p.noise
    L{end+1} = sprintf('   noise: SIGNOISE = %g rad (%g mr), seed %g, %d samples', ...
        p.SIGNOISE, 1000*p.SIGNOISE, p.seed, p.nsamp);
else
    L{end+1} = sprintf('   noise: OFF (''noise'', false), %d samples', p.nsamp);
end
L{end+1} = '';
L{end+1} = sprintf('%8s %12s %12s %12s %12s %12s', 't [s]', 'Y [ft]', ...
    'XLAMD', 'XLAMDH(3)', 'XLAMDH(2)', 'nT_hat [g]');
stride = max(1, round(0.5/p.TS));
rows = unique([1:stride:numel(R.t), numel(R.t)]);
for r = rows
    L{end+1} = sprintf('%8.2f %12.4f %12.5g %12.5g %12.5g %12.4f', ...
        R.t(r), R.y(r), R.xlamd(r), R.xlamdh(r), R.two_noisy.xlamdh(r), ...
        R.xnth_g(r)); %#ok<AGROW>
end
L{end+1} = '   (the last row is t = TF, where TGO = 1e-5 and the LOS-rate columns';
L{end+1} = '    diverge.  That is the listing''s own behaviour; no window scores it.)';
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-56s %s', vstr(K.oks(i)), K.names{i}, ...
        K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false means NOT';
    L{end+1} = '            CHECKED, which is not the same thing as "fine".';
else
    L{end+1} = 'BENCHMARKS  (book values grepped from zarchan_decoded.txt; lines in .source)';
    for i = 1:numel(R.bench)
        bb = R.bench(i);
        switch lower(bb.kind)
            case 'info'
                L{end+1} = sprintf('   INFO %-52s %s', bb.what, bb.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', bb.why); %#ok<AGROW>
            case 'skip'
                L{end+1} = sprintf('   SKIP %-52s %s', bb.what, bb.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', bb.why); %#ok<AGROW>
            case {'shape','bound'}
                L{end+1} = sprintf('   %-4s %-52s %s', vstr(bb.ok), bb.what, bb.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', bb.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', bb.source, bb.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf('   %-4s %-52s book %s %s / ours %s %s (%+.3g%%)', ...
                    vstr(bb.ok), bb.what, bb.book_str, bb.units, bb.ours_str, ...
                    bb.units, bb.pct); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', bb.source, bb.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped/info -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: Ch.7''s target-maneuver section publishes NO number of its own --';
L{end+1} = '   Figs. 7.23-7.25 are pictures.  Scored here: the GAINS, the two nominal';
L{end+1} = '   constants in prose, and two ORDERING claims.  Nothing off a figure.';
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
function drawCurves(R, opts)
%DRAWCURVES  The listing's own two figures (7.24 and 7.25), base graphics.
%   The book's axis([0 10 0 .05]) is applied to the first, but it is NOT
%   allowed to clip silently: the endgame record is excluded by t <= TF - TS
%   and the title says so.
w = R.t <= opts.TF - opts.TS;
figure; hold on; grid on;
plot(R.t(w), R.xlamd(w), '-');
plot(R.t(w), R.xlamdh(w), '--');
plot(R.two_noisy.t(w), R.two_noisy.xlamdh(w), ':');
xlabel('Time (S)'); ylabel('Line of Sight Rate (Rad/S)');
title(sprintf('Fig. 7.24 vs Fig. 7.2, b = %g (last record at t = TF omitted)', ...
    opts.BETA));
legend('actual XLAMD', 'three-state estimate', ...
    'two-state estimate (Listing 7.1)', 'Location', 'best');
axis([0 opts.TF 0 0.05]); hold off;

figure; hold on; grid on;
plot(R.t(w), R.xnt_g(w), '-');
plot(R.t(w), R.xnth_g(w), '--');
xlabel('Time (S)'); ylabel('Acceleration (G)');
title('Fig. 7.25 -- three-state filter estimate of the target maneuver');
legend('actual nT', 'estimate nT hat', 'Location', 'best'); hold off;
end
