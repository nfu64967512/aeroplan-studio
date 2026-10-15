function R = gravityTurn(opts)
%MGS.ZARCHAN.GRAVITYTURN  Zarchan Listing 12.2 -- two-stage booster sizing plus
%   the gravity-turn trajectory that booster actually flies.
%
%   R = mgs.zarchan.gravityTurn()
%   R = mgs.zarchan.gravityTurn('GAMDEG', 65, 'AMAX1', 20, 'AMAX2', 20)
%   R = mgs.zarchan.gravityTurn('assert', false)      % warn instead of error
%
%   Port of Listing 12.2 (zarchan.txt:3760-3773), integrated by the book's own
%   second-order Runge-Kutta (Heun) scheme.  The listing is two programs: its
%   first line is Listing 12.1's closed-form staging arithmetic (propellant,
%   structure, thrust, burn times) and the rest flies that design through a
%   gravity turn in an Earth-centred plane.  It is the boost trajectory a
%   ballistic missile flies, and the target model Chapters 14 and 33 reuse
%   (zarchan.txt:4322, :12594).
%
%   *** UNITS ARE THE BOOK'S: FEET, POUNDS, SECONDS, g = 32.2 ft/s^2, and
%   1 n.mi. = 6076 ft (the listing's own conversion, not 6076.1).  NOT SI.
%   The rest of mgs is metres; nothing is converted here, so that every number
%   printed lies straight against a published one.  Convert AT THE CALL SITE. ***
%
%   The gravity turn is OPEN LOOP -- nothing steers -- so the flight-path angle
%   is a boundary condition and nothing recovers it.  Below 80 deg the booster
%   falls over and hits the ground, smoothly, with no error and no warning and
%   a plausible-looking trajectory all the way down.  Benchmark 5 scores that.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_12_02.m does not parse.  Its header reports ONE
%   unresolved variable-to-variable comparison; there are two of the same kind.
%
%     1  "if T , TB1"            -> "if T < TB1"
%     2  "elseif T , (TB1+TB2)"  -> "elseif T < (TB1+TB2)"
%        WHICH WAY AND WHY: with "<", the schedule gives WGT(0) = WTOT (the
%        full 6169 lb stack), WGT(end) = WPAY, and two discontinuities equal to
%        exactly WS1 and WS2 -- structure dropped at staging, which is what the
%        Fig. 12.7 prose describes (zarchan.txt:3685).  With ">", T = 0 falls
%        through BOTH tests to the else branch: a 100 lb booster with zero
%        thrust that never lights.  Selfcheck 2 runs that flip as a delete
%        experiment and prints what it would have given.
%     3  "while STEP , =1" -> "while STEP <= 1"; "if S . =9.99999" ->
%        "if S >= 9.99999"; "while ~(ALT ,0 & T . 10)" -> "~(ALT < 0 & T > 10)".
%        "," is this book's OCR of "<" and "." of ">" throughout.
%     4  "BOOSTERS 269" mid-listing is the page header leaking in.  Dropped, as
%        are the plotting block and "save datfil output /ascii" (a typo for
%        -ascii; nothing is written to disk here at all).
%
%   Kept verbatim on purpose: V(0) = 1 ft/s and NOT zero (X1 = VRX, Y1 = VRY is
%   a UNIT vector, and VEL is a divisor in AXT = AT*X1/VEL, so a standstill is
%   0/0); ANG = ANGDEG/57.3 and the literal 1.5708 for pi/2, which deliver
%   84.99917 deg when 85 is asked for -- selfcheck 5 measures that rather than
%   assuming it.  NOT kept verbatim: CBETA is clamped to [-1,1] before acos(),
%   because at t = 0 it is 1 + 2e-16 and acos() then returns a COMPLEX
%   downrange that propagates silently into every recorded row.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS -- SIX rows; every book value was grepped out of zarchan.txt and
%   its line is in the row's .source field.  Nothing was read off a figure.
%
%     1  BOUND  the embedded Listing 12.1 line vs Table 12.1's Wtot, T1, T2,
%        tB1, tB2 (zarchan.txt:3691-3699), collapsed into ONE row.
%        WHY ONE AND NOT NINE: mgs.zarchan.thrustWeight is Listing 12.1 and
%        already scores all nine Table 12.1 entries separately.  Listing 12.2
%        embeds that same line verbatim, so re-scoring them here would add
%        nine greens for arithmetic already adjudicated once and make the
%        board look twice as verified as it is.  This row still catches a
%        corrupted embedded copy.  Bands are each printed figure's rounding
%        band: 0.1% at four significant figures, 0.2% for 32.2 s, 0.5% for 106.
%     2  VALUE  total burn time 69.1 s at amax = 20 g, quoted in CHAPTER 14
%        (zarchan.txt:4322) -- an independent statement of "half the time of
%        the nominal design" (:3785), and the one Chapter 12 number no other
%        port on the board scores.  Band 0.1%.
%     3  VALUE  max downrange, nominal design, gamma0 = 85 deg: "about
%        2300 n.mi." (:3786).
%     4  VALUE  max downrange, 20-g design, gamma0 = 65 deg: "about
%        2600 n.mi." (:3786).
%        BANDS FOR 3 AND 4 ARE 10%, the loosest here on purpose: both are the
%        author's own reading of his own figure, quoted with "about" and
%        rounded to two figures.  If one comes back red it is a real
%        disagreement about the trajectory, NOT an invitation to widen it.
%     5  SHAPE  ":3752 -- below 80 deg the booster immediately crashes",
%        scored as range(75 deg) < 0.5*range(85 deg) on the same design.
%     6  INFO   the burnout state, reported and never adjudicated.
%
%   NOT SCORED: Figs. 12.11/12.13 publish curves and no numbers beyond the two
%   "about" ranges.  The 20 kft/s goal (:3785) belongs to Listing 12.1's
%   drag-free ONE-dimensional integral -- thrustWeight scores it at 19999.75
%   ft/s -- not to this two-dimensional turn, which pays a gravity loss the
%   book never quantifies.  "About 4 g at the beginning" (:3752) is likewise
%   already on the board as thrustWeight's acceleration-envelope row.
%
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NEVER EXECUTED -- MATLAB is the
%   caller's. ***  The staging algebra (agreeing with thrustWeight's own
%   derived block) gives Wtot 6168.72 lb, Wp1 3473.67, Ws1 613.00, Wp2 1684.74,
%   Ws2 297.31, T1 26950.48 lb, T2 3973.07 lb, tB1 32.2227 s, tB2 106.0100 s,
%   burnout 138.2327 s; at 20 g, 16.1114 + 53.0050 = 69.1164 s, i.e. +0.024%
%   on row 2.  Liftoff acceleration T1/Wtot = 4.370 g.  GM/A^2 = 32.147 ft/s^2
%   against the listing's own 32.2, 0.16% apart.  Rows 3-5 need the
%   integration and are deliberately NOT predicted here: a hand-estimated
%   ballistic range would be a fabricated benchmark, which is the one thing
%   this package most needs not to produce.
%
%   R.design R.traj R.burnout R.range_nm R.tof_s R.hit_tmax R.bench R.selfcheck
%   R.pass R.verdict R.units R.p R.summary.  R.range_nm is taken at the FINAL
%   integrated state, not the last recorded sample (up to 'samp' earlier).
%
%   See also MGS.ZARCHAN.BENCHSUITE.
arguments
    % --- Listing 12.2 line 1, verbatim.  Pounds, seconds, feet.
    opts.XISP1  (1,1) double {mustBeFinite, mustBePositive} = 250
    opts.XISP2  (1,1) double {mustBeFinite, mustBePositive} = 250
    opts.XMF1   (1,1) double {mustBeFinite, mustBePositive} = 0.85
    opts.XMF2   (1,1) double {mustBeFinite, mustBePositive} = 0.85
    opts.WPAY   (1,1) double {mustBeFinite, mustBePositive} = 100
    opts.DELV   (1,1) double {mustBeFinite, mustBePositive} = 20000
    opts.AMAX1  (1,1) double {mustBeFinite, mustBePositive} = 10
    opts.AMAX2  (1,1) double {mustBeFinite, mustBePositive} = 10
    % --- gravity turn.  GAMDEG is the boundary condition the chapter is about.
    opts.GAMDEG (1,1) double {mustBeFinite} = 85
    opts.LEFT   (1,1) double {mustBeMember(opts.LEFT, [0 1])} = 1
    opts.ALTNM  (1,1) double {mustBeFinite, mustBeNonnegative} = 0
    opts.ANGDEG (1,1) double {mustBeFinite} = 90
    opts.H      (1,1) double {mustBeFinite, mustBePositive} = 0.01
    % The listing tests "S >= 9.99999", i.e. it records every 10 s.
    opts.samp   (1,1) double {mustBeFinite, mustBePositive} = 10
    % The listing has NO time guard (its only exit is ALT < 0 after t > 10 s),
    % so a near-orbital case would loop forever.  When the guard fires the
    % range rows SKIP rather than score a truncated trajectory.
    opts.tmax   (1,1) double {mustBeFinite, mustBePositive} = 4000
    % THE ONLY SWITCHES.  Not nargin: with an arguments block declaring only
    % name-value options, nargin is ALWAYS 0 -- such a mode would never fire.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

if opts.H >= opts.samp
    error('mgs:zarchan:gravityTurn:samp', ...
        ['H = %g s is not smaller than samp = %g s, so the recorded curve ' ...
         'would be sampled at the step rate while claiming the coarser ' ...
         'interval in its own time vector.'], opts.H, opts.samp);
end

% =========================================================================
%   Listing 12.2, line 1 (= Listing 12.1's staging arithmetic), verbatim.
% =========================================================================
D = boosterDesign(opts.XISP1, opts.XISP2, opts.XMF1, opts.XMF2, ...
                  opts.WPAY, opts.DELV, opts.AMAX1, opts.AMAX2);

% =========================================================================
%   Listing 12.2, the trajectory.  zarchan.txt:3762-3773.
% =========================================================================
TR = flyGravityTurn(D, opts.GAMDEG, opts.LEFT, opts.ALTNM, opts.ANGDEG, ...
                    opts.H, opts.samp, opts.tmax);

R = struct();
R.design    = D;
R.traj      = TR.rec;
R.burnout   = TR.burnout;
R.range_nm  = TR.range_nm;
R.alt_end_nm = TR.alt_end_nm;
R.tof_s     = TR.t_end;
R.hit_tmax  = TR.hit_tmax;
R.p = struct('XISP1',opts.XISP1, 'XISP2',opts.XISP2, 'XMF1',opts.XMF1, ...
             'XMF2',opts.XMF2, 'WPAY',opts.WPAY, 'DELV',opts.DELV, ...
             'AMAX1',opts.AMAX1, 'AMAX2',opts.AMAX2, 'GAMDEG',opts.GAMDEG, ...
             'LEFT',opts.LEFT, 'ALTNM',opts.ALTNM, 'ANGDEG',opts.ANGDEG, ...
             'H',opts.H, 'samp',opts.samp, 'tmax',opts.tmax, ...
             'A_ft',2.0926e7, 'GM',1.4077e16, 'FT_PER_NM',6076, ...
             'units','ft, lb, s; g = 32.2 ft/s^2; 1 n.mi. = 6076 ft');

R.units     = R.p.units;
R.selfcheck = structuralChecks(D, TR, opts.GAMDEG, opts.LEFT, opts.ANGDEG, ...
                               opts.WPAY, opts.samp);

if opts.benchmark
    [R.bench, R.aux] = benchmarks(R.p, TR);
    ev     = ~[R.bench.skipped];
    R.pass = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    R.aux   = struct();
    ev      = false;
    % NOT true.  "Not checked" must never render as "fine".
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawTrajectory(R); end

% ---- adjudication.  The verdict is FORMED either way; 'assert' chooses only
%      the reaction, so a red row never withholds the evidence to diagnose it.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated.  Chapter 12''s ' ...
        'published numbers are for Isp = 250 s, mf = 0.85, Wpay = 100 lb, ' ...
        'DV = 20000 ft/s; this run used %g/%g s, %g/%g, %g lb, %g ft/s.'], ...
        opts.XISP1, opts.XISP2, opts.XMF1, opts.XMF2, opts.WPAY, opts.DELV);
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
        error('mgs:zarchan:gravityTurn:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:gravityTurn:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function D = boosterDesign(XISP1, XISP2, XMF1, XMF2, WPAY, DELV, AMAX1, AMAX2)
%BOOSTERDESIGN  Listing 12.2 line 1 (zarchan.txt:3760-3761), verbatim.
%   Every constant is an ARGUMENT: MATLAB local functions cannot see the main
%   function's workspace.
DELV1 = 0.3333*DELV;
DELV2 = 0.6667*DELV;

TOP2  = WPAY*(exp(DELV2/(XISP2*32.2)) - 1.);
BOT2  = 1/XMF2 - ((1.-XMF2)/XMF2)*exp(DELV2/(XISP2*32.2));
if BOT2 <= 0
    error('mgs:zarchan:gravityTurn:stage2', ...
        ['stage-2 denominator is %.4g <= 0: the required mass ratio %.4g ' ...
         'exceeds what a fuel mass fraction of %g can deliver.  The formula ' ...
         'returns a NEGATIVE propellant weight and the simulation then flies ' ...
         'happily on it.'], BOT2, exp(DELV2/(XISP2*32.2)), XMF2);
end
WP2   = TOP2/BOT2;
WS2   = WP2*(1-XMF2)/XMF2;
WTOT2 = WP2 + WS2 + WPAY;
TRST2 = AMAX2*(WPAY + WS2);
TB2   = XISP2*WP2/TRST2;

TOP1  = WTOT2*(exp(DELV1/(XISP1*32.2)) - 1.);
BOT1  = 1/XMF1 - ((1.-XMF1)/XMF1)*exp(DELV1/(XISP1*32.2));
if BOT1 <= 0
    error('mgs:zarchan:gravityTurn:stage1', ...
        'stage-1 denominator is %.4g <= 0: see the stage-2 message.', BOT1);
end
WP1   = TOP1/BOT1;
WS1   = WP1*(1-XMF1)/XMF1;
WTOT  = WP1 + WS1 + WTOT2;
TRST1 = AMAX1*(WTOT2 + WS1);
TB1   = XISP1*WP1/TRST1;

D = struct('WPAY',WPAY, 'WP1',WP1, 'WS1',WS1, 'WP2',WP2, 'WS2',WS2, ...
           'WTOT2',WTOT2, 'WTOT',WTOT, 'TRST1',TRST1, 'TRST2',TRST2, ...
           'TB1',TB1, 'TB2',TB2, 'DELV1',DELV1, 'DELV2',DELV2);
end


% =============================================================================
function [WGT, TRST] = stageState(T, D)
%STAGESTATE  The listing's thrust-weight schedule.  THE RESOLVED COMPARISONS
%   ("if T < TB1", "elseif T < (TB1+TB2)") live here and nowhere else, so
%   selfcheck 2 can exercise them at five instants without touching the loop.
if T < D.TB1
    WGT  = -D.WP1*T/D.TB1 + D.WTOT;
    TRST = D.TRST1;
elseif T < (D.TB1 + D.TB2)
    WGT  = -D.WP2*T/D.TB2 + D.WTOT2 + D.WP2*D.TB1/D.TB2;
    TRST = D.TRST2;
else
    WGT  = D.WPAY;
    TRST = 0.;
end
end


% =============================================================================
function TR = flyGravityTurn(D, GAMDEG, LEFT, ALTNM, ANGDEG, H, samp, tmax)
%FLYGRAVITYTURN  Listing 12.2's trajectory, zarchan.txt:3762-3773.
A   = 2.0926e7;        % Earth radius [ft], the listing's own value
GM  = 1.4077e16;       % [ft^3/s^2].  GM/A^2 = 32.147, vs the listing's 32.2
NMI = 6076;            % [ft/n.mi.], the listing's own conversion

T   = 0.;
S   = 0.;
ALT = ALTNM*NMI;
ANG = ANGDEG/57.3;                        % 57.3, not 180/pi.  Verbatim.
if LEFT == 1
    VRX = cos(1.5708 - GAMDEG/57.3 + ANG);
    VRY = sin(1.5708 - GAMDEG/57.3 + ANG);
else
    VRX = cos(-1.5708 + GAMDEG/57.3 + ANG);
    VRY = sin(-1.5708 + GAMDEG/57.3 + ANG);
end
X = (A + ALT)*cos(ANG);
Y = (A + ALT)*sin(ANG);
ALT = sqrt(X^2 + Y^2) - A;
XFIRST = X;  YFIRST = Y;
X1 = VRX;  Y1 = VRY;      % |V(0)| = 1 ft/s, NOT zero -- VEL is a divisor

% Derivatives initialised before the loop so the FLAG==1 branch can never read
% a value left over from something else.
XD = 0.;  YD = 0.;  X1D = 0.;  Y1D = 0.;

TBTOT = D.TB1 + D.TB2;
nmax  = ceil(tmax/samp) + 8;
rec = struct('t',zeros(nmax,1), 'dist_nm',zeros(nmax,1), 'alt_nm',zeros(nmax,1), ...
             'vel_fps',zeros(nmax,1), 'gam_deg',zeros(nmax,1), ...
             'wgt_lb',zeros(nmax,1), 'at_g',zeros(nmax,1), 'energy',zeros(nmax,1));
n = 0;
maxPar = 0;                % worst |sin(angle between thrust and velocity)|
gotBO  = false;
BO = struct('t',NaN, 'alt_nm',NaN, 'dist_nm',NaN, 'vel_fps',NaN, 'gam_deg',NaN);

while ~(ALT < 0 && T > 10) && T < tmax
    XOLD = X;  YOLD = Y;  X1OLD = X1;  Y1OLD = Y1;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  T advances INSIDE it, exactly once:
    % that is what puts the second derivative evaluation at t+H.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            X  = X  + H*XD;
            Y  = Y  + H*YD;
            X1 = X1 + H*X1D;
            Y1 = Y1 + H*Y1D;
            T  = T  + H;
        end
        [WGT, TRST] = stageState(T, D);
        AT     = 32.2*TRST/WGT;
        VEL    = sqrt(X1^2 + Y1^2);
        AXT    = AT*X1/VEL;               % <<< THE GRAVITY TURN: thrust is
        AYT    = AT*Y1/VEL;               %     along VELOCITY, not position
        TEMBOT = (X^2 + Y^2)^1.5;
        X1D = -GM*X/TEMBOT + AXT;
        Y1D = -GM*Y/TEMBOT + AYT;
        XD  = X1;
        YD  = Y1;
        FLAG = 1;
        if AT > 0
            maxPar = max(maxPar, abs(AXT*Y1 - AYT*X1)/(AT*VEL));
        end
    end
    % The listing writes "FLAG = 0" here too; the outer loop re-initialises it
    % next pass, so it cannot be read.  Noted so nobody re-adds it.

    X  = (XOLD  + X )/2 + .5*H*XD;
    Y  = (YOLD  + Y )/2 + .5*H*YD;
    X1 = (X1OLD + X1)/2 + .5*H*X1D;
    Y1 = (Y1OLD + Y1)/2 + .5*H*Y1D;
    ALT = sqrt(X^2 + Y^2) - A;

    if ~gotBO && T >= TBTOT
        gotBO = true;
        BO.t       = T;
        BO.alt_nm  = ALT/NMI;
        BO.dist_nm = downrangeNm(X, Y, XFIRST, YFIRST, A, NMI);
        BO.vel_fps = sqrt(X1^2 + Y1^2);
        BO.gam_deg = fpaDeg(X, Y, X1, Y1);
    end

    S = S + H;
    if S >= samp - 0.5*H
        S = 0.;
        [WGTr, TRSTr] = stageState(T, D);
        n = n + 1;
        rec.t(n)       = T;
        rec.dist_nm(n) = downrangeNm(X, Y, XFIRST, YFIRST, A, NMI);
        rec.alt_nm(n)  = ALT/NMI;
        rec.vel_fps(n) = sqrt(X1^2 + Y1^2);
        rec.gam_deg(n) = fpaDeg(X, Y, X1, Y1);
        rec.wgt_lb(n)  = WGTr;
        rec.at_g(n)    = TRSTr/WGTr;
        rec.energy(n)  = 0.5*(X1^2 + Y1^2) - GM/sqrt(X^2 + Y^2);
    end
end

f = fieldnames(rec);
for i = 1:numel(f)
    rec.(f{i}) = rec.(f{i})(1:n);
end

TR = struct();
TR.rec         = rec;
TR.burnout     = BO;
TR.t_end       = T;
% The FINAL integrated state, not the last recorded sample (which can be up
% to 'samp' seconds earlier -- about 17 n.mi. at impact speed).
TR.range_nm    = downrangeNm(X, Y, XFIRST, YFIRST, A, NMI);
TR.alt_end_nm  = ALT/NMI;
TR.hit_tmax    = T >= tmax;
TR.maxPar      = maxPar;
TR.gam0_deg    = fpaDeg(XFIRST, YFIRST, VRX, VRY);
TR.tb_total    = TBTOT;
end


% =============================================================================
function d = downrangeNm(X, Y, XFIRST, YFIRST, A, NMI)
%DOWNRANGENM  The listing's great-circle downrange.  CBETA is CLAMPED: at
%   t = 0 it is 1 + 2e-16 and acos() then returns a COMPLEX number that
%   propagates silently through every recorded row.
R  = sqrt(X^2 + Y^2);
RF = sqrt(XFIRST^2 + YFIRST^2);
CBETA = (X*XFIRST + Y*YFIRST)/(R*RF);
CBETA = min(max(CBETA, -1), 1);
d = A*acos(CBETA)/NMI;
end


% =============================================================================
function g = fpaDeg(X, Y, X1, Y1)
%FPADEG  Flight-path angle above the local horizontal [deg].  Positive
%   climbing, negative descending.  sin(gamma) = (r . v)/(|r||v|).
R = sqrt(X^2 + Y^2);
V = sqrt(X1^2 + Y1^2);
if R <= 0 || V <= 0
    g = NaN;  return
end
s = (X*X1 + Y*Y1)/(R*V);
g = asind(min(max(s, -1), 1));
end


% =============================================================================
function K = structuralChecks(D, TR, GAMDEG, LEFT, ANGDEG, WPAY, samp)
%STRUCTURALCHECKS  Self-consistency.  NOT book values and never evidence that
%   this port reproduces Zarchan -- the weaker claim that it agrees with
%   itself, which is what catches wiring faults.  .names and .oks are what
%   benchSuite reads; stored under any other name they put ZERO rows on the
%   board.
K = struct();
names = {};  oks = [];  detail = {};

% 1  Weight bookkeeping closes.
tot = D.WP1 + D.WS1 + D.WP2 + D.WS2 + WPAY;
K.wclose = abs(tot/D.WTOT - 1);
names{end+1} = 'Wtot == Wp1+Ws1+Wp2+Ws2+Wpay';
oks(end+1)   = K.wclose <= 1e-12;
detail{end+1} = sprintf('closure error %.3e relative (%.4f lb)', K.wclose, tot - D.WTOT);

% 2  DELETE EXPERIMENT on the two resolved comparisons.  With "T < TB1" the
%    schedule must give WTOT at liftoff, drop exactly WS1 at TB1 and exactly
%    WS2 at burnout, and settle on WPAY.  With "T > TB1" instead, t = 0 falls
%    through to the else branch: a 100 lb booster with no thrust.
% ⚠️⚠️ e USED TO BE 1e-6, AND THAT ALONE MADE THIS ROW RED.  The diagnosis is
%    worth keeping because the wrong fix was one keystroke away.
%
%    The check reads the mass schedule at TB1 +/- e.  Between those two
%    instants an engine is still burning, so the measured drop is not WS1 but
%
%        WS1 + e*(mdot1 + mdot2),   mdot1 = WP1/TB1 = 107.80 lb/s
%                                   mdot2 = WP2/TB2 =  15.89 lb/s
%
%    At e = 1e-6 that is 613.0000653 + 1.2369e-4, i.e. 2.0177e-07 relative --
%    and the board measured 2.0178e-07.  The residual was ENTIRELY this
%    epsilon.
%
%    ⭐ How it was pinned down, and why the obvious move was wrong: the
%    tempting fix is to relax the 1e-9 tolerance, but 2e-7 does not look like
%    truncation, so it was swept first.  The residual is IDENTICAL at
%    H = 0.2 .. 0.00625 (ratio 1.00 at every halving) AND at
%    samp = 1.0 .. 0.03125.  Independent of both ⇒ NOT integration error and
%    NOT a sampling artefact ⇒ a fixed algebraic term, which is exactly what a
%    widened tolerance would have hidden.  Only then did the arithmetic above
%    identify it.
%
%    ⇒ Shrink the probe instead.  TB1 ~ 32.22 s and eps(32) ~ 3.6e-15, so
%    1e-10 is ~28000 eps -- the two sides of the comparison stay distinct,
%    the delete experiment below is untouched (w0 does not involve e), and the
%    residual lands near 2e-11 against an UNCHANGED 1e-9 tolerance.
e  = 1e-10;
w0 = stageState(0, D);
[w1m, t1m] = stageState(D.TB1 - e, D);
w1p = stageState(D.TB1 + e, D);
w2m = stageState(D.TB1 + D.TB2 - e, D);
[w2p, t2p] = stageState(D.TB1 + D.TB2 + e, D);
resid = max([abs(w0/D.WTOT - 1), ...
             abs((w1m - w1p)/D.WS1 - 1), ...
             abs((w2m - w2p)/D.WS2 - 1), ...
             abs(w2p/WPAY - 1)]);
K.stage_resid = resid;
K.thrust_ok   = t1m == D.TRST1 && t2p == 0;
names{end+1} = 'staging drops exactly Ws1 and Ws2 (resolves "if T < TB1")';
oks(end+1)   = resid <= 1e-9 && K.thrust_ok;
detail{end+1} = sprintf(['worst %.3e; FLIPPED to "T > TB1" this gives ' ...
    'WGT(0) = %.4g lb with zero thrust instead of %.4g lb'], resid, WPAY, D.WTOT);

% 3  The gravity turn IS thrust along velocity.  Wire AXT/AYT to position
%    instead (X, Y rather than X1, Y1) and the program still runs smoothly.
K.par_resid = TR.maxPar;
names{end+1} = 'thrust vector parallel to velocity (gravity turn)';
oks(end+1)   = TR.maxPar <= 1e-12;
detail{end+1} = sprintf('worst |sin(angle)| = %.3e over the powered arc', TR.maxPar);

% 4  Specific energy is conserved through free flight (TRST = 0, pure Kepler).
%    Normalised by GM/A, not by |E| itself, so a near-escape case cannot
%    divide by ~0 and manufacture a failure.
GMoA = 1.4077e16/2.0926e7;
coast = TR.rec.t > TR.tb_total + samp;
if sum(coast) >= 2
    Ec = TR.rec.energy(coast);
    K.energy_drift = max(abs(Ec - Ec(1)))/GMoA;
    ok4 = K.energy_drift <= 1e-6;
    d4  = sprintf('max |dE|/(GM/A) = %.3e over %d free-flight samples', ...
                  K.energy_drift, sum(coast));
else
    % NOT EVALUATED, and it says so.  A run that crashes during boost has no
    % free-flight arc; failing it would be a false red about the integrator.
    K.energy_drift = NaN;
    ok4 = true;
    d4  = sprintf('NOT EVALUATED: only %d free-flight sample(s) (t_end %.1f s, burnout %.1f s)', ...
                  sum(coast), TR.t_end, TR.tb_total);
end
names{end+1} = 'specific energy conserved after burnout';
oks(end+1)   = ok4;
detail{end+1} = d4;

% 5  The LEFT branch and the book's 57.3 / 1.5708 really do deliver GAMDEG.
if LEFT == 1
    K.gam0_err = abs(TR.gam0_deg - GAMDEG);
else
    K.gam0_err = abs(abs(TR.gam0_deg) - abs(GAMDEG));
end
names{end+1} = 'initial flight-path angle == GAMDEG';
oks(end+1)   = K.gam0_err <= 0.01;
detail{end+1} = sprintf(['%.5f deg vs %.5f asked (ANGDEG %g, LEFT %g); the ' ...
    '%.2e deg gap is the listing''s own 57.3 and 1.5708'], ...
    TR.gam0_deg, GAMDEG, ANGDEG, LEFT, K.gam0_err);

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
function [B, X] = benchmarks(p, TRmain)
%BENCHMARKS  Computed vs published.  Every book value was grepped out of
%   zarchan.txt and its line is in the .source field; nothing off a figure.
%
%   SIX ROWS, AND TABLE 12.1'S NINE ARE DELIBERATELY NOT AMONG THEM.
%   mgs.zarchan.thrustWeight (Listing 12.1) already scores Wtot, Wp1, Ws1, Wp2,
%   Ws2, T1, tB1, T2, tB2 as nine separate rows, and Listing 12.2 EMBEDS that
%   same line of code verbatim.  Re-scoring them here would put nine more
%   greens on the board for arithmetic already adjudicated once -- a board that
%   looks twice as verified as it is.  Row 1 collapses the check into a single
%   "the embedded copy is uncorrupted" row; the rest of the budget goes to what
%   only Listing 12.2 can answer, which is the trajectory.
B = emptyBench();
X = struct('T85',[], 'T75',[], 'T65',[]);

% The published numbers belong to ONE design.  The rows below rebuild it
% internally rather than reading the caller's, so changing AMAX cannot quietly
% re-point a row at a different booster.
bookDesign = abs(p.XISP1-250) < 1e-9 && abs(p.XISP2-250) < 1e-9 && ...
             abs(p.XMF1-0.85) < 1e-12 && abs(p.XMF2-0.85) < 1e-12 && ...
             abs(p.WPAY-100)  < 1e-9 && abs(p.DELV-20000) < 1e-9;
geomOK = abs(p.ALTNM) < 1e-12 && abs(p.ANGDEG-90) < 1e-9 && p.LEFT == 1;

whyDesign = sprintf(['off the chapter''s nominal design -- Table 12.1 is ' ...
    'Isp 250/250 s, mf 0.85/0.85, Wpay 100 lb, DV 20000 ft/s; this run used ' ...
    '%g/%g, %g/%g, %g, %g'], p.XISP1, p.XISP2, p.XMF1, p.XMF2, p.WPAY, p.DELV);
whyGeom = sprintf(['the published ranges are for a surface launch straight ' ...
    'up, counterclockwise (ALTNM 0, ANGDEG 90, LEFT 1); this run used ' ...
    '%g, %g, %g'], p.ALTNM, p.ANGDEG, p.LEFT);

TBL = 'Table 12.1, p.263 (zarchan.txt:3691-3699)';
RNG = 'p.271 text (zarchan.txt:3786)';

if bookDesign
    D10 = boosterDesign(p.XISP1, p.XISP2, p.XMF1, p.XMF2, p.WPAY, p.DELV, 10, 10);
    D20 = boosterDesign(p.XISP1, p.XISP2, p.XMF1, p.XMF2, p.WPAY, p.DELV, 20, 20);

    % ---- 1.  The embedded Listing 12.1 line, as ONE row.  Bands are each
    %      printed figure's own rounding band: 0.1% at four significant
    %      figures, 0.2% for 32.2 s, 0.5% for 106 s.
    lbl  = {'Wtot [lb]','T1 [lb]','T2 [lb]','tB1 [s]','tB2 [s]'};
    bkv  = [6169, 26950, 3973, 32.2, 106];
    ourv = [D10.WTOT, D10.TRST1, D10.TRST2, D10.TB1, D10.TB2];
    tolv = [0.001, 0.001, 0.001, 0.002, 0.005];
    rel  = abs(ourv./bkv - 1);
    [worst, iw] = max(rel./tolv);
    B = addBound(B, 'embedded Listing 12.1 line reproduces Table 12.1', ...
        'Wtot 6169 lb, T1 26,950, T2 3973, tB1 32.2 s, tB2 106 s', ...
        sprintf('worst %.3f of band at %s (%.6g vs %g, %+.4f%%)', worst, ...
            lbl{iw}, ourv(iw), bkv(iw), 100*(ourv(iw)/bkv(iw)-1)), ...
        all(rel <= tolv), TBL, 'lb, s', ...
        ['ONE row and not nine: thrustWeight (Listing 12.1) already scores ' ...
         'every Table 12.1 entry, and this listing embeds the same line.  ' ...
         'This catches a corrupted embedded copy without double-counting.']);

    % ---- 2.  The 20-g variant, quoted in CHAPTER 14 -- an independent
    %      statement of "half the time of the nominal design" (:3785), and the
    %      one Table-12.1-adjacent number no other port on the board scores.
    B = addVal(B, 'total burn time at amax = 20 g', 69.1, D20.TB1 + D20.TB2, ...
        'text, 3 sf', 0.001, 'p.302 text (zarchan.txt:4322; cf. :3785)', 's', ...
        ['weights do not depend on amax, so doubling thrust exactly halves ' ...
         'both burn times: 138.23/2 = 69.12.  This is the booster Chapter 14 ' ...
         'reuses as a boosting target.']);
else
    B = addSkip(B, 'embedded Listing 12.1 line reproduces Table 12.1', ...
        'Table 12.1', whyDesign, TBL, 'lb, s');
    B = addSkip(B, 'total burn time at amax = 20 g', '69.1', whyDesign, ...
        'p.302 text (zarchan.txt:4322)', 's');
end

% ---- 3-5.  The rows that actually fly -- the content unique to Listing 12.2.
if bookDesign && geomOK
    if abs(p.GAMDEG-85) < 1e-9 && abs(p.AMAX1-10) < 1e-12 && abs(p.AMAX2-10) < 1e-12
        T85 = TRmain;                       % reuse: same case, no second run
    else
        T85 = flyGravityTurn(D10, 85, 1, 0, 90, p.H, p.samp, p.tmax);
    end
    % 75 deg, not 79: the book states a THRESHOLD at 80 deg, and testing a
    % threshold on its knife edge tests the author's rounding, not the port.
    T75 = flyGravityTurn(D10, 75, 1, 0, 90, p.H, p.samp, p.tmax);
    T65 = flyGravityTurn(D20, 65, 1, 0, 90, p.H, p.samp, p.tmax);
    X.T85 = T85;  X.T75 = T75;  X.T65 = T65;

    guardWhy = sprintf(['the %g s time guard fired, so this trajectory is ' ...
        'TRUNCATED, not flown to impact.  Scoring a truncated range would be ' ...
        'scoring the guard.  Raise ''tmax''.'], p.tmax);

    if T85.hit_tmax
        B = addSkip(B, 'max downrange, nominal design, gamma0 = 85 deg', ...
            '2300', guardWhy, RNG, 'n.mi.');
    else
        B = addVal(B, 'max downrange, nominal design, gamma0 = 85 deg', ...
            2300, T85.range_nm, 'prose "about", 2 sf', 0.10, RNG, 'n.mi.', ...
            sprintf(['impact at t = %.1f s, alt %.4f n.mi.  The 10%% band is ' ...
                'the author''s own reading of Fig. 12.11.  DO NOT WIDEN IT.'], ...
                T85.t_end, T85.alt_end_nm));
    end

    if T65.hit_tmax
        B = addSkip(B, 'max downrange, 20-g design, gamma0 = 65 deg', ...
            '2600', guardWhy, RNG, 'n.mi.');
    else
        B = addVal(B, 'max downrange, 20-g design, gamma0 = 65 deg', ...
            2600, T65.range_nm, 'prose "about", 2 sf', 0.10, RNG, 'n.mi.', ...
            sprintf('impact at t = %.1f s; gamma0 65 deg on the 20-g booster', ...
                T65.t_end));
    end

    % ---- 5.  THE SHAPE CLAIM.  zarchan.txt:3752: "If the flight-path angle
    %      is less than 80 deg, the booster will immediately crash into the
    %      Earth."  Prose about a figure is greppable; the figure is not.
    okS = ~T85.hit_tmax && T75.range_nm < 0.5*T85.range_nm;
    B = addShape(B, 'gamma0 below 80 deg crashes (nominal design)', ...
        'range(75 deg) << range(85 deg)', ...
        sprintf('%.1f vs %.1f n.mi. (%.1f%%), impact t = %.1f s vs %.1f s', ...
            T75.range_nm, T85.range_nm, ...
            100*T75.range_nm/max(T85.range_nm, eps), T75.t_end, T85.t_end), ...
        okS, 'p.267 text (zarchan.txt:3752)', ...
        ['open loop: nothing steers, so gamma is a boundary condition and ' ...
         'the failure is smooth, silent and plausible all the way down']);
else
    why = whyDesign;
    if bookDesign, why = whyGeom; end
    B = addSkip(B, 'max downrange, nominal design, gamma0 = 85 deg', '2300', ...
        why, RNG, 'n.mi.');
    B = addSkip(B, 'max downrange, 20-g design, gamma0 = 65 deg', '2600', ...
        why, RNG, 'n.mi.');
    B = addSkip(B, 'gamma0 below 80 deg crashes (nominal design)', ...
        'range(75) << range(85)', why, 'p.267 text (zarchan.txt:3752)', '-');
end

% ---- 6.  INFO.  The burnout state -- reported, never adjudicated.  The
%      20 kft/s goal at zarchan.txt:3785 belongs to Listing 12.1's DRAG-FREE
%      ONE-dimensional integral (scored in thrustWeight as 19999.75 ft/s);
%      this two-dimensional turn pays a gravity loss, and the book publishes
%      no number for what that loss is.
B0 = TRmain.burnout;
B = addInfo(B, 'burnout conditions of the gravity turn', ...
    '20 kft/s is Listing 12.1''s 1-D value, not this one''s', ...
    sprintf('t %.2f s, V %.1f ft/s, gamma %.2f deg, alt %.2f nmi, down %.2f nmi', ...
        B0.t, B0.vel_fps, B0.gam_deg, B0.alt_nm, B0.dist_nm), ...
    'p.271 text (zarchan.txt:3785)', ...
    ['the gravity loss between the 1-D goal and this 2-D burnout speed is ' ...
     'real physics, but the book publishes no number for it, so there is ' ...
     'nothing here to adjudicate against']);
end


function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER must match the adders exactly.  Schema is
%   the one benchSuite's benchRows() consumes; a near miss (book_s for
%   book_str) ERRORS there rather than being accommodated.
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
p = R.p;  D = R.design;
L{end+1} = 'Zarchan Listing 12.2 -- GRAVITY TURN SIMULATION (two-stage booster)';
L{end+1} = '*** UNITS ARE FEET, POUNDS, SECONDS.  g = 32.2, 1 n.mi. = 6076 ft. ***';
L{end+1} = sprintf(['   Isp %g/%g s  mf %g/%g  Wpay %g lb  DV %g ft/s  amax %g/%g g\n' ...
    '   gamma0 %g deg  LEFT %g  alt0 %g nmi  theta0 %g deg  H %g s  samp %g s'], ...
    p.XISP1, p.XISP2, p.XMF1, p.XMF2, p.WPAY, p.DELV, p.AMAX1, p.AMAX2, ...
    p.GAMDEG, p.LEFT, p.ALTNM, p.ANGDEG, p.H, p.samp);
L{end+1} = '';
L{end+1} = 'BOOSTER DESIGN  (Listing 12.2 line 1 = Listing 12.1)';
L{end+1} = sprintf('   Wp1 %8.1f lb  Ws1 %7.1f lb  T1 %9.1f lb  tB1 %7.3f s', ...
    D.WP1, D.WS1, D.TRST1, D.TB1);
L{end+1} = sprintf('   Wp2 %8.1f lb  Ws2 %7.1f lb  T2 %9.1f lb  tB2 %7.3f s', ...
    D.WP2, D.WS2, D.TRST2, D.TB2);
L{end+1} = sprintf('   Wtot %7.1f lb  burnout %.3f s  liftoff accel %.3f g', ...
    D.WTOT, D.TB1 + D.TB2, D.TRST1/D.WTOT);
L{end+1} = '';

L{end+1} = 'TRAJECTORY';
L{end+1} = sprintf('%8s %10s %10s %11s %9s %9s %8s', ...
    't [s]', 'down[nmi]', 'alt[nmi]', 'V[ft/s]', 'gam[deg]', 'W[lb]', 'aT[g]');
tt = R.traj;  nrec = numel(tt.t);
if nrec > 0
    for r = unique([1:max(1,ceil(nrec/12)):nrec, nrec])
        L{end+1} = sprintf('%8.1f %10.2f %10.4f %11.1f %9.3f %9.1f %8.3f', ...
            tt.t(r), tt.dist_nm(r), tt.alt_nm(r), tt.vel_fps(r), ...
            tt.gam_deg(r), tt.wgt_lb(r), tt.at_g(r)); %#ok<AGROW>
    end
end
B0 = R.burnout;
L{end+1} = sprintf('   impact:  t %.2f s, downrange %.2f nmi, alt %.5f nmi%s', ...
    R.tof_s, R.range_nm, R.alt_end_nm, ...
    tern(R.hit_tmax, '   <<< TIME GUARD FIRED, TRUNCATED', ''));
L{end+1} = sprintf(['   burnout: t %.2f s, V %.1f ft/s (%.2f kft/s), gamma ' ...
    '%.2f deg, alt %.2f nmi, down %.2f nmi'], B0.t, B0.vel_fps, ...
    B0.vel_fps/1000, B0.gam_deg, B0.alt_nm, B0.dist_nm);
L{end+1} = ['   (burnout speed is NOT scored: the 20 kft/s goal at ' ...
    'zarchan.txt:3785 is Listing 12.1''s'];
L{end+1} = '    one-dimensional integral, which pays no gravity loss.)';
L{end+1} = '';

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
    L{end+1} = 'BENCHMARKS vs ZARCHAN CHAPTER 12';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        if b.skipped
            mark = '--';  dev = '';  tail = ['SKIP: ' b.why];
        else
            mark = vstr(b.ok);  tail = b.band;
            if isnan(b.pct), dev = ''; else, dev = sprintf('%+.3f%%', b.pct); end
        end
        L{end+1} = sprintf('%-4s %-46s %11s %11s %9s  %s', mark, b.what, ...
            b.book_str, b.ours_str, dev, tail); %#ok<AGROW>
        L{end+1} = sprintf('     %s', b.source); %#ok<AGROW>
    end
    L{end+1} = sprintf('   %d of %d rows evaluated; R.pass = %d', ...
        sum(ev), numel(R.bench), R.pass);
end
s = strjoin(L, newline);
end


function v = vstr(ok)
if ok, v = 'ok'; else, v = 'FAIL'; end
end

function s = tern(c, a, b)
if c, s = a; else, s = b; end
end


% =============================================================================
function drawTrajectory(R)
figure;
plot(R.traj.dist_nm, R.traj.alt_nm); grid on
xlabel('Downrange (Nmi)'); ylabel('Altitude (Nmi)');
title(sprintf('Zarchan 12.2 gravity turn, gamma_0 = %g deg', R.p.GAMDEG));
end
