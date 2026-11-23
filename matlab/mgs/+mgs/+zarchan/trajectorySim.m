function R = trajectorySim(opts)
%MGS.ZARCHAN.TRAJECTORYSIM  Zarchan Listing 10.1 -- ballistic flyout of an
%   impulsively launched missile through gravity and an exponential atmosphere.
%
%   R = mgs.zarchan.trajectorySim()
%   R = mgs.zarchan.trajectorySim('BETA', 500)          heavier drag
%   R = mgs.zarchan.trajectorySim('BETA', Inf)          the book's no-drag case
%   R = mgs.zarchan.trajectorySim('ALT0', 50000, 'assert', false)
%
%   Line-by-line port of Listing 10.1, integrated by the book's own two-pass
%   second-order Runge-Kutta (Heun), unchanged.
%
%   UNITS ARE THE BOOK'S: ft, ft/s, ft/s^2, lb/ft^2, g = 32.2 ft/s^2.  Ranges
%   print in KFT because the listing divides by 1000 itself (RM1K).  Nothing
%   here is converted to SI; convert at the call site and say so there.
%
%   WHAT IT IS AND IS NOT FOR.  Ch.10 is where the book leaves guided flight
%   for the ballistic (and from Ch.11 orbital) half.  THIS PROJECT FLIES
%   NEITHER.  No seeker, no autopilot, no guidance law, no target, no lift: a
%   point mass gets an impulsive 3000 ft/s at 45 deg and is then acted on by
%   constant gravity and zero-lift drag until it reaches the ground.  Do not
%   read a range, a time of flight or a terminal speed out of here and apply it
%   to a sortie.  It is in the package for two narrow reasons: it is the SAME
%   drag model Listing 16.1 uses for ballistic targets, DRAG = Q*g/beta with
%   Q = 0.5*rho*V^2 and beta = W/(Cd0*Sref), so pinning that line and the
%   two-branch atmosphere here is a prerequisite for Ch.16; and it is the
%   book's cleanest statement that DRAG AUTHORITY IS PHYSICS, NOT A TUNABLE --
%   a factor of 4 in beta moves the flyout range by 9x (30.8 -> 279.5 kft).
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  matlab/zarchan/_listings/listing_10_01.m, whose header
%   claims NO unresolved comparisons.  That claim is WRONG, and the one it
%   missed is the loop that ends the flight:
%
%     1  "while ~(T.0. & RM2 <= 0.)" -> "while ~(T > 0 && RM2 <= 0)".  The
%        extractor resolved the second comparison and left "T.0.", so the line
%        does not parse; "." is this book's OCR of ">".  DIRECTION IS FORCED:
%        RM2 starts AT 0 for a sea-level launch, so the T guard is the only
%        thing that lets the loop start, and the terminating event is impact
%        after apogee.  "<" would stop the run on its first pass.
%     2  "if RM2,30000." -> "if RM2 < 30000."      (already repaired)
%     3  "if S . = .99999" -> "if S >= .99999" (already repaired).  S is zeroed
%        on every record, so ">=" is what makes it a 1 s sampler ('samp').
%     4  The minus signs in "exp(-RM2/30000)" and "AM2 = -32.2 - ..." survived
%        in BOTH source files.  Noted only because this book's minus is often
%        mis-mapped to a digit 2; physics forces both signs anyway.
%     5  "save datfil.txt output /ascii" is the -ascii typo Listings 4.5/4.6
%        carry too.  Nothing is written to disk by this port.
%     6  The trailing "FLAG=0" is dead -- the outer loop re-initialises FLAG.
%        Dropped, recorded so nobody re-adds it thinking a state machine was
%        lost.
%     7  GAMDEG/57.3 kept VERBATIM, not tidied to pi/180: 0.0074% in the angle,
%        0.2 ft in 279,503.  Tidying would silently shift every row below.
%     8  BETA = Inf is the book's no-drag case and needs no special case --
%        the listing's own "DRAG = Q*32.2/BETA" evaluates to exactly 0.
%
%   ONE LINE NUMBER, VALID IN BOTH FILES.  zarchan.txt and zarchan_decoded.txt
%   are line-aligned (15120 lines each), so every .source below cites a single
%   "zarchan.txt:NNNN" and it resolves in either.  ONE CAVEAT, observed
%   directly: zarchan_decoded.txt was REGENERATED on 2026-08-24 partway through
%   this port being written.  Before that regeneration the decoded copy ran
%   15307 lines and Ch.10 sat +36 lines lower in it (LISTING 10.1 at 3168, not
%   3132).  Every citation below was re-grepped against the CURRENT file after
%   the change.  Any port citing a decoded-file line in Ch.10 and later may be
%   carrying the old numbering -- worth a re-grep, not an assumption.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Ch.10 publishes almost nothing scoreable -- it is a chapter of
%   figures, and extraction turned every figure into an image link.  What it
%   does publish is two sentences of flyout ranges and one of drag
%   decelerations, all grepped from the DECODED file, nothing read off a plot:
%     1-4  Fig. 10.12, 45 deg from SEA LEVEL (zarchan.txt:3120): about 300
%          kft no drag, about 100 at beta = 2000, about 55 at 1000, 30 at 500.
%     5-7  Fig. 10.13, same launch from 50 KFT (zarchan.txt:3131): about
%          350 kft no drag, about 275 at beta = 2000, 175 at beta = 500.
%       8  Fig. 10.4 (zarchan.txt:3052): at 3000 ft/s and beta = 500, about
%          22 g at sea level.  Model evaluation only, no integration.
%       9  Fig. 10.11 (zarchan.txt:3109): the 45-deg launch gives maximum
%          range.  SHAPE -- argmax over a launch-angle sweep, no drag.
%      10  zarchan.txt:3121: drag matters much less from 50 kft.  SHAPE,
%          because the claim is COMPARATIVE: the beta=2000/no-drag range ratio
%          must be far higher at 50 kft than at sea level.
%
%   BANDS: 10% on ranges, 15% on the deceleration, and NOT wider.  Six of the
%   seven ranges are prefixed "about" and read off axes ticked in tens of kft,
%   so the BOOK side carries one part in twenty; the 22 g comes off a LOG axis.
%   Set by provenance, not by what we compute -- and still discriminating:
%   swapping the atmosphere's two scale heights moves the upper-branch values
%   by tens of percent, deleting drag moves the sea-level rows by up to 9x.
%
%   THE SCORED ROWS IGNORE THE CALLER'S OPTIONS.  The seven cases always run at
%   the book's VM = 3000 ft/s and GAMDEG = 45 deg; opts.BETA/ALT0 choose the
%   TRAJECTORY returned in R.t/R.alt_kft, not the benchmark.  So this port can
%   never go unscored, and H is the only option touching a book row (0.007%
%   between H = 0.01 and 0.02).
%
%   NOT SCORED (INFO): Figs. 10.5-10.8, "about 4.5 s" to halve V at beta = 500.
%   Those belong to the CONSTANT-ALTITUDE, gravity-free closed form at
%   zarchan.txt:3042-3046 (V1 = V0*T0/(T0+dt), V halves at dt = T0).  Listing
%   10.1 always carries gravity and flies an arc: it cannot produce that curve.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED IN MATLAB. ***  Never
%   executed here -- MATLAB is the caller's.  These come from an independent
%   line-for-line reimplementation of the same listing in another language, so
%   they are evidence about the ARITHMETIC, not about this .m file.
%   Flyout range [kft], 45 deg, 3000 ft/s, H = 0.01 s:
%      sea level: no drag 279.52 (book 300, -6.8%)  b2000  98.04 (-2.0%)
%                 b1000    56.51 (book  55, +2.7%)  b 500  30.79 (+2.6%)
%      50 kft   : no drag 322.82 (book 350, -7.8%)  b2000 275.51 (+0.2%)
%                 b 500   177.28 (book 175, +1.3%)
%      drag deceleration, 3000 ft/s, beta = 500: 21.40 g at sea level (book 22)
%      structural: no-drag range vs vacuum closed form +0.0065% / +0.0058%
%
%   The two NO-DRAG rows are the loose ones, about -7% each, and they are loose
%   on the BOOK's side: both are exact vacuum ballistics (R = vx*(vy +
%   sqrt(vy^2 + 2*g*h0))/g = 279.503 and 322.801 kft) which the integrator
%   reproduces to 0.007%, and structural row 1 says so separately.  If one goes
%   red, that row says at once whether our side moved.  DO NOT WIDEN THE BAND.
%
%   OUTPUTS
%     R.t R.downrange_kft R.alt_kft  the listing's ArrayT/ArrayRM1K/ArrayRM2K
%     R.range_kft R.tof_s            flyout range and time of flight
%     R.cases                        the seven book cases (beta alt0 book ours)
%     R.gamma_sweep                  launch-angle sweep, [] if 'benchmark' off
%     R.bench R.selfcheck R.pass R.verdict R.p R.summary R.units
%
%   See also MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 10.1 line 1, verbatim.
    opts.VM     (1,1) double {mustBeFinite, mustBePositive} = 3000.0  % ft/s
    opts.GAMDEG (1,1) double {mustBeFinite, mustBePositive} = 45.0    % deg
    % NO mustBeFinite: BETA = Inf IS the book's no-drag case (zarchan.txt:3120) and it
    % flows through the listing's own "Q*32.2/BETA" without a special case.
    opts.BETA   (1,1) double {mustBePositive} = 1000.0                % lb/ft^2
    % The listing launches from RM2 = 0; zarchan.txt:3121 repeats the case at 50 kft.
    opts.ALT0   (1,1) double {mustBeFinite, mustBeNonnegative} = 0.0  % ft
    opts.H      (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s
    opts.samp   (1,1) double {mustBeFinite, mustBePositive} = 1.0     % s
    % Runaway guard.  A flyout that has not reached the ground in TMAX seconds
    % is an ERROR, not a truncated curve: a truncated trajectory comes back a
    % perfectly smooth arc with a wrong range and no symptom.
    opts.TMAX   (1,1) double {mustBeFinite, mustBePositive} = 600.0   % s
    % Launch angles for the Fig. 10.11 shape row.  Coarse ON PURPOSE and run at
    % 'gamma_H': the row is an ORDERING, and this sweep dominates the runtime
    % of the file.  Range is insensitive to H here (0.03% from 0.05 to 0.01 s).
    opts.gamma_sweep (1,:) double {mustBeFinite, mustBePositive} = ...
        [5 15 25 35 40 45 50 55 65 75 85]
    opts.gamma_H (1,1) double {mustBeFinite, mustBePositive} = 0.05   % s
    % THE ONLY SWITCHES.  Not nargin: with an arguments block declaring only
    % name-value options MATLAB reports nargin = 0 however many were passed.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

if opts.GAMDEG >= 90
    error('mgs:zarchan:trajectorySim:gamma', ...
        ['GAMDEG = %g deg.  At or past vertical the listing''s ' ...
         'atan2(VM2,VM1) turns the drag direction through the launch, which ' ...
         'is outside anything Fig. 10.11 covers (it sweeps to 80 deg).'], ...
        opts.GAMDEG);
end
if opts.H >= opts.samp
    error('mgs:zarchan:trajectorySim:samp', ...
        ['H = %g s is not smaller than samp = %g s, so the returned curve ' ...
         'would be sampled at the step rate while claiming the coarser ' ...
         'interval in its own time vector.'], opts.H, opts.samp);
end

% ---- the caller's own trajectory ------------------------------------------
F = flyout(opts.VM, opts.GAMDEG, opts.BETA, opts.ALT0, opts.H, opts.samp, opts.TMAX);

% ---- the seven published cases.  ALWAYS at the book's VM and GAMDEG. -------
SRC0  = 'Fig. 10.12 text, p.224 (zarchan.txt:3120)';
SRC50 = 'Fig. 10.13 text, p.225 (zarchan.txt:3131)';
C = struct( ...
    'beta', {Inf,   2000,  1000,  500,   Inf,   2000,  500}, ...
    'alt0', {0,     0,     0,     0,     50000, 50000, 50000}, ...
    'book', {300,   100,   55,    30,    350,   275,   175}, ...
    'src',  {SRC0,  SRC0,  SRC0,  SRC0,  SRC50, SRC50, SRC50});
for k = 1:numel(C)
    Fk = flyout(3000.0, 45.0, C(k).beta, C(k).alt0, opts.H, opts.samp, opts.TMAX);
    C(k).ours     = Fk.range_kft;
    C(k).interp   = Fk.range_interp_kft;
    C(k).tof      = Fk.tof_s;
    C(k).alt_end  = Fk.alt_end_ft;
    C(k).vm2_end  = Fk.vm2_end;
    C(k).dr_min_d = Fk.dr_min_step_ft;
end

% ---- launch-angle sweep for the Fig. 10.11 shape row ----------------------
SW = [];
if opts.benchmark
    gs = opts.gamma_sweep(:).';
    gs = gs(gs > 0 & gs < 90);
    SW = struct('gamdeg', gs, 'range_kft', zeros(size(gs)), 'H', opts.gamma_H);
    for k = 1:numel(gs)
        Fg = flyout(3000.0, gs(k), Inf, 0.0, opts.gamma_H, opts.samp, opts.TMAX);
        SW.range_kft(k) = Fg.range_kft;
    end
end

R = struct();
R.t             = F.t;
R.downrange_kft = F.dr_kft;
R.alt_kft       = F.alt_kft;
R.range_kft     = F.range_kft;
R.tof_s         = F.tof_s;
R.cases         = C;
R.gamma_sweep   = SW;
R.units = 'ft, ft/s, ft/s^2, lb/ft^2; g = 32.2 ft/s^2; ranges reported in kft';
R.p = struct('VM',opts.VM, 'GAMDEG',opts.GAMDEG, 'BETA',opts.BETA, ...
             'ALT0',opts.ALT0, 'H',opts.H, 'samp',opts.samp, ...
             'TMAX',opts.TMAX, 'gamma_H',opts.gamma_H, ...
             'n_step',F.nstep, 'units',R.units);

R.selfcheck = structuralChecks(C, opts.H);

if opts.benchmark
    R.bench = benchmarks(C, SW);
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

% ---- adjudication.  The verdict is FORMED either way; 'assert' chooses only
%      the reaction to it.  benchSuite runs every port with assert off.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
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
        error('mgs:zarchan:trajectorySim:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:trajectorySim:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function F = flyout(VM, GAMDEG, BETA, ALT0, H, samp, TMAX)
%FLYOUT  Listing 10.1 itself (zarchan.txt:3133-3141).
%
%   Every constant arrives as an ARGUMENT: MATLAB local functions cannot see
%   the main function's workspace.

% The listing's line 1, verbatim except that RM2 starts at ALT0 (:3121).
S   = 0.0;
T   = 0.0;
VM1 = VM*cos(GAMDEG/57.3);      % 57.3, not 180/pi.  Kept.  See the header.
VM2 = VM*sin(GAMDEG/57.3);
RM1 = 0.0;
RM2 = ALT0;
AM1 = 0.0;  AM2 = 0.0;          % set on pass 1 before pass 2 ever reads them

nmax   = ceil(TMAX/samp) + 8;
t      = zeros(nmax,1);
dr     = zeros(nmax,1);
alt    = zeros(nmax,1);
n      = 0;
nstep  = 0;
drMin  = Inf;
RM1PRV = RM1;  RM2PRV = RM2;

% "while ~(T.0. & RM2 <= 0.)" -- stop once the missile is past launch AND at or
% below the ground.  See transcription note 1.
while ~(T > 0.0 && RM2 <= 0.0)
    RM1OLD = RM1;  RM2OLD = RM2;  VM1OLD = VM1;  VM2OLD = VM2;
    RM1PRV = RM1;  RM2PRV = RM2;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor.  T advances INSIDE this loop, exactly
    % once -- move that line out and the scheme quietly degrades to first
    % order while still running smoothly.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            RM1 = RM1 + H*VM1;  RM2 = RM2 + H*VM2;
            VM1 = VM1 + H*AM1;  VM2 = VM2 + H*AM2;
            T   = T + H;
        end
        RHO  = rhoOf(RM2);
        V    = sqrt(VM1^2 + VM2^2);
        Q    = 0.5*RHO*V*V;
        GAM  = atan2(VM2, VM1);
        DRAG = Q*32.2/BETA;             % BETA = Inf -> exactly 0
        AM1  = -DRAG*cos(GAM);
        AM2  = -32.2 - DRAG*sin(GAM);   % gravity DOWN, drag opposing V
        FLAG = 1;
    end
    % The listing's trailing "FLAG=0" is dead here; see transcription note 6.

    % Heun corrector, in the listing's own algebra.
    RM1 = 0.5*(RM1OLD + RM1 + H*VM1);
    RM2 = 0.5*(RM2OLD + RM2 + H*VM2);
    VM1 = 0.5*(VM1OLD + VM1 + H*AM1);
    VM2 = 0.5*(VM2OLD + VM2 + H*AM2);

    drMin = min(drMin, RM1 - RM1OLD);
    nstep = nstep + 1;

    S = S + H;
    if S >= samp - 0.5*H
        S = 0.0;
        n = n + 1;
        t(n)   = T;
        dr(n)  = RM1/1000.0;            % the listing's RM1K
        alt(n) = RM2/1000.0;            % the listing's RM2K
    end

    if T > TMAX
        error('mgs:zarchan:trajectorySim:runaway', ...
            ['the flyout has not reached the ground after %g s ' ...
             '(beta = %g lb/ft^2, gamma = %g deg, alt0 = %g ft).  Refused ' ...
             'rather than truncated: a truncated arc looks smooth and ' ...
             'reports a wrong range with no symptom.'], TMAX, BETA, GAMDEG, ALT0);
    end
end

if nstep < 2
    error('mgs:zarchan:trajectorySim:short', ...
        ['the flight ended after %d step(s) at H = %g s.  At gamma = %g deg ' ...
         'and alt0 = %g ft the missile never leaves the ground.'], ...
        nstep, H, GAMDEG, ALT0);
end

F = struct();
F.t       = t(1:n);
F.dr_kft  = dr(1:n);
F.alt_kft = alt(1:n);
% The impact state is the POST-loop state, which the 1 s sampler will usually
% never have recorded; reading the range off the recorded array instead would
% throw away up to samp seconds of flight -- 2 kft at these speeds.
F.range_kft      = RM1/1000.0;
F.tof_s          = T;
F.alt_end_ft     = RM2;
F.vm2_end        = VM2;
F.nstep          = nstep;
F.dr_min_step_ft = drMin;
% Linear interpolation to RM2 = 0 across the last step.  NOT the answer -- the
% listing has no such refinement -- but reported so the size of the impact-step
% quantisation is visible rather than assumed.
if RM2PRV > RM2
    fr = RM2PRV/(RM2PRV - RM2);
    F.range_interp_kft = (RM1PRV + fr*(RM1 - RM1PRV))/1000.0;
else
    F.range_interp_kft = NaN;
end
end


% =============================================================================
function rho = rhoOf(alt_ft)
%RHOOF  The listing's two-branch exponential atmosphere, verbatim
%   (zarchan.txt:3140), in slug/ft^3.  Below 30 kft the sea-level fit
%   0.002378*exp(-h/30000); at or above it 0.0034*exp(-h/22000).  The branches
%   are NOT identical at the switch -- they disagree by 0.6% -- and that
%   disagreement is the structural check that pins all four constants.
if alt_ft < 30000.0
    rho = 0.002378*exp(-alt_ft/30000);
else
    rho = 0.0034*exp(-alt_ft/22000);
end
end


% =============================================================================
function r = vacuumRangeKft(VM, GAMDEG, ALT0)
%VACUUMRANGEKFT  Exact drag-free flyout, using the listing's own 57.3 and 32.2:
%   R = vx*(vy + sqrt(vy^2 + 2*g*h0))/g.  Shares no code with the integrator,
%   which is the whole reason it is worth having.
vx = VM*cos(GAMDEG/57.3);
vy = VM*sin(GAMDEG/57.3);
r  = vx*(vy + sqrt(vy^2 + 2*32.2*ALT0))/32.2/1000.0;
end


% =============================================================================
function K = structuralChecks(C, H)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never evidence that
%   this port reproduces Zarchan -- only that it agrees with itself, which is
%   what catches wiring faults.
K = struct();
names = {};  oks = [];  detail = {};

% 1  The two no-drag cases against exact vacuum ballistics.  This is what makes
%    the loose -7% on those two BENCHMARK rows readable: it says our side is
%    exact, so the gap is the book's reading of its own figure.
nd    = find(~isfinite([C.beta]));
res   = 0;
worst = NaN;
for k = nd
    v = vacuumRangeKft(3000.0, 45.0, C(k).alt0);
    d = abs(C(k).ours/v - 1);
    if d > res
        res = d;  worst = C(k).alt0;
    end
end
K.vac_resid = res;
names{end+1}  = 'no-drag range == exact vacuum ballistics';
oks(end+1)    = ~isempty(nd) && res <= 1e-3;
detail{end+1} = sprintf('worst %.4f%% (at alt0 = %g ft; band 0.1%%)', 100*res, worst);

% 2  The atmosphere's two branches meet at 30 kft.  Pins all four constants at
%    once: a wrong scale height moves this by 13%, a wrong rho0 by 1.5%.
lo = 0.002378*exp(-30000/30000);
hi = 0.0034*exp(-30000/22000);
K.rho_jump    = abs(hi/lo - 1);
names{end+1}  = 'atmosphere continuous at the 30 kft branch switch';
oks(end+1)    = K.rho_jump <= 0.01;
detail{end+1} = sprintf('rho jumps %+.3f%% crossing 30 kft (band 1%%)', ...
    100*(hi/lo - 1));

% 3  Every case stopped within ONE step of the ground, and the quantisation
%    that leaves behind is negligible against the interpolated impact.  Without
%    it, a systematically one-step-long range reads as a modelling residual.
q = 0;  qi = 0;
for k = 1:numel(C)
    q = max(q, abs(C(k).alt_end)/max(abs(C(k).vm2_end)*H, realmin));
    % max(x, NaN) RETURNS x in MATLAB, so a non-finite interpolated impact
    % would be silently ignored here rather than failing the check.  Made
    % explicit: no interpolation means no evidence, which is not a pass.
    if isfinite(C(k).interp) && C(k).interp ~= 0
        qi = max(qi, abs(C(k).ours/C(k).interp - 1));
    else
        qi = Inf;
    end
end
K.impact_steps = q;
K.impact_quant = qi;
names{end+1}   = 'impact detected within one integration step of the ground';
oks(end+1)     = q <= 1.5 && qi <= 1e-3;
detail{end+1}  = sprintf('worst overshoot %.3f step, %.4f%% of range', q, 100*qi);

% 4  DELETE EXPERIMENT.  Remove drag (beta -> Inf) and the beta = 500
%    sea-level range must move by a large factor.  If the drag term is ever
%    mis-wired to zero, all four sea-level rows land near 279 kft and this
%    ratio collapses to 1 -- which is the one failure the benchmark rows alone
%    would report as "three of four red" without ever naming the cause.
i500 = find([C.beta] == 500 & [C.alt0] == 0, 1);
iInf = find(~isfinite([C.beta]) & [C.alt0] == 0, 1);
if isempty(i500) || isempty(iInf)
    K.drag_ratio = NaN;
    names{end+1}  = 'delete experiment: drag term is load-bearing (>3x range)';
    oks(end+1)    = false;
    detail{end+1} = 'the sea-level beta=500 / no-drag pair is missing from the case table';
else
    K.drag_ratio  = C(iInf).ours/C(i500).ours;
    names{end+1}  = 'delete experiment: drag term is load-bearing (>3x range)';
    oks(end+1)    = K.drag_ratio >= 3;
    detail{end+1} = sprintf('no drag / beta=500 = %.2fx (%.1f vs %.1f kft)', ...
        K.drag_ratio, C(iInf).ours, C(i500).ours);
end

% 5  Downrange never reverses.  Drag only decelerates ALONG the velocity
%    vector, so VM1 stays positive for a sub-vertical launch; a sign error on
%    AM1 shows up here and nowhere else cheap.
K.dr_min_step = min([C.dr_min_d]);
names{end+1}  = 'downrange strictly increasing in all seven cases';
oks(end+1)    = K.dr_min_step > 0;
detail{end+1} = sprintf('smallest downrange step %+.4f ft', K.dr_min_step);

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
function B = benchmarks(C, SW)
%BENCHMARKS  Computed vs published, with the provenance of every band.
%
%   Every book value was grepped out of zarchan_decoded.txt; .source carries
%   BOTH files' line numbers because the two are not aligned in Ch.10.  Bands:
%   10% on ranges, 15% on the log-axis deceleration.  Do NOT widen one to turn
%   a row green -- the header says what each band is made of.
B = emptyBench();

% ---- 1-7.  the seven published flyout ranges ------------------------------
for k = 1:numel(C)
    if isfinite(C(k).beta)
        nm = sprintf('flyout range, 45 deg, alt0 = %g kft, beta = %g', ...
                     C(k).alt0/1000, C(k).beta);
        dt = sprintf('T = %.2f s; the book prints "%g kft" with an "about"', ...
                     C(k).tof, C(k).book);
    else
        nm = sprintf('flyout range, 45 deg, alt0 = %g kft, NO DRAG', ...
                     C(k).alt0/1000);
        dt = sprintf(['T = %.2f s; our side is exact vacuum ballistics ' ...
                      '(closed form %.3f kft, structural row 1), so the gap ' ...
                      'is the book''s reading of its own figure'], ...
                      C(k).tof, vacuumRangeKft(3000.0, 45.0, C(k).alt0));
    end
    B = addVal(B, nm, C(k).book, C(k).ours, 'prose "about"', 0.10, ...
               C(k).src, 'kft', dt);
end

% ---- 8.  the drag deceleration behind Fig. 10.4 ---------------------------
% Model evaluation only, no integration: n_D/g = 0.5*rho*V^2/beta.
nD = 0.5*rhoOf(0.0)*3000.0^2/500.0;
B = addVal(B, 'drag deceleration at sea level, 3000 ft/s, beta = 500', ...
    22.0, nD, 'prose "about", log axis', 0.15, ...
    'Fig. 10.4 text, p.218 (zarchan.txt:3052)', 'g', ...
    ['no integrator involved: the listing''s own DRAG = Q*32.2/BETA at ' ...
     'rho(0).  Band 15% because the sentence reads "about", on a LOG axis. ' ...
     'The same sentence quotes 3.5 g at 50 kft, where swapping the two ' ...
     'scale heights would move us +83% -- so 15% still discriminates']);

% ---- 9.  SHAPE: 45 deg maximises the drag-free range ----------------------
if ~isempty(SW) && any(SW.gamdeg == 45) && numel(SW.gamdeg) >= 5
    [pk, ip] = max(SW.range_kft);
    B = addShape(B, '45 deg gives maximum range (no drag)', ...
        'argmax over the launch-angle sweep is 45 deg', ...
        sprintf('argmax at %g deg, %.2f kft (%d angles, H = %g s)', ...
                SW.gamdeg(ip), pk, numel(SW.gamdeg), SW.H), ...
        SW.gamdeg(ip) == 45, 'Fig. 10.11 text, p.223 (zarchan.txt:3109)', ...
        ['an ordering, not a plotted value.  The sweep is coarse and runs at ' ...
         'a coarser H on purpose: range moves 0.03% between H = 0.05 and ' ...
         '0.01 s, while 40 and 50 deg sit 4.3 kft below the peak']);
else
    B = addSkip(B, '45 deg gives maximum range (no drag)', 'argmax = 45 deg', ...
        ['the launch-angle sweep does not contain 45 deg (or carries fewer ' ...
         'than five angles), so "the maximum is at 45" would be a statement ' ...
         'about the sweep rather than about the trajectory'], ...
        'Fig. 10.11 text, p.223 (zarchan.txt:3109)', 'deg');
end

% ---- 10.  SHAPE: drag matters much less from 50 kft -----------------------
r0  = ratioOf(C, 2000, 0);
r50 = ratioOf(C, 2000, 50000);
B = addShape(B, 'drag penalty much smaller for a 50 kft launch', ...
    'range(beta=2000)/range(no drag) far higher at 50 kft than at sea level', ...
    sprintf('%.3f at 50 kft vs %.3f at sea level (margin %+.3f, want >= 0.20)', ...
            r50, r0, r50 - r0), ...
    isfinite(r0) && isfinite(r50) && (r50 - r0) >= 0.20, ...
    'p.225 text (zarchan.txt:3121)', ...
    ['the book''s claim is comparative ("much smaller than before"), so it ' ...
     'is scored as a comparison.  0.20 is a floor on "much": these two ' ...
     'ratios are expected to differ by about 0.50']);

% ---- INFO.  Reported, never adjudicated. ---------------------------------
T0 = 2*500.0/(rhoOf(0.0)*32.2*3000.0);
B = addInfo(B, 'Figs. 10.5-10.8 velocity-halving times', ...
    'about 4.5 s at beta = 500', ...
    sprintf('T0 = 2*beta/(rho*g*V0) = %.2f s', T0), ...
    'p.218 text (zarchan.txt:3059, :3060)', ...
    ['NOT SCORED: those figures come from the CONSTANT-ALTITUDE, gravity-' ...
     'free closed form V1 = V0*T0/(T0+dt) at zarchan.txt:3042-3046, where V ' ...
     'halves at dt = T0.  Listing 10.1 always carries gravity and flies an ' ...
     'arc, so it cannot produce that curve at all -- the agreement above is ' ...
     'between the book''s formula and the book''s prose, not a check on ' ...
     'this port']);
end


function r = ratioOf(C, beta, alt0)
%RATIOOF  Drag-penalty ratio at one launch altitude: the beta case over the
%   no-drag case at the SAME altitude.
i = find([C.beta] == beta & [C.alt0] == alt0, 1);
j = find(~isfinite([C.beta]) & [C.alt0] == alt0, 1);
if isempty(i) || isempty(j)
    r = NaN;
else
    r = C(i).ours/C(j).ours;
end
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly -- this is the
%   schema mgs.zarchan.benchSuite's benchRows() consumes, and adaptGeneric is
%   strict about it on purpose.
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
%ADDINFO  Reported, never scored: it can neither pass nor fail.
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
p = R.p;
L{end+1} = 'Zarchan Listing 10.1 -- TRAJECTORY SIMULATION (ballistic flyout)';
L{end+1} = '*** UNITS ARE FEET, ft/s, lb/ft^2, g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['   this run: VM = %g ft/s   gamma = %g deg   beta = %g ' ...
    'lb/ft^2   alt0 = %g ft   H = %g s'], p.VM, p.GAMDEG, p.BETA, p.ALT0, p.H);
L{end+1} = sprintf('   range %.3f kft in %.2f s (%d steps, recorded every %g s)', ...
    R.range_kft, R.tof_s, p.n_step, p.samp);
L{end+1} = '   NOT A GUIDED FLIGHT: no seeker, no autopilot, no guidance law, no lift.';
L{end+1} = '';

L{end+1} = 'THE SEVEN PUBLISHED CASES  (always at the book''s 3000 ft/s and 45 deg)';
L{end+1} = sprintf('%10s %9s %11s %11s %10s %9s', ...
    'beta', 'alt0 kft', 'ours kft', 'book kft', 'pct', 'T [s]');
for k = 1:numel(R.cases)
    c = R.cases(k);
    if isfinite(c.beta)
        bs = sprintf('%10g', c.beta);
    else
        bs = sprintf('%10s', 'no drag');
    end
    L{end+1} = sprintf('%s %9g %11.3f %11g %+9.2f%% %9.2f', ...
        bs, c.alt0/1000, c.ours, c.book, 100*(c.ours/c.book - 1), c.tof); %#ok<AGROW>
end
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-52s %s', vstr(K.oks(i)), K.names{i}, ...
        K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning';
    L{end+1} = '            NOT CHECKED, which is not the same thing as "fine".';
else
    L{end+1} = 'BENCHMARKS  (prose values grepped from zarchan_decoded.txt; the';
    L{end+1} = '             line number in .source resolves in either book file)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
            case 'info'
                L{end+1} = sprintf('   INFO %-50s %s', b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case 'skip'
                L{end+1} = sprintf('   SKIP %-50s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case 'shape'
                L{end+1} = sprintf('   %-4s %-50s %s', vstr(b.ok), b.what, ...
                    b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf(['   %-4s %-50s book %s %s / ours %s %s ' ...
                    '(%+.2f%%, band %.4g%%)'], vstr(b.ok), b.what, ...
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
L{end+1} = 'REMINDER: Ch.10 is the start of the book''s BALLISTIC half.  This file';
L{end+1} = '   pins the drag model and the atmosphere that Ch.16 reuses.  It says';
L{end+1} = '   nothing about guidance, spread or terminal accuracy.';
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
%DRAWCURVES  The listing's own figure: altitude against downrange, both in kft.
figure;
plot(R.downrange_kft, R.alt_kft, '-');
grid on;
xlabel('Downrange (Kft)');
ylabel('Altitude (Kft) ');
if isfinite(R.p.BETA)
    bs = sprintf('beta = %g lb/ft^2', R.p.BETA);
else
    bs = 'no drag';
end
title(sprintf('Listing 10.1 flyout: %g ft/s at %g deg, %s', ...
    R.p.VM, R.p.GAMDEG, bs));
end
