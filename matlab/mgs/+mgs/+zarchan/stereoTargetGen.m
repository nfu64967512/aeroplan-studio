function R = stereoTargetGen(opts)
%MGS.ZARCHAN.STEREOTARGETGEN  Zarchan Listing 34.2 -- TARGET TRAJECTORY
%   GENERATOR.  The boosting threat that Ch. 34's kill-vehicle sizing, and the
%   stereo-tracking Monte Carlo of Listing 34.4, both fly against.
%
%   R = mgs.zarchan.stereoTargetGen()
%   R = mgs.zarchan.stereoTargetGen('ITGT', 1)              IRBM only, faster
%   R = mgs.zarchan.stereoTargetGen('H', 0.02, 'assert', false)
%
%   Port of zarchan.txt:13309-13337 (driver :13309/:13313, loop :13313-:13336).
%   LAMBERT3D and distance3dkm are Listing 28.3 and arrive here as local
%   functions copied from MGS.ZARCHAN.IMPULSIVEICBM, where the nine OCR
%   resolutions inside LAMBERT3D are justified line by line.  This is a TRUTH
%   MODEL: no seeker, no filter, no guidance loop for the interceptor.  It
%   makes a boosting missile fly where Lambert tells it to, and the only things
%   worth testing about it are CONSERVATION and CLOSED FORMS.
%
%   UNITS ARE THE BOOK'S: ft, ft/s, ft/s^2, g = 32.2 ft/s^2, A = 2.0926e7 ft,
%   GM = 1.4077e16 ft^3/s^2, km = 3280 ft (the listing's own divisor, NOT
%   3280.84), degrees = 57.3*radians.  Reported quantities are km, km/s, s, g.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_34_02.m reports 2 unresolved variable-to-
%   variable comparisons.  This book's OCR maps '<' -> ',' and '>' -> '.', and
%   every comparison in 34.2 falls out of that one substitution:
%
%     while ALT . -1        -> ALT > -1     [fly until 1 ft below the surface]
%     while STEP , = 1      -> STEP <= 1    [body sets STEP = 2 to leave]
%     if T , 180. / , 120 / , 240.          -> T <  (stage boundaries)
%     if S . = .99999       -> S >= .99999  [S is reset on every record]
%     elseif DEL , 500      -> DEL <  500
%
%   The two the extractor could NOT settle from shape, and how they are settled
%   here -- both are variable-vs-variable, so neither is forced by a literal:
%
%     1  "if T,TPZ&DEL.500"  ->  if T < TPZ && DEL > 500.  TPZ is the BURN TIME
%        (180 s IRBM, 240 s ICBM; prose :13145 and :13597 both say so) and this
%        is the branch that APPLIES thrust, so thrust must be on BEFORE burnout
%        and the velocity deficit DEL must be LARGE for anything to be left to
%        burn.  The elseif is the literal "DEL < 500" -- deficit closed, cut
%        off -- so the other reading makes the two branches the same test.
%     2  "if T,TUPT"  ->  if T < TUPT.  This is the vertical-rise override, and
%        the prose names it directly for the sibling listing: "the ICBM goes
%        straight up for 20 s (TUPT = 20)", zarchan.txt:12604.  Read as
%        "T > TUPT" the override would replace Lambert steering for the WHOLE
%        boost and the vehicle would go up and come straight back down.  That
%        is SELFCHECK 5, run as a delete experiment rather than asserted.
%
%   Dropped, not "fixed" into code: the "figure plot(...)" tail and "save
%   datfil.txt output /ascii" (the book's own typo for -ascii; nothing is
%   written to disk here).  Dead, kept as comments at their sites: "FLAG=0"
%   after the inner loop, "TRST=0." and "X1OLD=X1; Y1OLD=Y1" in the cut-off
%   branch.  Declared and never read by 34.2: QMIN, LEFT, CW, QOOMPH, XLONGM,
%   TFINISH.
%
%   THE BOOK CONTRADICTS ITSELF ABOUT TUPT.  The listing sets TUPT = 15
%   (:13309); the prose describing its nominal run says "(TLOFT = 200,
%   TUPT = 200)" (:13338).  The LISTING is scored, because TUPT = 200 exceeds
%   the 180-s burn -- the whole boost would be vertical and a 2000-km trajectory
%   would be impossible.  Both lines are cited on an INFO row.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Every book number was grepped out of zarchan_decoded.txt and
%   its line is in the row's .source.  Nothing was read off a figure IMAGE --
%   the extraction turned every figure into an image link.
%
%   BAND CONVENTION, fixed before anything was run: a value the book prints as
%   "approximately N" is scored to HALF ITS LAST PRINTED DIGIT (9 g -> +-0.5 g
%   = 5.56%; 13 g -> 3.85%), capped at 10% so that a one-significant-figure
%   apogee cannot be scored to +-25%.  A prose claim that is a READING OFF A
%   FIGURE with no independent arithmetic behind it (the cloud-deck crossings)
%   is scored as a BOUND at +-50%, not as a value: a tighter band there would
%   be scoring the author's eye against a 10-s tic grid, not the model.
%
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED. ***  MATLAB is the
%   caller's, not this author's; this file has never been executed.
%     TF  = 252 + .223*RDESKM - 5.44e-6*RDESKM^2 + TLOFT
%         = 676.24 + 200 = 876.24 s (IRBM),  1938 + 500 = 2438 s (ICBM)
%     apparent acceleration = TRST/WGT in g, exactly:
%         IRBM  t=0     54100/44000  = 1.2295 g
%         IRBM  t=180-  54100/5840   = 9.2637 g   vs the book's "9 g"
%         ICBM  t=120-  725850/126020= 5.7598 g   vs the book's "6 g"
%         ICBM  t=120+  182250/91080 = 2.0010 g   (the staging discontinuity)
%         ICBM  t=240-  182250/14040 = 12.9808 g  vs the book's "13 g"
%     aim point: A*XLONGT with XLONGT = RDESKM*3280/A is RDESKM km of arc
%       EXACTLY, the 57.3 cancelling -- SELFCHECK 3 is a machine-precision row.
%     IRBM apogee: a two-body radial estimate for the ~695 s of coast that a
%       876 s time of flight leaves puts it near 500 km, which is what the
%       book reads off Fig. 34.6.  That is an estimate, not a prediction.
%   If a residual comes back near a band EDGE that is not a pass to be
%   accepted.  Do not widen a band to make a row green.
%
%   OUTPUTS: R.cases (one struct per ITGT: .TF .t_impact_s .downrange_km
%   .apogee_km .apogee_kepler_km .t_bo_s .bo_reason .t7km_s .dE .dL .resid
%   .iters .hit_cap and the 1-s histories), R.bench, R.selfcheck
%   (.names/.oks/.detail), R.pass (true only if a row was SCORED and all
%   passed), R.verdict (ALWAYS formed; 'assert' picks error vs warn and
%   nothing else), R.p, R.units, R.summary.
%
%   See also MGS.ZARCHAN.IMPULSIVEICBM (Listing 32.1, the impulsive twin),
%   MGS.ZARCHAN.ICBMTRAJECTORY (28.2), MGS.ZARCHAN.BENCHSUITE.

arguments
    % 1 = IRBM (the listing's own nominal case), 2 = ICBM.  The ICBM's
    % RDESKM/TLOFT/TUPT are the prose's, zarchan.txt:13597, not the driver
    % line's -- see caseSetup below.
    opts.ITGT (1,:) double {mustBeMember(opts.ITGT, [1 2])} = [1 2]
    opts.H    (1,1) double {mustBeFinite, mustBePositive} = 0.01   % s, listing
    opts.TS   (1,1) double {mustBeFinite, mustBePositive} = 1.0    % s, record
    % NOT nargin: with only name-value options declared, MATLAB reports
    % nargin = 0 however many were passed, so a mode switch on it never fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

A     = 2.0926E7;      % ft
GM    = 1.4077E16;     % ft^3/s^2
FTPKM = 3280.;         % the BOOK'S ft/km

if opts.H >= opts.TS
    error('mgs:zarchan:stereoTargetGen:step', ...
        ['H = %g s is not smaller than the record interval TS = %g s, so the ' ...
         'trajectory would be sampled at the step rate while claiming the ' ...
         'coarser interval in its own time vector.'], opts.H, opts.TS);
end

ITGT = unique(opts.ITGT, 'stable');
C = runCase(ITGT(1), opts.H, opts.TS, A, GM, FTPKM, false);
for k = 2:numel(ITGT)
    C(k) = runCase(ITGT(k), opts.H, opts.TS, A, GM, FTPKM, false);
end

R = struct();
R.cases = C;
R.units = ['state ft, ft/s (A = 2.0926e7 ft, GM = 1.4077e16 ft^3/s^2, ' ...
           'g = 32.2 ft/s^2); reported km, km/s, s, g via the book''s 3280 ft/km'];
R.p = struct('ITGT', ITGT, 'H', opts.H, 'TS', opts.TS, 'A_ft', A, 'GM', GM, ...
             'FT_PER_KM', FTPKM, 'TFINISH_dead', 3000., 'units', R.units);

R.selfcheck = structuralChecks(C, opts.H, A, GM, FTPKM);

if opts.benchmark
    R.bench = benchmarks(C);
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
if opts.plot,   drawTraj(R);              end

msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = ['not one book value could be evaluated.  An unvalidated port ' ...
           'must not be handed back silently.'];
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
        error('mgs:zarchan:stereoTargetGen:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN.  R comes back complete so the caller can see WHICH
        % side of a comparison moved; benchSuite runs every port with assert
        % off for exactly that reason.
        warning('mgs:zarchan:stereoTargetGen:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [RDESKM, TLOFT, TUPT, TPZ, name] = caseSetup(ITGT)
%CASESETUP  The two cases the book actually publishes.
%   ITGT = 1 is the listing's own driver line verbatim (zarchan.txt:13309).
%   ITGT = 2 takes TPZ = 240 from the listing and RDESKM/TLOFT/TUPT from the
%   prose that re-runs this same listing for the ICBM: "a sample lofted
%   10,000-km trajectory for the ICBM was generated using Listing 34.2
%   (ITGT = 2, TLOFT = 500, TUPT = 20)", zarchan.txt:13597.  RDESKM is not
%   written as a variable there; "10,000-km" is.
if ITGT == 1
    RDESKM = 2000.;  TLOFT = 200.;  TUPT = 15.;  TPZ = 180.;  name = 'IRBM';
else
    RDESKM = 10000.; TLOFT = 500.;  TUPT = 20.;  TPZ = 240.;  name = 'ICBM';
end
end


% =============================================================================
function [AT, WGT, TRST] = atProfile(T, ITGT)
%ATPROFILE  The listing's thrust/weight block, zarchan.txt:13316-13322,
%   verbatim.  AT is the APPARENT TARGET ACCELERATION in ft/s^2; AT/32.2 is
%   the g figure the chapter quotes.  Every constant is an argument, because
%   MATLAB local functions cannot see the caller's workspace.
if ITGT == 1
    if T < 180.
        WGT = -212.*T + 44000.;  TRST = 54100.;
    else
        WGT = 3300.;  TRST = 0.;
    end
else
    if T < 120
        WGT = -2622*T + 440660.;  TRST = 725850.;
    elseif T < 240.
        WGT = -642.*T + 168120.;  TRST = 182250.;
    else
        WGT = 5500.;  TRST = 0.;
    end
end
AT = 32.2*TRST/WGT;
end


% =============================================================================
function C = runCase(ITGT, H, TS, A, GM, FTPKM, flipUPT)
%RUNCASE  Listing 34.2, zarchan.txt:13313-13336.
%
%   flipUPT is SELFCHECK 5's delete experiment: it swaps the resolution of the
%   vertical-rise gate ("T < TUPT" -> "T > TUPT") and nothing else.  It lives
%   INSIDE this function on purpose -- a copied-out variant would be free to
%   drift away from the transcription it is supposed to be testing.
[RDESKM, TLOFT, TUPT, TPZ, name] = caseSetup(ITGT);

% ---- driver line ---------------------------------------------------------
QBOOST = 1;  SWITCH = 0;  GAMDEG = 89.99;
T = 0.;  S = 0.;
ALT = 0;  ANGDEG = 0.;  ANG = ANGDEG/57.3;
X = (A+ALT)*cos(ANG);  Y = (A+ALT)*sin(ANG);  Z = 0;
ALT = sqrt(X^2+Y^2) - A;
XFIRST = X;  YFIRST = Y;  ZFIRST = Z;
% X1,Y1 are a UNIT vector, i.e. the vehicle starts at 1 ft/s pointing 89.99 deg
% above the local horizontal.  Not a typo for a velocity: it is the book's way
% of giving a direction to a vehicle that is effectively at rest.
X1 = cos(1.5708 - GAMDEG/57.3 + ANG);
Y1 = sin(1.5708 - GAMDEG/57.3 + ANG);
AXT = 0.;  AYT = 0.;
XLONGTDEG = 57.3*RDESKM*FTPKM./A;
TF = 252. + .223*RDESKM - (5.44E-6)*RDESKM*RDESKM;    % tFME, appendix fit
TF = TF + TLOFT;
XLONGT = XLONGTDEG/57.3;                              % the 57.3 cancels exactly
XF = A*cos(XLONGT);  YF = A*sin(XLONGT);  ZF = 0;

% ---- bookkeeping this port adds (the listing has none) --------------------
nmax    = ceil(4*TF/TS) + 8;
aT = zeros(nmax,1);  aD = zeros(nmax,1);  aA = zeros(nmax,1);  aV = zeros(nmax,1);
count   = 0;
% The listing's "while ALT > -1" is open ended.  Guarded here so a hyperbolic
% or non-finite solution cannot hang a suite; the cap is REPORTED via hit_cap
% rather than silently absorbed.  (TFINISH = 3000 is set by the driver line and
% never read by 34.2, so it is not used as the cap.)
stepMax = ceil(4*TF/H) + 16;
nStep   = 0;
itMax   = 0;  rsMax = 0;
TBO     = NaN;  boReason = 'still boosting at impact';  DELbo = NaN;
apoKm   = -inf;  t7 = NaN;  prevAltKm = (sqrt(X^2+Y^2)-A)/FTPKM;
E0 = NaN;  L0 = NaN;  dE = 0;  dL = 0;
% Pre-initialised so the FLAG==1 branch cannot read a value left over from
% anything else.  (DEL is NOT pre-initialised: it is assigned and read inside
% the same QBOOST block, and a stale NaN there would be worse than an error.)
X1D = 0.;  Y1D = 0.;  AT = 0.;

while ALT > -1 && nStep < stepMax
    XOLD = X;  YOLD = Y;  X1OLD = X1;  Y1OLD = Y1;
    STEP = 1;  FLAG = 0;
    % Zarchan's two-pass predictor loop.  Pass 1 evaluates the derivatives at
    % the current state; pass 2 takes a full Euler step, advances T by H and
    % re-evaluates there.  T advances INSIDE this loop, exactly once, which is
    % what makes the second evaluation happen at t+H.  Move that line out and
    % the scheme quietly degrades to first order while still running smoothly.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            X  = X  + H*X1;   Y  = Y  + H*Y1;
            X1 = X1 + H*X1D;  Y1 = Y1 + H*Y1D;
            T  = T  + H;
        end
        AT = atProfile(T, ITGT);
        TEMBOT = (X^2+Y^2)^1.5;
        X1D = -GM*X/TEMBOT + AXT;
        Y1D = -GM*Y/TEMBOT + AYT;
        ALT = sqrt(X^2+Y^2) - A;      % the loop test reads THIS, pre-corrector
        FLAG = 1;
    end
    % The listing's "FLAG = 0" here is dead: the outer loop re-initialises FLAG
    % on its next pass.  Kept as a comment so nobody re-adds it thinking a
    % state machine was lost.
    X  = (XOLD  + X )/2 + .5*H*X1;
    Y  = (YOLD  + Y )/2 + .5*H*Y1;
    X1 = (X1OLD + X1)/2 + .5*H*X1D;
    Y1 = (Y1OLD + Y1)/2 + .5*H*Y1D;
    S  = S + H;

    if QBOOST == 1
        TGOLAM = TF - T;
        [VRX, VRY, ~, it, rs] = lambert3d(X, Y, Z, TGOLAM, XF, YF, ZF, SWITCH, GM);
        itMax = max(itMax, it);  rsMax = max(rsMax, rs);
        DELX = VRX - X1;  DELY = VRY - Y1;  DEL = sqrt(DELX^2 + DELY^2);
        if T < TPZ && DEL > 500
            AXT = AT*DELX/DEL;  AYT = AT*DELY/DEL;
        elseif DEL < 500
            % Velocity deficit closed: cut off and snap onto the Lambert
            % solution.  Three of the listing's assignments here are dead and
            % are dropped: "TRST=0." (atProfile recomputes TRST from T on the
            % very next derivative pass) and "X1OLD=X1; Y1OLD=Y1" (both are
            % overwritten at the top of the next outer pass).  QBOOST=0 is
            % what actually ends the burn, by gating this whole block.
            QBOOST = 0;  AXT = 0.;  AYT = 0.;
            X1 = VRX;  Y1 = VRY;
            TBO = T;  DELbo = DEL;  boReason = 'DEL < 500 ft/s (velocity matched)';
        else
            QBOOST = 0;  AXT = 0.;  AYT = 0.;
            TBO = T;  DELbo = DEL;  boReason = 'T >= TPZ (burn time expired)';
        end
        % The vertical-rise override, resolved as T < TUPT.  See the header,
        % resolution 2, and SELFCHECK 5.
        if flipUPT,  upNow = T > TUPT;  else,  upNow = T < TUPT;  end
        if upNow
            RTMAG = sqrt(X^2+Y^2);
            AXT = AT*X/RTMAG;  AYT = AT*Y/RTMAG;
        end
    end

    nStep = nStep + 1;
    altKm = (sqrt(X^2+Y^2) - A)/FTPKM;
    if altKm > apoKm, apoKm = altKm; end
    % First upward crossing of the 7-km cloud deck (zarchan.txt:13339/:13597),
    % linear within the step.  Measured at STEP rate, not at the 1-s record
    % rate, so the answer does not depend on TS.
    if isnan(t7) && altKm >= 7 && prevAltKm < 7
        t7 = T - H*(altKm - 7)/max(altKm - prevAltKm, realmin);
    end
    prevAltKm = altKm;

    if S >= TS - 0.5*H
        S = 0.;
        count = count + 1;
        aT(count) = T;
        aD(count) = distance3dkm(X, Y, Z, XFIRST, YFIRST, ZFIRST, A, FTPKM);
        aA(count) = altKm;
        aV(count) = sqrt(X1^2 + Y1^2)/FTPKM;
        if QBOOST == 0
            % Kepler invariants of the BALLISTIC arc only.  Thrust is a real
            % external force, so energy is not conserved before burnout and a
            % check that included the boost would be measuring nothing.
            E = 0.5*(X1^2 + Y1^2) - GM/sqrt(X^2 + Y^2);
            L = X*Y1 - Y*X1;
            if isnan(E0)
                E0 = E;  L0 = L;
            else
                dE = max(dE, abs(E/E0 - 1));
                dL = max(dL, abs(L/L0 - 1));
            end
        end
    end
end

% Closed-form apogee of the post-burnout conic, from the FIRST ballistic
% sample.  Shares no code with the integrator, which is the whole point.
if isfinite(E0) && E0 < 0
    aSma = -GM/(2*E0);
    ecc  = sqrt(max(0, 1 + 2*E0*L0^2/GM^2));
    apoCf = (aSma*(1+ecc) - A)/FTPKM;
else
    apoCf = NaN;                       % escape, or never burned out
end

C = struct('name', name, 'ITGT', ITGT, 'RDESKM', RDESKM, 'TLOFT', TLOFT, ...
    'TUPT', TUPT, 'TPZ', TPZ, 'TF', TF, 't_impact_s', T, ...
    'downrange_km', distance3dkm(X, Y, Z, XFIRST, YFIRST, ZFIRST, A, FTPKM), ...
    'apogee_km', apoKm, 'apogee_kepler_km', apoCf, 't_bo_s', TBO, ...
    'bo_reason', boReason, 'DEL_bo', DELbo, 't7km_s', t7, ...
    'v_impact_kms', sqrt(X1^2+Y1^2)/FTPKM, 'dE', dE, 'dL', dL, ...
    'resid', rsMax, 'iters', itMax, 'hit_cap', nStep >= stepMax, ...
    'theta_rad', XLONGT, 'aim_km', distance3dkm(XF, YF, ZF, XFIRST, YFIRST, ...
    ZFIRST, A, FTPKM), 't', aT(1:count), 'downrange_km_hist', aD(1:count), ...
    'alt_km_hist', aA(1:count), 'vel_kms_hist', aV(1:count));
end


% =============================================================================
function K = structuralChecks(C, H, A, GM, FTPKM)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.
%   /!\ .names and .oks must BOTH exist and be the SAME LENGTH, or benchSuite
%   puts ZERO rows on the board and the board looks green for the wrong reason.
names = {};  oks = [];  detail = {};

% 1  CONSERVATION.  On the ballistic arc the only force is point-mass gravity,
%    so specific energy and angular momentum are constants of the motion.  A
%    sign error in X1D/Y1D, or a corrector averaging the wrong pair of states,
%    shows up here and nowhere else cheap: the trajectory would still look
%    exactly like a trajectory.
r1 = max([max([C.dE]) max([C.dL])]);
names{end+1} = 'specific energy and angular momentum conserved after burnout';
oks(end+1)   = isfinite(r1) && r1 <= 1e-5;
detail{end+1} = sprintf('worst relative drift max(|dE/E0|, |dL/L0|) = %.2e over %d case(s)', ...
    r1, numel(C));

% 2  CLOSED FORM.  Integrated apogee against r_a = a(1+e) built from the first
%    ballistic sample.  Two independent routes to one number.
rel2 = abs([C.apogee_km]./[C.apogee_kepler_km] - 1);
names{end+1} = 'integrated apogee == Kepler r_a = a(1+e) from the burnout state';
oks(end+1)   = all(isfinite(rel2)) && all(rel2 <= 0.005);
detail{end+1} = sprintf('integrated [%s] km vs closed form [%s] km (worst %+.3e rel)', ...
    strtrim(sprintf('%.1f ', [C.apogee_km])), ...
    strtrim(sprintf('%.1f ', [C.apogee_kepler_km])), maxOr(rel2));

% 3  THE UNIT CHAIN.  XLONGTDEG = 57.3*RDESKM*3280/A then XLONGT =
%    XLONGTDEG/57.3, so the aim point sits RDESKM km of great-circle arc from
%    the launch point EXACTLY.  Machine precision, and it is what would notice
%    someone "improving" 57.3 to 180/pi on one line but not the other, or
%    3280 to 3280.84.
rel3 = abs([C.aim_km]./[C.RDESKM] - 1);
names{end+1} = 'aim point is exactly RDESKM km of arc from the launch point';
oks(end+1)   = all(rel3 <= 1e-12);
detail{end+1} = sprintf('worst |aim/RDESKM - 1| = %.2e', maxOr(rel3));

% 4  LAMBERT3D converged on its OWN 1e-8 criterion, not by falling out of the
%    ITERS > 100 break.  The book's routine gives the caller no way to tell,
%    and a mis-resolved bracket comparison returns the 100th iterate silently.
names{end+1} = 'LAMBERT3D meets its own 1e-8 criterion, not the ITERS>100 break';
oks(end+1)   = max([C.resid]) <= 1e-8 && max([C.iters]) <= 100 && ...
               ~any([C.hit_cap]);
detail{end+1} = sprintf('worst |TF-T|/TF = %.2e, worst ITERS = %d, step cap hit: %s', ...
    max([C.resid]), max([C.iters]), mat2str([C.hit_cap]));

% 5  DELETE EXPERIMENT on resolution 2, the only one of the two unresolved
%    comparisons that a wrong reading would leave LOOKING fine.  Flip
%    "T < TUPT" to "T > TUPT" -- radial thrust for the whole boost instead of
%    the first TUPT seconds -- and the vehicle must miss its aim point badly.
%    Run at a coarse H because the claim is "grossly wrong", not "wrong in the
%    fourth digit"; a fine step would only make the same point slower.
[ok5, d5] = flipExperiment(C(1).ITGT, max(H, 0.05), A, GM, FTPKM);
names{end+1} = 'FLIP the vertical-rise gate (T<TUPT -> T>TUPT) -> range collapses';
oks(end+1)   = ok5;
detail{end+1} = d5;

K.names = names;  K.oks = logical(oks);  K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    q = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{q}, detail{q});
end
end


% =============================================================================
function [ok, txt] = flipExperiment(ITGT, H, A, GM, FTPKM)
%FLIPEXPERIMENT  SELFCHECK 5.  Re-flies THE SAME runCase above with only the
%   vertical-rise comparison swapped.  "Broken" means the flown range departs
%   from the aim point by more than 10%, or the run does not terminate inside
%   the step cap, or it returns something non-finite.
F = runCase(ITGT, H, 1.0, A, GM, FTPKM, true);
rel = F.downrange_km/F.RDESKM - 1;
ok  = ~isfinite(rel) || abs(rel) > 0.10 || F.hit_cap;
txt = sprintf(['flipped: downrange %.1f km against an aim of %g km (%+.1f%%), ' ...
    'apogee %.1f km, impact %.1f s, cap hit %d, at H = %g s'], ...
    F.downrange_km, F.RDESKM, 100*rel, F.apogee_km, F.t_impact_s, F.hit_cap, H);
end


% =============================================================================
function B = benchmarks(C)
%BENCHMARKS  Book values grepped from zarchan_decoded.txt; the line number is
%   in each row's .source.  See the header for the band convention.
B = emptyBench();

% ---- 1-3  the apparent-acceleration profile.  Closed form, no integration:
%           AT/g = TRST/WGT, evaluated in the limit approaching each stage
%           boundary from below (the branches are "T < 180." / "< 120" /
%           "< 240.", so the boundary itself belongs to the next branch).
B = addVal(B, 'IRBM peak apparent acceleration at burnout (t -> 180 s)', ...
    9, atProfile(180-1e-6, 1)/32.2, '5.56% (half of "9")', 0.5/9, ...
    ['prose "the maximum acceleration of the IRBM is approximately 9 g at ' ...
     '180 s", zarchan.txt:13145; the same 9 g is coded as XNTMAX = 9.*32.2 ' ...
     'in Listing 34.1, zarchan.txt:13193'], 'g', ...
    'TRST/WGT = 54100/(44000 - 212*180); no integrator involved');

B = addVal(B, 'ICBM peak acceleration, end of first stage (t -> 120 s)', ...
    6, atProfile(120-1e-6, 2)/32.2, '8.33% (half of "6")', 0.5/6, ...
    ['prose about Fig. 33.1, zarchan.txt:12589.  Listing 33.1 codes the ' ...
     'IDENTICAL WGT/TRST profile (zarchan.txt:12646-12648), so the number ' ...
     'is a value for this listing''s ITGT = 2 branch'], 'g', ...
    'TRST/WGT = 725850/(440660 - 2622*120)');

B = addVal(B, 'ICBM peak acceleration, end of second stage (t -> 240 s)', ...
    13, atProfile(240-1e-6, 2)/32.2, '3.85% (half of "13")', 0.5/13, ...
    'prose about Fig. 33.1, zarchan.txt:12589', 'g', ...
    'TRST/WGT = 182250/(168120 - 642*240)');

% ---- 4-6  the IRBM trajectory ------------------------------------------
j = find([C.ITGT] == 1, 1);
SRC1 = 'prose about Fig. 34.6, zarchan.txt:13338';
if isempty(j)
    B = addSkip(B, 'IRBM downrange travelled at impact', '2000', ...
        'ITGT = 1 was not among the cases flown', SRC1, 'km');
    B = addSkip(B, 'IRBM apogee', '500', ...
        'ITGT = 1 was not among the cases flown', SRC1, 'km');
    B = addSkip(B, 'IRBM impact time vs the listing''s own tFME + TLOFT', ...
        '876.24', 'ITGT = 1 was not among the cases flown', ...
        'zarchan.txt:13313', 's');
else
    B = addVal(B, 'IRBM downrange travelled at impact', 2000, ...
        C(j).downrange_km, '0.5%', 0.005, ...
        [SRC1 '; RDESKM = 2000 in the driver line, zarchan.txt:13309'], 'km', ...
        'great-circle arc from the launch point at the final integrated state');

    B = addVal(B, 'IRBM apogee', 500, C(j).apogee_km, '10% (band cap)', 0.10, ...
        SRC1, 'km', ...
        ['"approximately 500 km": half of the last printed digit is 10%, ' ...
         'which is also the cap, so the two agree here']);

    B = addVal(B, 'IRBM impact time vs the listing''s own tFME + TLOFT', ...
        C(j).TF, C(j).t_impact_s, '0.5%', 0.005, ...
        ['TF = 252 + .223*RDESKM - 5.44e-6*RDESKM^2 + TLOFT in the driver ' ...
         'line, zarchan.txt:13313; the tFME fit is derived in the appendix ' ...
         'and quoted at zarchan.txt:12156-12158'], 's', ...
        ['this scores whether the INTEGRATION lands where Lambert aimed it. ' ...
         'Burnout: ' C(j).bo_reason]);
end

% ---- 7  the ICBM trajectory --------------------------------------------
k = find([C.ITGT] == 2, 1);
SRC2 = 'prose about Fig. 34.13, zarchan.txt:13597';
if isempty(k)
    B = addSkip(B, 'ICBM apogee (TLOFT = 500, 10,000 km)', '2000', ...
        'ITGT = 2 was not among the cases flown', SRC2, 'km');
else
    B = addVal(B, 'ICBM apogee (TLOFT = 500, 10,000 km)', 2000, ...
        C(k).apogee_km, '10% (band cap)', 0.10, SRC2, 'km', ...
        ['"approximately 2000 km".  Half of the last printed digit would be ' ...
         '25%; the 10% cap of the header convention is used instead, which ' ...
         'is TIGHTER than the book''s own stated precision, deliberately']);
end

% ---- 8-9  the cloud-deck crossings, as BOUNDS ---------------------------
%      These are readings off tic-marked trajectory plots with no arithmetic
%      behind them, so a value row would be scoring the author's eye.
if ~isempty(j)
    B = addBound(B, 'IRBM clears the 7-km cloud deck', 80, 40, 120, ...
        C(j).t7km_s, ...
        ['prose "the IRBM can be seen by airborne IRST sensors at 80 s" for ' ...
         '7-km cloud cover, read off Fig. 34.7''s 10-s tics, zarchan.txt:13339'], ...
        's', 'bound is the book value +-50%; see the header band convention');
end
if ~isempty(k)
    B = addBound(B, 'ICBM clears the 7-km cloud deck', 60, 30, 90, ...
        C(k).t7km_s, ...
        ['prose "breaks the clouds, assuming a 7-km altitude cloud cover, at ' ...
         'approximately 60 s", read off Fig. 34.14''s 20-s tics, ' ...
         'zarchan.txt:13597'], 's', ...
        'bound is the book value +-50%; see the header band convention');
end

% ---- 10  the book against itself ----------------------------------------
B = addInfo(B, 'TUPT: the listing says 15, the prose says 200', ...
    'TUPT = 15 (scored)', ...
    ['zarchan.txt:13309 sets TUPT = 15. in the driver line; zarchan.txt:13338 ' ...
     'describes the SAME nominal run as "(TLOFT = 200, TUPT = 200)".  The ' ...
     'listing is scored: TUPT = 200 exceeds the 180-s burn (TPZ), so the ' ...
     'entire boost would be vertical and no 2000-km trajectory could result. ' ...
     'The ICBM prose at :13597 says TUPT = 20, consistent with the listing''s ' ...
     'order of magnitude, which is further evidence that :13338 is a typo.'], ...
    'zarchan.txt:13309 vs zarchan.txt:13338', 's');
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER are benchSuite's adaptGeneric schema, which
%   is STRICT: a near-miss (book_s for book_str) errors rather than adapting.
%   Every row is built through one of the four constructors below so the order
%   cannot drift.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.9g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.9g', ours);
B(i).band = band;  B(i).tol = tol;
B(i).units = units;  B(i).source = source;  B(i).detail = detail;
if isempty(ours) || ~isfinite(ours) || book == 0
    B(i).pct = NaN;  B(i).ok = false;  B(i).skipped = true;
    B(i).why = 'the computed value is not finite (or the book value is zero)';
    return
end
B(i).pct = 100*(ours/book - 1);
B(i).ok  = abs(ours/book - 1) <= tol;
B(i).skipped = false;  B(i).why = '';
end

function B = addBound(B, what, book, lo, hi, ours, source, units, detail)
%ADDBOUND  A published number that is a FIGURE READING.  Scored as an
%   interval, and the interval is stated in .band so nobody can quote the row
%   as a three-figure agreement.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'bound';
B(i).book = book;  B(i).book_str = sprintf('%.9g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.9g', ours);
B(i).band = sprintf('[%g, %g]', lo, hi);  B(i).tol = NaN;
B(i).units = units;  B(i).source = source;  B(i).detail = detail;
if isempty(ours) || ~isfinite(ours)
    B(i).pct = NaN;  B(i).ok = false;  B(i).skipped = true;
    B(i).why = 'the crossing never happened, so there is nothing to bound';
    return
end
B(i).pct = 100*(ours/book - 1);
B(i).ok  = ours >= lo && ours <= hi;
B(i).skipped = false;  B(i).why = '';
end

function B = addSkip(B, what, bookStr, why, source, units)
%ADDSKIP  A check that COULD NOT RUN.  Never a pass.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'skip';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = '--';
B(i).pct  = NaN;   B(i).band = 'skip';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true;  B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end

function B = addInfo(B, what, oursStr, why, source, units)
%ADDINFO  Recorded, never adjudicated.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'info';
B(i).book = NaN;   B(i).book_str = '(the book disagrees with itself)';
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'info';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true;  B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end


% =============================================================================
function [VRX, VRY, VRZ, ITERS, RESID, GAMMA] = lambert3d(XT, YT, ZT, TF, ...
                                                    XF, YF, ZF, SWITCH1, GM)
%LAMBERT3D  Zarchan Listing 28.3, zarchan.txt:10428.  Copied unchanged from
%   MGS.ZARCHAN.IMPULSIVEICBM, where all nine OCR resolutions inside it are
%   justified line by line and a bracket-flip delete experiment is run on
%   them.  Returns RESID and ITERS so a caller can tell a converged answer
%   from the ITERS > 100 break, which the book's version cannot.
PI_ = 3.1415926535898;
HALFPI = PI_/2.;
RF0X = XF-XT;  RF0Y = YF-YT;  RF0Z = ZF-ZT;
R0DOTRF  = XT*XF + YT*YF + ZT*ZF;
R0DOTRF0 = XT*RF0X + YT*RF0Y + ZT*RF0Z;
R0MAG  = sqrt(XT^2+YT^2+ZT^2);
RFMAG  = sqrt(XF^2+YF^2+ZF^2);
RF0MAG = sqrt(RF0X^2+RF0Y^2+RF0Z^2);
RATIO = R0MAG/RFMAG;  GMDIVR0 = GM/R0MAG;
COS_T = R0DOTRF/(R0MAG*RFMAG);
VNUMER = GMDIVR0*(1.-COS_T);  T_MIN = 0.;
if SWITCH1 == 0
    G_MIN = HALFPI - acos(R0DOTRF0/(R0MAG*RF0MAG));  G_MAX = HALFPI;
    THETA = acos(COS_T);
else
    G_MIN = -HALFPI;  G_MAX = -HALFPI + acos(R0DOTRF0/(R0MAG*RF0MAG));
    THETA = 2.*PI_ - acos(COS_T);
end
SIN_T = sin(THETA);  COT_HALFT = 1./tan(THETA/2.);
GAMMA = (G_MAX+G_MIN)/2.;  GOLD = G_MIN;  TOLD = 0.;  T = 0.;  ITERS = 1;
RV0MAG = NaN;
while abs(TF-T) > (0.00000001*TF)
    SIN_G = sin(GAMMA);  COS_G = cos(GAMMA);  TAN_G = SIN_G/COS_G;
    COS_TPLUSG = cos(THETA+GAMMA-2.*PI_);
    TERM1 = (RATIO*COS_G - COS_TPLUSG)*COS_G;
    RV0MAG = sqrt(VNUMER/TERM1);
    LAMBDA = RV0MAG*RV0MAG/GMDIVR0;
    if LAMBDA < 1.9999999                            % elliptic
        TERM0 = sqrt(2./LAMBDA-1.);
        TERM1 = (TAN_G*(1.-COS_T)+(1.-LAMBDA)*SIN_T)/((2.-LAMBDA)*RATIO);
        TERM2 = (COS_G+COS_G)/(LAMBDA*TERM0*TERM0*TERM0);
        TERM3 = atan2(TERM0, (COS_G*COT_HALFT-SIN_G));
        T = (R0MAG/(RV0MAG*COS_G))*(TERM1+TERM2*TERM3);
    elseif LAMBDA > 2.0000001                        % hyperbolic
        TERM0 = sqrt(1.-2./LAMBDA);
        TERM1 = (TAN_G*(1.-COS_T)+(1.-LAMBDA)*SIN_T)/((2.-LAMBDA)*RATIO);
        TERM2 = COS_G/(LAMBDA*TERM0*TERM0*TERM0);
        TERM3 = SIN_G - COS_G*COT_HALFT;
        TERM3 = log((TERM3-TERM0)/(TERM3+TERM0));
        T = (R0MAG/(RV0MAG*COS_G))*(TERM1-TERM2*TERM3);
    else                                             % parabolic
        TERM0 = COS_G*COT_HALFT;  TERM1 = TERM0 - SIN_G;
        TERM0 = (3*TERM0*TERM1+1.)/(TERM1*TERM1*TERM1);
        T = TERM0*(2.*R0MAG)/(3.*RV0MAG);
    end
    % Bracket update.  T rises with GAMMA, so a T that is too LONG lowers the
    % ceiling and a T that is too SHORT raises the floor.
    if T > TF && GAMMA < G_MAX,  G_MAX = GAMMA;               end
    if T < 0.  && GAMMA < G_MAX,  G_MAX = GAMMA;              end
    if T < TF && GAMMA > G_MIN,  G_MIN = GAMMA;  T_MIN = T;   end
    if T < 0.
        NEXT = (G_MIN+G_MAX)/2.;  GOLD = G_MIN;  TOLD = T_MIN;
    else
        NEXT = GAMMA + (TF-T)*(GAMMA-GOLD)/(T-TOLD);   % secant
        if NEXT >= G_MAX
            NEXT = (GAMMA+G_MAX)/2.;                   % ...clamped to bisection
        elseif NEXT <= G_MIN
            NEXT = (GAMMA+G_MIN)/2.;
        end
        GOLD = GAMMA;  TOLD = T;
    end
    GAMMA = NEXT;  ITERS = ITERS + 1;
    if ITERS > 100
        break
    end
end
RESID = abs(TF-T)/abs(TF);

if SWITCH1 == 0
    ANGLE = HALFPI - GAMMA;
else
    ANGLE = GAMMA - HALFPI;
end
SINA = sin(ANGLE);  COSA = cos(ANGLE);
V1X = XT;  V1Y = YT;  V1Z = ZT;
V2X = XF;  V2Y = YF;  V2Z = ZF;
MAG1 = sqrt(V1X*V1X+V1Y*V1Y+V1Z*V1Z);
DOTMAG = V1X*V2X + V1Y*V2Y + V1Z*V2Z;
CROSSX = V1Y*V2Z - V1Z*V2Y;
CROSSY = V1Z*V2X - V1X*V2Z;
CROSSZ = V1X*V2Y - V1Y*V2X;
CROSSMAG = sqrt(CROSSX*CROSSX+CROSSY*CROSSY+CROSSZ*CROSSZ);
C2 = MAG1*SINA/CROSSMAG;
C1 = COSA/MAG1 - DOTMAG*C2/(MAG1*MAG1);
VRX = RV0MAG*(C2*V2X + C1*V1X);
VRY = RV0MAG*(C2*V2Y + C1*V1Y);
VRZ = RV0MAG*(C2*V2Z + C1*V1Z);
end


% =============================================================================
function DISTKM = distance3dkm(XT, YT, ZT, XF, YF, ZF, A, FTPKM)
%DISTANCE3DKM  Zarchan Listing 28.3, zarchan.txt:10423.  Great-circle arc at
%   the Earth radius A.  "if CBETA,=1." -> "<=": the else branch exists only
%   to catch a rounding overshoot that would turn acos() complex.
R  = sqrt(XT^2+YT^2+ZT^2);
RF = sqrt(XF^2+YF^2+ZF^2);
CBETA = (XT*XF+YT*YF+ZT*ZF)/(R*RF);
if CBETA <= 1.
    DISTKM = A*acos(CBETA)/FTPKM;
else
    DISTKM = (XF-XT)/FTPKM;
end
end


% =============================================================================
function s = render(R, ev)
L = {};
L{end+1} = 'Zarchan Listing 34.2 -- TARGET TRAJECTORY GENERATOR (boost-phase intercept)';
L{end+1} = sprintf('   %s', R.units);
L{end+1} = sprintf('   H = %g s, TS = %g s', R.p.H, R.p.TS);
L{end+1} = '';
L{end+1} = '   case   RDESKM  TLOFT  TUPT   TPZ        TF     t_bo   t_impact    apogee  downrange    t(7km)';
L{end+1} = '            [km]    [s]   [s]   [s]       [s]      [s]        [s]      [km]       [km]       [s]';
for i = 1:numel(R.cases)
    c = R.cases(i);
    L{end+1} = sprintf('   %-6s %6g %6g %5g %5g %9.2f %8.2f %10.2f %9.1f %10.1f %9.2f', ...
        c.name, c.RDESKM, c.TLOFT, c.TUPT, c.TPZ, c.TF, c.t_bo_s, ...
        c.t_impact_s, c.apogee_km, c.downrange_km, c.t7km_s); %#ok<AGROW>
    L{end+1} = sprintf('          burnout: %s;  Kepler apogee %.1f km', ...
        c.bo_reason, c.apogee_kepler_km); %#ok<AGROW>
    if c.hit_cap
        L{end+1} = '          WARNING: step cap reached -- this arc never came down'; %#ok<AGROW>
    end
end
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %s', vstr(K.oks(i)), K.names{i}); %#ok<AGROW>
    L{end+1} = sprintf('        %s', K.detail{i});                %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning';
    L{end+1} = '            NOT CHECKED, which is not the same thing as "fine".';
else
    L{end+1} = 'BENCHMARKS  (book values grepped from zarchan_decoded.txt; lines in .source)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
            case {'skip','info'}
                L{end+1} = sprintf('   %-4s %-54s %s', upper(b.kind), b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why);            %#ok<AGROW>
                L{end+1} = sprintf('        [%s]', b.source);       %#ok<AGROW>
            case 'bound'
                L{end+1} = sprintf(['   %-4s %-54s book %s %s / ours %s %s ' ...
                    '(%+.2f%%, bound %s)'], vstr(b.ok), b.what, b.book_str, ...
                    b.units, b.ours_str, b.units, b.pct, b.band); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf(['   %-4s %-54s book %s %s / ours %s %s ' ...
                    '(%+.4f%%, band %.4g%%)'], vstr(b.ok), b.what, ...
                    b.book_str, b.units, b.ours_str, b.units, b.pct, 100*b.tol); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d not adjudicated -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: a TRUTH MODEL.  Round, non-rotating Earth, point-mass gravity,';
L{end+1} = '   NO atmosphere and NO drag (the listing computes no Q -- Listing 33.1';
L{end+1} = '   does).  Lambert steering is applied to the TARGET; there is no';
L{end+1} = '   interceptor, no seeker and no filter anywhere in this file.';
s = strjoin(L, newline);
end

function s = vstr(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end

function y = maxOr(x)
if isempty(x), y = NaN; else, y = max(x); end
end

function drawTraj(R)
figure;  hold on;
for i = 1:numel(R.cases)
    plot(R.cases(i).downrange_km_hist, R.cases(i).alt_km_hist, ...
        'DisplayName', sprintf('%s, %g km, TLOFT = %g s', R.cases(i).name, ...
        R.cases(i).RDESKM, R.cases(i).TLOFT));
end
grid on;  xlabel('Downrange (km)');  ylabel('Altitude (km)');
title('Zarchan Listing 34.2 -- boosting target trajectories');
legend('Location', 'best');  hold off;
end
