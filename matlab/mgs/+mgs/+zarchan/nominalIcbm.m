function R = nominalIcbm(opts)
%MGS.ZARCHAN.NOMINALICBM  Zarchan Listing 33.1 -- TRAJECTORY GENERATOR FOR A
%   NOMINAL ICBM.  A boosting two-stage APS ICBM: 20 s straight up, an
%   open-loop gravity turn to 100 s, then closed-loop LAMBERT3D steering onto a
%   7000-km aim point.  It is the TRUTH MODEL Listing 33.2's boost-phase Kalman
%   filters track, so the pair is the point of the chapter.
%
%   R = mgs.zarchan.nominalIcbm()
%   R = mgs.zarchan.nominalIcbm('H', 0.02, 'assert', false)   coarser + faster
%   R = mgs.zarchan.nominalIcbm('deleteexp', false)           skip the 3 extra runs
%
%   Port of zarchan.txt:12628-12680.  LAMBERT3D and distance3dkm are, by the
%   listing's own trailing comments, Listing 28.3's; both are transcribed again
%   at the bottom of this file.
%
% ===========================================================================
%   UNITS ARE THE BOOK'S.  ft, ft/s, ft/s^2, lbf, slug/ft^3, lb/ft^2;
%   g = 32.2 ft/s^2; A = 2.0926e7 ft; GM = 1.4077e16 ft^3/s^2; km = 3280 ft
%   (NOT 3280.84); deg = 57.3*rad (NOT 180/pi).  Nothing is SI, nothing is
%   converted, so every printed number lies straight against a published one.
%   ⚠️ DISTNM and ALTNM are named "NM" and are KILOMETRES -- both divide by
%   3280 and Fig. 33.2's own axes say "(km)".  R.dist_km / R.alt_km are named
%   for what they are.
% ===========================================================================
%
%   *** WHAT IT IS NOT. ***  Non-rotating spherical Earth, planar (Z = ZF = 0),
%   point mass, NO DRAG (the atmosphere appears only as rho for the dynamic-
%   pressure diagnostic), thrust that points anywhere instantly, no attitude
%   dynamics, no g limit.  Do not quote an impact accuracy out of it.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_33_01.m does not parse.  The OCR maps '<' to
%   ',' and '>' to '.' inside code, and every comparison falls out of that:
%     while ~(ALTNM,-1 | T.TFINISH) -> ~(ALTNM < -1 || T > TFINISH).  FORCED:
%        ALTNM starts at 0, so '>' would exit before the first step.  Listing
%        28.2 writes the same stop as "while ALTNM > -1", which agrees.
%     if (QBOOST==1 & T.TGRAVEND)   -> T > TGRAVEND.  <<< THE UNRESOLVED ONE,
%        resolved as "Lambert AFTER the gravity turn" and forced two ways:
%        :12604 says the turn runs to TGRAVEND = 100 s with closed-loop
%        guidance following it, and the elseif chain below already owns
%        T <= TGRAVEND, so '<' makes three of the four branches unreachable.
%     if (T,240 & DEL.500.)         -> T < 240 && DEL > 500.  FORCED: TRST is
%        zero past 240 s, and the elseif is the DEL < 500 case.
%     if (ELDEG.2. & ELDEG,85)      -> ELDEG > 2 && ELDEG < 85 -- which is
%        exactly the 2 deg acquisition floor :12691 states in prose.
%   Two blocks of prose leaked in: the Fig. 33.2 axis labels
%   (listing_33_01.m:11-20) and the whole Fig. 33.3 paragraph (:50-65).
%   Dropped, not "fixed" into code -- and the second is where two benchmarks
%   came from, which is how it was identified.
%
%   Book quirks kept deliberately, and the two deviations:
%     * "TF = 2000." on line 1 is DEAD -- line 2 overwrites it with
%       TF = 252 + .223*RDESKM - 5.44e-6*RDESKM^2 + TLOFT = 2046.44 s.  So TF
%       is not exposed as an option; RDESKM and TLOFT are.
%     * LEFT, QOOMPH, QFINISH, QZERO, CW, XLONGM, ACCG, VMAG are assigned and
%       never read.  Kept and marked, so the printed page still diffs cleanly.
%     * rho uses ALT from the SECOND derivative pass (Euler-predicted) while
%       ALTNM beside it is the CORRECTED altitude.  Kept; ~1e-4 ft at H = .01.
%     * ⚠️ DEVIATION 1: both acos() arguments are CLAMPED to [-1,1].  The book
%       clamps neither, and at t = 1 s the velocity is radial to within 1e-8,
%       so the GAMDEG ratio rounds above 1 and acos returns a COMPLEX number
%       that then propagates through the record arrays without an error.
%       Clamping cannot mask a real fault: the arguments are cosines.
%     * ⚠️ DEVIATION 2: a runaway guard on the record count.  The listing's
%       only exits are ALTNM < -1 and T > 999999; a vehicle that reached orbit
%       would run 1e8 steps.  See the comment at the guard.
%     * ⚠️ THE SNAP.  When the Lambert velocity error DEL first falls below
%       500 ft/s the listing sets X1 = VRX, Y1 = VRY -- it does not FLY the
%       last 500 ft/s, it teleports the velocity onto the Lambert solution and
%       shuts the motor down.  That is the book's closure cheat, and it is why
%       the impact rows can be scored tightly.  SELF-CHECK 5 deletes it.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Nine scored rows; every book value was grepped out of
%   zarchan_decoded.txt and its line, band and band PROVENANCE are in the row
%   itself (.source, .band, .detail).  Nothing was read off a figure -- the
%   extraction turned every figure into an image link -- but prose CLAIMS about
%   figures are scored, as values or as orderings.  The five 10% bands are all
%   the same claim: the book writes "approximately 1800 km", "approximately
%   6 g", "the first 90 s", "until 65 s", which are rounded readings of its own
%   output.  Rows 2 and 3 are closed forms of the listing's own WGT/TRST lines
%   (725850/126020 = 5.760 g, 182250/14040 = 12.98 g, so -4.0% and -0.2%) and
%   are the only two that survive off the book case.
%
%   CROSS-CHECK AGAINST MGS.ZARCHAN.ICBMTRAJECTORY (Listing 28.2).  The two
%   ICBM generators AGREE on the physics wherever they overlap: identical A and
%   GM literals, identical two-body law -GM*r/|r|^3, identical Zarchan two-pass
%   Heun, identical LAMBERT3D (icbmTrajectory.m:309 -- 33.1's own trailing
%   comment points at the same Listing 28.3), identical distance3dkm, identical
%   57.3 and 3280.  They differ in SCOPE, not in physics: 28.2 is IMPULSIVE
%   (one Lambert solve, then coast) on a ROTATING Earth in 3-D; 33.1 is POWERED
%   (staging, gravity turn, Lambert closed at 100 Hz) on a NON-ROTATING Earth
%   in 2-D.  SELF-CHECKS 1-2 are the runtime form of "same gravity model".
%   GM/A^2 = 32.147 ft/s^2 against the 32.2 both listings use for thrust -- a
%   0.16% inconsistency the BOOK carries, not this port.
%
% ---------------------------------------------------------------------------
%   ⚠️ NEVER EXECUTED BY ITS AUTHOR -- MATLAB is the caller's.  Every
%   expectation here is DERIVED, not measured.  What it SHOULD print: apogee
%   near 1800 km, impact near 7000 km at ~2046 s, boost ending (DEL < 500)
%   shortly before 240 s, ~2050 records at samp = .9999.  A red row is a
%   RESULT; do not widen a band to clear it.
%
%   OUTPUTS
%     R.t .dist_km .alt_km .q .isee .gamma_deg .at .axt .ayt .vel .x .y .xd .yd
%     R.apogee_km .t_apogee .t_impact .dist_impact_km .t_isee .t_q600 .q_100
%     R.tf .boost_end_t .lambert .aim .delete_pitchover .delete_snap
%     R.bench .selfcheck .pass .verdict .p .units .summary
%
%   See also MGS.ZARCHAN.ICBMTRAJECTORY (28.2), MGS.ZARCHAN.BOOSTERLAMBERT
%   (13.3, which 33.1 is "based on"), MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 33.1 lines 1-2, verbatim (zarchan.txt:12628).
    opts.RDESKM    (1,1) double {mustBeFinite, mustBePositive} = 7000.  % km
    opts.RDESRKM   (1,1) double {mustBeFinite, mustBePositive} = 560.   % km, radar
    opts.TLOFT     (1,1) double {mustBeFinite} = 500.                   % s, loft
    opts.TGRAVEND  (1,1) double {mustBeFinite, mustBePositive} = 100.   % s
    opts.TUPT      (1,1) double {mustBeFinite, mustBePositive} = 20.    % s
    opts.GAMDEGIC  (1,1) double {mustBeFinite} = 89.8                   % deg
    opts.H         (1,1) double {mustBeFinite, mustBePositive} = .01    % s
    % The listing's own "if S >= .9999" -- one record per second at H = .01.
    opts.samp      (1,1) double {mustBeFinite, mustBePositive} = .9999  % s
    % SELF-CHECK 5 reruns the whole flight with the velocity snap deleted.
    % Coarse on purpose: the effect it measures is ~2.7e5 m of impact error
    % against an H-sensitivity of metres, so 5*H buys the check for a fifth of
    % the cost without touching what it can conclude.
    opts.H_delete  (1,1) double {mustBeFinite, mustBePositive} = .05    % s
    opts.deleteexp (1,1) logical = true
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

% ---- the book's world constants.  Bundled into a struct because MATLAB local
%      functions cannot see this workspace and an 18-argument signature is how
%      a constant quietly gets passed in the wrong slot.
C = struct( ...
    'A',        2.0926E7, ...      % ft, Earth radius
    'GM',       1.4077E16, ...     % ft^3/s^2
    'G',        32.2, ...          % ft/s^2, the thrust/weight g
    'FT_KM',    3280., ...         % ft per km, the book's value
    'DEGRAD',   57.3, ...          % the book's deg per rad
    'TFINISH',  999999., ...       % s
    'ELMIN',    2., ...            % deg, radar acquisition floor (:12691)
    'ELMAX',    85., ...           % deg
    'RDESKM',   opts.RDESKM, 'RDESRKM', opts.RDESRKM, 'TLOFT', opts.TLOFT, ...
    'TGRAVEND', opts.TGRAVEND, 'TUPT', opts.TUPT, 'GAMDEGIC', opts.GAMDEGIC, ...
    'H', opts.H, 'samp', opts.samp, 'tStop', Inf, ...
    'doPitchover', true, 'doSnap', true);

if opts.samp < opts.H
    error('mgs:zarchan:nominalIcbm:samp', ...
        ['samp = %g s is below the step H = %g s, so the record would be ' ...
         'taken every step while its time vector claimed the coarser rate.'], ...
        opts.samp, opts.H);
end

% =========================================================================
%   LISTING 33.1
% =========================================================================
P = flyIcbm(C);

R = struct();
R.t = P.T;  R.dist_km = P.DISTNM;  R.alt_km = P.ALTNM;  R.q = P.Q;
R.isee = P.ISEE;  R.gamma_deg = P.GAMDEG;  R.at = P.AT;
R.axt = P.AXT;  R.ayt = P.AYT;  R.vel = P.VEL;
R.x = P.X;  R.y = P.Y;  R.xd = P.XD;  R.yd = P.YD;
R.tf = P.TF;  R.boost_end_t = P.boost_end_t;  R.lambert = P.lambert;
R.aim = [P.XF; P.YF];

% ---- the five published readings, all taken off the recorded arrays --------
[R.apogee_km, iap] = max(P.ALTNM);
R.t_apogee      = P.T(iap);
[R.t_impact, R.dist_impact_km] = impactCross(P.T, P.ALTNM, P.DISTNM);
R.t_isee        = firstTrue(P.T, P.ISEE == 1);
R.t_q600        = fallBelow(P.T, P.Q, 600.);
R.q_100         = atTime(P.T, P.Q, 100.);

% ---- the two delete experiments -------------------------------------------
R.delete_pitchover = [];
R.delete_snap      = [];
if opts.deleteexp
    Cp = C;  Cp.doPitchover = false;  Cp.tStop = C.TGRAVEND;
    Cb = C;  Cb.tStop = C.TGRAVEND;      % the SAME horizon, pitchover intact
    Dp = flyIcbm(Cp);  Db = flyIcbm(Cb);
    R.delete_pitchover = struct( ...
        'phi_with_deg', 90 - lastOr(Db.GAMDEG), ...
        'phi_without_deg', 90 - lastOr(Dp.GAMDEG), ...
        't', lastOr(Db.T));

    Cs = C;  Cs.doSnap = false;  Cs.H = opts.H_delete;
    Ds = flyIcbm(Cs);
    [tImpS, dImpS] = impactCross(Ds.T, Ds.ALTNM, Ds.DISTNM);
    R.delete_snap = struct('t_impact', tImpS, 'dist_km', dImpS, ...
        'err_km', abs(dImpS - C.RDESKM), 'H', opts.H_delete);
end

R.units = ['ft, ft/s, ft/s^2, lbf, slug/ft^3, lb/ft^2; g = 32.2 ft/s^2; ' ...
           'km = 3280 ft; deg = 57.3*rad'];
R.p = struct('RDESKM',opts.RDESKM, 'RDESRKM',opts.RDESRKM, 'TLOFT',opts.TLOFT, ...
    'TGRAVEND',opts.TGRAVEND, 'TUPT',opts.TUPT, 'GAMDEGIC',opts.GAMDEGIC, ...
    'H',opts.H, 'samp',opts.samp, 'H_delete',opts.H_delete, 'TF',P.TF, ...
    'n_samples',numel(P.T), 'n_lambert',P.lambert.calls, 'units',R.units);

R.selfcheck = structuralChecks(R, P, C, opts);

bookCase = isBookCase(opts);
if opts.benchmark
    R.bench = benchmarks(R, C, bookCase);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  "Not checked" must never render as "fine".
    R.pass  = false;
end

R.summary = render(R, ev, bookCase);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R); end

% ---- adjudication.  The verdict is FORMED either way; 'assert' chooses only
%      the reaction.  benchSuite runs every port with assert off so that one
%      red row cannot withhold the evidence that explains it.
msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false, so nothing was compared against the book.  R.pass is false meaning NOT CHECKED.';
elseif ~any(ev)
    msg = 'not one book value could be evaluated -- see the SKIP rows for which parameter moved off the book case.';
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question.  ' ...
        'First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf('%d of %d evaluated book values are outside tolerance (first: %s, book %s, ours %s, band %s).', ...
        numel(bad), sum(ev), R.bench(bad(1)).what, R.bench(bad(1)).book_str, ...
        R.bench(bad(1)).ours_str, R.bench(bad(1)).band);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:nominalIcbm:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:nominalIcbm:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function P = flyIcbm(C)
%FLYICBM  Listing 33.1, zarchan.txt:12628-12680, transcribed line for line.
%
%   C carries every constant.  C.doPitchover / C.doSnap are the two DELETE
%   SWITCHES (both true = the book); C.tStop is a horizon used only by the
%   pitchover experiment.  Nothing here reads a caller's workspace.

A = C.A;  GM = C.GM;  G = C.G;  FT_KM = C.FT_KM;  DEGRAD = C.DEGRAD;
H = C.H;  TGRAVEND = C.TGRAVEND;  TUPT = C.TUPT;  GAMDEGIC = C.GAMDEGIC;

% ---- the listing's line 1-2, verbatim -------------------------------------
count = 0;
LEFT = 1; QOOMPH = 1; QFINISH = 1; QZERO = 0; CW = 0;      %#ok<NASGU> dead in
SWITCH = 0; QBOOST = 1; QFIRST = 1;                        %      the listing
if ~C.doPitchover, QFIRST = 0; end   % <<< DELETE SWITCH: no pitchover branch
GAMDEG = GAMDEGIC;  T = 0.;  S = 0.;
ALTNM = 0.;  ALT = ALTNM*6076.;  ANGDEG = 0.;  ANG = ANGDEG/DEGRAD;
XLONGM = ANG;                                              %#ok<NASGU> dead
X = (A+ALT)*cos(ANG);  Y = (A+ALT)*sin(ANG);
ALT = sqrt(X^2+Y^2)-A;
XFIRST = X;  YFIRST = Y;
% Magnitude 1 ft/s, NOT zero: the 0.2 deg tilt in GAMDEGIC is the only lateral
% information the vehicle has before the pitchover at TUPT.
X1 = cos(1.5708-GAMDEG/DEGRAD+ANG);
Y1 = sin(1.5708-GAMDEG/DEGRAD+ANG);
AXT = 0.;  AYT = 0.;
XLONGTDEG = DEGRAD*C.RDESKM*FT_KM/A;
XLONGRDEG = DEGRAD*C.RDESRKM*FT_KM/A;
% "TF = 2000." on line 1 is dead -- this is the line that decides the flight.
TF = 252.+.223*C.RDESKM-(5.44E-6)*C.RDESKM*C.RDESKM;
TF = TF + C.TLOFT;
XLONGT = XLONGTDEG/DEGRAD;  XLONGR = XLONGRDEG/DEGRAD;
XF = A*cos(XLONGT);  YF = A*sin(XLONGT);
XR = A*cos(XLONGR);  YR = A*sin(XLONGR);
Z = 0;  ZF = 0;  ZFIRST = 0;   %#ok<NASGU> the listing assigns Z/ZF twice;
                               %   the loop re-assigns them before either is read

XD = X1;  YD = Y1;  X1D = 0.;  Y1D = 0.;  VEL = sqrt(XD^2+YD^2);
AT = 0.;  Q = 0.;  ISEE = 0;  DEL = NaN;   %#ok<NASGU> defensive, not the book's:
%   the record block sets Q and ISEE before anything reads them, but a caller
%   who lowers tStop below the first record must not get an undefined variable.
boost_end_t = NaN;  lamCalls = 0;  lamMaxIter = 0;  lamConv = true;

nmax = ceil(min(C.tStop, 4*TF)/C.samp) + 8;
aT = zeros(nmax,1); aDIST = aT; aALT = aT; aQ = aT; aISEE = aT;
aGAM = aT; aAT = aT; aAXT = aT; aAYT = aT; aVEL = aT;
aX = aT; aY = aT; aXD = aT; aYD = aT;

while ~(ALTNM < -1 || T > C.TFINISH)
    if T >= C.tStop, break; end
    XOLD = X;  YOLD = Y;  X1OLD = X1;  Y1OLD = Y1;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  T advances INSIDE it, exactly once,
    % which is what makes the second evaluation happen at t+H.  AXT/AYT are
    % HELD across both passes -- the guidance block below runs once per outer
    % step, after the corrector, exactly as printed.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            X = X+H*XD;  Y = Y+H*YD;  X1 = X1+H*X1D;  Y1 = Y1+H*Y1D;  T = T+H;
        end
        if T < 120.
            WGT = -2622*T+440660.;  TRST = 725850.;    % stage 1
        elseif T < 240.
            WGT = -642.*T+168120.;  TRST = 182250.;    % stage 2
        else
            WGT = 5500.;  TRST = 0.;                   % payload, coast
        end
        AT = G*TRST/WGT;
        XD = X1;  YD = Y1;  VEL = sqrt(XD^2+YD^2);
        TEMBOT = (X^2+Y^2)^1.5;
        X1D = -GM*X/TEMBOT+AXT;  Y1D = -GM*Y/TEMBOT+AYT;
        ALT = sqrt(X^2+Y^2)-A;
        ACCG = sqrt(AXT^2+AYT^2)/G;                    %#ok<NASGU> dead
        FLAG = 1;
    end
    % The listing writes "FLAG = 0" here too; the outer loop re-initialises it,
    % so the assignment cannot be read.  Recorded, not reproduced.

    X  = (XOLD+X)/2+.5*H*XD;    Y  = (YOLD+Y)/2+.5*H*YD;
    X1 = (X1OLD+X1)/2+.5*H*X1D; Y1 = (Y1OLD+Y1)/2+.5*H*Y1D;
    S = S+H;  Z = 0.;  ZF = 0.;

    TGOLAM = TF-T;                                     %#ok<NASGU> re-set below
    if (QBOOST == 1 && T > TGRAVEND)
        TGOLAM = TF-T;
        [VRX, VRY, ~, lam] = lambert3d(X, Y, Z, TGOLAM, XF, YF, ZF, SWITCH, GM);
        lamCalls = lamCalls+1;
        lamMaxIter = max(lamMaxIter, lam.iters);
        lamConv = lamConv && lam.converged;
        DELX = VRX-X1;  DELY = VRY-Y1;  DEL = sqrt(DELX^2+DELY^2);
        if (T < 240 && DEL > 500.)
            AXT = AT*DELX/DEL;  AYT = AT*DELY/DEL;
        elseif DEL < 500.
            TRST = 0.;  QBOOST = 0;  AXT = 0.;  AYT = 0.;   %#ok<NASGU> TRST
            if C.doSnap
                % <<< DELETE SWITCH.  THE SNAP: the last <500 ft/s of the
                % Lambert solution is teleported on rather than flown.
                X1 = VRX;  Y1 = VRY;  X1OLD = X1;  Y1OLD = Y1;  %#ok<NASGU>
            end
            boost_end_t = T;
        else
            QBOOST = 0;  AXT = 0.;  AYT = 0.;
            boost_end_t = T;
        end
    elseif (T >= TUPT && T <= TGRAVEND && QFIRST == 1)
        % THE PITCHOVER.  Re-points the velocity at GAMDEGIC, which is the only
        % thing that seeds the gravity turn -- straight-up thrust is radial, so
        % without it the flight-path angle stays at 90 deg.  SELF-CHECK 4.
        QFIRST = 0;  VEL = sqrt(XD^2+YD^2);
        X1 = VEL*cos(1.5708-GAMDEGIC/DEGRAD+ANG);
        Y1 = VEL*sin(1.5708-GAMDEGIC/DEGRAD+ANG);
        X1OLD = X1;  Y1OLD = Y1;                       %#ok<NASGU>
        AXT = AT*X1/VEL;  AYT = AT*Y1/VEL;
    elseif (T >= TUPT && T <= TGRAVEND)
        VEL = sqrt(XD^2+YD^2);
        AXT = AT*X1/VEL;  AYT = AT*Y1/VEL;             % gravity turn
    elseif T <= TUPT
        RTMAG = sqrt(X^2+Y^2);
        AXT = AT*X/RTMAG;  AYT = AT*Y/RTMAG;           % straight up (radial)
    end
    % No branch fires once QBOOST is 0 and T > TGRAVEND, so AXT/AYT keep the
    % zeros the exit branch set: the vehicle coasts.  That is the listing's.

    if S >= C.samp
        S = 0.;
        DISTNM = distance3dKm(X, Y, Z, XFIRST, YFIRST, ZFIRST, A, FT_KM);
        ALTNM  = (sqrt(X^2+Y^2)-A)/FT_KM;
        RMAG = sqrt(X^2+Y^2);
        VMAG = sqrt(XD^2+YD^2);      % dead in the listing; equals VEL exactly
        % clamp: see the DEVIATION note in the header.  Without it acos()
        % returns COMPLEX at the first sample and never says so.
        GAMDEG = 90-DEGRAD*acos(clamp1((X*XD+Y*YD)/(RMAG*VMAG)));
        RHO = .0034*exp(-ALT/22000.);
        Q = .5*RHO*VEL*VEL;
        RRMAG  = sqrt(XR^2+YR^2);
        RRTMAG = sqrt((X-XR)^2+(Y-YR)^2);
        ELDEG  = 90.-DEGRAD*acos(clamp1((XR*(X-XR)+YR*(Y-YR))/(RRMAG*RRTMAG)));
        if (ELDEG > C.ELMIN && ELDEG < C.ELMAX)
            ISEE = 1;
        else
            ISEE = 0;
        end
        count = count+1;
        aT(count) = T;  aDIST(count) = DISTNM;  aALT(count) = ALTNM;
        aQ(count) = Q;  aISEE(count) = ISEE;  aGAM(count) = GAMDEG;
        aAT(count) = AT;  aAXT(count) = AXT;  aAYT(count) = AYT;
        aVEL(count) = VEL;  aX(count) = X;  aY(count) = Y;
        aXD(count) = XD;  aYD(count) = YD;
        % RUNAWAY GUARD, not in the book.  The listing's only exits are
        % ALTNM < -1 and T > 999999; a vehicle that reaches orbit would run
        % 1e8 steps and grow the record arrays one element at a time.  nmax is
        % four flight times' worth of records, so hitting it means the flight
        % did not come down and R.t_impact comes back NaN -- which fails the
        % impact rows loudly instead of hanging.
        if count >= nmax, break; end
    end
end

P = struct('T',aT(1:count), 'DISTNM',aDIST(1:count), 'ALTNM',aALT(1:count), ...
    'Q',aQ(1:count), 'ISEE',aISEE(1:count), 'GAMDEG',aGAM(1:count), ...
    'AT',aAT(1:count), 'AXT',aAXT(1:count), 'AYT',aAYT(1:count), ...
    'VEL',aVEL(1:count), 'X',aX(1:count), 'Y',aY(1:count), ...
    'XD',aXD(1:count), 'YD',aYD(1:count), 'TF',TF, 'XF',XF, 'YF',YF, ...
    'XR',XR, 'YR',YR, 'boost_end_t',boost_end_t, 'del_at_end',DEL, ...
    'lambert',struct('calls',lamCalls,'max_iters',lamMaxIter,'converged',lamConv));
end


% =============================================================================
function [VRX, VRY, VRZ, lam] = lambert3d(XT, YT, ZT, TF, XF, YF, ZF, SWITCH1, GM)
%LAMBERT3D  zarchan.txt:10428-10456 (Listing 28.3), which Listing 33.1's own
%   trailing comment points at.  Transcribed line for line, and IDENTICAL to
%   the copy inside mgs.zarchan.icbmTrajectory (icbmTrajectory.m:309) -- where
%   its BENCHMARKS 1-3 score its launch velocity against the book's fifteen
%   published digits.  It is therefore NOT re-benchmarked here.
%
%   A secant / false-position hybrid on the launch flight-path angle GAMMA.
%   Flight time DECREASES with GAMMA over [G_MIN, G_MAX], which is what fixes
%   the direction of the three bracket tests; that resolution is the one the
%   28.2 port confirmed arithmetically against the published velocity.
%   With ZT = ZF = 0 the whole solve is planar and VRZ comes back exactly 0.
%
%   The book's S/BL/BT and its four unassigned unit constants are dropped here
%   rather than carried with %#ok: they are read by nothing, and the copy in
%   icbmTrajectory.m keeps them so the printed page still diffs cleanly there.
%   The two acos() arguments are CLAMPED for the reason given in the header.
HALFPI = pi/2.;
RF0X = XF-XT;  RF0Y = YF-YT;  RF0Z = ZF-ZT;
R0DOTRF  = XT*XF + YT*YF + ZT*ZF;
R0DOTRF0 = XT*RF0X + YT*RF0Y + ZT*RF0Z;
R0MAG  = sqrt(XT^2+YT^2+ZT^2);
RFMAG  = sqrt(XF^2+YF^2+ZF^2);
RF0MAG = sqrt(RF0X^2+RF0Y^2+RF0Z^2);
RATIO   = R0MAG/RFMAG;
GMDIVR0 = GM/R0MAG;
COS_T   = R0DOTRF/(R0MAG*RFMAG);
VNUMER  = GMDIVR0*(1.-COS_T);
T_MIN   = 0.;
if SWITCH1 == 0
    G_MIN = HALFPI - acos(clamp1(R0DOTRF0/(R0MAG*RF0MAG)));
    G_MAX = HALFPI;
    THETA = acos(clamp1(COS_T));
else
    G_MIN = -HALFPI;
    G_MAX = -HALFPI + acos(clamp1(R0DOTRF0/(R0MAG*RF0MAG)));
    THETA = 2.*pi - acos(clamp1(COS_T));
end
SIN_T     = sin(THETA);
COT_HALFT = 1./tan(THETA/2.);
GAMMA = (G_MAX+G_MIN)/2.;
GOLD  = G_MIN;  TOLD = 0.;  T = 0.;  ITERS = 1;
RV0MAG = NaN;

while (abs(TF-T) > (.00000001*TF))
    SIN_G = sin(GAMMA);  COS_G = cos(GAMMA);  TAN_G = SIN_G/COS_G;
    COS_TPLUSG = cos(THETA+GAMMA-2.*pi);
    TERM1  = (RATIO*COS_G-COS_TPLUSG)*COS_G;
    RV0MAG = sqrt(VNUMER/TERM1);
    LAMBDA = RV0MAG*RV0MAG/GMDIVR0;
    if LAMBDA < 1.9999999                                   % elliptic
        TERM0 = sqrt(2./LAMBDA-1.);
        TERM1 = (TAN_G*(1.-COS_T)+(1.-LAMBDA)*SIN_T)/((2.-LAMBDA)*RATIO);
        TERM2 = (COS_G+COS_G)/(LAMBDA*TERM0*TERM0*TERM0);
        TERM3 = atan2(TERM0, (COS_G*COT_HALFT-SIN_G));
        T = (R0MAG/(RV0MAG*COS_G))*(TERM1+TERM2*TERM3);
    elseif LAMBDA > 2.0000001                               % hyperbolic
        TERM0 = sqrt(1.-2./LAMBDA);
        TERM1 = (TAN_G*(1.-COS_T)+(1.-LAMBDA)*SIN_T)/((2.-LAMBDA)*RATIO);
        TERM2 = COS_G/(LAMBDA*TERM0*TERM0*TERM0);
        TERM3 = SIN_G-COS_G*COT_HALFT;
        TERM3 = log((TERM3-TERM0)/(TERM3+TERM0));
        T = (R0MAG/(RV0MAG*COS_G))*(TERM1-TERM2*TERM3);
    else                                                    % parabolic limit
        TERM0 = COS_G*COT_HALFT;
        TERM1 = TERM0-SIN_G;
        TERM0 = (3*TERM0*TERM1+1.)/(TERM1*TERM1*TERM1);
        T = TERM0*(2.*R0MAG)/(3.*RV0MAG);
    end
    if T > TF && GAMMA < G_MAX, G_MAX = GAMMA;  end
    if T < 0.  && GAMMA < G_MAX, G_MAX = GAMMA;  end
    if T < TF  && GAMMA > G_MIN, G_MIN = GAMMA;  T_MIN = T;  end
    if (T < 0.)
        NEXT = (G_MIN+G_MAX)/2.;  GOLD = G_MIN;  TOLD = T_MIN;
    else
        NEXT = GAMMA+(TF-T)*(GAMMA-GOLD)/(T-TOLD);
        if NEXT >= G_MAX
            NEXT = (GAMMA+G_MAX)/2.;
        elseif NEXT <= G_MIN
            NEXT = (GAMMA+G_MIN)/2.;
        end
        GOLD = GAMMA;  TOLD = T;
    end
    GAMMA = NEXT;
    ITERS = ITERS+1;
    if ITERS > 100, break; end
end

if SWITCH1 == 0
    ANGLE = HALFPI-GAMMA;
else
    ANGLE = GAMMA-HALFPI;
end
SINA = sin(ANGLE);  COSA = cos(ANGLE);
MAG1   = sqrt(XT*XT+YT*YT+ZT*ZT);
DOTMAG = XT*XF+YT*YF+ZT*ZF;
CROSSX = YT*ZF-ZT*YF;
CROSSY = ZT*XF-XT*ZF;
CROSSZ = XT*YF-YT*XF;
CROSSMAG = sqrt(CROSSX*CROSSX+CROSSY*CROSSY+CROSSZ*CROSSZ);
C2 = MAG1*SINA/CROSSMAG;
C1 = COSA/MAG1-DOTMAG*C2/(MAG1*MAG1);
VRX = RV0MAG*(C2*XF + C1*XT);
VRY = RV0MAG*(C2*YF + C1*YT);
VRZ = RV0MAG*(C2*ZF + C1*ZT);
lam = struct('gamma_deg', 57.3*GAMMA, 'iters', ITERS, 'tf_err', TF-T, ...
             'converged', ITERS <= 100, 'v0mag', RV0MAG);
end


% =============================================================================
function DISTKM = distance3dKm(XT, YT, ZT, XF, YF, ZF, A, FT_KM)
%DISTANCE3DKM  zarchan.txt:10423 (Listing 28.3), verbatim.  Great-circle arc
%   from the launch point, in km (A*beta/3280).  Here the book's own axis label
%   agrees that it is km.
R  = sqrt(XT^2+YT^2+ZT^2);
RF = sqrt(XF^2+YF^2+ZF^2);
CBETA = (XT*XF+YT*YF+ZT*ZF)/(R*RF);
if CBETA <= 1.
    DISTKM = A*acos(clamp1(CBETA))/FT_KM;
else
    DISTKM = (XF-XT)/FT_KM;      % the book's numerical-overshoot fallback
end
end


% =============================================================================
function K = structuralChecks(R, P, C, opts)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan -- they are the weaker
%   claim that it agrees with itself, which is what catches wiring faults.
%
%   ⚠️ .names AND .oks ARE THE CONTRACT.  benchSuite only puts a structural row
%   on the board when it finds BOTH, and a delete experiment nobody can see is
%   the same as a delete experiment that never ran.
K = struct();
names = {};  oks = [];  detail = {};
K.pitchover_ratio = NaN;  K.snap_err_km = NaN;

% Coast arc only: energy and angular momentum are two-body invariants, and
% thrust is precisely what breaks them.  Start one sample after burnout.
if isfinite(P.boost_end_t)
    co = R.t > P.boost_end_t + 2*opts.samp;
else
    co = false(size(R.t));
end
rr = sqrt(R.x.^2 + R.y.^2);
vv = R.xd.^2 + R.yd.^2;

% 1  Specific orbital energy conserved on the coast.  Catches a wrong GM, a
%    sign error on the gravity term, or a corrector gone plain Euler.
if nnz(co) > 3
    en = 0.5*vv(co) - C.GM./rr(co);
    K.energy_drift = max(abs(en - en(1)))/abs(en(1));
else
    K.energy_drift = Inf;
end
names{end+1} = 'specific orbital energy conserved on the coast arc';
oks(end+1)   = K.energy_drift <= 1e-6;
detail{end+1} = sprintf('max relative drift %.3e over %d coast samples', ...
    K.energy_drift, nnz(co));

% 2  Angular momentum: gravity is central and planar, so x*yd - y*xd is fixed.
if nnz(co) > 3
    hm = R.x(co).*R.yd(co) - R.y(co).*R.xd(co);
    K.h_drift = max(abs(hm - hm(1)))/abs(hm(1));
else
    K.h_drift = Inf;
end
names{end+1} = 'angular momentum x*yd - y*xd conserved (central force)';
oks(end+1)   = K.h_drift <= 1e-6;
detail{end+1} = sprintf('max relative drift %.3e', K.h_drift);

% 3  LAMBERT CLOSURE.  The closed loop's entire job: be at the aim point at
%    t = TF.  Interpolated between the two samples that bracket TF -- the
%    nearest sample is up to samp/2 away and the vehicle covers ~10 km in that
%    time, which would swamp what is being measured.  Expected: order 10 ft.
K.close_ft = closureFt(R.t, R.x, R.y, P.TF, P.XF, P.YF);
names{end+1} = 'LAMBERT3D closes: position at TF == the aim point';
oks(end+1)   = K.close_ft <= 2000;      % ft; 9e-5 of the 7000 km arc
detail{end+1} = sprintf('|r(TF) - r_aim| = %.1f ft (%.4f km), want <= 2000 ft', ...
    K.close_ft, K.close_ft/C.FT_KM);

% 4  ⭐ DELETE EXPERIMENT: remove the QFIRST pitchover.  Straight-up thrust is
%    RADIAL, so without the pitchover the flight-path angle never leaves 90 deg
%    and the "gravity turn" turns nothing.  Scored as a RATIO because the
%    absolute angles are small either way (~0.8 deg vs ~0.002 deg at t = 100 s)
%    and a ratio does not depend on my estimate of the turn being right.
%    ⚠️ HONEST LIMIT: this moves the flight-path angle, NOT the impact point --
%    the Lambert loop from 100 s absorbs most of it.  It tests that the branch
%    is wired and load-bearing for GAMMA, and claims nothing more.
%    ⚠️ NOT RUN when 'deleteexp' is false.  The row is then OMITTED rather than
%    scored false: a fabricated red row would make the verdict say "the port
%    disagrees with itself", which would not be true.  R.selfcheck.deletes_run
%    and the summary both say so out loud, which is the honest signal.
D = R.delete_pitchover;
if ~isempty(D)
    K.pitchover_ratio = abs(D.phi_with_deg)/max(abs(D.phi_without_deg), realmin);
    names{end+1} = 'DELETE the pitchover -> the gravity turn does not start';
    oks(end+1)   = K.pitchover_ratio > 20 && abs(D.phi_without_deg) < 0.05;
    detail{end+1} = sprintf(['90-gamma at t = %.1f s: %.4f deg with the branch, ' ...
        '%.6f deg without (ratio %.0f, want > 20 and < 0.05 deg)'], ...
        D.t, D.phi_with_deg, D.phi_without_deg, K.pitchover_ratio);
end

% 5  ⭐ DELETE EXPERIMENT: remove "X1 = VRX; Y1 = VRY" -- the snap.  The
%    vehicle then coasts with the up-to-500 ft/s of Lambert velocity error it
%    never flew.  Over the ~1800 s that remain that is ~9e5 ft = 270 km of
%    downrange error, against a run that otherwise looks completely normal:
%    same arch, same apogee, same burn, no warning, no NaN.
Ds = R.delete_snap;
if ~isempty(Ds)
    K.snap_err_km = Ds.err_km;
    base = abs(R.dist_impact_km - C.RDESKM);
    names{end+1} = 'DELETE the velocity snap -> impact walks by ~100s of km';
    oks(end+1)   = Ds.err_km > 20 && Ds.err_km > 20*max(base, 1e-3);
    detail{end+1} = sprintf(['downrange error %.1f km without the snap vs %.2f km ' ...
        'with it (H = %g s for the deleted run); want > 20 km and > 20x'], ...
        Ds.err_km, base, Ds.H);
end

K.deletes_run = opts.deleteexp;
K.names  = names;
K.oks    = logical(oks(:).');
K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    i = find(~K.oks, 1, 'first');
    K.first_bad = sprintf('%s (%s)', names{i}, detail{i});
end
end


% =============================================================================
function tf = isBookCase(o)
%ISBOOKCASE  Every published number below describes ONE run.  Off it, the
%   trajectory rows SKIP rather than being re-banded onto a different flight.
tf = abs(o.RDESKM  - 7000) < 1e-9 && abs(o.RDESRKM  - 560) < 1e-9 && ...
     abs(o.TLOFT   -  500) < 1e-9 && abs(o.TGRAVEND - 100) < 1e-9 && ...
     abs(o.TUPT    -   20) < 1e-9 && abs(o.GAMDEGIC - 89.8) < 1e-9;
end


% =============================================================================
function B = benchmarks(R, C, bookCase)
%BENCHMARKS  Computed vs published.  Every band's PROVENANCE is in its own row.
%
%   The 10% bands on rows 1-5 are all the same claim: the book writes
%   "approximately 1800 km", "approximately 6 g", "the first 90 s", "until
%   65 s" -- rounded readings of its own output, not measurements to a digit.
%   10% is what "approximately" is worth here and it is not widened for
%   anything.  Rows 2 and 3 are closed forms of the listing's own WGT/TRST
%   lines and should land at -4.0% and -0.2%; if either moves, the mass or
%   thrust profile was mistyped.
B = emptyBench();
L1 = 'Fig. 33.1 prose, zarchan.txt:12589';

% ---- 2 & 3 first: they depend only on the staging profile, never on RDESKM.
% AT = 32.2*TRST/WGT ft/s^2, so AT/g is just TRST/WGT -- these are the
% listing's own WGT lines evaluated at the instant before each burnout.
w1 = -2622*120+440660.;      % lbf, stack weight at t -> 120 s  (= 126020)
w2 = -642.*240+168120.;      % lbf, stack weight at t -> 240 s  (=  14040)
B = addVal(B, 'peak axial acceleration, end of stage 1 (t -> 120 s)', ...
    6.0, 725850./w1, '10% ("approximately", read off Fig. 33.1)', 0.10, ...
    L1, 'g', sprintf('TRST/WGT = 725850/%.0f lbf; a closed form of the listing''s own stage-1 lines', w1));
B = addVal(B, 'peak axial acceleration, end of stage 2 (t -> 240 s)', ...
    13.0, 182250./w2, '10% ("approximately", read off Fig. 33.1)', 0.10, ...
    L1, 'g', sprintf('TRST/WGT = 182250/%.0f lbf', w2));

if ~bookCase
    B = addSkip(B, 'the seven trajectory rows', 'see the header', ...
        ['the run is off the book case (RDESKM 7000, RDESRKM 560, TLOFT 500, ' ...
         'TGRAVEND 100, TUPT 20, GAMDEGIC 89.8), so no published number ' ...
         'describes it.  Widening a band to absorb that would be a lie.'], ...
        'zarchan.txt:12628', '-');
    B = addInfo(B, 'physics shared with Listing 28.2', 'same A, GM, Heun, LAMBERT3D, distance3dkm', ...
        'agrees; 28.2 adds Earth rotation and 3-D, 33.1 adds thrust', ...
        'zarchan.txt:12628 vs :10409-10456', ...
        'a source-level comparison, not a measurement -- see CROSS-CHECK in the header');
    return
end

B = addVal(B, 'apogee altitude', 1800., R.apogee_km, ...
    '10% ("approximately"; Fig. 33.2 gridlines are 500 km apart)', 0.10, ...
    'Fig. 33.2 prose, zarchan.txt:12627', 'km', ...
    sprintf('max of the recorded ALTNM array, at t = %.0f s', R.t_apogee));

B = addVal(B, 'time the radar first sees the ICBM (ISEE 0 -> 1)', 90., ...
    R.t_isee, '10% (the book writes "the first 90 s")', 0.10, ...
    'Fig. 33.4 prose, zarchan.txt:12691', 's', ...
    sprintf(['radar %g km downrange, acquisition band %g-%g deg elevation; ' ...
    'ISEE is evaluated once per record so the resolution is samp = %g s'], ...
    C.RDESRKM, C.ELMIN, C.ELMAX, R.p.samp));

B = addVal(B, 'time dynamic pressure falls below 600 lb/ft^2', 65., ...
    R.t_q600, '10% (a rounded reading of Fig. 33.3)', 0.10, ...
    'zarchan.txt:12689-12690', 's', ...
    'first record after the Q peak with Q < 600, linearly interpolated');

B = addBound(B, 'dynamic pressure at t = 100 s is below 50 lb/ft^2', ...
    '< 50', sprintf('%.2f', R.q_100), R.q_100 < 50, ...
    'zarchan.txt:12690', 'lb/ft^2', ...
    'the book''s conservative start-of-guidance condition, stated as an inequality');

B = addVal(B, 'downrange at impact', C.RDESKM, R.dist_impact_km, ...
    '1% (closure of the closed-loop Lambert steering)', 0.01, ...
    'zarchan.txt:12604 (7000-km trajectory), listing line 1 :12628', 'km', ...
    'ALTNM = 0 crossing, interpolated between the two records that bracket it');

B = addVal(B, 'impact time', R.tf, R.t_impact, ...
    '0.2% (~4 s; the record spacing alone is 1 s)', 0.002, ...
    'listing line 2, zarchan.txt:12628 (TF = 252 + .223*RDESKM - 5.44e-6*RDESKM^2, + TLOFT)', 's', ...
    sprintf(['TF = %.2f s.  CLOSURE, not an independent measurement: after the ' ...
    'snap the vehicle flies the Lambert conic exactly, so arriving at TF is ' ...
    'what the routine promised, and the row scores whether it kept the promise'], R.tf));

iUp = R.t <= R.p.TUPT;
if any(iUp)
    frac = max(abs(R.ayt(iUp)))/max(max(R.at(iUp)), realmin);
else
    frac = Inf;
end
B = addShape(B, 'downrange acceleration is zero for the first 20 s', ...
    'zero (the ICBM is going straight up)', sprintf('%.2e of AT', frac), ...
    frac < 0.01, 'Fig. 33.5 prose, zarchan.txt:12692', ...
    sprintf(['max |AYT|/AT over the %d records with t <= TUPT.  Thrust is ' ...
    'radial there (AXT = AT*X/RTMAG), so the only lateral term is the ' ...
    '%.1f deg tilt in GAMDEGIC; expect ~1e-9 before the pitchover and ' ...
    '~3.6e-3 at the pitchover record itself'], nnz(iUp), 90-R.p.GAMDEGIC));

B = addInfo(B, 'physics shared with Listing 28.2', ...
    'same A, GM, two-body law, Heun, LAMBERT3D, distance3dkm', ...
    'agrees; 28.2 adds Earth rotation and 3-D, 33.1 adds thrust and drops both', ...
    'zarchan.txt:12628 vs :10409-10456', ...
    'a source-level comparison, not a measurement -- SELF-CHECKS 1-2 are its runtime form');
end


% =============================================================================
%   Row constructors.  ONE constructor per kind so the field ORDER cannot
%   drift -- benchSuite's adaptGeneric is strict and a near-miss schema errors.
function B = emptyBench()
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.10g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.10g', ours);
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
%   Small helpers.  All constants arrive as arguments.
function y = clamp1(x)
%CLAMP1  Cosine arguments into [-1,1].  See the DEVIATION note in the header:
%   the book clamps neither acos and MATLAB returns a COMPLEX angle silently.
y = min(1, max(-1, x));
end

function [tImp, dImp] = impactCross(T, ALT, DIST)
%IMPACTCROSS  Linear interpolation to the ALT = 0 crossing.  The listing stops
%   at ALT < -1 km, so the crossing is bracketed by the last two records.
tImp = NaN;  dImp = NaN;
k = find(ALT(1:end-1) >= 0 & ALT(2:end) < 0, 1, 'last');
if isempty(k), return; end
f = ALT(k)/(ALT(k) - ALT(k+1));
tImp = T(k) + f*(T(k+1) - T(k));
dImp = DIST(k) + f*(DIST(k+1) - DIST(k));
end

function t = firstTrue(T, mask)
t = NaN;
k = find(mask, 1, 'first');
if ~isempty(k), t = T(k); end
end

function t = fallBelow(T, Y, thr)
%FALLBELOW  Time Y first drops below thr AFTER its peak, interpolated.  "After
%   the peak" matters: Q starts at 0 on the pad and rises through 600 first.
t = NaN;
[~, ip] = max(Y);
k = find(Y(ip:end) < thr, 1, 'first');
if isempty(k), return; end
k = ip + k - 1;
if k == 1, t = T(1); return; end
f = (Y(k-1) - thr)/max(Y(k-1) - Y(k), realmin);
t = T(k-1) + f*(T(k) - T(k-1));
end

function y = atTime(T, Y, t0)
y = NaN;
if isempty(T), return; end
[~, k] = min(abs(T - t0));
y = Y(k);
end

function d = closureFt(T, X, Y, TF, XF, YF)
%CLOSUREFT  |r(TF) - r_aim| in ft, interpolated between the records bracketing
%   TF.  Inf if the flight ended before TF -- which is itself a failure.
d = Inf;
k = find(T(1:end-1) <= TF & T(2:end) >= TF, 1, 'first');
if isempty(k), return; end
f  = (TF - T(k))/max(T(k+1) - T(k), realmin);
xi = X(k) + f*(X(k+1) - X(k));
yi = Y(k) + f*(Y(k+1) - Y(k));
d  = sqrt((xi - XF)^2 + (yi - YF)^2);
end

function y = lastOr(x)
if isempty(x), y = NaN; else, y = x(end); end
end

function s = vstr(ok)
if ok, s = 'ok'; else, s = '**'; end
end


% =============================================================================
function s = render(R, ev, bookCase)
L = {};  p = R.p;
L{end+1} = 'Zarchan Listing 33.1 -- TRAJECTORY GENERATOR FOR NOMINAL ICBM (boost + gravity turn + closed-loop Lambert)';
L{end+1} = '*** UNITS ARE THE BOOK''S: ft, ft/s, lbf, lb/ft^2; km = 3280 ft, deg = 57.3*rad. ***';
L{end+1} = sprintf('RDESKM %g km   radar %g km   TLOFT %g s   TUPT %g s   TGRAVEND %g s   GAMDEGIC %g deg', ...
    p.RDESKM, p.RDESRKM, p.TLOFT, p.TUPT, p.TGRAVEND, p.GAMDEGIC);
L{end+1} = sprintf('H %g s   samp %g s   TF %.2f s   %d records   %d LAMBERT3D calls', ...
    p.H, p.samp, p.TF, p.n_samples, p.n_lambert);
L{end+1} = sprintf('apogee %.1f km at %.0f s   impact %.1f km at %.2f s   boost ends %.2f s   radar acquires %.0f s', ...
    R.apogee_km, R.t_apogee, R.dist_impact_km, R.t_impact, R.boost_end_t, R.t_isee);
L{end+1} = sprintf('Q < 600 lb/ft^2 at %.1f s   Q(100 s) = %.2f lb/ft^2', R.t_q600, R.q_100);
L{end+1} = '';
L{end+1} = 'BENCHMARKS (book vs ours)';
for i = 1:numel(R.bench)
    b = R.bench(i);
    switch b.kind
        case 'value'
            if b.skipped
                L{end+1} = sprintf('  --  %-52s SKIP  %s', b.what, b.why); %#ok<AGROW>
            else
                L{end+1} = sprintf('  %s  %-52s %10s %10s  %+7.2f%%  [%s]', ...
                    vstr(b.ok), b.what, b.book_str, b.ours_str, b.pct, b.band); %#ok<AGROW>
            end
        case {'shape','bound'}
            L{end+1} = sprintf('  %s  %-52s %10s %10s  (%s)', ...
                vstr(b.ok), b.what, b.book_str, b.ours_str, b.kind); %#ok<AGROW>
        otherwise
            L{end+1} = sprintf('  --  %-52s %s  [%s]', b.what, b.ours_str, b.kind); %#ok<AGROW>
    end
end
L{end+1} = '';
L{end+1} = 'STRUCTURAL SELF-CHECKS (self-consistency, NOT book values)';
for i = 1:numel(R.selfcheck.names)
    L{end+1} = sprintf('  %s  %-52s %s', vstr(R.selfcheck.oks(i)), ...
        R.selfcheck.names{i}, R.selfcheck.detail{i}); %#ok<AGROW>
end
if ~R.selfcheck.deletes_run
    L{end+1} = '  ⚠️ THE TWO DELETE EXPERIMENTS DID NOT RUN (''deleteexp'' is false).';
    L{end+1} = '     They are omitted, not passed: nothing above tests the pitchover or the snap.';
end
if ~bookCase
    L{end+1} = '';
    L{end+1} = '⚠️ OFF THE BOOK CASE -- the trajectory rows are SKIPped, not re-banded.';
end
if ~any(ev)
    L{end+1} = '⚠️ NOTHING WAS ADJUDICATED.';
end
s = strjoin(L, newline);
end


% =============================================================================
function drawCurves(R)
figure;
subplot(2,1,1);
plot(R.dist_km, R.alt_km); grid on;
xlabel('Downrange (km)'); ylabel('Altitude (km)');
title('Listing 33.1 -- Fig. 33.2, nominal 7000-km ICBM trajectory');
subplot(2,1,2);
b = R.t <= 240;
plot(R.t(b), R.q(b)); grid on;
xlabel('Time (s)'); ylabel('Dynamic Pressure (lb/ft^2)');
title('Fig. 33.3 -- boost-phase dynamic pressure');
end
