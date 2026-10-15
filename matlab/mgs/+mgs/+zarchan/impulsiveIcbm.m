function R = impulsiveIcbm(opts)
%MGS.ZARCHAN.IMPULSIVEICBM  Zarchan Listing 32.1 -- two-dimensional trajectory
%   of an IMPULSIVE ICBM, flown by a Lambert-solved initial velocity.
%
%   R = mgs.zarchan.impulsiveIcbm()
%   R = mgs.zarchan.impulsiveIcbm('TLOFT', 0)              minimum energy only
%   R = mgs.zarchan.impulsiveIcbm('H', 0.1, 'assert', false)
%
% ===========================================================================
%   *** UNITS.  CHAPTER 32 IS ONE OF THE BOOK'S METRIC CORNERS. ***
%
%   The INTERNAL state is the book's English system -- FEET and ft/s, with
%   A = 2.0926e7 ft (Earth radius) and GM = 1.4077e16 ft^3/s^2.  Every
%   REPORTED quantity is km or km/s, converted by the listing's own
%   FT_PER_KM = 3280.  Times are seconds.  Most of mgs is ft/s with g = 32.2,
%   so read the column headings before laying any number here against another
%   port's.  There is no ft->m conversion anywhere in this file.
%
%   Two conversion constants are the BOOK'S, deliberately not "corrected":
%     * 3280 ft/km, not 3280.84 -- a 0.026% offset BAKED INTO the published
%       7.19 / 7.33 / 7.23 km/s.  "Fixing" it moves us off the book.
%     * 57.3 deg/rad, which CANCELS exactly: XLONGFDEG = 57.3*RDESKM*3280/A
%       and then XLONGF = XLONGFDEG/57.3.  Changing ONE would not cancel.
% ===========================================================================
%
%   THE TRANSCRIPTION, AND WHAT WAS DONE TO IT
%
%   Source: matlab/zarchan/_listings/listing_32_01.m (zarchan.txt:12160-12171).
%   32.1 calls two routines it does not contain -- LAMBERT3D and distance3dkm,
%   both Listing 28.3 (zarchan.txt:10428 and :10423) -- so THREE listings are
%   transcribed here.  (The book calls the second distance3d in Ch.28 and
%   distance3dkm in Ch.32; same code, km either way.)  Nothing in mgs had a
%   3-D Lambert before this: lambertEfficient/lambertBruteForce are planar.
%
%   Listing 32.1's own repairs, all forced by shape:
%     1  "while ALTTKM . -1" -> "while ALTTKM > -1".  "." is this book's OCR
%        of ">".  Flying until the altitude drops BELOW -1 km is the only
%        reading that terminates.
%     2  "while STEP ,=1"    -> "while STEP <= 1".  "," is "<"; the body sets
%        STEP = 2 to leave.
%     3  "if S .=(TS-.0001)" -> "if S >= (TS - 0.0001)".  Left UNRESOLVED by
%        the extractor.  S accumulates H and is zeroed on every record, so
%        "<=" would record every single step (S = 0.01 <= 0.9999 always) and
%        the -0.0001 guard would be pointless; ">=" with an epsilon guard is
%        the standard "have we reached the sample interval" test.
%     4  The prose paragraph and Fig. 32.1 axis labels trailing the
%        transcription (its header flags three residual ligatures and a line
%        of prose in the code) are book text.  Dropped, not made executable.
%     5  RTINIT is computed and never read.  Dropped, as are LAMBERT3D's dead
%        locals (EARTH_RADIUS, DEG_PER_RAD, FT_PER_KM, RAD_PER_SEC, S, BL, BT).
%     6  "save datfil.txt output /ascii" is the book's recurring -ascii typo.
%        Nothing is written to disk here at all.
%
%   LAMBERT3D carried NINE damaged comparisons, four of them
%   VARIABLE-TO-VARIABLE.  Resolved as, and only as:
%
%        while abs(TF-T) . (1e-8*TF)  ->  > 1e-8*TF   (loop while NOT converged)
%        if LAMBDA,1.9999999          ->  <           (elliptic branch)
%        elseif LAMBDA.2.0000001      ->  >           (hyperbolic branch)
%        if T.TF  & GAMMA,G_MAX       ->  T > TF  &  GAMMA < G_MAX
%        if T,0.  & GAMMA,G_MAX       ->  T < 0   &  GAMMA < G_MAX
%        if T,TF  & GAMMA.G_MIN       ->  T < TF  &  GAMMA > G_MIN
%        if (T,0.)                    ->  T < 0
%        if NEXT.=G_MAX               ->  NEXT >= G_MAX
%        elseif NEXT,=G_MIN           ->  NEXT <= G_MIN
%
%   WHY THAT WAY.  Time of flight rises monotonically with the launch
%   flight-path angle GAMMA inside [G_MIN, G_MAX].  So a T that is TOO LONG
%   means GAMMA is too high and the CEILING comes down (G_MAX = GAMMA); a T
%   that is TOO SHORT raises the FLOOR (G_MIN = GAMMA).  The trailing
%   "& GAMMA < G_MAX" / "& GAMMA > G_MIN" guards only stop the bracket from
%   widening.  The two NEXT clamps then read "the secant extrapolated outside
%   the bracket, so bisect instead" -- the only reading under which those
%   clamps ever do anything.
%
%   This is not merely a resolution that happens to run.  SELFCHECK 5 flips
%   the two bracket updates IN THIS FILE'S OWN lambert3d and shows the bracket
%   collapsing to a point, after which the secant divides by zero.  And the
%   resolution as a whole is confirmed OUT OF BAND by BENCHMARKS 1-3: three
%   independently published velocities it has no way to hit by accident.
%
%   /!\ The book's PROSE statement of the time-of-flight fit is OCR-damaged.
%   zarchan.txt:12157 reads "tFME = 252+ 0.223DRkm  5.44106DR2 km" -- the
%   minus sign and the 1e-6 exponent are both gone (this is the book's
%   minus-sign damage class, not the decoded ligature class).  The LISTING has
%   it intact at zarchan.txt:12160: 252 + 0.223*DR - 5.44e-6*DR^2, which gives
%   1938 s at DR = 10000 km and so agrees with the figure annotation.  The
%   listing is scored; both lines are cited on the row.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED FROM AN INDEPENDENT PYTHON
%   TRANSCRIPTION OF THE SAME ARITHMETIC, NOT FROM MATLAB. ***  This file has
%   never been executed; MATLAB belongs to the caller.  No runtime is claimed.
%
%     TLOFT     TFTOT      VBOT         t_impact    apogee      downrange
%       0 s    1938 s   7.193694 km/s    1938.37   1331.84 km   10000.00 km
%     500 s    2438 s   7.325046 km/s    2438.26   2288.78 km   10000.00 km
%    -200 s    1738 s   7.229680 km/s    1738.47    940.16 km   10000.00 km
%
%   VBOT is H-INDEPENDENT by construction (Lambert integrates nothing).
%   Sweeping H over 0.01 .. 0.5 s moved the impact time by 0.13 s and the
%   apogee by 0.001 km, so H is a speed knob here, not an accuracy one -- but
%   the listing's own 0.01 is kept as the default regardless.
%
% ---------------------------------------------------------------------------
%   OUTPUTS
%     R.cases     one struct per TLOFT: .name .TLOFT .TFTOT .VBOT_kms
%                 .t_impact_s .apogee_km .downrange_km .gam0_deg .iters
%                 .resid .dE .dL .hit_cap, plus .t .downrange_km_hist
%                 .alt_km_hist
%     R.bench     benchmark rows, schema shared with the other ports
%     R.selfcheck .names/.oks/.detail -- structural, NOT book values
%     R.pass      true only if at least one row was SCORED and all passed
%     R.verdict   '' when everything passed, else the reason.  ALWAYS formed;
%                 'assert' picks error vs warn-and-return-anyway, and
%                 benchSuite runs with assert off.
%     R.p, R.units, R.summary
%
%   No random numbers are used anywhere here, so there is no seed.
%
%   See also MGS.ZARCHAN.LAMBERTEFFICIENT, MGS.ZARCHAN.BALLISTICTARGETSIM,
%   MGS.ZARCHAN.BENCHSUITE.

arguments
    opts.RDESKM (1,1) double {mustBeFinite, mustBePositive} = 10000   % km
    % [] means the book's three cases: minimum energy, lofted, depressed.
    opts.TLOFT  double {mustBeReal, mustBeFinite} = []                % s
    opts.TS     (1,1) double {mustBeFinite, mustBePositive} = 1.0     % s, record
    opts.H      (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s, RK2
    % NOT nargin: with only name-value options declared, MATLAB reports
    % nargin = 0 however many were passed, so a mode switch on it never fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

A     = 2.0926E7;      % ft, the listing's Earth radius
GM    = 1.4077E16;     % ft^3/s^2
FTPKM = 3280.;         % the BOOK'S ft/km -- see the units block

TLOFT = opts.TLOFT(:).';
if isempty(TLOFT)
    TLOFT = [0 500 -200];          % zarchan.txt:12172, the book's three cases
end
if opts.H >= opts.TS
    error('mgs:zarchan:impulsiveIcbm:step', ...
        ['H = %g s is not smaller than the record interval TS = %g s, so the ' ...
         'trajectory would be sampled at the step rate while claiming the ' ...
         'coarser interval in its own time vector.'], opts.H, opts.TS);
end

nm = cell(1, numel(TLOFT));
for k = 1:numel(TLOFT)
    if TLOFT(k) == 0
        nm{k} = 'minimum energy';
    elseif TLOFT(k) > 0
        nm{k} = 'lofted';
    else
        nm{k} = 'depressed';
    end
end

C = runCase(opts.RDESKM, TLOFT(1), opts.TS, opts.H, A, GM, FTPKM, nm{1});
for k = 2:numel(TLOFT)
    C(k) = runCase(opts.RDESKM, TLOFT(k), opts.TS, opts.H, A, GM, FTPKM, nm{k});
end

R = struct();
R.cases = C;
R.units = ['internal state ft, ft/s (A = 2.0926e7 ft, GM = 1.4077e16 ft^3/s^2); ' ...
           'ALL REPORTED VALUES km, km/s, s via the book''s 3280 ft/km'];
R.p = struct('RDESKM', opts.RDESKM, 'TLOFT', TLOFT, 'TS', opts.TS, ...
             'H', opts.H, 'A_ft', A, 'GM', GM, 'FT_PER_KM', FTPKM, ...
             'units', R.units);

R.selfcheck = structuralChecks(C, opts.RDESKM, A, GM, FTPKM);

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
    msg = sprintf(['not one book value could be evaluated.  The published ' ...
        'numbers are for RDESKM = 10000 km with TLOFT in {0, 500, -200}; ' ...
        'this run used RDESKM = %g km, TLOFT = %s.  An unvalidated port ' ...
        'must not be handed back silently.'], opts.RDESKM, mat2str(TLOFT));
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
        error('mgs:zarchan:impulsiveIcbm:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:impulsiveIcbm:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function C = runCase(RDESKM, TLOFT, TS, H, A, GM, FTPKM, name)
%RUNCASE  Listing 32.1, zarchan.txt:12160-12171.  Every constant is an
%   ARGUMENT: MATLAB local functions cannot see the caller's workspace, and a
%   previous wave in this package lost an afternoon to exactly that.

TFTOT = 252. + .223*RDESKM - (5.44E-6)*RDESKM*RDESKM;   % zarchan.txt:12160
TFTOT = TFTOT + TLOFT;
XLONGFDEG = 57.3*RDESKM*FTPKM./A;
XLONGTDEG = 0.;  XLATTDEG = 0.;  XLATFDEG = 0.;
SWITCH = 0;  T = 0.;  S = 0.;
XLONGF = XLONGFDEG/57.3;  XLATF = XLATFDEG/57.3;       % the 57.3 cancels exactly
XF = A*cos(XLATF)*cos(XLONGF);  YF = A*cos(XLATF)*sin(XLONGF);  ZF = 0.;
XLONGT = XLONGTDEG/57.3;  XLATT = XLATTDEG/57.3;
XT = A*cos(XLATT)*cos(XLONGT);  YT = A*cos(XLATT)*sin(XLONGT);  ZT = 0.;
XTINIT = XT;  YTINIT = YT;  ZTINIT = 0.;
ALTTKM = (sqrt(XT^2+YT^2)-A)/FTPKM;

TGOLAM = TFTOT - T;
[VRX, VRY, VRZ, iters, resid, gam] = lambert3d(XT, YT, ZT, TGOLAM, ...
                                               XF, YF, ZF, SWITCH, GM, false);
XTD = VRX;  YTD = VRY;
ZTD = VRZ;   %#ok<NASGU>  the listing sets it; Z is never integrated (planar)
VBOT = sqrt(XTD^2+YTD^2)/FTPKM;

% Kepler invariants of the initial state, for SELFCHECK 4.
E0 = 0.5*(XTD^2+YTD^2) - GM/sqrt(XT^2+YT^2);
L0 = XT*YTD - YT*XTD;

nmax = ceil(2*max(TFTOT,1)/TS) + 8;
aT = zeros(nmax,1);  aD = zeros(nmax,1);  aA = zeros(nmax,1);
count = 0;  dE = 0;  dL = 0;
XTDD = 0.;  YTDD = 0.;

% Guard the listing's open-ended "while ALTTKM > -1": on a hyperbolic or
% non-finite solution it would never terminate (or, with NaN, would terminate
% instantly and silently).  The cap is reported either way, via .hit_cap.
stepMax = ceil(4*max(TFTOT,1)/H) + 16;
nStep   = 0;

while ALTTKM > -1 && nStep < stepMax
    XTOLD = XT;  YTOLD = YT;  XTDOLD = XTD;  YTDOLD = YTD;
    STEP = 1;  FLAG = 0;
    % Zarchan's two-pass predictor loop.  Pass 1 evaluates the derivatives at
    % the current state; pass 2 takes a full Euler step, advances T by H and
    % re-evaluates there.  T advances INSIDE this loop, exactly once.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            XT = XT + H*XTD;  YT = YT + H*YTD;
            XTD = XTD + H*XTDD;  YTD = YTD + H*YTDD;
            T = T + H;
        end
        TEMPBOTT = (XT^2+YT^2)^1.5;
        XTDD = -GM*XT/TEMPBOTT;
        YTDD = -GM*YT/TEMPBOTT;
        ALTTKM = (sqrt(XT^2+YT^2)-A)/FTPKM;
        FLAG = 1;
    end
    % The listing's "FLAG = 0" here is dead -- the outer loop re-initialises
    % FLAG on its next pass -- so it is kept as this comment, not as code, so
    % nobody re-adds it thinking a state machine was lost.
    XT  = .5*(XTOLD  + XT  + H*XTD);
    YT  = .5*(YTOLD  + YT  + H*YTD);
    XTD = .5*(XTDOLD + XTD + H*XTDD);
    YTD = .5*(YTDOLD + YTD + H*YTDD);
    S = S + H;
    nStep = nStep + 1;
    if S >= (TS - .0001)
        S = 0.;
        ALTTKM = (sqrt(XT^2+YT^2)-A)/FTPKM;
        DISTRTKNM = distance3dkm(XT, YT, ZT, XTINIT, YTINIT, ZTINIT, A, FTPKM);
        count = count + 1;
        aT(count) = T;  aD(count) = DISTRTKNM;  aA(count) = ALTTKM;
        E = 0.5*(XTD^2+YTD^2) - GM/sqrt(XT^2+YT^2);
        L = XT*YTD - YT*XTD;
        dE = max(dE, abs(E/E0 - 1));
        dL = max(dL, abs(L/L0 - 1));
    end
end

C = struct('name', name, 'TLOFT', TLOFT, 'TFTOT', TFTOT, 'VBOT_kms', VBOT, ...
    't_impact_s', T, 'apogee_km', maxOr(aA(1:count)), ...
    'downrange_km', lastOr(aD(1:count)), 'gam0_deg', gam*57.29577951308232, ...
    'iters', iters, 'resid', resid, 'VRZ', VRZ, 'E0', E0, 'L0', L0, ...
    'dE', dE, 'dL', dL, 'theta_rad', XLONGF, 'hit_cap', nStep >= stepMax, ...
    't', aT(1:count), 'downrange_km_hist', aD(1:count), ...
    'alt_km_hist', aA(1:count));
end


% =============================================================================
function [VRX, VRY, VRZ, ITERS, RESID, GAMMA] = lambert3d(XT, YT, ZT, TF, ...
                                                    XF, YF, ZF, SWITCH1, GM, FLIP)
%LAMBERT3D  Zarchan Listing 28.3, zarchan.txt:10428.  See the file header for
%   the nine OCR resolutions and the evidence behind each.  Returns RESID, the
%   loop's own relative convergence residual, and ITERS, so that a caller can
%   tell a converged answer from the ITERS > 100 break -- which the book's
%   version cannot, and which is what SELFCHECK 1 scores.
%
%   FLIP is SELFCHECK 5's delete experiment and is false on every real call.
%   It lives INSIDE this function on purpose: a copied-out variant would be
%   free to drift away from the transcription it is supposed to be testing,
%   at which point the experiment proves nothing about this code.
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
% The listing also forms S, BL and BT here.  None of the three is ever read.
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
    % ceiling and a T that is too SHORT raises the floor.  These are two of
    % the four variable-to-variable OCR resolutions; SELFCHECK 5 swaps exactly
    % this pair (and nothing else) and watches the bracket collapse.
    if FLIP
        tooLong = T < TF;   tooShort = T > TF;
    else
        tooLong = T > TF;   tooShort = T < TF;
    end
    if tooLong  && GAMMA < G_MAX,  G_MAX = GAMMA;               end
    if T < 0.   && GAMMA < G_MAX,  G_MAX = GAMMA;               end
    if tooShort && GAMMA > G_MIN,  G_MIN = GAMMA;  T_MIN = T;   end
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

% Unit vector at flight-path angle GAMMA above the local horizontal, built in
% the (r0, rf) plane as C1*r0 + C2*rf.  (The book's "GAMMA=GAMMA;" is a no-op
% and the missing semicolon after its second CROSSX is a display leak.)
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
VUNITX = C2*V2X + C1*V1X;
VUNITY = C2*V2Y + C1*V1Y;
VUNITZ = C2*V2Z + C1*V1Z;
VRX = RV0MAG*VUNITX;  VRY = RV0MAG*VUNITY;  VRZ = RV0MAG*VUNITZ;
end


% =============================================================================
function DISTKM = distance3dkm(XT, YT, ZT, XF, YF, ZF, A, FTPKM)
%DISTANCE3DKM  Zarchan Listing 28.3, zarchan.txt:10423 (named distance3d
%   there, distance3dkm in Ch.32).  Great-circle arc at the Earth radius A.
%   "if CBETA,=1." -> "<=": the else branch exists only to catch a rounding
%   overshoot that would otherwise turn acos() complex.
R  = sqrt(XT^2+YT^2+ZT^2);
RF = sqrt(XF^2+YF^2+ZF^2);
CBETA = (XT*XF+YT*YF+ZT*ZF)/(R*RF);
if CBETA <= 1.
    BETA = acos(CBETA);
    DISTKM = A*BETA/FTPKM;
else
    DISTKM = (XF-XT)/FTPKM;
end
end


% =============================================================================
function K = structuralChecks(C, RDESKM, A, GM, FTPKM)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.
%   /!\ .names and .oks must BOTH exist and be the SAME LENGTH, or benchSuite
%   puts ZERO rows on the board and the board looks green for the wrong reason.
names = {};  oks = [];  detail = {};

% 1  Lambert converged on its OWN criterion, not by falling out of the
%    ITERS > 100 break.  This is the row that notices a mis-resolved
%    comparison quietly returning the 100th iterate as though it were an
%    answer -- the book's routine gives the caller no way to tell.
r1 = max([C.resid]);  it1 = max([C.iters]);
names{end+1} = 'LAMBERT3D meets its own 1e-8 criterion, not the ITERS>100 break';
oks(end+1)   = r1 <= 1e-8 && it1 <= 100 && all(isfinite([C.VBOT_kms]));
detail{end+1} = sprintf('worst |TF-T|/TF = %.2e over %d case(s), worst ITERS = %d', ...
    r1, numel(C), it1);

% 2  The minimum-energy velocity against the classic closed form, which shares
%    NO code with the OCR'd Lambert iteration:
%        v_min^2 = (GM/A) * 2 sin(TH/2) / (1 + sin(TH/2))
%    Two-sided, and the two sides say different things.  The LOWER side is
%    exact mathematics -- no transfer between two points on the same sphere
%    can beat minimum energy -- so it is tight.  The UPPER side only says the
%    book's tFME POLYNOMIAL FIT lands near the true minimum-energy time of
%    flight, so it is loose on purpose; the fit's error grows with RDESKM.
j = find([C.TLOFT] == 0, 1);
if isempty(j)
    names{end+1} = 'minimum-energy VBOT vs closed form (not run: no TLOFT = 0 case)';
    oks(end+1)   = true;
    detail{end+1} = 'TLOFT = 0 was not among the requested cases';
else
    TH = C(j).theta_rad;  sh = sin(TH/2);
    vmin = sqrt((GM/A)*2*sh/(1+sh))/FTPKM;
    rel  = C(j).VBOT_kms/vmin - 1;
    names{end+1} = 'minimum-energy VBOT vs closed form: at or above it, within 1%';
    oks(end+1)   = rel >= -1e-6 && rel <= 0.01;
    detail{end+1} = sprintf('closed form %.6f, ours %.6f km/s (%+.3e rel); THETA = %.4f deg', ...
        vmin, C(j).VBOT_kms, rel, TH*57.29577951308232);
end

% 3  "Minimum energy" must actually be the cheapest of the cases flown.  If a
%    sign on TLOFT were lost this row is what would say so.
v = [C.VBOT_kms];  tl = [C.TLOFT];
names{end+1} = 'the TLOFT = 0 case needs the LEAST initial velocity of those flown';
if isempty(j) || numel(C) < 2
    oks(end+1)   = true;
    detail{end+1} = 'not run; needs TLOFT = 0 and at least one other case';
else
    oks(end+1)   = all(v(j) <= v + 1e-9);
    detail{end+1} = sprintf('VBOT = [%s] km/s at TLOFT = [%s] s', ...
        strtrim(sprintf('%.6f ', v)), strtrim(sprintf('%g ', tl)));
end

% 4  Kepler invariants along the integrated arc.  A sign error in XTDD/YTDD,
%    or a corrector that averages the wrong pair of states, shows up here and
%    nowhere else cheap -- the trajectory would still look like a trajectory.
r4 = max([max([C.dE]) max([C.dL])]);
names{end+1} = 'specific energy and angular momentum conserved along every arc';
oks(end+1)   = isfinite(r4) && r4 <= 1e-5;
detail{end+1} = sprintf('worst relative drift max(|dE/E0|, |dL/L0|) = %.2e', r4);

% 5  DELETE EXPERIMENT on the two variable-to-variable OCR resolutions: swap
%    which side of the bracket tightens.  If the flipped reading were the
%    right one, IT would converge and ours would not.
[okFlip, dFlip] = flipExperiment(RDESKM, A, GM, FTPKM);
names{end+1} = 'FLIP the bracket updates (T>TF <-> T<TF) -> LAMBERT3D stops converging';
oks(end+1)    = okFlip;
detail{end+1} = dFlip;

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
function [ok, txt] = flipExperiment(RDESKM, A, GM, FTPKM)
%FLIPEXPERIMENT  SELFCHECK 5.  Re-solves the minimum-energy Lambert problem
%   through THE SAME lambert3d above, with only the two bracket-update
%   comparisons swapped.  Lambert only -- no trajectory is integrated -- so it
%   costs a handful of iterations.
TF = 252. + .223*RDESKM - (5.44E-6)*RDESKM*RDESKM;
XLONGF = RDESKM*FTPKM/A;
[vx, vy, ~, ITERS, resid] = lambert3d(A, 0., 0., TF, A*cos(XLONGF), ...
                                      A*sin(XLONGF), 0., 0, GM, true);
v = sqrt(vx^2 + vy^2)/FTPKM;
% "Broken" means EITHER a non-finite answer (the bracket collapses to a point,
% the secant divides by zero, and the NaN comparison then exits the loop
% silently) OR a residual the correct reading never leaves behind.
ok  = ~isfinite(v) || ~isfinite(resid) || resid > 1e-6;
txt = sprintf(['flipped: %d iters, residual %.3g, VBOT %.6f km/s ' ...
    '(the correct reading converges in <= 8 iterations)'], ITERS, resid, v);
end


% =============================================================================
function B = benchmarks(C)
%BENCHMARKS  Every book value grepped from zarchan_decoded.txt; the line
%   number is in each row's .source.  Nothing here was read off a figure
%   IMAGE -- the extraction turned every figure into an image link.  The three
%   "s to Impact" numbers are figure ANNOTATION TEXT lines that survived
%   extraction verbatim, so they are quotable rather than squinted at, and
%   each is independently reproduced by the tFME formula inside the listing,
%   which is cited alongside on the same row.
B = emptyBench();

SRC_V = 'Listing 32.1 discussion, p.837; zarchan.txt:12172';
bk = struct('name',  {'minimum energy','lofted','depressed'}, ...
            'TLOFT', {0, 500, -200}, ...
            'V',     {7.19, 7.33, 7.23}, ...
            'TIMP',  {1938, 2438, 1738}, ...
            'TSRC',  {'zarchan.txt:12185','zarchan.txt:12186','zarchan.txt:12187'});

for k = 1:numel(bk)
    j = find([C.TLOFT] == bk(k).TLOFT, 1);
    what = sprintf('initial velocity, %s (TLOFT = %g s)', bk(k).name, bk(k).TLOFT);
    if isempty(j)
        B = addSkip(B, what, sprintf('%.2f', bk(k).V), ...
            sprintf('TLOFT = %g s was not among the cases flown', bk(k).TLOFT), ...
            SRC_V, 'km/s');
    else
        B = addVal(B, what, bk(k).V, C(j).VBOT_kms, '0.5%', 0.005, SRC_V, 'km/s', ...
            sprintf(['Lambert only, so H-independent; %d iterations, ' ...
            'gamma0 = %.3f deg'], C(j).iters, C(j).gam0_deg));
    end
end

for k = 1:numel(bk)
    j = find([C.TLOFT] == bk(k).TLOFT, 1);
    if isempty(j), continue, end
    B = addVal(B, sprintf('time to impact, %s', bk(k).name), bk(k).TIMP, ...
        C(j).t_impact_s, '0.5%', 0.005, ...
        sprintf(['Fig. 32.1 annotation, %s; the same number falls out of ' ...
        'tFME in the listing, zarchan.txt:12160'], bk(k).TSRC), 's', ...
        sprintf(['TFTOT from tFME = %.1f s, so this row scores the ' ...
        'INTEGRATION landing where Lambert aimed it.  The PROSE form of ' ...
        'tFME at :12157 is OCR-damaged (minus sign and 1e-6 exponent both ' ...
        'lost); the listing''s form is intact and is what is scored.'], ...
        C(j).TFTOT));
end

j = find([C.TLOFT] == 0, 1);
SRC_D = ['prose "traveling 10,000 km", zarchan.txt:12158; RDESKM in the ' ...
         'listing, zarchan.txt:12160'];
if isempty(j)
    B = addSkip(B, 'downrange travelled at impact', '10000', ...
        'TLOFT = 0 was not among the cases flown', SRC_D, 'km');
else
    B = addVal(B, 'downrange travelled at impact (minimum energy)', 10000, ...
        C(j).downrange_km, '0.5%', 0.005, SRC_D, 'km', ...
        'great-circle arc from the launch point, last recorded sample');
end

apo = [C.apogee_km];  timp = [C.t_impact_s];
distinct = numel(C) >= 2 && all(isfinite(apo)) && all(isfinite(timp)) && ...
           numel(uniquetol(apo, 1e-4)) == numel(apo) && ...
           numel(uniquetol(timp, 1e-6)) == numel(timp);
B = addShape(B, 'the three trajectories differ in apogee AND in arrival time', ...
    'all the trajectories have different apogees and arrive at different times', ...
    sprintf('apogees [%s] km, impacts [%s] s', ...
        strtrim(sprintf('%.1f ', apo)), strtrim(sprintf('%.1f ', timp))), ...
    distinct, 'zarchan.txt:12192', ...
    'an ORDERING claim the prose makes about Fig. 32.1, not a figure reading');

jm = find([C.TLOFT] == 0, 1);  jl = find([C.TLOFT] == 500, 1);
if isempty(jm) || isempty(jl)
    B = addSkip(B, 'lofted apogee exceeds the minimum-energy apogee', ...
        'apogee(lofted) > apogee(minimum energy)', ...
        'needs both TLOFT = 0 and TLOFT = 500 among the cases flown', ...
        'zarchan.txt:12558', 'km');
else
    B = addShape(B, 'lofted apogee exceeds the minimum-energy apogee', ...
        'its apogee is higher than that of a minimum energy trajectory', ...
        sprintf('%.1f km vs %.1f km', C(jl).apogee_km, C(jm).apogee_km), ...
        C(jl).apogee_km > C(jm).apogee_km, 'zarchan.txt:12558', ...
        ['the same TLOFT = 500 vs 0 comparison for the same 10,000-km ICBM, ' ...
         'stated later in Ch.32 where the loft case is re-run']);
end

B = addSkip(B, 'apogee MAGNITUDE for Fig. 32.1', '(never published)', ...
    ['Fig. 32.1 publishes only axis ticks (0/500/1000/1500/2000 km, ' ...
     'zarchan.txt:12176-12182) and states no apogee for any of the three ' ...
     'trajectories.  The "approximately 1800 km" at :12627 and "approximately ' ...
     '2000 km" at :13597 belong to Listings 33.1 and 34.2 at a different ' ...
     'range and a different booster, so neither is a value for this listing. ' ...
     'Only the ORDERING rows above are scoreable.'], ...
    'Fig. 32.1, zarchan.txt:12176-12188', 'km');
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER are benchSuite's adaptGeneric schema, which
%   is STRICT: a near-miss (book_s for book_str) errors rather than adapting.
%   Every row in this file is built through one of the three constructors
%   below so the order cannot drift.
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

function B = addShape(B, what, bookStr, oursStr, ok, source, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'shape';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'shape';  B(i).tol = NaN;
B(i).ok   = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = '-';  B(i).detail = detail;
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


% =============================================================================
function s = render(R, ev)
L = {};
L{end+1} = 'Zarchan Listing 32.1 -- IMPULSIVE ICBM, two-dimensional trajectory';
L{end+1} = sprintf('   %s', R.units);
L{end+1} = sprintf('   RDESKM = %g km, H = %g s, TS = %g s', ...
    R.p.RDESKM, R.p.H, R.p.TS);
L{end+1} = '';
L{end+1} = '   case              TLOFT     TFTOT       VBOT     gamma0    t_impact     apogee   downrange';
L{end+1} = '                       [s]       [s]     [km/s]      [deg]         [s]       [km]        [km]';
for i = 1:numel(R.cases)
    c = R.cases(i);
    L{end+1} = sprintf('   %-16s %7g %9.1f %10.6f %10.4f %11.2f %10.2f %11.2f', ...
        c.name, c.TLOFT, c.TFTOT, c.VBOT_kms, c.gam0_deg, c.t_impact_s, ...
        c.apogee_km, c.downrange_km); %#ok<AGROW>
    if c.hit_cap
        L{end+1} = '        WARNING: step cap reached -- this arc never came down'; %#ok<AGROW>
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
            case 'skip'
                L{end+1} = sprintf('   SKIP %-52s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why);                     %#ok<AGROW>
                L{end+1} = sprintf('        [%s]', b.source);                %#ok<AGROW>
            case 'shape'
                L{end+1} = sprintf('   %-4s %-52s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str);          %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail);  %#ok<AGROW>
            otherwise
                L{end+1} = sprintf(['   %-4s %-52s book %s %s / ours %s %s ' ...
                    '(%+.4f%%, band %.4g%%)'], vstr(b.ok), b.what, ...
                    b.book_str, b.units, b.ours_str, b.units, b.pct, 100*b.tol); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail);  %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: IMPULSIVE launch, KEPLERIAN point-mass gravity, a ROUND and';
L{end+1} = '   NON-ROTATING Earth, NO atmosphere and NO drag.  This file sizes a';
L{end+1} = '   burnout velocity and draws a ballistic arc; there is no guidance';
L{end+1} = '   loop anywhere in it.';
s = strjoin(L, newline);
end

function s = vstr(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end

function drawTraj(R)
figure;
hold on;
for i = 1:numel(R.cases)
    plot(R.cases(i).downrange_km_hist, R.cases(i).alt_km_hist, 'DisplayName', ...
        sprintf('%s (TLOFT = %g s)', R.cases(i).name, R.cases(i).TLOFT));
end
grid on;  xlabel('Downrange (km)');  ylabel('Altitude (km)');
title(sprintf('Impulsive %g km Ballistic Target', R.p.RDESKM));
legend('Location', 'best');
hold off;
end

function y = maxOr(x)
if isempty(x), y = NaN; else, y = max(x); end
end

function y = lastOr(x)
if isempty(x), y = NaN; else, y = x(end); end
end
