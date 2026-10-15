function R = fcsOpenLoop(opts)
%MGS.ZARCHAN.FCSOPENLOOP  Zarchan Listing 22.1 -- OPEN-LOOP FLIGHT-CONTROL
%   SYSTEM: the 10-g step response of Fig. 22.2 at the book's two flight
%   conditions, with ringing frequency and damping MEASURED off the response
%   rather than read out of the aero algebra.
%
%   R = mgs.zarchan.fcsOpenLoop()
%   R = mgs.zarchan.fcsOpenLoop('tstop', 1.99999)      % the listing verbatim
%   R = mgs.zarchan.fcsOpenLoop('benchmark', false, 'alt', 20000, 'XNCG', 5)
%
%   UNITS ARE THE BOOK'S: FEET, ft/s, slug/ft^3, psf, g = 32.2 ft/s^2, ANGLES
%   IN DEGREES.  nL in g, del/alpha deg, wAF/wz rad/s, Ma/Md 1/s^2, Za/Zd 1/s,
%   K1 g/deg, KDC deg/g.  Nothing is converted and the rest of mgs is METRIC:
%   convert AT THE CALL SITE and say so there.  1845 is the book's rounding of
%   57.3*32.2 = 1845.06, reproduced, not "fixed".
%
%   WHAT IT IS.  The open-loop autopilot is ONE GAIN, KDC = 1/K1, cancelling
%   the airframe's aero acceleration gain.  No rate gyro, accelerometer,
%   actuator or feedback: the command is divided by K1 and the quotient IS the
%   fin deflection.  So the response is the BARE AIRFRAME response, DC
%   corrected -- steady state lands on the command exactly, the transient does
%   not improve at all.  That is the chapter's point and why Ch.22 then adds a
%   rate gyro.
%
%   WHAT IT CANNOT SAY.  Constant speed and altitude, pitch plane only,
%   SUPERSONIC linearised aero (beta = sqrt(M^2-1) is imaginary below Mach 1),
%   coefficients frozen at ONE trim alpha, no gravity/drag/thrust/actuator, no
%   fin or acceleration limit, no guidance loop.  NOT a miss-distance model.
%   It is here because it is the smallest complete example of the thing we
%   cannot do on our own vehicle: damping and frequency DERIVED from geometry,
%   then MEASURED back out of the time history, the two agreeing to ~0.02%.
%
%   OVERLAP WITH AIRFRAMELINEAR -- READ BEFORE ADDING ROWS.  airframeLinear
%   (Listing 21.2) already pulls in 22.1's trim quadratic as trimAccel() and
%   already scores all twelve Table 22.1/22.2 numbers ALGEBRAICALLY.  Those are
%   NOT repeated.  This file adds what 21.2 lacks: the KDC autopilot, the
%   closed-loop step response, Figs. 22.3/22.4.  Its wAF/zAF rows measure them
%   FROM THE SIMULATED RESPONSE -- a different route, which fails on integrator
%   faults the algebraic rows cannot see.  The quadratic is transcribed again
%   rather than called across files so this port stands alone; self-checks 1-2
%   prove both trim equations hold to 1e-10 relative, which makes that safe.
%
%   TRANSCRIPTION: matlab/zarchan/_listings/listing_22_01.m, zarchan.txt:6954-
%   6964 (pp.504-505).  NO VARIABLE-TO-VARIABLE COMPARISON EXISTS HERE.  All
%   four damaged comparisons are variable-to-CONSTANT, were already restored by
%   the extractor, were re-checked against the raw text, and are forced by
%   shape: "if ALT,=30000." -> "<=" (ALT=0 must give RHO=.002378, standard
%   sea-level density, so the TRUE branch is the LOW one; reversed, sea level
%   runs at .0034, +43% Q, and everything is wrong while still looking like a
%   missile -- the 50-kft case exercises the else); "while T,1.99999" -> "<";
%   "while STEP,=1" -> "<=" (the body sets STEP=2 to leave); "if S.=.0099999"
%   -> ">=" (S is reset on each record).  Two trailing PROSE lines leaked in
%   from zarchan.txt:6966 on -- dropped, not "fixed" into code.  The dead
%   "FLAG=0" after the inner loop is dropped; "save datfil.txt" is not done.
%
%   THE BOOK'S T = 2 s IS A PLOT WINDOW, NOT A SETTLING TIME: 1/(zAF*wAF) is
%   0.68 s at sea level (2 s = three time constants, still +/-5% of command)
%   and 3.75 s at 50 kft (not even one).  'tstop' therefore defaults to
%   max(2 s, 8/(zAF*wAF)) = 5.44 and 30.0 s, while the Fig. 22.3 row still
%   looks only at the first 2 s.  Pass 'tstop', 1.99999 for the listing
%   verbatim; the steady-state rows then SKIP rather than score an endpoint
%   that is a point on a ring.
%
%   BENCHMARKS -- 10 rows, every book value grepped, line number in .source,
%   nothing read off a figure (every figure became an image link):
%     1-4  wAF and zAF at both conditions, Table 22.1 (zarchan.txt:6934-6935),
%          MEASURED -- wd from zero-crossing spacing, sigma from the log
%          decrement between first and last extremum, wn = hypot(sigma,wd).
%          Both are EXACT for a damped sinusoid (extrema recur exactly one
%          period apart; successive ratios are exactly exp(-s*2pi/wd)), so the
%          residual is sampling and fit bias only.  BANDS 1.5% / 5% ARE THE
%          BOOK'S ROUNDING, not our error: "10.0" is 0.5% wide, "0.027" 1.9%.
%     5-6  Steady-state nL = the 10-g command (zarchan.txt:6869-6874 for the
%          claim, :6928 for the 10 g, :6930 for the restatement), band
%          0.5%, ours = mean over the last two damped periods.
%     7    SHAPE, :6930 -- sea-level response "oscillatory due to the low
%          damping": peak > 1.5x command AND >= 6 crossings, within 2 s.
%     8    SHAPE, :6945 -- "increasing the altitude decreases the damping and
%          natural frequency".  Both inequalities, both measured.
%     9    SKIP -- Fig. 22.6's radome-slope band (:6972) is the output of an
%          ADJOINT of the Fig. 22.5 homing loop, which is NOT this listing and
%          is printed nowhere.  Reproducing it would mean inventing the loop.
%    10    INFO -- the trim roots; the chapter publishes no number for them.
%
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED -- never executed here;
%   MATLAB is the caller's. ***  Sea level: wAF 25.3398, zAF 0.058010, wz
%   43.2067, K1 -3.07153 g/deg, KDC -0.325570 deg/g, del -3.25570 deg, trim
%   alpha 0.04907 / delta -0.05682 deg, nL(0+) -3.4396 g (the RHP zero kicks
%   the WRONG WAY first), peak 21.169 g, 16 crossings of 10 g inside 2 s,
%   measured wn 25.3398 (+0.16%) and zeta 0.058011 (+0.02%), steady state
%   9.99974 g, tstop 5.442 s, 544 samples.  50 kft: wAF 9.9531, zAF 0.026796,
%   wz 18.8720, K1 -0.55951, KDC -1.787277, trim 0.25719 / -0.31193, nL(0+)
%   -2.7815 g, peak 21.738 g, measured wn 9.9531 (-0.47%) and zeta 0.026796
%   (-0.76%), steady state 10.00004 g, tstop 29.995 s, 2999 samples.
%   Structural: trim residuals ~1e-15 relative, |KDC*K1-1| ~1e-16, max
%   |nL_sim - nL_exact| 9.1e-5 g (SL) and 2.9e-5 g (50 kft).  A residual near a
%   band EDGE is not a pass to accept.  Do not widen a band to make a row green.
%
%   OUTPUTS.  R.cond (per altitude: .alt .aero .t .nl .nl_exact .wn .zeta .wd
%   .nl_ss .peak .nl0 .ncross2s .peak2s .tstop .cf_err .settled), R.bench,
%   R.selfcheck (.names .oks .details .all_ok .first_bad), R.pass (true only if
%   at least one row was SCORED and all passed), R.verdict (ALWAYS formed;
%   'assert' picks the reaction and nothing else), R.units, R.summary.
%
%   See also MGS.ZARCHAN.AIRFRAMELINEAR (Listing 21.2, the algebraic twin).

arguments
    opts.VM    (1,1) double {mustBeFinite, mustBePositive} = 3000    % ft/s
    opts.XNCG  (1,1) double {mustBeFinite, mustBePositive} = 10      % g command
    opts.alt   (1,:) double {mustBeFinite, mustBeNonnegative} = [0 50000]  % ft
    opts.H     (1,1) double {mustBeFinite, mustBePositive} = 0.0001  % s
    opts.samp  (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s
    opts.tstop double {mustBeReal} = []      % [] = max(2 s, 8/(zAF*wAF))
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so "if nargin == 0" here would be a mode that never fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

if opts.VM <= 1000
    error('mgs:zarchan:fcsOpenLoop:subsonic', ...
        ['VM = %g ft/s is at or below the book''s constant speed of sound ' ...
         '(1000 ft/s), so beta = sqrt(M^2-1) is zero or imaginary and the ' ...
         'supersonic fin approximation does not exist.  Refused rather than ' ...
         'returned complex.'], opts.VM);
end
if opts.samp < opts.H
    error('mgs:zarchan:fcsOpenLoop:samp', ...
        ['samp = %g s is below the step H = %g s, so the curve would be ' ...
         'sampled at the step rate while claiming the finer interval in its ' ...
         'own time vector.'], opts.samp, opts.H);
end
if ~isempty(opts.tstop) && (~isscalar(opts.tstop) || opts.tstop <= 4*opts.samp)
    error('mgs:zarchan:fcsOpenLoop:tstop', ...
        'tstop must be [] or a scalar longer than four sample intervals.');
end
if isempty(opts.alt)
    error('mgs:zarchan:fcsOpenLoop:alt', 'alt is empty; nothing to run.');
end

alts = opts.alt(:).';
C = struct('alt',{}, 'aero',{}, 'tstop',{}, 't',{}, 'nl',{}, 'nl_exact',{}, ...
           'wd',{}, 'wn',{}, 'zeta',{}, 'ncross',{}, 'nl_ss',{}, 'peak',{}, ...
           'nl0',{}, 'ncross2s',{}, 'peak2s',{}, 'nsteps',{}, 'cf_err',{}, ...
           'settled',{});

for k = 1:numel(alts)
    A  = aeroTrim(alts(k), opts.VM, opts.XNCG);
    ts = opts.tstop;
    if isempty(ts), ts = max(1.99999, 8/(A.zaf*A.waf)); end

    [t, nl, nsteps] = simRun(A, ts, opts.H, opts.samp);
    if numel(t) < 4
        error('mgs:zarchan:fcsOpenLoop:empty', ...
            ['the run at %g ft recorded %d sample(s).  Under four is refused: ' ...
             'a record that short cannot be measured, and returning it ' ...
             'quietly would look like a converged result.'], alts(k), numel(t));
    end

    nlx = stepExact(t, A.waf, A.zaf, A.wz, A.xk1, A.del_deg);
    M   = ringing(t, nl, opts.XNCG);

    % Steady state = mean over the last two DAMPED PERIODS.  Averaging over
    % whole periods removes the residual ring instead of sampling a random
    % point on it.  SETTLED means six envelope time constants of record
    % (residual 0.25% of the initial swing), which is what the 0.5% band
    % assumes; short runs skip the row rather than quote an endpoint.
    if isfinite(M.wd) && M.wd > 0
        win = min(2*(2*pi/M.wd), 0.5*(t(end) - t(1)));
    else
        win = 0.25*(t(end) - t(1));
    end
    tail    = t >= t(end) - win;
    settled = isfinite(M.zeta) && M.zeta > 0 && isfinite(M.wn) && ...
              (t(end) - t(1)) >= 6/(M.zeta*M.wn);

    % The book's own 2-s plot window, for the Fig. 22.3 row.  If samp is coarse
    % enough that nothing lands inside it these come back NaN and the row
    % SKIPS; it must not read as a shape that failed.
    in2 = t <= 2.0;
    nx2 = NaN;  pk2 = NaN;
    if any(in2)
        r2  = nl(in2) - opts.XNCG;
        nx2 = sum(r2(1:end-1).*r2(2:end) < 0);
        pk2 = max(nl(in2));
    end

    i = numel(C) + 1;
    C(i).alt   = alts(k);   C(i).aero  = A;       C(i).tstop = ts;
    C(i).t     = t;         C(i).nl    = nl;      C(i).nl_exact = nlx;
    C(i).wd    = M.wd;      C(i).wn    = M.wn;    C(i).zeta  = M.zeta;
    C(i).ncross = M.ncross; C(i).nl_ss = mean(nl(tail));
    C(i).peak  = max(nl);
    C(i).nl0   = stepExact(0, A.waf, A.zaf, A.wz, A.xk1, A.del_deg);
    C(i).ncross2s = nx2;    C(i).peak2s = pk2;
    C(i).nsteps = nsteps;   C(i).cf_err = max(abs(nl - nlx));
    C(i).settled = settled;
end

R = struct();
R.cond  = C;
R.units = ['ft, ft/s, slug/ft^3, psf; angles DEG, nL in g, g = 32.2 ft/s^2; ' ...
           'wAF/wz rad/s, K1 g/deg, KDC deg/g'];
R.p = struct('VM',opts.VM, 'XNCG',opts.XNCG, 'alt',alts, 'H',opts.H, ...
             'samp',opts.samp, 'tstop',opts.tstop, 'units',R.units);
R.selfcheck = structuralChecks(C, opts.XNCG);

if opts.benchmark
    R.bench = benchmarks(C, opts.VM, opts.XNCG);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine" -- the shape of green light this repo has been burned by.
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(C, opts.XNCG); end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' false: nothing compared.  R.pass = false means NOT CHECKED.';
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated.  Table 22.1 is ' ...
        'published for VM = 3000 ft/s and a 10-g command at 0 and 50000 ft; ' ...
        'this run used VM = %g, nc = %g g, alt = %s.'], ...
        opts.VM, opts.XNCG, mat2str(alts));
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, a wiring fault, not a tolerance question.  First: %s.'], ...
        sum(~R.selfcheck.oks), R.selfcheck.first_bad);
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
        error('mgs:zarchan:fcsOpenLoop:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN, complete, so the caller can see WHICH side moved.
        % benchSuite runs every port with assert off for exactly this reason.
        warning('mgs:zarchan:fcsOpenLoop:benchmark', ...
            '%s  (''assert'' false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function A = aeroTrim(ALT, VM, XNCG)
%AEROTRIM  Listing 22.1 line 1 through XKDC, verbatim (zarchan.txt:6954-6960).
%   Trims on a given ACCELERATION (22.1), not on a given fin deflection (21.2):
%   CN_trim = W*nc/(Q*Sref) is known and alpha_trim falls out of the quadratic
%   formed by eliminating delta through the zero-moment equation.
%
%   Every constant is an ARGUMENT.  MATLAB local functions cannot see the main
%   function's workspace; a previous wave in this package read one from the
%   caller and threw on the first line of its first run.
ASND = 1000.;                     % speed of sound, ft/s -- held constant
DIAM = 1.;  FR = 3.;  XL = 20.;
CTW = 0.;  CRW = 6.;  HW = 2.;  CTT = 0.;  CRT = 2.;  HT = 2.;
XN = 4.;  XCG = 10.;  XHL = 19.5;  WGT = 1000.;

if ALT <= 30000.                  % direction forced -- see the header
    RHO = .002378*exp(-ALT/30000.);
else
    RHO = .0034*exp(-ALT/22000.);
end

SWING = .5*HW*(CTW+CRW);
STAIL = .5*HT*(CTT+CRT);
SREF  = 3.1416*DIAM*DIAM/4.;
XLP   = FR*DIAM;
SPLAN = (XL-XLP)*DIAM + 1.33*XLP*DIAM/2.;
XCPN  = 2*XLP/3;
AN    = .67*XLP*DIAM;
AB    = (XL-XLP)*DIAM;
XCPB  = (.67*AN*XLP + AB*(XLP + .5*(XL-XLP)))/(AN+AB);
XCPW  = XLP + XN + .7*CRW - .2*CTW;
XMACH = VM/ASND;
XIYY  = WGT*(3*((DIAM/2)^2) + XL*XL)/(12*32.2);
TMP1  = (XCG-XCPW)/DIAM;  TMP2 = (XCG-XHL)/DIAM;
TMP3  = (XCG-XCPB)/DIAM;  TMP4 = (XCG-XCPN)/DIAM;
B     = sqrt(XMACH^2 - 1);
Q     = .5*RHO*VM*VM;
P1    = WGT*XNCG/(Q*SREF);                     % CN at trim
Y1 = 2. + 8*SWING/(B*SREF) + 8*STAIL/(B*SREF);
Y2 = 1.5*SPLAN/SREF;
Y3 = 8*STAIL/(B*SREF);
Y4 = 2*TMP4 + 8*SWING*TMP1/(B*SREF) + 8*STAIL*TMP2/(B*SREF);
Y5 = 1.5*SPLAN*TMP3/SREF;
Y6 = 8*STAIL*TMP2/(B*SREF);
P2 = Y2 - Y3*Y5/Y6;
P3 = Y1 - Y3*Y4/Y6;

disc = P3*P3 + 4.*P2*P1;
if disc < 0
    error('mgs:zarchan:fcsOpenLoop:trim', ...
        ['the trim quadratic has no real root at %g ft for a %g-g command ' ...
         '(discriminant %g): the airframe cannot make that acceleration at ' ...
         'this dynamic pressure.  A complex trim angle would propagate ' ...
         'silently into wAF and the whole response.'], ALT, XNCG, disc);
end
ALFTR = (-P3 + sqrt(disc))/(2.*P2);
DELTR = -Y4*ALFTR/Y6 - Y5*ALFTR*ALFTR/Y6;

CNA  = 2 + 1.5*SPLAN*ALFTR/SREF + 8*SWING/(B*SREF) + 8*STAIL/(B*SREF);
CND  = 8*STAIL/(B*SREF);
CMAP = 2*TMP4 + 1.5*SPLAN*ALFTR*TMP3/SREF + 8*SWING*TMP1/(B*SREF);
CMA  = CMAP + 8*STAIL*TMP2/(B*SREF);
CMD  = 8*STAIL*TMP2/(B*SREF);
XMA  = Q*SREF*DIAM*CMA/XIYY;
XMD  = Q*SREF*DIAM*CMD/XIYY;
ZA   = -32.2*Q*SREF*CNA/(WGT*VM);
ZD   = -32.2*Q*SREF*CND/(WGT*VM);

if XMA >= 0
    error('mgs:zarchan:fcsOpenLoop:unstable', ...
        ['Ma = %g 1/s^2 is not negative at %g ft, so wAF = sqrt(-Ma) is ' ...
         'imaginary: statically UNSTABLE at this trim, and the second-order ' ...
         'form this listing integrates does not apply.'], XMA, ALT);
end

WZ   = sqrt((XMA*ZD - ZA*XMD)/ZD);
WAF  = sqrt(-XMA);
ZAF  = .5*WAF*ZA/XMA;
XK1  = -VM*(XMA*ZD - XMD*ZA)/(1845*XMA);       % 1845 = the book's 57.3*32.2
TA   = XMD/(XMA*ZD - XMD*ZA);
XK3  = 1845*XK1/VM;
XKDC = 1./XK1;                                 % THE ENTIRE OPEN-LOOP AUTOPILOT

A = struct('alt',ALT, 'vm',VM, 'xncg',XNCG, 'rho',RHO, 'q',Q, 'mach',XMACH, ...
           'cn_trim',P1, 'alfa_trim',ALFTR, 'del_trim',DELTR, ...
           'y',[Y1 Y2 Y3 Y4 Y5 Y6], 'xma',XMA, 'xmd',XMD, 'za',ZA, 'zd',ZD, ...
           'waf',WAF, 'zaf',ZAF, 'wz',WZ, 'xk1',XK1, 'xk3',XK3, 'ta',TA, ...
           'xkdc',XKDC, 'del_deg',XKDC*XNCG);
end


% =============================================================================
function [t, nl, nsteps] = simRun(A, tstop, H, samp)
%SIMRUN  Listing 22.1's integration loop, verbatim (zarchan.txt:6961-6964).
%
%   E is the airframe state, E'' + 2*zAF*wAF*E' + wAF^2*E = wAF^2*DEL, and
%   nL = K1*(E - E''/wz^2) applies the numerator's zeros at +/-wz.  The "+wz"
%   one is the RIGHT-HALF-PLANE zero: the response starts on the wrong side.
%
%   Zarchan's two-pass predictor loop.  Pass 1 evaluates derivatives at the
%   current state; pass 2 takes a full Euler step, advances T by H and
%   re-evaluates there.  T advances INSIDE that loop exactly once -- move the
%   line out and the scheme quietly degrades to first order while still
%   running smoothly.
WAF = A.waf;  ZAF = A.zaf;  WZ = A.wz;  XK1 = A.xk1;  DEL = A.del_deg;
E = 0.;  ED = 0.;  EDD = 0.;  XNL = 0.;
T = 0.;  S = 0.;

nmax = ceil(tstop/samp) + 8;
t  = zeros(nmax,1);
nl = zeros(nmax,1);
n  = 0;
nsteps = 0;

while T < tstop
    EOLD = E;  EDOLD = ED;
    STEP = 1;  FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            E  = E  + H*ED;
            ED = ED + H*EDD;
            T  = T  + H;
        end
        EDD = WAF*WAF*(DEL - E - 2.*ZAF*ED/WAF);
        XNL = XK1*(E - EDD/WZ^2);
        FLAG = 1;
    end
    % The listing's "FLAG=0" here is dead (the outer loop re-initialises it).
    % Kept as a comment so nobody re-adds it thinking a state machine was lost.
    E  = .5*(EOLD  + E  + H*ED);
    ED = .5*(EDOLD + ED + H*EDD);
    nsteps = nsteps + 1;

    S = S + H;
    if S >= samp - 0.5*H
        S = 0.;
        n = n + 1;
        t(n)  = T;
        nl(n) = XNL;
    end
end
t = t(1:n);  nl = nl(1:n);
end


% =============================================================================
function y = stepExact(t, waf, zaf, wz, xk1, del)
%STEPEXACT  Exact step response of the TRANSCRIBED ODEs.  Shares no code with
%   the integrator, which is why it is worth having: a second opinion, not a
%   restatement of the first.
%
%     e   = del*(1 - e^{-z w t}(cos wd t + r sin wd t)),  r = z/sqrt(1-z^2)
%     e'' = del*w^2*e^{-z w t}(cos wd t - r sin wd t)
%     nL  = K1*(e - e''/wz^2)
%
%   e''(0) = del*w^2 reproduces EDD at the listing's own IC (E = ED = 0), and
%   nL(0) = -nc*(w/wz)^2 is the non-minimum-phase kick.  zeta < 1 only; both
%   book conditions are under 0.06.
if zaf >= 1
    y = nan(size(t));
    return
end
s   = sqrt(1 - zaf^2);
wd  = waf*s;
r   = zaf/s;
ex  = exp(-zaf*waf*t);
e   = del*(1 - ex.*(cos(wd*t) + r*sin(wd*t)));
edd = del*waf^2*ex.*(cos(wd*t) - r*sin(wd*t));
y   = xk1*(e - edd/wz^2);
end


% =============================================================================
function M = ringing(t, y, yss)
%RINGING  Frequency and damping MEASURED off the response.
%
%   For x = A*exp(-s t)*cos(wd t + p), dx/dt = 0 gives tan(wd t + p) = -s/wd,
%   recurring every pi/wd: extrema are exactly half a period apart and the
%   ratio of consecutive extrema is exactly exp(-s*pi/wd).  Both estimators are
%   EXACT for this signal; the residual is sampling and fit bias only.  wd from
%   zero crossings (linearly interpolated -- the crossing is the most nearly
%   linear part of the waveform), sigma from the log decrement between the
%   FIRST and LAST extremum, which averages the fit bias over the whole record.
M = struct('wd',NaN, 'wn',NaN, 'zeta',NaN, 'sigma',NaN, 'ncross',0, 'npeak',0);
t = t(:);  r = y(:) - yss;

sg  = sign(r);
idx = find(sg(1:end-1).*sg(2:end) < 0);
if numel(idx) >= 3
    a  = abs(r(idx));  b = abs(r(idx+1));
    tc = t(idx) + (t(idx+1) - t(idx)).*a./(a + b);
    M.ncross = numel(tc);
    M.wd = pi*(numel(tc) - 1)/(tc(end) - tc(1));
end

pv = [];
d  = diff(r);
j  = find(d(1:end-1).*d(2:end) < 0) + 1;      % r(j) is the interior extremum
if ~isempty(j)
    y0 = r(j-1);  y1 = r(j);  y2 = r(j+1);
    den = y0 - 2*y1 + y2;
    dx  = zeros(size(den));
    okd = den ~= 0;
    dx(okd) = 0.5*(y0(okd) - y2(okd))./den(okd);
    pv = y1 - 0.25*(y0 - y2).*dx;               % parabolic peak value
    % Drop anything at the numerical floor: a "zero" extremum sends the log
    % decrement to Inf and the row would then fail for the wrong reason.
    pv = pv(abs(pv) > 1e-9*max(abs(r)));
    M.npeak = numel(pv);
end

if M.npeak >= 3 && isfinite(M.wd) && M.wd > 0
    M.sigma = log(abs(pv(1))/abs(pv(end)))/((M.npeak - 1)*pi/M.wd);
    if M.sigma > 0
        M.wn   = hypot(M.sigma, M.wd);
        M.zeta = M.sigma/M.wn;
    end
end
end


% =============================================================================
function K = structuralChecks(C, XNCG)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never to be quoted
%   as evidence that this port reproduces Zarchan -- only that it agrees with
%   ITSELF, which is what catches wiring faults.
names = {};  oks = [];  det = {};
for k = 1:numel(C)
    A = C(k).aero;  lab = sprintf('@%g ft', A.alt);
    Y = A.y;  a = A.alfa_trim;  dtr = A.del_trim;

    % 1  MOMENT trim -- the equation the quadratic was solved through.
    mres = Y(4)*a + Y(5)*a*a + Y(6)*dtr;
    msc  = max(abs([Y(4)*a, Y(5)*a*a, Y(6)*dtr]));
    [names, oks, det] = add(names, oks, det, ...
        sprintf('moment trim y4*a + y5*a^2 + y6*d = 0 %s', lab), ...
        abs(mres) <= 1e-10*max(msc, realmin), ...
        sprintf('residual %.3e (largest term %.3e)', mres, msc));

    % 2  FORCE trim must return the CN the command asked for.
    fres = Y(1)*a + Y(2)*a*a + Y(3)*dtr - A.cn_trim;
    [names, oks, det] = add(names, oks, det, ...
        sprintf('force trim y1*a + y2*a^2 + y3*d = CN_trim %s', lab), ...
        abs(fres) <= 1e-10*max(abs(A.cn_trim), realmin), ...
        sprintf('residual %.3e against CN_trim %.5f', fres, A.cn_trim));

    % 3  The autopilot IS the cancellation.  If KDC stops being 1/K1 the
    %    response still looks like a missile and settles on a wrong number.
    [names, oks, det] = add(names, oks, det, ...
        sprintf('KDC*K1 = 1 (autopilot cancels the aero gain) %s', lab), ...
        abs(A.xkdc*A.xk1 - 1) <= 1e-12, ...
        sprintf('KDC %.6f deg/g, K1 %.5f g/deg, product-1 %.2e', ...
                A.xkdc, A.xk1, A.xkdc*A.xk1 - 1));

    % 4  Integration vs the exact solution of the SAME ODEs.  Heun at wAF*H ~
    %    2.5e-3 predicts a phase error of order w^3*H^2*T/6 ~ 1e-4 rad, so a
    %    band of 1e-4 of the command is about ten times the expected residual
    %    -- a real check with margin, not a restatement of the tolerance.
    [names, oks, det] = add(names, oks, det, ...
        sprintf('nL(t) matches the exact step response %s', lab), ...
        C(k).cf_err <= 1e-4*XNCG, ...
        sprintf('max |nL_sim - nL_exact| %.3e g (%.2e of command)', ...
                C(k).cf_err, C(k).cf_err/XNCG));

    % 5  NON-MINIMUM PHASE: nL(0+) = -nc*(wAF/wz)^2, opposite in sign to the
    %    command, from the RHP zero at +wz.  Flip the sign of -EDD/wz^2 and the
    %    response still settles on 10 g and still rings at the right frequency
    %    -- wrong, with no symptom.
    want = -XNCG*(A.waf/A.wz)^2;
    [names, oks, det] = add(names, oks, det, ...
        sprintf('nL(0+) = -nc*(wAF/wz)^2, RHP zero kicks first %s', lab), ...
        C(k).nl0 < 0 && abs(C(k).nl0 - want) <= 1e-10*abs(want), ...
        sprintf('nL(0+) %.5f g, predicted %.5f g', C(k).nl0, want));
end

K = struct('names',{names}, 'oks',logical(oks(:)).', 'details',{det}, ...
           'all_ok',all(logical(oks)), 'first_bad','');
if ~K.all_ok
    b = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{b}, det{b});
end
end

function [names, oks, det] = add(names, oks, det, name, ok, detail)
names{end+1} = name;  oks(end+1) = logical(ok);  det{end+1} = detail;
end


% =============================================================================
function B = benchmarks(C, VM, XNCG)
%BENCHMARKS  Every book value was grepped out of zarchan.txt; the line number
%   is in the row's .source.  Nothing was read off a figure.
S221 = 'Table 22.1, p.502 (zarchan.txt:6934-6935)';
SSS  = 'prose, pp.500/503 (zarchan.txt:6869-6874, :6928, :6930)';
S223 = 'prose about Fig. 22.3, p.503 (zarchan.txt:6930)';
S224 = 'prose about Fig. 22.4, p.503 (zarchan.txt:6945)';
S226 = 'prose about Fig. 22.6, p.506 (zarchan.txt:6972)';

B = emptyBench();
onBook = abs(VM - 3000) < 1e-9 && abs(XNCG - 10) < 1e-9;
offWhy = sprintf(['Table 22.1 is published for VM = 3000 ft/s and a 10-g ' ...
    'command at 0 and 50000 ft; this run has VM = %g, nc = %g g, alt = %s'], ...
    VM, XNCG, mat2str([C.alt]));

% BANDS ARE SET BY THE BOOK'S OWN ROUNDING: "10.0 rad/s" is 0.5% wide and
% "0.027" is 1.9% wide before we compute anything.
spec = {0, 25.3, 0.058, 'sea level'; 50000, 10.0, 0.027, '50 kft'};
for r = 1:size(spec,1)
    altv = spec{r,1};  bw = spec{r,2};  bz = spec{r,3};  lab = spec{r,4};
    k = find(abs([C.alt] - altv) < 1e-9, 1);
    if isempty(k) || ~onBook
        B = addSkip(B, sprintf('wAF measured from the ringing, %s', lab), bw, 'rad/s', S221, offWhy);
        B = addSkip(B, sprintf('zAF measured by log decrement, %s', lab), bz, '-', S221, offWhy);
        B = addSkip(B, sprintf('steady-state nL, %s', lab), XNCG, 'g', SSS, offWhy);
        continue
    end
    c = C(k);
    B = addVal(B, sprintf('wAF measured from the ringing, %s', lab), bw, c.wn, ...
        'book rounding, 3 s.f.', 0.015, S221, 'rad/s', ...
        sprintf(['zero-crossing spacing over %d crossings; the ALGEBRAIC wAF ' ...
                 'is %.4f rad/s, scored in airframeLinear'], c.ncross, c.aero.waf));
    B = addVal(B, sprintf('zAF measured by log decrement, %s', lab), bz, c.zeta, ...
        'book rounding, 2 s.f.', 0.05, S221, '-', ...
        sprintf('algebraic zAF %.6f; envelope time constant 1/(z*w) = %.3f s', ...
                c.aero.zaf, 1/(c.aero.zaf*c.aero.waf)));
    if c.settled
        B = addVal(B, sprintf('steady-state nL, %s', lab), XNCG, c.nl_ss, ...
            'settling', 0.005, SSS, 'g', ...
            sprintf(['mean over the last two damped periods of a %.2f s run; ' ...
                     'KDC = 1/K1 = %.6f deg/g is the whole autopilot'], ...
                     c.tstop, c.aero.xkdc));
    else
        B = addSkip(B, sprintf('steady-state nL, %s', lab), XNCG, 'g', SSS, ...
            sprintf(['the run is %.3f s, under six envelope time constants ' ...
                     '(1/(z*w) = %.3f s), so the endpoint is a point on a ' ...
                     '%.1f%% ring, not a steady state.  Widening the band to ' ...
                     'absorb that would be scoring the ring.'], c.tstop, ...
                     1/(c.aero.zaf*c.aero.waf), ...
                     100*exp(-c.aero.zaf*c.aero.waf*c.tstop)));
    end
end

% SHAPE -- Fig. 22.3, sea level, inside the book's own 2-s plot window.
k0 = find(abs([C.alt]) < 1e-9, 1);
if isempty(k0) || ~onBook || ~isfinite(C(k0).peak2s)
    B = addSkip(B, 'Fig. 22.3: sea-level response is oscillatory', NaN, '-', S223, ...
        ['the sea-level 10-g case was not run, or no sample landed inside ' ...
         'the book''s 2-s plot window']);
else
    c = C(k0);
    B = addShape(B, 'Fig. 22.3: sea-level response is oscillatory', ...
        'peak > 1.5x command AND >= 6 crossings of it within 2 s', ...
        sprintf('peak %.3f g (%.2fx), %d crossings', c.peak2s, c.peak2s/XNCG, c.ncross2s), ...
        c.peak2s > 1.5*XNCG && c.ncross2s >= 6, S223, ...
        ['"the flight-control system response due to a 10-g step at sea ' ...
         'level is oscillatory due to the low damping of the airframe"']);
end

% SHAPE -- Fig. 22.4, both conditions.
k5 = find(abs([C.alt] - 50000) < 1e-9, 1);
if isempty(k0) || isempty(k5) || ~onBook
    B = addSkip(B, 'Fig. 22.4: altitude lowers BOTH damping and frequency', NaN, ...
        '-', S224, 'both book conditions are needed and were not both run');
else
    z0 = C(k0).zeta;  z5 = C(k5).zeta;  w0 = C(k0).wn;  w5 = C(k5).wn;
    B = addShape(B, 'Fig. 22.4: altitude lowers BOTH damping and frequency', ...
        'zeta(50 kft) < zeta(SL) AND wn(50 kft) < wn(SL)', ...
        sprintf('zeta %.5f < %.5f, wn %.3f < %.3f rad/s', z5, z0, w5, w0), ...
        all(isfinite([z0 z5 w0 w5])) && z5 < z0 && w5 < w0, S224, ...
        'both measured off the responses, not taken from the aero');
end

% SKIP -- Fig. 22.6's radome-slope band.  See the header.
B = addSkip(B, 'Fig. 22.6 radome-slope tolerance band', NaN, '-', S226, ...
    ['the published band (rms miss blows up below -0.005 or above +0.04 at ' ...
     'zAF = 0.027, and only below -0.03 / above +0.06 at zAF = 0.7) is the ' ...
     'output of an ADJOINT of the Fig. 22.5 homing loop -- seeker lag, noise ' ...
     'filter, radome slope, Vc = 4000 ft/s, N'' = 3, tF = 10 s from Table ' ...
     '22.2.  That program is not Listing 22.1 and is printed nowhere in the ' ...
     'book, so reproducing it would mean inventing the loop.']);

% INFO -- the trim solution; the chapter publishes no number for the roots.
if ~isempty(k0)
    a0 = C(k0).aero;
    B = addInfo(B, 'trim solution at the 10-g command, sea level', 'not published', ...
        sprintf('alpha %.5f deg, delta %.5f deg, CN_trim %.5f', ...
                a0.alfa_trim, a0.del_trim, a0.cn_trim), ...
        'derivation, p.501 (zarchan.txt:6920-6927)', ...
        ['the chapter derives the quadratic but Table 22.1 lists only the ' ...
         'transfer-function parameters, so there is nothing to score against']);
end
end


% =============================================================================
%   ROW CONSTRUCTION.  ONE constructor, so the field ORDER cannot drift: a
%   struct array whose fields reordered makes [B.ok] mean something else and
%   fails with "Subscripted assignment between dissimilar structures", which
%   reads like a typo rather than a schema break.
% =============================================================================
function B = emptyBench()
B = struct('what',{}, 'kind',{}, 'book',{}, 'book_str',{}, 'ours',{}, ...
           'ours_str',{}, 'pct',{}, 'band',{}, 'tol',{}, 'ok',{}, ...
           'skipped',{}, 'why',{}, 'source',{}, 'units',{}, 'detail',{});
end

function b = newRow(what, kind, book, book_str, ours, ours_str, pct, band, ...
                    tol, ok, skipped, why, source, units, detail)
b = struct('what',what, 'kind',kind, 'book',book, 'book_str',book_str, ...
           'ours',ours, 'ours_str',ours_str, 'pct',pct, 'band',band, ...
           'tol',tol, 'ok',logical(ok), 'skipped',logical(skipped), ...
           'why',why, 'source',source, 'units',units, 'detail',detail);
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
if ~isfinite(ours) || book == 0
    B(end+1) = newRow(what, 'value', book, sprintf('%.4g',book), ours, ...
        'not finite', NaN, band, tol, false, true, ...
        'the computed value is not finite (or the book value is zero)', ...
        source, units, detail);
    return
end
B(end+1) = newRow(what, 'value', book, sprintf('%.4g',book), ours, ...
    sprintf('%.5g',ours), 100*(ours/book - 1), band, tol, ...
    abs(ours/book - 1) <= tol, false, '', source, units, detail);
end

function B = addShape(B, what, bookStr, oursStr, ok, source, detail)
B(end+1) = newRow(what, 'shape', NaN, bookStr, NaN, oursStr, NaN, 'shape', ...
    NaN, ok, false, '', source, '-', detail);
end

function B = addSkip(B, what, book, units, source, why)
%ADDSKIP  A check that COULD NOT RUN.  Never a pass: it counts as a check that
%   did not happen, which is a different thing from one that succeeded.
if isfinite(book), bs = sprintf('%.4g', book); else, bs = 'n/a'; end
B(end+1) = newRow(what, 'skip', book, bs, NaN, '--', NaN, 'skip', NaN, ...
    false, true, why, source, units, why);
end

function B = addInfo(B, what, bookStr, oursStr, source, why)
%ADDINFO  Reported, never adjudicated.  skipped = true keeps it out of the pass
%   count in BOTH directions: an INFO row can neither pass nor fail.
B(end+1) = newRow(what, 'info', NaN, bookStr, NaN, oursStr, NaN, 'info', NaN, ...
    false, true, why, source, '-', why);
end


% =============================================================================
function s = render(R, ev)
L = {'mgs.zarchan.fcsOpenLoop -- Listing 22.1, OPEN-LOOP FLIGHT-CONTROL SYSTEM'};
L{end+1} = sprintf('units: %s', R.units);
L{end+1} = sprintf('VM %g ft/s, command %g g, H %g s, record every %g s', ...
    R.p.VM, R.p.XNCG, R.p.H, R.p.samp);
L{end+1} = '   alt(ft)     wAF      zAF       wz  K1(g/deg) KDC(deg/g) del(deg)  nL(0+)    peak   nL_ss  tstop';
for k = 1:numel(R.cond)
    c = R.cond(k);  A = c.aero;
    L{end+1} = sprintf('%10.0f %7.3f %8.5f %8.3f %10.4f %10.5f %8.4f %7.3f %7.3f %7.4f %6.2f', ...
        A.alt, A.waf, A.zaf, A.wz, A.xk1, A.xkdc, A.del_deg, c.nl0, ...
        c.peak, c.nl_ss, c.tstop); %#ok<AGROW>
end
L{end+1} = 'MEASURED off the response (the algebraic route lives in airframeLinear):';
for k = 1:numel(R.cond)
    c = R.cond(k);
    L{end+1} = sprintf('%10.0f ft: wn %.4f rad/s, zeta %.6f  (algebraic %.4f / %.6f)', ...
        c.alt, c.wn, c.zeta, c.aero.waf, c.aero.zaf); %#ok<AGROW>
end
if isempty(R.bench)
    L{end+1} = 'BENCHMARKS: not run (''benchmark'' false) -- R.pass = false means NOT CHECKED.';
else
    L{end+1} = sprintf('BENCHMARKS (%d rows, %d evaluated):', numel(R.bench), sum(ev));
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
            case 'skip', tag = 'SKIP';
            case 'info', tag = 'INFO';
            otherwise,   tag = verd(b.ok);
        end
        if strcmpi(b.kind,'value') && ~b.skipped
            L{end+1} = sprintf('  [%s] %-46s book %-8s ours %-9s (%+.2f%%)', ...
                tag, b.what, b.book_str, b.ours_str, b.pct); %#ok<AGROW>
        else
            L{end+1} = sprintf('  [%s] %-46s %s', tag, b.what, b.ours_str); %#ok<AGROW>
        end
    end
end
L{end+1} = sprintf('STRUCTURAL: %d/%d hold', sum(R.selfcheck.oks), numel(R.selfcheck.oks));
for i = 1:numel(R.selfcheck.names)
    L{end+1} = sprintf('  [%s] %s -- %s', verd(R.selfcheck.oks(i)), ...
        R.selfcheck.names{i}, R.selfcheck.details{i}); %#ok<AGROW>
end
s = strjoin(L, newline);
end

function s = verd(ok)
if ok, s = ' ok '; else, s = 'FAIL'; end
end


% =============================================================================
function drawCurves(C, XNCG)
figure;  hold on;  grid on;
tend = 0;
for k = 1:numel(C)
    plot(C(k).t, C(k).nl, 'DisplayName', sprintf('%g ft', C(k).alt));
    tend = max(tend, C(k).t(end));
end
plot([0 tend], [XNCG XNCG], 'k--', 'DisplayName', 'command');
xlabel('Time (Sec)');                       % the listing's own labels
ylabel('Missile Acceleration (G)');
legend('show');
title('Listing 22.1 -- open-loop flight-control system');
end
