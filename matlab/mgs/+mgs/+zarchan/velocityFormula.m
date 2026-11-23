function R = velocityFormula(opts)
%MGS.ZARCHAN.VELOCITYFORMULA  Zarchan Listing 11.4 -- simulation demonstrating
%   the validity of the closed-form ballistic VELOCITY FORMULA.
%
%   R = mgs.zarchan.velocityFormula()                 the book's nominal case
%   R = mgs.zarchan.velocityFormula('DISTNM', 5000, 'GAMDEG', 20)
%   R = mgs.zarchan.velocityFormula('assert', false)  warn instead of error
%
%   Line-by-line port of Listing 11.4 (zarchan.txt:3477-3499).  The listing has
%   ONE job: take the closed form for required launch speed (zarchan.txt:3515-
%   3518),
%
%       V = sqrt(  gm(1-cos phi) / ( r0 cos g [ (r0 cos g)/a - cos(phi+g) ] )  )
%
%   fly the resulting state through the Earth-centred inverse-square equations
%   of motion, and see whether the missile lands where the formula said it
%   would.  So THAT agreement is the headline benchmark row, and the other rows
%   exist to say which half is at fault when it moves.
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft^3/s^2 for GM, SECONDS.  Distances are
%   REPORTED in n.mi. at the listing's own 6076 ft/n.mi.; angles in and out are
%   degrees converted by the listing's own 57.3 (NOT 180/pi).  Nothing is
%   converted -- the rest of mgs is metres, so convert AT THE CALL SITE.
% ===========================================================================
%
%   THE TRANSCRIPTION.  listing_11_04.m reports no unresolved comparisons and
%   that is right: both of them are against literals.  Three forced repairs:
%     1  "while ALT .=0."  -> "while ALT >= 0".  "." is this book's OCR of ">".
%        ALT starts at 0 and rises, so "<=" would never enter the loop.
%     2  "while STEP ,=1"  -> "while STEP <= 1".  "," is the OCR of "<"; the
%        body sets STEP=2 to leave, which only makes sense against "<=".
%     3  "if S.=9.99999"   -> "if S >= 9.99999".  S resets to 0 on every record,
%        so "<=" would record forever.  UNLIKE Listing 4.6 the literal is FAR
%        ABOVE one increment (9.99999 vs H = 0.01), so it genuinely means a 10 s
%        record and is kept verbatim as 'samp'.  The trailing plot/save block is
%        kept only as the optional 'plot'; "/ascii" is the book's typo for
%        -ascii and nothing is written to disk here at all.
%
%   *** THE TRAP IN THIS LISTING. ***  ALT is recomputed ONLY inside the 10 s
%   record block, so "while ALT >= 0" exits up to one full sample LATE: at the
%   nominal case the last recorded row is ALT = -11.9 n.mi. at DISTNM = 6028.0.
%   Reading ArrayDISTNM(end) as the impact range is wrong by ~28 n.mi. -- a 0.5%
%   error that looks entirely plausible.  This port keeps the loop and the
%   record exactly as written and ADDS a step-resolution altitude-crossing
%   diagnostic (R.impact_distnm, R.impact_tF_s), which is what the benchmarks
%   score.  R.last_recorded_distnm leaves the trap visible.
%
%   TWO BOOK EQUATIONS RECONSTRUCTED, their extractions being heavily garbled:
%     * FLIGHT TIME, zarchan.txt:3548-3559.  The denominator bracket of its
%       first term IS the trajectory equation and equals r0/rF -- selfcheck 1
%       verifies that to 1e-9, and that is the evidence the reconstruction is
%       the book's.  The printed "tan^-1" is on the WRONG principal branch once
%       cos(g)cot(phi/2) - sin(g) turns negative (above ~42.5 deg at 6000
%       n.mi.), where plain ATAN returns NEGATIVE flight times; ATAN2 is used,
%       forced by continuity and by tF > 0.  At the nominal case that
%       denominator is positive and atan2 == atan, so the headline number is
%       untouched by the repair.
%     * ESCAPE-VELOCITY FLIGHT-PATH ANGLE BOUNDS, zarchan.txt:3926-3934.  Used
%       only where r0 = rF, the only place the extraction's ambiguous
%       "2 r0 rF" ratio has been verified.
%
%   BENCHMARKS -- every book value was grepped from zarchan.txt and its line is
%   in the row's .source.  Nothing is read off a figure (the extraction turned
%   every figure into an image link); prose CLAIMS about figures are scored as
%   shape/ordering rows, which is the stronger check anyway.  Band labels print
%   as 'text' (a prose or caption claim), 'analytic' (a closed form with no
%   integrator on one side) and 'identity' (algebra that must hold to machine
%   precision); shape and bound rows carry no tolerance and none is possible.
%     1  impact downrange == commanded DISTNM (:3520-3522 and the Fig. 11.15
%        caption, "Closed-form solution for velocity is accurate").  Band 0.1%,
%        expected residual +0.00009%: the only error source is RK2, and the
%        range moves 1.5e-6 relative between H = 0.01 and H = 1.
%     2  peak altitude "in excess of 800 n.mi." (:3521), nominal case only.
%     3  simulated flight time vs the book's closed form (:3548-3559).  0.05%.
%     4  lambda at gamma_max == 2 (:3924-3934), band 1e-9: Ch.12's gamma_max fed
%        back into Ch.11's velocity formula must give escape velocity exactly.
%     5  shape (:3519): required velocity rises with distance.
%     6  shape (:3560): flight time rises monotonically with flight-path angle.
%
%   NOT SCORED, and why (info/skip rows):
%     * "about 1800 s at 5000 n.mi., 20 deg" and "nearly 2800 s at 40 deg"
%       (:3560) are the author READING HIS OWN Fig. 11.17.  The book's closed
%       form, printed nine lines above at :3548-3559, gives 1694.70 s and
%       2730.18 s -- 5.9% and 2.5% away.  Scoring either would be scoring a
%       figure reading, so both lines are cited on an info row and the ORDERING
%       the sentence asserts is scored instead, as row 6.
%     * The required velocity.  Fig. 11.14 plots the formula but no number for
%       V appears in the prose, so there is nothing to compare 24184.8 against.
%
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED -- this file has never
%   been executed; MATLAB is the caller's. ***  From an independent scratch
%   implementation of the same equations:
%     V = 24184.78 ft/s, lambda = 0.869481, 220 recorded samples
%     impact 6000.0055 n.mi. (+0.00009%) at 2192.378 s; closed form 2192.386 s
%     apogee 847.63 n.mi.;  gamma_min/max = -24.954 / +65.046 deg
%     energy drift ~1.9e-11, angular-momentum drift ~1.2e-12
%     probe (+0.3% on V) moves the impact range to 6077.74 n.mi., +1.30%
%   A residual near a band EDGE is not a pass: these sit 2-6 orders inside.
%
%   OUTPUTS
%     R.V R.lambda R.phi_rad R.gam_rad R.r0_ft      the launch condition
%     R.t R.altnm R.distnm R.xnm R.ynm              the listing's 10 s record
%     R.impact_distnm R.impact_tF_s R.peak_altnm    step-resolution diagnostics
%     R.last_recorded_distnm                        the trap above, left visible
%     R.tF_cf_s R.gam_min_deg R.gam_max_deg         reconstructed book forms
%     R.bench R.selfcheck R.pass R.verdict R.p R.units R.summary
%
%   See also MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 11.4 lines 1-2, verbatim.
    opts.DISTNM (1,1) double {mustBeFinite, mustBePositive}    = 6000      % n.mi.
    opts.GAMDEG (1,1) double {mustBeFinite}                    = 23        % deg
    opts.ALTNM  (1,1) double {mustBeFinite, mustBeNonnegative} = 0         % n.mi.
    opts.ANGDEG (1,1) double {mustBeFinite}                    = 0         % deg
    opts.H      (1,1) double {mustBeFinite, mustBePositive}    = 0.01      % s
    opts.samp   (1,1) double {mustBeFinite, mustBePositive}    = 9.99999   % s
    opts.A      (1,1) double {mustBeFinite, mustBePositive}    = 2.0926e7  % ft
    opts.GM     (1,1) double {mustBeFinite, mustBePositive}    = 1.4077e16 % ft^3/s^2
    % Runaway guard.  A lambda close to 2 gives an almost-parabolic arc whose
    % flight time diverges; the listing's "while ALT >= 0" would then spin
    % forever, silently, which is the one failure this file must not have.
    opts.tmax   (1,1) double {mustBeFinite, mustBePositive}    = 1e5       % s
    % Selfcheck 5: fractional error injected into V.  0.003 is chosen so the
    % resulting range error clears 3x the benchmark band at every case tried
    % (d(range)/range is 2.7-6.1 times d(V)/V over 3000-8000 n.mi.), not so
    % that the nominal case alone squeaks past.
    opts.probe  (1,1) double {mustBeFinite, mustBePositive}    = 0.003
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

% The listing's own constants.  57.3 and 1.5708 are NOT 180/pi and pi/2, and
% the difference is KEPT: 1.5708 biases the launch flight-path angle by
% +3.673e-6 rad, which selfcheck 4 measures rather than assumes.
DEGR   = 57.3;
FTNM   = 6076;
HALFPI = 1.5708;

A  = opts.A;   GM = opts.GM;   H = opts.H;
GAM = opts.GAMDEG/DEGR;
ANG = opts.ANGDEG/DEGR;

if opts.samp < H
    error('mgs:zarchan:velocityFormula:samp', ...
        ['samp = %g s is smaller than the integration step H = %g s, so the ' ...
         'record would fire every step while claiming a %g s interval.'], ...
        opts.samp, H, opts.samp);
end

% ---- the velocity formula itself, zarchan.txt:3515-3518 --------------------
[V, PHI, R0, lam] = vreq(opts.DISTNM, GAM, opts.ALTNM, A, GM, FTNM);
if lam >= 2 - 1e-9
    error('mgs:zarchan:velocityFormula:escape', ...
        ['lambda = r0 V^2/gm = %.6f >= 2: the required launch speed is at or ' ...
         'above escape velocity, the trajectory is not elliptical and never ' ...
         'returns to the surface, so the listing''s "while ALT >= 0" loop ' ...
         'would not terminate.  GAMDEG = %g deg is outside the band ' ...
         '[%.3f, %.3f] deg for %g n.mi. (zarchan.txt:3926-3934).'], ...
        lam, opts.GAMDEG, gammaBound(PHI,-1)*DEGR, gammaBound(PHI,+1)*DEGR, ...
        opts.DISTNM);
end

% =========================================================================
%   LISTING 11.4, zarchan.txt:3477-3499.
% =========================================================================
SIM = runSim(V, GAM, ANG, opts.ALTNM, A, GM, H, opts.samp, opts.tmax, ...
             FTNM, HALFPI);

% Perturbed twin for selfcheck 5.  Run at a COARSER step on purpose: it is a
% sensitivity probe, not a precision run, and the trajectory is insensitive to
% H here (1.5e-6 relative between H = 0.01 and H = 1).
%
% The probe is CAPPED so the perturbed lambda stays below 2.  Without the cap a
% near-escape setting would send the probe on a trajectory that never comes
% back, and the tmax guard would then spend ten million steps discovering that
% -- a self-check quietly turning into the slowest thing in the suite.  At the
% nominal case lambda = 0.869, the cap is 0.516, so 0.003 passes untouched.
probeUsed = min(opts.probe, max(sqrt(1.999/lam) - 1, 0));
if probeUsed <= 0
    error('mgs:zarchan:velocityFormula:probe', ...
        ['lambda = %.6f leaves no room to perturb V without exceeding escape ' ...
         'velocity, so selfcheck 5 cannot run at these settings.'], lam);
end
Hp  = max(H, 0.1);
PRB = runSim(V*(1+probeUsed), GAM, ANG, opts.ALTNM, A, GM, Hp, ...
             max(opts.samp, Hp), opts.tmax, FTNM, HALFPI);

R = struct();
R.V        = V;
R.lambda   = lam;
R.phi_rad  = PHI;
R.gam_rad  = GAM;
R.r0_ft    = R0;
R.t        = SIM.t;
R.altnm    = SIM.altnm;
R.distnm   = SIM.distnm;
R.xnm      = SIM.xnm;
R.ynm      = SIM.ynm;
R.impact_distnm = SIM.impact_distnm;
R.impact_tF_s   = SIM.impact_t;
R.peak_altnm    = SIM.peak_altnm;
R.last_recorded_distnm = SIM.distnm(end);
R.last_recorded_altnm  = SIM.altnm(end);
R.tF_cf_s      = tfClosedForm(PHI, GAM, lam, R0, V);
R.gam_min_deg  = gammaBound(PHI, -1)*DEGR;
R.gam_max_deg  = gammaBound(PHI, +1)*DEGR;
R.probe_distnm = PRB.impact_distnm;
R.units = ['ft, ft/s, ft^3/s^2 (GM); distances reported in n.mi. at ' ...
           '6076 ft/n.mi.; seconds; degrees in/out at the listing''s 57.3'];
R.p = struct('DISTNM',opts.DISTNM, 'GAMDEG',opts.GAMDEG, 'ALTNM',opts.ALTNM, ...
             'ANGDEG',opts.ANGDEG, 'H',H, 'samp',opts.samp, 'A',A, 'GM',GM, ...
             'probe',probeUsed, 'probe_asked',opts.probe, 'probe_H',Hp, ...
             'n_steps',SIM.nstep, 'n_records',numel(SIM.t), 'units',R.units);

R.selfcheck = structuralChecks(SIM, PRB, PHI, GAM, lam, R0, A, ...
                               opts.DISTNM, HALFPI, probeUsed);

if opts.benchmark
    R.bench = benchmarks(R, opts, PHI, GAM, A, GM, FTNM, DEGR);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    R.pass  = false;      % NOT CHECKED must never render as fine.
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot
    figure; plot(R.distnm, R.altnm); grid on
    xlabel('Downrange (Nmi)'); ylabel('Altitude (Nmi)');
    title(sprintf('Listing 11.4: %g n.mi. at %g deg', opts.DISTNM, opts.GAMDEG));
end

% ---- adjudication.  The verdict is FORMED either way; 'assert' picks only the
%      reaction to it.  With assert false the caller still gets R complete, so
%      the failing side stays diagnosable -- benchSuite runs every port that way.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = 'not one book value could be evaluated at these settings.';
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
        error('mgs:zarchan:velocityFormula:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:velocityFormula:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [V, PHI, R0, lam] = vreq(DISTNM, GAM, ALTNM, A, GM, FTNM)
%VREQ  The velocity formula, zarchan.txt:3515-3518, exactly as the listing
%   codes it at zarchan.txt:3484.  GAM arrives in RADIANS: the listing's
%   GAMDEG/57.3 conversion happens once at the call site, so that the
%   gamma_max benchmark can feed this an exact radian value instead of
%   round-tripping through 57.3 and landing on lambda = 1.99913 rather than 2.
%
%   Note the DENOMINATOR uses A, not R0: the target is on the surface, rF = a.
%   The general form (zarchan.txt:3825, :3920) carries rF there.
%
%   All constants are ARGUMENTS.  MATLAB local functions cannot see the main
%   function's workspace, and this package has already lost a wave to that.
PHI  = DISTNM*FTNM/A;
ALT  = ALTNM*FTNM;
R0   = A + ALT;
TOP  = GM*(1 - cos(PHI));
TEMP = R0*cos(GAM)/A - cos(PHI + GAM);
BOT  = R0*cos(GAM)*TEMP;
if ~(BOT > 0) || ~(TOP > 0)
    error('mgs:zarchan:velocityFormula:noSolution', ...
        ['the velocity formula has no real solution here: TOP = %.6g, ' ...
         'BOT = %.6g at phi = %.6f rad, gamma = %.6f rad.  sqrt(TOP/BOT) ' ...
         'would return a COMPLEX speed and the simulation would run on it ' ...
         'without complaint.'], TOP, BOT, PHI, GAM);
end
V   = sqrt(TOP/BOT);
lam = R0*V*V/GM;
end


% =============================================================================
function S = runSim(V, GAM, ANG, ALTNM, A, GM, H, samp, tmax, FTNM, HALFPI)
%RUNSIM  Listing 11.4's integration loop, zarchan.txt:3484-3495, verbatim,
%   with a step-resolution altitude-crossing DIAGNOSTIC added alongside (it
%   reads state, never writes it, so it cannot change the trajectory).

ALT = ALTNM*FTNM;
VRX = V*cos(HALFPI - GAM + ANG);
VRY = V*sin(HALFPI - GAM + ANG);
Sacc = 0;
X  = (A+ALT)*cos(ANG);   Y  = (A+ALT)*sin(ANG);
XFIRST = X;  YFIRST = Y;
X1 = VRX;    Y1 = VRY;   T = 0;
RF = sqrt(XFIRST^2 + YFIRST^2);

% Launch invariants, for the structural checks.
Rr = sqrt(X^2 + Y^2);
e0 = 0.5*(X1^2 + Y1^2) - GM/Rr;
h0 = X*Y1 - Y*X1;
S  = struct();
S.gam0 = atan2((X*X1 + Y*Y1)/Rr, (X*Y1 - Y*X1)/Rr);

nmax = ceil(tmax/samp) + 8;
tA = zeros(nmax,1);  altA = tA;  disA = tA;  xA = tA;  yA = tA;
n  = 0;  nstep = 0;
peak = -Inf;  de = 0;  dh = 0;
tPrev = 0;  aPrev = ALT;  dPrev = 0;
tCross = NaN;  dCross = NaN;  haveCross = false;
XD = 0;  YD = 0;  X1D = 0;  Y1D = 0;

while ALT >= 0
    if T > tmax
        error('mgs:zarchan:velocityFormula:tmax', ...
            ['the missile had not returned to the surface after %g s.  The ' ...
             'listing''s loop condition is "while ALT >= 0" and ALT is only ' ...
             'recomputed at the %g s record, so without this guard the loop ' ...
             'would spin forever without printing anything.'], tmax, samp);
    end
    XOLD = X;  YOLD = Y;  X1OLD = X1;  Y1OLD = Y1;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor.  T advances INSIDE this loop, exactly once,
    % which is what makes the second derivative evaluation happen at t+H.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            X = X + H*XD;   Y = Y + H*YD;
            X1 = X1 + H*X1D;  Y1 = Y1 + H*Y1D;
            T = T + H;
        end
        TEMBOT = (X^2 + Y^2)^1.5;
        X1D = -GM*X/TEMBOT;    % Newton's law of universal gravitation
        Y1D = -GM*Y/TEMBOT;
        XD  = X1;   YD = Y1;
        FLAG = 1;
    end
    % The listing writes "FLAG=0" here too; dropped, the outer pass
    % re-initialises it, so that assignment can never be read.

    % Heun corrector, in the listing's own algebra.
    X  = (XOLD  + X )/2 + 0.5*H*XD;
    Y  = (YOLD  + Y )/2 + 0.5*H*YD;
    X1 = (X1OLD + X1)/2 + 0.5*H*X1D;
    Y1 = (Y1OLD + Y1)/2 + 0.5*H*Y1D;
    nstep = nstep + 1;

    % ---- DIAGNOSTICS.  Not in the listing.  Read-only. --------------------
    Rr  = sqrt(X^2 + Y^2);
    aft = Rr - A;
    if aft > peak, peak = aft; end
    de = max(de, abs((0.5*(X1^2 + Y1^2) - GM/Rr - e0)/e0));
    dh = max(dh, abs((X*Y1 - Y*X1 - h0)/h0));
    cb = (X*XFIRST + Y*YFIRST)/(Rr*RF);
    cb = min(1, max(-1, cb));         % acos of 1+eps is complex, silently
    dnm = A*acos(cb)/FTNM;
    if ~haveCross && aPrev >= 0 && aft < 0
        % Linear in altitude between two steps.  Over H = 0.01 s the arc's
        % departure from a straight line is under 2e-3 ft; the listing's own
        % 10 s record could not resolve the crossing to better than ~28 n.mi.
        f = aPrev/(aPrev - aft);
        tCross = tPrev + f*(T - tPrev);
        dCross = dPrev + f*(dnm - dPrev);
        haveCross = true;
    end
    tPrev = T;  aPrev = aft;  dPrev = dnm;

    % ---- the listing's own record, zarchan.txt:3494 -----------------------
    Sacc = Sacc + H;
    if Sacc >= samp
        Sacc = 0;
        ALT  = sqrt(X^2 + Y^2) - A;
        n = n + 1;
        tA(n) = T;  altA(n) = ALT/FTNM;  disA(n) = dnm;
        xA(n) = X/FTNM;  yA(n) = Y/FTNM;
    end
end

if n < 2
    error('mgs:zarchan:velocityFormula:empty', ...
        ['the run recorded %d sample(s) at samp = %g s.  An empty or ' ...
         'one-point trajectory returned quietly looks like a result.'], n);
end
S.t = tA(1:n);  S.altnm = altA(1:n);  S.distnm = disA(1:n);
S.xnm = xA(1:n); S.ynm = yA(1:n);
S.impact_t      = tCross;
S.impact_distnm = dCross;
S.peak_altnm    = peak/FTNM;
S.de = de;  S.dh = dh;  S.nstep = nstep;
end


% =============================================================================
function tF = tfClosedForm(PHI, GAM, lam, R0, V)
%TFCLOSEDFORM  The book's flight-time closed form, zarchan.txt:3548-3559,
%   reconstructed from a badly garbled extraction (see the header).
%
%       tF = r0/(V cos g) * {  [tan g (1-cos phi) + (1-lam) sin phi]
%                              / ((2-lam) * BRK)
%                            + 2 cos g / (lam (2/lam - 1)^1.5)
%                              * atan2( sqrt(2/lam - 1),
%                                       cos g cot(phi/2) - sin g )  }
%
%   BRK = (1-cos phi)/(lam cos^2 g) + cos(g+phi)/cos g  IS the trajectory
%   equation and equals r0/rF; selfcheck 1 verifies that to machine precision,
%   which is the evidence that this reconstruction is the book's formula.
%   ATAN2, not ATAN: see the header for why the printed principal branch
%   returns negative flight times above ~42.5 deg.
BRK = (1 - cos(PHI))/(lam*cos(GAM)^2) + cos(GAM + PHI)/cos(GAM);
t1  = (tan(GAM)*(1 - cos(PHI)) + (1 - lam)*sin(PHI)) / ((2 - lam)*BRK);
q   = sqrt(2/lam - 1);
t2  = (2*cos(GAM))/(lam*(2/lam - 1)^1.5) ...
      * atan2(q, cos(GAM)/tan(PHI/2) - sin(GAM));
tF  = R0/(V*cos(GAM)) * (t1 + t2);
end


% =============================================================================
function g = gammaBound(PHI, sgn)
%GAMMABOUND  zarchan.txt:3926-3934, the two flight-path angles at which the
%   velocity formula returns exactly escape velocity (lambda = 2):
%
%       gamma = atan( [ sin phi -+ sqrt(2 (r0/rF)(1 - cos phi)) ] / (1-cos phi) )
%
%   sgn = -1 gives gamma_min, +1 gives gamma_max.  r0/rF is taken as 1: this is
%   only ever used for launch from the surface to a target on the surface, and
%   the extraction leaves the ratio's placement inside the radical ambiguous,
%   so it is not claimed anywhere else.
c = 1 - cos(PHI);
g = atan((sin(PHI) + sgn*sqrt(2*c))/c);
end


% =============================================================================
function K = structuralChecks(SIM, PRB, PHI, GAM, lam, R0, A, DISTNM, ...
                              HALFPI, probe)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never evidence that
%   this port reproduces Zarchan -- only that it agrees with itself, which is
%   what catches wiring faults.
%
%   .names and .oks are what benchSuite puts on the board.  Both, same length,
%   or the invariants below are invisible and the board looks green anyway.
K = struct();
names = {};  oks = [];  detail = {};

% 1  The trajectory equation closes: BRK == r0/rF.  This validates the velocity
%    formula ALGEBRAICALLY, with no integrator involved, and it is also the
%    evidence that the reconstructed flight-time formula is the book's.
BRK  = (1 - cos(PHI))/(lam*cos(GAM)^2) + cos(GAM + PHI)/cos(GAM);
K.brk_resid = abs(BRK - R0/A)/(R0/A);
names{end+1}  = 'trajectory equation closes (BRK == r0/rF)';
oks(end+1)    = K.brk_resid <= 1e-9;
detail{end+1} = sprintf('BRK = %.12f, r0/rF = %.12f, rel %.3e', ...
                        BRK, R0/A, K.brk_resid);

% 2  Specific orbital energy is conserved.  A sign error or a swapped
%    derivative in the inverse-square block moves this immediately.
names{end+1}  = 'specific energy conserved along trajectory';
oks(end+1)    = SIM.de <= 1e-6;
detail{end+1} = sprintf('max relative drift %.3e (expect ~2e-11 at H = 0.01)', SIM.de);

% 3  Angular momentum is conserved.  Catches a sign flip on X1D or Y1D that
%    energy alone would not, because energy is even in the velocity.
names{end+1}  = 'angular momentum conserved along trajectory';
oks(end+1)    = SIM.dh <= 1e-7;
detail{end+1} = sprintf('max relative drift %.3e (expect ~1e-12 at H = 0.01)', SIM.dh);

% 4  The launch rotation is right.  VRX/VRY use the listing's 1.5708, so the
%    flight-path angle actually flown is GAM + (pi/2 - 1.5708) = GAM + 3.673e-6
%    rad, NOT GAM.  Checking against that exact expression both verifies the
%    rotation and pins the size of the book's own rounding, instead of hiding
%    it inside a loose tolerance.
gam_expect = GAM + (pi/2 - HALFPI);
K.gam0_resid = abs(SIM.gam0 - gam_expect);
names{end+1}  = 'launch flight-path angle == GAM + (pi/2 - 1.5708)';
oks(end+1)    = K.gam0_resid <= 1e-10;
detail{end+1} = sprintf(['flown %.9f rad vs %.9f rad; the book''s 1.5708 ' ...
                         'costs %+.3e rad'], SIM.gam0, gam_expect, pi/2 - HALFPI);

% 5  PERTURBATION EXPERIMENT.  The point of benchmark 1 is that the impact
%    range is a SHARP test of the velocity formula.  If it were not -- if any
%    old speed landed near DISTNM -- the row would be decoration.  So inject a
%    known error into V and require the range to move by at least 3x the
%    benchmark band.  Expect +1.30% at the nominal case against a 0.10% band.
K.probe_shift = abs(PRB.impact_distnm - SIM.impact_distnm)/DISTNM;
names{end+1}  = 'probe on V must break the impact-range row';
oks(end+1)    = K.probe_shift >= 3*1e-3;
detail{end+1} = sprintf(['V*(1+%.4g) moves the impact range %+.4f n.mi. ' ...
                         '(%.4f%% of DISTNM) vs a %.2f%% band'], probe, ...
                        PRB.impact_distnm - SIM.impact_distnm, ...
                        100*K.probe_shift, 100*1e-3);

K.names = names;
K.oks   = logical(oks(:).');
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
function B = benchmarks(R, opts, PHI, GAM, A, GM, FTNM, DEGR)
%BENCHMARKS  Computed vs published.  Every book value was grepped from
%   zarchan.txt and its line number is in the row's .source.  Bands are set by
%   the PROVENANCE of the book value, never by what we happen to compute.
B = emptyBench();
nominal = abs(opts.DISTNM - 6000) < 1e-9 && abs(opts.GAMDEG - 23) < 1e-9 ...
          && opts.ALTNM == 0;

% ---- 1.  THE HEADLINE ROW: does it land where the formula said? -----------
B(end+1) = valRow('impact downrange == commanded DISTNM', ...
    opts.DISTNM, R.impact_distnm, 1e-3, 'text', ...
    'p.253 text + Fig. 11.15 caption (zarchan.txt:3520-3522)', 'n.mi.', ...
    ['the whole point of the listing.  Band is 0.1%: the only error source ' ...
     'is RK2 and the range moves 1.5e-6 relative between H = 0.01 and H = 1. ' ...
     'Selfcheck 5 shows a 0.3% error in V moves this 1.3%, so the row bites.']);

% ---- 2.  peak altitude, a BOUND and nothing more --------------------------
if nominal
    B(end+1) = boundRow('peak altitude in excess of 800 n.mi.', ...
        '> 800', sprintf('%.2f', R.peak_altnm), R.peak_altnm > 800, ...
        'p.253 text (zarchan.txt:3521)', 'n.mi.', ...
        ['the book states only a bound, so only a bound is scored -- ' ...
         'Fig. 11.15 publishes no number for the apogee']);
else
    B(end+1) = skipRow('peak altitude in excess of 800 n.mi.', '> 800', ...
        sprintf(['quoted for the listing''s own case (6000 n.mi., 23 deg, ' ...
                 'sea level); this run used %g n.mi., %g deg, %g n.mi.'], ...
                opts.DISTNM, opts.GAMDEG, opts.ALTNM), ...
        'p.253 text (zarchan.txt:3521)', 'n.mi.');
end

% ---- 3.  simulation vs the book's closed-form flight time -----------------
B(end+1) = valRow('flight time: simulation vs book closed form', ...
    R.tF_cf_s, R.impact_tF_s, 5e-4, 'analytic', ...
    'p.255 closed form (zarchan.txt:3548-3559)', 's', ...
    ['the second half of "the closed form is accurate": range AND time. ' ...
     'The formula is reconstructed from a garbled extraction; selfcheck 1 ' ...
     '(BRK == r0/rF to 1e-9) is the evidence the reconstruction is right.']);

% ---- 4.  cross-chapter identity: lambda(gamma_max) == 2 -------------------
% Feeds vreq RADIANS directly.  Round-tripping gamma_max through the listing's
% 57.3 instead lands on 1.99913 -- a 0.04% error that looks like a tolerance
% question and is really a units question.
if opts.ALTNM == 0
    gmax = gammaBound(PHI, +1);
    [~, ~, ~, lamMax] = vreq(opts.DISTNM, gmax, opts.ALTNM, A, GM, FTNM);
    B(end+1) = valRow('lambda at gamma_max == 2 (escape)', 2, lamMax, 1e-9, ...
        'identity', 'p.281 (zarchan.txt:3924-3934)', '-', ...
        sprintf(['pure formula, no integrator: Ch.12''s gamma_max fed back ' ...
                 'into Ch.11''s velocity formula must give escape velocity. ' ...
                 'gamma in [%.3f, %.3f] deg here.'], ...
                gammaBound(PHI,-1)*DEGR, gmax*DEGR));
else
    B(end+1) = skipRow('lambda at gamma_max == 2 (escape)', '2', ...
        ['the gamma_max extraction leaves the r0/rF ratio inside the radical ' ...
         'ambiguous, and it has only been verified where r0 = rF; this run ' ...
         'launches from altitude'], 'p.281 (zarchan.txt:3924-3934)', '-');
end

% ---- 5.  shape: longer distances need larger velocities -------------------
dsw = linspace(1000, 9000, 17);
vsw = nan(size(dsw));
% gammaBound assumes r0 = rF, so off the surface the window it returns is only
% approximate and vreq can still refuse a point inside it.  A SHAPE row must
% never be able to kill the port, so a refused point becomes a missing sample
% (and 'good' below reports how many survived) rather than an error.
for i = 1:numel(dsw)
    ph = dsw(i)*FTNM/A;
    if GAM > gammaBound(ph,-1) + 0.02 && GAM < gammaBound(ph,+1) - 0.02
        try
            vsw(i) = vreq(dsw(i), GAM, opts.ALTNM, A, GM, FTNM);
        catch
            vsw(i) = NaN;
        end
    end
end
good = ~isnan(vsw);
okV  = sum(good) >= 5 && all(diff(vsw(good)) > 0);
% NaN rather than [] when nothing is usable: sprintf silently swallows an
% empty argument and then reuses the NEXT one for that placeholder, which
% would print a confident, wrong sentence on the one row that already failed.
vlo = firstOr(min(vsw(good)));  vhi = firstOr(max(vsw(good)));
B(end+1) = shapeRow('required velocity rises with distance', ...
    'monotonically increasing', ...
    sprintf('%.0f -> %.0f ft/s over %d of %d sampled ranges', ...
            vlo, vhi, sum(good), numel(dsw)), okV, ...
    'p.253 text (zarchan.txt:3519)', ...
    ['Fig. 11.14 itself did not survive extraction; the prose CLAIM about ' ...
     'it did, and an ordering check is the stronger test anyway']);

% ---- 6.  shape: flight time rises monotonically with flight-path angle ----
glo = max(1/DEGR, gammaBound(PHI,-1) + 2/DEGR);
ghi = gammaBound(PHI,+1) - 5/DEGR;
gsw = linspace(glo, ghi, 25);
tsw = nan(size(gsw));
for i = 1:numel(gsw)
    try
        [Vi, ~, R0i, lami] = vreq(opts.DISTNM, gsw(i), opts.ALTNM, A, GM, FTNM);
    catch
        continue                     % same reason as the sweep above
    end
    if lami < 2 - 1e-9
        tsw(i) = tfClosedForm(PHI, gsw(i), lami, R0i, Vi);
    end
end
gd  = ~isnan(tsw);
okT = sum(gd) >= 10 && all(diff(tsw(gd)) > 0);
B(end+1) = shapeRow('flight time rises monotonically with flight-path angle', ...
    'monotonically increasing', ...
    sprintf('%.0f -> %.0f s over %.1f..%.1f deg', firstOr(min(tsw(gd))), ...
            firstOr(max(tsw(gd))), glo*DEGR, ghi*DEGR), okT, ...
    'p.255 text (zarchan.txt:3560)', ...
    ['the same sentence that supplies the two unscoreable Fig. 11.17 ' ...
     'readings below; the ORDERING it asserts is scoreable and is scored']);

% ---- INFO: the two Fig. 11.17 readings the book contradicts itself on -----
[V20, ~, R20, l20] = vreq(5000, 20/DEGR, 0, A, GM, FTNM);
[V40, ~, R40, l40] = vreq(5000, 40/DEGR, 0, A, GM, FTNM);
p5 = 5000*FTNM/A;
t20 = tfClosedForm(p5, 20/DEGR, l20, R20, V20);
t40 = tfClosedForm(p5, 40/DEGR, l40, R40, V40);
B(end+1) = infoRow('Fig. 11.17 readings at 5000 n.mi.', ...
    'about 1800 s at 20 deg / nearly 2800 s at 40 deg', ...
    sprintf('closed form gives %.1f s (%+.1f%%) / %.1f s (%+.1f%%)', ...
            t20, 100*(t20/1800-1), t40, 100*(t40/2800-1)), ...
    'p.255 text (zarchan.txt:3560) vs the formula at :3548-3559', ...
    ['the book disagrees with itself: :3560 is the author reading his own ' ...
     'Fig. 11.17, and the formula printed nine lines above at :3548-3559 ' ...
     'gives 5.9% and 2.5% less.  Scoring either number would be scoring a ' ...
     'figure reading, so the sentence is scored as the ordering row above.']);

% ---- SKIP: the required velocity has no published number ------------------
B(end+1) = skipRow('required launch velocity', 'Fig. 11.14 only', ...
    sprintf(['Fig. 11.14 plots the velocity formula but no number for V ' ...
             'appears anywhere in the prose, so there is nothing to compare ' ...
             'our %.1f ft/s against.  Every figure became an image link in ' ...
             'the extraction; a "figure reading" here would be fabrication.'], ...
            R.V), 'p.252-253 Fig. 11.14 (zarchan.txt:3509, :3519)', 'ft/s');
end


% =============================================================================
%   ROW CONSTRUCTION.  Every row goes through mkrow, so the field order is
%   defined in exactly ONE place -- benchSuite's adaptGeneric is strict and a
%   struct array that quietly reordered fields would make [B.ok] mean something
%   else.
% =============================================================================
function B = emptyBench()
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function b = mkrow(kind, what, book, book_str, ours, ours_str, pct, band, ...
                   tol, ok, skipped, why, source, units, detail)
b = struct('what', what, 'kind', kind, 'book', book, 'book_str', book_str, ...
           'ours', ours, 'ours_str', ours_str, 'pct', pct, 'band', band, ...
           'tol', tol, 'ok', logical(ok), 'skipped', logical(skipped), ...
           'why', why, 'source', source, 'units', units, 'detail', detail);
end

function b = valRow(what, book, ours, tol, band, source, units, detail)
if ~isfinite(ours) || ~isfinite(book) || book == 0
    b = mkrow('value', what, book, sprintf('%.6g', book), ours, ...
              sprintf('%.6g', ours), NaN, band, tol, false, true, ...
              'the computed value is not finite (or the book value is zero)', ...
              source, units, detail);
    return
end
b = mkrow('value', what, book, sprintf('%.6g', book), ours, ...
          sprintf('%.7g', ours), 100*(ours/book - 1), band, tol, ...
          abs(ours/book - 1) <= tol, false, '', source, units, detail);
end

function b = boundRow(what, book_str, ours_str, ok, source, units, detail)
b = mkrow('bound', what, NaN, book_str, NaN, ours_str, NaN, 'bound', NaN, ...
          ok, false, '', source, units, detail);
end

function b = shapeRow(what, book_str, ours_str, ok, source, detail)
b = mkrow('shape', what, NaN, book_str, NaN, ours_str, NaN, 'shape', NaN, ...
          ok, false, '', source, '-', detail);
end

function b = infoRow(what, book_str, ours_str, source, why)
% Reported, never scored.  skipped = true keeps it out of the pass count in
% BOTH directions: an INFO row can neither pass nor fail.
b = mkrow('info', what, NaN, book_str, NaN, ours_str, NaN, 'info', NaN, ...
          false, true, why, source, '-', '');
end

function b = skipRow(what, book_str, why, source, units)
% A check that COULD NOT RUN.  Never a pass: "did not happen" is a different
% thing from "succeeded".
b = mkrow('skip', what, NaN, book_str, NaN, '--', NaN, 'skip', NaN, ...
          false, true, why, source, units, '');
end


% =============================================================================
function s = render(R, ev)
L = {};
p = R.p;
L{end+1} = 'Zarchan Listing 11.4 -- VALIDITY OF THE BALLISTIC VELOCITY FORMULA';
L{end+1} = sprintf('*** UNITS: %s ***', R.units);
L{end+1} = sprintf(['   DISTNM = %g n.mi.  GAMDEG = %g deg  ALTNM = %g n.mi.  ' ...
    'H = %g s  samp = %g s'], p.DISTNM, p.GAMDEG, p.ALTNM, p.H, p.samp);
L{end+1} = sprintf(['   V = %.4f ft/s   lambda = %.6f   gamma band ' ...
    '[%.3f, %.3f] deg   %d steps, %d records'], R.V, R.lambda, ...
    R.gam_min_deg, R.gam_max_deg, p.n_steps, p.n_records);
L{end+1} = sprintf(['   impact %.4f n.mi. at %.3f s   apogee %.3f n.mi.   ' ...
    'closed-form tF %.4f s'], R.impact_distnm, R.impact_tF_s, ...
    R.peak_altnm, R.tF_cf_s);
L{end+1} = sprintf(['   (last RECORDED row is %.2f n.mi. at ALT %.2f n.mi. ' ...
    '-- the 10 s record overshoots impact; do not read it as the range)'], ...
    R.last_recorded_distnm, R.last_recorded_altnm);
L{end+1} = '';
L{end+1} = 'BENCHMARKS vs the book';
for i = 1:numel(R.bench)
    b = R.bench(i);
    switch b.kind
        case 'skip'
            L{end+1} = sprintf('   SKIP %-52s %s', b.what, b.book_str); %#ok<AGROW>
            L{end+1} = sprintf('        why: %s', b.why);               %#ok<AGROW>
        case 'info'
            L{end+1} = sprintf('   INFO %-52s %s', b.what, b.ours_str); %#ok<AGROW>
            L{end+1} = sprintf('        book: %s', b.book_str);         %#ok<AGROW>
        case {'shape','bound'}
            L{end+1} = sprintf('   %-4s %-52s want %s | got %s', ...
                vstr(b.ok), b.what, b.book_str, b.ours_str);            %#ok<AGROW>
        otherwise
            L{end+1} = sprintf('   %-4s %-52s book %s | ours %s %s (%+.4f%%, band %s)', ...
                vstr(b.ok), b.what, b.book_str, b.ours_str, b.units, ...
                b.pct, b.band);                                          %#ok<AGROW>
    end
end
L{end+1} = '';
L{end+1} = 'STRUCTURAL (self-consistency only -- NOT book values)';
for i = 1:numel(R.selfcheck.names)
    L{end+1} = sprintf('   %-4s %-52s %s', vstr(R.selfcheck.oks(i)), ...
        R.selfcheck.names{i}, R.selfcheck.detail{i});                    %#ok<AGROW>
end
L{end+1} = '';
if any(ev)
    L{end+1} = sprintf('   %d of %d evaluated book values pass; %d not scored.', ...
        sum([R.bench(ev).ok]), sum(ev), sum(~ev));
else
    L{end+1} = '   NOTHING WAS SCORED.';
end
s = strjoin(L, newline);
end

function v = vstr(ok)
if ok, v = 'ok'; else, v = 'FAIL'; end
end

function y = firstOr(x)
%FIRSTOR  x, or NaN if x is empty.  See the note at the velocity shape row.
if isempty(x), y = NaN; else, y = x(1); end
end
