function R = lambertEfficient(opts)
%MGS.ZARCHAN.LAMBERTEFFICIENT  Zarchan Listing 13.2 -- the EFFICIENT Lambert
%   routine: a secant search on flight-path angle, bracketed by the escape
%   velocity condition, in place of Listing 13.1's brute-force sweep.
%
%   R = mgs.zarchan.lambertEfficient()
%   R = mgs.zarchan.lambertEfficient('TF', 1500, 'assert', false)
%
%   Port of zarchan.txt:3941-3955 (driver :3942, lambertpz :3943-3955).  Units
%   are the book's: FEET, ft/s, seconds; a = 2.0926e7 ft, gm = 1.4077e16
%   ft^3/s^2.  No g and no metres appear in this listing.
%
%   Velocity is a closed form in flight-path angle gam (:3921).  Ruling out
%   escape (lambda = r0*V^2/gm = 2) bounds gam in [gmin, gmax] (:3928-:3934);
%   the search starts at their MIDPOINT (:3940) and steps by the secant rule
%   (:3936) gam(n+1) = gam(n) + (gam(n)-gam(n-1))*(tFDES-tF(n))/(tF(n)-tF(n-1)),
%   every proposal clamped back into the bracket, the bracket itself tightened
%   by which side of tFDES the flight time landed.  Seven passes replace 1084.
%
% ---------------------------------------------------------------------------
%   THE TWO UNRESOLVED COMPARISONS -- WHICH WAY, AND WHY
%
%   listing_13_02.m reports 2 unresolved variable-to-variable comparisons.  In
%   this book's OCR "," is "<" and "." is ">" (the same file's
%   "while ~(abs(tfdes-tf),=(...))" can only be "<=").  Both are resolved that
%   way and BOTH were then confirmed against the book's own numbers:
%
%     1  "if tf.tfdes  gmax=gam; else gmin=gam;"  ->  tf > tfdes.
%        CONFIRMED ARITHMETICALLY.  Iteration 1 has tF = 1239 > 1000 s, so this
%        branch fires; iteration 2's published angle 11.2508376 deg equals
%        (33.7524947 + gmin)/2, which pins gmin = -11.25082 deg -- so it was
%        gmax that moved.  "<" moves the other end and the published sequence
%        cannot occur.  Structural check 4 runs the flipped version.
%     2  "if xlongt.xlongm"  ->  xlongt > xlongm.  Target 90 deg, missile
%        45 deg, so this is the branch Table 13.2 was printed from: it gives
%        vrx = v*sin(gam-xlongm), reproducing the table's signs and magnitudes;
%        the other branch reproduces neither.  ⚠️ It changes NOTHING about tF
%        or the iteration count -- a silent failure, so structural check 5.
%
%   Also recorded, not "fixed": the file is lambertpz.m but declares
%   "function ...=lambert(...)" (MATLAB dispatches on the FILE name, so the
%   book's call works); "a=2.0926e7" and "degrad" inside lambertpz are assigned
%   and never read.  No '¼ð Þþ½' prose leaked in, so nothing was dropped.
%
% ---------------------------------------------------------------------------
%   ⭐ TABLE 13.2's ANGLE COLUMN IS 57.3*gam, NOT rad2deg(gam)
%
%   rad2deg misses every entry of that column by 7.4e-5 relative -- ten times
%   the residual on the velocity columns, in a table printed to nine digits.
%   Listing 13.1 stores its angle as "ArrayBGAM(count)=57.3*gam" (:3907), and
%   57.3 pulls the whole column to ~1e-7.  R.tab.gamdeg is therefore 57.3*gam.
%   A port that silently used 57.29578 would sit inside a 1e-4 band and look
%   fine.
%
%   ⚠️ RESIDUAL, NAMED: the velocity columns still differ by ~0.023 ft/s
%   (6.2e-6 relative at worst, on iteration 1's small VRX).  ONE constant
%   causes it: the driver sets PI = 3.14159 while lambertpz writes "pi/2.".
%   Using the driver's PI there too shrinks the worst VRX gap to ~0.0023 ft/s,
%   i.e. the book's table came from a run with one constant throughout.  This
%   port stays FAITHFUL to the printed "pi/2." and reports the alternative as
%   R.pi_diag rather than adopting it.  The 1e-5 band on every table row is the
%   smallest round number above that named residual -- chosen once, from a
%   cause, and applied to all of them.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS: Table 13.2 (:3962-3970), Table 13.1's nominal row (:3913),
%   prose (:3957).  ⚠️ Table 13.2's VRX column has its minus sign OCR'd to a
%   leading "2": "212075.71473" is -12075.71473.  Decoded by arithmetic, not by
%   taste -- v at iteration 2's own gam is 21731 ft/s, so 212075 is impossible
%   and -12075.7 is what the formula gives.  Table 13.1's "27696" is -7696 the
%   same way.  VRY is genuinely positive.
%
% ---------------------------------------------------------------------------
%   ⭐⭐⭐ NEAR-PARABOLIC GUARD -- lambda = r0*|v|^2/gm MUST STAY BELOW 1.999999
%
%   THE SILENT WRONG ANSWER.  Listing 13.2 is ELLIPTIC-ONLY.  Its bracket
%   [gmin, gmax] is DEFINED by the escape condition lambda = 2, and its
%   flight-time closed form divides by bot2 = xlam*(2/xlam-1)^1.5, which
%   collapses as lambda -> 2.  Handed a transfer that is genuinely HYPERBOLIC
%   the secant cannot leave the bracket, so it crawls to the bracket EDGE and
%   halts on an orbit that is not the solution -- while its own residual test
%   says it converged.
%
%   MEASURED 2026-08-24 -- alt 300 nm -> 0, transfer angle 78 deg, TF = 750 s:
%     true transfer needs   lambda = 2.014012          (HYPERBOLIC)
%     lambertpz halts at    lambda = 1.999999919,  bot2 = 1.6409e-11
%     it reports            tF = 749.999996 s   -> passes its own 1e-8 test
%     ⚠⚠ converged = TRUE
%     propagating its answer (ode45, RelTol 1e-13) MISSES THE TARGET BY
%     98,751.56 ft.  An independent universal-variable (Bate-Mueller-White)
%     Lambert returns 35302.6106 ft/s and misses by 0.00 ft; this listing
%     returns 35179.5920 ft/s.
%
%   ⚠⚠ THE GUARD IS NOT KEYED ON THE ROUTINE'S `converged` FLAG.  That flag is
%   the thing being caught -- it is TRUE on the wrong answer, in all six
%   transcriptions of this listing carried in this package.
%   ⚠⚠ AND MUTUAL AGREEMENT DID NOT CATCH IT EITHER.  Those six transcriptions
%   agree to 0.000e+00 over 954 converged geometries with identical iteration
%   counts 954/954, because the misreading is in the LISTING, not in any one
%   transcription of it.  Only a quantity the routine does not control
%   separates the two answers, and lambda is that quantity.
%
%   ⚠ MEASURED SEPARATION, and it is NOT the "enormous margin" this guard was
%   commissioned on.  Over a 6150-geometry sweep (50 transfer angles x 41
%   flight times x 3 altitude pairs) the 3043 converged-and-CORRECT cases reach
%   lambda = 1.999997634 and the 17 converged-and-WRONG cases start at
%   1.999999740 (misses 66,600 ft to 24,278,620 ft, all adjudicated by
%   propagation).  The threshold separates them with ZERO false alarms, but the
%   gap is ~1.4e-6, not the ~0.039 a coarser 1080-point grid suggested.
%   ⚠ ONE CASE LEAKS: transfer angle 99 deg, TF = 900 s halts at
%   lambda = 1.999997634 -- BELOW the threshold -- and still misses by 141.89 ft.
%   The threshold is left where it was specified; widening it to swallow that
%   case is not something any source supports.
%
%   THIS PORT'S REACH.  lambertEfficient exposes the geometry directly, so the
%   band is reachable through its own public options: 29 of the 297 grid points
%   swept on 2026-08-24 land above the cap.  The demonstration point recorded in
%   this package is XLONGMDEG = 0, XLONGTDEG = 78, ALTNMM = ALTNMT = 300,
%   TF = 585 -> lambda = 1.999999983, the guard goes RED, and propagating that
%   answer misses by 7,715,810 ft.
%   ⚠ Note the driver's PI = 3.14159 (not pi) scales every input angle by
%   1 + 8.4e-7, which in a band this narrow moves the failing flight times:
%   the raw 78 deg / 750 s point above is NOT the one this entry point reaches
%   (its effective transfer angle there is 77.999934 deg).
%   MARGIN AT THE BOOK'S OWN CASE (45 deg -> 90 deg, TF = 1000 s):
%   lambda_out = 0.587458159, peak over the 7 passes 0.702306240.
% ---------------------------------------------------------------------------
%
%   R: .vrx .vry .iters .tab .gmin_deg .gmax_deg .phi_deg .pi_diag .p .units
%      .lambda_out .lambda_max .selfcheck (.names/.oks/.detail) .bench .pass
%      .summary .verdict

arguments
    opts.XLONGMDEG (1,1) double {mustBeFinite} = 45      % missile longitude, deg
    opts.XLONGTDEG (1,1) double {mustBeFinite} = 90      % target  longitude, deg
    opts.ALTNMM    (1,1) double {mustBeFinite} = 0       % missile altitude, n mi
    opts.ALTNMT    (1,1) double {mustBeFinite} = 0       % target  altitude, n mi
    opts.TF        (1,1) double {mustBeFinite, mustBePositive} = 1000   % s
    opts.reltol    (1,1) double {mustBePositive} = 1e-8  % the listing's .00000001
    % NOT IN THE LISTING.  The book's while loop has no cap; a flipped bracket
    % test (structural check 4) makes gam NaN, and ~(abs(NaN)<=tol) is TRUE
    % forever.  200 never binds at book settings (7 used).
    opts.maxit     (1,1) double {mustBePositive, mustBeInteger} = 200
    % THE ONLY SWITCHES.  Not nargin: with a name-value-only arguments block
    % MATLAB reports nargin = 0 however many were passed.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.assert    (1,1) logical = true
end

% ---- driver, zarchan.txt:3942 -------------------------------------------
PI     = 3.14159;                 % the listing's own PI, not MATLAB's pi
DEGRAD = 360./(2.*PI);
A      = 2.0926e7;                % ft, Earth radius
GM     = 1.4077e16;               % ft^3/s^2
ALTT   = opts.ALTNMT*6076.;
ALTM   = opts.ALTNMM*6076.;
XLONGM = opts.XLONGMDEG/DEGRAD;
XLONGT = opts.XLONGTDEG/DEGRAD;
XM = (A+ALTM)*cos(XLONGM);   YM = (A+ALTM)*sin(XLONGM);
XT = (A+ALTT)*cos(XLONGT);   YT = (A+ALTT)*sin(XLONGT);

sw = struct('secant',true, 'gtBranch',true, 'bracketGT',true, 'piRes',pi);
[VRXM, VRYM, T] = lambertpz(XM, YM, opts.TF, XT, YT, XLONGM, XLONGT, ...
                            GM, opts.reltol, opts.maxit, sw);

% ---- the three variants the structural checks need ----------------------
swB = sw;  swB.secant    = false;   % delete the secant step -> bisection only
swF = sw;  swF.bracketGT = false;   % flip "if tf > tfdes"
swL = sw;  swL.gtBranch  = false;   % flip "if xlongt > xlongm"
swP = sw;  swP.piRes      = PI;     % the pi-constant diagnosis
[~,   ~,   TB] = lambertpz(XM,YM,opts.TF,XT,YT,XLONGM,XLONGT,GM,opts.reltol,opts.maxit,swB);
[~,   ~,   TF_] = lambertpz(XM,YM,opts.TF,XT,YT,XLONGM,XLONGT,GM,opts.reltol,opts.maxit,swF);
[VLX, VLY, TL] = lambertpz(XM,YM,opts.TF,XT,YT,XLONGM,XLONGT,GM,opts.reltol,opts.maxit,swL);
[~,   ~,   TP] = lambertpz(XM,YM,opts.TF,XT,YT,XLONGM,XLONGT,GM,opts.reltol,opts.maxit,swP);

BOOK = bookTable();
R = struct();
R.vrx = VRXM;  R.vry = VRYM;  R.iters = T.n;
R.tab = struct('it',(1:T.n)', 'gamdeg',T.gamdeg, 'vrx',T.vrx, 'vry',T.vry, ...
               'tf',T.tf, 'gmin',T.gminDeg, 'gmax',T.gmaxDeg);
R.gmin_deg = T.gmin0Deg;  R.gmax_deg = T.gmax0Deg;  R.phi_deg = T.phiDeg;
R.lambda_out = T.lamOut;  R.lambda_max = max(T.lamPath);
R.book = BOOK;
R.alt  = struct('bisect_iters',TB.n, 'bisect_ok',TB.converged, ...
                'flipbracket_iters',TF_.n, 'flipbracket_ok',TF_.converged, ...
                'lonflip_iters',TL.n, 'lonflip_vrx',VLX, 'lonflip_vry',VLY);
n = min(T.n, size(BOOK,1));
R.pi_diag = struct( ...
    'faithful_maxabs_vrx', maxAbs(T.vrx(1:n),  BOOK(1:n,2)), ...
    'altPI_maxabs_vrx',    maxAbs(TP.vrx(1:n), BOOK(1:n,2)), ...
    'note', ['faithful uses pi/2 as printed; altPI uses the driver''s ' ...
             'PI = 3.14159 in the same place']);
R.units = ['ft, ft/s, s, deg.  a = 2.0926e7 ft, gm = 1.4077e16 ft^3/s^2.  ' ...
           'Angles in R.tab.gamdeg are the BOOK''s 57.3*gam, not rad2deg.'];
R.p = struct('XLONGMDEG',opts.XLONGMDEG, 'XLONGTDEG',opts.XLONGTDEG, ...
             'ALTNMM',opts.ALTNMM, 'ALTNMT',opts.ALTNMT, 'TF',opts.TF, ...
             'reltol',opts.reltol, 'maxit',opts.maxit, 'PI',PI, 'A',A, ...
             'GM',GM, 'converged',T.converged, 'units',R.units);

R.selfcheck = structuralChecks(T, TB, TF_, TL, VRXM, VRYM, VLX, VLY);

if opts.benchmark
    R.bench = benchmarks(R, T, BOOK, opts);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    R.pass  = false;   % NOT CHECKED is not the same thing as "fine"
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end

msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated at XLONGMDEG = %g, ' ...
        'XLONGTDEG = %g, TF = %g s.'], opts.XLONGMDEG, opts.XLONGTDEG, opts.TF);
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question.  ' ...
        'First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
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
        error('mgs:zarchan:lambertEfficient:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:lambertEfficient:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [vrx, vry, T] = lambertpz(xic, yic, tfdes, xf, yf, xlongm, xlongt, ...
                                   gm, reltol, maxit, sw)
%LAMBERTPZ  zarchan.txt:3943-3955, line for line.
%   ⚠️ Every constant is an ARGUMENT.  MATLAB local functions cannot see the
%   caller's workspace, and a previous wave in this package lost a day to that.
%   sw carries the four experiment switches (all default-true = the listing).
ric  = sqrt(xic^2 + yic^2);
rf   = sqrt(xf^2  + yf^2);
cphi = (xic*xf + yic*yf)/(ric*rf);
phi  = acos(cphi);
sphi = sin(phi);
r0   = ric;
icount = 0;

gmin = atan2((sphi - sqrt(2.*r0*(1.-cphi)/rf)), (1-cphi));
gmax = atan2((sphi + sqrt(2.*r0*(1.-cphi)/rf)), (1-cphi));
% ⚠️ gmin/gmax are reported in the BOOK's 57.3 convention because they share a
%    column with Table 13.2's angles.  phi is ours alone, so it uses rad2deg.
T = struct('gmin0Deg', 57.3*gmin, 'gmax0Deg', 57.3*gmax, 'phiDeg', phi*180/pi, ...
           'lamMin', escapeLambda(gmin, phi, r0, rf, gm), ...
           'lamMax', escapeLambda(gmax, phi, r0, rf, gm));
gam = (gmin + gmax)/2.;
tf  = 0;
gold = NaN;  told = NaN;
[g, vx, vy, tv, lo, hi, lam] = deal(nan(maxit,1));

while ~(abs(tfdes-tf) <= (reltol*tfdes))
    top  = gm*(1.-cos(phi));
    temp = r0*cos(gam)/rf - cos(phi+gam);
    bot  = r0*cos(gam)*temp;
    v    = sqrt(top/bot);
    % "if xlongt.xlongm" resolved as ">".  sw.gtBranch = false FLIPS the
    % comparison (not the geometry), so the experiment is the same experiment
    % whichever way round the two longitudes happen to be.
    if sw.gtBranch, takeFirst = xlongt > xlongm; else, takeFirst = xlongt < xlongm; end
    if takeFirst
        vrx = v*cos(sw.piRes/2. - gam + xlongm);
        vry = v*sin(sw.piRes/2. - gam + xlongm);
    else
        vrx = v*cos(-sw.piRes/2. + gam + xlongm);
        vry = v*sin(-sw.piRes/2. + gam + xlongm);
    end
    xlam  = r0*v*v/gm;
    top1  = tan(gam)*(1-cos(phi)) + (1-xlam)*sin(phi);
    bot1p = (1-cos(phi))/(xlam*cos(gam)*cos(gam));
    bot1  = (2-xlam)*(bot1p + cos(gam+phi)/cos(gam));
    top2  = 2*cos(gam);
    bot2  = xlam*((2/xlam-1)^1.5);
    top3  = sqrt(2/xlam-1);
    bot3  = cos(gam)/tan(phi/2) - sin(gam);
    temp  = (top2/bot2)*atan2(top3, bot3);
    tf    = r0*(top1/bot1 + temp)/(v*cos(gam));
    icount = icount + 1;

    g(icount) = 57.3*gam;   vx(icount) = vrx;  vy(icount) = vry;
    tv(icount) = tf;        lo(icount) = 57.3*gmin;  hi(icount) = 57.3*gmax;
    lam(icount) = xlam;

    % "if tf.tfdes gmax=gam; else gmin=gam;" resolved as ">".
    % sw.bracketGT = false is the delete experiment on that resolution.
    if sw.bracketGT, tooLong = tf > tfdes; else, tooLong = tf < tfdes; end
    if tooLong
        gmax = gam;
    else
        gmin = gam;
    end
    if icount == 1 || ~sw.secant
        xnext = (gmax+gmin)/2.;
    else
        xnext = gam + (gam-gold)*(tfdes-tf)/(tf-told);
        % NaN fails BOTH tests, so a NaN proposal is kept -- which is exactly
        % what the book's code does.  maxit is the only thing that stops it.
        if (xnext > gmax || xnext < gmin)
            xnext = (gmax+gmin)/2.;
        end
    end
    gold = gam;  told = tf;  gam = xnext;
    if icount >= maxit, break, end
end

T.n = icount;
T.gamdeg = g(1:icount);  T.vrx = vx(1:icount);  T.vry = vy(1:icount);
T.tf = tv(1:icount);     T.gminDeg = lo(1:icount);  T.gmaxDeg = hi(1:icount);
T.converged = abs(tfdes - tf) <= (reltol*tfdes);
% NEAR-PARABOLIC GUARD, see the file header.  lambda is recomputed from the
% RETURNED velocity rather than read out of the loop's bookkeeping, so the
% number the check sees is the one a caller would actually fly.  vrx^2 + vry^2
% is v^2 for either longitude branch and for either value of sw.piRes.
T.lamOut  = r0*(vrx^2 + vry^2)/gm;
T.lamPath = lam(1:icount);
T.errAbs = abs(tfdes - tv(1:icount));
T.tolAbs = reltol*tfdes;
end


function lam = escapeLambda(gam, phi, r0, rf, gm)
%ESCAPELAMBDA  lambda = r0*V^2/gm at a given flight-path angle.  gmin and gmax
%   are DEFINED as the roots of lambda = 2, so this must return 2 at both.
temp = r0*cos(gam)/rf - cos(phi+gam);
lam  = r0*(gm*(1.-cos(phi))/(r0*cos(gam)*temp))/gm;
end


function r = maxAbs(a, b)
r = max(abs(a(:) - b(:)));
end


% =============================================================================
function K = structuralChecks(T, TB, TF_, TL, vrx, vry, vlx, vly)
%STRUCTURALCHECKS  Self-consistency, NOT book values.  ⚠️ .names and .oks must
%   both exist and be the SAME LENGTH or benchSuite puts ZERO rows on the board
%   (and now flags the port red for it).
names = {};  oks = [];  detail = {};

names{end+1} = 'escape-velocity bracket: lambda(gmin) = lambda(gmax) = 2';
r1 = max(abs([T.lamMin T.lamMax] - 2));
oks(end+1) = r1 <= 1e-9;
detail{end+1} = sprintf('max |lambda-2| = %.2e at gmin/gmax = %.5f/%.5f deg', ...
    r1, T.gmin0Deg, T.gmax0Deg);

names{end+1} = 'first guess is the bracket midpoint; no iterate ever leaves it';
mid = (T.gmin0Deg + T.gmax0Deg)/2;
inBr = all(T.gamdeg >= T.gminDeg - 1e-12) && all(T.gamdeg <= T.gmaxDeg + 1e-12);
shrink = all(diff(T.gmaxDeg - T.gminDeg) <= 1e-12);
oks(end+1) = abs(T.gamdeg(1) - mid) <= 1e-9*max(1,abs(mid)) && inBr && shrink;
detail{end+1} = sprintf('gam(1) = %.7f vs midpoint %.7f; %d iterates in bracket, width %.4g -> %.4g deg', ...
    T.gamdeg(1), mid, T.n, T.gmaxDeg(1)-T.gminDeg(1), T.gmaxDeg(end)-T.gminDeg(end));

names{end+1} = 'DELETE the secant step -> bisection alone needs more iterations';
oks(end+1) = TB.n > T.n;
detail{end+1} = sprintf('bisection %d iterations (converged %d) vs secant %d', ...
    TB.n, TB.converged, T.n);

names{end+1} = 'FLIP "if tf > tfdes" -> the search stops converging';
oks(end+1) = ~(TF_.converged && TF_.n <= T.n);
detail{end+1} = sprintf('flipped: %d iterations, converged = %d', TF_.n, TF_.converged);

names{end+1} = 'FLIP "if xlongt > xlongm" -> SAME tF and count, wrong velocity';
sameTf = TL.n == T.n && max(abs(TL.tf - T.tf)) <= 1e-9*max(abs(T.tf));
oks(end+1) = sameTf && max(abs([vlx vly] - [vrx vry])) > 1;
detail{end+1} = sprintf('tF identical (%d iters); (vrx,vry) %.2f,%.2f -> %.2f,%.2f ft/s', ...
    TL.n, vrx, vry, vlx, vly);

% 6  NEAR-PARABOLIC GUARD.  See the file header for the 98,751 ft miss this
%    exists to catch, why it is not keyed on T.converged (that flag is TRUE on
%    the wrong answer) and why agreement with this package's five other
%    transcriptions of the same listing is not evidence either (all six agree).
names{end+1} = 'near-parabolic guard: lambda = r0|v|^2/gm < 1.999999';
LAMCAP = 1.999999;
K.lambda_out = T.lamOut;          % same field names the other five ports use
K.lambda_max = max(T.lamPath);
oks(end+1) = isfinite(T.lamOut) && T.lamOut < LAMCAP;
detail{end+1} = sprintf(['lambda from the RETURNED velocity = %.9f ' ...
    '(cap %.6f, margin %.3e); peak over %d passes = %.9f'], ...
    T.lamOut, LAMCAP, LAMCAP - T.lamOut, T.n, max(T.lamPath));

K.names = names;  K.oks = logical(oks);  K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    j = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{j}, detail{j});
end
end


% =============================================================================
function M = bookTable()
%BOOKTABLE  Table 13.2, zarchan.txt:3964-3970, columns
%   [57.3*gam (deg), VRX (ft/s), VRY (ft/s), tF (s)].
%   ⚠️ The VRX column's minus signs were OCR'd to a leading "2" and are
%   restored here; see the file header for the arithmetic that proves it.
M = [ 33.7524947,   -3764.57976,  18926.02426,  1239.37545
      11.2508376,  -12075.71473,  18072.66484,   813.53185
      21.1038504,   -8103.20444,  18287.99279,   979.68031
      22.3088581,   -7665.88409,  18332.46792,  1001.50708
      22.2256555,   -7695.84063,  18329.28399,   999.98566
      22.2264396,   -7695.55815,  18329.31392,   999.99999
      22.2264402,   -7695.55795,  18329.31394,  1000.00000 ];
end


function B = benchmarks(R, T, BOOK, opts)
%BENCHMARKS  Ten rows.  Band on every Table 13.2 row is 1e-5 relative, chosen
%   from the NAMED pi-constant residual in the header and applied to all of
%   them -- not fitted row by row.
TOL  = 1e-5;
SRC2 = 'Table 13.2, zarchan.txt:3962-3970';
onBook = abs(opts.TF - 1000) < 1e-9 && abs(opts.XLONGMDEG - 45) < 1e-9 && ...
         abs(opts.XLONGTDEG - 90) < 1e-9 && opts.ALTNMM == 0 && opts.ALTNMT == 0;
B = emptyBench();

if ~onBook
    B = addSkip(B, 'Table 13.2 (all rows)', '7 iterations', ...
        sprintf(['the table is the 45->90 deg, sea-level, TF = 1000 s case; ' ...
                 'this run is %g->%g deg, TF = %g s.  A band widened to ' ...
                 'absorb off-book inputs is not a check.'], ...
                 opts.XLONGMDEG, opts.XLONGTDEG, opts.TF), SRC2, '-');
    return
end

B = addVal(B, 'iterations to converge', 7, R.iters, 'rel', 0, ...
    SRC2, '-', 'the table has seven rows; prose ":3957" says seven');

k = min(7, T.n);
B = addVal(B, 'iter 7 flight-path angle (57.3*gam)', BOOK(7,1), T.gamdeg(k), ...
    'rel', TOL, SRC2, 'deg', ...
    'rad2deg instead of 57.3 lands at 7.4e-5 -- ten times this residual');
B = addVal(B, 'iter 7 VRX', BOOK(7,2), T.vrx(k), 'rel', TOL, SRC2, 'ft/s', ...
    sprintf('book prints "27695.55795"; pi-constant residual %.4f ft/s', ...
            R.pi_diag.faithful_maxabs_vrx));
B = addVal(B, 'iter 7 VRY', BOOK(7,3), T.vry(k), 'rel', TOL, SRC2, 'ft/s', '');
B = addVal(B, 'iter 2 VRX (mid-search, minus decoded)', BOOK(2,2), T.vrx(min(2,T.n)), ...
    'rel', TOL, SRC2, 'ft/s', ...
    'book prints "212075.71473"; v at this gam is 21731 ft/s, so 212075 is impossible');
B = addVal(B, 'iter 4 flight time', BOOK(4,4), T.tf(min(4,T.n)), 'rel', TOL, ...
    SRC2, 's', 'the "very accurate after only four iterations" row, :3957');

% cross-table: Table 13.1's nominal brute-force answer, printed to the nearest
% ft/s.  Band is HALF THE LAST PRINTED DIGIT -- a rounding band, not a fitted
% one.  ⚠️ "27696" is -7696: same OCR damage as Table 13.2's VRX column.
B = addVal(B, 'Table 13.1 nominal VRX (brute force, rounded)', -7696, R.vrx, ...
    'rel', 0.5/7696, 'Table 13.1 nominal row, zarchan.txt:3913', 'ft/s', ...
    'the two tables must agree; 13.1 prints to the nearest ft/s');

worst = 0;  where = '';
for i = 1:min(7, T.n)
    ours = [T.gamdeg(i) T.vrx(i) T.vry(i) T.tf(i)];
    rel  = abs(ours./BOOK(i,:) - 1);
    [m, j] = max(rel);
    if m > worst, worst = m;  where = sprintf('iter %d col %d', i, j); end
end
B = addShape(B, 'every cell of Table 13.2 within 1e-5 relative', ...
    'all 28 cells', sprintf('worst %.2e (%s)', worst, where), ...
    T.n >= 7 && worst <= TOL, SRC2, ...
    '4 columns x 7 iterations, scored together so no single lucky row carries it');

B = addBound(B, 'more than two orders of magnitude faster than brute force', ...
    '1084/iters >= 100', sprintf('1084/%d = %.1f', R.iters, 1084/max(R.iters,1)), ...
    1084/max(R.iters,1) >= 100, ...
    'prose zarchan.txt:3957 + Table 13.1 :3913', '-', ...
    'the book''s claim is about iterations; wall time is not published');

B = addSkip(B, 'agreement with Listing 13.1 brute force, run side by side', ...
    'VRX/VRY equal to ~0.1 ft/s', ...
    ['mgs.zarchan.lambertBruteForce does not exist in +mgs/+zarchan at the ' ...
     'time this port was written, so the direct cross-check could not be ' ...
     'run.  The Table 13.1 row above is the paper substitute for it.'], ...
    'Listing 13.1, zarchan.txt:3901-3909', 'ft/s');
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER are benchSuite's adaptGeneric schema.
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

function B = addBound(B, what, bookStr, oursStr, ok, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'bound';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'bound';  B(i).tol = NaN;
B(i).ok   = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = units;  B(i).detail = detail;
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
L{end+1} = 'Zarchan Listing 13.2 -- MORE EFFICIENT LAMBERT ROUTINE (secant + escape bracket)';
L{end+1} = sprintf('   %s', R.units);
L{end+1} = sprintf(['   missile %g deg / target %g deg, alt %g / %g n mi, ' ...
    'tFDES = %g s, reltol = %g'], R.p.XLONGMDEG, R.p.XLONGTDEG, R.p.ALTNMM, ...
    R.p.ALTNMT, R.p.TF, R.p.reltol);
L{end+1} = sprintf(['   phi = %.5f deg, bracket [%.5f, %.5f] deg -> %d ' ...
    'iterations, VRX = %.5f, VRY = %.5f ft/s'], R.phi_deg, R.gmin_deg, ...
    R.gmax_deg, R.iters, R.vrx, R.vry);
L{end+1} = '';
L{end+1} = '   it   57.3*gam [deg]        VRX [ft/s]        VRY [ft/s]        tF [s]';
for i = 1:numel(R.tab.it)
    L{end+1} = sprintf('   %2d   %15.7f  %16.5f  %16.5f  %13.5f', ...
        R.tab.it(i), R.tab.gamdeg(i), R.tab.vrx(i), R.tab.vry(i), R.tab.tf(i)); %#ok<AGROW>
end
L{end+1} = sprintf(['   pi-constant diagnosis: worst |dVRX| vs book = %.4f ft/s ' ...
    'as printed, %.4f ft/s if pi/2 uses the driver''s PI = 3.14159'], ...
    R.pi_diag.faithful_maxabs_vrx, R.pi_diag.altPI_maxabs_vrx);
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-58s %s', vstr(K.oks(i)), K.names{i}, K.detail{i}); %#ok<AGROW>
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
            case 'skip'
                L{end+1} = sprintf('   SKIP %-52s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case {'shape','bound'}
                L{end+1} = sprintf('   %-4s %-52s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf(['   %-4s %-52s book %s %s / ours %s %s ' ...
                    '(%+.5f%%, band %.4g%%)'], vstr(b.ok), b.what, b.book_str, ...
                    b.units, b.ours_str, b.units, b.pct, 100*b.tol); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped/info -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: this routine assumes a KEPLERIAN, drag-free, non-rotating';
L{end+1} = '   Earth and an IMPULSIVE velocity change.  It sizes a velocity';
L{end+1} = '   vector; it is not a guidance law.  Listing 13.3 is what closes';
L{end+1} = '   the loop around it.';
s = strjoin(L, newline);
end

function s = vstr(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end
