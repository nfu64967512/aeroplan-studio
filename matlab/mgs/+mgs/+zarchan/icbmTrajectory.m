function R = icbmTrajectory(opts)
%MGS.ZARCHAN.ICBMTRAJECTORY  Zarchan Listing 28.2 -- BALLISTIC MISSILE
%   TRAJECTORY GENERATOR.  A three-dimensional Lambert solve (LAMBERT3D) picks
%   the impulsive launch velocity; a two-body ECI integration then flies Monte
%   Carlo -> Las Vegas in 2000 s.
%
%   R = mgs.zarchan.icbmTrajectory()
%   R = mgs.zarchan.icbmTrajectory('H', 0.01, 'assert', false)   coarse + fast
%   R = mgs.zarchan.icbmTrajectory('plot', true)
%
%   Port of zarchan.txt:10409-10456 (driver :10409-:10421, distance3d :10423,
%   LAMBERT3D :10428-:10456).  mgs.zarchan.engage3d is Listing 28.1, the
%   tactical flat-Earth half of this chapter; the two share only a number.
%
%   UNITS ARE THE BOOK'S.  ft and ft/s integrate; a = 2.0926e7 ft,
%   gm = 1.4077e16 ft^3/s^2 (:10375), w = -6.283185/86400 rad/s (:10391 quotes
%   7.27e-5).  nmi = 6076 ft (the ALTNM divisor); km = 3280 ft (the distance3d
%   and Listing 28.3 divisor -- NOT 3280.84); degrees are 57.3*radians, the
%   book's own constant, in and out.  180/pi instead moves the launch point by
%   0.0074% of its own latitude -- MEASURED 1185.7 ft, 0.36 km on the ground,
%   nine orders above BENCHMARK 4's band while still looking like rounding.
%
%   THE EARTH MODEL IS A SPHERE AND THE LATITUDE IS GEOCENTRIC.  The listing
%   writes r = a*[cos(lat)cos(long); cos(lat)sin(long); sin(lat)] with one
%   constant a, so |r| == a identically; there is no flattening anywhere in
%   Chapter 28.  Reading 43.75 deg as GEODETIC and converting to geocentric
%   would move the launch point 0.1922 deg -- 70201 ft on the ground, MEASURED,
%   5.079e4 ft of max-component error -- and it is neither what the book does
%   nor what this port does.
%
%   THE FRAME IS RIGHT-HANDED AND ITS X AXIS IS GREENWICH AT t = 0, not the
%   vernal equinox.  The launch longitude enters r0 with NO rotation applied
%   and eci2ecef() is the identity at T = 0 (cos(W*0) = 1, sin(W*0) = 0), so
%   "ECI" here means "ECEF frozen at the epoch"; only the AIM point is led, by
%   -W*TF.  Negating Y -- a left-handed reading -- moves r0 by 3.9e6 ft.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_28_02.m reports 2 unresolved variable-to-
%   variable comparisons and 3 lines of leaked prose.  This book's OCR maps
%   '<' -> ',' and '>' -> '.', and EVERY comparison in 28.2 falls out of that
%   one substitution:
%
%     while (abs(TF-T).(1e-8*TF))  -> >     [must run WHILE the error is large]
%     if T.TF & GAMMA,G_MAX        -> T > TF && GAMMA < G_MAX
%     if T < 0. & GAMMA,G_MAX      -> T < 0  && GAMMA < G_MAX
%     if T,TF & GAMMA.G_MIN        -> T < TF && GAMMA > G_MIN
%     if (T,0.)                    -> T < 0
%     NEXT.=G_MAX / NEXT,=G_MIN    -> >= / <=   [clamp INTO the bracket]
%     ITERS.100 / LAMBDA,1.9999999 / LAMBDA.2.0000001  -> > / < / >
%     ALTNM.-1 / STEP,=1 / S.=9.9999 / CBETA,=1. -> > -1 / <= 1 / >= / <=
%
%   ⭐ THE BRACKET DIRECTION IS NOT A GUESS.  Flight time DECREASES with gamma
%   over [G_MIN, G_MAX] (G_MAX = pi/2 is straight up, i.e. the long way), so a
%   solution that flew too LONG means gamma was too small and that gamma
%   becomes the new UPPER bound.  Flip it and the bracket inverts on iteration
%   1.  Confirmed arithmetically as well: this resolution is what reproduces
%   the book's own published launch velocity (BENCHMARK 1) to the digits shown.
%
%   Three lines of prose leaked into the tail (the Fig. 28.9 paragraph, the
%   next section heading, the Table 28.1 passage, :10457+).  Dropped, not
%   "fixed" into code -- and they are where three benchmarks came from, which
%   is how they were identified.
%
%   Book quirks kept deliberately:
%     * distance3d returns "DISTKM" = a*beta/3280 -- ft per KM -- and the
%       driver stores it in DISTRTNM under an axis labelled "Nmi".  It cannot
%       be both, and only the /6076 reading gives the prose's "nearly 5000 n
%       miles".  R.dist_km is exactly what the listing computes; R.dist_nmi
%       rescales it, and that is what is scored.  See the INFO row.
%     * ALTNM is computed INSIDE the predictor loop, so the altitude recorded
%       (and tested by the outer loop) is the EULER-PREDICTED one while the
%       downrange beside it is CORRECTED.  Kept; ~1e-5 ft at H = 1 ms.
%     * S/BL/BT and four unit constants in LAMBERT3D are assigned and never
%       read.  Kept and marked, so the printed page still diffs cleanly.
%     * "save datfil.txt output /ascii" is the book's recurring -ascii typo;
%       nothing is written to disk here at all, so it is dropped.
%     * The SWITCH1 = 1 branch is missing a semicolon after CROSSX (:10455).
%       Added; it is an echo, not a computation.
%
%   ⚠️ THE SILENT FAILURE IN THIS LISTING is "XLONGF = XLONGF - W*TF", which
%   aims at where Las Vegas WILL BE in 2000 s (W is negative, so this ADDS
%   8.33 deg).  Delete that one line and the program still runs, still solves
%   Lambert, still draws a plausible arch with the right apogee and the right
%   flight time -- 8.3 deg from the target, with no symptom.  STRUCTURAL CHECK
%   4 is a DELETE EXPERIMENT on exactly that line.
%
% ---------------------------------------------------------------------------
%   ⭐⭐ THE REAL*4 LITERALS.  Listing 28.3's header line (:10475) publishes
%   THIS listing's launch state to fifteen digits.  Evaluate Listing 28.2's own
%   closed form in double precision and the position lands 0.1537 ft from those
%   digits -- 1.0e-8 relative, digit nine.  That is far too large for a closed
%   form whose constants are exact and whose trig is correctly rounded, and far
%   too small for any of the usual suspects.  ALL MEASURED, none of them fit:
%
%       180/pi instead of 57.3 .................  1185.7 ft
%       geodetic instead of geocentric latitude .   70201 ft
%       the Earth-rotation lead applied to r0 ...   3.0e6 ft
%       a left-handed frame (Y negated) .........   3.9e6 ft
%       double-precision literals (this port) ...  0.1537 ft   <- the residual
%
%   (displacements on the ground; the benchmark's own max-component numbers are
%   in the FAULT INJECTION table below, and they tell the same story.)
%
%   The book's number is reproduced EXACTLY -- 3.7e-8 ft, 0.37 of the last
%   printed digit -- by rounding the three literals the expression consumes to
%   IEEE SINGLE first and then doing the arithmetic in double:
%
%       la = double(single(43.75)) / double(single(57.3))
%       lo = double(single(7.42))  / double(single(57.3))
%
%   which is precisely what a FORTRAN untyped constant does inside a REAL*8
%   expression.  ⭐ THIS IS NOT A FIT.  It has no free parameter and it
%   satisfies TWO independent equations at once, to seven figures:
%
%       lat offset  measured +1.331482e-08   predicted +1.331482e-08
%                                            = -rel(single(57.3))
%       long offset measured +2.359703e-08   predicted +2.359703e-08
%                                            =  rel(single(7.42)) - rel(single(57.3))
%
%   and the SPLIT is forced by the binary, not chosen: 43.75 and 2.0926e7 are
%   exactly representable in single, 7.42 and 57.3 are not, so the latitude may
%   move only by the 57.3 error while the longitude must move by both.  It does.
%   Geometrically the book's launch point sits 6.5 cm north and 1.4 cm east of
%   this port's -- which is also why no downstream number can arbitrate: seeding
%   the run with the book's own r0 leaves the published v0 1.871e-3 ft/s away
%   instead of 1.953e-3, and Table 28.1 marginally WORSE (0.002791 km vs
%   0.002750 km).  The r0 print is the only place the signature is visible.
%
%   ⚠️ THIS PORT DOES NOT ADOPT THE SINGLE ROUNDING.  R.r0 and everything
%   downstream stay double, which is the correct evaluation of the source as
%   printed; the REAL*4 form is built ONLY to score the published digits, and
%   both numbers are on the board (BENCHMARK 4 plus its INFO row) and in
%   R.r0_book_form.  ⚠️ The launch VELOCITY does not follow the same story:
%   single literals move it from 1.953e-3 to 1.707e-3 ft/s of the published
%   value, so ~2e-6 relative of BENCHMARKS 1-3 remains unexplained -- and it is
%   NOT the Lambert exit tolerance, since tightening 1e-8 -> 1e-14 moves v0 by
%   3e-8 ft/s.  Those rows pass their 1e-5 band regardless; the gap is logged
%   here rather than papered over.
%
% ---------------------------------------------------------------------------
%   ⭐⭐ FAULT INJECTION.  A row that cannot fail prints green and makes the
%   port look checked, so BENCHMARK 4 was driven red on demand, one fault at a
%   time, and these are the numbers it printed (band is 1e-6 ft; H = 0.05 s):
%
%     fault injected                      BENCH4 "ours"   row   R.pass
%     ---------------------------------   -------------   ---   ------
%     none (baseline)                       3.725e-08 ft  PASS   PASS
%     57.3 -> 180/pi                        8.500e+02 ft  FAIL   FAIL
%     geodetic instead of geocentric lat    5.079e+04 ft  FAIL   FAIL
%     left-handed frame (Y negated)         3.904e+06 ft  FAIL   FAIL
%     a = 2.0926e7 + 1 ft                   7.164e-01 ft  FAIL   FAIL
%     REAL*4 dropped on 57.3                1.537e-01 ft  FAIL   FAIL
%     REAL*4 dropped on 7.42 ONLY           1.996e-02 ft  FAIL   FAIL
%     REAL*4 dropped entirely (= double)    1.537e-01 ft  FAIL   FAIL
%     launch point ALSO led by -W*TF        3.725e-08 ft  pass   FAIL
%
%   ⚠️ READ THE LAST TWO LINES.  The 7.42-only fault is the sharp one: it moves
%   the answer by 0.02 ft, four orders outside the band, so the row discriminates
%   WHICH literal is exactly representable and would go red if the identification
%   above were merely a curve fit.  And the last row is why STRUCTURAL CHECK 6
%   exists -- BENCHMARK 4 scores a reconstruction, so it is blind to r0 itself;
%   lead the LAUNCH point the way the aim point is led and the book row stays
%   green while check 6 catches it.  The smallest failing margin anywhere in the
%   table is 20000x the band, so the band is nowhere near an edge.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Every book value was grepped from zarchan_decoded.txt and the
%   line number, the band and the band's PROVENANCE are in the row itself --
%   see .source, .band and .detail, and the comment block over benchmarks().
%   Nothing was read off a figure; the extraction turned every figure into an
%   image link.  In order:
%     1-3  LAMBERT3D's launch velocity, three components, against the FIFTEEN
%          DIGITS at :10475 -- Listing 28.3's first line is headed "Obtained
%          from output of Listing C28L2", so it carries this listing's own
%          answer.  The sharpest rows in the file.
%     4    the launch ECI position from the same line.  ⭐ THE PRINTED r0
%          CANNOT BE PRODUCED BY THIS LISTING IN DOUBLE PRECISION (see THE
%          REAL*4 LITERALS above), so the row scores the one evaluation that
%          DOES produce it against the book's fifteen digits, and reports the
%          double-precision reading beside it in an INFO row
%     5-6  Table 28.1's "Integration" row at t = 1000 s (:10469).  ⚠️ Listing
%          28.3 loops "while T <= TF" and so prints the state at 1000 + H;
%          scored at step round(1000/H)+1 for that reason.
%     7-8  "nearly 5000 n miles" and "approximately 900 n miles" (:10457)
%     9    the 2000 s flight time (:10407, :10409)
%     10   impact point vs 36.175 / -115.136 (:10409) -- CLOSURE, not an
%          independent measurement, and the row the delete experiment moves
%     11   the shape of Fig. 28.9 (:10466), scored as an ordering
%     INFO the 3280-vs-Nmi divisor;  SKIP Table 28.1's Kepler column, which
%          belongs to KEPLER1 in Listing 28.3 -- code this port does not have
%
%   OUTPUTS
%     R.t .alt_nmi .dist_km .dist_nmi .long_deg .lat_deg   the listing's arrays
%     R.pos .vel .r0 .v0 .lam .tab281 .delete_rot
%     R.r0_book_form   the REAL*4-literal launch position -- the book's printed
%                      one.  NOT used by the trajectory; see BENCHMARK 4.
%     R.apogee_nmi .t_apogee .t_impact .t_end .downrange_nmi
%     R.impact_lat_deg .impact_long_deg
%     R.bench .selfcheck .pass .verdict .p .units .summary
%
%   ⚠️ NEVER EXECUTED BY ITS AUTHOR -- MATLAB is the caller's.  Every
%   expectation stated here is DERIVED, not measured.
%
%   See also MGS.ZARCHAN.ENGAGE3D (Listing 28.1), MGS.ZARCHAN.LAMBERTEFFICIENT
%   (Listing 13.2, the 2-D ancestor of LAMBERT3D), MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 28.2 line 1, verbatim (zarchan.txt:10409).  Degrees.
    opts.XLONGTDEG (1,1) double {mustBeFinite} = 7.42        % Monte Carlo
    opts.XLATTDEG  (1,1) double {mustBeFinite} = 43.75
    opts.XLATFDEG  (1,1) double {mustBeFinite} = 36.175      % Las Vegas
    opts.XLONGFDEG (1,1) double {mustBeFinite} = -115.136
    opts.TF        (1,1) double {mustBeFinite, mustBePositive} = 2000.0  % s
    opts.H         (1,1) double {mustBeFinite, mustBePositive} = 0.001   % s
    % The listing's "if S >= 9.9999" with H = .001 records every 10 s.
    opts.samp      (1,1) double {mustBeFinite, mustBePositive} = 10.0    % s
    % 0 = the short way round (the driver's value).  1 is the book's other
    % branch; it is transcribed but the book never runs it in this listing.
    opts.SWITCH1   (1,1) double {mustBeMember(opts.SWITCH1,[0 1])} = 0
    % Step for the DELETE EXPERIMENT's two re-flights only.  Coarse on purpose:
    % the effect it measures is 8.3 deg and the coarse-step error is ~1e-3 deg.
    opts.H_delete  (1,1) double {mustBeFinite, mustBePositive} = 0.05    % s
    % THE ONLY SWITCHES.  Not nargin: with an arguments block that declares
    % only name-value options MATLAB reports nargin = 0 however many were
    % passed, so an "if nargin == 0" mode here would never fire.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

% ---- the listing's constants ----------------------------------------------
A       = 2.0926E7;          % ft, Earth radius        (zarchan.txt:10409)
GM      = 1.4077E16;         % ft^3/s^2                (:10409, prose :10375)
W       = -6.283185/86400.;  % rad/s, NEGATIVE         (:10409, prose :10391)
DEGRAD  = 57.3;              % the book's constant.  NOT 180/pi.  See header.
FT_NMI  = 6076.;             % the listing's ALTNM divisor
FT_KM   = 3280.;             % the listing's distance3d divisor
PIBOOK  = 3.1415926535898;   % LAMBERT3D's own literal (:10428)

if opts.H >= opts.samp
    error('mgs:zarchan:icbmTrajectory:samp', ...
        ['H = %g s is not smaller than samp = %g s, so the recorded arrays ' ...
         'would be sampled at the step rate while claiming the coarser ' ...
         'interval in their own time vector.'], opts.H, opts.samp);
end
if opts.TF <= 100*opts.H
    error('mgs:zarchan:icbmTrajectory:short', ...
        'TF = %g s is only %g steps at H = %g s.', opts.TF, opts.TF/opts.H, opts.H);
end

% =========================================================================
%   THE LISTING'S DRIVER, zarchan.txt:10409.
% =========================================================================
[r0, rfEci] = endPoints(opts.XLATTDEG, opts.XLONGTDEG, opts.XLATFDEG, ...
                        opts.XLONGFDEG, opts.TF, A, W, DEGRAD, true);

[v0, lam] = lambert3d(r0(1), r0(2), r0(3), opts.TF, rfEci(1), rfEci(2), ...
                      rfEci(3), opts.SWITCH1, GM, PIBOOK);

% One pass covers everything: the 10-s record the book plots, the Table 28.1
% snapshot at 1000 s, the step-resolution apogee, and the zero-altitude
% crossing.  The book's own stop is "while ALTNM > -1", i.e. 1 nmi UNDER the
% surface, so the run ends a little past TF; that is NOT the flight time and is
% reported separately as R.t_end.
snapStep = round(1000/opts.H) + 1;     % Listing 28.3's "while T <= TF"; header
P = flyBallistic(r0, v0, GM, A, W, opts.H, opts.samp, FT_NMI, FT_KM, ...
                 DEGRAD, -1.0, Inf, snapStep, 1.5*opts.TF);

if numel(P.T) < 5
    % Refused, not warned about.  An empty or near-empty record returned
    % quietly looks like a converged result, and the energy / angular-momentum
    % invariants below would then index P.pos(1) into an empty array and throw
    % far from the samp value that caused it.
    error('mgs:zarchan:icbmTrajectory:empty', ...
        ['the run recorded %d sample(s) at TF = %g s, samp = %g s, H = %g s. ' ...
         'At least five are needed before any shape or invariant check means ' ...
         'anything.'], numel(P.T), opts.TF, opts.samp, opts.H);
end

R = struct();
R.listing  = 'Zarchan Listing 28.2 (zarchan.txt:10409-10456, pp.730-735)';
R.t        = P.T;
R.alt_nmi  = P.altnm;                       % the listing's ArrayALTNM
R.dist_km  = P.distkm;                      % the listing's ArrayDISTRTNM
R.dist_nmi = P.distkm*FT_KM/FT_NMI;         % what the axis LABEL claims
R.long_deg = P.longdeg;
R.lat_deg  = P.latdeg;
R.pos      = P.pos;
R.vel      = P.vel;
R.r0       = r0;
% The SAME closed form with its literals read as IEEE single -- what produced
% the fifteen digits at :10475.  Built for the benchmark only; nothing
% downstream may use it, or the port would stop being a port of the listing.
R.r0_book_form = real4LaunchPosition(opts.XLATTDEG, opts.XLONGTDEG, A, DEGRAD);
R.v0       = v0;
R.lam      = lam;
R.t_end    = P.t_end;
R.t_apogee = P.apogee_t;
R.apogee_nmi      = max(P.altnm);           % from the BOOK's own array
R.apogee_nmi_fine = (P.apogee_r - A)/FT_NMI;
R.t_impact = P.t_cross;

% Impact point: the corrected state interpolated to zero altitude, rotated into
% ECEF at that same instant.  (The recorded array's last entry is already below
% the surface -- the listing keeps flying to -1 nmi.)
if isfinite(P.t_cross)
    ecefI = eci2ecef(P.pos_cross, W, P.t_cross);
    [R.impact_long_deg, R.impact_lat_deg] = ecef2ll(ecefI, DEGRAD);
    R.downrange_nmi = distance3dKm(ecefI(1), ecefI(2), ecefI(3), ...
                                   r0(1), r0(2), r0(3), A, FT_KM)*FT_KM/FT_NMI;
else
    R.impact_long_deg = NaN;  R.impact_lat_deg = NaN;  R.downrange_nmi = NaN;
end

% Table 28.1's "Integration" row, in the book's km via /3280.
R.tab281 = struct( ...
    'pos_km',      P.snap_pos/FT_KM,      'vel_kms',      P.snap_vel/FT_KM, ...
    'pos_km_prev', P.snap_prev_pos/FT_KM, 'vel_kms_prev', P.snap_prev_vel/FT_KM, ...
    't',           P.snap_t,              't_prev',       P.snap_prev_t, ...
    'book_pos_km',  [2547.4175 -3583.1082 6758.2423], ...
    'book_vel_kms', [-3.558504 -3.245617 -0.379447]);

% ---- THE DELETE EXPERIMENT.  Run here, in the open, not buried in a check. --
R.delete_rot = deleteRotation(opts, A, GM, W, DEGRAD, FT_NMI, FT_KM, PIBOOK);

R.p = struct('XLONGTDEG',opts.XLONGTDEG, 'XLATTDEG',opts.XLATTDEG, ...
             'XLATFDEG',opts.XLATFDEG, 'XLONGFDEG',opts.XLONGFDEG, ...
             'TF',opts.TF, 'H',opts.H, 'samp',opts.samp, ...
             'SWITCH1',opts.SWITCH1, 'H_delete',opts.H_delete, ...
             'A',A, 'GM',GM, 'W',W, 'DEGRAD',DEGRAD, 'nsteps',P.nsteps);
R.units = ['ft, ft/s (ECI integration); nmi = 6076 ft; km = 3280 ft ' ...
           '(the book''s divisor, not 3280.84); deg = 57.3*rad; ' ...
           'gm = 1.4077e16 ft^3/s^2, a = 2.0926e7 ft'];

R.selfcheck = structuralChecks(R, P, GM, A, opts.TF, FT_NMI);

bookCase = isBookCase(opts);
if opts.benchmark
    R.bench = benchmarks(R, opts, bookCase);
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
%      the reaction to it.  benchSuite runs every port with assert off so that
%      one red row cannot withhold the evidence for the rest of the board.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the book. ' ...
           'R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = ['not one book value could be evaluated: this run is off Listing ' ...
           '28.2''s own case (Monte Carlo 43.75/7.42 -> Las Vegas ' ...
           '36.175/-115.136, TF = 2000 s, SWITCH1 = 0).  An unvalidated port ' ...
           'must not be handed back silently.'];
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question. ' ...
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
        error('mgs:zarchan:icbmTrajectory:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:icbmTrajectory:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [r0, rf] = endPoints(latTdeg, longTdeg, latFdeg, longFdeg, TF, A, W, ...
                              DEGRAD, doRotate)
%ENDPOINTS  zarchan.txt:10409, the lines before the LAMBERT3D call.
%
%   doRotate = false is the DELETE EXPERIMENT: it removes
%   "XLONGF = XLONGF - W*TF", the line that aims at where the destination WILL
%   be.  Everything downstream still runs, which is the entire point.
XLONGF = longFdeg/DEGRAD;
XLATF  = latFdeg/DEGRAD;
XLONGT = longTdeg/DEGRAD;
XLATT  = latTdeg/DEGRAD;
if doRotate
    XLONGF = XLONGF - W*TF;   % <<< THE LINE.  W is negative; this ADDS 8.33 deg.
end
rf = [A*cos(XLATF)*cos(XLONGF); A*cos(XLATF)*sin(XLONGF); A*sin(XLATF)];
r0 = [A*cos(XLATT)*cos(XLONGT); A*cos(XLATT)*sin(XLONGT); A*sin(XLATT)];
end


% =============================================================================
function r = doubleLaunchPosition(latDeg, lonDeg, A, DEGRAD)
%DOUBLELAUNCHPOSITION  Listing 28.2's launch-point closed form (:10409) written
%   out a second time from the option values alone.  The duplication is the
%   point: structural check 6 compares endPoints()'s r0 against this, so a
%   rotation or a convention that crept into endPoints() has somewhere to show.
la = latDeg/DEGRAD;  lo = lonDeg/DEGRAD;
r  = [A*cos(la)*cos(lo); A*cos(la)*sin(lo); A*sin(la)];
end


% =============================================================================
function r = real4LaunchPosition(latDeg, lonDeg, A, DEGRAD)
%REAL4LAUNCHPOSITION  The same closed form with every literal it consumes
%   rounded to IEEE single FIRST, then evaluated in double -- a FORTRAN untyped
%   constant meeting a REAL*8 expression.  THIS, and not the double reading, is
%   what produced the fifteen digits published at zarchan.txt:10475; see THE
%   REAL*4 LITERALS in the header for the two-equation identification.
%
%   ⚠️ NOT USED BY THE TRAJECTORY, and it must never be.  R.r0, LAMBERT3D and
%   the integration all stay double, because double is the correct evaluation
%   of the source as the book prints it.  This function exists so the book's
%   number can be SCORED, not so the port can be bent onto it.
%
%   single(43.75) and single(2.0926e7) are exact, so the rounding reaches the
%   latitude only through 57.3 while the longitude takes both errors.  That
%   asymmetry is the signature, and removing any one of the three s() calls
%   below moves the result 0.02 to 0.15 ft -- four to five orders outside
%   BENCHMARK 4's band.
s  = @(x) double(single(x));
la = s(latDeg)/s(DEGRAD);  lo = s(lonDeg)/s(DEGRAD);
r  = [s(A)*cos(la)*cos(lo); s(A)*cos(la)*sin(lo); s(A)*sin(la)];
end


% =============================================================================
function [V, lam] = lambert3d(XT, YT, ZT, TF, XF, YF, ZF, SWITCH1, GM, PI)
%LAMBERT3D  zarchan.txt:10428-10456, transcribed line for line.
%
%   Every constant arrives as an ARGUMENT.  MATLAB local functions cannot see
%   the caller's workspace, and a previous wave in this package lost a whole
%   run to exactly that.
%
%   A secant / false-position hybrid on the launch flight-path angle GAMMA,
%   bracketed by [G_MIN, G_MAX], G_MAX = pi/2.  Flight time DECREASES with
%   GAMMA over that bracket, which is what fixes the direction of the three
%   bracket tests -- see the header.
DEG_PER_RAD  = 57.3;      %#ok<NASGU>  the book assigns these and never reads
EARTH_RADIUS = 2.0926E7;  %#ok<NASGU>  them; kept so the page still diffs
FT_PER_KM    = 3280.;     %#ok<NASGU>
RAD_PER_SEC  = 1/57.3;    %#ok<NASGU>
HALFPI       = PI/2.;

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
    G_MIN = HALFPI - acos(R0DOTRF0/(R0MAG*RF0MAG));
    G_MAX = HALFPI;
    THETA = acos(COS_T);
else
    G_MIN = -HALFPI;
    G_MAX = -HALFPI + acos(R0DOTRF0/(R0MAG*RF0MAG));
    THETA = 2.*PI - acos(COS_T);
end
SIN_T     = sin(THETA);
COT_HALFT = 1./tan(THETA/2.);
GAMMA = (G_MAX+G_MIN)/2.;
GOLD  = G_MIN;  TOLD = 0.;  T = 0.;  ITERS = 1;
S  = .5*(R0MAG+RFMAG+RF0MAG);           % read only by BL and BT, which are not
BL = sqrt(R0MAG*RFMAG)*cos(THETA/2.)/S;    %#ok<NASGU>  a dead Battin-style
BT = sqrt(8.*GM/(S*S*S))*TF;               %#ok<NASGU>  bound; kept, see above
RV0MAG = NaN;

while (abs(TF-T) > (.00000001*TF))
    SIN_G = sin(GAMMA);  COS_G = cos(GAMMA);  TAN_G = SIN_G/COS_G;
    COS_TPLUSG = cos(THETA+GAMMA-2.*PI);
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
    % The bracket.  T > TF means GAMMA was too SMALL, so GAMMA becomes the new
    % UPPER bound; T < TF moves the lower bound up.  See the header for why
    % that direction is forced and how it was confirmed arithmetically.
    if T > TF && GAMMA < G_MAX
        G_MAX = GAMMA;
    end
    if T < 0. && GAMMA < G_MAX
        G_MAX = GAMMA;
    end
    if T < TF && GAMMA > G_MIN
        G_MIN = GAMMA;  T_MIN = T;
    end
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
    if ITERS > 100
        break
    end
end

% The book writes a literal "GAMMA=GAMMA;" in the SWITCH1 == 0 branch.  It is
% a no-op and is recorded here rather than reproduced.
if SWITCH1 == 0
    ANGLE = HALFPI-GAMMA;
else
    ANGLE = GAMMA-HALFPI;
end
SINA = sin(ANGLE);  COSA = cos(ANGLE);
V1X = XT;  V1Y = YT;  V1Z = ZT;
V2X = XF;  V2Y = YF;  V2Z = ZF;
MAG1   = sqrt(V1X*V1X+V1Y*V1Y+V1Z*V1Z);
DOTMAG = V1X*V2X+V1Y*V2Y+V1Z*V2Z;
CROSSX = V1Y*V2Z-V1Z*V2Y;
CROSSY = V1Z*V2X-V1X*V2Z;
CROSSZ = V1X*V2Y-V1Y*V2X;
CROSSMAG = sqrt(CROSSX*CROSSX+CROSSY*CROSSY+CROSSZ*CROSSZ);
C2 = MAG1*SINA/CROSSMAG;
C1 = COSA/MAG1-DOTMAG*C2/(MAG1*MAG1);
VUNITX = C2*V2X + C1*V1X;
VUNITY = C2*V2Y + C1*V1Y;
VUNITZ = C2*V2Z + C1*V1Z;

V   = [RV0MAG*VUNITX; RV0MAG*VUNITY; RV0MAG*VUNITZ];
lam = struct('gamma', GAMMA, 'gamma_deg', 57.3*GAMMA, 'iters', ITERS, ...
             'tf', T, 'tf_err', TF-T, 'converged', ITERS <= 100, ...
             'gmin_deg', 57.3*G_MIN, 'gmax_deg', 57.3*G_MAX, ...
             'theta_deg', 57.3*THETA, 'v0mag', RV0MAG);
end


% =============================================================================
function P = flyBallistic(r0, v0, GM, A, W, H, samp, FT_NMI, FT_KM, DEGRAD, ...
                          altStopNmi, maxSteps, snapStep, tGuess)
%FLYBALLISTIC  The driver's integration loop, zarchan.txt:10410-10421.
%
%   Zarchan's two-pass predictor loop: pass 1 evaluates the derivatives at the
%   current state; pass 2 takes a full Euler step, advances T by H, and
%   re-evaluates there; the corrector then averages.  T advances INSIDE the
%   inner loop, exactly once, which is what makes the second evaluation happen
%   at t+H -- move that line out and the scheme degrades to first order while
%   still running perfectly smoothly.
%
%   ⚠️ ALTNM is computed inside the inner loop, so the altitude recorded (and
%   the altitude the OUTER loop tests) is the EULER-PREDICTED one, while
%   DISTRTNM beside it comes from the CORRECTED state.  That is the book's and
%   it is kept.  The apogee and the zero crossing are tracked from the
%   corrected state at step resolution and reported separately.
XT = r0(1);  YT = r0(2);  ZT = r0(3);
XTD = v0(1); YTD = v0(2); ZTD = v0(3);
XTINIT = XT; YTINIT = YT; ZTINIT = ZT;
XTDD = 0.; YTDD = 0.; ZTDD = 0.;    % pass 1 sets these before pass 2 reads them
T = 0.;  S = 0.;
ALTNM = (sqrt(XT^2+YT^2+ZT^2)-A)/FT_NMI;

nmax = ceil(tGuess/samp) + 8;
P = struct();
P.T = zeros(nmax,1);  P.altnm = zeros(nmax,1);  P.distkm = zeros(nmax,1);
P.longdeg = zeros(nmax,1);  P.latdeg = zeros(nmax,1);
P.pos = zeros(nmax,3);  P.vel = zeros(nmax,3);
n = 0;  k = 0;
P.apogee_r = 0;  P.apogee_t = NaN;
P.t_cross = NaN;  P.pos_cross = [NaN;NaN;NaN];
P.snap_pos = nan(3,1);  P.snap_vel = nan(3,1);  P.snap_t = NaN;
P.snap_prev_pos = nan(3,1);  P.snap_prev_vel = nan(3,1);  P.snap_prev_t = NaN;
prevAlt = 0;  prevPos = [XT;YT;ZT];  prevT = 0;  crossed = false;

while ALTNM > altStopNmi && k < maxSteps
    XTOLD = XT;  YTOLD = YT;  ZTOLD = ZT;
    XTDOLD = XTD;  YTDOLD = YTD;  ZTDOLD = ZTD;
    STEP = 1;  FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            XT = XT+H*XTD;  YT = YT+H*YTD;  ZT = ZT+H*ZTD;
            XTD = XTD+H*XTDD;  YTD = YTD+H*YTDD;  ZTD = ZTD+H*ZTDD;
            T = T+H;
        end
        TEMPBOTT = (XT^2+YT^2+ZT^2)^1.5;
        XTDD = -GM*XT/TEMPBOTT;
        YTDD = -GM*YT/TEMPBOTT;
        ZTDD = -GM*ZT/TEMPBOTT;
        ALTNM = (sqrt(XT^2+YT^2+ZT^2)-A)/FT_NMI;
        FLAG = 1;
    end
    % The listing writes "FLAG = 0" here as well.  The outer loop re-initialises
    % FLAG on its next pass, so the assignment cannot be read; kept as a comment
    % rather than as code so nobody re-adds it thinking a state machine was lost.
    XT  = .5*(XTOLD+XT+H*XTD);
    YT  = .5*(YTOLD+YT+H*YTD);
    ZT  = .5*(ZTOLD+ZT+H*ZTD);
    XTD = .5*(XTDOLD+XTD+H*XTDD);
    YTD = .5*(YTDOLD+YTD+H*YTDD);
    ZTD = .5*(ZTDOLD+ZTD+H*ZTDD);
    k = k + 1;

    rNow   = sqrt(XT^2+YT^2+ZT^2);
    altNow = rNow - A;
    if rNow > P.apogee_r
        P.apogee_r = rNow;  P.apogee_t = T;
    end
    % Zero-altitude crossing, AFTER apogee only: the launch point is also at
    % zero altitude, so a bare sign test fires on step 1.
    if ~crossed && ~isnan(P.apogee_t) && T > P.apogee_t && altNow <= 0
        f = prevAlt/max(prevAlt - altNow, realmin);
        P.t_cross   = prevT + f*H;
        P.pos_cross = prevPos + f*([XT;YT;ZT] - prevPos);
        crossed = true;
    end
    if k == snapStep-1
        P.snap_prev_pos = [XT;YT;ZT];  P.snap_prev_vel = [XTD;YTD;ZTD];
        P.snap_prev_t = T;
    elseif k == snapStep
        P.snap_pos = [XT;YT;ZT];  P.snap_vel = [XTD;YTD;ZTD];  P.snap_t = T;
    end
    prevAlt = altNow;  prevPos = [XT;YT;ZT];  prevT = T;

    S = S + H;
    if S >= samp - 0.5*H
        S = 0.;
        ecef = eci2ecef([XT;YT;ZT], W, T);
        [lonDeg, latDeg] = ecef2ll(ecef, DEGRAD);
        n = n + 1;
        if n > nmax                       % guard; tGuess should prevent it
            break
        end
        P.T(n) = T;
        P.altnm(n) = ALTNM;               % the book's: the PREDICTED altitude
        P.distkm(n) = distance3dKm(ecef(1), ecef(2), ecef(3), ...
                                   XTINIT, YTINIT, ZTINIT, A, FT_KM);
        P.longdeg(n) = lonDeg;  P.latdeg(n) = latDeg;
        P.pos(n,:) = [XT YT ZT];  P.vel(n,:) = [XTD YTD ZTD];
    end
end

P.T = P.T(1:n);  P.altnm = P.altnm(1:n);  P.distkm = P.distkm(1:n);
P.longdeg = P.longdeg(1:n);  P.latdeg = P.latdeg(1:n);
P.pos = P.pos(1:n,:);  P.vel = P.vel(1:n,:);
P.t_end = T;  P.nsteps = k;
P.state_end = [XT;YT;ZT;XTD;YTD;ZTD];
end


% =============================================================================
function DISTKM = distance3dKm(XT, YT, ZT, XF, YF, ZF, A, FT_KM)
%DISTANCE3DKM  zarchan.txt:10423, verbatim.  ⚠️ The book calls the result
%   DISTKM and divides by 3280 (ft per km), then the driver stores it in
%   DISTRTNM and labels the axis "Nmi".  Kept as printed; the caller converts.
R  = sqrt(XT^2+YT^2+ZT^2);
RF = sqrt(XF^2+YF^2+ZF^2);
CBETA = (XT*XF+YT*YF+ZT*ZF)/(R*RF);
if CBETA <= 1.
    BETA = acos(CBETA);
    DISTKM = A*BETA/FT_KM;
else
    DISTKM = (XF-XT)/FT_KM;    % the book's numerical-overshoot fallback
end
end


% =============================================================================
function e = eci2ecef(x, W, T)
%ECI2ECEF  zarchan.txt:10420.  W is NEGATIVE in this listing, which is why the
%   signs read backwards from the usual textbook rotation.
e = [x(1)*cos(W*T) - x(2)*sin(W*T);
     x(1)*sin(W*T) + x(2)*cos(W*T);
     x(3)];
end

function [lonDeg, latDeg] = ecef2ll(e, DEGRAD)
%ECEF2LL  zarchan.txt:10420.  DEGRAD is the book's 57.3, NOT 180/pi.
latDeg = DEGRAD*atan2(e(3), sqrt(e(1)^2+e(2)^2));
lonDeg = DEGRAD*atan2(e(2), e(1));
end


% =============================================================================
function D = deleteRotation(opts, A, GM, W, DEGRAD, FT_NMI, FT_KM, PIBOOK)
%DELETEROTATION  THE DELETE EXPERIMENT, run in the open.
%
%   Removes "XLONGF = XLONGF - W*TF" and re-flies.  BOTH re-flights use the
%   coarse step H_delete, so they are compared against each other and not
%   against the fine run: the effect is 8.3 deg of longitude, and the coarse
%   step costs ~1e-3 deg -- four orders below it.
Hd = opts.H_delete;
nSteps = round(opts.TF/Hd);
D = struct('H', Hd, 'lon_with', NaN, 'lon_without', NaN, ...
           'err_with_deg', NaN, 'err_without_deg', NaN, 'shift_deg', NaN, ...
           'earth_turn_deg', abs(W)*opts.TF*DEGRAD);
for withRot = [true false]
    [q0, qf] = endPoints(opts.XLATTDEG, opts.XLONGTDEG, opts.XLATFDEG, ...
                         opts.XLONGFDEG, opts.TF, A, W, DEGRAD, withRot);
    v = lambert3d(q0(1), q0(2), q0(3), opts.TF, qf(1), qf(2), qf(3), ...
                  opts.SWITCH1, GM, PIBOOK);
    Q = flyBallistic(q0, v, GM, A, W, Hd, opts.TF, FT_NMI, FT_KM, DEGRAD, ...
                     -Inf, nSteps, 0, 2*opts.TF);
    e = eci2ecef(Q.state_end(1:3), W, Q.t_end);
    lon = ecef2ll(e, DEGRAD);
    if withRot
        D.lon_with = lon;
        D.err_with_deg = abs(lon - opts.XLONGFDEG);
    else
        D.lon_without = lon;
        D.err_without_deg = abs(lon - opts.XLONGFDEG);
    end
end
D.shift_deg = abs(D.lon_with - D.lon_without);
end


% =============================================================================
function K = structuralChecks(R, P, GM, A, TF, FT_NMI)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan -- they are the weaker
%   claim that it agrees with itself, which is what catches wiring faults.
%
%   ⚠️ .names AND .oks ARE THE CONTRACT.  benchSuite only puts a structural row
%   on the board when it finds BOTH, and a delete experiment nobody can see is
%   the same as a delete experiment that never ran.
K = struct();
names = {};  oks = [];  detail = {};

% 1  LAMBERT CLOSURE.  The routine's entire job: fly its velocity for TF and
%    arrive at the aim point.  Measured in feet against the ECI aim point.
[~, rfEci] = endPoints(R.p.XLATTDEG, R.p.XLONGTDEG, R.p.XLATFDEG, ...
                       R.p.XLONGFDEG, TF, A, R.p.W, R.p.DEGRAD, true);
jTF = find(P.T >= TF - 0.5*R.p.samp, 1, 'first');
if isempty(jTF)
    closeErr = Inf;
else
    % rfEci is 3x1 and P.pos(jTF,:).' is 3x1.  Transposing either one here
    % would broadcast into a 3x3 and norm() would happily return its largest
    % singular value -- a plausible-looking number, silently wrong.
    closeErr = norm(P.pos(jTF,:).' - rfEci);
end
K.lambert_close_ft = closeErr;
names{end+1} = 'LAMBERT3D closes: position at TF == the ECI aim point';
oks(end+1)   = closeErr <= 1000;      % ft; 3e-5 of a 9400 km flight
detail{end+1} = sprintf('|r(TF) - r_aim| = %.1f ft (%.4f nmi), want <= 1000 ft', ...
    closeErr, closeErr/FT_NMI);

% 2  Two-body specific energy is conserved.  Catches a wrong GM, a sign error
%    on the gravity term, or a corrector that quietly became plain Euler.
sp = sqrt(sum(P.pos.^2, 2));
en = 0.5*sum(P.vel.^2, 2) - GM./sp;
K.energy_drift = max(abs(en - en(1)))/abs(en(1));
names{end+1} = 'specific orbital energy conserved along the trajectory';
oks(end+1)   = K.energy_drift <= 1e-6;
detail{end+1} = sprintf('max relative drift %.3e (E0 = %.6e ft^2/s^2)', ...
    K.energy_drift, en(1));

% 3  Angular momentum is conserved: gravity is central, so r x v cannot move.
hv = cross(P.pos, P.vel, 2);
hm = sqrt(sum(hv.^2, 2));
K.h_drift = max(abs(hm - hm(1)))/hm(1);
names{end+1} = 'angular momentum |r x v| conserved (central force)';
oks(end+1)   = K.h_drift <= 1e-6;
detail{end+1} = sprintf('max relative drift %.3e', K.h_drift);

% 4  ⭐ DELETE EXPERIMENT: remove "XLONGF = XLONGF - W*TF" and the impact point
%    must walk by one Earth-turn of longitude.  Deleting that line breaks
%    NOTHING that shows: same arch, same flight time, same apogee.
D = R.delete_rot;
K.delete_shift_deg = D.shift_deg;
names{end+1} = 'DELETE the Earth-rotation lead -> impact misses by ~w*TF';
oks(end+1)   = D.err_with_deg < 0.05 && D.err_without_deg > 5 && ...
               abs(D.shift_deg - D.earth_turn_deg) < 0.5;
detail{end+1} = sprintf(['long error %.4f deg with the line, %.3f deg without; ' ...
    'shift %.3f deg vs w*TF = %.3f deg (H = %g s)'], D.err_with_deg, ...
    D.err_without_deg, D.shift_deg, D.earth_turn_deg, D.H);

% 5  Launch and impact are at the SAME radius, so the two-body arc is symmetric
%    and apogee must sit at TF/2.  This is also what makes the Table 28.1
%    snapshot at 1000 s an apogee state, which is why it has |zdot| ~ 0.38 km/s.
K.apogee_offset = P.apogee_t - TF/2;
names{end+1} = 'apogee at TF/2 (equal launch/impact radii => symmetric arc)';
oks(end+1)   = abs(K.apogee_offset) <= max(0.05, 2*R.p.H);
detail{end+1} = sprintf('t_apogee = %.4f s, TF/2 = %.1f s, offset %+.4f s', ...
    P.apogee_t, TF/2, K.apogee_offset);

% 6  r0 IS the listing's unrotated closed form.  Re-derived here from the
%    option values alone, deliberately a SECOND copy of the expression and
%    independent of endPoints().  BENCHMARK 4 is scored against a
%    reconstruction, so without this the port could compute r0 any way it liked
%    -- lead the LAUNCH point by -W*TF, say, which endPoints() is one argument
%    away from doing -- and the book row would stay green regardless.
rD = doubleLaunchPosition(R.p.XLATTDEG, R.p.XLONGTDEG, A, R.p.DEGRAD);
K.r0_form_ft = max(abs(R.r0(:) - rD(:)));
names{end+1} = 'r0 == a*[cos lat cos long, cos lat sin long, sin lat], unrotated';
oks(end+1)   = K.r0_form_ft <= 1e-9;
detail{end+1} = sprintf(['max |r0 - closed form| = %.3e ft, re-derived from the ' ...
    'options rather than from endPoints(); |r0| - a = %+.3e ft (a sphere, so ' ...
    'this is identically zero)'], K.r0_form_ft, norm(R.r0) - A);

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
tf = abs(o.XLONGTDEG - 7.42) < 1e-12 && abs(o.XLATTDEG - 43.75) < 1e-12 && ...
     abs(o.XLATFDEG - 36.175) < 1e-12 && abs(o.XLONGFDEG + 115.136) < 1e-12 && ...
     abs(o.TF - 2000) < 1e-9 && o.SWITCH1 == 0;
end


% =============================================================================
function B = benchmarks(R, opts, bookCase)
%BENCHMARKS  Computed vs published, with the PROVENANCE of every band.
%
%   Every book value here was grepped out of zarchan_decoded.txt and the line
%   number is in the row's .source.  Nothing was read off a figure -- the
%   extraction turned every figure into an image link.  Where the book states a
%   SHAPE about a figure, that prose IS greppable and is scored as an ordering.
%
%   Bands and what each one means:
%     'listing'   1e-5 relative -- against a 15-digit published number
%     'table'     absolute, set by the book's own Integration-vs-Kepler gap
%     'prose'     5%      -- "nearly 5000", "approximately 900"
%     'spec'      0.01%   -- a number the program is SOLVING for (TF = 2000 s)
%     'bound'     absolute closure limits
%     'shape'     an ordering; no tolerance is possible
%
%   Do NOT widen a band to turn a row green.  Bands come from the provenance of
%   the book value, not from what we happen to compute.
B = emptyBench();
whyOff = sprintf(['off Listing 28.2''s own case: this run used %g/%g -> ' ...
    '%g/%g deg, TF = %g s, SWITCH1 = %g'], opts.XLATTDEG, opts.XLONGTDEG, ...
    opts.XLATFDEG, opts.XLONGFDEG, opts.TF, opts.SWITCH1);
SRC_L283 = 'Listing 28.3 header, p.737 (zarchan.txt:10475)';
SRC_T281 = 'Table 28.1, "Integration" row, p.736 (zarchan.txt:10469)';
SRC_FIG9 = 'Fig. 28.9 prose, p.735 (zarchan.txt:10457)';

BOOK_R0 = [14990432.9744621; 1952093.10305573; 14469752.1663352];
BOOK_V0 = [996.773566434768; -14954.8124604715; 17528.1931768263];
comp = {'X','Y','Z'};

% ---- 1.  LAMBERT3D's launch velocity, three components ---------------------
for i = 1:3
    what = sprintf('LAMBERT3D launch velocity %sTD', comp{i});
    if bookCase
        B = addVal(B, what, BOOK_V0(i), R.v0(i), 'listing', 1e-5, SRC_L283, ...
            'ft/s', ['Listing 28.3 is SEEDED with Listing 28.2''s output ' ...
            '("Obtained from output of Listing C28L2"), so this is 28.2''s ' ...
            'own answer printed to 15 digits']);
    else
        B = addSkip(B, what, sprintf('%.12g', BOOK_V0(i)), whyOff, SRC_L283, 'ft/s');
    end
end

% ---- 2.  the launch position: A*cos/sin with the book's 57.3 ---------------
%   ⭐ Scored through the REAL*4 form, which is the ONLY evaluation of this
%   listing that produces the published digits -- see THE REAL*4 LITERALS in
%   the header for the two-equation identification.  ⚠️ THE BAND EDGE IS NOT
%   FITTED AND WAS NOT WIDENED TO PASS ANYTHING.  It is the resolution of the
%   published number itself (fifteen significant digits on 14990432.9744621 is
%   1e-7 ft), taken one decade loose so a platform's last-ulp trig cannot flip
%   the row.  Nothing between 1e-6 and 1e-1 ft changes a verdict here: the
%   correct form lands at 3.7e-8 ft and every rival geometry at 1e-1 to 4e6 ft.
if bookCase
    rS = real4LaunchPosition(opts.XLATTDEG, opts.XLONGTDEG, R.p.A, R.p.DEGRAD);
    e  = max(abs(rS(:) - BOOK_R0));
    eD = max(abs(R.r0(:) - BOOK_R0));
    B = addBound(B, 'launch ECI position (3 components)', ...
        'max |error| <= 1e-6 ft', sprintf('%.3e ft', e), e <= 1e-6, ...
        SRC_L283, 'ft', sprintf(['the book''s fifteen digits, reproduced by ' ...
        'Listing 28.2''s own closed form with the literals it consumes ' ...
        '(43.75, 7.42, 57.3) read as IEEE single -- a FORTRAN untyped ' ...
        'constant inside a REAL*8 expression.  This port''s DOUBLE r0 is ' ...
        '%.4f ft from the same digits; that is the INFO row below, not a ' ...
        'hidden allowance.  The row still pins a = 2.0926e7, the 57.3 ' ...
        'convention, the spherical GEOCENTRIC latitude and the right-handed ' ...
        'axes -- MEASURED by injecting each: 180/pi gives 8.500e2 ft, ' ...
        'geodetic latitude 5.079e4 ft, a negated Y 3.904e6 ft, a+1 ft gives ' ...
        '7.164e-1 ft, and dropping the single rounding on 7.42 ALONE gives ' ...
        '1.996e-2 ft -- 4 to 12 orders outside this band'], eD));
    B = addInfo(B, 'the same r0 evaluated in double precision', ...
        'the listing exactly as printed', ...
        sprintf('%.4f ft from the book (%.3e relative); REAL*4 form %.3e ft', ...
        eD, eD/R.p.A, e), SRC_L283, ...
        ['NOT a tolerance question and NOT an error in this port: no ' ...
         'double-precision evaluation of "XLATT = XLATTDEG/57.3" can land on ' ...
         'the published digits, because they were not produced by one.  The ' ...
         'residual puts the book''s launch point 6.5 cm north and 1.4 cm ' ...
         'east of ours, matching -rel(single(57.3)) in latitude and ' ...
         'rel(single(7.42))-rel(single(57.3)) in longitude to seven figures. ' ...
         'R.r0 stays the double value on purpose']);
else
    B = addSkip(B, 'launch ECI position (3 components)', ...
        '14990432.97 / 1952093.10 / 14469752.17', whyOff, SRC_L283, 'ft');
end

% ---- 3.  Table 28.1, the "Integration" row ---------------------------------
T = R.tab281;
fineEnough = opts.H <= 0.002;
whyH = sprintf(['H = %g s.  The row is skipped above 0.002 s because the ' ...
    'O(H^2) global error then exceeds the 0.01 km band, and the row would be ' ...
    'measuring the caller''s step size rather than the port'], opts.H);
if bookCase && fineEnough && all(isfinite(T.pos_km))
    ep = max(abs(T.pos_km(:).' - T.book_pos_km));
    ev = max(abs(T.vel_kms(:).' - T.book_vel_kms));
    epPrev = max(abs(T.pos_km_prev(:).' - T.book_pos_km));
    B = addBound(B, 'Table 28.1 position at t = 1000 s (3 components)', ...
        'max |error| <= 0.01 km', sprintf('%.5f km', ep), ep <= 0.01, ...
        SRC_T281, 'km', sprintf(['scored at step round(1000/H)+1, the state ' ...
        'Listing 28.3''s "while T <= TF" actually prints (t = %.4f s); the ' ...
        'step before gives %.5f km.  Band = the book''s own ' ...
        'Integration-vs-Kepler gap (0.0039 km) plus the one-step ambiguity ' ...
        '(0.0036 km)'], T.t, epPrev));
    B = addBound(B, 'Table 28.1 velocity at t = 1000 s (3 components)', ...
        'max |error| <= 2e-5 km/s', sprintf('%.3e km/s', ev), ev <= 2e-5, ...
        SRC_T281, 'km/s', ['same snapshot; band = the book''s own ' ...
        'Integration-vs-Kepler gap (8e-6 km/s) plus the one-step ambiguity']);
else
    if bookCase
        why = whyH;
    else
        why = whyOff;
    end
    B = addSkip(B, 'Table 28.1 position at t = 1000 s (3 components)', ...
        '2547.4175 / -3583.1082 / 6758.2423', why, SRC_T281, 'km');
    B = addSkip(B, 'Table 28.1 velocity at t = 1000 s (3 components)', ...
        '-3.558504 / -3.245617 / -0.379447', why, SRC_T281, 'km/s');
end

% ---- 4.  the two prose readings about Fig. 28.9 ----------------------------
if bookCase
    B = addVal(B, 'downrange Monte Carlo -> Las Vegas', 5000, R.downrange_nmi, ...
        'prose', 0.05, SRC_FIG9, 'nmi', ...
        ['"nearly 5000 n miles" -- one significant figure of prose, hence a ' ...
         '5% band.  ⚠️ computed from the listing''s own distance3d with its ' ...
         '/3280 rescaled to /6076; see the INFO row']);
    B = addVal(B, 'target apogee', 900, R.apogee_nmi, 'prose', 0.05, ...
        SRC_FIG9, 'nmi', ['"approximately 900 n miles", read off the ' ...
        'listing''s own ArrayALTNM (the predictor-state altitude it plots)']);
else
    B = addSkip(B, 'downrange Monte Carlo -> Las Vegas', '5000', whyOff, ...
        SRC_FIG9, 'nmi');
    B = addSkip(B, 'target apogee', '900', whyOff, SRC_FIG9, 'nmi');
end

% ---- 5.  the specified flight time -----------------------------------------
if isfinite(R.t_impact)
    B = addVal(B, 'time to reach the destination (altitude = 0)', opts.TF, ...
        R.t_impact, 'spec', 1e-4, ...
        'prose p.729 + listing line 1 (zarchan.txt:10407, :10409)', 's', ...
        sprintf(['LAMBERT3D converges to |TF-T| <= 1e-8*TF = %.1e s and the ' ...
        'Heun error over TF at H = %g s is far below that, so 0.01%% is four ' ...
        'orders of margin.  The RUN ends later, at %.3f s, because the ' ...
        'listing flies on to -1 nmi -- that is not the flight time'], ...
        1e-8*opts.TF, opts.H, R.t_end));
else
    B = addSkip(B, 'time to reach the destination (altitude = 0)', ...
        sprintf('%g', opts.TF), ...
        'the altitude never crossed zero after apogee', ...
        'zarchan.txt:10407', 's');
end

% ---- 6.  impact-point closure ----------------------------------------------
if bookCase && isfinite(R.impact_lat_deg)
    dLat = abs(R.impact_lat_deg - opts.XLATFDEG);
    dLon = abs(R.impact_long_deg - opts.XLONGFDEG);
    B = addBound(B, 'impact point == Las Vegas (36.175 N, 115.136 W)', ...
        'max |error| <= 0.01 deg', ...
        sprintf('%.4f deg (lat %.4f, long %.4f)', max(dLat,dLon), dLat, dLon), ...
        max(dLat,dLon) <= 0.01, 'Listing 28.2 line 1 (zarchan.txt:10409)', ...
        'deg', ['CLOSURE, not an independent measurement: the Lambert solve, ' ...
        'the ECI integration and the ECI->ECEF rotation all have to agree at ' ...
        'once.  The delete experiment moves this row by 8.3 deg']);
else
    B = addSkip(B, 'impact point == Las Vegas (36.175 N, 115.136 W)', ...
        '36.175 / -115.136', whyOff, 'zarchan.txt:10409', 'deg');
end

% ---- 7.  the shape of Fig. 28.9 --------------------------------------------
alt = R.alt_nmi;
if numel(alt) >= 5
    [pk, ipk] = max(alt);
    interior = ipk > 1 && ipk < numel(alt);
    % Non-strict on purpose.  Near apogee consecutive samples differ by
    % ~0.5*|rddot|*samp^2, which at samp = H is a few microns: a STRICT test
    % would go red on floating cancellation in a perfectly correct run.
    rise = all(diff(alt(1:ipk)) >= 0);
    fall = all(diff(alt(ipk:end)) <= 0);
    % "returns to the surface" is scored as the zero crossing existing, NOT as
    % a fraction of the peak: the last RECORDED sample sits wherever samp
    % happens to leave it (37 nmi up at samp = 60 s), so a fractional test
    % would make this row a function of the caller's sampling interval.
    lands = isfinite(R.t_impact);
    B = addShape(B, 'ballistic arch: one interior apogee, unimodal, lands', ...
        'altitude rises to ONE interior maximum, falls monotonically, reaches 0', ...
        sprintf('peak %.1f nmi at sample %d of %d; monotone up %d, down %d; lands %d', ...
        pk, ipk, numel(alt), rise, fall, lands), ...
        interior && rise && fall && lands, ...
        'Fig. 28.9 caption + prose (zarchan.txt:10466, :10457)', ...
        'an ordering check: Fig. 28.9 itself did not survive the extraction');
else
    B = addSkip(B, 'ballistic arch: one interior apogee, unimodal, lands', ...
        'interior maximum', 'fewer than five recorded samples', ...
        'zarchan.txt:10466', '-');
end

% ---- INFO and SKIP ---------------------------------------------------------
if ~isempty(R.dist_km)
    lastKm = R.dist_km(end);  lastNmi = R.dist_nmi(end);
else
    lastKm = NaN;  lastNmi = NaN;
end
B = addInfo(B, 'distance3d divides by 3280 while the axis says "Nmi"', ...
    'a*beta/3280', sprintf('%.1f "Nmi" as printed = %.1f nmi at /6076', ...
    lastKm, lastNmi), 'Listing 28.2 (zarchan.txt:10421, :10423)', ...
    ['3280 is ft/km and 6076 is ft/nmi, and the SAME listing uses 6076 for ' ...
     'ALTNM.  Only the /6076 reading matches the prose''s "nearly 5000 n ' ...
     'miles", so the axis label and the divisor cannot both be right.  ' ...
     'Reported, not "fixed": R.dist_km is exactly what the listing computes']);
B = addSkip(B, 'Table 28.1 "Kepler" row', ...
    '2547.4148 / -3583.1074 / 6758.2384 km', ...
    ['KEPLER1 is Listing 28.3 (zarchan.txt:10480), a different program.  ' ...
     'Scoring it here would be scoring code this port does not contain'], ...
    'Table 28.1, p.736 (zarchan.txt:10470)', 'km');
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want: a struct array
%   that quietly reordered fields would make [B.ok] mean something else.
%   Schema is the one mgs.zarchan.benchSuite's shared benchRows() consumes.
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
function s = render(R, ev, bookCase)
L = {};  p = R.p;
L{end+1} = 'Zarchan Listing 28.2 -- BALLISTIC MISSILE TRAJECTORY GENERATOR (3-D Lambert + two-body ECI)';
L{end+1} = '*** UNITS ARE FEET AND ft/s.  nmi = 6076 ft, km = 3280 ft, deg = 57.3*rad. ***';
L{end+1} = sprintf('   Monte Carlo (%.4g N, %.4g E) -> Las Vegas (%.4g N, %.4g E),  TF = %g s', ...
    p.XLATTDEG, p.XLONGTDEG, p.XLATFDEG, p.XLONGFDEG, p.TF);
L{end+1} = sprintf('   H = %g s (%d steps), record every %g s, SWITCH1 = %g', ...
    p.H, p.nsteps, p.samp, p.SWITCH1);
L{end+1} = sprintf(['   LAMBERT3D: gamma = %.6f deg in [%.4f, %.4f], %d iterations, ' ...
    '|V0| = %.4f ft/s, tf residual %.3e s'], R.lam.gamma_deg, R.lam.gmin_deg, ...
    R.lam.gmax_deg, R.lam.iters, R.lam.v0mag, R.lam.tf_err);
L{end+1} = sprintf('   V0 = [%.9f  %.9f  %.9f] ft/s', R.v0(1), R.v0(2), R.v0(3));
L{end+1} = sprintf(['   r0 = [%.7f  %.7f  %.7f] ft   (the REAL*4-literal form ' ...
    'the book PRINTED is %.4f ft away -- BENCHMARK 4)'], R.r0(1), R.r0(2), ...
    R.r0(3), norm(R.r0 - R.r0_book_form));
L{end+1} = '';

L{end+1} = 'TRAJECTORY  (the listing''s own arrays, one row every samp seconds)';
L{end+1} = sprintf('%10s %12s %14s %13s %12s %10s', 't [s]', 'alt [nmi]', ...
    'downrng [km]', 'downrng[nmi]', 'long [deg]', 'lat [deg]');
if ~isempty(R.t)
    stride = max(1, round(numel(R.t)/12));
    rows = unique([1:stride:numel(R.t), numel(R.t)]);
    for r = rows
        L{end+1} = sprintf('%10.1f %12.2f %14.1f %13.1f %12.4f %10.4f', ...
            R.t(r), R.alt_nmi(r), R.dist_km(r), R.dist_nmi(r), ...
            R.long_deg(r), R.lat_deg(r)); %#ok<AGROW>
    end
end
L{end+1} = '';
L{end+1} = sprintf(['   apogee %.2f nmi at t = %.3f s;  altitude reaches zero at ' ...
    't = %.4f s;  run ends at %.3f s (-1 nmi)'], R.apogee_nmi, R.t_apogee, ...
    R.t_impact, R.t_end);
L{end+1} = sprintf('   impact %.5f N, %.5f E;  downrange %.1f nmi', ...
    R.impact_lat_deg, R.impact_long_deg, R.downrange_nmi);
T = R.tab281;
L{end+1} = sprintf(['   t = 1000 s state (Table 28.1): [%.4f %.4f %.4f] km, ' ...
    '[%.6f %.6f %.6f] km/s'], T.pos_km(1), T.pos_km(2), T.pos_km(3), ...
    T.vel_kms(1), T.vel_kms(2), T.vel_kms(3));
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-56s %s', vstr(K.oks(i)), K.names{i}, K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning';
    L{end+1} = '            NOT CHECKED, which is not the same thing as "fine".';
else
    L{end+1} = 'BENCHMARKS  (book values grepped from zarchan_decoded.txt; line numbers in .source)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
            case 'info'
                L{end+1} = sprintf('   INFO %-52s %s', b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case 'skip'
                L{end+1} = sprintf('   SKIP %-52s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case {'shape','bound'}
                L{end+1} = sprintf('   %-4s %-52s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf('   %-4s %-52s book %s %s / ours %s %s (%+.5f%%, band %.4g%%)', ...
                    vstr(b.ok), b.what, b.book_str, b.units, b.ours_str, ...
                    b.units, b.pct, 100*b.tol); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped/info -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
if ~bookCase
    L{end+1} = '';
    L{end+1} = '   NOTE: off Listing 28.2''s own case, so the book rows are SKIPPED, not passed.';
end
L{end+1} = '';
L{end+1} = 'REMINDER: the Earth-rotation lead (XLONGF = XLONGF - W*TF) is the silent';
L{end+1} = '   failure in this listing -- delete it and everything still looks right,';
L{end+1} = '   8.3 deg away from the target.  Structural check 4 is that delete experiment.';
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
%DRAWCURVES  The listing's own figure -- altitude against downrange -- plus the
%   ground track.  Base MATLAB graphics only.  ⚠️ the x axis carries BOTH
%   readings of distance3d, because the book's label and the book's divisor
%   disagree; see the INFO row.
figure('Name', 'Listing 28.2 -- ballistic target trajectory');
subplot(2,1,1);
plot(R.dist_nmi, R.alt_nmi, '-'); grid on;
xlabel('Downrange (Nmi)   [distance3d/3280 rescaled to /6076]');
ylabel('Altitude (Nmi)');
title(sprintf('Monte Carlo -> Las Vegas in %g s: apogee %.0f nmi, range %.0f nmi', ...
    R.p.TF, R.apogee_nmi, R.downrange_nmi));

subplot(2,1,2);
plot(R.long_deg, R.lat_deg, '-'); grid on; hold on;
plot(R.p.XLONGTDEG, R.p.XLATTDEG, 'o');
plot(R.p.XLONGFDEG, R.p.XLATFDEG, 'x');
xlabel('Longitude (deg, 57.3*rad)');
ylabel('Latitude (deg, 57.3*rad)');
legend('ground track', 'launch', 'aim point', 'Location', 'best');
hold off;
end
