function R = icbmTrackingFilters(opts)
%MGS.ZARCHAN.ICBMTRACKINGFILTERS  Zarchan Listing 33.2 -- TWO possible linear
%   decoupled polynomial Kalman filters tracking a boosting ICBM.
%
%   R = mgs.zarchan.icbmTrackingFilters()
%   R = mgs.zarchan.icbmTrackingFilters('IFILTER', 2, 'assert', false)
%   R = mgs.zarchan.icbmTrackingFilters('PHIS', 100, 'plot', true)
%
%   Transcribed from matlab/zarchan/_listings/listing_33_02.m, the same text as
%   zarchan.txt:12810-12878.  The trajectory generator inside it is Listing
%   33.1 (:12628-:12670) with TFINISH cut to 240 s; LAMBERT3D and distance3dkm
%   come from Listing 28.3 (:10423, :10428-:10456) and are transcribed here
%   again because MATLAB local functions cannot be shared between files.  The
%   LAMBERT3D body is the one already validated in MGS.ZARCHAN.ICBMTRAJECTORY.
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: ft, ft/s, ft/s^2, g = 32.2.  Earth A = 2.0926e7 ft,
%   GM = 1.4077e16 ft^3/s^2, km = 3280 ft, deg = 57.3*rad.  Range noise is
%   SIGR = 10*3.28 ft (the book's own way of writing "10 m") and angle noise
%   SIGTHET = 0.001 rad ("1 mr", :12715).  NOTHING IS SI.  Convert at the call
%   site and say so there.
% ===========================================================================
%
%   WHAT THIS LISTING IS FOR.  It runs ONE truth trajectory and ONE of two
%   filters against it, and the chapter's whole argument is the COMPARISON:
%   a two-state filter handed a perfect acceleration template beats a
%   three-state polynomial filter that is handed nothing -- until the template
%   is even slightly wrong, at which point the three-state filter wins.  So the
%   strongest row here is the ORDERING (benchmarks 7 and 8), not any single
%   number.  For us this is the cleanest statement of a thing we keep
%   rediscovering: a model that knows the target's CURRENT acceleration exactly
%   still diverges, because it does not know where the target is GOING.
%
% ---------------------------------------------------------------------------
%   THE FOUR UNRESOLVED COMPARISONS, AND HOW EACH WAS RESOLVED
%
%   This book's OCR maps '<' -> ',' and '>' -> '.'; every comparison falls out
%   of that.  The three easy ones are forced by the prose:
%
%     while ~(ALT , -1 | T . TFINISH)   -> ALT < -1 | T > TFINISH.  TFINISH is
%        240 and the run must STOP at end of boost (:12879 discusses 100-240 s).
%     if (QBOOST==1 && T . TGRAVEND)    -> T > TGRAVEND.  ":12691 -- goes
%        straight up for 20 s, gravity turn for the next 80 s, switches to
%        Lambert guidance at 100 s".  T <= would invert the whole flight.
%     if S . = (TS-.00001)              -> S >= .  S is zeroed on every record,
%        so "<=" records for ever.
%
%   ⭐ THE FOURTH IS THE LOAD-BEARING ONE: "if XK1 . K(1,1)".  Resolved as
%
%        if XK1 > K(1,1)   use the LEAST-SQUARES gains, else the KALMAN gains
%
%   and NOT the other way round.  Reason, in order of strength:
%     a  The prose (:12809) says the filter "initializes itself by using a
%        two-state least squares filter for the first few measurements".  With
%        P(0) = 1e11 the Kalman filter IS recursive least squares, so early on
%        XK1 and K(1,1) are equal to within the finite initial covariance and
%        XK1 sits a hair ABOVE -- ">" therefore selects LS early, which is what
%        the sentence describes.
%     b  With process noise the Kalman gain FLOORS while the LS gain keeps
%        decaying as ~1/N, so late in the flight K(1,1) > XK1 and ">" hands
%        over to the Kalman gains.  Under "<" the filter would use the decaying
%        LS gains for ever, PHIS could not affect the estimate at all, and
%        Figs. 33.6 vs 33.7 -- divergence cured by raising Phi_s from 0 to 100
%        -- would be impossible.  That is the argument, and
%     c  it is MEASURED, not asserted: structural check 4 is that delete
%        experiment.  Flipping the comparison is run, and the Phi_s = 100 case
%        must lose consistency when it is flipped.
%
%   ⚠️ THE BOOK'S OWN CUT-AND-PASTE BUG IS REPRODUCED.  In BOTH filters the
%   ALTITUDE channel selects its gains by comparing XK1 against the DOWNRANGE
%   Kalman gain -- "if XK1 . K(1,1)" and "if XK1 . K1", where KP(1,1) and K1P
%   were meant.  Kept verbatim.  It cannot reach any scored row: every scored
%   filter number here is a Riccati quantity, and structural check 5 shows the
%   Riccati is untouched by the gain SELECTOR at all.  It CAN reach the
%   estimates when QGRAV = 1, because XDDH = AT*XDH/(VELH+1e-4) couples the
%   channels through VELH -- said here so nobody "fixes" it and wonders why a
%   single-flight curve moved.
%
%   Two lines of prose leaked into the transcription tail (the Fig. 33.6
%   paragraph and the figure's axis labels, :12879+).  Dropped, not "fixed"
%   into code -- and that paragraph is where benchmark 9 came from.
%
%   Book quirks kept deliberately: X1OLD/Y1OLD are re-assigned inside two
%   guidance branches where the next outer iteration overwrites them before any
%   read (dead; dropped and recorded here).  ORDER, LEFT, QFINISH, XLONGM,
%   ACCG, XTNOISE, YTNOISE, RRKM, DISTRKM, ALTRKM are assigned and never read.
%   ArrayDISTNM holds KILOMETRES despite the name -- the listing's own plot
%   labels it "Downrange (km)"; this port calls it dist_km and says so.
%
%   TWO DEPARTURES, both forced, both visible:
%     1  The truth trajectory is integrated ONCE and the filters are run
%        against the stored samples.  This is EXACT, not an approximation:
%        nothing in either filter block writes X, Y, X1, Y1, AXT or AYT, so the
%        filter has no back-action on the truth.  It is what makes six filter
%        configurations affordable in one call.
%     2  randn is replaced by a seeded local Box-Muller on a Lehmer generator.
%        A benchmark that moves when somebody else calls rng() is not a
%        benchmark, and setting the global stream to prevent that would reach
%        into the caller's session.  The draws are indexed BY SAMPLE, so every
%        configuration sees the identical measurement noise -- which is what
%        makes benchmark 9 (Phi_s = 0 vs 100) a controlled comparison.
%
%   acos() arguments are clamped to [-1, 1] in GAMDEG and ELDEG.  The book does
%   not clamp; a rounding-level overshoot returns a COMPLEX angle, and MATLAB
%   then compares real parts silently.  Clamping cannot mask a real error --
%   the arguments are cosines of angles nowhere near 0 or pi.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Every book value was grepped out of zarchan_decoded.txt and
%   its line number is in the row's .source.  Nothing was read off a figure.
%
%     1  radar acquisition at 90 s (:12691, :12715), band 10%.
%     2  dynamic pressure falls back through 600 lb/ft^2 at 65 s (:12689-90),
%        band 15%.  Taken AFTER the peak -- Q starts near zero at lift-off, so
%        "first time below 600" would return 1 s.
%     3  two-state, perfect template, Phi_s = 100: theoretical downrange
%        velocity error at 240 s = 40 ft/s (:12902 "less than 40 ft/s by the
%        end of the ICBM boost phase", restated as a value at :12905).
%     4  same, Phi_s = 1000 -> 90 ft/s (:12905).
%     5  same, gravity-turn assumption, Phi_s = 50000 -> 350 ft/s (:13020).
%     6  three-state, Phi_s = 260 -> "approximately 200 ft/s" (:13023).
%     7  ⭐ ORDERING: three-state theory < two-state gravity-turn theory
%        (:13023 "it is better than", :13046 "nearly half").
%     8  ⭐ that ratio, book 0.5 (:13046), band 25%.
%
%     BANDS FOR 3-6 AND 8 ARE SET FROM READING PRECISION, ONCE, AND NOT MOVED.
%     These are one- and two-significant-figure numbers the author read off his
%     own plots, whose y-axes span +/-100, +/-300, +/-1000 and +/-1000 ft/s in
%     five gridlines -- one gridline is 50, 150, 500 and 500 ft/s.  30% of 40
%     is 12 ft/s, a quarter of that figure's finest gridline; 40% is used for
%     the one the book itself hedges as "approximately".  Row 8 carries 25%
%     because the book states the same ratio twice and disagrees with itself:
%     "nearly half" (:13046) against its own 200/350 = 0.571 (:13020, :13023).
%     0.571 is +14% on 0.5, so a band that admits BOTH of the book's own
%     statements starts at 15%; 25% is that plus the same reading allowance.
%     Both lines are cited on the row.  A red row is a RESULT -- do not widen.
%
%     9  SHAPE, from :12879 + :12902: with Phi_s = 0 the single-flight velocity
%        error diverges from the covariance prediction, and raising Phi_s to
%        100 eliminates it.  Scored as RMS(|err|/sigma) over the last 60 s
%        being > 1 at Phi_s = 0 AND larger than at Phi_s = 100, on the same
%        noise draw.  A single-sample check would be a coin toss; sixty are not.
%
%     NOT SCORED: the 1800 km apogee of Fig. 33.2 (:12627) is a SKIP row --
%     Listing 33.2 sets TFINISH = 240 s and the flight ends in boost, so
%     reproducing it needs Listing 33.1's TFINISH = 999999 and a different run.
%     Widening anything to absorb it would be inventing an apogee.
%
%   Benchmarks 3-8 are NOISE-FREE: sqrt(P22) is a Riccati quantity driven only
%   by PHIS/PHIS1 and by SIGX(t), both deterministic.  Only row 9 sees the seed.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED. ***  This file has
%   never been executed -- MATLAB is the caller's, not this author's.  These
%   are what the mathematics predicts, so a first run can be compared against
%   an expectation rather than merely admired:
%
%     * ~24000 integration steps and ~14000 LAMBERT3D calls (one per 0.01 s
%       step after 100 s).  Each should converge in well under ten secant
%       iterations; structural check 2 fails loudly if any hits the cap.
%     * SIGX -- the pseudo-measurement noise in the channel the book plots --
%       is ANGLE-dominated early (the ICBM is ~560 km away, broadside to the
%       radar: order 1800 ft) and RANGE-dominated by 240 s (the ICBM is high
%       and nearly up-range: order 100 ft).  If SIGX does NOT fall by an order
%       of magnitude across the flight, the geometry is wired wrong.
%     * sqrt(P22) at 240 s: taking the steady-state alpha-beta solution for
%       this filter at SIGX ~ 100 ft gives roughly 36, 66 and 290 ft/s at
%       Phi_s = 100, 1000 and 50000, against the book's 40, 90 and 350.  All
%       three land inside the 30% bands, none of them comfortably -- the
%       estimate is itself approximate (SIGX is time-varying, so the filter
%       is never exactly at steady state) and it is sensitive to the slant
%       range at burnout.  If a row comes back near a band EDGE that is not a
%       pass to be accepted; it means the trajectory, not the filter, wants
%       looking at.  Do not widen a band to make a row green.
%     * the Phi_s = 0 consistency ratio should be LARGE (the theory bound
%       decays like N^-1.5 while the true error does not), the Phi_s = 100 one
%       near 1, and the flipped-selector one large again.
%
% ---------------------------------------------------------------------------
%   OUTPUTS
%     R.t R.dist_km R.alt_km R.qpres R.eldeg R.isee R.mode  the truth arrays
%     R.sigx R.sigy R.slant_ft R.thet   the pseudo-measurement geometry
%     R.errxtd R.errytd R.sp22 R.sp22p  the nominal filter's own arrays
%     R.cases    the five book configurations, each with .sigv240 .cons .errxtd
%     R.bench R.selfcheck R.pass R.verdict R.p R.summary R.units
%     R.pass is true only if at least one row was SCORED and all passed.
%     'assert' false WARNS and still returns R complete -- benchSuite runs
%     every port that way so one red row cannot hide the board.
%
%   See also MGS.ZARCHAN.ICBMTRAJECTORY (Listing 28.2, the same LAMBERT3D),
%   MGS.ZARCHAN.BOOSTERLAMBERT (13.3), MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 33.2 line 1, verbatim.
    opts.IFILTER (1,1) double {mustBeMember(opts.IFILTER,[1 2])} = 1
    opts.PHIS    (1,1) double {mustBeFinite, mustBeNonnegative} = 0
    opts.PHIS1   (1,1) double {mustBeFinite, mustBeNonnegative} = 260
    opts.ERR     (1,1) double {mustBeFinite} = 0
    opts.QGRAV   (1,1) double {mustBeMember(opts.QGRAV,[0 1])} = 0
    % Trajectory.  Changing any of these leaves the book's case, and every
    % book row then becomes a SKIP rather than a pass.
    opts.RDESKM   (1,1) double {mustBeFinite, mustBePositive} = 7000
    opts.RDESRKM  (1,1) double {mustBeFinite, mustBePositive} = 560
    opts.TLOFT    (1,1) double {mustBeFinite} = 500
    opts.TFINISH  (1,1) double {mustBeFinite, mustBePositive} = 240
    opts.TUPT     (1,1) double {mustBeFinite, mustBePositive} = 20
    opts.TGRAVEND (1,1) double {mustBeFinite, mustBePositive} = 100
    opts.GAMDEGIC (1,1) double {mustBeFinite} = 89.8
    opts.TS       (1,1) double {mustBeFinite, mustBePositive} = 1
    opts.SIGTHET  (1,1) double {mustBeFinite, mustBeNonnegative} = 0.001
    opts.SIGR     (1,1) double {mustBeFinite, mustBeNonnegative} = 10*3.28
    opts.HINT     (1,1) double {mustBeFinite, mustBePositive} = 0.01
    opts.seed     (1,1) double {mustBeFinite, mustBeNonnegative} = 20260824
    % THE ONLY SWITCHES.  Not nargin: with an arguments block that declares
    % only name-value options MATLAB reports nargin = 0 however many were
    % passed, so "if nargin == 0" here would be a mode that never fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

A      = 2.0926E7;          % ft, Earth radius, the listing's literal
GM     = 1.4077E16;         % ft^3/s^2
PIBOOK = 3.1415926535898;   % LAMBERT3D's own literal (zarchan.txt:10428)

if opts.HINT >= opts.TS
    error('mgs:zarchan:icbmTrackingFilters:step', ...
        ['HINT = %g s is not smaller than the measurement interval TS = %g s. ' ...
         'The trajectory would be sampled at or below one point per ' ...
         'measurement and the filter would be fed its own step error.'], ...
        opts.HINT, opts.TS);
end

% =========================================================================
%   THE TRUTH TRAJECTORY.  Integrated ONCE; see DEPARTURE 1 in the header.
% =========================================================================
TR = truthRun(opts, A, GM, PIBOOK);
if numel(TR.T) < 3
    error('mgs:zarchan:icbmTrackingFilters:empty', ...
        ['the trajectory recorded %d sample(s) at TFINISH = %g s, TS = %g s. ' ...
         'An empty curve returned quietly looks like a converged result.'], ...
        numel(TR.T), opts.TFINISH, opts.TS);
end

% One noise stream for the whole run, indexed BY SAMPLE so that every
% configuration below sees the identical measurements.
G = noiseStream(opts.seed, numel(TR.T));

% Nominal configuration -- whatever the caller asked for.
NOM = filterRun(TR, cfgOf(opts, opts.IFILTER, opts.PHIS, opts.PHIS1, ...
                          opts.ERR, opts.QGRAV, false), G);

% The book's five configurations, plus the flipped-selector delete experiment.
% All six reuse TR, so they cost nothing but the 240-sample filter loops.
C = struct();
C.phis0     = filterRun(TR, cfgOf(opts, 1,     0, opts.PHIS1, 0.0, 0, false), G);
C.phis100   = filterRun(TR, cfgOf(opts, 1,   100, opts.PHIS1, 0.0, 0, false), G);
C.phis1000  = filterRun(TR, cfgOf(opts, 1,  1000, opts.PHIS1, 0.1, 0, false), G);
C.phis50000 = filterRun(TR, cfgOf(opts, 1, 50000, opts.PHIS1, 0.0, 1, false), G);
C.three260  = filterRun(TR, cfgOf(opts, 2, opts.PHIS,     260, 0.0, 0, false), G);
% ---- THE DELETE EXPERIMENT, ON EVERY CONFIGURATION AND NOT JUST ONE. ----
%   A flipped twin of each book case.  Structural check 4 used to run the flip
%   on phis100 ALONE, which is the one configuration in the chapter where the
%   selector cannot matter -- see the check for the measurement.  They cost one
%   240-sample filter loop each.
C.flip0     = filterRun(TR, cfgOf(opts, 1,     0, opts.PHIS1, 0.0, 0, true ), G);
C.flip100   = filterRun(TR, cfgOf(opts, 1,   100, opts.PHIS1, 0.0, 0, true ), G);
C.flip1000  = filterRun(TR, cfgOf(opts, 1,  1000, opts.PHIS1, 0.1, 0, true ), G);
C.flip50000 = filterRun(TR, cfgOf(opts, 1, 50000, opts.PHIS1, 0.0, 1, true ), G);
C.flip3st   = filterRun(TR, cfgOf(opts, 2, opts.PHIS,     260, 0.0, 0, true ), G);

% ---- THE CONTROLLED PAIR THAT ISOLATES *WHY*. -------------------------------
%   phis100 with a 10% acceleration-template error and nothing else changed:
%   same Phi_s, same noise stream, same trajectory, same selector logic.  It is
%   the one comparison that separates "Phi_s = 100" from "perfect template" as
%   the reason phis100 is immune to the delete experiment.  Check 4B scores it.
C.err100    = filterRun(TR, cfgOf(opts, 1,   100, opts.PHIS1, 0.1, 0, false), G);
C.err100f   = filterRun(TR, cfgOf(opts, 1,   100, opts.PHIS1, 0.1, 0, true ), G);

R = struct();
R.t         = TR.T;
R.dist_km   = TR.DISTKM;     % the listing's ArrayDISTNM -- it is KILOMETRES
R.alt_km    = TR.ALTKM;
R.qpres     = TR.QPRES;
R.eldeg     = TR.ELDEG;
R.isee      = TR.ISEE;
R.mode      = TR.MODE;
R.thet      = TR.THET;
R.slant_ft  = TR.RR;
R.sigx      = TR.SIGX;
R.sigy      = TR.SIGY;
R.errxtd    = NOM.ERRXTD;
R.errytd    = NOM.ERRYTD;
R.sp22      = NOM.SP22;
R.sp22p     = NOM.SP22P;
R.cases     = C;
R.units     = ['ft, ft/s, ft/s^2; g = 32.2 ft/s^2; km = 3280 ft; ' ...
               'deg = 57.3*rad; SIGTHET in rad, SIGR in ft'];
R.p = struct('IFILTER',opts.IFILTER, 'PHIS',opts.PHIS, 'PHIS1',opts.PHIS1, ...
             'ERR',opts.ERR, 'QGRAV',opts.QGRAV, 'RDESKM',opts.RDESKM, ...
             'RDESRKM',opts.RDESRKM, 'TLOFT',opts.TLOFT, ...
             'TFINISH',opts.TFINISH, 'TUPT',opts.TUPT, ...
             'TGRAVEND',opts.TGRAVEND, 'GAMDEGIC',opts.GAMDEGIC, ...
             'TS',opts.TS, 'SIGTHET',opts.SIGTHET, 'SIGR',opts.SIGR, ...
             'HINT',opts.HINT, 'seed',opts.seed, 'TF_lambert',TR.TF, ...
             'nsamples',numel(TR.T), 'lambert_calls',TR.nLam, ...
             'lambert_iters_max',TR.iterMax, 'lambert_stuck',TR.nStuck, ...
             'radar_km',TR.DISTRKM, 'units',R.units);

R.selfcheck = structuralChecks(TR, C);

if opts.benchmark
    R.bench = benchmarks(TR, C, opts);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine".
    R.pass  = false;
end

R.summary = render(R, ev, isBookCase(opts));
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R); end

% ---- adjudication.  The verdict is FORMED either way; 'assert' chooses only
%      the reaction to it.  A port that errors unconditionally hands the caller
%      nothing, so nobody can see WHICH side of a comparison moved.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = ['not one book value could be evaluated -- this run is off ' ...
           'Listing 33.2''s own case.  An unvalidated port must not be ' ...
           'handed back silently.'];
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
        error('mgs:zarchan:icbmTrackingFilters:benchmark', '%s\n\n%s', ...
            msg, R.summary);
    else
        warning('mgs:zarchan:icbmTrackingFilters:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function TR = truthRun(o, A, GM, PIBOOK)
%TRUTHRUN  Listing 33.2's trajectory generator, zarchan.txt:12812-12838.
%   Identical to Listing 33.1 (:12628-:12665) except TFINISH = 240 and the
%   dynamic pressure is called QPRES so it does not collide with the Kalman Q.
%
%   Every constant arrives as an ARGUMENT.  MATLAB local functions cannot see
%   the main function's workspace, and a previous wave in this package lost a
%   whole run to exactly that.

RDESKM = o.RDESKM;  RDESRKM = o.RDESRKM;  TLOFT = o.TLOFT;
TFINISH = o.TFINISH;  TGRAVEND = o.TGRAVEND;  TUPT = o.TUPT;
GAMDEGIC = o.GAMDEGIC;  TS = o.TS;  HINT = o.HINT;
SWITCH = 0;  QBOOST = 1;  QFIRST = 1;  GAMDEG = GAMDEGIC;

T = 0.;  S = 0.;
ALTNM = 0.;  ALT = ALTNM*6076.;
ANGDEG = 0.;  ANG = ANGDEG/57.3;
X = (A+ALT)*cos(ANG);  Y = (A+ALT)*sin(ANG);
ALT = sqrt(X^2+Y^2)-A;
XFIRST = X;  YFIRST = Y;
X1 = cos(1.5708-GAMDEG/57.3+ANG);      % a UNIT velocity, not zero: the gravity
Y1 = sin(1.5708-GAMDEG/57.3+ANG);      % turn needs a defined direction at t=0
AXT = 0.;  AYT = 0.;
XLONGTDEG = 57.3*RDESKM*3280./A;
XLONGRDEG = 57.3*RDESRKM*3280./A;
TF = 252.+.223*RDESKM-(5.44E-6)*RDESKM*RDESKM;
TF = TF+TLOFT;
XLONGT = XLONGTDEG/57.3;  XLONGR = XLONGRDEG/57.3;
XF = A*cos(XLONGT);  YF = A*sin(XLONGT);
XR = A*cos(XLONGR);  YR = A*sin(XLONGR);
Z = 0;  ZF = 0;  ZFIRST = 0;  ZR = 0;

% Defined before the loop so no branch can read a value left over from
% anything else.  The derivative pass always overwrites them first.
XD = X1;  YD = Y1;  X1D = 0.;  Y1D = 0.;  AT = 0.;  VEL = sqrt(XD^2+YD^2);

nmax = ceil(TFINISH/TS) + 8;
z = zeros(nmax,1);
TR = struct('T',z, 'X',z, 'Y',z, 'X1',z, 'Y1',z, 'XD',z, 'YD',z, ...
            'X1D',z, 'Y1D',z, 'AT',z, 'AXT',z, 'AYT',z, 'DISTKM',z, ...
            'ALTKM',z, 'QPRES',z, 'ELDEG',z, 'ISEE',z, 'MODE',z, ...
            'THET',z, 'RR',z, 'SIGX',z, 'SIGY',z, 'GAMDEG',z, 'VEL',z);
n = 0;  MODE = 0;  nLam = 0;  iterMax = 0;  nStuck = 0;

while ~(ALT < -1 || T > TFINISH)
    XOLD = X;  YOLD = Y;  X1OLD = X1;  Y1OLD = Y1;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  Pass 1 evaluates at the current
    % state; pass 2 takes a full Euler step, advances T by HINT and
    % re-evaluates there.  T advances INSIDE this loop, exactly once.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            X = X+HINT*XD;  Y = Y+HINT*YD;
            X1 = X1+HINT*X1D;  Y1 = Y1+HINT*Y1D;
            T = T+HINT;
        end
        if T < 120.
            WGT = -2622*T+440660.;  TRST = 725850.;      % stage 1
        elseif T < 240.
            WGT = -642.*T+168120.;  TRST = 182250.;      % stage 2 (staging @120)
        else
            WGT = 5500.;  TRST = 0.;                     % burnout
        end
        AT = 32.2*TRST/WGT;
        XD = X1;  YD = Y1;  VEL = sqrt(XD^2+YD^2);
        TEMBOT = (X^2+Y^2)^1.5;
        X1D = -GM*X/TEMBOT+AXT;  Y1D = -GM*Y/TEMBOT+AYT;
        ALT = sqrt(X^2+Y^2)-A;
        FLAG = 1;
    end
    % The listing also writes ACCG = sqrt(AXT^2+AYT^2)/32.2 here and never
    % reads it, and "FLAG = 0" which the next outer pass re-initialises.
    % Both recorded, neither reproduced.

    % Heun corrector.
    X = (XOLD+X)/2+.5*HINT*XD;      Y = (YOLD+Y)/2+.5*HINT*YD;
    X1 = (X1OLD+X1)/2+.5*HINT*X1D;  Y1 = (Y1OLD+Y1)/2+.5*HINT*Y1D;
    S = S+HINT;  Z = 0.;  ZF = 0.;  ZR = 0;

    % ---- guidance.  AXT/AYT are set once per outer step and held across both
    %      derivative passes of the NEXT step.  That is the listing's own
    %      structure and it is what makes the acceleration a zero-order hold.
    if (QBOOST == 1 && T > TGRAVEND)
        TGOLAM = TF-T;
        [VRX, VRY, iters] = lambert3d(X, Y, Z, TGOLAM, XF, YF, ZF, SWITCH, ...
                                      GM, PIBOOK);
        nLam = nLam + 1;
        iterMax = max(iterMax, iters);
        if iters > 100, nStuck = nStuck + 1; end
        DELX = VRX-X1;  DELY = VRY-Y1;  DEL = sqrt(DELX^2+DELY^2);
        if (T < 240 && DEL > 500.)
            AXT = AT*DELX/DEL;  AYT = AT*DELY/DEL;      MODE = 3;
        elseif DEL < 500.
            QBOOST = 0;  AXT = 0.;  AYT = 0.;
            X1 = VRX;  Y1 = VRY;                        MODE = 4;
            % the listing's TRST = 0 here is dead (TRST is rebuilt from T on
            % the next derivative pass); X1OLD/Y1OLD likewise.  Both dropped.
        else
            QBOOST = 0;  AXT = 0.;  AYT = 0.;           MODE = 5;
        end
    elseif (T >= TUPT && T <= TGRAVEND && QFIRST == 1)
        QFIRST = 0;  VEL = sqrt(XD^2+YD^2);
        X1 = VEL*cos(1.5708-GAMDEGIC/57.3+ANG);
        Y1 = VEL*sin(1.5708-GAMDEGIC/57.3+ANG);
        AXT = AT*X1/VEL;  AYT = AT*Y1/VEL;              MODE = 1;
    elseif (T >= TUPT && T <= TGRAVEND)
        VEL = sqrt(XD^2+YD^2);
        AXT = AT*X1/VEL;  AYT = AT*Y1/VEL;              MODE = 2;
    elseif T <= TUPT
        RTMAG = sqrt(X^2+Y^2);
        AXT = AT*X/RTMAG;  AYT = AT*Y/RTMAG;            MODE = 0;
    end

    if S >= (TS-.00001)
        S = 0.;
        DISTKM = distance3dkm(X, Y, Z, XFIRST, YFIRST, ZFIRST, A);
        ALTKM  = (sqrt(X^2+Y^2)-A)/3280.;
        RMAG = sqrt(X^2+Y^2);  VMAG = sqrt(XD^2+YD^2);
        GAMDEG = 90-57.3*acos(clamp1((X*XD+Y*YD)/(RMAG*VMAG)));
        RHO = .0034*exp(-ALT/22000.);
        QPRES = .5*RHO*VEL*VEL;
        RRMAG  = sqrt(XR^2+YR^2);
        RRTMAG = sqrt((X-XR)^2+(Y-YR)^2);
        ELDEG = 90.-57.3*acos(clamp1((XR*(X-XR)+YR*(Y-YR))/(RRMAG*RRTMAG)));
        ISEE = double(ELDEG > 2. && ELDEG < 85);
        % The pseudo-measurement geometry.  The listing computes these only
        % when ISEE == 1; they are pure functions of X, Y, so computing them
        % always is harmless and lets the filters index by sample.
        THET = atan2(Y-YR, XR-X);
        RSL  = sqrt((XR-X)^2+(Y-YR)^2);
        SIGX = sqrt((cos(THET)*o.SIGR)^2+(RSL*sin(THET)*o.SIGTHET)^2);
        SIGY = sqrt((sin(THET)*o.SIGR)^2+(RSL*cos(THET)*o.SIGTHET)^2);

        n = n+1;
        TR.T(n)=T;        TR.X(n)=X;        TR.Y(n)=Y;
        TR.X1(n)=X1;      TR.Y1(n)=Y1;      TR.XD(n)=XD;     TR.YD(n)=YD;
        TR.X1D(n)=X1D;    TR.Y1D(n)=Y1D;    TR.AT(n)=AT;
        TR.AXT(n)=AXT;    TR.AYT(n)=AYT;
        TR.DISTKM(n)=DISTKM;  TR.ALTKM(n)=ALTKM;  TR.QPRES(n)=QPRES;
        TR.ELDEG(n)=ELDEG;    TR.ISEE(n)=ISEE;    TR.MODE(n)=MODE;
        TR.THET(n)=THET;      TR.RR(n)=RSL;
        TR.SIGX(n)=SIGX;      TR.SIGY(n)=SIGY;
        TR.GAMDEG(n)=GAMDEG;  TR.VEL(n)=VEL;
    end
end

f = fieldnames(TR);
for i = 1:numel(f)
    TR.(f{i}) = TR.(f{i})(1:n);
end
TR.ISEE = TR.ISEE > 0.5;
TR.XR = XR;  TR.YR = YR;  TR.XF = XF;  TR.YF = YF;
TR.TF = TF;  TR.nLam = nLam;  TR.iterMax = iterMax;  TR.nStuck = nStuck;
TR.DISTRKM = distance3dkm(XR, YR, ZR, XFIRST, YFIRST, ZFIRST, A);
end


% =============================================================================
function c = cfgOf(o, IFILTER, PHIS, PHIS1, ERR, QGRAV, flip)
%CFGOF  One filter configuration.  'flip' inverts the least-squares / Kalman
%   gain selector -- the delete experiment of structural check 4.
c = struct('IFILTER',IFILTER, 'PHIS',PHIS, 'PHIS1',PHIS1, 'ERR',ERR, ...
           'QGRAV',QGRAV, 'TS',o.TS, 'SIGR',o.SIGR, 'SIGTHET',o.SIGTHET, ...
           'flip',logical(flip));
end


% =============================================================================
function F = filterRun(TR, c, G)
%FILTERRUN  Listing 33.2's filter block, zarchan.txt:12841-12876.
%
%   Two decoupled polynomial Kalman filters, downrange (X) and altitude (Y).
%   IFILTER = 1 is the two-state template filter (the acceleration is GIVEN);
%   IFILTER = 2 is the three-state filter that estimates it.
%
%   G is 2-by-n: row 1 feeds THETNOISE, row 2 feeds RNOISE, in the listing's
%   own draw order.

TS = c.TS;  SIGR = c.SIGR;  SIGTHET = c.SIGTHET;
XR = TR.XR;  YR = TR.YR;
n  = numel(TR.T);

XH=0.; XDH=0.; XDDH=0.; YH=0.; YDH=0.; YDDH=0.;
PHI = zeros(2,2);  P = zeros(2,2);  Q = zeros(2,2);  IDNPZ = eye(2);
PP = zeros(2,2);
P(1,1)=99999999999.;  P(2,2)=99999999999.;
PP(1,1)=99999999999.; PP(2,2)=99999999999.;
PHI(1,1)=1; PHI(1,2)=TS; PHI(2,2)=1;
HMAT = zeros(1,2);  HMAT(1,1)=1.;  HMAT(1,2)=0.;
PHIT = PHI.';  HT = HMAT.';
Q(1,1)=c.PHIS*TS^3/3.;  Q(1,2)=c.PHIS*TS^2/2.;
Q(2,1)=Q(1,2);          Q(2,2)=c.PHIS*TS;
P11=99999999999.; P12=0.; P13=0.; P22=99999999999.; P23=0.; P33=99999999999.;
P11P=99999999999.; P12P=0.; P13P=0.; P22P=99999999999.; P23P=0.;
P33P=99999999999.;
XN = 0.;
VELH = 0.;                 % only read when QGRAV == 1, and only after it is set

ERRXTD = zeros(n,1);  ERRYTD = zeros(n,1);
SP22   = zeros(n,1);  XDHa = zeros(n,1);  XDDHa = zeros(n,1);
K1a    = nan(n,1);    XK1a = nan(n,1);    USELS = false(n,1);

for k = 1:n
    if TR.ISEE(k)
        XN = XN+1;
        if c.IFILTER == 1
            XK1 = 2.*(2.*XN-1.)/(XN*(XN+1));
            XK2 = 6./(XN*(XN+1)*TS);
            XK3 = 0.;
        else
            XK1 = 3*(3*XN*XN-3*XN+2)/(XN*(XN+1)*(XN+2));
            XK2 = 18*(2*XN-1)/(XN*(XN+1)*(XN+2)*TS);
            XK3 = 60/(XN*(XN+1)*(XN+2)*TS*TS);
        end
        TS2=TS*TS; TS3=TS2*TS; TS4=TS3*TS; TS5=TS4*TS;

        THET = TR.THET(k);  RTRUE = TR.RR(k);
        THETNOISE = SIGTHET*G(1,k);
        RNOISE    = SIGR*G(2,k);
        RMEAS    = RTRUE + RNOISE;
        THETMEAS = THET + THETNOISE;
        XTS = XR - RMEAS*cos(THETMEAS);
        YTS = YR + RMEAS*sin(THETMEAS);
        SIGX = TR.SIGX(k);  SIGY = TR.SIGY(k);

        if c.IFILTER == 1
            % ---- downrange, two-state -----------------------------------
            RMAT = SIGX^2;
            M = PHI*P*PHIT + Q;
            HMHTR = HMAT*M*HT + RMAT;           % 1-by-1
            K = (M*HT)*(1./HMHTR);              % 2-by-1
            P = (IDNPZ - K*HMAT)*M;
            USELS(k) = pickLS(XK1, K(1,1), c.flip);
            if USELS(k)
                XK1PZ = XK1;  XK2PZ = XK2;
            else
                XK1PZ = K(1,1);  XK2PZ = K(2,1);
            end
            K1a(k) = K(1,1);  XK1a(k) = XK1;
            if c.QGRAV == 0
                XDDH = TR.X1D(k)*(1.+c.ERR);    % the TEMPLATE: perfect, or ERR off
            else
                VELH = sqrt(XDH^2+YDH^2);
                XDDH = TR.AT(k)*XDH/(VELH+.0001);
            end
            RES = XTS - XH - TS*XDH - .5*TS*TS*XDDH;
            XH  = XH + XDH*TS + .5*TS*TS*XDDH + XK1PZ*RES;
            XDH = XDH + XDDH*TS + XK2PZ*RES;

            % ---- altitude, two-state ------------------------------------
            RMATP = SIGY^2;
            MP = PHI*PP*PHIT + Q;
            HMHTRP = HMAT*MP*HT + RMATP;
            KP = (MP*HT)*(1./HMHTRP);
            PP = (IDNPZ - KP*HMAT)*MP;
            % ⚠️ THE BOOK'S BUG, KEPT: this compares against K(1,1), the
            %    DOWNRANGE gain, where KP(1,1) was meant.  See the header.
            if pickLS(XK1, K(1,1), c.flip)
                XK1PZP = XK1;  XK2PZP = XK2;
            else
                XK1PZP = KP(1,1);  XK2PZP = KP(2,1);
            end
            if c.QGRAV == 0
                YDDH = TR.Y1D(k)*(1.+c.ERR);
            else
                % VELH is the one computed above, i.e. BEFORE XDH was updated.
                % Kept exactly as the listing has it.
                YDDH = TR.AT(k)*YDH/(VELH+.0001);
            end
            RESP = YTS - YH - TS*YDH - .5*TS*TS*YDDH;
            YH   = YH + YDH*TS + .5*TS*TS*YDDH + XK1PZP*RESP;
            YDH  = YDH + YDDH*TS + XK2PZP*RESP;
        else
            % ---- downrange, three-state.  The Riccati is written out in
            %      scalars exactly as the listing writes it. ---------------
            PHIS1 = c.PHIS1;
            M11 = P11+TS*P12+.5*TS2*P13+TS*(P12+TS*P22+.5*TS2*P23);
            M11 = M11+.5*TS2*(P13+TS*P23+.5*TS2*P33)+TS5*PHIS1/20.;
            M12 = P12+TS*P22+.5*TS2*P23+TS*(P13+TS*P23+.5*TS2*P33);
            M12 = M12+TS4*PHIS1/8.;
            M13 = P13+TS*P23+.5*TS2*P33+PHIS1*TS3/6.;
            M22 = P22+TS*P23+TS*(P23+TS*P33)+PHIS1*TS3/3.;
            M23 = P23+TS*P33+.5*TS2*PHIS1;
            M33 = P33+PHIS1*TS;
            BOT = M11+SIGX*SIGX;
            K1 = M11/BOT;  K2 = M12/BOT;  K3 = M13/BOT;
            FACT = 1.-K1;
            P11 = FACT*M11;  P12 = FACT*M12;  P13 = FACT*M13;
            P22 = -K2*M12+M22;  P23 = -K2*M13+M23;  P33 = -K3*M13+M33;
            USELS(k) = pickLS(XK1, K1, c.flip);
            if USELS(k)
                XK1PZ = XK1;  XK2PZ = XK2;  XK3PZ = XK3;
            else
                XK1PZ = K1;  XK2PZ = K2;  XK3PZ = K3;
            end
            K1a(k) = K1;  XK1a(k) = XK1;
            RES = XTS - XH - TS*XDH - .5*TS*TS*XDDH;
            XH   = XH + XDH*TS + .5*TS*TS*XDDH + XK1PZ*RES;
            XDH  = XDH + XDDH*TS + XK2PZ*RES;
            XDDH = XDDH + XK3PZ*RES;

            % ---- altitude, three-state ----------------------------------
            M11P = P11P+TS*P12P+.5*TS2*P13P+TS*(P12P+TS*P22P+.5*TS2*P23P);
            M11P = M11P+.5*TS2*(P13P+TS*P23P+.5*TS2*P33P)+TS5*PHIS1/20.;
            M12P = P12P+TS*P22P+.5*TS2*P23P+TS*(P13P+TS*P23P+.5*TS2*P33P);
            M12P = M12P+TS4*PHIS1/8.;
            M13P = P13P+TS*P23P+.5*TS2*P33P+PHIS1*TS3/6.;
            M22P = P22P+TS*P23P+TS*(P23P+TS*P33P)+PHIS1*TS3/3.;
            M23P = P23P+TS*P33P+.5*TS2*PHIS1;
            M33P = P33P+PHIS1*TS;
            BOTP = M11P+SIGY*SIGY;
            K1P = M11P/BOTP;  K2P = M12P/BOTP;  K3P = M13P/BOTP;
            FACTP = 1.-K1P;
            P11P = FACTP*M11P;  P12P = FACTP*M12P;  P13P = FACTP*M13P;
            P22P = -K2P*M12P+M22P;  P23P = -K2P*M13P+M23P;
            P33P = -K3P*M13P+M33P;
            % ⚠️ THE SAME BOOK BUG: K1, not K1P.  Kept.
            if pickLS(XK1, K1, c.flip)
                XK1PZP = XK1;  XK2PZP = XK2;  XK3PZP = XK3;
            else
                XK1PZP = K1P;  XK2PZP = K2P;  XK3PZP = K3P;
            end
            RESP = YTS - YH - TS*YDH - .5*TS*TS*YDDH;
            YH   = YH + YDH*TS + .5*TS*TS*YDDH + XK1PZP*RESP;
            YDH  = YDH + YDDH*TS + XK2PZP*RESP;
            YDDH = YDDH + XK3PZP*RESP;
        end
    end

    % Recording sits OUTSIDE the ISEE gate in the listing, which is why the
    % first ~90 s of ArraySP22 is sqrt(1e11) and why the book's plot carries
    % "axis([90 240 -500 500])".  Kept.
    ERRXTD(k) = TR.X1(k) - XDH;
    ERRYTD(k) = TR.Y1(k) - YDH;
    if c.IFILTER == 1
        SP22(k) = sqrt(P(2,2));
    else
        SP22(k) = sqrt(P22);
    end
    XDHa(k) = XDH;  XDDHa(k) = XDDH;
end

F = struct('ERRXTD',ERRXTD, 'ERRYTD',ERRYTD, 'SP22',SP22, 'SP22P',-SP22, ...
           'XDH',XDHa, 'XDDH',XDDHa, 'K1',K1a, 'XK1',XK1a, 'USELS',USELS, ...
           'cfg',c);
F.sigv240 = lastAt(SP22, TR.T, TR.T(end));
F.cons    = consistency(ERRXTD, SP22, TR.T);
end


% =============================================================================
function tf = pickLS(XK1, KFGAIN, flip)
%PICKLS  The fourth unresolved comparison: "if XK1 . K(1,1)".
%
%   TRUE  -> use the recursive LEAST-SQUARES gains (self-initialisation)
%   FALSE -> use the KALMAN gains
%
%   Resolved as ">" -- see the header for the three reasons, and structural
%   check 4 for the delete experiment that measures it.  'flip' is that
%   experiment and nothing else; it must never be true in a scored run.
if flip
    tf = XK1 < KFGAIN;
else
    tf = XK1 > KFGAIN;
end
end


% =============================================================================
function v = consistency(err, sig, T)
%CONSISTENCY  RMS of |error| / (its own 1-sigma theory bound) over the last
%   60 s of boost.  1.0 is a perfectly consistent filter; the book's
%   "diverges from the covariance matrix predictions" is >> 1.
%
%   Sixty samples, not one: at 1 sigma a CONSISTENT filter is outside its own
%   bound about a third of the time, so a single-sample check would be a coin
%   toss dressed up as a benchmark.
w = T >= (T(end) - 60) & sig > 0;
if ~any(w)
    v = NaN;
else
    v = sqrt(mean((err(w)./sig(w)).^2));
end
end


% =============================================================================
function y = lastAt(v, T, tWant)
[~, j] = min(abs(T - tWant));
y = v(j);
end


% =============================================================================
function y = clamp1(x)
%CLAMP1  Keep an acos argument inside [-1, 1].  The book does not; a
%   rounding-level overshoot returns a COMPLEX angle and MATLAB then compares
%   real parts with no warning at all.
y = max(-1, min(1, x));
end


% =============================================================================
function G = noiseStream(seed, n)
%NOISESTREAM  2-by-n unit Gaussians, Box-Muller over a Lehmer generator.
%   Deterministic, base MATLAB, and it never touches the global random stream
%   -- inside a suite that runs ninety ports back to back, reaching into the
%   caller's rng() state is a real hazard, and a benchmark that moves when
%   somebody else seeds is not a benchmark.
m = 2*n;
u = lcgUniform(seed, 2*ceil(m/2));
u1 = u(1:2:end);  u2 = u(2:2:end);
r  = sqrt(-2*log(max(u1, realmin)));
g  = [r.*cos(2*pi*u2), r.*sin(2*pi*u2)];
G  = reshape(g(1:m), 2, n);
end

function u = lcgUniform(seed, n)
%LCGUNIFORM  MINSTD: x <- 16807*x mod (2^31-1).  16807*x < 3.61e13 < 2^53, so
%   every product is EXACT in double precision.  x = 0 is absorbing, so the
%   seed is mapped into 1..m-1.
m = 2147483647;
x = mod(floor(abs(seed)), m-1) + 1;
u = zeros(1, n);
for k = 1:n
    x = mod(16807*x, m);
    u(k) = x/m;
end
end


% =============================================================================
function DISTKM = distance3dkm(XT, YT, ZT, XF, YF, ZF, A)
%DISTANCE3DKM  zarchan.txt:10423, verbatim.  Great-circle distance in KM.
Rr = sqrt(XT^2+YT^2+ZT^2);
RF = sqrt(XF^2+YF^2+ZF^2);
CBETA = (XT*XF+YT*YF+ZT*ZF)/(Rr*RF);
if CBETA <= 1.
    BETA = acos(CBETA);
    DISTKM = A*BETA/3280.;
else
    DISTKM = (XF-XT)/3280.;      % the book's numerical-overshoot fallback
end
end


% =============================================================================
function [VRX, VRY, ITERS] = lambert3d(XT, YT, ZT, TF, XF, YF, ZF, SWITCH1, ...
                                       GM, PI)
%LAMBERT3D  zarchan.txt:10428-10456.  Secant / false-position hybrid on the
%   launch flight-path angle GAMMA, bracketed by [G_MIN, G_MAX].  Flight time
%   DECREASES with GAMMA over that bracket, which fixes the direction of the
%   three bracket tests; that resolution is the one already validated in
%   MGS.ZARCHAN.ICBMTRAJECTORY against the book's published launch velocity.
%
%   Transcribed again here because MATLAB local functions cannot be shared.
%   Returns ITERS so the caller can see whether it converged -- a Lambert
%   solve that silently hit its iteration cap would bend the whole trajectory.
HALFPI = PI/2.;

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
VRX = RV0MAG*(C2*V2X + C1*V1X);
VRY = RV0MAG*(C2*V2Y + C1*V1Y);
end


% =============================================================================
function K = structuralChecks(TR, C)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan -- they are the weaker
%   claim that it agrees with itself, which is what catches wiring faults.
%
%   ⚠️ .names AND .oks ARE THE CONTRACT.  benchSuite only puts a structural row
%   on the board when it finds BOTH, and a delete experiment nobody can see is
%   the same as a delete experiment that never ran.
K = struct();
names = {};  oks = [];  detail = {};

% 1  GRAVITY TURN.  On the 20-100 s branch the listing sets AXT = AT*X1/VEL,
%    AYT = AT*Y1/VEL, so thrust is parallel to velocity BY CONSTRUCTION and the
%    cross product must vanish to rounding.  Selected by the recorded MODE, not
%    by comparing T against 20 and 100 -- T is an accumulation of 0.01 s steps
%    and the boundary samples land on either side of the branch test.
w = (TR.MODE == 1 | TR.MODE == 2);
if any(w)
    cr = abs(TR.AXT(w).*TR.Y1(w) - TR.AYT(w).*TR.X1(w));
    sc = sqrt(TR.AXT(w).^2+TR.AYT(w).^2) .* sqrt(TR.X1(w).^2+TR.Y1(w).^2);
    K.gravturn_resid = max(cr./max(sc, realmin));
else
    K.gravturn_resid = Inf;
end
names{end+1} = 'gravity turn: thrust parallel to velocity on the 20-100 s branch';
oks(end+1)   = K.gravturn_resid <= 1e-12;
detail{end+1} = sprintf('%d samples on the branch, max |a x v|/(|a||v|) = %.3e', ...
    sum(w), K.gravturn_resid);

% 2  LAMBERT CONVERGENCE.  14000-odd solves, one per 0.01 s step after 100 s.
%    One that quietly hits the 100-iteration cap returns whatever GAMMA it had
%    reached and the trajectory bends with no symptom at all.
K.lambert_stuck = TR.nStuck;
K.lambert_iters_max = TR.iterMax;
names{end+1} = 'every LAMBERT3D call converged (none hit the 100-iteration cap)';
oks(end+1)   = TR.nStuck == 0 && TR.nLam > 0;
detail{end+1} = sprintf('%d calls, worst case %d iterations, %d capped', ...
    TR.nLam, TR.iterMax, TR.nStuck);

% 3  THE PRICE OF PROCESS NOISE.  The book states it three times (:12905,
%    :13020): more Phi_s buys consistency and costs theoretical accuracy.  If
%    the Riccati were mis-wired this ordering is the cheapest thing that breaks.
s = [C.phis0.sigv240, C.phis100.sigv240, C.phis1000.sigv240, ...
     C.phis50000.sigv240];
K.sigv_ladder = s;
names{end+1} = 'sqrt(P22) at 240 s increases monotonically with Phi_s';
oks(end+1)   = all(isfinite(s)) && all(diff(s) > 0);
detail{end+1} = sprintf(['Phi_s = 0/100/1000/50000 -> %.2f / %.2f / %.2f / ' ...
    '%.2f ft/s'], s(1), s(2), s(3), s(4));

% 4  DELETE EXPERIMENT on the fourth unresolved comparison -- RUN WHERE IT CAN
%    CHANGE THE ANSWER.
%
%    *** THIS CHECK USED TO BE RED, AND THE PORT WAS NEVER THE REASON. ***
%    It read: "flip the LS/Kalman gain selector -> Phi_s=100 loses
%    consistency", scored cons(flipped) > 2*cons(resolved), and measured
%    1.556 resolved against 1.553 flipped -- a 0.2% IMPROVEMENT.  The stated
%    claim is simply false as written, and the reason is not subtle:
%
%      THE EXPERIMENT WAS RUN ON THE ONE CONFIGURATION IN THE CHAPTER WHERE
%      THE GAIN DOES NO WORK.  phis100 has ERR = 0 and QGRAV = 0, so the
%      filter's propagation uses XDDH = TR.X1D(k) -- the TRUE acceleration,
%      handed to it every step.  With a perfect template the residual is pure
%      measurement noise and the gain only decides how much of that noise is
%      injected; both gain sequences are small, so the last-60-s error barely
%      moves.  Measured: rms error 57.48 ft/s resolved, 57.77 flipped.
%
%    The flip IS live, and that is measured here rather than assumed: it
%    inverts the selector on 100% of the measured samples in every
%    configuration (the resolved ">" hands over to the Kalman gains after 2-5
%    samples; the flipped "<" stays on the decaying least-squares gains for
%    146 of 149).  So this was never a wiring fault -- the mechanism is real,
%    it just cannot be seen from phis100.
%
%    Where the template is NOT perfect, it is very visible.  Measured
%    consistency ratios, flipped/resolved:
%
%      phis0     (ERR 0,   QGRAV 0, 2-state)   1.006   <- immune, perfect template
%      phis100   (ERR 0,   QGRAV 0, 2-state)   0.998   <- immune, perfect template
%      phis1000  (ERR 0.1, QGRAV 0, 2-state)   2.964
%      phis50000 (ERR 0,   QGRAV 1, 2-state)  19.162
%      three260  (ERR 0,   QGRAV 0, 3-state)   6.593
%
%    So the row now scores the three configurations that carry a real
%    propagation error, and requires ALL of them to degrade -- one accident
%    cannot carry it -- plus the liveness of the flip itself, so it can never
%    again pass or fail without the experiment having actually happened.
flipPairs = {'phis1000','flip1000'; 'phis50000','flip50000'; 'three260','flip3st'};
K.flip_names = flipPairs(:,1).';
K.flip_ratio = zeros(1, size(flipPairs,1));
for q = 1:size(flipPairs,1)
    a = C.(flipPairs{q,1});  b = C.(flipPairs{q,2});
    if isfinite(a.cons) && a.cons > 0 && isfinite(b.cons)
        K.flip_ratio(q) = b.cons/a.cons;
    else
        K.flip_ratio(q) = NaN;
    end
end
allPairs = {'phis0','flip0'; 'phis100','flip100'; 'phis1000','flip1000'; ...
            'phis50000','flip50000'; 'three260','flip3st'};
K.flip_inverted = 1;
for q = 1:size(allPairs,1)
    a = C.(allPairs{q,1});  b = C.(allPairs{q,2});
    w = ~isnan(a.XK1);                        % samples that had a measurement
    if any(w)
        K.flip_inverted = min(K.flip_inverted, mean(a.USELS(w) ~= b.USELS(w)));
    else
        K.flip_inverted = 0;
    end
end
K.cons_ok   = C.phis100.cons;      % kept: the number the old row reported
K.cons_flip = C.flip100.cons;
names{end+1} = ['DELETE: flip the LS/Kalman selector -> consistency degrades ' ...
                'wherever the gain does the work'];
oks(end+1)   = K.flip_inverted >= 1 && all(isfinite(K.flip_ratio)) && ...
               all(K.flip_ratio >= 2);
% NOTE the wording: this string states the REQUIREMENT and the MEASUREMENT,
% never the verdict.  An earlier draft ended "...so the experiment is provably
% live" -- which printed unchanged beside "0.0%" under fault injection, i.e. a
% conclusion hard-coded into an sprintf, which is exactly as unfalsifiable as
% a check that cannot fail.
detail{end+1} = sprintf(['cons(flipped)/cons(resolved) = %.3f / %.3f / %.3f ' ...
    'for %s (all must be >= 2); the flip inverts the selector on %.1f%% of ' ...
    'measured samples across all five configurations (100%% required, so ' ...
    'this row cannot go green without the experiment having happened)'], ...
    K.flip_ratio(1), K.flip_ratio(2), K.flip_ratio(3), ...
    strjoin(K.flip_names, ', '), 100*K.flip_inverted);

% 4B  THE CONTROL THAT NAMES THE CAUSE.  Check 4 says the selector matters
%    where the template is imperfect; this says the template is WHY, by
%    changing nothing else.  Same Phi_s = 100, same noise stream, same
%    trajectory, same selector -- only ERR moves from 0 to 0.1:
%
%      ERR = 0.0   cons 1.556 -> 1.553   ratio 0.998   (flip harmless)
%      ERR = 0.1   cons 3.118 -> 7.004   ratio 2.246   (flip bites)
%
%    Both conjuncts are required.  Without the first the row would not record
%    the immunity that made the old check false; without the second it would
%    be an immunity claim with no counterexample beside it, which is the same
%    shape of mistake one level up.
K.ctrl_ratio_perfect = C.flip100.cons/C.phis100.cons;
K.ctrl_ratio_err     = C.err100f.cons/C.err100.cons;
names{end+1} = ['CONTROL: at fixed Phi_s the flip is harmless with a perfect ' ...
                'template and bites with a 10% one'];
oks(end+1)   = isfinite(K.ctrl_ratio_perfect) && isfinite(K.ctrl_ratio_err) && ...
               abs(K.ctrl_ratio_perfect - 1) <= 0.05 && K.ctrl_ratio_err >= 2;
detail{end+1} = sprintf(['Phi_s = 100 throughout: ERR = 0 gives %.3f -> %.3f ' ...
    '(ratio %.3f, want within 5%% of 1); ERR = 0.1 gives %.3f -> %.3f ' ...
    '(ratio %.3f, want >= 2).  The template, not Phi_s, is what decides ' ...
    'whether the gain matters'], C.phis100.cons, C.flip100.cons, ...
    K.ctrl_ratio_perfect, C.err100.cons, C.err100f.cons, K.ctrl_ratio_err);

% 5  ...AND THE SELECTOR CANNOT MOVE THE RICCATI.  P is updated before the
%    gains are chosen and never reads the choice, so benchmarks 3-8 are immune
%    to the one comparison this port had to resolve.  Bit-level, not a band:
%    anything but zero here means the two are coupled somewhere.
%
%    ⚠️ THIS AND CHECK 4 DO NOT CONTRADICT EACH OTHER, and the numbers say so
%    rather than the prose.  Check 5 is about the COVARIANCE, check 4 about
%    the ESTIMATE, and the flip reaches only the second: across all five
%    flipped pairs sqrt(P22) is identical at EVERY sample -- max difference
%    exactly 0, not merely small -- while the last-60-s estimate error moves
%    by up to 19.1x.  A pair that could not both be right would show a
%    non-zero delta here; this one shows zero, to the bit.
%
%    Widened from one pair at one instant to all five pairs at every sample:
%    the old form could have missed a coupling that cancelled by 240 s.
K.p22_selector_delta = 0;
for q = 1:size(allPairs,1)
    a = C.(allPairs{q,1});  b = C.(allPairs{q,2});
    K.p22_selector_delta = max(K.p22_selector_delta, max(abs(a.SP22 - b.SP22)));
end
K.p22_pairs = size(allPairs,1);
names{end+1} = 'the gain selector cannot move sqrt(P22) (benchmarks 3-8 immune)';
oks(end+1)   = K.p22_selector_delta == 0;
detail{end+1} = sprintf(['max |sqrt(P22)(flipped) - sqrt(P22)(resolved)| = ' ...
    '%.3e ft/s over every sample of all %d flipped pairs, while the estimate ' ...
    'error moves up to %.1fx (check 4)'], K.p22_selector_delta, ...
    K.p22_pairs, max(K.flip_ratio));

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
%ISBOOKCASE  The trajectory and radar of Listing 33.2.  Off it, every book row
%   becomes a SKIP rather than a pass -- a check that did not happen is not the
%   same thing as one that succeeded.  HINT is deliberately NOT included: it is
%   an integration step, not a modelling choice, and it is reported instead.
tf = o.RDESKM == 7000 && o.RDESRKM == 560 && o.TLOFT == 500 && ...
     o.TFINISH == 240 && o.TUPT == 20 && o.TGRAVEND == 100 && ...
     o.GAMDEGIC == 89.8 && o.TS == 1 && o.SIGTHET == 0.001 && ...
     abs(o.SIGR - 10*3.28) < 1e-12;
end


% =============================================================================
function B = benchmarks(TR, C, o)
%BENCHMARKS  Book values only.  Line numbers are in each row's .source and
%   every one was grepped out of zarchan_decoded.txt -- nothing was read off a
%   figure, because the extraction turned every figure into an image link.
B = emptyBench();
bc = isBookCase(o);
why = ['off Listing 33.2''s own trajectory/radar case, so the published ' ...
       'number is not about this run'];

% ---- 1  radar acquisition ------------------------------------------------
kAcq = find(TR.ISEE, 1, 'first');
if ~bc
    B = addSkip(B, 'radar acquires the ICBM', '90', why, ...
        'prose, zarchan.txt:12691 and :12715', 's');
elseif isempty(kAcq)
    B = addSkip(B, 'radar acquires the ICBM', '90', ...
        'the radar never saw the target: ISEE is false at every sample', ...
        'prose, zarchan.txt:12691 and :12715', 's');
else
    B = addVal(B, 'radar acquires the ICBM', 90, TR.T(kAcq), '10%', 0.10, ...
        'prose, zarchan.txt:12691 and :12715', 's', ...
        sprintf(['first sample with 2 < ELDEG < 85 deg; ELDEG there = ' ...
        '%.3f deg, slant range %.1f km'], TR.ELDEG(kAcq), TR.RR(kAcq)/3280));
end

% ---- 2  dynamic pressure back through 600 lb/ft^2 ------------------------
[qpk, kpk] = max(TR.QPRES);
kq = kpk - 1 + find(TR.QPRES(kpk:end) < 600, 1, 'first');
if ~bc
    B = addSkip(B, 'dynamic pressure falls back through 600 lb/ft^2', '65', ...
        why, 'prose, zarchan.txt:12689-12690 (Fig. 33.3)', 's');
elseif isempty(kq)
    B = addSkip(B, 'dynamic pressure falls back through 600 lb/ft^2', '65', ...
        sprintf('Q never returns below 600 lb/ft^2 (peak %.1f)', qpk), ...
        'prose, zarchan.txt:12689-12690 (Fig. 33.3)', 's');
else
    B = addVal(B, 'dynamic pressure falls back through 600 lb/ft^2', 65, ...
        TR.T(kq), '15%', 0.15, ...
        'prose, zarchan.txt:12689-12690 (Fig. 33.3)', 's', ...
        sprintf(['taken AFTER the peak (%.0f lb/ft^2 at %.0f s); Q at 100 s ' ...
        'is %.1f lb/ft^2, which the book calls "below 50"'], qpk, TR.T(kpk), ...
        lastAt(TR.QPRES, TR.T, 100)));
end

% ---- 3-6  theoretical velocity error at 240 s ----------------------------
B = addSigmaRow(B, bc, why, ...
    'two-state, perfect template, Phi_s = 100', 40, C.phis100.sigv240, 0.30, ...
    'Figs. 33.7/33.10, zarchan.txt:12902 and :12905');
B = addSigmaRow(B, bc, why, ...
    'two-state, 10% template error, Phi_s = 1000', 90, C.phis1000.sigv240, ...
    0.30, 'Fig. 33.10, zarchan.txt:12905');
B = addSigmaRow(B, bc, why, ...
    'two-state, gravity turn, Phi_s = 50000', 350, C.phis50000.sigv240, ...
    0.30, 'Fig. 33.12, zarchan.txt:13020');
B = addSigmaRow(B, bc, why, ...
    'three-state polynomial, Phi_s = 260', 200, C.three260.sigv240, 0.40, ...
    'Fig. 33.14, zarchan.txt:13023 ("approximately")');

% ---- 7  ⭐ THE ORDERING.  The chapter's whole argument. -------------------
s3 = C.three260.sigv240;  s2 = C.phis50000.sigv240;
if ~bc
    B = addSkip(B, 'ORDERING: three-state beats two-state gravity turn', ...
        'sigma(3-state) < sigma(2-state, grav turn)', why, ...
        'zarchan.txt:13023 and Fig. 33.15 :13046', 'ft/s');
else
    B = addBound(B, 'ORDERING: three-state beats two-state gravity turn', ...
        'sigma(3-state) < sigma(2-state, gravity turn) at 240 s', ...
        sprintf('%.2f < %.2f ft/s', s3, s2), s3 < s2, ...
        'zarchan.txt:13023 ("it is better than") and Fig. 33.15 :13046', ...
        'ft/s', ['both filters carry enough process noise to be consistent, ' ...
        'so the Riccati bounds are comparable -- that is :13046''s premise']);
end

% ---- 8  and by how much --------------------------------------------------
if ~bc
    B = addSkip(B, 'that ratio ("nearly half")', '0.5', why, ...
        'Fig. 33.15, zarchan.txt:13046', '-');
elseif ~(s2 > 0)
    B = addSkip(B, 'that ratio ("nearly half")', '0.5', ...
        'the two-state gravity-turn bound is zero or not finite', ...
        'Fig. 33.15, zarchan.txt:13046', '-');
else
    B = addVal(B, 'that ratio ("nearly half")', 0.5, s3/s2, '25%', 0.25, ...
        'Fig. 33.15, zarchan.txt:13046', '-', ...
        ['⚠️ the book states this twice and disagrees with itself: "nearly ' ...
         'half" at :13046 against its own 200/350 = 0.571 from :13020 and ' ...
         ':13023.  0.5 is scored, the band admits both, both lines cited.']);
end

% ---- 9  SHAPE: divergence, and its cure ----------------------------------
c0 = C.phis0.cons;  c100 = C.phis100.cons;
if ~bc
    B = addSkip(B, 'Phi_s = 0 diverges, Phi_s = 100 does not', ...
        'RMS(|err|/sigma) > 1 at Phi_s = 0 and smaller at Phi_s = 100', ...
        why, 'zarchan.txt:12879 (Fig. 33.6) and :12902 (Fig. 33.7)', '-');
else
    B = addShape(B, 'Phi_s = 0 diverges, Phi_s = 100 does not', ...
        'RMS(|err|/sigma) over the last 60 s: > 1 at Phi_s = 0, and smaller at Phi_s = 100', ...
        sprintf('%.3f at Phi_s = 0, %.3f at Phi_s = 100', c0, c100), ...
        isfinite(c0) && isfinite(c100) && c0 > 1 && c0 > c100, ...
        'zarchan.txt:12879 (Fig. 33.6) and :12902 (Fig. 33.7)', ...
        sprintf(['same trajectory and same noise draw (seed %d), so the ' ...
        'only difference between the two runs is Phi_s'], o.seed));
end

% ---- NOT SCORED ----------------------------------------------------------
B = addSkip(B, 'apogee of the 7000-km trajectory', '1800', ...
    ['Listing 33.2 sets TFINISH = 240 s and the run ends in boost, at ' ...
     'roughly a fifth of that altitude.  The 1800 km figure belongs to ' ...
     'Listing 33.1 with TFINISH = 999999; reproducing it needs a different ' ...
     'run, and widening anything to absorb it would be inventing an apogee.'], ...
    'Fig. 33.2, zarchan.txt:12627', 'km');
end


% =============================================================================
function B = addSigmaRow(B, bc, why, what, book, ours, tol, source)
%ADDSIGMAROW  The four "theoretical velocity error at 240 s" rows, built the
%   same way so a band cannot drift between them.
name = sprintf('theoretical velocity error at 240 s -- %s', what);
if ~bc
    B = addSkip(B, name, sprintf('%g', book), why, source, 'ft/s');
else
    B = addVal(B, name, book, ours, sprintf('%g%%', 100*tol), tol, source, ...
        'ft/s', ['sqrt(P22) from the Riccati -- a NOISE-FREE quantity, ' ...
        'driven only by the process noise and by SIGX(t)']);
end
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match pushRow exactly.  Schema is the
%   one mgs.zarchan.benchSuite's shared benchRows() consumes.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = pushRow(B, what, kind, book, book_str, ours, ours_str, pct, ...
                     band, tol, ok, skipped, why, source, units, detail)
%PUSHROW  ⭐ THE ONE CONSTRUCTOR.  Every row in this file goes through here, so
%   the field order cannot drift between row kinds -- benchSuite's adaptGeneric
%   is strict and a near-miss schema ERRORS rather than being accommodated.
i = numel(B) + 1;
B(i).what = what;          B(i).kind = kind;
B(i).book = book;          B(i).book_str = book_str;
B(i).ours = ours;          B(i).ours_str = ours_str;
B(i).pct  = pct;           B(i).band = band;
B(i).tol  = tol;           B(i).ok = logical(ok);
B(i).skipped = logical(skipped);  B(i).why = why;
B(i).source = source;      B(i).units = units;  B(i).detail = detail;
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
if ~isfinite(ours) || book == 0
    B = pushRow(B, what, 'value', book, sprintf('%.10g', book), ours, ...
        sprintf('%.10g', ours), NaN, band, tol, false, true, ...
        'the computed value is not finite (or the book value is zero)', ...
        source, units, detail);
    return
end
pct = 100*(ours/book - 1);
B = pushRow(B, what, 'value', book, sprintf('%.10g', book), ours, ...
    sprintf('%.10g', ours), pct, band, tol, abs(ours/book - 1) <= tol, ...
    false, '', source, units, detail);
end

function B = addShape(B, what, bookStr, oursStr, ok, source, detail)
B = pushRow(B, what, 'shape', NaN, bookStr, NaN, oursStr, NaN, 'shape', ...
    NaN, ok, false, '', source, '-', detail);
end

function B = addBound(B, what, bookStr, oursStr, ok, source, units, detail)
B = pushRow(B, what, 'bound', NaN, bookStr, NaN, oursStr, NaN, 'bound', ...
    NaN, ok, false, '', source, units, detail);
end

function B = addSkip(B, what, bookStr, why, source, units)
%ADDSKIP  A check that COULD NOT RUN.  Never a pass: a check that did not
%   happen is a different thing from one that succeeded.
B = pushRow(B, what, 'skip', NaN, bookStr, NaN, '--', NaN, 'skip', NaN, ...
    false, true, why, source, units, '');
end


% =============================================================================
function s = render(R, ev, bookCase)
L = {};  p = R.p;
L{end+1} = 'Zarchan Listing 33.2 -- TWO LINEAR DECOUPLED POLYNOMIAL KALMAN FILTERS vs a BOOSTING ICBM';
L{end+1} = '*** UNITS ARE FEET AND ft/s.  km = 3280 ft, deg = 57.3*rad, g = 32.2 ft/s^2. ***';
L{end+1} = sprintf(['   %g-km trajectory, TLOFT = %g, radar at %.1f km; boost ends ' ...
    'at %g s;  HINT = %g s, TS = %g s, %d samples'], p.RDESKM, p.TLOFT, ...
    p.radar_km, p.TFINISH, p.HINT, p.TS, p.nsamples);
L{end+1} = sprintf(['   radar: SIGR = %.4g ft (= %.1f m), SIGTHET = %.4g rad ' ...
    '(= %.1f mr), gate 2 < ELDEG < 85 deg, seed %d'], p.SIGR, p.SIGR/3.28, ...
    p.SIGTHET, 1000*p.SIGTHET, p.seed);
L{end+1} = sprintf(['   Lambert: TF = %.2f s, %d calls, worst case %d iterations, ' ...
    '%d capped'], p.TF_lambert, p.lambert_calls, p.lambert_iters_max, ...
    p.lambert_stuck);
L{end+1} = sprintf('   nominal run: IFILTER = %g, PHIS = %g, PHIS1 = %g, ERR = %g, QGRAV = %g', ...
    p.IFILTER, p.PHIS, p.PHIS1, p.ERR, p.QGRAV);
L{end+1} = '';

L{end+1} = 'TRUTH TRAJECTORY  (the listing''s own arrays; ArrayDISTNM is KILOMETRES)';
L{end+1} = sprintf('%8s %12s %12s %13s %10s %8s', 't [s]', 'downrng[km]', ...
    'alt [km]', 'Q [lb/ft^2]', 'ELDEG', 'ISEE');
if ~isempty(R.t)
    stride = max(1, round(numel(R.t)/10));
    rows = unique([1:stride:numel(R.t), numel(R.t)]);
    for r = rows
        L{end+1} = sprintf('%8.0f %12.1f %12.1f %13.1f %10.2f %8d', ...
            R.t(r), R.dist_km(r), R.alt_km(r), R.qpres(r), R.eldeg(r), ...
            R.isee(r)); %#ok<AGROW>
    end
end
L{end+1} = '';

L{end+1} = 'THE FIVE BOOK CONFIGURATIONS  (sqrt(P22) at 240 s, and filter consistency)';
L{end+1} = sprintf('%-42s %14s %14s', 'configuration', 'sigma_v [ft/s]', ...
    'RMS|err|/sigma');
nm = {'2-state, perfect template, Phi_s = 0     (Fig 33.6)', ...
      '2-state, perfect template, Phi_s = 100   (Fig 33.7)', ...
      '2-state, 10% template err, Phi_s = 1000  (Fig 33.10)', ...
      '2-state, gravity turn,     Phi_s = 50000 (Fig 33.12)', ...
      '3-state polynomial,        Phi_s = 260   (Fig 33.14)'};
cs = {R.cases.phis0, R.cases.phis100, R.cases.phis1000, ...
      R.cases.phis50000, R.cases.three260};
for i = 1:numel(nm)
    L{end+1} = sprintf('%-42s %14.2f %14.3f', nm{i}, cs{i}.sigv240, ...
        cs{i}.cons); %#ok<AGROW>
end
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-62s %s', vstr(K.oks(i)), K.names{i}, ...
        K.detail{i}); %#ok<AGROW>
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
            case 'skip'
                L{end+1} = sprintf('   SKIP %-56s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case {'shape','bound'}
                L{end+1} = sprintf('   %-4s %-56s %s', vstr(b.ok), b.what, ...
                    b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf(['   %-4s %-56s book %s %s / ours %s %s ' ...
                    '(%+.3f%%, band %s)'], vstr(b.ok), b.what, b.book_str, ...
                    b.units, b.ours_str, b.units, b.pct, b.band); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
if ~bookCase
    L{end+1} = '';
    L{end+1} = '   NOTE: off Listing 33.2''s own case, so the book rows are SKIPPED, not passed.';
end
L{end+1} = '';
L{end+1} = 'REMINDER: the one comparison this transcription had to resolve is the';
L{end+1} = '   least-squares / Kalman gain selector.  Structural check 4 is the delete';
L{end+1} = '   experiment for it -- run on the configurations whose propagation template';
L{end+1} = '   is imperfect, because with a PERFECT template the gain does no work and';
L{end+1} = '   the flip is measurably harmless (check 4B is that control).  Check 5 shows';
L{end+1} = '   the selector cannot reach benchmarks 3-8 at all.';
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
figure;
subplot(2,1,1);
plot(R.dist_km, R.alt_km); grid on;
xlabel('Downrange (km)'); ylabel('Altitude (km)');
title('Listing 33.2 -- boost-phase trajectory');

subplot(2,1,2);
plot(R.t, R.errxtd, R.t, R.sp22, R.t, R.sp22p); grid on;
xlabel('Time (s)'); ylabel('Velocity Error (ft/s)');
% The listing's own axis.  It starts at 90 s because the filter has not been
% fed a measurement before then and ArraySP22 is still sqrt(1e11).
axis([90 R.p.TFINISH -500 500]);
legend('single flight', '+theory', '-theory', 'Location', 'best');
end
