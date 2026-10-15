function R = gravityField(opts)
%MGS.ZARCHAN.GRAVITYFIELD  Zarchan Listing 11.1 -- gravity field simulation:
%   ONE launch flown TWICE in the same loop, by a flat-Earth constant-g model
%   and by an Earth-centred inverse-square model, and compared.
%
%   R = mgs.zarchan.gravityField()
%   R = mgs.zarchan.gravityField('V', 6000)                 one speed only
%   R = mgs.zarchan.gravityField('assert', false, 'plot', true)
%
%   Line-by-line port of Listing 11.1 (zarchan.txt:3245-3259), integrated by
%   the book's own two-pass second-order Runge-Kutta (Heun) at H = 0.01 s.
%   'V' is a VECTOR, defaulting to the book's three cases [3000 6000 12000]
%   ft/s: every published number here is a comparison ACROSS those speeds
%   (:3223, :3233, :3234), so one speed alone cannot score the section.
%
%   *** WE DO NOT FLY CHAPTER 11. ***  It opens the orbital half of the book:
%   no seeker, no autopilot, no guidance law, no target, no atmosphere, no
%   drag, no thrust -- a point mass in a gravity field, and in one of the two
%   channels a FLAT EARTH.  No number out of this file belongs in a sortie
%   argument.  It is here because it is the book's own audit of WHEN A
%   MODELLING ASSUMPTION STOPS BEING FREE: the flat-Earth constant-g model the
%   whole tactical half of Zarchan rests on is indistinguishable from the
%   truth at 3000 ft/s and 12% short in range at 12000 ft/s -- same code, no
%   symptom but the two curves separating.
%
%   UNITS ARE THE BOOK'S: ft, ft/s, ft/s^2; g = 32.2, gm = 1.4077e16 ft^3/s^2
%   (:3189), a = 2.0926e7 ft.  Distances REPORT in n.mi. at the book's own
%   6076 ft/n.mi. because the listing divides by 6076 itself.  Degrees convert
%   with the listing's 57.3 and the launch heading uses its 1.5708 for pi/2.
%   Both approximations are KEPT, not improved: 1.5708 tilts the initial
%   velocity by 3.67e-6 rad, which is what BENCHMARK 8 measures and why that
%   row is banded at 1e-5 and not 1e-12.  Nothing is SI; convert at the call
%   site and say so there.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION (matlab/zarchan/_listings/listing_11_01.m).  Its header
%   reports NO unresolved variable-to-variable comparisons and that is right:
%   every comparison in 11.1 is against a LITERAL.  Repairs, all shape-forced:
%     1  "while ALTNM . -.0001" -> ">".  "." is this book's OCR of ">"; ALTNM
%        starts at 0, so "<" never enters the loop and nothing would print.
%     2  "while STEP ,=1" -> "<=".  "," is the OCR of "<"; the body sets
%        STEP = 2 to leave, which only works against "<=".
%     3  "if S.=1.99999" -> ">=".  S is zeroed on every record, so "<=" would
%        record forever.  Written here as "S >= samp - 0.5*H": the same steps
%        at the listing's H = 0.01, samp = 2 s, without the magic 1.99999.
%     4  Trailing "POLAR COORDINATE SYSTEM [4, 5]" is the book's next SECTION
%        HEADING leaking in (:3260).  Dropped, not "fixed" into code.
%     5  "save datfil.txt output /ascii" is the book's recurring typo for
%        -ascii.  Nothing is written to disk here: recorded, dropped.
%
%   TWO BOOK ERRATA, both dead code at the published ALTNM = 0 (so this port
%   is bit-identical to the listing there), both repaired -- see R.p.repairs:
%     a  "ALT=ALTNM/6076." converts n.mi. to feet by DIVIDING; Listings 11.3
%        (:3459) and 11.4 (:3477) MULTIPLY.  A 1000 n.mi. launch would start
%        at 0.000027 n.mi. and nothing would complain.  (Same erratum, same
%        verdict, in MGS.ZARCHAN.GRAVITYFRAMES.)
%     b  "RT1=ALT*cos(ANG); RT2=ALT*sin(ANG);" puts the launch ALTITUDE into
%        the flat-Earth DOWNRANGE channel at ANG = 0, contradicting the book's
%        own ICs two pages earlier (R1(0)=dr, R2(0)=alt, :3172-3176).  Here
%        RT1 = 0, RT2 = ALT.
%
%   WHAT "RANGE" MEANS HERE.  The loop terminates on ALTNM, recomputed only
%   inside the 2 s record block, so the last recorded point is up to one
%   sample PAST impact -- and the flat channel, whose termination nobody
%   tests, is left flying underground (-191 n.mi. at V = 12000).  Range is
%   therefore the interpolated ALTITUDE ZERO CROSSING, per channel;
%   .term_dist_nm keeps the raw last point.  That is probably where the book's
%   "about 47 n.mi." came from: the crossing is 46.385 and only the last
%   recorded point, 46.577, rounds to 47.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Every book value was grepped out of the DECODED zarchan.txt
%   and its line is in the row's .source.  Nothing was read off a figure --
%   the extraction turned every figure into an image link.
%
%   1-6  The six published trajectory numbers, all prose, all hedged "about":
%          V= 3000: range 47 n.mi., apogee 12   (:3223)
%          V= 6000: range 180, apogee 50        (:3233)
%          V=12000: range MORE THAN 800 (a BOUND), apogee 220   (:3234)
%        Scored against the EARTH-CENTRED channel on the book's own
%        instruction (:3233, "the correct answers are the ones given by the
%        Earth-centered ... differential equations"); the flat counterpart is
%        in each row's .detail, and at V = 12000 it is 16% low, which is what
%        settles that the book is quoting the round-Earth numbers.
%        BAND 10%, one band for the class, set by PROVENANCE: a
%        two-significant-figure eyeball read off a trajectory plot.  Measured
%        -1.3, -3.1, +5.2, -4.0, (bound), -0.6 %.  Five would survive 5%; the
%        180 row would not, and the reason is the book, not the port: its own
%        trajectory equation puts that range at 189.33 (BENCHMARK 7 agrees to
%        4e-7) and the flat channel at 184.00, and it quotes a single "about
%        180" for the pair.  Do not widen this band further, and do not narrow
%        it to make a point -- a red row here is a claim about the author's
%        eyesight, not about the integrator.
%
%   7    r(theta) vs the book's trajectory equation (:3433, lambda at :3399),
%        every recorded sample of every case, worst scored, band 1e-5 of r0
%        (measured 4.4e-7, about 9 ft).  This is what makes the port
%        trustworthy independently of anybody's plot-reading: an exact
%        solution of the transcribed ODEs, sharing no code with the
%        integrator, pinning range AND apogee AND the shape between them.
%        SUBORBITAL ONLY -- theta comes from an acos and folds above pi, so
%        the row skips itself past 170 deg of central angle.
%
%   8    The first integral r^2*thetadot = (a+alt)*V*cos(gam) (:3318, restated
%        :3349) at launch, band 1e-5 -- not 1e-12, and the reason is
%        arithmetic rather than integration: 1.5708 - pi/2 tilts h by exactly
%        +3.673e-6 before the first step.  A reading far BELOW 3.6e-6 means
%        somebody tidied 1.5708 away.  The DRIFT off this constant over the
%        flight is SELF-CHECK 2 (9e-13).
%
%   9    SHAPE, from three prose claims in sequence -- "the same missile
%        trajectories" (:3223), "fairly accurate" (:3233), "large
%        discrepancies ... travels much farther than the flat-Earth model
%        indicates" (:3234).  An ORDERING: the round-vs-flat range discrepancy
%        positive at all three speeds, strictly increasing, < 2% at 3000 and
%        > 5% at 12000.  Measured 0.84, 2.89, 11.64%.
%
%   NOT SCORED: Ch.11's other numbers (6000 n.mi./800 n.mi. at :3521, flight
%   times "about 1800 s"/"nearly 2800 s" at :3563) are Listing 11.4's, need
%   the hit equation, and are not smuggled in here.
%
% ---------------------------------------------------------------------------
%   SELF-CHECKS (structural -- self-consistency, NEVER evidence of Zarchan):
%   energy conserved (1e-9); angular momentum conserved (1e-9); the flat
%   channel EXACTLY its own analytic parabola (Heun is exact for constant
%   acceleration, so that residual is roundoff -- 7e-6 ft over 640 s); plus
%   two DELETE EXPERIMENTS, both run through the SAME integrator behind a mode
%   flag, because a private copy of the loop would only prove the copy broken:
%     4  delete the Heun corrector (leave the two-pass loop after pass 1).
%        H/2 must move the impact range < 1e-4 n.mi. (converged) and Euler
%        must move it MORE and break check 1.  Measured 2.6e-9 vs 3.6e-3
%        n.mi.; energy drift 2e-13 -> 1.0e-6.
%     5  swap the x and y components of the inverse-square term -- a slip that
%        still runs and still looks like a trajectory.  It must destroy check
%        2 (measured drift 2.1, nine decades past the band).  Without it,
%        check 2 and BENCHMARK 8 could be passing because nothing in the file
%        is able to move them.
%
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED IN MATLAB. ***  This
%   file has never been executed; MATLAB is the caller's.  These come from an
%   independent replica of the same scheme, so a first run has an expectation
%   to meet rather than merely be admired:
%     V=3000 : impact 46.385 n.mi. (flat 46.000), apogee 11.633 (flat 11.502)
%     V=6000 : impact 189.329     (flat 184.003), apogee 47.989 (flat 46.007)
%     V=12000: impact 821.692     (flat 736.016), apogee 218.700 (flat 184.027)
%     drifts <= 6e-13 (energy), 9e-13 (h); trajectory residual <= 4.4e-7 r0;
%     h vs book +3.673e-6; row deviations -1.3, -3.1, +5.2, -4.0, -0.6 %.
%   A residual near a band EDGE is not a pass to be accepted: the mechanisms
%   above say where each one should land.
%
%   OUTPUTS.  R.run(k) per speed: .V .t .rt1_nm .rt2_nm .dist_nm .alt_nm (the
%   listing's five recorded arrays) plus .range_nm .apogee_nm .flat_range_nm
%   .flat_apogee_nm .term_dist_nm .tf_s .lam .ecc .h0 .h_book .drift_E
%   .drift_h .traj_resid .flat_resid .theta_max .completed.  R.bench uses the
%   schema shared with the other ports.  R.selfcheck carries BOTH .names and
%   .oks, or benchSuite boards none of it and the board looks green.  R.pass
%   is true only if a row was SCORED and everything passed.  R.verdict is
%   ALWAYS formed; 'assert' only chooses error vs WARN-and-return-complete,
%   which is how benchSuite runs every port.  Also R.p, R.units, R.summary.
%
%   See also MGS.ZARCHAN.GRAVITYFRAMES (Listing 11.2, the polar/Cartesian
%   twin), MGS.ZARCHAN.ORBITGENERATOR (Listing 11.3), MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 11.1 line 1, verbatim.  Feet, seconds, degrees.
    opts.V      (1,:) double {mustBeFinite, mustBePositive} = [3000 6000 12000]
    opts.GAM    (1,1) double {mustBeFinite} = 45      % deg, /57.3 as the book
    opts.ALTNM  (1,1) double {mustBeFinite, mustBeNonnegative} = 0
    opts.ANGDEG (1,1) double {mustBeFinite} = 0
    opts.H      (1,1) double {mustBeFinite, mustBePositive} = 0.01
    opts.samp   (1,1) double {mustBeFinite, mustBePositive} = 2.0
    % Guard, not a model parameter.  The book's three cases fly 134, 278 and
    % 640 s; a launch that never comes down would otherwise loop forever
    % (the mutated run in SELF-CHECK 5 is exactly such a launch).
    opts.tmax   (1,1) double {mustBeFinite, mustBePositive} = 2000
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

% The listing's own constants, and its own conversions.  1.5708 and 57.3 are
% the book's; see the UNITS block.
A = 2.0926e7;  GM = 1.4077e16;  G = 32.2;  FTNM = 6076;  DEG = 57.3;  QPI = 1.5708;

if opts.samp < opts.H
    error('mgs:zarchan:gravityField:samp', ...
        ['samp = %g s is shorter than the step H = %g s, so the record would ' ...
         'be taken at the step rate while claiming the coarser interval.'], ...
        opts.samp, opts.H);
end
lamchk = (A + opts.ALTNM*FTNM)*max(opts.V)^2/GM;
if lamchk >= 2
    error('mgs:zarchan:gravityField:escape', ...
        ['V = %g ft/s gives lambda = r0*V^2/gm = %.4g >= 2.  The book calls ' ...
         'that escape velocity (:3446): the trajectory is parabolic or ' ...
         'hyperbolic, never returns, and this listing terminates on altitude. ' ...
         'Refused rather than left to spin to tmax.'], max(opts.V), lamchk);
end

Vs = opts.V(:).';
if isempty(Vs)
    error('mgs:zarchan:gravityField:noV', ...
        ['''V'' is empty, so nothing would be flown and nothing compared. ' ...
         'The published cases are [3000 6000 12000] ft/s.']);
end
for k = 1:numel(Vs)
    C = runCase(Vs(k), opts.GAM, opts.ALTNM, opts.ANGDEG, opts.H, opts.samp, ...
                opts.tmax, 0, A, GM, G, FTNM, DEG, QPI);
    if k == 1, runs = C; else, runs(k) = C; end
end

R = struct();
R.run = runs;
R.p = struct('V', Vs, 'GAM', opts.GAM, 'ALTNM', opts.ALTNM, ...
    'ANGDEG', opts.ANGDEG, 'H', opts.H, 'samp', opts.samp, 'tmax', opts.tmax, ...
    'A', A, 'GM', GM, 'G', G, 'FTNM', FTNM, 'DEG', DEG, 'QPI', QPI, ...
    'repairs', {{'ALT = ALTNM*6076 (listing divides; 11.3/11.4 multiply)', ...
                 'flat-Earth IC RT1 = 0, RT2 = ALT (listing swaps them at ANG=0)'}}, ...
    'units', 'ft, ft/s, ft/s^2; n.mi. = 6076 ft; g = 32.2; gm = 1.4077e16 ft^3/s^2');
R.units = R.p.units;

R.selfcheck = structuralChecks(runs, opts, A, GM, G, FTNM, DEG, QPI);

if opts.benchmark
    R.bench = benchmarks(runs, Vs, opts.samp);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    R.pass  = false;    % NOT CHECKED is not the same thing as fine
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R); end

msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the book. ' ...
           'R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = ['not one book value could be evaluated: the published cases are ' ...
           'V = 3000, 6000 and 12000 ft/s at GAM = 45 deg from the surface.'];
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question. ' ...
        'First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf(['%d of %d evaluated book values are outside tolerance ' ...
        '(first: %s, book %s, ours %s).'], numel(bad), sum(ev), ...
        R.bench(bad(1)).what, R.bench(bad(1)).book_str, R.bench(bad(1)).ours_str);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:gravityField:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN.  R comes back complete so the caller can see WHICH
        % side of a comparison moved; benchSuite runs every port with assert
        % off for exactly that reason.
        warning('mgs:zarchan:gravityField:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function C = runCase(V, GAMdeg, ALTNM, ANGDEG, H, samp, tmax, mode, ...
                     A, GM, G, FTNM, DEG, QPI)
%RUNCASE  Listing 11.1, zarchan.txt:3245-3259.  ONE integrator, three modes:
%     mode 0  the listing
%     mode 1  DELETE the Heun corrector (leave after pass 1) -- self-check 4
%     mode 2  MUTATE gravity to a non-central field          -- self-check 5
%   The delete experiments run through this same function on purpose.  A
%   private copy of the loop would prove only that the copy is broken.
%
%   Every constant arrives as an ARGUMENT: MATLAB local functions cannot see
%   the caller's workspace.

ALT  = ALTNM*FTNM;             % erratum (a): the listing DIVIDES.  See header.
ANG  = ANGDEG/DEG;
gamr = GAMdeg/DEG;
VRX  = V*cos(QPI - gamr + ANG);
VRY  = V*sin(QPI - gamr + ANG);

RT1 = 0.0;   RT2 = ALT;        % erratum (b): the listing writes ALT*cos, ALT*sin
VT1 = VRX;   VT2 = VRY;
X   = (A+ALT)*cos(ANG);  Y = (A+ALT)*sin(ANG);
XFIRST = X;  YFIRST = Y;  X1 = VRX;  Y1 = VRY;
T = 0.0;  S = 0.0;  ALTNMc = ALTNM;

r0  = A + ALT;
lam = r0*V*V/GM;
cg  = cos(gamr);
RF  = sqrt(XFIRST^2 + YFIRST^2);
h0  = X*Y1 - Y*X1;
E0  = 0.5*(X1^2 + Y1^2) - GM/sqrt(X^2 + Y^2);

% Derivatives initialised before the loop so the FLAG==1 branch cannot read a
% value left over from anything else.
RT1D = 0; RT2D = 0; VT1D = 0; VT2D = 0; XD = 0; YD = 0; X1D = 0; Y1D = 0;

nmax = ceil(tmax/samp) + 4;
t = zeros(nmax,1);  rt1 = zeros(nmax,1);  rt2 = zeros(nmax,1);
dst = zeros(nmax,1);  alt = zeros(nmax,1);
n = 0;  drE = 0;  drH = 0;  trj = 0;  flt = 0;  thmax = 0;

while ALTNMc > -0.0001 && T < tmax
    RT1OLD = RT1; RT2OLD = RT2; VT1OLD = VT1; VT2OLD = VT2;
    XOLD = X; YOLD = Y; X1OLD = X1; Y1OLD = Y1;
    STEP = 1;  FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            RT1 = RT1 + H*RT1D;  RT2 = RT2 + H*RT2D;
            VT1 = VT1 + H*VT1D;  VT2 = VT2 + H*VT2D;
            X  = X  + H*XD;      Y  = Y  + H*YD;
            X1 = X1 + H*X1D;     Y1 = Y1 + H*Y1D;
            T  = T + H;
        end
        AT1 = 0.0;  AT2 = -G;                 % flat Earth: constant, downward
        RT1D = VT1;  RT2D = VT2;  VT1D = AT1;  VT2D = AT2;
        TEMBOT = (X^2 + Y^2)^1.5;             % Newton, Earth-centred
        if mode == 2
            X1D = -GM*Y/TEMBOT;  Y1D = -GM*X/TEMBOT;   % MUTATION: not central
        else
            X1D = -GM*X/TEMBOT;  Y1D = -GM*Y/TEMBOT;
        end
        XD = X1;  YD = Y1;
        FLAG = 1;
        if mode == 1, STEP = 2; end           % DELETE: no second pass
    end
    % The listing writes "FLAG=0" here too; the outer loop re-initialises it,
    % so the assignment cannot be read.  Kept as a comment, not as code.
    if mode == 1
        RT1 = RT1OLD + H*RT1D;  RT2 = RT2OLD + H*RT2D;
        VT1 = VT1OLD + H*VT1D;  VT2 = VT2OLD + H*VT2D;
        X   = XOLD  + H*XD;     Y   = YOLD  + H*YD;
        X1  = X1OLD + H*X1D;    Y1  = Y1OLD + H*Y1D;
        T   = T + H;
    else
        RT1 = (RT1OLD+RT1)/2 + 0.5*H*RT1D;   RT2 = (RT2OLD+RT2)/2 + 0.5*H*RT2D;
        VT1 = (VT1OLD+VT1)/2 + 0.5*H*VT1D;   VT2 = (VT2OLD+VT2)/2 + 0.5*H*VT2D;
        X   = (XOLD+X)/2   + 0.5*H*XD;       Y   = (YOLD+Y)/2   + 0.5*H*YD;
        X1  = (X1OLD+X1)/2 + 0.5*H*X1D;      Y1  = (Y1OLD+Y1)/2 + 0.5*H*Y1D;
    end

    S = S + H;
    if S >= samp - 0.5*H
        S = 0.0;
        r = sqrt(X^2 + Y^2);
        ALTNMc = (r - A)/FTNM;
        CBETA = (X*XFIRST + Y*YFIRST)/(r*RF);
        CBETA = min(1, max(-1, CBETA));   % the book does not clamp; a 1-ulp
                                          % overshoot would make BETA COMPLEX
                                          % and propagate silently
        BETA = acos(CBETA);
        n = n + 1;
        t(n) = T;  rt1(n) = RT1/FTNM;  rt2(n) = RT2/FTNM;
        dst(n) = A*BETA/FTNM;  alt(n) = ALTNMc;

        % Invariants, evaluated at the RECORDED samples only.  Per-step would
        % add an acos to every step for no extra information: all four of
        % these drift monotonically.
        drE = max(drE, abs((0.5*(X1^2+Y1^2) - GM/r)/E0 - 1));
        drH = max(drH, abs((X*Y1 - Y*X1)/h0 - 1));
        den = 1 - cos(BETA) + lam*cg*cos(BETA + gamr);
        trj = max(trj, abs(r - r0*lam*cg^2/den)/r0);      % book :3433
        flt = max([flt, abs(RT1 - VRX*T), ...
                        abs(RT2 - (ALT + VRY*T - 0.5*G*T^2))]);
        thmax = max(thmax, BETA);
    end
end

t = t(1:n);  rt1 = rt1(1:n);  rt2 = rt2(1:n);  dst = dst(1:n);  alt = alt(1:n);

C = struct();
C.V = V;  C.t = t;  C.rt1_nm = rt1;  C.rt2_nm = rt2;
C.dist_nm = dst;  C.alt_nm = alt;
C.completed = ~isempty(alt) && alt(end) <= -0.0001;
C.tf_s          = lastOrNaN(t);
C.term_dist_nm  = lastOrNaN(dst);
C.range_nm      = zeroCross(alt, dst);
C.flat_range_nm = zeroCross(rt2, rt1);
C.apogee_nm      = maxOrNaN(alt);
C.flat_apogee_nm = maxOrNaN(rt2);
C.lam = lam;
C.ecc = sqrt(max(0, 1 + lam*(lam - 2)*cg^2));    % book :3442
C.h0  = h0;
C.h_book = r0*V*cg;                              % book :3318, :3349
C.E0 = E0;
C.drift_E = drE;  C.drift_h = drH;
C.traj_resid = trj;  C.flat_resid = flt;
C.theta_max = thmax;  C.nrec = n;
end


% =============================================================================
function v = lastOrNaN(x)
if isempty(x), v = NaN; else, v = x(end); end
end

function v = maxOrNaN(x)
if isempty(x), v = NaN; else, v = max(x); end
end

function xc = zeroCross(alt, x)
%ZEROCROSS  First downward crossing of altitude = 0, linearly interpolated.
%   NOT the last recorded point: the loop stops on the Earth-centred altitude
%   and the flat channel is left flying underground.  See the header.
xc = NaN;
for k = 2:numel(alt)
    if alt(k-1) >= 0 && alt(k) < 0
        xc = x(k-1) + (x(k) - x(k-1))*(0 - alt(k-1))/(alt(k) - alt(k-1));
        return
    end
end
end


% =============================================================================
function K = structuralChecks(runs, opts, A, GM, G, FTNM, DEG, QPI)
%STRUCTURALCHECKS  Self-consistency, NOT book values, and never evidence that
%   this port reproduces Zarchan.  .names AND .oks, both, or benchSuite puts
%   none of this on the board and the board looks green.
K = struct();
names = {};  oks = [];  detail = {};

done = all([runs.completed]);

K.drift_E = max([runs.drift_E]);
names{end+1} = 'specific energy conserved (Earth-centred, rel < 1e-9)';
oks(end+1)   = K.drift_E < 1e-9 && done;
detail{end+1} = sprintf('worst %.3g over %d case(s)', K.drift_E, numel(runs));

K.drift_h = max([runs.drift_h]);
names{end+1} = 'angular momentum conserved (rel < 1e-9)';
oks(end+1)   = K.drift_h < 1e-9 && done;
detail{end+1} = sprintf('worst %.3g', K.drift_h);

K.flat_resid = max([runs.flat_resid]);
names{end+1} = 'flat channel == its analytic parabola (Heun is exact there)';
oks(end+1)   = K.flat_resid < 1e-3;
detail{end+1} = sprintf('worst %.3g ft (roundoff; band 1e-3 ft)', K.flat_resid);

% ---- 4  DELETE EXPERIMENT: refinement vs corrector ------------------------
V1 = runs(1).V;
Chalf = runCase(V1, opts.GAM, opts.ALTNM, opts.ANGDEG, opts.H/2, opts.samp, ...
                opts.tmax, 0, A, GM, G, FTNM, DEG, QPI);
Ceul  = runCase(V1, opts.GAM, opts.ALTNM, opts.ANGDEG, opts.H, opts.samp, ...
                opts.tmax, 1, A, GM, G, FTNM, DEG, QPI);
K.d_half  = Chalf.range_nm - runs(1).range_nm;
K.d_euler = Ceul.range_nm  - runs(1).range_nm;
K.euler_drift_E = Ceul.drift_E;
names{end+1} = 'DELETE: Heun corrector is load-bearing (and H/2 is converged)';
oks(end+1)   = abs(K.d_half) < 1e-4 && abs(K.d_euler) > 1e-4 && ...
               K.euler_drift_E > 1e-9;
detail{end+1} = sprintf(['H/2 moves range %+.3g n.mi.; Euler moves %+.3g and ' ...
    'takes energy drift to %.3g'], K.d_half, K.d_euler, K.euler_drift_E);

% ---- 5  DELETE EXPERIMENT: gravity really is central ----------------------
%      Capped at the nominal flight time: the mutated field does not bring the
%      missile back, so this run would otherwise spin to tmax.
tcap = min(opts.tmax, max(runs(1).tf_s, 10*opts.samp));
Cmut = runCase(V1, opts.GAM, opts.ALTNM, opts.ANGDEG, opts.H, opts.samp, ...
               tcap, 2, A, GM, G, FTNM, DEG, QPI);
K.mut_drift_h = Cmut.drift_h;
names{end+1} = 'DELETE: swapping the gravity components destroys check 2';
oks(end+1)   = K.mut_drift_h > 1e-3;
detail{end+1} = sprintf('mutated h drift %.3g (check 2 is banded at 1e-9)', ...
                        K.mut_drift_h);

K.names = names;  K.oks = logical(oks);  K.detail = detail;
K.all_ok = all(K.oks);
K.first_bad = '';
if ~K.all_ok
    j = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{j}, detail{j});
end
end


% =============================================================================
function B = benchmarks(runs, Vs, samp)
%BENCHMARKS  Computed vs published.  EVERY book value was grepped out of the
%   decoded zarchan.txt and the line number is in .source.  Bands come from
%   the PROVENANCE of the book value, never from what we happen to compute.
B = emptyBench();
TOLP = 0.10;                   % "about NNN": two significant figures, off a plot
s3  = 'p.233 text (zarchan.txt:3223, Fig. 11.4)';
s6  = 'p.234 text (zarchan.txt:3233, Fig. 11.5)';
s12 = 'p.234 text (zarchan.txt:3234, Fig. 11.6)';
whyNo = @(v) sprintf(['this run did not include V = %g ft/s (it flew %s), so ' ...
    'the published case was not reproduced'], v, mat2str(Vs));

j3 = findV(Vs, 3000);  j6 = findV(Vs, 6000);  j12 = findV(Vs, 12000);

% ---- 1-2  V = 3000 ft/s ---------------------------------------------------
if ~isempty(j3)
    C = runs(j3);
    B = addVal(B, 'range at V = 3000 ft/s', 47, C.range_nm, 'figure-read', ...
        TOLP, s3, 'n.mi.', sprintf(['Earth-centred impact (zero crossing). ' ...
        'Flat channel %.3f; last recorded point %.3f -- which is the value ' ...
        'that rounds to 47'], C.flat_range_nm, C.term_dist_nm));
    B = addVal(B, 'peak altitude at V = 3000 ft/s', 12, C.apogee_nm, ...
        'figure-read', TOLP, s3, 'n.mi.', ...
        sprintf('flat channel %.3f n.mi.; apogee sampled on the %g s grid', ...
                C.flat_apogee_nm, samp));
else
    B = addSkip(B, 'range at V = 3000 ft/s', '47', whyNo(3000), s3, 'n.mi.');
    B = addSkip(B, 'peak altitude at V = 3000 ft/s', '12', whyNo(3000), s3, 'n.mi.');
end

% ---- 3-4  V = 6000 ft/s ---------------------------------------------------
if ~isempty(j6)
    C = runs(j6);
    B = addVal(B, 'range at V = 6000 ft/s', 180, C.range_nm, 'figure-read', ...
        TOLP, s6, 'n.mi.', sprintf(['THE LOOSEST ROW IN THE FILE. Flat ' ...
        'channel %.3f, Earth-centred %.3f; the book quotes a single "about ' ...
        '180" for the pair. The trajectory-equation row pins this same ' ...
        'trajectory to the book''s closed form at 4e-7'], ...
        C.flat_range_nm, C.range_nm));
    B = addVal(B, 'peak altitude at V = 6000 ft/s', 50, C.apogee_nm, ...
        'figure-read', TOLP, s6, 'n.mi.', ...
        sprintf('flat channel %.3f n.mi.', C.flat_apogee_nm));
else
    B = addSkip(B, 'range at V = 6000 ft/s', '180', whyNo(6000), s6, 'n.mi.');
    B = addSkip(B, 'peak altitude at V = 6000 ft/s', '50', whyNo(6000), s6, 'n.mi.');
end

% ---- 5-6  V = 12000 ft/s --------------------------------------------------
if ~isempty(j12)
    C = runs(j12);
    B = addBound(B, 'range at V = 12000 ft/s', '"more than 800 n.mi."', ...
        sprintf('%.3f n.mi.', C.range_nm), C.range_nm > 800, s12, 'n.mi.', ...
        sprintf(['a BOUND, not a value: the book says "more than 800". Flat ' ...
        'channel %.3f, i.e. the flat model under-reports by %.1f%%'], ...
        C.flat_range_nm, 100*(1 - C.flat_range_nm/C.range_nm)));
    B = addVal(B, 'peak altitude at V = 12000 ft/s', 220, C.apogee_nm, ...
        'figure-read', TOLP, s12, 'n.mi.', sprintf(['flat channel %.3f, ' ...
        '%.0f%% low -- so the book''s 220 is unambiguously the Earth-centred ' ...
        'number, which is why every prose row is scored against that channel'], ...
        C.flat_apogee_nm, 100*(1 - C.flat_apogee_nm/C.apogee_nm)));
else
    B = addSkip(B, 'range at V = 12000 ft/s', '> 800', whyNo(12000), s12, 'n.mi.');
    B = addSkip(B, 'peak altitude at V = 12000 ft/s', '220', whyNo(12000), s12, 'n.mi.');
end

% ---- 7  the trajectory equation ------------------------------------------
trj    = max([runs.traj_resid]);
thmax  = max([runs.theta_max]);
srcTrj = 'p.245 closed form (zarchan.txt:3433; lambda at :3399)';
if thmax < 170/57.3
    B = addBound(B, 'r(theta) vs the book trajectory equation, all cases', ...
        '<= 1e-5 of r0', sprintf('%.3g of r0', trj), trj <= 1e-5, srcTrj, '-', ...
        ['exact solution of the transcribed ODEs; shares no code with the ' ...
         'integrator, and pins range, apogee and everything between at once']);
else
    B = addSkip(B, 'r(theta) vs the book trajectory equation, all cases', ...
        '<= 1e-5 of r0', sprintf(['central angle reached %.1f deg: theta comes ' ...
        'from an acos and folds back above 180, so the comparison would be ' ...
        'against the wrong branch'], 57.3*thmax), srcTrj, '-');
end

% ---- 8  the angular-momentum first integral -------------------------------
hr = max(abs([runs.h0]./[runs.h_book] - 1));
B = addBound(B, 'r^2*thetadot vs (a+alt)*V*cos(gam) at launch', ...
    '<= 1e-5 relative', sprintf('%+.3e', hr), hr <= 1e-5, ...
    'p.239 first integral (zarchan.txt:3318, restated :3349)', 'ft^2/s', ...
    ['+3.673e-6 of this is the listing''s own 1.5708 for pi/2, present ' ...
     'before the first step is taken.  A reading far BELOW 3.6e-6 means ' ...
     'somebody tidied 1.5708 away and the port is no longer the listing']);

% ---- 9  SHAPE: the flat model degrades with speed -------------------------
srcSh = 'zarchan.txt:3223, :3233, :3234';
if ~isempty(j3) && ~isempty(j6) && ~isempty(j12)
    d = arrayfun(@(C) C.range_nm/C.flat_range_nm - 1, runs([j3 j6 j12]));
    okS = all(d > 0) && all(diff(d) > 0) && d(1) < 0.02 && d(3) > 0.05;
    B = addShape(B, 'flat-Earth error grows with speed (round always farther)', ...
        '"same trajectories" -> "fairly accurate" -> "large discrepancies"', ...
        sprintf('%+.2f%%, %+.2f%%, %+.2f%% at 3/6/12 kft/s', 100*d), okS, srcSh, ...
        ['an ordering, not a plot reading: positive, strictly increasing, ' ...
         '< 2% at 3000 ft/s and > 5% at 12000 ft/s']);
else
    B = addSkip(B, 'flat-Earth error grows with speed', 'ordering', ...
        'all three published speeds are needed for an ordering', srcSh, '-');
end

% ---- INFO: the two errata -------------------------------------------------
B = addInfo(B, 'listing errata repaired (dead code at the published case)', ...
    'ALT=ALTNM/6076; RT1=ALT*cos(ANG)', 'ALT=ALTNM*6076; RT1=0, RT2=ALT', ...
    'zarchan.txt:3245 vs :3459/:3477 and :3172-3176', ...
    ['both are unreachable at the book''s ALTNM = 0, so this port is ' ...
     'bit-identical to the listing there and cannot be scored on them']);
end

function j = findV(Vs, v)
j = find(abs(Vs - v) < 1e-9, 1);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly.  Schema is
%   the one mgs.zarchan.benchSuite's shared benchRows() consumes.
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
L{end+1} = 'Zarchan Listing 11.1 -- GRAVITY FIELD SIMULATION (flat Earth vs Newton)';
L{end+1} = '*** UNITS ARE FEET AND NAUTICAL MILES (6076 ft).  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['   GAM = %g deg   ALTNM = %g n.mi.   ANG = %g deg   ' ...
    'H = %g s   record every %g s'], p.GAM, p.ALTNM, p.ANGDEG, p.H, p.samp);
L{end+1} = '';
L{end+1} = sprintf('%9s %8s %11s %11s %11s %11s %9s %9s', ...
    'V [ft/s]', 'tF [s]', 'range', 'apogee', 'flat rng', 'flat apo', 'lambda', 'ecc');
for k = 1:numel(R.run)
    C = R.run(k);
    L{end+1} = sprintf('%9.0f %8.2f %11.3f %11.3f %11.3f %11.3f %9.5f %9.5f', ...
        C.V, C.tf_s, C.range_nm, C.apogee_nm, C.flat_range_nm, ...
        C.flat_apogee_nm, C.lam, C.ecc); %#ok<AGROW>
    if ~C.completed
        L{end+1} = sprintf(['        !! V = %g ft/s did not reach the ground ' ...
            'before tmax = %g s; its rows are unusable'], C.V, p.tmax); %#ok<AGROW>
    end
end
L{end+1} = '   range/apogee are the Earth-centred (correct) channel; "flat" is the';
L{end+1} = '   constant-g model flown from the same launch in the same loop.';
L{end+1} = '   range = interpolated altitude zero crossing, NOT the last plotted point.';
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-62s %s', tick(K.oks(i)), K.names{i}, ...
        K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning';
    L{end+1} = '            NOT CHECKED, which is not the same thing as "fine".';
else
    L{end+1} = 'BENCHMARKS vs the book';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        if b.skipped
            L{end+1} = sprintf('   %-4s %-50s book %-14s %s', ...
                upper(b.kind), b.what, b.book_str, b.why); %#ok<AGROW>
        elseif strcmp(b.kind, 'value')
            L{end+1} = sprintf('   %-4s %-50s book %-14s ours %-10s %+7.2f%% (band %.3g)', ...
                tick(b.ok), b.what, b.book_str, b.ours_str, b.pct, b.tol); %#ok<AGROW>
        else
            L{end+1} = sprintf('   %-4s %-50s book %-14s ours %s', ...
                tick(b.ok), b.what, b.book_str, b.ours_str); %#ok<AGROW>
        end
    end
    L{end+1} = sprintf('   %d/%d scored rows pass; %d/%d structural.', ...
        sum([R.bench(ev).ok]), sum(ev), sum(K.oks), numel(K.oks));
end
s = strjoin(L, newline);
end

function s = tick(ok)
if ok, s = 'ok'; else, s = 'FAIL'; end
end

function drawCurves(R)
figure;
hold on
for k = 1:numel(R.run)
    C = R.run(k);
    plot(C.rt1_nm, C.rt2_nm, '--');
    plot(C.dist_nm, C.alt_nm, '-');
end
grid on
xlabel('Downrange (Nmi)');
ylabel('Altitude (Nmi)');
title('Listing 11.1: flat Earth (dashed) vs Earth-centred (solid)');
hold off
end
