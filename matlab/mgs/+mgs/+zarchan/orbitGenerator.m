function R = orbitGenerator(opts)
%MGS.ZARCHAN.ORBITGENERATOR  Zarchan Listing 11.3 -- the MATLAB orbit
%   generator: a two-body Earth-centered point mass launched with zero
%   flight-path angle at a speed parameterised by lambda = r0*V^2/GM.
%
%   R = mgs.zarchan.orbitGenerator()
%   R = mgs.zarchan.orbitGenerator('XLAM', 0.5)               suborbital
%   R = mgs.zarchan.orbitGenerator('XLAM', 2, 'plot', true)   escape
%   R = mgs.zarchan.orbitGenerator('H', 1, 'assert', false)   coarse and fast
%
%   Line-by-line port of Listing 11.3 (zarchan.txt:3459-3469), integrated by
%   the book's own two-pass second-order Runge-Kutta (Heun), unchanged.
%
%   UNITS ARE THE BOOK'S: ft, ft/s, ft/s^2, GM = 1.4077e16 ft^3/s^2,
%   A = 2.0926e7 ft.  Positions PLOT in n.mi. because the listing divides by
%   6076 itself -- the book's own conversion, not 6076.115.  Nothing is
%   converted to SI; convert at the call site and say so there.
%
%   WHAT IT IS NOT.  No seeker, no autopilot, no guidance, no target, no drag,
%   no atmosphere: a point mass in inverse-square gravity.  It is here because
%   Ch.11's lambda family is the book's cleanest statement that TRAJECTORY
%   SHAPE FOLLOWS FROM SPEED ALONE -- the same footing our terminal-speed and
%   dive-authority arguments stand on.  Do not read an orbit, a velocity or a
%   time out of here and apply it to a sortie.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  matlab/zarchan/_listings/listing_11_03.m.  Its header
%   reports one unresolved comparison, two ligature residues, two prose lines:
%
%     1  "while T , TF" -> "while T < TF".  FORCED: T starts at 0 and only
%        increases by H; ">" never enters and "==" never fires on a float.
%     2  "while STEP , =1" -> "while STEP <= 1"; the body sets STEP = 2 to
%        leave.  Same OCR substitution ("," for "<") as everywhere in Ch.11.
%     3  Transcription lines 20-22 are PROSE (the "Theory says ... l 0.5"
%        paragraph, the HIT EQUATION heading, its first paragraph).  Dropped,
%        not "fixed" into code.  That paragraph is zarchan.txt:3473 and it is
%        where BENCHMARKS 4-6 come from, which is how it was identified.
%     4  "save datfil.txt output /ascii" is the book's usual typo for -ascii.
%        Nothing is written to disk here; recorded and dropped.  So is the
%        dead "FLAG=0" after the inner loop, and the never-read SCOUNT.
%
%   TWO BOOK QUIRKS REPRODUCED DELIBERATELY, because they are measurable:
%     * 57.3 for deg->rad and 1.5708 for pi/2 leave the run a residual
%       flight-path angle GAM/57.3 - (1.5708 - pi/2) = -3.6732e-6 rad at
%       GAM = 0.  The listing launches a HAIR DESCENDING (cos(1.5708) < 0),
%       so the lambda = 1 orbit is circular to 3.7e-6 in eccentricity and NOT
%       to machine precision.  BENCHMARK 1's band is sized from that number
%       rather than from the integrator.  See gamActual() -- the sign is the
%       easy thing to get backwards here.
%     * "if S >= 49.99999" is "record every 50 s with a 1e-5 float guard".
%       This port tests "S >= samp - 0.5*H" so a caller's samp cannot be
%       silently halved; at the listing's own 50 s and H = 0.01 the two tests
%       fire on exactly the same steps.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Every book value was grepped out of zarchan_decoded.txt and
%   the line is in .source.  NOTHING was read off a figure -- the extraction
%   turned every figure into an image link.  What IS scoreable about
%   Figs. 11.10-11.13 is the prose describing them, as orderings.
%
%   zarchan.txt:3446 (gamma = 0): circular at lambda = 1, elliptical for
%   0 < lambda < 2, parabolic at 2, hyperbolic above 2.  :3451 confirms 1 and
%   1.5 against the simulation.  :3473 adds that 0.5 gives an ellipse that
%   INTERSECTS THE EARTH and that 2 is escape velocity and does not.  Five
%   members, and the rows follow them:
%
%     1  lambda = 1 circular -- bound, e <= 1e-4 (a correct run: 3.7e-6)
%     2  lambda = 1.5 eccentricity vs :3442, book 0.5.  Launch at PERIGEE.
%     3  lambda = 0.5 eccentricity vs :3442, book 0.5.  Launch at APOGEE.
%        Two different orbits on one published number; hence two rows.
%     4  rmin(0.5) < A <= rmin(1.5)  (:3473 makes exactly one comparison)
%     5  lambda = 2 escapes, e = 1, and clears the Earth  (:3473)
%     6  lambda = 2.5 is hyperbolic: e > 1 and energy > 0  (:3446)
%     7  integrated r(u) vs the closed form :3434, whole lambda = 1.5 orbit
%     INFO  the 1000 n.mi. launch altitude (:3450) -- an INPUT, so scoring it
%           would be scoring an echo of our own arguments block.
%
%   Rows 1 and 4-6 SKIP at GAM ~= 0 (:3446 says "if we set the flight-path
%   angle g to zero", and "clears the Earth" is untrue for a diving launch).
%   Rows 2, 3 and 7 carry gamma explicitly and score at any launch angle.
%
%   THREE SCORED EQUATIONS LOST THEIR MINUS SIGNS to the extraction (this
%   book's third damage class).  They are RESTORED BY ALGEBRA, not guessed:
%
%     :3442  e = [1 + lambda*(lambda - 2)*cos^2 g]^0.5
%     :3434  r = r0*lambda*cos^2 g
%                / [1 - lambda*sin(u)*cos g*sin g - cos(u)*(1 - lambda*cos^2 g)]
%     :3439-3440  e*sin(v) = lambda*cos g*sin g,  e*cos(v) = 1 - lambda*cos^2 g
%
%   Squaring and adding :3439/:3440 gives :3442 EXACTLY with the minus inside
%   (lambda - 2); a plus there would keep e above 1 for every lambda and the
%   circle at lambda = 1 could not exist.  Substituting the same pair into the
%   polar ellipse at :3436 reproduces :3434 term for term, and its numerator
%   a1*(1 - e^2) equals r0*lambda*cos^2 g only with those signs.  Both also
%   satisfy r(u = 0) = r0, which no other sign variant does.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED -- this file has never
%   been executed; MATLAB belongs to the caller. ***  At the defaults,
%   r0 = A + 1000*6076 = 2.7002e7 ft, so
%
%     V = sqrt(GM/r0) = 22833 ft/s;  circular period 7431 s, so the listing's
%     TF = 30000 s is four orbits;  r0 = 4444 n.mi. (the plotted radius);
%     e of the default run 3.67e-6, NOT 0;  lambda = 0.5 sweeps r0 down to
%     r0/3 = 9.00e6 ft, well inside A = 2.09e7 ft, which IS Fig. 11.12;
%     lambda = 1.5 reaches 3*r0.  Both give e = 0.5.
%
%   A residual near a band EDGE is not a pass to accept -- the mechanisms
%   above say where each should land.  Do not widen a band to make a row green.
%
%   COST.  The default main run is TF/H = 3,000,000 Heun steps, because 30000 s
%   and 0.01 s are the listing's own.  In a suite, 'H', 1 cuts that a
%   hundredfold and is SAFE: not one scored row reads the main run (1-7 all
%   come from the lambda family at case_H).  It feeds only structural checks 1
%   and 2, which a coarser H makes HARDER -- so the shortcut cannot go green.
%
%   OUTPUTS
%     R.t R.xnm R.ynm R.r_nm  the listing's own recorded arrays [s, n.mi.]
%     R.x_ft R.y_ft R.theta   same samples in ft, plus central angle u [rad]
%     R.k                     main-run scalars (V, rmin, rmax, ecc, drifts)
%     R.cases                 the five-member lambda family, one struct each
%     R.bench R.selfcheck R.pass R.verdict R.p R.summary R.units
%
%   See also MGS.ZARCHAN.TRAJECTORYSIM, MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 11.3 line 1, verbatim.  GAM is in DEGREES despite the name --
    % the listing writes GAM/57.3 wherever it uses it.
    opts.XLAM   (1,1) double {mustBeFinite, mustBePositive}    = 1.0
    opts.ALTNM  (1,1) double {mustBeFinite, mustBeNonnegative} = 1000.0  % n.mi.
    opts.GAM    (1,1) double {mustBeFinite}                    = 0.0     % deg
    opts.ANGDEG (1,1) double {mustBeFinite}                    = 90.0    % deg
    opts.A      (1,1) double {mustBeFinite, mustBePositive}    = 2.0926e7  % ft
    opts.GM     (1,1) double {mustBeFinite, mustBePositive}    = 1.4077e16
    opts.H      (1,1) double {mustBeFinite, mustBePositive}    = 0.01    % s
    opts.TF     (1,1) double {mustBeFinite, mustBePositive}    = 30000.0 % s
    opts.samp   (1,1) double {mustBeFinite, mustBePositive}    = 50.0    % s
    % The lambda family runs at its OWN step, not at H = 0.01.  Justification
    % is not "it is faster": at H = 0.5 s against a 7431 s circular period the
    % per-step phase is 4.2e-4 rad, so Heun's amplitude growth is (wH)^4/4 =
    % 8e-15 per step.  That claim is not taken on trust -- structural check 4
    % reruns one case at case_H/4 and requires the eccentricity not to move.
    opts.case_H   (1,1) double {mustBeFinite, mustBePositive} = 0.5   % s
    opts.case_pts (1,1) double {mustBeFinite, mustBePositive} = 2000  % samples
    % THE ONLY SWITCHES.  Not nargin: with an arguments block declaring only
    % name-value options MATLAB reports nargin = 0 however many were passed.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

if opts.H >= opts.samp
    error('mgs:zarchan:orbitGenerator:samp', ...
        ['H = %g s is not smaller than samp = %g s, so the returned curve ' ...
         'would be sampled at the step rate while claiming the coarser ' ...
         'interval in its own time vector.'], opts.H, opts.samp);
end
if opts.TF <= 2*opts.samp
    error('mgs:zarchan:orbitGenerator:short', ...
        'TF = %g s records fewer than two samples at samp = %g s.', ...
        opts.TF, opts.samp);
end

NMI = 6076;                     % the LISTING's conversion.  Not 6076.115.
R0  = opts.A + opts.ALTNM*NMI;

% =========================================================================
%   LISTING 11.3, at the caller's parameters.  zarchan.txt:3459-3469.
% =========================================================================
O = runOrbit(opts.XLAM, opts.GAM, opts.ANGDEG, opts.A, opts.ALTNM, ...
             opts.GM, opts.H, opts.TF, opts.samp, +1);

% =========================================================================
%   THE LAMBDA FAMILY.  FIXED, and deliberately not a caller option: every
%   member is named by a specific sentence (zarchan.txt:3446, :3451, :3473),
%   so a caller who changed the list would silently unhook the benchmark rows
%   from the prose that justifies them.
% =========================================================================
lamCases = [0.5 1.0 1.5 2.0 2.5];
C = repmat(emptyCase(), 1, numel(lamCases));
for k = 1:numel(lamCases)
    C(k) = oneCase(lamCases(k), opts, R0);
end

R = struct();
R.t     = O.t;
R.x_ft  = O.x;      R.y_ft  = O.y;
R.xnm   = O.x/NMI;  R.ynm   = O.y/NMI;     % the listing's XNM, YNM
R.r_nm  = hypot(O.x, O.y)/NMI;
R.theta = O.th;
R.k     = O.k;
R.cases = C;
R.units = 'ft, ft/s, ft/s^2; GM = 1.4077e16 ft^3/s^2; A = 2.0926e7 ft; plots in n.mi. at the listing''s own 6076 ft/n.mi.';
R.p     = struct('XLAM',opts.XLAM, 'ALTNM',opts.ALTNM, 'GAM',opts.GAM, ...
                 'ANGDEG',opts.ANGDEG, 'A',opts.A, 'GM',opts.GM, ...
                 'H',opts.H, 'TF',opts.TF, 'samp',opts.samp, 'R0',R0, ...
                 'lam_cases',lamCases, 'case_H',opts.case_H, ...
                 'case_pts',opts.case_pts, 'units',R.units);

R.selfcheck = structuralChecks(O, C, opts, R0);

if opts.benchmark
    R.bench = benchmarks(C, opts, R0);
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
if opts.plot,   drawOrbit(R); end

msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated.  The Ch.11 shape ' ...
        'claims are stated for flight-path angle ZERO (zarchan.txt:3446); ' ...
        'this run used GAM = %g deg.'], opts.GAM);
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
        error('mgs:zarchan:orbitGenerator:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN, complete.  benchSuite runs every port with assert
        % off precisely so one red row cannot hide the board.
        warning('mgs:zarchan:orbitGenerator:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function O = runOrbit(XLAM, GAMdeg, ANGDEG, A, ALTNM, GM, H, TFrun, samp, gsign)
%RUNORBIT  Listing 11.3, zarchan.txt:3459-3469.
%
%   Every constant arrives as an ARGUMENT.  MATLAB local functions cannot see
%   the main function's workspace, and a previous wave in this package lost a
%   run to exactly that.
%
%   gsign is +1 for the listing.  gsign = -1 REPELS, and exists only so that
%   structural check 5 can run a delete experiment on the gravity sign and
%   prove the circularity row would go red.  It is not a physical option.

NMI = 6076;
ALT = ALTNM*NMI;
R0  = A + ALT;
V   = sqrt(GM*XLAM/R0);              % the listing: V = sqrt(GM*XLAM/(A+ALT))
ANG = ANGDEG/57.3;
VRX = V*cos(1.5708 - GAMdeg/57.3 + ANG);
VRY = V*sin(1.5708 - GAMdeg/57.3 + ANG);
X  = R0*cos(ANG);   Y  = R0*sin(ANG);
X1 = VRX;           Y1 = VRY;
T  = 0.0;           S  = 0.0;

% Derivatives initialised before the loop so the FLAG==1 branch cannot read a
% value left over from anything else.
XD = 0.0;  YD = 0.0;  X1D = 0.0;  Y1D = 0.0;

% The recording test below fires when S reaches samp - 0.5*H, so the realised
% interval can be SHORTER than samp when samp is not a whole number of steps.
% Sizing on samp alone silently overruns and MATLAB then grows the arrays one
% element at a time -- correct, and about a hundred times slower.
nmax = ceil(TFrun/max(samp - 0.5*H, H)) + 8;
tA = zeros(nmax,1);  xA = zeros(nmax,1);  yA = zeros(nmax,1);  thA = zeros(nmax,1);
n = 1;  tA(1) = 0;  xA(1) = X;  yA(1) = Y;  thA(1) = 0;

% Running invariants are taken at EVERY step, not at the sample points.  The
% lambda = 0.5 perigee is swept in seconds; sampling it at 50 s would report
% rmin ~0.3% high and quietly bias the eccentricity rows.
r = sqrt(X*X + Y*Y);
rmin = r;  rmax = r;  nDown = 0;
epsN = 0.5*(X1*X1 + Y1*Y1) - GM/r;     epsMin = epsN;  epsMax = epsN;
pN   = X*Y1 - Y*X1;                    pMin   = pN;    pMax   = pN;
thPrev = atan2(Y, X);  thAcc = 0.0;
nSteps = 0;

while T < TFrun
    XOLD = X;  YOLD = Y;  X1OLD = X1;  Y1OLD = Y1;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor.  Pass 1 evaluates at the current state;
    % pass 2 takes a full Euler step, advances T by H, and re-evaluates there.
    % T advances INSIDE this loop, exactly once -- move that line out and the
    % scheme degrades to first order while still running perfectly smoothly.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            X  = X  + H*XD;    Y  = Y  + H*YD;
            X1 = X1 + H*X1D;   Y1 = Y1 + H*Y1D;
            T  = T  + H;
        end
        TEMBOT = (X^2 + Y^2)^1.5;
        X1D = -gsign*GM*X/TEMBOT;
        Y1D = -gsign*GM*Y/TEMBOT;
        XD  = X1;   YD = Y1;
        FLAG = 1;
    end
    % The listing writes "FLAG = 0" here too.  Dropped: the outer loop
    % re-initialises FLAG on its next pass, so the assignment cannot be read.

    X  = (XOLD  + X )/2 + 0.5*H*XD;
    Y  = (YOLD  + Y )/2 + 0.5*H*YD;
    X1 = (X1OLD + X1)/2 + 0.5*H*X1D;
    Y1 = (Y1OLD + Y1)/2 + 0.5*H*Y1D;
    nSteps = nSteps + 1;

    rNew = sqrt(X*X + Y*Y);
    if rNew < rmin, rmin = rNew; end
    if rNew > rmax, rmax = rNew; end
    if rNew < r,    nDown = nDown + 1; end
    r = rNew;
    epsN = 0.5*(X1*X1 + Y1*Y1) - GM/r;
    if epsN < epsMin, epsMin = epsN; end
    if epsN > epsMax, epsMax = epsN; end
    pN = X*Y1 - Y*X1;
    if pN < pMin, pMin = pN; end
    if pN > pMax, pMax = pN; end

    % Central angle u, accumulated.  Not unwrap(): the increment is wrapped
    % into (-pi, pi] here so the accumulation cannot depend on how densely the
    % SAMPLE grid happens to fall near the branch cut.
    thNow = atan2(Y, X);
    dth   = thNow - thPrev;
    dth   = dth - 2*pi*floor((dth + pi)/(2*pi));
    thAcc = thAcc + dth;
    thPrev = thNow;

    S = S + H;
    if S >= samp - 0.5*H          % the listing: "if S >= 49.99999", samp = 50
        S = 0.0;
        n = n + 1;
        tA(n) = T;  xA(n) = X;  yA(n) = Y;  thA(n) = thAcc;
    end
end

O = struct();
O.t = tA(1:n);  O.x = xA(1:n);  O.y = yA(1:n);  O.th = thA(1:n);

% Eccentricity from the FINAL integrated state, via the Laplace-Runge-Lenz
% vector.  It is a conserved quantity of the exact flow, so reading it at the
% end tests the whole integration, not just the initial conditions.
v2 = X1*X1 + Y1*Y1;
rv = X*X1 + Y*Y1;
ex = ((v2 - GM/r)*X - rv*X1)/GM;
ey = ((v2 - GM/r)*Y - rv*Y1)/GM;

K = struct();
K.R0 = R0;  K.V = V;  K.ANG = ANG;
% The flight-path angle the run ACTUALLY flies, in the BOOK'S sign convention
% (positive = climbing = rdot > 0), radians.  The listing's velocity heading is
% 1.5708 - GAM/57.3 + ANG and the local horizontal is ANG + pi/2, so
%     gamma = (ANG + pi/2) - heading = GAM/57.3 - (1.5708 - pi/2).
% Sign matters and is easy to get backwards: at GAM = 0 this is -3.6732e-6 rad,
% i.e. the listing launches a hair DESCENDING, because cos(1.5708) < 0.
K.gam_eff = gamActual(GAMdeg);
K.rmin = rmin;  K.rmax = rmax;  K.r_end = r;
K.n_down = nDown;  K.n_steps = nSteps;
K.ecc      = hypot(ex, ey);
K.ecc_apsis = (rmax - rmin)/(rmax + rmin);
K.eps_end  = epsN;
K.eps_norm = epsN/(GM/R0);
K.eps_drift = (epsMax - epsMin)/(GM/R0);
K.p_drift   = (pMax - pMin)/max(abs(pMin), realmin);
O.k = K;
end


% =============================================================================
function C = emptyCase()
% Built field by field, in this order.  repmat/element-assignment in oneCase's
% caller requires the field NAMES AND ORDER to match exactly, so both ends of
% that assignment come from this one function and cannot drift apart.
C.lam = NaN;  C.tf = NaN;  C.H = NaN;  C.k = struct('ecc', NaN);
C.r_cf_resid = NaN;
end

% =============================================================================
function C = oneCase(lam, opts, R0)
%ONECASE  One member of the lambda family, integrated over its own natural
%   span: a full orbital period when bound, two circular periods when not.
%
%   The period comes from vis-viva, 1/a1 = (2 - lambda)/r0, which is exactly
%   the book's own lambda definition rearranged -- NOT a fitted number.
Tcirc = 2*pi*sqrt(R0^3/opts.GM);
if lam < 1.999
    a1 = R0/(2 - lam);
    tf = 2*pi*sqrt(a1^3/opts.GM);
else
    tf = 2*Tcirc;
end
% Clamped at the step: a sample interval below H would claim a resolution the
% integration does not have, and would then be recorded at the step rate.
samp = max(tf/opts.case_pts, opts.case_H);
O = runOrbit(lam, opts.GAM, opts.ANGDEG, opts.A, opts.ALTNM, opts.GM, ...
             opts.case_H, tf, samp, +1);

C = emptyCase();
C.lam = lam;  C.tf = tf;  C.H = opts.case_H;  C.k = O.k;

% Integrated r(u) against the closed form, zarchan.txt:3434.  Only meaningful
% where the denominator stays away from zero, i.e. for the bound cases; on an
% escape trajectory u approaches an asymptote and the form diverges.
if lam < 1.999
    rSim = hypot(O.x, O.y);
    rCF  = rClosedForm(O.th, R0, lam, gamActual(opts.GAM));
    C.r_cf_resid = max(abs(rSim - rCF))/R0;
end
end


% =============================================================================
function r = rClosedForm(u, r0, lam, g)
%RCLOSEDFORM  zarchan.txt:3434, minus signs restored (see the header for the
%   algebra that forces them):
%
%     r = r0*lam*cos^2 g / [1 - lam*sin(u)*cos(g)*sin(g) - cos(u)*(1 - lam*cos^2 g)]
%
%   Shares no code with the integrator.  That is the entire point of it.
cg = cos(g);  sg = sin(g);
den = 1 - lam*sin(u)*cg*sg - cos(u)*(1 - lam*cg*cg);
r   = r0*lam*cg*cg ./ den;
end


% =============================================================================
function e = eBook(lam, g)
%EBOOK  zarchan.txt:3442, minus restored: e = [1 + lam*(lam - 2)*cos^2 g]^0.5.
%   g in RADIANS, and it should be gamActual(GAM), not the nominal GAM/57.3 --
%   see gamActual.  The two differ by 3.67e-6 rad, which moves e by 1e-11 at
%   lambda = 1.5 and by the whole of e at lambda = 1 (where e = |sin g|).
e = sqrt(max(1 + lam*(lam - 2)*cos(g)^2, 0));
end


% =============================================================================
function g = gamActual(GAMdeg)
%GAMACTUAL  The flight-path angle the listing actually flies, radians, in the
%   book's sign convention (positive = climbing).
%
%       gamma = GAM/57.3 - (1.5708 - pi/2)
%
%   The correction is -3.6732e-6 rad and it is NOT noise to be tidied away: it
%   is the entire eccentricity of the "circular" lambda = 1 orbit, and it is
%   what BENCHMARK 1's band is sized from.  Using the nominal GAM there would
%   predict e = 0 and make a correct run look like a 3.7e-6 mystery.
g = GAMdeg/57.3 - (1.5708 - pi/2);
end


% =============================================================================
function K = structuralChecks(O, C, opts, R0)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.
%
%   *** .names AND .oks ARE THE ONLY THING benchSuite CAN SEE. ***  A check
%   computed into a field with no name/ok pair is a check nobody runs.
K = struct();
names = {};  oks = [];  detail = {};

% 1  Specific energy is conserved by the exact flow.  Predicted drift for the
%    default run: ~1e-13 relative (roundoff), against a 1e-6 band.  A wiring
%    fault in TEMBOT moves it by order 1.
names{end+1}  = 'specific energy conserved over the main run';
oks(end+1)    = O.k.eps_drift <= 1e-6;
detail{end+1} = sprintf('drift = %.3e of GM/r0 (band 1e-6)', O.k.eps_drift);

% 2  Angular momentum p = x*ydot - y*xdot, same argument.  This is the check
%    that would notice X1D and Y1D swapped, which leaves the energy alone.
names{end+1}  = 'angular momentum conserved over the main run';
oks(end+1)    = O.k.p_drift <= 1e-6;
detail{end+1} = sprintf('drift = %.3e relative (band 1e-6)', O.k.p_drift);

% 3  Two INDEPENDENT eccentricity measurements agree on the bound cases: the
%    Laplace-Runge-Lenz vector at the final state, and (rmax-rmin)/(rmax+rmin)
%    from the swept extremes.  They share no arithmetic.
bnd = [C.lam] < 1.999;
eA = arrayfun(@(c) c.k.ecc,       C(bnd));
eB = arrayfun(@(c) c.k.ecc_apsis, C(bnd));
K.ecc_gap = max(abs(eA - eB));
names{end+1}  = 'LRL eccentricity == apsis eccentricity (bound cases)';
oks(end+1)    = K.ecc_gap <= 1e-3;
detail{end+1} = sprintf('worst gap %.3e over lambda = %s (band 1e-3)', ...
    K.ecc_gap, mat2str([C(bnd).lam]));

% 4  case_H is defensible only if the answer does not depend on it.  Rerun
%    lambda = 1.5 at a quarter of the step and require the eccentricity to
%    stand still.  Without this row, "H = 0.5 s is fine" is an assertion.
j15 = find(abs([C.lam] - 1.5) < 1e-12, 1);
Oref = runOrbit(1.5, opts.GAM, opts.ANGDEG, opts.A, opts.ALTNM, opts.GM, ...
                opts.case_H/4, C(j15).tf, C(j15).tf/50, +1);
K.h_gap = abs(C(j15).k.ecc - Oref.k.ecc);
names{end+1}  = 'lambda = 1.5 eccentricity unchanged at case_H/4';
oks(end+1)    = K.h_gap <= 1e-4;
detail{end+1} = sprintf('|de| = %.3e between H = %g and %g s (band 1e-4)', ...
    K.h_gap, opts.case_H, opts.case_H/4);

% 5  *** DELETE EXPERIMENT. ***  Flip the sign of the gravity term and require
%    BENCHMARK 1 to go RED.  A guard that is never seen failing is a guard
%    nobody has tested; this one is run on every call, on the production
%    configuration, and its result is on the board.
%    GAM is pinned to 0 here, NOT taken from opts: this probes the gravity
%    term, and at a caller's non-zero launch angle the CONTROL run would be
%    elliptical by construction and this structural row would go red on a
%    perfectly correct port.  A structural check that can false-red is worse
%    than no check, because the next person deletes it.
span = 0.1*2*pi*sqrt(R0^3/opts.GM);
Ok   = runOrbit(1.0, 0.0, opts.ANGDEG, opts.A, opts.ALTNM, opts.GM, ...
                opts.case_H, span, span/50, +1);
Obad = runOrbit(1.0, 0.0, opts.ANGDEG, opts.A, opts.ALTNM, opts.GM, ...
                opts.case_H, span, span/50, -1);
K.del_ok  = Ok.k.ecc;
K.del_bad = Obad.k.ecc;
names{end+1}  = 'DELETE EXPT: gravity sign flipped => circularity row RED';
oks(end+1)    = (K.del_bad > 1e-4) && (K.del_ok <= 1e-4);
detail{end+1} = sprintf('control e = %.3e (must pass 1e-4), repulsive e = %.3e (must fail)', ...
    K.del_ok, K.del_bad);

K.names    = names;
K.oks      = logical(oks);
K.detail   = detail;
K.all_ok   = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    j = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{j}, detail{j});
end
end


% =============================================================================
function B = benchmarks(C, opts, R0)
%BENCHMARKS  Computed vs published, with the PROVENANCE of every band.
%
%   Bands:
%     'shape'     an ordering out of the book's prose; no tolerance possible
%     'bound'     a one-sided limit whose size is derived in the header
%     'analytic'  0.5% against a closed FORM (rows 2 and 3)
%
%   Do NOT widen a band to turn a row green.
B = emptyBench();
g0     = abs(opts.GAM) < 1e-9;
whyGam = sprintf(['zarchan.txt:3446 states the lambda family "if we set the ' ...
    'flight-path angle g to zero"; this run used GAM = %g deg, at which ' ...
    '"does not intersect the Earth" is simply not the book''s claim'], opts.GAM);
c05 = C(abs([C.lam] - 0.5) < 1e-12);
c10 = C(abs([C.lam] - 1.0) < 1e-12);
c15 = C(abs([C.lam] - 1.5) < 1e-12);
c20 = C(abs([C.lam] - 2.0) < 1e-12);
c25 = C(abs([C.lam] - 2.5) < 1e-12);

% ---- 1.  lambda = 1 is circular -------------------------------------------
if g0
    B = addBound(B, 'lambda = 1 orbit is CIRCULAR', '<= 1e-4 eccentricity', ...
        sprintf('%.3e', c10.k.ecc), c10.k.ecc <= 1e-4, ...
        'p.245-246 text (zarchan.txt:3451, family at :3446)', '-', ...
        sprintf(['band is NOT machine zero: the listing''s 1.5708 and 57.3 ' ...
                 'leave gamma = %.3e rad, so a correct run lands near %.2e'], ...
                 c10.k.gam_eff, abs(sin(c10.k.gam_eff))));
else
    B = addSkip(B, 'lambda = 1 orbit is CIRCULAR', 'e = 0', whyGam, ...
        'p.245-246 text (zarchan.txt:3451, family at :3446)', '-');
end

% ---- 2/3.  eccentricity against the printed formula ------------------------
% Two orbits, one published number.  lambda = 1.5 starts at PERIGEE and
% lambda = 0.5 at APOGEE, so a fault that mixes up which apsis the launch
% point is cannot pass both.
B = addVal(B, 'ecc at lambda = 1.5 vs [1+l(l-2)cos^2 g]^0.5', ...
    eBook(1.5, gamActual(opts.GAM)), c15.k.ecc, 'analytic', 0.005, ...
    'p.244 eq. (zarchan.txt:3442; signs forced by :3439-3440)', '-', ...
    'launch is at PERIGEE here (e*cos v = 1 - lambda < 0); expected residual ~1e-6');
B = addVal(B, 'ecc at lambda = 0.5 vs [1+l(l-2)cos^2 g]^0.5', ...
    eBook(0.5, gamActual(opts.GAM)), c05.k.ecc, 'analytic', 0.005, ...
    'p.244 eq. (zarchan.txt:3442) + p.248 text (:3473)', '-', ...
    'launch is at APOGEE here; same published 0.5 from a different orbit');

% ---- 4.  suborbital vs orbital --------------------------------------------
if g0
    ok4 = (c05.k.rmin < opts.A) && (c15.k.rmin >= opts.A);
    B = addShape(B, 'lambda 0.5 hits the Earth, lambda 1.5 does not', ...
        'rmin(0.5) < A <= rmin(1.5)', ...
        sprintf('rmin = %.4g and %.4g ft, A = %.4g ft', ...
            c05.k.rmin, c15.k.rmin, opts.A), ok4, ...
        'p.248 text (zarchan.txt:3473)', ...
        ['the sentence makes exactly one comparison, so this is one row.  ' ...
         'Fig. 11.12 is the picture of the left half']);
else
    B = addSkip(B, 'lambda 0.5 hits the Earth, lambda 1.5 does not', ...
        'rmin(0.5) < A <= rmin(1.5)', whyGam, ...
        'p.248 text (zarchan.txt:3473)', 'ft');
end

% ---- 5.  lambda = 2 is escape, parabolic, and clears the Earth ------------
if g0
    ok5 = (abs(c20.k.ecc - 1) <= 1e-4) && (c20.k.rmin >= opts.A) && ...
          (c20.k.r_end > 3*R0);
    B = addShape(B, 'lambda = 2 is ESCAPE, parabolic, clears the Earth', ...
        'e = 1, rmin >= A, r leaves', ...
        sprintf('e = %.6f, rmin/A = %.4f, r_end/r0 = %.3g, energy/(GM/r0) = %+.2e', ...
            c20.k.ecc, c20.k.rmin/opts.A, c20.k.r_end/R0, c20.k.eps_norm), ...
        ok5, 'p.248 text (zarchan.txt:3473)', ...
        ['e = 1 EXACTLY here at any gamma, since the (lambda - 2) factor in ' ...
         ':3442 vanishes -- which is why 1e-4 is a real limit and not a guess']);
else
    B = addSkip(B, 'lambda = 2 is ESCAPE, parabolic, clears the Earth', ...
        'e = 1, rmin >= A', whyGam, 'p.248 text (zarchan.txt:3473)', '-');
end

% ---- 6.  lambda > 2 is hyperbolic -----------------------------------------
if g0
    ok6 = (c25.k.ecc > 1) && (c25.k.eps_norm > 0) && (c25.k.r_end > 3*R0);
    B = addShape(B, 'lambda = 2.5 is HYPERBOLIC', 'e > 1 and energy > 0', ...
        sprintf('e = %.5f, energy/(GM/r0) = %+.5f (formula: %.5f)', ...
            c25.k.ecc, c25.k.eps_norm, eBook(2.5, gamActual(opts.GAM))), ok6, ...
        'p.245 text (zarchan.txt:3446)', ...
        'the fourth clause of the family sentence; energy sign is the sharp part');
else
    B = addSkip(B, 'lambda = 2.5 is HYPERBOLIC', 'e > 1', whyGam, ...
        'p.245 text (zarchan.txt:3446)', '-');
end

% ---- 7.  the whole orbit against the closed-form trajectory equation ------
B = addBound(B, 'r(u) vs closed form, whole lambda = 1.5 orbit', ...
    '<= 0.1% of r0', sprintf('%.4f%% of r0', 100*c15.r_cf_resid), ...
    c15.r_cf_resid <= 0.001, ...
    'p.244 eq. (zarchan.txt:3434; signs forced by :3436 and r(0) = r0)', 'ft', ...
    ['shares no code with the integrator; expected residual is the Heun ' ...
     'error at case_H, order 1e-5 of r0.  It tests the orbit''s GEOMETRY and ' ...
     'not its timing -- u is taken from the simulation itself, so a phase ' ...
     'error cannot show up here.  Structural checks 1-2 are what cover that']);

% ---- INFO.  The launch altitude ------------------------------------------
B = addInfo(B, 'initial altitude', '1000 n.mi.', ...
    sprintf('%.4g n.mi., r0 = %.5g ft = %.5g n.mi.', ...
        opts.ALTNM, R0, R0/6076), ...
    'p.245 text (zarchan.txt:3450)', ...
    ['this is an INPUT to the listing, so scoring it would be scoring an ' ...
     'echo of our own arguments block']);
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
function s = render(R, ev)
L = {};
p = R.p;
L{end+1} = 'Zarchan Listing 11.3 -- MATLAB ORBIT GENERATOR';
L{end+1} = '*** UNITS ARE FEET AND ft/s.  Plots are n.mi. at the book''s 6076 ft. ***';
L{end+1} = sprintf(['   lambda = %g   alt = %g n.mi.   GAM = %g deg   ANG = %g deg   ' ...
    'H = %g s   TF = %g s   samp = %g s'], p.XLAM, p.ALTNM, p.GAM, p.ANGDEG, ...
    p.H, p.TF, p.samp);
L{end+1} = sprintf(['   r0 = %.5g ft (%.5g n.mi.)   V = %.5g ft/s   ' ...
    'residual gamma = %.3e rad (the 1.5708 quirk)'], ...
    p.R0, p.R0/6076, R.k.V, R.k.gam_eff);
L{end+1} = sprintf(['   main run: %d steps, %d samples, rmin/rmax = %.5g / %.5g ft, ' ...
    'e = %.5g'], R.k.n_steps, numel(R.t), R.k.rmin, R.k.rmax, R.k.ecc);
L{end+1} = '';

L{end+1} = 'THE LAMBDA FAMILY  (fixed; each member is named by a sentence in Ch.11)';
L{end+1} = sprintf('%8s %10s %8s %12s %12s %10s %12s', ...
    'lambda', 'span [s]', 'ecc', 'rmin [ft]', 'rmax [ft]', 'rmin/A', 'E/(GM/r0)');
for k = 1:numel(R.cases)
    c = R.cases(k);
    L{end+1} = sprintf('%8.2f %10.1f %8.5f %12.5g %12.5g %10.4f %+12.4e', ...
        c.lam, c.tf, c.k.ecc, c.k.rmin, c.k.rmax, c.k.rmin/p.A, c.k.eps_norm); %#ok<AGROW>
end
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-52s %s', vstr(K.oks(i)), K.names{i}, K.detail{i}); %#ok<AGROW>
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
            case 'info'
                L{end+1} = sprintf('   INFO %-52s %s', b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why);                    %#ok<AGROW>
            case 'skip'
                L{end+1} = sprintf('   SKIP %-52s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why);                    %#ok<AGROW>
            case {'shape','bound'}
                L{end+1} = sprintf('   %-4s %-52s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str);         %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf('   %-4s %-52s book %s %s / ours %s %s (%+.4f%%, band %.4g%%)', ...
                    vstr(b.ok), b.what, b.book_str, b.units, b.ours_str, ...
                    b.units, b.pct, 100*b.tol);                             %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped/info -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: no drag, no atmosphere, no thrust, no guidance.  Ch.11 is';
L{end+1} = '   two-body Kepler and nothing else.  Its one transferable message is';
L{end+1} = '   that trajectory SHAPE follows from speed alone -- lambda is r0*V^2/GM.';
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
function drawOrbit(R)
%DRAWORBIT  Base MATLAB graphics only.  The listing plots XNM against YNM with
%   no Earth on it; the Earth circle is added here because Figs. 11.10-11.13
%   are unreadable without it -- "the orbit intersects the Earth" is not a
%   statement you can check against an unmarked pair of axes.
figure;
hold on; grid on; axis equal;
th = linspace(0, 2*pi, 361);
plot(R.p.A*cos(th)/6076, R.p.A*sin(th)/6076, '--');
plot(R.xnm, R.ynm, '-');
plot(R.xnm(1), R.ynm(1), 'o');
xlabel('X (Nmi)');
ylabel('Y (Nmi)');
title(sprintf('Listing 11.3 orbit generator, lambda = %g (e = %.4g)', ...
    R.p.XLAM, R.k.ecc));
legend('Earth surface', 'trajectory', 'launch point', 'Location', 'best');
hold off;
end
