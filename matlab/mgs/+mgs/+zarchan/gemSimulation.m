function R = gemSimulation(opts)
%MGS.ZARCHAN.GEMSIMULATION  Zarchan Listing 13.4 -- GEM booster steering.
%
%   R = mgs.zarchan.gemSimulation()
%   R = mgs.zarchan.gemSimulation('companion', false)      % skip Listing 13.3
%   R = mgs.zarchan.gemSimulation('H', 0.02, 'assert', false)
%
%   Line-by-line port of Listing 13.4 (zarchan.txt:4091-4118): a two-stage
%   booster leaves the ground at longitude 30 deg and must hit a ground target
%   at 45 deg exactly 500 s later.  Under GENERAL ENERGY MANAGEMENT all the
%   fuel must be burned, so the thrust vector is held at an angle THET off the
%   velocity-to-be-gained vector and the excess energy is thrown away sideways:
%
%        THET = sqrt(6*(1 - DEL/DVCAP)),   DVCAP = DVCAP - H*aT
%
%   TWO OTHER LISTINGS ARE IN THIS FILE because 13.4 is not runnable or
%   checkable without them:
%     * Listing 13.2's Lambert routine (zarchan.txt:3943-3955) -- 13.4 calls it
%       every integration step and the chapter says so explicitly (:4071).
%     * Listing 13.3, the SAME booster under plain Lambert steering
%       (zarchan.txt:3987-4008).  Figs. 13.12/13.13 are comparisons between the
%       two trajectories, so the two comparison rows below need both.  It is
%       the same integrator with one steering branch swapped; 'companion'
%       false drops it and those rows SKIP rather than pass.
%
% =========================================================================
%   UNITS ARE THE BOOK'S: ft, ft/s, ft/s^2, s, deg.  g = 32.2 ft/s^2,
%   1 nmi = 6076 ft, Earth radius A = 2.0926e7 ft, GM = 1.4077e16 ft^3/s^2.
%   Angles convert with the book's 57.3, NOT 180/pi.  That is a 0.007% offset
%   and it is part of the listing, so it is kept rather than "corrected".
% =========================================================================
%
%   THE TRANSCRIPTION.  matlab/zarchan/_listings/listing_13_04.m does not
%   parse.  The OCR maps '<' -> ',' and '>' -> '.' throughout, so most repairs
%   are forced by shape: "while STEP ,=1" -> "<= 1" (the body sets STEP=2 to
%   leave), "if T,TB1" -> "T < TB1" (a burn schedule), "if S.=9.99999" ->
%   ">= " (S is reset on every record).  Two comparisons are variable against
%   variable and had to be resolved by argument, not by shape:
%
%     1  "if (QZERO==0 & DVCAP.DEL)"  ->  DVCAP > DEL.  FORCED: the guarded
%        line is THET = sqrt(6*(1 - DEL/DVCAP)), which is imaginary for
%        DEL > DVCAP.  The else-branch latches QZERO = 1 and stops updating
%        THET, which is exactly the "we no longer have surplus energy" case.
%     2  "if XLONGT.XLONGM"  ->  XLONGT > XLONGM.  Resolved from the prose at
%        zarchan.txt:4056: for COUNTERCLOCKWISE travel aXT = aT*cos(phi - u).
%        The booster goes 30 deg -> 45 deg, i.e. counterclockwise, i.e.
%        XLONGT > XLONGM, and that is the branch that must carry the minus.
%        Listing 13.2 contains the identical line and is resolved the same
%        way.  This one is LOAD-BEARING and is scored by the delete experiment
%        in R.selfcheck: inverting it flies the booster into the ground inside
%        ten seconds, because PHI+THET at t = 0 points 54 deg BELOW the local
%        horizon instead of 22 deg off the local vertical.
%
%   Also repaired, each documented at its own line: acos() arguments clamped
%   to [-1,1] (a rounding-level 1+2e-16 returns a COMPLEX angle and the sim
%   then runs on smoothly with complex state); the Lambert secant guarded
%   against a non-finite step (tf == told gives Inf/NaN, and NaN fails BOTH
%   bracket tests silently, so the listing would spin forever); THET given an
%   initial 0; and every loop given an iteration cap whose exhaustion is
%   REPORTED, never absorbed.  Prose leaking in after the plotting call (the
%   chapter's SUMMARY and REFERENCES) is dropped, and "save datfil.txt output
%   /ascii" writes nothing.
%
%   TWO DEVIATIONS THAT ARE NOT REPAIRS.  (a) The listing records every 10 s,
%   which is 50 points for a 500 s flight and cannot show the boost phase at
%   all; 'samp' defaults to 0.1 s and the book's record is the subset at
%   multiples of 10.  (b) DVCAP/THET/DEL are recorded only while the booster
%   is still guided -- the listing keeps decrementing DVCAP after cutoff,
%   harmless there (cutoff IS burnout) but negative in the Lambert companion.
%
%   ⚠️ THE BOOK CONTRADICTS ITSELF ABOUT THE 50 ft/s CUTOFF.  zarchan.txt:4071
%   says the GEM logic stops when the velocity TO BE GAINED drops below 50;
%   the listing tests DVCAP, the velocity CAPABILITY.  Transcribed as written;
%   the mismatch is an INFO row, not a scored one, because under GEM the two
%   quantities converge on each other by burnout and the nominal case cannot
%   separate them.
%
%   ⚠️ WHAT THE 500 s IMPACT ROW DOES NOT PROVE.  At cutoff the listing SNAPS
%   the velocity to the Lambert solution (X1 = VRX; Y1 = VRY), so impact at TF
%   is guaranteed once the Lambert routine and the coast integration are right
%   -- it says nothing about the GEM steering.  Row 7 (velocity to be gained
%   still standing at burnout) and the delete experiment are what test that.
%
%   OUTPUTS
%     R.gem, R.lambert   per-run structs: .t .x .y .alt_nm .dist_nm .thet_deg
%                        .dvcap .del .t_impact .t_cut .del_at_cut .n_maxit
%                        (.lambert is [] when 'companion' is false)
%     R.sizing           the listing's rocket sizing block, all weights in lb
%     R.delete           the sign-inverted run the delete experiment uses
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
%   ⚠ WHY THIS PORT'S EXISTING "xlam < 2" TEST WAS NOT ENOUGH.  lambertpz here
%   already refuses to evaluate the elliptic closed form unless
%   "xlam > 0 && xlam < 2", falling back to tf = inf.  That is the ESCAPE
%   condition, and the failing solve halts at lambda = 1.999999919, which
%   satisfies it -- so the fallback never fires and the wrong answer is
%   returned normally.  2 is the value the bracket is DEFINED by; 1.999999 is
%   the value that separates right answers from wrong ones.
%
%   SCORED OVER EVERY CALL.  This listing closes Lambert in a guidance loop, so
%   the guard tracks the largest lambda over ALL calls in the GEM run, the
%   Lambert companion run and the delete-experiment run -- not just a final
%   one.  A default run censuses ~3000 calls.
%
%   THIS PORT'S REACH -- THE GUARD BITES, BUT NEVER ALONE.  MEASURED: 56 of 80
%   swept (XLONGTDEG, TF) points drive it red, e.g. XLONGTDEG = 48, TF = 160 ->
%   lambda_max = 2.000000000 over 3003 calls, guard RED.
%   ⚠ HONEST LIMIT.  Every one of those 56 also has n_maxit > 0, so invariant 3
%   ("Lambert routine converged on every call") is red there too.  A refined
%   scan (3 longitudes x 100 flight times at 0.5 s resolution) found NO
%   geometry where the guard is the ONLY red row: the best fully-converged run
%   reaches lambda = 1.997362372, just under the cap.  So on THIS port the
%   guard is verified to fire, but its unique-catch value is NOT demonstrated
%   -- driving the loop into the near-parabolic band also drives at least one
%   call into the iteration cap, which invariant 3 already catches.
%   MARGIN AT THE BOOK'S OWN CASE: lambda_max = 0.262892893.
% ---------------------------------------------------------------------------
%
%     R.lambda_max  the largest lambda = r0|v|^2/gm over every Lambert call in
%                   every run this invocation made
%     R.bench .selfcheck .pass .verdict .p .summary   -- schema shared with the
%                        other ports; benchSuite's adaptGeneric consumes it.
%
%   See also MGS.ZARCHAN.BENCHSUITE, MGS.ZARCHAN.TRAJECTORYSIM.

arguments
    opts.TF        (1,1) double {mustBeFinite, mustBePositive} = 500     % s
    opts.H         (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s
    opts.samp      (1,1) double {mustBeFinite, mustBePositive} = 0.1     % s
    opts.ANGDEG    (1,1) double {mustBeFinite} = 30                      % deg
    opts.XLONGTDEG (1,1) double {mustBeFinite} = 45                      % deg
    opts.DELV      (1,1) double {mustBeFinite, mustBePositive} = 20000   % ft/s
    % Hard wall on the outer loop.  The listing's only exit is "underground
    % and past 10 s"; a booster that never comes down would hang the suite.
    opts.tmax      (1,1) double {mustBeFinite, mustBePositive} = 1500    % s
    opts.maxit     (1,1) double {mustBeFinite, mustBePositive} = 200
    opts.companion (1,1) logical = true
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.assert    (1,1) logical = true
end

if opts.H > opts.samp
    error('mgs:zarchan:gemSimulation:samp', ...
        ['H = %g s exceeds samp = %g s, so the record would be sampled at ' ...
         'the step rate while claiming the coarser interval.'], opts.H, opts.samp);
end
if opts.tmax <= opts.TF
    error('mgs:zarchan:gemSimulation:tmax', ...
        'tmax = %g s must exceed TF = %g s or impact can never be reached.', ...
        opts.tmax, opts.TF);
end

P        = sizeBooster(opts.DELV);
P.A      = 2.0926e7;      P.GM        = 1.4077e16;
P.H      = opts.H;        P.TF        = opts.TF;
P.samp   = opts.samp;     P.ANGDEG    = opts.ANGDEG;
P.DELV   = opts.DELV;     P.XLONGTDEG = opts.XLONGTDEG;
P.maxit  = opts.maxit;

G = flyBooster(P, 'gem', opts.tmax, false);
if opts.companion
    L = flyBooster(P, 'lambert', opts.tmax, false);
else
    L = [];
end
% DELETE EXPERIMENT.  Same run, "if XLONGT > XLONGM" inverted.  Boost phase
% only -- the answer is decided in the first ten seconds and a full 500 s run
% would cost as much as the nominal case to say the same thing.
D = flyBooster(P, 'gem', P.TB1 + P.TB2 + 1, true);

R          = struct();
R.gem      = G;
R.lambert  = L;
R.delete   = D;
R.sizing   = P;
R.p        = struct('TF',opts.TF, 'H',opts.H, 'samp',opts.samp, ...
                    'ANGDEG',opts.ANGDEG, 'XLONGTDEG',opts.XLONGTDEG, ...
                    'DELV',opts.DELV, 'tmax',opts.tmax, 'maxit',opts.maxit, ...
                    'companion',opts.companion, ...
                    'units','ft, ft/s, ft/s^2, s, deg; nmi = 6076 ft; g = 32.2');
R.units    = 'ft, ft/s, ft/s^2, s, deg; nmi = 6076 ft; g = 32.2 ft/s^2';
R.lambda_max = maxLambda({G, L, D});
R.selfcheck  = structuralChecks(P, G, D, L);

if opts.benchmark
    R.bench = benchmarks(P, G, L, opts);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && all(R.selfcheck.oks);
else
    R.bench = emptyBench();
    ev      = false;
    R.pass  = false;      % NOT CHECKED is not the same thing as fine
end

R.summary = render(P, G, L, R, ev);
if opts.render, fprintf('%s\n', R.summary); end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false; nothing was compared against the book.';
elseif ~any(ev)
    msg = 'not one book value could be evaluated at these parameters.';
elseif ~all(R.selfcheck.oks)
    bad = find(~R.selfcheck.oks, 1);
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question. ' ...
        'First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.names{bad});
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
        error('mgs:zarchan:gemSimulation:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:gemSimulation:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function P = sizeBooster(DELV)
%SIZEBOOSTER  Listing 13.4 line 1, verbatim (zarchan.txt:4091).  Weights in lb.
%   Stage 2 is sized first, against the payload; stage 1 is then sized against
%   the whole of stage 2.  Both are the rocket equation solved for propellant
%   weight at a fixed mass fraction, which is what the sizing self-check
%   verifies independently.
P = struct();
P.XISP1 = 300.;  P.XISP2 = 300.;  P.XMF1 = .90;  P.XMF2 = .90;
P.WPAY  = 100.;  P.DELV1 = .3333*DELV;  P.DELV2 = .6667*DELV;
P.AMAX1 = 20.;   P.AMAX2 = 20.;
TOP2   = P.WPAY*(exp(P.DELV2/(P.XISP2*32.2))-1.);
BOT2   = 1/P.XMF2-((1.-P.XMF2)/P.XMF2)*exp(P.DELV2/(P.XISP2*32.2));
P.WP2  = TOP2/BOT2;
P.WS2  = P.WP2*(1-P.XMF2)/P.XMF2;
P.WTOT2 = P.WP2+P.WS2+P.WPAY;
P.TRST2 = P.AMAX2*(P.WPAY+P.WS2);
P.TB2   = P.XISP2*P.WP2/P.TRST2;
TOP1   = P.WTOT2*(exp(P.DELV1/(P.XISP1*32.2))-1.);
BOT1   = 1/P.XMF1-((1.-P.XMF1)/P.XMF1)*exp(P.DELV1/(P.XISP1*32.2));
P.WP1  = TOP1/BOT1;
P.WS1  = P.WP1*(1-P.XMF1)/P.XMF1;
P.WTOT = P.WP1+P.WS1+P.WTOT2;
P.TRST1 = P.AMAX1*(P.WTOT2+P.WS1);
P.TB1   = P.XISP1*P.WP1/P.TRST1;
% Peak axial acceleration in each stage: the burn schedule is linear in T, so
% the peak is at the END of each stage, where the weight is lowest.  Formed
% from the SIZED weights, not by substituting AMAX back in -- that is what
% makes it a check on the transcription rather than a restatement of it.
P.at_max1 = 32.2*P.TRST1/(P.WTOT  - P.WP1);
P.at_max2 = 32.2*P.TRST2/(P.WTOT2 - P.WP2);
end


% =============================================================================
function S = flyBooster(P, mode, tstop, flipdir)
%FLYBOOSTER  The integration routine shared by Listings 13.3 and 13.4.
%
%   mode    'gem'     -> Listing 13.4's GEM branch (zarchan.txt:4105-4116)
%           'lambert' -> Listing 13.3's Lambert branch (zarchan.txt:4001-4006)
%   tstop   hard wall on T; the listing has none
%   flipdir inverts "XLONGT > XLONGM" -- the delete experiment, GEM mode only
%
%   Every constant arrives as a field of P.  MATLAB local functions cannot see
%   the caller's workspace, and this package has already lost a wave to that.

A = P.A;  GM = P.GM;  H = P.H;  TF = P.TF;  samp = P.samp;
TBTOT = P.TB1 + P.TB2;

QBOOST = 1;  QZERO = 0;  T = 0.;  Sacc = 0.;
ANG    = P.ANGDEG/57.3;          XLONGM = ANG;
XLONGT = P.XLONGTDEG/57.3;
ALT = 0.;                                  % ALTNM = 0 -> launch from the ground
X = (A+ALT)*cos(ANG);  Y = (A+ALT)*sin(ANG);
ALT = sqrt(X^2+Y^2)-A;
X1 = 0.;  Y1 = 0.;  AXT = 0.;  AYT = 0.;
XD = 0.;  YD = 0.;  X1D = 0.;  Y1D = 0.;  AT = 0.;  TRST = 0.;
% Listing 13.4's distance datum is the TARGET (XFIRST = XF), unlike 13.3 where
% it is the launch point; DISTNM is then formed as DISTINITNM - (arc to target)
% so that both listings plot downrange-from-launch on the same axis.
XF = A*cos(XLONGT);  YF = A*sin(XLONGT);
XFIRST = XF;  YFIRST = YF;  RF = sqrt(XFIRST^2+YFIRST^2);
DISTINITNM = A*arcAngle(X, Y, XFIRST, YFIRST, RF)/6076.;
DVCAP = P.DELV;
THET = 0.;  DEL = NaN;      % THET is READ BEFORE IT IS ASSIGNED if QZERO
                            % latches on the first guided pass, which in the
                            % listing is an "Undefined function or variable"
                            % error; 0 is the only value that leaves the
                            % thrust on the velocity-to-be-gained vector.

nmax = ceil(tstop/samp) + 8;
S = struct('t',zeros(nmax,1), 'x',zeros(nmax,1), 'y',zeros(nmax,1), ...
           'alt_nm',zeros(nmax,1), 'dist_nm',zeros(nmax,1), ...
           'thet_deg',nan(nmax,1), 'dvcap',nan(nmax,1), 'del',nan(nmax,1));
n = 0;
S.mode = mode;  S.n_maxit = 0;  S.hit_tmax = false;
S.lam_max = 0;  S.lam_calls = 0;   % NEAR-PARABOLIC GUARD, see the file header
S.t_cut = NaN;  S.del_at_cut = NaN;  S.thet0_deg = NaN;
S.alt_min_boost_ft = inf;  S.dlong_min_deg = inf;  S.t_impact = NaN;

while ~(ALT < 0. && T > 10.)
    if T >= tstop, S.hit_tmax = true; break; end
    XOLD = X;  YOLD = Y;  X1OLD = X1;  Y1OLD = Y1;
    STEP = 1;  FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            X = X+H*XD;  Y = Y+H*YD;  X1 = X1+H*X1D;  Y1 = Y1+H*Y1D;
            T = T+H;  STEP = 2;
        end
        if T < P.TB1
            WGT = -P.WP1*T/P.TB1+P.WTOT;                     TRST = P.TRST1;
        elseif T < TBTOT
            WGT = -P.WP2*T/P.TB2+P.WTOT2+P.WP2*P.TB1/P.TB2;  TRST = P.TRST2;
        else
            WGT = P.WPAY;                                    TRST = 0.;
        end
        AT = 32.2*TRST/WGT;  XD = X1;  YD = Y1;  TEMBOT = (X^2+Y^2)^1.5;
        X1D = -GM*X/TEMBOT+AXT;  Y1D = -GM*Y/TEMBOT+AYT;  FLAG = 1;
    end
    X  = (XOLD+X)/2+.5*H*XD;    Y  = (YOLD+Y)/2+.5*H*YD;
    X1 = (X1OLD+X1)/2+.5*H*X1D; Y1 = (Y1OLD+Y1)/2+.5*H*Y1D;
    ALT = sqrt(X^2+Y^2)-A;  TGOLAM = TF-T;
    DVCAP = DVCAP-H*AT;

    if strcmp(mode, 'gem')
        % ---- Listing 13.4, zarchan.txt:4105-4116 --------------------------
        if QBOOST == 1 && DVCAP > 50.
            XLONGM = atan2(Y,X);
            [VRX, VRY, hitcap, lamOut] = lambertpz(X, Y, TGOLAM, XF, YF, XLONGM, XLONGT, P.maxit);
            S.n_maxit = S.n_maxit + hitcap;
            S = noteLambda(S, lamOut);
            DELX = VRX-X1;  DELY = VRY-Y1;  DEL = sqrt(DELX^2+DELY^2);
            if QZERO == 0 && DVCAP > DEL
                THET = sqrt(6.*(1.-DEL/DVCAP));
            else
                QZERO = 1;                 % THET freezes at its last value
            end
            PHI = atan2(DELY,DELX);
            towardTarget = (XLONGT > XLONGM);
            if flipdir, towardTarget = ~towardTarget; end
            if towardTarget
                AXT = AT*cos(PHI-THET);  AYT = AT*sin(PHI-THET);
            else
                AXT = AT*cos(PHI+THET);  AYT = AT*sin(PHI+THET);
            end
            S.del_at_cut = DEL;  S.t_cut = T;
            if ~isfinite(S.thet0_deg), S.thet0_deg = 57.3*THET; end
        elseif QBOOST == 1
            [VRX, VRY, hitcap, lamOut] = lambertpz(X, Y, TGOLAM, XF, YF, XLONGM, XLONGT, P.maxit);
            S.n_maxit = S.n_maxit + hitcap;
            S = noteLambda(S, lamOut);
            QBOOST = 0;  AXT = 0.;  AYT = 0.;  X1 = VRX;  Y1 = VRY;
        else
            QBOOST = 0;  AXT = 0.;  AYT = 0.;
        end
    else
        % ---- Listing 13.3, zarchan.txt:4001-4006 --------------------------
        if QBOOST == 1
            XLONGM = atan2(Y,X);
            [VRX, VRY, hitcap, lamOut] = lambertpz(X, Y, TGOLAM, XF, YF, XLONGM, XLONGT, P.maxit);
            S.n_maxit = S.n_maxit + hitcap;
            S = noteLambda(S, lamOut);
            DELX = VRX-X1;  DELY = VRY-Y1;  DEL = sqrt(DELX^2+DELY^2);
            if TRST > 0. && DEL > 500.
                AXT = AT*DELX/DEL;  AYT = AT*DELY/DEL;
                S.del_at_cut = DEL;  S.t_cut = T;
            elseif DEL < 500.
                QBOOST = 0;  AXT = 0.;  AYT = 0.;  X1 = VRX;  Y1 = VRY;
                S.del_at_cut = DEL;  S.t_cut = T;
            else
                QBOOST = 0;  AXT = 0.;  AYT = 0.;
            end
        end
    end

    % ⚠️ The window starts at 1 s, not 0.  AXT/AYT are still zero when the
    % FIRST derivative pass runs -- the listing computes guidance only after
    % the integration routine -- so step one is unguided free fall and ALT
    % dips about 0.002 ft below the surface before thrust is ever applied.
    % That artefact is present in the book's own code and must not be mistaken
    % for the delete experiment's answer, which is thousands of feet deep.
    if T >= 1. && T <= TBTOT
        S.alt_min_boost_ft = min(S.alt_min_boost_ft, ALT);
    end
    if QBOOST == 1
        S.dlong_min_deg = min(S.dlong_min_deg, 57.3*(atan2(Y,X) - ANG));
    end

    Sacc = Sacc + H;
    if Sacc >= samp - 0.5*H
        Sacc = 0.;  n = n + 1;
        Rm = sqrt(X^2+Y^2);
        S.t(n) = T;  S.x(n) = X;  S.y(n) = Y;
        S.alt_nm(n)  = (Rm-A)/6076.;
        S.dist_nm(n) = DISTINITNM - A*arcAngle(X, Y, XFIRST, YFIRST, RF)/6076.;
        if QBOOST == 1
            S.thet_deg(n) = 57.3*THET;  S.dvcap(n) = DVCAP;  S.del(n) = DEL;
        end
    end
end
if ~S.hit_tmax, S.t_impact = T; end
S.t = S.t(1:n);  S.x = S.x(1:n);  S.y = S.y(1:n);
S.alt_nm = S.alt_nm(1:n);  S.dist_nm = S.dist_nm(1:n);
S.thet_deg = S.thet_deg(1:n);  S.dvcap = S.dvcap(1:n);  S.del = S.del(1:n);
S.dist_init_nm = DISTINITNM;
end


% =============================================================================
function b = arcAngle(x, y, xf, yf, rf)
%ARCANGLE  The listing's CBETA/BETA pair, with the acos argument CLAMPED.
%   At launch the vehicle is on the sphere and CBETA can come back as
%   1 + 2e-16; acos of that is complex, and the simulation then carries on
%   perfectly smoothly with complex state.  The clamp cannot mask a real error
%   -- a genuine |CBETA| > 1 is not reachable from a dot product of two real
%   vectors divided by the product of their own norms.
r = sqrt(x^2+y^2);
c = (x*xf+y*yf)/(r*rf);
b = acos(min(1, max(-1, c)));
end


% =============================================================================
function [vrx, vry, hitcap, lamout] = lambertpz(xic, yic, tfdes, xf, yf, xlongm, xlongt, maxit)
%LAMBERTPZ  Listing 13.2's efficient Lambert routine, zarchan.txt:3943-3955.
%
%   Given a position, a target position and a desired flight time, iterate on
%   the flight-path angle GAM until the closed-form flight time matches.  The
%   bracket [gmin, gmax] comes from the escape-velocity condition; the step is
%   a secant that falls back to bisection whenever it leaves the bracket.
%
%   hitcap is 1 when the iteration cap was reached.  The listing has no cap and
%   would spin forever on a non-finite secant step; exhausting it here returns
%   the current bracket midpoint's velocity and lets a structural row say so.
gm = 1.4077e16;
ric = sqrt(xic^2+yic^2);  rf = sqrt(xf^2+yf^2);
cphi = (xic*xf+yic*yf)/(ric*rf);
cphi = min(1, max(-1, cphi));                 % see arcAngle for why
phi = acos(cphi);  sphi = sin(phi);  r0 = ric;
icount = 0;  hitcap = 0;
gmin = atan2((sphi-sqrt(2.*r0*(1.-cphi)/rf)), (1-cphi));
gmax = atan2((sphi+sqrt(2.*r0*(1.-cphi)/rf)), (1-cphi));
gam = (gmin+gmax)/2.;  tf = 0;
vrx = NaN;  vry = NaN;  gold = gam;  told = 0;
while ~(abs(tfdes-tf) <= (.00000001*tfdes))
    top = gm*(1.-cos(phi));
    temp = r0*cos(gam)/rf-cos(phi+gam);
    bot = r0*cos(gam)*temp;
    if bot > 0
        v = sqrt(top/bot);
        if xlongt > xlongm                    % see header, resolution 2
            vrx = v*cos(pi/2.-gam+xlongm);  vry = v*sin(pi/2.-gam+xlongm);
        else
            vrx = v*cos(-pi/2.+gam+xlongm); vry = v*sin(-pi/2.+gam+xlongm);
        end
        xlam = r0*v*v/gm;
        if xlam > 0 && xlam < 2
            top1 = tan(gam)*(1-cos(phi))+(1-xlam)*sin(phi);
            bot1p = (1-cos(phi))/(xlam*cos(gam)*cos(gam));
            bot1 = (2-xlam)*(bot1p+cos(gam+phi)/cos(gam));
            top2 = 2*cos(gam);
            bot2 = xlam*((2/xlam-1)^1.5);
            top3 = sqrt(2/xlam-1);
            bot3 = cos(gam)/tan(phi/2)-sin(gam);
            temp = (top2/bot2)*atan2(top3,bot3);
            tf = r0*(top1/bot1+temp)/(v*cos(gam));
        else
            tf = inf;                         % at or past escape: too lofted
        end
    else
        tf = inf;
    end
    if ~isfinite(tf), tf = inf; end
    icount = icount+1;
    if tf > tfdes, gmax = gam; else, gmin = gam; end
    if icount == 1
        xnext = (gmax+gmin)/2.;
    else
        xnext = gam+(gam-gold)*(tfdes-tf)/(tf-told);
        % NaN fails BOTH of the listing's bracket tests, so a non-finite
        % secant step would propagate into gam and never converge.
        if ~isfinite(xnext) || xnext > gmax || xnext < gmin
            xnext = (gmax+gmin)/2.;
        end
    end
    gold = gam;  told = tf;  gam = xnext;
    if icount >= maxit, hitcap = 1; break; end
end
% NEAR-PARABOLIC GUARD, see the file header.  Recomputed from the RETURNED
% velocity rather than from the loop's xlam, so what gets scored is the number
% the booster is actually steered on.  Non-finite when the routine never found
% a usable branch -- that case is n_maxit's, not this guard's.
lamout = r0*(vrx^2 + vry^2)/gm;
end


% =============================================================================
function B = benchmarks(P, G, L, opts)
%BENCHMARKS  Computed vs published.  EVERY book value was grepped out of
%   zarchan_decoded.txt and its line number is in the .source field.  Nothing
%   here was read off a figure -- the extraction turned Figs. 13.11-13.13 into
%   image links.  Where the book states a SHAPE about a figure that prose is
%   greppable, and an ordering check is stronger than an eyeballed value.
%
%   Bands: 'text ~N' = the book wrote "about"/"nearly" N, so the band is the
%   rounding it implies; 'exact' = an identity of the listing's own arithmetic;
%   'sim' = the whole simulation, banded at 1% of the flight time.
B = emptyBench();
bookCase = abs(opts.TF-500) < 1e-9 && abs(opts.ANGDEG-30) < 1e-9 && ...
           abs(opts.XLONGTDEG-45) < 1e-9 && abs(opts.DELV-20000) < 1e-9;
whyOff = sprintf(['off the book''s own case (30 deg -> 45 deg, 20,000 ft/s, ' ...
    'TF = 500 s); this run used %g -> %g deg, %g ft/s, TF = %g s'], ...
    opts.ANGDEG, opts.XLONGTDEG, opts.DELV, opts.TF);

% ---- 1.  staging time ----------------------------------------------------
B = addVal(B, 'first-stage burn time TB1', 15, P.TB1, 'text ~15', 0.10, ...
    'p.288 text, "the discontinuity ... at about 15 s is due to staging" (zarchan.txt:4016)', ...
    's', ['from the sizing block alone -- no trajectory involved.  "About ' ...
          '15 s" is one significant figure, so the band is 10%']);

% ---- 2.  total burn ------------------------------------------------------
B = addVal(B, 'total burn time TB1 + TB2', 60, P.TB1+P.TB2, 'text ~60', 0.05, ...
    'p.284 "burnout ... at about 60 s" (zarchan.txt:3983); p.292 "the booster burn lasts for nearly 60 s" (zarchan.txt:4072)', ...
    's', ['two independent sentences give the same 60 s, which is why this ' ...
          'carries 5% where row 1 carries 10%']);

% ---- 3.  20 g in each stage ---------------------------------------------
B = addVal(B, 'peak axial acceleration, stage 1', 20*32.2, P.at_max1, ...
    'exact', 1e-12, ...
    'p.284 "the maximum acceleration in each stage is 20 g" (zarchan.txt:3983)', ...
    'ft/s^2', sprintf(['the burn schedule is linear in T, so the peak is at ' ...
    'the end of the stage.  Stage 2 gives %.6f ft/s^2 by the same route and ' ...
    'the two are compared to each other in R.selfcheck'], P.at_max2));

% ---- 4.  the sizing really does buy 20,000 ft/s --------------------------
dv = P.XISP1*32.2*log(P.WTOT/(P.WTOT-P.WP1)) + ...
     P.XISP2*32.2*log(P.WTOT2/(P.WTOT2-P.WP2));
B = addVal(B, 'ideal delta-V of the sized two-stage booster', ...
    opts.DELV, dv, 'exact', 1e-9, ...
    'p.284 "a capability of reaching a velocity of 20,000 ft/s" (zarchan.txt:3983, restated :4029)', ...
    'ft/s', ['the rocket equation applied to the WEIGHTS the listing sized, ' ...
    'not to the DELV1/DELV2 it started from -- so it checks the two sizing ' ...
    'blocks rather than restating them.  The listing splits the budget as ' ...
    '.3333 + .6667, which sums to 1 exactly in double precision, so no ' ...
    'allowance for the truncated thirds is needed']);

% ---- 5.  the GEM booster arrives at TF ----------------------------------
if bookCase
    B = addVal(B, 'GEM impact time', opts.TF, G.t_impact, 'sim 1%', 0.01, ...
        'p.284 "reach a target at 45 deg ... in 500 s (TF = 500)" (zarchan.txt:3983); p.297 "both hit the target at the same time" (zarchan.txt:4122)', ...
        's', ['⚠️ WEAKER THAN IT LOOKS: at cutoff the listing snaps the ' ...
        'velocity to the Lambert solution, so this row tests the Lambert ' ...
        'routine and the 440 s coast, NOT the GEM steering.  Row 7 and the ' ...
        'delete experiment are what test the steering']);
else
    B = addSkip(B, 'GEM impact time', '500', whyOff, ...
        'p.284 (zarchan.txt:3983); p.297 (zarchan.txt:4122)', 's');
end

% ---- 6.  the Lambert companion reaches its solution early ---------------
if isempty(L)
    B = addSkip(B, 'Lambert-steered cutoff time', '~45', ...
        '''companion'' is false, so Listing 13.3 was not run', ...
        'p.289 "the Lambert solution was reached in about 45 s" (zarchan.txt:4027)', 's');
elseif bookCase
    B = addVal(B, 'Lambert-steered cutoff time', 45, L.t_cut, 'text ~45', 0.10, ...
        'p.289 "the Lambert solution was reached in about 45 s" (zarchan.txt:4027); also :4016 "converge at about 45 s"', ...
        's', ['the time DEL first falls below the listing''s own 500 ft/s ' ...
        'threshold.  This is the row that says the ported Lambert routine ' ...
        'is producing the right velocity, not merely a convergent one']);
else
    B = addSkip(B, 'Lambert-steered cutoff time', '~45', whyOff, ...
        'p.289 (zarchan.txt:4027)', 's');
end

% ---- 7.  GEM really does spend the surplus ------------------------------
B = addBound(B, 'velocity to be gained at the end of the GEM phase', ...
    '< 500 ft/s (the book''s own "solution reached" threshold)', ...
    sprintf('%.4g ft/s', G.del_at_cut), G.del_at_cut < 500, ...
    'p.291 "we can still achieve the Lambert solution at the end of burn and hit the target" (zarchan.txt:4057); the 500 ft/s scale is Listing 13.3''s own test (zarchan.txt:4002)', ...
    'ft/s', ['THE ROW THAT TESTS GEM.  All the fuel is burned, so the only ' ...
    'way DEL can still be near zero at burnout is if the surplus was thrown ' ...
    'away sideways.  500 ft/s is not invented here -- it is the threshold ' ...
    'Listing 13.3 uses to declare the Lambert solution achieved']);

% ---- 8.  the wrong-way start --------------------------------------------
if isempty(L)
    B = addSkip(B, 'GEM starts off in the wrong direction, Lambert does not', ...
        'GEM longitude decreases first; Lambert''s does not', ...
        '''companion'' is false, so Listing 13.3 was not run', ...
        'p.293 (zarchan.txt:4082)', 'deg');
else
    okDir = G.dlong_min_deg < -1e-3 && L.dlong_min_deg > -1e-6;
    B = addShape(B, 'GEM starts off in the wrong direction, Lambert does not', ...
        'GEM longitude dips >= 0.001 deg below the launch longitude; Lambert''s does not', ...
        sprintf('GEM min %+.4g deg, Lambert min %+.4g deg', ...
                G.dlong_min_deg, L.dlong_min_deg), okDir, ...
        'p.293 "it initially appears to be heading in the wrong direction ... after wasting energy, the GEM-guided booster heads in the right direction" (zarchan.txt:4082)', ...
        ['a pure ordering: at t = 0 the GEM thrust sits at PHI-THET = about ' ...
        '8 deg inertial against a local vertical of 30 deg, i.e. tilted ' ...
        'retrograde, so longitude falls before it rises; Lambert points along ' ...
        'DEL from the first step and never does.  The thresholds are ' ...
        'deliberately asymmetric -- atan2(Y,X) - ANG carries about 6e-15 deg ' ...
        'of rounding at the launch point, so "never dips" needs a floor']);
end

% ---- 9.  vastly different, then converged -------------------------------
if isempty(L)
    B = addSkip(B, 'boost trajectories differ vastly, then converge', ...
        'boost separation >= 10x the late separation', ...
        '''companion'' is false, so Listing 13.3 was not run', ...
        'p.293 (zarchan.txt:4082), p.297 (zarchan.txt:4122)', 'nmi');
elseif min(numel(G.t), numel(L.t)) < 2
    B = addSkip(B, 'boost trajectories differ vastly, then converge', ...
        'boost separation >= 10x the late separation', ...
        'one of the two runs recorded fewer than two samples', ...
        'p.293 (zarchan.txt:4082), p.297 (zarchan.txt:4122)', 'nmi');
else
    nc = min(numel(G.t), numel(L.t));
    sep = hypot(G.x(1:nc)-L.x(1:nc), G.y(1:nc)-L.y(1:nc))/6076.;   % nmi
    inBoost = G.t(1:nc) <= (P.TB1+P.TB2);
    if any(inBoost), sepBoost = max(sep(inBoost)); else, sepBoost = NaN; end
    sepLate  = sep(nc);
    ratio    = sepBoost/max(sepLate, 1e-9);
    gridOK   = any(inBoost) && max(abs(G.t(1:nc)-L.t(1:nc))) < 0.5*P.samp;
    B = addShape(B, 'boost trajectories differ vastly, then converge', ...
        'peak boost separation >= 10x the separation at the last common sample', ...
        sprintf('%.4g nmi in boost vs %.4g nmi late (%.0fx)', ...
                sepBoost, sepLate, ratio), gridOK && ratio >= 10, ...
        'p.293 "both trajectories are vastly different during the boost phase" (zarchan.txt:4082); p.297 "they eventually converge, and both hit the target at the same time" (zarchan.txt:4122)', ...
        ['a RATIO, so it needs no invented length scale.  The two runs share ' ...
        'one time grid by construction, which gridOK re-checks rather than ' ...
        'assumes.  10x is a reading of "vastly different ... eventually ' ...
        'converge", not a measurement -- the book prints no number off ' ...
        'Fig. 13.12 or Fig. 13.13']);
end

% ---- 10.  the GEM angle settles -----------------------------------------
thet = G.thet_deg(isfinite(G.thet_deg));
if numel(thet) < 3
    B = addSkip(B, 'GEM angle reaches steady state before burnout', ...
        'final angle small against the initial angle', ...
        'fewer than three guided samples were recorded', ...
        'p.292-293 (zarchan.txt:4072, :4081)', 'deg');
else
    frac = thet(end)/max(thet(1), realmin);
    B = addShape(B, 'GEM angle reaches steady state before burnout', ...
        'final GEM angle <= 10% of its initial value', ...
        sprintf('%.4g deg -> %.4g deg (%.1f%%)', thet(1), thet(end), 100*frac), ...
        frac <= 0.10, ...
        'p.292-293 "the angle the thrust vector makes with respect to the velocity to be gained vector approaches steady state in slightly over 50 s" (zarchan.txt:4072, :4081); Fig. 13.11 caption (zarchan.txt:4083)', ...
        ['⚠️ the two book statements disagree in TONE -- :4081 says slightly ' ...
        'over 50 s, the caption at :4083 says "quickly" -- so the SETTLING ' ...
        'TIME is only reported (INFO row) and the settled VALUE is what is ' ...
        'scored.  Ideally THET is proportional to DVCAP, both decaying to ' ...
        'zero together, which is why the end of burn is where to look']);
end

% ---- INFO.  prose and code disagree about what the 50 ft/s test measures --
tset = NaN;
if numel(thet) >= 3
    tt = G.t(isfinite(G.thet_deg));
    j  = find(thet > 0.10*thet(1), 1, 'last');
    if ~isempty(j) && j < numel(tt), tset = tt(j+1); end
end
B = addInfo(B, 'GEM cutoff threshold, and when the angle settles', ...
    '"terminated when the velocity to be gained drops below 50 ft/s"', ...
    sprintf('code tests DVCAP > 50 (capability), not DEL; angle within 10%% of settled from t = %.4g s', tset), ...
    'p.292 (zarchan.txt:4071) against the listing itself (zarchan.txt:4105)', ...
    ['NOT SCORED, two reasons.  (a) The prose says velocity TO BE GAINED, the ' ...
    'listing tests velocity CAPABILITY; under GEM the two converge on each ' ...
    'other by burnout, so the nominal case cannot separate them and scoring ' ...
    'either would be scoring an assumption.  (b) A settling time depends ' ...
    'entirely on the threshold chosen for "settled", and the book gives none.']);
end


% =============================================================================
function S = noteLambda(S, lamOut)
%NOTELAMBDA  NEAR-PARABOLIC GUARD accumulator, see the file header.  A
%   non-finite lambda means lambertpz never found a usable branch at all; that
%   is n_maxit's failure to report, not this guard's, so it is counted and not
%   folded into the maximum (max() would silently drop a NaN and make the
%   guard look green).
if isfinite(lamOut)
    S.lam_max = max(S.lam_max, lamOut);
end
S.lam_calls = S.lam_calls + 1;
end


% =============================================================================
function m = maxLambda(runs)
%MAXLAMBDA  Largest lambda over whichever runs actually happened.
m = 0;
for i = 1:numel(runs)
    r = runs{i};
    if ~isempty(r) && isfield(r, 'lam_max'), m = max(m, r.lam_max); end
end
end


% =============================================================================
function K = structuralChecks(P, G, D, L)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never evidence that
%   this port reproduces Zarchan -- the weaker claim that it agrees with
%   itself, which is what catches wiring faults.  .names and .oks are what
%   benchSuite's adaptGeneric turns into board rows; without BOTH, everything
%   computed here is invisible and the board still looks green.
K = struct();
names = {};  oks = [];

% 1  the sizing block is the rocket equation, solved for propellant weight
d1 = P.XISP1*32.2*log(P.WTOT/(P.WTOT-P.WP1))/P.DELV1 - 1;
d2 = P.XISP2*32.2*log(P.WTOT2/(P.WTOT2-P.WP2))/P.DELV2 - 1;
K.sizing_resid = max(abs([d1 d2]));
names{end+1} = 'both stages satisfy the rocket equation';
oks(end+1)   = K.sizing_resid <= 1e-12;

% 2  "20 g in EACH stage" -- the two peaks must be the same number
K.at_peak_resid = abs(P.at_max1/P.at_max2 - 1);
names{end+1} = 'peak axial acceleration identical in the two stages';
oks(end+1)   = K.at_peak_resid <= 1e-12;

% 3  the Lambert routine converged every single call, in all three runs
K.n_maxit = G.n_maxit + D.n_maxit;
names{end+1} = 'Lambert routine converged on every call';
oks(end+1)   = K.n_maxit == 0;

% 4  Vcap only ever falls, and its slope JUMPS at staging -- the book's
%    "discontinuity in that curve is due to staging" (zarchan.txt:4081).
%    AT ends stage 1 at 20 g and restarts stage 2 near 5 g, so the drop is
%    large; a factor of 2 is a floor well clear of that, not a measurement.
dv = G.dvcap(isfinite(G.dvcap));
K.dvcap_max_rise = 0;
if numel(dv) >= 2, K.dvcap_max_rise = max(diff(dv)); end
K.stage_at_ratio = P.at_max1/max(32.2*P.TRST2/P.WTOT2, realmin);
names{end+1} = 'Vcap never rises, and axial accel drops >2x at staging';
oks(end+1)   = K.dvcap_max_rise <= 1e-9 && K.stage_at_ratio > 2;

% 5  ⭐ DELETE EXPERIMENT.  Invert the one comparison the transcription could
%    not settle from shape -- "if XLONGT > XLONGM" -- and nothing else.  The
%    shipped resolution must keep the booster above ground through the burn;
%    the inverted one must not.  If both survive, the resolution is not
%    load-bearing and the argument for it in the header is unverified.
K.alt_min_boost_ft      = G.alt_min_boost_ft;
K.alt_min_boost_flip_ft = D.alt_min_boost_ft;
names{end+1} = 'DELETE: inverting the XLONGT>XLONGM branch flies it into the ground';
oks(end+1)   = G.alt_min_boost_ft > 0 && D.alt_min_boost_ft < 0;

% 6  NEAR-PARABOLIC GUARD.  See the file header for the 98,751 ft miss this
%    exists to catch.  ⚠ NOT keyed on any convergence flag -- invariant 3 above
%    IS that flag, and it reads GREEN on the wrong answer.  ⚠ Nor on agreement
%    with the package's five other transcriptions of Listing 13.2: all six
%    agree to 0.000e+00 on the failing geometry, which is why a six-way
%    comparison could not find this and an independent solver had to.
K.lambda_max   = maxLambda({G, L, D});
K.lambda_calls = G.lam_calls + D.lam_calls;
if ~isempty(L), K.lambda_calls = K.lambda_calls + L.lam_calls; end
names{end+1} = 'near-parabolic guard: lambda = r0|v|^2/gm < 1.999999';
oks(end+1)   = isfinite(K.lambda_max) && K.lambda_max < 1.999999;

K.names = names;
K.oks   = logical(oks(:).');
end


% =============================================================================
function s = render(P, G, L, R, ev)
%RENDER  Reads R.p and the run structs, never opts.
lines = {};
lines{end+1} = 'mgs.zarchan.gemSimulation -- Zarchan Listing 13.4, GEM booster steering';
lines{end+1} = sprintf('  units: %s', R.units);
lines{end+1} = sprintf(['  sizing: TB1 = %.4g s, TB2 = %.4g s, WTOT = %.6g lb, ' ...
    'peak axial accel %.5g / %.5g ft/s^2'], P.TB1, P.TB2, P.WTOT, P.at_max1, P.at_max2);
lines{end+1} = sprintf(['  GEM     : impact %.6g s, cutoff %.4g s, DEL at cutoff ' ...
    '%.4g ft/s, THET %.4g -> %.4g deg'], G.t_impact, G.t_cut, G.del_at_cut, ...
    G.thet0_deg, lastFinite(G.thet_deg));
if isempty(L)
    lines{end+1} = '  Lambert : not run (''companion'' is false)';
else
    lines{end+1} = sprintf('  Lambert : impact %.6g s, cutoff %.4g s', ...
        L.t_impact, L.t_cut);
end
for i = 1:numel(R.bench)
    b = R.bench(i);
    switch lower(b.kind)
    case 'info',  tag = 'INFO';
    case 'skip',  tag = 'SKIP';
    otherwise,    tag = verdictTag(b.ok);
    end
    lines{end+1} = sprintf('  [%s] %-52s book %-14s ours %s', ...
        tag, b.what, b.book_str, b.ours_str); %#ok<AGROW>
end
for i = 1:numel(R.selfcheck.names)
    lines{end+1} = sprintf('  [%s] structural: %s', ...
        verdictTag(R.selfcheck.oks(i)), R.selfcheck.names{i}); %#ok<AGROW>
end
lines{end+1} = sprintf('  scored %d row(s); pass = %d', sum(ev), R.pass);
s = strjoin(lines, newline);
end

function t = verdictTag(ok)
if ok, t = ' ok '; else, t = 'FAIL'; end
end

function v = lastFinite(x)
f = x(isfinite(x));
if isempty(f), v = NaN; else, v = f(end); end
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER must match the adders exactly; a reordered
%   struct makes [B.ok] mean something else while still looking like a typo.
%   This is the schema mgs.zarchan.benchSuite's adaptGeneric consumes, and it
%   is STRICT -- a near miss errors rather than being accommodated.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.6g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.6g', ours);
B(i).pct  = NaN;   B(i).band = band;  B(i).tol = tol;
B(i).ok   = false; B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = units;  B(i).detail = detail;
if ~isfinite(ours) || book == 0
    B(i).skipped = true;
    B(i).why = 'the computed value is not finite (or the book value is zero)';
    return
end
B(i).pct = 100*(ours/book - 1);
B(i).ok  = abs(ours/book - 1) <= tol;
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
%ADDINFO  Reported, never scored: skipped = true keeps it out of the pass
%   count in BOTH directions.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'info';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'info';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true;  B(i).why = why;
B(i).source = source;  B(i).units = '-';  B(i).detail = '';
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
