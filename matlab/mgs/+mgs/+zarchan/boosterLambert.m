function R = boosterLambert(opts)
%MGS.ZARCHAN.BOOSTERLAMBERT  Zarchan Listing 13.3 -- two-stage booster flown by
%   LAMBERT GUIDANCE, with Listing 13.2's Lambert routine (lambertpz) inside it.
%
%   R = mgs.zarchan.boosterLambert()
%   R = mgs.zarchan.boosterLambert('TF', 600, 'assert', false)
%   R = mgs.zarchan.boosterLambert('deleteexp', false)   % skip the open-loop run
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: ft, ft/s, ft/s^2, lbf, g = 32.2, nmi = 6076 ft.
%   Earth: A = 2.0926e7 ft, GM = 1.4077e16 ft^3/s^2.  NOTHING IS SI and
%   nothing is converted, so every number printed lies straight against a
%   published one.  Convert AT THE CALL SITE and say so there.
%   Degrees use the BOOK'S 57.3, not 180/pi -- see THE 57.3 CONVENTION.
% ===========================================================================
%
%   WHY THIS ONE.  Listings 13.1/13.2 solve Lambert's problem once, open loop.
%   13.3 is the first place the solver is CLOSED IN A LOOP: every 0.01 s the
%   boosting missile asks "what velocity puts me on the target in the time I
%   have left", subtracts what it has, and points the thrust along the
%   difference.  So what is worth scoring is not any intermediate velocity but
%   whether the guided booster ARRIVES -- hence the two headline rows (impact
%   time, book "exactly 500 s"; along-surface miss) and hence a delete
%   experiment that FREEZES the Lambert call at t = 0 rather than deleting
%   guidance outright.  A booster with no steering never leaves the pad and
%   fails for an uninteresting reason; one steered by a STALE Lambert solution
%   flies a perfectly smooth trajectory and lands 105 nmi long, 85 s late.
%   Only the second tests the loop.  For us this is the shape of "recompute
%   the whole terminal solution every tick and steer along the error" -- the
%   opposite of schedule-once-then-track, whose predictor error keeps biting.
%   *** WHAT IT IS NOT: *** no atmosphere, no drag, no g limit, no range
%   safety, thrust that can point anywhere instantly, and a STATIONARY aim
%   point on a non-rotating spherical Earth.  Do not quote a miss out of here
%   as an achievable accuracy.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_13_03.m does not parse.  This book's OCR maps
%   '<' to ',' and '>' to '.' inside code:
%     while ~(ALT,0.&T.10.) -> ~(ALT < 0 && T > 10)    stop on impact
%     while STEP ,=1        -> STEP <= 1               (body sets STEP=2)
%     if T,TB1              -> T < TB1      <<< THE UNRESOLVED ONE
%     elseif T , (TB1+TB2)  -> T < (TB1+TB2)
%     if (TRST.0.&DEL.500.) -> (TRST > 0 && DEL > 500)
%     if S.=9.99999         -> S >= 9.99999            (S resets to 0)
%
%   listing_13_03.m's header reports ONE unresolved variable-to-variable
%   comparison, "if T,TB1".  Resolved as T < TB1 -- stage 1 burns FIRST --
%   forced by the branch body, not taste: it sets WGT = -WP1*T/TB1 + WTOT,
%   which is the full stack at T = 0 and WTOT - WP1 at T = TB1, a mass
%   depletion law physical only on [0,TB1]; its mirror likewise only on
%   [TB1,TB1+TB2].  MEASURABLE too: T > TB1 flies stage 2 first and moves the
%   miss from 0.03 ft to 119 ft while still arriving at 500.01 s -- which is
%   why the impact-TIME row cannot police it and the MISS row exists.
%
%   Listing 13.2's lambertpz carries two more, resolved the same way:
%     xlongt.xlongm -> xlongt > xlongm, which side of the radius the velocity
%        is thrown to.  Flipped, the missile goes the wrong way round the
%        Earth: miss 1.2e7 ft.  Table 13.2 reproduces only under '>'.
%     tf.tfdes -> tf > tfdes, the bracket update.  Flight time increases with
%        flight-path angle over [gmin,gmax] -- visible in Table 13.2 itself,
%        33.75 deg giving 1239 s against 11.25 deg giving 814 s -- so a tf
%        that is too LONG must become the new upper bound.
%     (xnext.gmax|xnext,gmin) -> (xnext > gmax || xnext < gmin), the secant
%        step's safety net; both directions forced by "outside the bracket".
%
%   Book quirks kept, and one that could not be:
%     * TRST = 0 in the thrust-termination branch is DEAD -- the staging logic
%       reassigns TRST from T on the next pass.  QBOOST = 0 is what actually
%       cuts off, by freezing AXT/AYT at the zeros that branch set.  Kept
%       exactly; STRUCTURAL CHECK 3 is what proves thrust really stops.
%     * X1OLD/Y1OLD are written there too and overwritten next step.  Dead.
%     * LEFT and DELVK are set and never used.  Dropped.
%     * lambertpz has NO iteration cap and spins forever on a call that cannot
%       converge.  'lam_maxit' is added; exceeding it is counted and SCORED,
%       never silently accepted.  A deviation, stated.
%     * "save datfil.txt output /ascii" is the book's -ascii typo.  Nothing is
%       written to disk here at all.
%
%   THE 57.3 CONVENTION.  The listing writes ANG = ANGDEG/57.3, so "30 deg" is
%   29.9978 true degrees and "45 deg" is 44.9967.  Kept: it is self-consistent
%   (both ends use it), and Table 13.2 prints 57.3*gam -- our converged gam is
%   0.38789647 rad = 22.22480 TRUE deg, and 57.3*0.38789647 = 22.22645 against
%   the book's printed 22.2264402.  Use 180/pi and that agreement is thrown
%   away for nothing.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS -- 9 scored, 3 info.  Every book value was grepped out of the
%   DECODED zarchan.txt; the line number is in each row's .source and the band
%   provenance in its .detail.  Nothing was read off a figure (the extraction
%   turned every figure into an image link); the two SHAPE/BOUND rows come
%   from PROSE about figures, which is greppable.
%     1 impact time vs "reaches the target at exactly 500 s" (:4026)
%     2 along-surface miss (BOUND, 1 ppm of the flown range -- the book
%       publishes no miss number, so the threshold is scaled, not chosen)
%     3 stage-1 burnout vs "about 15 s ... staging" (:4016), 10%
%     4 total burn vs "about 60 s" (:3983, restated :4027), 10%
%     5 thrust termination vs "reached in about 45 s" (:4016, :4027), 10%
%     6-8 TABLE 13.2 (:3962-3970): VRX, VRY and the iteration count.  Its
%       driver is Listing 13.2's own nominal, NOT the booster's, so these
%       three score at every setting of every option here.
%     9 SHAPE: "this component [y] is much larger than the x component" (:4016)
%
%   *** THE MINUS SIGN ON VRX IS RECONSTRUCTED. ***  The extraction maps '-'
%   to '2', so Table 13.2 prints VRX as "27695.55795".  Read as positive it
%   contradicts its own row: at gam = 22.2264 deg, xlongm = 45 deg the routine
%   returns v*cos(112.77 deg), which is NEGATIVE, and |v| recovered from VRY
%   alone puts it at -7.7e3.  Row 2 settles it -- "212075.71473" as -12075.7
%   satisfies the same identity to four figures where +212075 does not.
%   Table 13.1 (:3913) prints the same solution rounded and is cited alongside.
%
%   NOT SCORED (info): apogee/downrange (Fig. 13.8 publishes no numbers);
%   Table 13.1's 1084 iterations (that count is Listing 13.1's, not ported
%   here); speed at cut-off (DELV = 20,000 ft/s is an ideal delta-V budget,
%   not a predicted terminal speed, and the book never claims it is reached).
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED IN MATLAB. ***  This
%   file has never been executed; these come from an independent replica of
%   the same arithmetic in another language, so a first run can be compared
%   against an expectation instead of admired:
%     TB1 = 14.9076 s, TB2 = 44.6433 s, total 59.5509 s
%     cut-off (DEL < 500 ft/s) at 43.92 s; 4392 Lambert calls, never above 6
%       iterations, DEL not increasing on a single one of them
%     impact 500.00 s, miss 0.034 ft, apogee 113.85 nmi, downrange 901.58 nmi
%     Table 13.2 replica: 7 iterations, VRX -7695.580, VRY 18329.304,
%       57.3*gam = 22.22645 deg
%     round trip (impulsive Lambert + coast): 0.003 ft at H_rt = 0.05 s
%     open-loop delete experiment: impact 585.19 s, miss 641,158 ft
%   If a row lands near a band EDGE that is not a pass to be accepted.  Do NOT
%   widen a band to make a row green.
%
%   COST: ~50k RK2 steps plus ~4.4k Lambert calls, and the delete experiment
%   ~58k steps more.  'deleteexp', false halves it -- and turns that
%   structural row RED, not green, because not-run is not a pass.
%
%   OUTPUTS
%     R.traj      .t .distnm .altnm .energy, every 'samp' s as the listing does
%     R.boost     boost history: .t .vx .vy .vrx .vry .del .at
%     R.impact    .t .lon_deg .miss_ft .alt_ft .apogee_nmi .downrange_nmi
%     R.cutoff    .t .reason ('lambert'|'burnout'|'none') .nlam .maxit .nonconv
%                 .lam_max (largest lambda = r0|v|^2/gm over the loop's calls)
%     R.table132  the Listing 13.2 replica: .vrx .vry .iters .gamdeg
%     R.open      the open-loop delete experiment ([] if 'deleteexp' false)
%     R.bench / R.selfcheck / R.pass / R.verdict / R.p / R.summary
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
%   SCORED OVER EVERY CALL.  Listing 13.3 is the first listing that closes the
%   Lambert solver in a LOOP -- a default run makes ~4.4k calls -- so the guard
%   tracks the largest lambda over all of them, plus the round-trip call in
%   invariant 2 and the Table 13.2 replica call.  A guard on one final solve
%   would leave the other 4,399 unchecked.
%
%   ⚠ WHY INVARIANT 5 WAS NOT ENOUGH.  It asks whether every call converged
%   inside lam_maxit.  The failing solve DOES converge, inside 35 passes, well
%   under the cap -- that is the entire point.  Invariant 5 reads green on it.
%
%   THIS PORT'S REACH -- THE GUARD BITES, BUT NEVER ALONE.  MEASURED:
%     XLONGTDEG = 51, TF = 190 s -> in-loop lambda = 1.999999972, guard RED
%     XLONGTDEG = 49, TF = 183.5 -> in-loop lambda = 1.999999929
%     XLONGTDEG = 45, TF = 146   -> lambda = 1.999999910 (the ROUND-TRIP call
%                                   of invariant 2, not the loop)
%   ⚠ HONEST LIMIT.  All of these also have cutoff.nonconv = 1, so invariant 5
%   is red there too.  A refined scan (6 longitudes x 73 flight times at 0.5 s,
%   then 5 x 81 at 0.05 s) found NO geometry where the guard is the ONLY red
%   row: the best run with every call converged reaches lambda = 1.999683457,
%   just under the cap.  So on THIS port the guard is verified to fire, but its
%   unique-catch value is NOT demonstrated -- driving the loop into the
%   near-parabolic band also drives at least one call into the cap.
%   ⭐ Contrast ballisticIntercept and pulsedGuidance, where a geometry exists
%   with EVERY other invariant green and only this row red.
%   MARGIN AT THE BOOK'S OWN CASE: 0.256764805 over the 4392 loop calls;
%   0.587458159 once the round trip and the Table 13.2 replica are folded in.
% ---------------------------------------------------------------------------
%
%   See also MGS.ZARCHAN.BENCHSUITE.
arguments
    % Listing 13.3's own nominal case, verbatim.
    opts.TF        (1,1) double {mustBeFinite, mustBePositive} = 500.0   % s
    opts.ANGDEG    (1,1) double {mustBeFinite} = 30.0                    % deg
    opts.XLONGTDEG (1,1) double {mustBeFinite} = 45.0                    % deg
    opts.DELV      (1,1) double {mustBeFinite, mustBePositive} = 20000.0 % ft/s
    opts.H         (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s
    opts.samp      (1,1) double {mustBeFinite, mustBePositive} = 10.0    % s
    % The listing has no cap and hangs on a non-converging call.  100 is ~15x
    % the worst count seen on the nominal case (6).
    opts.lam_maxit (1,1) double {mustBeFinite, mustBePositive} = 100
    % The open-loop run: same booster, Lambert solution FROZEN at t = 0.  The
    % delete experiment for "closed in a loop".
    opts.deleteexp (1,1) logical = true
    % THE ONLY SWITCHES.  Not nargin: with a name-value-only arguments block
    % MATLAB reports nargin = 0 however many were passed.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

RAD  = 57.3;                % THE BOOK'S degrees per radian.  See header.
AE   = 2.0926e7;            % ft
GM   = 1.4077e16;           % ft^3/s^2
FTNM = 6076.0;              % ft per nmi, the listing's own value

if opts.H >= opts.samp
    error('mgs:zarchan:boosterLambert:samp', ...
        ['H = %g s is not smaller than the record interval samp = %g s, so ' ...
         'the trajectory would be recorded at the step rate while claiming ' ...
         'the coarser interval in its own time vector.'], opts.H, opts.samp);
end
if opts.H > 0.5
    error('mgs:zarchan:boosterLambert:step', ...
        ['H = %g s.  The booster pulls 20 g and stages at ~15 s; at this ' ...
         'step the mass history is sampled a handful of times per stage and ' ...
         'RK2 returns a smooth, entirely wrong trajectory rather than ' ...
         'blowing up, which is why this is refused and not warned about.'], opts.H);
end

P = sizeBooster(opts.DELV);

if opts.TF <= P.TB1 + P.TB2
    error('mgs:zarchan:boosterLambert:tooshort', ...
        ['TF = %g s is not longer than the %.2f s of available burn.  The ' ...
         'guidance call would then be handed TGOLAM = TF - T <= 0, and ' ...
         'lambertpz''s convergence test |tfdes - tf| <= 1e-8*tfdes cannot be ' ...
         'satisfied for a non-positive tfdes -- so every call would run to ' ...
         'the iteration cap and the port would grind for minutes before ' ...
         'reporting a wrong trajectory.'], opts.TF, P.TB1 + P.TB2);
end

% =========================================================================
%   LISTING 13.3, closed loop.  zarchan.txt:3985-4008.
% =========================================================================
S = boosterRun(P, opts, RAD, AE, GM, FTNM, false);

% The delete experiment: identical booster, Lambert solved ONCE at t = 0 and
% then held.  See the header for why this and not "guidance off".
if opts.deleteexp
    O = boosterRun(P, opts, RAD, AE, GM, FTNM, true);
else
    O = [];
end

% =========================================================================
%   LISTING 13.2's own nominal driver, for Table 13.2.  Independent of every
%   option above -- which is what keeps three rows scoreable off-case.
% =========================================================================
T132 = table132(AE, GM, opts.lam_maxit);

R = struct();
R.traj     = S.traj;
R.boost    = S.boost;
R.impact   = S.impact;
R.cutoff   = S.cutoff;
R.table132 = T132;
% NEAR-PARABOLIC GUARD, see the file header.  The loop's own worst case; the
% invariant folds in the round-trip and Table 13.2 calls as well.
R.lambda_max = S.cutoff.lam_max;
R.open     = O;
R.p = struct('TF',opts.TF, 'ANGDEG',opts.ANGDEG, 'XLONGTDEG',opts.XLONGTDEG, ...
             'DELV',opts.DELV, 'H',opts.H, 'samp',opts.samp, ...
             'lam_maxit',opts.lam_maxit, 'deleteexp',opts.deleteexp, ...
             'TB1',P.TB1, 'TB2',P.TB2, 'WTOT',P.WTOT, ...
             'TRST1',P.TRST1, 'TRST2',P.TRST2, 'RAD',RAD, 'A',AE, 'GM',GM, ...
             'units','ft, ft/s, ft/s^2, lbf, s; g = 32.2; nmi = 6076 ft');

R.selfcheck = structuralChecks(S, O, T132, opts, RAD, AE, GM);

if opts.benchmark
    R.bench = benchmarks(S, T132, P, opts, FTNM);
    ev = ~[R.bench.skipped];
    R.pass = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev = false;
    R.pass = false;     % NOT true.  "Not checked" must never render as "fine".
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawTraj(R); end

% ---- adjudication.  The verdict is FORMED either way; 'assert' chooses only
%      the reaction to it.  benchSuite runs every port with assert off so that
%      one red row cannot hide the board, so this must return R complete.
msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false, so nothing was compared against the book.  R.pass is false meaning NOT CHECKED.';
elseif ~any(ev)
    msg = 'not one book value could be evaluated -- an unvalidated port must not be handed back silently.';
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question.  First: %s.'], ...
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
        error('mgs:zarchan:boosterLambert:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:boosterLambert:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function P = sizeBooster(DELV)
%SIZEBOOSTER  Listing 13.3's first two lines, verbatim.  Weights lbf, times s.
%
%   Stage 2 is sized FIRST, because it carries only the payload; stage 1 is
%   then sized to lift the whole of stage 2.  That order is not a style
%   choice: WTOT2 is an input to TOP1.
XISP1 = 300.;  XISP2 = 300.;  XMF1 = .90;  XMF2 = .90;  WPAY = 100.;
DELV1 = .3333*DELV;   DELV2 = .6667*DELV;
AMAX1 = 20.;          AMAX2 = 20.;

TOP2  = WPAY*(exp(DELV2/(XISP2*32.2)) - 1.);
BOT2  = 1/XMF2 - ((1.-XMF2)/XMF2)*exp(DELV2/(XISP2*32.2));
WP2   = TOP2/BOT2;    WS2 = WP2*(1-XMF2)/XMF2;   WTOT2 = WP2 + WS2 + WPAY;
TRST2 = AMAX2*(WPAY + WS2);        TB2 = XISP2*WP2/TRST2;

TOP1  = WTOT2*(exp(DELV1/(XISP1*32.2)) - 1.);
BOT1  = 1/XMF1 - ((1.-XMF1)/XMF1)*exp(DELV1/(XISP1*32.2));
WP1   = TOP1/BOT1;    WS1 = WP1*(1-XMF1)/XMF1;   WTOT = WP1 + WS1 + WTOT2;
TRST1 = AMAX1*(WTOT2 + WS1);       TB1 = XISP1*WP1/TRST1;

if ~(BOT2 > 0) || ~(BOT1 > 0)
    error('mgs:zarchan:boosterLambert:sizing', ...
        ['stage sizing is infeasible at DELV = %g ft/s: a mass fraction of ' ...
         '%g cannot deliver this delta-V, so the rocket equation returns a ' ...
         'NEGATIVE propellant weight.  That then runs the mass-depletion law ' ...
         'backwards with no other symptom, which is why this is refused.'], ...
        DELV, XMF2);
end
P = struct('WPAY',WPAY, 'WP1',WP1, 'WS1',WS1, 'WP2',WP2, 'WS2',WS2, ...
           'WTOT',WTOT, 'WTOT2',WTOT2, 'TRST1',TRST1, 'TRST2',TRST2, ...
           'TB1',TB1, 'TB2',TB2);
end


% =============================================================================
function S = boosterRun(P, opts, RAD, AE, GM, FTNM, freezeLambert)
%BOOSTERRUN  Listing 13.3's integration loop, zarchan.txt:3985-4008.
%
%   Every constant arrives as an ARGUMENT.  MATLAB local functions cannot see
%   the caller's workspace, and a previous wave in this package lost a run to
%   exactly that.
%
%   freezeLambert = true runs the DELETE EXPERIMENT: lambertPZ is called once,
%   on the first step, and its answer is held for the rest of the boost.
%   Everything else is bit-identical to the closed loop.

H  = opts.H;   TF = opts.TF;
T  = 0.;  Ssamp = 0.;
ANG    = opts.ANGDEG/RAD;
XLONGT = opts.XLONGTDEG/RAD;
X  = AE*cos(ANG);   Y = AE*sin(ANG);   ALT = sqrt(X^2 + Y^2) - AE;
XFIRST = X;  YFIRST = Y;
X1 = 0.;  Y1 = 0.;  AXT = 0.;  AYT = 0.;
XF = AE*cos(XLONGT);  YF = AE*sin(XLONGT);
QBOOST = 1;  TRST = 0.;  AT = 0.;
XD = 0.;  YD = 0.;  X1D = 0.;  Y1D = 0.;
VRX = NaN;  VRY = NaN;  frozen = false;

% Hard stop.  The listing has none: at a large DELV the trajectory never comes
% back down and the while loop runs forever.
tstop = 8*TF;

nmax = ceil(tstop/opts.samp) + 8;
tr_t = zeros(nmax,1);  tr_d = zeros(nmax,1);  tr_a = zeros(nmax,1);
tr_e = zeros(nmax,1);  nrec = 0;
% The boost cannot outlast burnout: at T >= TB1+TB2 the staging logic sets
% TRST = 0, so the guidance block falls into a branch that clears QBOOST.
bmax = ceil((P.TB1 + P.TB2)/H) + 16;
b_t  = zeros(bmax,1);  b_vx = zeros(bmax,1);  b_vy = zeros(bmax,1);
b_rx = zeros(bmax,1);  b_ry = zeros(bmax,1);  b_dl = zeros(bmax,1);
b_at = zeros(bmax,1);  nb = 0;

cut = struct('t',NaN, 'reason','none', 'nlam',0, 'maxit',0, 'nonconv',0, ...
             'lam_max',0);

while ~(ALT < 0 && T > 10.)
    XOLD = X;  YOLD = Y;  X1OLD = X1;  Y1OLD = Y1;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor.  T advances INSIDE this loop, exactly
    % once, which is what puts the second evaluation at t+H.  AXT/AYT are held
    % across BOTH passes: the guidance updates once per outer step.
    while STEP <= 1
        if FLAG == 1
            X  = X  + H*XD;    Y  = Y  + H*YD;
            X1 = X1 + H*X1D;   Y1 = Y1 + H*Y1D;
            T  = T + H;  STEP = 2;
        end
        if T < P.TB1                            % <<< the resolved comparison
            WGT = -P.WP1*T/P.TB1 + P.WTOT;                 TRST = P.TRST1;
        elseif T < (P.TB1 + P.TB2)
            WGT = -P.WP2*T/P.TB2 + P.WTOT2 + P.WP2*P.TB1/P.TB2;  TRST = P.TRST2;
        else
            WGT = P.WPAY;                                  TRST = 0.;
        end
        AT  = 32.2*TRST/WGT;
        XD  = X1;  YD = Y1;
        TEMBOT = (X^2 + Y^2)^1.5;
        X1D = -GM*X/TEMBOT + AXT;
        Y1D = -GM*Y/TEMBOT + AYT;
        FLAG = 1;
    end
    % The listing's trailing "FLAG = 0" is dropped: the outer loop
    % re-initialises FLAG, so that assignment can never be read.

    X  = (XOLD  + X )/2 + .5*H*XD;    Y  = (YOLD  + Y )/2 + .5*H*YD;
    X1 = (X1OLD + X1)/2 + .5*H*X1D;   Y1 = (Y1OLD + Y1)/2 + .5*H*Y1D;
    ALT = sqrt(X^2 + Y^2) - AE;

    % ---- THE LAMBERT FEEDBACK LOOP ------------------------------------
    if QBOOST == 1
        if ~(freezeLambert && frozen)
            TGOLAM = TF - T;
            XLONGM = atan2(Y, X);
            [VRX, VRY, nit, conv, lamOut] = lambertPZ(X, Y, TGOLAM, XF, YF, ...
                                        XLONGM, XLONGT, GM, opts.lam_maxit);
            cut.nlam    = cut.nlam + 1;
            cut.maxit   = max(cut.maxit, nit);
            cut.nonconv = cut.nonconv + double(~conv);
            % NEAR-PARABOLIC GUARD accumulator.  A non-finite lambda is
            % invariant 5's failure to report, not this guard's, so it is left
            % out of the maximum rather than silently dropped by max().
            if isfinite(lamOut), cut.lam_max = max(cut.lam_max, lamOut); end
            frozen = true;
        end
        DELX = VRX - X1;  DELY = VRY - Y1;
        DEL  = sqrt(DELX^2 + DELY^2);

        nb = nb + 1;
        b_t(nb) = T;   b_vx(nb) = X1;  b_vy(nb) = Y1;
        b_rx(nb) = VRX; b_ry(nb) = VRY; b_dl(nb) = DEL;  b_at(nb) = AT;

        if (TRST > 0. && DEL > 500.)
            AXT = AT*DELX/DEL;   AYT = AT*DELY/DEL;
        elseif DEL < 500.
            % Thrust termination.  TRST = 0 here is DEAD -- the staging logic
            % reassigns it on the next pass.  QBOOST = 0 is what actually cuts
            % off, by freezing AXT/AYT at the zeros set on this line.  Kept as
            % the book wrote it; STRUCTURAL CHECK 3 proves thrust really ends.
            TRST = 0.;  QBOOST = 0;  AXT = 0.;  AYT = 0.;
            X1 = VRX;  Y1 = VRY;              % snap to the Lambert solution
            X1OLD = X1;  Y1OLD = Y1;          %#ok<NASGU>  dead, kept
            cut.t = T;  cut.reason = 'lambert';
        else
            QBOOST = 0;  AXT = 0.;  AYT = 0.;
            cut.t = T;  cut.reason = 'burnout';
        end
    end

    Ssamp = Ssamp + H;
    if Ssamp >= opts.samp - 0.5*H
        Ssamp = 0.;
        Rr = sqrt(X^2 + Y^2);  RFst = sqrt(XFIRST^2 + YFIRST^2);
        CBETA = (X*XFIRST + Y*YFIRST)/(Rr*RFst);
        % The listing has no clamp.  acos(1 + 1e-16) is COMPLEX, and a complex
        % downrange plots and saves to disk without complaint.
        BETA = acos(min(1, max(-1, CBETA)));
        nrec = nrec + 1;
        tr_t(nrec) = T;
        tr_d(nrec) = AE*BETA/FTNM;
        tr_a(nrec) = (Rr - AE)/FTNM;
        tr_e(nrec) = (X1^2 + Y1^2)/2 - GM/Rr;   % specific energy, for check 3
    end

    if T > tstop
        break
    end
end

lon = atan2(Y, X);
S.traj   = struct('t',tr_t(1:nrec), 'distnm',tr_d(1:nrec), ...
                  'altnm',tr_a(1:nrec), 'energy',tr_e(1:nrec));
S.boost  = struct('t',b_t(1:nb), 'vx',b_vx(1:nb), 'vy',b_vy(1:nb), ...
                  'vrx',b_rx(1:nb), 'vry',b_ry(1:nb), 'del',b_dl(1:nb), ...
                  'at',b_at(1:nb));
S.impact = struct('t',T, 'lon_deg',lon*RAD, 'alt_ft',ALT, ...
                  'miss_ft',AE*abs(lon - XLONGT), ...
                  'apogee_nmi',maxOr(tr_a(1:nrec)), ...
                  'downrange_nmi',lastOr(tr_d(1:nrec)), ...
                  'x',X, 'y',Y, 'vx',X1, 'vy',Y1, 'timedout',T > tstop);
S.cutoff = cut;
end


% =============================================================================
function [vrx, vry, icount, converged, lamout] = lambertPZ(xic, yic, tfdes, ...
                                            xf, yf, xlongm, xlongt, gm, maxit)
%LAMBERTPZ  Listing 13.2's efficient Lambert routine, zarchan.txt:3942-3955.
%
%   Given where we are (xic,yic), where we want to be (xf,yf) and how long we
%   have (tfdes), return the velocity that gets us there in a 1/r^2 field.
%   Secant iteration on the flight-path angle inside a bisection bracket taken
%   from the escape-velocity condition, falling back to bisection whenever the
%   secant step leaves the bracket.
%
%   'maxit' is NOT in the book.  The listing's while has no cap and spins
%   forever on a call that cannot converge.  Non-convergence is RETURNED here,
%   not swallowed: the caller counts it and a structural row scores it.

ric  = sqrt(xic^2 + yic^2);
rf   = sqrt(xf^2 + yf^2);
cphi = (xic*xf + yic*yf)/(ric*rf);
cphi = min(1, max(-1, cphi));        % see the acos note in boosterRun
phi  = acos(cphi);
sphi = sin(phi);
r0   = ric;

if phi < 1e-9
    error('mgs:zarchan:boosterLambert:degenerate', ...
        ['Lambert called with a transfer angle of %g rad.  bot3 divides by ' ...
         'tan(phi/2), so this returns Inf and then a NaN velocity that ' ...
         'propagates silently through the whole trajectory.'], phi);
end

gmin = atan2(sphi - sqrt(2.*r0*(1.-cphi)/rf), 1 - cphi);
gmax = atan2(sphi + sqrt(2.*r0*(1.-cphi)/rf), 1 - cphi);
gam  = (gmin + gmax)/2.;
tf   = 0;  icount = 0;
gold = NaN;  told = NaN;
vrx  = NaN;  vry  = NaN;

while ~(abs(tfdes - tf) <= (.00000001*tfdes)) && icount < maxit
    top  = gm*(1. - cos(phi));
    temp = r0*cos(gam)/rf - cos(phi + gam);
    bot  = r0*cos(gam)*temp;
    v    = sqrt(top/bot);
    if xlongt > xlongm                       % <<< resolved; see header
        vrx = v*cos(pi/2. - gam + xlongm);   vry = v*sin(pi/2. - gam + xlongm);
    else
        vrx = v*cos(-pi/2. + gam + xlongm);  vry = v*sin(-pi/2. + gam + xlongm);
    end
    xlam  = r0*v*v/gm;
    top1  = tan(gam)*(1 - cos(phi)) + (1 - xlam)*sin(phi);
    bot1p = (1 - cos(phi))/(xlam*cos(gam)*cos(gam));
    bot1  = (2 - xlam)*(bot1p + cos(gam + phi)/cos(gam));
    top2  = 2*cos(gam);
    bot2  = xlam*((2/xlam - 1)^1.5);
    top3  = sqrt(2/xlam - 1);
    bot3  = cos(gam)/tan(phi/2) - sin(gam);
    temp  = (top2/bot2)*atan2(top3, bot3);
    tf    = r0*(top1/bot1 + temp)/(v*cos(gam));
    icount = icount + 1;
    if tf > tfdes                            % <<< resolved; see header
        gmax = gam;
    else
        gmin = gam;
    end
    if icount == 1
        xnext = (gmax + gmin)/2.;
    else
        xnext = gam + (gam - gold)*(tfdes - tf)/(tf - told);
        if (xnext > gmax || xnext < gmin)
            xnext = (gmax + gmin)/2.;
        end
    end
    gold = gam;  told = tf;  gam = xnext;
end
converged = abs(tfdes - tf) <= (.00000001*tfdes);
% NEAR-PARABOLIC GUARD, see the file header.  Recomputed from the RETURNED
% velocity rather than from the loop's xlam, so what gets scored is the number
% the booster is actually steered on.  ⚠ Deliberately NOT combined with
% `converged` above: that flag is TRUE on the answer this guard exists to
% catch.
lamout = r0*(vrx^2 + vry^2)/gm;
end


% =============================================================================
function T = table132(AE, GM, maxit)
%TABLE132  Listing 13.2's own driver, zarchan.txt:3942.  Reproduces Table 13.2.
%
%   PI = 3.14159 and DEGRAD = 360/(2*PI) are the LISTING'S, not MATLAB's pi.
%   Using pi here moves VRX by 5e-2 ft/s AWAY from the printed value.
PI = 3.14159;  DEGRAD = 360./(2.*PI);
XLONGM = 45./DEGRAD;   XLONGT = 90./DEGRAD;
XM = AE*cos(XLONGM);   YM = AE*sin(XLONGM);
XT = AE*cos(XLONGT);   YT = AE*sin(XLONGT);
[vrx, vry, n, conv, lamT] = lambertPZ(XM, YM, 1000., XT, YT, XLONGM, XLONGT, GM, maxit);
% Table 13.2's angle column is 57.3*gam, so gam is recovered by inverting the
% routine's own "xlongt > xlongm" branch -- vrx = v*cos(pi/2 - gam + xlongm),
% vry = v*sin(...) -- and then printed the way the book printed it.
gam = pi/2 + XLONGM - atan2(vry, vrx);
T = struct('vrx',vrx, 'vry',vry, 'iters',n, 'converged',conv, ...
           'gamdeg',57.3*gam, 'lam_out',lamT);
end


% =============================================================================
function K = structuralChecks(S, O, T132, opts, RAD, AE, GM)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never evidence that
%   this port reproduces Zarchan -- only that it agrees with itself, which is
%   what catches wiring faults.
%
%   *** .names AND .oks ARE THE CONTRACT. ***  benchSuite puts a structural row
%   on the board only when it finds BOTH, same length.  A delete experiment
%   nobody can see is the same as one that was never run.
K = struct();
names = {};  oks = [];  detail = {};

% 1  DELETE EXPERIMENT.  Freeze the Lambert solution at t = 0 and the LOOP is
%    gone while every other line stays.  This is the whole claim of Listing
%    13.3 and the only check that tests it.
if ~isempty(O)
    ratio = O.impact.miss_ft / max(S.impact.miss_ft, realmin);
    K.open_ratio = ratio;
    K.open_miss  = O.impact.miss_ft;
    names{end+1} = 'DELETE EXP: freezing Lambert at t=0 breaks the intercept';
    oks(end+1)   = ratio >= 1e4;
    detail{end+1} = sprintf(['open-loop miss %.4g ft vs closed-loop %.4g ft ' ...
        '(%.3g x); impact %.2f s vs %.2f s'], O.impact.miss_ft, ...
        S.impact.miss_ft, ratio, O.impact.t, S.impact.t);
else
    K.open_ratio = NaN;  K.open_miss = NaN;
    names{end+1} = 'DELETE EXP: freezing Lambert at t=0 breaks the intercept';
    oks(end+1)   = false;      % NOT green-by-omission.  It did not run.
    detail{end+1} = '''deleteexp'' is false: NOT RUN, and not-run is not a pass';
end

% 2  ROUND TRIP.  Launch impulsively on the Lambert solution at t = 0 and coast
%    for TF with the SAME integrator the trajectory uses.  This ties the solver
%    to the propagator; if either drifts, the closed-loop miss stops meaning
%    anything.  H_rt is coarsened to 0.05 s -- RK2 error is O(H^2) and the
%    replica measures 0.0027 ft there against a 1 ft limit, so the extra 40k
%    steps at H = 0.01 buy three decimal places nothing needs.
ANG = opts.ANGDEG/RAD;   XLONGT = opts.XLONGTDEG/RAD;
X0  = AE*cos(ANG);  Y0 = AE*sin(ANG);
XFt = AE*cos(XLONGT);  YFt = AE*sin(XLONGT);
H_rt = max(opts.H, 0.05);
[vx0, vy0, ~, cv0, lamRT] = lambertPZ(X0, Y0, opts.TF, XFt, YFt, ANG, XLONGT, GM, opts.lam_maxit);
[xe, ye, dEc] = coastArc(X0, Y0, vx0, vy0, opts.TF, H_rt, GM);
K.roundtrip_ft = AE*abs(atan2(ye, xe) - XLONGT);
K.coast_dE     = dEc;
names{end+1} = 'impulsive Lambert solution + ballistic coast lands on the aim point';
oks(end+1)   = cv0 && K.roundtrip_ft <= 1.0 && dEc <= 1e-8;
detail{end+1} = sprintf(['along-surface error after %g s coast = %.5f ft ' ...
    '(limit 1 ft, H_rt = %g s); |dE/E| = %.3e (limit 1e-8)'], ...
    opts.TF, K.roundtrip_ft, H_rt, dEc);

% 3  THRUST REALLY STOPPED.  Specific energy must RISE while the booster
%    thrusts and be CONSTANT afterwards.  This is the row that polices the
%    dead "TRST = 0": if QBOOST stopped doing the cutting off, the trajectory
%    stays smooth and energy keeps climbing past cut-off with no other symptom.
e  = S.traj.energy;  tt = S.traj.t;
pre  = tt <  S.cutoff.t;
post = tt >= S.cutoff.t;
% The powered half needs two samples to have a difference at all.  At the
% listing's samp = 10 s there are four; a caller who asks for samp = 30 s gets
% one, and that must not read as a FAILED invariant -- it is an unmeasured
% one.  The coast half is what the row is really for and is always required.
if nnz(post) >= 2
    K.e_post_drift = max(abs(e(post)/e(find(post,1)) - 1));
    if nnz(pre) >= 2
        K.e_pre_min = min(diff(e(pre)));
        risingOK = K.e_pre_min > 0;
    else
        K.e_pre_min = NaN;  risingOK = true;
    end
    ok3 = K.e_post_drift <= 1e-8 && risingOK;
else
    K.e_post_drift = NaN;  K.e_pre_min = NaN;  ok3 = false;
end
names{end+1} = 'energy rises under thrust and is constant after cut-off';
oks(end+1)   = ok3;
detail{end+1} = sprintf(['%d powered / %d coast samples; worst post-cut-off ' ...
    '|dE/E| = %.3e (limit 1e-8), smallest powered rise %+.4g ft^2/s^2'], ...
    nnz(pre), nnz(post), K.e_post_drift, K.e_pre_min);

% 4  THE FEEDBACK CLOSES.  "the desired velocity will be obtained in a feedback
%    fashion" (zarchan.txt:3961) means the velocity-to-be-gained must never
%    grow.  Cheap, and it is what a sign error on AXT/AYT breaks first --
%    that fault still produces a smooth trajectory and a plausible cut-off.
if numel(S.boost.del) >= 2
    K.del_worst_rise = max([diff(S.boost.del); -Inf]);
    ok4 = K.del_worst_rise <= 1e-9*S.boost.del(1);
else
    K.del_worst_rise = NaN;  ok4 = false;
end
names{end+1} = 'velocity-to-be-gained never increases during boost';
oks(end+1)   = ok4;
detail{end+1} = sprintf('%d guidance calls, DEL %.1f -> %.1f ft/s, worst step-to-step rise %+.3e', ...
    numel(S.boost.del), firstOr(S.boost.del), lastOr(S.boost.del), K.del_worst_rise);

% 5  Every Lambert call converged inside the cap THIS PORT added.  The book has
%    no cap; a port that quietly returned whichever iterate it was on would fly
%    a smooth, wrong trajectory.
K.nonconv = S.cutoff.nonconv + double(~T132.converged);
names{end+1} = 'every Lambert call converged inside lam_maxit';
oks(end+1)   = K.nonconv == 0 && ~S.impact.timedout;
detail{end+1} = sprintf(['%d non-convergent of %d calls; worst %d iterations ' ...
    '(cap %d); run timed out: %d'], K.nonconv, S.cutoff.nlam + 1, ...
    S.cutoff.maxit, opts.lam_maxit, S.impact.timedout);

% 6  NEAR-PARABOLIC GUARD.  See the file header for the 98,751 ft miss this
%    exists to catch.  ⚠ NOT keyed on convergence -- invariant 5 above IS the
%    convergence flag, and it reads GREEN on the wrong answer.  ⚠ Nor on
%    agreement with the package's five other transcriptions of Listing 13.2:
%    all six agree to 0.000e+00 on the failing geometry, which is why a
%    six-way comparison could not find this and an independent
%    universal-variable solver had to.
%    Covers the ~4.4k in-loop calls, invariant 2's round-trip call, and the
%    Table 13.2 replica call.
K.lambda_max = max([S.cutoff.lam_max, lamRT, T132.lam_out]);
LAMCAP = 1.999999;
names{end+1} = 'near-parabolic guard: lambda = r0|v|^2/gm < 1.999999';
oks(end+1)   = isfinite(K.lambda_max) && K.lambda_max < LAMCAP;
detail{end+1} = sprintf(['max lambda over %d loop calls + round trip + ' ...
    'Table 13.2 = %.9f (cap %.6f, margin %.3e)'], S.cutoff.nlam, ...
    K.lambda_max, LAMCAP, LAMCAP - K.lambda_max);

K.names  = names;
K.oks    = logical(oks);
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
function [x, y, dEmax] = coastArc(x, y, vx, vy, tstop, H, GM)
%COASTARC  The listing's own RK2 with the thrust removed.  Used only by the
%   structural round-trip row, so that it exercises the SAME scheme the
%   trajectory does rather than a tidier one.
T  = 0.;
e0 = (vx^2 + vy^2)/2 - GM/sqrt(x^2 + y^2);
dEmax = 0.;
XD = 0.;  YD = 0.;  X1D = 0.;  Y1D = 0.;
while T < tstop - 1e-9
    XOLD = x;  YOLD = y;  X1OLD = vx;  Y1OLD = vy;
    STEP = 1;  FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            x = x + H*XD;  y = y + H*YD;
            vx = vx + H*X1D;  vy = vy + H*Y1D;
            T = T + H;  STEP = 2;
        end
        XD = vx;  YD = vy;
        TEMBOT = (x^2 + y^2)^1.5;
        X1D = -GM*x/TEMBOT;  Y1D = -GM*y/TEMBOT;
        FLAG = 1;
    end
    x  = (XOLD  + x )/2 + .5*H*XD;    y  = (YOLD  + y )/2 + .5*H*YD;
    vx = (X1OLD + vx)/2 + .5*H*X1D;   vy = (Y1OLD + vy)/2 + .5*H*Y1D;
    ee = (vx^2 + vy^2)/2 - GM/sqrt(x^2 + y^2);
    dEmax = max(dEmax, abs(ee/e0 - 1));
end
end


% =============================================================================
function B = benchmarks(S, T132, P, opts, FTNM)
%BENCHMARKS  Computed vs published.  Every book value was grepped out of the
%   DECODED zarchan.txt and its line number is in .source.  Nothing was read
%   off a figure.  Bands come from the PROVENANCE of the book value, never
%   from what we happen to compute.  Do NOT widen one to turn a row green.
B = emptyBench();

bookCase = abs(opts.TF - 500) < 1e-9 && abs(opts.ANGDEG - 30) < 1e-9 && ...
           abs(opts.XLONGTDEG - 45) < 1e-9 && abs(opts.DELV - 20000) < 1e-9;
whyCase = sprintf(['off the listing''s own case -- the book''s numbers are ' ...
    'for ANGDEG = 30, XLONGTDEG = 45, TF = 500 s, DELV = 20000 ft/s; this ' ...
    'run used %g, %g, %g s, %g ft/s'], opts.ANGDEG, opts.XLONGTDEG, ...
    opts.TF, opts.DELV);
whyDELV = sprintf('DELV = %g ft/s, not the book''s 20000', opts.DELV);

% ---- 1.  THE HEADLINE: the guided booster arrives, on time ---------------
if bookCase
    tolT = (4*opts.H + 0.02)/opts.TF;
    B = addVal(B, 'impact time vs "reaches the target at exactly 500 s"', ...
        500.0, S.impact.t, 'text-exact', tolT, ...
        'p.289 text (zarchan.txt:4026)', 's', ...
        sprintf(['stops at the first step with ALT < 0, so impact cannot be ' ...
                 'resolved finer than H = %g s; band 4H+0.02 s.  This row ' ...
                 'does NOT police the stage gate (flipped, it still lands at ' ...
                 '500.01 s) -- row 2 does'], opts.H));
else
    B = addSkip(B, 'impact time vs "reaches the target at exactly 500 s"', ...
        '500', whyCase, 'p.289 text (zarchan.txt:4026)', 's');
end

% ---- 2.  THE HEADLINE, part two: it arrives AT THE AIM POINT -------------
if bookCase
    rangeFt = max(S.impact.downrange_nmi*FTNM, 1);
    lim = 1e-6*rangeFt;                  % 1 ppm of the range actually flown
    B = addBound(B, 'along-surface miss at impact ("Booster reaches target")', ...
        sprintf('<= 1 ppm of the flown range (%.2f ft)', lim), ...
        sprintf('%.4f ft (%.3g ppm of %.1f nmi)', S.impact.miss_ft, ...
                1e6*S.impact.miss_ft/rangeFt, S.impact.downrange_nmi), ...
        S.impact.miss_ft <= lim, ...
        'Fig. 13.8 caption + p.289 text (zarchan.txt:4026, :4031)', 'ft', ...
        ['no miss number is published, so the threshold is scaled to the ' ...
         'problem, not chosen.  Replica reference points: correct 0.034 ft, ' ...
         'stage gate flipped 119 ft, Lambert quadrant flipped 1.2e7 ft']);
else
    B = addSkip(B, 'along-surface miss at impact ("Booster reaches target")', ...
        '"reaches target"', whyCase, ...
        'Fig. 13.8 caption + p.289 text (zarchan.txt:4026, :4031)', 'ft');
end

% ---- 3/4.  the two burn times.  "about"/"nearly" => 10%. -----------------
if abs(opts.DELV - 20000) < 1e-9
    B = addVal(B, 'stage-1 burnout vs "discontinuity at about 15 s ... staging"', ...
        15.0, P.TB1, 'text-about', 0.10, ...
        'p.288 text (zarchan.txt:4016)', 's', ...
        'closed form out of the rocket equation; no integration involved');
    B = addVal(B, 'total burn vs "burnout ... completed at about 60 s"', ...
        60.0, P.TB1 + P.TB2, 'text-about', 0.10, ...
        'p.284 text (zarchan.txt:3983); restated :4027 as "nearly 60 s"', 's', ...
        'the book states the same quantity twice in the same words; scored once');
else
    B = addSkip(B, 'stage-1 burnout vs "discontinuity at about 15 s ... staging"', ...
        '15', whyDELV, 'p.288 text (zarchan.txt:4016)', 's');
    B = addSkip(B, 'total burn vs "burnout ... completed at about 60 s"', ...
        '60', whyDELV, 'p.284 text (zarchan.txt:3983)', 's');
end

% ---- 5.  thrust termination: the loop CONVERGES before burnout -----------
if bookCase && strcmp(S.cutoff.reason, 'lambert')
    B = addVal(B, 'thrust termination vs "Lambert solution reached in about 45 s"', ...
        45.0, S.cutoff.t, 'text-about', 0.10, ...
        'p.288/289 text (zarchan.txt:4016, :4027)', 's', ...
        ['the point of the section: ~45 s of cut-off against ~60 s of burn, ' ...
         'so the excess energy is thrown away by terminating thrust. ' ...
         'Listing 13.4 (GEM) is the alternative when you cannot']);
elseif bookCase
    B = addSkip(B, 'thrust termination vs "Lambert solution reached in about 45 s"', ...
        '45', sprintf(['the boost ended by "%s", not by reaching the Lambert ' ...
        'solution, so there is no convergence time to compare'], S.cutoff.reason), ...
        'p.288/289 text (zarchan.txt:4016, :4027)', 's');
else
    B = addSkip(B, 'thrust termination vs "Lambert solution reached in about 45 s"', ...
        '45', whyCase, 'p.288/289 text (zarchan.txt:4016, :4027)', 's');
end

% ---- 6-8.  TABLE 13.2.  Independent of every option; always scored. ------
% Band 1e-4: the driver's PI = 3.14159 costs ~2.9e-6 relative in VRX.
% *** The minus sign on VRX is RECONSTRUCTED from the OCR's '-' -> '2'; the
% header gives the two-row consistency argument that fixes it. ***
srcT = ['Table 13.2 row 7, p.283 (zarchan.txt:3970); same solution rounded ' ...
        'in Table 13.1 (:3913)'];
B = addVal(B, 'Lambert VRX vs Table 13.2 (45 deg -> 90 deg, tF = 1000 s)', ...
    -7695.55795, T132.vrx, 'table-9fig', 1e-4, srcT, 'ft/s', ...
    ['9 published figures, the only ones in this chapter.  SIGN ' ...
     'RECONSTRUCTED: "27695.55795" as printed contradicts its own row, ' ...
     'because v*cos(112.77 deg) is negative']);
B = addVal(B, 'Lambert VRY vs Table 13.2', 18329.31394, T132.vry, ...
    'table-9fig', 1e-4, srcT, 'ft/s', ...
    'positive as printed; no reconstruction needed');
B = addVal(B, 'Lambert iterations to converge vs Table 13.2', 7, T132.iters, ...
    'table-exact', 1e-9, ...
    'Table 13.2, p.283 (zarchan.txt:3962-3970) is 7 rows; prose :3957', '-', ...
    ['an integer, so the band is exact.  Catches a secant/bisection ' ...
     'mix-up: bisection alone needs ~30 iterations for the same answer and ' ...
     'is just as accurate, so nothing else would notice']);

% ---- 9.  SHAPE, from prose about Figs. 13.6/13.7 -------------------------
if ~isempty(S.boost.t)
    pkx = max(abs(S.boost.vx));  pky = max(abs(S.boost.vy));
    B = addShape(B, 'Vy much larger than Vx through the boost', ...
        'peak |Vy| >= 3 x peak |Vx|', ...
        sprintf('peak |Vy| = %.1f ft/s vs peak |Vx| = %.1f ft/s (%.2f x)', ...
                pky, pkx, pky/max(pkx, realmin)), ...
        pky >= 3*pkx, 'p.288 text (zarchan.txt:4016)', ...
        ['prose about a figure IS greppable; the figure is not.  The ' ...
         'near-vertical ascent is what makes this true and a wrong quadrant ' ...
         'in lambertPZ is what destroys it']);
else
    B = addSkip(B, 'Vy much larger than Vx through the boost', '|Vy| >> |Vx|', ...
        'no boost-phase samples were recorded', ...
        'p.288 text (zarchan.txt:4016)', 'ft/s');
end

% ---- INFO rows.  Reported, never scored. --------------------------------
B = addInfo(B, 'apogee and downrange', '--', ...
    sprintf('apogee %.2f nmi, downrange %.2f nmi', S.impact.apogee_nmi, ...
            S.impact.downrange_nmi), 'Fig. 13.8 -- shape only', ...
    'NOT SCORED: Fig. 13.8 publishes no numbers; the extraction made it an image link');
B = addInfo(B, 'Listing 13.1 brute force: 1084 iterations', '1084', ...
    sprintf('%d (Listing 13.2''s secant search, not 13.1''s)', T132.iters), ...
    'Table 13.1, p.280 (zarchan.txt:3913; prose :3910)', ...
    'NOT SCORED: 1084 belongs to Listing 13.1, which is not ported here');
B = addInfo(B, 'Lambert calls in the closed loop', '--', ...
    sprintf('%d calls, worst %d iterations', S.cutoff.nlam, S.cutoff.maxit), ...
    'program structure, not a published quantity', ...
    'NOT SCORED: the count is set by H, which the book ties to nothing');
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER must match the adders exactly.  This is the
%   schema benchSuite's adaptGeneric consumes and it is STRICT -- book_s for
%   book_str ERRORS rather than being accommodated.  One constructor per kind,
%   so the order cannot drift.
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
if ~isfinite(ours) || book == 0
    B(i).pct = NaN;  B(i).ok = false;  B(i).skipped = true;
    B(i).why = 'the computed value is not finite (or the book value is zero)';
    return
end
B(i).pct = 100*(ours/book - 1);
B(i).ok  = abs(ours/book - 1) <= tol;
B(i).skipped = false;  B(i).why = '';
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

function B = addShape(B, what, bookStr, oursStr, ok, source, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'shape';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'shape';  B(i).tol = NaN;
B(i).ok   = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = '-';  B(i).detail = detail;
end

function B = addInfo(B, what, bookStr, oursStr, source, why)
%ADDINFO  skipped = true keeps it out of the pass count in BOTH directions:
%   an INFO row can neither pass nor fail.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'info';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'info';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true;  B(i).why = why;
B(i).source = source;  B(i).units = '-';  B(i).detail = '';
end

function B = addSkip(B, what, bookStr, why, source, units)
%ADDSKIP  A check that COULD NOT RUN.  Never a pass: not-happened is a
%   different thing from succeeded.
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
L = {};  p = R.p;
L{end+1} = 'Zarchan Listing 13.3 -- BOOSTER SIMULATION WITH LAMBERT GUIDANCE';
L{end+1} = '   (Listing 13.2''s lambertpz is inside it, closed at every step)';
L{end+1} = '*** UNITS ARE ft, ft/s, lbf, s; g = 32.2; nmi = 6076 ft.  NOT SI. ***';
L{end+1} = sprintf(['   launch %g deg -> aim %g deg in TF = %g s   ' ...
    'DELV = %g ft/s   H = %g s   (degrees via the book''s 57.3)'], ...
    p.ANGDEG, p.XLONGTDEG, p.TF, p.DELV, p.H);
L{end+1} = sprintf('   staging: TB1 = %.4f s, TB2 = %.4f s, total %.4f s;  WTOT = %.1f lbf', ...
    p.TB1, p.TB2, p.TB1 + p.TB2, p.WTOT);
L{end+1} = sprintf('   cut-off: %s at t = %.2f s after %d Lambert calls (worst %d iterations)', ...
    R.cutoff.reason, R.cutoff.t, R.cutoff.nlam, R.cutoff.maxit);
L{end+1} = sprintf('   impact : t = %.4f s at %.6f deg -- miss %.4f ft along the surface', ...
    R.impact.t, R.impact.lon_deg, R.impact.miss_ft);
if ~isempty(R.open)
    L{end+1} = sprintf('   OPEN LOOP (Lambert frozen at t=0): impact t = %.2f s, miss %.4g ft', ...
        R.open.impact.t, R.open.impact.miss_ft);
else
    L{end+1} = '   OPEN LOOP delete experiment: NOT RUN (''deleteexp'', false)';
end
L{end+1} = '';
L{end+1} = sprintf('TRAJECTORY every %g s   (the listing''s own plot: downrange vs altitude)', p.samp);
L{end+1} = sprintf('%9s %14s %14s', 't [s]', 'downrange nmi', 'altitude nmi');
stride = max(1, ceil(numel(R.traj.t)/12));
for k = 1:stride:numel(R.traj.t)
    L{end+1} = sprintf('%9.1f %14.3f %14.3f', R.traj.t(k), ...
        R.traj.distnm(k), R.traj.altnm(k)); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = 'TABLE 13.2 REPLICA (Listing 13.2''s driver: 45 deg -> 90 deg, tF = 1000 s)';
L{end+1} = sprintf('   ours      : %d iterations   57.3*gam = %.7f deg   VRX = %.5f   VRY = %.5f', ...
    R.table132.iters, R.table132.gamdeg, R.table132.vrx, R.table132.vry);
L{end+1} = '   book row 7:  7 iterations   22.2264402 deg   VRX = -7695.55795   VRY = 18329.31394';
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-58s %s', vstr(K.oks(i)), K.names{i}, ...
        K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning NOT CHECKED.';
else
    L{end+1} = 'BENCHMARKS  (book values grepped from the DECODED zarchan.txt; see .source)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        if b.skipped
            L{end+1} = sprintf('   %-4s %-58s %s', upper(b.kind), b.what, b.why); %#ok<AGROW>
        elseif strcmp(b.kind, 'value')
            L{end+1} = sprintf('   %-4s %-58s book %-13s ours %-13s (%+.4f%%, band %g%%)', ...
                vstr(b.ok), b.what, b.book_str, b.ours_str, b.pct, 100*b.tol); %#ok<AGROW>
        else
            L{end+1} = sprintf('   %-4s %-58s %s | %s', vstr(b.ok), b.what, ...
                b.book_str, b.ours_str); %#ok<AGROW>
        end
    end
    L{end+1} = sprintf('   %d of %d rows evaluated; %d informational or skipped.', ...
        sum(ev), numel(R.bench), sum(~ev));
end
s = strjoin(L, newline);
end


% =============================================================================
function drawTraj(R)
figure;
plot(R.traj.distnm, R.traj.altnm);
grid on;
xlabel('Downrange (Nmi)');
ylabel('Altitude (Nmi)');
title('Zarchan Listing 13.3 -- booster with Lambert guidance');
end


% =============================================================================
function v = vstr(ok)
if ok
    v = 'OK';
else
    v = 'FAIL';
end
end

function y = maxOr(x)
if isempty(x)
    y = NaN;
else
    y = max(x);
end
end

function y = lastOr(x)
if isempty(x)
    y = NaN;
else
    y = x(end);
end
end

function y = firstOr(x)
if isempty(x)
    y = NaN;
else
    y = x(1);
end
end
