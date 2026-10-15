function R = predictiveGuidance(opts)
%MGS.ZARCHAN.PREDICTIVEGUIDANCE  Zarchan Listing 15.1 -- PREDICTIVE guidance
%   against a boosting target: an impulsively launched ballistic interceptor
%   versus a two-stage booster, in Earth-centred inertial coordinates.
%
%   R = mgs.zarchan.predictiveGuidance()
%   R = mgs.zarchan.predictiveGuidance('h_pred', 1.0)      coarse predictor
%   R = mgs.zarchan.predictiveGuidance('tgo_scale', 1.4)   wrong tgo (below)
%   R = mgs.zarchan.predictiveGuidance('sweep', false, 'assert', false)
%
%   Port of zarchan.txt:4493-4537 (driver :4493-4521, predictg :4522-4537).
%   lambertpz, called but not printed (:4538), is Listing 13.2 at :3943-3955;
%   transcribed here too so the file stands alone.  UNITS ARE THE BOOK'S:
%   FEET, ft/s, ft/s^2, lb, s, g = 32.2, a = 2.0926e7 ft, gm = 1.4077e16
%   ft^3/s^2, 1 n mi = 6076 ft.  Nothing is converted, so every number printed
%   lies straight against a published one.
%
%   predictg.m integrates BOTH vehicles to the fixed intercept time TF and
%   differences the predicted positions: that is the zero effort miss (:4473).
%   Its component perpendicular to the LOS drives nc = Nprime*ZEM_PLOS/tgo^2
%   (:4479).  At t = 0 it also supplies the aim point for the Lambert solve
%   that sets the missile's impulsive launch velocity; then it runs at 1 Hz
%   in flight.  No closed form anywhere -- the idea, and the exposure.
%
% ---------------------------------------------------------------------------
%   *** WHAT A WRONG tgo COSTS -- READ BEFORE QUOTING THIS FILE HERE. ***
%
%   Dividing by tgo^2 means a tgo scaled by k scales every command by 1/k^2:
%   the EFFECTIVE navigation ratio is Nprime/k^2.  At this project's measured
%   predictor scale error of 1.4-1.5, Nprime = 3 flies as 1.3-1.5 -- below the
%   value at which a PN-type loop drives its miss to zero at all.
%
%   The trap is the SIGN of the penalty.  Over-estimating tgo commands LESS
%   acceleration now, which sounds conservative and is not: the zero effort
%   miss left unremoved is still there later, when 1/tgo^2 is larger.  In a
%   replica of this arithmetic outside MATLAB, at h_pred = 0.1 s the divert
%   rises 1066 -> 1480 -> 1649 ft/s for k = 1.0, 1.4, 1.5; at h_pred = 1 s,
%   1818 -> 2882 -> 3319 (+59%, +83%).  The MISS does not move monotonically
%   with k (21 ft at 1.4, 53 at 1.5, against 15 at 1.0), so a run judged on
%   miss alone reads as noise while the fuel bill nearly doubles.  And the
%   nominal case cannot see any of it: with a perfect predictor ZEM ~ 4e-5 ft,
%   so nc ~ 0 for EVERY k and the miss is 4.95e-5 ft at k = 1.4 and 1.5
%   against 4.94e-5 at k = 1.  A tgo error is invisible until there is a real
%   ZEM to remove.  'tgo_scale' exists so it can be exercised, not discovered.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  This book's OCR writes "<" as "," and ">" as ".".
%   Every comparison is resolved that way and each is forced by shape:
%   "if T,TB1"/"elseif T,(TB1+TB2)" -> T < (WGT = -WP1*T/TB1+WTOT FALLS across
%   [0,TB1], so it is the early branch, stage 1 burning); "while VC.=0." ->
%   VC >= 0, ending at closest approach; "if TGO . .1" -> TGO > 0.1 (note the
%   space), H = .01 far out and .0001 in the endgame, never the reverse;
%   "if S.=.99999" -> S >= .99999, a 1-Hz update, since S is reset to 0 there;
%   "if XNC.966." -> XNC > 966, the clamp.  ONE IS AMBIGUOUS: "if tgo.1" in
%   predictg may be "tgo > 1" or "tgo > .1" -- the decimal may have been
%   eaten.  Taken as transcribed (> 1).  The readings differ only for tgo in
%   (0.1, 1], which here is ONE guidance update, and there the command is
%   already on the clamp: the replica gives a bit-identical run either way.
%
%   TWO BOOK BUGS, both the missing-minus class, both in one paragraph: :4474
%   prints "ZEMx = xTF  xMF" (must be xTF - xMF) and :4477 prints "ZEMPLOS =
%   ZEMx sin l + ZEMy cos l" against the code's -ZEM1*sin+ZEM2*cos at :4515.
%   The CODE is right -- the same chapter's aPLOS = yddT*cos - xddT*sin
%   (:4321) kept its minus and is the same projection.  The code's sign is
%   ported; structural check 4 INJECTS the prose's sign to price it.
%
%   Deliberate deviations, each because the book's form fails SILENTLY:
%   acos(CBETA) is clamped to [-1,1] (unclamped, a rounding-level 1+2e-16
%   returns a COMPLEX downrange with no error); lambertpz and predictg get
%   iteration caps that ERROR (the book has none and a stuck Lambert hangs);
%   "save datfil.txt output /ascii" is the book's usual -ascii typo, so
%   nothing is written to disk and the arrays come back in R.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS -- grepped from zarchan_decoded.txt, line numbers in .source.
%   Nothing off a figure: Figs. 15.5/15.6 survived extraction only as image
%   links, so their curves are a declared SKIP row.
%     1-4  The booster on line 1, against Table 12.1 (:3691, :3692, :3697,
%          :3699), printed at AMAX = 10 g: Wtot 6169 lb, Wp1 3474 lb, tB1
%          32.2 s, tB2 106 s.  Weights are AMAX-independent, burn times are
%          not, so the table is evaluated at 10 g beside the listing's 20 g.
%     5    tB1+tB2 = 69.1 s at AMAX = 20 (:4322) -- THIS listing's booster.
%     6    "virtually requires zero acceleration" (:4487): peak |nc| over the
%          guided phase <= 0.01 g.  The replica gives ~1e-8 g, so the bound is
%          not doing quiet work.
%     7-8  "the miss distance and lateral divert increase as the integration
%          step size grows" (:4540), h = 0.01, 0.1, 1 s -- two ordering rows,
%          the sharpest thing this chapter publishes.
%     9-10 At h = 1 s the divert must exceed augmented PN's ~1500 ft/s (:4540
%          claims it, :4411 has the number) and stay below PN's (:4544 claims
%          it; :4411 says ~5200 and :4341 says ~5300 for the SAME figure --
%          the book contradicts itself.  5200 is scored, being tighter; both
%          are cited).  Together they bracket one number.
%     11   "the missile acceleration response would be unstable" at h = 1 s
%          (:4487), scored as an oscillation signature and not as a big
%          number: the command must REVERSE SIGN above 1 g -- true at 1 s,
%          false at 0.01 and 0.1 s.  A PROXY, not a proof of divergence.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED FROM A REPLICA OUTSIDE MATLAB, NOT
%   MEASURED HERE. ***  This file has never been executed; MATLAB is the
%   caller's.  Stated so that a first run meets an expectation:
%     Lambert 12 iters, |V| = 34382.2 ft/s, gam = 8.648 deg, lambda = 1.757
%     h_pred 0.01 s: miss 4.94e-5 ft, divert 8.8e-7 ft/s, peak |nc| 1e-8 g
%     h_pred 0.1  s: miss 14.72   ft, divert 1066.0  ft/s, peak 30 g (clamp)
%     h_pred 1.0  s: miss 444.0   ft, divert 1817.5  ft/s, peak 12.6 g, the
%                    command reversing +4.85 -> -12.61 g over the last 2 s
%     intercept altitude 47.199 n mi; guidance OFF misses by 5.1e-5 ft
%
%   *** THE 30-g GHOST. ***  The LAST recorded sample of the nominal run has
%   tgo = -1.3e-9 (computed after closest approach, VC already negative), so
%   nc is pinned on the 966 ft/s^2 = 30 g clamp.  It is never integrated into
%   the divert -- DELVD uses the PREVIOUS command -- and that run's divert is
%   8.8e-7 ft/s.  A naive max(abs(R.ncg)) reports 30 g and makes "virtually
%   zero acceleration" read as false.  R.valid (tgo > 0) masks it, benchmark 6
%   uses the mask, and structural check 5 asserts that at most one sample is
%   masked and that it is the last.
%
%   OUTPUTS: R.miss_ft and R.divert_fps are the two numbers the listing echoes
%   ("RTM DELV"); R.t .distnmT .altnmT .distnmM .altnmM .ncg .delv .atplosg
%   are its eight recorded arrays at 1 Hz in its own order; R.tgo .zemplos
%   .valid are this port's added diagnostics; also R.lambert, R.design and
%   R.design10 (AMAX and 10 g), R.sweep, R.guid_off (the guidance-OFF control
%   run), R.bench, R.selfcheck, R.pass, R.verdict, R.p, R.summary, R.units.
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
%   ⚠ WHY INVARIANT 1 WAS NOT ENOUGH.  It already carried "E.lambert.lambda < 2",
%   the ESCAPE condition.  The failing solve halts at lambda = 1.999999919,
%   which passes "< 2" comfortably -- 2 is the value the bracket is DEFINED by,
%   so the iterate can never exceed it and that term could only ever fire on a
%   NaN.  1.999999 is the cap that separates the two populations; "< 2" is not.
%   Invariant 1 is left as it was and the guard is a row of its own, so the two
%   claims stay distinguishable on the board.
%
%   THIS PORT'S REACH -- MEASURED, AND THE GUARD BITES.
%     XLONGMDEG = 76, TF = 129.50 s -> lambda = 1.999999568 in 37 passes,
%     lambert.converged = 1, and the guard goes RED.  TF = 129.48 and 129.52
%     are honest refusals (the routine raises), so the band is ~0.02 s wide in
%     flight time at this longitude -- it was found by scanning TF at 0.02 s
%     resolution, not by luck.
%     ⚠ TO REPRODUCE IT, PASS 'sweep', false.  The h_pred sweep re-solves the
%     same transfer with a coarser prediction step and that call lands OUTSIDE
%     the band, exhausting lam_maxit and RAISING before any row is scored --
%     so at the defaults this geometry throws rather than printing red.  The
%     guard is what catches it on the flown solve; verified 2026-08-24:
%       'sweep', false  -> 3 of 5 rows red, guard among them, lambda 1.999999568
%       'sweep', true   -> error('lambertpz did not converge in 200 iterations')
%   ⭐ THIS IS THE ONE PORT THAT WAS ALREADY PROTECTED, and by accident of
%   design rather than by the Lambert row: invariant 1 re-measures |ZEM| after
%   the impulse and reads 2.56e+03 ft there instead of ~0, so it goes red too.
%   The other five have no such downstream witness.  The guard is still the
%   direct test -- invariant 1 fails on a CONSEQUENCE and only because this
%   listing happens to run a second prediction.
%   ⚠ MARGIN AT THE BOOK'S OWN CASE IS THE THINNEST OF THE SIX:
%   lambda_out = 1.757285959, i.e. 0.243 from the cap.  The others sit at
%   0.26-0.70.  This port flies closest to the escape boundary by design.
% ---------------------------------------------------------------------------
%
%   See also MGS.ZARCHAN.THRUSTWEIGHT (Listing 12.1 -- the same booster),
%   MGS.ZARCHAN.LAMBERTEFFICIENT (13.2), MGS.ZARCHAN.BALLISTICTARGETSIM.

arguments
    % Listing 15.1 line 1 and the T = 0 block, verbatim.
    opts.TF         (1,1) double {mustBeFinite, mustBePositive} = 50.0
    opts.XNP        (1,1) double {mustBeFinite, mustBeNonnegative} = 3.0
    opts.PREDERR    (1,1) double {mustBeFinite} = 0.0      % ft, added to YTF
    opts.XLONGMDEG  (1,1) double {mustBeFinite} = 85.0
    opts.XLONGTDEG  (1,1) double {mustBeFinite} = 90.0
    opts.XKICKDEG   (1,1) double {mustBeFinite} = 80.0
    opts.LEFT       (1,1) double {mustBeMember(opts.LEFT, [0 1])} = 0
    opts.XNCLIM     (1,1) double {mustBeFinite, mustBePositive} = 966.0  % ft/s^2
    % AMAX1 = AMAX2 = 20 g in this listing.  Table 12.1 is at 10 g and is
    % evaluated separately; this option moves the FLOWN booster only.
    opts.AMAX       (1,1) double {mustBeFinite, mustBePositive} = 20.0
    % The h inside predictg.  0.01 s is the listing's literal.  The book's own
    % experiment replaces that literal with 0.1 and 1 s (:4540).
    opts.h_pred     (1,1) double {mustBeFinite, mustBePositive} = 0.01
    opts.h_sweep    (1,:) double {mustBeFinite, mustBePositive} = [0.01 0.1 1.0]
    % Multiplies the tgo used in the GUIDANCE GAIN only (not the prediction
    % horizon, which is the fixed TF).  1.0 is the listing.  See the header.
    opts.tgo_scale  (1,1) double {mustBeFinite, mustBePositive} = 1.0
    opts.lam_maxit  (1,1) double {mustBeFinite, mustBePositive} = 200
    % Runaway guards.  The book has none; a mis-edited rate can loop forever.
    opts.max_steps  (1,1) double {mustBeFinite, mustBePositive} = 2e6
    opts.max_pred   (1,1) double {mustBeFinite, mustBePositive} = 2e6
    % THE ONLY SWITCHES.  Not nargin: with an arguments block declaring only
    % name-value options MATLAB reports nargin = 0 however many were passed,
    % so an "if nargin == 0" mode here would be a mode that never fires.
    opts.sweep      (1,1) logical = true
    opts.benchmark  (1,1) logical = true
    opts.render     (1,1) logical = true
    opts.plot       (1,1) logical = false
    opts.assert     (1,1) logical = true
end

A  = 2.0926e7;      % ft, Earth radius
GM = 1.4077e16;     % ft^3/s^2

% ---- the booster, from line 1 of the listing.  Weights are AMAX-independent;
%      thrust and burn time are proportional to AMAX.
D   = boosterDesign(opts.AMAX);
D10 = boosterDesign(10.0);          % Table 12.1's case, for benchmarks 1-4

P = struct('TF', opts.TF, 'XNP', opts.XNP, 'PREDERR', opts.PREDERR, ...
    'XLONGMDEG', opts.XLONGMDEG, 'XLONGTDEG', opts.XLONGTDEG, ...
    'XKICKDEG', opts.XKICKDEG, 'LEFT', opts.LEFT, 'XNCLIM', opts.XNCLIM, ...
    'tgo_scale', opts.tgo_scale, 'A', A, 'GM', GM, ...
    'lam_maxit', opts.lam_maxit, 'max_steps', opts.max_steps, ...
    'max_pred', opts.max_pred);

% =========================================================================
%   THE NOMINAL RUN.  zsign = -1 is the LISTING's ZEM_PLOS sign (:4515);
%   +1 is the prose's (:4477) and is used only by structural check 4.
% =========================================================================
E = engagement(D, P, opts.h_pred, true, -1.0);

% The book's own control experiment: "A nominal case was run with the guidance
% system turned off to ensure that the missile and booster were on a collision
% triangle" (:4323, restated for this chapter at :4487).  Cheap -- with the
% guidance off predictg is never called inside the flight.
Eoff = engagement(D, P, opts.h_pred, false, -1.0);

% ---- the step-size study, :4540.
SW = struct('h', {}, 'miss_ft', {}, 'divert_fps', {}, 'peak_nc_g', {}, ...
            'nc_max_g', {}, 'nc_min_g', {}, 't_end', {});
if opts.sweep
    for k = 1:numel(opts.h_sweep)
        hk = opts.h_sweep(k);
        if hk == opts.h_pred
            Ek = E;                              % do not pay for it twice
        else
            Ek = engagement(D, P, hk, true, -1.0);
        end
        SW(k).h          = hk;
        SW(k).miss_ft    = Ek.miss_ft;
        SW(k).divert_fps = Ek.divert_fps;
        SW(k).peak_nc_g  = Ek.peak_nc_g;
        SW(k).nc_max_g   = Ek.nc_max_g;
        SW(k).nc_min_g   = Ek.nc_min_g;
        SW(k).t_end      = Ek.t_end;
    end
end

% ---- FAULT INJECTION for structural check 4: the prose's ZEM sign.  Run at
%      the COARSEST sweep step, where there is a real ZEM for the sign to be
%      wrong about; on the nominal case ZEM ~ 4e-5 ft and the sign is invisible.
if opts.sweep && ~isempty(opts.h_sweep)
    hflip = max(opts.h_sweep);
    Eref  = engagement(D, P, hflip, true, -1.0);
    Eflip = engagement(D, P, hflip, true, +1.0);
    flipInfo = struct('h', hflip, 'miss_ref', Eref.miss_ft, ...
                      'miss_flip', Eflip.miss_ft, 'ran', true);
else
    flipInfo = struct('h', NaN, 'miss_ref', NaN, 'miss_flip', NaN, 'ran', false);
end

R = struct();
R.miss_ft    = E.miss_ft;
R.divert_fps = E.divert_fps;
R.t = E.t;  R.distnmT = E.distnmT;  R.altnmT = E.altnmT;
R.distnmM = E.distnmM;  R.altnmM = E.altnmM;
R.ncg = E.ncg;  R.delv = E.delv;  R.atplosg = E.atplosg;
R.tgo = E.tgo;  R.zemplos = E.zemplos;  R.valid = E.valid;
R.peak_nc_g  = E.peak_nc_g;
R.t_end      = E.t_end;
R.lambert    = E.lambert;
R.zem0_ft    = E.zem0_ft;
R.design     = D;
R.design10   = D10;
R.sweep      = SW;
R.guid_off   = struct('miss_ft', Eoff.miss_ft, 'divert_fps', Eoff.divert_fps, ...
                      't_end', Eoff.t_end);
R.flip       = flipInfo;
R.units = ['ft, ft/s, ft/s^2, lb, s; g = 32.2 ft/s^2; a = 2.0926e7 ft, ' ...
           'gm = 1.4077e16 ft^3/s^2; 1 n mi = 6076 ft'];
R.p = struct('TF', opts.TF, 'XNP', opts.XNP, 'PREDERR', opts.PREDERR, ...
    'XLONGMDEG', opts.XLONGMDEG, 'XLONGTDEG', opts.XLONGTDEG, ...
    'XKICKDEG', opts.XKICKDEG, 'LEFT', opts.LEFT, 'AMAX', opts.AMAX, ...
    'XNCLIM', opts.XNCLIM, 'h_pred', opts.h_pred, 'h_sweep', opts.h_sweep, ...
    'tgo_scale', opts.tgo_scale, 'n_steps', E.n_steps, ...
    'n_pred_calls', E.n_pred_calls, 'n_pred_steps', E.n_pred_steps, ...
    'units', R.units);

R.selfcheck = structuralChecks(E, Eoff, D, flipInfo);

if opts.benchmark
    R.bench = benchmarks(E, D, D10, SW, opts);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine" -- that is the shape of green light this repo has been burned
    % by more than once.
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R); end

% ---- adjudication.  The verdict is FORMED either way; 'assert' chooses only
%      the reaction.  With assert false the result comes back COMPLETE, with a
%      warning -- benchSuite runs every port that way so that one red row
%      cannot hide the board, and a port that refuses to return is a port
%      nobody can diagnose.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = ['not one book value could be evaluated at these settings.  An ' ...
           'unvalidated port must not be handed back silently.'];
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
        error('mgs:zarchan:predictiveGuidance:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:predictiveGuidance:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function D = boosterDesign(AMAX)
%BOOSTERDESIGN  The staging arithmetic on line 1 of Listing 15.1 (:4493), which
%   is line 1 of Listing 12.1 (:3711-3713) and of Listing 14.2 (:4343) unchanged.
%
%   Every constant is an argument or a literal HERE.  MATLAB local functions
%   cannot see the caller's workspace, and a previous wave in this package lost
%   a run to exactly that.
XISP1 = 250.;  XISP2 = 250.;  XMF1 = .85;  XMF2 = .85;
WPAY  = 100.;  DELV  = 20000.;
DELV1 = .3333*DELV;   DELV2 = .6667*DELV;
AMAX1 = AMAX;         AMAX2 = AMAX;

TOP2 = WPAY*(exp(DELV2/(XISP2*32.2))-1.);
BOT2 = 1/XMF2-((1.-XMF2)/XMF2)*exp(DELV2/(XISP2*32.2));
WP2  = TOP2/BOT2;
WS2  = WP2*(1-XMF2)/XMF2;
WTOT2 = WP2+WS2+WPAY;
TRST2 = AMAX2*(WPAY+WS2);
TB2   = XISP2*WP2/TRST2;

TOP1 = WTOT2*(exp(DELV1/(XISP1*32.2))-1.);
BOT1 = 1/XMF1-((1.-XMF1)/XMF1)*exp(DELV1/(XISP1*32.2));
WP1  = TOP1/BOT1;
WS1  = WP1*(1-XMF1)/XMF1;
WTOT = WP1+WS1+WTOT2;
TRST1 = AMAX1*(WTOT2+WS1);
TB1   = XISP1*WP1/TRST1;

D = struct('AMAX', AMAX, 'WPAY', WPAY, 'WP1', WP1, 'WS1', WS1, 'WP2', WP2, ...
           'WS2', WS2, 'WTOT', WTOT, 'WTOT2', WTOT2, 'TRST1', TRST1, ...
           'TRST2', TRST2, 'TB1', TB1, 'TB2', TB2);
end


% =============================================================================
function [WGT, TRST] = boosterWeight(T, D)
%BOOSTERWEIGHT  The three-branch thrust/weight schedule, :4506-4509 (driver)
%   and :4529-4532 (predictg), which are the same lines in different case.
%
%   Both comparisons are OCR "," = "<" -- see the header.  Note the STAGING
%   DISCONTINUITY this produces: WGT drops by WS1 at T = TB1 and by WS2 at
%   T = TB1+TB2, which is the structural weight being jettisoned and is the
%   book's own intent ("the sharp weight drops ... represent staging events",
%   :3685).  Structural check 2 pins both drops.
if T < D.TB1
    WGT  = -D.WP1*T/D.TB1 + D.WTOT;
    TRST = D.TRST1;
elseif T < (D.TB1 + D.TB2)
    WGT  = -D.WP2*T/D.TB2 + D.WTOT2 + D.WP2*D.TB1/D.TB2;
    TRST = D.TRST2;
else
    WGT  = D.WPAY;
    TRST = 0.;
end
end


% =============================================================================
function E = engagement(D, P, hpred, guided, zsign)
%ENGAGEMENT  Listing 15.1's driver, zarchan.txt:4493-4520.
%
%   guided = false is the book's collision-triangle control run (:4323): the
%   guidance block is skipped entirely, so predictg is never called in flight
%   and XNC stays 0.  zsign selects the ZEM_PLOS sign; -1 is the listing's.

A = P.A;  GM = P.GM;
DEGRAD = 360./(2.*pi);
XLONGM = P.XLONGMDEG/DEGRAD;
XLONGT = P.XLONGTDEG/DEGRAD;
ALTT = 0.;  ALTM = 0.;
XM = (A+ALTM)*cos(XLONGM);  YM = (A+ALTM)*sin(XLONGM);
XT = (A+ALTT)*cos(XLONGT);  YT = (A+ALTT)*sin(XLONGT);
XFIRSTT = XT;  YFIRSTT = YT;

% The target's initial velocity is a UNIT vector, not a speed: the booster
% starts essentially at rest and the kick angle only sets the DIRECTION of the
% gravity turn.  VEL = sqrt(X1T^2+Y1T^2) = 1 on the first derivative pass, so
% AXT = AT*X1T/VEL is well defined; it would divide by zero at a true rest.
if P.LEFT == 1
    X1T = cos(pi/2. - P.XKICKDEG/DEGRAD + XLONGT);
    Y1T = sin(pi/2. - P.XKICKDEG/DEGRAD + XLONGT);
else
    X1T = cos(-pi/2. + P.XKICKDEG/DEGRAD + XLONGT);
    Y1T = sin(-pi/2. + P.XKICKDEG/DEGRAD + XLONGT);
end

T = 0.;  TF = P.TF;  TGO = TF - T;  X1M = 0.;  Y1M = 0.;  S = 0.;

% First prediction: the aim point.  It is called with the missile AT REST
% (X1M = Y1M = 0) and its ZEM output is discarded -- only XTF, YTF are used.
[XTF, YTF, ~, ~, np] = predictg(T, TF, XT, YT, X1T, Y1T, XM, YM, X1M, Y1M, ...
                                TGO, hpred, D, A, GM, P.max_pred);
nPredSteps = np;  nPredCalls = 1;
YTF = YTF + P.PREDERR;

LAM = lambertpz(XM, YM, TF, XTF, YTF, XLONGM, XLONGT, A, GM, P.lam_maxit);
X1M = LAM.vrx;  Y1M = LAM.vry;

% The zero effort miss with the Lambert velocity installed: the collision
% triangle, MEASURED rather than assumed (structural check 1).  Not in the
% listing; it costs one extra prediction and it is the only thing that would
% notice a Lambert converging onto the wrong branch.
[~, ~, z1, z2, np] = predictg(T, TF, XT, YT, X1T, Y1T, XM, YM, X1M, Y1M, ...
                              TGO, hpred, D, A, GM, P.max_pred);
zem0 = sqrt(z1^2 + z2^2);
nPredSteps = nPredSteps + np;  nPredCalls = nPredCalls + 1;

RTM1 = XT-XM;  RTM2 = YT-YM;  RTM = sqrt(RTM1^2+RTM2^2);
VTM1 = X1T-X1M;  VTM2 = Y1T-Y1M;
VC = -(RTM1*VTM1+RTM2*VTM2)/RTM;
DELV = 0.;  XNC = 0.;

% Derivatives pre-initialised so the FLAG==1 branch cannot read a leftover.
XDT = 0.;  YDT = 0.;  X1DT = 0.;  Y1DT = 0.;
XDM = 0.;  YDM = 0.;  X1DM = 0.;  Y1DM = 0.;  DELVD = 0.;
XLAM = atan2(RTM2, RTM1);  ATPLOS = 0.;
ZEMPLOS = 0.;  %#ok<NASGU> defensive: reassigned before every read of it

nmax = ceil(TF) + 8;
ArrT = zeros(nmax,1);  ArrDT = zeros(nmax,1);  ArrAT = zeros(nmax,1);
ArrDM = zeros(nmax,1); ArrAM = zeros(nmax,1);  ArrNC = zeros(nmax,1);
ArrDV = zeros(nmax,1); ArrAP = zeros(nmax,1);  ArrTG = zeros(nmax,1);
ArrZP = zeros(nmax,1);
count = 0;  nstep = 0;

while VC >= 0.
    TGO = RTM/VC;
    if TGO > .1
        H = .01;
    else
        H = .0001;
    end
    XOLDT = XT;  YOLDT = YT;  X1OLDT = X1T;  Y1OLDT = Y1T;
    XOLDM = XM;  YOLDM = YM;  X1OLDM = X1M;  Y1OLDM = Y1M;
    DELVOLD = DELV;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  Pass 1 evaluates the derivatives at
    % the current state; pass 2 takes a full Euler step, advances T by H, and
    % re-evaluates there.  T advances INSIDE this loop, exactly once, which is
    % what makes the second evaluation happen at T+H.  Move that line out and
    % the scheme quietly degrades to first order while still running smoothly.
    while STEP <= 1
        if FLAG == 1
            XT = XT+H*XDT;   YT = YT+H*YDT;
            X1T = X1T+H*X1DT;  Y1T = Y1T+H*Y1DT;
            XM = XM+H*XDM;   YM = YM+H*YDM;
            X1M = X1M+H*X1DM;  Y1M = Y1M+H*Y1DM;
            DELV = DELV+H*DELVD;
            T = T+H;
            STEP = 2;
        end
        [WGT, TRST] = boosterWeight(T, D);
        AT   = 32.2*TRST/WGT;
        VEL  = sqrt(X1T^2+Y1T^2);
        AXT  = AT*X1T/VEL;
        AYT  = AT*Y1T/VEL;
        TEMBOTT = (XT^2+YT^2)^1.5;
        X1DT = -GM*XT/TEMBOTT + AXT;
        Y1DT = -GM*YT/TEMBOTT + AYT;
        XDT  = X1T;  YDT = Y1T;
        RTM1 = XT-XM;  RTM2 = YT-YM;  RTM = sqrt(RTM1^2+RTM2^2);
        VTM1 = X1T-X1M;  VTM2 = Y1T-Y1M;
        VC   = -(RTM1*VTM1+RTM2*VTM2)/RTM;
        TGO  = RTM/VC;
        XLAM = atan2(RTM2, RTM1);
        XLAMD = (RTM1*VTM2-RTM2*VTM1)/(RTM*RTM); %#ok<NASGU>
        % XLAMD is DEAD in this listing -- predictive guidance never uses the
        % LOS rate.  It is the line where Listing 14.2's proportional
        % navigation command was formed.  Kept so that the diff against 14.2
        % is one guidance block and not a scatter of deletions.
        ATPLOS  = AYT*cos(XLAM) - AXT*sin(XLAM);
        AXMGUID = -XNC*sin(XLAM);
        AYMGUID =  XNC*cos(XLAM);
        DELVD   = abs(XNC);
        TEMBOTM = (XM^2+YM^2)^1.5;
        X1DM = -GM*XM/TEMBOTM + AXMGUID;
        Y1DM = -GM*YM/TEMBOTM + AYMGUID;
        XDM  = X1M;  YDM = Y1M;
        FLAG = 1;
    end
    % The listing writes "FLAG = 0" here as well.  Dropped: the outer loop
    % re-initialises FLAG on its next pass, so the assignment cannot be read.
    % Recorded here rather than deleted silently, so that nobody re-adds it
    % thinking a state machine was lost.

    XT  = (XOLDT+XT)/2 + .5*H*XDT;    YT  = (YOLDT+YT)/2 + .5*H*YDT;
    X1T = (X1OLDT+X1T)/2 + .5*H*X1DT; Y1T = (Y1OLDT+Y1T)/2 + .5*H*Y1DT;
    XM  = (XOLDM+XM)/2 + .5*H*XDM;    YM  = (YOLDM+YM)/2 + .5*H*YDM;
    X1M = (X1OLDM+X1M)/2 + .5*H*X1DM; Y1M = (Y1OLDM+Y1M)/2 + .5*H*Y1DM;
    DELV = (DELVOLD+DELV)/2. + .5*H*DELVD;

    ALTT = sqrt(XT^2+YT^2) - A;
    ALTM = sqrt(XM^2+YM^2) - A;
    S = S + H;
    nstep = nstep + 1;

    if S >= .99999
        S = 0.;
        if guided
            [XTF, YTF, ZEM1, ZEM2, np] = predictg(T, TF, XT, YT, X1T, Y1T, ...
                XM, YM, X1M, Y1M, TGO, hpred, D, A, GM, P.max_pred); %#ok<ASGLU>
            nPredSteps = nPredSteps + np;  nPredCalls = nPredCalls + 1;
            % :4515.  zsign = -1 is the CODE's sign; the prose at :4477 prints
            % +1 and is missing a minus -- see the header.
            ZEMPLOS = zsign*ZEM1*sin(XLAM) + ZEM2*cos(XLAM);
            TGOG = P.tgo_scale*TGO;
            XNC  = P.XNP*ZEMPLOS/(TGOG*TGOG);
            if XNC >  P.XNCLIM, XNC =  P.XNCLIM; end
            if XNC < -P.XNCLIM, XNC = -P.XNCLIM; end
        else
            ZEMPLOS = 0.;  XNC = 0.;
        end

        ALTNMT = ALTT/6076.;
        RF = sqrt(XFIRSTT^2 + YFIRSTT^2);
        R1 = sqrt(XT^2+YT^2);
        CBETA = (XT*XFIRSTT + YT*YFIRSTT)/(R1*RF);
        BETA  = acos(min(1., max(-1., CBETA)));   % clamped: see header
        DISTNMT = A*BETA/6076.;
        ALTNMM = ALTM/6076.;
        XNCG = XNC/32.2;
        R1 = sqrt(XM^2+YM^2);
        CBETA = (XM*XFIRSTT + YM*YFIRSTT)/(R1*RF);
        BETA  = acos(min(1., max(-1., CBETA)));
        DISTNMM = A*BETA/6076.;
        ATPLOSG = ATPLOS/32.2;

        count = count + 1;
        if count > nmax
            error('mgs:zarchan:predictiveGuidance:record', ...
                'more than %d guidance updates at TF = %g s.', nmax, TF);
        end
        ArrT(count)  = T;       ArrDT(count) = DISTNMT;  ArrAT(count) = ALTNMT;
        ArrDM(count) = DISTNMM; ArrAM(count) = ALTNMM;   ArrNC(count) = XNCG;
        ArrDV(count) = DELV;    ArrAP(count) = ATPLOSG;  ArrTG(count) = TGO;
        ArrZP(count) = ZEMPLOS;
    end

    if nstep > P.max_steps
        error('mgs:zarchan:predictiveGuidance:runaway', ...
            ['the engagement loop passed %d steps without VC going ' ...
             'negative.  The book has no such guard and would run forever.'], ...
            P.max_steps);
    end
end

if count < 2
    % Refused, not warned about.  An almost-empty record returned quietly
    % looks like a converged result, and every shape row below would then be
    % adjudicating one number.
    error('mgs:zarchan:predictiveGuidance:empty', ...
        ['the engagement recorded %d guidance update(s) before VC went ' ...
         'negative (TF = %g s).  At least two are needed for any ordering ' ...
         'or peak to mean anything.'], count, TF);
end

E = struct();
E.miss_ft    = RTM;      % the listing echoes "RTM DELV" at :4520
E.divert_fps = DELV;
E.t = ArrT(1:count);  E.distnmT = ArrDT(1:count);  E.altnmT = ArrAT(1:count);
E.distnmM = ArrDM(1:count);  E.altnmM = ArrAM(1:count);
E.ncg = ArrNC(1:count);  E.delv = ArrDV(1:count);  E.atplosg = ArrAP(1:count);
E.tgo = ArrTG(1:count);  E.zemplos = ArrZP(1:count);
% THE 30-g GHOST, see the header: a sample recorded after closest approach has
% tgo <= 0 and carries a command that was never flown.
E.valid = E.tgo > 0;
if any(E.valid)
    E.peak_nc_g = max(abs(E.ncg(E.valid)));
    E.nc_max_g  = max(E.ncg(E.valid));
    E.nc_min_g  = min(E.ncg(E.valid));
else
    E.peak_nc_g = NaN;  E.nc_max_g = NaN;  E.nc_min_g = NaN;
end
E.t_end  = T;
E.zem0_ft = zem0;
E.lambert = LAM;
E.n_steps = nstep;
E.n_pred_calls = nPredCalls;
E.n_pred_steps = nPredSteps;
end


% =============================================================================
function [xtf, ytf, zem1, zem2, nstep] = predictg(tdum, tf, xdum, ydum, ...
    x1dum, y1dum, xmdum, ymdum, x1mdum, y1mdum, tgo, hnom, D, a, gm, maxstep) %#ok<INUSL>
%PREDICTG  zarchan.txt:4522-4537.  Propagates BOTH vehicles to the fixed
%   intercept time tf and returns the target's predicted position and the zero
%   effort miss.  The missile is propagated BALLISTICALLY (no guidance term) --
%   that is what makes the difference a ZERO EFFORT miss.
%
%   "a" is unused here, exactly as in the book's own routine, which sets it on
%   :4526 and never reads it.  Kept so the transcription lines up.
%
%   hnom replaces the listing's literal .01; the tgo <= 1 branch takes a single
%   step to the end.  That branch is why an iteration cap is needed: a tiny tgo
%   with a non-tiny tf - t would otherwise be a very long loop with no symptom.
if tgo > 1
    h = hnom;
else
    h = tgo;
end
if h <= 0
    % Cannot fire in the book's own case: the driver only calls this while
    % VC >= 0, and at the one sample where tgo goes negative tf - t is already
    % 0, so the loop below does not execute.  Guarded anyway -- h <= 0 with
    % t < tf is an infinite loop, not an error.
    h = tf - tdum;
end

t  = tdum;
x  = xdum;   y  = ydum;   x1 = x1dum;  y1 = y1dum;
xm = xmdum;  ym = ymdum;  x1m = x1mdum;  y1m = y1mdum;
xd = 0.;  yd = 0.;  x1d = 0.;  y1d = 0.;
xdm = 0.; ydm = 0.; x1dm = 0.; y1dm = 0.;
nstep = 0;

while t <= (tf - .00001)
    if h <= 0
        break
    end
    xold = x;  yold = y;  x1old = x1;  y1old = y1;
    xoldm = xm;  yoldm = ym;  x1oldm = x1m;  y1oldm = y1m;
    step = 1;  flag = 0;
    while step <= 1
        if flag == 1
            x = x+h*xd;    y = y+h*yd;
            x1 = x1+h*x1d; y1 = y1+h*y1d;
            xm = xm+h*xdm; ym = ym+h*ydm;
            x1m = x1m+h*x1dm;  y1m = y1m+h*y1dm;
            t = t+h;
            step = 2;
        end
        [wgt, trst] = boosterWeight(t, D);
        at  = 32.2*trst/wgt;
        vel = sqrt(x1^2+y1^2);
        axt = at*x1/vel;
        ayt = at*y1/vel;
        tembott = (x^2+y^2)^1.5;
        x1d = -gm*x/tembott + axt;
        y1d = -gm*y/tembott + ayt;
        xd = x1;  yd = y1;
        tembotm = (xm^2+ym^2)^1.5;
        x1dm = -gm*xm/tembotm;      % NO guidance term: zero effort
        y1dm = -gm*ym/tembotm;
        xdm = x1m;  ydm = y1m;
        flag = 1;
    end
    x  = (xold+x)/2 + .5*h*xd;     y  = (yold+y)/2 + .5*h*yd;
    x1 = (x1old+x1)/2 + .5*h*x1d;  y1 = (y1old+y1)/2 + .5*h*y1d;
    xm = (xoldm+xm)/2 + .5*h*xdm;  ym = (yoldm+ym)/2 + .5*h*ydm;
    x1m = (x1oldm+x1m)/2 + .5*h*x1dm;  y1m = (y1oldm+y1m)/2 + .5*h*y1dm;
    nstep = nstep + 1;
    if nstep > maxstep
        error('mgs:zarchan:predictiveGuidance:predrunaway', ...
            ['predictg passed %d steps with h = %g s over t = %g to %g s.  ' ...
             'The book has no cap here and would not stop.'], maxstep, h, tdum, tf);
    end
end

xtf = x;  ytf = y;  zem1 = x - xm;  zem2 = y - ym;
end


% =============================================================================
function L = lambertpz(xic, yic, tfdes, xf, yf, xlongm, xlongt, a, gm, maxit) %#ok<INUSL>
%LAMBERTPZ  Listing 13.2, zarchan.txt:3943-3955, which Listing 15.1 calls but
%   does not reprint (:4538).  Secant search on flight-path angle, bracketed by
%   the escape condition lambda = r0*V^2/gm = 2.  "a" is unused, as in the book.
%
%   Its two OCR comparisons are resolved as in MGS.ZARCHAN.LAMBERTEFFICIENT,
%   where both were confirmed against the book's printed iteration table:
%   "if tf.tfdes" -> tf > tfdes, and "if xlongt.xlongm" -> xlongt > xlongm.
%   Here the target is at 90 deg and the missile at 85, so the branch taken is
%   the FIRST one -- and this call sits close to the bracket edge: 5 deg of arc
%   in 50 s needs about 34.4 kft/s against an escape speed of 36.7, i.e.
%   lambda ~ 1.76 out of 2.  Structural check 1 records it for that reason.
ric  = sqrt(xic^2+yic^2);
rf   = sqrt(xf^2+yf^2);
cphi = (xic*xf+yic*yf)/(ric*rf);
phi  = acos(min(1., max(-1., cphi)));
sphi = sin(phi);
r0   = ric;
icount = 0;

gmin = atan2((sphi-sqrt(2.*r0*(1.-cphi)/rf)), (1-cphi));
gmax = atan2((sphi+sqrt(2.*r0*(1.-cphi)/rf)), (1-cphi));
gam  = (gmin+gmax)/2.;
tf   = 0;
gold = gam;  told = 0;
vrx = NaN;  vry = NaN;  v = NaN;

while ~(abs(tfdes-tf) <= (.00000001*tfdes))
    top  = gm*(1.-cos(phi));
    temp = r0*cos(gam)/rf - cos(phi+gam);
    bot  = r0*cos(gam)*temp;
    v    = sqrt(top/bot);
    if xlongt > xlongm
        vrx = v*cos(pi/2.-gam+xlongm);
        vry = v*sin(pi/2.-gam+xlongm);
    else
        vrx = v*cos(-pi/2.+gam+xlongm);
        vry = v*sin(-pi/2.+gam+xlongm);
    end
    xlam = r0*v*v/gm;
    top1 = tan(gam)*(1-cos(phi)) + (1-xlam)*sin(phi);
    bot1p = (1-cos(phi))/(xlam*cos(gam)*cos(gam));
    bot1 = (2-xlam)*(bot1p + cos(gam+phi)/cos(gam));
    top2 = 2*cos(gam);
    bot2 = xlam*((2/xlam-1)^1.5);
    top3 = sqrt(2/xlam-1);
    bot3 = cos(gam)/tan(phi/2) - sin(gam);
    temp = (top2/bot2)*atan2(top3, bot3);
    tf   = r0*(top1/bot1 + temp)/(v*cos(gam));
    icount = icount + 1;
    if tf > tfdes
        gmax = gam;
    else
        gmin = gam;
    end
    if icount == 1
        xnext = (gmax+gmin)/2.;
    elseif tf == told
        % Not in the book.  A repeated tF divides by zero in the secant step
        % and returns Inf, which then fails the bracket test and bisects --
        % the same outcome, reached without a divide-by-zero warning.
        xnext = (gmax+gmin)/2.;
    else
        xnext = gam + (gam-gold)*(tfdes-tf)/(tf-told);
        if (xnext > gmax || xnext < gmin)
            xnext = (gmax+gmin)/2.;
        end
    end
    gold = gam;  told = tf;  gam = xnext;
    if icount >= maxit
        error('mgs:zarchan:predictiveGuidance:lambert', ...
            ['lambertpz did not converge in %d iterations: tF = %g s ' ...
             'against tFDES = %g s.  The book has no cap here.'], ...
            maxit, tf, tfdes);
    end
end

% NEAR-PARABOLIC GUARD, see the file header.  lam_out is recomputed from the
% RETURNED velocity rather than from the loop's surviving v, so the number the
% guard scores is the one the missile is actually launched on.
% ⚠ 'converged' below is a LITERAL true, and it is sound only because the loop
% above has exactly two exits: the convergence test, or error().  It is
% therefore a field that can never read false -- do not treat it as evidence.
L = struct('vrx', vrx, 'vry', vry, 'v', v, 'iters', icount, 'tf', tf, ...
           'gam_deg', gold*360./(2.*pi), 'lambda', r0*v*v/gm, ...
           'lam_out', r0*(vrx^2 + vry^2)/gm, 'converged', true);
end


% =============================================================================
function K = structuralChecks(E, Eoff, D, flipInfo)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan -- they are the weaker
%   claim that it agrees with ITSELF, which is what catches wiring faults.
K = struct();
names = {};  oks = [];  detail = {};

% 1  The Lambert solve converged AND actually put the missile on a collision
%    triangle: the zero effort miss right after the impulse must be ~0.  A
%    Lambert that converged onto the wrong branch would still "converge".
%    ⚠ The "lambda < 2" term below is the ESCAPE condition, NOT the
%    near-parabolic guard -- 2 is the value the bracket is defined by, so an
%    iterate can never exceed it.  Invariant 6 carries the real guard.
names{end+1}  = 'Lambert closes the collision triangle (|ZEM| at launch)';
oks(end+1)    = E.lambert.converged && E.zem0_ft <= 0.01 && E.lambert.lambda < 2;
detail{end+1} = sprintf(['|ZEM| = %.3e ft in %d iters, |V| = %.1f ft/s, ' ...
    'lambda = %.4f (escape at 2)'], E.zem0_ft, E.lambert.iters, E.lambert.v, ...
    E.lambert.lambda);

% 2  Staging.  The weight must DROP by exactly WS1 at TB1 and by exactly WS2 at
%    burnout, and end at the payload weight.  A transcription slip in the
%    stage-2 offset term (+WP2*TB1/TB2) shows up here and essentially nowhere
%    else, because the trajectory it produces is still perfectly smooth.
eps1 = 1e-6;
w1a = boosterWeight(D.TB1-eps1, D);
w1b = boosterWeight(D.TB1+eps1, D);
w2a = boosterWeight(D.TB1+D.TB2-eps1, D);
w2b = boosterWeight(D.TB1+D.TB2+eps1, D);
d1 = abs((w1a-w1b) - D.WS1);
d2 = abs((w2a-w2b) - D.WS2);
names{end+1}  = 'staging drops exactly WS1 then WS2, ending at WPAY';
oks(end+1)    = d1 <= 1e-6*D.WTOT && d2 <= 1e-6*D.WTOT && abs(w2b-D.WPAY) <= 1e-9;
detail{end+1} = sprintf('drop errors %.3e / %.3e lb, final weight %.4f lb', ...
    d1, d2, w2b);

% 3  DELETE EXPERIMENT.  Turn the guidance off entirely.  The book runs this
%    case on purpose (:4323) and it must still hit, because the Lambert solve
%    already put the missile on a collision triangle.  If it does NOT hit, the
%    nominal "virtually zero acceleration" result is being produced by the
%    guidance law rather than by the geometry, and benchmark 6 would be passing
%    for the wrong reason.
names{end+1}  = 'DELETE: guidance OFF still hits (collision triangle)';
oks(end+1)    = Eoff.miss_ft <= 1.0 && Eoff.divert_fps == 0;
detail{end+1} = sprintf('miss %.4e ft, divert %.4e ft/s with XNC forced to 0', ...
    Eoff.miss_ft, Eoff.divert_fps);

% 4  FAULT INJECTION.  Use the PROSE's ZEM_PLOS sign (:4477) instead of the
%    CODE's (:4515) and the miss must get much worse.  Run at the coarsest
%    sweep step: on the nominal case ZEM is ~4e-5 ft and its sign is invisible,
%    which is exactly why a single nominal run cannot adjudicate this.
if flipInfo.ran
    names{end+1}  = 'INJECT: prose ZEM sign (+) is far worse than code sign (-)';
    oks(end+1)    = flipInfo.miss_flip > 10*max(flipInfo.miss_ref, eps);
    detail{end+1} = sprintf(['at h_pred = %g s: miss %.1f ft with the code ' ...
        'sign, %.1f ft with the prose sign'], flipInfo.h, flipInfo.miss_ref, ...
        flipInfo.miss_flip);
end

% 5  The 30-g ghost is exactly one sample and it is the LAST one.  This is what
%    licenses benchmark 6 to mask it; if a run ever masks more than one sample,
%    the mask is hiding flight, not an artifact.
nbad = sum(~E.valid);
lastOnly = (nbad == 0) || (nbad == 1 && ~E.valid(end));
names{end+1}  = 'at most one post-intercept sample (tgo <= 0), and it is last';
oks(end+1)    = lastOnly;
detail{end+1} = sprintf(['%d of %d samples masked; final tgo = %.3e s, ' ...
    'final nc = %.3f g'], nbad, numel(E.valid), E.tgo(end), E.ncg(end));

% 6  NEAR-PARABOLIC GUARD.  See the file header for the 98,751 ft miss this
%    exists to catch.  ⚠ NOT keyed on E.lambert.converged -- that field is a
%    hard-coded literal in this port (sound, but it can never read false) and
%    the flag it stands for is TRUE on the wrong answer in every one of the six
%    transcriptions of Listing 13.2 this package carries.  ⚠ Nor on agreement
%    between those six: they agree to 0.000e+00 on the failing geometry.
K.lambda_out = E.lambert.lam_out;
LAMCAP = 1.999999;
names{end+1}  = 'near-parabolic guard: lambda = r0|v|^2/gm < 1.999999';
oks(end+1)    = isfinite(K.lambda_out) && K.lambda_out < LAMCAP;
detail{end+1} = sprintf(['lambda from the RETURNED velocity = %.9f ' ...
    '(cap %.6f, margin %.3e) in %d passes'], K.lambda_out, LAMCAP, ...
    LAMCAP - K.lambda_out, E.lambert.iters);

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
function B = benchmarks(E, D, D10, SW, opts)
%BENCHMARKS  Book values only.  Line numbers are in .source.
B = emptyBench();
onBook = (opts.PREDERR == 0) && (opts.tgo_scale == 1) && (opts.XNP == 3) && ...
         (opts.TF == 50) && (opts.XNCLIM == 966) && (opts.LEFT == 0) && ...
         (opts.AMAX == 20);

% ---- 1-4  Table 12.1, printed at AMAX = 10 g.  Weights are AMAX-independent;
%      burn times are not, so D10 is evaluated alongside the flown design.
B = addVal(B, 'Wtot, total booster weight', 6169, D10.WTOT, '0.1%', 0.001, ...
    'Table 12.1, p.263; zarchan.txt:3691', 'lb', ...
    'AMAX-independent, so this IS the booster Listing 15.1 flies');
B = addVal(B, 'Wp1, first-stage propellant', 3474, D10.WP1, '0.1%', 0.001, ...
    'Table 12.1, p.263; zarchan.txt:3692', 'lb', 'AMAX-independent');
B = addVal(B, 'tB1, first-stage burn time (AMAX = 10 g)', 32.2, D10.TB1, ...
    '0.5%', 0.005, 'Table 12.1, p.263; zarchan.txt:3697', 's', ...
    'the table is the 10-g design; this listing flies 20 g');
B = addVal(B, 'tB2, second-stage burn time (AMAX = 10 g)', 106, D10.TB2, ...
    '0.5%', 0.005, 'Table 12.1, p.263; zarchan.txt:3699', 's', ...
    'burn time is inversely proportional to AMAX');

% ---- 5  the flown booster, :4322.
if opts.AMAX == 20
    B = addVal(B, 'total burn time at AMAX = 20 g', 69.1, D.TB1+D.TB2, ...
        '0.5%', 0.005, 'text p.314; zarchan.txt:4322', 's', ...
        sprintf('tB1 = %.4f s, tB2 = %.4f s', D.TB1, D.TB2));
else
    B = addSkip(B, 'total burn time at AMAX = 20 g', '69.1 s', ...
        sprintf(['this run flies AMAX = %g g, and burn time scales as ' ...
                 '1/AMAX (%.4f s here).  Rescaling it to compare would be ' ...
                 'scoring the scaling, not the port.'], opts.AMAX, ...
                 D.TB1+D.TB2), 'text p.314; zarchan.txt:4322', 's');
end

% ---- 6  "virtually requires zero acceleration", :4487.
if onBook && opts.h_pred == 0.01
    B = addBound(B, 'nominal peak |nc| is virtually zero', ...
        '<= 0.01 g over the guided phase (tgo > 0)', ...
        sprintf('%.4g g', E.peak_nc_g), E.peak_nc_g <= 0.01, ...
        'text p.330 (Fig. 15.5); zarchan.txt:4487', 'g', ...
        sprintf(['the LAST sample, at tgo = %.2e s, sits on the 966 ft/s^2 ' ...
                 'clamp and is excluded -- it was never flown'], E.tgo(end)));
else
    B = addSkip(B, 'nominal peak |nc| is virtually zero', 'virtually zero', ...
        ['off-book settings: this row needs the listing''s own case with ' ...
         'h_pred = 0.01 s'], 'text p.330 (Fig. 15.5); zarchan.txt:4487', 'g');
end

% ---- 7-8  the step-size study, :4540.
hs = [SW.h];
if numel(hs) >= 2 && issorted(hs) && onBook
    miss = [SW.miss_ft];  div = [SW.divert_fps];
    B = addShape(B, 'miss increases with predictor step h', ...
        'strictly increasing over h = 0.01, 0.1, 1 s', ...
        joinNums(miss, '%.4g', 'ft'), all(diff(miss) > 0), ...
        'text p.337 (Fig. 15.6); zarchan.txt:4540', ...
        sprintf('h = %s s', mat2str(hs)));
    B = addShape(B, 'lateral divert increases with predictor step h', ...
        'strictly increasing over h = 0.01, 0.1, 1 s', ...
        joinNums(div, '%.4g', 'ft/s'), all(diff(div) > 0), ...
        'text p.337 (Fig. 15.6); zarchan.txt:4540', ...
        sprintf('h = %s s', mat2str(hs)));
else
    B = addSkip(B, 'miss/divert increase with predictor step h', ...
        'strictly increasing', ...
        ['needs at least two ASCENDING h values in ''h_sweep'' at book ' ...
         'settings'], 'text p.337 (Fig. 15.6); zarchan.txt:4540', '-');
end

% ---- 9-10  the h = 1 s divert, bracketed between APN's and PN's.
j1 = find(hs == 1.0, 1);
if ~isempty(j1) && onBook
    dv1 = SW(j1).divert_fps;
    B = addBound(B, 'divert at h = 1 s exceeds augmented PN''s', ...
        '> 1500 ft/s (APN, Fig. 14.13)', sprintf('%.1f ft/s', dv1), ...
        dv1 > 1500, 'claim at :4540; the number at zarchan.txt:4411', 'ft/s', ...
        ['this is the whole point of Fig. 15.6: a coarse predictor is ' ...
         'worse than augmented proportional navigation']);
    B = addBound(B, 'divert at h = 1 s is still below PN''s', ...
        '< 5200 ft/s (PN, Fig. 14.12)', sprintf('%.1f ft/s', dv1), ...
        dv1 < 5200, 'claim at :4544; number at zarchan.txt:4411', 'ft/s', ...
        [':4411 says about 5200 ft/s and :4341 says about 5300 ft/s for the ' ...
         'SAME figure -- the book contradicts itself.  5200 is scored ' ...
         'because it is the tighter bound; both lines are cited.']);
else
    B = addSkip(B, 'divert at h = 1 s, bracketed by APN and PN', ...
        '1500 < divert < 5200 ft/s', ...
        'h = 1 s is not in ''h_sweep'' at book settings', ...
        'zarchan.txt:4411, 4540, 4544', 'ft/s');
end

% ---- 11  "the missile acceleration response would be unstable", :4487.
if ~isempty(j1) && onBook
    rings = SW(j1).nc_max_g > 1 && SW(j1).nc_min_g < -1;
    B = addShape(B, 'h = 1 s command rings (sign reversal above 1 g)', ...
        'unstable acceleration response at h = 1 s', ...
        sprintf('nc in [%.2f, %.2f] g', SW(j1).nc_min_g, SW(j1).nc_max_g), ...
        rings, 'text p.330 (Fig. 15.5); zarchan.txt:4487', ...
        ['PROXY: a sign reversal at amplitude, not a proof of divergence.  ' ...
         'False at h = 0.01 and 0.1 s, where the command never changes sign ' ...
         'above 1 g.']);
end

% ---- what cannot be scored.
B = addSkip(B, 'the divert and acceleration CURVES of Figs. 15.5/15.6', ...
    'the plotted histories', ...
    ['figure-only.  Every figure in this book became an image link in the ' ...
     'extraction, so no ordinate can be read out of one.  The prose CLAIMS ' ...
     'about these two figures are scored above as ordering and bound rows; ' ...
     'anything else quoted from them would be fabricated.'], ...
    'Figs. 15.5, 15.6; zarchan.txt:4490, 4542', '-');

B = addInfo(B, 'guidance-command clamp', '966 ft/s^2', ...
    sprintf('%.0f ft/s^2 = %.1f g', opts.XNCLIM, opts.XNCLIM/32.2), ...
    'listing line; zarchan.txt:4516', ...
    ['a listing literal, not a published result.  Listing 14.2 uses ' ...
     'XNCLIM = 644 = 20 g (:4343) and the text calls it a 20-g limit ' ...
     '(:4322); 15.1 raises it to 30 g without saying so anywhere.']);
end


% =============================================================================
function s = joinNums(v, fmt, unitStr)
%JOINNUMS  "a -> b -> c unit", for the ordering rows' ours_str.
parts = cell(1, numel(v));
for i = 1:numel(v)
    parts{i} = sprintf(fmt, v(i));
end
s = [strjoin(parts, ' -> ') ' ' unitStr];
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER are benchSuite's adaptGeneric schema.  Every
%   row is built through one of the constructors below so the order cannot
%   drift: adaptGeneric ERRORS on a near-miss schema rather than adapting.
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

function B = addInfo(B, what, bookStr, oursStr, source, why)
%ADDINFO  Reported, never scored.  skipped = true keeps it out of the pass
%   count in BOTH directions -- an INFO row can neither pass nor fail.
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
L = {};
L{end+1} = 'Zarchan Listing 15.1 -- PREDICTIVE GUIDANCE vs A BOOSTING TARGET';
L{end+1} = sprintf('   %s', R.units);
L{end+1} = sprintf(['   missile %g deg / target %g deg, kick %g deg, ' ...
    'TF = %g s, Nprime = %g, h_pred = %g s, tgo_scale = %g'], ...
    R.p.XLONGMDEG, R.p.XLONGTDEG, R.p.XKICKDEG, R.p.TF, R.p.XNP, ...
    R.p.h_pred, R.p.tgo_scale);
L{end+1} = sprintf(['   booster at AMAX = %g g: Wtot %.1f lb, tB1 %.3f s, ' ...
    'tB2 %.3f s, total %.3f s'], R.design.AMAX, R.design.WTOT, R.design.TB1, ...
    R.design.TB2, R.design.TB1+R.design.TB2);
L{end+1} = sprintf(['   Lambert: %d iters, |V| = %.1f ft/s, gam = %.4f deg, ' ...
    'lambda = %.4f, |ZEM| at launch = %.3e ft'], R.lambert.iters, ...
    R.lambert.v, R.lambert.gam_deg, R.lambert.lambda, R.zem0_ft);
L{end+1} = sprintf(['   RESULT: miss %.6g ft, lateral divert %.6g ft/s, ' ...
    't_end %.4f s'], R.miss_ft, R.divert_fps, R.t_end);
L{end+1} = sprintf('   guidance OFF control run: miss %.6g ft, divert %.6g ft/s', ...
    R.guid_off.miss_ft, R.guid_off.divert_fps);
L{end+1} = '';

if ~isempty(R.sweep)
    L{end+1} = 'STEP-SIZE STUDY (Fig. 15.6)';
    L{end+1} = '     h_pred [s]       miss [ft]    divert [ft/s]    peak |nc| [g]';
    for i = 1:numel(R.sweep)
        L{end+1} = sprintf('   %12.4g   %13.5g   %14.5g   %14.5g', ...
            R.sweep(i).h, R.sweep(i).miss_ft, R.sweep(i).divert_fps, ...
            R.sweep(i).peak_nc_g); %#ok<AGROW>
    end
    L{end+1} = '';
end

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-56s %s', vstr(K.oks(i)), K.names{i}, ...
        K.detail{i}); %#ok<AGROW>
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
                L{end+1} = sprintf('   SKIP %-50s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case 'info'
                L{end+1} = sprintf('   INFO %-50s %s', b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case {'shape','bound'}
                L{end+1} = sprintf('   %-4s %-50s %s', vstr(b.ok), b.what, ...
                    b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf(['   %-4s %-50s book %s %s / ours %s %s ' ...
                    '(%+.5f%%, band %s)'], vstr(b.ok), b.what, b.book_str, ...
                    b.units, b.ours_str, b.units, b.pct, b.band); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped/info -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: predictive guidance is only as good as its models AND its';
L{end+1} = '   tgo.  The command is Nprime*ZEM/tgo^2, so a tgo scaled by k flies';
L{end+1} = '   an effective Nprime/k^2 -- and on a PERFECT-prediction case that';
L{end+1} = '   error is completely invisible, because ZEM is already zero.  Use';
L{end+1} = '   ''tgo_scale'' together with a COARSE ''h_pred'' to see the bill.';
s = strjoin(L, newline);
end

function s = vstr(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end


% =============================================================================
function drawCurves(R)
%DRAWCURVES  The listing's four figures (:4520), in its own axes and units.
figure;
plot(R.distnmT, R.altnmT, R.distnmM, R.altnmM); grid on
xlabel('Downrange (Nmi)'); ylabel('Altitude (Nmi)');
legend('target', 'missile'); title('Listing 15.1 engagement');

figure; plot(R.t, R.atplosg); grid on
xlabel('Time (Sec)'); ylabel('Acceleration Perp. To LOS (G)');

figure; plot(R.t, R.ncg); grid on
xlabel('Time (Sec)'); ylabel('Acceleration (G)');

figure; plot(R.t, R.delv); grid on
xlabel('Time (Sec)'); ylabel('Lateral Divert (Ft/Sec)');
end
