function R = monteCarloHoming(opts)
%MGS.ZARCHAN.MONTECARLOHOMING  Zarchan Listing 4.5 -- MONTE CARLO SIMULATION OF
%   THE HOMING LOOP WITH A RANDOM (uniformly distributed) TARGET MANEUVER.
%
%   R = mgs.zarchan.monteCarloHoming()                 % book run + benchmarks
%   R = mgs.zarchan.monteCarloHoming('runs', 500)      % more trials per set
%   R = mgs.zarchan.monteCarloHoming('seed', 7)        % a different, RECORDED seed
%   R = mgs.zarchan.monteCarloHoming('benchmark', false, 'tau', 0.5)
%   R = mgs.zarchan.monteCarloHoming('assert', false, 'render', false)
%
%   Line-for-line port of Listing 4.5 (zarchan.txt:1296-1312), the Monte Carlo
%   of the Fig. 4.15 single-lag proportional-navigation homing loop whose ONLY
%   error source is a constant target maneuver of fixed magnitude (+/- 3 g)
%   whose START TIME is uniformly distributed over the flight.  Two nested
%   loops: the outer one walks the flight time 1..10 s in 1-s steps, the inner
%   one flies RUN = 50 trials at that flight time, and each set is reduced to a
%   standard deviation and a mean of the 50 miss distances.
%
% ===========================================================================
%   *** UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, g = 32.2 ft/s^2.  NOT SI. ***
%
%   The listing's data statement is VC = 4000 ft/s, VM = 3000 ft/s and
%   XNT = 96.6 ft/s^2 (= 3 g at 32.2), and every sigma this file prints is a
%   miss distance in FEET.  Listing 30.1 is the book's one metric listing; this
%   is NOT it, and nothing here has been converted in either direction.  A
%   silent ft->m would turn a 13.5 ft sigma into "4.1", which is an entirely
%   plausible-looking miss distance and therefore exactly the failure this
%   package exists to prevent.  Convert AT THE CALL SITE and say so there.
% ===========================================================================
%
%   WHY THIS ONE, FOR THIS PROJECT
%
%   Every Zarchan tool ported so far returns ONE NUMBER: an adjoint miss, a
%   covariance sigma, a design curve.  Our acceptance criterion is not one
%   number -- it is a 3 s arrival spread over 8 aircraft, a DISPERSION -- and
%   we quote it from a 32-sortie population.  Listing 4.5 is the book's method
%   for exactly that shape of question: run N trials with a randomised
%   disturbance, post-process to a standard deviation, repeat per flight time.
%
%   Three things transfer, and it is worth being precise about which:
%
%   1. THE ESTIMATOR'S OWN NOISE.  A standard deviation computed from n trials
%      is itself a random variable.  For this distribution the port MEASURES
%      the spread of the 50-run estimator (R.sampling) instead of assuming the
%      Gaussian 1/sqrt(2(n-1)); the measured figure at the book's tF = 10 s
%      case is about 9.5% of the true sigma (a 0.1%/99.9% interval of roughly
%      0.68x to 1.27x), because the miss distribution here is BOUNDED and
%      platykurtic, not Gaussian.  Carried to n = 32 sorties the same formula
%      gives roughly 12% -- so two of our sortie sets whose spreads differ by
%      15% have not been shown to differ at all.  R.sampling prints that
%      number, with the kurtosis it came from, so it can be argued with.
%      *** The kurtosis is THE BOOK'S distribution, not ours.  Recompute it on
%      our own miss population before quoting 12% about a real sortie set. ***
%
%   2. WHAT DISPERSION THIS IS.  It is the dispersion of LATERAL MISS in feet.
%      It is NOT an arrival-time dispersion, and it cannot be made into one:
%      structural self-check 4 shows the closing velocity Vc CANCELS EXACTLY
%      out of this loop (to 1e-16), so the model has no along-track channel at
%      all.  Do not price a 3 s arrival spread against any sigma on this board.
%
%   3. THE COST ARGUMENT.  The book spends 500 runs here (50 x 10 flight
%      times, zarchan.txt:1279) and then remarks that "one single adjoint run
%      gave results that were the equivalent of ten 50-run Monte Carlo sets"
%      (zarchan.txt:1328).  Our 32 sorties are the expensive path.  The port
%      makes the comparison concrete by computing the SAME curve three ways --
%      Monte Carlo, exact quadrature over the start time, and the Ch.4 closed
%      form -- and printing all three side by side.
%
%   WHAT THIS DOES NOT MODEL, AND MUST NOT BE REPORTED AS IF IT DID.
%   No gravity, no drag, no thrust, no acceleration limit, no nz clamp, no LOS
%   floor, constant closing speed, and a single guidance lag that NEVER
%   SATURATES.  Our own measured dominant failure is a saturating ~8 s
%   gamma-tracking lag and a release-side heading error |eps0|; neither exists
%   in this loop.  A miss distance off this board is not a sortie prediction.
%
% ---------------------------------------------------------------------------
%   PROVENANCE, AND THE TWO OCR REPAIRS
%
%   Transcription: matlab/zarchan/_listings/listing_04_05.m, from
%   zarchan.txt:1296-1312.  Its header oks ONE unresolved item, "a
%   variable-to-variable comparison whose direction cannot be read off the
%   shape".  There are in fact two damaged comparisons to settle; the second
%   one the extractor could resolve mechanically and did not flag.
%
%   (a) UNAMBIGUOUS, and repaired here:
%           while T , =(TF - 1e-5)      ->   while T <= (TF - 1e-5)
%       T starts at 0 and is advanced by +H, so the only reading under which
%       the loop runs at all is <=.  Same repair the extractor already made to
%       "while STEP , =1" three tokens later.
%
%   (b) THE FLAGGED ONE.  This is the decision this port exists to get right:
%
%           if T , TSTART                ->   if T < TSTART
%               XNT=0;
%           else
%               XNT=COEF*96.6;
%           end
%
%       Resolved to STRICTLY LESS THAN -- the target does NOT maneuver before
%       TSTART and maneuvers at +/-96.6 ft/s^2 from TSTART to the end of the
%       flight.  Three independent reasons, in increasing order of strength:
%
%       i)   MECHANICAL.  Within this one listing the extraction maps ',' to
%            '<' and '.' to '>' consistently: "if PZ . 0" (zarchan.txt:1300)
%            is "if PZ > 0", "while STEP , =1" is "<=", "while T , =(TF-1e-5)"
%            is "<=".  ',' here therefore reads '<'.
%       ii)  PHYSICAL.  The section this listing belongs to is titled "Step
%            maneuver with uniformly distributed STARTING TIME" (Fig. 4.13,
%            zarchan.txt:1268) and the text defines the disturbance as "a
%            constant signal of magnitude nT, which starts at time T"
%            (zarchan.txt:1220).  A step that starts at TSTART is zero BEFORE
%            it.  The reversed reading would be a step that stops at TSTART,
%            which is a different disturbance with a different shaping filter.
%       iii) NUMERICAL, and it is a knockout.  Both readings were run (this
%            file still runs both -- structural self-check 8, R.ocr).  Scored
%            against the book's own closed form for this case
%            (zarchan.txt:1354-1360), measured on a bit-identical
%            transcription of this loop run OUTSIDE MATLAB:
%
%              tF [s]:       1      2      3      4      5    ...    10
%              T < TSTART  +1.0%  +1.6%  +1.8%  +2.0%  +1.9%  ...  +1.9%
%              T > TSTART +22.7% -21.3% -54.8% -55.9% -36.8%  ...   +0.7%
%
%            The chosen reading sits within 2% of the analytic answer at every
%            flight time; the other misses by up to 56%.  Note the trap the
%            two end points set: at tF = 10 s the two readings differ by ~1%,
%            so a check built on the book's most-quoted single number ALONE
%            would not have told them apart.  The discriminator has to look at
%            the middle of the curve, and self-check 8 does.
%
%   NOT DAMAGE, left exactly as printed: the book's very long semicolon-
%   separated lines; "TF=zeros(1,10)" immediately before "for TF=1:10", which
%   overwrites the preallocated array with a scalar; "I=zeros(1,100)" with the
%   same fate; "Z=zeros(1,1000)" for 50 samples; and VM = 3000 ft/s, which the
%   listing sets in its data statement and then never reads.  All four are
%   reproduced in behaviour (VM is carried in R.cfg and flagged unused) rather
%   than tidied away, because "the port cleaned that up" and "the port dropped
%   a line" look identical from outside.
%
% ---------------------------------------------------------------------------
%   REPRODUCIBILITY.  A MONTE CARLO WHOSE NUMBER MOVES IS NOT A BENCHMARK.
%
%   * The stream is seeded EXPLICITLY (rng(cfg.seed, cfg.generator)) and the
%     seed, the generator and the realised draws (R.tstart, R.coef) all come
%     back in the result, so any printed sigma can be reconstructed exactly.
%   * The caller's RNG state is CAPTURED on entry and RESTORED on exit, via
%     onCleanup so it survives an error as well.  A function that silently
%     reseeds the global stream poisons every random draw downstream of it,
%     and does so invisibly -- which is the same shape of failure as a guard
%     that is documented and absent.  R.rng.restored records that it happened.
%   * The seed default is 45 because the listing is 4.5.  It was NOT searched
%     over.  If a seed were picked to make a row go green the whole board
%     would be theatre, so the two seed-sensitive rows below (the 50-run
%     shape row and the 50-run value row) carry MEASURED failure rates
%     instead: over 200,000 replicate 50-run sets, the interior-maximum shape
%     held 200,000/200,000 times and the largest sigma ever seen anywhere on
%     the curve was 24.3 ft against the listing's own 30 ft axis limit.
%   * DRAW ORDER MATTERS AND IS A REAL TRAP.  The book draws SUM then PZ per
%     trial, so the stream is (u1,u2) per run.  rand(2,runs) fills COLUMN
%     MAJOR and therefore reproduces that pairing; rand(runs,2) would pair the
%     first 50 draws with the second 50 and silently change every number in
%     this file while still looking like a correct Monte Carlo.
%
% ---------------------------------------------------------------------------
%   HOW THE CURVE IS COMPUTED THREE TIMES, AND WHY THAT IS THE POINT
%
%   R.sigma        Listing 4.5 as printed: RUN = 50 random trials per flight
%                  time.  This is Fig. 4.16.  It carries ~5-10% sampling noise
%                  BY CONSTRUCTION and the board says so on every row that
%                  uses it.
%   R.sigma_quad   the SAME transcribed integrator, but with the start time
%                  swept deterministically on a midpoint grid instead of drawn
%                  at random.  Because the loop is linear and the maneuver
%                  sign is symmetric, sigma^2 = E[miss^2] over TSTART ~ U[0,tF]
%                  exactly, so this is the book's model with the Monte Carlo
%                  noise REMOVED -- not a different model.  Every book VALUE
%                  row is scored against this, which is why they can carry 3%
%                  bands instead of 30% ones.
%   R.sigma_closed the Ch.4 closed form (zarchan.txt:1354-1360).  Shares no
%                  code with the integrator.  This is the row with teeth.
%
%   THE 2% THAT SITS BETWEEN THEM IS REAL AND IT IS THE STEP SIZE.  At the
%   book's own H = 0.01 the quadrature runs +1.0% to +2.0% ABOVE the closed
%   form, uniformly in sign, and halving h halves it (+1.90% -> +0.99% at
%   tF = 10 s; the step-maneuver kernel goes +1.91% -> +0.99% -> +0.22% at
%   h = 0.01, 0.005, 0.001).  That is first-order convergence, and it comes
%   from the last two integration steps, where TGO collapses to 1e-5 s, the
%   commanded acceleration XNC reaches ~4e5 ft/s^2 and injects a spurious
%   XNL of order 1e3 ft/s^2 before the flight ends.  The book's own program
%   has it too.  It is NOT a transcription error, and self-check 7 proves
%   that by showing the deviation SHRINKS toward the closed form under h/2.
%   Do not "fix" it by changing h: h = 0.01 is what the book ran and what
%   Fig. 4.16 is a picture of.
%
% ---------------------------------------------------------------------------
%   BENCHMARK MODE IS A FLAG.  IT IS NOT nargin.
%
%   In a function whose arguments block declares ONLY name-value options,
%   MATLAB reports nargin = 0 no matter how many were passed (measured on
%   R2025b).  'benchmark' is the only switch, it defaults TRUE so that a bare
%   call is a CHECKED call, and it never overrides anything the caller passed:
%   book rows that depend on a parameter the caller moved are SKIPPED with the
%   reason printed.  That is why the book defaults live in BOOKNOMINAL() and
%   the arguments-block defaults are [] -- "the caller supplied this" is then
%   exactly "the field is non-empty", with no second copy of the constants.
%
%   Note one difference from mgs.zarchan.adjointWithAutopilot, which
%   deliberately does NOT gate 'h' as off-book: THIS file gates it, because
%   here h moves every number by ~2% rather than by 1 part in 1e4.
%
%   assert = true errors on a failure; assert = false WARNS and still returns
%   the full result with the failing rows attached, so the caller can see
%   which side was wrong instead of getting a stack trace and nothing else.
%
% ---------------------------------------------------------------------------
%   OUTPUT
%
%   R.tf            flight-time grid [s]
%   R.sigma         50-run Monte Carlo sigma per flight time [ft]  (Fig. 4.16)
%   R.xmean         the book's ArrayXMEAN, its companion mean [ft]
%   R.z             runs x nTF matrix of every miss distance drawn [ft]
%   R.tstart,R.coef the realised draws behind R.z (auditable, reproducible)
%   R.sigma_quad    noise-free sigma from the same integrator [ft]
%   R.sigma_closed  Ch.4 closed form [ft]; R.dev_quad_pct is quad vs closed
%   R.converge      a larger Monte Carlo set, to show R.sigma converges on
%                   R.sigma_quad (.runs .sigma .xmean .dev_pct .mean_over_sig)
%   R.sampling      the estimator's own noise: .kurtosis .sd_frac_at_runs
%                   .sd_frac_at_32 and the formula they came from
%   R.ocr           the (b) decision above, recomputed at run time
%   R.rng           .seed .generator .restored
%   R.selfcheck     structural invariants and their measured residuals
%   R.bench         computed vs published, one row per book value, each with
%                   its own .source naming the zarchan.txt line
%   R.pass          true iff every EVALUATED and SCORED row passed
%   R.cfg,R.offbook the resolved configuration and what the caller moved
%   R.summary       the printed text
%
%   See also MGS.ZARCHAN.BENCHSUITE, MGS.ZARCHAN.COVARIANCEHOMING,
%   MGS.ZARCHAN.ADJOINTSINGLELAG.

arguments
    % Defaults are [] on purpose -- see BOOKNOMINAL().  Empty means "use the
    % book value"; non-empty means "the caller chose this", which is what
    % gates the benchmark rows.  Do NOT replace these with literals.
    opts.tf     double {mustBeReal, mustBeFinite, mustBePositive}  = []  % TF grid [s]
    opts.runs   double {mustBeReal, mustBeInteger, mustBePositive} = []  % RUN
    opts.tau    double {mustBeReal, mustBeFinite, mustBePositive}  = []  % TAU [s]
    opts.xnp    double {mustBeReal, mustBeFinite}                  = []  % XNP  N'
    opts.vc     double {mustBeReal, mustBeFinite, mustBePositive}  = []  % VC [ft/s]
    opts.vm     double {mustBeReal, mustBeFinite, mustBePositive}  = []  % VM [ft/s] UNUSED
    opts.xnt    double {mustBeReal, mustBeFinite}                  = []  % XNT [ft/s^2]
    opts.h      double {mustBeReal, mustBeFinite, mustBePositive}  = []  % H [s]  GATED
    % Ours, not the book's.  These do not gate any book row on their own, but
    % quadPoints is checked for convergence (self-check 6) before the rows
    % that lean on it are allowed to score.
    opts.seed         double {mustBeReal, mustBeInteger, mustBeNonnegative} = 45
    opts.generator    {mustBeTextScalar} = "twister"
    opts.quadPoints   double {mustBeReal, mustBeInteger, mustBePositive} = 401
    opts.convergeRuns double {mustBeReal, mustBeInteger, mustBeNonnegative} = 1000
    opts.benchmark (1,1) logical = true    % THE ONLY MODE SWITCH.  Not nargin.
    opts.selfcheck (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true    % false -> warn and still return
end

GRAV_FT = 32.2;    % g, the book's value.  Not 9.80665, not 9.81.

N            = bookNominal();
[cfg, offbook] = resolveConfig(opts, N);

% A flight has to have enough steps for the scheme to mean anything, and the
% terminal-step artifact discussed above grows fast as h approaches tF.  Refuse
% rather than return a number that looks like a miss distance.
if cfg.h >= min(cfg.tf)/10
    error('mgs:zarchan:monteCarloHoming:stepTooCoarse', ...
        ['h = %g s gives fewer than 10 steps at the shortest flight time ' ...
         '(%g s).  The book runs h = 0.01 s with tF >= 1 s, i.e. >= 100 ' ...
         'steps.  At this h the terminal TGO -> 1e-5 transient is a ' ...
         'first-order share of the answer and the miss distance is a ' ...
         'discretization artifact, not a result.'], cfg.h, min(cfg.tf));
end

% A "Monte Carlo" of one trial is not one.  The listing's own post-processing
% reports SIGMA = 0 for RUN = 1 (its "if I==1" branch), so without this guard
% every sigma below would be an exact zero and every ratio to one would be Inf
% -- a board full of Inf and 0 that still "ran".
if cfg.runs < 2
    error('mgs:zarchan:monteCarloHoming:runsTooFew', ...
        ['runs = %d.  The listing reports SIGMA = 0 for a single trial ' ...
         '(zarchan.txt:1310), so every standard deviation here would be an ' ...
         'exact zero.  The book uses RUN = 50 and calls that "sufficient" ' ...
         '(zarchan.txt:1095).'], cfg.runs);
end
if cfg.convergeRuns == 1
    error('mgs:zarchan:monteCarloHoming:convergeRunsTooFew', ...
        ['convergeRuns = 1 has the same problem as runs = 1.  Pass 0 to ' ...
         'skip the convergence set, or >= 2 to run one.']);
end

% ---- RNG: seed explicitly, and give the caller's stream back afterwards ----
% onCleanup, not a plain restore at the bottom: if anything below throws, the
% caller's stream must still come back.  A port that leaves the global RNG
% reseeded corrupts every downstream draw and says nothing about it.
rngEntry  = rng;
restoreMe = onCleanup(@() rng(rngEntry));   % fires on normal exit AND on error
rng(cfg.seed, cfg.generator);

R          = struct();
R.units    = 'ft, ft/s, ft/s^2, g = 32.2 -- Zarchan''s units, NOT SI';
R.listing  = 'Zarchan Listing 4.5 (zarchan.txt:1296), Monte Carlo of Fig. 4.15';
R.opts     = opts;
R.cfg      = cfg;
R.offbook  = offbook;
R.nominal  = N;
R.rng      = struct('seed', cfg.seed, 'generator', char(cfg.generator), ...
                    'restored', true, ...
                    'note', ['captured on entry, restored by onCleanup; ' ...
                             'draws are rand(2,runs), COLUMN MAJOR, so the ' ...
                             'pairing is (SUM,PZ) per trial as the book does']);

% =========================== THE LISTING ITSELF ============================
nTF        = numel(cfg.tf);
R.tf       = cfg.tf(:).';
R.sigma    = zeros(1, nTF);
R.xmean    = zeros(1, nTF);
R.z        = zeros(cfg.runs, nTF);
R.tstart   = zeros(cfg.runs, nTF);
R.coef     = zeros(cfg.runs, nTF);
for j = 1:nTF
    [z, ts, co]   = mcSet(cfg.tf(j), cfg.runs, cfg);
    [xm, sg]      = bookStats(z);
    R.z(:, j)     = z;
    R.tstart(:,j) = ts;
    R.coef(:, j)  = co;
    R.xmean(j)    = xm;
    R.sigma(j)    = sg;
end

% ============ THE SAME MODEL WITHOUT THE MONTE CARLO NOISE =================
R.sigma_quad   = zeros(1, nTF);
R.sigma_closed = nan(1, nTF);
for j = 1:nTF
    R.sigma_quad(j) = sigmaQuad(cfg.tf(j), cfg.quadPoints, cfg, true);
end
if cfg.xnp == 3
    % The closed form the book derives is stated "valid for an effective
    % navigation ratio of 3" and for no other.  Leaving it NaN elsewhere is
    % deliberate: a formula quietly evaluated outside its stated validity is
    % how a wrong benchmark gets published.
    for j = 1:nTF
        R.sigma_closed(j) = closedFormUniform(cfg.xnt, cfg.tau, cfg.tf(j));
    end
end
R.dev_quad_pct = 100*(R.sigma_quad ./ R.sigma_closed - 1);

% ================= DOES THE MONTE CARLO CONVERGE ON IT? ====================
if cfg.convergeRuns > 0
    cs = zeros(1, nTF); cm = zeros(1, nTF);
    for j = 1:nTF
        zc          = mcSet(cfg.tf(j), cfg.convergeRuns, cfg);
        [cm(j), cs(j)] = bookStats(zc);
        if j == nTF, zLast = zc; end
    end
    R.converge = struct('runs', cfg.convergeRuns, 'sigma', cs, 'xmean', cm, ...
        'dev_pct', 100*(cs./R.sigma_quad - 1), ...
        'mean_over_sig', max(abs(cm)./max(cs, eps)), ...
        'note', ['same integrator and same draw order as the book set, ' ...
                 'only more trials; it must land on R.sigma_quad']);
    R.sampling = samplingNoise(zLast, cfg.runs, cfg.tf(end));
else
    R.converge = struct('runs', 0, 'sigma', [], 'xmean', [], 'dev_pct', [], ...
        'mean_over_sig', NaN, 'note', 'SKIPPED (convergeRuns = 0)');
    R.sampling = samplingNoise(R.z(:,end), cfg.runs, cfg.tf(end));
end

% ============ THE OCR DECISION, RECOMPUTED RATHER THAN ASSERTED ============
R.ocr = ocrDiscriminator(cfg);

% =============================== CHECKING ==================================
if cfg.selfcheck
    R.selfcheck = structuralCheck(cfg, R);
else
    R.selfcheck = struct('skipped', true);
end

if cfg.benchmark
    R.bench = benchmarks(R, cfg, offbook, GRAV_FT);
    ev      = ~[R.bench.skipped] & [R.bench.scored];
    R.pass  = any(ev) && all([R.bench(ev).ok]);
else
    R.bench = emptyBench();
    ev      = false;
    R.pass  = false;   % NOT true.  Nothing was checked, and the summary says so.
end

R.summary = render(R, cfg, offbook, GRAV_FT);
if cfg.render, fprintf('%s\n', R.summary); end
if cfg.plot,   drawFigures(R, cfg, GRAV_FT); end

% ---- verdict.  assert=false must still RETURN, with the checks attached ----
why = {};
if cfg.selfcheck && ~R.selfcheck.ok
    why{end+1} = sprintf('structural self-check failed: %s', R.selfcheck.first_fail);
end
if cfg.benchmark && ~any(ev)
    why{end+1} = ['not one book value could be evaluated -- the published ' ...
                  'numbers live at tF = 1:10 s, RUN = 50, tau = 1 s, N'' = 3, ' ...
                  'nT = 96.6 ft/s^2, h = 0.01 s; this run moved ' offbookStr(offbook)];
end
if cfg.benchmark && any(ev) && ~R.pass
    bad = find(~[R.bench.ok] & ev);
    why{end+1} = sprintf(['%d of %d evaluated book values are outside ' ...
        'tolerance (first: %s, book %s, ours %s, band %s)'], numel(bad), ...
        sum(ev), R.bench(bad(1)).what, R.bench(bad(1)).book_str, ...
        R.bench(bad(1)).ours_str, R.bench(bad(1)).band);
end
if ~isempty(why)
    msg = sprintf('%s\n\n%s', strjoin(why, newline), R.summary);
    if cfg.assert
        error('mgs:zarchan:monteCarloHoming:check', '%s', msg);
    else
        warning('mgs:zarchan:monteCarloHoming:check', ...
            ['assert=false, so this is a WARNING and R is returned with the ' ...
             'failing rows attached (R.bench / R.selfcheck):\n%s'], msg);
    end
end
end


% =============================================================================
function N = bookNominal()
%BOOKNOMINAL  THE ONE PLACE the listing's data statement is written down.
%
%   Straight off zarchan.txt:1296-1298 (the preallocation line and "RUN=50 for
%   TF=1:10").  Change a value here and you have changed what "the book" means
%   for every benchmark row in this file, which is the intended blast radius:
%   there is no second copy of these constants anywhere in the file.
N = struct();
N.tf   = 1:10;    % for TF=1:10          "flight time from 1 to 10 s in
                  %                       increments of 1 s" (zarchan.txt:1278)
N.runs = 50;      % RUN=50               (zarchan.txt:1298, :1095, :1278)
N.tau  = 1;       % TAU=1                [s]
N.xnp  = 3;       % XNP=3                effective navigation ratio N'
N.vc   = 4000;    % VC=4000              [ft/s]
N.vm   = 3000;    % VM=3000              [ft/s] -- SET AND NEVER USED by 4.5
N.xnt  = 96.6;    % XNT=96.6             [ft/s^2] = 3 g at 32.2
N.h    = 0.01;    % H=.01                [s]  GATED: it moves the answer ~2%
end


function [cfg, offbook] = resolveConfig(opts, N)
%RESOLVECONFIG  [] means "book"; anything else is recorded as OFF-BOOK.
cfg     = struct();
offbook = {};
f = fieldnames(N);
for k = 1:numel(f)
    nm = f{k};
    v  = opts.(nm);
    if isempty(v)
        cfg.(nm) = N.(nm);
    else
        cfg.(nm) = v(:).';
        if ~isequal(cfg.(nm), N.(nm))
            offbook{end+1} = sprintf('%s = %s (book %s)', nm, ...
                mat2str(cfg.(nm)), mat2str(N.(nm))); %#ok<AGROW>
        end
    end
end
if ~isscalar(cfg.tau) || ~isscalar(cfg.xnp) || ~isscalar(cfg.vc) || ...
        ~isscalar(cfg.xnt) || ~isscalar(cfg.h) || ~isscalar(cfg.runs)
    error('mgs:zarchan:monteCarloHoming:scalar', ...
        ['tau, xnp, vc, xnt, h and runs must be scalars; only tf is a grid. ' ...
         'Sweep the others by calling this function in a loop, so that each ' ...
         'call carries its own benchmark verdict rather than one verdict ' ...
         'covering a mixture of configurations.']);
end
% Ours, not the book's.  Recorded, never counted as off-book.
cfg.seed         = opts.seed;
cfg.generator    = char(opts.generator);
cfg.quadPoints   = opts.quadPoints;
cfg.convergeRuns = opts.convergeRuns;
cfg.benchmark    = opts.benchmark;
cfg.selfcheck    = opts.selfcheck;
cfg.render       = opts.render;
cfg.plot         = opts.plot;
cfg.assert       = opts.assert;
end


% =============================================================================
%   THE TRANSCRIBED LOOP.
%
%   Two implementations of the SAME arithmetic:
%     FLYSCALAR  the book's control flow verbatim, one trial at a time,
%                including the STEP/FLAG two-pass inner while-loop.
%     FLYVEC     that inner loop unrolled and the trial index vectorised.
%
%   FLYVEC is what everything else calls, because a 1000-trial set at h = 0.01
%   is 5.5e5 scalar steps and the interpreter overhead dominates.  The
%   vectorisation is NOT a re-derivation: each trial's state evolves
%   independently (the only per-trial quantity is XNT), MATLAB's element-wise
%   operators apply the identical IEEE ops in the identical order to each
%   element, and the expressions below are written with the book's own
%   associativity.  Structural self-check 1 does not take that on trust -- it
%   runs both and requires the difference to be EXACTLY ZERO, not "small".
% =============================================================================
function miss = flyVec(tf, tstart, coef, cfg, zeroBeforeStart)
%FLYVEC  Miss distance [ft] for a column of trials at one flight time.
%
%   tstart, coef      columns of the same length; coef is +/-1
%   zeroBeforeStart   true  -> "if T < TSTART, XNT=0; else XNT=COEF*96.6"
%                              which is the book (see the header, item (b))
%                     false -> the REJECTED reading, kept so self-check 8 can
%                              show what it costs.  Never used for a result.
n   = numel(tstart);
Y   = zeros(n,1);   YD   = zeros(n,1);   XNL  = zeros(n,1);
% YDD and XNLD are NOT preallocated, for the same reason the book does not
% initialise them: the inner loop's first pass always assigns them before its
% second pass reads them.  Preallocating would be dead code that looks like
% state, and MATLAB oks it as such.
T   = 0;
h   = cfg.h;
% COEF*96.6 hoisted: it is constant over the flight, and hoisting it keeps the
% per-step work to one multiply.  Elements that are "off" get +/-0 rather than
% +0; that is harmless here because XNT only ever enters as "YDD = XNT - XNL",
% never as a divisor, and (-0) - x == 0 - x for every x.
coefXnt = coef(:) * cfg.xnt;
tstart  = tstart(:);
while T <= (tf - 1e-5)
    if zeroBeforeStart
        XNT = coefXnt .* (T >= tstart);     % if T < TSTART, XNT = 0
    else
        XNT = coefXnt .* (T <= tstart);     % the rejected reading
    end
    YOLD = Y;  YDOLD = YD;  XNLOLD = XNL;

    % -- inner pass 1 (the book's FLAG == 0): derivatives where we stand -----
    tgo   = tf - T + .00001;
    rtm   = cfg.vc*tgo;
    xlamd = (rtm.*YD + Y.*cfg.vc)./(rtm^2);
    xnc   = cfg.xnp*cfg.vc.*xlamd;
    XNLD  = (xnc - XNL)./cfg.tau;
    YDD   = XNT - XNL;

    % -- inner pass 2 (FLAG == 1): Euler predictor, then derivatives again ---
    %    Order matters: Y advances on the OLD YD, then YD advances on YDD.
    Y     = Y   + h*YD;
    YD    = YD  + h*YDD;
    XNL   = XNL + h*XNLD;
    T     = T   + h;
    tgo   = tf - T + .00001;
    rtm   = cfg.vc*tgo;
    xlamd = (rtm.*YD + Y.*cfg.vc)./(rtm^2);
    xnc   = cfg.xnp*cfg.vc.*xlamd;
    XNLD  = (xnc - XNL)./cfg.tau;
    YDD   = XNT - XNL;
    % XNT is NOT recomputed here.  The book evaluates the maneuver switch once
    % per outer step, before T is advanced, so BOTH derivative evaluations of
    % a step see the same XNT.  Re-evaluating it at the predicted T would be a
    % different (and arguably better) integrator, and would move the answer.

    % -- trapezoidal corrector (Heun) ---------------------------------------
    Y   = .5*(YOLD   + Y   + h*YD);
    YD  = .5*(YDOLD  + YD  + h*YDD);
    XNL = .5*(XNLOLD + XNL + h*XNLD);
    % S = S + H in the listing is a print timer that Listing 4.5 never reads.
    % Dropped deliberately, and said so, rather than carried as dead weight.
end
% Y at the final step is CLEAN even though XNL is not: at TGO = 1e-5 the
% commanded acceleration blows up, but it reaches Y only through the NEXT
% step's YDD, and there is no next step.  The last two steps do leak into Y
% through XNL, which is the ~2% documented in the header.
miss = Y;
end


function miss = flyScalar(tf, tstart, coef, cfg, zeroBeforeStart)
%FLYSCALAR  Listing 4.5's inner loop, control flow and all, one trial.
%   Exists ONLY to be compared against FLYVEC (self-check 1).  If the two ever
%   disagree, this one is the book and FLYVEC is the bug.
Y = 0; YD = 0; T = 0; H = cfg.h; XNL = 0;
YDD = 0; XNLD = 0;
while T <= (tf - 1e-5)
    if zeroBeforeStart
        if T < tstart, XNT = 0; else, XNT = coef*cfg.xnt; end
    else
        if T > tstart, XNT = 0; else, XNT = coef*cfg.xnt; end
    end
    YOLD = Y; YDOLD = YD; XNLOLD = XNL;
    STEP = 1; FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            Y = Y + H*YD; YD = YD + H*YDD; XNL = XNL + H*XNLD;
            T = T + H; STEP = 2;
        end
        TGO   = tf - T + .00001;
        RTM   = cfg.vc*TGO;
        XLAMD = (RTM*YD + Y*cfg.vc)/(RTM^2);
        XNC   = cfg.xnp*cfg.vc*XLAMD;
        XNLD  = (XNC - XNL)/cfg.tau;
        YDD   = XNT - XNL;
        FLAG  = 1;
    end
    FLAG = 0; %#ok<NASGU> -- the book resets it here; kept for line fidelity
    Y   = .5*(YOLD + Y + H*YD);
    YD  = .5*(YDOLD + YD + H*YDD);
    XNL = .5*(XNLOLD + XNL + H*XNLD);
end
miss = Y;
end


function [z, tstart, coef] = mcSet(tf, runs, cfg)
%MCSET  One Monte Carlo set: RUN trials at a single flight time.
%
%   SUM=uniform; TSTART=TF*SUM; PZ=uniform; PZ=PZ-.5; if PZ > 0, COEF=1; ...
%
%   rand(2,runs) fills COLUMN MAJOR, so U(1,i) is draw 2i-1 and U(2,i) is draw
%   2i: exactly the book's (SUM, PZ) pairing per trial.  rand(runs,2) would
%   pair draw i with draw runs+i and change every number in this file without
%   changing anything that looks wrong.
U      = rand(2, runs);
tstart = (tf * U(1,:)).';
pz     = U(2,:).' - 0.5;
coef   = -ones(runs, 1);
coef(pz > 0) = 1;          % the book's strict >: PZ == 0 exactly gives -1
z      = flyVec(tf, tstart, coef, cfg, true);
end


function [xmean, sigma] = bookStats(z)
%BOOKSTATS  The listing's own post-processing, accumulation order included.
%
%   Not mean()/std().  Those would agree to ~1e-15 but they are a different
%   summation, and this file's whole claim is that it runs the book's
%   arithmetic.  The RUN == 1 branch is the book's too: it reports SIGMA = 0
%   for a single trial rather than a NaN, and that quirk is reachable from a
%   legitimate off-book call.
n     = numel(z);
xmean = NaN;
Z1    = 0;
for I = 1:n
    Z1    = z(I) + Z1;
    xmean = Z1/I;
end
sigma = 0;
Z1    = 0;
for I = 1:n
    Z1 = (z(I) - xmean)^2 + Z1;
    if I == 1
        sigma = 0;
    else
        sigma = sqrt(Z1/(I-1));
    end
end
end


function s = sigmaQuad(tf, npts, cfg, zeroBeforeStart)
%SIGMAQUAD  The Monte Carlo's own limit, with the sampling noise removed.
%
%   The loop is LINEAR and the maneuver sign is +/-1 with equal probability,
%   so the miss is COEF*m(TSTART) with m deterministic.  Hence
%       E[miss]   = 0                     (exactly, not approximately)
%       sigma^2   = E[m(TSTART)^2],  TSTART ~ U[0, tF]
%   which is an ordinary integral.  Midpoint rule, COEF fixed at +1.
%
%   This is the SAME transcribed integrator, so it is not an independent
%   check of the dynamics -- the closed form is.  What it removes is only the
%   Monte Carlo noise, which is why book VALUE rows are scored against it.
%
%   m(TSTART) is a staircase in TSTART with steps of h (the maneuver switch
%   fires on a grid step), so the quadrature carries ~0.1% jitter in npts.
%   Self-check 6 measures it rather than assuming it away.
k      = (0:npts-1).' + 0.5;
tstart = (k/npts) * tf;
m      = flyVec(tf, tstart, ones(npts,1), cfg, zeroBeforeStart);
s      = sqrt(sum(m.^2)/npts);
end


% =============================================================================
function m = closedFormUniform(nT_ftps2, T_s, tF_s)
%CLOSEDFORMUNIFORM  Zarchan Ch.4, p.85-86 (zarchan.txt:1354-1360).  The rms
%   miss due to a target maneuver with uniformly distributed starting time,
%   VALID FOR N' = 3 ONLY:
%
%       M = (nT/4) * sqrt( (T^5/tF) * [3 - exp(-2x)(2x^4+4x^3+6x^2+6x+3)] )
%       x = tF/T
%
%   Shares NO CODE with the integrator, which is the entire point of it.
%
%   THE OCR LOST TWO MINUS SIGNS, in "3 -" and in exp(-2x).  Both are
%   recovered, and not by taste: the book states the formula it came from,
%       M_UDNT = nT*sqrt( (1/tF) * integral_0^tF M_NT(t)^2 dt )
%   with M_NT/nT = 0.5*t^2*exp(-t/T) (zarchan.txt:1339-1345), and carrying out
%   that integral by parts gives (T^5/16)*[3 - e^{-2x}(2x^4+4x^3+6x^2+6x+3)]
%   -- the bracket, the 1/4 and the T^5/tF all fall out.  With '+' instead the
%   bracket tends to 6 rather than 0 as tF -> 0 and the formula would predict
%   a finite miss for a zero-length flight.
%
%   Returns FEET.  nT in ft/s^2; T and tF in seconds.
arguments
    nT_ftps2 (1,1) double {mustBeReal, mustBeFinite}
    T_s      (1,1) double {mustBePositive, mustBeFinite}
    tF_s     (1,1) double {mustBePositive, mustBeFinite}
end
x   = tF_s/T_s;
brk = 3 - exp(-2*x)*(2*x^4 + 4*x^3 + 6*x^2 + 6*x + 3);
if brk < 0
    % Cannot happen for x > 0, but the caller gets an error rather than a
    % complex "miss distance" if it ever does.
    error('mgs:zarchan:monteCarloHoming:closedFormNeg', ...
        'closed-form bracket went negative (%.3g) at tF/T = %g.', brk, x);
end
m = abs(nT_ftps2)/4 * sqrt(T_s^5/tF_s * brk);
end


function m = closedFormStep(nT_ftps2, T_s, tF_s)
%CLOSEDFORMSTEP  Zarchan Ch.3 normalization, zarchan.txt:888-890:
%
%       Miss/nT |_{N'=3} = 0.5 * tF^2 * exp(-tF/T)        [ft, ft/s^2, s]
%
%   The DETERMINISTIC parent of the closed form above: the miss for a step
%   maneuver that starts at the beginning of the flight.  Used to check the
%   integrator kernel on its own, before any statistics are laid on top -- if
%   this row fails, nothing downstream of it means anything.
%
%   It is also the reason the uniform-start case reduces to an integral: this
%   loop is time-varying only through TGO, so a maneuver starting at TSTART
%   with flight time tF is the SAME problem as a maneuver at t = 0 with flight
%   time tF - TSTART, the loop having been at rest until then.
arguments
    nT_ftps2 (1,1) double {mustBeReal, mustBeFinite}
    T_s      (1,1) double {mustBePositive, mustBeFinite}
    tF_s     (1,1) double {mustBePositive, mustBeFinite}
end
m = nT_ftps2 * 0.5 * tF_s^2 * exp(-tF_s/T_s);
end


% =============================================================================
function S = samplingNoise(z, nBook, tf)
%SAMPLINGNOISE  How noisy is a standard deviation computed from n trials?
%
%   sd(s)/sigma ~= sqrt((kappa - 1)/(4n)),  kappa = E[(z-mu)^4]/sigma^4
%
%   For a Gaussian kappa = 3 and this reduces to the familiar 1/sqrt(2n).  The
%   miss distribution here is NOT Gaussian -- it is bounded and symmetric with
%   kappa near 2.7 -- so the Gaussian rule OVERSTATES the noise by ~8%.  The
%   kurtosis is measured from the largest sample this run produced.
%
%   *** THE 32-SORTIE FIGURE IS AN EXTRAPOLATION OF THE BOOK'S DISTRIBUTION,
%   NOT OF OURS.  It is here to make the point that a spread quoted from 32
%   flights has its own error bar of order 12%, so two sortie sets differing
%   by 15% have not been shown to differ.  Recompute kappa on OUR miss
%   population before quoting it about a real sortie set. ***
z  = z(:);
n  = numel(z);
mu = sum(z)/n;
d  = z - mu;
v  = sum(d.^2)/n;
if v <= 0
    S = struct('n_sample', n, 'kurtosis', NaN, 'sd_frac_at_runs', NaN, ...
        'sd_frac_gaussian', NaN, 'sd_frac_at_32', NaN, 'runs', nBook, ...
        'tf', tf, 'formula', 'sd(s)/sigma ~= sqrt((kappa-1)/(4n))', ...
        'note', 'degenerate sample (zero variance)');
    return
end
kap = (sum(d.^4)/n) / v^2;
S = struct( ...
    'n_sample',         n, ...
    'kurtosis',         kap, ...
    'runs',             nBook, ...
    'tf',               tf, ...
    'sd_frac_at_runs',  sqrt(max(kap-1,0)/(4*nBook)), ...
    'sd_frac_gaussian', 1/sqrt(2*max(nBook-1,1)), ...
    'sd_frac_at_32',    sqrt(max(kap-1,0)/(4*32)), ...
    'formula',          'sd(s)/sigma ~= sqrt((kappa-1)/(4n))', ...
    'note',             ['kurtosis measured at the LONGEST flight time, ' ...
                         'where this estimator is noisiest']);
end


function O = ocrDiscriminator(cfg)
%OCRDISCRIMINATOR  Re-run the transcription decision instead of asserting it.
%
%   "if T , TSTART" had to be read as '<' or '>'.  Both are flown here on a
%   coarse quadrature and scored against the Ch.4 closed form.  The verdict is
%   not an opinion in a comment; it is a number that comes back in R.ocr and
%   is gated by self-check 8.
%
%   Cheap on purpose: 101 quadrature points is far more than enough to tell a
%   2% deviation from a 56% one, and this runs on every call.
npts = 101;
nTF  = numel(cfg.tf);
O = struct('tf', cfg.tf(:).', 'dev_chosen_pct', nan(1,nTF), ...
           'dev_rejected_pct', nan(1,nTF), 'max_chosen_pct', NaN, ...
           'max_rejected_pct', NaN, 'ratio', NaN, 'evaluable', false, ...
           'chosen', 'T < TSTART  (XNT = 0 BEFORE the start time)', ...
           'rejected', 'T > TSTART  (XNT = 0 AFTER the start time)');
if cfg.xnp ~= 3
    O.note = ['not evaluable: the closed form the two readings are scored ' ...
              'against is stated valid for N'' = 3 only'];
    return
end
for j = 1:nTF
    c = closedFormUniform(cfg.xnt, cfg.tau, cfg.tf(j));
    O.dev_chosen_pct(j)   = 100*(sigmaQuad(cfg.tf(j), npts, cfg, true )/c - 1);
    O.dev_rejected_pct(j) = 100*(sigmaQuad(cfg.tf(j), npts, cfg, false)/c - 1);
end
O.max_chosen_pct   = max(abs(O.dev_chosen_pct));
O.max_rejected_pct = max(abs(O.dev_rejected_pct));
O.ratio            = O.max_rejected_pct / max(O.max_chosen_pct, eps);
O.evaluable        = true;
O.note = ['the chosen reading must be the one that lands on the closed ' ...
          'form; at tF = 10 s ALONE the two are ~1% apart, so the middle ' ...
          'of the grid is what does the discriminating'];
end


% =============================================================================
function C = structuralCheck(cfg, R)
%STRUCTURALCHECK  Wiring, not the book.  A failure here voids everything above.
%
%   Every one of these is an EXACT or near-exact identity of the transcribed
%   loop, so the tolerances are 1e-12, not percentages.  Three of them (1, 2,
%   3) are bit-exact and are compared to zero with ==, deliberately: "small"
%   is what a subtly different integrator looks like.
C     = struct();
C.notes = {};
nTF   = numel(cfg.tf);
tfc   = cfg.tf(ceil(nTF/2));               % a mid-length flight
ts    = [0; 0.37; 0.50; 0.93].' * tfc;
ts    = ts(:);
co    = [1; -1; 1; -1];

% -- 1.  the vectorised loop IS the book's loop, bit for bit ---------------
mv = flyVec(tfc, ts, co, cfg, true);
ms = zeros(numel(ts),1);
for k = 1:numel(ts)
    ms(k) = flyScalar(tfc, ts(k), co(k), cfg, true);
end
C.vec_res = max(abs(mv - ms));
C.vec_tol = 0;
C.vec_ok  = C.vec_res == 0;
if ~C.vec_ok
    C.notes{end+1} = ['FLYVEC and FLYSCALAR disagree.  FLYSCALAR is the ' ...
        'book; treat every number above as void until they agree EXACTLY.'];
end

% -- 2.  the loop is linear: flipping COEF flips the miss, exactly ----------
%    Catches a maneuver switch that leaks a sign, and proves the population
%    mean is structurally zero (which is why sigmaQuad may use E[m^2]).
mb = flyVec(tfc, ts, -co, cfg, true);
C.anti_res = max(abs(mv + mb));
C.anti_tol = 0;
C.anti_ok  = C.anti_res == 0;

% -- 3.  no maneuver, no miss ----------------------------------------------
%    TSTART beyond the end of the flight must give EXACTLY zero.  This is the
%    check that fails loudly if the maneuver switch is inverted.
mz = flyVec(tfc, [2*tfc; 5*tfc], [1; -1], cfg, true);
C.zero_res = max(abs(mz));
C.zero_tol = 0;
C.zero_ok  = C.zero_res == 0;

% -- 4.  Vc CANCELS.  There is no along-track channel in this model. --------
%    XNC = XNP*VC*(RTM*YD + Y*VC)/RTM^2 with RTM = VC*TGO reduces to
%    XNP*(TGO*YD + Y)/TGO^2.  Measured at 1e-16, i.e. rounding only.
%    The consequence is the one that matters to this project: a model in
%    which closing speed provably does not matter cannot produce an ARRIVAL
%    TIME dispersion.  Every sigma here is line-of-sight normal.
cfgB      = cfg;  cfgB.vc = 10*cfg.vc;
sA        = sigmaQuad(tfc, 101, cfg,  true);
sB        = sigmaQuad(tfc, 101, cfgB, true);
C.vc_rel  = abs(sB/sA - 1);
C.vc_tol  = 1e-12;
C.vc_ok   = C.vc_rel <= C.vc_tol;

% -- 5.  sigma is exactly linear in the maneuver level ----------------------
cfgC      = cfg;  cfgC.xnt = 2.5*cfg.xnt;
sC        = sigmaQuad(tfc, 101, cfgC, true);
C.lin_rel = abs(sC/(2.5*sA) - 1);
C.lin_tol = 1e-12;
C.lin_ok  = C.lin_rel <= C.lin_tol;

% -- 6.  the quadrature has converged in npts ------------------------------
%    m(TSTART) is a staircase with steps of h, so this jitters at ~0.1%.
%    Tolerance MEASURED, not guessed: halving the point count moves the
%    answer by 7.6e-6 at tF = 5 s but by 2.7e-3 at tF = 4 s, because where the
%    staircase edges fall relative to the midpoints depends on the flight
%    time.  5e-3 covers that; anything tighter fails on an innocent grid.
sHalf      = sigmaQuad(tfc, max(2, floor(cfg.quadPoints/2)), cfg, true);
sFull      = sigmaQuad(tfc, cfg.quadPoints, cfg, true);
C.quad_rel = abs(sHalf/sFull - 1);
C.quad_tol = 5e-3;
C.quad_ok  = C.quad_rel <= C.quad_tol;

% -- 7.  the h artifact SHRINKS under refinement ---------------------------
%    The header claims the +1..2% offset from the closed form is the step
%    size and not a transcription error.  This is that claim, measured: at
%    h/2 the deviation must move TOWARD the closed form.  If it grows, the
%    offset is a bug and the header is wrong.
if cfg.xnp == 3
    cfgH        = cfg;  cfgH.h = cfg.h/2;
    cRef        = closedFormUniform(cfg.xnt, cfg.tau, tfc);
    devH        = sigmaQuad(tfc, 101, cfg,  true)/cRef - 1;
    devH2       = sigmaQuad(tfc, 101, cfgH, true)/cRef - 1;
    C.h_dev     = devH;
    C.h_dev_half= devH2;
    C.h_ok      = abs(devH2) < abs(devH) && abs(devH) < 0.05;
    C.h_skipped = false;
    C.notes{end+1} = sprintf(['h-refinement at tF = %g s: %+0.3f%% at ' ...
        'h = %g, %+0.3f%% at h/2 (first order, so the offset is the ' ...
        'terminal TGO -> 1e-5 step, not the transcription)'], tfc, ...
        100*devH, cfg.h, 100*devH2);
else
    C.h_dev = NaN;  C.h_dev_half = NaN;  C.h_ok = true;  C.h_skipped = true;
end

% -- 8.  the OCR decision, scored ------------------------------------------
if R.ocr.evaluable
    C.ocr_ratio   = R.ocr.ratio;
    C.ocr_tol     = 5;          % the rejected reading must be >=5x worse
    C.ocr_ok      = R.ocr.ratio >= C.ocr_tol && R.ocr.max_chosen_pct < 5;
    C.ocr_skipped = false;
else
    C.ocr_ratio = NaN; C.ocr_tol = NaN; C.ocr_ok = true; C.ocr_skipped = true;
end

% -- 9.  the Monte Carlo converges on the quadrature -----------------------
%    Band from the estimator's own noise: at n = convergeRuns the sampling sd
%    is sd50/sqrt(n/50), about 1.5% at n = 1000, so 10% is >6 sigma.  This is
%    the row that proves the RANDOM part (TSTART = TF*u, the +/-1 sign, the
%    draw pairing) and not just the deterministic kernel.
if cfg.convergeRuns > 0
    C.conv_rel     = max(abs(R.converge.dev_pct))/100;
    C.conv_tol     = 0.10;
    C.conv_ok      = C.conv_rel <= C.conv_tol;
    C.mean_rel     = R.converge.mean_over_sig;
    C.mean_tol     = 0.20;
    C.mean_ok      = C.mean_rel <= C.mean_tol;
    C.conv_skipped = false;
else
    C.conv_rel = NaN; C.conv_tol = NaN; C.conv_ok = true;
    C.mean_rel = NaN; C.mean_tol = NaN; C.mean_ok = true;
    C.conv_skipped = true;
    C.notes{end+1} = ['convergeRuns = 0: the RANDOM half of this port is ' ...
        'UNCHECKED.  The kernel rows still bite; the draw machinery does not.'];
end

% -- 10.  the maneuver sign really took both values ------------------------
%    Guards the one bug the anti-symmetry check cannot see: a COEF that is
%    always +1.  With 50 trials the count is 25 +/- 3.5, so "both present" is
%    a formality -- but a formality that is absent is how ports get away with
%    a broken sign for three waves.
C.sign_pos  = nnz(R.coef(:) > 0);
C.sign_neg  = nnz(R.coef(:) < 0);
C.sign_ok   = C.sign_pos > 0 && C.sign_neg > 0;

oks = [C.vec_ok C.anti_ok C.zero_ok C.vc_ok C.lin_ok C.quad_ok C.h_ok ...
         C.ocr_ok C.conv_ok C.mean_ok C.sign_ok];
names = {'1 vectorised == scalar (bit exact)', '2 anti-symmetry in COEF', ...
         '3 no maneuver -> no miss', '4 Vc cancels', ...
         '5 linear in nT', '6 quadrature converged in npts', ...
         '7 h artifact shrinks under h/2', '8 OCR reading discriminated', ...
         '9 Monte Carlo converges on quadrature', ...
         '10 |mean|/sigma small at convergeRuns', '11 COEF took both signs'};
C.ok = all(oks);
if C.ok
    C.first_fail = '';
else
    C.first_fail = names{find(~oks, 1)};
end
C.names = names;
C.oks = oks;
end


% =============================================================================
function B = benchmarks(R, cfg, offbook, GRAV_FT)
%BENCHMARKS  Computed vs published, with a band and the PROVENANCE of it.
%
%   *** EVERY BOOK VALUE HERE WAS GREPPED OUT OF zarchan.txt AND THE LINE
%   NUMBER IS IN THE .source FIELD. ***  The extraction turned every figure
%   into an image link, so nothing that lives only on a plot survives into the
%   text.  Fig. 4.16 is the picture this listing draws and it is GONE: there
%   is no published peak sigma to score against, and the INFO row below says
%   so rather than inventing one.  What the book DOES say about that figure in
%   prose ("the miss distance is small for both small and large flight times")
%   is greppable, and an ordering check on it is stronger than a value
%   eyeballed off a plot anyway.
%
%   BANDS AND WHERE THEY COME FROM -- none of them was widened to make a row
%   green, and each names the effect that sets it:
%
%     'config'      exact    the listing's own constants (RUN = 50, 500 runs)
%     'text-h'      3%       a value printed in the running text to three
%                            figures, compared against this file's h = 0.01
%                            integration.  3 figures is +/-0.4% of quantization;
%                            the terminal-step artifact documented in the
%                            header is a further ~2% and self-check 7 MEASURES
%                            it.  0.4 + 2 rounded out = 3.
%     'analytic-h'  3%       against a closed-form FORMULA from the book,
%                            evaluated here.  Same artifact, same arithmetic.
%     'shape'       n/a      a prose claim about a figure, scored as an
%                            ordering, on the NOISE-FREE curve
%     'shape-mc'    n/a      the same claim scored on the 50-run curve, i.e.
%                            on Fig. 4.16 itself.  Carries sampling noise; see
%                            the row's own detail line for the measured
%                            failure rate.
%     'axis'        n/a      the listing's own axis([00,10,00,30]) -- code,
%                            not a figure reading -- scored as a bracket
%     'monte-carlo' 35%      the 50-run estimator's OWN spread -- nothing to
%                            do with the port.  Measured over 200,000
%                            replicate 50-run sets: sd 9.5% of the true sigma
%                            at tF = 10 s, with 0.1%/99.9% quantiles of the
%                            ratio to 13.3 ft at 0.69x/1.29x.  35% is 3.5
%                            measured sd, which puts a correct port's failure
%                            rate below 1e-3.  This row is WEAK BY
%                            CONSTRUCTION and is labelled so; the sharp
%                            evidence for 13.3 ft is the 'text-h' row above.
B = emptyBench();

% What each family of rows needs before it is allowed to score.
n3       = (cfg.xnp == 3);
bookGrid = isequal(cfg.tf, 1:10);
bookDyn  = n3 && cfg.tau == 1 && abs(cfg.xnt - 96.6) < 1e-9 && cfg.h == 0.01;
bookCase = bookDyn && bookGrid && cfg.runs == 50 && cfg.vc == 4000;
whyDyn   = ['off the listing''s dynamics (needs N'' = 3, tau = 1 s, ' ...
            'nT = 96.6 ft/s^2, h = 0.01 s)'];
whyCase  = ['off the listing''s published case (needs the dynamics above ' ...
            'plus tF = 1:10 s, RUN = 50, Vc = 4000 ft/s)'];
j10      = find(cfg.tf == 10, 1);

% ---- 1.  THE SET SIZE ----------------------------------------------------
% zarchan.txt:1298 "RUN=50"; :1278 "The inner loop performs 50 sets of runs";
% :1095 "Usually we will consider 50 runs to be sufficient in the tradeoff
% between computer running time and numerical accuracy."
i = numel(B) + 1;
B(i) = mkbench('trials per Monte Carlo set', 50, 'runs', 'config', 0, ...
               'listing + p.65,68 text (zarchan.txt:1298, :1095, :1278)');
B(i) = scoreNum(B(i), cfg.runs, 'runs', ...
    ['the number the book calls "sufficient"; R.sampling prices what that ' ...
     'buys you in estimator noise']);

% ---- 2.  THE TOTAL, WHICH THE BOOK STATES ARITHMETICALLY -----------------
% zarchan.txt:1279 "Fifty-run Monte Carlo sets for 10 different flight times
% were run for this single case, which actually encompassed a total of 500
% runs (50 10)."  The "(50 10)" is "(50 x 10)" with the multiply lost in
% extraction; the 500 is unambiguous and is what is scored.
i = numel(B) + 1;
B(i) = mkbench('runs in the published experiment', 500, 'runs', 'config', 0, ...
               'p.68 text (zarchan.txt:1279)');
B(i) = scoreNum(B(i), numel(cfg.tf)*cfg.runs, 'runs', ...
    sprintf('grid = %s s (book 1:10, "1 to 10 s in increments of 1 s")', ...
            mat2str(cfg.tf)));

% ---- 3.  THE HEADLINE VALUE ---------------------------------------------
% zarchan.txt:1536 -- "the covariance analysis program indicates that the
% standard deviation of the miss distance is 13.3 ft, which is identical to
% the adjoint results of Chapter 4 (see Fig. 4.18)."  Fig. 4.18 is the figure
% that superimposes THIS listing's Monte Carlo on that adjoint, and the text
% at :1320 says the two "are in close proximity" -- so 13.3 ft is the number
% this Monte Carlo is supposed to land on.  Scored on the NOISE-FREE curve.
i = numel(B) + 1;
B(i) = mkbench('sigma_miss at tF = 10 s', 13.3, 'ft', 'text-h', 0.03, ...
               'p.98 text (zarchan.txt:1536), cross-ref Fig. 4.18 (:1320)');
if bookDyn && ~isempty(j10) && cfg.quadPoints >= 101
    B(i) = scoreNum(B(i), R.sigma_quad(j10), 'ft', ...
        sprintf(['noise-free (%d-point) evaluation of the book''s own ' ...
                 'model; the closed form for the same case is %.4f ft'], ...
                cfg.quadPoints, R.sigma_closed(j10)));
else
    B(i) = markSkipped(B(i), [whyDyn ', with tF = 10 s in the grid and ' ...
                              'quadPoints >= 101']);
end

% ---- 4.  THE SECOND OPINION, ACROSS THE WHOLE CURVE ---------------------
% zarchan.txt:1354-1360, the Ch.4 closed form for N' = 3.  It shares no code
% with the integrator, so this is the row with real teeth: a transcription
% error anywhere in the guidance law, the lag or the corrector moves it, and
% moves it differently at different flight times.
i = numel(B) + 1;
% NaN as the "book value", not 0: this row is scored as a THRESHOLD on a
% derived quantity, and a skipped row that printed "book 0 %" would read as
% though the book had published a zero.
B(i) = mkbench('max |sigma - closed form| over the grid', NaN, '%', ...
               'analytic-h', 0.03, 'p.85-86 closed form (zarchan.txt:1354)');
if n3 && cfg.quadPoints >= 101
    dev = abs(R.dev_quad_pct)/100;
    [mx, jm] = max(dev);
    sgn = 'mixed';
    if all(R.dev_quad_pct > 0), sgn = 'all POSITIVE'; end
    if all(R.dev_quad_pct < 0), sgn = 'all NEGATIVE'; end
    B(i) = scoreShape(B(i), mx <= 0.03, '<= 3.0%', ...
        sprintf('%.3f%% (worst at tF = %g s)', 100*mx, cfg.tf(jm)), ...
        sprintf(['deviations are %s -- a uniform offset is the h = %g ' ...
                 'terminal-step artifact (self-check 7); a transcription ' ...
                 'error would scatter with flight time'], sgn, cfg.h));
    B(i).ours = 100*mx;  B(i).pct = NaN;
else
    B(i) = markSkipped(B(i), ['the closed form is stated valid for N'' = 3 ' ...
        'only, and needs quadPoints >= 101']);
end

% ---- 5.  THE KERNEL ON ITS OWN, BEFORE ANY STATISTICS -------------------
% zarchan.txt:888-890, Miss/nT|N'=3 = 0.5*tF^2*exp(-tF/T).  Scored at the
% flight time where that miss is largest (tF = 2T), so the comparison is not
% dominated by a number close to zero.  If THIS fails, every sigma above is
% meaningless regardless of how the statistics were done.
i = numel(B) + 1;
tfStep = 2*cfg.tau;
B(i) = mkbench('step maneuver at t=0, tF = 2*tau', NaN, 'ft', 'analytic-h', ...
               0.03, 'p.55 normalization (zarchan.txt:888-890)');
if n3 && tfStep >= 10*cfg.h
    cRef  = closedFormStep(cfg.xnt, cfg.tau, tfStep);
    ours  = flyVec(tfStep, 0, 1, cfg, true);
    B(i).book   = cRef;
    B(i).book_str = sprintf('%.4g ft', cRef);
    B(i) = scoreNum(B(i), ours, 'ft', ...
        sprintf(['one deterministic trial, TSTART = 0, tF = %g s -- the ' ...
                 'peak of 0.5*tF^2*exp(-tF/T)'], tfStep));
else
    B(i).book_str = '0.5*nT*tF^2*exp(-tF/tau)';   % the formula, since no number
    B(i) = markSkipped(B(i), 'needs N'' = 3 and tF = 2*tau >= 10 integration steps');
end

% ---- 6.  THE SHAPE CLAIM, ON THE NOISE-FREE CURVE -----------------------
% zarchan.txt:1279 -- "We can see that the miss distance is small for both
% small and large flight times.  The miss is small for short flight times
% because the miss distance does not have enough time to develop.  At the
% larger flight times there is a good chance that the target maneuver
% initiation time will be at a long time to go ... and will therefore induce a
% smaller miss distance."  Prose about Fig. 4.16, so it is greppable.  The
% claim is that the maximum is INTERIOR: both ends below the peak.
i = numel(B) + 1;
B(i) = mkbench('sigma small at both ends (exact model)', NaN, '', 'shape', ...
               NaN, 'p.68 text (zarchan.txt:1279)');
if bookGrid && numel(cfg.tf) >= 3
    B(i) = shapeInterior(B(i), R.sigma_quad, cfg.tf, ...
        'the quadrature curve: the book''s model with the sampling noise out');
else
    B(i) = markSkipped(B(i), ['the sentence is about the book''s 1:10 s ' ...
        'grid; it says nothing about an arbitrary grid']);
end

% ---- 7.  THE SAME CLAIM ON FIG. 4.16 ITSELF -----------------------------
%   Fig. 4.16 IS the 50-run curve, so the claim has to survive on it too.
%   Measured failure rate: 0 in 200,000 replicate 50-run sets.  If this row
%   fails while row 6 passes, the seed is the culprit and not the port -- that
%   is precisely why the two rows are separate.
i = numel(B) + 1;
B(i) = mkbench('sigma small at both ends (the 50-run set)', NaN, '', ...
               'shape-mc', NaN, 'Fig. 4.16 / p.68 text (zarchan.txt:1279)');
if bookCase
    B(i) = shapeInterior(B(i), R.sigma, cfg.tf, ...
        sprintf(['the RUN = %d curve, seed %d.  Sampling noise here is ' ...
                 '~%.1f%% of sigma; the interior maximum survived ' ...
                 '200000/200000 replicate sets in the measurement behind ' ...
                 'this row'], cfg.runs, cfg.seed, ...
                 100*R.sampling.sd_frac_at_runs));
else
    B(i) = markSkipped(B(i), whyCase);
end

% ---- 8.  THE AUTHOR'S OWN AXIS, SCORED AS A BRACKET ---------------------
% zarchan.txt:1312 -- axis([00,10,00,30]) in the listing itself, repeated in
% the adjoint of Listing 4.6 (:1337).  This is CODE, not a figure reading:
% the author chose a 30 ft ceiling knowing what the curve does, so a run whose
% sigma pokes through 30 would not have fitted in his own picture.  Largest
% sigma seen anywhere over 200,000 replicate 50-run sets was 24.3 ft.
i = numel(B) + 1;
B(i) = mkbench('every sigma fits the listing''s 30 ft axis', NaN, '', ...
               'axis', NaN, 'listing''s own axis (zarchan.txt:1312, :1337)');
if bookCase
    mx = max(R.sigma);
    B(i) = scoreShape(B(i), mx < 30 && min(R.sigma) >= 0, ...
        '0 <= sigma < 30 ft', sprintf('max %.2f ft at tF = %g s', mx, ...
        cfg.tf(find(R.sigma == mx, 1))), ...
        ['a bracket, not a value -- it says the curve fits the picture, ' ...
         'nothing more']);
else
    B(i) = markSkipped(B(i), whyCase);
end

% ---- 9.  THE 50-RUN SET AGAINST THE PUBLISHED NUMBER --------------------
%   Deliberately kept, deliberately WEAK, and labelled: this is what the book
%   actually ran, and a 50-run set simply cannot resolve 13.3 ft to better
%   than tens of percent.  It catches gross breakage and nothing finer.  The
%   sharp version of this comparison is row 3.
i = numel(B) + 1;
B(i) = mkbench('sigma_miss at tF = 10 s, RUN = 50', 13.3, 'ft', ...
               'monte-carlo', 0.35, ...
               'p.98 text (zarchan.txt:1536) vs Fig. 4.18 (:1320)');
if bookCase && ~isempty(j10)
    B(i) = scoreNum(B(i), R.sigma(j10), 'ft', ...
        sprintf(['WEAK BY CONSTRUCTION: 50 trials give sd ~%.1f%% on this ' ...
                 'estimator (measured 0.1%%/99.9%% band 0.69x..1.29x).  ' ...
                 'The quadrature row above is the evidence; this row is a ' ...
                 'tripwire'], 100*R.sampling.sd_frac_at_runs));
else
    B(i) = markSkipped(B(i), whyCase);
end

% ---- INFO rows.  Reported, never scored. --------------------------------
i = numel(B) + 1;
[pk, ipk] = max(R.sigma_quad);
B(i) = mkinfo('peak sigma over the grid', ...
    sprintf('%.2f ft at tF = %g s (50-run set: %.2f ft at tF = %g s)', ...
            pk, cfg.tf(ipk), max(R.sigma), cfg.tf(find(R.sigma == max(R.sigma),1))), ...
    'Fig. 4.16 shape only -- NO published peak value survived extraction', ...
    ['NOT SCORED.  Fig. 4.16 became an image link, so any "book peak" ' ...
     'quoted for this listing would be fabricated or lifted from another ' ...
     'chapter.  The prose claim about that figure is scored instead (rows 6-7).']);

i = numel(B) + 1;
B(i) = mkinfo('adjoint vs Monte Carlo cost', ...
    sprintf('this run: %d Monte Carlo trials for the same curve', ...
            numel(cfg.tf)*cfg.runs), ...
    'p.83 text (zarchan.txt:1328)', ...
    ['NOT SCORED: "one single adjoint run gave results that were the ' ...
     'equivalent of ten 50-run Monte Carlo sets" is a statement about ' ...
     'compute, not a number this file can reproduce.  The listing''s own ' ...
     'adjoint twin is Listing 4.6 (zarchan.txt:1331), not yet ported.']);

i = numel(B) + 1;
B(i) = mkinfo('estimator noise (the transferable result)', ...
    sprintf('kurtosis %.3f -> sd(s)/sigma = %.1f%% at n = %d, %.1f%% at n = 32', ...
            R.sampling.kurtosis, 100*R.sampling.sd_frac_at_runs, ...
            R.sampling.runs, 100*R.sampling.sd_frac_at_32), ...
    'derived here, not published: sd(s)/sigma ~= sqrt((kappa-1)/(4n))', ...
    ['the n = 32 figure extrapolates THE BOOK''S miss distribution, not ' ...
     'ours.  Recompute kappa on our own sortie population before quoting ' ...
     'an error bar on a real spread.']);

i = numel(B) + 1;
if R.converge.runs > 0
    B(i) = mkinfo('Monte Carlo -> quadrature', ...
        sprintf('%d runs: worst deviation %+.2f%%, |mean|/sigma %.3f', ...
                R.converge.runs, maxSigned(R.converge.dev_pct), ...
                R.converge.mean_over_sig), ...
        'no published value: this is the port checking its own draws', ...
        ['the deterministic kernel is checked by rows 4-5; THIS is what ' ...
         'checks TSTART = TF*u, the +/-1 sign and the (SUM,PZ) draw pairing']);
else
    B(i) = mkinfo('Monte Carlo -> quadrature', 'SKIPPED (convergeRuns = 0)', ...
        'no published value', 'the random half of the port is unchecked');
end

i = numel(B) + 1;
if R.ocr.evaluable
    B(i) = mkinfo('OCR reading of "if T , TSTART"', ...
        sprintf('T < TSTART: max %.2f%% off closed form | T > TSTART: max %.2f%%', ...
                R.ocr.max_chosen_pct, R.ocr.max_rejected_pct), ...
        'transcription decision, re-measured every call', ...
        ['the chosen reading is T < TSTART (target is quiet BEFORE the ' ...
         'start time).  At tF = 10 s alone the two readings are ~1% apart ' ...
         '-- the middle of the grid is what tells them apart.']);
else
    B(i) = mkinfo('OCR reading of "if T , TSTART"', 'not evaluable (N'' ~= 3)', ...
        'transcription decision', 'the closed form it is scored against needs N'' = 3');
end

i = numel(B) + 1;
B(i) = mkinfo('sample mean (the book''s ArrayXMEAN)', ...
    sprintf('max |mean| = %.3f ft, max |mean|/sigma = %.3f', ...
            max(abs(R.xmean)), max(abs(R.xmean)./max(R.sigma, eps))), ...
    'plotted by the listing (ylabel "Standard Deviation / Mean")', ...
    ['the TRUE mean is exactly zero here -- COEF is +/-1 with equal ' ...
     'probability and the loop is linear (self-check 2) -- so this curve ' ...
     'is a sampling-noise diagnostic, not a result.  Expect ~1/sqrt(n).']);

i = numel(B) + 1;
B(i) = mkinfo('VM = 3000 ft/s', sprintf('%g ft/s, carried and UNUSED', cfg.vm), ...
    'listing''s data statement (zarchan.txt:1296)', ...
    ['Listing 4.5 sets VM and never reads it: this is a linearised loop ' ...
     'in which only the CLOSING speed appears, and even that cancels ' ...
     '(self-check 4).  Kept so the data statement is complete.']);

i = numel(B) + 1;
B(i) = mkinfo('units tripwire: nT = 96.6 ft/s^2', ...
    sprintf('%.4g ft/s^2 = %.4g g at g = %g', cfg.xnt, cfg.xnt/GRAV_FT, GRAV_FT), ...
    'listing''s data statement (zarchan.txt:1296)', ...
    ['3 g x 32.2 = 96.6.  If this ever reads 29.4 (3 x 9.81) the file has ' ...
     'been converted to SI and every miss distance above is wrong by 3.28x ' ...
     'while still looking entirely plausible.']);

if ~isempty(offbook)
    i = numel(B) + 1;
    B(i) = mkinfo('OFF-BOOK PARAMETERS', offbookStr(offbook), ...
        'the caller moved these', ...
        'rows that depend on them were SKIPPED, not silently re-based');
end
end


function b = shapeInterior(b, sig, tf, detail)
%SHAPEINTERIOR  "small for both small and large flight times" as an ordering.
[~, k] = max(sig);
ok = k > 1 && k < numel(sig);
b = scoreShape(b, ok, 'max is INTERIOR (both ends below the peak)', ...
    sprintf('peak %.2f ft at tF = %g s; ends %.2f / %.2f ft', ...
            sig(k), tf(k), sig(1), sig(end)), detail);
end


function v = maxSigned(x)
%MAXSIGNED  The largest deviation, keeping its sign (for reporting only).
[~, k] = max(abs(x));
if isempty(k), v = NaN; else, v = x(k); end
end


% =============================================================================
function B = emptyBench()
B = struct('what',{}, 'kind',{}, 'book',{}, 'book_str',{}, 'ours',{}, 'ours_str',{}, ...
           'pct',{}, 'band',{}, 'tol',{}, 'ok',{}, 'skipped',{}, ...
           'scored',{}, 'why',{}, 'detail',{}, 'source',{}, 'units',{});
end

function b = mkbench(what, book, units, band, tol, source)
%MKBENCH  Field names and ORDER must match emptyBench exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want: a struct array
%   that quietly reordered fields would make [B.ok] mean something else.
b = struct('what',what, 'kind','value', 'book',book, 'book_str','', 'ours',NaN, 'ours_str','', ...
           'pct',NaN, 'band',band, 'tol',tol, 'ok',false, 'skipped',false, ...
           'scored',true, 'why','', 'detail','', 'source',source, 'units',units);
if isfinite(book)
    b.book_str = sprintf('%.4g %s', book, units);
else
    b.book_str = '(shape, see verdict)';
end
end

function b = mkinfo(what, ours_str, source, detail)
% ⚠️ 'kind' 必須在這裡也有,而且順序與 emptyBench 逐字相同 —— 少一個欄位
%    或順序不同,`B(i) = ...` 就丟
%    "Subscripted assignment between dissimilar structures"。
%    ⭐ 那正是要的行為:悄悄重排欄位的 struct array 會讓 [B.ok] 變成別的意思。
b = struct('what',what, 'kind','info', 'book',NaN, 'book_str','--', 'ours',NaN, ...
           'ours_str',ours_str, 'pct',NaN, 'band','info', 'tol',NaN, ...
           'ok',false, 'skipped',false, 'scored',false, 'why','', ...
           'detail',detail, 'source',source, 'units','');
end

function b = scoreNum(b, ours, units, detail)
if ~isfinite(b.book)
    error('mgs:zarchan:monteCarloHoming:bench', ...
        ['benchmark "%s" has no numeric book value to score against, so ' ...
         'scoring it as a number would invent one.'], b.what);
end
b.ours   = ours;
b.ours_str = sprintf('%.4f %s', ours, units);
if b.book == 0
    b.pct = NaN;
    b.ok  = abs(ours) <= b.tol;
else
    b.pct = 100*(ours/b.book - 1);
    b.ok  = abs(ours/b.book - 1) <= b.tol;
end
b.detail = detail;
end

function b = scoreShape(b, ok, wantStr, gotStr, detail)
b.book_str = wantStr;
b.ours_str = gotStr;
b.ok     = logical(ok);
b.detail = detail;
end

function b = markSkipped(b, why)
% Skipped rows are NOT passes.  They are excluded from R.pass entirely, and
% "nothing was evaluated" is an error, not a green light.
b.ours = NaN;  b.ours_str = 'skipped';  b.pct = NaN;
b.ok   = false;  b.skipped = true;  b.why = why;
end

function s = offbookStr(offbook)
if isempty(offbook)
    s = 'nothing (this is the book''s own configuration)';
else
    s = strjoin(offbook, '; ');
end
end


% =============================================================================
function s = render(R, cfg, offbook, GRAV_FT)
W = 100;
L = {};
L{end+1} = 'Zarchan Listing 4.5 -- MONTE CARLO OF THE HOMING LOOP, RANDOM TARGET MANEUVER';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['RUN = %d   tF = %s s   tau = %g s   N'' = %g   ' ...
    'Vc = %g ft/s   nT = %g ft/s^2 (%.4g g)   h = %g s'], cfg.runs, ...
    mat2str(cfg.tf), cfg.tau, cfg.xnp, cfg.vc, cfg.xnt, cfg.xnt/GRAV_FT, cfg.h);
L{end+1} = sprintf(['seed = %d (%s), stream restored on exit; draws are ' ...
    'rand(2,RUN) COLUMN MAJOR = (SUM,PZ) per trial'], cfg.seed, cfg.generator);
L{end+1} = sprintf(['integrator: the listing''s own 2nd-order Runge-Kutta ' ...
    '(Heun), transcribed; quadrature %d pts; convergence set %d runs'], ...
    cfg.quadPoints, R.converge.runs);
L{end+1} = sprintf('off-book: %s', offbookStr(offbook));
L{end+1} = repmat('-', 1, W);

% ---- self-check block ----------------------------------------------------
if isfield(R.selfcheck, 'ok')
    C = R.selfcheck;
    L{end+1} = 'STRUCTURAL SELF-CHECK  (wiring, not the book -- a failure here voids everything)';
    L{end+1} = sprintf('  %-42s res %10.3e  tol %8.1e  %s', C.names{1}, ...
        C.vec_res, C.vec_tol, tf2s(C.vec_ok));
    L{end+1} = sprintf('  %-42s res %10.3e  tol %8.1e  %s', C.names{2}, ...
        C.anti_res, C.anti_tol, tf2s(C.anti_ok));
    L{end+1} = sprintf('  %-42s res %10.3e  tol %8.1e  %s', C.names{3}, ...
        C.zero_res, C.zero_tol, tf2s(C.zero_ok));
    L{end+1} = sprintf('  %-42s rel %10.3e  tol %8.1e  %s   <- no along-track channel', ...
        C.names{4}, C.vc_rel, C.vc_tol, tf2s(C.vc_ok));
    L{end+1} = sprintf('  %-42s rel %10.3e  tol %8.1e  %s', C.names{5}, ...
        C.lin_rel, C.lin_tol, tf2s(C.lin_ok));
    L{end+1} = sprintf('  %-42s rel %10.3e  tol %8.1e  %s', C.names{6}, ...
        C.quad_rel, C.quad_tol, tf2s(C.quad_ok));
    if C.h_skipped
        L{end+1} = sprintf('  %-42s SKIPPED (needs N'' = 3)', C.names{7});
    else
        L{end+1} = sprintf('  %-42s %+7.3f%% -> %+7.3f%% at h/2      %s', ...
            C.names{7}, 100*C.h_dev, 100*C.h_dev_half, tf2s(C.h_ok));
    end
    if C.ocr_skipped
        L{end+1} = sprintf('  %-42s SKIPPED (needs N'' = 3)', C.names{8});
    else
        L{end+1} = sprintf('  %-42s rejected reading is %6.1fx worse (need >= %gx)  %s', ...
            C.names{8}, C.ocr_ratio, C.ocr_tol, tf2s(C.ocr_ok));
    end
    if C.conv_skipped
        L{end+1} = sprintf('  %-42s SKIPPED (convergeRuns = 0)', C.names{9});
        L{end+1} = sprintf('  %-42s SKIPPED (convergeRuns = 0)', C.names{10});
    else
        L{end+1} = sprintf('  %-42s rel %10.3e  tol %8.1e  %s', C.names{9}, ...
            C.conv_rel, C.conv_tol, tf2s(C.conv_ok));
        L{end+1} = sprintf('  %-42s rel %10.3e  tol %8.1e  %s', C.names{10}, ...
            C.mean_rel, C.mean_tol, tf2s(C.mean_ok));
    end
    L{end+1} = sprintf('  %-42s %d plus / %d minus                        %s', ...
        C.names{11}, C.sign_pos, C.sign_neg, tf2s(C.sign_ok));
    for k = 1:numel(C.notes)
        L{end+1} = ['    note: ' C.notes{k}]; %#ok<AGROW>
    end
else
    L{end+1} = 'STRUCTURAL SELF-CHECK  ***SKIPPED*** (selfcheck=false) -- nothing here is verified';
end
L{end+1} = repmat('-', 1, W);

% ---- benchmark board -----------------------------------------------------
if isempty(R.bench)
    L{end+1} = 'BOOK BENCHMARKS  ***SKIPPED*** (benchmark=false) -- R.pass is FALSE, not true';
else
    L{end+1} = 'BOOK BENCHMARKS  (every book value was grepped from zarchan.txt; the line is in ''source'')';
    L{end+1} = sprintf('  %-40s %-24s %-30s %-12s %s', 'what', 'book', 'ours', 'band', 'verdict');
    for i = 1:numel(R.bench)
        b = R.bench(i);
        if b.skipped
            L{end+1} = sprintf('  %-40s %-24s %-30s %-12s SKIP', ...
                b.what, b.book_str, 'skipped', b.band); %#ok<AGROW>
            L{end+1} = sprintf('  %-40s   why: %s', '', b.why); %#ok<AGROW>
        elseif ~b.scored
            L{end+1} = sprintf('  %-40s %-24s %-30s %-12s INFO', ...
                b.what, b.book_str, b.ours_str, b.band); %#ok<AGROW>
        elseif isfinite(b.pct)
            L{end+1} = sprintf('  %-40s %-24s %-30s %-12s %s  (%+.2f%%, tol %.1f%%)', ...
                b.what, b.book_str, b.ours_str, b.band, tf2s(b.ok), b.pct, 100*b.tol); %#ok<AGROW>
        else
            L{end+1} = sprintf('  %-40s %-24s %-30s %-12s %s', ...
                b.what, b.book_str, b.ours_str, b.band, tf2s(b.ok)); %#ok<AGROW>
        end
        if ~isempty(b.detail)
            L{end+1} = sprintf('  %-40s   %s', '', b.detail); %#ok<AGROW>
        end
        L{end+1} = sprintf('  %-40s   source: %s', '', b.source); %#ok<AGROW>
    end
    L{end+1} = '  bands: config (exact) | text-h 3% (3 figures in the text, vs h=0.01 integration)';
    L{end+1} = '  | analytic-h 3% (book formula, same artifact) | shape / shape-mc (prose claim as an';
    L{end+1} = '  ordering, noise-free / on the 50-run curve) | axis (the listing''s own axis, a';
    L{end+1} = '  bracket) | monte-carlo 35% (the 50-run estimator''s own measured spread, WEAK)';
end
L{end+1} = repmat('-', 1, W);

% ---- the three curves ----------------------------------------------------
L{end+1} = sprintf('SIGMA vs FLIGHT TIME   [ft]   (RUN = %d, seed %d)', cfg.runs, cfg.seed);
L{end+1} = sprintf('  %6s %14s %14s %14s %12s %14s', 'tF [s]', 'MC sigma', ...
    'quadrature', 'closed form', 'quad-cf %', 'MC mean');
for j = 1:numel(cfg.tf)
    if isnan(R.sigma_closed(j))
        cfs = '           n/a';  dvs = '         n/a';
    else
        cfs = sprintf('%14.4f', R.sigma_closed(j));
        dvs = sprintf('%+12.3f', R.dev_quad_pct(j));
    end
    L{end+1} = sprintf('  %6g %14.4f %14.4f %s %s %14.4f', cfg.tf(j), ...
        R.sigma(j), R.sigma_quad(j), cfs, dvs, R.xmean(j)); %#ok<AGROW>
end
if R.converge.runs > 0
    L{end+1} = sprintf(['  convergence set (%d runs): worst deviation from the ' ...
        'quadrature %+.2f%%'], R.converge.runs, maxSigned(R.converge.dev_pct));
end
L{end+1} = sprintf(['  estimator noise: kurtosis %.3f -> sd(sigma_hat)/sigma = ' ...
    '%.1f%% at n = %d (Gaussian rule would say %.1f%%)'], R.sampling.kurtosis, ...
    100*R.sampling.sd_frac_at_runs, R.sampling.runs, ...
    100*R.sampling.sd_frac_gaussian);
L{end+1} = repmat('-', 1, W);

L{end+1} = 'READ THIS BEFORE QUOTING ANY NUMBER ABOVE';
L{end+1} = '  * The "MC sigma" column is Fig. 4.16 and carries its own sampling noise, printed';
L{end+1} = '    above.  The "quadrature" column is the same model without that noise and is what';
L{end+1} = '    the book VALUE rows are scored against.  Quote the one you mean, by name.';
L{end+1} = '  * Vc CANCELS EXACTLY out of this loop (self-check 4).  A model in which closing';
L{end+1} = '    speed provably does not matter has no along-track channel, so every sigma here is';
L{end+1} = '    LINE-OF-SIGHT NORMAL MISS DISTANCE in feet.  It is NOT an arrival-time';
L{end+1} = '    dispersion.  Do not price a 3 s arrival spread against it.';
L{end+1} = '  * What DOES transfer is the estimator-noise line: a spread computed from n flights';
L{end+1} = '    has an error bar of order sqrt((kappa-1)/4n).  On 32 sorties that is ~12% for the';
L{end+1} = '    BOOK''S distribution -- recompute kappa on ours before quoting it.';
if any(isfinite(R.dev_quad_pct))
    L{end+1} = sprintf(['  * The %+.2f%% sitting between the quadrature and ' ...
        'the closed form is the STEP SIZE'], maxSigned(R.dev_quad_pct));
    L{end+1} = sprintf(['    (h = %g, the book''s own), not a transcription ' ...
        'error: self-check 7 shows it'], cfg.h);
    L{end+1} = '    halving under h/2.  Do not change h to make it go away -- Fig. 4.16 is a';
    L{end+1} = '    picture of h = 0.01.';
else
    L{end+1} = '  * No closed form was evaluated (it is stated valid for N'' = 3 only), so the';
    L{end+1} = '    quadrature column has NO independent check behind it on this run.';
end
L{end+1} = '  * The only error source is the target maneuver.  No gravity, no drag, no thrust, no';
L{end+1} = '    nz clamp, no LOS floor, and the single lag NEVER SATURATES.  Our own flight data';
L{end+1} = '    says a saturating ~8 s lag dominates.  A miss distance off this board is not a';
L{end+1} = '    sortie prediction.';
s = strjoin(L, newline);
end


function drawFigures(R, cfg, GRAV_FT)
%DRAWFIGURES  Fig. 4.16, with the book's own axis limits, plus Fig. 4.18's
%   overlay of the analytic answer on the Monte Carlo scatter.
figure('Name', 'Zarchan Fig. 4.16 -- Monte Carlo, uniformly distributed maneuver');
plot(R.tf, R.sigma, 'c+', 'MarkerSize', 10, 'LineWidth', 1.4); hold on;
plot(R.tf, R.sigma_quad, 'k-', 'LineWidth', 1.2);
if any(isfinite(R.sigma_closed))
    plot(R.tf, R.sigma_closed, 'r--', 'LineWidth', 1.2);
    legend({sprintf('Monte Carlo, RUN = %d (Fig. 4.16)', cfg.runs), ...
            'same model, sampling noise removed', ...
            'Ch.4 closed form (N'' = 3)'}, 'Location', 'best');
else
    legend({sprintf('Monte Carlo, RUN = %d', cfg.runs), ...
            'same model, sampling noise removed'}, 'Location', 'best');
end
grid on;
xlabel('Time'); ylabel('Standard Deviation / Mean');
title(sprintf(['Shaping filter Monte Carlo results -- tau = %g s, ' ...
    '%.4g-g uniformly distributed maneuver'], cfg.tau, cfg.xnt/GRAV_FT));
% axis([00,10,00,30]) is the BOOK'S OWN LINE (zarchan.txt:1312).  Applied only
% when the grid is the book's, so an off-book sweep is not silently cropped
% out of its own picture -- a cropped curve and a small curve look identical.
if isequal(cfg.tf, 1:10)
    axis([0 10 0 30]);
else
    title(sprintf(['Shaping filter Monte Carlo results (OFF-BOOK GRID; the ' ...
        'listing''s axis [0 10 0 30] NOT applied) -- tau = %g s'], cfg.tau));
end
hold off;

figure('Name', 'Zarchan Listing 4.5 -- the mean the listing also plots');
plot(R.tf, R.xmean, 'b+-', 'LineWidth', 1.2); grid on;
xlabel('Time'); ylabel('Mean miss (ft)');
title('Sample mean: TRUE value is exactly zero (COEF is +/-1) -- this is noise');
end


% =============================================================================
function s = tf2s(b)
if b, s = 'PASS'; else, s = 'FAIL'; end
end
