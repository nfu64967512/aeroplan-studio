function R = lawEffectivenessMC(opts)
%MGS.ZARCHAN.LAWEFFECTIVENESSMC  Zarchan Listing 31.1 -- linearised Monte Carlo
%   comparing guidance-law effectiveness with measurement noise and a
%   three-state Kalman filter in the loop.
%
%   R = mgs.zarchan.lawEffectivenessMC()
%   R = mgs.zarchan.lawEffectivenessMC('runs', 100, 'tf', 1:10)
%   R = mgs.zarchan.lawEffectivenessMC('benchmark',true,'assert',false,'render',false)
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: ft, ft/s, ft/s^2, s, rad; g = 32.2 ft/s^2;
%   Vc = 5*3280 ft/s (the book converts km at 3280 ft/km, not 3280.84).
%   THE MONTE CARLO IS SEEDED -- an internal MINSTD stream, default seed
%   20260824, recorded in R.p.seed and R.units.  It never touches rand/randn,
%   so a caller's rng() cannot move these numbers and this file cannot move the
%   caller's.  An unseeded Monte Carlo cannot be benchmarked at all.
% ===========================================================================
%
%   WHAT IT IS.  Fig. 31.6's homing loop linearised about the collision course:
%   y'' = nT - nL, a first-order flight-control lag nL/nc = 1/(1+sT), an angle
%   measurement y/(Vc*tgo) corrupted by 0.1 mr of white noise, and the Chapter 9
%   three-state Kalman filter estimating (y, y', nT) from it.  Four guidance
%   laws share that loop (the listing's APN): 0 = proportional navigation,
%   1 = optimal guidance, 2 = differential game with bounded controls
%   (bang-bang), 3 = hybrid (optimal until tgo = TSW, then differential game).
%   Four random target maneuvers share it too (ICONSTANT): 1 = Poisson,
%   2 = uniform constant, 3 = random weave, 4 = random vertical-S.
%
%   *** WHAT IT CANNOT SAY. ***  Linearised and planar: constant closing
%   velocity, no gravity, drag, thrust, seeker dynamics or radome, and the
%   missile knows its own achieved nL perfectly.  The only nonlinearity is the
%   acceleration limit.  The bang-bang law's headline result assumes an actuator
%   that follows a command reversing every few hundredths of a second -- exactly
%   the assumption our own gamma channel is measured NOT to satisfy (tracking
%   gain 0.063, ~8 s of lag, saturating on 26 of 32 sorties).  Read it as a
%   clean-loop RANKING of guidance laws under estimation error, never as an
%   achievable miss.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  matlab/zarchan/_listings/listing_31_01.m does not parse.
%   Its header reports SIX unresolved variable-to-variable comparisons.  All six
%   are resolved here and each direction is FORCED, not guessed:
%
%     1  "while T ,= (TF - 1e-5)" -> "T <= ...".  "," is this book's OCR of "<";
%        T only increases from 0, so ">=" never terminates.
%     2  "if T.= (DELT+TNOW)" -> "T >= ...".  "." is the OCR of ">".  DELT is the
%        drawn Poisson dwell, TNOW the epoch it was drawn at, so the sign flips
%        once the dwell has ELAPSED.  "<=" would flip on every step from t = 0.
%     3-5  "if T , TSTART" (ICONSTANT 2, 3 and 4) -> "T < TSTART, XNTC = 0, else
%        <maneuver>", i.e. the maneuver STARTS at TSTART.  Forced by the
%        chapter's own definition, zarchan.txt:11614: the starting time is
%        "equally likely to occur anywhere during the flight".  The other
%        reading is a maneuver that STOPS at TSTART -- a different signal, for
%        which the filter's assumed process noise would be wrong in its own
%        nominal case.
%     6  "if TGO.TSW" -> "TGO > TSW".  Forced by PROSE, not shape: zarchan.txt
%        :12009 puts optimal guidance "for most of the flight" and differential
%        game in the last TSW seconds.  The other direction inverts BENCHMARK 7
%        from 80% to 20%, which is why that row is on the board.
%
%   Four further repairs, none a judgement call: "while STEP ,=1" -> "<= 1" and
%   "if S . =(TS - 1e-5)" -> ">=" (same two OCR substitutions, both against
%   constants; S resets to 0 each record, so "<=" would record for ever);
%   Table 31.1 and two running page headers leaked INTO the listing and are
%   dropped as prose (then used, from their real home at zarchan.txt:11796-11803,
%   as BENCHMARKS 1-4); and the comment "% TYPE OF MANEUVER (1=POISSON,...,
%   4=RANDOM VS" swallowed the assignment "ICONSTANT=2;" when its closing paren
%   and newline were lost -- restored as comment plus assignment.
%
%   FIVE OF THE BOOK'S QUIRKS ARE REPRODUCED DELIBERATELY:
%     *  PHIN = XNTIC^2/10.  The text (:11684-11688) defines the process-noise
%        density as nTMAX^2/tF, but the listing pins the DENOMINATOR at 10 s --
%        the longest flight in the sweep -- so every shorter flight runs a filter
%        tuned for a 10-s flight.  Exposed as 'PHIN_TF' so it can be priced.
%     *  Two epsilons on tgo: +1e-5 in the derivative passes, +1e-6 in the
%        sampling block.  Both kept; the guidance divides by tgo^2, so this is
%        checkable rather than assumed.
%     *  The measurement uses the XLAM left over from the PREDICTOR pass, not
%        the corrected y.  An O(H^2) inconsistency in the book; reproduced.
%     *  RMS = sqrt(sum(Z.^2)/(RUN-1)) -- the mean SQUARE over RUN-1, not RUN, so
%        the plotted "RMS MISS" is inflated by sqrt(RUN/(RUN-1)) = 1.01 at
%        RUN = 50.  Reproduced (it drew Figs. 31.11-31.26); the conventional
%        estimator is also returned, as R.mc.rms_n.
%     *  XNEW in the hybrid branch, and RTM/XLAMD in the derivative block, are
%        computed and never read.  Dropped, and recorded here.
%
%   TWO DELIBERATE DEPARTURES, both to sharpen the comparison:
%     *  COMMON RANDOM NUMBERS.  The book draws sequentially, so its optimal and
%        differential-game sets see different noise.  Here every law at a given
%        flight time gets the SAME noise matrix and the SAME maneuvers, making
%        BENCHMARK 10 a PAIRED comparison.  A structural row checks it held.
%     *  Replications are an array dimension, not a loop -- exact, since they are
%        independent and share the time grid.  The Kalman covariance and gains
%        are data-independent (SIGN2 depends only on tgo), so they stay scalar.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Every book number was grepped out of zarchan.txt and its line
%   is in the row's .source.  ⚠️ CHAPTER 31 PUBLISHES NO RMS MISS NUMBER IN
%   PROSE: Figs. 31.11-31.26 carry all of them and the extraction turned every
%   figure into an image link.  So the value rows are the published PARAMETERS,
%   the book's claims ABOUT those figures are scored as ORDERING rows, and one
%   SKIP row says this out loud instead of inventing a curve to match.
%
%     1-4  Table 31.1 (:11796-11803) against this run's constants, in the
%          table's own units: nT = 5 g, aMAX = 10 g, Vc = 5 km/s, sNoise =
%          0.0001 r (= the prose's 0.1 mr, :11683).  XNTIC, XLIM, VC and
%          SIGNOISE are the four numbers a silent OCR slip would move.  A caller
%          who deliberately runs off-book SHOULD see these red.
%     5    PHIN = nTMAX^2/tF = 161^2/10 (:11684-11688).
%     6    Poisson zero-crossing rate = 0.5 /s (:11620), measured from the dwell
%          generator alone: SIG = 1/sqrt(2*XNU) makes SIG^2*(z1^2+z2^2)
%          exponential with mean 1/XNU.  Band = 4 standard errors of the measured
%          mean, floored at 1% (~4% at the default 2000 replications).  This is
%          the row that catches SIG built with the wrong factor of 2 -- which
%          changes the maneuver's character while leaving every plot looking
%          exactly like a Poisson square wave.
%     7    "Chattering has been removed from 80% of the flight" (Fig. 31.19,
%          :12010): a single flight, TSW = 2 s, tF = 10 s.  Scored as the
%          fraction of guidance updates that took the OPTIMAL branch, which is
%          (tF - TSW)/tF exactly.  Band 1%.
%     8    SHAPE, Fig. 31.1 prose (:11578): the effective navigation ratio is
%          "approaching 3" at long tgo and "grows considerably" at short tgo.
%          Scored as strictly decreasing in tgo/T over [0.5, 200], within 0.1 of
%          3 at the top, at least twice that at 0.5.
%          ⚠️ The grid starts at 0.5 ON PURPOSE: the listing guards its
%          denominator with "+.0001", and below tgo/T ~ 0.25 that epsilon
%          dominates a denominator falling like 0.3*x^5, so the LISTING's ratio
%          turns over and collapses while the BOOK's formula diverges.  Scoring
%          monotonicity down to 0 would score the epsilon, not the book.
%     9    SHAPE (:11580): differential game guidance is "bang-bang in nature
%          (missile executes either maximum positive or negative acceleration)".
%          Scored on the command history: nothing strictly inside the limit, and
%          at least 99% of commands exactly at it.
%    10    ORDERING (:11865, Fig. 31.13 at :11922): against the 5-g Poisson
%          maneuver at a 10-g limit, differential game "performed much better
%          than optimal guidance".  "Much better" is an adjective, so only
%          rms(DG) < rms(OG) at every flight time is scored.  ⚠️ THE ONLY
%          STOCHASTIC SCORED ROW, and the only one costing a simulation; a red
%          here is a guidance-wiring fault, not a tolerance question.
%
%   NOT SCORED: every rms miss VALUE (SKIP row -- figure-only, as above); the
%   book's sample size, 50 runs at each of 100 flight times over 0.1:0.1:10 s
%   (:11749), reported as INFO against what this run did, because nothing scored
%   depends on the sweep being 100 points and the board reruns on every commit;
%   and Fig. 31.12's "similar performance for flight times greater than 4 s"
%   (:11863), which cannot be scored without inventing a band for "similar".
%
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED -- this file has never
%   been executed; MATLAB is the caller's. ***  Rows 1-5 are exact arithmetic and
%   should read 0.00%.  Row 6 should land within ~2% of 0.5 /s.  Row 7 should be
%   800 optimal updates out of 1000, i.e. 80.0%.  Rows 8-10 are pass/fail.  Cost
%   is 2 laws x 3 flight times at H = 0.001 s, about 4.2e4 vector steps of width
%   32, plus one 1e4-step single flight: it is the STEP count that costs, not
%   the replication count, so raise 'runs' before raising anything else.
%
% ---------------------------------------------------------------------------
%   OUTPUTS
%     R.mc      .tf .laws .rms .rms_n .sigma .mean .miss{} -- the sweep
%     R.hybrid  .frac_og .n_updates .TSW .tf   -- BENCHMARK 7's single flight
%     R.dg      bang-bang command census        -- BENCHMARK 9
%     R.poisson .rate .se .mean_count .runs     -- BENCHMARK 6
%     R.effnav  .x .Np, the listing's XNPP on a grid -- BENCHMARK 8
%     R.bench   benchmark rows, schema shared with the other ports
%     R.selfcheck .names .oks .details .all_ok .first_bad
%     R.pass    true only if a row was SCORED and every scored row passed
%     R.verdict '' when everything passed, else the one-line reason.  ALWAYS
%               formed; 'assert' chooses only whether to error or warn, and with
%               it false R comes back COMPLETE so a failure can be diagnosed.
%     R.p       the parameters actually used;  R.units;  R.summary
%
%   See also MGS.ZARCHAN.SHAPINGFILTERADJOINT, MGS.ZARCHAN.MONTECARLOHOMING,
%   MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 31.1 line 1, verbatim.  Feet, seconds, radians.
    opts.VC        (1,1) double {mustBeFinite, mustBePositive} = 5*3280
    opts.XNTIC     (1,1) double {mustBeFinite, mustBePositive} = 161
    opts.XLIM      (1,1) double {mustBeFinite, mustBePositive} = 322
    opts.XNP       (1,1) double {mustBeFinite, mustBePositive} = 3
    opts.TAU       (1,1) double {mustBeFinite, mustBePositive} = 0.2
    opts.TS        (1,1) double {mustBeFinite, mustBePositive} = 0.01
    opts.SIGNOISE  (1,1) double {mustBeFinite, mustBeNonnegative} = 1e-4
    opts.VM        (1,1) double {mustBeFinite, mustBePositive} = 3000
    opts.HEDEG     (1,1) double {mustBeFinite, mustBeReal}     = 20
    opts.XNU       (1,1) double {mustBeFinite, mustBePositive} = 0.5
    opts.W         (1,1) double {mustBeFinite, mustBePositive} = 2
    % ⚠️ The LISTING's line 1 has TSW = 1.  This defaults to 2 because the only
    % published hybrid number in the chapter is Fig. 31.19's, which is TSW = 2,
    % and BENCHMARK 7 SKIPs rather than fails at any other value.  Set TSW = 1
    % to run the listing exactly; the row will then say why it did not score.
    opts.TSW       (1,1) double {mustBeFinite, mustBePositive} = 2
    % The listing writes XNTIC^2/10 -- the 10 is the LONGEST flight time in the
    % sweep, not the current one.  Exposed, defaulted to the book's value.
    opts.PHIN_TF   (1,1) double {mustBeFinite, mustBePositive} = 10
    opts.H         (1,1) double {mustBeFinite, mustBePositive} = 0.001
    % 1 = Poisson, 2 = uniform constant, 3 = random weave, 4 = random vertical-S.
    % 1 is the default because BENCHMARK 10 is Fig. 31.13's claim.
    opts.maneuver  (1,1) double {mustBeMember(opts.maneuver,[1 2 3 4])} = 1
    % The book sweeps 0.1:0.1:10 with 50 runs.  Three flight times and 24 runs
    % is enough for an ORDERING and nothing scored needs more; see the INFO row.
    opts.tf        double {mustBeReal, mustBeFinite, mustBePositive} = [4 7 10]
    opts.runs      (1,1) double {mustBeFinite, mustBePositive} = 32
    opts.rate_runs (1,1) double {mustBeFinite, mustBePositive} = 2000
    opts.seed      (1,1) double {mustBeFinite, mustBeNonnegative} = 20260824
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so "if nargin == 0" here would be a mode that never fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.assert    (1,1) logical = true
end

P = struct('VC',opts.VC, 'XNTIC',opts.XNTIC, 'XLIM',opts.XLIM, 'XNP',opts.XNP, ...
    'TAU',opts.TAU, 'TS',opts.TS, 'SIGNOISE',opts.SIGNOISE, 'VM',opts.VM, ...
    'HEDEG',opts.HEDEG, 'XNU',opts.XNU, 'W',opts.W, 'TSW',opts.TSW, ...
    'PHIN_TF',opts.PHIN_TF, 'H',opts.H, 'runs',round(opts.runs));

if P.H >= P.TAU
    error('mgs:zarchan:lawEffectivenessMC:stiff', ...
        ['H = %g s is not smaller than the flight-control time constant ' ...
         'TAU = %g s.  The lag state would be integrated at or below one ' ...
         'sample per time constant; the loop does not blow up, it returns a ' ...
         'smooth and entirely wrong miss, so this is refused not warned.'], ...
        P.H, P.TAU);
end
if P.H > P.TS
    error('mgs:zarchan:lawEffectivenessMC:sampling', ...
        ['H = %g s exceeds the Kalman sampling time TS = %g s, so the filter ' ...
         'would update less often than the listing''s accumulator intends.'], ...
        P.H, P.TS);
end
if P.runs < 4
    error('mgs:zarchan:lawEffectivenessMC:runs', ...
        ['runs = %d.  SIGMA and RMS divide by RUN-1, and a sample this small ' ...
         'has no usable dispersion, so an ordering row would pass on noise.'], ...
        P.runs);
end
tfList = opts.tf(:).';

% =========================================================================
%   THE SWEEP.  Optimal guidance (APN = 1) and differential game (APN = 2) on
%   COMMON RANDOM NUMBERS at every flight time.  This is BENCHMARK 10.
% =========================================================================
laws   = [1 2];
lawNam = {'optimal (APN=1)', 'diff-game (APN=2)'};
nT = numel(tfList);  nL = numel(laws);

MC = struct();
MC.tf    = tfList;
MC.laws  = lawNam;
MC.rms   = zeros(nL, nT);
MC.rms_n = zeros(nL, nT);
MC.sigma = zeros(nL, nT);
MC.mean  = zeros(nL, nT);
MC.miss  = cell(nL, nT);
MC.crn   = zeros(nL, nT);
MC.steps = 0;

dgCensus = struct('nCmd',0, 'nBang',0, 'nZero',0, 'nInterior',0, 'nUpd',0);
K1lo = Inf;  K1hi = -Inf;  Plo = Inf;  tgoLo = Inf;  cmdHi = 0;  ovf = 0;

for k = 1:nT
    rnd = drawBlock(opts.seed + 1000*k, P.runs, dwellPool(tfList(k), P.XNU), ...
                    sampleCount(tfList(k), P.H, P.TS));
    for j = 1:nL
        O = mcBatch(P, tfList(k), laws(j), opts.maneuver, rnd);
        Z = O.Z;
        XMEAN = sum(Z)/P.runs;
        MC.mean(j,k)  = XMEAN;
        MC.sigma(j,k) = sqrt(sum((Z-XMEAN).^2)/(P.runs-1));
        MC.rms(j,k)   = sqrt(sum(Z.^2)/(P.runs-1));   % the BOOK'S divisor
        MC.rms_n(j,k) = sqrt(sum(Z.^2)/P.runs);
        MC.miss{j,k}  = Z;
        MC.crn(j,k)   = O.crn;
        MC.steps      = MC.steps + O.nSteps;
        if laws(j) == 2
            dgCensus.nCmd      = dgCensus.nCmd      + O.nCmd;
            dgCensus.nBang     = dgCensus.nBang     + O.nBang;
            dgCensus.nZero     = dgCensus.nZero     + O.nZero;
            dgCensus.nInterior = dgCensus.nInterior + O.nInterior;
            dgCensus.nUpd      = dgCensus.nUpd      + O.nUpdates;
        end
        K1lo = min(K1lo, O.K1min);  K1hi = max(K1hi, O.K1max);
        Plo  = min(Plo,  O.Pmin);   tgoLo = min(tgoLo, O.minTGO);
        cmdHi = max(cmdHi, O.maxAbsCmd);  ovf = ovf + O.dwellOverflow;
    end
end

% =========================================================================
%   BENCHMARK 7's single flight: hybrid guidance (APN = 3), TSW = 2 s,
%   tF = 10 s, vertical-S maneuver -- Fig. 31.19's own case.
% =========================================================================
tfH = 10;
rndH = drawBlock(opts.seed + 90000, 1, dwellPool(tfH, P.XNU), ...
                 sampleCount(tfH, P.H, P.TS));
OH = mcBatch(P, tfH, 3, 4, rndH);
HY = struct('frac_og', OH.nOG/max(OH.nUpdates,1), 'n_og', OH.nOG, ...
            'n_updates', OH.nUpdates, 'TSW', P.TSW, 'tf', tfH, 'miss', OH.Z);
MC.steps = MC.steps + OH.nSteps;
tgoLo = min(tgoLo, OH.minTGO);  cmdHi = max(cmdHi, OH.maxAbsCmd);
ovf = ovf + OH.dwellOverflow;

% =========================================================================
%   BENCHMARK 6: the Poisson dwell generator on its own, no dynamics.
% =========================================================================
SIG = 1/sqrt(2*P.XNU);
% The dwell pool is sized FROM XNU, not fixed at the book's rate.  A hard 30
% would be exhausted for XNU > ~2 and would then light the "pool never
% exhausted" structural row -- a red caused by this file's own sizing, on a run
% that is otherwise perfectly legitimate.
PO = poissonRate(SIG, 10, round(opts.rate_runs), dwellPool(10, P.XNU), opts.seed + 1);

% =========================================================================
%   BENCHMARK 8: the listing's effective navigation ratio on a grid.
% =========================================================================
EN = struct();
EN.x  = logspace(log10(0.5), log10(200), 240);
EN.Np = effNav(EN.x);

R = struct();
R.mc      = MC;
R.hybrid  = HY;
R.dg      = dgCensus;
R.poisson = PO;
R.effnav  = EN;
R.units   = sprintf(['ft, ft/s, ft/s^2, s, rad; g = 32.2 ft/s^2; ' ...
                     'Vc = %g ft/s (book: 3280 ft/km); MINSTD seed %d'], ...
                     P.VC, opts.seed);
R.p = P;
R.p.tf = tfList;  R.p.maneuver = opts.maneuver;  R.p.seed = opts.seed;
R.p.rate_runs = round(opts.rate_runs);  R.p.units = R.units;
R.p.n_vector_steps = MC.steps;

R.selfcheck = structuralChecks(P, MC, K1lo, K1hi, Plo, tgoLo, cmdHi, ovf, PO);

if opts.benchmark
    R.bench = benchmarks(R, P, opts.maneuver, PO, HY, EN, dgCensus, tfList);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  "Not checked" must never render as "fine".
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end

msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the book. ' ...
           'R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = 'not one book value could be evaluated; an unvalidated port must not be handed back silently.';
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
        error('mgs:zarchan:lawEffectivenessMC:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN, complete.  benchSuite runs every port with assert
        % off precisely so one red row cannot hide the rest of the board.
        warning('mgs:zarchan:lawEffectivenessMC:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function O = mcBatch(P, TF, APN, ICONSTANT, rnd)
%MCBATCH  Listing 31.1's two inner loops, zarchan.txt:11793-11861, run for every
%   replication at once.  EVERY constant arrives as an argument: MATLAB local
%   functions cannot see the caller's workspace, and a previous wave in this
%   package lost a day to exactly that.
n   = numel(rnd.u_start);
H   = P.H;   TAU = P.TAU;  TS = P.TS;   VC = P.VC;
XLIM = P.XLIM;  XNTIC = P.XNTIC;  XNP = P.XNP;  SIGNOISE = P.SIGNOISE;
TSW = P.TSW;  W = P.W;

% Listing: TSTART=TF*SUM;  PZ=rand-.5;  COEF = +1 if PZ>0 else -1;
%          SUM=rand;  PHASE=6.28*SUM.
TSTART = TF*rnd.u_start;
COEF   = ones(1,n);   COEF(rnd.u_sign - 0.5 <= 0) = -1;
PHASE  = 6.28*rnd.u_phase;

Y = zeros(1,n);  YD = zeros(1,n);  XNL = zeros(1,n);      % YIC = 0
YH = zeros(1,n); YDH = zeros(1,n); XNTH = zeros(1,n);
% XLAM is preallocated even though pass 1 always writes it before the sampling
% block reads it: if the while loop takes zero steps (TF <= 1e-5) the sampling
% block never runs either, and an undefined XLAM would then turn a degenerate
% input into an "Unrecognized variable" thrown from the middle of the filter.
XNC = zeros(1,n);  XLAM = zeros(1,n);   %#ok<PREALL>

% ---- Kalman covariance.  SCALAR: SIGN2 depends on tgo only, never on data,
%      so the Riccati recursion is identical for every replication.
TS2 = TS*TS;  TS3 = TS2*TS;  TS4 = TS3*TS;  TS5 = TS4*TS;
PHIN = XNTIC*XNTIC/P.PHIN_TF;                  % book: nTMAX^2/tF, tF pinned at 10
RTM = VC*TF;  SIGPOS = RTM*SIGNOISE;  SIGN2 = SIGPOS^2;
P11 = SIGN2;  P12 = 0;  P13 = 0;
P22 = (P.VM*P.HEDEG/57.3)^2;  P23 = 0;  P33 = XNTIC*XNTIC;

% ---- Poisson maneuver state.  SIG = 1/sqrt(2*XNU) makes the dwell
%      SIG^2*(z1^2+z2^2) exponential with mean 2*SIG^2 = 1/XNU seconds, i.e.
%      XNU zero crossings per second.  BENCHMARK 6 measures that.
SIG  = 1/sqrt(2*P.XNU);
BETA = XNTIC;
XNTP = BETA*ones(1,n);   XNTP(rnd.z_init <= 0) = -BETA;
DELT = 9999*ones(1,n);   TNOW = zeros(1,n);
QFIRST = true(1,n);      kInt = zeros(1,n);
nInt = size(rnd.zd1, 1);
dwellOverflow = 0;

T = 0;  S = 0;  iSamp = 0;  nSteps = 0;
nUpdates = 0;  nOG = 0;  nBang = 0;  nZero = 0;  nInterior = 0;
K1min = Inf;  K1max = -Inf;  Pmin = Inf;  minTGO = Inf;  maxAbsCmd = 0;

while T <= (TF - 1e-5)
    % ---- Poisson square wave: draw a dwell when due, flip when it elapses.
    if any(QFIRST)
        cols = find(QFIRST);
        kInt(cols) = kInt(cols) + 1;
        kk = kInt(cols);
        over = kk > nInt;
        if any(over)
            % Pool exhausted: stop switching rather than index out of bounds.
            % Counted, and reported by a structural row -- silently reusing the
            % last dwell would bias the maneuver toward long constant segments.
            dwellOverflow = dwellOverflow + sum(over);
            DELT(cols(over)) = Inf;
        end
        good = ~over;
        if any(good)
            lin = sub2ind([nInt n], kk(good), cols(good));
            DELT(cols(good)) = SIG^2*(rnd.zd1(lin).^2 + rnd.zd2(lin).^2);
        end
        QFIRST(cols) = false;
        TNOW(cols)   = T;
    end
    % "doFlip", not "flip": flip is a MATLAB builtin and shadowing it inside a
    % 10,000-iteration loop is the kind of thing that works until it doesn't.
    doFlip = T >= (DELT + TNOW);
    XNTP(doFlip)   = -XNTP(doFlip);
    QFIRST(doFlip) = true;

    YOLD = Y;  YDOLD = YD;  XNLOLD = XNL;

    % ---- pass 1 (the listing's FLAG == 0 leg): derivatives at (T, state).
    %      RTM and XLAMD are computed by the listing here and never read; the
    %      guidance runs entirely off the filter.  Dropped.
    XNTC = maneuverAccel(ICONSTANT, T, XNTP, COEF, TSTART, PHASE, XNTIC, W);
    TGOd = TF - T + 0.00001;
    XLAM = Y./(VC*TGOd);   %#ok<NASGU> -- dead, and KEPT: see pass 2
    XNLD = (XNC - XNL)/TAU;
    YDD  = XNTC - XNL;

    % ---- predictor.  ORDER MATTERS: Y uses the OLD YD, YD the OLD YDD.
    Y = Y + H*YD;  YD = YD + H*YDD;  XNL = XNL + H*XNLD;  T = T + H;

    % ---- pass 2 (FLAG == 1): same derivatives at the ADVANCED time and the
    %      predicted state.  XNTC is re-evaluated at T+H, as in the listing.
    XNTC = maneuverAccel(ICONSTANT, T, XNTP, COEF, TSTART, PHASE, XNTIC, W);
    TGOd = TF - T + 0.00001;
    % THIS XLAM is the one the measurement uses -- the listing computes XLAM in
    % BOTH derivative passes and the sampling block reads whatever the loop left
    % behind, i.e. the PREDICTED y at T+H, not the corrected y.  The pass-1
    % assignment above is therefore dead, and is kept so that nobody "tidies" it
    % away, decides the surviving one looks misplaced, and moves it before the
    % corrector -- which would be a different (and more defensible) simulation
    % than the one that drew Figs. 31.11-31.26.
    XLAM = Y./(VC*TGOd);
    XNLD = (XNC - XNL)/TAU;
    YDD  = XNTC - XNL;

    % ---- corrector, in the listing's own algebra.
    Y   = 0.5*(YOLD   + Y   + H*YD);
    YD  = 0.5*(YDOLD  + YD  + H*YDD);
    XNL = 0.5*(XNLOLD + XNL + H*XNLD);
    nSteps = nSteps + 1;

    S = S + H;
    if S >= (TS - 1e-5)
        S = 0;
        TGO = TF - T + 0.000001;      % NOTE: 1e-6 here, 1e-5 above.  Book's.
        RTM = VC*TGO;  SIGPOS = RTM*SIGNOISE;  SIGN2 = SIGPOS^2;

        % ---- covariance propagation and gains (Chapter 9, three states)
        M11 = P11 + TS*P12 + .5*TS2*P13 + TS*(P12 + TS*P22 + .5*TS2*P23);
        M11 = M11 + .5*TS2*(P13 + TS*P23 + .5*TS2*P33) + TS5*PHIN/20.;
        M12 = P12 + TS*P22 + .5*TS2*P23 + TS*(P13 + TS*P23 + .5*TS2*P33) ...
              + TS4*PHIN/8.;
        M13 = P13 + TS*P23 + .5*TS2*P33 + PHIN*TS3/6.;
        M22 = P22 + TS*P23 + TS*(P23 + TS*P33) + PHIN*TS3/3.;
        M23 = P23 + TS*P33 + .5*TS2*PHIN;
        M33 = P33 + PHIN*TS;
        K1 = M11/(M11+SIGN2);  K2 = M12/(M11+SIGN2);  K3 = M13/(M11+SIGN2);
        P11 = (1.-K1)*M11;  P12 = (1.-K1)*M12;  P13 = (1.-K1)*M13;
        P22 = -K2*M12 + M22;  P23 = -K2*M13 + M23;  P33 = -K3*M13 + M33;

        % ---- measurement and filter update (vector across replications)
        iSamp = iSamp + 1;
        if iSamp > size(rnd.zmeas,1)
            error('mgs:zarchan:lawEffectivenessMC:noise', ...
                ['the measurement-noise pool (%d samples) ran out at t = %g s. ' ...
                 'This is a sizing fault in sampleCount(), not a tolerance ' ...
                 'question: recycling noise would silently correlate the runs.'], ...
                size(rnd.zmeas,1), T);
        end
        XLAMNOISE = SIGNOISE*rnd.zmeas(iSamp,:);
        YSTAR = RTM*(XLAM + XLAMNOISE);
        RES  = YSTAR - YH - TS*YDH - .5*TS*TS*(XNTH - XNC);
        YH   = K1*RES + YH + TS*YDH + .5*TS*TS*(XNTH - XNC);
        YDH  = K2*RES + YDH + TS*(XNTH - XNC);
        XNTH = K3*RES + XNTH;

        % ---- guidance.  ZEM1H is the differential-game zero-effort miss (no
        %      target-maneuver term at all); ZEM2H adds the optimal law's
        %      0.5*nT*tgo^2.  That single term is the whole difference between
        %      the two laws' knowledge of the target.
        X = TGO/TAU;
        lagT  = XNL*TAU*TAU*(exp(-X) + X - 1.);
        ZEM1H = YH + YDH*TGO - lagT;
        ZEM2H = ZEM1H + .5*XNTH*TGO*TGO;
        usedOG = true;
        switch APN
            case 0                                   % proportional navigation
                XNC = XNP*(YH + YDH*TGO)/TGO^2;
            case 1                                   % optimal guidance
                XNC = effNav(X)*ZEM2H/TGO^2;
            case 2                                   % differential game
                XNC = XLIM*sign(ZEM1H);
                usedOG = false;
            otherwise                                % hybrid
                if TGO > TSW
                    XNC = effNav(X)*ZEM2H/TGO^2;
                    XNC = min(max(XNC, -XLIM), XLIM); % listing's inner clamp
                else
                    XNC = XLIM*sign(ZEM1H);
                    usedOG = false;
                end
        end
        XNC = min(max(XNC, -XLIM), XLIM);

        nUpdates = nUpdates + 1;
        nOG      = nOG + double(usedOG);
        a = abs(XNC);
        nBang     = nBang     + sum(abs(a - XLIM) <= 1e-9*XLIM);
        nZero     = nZero     + sum(a == 0);
        nInterior = nInterior + sum(a > 0 & (XLIM - a) > 1e-9*XLIM);
        K1min = min(K1min, K1);  K1max = max(K1max, K1);
        Pmin  = min([Pmin P11 P22 P33]);
        minTGO = min(minTGO, TGO);  maxAbsCmd = max(maxAbsCmd, max(a));
    end
end

O = struct();
O.Z = Y;                                  % Listing: Z(I) = Y at t = TF
O.nSteps = nSteps;  O.nUpdates = nUpdates;  O.nOG = nOG;
O.nCmd = nUpdates*n;  O.nBang = nBang;  O.nZero = nZero;  O.nInterior = nInterior;
O.K1min = K1min;  O.K1max = K1max;  O.Pmin = Pmin;
O.minTGO = minTGO;  O.maxAbsCmd = maxAbsCmd;
O.dwellOverflow = dwellOverflow;
% Fingerprint of the random input this batch consumed.  Two laws at the same
% flight time MUST produce the same number, or the comparison is not paired.
O.crn = sum(TSTART) + sum(COEF) + sum(rnd.zmeas(:)) + sum(rnd.z_init);
end


% =============================================================================
function XNTC = maneuverAccel(ICONSTANT, T, XNTP, COEF, TSTART, PHASE, XNTIC, W)
%MANEUVERACCEL  The listing's four ICONSTANT branches, zarchan.txt:11818-11828.
%   "if T , TSTART" is resolved as T < TSTART -> no maneuver yet; see header.
switch ICONSTANT
    case 1                                            % Poisson square wave
        XNTC = XNTP;
    case 2                                            % uniform constant
        XNTC = COEF.*XNTIC.*double(T >= TSTART);
    case 3                                            % random weave (no COEF)
        XNTC = XNTIC.*sin(W*T + PHASE).*double(T >= TSTART);
    otherwise                                         % random vertical-S
        XNTC = COEF.*XNTIC.*sign(sin(W*(T - TSTART))).*double(T >= TSTART);
end
end


% =============================================================================
function Np = effNav(X)
%EFFNAV  The listing's XNPP, zarchan.txt:11842 and :11845, and the closed form at
%   :11573 -- N' = 6x^2(e^-x - 1 + x)/(2x^3 + 3 + 6x - 6x^2 - 12x e^-x - 3 e^-2x).
%
%   ⚠️ THE "+.0001" IS THE LISTING'S AND IS KEPT.  The denominator's leading
%   term is 0.3*x^5, so the guard dominates below x ~ 0.25 and the ratio turns
%   OVER and collapses to zero there instead of diverging.  Removing it divides
%   by zero at the last sample; keeping it means BENCHMARK 8's monotonicity is
%   only scored above x = 0.5.
TOP  = 6.*X.*X.*(exp(-X) - 1. + X);
BOT1 = 2*X.*X.*X + 3. + 6.*X - 6.*X.*X;
BOT2 = -12.*X.*exp(-X) - 3.*exp(-2.*X);
Np   = TOP./(.0001 + BOT1 + BOT2);
end


% =============================================================================
function PO = poissonRate(SIG, TFr, nRep, nInt, seed)
%POISSONRATE  BENCHMARK 6: the dwell generator alone, no dynamics.  The listing
%   draws DELT = XNOISE1^2 + XNOISE2^2 with XNOISE = SIG*randn, so DELT is
%   exponential with mean 2*SIG^2 = 1/XNU and the switch count over tF is
%   Poisson(XNU*tF).  Measured here, scored against the book's 0.5 /s.
u  = lcgUniform(seed, 2*nInt*nRep);
z  = boxMuller(u);
z1 = reshape(z(1:nInt*nRep), nInt, nRep);
z2 = reshape(z(nInt*nRep+1:end), nInt, nRep);
D  = SIG^2*(z1.^2 + z2.^2);
C  = cumsum(D, 1);
k  = sum(C <= TFr, 1);
PO = struct();
PO.mean_count = mean(k);
PO.rate       = PO.mean_count/TFr;
PO.se         = (std(k)/sqrt(nRep))/TFr;
PO.runs       = nRep;
PO.exhausted  = sum(k >= nInt);
PO.mean_dwell = mean(D(:));
end


% =============================================================================
function rnd = drawBlock(seed, n, nInt, nSamp)
%DRAWBLOCK  Every random number one (maneuver, flight-time) cell needs, drawn
%   ONCE and reused by every guidance law -- that is what makes the comparison
%   paired.  Blocks are contiguous and in a fixed order so that changing nSamp
%   cannot shift the maneuver draws.
nU = 3*n;
nZ = n + 2*nInt*n + nSamp*n;
u  = lcgUniform(seed, nU + 2*ceil(nZ/2));
rnd = struct();
rnd.u_start = u(1:n);
rnd.u_sign  = u(n + (1:n));
rnd.u_phase = u(2*n + (1:n));
z = boxMuller(u(nU+1:end));
p = 0;
rnd.z_init = z(p + (1:n));                          p = p + n;
rnd.zd1    = reshape(z(p + (1:nInt*n)), nInt, n);   p = p + nInt*n;
rnd.zd2    = reshape(z(p + (1:nInt*n)), nInt, n);   p = p + nInt*n;
rnd.zmeas  = reshape(z(p + (1:nSamp*n)), nSamp, n);
end


function nInt = dwellPool(TF, XNU)
%DWELLPOOL  How many Poisson dwells one flight can need.  Mean count is XNU*TF;
%   this is that plus 8 sigma plus 20, and exhaustion is reported, never hidden.
m = XNU*TF;
nInt = ceil(m + 8*sqrt(max(m,1)) + 20);
end


function nSamp = sampleCount(TF, H, TS)
%SAMPLECOUNT  How many times "S >= TS - 1e-5" can fire.  The accumulator adds H
%   once per step and fires every ceil((TS-1e-5)/H) steps.  Sized from that, not
%   from TF/TS, so a caller who sets H > TS still gets a big enough pool.
stepsPer = max(1, ceil((TS - 1e-5)/H));
nSteps   = ceil(TF/H) + 2;
nSamp    = floor(nSteps/stepsPer) + 4;
end


% =============================================================================
function u = lcgUniform(seed, n)
%LCGUNIFORM  MINSTD (Lehmer) uniforms in (0,1).  Base MATLAB, deterministic, and
%   it never touches the global random stream -- a benchmark that moves when
%   somebody else calls rng() is not a benchmark, and setting the global stream
%   to prevent that would reach into the caller's session.
%   x <- 16807*x mod (2^31 - 1); 16807*x < 3.61e13 < 2^53, so every product is
%   exact in double precision.  x = 0 is absorbing, hence the seed mapping.
m = 2147483647;
x = mod(floor(abs(seed)), m - 1) + 1;
u = zeros(1, n);
for k = 1:n
    x = mod(16807*x, m);
    u(k) = x/m;
end
end


function z = boxMuller(u)
%BOXMULLER  Standard normals from an EVEN number of uniforms in (0,1).
%   MINSTD never returns 0 (x is in 1..m-1), so log(u1) is always finite.
u1 = u(1:2:end);  u2 = u(2:2:end);
r  = sqrt(-2*log(u1));
z  = zeros(1, 2*numel(u1));
z(1:2:end) = r.*cos(2*pi*u2);
z(2:2:end) = r.*sin(2*pi*u2);
end


% =============================================================================
function K = structuralChecks(P, MC, K1lo, K1hi, Plo, tgoLo, cmdHi, ovf, PO)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan -- they are the weaker
%   claim that it agrees with itself, which is what catches wiring faults.
names = {};  oks = [];  det = {};

names{end+1} = 'Kalman gain K1 stays in [0,1]';
oks(end+1)   = K1lo >= -1e-12 && K1hi <= 1 + 1e-12;
det{end+1}   = sprintf('K1 in [%.4f, %.4f]', K1lo, K1hi);

% The listing uses the SIMPLIFIED covariance update (P22 = M22 - K2*M12), not
% the Joseph form.  P22 = M22*SIGN2/(M11+SIGN2) analytically, so it is positive
% for any SIGN2 > 0; but SIGN2 = (Vc*tgo*sNoise)^2 collapses as tgo -> 0, and if
% it ever fell far enough below M11 the subtraction would round through zero.
% At the listing's own numbers the margin is about five orders of magnitude --
% this row is what would notice if a parameter change spent it.
names{end+1} = 'covariance diagonal stays non-negative';
oks(end+1)   = Plo >= 0;
det{end+1}   = sprintf('min(P11,P22,P33) = %.4g', Plo);

names{end+1} = 'tgo positive at every guidance update';
oks(end+1)   = tgoLo > 0;
det{end+1}   = sprintf('min tgo = %.3e s', tgoLo);

names{end+1} = 'command never exceeds the acceleration limit';
oks(end+1)   = cmdHi <= P.XLIM*(1 + 1e-9);
det{end+1}   = sprintf('max |nc| = %.4f ft/s^2 vs XLIM = %g', cmdHi, P.XLIM);

names{end+1} = 'Poisson dwell pool never exhausted';
oks(end+1)   = (ovf == 0) && (PO.exhausted == 0);
det{end+1}   = sprintf('%d in-flight overflows, %d rate-check overflows', ovf, PO.exhausted);

% Common random numbers: both laws at a flight time must have consumed the
% identical draws, or BENCHMARK 10 is an unpaired comparison wearing a paired
% comparison's band.  This is the row that would notice a re-draw inside mcBatch.
crnOK = true;  worst = 0;
for k = 1:size(MC.crn,2)
    d = max(abs(MC.crn(:,k) - MC.crn(1,k)));
    worst = max(worst, d);
    crnOK = crnOK && (d <= 1e-9*max(1, abs(MC.crn(1,k))));
end
names{end+1} = 'guidance laws share one random realisation (paired)';
oks(end+1)   = crnOK;
det{end+1}   = sprintf('max cross-law fingerprint gap %.3e', worst);

K = struct();
K.names = names;
K.oks   = logical(oks(:).');
K.details = det;
K.all_ok  = all(K.oks);
bad = find(~K.oks, 1);
if isempty(bad)
    K.first_bad = '';
else
    K.first_bad = sprintf('%s (%s)', names{bad}, det{bad});
end
end


% =============================================================================
function B = benchmarks(R, P, maneuver, PO, HY, EN, DG, tfList)
%BENCHMARKS  See the header for the provenance of every row.  Chapter 31
%   publishes no rms miss number in prose, so the value rows are parameters and
%   the figure claims are ordering rows.
%
%   Rows 1-5 compare THIS RUN'S parameters against Table 31.1.  A caller who
%   deliberately moves XNTIC or XLIM off the book's values SHOULD see them turn
%   red -- that is the row doing its job, not a false alarm.  Rows 6, 7 and 10
%   depend on inputs the book fixed (XNU, TSW, the maneuver type and the 2-to-1
%   advantage), so those SKIP rather than fail when the run is off-book: a
%   check that no longer asks the book's question must not be scored as though
%   it did.
B = emptyBench();
G = 32.2;
TBL = 'Table 31.1, p.817; zarchan.txt:11796-11803';

B = addVal(B, 'target maneuver magnitude nT', 5, P.XNTIC/G, '0.2%', 0.002, ...
    TBL, 'g', sprintf('listing XNTIC = %g ft/s^2', P.XNTIC));

B = addVal(B, 'missile acceleration limit aMAX', 10, P.XLIM/G, '0.2%', 0.002, ...
    TBL, 'g', sprintf('listing XLIM = %g ft/s^2; aMAX/nT = %.3f, the chapter''s 2-to-1 advantage (:11748)', ...
    P.XLIM, P.XLIM/P.XNTIC));

B = addVal(B, 'closing velocity Vc', 5, P.VC/3280, '0.2%', 0.002, ...
    TBL, 'km/s', sprintf('listing VC = %g ft/s, converted with the book''s 3280 ft/km', P.VC));

B = addVal(B, 'measurement noise sigma', 0.1, P.SIGNOISE*1000, '0.2%', 0.002, ...
    'Table 31.1 (0.0001 r) and prose "0.1 mr"; zarchan.txt:11799, :11683', ...
    'mr', 'the table and the prose are the same number in different units');

B = addVal(B, 'process noise density nTMAX^2/tF', 161^2/10, ...
    P.XNTIC^2/P.PHIN_TF, '0.2%', 0.002, ...
    'Qc(3,3) = 161^2/10; zarchan.txt:11684-11688', '(ft/s^2)^2/s', ...
    'the listing pins the denominator at 10 s for EVERY flight time in the sweep, not at the current tF');

if abs(P.XNU - 0.5) < 1e-12
    band6 = max(4*PO.se/0.5, 0.01);
    B = addVal(B, 'Poisson zero-crossing rate', 0.5, PO.rate, ...
        sprintf('%.1f%% (4 s.e.)', 100*band6), band6, ...
        'Fig. 31.3 caption and prose; zarchan.txt:11620, :11660', '1/s', ...
        sprintf('%d replications x 10 s, mean %.3f crossings, mean dwell %.3f s (expected 1/XNU = %.3f)', ...
        PO.runs, PO.mean_count, PO.mean_dwell, 1/P.XNU));
else
    B = addSkip(B, 'Poisson zero-crossing rate', '0.5 /s', ...
        sprintf(['the book''s 0.5 /s is XNU itself; this run set XNU = %g, so ' ...
                 'the measured rate (%.4f /s) is being compared against nothing ' ...
                 'the book published.'], P.XNU, PO.rate), ...
        'zarchan.txt:11620', '1/s');
end

if abs(P.TSW - 2) < 1e-12
    B = addVal(B, 'hybrid: flight fraction without chatter', 80, ...
        100*HY.frac_og, '1%', 0.01, ...
        'Fig. 31.19 prose "removed from 80% of the flight"; zarchan.txt:12010', '%', ...
        sprintf('single flight, TSW = %g s, tF = %g s, vertical-S: %d of %d updates on the optimal branch; (tF-TSW)/tF = %.3f', ...
        HY.TSW, HY.tf, HY.n_og, HY.n_updates, (HY.tf-HY.TSW)/HY.tf));
else
    B = addSkip(B, 'hybrid: flight fraction without chatter', '80%', ...
        sprintf(['the book''s 80%% is (tF-TSW)/tF for Fig. 31.19''s TSW = 2 s at ' ...
                 'tF = 10 s. This run used TSW = %g s, which makes the exact ' ...
                 'arithmetic %.1f%% -- a different number, not a looser band.'], ...
                 P.TSW, 100*(HY.tf - P.TSW)/HY.tf), ...
        'zarchan.txt:12010', '%');
end

% ---- SHAPE 8: effective navigation ratio
Np = EN.Np;
dec  = all(diff(Np) < 0);
tail = abs(Np(end) - 3) <= 0.1;
grow = Np(1) >= 2*Np(end);
B = addShape(B, 'effective nav ratio: ->3 long tgo, grows', ...
    'approaching 3 at long tgo; grows considerably at small tgo', ...
    sprintf('N''(0.5T)=%.2f, N''(200T)=%.3f, monotone=%d', Np(1), Np(end), dec), ...
    dec && tail && grow, 'Fig. 31.1 prose; zarchan.txt:11578 (formula :11573)', ...
    'grid is tgo/T in [0.5,200]; below ~0.25 the listing''s +.0001 guard makes the ratio turn over, so it is not scored there');

% ---- SHAPE 9: bang-bang
frac = DG.nBang/max(DG.nCmd,1);
B = addShape(B, 'differential game command is bang-bang', ...
    'either maximum positive or negative acceleration', ...
    sprintf('%.2f%% of %d commands at +/-aMAX, %d strictly inside', ...
    100*frac, DG.nCmd, DG.nInterior), ...
    DG.nCmd > 0 && DG.nInterior == 0 && frac >= 0.99, ...
    'zarchan.txt:11580', ...
    sprintf('%d commands were exactly zero (sign(ZEM)=0), which is the only other value the law can take', DG.nZero));

% ---- SHAPE 10: the ordering Fig. 31.13 claims.  Poisson maneuver, 2-to-1
%      advantage; anything else is a different experiment, so it SKIPs.
rOG = R.mc.rms(1,:);  rDG = R.mc.rms(2,:);
adv = P.XLIM/P.XNTIC;
if maneuver == 1 && abs(adv - 2) < 1e-9
    B = addShape(B, 'diff-game beats optimal vs Poisson', ...
        'differential game performed much better than optimal guidance', ...
        sprintf('rms(OG)/rms(DG) = %s at tF = %s s', ...
        vstr(rOG./max(rDG,realmin)), vstr(tfList)), ...
        all(rDG < rOG), 'Fig. 31.13 and its prose; zarchan.txt:11865, :11922', ...
        sprintf(['"much better" is an adjective, so only the ORDERING is scored. ' ...
                 'rms OG = %s ft, DG = %s ft (book''s RUN-1 divisor). THIS IS THE ' ...
                 'ONLY STOCHASTIC SCORED ROW: a red here means guidance wiring, ' ...
                 'not a tolerance -- read R.mc.rms before touching anything.'], ...
        vstr(rOG), vstr(rDG)));
else
    B = addSkip(B, 'diff-game beats optimal vs Poisson', ...
        'DG rms < OG rms', ...
        sprintf(['Fig. 31.13 is the POISSON maneuver at a 2-to-1 advantage. This ' ...
                 'run used maneuver %d at %.3f-to-1, so the ordering it would ' ...
                 'score is not the ordering the book claimed. Measured anyway: ' ...
                 'rms OG = %s ft, DG = %s ft.'], maneuver, adv, vstr(rOG), vstr(rDG)), ...
        'zarchan.txt:11865, :11922', 'ft');
end

B = addInfo(B, 'Monte Carlo sample size', ...
    '50 runs at each of 100 flight times (0.1:0.1:10 s)', ...
    sprintf('%d runs at %d flight times (%s s), H = %g s', ...
    P.runs, numel(tfList), vstr(tfList), P.H), ...
    'zarchan.txt:11749', ...
    'reported, not adjudicated: nothing scored here depends on the sweep size, and the board reruns on every commit');

B = addSkip(B, 'rms miss vs flight time (Figs. 31.11-26)', 'curves only', ...
    ['every rms-miss number in Chapter 31 lives on a figure and the extraction ' ...
     'turned each figure into an image link, so there is no published value to ' ...
     'score. The prose states only orderings, which are the shape rows above.'], ...
    'Figs. 31.11-31.26; zarchan.txt:11862-12110', 'ft');
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match mkRow exactly, or the appends
%   throw -- which is the behaviour we want: a struct array that quietly
%   reordered fields would make [B.ok] mean something else.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function r = mkRow(what, kind, book, book_str, ours, ours_str, pct, band, ...
                   tol, ok, skipped, why, source, units, detail)
%MKROW  THE ONE CONSTRUCTOR.  Every row in this file is built here so the field
%   order cannot drift between adders; benchSuite's adaptGeneric is strict about
%   it and a reordered struct array fails with "Subscripted assignment between
%   dissimilar structures", which reads like a typo rather than a schema break.
r = struct('what',what, 'kind',kind, 'book',book, 'book_str',book_str, ...
           'ours',ours, 'ours_str',ours_str, 'pct',pct, 'band',band, ...
           'tol',tol, 'ok',logical(ok), 'skipped',logical(skipped), ...
           'why',why, 'source',source, 'units',units, 'detail',detail);
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
if ~isfinite(ours) || book == 0
    B(end+1) = mkRow(what, 'value', book, sprintf('%.4g',book), ours, ...
        sprintf('%.5g',ours), NaN, band, tol, false, true, ...
        'the computed value is not finite (or the book value is zero)', ...
        source, units, detail);
    return
end
B(end+1) = mkRow(what, 'value', book, sprintf('%.4g',book), ours, ...
    sprintf('%.5g',ours), 100*(ours/book - 1), band, tol, ...
    abs(ours/book - 1) <= tol, false, '', source, units, detail);
end

function B = addShape(B, what, bookStr, oursStr, ok, source, detail)
B(end+1) = mkRow(what, 'shape', NaN, bookStr, NaN, oursStr, NaN, 'shape', ...
    NaN, ok, false, '', source, '-', detail);
end

function B = addInfo(B, what, bookStr, oursStr, source, why)
%ADDINFO  Reported, never scored.  skipped = true keeps it out of the pass count
%   in BOTH directions: an INFO row can neither pass nor fail.
B(end+1) = mkRow(what, 'info', NaN, bookStr, NaN, oursStr, NaN, 'info', ...
    NaN, false, true, why, source, '-', '');
end

function B = addSkip(B, what, bookStr, why, source, units)
%ADDSKIP  A check that COULD NOT RUN.  Never a pass: a check that did not happen
%   is a different thing from one that succeeded.
B(end+1) = mkRow(what, 'skip', NaN, bookStr, NaN, '--', NaN, 'skip', ...
    NaN, false, true, why, source, units, '');
end


% =============================================================================
function s = vstr(v)
s = ['[' strtrim(sprintf('%.3g ', v)) ']'];
end

function s = render(R, ev)
L = {};
L{end+1} = 'mgs.zarchan.lawEffectivenessMC -- Zarchan Listing 31.1 (Ch.31)';
L{end+1} = sprintf('  units : %s', R.units);
L{end+1} = sprintf('  sweep : maneuver %d, tF = %s s, %d runs/point, H = %g s, %d vector steps', ...
    R.p.maneuver, vstr(R.p.tf), R.p.runs, R.p.H, R.p.n_vector_steps);
for j = 1:numel(R.mc.laws)
    L{end+1} = sprintf('  rms miss, %-18s : %s ft', R.mc.laws{j}, vstr(R.mc.rms(j,:))); %#ok<AGROW>
end
L{end+1} = sprintf('  hybrid TSW=%g s, tF=%g s : %d of %d updates on optimal guidance (%.1f%%)', ...
    R.hybrid.TSW, R.hybrid.tf, R.hybrid.n_og, R.hybrid.n_updates, 100*R.hybrid.frac_og);
L{end+1} = sprintf('  Poisson rate : %.4f /s (s.e. %.4f) over %d replications', ...
    R.poisson.rate, R.poisson.se, R.poisson.runs);
L{end+1} = '  benchmarks:';
for i = 1:numel(R.bench)
    b = R.bench(i);
    if b.skipped
        mark = ' -- ';
    elseif b.ok
        mark = ' ok ';
    else
        mark = 'FAIL';
    end
    if strcmpi(b.kind,'value')
        L{end+1} = sprintf('   [%s] %-46s book %-9s ours %-9s (%+.2f%%, band %s)', ...
            mark, b.what, b.book_str, b.ours_str, b.pct, b.band); %#ok<AGROW>
    else
        L{end+1} = sprintf('   [%s] %-46s %s | %s', mark, b.what, b.book_str, b.ours_str); %#ok<AGROW>
    end
end
L{end+1} = '  structural:';
for i = 1:numel(R.selfcheck.names)
    if R.selfcheck.oks(i), m = ' ok '; else, m = 'FAIL'; end
    L{end+1} = sprintf('   [%s] %-46s %s', m, R.selfcheck.names{i}, R.selfcheck.details{i}); %#ok<AGROW>
end
if any(ev)
    L{end+1} = sprintf('  scored %d row(s); pass = %d', sum(ev), R.pass);
else
    L{end+1} = '  NOTHING SCORED.';
end
s = strjoin(L, newline);
end
