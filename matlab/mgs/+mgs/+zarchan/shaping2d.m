function R = shaping2d(opts)
%MGS.ZARCHAN.SHAPING2D  Zarchan Listing 30.1 -- 2-D engagement against a
%   STATIONARY target with a TERMINAL FLIGHT-PATH-ANGLE constraint, by either
%   optimal trajectory shaping or biased proportional navigation, with the
%   acceleration command aimed either perpendicular to the LINE OF SIGHT or
%   perpendicular to the MISSILE VELOCITY VECTOR.
%
%   R = mgs.zarchan.shaping2d()                          % board + all book cases
%   R = mgs.zarchan.shaping2d('iguid',1,'ichoice',1)     % pick a 2x2 corner
%   R = mgs.zarchan.shaping2d('gamfdeg',-60,'benchmark',false)   % off-book run
%
%   Line-by-line port of Listing 30.1, "Two-Dimensional Engagement Simulation
%   with Optimal Trajectory Shaping and Biased Proportional Navigation
%   Guidance Options", Zarchan, Tactical and Strategic Missile Guidance,
%   Ch. 30, pp. 780-783.  Book source in this repo:
%       matlab/zarchan/_source/zarchan.txt:10931  (the listing heading)
%       matlab/zarchan/_source/zarchan.txt:10934  (the data statement + loop)
%       matlab/zarchan/_source/zarchan.txt:10930  (the prose that configures it)
%       matlab/zarchan/_source/zarchan.txt:10918-10928 (the bias derivation)
%   Transcription this port was made from:
%       matlab/zarchan/_listings/listing_30_01.m
%
% ===========================================================================
%   UNITS.  THIS IS THE ONE LISTING IN THE BOOK THAT IS NOT IN FEET -- AND IT
%   IS NOT PURELY METRIC EITHER.  READ THIS PARAGRAPH BEFORE QUOTING ANYTHING.
% ===========================================================================
%   The prose says the units "have been converted to the metric system"
%   (:10930).  What the listing actually does is:
%
%       INPUTS   metric      RT1IC = 10000.*3.28   VM = 250.*3.28
%       LOOP     FEET        ft, ft/s, ft/s^2, XNCLIM = 32.2*XNCLIMG
%       OUTPUTS  metric      /3280 -> km, /3.28 -> m/s, /32.2 -> g,
%                            /(3.28*3.28) -> m^2/s^3
%
%   So the PUBLIC SURFACE of this file is metric -- every option you pass and
%   every field you read is m, m/s, km, g, deg, m^2/s^3 -- and the integration
%   underneath is in feet, exactly as printed.  Nothing is "helpfully"
%   converted in either direction:
%
%     * 3.28 is kept, NOT 3.28084.  The round trip 250 -> 820 -> 250 is exact
%       because the same 3.28 is used both ways, so the port is self-consistent;
%       what the 3.28 costs is that the book's "10 g" limit is really
%       32.2/3.28 = 9.8171 m/s^2 per g, i.e. 98.171 m/s^2, 0.11% above
%       10*9.80665.  There is no gravity anywhere in the dynamics -- 32.2
%       appears ONLY in the acceleration limit and in the g-units of the
%       logged command -- so that 0.11% is the whole of its effect.
%     * 3280 is the foot->km divisor and equals 1000*3.28 exactly, so the km
%       and m/s channels agree with each other.
%     * 57.3 is the book's rad<->deg constant, NOT 180/pi.  GAMF = -90/57.3 is
%       -1.5707... which is NOT -pi/2, and the published trajectories were made
%       with that number.
%
%   THE REST OF MGS.ZARCHAN IS IN FEET.  mgs.zarchan.engage2d guards against a
%   metric speed being handed to a feet file; this file has to guard the
%   OPPOSITE way, and does (see the vmic guard below): 3000 handed to a metric
%   file is a Mach-9 missile that still returns a plausible-looking answer.
%
% ===========================================================================
%   WHY THIS LISTING MATTERS TO THIS PROJECT MORE THAN ANY OTHER IN THE BOOK
% ===========================================================================
%   Every clause of the book's own description of the nominal case (:10930) is
%   our problem statement:
%
%     stationary target                  -> our bullseye, fixed, 0 m AMSL
%     final flight path angle -90 deg    -> our dive-angle requirement
%     launched with a 30-deg heading err -> our surviving discriminant |eps0|
%     metric, 250 m/s                    -> our cruise is 150-210 m/s ground
%     command _|_ VM instead of _|_ LOS  -> the _|_VM branch IS Pure PN, which
%                                           is what we fly; the whole rest of
%                                           the book is the _|_LOS branch
%
%   Two results here bear directly on things we have measured in flight:
%
%   (1) THE _|_VM BRANCH CONSERVES SPEED EXACTLY AND THE _|_LOS BRANCH DOES
%       NOT.  Not approximately -- a force perpendicular to the velocity does
%       no work, so |V| is invariant to machine precision (this port measures
%       ~2e-11 relative).  The _|_LOS branch loses ~74% of the speed to hit
%       the same target with the same terminal angle.  That is the mechanism
%       behind [[terminal-204-is-groundspeed]] and [[dive-pace-saturates-vmin]]:
%       aiming the command at the LOS rather than across the velocity is a
%       SPEED BUDGET decision, not a bookkeeping detail.  Zarchan's own verdict
%       (:10980) is that the _|_LOS velocity loss "cannot be tolerated" and
%       that all further trajectory-shaping work assumes ICHOICE = 1.
%
%   (2) BIASED PN REACHES THE SAME TERMINAL ANGLE WITHOUT TIME-TO-GO.  Our
%       predictor is measurably broken ([[predictor-underestimates-dive]],
%       median +18.7 s), and the optimal law here divides by tgo.  Biased PN
%       needs only LOS rate and own speed, and this port measures its cost
%       against optimal on the book's own performance index.
%
%   WHAT IS NOT MODELLED, because this file is very easy to over-read: no
%   gravity, no drag, no thrust, no autopilot lag, no seeker, no wind, no
%   airframe.  Constant-mass point mass whose speed changes ONLY through the
%   along-velocity component of the commanded acceleration.  The single
%   actuator authority limit is a symmetric +/-XNCLIM on the command.  A miss
%   distance off this board is not a sortie prediction.
%
% ===========================================================================
%   THE OCR DAMAGE, AND HOW EACH PIECE WAS RESOLVED
% ===========================================================================
%   The PDF text layer destroys MATLAB comparison operators: ',' is '<' and
%   '.' is '>'.  The transcription resolved the ones settleable from shape and
%   left THREE VARIABLE-TO-VARIABLE comparisons and THREE LIGATURE RESIDUES.
%   All six are recorded in R.transcription, and the first four rows of the
%   STRUCTURAL block below are DELETE EXPERIMENTS that actually run the
%   alternative reading rather than asserting about it.  A comment claiming a
%   guard exists is not a guard ([[promised-guards-that-do-not-exist]]).
%
%   COMPARISON 1.  "if T,TBEG"    -> if T < TBEG        (biased-PN branch)
%   COMPARISON 2.  "elseif T,TEND"-> elseif T < TEND
%       WHY:  the prose (:10930) says "the bias is turned on at time zero
%       (TBEG = 0) and turned off 30 s later (DELT = 30)", so the bias must be
%       ACTIVE on TBEG <= T < TEND.  With '<' the three branches read
%       before / during / after the window, which is that sentence.
%       WHY IT NEEDED AN EXPERIMENT:  under the inverted reading (T > TBEG)
%       the first branch swallows the whole flight at TBEG = 0, the bias is
%       NEVER applied, and the missile STILL HITS THE TARGET -- miss ~0.002 m.
%       The only channel that shows the difference is the terminal flight path
%       angle, which lands on exactly -15 deg instead of -90 deg.  That is
%       gamIC/(1-N) = 30/(1-3), the unbiased-PN closed form from the book's own
%       integrated law (:10922), and this file checks it to <0.01 deg.  A
%       miss-distance-only regression would have passed the wrong reading.
%
%   COMPARISON 3.  "if XNC.XNCLIM" -> if XNC > XNCLIM   (upper limiter)
%       WHY:  the very next statement is the transcription-resolved
%       "if XNC < -XNCLIM, XNC = -XNCLIM", so the pair is a symmetric
%       saturation and the first arm must be the upper one.
%       WHY IT NEEDED AN EXPERIMENT:  at the book's own 10 g limit the clamp
%       is INERT -- the logged peak demand is under 3 g -- so no book case
%       discriminates the two readings by outcome.  The delete experiment
%       runs the inverted arm and measures 100% of derivative evaluations
%       clamped and a ~9.7 km miss, and a second experiment at a 2 g limit
%       shows the correct arm firing AND RELEASING (~33% of evaluations).
%
%   LIGATURES 1-3.  The three "IGUID (1/4) 0", "ICHOICE (1/4) 0",
%       "ICHOICE (1/4) 1" residues all sit inside ONE leaked prose paragraph
%       (transcription lines 43-57, book :10961 plus the axis text of
%       Fig. 30.2).  Prose leaked into the code region is DROPPED, never
%       repaired into code.  Nothing was lost: that paragraph is the Fig.
%       30.2-30.5 discussion and it is scored as benchmark rows below.
%
%   ALSO REPAIRED, and settleable from shape alone (so not counted above):
%       "while VC .=0"    -> while VC >= 0
%       "if RTM ,1000"    -> if RTM < 1000   (terminal step refinement; RTM
%                            starts at 32800 ft and decreases, so '<' is the
%                            only reading in which the fine step is terminal)
%       "while STEP ,=1"  -> while STEP <= 1 ('>=' never exits: STEP is set to
%                            2 and 2 >= 1 forever)
%       "if XNC,-XNCLIM"  -> if XNC < -XNCLIM
%       "if S.= .09999"   -> if S >= .09999  (the 0.1 s log cadence)
%
% ===========================================================================
%   THE BIAS SIGN -- A BOOK ERRATUM, AND THE LISTING WINS
% ===========================================================================
%   The prose (:10928) prints
%
%       bias = [ gamF*(N-1) + N*lamIC - gamIC ] / dt                (prose)
%
%   while the listing (:10934) prints
%
%       BIASDEG = (-GAMFDEG*(XNP-1.) + XNP*XLAM*57.3 - GAMDEG)/DELT (listing)
%                  ^ leading minus
%
%   Solve the book's OWN integrated law, printed two lines above the formula
%   (:10922), for a stationary target where lamF = gamF:
%
%       gamF - gamIC = N*(gamF - lamIC) + bias*dt
%   =>  bias*dt      = -gamF*(N-1) + N*lamIC - gamIC        <-- the LISTING
%
%   so the prose has a dropped minus sign, and it is a SIGN, not a rounding:
%   on the nominal numbers the listing gives +5 deg/s and the prose -7 deg/s.
%   +5 is also the only physically sensible one -- to arrive at -90 deg the
%   missile must first LOFT so that it has altitude to dive out of, which is
%   the ~3-5 km apogee the book's own Fig. 30.2/30.6 draw.  The 'text' option
%   runs the prose variant; the structural block scores it, and it misses by
%   ~5.95 km.  Same erratum, same conclusion, already on record in this repo
%   at matlab/ref/zarchan_listing30_1.m and matlab/zarchan/README.md.
%
% ===========================================================================
%   BENCHMARKS -- AND WHAT IS DELIBERATELY NOT SCORED
% ===========================================================================
%   Ch.30's figures survived extraction as image links with no data on them.
%   Everything scored here therefore comes from PROSE, either as a published
%   number or as an ordering/sign claim about a figure.  Three things are
%   reported as INFO and NOT scored, on purpose:
%
%     * "the maximum accelerators are similar" (:11020).  An adjective, and
%       the three peaks span a factor of 1.87.  Printed, never adjudicated.
%     * "the missile trajectories are nearly identical" (:10961).  An
%       adjective.  What IS scored is the apogee agreement under a band this
%       file declares as its own; the sharper max-altitude-separation metric
%       does NOT meet that band and is printed beside it rather than being
%       hidden.  Do not relax the band to turn it green.
%     * the TERMINAL commanded acceleration of the optimal law.  It is a
%       genuine 0/0: the 2*Vc*(lam-gamF)/tgo term has both numerator and
%       denominator going to zero at intercept, and this port measures the
%       raw demand reaching ~1e7 g on the final one or two derivative
%       evaluations -- all of them at RTM < 0.01 ft, one of them after VC has
%       already changed sign.  A number read off a diverging curve is not a
%       benchmark.  What IS scored is the peak over the book's own 0.1 s log,
%       because that is what Fig. 30.3/30.7 plot: the book's ordinate ranges
%       (-3..1 g on Fig. 30.3, -5..1 g on Fig. 30.7) match the LOGGED peaks
%       and not the per-step ones, which is how we know the figures are drawn
%       from the same 0.1 s array.
%
%   ONE PUBLISHED NUMBER, and it does not land exactly: ":11058 -- the
%   performance index for biased proportional navigation starting at time zero
%   is only 33% larger."  This port computes ~29.4%.  The 33 is read off
%   Fig. 30.9, whose printed ordinate ticks run 0..12000 in steps of 2000, and
%   (12000-9000)/9000 = 33.3% is exactly what a reader gets from the gridlines;
%   the computed pair is ~9295 and ~12029.  Scored in the 'graph' band for
%   that reason, with both numbers printed.  The band is not there to buy
%   slack -- an ordering row scores X_optimal < X_biasedPN exactly, separately.
%
% ===========================================================================
%   RETURNED STRUCT
% ===========================================================================
%   R.run          the ONE engagement selected by 'iguid'/'ichoice'.  Metric
%                  channels on the book's 0.1 s grid, plus scalars.  On a
%                  default call this IS one of the board's four corners and is
%                  not flown a second time; R.run_reused says so, and the
%                  printed evaluation count does not double-count it.
%   R.cases        every book case: .opt_los .opt_v .bpn_v0 .bpn_v10 .bpn_los
%                  and .sweep(4) for the Fig. 30.10/30.11 gamF family
%   R.bench        computed vs published, one row per claim, each with .source
%                  (zarchan.txt line) and .band.  Schema is the one
%                  mgs.zarchan.benchSuite consumes through benchRows().
%   R.pass         true iff every EVALUATED row passed AND at least one of
%                  them described a TRAJECTORY.  FALSE when nothing was
%                  evaluated -- that means "not checked", not "fine" -- and
%                  false when only the configuration echoes ran, which is what
%                  an off-book call produces and which would otherwise print
%                  a full-looking "10/10 PASS".
%   R.n_evaluated, R.n_result_evaluated   the two counts behind that rule
%   R.selfcheck    structural invariants incl. the four delete experiments
%   R.transcription the six unresolved OCR items and how each was settled
%   R.cfg, R.offbook, R.units, R.listing, R.entry, R.summary
%
%   RUNTIME.  Not stated here, because two previous waves in this package
%   stated one and were wrong.  The board prints its own elapsed time and its
%   own derivative-evaluation count; measure it.  For scale, the step rule in
%   the listing (1e-4 s, dropping to 1e-5 s inside 1000 ft) puts the twelve
%   benchmark engagements in the region of 1e7 derivative evaluations, which
%   is an ESTIMATE from the transcription's own arithmetic and not a
%   measurement.
%
%   See also MGS.ZARCHAN.ENGAGE2D (Listing 2.1, the zero-lag planar baseline),
%   MGS.ZARCHAN.ADDITIVELAW, MGS.ZARCHAN.BENCHSUITE, MGS.ZARCHAN.CROSSCHECK.

arguments
    % ---- the 2x2 the chapter is about.  NOT gated: the benchmark runs all
    %      four corners no matter what these are set to, so a caller may pick
    %      a corner without losing the board.  These two only choose which
    %      engagement lands in R.run.
    opts.iguid    double {mustBeReal, mustBeFinite} = []   % 0 optimal shaping, 1 biased PN
    opts.ichoice  double {mustBeReal, mustBeFinite} = []   % 0 a_cmd _|_ LOS,   1 _|_ velocity
    % ---- the book's data statement (:10934), metric.  Defaults are []: empty
    %      means "use the book value", non-empty means "the caller chose this",
    %      and that is the ONLY mechanism telling an off-book run apart from a
    %      broken one.  Do not replace them with literals.
    opts.xnp      double {mustBeReal, mustBeFinite} = []   % XNP     navigation ratio N
    opts.gamdeg   double {mustBeReal, mustBeFinite} = []   % GAMDEG  initial flight path angle [deg]
    opts.gamfdeg  double {mustBeReal, mustBeFinite} = []   % GAMFDEG desired terminal gamma  [deg]
    opts.delt     double {mustBeReal, mustBeFinite} = []   % DELT    bias-on duration [s]
    opts.tbeg     double {mustBeReal, mustBeFinite} = []   % TBEG    bias turn-on time [s]
    opts.xnclimg  double {mustBeReal, mustBeFinite} = []   % XNCLIMG command limit [g]
    opts.vmic     double {mustBeReal, mustBeFinite} = []   % VM      initial missile speed [m/s]
    opts.rangem   double {mustBeReal, mustBeFinite} = []   % RT1IC   downrange to target [m]
    opts.rt2m     double {mustBeReal, mustBeFinite} = []   % RT2IC   target altitude [m] -- see below
    opts.biasform (1,:) char {mustBeMember(opts.biasform, ...
                             {'listing','text','none'})} = 'listing'
    % ---- discretization (:10934).  Gated SEPARATELY: moving these moves the
    %      miss floor, i.e. the criterion, not the answer.
    opts.h        double {mustBeReal, mustBeFinite} = []   % H  [s] coarse
    opts.hfine    double {mustBeReal, mustBeFinite} = []   % H  [s] inside rfine
    opts.rfine    double {mustBeReal, mustBeFinite} = []   % [ft] range at which H drops
    % ---- housekeeping (none of it is in the book) --------------------------
    opts.tmax     (1,1) double {mustBePositive} = 300      % runaway guard [s]
    opts.allowFastMissile (1,1) logical = false            % see the vmic guard
    opts.benchmark (1,1) logical = true    % THE ONLY SWITCH.  Never nargin.
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
    opts.savedat   char = ''   % path for the listing's datfil.txt.  NO (1,:)
                               % size spec: the default '' is 0x0 char and a
                               % (1,:) constraint on it is a first-call throw
                               % that no static check sees.
end

% ⚠️⚠️ NOT `nargin == 0`.  In a function whose `arguments` block declares ONLY
%    name-value options, MATLAB reports nargin = 0 NO MATTER HOW MANY WERE
%    PASSED -- the packed opts struct is not a positional argument.  Measured
%    on R2025b in this package.  engage2d's first draft used it as a mode
%    switch, made the public entry call itself, and died with
%    MATLAB:lang:StackOverflow on the first bare call.  Here it would have
%    been quieter: every caller's single run silently replaced by the board.
%    ⇒ 'benchmark' is the switch, it defaults TRUE, and the benchmark cases
%    are run through the LOCAL flyOne(), never through this entry point.

N = bookNominal();
[cfg, offbook, discOffbook] = resolveConfig(opts, N);

R = struct();
R.listing = 'Zarchan Listing 30.1 (zarchan.txt:10931, Ch.30 pp.780-783)';
R.units   = ['PUBLIC metric: m, m/s, km, g, deg, m^2/s^3.  LOOP in FEET with ' ...
             'g = 32.2, 3.28 m->ft, 3280 ft->km, 57.3 rad->deg -- the ' ...
             'listing''s own constants, none of them replaced.'];
R.entry   = 'mgs.zarchan.shaping2d(''benchmark'',true,''assert'',false,''render'',false)';
R.opts    = opts;
R.cfg     = cfg;
R.offbook = offbook;
R.disc_offbook = discOffbook;
R.nominal = N;
R.transcription = transcriptionNotes();

t0 = tic;
if cfg.benchmark
    % The board first, THEN the caller's single run -- because on a default
    % call the caller's run is one of the four corners the board already flew,
    % and flying it twice is a fifth of the whole cost for nothing.  pickRun
    % returns the already-computed case when it is bit-for-bit the same
    % request and flies a fresh one otherwise; R.run_reused says which.
    R.cases     = bookCases(cfg);
    [R.run, R.run_reused] = pickRun(R.cases, cfg);
    R.selfcheck = structuralCheck(R.cases, cfg);
    [R.bench, nPre] = benchmarks(R.cases, R.selfcheck, cfg, offbook, discOffbook);
    ev          = ~[R.bench.skipped];
    % evResult drops the configuration-echo preamble.  R.pass requires that a
    % row describing an actual TRAJECTORY was evaluated -- see benchmarks().
    evResult    = ev;
    evResult(1:min(nPre, numel(evResult))) = false;
    R.n_evaluated = sum(ev);
    R.n_result_evaluated = sum(evResult);
    R.pass      = any(evResult) && all([R.bench(ev).ok]);
else
    R.cases     = struct('skipped', true);
    R.run       = flyOne(cfg);
    R.run_reused = false;
    R.selfcheck = struct('skipped', true);
    R.bench     = emptyBench();
    ev          = false;
    R.n_evaluated = 0;
    R.n_result_evaluated = 0;
    % NOT true.  Nothing was checked, and the summary says exactly that.
    R.pass      = false;
end
R.elapsed_s = toc(t0);
R.nevals    = totalEvals(R);

R.summary = render(R, cfg, offbook, discOffbook, ev);
if cfg.render, fprintf('%s\n', R.summary); end
if cfg.plot,   drawFigures(R, cfg); end
if ~isempty(cfg.savedat), writeDatfil(R.run, cfg.savedat); end

% ---- adjudication.  ⚠️ 'assert', false must still RETURN the result with the
%      verdict attached.  A previous wave in this package errored
%      unconditionally and nobody could see WHICH side was wrong.
if cfg.benchmark
    if R.n_result_evaluated == 0
        complain(cfg.assert, 'mgs:zarchan:shaping2d:noBenchmark', ...
            ['not one RESULT row could be evaluated (%d configuration echoes ' ...
             'were, and they only lock the data statement).  The published ' ...
             'results ' ...
             'live at the Listing 30.1 nominal (10 km downrange, VM = 250 m/s, ' ...
             'gamIC = 30 deg, gamF = -90 deg, N = 3, 10 g limit, TBEG = 0, ' ...
             'DELT = 30 s); this run moved %s.  Refusing to hand back an ' ...
             'unvalidated port silently -- pass ''benchmark'', false if you ' ...
             'meant to run off-book.'], R.n_evaluated, offbookStr(offbook));
    elseif ~R.pass
        bad = find(~[R.bench.ok] & ev);
        complain(cfg.assert, 'mgs:zarchan:shaping2d:benchmark', ...
            ['%d of %d evaluated book values are outside tolerance (first: ' ...
             '%s -- book %s, ours %s).  The full board is in R.summary.'], ...
            numel(bad), sum(ev), R.bench(bad(1)).what, ...
            R.bench(bad(1)).book_str, R.bench(bad(1)).ours_str);
    end
    if isfield(R.selfcheck, 'ok') && ~R.selfcheck.ok
        complain(cfg.assert, 'mgs:zarchan:shaping2d:structural', ...
            ['a STRUCTURAL self-check failed: %s.  These are wiring and ' ...
             'OCR-resolution invariants, not book values -- a failure here ' ...
             'voids every benchmark row above it.'], R.selfcheck.first_fail);
    end
end
end


% =============================================================================
%   THE BOOK CONSTANTS.  ONE COPY, HERE.
% =============================================================================
function N = bookNominal()
%BOOKNOMINAL  Every literal from the Listing 30.1 data statement
%   (zarchan.txt:10934), cross-checked against the prose at :10930.
N.iguid   = 0;        % IGUID   = 0 optimal trajectory shaping, 1 biased PN   :10930
N.ichoice = 0;        % ICHOICE = 0 command _|_ LOS, 1 command _|_ velocity   :10930
N.xnp     = 3;        % XNP     navigation ratio N.  Listing literal only --
                      %         the prose names no value.  Note the book is
                      %         careful (:10908) that this N is NOT the
                      %         effective ratio N' of Ch.2; for a STATIONARY
                      %         target Vc ~ VM so they coincide, which is the
                      %         only reason a 3 here is comparable with a 3
                      %         in engage2d.
N.gamdeg  = 30;       % GAMDEG  "launched with a 30-deg heading error (or
                      %         initial flight path angle of 30 deg)"         :10930
N.gamfdeg = -90;      % GAMFDEG "a final flight path angle of -90 deg"        :10930
N.delt    = 30;       % DELT    "turned off 30 s later (DELT = 30)"           :10930
N.tbeg    = 0;        % TBEG    "the bias is turned on at time zero"          :10930
N.xnclimg = 10;       % XNCLIMG command limit [g].  Listing literal only.
N.vmic    = 250;      % VM      "the initial missile velocity is 250 m/s"     :10930
N.rangem  = 10000;    % RT1IC   "a stationary target 10 km downrange"         :10930
N.rt2m    = 0;        % RT2IC   the listing hard-codes 0: the target sits at
                      %         the launch altitude.  ⚠️ THIS IS THE ONLY LINE
                      %         IN THIS FILE THAT IS NOT ZARCHAN'S: it is
                      %         exposed because our engagement releases ABOVE a
                      %         sea-level bullseye and needs rt2m = -z0.  The
                      %         default 0 reproduces the book exactly and every
                      %         benchmark row gates on it.
N.h       = 1e-4;     % H       coarse step [s]                               :10934
N.hfine   = 1e-5;     % H       step inside rfine [s]                         :10934
N.rfine   = 1000;     % [ft]    "if RTM < 1000".  ⚠️ FEET: this threshold is
                      %         INSIDE the loop, which is in feet, so it is
                      %         305 m and not 1000 m.  Converting it "to be
                      %         consistent with the metric interface" would
                      %         triple the refined region and change nothing
                      %         visible except the miss floor.
end


function [cfg, offbook, discOffbook] = resolveConfig(opts, N)
%RESOLVECONFIG  Book value where the caller passed nothing, caller's value
%   otherwise, plus TWO lists of what they moved.
%
%   TWO lists, because they gate different rows:
%     offbook      MODEL parameters.  Moving one puts the book's published
%                  results out of reach and skips essentially the whole board.
%                  iguid/ichoice are deliberately NOT in it -- the benchmark
%                  runs all four corners regardless, so choosing a corner must
%                  not cost you the board.
%     discOffbook  DISCRETIZATION.  Moving one moves the miss floor
%                  hfine*|V|, i.e. the CRITERION the miss rows are stated in,
%                  so only those rows skip.
f = fieldnames(N);
cfg = struct();
offbook = {};
discOffbook = {};
DISC  = {'h', 'hfine', 'rfine'};
UNGATED = {'iguid', 'ichoice'};
for i = 1:numel(f)
    k = f{i};
    if isempty(opts.(k))
        cfg.(k) = N.(k);
        continue
    end
    cfg.(k) = opts.(k);
    if isequal(cfg.(k), N.(k)) || any(strcmp(k, UNGATED))
        continue
    end
    if any(strcmp(k, DISC))
        discOffbook{end+1} = k; %#ok<AGROW>
    else
        offbook{end+1} = k; %#ok<AGROW>
    end
end
cfg.biasform = opts.biasform;
if ~strcmp(cfg.biasform, 'listing')
    offbook{end+1} = 'biasform';
end

% clampsense is NOT a public option and is NOT in the arguments block.  It
% exists so structuralCheck can RUN the inverted OCR reading of
% "if XNC.XNCLIM" instead of merely asserting about it.  Reachable only from
% inside this file; a caller cannot select the wrong law by accident.
cfg.clampsense = 'gt';

cfg.tmax      = opts.tmax;
cfg.allowFastMissile = opts.allowFastMissile;
cfg.benchmark = opts.benchmark;
cfg.render    = opts.render;
cfg.plot      = opts.plot;
cfg.assert    = opts.assert;
cfg.savedat   = opts.savedat;

% ---- guards.  A function that cannot do what it was asked ERRORS. --------
if ~isscalar(cfg.iguid) || ~any(cfg.iguid == [0 1])
    error('mgs:zarchan:shaping2d:iguid', ...
        ['iguid must be 0 (optimal trajectory shaping) or 1 (biased PN); ' ...
         'got %s.  The listing branches on ''if IGUID==0'' (:10934), so ANY ' ...
         'other value silently selects biased PN -- a different guidance law ' ...
         'that still flies and still hits.'], mat2str(cfg.iguid));
end
if ~isscalar(cfg.ichoice) || ~any(cfg.ichoice == [0 1])
    error('mgs:zarchan:shaping2d:ichoice', ...
        ['ichoice must be 0 (command _|_ LOS) or 1 (command _|_ velocity); ' ...
         'got %s.  The listing branches on ''if ICHOICE==0'', so any other ' ...
         'value silently selects the _|_velocity branch -- which conserves ' ...
         'speed where the other loses roughly three quarters of it.'], ...
        mat2str(cfg.ichoice));
end
if cfg.vmic > 1000 && ~cfg.allowFastMissile
    error('mgs:zarchan:shaping2d:units', ...
        ['vmic = %g.  ⚠️ THIS FILE''S INTERFACE IS METRIC (the rest of ' ...
         'mgs.zarchan is in feet): %g m/s is %.0f ft/s.  The likely cause is ' ...
         'a Zarchan foot-per-second speed passed straight in -- and that does ' ...
         'not fail, it simulates a Mach-9 missile and hands back a "miss ' ...
         'distance" in metres that reads plausibly. Convert ' ...
         '(ft/s / 3.28 = m/s), or pass ''allowFastMissile'', true.'], ...
        cfg.vmic, cfg.vmic, cfg.vmic*3.28);
end
if cfg.vmic <= 0
    error('mgs:zarchan:shaping2d:vmic', 'vmic must be > 0; got %g.', cfg.vmic);
end
if cfg.rangem <= 0
    error('mgs:zarchan:shaping2d:rangem', ...
        ['rangem must be > 0; got %g.  At zero the initial RTM is 0 and ' ...
         'XLAM = atan2(0,0) is 0 by convention, so the run would start with ' ...
         'a divide by RTM and produce NaN rather than refusing.'], cfg.rangem);
end
if cfg.delt == 0 && cfg.iguid == 1 && ~strcmp(cfg.biasform, 'none')
    error('mgs:zarchan:shaping2d:delt', ...
        ['delt = 0 with biased PN: BIASDEG divides by DELT (:10934) and ' ...
         'would be Inf.  MATLAB does not stop there -- the command goes Inf, ' ...
         'the clamp catches it at XNCLIM, and the run continues to a ' ...
         'plausible-looking miss.  Set delt > 0, or ''biasform'', ''none''.']);
end
if cfg.xnclimg <= 0
    error('mgs:zarchan:shaping2d:xnclimg', ...
        ['xnclimg must be > 0; got %g.  At 0 the symmetric clamp pins the ' ...
         'command to exactly zero and the missile flies straight -- a ' ...
         'ballistic run reported as a guided one.'], cfg.xnclimg);
end
if cfg.h <= 0 || cfg.hfine <= 0
    error('mgs:zarchan:shaping2d:step', 'h and hfine must both be > 0.');
end
end


% =============================================================================
%   LISTING 30.1 ITSELF
% =============================================================================
function r = flyOne(c)
%FLYONE  The listing, line by line.  The book's variable names are kept
%   throughout so a side-by-side diff against zarchan.txt:10934 stays
%   readable, and every constant it prints is used as printed.
%
%   ⚠️ EVERY CONSTANT THIS FUNCTION NEEDS IS DEFINED **INSIDE IT**.  MATLAB
%   local functions CANNOT see the main function's workspace: a port in this
%   package put g = 32.2 in the main function, read it from a local one, and
%   threw `Unrecognized function or variable` on its first run -- which static
%   checking could not see, because the name is a perfectly legal identifier.

G   = 32.2;    % the book's g, ft/s^2.  NOT 9.80665, NOT 9.81.
DEG = 57.3;    % the book's rad<->deg constant.  NOT 180/pi (= 57.29578).
FT  = 3.28;    % the book's metre -> foot constant.  NOT 3.28084.
KM  = 3280.;   % the book's foot -> kilometre divisor.  Equals 1000*FT exactly.

n = 0;
IGUID = c.iguid;  ICHOICE = c.ichoice;

GAMDEG = c.gamdeg;  GAMIC = GAMDEG/DEG;
DELT = c.delt;  TBEG = c.tbeg;  TEND = TBEG + DELT;
XNP = c.xnp;
RM1IC = 0.;  RM2IC = 0.;
RT1IC = c.rangem*FT;  RT2IC = c.rt2m*FT;
VM = c.vmic*FT;
XNCLIMG = c.xnclimg;  GAMFDEG = c.gamfdeg;
XNCLIM = G*XNCLIMG;
RM1 = RM1IC;  RM2 = RM2IC;  RT1 = RT1IC;  RT2 = RT2IC;
VT1 = 0.;  VT2 = 0.;         % the target is STATIONARY.  Listing 24.2's target
                             % flight-path-rate line (BETAD = XNT/VT) is gone
                             % from Listing 30.1 precisely because it divides
                             % by zero here -- see zarchan.txt:8338.
T = 0.;  S = 0.;
RTM1 = RT1-RM1;  RTM2 = RT2-RM2;  RTM = sqrt(RTM1^2+RTM2^2);
XLAM = atan2(RTM2, RTM1);
VM1 = VM*cos(GAMIC);  VM2 = VM*sin(GAMIC);
VTM1 = VT1-VM1;  VTM2 = VT2-VM2;
VC = -(RTM1*VTM1+RTM2*VTM2)/RTM;
GAMF = GAMFDEG/DEG;

% The bias.  See the header: the LISTING form is authoritative; 'text' is the
% prose form with its dropped minus, kept runnable so the erratum stays a
% testable claim instead of a comment.  XLAM here is the INITIAL LOS angle
% lamIC, evaluated before the loop -- that is what the derivation at :10922
% integrates from.
switch c.biasform
    case 'listing', BIASDEG = (-GAMFDEG*(XNP-1.) + XNP*XLAM*DEG - GAMDEG)/DELT;
    case 'text',    BIASDEG = ( GAMFDEG*(XNP-1.) + XNP*XLAM*DEG - GAMDEG)/DELT;
    case 'none',    BIASDEG = 0.;
    otherwise
        % Unreachable while mustBeMember guards the option.  Kept so that
        % adding a fourth form and forgetting this switch is a stop, not a run
        % at whatever BIASDEG happened to be left in scope.
        error('mgs:zarchan:shaping2d:biasform', ...
            'unhandled biasform "%s"', c.biasform);
end
BIAS = BIASDEG/DEG;
X = 0.;

% ---- pre-loop guards (NOT in the book) -----------------------------------
if RTM <= 0
    error('mgs:zarchan:shaping2d:coincident', ...
        'missile and target start at the same point; there is no engagement.');
end
if VC <= 0
    error('mgs:zarchan:shaping2d:opening', ...
        ['closing velocity is %.1f ft/s at t = 0, so "while VC >= 0" exits ' ...
         'immediately and the INITIAL separation (%.0f m) would come back ' ...
         'labelled as a miss distance.  gamIC = %g deg points the missile ' ...
         'away from the target.'], VC, RTM/FT, GAMDEG);
end

% Preallocation and explicit zero-init of the derivative-block outputs.  The
% listing relies on AM1/AM2/XD existing by the time FLAG==1; being explicit
% keeps Code Analyzer quiet and stops a future edit from reading a stale
% workspace variable of the same name.
NMAX = ceil(c.tmax/0.1) + 10;
ArrayT = zeros(1,NMAX);
ArrayRT1K = ArrayT;  ArrayRT2K = ArrayT;
ArrayRM1K = ArrayT;  ArrayRM2K = ArrayT;
ArrayXNCG = ArrayT;  ArrayGAMDEG = ArrayT;
ArrayVMM  = ArrayT;  ArrayXM = ArrayT;   ArrayVC = ArrayT;
AM1 = 0;  AM2 = 0;  XD = 0;  XNC = 0;  GAM = GAMIC;

% Hoisted out of the inner loop on purpose: this branch is taken on EVERY
% derivative evaluation, of which a single book case has on the order of a
% million, and a string comparison there is pure overhead that also makes the
% hot loop read as though the sense were a per-step decision.  It is not -- it
% is fixed for the whole run.
CLAMP_GT = strcmp(c.clampsense, 'gt');

nsteps = 0;  nevals = 0;  nclamp = 0;
clamp_max_rtm = 0;  clamp_first_t = NaN;
xnc_step_absmax = 0;  xnc_raw_absmax = 0;
rtm_min = RTM;  timed_out = false;

% =========================== LISTING 30.1 ====================================
while VC >= 0
    % Terminal step refinement.  rfine is in FEET (it is inside the loop).
    if RTM < c.rfine
        H = c.hfine;
    else
        H = c.h;
    end

    RM1OLD = RM1;  RM2OLD = RM2;  VM1OLD = VM1;  VM2OLD = VM2;
    XOLD = X;  STEP = 1;  FLAG = 0;

    % Two derivative evaluations per step: the first at the old state, the
    % second at the Euler-predicted state.  This is the book's own
    % predictor/trapezoidal-corrector spelling of RK2 (Heun), kept
    % structurally identical rather than refactored into something tidier.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            RM1 = RM1+H*VM1;  RM2 = RM2+H*VM2;
            VM1 = VM1+H*AM1;  VM2 = VM2+H*AM2;
            X = X+H*XD;  T = T+H;
        end
        GAM = atan2(VM2, VM1);
        VM  = sqrt(VM1^2+VM2^2);
        RTM1 = RT1-RM1;  RTM2 = RT2-RM2;  RTM = sqrt(RTM1^2+RTM2^2);
        VTM1 = VT1-VM1;  VTM2 = VT2-VM2;
        VC = -(RTM1*VTM1+RTM2*VTM2)/RTM;
        XLAM = atan2(RTM2, RTM1);
        % LOS rate ANALYTICALLY from the relative kinematics -- never by
        % differencing XLAM.  At H = 1e-5 s a difference would be numerical
        % noise of the same order as the signal, and the resulting command
        % would still look like a guidance command.
        XLAMD = (RTM1*VTM2-RTM2*VTM1)/(RTM*RTM);

        if IGUID == 0
            % OPTIMAL TRAJECTORY SHAPING, stationary-target form (:10887):
            %   nc = 4*Vc*lamdot + 2*Vc*(lam - gamF)/tgo
            % This is Ch.24's law with nT = 0 and lamF = gamF.  It NEEDS tgo.
            % ⚠️ The second term is a 0/0 at intercept -- see the header.
            % TGO is kept as its own named line, exactly as the listing prints
            % it, because tgo is the whole reason this branch is the one WE
            % cannot fly: our own estimate of it is measurably broken
            % ([[predictor-underestimates-dive]], median +18.7 s).
            TGO = RTM/VC;
            XNC = 4.*VC*XLAMD + 2.*VC*(XLAM-GAMF)/TGO;
        else
            % BIASED PROPORTIONAL NAVIGATION (:10918):  gammadot = N*lamdot
            % + bias, with the bias non-zero only on TBEG <= T < TEND.
            % ⚠️ OCR-RESOLVED COMPARISONS 1 and 2 -- '<', both of them.  See
            %    the header: the inverted reading still HITS and only the
            %    terminal flight path angle shows it.
            if T < TBEG
                GAMD = XNP*XLAMD;
            elseif T < TEND
                GAMD = XNP*XLAMD + BIAS;
            else
                GAMD = XNP*XLAMD;
            end
            % nc = VM*gammadot.  Note this is NOT nc = N'*Vc*lamdot: it is the
            % flight-path-rate form, which is why the chapter is careful that
            % N here is not Ch.2's effective ratio N' (:10908).
            XNC = VM*GAMD;
        end

        XNCRAW = XNC;                      % pre-clamp, for the diagnostics only
        % ⚠️ OCR-RESOLVED COMPARISON 3.  '>' is the upper arm of a symmetric
        %    saturation whose lower arm is the next statement.  Inert at the
        %    book's 10 g limit, which is why structuralCheck RUNS the
        %    alternative rather than trusting this comment.
        if CLAMP_GT
            if XNC > XNCLIM,  XNC = XNCLIM;  end
        else
            if XNC < XNCLIM,  XNC = XNCLIM;  end   % the inverted reading
        end
        if XNC < -XNCLIM,  XNC = -XNCLIM;  end

        nevals = nevals + 1;
        if abs(XNCRAW) > xnc_raw_absmax,  xnc_raw_absmax = abs(XNCRAW);  end
        if abs(XNC)    > xnc_step_absmax, xnc_step_absmax = abs(XNC);     end
        if XNC ~= XNCRAW
            nclamp = nclamp + 1;
            if RTM > clamp_max_rtm, clamp_max_rtm = RTM; end
            if isnan(clamp_first_t), clamp_first_t = T; end
        end

        if ICHOICE == 0
            % Command PERPENDICULAR TO THE LINE OF SIGHT.  The along-velocity
            % component is nc*sin(gam-lam), so this branch does work on the
            % missile and |V| is NOT conserved.
            AM1 = -XNC*sin(XLAM);
            AM2 =  XNC*cos(XLAM);
        else
            % Command PERPENDICULAR TO THE VELOCITY VECTOR.  Zero component
            % along V by construction => |V| is invariant to machine
            % precision, and gammadot = nc/VM exactly, which is the identity
            % the whole biased-PN bias derivation rests on.
            AM1 = -XNC*sin(GAM);
            AM2 =  XNC*cos(GAM);
        end
        XD = XNC*XNC;      % integrand of the performance index
        FLAG = 1;
    end
    FLAG = 0;   %#ok<NASGU>  the listing prints this line and it is kept, but
                % it is dead: the next outer iteration sets STEP = 1; FLAG = 0
                % before the inner loop can read it.  Removing it would be a
                % silent departure from zarchan.txt:10955 for no gain.

    RM1 = .5*(RM1OLD+RM1+H*VM1);  RM2 = .5*(RM2OLD+RM2+H*VM2);
    VM1 = .5*(VM1OLD+VM1+H*AM1);  VM2 = .5*(VM2OLD+VM2+H*AM2);
    X   = .5*(XOLD+X+H*XD);
    % X is a trapezoid on a non-negative integrand: X_new = X_old +
    % 0.5*H*(XD1+XD2) >= X_old.  structuralCheck asserts that, because a sign
    % slip in either XD or this corrector would produce a performance index
    % that still looks like a number.

    nsteps = nsteps + 1;
    if RTM < rtm_min, rtm_min = RTM; end

    S = S + H;
    if S >= .09999
        S = 0.;  n = n + 1;
        if n > NMAX
            error('mgs:zarchan:shaping2d:logOverflow', ...
                ['log overflow at t = %.2f s.  Raise tmax, or the run is ' ...
                 'not terminating and the trajectory should not be ' ...
                 'trusted.'], T);
        end
        RT1K = RT1/KM;   RT2K = RT2/KM;
        RM1K = RM1/KM;   RM2K = RM2/KM;
        XNCG = XNC/G;
        % ⚠️ The listing reuses the name GAMDEG for the LOGGED angle here,
        %    clobbering the initial condition.  Harmless in the original ONLY
        %    because BIASDEG is computed before the loop.  A separate name is
        %    used here: move a bias recomputation inside the loop with the
        %    book's spelling and it silently picks up the CURRENT angle in
        %    place of the constant setpoint -- which is exactly the bug that
        %    made additiveLaw an attractor with 32/32 signs reversed
        %    ([[additive-law-was-not-zarchan]]).
        GAMLOG = GAM*DEG;
        VMM = VM/FT;
        XM  = X/(FT*FT);
        ArrayT(n)=T;  ArrayRT1K(n)=RT1K;  ArrayRT2K(n)=RT2K;
        ArrayRM1K(n)=RM1K;  ArrayRM2K(n)=RM2K;
        ArrayXNCG(n)=XNCG;  ArrayGAMDEG(n)=GAMLOG;  ArrayXM(n)=XM;
        % VMM and VC are computed by the listing but never stored: the printed
        % log block has no ArrayVMM line even though the prose (:10930) says
        % missile velocity is written to the file and Fig. 30.5 plots it.
        % Logged here.  That, and ArrayVC (needed for the miss quantum), are
        % the ONLY additions to the book's log block.
        ArrayVMM(n)=VMM;  ArrayVC(n)=VC;
    end

    if T > c.tmax
        timed_out = true;
        break
    end
end
RTM = sqrt(RTM1^2+RTM2^2);      % the book's final display = the miss
VM  = sqrt(VM1^2+VM2^2);        % ...and the terminal speed
% ========================= end of Listing 30.1 ===============================

if n == 0
    error('mgs:zarchan:shaping2d:noLog', ...
        ['the engagement ended at t = %.4f s, before the first 0.1 s log ' ...
         'point, so there is no time history at all and every summary below ' ...
         'would be read off an empty array.'], T);
end

k = 1:n;
r = struct();
r.timed_out = timed_out;
r.cfg = c;

% the book's logged channels, metric
r.T = ArrayT(k);
r.RT1K = ArrayRT1K(k);  r.RT2K = ArrayRT2K(k);
r.RM1K = ArrayRM1K(k);  r.RM2K = ArrayRM2K(k);
r.XNCG = ArrayXNCG(k);  r.GAMDEG = ArrayGAMDEG(k);
r.XM = ArrayXM(k);
% additions (see the logging block)
r.VMM = ArrayVMM(k);    r.VC = ArrayVC(k);

r.bias_degps = BIASDEG;
r.tf_s       = T;
r.nlog       = n;
r.nsteps     = nsteps;
r.nevals     = nevals;

if timed_out
    % ⚠️ NaN, not the separation at tmax.  An engagement that never closed has
    %    no miss distance, and handing back the separation as though it were
    %    one is precisely the plausible-wrong-answer this package refuses to
    %    produce.  r.timed_out is the flag; the caller decides.
    r.miss_m = NaN;  r.gam_f_deg = NaN;  r.x_perf = NaN;  r.vm_f_ms = NaN;
else
    r.miss_m    = RTM/FT;
    r.gam_f_deg = GAM*DEG;
    r.x_perf    = X/(FT*FT);      % performance index, m^2/s^3
    r.vm_f_ms   = VM/FT;
end
r.miss_min_m  = rtm_min/FT;       % closest RTM actually seen; always <= miss_m
r.vm_ic_ms    = c.vmic;           % the EXACT initial speed, not the first log
                                  % sample -- for ICHOICE = 0 the speed has
                                  % already moved by t = 0.1 s.
r.vm_first_log_ms = r.VMM(1);
r.vm_min_ms   = min(r.VMM);
r.vm_max_ms   = max(r.VMM);
r.apogee_km   = max(r.RM2K);
% ⚠️ TWO different peak accelerations, and they are NOT interchangeable.
%    nc_max_g_log is the peak over the book's own 0.1 s log and is what
%    Figs. 30.3/30.7 plot -- the only one comparable with the book.
%    nc_max_g_step is the peak over every derivative evaluation and, for the
%    optimal law, is dominated by the terminal 0/0 and pinned at the limiter.
r.nc_max_g_log   = max(abs(r.XNCG));
r.nc_max_g_step  = xnc_step_absmax/G;
r.nc_raw_absmax_g = xnc_raw_absmax/G;
r.clamp_hits      = nclamp;
r.clamp_frac      = nclamp/max(nevals,1);
r.clamp_max_rtm_ft = clamp_max_rtm;
r.clamp_first_t_s  = clamp_first_t;
% The miss's own error bar.  The target is stationary and the terminal dive
% has gam -> lam, so the closing velocity there IS the missile speed: one
% refined step of closing is hfine*|V|.  A miss below this is zero as far as
% this file knows, and MUST NOT be quoted as a trend.
r.quantum_m = c.hfine*r.vm_f_ms;
if ~isfinite(r.quantum_m) || r.quantum_m <= 0
    r.quantum_m = c.hfine*c.vmic;
end
r.miss_over_quantum = r.miss_m/r.quantum_m;
r.xperf_monotone_slack = min([0, diff(r.XM)]);   % <= 0; must be ~0
end


% =============================================================================
%   THE BOOK'S CASES
% =============================================================================
function C = bookCases(c0)
%BOOKCASES  Every engagement the chapter publishes, plus the four delete
%   experiments and the corner the book never runs.
%
%   ALL OF THEM GO THROUGH flyOne(), NEVER THROUGH THE PUBLIC ENTRY.  The
%   public entry runs the benchmark, so calling it from here would recurse.
%   engage2d can afford to call its own entry point because its benchmark is
%   behind a flag that defaults FALSE; ours defaults TRUE.

base = c0;
base.biasform = 'listing';       % the benchmark only runs when biasform is
                                 % already 'listing' (it is gated), so this is
                                 % belt-and-braces, not a silent override.
base.clampsense = 'gt';

C = struct();

% ---- Figs. 30.2-30.5: OPTIMAL shaping, the two command orientations -------
C.opt_los = flyOne(gridCorner(base, 0, 0));    % ICHOICE = 0, the book's nominal
C.opt_v   = flyOne(gridCorner(base, 0, 1));    % ICHOICE = 1

% ---- Figs. 30.6-30.9: BIASED PN vs optimal, all _|_ velocity --------------
%      "In future studies of trajectory shaping in this section we shall
%       assume that the acceleration command is perpendicular to the velocity
%       vector (ICHOICE = 1)" -- zarchan.txt:10980.
C.bpn_v0 = flyOne(gridCorner(base, 1, 1));                      % TBEG = 0
c10 = gridCorner(base, 1, 1);  c10.tbeg = 10;
C.bpn_v10 = flyOne(c10);                                    % TBEG = 10 s

% ---- the fourth 2x2 corner, which the book NEVER runs ---------------------
%      Reported, not scored.  See the INFO row for why it cannot work.
C.bpn_los = flyOne(gridCorner(base, 1, 0));

% ---- Figs. 30.10/30.11: the gamF family --------------------------------
%      Cases taken from the Fig. 30.10 legend, which survived extraction as
%      plain text at zarchan.txt:11135-11139:
%          "gamF=-45 deg / -60 / -75 / -90", "gamIC=30 deg, dt=30 s, Biased PN"
%      Those labels DEFINE the cases; no benchmark value is read off the
%      figure itself.
C.sweep_gamfdeg = [-45 -60 -75 -90];
C.sweep = struct('gamfdeg', {}, 'r', {});
for i = 1:numel(C.sweep_gamfdeg)
    gf = C.sweep_gamfdeg(i);
    if gf == base.gamfdeg && base.tbeg == 0 && base.delt == 30
        rr = C.bpn_v0;                 % identical run; do not fly it twice
    else
        cs = gridCorner(base, 1, 1);
        cs.gamfdeg = gf;  cs.tbeg = 0;  cs.delt = 30;
        rr = flyOne(cs);
    end
    C.sweep(i).gamfdeg = gf;
    C.sweep(i).r = rr;
end

% ---- DELETE EXPERIMENTS.  Each one RUNS the alternative reading. ----------
% D1  the prose form of the bias (the erratum).
cp = gridCorner(base, 1, 1);  cp.biasform = 'text';
C.bias_prose = flyOne(cp);

% D2  the bias never applied.  At TBEG = 0 this is EXACTLY what the inverted
%     reading of "if T,TBEG" produces: T > TBEG is false only at T == 0 and
%     true for every subsequent evaluation, so the first branch (plain PN)
%     swallows the whole flight and the bias is never reached.  Running the
%     unbiased case therefore tests the inverted reading without carrying a
%     deliberately wrong comparison in flyOne.
cn = gridCorner(base, 1, 1);  cn.biasform = 'none';
C.bias_none = flyOne(cn);

% D3  the inverted reading of "if XNC.XNCLIM".  There is no equivalent trick
%     for this one, so cfg.clampsense actually runs it.
ci = gridCorner(base, 1, 1);  ci.clampsense = 'lt';
C.clamp_inverted = flyOne(ci);

% D4  the correct reading with a limit low enough to be ACTIVE, so that
%     "fires and releases" is measured rather than assumed.  Under the
%     inverted reading this fraction would be 1.000 by construction.
c2 = gridCorner(base, 1, 1);  c2.xnclimg = 2;
C.clamp_active = flyOne(c2);
end


function [r, reused] = pickRun(C, c)
%PICKRUN  The caller's single engagement, taken from the board when the board
%   already flew exactly it.
%
%   bookCases builds every case from the caller's own cfg and overrides only
%   iguid/ichoice (and, on the delete experiments, biasform / clampsense /
%   tbeg / xnclimg).  clampsense is not a public option and is always 'gt'
%   here, so the ONLY thing that can make the caller's request differ from a
%   corner is a non-'listing' biasform.
%
%   ⚠️ The condition is written as an explicit whitelist rather than "the
%      corners cover everything".  If a future edit gives bookCases a
%      different base -- book values instead of the caller's, say -- this
%      function would start handing back a case that is NOT the requested run
%      while every field name still read correctly.  Keeping the guard
%      explicit means such an edit has to touch this line too.
reused = false;
if strcmp(c.biasform, 'listing') && strcmp(c.clampsense, 'gt')
    reused = true;
    if     c.iguid == 0 && c.ichoice == 0, r = C.opt_los;  return
    elseif c.iguid == 0 && c.ichoice == 1, r = C.opt_v;    return
    elseif c.iguid == 1 && c.ichoice == 1, r = C.bpn_v0;   return
    elseif c.iguid == 1 && c.ichoice == 0, r = C.bpn_los;  return
    end
    reused = false;   % unreachable while the iguid/ichoice guards hold
end
r = flyOne(c);
end


function c = gridCorner(c, iguid, ichoice)
%GRIDCORNER  One cell of the 2x2.  Named rather than inlined so that a case list
%   reads as a grid and a missing corner is visible.
c.iguid = iguid;
c.ichoice = ichoice;
end


% =============================================================================
%   STRUCTURAL SELF-CHECK.  WIRING AND OCR RESOLUTION, NOT BOOK VALUES.
% =============================================================================
function K = structuralCheck(C, c)
%STRUCTURALCHECK  Invariants that must hold for the benchmark rows above them
%   to mean anything.  A failure here voids the board.
K = struct();
K.skipped = false;

% ---- 1. the bias identity, pure algebra, no simulation --------------------
%   bias*dt must equal (gamF - gamIC) - N*(gamF - lamIC), which is the book's
%   own integrated law at :10922 with lamF = gamF.  lamIC = 0 for the nominal
%   geometry (target dead ahead at the launch altitude).
DEG = 57.3;
lamIC_deg = atan2(c.rt2m, c.rangem)*DEG;
want = ((c.gamfdeg - c.gamdeg) - c.xnp*(c.gamfdeg - lamIC_deg))/c.delt;
K.bias_degps       = C.bpn_v0.bias_degps;
K.bias_want_degps  = want;
K.bias_resid_degps = abs(K.bias_degps - want);
% RELATIVE, not a bare 1e-9.  Both sides divide by DELT, so a caller running a
% short bias window scales the bias up without changing the identity -- an
% absolute floor would turn that into a spurious structural FAILURE, which is
% the worst kind because a structural failure voids the whole board above it.
K.bias_ok          = K.bias_resid_degps <= 1e-9*max(1, abs(want));

% ---- 2. the unbiased-PN terminal angle, also closed form ------------------
%   With the bias off, integrating gammadot = N*lamdot from gamIC to gamF for
%   a stationary target (lamF = gamF) gives gamF = gamIC/(1-N).  For the
%   nominal that is 30/(1-3) = -15 deg exactly.  This is the row that
%   discriminates OCR comparisons 1 and 2: the inverted reading lands here
%   instead of at -90, WHILE STILL HITTING THE TARGET.
K.nobias_gamf_deg  = C.bias_none.gam_f_deg;
K.nobias_pred_deg  = c.gamdeg/(1 - c.xnp);
K.nobias_resid_deg = abs(K.nobias_gamf_deg - K.nobias_pred_deg);
K.nobias_miss_m    = C.bias_none.miss_m;
K.nobias_ok        = K.nobias_resid_deg < 0.01;
% and the point of the row: the miss distance is USELESS as a discriminator.
K.nobias_still_hits = C.bias_none.miss_m < 1;

% ---- 3. the bias-sign erratum, run rather than asserted -------------------
K.prose_bias_degps = C.bias_prose.bias_degps;
K.prose_miss_m     = C.bias_prose.miss_m;
K.prose_gamf_deg   = C.bias_prose.gam_f_deg;
K.prose_ok         = C.bias_prose.miss_m > 1000 && ...
                     sign(C.bias_prose.bias_degps) ~= sign(C.bpn_v0.bias_degps);

% ---- 4. the clamp is inert at the book's own limit ------------------------
%   "Inert" is stated as a BOUND, not as zero: the optimal law touches the
%   limiter on its final one or two derivative evaluations, where tgo -> 0.
%   Every clamped evaluation must be inside the last foot of the engagement.
rs = {C.opt_los, C.opt_v, C.bpn_v0, C.bpn_v10};
K.clamp_frac_max   = max(cellfun(@(x) x.clamp_frac, rs));
K.clamp_rtm_max_ft = max(cellfun(@(x) x.clamp_max_rtm_ft, rs));
K.clamp_peak_log_g = max(cellfun(@(x) x.nc_max_g_log, rs));
K.clamp_inert_ok   = K.clamp_frac_max < 1e-5 && K.clamp_rtm_max_ft < 1 && ...
                     K.clamp_peak_log_g < c.xnclimg;

% ---- 5. DELETE EXPERIMENT: the inverted clamp reading ---------------------
K.invclamp_frac   = C.clamp_inverted.clamp_frac;
K.invclamp_miss_m = C.clamp_inverted.miss_m;
K.invclamp_ok     = K.invclamp_frac > 0.99 && C.clamp_inverted.miss_m > 1000;

% ---- 6. DELETE EXPERIMENT: the correct clamp fires AND RELEASES -----------
%   Under the inverted reading this fraction is 1.000 by construction, so a
%   value strictly inside (0,1) is what tells the two apart on a case where
%   the clamp is actually doing something.
K.clamp2_frac   = C.clamp_active.clamp_frac;
K.clamp2_miss_m = C.clamp_active.miss_m;
K.clamp2_gamf   = C.clamp_active.gam_f_deg;
K.clamp2_ok     = K.clamp2_frac > 0.05 && K.clamp2_frac < 0.95 && ...
                  C.clamp_active.miss_m < 1;

% ---- 7. the ICHOICE branches are the right way round ----------------------
%   A force perpendicular to the velocity does no work, so ICHOICE = 1 must
%   conserve |V| to machine precision and ICHOICE = 0 must not.  If the two
%   branches were swapped this row flips and NOTHING ELSE in the file would
%   notice -- both branches fly, both hit, both reach -90 deg.
K.perpv_speed_rel  = abs(C.opt_v.vm_f_ms/C.opt_v.vm_ic_ms - 1);
K.perplos_loss_rel = (C.opt_los.vm_ic_ms - C.opt_los.vm_f_ms)/C.opt_los.vm_ic_ms;
K.ichoice_ok       = K.perpv_speed_rel < 1e-8 && K.perplos_loss_rel > 0.10;

% ---- 8. the performance index never decreases ----------------------------
K.xperf_slack = min(cellfun(@(x) x.xperf_monotone_slack, ...
                    [rs, {C.bpn_los, C.bias_prose, C.bias_none}]));
K.xperf_ok    = K.xperf_slack >= -1e-9;

% ---- 9. the reported miss really is numerical, not physical --------------
%   Not "the miss is small" -- "the miss is at the integrator's own floor".
%   Every book case must land within a couple of terminal steps of zero.
%   ⚠️ This is the OPPOSITE test to the one that invalidated
%      scripts/research/ditcg/sim3d.py: there the miss halved with dt while
%      being far ABOVE the quantum, i.e. it was quantisation residue
%      masquerading as physics.  Here it must BE the quantum.
mq = cellfun(@(x) x.miss_over_quantum, [rs, {C.sweep(1).r, C.sweep(2).r, C.sweep(3).r}]);
K.miss_over_quantum_max = max(mq);
K.quantum_ok = K.miss_over_quantum_max < 2;

% ---- 10. nothing timed out ------------------------------------------------
allr = [rs, {C.bpn_los, C.bias_prose, C.bias_none, C.clamp_inverted, C.clamp_active}];
K.timed_out_any = any(cellfun(@(x) x.timed_out, allr));
K.closure_ok    = ~K.timed_out_any;

names = {'bias identity', 'unbiased-PN terminal angle', 'bias-sign erratum', ...
         'clamp inert at the book limit', 'DELETE: inverted clamp reading', ...
         'DELETE: clamp fires and releases', 'ICHOICE branches not swapped', ...
         'performance index monotone', 'miss is at the integrator floor', ...
         'every engagement closed'};
oks   = [K.bias_ok, K.nobias_ok, K.prose_ok, K.clamp_inert_ok, K.invclamp_ok, ...
         K.clamp2_ok, K.ichoice_ok, K.xperf_ok, K.quantum_ok, K.closure_ok];

% ⚠️⚠️ 這兩行以前不存在,而 names/oks 只是**區域變數**。
%   benchSuite 的 adaptGeneric 需要 .names/.oks 才會把結構檢查放上板;
%   拿不到就(當時)靜默 return ⇒ 上面這十條 —— 含兩個刪除實驗 ——
%   **一條都沒上過板**,而板面全綠。跑了刪除實驗卻沒有人看得到結果,
%   等於沒跑。adaptGeneric 那一側已改成大聲報錯,這裡把資料補上。
K.names = names;
K.oks   = logical(oks);

K.ok = all(oks);
if K.ok
    K.first_fail = '';
else
    K.first_fail = names{find(~oks, 1)};
end
end


% =============================================================================
%   THE BOOK BENCHMARKS
% =============================================================================
function [B, nPreamble] = benchmarks(C, K, c, offbook, discOffbook)
%BENCHMARKS  Computed vs published, with a band, a reason for the band, and
%   the zarchan.txt line each claim came from.
B = emptyBench();

% Gates.  A row is SKIPPED, with the reason printed, when the caller moved a
% parameter the book value depends on.  It is never silently re-scored.
MODEL = offbook;
gM  = gateWhy(MODEL, {'xnp','gamdeg','gamfdeg','delt','tbeg','xnclimg', ...
                      'vmic','rangem','rt2m','biasform'});
gT  = gateWhy(MODEL, {'xnp','gamdeg','gamfdeg','delt','xnclimg', ...
                      'vmic','rangem','rt2m','biasform'});   % tbeg is set by the case
gS  = gateWhy(MODEL, {'xnp','gamdeg','delt','tbeg','xnclimg', ...
                      'vmic','rangem','rt2m','biasform'});   % gamfdeg is swept
% gQ gates ONLY on the terminal step, and only the miss rows use it.  The
% model gating for those rows is already done by whichever of gM/gT/gS encloses
% them, so folding the model list in here as well would make a swept parameter
% (gamfdeg, which the sweep sets itself) skip its own miss row.
gQ  = gateWhy(discOffbook, {'hfine'});

SRC_PROSE = 'p.780 prose (zarchan.txt:10930)';
SRC_LIST  = 'Listing 30.1 data statement (zarchan.txt:10934)';
SRC_DERIV = 'bias derivation (zarchan.txt:10918-10928)';
SRC_F25   = 'Figs. 30.2-30.5 discussion (zarchan.txt:10961)';
SRC_F5    = 'Fig. 30.5 discussion (zarchan.txt:10980)';
SRC_F4CAP = 'Fig. 30.4 caption (zarchan.txt:11014)';
SRC_F5CAP = 'Fig. 30.5 caption (zarchan.txt:11037)';
SRC_F69   = 'Figs. 30.6-30.9 discussion (zarchan.txt:11020)';
SRC_F9CAP = 'Fig. 30.9 caption (zarchan.txt:11123)';
SRC_33    = 'performance index comparison (zarchan.txt:11058)';
SRC_SUM   = 'Ch.30 section summary (zarchan.txt:11060)';
SRC_F1011 = 'Figs. 30.10/30.11 discussion (zarchan.txt:11059)';
SRC_F11CAP= 'Fig. 30.11 caption (zarchan.txt:11171)';
SRC_F10LEG= 'Fig. 30.10 legend (zarchan.txt:11135-11139)';

A = C.opt_los;  Bv = C.opt_v;  P0 = C.bpn_v0;  P10 = C.bpn_v10;

% -------------------------------------------------------------------------
% A. CONFIGURATION ECHO.  Cheap, and they lock the transcription of the data
%    statement against the sentence of prose that describes it.  If :10934 had
%    been mis-transcribed, every magnitude row below would move too and there
%    would be nothing to say which had happened.
% -------------------------------------------------------------------------
B = addVal(B, 'downrange to target', 10000, c.rangem, 'exact', SRC_PROSE, 'm', ...
    '"a stationary target 10 km downrange".  Listing writes RT1IC = 10000.*3.28');
B = addVal(B, 'initial missile speed', 250, c.vmic, 'exact', SRC_PROSE, 'm/s', ...
    '"the initial missile velocity is 250 m/s".  Listing writes VM = 250.*3.28');
B = addVal(B, 'desired terminal gamma', -90, c.gamfdeg, 'exact', SRC_PROSE, 'deg', ...
    '"a final flight path angle of -90 deg"');
B = addVal(B, 'initial gamma (heading error)', 30, c.gamdeg, 'exact', SRC_PROSE, 'deg', ...
    '"launched with a 30-deg heading error (or initial flight path angle of 30 deg)"');
B = addVal(B, 'bias turn-on TBEG (+1 s offset)', 1, c.tbeg + 1, 'exact', ...
    SRC_PROSE, 's+1', ...
    ['"the bias is turned on at time zero (TBEG = 0)".  Scored as TBEG+1 ' ...
     'against 1 because a RELATIVE band cannot be formed against a book value ' ...
     'of zero -- addVal skips those rows rather than dividing by zero, and a ' ...
     'SKIPPED echo row would leave the transcription of TBEG unlocked.  The ' ...
     '+1 s is applied to BOTH sides and to nothing else in the file']);
B = addVal(B, 'bias duration DELT', 30, c.delt, 'exact', SRC_PROSE, 's', ...
    '"turned off 30 s later (DELT = 30)"');
B = addVal(B, 'navigation ratio N', 3, c.xnp, 'exact', SRC_LIST, '-', ...
    ['XNP = 3.  Listing literal; the prose names no value.  ⚠️ This N is the ' ...
     'FLIGHT-PATH-RATE ratio (gammadot = N*lamdot), not Ch.2''s effective ' ...
     'N''; they coincide here only because Vc ~ VM against a stationary ' ...
     'target (zarchan.txt:10908 and :10917)']);
B = addVal(B, 'command limit XNCLIMG', 10, c.xnclimg, 'exact', SRC_LIST, 'g', ...
    'XNCLIMG = 10.  Listing literal; inert at the nominal, see the structural block');
B = addShape(B, 'target is stationary', 'VT1 = VT2 = 0', ...
    sprintf('RT2IC = %g m', c.rt2m), c.rt2m == 0, SRC_LIST, ...
    ['the listing hard-codes RT2IC = 0 and has no target-motion block at ' ...
     'all -- Listing 24.2''s BETAD = XNT/VT line is absent precisely because ' ...
     'it divides by zero for a stationary target (zarchan.txt:8338)']);

% -------------------------------------------------------------------------
% B. THE BIAS, WHICH IS THE ONE PIECE OF ALGEBRA THE CHAPTER ADDS
% -------------------------------------------------------------------------
B = addBound(B, 'bias satisfies the book''s integrated law', ...
    'bias*dt == (gamF-gamIC) - N*(gamF-lamIC)', ...
    sprintf('%+.4f deg/s, resid %.2e', K.bias_degps, K.bias_resid_degps), ...
    K.bias_ok, SRC_DERIV, 'deg/s', ...
    ['the LISTING form, not the prose form.  The prose at :10928 prints ' ...
     '+gamF*(N-1) where the derivation two lines above it requires ' ...
     '-gamF*(N-1); on the nominal that is +5 vs -7 deg/s, a SIGN and not a ' ...
     'rounding.  The listing wins and the erratum is run in the structural block']);

% ⚠️ EVERYTHING ABOVE THIS LINE IS A CONFIGURATION ECHO OR A PIECE OF ALGEBRA.
%    None of it involves a trajectory, so none of it can fail because the
%    SIMULATION is wrong -- only because the transcription of the data
%    statement is.  nPreamble is handed back so the caller can require that at
%    least one RESULT row was evaluated before calling the port validated: a
%    run that moved a model parameter skips every block below and would
%    otherwise report "10/10 PASS" while having checked nothing but its own
%    arguments.  That is the exact shape of green light this repo has been
%    burned by ([[promised-guards-that-do-not-exist]]).
nPreamble = numel(B);

% -------------------------------------------------------------------------
% C. FIGS. 30.2-30.5.  OPTIMAL SHAPING, _|_LOS vs _|_VELOCITY.
% -------------------------------------------------------------------------
if ~isempty(gM)
    B = addSkip(B, 'Figs 30.2-30.5 block', 'the two ICHOICE cases', gM, SRC_F25, '-');
else
    if ~isempty(gQ)
        B = addSkip(B, 'both ICHOICE cases hit the target', 'miss < 0.01 m each', ...
            gQ, SRC_F25, 'm');
    else
        B = addBound(B, 'both ICHOICE cases hit the target', 'miss < 0.01 m each', ...
            sprintf('%.5f / %.5f m', A.miss_m, Bv.miss_m), ...
            A.miss_m < 0.01 && Bv.miss_m < 0.01, SRC_F25, 'm', ...
            ['"in both cases ... the missile hits the stationary target near ' ...
             'vertically".  0.01 m is roughly four terminal steps of closing; ' ...
             'the structural block scores the tighter statement, that the miss ' ...
             'IS the quantum.  Gated on hfine because hfine sets that floor, ' ...
             'i.e. the criterion and not the answer']);
    end
    B = addVal(B, 'terminal gamma, _|_LOS', -90, A.gam_f_deg, 'text', ...
        [SRC_F25 ' + ' SRC_F4CAP], 'deg', ...
        '"both guidance command implementations result in the flight path angle approaching -90 deg at intercept"');
    B = addVal(B, 'terminal gamma, _|_VM', -90, Bv.gam_f_deg, 'text', ...
        [SRC_F25 ' + ' SRC_F4CAP], 'deg', 'the same sentence, other branch');
    B = addShape(B, 'peak |n_c| smaller for _|_LOS', ...
        'max|n_c|(_|_LOS) < max|n_c|(_|_VM)', ...
        sprintf('%.3f < %.3f g', A.nc_max_g_log, Bv.nc_max_g_log), ...
        A.nc_max_g_log < Bv.nc_max_g_log, SRC_F25, ...
        ['"the maximum value of the acceleration command that is ' ...
         'perpendicular to the line of sight is much smaller".  Scored on the ' ...
         'peak over the book''s 0.1 s LOG, because that is what Fig. 30.3 ' ...
         'plots -- its printed ordinate runs -3..1 g, which matches the logged ' ...
         'peaks and not the per-step ones.  "much" is an adjective and is not scored']);
    B = addShape(B, 'flight time longer for _|_LOS', 'tf(_|_LOS) > tf(_|_VM)', ...
        sprintf('%.2f > %.2f s', A.tf_s, Bv.tf_s), A.tf_s > Bv.tf_s, SRC_F25, ...
        ['"and the flight time is much longer".  The subject of that clause is ' ...
         'the _|_LOS case, and the physics agrees: it arrives slower, so it ' ...
         'takes longer over a comparable path']);
    B = addBound(B, '_|_VM loses NO speed', '|V_f/V_0 - 1| < 1e-8', ...
        sprintf('%.3e', K.perpv_speed_rel), K.perpv_speed_rel < 1e-8, SRC_F5, '-', ...
        ['"no velocity loss at all when the missile acceleration command is ' ...
         'perpendicular to the velocity vector".  This is EXACT, not ' ...
         'approximate: a force perpendicular to V does no work.  It is the ' ...
         'sharpest check in this file and the one that proves the ICHOICE ' ...
         'branches are not swapped']);
    B = addBound(B, '_|_LOS loses speed severely', 'V_f/V_0 loss > 50%', ...
        sprintf('%.1f%% (%.1f -> %.1f m/s)', 100*K.perplos_loss_rel, ...
                A.vm_ic_ms, A.vm_f_ms), K.perplos_loss_rel > 0.50, SRC_F5CAP, '-', ...
        ['"causes missile velocity to decrease significantly" / "a severe ' ...
         'missile velocity loss".  ⚠️ THE 50% THRESHOLD IS OURS -- the book ' ...
         'quotes no number and Fig. 30.5''s curve did not survive extraction. ' ...
         'What is book-supported is the DIRECTION and the contrast with the ' ...
         'row above; the magnitude is printed, not adjudicated against a value']);
    dApo = abs(A.apogee_km - Bv.apogee_km)/A.apogee_km;
    B = addShape(B, 'trajectories "nearly identical" (apogee)', ...
        'apogee disagreement < 10%', ...
        sprintf('%.2f%% (%.4f vs %.4f km)', 100*dApo, A.apogee_km, Bv.apogee_km), ...
        dApo < 0.10, SRC_F25, ...
        ['⚠️ THE 10% BAND IS OURS.  "nearly identical" is an adjective; this ' ...
         'is the substantive part of it that can be scored.  The sharper ' ...
         'metric is the INFO row below and it does NOT meet 10%.  DO NOT ' ...
         'relax this band to make that one green']);
    sep = maxAltSeparation(A, Bv);
    B = addInfo(B, 'trajectory separation at equal downrange', ...
        '"nearly identical" (adjective)', ...
        sprintf('max %.1f m = %.1f%% of apogee', 1000*sep, 100*sep/A.apogee_km), ...
        SRC_F25, ...
        ['NOT SCORED.  The claim is an adjective, and altitude-vs-downrange ' ...
         'is ill-conditioned in the terminal plunge where downrange is nearly ' ...
         'stationary while altitude collapses.  Reported because the earlier ' ...
         'audit of this same listing (matlab/zarchan/README.md) found ~16% ' ...
         'here against a pre-registered 10% and left it failing rather than ' ...
         'widening the bound.  Same finding, same refusal']);
    B = addInfo(B, '"much smaller" / "much longer", quantified', ...
        'adjectives (:10961)', ...
        sprintf('n_c ratio %.2fx, tf ratio %.2fx', ...
                Bv.nc_max_g_log/A.nc_max_g_log, A.tf_s/Bv.tf_s), SRC_F25, ...
        'the inequalities are scored above; the magnitudes are printed only');
    B = addInfo(B, 'terminal n_c of the OPTIMAL law diverges', ...
        'not published (0/0 at intercept)', ...
        sprintf('raw peak %.3g g, clamped on %d of %d evals, all at RTM < %.3g ft', ...
                max(A.nc_raw_absmax_g, Bv.nc_raw_absmax_g), ...
                A.clamp_hits + Bv.clamp_hits, A.nevals + Bv.nevals, ...
                max(A.clamp_max_rtm_ft, Bv.clamp_max_rtm_ft)), SRC_F25, ...
        ['NOT SCORED, deliberately.  2*Vc*(lam-gamF)/tgo has numerator and ' ...
         'denominator both going to zero at intercept; one of the clamped ' ...
         'evaluations happens after VC has already changed sign, i.e. past ' ...
         'closest approach, where tgo is negative.  A number read off a ' ...
         'diverging curve is not a benchmark.  It is invisible on the 0.1 s ' ...
         'log and therefore on Fig. 30.3 -- which is why the peak rows above ' ...
         'are scored on the logged array']);
end

% -------------------------------------------------------------------------
% D. FIGS. 30.6-30.9.  BIASED PN vs OPTIMAL, BOTH _|_VELOCITY.
% -------------------------------------------------------------------------
if ~isempty(gT)
    B = addSkip(B, 'Figs 30.6-30.9 block', 'optimal vs biased PN', gT, SRC_F69, '-');
else
    if ~isempty(gQ)
        B = addSkip(B, 'all three cases hit the target', 'miss < 0.01 m each', ...
            gQ, SRC_F69, 'm');
    else
        B = addBound(B, 'all three cases hit the target', 'miss < 0.01 m each', ...
            sprintf('%.5f / %.5f / %.5f m', Bv.miss_m, P0.miss_m, P10.miss_m), ...
            Bv.miss_m < 0.01 && P0.miss_m < 0.01 && P10.miss_m < 0.01, SRC_F69, 'm', ...
            '"both guidance laws result in missile trajectories that hit the stationary target near vertically"');
    end
    B = addVal(B, 'terminal gamma, biased PN TBEG=0', -90, P0.gam_f_deg, 'text', ...
        SRC_F69, 'deg', ...
        '"Figure 30.8 shows that both guidance laws achieve success in that the flight path angle approaches -90 deg"');
    B = addVal(B, 'terminal gamma, biased PN TBEG=10', -90, P10.gam_f_deg, 'text', ...
        SRC_F69, 'deg', 'the same sentence, the bias-starts-at-10-s case');
    B = addShape(B, 'optimal trajectory is tighter than biased PN', ...
        'apogee(optimal) < apogee(BPN TBEG=0)', ...
        sprintf('%.4f < %.4f km', Bv.apogee_km, P0.apogee_km), ...
        Bv.apogee_km < P0.apogee_km, SRC_F69, ...
        '"However the optimal guidance law trajectory is much tighter"');
    B = addShape(B, 'starting the bias later tightens it', ...
        'apogee(TBEG=10) < apogee(TBEG=0)', ...
        sprintf('%.4f < %.4f km', P10.apogee_km, P0.apogee_km), ...
        P10.apogee_km < P0.apogee_km, SRC_F69, ...
        '"starting the bias later at 10 s also tightens the trajectory of the missile using biased proportional navigation"');
    B = addShape(B, 'tighter trajectories fly shorter', ...
        'tf(optimal) and tf(TBEG=10) both < tf(TBEG=0)', ...
        sprintf('%.2f, %.2f < %.2f s', Bv.tf_s, P10.tf_s, P0.tf_s), ...
        Bv.tf_s < P0.tf_s && P10.tf_s < P0.tf_s, SRC_F69, ...
        '"the flight time is much shorter for the tighter trajectories (optimal guidance case and biased proportional navigation when bias starts at 10 s)"');
    B = addShape(B, 'optimal minimises the performance index', ...
        'X(optimal) is the smallest of the three', ...
        sprintf('%.1f < %.1f, %.1f m^2/s^3', Bv.x_perf, P0.x_perf, P10.x_perf), ...
        Bv.x_perf < P0.x_perf && Bv.x_perf < P10.x_perf, [SRC_F9CAP ' + ' SRC_SUM], ...
        ['Fig. 30.9 caption "Optimal guidance minimizes performance index", ' ...
         'and :11060 "Optimal guidance does require a smaller performance ' ...
         'index ... than biased proportional navigation".  Exact ordering, no band']);
    excess = 100*(P0.x_perf/Bv.x_perf - 1);
    B = addVal(B, 'biased PN (TBEG=0) index excess', 33, excess, 'graph', SRC_33, '%', ...
        ['"the performance index for biased proportional navigation starting ' ...
         'at time zero is only 33% larger".  The 33 is READ OFF Fig. 30.9, ' ...
         'whose printed ordinate ticks run 0..12000 every 2000: place the ' ...
         'optimal curve at ~9000 and the biased-PN curve on its 12000 tick and ' ...
         'you get exactly 33.3%.  Hence the graph band.  The ' ...
         'ORDERING it depends on is scored exactly in the row above']);
    B = addInfo(B, '"the maximum accelerators are similar"', ...
        'adjective (:11020)', ...
        sprintf('%.3f / %.3f / %.3f g (spread %.2fx)', Bv.nc_max_g_log, ...
                P0.nc_max_g_log, P10.nc_max_g_log, ...
                max([Bv.nc_max_g_log P0.nc_max_g_log P10.nc_max_g_log])/ ...
                min([Bv.nc_max_g_log P0.nc_max_g_log P10.nc_max_g_log])), ...
        SRC_F69, ...
        ['NOT SCORED.  "similar" is an adjective and the three peaks here ' ...
         'span most of a factor of two, so any band would be an invented ' ...
         'criterion dressed as a book value.  Fig. 30.7''s printed ordinate ' ...
         'runs -5..1 g, which is consistent with the largest of the three']);
    B = addInfo(B, 'biased PN TBEG=10 vs optimal apogee', ...
        'the book claims no ordering', ...
        sprintf('%.4f vs %.4f km', P10.apogee_km, Bv.apogee_km), SRC_F69, ...
        ['NOT SCORED.  The book orders optimal against BPN(TBEG=0) and ' ...
         'BPN(TBEG=10) against BPN(TBEG=0), never optimal against ' ...
         'BPN(TBEG=10) -- and on this transcription the later-bias case comes ' ...
         'out marginally the tighter of those two.  Fig. 30.6''s data did not ' ...
         'survive extraction, so nothing can settle it']);
end

% -------------------------------------------------------------------------
% E. FIGS. 30.10/30.11.  THE gamF FAMILY.
% -------------------------------------------------------------------------
if ~isempty(gS)
    B = addSkip(B, 'Figs 30.10/30.11 block', 'gamF = -45/-60/-75/-90', gS, SRC_F1011, '-');
else
    sw = C.sweep;
    miss = arrayfun(@(s) s.r.miss_m, sw);
    gerr = arrayfun(@(s) abs(s.r.gam_f_deg - s.gamfdeg), sw);
    xs   = arrayfun(@(s) s.r.x_perf, sw);
    ncs  = arrayfun(@(s) s.r.nc_max_g_log, sw);
    if ~isempty(gQ)
        B = addSkip(B, 'biased PN hits at every gamF', 'miss < 0.01 m, all four', ...
            gQ, SRC_F1011, 'm');
    else
        B = addBound(B, 'biased PN hits at every gamF', 'miss < 0.01 m, all four', ...
            sprintf('max %.5f m', max(miss)), max(miss) < 0.01, SRC_F1011, 'm', ...
            ['"biased proportional navigation hits the stationary target for a ' ...
             'variety of final impact angles".  Cases from the Fig. 30.10 legend, ' ...
             'which survived extraction as text (' SRC_F10LEG ')']);
    end
    B = addBound(B, 'terminal gamma matches the demand', ...
        '|gam_f - gamF| < 0.05 deg, all four', ...
        sprintf('max %.4f deg', max(gerr)), max(gerr) < 0.05, SRC_F1011, 'deg', ...
        ['the strongest quantitative statement the chapter makes: the whole ' ...
         'point of the bias is to land on a CHOSEN terminal angle without ' ...
         'time-to-go, and here it does so to under a thousandth of a degree ' ...
         'at four different demands']);
    B = addShape(B, 'acceleration cost rises with steeper gamF', ...
        'X strictly increasing over gamF = -45,-60,-75,-90', ...
        sprintf('%.0f < %.0f < %.0f < %.0f m^2/s^3', xs(1), xs(2), xs(3), xs(4)), ...
        all(diff(xs) > 0), SRC_F11CAP, ...
        ['Fig. 30.11 caption "Acceleration requirements increase as final ' ...
         'flight path angle increases for biased PN".  "Acceleration ' ...
         'requirements" is read as the book''s OWN performance index -- the ' ...
         'integral of commanded acceleration squared, which is the only ' ...
         'acceleration cost Ch.30 defines (:10930).  On that reading it is ' ...
         'exactly monotone.  The peak-command reading is the INFO row below']);
    B = addInfo(B, 'peak |n_c| across the gamF family', ...
        'which measure Fig. 30.11 means is not stated', ...
        sprintf('%.3f / %.3f / %.3f / %.3f g', ncs(1), ncs(2), ncs(3), ncs(4)), ...
        SRC_F11CAP, ...
        ['NOT SCORED.  Read as PEAK COMMAND the caption holds on the first ' ...
         'two steps and inverts on the last; read as the performance index it ' ...
         'is exactly monotone (scored above).  The inversion is a few percent ' ...
         '-- under half a minor division of Fig. 30.11''s printed ordinate ' ...
         '(-5..1 g in 1 g steps) -- so the figure cannot settle which reading ' ...
         'the caption meant, and inventing a band would be picking one']);
end

% -------------------------------------------------------------------------
% F. THE FOURTH 2x2 CORNER, WHICH THE BOOK NEVER RUNS.
% -------------------------------------------------------------------------
if isempty(gM)
    L = C.bpn_los;
    B = addInfo(B, 'biased PN with the command _|_ LOS', ...
        'the book runs no such case', ...
        sprintf('miss %.0f m, gam_f %+.1f deg, V %.0f -> %.0f m/s', ...
                L.miss_m, L.gam_f_deg, L.vm_ic_ms, L.vm_f_ms), SRC_F5, ...
        ['NOT SCORED -- there is nothing to score it against, and it is not a ' ...
         'port defect.  The bias VALUE is derived from gammadot = n_c/VM, ' ...
         'which holds ONLY when the command is perpendicular to the velocity. ' ...
         'With ICHOICE = 0 the achieved rate is n_c*cos(gam-lam)/VM and the ' ...
         'speed changes as well, so the integral gamF-gamIC = N*(lamF-lamIC) ' ...
         '+ bias*dt that fixes the bias is simply not the equation being ' ...
         'flown.  The book narrows to ICHOICE = 1 for every biased-PN study ' ...
         'at :10980, and this corner is why.  ⚠️ FOR US: it is the same class ' ...
         'of error as computing an authority budget for one channel and ' ...
         'spending it in another']);
end
end


% =============================================================================
%   BENCH ROW HELPERS.  Schema matches what mgs.zarchan.benchSuite consumes
%   through its benchRows() adapter: what/kind/book/book_str/ours/ours_str/
%   pct/band/tol/ok/skipped/why/source/units/detail.
% =============================================================================
function B = emptyBench()
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, source, units, detail)
i = numel(B) + 1;
tol = bandTol(band);
B(i).what = what;   B(i).kind = 'value';
B(i).book = book;   B(i).book_str = sprintf('%.4g', book);
B(i).ours = ours;   B(i).ours_str = sprintf('%.5g', ours);
B(i).band = band;   B(i).tol = tol;
B(i).units = units; B(i).source = source;  B(i).detail = detail;
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
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'skip';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = '--';
B(i).pct  = NaN;   B(i).band = 'skip';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end

function t = bandTol(band)
%BANDTOL  Bands encode the PROVENANCE of the book value.  Inventing one
%   silently sets an arbitrary pass criterion, so an unknown band ERRORS.
switch lower(band)
    case 'exact',  t = 1e-12;  % a parameter echoed from prose, or an identity
    case 'text',   t = 0.02;   % printed in prose to 2-3 figures
    case 'approx', t = 0.05;   % the book says "approximately"
    case 'graph',  t = 0.15;   % the author read it off a figure's ordinate
    otherwise
        error('mgs:zarchan:shaping2d:band', ...
            ['unknown tolerance band "%s".  Bands encode the PROVENANCE of ' ...
             'the book value; inventing one silently sets an arbitrary pass ' ...
             'criterion.'], band);
end
end

function w = gateWhy(offbook, names)
%GATEWHY  '' if the caller left all of NAMES at the book value; otherwise the
%   reason the row cannot be compared against the book.
hit = intersect(offbook, names);
if isempty(hit)
    w = '';
else
    w = sprintf('off-book: %s', strjoin(sort(hit), ', '));
end
end


% =============================================================================
%   SMALL ANALYSIS HELPERS
% =============================================================================
function sep = maxAltSeparation(a, b)
%MAXALTSEPARATION  Largest altitude difference between two trajectories
%   sampled at the same downrange, in km.
%
%   ⚠️ Interpolation is restricted to the strictly-INCREASING prefix of each
%      downrange array.  Both trajectories end in a near-vertical plunge in
%      which downrange is stationary and can go slightly backwards; handing
%      that to interp1 either errors on non-unique sample points or, worse,
%      silently picks one of the duplicated branches.  Restricting to the
%      monotone prefix is also the only reading under which
%      "altitude at a given downrange" is a function at all.
xa = monoPrefix(a.RM1K);  ya = a.RM2K(1:numel(xa));
xb = monoPrefix(b.RM1K);  yb = b.RM2K(1:numel(xb));
lo = max(min(xa), min(xb));
hi = min(max(xa), max(xb));
if ~(hi > lo)
    sep = NaN;
    return
end
xq = lo + (hi-lo)*(0:399)/399;
sep = 0;
for i = 1:numel(xq)
    ha = lerp(xa, ya, xq(i));
    hb = lerp(xb, yb, xq(i));
    if isfinite(ha) && isfinite(hb) && abs(ha-hb) > sep
        sep = abs(ha-hb);
    end
end
end

function x = monoPrefix(x)
k = 1;
while k < numel(x) && x(k+1) > x(k)
    k = k + 1;
end
x = x(1:k);
end

function y = lerp(xs, ys, q)
if q < xs(1) || q > xs(end)
    y = NaN;  return
end
j = find(xs <= q, 1, 'last');
if j >= numel(xs)
    y = ys(end);  return
end
f = (q - xs(j))/(xs(j+1) - xs(j));
y = ys(j) + f*(ys(j+1) - ys(j));
end

function n = totalEvals(R)
% ⚠️ R.run is counted ONLY when it was actually flown.  On a default call it
%    is one of the cases below, and adding it again would inflate the printed
%    cost of the board by about a fifth -- a number nobody would query because
%    it would still look plausible.
if R.run_reused
    n = 0;
else
    n = R.run.nevals;
end
if ~isfield(R.cases, 'opt_los'), return, end
C = R.cases;
rs = {C.opt_los, C.opt_v, C.bpn_v0, C.bpn_v10, C.bpn_los, ...
      C.bias_prose, C.bias_none, C.clamp_inverted, C.clamp_active};
for i = 1:numel(rs), n = n + rs{i}.nevals; end
for i = 1:numel(C.sweep)
    if C.sweep(i).gamfdeg ~= -90
        n = n + C.sweep(i).r.nevals;
    end
end
end

function complain(doThrow, id, varargin)
%COMPLAIN  error when asserting, warning otherwise.
%   ⚠️ 'assert', false MUST still return the result with the verdict attached.
%      A previous port in this package errored unconditionally and nobody
%      could tell which side of the comparison was wrong.
msg = sprintf(varargin{:});
if doThrow
    error(id, '%s', msg);
else
    warning(id, '%s  (''assert'', false: returning anyway)', msg);
end
end

function s = offbookStr(offbook)
if isempty(offbook)
    s = 'nothing';
else
    s = strjoin(sort(offbook), ', ');
end
end

function T = transcriptionNotes()
%TRANSCRIPTIONNOTES  The six items matlab/zarchan/_listings/listing_30_01.m
%   flags as unresolved, and how each was settled.  Carried in the returned
%   struct so the resolution travels with the result and not only with the
%   file's comments.
T.comparisons = { ...
 'if T,TBEG',     'if T < TBEG', ...
   ['prose :10930 "the bias is turned on at time zero ... turned off 30 s ' ...
    'later" => the bias is ACTIVE on TBEG <= T < TEND, so the three branches ' ...
    'read before / during / after.  DELETE EXPERIMENT: the inverted reading ' ...
    'still HITS (miss ~0.002 m) and only the terminal flight path angle ' ...
    'shows it, landing on gamIC/(1-N) = -15 deg instead of -90.']; ...
 'elseif T,TEND', 'elseif T < TEND', ...
   'same sentence, same window; the pair has to be read together.'; ...
 'if XNC.XNCLIM', 'if XNC > XNCLIM', ...
   ['the next statement is the symmetric lower arm "if XNC < -XNCLIM", so ' ...
    'this is the upper arm.  DELETE EXPERIMENT: the inverted reading clamps ' ...
    '100% of derivative evaluations and misses by ~9.7 km; a second run at a ' ...
    '2 g limit shows the correct arm firing AND releasing.']};
T.ligatures = { ...
 'IGUID (1/4) 0 / ICHOICE (1/4) 0 / ICHOICE (1/4) 1', ...
 ['all three sit inside ONE leaked prose paragraph (transcription lines ' ...
  '43-57 = zarchan.txt:10961 plus the axis text of Fig. 30.2).  Prose leaked ' ...
  'into the code region is DROPPED, never repaired into code.  Nothing was ' ...
  'lost: that paragraph is the Fig. 30.2-30.5 discussion and it is scored as ' ...
  'benchmark rows.']};
T.shape_resolved = { ...
 'while VC .=0',   'while VC >= 0'; ...
 'if RTM ,1000',   'if RTM < 1000    (feet; terminal refinement)'; ...
 'while STEP ,=1', 'while STEP <= 1  (>= never exits)'; ...
 'if XNC,-XNCLIM', 'if XNC < -XNCLIM'; ...
 'if S.= .09999',  'if S >= .09999   (the 0.1 s log cadence)'};
T.dropped = { ...
 'transcription lines 43-57', ...
 'leaked prose + Fig. 30.2 axis text; not code'; ...
 'the listing''s plot labels', ...
 ['xlabel "Downrange (Kft)" / ylabel "Altitude (Kft)" are stale from ' ...
  'Listing 24.2: the arrays are divided by 3280, which is KILOMETRES, and ' ...
  'the book''s own Fig. 30.2 axis says "Downrange (km)".  This port labels km.']};
T.additions = { ...
 'ArrayVMM', ...
 ['the listing computes VMM and never stores it, although the prose says ' ...
  'missile velocity is written to the file and Fig. 30.5 plots it.']; ...
 'ArrayVC', ...
 'needed for the terminal-quantum diagnostics; not in the book.'; ...
 'rt2m', ...
 ['the only non-Zarchan SCENARIO line: RT2IC is hard-coded 0 in the book. ' ...
  'Default 0 reproduces it exactly and every benchmark row gates on it.']};
end


% =============================================================================
%   OUTPUT
% =============================================================================
function s = render(R, c, offbook, discOffbook, ev)
W = 104;
L = {};
L{end+1} = 'Zarchan Listing 30.1 -- 2-D ENGAGEMENT, STATIONARY TARGET, TERMINAL FLIGHT-PATH-ANGLE CONSTRAINT';
L{end+1} = '*** UNITS: interface METRIC (m, m/s, km, g, deg, m^2/s^3); loop in FEET with the book''s';
L{end+1} = '    own 32.2 / 3.28 / 3280 / 57.3.  This is the ONE listing in the book that is not in feet.';
L{end+1} = sprintf(['scenario: stationary target %g m downrange at %g m altitude | VM(0) = %g m/s, ' ...
                    'gamIC = %+g deg, gamF = %+g deg'], ...
    c.rangem, c.rt2m, c.vmic, c.gamdeg, c.gamfdeg);
L{end+1} = sprintf('guidance: N = %g, limit %g g, bias window [%g, %g] s (DELT = %g), bias form ''%s''', ...
    c.xnp, c.xnclimg, c.tbeg, c.tbeg + c.delt, c.delt, c.biasform);
L{end+1} = sprintf(['integrator: Zarchan''s two-pass predictor/corrector (Heun), H = %g s dropping ' ...
                    'to %g s inside %g ft'], c.h, c.hfine, c.rfine);
L{end+1} = sprintf('selected run: IGUID = %d (%s), ICHOICE = %d (%s)', ...
    c.iguid, guidName(c.iguid), c.ichoice, cmdName(c.ichoice));
if isempty(offbook) && isempty(discOffbook)
    L{end+1} = 'configuration: BOOK NOMINAL (nothing moved)';
else
    L{end+1} = sprintf('configuration: OFF-BOOK -- model %s | discretization %s', ...
        offbookStr(offbook), offbookStr(discOffbook));
end
L{end+1} = repmat('-', 1, W);

% ---- the selected run ----------------------------------------------------
r = R.run;
L{end+1} = 'SELECTED RUN';
L{end+1} = sprintf(['  tf %.3f s | miss %.5f m (floor hfine*|V| = %.5f m, ratio %.2f) | ' ...
                    'gam_f %+.4f deg'], r.tf_s, r.miss_m, r.quantum_m, ...
                    r.miss_over_quantum, r.gam_f_deg);
L{end+1} = sprintf(['  V %.2f -> %.2f m/s (min %.2f, max %.2f) | apogee %.4f km | ' ...
                    'bias %+.4f deg/s'], r.vm_ic_ms, r.vm_f_ms, r.vm_min_ms, ...
                    r.vm_max_ms, r.apogee_km, r.bias_degps);
L{end+1} = sprintf(['  peak |n_c| %.4f g on the 0.1 s log (Fig. 30.3/30.7 scale) | ' ...
                    '%.4g g per step | raw pre-clamp %.4g g'], ...
                    r.nc_max_g_log, r.nc_max_g_step, r.nc_raw_absmax_g);
L{end+1} = sprintf('  performance index X = %.2f m^2/s^3 | %d steps, %d derivative evals, %d log points', ...
    r.x_perf, r.nsteps, r.nevals, r.nlog);
if r.timed_out
    L{end+1} = '  ⚠️ TIMED OUT: this engagement never closed.  miss/gam_f/X are NaN on purpose.';
end
L{end+1} = repmat('-', 1, W);

% ---- the case table ------------------------------------------------------
if isfield(R.cases, 'opt_los')
    C = R.cases;
    L{end+1} = 'BOOK CASES';
    L{end+1} = sprintf('  %-30s %8s %10s %10s %9s %9s %9s %11s', ...
        'case', 'tf [s]', 'miss [m]', 'gam_f', 'V_f m/s', 'apo km', 'nc_max g', 'X m^2/s^3');
    tab = { 'OPT  _|_LOS  (Figs 30.2-30.5)', C.opt_los; ...
            'OPT  _|_VM   (Figs 30.2-30.5)', C.opt_v; ...
            'BPN  _|_VM   TBEG=0',           C.bpn_v0; ...
            'BPN  _|_VM   TBEG=10',          C.bpn_v10; ...
            'BPN  _|_LOS  (book never runs)',C.bpn_los; ...
            'BPN  prose bias (ERRATUM)',     C.bias_prose; ...
            'BPN  bias off (== inverted T)', C.bias_none; ...
            'BPN  clamp reading INVERTED',   C.clamp_inverted; ...
            'BPN  limit 2 g (clamp active)', C.clamp_active};
    for i = 1:size(tab,1)
        q = tab{i,2};
        L{end+1} = sprintf('  %-30s %8.3f %10.4f %10.3f %9.2f %9.4f %9.4f %11.1f', ...
            tab{i,1}, q.tf_s, q.miss_m, q.gam_f_deg, q.vm_f_ms, q.apogee_km, ...
            q.nc_max_g_log, q.x_perf); %#ok<AGROW>
    end
    for i = 1:numel(C.sweep)
        q = C.sweep(i).r;
        L{end+1} = sprintf('  %-30s %8.3f %10.4f %10.3f %9.2f %9.4f %9.4f %11.1f', ...
            sprintf('BPN  _|_VM   gamF=%+g', C.sweep(i).gamfdeg), ...
            q.tf_s, q.miss_m, q.gam_f_deg, q.vm_f_ms, q.apogee_km, ...
            q.nc_max_g_log, q.x_perf); %#ok<AGROW>
    end
    L{end+1} = repmat('-', 1, W);
end

% ---- structural ----------------------------------------------------------
if isfield(R.selfcheck, 'ok')
    K = R.selfcheck;
    L{end+1} = 'STRUCTURAL SELF-CHECK  (wiring and OCR resolution -- a failure here voids everything above)';
    L{end+1} = sprintf('  %-46s %-34s %s', 'bias satisfies the book''s integrated law', ...
        sprintf('%+.4f deg/s, resid %.1e', K.bias_degps, K.bias_resid_degps), tf2s(K.bias_ok));
    L{end+1} = sprintf('  %-46s %-34s %s', 'unbiased PN lands on gamIC/(1-N)', ...
        sprintf('%+.5f vs %+.5f deg', K.nobias_gamf_deg, K.nobias_pred_deg), tf2s(K.nobias_ok));
    L{end+1} = '      ^ THIS is the row that resolves "if T,TBEG".  Under the inverted reading the';
    L{end+1} = sprintf('        nominal case lands HERE, and still hits (miss %.4f m).  Miss distance is blind to it.', ...
        K.nobias_miss_m);
    L{end+1} = sprintf('  %-46s %-34s %s', 'DELETE EXPT: prose bias form (erratum)', ...
        sprintf('%+.3f deg/s -> miss %.0f m', K.prose_bias_degps, K.prose_miss_m), tf2s(K.prose_ok));
    L{end+1} = sprintf('  %-46s %-34s %s', 'clamp inert at the book''s own 10 g', ...
        sprintf('%.2e of evals, all RTM < %.3g ft', K.clamp_frac_max, K.clamp_rtm_max_ft), ...
        tf2s(K.clamp_inert_ok));
    L{end+1} = sprintf('  %-46s %-34s %s', 'DELETE EXPT: inverted clamp reading', ...
        sprintf('%.3f clamped, miss %.0f m', K.invclamp_frac, K.invclamp_miss_m), tf2s(K.invclamp_ok));
    L{end+1} = sprintf('  %-46s %-34s %s', 'DELETE EXPT: clamp FIRES and RELEASES at 2 g', ...
        sprintf('%.3f clamped, miss %.4f m', K.clamp2_frac, K.clamp2_miss_m), tf2s(K.clamp2_ok));
    L{end+1} = '      ^ under the inverted reading this fraction is 1.000 by construction.';
    L{end+1} = sprintf('  %-46s %-34s %s', 'ICHOICE branches not swapped', ...
        sprintf('_|_VM %.1e, _|_LOS %.1f%% loss', K.perpv_speed_rel, 100*K.perplos_loss_rel), ...
        tf2s(K.ichoice_ok));
    L{end+1} = sprintf('  %-46s %-34s %s', 'performance index never decreases', ...
        sprintf('worst slack %.1e', K.xperf_slack), tf2s(K.xperf_ok));
    L{end+1} = sprintf('  %-46s %-34s %s', 'miss is at the integrator floor, not physical', ...
        sprintf('max miss/quantum %.3f', K.miss_over_quantum_max), tf2s(K.quantum_ok));
    L{end+1} = sprintf('  %-46s %-34s %s', 'every engagement closed (no tmax break)', ...
        sprintf('timed out: %s', tf2yn(K.timed_out_any)), tf2s(K.closure_ok));
else
    L{end+1} = 'STRUCTURAL SELF-CHECK  ***SKIPPED*** (benchmark = false) -- nothing is verified';
end
L{end+1} = repmat('-', 1, W);

% ---- the board -----------------------------------------------------------
if isempty(R.bench)
    L{end+1} = 'BOOK BENCHMARKS  ***NOT RUN*** (benchmark = false).  R.pass is false, meaning';
    L{end+1} = '  "nothing was checked", NOT "everything is fine".';
else
    L{end+1} = 'BOOK BENCHMARKS   computed vs published, with the zarchan.txt line each claim came from';
    L{end+1} = sprintf('  %-42s %-30s %-26s %s', 'what', 'book', 'ours', 'verdict');
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch b.kind
            case 'info'
                L{end+1} = sprintf('  %-42s %-30s %-26s INFO', ...
                    trunc(b.what,42), trunc(b.book_str,30), trunc(b.ours_str,26)); %#ok<AGROW>
            case 'skip'
                L{end+1} = sprintf('  %-42s %-30s %-26s SKIP (%s)', ...
                    trunc(b.what,42), trunc(b.book_str,30), '--', b.why); %#ok<AGROW>
            case {'shape','bound'}
                L{end+1} = sprintf('  %-42s %-30s %-26s %s', ...
                    trunc(b.what,42), trunc(b.book_str,30), trunc(b.ours_str,26), ...
                    tf2s(b.ok)); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf('  %-42s %-30s %-26s %s', ...
                    trunc(b.what,42), trunc([b.book_str ' ' b.units],30), ...
                    trunc([b.ours_str ' ' b.units],26), tf2s(b.ok)); %#ok<AGROW>
        end
    end
    nEval = sum(ev);
    nPass = sum([R.bench(ev).ok]);
    nRes  = R.n_result_evaluated;
    nInfo = sum(strcmp({R.bench.kind}, 'info'));
    nSkip = sum(strcmp({R.bench.kind}, 'skip'));
    L{end+1} = repmat('-', 1, W);
    L{end+1} = sprintf(['VERDICT: %d/%d evaluated rows PASS   (%d INFO, not scored; ' ...
                        '%d SKIP, not evaluable)'], nPass, nEval, nInfo, nSkip);
    L{end+1} = sprintf(['         %d of those describe a TRAJECTORY; the rest lock the data ' ...
                        'statement only.'], nRes);
    if nRes == 0
        L{end+1} = '         ⚠️ ZERO trajectory rows evaluated -- R.pass is FALSE regardless of the count above.';
    end
end
L{end+1} = sprintf('%d derivative evaluations, %.3f s elapsed -- both measured by this run, not estimated.', ...
    R.nevals, R.elapsed_s);
L{end+1} = 'NO gravity, NO drag, NO thrust, NO autopilot lag, NO seeker.  The miss distances here are';
L{end+1} = 'numerical floors by construction and are NOT sortie predictions.';
s = strjoin(L, newline);
end

function s = guidName(i)
if i == 0, s = 'optimal trajectory shaping, needs tgo';
else,      s = 'biased PN, no tgo';
end
end

function s = cmdName(i)
if i == 0, s = 'a_cmd _|_ LOS';
else,      s = 'a_cmd _|_ velocity';
end
end

function s = tf2s(b)
if b, s = 'PASS'; else, s = 'FAIL'; end
end

function s = tf2yn(b)
if b, s = 'YES'; else, s = 'no'; end
end

function s = trunc(s, n)
if numel(s) > n, s = [s(1:n-2) '..']; end
end


function drawFigures(R, c)
%DRAWFIGURES  The listing's own three plots (:10957-10959) plus Fig. 30.5.
%   ⚠️ The listing labels its axes "Kft".  The arrays are divided by 3280,
%      which is KILOMETRES, and the book's own Fig. 30.2 axis says
%      "Downrange (km)".  The label in the printed listing is stale from
%      Listing 24.2, which really was in feet.  Labelled km here.
if isfield(R.cases, 'opt_los')
    A = R.cases.opt_los;  B = R.cases.opt_v;
    figure;
    plot(A.RM1K, A.RM2K, '-', B.RM1K, B.RM2K, '--', ...
         A.RT1K(1), A.RT2K(1), 'kx', 'LineWidth', 1.3); grid on
    title('Fig. 30.2  Engagement geometry, optimal shaping');
    xlabel('Downrange (km)'); ylabel('Altitude (km)');
    legend('perpendicular to LOS', 'perpendicular to V_M', 'target', 'Location', 'best');

    figure; plot(A.T, A.XNCG, '-', B.T, B.XNCG, '--', 'LineWidth', 1.3); grid on
    title('Fig. 30.3  Commanded acceleration');
    xlabel('Time (s)'); ylabel('n_c (g)');
    legend('perpendicular to LOS', 'perpendicular to V_M', 'Location', 'best');

    figure; plot(A.T, A.GAMDEG, '-', B.T, B.GAMDEG, '--', 'LineWidth', 1.3); grid on
    title('Fig. 30.4  Flight path angle');
    xlabel('Time (s)'); ylabel('\gamma (deg)');
    legend('perpendicular to LOS', 'perpendicular to V_M', 'Location', 'best');

    figure; plot(A.T, A.VMM, '-', B.T, B.VMM, '--', 'LineWidth', 1.3); grid on
    title('Fig. 30.5  Missile velocity');
    xlabel('Time (s)'); ylabel('V_M (m/s)');
    legend('perpendicular to LOS', 'perpendicular to V_M', 'Location', 'best');
else
    r = R.run;
    figure; plot(r.RM1K, r.RM2K, '-', 'LineWidth', 1.3); grid on
    title(sprintf('Listing 30.1  IGUID = %d, ICHOICE = %d', c.iguid, c.ichoice));
    xlabel('Downrange (km)'); ylabel('Altitude (km)');
end
end


function writeDatfil(r, path)
%WRITEDATFIL  The listing's "save datfil.txt output /ascii" (:10960).
%   Columns 1-7 are the book's, in the book's order.  ⚠️ Columns 8 and 9 are
%   ADDED: the printed output matrix omits missile velocity and the
%   performance index even though the prose (:10930) says both are written to
%   the file.  Appending rather than inserting keeps the first seven columns
%   byte-compatible with anything that reads the book's format.
output = [r.T', r.RT1K', r.RT2K', r.RM1K', r.RM2K', r.XNCG', r.GAMDEG', ...
          r.VMM', r.XM'];
save(path, 'output', '-ascii');
end
