function R = threeLoopAutopilot(opts)
%MGS.ZARCHAN.THREELOOPAUTOPILOT  Zarchan Listing 23.1 -- THREE-LOOP AUTOPILOT
%   STEP RESPONSE SIMULATION, ported line for line.
%
%   R = mgs.zarchan.threeLoopAutopilot()                    % book run + benchmarks
%   R = mgs.zarchan.threeLoopAutopilot('tau', 0.1)          % Fig. 23.5
%   R = mgs.zarchan.threeLoopAutopilot('wcr', 100)          % Fig. 23.8
%   R = mgs.zarchan.threeLoopAutopilot('wcr', 100, 'wact', 300)   % Fig. 23.9
%   R = mgs.zarchan.threeLoopAutopilot('zeta', 0.4)         % Fig. 23.11
%   R = mgs.zarchan.threeLoopAutopilot('crosscheck', false) % skip the drift row
%
%   Five states, Zarchan's two-pass predictor/corrector, TF = 1 s: a 10-g step
%   into a three-loop autopilot (rate gyro, accelerometer, integrator) driving
%   a second-order actuator and a second-order airframe with a right-half-plane
%   zero.  The gains are not chosen; they are COMPUTED from the airframe by the
%   Ch.23 gain algorithm, given a desired time constant, damping and open-loop
%   crossover frequency.
%
% ===========================================================================
%   *** UNITS ARE THE BOOK'S.  FEET, ft/s, g = 32.2 ft/s^2, DEGREES. ***
%
%   Zarchan works in feet with g = 32.2 and converts rad->deg with the literal
%   57.3 (and the combined 1845 = 57.3*32.2).  The rest of mgs is metres.  This
%   file is DELIBERATELY left in the book's units so the published numbers can
%   be compared digit for digit with nothing in between to be got wrong.
%   Accelerations are g, fin deflections DEGREES, fin rates deg/s, K1 is g/deg,
%   KA is deg/g-s, KR is s, wI/W/wAF/wZ/wACT are rad/s, times seconds.
%   Convert at the CALL SITE, visibly.
% ===========================================================================
%
%   WHY THIS LISTING IS WORTH A PORT OF ITS OWN
%
%   Every other lag in this package is a LUMPED time constant -- one pole
%   (Listing 3.1), five equal poles (Listing 6.1).  You pick T and the loop
%   obeys.  Our vehicle does not work like that: we MEASURED a gamma-tracking
%   gain of 0.063 against the model's 1.385, with roughly 8 s of lag saturating
%   on 26 of 32 sorties, and we cannot derive that 8 s from anything -- only
%   fit it.
%
%   Listing 23.1 is where the book MEASURES a flight-control time constant
%   instead of asserting one.  Everything upstream of the step is real: gains
%   from an algorithm, an actuator at 150 rad/s, an airframe from geometry.
%   The lag falls out, and it can be checked against the closed form
%
%       tau_TOT = tau + 2*zeta/W          (zarchan.txt:7535, p.538)
%
%   At sea level that is 0.3 + 1.4/35.43 = 0.3395 s, and the step response
%   reaches 63% of the command in 0.34 s.  Two independent computations of one
%   lag, agreeing.  R.lag collects them.
%
%   The transferable result is stronger than the number: the algorithm HOLDS
%   that lag fixed while the airframe underneath changes by a factor of six
%   (Ma goes -642 -> -99.1 over 50 kft).  Ours is held fixed by nothing --
%   ArduPilot's pitch loop has no such scheduler.  That is a concrete,
%   checkable difference between the two systems rather than a vague "our
%   autopilot is worse".
%
% ---------------------------------------------------------------------------
%   *** THE FINDING OF THIS PORT: THE BOOK'S 0.34 s IS A DECIMATED READING ***
%
%   Listing 23.1 does not plot every integration step.  It carries a sampler
%
%       S = S + H;  if S >= .0099999, S = 0.; count = count+1; ArrayT(count)=T;
%
%   so the arrays that become Figs. 23.3 and 23.12 are on a 0.01 s grid, NOT on
%   the 0.0001 s integration grid.  That is not cosmetic:
%
%       reading 63% off the 0.01 s OUTPUT grid  ->  0.3400 s   at 0, 30, 50 kft
%       reading 63% off the full-rate array     ->  0.3367 / 0.3351 / 0.3304 s
%
%   The book prints 0.34 s.  It is the FIRST reading, exactly, at every one of
%   the three altitudes -- which is also the literal sense in which "the gains
%   enable the flight-control system to maintain the same step response for the
%   three different flight conditions" (zarchan.txt:7678) is true: on the grid
%   the author actually plotted, the three responses reach 63% at the SAME
%   sample.  The full-rate reading spreads by 1.9% and is 1% below the printed
%   value.
%
%   Both are computed and both are reported.  t63_s is the BOOK'S grid (it is
%   what the book's number means); t63_fullrate_s is the full-rate reading (it
%   is what mgs.zarchan.adjointWithAutopilot returns, and the cross-check row
%   below compares against it).  Neither is wrong; quoting one as the other is.
%   This is also why H is guarded to divide 0.01 s exactly -- see RESOLVECONFIG.
%
% ---------------------------------------------------------------------------
%   RELATIONSHIP TO MGS.ZARCHAN.ADJOINTWITHAUTOPILOT (Listing 23.3)
%
%   Listings 23.1, 23.2 and 23.3 each carry a BYTE-IDENTICAL copy of the
%   aerodynamic and gain-algorithm preamble; the book duplicated it three
%   times.  23.3 was ported first and, to stand alone, pulled in both that
%   preamble (as its AIRFRAME) and the five-state step loop (as its FCSSTEP).
%   So a second copy of both already exists.
%
%   WHAT WAS DONE ABOUT IT, AND WHY NOT THE OBVIOUS THING.  The obvious fix is
%   to call the neighbour's code.  It is not reachable: AIRFRAME and FCSSTEP
%   are LOCAL functions of adjointWithAutopilot.m and MATLAB gives no way to
%   call a local function from another file.  The alternatives were to refactor
%   an audited 63-row port so this one could borrow from it -- editing verified
%   code to serve unverified code -- or to duplicate and then PROVE the two
%   copies still agree.
%
%   This file duplicates and proves.  NEIGHBOURCROSSCHECK() runs
%   adjointWithAutopilot at whatever flight conditions and design goals THIS
%   run is using and compares all nineteen aero and gain quantities at a
%   relative 1e-12, plus the full-rate t63.  If either file's preamble is
%   edited, the row goes red and names the quantity that moved.  A shared copy
%   nobody checks prevents nothing; this repo has been burned three times in
%   one day by guards that were documented and absent, and the only thing that
%   ever settled it was running the experiment.  The row costs a few seconds
%   ('crosscheck', false turns it off) and it is the single most valuable row
%   in the file, because unlike every book row it still bites off-book.
%
%   WHAT IS HERE AND NOT THERE: the 0.01 s output decimation and everything
%   that follows from it (above), the fin-rate and fin-deflection histories,
%   the four off-nominal studies of Figs. 23.5/23.8/23.9/23.11, and the ringing
%   metric that turns the book's word "oscillatory" into a measurement.
%   FCSSTEP has none of that; it is the inner loop only.
%
% ---------------------------------------------------------------------------
%   PROVENANCE, AND THE THREE OCR REPAIRS
%
%   Transcription: matlab/zarchan/_listings/listing_23_01.m, from
%   zarchan.txt:7536-7600 (p.537-540).  Three comparisons were damaged the
%   usual way, all three resolved and none guessed:
%
%     while T,=(TF-.00001)   ->  while T <= (TF-.00001)
%         T starts at 0, is incremented by H, and the loop must end at TF.
%         Only '<=' terminates; '>=' never enters.  (Same repair as 23.3.)
%
%     while STEP,=1          ->  while STEP <= 1
%         Already repaired by the extractor.  STEP is 1 on the first pass and
%         set to 2 on the second, which is how the loop exits.
%
%     if S..0099999          ->  if S >= .0099999
%         NOT resolved from shape -- resolved from FOUR SIBLING LISTINGS that
%         carry the same sampler with the operator intact.  listing_22_01.m
%         and listing_22_02.m have the identical line down to the payload
%         (ArrayT/ArrayXNL/ArrayXNCG): "S=S+H; if S >= .0099999, S=0.;".
%         listing_21_01.m and listing_21_02.m are the same at 0.0099999, and
%         listing_01_01.m and listing_04_06.m the same at .000999.  S
%         accumulates H and is reset to 0, so only '>=' (or '>') can be meant,
%         and the siblings settle which.  This one MATTERS -- it is the whole
%         0.34-vs-0.3367 finding above -- so it was not left to inference.
%
%   Dropped, not "fixed": the transcription's last lines carry prose that leaked
%   into the code block during extraction ("The open-loop transfer function ...
%   HGNO ACTUATOR").  That is page text, not a statement.
%
%   Not carried, because the listing never reads them: XNLLIN=0., XACC=XCG
%   (leftovers from the Ch.22 nonlinear simulation) and XK2=XK1.  The stray
%   double semicolons after ZA= and XKA= are the book's own typing.
%
%   ONE INFERENCE, FLAGGED BECAUSE IT IS AN INFERENCE.  Fig. 23.13 plots fin
%   rate against time for this same 10-g step at the three altitudes, and the
%   fin-rate benchmark rows below rest on it -- but Listing 23.1 AS PRINTED
%   saves only ArrayT, ArrayXNL and ArrayXNCG.  It does not store DELD.  The
%   author must have added it to produce that figure.  This port computes and
%   returns DELD (there is no other candidate run in the chapter: same command,
%   same three altitudes, same page), and says so here rather than presenting
%   the fin-rate rows as if the listing had printed them.
%
% ---------------------------------------------------------------------------
%   WHAT THE BOOK PUBLISHES ABOUT THIS STEP RESPONSE -- AND WHAT IT DOES NOT
%
%   Grepped, and the absence is the answer: the words "overshoot", "settling"
%   and "rise time" DO NOT OCCUR anywhere in Ch.23 (zarchan.txt:7380-7830).
%   The only published step-response characteristic is the 63% rise time of
%   0.34 s.  There is no overshoot figure and no settling time to score, so
%   none is invented; R.fc(i).step reports peak and settled fraction as data,
%   not as benchmarks.  (For the record: at the book's parameters the response
%   does not overshoot at all inside TF = 1 s -- it is still rising, at 96% of
%   steady state, when the run stops.  Anyone expecting an overshoot number
%   from this chapter is looking for something that is not there.)
%
%   The book's claims about the OFF-NOMINAL cases are adjectives -- "has an
%   oscillatory response", "stabilizes and is well behaved" -- so they are
%   scored as shape rows against a ringing count, which is sharper than a level
%   read off a plot: 0 reversals nominal, 32 at wcr = 100 with the 150 rad/s
%   actuator, 0 again when the actuator goes to 300 rad/s, 18 at zeta = 0.4.
%
% ---------------------------------------------------------------------------
%   BENCHMARK MODE IS A FLAG.  IT IS NOT nargin.
%
%   In a function whose arguments block declares ONLY name-value options,
%   MATLAB reports nargin = 0 NO MATTER HOW MANY were passed (measured, R2025b).
%   Three ports in the first wave used `if nargin == 0`: one became infinite
%   recursion and two silently replaced the caller's arguments with the
%   benchmark run.  'benchmark' is the ONLY mode switch here.
%
%   It defaults TRUE, so a bare call is a CHECKED call.  Benchmark mode does
%   not override anything the caller passed; instead every book value that
%   depends on a parameter the caller moved is SKIPPED with the reason printed.
%   That is why the book defaults live in BOOKNOMINAL() and the arguments-block
%   defaults are [] -- "the caller supplied this" is then exactly "the field is
%   non-empty", with no second copy of the constants to drift out of agreement.
%
%   COST: milliseconds, except for the cross-check row, which runs the
%   neighbouring adjoint port and takes a few seconds.
%
% ---------------------------------------------------------------------------
%   WHAT THIS DOES NOT MODEL, AND MUST NOT BE REPORTED AS IF IT DID
%
%   Constant speed (Mach 3), constant altitude, no gravity, no drag, no thrust,
%   no acceleration limit, no fin-rate or fin-deflection limit, and aero
%   LINEARIZED about a 10-g trim.  It is a SINGLE-CHANNEL step response into a
%   linear plant -- there is no guidance loop, no seeker and no target.  A time
%   constant computed here is a property of that linear plant, not a sortie
%   prediction, and our own gamma-tracking lag is not measured this way.
%
% ---------------------------------------------------------------------------
%   OUTPUT
%
%   R.lag        the point of the exercise, one column per flight condition:
%                .tau_design_s, .tau_total_s (the p.538 closed form),
%                .t63_s (BOOK 0.01 s grid -- what "0.34 s" means),
%                .t63_fullrate_s (the integration grid -- what 23.3 returns),
%                .t63_vs_last_sample_s (the mis-reading, carried to be
%                recognised), .dc_gain (an identity, must be 1), .settled_frac
%   R.fc(i)      per altitude: .rho .q .mach .trim, aero (.xma .xmd .za .zd
%                .wz .waf .zaf .xk1 .ta .xk3), gains (.xkc .xka .wi .xkr .xkdc
%                .w .w0 .z0 .xk0 .xk), .tau_total, .step
%   R.fc(i).step .t .nl_g .finrate_dps .del_deg           full integration grid
%                .tb .nl_book_g .finrate_book_dps .nc_book_g   BOOK 0.01 s grid
%                .ss_g .dc_gain .t63_s .t63_fullrate_s .peak_finrate_dps
%                .wrongway_g .nrev .ripple_g .settled_frac ...
%   R.variants   Figs. 23.5 / 23.8 / 23.9 / 23.11, each with .claim and .source
%   R.cross      the adjointWithAutopilot drift check (or why it was skipped)
%   R.bench      computed vs book, one row per published value, each carrying
%                .source (table/figure + page + zarchan.txt line)
%   R.pass       true iff every EVALUATED row passed
%   R.selfcheck  structural invariants and their measured residuals
%   R.cfg        the resolved configuration; R.offbook names what the caller moved
%   R.summary    the printed text
%
%   Bench rows carry BOTH numeric (.book/.ours, NaN on shape rows) and text
%   (.book_str/.ours_str, always populated), the SAME schema as
%   adjointWithAutopilot, so benchSuite's adaptAutopilot() consumes them
%   unchanged.  NOTE this port is not yet wired into benchSuite: doing so means
%   adding a manifest entry AND bumping AUDITED_TOTAL, and that file refuses to
%   run if the two disagree -- which is the intended behaviour, not an obstacle.
%
%   See also MGS.ZARCHAN.ADJOINTWITHAUTOPILOT, MGS.ZARCHAN.BENCHSUITE.

arguments
    % Defaults are [] on purpose -- see BOOKNOMINAL().  Empty means "use the
    % book value"; non-empty means "the caller chose this", which is what gates
    % the benchmark rows.  Do NOT replace these with literals: that puts a
    % second copy of every book constant in the file and destroys the only
    % mechanism this port has for telling an off-book run apart from a broken
    % one.
    opts.alt        double {mustBeReal, mustBeFinite} = []   % ALT   [ft], swept
    opts.tau        double {mustBeReal, mustBeFinite} = []   % TAU   [s]     FCS time-constant goal
    opts.wcr        double {mustBeReal, mustBeFinite} = []   % WCR   [rad/s] open-loop crossover goal
    opts.zeta       double {mustBeReal, mustBeFinite} = []   % ZETA  [-]     FCS damping goal
    opts.wact       double {mustBeReal, mustBeFinite} = []   % WACT  [rad/s] actuator
    opts.zact       double {mustBeReal, mustBeFinite} = []   % ZACT  [-]     actuator
    opts.vm         double {mustBeReal, mustBeFinite} = []   % VM    [ft/s]
    opts.wgt        double {mustBeReal, mustBeFinite} = []   % WGT   [lb]
    opts.xncg       double {mustBeReal, mustBeFinite} = []   % XNCG  [g] step command AND trim point
    opts.tf         double {mustBeReal, mustBeFinite} = []   % TF    [s]
    % H is EXCLUDED from the off-book gating below: it is a discretization
    % knob, not a model parameter.  Between h = 1e-4 and h = 2e-3 the BOOK-GRID
    % t63 does not move at all and the full-rate t63 moves by one sample.  What
    % h CAN do is destabilise the 150 rad/s actuator and break the 0.01 s output
    % grid, and both are guarded explicitly rather than left to a tolerance.
    opts.h          double {mustBeReal, mustBeFinite} = []   % H     [s]
    opts.benchmark  (1,1) logical = true    % THE ONLY MODE SWITCH.  Not nargin.
    opts.crosscheck (1,1) logical = true    % run adjointWithAutopilot and compare
    opts.render     (1,1) logical = true
    opts.plot       (1,1) logical = false
    opts.assert     (1,1) logical = true
end

GRAV_FT   = 32.2;    % g, the book's value.  Not 9.80665, not 9.81.
BOOK_GRID = 0.01;    % s, the listing's own output sampler (if S >= .0099999)

N = bookNominal();
[cfg, offbook] = resolveConfig(opts, N, BOOK_GRID);

R = struct();
R.units    = 'ft, ft/s, deg, deg/s, g = 32.2 ft/s^2 -- Zarchan''s units, NOT SI';
R.listing  = 'Zarchan Listing 23.1 (zarchan.txt:7536), three-loop autopilot step response';
R.opts     = opts;
R.cfg      = cfg;
R.offbook  = offbook;
R.nominal  = N;
R.book_grid_s = BOOK_GRID;

% ---- flight conditions: aero, gains, step response ------------------------
for i = 1:numel(cfg.alt)
    F = airframe(cfg.alt(i), cfg, GRAV_FT);
    F.step = stepResponse(F, cfg, BOOK_GRID);
    if i == 1, R.fc = F; else, R.fc(i) = F; end %#ok<AGROW>
end

R.lag = lagSummary(R.fc, cfg);

% ---- the four off-nominal studies the chapter makes claims about ---------
if cfg.benchmark
    R.variants = variantStudies(cfg, GRAV_FT, BOOK_GRID);
else
    R.variants = [];
end

% ---- the drift check against the neighbouring port -----------------------
if cfg.crosscheck
    R.cross = neighbourCrossCheck(R.fc, cfg);
else
    R.cross = struct('ran', false, 'ok', false, 'nchecked', 0, 'worst_rel', NaN, ...
        'worst_name', '', 't63_delta_s', NaN, 'why', ...
        'crosscheck = false: the duplicated preamble was NOT compared this run');
end

% ---- structural invariants and the board ---------------------------------
if cfg.benchmark
    R.selfcheck = structuralCheck(cfg, GRAV_FT, BOOK_GRID);
    R.bench     = benchmarks(R, cfg, offbook);
    ev          = ~[R.bench.skipped];
    R.pass      = any(ev) && all([R.bench(ev).ok]);
else
    R.selfcheck = struct('skipped', true);
    R.bench     = emptyBench();
    ev          = false;
    R.pass      = false;   % NOT true.  Nothing was checked; the summary says so.
end

R.summary = render(R, cfg, offbook);
if cfg.render, fprintf('%s\n', R.summary); end
if cfg.plot,   drawCurves(R, cfg); end

if cfg.assert && cfg.benchmark
    % "Nothing was evaluated" must NOT read as "everything passed".  A run
    % whose altitudes put every book value out of reach has checked NOTHING,
    % and that is the shape of green light this repo has been burned by.
    if ~any(ev)
        error('mgs:zarchan:threeLoopAutopilot:noBenchmark', ...
            ['not one book value could be evaluated.  The published numbers ' ...
             'live at alt = 0, 30000 and 50000 ft with the nominal design ' ...
             'goals; this run has alt = %s and moved %s.  Refusing to hand ' ...
             'back an unvalidated port silently -- pass ''benchmark'', false ' ...
             'if you meant to run off-book.'], mat2str(cfg.alt), offbookStr(offbook));
    end
    if ~R.pass
        bad = find(~[R.bench.ok] & ev);
        error('mgs:zarchan:threeLoopAutopilot:benchmark', ...
            ['%d of %d evaluated book values are outside tolerance (first: ' ...
             '%s, book %s, ours %s, band %s).  Either the port broke or the ' ...
             'parameters left the region the book covers.  Full board:\n\n%s'], ...
            numel(bad), sum(ev), R.bench(bad(1)).what, R.bench(bad(1)).book_str, ...
            R.bench(bad(1)).ours_str, R.bench(bad(1)).band, R.summary);
    end
end
end


% =============================================================================
function N = bookNominal()
%BOOKNOMINAL  THE ONE PLACE the listing's data statement is written down.
%
%   Straight off zarchan.txt:7536 (the first line of Listing 23.1), plus the
%   altitude grid the chapter's "EXPERIMENTS WITH FLIGHT CONDITION" section
%   uses (zarchan.txt:7677, Tables 23.1 and 23.2).  Change a value here and you
%   have changed what "the book" means for every benchmark row in this file,
%   which is the intended blast radius -- there is no second copy anywhere.
N = struct();
N.alt  = [0 30000 50000];  % ALT=0. in the listing; Tables 23.1/23.2 run three
N.tau  = 0.3;              % TAU=.3
N.wcr  = 50;               % WCR=50.
N.zeta = 0.7;              % ZETA=.7
N.wact = 150;              % WACT=150.
N.zact = 0.7;              % ZACT=.7
N.vm   = 3000;             % VM=3000.
N.wgt  = 1000;             % WGT=1000.
N.xncg = 10;               % XNCG=10.
N.tf   = 1;                % TF=1.
N.h    = 0.0001;           % H=.0001
end


% =============================================================================
function [cfg, offbook] = resolveConfig(opts, N, BOOK_GRID)
%RESOLVECONFIG  Fill the empties from the book, and record what was moved.
%
%   offbook is the list of names the caller supplied with a value DIFFERENT
%   from the book's.  Supplying the book's own value is not "off book" -- a
%   caller who writes 'tau', 0.3 explicitly should still get the tau rows.
cfg = struct();
offbook = {};
f = fieldnames(N);
for k = 1:numel(f)
    nm = f{k};
    v  = opts.(nm);
    if isempty(v)
        cfg.(nm) = N.(nm);
    else
        cfg.(nm) = v(:).';                    % row vector, always
        if ~isequal(cfg.(nm), N.(nm))
            offbook{end+1} = nm; %#ok<AGROW>
        end
    end
end
cfg.benchmark  = opts.benchmark;
cfg.crosscheck = opts.crosscheck;
cfg.render     = opts.render;
cfg.plot       = opts.plot;
cfg.assert     = opts.assert;

% ---- scalarness: the thing the arguments block cannot express -------------
sc = {'tau','wcr','zeta','wact','zact','vm','wgt','xncg','tf','h'};
for k = 1:numel(sc)
    if ~isscalar(cfg.(sc{k}))
        error('mgs:zarchan:threeLoopAutopilot:notScalar', ...
            ['''%s'' must be scalar; %d values were given.  Only alt is ' ...
             'swept -- the other studies in this chapter are separate runs ' ...
             'and are returned in R.variants.'], sc{k}, numel(cfg.(sc{k})));
    end
end

% A caller CANNOT reach this: passing [] means "use the book value", so an
% empty grid can only come from BOOKNOMINAL() itself being edited to one.  The
% guard is here for that edit, because an empty alt grid produces a zero-row
% result struct that renders as a clean, entirely empty report.
if isempty(cfg.alt)
    error('mgs:zarchan:threeLoopAutopilot:emptyGrid', ...
        ['the alt grid is empty, which can only happen if bookNominal() was ' ...
         'edited.  An empty grid runs nothing and prints a report that looks ' ...
         'complete.']);
end
if cfg.tf <= 0
    error('mgs:zarchan:threeLoopAutopilot:tf', 'tf = %g s is not a run.', cfg.tf);
end
if cfg.h <= 0 || cfg.h > cfg.tf/10
    error('mgs:zarchan:threeLoopAutopilot:step', ...
        'h = %g s is not a usable step for tf = %g s.', cfg.h, cfg.tf);
end

% Heun on the actuator's own poles.  lambda = -zact*wact +/- j*wact*sqrt(1-z^2)
% has |lambda| = wact, and RK2 goes unstable at |lambda|*h of order 2.  The
% failure is silent and total: the states grow without bound and the step
% response comes back as very large FINITE numbers.  The book runs wact = 150,
% h = 1e-4 (product 0.015), a factor of 30 inside this bound.
if cfg.wact*cfg.h > 0.5
    error('mgs:zarchan:threeLoopAutopilot:actuatorStep', ...
        ['wact*h = %.3g.  The 2nd-order actuator is the fastest mode in the ' ...
         'loop and this two-pass (Heun) integrator goes unstable on it near ' ...
         'wact*h ~ 2, returning huge FINITE numbers rather than an error.  ' ...
         'The book runs wact = 150, h = 1e-4 (product 0.015).  Reduce h or ' ...
         'lower wact.'], cfg.wact*cfg.h);
end

% THE BOOK'S OUTPUT GRID MUST STILL BE THE BOOK'S OUTPUT GRID.  The listing
% samples with `S = S+H; if S >= .0099999, S = 0.`, which lands on multiples of
% 0.01 s only when 0.01/h is a whole number.  This is not a rounding detail: at
% h = 3e-3 the sampler fires every 0.012 s instead, and the book-grid t63 --
% the reading that reproduces the published 0.34 s at all three altitudes --
% drifts to 0.348 s at sea level and 0.336 s at 50 kft.  It stops being
% constant AND stops matching, for a reason that has nothing to do with the
% autopilot.  Refuse rather than report a grid artefact as a time constant.
q = BOOK_GRID/cfg.h;
if abs(q - round(q)) > 1e-9
    error('mgs:zarchan:threeLoopAutopilot:outputGrid', ...
        ['h = %g s does not divide the listing''s %g s output interval ' ...
         '(0.01/h = %.6f, not a whole number).  Listing 23.1 samples its ' ...
         'output arrays with "S = S+H; if S >= .0099999, S = 0.", so a ' ...
         'non-dividing h moves every plotted sample off the 0.01 s grid and ' ...
         'the book-grid t63 becomes an artefact of h rather than a property ' ...
         'of the autopilot.  Use 1e-4 (the book''s), 5e-4, 1e-3, 2e-3 or any ' ...
         'other exact divisor.'], cfg.h, BOOK_GRID, q);
end

if cfg.zact <= 0 || cfg.wact <= 0
    error('mgs:zarchan:threeLoopAutopilot:actuator', ...
        ['the actuator is 1/(1 + 2*zact*s/wact + s^2/wact^2); wact = %g and ' ...
         'zact = %g must both be positive or it is not a lag at all.'], ...
        cfg.wact, cfg.zact);
end
if cfg.xncg == 0
    % Everything in this file is measured as a fraction of the command.  A zero
    % command makes t63 the 63% point of nothing and dc_gain a 0/0.
    error('mgs:zarchan:threeLoopAutopilot:zeroCommand', ...
        ['xncg = 0 g.  It is BOTH the step command and the trim point the ' ...
         'aero is linearized about, so a zero command leaves t63 and the DC ' ...
         'gain undefined rather than merely uninteresting.']);
end
end


% =============================================================================
function F = airframe(ALT, cfg, GRAV_FT)
%AIRFRAME  The aerodynamic + autopilot-gain preamble of Listing 23.1,
%   transcribed.  Byte-identical text appears in Listings 23.2 and 23.3, and
%   NEIGHBOURCROSSCHECK() below proves this copy still agrees with the one
%   inside adjointWithAutopilot.m to a relative 1e-12.
%
%   Geometry (the Ch.21/22 airframe) is NOT exposed as options.  It is one
%   specific missile -- 20 ft long, 1 ft diameter, cruciform, tail control --
%   and the published numbers are its numbers.  Making it adjustable would
%   invite a run that is neither the book's airframe nor ours.
%
%   TWO THINGS IN HERE LOOK LIKE BUGS AND ARE NOT:
%
%   * A = 1000 ft/s at EVERY altitude.  The real speed of sound is nearer
%     995 ft/s at 30 kft and 968 ft/s at 50 kft, so Mach is off by a few per
%     cent up high.  Zarchan uses the flat 1000 anyway and the published Table
%     23.2 is what that produces.  "Fixing" it moves Ma/Md/Za/Zd off the book
%     and every benchmark below goes red for a correct reason, which is worse
%     than a documented approximation.
%   * The trim solve uses XNCG.  Here -- unlike in the adjoint of Listing 23.3,
%     where nothing is commanded to 10 g -- that is genuinely the same 10 g as
%     the step command, and the two uses are consistent: the aero is linearized
%     about the manoeuvre being commanded.  It is still two roles for one
%     symbol, so moving xncg moves the AIRFRAME as well as the command, and the
%     benchmark rows that depend on the aero are gated on xncg for that reason.

FR=3.; DIAM=1.; XL=20.; CTW=0.; CRW=6.; HW=2.; CTT=0.; CRT=2.; HT=2.;
XN=4.; XCG=10.; XHL=19.5; A=1000.;
% XNLLIN=0. and XACC=XCG appear in the listing and are never read: leftovers
% from the Ch.22 nonlinear simulation.  Not carried.  XK2=XK1 likewise.

VM = cfg.vm; WGT = cfg.wgt; XNCG = cfg.xncg;
TAU = cfg.tau; WCR = cfg.wcr; ZETA = cfg.zeta;

if ALT <= 30000.
    RHO=.002378*exp(-ALT/30000.);
else
    RHO=.0034*exp(-ALT/22000.);
end

SWING=.5*HW*(CTW+CRW); STAIL=.5*HT*(CTT+CRT); SREF=3.1416*DIAM*DIAM/4.;
XLP=FR*DIAM; SPLAN=(XL-XLP)*DIAM+1.33*XLP*DIAM/2.; XCPN=2*XLP/3;
AN=.67*XLP*DIAM; AB=(XL-XLP)*DIAM;
XCPB=(.67*AN*XLP+AB*(XLP+.5*(XL-XLP)))/(AN+AB);
XCPW=XLP+XN+.7*CRW-.2*CTW; XMACH=VM/A;
XIYY=WGT*(3*((DIAM/2)^2)+XL*XL)/(12*GRAV_FT);
TMP1=(XCG-XCPW)/DIAM; TMP2=(XCG-XHL)/DIAM;
TMP3=(XCG-XCPB)/DIAM; TMP4=(XCG-XCPN)/DIAM;

if XMACH <= 1
    % B = sqrt(M^2-1) is the Prandtl-Glauert factor and it is IMAGINARY below
    % Mach 1.  MATLAB would hand back complex Ma/Md/Za/Zd and the whole file
    % would keep running on complex numbers, printing real-looking magnitudes.
    error('mgs:zarchan:threeLoopAutopilot:subsonic', ...
        ['vm = %g ft/s is Mach %.3g against the book''s fixed a = 1000 ft/s. ' ...
         'This linearized aero is SUPERSONIC-ONLY: every derivative is built ' ...
         'on B = sqrt(M^2-1), which is imaginary here.  Ch.22 has no subsonic ' ...
         'branch to fall back to.'], VM, XMACH);
end
B=sqrt(XMACH^2-1); Q=.5*RHO*VM*VM;

P1=WGT*XNCG/(Q*SREF);
Y1=2+8*SWING/(B*SREF)+8*STAIL/(B*SREF);
Y2=1.5*SPLAN/SREF; Y3=8*STAIL/(B*SREF);
Y4=2*TMP4+8*SWING*TMP1/(B*SREF)+8*STAIL*TMP2/(B*SREF);
Y5=1.5*SPLAN*TMP3/SREF; Y6=8*STAIL*TMP2/(B*SREF);
P2=Y2-Y3*Y5/Y6; P3=Y1-Y3*Y4/Y6;
disc = P3*P3+4.*P2*P1;
if P2 == 0 || disc < 0
    error('mgs:zarchan:threeLoopAutopilot:trim', ...
        ['the %g-g trim solve has no real root at alt = %g ft (P2 = %g, ' ...
         'discriminant = %g).  The quadratic in Ch.22 assumes the body-lift ' ...
         'term P2 is non-zero and the manoeuvre attainable; an imaginary trim ' ...
         'angle would propagate silently into CNA and CMA.'], ...
        XNCG, ALT, P2, disc);
end
ALFTR=(-P3+sqrt(disc))/(2.*P2);
DELTR=-Y4*ALFTR/Y6-Y5*ALFTR*ALFTR/Y6;

CNA=2+1.5*SPLAN*ALFTR/SREF+8*SWING/(B*SREF)+8*STAIL/(B*SREF);
CND=8*STAIL/(B*SREF);
CMAP=2*TMP4+1.5*SPLAN*ALFTR*TMP3/SREF+8*SWING*TMP1/(B*SREF);
CMA=CMAP+8*STAIL*TMP2/(B*SREF); CMD=8*STAIL*TMP2/(B*SREF);
XMA=Q*SREF*DIAM*CMA/XIYY; XMD=Q*SREF*DIAM*CMD/XIYY;
ZA=-GRAV_FT*Q*SREF*CNA/(WGT*VM); ZD=-GRAV_FT*Q*SREF*CND/(WGT*VM);

if XMA >= 0
    error('mgs:zarchan:threeLoopAutopilot:unstable', ...
        ['Ma = %g >= 0 at alt = %g ft: this airframe is statically UNSTABLE ' ...
         'there and wAF = sqrt(-Ma) is imaginary.  The whole three-loop gain ' ...
         'algorithm of Ch.23 is derived for a stable airframe (wAF real); it ' ...
         'has no unstable branch.'], XMA, ALT);
end
if ZD == 0 || (XMA*ZD-ZA*XMD)/ZD < 0
    error('mgs:zarchan:threeLoopAutopilot:noZero', ...
        ['wZ^2 = (Ma*Zd - Za*Md)/Zd came out %g at alt = %g ft.  The airframe ' ...
         'right-half-plane zero is what produces the wrong-way effect and it ' ...
         'must be real; a complex wZ would silently make every acceleration ' ...
         'in the loop complex.'], (XMA*ZD-ZA*XMD)/(ZD+(ZD==0)*eps), ALT);
end
WZ=sqrt((XMA*ZD-ZA*XMD)/ZD); WAF=sqrt(-XMA); ZAF=.5*WAF*ZA/XMA;
% The 1845 is Zarchan's rad-to-deg-and-g bookkeeping constant (57.3*32.2), so
% K1 comes out in g/deg and K3 in deg/deg.  It is NOT a tuning number.
XK1=-VM*(XMA*ZD-XMD*ZA)/(1845*XMA);
TA=XMD/(XMA*ZD-XMD*ZA); XK3=1845*XK1/VM;

% ---- the gain algorithm (zarchan.txt:7396-7520, pp.533-536) --------------
W=(TAU*WCR*(1+2.*ZAF*WAF/WCR)-1)/(2*ZETA*TAU);
if W <= 0
    % tau*wcr*(1 + 2*zAF*wAF/wcr) must exceed 1 or the algorithm asks for a
    % negative closed-loop natural frequency.  Everything downstream (W0, Z0,
    % KC, KA, wI, KR) still EVALUATES and returns a set of gains that
    % stabilises nothing.  Book values give W = 35.4 rad/s.
    error('mgs:zarchan:threeLoopAutopilot:infeasibleGains', ...
        ['the autopilot gain algorithm has no solution for tau = %g s, ' ...
         'wcr = %g rad/s, zeta = %g at alt = %g ft: it needs ' ...
         'tau*wcr*(1 + 2*zAF*wAF/wcr) > 1 and got %.4g.  The formulae would ' ...
         'still return numbers -- they would just not be an autopilot.'], ...
        TAU, WCR, ZETA, ALT, TAU*WCR*(1+2.*ZAF*WAF/WCR));
end
W0=W/sqrt(TAU*WCR);
Z0=.5*W0*(2*ZETA/W+TAU-WAF^2/(W0*W0*WCR));
XKC=(-W0^2/WZ^2-1.+2.*Z0*W0*TA)/(1.-2.*Z0*W0*TA+W0*W0*TA*TA);
XKA=XK3/(XK1*XKC); XK0=-W*W/(TAU*WAF*WAF);
XK=XK0/(XK1*(1+XKC));
WI=XKC*TA*W0*W0/(1+XKC+W0^2/WZ^2);
XKR=XK/(XKA*WI); XKDC=1.+1845./(XKA*VM);

gains = [W W0 Z0 XKC XKA XK0 XK WI XKR XKDC];
if ~all(isfinite(gains)) || ~isreal(gains)
    error('mgs:zarchan:threeLoopAutopilot:gains', ...
        ['the gain algorithm produced a non-finite or complex gain at ' ...
         'alt = %g ft ([W W0 Z0 KC KA K0 K wI KR KDC] = %s).  A division in ' ...
         'the chain went to zero -- most often KC or wI.  Refusing to fly an ' ...
         'autopilot made of Inf.'], ALT, mat2str(gains, 4));
end

F = struct();
F.alt = ALT; F.rho = RHO; F.q = Q; F.mach = XMACH;
F.trim = struct('alftr_deg', ALFTR, 'deltr_deg', DELTR, 'xncg_g', XNCG);
F.xma = XMA; F.xmd = XMD; F.za = ZA; F.zd = ZD;
F.wz = WZ; F.waf = WAF; F.zaf = ZAF;
F.xk1 = XK1; F.ta = TA; F.xk3 = XK3;
F.w = W; F.w0 = W0; F.z0 = Z0; F.xkc = XKC; F.xk0 = XK0; F.xk = XK;
F.xka = XKA; F.wi = WI; F.xkr = XKR; F.xkdc = XKDC;
% zarchan.txt:7535 (p.538): the overall time constant of the third-order
% flight-control system.  THIS is the derived lag, in closed form.
F.tau_total = TAU + 2*ZETA/W;
F.step = [];
end


% =============================================================================
function P = stepResponse(F, cfg, BOOK_GRID)
%STEPRESPONSE  Listing 23.1's five-state closed-loop step response, plus the
%   listing's own 0.01 s output sampler.  Transcribed (zarchan.txt:7551-7600).
%
%   STATES, in the book's names:
%     E, ED      airframe: angle of attack and its rate (second order, wAF/zAF)
%     DEL, DELD  actuator: fin deflection and fin rate (second order, wACT/zACT)
%     X          the autopilot integrator
%   and the algebraic outputs
%     THD        body rate feedback     = K3*(E + Ta*ED)
%     DELC       commanded fin          = KR*(X + THD)
%     XNL        achieved acceleration  = K1*(E - EDD/wZ^2)   <- the RHP zero
%
%   ORDER MATTERS INSIDE THE PASS.  XNL uses EDD, which is computed one line
%   above it from DEL.  Move the XNL line up and the accelerometer feeds back
%   the PREVIOUS step's angular acceleration, which reads as a small extra lag
%   and is nearly invisible in the answer.  The book's order is kept exactly.
%
%   WHAT t63 IS MEASURED AGAINST, AND THE TWO TRAPS IN IT.
%
%   The closed-loop DC gain is EXACT ALGEBRA, not the last sample.  At steady
%   state ED = EDD = DELD = DELDD = XD = 0, so DEL = E, DELC = DEL,
%   THD = K3*E and nL = K1*E, and XD = 0 leaves
%
%       E_ss = KA*XNCG*KDC / (K3 + KA*K1),      nL_ss = K1*E_ss
%
%   Substituting K3 = 1845*K1/VM, KA = K3/(K1*KC) and KDC = 1 + 1845/(KA*VM)
%   collapses that to nL_ss/XNCG = KDC/(1+KC) = 1 EXACTLY -- which is what KDC
%   is for.  The actuator does not change it either: a second-order actuator
%   has unity DC gain too.  The self-check below tests this identity at 1e-9
%   and it is the sharpest thing in the file, because it exercises the whole
%   chain K1 -> K3 -> KC -> KA -> KDC in one number.
%
%   TRAP 1: nl_g(end) IS NOT THE STEADY STATE.  Listing 23.1 stops at TF = 1 s,
%   which is only about three time constants, so the last sample is 96% of
%   steady state at tau = 0.3 s and 86% at tau = 0.5 s -- STILL RISING.
%   Reading "63% of the steady-state value" off that last sample gives 0.318 s
%   instead of 0.34 s and quietly shortens the measured lag by 6%.  Both t63
%   readings below are therefore measured against the ANALYTIC steady state.
%   The mis-reading is kept as t63_vs_last_sample_s ONLY so that whoever
%   reproduces 0.318 s can find out where it came from.
%
%   TRAP 2: WHICH GRID.  See the file header.  t63_s is read off the BOOK'S
%   0.01 s output array (the one that becomes Figs. 23.3 and 23.12, and the one
%   that gives the published 0.34 s at all three altitudes); t63_fullrate_s is
%   read off the integration grid and is about 1% shorter.  Reporting one as
%   the other is how "0.34" and "0.3367" get argued about.

XK1=F.xk1; XK3=F.xk3; TA=F.ta; WAF=F.waf; ZAF=F.zaf; WZ2=F.wz*F.wz;
XKR=F.xkr; XKA=F.xka; WI=F.wi; XKDC=F.xkdc;
WACT=cfg.wact; ZACT=cfg.zact; XNCG=cfg.xncg; H=cfg.h; TF=cfg.tf;

E=0.; ED=0.; DELD=0.; DEL=0.; X=0.; T=0.; S=0.;
EDD=0.; DELDD=0.; XD=0.; XNL=0.;

n = ceil(TF/H)+2;              % full integration grid
tA=zeros(1,n); nlA=zeros(1,n); frA=zeros(1,n); dlA=zeros(1,n); k=0;
m = ceil(TF/BOOK_GRID)+2;      % the listing's own 0.01 s output arrays
tB=zeros(1,m); nlB=zeros(1,m); frB=zeros(1,m); ncB=zeros(1,m); j=0;

while T <= (TF-.00001)
    EOLD=E; EDOLD=ED; DELOLD=DEL; DELDOLD=DELD; XOLD=X;
    for pass = 1:2                       % the book's STEP/FLAG inner loop
        if pass == 2
            E=E+H*ED; ED=ED+H*EDD; DEL=DEL+H*DELD; DELD=DELD+H*DELDD;
            X=X+H*XD; T=T+H;
        end
        THD=XK3*(E+TA*ED);
        DELC=XKR*(X+THD);
        DELDD=WACT*WACT*(DELC-DEL-2.*ZACT*DELD/WACT);
        EDD=WAF*WAF*(DEL-E-2.*ZAF*ED/WAF);
        XNL=XK1*(E-EDD/WZ2);
        XD=WI*(THD+XKA*(XNL-XNCG*XKDC));
    end
    E=.5*(EOLD+E+H*ED); ED=.5*(EDOLD+ED+H*EDD);
    DEL=.5*(DELOLD+DEL+H*DELD); DELD=.5*(DELDOLD+DELD+H*DELDD);
    X=.5*(XOLD+X+H*XD);

    k=k+1; tA(k)=T; nlA(k)=XNL; frA(k)=DELD; dlA(k)=DEL;

    % The listing's output sampler, transcribed.  The threshold is .0099999
    % and NOT .01 on purpose: S is a running sum of H and after 100 additions
    % of 1e-4 it lands a few ulp either side of 0.01, so a test against 0.01
    % exactly would drop or duplicate samples depending on rounding.  The
    % 1e-9 of slack is what makes the grid reproducible.
    S = S + H;
    if S >= .0099999
        S = 0.;
        j=j+1; tB(j)=T; nlB(j)=XNL; frB(j)=DELD; ncB(j)=XNCG;
    end
end

if k < 10
    error('mgs:zarchan:threeLoopAutopilot:stepSamples', ...
        'the step response produced %d integration samples; h = %g is too coarse.', k, H);
end
if j < 3
    error('mgs:zarchan:threeLoopAutopilot:bookSamples', ...
        ['the listing''s 0.01 s output sampler produced only %d samples over ' ...
         'tf = %g s.  Every published reading in this chapter is taken off ' ...
         'that grid, so there is nothing to compare the book against.'], j, TF);
end
tA=tA(1:k); nlA=nlA(1:k); frA=frA(1:k); dlA=dlA(1:k);
tB=tB(1:j); nlB=nlB(1:j); frB=frB(1:j); ncB=ncB(1:j);

if ~all(isfinite(nlA)) || ~all(isfinite(frA))
    error('mgs:zarchan:threeLoopAutopilot:stepDiverged', ...
        ['the step response went non-finite at alt = %g ft after %d steps.  ' ...
         'Two causes needing different fixes: the INTEGRATOR is unstable ' ...
         '(check wact*h = %.3g), or the CLOSED LOOP is unstable at these ' ...
         'design goals -- Ch.23 shows the loop going bad when the crossover ' ...
         'frequency approaches the actuator bandwidth, which is a RESULT ' ...
         'rather than a numerical failure, but at the book''s parameters it ' ...
         'does not reach Inf inside 1 s.'], F.alt, k, WACT*H);
end

den = XK3 + XKA*XK1;
if den == 0 || ~isfinite(den)
    error('mgs:zarchan:threeLoopAutopilot:noSteadyState', ...
        ['K3 + KA*K1 = %g at alt = %g ft, so the closed loop has no finite ' ...
         'steady state and t63 has nothing to be measured against.  Every lag ' ...
         'number in this file is defined relative to that steady state.'], ...
        den, F.alt);
end
ss = XK1*(XKA*XNCG*XKDC/den);      % ANALYTIC steady state; == XNCG exactly
if ~isfinite(ss) || ss/XNCG <= 0
    % A steady state of the wrong sign would make 0.63*ss unreachable in the
    % commanded direction, FIND would match the very first sample or none at
    % all, and t63 would come back as one time step -- a lag of 0.0001 s,
    % printed with a straight face.
    error('mgs:zarchan:threeLoopAutopilot:badSteadyState', ...
        ['the closed-loop steady state came out %g g for a %+g g command at ' ...
         'alt = %g ft.  The gain algorithm''s unity-DC-gain identity has ' ...
         'broken down, so there is nothing for t63 to be 63%% of.'], ...
        ss, XNCG, F.alt);
end

% Signed threshold so a negative command works too: 0.63*ss is 63% of the way
% toward the steady state in whichever direction it lies.
if ss > 0
    iB = find(nlB >= 0.63*ss,       1);
    iA = find(nlA >= 0.63*ss,       1);
    jA = find(nlA >= 0.63*nlA(end), 1);
else
    iB = find(nlB <= 0.63*ss,       1);
    iA = find(nlA <= 0.63*ss,       1);
    jA = find(nlA <= 0.63*nlA(end), 1);
end

[~, ipkA] = max(abs(frA));      % peak fin rate, integration grid
[~, ipkB] = max(abs(frB));      % peak fin rate as the book would have plotted it
[wwy, iww] = min(nlA*sign(ss)); % wrong-way (RHP zero) excursion, signed
wwy = wwy*sign(ss);
[~, ipeak] = max(nlA*sign(ss));

% Ringing, measured on the BOOK'S grid because that is the curve the book calls
% "oscillatory" or "well behaved".  Reported on both so the metric cannot be
% accused of being an artefact of decimation -- they agree (0/0, 32/32, 0/0,
% 20/18 across the four studies).
[nrevB, ampB] = ringing(tB, nlB, 0.05);
[nrevA, ampA] = ringing(tA, nlA, 0.05);

P = struct();
P.t = tA; P.nl_g = nlA; P.finrate_dps = frA; P.del_deg = dlA;
P.tb = tB; P.nl_book_g = nlB; P.finrate_book_dps = frB; P.nc_book_g = ncB;
P.book_grid_s   = BOOK_GRID;
P.xncg_g        = XNCG;
P.ss_g          = ss;                  % analytic, NOT nl_g(end)
P.dc_gain       = ss/XNCG;             % must be 1 to ~1e-15; self-checked
P.nl_at_tf_g    = nlA(end);
P.settled_frac  = nlA(end)/ss;         % ~0.96 at tau = 0.3 s, TF = 1 s
P.t63_s          = pick(tB, iB);       % BOOK grid -- what "0.34 s" means
P.t63_fullrate_s = pick(tA, iA);       % integration grid -- what 23.3 returns
P.t63_vs_last_sample_s = pick(tA, jA); % the 6%-short mis-reading, kept on purpose
P.peak_finrate_dps      = frA(ipkA);
P.peak_finrate_book_dps = frB(ipkB);
P.t_peak_finrate_s      = tA(ipkA);
P.wrongway_g   = wwy;      % opposite sign to the command: the RHP zero
P.t_wrongway_s = tA(iww);
P.peak_g       = nlA(ipeak);
P.t_peak_s     = tA(ipeak);
P.nrev         = nrevB;    % direction reversals after 0.05 s, BOOK grid
P.ripple_g     = ampB;     % largest peak-to-trough among them, BOOK grid
P.nrev_full    = nrevA;
P.ripple_full_g = ampA;
P.nsteps       = k;
P.nbook        = j;

if isnan(P.t63_s)
    % NaN, not the horizon.  Returning TF would read as "the lag is 1 s",
    % which is a claim, and a false one.
    warning('mgs:zarchan:threeLoopAutopilot:noT63', ...
        ['the response never reached 63%% of its %g g steady state inside ' ...
         'tf = %g s at alt = %g ft, so t63 is NaN rather than a number.  The ' ...
         'closed-form lag tau + 2*zeta/W = %.4f s says it needs longer.'], ...
        ss, TF, F.alt, F.tau_total);
end
if P.settled_frac < 0.7
    % Below this the 63% point is still on the steep part but the run says
    % almost nothing about settling, and anyone reading nl_at_tf_g as a steady
    % state would be out by more than 30%.
    warning('mgs:zarchan:threeLoopAutopilot:notSettled', ...
        ['the step response reached only %.0f%% of steady state by tf = %g s ' ...
         '(alt = %g ft, tau = %g s).  t63 is still measured against the ' ...
         'ANALYTIC steady state and is fine, but nl_at_tf_g is not a settled ' ...
         'value.'], 100*P.settled_frac, TF, F.alt, cfg.tau);
end
end


function v = pick(t, i)
%PICK  t(i), or NaN if FIND returned empty.  Never the horizon.
if isempty(i), v = NaN; else, v = t(i); end
end


% =============================================================================
function [nrev, amp] = ringing(t, y, tStart)
%RINGING  Turn the book's adjective into a measurement.
%
%   Counts direction reversals of y after tStart and returns the largest
%   peak-to-trough excursion among them.  A well-damped rise to a steady state
%   is MONOTONIC and scores 0; a loop near its stability margin rings and
%   scores tens.
%
%   WHY NOT OVERSHOOT.  The obvious metric -- percent overshoot past steady
%   state -- is USELESS here and would have produced a false negative on both
%   of the book's "oscillatory" cases.  Listing 23.1 stops at TF = 1 s with the
%   response still at ~96% of steady state, so nl NEVER crosses ss and the
%   overshoot is negative for every case including the ones the book calls
%   oscillatory.  Measured: 0.00% "overshoot" at wcr = 100 rad/s, which the
%   book explicitly calls oscillatory (zarchan.txt:7653), against 32 reversals
%   and 2.13 g of ripple on the same run.  Reversals see the ringing riding on
%   the rise; overshoot cannot.
%
%   tStart excludes the wrong-way dip, which is a real feature of the RHP zero
%   and not ringing.  The dip is over by 0.037 s at every flight condition and
%   in every study in this chapter; 0.05 s clears it with margin.
i0 = find(t >= tStart, 1);
if isempty(i0), i0 = 1; end
nrev = 0; amp = 0; prev = 0; lastExt = NaN;
for i = i0+1:numel(y)
    d = y(i) - y(i-1);
    if d > 0, s = 1; elseif d < 0, s = -1; else, continue; end
    if prev ~= 0 && s ~= prev
        nrev = nrev + 1;
        if ~isnan(lastExt), amp = max(amp, abs(y(i-1) - lastExt)); end
        lastExt = y(i-1);
    end
    prev = s;
end
end


% =============================================================================
function L = lagSummary(FC, cfg)
%LAGSUMMARY  The number this project came for, in one place, on both grids.
L = struct();
L.tau_design_s   = cfg.tau;
L.alt_ft         = [FC.alt];
L.w_rad_s        = [FC.w];
L.tau_total_s    = [FC.tau_total];                          % tau + 2*zeta/W
L.t63_s          = arrayfun(@(f) f.step.t63_s, FC);         % BOOK 0.01 s grid
L.t63_fullrate_s = arrayfun(@(f) f.step.t63_fullrate_s, FC);
L.dc_gain        = arrayfun(@(f) f.step.dc_gain, FC);       % analytic; must be 1
L.settled_frac   = arrayfun(@(f) f.step.settled_frac, FC);
% The mis-reading, carried so it can be recognised rather than rediscovered.
L.t63_vs_last_sample_s = arrayfun(@(f) f.step.t63_vs_last_sample_s, FC);
L.note = [ ...
 'tau_total is the p.538 closed form tau + 2*zeta/W.  t63_s is read off the ' ...
 'listing''s OWN 0.01 s output array -- the grid Figs. 23.3 and 23.12 are ' ...
 'drawn from -- against the ANALYTIC steady state, and it is 0.34 s at every ' ...
 'altitude, which is the published number exactly.  t63_fullrate_s reads the ' ...
 'same crossing off the integration grid and is ~1% shorter; that is the ' ...
 'number mgs.zarchan.adjointWithAutopilot returns.  Three independent ' ...
 'computations of one lag.  The transferable result is that the gain ' ...
 'algorithm HOLDS this lag constant while the airframe changes by 6x over ' ...
 'the altitude sweep -- our own ~8 s gamma-tracking lag is held constant by ' ...
 'nothing, and that is the gap, not the size of the number.'];
end


% =============================================================================
function V = variantStudies(cfg, GRAV_FT, BOOK_GRID)
%VARIANTSTUDIES  The four off-nominal runs Ch.23 makes a claim about.
%
%   Each is a separate run of the SAME listing at sea level with one or two
%   design goals moved, which is exactly what the book did.  They are computed
%   from cfg, not from the book defaults, so a caller who has already moved vm
%   or wgt gets variants of THEIR airframe -- and the benchmark rows that quote
%   the book are gated off in that case, so nothing is compared to the wrong
%   system.
%
%   WHY THESE FOUR AND NOT A SWEEP.  Every one corresponds to a sentence in the
%   text with a figure number attached.  A finer sweep would produce curves the
%   book never published and could not check, and the temptation would then be
%   to score a shape read off our own plot.
V = struct('name',{},'claim',{},'source',{},'over',{},'fc',{},'step',{});

C = {
 'tau = 0.1 s', ...
   'gains become KA = 3.72 deg/g-s, wI = 11.2 rad/s, KR = 0.098 s', ...
   'Fig. 23.5 text, p.542 (zarchan.txt:7623)', ...
   struct('tau', 0.1)
 'wcr = 100 rad/s (wact 150)', ...
   'has an oscillatory response', ...
   'Fig. 23.8 text, p.546 (zarchan.txt:7653)', ...
   struct('wcr', 100)
 'wcr = 100, wact = 300 rad/s', ...
   'the step response stabilizes and is well behaved', ...
   'Fig. 23.9 text, p.546 (zarchan.txt:7654)', ...
   struct('wcr', 100, 'wact', 300)
 'zeta = 0.4', ...
   'the system step response becomes oscillatory', ...
   'Fig. 23.11 text, p.547 (zarchan.txt:7675)', ...
   struct('zeta', 0.4)
};

for r = 1:size(C,1)
    c = cfg;
    f = fieldnames(C{r,4});
    for q = 1:numel(f), c.(f{q}) = C{r,4}.(f{q}); end
    % The actuator-stability guard applies to the variants too, and wact = 300
    % halves the margin.  Check here rather than letting a variant return
    % large finite garbage that then drives a shape row.
    if c.wact*c.h > 0.5
        error('mgs:zarchan:threeLoopAutopilot:variantStep', ...
            ['study "%s" needs wact = %g rad/s, and wact*h = %.3g exceeds the ' ...
             'Heun stability bound at h = %g s.  Reduce h before running the ' ...
             'benchmark board.'], C{r,1}, c.wact, c.wact*c.h, c.h);
    end
    Fv = airframe(0, c, GRAV_FT);
    Sv = stepResponse(Fv, c, BOOK_GRID);
    V(r).name   = C{r,1};
    V(r).claim  = C{r,2};
    V(r).source = C{r,3};
    V(r).over   = C{r,4};
    V(r).fc     = Fv;
    V(r).step   = Sv;
end
end


% =============================================================================
function X = neighbourCrossCheck(FC, cfg)
%NEIGHBOURCROSSCHECK  Prove the duplicated preamble has not drifted.
%
%   Listings 23.1 and 23.3 share a byte-identical aerodynamic and gain
%   algorithm, and both are ported.  MATLAB gives no way to call the local
%   AIRFRAME/FCSSTEP of adjointWithAutopilot.m from here, so the copy cannot be
%   shared -- it can only be CHECKED.  This runs the neighbour at the same
%   flight conditions and the same design goals as this run and compares every
%   aero and gain quantity at a relative 1e-12.
%
%   THIS IS THE ONE ROW THAT STILL BITES OFF-BOOK.  Every benchmark row in this
%   file is gated on the caller staying at the book's parameters; this one is
%   not, because the two implementations must agree at ANY parameters.  It is
%   also a delete-experiment in the sense this repo has learned to insist on:
%   comment out one line of either preamble and the row names the quantity that
%   moved, rather than a comment somewhere claiming the files agree.
%
%   THE ADJOINT IS MADE CHEAP, DELIBERATELY AND VISIBLY.  The neighbour is
%   called with 'radome', 0 (one slope instead of thirteen) and 'tf', 0.5 (a
%   short adjoint horizon).  Neither touches anything compared here: the gains
%   come from its AIRFRAME, which does not read tf or the radome grid at all,
%   and its FCSSTEP hard-codes its own TF = 1 s.  That last fact is an
%   assumption about someone else's file, so it is VERIFIED below rather than
%   trusted -- if the neighbour's step response ever starts following its tf,
%   the t63 comparison is skipped with the reason instead of comparing a 1 s
%   response against a 0.5 s one and calling the difference drift.
%
%   FIELDS COMPARED: the four Table 23.2 aero derivatives, the three airframe
%   quantities derived from them, the three transfer-function gains, the ten
%   algorithm gains, the closed-form lag, and the full-rate t63.
%
%   THE DIRECTION OF THE CALL IS LOAD-BEARING.  This file calls the neighbour;
%   the neighbour must never call this one.  It does not today (it carries its
%   own FCSSTEP copy of the step loop, which is exactly why this check exists),
%   but "reuse threeLoopAutopilot instead of FCSSTEP" is an obvious-looking
%   tidy-up and it would produce unbounded mutual recursion the first time
%   either is called with both defaults on -- not a stack overflow at the call
%   site, but two files that each look correct in isolation.  If FCSSTEP is
%   ever replaced by a call to this function, delete this cross-check in the
%   same edit; there is nothing left to compare once the copy is gone.

X = struct('ran', false, 'ok', false, 'nchecked', 0, 'worst_rel', NaN, ...
           'worst_name', '', 't63_delta_s', NaN, 'why', '', 'alt', [FC.alt], ...
           'tol', 1e-12, 'detail', '');

if isempty(which('mgs.zarchan.adjointWithAutopilot'))
    X.why = ['mgs.zarchan.adjointWithAutopilot is not on the path, so the ' ...
             'duplicated Ch.23 preamble could not be compared.  This port is ' ...
             'unaffected; what is missing is the proof that the OTHER copy ' ...
             'still agrees with it.'];
    return
end

try
    A = mgs.zarchan.adjointWithAutopilot( ...
        'alt', cfg.alt, 'radome', 0, 'tf', 0.5, ...
        'tau', cfg.tau, 'wcr', cfg.wcr, 'zeta', cfg.zeta, ...
        'wact', cfg.wact, 'zact', cfg.zact, 'xncg', cfg.xncg, ...
        'vm', cfg.vm, 'wgt', cfg.wgt, 'h', cfg.h, ...
        'benchmark', false, 'assert', false, 'render', false, 'plot', false);
catch ME
    % The neighbour has guards of its own (subsonic, unstable airframe,
    % infeasible gains).  If they fire, that is information, not a crash of
    % this port -- but it must be REPORTED, not swallowed.
    X.why = sprintf(['adjointWithAutopilot errored (%s: %s), so the ' ...
        'duplicated preamble was not compared.  Its guards cover the same ' ...
        'ground as ours, so this usually means the parameters are outside ' ...
        'what either file models.'], ME.identifier, oneLine(ME.message));
    return
end

if ~isfield(A, 'fc') || numel(A.fc) ~= numel(FC)
    X.why = sprintf(['adjointWithAutopilot returned %d flight conditions for ' ...
        'the %d asked for, so the columns could not be paired.'], ...
        numel(A.fc), numel(FC));
    return
end

names = {'xma','xmd','za','zd','wz','waf','zaf','xk1','ta','xk3', ...
         'w','w0','z0','xkc','xk0','xk','xka','wi','xkr','xkdc','tau_total'};
worst = 0; worstName = ''; nchk = 0;
for i = 1:numel(FC)
    if abs(A.fc(i).alt - FC(i).alt) > 1e-9
        X.why = sprintf(['altitude column %d came back as %g ft instead of ' ...
            '%g ft; refusing to compare mismatched flight conditions.'], ...
            i, A.fc(i).alt, FC(i).alt);
        return
    end
    for q = 1:numel(names)
        a = A.fc(i).(names{q});  b = FC(i).(names{q});
        d = abs(a-b) / max(abs(b), realmin);
        nchk = nchk + 1;
        if d > worst, worst = d; worstName = sprintf('%s @ %g ft', names{q}, FC(i).alt); end
    end
end
X.nchecked   = nchk;
X.worst_rel  = worst;
X.worst_name = worstName;

% The full-rate t63 is a GRID INDEX, so the smallest disagreement it can
% express is one sample of h.  A relative 1e-12 on it would flag an ulp-level
% difference in the last arithmetic op as drift; one sample is the honest
% resolution, and anything larger is real.
t63ok = true; dmax = 0;
tfStep = arrayfun(@(f) f.step.t(end), A.fc);
if any(abs(tfStep - 1.0) > 1e-6)
    X.detail = sprintf(['t63 NOT compared: the neighbour''s step response now ' ...
        'runs to %.4g s, not the 1 s Listing 23.1 specifies, so its t63 is ' ...
        'no longer the same quantity.  The gains above were still compared.'], ...
        tfStep(1));
elseif abs(cfg.tf - 1.0) > 1e-9
    X.detail = sprintf(['t63 NOT compared: this run has tf = %g s while the ' ...
        'neighbour''s step response is fixed at 1 s.  The gains above were ' ...
        'still compared.'], cfg.tf);
else
    for i = 1:numel(FC)
        a = A.fc(i).step.t63_s;            % the neighbour reads the FULL rate
        b = FC(i).step.t63_fullrate_s;     % ours, same definition, same grid
        if ~isfinite(a) || ~isfinite(b)
            t63ok = false; dmax = Inf;
        else
            dmax = max(dmax, abs(a-b));
        end
    end
    t63ok = t63ok && dmax <= 1.5*cfg.h;
    X.t63_delta_s = dmax;
    X.detail = sprintf(['t63 compared against the neighbour''s FULL-RATE ' ...
        'reading (%.4f s at %g ft), not against our book-grid %.4f s -- the ' ...
        'two files read the same crossing off different grids on purpose.'], ...
        A.fc(1).step.t63_s, FC(1).alt, FC(1).step.t63_s);
end

X.ran = true;
X.ok  = (worst <= X.tol) && t63ok;
if ~X.ok
    X.why = sprintf(['the two Ch.23 ports no longer agree: worst relative ' ...
        'difference %.3e on %s (tolerance %.0e), t63 gap %.3g s.  One of the ' ...
        'duplicated preambles in threeLoopAutopilot.m or ' ...
        'adjointWithAutopilot.m has been edited.'], worst, worstName, X.tol, dmax);
end
end


% =============================================================================
function C = structuralCheck(cfg, GRAV_FT, BOOK_GRID)
%STRUCTURALCHECK  Invariants that must hold or every number above is garbage.
%
%   These are NOT book values.  They test the WIRING, and unlike the benchmarks
%   they still bite when the caller runs off-book.  This function makes its OWN
%   runs at its OWN flight conditions so that it checks the same things no
%   matter what the caller asked for.
%
%   1. UNITY DC GAIN, AS EXACT ALGEBRA.  Substituting the gain algorithm into
%      the closed-loop steady state gives nL_ss/XNCG = KDC/(1+KC), and the
%      algorithm's own KDC = 1 + 1845/(KA*VM) is identically 1 + KC (because
%      KA = 1845/(VM*KC)).  So this ratio is 1 to floating-point round-off at
%      EVERY flight condition and every tau, and one number exercises the whole
%      chain K1 -> K3 -> KC -> KA -> KDC.  Tolerance 1e-9, not a few per cent:
%      it is an identity, not a design goal.  Note it does NOT touch KR or wI
%      -- check 2 is what exercises those.
%   2. THE LAG IS THE LAG.  The measured 63% rise time must agree with the
%      closed form tau + 2*zeta/W.  Two independent computations -- one an
%      integration through the actuator and the airframe, one three lines of
%      algebra -- of the number this file exists to produce.  KR and wI appear
%      only in the integration, so this is the check that they are right.
%   3. THE OUTPUT SAMPLER IS THE BOOK'S.  The 0.01 s grid must actually be a
%      0.01 s grid: right number of samples, uniform spacing.  If this fails,
%      the whole 0.34 s reading is an artefact and the guard in resolveConfig
%      did not catch it.
%   4. THE RHP ZERO IS WIRED IN.  nL = K1*(E - EDD/wZ^2); the second term is
%      the right-half-plane zero and it is the ONLY thing that can make the
%      acceleration go the wrong way first.  A transcription that drops it, or
%      flips its sign, still produces a plausible-looking rise to the right
%      steady state -- the DC gain check above would still pass, because the
%      zero contributes nothing at DC.  So the wrong-way excursion must be
%      present, must oppose the command, and must GROW with altitude as wZ
%      falls.  Checked between sea level and 50 kft, where wZ halves.

C = struct(); C.skipped = false;

F0  = airframe(0,     cfg, GRAV_FT);
F50 = airframe(50000, cfg, GRAV_FT);
S0  = stepResponse(F0,  cfg, BOOK_GRID);
S50 = stepResponse(F50, cfg, BOOK_GRID);

% -- 1. unity DC gain, as exact algebra
C.dc_gain = [S0.dc_gain S50.dc_gain];
C.dc_err  = max(abs(C.dc_gain - 1));
C.dc_tol  = 1e-9;
C.dc_ok   = C.dc_err <= C.dc_tol;

% -- 2. measured lag vs the closed-form lag (book grid, the primary reading)
C.lag_meas = [S0.t63_s S50.t63_s];
C.lag_form = [F0.tau_total F50.tau_total];
C.lag_rel  = max(abs(C.lag_meas - C.lag_form) ./ C.lag_form);
C.lag_tol  = 0.05;
C.lag_ok   = all(isfinite(C.lag_meas)) && C.lag_rel <= C.lag_tol;

% -- 3. the output sampler really is the book's 0.01 s grid
dt = diff(S0.tb);
C.grid_n       = S0.nbook;
C.grid_n_want  = floor(cfg.tf/BOOK_GRID);
C.grid_dt_err  = max(abs(dt - BOOK_GRID));
C.grid_tol     = 1e-9;
C.grid_ok      = C.grid_dt_err <= C.grid_tol && ...
                 abs(C.grid_n - C.grid_n_want) <= 1;

% -- 4. the right-half-plane zero is present, opposes, and grows with altitude
C.ww = [S0.wrongway_g S50.wrongway_g];
C.ww_opposes = all(C.ww * sign(cfg.xncg) < 0);
C.ww_grows   = abs(C.ww(2)) > abs(C.ww(1));
C.ww_ratio   = abs(C.ww(2))/max(abs(C.ww(1)), realmin);
C.ww_ok      = C.ww_opposes && C.ww_grows;

C.settled_frac = [S0.settled_frac S50.settled_frac];   % reported, not gated
C.ok = C.dc_ok && C.lag_ok && C.grid_ok && C.ww_ok;
if ~C.ok
    error('mgs:zarchan:threeLoopAutopilot:selfcheck', ...
        ['a STRUCTURAL invariant failed, so every number this function ' ...
         'computed is wrong:\n' ...
         '  unity DC gain (algebraic identity)     : %9.3e  tol %.0e   %s\n' ...
         '  measured t63 vs tau + 2*zeta/W         : %9.3e  tol %.0e   %s\n' ...
         '  output sampler is a %g s grid          : %9.3e  n = %d/%d  %s\n' ...
         '  RHP zero present, opposes, grows w/ alt: %9.3f x     %s\n' ...
         'These test the wiring, not the book.  A failure here means the ' ...
         'transcription of Listing 23.1 has been damaged.'], ...
        C.dc_err, C.dc_tol, tf2s(C.dc_ok), ...
        C.lag_rel, C.lag_tol, tf2s(C.lag_ok), ...
        BOOK_GRID, C.grid_dt_err, C.grid_n, C.grid_n_want, tf2s(C.grid_ok), ...
        C.ww_ratio, tf2s(C.ww_ok));
end
end


% =============================================================================
%   BOOK VALUES.  THE ONE PLACE a published number is written down.
% =============================================================================
function V = bookValues()
%BOOKVALUES  Every number in this file that came out of the book, with the
%   exact text it was grepped from.
%
%   RULE OBSERVED HERE, AT COST: a value that can be grepped out of zarchan.txt
%   is usable; a value that lives only on a figure does not exist.  The
%   extraction turned every figure into an image link, so NO number that was
%   only ever plotted survives.  Three "benchmarks" handed to an earlier wave
%   were figure-derived and all three were wrong, one contaminated from a
%   different chapter.  What the book says in PROSE about a figure ("has an
%   oscillatory response", "maintain the same step response") IS greppable and
%   IS used -- as shape rows, which are often sharper than a level read off a
%   plot.
%
%   Everything below was grepped from matlab/zarchan/_source/zarchan.txt at the
%   line quoted.

V = struct();

% -- Table 23.2, "LINEARIZED AERODYNAMIC PARAMETERS FOR DIFFERENT FLIGHT
%    CONDITIONS", p.551, zarchan.txt:7738-7742.  The OCR renders a minus sign
%    as '2' and an exponent as a trailing digit ("2642 s22" is -642 s^-2).
%       alt      Ma      Md      Za       Zd
V.aero = [ 0    -642    -555   -2.94   -0.65
        30000   -240    -204   -1.17   -0.239
        50000    -99.1   -81.7 -0.533  -0.0957 ];

% -- Table 23.1, "AUTOPILOT GAINS VARY WITH FLIGHT CONDITION", p.549,
%    zarchan.txt:7691-7695.  The sea-level row is ALSO quoted in prose at
%    zarchan.txt:7528 ("turn out to be KA = 1.15 deg/g-s, wI = 12.9 rad/s, and
%    KR = 0.0928 s"), and the two agree, so the table is cited as primary.
%       alt      KA[deg/g-s]  KR[s]    wI[rad/s]
V.gains = [ 0     1.15        0.0928   12.9
         30000    1.98        0.264    19.3
         50000    4.16        0.726    19.7 ];

% -- p.550, zarchan.txt:7700-7707.  Worked out in the text FROM the rounded
%    Table 23.2 entries, so it carries their rounding: hence the 'text' band
%    and not 'table'.
V.wz = [43.2 27.6 18.8];

% -- p.552, zarchan.txt:7748-7757.  Same provenance.  MAGNITUDE only: the
%    book's formula K1 = VM(Ma*Zd - Za*Md)/(1845*Ma) prints +3.07, while the
%    LISTING computes XK1 = -VM(...)/(1845*Ma) and gets -3.07.  The listing's
%    sign is the one the loop uses; the comparison is on |K1|, and this
%    sentence is why.
V.k1_mag = [3.07 1.23 0.558];

% -- p.538, zarchan.txt:7535: "Because the damping is 0.7 and the natural
%    frequency is approximately 35 rad/s, the overall or total time constant
%    is 0.34 s ... In other words the response reaches 63% of the steady-state
%    value in 0.34 s."  Sea level.
V.w_natural = 35;  V.tau_total = 0.34;  V.t63 = 0.34;

% -- p.542, zarchan.txt:7623: "When the desired time constant is reduced to
%    0.1 s, the new gains become KA = 3.72 deg/g-s, wI = 11.2 rad/s, and
%    KR = 0.098 s."  Sea level.
V.gains_tau01 = [3.72 0.098 11.2];      % KA, KR, wI -- same order as V.gains

% -- p.550, zarchan.txt:7710: "At sea level only a few deg/s of fin rate are
%    required whereas at 50 kft altitude nearly 600 deg/s are required".
%    Fig. 23.13.  The 600 is the author reading a plot, hence the 'graph' band.
V.step_finrate_50k = 600;

% -- p.547, zarchan.txt:7654: "In practice the maximum open-loop crossover
%    frequency should be approximately one third the bandwidth of the actuator."
V.wcr_over_wact = 1/3;

% -- p.546, zarchan.txt:7641 and p.543, zarchan.txt:7622.  NOT SCORED HERE --
%    these belong to Listing 23.2, the frequency-domain program, which this
%    file does not implement.  Carried so the INFO row can quote them.
V.gm_db_nominal = 11;  V.pm_deg_nominal = 45;  V.pm_deg_nominal_alt = 43;
V.wcr_achieved_nominal = 60;  V.wcr_achieved_100 = 104;
end


% =============================================================================
function B = benchmarks(R, cfg, offbook)
%BENCHMARKS  Computed vs published, with a band, a reason for the band, and the
%   table/figure/page each value came from.
%
%   FOUR TOLERANCE CLASSES, and the difference is PROVENANCE, not taste:
%     'table'  1%   printed in a table to three significant figures
%     'text'   2%   printed in running prose to two or three figures, or
%                   recomputed there from already-rounded table entries
%     'approx' 5%   the book says "approximately"
%     'graph' 15%   the author read it off a plot ("nearly 600 deg/s")
%   plus 'shape' and 'bound', which are inequalities and orderings taken from
%   prose and carry no tolerance at all.
%
%   Do NOT widen a band to turn a row green.

V = bookValues();
B = emptyBench();

% What each family of rows depends on.  xncg is in GATE_AERO because in THIS
% listing it is the trim linearization point as well as the command, so moving
% it moves Table 23.2 as well as the step.
GATE_AERO = {'vm','wgt','xncg'};
GATE_GAIN = [GATE_AERO, {'tau','wcr','zeta'}];
GATE_STEP = [GATE_GAIN, {'wact','zact','tf'}];

wA = gateWhy(offbook, GATE_AERO);
wG = gateWhy(offbook, GATE_GAIN);
wS = gateWhy(offbook, GATE_STEP);

% Every "grows with altitude" row below is a diff() over R.fc in the order the
% caller listed the altitudes.  Hand it a descending or shuffled alt grid and a
% CORRECT port fails those rows, so they are gated on the grid being ascending
% rather than silently reordered behind the caller's back.
ascAlt = numel(cfg.alt) >= 2 && issorted(cfg.alt);
whyAsc = 'needs 2 or more altitudes given in ASCENDING order';

altBook = V.aero(:,1).';
i0  = findAlt(cfg, 0);
i50 = findAlt(cfg, 50000);

% ---- A. airframe, Table 23.2, and the two quantities derived from it ------
nmA = {'Ma','Md','Za','Zd'};
unA = {'1/s^2','1/s^2','1/s','1/s'};
flA = {'xma','xmd','za','zd'};
for a = 1:numel(altBook)
    i = findAlt(cfg, altBook(a));
    for c = 1:4
        what = sprintf('aero %-2s @ %2g kft', nmA{c}, altBook(a)/1000);
        src  = 'Table 23.2, p.551 (zarchan.txt:7738)';
        if ~isempty(wA)
            B = addSkip(B, what, sprintf('%g', V.aero(a,1+c)), wA, src, unA{c});
        elseif isempty(i)
            B = addSkip(B, what, sprintf('%g', V.aero(a,1+c)), ...
                sprintf('alt = %g ft is not in the alt grid', altBook(a)), src, unA{c});
        else
            B = addVal(B, what, V.aero(a,1+c), R.fc(i).(flA{c}), 'table', src, unA{c}, '');
        end
    end
    what = sprintf('wZ (RHP zero) @ %2g kft', altBook(a)/1000);
    src  = 'p.550 worked calc (zarchan.txt:7700)';
    if ~isempty(wA)
        B = addSkip(B, what, sprintf('%g', V.wz(a)), wA, src, 'rad/s');
    elseif isempty(i)
        B = addSkip(B, what, sprintf('%g', V.wz(a)), 'alt not in grid', src, 'rad/s');
    else
        B = addVal(B, what, V.wz(a), R.fc(i).wz, 'text', src, 'rad/s', ...
            'the book recomputes this from the ROUNDED Table 23.2 entries');
    end
    what = sprintf('|K1| aero gain @ %2g kft', altBook(a)/1000);
    src  = 'p.552 worked calc (zarchan.txt:7748)';
    if ~isempty(wA)
        B = addSkip(B, what, sprintf('%g', V.k1_mag(a)), wA, src, 'g/deg');
    elseif isempty(i)
        B = addSkip(B, what, sprintf('%g', V.k1_mag(a)), 'alt not in grid', src, 'g/deg');
    else
        B = addVal(B, what, V.k1_mag(a), abs(R.fc(i).xk1), 'text', src, 'g/deg', ...
            'MAGNITUDE: the listing''s XK1 is negative, the book''s K1 formula is not');
    end
end

% ---- B. Table 23.1, the three autopilot gains at three flight conditions ---
nmG = {'KA','KR','wI'};
unG = {'deg/g-s','s','rad/s'};
flG = {'xka','xkr','wi'};
for a = 1:numel(altBook)
    i = findAlt(cfg, altBook(a));
    for c = 1:3
        what = sprintf('gain %-2s @ %2g kft', nmG{c}, altBook(a)/1000);
        src  = 'Table 23.1, p.549 (zarchan.txt:7691)';
        det  = '';
        if a == 1
            src = 'Table 23.1, p.549 + text p.537 (zarchan.txt:7691, 7528)';
            det = 'the sea-level row is quoted twice in the book and agrees with itself';
        end
        if ~isempty(wG)
            B = addSkip(B, what, sprintf('%g', V.gains(a,1+c)), wG, src, unG{c});
        elseif isempty(i)
            B = addSkip(B, what, sprintf('%g', V.gains(a,1+c)), ...
                sprintf('alt = %g ft is not in the alt grid', altBook(a)), src, unG{c});
        else
            B = addVal(B, what, V.gains(a,1+c), R.fc(i).(flG{c}), 'table', src, unG{c}, det);
        end
    end
end

% ---- C. the derived lag, sea level ---------------------------------------
src538 = 'p.538 text (zarchan.txt:7535)';
if ~isempty(wG)
    B = addSkip(B, 'W   FCS nat freq   @ 0 kft', sprintf('%g', V.w_natural), wG, src538, 'rad/s');
    B = addSkip(B, 'tau_TOT = tau+2z/W @ 0 kft', sprintf('%g', V.tau_total), wG, src538, 's');
elseif isempty(i0)
    B = addSkip(B, 'W   FCS nat freq   @ 0 kft', sprintf('%g', V.w_natural), ...
        'sea level is not in the alt grid', src538, 'rad/s');
    B = addSkip(B, 'tau_TOT = tau+2z/W @ 0 kft', sprintf('%g', V.tau_total), ...
        'sea level is not in the alt grid', src538, 's');
else
    B = addVal(B, 'W   FCS nat freq   @ 0 kft', V.w_natural, R.fc(i0).w, 'approx', ...
        src538, 'rad/s', 'the book says "approximately 35 rad/s"');
    B = addVal(B, 'tau_TOT = tau+2z/W @ 0 kft', V.tau_total, R.fc(i0).tau_total, ...
        'text', src538, 's', 'closed form, no integration involved');
end

% ---- D. t63: THE measurement, on the book's own output grid ---------------
if ~isempty(wS)
    B = addSkip(B, 't63 step response  @ 0 kft', sprintf('%g', V.t63), wS, src538, 's');
elseif isempty(i0)
    B = addSkip(B, 't63 step response  @ 0 kft', sprintf('%g', V.t63), ...
        'sea level is not in the alt grid', src538, 's');
else
    S = R.fc(i0).step;
    B = addVal(B, 't63 step response  @ 0 kft', V.t63, S.t63_s, 'text', src538, 's', ...
        sprintf(['read off the LISTING''S OWN %g s output array (the grid ' ...
                 'Fig. 23.3 is drawn from), against the ANALYTIC steady ' ...
                 'state'], S.book_grid_s));
    B = addInfo(B, 't63 the other two readings', '0.34 s is the only one published', ...
        sprintf('full-rate %.4f s | vs last sample %.4f s', ...
                S.t63_fullrate_s, S.t63_vs_last_sample_s), src538, ...
        sprintf(['NOT SCORED, REPORTED. Same crossing, three grids. ' ...
                 'FULL-RATE %.4f s is what mgs.zarchan.adjointWithAutopilot ' ...
                 'returns and is 1%% below the published value. VS LAST ' ...
                 'SAMPLE %.4f s is a MISTAKE kept on purpose: the run stops ' ...
                 'at %.0f%% of steady state, so 63%% of the last sample is ' ...
                 'not 63%% of anything, and it shortens the lag by 6%%.'], ...
                S.t63_fullrate_s, S.t63_vs_last_sample_s, 100*S.settled_frac));
end

% ---- E. Fig. 23.5: the algorithm actually controls the time constant ------
Vt = findVariant(R.variants, 'tau = 0.1 s');
src05 = 'Fig. 23.5 text, p.542 (zarchan.txt:7623)';
for c = 1:3
    what = sprintf('tau=0.1 gain %-2s @ 0 kft', nmG{c});
    if ~isempty(wG)
        B = addSkip(B, what, sprintf('%g', V.gains_tau01(c)), wG, src05, unG{c});
    elseif isempty(Vt)
        B = addSkip(B, what, sprintf('%g', V.gains_tau01(c)), ...
            'the tau = 0.1 s study was not run', src05, unG{c});
    else
        B = addVal(B, what, V.gains_tau01(c), Vt.fc.(flG{c}), 'text', src05, unG{c}, ...
            'a SECOND, independent exercise of the gain algorithm at a different goal');
    end
end
if ~isempty(wS) || isempty(Vt) || isempty(i0)
    B = addSkip(B, 'tau 0.3 -> 0.1 shortens the lag', 'both t63 and tau+2z/W fall', ...
        firstNonEmpty(wS, 'needs sea level and the tau = 0.1 s study'), src05, '-');
else
    S3 = R.fc(i0).step;
    ok = Vt.step.t63_s < S3.t63_s && Vt.fc.tau_total < R.fc(i0).tau_total;
    B = addShape(B, 'tau 0.3 -> 0.1 shortens the lag', 'both t63 and tau+2z/W fall', ...
        sprintf('t63 %.4f -> %.4f s, closed form %.4f -> %.4f s', ...
                S3.t63_s, Vt.step.t63_s, R.fc(i0).tau_total, Vt.fc.tau_total), ok, ...
        src05, 'the point of Fig. 23.5: the goal is a goal, not a wish');
end

% ---- F. altitude mechanism rows ------------------------------------------
if isempty(wA) && ascAlt
    wzv = [R.fc.wz];  k1v = abs([R.fc.xk1]);
    B = addShape(B, 'wZ falls as altitude rises', 'strictly decreasing', ...
        num2str(wzv, '%.2f '), all(diff(wzv) < 0), 'p.550 text (zarchan.txt:7709)', ...
        'the mechanism behind the wrong-way effect growing with altitude');
    B = addShape(B, '|K1| falls as altitude rises', 'strictly decreasing', ...
        num2str(k1v, '%.3f '), all(diff(k1v) < 0), 'p.552 text (zarchan.txt:7748)', ...
        'the mechanism behind fin rate growing with altitude');
else
    B = addSkip(B, 'wZ falls as altitude rises', 'strictly decreasing', ...
        firstNonEmpty(wA, whyAsc), 'p.550 text (zarchan.txt:7709)', '-');
    B = addSkip(B, '|K1| falls as altitude rises', 'strictly decreasing', ...
        firstNonEmpty(wA, whyAsc), 'p.552 text (zarchan.txt:7748)', '-');
end

% ---- G. Fig. 23.12: the response is held constant across flight condition --
src12 = 'Fig. 23.12 text, p.550 (zarchan.txt:7678)';
if isempty(wS) && numel(cfg.alt) >= 2
    t63 = arrayfun(@(f) f.step.t63_s, R.fc);
    t63f = arrayfun(@(f) f.step.t63_fullrate_s, R.fc);
    same = all(isfinite(t63)) && (max(t63) - min(t63)) == 0;
    B = addShape(B, 'same step response at every alt', 'IDENTICAL on the book grid', ...
        sprintf('%s s (full rate %s s, spread %.1f%%)', num2str(t63, '%.4f '), ...
                num2str(t63f, '%.4f '), 100*(max(t63f)/min(t63f)-1)), same, src12, ...
        ['on the 0.01 s array the book actually plotted, the three responses ' ...
         'reach 63% at the SAME SAMPLE -- an exact equality, not a tolerance. ' ...
         'Off that grid the same claim is true only to 1.9%, which is why the ' ...
         'grid is transcribed rather than skipped']);
else
    B = addSkip(B, 'same step response at every alt', 'IDENTICAL on the book grid', ...
        firstNonEmpty(wS, 'needs 2 or more altitudes'), src12, '-');
end
if isempty(wS) && ascAlt
    ww = arrayfun(@(f) abs(f.step.wrongway_g), R.fc);
    B = addShape(B, 'wrong-way dip grows with altitude', 'strictly increasing', ...
        sprintf('%s g', num2str(ww, '%.3f ')), all(diff(ww) > 0), src12, ...
        'the one difference the book allows between the three responses');
else
    B = addSkip(B, 'wrong-way dip grows with altitude', 'strictly increasing', ...
        firstNonEmpty(wS, whyAsc), src12, '-');
end

% ---- H. Fig. 23.13: fin rate ---------------------------------------------
src13 = 'Fig. 23.13 text, p.550 (zarchan.txt:7710)';
finNote = ['NOTE Listing 23.1 as printed saves only ArrayT/ArrayXNL/ArrayXNCG ' ...
           '-- it does NOT store DELD, so Fig. 23.13 came from a run the ' ...
           'author extended.  This row rests on that identification'];
if isempty(wS) && ascAlt
    pk = arrayfun(@(f) abs(f.step.peak_finrate_book_dps), R.fc);
    B = addShape(B, 'step fin rate grows with altitude', 'strictly increasing', ...
        sprintf('%s deg/s', num2str(pk, '%.1f ')), all(diff(pk) > 0), src13, finNote);
else
    B = addSkip(B, 'step fin rate grows with altitude', 'strictly increasing', ...
        firstNonEmpty(wS, whyAsc), src13, '-');
end
if isempty(wS) && ~isempty(i50)
    B = addVal(B, 'step peak fin rate @ 50 kft', V.step_finrate_50k, ...
        abs(R.fc(i50).step.peak_finrate_book_dps), 'graph', src13, 'deg/s', ...
        ['the book says "nearly 600 deg/s"; read off the 0.01 s grid, which ' ...
         'is what the figure shows. ' finNote]);
    if ~isempty(i0)
        % The port DISAGREES with the adjective in the same sentence, and that
        % is reported rather than buried.  17 deg/s is not "a few".
        B = addInfo(B, 'step peak fin rate @ 0 kft', 'a few deg/s', ...
            sprintf('%.2f deg/s', abs(R.fc(i0).step.peak_finrate_book_dps)), src13, ...
            ['NOT SCORED. The book''s only sea-level statement is the ' ...
             'adjective "a few", and 17 deg/s is not that. The RATIO to ' ...
             '50 kft (32x) is what the sentence is really about and it is ' ...
             'scored above. There is no number here to score against.']);
    end
else
    B = addSkip(B, 'step peak fin rate @ 50 kft', sprintf('%g deg/s', V.step_finrate_50k), ...
        firstNonEmpty(wS, '50 kft is not in the alt grid'), src13, 'deg/s');
end

% ---- I. p.549: how the three gains move with altitude ---------------------
src49 = 'p.549 text (zarchan.txt:7677)';
if isempty(wG) && ~isempty(i0) && ~isempty(i50)
    rKA = R.fc(i50).xka / R.fc(i0).xka;
    rKR = R.fc(i50).xkr / R.fc(i0).xkr;
    rWI = R.fc(i50).wi  / R.fc(i0).wi;
    B = addShape(B, 'KA rises a lot over 50 kft', 'ratio >= 3', ...
        sprintf('%.2fx (%.3f -> %.3f)', rKA, R.fc(i0).xka, R.fc(i50).xka), ...
        rKA >= 3, src49, 'the book calls it "a factor of 4"; see the INFO row');
    B = addShape(B, 'KR rises more than KA', 'ratio >= 5 and > KA''s', ...
        sprintf('%.2fx (%.4f -> %.4f)', rKR, R.fc(i0).xkr, R.fc(i50).xkr), ...
        rKR >= 5 && rKR > rKA, src49, 'the book calls it "nearly 10"; see the INFO row');
    B = addShape(B, 'wI moves least of the three gains', 'wI spread < KA and < KR', ...
        sprintf('wI %.2fx vs KA %.2fx, KR %.2fx', rWI, rKA, rKR), ...
        rWI < rKA && rWI < rKR, src49, ...
        'the defensible reading of "remains approximately constant"');
    B = addInfo(B, 'the three adjectives on p.549', ...
        '"factor of 4", "nearly 10", "approximately constant"', ...
        sprintf('%.2fx, %.2fx, %.2fx', rKA, rKR, rWI), src49, ...
        sprintf(['NOT SCORED, and the reason is that the book''s own Table ' ...
                 '23.1 disagrees with the book''s own adjectives. Recomputing ' ...
                 'the ratios from the PUBLISHED table gives %.2fx, %.2fx and ' ...
                 '%.2fx -- so "a factor of 4" is really 3.6, "nearly 10" is ' ...
                 'really 7.8, and wI "remains approximately constant" while ' ...
                 'moving +53%%. This port reproduces the TABLE (scored above ' ...
                 'at 1%%); the adjectives are the author rounding his own ' ...
                 'numbers and must not be quoted as data.'], ...
                V.gains(3,2)/V.gains(1,2), V.gains(3,3)/V.gains(1,3), ...
                V.gains(3,4)/V.gains(1,4)));
else
    B = addSkip(B, 'KA rises a lot over 50 kft', 'ratio >= 3', ...
        firstNonEmpty(wG, 'needs both 0 and 50 kft in the alt grid'), src49, '-');
    B = addSkip(B, 'KR rises more than KA', 'ratio >= 5', ...
        firstNonEmpty(wG, 'needs both 0 and 50 kft in the alt grid'), src49, '-');
    B = addSkip(B, 'wI moves least of the three gains', 'wI spread < KA and < KR', ...
        firstNonEmpty(wG, 'needs both 0 and 50 kft in the alt grid'), src49, '-');
end

% ---- J. Figs. 23.8/23.9/23.11: the adjectives, as a ringing count ---------
%   RIP = the ripple threshold.  0.5 g on a 10-g command: an order of magnitude
%   above the 0.00 g the nominal case produces and an order below the 2.1-2.7 g
%   the two "oscillatory" cases produce, so the classification is not close.
RIP = 0.5;
if isempty(wS) && ~isempty(i0)
    S0 = R.fc(i0).step;
    B = addShape(B, 'nominal step is well behaved', 'no direction reversals', ...
        sprintf('%d reversals, ripple %.3f g', S0.nrev, S0.ripple_g), ...
        S0.nrev == 0, 'Figs. 23.3/23.12, p.537/550 (zarchan.txt:7533)', ...
        'the baseline the two "oscillatory" rows below are contrasted against');
else
    B = addSkip(B, 'nominal step is well behaved', 'no direction reversals', ...
        firstNonEmpty(wS, 'sea level is not in the alt grid'), ...
        'Figs. 23.3/23.12 (zarchan.txt:7533)', '-');
end

oscRows = { 'wcr = 100 rad/s (wact 150)', 'wcr 100: oscillatory',        true
            'wcr = 100, wact = 300 rad/s','wact 300 rescues wcr 100',    false
            'zeta = 0.4',                 'zeta 0.4: oscillatory',       true };
for r = 1:size(oscRows,1)
    Vv   = findVariant(R.variants, oscRows{r,1});
    want = oscRows{r,3};
    if want, bookStr = 'rings'; else, bookStr = 'no reversals'; end
    if ~isempty(wS)
        B = addSkip(B, oscRows{r,2}, bookStr, wS, 'Ch.23 Figs. 23.8/23.9/23.11', '-');
    elseif isempty(Vv)
        B = addSkip(B, oscRows{r,2}, bookStr, ...
            sprintf('the "%s" study was not run', oscRows{r,1}), ...
            'Ch.23 Figs. 23.8/23.9/23.11', '-');
    else
        s = Vv.step;
        if want
            ok = s.nrev > 0 && s.ripple_g > RIP;
        else
            ok = s.nrev == 0;
        end
        B = addShape(B, oscRows{r,2}, bookStr, ...
            sprintf('%d reversals, ripple %.3f g', s.nrev, s.ripple_g), ok, ...
            Vv.source, sprintf('book: "%s"', Vv.claim));
    end
end

% The design rule, tested as a CLASSIFIER rather than as a number: it should
% put the two well-behaved cases on one side and the ringing one on the other.
Vh  = findVariant(R.variants, 'wcr = 100 rad/s (wact 150)');
Vh3 = findVariant(R.variants, 'wcr = 100, wact = 300 rad/s');
src47 = 'p.547 text (zarchan.txt:7654)';
if isempty(wS) && ~isempty(i0) && ~isempty(Vh) && ~isempty(Vh3)
    trio = [cfg.wcr/cfg.wact, 100/cfg.wact, 100/300];
    rings = [R.fc(i0).step.nrev > 0, Vh.step.nrev > 0, Vh3.step.nrev > 0];
    % STRICT '>' ON PURPOSE, AND IT SITS EXACTLY ON THE BOUNDARY.  Two of the
    % three cases are AT the rule, not near it: 50/150 and 100/300 both round
    % to the same double as 1/3, so the test is a floating-point equality in
    % disguise.  '>' puts equality on the "does not ring" side, which is the
    % rule as written ("should be approximately one third", i.e. up to a
    % third is fine) and is also what both cases actually do.  Flip it to
    % '>=' and a correct port fails two rows.
    ok = all((trio > V.wcr_over_wact) == rings);
    B = addShape(B, 'wcr <= wact/3 predicts which rings', ...
        'rule classifies all 3 cases', ...
        sprintf('wcr/wact %.2f/%.2f/%.2f -> rings %d/%d/%d', trio, rings), ...
        ok, src47, ...
        ['the book''s design rule "the maximum open-loop crossover frequency ' ...
         'should be approximately one third the bandwidth of the actuator", ' ...
         'used as a prediction over the three cases it was drawn from']);
else
    B = addSkip(B, 'wcr <= wact/3 predicts which rings', 'rule classifies all 3 cases', ...
        firstNonEmpty(wS, 'needs sea level and both wcr = 100 studies'), src47, '-');
end

% ---- K. what this chapter publishes that this listing cannot produce -------
B = addInfo(B, 'Ch.23 frequency-domain values', ...
    sprintf('gm %g dB, pm %g deg, wcr %g rad/s', ...
            V.gm_db_nominal, V.pm_deg_nominal, V.wcr_achieved_nominal), ...
    'not computed here', 'p.543/546 (zarchan.txt:7622, 7641)', ...
    sprintf(['NOT SCORED, AND NOT AN OVERSIGHT. Gain and phase margins and ' ...
             'the achieved crossover frequency (%g rad/s nominal, %g at the ' ...
             'wcr = 100 goal) come from LISTING 23.2, a separate ' ...
             'frequency-domain program that is not ported. Computing them ' ...
             'inside 23.1 would put a third copy of the same preamble in the ' ...
             'package -- the exact drift this file exists to avoid. Note also ' ...
             'that the book quotes the NOMINAL phase margin as %g deg on ' ...
             'p.543 and as %g deg on p.547 for the same case; whichever is ' ...
             'right, do not quote either as verified here.'], ...
            V.wcr_achieved_nominal, V.wcr_achieved_100, ...
            V.pm_deg_nominal, V.pm_deg_nominal_alt));

B = addInfo(B, 'overshoot / settling time', 'the book publishes neither', ...
    stepShapeStr(R, i0), 'Ch.23, grepped (zarchan.txt:7380-7830)', ...
    ['NOT SCORED because there is nothing to score against: the words ' ...
     '"overshoot", "settling" and "rise time" do not occur anywhere in ' ...
     'Ch.23. The 63% rise time is the ONLY published step-response ' ...
     'characteristic. Reported here so that a reader looking for an ' ...
     'overshoot benchmark learns it does not exist rather than assuming it ' ...
     'was forgotten -- and because at the book''s parameters the response ' ...
     'never overshoots inside TF = 1 s anyway; it is still rising.']);

% ---- L. the drift check against Listing 23.3's copy of this preamble -------
X = R.cross;
srcX = 'structural - NOT a book value';
if ~X.ran
    B = addSkip(B, 'no drift vs adjointWithAutopilot', 'agree to 1e-12', ...
        X.why, srcX, '-');
else
    B = addBound(B, 'no drift vs adjointWithAutopilot', ...
        sprintf('%d quantities agree to %.0e', X.nchecked, X.tol), ...
        sprintf('worst %.2e on %s', X.worst_rel, X.worst_name), X.ok, srcX, '-', ...
        ['THE ONE ROW THAT STILL BITES OFF-BOOK. Listings 23.1 and 23.3 ' ...
         'share a byte-identical preamble and both are ported; MATLAB cannot ' ...
         'share a local function between files, so the copies are compared ' ...
         'instead of merged. ' X.detail]);
end
end


function s = stepShapeStr(R, i0)
if isempty(i0), s = '(sea level not in grid)'; return; end
S = R.fc(i0).step;
s = sprintf('peak %.3f g of %.3f g at tf; %.0f%% settled', ...
            S.peak_g, S.ss_g, 100*S.settled_frac);
end


function V = findVariant(variants, name)
%FINDVARIANT  [] rather than an error: a caller with benchmark on but a study
%   that could not run must get a SKIP row naming the study, not an exception
%   from deep inside the board builder.
V = [];
if isempty(variants), return; end
i = find(strcmp({variants.name}, name), 1);
if ~isempty(i), V = variants(i); end
end


% =============================================================================
%   BENCH ROW PLUMBING
%
%   Field names, order and semantics are IDENTICAL to
%   mgs.zarchan.adjointWithAutopilot's, on purpose: benchSuite's
%   adaptAutopilot() switches on .kind and reads .book_str/.ours_str/.band/
%   .tol/.ok/.skipped/.why/.source/.detail, so it consumes these rows unchanged.
%   If you add a field here, add it there too or the struct-array append in the
%   adapter throws -- which is the behaviour we want.
% =============================================================================
function B = emptyBench()
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end


function B = addVal(B, what, book, ours, band, source, units, detail)
i = numel(B) + 1;
tol = bandTol(band);
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.4g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.5g', ours);
B(i).band = band;  B(i).tol = tol;
B(i).units = units; B(i).source = source; B(i).detail = detail;
if ~isfinite(ours) || book == 0
    B(i).pct = NaN;
    B(i).ok  = false;
    B(i).skipped = true;
    B(i).why = 'the computed value is not finite (or the book value is zero)';
    return;
end
B(i).pct = 100*(ours/book - 1);
B(i).ok  = abs(ours/book - 1) <= tol;
B(i).skipped = false;
B(i).why = '';
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
switch lower(band)
case 'table',  t = 0.01;   % three significant figures, printed in a table
case 'text',   t = 0.02;   % prose, 2-3 figures, or recomputed from rounded inputs
case 'approx', t = 0.05;   % the book says "approximately"
case 'graph',  t = 0.15;   % the author read it off a plot
otherwise
    error('mgs:zarchan:threeLoopAutopilot:band', ...
        ['unknown tolerance band "%s".  Bands encode the PROVENANCE of the ' ...
         'book value; inventing one silently sets an arbitrary pass ' ...
         'criterion.'], band);
end
end


function w = gateWhy(offbook, names)
%GATEWHY  '' if the caller left all of NAMES at the book value; otherwise the
%   reason this row cannot be compared against the book.
hit = intersect(offbook, names);
if isempty(hit)
    w = '';
else
    w = sprintf(['off-book: %s was changed, so this row would compare our ' ...
                 'number against a book value for a different system'], ...
                strjoin(hit, ', '));
end
end


function s = firstNonEmpty(a, b)
if isempty(a), s = b; else, s = a; end
end


function i = findAlt(cfg, a)
i = find(abs(cfg.alt - a) < 1e-9, 1);
end


function s = offbookStr(offbook)
if isempty(offbook), s = 'nothing'; else, s = strjoin(offbook, ', '); end
end


function s = tf2s(b)
if b, s = 'PASS'; else, s = 'FAIL'; end
end


function s = bandTag(band, tol)
%BANDTAG  Short label for the printed board.  'table' and 'text' both start
%   with 't', so the first letter is NOT enough -- an abbreviation that
%   collided would print two different provenances as the same thing.
switch lower(band)
case 'table',  s = sprintf('tbl %.0f%%', 100*tol);
case 'text',   s = sprintf('txt %.0f%%', 100*tol);
case 'approx', s = sprintf('apx %.0f%%', 100*tol);
case 'graph',  s = sprintf('fig %.0f%%', 100*tol);
otherwise,     s = band;
end
end


function s = trunc(str, n)
str = char(str);
if numel(str) > n, s = [str(1:n-1) '~']; else, s = str; end
end


function s = oneLine(msg)
%ONELINE  Collapse a multi-line error message so it fits one report row.
s = regexprep(char(msg), '\s+', ' ');
s = trunc(s, 160);
end


% =============================================================================
%   RENDER
% =============================================================================
function s = render(R, cfg, offbook)
W = 100;
L = {};
L{end+1} = 'Zarchan Listing 23.1 -- THREE-LOOP AUTOPILOT STEP RESPONSE SIMULATION';
L{end+1} = '*** UNITS ARE FEET, ft/s, deg, deg/s, g = 32.2 ft/s^2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['step command = %g g   tf = %g s   h = %g s   Vm = %g ft/s ' ...
                    '(Mach %.2f)   Wgt = %g lb'], ...
    cfg.xncg, cfg.tf, cfg.h, cfg.vm, R.fc(1).mach, cfg.wgt);
L{end+1} = sprintf(['FCS design goals: tau = %g s, crossover = %g rad/s, damping = %g' ...
                    '   |  actuator %g rad/s / %g'], ...
    cfg.tau, cfg.wcr, cfg.zeta, cfg.wact, cfg.zact);
L{end+1} = sprintf('alt grid    = %s ft', mat2str(cfg.alt));
L{end+1} = sprintf('off-book    : %s', offbookStr(offbook));
L{end+1} = repmat('=', 1, W);

% ---- the lag -------------------------------------------------------------
L{end+1} = 'THE DERIVED LAG -- the reason this port exists';
L{end+1} = sprintf('  %8s %9s %10s %12s %12s %12s %9s %8s', 'alt [ft]', 'tau goal', ...
    'W [rad/s]', 'tau+2z/W [s]', 't63 BOOK[s]', 't63 full[s]', 'DC gain', 'settled');
for i = 1:numel(R.fc)
    S = R.fc(i).step;
    L{end+1} = sprintf('  %8g %9.3f %10.2f %12.4f %12.4f %12.4f %9.6f %7.1f%%', ...
        R.fc(i).alt, cfg.tau, R.fc(i).w, R.fc(i).tau_total, ...
        S.t63_s, S.t63_fullrate_s, S.dc_gain, 100*S.settled_frac); %#ok<AGROW>
end
L{end+1} = sprintf(['  t63 BOOK is read off the listing''s OWN %g s output array -- the grid ' ...
                    'Figs. 23.3'], R.book_grid_s);
L{end+1} = '  and 23.12 are drawn from -- and it is the published 0.34 s at EVERY altitude.';
L{end+1} = '  t63 full reads the same crossing off the integration grid and is ~1% shorter;';
L{end+1} = '  that is the number mgs.zarchan.adjointWithAutopilot returns.  tau+2z/W is the';
L{end+1} = '  p.538 closed form.  Three independent computations of one lag.';
L{end+1} = sprintf(['  Reading t63 off the LAST SAMPLE instead (only %.0f%% settled at tf = %g s) ' ...
                    'gives'], 100*R.fc(1).step.settled_frac, cfg.tf);
L{end+1} = sprintf('  %.4f s -- 6%% short, and does not land on the book.  DC gain is an exact identity.', ...
    R.fc(1).step.t63_vs_last_sample_s);
L{end+1} = '  The gain algorithm holds this lag constant while the airframe changes by 6x.';
L{end+1} = '  Our own ~8 s gamma-tracking lag is held constant by nothing -- that is the gap.';
L{end+1} = repmat('-', 1, W);

% ---- flight conditions ---------------------------------------------------
L{end+1} = 'FLIGHT CONDITIONS -- aero from geometry (Ch.21/22), gains from the Ch.23 algorithm';
L{end+1} = sprintf('  %8s %10s %9s %9s %9s %10s %8s %8s %8s %8s', ...
    'alt', 'Q[psf]', 'Ma', 'Md', 'Za', 'Zd', 'wZ', 'wAF', 'zAF', 'alf_tr');
for i = 1:numel(R.fc)
    F = R.fc(i);
    L{end+1} = sprintf('  %8g %10.1f %9.3f %9.3f %9.4f %10.5f %8.3f %8.3f %8.5f %8.4f', ...
        F.alt, F.q, F.xma, F.xmd, F.za, F.zd, F.wz, F.waf, F.zaf, F.trim.alftr_deg); %#ok<AGROW>
end
L{end+1} = sprintf('  %8s %10s %9s %9s %9s %9s %9s %8s %8s', ...
    'alt', 'K1[g/dg]', 'Ta[s]', 'K3', 'KA', 'wI', 'KR[s]', 'KDC', 'KC');
for i = 1:numel(R.fc)
    F = R.fc(i);
    L{end+1} = sprintf('  %8g %10.4f %9.4f %9.4f %9.4f %9.4f %9.5f %8.4f %8.4f', ...
        F.alt, F.xk1, F.ta, F.xk3, F.xka, F.wi, F.xkr, F.xkdc, F.xkc); %#ok<AGROW>
end
L{end+1} = '  Ma/Md are 1/s^2, Za/Zd 1/s, alf_tr is the 10-g trim angle of attack in deg.';
L{end+1} = '  a = 1000 ft/s at EVERY altitude -- the book''s approximation, deliberately kept.';
L{end+1} = repmat('-', 1, W);

% ---- the step responses themselves ---------------------------------------
L{end+1} = 'STEP RESPONSE (Figs. 23.3 / 23.12 / 23.13)';
L{end+1} = sprintf('  %8s %10s %10s %12s %12s %9s %9s', 'alt [ft]', 'peak [g]', ...
    'at tf [g]', 'wrong-way[g]', 'pk rate[d/s]', 'reversal', 'ripple[g]');
for i = 1:numel(R.fc)
    S = R.fc(i).step;
    L{end+1} = sprintf('  %8g %10.4f %10.4f %12.4f %12.2f %9d %9.4f', ...
        R.fc(i).alt, S.peak_g, S.nl_at_tf_g, S.wrongway_g, ...
        S.peak_finrate_book_dps, S.nrev, S.ripple_g); %#ok<AGROW>
end
L{end+1} = '  wrong-way is the RHP-zero excursion OPPOSITE the command, and it grows as wZ';
L{end+1} = '  falls with altitude.  pk rate is off the book''s 0.01 s grid (Fig. 23.13); note';
L{end+1} = '  Listing 23.1 as printed does NOT save fin rate -- see the header.  "reversal" is';
L{end+1} = '  the count of direction changes after 0.05 s: 0 = monotone rise, tens = ringing.';
L{end+1} = repmat('-', 1, W);

% ---- the variant studies -------------------------------------------------
if ~isempty(R.variants)
    L{end+1} = 'OFF-NOMINAL STUDIES at sea level -- one run per claim in the text';
    L{end+1} = sprintf('  %-28s %9s %9s %9s %9s %9s', 'study', 'KA', 'wI', 'KR', ...
        'reversal', 'ripple');
    for r = 1:numel(R.variants)
        Vv = R.variants(r);
        L{end+1} = sprintf('  %-28s %9.4f %9.4f %9.5f %9d %9.4f', ...
            trunc(Vv.name,28), Vv.fc.xka, Vv.fc.wi, Vv.fc.xkr, ...
            Vv.step.nrev, Vv.step.ripple_g); %#ok<AGROW>
        L{end+1} = sprintf('    book: %s', trunc(Vv.claim, 88)); %#ok<AGROW>
    end
    L{end+1} = repmat('-', 1, W);
end

% ---- the cross-check -----------------------------------------------------
X = R.cross;
L{end+1} = 'CROSS-CHECK vs mgs.zarchan.adjointWithAutopilot (Listing 23.3''s copy of the';
L{end+1} = 'same preamble).  The one check here that still bites when the run is off-book.';
if ~X.ran
    L{end+1} = sprintf('  NOT RUN: %s', trunc(X.why, 92));
else
    L{end+1} = sprintf('  %d quantities compared over %s ft, worst relative %.3e (tol %.0e)  %s', ...
        X.nchecked, mat2str(X.alt), X.worst_rel, X.tol, tf2s(X.ok));
    L{end+1} = sprintf('  worst on: %s   t63 gap %.3g s', X.worst_name, X.t63_delta_s);
    if ~isempty(X.detail), L{end+1} = sprintf('  %s', trunc(X.detail, 96)); end
    if ~X.ok,             L{end+1} = sprintf('  %s', trunc(X.why, 96)); end
end
L{end+1} = repmat('-', 1, W);

% ---- structural self-check ----------------------------------------------
if isfield(R.selfcheck, 'ok')
    C = R.selfcheck;
    L{end+1} = 'STRUCTURAL SELF-CHECK (wiring, not the book -- a failure here voids everything)';
    L{end+1} = sprintf('  unity closed-loop DC gain (identity)         : %9.3e  tol %.0e  %s', ...
        C.dc_err, C.dc_tol, tf2s(C.dc_ok));
    L{end+1} = sprintf('  measured t63 vs closed form tau + 2*zeta/W   : %9.3e  tol %.0e  %s', ...
        C.lag_rel, C.lag_tol, tf2s(C.lag_ok));
    L{end+1} = sprintf('  output sampler really is the book''s grid     : %9.3e  n = %d/%d %s', ...
        C.grid_dt_err, C.grid_n, C.grid_n_want, tf2s(C.grid_ok));
    L{end+1} = sprintf('  RHP zero present, opposes, grows with alt    : %9.3f x            %s', ...
        C.ww_ratio, tf2s(C.ww_ok));
else
    L{end+1} = 'STRUCTURAL SELF-CHECK  ***NOT RUN*** (benchmark = false) -- nothing is verified';
end
L{end+1} = repmat('-', 1, W);

% ---- the board -----------------------------------------------------------
if isempty(R.bench)
    L{end+1} = '*** NO BOOK COMPARISON WAS MADE (benchmark = false). ***';
    L{end+1} = '    Every number above is this code''s opinion of itself.  R.pass is false and';
    L{end+1} = '    stays false: "not checked" and "checked and correct" must not print alike.';
else
    ev = ~[R.bench.skipped];
    L{end+1} = sprintf('BOOK BENCHMARKS -- %d evaluated, %d passed, %d skipped/reported', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev));
    L{end+1} = sprintf('  %-34s %-16s %-24s %8s %-7s %s', ...
        'what', 'book', 'ours', 'diff', 'band', 'verdict');
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch b.kind
        case 'value'
            if b.skipped
                L{end+1} = sprintf('  %-34s %-16s %-24s %8s %-7s SKIP', ...
                    trunc(b.what,34), trunc(b.book_str,16), '--', '--', b.band); %#ok<AGROW>
            else
                L{end+1} = sprintf('  %-34s %-16s %-24s %+7.2f%% %-7s %s', ...
                    trunc(b.what,34), trunc(b.book_str,16), trunc(b.ours_str,24), ...
                    b.pct, bandTag(b.band, b.tol), tf2s(b.ok)); %#ok<AGROW>
            end
        case {'shape','bound'}
            L{end+1} = sprintf('  %-34s %-16s %-24s %8s %-7s %s', ...
                trunc(b.what,34), trunc(b.book_str,16), trunc(b.ours_str,24), ...
                '--', b.band, tf2s(b.ok)); %#ok<AGROW>
        case 'info'
            L{end+1} = sprintf('  %-34s %-16s %-24s %8s %-7s INFO', ...
                trunc(b.what,34), trunc(b.book_str,16), trunc(b.ours_str,24), ...
                '--', 'info'); %#ok<AGROW>
        otherwise
            L{end+1} = sprintf('  %-34s %-16s %-24s %8s %-7s SKIP', ...
                trunc(b.what,34), trunc(b.book_str,16), '--', '--', 'skip'); %#ok<AGROW>
        end
    end
    L{end+1} = '  bands: tbl 1% = table (3 sig figs)   txt 2% = prose   apx 5% = "approximately"';
    L{end+1} = '         fig 15% = read off a figure   shape/bound = prose inequality, no tolerance';
    L{end+1} = '  every row''s table/figure/page and zarchan.txt line is in R.bench(i).source.';
    % Anything skipped, reported or failed gets its reason spelled out, so a
    % green-looking board cannot hide a row that was never evaluated.
    idx = find([R.bench.skipped] | ~[R.bench.ok]);
    if ~isempty(idx)
        L{end+1} = '  --- rows not scored green, and why ---';
        for j = idx(:).'
            b = R.bench(j);
            r = b.why;
            if isempty(r) && ~b.ok, r = ['FAILED: ' b.detail]; end
            if isempty(r), r = '(no reason recorded)'; end
            L{end+1} = sprintf('    %-34s %s', trunc(b.what,34), trunc(r, 60)); %#ok<AGROW>
            L{end+1} = sprintf('    %-34s src %s', '', b.source); %#ok<AGROW>
        end
    end
end
L{end+1} = repmat('=', 1, W);

% ---- the footer that has to travel with any quoted number ---------------
L{end+1} = 'READ THIS BEFORE QUOTING ANY NUMBER ABOVE';
L{end+1} = '  This is a SINGLE-CHANNEL STEP RESPONSE into a linear plant.  There is no';
L{end+1} = '  guidance loop, no seeker and no target: constant speed (Mach 3), constant';
L{end+1} = '  altitude, no gravity, no drag, no thrust, no acceleration limit, no fin-rate or';
L{end+1} = '  fin-deflection limit, and aero LINEARIZED about a 10-g trim.  A time constant';
L{end+1} = '  computed here is a property of that linear plant.  What DOES transfer is the';
L{end+1} = '  lag block at the top: a flight-control time constant DERIVED from gains and';
L{end+1} = '  airframe rather than fitted, and held constant across flight conditions by a';
L{end+1} = '  gain algorithm we do not have.  Our own gamma-tracking lag is not measured this';
L{end+1} = '  way and the two numbers are not comparable without saying how they were taken.';
if ~isempty(offbook)
    L{end+1} = sprintf(['  NOTE this run moved %s off the book, so the rows that depend on ' ...
        'them were skipped, not compared.'], offbookStr(offbook));
end
s = strjoin(L, newline);
end


% =============================================================================
function drawCurves(R, cfg)
%DRAWCURVES  The listing's own figure (Figs. 23.3/23.12), plus fin rate
%   (Fig. 23.13) and the off-nominal studies.  Plotting is OFF by default.
%
%   Everything is drawn from the BOOK'S 0.01 s arrays, not the integration
%   grid, because those are the arrays Listing 23.1 saves and plots -- and
%   because the ringing that distinguishes Fig. 23.8 from Fig. 23.9 has to be
%   visible at the resolution the author actually had.

figure('Name', 'Zarchan Figs. 23.3/23.12 -- autopilot step response');
hold on;
for i = 1:numel(R.fc)
    plot(R.fc(i).step.tb, R.fc(i).step.nl_book_g, 'LineWidth', 1.2);
end
plot(R.fc(1).step.tb, R.fc(1).step.nc_book_g, 'k--');
grid on; hold off;
xlabel('Time (Sec)'); ylabel('Acceleration (G)');
legend([arrayfun(@(f) sprintf('%g kft', f.alt/1000), R.fc, 'UniformOutput', false), ...
        {'command'}], 'Location', 'best');
title('Listing 23.1 step response -- the lag, measured');

figure('Name', 'Zarchan Fig. 23.13 -- fin rate increases with altitude');
hold on;
for i = 1:numel(R.fc)
    plot(R.fc(i).step.tb, R.fc(i).step.finrate_book_dps, 'LineWidth', 1.2);
end
grid on; hold off;
xlabel('Time (Sec)'); ylabel('Fin rate (Deg/Sec)');
legend(arrayfun(@(f) sprintf('%g kft', f.alt/1000), R.fc, 'UniformOutput', false), ...
    'Location', 'best');
title('Fin rate increases with altitude (fin rate is NOT saved by the printed listing)');

if ~isempty(R.variants)
    figure('Name', 'Zarchan Figs. 23.5/23.8/23.9/23.11 -- off-nominal studies');
    hold on;
    i0 = findAlt(cfg, 0);
    if ~isempty(i0)
        plot(R.fc(i0).step.tb, R.fc(i0).step.nl_book_g, 'k', 'LineWidth', 1.6);
        names = {'nominal'};
    else
        names = {};
    end
    for r = 1:numel(R.variants)
        plot(R.variants(r).step.tb, R.variants(r).step.nl_book_g, 'LineWidth', 1.1);
        names{end+1} = R.variants(r).name; %#ok<AGROW>
    end
    grid on; hold off;
    xlabel('Time (Sec)'); ylabel('Acceleration (G)');
    legend(names, 'Location', 'best');
    title('Crossover frequency and damping: what "oscillatory" looks like');
end
end
