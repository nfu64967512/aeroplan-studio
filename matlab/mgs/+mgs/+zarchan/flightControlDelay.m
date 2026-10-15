function R = flightControlDelay(opts)
%MGS.ZARCHAN.FLIGHTCONTROLDELAY  Zarchan Listing 22.4 -- the rate gyro
%   flight-control system with a PURE TRANSPORT DELAY in series with the
%   autopilot gain, ported line for line.
%
%   R = mgs.zarchan.flightControlDelay()                  % book run + benchmarks
%   R = mgs.zarchan.flightControlDelay('delay', 0.010)    % a shorter delay
%   R = mgs.zarchan.flightControlDelay('path', 'lag')     % same T, first-order lag
%   R = mgs.zarchan.flightControlDelay('benchmark', false, 'xkr', 0.4)
%
%   The Ch.21 notional missile (20 ft x 1 ft, 1000 lb, Mach 3 at sea level),
%   linearised at its 10-g trim, closed by the rate gyro autopilot of
%   Fig. 22.7, driven through a second-order actuator, with DELAY seconds of
%   pure transport delay inserted ahead of the actuator.  A 10-g step is
%   commanded and the achieved acceleration is integrated for 1 s.
%
% ===========================================================================
%   *** UNITS ARE THE BOOK'S.  FEET, ft/s, g = 32.2 ft/s^2, DEGREES. ***
%
%   Zarchan works in feet with g = 32.2 and converts rad->deg with the
%   literal 57.3 (it appears here folded into 1845 = 57.3*32.2).  The rest of
%   mgs is metres.  This file is DELIBERATELY left in the book's units so the
%   published numbers can be compared digit for digit with no conversion
%   sitting in between to be got wrong.  Convert at the CALL SITE, visibly.
%
%   Two unit facts that are easy to get backwards here, both verified by an
%   identity in STRUCTURALCHECK rather than asserted:
%     * ALFTR and DELTR are RADIANS.  The slender-body term "2*alpha" in CN
%       is per radian, and CN(alpha_tr, delta_tr) reproduces the commanded
%       10 g to 6e-16 relative only if they are.  R.aero also carries
%       .alftr_deg / .deltr_deg for reading (2.81 deg and -3.26 deg).
%     * K1 is nevertheless g/DEG, because the 1845 = 57.3*32.2 in XK1 does
%       both conversions at once.  A "K1 in g/rad" is 57.3x too big and still
%       looks like a plausible number.
% ===========================================================================
%
%   WHY THIS LISTING, OUT OF THE WHOLE CHAPTER
%
%   Everything else this package has ported models a guidance lag as a POLE:
%   one (Listing 3.1), five equal ones (Listing 6.1), a whole autopilot
%   (Listing 23.3).  This one models it as a pure DELAY, and the two are not
%   the same object:
%
%       first-order lag   1/(1+Ts)   phase -atan(wT), magnitude 1/sqrt(1+w^2T^2)
%       pure delay        exp(-Ts)   phase -wT,       magnitude EXACTLY 1
%
%   The lag pays for its phase loss by throwing gain away, which drags the
%   gain crossover frequency down to where there is phase margin to spare.
%   The delay pays nothing.  It eats phase margin at a rate that grows
%   without bound in frequency while leaving the loop gain untouched.
%
%   That is our shape.  Guidance runs on a fixed tick, the command goes out
%   over MAVLink, and the airframe answers late -- measured at ~8 s of lag
%   with a gamma-tracking gain of 0.063 (memory: gamma-tracking-deficit).
%   A loop like that tracks badly without ever looking sluggish, because
%   nothing is being attenuated.
%
%   MEASURED HERE, and this is the sentence to take away:
%       this loop tolerates 0.0147 s of pure delay before it goes unstable
%       (147 steps of h, book value 0.015 s), and tolerates a first-order lag
%       at the delay's own time constant, at 10x it and at 100x it -- 0.015,
%       0.15 and 1.5 s -- without going unstable at any of them.  Those three
%       are what R.lagprobe runs; a separate sweep out to 2 s found no
%       unstable lag either, which is not surprising, because a first-order
%       lag can cost at most 90 deg of phase and pays for it in gain.
%   So "8 s of lag" and "8 s of delay" are not two phrasings of one
%   measurement.  Our own number was fitted as a TRACKING GAIN plus a
%   settling time, which does not separate them, and this port is the reason
%   to stop treating them as interchangeable in the write-up.
%   See R.lagprobe and the "LAG IS NOT DELAY" block of the printed report.
%
% ---------------------------------------------------------------------------
%   WHAT THE TRANSCRIPTION LEFT UNRESOLVED, AND HOW EACH WAS DECIDED
%
%   Source: matlab/zarchan/_listings/listing_22_04.m, extracted from
%   matlab/zarchan/_source/zarchan.txt:7191-7209.  The extractor could not
%   resolve comparison operators whose glyphs the OCR destroyed; it left them
%   as ',' and '.' rather than guess.  Every one of them is decided below
%   from the surrounding listings, and none of them was decided from "what
%   makes the answer come out right".
%
%   1. "if (ALT , 30000.)"  ->  ALT < 30000
%      ',' is the OCR of '<' throughout these listings; the two branches are
%      the two standard-atmosphere density fits either side of the
%      tropopause.  Listings 22.1 and 22.2 render the SAME line as
%      "if ALT,=30000." (i.e. <=), so the book is not self-consistent about
%      the boundary -- but the fits are continuous there (8.747e-4 vs
%      8.694e-4 slug/ft^3 at exactly 30 kft), so the disagreement cannot
%      change any result.  Transcribed as it appears in 22.4: '<'.
%
%   2. "while ~(T . .99999)"  ->  ~(T > 0.99999)
%      '.' is the OCR of '>'; Listing 22.1 renders the same construct as
%      "if S.=.0099999" (>=).  The run has to stop at 1 s because Fig. 22.17
%      counts peaks IN a 1-s window.  '>' and '>=' give the identical
%      trajectory here (T lands on 0.99999999...), so this is a provenance
%      choice, not a behavioural one.
%
%   3. "if S . .00099999"  ->  S > 0.00099999
%      Same evidence.  MEASURED, so it is not asserted: at H = 1e-4 the
%      accumulated S lands on 1.0000000000000002e-3, so the book's odd
%      constant and a tidy "S >= 0.001" fire on the same step and the
%      constant is INERT at the book's step size.  It is not inert in
%      general -- at H = 2.5e-5 they fire on step 40 and step 41, which
%      would decimate at 1.025 ms instead of 1.000 ms and drift the whole
%      timebase.  Kept as written.
%
%   4. "if ((I+1) , DINT)"  ->  (I+1) < DINT
%      The buffer-priming branch: until DINT samples exist, feed the
%      actuator Z(1), which the listing sets to 0.  Any other reading indexes
%      a slot that has not been written.
%
%   5. "while STEP ,=1" -> "while STEP <= 1" was already resolved by the
%      extractor; it is Zarchan's standard two-pass (Heun) integrator.
%
%   THE OCR'S FOUR SUBSTITUTIONS, collected so the next reader does not have
%   to rediscover them:  ',' is '<',  '.' is '>',  a leading '2' on a number
%   is a minus sign ("2642 s22" is -642 s^-2), and '=' comes out as the
%   vulgar fraction one-quarter.  Book quotes reproduced in the comments
%   below have that last one undone so this file stays pure ASCII; the other
%   three are called out individually wherever they mattered.
%
%   6. The transcription's last line is a paragraph of the book's running
%      text ("We have just seen that there is a relationship between the time
%      and frequency domains...") that leaked into the extraction.  DROPPED,
%      not converted into code.
%
%   7. "DELC=Z(I+2-DINT); % I Have changed the code here from FORTRAN!"
%      The book's own comment, and the one line where a silent off-by-one
%      would produce a perfectly plausible wrong answer -- a loop delayed by
%      148 or 152 steps still oscillates and still diverges, just at the
%      wrong margin.  So it is not argued, it is MEASURED: structural check
%      "delay buffer realises DINT steps" replays the recorded DELC against
%      the recorded DELCP and demands bit equality at exactly DINT samples of
%      offset.  0 violations of 19851 at the book's settings.  The +2 is
%      correct: the value handed to the actuator during step k was computed
%      at step k-DINT.
%
% ---------------------------------------------------------------------------
%   THE TRAPS IN THIS PARTICULAR LISTING
%
%   * DELAY < H makes DINT = 0, and then Z(I+2-DINT) = Z(I+2) is a slot that
%     has not been written yet.  It reads 0 forever, so DELC is 0 forever,
%     so the actuator never moves and XNL stays EXACTLY 0 for the whole run.
%     That prints as a flat line: the most stable-looking output the program
%     can produce, from a loop that has been cut.  This function ERRORS.
%
%   * The whole point of the run is to detect an instability, and a two-pass
%     integrator run too coarsely MANUFACTURES one.  The book says so at
%     zarchan.txt:7032: "If the integration interval were 10 times larger,
%     the system would appear to go unstable ... this instability is due
%     simply to numerical integration."  The fastest pole here is the
%     actuator at WACT rad/s.  WACT*H is guarded explicitly (book: 0.015)
%     rather than left to a tolerance, because the failure returns large
%     FINITE numbers that are indistinguishable from the physical answer.
%
%   * The peak COUNT is the book's own measurement (10 peaks, and 24 peaks
%     for the gain-margin case).  A count read off a decimated array aliases
%     if the sample interval is not small against the oscillation period, and
%     an aliased count is a plausible integer.  The peak rows are skipped,
%     with a reason, when fewer than 8 stored samples per period survive.
%
%   * The peak detector has NO prominence threshold.  A threshold is a knob
%     that can be turned until the count comes out at 10, which would make
%     the sharpest benchmark on the board meaningless.  A peak is a strict
%     local maximum of the stored array and nothing else.
%
%   * XKDC = (1-XKR*XK3)/(XK1*XKR) is a function of XKR.  Sweeping the
%     autopilot gain with XKDC frozen leaves the loop dynamics right and the
%     steady state wrong, which looks like a modelling error rather than a
%     bookkeeping one.  SIMULATE recomputes it from the XKR it was handed.
%
%   * `nargin` is ALWAYS 0 in a function whose arguments block declares only
%     name-value options (measured, R2025b).  It is NOT used as a mode
%     switch anywhere in this file; 'benchmark' is the only switch, and it
%     defaults true.  Three ports in the first wave got this wrong.
%
% ---------------------------------------------------------------------------
%   WHAT IS BENCHMARKED, AND THE RULE FOR WHAT COUNTS AS A BOOK VALUE
%
%   Greppable in zarchan.txt = usable.  Present only on a figure = does not
%   exist (the extraction turned every figure into an image link).  A PROSE
%   CLAIM about a figure is greppable and is used, as a shape or ordering
%   check.  An adjective is reported as INFO and NOT scored.  See BOOKVALUES.
%
%   The rows fall into five groups:
%     A  the linearised airframe (Tables 22.1, 22.2, 23.2, Listing 22.3)
%     B  the delayed run itself: 10 peaks, 62.8 rad/s, divergence (Fig 22.17)
%     C  the delay margin, measured by bisection on this same loop, against
%        the book's 0.015 s / 0.96 rad / 55 deg phase-margin arithmetic
%     D  the GAIN margin route with the delay path removed: 24 peaks at
%        KR = 0.4, oscillation at 150 rad/s, KUNSTABLE = 0.376
%     E  the damping design point ("approximately 0.1 ... to unity")
%
%   Group D matters as much as group B.  It is the same loop destabilised the
%   other way, and it oscillates at a DIFFERENT frequency (the phase
%   crossover, 150 rad/s) than the delay case does (the gain crossover,
%   64 rad/s).  If a port reproduced both counts but at one frequency, it
%   would have got the physics of this section backwards while looking right.
%
% ---------------------------------------------------------------------------
%   RETURNS
%
%   R.aero        the linearised airframe: .rho .q .xmach .b .sref .splan
%                 .xiyy .alftr (rad) .alftr_deg .deltr .deltr_deg .cna .cnd
%                 .cma .cmd .xma .xmd .za .zd .wz .waf .zaf .xk1 .ta .xk3
%   R.fcs         closed-loop form of p.510: .w .zeta .kr_for_unit_zeta
%                 .w_over_waf .xkdc
%   R.run         THE LISTING: .t .xnl .xncg .del .delc .delcp .dint
%                 .delay_realized_s .npeaks .t_peak .w_osc .sigma .pp_last
%                 .final .maxabs .samples_per_period
%   R.nodelay     the same loop with the delay path bypassed (the delete
%                 experiment: this is what "no instability" looks like)
%   R.gain04      the same loop, no delay, XKR = 0.4 (Fig. 22.13)
%   R.margin      .dint_neutral .delay_neutral_s .w_at_delay_neutral
%                 .phase_loss_rad .phase_loss_deg .kr_neutral
%                 .gain_margin_db .w_at_kr_neutral
%   R.lagprobe    first-order lags in the delay's place: .tlag .sigma .stable
%   R.bench       computed vs book, one row per published value, each with
%                 .source (table/figure/page + the zarchan.txt line)
%   R.selfcheck   structural invariants and their MEASURED residuals
%   R.pass        true iff every EVALUATED row passed
%   R.cfg/.offbook/.summary
%
%   Bench rows carry BOTH numeric (.book/.ours, NaN on shape rows) and text
%   (.book_str/.ours_str, always populated), and the field set is identical
%   to mgs.zarchan.adjointWithAutopilot's, so benchSuite's adaptAutopilot
%   pattern transfers unchanged: kinds are value/shape/bound/info/skip.
%
%   RUNTIME: ~4 s.  The nominal run is 10,000 steps; the two margin
%   bisections are about 30 runs of 20,000 steps between them.  Pass
%   ('margins', false) to skip them -- the group C and D rows then skip too,
%   with a reason, and R.pass still means what it says.
%
%   See also MGS.ZARCHAN.BENCHSUITE, MGS.ZARCHAN.ADJOINTWITHAUTOPILOT.

arguments
    % Defaults are [] on purpose -- see BOOKNOMINAL().  Empty means "use the
    % book value"; non-empty means "the caller chose this", which is what
    % gates the benchmark rows.  Do NOT replace these with literals: that
    % puts a second copy of every book constant in the file and destroys the
    % only mechanism this port has for telling an off-book run apart from a
    % broken one.
    opts.alt    double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % ALT   [ft]
    opts.vm     double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % VM    [ft/s]
    opts.wgt    double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % WGT   [lb]
    opts.xncg   double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % XNCG  [g] step + trim
    opts.xkr    double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % XKR   autopilot gain
    opts.wact   double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % WACT  [rad/s]
    opts.zact   double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % ZACT  [-]
    opts.delay  double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % DELAY [s]
    opts.h      double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % H     [s]
    opts.tstop  double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % run length [s]
    opts.tsamp  double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % storage interval [s]
    opts.tmargin double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = [] % margin-run length [s]
    % 'delay' is the listing.  'none' bypasses the delay path (Listing 22.2
    % plus the actuator) and 'lag' replaces it with a first-order lag of the
    % SAME time constant.  Those two are not cosmetic: they are the delete
    % experiment and the lag/delay contrast, and both are used by the
    % structural checks regardless of what the caller asks for here.
    opts.path   (1,:) char {mustBeMember(opts.path, {'delay','none','lag'})} = 'delay'
    opts.margins   (1,1) logical = true    % run the two bisections
    opts.benchmark (1,1) logical = true    % THE ONLY SWITCH.  Not nargin.
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

GRAV_FT = 32.2;    % g, the book's value.  Not 9.80665, not 9.81.

N = bookNominal();
[cfg, offbook] = resolveConfig(opts, N);

R = struct();
R.units   = 'ft, ft/s, rad (alpha/delta), deg, g = 32.2 ft/s^2 -- Zarchan''s units, NOT SI';
R.listing = 'Zarchan Listing 22.4 (zarchan.txt:7191), Fig. 22.17';
R.opts    = opts;
R.cfg     = cfg;
R.offbook = offbook;
R.nominal = N;

% ---- the airframe preamble and the closed-form autopilot algebra ----------
R.aero = airframe(cfg, GRAV_FT);
R.fcs  = closedLoop(R.aero, cfg);

% ---- THE LISTING ---------------------------------------------------------
% Every call to SIMULATE in this file passes all eight arguments, including
% the trailing [] that means "derive DINT the book's way".  SIMULATE is an
% ordinary function so its nargin is honest -- but leaving the count to vary
% is how a defaulted argument gets silently repurposed later, and this
% package has been burned by exactly that shape of bug.
R.run = simulate(R.aero, cfg, cfg.delay, cfg.xkr, cfg.path, cfg.tstop, true, []);

if cfg.benchmark
    % The delete experiment and the gain-margin case.  Both ignore whatever
    % the caller set 'path' to, because both are DEFINED as "the delay path
    % removed" -- a caller asking for path='lag' must still get the same
    % structural evidence, not a quietly different pair of runs.
    R.nodelay = simulate(R.aero, cfg, cfg.delay, cfg.xkr, 'none', cfg.tmargin, true, []);
    R.gain04  = simulate(R.aero, cfg, cfg.delay, 0.4,     'none', cfg.tstop,   false, []);

    if cfg.margins
        R.margin   = margins(R.aero, cfg);
        R.lagprobe = lagProbe(R.aero, cfg);
    else
        R.margin   = struct('skipped', true);
        R.lagprobe = struct('skipped', true);
    end

    R.selfcheck = structuralCheck(R.aero, cfg, R);
    R.bench     = benchmarks(R, cfg, offbook);
    ev          = ~[R.bench.skipped];
    R.pass      = any(ev) && all([R.bench(ev).ok]);
else
    R.nodelay   = [];
    R.gain04    = [];
    R.margin    = struct('skipped', true);
    R.lagprobe  = struct('skipped', true);
    R.selfcheck = struct('skipped', true);
    R.bench     = emptyBench();
    ev          = false;
    R.pass      = false;   % NOT true.  Nothing was checked; the summary says so.
end

R.summary = render(R, cfg, offbook);
if cfg.render, fprintf('%s\n', R.summary); end
if cfg.plot,   drawCurves(R, cfg); end

if cfg.assert && cfg.benchmark
    % "Nothing was evaluated" must NOT read as "everything passed".
    if ~any(ev)
        error('mgs:zarchan:flightControlDelay:noBenchmark', ...
            ['not one book value could be evaluated.  The published numbers ' ...
             'live at alt = 0 ft, Vm = 3000 ft/s, a 10-g command, KR = 0.1 ' ...
             'and a 0.015 s delay; this run moved %s.  Refusing to hand back ' ...
             'an unvalidated port silently -- pass ''benchmark'', false if ' ...
             'you meant to run off-book.'], offbookStr(offbook));
    end
    if ~R.pass
        bad = find(~[R.bench.ok] & ev);
        error('mgs:zarchan:flightControlDelay:benchmark', ...
            ['%d of %d evaluated book values are outside tolerance (first: ' ...
             '%s, book %s, ours %s, band %s).  Either the port broke or the ' ...
             'parameters left the region the book covers.  Full board:\n\n%s'], ...
            numel(bad), sum(ev), R.bench(bad(1)).what, R.bench(bad(1)).book_str, ...
            R.bench(bad(1)).ours_str, R.bench(bad(1)).band, R.summary);
    end
end
end


% =============================================================================
%   CONFIGURATION
% =============================================================================
function N = bookNominal()
%BOOKNOMINAL  THE ONE PLACE the listing's data statement is written down.
%
%   Straight off zarchan.txt:7195-7204 (the body of Listing 22.4).  Change a
%   value here and you have changed what "the book" means for every benchmark
%   row in this file, which is the intended blast radius -- there is no
%   second copy anywhere.
N = struct();
N.alt   = 0;        % ALT=0
N.vm    = 3000;     % VM=3000
N.wgt   = 1000;     % WGT=1000
N.xncg  = 10;       % XNCG=10   (the step command AND the trim condition)
N.xkr   = 0.1;      % XKR=.1
N.wact  = 150;      % WACT=150
N.zact  = 0.7;      % ZACT=.7
N.delay = 0.015;    % DELAY=.015
N.h     = 0.0001;   % H=.0001
N.tstop = 0.99999;  % while ~(T > .99999)
N.tsamp = 0.00099999;  % if S > .00099999    -- see header note 3
% Not in the listing.  The margin bisections need a run long enough for a
% growth exponent to separate from the transient; 2 s was chosen because the
% neutral delay it returns is unchanged between 1.5 s and 2 s (0.0147 s) and
% moves by one step (0.0148 s) at 1 s.  Stated rather than tuned.
N.tmargin = 2.0;
end


function [cfg, offbook] = resolveConfig(opts, N)
%RESOLVECONFIG  Fill the empties from the book, and record what was moved.
%
%   offbook is the list of names the caller supplied with a value DIFFERENT
%   from the book's.  Supplying the book's own value is not "off book".
cfg = struct();
offbook = {};
f = fieldnames(N);
for k = 1:numel(f)
    nm = f{k};
    v  = opts.(nm);
    if isempty(v)
        cfg.(nm) = N.(nm);
    else
        cfg.(nm) = v;
        if ~isequal(cfg.(nm), N.(nm))
            offbook{end+1} = nm; %#ok<AGROW>
        end
    end
end
cfg.path      = opts.path;
cfg.margins   = opts.margins;
cfg.benchmark = opts.benchmark;
cfg.render    = opts.render;
cfg.plot      = opts.plot;
cfg.assert    = opts.assert;
if ~strcmp(cfg.path, 'delay'), offbook{end+1} = 'path'; end

% ---- the guards.  Each one names the plausible wrong answer it prevents ---
if cfg.h <= 0
    error('mgs:zarchan:flightControlDelay:step', 'h must be positive; got %g.', cfg.h);
end
if cfg.tstop <= 10*cfg.h
    error('mgs:zarchan:flightControlDelay:tstop', ...
        ['tstop = %g s is fewer than 10 steps at h = %g s.  The response ' ...
         'would be its own first transient and every derived quantity ' ...
         '(peaks, growth exponent) an artefact of the start-up.'], ...
        cfg.tstop, cfg.h);
end
% THE trap of this listing.  DINT = floor(DELAY/H); at DINT = 0 the book's
% index Z(I+2-DINT) = Z(I+2) points one slot PAST the newest sample, which is
% never written, so DELC reads 0 for the whole run.  The actuator never
% moves, XNL is exactly 0 at every sample, and the plot is a flat line -- the
% most stable-looking output this program can produce, from a loop that has
% been cut open.  Nothing downstream can tell that apart from "perfectly
% damped", so it is refused here.
if strcmp(cfg.path, 'delay') && cfg.delay < cfg.h
    error('mgs:zarchan:flightControlDelay:delayTooSmall', ...
        ['delay = %g s is smaller than h = %g s, so DINT = floor(delay/h) ' ...
         'is 0 and the book''s buffer index Z(I+2-DINT) reads a slot that ' ...
         'is never written.  DELC stays 0, the actuator never moves, and ' ...
         'the run returns XNL = 0 at every sample -- a flat line that looks ' ...
         'like a perfectly damped system rather than an open loop.  Use ' ...
         '''path'',''none'' if you want the delay removed on purpose.'], ...
        cfg.delay, cfg.h);
end
if strcmp(cfg.path, 'lag') && cfg.delay <= 0
    error('mgs:zarchan:flightControlDelay:lagTimeConstant', ...
        ['path = ''lag'' uses delay (%g s) as the lag time constant and ' ...
         'divides by it.'], cfg.delay);
end
if cfg.delay < 0
    error('mgs:zarchan:flightControlDelay:negativeDelay', ...
        'delay = %g s is negative; a non-causal loop is not a model of anything.', ...
        cfg.delay);
end
% The run exists to decide whether the loop is unstable, and Heun on a fast
% pole MANUFACTURES an instability -- the book says so in as many words at
% zarchan.txt:7032.  |lambda| = WACT for the actuator's complex pair, and
% two-pass RK loses stability near |lambda|*h ~ 2.  Book product: 0.015.
if cfg.wact*cfg.h > 0.5
    error('mgs:zarchan:flightControlDelay:actuatorStep', ...
        ['wact*h = %.3g.  The 2nd-order actuator is the fastest mode in the ' ...
         'loop and this two-pass (Heun) integrator goes unstable on it near ' ...
         'wact*h ~ 2, returning huge FINITE numbers that look exactly like ' ...
         'the physical instability this listing exists to show.  The book ' ...
         'runs wact = 150, h = 1e-4 (product 0.015) and warns about this at ' ...
         'zarchan.txt:7032.  Reduce h or lower wact.'], cfg.wact*cfg.h);
end
if cfg.tsamp < cfg.h
    error('mgs:zarchan:flightControlDelay:sample', ...
        ['tsamp = %g s is below the integration step h = %g s, so the ' ...
         'storage test fires every step and the "peaks in 1 s" count ' ...
         'becomes a count of integration steps.'], cfg.tsamp, cfg.h);
end
if cfg.xkr == 0
    % XKDC = (1-XKR*XK3)/(XK1*XKR) divides by XKR.
    error('mgs:zarchan:flightControlDelay:zeroGain', ...
        ['xkr = 0 opens the rate loop and makes XKDC infinite ' ...
         '(it carries 1/XKR).  There is no zero-gain member of this ' ...
         'family to fall back to.']);
end
if cfg.vm <= 1000
    % B = sqrt(XMACH^2-1) with the book's fixed a = 1000 ft/s.
    error('mgs:zarchan:flightControlDelay:subsonic', ...
        ['vm = %g ft/s is not supersonic against the book''s fixed speed of ' ...
         'sound (a = 1000 ft/s at EVERY altitude, deliberately kept).  The ' ...
         'Prandtl-Glauert factor sqrt(M^2-1) would be complex and the whole ' ...
         'linearisation is a supersonic one.'], cfg.vm);
end
if cfg.xncg <= 0
    % The trim quadratic takes sqrt(P3^2 + 4*P2*P1) with P1 proportional to
    % XNCG; a zero or negative command puts the missile at zero or negative
    % trim and the "10-g linearisation" stops being about anything.
    error('mgs:zarchan:flightControlDelay:command', ...
        'xncg = %g g; the aero is linearised AT the trim this command sets.', ...
        cfg.xncg);
end
if cfg.tmargin <= 4*cfg.h
    error('mgs:zarchan:flightControlDelay:tmargin', ...
        'tmargin = %g s is too short to hold two growth-exponent windows.', ...
        cfg.tmargin);
end
end


% =============================================================================
%   THE AIRFRAME.  Transcribed from Listing 22.4's preamble, which is
%   character-identical to Listings 22.1, 22.2 and 23.1-23.3.
% =============================================================================
function F = airframe(cfg, GRAV_FT)
%AIRFRAME  Geometry -> trim -> linearised aerodynamics -> autopilot gains.
%
%   The geometry is NOT exposed as options.  It is one specific vehicle (the
%   Ch.21 notional missile: 20 ft long, 1 ft diameter, 3-calibre nose, 1000
%   lb, cg at 10 ft, hinge line at 19.5 ft) and every book value on the board
%   belongs to it.  Making the fins adjustable would let a caller produce a
%   different airframe and still be scored against Table 22.1.

ALT  = cfg.alt;   VM = cfg.vm;   WGT = cfg.wgt;   XNCG = cfg.xncg;

A    = 1000;   % speed of sound, ft/s, at EVERY altitude.  The book's
               % approximation, deliberately kept: it is what makes XMACH,
               % and therefore B, and therefore every derivative below,
               % reproduce the published table.
DIAM = 1; FR = 3; XL = 20;
CTW  = 0; CRW = 6; HW = 2;      % wing:  tip chord, root chord, semi-span
CTT  = 0; CRT = 2; HT = 2;      % tail:  tip chord, root chord, semi-span
XN   = 4; XCG = 10; XHL = 19.5;

% Transcription note 1 (see header): '<' as it appears in Listing 22.4.
if (ALT < 30000)
    RHO = .002378*exp(-ALT/30000.);
else
    RHO = .0034*exp(-ALT/22000.);
end

SWING = .5*HW*(CTW+CRW);
STAIL = .5*HT*(CTT+CRT);
SREF  = 3.1416*DIAM*DIAM/4;     % the book's 3.1416, NOT pi.  Using pi moves
                                % SREF by 2e-6 relative, which is invisible
                                % here but is exactly the sort of silent
                                % edit that makes a digit-for-digit
                                % comparison stop being one.
XLP   = FR*DIAM;
SPLAN = (XL-XLP)*DIAM + 1.33*XLP*DIAM/2;
XCPN  = 2*XLP/3;
AN    = .67*XLP*DIAM;
AB    = (XL-XLP)*DIAM;
XCPB  = (.67*AN*XLP + AB*(XLP + .5*(XL-XLP)))/(AN+AB);
XCPW  = XLP + XN + .7*CRW - .2*CTW;
XMACH = VM/A;
XIYY  = WGT*(3*((DIAM/2)^2) + XL*XL)/(12*GRAV_FT);
TMP1  = (XCG-XCPW)/DIAM;
TMP2  = (XCG-XHL)/DIAM;
TMP3  = (XCG-XCPB)/DIAM;
TMP4  = (XCG-XCPN)/DIAM;
B     = sqrt(XMACH^2 - 1);
Q     = .5*RHO*VM*VM;
P1    = WGT*XNCG/(Q*SREF);      % the normal-force coefficient the command needs

Y1 = 2. + 8*SWING/(B*SREF) + 8*STAIL/(B*SREF);
Y2 = 1.5*SPLAN/SREF;
Y3 = 8*STAIL/(B*SREF);
Y4 = 2*TMP4 + 8*SWING*TMP1/(B*SREF) + 8*STAIL*TMP2/(B*SREF);
Y5 = 1.5*SPLAN*TMP3/SREF;
Y6 = 8*STAIL*TMP2/(B*SREF);
P2 = Y2 - Y3*Y5/Y6;
P3 = Y1 - Y3*Y4/Y6;

% Trim: CN = P1 and CM = 0 solved simultaneously.  ALFTR and DELTR come out
% in RADIANS -- the "2*alpha" term is the slender-body CN_alpha of 2 per
% radian.  STRUCTURALCHECK verifies both identities instead of trusting this
% paragraph.
ALFTR = (-P3 + sqrt(P3*P3 + 4.*P2*P1))/(2.*P2);
DELTR = -Y4*ALFTR/Y6 - Y5*ALFTR*ALFTR/Y6;

CNA  = 2 + 1.5*SPLAN*ALFTR/SREF + 8*SWING/(B*SREF) + 8*STAIL/(B*SREF);
CND  = 8*STAIL/(B*SREF);
CMAP = 2*TMP4 + 1.5*SPLAN*ALFTR*TMP3/SREF + 8*SWING*TMP1/(B*SREF);
CMA  = CMAP + 8*STAIL*TMP2/(B*SREF);
CMD  = 8*STAIL*TMP2/(B*SREF);

XMA = Q*SREF*DIAM*CMA/XIYY;
XMD = Q*SREF*DIAM*CMD/XIYY;
ZA  = -GRAV_FT*Q*SREF*CNA/(WGT*VM);
ZD  = -GRAV_FT*Q*SREF*CND/(WGT*VM);

if XMA >= 0
    % WAF = sqrt(-XMA).  A statically stable airframe has XMA < 0; a positive
    % one makes WAF imaginary and MATLAB would carry the complex number all
    % the way to a complex "acceleration" without a word.
    error('mgs:zarchan:flightControlDelay:staticallyUnstable', ...
        ['Ma = %.4g >= 0: this geometry is statically UNSTABLE at the ' ...
         'requested condition, so WAF = sqrt(-Ma) is imaginary and the ' ...
         'whole second-order airframe form stops applying.  MATLAB would ' ...
         'return complex accelerations rather than error.'], XMA);
end

WZ  = sqrt((XMA*ZD - ZA*XMD)/ZD);       % airframe zero (right-half-plane)
WAF = sqrt(-XMA);
ZAF = .5*WAF*ZA/XMA;
% 1845 = 57.3*32.2: rad->deg AND ft/s^2->g in one constant, which is why K1
% comes out in g/deg although ALFTR is radians.
XK1 = -VM*(XMA*ZD - XMD*ZA)/(1845*XMA);
TA  = XMD/(XMA*ZD - XMD*ZA);
XK3 = 1845*XK1/VM;

F = struct('alt',ALT, 'vm',VM, 'wgt',WGT, 'xncg',XNCG, ...
    'rho',RHO, 'q',Q, 'xmach',XMACH, 'b',B, 'sref',SREF, 'splan',SPLAN, ...
    'xiyy',XIYY, 'swing',SWING, 'stail',STAIL, ...
    'alftr',ALFTR, 'alftr_deg',57.3*ALFTR, 'deltr',DELTR, 'deltr_deg',57.3*DELTR, ...
    'cna',CNA, 'cnd',CND, 'cma',CMA, 'cmd',CMD, ...
    'xma',XMA, 'xmd',XMD, 'za',ZA, 'zd',ZD, ...
    'wz',WZ, 'waf',WAF, 'zaf',ZAF, 'xk1',XK1, 'ta',TA, 'xk3',XK3, ...
    'y1',Y1, 'y2',Y2, 'y3',Y3, 'y4',Y4, 'y5',Y5, 'y6',Y6, 'p1',P1);
end


function C = closedLoop(F, cfg)
%CLOSEDLOOP  The rate gyro closed-loop form the book derives on p.510
%   (zarchan.txt:7015-7022), plus the gain that would make the damping
%   exactly unity.
%
%       w    = wAF*sqrt(1 - KR*K3)
%       zeta = (w/2) * [ (2*zAF/wAF - KR*K3*Ta) / (1 - KR*K3) ]
%
%   These are NOT used by the simulation -- the loop integrates the states
%   directly.  They are here because the book's design sentence at
%   zarchan.txt:7025 ("an autopilot gain of approximately 0.1 is required to
%   increase the damping ... to unity") is a claim about THIS algebra, and
%   inverting it is a sharper test than checking that zeta(0.1) is near 1.
KR = cfg.xkr;
C = struct();
C.w        = F.waf*sqrt(1 - KR*F.xk3);
C.zeta     = 0.5*C.w*((2*F.zaf/F.waf - KR*F.xk3*F.ta)/(1 - KR*F.xk3));
C.w_over_waf = C.w/F.waf;
C.xkdc     = (1 - KR*F.xk3)/(F.xk1*KR);

% Invert zeta(KR) = 1 by bisection.  zeta is monotone increasing in KR over
% this range (numerator linear in KR, denominator sqrt), so a bracket check
% plus bisection is enough and no derivative is needed.
z = @(k) 0.5*F.waf*sqrt(1-k*F.xk3)*((2*F.zaf/F.waf - k*F.xk3*F.ta)/(1-k*F.xk3));
lo = 1e-4;  hi = 1.0;
if ~(z(lo) < 1 && z(hi) > 1)
    C.kr_for_unit_zeta = NaN;   % reported as such; never silently clamped
else
    for k = 1:60
        mid = 0.5*(lo+hi);
        if z(mid) > 1, hi = mid; else, lo = mid; end
    end
    C.kr_for_unit_zeta = 0.5*(lo+hi);
end
end


% =============================================================================
%   THE SIMULATION.  Listing 22.4's loop, transcribed.
% =============================================================================
function OUT = simulate(F, cfg, delaySec, XKR, path, tstop, keepFull, dintForce)
%SIMULATE  The listing's two-pass integration of the rate gyro loop.
%
%   PATH selects what sits between the autopilot output DELCP and the
%   actuator command DELC:
%     'delay'  the listing: a DINT-sample transport delay through Z
%     'none'   nothing (Listing 22.2 plus the actuator).  NOT a transcription
%              of Listing 22.2: 22.2 computes THD before DEL inside the pass
%              and this loop computes it after, so the second Heun pass here
%              uses a THD that is one pass old.  That is worth 1e-4 s of
%              extra phase (0.9 deg at 150 rad/s) and is stated rather than
%              papered over.
%     'lag'    a first-order lag of time constant delaySec in the same place
%
%   XKDC is recomputed from the XKR passed in.  Freezing it at the nominal
%   gain would leave the dynamics right and the steady state wrong, which
%   reads as a modelling error instead of a bookkeeping one.
%
%   DINTFORCE (optional, [] = the book's behaviour) sets the delay length in
%   SAMPLES directly instead of deriving it from delaySec.  MARGINS needs it,
%   and the reason is arithmetic, not taste: the book's DINT = floor(DELAY/H)
%   does not round-trip an integer.  MEASURED at H = 1e-4, 23 of the first
%   400 step counts come back one short -- 49*1e-4 divided by 1e-4 is
%   48.99999999999999, so floor() gives 48.  A bisection that assumed
%   "delay = n*h" meant "n samples" would report a delay margin one step off
%   for about 6% of the values it might land on, with nothing to show for it.
%   Callers using the book's parameterisation pass [] and get floor().

H     = cfg.h;
WACT  = cfg.wact;
ZACT  = cfg.zact;
XNCG  = cfg.xncg;
SAMP  = cfg.tsamp;

WAF = F.waf;  ZAF = F.zaf;  WZ = F.wz;  XK1 = F.xk1;  XK3 = F.xk3;  TA = F.ta;
XKDC = (1. - XKR*XK3)/(XK1*XKR);

if nargin < 8 || isempty(dintForce)
    DINT = floor(delaySec/H);       % the book's line, verbatim
else
    if ~isscalar(dintForce) || dintForce < 1 || dintForce ~= floor(dintForce)
        error('mgs:zarchan:flightControlDelay:dintForce', ...
            'dintForce must be a positive whole number of samples; got %s.', ...
            mat2str(dintForce));
    end
    DINT = dintForce;
    delaySec = DINT*H;              % delaySec is now DERIVED; the caller's
                                    % value (if any) is deliberately discarded
end
% String comparison inside a 20,000-iteration scalar loop is measurably
% slower than an integer test, and this function is called ~35 times by the
% bisections.  Resolved once, here.
switch path
case 'delay', pathId = 1;
case 'none',  pathId = 2;
case 'lag',   pathId = 3;
otherwise
    % Unreachable through the arguments block, reachable through an edit.
    % Without this, pathId would simply not exist and the failure would be
    % "Unrecognized function or variable 'pathId'" 40 lines later.
    error('mgs:zarchan:flightControlDelay:path', ...
        'unknown path "%s"; expected delay | none | lag.', path);
end
if pathId == 1 && DINT < 1
    error('mgs:zarchan:flightControlDelay:dintZero', ...
        'DINT = floor(%g/%g) = 0; see the delayTooSmall guard.', delaySec, H);
end

nStep = ceil(tstop/H) + 2;
% The book writes Z = zeros(size(1:20002)), which is exactly 2/H + 2 for its
% own H and nothing else.  MATLAB GROWS an array silently on an out-of-range
% ASSIGNMENT rather than erroring, so the literal would not have been wrong,
% only quietly reallocating on every step of a longer run -- and the READ,
% Z(I+2-DINT), never exceeds I+1 so it was always in range.  Sized from the
% run instead, for speed and so the number means something.
Z = zeros(1, nStep + DINT + 4);

nSamp = ceil(tstop/SAMP) + 2;
ArrayT    = zeros(1, nSamp);
ArrayXNL  = zeros(1, nSamp);
ArrayDEL  = zeros(1, nSamp);
ArrayDELC = zeros(1, nSamp);
if keepFull
    fullDELC  = zeros(1, nStep);
    fullDELCP = zeros(1, nStep);
else
    fullDELC = [];  fullDELCP = [];
end

count = 0;
E = 0; ED = 0; DELD = 0; DEL = 0; THD = 0; DELC = 0; DELCP = 0;
T = 0; Z(1) = 0; I = 1; S = 0;
% Declared before the loop because the Heun corrector below reads the SECOND
% pass's values after the inner loop has ended.
EDD = 0; DELDD = 0; XNL = 0;

while ~(T > tstop)
    S = S + H;
    EOLD = E;  EDOLD = ED;  DELOLD = DEL;  DELDOLD = DELD;
    STEP = 1;  FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            E    = E + H*ED;
            ED   = ED + H*EDD;
            DEL  = DEL + H*DELD;
            DELD = DELD + H*DELDD;
            T    = T + H;
            STEP = 2;
        end
        DELCP = XKR*(XKDC*XNCG + THD);
        DELDD = WACT*WACT*(DELC - DEL - 2.*ZACT*DELD/WACT);
        EDD   = WAF*WAF*(DEL - E - 2.*ZAF*ED/WAF);
        XNL   = XK1*(E - EDD/WZ^2);
        THD   = XK3*(E + TA*ED);
        FLAG  = 1;
    end
    FLAG = 0; %#ok<NASGU>  the book's line; dead but transcribed
    E    = .5*(EOLD + E + H*ED);
    ED   = .5*(EDOLD + ED + H*EDD);
    DEL  = .5*(DELOLD + DEL + H*DELD);
    DELD = .5*(DELDOLD + DELD + H*DELDD);

    Z(I+1) = DELCP;
    if pathId == 1
        % Transcription note 4/7 (see header).  Priming: until DINT samples
        % exist, the actuator sees Z(1) = 0.
        if (I+1) < DINT
            DELC = Z(1);
        else
            DELC = Z(I+2-DINT);
        end
    elseif pathId == 2
        DELC = DELCP;
    else
        % Explicit Euler on dDELC/dt = (DELCP - DELC)/T.  Deliberately the
        % crudest possible integration of the lag: it is here to answer "is
        % this loop stable with a lag of this size", and Euler's own error
        % is itself a small extra lag, i.e. it biases the answer TOWARDS
        % instability.  Finding stability anyway is therefore the strong
        % direction of the result.
        DELC = DELC + H*(DELCP - DELC)/delaySec;
    end
    if keepFull
        fullDELC(I)  = DELC;
        fullDELCP(I) = DELCP;
    end
    I = I + 1;

    if S > SAMP
        S = 0;
        count = count + 1;
        ArrayT(count)    = T;
        ArrayXNL(count)  = XNL;
        ArrayDEL(count)  = DEL;
        ArrayDELC(count) = DELC;
    end
end

% A run that stored fewer than three samples cannot be measured -- ppLast,
% the growth exponent and the peak count all reduce to properties of the
% storage grid -- and xnl(end) on an empty array is a bare index error 40
% lines from its cause.  count = 0 is reachable whenever tsamp > tstop.
if count < 3
    error('mgs:zarchan:flightControlDelay:noSamples', ...
        ['the run stored %d sample(s): tsamp = %g s against tstop = %g s.  ' ...
         'Every derived quantity below (peaks, growth exponent, ' ...
         'peak-to-peak) would be a property of the storage grid rather ' ...
         'than of the response.'], count, SAMP, tstop);
end

% The output struct is OUT, not S.  S is the book's sample-interval
% accumulator and is live right up to the last iteration above; reusing the
% name here worked, but it left a function in which S is a double for 20,000
% lines of execution and a struct afterwards.
OUT = struct();
OUT.path   = path;
OUT.xkr    = XKR;
OUT.xkdc   = XKDC;
OUT.dint   = DINT;
OUT.nstep  = I - 1;
OUT.t      = ArrayT(1:count);
OUT.xnl    = ArrayXNL(1:count);
OUT.del    = ArrayDEL(1:count);
OUT.delc   = ArrayDELC(1:count);
OUT.xncg   = XNCG + zeros(1, count);
OUT.full_delc  = fullDELC(1:min(numel(fullDELC),  OUT.nstep));
OUT.full_delcp = fullDELCP(1:min(numel(fullDELCP), OUT.nstep));
if pathId == 1
    OUT.delay_realized_s = DINT*H;  % verified by structuralCheck, not asserted
else
    OUT.delay_realized_s = 0;
end

% ---- derived measurements ------------------------------------------------
[OUT.npeaks, OUT.t_peak] = peaksOf(OUT.t, OUT.xnl);
OUT.w_osc   = oscFreq(OUT.t_peak);
OUT.sigma   = growthExponent(OUT.t, OUT.xnl, 0.25);
OUT.pp_last = ppLast(OUT.xnl, 0.25);
OUT.final   = OUT.xnl(end);
OUT.maxabs  = max(abs(OUT.xnl));
if isfinite(OUT.w_osc) && OUT.w_osc > 0
    OUT.samples_per_period = (2*pi/OUT.w_osc)/cfg.tsamp;
else
    OUT.samples_per_period = Inf;
end
end


% =============================================================================
%   MEASUREMENTS ON A RUN
% =============================================================================
function [n, tp] = peaksOf(t, y)
%PEAKSOF  Strict local maxima of the STORED array -- the same thing the
%   author counted off Fig. 22.17 ("there are 10 peaks in the response") and
%   Fig. 22.13 ("there are 24 peaks in 1 s").
%
%   NO PROMINENCE THRESHOLD, ON PURPOSE.  A threshold is a knob that can be
%   turned until the count comes out at 10, and the count is the sharpest
%   benchmark on this board.  The signal is a noise-free LTI response, so a
%   strict local maximum is unambiguous; the only thing that can corrupt the
%   count is ALIASING, which is why the caller checks samples-per-period
%   before scoring anything derived from it.
n = 0;  tp = zeros(1, 0);
for i = 2:numel(y)-1
    if y(i) > y(i-1) && y(i) >= y(i+1)
        n = n + 1;
        tp(n) = t(i); %#ok<AGROW>
    end
end
end


function w = oscFreq(tp)
%OSCFREQ  Oscillation frequency in rad/s from the MEAN peak spacing.
%   Deliberately independent of the count-over-window arithmetic the book
%   uses (10 peaks / 1 s = 10 Hz), so the two agreeing is evidence rather
%   than a restatement.
if numel(tp) < 3
    w = NaN;
else
    w = 2*pi*(numel(tp)-1)/(tp(end) - tp(1));
end
end


function s = growthExponent(t, y, w)
%GROWTHEXPONENT  Exponential growth rate of the oscillation, 1/s.
%
%   Total variation of y over two EQUAL, ADJACENT windows at the end of the
%   run; s = log(TV2/TV1)/dt.  Total variation is used rather than an
%   envelope fit because it needs no detrending and no peak picking: it is
%   dominated by the fastest-oscillating component and is blind to the DC
%   level, so the same estimator works whether the run settles at 10 g or
%   diverges through 20 g.
%
%   THE WINDOWS MUST BE EQUAL IN LENGTH.  Measured, during development: with
%   windows [0.50,0.70) and [0.70,1.00] of a diverging run this returns
%   +1.416 where the correct value is +0.319 -- a wrong number with the right
%   sign, which is the worst kind.  Hence the single fraction w and the
%   error below rather than two independent bounds.
if ~(w > 0 && w <= 0.5)
    error('mgs:zarchan:flightControlDelay:window', ...
        ['growth-exponent window fraction must be in (0, 0.5] so the two ' ...
         'windows are equal in length; got %g.'], w);
end
n = numel(y);
m = floor(w*n);
% Built backwards from the END of the record so both windows hold EXACTLY m
% differences.  Anchoring them at fractions of n instead gives m and m-1 for
% most n, which biases the ratio by 1/m before any physics enters.
i1 = n - 2*m;
i2 = n - m;
if m < 3 || i1 < 1
    s = NaN;  return
end
tv1 = sum(abs(diff(y(i1:i2))));
tv2 = sum(abs(diff(y(i2:n))));
dt  = t(i2) - t(i1);
if tv1 <= 0 || tv2 <= 0 || dt <= 0
    s = NaN;                      % NOT 0.  "flat" and "unmeasurable" differ.
else
    s = log(tv2/tv1)/dt;
end
end


function v = ppLast(y, frac)
%PPLAST  Peak-to-peak of the last FRAC of the record.  The amplitude measure
%   used by the delete experiment: it ignores the start-up transient (this
%   airframe has a right-half-plane zero and undershoots to about -2.8 g
%   before it ever rises), so a settled run reads ~0 and a diverging one
%   reads its current amplitude.
n = numel(y);
i = max(1, floor((1-frac)*n) + 1);
v = max(y(i:n)) - min(y(i:n));
end


% =============================================================================
%   MARGINS.  Measured on this loop, by bisection, against the book's
%   frequency-domain arithmetic.
% =============================================================================
function M = margins(F, cfg)
%MARGINS  How much pure delay, and how much extra gain, this loop tolerates.
%
%   Both are found by bracketing on the sign of the growth exponent and then
%   bisecting.  Two things are deliberate:
%
%   * The DELAY search bisects on DINT, an INTEGER.  The delay this program
%     can represent is quantised to h, so a real-valued bisection would
%     report six figures of a quantity that only has four, and would hide the
%     fact that the resolution is exactly h (0.0067 of the answer).
%
%   * Neither search silently returns a bracket endpoint.  If the loop is
%     already unstable with no delay, or still stable at the widest gain
%     tried, the field comes back NaN and the benchmark row that needs it
%     SKIPS with that reason.  A margin quietly reported as the end of the
%     search range is a number that looks measured and is not.
M = struct('skipped', false);
TS = cfg.tmargin;
MAXDOUBLE = 12;      % bracket doublings before giving up

% ---- delay margin at the book's gain -------------------------------------
%   The search variable is DINT, an integer number of samples.  It is NEVER
%   converted to seconds and back -- see SIMULATE's dintForce note.
sig0 = sigmaDelay(F, cfg, 1, cfg.xkr, TS);        % one sample of delay
if ~(sig0 < 0)
    error('mgs:zarchan:flightControlDelay:alreadyUnstable', ...
        ['the loop is already diverging at DINT = 1 (growth exponent ' ...
         '%+.4g 1/s), so there is no delay margin to measure.  At the ' ...
         'book''s parameters this loop is comfortably stable; check xkr, ' ...
         'wact and h.'], sig0);
end
dHi  = max(2, floor(cfg.delay/cfg.h));
grew = false;
for k = 1:MAXDOUBLE
    if sigmaDelay(F, cfg, dHi, cfg.xkr, TS) > 0
        grew = true;  break
    end
    dHi = 2*dHi;
end
M.delay_resolution_s = cfg.h;
if grew
    dLo = 1;
    while dHi - dLo > 1
        mid = floor((dLo + dHi)/2);
        if sigmaDelay(F, cfg, mid, cfg.xkr, TS) > 0
            dHi = mid;
        else
            dLo = mid;
        end
    end
    Sn = simulate(F, cfg, NaN, cfg.xkr, 'delay', TS, false, dHi);
    % Read the realised DINT back off the run rather than trusting dHi.  If
    % SIMULATE ever stops honouring dintForce, this row must go wrong loudly
    % instead of reporting the number that was asked for.
    M.dint_neutral    = Sn.dint;
    M.delay_neutral_s = Sn.delay_realized_s;
    if Sn.dint ~= dHi
        error('mgs:zarchan:flightControlDelay:dintIgnored', ...
            'asked for DINT = %d, the run realised %d.', dHi, Sn.dint);
    end
    M.w_at_delay_neutral = tailFreq(Sn);
    % The book's own arithmetic, run the other way round: the phase loss of
    % an ideal delay at the oscillation frequency is w*T (zarchan.txt:7189).
    M.phase_loss_rad = M.w_at_delay_neutral * M.delay_neutral_s;
    M.phase_loss_deg = 57.3 * M.phase_loss_rad;
else
    M.dint_neutral = NaN;  M.delay_neutral_s = NaN;
    M.w_at_delay_neutral = NaN;  M.phase_loss_rad = NaN;  M.phase_loss_deg = NaN;
    M.delay_why = sprintf(['no delay up to %g s destabilised the loop; the ' ...
        'bracket search gave up after %d doublings'], dHi*cfg.h, MAXDOUBLE);
end

% ---- gain margin, delay path removed -------------------------------------
kLo = cfg.xkr;
if ~(sigmaGain(F, cfg, kLo, TS) < 0)
    error('mgs:zarchan:flightControlDelay:alreadyUnstableGain', ...
        ['the loop with NO delay is already diverging at xkr = %g, so there ' ...
         'is no gain margin to measure.'], kLo);
end
kHi = 2*kLo;  grewK = false;
for k = 1:MAXDOUBLE
    if sigmaGain(F, cfg, kHi, TS) > 0
        grewK = true;  break
    end
    kHi = 2*kHi;
end
if grewK
    for k = 1:24                  % 24 halvings: resolution ~6e-8 of the bracket
        mid = 0.5*(kLo + kHi);
        if sigmaGain(F, cfg, mid, TS) > 0
            kHi = mid;
        else
            kLo = mid;
        end
    end
    M.kr_neutral     = 0.5*(kLo + kHi);
    M.gain_margin_db = 20*log10(M.kr_neutral/cfg.xkr);
    Sk = simulate(F, cfg, cfg.delay, M.kr_neutral, 'none', TS, false, []);
    M.w_at_kr_neutral = tailFreq(Sk);
else
    M.kr_neutral = NaN;  M.gain_margin_db = NaN;  M.w_at_kr_neutral = NaN;
    M.gain_why = sprintf(['no gain up to %g destabilised the delay-free ' ...
        'loop after %d doublings'], kHi, MAXDOUBLE);
end
end


function s = sigmaDelay(F, cfg, dint, XKR, tstop)
S = simulate(F, cfg, NaN, XKR, 'delay', tstop, false, dint);
s = S.sigma;
end


function s = sigmaGain(F, cfg, XKR, tstop)
S = simulate(F, cfg, cfg.delay, XKR, 'none', tstop, false, []);
s = S.sigma;
end


function w = tailFreq(S)
%TAILFREQ  Ringing frequency from the LAST HALF of a run, so the start-up
%   transient (which for this right-half-plane-zero airframe is a large
%   negative excursion, not an oscillation) cannot contribute peaks.
n = numel(S.t);
i = max(1, floor(n/2));
[~, tp] = peaksOf(S.t(i:n), S.xnl(i:n));
w = oscFreq(tp);
end


function L = lagProbe(F, cfg)
%LAGPROBE  The comparison this port exists for: a FIRST-ORDER LAG in the same
%   place as the delay, at the delay's own time constant and at 10x and 100x
%   it.
%
%   No book value scores this -- Zarchan never contrasts the two -- so it
%   lives in the structural section and in the printed report, NOT on the
%   benchmark board.  Reporting it as a "book benchmark" would be inventing
%   a citation, which is the exact failure mode this package was built
%   against.
%
%   Run length scales with the lag so a slow lag is not scored on its own
%   transient.
L = struct('skipped', false);
Ts = cfg.delay * [1 10 100];
L.tlag   = Ts;
L.sigma  = zeros(1, numel(Ts));
L.stable = false(1, numel(Ts));
L.final  = zeros(1, numel(Ts));
for k = 1:numel(Ts)
    ts = max(cfg.tmargin, 6*Ts(k));
    S  = simulate(F, cfg, Ts(k), cfg.xkr, 'lag', ts, false, []);
    L.sigma(k)  = S.sigma;
    L.stable(k) = isfinite(S.sigma) && S.sigma < 0;
    L.final(k)  = S.final;
end
L.all_stable = all(L.stable);
end


% =============================================================================
%   STRUCTURAL SELF-CHECK.  Wiring, not the book.  A failure here voids
%   every benchmark row above it.
% =============================================================================
function C = structuralCheck(F, cfg, R)
C = struct();

% ---- 1. the trim solution really is the commanded 10 g -------------------
%   CN(alpha_tr, delta_tr) must equal WGT*XNCG/(Q*SREF) exactly.  This is the
%   check that says ALFTR is in radians: at any other unit the identity is
%   off by 57.3 or 57.3^2 and every derivative below it is wrong while still
%   looking like an airframe.
cn = F.y1*F.alftr + F.y2*F.alftr^2 + F.y3*F.deltr;
C.trim_cn_resid = abs(cn/F.p1 - 1);
C.trim_cn_tol   = 1e-12;
C.trim_cn_ok    = C.trim_cn_resid <= C.trim_cn_tol;

% ---- 2. and it really is a TRIM (zero moment) ----------------------------
cm = F.y4*F.alftr + F.y5*F.alftr^2 + F.y6*F.deltr;
C.trim_cm_resid = abs(cm)/max(abs(F.y6*F.deltr), eps);
C.trim_cm_tol   = 1e-12;
C.trim_cm_ok    = C.trim_cm_resid <= C.trim_cm_tol;

% ---- 3. the delay buffer really delays by DINT samples -------------------
%   THE resolution of the transcription's Z(I+2-DINT).  Replay the recorded
%   actuator command against the recorded autopilot output and demand BIT
%   equality at exactly DINT samples of offset.  The tolerance is zero: these
%   are the same doubles moved through an array, so anything but exact
%   equality means the index is wrong.  An off-by-one here still produces a
%   loop that oscillates and diverges -- just at the wrong margin -- which no
%   amount of looking at the plot would catch.
if strcmp(R.run.path, 'delay') && ~isempty(R.run.full_delc)
    d  = R.run.dint;
    dc = R.run.full_delc;  cp = R.run.full_delcp;
    idx = d:numel(dc);                     % 1-based: DELC(k) == DELCP(k-d+1)
    C.buffer_violations = sum(dc(idx) ~= cp(idx - d + 1));
    C.buffer_checked    = numel(idx);
    C.buffer_ok         = C.buffer_violations == 0 && C.buffer_checked > 100;
    C.buffer_skipped    = false;
else
    C.buffer_violations = NaN;  C.buffer_checked = 0;
    C.buffer_ok = false;  C.buffer_skipped = true;
end

% ---- 4. DC gain identity, with the delay path removed --------------------
%   XKDC = (1-KR*K3)/(K1*KR) makes the achieved acceleration equal the
%   command in the steady state.  Exercises K1 -> K3 -> KDC in one number.
C.dc_err = abs(R.nodelay.final/cfg.xncg - 1);
C.dc_tol = 1e-5;
C.dc_ok  = C.dc_err <= C.dc_tol;

% ---- 5. THE DELETE EXPERIMENT -------------------------------------------
%   Remove the delay and the divergence must go away.  Without this row,
%   "the response grows" is equally consistent with a broken integrator, a
%   wrong sign on the rate feedback, or a numerically unstable actuator --
%   all of which would grow with the delay removed too.
%
%   Only meaningful when the run under test IS the delayed one.  A caller
%   who asked for path='none' would otherwise be comparing a run against
%   itself and getting a ratio of 1, i.e. a FAILURE reported for a correct
%   port.  Skipped, with the reason, instead.
onDelay = strcmp(R.run.path, 'delay');
C.delete_pp_delay   = R.run.pp_last;
C.delete_pp_nodelay = R.nodelay.pp_last;
C.delete_ratio      = R.run.pp_last / max(R.nodelay.pp_last, realmin);
C.delete_tol        = 1e3;
C.delete_skipped    = ~onDelay;
C.delete_ok         = onDelay && C.delete_ratio >= C.delete_tol;

% ---- 6. LAG IS NOT DELAY -------------------------------------------------
%   The same time constant, the same place in the loop, opposite verdicts.
if isfield(R.lagprobe, 'all_stable') && onDelay
    C.lag_all_stable = R.lagprobe.all_stable;
    C.lag_sigma      = R.lagprobe.sigma;
    C.delay_sigma    = R.run.sigma;
    C.lag_ok = C.lag_all_stable && isfinite(R.run.sigma) && R.run.sigma > 0;
    C.lag_skipped = false;
else
    C.lag_all_stable = false;  C.lag_sigma = [];  C.delay_sigma = NaN;
    C.lag_ok = false;  C.lag_skipped = true;
end

C.ok = C.trim_cn_ok && C.trim_cm_ok && C.dc_ok && ...
       (C.delete_ok || C.delete_skipped) && ...
       (C.buffer_ok  || C.buffer_skipped) && ...
       (C.lag_ok     || C.lag_skipped);
end


% =============================================================================
%   BOOK VALUES.  THE ONE PLACE a published number is written down.
% =============================================================================
function V = bookValues()
%BOOKVALUES  Every number in this file that came out of the book, with the
%   exact text it was grepped from.
%
%   RULE OBSERVED HERE, AT COST: a value that can be grepped out of
%   zarchan.txt is usable; a value that lives only on a figure does not
%   exist, because the extraction turned every figure into an image link.
%   What the book says in PROSE about a figure IS greppable and IS used --
%   as counts, shape rows and orderings, which are frequently sharper than a
%   level read off a plot.  An ADJECTIVE is reported as INFO and not scored.
%
%   Everything below was grepped from
%   matlab/zarchan/_source/zarchan.txt at the line quoted.

V = struct();

% -- Table 22.1, "MISSILE TRANSFER FUNCTION PARAMETERS AT TWO FLIGHT
%    CONDITIONS", p.501, sea-level column.  zarchan.txt:6934-6937.
%    The OCR renders a leading minus as '2': "23.07 g/deg" is -3.07.
V.waf     = 25.3;      % :6934  "vAF Airframe natural frequency 25.3 rad/s"
V.zaf     = 0.058;     % :6935  "zAF Airframe damping 0.058"
V.wz      = 43.2;      % :6936  "vz Airframe zero 43.2 rad/s"
V.k1_mag  = 3.07;      % :6937  "K1 Aerodynamic acceleration gain 23.07 g/deg"

% -- Table 22.2, "ADDITIONAL LINEARIZED AERODYNAMICS", p.507, sea level.
V.ta      = 0.457;     % :6976  "Ta Turning rate time constant 0.457 s"
V.k3      = -1.89;     % :6977  "K3 Aerodynamic body rate gain 21.89 1/s"
%    K3's sign is NOT inferred from that OCR'd '2'.  Listing 22.3 hard-codes
%    it: zarchan.txt:7150 "K3=-1.89; TA=.457; ZAF=.058; WAF=25.3; KR=.1".
%    That line is also an independent confirmation of the other three.
%    K1 has no such confirmation -- the book's PROSE formula for K1 has the
%    opposite sign to the listing's XK1 -- so K1 is compared on MAGNITUDE.

% -- Table 23.2, "LINEARIZED AERODYNAMIC PARAMETERS FOR DIFFERENT FLIGHT
%    CONDITIONS", p.551, zarchan.txt:7738-7740, "0 kft" row.  Same airframe,
%    same Mach, same 10-g trim; Ch.23 simply tabulates the derivatives that
%    Ch.22 leaves inside the listing.  "2642 s22" is -642 s^-2.
V.xma = -642;  V.xmd = -555;  V.za = -2.94;  V.zd = -0.65;

% -- p.518-519, the Bode plot of Fig. 22.16 read in prose.  zarchan.txt:7159
%    "At the gain crossover frequency ... the phase is 2125 deg ... the phase
%    margin is 55 deg (1802 125 = 55).  At the phase crossover frequency ...
%    the gain is 211.5 dB ... the gain margin is 11.5 dB."
%    zarchan.txt:7164 "the gain and phase crossover frequencies are 64 rad/s
%    and 150 rad/s respectively."
V.phase_margin_deg = 55;
V.gain_margin_db   = 11.5;
V.w_gain_cross     = 64;
V.w_phase_cross    = 150;

% -- p.519, zarchan.txt:7166: "A gain increase of 11.5 dB means that KR must
%    increase from 0.1 to 0.376 to destabilize the system."
V.kr_unstable = 0.376;

% -- p.520, zarchan.txt:7189: "a delay of 0.015 s in the time domain
%    corresponds to a phase loss of 55 deg (64 x 0.015 = 0.96 rad = 55 deg =
%    phase margin) ... if a pure delay of 0.015 s were inserted in series
%    with KR, the system would go unstable and oscillate at a frequency of
%    64 rad/s".
V.delay_margin_s   = 0.015;
V.phase_loss_rad   = 0.96;

% -- p.521, zarchan.txt:7190, THE measurement Fig. 22.17 is quoted for:
%    "Because there are 10 peaks in the response, the frequency of
%    oscillation is 10 Hz or 62.8 rad/s, which is approximately the gain
%    crossover frequency."  The 62.8 IS the count divided by the window, so
%    it inherits the count's +/-1 quantisation (+/-10%).
V.peaks_delay      = 10;
V.w_osc_delay      = 62.8;

% -- p.513, zarchan.txt:7058, the GAIN route with no delay: "If we increase
%    the autopilot gain to 0.4, we can see from Fig. 22.13 that the
%    flight-control system response goes unstable.  Because there are 24
%    peaks in 1 s, the frequency of oscillation is 24 Hz".  Confirmed in
%    rad/s at zarchan.txt:7174: "when the autopilot gain was increased to 0.4
%    the rate gyro flight-control system indeed oscillated at 150 rad/s."
V.peaks_gain04 = 24;
V.xkr_gain04   = 0.4;

% -- p.509, zarchan.txt:7025: "At sea level an autopilot gain of
%    approximately 0.1 is required to increase the damping of the rate gyro
%    flight-control system to unity".  Also :7055, "An autopilot gain of 0.1
%    (XKR = 0.1) was chosen at sea level ... to ensure that the flight-
%    control system damping was approximately unity."
V.kr_for_unit_zeta = 0.1;

% -- zarchan.txt:7025, same sentence, tail: "the autopilot gain does not
%    cause the natural frequency of the flight-control system to be
%    significantly different from the airframe's natural frequency."
%    An ADJECTIVE.  Reported, never scored -- see the row that carries it.
end


% =============================================================================
function B = benchmarks(R, cfg, offbook)
%BENCHMARKS  Computed vs published, with a band, a reason for the band, and
%   the table/figure/page each value came from.
%
%   FOUR TOLERANCE CLASSES, and the difference is PROVENANCE, not taste:
%     'table'  1%   printed in a table to three significant figures
%     'text'   2%   printed in running prose to two or three figures, or
%                   recomputed there from already-rounded table entries
%     'approx' 5%   the book says "approximately", or the value was read off
%                   a Bode plot
%     'graph' 15%   the author read a level off a plot with no gridline near
%   plus 'shape' and 'bound', which are inequalities and orderings taken from
%   prose and carry no tolerance at all.
%
%   ONE RULE, DECIDED ONCE, FOR THIS CHAPTER.  Seven of the numbers here
%   trace to one source: the author reading Fig. 22.16.  The phase margin
%   (55 deg = 180 - 125), the gain margin (11.5 dB), the two crossover
%   frequencies (64 and 150 rad/s), and the two values computed FROM them in
%   the prose (0.96 rad, 0.015 s, 0.376) are all Bode-plot readings, however
%   exact the arithmetic downstream of them looks.  They ALL get 'approx',
%   even the ones this port happens to match to a fraction of a percent.
%   Assigning a 2% band to a plot reading because our number is close enough
%   to survive it is the same mistake as widening one because it is not.
%
%   Nothing in this file uses 'text' as a result.  That branch stays in
%   BANDTOL for a future row with genuine prose provenance; it is not an
%   oversight.
%
%   Do NOT widen a band to turn a row green.

V = bookValues();
B = emptyBench();

GATE_AERO = {'alt','vm','wgt','xncg'};
GATE_LOOP = [GATE_AERO, {'wact','zact','h'}];
GATE_RUN  = [GATE_LOOP, {'xkr','delay','tstop','tsamp','path'}];
GATE_DM   = [GATE_LOOP, {'xkr','tmargin'}];      % delay margin: xkr fixed, delay swept
GATE_GM   = [GATE_LOOP, {'tmargin'}];            % gain margin: xkr swept, delay removed

wA = gateWhy(offbook, GATE_AERO);
wR = gateWhy(offbook, GATE_RUN);
wD = gateWhy(offbook, GATE_DM);
wG = gateWhy(offbook, GATE_GM);

seaLevel = (cfg.alt == 0);
whySea   = 'the published column is SEA LEVEL only (alt = 0)';

% =========================== A. the airframe =============================
src221 = 'Table 22.1 p.501 (zarchan.txt:6934)';
src222 = 'Table 22.2 p.507 + Listing 22.3 (zarchan.txt:6976,7150)';
src232 = 'Table 23.2 p.551, 0 kft row (zarchan.txt:7740)';

B = addAero(B, 'wAF airframe nat. freq',  V.waf,    R.aero.waf,  'rad/s', src221, wA, seaLevel, whySea, ...
    'Listing 22.3 hard-codes WAF=25.3 at zarchan.txt:7150, independently of the table');
B = addAero(B, 'zAF airframe damping',    V.zaf,    R.aero.zaf,  '-',     src221, wA, seaLevel, whySea, ...
    'the low damping is what the rate gyro loop exists to raise');
B = addAero(B, 'wZ airframe zero (RHP)',  V.wz,     R.aero.wz,   'rad/s', src221, wA, seaLevel, whySea, ...
    'right-half-plane: it is why the step response undershoots to about -2.8 g first');
B = addAero(B, '|K1| accel gain',         V.k1_mag, abs(R.aero.xk1), 'g/deg', src221, wA, seaLevel, whySea, ...
    'MAGNITUDE: the book''s prose formula for K1 has the opposite sign to the listing''s XK1');
B = addAero(B, 'Ta turning-rate const',   V.ta,     R.aero.ta,   's',     src222, wA, seaLevel, whySea, ...
    'confirmed with an explicit sign by Listing 22.3 (TA=.457)');
B = addAero(B, 'K3 body-rate gain',       V.k3,     R.aero.xk3,  '1/s',   src222, wA, seaLevel, whySea, ...
    'SIGNED: Listing 22.3 hard-codes K3=-1.89, so the table''s OCR''d minus is not being guessed');
B = addAero(B, 'Ma pitch moment / alpha', V.xma,    R.aero.xma,  '1/s^2', src232, wA, seaLevel, whySea, ...
    'Ch.23 tabulates the derivatives Ch.22 leaves inside the listing; same airframe, same trim');
B = addAero(B, 'Md pitch moment / delta', V.xmd,    R.aero.xmd,  '1/s^2', src232, wA, seaLevel, whySea, '');
B = addAero(B, 'Za normal force / alpha', V.za,     R.aero.za,   '1/s',   src232, wA, seaLevel, whySea, '');
B = addAero(B, 'Zd normal force / delta', V.zd,     R.aero.zd,   '1/s',   src232, wA, seaLevel, whySea, '');

% ====================== B. the delayed run itself ========================
srcFig17 = 'Fig. 22.17 / p.521 prose (zarchan.txt:7190)';
countable = R.run.samples_per_period >= 8;
whyAlias  = sprintf(['the stored record holds %.1f samples per oscillation ' ...
    'period; below 8 the peak COUNT aliases and returns a plausible integer'], ...
    R.run.samples_per_period);

% All three branches emit the SAME FOUR ROW NAMES.  A board whose row set
% changes with the configuration cannot be diffed against a previous run,
% and a check that vanishes is indistinguishable from a check that passed.
whyCount = '';
if ~isempty(wR)
    whyCount = wR;
elseif ~countable
    whyCount = whyAlias;
end
if ~isempty(whyCount)
    B = addSkip(B, 'peaks in the 1 s response', sprintf('%d', V.peaks_delay), ...
        whyCount, srcFig17, 'peaks');
    B = addSkip(B, 'oscillation freq (delay)', sprintf('%g', V.w_osc_delay), ...
        whyCount, srcFig17, 'rad/s');
    B = addSkip(B, 'osc freq vs gain crossover', sprintf('%g', V.w_gain_cross), ...
        whyCount, 'p.520 (zarchan.txt:7189,7164)', 'rad/s');
else
    B = addVal(B, 'peaks in the 1 s response', V.peaks_delay, R.run.npeaks, 'table', ...
        srcFig17, 'peaks', ...
        ['the author''s own measurement off the figure.  A count is an ' ...
         'integer, so the 1% band means EXACTLY 10 and nothing else']);
    B = addVal(B, 'oscillation freq (delay)', V.w_osc_delay, R.run.w_osc, 'approx', ...
        srcFig17, 'rad/s', ...
        ['measured from MEAN PEAK SPACING, not from the count, so agreeing ' ...
         'with the book is evidence and not a restatement.  The book''s 62.8 ' ...
         'is 10 peaks / 1 s and therefore quantised to +/-10%; this 5% band ' ...
         'is already tighter than its own resolution']);
    B = addVal(B, 'osc freq vs gain crossover', V.w_gain_cross, R.run.w_osc, 'approx', ...
        'p.520 (zarchan.txt:7189,7164)', 'rad/s', ...
        ['the section''s claim: a loop destabilised by DELAY rings at the ' ...
         'GAIN crossover frequency, not the phase crossover.  64 rad/s is a ' ...
         'Fig. 22.16 reading -- see the provenance rule in this function''s ' ...
         'header']);
end
% The divergence itself needs neither the sample rate nor the count, so it is
% scored whenever the run IS the book's delayed one.
if ~isempty(wR)
    B = addSkip(B, 'delayed run diverges', 'unstable', wR, srcFig17, '-');
else
    B = addShape(B, 'delayed run diverges', 'unstable (Fig 22.17)', ...
        sprintf('sigma %+.3f 1/s', R.run.sigma), ...
        isfinite(R.run.sigma) && R.run.sigma > 0, srcFig17, ...
        ['"the system does go unstable when a delay of 0.015 s is inserted".  ' ...
         'Paired with the delete experiment in the structural block, which ' ...
         'is what rules out a broken integrator growing for its own reasons']);
end

% ====================== C. the delay margin ==============================
srcPM = 'p.520 (zarchan.txt:7189)';
haveM = isfield(R.margin, 'delay_neutral_s') && isfinite(R.margin.delay_neutral_s);
% Three different reasons this group can go unscored, and they must not print
% alike: off-book, margins switched off, or a bisection that found no bracket.
% The old fallback said "margins were not run" for all three.
if ~isempty(wD)
    whyM = wD;
elseif ~isfield(R.margin, 'delay_neutral_s')
    whyM = 'margins were not run (''margins'', false)';
elseif isfield(R.margin, 'delay_why')
    whyM = R.margin.delay_why;
else
    whyM = 'the delay margin came back non-finite';
end
if ~haveM || ~isempty(wD)
    B = addSkip(B, 'delay margin',            sprintf('%g', V.delay_margin_s), whyM, srcPM, 's');
    B = addSkip(B, 'phase loss w*T at neutral', sprintf('%g', V.phase_loss_rad), whyM, srcPM, 'rad');
    B = addSkip(B, 'that phase loss = PM',    sprintf('%g', V.phase_margin_deg), whyM, srcPM, 'deg');
    B = addSkip(B, 'ring freq at neutral delay', sprintf('%g', V.w_gain_cross), whyM, ...
        'p.518 (zarchan.txt:7164)', 'rad/s');
else
    B = addVal(B, 'delay margin', V.delay_margin_s, R.margin.delay_neutral_s, 'approx', ...
        srcPM, 's', ...
        ['THE headline of this listing, measured by bisecting DINT on the ' ...
         'sign of the growth exponent.  5% because the book''s 0.015 s is ' ...
         'itself 55 deg / 57.3 / 64 rad/s and both of those were read off ' ...
         'Fig. 22.16 to two figures -- that rounding alone is +/-1.7%.  Our ' ...
         'own resolution is one step, h = 1e-4 s, i.e. 0.7%']);
    B = addVal(B, 'phase loss w*T at neutral', V.phase_loss_rad, R.margin.phase_loss_rad, ...
        'approx', srcPM, 'rad', ...
        ['the book''s arithmetic run backwards: OUR neutral delay times OUR ' ...
         'measured ringing frequency.  Nothing from the Bode plot enters it']);
    B = addVal(B, 'that phase loss = PM', V.phase_margin_deg, R.margin.phase_loss_deg, ...
        'approx', 'p.518,520 (zarchan.txt:7159,7189)', 'deg', ...
        ['closes the loop between the two domains: the amount of phase a ' ...
         'pure delay costs at the ringing frequency IS the phase margin']);
    B = addVal(B, 'ring freq at neutral delay', V.w_gain_cross, R.margin.w_at_delay_neutral, ...
        'approx', 'p.518 (zarchan.txt:7164)', 'rad/s', ...
        'the gain crossover frequency, reached entirely from the time domain');
end

% ====================== D. the gain margin route =========================
srcGM = 'p.519 (zarchan.txt:7166)';
haveG = isfield(R.margin, 'kr_neutral') && isfinite(R.margin.kr_neutral);
if ~isempty(wG)
    whyG = wG;
elseif ~isfield(R.margin, 'kr_neutral')
    whyG = 'margins were not run (''margins'', false)';
elseif isfield(R.margin, 'gain_why')
    whyG = R.margin.gain_why;
else
    whyG = 'the gain margin came back non-finite';
end
if ~haveG || ~isempty(wG)
    B = addSkip(B, 'KUNSTABLE (no delay)', sprintf('%g', V.kr_unstable), whyG, srcGM, '-');
    B = addSkip(B, 'ring freq at KUNSTABLE', sprintf('%g', V.w_phase_cross), whyG, srcGM, 'rad/s');
    B = addInfo(B, 'gain margin in dB', sprintf('%g dB', V.gain_margin_db), '--', ...
        'p.518 (zarchan.txt:7159)', whyG);
else
    B = addVal(B, 'KUNSTABLE (no delay)', V.kr_unstable, R.margin.kr_neutral, 'approx', ...
        srcGM, '-', ...
        ['the OTHER way to destabilise the same loop.  The book gets 0.376 ' ...
         'from 10^(11.5/20)*0.1 with 11.5 dB read off Fig. 22.16; 5% on the ' ...
         'gain is 0.42 dB of plot-reading, which is about the best a log ' ...
         'plot supports']);
    B = addVal(B, 'ring freq at KUNSTABLE', V.w_phase_cross, R.margin.w_at_kr_neutral, ...
        'approx', 'p.518,519 (zarchan.txt:7164,7166)', 'rad/s', ...
        ['the PHASE crossover.  This is the row that makes the pair mean ' ...
         'something: same loop, same code, two instabilities, two different ' ...
         'frequencies']);
    B = addInfo(B, 'gain margin in dB', sprintf('%g dB', V.gain_margin_db), ...
        sprintf('%.3g dB', R.margin.gain_margin_db), 'p.518 (zarchan.txt:7159)', ...
        ['the same measurement as KUNSTABLE expressed logarithmically; ' ...
         'scoring both would double-count one bisection']);
end

% ------ the KR = 0.4 case, Fig. 22.13 ------------------------------------
srcF13 = 'Fig. 22.13 / p.513 (zarchan.txt:7058)';
if ~isempty(wG)
    B = addSkip(B, 'peaks in 1 s at KR = 0.4', sprintf('%d', V.peaks_gain04), wG, srcF13, 'peaks');
    B = addSkip(B, 'osc freq at KR = 0.4',     sprintf('%g', V.w_phase_cross), wG, srcF13, 'rad/s');
elseif ~isempty(R.gain04) && R.gain04.samples_per_period >= 8 && abs(cfg.tstop - 1) < 0.01
    B = addVal(B, 'peaks in 1 s at KR = 0.4', V.peaks_gain04, R.gain04.npeaks, 'table', ...
        srcF13, 'peaks', ...
        ['delay path removed, gain raised: "Because there are 24 peaks in ' ...
         '1 s".  An integer, so 1% means exactly 24']);
    B = addVal(B, 'osc freq at KR = 0.4', V.w_phase_cross, R.gain04.w_osc, 'approx', ...
        'p.519 (zarchan.txt:7174)', 'rad/s', ...
        '"indeed oscillated at 150 rad/s" -- the phase crossover again');
else
    B = addSkip(B, 'peaks in 1 s at KR = 0.4', sprintf('%d', V.peaks_gain04), ...
        'the count is over a 1 s window at the book sample rate', srcF13, 'peaks');
    B = addSkip(B, 'osc freq at KR = 0.4', sprintf('%g', V.w_phase_cross), ...
        'same window requirement', srcF13, 'rad/s');
end

% ------ the ordering that is the whole point of the section --------------
if haveM && haveG && isempty(wD) && isempty(wG) && ...
        isfinite(R.margin.w_at_delay_neutral) && isfinite(R.margin.w_at_kr_neutral)
    ratio_book = V.w_phase_cross/V.w_gain_cross;
    ratio_ours = R.margin.w_at_kr_neutral/R.margin.w_at_delay_neutral;
    B = addShape(B, 'delay rings slower than gain', ...
        sprintf('%.2fx (150/64)', ratio_book), sprintf('%.2fx', ratio_ours), ...
        R.margin.w_at_kr_neutral > R.margin.w_at_delay_neutral, ...
        'p.524 (zarchan.txt:7211)', ...
        ['"When the system oscillates ... due to the increased gain, the ' ...
         'frequency of oscillation will be the PHASE crossover frequency ... ' ...
         'due to the phase loss (that is, time delay), the frequency ... will ' ...
         'be the ... GAIN crossover frequency."  A port that matched both ' ...
         'peak counts at ONE frequency would have this backwards and still ' ...
         'look right']);
else
    B = addSkip(B, 'delay rings slower than gain', '150 > 64 rad/s', ...
        'needs both margins measured on-book', 'p.524 (zarchan.txt:7211)', '-');
end

% ====================== E. the damping design point ======================
srcDes = 'p.509 (zarchan.txt:7025)';
if ~isempty(wA)
    B = addSkip(B, 'KR for unity FCS damping', '~0.1', wA, srcDes, '-');
elseif ~isfinite(R.fcs.kr_for_unit_zeta)
    B = addSkip(B, 'KR for unity FCS damping', '~0.1', ...
        'zeta(KR) = 1 has no root in (1e-4, 1] for this airframe', srcDes, '-');
else
    kr = R.fcs.kr_for_unit_zeta;
    B = addBound(B, 'KR for unity FCS damping', 'approx 0.1 (+/-10%)', ...
        sprintf('%.4f', kr), abs(kr/V.kr_for_unit_zeta - 1) <= 0.10, srcDes, '-', ...
        ['scored by INVERTING the book''s closed form rather than by ' ...
         'checking zeta(0.1) against 1, because the book chose the round ' ...
         'number 0.1 and not the gain that makes zeta exactly unity.  ' ...
         'at the book''s own KR = 0.1 this airframe gives zeta = 1.057, ' ...
         'which an "approximately" 5% band would FAIL on a correct port -- ' ...
         'widening that band to 6% to go green is exactly the move this ' ...
         'file refuses, so the row is posed the other way round']);
end
B = addInfo(B, 'FCS nat freq vs airframe''s', ...
    'not significantly different', ...
    sprintf('w/wAF = %.3f (%.2f vs %.2f rad/s)', R.fcs.w_over_waf, R.fcs.w, R.aero.waf), ...
    srcDes, ...
    ['the book''s word is an ADJECTIVE ("not significantly different").  ' ...
     'Turning +9.0% into a pass or a fail would be inventing a threshold ' ...
     'the book never states, so this is reported and not scored']);
end


% =============================================================================
%   BENCH-ROW CONSTRUCTORS.  Field names and order match
%   mgs.zarchan.adjointWithAutopilot exactly so benchSuite's adapter for that
%   port transfers unchanged.
% =============================================================================
function B = emptyBench()
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end


function B = addAero(B, what, book, ours, units, source, whyOff, seaLevel, whySea, detail)
%ADDAERO  One airframe row, with the two gates it always needs.
if ~isempty(whyOff)
    B = addSkip(B, what, sprintf('%g', book), whyOff, source, units);
elseif ~seaLevel
    B = addSkip(B, what, sprintf('%g', book), whySea, source, units);
else
    B = addVal(B, what, book, ours, 'table', source, units, detail);
end
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
case 'approx', t = 0.05;   % "approximately", or a quarter-dB off a Bode plot
case 'graph',  t = 0.15;   % a level read off a plot
otherwise
    error('mgs:zarchan:flightControlDelay:band', ...
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


function s = offbookStr(offbook)
if isempty(offbook)
    s = '(nothing -- this is the book configuration)';
else
    s = strjoin(offbook, ', ');
end
end


% =============================================================================
%   REPORT
% =============================================================================
function s = render(R, cfg, offbook)
W = 100;
L = {};
L{end+1} = 'Zarchan Listing 22.4 -- RATE GYRO FLIGHT-CONTROL SYSTEM WITH A PURE TRANSPORT DELAY';
L{end+1} = '*** UNITS ARE FEET, ft/s, deg, g = 32.2 ft/s^2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['alt = %g ft   Vm = %g ft/s   Wgt = %g lb   command = %g g   ' ...
                    'KR = %g'], cfg.alt, cfg.vm, cfg.wgt, cfg.xncg, cfg.xkr);
L{end+1} = sprintf(['actuator %g rad/s / %g   |   h = %g s   tstop = %g s   ' ...
                    'sample = %g s'], cfg.wact, cfg.zact, cfg.h, cfg.tstop, cfg.tsamp);
L{end+1} = sprintf('path = %-5s  delay = %g s -> DINT = %d steps -> realised %g s', ...
    cfg.path, cfg.delay, R.run.dint, R.run.delay_realized_s);
L{end+1} = sprintf('off-book    : %s', offbookStr(offbook));
L{end+1} = repmat('=', 1, W);

% ---- airframe ------------------------------------------------------------
L{end+1} = 'AIRFRAME -- Ch.21 notional missile, linearised at its 10-g trim';
L{end+1} = sprintf('  %9s %9s %9s %9s %9s %9s %9s %9s', ...
    'Q[psf]', 'Ma[1/s2]', 'Md[1/s2]', 'Za[1/s]', 'Zd[1/s]', 'wZ', 'wAF', 'zAF');
L{end+1} = sprintf('  %9.1f %9.3f %9.3f %9.4f %9.5f %9.3f %9.4f %9.5f', ...
    R.aero.q, R.aero.xma, R.aero.xmd, R.aero.za, R.aero.zd, ...
    R.aero.wz, R.aero.waf, R.aero.zaf);
L{end+1} = sprintf('  %9s %9s %9s %9s %9s %9s %9s %9s', ...
    'K1[g/dg]', 'Ta[s]', 'K3[1/s]', 'KDC', 'alf_tr', 'del_tr', 'w_cl', 'zeta_cl');
L{end+1} = sprintf('  %9.4f %9.4f %9.4f %9.4f %8.3fd %8.3fd %9.3f %9.4f', ...
    R.aero.xk1, R.aero.ta, R.aero.xk3, R.fcs.xkdc, ...
    R.aero.alftr_deg, R.aero.deltr_deg, R.fcs.w, R.fcs.zeta);
L{end+1} = '  alf_tr/del_tr printed in DEGREES; the states themselves are radians (see header).';
L{end+1} = '  a = 1000 ft/s at EVERY altitude -- the book''s approximation, deliberately kept.';
L{end+1} = repmat('-', 1, W);

% ---- the run -------------------------------------------------------------
L{end+1} = sprintf('THE RUN (path = %s) -- Fig. 22.17', cfg.path);
L{end+1} = sprintf('  peaks in the %g s window        %8d', cfg.tstop, R.run.npeaks);
L{end+1} = sprintf('  ringing frequency              %8.2f rad/s  (%.3f Hz, from mean peak spacing)', ...
    R.run.w_osc, R.run.w_osc/(2*pi));
L{end+1} = sprintf('  growth exponent                %+8.3f 1/s    -> %s', ...
    R.run.sigma, divergeStr(R.run.sigma));
L{end+1} = sprintf('  peak-to-peak, last quarter     %8.4g g', R.run.pp_last);
L{end+1} = sprintf('  final / max |XNL|              %8.4g / %.4g g   (command %g g)', ...
    R.run.final, R.run.maxabs, cfg.xncg);
L{end+1} = sprintf('  stored samples per period      %8.1f   (peak counting aliases below 8)', ...
    R.run.samples_per_period);
if ~isempty(R.nodelay)
    L{end+1} = sprintf('  SAME LOOP, DELAY PATH REMOVED  pp %.4g g, settles at %.6f g', ...
        R.nodelay.pp_last, R.nodelay.final);
end
L{end+1} = repmat('-', 1, W);

% ---- margins -------------------------------------------------------------
if isfield(R.margin, 'delay_neutral_s')
    M = R.margin;
    L{end+1} = 'MARGINS MEASURED ON THIS LOOP (bisection on the sign of the growth exponent)';
    if isfinite(M.delay_neutral_s)
        L{end+1} = sprintf(['  delay margin      %8.5f s  (DINT %d, resolution %g s)   ' ...
                            'book 0.015 s'], M.delay_neutral_s, M.dint_neutral, M.delay_resolution_s);
        L{end+1} = sprintf(['  ...rings at       %8.2f rad/s                            ' ...
                            'book 64 (gain crossover)'], M.w_at_delay_neutral);
        L{end+1} = sprintf(['  ...phase eaten    %8.4f rad = %.2f deg                  ' ...
                            'book 0.96 rad = 55 deg'], M.phase_loss_rad, M.phase_loss_deg);
    else
        L{end+1} = sprintf('  delay margin      NOT FOUND: %s', M.delay_why);
    end
    if isfinite(M.kr_neutral)
        L{end+1} = sprintf(['  KUNSTABLE (no dly)%8.4f     (= %+.2f dB on KR)            ' ...
                            'book 0.376 (11.5 dB)'], M.kr_neutral, M.gain_margin_db);
        L{end+1} = sprintf(['  ...rings at       %8.2f rad/s                            ' ...
                            'book 150 (phase crossover)'], M.w_at_kr_neutral);
    else
        L{end+1} = sprintf('  gain margin       NOT FOUND: %s', M.gain_why);
    end
    L{end+1} = '  Two ways to break one loop, and they ring at DIFFERENT frequencies: that is';
    L{end+1} = '  the whole content of p.524 and the reason both are measured here.';
else
    L{end+1} = 'MARGINS  ***NOT RUN*** (''margins'', false) -- groups C and D of the board skip';
end
L{end+1} = repmat('-', 1, W);

% ---- lag is not delay ----------------------------------------------------
L{end+1} = 'LAG IS NOT DELAY -- why this port matters to us';
if isfield(R.lagprobe, 'tlag')
    P = R.lagprobe;
    L{end+1} = sprintf('  pure delay    T = %8.5f s   sigma %+8.3f 1/s   %s', ...
        cfg.delay, R.run.sigma, divergeStr(R.run.sigma));
    for k = 1:numel(P.tlag)
        L{end+1} = sprintf('  1st-order lag T = %8.5f s   sigma %+8.3f 1/s   %s', ...
            P.tlag(k), P.sigma(k), divergeStr(P.sigma(k))); %#ok<AGROW>
    end
    L{end+1} = '  Same place in the loop, same time constant, opposite verdicts -- and the lag stays';
    L{end+1} = '  stable at 10x and 100x the delay that kills it.  The lag buys its phase loss by';
    L{end+1} = '  throwing gain away and dragging the crossover down; the delay pays nothing.';
    L{end+1} = '  OUR ~8 s gamma-tracking deficit (gain 0.063) was fitted as a gain plus a settling';
    L{end+1} = '  time, which does NOT distinguish the two.  Nothing here says which one we have --';
    L{end+1} = '  it says the question has an answer and that we have not asked it.';
else
    L{end+1} = '  not run (''margins'', false).';
end
L{end+1} = repmat('-', 1, W);

% ---- structural ----------------------------------------------------------
if isfield(R.selfcheck, 'ok')
    C = R.selfcheck;
    L{end+1} = 'STRUCTURAL SELF-CHECK (wiring, not the book -- a failure here voids everything)';
    L{end+1} = sprintf('  trim reproduces the %g-g command CN     : %9.3e  tol %.0e  %s', ...
        cfg.xncg, C.trim_cn_resid, C.trim_cn_tol, tf2s(C.trim_cn_ok));
    L{end+1} = sprintf('  trim moment is zero                    : %9.3e  tol %.0e  %s', ...
        C.trim_cm_resid, C.trim_cm_tol, tf2s(C.trim_cm_ok));
    if C.buffer_skipped
        L{end+1} = '  delay buffer realises DINT steps       :        --  (path is not ''delay'')';
    else
        L{end+1} = sprintf('  delay buffer realises DINT steps       : %5d bad / %d  tol 0     %s', ...
            C.buffer_violations, C.buffer_checked, tf2s(C.buffer_ok));
    end
    L{end+1} = sprintf('  DC gain = 1 with the delay removed     : %9.3e  tol %.0e  %s', ...
        C.dc_err, C.dc_tol, tf2s(C.dc_ok));
    if C.delete_skipped
        L{end+1} = '  DELETE EXPERIMENT                      :        --  (path is not ''delay'')';
    else
        L{end+1} = sprintf('  DELETE EXPERIMENT: pp delay / no delay : %9.3e  tol >=%.0e %s', ...
            C.delete_ratio, C.delete_tol, tf2s(C.delete_ok));
    end
    if C.lag_skipped
        L{end+1} = '  lag of the same T is stable, delay is not:        --  (margins not run)';
    else
        L{end+1} = sprintf('  lag of the same T stable, delay is not : %9s  tol --    %s', ...
            boolStr(C.lag_all_stable), tf2s(C.lag_ok));
    end
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
    L{end+1} = sprintf('  %-30s %-16s %-24s %8s %-7s %s', ...
        'what', 'book', 'ours', 'diff', 'band', 'verdict');
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch b.kind
        case 'value'
            if b.skipped
                L{end+1} = sprintf('  %-30s %-16s %-24s %8s %-7s SKIP', ...
                    trunc(b.what,30), trunc(b.book_str,16), '--', '--', b.band); %#ok<AGROW>
            else
                L{end+1} = sprintf('  %-30s %-16s %-24s %+7.2f%% %-7s %s', ...
                    trunc(b.what,30), trunc([b.book_str ' ' b.units],16), ...
                    trunc([b.ours_str ' ' b.units],24), ...
                    b.pct, bandTag(b.band, b.tol), tf2s(b.ok)); %#ok<AGROW>
            end
        case {'shape','bound'}
            L{end+1} = sprintf('  %-30s %-16s %-24s %8s %-7s %s', ...
                trunc(b.what,30), trunc(b.book_str,16), trunc(b.ours_str,24), ...
                '--', b.band, tf2s(b.ok)); %#ok<AGROW>
        case 'info'
            L{end+1} = sprintf('  %-30s %-16s %-24s %8s %-7s INFO', ...
                trunc(b.what,30), trunc(b.book_str,16), trunc(b.ours_str,24), ...
                '--', 'info'); %#ok<AGROW>
        otherwise
            L{end+1} = sprintf('  %-30s %-16s %-24s %8s %-7s SKIP', ...
                trunc(b.what,30), trunc(b.book_str,16), '--', '--', 'skip'); %#ok<AGROW>
        end
    end
    L{end+1} = '  bands: tbl 1% = table (3 sig figs) or an integer count.  apx 5% = every number';
    L{end+1} = '         the author read off the Bode plot of Fig. 22.16, and everything the prose';
    L{end+1} = '         computes FROM those readings -- 55 deg, 11.5 dB, 64 and 150 rad/s, 0.96 rad,';
    L{end+1} = '         0.015 s, 0.376.  shape/bound = a prose inequality, no tolerance at all.';
    L{end+1} = '  every row''s table/figure/page and zarchan.txt line is in R.bench(i).source.';
    idx = find([R.bench.skipped] | ~[R.bench.ok]);
    if ~isempty(idx)
        L{end+1} = '  --- rows not scored green, and why ---';
        for j = idx(:).'
            b = R.bench(j);
            r = b.why;
            if isempty(r) && ~b.ok, r = ['FAILED: ' b.detail]; end
            if isempty(r), r = '(no reason recorded)'; end
            L{end+1} = sprintf('    %-30s %s', trunc(b.what,30), trunc(r, 62)); %#ok<AGROW>
            L{end+1} = sprintf('    %-30s src %s', '', b.source); %#ok<AGROW>
        end
    end
end
L{end+1} = repmat('-', 1, W);
L{end+1} = 'This is a 1000-lb supersonic missile airframe with a rate gyro autopilot.  It is NOT';
L{end+1} = 'a model of our vehicle: no gravity, no drag, no thrust, no nz clamp, no LOS floor and';
L{end+1} = 'no saturating lag.  A margin off this board is a check on our integrator and on the';
L{end+1} = 'lag/delay distinction, never a number to quote about a sortie.';
s = strjoin(L, newline);
end


function drawCurves(R, cfg)
figure('Name', 'Zarchan Listing 22.4 -- rate gyro FCS with pure delay');
plot(R.run.t, R.run.xnl, 'LineWidth', 1.2); hold on
plot(R.run.t, R.run.xncg, '--', 'LineWidth', 1);
if ~isempty(R.nodelay)
    nk = R.nodelay.t <= cfg.tstop;
    plot(R.nodelay.t(nk), R.nodelay.xnl(nk), ':', 'LineWidth', 1.2);
    legend({'achieved (with delay)', 'commanded', 'achieved (delay removed)'}, ...
        'Location', 'best');
else
    legend({'achieved', 'commanded'}, 'Location', 'best');
end
grid on
xlabel('Time (s)');
ylabel('Acceleration (g)');
title(sprintf('Fig. 22.17: rate gyro FCS, delay = %g s (DINT = %d)', ...
    cfg.delay, R.run.dint));
end


% =============================================================================
%   SMALL HELPERS
% =============================================================================
function s = tf2s(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end

function s = boolStr(v)
if v, s = 'true'; else, s = 'false'; end
end

function s = divergeStr(sig)
if ~isfinite(sig)
    s = 'not measurable';
elseif sig > 0
    s = 'DIVERGING';
else
    s = 'decaying';
end
end

function s = bandTag(band, tol)
switch lower(band)
case 'table',  s = 'tbl 1%';
case 'text',   s = 'txt 2%';
case 'approx', s = 'apx 5%';
case 'graph',  s = 'fig 15%';
otherwise,     s = sprintf('%.3g%%', 100*tol);
end
end

function s = trunc(s, n)
if isstring(s), s = char(s); end
if numel(s) > n, s = [s(1:n-2) '..']; end
end
