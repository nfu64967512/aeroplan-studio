function R = adjointWithAutopilot(opts)
%MGS.ZARCHAN.ADJOINTWITHAUTOPILOT  Zarchan Listing 23.3 -- adjoint of the
%   homing loop with a DETAILED three-loop flight-control system, ported line
%   for line.
%
%   R = mgs.zarchan.adjointWithAutopilot()              % book run + benchmarks
%   R = mgs.zarchan.adjointWithAutopilot('benchmark', false, 'alt', 50000)
%   R = mgs.zarchan.adjointWithAutopilot('benchmark', false, ...
%           'miss', 2, 'tint', 0.5)                     % rms fin rate at tgo
%
%   Fifteen-state adjoint of Fig. 23.15: seeker track loop, noise filter,
%   radome feedback, a three-loop autopilot with its own rate/accelerometer/
%   integrator gains, a second-order actuator, and a second-order airframe
%   with a right-half-plane zero.  Four error sources are propagated to rms
%   miss distance (Table 23.5, Figs. 23.16-23.18), rms fin rate (Figs.
%   23.19-23.21) and rms commanded acceleration (Figs. 23.22-23.24).
%
% ===========================================================================
%   *** UNITS ARE THE BOOK'S.  FEET, ft/s, g = 32.2 ft/s^2, DEGREES. ***
%
%   Zarchan works in feet with g = 32.2 and converts rad->deg with the
%   literal 57.3.  The rest of mgs is metres.  This file is DELIBERATELY left
%   in the book's units so the published numbers can be compared digit for
%   digit with no conversion sitting in between to be got wrong.  A miss
%   distance of "15.6" is plausible in either unit, so a silent conversion
%   produces a number that looks right and is wrong by 3.28x.  Convert at the
%   CALL SITE, visibly.  Fin deflections and fin rates are DEGREES and deg/s
%   (K1 is g/deg), accelerations are g, frequencies rad/s, times seconds.
% ===========================================================================
%
%   WHY THIS PORT IS THE ONE THIS PROJECT CAME FOR
%
%   Every other lag we have ported is a LUMPED time constant -- one pole
%   (Listing 3.1) or five equal poles (Listing 6.1).  You choose T and the
%   loop obeys.  Our own vehicle does not work like that: we MEASURED a
%   gamma-tracking gain of 0.063 against the model's 1.385, roughly 8 s of
%   lag, saturating on 26 of 32 sorties -- and we cannot derive that 8 s from
%   anything, only fit it.
%
%   Listing 23.3 is the first structure in the book where the lag is not a
%   parameter but a RESULT.  The autopilot has real gains (KA, wI, KR), a
%   real actuator (150 rad/s, zeta 0.7) and a real airframe (wAF, zAF, wZ)
%   derived from geometry and flight condition, and the closed-loop time
%   constant FALLS OUT:
%
%       tau_TOT = tau + 2*zeta/w        (zarchan.txt:7535, p.538)
%
%   At the book's sea-level condition that is 0.3 + 1.4/35.4 = 0.34 s, and
%   the step response reaches 63% of the command in 0.337 s.  Two independent
%   computations of the same lag, agreeing.  That is the shape of answer we
%   need for our own 8 s, and R.lag collects it.
%
%   The transferable result is stronger than the number.  Zarchan's gain
%   algorithm HOLDS tau_TOT fixed across a 50-kft altitude sweep while the
%   airframe underneath it changes by a factor of six (Ma goes -642 -> -99.1),
%   and Table 23.5 shows the miss distance barely moving as a consequence.
%   Our lag is NOT held fixed by anything -- ArduPilot's pitch loop has no
%   such scheduler -- which is a concrete, checkable difference between the
%   two systems rather than a vague "our autopilot is worse".
%
%   WHAT THIS DOES NOT MODEL, AND MUST NOT BE REPORTED AS IF IT DID.
%   Constant speed, constant altitude, no gravity, no drag, no thrust, no
%   acceleration limit, LINEARIZED aero about a 10-g trim, and a SUPERSONIC
%   airframe (Mach 3).  The disturbances are three seeker noises and a
%   uniformly distributed target maneuver.  Our bullseye is STATIONARY and
%   our measured discriminant is release-side heading error |eps0|, which is
%   an initial-condition channel that does not appear in this adjoint at all.
%   A miss distance computed here is NOT a sortie prediction.
%
% ---------------------------------------------------------------------------
%   PROVENANCE, AND WHAT WAS PULLED IN FROM NEIGHBOURING LISTINGS
%
%   The transcription is matlab/zarchan/_listings/listing_23_03.m, from
%   zarchan.txt:7823-7849.  Two OCR comparisons in it were damaged the usual
%   way (',' for '<'), both resolved from context and both unambiguous:
%       while TP,=(TF - 1e-5)   ->  while TP <= (TF - 1e-5)   (TP grows to TF)
%       while STEP,=1           ->  while STEP <= 1           (STEP is 1 then 2)
%   Nothing else in the listing needed a decision.  The book's very long
%   semicolon-separated lines are the original formatting, not damage.
%
%   Listings 23.1, 23.2 and 23.3 each carry a BYTE-IDENTICAL copy of the
%   aerodynamic and gain-algorithm preamble, so AIRFRAME() below is 23.3's
%   own preamble, not an import.  23.3 stands alone and needs nothing.
%
%   PULLED IN ANYWAY, AND SAID SO: the five-state closed-loop step response
%   from Listing 23.1 (zarchan.txt:7551-7556), verbatim, as FCSSTEP() below.
%   23.3 does not need it.  It is here because it is the only thing in this
%   chapter that MEASURES the flight-control time constant instead of
%   asserting one, which is the number this project came for, and because it
%   is an independent check on gains the adjoint only exercises indirectly.
%   Nothing from Listing 23.2 (the Bode sweep) was taken.
%
% ---------------------------------------------------------------------------
%   THE THING MOST LIKELY TO BE GOT WRONG: WHICH NOISE IS WHICH
%
%   The listing's own variable names do NOT line up with Table 23.4's row
%   names, and getting this backwards swaps two ~2 ft columns that both look
%   plausible.  Settled from the CODE, not from the names:
%
%     XMFN  <- X15, integrand  Y1PZ^2                 PHIFN = 8e-8  rad^2/Hz
%           angular noise with NO range weighting  =  "RANGE INDEPENDENT
%           NOISE" of Table 23.4 (0.002 rad in 0.01 s -> 2*.01*.002^2 = 8e-8)
%     XMRN  <- X14, integrand (Y1PZ*VC*TGO/RA)^2      PHIRN = 2e-6  rad^2/Hz
%           angular noise scaled by range/RA        =  "SEMIACTIVE NOISE"
%           (0.01 rad AT 30 kft in 0.01 s -> 2*.01*.01^2 = 2e-6, RA = 30000)
%     XMGL  <- X13, integrand (Y1PZ/(VC*TGO))^2       PHIGL = 20    ft^2/Hz
%           a LINEAR error at the target, divided by range to become an angle
%     XMUDNT<- X12, integrand X1^2, scaled XNT*sqrt(X12/tgo)
%           uniformly distributed target maneuver, PHI = nT^2/tF = 64.4^2/5
%
%   Zarchan's own Listing 18.1 labels these "Fading Noise" (FN) and "Missile
%   Semiactive Noise" (RN) on the plot axes (zarchan.txt:5393) while Table
%   18.1 calls FN the range-independent one -- fading noise and range
%   independent noise are the same thing in his vocabulary.  The port keeps
%   the book's variable names and prints the Table 23.4 row name beside each,
%   so neither convention can be read as the other.  The benchmark proves the
%   mapping: it reproduces 2.01 and 2.27 in the right columns to 0.3%.
%
% ---------------------------------------------------------------------------
%   MODES.  MISS IS AN INITIAL CONDITION, NOT A POST-PROCESSING CHOICE.
%
%     miss = 1, tint = 0     X3 = 1              rms MISS DISTANCE      [ft]
%     miss = 2, tint = tgo   X8 = 1              rms FIN RATE        [deg/s]
%     miss = 3, tint = tgo   X6 = N'*Vc/32.2     rms CMD ACCELERATION    [g]
%
%   The adjoint runs from TP = tint to TP = tf, so the answer is the quantity
%   at time-to-go = tint, i.e. at flight time tf - tint.  Modes 2 and 3 are
%   SINGULAR at tint = 0: the glint channel carries 1/(Vc*tgo) and the very
%   first step then divides by 0.08 ft, producing a large finite number that
%   is pure discretization.  This function ERRORS on tint = 0 in modes 2 and
%   3 rather than returning it.  The book only ever quotes those modes "at a
%   particular time to go" (zarchan.txt:7782).
%
% ---------------------------------------------------------------------------
%   BENCHMARK MODE IS A FLAG.  IT IS NOT nargin.
%
%   In a function whose arguments block declares ONLY name-value options,
%   MATLAB reports nargin = 0 NO MATTER HOW MANY were passed (measured on
%   R2025b).  Three ports in the previous wave used `if nargin == 0`: one
%   became infinite recursion and two silently replaced the caller's
%   arguments with the benchmark run.  'benchmark' is the ONLY switch here.
%
%   It defaults TRUE, so a bare call is a CHECKED call.  Benchmark mode does
%   not override anything the caller passed; instead every book value that
%   depends on a parameter the caller moved is SKIPPED with the reason
%   printed.  That is why the book defaults live in BOOKNOMINAL() and the
%   arguments-block defaults are [] -- "the caller supplied this" is then
%   exactly "the field is non-empty", with no second copy of the constants to
%   drift out of agreement.
%
%   COST: the full board is 75 adjoint integrations of 50,000 steps plus 5
%   step responses -- expect a minute or two, not seconds.  If that is in the
%   way, 'h', 1e-3 cuts it tenfold; h is deliberately NOT gated as an
%   off-book parameter because it was measured to move every Table 23.5 entry
%   by under 1 part in 1e4 between 1e-4 and 1e-3.  Do not go much coarser:
%   the guard on wact*h is the real limit and it is there for a reason.
%
% ---------------------------------------------------------------------------
%   OUTPUT
%
%   R.lag           the reason this file exists, one column per flight
%                   condition: .tau_design_s, .tau_total_s (the p.538 closed
%                   form), .t63_measured_s (the Listing 23.1 step response),
%                   .dc_gain (an identity, must be 1), .settled_frac (how far
%                   the 1-s run got), .t63_vs_last_sample_s (the mis-reading,
%                   carried so it can be recognised) and .note
%   R.fc(i)         one per altitude: aero (.xma .xmd .za .zd .wz .waf .zaf
%                   .xk1 .ta .xk3), gains (.xka .wi .xkr .xkdc .w .w0 .z0),
%                   .trim, .step (the Listing 23.1 response), .sweep (rms vs
%                   radome slope, four channels + rms)
%   R.finrate       .tgo, .alt and per-altitude matrices of the four channels
%   R.accel         same shape, mode 3
%   R.tau05         the tau = 0.5 s comparisons of Figs. 23.18/23.21/23.24
%   R.bench         computed vs book, one row per published value, each
%                   carrying .source (table/figure + zarchan.txt line)
%   R.pass          true iff every EVALUATED row passed
%   R.selfcheck     structural invariants and their measured residuals
%   R.cfg           the resolved configuration; R.offbook names what the
%                   caller moved away from the book
%   R.summary       the printed text
%
%   Bench rows carry BOTH numeric (.book/.ours, NaN on shape rows) and text
%   (.book_str/.ours_str, always populated) so a suite adapter can render
%   value rows and shape rows through one path.
%
%   See also MGS.ZARCHAN.BENCHSUITE, MGS.ZARCHAN.ADJOINTBINOMIAL5,
%   MGS.ZARCHAN.ADJOINTSINGLELAG.

arguments
    % Defaults are [] on purpose -- see BOOKNOMINAL().  Empty means "use the
    % book value"; non-empty means "the caller chose this", which is what
    % gates the benchmark rows.  Do NOT replace these with literals: that
    % puts a second copy of every book constant in the file and destroys the
    % only mechanism this port has for telling an off-book run apart from a
    % broken one.
    opts.alt       double {mustBeReal, mustBeFinite} = []   % ALT    [ft]
    opts.radome    double {mustBeReal, mustBeFinite} = []   % R      radome slope [-]
    opts.miss      double {mustBeReal, mustBeFinite} = []   % MISS   1|2|3
    opts.tint      double {mustBeReal, mustBeFinite} = []   % TINT   [s] time-to-go of the output
    opts.tf        double {mustBeReal, mustBeFinite} = []   % TF     [s]
    opts.vm        double {mustBeReal, mustBeFinite} = []   % VM     [ft/s]
    opts.vc        double {mustBeReal, mustBeFinite} = []   % VC     [ft/s]
    opts.tau       double {mustBeReal, mustBeFinite} = []   % TAU    [s]  FCS time constant goal
    opts.wcr       double {mustBeReal, mustBeFinite} = []   % WCR    [rad/s] crossover goal
    opts.zeta      double {mustBeReal, mustBeFinite} = []   % ZETA   [-]  FCS damping goal
    opts.xnp       double {mustBeReal, mustBeFinite} = []   % XNP    N'
    opts.t1        double {mustBeReal, mustBeFinite} = []   % T1     [s]  seeker track loop
    opts.t2        double {mustBeReal, mustBeFinite} = []   % T2     [s]  noise filter
    opts.wact      double {mustBeReal, mustBeFinite} = []   % WACT   [rad/s]
    opts.zact      double {mustBeReal, mustBeFinite} = []   % ZACT   [-]
    opts.xnt       double {mustBeReal, mustBeFinite} = []   % XNT    [ft/s^2] target maneuver, 2 g
    opts.ra        double {mustBeReal, mustBeFinite} = []   % RA     [ft] semiactive reference range
    opts.phirn     double {mustBeReal, mustBeFinite} = []   % PHIRN  [rad^2/Hz] semiactive
    opts.phigl     double {mustBeReal, mustBeFinite} = []   % PHIGL  [ft^2/Hz]  glint
    opts.phifn     double {mustBeReal, mustBeFinite} = []   % PHIFN  [rad^2/Hz] range independent
    opts.wgt       double {mustBeReal, mustBeFinite} = []   % WGT    [lb]
    opts.xncg      double {mustBeReal, mustBeFinite} = []   % XNCG   [g] trim / step command
    % h is EXCLUDED from the off-book gating below.  It is a discretization
    % knob, not a model parameter, and it was measured (against a
    % bit-identical transcription of this loop run outside MATLAB) to move
    % every Table 23.5 entry by less than 1 part in 1e4 between h = 1e-4 and
    % h = 1e-3.  What h CAN do is destabilise the 150 rad/s actuator, and
    % that is guarded explicitly rather than left to a tolerance.
    opts.h         double {mustBeReal, mustBeFinite} = []   % H      [s]
    opts.benchmark (1,1) logical = true    % THE ONLY SWITCH.  Not nargin.
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

GRAV_FT = 32.2;    % g, the book's value.  Not 9.80665, not 9.81.

N = bookNominal();
[cfg, offbook] = resolveConfig(opts, N);

R = struct();
R.units    = 'ft, ft/s, deg, deg/s, g = 32.2 ft/s^2 -- Zarchan''s units, NOT SI';
R.listing  = 'Zarchan Listing 23.3 (zarchan.txt:7823), adjoint of Fig. 23.15';
R.opts     = opts;
R.cfg      = cfg;
R.offbook  = offbook;
R.nominal  = N;

% ---- flight conditions: aero, gains, step response, radome sweep ----------
nA = numel(cfg.alt);
for i = 1:nA
    F = airframe(cfg.alt(i), cfg, GRAV_FT);
    F.step  = fcsStep(F, cfg, GRAV_FT);
    F.sweep = radomeSweep(F, cfg);
    if i == 1
        R.fc = F;
    else
        R.fc(i) = F; %#ok<AGROW>
    end
end

% ---- the lag summary: the point of the exercise ---------------------------
R.lag = lagSummary(R.fc, cfg);

% ---- auxiliary modes, the tau = 0.5 s comparison, and the board -----------
if cfg.benchmark
    R.finrate   = modeSweep(R.fc, cfg, 2, cfg.tgo_fin);
    R.accel     = modeSweep(R.fc, cfg, 3, cfg.tgo_acc);
    R.tau05     = tauComparison(cfg, GRAV_FT);
    R.selfcheck = structuralCheck(cfg, GRAV_FT);
    R.bench     = benchmarks(R, cfg, offbook);
    ev          = ~[R.bench.skipped];
    R.pass      = any(ev) && all([R.bench(ev).ok]);
else
    R.finrate   = [];
    R.accel     = [];
    R.tau05     = [];
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
    % whose altitudes or radome grid put every book value out of reach has
    % checked NOTHING, and that is the shape of green light this repo has
    % been burned by.
    if ~any(ev)
        error('mgs:zarchan:adjointWithAutopilot:noBenchmark', ...
            ['not one book value could be evaluated.  The published numbers ' ...
             'live at alt = 0, 30000 and 50000 ft with the nominal guidance ' ...
             'parameters; this run has alt = %s and moved %s.  Refusing to ' ...
             'hand back an unvalidated port silently -- pass ''benchmark'', ' ...
             'false if you meant to run off-book.'], ...
            mat2str(cfg.alt), offbookStr(offbook));
    end
    if ~R.pass
        bad = find(~[R.bench.ok] & ev);
        error('mgs:zarchan:adjointWithAutopilot:benchmark', ...
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
%   Straight off zarchan.txt:7823 (the first line of Listing 23.3), plus the
%   sweep grids the listing and the surrounding text use.  Change a value
%   here and you have changed what "the book" means for every benchmark row
%   in this file, which is the intended blast radius -- there is no second
%   copy anywhere.
N = struct();
N.alt    = [0 30000 50000];   % ALT.  The listing sets one; Table 23.5 runs three.
N.radome = -0.06:0.01:0.06;   % for R=-.06:.01:.06
N.miss   = 1;                 % MISS=1
N.tint   = 0;                 % TINT=0.
N.tf     = 5;                 % TF=5.
N.vm     = 3000;              % VM=3000.
N.vc     = 4000;              % VC=4000.
N.tau    = 0.3;               % TAU=.3
N.wcr    = 50;                % WCR=50.
N.zeta   = 0.7;               % ZETA=.7
N.xnp    = 3;                 % XNP=3.
N.t1     = 0.1;               % T1=.1
N.t2     = 0.15;              % T2=.15
N.wact   = 150;               % WACT=150.
N.zact   = 0.7;               % ZACT=.7
N.xnt    = 64.4;              % XNT=64.4   (2 g, in ft/s^2)
N.ra     = 30000;             % RA=30000.
N.phirn  = 0.000002;          % PHIRN=.000002    semiactive        (Table 23.4)
N.phigl  = 20;                % PHIGL=20.        glint
N.phifn  = 0.00000008;        % PHIFN=.00000008  range independent
N.wgt    = 1000;              % WGT=1000.
N.xncg   = 10;                % XNCG=10.
N.h      = 0.0001;            % H=.0001
end


% =============================================================================
function [cfg, offbook] = resolveConfig(opts, N)
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
        cfg.(nm) = v(:).';                   % row vector, always
        if ~isequal(cfg.(nm), N.(nm))
            offbook{end+1} = nm; %#ok<AGROW>
        end
    end
end
cfg.benchmark = opts.benchmark;
cfg.render    = opts.render;
cfg.plot      = opts.plot;
cfg.assert    = opts.assert;

% Grids for the auxiliary modes.  Fixed here rather than swept finely,
% because both modes DIVERGE as tgo -> 0 (the glint channel carries
% 1/(Vc*tgo)), so "the peak of the curve" is a property of where you stopped
% sampling, not of the system.  A fixed, stated grid is the only way a
% benchmark against those figures means anything, and it is why the fin-rate
% row below is scored as a BOUND rather than as a value.
cfg.tgo_fin = [0.25 0.5 2 4.9];
cfg.tgo_acc = [0.25 1 4];

% ---- scalarness: the thing the arguments block cannot express -------------
sc = {'miss','tint','tf','vm','vc','tau','wcr','zeta','xnp','t1','t2', ...
      'wact','zact','xnt','ra','phirn','phigl','phifn','wgt','xncg','h'};
for k = 1:numel(sc)
    if ~isscalar(cfg.(sc{k}))
        error('mgs:zarchan:adjointWithAutopilot:notScalar', ...
            ['''%s'' must be scalar; %d values were given.  Only alt and ' ...
             'radome are swept.'], sc{k}, numel(cfg.(sc{k})));
    end
end

% A caller CANNOT reach this: passing [] means "use the book value", so an
% empty grid can only come from BOOKNOMINAL() itself being edited to one.
% The guard is here for that edit, because an empty alt grid produces a
% zero-row result struct that renders as a clean, entirely empty report.
if isempty(cfg.alt) || isempty(cfg.radome)
    error('mgs:zarchan:adjointWithAutopilot:emptyGrid', ...
        ['the alt or radome grid is empty, which can only happen if ' ...
         'bookNominal() was edited.  An empty grid runs nothing and prints ' ...
         'a report that looks complete.']);
end
if ~any(cfg.miss == [1 2 3])
    error('mgs:zarchan:adjointWithAutopilot:mode', ...
        ['miss = %g is not a mode of Listing 23.3.  1 = miss distance, ' ...
         '2 = fin rate at tgo, 3 = commanded acceleration at tgo; the mode ' ...
         'is an INITIAL CONDITION on the adjoint, so there is no sensible ' ...
         'default to fall back to.'], cfg.miss);
end
if cfg.miss ~= 1 && cfg.tint <= 0
    % The book never quotes these modes at tgo = 0, and the reason is not
    % taste: X13D carries (Y1PZ/(Vc*tgo))^2 and at tgo = 1e-5 that divides by
    % 0.08 ft.  The answer comes back large, finite and made entirely of the
    % first integration step -- exactly the kind of plausible wrong number
    % this package exists to refuse.
    error('mgs:zarchan:adjointWithAutopilot:tintZero', ...
        ['miss = %g needs tint > 0.  Modes 2 and 3 read an output AT a ' ...
         'time-to-go and the glint channel is singular at tgo = 0 (it ' ...
         'carries 1/(Vc*tgo)); the value returned would be a discretization ' ...
         'artefact of the first step, not a fin rate.  The book quotes both ' ...
         'modes "at a particular time to go" (zarchan.txt:7782).'], cfg.miss);
end
if cfg.miss ~= 1 && cfg.tint < 0.05
    warning('mgs:zarchan:adjointWithAutopilot:tintSmall', ...
        ['tint = %g s is inside the region where the glint channel grows ' ...
         'without bound as tgo -> 0.  The number is real but it is dominated ' ...
         'by the last fraction of a second; compare it only against another ' ...
         'run at the SAME tgo.'], cfg.tint);
end
if cfg.tint >= cfg.tf
    error('mgs:zarchan:adjointWithAutopilot:tintHorizon', ...
        ['tint = %g s is not less than tf = %g s, so the adjoint integrates ' ...
         'over zero flight time and every output is exactly its initial ' ...
         'condition.  That is a null run, not a result.'], cfg.tint, cfg.tf);
end
if cfg.h <= 0 || cfg.h > cfg.tf/10
    error('mgs:zarchan:adjointWithAutopilot:step', ...
        'h = %g s is not a usable step for tf = %g s.', cfg.h, cfg.tf);
end
% Heun on the actuator's own poles.  lambda = -zact*wact +/- j*wact*sqrt(1-z^2)
% has |lambda| = wact, and RK2 goes unstable at |lambda|*h of order 2.  The
% failure is silent and total: the states grow without bound and the rms
% outputs come back as very large FINITE numbers.  Book values sit at
% wact*h = 0.015, a factor of 30 inside this bound.
if cfg.wact*cfg.h > 0.5
    error('mgs:zarchan:adjointWithAutopilot:actuatorStep', ...
        ['wact*h = %.3g.  The 2nd-order actuator is the fastest mode in the ' ...
         'loop and this two-pass (Heun) integrator goes unstable on it near ' ...
         'wact*h ~ 2, returning huge FINITE rms numbers rather than an ' ...
         'error.  The book runs wact = 150, h = 1e-4 (product 0.015).  ' ...
         'Reduce h or lower wact.'], cfg.wact*cfg.h);
end
if cfg.t1 <= 0 || cfg.t2 <= 0
    error('mgs:zarchan:adjointWithAutopilot:seeker', ...
        't1 and t2 are divided by in Y1PZ and X6D; neither may be <= 0.');
end
if cfg.vc <= 0 || cfg.ra <= 0
    error('mgs:zarchan:adjointWithAutopilot:range', ...
        ['vc and ra set the range scaling of two of the four noise channels ' ...
         '(vc*tgo is the range); neither may be <= 0.']);
end
end


% =============================================================================
function F = airframe(ALT, cfg, GRAV_FT)
%AIRFRAME  The aerodynamic + autopilot-gain preamble of Listing 23.3,
%   transcribed.  Identical text appears in Listings 23.1 and 23.2.
%
%   Geometry (Ch.22's airframe) is NOT exposed as options.  It is one
%   specific missile -- 20 ft long, 1 ft diameter, cruciform, tail control --
%   and the published numbers are its numbers.  Making it adjustable would
%   invite a run that is neither the book's airframe nor ours.
%
%   TWO THINGS IN HERE LOOK LIKE BUGS AND ARE NOT:
%
%   * A = 1000 ft/s at EVERY altitude.  The real speed of sound is nearer
%     995 ft/s at 30 kft and 968 ft/s at 50 kft, so Mach is off by a few per
%     cent up high.  Zarchan uses the flat 1000 anyway and the published
%     Table 23.2 is what that produces.  "Fixing" it moves Ma/Md/Za/Zd off
%     the book and every benchmark below goes red for a correct reason,
%     which is worse than a documented approximation.
%   * The trim solve uses XNCG (10 g) even in the adjoint, where nothing is
%     commanded to 10 g.  It is the LINEARIZATION POINT: CNA and CMA are
%     evaluated at the 10-g trim angle of attack because that is the
%     condition the linear model is meant to represent.  It is not a command.

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
    error('mgs:zarchan:adjointWithAutopilot:subsonic', ...
        ['vm = %g ft/s is Mach %.3g against the book''s fixed a = 1000 ft/s. ' ...
         'This linearized aero is SUPERSONIC-ONLY: every derivative is built ' ...
         'on B = sqrt(M^2-1), which is imaginary here.  Ch.22 has no ' ...
         'subsonic branch to fall back to.'], VM, XMACH);
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
    error('mgs:zarchan:adjointWithAutopilot:trim', ...
        ['the %g-g trim solve has no real root at alt = %g ft (P2 = %g, ' ...
         'discriminant = %g).  The quadratic in Ch.22 assumes the body-lift ' ...
         'term P2 is non-zero and the manoeuvre attainable; an imaginary ' ...
         'trim angle would propagate silently into CNA and CMA.'], ...
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
    error('mgs:zarchan:adjointWithAutopilot:unstable', ...
        ['Ma = %g >= 0 at alt = %g ft: this airframe is statically UNSTABLE ' ...
         'there and wAF = sqrt(-Ma) is imaginary.  The whole three-loop gain ' ...
         'algorithm of Ch.23 is derived for a stable airframe (wAF real); ' ...
         'it has no unstable branch.'], XMA, ALT);
end
if ZD == 0 || (XMA*ZD-ZA*XMD)/ZD < 0
    error('mgs:zarchan:adjointWithAutopilot:noZero', ...
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
    error('mgs:zarchan:adjointWithAutopilot:infeasibleGains', ...
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
    error('mgs:zarchan:adjointWithAutopilot:gains', ...
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
% flight-control system.  THIS is the derived lag.
F.tau_total = TAU + 2*ZETA/W;
F.step  = [];
F.sweep = [];
end


% =============================================================================
function O = adjointRun(F, cfg, R_slope, MISSMODE, TINT)
%ADJOINTRUN  Listing 23.3 itself.  Fifteen states, Zarchan's two-pass
%   predictor/corrector, transcribed.
%
%   STATE MAP (Fig. 23.15).  Names are the book's X1..X15.
%     X1,X2,X3   double integrator to the miss; X3 carries the MISS=1 impulse
%     X4,X5      TRUE and SEEKER-TRACKED line-of-sight integrators.  The
%                radome enters ONLY as the difference between them:
%                d(X4-X5)/dt = -R*Y1PZ exactly, so at R = 0 they hold the
%                same number bit for bit and the radome path is off.  That
%                identity is structural self-check 1 below.
%     X6         noise filter (T2); carries the MISS=3 initial condition
%     X7         autopilot integrator; the fin-rate output path
%     X8,X9      second-order actuator; X8 carries the MISS=2 impulse
%     X10,X11    second-order airframe
%     X12..X15   the four squared-and-integrated error channels
%
%   THREE THINGS THAT LOOK LIKE BUGS AND ARE NOT:
%     * TGO = TP + 1e-5.  Zarchan's dodge for 1/tgo at tgo = 0.  Remove it
%       and the first step divides by zero.
%     * TP IS advanced between the two passes and is NOT rolled back, so the
%       1/tgo term is evaluated at both times.  That detail is part of why
%       the port lands on the book.
%     * XMUDNT divides by the LAST TGO, not by tf.  They differ by 1e-5 and
%       say the same thing: PHI_nT = nT^2/tF.
%
%   The derivative block is written ONCE inside a two-pass loop rather than
%   split into a function, because a function call per pass is 100,000 calls
%   per run and this file makes about 75 runs.  Every parameter is unpacked
%   to a local scalar before the loop for the same reason: struct field
%   access inside a MATLAB loop this tight costs more than the arithmetic.

VC=cfg.vc; T1=cfg.t1; T2=cfg.t2; XNP=cfg.xnp; RA=cfg.ra;
WACT=cfg.wact; ZACT=cfg.zact; XNT=cfg.xnt; TF=cfg.tf; H=cfg.h;
PHIRN=cfg.phirn; PHIGL=cfg.phigl; PHIFN=cfg.phifn;
XKA=F.xka; WI=F.wi; XKR=F.xkr; XKDC=F.xkdc; XK1=F.xk1; XK3=F.xk3;
WAF=F.waf; WZ=F.wz; ZAF=F.zaf; TA=F.ta;
WZ2 = WZ*WZ; WAF2 = WAF*WAF; WACT2 = WACT*WACT;
R = R_slope;

X1=0.; X2=0.; X3=0.; X4=0.; X5=0.; X6=0.; X7=0.; X8=0.;
X9=0.; X10=0.; X11=0.; X12=0.; X13=0.; X14=0.; X15=0.;
switch MISSMODE
case 1, X3=1.;                    % miss distance
case 2, X8=1.;                    % fin rate at tgo = TINT
case 3, X6=XNP*VC/32.2;           % commanded acceleration at tgo = TINT
end

TP = 0. + .00001 + TINT;          % TP=T+.00001+TINT
TGO = TP + .00001;
X1D=0.;X2D=0.;X3D=0.;X4D=0.;X5D=0.;X6D=0.;X7D=0.;X8D=0.;
X9D=0.;X10D=0.;X11D=0.;X12D=0.;X13D=0.;X14D=0.;X15D=0.;
nstep = 0;

while TP <= (TF - 1e-5)
    X1OLD=X1; X2OLD=X2; X3OLD=X3; X4OLD=X4; X5OLD=X5; X6OLD=X6;
    X7OLD=X7; X8OLD=X8; X9OLD=X9; X10OLD=X10; X11OLD=X11;
    X12OLD=X12; X13OLD=X13; X14OLD=X14; X15OLD=X15;

    for pass = 1:2                       % the book's STEP/FLAG inner loop
        if pass == 2
            X1=X1+H*X1D; X2=X2+H*X2D; X3=X3+H*X3D; X4=X4+H*X4D;
            X5=X5+H*X5D; X6=X6+H*X6D; X7=X7+H*X7D; X8=X8+H*X8D;
            X9=X9+H*X9D; X10=X10+H*X10D; X11=X11+H*X11D; X12=X12+H*X12D;
            X13=X13+H*X13D; X14=X14+H*X14D; X15=X15+H*X15D;
            TP=TP+H;
        end
        TGO=TP+.00001;
        X1D=X2;
        X2D=X3;
        Y1PZ=(X6/T2+X5)/T1;
        X3D=Y1PZ/(VC*TGO);
        X4D=-Y1PZ;
        X5D=-Y1PZ+R*Y1PZ;               % the ONLY appearance of radome slope
        Y2PZ=-XKA*WI*X7;
        X6D=-X6/T2+XNP*VC*XKDC*Y2PZ/32.2;
        X7D=XKR*WACT2*X8;
        X8D=X9-2.*ZACT*WACT*X8;
        Y4PZ=XK1*(-32.2*X2-Y2PZ);
        X9D=-WACT2*X8+X10*WAF2-WAF2*Y4PZ/WZ2;
        % Y3PZ USES X7D, computed three lines above.  Order matters: move
        % this line up and the rate-gyro path silently uses the previous
        % step's fin rate, which reads as a small extra lag and is nearly
        % invisible in the answer.
        Y3PZ=XK3*(X7D+WI*X7+(X4-X5)/57.3);
        X10D=-2.*ZAF*WAF*(X10-Y4PZ/WZ2)+X11+TA*Y3PZ;
        X11D=-WAF2*(X10-Y4PZ/WZ2)+Y4PZ+Y3PZ;
        X12D=X1*X1;                      % target maneuver channel
        X13D=(Y1PZ/(VC*TGO))^2;          % glint             (linear at target)
        X14D=(Y1PZ*VC*TGO/RA)^2;         % semiactive        (angle x range/RA)
        X15D=Y1PZ^2;                     % range independent (bare angle)
    end

    X1=.5*(X1OLD+X1+H*X1D);     X2=.5*(X2OLD+X2+H*X2D);
    X3=.5*(X3OLD+X3+H*X3D);     X4=.5*(X4OLD+X4+H*X4D);
    X5=.5*(X5OLD+X5+H*X5D);     X6=.5*(X6OLD+X6+H*X6D);
    X7=.5*(X7OLD+X7+H*X7D);     X8=.5*(X8OLD+X8+H*X8D);
    X9=.5*(X9OLD+X9+H*X9D);     X10=.5*(X10OLD+X10+H*X10D);
    X11=.5*(X11OLD+X11+H*X11D); X12=.5*(X12OLD+X12+H*X12D);
    X13=.5*(X13OLD+X13+H*X13D); X14=.5*(X14OLD+X14+H*X14D);
    X15=.5*(X15OLD+X15+H*X15D);
    nstep = nstep + 1;
end

if nstep == 0
    % Zero steps returns every channel as exactly 0, which prints as a
    % perfect miss distance.  tint just short of tf gets here even though
    % tint < tf, so the horizon guard in resolveConfig does not cover it.
    error('mgs:zarchan:adjointWithAutopilot:noSteps', ...
        ['the adjoint took ZERO integration steps (tint = %g s, tf = %g s, ' ...
         'h = %g s), so every output would be exactly its initial condition ' ...
         '-- which reads as a miss distance of 0 ft.'], TINT, TF, H);
end

st = [X1 X2 X3 X4 X5 X6 X7 X8 X9 X10 X11 X12 X13 X14 X15];
if ~all(isfinite(st))
    error('mgs:zarchan:adjointWithAutopilot:diverged', ...
        ['the adjoint diverged at alt = %g ft, radome slope R = %g, mode %g ' ...
         '(states went non-finite after %d steps).  Two causes needing ' ...
         'different fixes: the INTEGRATOR is unstable (check wact*h = %.3g), ' ...
         'or the CLOSED LOOP is unstable at this radome slope -- Ch.23 shows ' ...
         'the loop going bad at large |R| high up, which is a result rather ' ...
         'than a numerical failure, but at the book''s parameters it does ' ...
         'not reach Inf inside 5 s.'], ...
        F.alt, R, MISSMODE, nstep, cfg.wact*cfg.h);
end
if X12 < 0 || X13 < 0 || X14 < 0 || X15 < 0
    error('mgs:zarchan:adjointWithAutopilot:negativeSquare', ...
        ['an integral of a squared quantity came out negative ' ...
         '(X12..X15 = %s).  That is arithmetically impossible for these ' ...
         'integrands and means the state vector has been corrupted; sqrt() ' ...
         'would go complex and the rms columns would print as real ' ...
         'magnitudes.'], mat2str([X12 X13 X14 X15], 4));
end

O = struct();
O.mode  = MISSMODE;
O.tint  = TINT;
O.R     = R;
O.tgo   = TGO;
O.nstep = nstep;
% Channel names below are TABLE 23.4's, not the code's -- see the file header
% for why the two disagree and which is which.
O.fn  = sqrt(X15*PHIFN);        % XMFN  : range independent noise
O.rn  = sqrt(X14*PHIRN);        % XMRN  : semiactive noise
O.gl  = sqrt(X13*PHIGL);        % XMGL  : glint noise
O.nt  = XNT*sqrt(X12/TGO);      % XMUDNT: random target maneuver
O.rms = sqrt(O.fn^2+O.rn^2+O.gl^2+O.nt^2);
O.x4_minus_x5 = X4 - X5;        % radome-path witness; exactly 0 iff R == 0
end


% =============================================================================
function S = radomeSweep(F, cfg)
%RADOMESWEEP  The listing's own outer loop, `for R=-.06:.01:.06`.
nR = numel(cfg.radome);
S = struct('radome', cfg.radome, 'mode', cfg.miss, 'tint', cfg.tint, ...
    'units', missUnits(cfg.miss), ...
    'fn', zeros(1,nR), 'rn', zeros(1,nR), 'gl', zeros(1,nR), ...
    'nt', zeros(1,nR), 'rms', zeros(1,nR), 'x4_minus_x5', zeros(1,nR));
for k = 1:nR
    O = adjointRun(F, cfg, cfg.radome(k), cfg.miss, cfg.tint);
    S.fn(k)=O.fn; S.rn(k)=O.rn; S.gl(k)=O.gl; S.nt(k)=O.nt; S.rms(k)=O.rms;
    S.x4_minus_x5(k) = O.x4_minus_x5;
end
end


function M = modeSweep(FC, cfg, MISSMODE, tgoGrid)
%MODESWEEP  Modes 2 and 3 over a FIXED time-to-go grid, at zero radome slope.
%   The book inverts a set of these to plot rms fin rate / rms commanded
%   acceleration against flight time (Figs. 23.19, 23.22); flight time is
%   tf - tgo.
nA = numel(FC); nT = numel(tgoGrid);
M = struct('mode', MISSMODE, 'tgo', tgoGrid, ...
    'flight_time', cfg.tf - tgoGrid, 'alt', [FC.alt], ...
    'units', missUnits(MISSMODE), ...
    'fn', zeros(nA,nT), 'rn', zeros(nA,nT), 'gl', zeros(nA,nT), ...
    'nt', zeros(nA,nT), 'rms', zeros(nA,nT));
for i = 1:nA
    for k = 1:nT
        O = adjointRun(FC(i), cfg, 0.0, MISSMODE, tgoGrid(k));
        M.fn(i,k)=O.fn; M.rn(i,k)=O.rn; M.gl(i,k)=O.gl;
        M.nt(i,k)=O.nt; M.rms(i,k)=O.rms;
    end
end
end


function u = missUnits(m)
switch m
case 1, u = 'ft (rms miss distance)';
case 2, u = 'deg/s (rms fin rate at tgo)';
case 3, u = 'g (rms commanded acceleration at tgo)';
otherwise, u = '(unknown mode)';
end
end


% =============================================================================
function P = fcsStep(F, cfg, GRAV_FT)
%FCSSTEP  Listing 23.1's closed-loop step response, transcribed
%   (zarchan.txt:7551-7556).  PULLED IN FROM A NEIGHBOURING LISTING -- see
%   the file header.  Five states, same two-pass integrator, TF = 1 s.
%
%   This is the only thing in Ch.23 that MEASURES the flight-control time
%   constant.  Everything else asserts one.
%
%   WHAT t63 IS MEASURED AGAINST, AND THE TRAP IN IT.
%
%   The closed-loop DC gain is EXACT ALGEBRA, not the last sample.  At steady
%   state ED = EDD = DELD = DELDD = XD = 0, so DEL = E, DELC = DEL,
%   THD = K3*E and nL = K1*E, and XD = 0 leaves
%
%       E_ss = KA*XNCG*KDC / (K3 + KA*K1),      nL_ss = K1*E_ss
%
%   Substituting K3 = 1845*K1/VM, KA = K3/(K1*KC) and KDC = 1 + 1845/(KA*VM)
%   collapses that to nL_ss/XNCG = KDC/(1+KC) = 1 EXACTLY -- which is what
%   KDC is for.  The actuator does not change it either: a second-order
%   actuator has unity DC gain too.  The self-check below tests this identity
%   at 1e-9 and it is the sharpest thing in the file, because it exercises
%   the whole chain K1 -> K3 -> KC -> KA -> KDC in one number.
%
%   *** THE TRAP: nl_g(end) IS NOT THE STEADY STATE. ***
%   Listing 23.1 stops at TF = 1 s, which is only about three time constants,
%   so the last sample is 96% of steady state at tau = 0.3 s and 86% at
%   tau = 0.5 s -- STILL RISING.  Reading "63% of the steady-state value" off
%   that last sample gives 0.318 s instead of 0.337 s and quietly shortens
%   the measured lag by 6%.  t63_s is therefore measured against the ANALYTIC
%   steady state (which equals the command), and lands on the book's 0.34 s
%   (zarchan.txt:7535).  The mis-reading is kept as t63_vs_last_sample_s
%   ONLY so that whoever reproduces 0.318 s can find out where it came from.

XK1=F.xk1; XK3=F.xk3; TA=F.ta; WAF=F.waf; ZAF=F.zaf; WZ2=F.wz*F.wz;
XKR=F.xkr; XKA=F.xka; WI=F.wi; XKDC=F.xkdc;
WACT=cfg.wact; ZACT=cfg.zact; XNCG=cfg.xncg; H=cfg.h;
TF = 1.0;                 % TF=1. in Listing 23.1: one second is ~3 tau

E=0.; ED=0.; DELD=0.; DEL=0.; X=0.; T=0.;
EDD=0.; DELDD=0.; XD=0.; XNL=0.;
n = ceil(TF/H)+2;
tA = zeros(1,n); nlA = zeros(1,n); rA = zeros(1,n); k = 0;

while T <= (TF-.00001)
    EOLD=E; EDOLD=ED; DELOLD=DEL; DELDOLD=DELD; XOLD=X;
    for pass = 1:2
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
    k=k+1; tA(k)=T; nlA(k)=XNL; rA(k)=DELD;
end

if k < 10
    error('mgs:zarchan:adjointWithAutopilot:stepSamples', ...
        'the step response produced %d samples; h = %g is too coarse.', k, H);
end
tA=tA(1:k); nlA=nlA(1:k); rA=rA(1:k);
if ~all(isfinite(nlA)) || ~all(isfinite(rA))
    error('mgs:zarchan:adjointWithAutopilot:stepDiverged', ...
        ['the Listing 23.1 step response diverged at alt = %g ft.  The ' ...
         'adjoint uses the same gains, so nothing computed from this flight ' ...
         'condition is trustworthy either.'], F.alt);
end

den = XK3 + XKA*XK1;
if den == 0 || ~isfinite(den)
    error('mgs:zarchan:adjointWithAutopilot:noSteadyState', ...
        ['K3 + KA*K1 = %g at alt = %g ft, so the closed loop has no finite ' ...
         'steady state and t63 has nothing to be measured against.  Every ' ...
         'lag number in this file is defined relative to that steady ' ...
         'state.'], den, F.alt);
end
ss  = XK1*(XKA*XNCG*XKDC/den);      % ANALYTIC steady state; == XNCG exactly
if ~isfinite(ss) || ss <= 0
    % A negative steady state would make 0.63*ss negative, the FIND below
    % would match the very first sample, and t63 would come back as one time
    % step -- a lag of 0.0001 s, printed with a straight face.
    error('mgs:zarchan:adjointWithAutopilot:badSteadyState', ...
        ['the closed-loop steady state came out %g g for a +%g g command at ' ...
         'alt = %g ft.  The gain algorithm''s unity-DC-gain identity has ' ...
         'broken down, so there is nothing for t63 to be 63%% of.'], ...
        ss, XNCG, F.alt);
end
i63 = find(nlA >= 0.63*ss,       1);   % the honest reading
j63 = find(nlA >= 0.63*nlA(end), 1);   % the 6%-short mis-reading, kept on purpose
[~, ipk] = max(abs(rA));
[wwy, iww] = min(nlA);                 % wrong-way (RHP zero) undershoot

P = struct();
P.t = tA; P.nl_g = nlA; P.finrate_dps = rA;
P.xncg_g       = XNCG;
P.ss_g         = ss;                   % analytic, NOT nl_g(end)
P.dc_gain      = ss/XNCG;              % must be 1 to ~1e-15; self-checked
P.nl_at_tf_g   = nlA(end);
P.settled_frac = nlA(end)/ss;          % ~0.96 at tau = 0.3 s, TF = 1 s
if P.settled_frac < 0.7
    % Below this the 63% point is still on the steep part but the run says
    % almost nothing about settling, and anyone reading nl_at_tf_g as a
    % steady state would be out by more than 30%.
    warning('mgs:zarchan:adjointWithAutopilot:notSettled', ...
        ['the step response reached only %.0f%% of steady state by TF = %g s ' ...
         '(alt = %g ft, tau = %g s).  t63 is still measured against the ' ...
         'ANALYTIC steady state and is fine, but nl_at_tf_g is not a ' ...
         'settled value.'], 100*P.settled_frac, TF, F.alt, F.tau_total);
end
if isempty(i63)
    % NaN, not the horizon.  Returning TF would read as "the lag is 1 s",
    % which is a claim, and a false one.
    P.t63_s = NaN;
else
    P.t63_s = tA(i63);
end
if isempty(j63), P.t63_vs_last_sample_s = NaN; else, P.t63_vs_last_sample_s = tA(j63); end
P.peak_finrate_dps = rA(ipk);
P.t_peak_finrate_s = tA(ipk);
P.wrongway_g       = wwy;      % negative: acceleration goes the wrong way first
P.t_wrongway_s     = tA(iww);
P.grav             = GRAV_FT;
end


% =============================================================================
function L = lagSummary(FC, cfg)
%LAGSUMMARY  The number this project came for, in one place.
L = struct();
L.tau_design_s   = cfg.tau;
L.alt_ft         = [FC.alt];
L.w_rad_s        = [FC.w];
L.tau_total_s    = [FC.tau_total];              % tau + 2*zeta/w  (p.538)
L.t63_measured_s = arrayfun(@(f) f.step.t63_s,    FC);
L.dc_gain        = arrayfun(@(f) f.step.dc_gain,  FC);   % analytic; must be 1
L.settled_frac   = arrayfun(@(f) f.step.settled_frac, FC);
% The mis-reading, carried so it can be recognised rather than rediscovered.
L.t63_vs_last_sample_s = arrayfun(@(f) f.step.t63_vs_last_sample_s, FC);
L.note = [ ...
 'tau_total is the p.538 approximation tau + 2*zeta/w; t63_measured is read ' ...
 'off the Listing 23.1 step response against the ANALYTIC steady state (see ' ...
 'FCSSTEP: the last sample is only ~96% of it, because TF = 1 s). ' ...
 'They are independent computations of the same lag. The transferable ' ...
 'result is that the gain algorithm HOLDS this lag nearly constant while ' ...
 'the airframe changes by 6x over the altitude sweep -- our own 8 s ' ...
 'gamma-tracking lag is held constant by nothing.'];
end


% =============================================================================
function T = tauComparison(cfg, GRAV_FT)
%TAUCOMPARISON  Figs. 23.18/23.21/23.24: tau = 0.3 s vs tau = 0.5 s at
%   50 kft, the altitude where all three of those figures are drawn.
%
%   Only the points the prose makes a claim about are run: each one is a full
%   5-s integration and the claims are about direction, not shape.
ALT = 50000;
c3 = cfg;  c3.tau = 0.3;
c5 = cfg;  c5.tau = 0.5;
F3 = airframe(ALT, c3, GRAV_FT);
F5 = airframe(ALT, c5, GRAV_FT);

T = struct();
T.alt    = ALT;
T.tau    = [0.3 0.5];
T.radome = [-0.06 -0.05];
T.rms_miss = zeros(2, numel(T.radome));
for k = 1:numel(T.radome)
    T.rms_miss(1,k) = adjointRunRms(F3, c3, T.radome(k), 1, 0);
    T.rms_miss(2,k) = adjointRunRms(F5, c5, T.radome(k), 1, 0);
end
T.tgo_fin     = 2.0;
T.rms_finrate = [adjointRunRms(F3, c3, 0, 2, T.tgo_fin), ...
                 adjointRunRms(F5, c5, 0, 2, T.tgo_fin)];
T.tgo_acc     = cfg.tgo_acc;
T.rms_accel   = zeros(2, numel(T.tgo_acc));
for k = 1:numel(T.tgo_acc)
    T.rms_accel(1,k) = adjointRunRms(F3, c3, 0, 3, T.tgo_acc(k));
    T.rms_accel(2,k) = adjointRunRms(F5, c5, 0, 3, T.tgo_acc(k));
end
S3 = fcsStep(F3, c3, GRAV_FT);  S5 = fcsStep(F5, c5, GRAV_FT);
T.t63_s      = [S3.t63_s S5.t63_s];
T.tau_total  = [F3.tau_total F5.tau_total];
end


function v = adjointRunRms(F, cfg, R, mode, tint)
O = adjointRun(F, cfg, R, mode, tint);
v = O.rms;
end


% =============================================================================
function C = structuralCheck(cfg, GRAV_FT)
%STRUCTURALCHECK  Invariants that must hold or every number above is garbage.
%
%   These are NOT book values.  They test the WIRING, and unlike the
%   benchmarks they still bite when the caller runs off-book.  This function
%   makes its OWN runs at its OWN flight conditions so that it checks the
%   same things no matter what the caller asked for.
%
%   1. RADOME PATH GATED BY R.  X4 and X5 integrate -Y1PZ and -(1-R)*Y1PZ, so
%      d(X4-X5)/dt = -R*Y1PZ.  At R = 0 the two states hold the same number
%      BIT FOR BIT (IEEE: -y + 0*y == -y), so X4 - X5 must be EXACTLY 0.  If
%      a transcription puts R anywhere else -- on X4, on Y3PZ, on both -- this
%      is non-zero at R = 0 and the check fails.  Exact test, no tolerance.
%   2. RADOME PATH PRESENT.  The mirror image: the same difference must be
%      non-zero at R = -0.06 and the miss must MOVE.  Checked at 50 kft,
%      where Ch.23 says the sensitivity is large; at sea level deleting the
%      whole radome path changes the miss by 2%, so a sea-level-only version
%      of this check would pass on a broken port.
%   3. THE LAG IS THE LAG.  The measured 63% rise time of the Listing 23.1
%      step response must agree with the closed form tau + 2*zeta/w.  Two
%      independent computations -- one an integration, one algebra -- of the
%      number this file exists to produce.
%   4. UNITY DC GAIN, AS EXACT ALGEBRA.  Substituting the gain algorithm into
%      the closed-loop steady state gives nL_ss/XNCG = KDC/(1+KC), and the
%      algorithm's own KDC = 1 + 1845/(KA*VM) is identically 1 + KC (because
%      KA = 1845/(VM*KC)).  So this ratio is 1 to floating-point round-off at
%      EVERY flight condition and every tau, and one number exercises the
%      whole chain K1 -> K3 -> KC -> KA -> KDC.  Tolerance 1e-9, not a few
%      per cent: it is an identity, not a design goal.  Note this does NOT
%      touch KR or wI -- check 3 is what exercises those.
%      (The simulated last sample is ~96% of steady state because Listing
%      23.1 stops at TF = 1 s.  Do not confuse the two; see FCSSTEP.)

C = struct(); C.skipped = false;

F0  = airframe(0,     cfg, GRAV_FT);
F50 = airframe(50000, cfg, GRAV_FT);

% -- 1. exactly zero at R = 0
O0 = adjointRun(F0, cfg, 0.0, 1, 0);
C.radome_zero_resid = abs(O0.x4_minus_x5);
C.radome_zero_tol   = 0;
C.radome_zero_ok    = C.radome_zero_resid <= C.radome_zero_tol;

% -- 2. alive at R ~= 0, at the altitude where it matters
A = adjointRun(F50, cfg, 0.0,   1, 0);
B = adjointRun(F50, cfg, -0.06, 1, 0);
C.radome_alive_ratio = B.rms / A.rms;
C.radome_alive_tol   = 2.0;
C.radome_alive_ok    = C.radome_alive_ratio >= C.radome_alive_tol && ...
                       abs(B.x4_minus_x5) > 0;

% -- 3. measured lag vs the closed-form lag
S0  = fcsStep(F0,  cfg, GRAV_FT);
S50 = fcsStep(F50, cfg, GRAV_FT);
C.lag_meas = [S0.t63_s S50.t63_s];
C.lag_form = [F0.tau_total F50.tau_total];
C.lag_rel  = max(abs(C.lag_meas - C.lag_form) ./ C.lag_form);
C.lag_tol  = 0.05;
C.lag_ok   = all(isfinite(C.lag_meas)) && C.lag_rel <= C.lag_tol;

% -- 4. unity DC gain, as exact algebra
C.dc_gain      = [S0.dc_gain S50.dc_gain];
C.dc_err       = max(abs(C.dc_gain - 1));
C.dc_tol       = 1e-9;
C.dc_ok        = C.dc_err <= C.dc_tol;
C.settled_frac = [S0.settled_frac S50.settled_frac];   % reported, not gated

C.ok = C.radome_zero_ok && C.radome_alive_ok && C.lag_ok && C.dc_ok;
if ~C.ok
    error('mgs:zarchan:adjointWithAutopilot:selfcheck', ...
        ['a STRUCTURAL invariant failed, so every number this function ' ...
         'computed is wrong:\n' ...
         '  radome gated by R (X4-X5 == 0 at R=0) : %.3e  tol %g       %s\n' ...
         '  radome path alive (miss ratio, 50 kft): %.3f  tol >= %.1f  %s\n' ...
         '  measured lag vs tau + 2*zeta/w        : %.3e  tol %.0e     %s\n' ...
         '  unity DC gain (algebraic identity)    : %.3e  tol %.0e     %s\n' ...
         'These test the wiring, not the book.  A failure here means the ' ...
         'transcription of Listing 23.3 has been damaged.'], ...
        C.radome_zero_resid, C.radome_zero_tol, tf2s(C.radome_zero_ok), ...
        C.radome_alive_ratio, C.radome_alive_tol, tf2s(C.radome_alive_ok), ...
        C.lag_rel, C.lag_tol, tf2s(C.lag_ok), ...
        C.dc_err, C.dc_tol, tf2s(C.dc_ok));
end
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
%   exist.  The extraction turned every figure into an image link, so NO
%   number that was only ever plotted survives.  Three "benchmarks" handed to
%   the previous wave were figure-derived and all three were wrong, one of
%   them contaminated from a different chapter.  What the book says in PROSE
%   about a figure ("monotonically decreasing", "independent of flight
%   condition") IS greppable and IS used -- as shape rows, which are often
%   sharper than a level read off a plot.
%
%   Everything below was grepped from
%   matlab/zarchan/_source/zarchan.txt at the line quoted.

V = struct();

% -- Table 23.2, "LINEARIZED AERODYNAMIC PARAMETERS FOR DIFFERENT FLIGHT
%    CONDITIONS", p.551, zarchan.txt:7736-7742.  The OCR renders a minus sign
%    as '2' and an exponent as a trailing digit ("2642 s22" is -642 s^-2).
%       alt      Ma      Md      Za       Zd
V.aero = [ 0    -642    -555   -2.94   -0.65
        30000   -240    -204   -1.17   -0.239
        50000    -99.1   -81.7 -0.533  -0.0957 ];

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

% -- p.537, zarchan.txt:7528: "The three autopilot gains for these
%    requirements at this flight condition turn out to be KA = 1.15 deg/g-s,
%    wI = 12.9 rad/s, and KR = 0.0928 s."  Sea level only.
V.ka = 1.15; V.wi = 12.9; V.kr = 0.0928;

% -- p.538, zarchan.txt:7535: "Because the damping is 0.7 and the natural
%    frequency is approximately 35 rad/s, the overall or total time constant
%    is 0.34 s ... the response reaches 63% of the steady-state value in
%    0.34 s."  Sea level.
V.w_natural = 35;  V.tau_total = 0.34;  V.t63 = 0.34;

% -- Table 23.5, "MISS DISTANCE PERFORMANCE APPEARS TO BE INDEPENDENT OF
%    ALTITUDE", p.556, zarchan.txt:7812-7816.  Radome slope zero.
%    COLUMN ORDER IS THE BOOK'S ROW ORDER, and the first two are the pair
%    that is easy to swap -- see the file header.
%       alt    RIN    semiactive  glint  maneuver  RMS
V.miss = [ 0    2.01   2.27       9.03   12.4      15.6
        30000   2.06   2.33       9.21   12.7      16.0
        50000   2.17   2.44       9.61   13.2      16.6 ];

% -- p.562, zarchan.txt:7850: "If the maximum rms miss that can be tolerated
%    at 50-kft altitude is 25 ft, then the radome slope swing would have to
%    be confined to range from R = -0.03 to R = 0.03 for a total radome swing
%    of 0.06."
V.miss_limit_ft = 25;  V.radome_swing = 0.06;

% -- p.563, zarchan.txt:7861: "the actuator must be able to support rms fin
%    rates in excess of 500 deg/s" at 50 kft, 5-s flight, zero radome slope.
V.finrate_bound_50k = 500;

% -- p.550, zarchan.txt:7710: "At sea level only a few deg/s of fin rate are
%    required whereas at 50 kft altitude nearly 600 deg/s are required".
%    This is the Listing 23.1 STEP RESPONSE (Fig. 23.13), not the adjoint.
V.step_finrate_50k = 600;

% -- p.565, zarchan.txt:7887: "For this example 25 g of rms missile
%    acceleration capability would be required".  REPORTED, NOT SCORED --
%    see the row that carries it for why.
V.accel_sizing_g = 25;
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
%     'approx' 5%   the book says "approximately"
%     'graph' 15%   the author read it off a plot ("nearly 600 deg/s")
%   plus 'shape' and 'bound', which are inequalities and orderings taken from
%   prose, and carry no tolerance at all.
%
%   Do NOT widen a band to turn a row green.

V  = bookValues();
B  = emptyBench();

GATE_AERO  = {'vm','wgt','xncg'};
GATE_GAIN  = [GATE_AERO, {'tau','wcr','zeta'}];
GATE_LOOP  = [GATE_GAIN, {'tf','vc','xnp','t1','t2','wact','zact','xnt', ...
                          'ra','phirn','phigl','phifn'}];
GATE_SWEEP = [GATE_LOOP, {'miss','tint'}];   % the radome sweep runs in cfg.miss

wA = gateWhy(offbook, GATE_AERO);
wG = gateWhy(offbook, GATE_GAIN);
wL = gateWhy(offbook, GATE_LOOP);
wS = gateWhy(offbook, GATE_SWEEP);

% Every "grows with altitude" row below is a diff() over R.fc in the order
% the caller listed the altitudes.  Hand it a descending or shuffled alt
% grid and a CORRECT port fails those rows, so they are gated on the grid
% being ascending rather than silently reordered behind the caller's back.
ascAlt   = numel(cfg.alt) >= 2 && issorted(cfg.alt);
whyAsc   = 'needs 2 or more altitudes given in ASCENDING order';

altBook = V.aero(:,1).';
nm = {'Ma','Md','Za','Zd'};
un = {'1/s^2','1/s^2','1/s','1/s'};
fl = {'xma','xmd','za','zd'};

% ---- A. airframe, Table 23.2 --------------------------------------------
for a = 1:numel(altBook)
    i = findAlt(cfg, altBook(a));
    for c = 1:4
        what = sprintf('aero %-2s @ %2g kft', nm{c}, altBook(a)/1000);
        src  = 'Table 23.2, p.551 (zarchan.txt:7736)';
        if ~isempty(wA)
            B = addSkip(B, what, sprintf('%g', V.aero(a,1+c)), wA, src, un{c});
        elseif isempty(i)
            B = addSkip(B, what, sprintf('%g', V.aero(a,1+c)), ...
                sprintf('alt = %g ft is not in the alt grid', altBook(a)), src, un{c});
        else
            B = addVal(B, what, V.aero(a,1+c), R.fc(i).(fl{c}), 'table', src, ...
                un{c}, '');
        end
    end
    % right-half-plane airframe zero, and the aerodynamic gain
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

% ---- B. the autopilot gains and the derived lag, sea level ---------------
i0 = findAlt(cfg, 0);
src537 = 'p.537 text (zarchan.txt:7528)';
src538 = 'p.538 text (zarchan.txt:7535)';
gainRows = { 'KA  autopilot gain @ 0 kft', V.ka,        'xka',  'deg/g-s', src537, 'text'
             'wI  autopilot gain @ 0 kft', V.wi,        'wi',   'rad/s',   src537, 'text'
             'KR  autopilot gain @ 0 kft', V.kr,        'xkr',  's',       src537, 'text'
             'W   FCS nat freq   @ 0 kft', V.w_natural, 'w',    'rad/s',   src538, 'approx'
             'tau_TOT = tau+2z/w @ 0 kft', V.tau_total, 'tau_total','s',   src538, 'text' };
for r = 1:size(gainRows,1)
    what = gainRows{r,1};
    if ~isempty(wG)
        B = addSkip(B, what, sprintf('%g', gainRows{r,2}), wG, gainRows{r,5}, gainRows{r,4});
    elseif isempty(i0)
        B = addSkip(B, what, sprintf('%g', gainRows{r,2}), ...
            'sea level is not in the alt grid', gainRows{r,5}, gainRows{r,4});
    else
        B = addVal(B, what, gainRows{r,2}, R.fc(i0).(gainRows{r,3}), ...
            gainRows{r,6}, gainRows{r,5}, gainRows{r,4}, '');
    end
end

% t63 from the Listing 23.1 step response -- the MEASURED lag
what = 't63 step response  @ 0 kft';
if ~isempty(wG)
    B = addSkip(B, what, sprintf('%g', V.t63), wG, src538, 's');
elseif isempty(i0)
    B = addSkip(B, what, sprintf('%g', V.t63), 'sea level not in grid', src538, 's');
else
    B = addVal(B, what, V.t63, R.fc(i0).step.t63_s, 'text', src538, 's', ...
        sprintf(['63%% of the ANALYTIC steady state. Read against the last ' ...
                 'sample instead (only %.0f%% settled at TF = 1 s) it is ' ...
                 '%.4f s -- 6%% short, and does not land on the book'], ...
                 100*R.fc(i0).step.settled_frac, ...
                 R.fc(i0).step.t63_vs_last_sample_s));
end

% ---- C. Table 23.5, the miss-distance error budget at R = 0 -------------
chan  = {'RIN','semiact','glint','maneuv','RMS'};
cfld  = {'fn','rn','gl','nt','rms'};
src55 = 'Table 23.5, p.556 (zarchan.txt:7812)';
kR0   = findRadome(cfg, 0);
for a = 1:numel(altBook)
    i = findAlt(cfg, altBook(a));
    for c = 1:5
        what = sprintf('miss %-7s @ %2g kft', chan{c}, altBook(a)/1000);
        bk   = V.miss(a,1+c);
        if ~isempty(wS)
            B = addSkip(B, what, sprintf('%g ft', bk), wS, src55, 'ft');
        elseif isempty(i)
            B = addSkip(B, what, sprintf('%g ft', bk), ...
                sprintf('alt = %g ft is not in the alt grid', altBook(a)), src55, 'ft');
        elseif isempty(kR0)
            B = addSkip(B, what, sprintf('%g ft', bk), ...
                'radome slope 0 is not on the radome grid', src55, 'ft');
        else
            B = addVal(B, what, bk, R.fc(i).sweep.(cfld{c})(kR0), 'table', ...
                src55, 'ft', '');
        end
    end
end

% ---- D. shape rows from prose about Table 23.5 --------------------------
if isempty(wS) && ~isempty(kR0) && numel(cfg.alt) >= 2
    rmsAll = arrayfun(@(f) f.sweep.rms(kR0), R.fc);
    ratio  = max(rmsAll)/min(rmsAll);
    B = addShape(B, 'RMS miss ~ independent of altitude', 'ratio <= 1.10', ...
        sprintf('%.3f (%s ft)', ratio, num2str(rmsAll, '%.2f ')), ratio <= 1.10, ...
        'Table 23.5 caption + p.554 (zarchan.txt:7783)', ...
        'the gain algorithm holds the FCS response fixed, so the miss should barely move');
else
    B = addSkip(B, 'RMS miss ~ independent of altitude', 'ratio <= 1.10', ...
        firstNonEmpty(wS, 'needs R = 0 on the grid and 2+ altitudes'), ...
        'Table 23.5 caption + p.554 (zarchan.txt:7783)', '-');
end

if isempty(wS) && ~isempty(kR0)
    okOrder = true; det = '';
    for i = 1:numel(R.fc)
        s = R.fc(i).sweep;
        okOrder = okOrder && s.nt(kR0) > s.gl(kR0) && ...
                             s.gl(kR0) > s.rn(kR0) && s.rn(kR0) > s.fn(kR0);
        det = [det sprintf('%gk:%.2f>%.2f>%.2f>%.2f  ', R.fc(i).alt/1000, ...
               s.nt(kR0), s.gl(kR0), s.rn(kR0), s.fn(kR0))]; %#ok<AGROW>
    end
    B = addShape(B, 'budget order maneuv>glint>semi>RIN', ...
        'maneuver and glint dominate', det, okOrder, ...
        'p.554 text (zarchan.txt:7783)', ...
        'also the check that the two ~2 ft noise columns are not swapped');
else
    B = addSkip(B, 'budget order maneuv>glint>semi>RIN', 'maneuver and glint dominate', ...
        firstNonEmpty(wS, 'needs R = 0 on the grid'), 'p.554 text (zarchan.txt:7783)', '-');
end

% ---- E. radome-slope sensitivity ---------------------------------------
src16 = 'Fig. 23.16 prose, p.557 (zarchan.txt:7821)';
i0s  = findAlt(cfg, 0);
i50  = findAlt(cfg, 50000);
haveSweep = isempty(wS) && ~isempty(kR0) && numel(cfg.radome) >= 3;

if haveSweep && ~isempty(i0s)
    s = R.fc(i0s).sweep;
    rat = max(s.rms)/s.rms(kR0);
    B = addShape(B, 'sea level: flat vs radome slope', 'max/at-zero <= 1.5', ...
        sprintf('%.3f (%.2f .. %.2f ft)', rat, min(s.rms), max(s.rms)), rat <= 1.5, ...
        src16, 'the book: "very little miss distance variation" down low');
else
    B = addSkip(B, 'sea level: flat vs radome slope', 'max/at-zero <= 1.5', ...
        firstNonEmpty(wS, 'needs sea level and a radome grid'), src16, '-');
end

if haveSweep && ~isempty(i50)
    s = R.fc(i50).sweep;
    lo = s.rms(1)/s.rms(kR0); hi = s.rms(end)/s.rms(kR0);
    B = addShape(B, '50 kft: blows up at BOTH extremes', 'both ends >= 2x', ...
        sprintf('R=%+.2f -> %.1fx,  R=%+.2f -> %.1fx', cfg.radome(1), lo, ...
                cfg.radome(end), hi), lo >= 2 && hi >= 2, src16, ...
        'the book: "very large due to either large negative or positive slopes"');

    % the 25-ft radome swing.  A NUMBER, and it is on the 'graph' band
    % because the book read the two crossings off Fig. 23.17 with 0.01
    % gridlines.
    [rneg, rpos, swing] = radomeSwing(cfg.radome, s.rms, V.miss_limit_ft);
    src17 = 'Fig. 23.17 prose, p.562 (zarchan.txt:7850)';
    if isnan(swing)
        B = addSkip(B, '50 kft radome swing for 25 ft rms', ...
            sprintf('%g', V.radome_swing), ...
            'the 25 ft level is not crossed on both sides of the radome grid', ...
            src17, '-');
    else
        B = addVal(B, '50 kft radome swing for 25 ft rms', V.radome_swing, swing, ...
            'graph', src17, '-', sprintf(['crossings at R = %+.4f and %+.4f ' ...
            '(book reads -0.03 and +0.03 off the figure)'], rneg, rpos));
    end

    B = addShape(B, '50 kft: -R hurts all, +R is maneuver', ...
        'noise dominates at -R, maneuver at +R', ...
        sprintf(['R=%+.2f: RIN %.0f semi %.0f glint %.0f vs maneuv %.0f | ' ...
                 'R=%+.2f: maneuv %.0f vs glint %.0f'], ...
                cfg.radome(1), s.fn(1), s.rn(1), s.gl(1), s.nt(1), ...
                cfg.radome(end), s.nt(end), s.gl(end)), ...
        s.fn(1) > s.nt(1) && s.rn(1) > s.nt(1) && s.gl(1) > s.nt(1) && ...
        s.nt(end) > s.gl(end) && s.nt(end) > s.rn(end) && s.nt(end) > s.fn(end), ...
        'Fig. 23.17 prose, p.562 (zarchan.txt:7850)', '');
else
    B = addSkip(B, '50 kft: blows up at BOTH extremes', 'both ends >= 2x', ...
        firstNonEmpty(wS, 'needs 50 kft and a radome grid'), src16, '-');
    B = addSkip(B, '50 kft radome swing for 25 ft rms', sprintf('%g', V.radome_swing), ...
        firstNonEmpty(wS, 'needs 50 kft and a radome grid'), ...
        'Fig. 23.17 prose, p.562 (zarchan.txt:7850)', '-');
    B = addSkip(B, '50 kft: -R hurts all, +R is maneuver', 'noise at -R, maneuver at +R', ...
        firstNonEmpty(wS, 'needs 50 kft and a radome grid'), ...
        'Fig. 23.17 prose, p.562 (zarchan.txt:7850)', '-');
end

% ---- F. altitude mechanism rows -----------------------------------------
if isempty(wA) && ascAlt
    wzv = [R.fc.wz];  k1v = abs([R.fc.xk1]);
    B = addShape(B, 'wZ falls as altitude rises', 'strictly decreasing', ...
        num2str(wzv, '%.2f '), all(diff(wzv) < 0), ...
        'p.550 text (zarchan.txt:7709)', ...
        'the mechanism behind the wrong-way effect growing with altitude');
    B = addShape(B, '|K1| falls as altitude rises', 'strictly decreasing', ...
        num2str(k1v, '%.3f '), all(diff(k1v) < 0), ...
        'p.552 text (zarchan.txt:7748)', ...
        'the mechanism behind fin rate growing with altitude');
else
    B = addSkip(B, 'wZ falls as altitude rises', 'strictly decreasing', ...
        firstNonEmpty(wA, whyAsc), 'p.550 text (zarchan.txt:7709)', '-');
    B = addSkip(B, '|K1| falls as altitude rises', 'strictly decreasing', ...
        firstNonEmpty(wA, whyAsc), 'p.552 text (zarchan.txt:7748)', '-');
end

% ---- G. Listing 23.1 step response: the lag, and fin-rate sizing ---------
% This one is a SPREAD, so it does not care what order the altitudes came in.
if isempty(wG) && numel(cfg.alt) >= 2
    t63 = arrayfun(@(f) f.step.t63_s, R.fc);
    sp  = max(t63)/min(t63);
    B = addShape(B, 'FCS response ~ independent of alt', 'spread <= 3%', ...
        sprintf('%.2f%% (%s s)', 100*(sp-1), num2str(t63, '%.4f ')), sp <= 1.03, ...
        'Fig. 23.12 caption, p.550 (zarchan.txt:7711)', ...
        'this constancy IS the gain algorithm; it is what our vehicle lacks');
else
    B = addSkip(B, 'FCS response ~ independent of alt', 'spread <= 3%', ...
        firstNonEmpty(wG, 'needs 2 or more altitudes'), ...
        'Fig. 23.12 caption, p.550 (zarchan.txt:7711)', '-');
end
% These two are ORDERINGS and do.
if isempty(wG) && ascAlt
    ww = arrayfun(@(f) abs(f.step.wrongway_g), R.fc);
    B = addShape(B, 'wrong-way dip grows with altitude', 'strictly increasing', ...
        num2str(ww, '%.3f '), all(diff(ww) > 0), ...
        'p.550 text (zarchan.txt:7709)', 'the symptom of wZ falling, same sentence');
    pk = arrayfun(@(f) abs(f.step.peak_finrate_dps), R.fc);
    B = addShape(B, 'step fin rate grows with altitude', 'strictly increasing', ...
        num2str(pk, '%.1f '), all(diff(pk) > 0), ...
        'Fig. 23.13 prose, p.550 (zarchan.txt:7710)', '');
else
    B = addSkip(B, 'wrong-way dip grows with altitude', 'strictly increasing', ...
        firstNonEmpty(wG, whyAsc), 'p.550 text (zarchan.txt:7709)', '-');
    B = addSkip(B, 'step fin rate grows with altitude', 'strictly increasing', ...
        firstNonEmpty(wG, whyAsc), 'Fig. 23.13 prose, p.550 (zarchan.txt:7710)', '-');
end

src13 = 'Fig. 23.13 prose, p.550 (zarchan.txt:7710)';
if isempty(wG) && ~isempty(i50)
    B = addVal(B, 'step peak fin rate @ 50 kft', V.step_finrate_50k, ...
        abs(R.fc(i50).step.peak_finrate_dps), 'graph', src13, 'deg/s', ...
        '10-g step, Listing 23.1; the book says "nearly 600 deg/s"');
    if ~isempty(i0s)
        % The port DISAGREES with the adjective in the same sentence, and
        % that is reported rather than buried.  17 deg/s is not "a few".
        B = addInfo(B, 'step peak fin rate @ 0 kft', 'a few deg/s', ...
            sprintf('%.2f deg/s', abs(R.fc(i0s).step.peak_finrate_dps)), src13, ...
            ['the book''s only sea-level statement is the adjective "a few", ' ...
             'and 17 deg/s is not that. The RATIO to 50 kft (32x) is what the ' ...
             'sentence is really about and it is scored above. Not scored: ' ...
             'there is no number to score against.']);
    end
else
    B = addSkip(B, 'step peak fin rate @ 50 kft', sprintf('%g deg/s', V.step_finrate_50k), ...
        firstNonEmpty(wG, '50 kft is not in the alt grid'), src13, 'deg/s');
end

% ---- H. adjoint fin rate, mode 2 ----------------------------------------
src19 = 'Fig. 23.19 prose, p.563 (zarchan.txt:7861)';
if ~isempty(wL) || isempty(R.finrate)
    B = addSkip(B, 'rms fin rate @ 50 kft exceeds 500', '> 500 deg/s', ...
        firstNonEmpty(wL, 'mode-2 runs were not made'), src19, 'deg/s');
elseif isempty(i50)
    B = addSkip(B, 'rms fin rate @ 50 kft exceeds 500', '> 500 deg/s', ...
        '50 kft is not in the alt grid', src19, 'deg/s');
else
    fr  = R.finrate.rms(i50, :);
    pk  = max(fr);
    B = addBound(B, 'rms fin rate @ 50 kft exceeds 500', ...
        sprintf('> %g deg/s', V.finrate_bound_50k), sprintf('%.1f deg/s', pk), ...
        pk > V.finrate_bound_50k, src19, 'deg/s', ...
        sprintf(['on the fixed tgo grid %s s. A BOUND, not a value: the ' ...
                 'glint channel carries 1/(Vc*tgo) so the curve has no ' ...
                 'maximum as tgo -> 0 and any "peak" is a property of where ' ...
                 'sampling stopped'], mat2str(R.finrate.tgo)));

    if ascAlt
        okA = all(all(diff(R.finrate.rms, 1, 1) > 0));
        B = addShape(B, 'rms fin rate grows with altitude', 'increasing at every tgo', ...
            sprintf('%.0f / %.0f deg/s at tgo = %g s', R.finrate.rms(1,1), ...
                    R.finrate.rms(end,1), R.finrate.tgo(1)), okA, src19, '');
    else
        B = addSkip(B, 'rms fin rate grows with altitude', 'increasing at every tgo', ...
            whyAsc, src19, 'deg/s');
    end

    % Fig. 23.20: semiactive early, glint late.  "Early" is LARGE tgo.
    [~, kEarly] = max(R.finrate.tgo);
    [~, kLate ] = min(R.finrate.tgo);
    early = [R.finrate.fn(i50,kEarly) R.finrate.rn(i50,kEarly) R.finrate.gl(i50,kEarly)];
    late  = [R.finrate.fn(i50,kLate ) R.finrate.rn(i50,kLate ) R.finrate.gl(i50,kLate )];
    B = addShape(B, 'fin rate: semi early, glint late', ...
        'semiactive dominates early, glint late', ...
        sprintf('tgo %.2g: RIN %.0f semi %.0f glint %.0f | tgo %.2g: RIN %.0f semi %.0f glint %.0f', ...
                R.finrate.tgo(kEarly), early, R.finrate.tgo(kLate), late), ...
        early(2) > early(1) && early(2) > early(3) && ...
        late(3)  > late(1)  && late(3)  > late(2), ...
        'Fig. 23.20 prose, p.564 (zarchan.txt:7874)', ...
        'beginning of flight is LARGE tgo -- the adjoint runs backwards');

    if ~isempty(R.tau05)
        rat = R.tau05.rms_finrate(2)/R.tau05.rms_finrate(1);
        B = addShape(B, 'tau 0.3->0.5 cuts fin rate', 'ratio <= 0.90', ...
            sprintf('%.3f (%.0f -> %.0f deg/s at tgo %g)', rat, ...
                    R.tau05.rms_finrate(1), R.tau05.rms_finrate(2), R.tau05.tgo_fin), ...
            rat <= 0.90, 'Fig. 23.21 prose, p.564 (zarchan.txt:7875)', ...
            '"significantly reduced" -- direction and materiality, at 50 kft');
    end
end

% ---- I. adjoint commanded acceleration, mode 3 --------------------------
src22 = 'Fig. 23.22 prose, p.565 (zarchan.txt:7887)';
if ~isempty(wL) || isempty(R.accel)
    B = addSkip(B, 'rms cmd accel ~ independent of alt', 'spread <= 2%', ...
        firstNonEmpty(wL, 'mode-3 runs were not made'), src22, 'g');
elseif numel(cfg.alt) < 2
    B = addSkip(B, 'rms cmd accel ~ independent of alt', 'spread <= 2%', ...
        'needs 2+ altitudes', src22, 'g');
else
    sp = max(max(R.accel.rms, [], 1) ./ min(R.accel.rms, [], 1));
    B = addShape(B, 'rms cmd accel ~ independent of alt', 'spread <= 2%', ...
        sprintf('%.2f%% worst over tgo = %s s', 100*(sp-1), mat2str(R.accel.tgo)), ...
        sp <= 1.02, src22, ...
        'the same claim as Table 23.5, on a different output of the same loop');

    if ~isempty(R.tau05)
        rat = min(R.tau05.rms_accel(2,:) ./ R.tau05.rms_accel(1,:));
        B = addShape(B, 'tau 0.3->0.5 does NOT cut cmd accel', 'min ratio >= 0.98', ...
            sprintf('%.3f worst over tgo = %s s', rat, mat2str(R.tau05.tgo_acc)), ...
            rat >= 0.98, 'Fig. 23.24 prose, p.566 (zarchan.txt:7899)', ...
            'the counterpart to the fin-rate row: tau buys fin rate, not g');
    end

    if ~isempty(i50)
        [~, kLate] = min(R.accel.tgo);
        B = addInfo(B, 'cmd accel sizing (25 g)', '25 g', ...
            sprintf('%.1f g at tgo = %g s', R.accel.rms(i50,kLate), R.accel.tgo(kLate)), ...
            src22, ...
            ['NOT SCORED. The book''s "25 g would be required" is a sizing ' ...
             'statement read off a curve that DIVERGES as tgo -> 0 (glint ' ...
             'carries 1/(Vc*tgo)), so the number depends entirely on where ' ...
             'the plot stops. Scoring it would be scoring our own grid ' ...
             'choice against the author''s.']);
    end
end
end


% =============================================================================
%   BENCH ROW PLUMBING
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
    error('mgs:zarchan:adjointWithAutopilot:band', ...
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


function k = findRadome(cfg, r)
% Exact-ish only.  Interpolating here would invent a radome slope the sweep
% never ran and then score the book against it.
k = find(abs(cfg.radome - r) < 1e-12, 1);
end


function [rneg, rpos, swing] = radomeSwing(rad, rms, level)
%RADOMESWING  Where the rms miss crosses LEVEL either side of zero slope.
%   Linear interpolation between adjacent grid points; NaN if the level is
%   already exceeded at R = 0, or if it is never crossed inside the grid.
rneg = NaN; rpos = NaN; swing = NaN;
if numel(rad) < 3 || ~issorted(rad)
    % An unsorted grid would make "either side of zero" meaningless and the
    % interpolation would return a crossing that is not on any curve.
    return;
end
[~, i0] = min(abs(rad));
if ~isfinite(rms(i0)) || rms(i0) > level, return; end
for i = i0-1:-1:1
    if rms(i) > level
        rneg = rad(i+1) + (rad(i)-rad(i+1))*(level-rms(i+1))/(rms(i)-rms(i+1));
        break;
    end
end
for i = i0+1:numel(rad)
    if rms(i) > level
        rpos = rad(i-1) + (rad(i)-rad(i-1))*(level-rms(i-1))/(rms(i)-rms(i-1));
        break;
    end
end
if isfinite(rneg) && isfinite(rpos), swing = rpos - rneg; end
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
%   collides would print two different provenances as the same thing.
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


% =============================================================================
%   RENDER
% =============================================================================
function s = render(R, cfg, offbook)
W = 100;
L = {};
L{end+1} = 'Zarchan Listing 23.3 -- ADJOINT OF HOMING LOOP WITH DETAILED FLIGHT-CONTROL SYSTEM';
L{end+1} = '*** UNITS ARE FEET, ft/s, deg, deg/s, g = 32.2 ft/s^2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['tf = %g s   h = %g s   Vm = %g ft/s   Vc = %g ft/s   N'' = %g   ' ...
                    'seeker T1 = %g s  filter T2 = %g s'], ...
    cfg.tf, cfg.h, cfg.vm, cfg.vc, cfg.xnp, cfg.t1, cfg.t2);
L{end+1} = sprintf(['FCS design goals: tau = %g s, crossover = %g rad/s, damping = %g   ' ...
                    '|  actuator %g rad/s / %g'], cfg.tau, cfg.wcr, cfg.zeta, cfg.wact, cfg.zact);
L{end+1} = sprintf('sweep mode = %g  [%s]   tint = %g s', cfg.miss, missUnits(cfg.miss), cfg.tint);
L{end+1} = sprintf('alt grid    = %s ft', mat2str(cfg.alt));
L{end+1} = sprintf('radome grid = %s', gridStr(cfg.radome));
L{end+1} = sprintf('off-book    : %s', offbookStr(offbook));
L{end+1} = repmat('=', 1, W);

% ---- the lag ------------------------------------------------------------
L{end+1} = 'THE DERIVED LAG -- the reason this port exists';
L{end+1} = sprintf('  %8s  %9s  %11s  %13s  %13s  %9s  %9s', 'alt [ft]', 'tau goal', ...
    'W [rad/s]', 'tau+2z/W [s]', 't63 meas [s]', 'DC gain', 'settled');
for i = 1:numel(R.fc)
    L{end+1} = sprintf('  %8g  %9.3f  %11.2f  %13.4f  %13.4f  %9.6f  %8.1f%%', ...
        R.fc(i).alt, cfg.tau, R.fc(i).w, R.fc(i).tau_total, ...
        R.fc(i).step.t63_s, R.fc(i).step.dc_gain, ...
        100*R.fc(i).step.settled_frac); %#ok<AGROW>
end
L{end+1} = '  tau+2z/W is the p.538 closed form; t63 is measured off the Listing 23.1 step';
L{end+1} = '  response against the ANALYTIC steady state.  DC gain is that steady state over';
L{end+1} = '  the command and is 1 by an exact identity; "settled" is how far the 1-s run got,';
L{end+1} = '  and reading t63 off THAT last sample instead costs 6% of the lag.';
L{end+1} = '  The gain algorithm holds this lag near-constant while the airframe changes by 6x.';
L{end+1} = '  Our own ~8 s gamma-tracking lag is held constant by nothing -- that is the gap.';
L{end+1} = repmat('-', 1, W);

% ---- flight conditions ---------------------------------------------------
L{end+1} = 'FLIGHT CONDITIONS -- aero from geometry (Ch.22), gains from the Ch.23 algorithm';
L{end+1} = sprintf('  %8s %10s %9s %9s %9s %9s %9s %8s %8s %8s', ...
    'alt', 'Q[psf]', 'Ma', 'Md', 'Za', 'Zd', 'wZ', 'wAF', 'zAF', 'alf_tr');
for i = 1:numel(R.fc)
    F = R.fc(i);
    L{end+1} = sprintf('  %8g %10.1f %9.3f %9.3f %9.4f %9.5f %9.3f %8.3f %8.5f %8.4f', ...
        F.alt, F.q, F.xma, F.xmd, F.za, F.zd, F.wz, F.waf, F.zaf, F.trim.alftr_deg); %#ok<AGROW>
end
L{end+1} = sprintf('  %8s %10s %9s %9s %9s %9s %9s %8s', ...
    'alt', 'K1[g/dg]', 'Ta[s]', 'K3', 'KA', 'wI', 'KR[s]', 'KDC');
for i = 1:numel(R.fc)
    F = R.fc(i);
    L{end+1} = sprintf('  %8g %10.4f %9.4f %9.4f %9.4f %9.4f %9.5f %8.4f', ...
        F.alt, F.xk1, F.ta, F.xk3, F.xka, F.wi, F.xkr, F.xkdc); %#ok<AGROW>
end
L{end+1} = '  Ma/Md are 1/s^2, Za/Zd 1/s, alf_tr is the 10-g trim angle of attack in deg.';
L{end+1} = '  a = 1000 ft/s at EVERY altitude -- the book''s approximation, deliberately kept.';
L{end+1} = repmat('-', 1, W);

% ---- structural self-check ----------------------------------------------
if isfield(R.selfcheck, 'ok')
    C = R.selfcheck;
    L{end+1} = 'STRUCTURAL SELF-CHECK (wiring, not the book -- a failure here voids everything)';
    L{end+1} = sprintf('  radome gated by R: X4-X5 == 0 at R = 0        : %9.3e  tol %g      %s', ...
        C.radome_zero_resid, C.radome_zero_tol, tf2s(C.radome_zero_ok));
    L{end+1} = sprintf('  radome path alive: miss ratio at 50 kft, -.06 : %9.3f  tol >=%.1f  %s', ...
        C.radome_alive_ratio, C.radome_alive_tol, tf2s(C.radome_alive_ok));
    L{end+1} = sprintf('  measured t63 vs closed form tau + 2*zeta/W    : %9.3e  tol %.0e  %s', ...
        C.lag_rel, C.lag_tol, tf2s(C.lag_ok));
    L{end+1} = sprintf('  DC gain ~ 1 and equal across altitude         : %9.3e  tol %.0e  %s', ...
        C.dc_err, C.dc_tol, tf2s(C.dc_ok));
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
    L{end+1} = sprintf('  %-34s %-14s %-22s %8s %-7s %s', ...
        'what', 'book', 'ours', 'diff', 'band', 'verdict');
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch b.kind
        case 'value'
            if b.skipped
                L{end+1} = sprintf('  %-34s %-14s %-22s %8s %-7s SKIP', ...
                    trunc(b.what,34), trunc(b.book_str,14), '--', '--', b.band); %#ok<AGROW>
            else
                L{end+1} = sprintf('  %-34s %-14s %-22s %+7.2f%% %-7s %s', ...
                    trunc(b.what,34), trunc(b.book_str,14), trunc(b.ours_str,22), ...
                    b.pct, bandTag(b.band, b.tol), tf2s(b.ok)); %#ok<AGROW>
            end
        case {'shape','bound'}
            L{end+1} = sprintf('  %-34s %-14s %-22s %8s %-7s %s', ...
                trunc(b.what,34), trunc(b.book_str,14), trunc(b.ours_str,22), ...
                '--', b.band, tf2s(b.ok)); %#ok<AGROW>
        case 'info'
            L{end+1} = sprintf('  %-34s %-14s %-22s %8s %-7s INFO', ...
                trunc(b.what,34), trunc(b.book_str,14), trunc(b.ours_str,22), ...
                '--', 'info'); %#ok<AGROW>
        otherwise
            L{end+1} = sprintf('  %-34s %-14s %-22s %8s %-7s SKIP', ...
                trunc(b.what,34), trunc(b.book_str,14), '--', '--', 'skip'); %#ok<AGROW>
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
L{end+1} = repmat('-', 1, W);

% ---- the sweep -----------------------------------------------------------
L{end+1} = sprintf('SWEEP vs RADOME SLOPE  --  %s  (Figs. 23.16/23.17)', missUnits(cfg.miss));
hdr = sprintf('  %7s', 'R');
for i = 1:numel(R.fc), hdr = [hdr sprintf('  %11s', sprintf('%g kft', R.fc(i).alt/1000))]; end %#ok<AGROW>
L{end+1} = hdr;
for k = 1:numel(cfg.radome)
    row = sprintf('  %+7.3f', cfg.radome(k));
    for i = 1:numel(R.fc)
        row = [row sprintf('  %11.3f', R.fc(i).sweep.rms(k))]; %#ok<AGROW>
    end
    L{end+1} = row; %#ok<AGROW>
end
L{end+1} = '  channel breakdown per altitude is in R.fc(i).sweep.{fn,rn,gl,nt}:';
L{end+1} = '    fn = range independent noise   rn = SEMIACTIVE   gl = glint   nt = maneuver';
L{end+1} = '    (fn/rn follow the LISTING''s names, which are not Table 23.4''s -- see header)';
L{end+1} = repmat('-', 1, W);

% ---- modes 2 and 3 -------------------------------------------------------
if ~isempty(R.finrate)
    L{end+1} = 'RMS FIN RATE [deg/s] at fixed time-to-go, zero radome slope (Figs. 23.19/23.20)';
    L{end+1} = modeHeader(R.finrate);
    for i = 1:numel(R.fc)
        L{end+1} = modeRow(R.finrate, i, R.fc(i).alt); %#ok<AGROW>
    end
    L{end+1} = '  flight time = tf - tgo, so the LEFTMOST column is the END of the flight.';
end
if ~isempty(R.accel)
    L{end+1} = 'RMS COMMANDED ACCELERATION [g] at fixed time-to-go (Figs. 23.22/23.23)';
    L{end+1} = modeHeader(R.accel);
    for i = 1:numel(R.fc)
        L{end+1} = modeRow(R.accel, i, R.fc(i).alt); %#ok<AGROW>
    end
end
if ~isempty(R.tau05)
    T = R.tau05;
    L{end+1} = sprintf('TAU 0.3 vs 0.5 s at %g kft (Figs. 23.18/23.21/23.24)', T.alt/1000);
    L{end+1} = sprintf('  rms miss    R = %+.2f : %10.2f -> %10.2f ft', ...
        T.radome(1), T.rms_miss(1,1), T.rms_miss(2,1));
    L{end+1} = sprintf('  rms miss    R = %+.2f : %10.2f -> %10.2f ft', ...
        T.radome(2), T.rms_miss(1,2), T.rms_miss(2,2));
    L{end+1} = sprintf('  rms fin rate tgo = %g s: %10.2f -> %10.2f deg/s', ...
        T.tgo_fin, T.rms_finrate(1), T.rms_finrate(2));
    L{end+1} = sprintf('  rms cmd accel        : %s  ->  %s  g   (tgo = %s s)', ...
        num2str(T.rms_accel(1,:), '%.2f '), num2str(T.rms_accel(2,:), '%.2f '), ...
        mat2str(T.tgo_acc));
    L{end+1} = sprintf('  t63                  : %.4f -> %.4f s', T.t63_s(1), T.t63_s(2));
    L{end+1} = '  a longer time constant buys FIN RATE and radome margin, not acceleration.';
end
L{end+1} = repmat('=', 1, W);

% ---- the footer that has to travel with any quoted number ---------------
L{end+1} = 'READ THIS BEFORE QUOTING ANY NUMBER ABOVE';
L{end+1} = '  Constant speed (Mach 3), constant altitude, no gravity, no drag, no thrust, no';
L{end+1} = '  acceleration limit, aero LINEARIZED about a 10-g trim.  The disturbances are three';
L{end+1} = '  seeker noises and a uniformly distributed target maneuver.  Our bullseye is';
L{end+1} = '  STATIONARY and our measured discriminant is release heading error |eps0|, an';
L{end+1} = '  initial-condition channel that is not in this adjoint at all.  A miss distance';
L{end+1} = '  computed here is NOT a sortie prediction.  What DOES transfer is the lag block';
L{end+1} = '  above: a flight-control time constant derived from gains and airframe, and held';
L{end+1} = '  constant across flight conditions by a gain algorithm we do not have.';
if ~isempty(offbook)
    L{end+1} = sprintf(['  NOTE this run moved %s off the book, so the rows that depend on ' ...
        'them were skipped, not compared.'], offbookStr(offbook));
end
s = strjoin(L, newline);
end


function h = modeHeader(M)
h = sprintf('  %8s', 'alt [ft]');
for k = 1:numel(M.tgo)
    h = [h sprintf('  %12s', sprintf('tgo %.4g s', M.tgo(k)))]; %#ok<AGROW>
end
end


function r = modeRow(M, i, alt)
r = sprintf('  %8g', alt);
for k = 1:numel(M.tgo)
    r = [r sprintf('  %12.3f', M.rms(i,k))]; %#ok<AGROW>
end
end


function s = gridStr(v)
% numel(v) >= 3, not >= 2: with two points diff(diff(v)) is EMPTY, and
% `empty < 1e-12` is an empty logical, which makes && throw "Operands to the
% || and && operators must be convertible to logical scalar values".  A
% display helper is not worth an exception, and a two-point grid is exactly
% what someone debugging would pass.
if numel(v) == 1
    s = sprintf('%g', v);
elseif numel(v) >= 3 && max(abs(diff(diff(v)))) < 1e-12
    s = sprintf('%g : %g : %g  (%d points)', v(1), v(2)-v(1), v(end), numel(v));
else
    s = mat2str(v);
end
end


% =============================================================================
function drawCurves(R, cfg)
%DRAWCURVES  Figs. 23.16/23.17 (miss vs radome slope), 23.3/23.12 (step
%   response) and 23.13 (fin rate).  Plotting is OFF by default: the
%   benchmark run opens one window per altitude otherwise.
figure('Name', 'Zarchan Fig. 23.16 -- rms miss vs radome slope');
hold on;
for i = 1:numel(R.fc)
    plot(cfg.radome, R.fc(i).sweep.rms, 'LineWidth', 1.2);
end
grid on; hold off;
xlabel('Radome slope'); ylabel(sprintf('Standard deviation of miss [%s]', R.fc(1).sweep.units));
legend(arrayfun(@(f) sprintf('%g kft', f.alt/1000), R.fc, 'UniformOutput', false), ...
    'Location', 'best');
title('Miss distance vs radome slope (Listing 23.3)');

figure('Name', 'Zarchan Fig. 23.17 -- error budget vs radome slope, top altitude');
i = numel(R.fc);  s = R.fc(i).sweep;
plot(cfg.radome, [s.fn; s.rn; s.gl; s.nt; s.rms], 'LineWidth', 1.2); grid on;
xlabel('Radome slope'); ylabel('Standard deviation of miss [ft]');
legend({'range independent','semiactive','glint','target maneuver','RMS'}, 'Location','best');
title(sprintf('Error budget at %g kft', R.fc(i).alt/1000));

figure('Name', 'Zarchan Figs. 23.3/23.12 -- autopilot step response');
hold on;
for i = 1:numel(R.fc)
    plot(R.fc(i).step.t, R.fc(i).step.nl_g, 'LineWidth', 1.2);
end
yline(cfg.xncg, 'k--'); grid on; hold off;
xlabel('Time [s]'); ylabel('Achieved acceleration [g]');
legend([arrayfun(@(f) sprintf('%g kft', f.alt/1000), R.fc, 'UniformOutput', false), ...
        {'command'}], 'Location', 'best');
title('Listing 23.1 step response -- the lag, measured');

figure('Name', 'Zarchan Fig. 23.13 -- fin rate, step response');
hold on;
for i = 1:numel(R.fc)
    plot(R.fc(i).step.t, R.fc(i).step.finrate_dps, 'LineWidth', 1.2);
end
grid on; hold off;
xlabel('Time [s]'); ylabel('Fin rate [deg/s]');
legend(arrayfun(@(f) sprintf('%g kft', f.alt/1000), R.fc, 'UniformOutput', false), ...
    'Location', 'best');
title('Fin rate increases with altitude');
end
