function R = airframeSim(opts)
%MGS.ZARCHAN.AIRFRAMESIM  Zarchan Listing 21.1 -- the NONLINEAR force-and-
%   moment airframe simulation, ported line for line.
%
%   R = mgs.zarchan.airframeSim()                       % book runs + benchmarks
%   R = mgs.zarchan.airframeSim('alt', 30000)           % one off-book altitude
%   R = mgs.zarchan.airframeSim('del_deg', 10, 'benchmark', false)
%   R = mgs.zarchan.airframeSim('plot', true)
%
%   A 5-deg tail deflection is applied to the hypothetical 1000-lb, 20-ft,
%   1-ft-diameter tail-controlled missile of Fig. 21.3 at Mach 3, and the two
%   airframe differential equations -- angle-of-attack rate and body angular
%   acceleration -- are integrated for 2 s (Figs. 21.4-21.7).
%
% ===========================================================================
%   *** UNITS ARE THE BOOK'S.  FEET, SLUGS, LB, g = 32.2, deg via 57.3. ***
%
%   Zarchan works in ft, ft/s, lb, slug/ft^3 with g = 32.2 ft/s^2 and converts
%   radians to degrees with the rounded 57.3, not 180/pi.  The rest of mgs is
%   metres.  This file is DELIBERATELY left in the book's units so the
%   published numbers can be compared digit for digit with no conversion
%   sitting in between to be got wrong.
%
%   Two rounded constants are kept ON PURPOSE and both are load-bearing:
%       57.3   for rad->deg           (180/pi = 57.29578)
%       1845   in K1 and K3           (57.3*32.2 = 1844.66)
%   The second one means the algebraic identity  nL_trim[g] == K1*delta[deg]
%   holds only to 1 part in 5400 rather than to machine precision.  That is
%   NOT a port defect; it is what the book's arithmetic does, and the
%   structural check below asserts the residual at 2e-4 and says why rather
%   than tightening it to 1e-12 and failing forever.  Convert to SI at the
%   CALL SITE, visibly, or not at all.
% ===========================================================================
%
%   WHY THIS CHAPTER MATTERS MORE THAN ANY OTHER PORT WE HAVE
%
%   Every lag in every previous port is a LUMPED CONSTANT SOMEBODY CHOSE:
%   one pole (Listing 3.1), five equal poles (Listing 6.1), an autopilot
%   design goal tau = 0.3 s (Listing 23.3).  Our own vehicle has none of
%   that.  We MEASURED a gamma-tracking gain of 0.063 against the model's
%   1.385 -- roughly 8 s of lag, saturating on 26 of 32 sorties -- and we
%   cannot derive that 8 s from anything.  We can only fit it.
%
%   Chapter 21 is where a time constant comes out of AERODYNAMICS.  Nothing
%   here is chosen: weight, length, diameter, fin areas and c.g. go in, and
%   Ma, Md, Za, Zd, wAF, zAF, wZ and Ta come out.  This port therefore gives
%   us the vocabulary for a T we could DEFEND instead of fit.
%
%   The single most useful result, and it is not the one you expect:
%
%     * the AIRFRAME time constant 1/wAF is 0.040 s at sea level and 0.104 s
%       at 50 kft (book line 6784: "the time constant variation is between
%       0.033 s and 0.1 s"; line 6794: "in general the airframe time constant
%       is fast").  That is 200x faster than our measured 8 s.  Whatever our
%       lag is, IT IS NOT AIRFRAME DYNAMICS.  A real airframe's own pitch
%       response is tens of milliseconds; ours cannot be blamed on the body.
%
%     * the TURNING-RATE time constant Ta is 0.56 s at sea level and 3.78 s
%       at 50 kft (book line 6810: "approximately 4 s").  THAT is the one in
%       our range.  Ta is how long the airframe takes to turn the VELOCITY
%       VECTOR once the body has already pitched -- i.e. it is a
%       flight-path-angle lag, which is exactly the quantity our 0.063 gain
%       is measured on.  It scales as 1/Q, so it gets worse with altitude
%       and better with dynamic pressure.
%
%   So Ch.21 splits our 8 s into two questions that used to be one: how much
%   of it is the aerodynamic cost of turning a velocity vector (Ta, derivable,
%   seconds) and how much is ArduPilot's outer loop on top (not derivable
%   from here, and the residual is what we should actually be attacking).
%   R.lag collects both columns for exactly this comparison.
%
%   WHAT THIS DOES NOT MODEL, AND MUST NOT BE REPORTED AS IF IT DID.
%   Constant speed (VM is an input and never changes), constant altitude,
%   no gravity, no drag bookkeeping, no thrust, no actuator, no autopilot,
%   no guidance, no fin rate or fin travel limit, a FLAT 1000 ft/s speed of
%   sound, and supersonic linear-theory aero (B = sqrt(M^2-1)).  Pitch plane
%   only.  There is no miss distance anywhere in this file and none can be
%   computed from it.
%
% ---------------------------------------------------------------------------
%   PROVENANCE, AND THE TRANSCRIPTION
%
%   Source: matlab/zarchan/_listings/listing_21_01.m, extracted from
%   zarchan.txt:6510-6513 and OCR-repaired.  Its header reports FOUR repaired
%   comparisons and NO unresolved ones, and all four are unambiguous from
%   context -- there is no variable-to-variable comparison in this listing:
%       if ALT,=30000.    ->  if ALT <= 30000.     (sea level takes branch 1)
%       while T,1.99999   ->  while T < 1.99999    (T grows from 0)
%       while STEP,=1     ->  while STEP <= 1      (STEP is 1 then 2)
%       if S.=.0099999    ->  if S >= .0099999     (S accumulates H upward)
%   The book's very long semicolon-separated lines are its own formatting,
%   not damage.  Nothing else needed a decision.
%
%   PULLED IN FROM THE SAME CHAPTER, AND SAID SO.  Listing 21.1 alone cannot
%   be checked against most of what Ch.21 publishes, because the published
%   numbers (K1, K3, Ta, wAF, zAF) are LINEARIZED quantities.  The trim
%   quadratic and the linearization are derived in the chapter's own prose
%   (zarchan.txt:6698-6699, p.487) and appear verbatim in the preamble of
%   Listing 21.2.  LINEARAERO() below is that preamble ONLY -- the trim
%   solve and the aerodynamic derivatives.  Listing 21.2's own integration
%   loop is NOT here; that is a separate port.  Every row derived this way is
%   labelled 'Listing 21.2 preamble' in its source string so it can never be
%   read as an output of Listing 21.1.
%
% ---------------------------------------------------------------------------
%   THE OCR MINUS SIGN.  READ THIS BEFORE DOUBTING A BOOK COLUMN.
%
%   The extraction turned this chapter's minus signs into the digit '2'.
%   Left unread, the prose says the missile trims at +25 deg of alpha, which
%   is nonsense and would send a reader hunting for a factor-of-five bug that
%   does not exist.  The reading is forced by the book's OWN arithmetic on
%   line 6853, which survived with both the mangled operands and the mangled
%   product:
%
%       "22.7  5 = 213.5"      ->  -2.7 * 5 = -13.5
%       "2 0.4 5 = 22"         ->  -0.4 * 5 = -2
%       "21.6 5 = 28"          ->  -1.6 * 5 = -8
%       "20.24 5 = 21.2"       ->  -0.24 * 5 = -1.2
%
%   and it is confirmed independently by line 6527, which states the SIGN in
%   words: "a 5-deg fin deflection in the positive direction causes the
%   missile to build up to a NEGATIVE angle of attack".  So on line 6527
%   "initially approaches 28 deg ... trim value of approximately 25 deg"
%   reads -8 deg and -5 deg.  This port computes -8.3 and -4.45.  Every
%   affected benchmark row carries the raw OCR text AND the reading in its
%   detail string, so the decision is auditable and is not hidden in a
%   constant.
%
% ---------------------------------------------------------------------------
%   WHAT IS SCORED AND WHAT IS DELIBERATELY NOT
%
%   Ch.21's numeric content is in the running text and is greppable.  Its
%   figures are image links that did not survive extraction, so a value that
%   exists only on a plot does not exist for us.  The rule this port follows:
%
%     SCORED   a number printed in the prose (13 g, -2.7, -1.6, 4 s, ...),
%              with the band set by how the AUTHOR got it -- almost all of
%              these are him reading Figs. 21.5/21.16/21.18/21.19 by eye and
%              quoting one or two significant figures, hence 'graph' 15%.
%     SCORED   an ORDERING or SIGN claim in the prose, turned into a shape
%              row.  "the airframe natural frequency decreases with
%              increasing altitude" is a sharper test than any single number
%              read off a plot, because it cannot be passed by accident.
%     NOT      an adjective.  "the linear approximation starts to
%              SCORED    deteriorate" (line 6770) is reported as INFO with
%              the computed nonlinearity index beside it, and no verdict.
%     NOT      the two Fig. 21.12/21.13 FAMILY bands (wAF in 10-30 rad/s,
%              SCORED    zAF in 0.02-0.065).  Those endpoints are the extremes
%              of a whole altitude x VELOCITY family whose velocity grid is an
%              image link.  Our sea-level point sits comfortably inside both
%              and IS scored as a bound; our 50-kft point at 3000 ft/s sits
%              a few per cent BELOW both lower ends (~4% on wAF, ~3% on zAF),
%              which cannot be adjudicated without knowing which speeds the
%              figure swept -- and the sign of the miss is the direction a
%              richer velocity grid would move the endpoints anyway.
%              That row is INFO and says so.  Widening the band to make it
%              green would be inventing a book claim.
%
% ---------------------------------------------------------------------------
%   BENCHMARK MODE IS A FLAG.  IT IS NOT nargin.
%
%   In a function whose arguments block declares ONLY name-value options,
%   MATLAB reports nargin = 0 NO MATTER HOW MANY were passed (measured,
%   R2025b).  Three ports in the first wave used `if nargin == 0`: one became
%   infinite recursion and two silently replaced the caller's arguments with
%   the benchmark run.  'benchmark' is the ONLY switch here.
%
%   It defaults TRUE, so a bare call is a CHECKED call.  Benchmark mode never
%   overrides anything the caller passed; instead every book value that
%   depends on a parameter the caller moved is SKIPPED with the reason
%   printed.  That is why the book defaults live in BOOKNOMINAL() and the
%   arguments-block defaults are [] -- "the caller supplied this" is then
%   exactly "the field is non-empty", with no second copy of the constants to
%   drift out of agreement.
%
% ---------------------------------------------------------------------------
%   OUTPUT
%
%   R.fc(i)      one per altitude, in R.cfg.alt order
%                  .alt .rho .q .mach            flight condition
%                  .aero                          geometry + Q + B + TMP1..4
%                  .trim  .alftr .alftr_deg .y1 .y2 .y3 .disc .dcm_dalf
%                  .lin   Listing 21.2's preamble: .cna .cnd .cma .cmd
%                         .xma .xmd .za .zd .wz .waf .zaf .xk1 .xk3 .ta
%                         plus the JACOBIAN twins .xma_jac .waf_jac .zaf_jac
%                  .sim   .t .accel_g .alpha_deg .thetadot_dps  (the book's
%                         0.01-s sampled arrays; thetadot is ours, see below)
%                  .resp  measured features of .sim: .settled_accel_g
%                         .settled_alpha_deg .settled_thd_dps .peak_alpha_deg
%                         .wn .zeta .n_extrema .wrongway_* and .ta_meas_s,
%                         the turning-rate time constant read straight off
%                         the run as alpha/gammadot.  Each channel also
%                         carries .settled_model ('ring' or, degraded,
%                         'kernel'), .settled_kernel (what the superseded
%                         extremum kernel said), .settled_s/.settled_w (the
%                         pole pair the fit ended on) and .settled_res
%                  .dc_offset_rel  the predicted second-order offset the
%                         alpha^2 term puts on the centre of the ring; it is
%                         the whole residual of the trim self-check
%   R.lag        the reason this file exists: per altitude, .tau_airframe_s
%                (1/wAF), .ta_turnrate_s, and the comparison against our own
%                measured ~8 s
%   R.bench      computed vs book, one row per published statement, each with
%                .source naming the figure/page AND the zarchan.txt line
%   R.selfcheck  structural invariants and their measured residuals
%   R.pass       true iff every EVALUATED row passed; false when benchmark
%                mode is off, because nothing was checked
%   R.cfg        resolved configuration; R.offbook names what the caller moved
%   R.summary    the printed text
%
%   Bench rows carry BOTH numeric (.book/.ours, NaN on shape/bound/info rows)
%   and text (.book_str/.ours_str, always populated) so a benchSuite adapter
%   can render every kind through one path -- the same schema
%   adjointWithAutopilot uses, so its adapter is the template.
%
%   See also MGS.ZARCHAN.BENCHSUITE, MGS.ZARCHAN.ADJOINTWITHAUTOPILOT,
%   MGS.ZARCHAN.ADJOINTBINOMIAL5.

arguments
    % Defaults are [] on purpose -- see BOOKNOMINAL().  Empty means "use the
    % book value"; non-empty means "the caller chose this", which is what
    % gates the benchmark rows.  Do NOT replace these with literals: that
    % puts a second copy of every book constant in the file and destroys the
    % only mechanism this port has for telling an off-book run apart from a
    % broken one.
    opts.alt      double {mustBeReal, mustBeFinite} = []  % ALT     [ft]
    opts.del_deg  double {mustBeReal, mustBeFinite} = []  % DEL*57.3 [deg] fin
    opts.vm       double {mustBeReal, mustBeFinite} = []  % VM      [ft/s]
    opts.wgt      double {mustBeReal, mustBeFinite} = []  % WGT     [lb]
    opts.a        double {mustBeReal, mustBeFinite} = []  % A       [ft/s] speed of sound
    opts.tstop    double {mustBeReal, mustBeFinite} = []  % the 1.99999 in the loop
    opts.h        double {mustBeReal, mustBeFinite} = []  % H       [s]
    opts.benchmark (1,1) logical = true   % THE ONLY SWITCH.  Not nargin.
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

GRAV_FT = 32.2;    % g, the book's value.  Not 9.80665, not 9.81.
DEG     = 57.3;    % the book's rad->deg.  NOT 180/pi -- see the units block.

N = bookNominal();
[cfg, offbook] = resolveConfig(opts, N);

R = struct();
R.units   = 'ft, ft/s, lb, deg, g = 32.2 ft/s^2, rad->deg via 57.3 -- Zarchan''s units, NOT SI';
R.listing = 'Zarchan Listing 21.1 (zarchan.txt:6510), nonlinear airframe of Fig. 21.3';
R.opts    = opts;
R.cfg     = cfg;
R.offbook = offbook;
R.nominal = N;

% ---- one flight condition per altitude ------------------------------------
for i = 1:numel(cfg.alt)
    F = flightCondition(cfg.alt(i), cfg, GRAV_FT, DEG);
    if i == 1, R.fc = F; else, R.fc(i) = F; end %#ok<AGROW>
end

% ---- the lag summary: the point of the exercise ---------------------------
R.lag = lagSummary(R.fc);

% ---- board ----------------------------------------------------------------
if cfg.benchmark
    R.selfcheck = structuralCheck(R.fc, cfg, GRAV_FT, DEG);
    R.bench     = benchmarks(R, cfg, offbook, DEG);
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
    % whose altitude or fin deflection puts every book value out of reach has
    % checked NOTHING, and that is the shape of green light this repo has
    % been burned by.
    if ~any(ev)
        error('mgs:zarchan:airframeSim:noBenchmark', ...
            ['not one book value could be evaluated.  Ch.21 publishes numbers ' ...
             'at alt = 0 and 50000 ft with a 5-deg fin at 3000 ft/s; this run ' ...
             'has alt = %s and moved %s.  Refusing to hand back an ' ...
             'unvalidated port silently -- pass ''benchmark'', false if you ' ...
             'meant to run off-book.'], mat2str(cfg.alt), offbookStr(offbook));
    end
    if ~R.pass
        bad = find(~[R.bench.ok] & ev);
        error('mgs:zarchan:airframeSim:benchmark', ...
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
%BOOKNOMINAL  Every constant Listing 21.1 sets, in ONE place.
%
%   ALT is a VECTOR here and it is the only nominal that is.  Listing 21.1
%   itself has ALT=0., and the book then says "The simulation was also run
%   when the altitude was increased to 50 kft (ALT=50000)" (line 6529).  Both
%   runs are needed: half the published numbers are the 50-kft ones, and
%   every ordering claim in the chapter is a comparison BETWEEN them.  A port
%   that ran only sea level could not evaluate a single shape row.
N = struct();
N.alt     = [0 50000];   % ALT [ft]   -- book: 0, then line 6529's 50 kft
N.del_deg = 5;           % DEL = 5./57.3
N.vm      = 3000;        % VM  [ft/s] -- Mach 3 against the flat A = 1000
N.wgt     = 1000;        % WGT [lb]
N.a       = 1000;        % A   [ft/s] speed of sound, ALTITUDE INDEPENDENT
N.tstop   = 2.0;         % the loop runs while T < 1.99999
N.h       = 0.0025;      % H   [s]
end


% =============================================================================
function [cfg, offbook] = resolveConfig(opts, N)
%RESOLVECONFIG  Merge caller over book, and RECORD which fields moved.
%
%   The record is the whole point.  A benchmark row compared against a book
%   value for a different flight condition is worse than no row at all: it
%   prints a red verdict that means "you changed something", and the next
%   reader spends an afternoon on a port that was never broken.
cfg = struct();
offbook = {};
f = fieldnames(N);
for i = 1:numel(f)
    if isempty(opts.(f{i}))
        cfg.(f{i}) = N.(f{i});
    else
        cfg.(f{i}) = opts.(f{i});
        if ~isequal(cfg.(f{i}), N.(f{i}))
            offbook{end+1} = f{i}; %#ok<AGROW>
        end
    end
end
cfg.benchmark = opts.benchmark;
cfg.render    = opts.render;
cfg.plot      = opts.plot;
cfg.assert    = opts.assert;

if isempty(cfg.alt)
    error('mgs:zarchan:airframeSim:noAlt', ...
        '''alt'' is empty; there is no flight condition to simulate.');
end
if cfg.h <= 0 || cfg.tstop <= 0
    error('mgs:zarchan:airframeSim:step', ...
        'h = %g and tstop = %g must both be positive.', cfg.h, cfg.tstop);
end
if cfg.del_deg == 0
    % Not a pathological input so much as a NON-EXPERIMENT.  With no fin
    % deflection the airframe sits at alpha = 0 for ever: no transient, no
    % extrema, no settled value to estimate, and every measured feature
    % downstream comes back NaN.  Worse, the trim angle is exactly zero, so
    % every relative residual in the structural check divides by it.  Say so
    % here, once, instead of emitting a cascade of NaNs that has to be
    % reverse-engineered.
    error('mgs:zarchan:airframeSim:noFin', ...
        ['del_deg = 0: nothing excites the airframe, so there is no ' ...
         'response to measure and the trim angle every residual is ' ...
         'normalised by is exactly zero.  Ch.21''s case is a 5-deg fin.']);
end
if cfg.tstop <= 4*cfg.h
    error('mgs:zarchan:airframeSim:tooShort', ...
        ['tstop = %g s is not even four steps of h = %g s.  The book''s ' ...
         'sampler emits nothing in that time and every measured feature ' ...
         'below would come back NaN and read as "not applicable" rather ' ...
         'than as "you asked for no simulation".'], cfg.tstop, cfg.h);
end

% The book's sampler is `if S >= .0099999`, kept verbatim below.  It emits one
% sample per 0.01 s only when h divides 0.01.  Other steps still integrate
% correctly but land on a DRIFTING, non-uniform grid -- and the parabolic
% extremum refinement in RESPONSEFEATURES assumes a uniform grid, so this is
% not merely cosmetic.  Degrade visibly, never silently.
rr = rem(0.01, cfg.h);
if abs(rr) > 1e-12 && abs(rr - cfg.h) > 1e-12
    warning('mgs:zarchan:airframeSim:sampleGrid', ...
        ['h = %g does not divide the book''s 0.01-s sample interval, so the ' ...
         'output grid is non-uniform.  The integration is unaffected; the ' ...
         'measured frequency and damping lose their sub-sample refinement.'], ...
        cfg.h);
end
end


% =============================================================================
function F = flightCondition(ALT, cfg, GRAV_FT, DEG)
%FLIGHTCONDITION  Aero constants, trim, linearization, the run, the features.
F = struct();
F.alt  = ALT;
F.aero = aeroConstants(ALT, cfg, GRAV_FT);
F.rho  = F.aero.rho;  F.q = F.aero.q;  F.mach = F.aero.mach;
F.trim = trimForFin(F.aero, cfg.del_deg/DEG, DEG);
F.lin  = linearAero(F.aero, F.trim, cfg, GRAV_FT);

% The step-size guard belongs HERE, after wAF is known, because it is not a
% guard on h -- it is a guard on the ratio between the numerical
% ANTI-damping of the integrator and the physical damping of the airframe.
% Heun's amplification on a purely imaginary eigenvalue is |1+z+z^2/2| =
% sqrt(1+(w*h)^4/4), i.e. GROWTH of (w*h)^4/8 per step, or w^4*h^3/8 per
% second.  The airframe decays at zeta*w per second.  If the first is a
% noticeable fraction of the second, the measured damping below is partly the
% integrator's and this file would report a fitted number as a derived one --
% precisely the failure it exists to prevent.  The book's h = 0.0025 puts the
% ratio at 6e-4 at sea level, so this never fires on a book run.
antidamp = F.lin.waf_jac^3 * cfg.h^3 / (8 * F.lin.zaf_jac);
F.antidamp_ratio = antidamp;
if antidamp > 0.05
    error('mgs:zarchan:airframeSim:stepSize', ...
        ['at alt = %g ft the integrator''s own anti-damping is %.1f%% of the ' ...
         'airframe''s physical damping (h = %g s, wAF = %.4g rad/s, ' ...
         'zAF = %.4g).  Heun grows a purely oscillatory mode at w^4*h^3/8 ' ...
         'per second; above a few per cent the measured damping ratio stops ' ...
         'being the airframe''s.  Use h <= %.3g s.'], ...
        ALT, 100*antidamp, cfg.h, F.lin.waf_jac, F.lin.zaf_jac, ...
        (0.4*F.lin.zaf_jac)^(1/3)/F.lin.waf_jac);
end

F.sim  = simulate(F.aero, cfg.del_deg/DEG, cfg, GRAV_FT, DEG);
F.resp = responseFeatures(F.sim);

% The second-order DC offset a QUADRATIC nonlinearity puts on the centre of
% the oscillation.  Expanding CM about trim gives  udd + w^2 u + beta u^2 = 0
% with beta = -K*y2, whose periodic solution of amplitude a is centred not on
% zero but on  u_dc = -beta*a^2/(2*w^2).  This is PHYSICS, not integration
% error: negligible at sea level, where the ring has decayed to ~0.005 rad by
% t = 2 s, and ~0.5% of trim at 50 kft, where zeta ~ 0.019 leaves the ring
% almost intact.
%
% ⚠️ 2026-08-23:這個數字曾被當成「ODE 收斂到二次式根」殘差的**全部**解釋。
%    不再是了 —— RESPONSEFEATURES 的環模型已經把這一項連同它的二次諧波一起
%    當基底擬掉,alpha 殘差因此從 2.95e-3 掉到 8.6e-5,**比這裡的預測小一個
%    數量級**。留著它是因為它現在有更好的用途:它是「訂正前」殘差該有多大的
%    獨立預測,對得上就證明機制認對了。它不再是任何容差的理由。
KK   = F.aero.q*F.aero.sref*F.aero.diam/F.aero.xiyy;
beta = -KK*F.trim.y2;
amp  = ringAmplitudeRad(F.resp.alpha, DEG);
F.dc_offset_rad = -beta*amp^2/(2*(-F.lin.xma_jac));
F.dc_offset_rel = F.dc_offset_rad / F.trim.alftr;
F.ring_amp_rad  = amp;
end


% -----------------------------------------------------------------------------
function a = ringAmplitudeRad(Qa, DEG)
%RINGAMPLITUDERAD  Half the peak-to-peak of the LAST half cycle, in radians.
%   NaN if there are not two extrema to measure it from -- never 0, because a
%   zero here would predict a zero DC offset and quietly excuse a residual it
%   had not actually accounted for.
if numel(Qa.y_ext) < 2
    a = NaN;
else
    a = 0.5*abs(Qa.y_ext(end) - Qa.y_ext(end-1))/DEG;
end
end


% =============================================================================
function A = aeroConstants(ALT, cfg, GRAV_FT)
%AEROCONSTANTS  The preamble of Listing 21.1, transcribed.
%
%   GEOMETRY IS NOT AN OPTION.  This is one specific missile -- the
%   hypothetical 1000-lb, 20-ft, 1-ft-diameter tail-controlled airframe of
%   Fig. 21.3, with a 6-ft-root triangular wing and a 2-ft-root triangular
%   tail -- and every published number in the chapter is its number.  Making
%   the geometry adjustable would invite a run that is neither the book's
%   airframe nor ours, and there is no benchmark for that vehicle.
%
%   THREE THINGS IN HERE LOOK WRONG AND ARE NOT:
%
%   * A = 1000 ft/s at EVERY altitude.  The real speed of sound is nearer
%     968 ft/s at 50 kft, so Mach is a few per cent off up high.  The book
%     says so itself (line 6505: "this is an approximation since the speed of
%     sound is altitude dependent and can be as much as 10% different") and
%     uses the flat 1000 anyway.  "Fixing" it moves B, and with it every
%     derivative and every benchmark, off the book.
%   * SREF uses 3.1416, not pi.  Keeping pi shifts Sref by 2.3e-6 relative;
%     harmless here, but the file's rule is that a book constant stays a book
%     constant, because the ones that DO matter (57.3, 1845) look just as
%     harmless.
%   * XCG = 10 ft is written as a literal, not as 0.5*XL.  The book's prose
%     (p.476) says the c.g. is assumed at mid-body; the listing hard-codes
%     the result.  Both agree for THIS airframe, and the listing is what is
%     transcribed.

% Fig. 21.3, verbatim from the listing.
FR=3.; DIAM=1.; XL=20.; CTW=0.; CRW=6.; HW=2.; CTT=0.; CRT=2.; HT=2.;
XN=4.; XCG=10.; XHL=19.5;

VM = cfg.vm;  WGT = cfg.wgt;  A_SOUND = cfg.a;

% The exponential atmosphere first introduced in Ch.10 (line 6505 says so).
if ALT <= 30000.
    RHO=.002378*exp(-ALT/30000.);
else
    RHO=.0034*exp(-ALT/22000.);
end

SWING=.5*HW*(CTW+CRW); STAIL=.5*HT*(CTT+CRT); SREF=3.1416*DIAM*DIAM/4.;
XLP=FR*DIAM; SPLAN=(XL-XLP)*DIAM+1.33*XLP*DIAM/2.; XCPN=2*XLP/3;
AN=.67*XLP*DIAM; AB=(XL-XLP)*DIAM;
XCPB=(.67*AN*XLP+AB*(XLP+.5*(XL-XLP)))/(AN+AB);
XCPW=XLP+XN+.7*CRW-.2*CTW; XMACH=VM/A_SOUND;
XIYY=WGT*(3*((DIAM/2)^2)+XL*XL)/(12*GRAV_FT);
TMP1=(XCG-XCPW)/DIAM; TMP2=(XCG-XHL)/DIAM;
TMP3=(XCG-XCPB)/DIAM; TMP4=(XCG-XCPN)/DIAM;

if XMACH <= 1
    % B = sqrt(M^2-1) is the Prandtl-Glauert factor and it is IMAGINARY below
    % Mach 1.  MATLAB would hand back complex CN, CM, Ma, Md and the whole
    % file would keep running on complex numbers, printing real-looking
    % magnitudes for a flight condition the aero model does not cover.
    error('mgs:zarchan:airframeSim:subsonic', ...
        ['vm = %g ft/s is Mach %.3g against the book''s flat a = %g ft/s.  ' ...
         'This aero is SUPERSONIC-ONLY: every wing and tail term is built on ' ...
         'B = sqrt(M^2-1), which is imaginary here.  Ch.21 has no subsonic ' ...
         'branch to fall back to.'], VM, XMACH, A_SOUND);
end
B=sqrt(XMACH^2-1); Q=.5*RHO*VM*VM;

A = struct('diam',DIAM,'xl',XL,'xcg',XCG,'xhl',XHL,'xn',XN, ...
           'swing',SWING,'stail',STAIL,'sref',SREF,'xlp',XLP,'splan',SPLAN, ...
           'xcpn',XCPN,'xcpb',XCPB,'xcpw',XCPW,'an',AN,'ab',AB, ...
           'xiyy',XIYY,'tmp1',TMP1,'tmp2',TMP2,'tmp3',TMP3,'tmp4',TMP4, ...
           'b',B,'q',Q,'rho',RHO,'mach',XMACH,'vm',VM,'wgt',WGT);
end


% =============================================================================
function TR = trimForFin(A, DEL, DEG)
%TRIMFORFIN  The trim angle of attack for a GIVEN FIN DEFLECTION.
%
%   Zarchan Ch.21, p.487 (zarchan.txt:6698-6699), and verbatim in the
%   preamble of Listing 21.2.  The vehicle is at trim when the moment is
%   zero, and CM is exactly quadratic in alpha:
%
%       CM = y1*alpha + y2*alpha^2 + y3      (y3 already carries delta)
%        0 = y1*aTR   + y2*aTR^2   + y3
%       aTR = (-y1 - sqrt(y1^2 - 4*y2*y3)) / (2*y2)     "after eliminating
%                                                        the unrealistic root"
%
%   NOTE WHICH TRIM THIS IS.  Ch.22 and Ch.23 solve a DIFFERENT quadratic --
%   trim for a given ACCELERATION (10 g), not for a given fin deflection --
%   and mgs.zarchan.adjointWithAutopilot uses that one.  The two give
%   different alpha for the same airframe.  Do not copy either into the other.
%
%   AND NOTE WHAT IS ALTITUDE-INDEPENDENT.  y1, y2, y3 contain no Q and no
%   rho: they are pure geometry, B and delta.  So in THIS model the trim
%   angle of attack is EXACTLY the same at sea level and at 50 kft for a
%   given fin deflection and speed.  That is asserted as a structural
%   invariant below, and it is what makes the book's line 6529 ("at higher
%   altitudes the missile will pull more angle of attack") a statement about
%   the TRANSIENT envelope and not about trim.  See the INFO row.
Y1 = 2*A.tmp4 + 8*A.swing*A.tmp1/(A.b*A.sref) + 8*A.stail*A.tmp2/(A.b*A.sref);
Y2 = 1.5*A.splan*A.tmp3/A.sref;
Y3 = 8*A.stail*A.tmp2*DEL/(A.b*A.sref);

if Y2 == 0
    % The body-lift term is what makes this quadratic.  With y2 = 0 the
    % expression below is 0/0 and MATLAB returns NaN, which would propagate
    % into CNA, CMA and every derived gain as a quiet NaN rather than a stop.
    error('mgs:zarchan:airframeSim:trimDegenerate', ...
        ['the body-lift coefficient y2 = 1.5*Splan*(Xcg-Xcpb)/Sref is exactly ' ...
         'zero, so the trim equation is linear, not quadratic, and the book''s ' ...
         'quadratic-formula root is 0/0.  This happens only if the c.g. sits ' ...
         'exactly on the body centre of pressure.']);
end
disc = Y1*Y1 - 4.*Y2*Y3;
if disc < 0
    error('mgs:zarchan:airframeSim:trimNoRoot', ...
        ['no real trim angle of attack for a %.4g-deg fin: the discriminant ' ...
         'y1^2 - 4*y2*y3 is %.4g.  The fin is asking for more moment than ' ...
         'the airframe can trim; an imaginary alpha would propagate silently ' ...
         'into every linearized derivative.'], DEL*DEG, disc);
end
root_small = (-Y1 - sqrt(disc))/(2*Y2);
root_other = (-Y1 + sqrt(disc))/(2*Y2);

if ~(abs(root_small) < abs(root_other))
    % The book's phrase is "after eliminating the unrealistic root".  Which
    % of the two signs is the physical one depends on the SIGNS of y1, y2 and
    % y3, and those come straight out of the geometry transcription.  If a
    % sign got damaged, this formula quietly returns the OTHER root -- a
    % perfectly finite angle of attack of a few hundred degrees, which then
    % produces perfectly finite (and completely wrong) CNA and CMA.  This is
    % the cheapest possible test of the whole sign structure.
    error('mgs:zarchan:airframeSim:trimRoot', ...
        ['the root the book''s formula selects (%.4g rad) is not the one ' ...
         'nearer zero (%.4g rad).  In this airframe the physical trim is ' ...
         'always the small root; picking the large one means a sign in y1, ' ...
         'y2 or y3 -- i.e. in the transcribed geometry -- is wrong.'], ...
        root_small, root_other);
end

TR = struct();
TR.del        = DEL;
TR.del_deg    = DEL*DEG;
TR.y1 = Y1;  TR.y2 = Y2;  TR.y3 = Y3;  TR.disc = disc;
TR.alftr      = root_small;
TR.alftr_deg  = root_small*DEG;
TR.root_other = root_other;
% dCM/dalpha AT THE TRIM POINT.  This is the true JACOBIAN slope and it is
% NOT Zarchan's CMA -- see LINEARAERO.  For a quadratic with root aTR the
% slope at the root is +/- sqrt(disc), which is asserted below.
TR.dcm_dalf   = Y1 + 2*Y2*root_small;
% CM residual at the chosen root, kept so the structural check can show that
% the quadratic really was solved rather than approximated.
TR.cm_resid   = Y1*root_small + Y2*root_small^2 + Y3;
end


% =============================================================================
function L = linearAero(A, TR, cfg, GRAV_FT)
%LINEARAERO  The linearization, from Ch.21 pp.482-486 and the preamble of
%   Listing 21.2 (zarchan.txt:6751-6755).  NOT part of Listing 21.1.
%
%   It is here because Listing 21.1 alone cannot be checked against most of
%   what Ch.21 publishes: K1, K3, Ta, wAF and zAF are all linearized
%   quantities and all of them appear as numbers in the running text.
%
%   *** ZARCHAN'S CMA IS A SECANT, NOT A DERIVATIVE.  THIS MATTERS. ***
%
%   His recipe (p.483) is "simply divide the angle of attack terms in CM by
%   alpha".  Applied to the quadratic term 1.5*Splan*alpha^2/Sref that gives
%   1.5*Splan*alpha/Sref -- a SECANT through the origin, not the tangent
%   3*Splan*alpha/Sref.  Same for CN.  So at the trim point
%
%       CMA        = y1 +   y2*aTR      (Zarchan)      = -76.96 at sea level
%       dCM/dalpha = y1 + 2*y2*aTR      (Jacobian)     = -75.56 at sea level
%
%   These are 1.9% apart in moment slope and 10% apart in damping ratio.
%   Zarchan's choice is the RIGHT one for his purpose -- it reproduces the
%   trim acceleration EXACTLY, which is what the transfer-function gain K1 is
%   for -- but the frequency and damping the NONLINEAR simulation actually
%   exhibits about trim are the JACOBIAN's, because small oscillations obey
%   the tangent.  Both are computed here.  Benchmarks against the book use
%   Zarchan's; the structural check against the measured response uses the
%   Jacobian's.  Conflating them costs 9% of a damping ratio and looks like
%   an integrator bug.

VM = cfg.vm;  WGT = cfg.wgt;
SREF=A.sref; SPLAN=A.splan; SWING=A.swing; STAIL=A.stail; B=A.b; Q=A.q;
DIAM=A.diam; XIYY=A.xiyy; TMP1=A.tmp1; TMP2=A.tmp2; TMP3=A.tmp3; TMP4=A.tmp4;
ALFTR = TR.alftr;

% -- Zarchan's coefficients, evaluated at the trim angle of attack ----------
CNA=2+1.5*SPLAN*ALFTR/SREF+8*SWING/(B*SREF)+8*STAIL/(B*SREF);
CND=8*STAIL/(B*SREF);
CMAP=2*TMP4+1.5*SPLAN*ALFTR*TMP3/SREF+8*SWING*TMP1/(B*SREF);
CMA=CMAP+8*STAIL*TMP2/(B*SREF);
CMD=8*STAIL*TMP2/(B*SREF);

XMA=Q*SREF*DIAM*CMA/XIYY;  XMD=Q*SREF*DIAM*CMD/XIYY;
ZA=-GRAV_FT*Q*SREF*CNA/(WGT*VM);  ZD=-GRAV_FT*Q*SREF*CND/(WGT*VM);

if XMA >= 0
    error('mgs:zarchan:airframeSim:unstable', ...
        ['Ma = %g >= 0: this airframe is statically UNSTABLE at this flight ' ...
         'condition and wAF = sqrt(-Ma) is imaginary.  Ch.21''s transfer ' ...
         'functions are derived for a stable airframe; there is no unstable ' ...
         'branch, and the nonlinear run would diverge rather than oscillate.'], XMA);
end
% The numerator Ma*Zd - Md*Za is the ONE quantity K1, K3, Ta and wZ are all
% built on.  Zero it and wZ is 0, K1 is 0, and Ta is Inf -- three different
% flavours of nonsense from one cause, none of which announces itself.
if ZD == 0 || (XMA*ZD-ZA*XMD) == 0 || (XMA*ZD-ZA*XMD)/ZD < 0
    error('mgs:zarchan:airframeSim:noZero', ...
        ['(Ma*Zd - Za*Md) = %g and wZ^2 = %g.  The right-half-plane airframe ' ...
         'zero is what produces the wrong-way tail effect; it must be real ' ...
         'and nonzero, because K1, K3 and Ta all divide by or vanish with ' ...
         'this same numerator, and a complex or zero wZ would still print a ' ...
         'plausible magnitude.'], (XMA*ZD-ZA*XMD), ...
        (XMA*ZD-ZA*XMD)/(ZD+(ZD==0)*eps));
end

WZ=sqrt((XMA*ZD-ZA*XMD)/ZD);  WAF=sqrt(-XMA);  ZAF=.5*WAF*ZA/XMA;
% 1845 is Zarchan's rad-to-deg-and-g bookkeeping constant, a ROUNDED
% 57.3*32.2 = 1844.66, so K1 comes out in g/deg and K3 in deg/deg-of-fin per
% second.  It is not a tuning number, and its rounding is why the identity
% nL_trim == K1*delta holds to 1.8e-4 and not to 1e-12.
XK1=-VM*(XMA*ZD-XMD*ZA)/(1845*XMA);
TA=XMD/(XMA*ZD-XMD*ZA);
XK3=1845*XK1/VM;

% -- the JACOBIAN twins: what the nonlinear sim actually oscillates at ------
% d/dalpha of 1.5*Splan*alpha^2/Sref is 3*Splan*alpha/Sref, hence the 3.
CN_ALF = 2 + 3*SPLAN*ALFTR/SREF + 8*SWING/(B*SREF) + 8*STAIL/(B*SREF);
CM_ALF = 2*TMP4 + 3*SPLAN*ALFTR*TMP3/SREF + 8*SWING*TMP1/(B*SREF) ...
         + 8*STAIL*TMP2/(B*SREF);
XMA_J  = Q*SREF*DIAM*CM_ALF/XIYY;
ZA_J   = -GRAV_FT*Q*SREF*CN_ALF/(WGT*VM);
if XMA_J >= 0
    error('mgs:zarchan:airframeSim:unstableJac', ...
        ['the JACOBIAN moment slope is %g >= 0 at trim: the airframe is ' ...
         'locally unstable about its own trim point even though Zarchan''s ' ...
         'secant Ma = %g says otherwise.  The nonlinear run would diverge.'], ...
        XMA_J, XMA);
end

L = struct('cna',CNA,'cnd',CND,'cma',CMA,'cmd',CMD,'cmap',CMAP, ...
           'xma',XMA,'xmd',XMD,'za',ZA,'zd',ZD, ...
           'wz',WZ,'waf',WAF,'zaf',ZAF,'xk1',XK1,'xk3',XK3,'ta',TA, ...
           'cn_alf',CN_ALF,'cm_alf',CM_ALF,'xma_jac',XMA_J,'za_jac',ZA_J, ...
           'waf_jac',sqrt(-XMA_J),'zaf_jac',.5*sqrt(-XMA_J)*ZA_J/XMA_J, ...
           'tau_airframe',1/WAF);

% The steady-state acceleration and body rate the LINEAR model predicts, in
% the book's units.  These are the quantities lines 6853/6854 quote.
L.nl_ss_g   = XK1 * TR.del_deg;      % g
L.thd_ss_dps= XK3 * TR.del_deg;      % deg/s
end


% =============================================================================
function S = simulate(A, DEL, cfg, GRAV_FT, DEG)
%SIMULATE  Listing 21.1 itself.  Book variable names kept.
%
%   Zarchan's two-pass predictor/corrector (Heun), structure verbatim: pass
%   one takes derivatives at the current state, pass two Euler-advances the
%   state AND the clock and takes derivatives there, and the corrector
%   averages.  Do not replace it with ode45: the published numbers are the
%   numbers this scheme at this step size produces.
%
%   THREE DETAILS OF THE BOOK'S BOOKKEEPING, ALL KEPT:
%
%   * T IS ADVANCED INSIDE the inner loop and is NOT rolled back.  Nothing
%     here depends on t explicitly, so it only affects the recorded time tag.
%   * The sampler records XNL from the SECOND PASS -- i.e. evaluated at the
%     Euler-PREDICTED state -- but ALF and THD AFTER the corrector has run.
%     So the acceleration column is one predictor-error behind the angle
%     column.  That error is O(h^2)*alpha_ddot and vanishes at steady state,
%     which is where every benchmark reads it, but it is real and it is the
%     book's, so it stays.  "Tidying" it would silently change the numbers.
%   * `FLAG=0;` after the inner loop is a dead store (the outer loop sets
%     FLAG=0 again immediately).  Transcribed anyway.
%
%   WE ADD ONE ARRAY THE BOOK DOES NOT KEEP.  Listing 21.1 stores T, XNL/32.2
%   and ALF*57.3.  THD is a state the loop already integrates; we store it
%   too, because Ch.21 line 6854 publishes its steady-state value ("a 5-deg
%   fin deflection would result in -8 deg/s at sea level") and there is no
%   other way to score that sentence.  Storing a variable the loop already
%   computes changes nothing about the simulation.

SREF=A.sref; SPLAN=A.splan; SWING=A.swing; STAIL=A.stail; B=A.b; Q=A.q;
DIAM=A.diam; XIYY=A.xiyy; TMP1=A.tmp1; TMP2=A.tmp2; TMP3=A.tmp3; TMP4=A.tmp4;
VM = cfg.vm;  WGT = cfg.wgt;  H = cfg.h;  TSTOP = cfg.tstop;

nMax = ceil(TSTOP/H) + 4;
tA = zeros(1,nMax); gA = zeros(1,nMax); aA = zeros(1,nMax); qA = zeros(1,nMax);

THD=0; ALF=0; T=0; SACC=0.; n=0;
% Initialised only so the Code Analyzer cannot claim a use-before-definition.
% At runtime the first read of either is always after the first assignment,
% because FLAG is 0 on the first inner pass and the block that reads them is
% guarded by FLAG==1.  Do not "simplify" the FLAG machinery away: it is what
% makes pass one evaluate at the current state.
THDD=0; ALFD=0; XNL=0; CN=0; CM=0;

while T < (TSTOP - 1e-5)                      % while T,1.99999
    THDOLD=THD; ALFOLD=ALF; STEP=1; FLAG=0;
    while STEP <= 1
        if FLAG==1
            STEP=2; THD=THD+H*THDD; ALF=ALF+H*ALFD; T=T+H;
        end
        CN=2*ALF+1.5*SPLAN*ALF*ALF/SREF+8*SWING*ALF/(B*SREF)+ ...
           8*STAIL*(ALF+DEL)/(B*SREF);
        CM=2*ALF*TMP4+1.5*SPLAN*ALF*ALF*TMP3/SREF+ ...
           8*SWING*ALF*TMP1/(B*SREF)+8*STAIL*(ALF+DEL)*TMP2/(B*SREF);
        THDD=Q*SREF*DIAM*CM/XIYY;
        XNL=GRAV_FT*Q*SREF*CN/WGT;
        ALFD=THD-XNL/VM;
        FLAG=1;
    end
    % The book's dead store, transcribed: the outer loop clears FLAG again
    % on the next pass.  Kept because the transcription is the artifact.
    FLAG=0; %#ok<NASGU>
    THD=.5*(THDOLD+THD+H*THDD);
    ALF=.5*(ALFOLD+ALF+H*ALFD);
    SACC=SACC+H;
    if SACC >= .0099999
        SACC=0.; n=n+1;
        tA(n)=T;
        gA(n)=XNL/GRAV_FT;      % ArrayXNLG   -- g
        aA(n)=ALF*DEG;          % ArrayALFDEG -- deg
        qA(n)=THD*DEG;          % ours, not the book's -- deg/s
    end
    if ~isfinite(ALF) || ~isfinite(THD)
        error('mgs:zarchan:airframeSim:diverged', ...
            ['the integration went non-finite at t = %g s.  With h = %g s ' ...
             'and a %.4g-deg fin this airframe/step-size pair does not ' ...
             'integrate; returning the array up to the blow-up would hand ' ...
             'back a plausible-looking transient.'], T, H, DEL*DEG);
    end
end

if n < 3
    error('mgs:zarchan:airframeSim:noSamples', ...
        ['the run produced %d samples (tstop = %g s, h = %g s).  Nothing ' ...
         'downstream -- settled value, damping, frequency -- can be measured ' ...
         'from that, and an all-NaN feature set reads as "not applicable" ' ...
         'rather than as "the run was too short".'], n, TSTOP, H);
end

S = struct();
S.t            = tA(1:n);
S.accel_g      = gA(1:n);
S.alpha_deg    = aA(1:n);
S.thetadot_dps = qA(1:n);
S.n            = n;
S.h            = H;
S.del_deg      = DEL*DEG;

amax = max(abs(S.alpha_deg));
S.alpha_max_deg = amax;
if amax > 30
    % Line 6423: the transfer-function representation is "an excellent
    % approximation to the nonlinear equations for angles of attack less than
    % 20 to 30 deg".  Above that the 1.5*Splan*alpha^2 term is no longer a
    % small correction and the linear-theory wing/tail terms are outside
    % their own validity too -- but the equations still evaluate and still
    % return numbers, which is exactly why this has to be said out loud.
    warning('mgs:zarchan:airframeSim:largeAlpha', ...
        ['peak |alpha| = %.3g deg exceeds the 20-30 deg range the book ' ...
         'states this aero model is good for (zarchan.txt:6423).  The ' ...
         'integration is fine; the AERODYNAMICS is being extrapolated.'], amax);
end
if amax > 90
    error('mgs:zarchan:airframeSim:alphaAbsurd', ...
        ['peak |alpha| reached %.4g deg.  Linear-theory normal force with a ' ...
         'single alpha^2 term is meaningless past 90 deg; the numbers this ' ...
         'run produced are arithmetic, not aerodynamics.'], amax);
end
end


% =============================================================================
function P = responseFeatures(S)
%RESPONSEFEATURES  Settled value, peak, damped frequency and damping ratio,
%   measured from the sampled time histories.
%
%   WHY NOT JUST TAKE THE LAST SAMPLE AS THE STEADY STATE.  Because it is
%   not one.  At sea level the airframe damping is ~0.046, so after the
%   book's 2 s the response is still ringing at ~8% of its initial excursion
%   -- about +/-0.9 g on a 13-g answer.  A "steady-state" read off the last
%   sample would be a 7% coin-flip on the phase, and the 13-g benchmark would
%   pass or fail depending on where the clock stopped.  At 50 kft it is far
%   worse: 2 s is only three cycles of a 9.6 rad/s mode with zeta ~0.018 and
%   the envelope has barely decayed at all.
%
%   THE FIRST ESTIMATOR, AND THE 8% IT COST US.  It is kept below, as
%   .settled_kernel, because the way it failed is the whole lesson.  For
%   y = c + A*exp(-s*t)*cos(w*t+phi) the extrema alternate about c with ratio
%   r = exp(-s*T/2) per half period.  Writing the last three refined extrema
%   as e1, e2, e3:
%
%       midpoint(e1,e2) - c = +(A/2)(1-r)      [one half-period bias]
%       midpoint(e2,e3) - c = -(A r/2)(1-r)
%
%   so averaging the two midpoints -- the kernel (1/4, 1/2, 1/4) over
%   e1,e2,e3 -- cancels the first-order bias and leaves (A/4)(1-r)^2.  Every
%   line of that is correct, and it is correct about the wrong signal.
%
%   *** THE CENTRE OF THE RING IS NOT A CONSTANT.  THAT IS THE BUG. ***
%
%   This airframe is QUADRATIC in alpha: the 1.5*Splan*alpha^2/Sref term sits
%   in BOTH CN and CM.  Push a ring alpha = a_c + a*exp(-s*t)*cos(w*t) through
%   a quadratic and order a^2 hands back two things that the model above does
%   not contain -- a RECTIFIED DC term (a^2/2)*exp(-2*s*t), because the mean of
%   a squared cosine is not zero, and a second harmonic at 2*w carrying the
%   same exp(-2*s*t) envelope.  Both decay at 2*s, i.e. in HALF the ring's own
%   time constant, so they are gone long before the run ends -- and both are
%   still large at t = 2 s.  Worse, the kernel is blind to them by
%   construction: they are EVEN about the ring, so they enter midpoint(e1,e2)
%   and midpoint(e2,e3) with the SAME sign and survive the average that was
%   built to cancel opposite signs.  Measured at 50 kft, acceleration channel:
%   mid1 - c = +0.2101, mid2 - c = +0.0916, average +0.1508 -- exactly the
%   error, and no sign change anywhere in it.
%
%   WHY IT SHOWED UP ON TWO CHANNELS AND NOT ON THE THIRD.  The rectified DC
%   is roughly the same ABSOLUTE size in each channel's own units, so what
%   decides whether it matters is the ring-to-settled ratio.  At the last
%   extremum of the book's 2 s that ratio is 0.115 (alpha, sea level), 0.719
%   (alpha, 50 kft), 0.144 (accel, sea level), 0.983 (accel, 50 kft), 1.48
%   (thetadot, sea level) and 26.7 (thetadot, 50 kft).  Hence the pattern the
%   board showed: alpha fine everywhere, acceleration fine at sea level and 8%
%   out at altitude, body rate 1% out at SEA LEVEL -- which is where the
%   thetadot_ss == nL_ss*g/VM row was actually failing, not at 50 kft.
%
%   AND IT IS NOT "2 s IS TOO SHORT".  That was the obvious suspect and it is
%   wrong.  Fitting the IDEAL model (constant + one decaying sinusoid) to the
%   same 2 s by least squares over all 200 samples still lands 4.3% and 5.9%
%   out at 50 kft -- no better than the kernel.  The record is not the limit;
%   the model of the record was.  Add the two rectification columns and the
%   same 2 s gives the settled value to 1e-4.  Truth here is a 400-s run of
%   the same integrator, which reproduces the closed forms to 1e-7 (alpha),
%   3e-5 (nL) and machine precision (thetadot, Ta) -- so the closed forms were
%   never the suspect either.
%
%   THE ESTIMATOR NOW USED.  Least squares of each channel onto the basis the
%   quadratic airframe actually generates,
%
%       [ 1,  e^-st cos wt,  e^-st sin wt,
%             e^-2st,        e^-2st cos 2wt,  e^-2st sin 2wt ]
%
%   and the SETTLED VALUE IS THE CONSTANT COLUMN'S COEFFICIENT.  s and w are
%   seeded from the log decrement below and then refined inside the fit --
%   which is not a free parameter but a strictly better measurement of the
%   same two numbers, from 200 samples instead of 3 extrema.  It has to be
%   refined: the log decrement reads one period of an oscillator whose
%   frequency is amplitude dependent, and on a channel whose ring is 27x its
%   settled value a 1e-3 error in w lands on the constant magnified 27-fold.
%   Seeded-only, thetadot is 1.2e-2 out at 40 kft; refined, 9.7e-4 across
%   0-60 kft.  The refined s and w are NOT fed back into .wn/.zeta -- those
%   rows compare the LOG DECREMENT against the Jacobian and must keep
%   measuring what they say they measure.
%
%   Extrema are refined by fitting a parabola through the sample and its two
%   neighbours, which removes the O((w*dt)^2) sampling error in the extremum
%   VALUE (0.8% of the local amplitude at sea level on the 0.01-s grid).  They
%   still set the peak, the extremum count and the log decrement; only the
%   settled value moved off them.
%
%   EVERYTHING HERE CAN COME BACK NaN, AND SAYS WHY.  A record with fewer
%   than three extrema has no settled value that this method can defend, and
%   the benchmark rows that need one SKIP with the reason attached instead of
%   falling back to the last sample.  If the ring fit cannot be built the
%   kernel value is kept and .settled_model says 'kernel' -- degraded, but
%   never silently.

P = struct();
P.accel    = featureSet(S.t, S.accel_g);
P.alpha    = featureSet(S.t, S.alpha_deg);
P.thetadot = featureSet(S.t, S.thetadot_dps);

% ONE pole pair for three channels, and it is alpha's.  All three signals are
% outputs of the same second-order airframe, so the ring basis is the same
% basis; alpha is where it is measured because its own transfer function has
% no zero near the pole pair (see .wn/.zeta below), and a log decrement read
% on the acceleration channel is biased by the right-half-plane zero.  If
% alpha's decrement failed there is no basis to build and all three channels
% fall back to the kernel -- together, so the board cannot show one refined
% channel being compared against one un-refined one.
if P.alpha.ok
    s_ring = P.alpha.zeta * P.alpha.wn;
    w_ring = P.alpha.wd;
else
    s_ring = NaN;  w_ring = NaN;
end
P.ring_s_seed = s_ring;
P.ring_w_seed = w_ring;
P.alpha    = refineSettled(P.alpha,    S.t, S.alpha_deg,    s_ring, w_ring);
P.accel    = refineSettled(P.accel,    S.t, S.accel_g,      s_ring, w_ring);
P.thetadot = refineSettled(P.thetadot, S.t, S.thetadot_dps, s_ring, w_ring);
P.settled_model = P.alpha.settled_model;

% Convenience mirrors, so callers and the renderer do not have to remember
% which sub-struct a number lives in.
P.settled_accel_g   = P.accel.settled;
P.settled_alpha_deg = P.alpha.settled;
P.settled_thd_dps   = P.thetadot.settled;
% The superseded kernel values, kept so a future regression can be told apart
% from a genuine change in the run.  These are what the board printed while
% the two structural rows were red; do not delete them to tidy up.
P.kernel_accel_g    = P.accel.settled_kernel;
P.kernel_alpha_deg  = P.alpha.settled_kernel;
P.kernel_thd_dps    = P.thetadot.settled_kernel;
P.peak_alpha_deg    = P.alpha.peak;
P.peak_alpha_t      = P.alpha.peak_t;
P.final_accel_g     = P.accel.final;
P.final_alpha_deg   = P.alpha.final;

% THE TURNING-RATE TIME CONSTANT, MEASURED.  Zarchan defines Ta as "the
% amount of time it takes to turn the missile flight path angle g through an
% equivalent angle of attack a" (zarchan.txt:5418), and writes it as
% Ta = alpha/gammadot (:1839).  At steady state alphadot = 0, so gammadot =
% thetadot and Ta is just the ratio of the two settled channels -- and the
% DEGREES CANCEL, so no conversion constant enters.  This is Ta straight off
% the nonlinear run with no transfer function in between; the structural
% check compares it against the Md/(Ma*Zd - Md*Za) closed form, which it
% reproduces exactly on paper.
P.ta_meas_s = P.settled_alpha_deg / P.settled_thd_dps;

% Frequency and damping are measured on ALPHA.  The acceleration channel
% carries the right-half-plane zero, which distorts the early transient and
% biases a log-decrement read on it; alpha's transfer function has its only
% zero at -Md/Zd = -853 rad/s at sea level, 34x beyond the pole pair, so its
% decay is clean second order.
P.wn        = P.alpha.wn;
P.wd        = P.alpha.wd;
P.zeta      = P.alpha.zeta;
P.n_extrema = P.alpha.n_extrema;
P.meas_ok   = P.alpha.ok;
P.meas_why  = P.alpha.why;

% -- the wrong-way tail effect (line 6835) ---------------------------------
% A tail-controlled missile's first response to a positive fin deflection is
% a force in the WRONG direction: the tail lift acts before the body has
% developed any angle of attack.  That is the right-half-plane zero wZ.  It
% is visible in the acceleration channel as an initial excursion whose sign
% is opposite to the steady state.
g0 = S.accel_g(1);
gs = P.settled_accel_g;
P.wrongway_present = isfinite(gs) && gs ~= 0 && sign(g0) == -sign(gs);
P.wrongway_first_g = g0;
if P.wrongway_present
    % How long the acceleration stays on the wrong side of zero, and how far
    % it gets, relative to the steady state it is heading for.
    k = find(sign(S.accel_g) ~= sign(g0), 1, 'first');
    if isempty(k)
        P.wrongway_dur_s = NaN;
    else
        P.wrongway_dur_s = S.t(k);
    end
    P.wrongway_ratio = max(abs(S.accel_g(sign(S.accel_g) == sign(g0)))) / abs(gs);
else
    P.wrongway_dur_s = NaN;
    P.wrongway_ratio = NaN;
end
end


% -----------------------------------------------------------------------------
function Q = featureSet(t, y)
%FEATURESET  Extrema, settled value, peak, wn and zeta for one channel.
Q = struct('n_extrema',0,'t_ext',[],'y_ext',[],'settled',NaN,'final',y(end), ...
           'peak',NaN,'peak_t',NaN,'wd',NaN,'wn',NaN,'zeta',NaN, ...
           'ok',false,'why','');

if numel(y) < 5
    Q.why = sprintf('only %d samples', numel(y));
    Q.peak = y(end);
    return;
end

dt = diff(t);
uniform = max(abs(dt - dt(1))) <= 1e-9 * max(abs(dt(1)), eps);

dy = diff(y);
% Interior extremum: the slope changes sign.  Strict < 0 misses an exact
% plateau, which cannot happen in this signal at double precision and which
% would otherwise be reported twice.
ix = find(dy(1:end-1).*dy(2:end) < 0) + 1;

te = zeros(1, numel(ix));  ye = zeros(1, numel(ix));
for k = 1:numel(ix)
    i = ix(k);
    ym1 = y(i-1);  y0 = y(i);  yp1 = y(i+1);
    den = ym1 - 2*y0 + yp1;
    if den == 0 || ~uniform
        te(k) = t(i);  ye(k) = y0;
    else
        d = 0.5*(ym1 - yp1)/den;      % vertex offset in samples
        if abs(d) > 1
            % A vertex outside the bracketing triple means the three points
            % are not describing a single smooth extremum.  Refusing the
            % refinement is safe; accepting it would move the extremum to a
            % time no sample supports.
            te(k) = t(i);  ye(k) = y0;
        else
            te(k) = t(i) + d*(t(i+1)-t(i));
            ye(k) = y0 - 0.25*(ym1 - yp1)*d;
        end
    end
end
Q.n_extrema = numel(ix);
Q.t_ext = te;  Q.y_ext = ye;

if isempty(ye)
    [~, im] = max(abs(y));
    Q.peak = y(im);  Q.peak_t = t(im);
    Q.why = 'no interior extremum in the record';
    return;
end
[~, ip] = max(abs(ye));
Q.peak = ye(ip);  Q.peak_t = te(ip);

if numel(ye) < 3
    Q.why = sprintf(['only %d extrema in the record; the (1/4,1/2,1/4) ' ...
                     'settled estimator needs 3'], numel(ye));
    return;
end
e = ye(end-2:end);
Q.settled = 0.25*e(1) + 0.5*e(2) + 0.25*e(3);

% Log decrement over the LAST FULL PERIOD: extrema end-2 and end are on the
% same side of the settled value, one period apart.
r1 = ye(end-2) - Q.settled;
r3 = ye(end)   - Q.settled;
Tfull = te(end) - te(end-2);
scale = max(abs(ye)) + eps;
if Tfull <= 0
    Q.why = 'non-increasing extremum times';  return;
end
if r1*r3 <= 0
    Q.why = 'the two same-side extrema straddle the settled value';  return;
end
if abs(r3) < 1e-9*scale
    % The ring has died into round-off.  A log decrement computed here is a
    % ratio of two numerical noises and would print as a confident damping
    % ratio of anything at all.
    Q.why = 'final residual is at round-off; damping is unmeasurable';  return;
end
if abs(r1) <= abs(r3)
    Q.why = 'the oscillation is not decaying over the last period';  return;
end
d = log(abs(r1/r3));
Q.zeta = d/sqrt(4*pi^2 + d^2);
Q.wd   = 2*pi/Tfull;
Q.wn   = Q.wd/sqrt(1 - Q.zeta^2);
Q.ok   = true;
end


% -----------------------------------------------------------------------------
function Q = refineSettled(Q, t, y, s0, w0)
%REFINESETTLED  Replace the extremum kernel's settled value with the constant
%   term of a fit to the ring model the QUADRATIC airframe actually produces.
%   See the derivation in RESPONSEFEATURES; this function is only the
%   plumbing and the refusals.
%
%   IT NEVER ERRORS.  Every way this can fail leaves the kernel value in
%   place, sets .settled_model = 'kernel' and writes .settled_why.  A settled
%   value is an INPUT to four structural rows, and a run that dies here would
%   take the whole flight condition with it over an estimator upgrade.

Q.settled_kernel = Q.settled;   % what the (1/4,1/2,1/4) kernel said
Q.settled_model  = 'kernel';
Q.settled_why    = '';
Q.settled_s      = NaN;
Q.settled_w      = NaN;
Q.settled_res    = NaN;   % fit residual, relative to the signal's own swing
Q.settled_cond   = NaN;

if ~isfinite(Q.settled)
    % FEATURESET already declined (fewer than three extrema, no extremum at
    % all, ...).  Its reason is the right one; do not paper over it with a
    % fit, which would hand back a confident number for a record that has
    % not even completed a cycle.
    Q.settled_why = Q.why;
    return
end
if ~isfinite(s0) || ~isfinite(w0) || s0 <= 0 || w0 <= 0
    Q.settled_why = ['no measured pole pair (alpha''s log decrement failed), ' ...
                     'so the ring basis cannot be built'];
    return
end

t = t(:);  y = y(:);
nMin = 4*6;   % six basis columns; fewer than four rows each is not a fit
if numel(y) < nMin
    Q.settled_why = sprintf(['%d samples for a 6-column ring basis; below ' ...
        '%d the fit is interpolating the record rather than measuring it'], ...
        numel(y), nMin);
    return
end

[c, sB, wB, resRel, cnd] = ringFit(t - t(1), y, s0, w0);

if ~isfinite(c)
    Q.settled_why = 'the ring fit returned a non-finite constant term';
    return
end
if cnd > 1e8
    % Two columns of the basis have become indistinguishable over this
    % record -- typically exp(-2*s*t) against the constant when the record
    % is far shorter than 1/s.  The solve still returns a number and it is
    % the difference of two large cancelling coefficients, i.e. exactly the
    % kind of confident garbage this file exists to refuse.
    Q.settled_why = sprintf(['ring basis is ill-conditioned over this ' ...
        'record (cond = %.3g); the constant and the exp(-2*s*t) columns ' ...
        'are not separable here'], cnd);
    Q.settled_cond = cnd;
    return
end

Q.settled       = c;
Q.settled_model = 'ring';
Q.settled_s     = sB;
Q.settled_w     = wB;
Q.settled_res   = resRel;
Q.settled_cond  = cnd;
end


% -----------------------------------------------------------------------------
function [c, sBest, wBest, resRel, cnd] = ringFit(tt, y, s0, w0)
%RINGFIT  Least squares onto [1, e^-st{cos,sin}wt, e^-2st{1,cos,sin}2wt].
%
%   FOR FIXED (s,w) THE FIT IS LINEAR, so the inner solve is one backslash on
%   a 6-column matrix -- no toolbox, no iteration, no starting guess to get
%   wrong.  Only s and w are searched, and they are searched because the log
%   decrement that seeds them reads ONE period of an oscillator whose
%   frequency is amplitude dependent; on the body-rate channel, whose ring is
%   27x its settled value at 50 kft, that seed error is what is left over.
%
%   THE SEARCH IS A SHRINKING GRID, NOT A GOLDEN SECTION.  The residual
%   surface in (s,w) is smooth but not guaranteed unimodal -- w has side
%   minima roughly one record-length of phase away -- and a bracketing method
%   that assumes unimodality would walk into one of them and report a
%   confident wrong frequency.  A grid that keeps the best cell and shrinks
%   to +/- one step cannot do that.  7 points x 9 passes takes the initial
%   bracket down by 3^-9 and costs about 5 ms per channel, measured; the cost
%   of this whole function on the book's two flight conditions is tens of
%   milliseconds against a suite that runs in seconds.
%
%   The s bracket is wide (0.4x to 2.5x the seed) because a log decrement on
%   three extrema is a crude damping estimate.  The w bracket is +/-3%
%   because a period measured between two extrema two half-cycles apart is
%   not crude at all -- and because a w that has to move further than 3% is
%   evidence the record is not the mode we think it is, which the caller can
%   see in .settled_w against .ring_w_seed.
%
%   THE MEASURED ENVELOPE, so nobody has to rediscover it.  Sweeping the
%   book's case over altitude and horizon (truth = the same integrator run to
%   400 s):
%       alt 0..60 kft at the book's 2 s : settled values within 1.9e-4
%       tstop 1.5 / 2 / 3 s at 50 kft   : channel-tie residual 6.0e-4 /
%                                         2.7e-4 / 7.5e-4
%       tstop 1.0 s at 50 kft           : 1.06e-2 -- OUTSIDE the 5e-3 band,
%                                         and it throws, correctly
%   1.0 s at 50 kft is 1.5 periods and exactly three extrema.  Below roughly
%   two periods the six columns stop being separable by the record and the
%   answer degrades fast.  That is a real limit of a 1-s run at that damping,
%   not a defect, and the loud failure is the right outcome.
%
%   ⚠️ AND DO NOT BUILD A GUARD OUT OF THE FIT RESIDUAL.  It was the obvious
%   candidate and it is worthless here: at tstop = 1 s the fit residual is
%   1.9e-05 -- BETTER than the 5.6e-05 of the 2-s run that is two orders more
%   accurate -- because a short record is easier to fit, not better
%   determined.  A "confidence" derived from it would be highest exactly
%   where the answer is worst.

np    = 7;
npass = 9;
sLo = 0.40*s0;  sHi = 2.50*s0;
wLo = 0.97*w0;  wHi = 1.03*w0;

best  = Inf;
c     = NaN;
sBest = s0;
wBest = w0;

for p = 1:npass
    ss = linspace(sLo, sHi, np);
    ww = linspace(wLo, wHi, np);
    for a = 1:np
        for b = 1:np
            B = ringBasis(tt, ss(a), ww(b));
            q = B \ y;
            rr = norm(B*q - y);
            % Strict <, so a tie keeps the earlier (lower-frequency) cell and
            % the search is reproducible run to run.
            if rr < best
                best  = rr;
                sBest = ss(a);
                wBest = ww(b);
                c     = q(1);
            end
        end
    end
    ds = (sHi - sLo)/(np - 1);
    dw = (wHi - wLo)/(np - 1);
    % Re-centre on the winner and shrink to +/- one old step.  The bracket is
    % allowed to walk outside its initial range -- that is a local descent,
    % and .settled_s / .settled_w record where it ended up.
    sLo = max(sBest - ds, eps);  sHi = sBest + ds;
    wLo = max(wBest - dw, eps);  wHi = wBest + dw;
end

B      = ringBasis(tt, sBest, wBest);
cnd    = cond(B);
% Residual relative to the signal's own swing about the fitted centre, NOT to
% |c|: on the body-rate channel |c| is 4% of the swing, so normalising by it
% would report a 1e-3 fit as a 3% one.
resRel = best / max(norm(y - c), eps);
end


% -----------------------------------------------------------------------------
function B = ringBasis(tt, s, w)
%RINGBASIS  The six columns a quadratic nonlinearity generates from one ring.
%   Columns 4-6 are the order-a^2 terms: the rectified DC and the second
%   harmonic, both carrying exp(-2*s*t).  Dropping them is what cost 8%.
%
%   THE DELETE EXPERIMENT, run 2026-08-23, because "the comment says these
%   columns matter" is not evidence.  Refitting the same records with columns
%   1-3 only and recomputing the structural residuals:
%
%              columns 1-3 (deleted)      columns 1-6 (as shipped)
%       alpha       6.69e-03                    8.61e-05
%       sim         4.77e-02  FAIL              2.45e-04
%       ta          5.72e-02  FAIL              3.31e-04
%       chan        2.83e-03  pass              2.67e-04
%
%   Two orders of magnitude on every row, and the two rows that were red on
%   the board come back red.  ⚠️ NOTE WHICH ROW DOES NOT: `chan` passes even
%   with the columns deleted, because it is a DIFFERENCE of two channel
%   biases and at these two altitudes they largely cancel (nL -1.27e-2 /
%   -4.51e-2 against thetadot -1.41e-2 / -4.77e-2).  That is the same
%   cancellation documented at check 5, seen from the other side.
e1 = exp(-s*tt);
e2 = e1 .* e1;
B  = [ones(numel(tt),1), e1.*cos(w*tt), e1.*sin(w*tt), ...
      e2, e2.*cos(2*w*tt), e2.*sin(2*w*tt)];
end


% =============================================================================
function L = lagSummary(FC)
%LAGSUMMARY  The reason this file exists, one column per flight condition.
%
%   TWO DIFFERENT TIME CONSTANTS COME OUT OF THIS CHAPTER AND THEY ARE NOT
%   INTERCHANGEABLE.  Quoting the wrong one at our own 8 s would make the
%   comparison meaningless in opposite directions.
%
%     tau_airframe = 1/wAF   the BODY's pitch response.  0.040 s at sea
%           level, 0.104 s at 50 kft.  Line 6794: "in general the airframe
%           time constant is fast, and an autopilot is usually not required
%           to artificially speed up the airframe response."  Two hundred
%           times faster than our measured lag.  Our 8 s IS NOT THIS.
%
%     Ta = Md/(Ma*Zd - Md*Za)   the TURNING RATE time constant.  0.56 s at
%           sea level, 3.78 s at 50 kft (line 6810: "approximately 4 s").
%           Zarchan's own definition (zarchan.txt:5418): "the amount of time
%           it takes to turn the missile flight path angle g through an
%           equivalent angle of attack a", written Ta = alpha/gammadot at
%           :1839.  That is a FLIGHT-PATH-ANGLE lag -- the same quantity our
%           0.063 gamma-tracking gain is measured on -- and the nonlinear run
%           measures it directly as the ratio of two settled channels, which
%           is a structural check here rather than an interpretation.  Same
%           order as our 8 s.  It scales as 1/Q, so altitude makes it worse.
%
%   WHAT THIS DOES AND DOES NOT LICENCE US TO SAY.  It does licence: "an
%   aerodynamic vehicle's flight-path lag is a derivable quantity of order
%   seconds that grows as dynamic pressure falls."  It does NOT licence any
%   number for OUR vehicle: this Ta is Mach 3, 1000 lb, cruciform, tail
%   controlled, with linear-theory supersonic aero.  Ours is subsonic, and
%   this aero model has no subsonic branch -- AEROCONSTANTS errors out below
%   Mach 1 rather than returning the complex numbers it would otherwise
%   produce.  The transferable result is the STRUCTURE, not the value.
L = struct();
L.alt            = [FC.alt];
L.tau_airframe_s = arrayfun(@(f) f.lin.tau_airframe, FC);
L.waf            = arrayfun(@(f) f.lin.waf,          FC);
L.ta_turnrate_s  = arrayfun(@(f) f.lin.ta,           FC);
L.zaf            = arrayfun(@(f) f.lin.zaf,          FC);
L.our_measured_s = 8.0;    % our fitted gamma-tracking lag, for scale only
L.note = ['airframe lag is 2 orders below our measured 8 s; the turning-rate ' ...
          'lag Ta is the same order.  Ta is the comparable quantity.'];
end


% =============================================================================
function C = structuralCheck(FC, cfg, GRAV_FT, DEG)
%STRUCTURALCHECK  Invariants that must hold or every number above is garbage.
%
%   These are NOT benchmarks.  They test the WIRING against algebra and
%   against each other, using properties this model has for structural
%   reasons, and they keep working when the caller runs off-book parameters.
%
%   1. THE QUADRATIC WAS SOLVED.  CM(aTR) = 0 to round-off, and the slope at
%      the root is +/-sqrt(disc) -- a property of any quadratic at its root,
%      so it tests the Jacobian expression against the trim expression.
%   2. THE JACOBIAN AND THE TRIM POLYNOMIAL AGREE.  dCM/dalpha built term by
%      term from the geometry must equal y1 + 2*y2*aTR exactly.  This is the
%      only check that can see a mistyped 3 (or a left-behind 1.5) in the
%      derivative.
%   3. THE NONLINEAR RUN REACHES THE ALGEBRAIC TRIM.  The settled alpha
%      measured from the integrated time history must equal the root of the
%      quadratic.  Two completely different computations -- an ODE and a
%      quadratic formula -- landing on the same number.  If the integrator or
%      the force/moment transcription is damaged, this is what catches it.
%   4. nL_ss == K1*delta AND thetadot_ss == K3*delta.  These are ALGEBRAIC
%      IDENTITIES, not approximations: substituting aTR = -CMD*delta/CMA into
%      (g*Q*Sref/W)*(CNA*aTR + CND*delta) reproduces K1*delta exactly, EXCEPT
%      for Zarchan's 1845 vs 57.3*32.2 = 1844.66.  The tolerance is therefore
%      2e-4, not 1e-12, and the residual is REPORTED so the rounding stays
%      visible instead of being absorbed into a loose band.
%   5. thetadot_ss == nL_ss*g/VM.  Straight out of alphadot = thetadot -
%      nL/VM at alphadot = 0.  Ties the two output channels together.
%   6. DYNAMIC-PRESSURE SCALING.  At fixed VM and fin, the trim angle is
%      EXACTLY altitude independent (y1,y2,y3 contain no rho), and every
%      derived quantity is a fixed power of Q: Ma,Md,Za,Zd,K1,K3 ~ Q;
%      wAF,zAF,wZ ~ sqrt(Q); Ta ~ 1/Q.  One residual exercises the entire
%      aero chain including the atmosphere branch.  Needs >= 2 altitudes.
%   7. MEASURED wn AND zeta MATCH THE JACOBIAN LINEARIZATION -- and NOT
%      Zarchan's secant.  This is the row that proves the secant/tangent
%      distinction in LINEARAERO is real rather than a story: the measured
%      damping sits within a per cent of the Jacobian value and ~10% away from
%      Zarchan's.  Both numbers are printed.

C = struct();
C.skipped = false;
nA = numel(FC);

% -- 1 & 2: the trim algebra
C.trim_resid = max(arrayfun(@(f) abs(f.trim.cm_resid) / ...
                    max(abs(f.trim.y3), eps), FC));
C.trim_tol   = 1e-12;
C.trim_ok    = C.trim_resid <= C.trim_tol;

C.slope_resid = max(arrayfun(@(f) ...
    abs(abs(f.trim.dcm_dalf) - sqrt(f.trim.disc)) / sqrt(f.trim.disc), FC));
C.slope_tol   = 1e-12;
C.slope_ok    = C.slope_resid <= C.slope_tol;

C.jac_resid = max(arrayfun(@(f) ...
    abs(f.lin.cm_alf - f.trim.dcm_dalf) / max(abs(f.trim.dcm_dalf), eps), FC));
C.jac_tol   = 1e-12;
C.jac_ok    = C.jac_resid <= C.jac_tol;

% -- 3: the ODE lands on the quadratic's root
%    THE TOLERANCE IS 1.5%.  It was set when the settled value came off the
%    (1/4,1/2,1/4) extremum kernel, whose residual at 50 kft was the quadratic
%    DC offset -beta*a^2/(2*w^2) (see FLIGHTCONDITION) at ~5e-3 -- so a 2e-3
%    band would have failed a correct port at altitude.  Since 2026-08-23 the
%    estimator fits that offset instead of ignoring it and the residual is
%    8.6e-5, two orders inside the band.
%    ⚠️ THE BAND IS LEFT AT 1.5% ON PURPOSE.  Tightening it to 3e-4 "because
%    that is what we measure now" would be fitting the test to today's
%    estimator, and the next reader would have no way to tell a real
%    regression from an estimator they are allowed to change.  What polices
%    the estimator is that .alpha_dc_pred is still computed and still printed
%    beside the residual: prediction 5e-3, residual 8.6e-5 means the offset
%    was modelled, not that it was never there.
C.alpha_meas  = arrayfun(@(f) f.resp.settled_alpha_deg, FC);
C.alpha_book  = arrayfun(@(f) f.trim.alftr_deg,          FC);
C.alpha_resid = max(abs(C.alpha_meas - C.alpha_book) ./ abs(C.alpha_book));
C.alpha_dc_pred = arrayfun(@(f) abs(f.dc_offset_rel), FC);
C.alpha_tol   = 1.5e-2;
C.alpha_ok    = all(isfinite(C.alpha_meas)) && C.alpha_resid <= C.alpha_tol;

% -- 4: K1 and K3 identities, algebraic and simulated
%    Algebraic: the linear model's own steady state against the nonlinear
%    force equation evaluated AT the trim angle.  Nothing simulated here.
alg_k1 = zeros(1,nA); alg_k3 = zeros(1,nA);
for i = 1:nA
    f = FC(i);
    a = f.aero;
    CNt = 2*f.trim.alftr + 1.5*a.splan*f.trim.alftr^2/a.sref ...
          + 8*a.swing*f.trim.alftr/(a.b*a.sref) ...
          + 8*a.stail*(f.trim.alftr + f.trim.del)/(a.b*a.sref);
    nl_trim_g = a.q*a.sref*CNt/cfg.wgt;              % g, since XNL/g cancels
    alg_k1(i) = abs(nl_trim_g/f.lin.nl_ss_g - 1);
    % thetadot at trim, from the differential equation: alphadot = 0 =>
    % thetadot = nL/VM.  In deg/s.
    thd_trim = nl_trim_g*GRAV_FT/cfg.vm*DEG;
    alg_k3(i) = abs(thd_trim/f.lin.thd_ss_dps - 1);
end
C.k1_alg_resid = max(alg_k1);
C.k3_alg_resid = max(alg_k3);
% 1845/(57.3*32.2) - 1 = 1.84e-4.  The band is that number with headroom, and
% it is deliberately NOT 1e-12: see the units block at the top.
%
% WATCH THE TWO CHANNELS BEHAVE DIFFERENTLY, BECAUSE IT IS A RESULT.  K1
% carries ONE factor of 1845, so its residual FLOORS at 1.84e-4 and cannot go
% below it.  K3 = 1845*K1/VM carries a SECOND one, which divides the first
% back out, so the body-rate identity is exact to machine precision.  If both
% residuals came back at 1.84e-4, or both at 1e-16, one of the two 1845s has
% been dropped or duplicated somewhere -- so the ASYMMETRY is itself the test,
% and it is asserted below rather than merely printed.
C.k_alg_tol = 5e-4;
C.k_alg_expected = 1845/(57.3*GRAV_FT) - 1;
C.k_alg_asym_ok = C.k1_alg_resid > 0.5*abs(C.k_alg_expected) && ...
                  C.k3_alg_resid < 0.05*abs(C.k_alg_expected);
C.k_alg_ok  = C.k1_alg_resid <= C.k_alg_tol && ...
              C.k3_alg_resid <= C.k_alg_tol && C.k_alg_asym_ok;

% Simulated: the measured settled values against the same closed forms.
C.nl_meas  = arrayfun(@(f) f.resp.settled_accel_g, FC);
C.nl_form  = arrayfun(@(f) f.lin.nl_ss_g,          FC);
C.thd_meas = arrayfun(@(f) f.resp.settled_thd_dps, FC);
C.thd_form = arrayfun(@(f) f.lin.thd_ss_dps,       FC);
C.sim_resid = max([abs(C.nl_meas./C.nl_form - 1), abs(C.thd_meas./C.thd_form - 1)]);
% Same 1.5% for the same reason as check 3, plus the 1.84e-4 floor from 1845.
% ⚠️ THIS ROW IS THE ONE THAT SAW THE ESTIMATOR BUG FIRST and it is not on
%    the benchSuite board -- it only reached a reader as an unconditional
%    selfcheck throw.  Before 2026-08-23 it read 7.9e-2, five times its own
%    band, while the two rows that ARE on the board read 8.7e-3 and 8.6e-2.
%    If a residual here ever moves without the board moving, that is this
%    same blind spot, not a new bug.
C.sim_tol   = 1.5e-2;
C.sim_ok    = all(isfinite([C.nl_meas C.thd_meas])) && C.sim_resid <= C.sim_tol;

% -- 5: the two output channels tie together through alphadot = 0
%    This one has NO quadratic-offset excuse and is held tighter on purpose.
%    Averaging alphadot = thetadot - nL/VM over a cycle gives mean(alphadot)
%    = 0 for a bounded alpha, so the DC offset that moves BOTH channels in
%    check 3 cancels here exactly.  If this row is looser than check 3's
%    residual, the two channels are not describing the same vehicle.
%
%    ⚠️⚠️ THAT CANCELLATION IS A CLAIM ABOUT MEANS, AND IT COST US A RED
%    BOARD.  It is true of the mean of each channel over a cycle.  It is NOT
%    true of two SEPARATE estimates of two settled values: the residual left
%    on each channel depends on that channel's own ring-to-settled ratio, and
%    nL and thetadot are 90 deg apart in phase with ratios that differ by a
%    factor of ten.  So this row does not measure the cancellation, it
%    measures the DIFFERENCE of the two estimators' biases -- and it reads
%    exactly that: before 2026-08-23 the residual was 8.75e-3 at SEA LEVEL
%    (nL biased -1.6e-3, thetadot -1.02e-2; the difference is 8.65e-3) and
%    only 2.6e-3 at 50 kft.  The row was failing at the flight condition
%    nobody was looking at, because at 50 kft the two biases happened to be
%    nearly equal and cancelled in the ratio.  ⚠️ A PASS ON THIS ROW IS NOT
%    EVIDENCE THAT EITHER CHANNEL IS RIGHT -- two equally wrong channels pass
%    it.  Check 4 above is what tests them against the closed form.
C.chan_resid = max(abs((C.nl_meas*GRAV_FT/cfg.vm*DEG) ./ C.thd_meas - 1));
C.chan_tol   = 5e-3;
% ⚠️⚠️ `isfinite(max(...))` IS NOT `all(isfinite(...))`.  MATLAB's max() drops
%    NaNs, so a flight condition whose settled values could not be measured
%    vanishes from the maximum and this row goes GREEN on whatever altitudes
%    were left.  Found 2026-08-23 while checking short horizons: at
%    tstop = 0.8 s the 50-kft record has only two extrema, thetadot_ss is NaN,
%    and this row still reported 2.02e-04 PASS -- a green light for a
%    condition that was never measured.  The neighbouring alpha_ok, sim_ok and
%    ta_ok rows all guard with all(isfinite(...)); this one did not.  Fixed to
%    match them, and it is worth more than the residual it guards.
C.chan_ok    = all(isfinite([C.nl_meas C.thd_meas])) && ...
               isfinite(C.chan_resid) && C.chan_resid <= C.chan_tol;

% -- 5b: Ta MEASURED == Ta from the transfer function
%    Zarchan's definition (zarchan.txt:5418, :1839) is Ta = alpha/gammadot,
%    and at steady state gammadot = thetadot, so the nonlinear run measures
%    Ta directly as a ratio of two settled channels.  Substituting
%    alphaTR = -(Md/Ma)*delta into that ratio reproduces Md/(Ma*Zd - Md*Za)
%    EXACTLY, so this row ties the chapter's most project-relevant number --
%    the flight-path lag -- to the integrated run instead of leaving it an
%    unchecked output of a formula.
%
%    ⚠️ IT IS ALSO THE MOST SENSITIVE ROW ON THE BOARD, and for a reason
%    worth keeping: Ta is alpha_ss/thetadot_ss, and thetadot is the channel
%    whose ring is 1.5x (sea level) to 27x (50 kft) its own settled value.
%    Whatever a settled-value estimator gets wrong, it gets wrong HERE first
%    and by the largest margin -- 8.6e-2 against a 1.5e-2 band before
%    2026-08-23, while alpha alone was still inside its band.  Treat a
%    movement in this row as a question about the thetadot channel, not
%    about Ta.
C.ta_form  = arrayfun(@(f) f.lin.ta,         FC);
C.ta_meas  = arrayfun(@(f) f.resp.ta_meas_s, FC);
C.ta_resid = max(abs(C.ta_meas./C.ta_form - 1));
C.ta_tol   = 1.5e-2;
C.ta_ok    = all(isfinite(C.ta_meas)) && C.ta_resid <= C.ta_tol;

% -- 6: dynamic-pressure scaling across altitude
if nA >= 2
    q  = arrayfun(@(f) f.q, FC);
    rq = q/q(1);
    ref = FC(1);
    dev = [];
    for i = 2:nA
        f = FC(i);
        dev(end+1) = relDev(f.trim.alftr, ref.trim.alftr);            %#ok<AGROW>
        dev(end+1) = relDev(f.lin.xma, ref.lin.xma*rq(i));            %#ok<AGROW>
        dev(end+1) = relDev(f.lin.xmd, ref.lin.xmd*rq(i));            %#ok<AGROW>
        dev(end+1) = relDev(f.lin.za,  ref.lin.za *rq(i));            %#ok<AGROW>
        dev(end+1) = relDev(f.lin.zd,  ref.lin.zd *rq(i));            %#ok<AGROW>
        dev(end+1) = relDev(f.lin.xk1, ref.lin.xk1*rq(i));            %#ok<AGROW>
        dev(end+1) = relDev(f.lin.xk3, ref.lin.xk3*rq(i));            %#ok<AGROW>
        dev(end+1) = relDev(f.lin.waf, ref.lin.waf*sqrt(rq(i)));      %#ok<AGROW>
        dev(end+1) = relDev(f.lin.zaf, ref.lin.zaf*sqrt(rq(i)));      %#ok<AGROW>
        dev(end+1) = relDev(f.lin.wz,  ref.lin.wz *sqrt(rq(i)));      %#ok<AGROW>
        dev(end+1) = relDev(f.lin.ta,  ref.lin.ta /rq(i));            %#ok<AGROW>
    end
    C.qscale_resid = max(dev);
    C.qscale_tol   = 1e-10;
    C.qscale_ok    = C.qscale_resid <= C.qscale_tol;
    C.qscale_skipped = false;
else
    C.qscale_resid = NaN;  C.qscale_tol = 1e-10;
    C.qscale_ok = false;   C.qscale_skipped = true;
end

% -- 7: measured dynamics vs the Jacobian, and vs Zarchan's secant
C.wn_meas  = arrayfun(@(f) f.resp.wn,   FC);
C.wn_jac   = arrayfun(@(f) f.lin.waf_jac, FC);
C.wn_zar   = arrayfun(@(f) f.lin.waf,     FC);
C.ze_meas  = arrayfun(@(f) f.resp.zeta,   FC);
C.ze_jac   = arrayfun(@(f) f.lin.zaf_jac, FC);
C.ze_zar   = arrayfun(@(f) f.lin.zaf,     FC);
C.dyn_meas_ok = all(arrayfun(@(f) f.resp.meas_ok, FC));
if C.dyn_meas_ok
    C.wn_resid = max(abs(C.wn_meas./C.wn_jac - 1));
    C.ze_resid = max(abs(C.ze_meas./C.ze_jac - 1));
else
    C.wn_resid = NaN;  C.ze_resid = NaN;
end
% ABSOLUTE bands, generous, because the log decrement is read over one cycle
% of a lightly damped oscillation whose damping coefficient itself varies
% with alpha across that cycle.  1% and 10% are loose enough not to fail a
% correct port and would still catch a wrong linearization, which is off by
% factors, not per cents.
C.wn_tol = 0.01;
C.ze_tol = 0.10;
% THE DISCRIMINATING TEST, and the reason this pair of rows exists at all.
% Zarchan's secant sits +0.9% (wn) and +10% (zeta) from the tangent, so a
% loose absolute band alone could not tell the two apart on the damping
% channel.  This asks the sharper question directly: is the MEASURED value
% nearer the tangent than the secant?  It is robust to a few per cent of
% measurement error and it is what actually establishes that the nonlinear
% airframe oscillates on the Jacobian and not on CMA.
C.wn_secant_gap = max(abs(C.wn_zar./C.wn_jac - 1));
C.ze_secant_gap = max(abs(C.ze_zar./C.ze_jac - 1));
if C.dyn_meas_ok
    C.wn_nearer = all(abs(C.wn_meas - C.wn_jac) < abs(C.wn_meas - C.wn_zar));
    C.ze_nearer = all(abs(C.ze_meas - C.ze_jac) < abs(C.ze_meas - C.ze_zar));
else
    C.wn_nearer = false;  C.ze_nearer = false;
end
C.dyn_ok = C.dyn_meas_ok && C.wn_resid <= C.wn_tol && C.ze_resid <= C.ze_tol ...
           && C.wn_nearer && C.ze_nearer;

C.ok = C.trim_ok && C.slope_ok && C.jac_ok && C.alpha_ok && C.k_alg_ok && ...
       C.sim_ok && C.chan_ok && C.ta_ok && ...
       (C.qscale_ok || C.qscale_skipped) && C.dyn_ok;
% ⚠️⚠️ 2026-08-23:自檢失敗原本是**無條件**報錯,而 `assert=false` 對它
%    沒有作用 ⇒ 一旦紅燈,連 R 都拿不到,**沒辦法診斷是哪一側錯**。
%    「做不到就報錯」是對的,但它不該同時把證據一起帶走:
%    現在 assert=false 時把整份自檢掛回 R 並印警告,呼叫端自己決定。
%    ⚠️ 預設仍是報錯 —— 靜默回傳一份「每個數都可疑」的結果更糟。
if ~C.ok && ~cfg.assert
    warning('mgs:zarchan:airframeSim:selfcheckSoft', ...
        ['STRUCTURAL invariant(s) failed and assert=false, so R is being ' ...
         'returned ANYWAY.  Every number in it is suspect.  R.selfcheck ' ...
         'carries the per-check residuals.']);
end
if ~C.ok && cfg.assert
    error('mgs:zarchan:airframeSim:selfcheck', ...
        ['a STRUCTURAL invariant failed, so every number this function ' ...
         'computed is suspect:\n' ...
         '  CM(alphaTR) == 0                         : rel %.3e (tol %.1e) %s\n' ...
         '  slope at root == sqrt(disc)              : rel %.3e (tol %.1e) %s\n' ...
         '  dCM/dalpha == y1 + 2*y2*alphaTR          : rel %.3e (tol %.1e) %s\n' ...
         '  ODE settles on the quadratic''s root      : rel %.3e (tol %.1e) %s\n' ...
         '  nL_trim == K1*delta (algebra)            : rel %.3e (tol %.1e) %s\n' ...
         '  thetadot_trim == K3*delta (algebra)      : rel %.3e (tol %.1e) %s\n' ...
         '  the 1845 appears once in K1, twice in K3 : %s\n' ...
         '  simulated settled == closed form         : rel %.3e (tol %.1e) %s\n' ...
         '  thetadot_ss == nL_ss*g/VM                : rel %.3e (tol %.1e) %s\n' ...
         '  Ta = alpha/gammadot measured vs closed fm: rel %.3e (tol %.1e) %s\n' ...
         '  Q-scaling across altitude                : rel %.3e (tol %.1e) %s\n' ...
         '  measured wn vs Jacobian                  : rel %.3e (tol %.1e) %s\n' ...
         '  measured zeta vs Jacobian                : rel %.3e (tol %.1e) %s\n' ...
         '  measured wn/zeta nearer tangent than CMA : %s / %s\n' ...
         'These test the wiring, not the book.  A failure here means the ' ...
         'transcription of Listing 21.1 or of the Ch.21 linearization has ' ...
         'been damaged.'], ...
        C.trim_resid, C.trim_tol, tf2s(C.trim_ok), ...
        C.slope_resid, C.slope_tol, tf2s(C.slope_ok), ...
        C.jac_resid, C.jac_tol, tf2s(C.jac_ok), ...
        C.alpha_resid, C.alpha_tol, tf2s(C.alpha_ok), ...
        C.k1_alg_resid, C.k_alg_tol, tf2s(C.k1_alg_resid <= C.k_alg_tol), ...
        C.k3_alg_resid, C.k_alg_tol, tf2s(C.k3_alg_resid <= C.k_alg_tol), ...
        tf2s(C.k_alg_asym_ok), ...
        C.sim_resid, C.sim_tol, tf2s(C.sim_ok), ...
        C.chan_resid, C.chan_tol, tf2s(C.chan_ok), ...
        C.ta_resid, C.ta_tol, tf2s(C.ta_ok), ...
        C.qscale_resid, C.qscale_tol, tf2s(C.qscale_ok || C.qscale_skipped), ...
        C.wn_resid, C.wn_tol, tf2s(isfinite(C.wn_resid) && C.wn_resid <= C.wn_tol), ...
        C.ze_resid, C.ze_tol, tf2s(isfinite(C.ze_resid) && C.ze_resid <= C.ze_tol), ...
        tf2s(C.wn_nearer), tf2s(C.ze_nearer));
end
end


function d = relDev(a, b)
d = abs(a - b) / max(abs(b), eps);
end


% =============================================================================
%   BOOK BENCHMARKS
% =============================================================================
function B = benchmarks(R, cfg, offbook, DEG)
%BENCHMARKS  Computed vs published, with a band and a reason for the band.
%
%   BANDS ENCODE PROVENANCE, NOT CONVENIENCE.  Almost every number Ch.21
%   prints was read off a figure by the author and quoted to one or two
%   significant figures ("approximately 13 g", "approximately 22.7" i.e.
%   -2.7, "approximately 4 s").  Demanding 1% of a number somebody eyeballed
%   off a plot would fail a correct port.  Demanding 15% of a number printed
%   to three figures would pass a broken one.  Do NOT widen a band to turn a
%   row green.
%
%   THE OFF-BOOK GATE.  Every row names the parameters its book value depends
%   on.  If the caller moved one, the row is SKIPPED with the reason, rather
%   than compared against a book value for a different vehicle.
%
%   'alt' IS DELIBERATELY NOT IN THE GATE.  Altitude is handled by FINDALT
%   instead: a row that needs 50 kft is evaluated iff 50 kft is actually in
%   the run, and FINDALT refuses to interpolate.  Gating on 'alt' as well
%   would skip every altitude row the moment a caller asked for the book's
%   two altitudes PLUS a third -- a strictly more informative run being
%   punished for it.
%
%   'h' AND 'tstop' ARE NOT IN THE GATE EITHER, and that is a claim this file
%   has to back up rather than assume.  h is a discretization knob whose
%   effect is bounded by the anti-damping guard in FLIGHTCONDITION; tstop is
%   a horizon, and the settled estimator is built to be horizon-insensitive
%   -- it MODELS the ring rather than waiting for it to stop -- which is
%   exactly what the "ODE settles on the quadratic's root" structural check
%   polices.  If that check ever loosens, these two belong in the gate.
%   ⚠️ "Horizon-insensitive" is a claim with a floor: the fit needs a full
%   cycle (three extrema) before it means anything, and RESPONSEFEATURES
%   refuses below that rather than extrapolating.  Verified 2026-08-23 by
%   running the same integrator to 400 s: the settled values the 2-s record
%   produces reproduce the 400-s ones to 1e-4, which is what makes leaving
%   tstop out of the gate defensible instead of merely convenient.

B = emptyBench();

CORE = {'vm','wgt','a','del_deg'};   % anything that changes the flight case
i0  = findAlt(cfg, 0);
i50 = findAlt(cfg, 50000);

L6527 = 'Fig. 21.4 / p.481 text, zarchan.txt:6527';
L6528 = 'Fig. 21.5 / p.481 text, zarchan.txt:6528';
L6529 = 'Figs. 21.6-21.7 / p.481 text, zarchan.txt:6529';
L6423 = 'Ch.21 introduction, zarchan.txt:6423';
L6784 = 'Fig. 21.12 / p.493 text, zarchan.txt:6784';
L6798 = 'Fig. 21.13 / p.494 text, zarchan.txt:6798';
L6810 = 'Fig. 21.16 / p.495 text, zarchan.txt:6810';
L6835 = 'Fig. 21.17 / p.496 text, zarchan.txt:6835';
L6853 = 'Fig. 21.18 / p.497 text, zarchan.txt:6853';
L6854 = 'Fig. 21.19 / p.497 text, zarchan.txt:6854';
L6770 = 'Fig. 21.10 / p.492 text, zarchan.txt:6770';

OCR = ['book prints "%s"; the OCR turned this chapter''s minus signs into ' ...
       'the digit 2 -- forced by the surviving arithmetic on line 6853 ' ...
       '("22.7  5 = 213.5") and by line 6527 stating the sign in words'];

% ---------------------------------------------------------------------------
%   SEA LEVEL, 5-DEG FIN, 3000 ft/s -- the case of Figs. 21.4 and 21.5
% ---------------------------------------------------------------------------
w = gateWhy(offbook, CORE);
if isempty(i0)
    B = addSkip(B, 'trim alpha, sea level', '-5 deg', ...
        'alt = 0 is not in this run', L6527, 'deg');
    B = addSkip(B, 'peak alpha, sea level', '-8 deg', ...
        'alt = 0 is not in this run', L6527, 'deg');
    B = addSkip(B, 'steady accel, sea level (Fig. 21.5)', '-13 g', ...
        'alt = 0 is not in this run', L6528, 'g');
elseif ~isempty(w)
    B = addSkip(B, 'trim alpha, sea level', '-5 deg', w, L6527, 'deg');
    B = addSkip(B, 'peak alpha, sea level', '-8 deg', w, L6527, 'deg');
    B = addSkip(B, 'steady accel, sea level (Fig. 21.5)', '-13 g', w, L6528, 'g');
else
    F0 = R.fc(i0);
    B = addVal(B, 'trim alpha, sea level', -5, F0.resp.settled_alpha_deg, ...
        'graph', L6527, 'deg', ...
        [sprintf(OCR, '25 deg') '.  Quoted as "a steady-state or trim value ' ...
         'of approximately -5 deg after several seconds" -- one significant ' ...
         'figure, read off Fig. 21.4, hence the graph band']);
    B = addVal(B, 'peak alpha, sea level', -8, F0.resp.peak_alpha_deg, ...
        'graph', L6527, 'deg', ...
        [sprintf(OCR, '28 deg') '.  "The angle of attack initially ' ...
         'approaches -8 deg, oscillates".  The overshoot is 1.87x trim ' ...
         'because zeta is 0.046']);
    B = addVal(B, 'steady accel, sea level (Fig. 21.5)', -13, ...
        F0.resp.settled_accel_g, 'graph', L6528, 'g', ...
        ['"the steady-state acceleration due to a 5-deg fin deflection is ' ...
         'approximately 13 g"; the prose gives no sign there, and the sign ' ...
         'is supplied by line 6853''s arithmetic (-2.7*5 = -13.5) and by ' ...
         'line 6527''s "negative angle of attack".  NOTE the book quotes ' ...
         'BOTH 13 and 13.5 g for this same quantity -- see the next row']);
end

% ---------------------------------------------------------------------------
%   THE K1 / K3 ARITHMETIC ON pp.497 -- both altitudes, both channels
% ---------------------------------------------------------------------------
B = addAltVal(B, R, cfg, offbook, i0,  'K1 (accel gain), sea level', -2.7, ...
    @(f) f.lin.xk1, 'graph', L6853, 'g/deg', 0, CORE, ...
    [sprintf(OCR, '22.7') '.  "if the missile is traveling at 3000 ft/s the ' ...
     'aerodynamic gain is approximately -2.7 at sea level"']);
B = addAltVal(B, R, cfg, offbook, i50, 'K1 (accel gain), 50 kft', -0.4, ...
    @(f) f.lin.xk1, 'graph', L6853, 'g/deg', 50000, CORE, ...
    [sprintf(OCR, '20.4') '.  One significant figure -- the loosest source ' ...
     'value in the chapter, and the band still holds it']);
B = addAltVal(B, R, cfg, offbook, i0,  'steady accel = K1*5deg, sea level', -13.5, ...
    @(f) f.lin.nl_ss_g, 'graph', L6853, 'g', 0, CORE, ...
    ['the book''s own arithmetic "-2.7 * 5 = -13.5", which disagrees with ' ...
     'its own "approximately 13 g" on line 6528 by 4%.  A port that lands ' ...
     'BETWEEN the two is right; one that matches either exactly is fitting']);
B = addAltVal(B, R, cfg, offbook, i50, 'steady accel (nonlinear run), 50 kft', -2, ...
    @(f) f.resp.settled_accel_g, 'graph', L6853, 'g', 50000, CORE, ...
    ['"only -2 g at 50 kft altitude (i.e., -0.4 * 5 = -2)".  Line 6853 adds ' ...
     '"The nonlinear results of Fig. 21.7 confirm these calculations", so ' ...
     'this row is scored on the SIMULATED settled value, not on K1*delta']);

B = addAltVal(B, R, cfg, offbook, i0,  'K3 (body-rate gain), sea level', -1.6, ...
    @(f) f.lin.xk3, 'graph', L6854, '1/s', 0, CORE, ...
    [sprintf(OCR, '21.6 s21') '.  The sharpest of the four gain readings']);
B = addAltVal(B, R, cfg, offbook, i50, 'K3 (body-rate gain), 50 kft', -0.24, ...
    @(f) f.lin.xk3, 'graph', L6854, '1/s', 50000, CORE, ...
    sprintf(OCR, '20.24 s21'));
B = addAltVal(B, R, cfg, offbook, i0,  'body rate ss, sea level', -8, ...
    @(f) f.resp.settled_thd_dps, 'graph', L6854, 'deg/s', 0, CORE, ...
    ['"a 5-deg fin deflection would result in -8 deg/s at sea level".  ' ...
     'Scored on the SIMULATED settled body rate -- the array Listing 21.1 ' ...
     'does not keep and this port does, precisely so this sentence is ' ...
     'checkable']);
B = addAltVal(B, R, cfg, offbook, i50, 'body rate ss, 50 kft', -1.2, ...
    @(f) f.resp.settled_thd_dps, 'graph', L6854, 'deg/s', 50000, CORE, ...
    '"and only -1.2 deg/s at 50 kft altitude (i.e., -0.24 * 5 = -1.2)"');

B = addAltVal(B, R, cfg, offbook, i50, 'turning-rate time constant Ta, 50 kft', 4, ...
    @(f) f.lin.ta, 'graph', L6810, 's', 50000, CORE, ...
    ['"the turning rate time constant is approximately 4 s when the missile ' ...
     'is traveling at 3000 ft/s".  THE MOST IMPORTANT ROW IN THIS FILE for ' ...
     'our purposes: Ta is defined (zarchan.txt:5418) as "the amount of time ' ...
     'it takes to turn the missile flight path angle g through an equivalent ' ...
     'angle of attack a", i.e. Ta = alpha/gammadot -- the SAME quantity our ' ...
     '0.063 gamma-tracking gain is measured on, and the only derived time ' ...
     'constant in Ch.21 of the same order as our 8 s']);

% ---------------------------------------------------------------------------
%   BOUNDS -- ranges the prose states, evaluated where they can be adjudicated
% ---------------------------------------------------------------------------
if ~isempty(i0) && isempty(gateWhy(offbook, CORE))
    F0 = R.fc(i0);
    ok = F0.lin.waf >= 10 && F0.lin.waf <= 30;
    B = addBound(B, 'wAF inside the Fig. 21.12 family, sea level', ...
        '10 to 30 rad/s', sprintf('%.4g rad/s', F0.lin.waf), ok, L6784, 'rad/s', ...
        ['"the airframe natural frequency varied between 10 rad/s and ' ...
         '30 rad/s".  Scored ONLY at sea level -- see the INFO row for why ' ...
         'the 50-kft point cannot be adjudicated against this band']);
    ok = F0.lin.zaf >= 0.02 && F0.lin.zaf <= 0.065;
    B = addBound(B, 'zAF inside the Fig. 21.13 family, sea level', ...
        '0.02 to 0.065', sprintf('%.4g', F0.lin.zaf), ok, L6798, '-', ...
        ['"The airframe damping is quite low and in this example varies ' ...
         'between 0.02 and 0.065".  This is Zarchan''s SECANT zeta, which is ' ...
         'what Fig. 21.13 plots; the measured one is ~10% lower and is a ' ...
         'separate row']);
    ok = F0.sim.alpha_max_deg < 20;
    B = addBound(B, 'peak |alpha| inside the stated aero validity', ...
        '< 20 to 30 deg', sprintf('%.4g deg', F0.sim.alpha_max_deg), ok, ...
        L6423, 'deg', ...
        ['"an excellent approximation to the nonlinear equations for angles ' ...
         'of attack less than 20 to 30 deg".  Scored against the STRICT end ' ...
         'of the book''s own range, so a fin deflection that pushes the ' ...
         'model out of validity turns this row red instead of quietly ' ...
         'extrapolating']);
end

% ---------------------------------------------------------------------------
%   SHAPE ROWS -- prose claims about sign and ordering.  These cannot be
%   passed by accident, which makes several of them stronger than any number
%   read off a plot.
% ---------------------------------------------------------------------------
if ~isempty(i0) && isempty(gateWhy(offbook, CORE))
    F0 = R.fc(i0);
    B = addShape(B, 'positive fin -> negative alpha', ...
        'alpha < 0 for delta > 0', ...
        sprintf('alpha_trim = %+.4g deg for delta = %+.4g deg', ...
                F0.resp.settled_alpha_deg, cfg.del_deg), ...
        F0.resp.settled_alpha_deg < 0 && cfg.del_deg > 0, L6527, ...
        ['"a 5-deg fin deflection in the positive direction causes the ' ...
         'missile to build up to a negative angle of attack".  This is the ' ...
         'sentence that settles the OCR minus-sign question for the whole ' ...
         'chapter']);
    B = addShape(B, 'the oscillation decays (low but positive damping)', ...
        'decaying oscillation', ...
        sprintf('measured zeta = %.4g over %d extrema', ...
                F0.resp.zeta, F0.resp.n_extrema), ...
        isfinite(F0.resp.zeta) && F0.resp.zeta > 0 && F0.resp.zeta < 0.2, ...
        L6527, ...
        ['"The decaying oscillations indicate that the missile airframe has ' ...
         'low damping."  Measured by log decrement over the last full ' ...
         'period, not asserted from Ma and Za']);
    B = addShape(B, 'acceleration first moves the WRONG way', ...
        'initial sign opposite to steady state', ...
        sprintf('n(0) = %+.3g g -> n_ss = %+.3g g, wrong-way for %.3g s', ...
                F0.resp.wrongway_first_g, F0.resp.settled_accel_g, ...
                F0.resp.wrongway_dur_s), ...
        F0.resp.wrongway_present, L6835, ...
        ['the right-half-plane airframe zero wZ, which the book calls the ' ...
         '"wrong-way tail effect".  The tail force acts before the body has ' ...
         'any angle of attack.  A transcription that dropped the tail term ' ...
         'from CN would still settle correctly and would lose THIS']);
end

if ~isempty(i0) && ~isempty(i50) && isempty(gateWhy(offbook, CORE))
    F0 = R.fc(i0);  F5 = R.fc(i50);
    B = addShape(B, 'natural frequency falls with altitude', ...
        'wAF(50 kft) < wAF(sea level)', ...
        sprintf('%.4g < %.4g rad/s', F5.lin.waf, F0.lin.waf), ...
        F5.lin.waf < F0.lin.waf, L6529, ...
        '"In addition, the airframe natural frequency decreases with increasing altitude."');
    B = addShape(B, 'less acceleration available at altitude', ...
        '|n_ss(50 kft)| < |n_ss(sea level)|', ...
        sprintf('%.4g < %.4g g', abs(F5.resp.settled_accel_g), ...
                abs(F0.resp.settled_accel_g)), ...
        abs(F5.resp.settled_accel_g) < abs(F0.resp.settled_accel_g), L6529, ...
        ['"at higher altitudes the missile will ... have less acceleration ' ...
         'available for a given fin deflection"']);
    B = addShape(B, 'more angle of attack at altitude (PEAK, not trim)', ...
        '|alpha_peak(50 kft)| > |alpha_peak(sea level)|', ...
        sprintf('%.4g > %.4g deg', abs(F5.resp.peak_alpha_deg), ...
                abs(F0.resp.peak_alpha_deg)), ...
        abs(F5.resp.peak_alpha_deg) > abs(F0.resp.peak_alpha_deg), L6529, ...
        ['"at higher altitudes the missile will pull more angle of attack".  ' ...
         'TRUE OF THE PEAK ONLY -- the TRIM angle is exactly altitude ' ...
         'independent in this model.  See the INFO row; scoring this on trim ' ...
         'would fail a correct port']);
    B = addShape(B, 'Ta grows with altitude', 'Ta(50 kft) > Ta(sea level)', ...
        sprintf('%.4g > %.4g s', F5.lin.ta, F0.lin.ta), ...
        F5.lin.ta > F0.lin.ta, L6810, ...
        ['"the missile turning rate time constant Ta increases with ' ...
         'increasing altitude and increasing missile speed"']);
    B = addShape(B, '|K1| shrinks with altitude', '|K1(50 kft)| < |K1(0)|', ...
        sprintf('%.4g < %.4g g/deg', abs(F5.lin.xk1), abs(F0.lin.xk1)), ...
        abs(F5.lin.xk1) < abs(F0.lin.xk1), L6853, ...
        '"the magnitude of the aerodynamic gain gets smaller as the altitude increases"');
    B = addShape(B, '|K3| shrinks with altitude', '|K3(50 kft)| < |K3(0)|', ...
        sprintf('%.4g < %.4g 1/s', abs(F5.lin.xk3), abs(F0.lin.xk3)), ...
        abs(F5.lin.xk3) < abs(F0.lin.xk3), L6854, ...
        '"the magnitude of the aerodynamic gain gets smaller as the altitude increases"');
    B = addShape(B, 'airframe zero falls with altitude', ...
        'wZ(50 kft) < wZ(sea level)', ...
        sprintf('%.4g < %.4g rad/s', F5.lin.wz, F0.lin.wz), ...
        F5.lin.wz < F0.lin.wz, L6835, ...
        ['"the airframe zero decreases with increasing altitude and ' ...
         'decreasing missile velocity"']);
end

% ---------------------------------------------------------------------------
%   INFO -- reported, never scored.  Each one says WHY it is not scored.
% ---------------------------------------------------------------------------
if ~isempty(i50)
    F5 = R.fc(i50);
    B = addInfo(B, 'wAF / zAF at 50 kft vs the Fig. 21.12/21.13 bands', ...
        '10-30 rad/s and 0.02-0.065', ...
        sprintf('%.4g rad/s (%.1f%% below 10) and %.4g (%.1f%% below 0.02)', ...
                F5.lin.waf, 100*(1 - F5.lin.waf/10), ...
                F5.lin.zaf, 100*(1 - F5.lin.zaf/0.02)), ...
        [L6784 ' and ' L6798], ...
        ['NOT SCORED.  Those endpoints are the extremes of a whole altitude ' ...
         'x VELOCITY family, and the figure''s velocity grid is an image ' ...
         'link that did not survive extraction.  Our 3000-ft/s point at ' ...
         '50 kft falls just outside both, which is consistent with the ' ...
         'family reaching lower speeds or higher altitudes than the two ' ...
         'points we run -- but it cannot be adjudicated, and widening the ' ...
         'band to swallow it would be inventing a book claim.  Same for the ' ...
         '"time constant variation is between 0.033 s and 0.1 s" sentence: ' ...
         'ours is 1/wAF = ' sprintf('%.4g', 1/F5.lin.waf) ' s at 50 kft']);
end

if ~isempty(i0) && ~isempty(i50)
    F0 = R.fc(i0);  F5 = R.fc(i50);
    B = addInfo(B, 'trim alpha is EXACTLY altitude independent', ...
        'book implies it rises with altitude', ...
        sprintf('%.6g deg at sea level, %.6g deg at 50 kft (identical)', ...
                F0.trim.alftr_deg, F5.trim.alftr_deg), L6529, ...
        ['NOT SCORED.  In this model the trim condition is CM = 0, and CM''s ' ...
         'coefficients y1, y2, y3 contain no density and no dynamic ' ...
         'pressure -- only geometry, B and delta.  So a given fin deflection ' ...
         'at a given Mach trims at the SAME alpha at every altitude, exactly. ' ...
         'The book''s "pull more angle of attack" (line 6529) is therefore a ' ...
         'statement about the transient envelope, which at 50 kft has barely ' ...
         'decayed inside the 2-s window (zeta ~ ' ...
         sprintf('%.3g', F5.lin.zaf) ').  This is asserted as a structural ' ...
         'invariant, not tolerated as a discrepancy']);
    % n(t=0)/|n_ss| = CN(0,delta)/|CN(alphaTR,delta)|.  Both CN evaluations
    % sit on the same Q*Sref/W, so the ratio is EXACTLY altitude independent
    % at fixed VM -- computed here rather than measured, because the sampled
    % maximum is not the t=0 value and the 0.01-s grid catches a 2.6x slower
    % oscillation at a different phase.
    r0 = @(f) (8*f.aero.stail*f.trim.del/(f.aero.b*f.aero.sref)) / ...
              abs(f.lin.cna*f.trim.alftr + f.lin.cnd*f.trim.del);
    B = addInfo(B, 'wrong-way excursion: size invariant, duration is not', ...
        'smaller wZ causes "more wrong-way tail effect"', ...
        sprintf(['analytic n(0)/|n_ss| = %.5g vs %.5g (invariant); sampled ' ...
                 'max %.4g vs %.4g; duration %.3g s vs %.3g s'], ...
                r0(F0), r0(F5), F0.resp.wrongway_ratio, F5.resp.wrongway_ratio, ...
                F0.resp.wrongway_dur_s, F5.resp.wrongway_dur_s), L6835, ...
        ['NOT SCORED.  At fixed VM every force in this model scales with Q, ' ...
         'so the wrong-way excursion and the steady state scale together and ' ...
         'their true RATIO is altitude invariant exactly -- the first two ' ...
         'numbers above.  The SAMPLED maxima differ only because the book''s ' ...
         '0.01-s grid catches a 2.6x slower oscillation at a different ' ...
         'phase, which is a sampling artefact and is shown so it is not ' ...
         'mistaken for physics.  What altitude really changes is how LONG ' ...
         'the wrong-way phase lasts, which goes as 1/sqrt(Q).  The book''s ' ...
         'claim is an adjective ("more") about a plot, so it gets no verdict ' ...
         'here -- only the three measurements that decompose it']);
end

if ~isempty(i0)
    F0 = R.fc(i0);
    % Nonlinearity index: how big the alpha^2 moment term is against the
    % linear one, at trim.  This is the mechanism behind Fig. 21.10.
    ix5  = abs(F0.trim.y2 * F0.trim.alftr / F0.trim.y1);
    TR10 = trimForFin(F0.aero, 2*F0.trim.del, DEG);
    ix10 = abs(TR10.y2 * TR10.alftr / TR10.y1);
    B = addInfo(B, 'linear model "deteriorates" at 10-deg fin', ...
        'adjective only -- no number published', ...
        sprintf(['alpha_trim %.4g -> %.4g deg (superlinear); alpha^2 moment ' ...
                 'share %.3g%% -> %.3g%%'], ...
                F0.trim.alftr_deg, TR10.alftr_deg, 100*ix5, 100*ix10), ...
        L6770, ...
        ['NOT SCORED.  "when the fin deflection is increased the linear ' ...
         'approximation to the airframe starts to deteriorate" is an ' ...
         'adjective about Fig. 21.10, and the comparison it describes needs ' ...
         'Listing 21.2''s integration loop, which is a separate port.  What ' ...
         'is reported instead is the MECHANISM: doubling the fin more than ' ...
         'doubles the trim angle, and the share of the pitching moment ' ...
         'carried by the alpha^2 body-lift term doubles with it']);

    B = addInfo(B, 'Zarchan''s CMA is a secant, not the Jacobian', ...
        'not a book claim', ...
        sprintf(['wAF %.4g (secant) vs %.4g (tangent) = %+.2f%%;  ' ...
                 'zAF %.4g vs %.4g = %+.2f%%'], ...
                F0.lin.waf, F0.lin.waf_jac, 100*(F0.lin.waf/F0.lin.waf_jac-1), ...
                F0.lin.zaf, F0.lin.zaf_jac, 100*(F0.lin.zaf/F0.lin.zaf_jac-1)), ...
        'Ch.21 p.483 recipe, zarchan.txt:6563', ...
        ['NOT SCORED -- there is no book value to score it against, and the ' ...
         'book is not wrong.  Dividing the alpha^2 term by alpha gives a ' ...
         'SECANT slope, which is exactly right for reproducing the trim ' ...
         'acceleration (that is what K1 is for) and 10% off for predicting ' ...
         'the damping the nonlinear airframe actually shows.  The structural ' ...
         'check confirms the MEASURED damping tracks the tangent, not the ' ...
         'secant.  Anyone comparing a measured zeta against Zarchan''s zAF ' ...
         'and finding 10% should read this row before hunting an integrator ' ...
         'bug']);
end

% ---------------------------------------------------------------------------
%   The linear-vs-nonlinear match of Fig. 21.9 needs the other listing.
% ---------------------------------------------------------------------------
B = addSkip(B, 'linear model is a "near perfect match" (Fig. 21.9)', ...
    'qualitative', ...
    ['needs Listing 21.2''s integration loop, which is a separate port; ' ...
     'only its aerodynamic preamble is used here, and comparing this file ' ...
     'against its own preamble would not test the claim'], ...
    'Fig. 21.9 / p.489 text, zarchan.txt:6747', '-');
end


% -----------------------------------------------------------------------------
function B = addAltVal(B, R, cfg, offbook, idx, what, book, getter, band, ...
                       source, units, altWanted, core, detail)
%ADDALTVAL  One value row at one altitude, with the off-book gate applied.
w = gateWhy(offbook, core);
if isempty(idx)
    B = addSkip(B, what, sprintf('%.4g %s', book, units), ...
        sprintf('alt = %g ft is not in this run (have %s)', altWanted, ...
                mat2str(cfg.alt)), source, units);
elseif ~isempty(w)
    B = addSkip(B, what, sprintf('%.4g %s', book, units), w, source, units);
else
    B = addVal(B, what, book, getter(R.fc(idx)), band, source, units, detail);
end
end


% =============================================================================
%   BENCH ROW PLUMBING.  Schema matches mgs.zarchan.adjointWithAutopilot so
%   the benchSuite adapter for this port is a copy of adaptAutopilot.
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
B(i).units = units;  B(i).source = source;  B(i).detail = detail;
if ~isfinite(ours) || book == 0
    B(i).pct = NaN;  B(i).ok = false;  B(i).skipped = true;
    B(i).why = 'the computed value is not finite (or the book value is zero)';
    return;
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
switch lower(band)
case 'table',  t = 0.01;   % three significant figures, printed in a table
case 'text',   t = 0.02;   % prose, 2-3 figures, or recomputed from rounded inputs
case 'approx', t = 0.05;   % the book says "approximately" about a computed value
case 'graph',  t = 0.15;   % the author read it off a plot: 1-2 significant figures
otherwise
    error('mgs:zarchan:airframeSim:band', ...
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
                 'number against a book value for a different flight case'], ...
                strjoin(hit, ', '));
end
end


function i = findAlt(cfg, a)
% Exact-ish only.  Interpolating between altitudes would invent a flight
% condition the run never simulated and then score the book against it.
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


function s = numList(v, fmt)
s = strjoin(arrayfun(@(x) sprintf(fmt, x), v(:).', 'UniformOutput', false), ' / ');
end


% =============================================================================
%   RENDER
% =============================================================================
function s = render(R, cfg, offbook)
W = 100;
L = {};
L{end+1} = 'Zarchan Listing 21.1 -- NONLINEAR AIRFRAME SIMULATION (Ch.21, Figs. 21.3-21.7)';
L{end+1} = '*** UNITS ARE ft, ft/s, lb, deg, g = 32.2.  rad->deg via the BOOK''S 57.3. ***';
L{end+1} = sprintf(['fin = %g deg   VM = %g ft/s (Mach %.3g vs a flat a = %g)   ' ...
                    'W = %g lb   h = %g s   t = 0..%g s'], ...
    cfg.del_deg, cfg.vm, cfg.vm/cfg.a, cfg.a, cfg.wgt, cfg.h, cfg.tstop);
L{end+1} = sprintf('altitudes: %s ft        off-book: %s', ...
    mat2str(cfg.alt), offbookStr(offbook));
L{end+1} = 'integrator: Zarchan''s two-pass predictor/corrector, transcribed';
L{end+1} = repmat('-', 1, W);

% -- flight conditions
L{end+1} = 'FLIGHT CONDITION AND LINEARIZED AIRFRAME  (trim solve + derivatives: Ch.21 pp.483-487,';
L{end+1} = '  identical to the preamble of Listing 21.2 -- NOT outputs of Listing 21.1)';
L{end+1} = sprintf('  %-9s %9s %9s %9s %9s %9s %9s %9s %9s', ...
    'alt [ft]', 'rho', 'Q [psf]', 'aTR[deg]', 'Ma', 'Md', 'Za', 'Zd', 'wZ');
for i = 1:numel(R.fc)
    f = R.fc(i);
    L{end+1} = sprintf('  %-9g %9.3e %9.1f %9.4f %9.2f %9.2f %9.4f %9.5f %9.3f', ...
        f.alt, f.rho, f.q, f.trim.alftr_deg, f.lin.xma, f.lin.xmd, ...
        f.lin.za, f.lin.zd, f.lin.wz); %#ok<AGROW>
end
L{end+1} = sprintf('  %-9s %9s %9s %9s %9s %9s %9s', ...
    'alt [ft]', 'wAF', 'zAF', '1/wAF[s]', 'K1[g/dg]', 'K3[1/s]', 'Ta [s]');
for i = 1:numel(R.fc)
    f = R.fc(i);
    L{end+1} = sprintf('  %-9g %9.4f %9.5f %9.4f %9.4f %9.4f %9.4f', ...
        f.alt, f.lin.waf, f.lin.zaf, f.lin.tau_airframe, f.lin.xk1, ...
        f.lin.xk3, f.lin.ta); %#ok<AGROW>
end
L{end+1} = repmat('-', 1, W);

% -- measured response
L{end+1} = 'MEASURED FROM THE INTEGRATED RUN  (settled = constant term of a fit to the ring model';
L{end+1} = '  the quadratic airframe generates: 1, e^-st cos/sin wt, e^-2st {1, cos/sin 2wt}.';
L{end+1} = '  The LAST SAMPLE is not a steady state at these damping ratios, and neither is the';
L{end+1} = '  centre of the ring -- the e^-2st columns are that offset, and dropping them cost 8%)';
L{end+1} = sprintf('  %-9s %10s %10s %10s %10s %9s %9s %6s', ...
    'alt [ft]', 'n_ss [g]', 'a_ss[deg]', 'a_pk[deg]', 'q_ss[d/s]', ...
    'wn meas', 'z meas', '#ext');
for i = 1:numel(R.fc)
    f = R.fc(i);
    L{end+1} = sprintf('  %-9g %10.4f %10.4f %10.4f %10.4f %9.4f %9.5f %6d', ...
        f.alt, f.resp.settled_accel_g, f.resp.settled_alpha_deg, ...
        f.resp.peak_alpha_deg, f.resp.settled_thd_dps, f.resp.wn, ...
        f.resp.zeta, f.resp.n_extrema); %#ok<AGROW>
    if ~f.resp.meas_ok
        L{end+1} = sprintf('      !! frequency/damping not measurable: %s', ...
            f.resp.meas_why); %#ok<AGROW>
    end
end
L{end+1} = sprintf(['  last sample vs settled, acceleration: %s g  ' ...
                    '-- the gap is the ring the book''s 2 s does not outlast'], ...
    numList(arrayfun(@(f) f.resp.final_accel_g - f.resp.settled_accel_g, R.fc), '%+.3f'));
% The superseded extremum kernel, printed beside the ring fit.  Two numbers on
% one line are the cheapest possible regression detector for this estimator:
% at sea level they agree to 0.02 g and at 50 kft they are 0.15 g apart, and
% THAT SPREAD IS THE DEFECT this file was red for.  If they ever converge the
% e^-2st columns have stopped doing anything.
L{end+1} = sprintf(['  ring fit vs the old (1/4,1/2,1/4) kernel, acceleration: %s g' ...
                    '  (%s)'], ...
    numList(arrayfun(@(f) f.resp.settled_accel_g - f.resp.kernel_accel_g, R.fc), '%+.3f'), ...
    strjoin(arrayfun(@(f) f.resp.settled_model, R.fc, 'UniformOutput', false), '/'));
L{end+1} = repmat('-', 1, W);

% -- self check
if isfield(R.selfcheck, 'ok')
    C = R.selfcheck;
    L{end+1} = 'STRUCTURAL SELF-CHECK  (wiring and algebra, not the book -- a failure voids everything)';
    L{end+1} = sprintf('  CM(alphaTR) == 0                          : rel %9.2e  tol %.0e  %s', ...
        C.trim_resid, C.trim_tol, tf2s(C.trim_ok));
    L{end+1} = sprintf('  dCM/dalpha at the root == sqrt(disc)      : rel %9.2e  tol %.0e  %s', ...
        C.slope_resid, C.slope_tol, tf2s(C.slope_ok));
    L{end+1} = sprintf('  Jacobian built term-by-term == y1+2y2a    : rel %9.2e  tol %.0e  %s', ...
        C.jac_resid, C.jac_tol, tf2s(C.jac_ok));
    L{end+1} = sprintf('  ODE settles on the quadratic''s root       : rel %9.2e  tol %.0e  %s', ...
        C.alpha_resid, C.alpha_tol, tf2s(C.alpha_ok));
    L{end+1} = sprintf(['      of which %s is the PREDICTED quadratic DC offset -beta*a^2/(2w^2), ' ...
                        'not error'], numList(C.alpha_dc_pred, '%.2e'));
    L{end+1} = sprintf('  nL_trim == K1*delta   (algebraic)         : rel %9.2e  tol %.0e  %s', ...
        C.k1_alg_resid, C.k_alg_tol, tf2s(C.k1_alg_resid <= C.k_alg_tol));
    L{end+1} = sprintf('  qdot_trim == K3*delta (algebraic)         : rel %9.2e  tol %.0e  %s', ...
        C.k3_alg_resid, C.k_alg_tol, tf2s(C.k3_alg_resid <= C.k_alg_tol));
    L{end+1} = sprintf(['      K1 FLOORS at %.2e (one rounded 1845 = 57.3*32.2 = 1844.66); K3 ' ...
                        'carries a second 1845 that divides it back out, so K3 is exact.'], ...
        abs(C.k_alg_expected));
    L{end+1} = sprintf('  that asymmetry holds (both 1845s present) : %s', tf2s(C.k_alg_asym_ok));
    L{end+1} = sprintf('  simulated settled == closed form          : rel %9.2e  tol %.0e  %s', ...
        C.sim_resid, C.sim_tol, tf2s(C.sim_ok));
    L{end+1} = sprintf('  qdot_ss == n_ss*g/VM  (alphadot = 0)      : rel %9.2e  tol %.0e  %s', ...
        C.chan_resid, C.chan_tol, tf2s(C.chan_ok));
    L{end+1} = sprintf('  Ta measured as alpha/gammadot == closed fm: rel %9.2e  tol %.0e  %s', ...
        C.ta_resid, C.ta_tol, tf2s(C.ta_ok));
    L{end+1} = sprintf(['      measured Ta = %s s   closed form = %s s   -- Zarchan''s own ' ...
                        'definition of Ta, zarchan.txt:5418'], ...
        numList(C.ta_meas, '%.4f'), numList(C.ta_form, '%.4f'));
    if C.qscale_skipped
        L{end+1} = '  Q-scaling across altitude                 : SKIP (needs >= 2 altitudes)';
    else
        L{end+1} = sprintf('  Q-scaling across altitude (11 quantities) : rel %9.2e  tol %.0e  %s', ...
            C.qscale_resid, C.qscale_tol, tf2s(C.qscale_ok));
    end
    L{end+1} = sprintf('  measured wn   vs JACOBIAN linearization   : rel %9.2e  tol %.0e  %s', ...
        C.wn_resid, C.wn_tol, tf2s(isfinite(C.wn_resid) && C.wn_resid <= C.wn_tol));
    L{end+1} = sprintf('  measured zeta vs JACOBIAN linearization   : rel %9.2e  tol %.0e  %s', ...
        C.ze_resid, C.ze_tol, tf2s(isfinite(C.ze_resid) && C.ze_resid <= C.ze_tol));
    L{end+1} = sprintf(['      Zarchan''s SECANT sits %+.2f%% (wn) and %+.1f%% (zeta) from the ' ...
                        'tangent; measured is nearer the TANGENT: %s / %s'], ...
        100*C.wn_secant_gap, 100*C.ze_secant_gap, tf2s(C.wn_nearer), tf2s(C.ze_nearer));
    L{end+1} = sprintf('  Heun anti-damping / physical damping      : %s  (errors above 5%%)', ...
        numList(100*arrayfun(@(f) f.antidamp_ratio, R.fc), '%.3f%%'));
else
    L{end+1} = 'STRUCTURAL SELF-CHECK  ***SKIPPED*** (benchmark=false) -- nothing here is verified';
end
L{end+1} = repmat('-', 1, W);

% -- the board
if isempty(R.bench)
    L{end+1} = 'BOOK BENCHMARKS  ***NOT RUN*** (benchmark=false).  R.pass is false because';
    L{end+1} = '  NOTHING WAS CHECKED, which is not the same thing as everything being fine.';
else
    L{end+1} = 'BOOK BENCHMARKS   Ch.21, pp.478-497.  Every book column is greppable prose;';
    L{end+1} = '  no value was taken off a figure image, because the figures did not survive extraction.';
    L{end+1} = sprintf('  %-44s %12s %12s %9s %-9s %s', ...
        'what', 'book', 'ours', 'diff', 'band', 'verdict');
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
        case 'value'
            if b.skipped
                L{end+1} = sprintf('  %-44s %12s %12s %9s %-9s SKIP', ...
                    trunc(b.what,44), [b.book_str ' ' b.units], '--', '--', ...
                    bandTag(b.band, b.tol)); %#ok<AGROW>
                L{end+1} = sprintf('      why: %s', b.why); %#ok<AGROW>
            else
                L{end+1} = sprintf('  %-44s %12s %12s %+8.1f%% %-9s %s', ...
                    trunc(b.what,44), [b.book_str ' ' b.units], ...
                    [b.ours_str ' ' b.units], b.pct, ...
                    bandTag(b.band, b.tol), tf2s(b.ok)); %#ok<AGROW>
            end
        case {'shape','bound'}
            L{end+1} = sprintf('  %-44s %12s %12s %9s %-9s %s', ...
                trunc(b.what,44), trunc(b.book_str,12), '(see below)', '--', ...
                upper(b.kind), tf2s(b.ok)); %#ok<AGROW>
            L{end+1} = sprintf('      book: %s     ours: %s', ...
                b.book_str, b.ours_str); %#ok<AGROW>
        case 'info'
            L{end+1} = sprintf('  %-44s %12s %12s %9s %-9s INFO', ...
                trunc(b.what,44), trunc(b.book_str,12), '(see below)', '--', 'info'); %#ok<AGROW>
            L{end+1} = sprintf('      book: %s     ours: %s', ...
                b.book_str, b.ours_str); %#ok<AGROW>
            L{end+1} = sprintf('      %s', b.why); %#ok<AGROW>
        case 'skip'
            L{end+1} = sprintf('  %-44s %12s %12s %9s %-9s SKIP', ...
                trunc(b.what,44), trunc(b.book_str,12), '--', '--', 'skip'); %#ok<AGROW>
            L{end+1} = sprintf('      why: %s', b.why); %#ok<AGROW>
        end
        L{end+1} = sprintf('      src: %s', b.source); %#ok<AGROW>
    end
    ev = ~[R.bench.skipped];
    L{end+1} = sprintf('  %d scored, %d passed, %d failed, %d info/skip.  R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(ev & ~[R.bench.ok]), ...
        sum(~ev), tf2s(R.pass));
    L{end+1} = '  bands: fig 15% = the author read it off a plot and quoted 1-2 figures.';
    L{end+1} = '  SHAPE/BOUND rows have no tolerance: they are exact orderings, and they are the';
    L{end+1} = '  rows that cannot be passed by accident.';
end
L{end+1} = repmat('-', 1, W);

% -- the lag bridge
G = R.lag;
L{end+1} = 'THE TWO TIME CONSTANTS THIS CHAPTER DERIVES, AND WHICH ONE IS COMPARABLE TO OURS';
L{end+1} = sprintf('  altitude [ft]                : %s', numList(G.alt, '%g'));
L{end+1} = sprintf('  airframe   1/wAF [s]         : %s   <- the BODY''s pitch response', ...
    numList(G.tau_airframe_s, '%.4f'));
L{end+1} = sprintf('  turning-rate Ta  [s]         : %s   <- the VELOCITY VECTOR''s lag', ...
    numList(G.ta_turnrate_s, '%.4f'));
L{end+1} = sprintf('  our measured gamma-track lag : %.1f s (gain 0.063 vs the model''s 1.385)', ...
    G.our_measured_s);
L{end+1} = '  READ IT THIS WAY.  The airframe constant is 2 orders of magnitude below our 8 s, so';
L{end+1} = '  our lag is NOT body dynamics -- no amount of airframe work can produce it.  Ta is the';
L{end+1} = '  same order as ours and is the quantity our 0.063 gamma-tracking gain measures.  Ta';
L{end+1} = '  scales as 1/Q, so it is an ALTITUDE and SPEED problem, not a gain-tuning problem.';
L{end+1} = repmat('-', 1, W);

L{end+1} = 'READ THIS BEFORE QUOTING ANY NUMBER ABOVE';
L{end+1} = '  This is ONE airframe: 1000 lb, 20 ft, 1 ft diameter, tail controlled, cruciform,';
L{end+1} = '  at Mach 3 with a FLAT 1000 ft/s speed of sound and supersonic linear-theory aero.';
L{end+1} = '  Constant speed, constant altitude, no gravity, no thrust, no actuator, no autopilot,';
L{end+1} = '  no guidance, no fin travel limit, pitch plane only.  There is no miss distance here';
L{end+1} = '  and none can be computed from it.  Our vehicle is SUBSONIC and this aero model has';
L{end+1} = '  no subsonic branch -- it errors out below Mach 1 rather than returning the complex';
L{end+1} = '  numbers B = sqrt(M^2-1) would otherwise hand back.  What transfers is the STRUCTURE';
L{end+1} = '  (a flight-path lag is derivable, scales as 1/Q, and is separable from body lag),';
L{end+1} = '  not any value in the tables above.';
s = strjoin(L, newline);
end


% =============================================================================
function drawCurves(R, cfg)
%DRAWCURVES  Figs. 21.4-21.7: angle of attack and acceleration vs time.
for i = 1:numel(R.fc)
    f = R.fc(i);
    figure('Name', sprintf('Zarchan Ch.21 -- airframe at %g ft', f.alt));

    subplot(2,1,1);
    plot(f.sim.t, f.sim.alpha_deg, 'LineWidth', 1.1); grid on; hold on;
    % The trim line is the algebraic root, drawn so the eye can check the
    % same thing the structural row checks numerically.
    plot([f.sim.t(1) f.sim.t(end)], f.trim.alftr_deg*[1 1], '--');
    xlabel('Time (Sec)'); ylabel('Angle of Attack (Deg)');
    title(sprintf('%g-deg fin, %g ft/s, %g ft   (Figs. 21.4 / 21.6)', ...
        cfg.del_deg, cfg.vm, f.alt));
    legend('nonlinear', 'analytic trim', 'Location', 'best');

    subplot(2,1,2);
    plot(f.sim.t, f.sim.accel_g, 'LineWidth', 1.1); grid on; hold on;
    plot([f.sim.t(1) f.sim.t(end)], f.lin.nl_ss_g*[1 1], '--');
    xlabel('Time (Sec)'); ylabel('Missile Acceleration (G)');
    legend('nonlinear', 'K_1\delta', 'Location', 'best');
end
end
