function R = accelMissAdjoint(opts)
%MGS.ZARCHAN.ACCELMISSADJOINT  Zarchan Listing 5.3 -- acceleration AND miss
%   distance adjoint of the single-lag proportional-navigation homing loop.
%
%   R = mgs.zarchan.accelMissAdjoint()                    % Fig. 5.8: tgo = 0.5, 1, 2 s
%   R = mgs.zarchan.accelMissAdjoint('mode','miss')       % Figs. 4.18 / 5.5 miss curve
%   R = mgs.zarchan.accelMissAdjoint('tgo',[0.25 0.5 1])  % other observation times
%   R = mgs.zarchan.accelMissAdjoint('render',false,'assert',false)
%
%   Line-by-line port of Listing 5.3 (zarchan.txt:1584), the adjoint of the
%   Fig. 5.7 block diagram, integrated by the book's own two-pass predictor/
%   corrector.  The error source is a 3-g step target maneuver whose STARTING
%   TIME is uniformly distributed over the flight (Ch.4, :1263), so every
%   number this file produces is a STANDARD DEVIATION, not a single-shot value.
%
% ===========================================================================
%   *** UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, g = 32.2.  NOT SI. ***
%
%   The rest of mgs is metres.  This file is deliberately NOT converted so
%   that every number it prints can be laid straight against a published one
%   with no factor floating around to be got wrong.  A silent ft->m here would
%   turn 13.4 ft of miss into "4.1", which is still a perfectly plausible miss
%   distance -- precisely the failure this package exists to prevent.  If you
%   need metres, convert AT THE CALL SITE and say so there.
%
%   The two channels carry DIFFERENT units, and the listing itself is where
%   the difference is made ("if MISS==0, XMUDNT=XMUDNT/32.2"):
%       mode 'miss'   sigma of MISS DISTANCE          [ft]
%       mode 'accel'  sigma of MISSILE ACCELERATION   [g]
%   R.curve(k).units carries the string; do not assume one of them.
% ===========================================================================
%
%   WHY THIS PORT EXISTS: IT IS ONE HALF OF A CROSS-CHECK
%
%   Chapter 5 computes the same two quantities twice, by two methods that
%   share no machinery -- covariance analysis (Listing 5.2: forward, RK4,
%   propagating a 4x4 covariance matrix) and this adjoint (backward, RK2,
%   four states plus a squarer/integrator).  The book asserts they agree, in
%   these words:
%
%     ":1536  the standard deviation of the miss distance is 13.3 ft, which is
%             identical to the adjoint results of Chapter 4 (see Fig. 4.18)"
%     ":1583  (12 g, 9 g, and 6.1 g) respectively agree exactly with the
%             single run covariance analysis results of Fig. 5.6"
%     ":1556  the covariance analysis technique can be used to provide an
%             independent check of the accuracy of an adjoint simulation"
%     ":1548  Fig. 5.5 Covariance analysis miss distance results agree with
%             adjoint"        ":1579  Fig. 5.8 Adjoint and covariance analysis
%             acceleration results agree"
%
%   So porting BOTH gives us an agreement test that owes nothing to either
%   implementation.  What the covariance port has to produce to complete it,
%   at XNT=96.6, XNP=3, TAU=1, VC=4000, TF=10 (Listing 5.2's own header line):
%
%       sqrt(X(1,1))  at t = tF = 10 s        <->  our 'miss'  curve at tF = 10
%       sqrt(A*X*A')  at t = 9.5, 9.0, 8.0 s  <->  our 'accel' curves at
%                                                  tgo = 0.5, 1, 2, read at
%                                                  tF = 10
%   (t_obs = tF - tgo; that mapping is the whole content of ":1577".)
%
%   EXPECT AGREEMENT TO ABOUT 1.5%, NOT TO MACHINE PRECISION, AND EXPECT THE
%   ADJOINT TO READ HIGH.  This listing is second-order Runge-Kutta against a
%   1/tgo singularity at the start of the run; measured against the analytic
%   closed form below it sits +1.43% high at H = 0.01 (SELF-CHECK 1).  The
%   covariance program uses RK4 and should land essentially on the analytic
%   value.  If the two ports ever agree to 1e-12, something is wrong: they are
%   not supposed to be the same computation.
%
% ---------------------------------------------------------------------------
%   THE TWO MODES, AND WHY tgo = 0 IS AN ERROR IN ONE OF THEM
%
%   ":1577  In this program TINT represents the time to go in the original
%           system in which we wish to observe the quantity of interest.  If we
%           want to compute miss distance, MISS should be set to 1 and TINT set
%           to zero.  For an acceleration adjoint, MISS should be set to 0 and
%           TINT set to a number representing the time to go at which we want
%           to observe the acceleration."
%
%   TINT is 'tgo' here.  In an adjoint, ADJOINT TIME IS TIME-TO-GO, so an
%   observation wanted tgo seconds before intercept becomes an initial
%   condition applied at ADJOINT time tgo, and the run starts there:
%
%       mode 'miss'    tgo = 0,  x3(0)   = 1,             x4(0)   = 0
%       mode 'accel'   tgo > 0,  x3(tgo) = N'/(TAU*tgo),  x4(tgo) = -1/TAU
%
%   The acceleration ICs are the observation weights of nL = A*x on the two
%   states that carry it (:1552, A = [N'/(T*tgo), 0, 0, -N'*Vc/T]); the Vc in
%   A is absorbed by this adjoint's state scaling, which is why -- unlike the
%   covariance listing -- Vc appears nowhere in this file.  There is nothing
%   to check it against: do not "restore" a Vc here.
%
%   *** tgo = 0 IN 'accel' MODE IS REFUSED. ***  x3(0) = N'/(TAU*0) is Inf in
%   MATLAB, not an error; the run then propagates Inf and NaN, sqrt(NaN) is
%   NaN, and the caller gets a full-length curve of NaNs that plots as an empty
%   axes and reads as "nothing happened".  So it is checked up front.
%
%   The related trap is the book's own 15-g figure (":1556  68% of the time
%   15 g (that is, 483 ft/s2) ... is required to avoid acceleration
%   saturation").  That is Fig. 5.6 read at the END of the flight, i.e. at
%   tgo -> 0, and IT IS NOT A BENCHMARK FOR THIS FILE at any stated tgo.  This
%   adjoint DIVERGES as tgo -> 0 -- measured 12.0 g at tgo = 0.5, 15.9 at 0.25,
%   24.8 at 0.1, 38.4 at 0.05 -- so "15 g" happens to correspond to tgo ~ 0.29
%   s, an abscissa the book never states.  Adding a 15-g row to the table below
%   would be scoring a figure read-off against a made-up time-to-go, which is
%   exactly how the previous wave of ports acquired three benchmark values that
%   do not exist.  Do not add it.
%
% ---------------------------------------------------------------------------
%   WHERE EVERY BOOK NUMBER IN THIS FILE COMES FROM.  All of them are greppable
%   in matlab/zarchan/_source/zarchan.txt; none is read off a figure by us,
%   because the extraction turned every figure into an image link and no number
%   that lives only on a plot survived into the text at all.
%
%     :1583  "(12 g, 9 g, and 6.1 g)" for a 10-s flight at tgo = 0.5, 1, 2
%     :1578  "12 g at 0.5 s to go for a 10-s flight, approximately 12 g at
%             0.5 s to go for a 6-s flight, and approximately 4 g for a 2-s
%             flight"    -- the two "approximately" values get a wider band
%     :1536  "the standard deviation of the miss distance is 13.3 ft"
%     :1355  the closed form for the uniformly distributed maneuver at N' = 3
%     :1345  MNT|N'=3 = 0.5*tF^2*exp(-tF/T), the step-maneuver sensitivity that
%            closed form is built from
%     :1556  "monotonically increasing for the 10-s flight"  -> an ORDERING
%            check across tgo, which needs no number at all
%     :1284  the miss curve is "small for both small and large flight times"
%            -> an interior-maximum check
%
% ---------------------------------------------------------------------------
%   WHAT IS CHECKED, AND THE ONE THING THAT IS NOT
%
%   SELF-CHECK, on the CALLER's parameters, so it still bites off-book:
%     1  the miss channel against an ANALYTIC closed form.  For N' = 3 the book
%        prints it outright at :1355; for any integer N' >= 3 this file rebuilds
%        it by the same route (cfSigma below) and the two agree to 2.1e-5.
%        Nothing in it touches the integrator.
%     2  time-scale invariance.  Scale TAU, TF, tgo, H and samp together: the
%        'accel' curve must be UNCHANGED and the 'miss' curve must scale by
%        TAU^2.  This is the only check that can see a dropped or doubled TAU
%        inside the acceleration initial conditions.
%
%   BOOK BENCHMARKS: always run at the BOOK's own parameters, whatever the
%   caller passed, so the table is an audit of the TRANSCRIPTION and is always
%   evaluable.  A table of SKIP rows reads as a pass to everyone who scrolls
%   past it, so there are none.
%
%   *** THE GAP, STATED PLAINLY. ***  The two modes share one ODE, so
%   self-check 1 validates the wiring for both.  What it CANNOT see is the pair
%   of acceleration initial conditions x3 = N'/(TAU*tgo), x4 = -1/TAU: the
%   closed form is for the miss channel, and the book publishes no closed form
%   for the acceleration channel.  Those two numbers are covered by exactly two
%   things -- the three published g-values, and self-check 2.  Change them and
%   the g rows are all that stands between you and a smooth, plausible, wrong
%   curve.
%
% ---------------------------------------------------------------------------
%   *** THIS IS NOT A MODEL OF OUR VEHICLE. ***
%   Linearised, planar, constant closing speed, FIRST-ORDER lag, and the only
%   disturbance is a target maneuver at a random time.  Our bullseye is
%   stationary, our measured discriminant is release-side heading error |eps0|,
%   and our lag SATURATES (measured gamma-tracking gain 0.063 against the
%   model's 1.385, ~8 s of it, saturating on 26 of 32 sorties).  Read a number
%   out of here as "what a clean single-lag loop would do", never as a
%   prediction about a sortie.
%
% ---------------------------------------------------------------------------
%   OUTPUT
%
%   R.mode         'accel' | 'miss'
%   R.tgo          1xK observation times-to-go actually run [s]
%   R.curve(k)     .tgo  .t  .sigma  .units  .label
%                  .t is ADJOINT time, readable as the FLIGHT TIME tF of the
%                  original system (":1578  We can interpret the abscissa of
%                  the plot as either adjoint time or flight time").
%                  .units is 'ft' in miss mode and 'g' in accel mode -- READ IT
%                  rather than assuming, the two differ by a factor of 32.2.
%   R.read         @(tgo,tF) -> sigma, at the nearest recorded sample.  This is
%                  the handle the covariance cross-check should call, so nobody
%                  re-derives the tF <-> tgo mapping at the call site.  It
%                  ERRORS on an unrun tgo or an out-of-range tF instead of
%                  returning NaN: a cross-check that compares NaN to a number
%                  reports agreement.
%   R.selfcheck    residuals and tolerances of the structural checks, plus the
%                  reference miss run (.miss_t .miss_sigma .miss_cf)
%   R.bench        struct array, computed vs book.  Its fields are the ones
%                  mgs.zarchan.benchSuite's adapters already read off the other
%                  adjoint ports -- .what .book .ours .band .tol .ok .skipped
%                  .why -- plus .unit, .t_read, and (unlike them) .source,
%                  which carries the zarchan.txt line the book value came from.
%                  An adapter should take the citation from .source rather than
%                  from benchSuite's cite() map: a map keyed on row NAME goes
%                  stale silently the first time a row is renamed.
%   R.shape        the claims the book states in prose:
%                  .what .claim .source .ok .skipped .detail.  No .book, so an
%                  adapter should emit these as 'shape only' rows.
%   R.pass         true iff every scored bench row and every evaluated shape
%                  check passed.  The self-check is NOT part of it: that one
%                  ERRORS instead of returning false, so reaching R.pass at all
%                  already means it held.
%   R.summary      the printed text
%
% ---------------------------------------------------------------------------
%   OCR DAMAGE RESOLVED IN THE TRANSCRIPTION (listing_05_03.m header lists what
%   the extractor could not decide).  In this listing "<" was OCR'd as "," and
%   ">" as "."; the extractor repaired the two it could see from shape and left
%   one:
%
%     "while TP,=(TF - 1e-5)"  ->  while TP <= (TF - 1e-5)
%
%   Resolved from context, three ways, all agreeing: TP starts at TINT+1e-5 and
%   is INCREASED by H, so only "<=" terminates; Listing 4.6 (:1329) is the same
%   loop with the same OCR and reads "while TP , = (TF - 1e-5)"; and Listings
%   3.1 and 6.1 were resolved identically in the two sibling ports.  The other
%   two, "while STEP,=1" -> "STEP <= 1" and "if S . =.099999" -> "S >= .099999",
%   were already repaired by the extractor.
%
%   Everything after the "%figure plot(...)" line of the transcription is
%   Chapter 5's SUMMARY and REFERENCES prose that leaked into the code block.
%   It is dropped, not translated.  The book's own "save datfil.txt output
%   /ascii" is dropped too: a port that writes a file into the caller's working
%   directory as a side effect of being called is a booby trap.
%
%   See also MGS.ZARCHAN.ADJOINTSINGLELAG, MGS.ZARCHAN.ADJOINTBINOMIAL5,
%   MGS.ZARCHAN.BENCHSUITE.

arguments
    % Book names, book case (Listing 5.3, line 1).  The UPPERCASE ones are the
    % listing's own variables, spelled exactly as the book spells them -- the
    % convention adjointSingleLag.m uses for the sibling listing.
    opts.mode  (1,1) string {mustBeMember(opts.mode, ["accel","miss"])} = "accel"
    % NOT declared (1,:): the default has to be a sentinel meaning "the book's
    % set for this mode", and MATLAB's own validator REJECTS "(1,:) double = []"
    % because [] is 0x0 -- the same trap benchSuite.m documents for its 'only'
    % option.  Shape is checked in the body, where the message can say what
    % actually went wrong.
    opts.tgo         double = []       % TINT [s]; [] -> [0.5 1 2] accel, 0 miss
    opts.XNT   (1,1) double {mustBeFinite, mustBePositive} = 96.6  % ft/s^2 = 3 g
    opts.XNP   (1,1) double {mustBeFinite, mustBePositive} = 3.0   % N', dimensionless
    opts.TAU   (1,1) double {mustBeFinite, mustBePositive} = 1.0   % s
    opts.TF    (1,1) double {mustBeFinite, mustBePositive} = 10.0  % s
    opts.H     (1,1) double {mustBeFinite, mustBePositive} = 0.01  % s, RK2 step
    opts.samp  (1,1) double {mustBeFinite, mustBePositive} = 0.1   % s, record every
    % *** THESE FLAGS ARE THE ONLY SWITCHES.  There is deliberately no
    % "if nargin == 0" anywhere in this file: in a function whose arguments
    % block declares ONLY name-value options, MATLAB reports nargin = 0 no
    % matter how many were passed (measured R2025b), so such a test is ALWAYS
    % TRUE.  Three ports in the last wave used one; one became infinite
    % recursion and two silently replaced the caller's arguments with the
    % benchmark run.
    opts.benchmark (1,1) logical = true
    opts.selfcheck (1,1) logical = true
    opts.assert    (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
end

GRAV_FT = 32.2;                    % g in the book's units.  XNT = 96.6 = 3 g.
isMiss  = (opts.mode == "miss");

% ---- resolve and police tgo ------------------------------------------------
tgo = opts.tgo;
if isempty(tgo)
    if isMiss, tgo = 0; else, tgo = [0.5 1 2]; end     % the book's own sets
end
if ~isvector(tgo) || ~isreal(tgo) || any(~isfinite(tgo))
    error('mgs:zarchan:accelMissAdjoint:tgoShape', ...
        ['''tgo'' must be a real finite vector of times-to-go in SECONDS; got a ' ...
         '%s of size %s.'], class(tgo), mat2str(size(tgo)));
end
tgo = reshape(tgo, 1, []);
if numel(unique(tgo)) ~= numel(tgo)
    error('mgs:zarchan:accelMissAdjoint:tgoDup', ...
        ['''tgo'' repeats a value (%s).  R.read indexes curves BY tgo, so a ' ...
         'duplicate would make one of them permanently unreachable.'], mat2str(tgo));
end

if isMiss
    % The book's rule, :1577.  A miss adjoint with TINT ~= 0 still RUNS -- it
    % just applies the unit impulse late, which slides the whole curve along an
    % axis labelled "flight time" and answers a question nobody asked.  No
    % symptom: the curve stays smooth and the right order of magnitude.
    if ~isscalar(tgo) || tgo ~= 0
        error('mgs:zarchan:accelMissAdjoint:missTgo', ...
            ['mode ''miss'' requires tgo = 0 (Zarchan :1577: "If we want to compute ' ...
             'miss distance, MISS should be set to 1 and TINT set to zero"); got %s. ' ...
             'A nonzero tgo does not fail inside the loop -- it silently returns a ' ...
             'time-shifted curve that still looks like a miss curve.  If you wanted ' ...
             'acceleration at %g s to go, pass ''mode'',''accel''.'], ...
            mat2str(tgo), tgo(1));
    end
else
    if any(tgo <= 0)
        error('mgs:zarchan:accelMissAdjoint:accelTgo', ...
            ['mode ''accel'' needs every tgo > 0; got %s.  The acceleration initial ' ...
             'condition is x3 = N''/(TAU*tgo), so tgo = 0 evaluates to Inf -- MATLAB ' ...
             'does not error, it propagates Inf then NaN and hands back a ' ...
             'full-length curve of NaNs.  There is no tgo -> 0 limit to take: this ' ...
             'adjoint DIVERGES there (12.0 g at 0.5 s, 24.8 at 0.1, 38.4 at 0.05). ' ...
             'For miss distance pass ''mode'',''miss''.'], mat2str(tgo));
    end
    if any(tgo >= opts.TF - opts.samp)
        error('mgs:zarchan:accelMissAdjoint:tgoHorizon', ...
            ['tgo = %g s leaves less than one sample interval before TF = %g s, so ' ...
             'that run would record no samples and return an EMPTY curve -- which ' ...
             'plots as nothing and averages to NaN.'], max(tgo), opts.TF);
    end
end

% ---- step-size guards (same reasoning as adjointSingleLag.m) ---------------
if opts.H > opts.samp
    error('mgs:zarchan:accelMissAdjoint:step', ...
        ['integration step H = %g s is larger than the sample interval samp = %g s, ' ...
         'so the curve would be sampled at the step rate and silently disagree with ' ...
         'its own time vector.'], opts.H, opts.samp);
end
if opts.H >= opts.TAU
    error('mgs:zarchan:accelMissAdjoint:stiff', ...
        ['H = %g s is not smaller than the guidance time constant TAU = %g s.  The ' ...
         'lag state would be integrated at or below one sample per time constant; ' ...
         'RK2 does not blow up there, it returns a smooth and entirely wrong curve, ' ...
         'so this is refused rather than warned about.'], opts.H, opts.TAU);
end
rr = rem(opts.samp, opts.H);
if abs(rr) > 1e-12 && abs(rr - opts.H) > 1e-12
    % Not fatal: the integration is unaffected, only the instants at which it is
    % recorded drift.  Said out loud because someone comparing two runs sample
    % by sample would otherwise blame the drift on the physics.
    warning('mgs:zarchan:accelMissAdjoint:sampleGrid', ...
        ['samp = %g is not a whole number of steps of H = %g, so recording instants ' ...
         'drift by up to one step.  Curve VALUES are unaffected.'], opts.samp, opts.H);
end

% ---- the runs the caller asked for ----------------------------------------
R = struct();
R.listing = 'Zarchan Listing 5.3 (zarchan.txt:1584), adjoint of Fig. 5.7';
R.units   = 'feet, ft/s, ft/s^2, g = 32.2 -- Zarchan''s units, NOT SI';
R.mode    = char(opts.mode);
R.opts    = opts;
R.tgo     = tgo;

for k = 1:numel(tgo)
    A = adjointRun(isMiss, tgo(k), opts.XNT, opts.XNP, opts.TAU, opts.TF, ...
                   opts.H, opts.samp, GRAV_FT);
    R.curve(k).tgo   = tgo(k);
    R.curve(k).t     = A.t;
    R.curve(k).sigma = A.sigma;
    if isMiss
        R.curve(k).units = 'ft';
        R.curve(k).label = 'sigma miss distance';
    else
        R.curve(k).units = 'g';
        R.curve(k).label = sprintf('sigma missile acceleration, tgo = %g s', tgo(k));
    end
end
C = R.curve;                       % captured BY VALUE for the R.read closure
R.read = @(tgoWant, tfWant) readCurve(C, tgoWant, tfWant, opts.samp);

% ---- structural self-check ------------------------------------------------
if opts.selfcheck
    R.selfcheck = structuralCheck(opts, tgo, isMiss, GRAV_FT);
else
    R.selfcheck = struct('skipped', true);
end

% ---- shape claims the book states in prose --------------------------------
R.shape = shapeChecks(R, opts, isMiss);

% ---- book benchmarks ------------------------------------------------------
if opts.benchmark
    R.bench = benchmarks(GRAV_FT);
else
    R.bench = emptyBench();
end

scored = ~[R.bench.skipped];
R.pass = all([R.bench(scored).ok]) && all([R.shape.ok] | [R.shape.skipped]);

R.summary = render(R, opts, isMiss);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R, opts, isMiss); end

if opts.assert
    % "Nothing was evaluated" must never read as "everything passed".  That is
    % the shape of green light this repo has been burned by more than once.
    if ~opts.benchmark && ~opts.selfcheck
        error('mgs:zarchan:accelMissAdjoint:noCheck', ...
            ['both ''benchmark'' and ''selfcheck'' are false, so this run verified ' ...
             'NOTHING and the curve it is about to return is unaudited.  Pass ' ...
             '''assert'',false if that is genuinely what you want.']);
    end
    if ~R.pass
        bad = find(~[R.bench.ok] & scored);
        if isempty(bad)
            error('mgs:zarchan:accelMissAdjoint:shape', ...
                ['a SHAPE claim the book states in prose does not hold for this run. ' ...
                 'Shape checks need no published number and are the part of this ' ...
                 'audit that survives off-book parameters.  Full table:\n\n%s'], ...
                R.summary);
        end
        b = R.bench(bad(1));
        error('mgs:zarchan:accelMissAdjoint:benchmark', ...
            ['%d of %d book benchmarks are outside tolerance (first: %s, book %.4g ' ...
             '%s, ours %.4g, band %s %.0f%%).  The benchmark runs the BOOK''s ' ...
             'parameters, not yours, so a red row means the TRANSCRIPTION is wrong ' ...
             '-- it is not a symptom of the arguments you passed.  Full table:\n\n%s'], ...
            numel(bad), sum(scored), b.what, b.book, b.unit, b.ours, b.band, ...
            100*b.tol, R.summary);
    end
end
end


% =============================================================================
function A = adjointRun(isMiss, TINT, XNT, XNP, TAU, TF, H, samp, GRAV_FT)
%ADJOINTRUN  Listing 5.3 itself.  Book variable names kept verbatim.
%
%   Fig. 5.7 states.  Apart from x1 none of them has a physical meaning; do not
%   try to interpret x3 or x4 as anything:
%     x1  <- int x2        the quantity being accumulated  (adjoint of nT input)
%     x2  <- int x3
%     x3  carries the impulse / observation weight, driven by N'*y1/tgo
%     x4  the single guidance lag
%     x5  <- int x1^2      the shaping-filter integrator (Ch.4, :1339)
%
%   x5 IS WHAT MAKES THIS A STOCHASTIC ADJOINT.  A step maneuver with a start
%   time uniformly distributed over the flight is equivalent to white noise of
%   spectral density nT^2/tF through an integrator (:1263), so the standard
%   deviation of the output is
%
%       sigma = nT * sqrt( (1/tF) * int_0^tF x1^2 dt )  =  XNT*sqrt(X5/TGO)
%
%   which is the book's output line exactly.  TGO there is the CURRENT adjoint
%   time, i.e. the flight time tF of the original system being reported on --
%   NOT the observation time-to-go TINT.  Reading it as TINT gives a curve that
%   is wrong by a factor of sqrt(tF/TINT) and still looks completely ordinary.
%
%   Two things in here look like bugs and are not:
%     * TP starts at +1e-5 and TGO carries a second +1e-5.  The adjoint gain is
%       N'/tgo with tgo the adjoint time, so an exact zero divides by zero on
%       the first derivative evaluation.  The offset is small enough not to move
%       the published numbers and is part of what reproduces them.
%     * In 'accel' mode the run STARTS at adjoint time TINT rather than 0.  That
%       is not a truncation: x1 is identically zero before TINT (a maneuver
%       cannot affect an acceleration that has already been observed), so the
%       integral from TINT IS the whole integral.
%
%   Do not replace this with ode45.  The published numbers are the numbers this
%   scheme at this step size produces, and landing on them is the entire point.

TP = 0.00001 + TINT;                 % TP=T+.00001+TINT, with the book's T=0
X1 = 0.0;  X2 = 0.0;  X5 = 0.0;
if isMiss
    X3 = 1.0;                        % <<< the unit impulse: this is the miss adjoint
    X4 = 0.0;
else
    X3 = XNP/(TAU*TINT);             % <<< the nL observation weights (see header)
    X4 = -1.0/TAU;
end
S = 0.0;

% Derivatives initialised before the loop so the FLAG==1 branch cannot read a
% stale value left behind by anything else.
X1D = 0.0;  X2D = 0.0;  X3D = 0.0;  X4D = 0.0;  X5D = 0.0;
TGO = TP + 0.00001;

nmax = ceil((TF - TINT)/samp) + 4;
tA   = zeros(1, nmax);
yA   = zeros(1, nmax);
n    = 0;

while TP <= (TF - 1e-5)              % OCR: "while TP,=(TF - 1e-5)"; see header
    X1OLD = X1;  X2OLD = X2;  X3OLD = X3;  X4OLD = X4;  X5OLD = X5;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  Pass 1 evaluates the derivatives where
    % we are; pass 2 takes a full Euler step, advances TP by H, and re-evaluates
    % there.  TP IS advanced inside this loop, exactly once, and is NOT rolled
    % back -- so the 1/tgo gain is evaluated at both times.  Move that line out
    % and the scheme quietly degrades to first order while still running.
    while STEP <= 1
        if FLAG == 1
            X1 = X1 + H*X1D;  X2 = X2 + H*X2D;  X3 = X3 + H*X3D;
            X4 = X4 + H*X4D;  X5 = X5 + H*X5D;
            TP = TP + H;
            STEP = 2;
        end
        X1D = X2;
        X2D = X3;
        Y1  = (X4 - X2)/TAU;         % the single lag
        TGO = TP + 0.00001;          % adjoint time IS time-to-go
        X3D = XNP*Y1/TGO;            % the time-varying PN gain N'/tgo
        X4D = -Y1;
        X5D = X1*X1;                 % the squarer feeding the shaping filter
        FLAG = 1;
    end
    FLAG = 0; %#ok<NASGU>  kept so the transcription reads like the listing

    % Heun corrector: (x_old + x_pred)/2 + (H/2)*f(t+H, x_pred), which equals
    % x_old + (H/2)*(f_old + f_pred).  X1D..X5D here are the SECOND-pass
    % derivatives; averaging with X1OLD is what puts the first pass back in.
    X1 = (X1OLD + X1)/2 + 0.5*H*X1D;
    X2 = (X2OLD + X2)/2 + 0.5*H*X2D;
    X3 = (X3OLD + X3)/2 + 0.5*H*X3D;
    X4 = (X4OLD + X4)/2 + 0.5*H*X4D;
    X5 = (X5OLD + X5)/2 + 0.5*H*X5D;

    % The book writes "if S >= .099999", a threshold hard-wired to H = 0.01 and
    % a 0.1 s sample interval that mis-samples at any other step size.  The form
    % below is BIT-IDENTICAL at the book's values and stays correct elsewhere.
    S = S + H;
    if S >= samp - 0.5*H
        S = 0.0;
        XMUDNT = XNT*sqrt(X5/TGO);
        if ~isMiss
            XMUDNT = XMUDNT/GRAV_FT;      % ft/s^2 -> g, the book's "if MISS==0"
        end
        n = n + 1;
        tA(n) = TP;
        yA(n) = XMUDNT;
    end
end

if n < 2
    error('mgs:zarchan:accelMissAdjoint:noSamples', ...
        ['the adjoint recorded %d samples (TF = %g, tgo = %g, samp = %g).  A curve ' ...
         'cannot be built from that, and returning it would hand back something ' ...
         'that plots as nothing at all.'], n, TF, TINT, samp);
end
A.t     = tA(1:n);
A.sigma = yA(1:n);
end


% =============================================================================
function y = stepMissKernel(t, TAU, Np)
%STEPMISSKERNEL  MNT(t), the miss sensitivity to a STEP target maneuver, in
%   closed form.  Analytic: it shares no code with the integrator.
%
%   From 1 - H(s) = [s/(s + 1/T)]^N' (p.52) and "miss is the maneuver after
%   three integrations", MNT/nT = (1-H)/s^3.  Expand the numerator about the
%   pole, s^p = ((s+a) - a)^p with a = 1/T and p = N'-3, and invert term by
%   term.  EXACT for integer N' -- the sum is finite, nothing is truncated.
%
%   At N' = 3 this collapses to 0.5*t^2*exp(-t/T), which is what the book
%   prints at :1345.  That identity is the evidence this routine is the book's
%   and not something invented here; it is asserted numerically in cfSigma's
%   caller (SELF-CHECK 1b) rather than merely claimed in a comment.
%
%   INTEGER N' ONLY.  For a fractional N' the binomial is an infinite series
%   and this would silently return a truncation of it, which is not a closed
%   form and must not be printed beside the words "closed form".  The caller
%   gates on integrality; do not remove that gate.
a = 1.0/TAU;
p = Np - 3;
if p < 0 || abs(p - round(p)) > 0
    error('mgs:zarchan:accelMissAdjoint:cfOrder', ...
        'closed form needs an INTEGER N'' >= 3; got %g.', Np);
end
y = zeros(size(t));
for k = 0:p
    % nchoosek and factorial are base MATLAB -- no toolbox is pulled in here.
    y = y + nchoosek(p,k) * (-a)^k * t.^(2+k) / factorial(2+k);
end
y = y .* exp(-a*t);
end


% =============================================================================
function s = cfSigma(t, TAU, Np, XNT, hFine)
%CFSIGMA  The analytic answer for the MISS channel, at the sample times t.
%
%       sigma(tF) = nT * sqrt( (1/tF) * int_0^tF MNT(u)^2 du )        (:1339)
%
%   with MNT from stepMissKernel.  The inner integral is done by cumulative
%   trapezoid on a FINE grid (hFine <= 5e-4 s) of the ANALYTIC integrand, then
%   interpolated to t -- so the only numerical content here is a quadrature of
%   a smooth function, not an ODE solve.  At N' = 3 it reproduces the closed
%   form the book prints at :1355 to 2.1e-5 relative, which is the quadrature
%   error and nothing else -- it falls as hFine^2, so it is the grid and not a
%   disagreement about what the formula says.
%
%   cumtrapz and interp1 are both base MATLAB; nothing here needs a toolbox.
nF = ceil(max(t)/hFine) + 2;
u  = (0:nF-1) * hFine;
f  = stepMissKernel(u, TAU, Np).^2;
F  = cumtrapz(u, f);
Fi = interp1(u, F, t, 'linear');
s  = XNT * sqrt(Fi ./ t);
end


% =============================================================================
function s = cfSigmaPublished(t, TAU, XNT)
%CFSIGMAPUBLISHED  The book's own closed form, :1355, valid at N' = 3 ONLY:
%
%     MUDNT|N'=3 = (nT/4) * sqrt( (T^5/tF) * [3 - exp(-2x)*(2x^4 + 4x^3 + 6x^2
%                                                            + 6x + 3)] )
%     x = tF/T
%
%   Two OCR repairs, both forced by the ligature map and both confirmed by
%   re-deriving the integral by hand: "e2x" is exp(-2x) (the superscript minus
%   was lost), and the character before it is a minus, not a space.  The
%   re-derivation: int_0^X u^4 exp(-2u/T) du = (3/4)T^5*[1 - exp(-2x)*(1 + 2x +
%   2x^2 + (4/3)x^3 + (2/3)x^4)], times the 0.25 from MNT^2 and the 1/tF, gives
%   exactly the bracket above once 3/16 is factored out.  Both readings had to
%   be right for that to close.
%
%   *** NUMERICALLY UNSTABLE FOR SMALL x. ***  The bracket is 3 - (something
%   that tends to 3): at x = 0.1 it is ~6.7e-6, so it loses about 11 of the 16
%   available digits, and by x = 0.001 it is pure rounding noise.  That is a
%   property of the printed form, not of the physics.  CFSIGMA is stable
%   everywhere and is what the self-check actually compares against; this
%   routine is only used to prove CFSIGMA is the book's formula, and only over
%   x >= 0.1.
x = t ./ TAU;
b = 3 - exp(-2*x) .* (2*x.^4 + 4*x.^3 + 6*x.^2 + 6*x + 3);
s = (XNT/4) .* sqrt( (TAU^5 ./ t) .* b );
end


% =============================================================================
function [v, tAct] = nearestSample(t, y, want, samp)
%NEARESTSAMPLE  Value at the sample nearest 'want', or NaN if none is within
%   half a sample interval.  NEVER extrapolates and never interpolates: the
%   benchmark rows must read a number this integrator actually produced.
%   NaN, not the nearest edge value, because NaN fails every tolerance test
%   loudly whereas an edge value passes some of them quietly.
[d, j] = min(abs(t - want));
if d > 0.5005*samp                 % 0.5005, not 0.5: a sample sits at
    v = NaN;  tAct = NaN;          % tgo + k*samp + 1e-5, so an exactly
    return;                        % half-interval offset must not fall out
end
v    = y(j);
tAct = t(j);
end


% =============================================================================
function v = readCurve(C, tgoWant, tfWant, samp)
%READCURVE  R.read: sigma at (tgo, flight time).  The handle the covariance
%   cross-check should call, so that nobody has to re-derive the tF <-> tgo
%   mapping at the call site.  ERRORS rather than returning NaN, because a
%   cross-check that silently compares NaN to a number reports agreement.
k = find(abs([C.tgo] - tgoWant) < 1e-12, 1);
if isempty(k)
    error('mgs:zarchan:accelMissAdjoint:readTgo', ...
        ['no curve was run at tgo = %g s.  This run has %s.  Each tgo is a ' ...
         'SEPARATE adjoint run -- there is nothing to interpolate between them, ' ...
         'so pass it in ''tgo'' and re-run.'], tgoWant, mat2str([C.tgo]));
end
[v, tAct] = nearestSample(C(k).t, C(k).sigma, tfWant, samp);
if ~isfinite(v)
    error('mgs:zarchan:accelMissAdjoint:readTf', ...
        ['flight time %g s is outside the tgo = %g curve, which covers %.4f to ' ...
         '%.4f s.'], tfWant, tgoWant, C(k).t(1), C(k).t(end));
end
if abs(tAct - tfWant) > 0.05*samp
    % Sub-sample honesty: silent rounding to the nearest sample is fine for a
    % plot and is not fine for a number someone is about to quote.  The
    % threshold is a twentieth of a sample and NOT something tight like 1e-6,
    % because every sample instant carries Zarchan's structural +1e-5 offset --
    % a 1e-6 threshold would warn on every correct call and be tuned out.
    warning('mgs:zarchan:accelMissAdjoint:readSnap', ...
        'tF = %g snapped to the nearest sample at %.5f s.', tfWant, tAct);
end
end


% =============================================================================
function K = structuralCheck(opts, tgo, isMiss, GRAV_FT)
%STRUCTURALCHECK  Invariants that must hold or every number above is garbage.
%
%   These are NOT book benchmarks.  They test the WIRING, on the CALLER's
%   parameters, and they are the only checks in this file that still work when
%   the caller runs off-book.
%
%   1  CLOSED FORM (miss channel).  The analytic sigma of the uniformly
%      distributed maneuver, from an inverse Laplace transform that never
%      touches the integrator.  Discretization against the 1/tgo singularity
%      leaves an honest residual; measured, as a fraction of the curve's own
%      peak, at H = 0.01:
%           N' = 3   1.43%        N' = 5   2.34%
%           N' = 4   1.89%        N' = 6   2.81%
%      i.e. very nearly 0.48% * N' * (H/0.01), which is where the band
%      tol = max(0.5%, 1.5*N'*H) comes from: a uniform 3.1x-3.2x margin at
%      every one of those, with a FLOOR so that refining H to get a BETTER
%      answer cannot be punished with a spurious error once the residual stops
%      falling as fast as a linear band does.
%      Normalised by the PEAK, not pointwise: the curve starts near zero, and a
%      pointwise relative error there divides a small number by a smaller one
%      and manufactures a failure out of nothing.
%
%   1b The generic route above, evaluated at N' = 3, must equal the closed form
%      the book actually prints (:1355).  Measured 2.1e-5 relative at hFine =
%      5e-4, and 8.1e-5 at 1e-3 -- second order in the grid, i.e. it is the
%      quadrature and nothing else.  This is what makes 1 a check
%      against ZARCHAN rather than against an inversion we wrote ourselves.
%
%   2  TIME-SCALE INVARIANCE.  Scale TAU, TF, tgo, H and samp together by 2.
%      The equations are invariant under that scaling apart from the two fixed
%      1e-5 offsets, so:
%           'accel' sigma is DIMENSIONLESS x nT   -> must be UNCHANGED
%           'miss'  sigma scales as T^2            -> must scale by 4
%      Measured residuals 4.9e-5 and 2.8e-4, both entirely from the 1e-5
%      offsets not scaling; bands 1e-3 and 5e-3 leave ~20x and ~18x.
%      *** This is the ONLY check anywhere in this file that can see a wrong
%      TAU inside the acceleration initial conditions. ***

K = struct();
K.skipped = false;

% -- 1. closed form, miss channel, on the caller's XNT/XNP/TAU/TF/H ---------
K.cf_integer = (abs(opts.XNP - round(opts.XNP)) < 1e-12) && (round(opts.XNP) >= 3);
K.cf_resid = NaN;  K.cf_tol = NaN;  K.cf_ok = false;
K.pub_resid = NaN; K.pub_tol = NaN; K.pub_ok = false;  K.pub_run = false;

M = adjointRun(true, 0.0, opts.XNT, opts.XNP, opts.TAU, opts.TF, opts.H, ...
               opts.samp, GRAV_FT);
K.miss_t     = M.t;
K.miss_sigma = M.sigma;

if K.cf_integer
    % 5e-4 rather than 1e-3: the trapezoid error is second order in it, and at
    % 1e-3 the generic-vs-published margin is only 12x against a 1e-3 band.
    % 20 000 points of a smooth analytic integrand costs nothing.
    hFine  = min(opts.H, 5e-4);
    cf     = cfSigma(M.t, opts.TAU, round(opts.XNP), opts.XNT, hFine);
    K.miss_cf = cf;
    pk = max(abs(cf));
    if pk <= 0
        error('mgs:zarchan:accelMissAdjoint:degenerate', ...
            ['the closed-form miss curve is identically zero, so any residual would ' ...
             'compare as perfect -- refusing to report a pass on it.']);
    end
    K.cf_resid = max(abs(M.sigma - cf))/pk;
    K.cf_tol   = max(0.005, 1.5*opts.XNP*opts.H);
    K.cf_ok    = K.cf_resid <= K.cf_tol;

    % -- 1b. generic route vs the formula the book prints, N' = 3 only.
    % Restricted to x = tF/T >= 0.1: below that the PRINTED form is subtractive
    % cancellation noise (see cfSigmaPublished), so a disagreement there would
    % be the book's algebra losing digits, not our inversion being wrong.
    if round(opts.XNP) == 3
        sel = (M.t/opts.TAU) >= 0.1;
        if any(sel)
            pub = cfSigmaPublished(M.t(sel), opts.TAU, opts.XNT);
            K.pub_resid = max(abs(cf(sel)./pub - 1));
            K.pub_tol   = 1e-3;
            K.pub_ok    = K.pub_resid <= K.pub_tol;
            K.pub_run   = true;
        end
    end
else
    K.miss_cf = nan(size(M.t));
end

% -- 2. time-scale invariance ----------------------------------------------
alpha = 2.0;
if isMiss
    % No caller tgo to borrow, so probe the acceleration channel at the book's
    % 0.5 s scaled by TAU -- pulled in if TF is too short to leave room for it.
    % A probe that overruns the horizon would raise :noSamples from inside a
    % self-check, which reads like a broken port rather than a short run.
    tgoRef = min(0.5*opts.TAU, 0.5*(opts.TF - 2*opts.samp));
else
    tgoRef = tgo(1);
end
K.scale_accel_rel = NaN;  K.scale_accel_tol = 1e-3;
% 2.5 sample intervals, not 1: the probe needs at least two RECORDED samples
% to be comparable at all, and one that returns a single point would raise
% :noSamples from inside a self-check -- which reads like a broken port.
K.scale_accel_run = tgoRef > 0 && tgoRef < opts.TF - 2.5*opts.samp;
K.scale_accel_ok  = false;
if K.scale_accel_run
    aBase = adjointRun(false, tgoRef, opts.XNT, opts.XNP, opts.TAU, opts.TF, ...
                       opts.H, opts.samp, GRAV_FT);
    aScal = adjointRun(false, alpha*tgoRef, opts.XNT, opts.XNP, alpha*opts.TAU, ...
                       alpha*opts.TF, alpha*opts.H, alpha*opts.samp, GRAV_FT);
    K.scale_accel_rel = compareScaled(aBase.sigma, aScal.sigma, 1.0);
    K.scale_accel_ok  = K.scale_accel_rel <= K.scale_accel_tol;
end
K.scale_accel_tgo = tgoRef;

mScal = adjointRun(true, 0.0, opts.XNT, opts.XNP, alpha*opts.TAU, alpha*opts.TF, ...
                   alpha*opts.H, alpha*opts.samp, GRAV_FT);
K.scale_miss_rel = compareScaled(M.sigma, mScal.sigma, alpha^2);
K.scale_miss_tol = 5e-3;
K.scale_miss_ok  = K.scale_miss_rel <= K.scale_miss_tol;

K.ok = (K.cf_ok || ~K.cf_integer) && (K.pub_ok || ~K.pub_run) && ...
       (K.scale_accel_ok || ~K.scale_accel_run) && K.scale_miss_ok;

if ~K.ok
    % A structural failure voids every curve this function computed, so it
    % ERRORS.  An adjoint that is wrong and returns its curve anyway is the
    % worst outcome available here: the curve is smooth, plausible and
    % indistinguishable from an answer.
    error('mgs:zarchan:accelMissAdjoint:selfcheck', ...
        ['a STRUCTURAL invariant failed, so every curve this function computed is ' ...
         'wrong:\n' ...
         '  miss vs analytic closed form   : rel %.3e (tol %.1e) %s\n' ...
         '  generic route vs book''s :1355  : rel %.3e (tol %.1e) %s\n' ...
         '  accel invariant under t-scaling: rel %.3e (tol %.1e) %s\n' ...
         '  miss  scales as TAU^2          : rel %.3e (tol %.1e) %s\n' ...
         'These test the WIRING, not the book.  A failure here means the ' ...
         'transcription of Listing 5.3, or of the closed form, has been damaged.'], ...
        K.cf_resid, K.cf_tol, tf2s(K.cf_ok || ~K.cf_integer), ...
        K.pub_resid, K.pub_tol, tf2s(K.pub_ok || ~K.pub_run), ...
        K.scale_accel_rel, K.scale_accel_tol, tf2s(K.scale_accel_ok || ~K.scale_accel_run), ...
        K.scale_miss_rel, K.scale_miss_tol, tf2s(K.scale_miss_ok));
end
end


function r = compareScaled(base, scaled, factor)
%COMPARESCALED  Max relative gap between a curve and a scaled re-run of it.
%   The two grids are aligned by construction (every time in the scaled run is
%   alpha times the corresponding time in the base run), but the 1e-5 offsets
%   do not scale, so the run lengths can differ by one sample.  More than one
%   means the alignment assumption itself broke and the comparison would be
%   meaningless -- so that ERRORS instead of quietly comparing the overlap.
if abs(numel(base) - numel(scaled)) > 1
    error('mgs:zarchan:accelMissAdjoint:scaleGrid', ...
        ['scaled re-run produced %d samples against %d -- the two grids are not ' ...
         'the same grid, so the invariant cannot be evaluated (and an ' ...
         'unevaluated invariant is not a passing one).'], numel(scaled), numel(base));
end
n = min(numel(base), numel(scaled));
b = base(1:n);
s = scaled(1:n) / factor;
r = max(abs(s - b) ./ max(abs(b), eps));
end


% =============================================================================
function SH = shapeChecks(R, opts, isMiss)
%SHAPECHECKS  The two claims the book makes about these curves in PROSE.
%
%   A prose claim survives the extraction where a figure value does not, and an
%   ordering is often a sharper test than a number read off a plot: it has no
%   tolerance to argue about.  Both of these are evaluated on the CALLER's own
%   run, so they keep working off-book.
%
%   SH1  ":1556  the standard deviation of the missile acceleration is
%        monotonically increasing for the 10-s flight".  Fig. 5.6 is
%        acceleration against TIME for one flight; our tgo curves sample that
%        same profile backwards, so rising in time is FALLING in tgo:
%        sigma(tgo=0.5) > sigma(tgo=1) > sigma(tgo=2) at any fixed flight time.
%        Needs at least two tgo values.
%
%   SH2  ":1284  the miss distance is small for both small and large flight
%        times" -- the miss curve has an INTERIOR maximum.  Zarchan says why in
%        the same passage: too little time for the miss to develop at one end,
%        and at the other the maneuver most likely starts at a large time-to-go
%        relative to T.  Evaluated on the reference miss run.

SH = struct('what', {}, 'claim', {}, 'source', {}, 'ok', {}, 'skipped', {}, ...
            'detail', {});

% -- SH1 --------------------------------------------------------------------
i = 1;
SH(i).what   = 'accel falls as tgo grows';
SH(i).claim  = 'sigma_nL decreasing in tgo at fixed flight time';
SH(i).source = 'zarchan.txt:1556 ("monotonically increasing for the 10-s flight")';
if isMiss || numel(R.tgo) < 2
    SH(i) = markShapeSkipped(SH(i), 'needs mode=accel with 2 or more tgo values');
else
    [ts, ord] = sort(R.tgo);
    % Read at the EARLIEST end-of-run among the curves, not the last one's end:
    % different tgo values start the run at different instants, so the sample
    % grids need not finish together and reading past the shortest curve would
    % return NaN and report a FAIL for a perfectly sound run.
    tRead = min(arrayfun(@(c) c.t(end), R.curve));
    v = nan(1, numel(ts));
    for j = 1:numel(ts)
        v(j) = nearestSample(R.curve(ord(j)).t, R.curve(ord(j)).sigma, tRead, opts.samp);
    end
    if ~all(isfinite(v))
        % SKIP, not FAIL.  "The curves do not share a sample instant" is a
        % statement about the grid, and scoring it as a violated book claim
        % would put a red row next to a citation that says nothing of the kind.
        SH(i) = markShapeSkipped(SH(i), sprintf( ...
            'curves share no sample instant near tF = %.3f s (tgo values %s)', ...
            tRead, mat2str(ts)));
    else
        SH(i).ok      = all(diff(v) < 0);
        SH(i).skipped = false;
        SH(i).detail  = sprintf('at tF = %.2f s: %s for tgo = %s', ...
            tRead, mat2str(round(v,4)), mat2str(ts));
    end
end

% -- SH2 --------------------------------------------------------------------
i = 2;
SH(i).what   = 'miss curve peaks in the interior';
SH(i).claim  = 'sigma_miss small at small AND large flight time';
SH(i).source = 'zarchan.txt:1284';
if isfield(R.selfcheck, 'miss_sigma')
    ms = R.selfcheck.miss_sigma;
    mt = R.selfcheck.miss_t;
elseif isMiss
    ms = R.curve(1).sigma;
    mt = R.curve(1).t;
else
    ms = [];  mt = [];
end
if numel(ms) < 3
    SH(i) = markShapeSkipped(SH(i), 'no miss curve available (selfcheck off, mode=accel)');
else
    [pk, ip] = max(ms);
    ok = (ip > 1) && (ip < numel(ms)) && (ms(1) < pk) && (ms(end) < pk);
    SH(i).ok      = ok;
    SH(i).skipped = false;
    SH(i).detail  = sprintf('peak %.4f ft at tF = %.3f s; ends %.4f and %.4f ft', ...
        pk, mt(ip), ms(1), ms(end));
end
end


function s = markShapeSkipped(s, why)
s.ok = false;  s.skipped = true;  s.detail = why;
end


% =============================================================================
function B = benchmarks(GRAV_FT)
%BENCHMARKS  Computed vs published, with a band and a REASON for the band.
%
%   *** THESE RUNS ALWAYS USE THE BOOK'S PARAMETERS, WHATEVER THE CALLER
%   PASSED. ***  XNT=96.6, XNP=3, TAU=1, TF=10, H=0.01, samp=0.1, straight off
%   Listing 5.3's first line.  The table is therefore an audit of the
%   TRANSCRIPTION and is always evaluable.  The alternative -- score the
%   caller's run and SKIP when it is off-book -- produces a table of SKIPs that
%   every reader scrolls past as if it were green.  The caller's own parameters
%   are covered by the structural self-check instead, which is the half of the
%   audit that is actually capable of covering them.
%
%   THREE TOLERANCE BANDS, and the difference between them is provenance:
%
%   'text'   5%  -- printed as a number in the running text (12 g, 9 g, 6.1 g,
%                   13.3 ft) but ORIGINATING in a figure the author read by
%                   eye.  "12 g" carries two significant figures: its own
%                   rounding is already +-4.2%.  A 1% band here would be
%                   testing the author's eyesight, and this scheme's honest
%                   discretization is +1.4% on top.
%   'graph' 15%  -- the values the author himself qualified with
%                   "approximately" (:1578).
%   'form'       -- against the CLOSED FORM at :1355, which is an equation and
%                   not a reading.  Band is max(0.5%, 1.5*N'*H) = 4.5% here,
%                   the measured discretization law of this RK2 scheme
%                   (see STRUCTURALCHECK).  This is the sharpest row.
%
%   DO NOT widen a band to turn a row green.  The bands are set by where the
%   book value came from, not by what we happen to compute.

XNT = 96.6;  XNP = 3.0;  TAU = 1.0;  TF = 10.0;  H = 0.01;  samp = 0.1;

Mrun = adjointRun(true, 0.0, XNT, XNP, TAU, TF, H, samp, GRAV_FT);
A05  = adjointRun(false, 0.5, XNT, XNP, TAU, TF, H, samp, GRAV_FT);
A10  = adjointRun(false, 1.0, XNT, XNP, TAU, TF, H, samp, GRAV_FT);
A20  = adjointRun(false, 2.0, XNT, XNP, TAU, TF, H, samp, GRAV_FT);

B = emptyBench();

% -- the miss channel -------------------------------------------------------
B = addRow(B, 'miss sigma @ tF=10 s', 'ft', 13.3, 'text', 0.05, ...
    Mrun, 10.0, samp, ...
    'zarchan.txt:1536, Fig. 5.5 / Fig. 4.18 (covariance value, asserted equal to the adjoint)');

% The closed form is an EQUATION, so this row is scored against the equation
% evaluated at the same point -- not against anything anybody read off a plot.
bookCF = cfSigmaPublished(10.0, TAU, XNT);
B = addRow(B, 'miss vs :1355 closed form', 'ft', bookCF, 'form', ...
    max(0.005, 1.5*XNP*H), Mrun, 10.0, samp, ...
    'zarchan.txt:1355, MUDNT|N''=3 evaluated at tF/T = 10');

% -- the acceleration channel ----------------------------------------------
B = addRow(B, 'accel tgo=0.5 @ tF=10 s', 'g', 12.0, 'text', 0.05, ...
    A05, 10.0, samp, 'zarchan.txt:1583, Fig. 5.8 / Fig. 5.6');
B = addRow(B, 'accel tgo=1.0 @ tF=10 s', 'g', 9.0,  'text', 0.05, ...
    A10, 10.0, samp, 'zarchan.txt:1583, Fig. 5.8 / Fig. 5.6');
B = addRow(B, 'accel tgo=2.0 @ tF=10 s', 'g', 6.1,  'text', 0.05, ...
    A20, 10.0, samp, 'zarchan.txt:1583, Fig. 5.8 / Fig. 5.6');

% The same tgo = 0.5 curve read at two other flight times -- free, from a run
% already done, and they are what pins the SHAPE of Fig. 5.8 rather than one
% point on it.  Both are "approximately" in the book's own words.
B = addRow(B, 'accel tgo=0.5 @ tF=6 s', 'g', 12.0, 'graph', 0.15, ...
    A05, 6.0, samp, 'zarchan.txt:1578 ("approximately 12 g ... for a 6-s flight")');
B = addRow(B, 'accel tgo=0.5 @ tF=2 s', 'g', 4.0,  'graph', 0.15, ...
    A05, 2.0, samp, 'zarchan.txt:1578 ("approximately 4 g for a 2-s flight")');
end


function B = emptyBench()
B = struct('what', {}, 'unit', {}, 'book', {}, 'ours', {}, 'pct', {}, ...
           'band', {}, 'tol', {}, 'ok', {}, 'skipped', {}, 'why', {}, ...
           'source', {}, 't_read', {});
end


function B = addRow(B, what, unitStr, book, band, tol, A, tf, samp, source)
%ADDROW  One benchmark row.  The run struct is called A and NOT `run`: `run` is
%   a MATLAB built-in, and shadowing it inside a file this long is how a later
%   edit acquires a bug that only shows up in someone else's session.
i = numel(B) + 1;
B(i).what = what;   B(i).unit  = unitStr;  B(i).book = book;
B(i).band = band;   B(i).tol = tol;      B(i).source = source;
[ours, tAct] = nearestSample(A.t, A.sigma, tf, samp);
B(i).t_read = tAct;
if ~isfinite(ours)
    % Cannot happen at the book's parameters; kept because "cannot happen" is
    % how the previous wave's silent NaN comparisons got in.
    B(i).ours = NaN;  B(i).pct = NaN;  B(i).ok = false;  B(i).skipped = true;
    B(i).why  = sprintf('no sample within half an interval of tF = %g s', tf);
    return;
end
B(i).ours    = ours;
B(i).pct     = 100*(ours/book - 1);
B(i).ok      = abs(ours/book - 1) <= tol;
B(i).skipped = false;
B(i).why     = '';
end


% =============================================================================
function s = render(R, opts, isMiss)
W = 92;
L = {};
L{end+1} = 'Zarchan Listing 5.3 -- ACCELERATION AND MISS DISTANCE ADJOINT (single-lag PN loop)';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf('mode = %-5s   XNT = %g ft/s^2 (%.2f g)   N'' = %g   TAU = %g s   TF = %g s   H = %g s', ...
    R.mode, opts.XNT, opts.XNT/32.2, opts.XNP, opts.TAU, opts.TF, opts.H);
L{end+1} = sprintf('tgo = %s s   sampled every %g s   integrator: Zarchan''s two-pass RK2, transcribed', ...
    mat2str(R.tgo), opts.samp);
L{end+1} = 'disturbance: 3-g step target maneuver, START TIME UNIFORM OVER THE FLIGHT -> every';
L{end+1} = 'number below is a STANDARD DEVIATION (Ch.4 shaping filter, zarchan.txt:1263).';
L{end+1} = repmat('-', 1, W);

% ---- the curves -----------------------------------------------------------
if isMiss
    L{end+1} = 'MISS CURVE   sigma_miss [ft] against flight time tF (Figs. 4.18 / 5.5)';
else
    L{end+1} = 'ACCELERATION CURVES   sigma_nL [g] against flight time tF (Fig. 5.8)';
end
hdr = sprintf('    %7s', 'tF [s]');
for k = 1:numel(R.curve)
    if isMiss
        hdr = [hdr sprintf('  %11s', 'sigma [ft]')]; %#ok<AGROW>
    else
        hdr = [hdr sprintf('  %11s', sprintf('tgo=%g [g]', R.curve(k).tgo))]; %#ok<AGROW>
    end
end
if isMiss && isfield(R.selfcheck, 'miss_cf') && any(isfinite(R.selfcheck.miss_cf))
    hdr = [hdr sprintf('  %11s', 'closed [ft]')];
end
L{end+1} = hdr;

dtRow = max(opts.samp, opts.samp*round((opts.TF/10)/opts.samp));
tRows = dtRow*(1:floor(opts.TF/dtRow + 1e-9));
tRows = tRows(1:min(numel(tRows), 20));
for x = tRows
    row = sprintf('    %7.2f', x);
    for k = 1:numel(R.curve)
        v = nearestSample(R.curve(k).t, R.curve(k).sigma, x, opts.samp);
        if isfinite(v)
            row = [row sprintf('  %11.4f', v)]; %#ok<AGROW>
        else
            row = [row sprintf('  %11s', '--')]; %#ok<AGROW>
        end
    end
    if isMiss && isfield(R.selfcheck, 'miss_cf') && any(isfinite(R.selfcheck.miss_cf))
        v = nearestSample(R.selfcheck.miss_t, R.selfcheck.miss_cf, x, opts.samp);
        if isfinite(v)
            row = [row sprintf('  %11.4f', v)]; %#ok<AGROW>
        else
            row = [row sprintf('  %11s', '--')]; %#ok<AGROW>
        end
    end
    L{end+1} = row; %#ok<AGROW>
end
if ~isMiss
    L{end+1} = '  a column starts only once tF exceeds its own tgo: adjoint time IS time-to-go,';
    L{end+1} = '  so the tgo = 2 s run has nothing to say about a 2-s flight.';
end
L{end+1} = repmat('-', 1, W);

% ---- self-check -----------------------------------------------------------
if isfield(R.selfcheck, 'ok')
    K = R.selfcheck;
    L{end+1} = 'STRUCTURAL SELF-CHECK  (your parameters, not the book''s -- a failure voids everything)';
    if K.cf_integer
        L{end+1} = sprintf('  miss vs analytic closed form (max/peak)   : rel %9.3e  tol %.1e  %s', ...
            K.cf_resid, K.cf_tol, tf2s(K.cf_ok));
    else
        L{end+1} = sprintf(['  miss vs analytic closed form              : SKIPPED, N'' = %g is not ' ...
            'an integer >= 3'], opts.XNP);
        L{end+1} = '        the inversion is a finite binomial only for integer N''; a truncation of';
        L{end+1} = '        the infinite series is NOT a closed form and will not be printed as one.';
    end
    if K.pub_run
        L{end+1} = sprintf('  that route vs the book''s own :1355 form   : rel %9.3e  tol %.1e  %s', ...
            K.pub_resid, K.pub_tol, tf2s(K.pub_ok));
    end
    if K.scale_accel_run
        L{end+1} = sprintf('  accel unchanged under time scaling (x2)   : rel %9.3e  tol %.1e  %s   [tgo %g s]', ...
            K.scale_accel_rel, K.scale_accel_tol, tf2s(K.scale_accel_ok), K.scale_accel_tgo);
    else
        L{end+1} = sprintf(['  accel unchanged under time scaling (x2)   : SKIPPED, TF = %g s leaves ' ...
            'no room for a probe run'], opts.TF);
    end
    L{end+1} = sprintf('  miss scales as TAU^2 under scaling (x2)   : rel %9.3e  tol %.1e  %s', ...
        K.scale_miss_rel, K.scale_miss_tol, tf2s(K.scale_miss_ok));
    L{end+1} = '  the scaling rows are the ONLY thing here that can see a wrong TAU inside the';
    L{end+1} = '  acceleration initial conditions x3 = N''/(TAU*tgo), x4 = -1/TAU.';
else
    L{end+1} = 'STRUCTURAL SELF-CHECK  ***SKIPPED*** (selfcheck=false) -- your parameters are unverified';
end
L{end+1} = repmat('-', 1, W);

% ---- benchmarks -----------------------------------------------------------
if isempty(R.bench)
    L{end+1} = 'BOOK BENCHMARKS  ***SKIPPED*** (benchmark=false) -- the transcription is unaudited';
else
    L{end+1} = 'BOOK BENCHMARKS  run at the BOOK''s parameters (XNT=96.6, N''=3, TAU=1, TF=10, H=0.01),';
    L{end+1} = 'NOT at yours -- this table audits the transcription, the self-check above audits your run.';
    L{end+1} = sprintf('  %-26s %5s %9s %9s %9s %-6s %s', ...
        'what', 'unit', 'book', 'ours', 'diff', 'band', 'verdict');
    for i = 1:numel(R.bench)
        b = R.bench(i);
        if b.skipped
            L{end+1} = sprintf('  %-26s %5s %9.4g %9s %9s %-6s SKIP (%s)', ...
                b.what, b.unit, b.book, '--', '--', b.band, b.why); %#ok<AGROW>
        else
            L{end+1} = sprintf('  %-26s %5s %9.4g %9.4f %+8.2f%% %-6s %s', ...
                b.what, b.unit, b.book, b.ours, b.pct, b.band, tf2s(b.ok)); %#ok<AGROW>
        end
    end
    L{end+1} = '  provenance of every book value above:';
    for i = 1:numel(R.bench)
        L{end+1} = sprintf('    %-26s %s', R.bench(i).what, R.bench(i).source); %#ok<AGROW>
    end
    L{end+1} = '  bands: text 5% = a number printed in the text but read off a figure (2 s.f.);';
    L{end+1} = '         graph 15% = the author wrote "approximately"; form = against an equation,';
    L{end+1} = '         banded by this scheme''s measured discretization max(0.5%, 1.5*N''*H).';
    L{end+1} = '  the adjoint reads ~1.4% HIGH against the closed form by construction: RK2 against';
    L{end+1} = '  a 1/tgo singularity.  That is the honest residual, not an error to be tuned away.';
end
L{end+1} = repmat('-', 1, W);

% ---- shape checks ---------------------------------------------------------
L{end+1} = 'SHAPE CLAIMS THE BOOK STATES IN PROSE  (no number to read off a plot, so no tolerance)';
for i = 1:numel(R.shape)
    sh = R.shape(i);          % NOT `s`: `s` is this function's output variable
    if sh.skipped
        L{end+1} = sprintf('  %-30s SKIP  %s', sh.what, sh.detail); %#ok<AGROW>
    else
        L{end+1} = sprintf('  %-30s %s  %s', sh.what, tf2s(sh.ok), sh.detail); %#ok<AGROW>
    end
    L{end+1} = sprintf('    claim: %s   [%s]', sh.claim, sh.source); %#ok<AGROW>
end
L{end+1} = repmat('-', 1, W);

% ---- the cross-check this file exists for ---------------------------------
L{end+1} = 'CROSS-CHECK WITH THE COVARIANCE PORT (Listing 5.2) -- what to compare, and to what';
L{end+1} = '  sqrt(X(1,1)) at t = 10 s          <->  R.read(0, 10)      [mode ''miss'']';
L{end+1} = '  sqrt(A*X*A'') at t = 9.5, 9, 8 s  <->  R.read(0.5, 10), R.read(1, 10), R.read(2, 10)';
L{end+1} = '  Expect ~1.5%, not machine precision, and expect THIS side to read HIGH: covariance';
L{end+1} = '  runs RK4 and lands on the analytic value, this adjoint runs the book''s RK2 against';
L{end+1} = '  a 1/tgo singularity.  Agreement to 1e-12 would mean one of them is not what it says.';
L{end+1} = repmat('-', 1, W);
L{end+1} = 'READ THIS BEFORE QUOTING ANY NUMBER ABOVE';
L{end+1} = '  Linearised, planar, constant closing speed, single lag, and the ONLY disturbance is a';
L{end+1} = '  target maneuver at a random start time.  Our bullseye is stationary, our measured';
L{end+1} = '  discriminant is release heading error |eps0|, and our lag saturates.  This file prices';
L{end+1} = '  what a clean single-lag loop does; it does not predict a sortie and must not be';
L{end+1} = '  reported as if it did.  The book''s 15-g figure (:1556) is a tgo -> 0 reading and is';
L{end+1} = '  deliberately NOT a row above -- this adjoint diverges there.';
s = strjoin(L, newline);
end


% =============================================================================
function drawCurves(R, opts, isMiss)
figure('Name', sprintf('Zarchan Listing 5.3 -- %s adjoint', R.mode));
hold on; grid on;
for k = 1:numel(R.curve)
    plot(R.curve(k).t, R.curve(k).sigma, 'LineWidth', 1.2);
end
if isMiss
    if isfield(R.selfcheck, 'miss_cf') && any(isfinite(R.selfcheck.miss_cf))
        plot(R.selfcheck.miss_t, R.selfcheck.miss_cf, '--', 'LineWidth', 1.0);
        legend({'adjoint (Listing 5.3)', 'closed form (zarchan.txt:1355)'}, 'Location', 'best');
    end
    ylabel('\sigma miss distance  [ft]');
    title(sprintf('Miss distance adjoint, TAU = %g s, N'' = %g (Figs. 4.18 / 5.5)', ...
        opts.TAU, opts.XNP));
else
    ylabel('\sigma missile acceleration  [g]');
    title(sprintf('Acceleration adjoint, TAU = %g s, N'' = %g (Fig. 5.8)', ...
        opts.TAU, opts.XNP));
    legend(arrayfun(@(c) sprintf('tgo = %g s', c.tgo), R.curve, ...
        'UniformOutput', false), 'Location', 'best');
end
xlabel('Flight time  t_F  [s]   (= adjoint time)');
hold off;
end


function s = tf2s(b)
if b, s = 'PASS'; else, s = 'FAIL'; end
end
