function R = shapingFilterAdjoint(opts)
%MGS.ZARCHAN.SHAPINGFILTERADJOINT  Zarchan Listing 4.6 -- stochastic adjoint
%   via the shaping-filter approach, plus its Listing 4.5 Monte Carlo twin.
%
%   R = mgs.zarchan.shapingFilterAdjoint()
%   R = mgs.zarchan.shapingFilterAdjoint('montecarlo', false)     adjoint only
%   R = mgs.zarchan.shapingFilterAdjoint('TAU', 2.0, 'assert', false)
%   R = mgs.zarchan.shapingFilterAdjoint('mc_runs', 4000)         tighter MC band
%
%   Line-by-line port of Listing 4.6 (zarchan.txt:1329-1337) -- the adjoint of
%   Fig. 4.17 -- integrated by the book's own second-order Runge-Kutta (Heun)
%   scheme.  Listing 4.5 (zarchan.txt:1292-1312), the 500-run Monte Carlo the
%   book compares it against, is ported here too, in the same file, because the
%   ONLY thing that makes 4.6 trustworthy is that it agrees with 4.5.
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, g = 32.2 ft/s^2.  NOT SI.
%
%   Chapter 4 is a ft/s chapter (Listing 30.1 is the book's metric exception;
%   this is not it).  The rest of mgs is metres.  Nothing here is converted, so
%   that every number printed can be laid straight against a published one.  A
%   silent ft->m would turn the book's 13.3 ft of miss into "4.05" and it would
%   still read as a perfectly reasonable miss distance -- which is precisely
%   the failure mode this package exists to prevent.  Convert AT THE CALL SITE
%   and say so there.
% ===========================================================================
%
%   WHY THIS ONE MATTERS HERE
%
%   This is the listing that gets a DISPERSION out of ONE run.
%
%   Listing 4.5 obtains the standard deviation of miss by flying 50 sorties per
%   flight time, ten flight times, 500 sorties in total.  Listing 4.6 obtains
%   the same curve -- every flight time at once -- from a single integration,
%   by driving the adjoint through a shaping filter whose second-order
%   statistics match the random disturbance.  The book's own summary of that,
%   zarchan.txt:1328: one adjoint run gave results equivalent to ten 50-run
%   Monte Carlo sets.
%
%   We currently price a parameter against our 3 s spread requirement by flying
%   sorties.  That is the 500-run column.  This file is the template for the
%   one-run column.  What it does NOT do is make our spread problem linear:
%   see WHAT IT IS NOT, below, before quoting a number out of here at anybody.
%
%   *** WHAT IT IS NOT. ***  A LINEARISED, PLANAR homing loop with constant
%   closing velocity, a FIRST-ORDER lag, no gravity, no drag, no thrust, no
%   acceleration limit and no LOS floor.  The single random quantity is the
%   START TIME of a step target maneuver, uniform over the flight.  Our spread
%   is dominated by release-time scheduling and by a gamma-tracking lag that
%   SATURATES (measured gain 0.063 against a model 1.385, ~8 s of it, hitting
%   its stop on 26 of 32 sorties) -- and a saturating lag is exactly what an
%   adjoint cannot represent, because the adjoint identity is a LINEARITY
%   identity.  Read this file as the machinery, and as the clean-loop baseline
%   our failures are measured against.  Do not read a dispersion out of it and
%   call it a prediction for an aircraft.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION, AND WHAT WAS DONE TO IT
%
%   matlab/zarchan/_listings/listing_04_06.m does not parse as MATLAB.  Its own
%   header reports NO unresolved variable-to-variable comparisons, and that is
%   correct -- every comparison in 4.6 is against a literal.  Four repairs, all
%   forced, none of them a judgement call:
%
%     1  "while TP , = (TF - 1e-5)"   -> "while TP <= (TF - 1e-5)".  The OCR
%        renders "<" as "," throughout this book.  Shape-forced: TP only ever
%        increases, so ">=" never terminates and "==" never fires.
%     2  "while STEP , =1"            -> "while STEP <= 1".  Same substitution;
%        the body sets STEP=2 to leave, which only makes sense against "<=".
%     3  "if S . =.000999"            -> "if S >= .000999".  "." is the OCR of
%        ">".  S is reset to 0 on every record, so "<=" would record forever.
%     4  The line "CLOSED-FORM SOLUTION FOR RANDOM TARGET MANEUVER" trailing
%        the transcription is the next SECTION HEADING of the book leaking in:
%        the listing ends at zarchan.txt:1337 and that heading is :1338.
%        Dropped, not "fixed" into code.  (It is the heading of the passage
%        that supplies BENCHMARK 3 below, which is how it was identified.)
%
%   Two of the book's own quirks are reproduced deliberately and one is not:
%
%     * "save datfil.txt output /ascii" is a typo for -ascii (Listing 4.5, four
%       pages earlier, has it right).  Nothing is written to disk here at all,
%       so it is recorded and dropped.
%     * TP starts at +1e-5 and TGO carries a SECOND +1e-5.  Kept.  The adjoint
%       gain is N'/tgo and tgo IS the adjoint time; an exact zero divides by
%       zero on the first derivative evaluation.  Also kept in the sampling
%       arithmetic: XMUDNT divides X5 by TGO, not by TP, and this port does the
%       same, so the closed form below is evaluated at TGO too.  The difference
%       is 1e-5 s and matters nowhere, but "matters nowhere" is a claim that
%       should be checkable rather than assumed.
%     * The listing does "S = S + H" TWICE per outer step and then tests
%       "S >= .000999".  Since 0.000999 is far below one increment, the test is
%       true every step: the listing records EVERY step, and the double
%       increment changes nothing at its own settings.  This port increments
%       ONCE and tests against 'samp' (default = H, i.e. still every step).
%       Reason: keeping the literal 0.000999 would silently change meaning for
%       H < 0.0005, and keeping the double increment would hand a caller who
%       asks for samp = 0.1 a curve sampled at 0.05 -- an off-by-two in the
%       time axis that plots perfectly smoothly.
%
% ---------------------------------------------------------------------------
%   LISTING 4.5 IS IN THIS FILE, AND ITS ONE UNRESOLVED COMPARISON
%
%   The 4.5 transcription DOES carry an unresolved variable-to-variable
%   comparison: "if T , TSTART".  It is resolved here as
%
%        if T < TSTART,  XNT = 0,  else  XNT = COEF*nT     (maneuver STARTS)
%
%   and NOT as "T > TSTART" (which would make the maneuver STOP at TSTART).
%   Three independent reasons, in decreasing order of strength:
%
%     a  The chapter derives the shaping filter for x(t) = nT*u(t - T) and
%        prints the definition next to it (zarchan.txt:1226): u(t-T) = 0 for
%        t < T, 1 otherwise.  A maneuver that stops at T is a different signal
%        with a different autocorrelation, and the whole point of 4.5 is to be
%        the Monte Carlo of THAT signal.
%     b  The closed form at zarchan.txt:1354-1357 integrates MNT(t)^2 over the
%        flight, where MNT(t) is the miss from a step maneuver applied with t
%        seconds TO GO.  That construction requires the maneuver to be ON for
%        the last t seconds.
%     c  It is measurable, and it is measured, by the MC-vs-adjoint benchmark
%        below.  Flipping the comparison changes sigma at tF = 3 s from 20.42
%        to 9.03 ft (-56%) and at tF = 1 s from 9.60 to 11.50 ft (+20%) -- both
%        figures derived analytically from the closed form, not measured.  The
%        band on that row is ~10% at the default sample size.  It would be
%        caught, loudly, at eight of the ten book flight times.
%        BUT NOT AT tF = 10 s, where the flipped and correct answers are nearly
%        equal in magnitude.  A single-point check at the book's headline
%        flight time would have passed the wrong resolution.  That is why the
%        agreement row is scored across the WHOLE sweep.
%
%   Two further things about 4.5 that are easy to "tidy" into wrongness:
%
%     * COEF = +/-1 IS LOad-BEARING.  The listing draws a second uniform and
%       flips the SIGN of the maneuver on each run.  Delete it and every run
%       maneuvers the same way; the sample MEAN becomes nonzero and the sample
%       STANDARD DEVIATION stops estimating the quantity the adjoint computes.
%       Analytically, at tF = 10 s: E[Y] = nT*(1/tF)*int MNT = 9.66 ft and
%       E[Y^2] = 174.9 ft^2, so dropping the sign gives sqrt(174.9 - 93.3) =
%       9.03 ft where the truth is 13.23 ft.  A 32% error, from a smooth run,
%       with no symptom.  The structural check "MC sample mean ~ 0" exists for
%       this and nothing else.
%     * VC CANCELS.  4.5 sets VC = 4000 and computes XLAMD =
%       (RTM*YD + Y*VC)/RTM^2 with RTM = VC*TGO, then XNC = XNP*VC*XLAMD.
%       Substitute and VC drops out completely: XNC = XNP*(TGO*YD + Y)/TGO^2.
%       The miss does not depend on closing velocity at all, which is why the
%       adjoint of Fig. 4.17 has no VC in it either.  A structural row runs two
%       deterministic trajectories at VC and 3*VC and requires bit-level
%       agreement; if a future edit puts a real VC dependence in, that row is
%       the only thing that will notice.  (VM = 3000 in 4.5 is never used.)
%
%   The Monte Carlo is VECTORISED ACROSS REPLICATIONS.  Every replication at a
%   given flight time shares the same time grid and the same loop bounds, and
%   the replications are completely independent, so the book's inner "for
%   I = 1:RUN" becomes an element index.  The arithmetic per replication, and
%   its ORDER, is the listing's.  This is the difference between a check that
%   runs and one nobody waits for.
%
% ---------------------------------------------------------------------------
%   THE ADJOINT, AND WHY THE IMPULSE IS ON x3
%
%   Adjoint rules swap inputs and outputs.  We want MISS, so the impulse goes
%   on the adjoint image of the miss node; the original system's white-noise
%   input becomes an adjoint OUTPUT, which is then squared and integrated.
%   Trace the chain from the listing itself: X1D = X2 and X2D = X3, so
%   x3 -> x2 -> x1 reverses nT -> ydd -> yd -> y, putting x3 at the y end.
%   Hence X3 = 1 at t = 0 and nothing else is nonzero.
%
%   The extra machinery relative to Listing 3.1 is the shaping filter: x1 is
%   the output of the integrator that represents H(s) = 1/s, and x5 = int x1^2
%   accumulates the mean-square response.  The spectral density of the driving
%   white noise is PHI_s = nT^2/tF (zarchan.txt:1290-1291), and the listing folds it
%   in at the sampling line as
%
%        XMUDNT = XNT*sqrt(X5/TGO)
%
%   That single expression IS the shaping-filter approach.  If it looks like a
%   normalisation you could move or drop: dividing by TGO is where nT^2/tF
%   enters, and without it the curve rises monotonically instead of peaking,
%   i.e. it would contradict the prose at zarchan.txt:1279 that BENCHMARK 5
%   scores.
%
%   PUT THE IMPULSE ON x1, x2 OR x4 AND THE PROGRAM STILL RUNS, smoothly, at
%   the right order of magnitude, answering a different question.  There is no
%   symptom.  The impulse state is hard-coded for that reason; BENCHMARK 1 is
%   what actually catches it.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS -- every book value was grepped out of zarchan.txt and the line
%   number is in the .source field of its row.  Nothing here was read off a
%   figure: the extraction turned every figure into an image link.
%
%   1  x1(t) vs the book's own closed form for the miss from a STEP target
%      maneuver, N' = 3 (zarchan.txt:1345, also stated at :890):
%
%            MNT|N'=3 = 0.5 * tF^2 * exp(-tF/T)          [s^2, a SENSITIVITY]
%
%      This is an EXACT solution of the transcribed ODEs, not an approximation.
%      Substituting it back gives, for general T and N' = 3,
%
%            x1 = e^(-t/T) * 0.5 t^2
%            x2 = e^(-t/T) * (t - t^2/2T)
%            x3 = e^(-t/T) * (1 - 2t/T + t^2/2T^2)
%            x4 = e^(-t/T) * (t^2/2T - t^3/6T^2)
%
%      and these satisfy X1D=X2, X2D=X3, X3D=3*Y1/t, X4D=-Y1 with Y1=(x4-x2)/T
%      term for term, with x3(0) = 1 and everything else zero -- the listing's
%      own initial conditions.  Only the x1 line is printed in the book; the
%      other three are derived HERE and are therefore scored as STRUCTURAL
%      rows, not as book values.  They are what makes the check sharp: a
%      swapped derivative or a sign flip on X4D moves them enormously while
%      leaving x1 plausible-looking.
%
%   2  sigma_miss(tF) vs the book's closed form (zarchan.txt:1354-1357):
%
%            MUDNT|N'=3 = (nT/4)*sqrt( (T^5/tF)*[3 - e^(-2x)*
%                                       (2x^4+4x^3+6x^2+6x+3)] ),  x = tF/T
%
%      Derivable, and derived, from BENCHMARK 1: MUDNT = nT*sqrt((1/tF)*int_0^tF
%      MNT^2 dt) and int_0^tF 0.25 t^4 e^(-2t/T) dt = (T^5/16)*[the bracket].
%      The bracket equals 4*int_0^x s^4 e^(-2s) ds, which is how this port
%      evaluates it below x = 1: as printed, "3 - e^(-2x)(...)" loses every
%      significant digit to cancellation for small x (at x = 0.01 the true
%      value is 8e-11 against a leading 3), and it does so by returning a
%      PLAUSIBLE SMALL NUMBER rather than a NaN.
%
%      TOLERANCE for 1 and 2: max(0.5% of peak, 4*H), the same band and the
%      same provenance as mgs.zarchan.adjointSingleLag.  The residual here is
%      DISCRETIZATION, not modelling error, and it has a known mechanism: on
%      the first step Y1 = (x4-x2)/T is exactly 0 while the true x3'(0) is
%      -N'/T, so Heun takes half the correct first slope and injects a one-time
%      offset of about 0.5*N'*H into x3, which then propagates as a constant
%      RELATIVE error of about 0.5*N'*H.  At N' = 3, H = 0.01 that predicts
%      +1.5%.  The same prediction for adjointSingleLag (N' = 4) gives +2.0%
%      against its MEASURED +1.88%, so the mechanism is not a story.
%      The 0.5% FLOOR is there so that refining H does not eventually punish a
%      correct run: this residual falls at roughly H^0.67 against the 1/tgo
%      singularity, not at first order.
%
%   3  sigma_miss at tF = 10 s vs the book's printed 13.3 ft.
%      zarchan.txt:1536, in Chapter 5: "the covariance analysis program
%      indicates that the standard deviation of the miss distance is 13.3 ft,
%      which is identical to the adjoint results of Chapter 4 (see Fig. 4.18)".
%      Fig. 4.18 IS this listing's output, so the sentence is a published value
%      FOR THIS PORT, in prose, three significant figures.  It is scored TWICE
%      and the split is the point:
%        3a  CLOSED FORM vs 13.3, band 1% ('text').  No integrator involved.
%            This asks "is the book's 13.3 the same problem as this formula".
%        3b  ADJOINT vs 13.3, band 1% + 4*H = 5%.  Literally the sum of the
%            two bands above, formed in code, so it can never be tighter than
%            either.  This asks "and does our integrator land on it".
%      Splitting it means a failure says WHICH half is wrong.  A single 13.3
%      row would have to carry the sum anyway, and would then be a 5% check
%      being quoted as a 3-figure agreement.
%
%   4  THE ONE THE BRIEF EXISTS FOR: Listing 4.6 vs Listing 4.5.
%      zarchan.txt:1320 -- "the adjoint and Monte Carlo results are in close
%      proximity, thus experimentally confirming the shaping filter approach".
%      "Close proximity" is an ADJECTIVE, so it is not scored as a value.  What
%      IS scored is the statistically correct version of it: at every one of
%      the book's flight times the two must agree to within k standard errors
%      of the Monte Carlo estimate, where the standard error is computed from
%      the SAMPLE'S OWN fourth moment,
%
%            SE(s) ~ sqrt( (m4 - m2^2)/n ) / (2s)
%
%      and not from a normal-theory formula -- the sample is bimodal by
%      construction (COEF = +/-1) and its kurtosis is about 2.7, not 3.
%      Band = max(k*SE, 3% of the adjoint value), k = 5 by default -- see the
%      arguments block for why 5 and not 3.  At the default 1000 runs that is
%      a band of about 10%, against fault-injection sizes of 56% and 32%.
%      The 3%
%      FLOOR covers the two schemes' independent O(H) biases (a start-up error
%      in the adjoint's x3, an endgame error in the Monte Carlo's last step as
%      tgo -> 1e-5); the siblings measure those at +1.88% and +0.53% against
%      this same closed form.  Without the floor, raising mc_runs far enough
%      would eventually turn a systematic 1% offset into a red light.
%
%      Row 4 is scored ALONGSIDE row 4b, "Monte Carlo vs the closed form",
%      under the identical band.  That pairing is deliberate.  The failure mode
%      this file is exposed to is two implementations that each match a figure
%      but not each other; with both pairings on the board, a break in either
%      one lights exactly one row and names the guilty side.
%
%      Row 4 is ALSO the only scored row that survives off-book parameters.
%      The closed forms are N' = 3 only, so changing XNP skips 1, 2, 3.  The
%      adjoint/Monte-Carlo pairing is valid at any N', any TAU, any nT -- which
%      is what makes this file usable for pricing a parameter rather than only
%      for reproducing page 85.
%
%   5  SHAPE, from prose about Fig. 4.16 (zarchan.txt:1279): "the miss distance
%      is small for both small and large flight times".  Scored as an ordering:
%      the sigma curve must have an INTERIOR maximum over the book's 1..tF s
%      range, with both ends below 80% of it.  The closed form puts the peak at
%      tF/T = 3.16 (sigma = 20.4 ft) with 9.60 ft at tF = 1 and 13.23 ft at
%      tF = 10 -- 47% and 65% of peak.  An ordering check like this is stronger
%      than a number eyeballed off a plot, and it is the row that would catch
%      the TGO divisor in XMUDNT being dropped.
%
%   NOT SCORED, and why (INFO rows):
%     * The book's OWN Monte Carlo sample size.  A 50-run set carries a
%       sampling error of about 9% of its own answer (sqrt((kappa-1)/4n) with
%       kappa ~ 2.73), so "does the 50-run set match" is a question about the
%       draw, not about the port.  The first 50 replications of the large
%       sample are reported as an INFO row so the book's actual experiment is
%       visible, and are not adjudicated.
%     * The peak of the sigma curve.  Figs. 4.16/4.18/4.19 publish no number
%       for it; the "axis([0 10 0 30])" in the listing is a plot range the
%       author chose, not a measurement.
%     * The cost ratio behind zarchan.txt:1328.  Reported as counted right-hand
%       side evaluations, because the sentence is about program structure, not
%       about a quantity the book measured.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED. ***  This file has
%   never been executed -- MATLAB is the caller's, not this author's.  These
%   are the values the mathematics above predicts, so that a first run can be
%   compared against an expectation instead of merely admired:
%
%     sigma_ft at tF = 1,2,3,...,10 s, closed form (exact):
%        9.60  18.02  20.42  19.85  18.43  17.01  15.79  14.79  13.94  13.23
%     peak of the closed form: 20.45 ft at tF = 3.16 s
%     adjoint should sit about +1.4% above each of those at H = 0.01,
%        so ~13.4 ft at tF = 10 s against the book's 13.3
%     BENCHMARK 1 and 2 residuals: order 1.5% of peak, band 4%
%     BENCHMARK 3a: about -0.55%, band 1%
%     BENCHMARK 3b: about +0.9%, band 5%
%
%   If a residual comes back near a band EDGE, that is not a pass to be
%   accepted -- the mechanism above says where it should land, and the gap
%   between "about 1.5%" and "about 4%" is the entire safety margin.
%   Do not widen a band to make a row green.
%
% ---------------------------------------------------------------------------
%   OUTPUTS
%     R.t            adjoint time [s].  This IS the original system's FLIGHT
%                    TIME tF: one sweep, every flight time, one run.
%     R.tgo          the TGO the listing actually divides by (= R.t + 1e-5)
%     R.x1 .. R.x5   the five adjoint states.  x1 is the miss SENSITIVITY to a
%                    step target maneuver [s^2]; x5 = int x1^2 [s^5].
%     R.sigma_ft     XNT*sqrt(x5/tgo) -- the listing's XMUDNT [ft].  THE ANSWER.
%     R.sigma_cf_ft  closed-form counterpart (NaN unless N' = 3)
%     R.x1_cf        closed-form counterpart of x1 (NaN unless N' = 3)
%     R.mc           Listing 4.5 results: .tf .sigma_ft .mean_ft .se_ft .adj_ft
%                    .cf_ft .dev_ft .z .runs .seed, plus .sigma50_ft (the
%                    book's own 50-run set) -- [] if 'montecarlo' is false
%     R.bench        benchmark rows, schema shared with the other ports
%     R.selfcheck    structural residuals (self-consistency, NOT book values)
%     R.pass         true only if at least one row was SCORED and all passed
%     R.verdict      '' when everything passed; otherwise the one-line reason.
%                    ALWAYS FORMED.  'assert' chooses the reaction to it and
%                    nothing else: true errors, false WARNS and still returns
%                    R complete, with both curves and every row attached.  It
%                    is never silent, and it never withholds the evidence --
%                    a port that refuses to return is a port nobody can
%                    diagnose, and benchSuite runs every port with assert off
%                    precisely so that one red row cannot hide the board.
%     R.p            the parameters actually used
%     R.summary      the rendered text block
%
%   See also MGS.ZARCHAN.COVARIANCEHOMING (Listing 5.2 -- the third independent
%   route to the same 13.3 ft), MGS.ZARCHAN.ADJOINTSINGLELAG, MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 4.6 line 1, verbatim.  Feet and seconds.
    opts.XNT  (1,1) double {mustBeFinite, mustBePositive} = 96.6   % ft/s^2 = 3 g
    opts.XNP  (1,1) double {mustBeFinite, mustBePositive} = 3.0    % N-prime
    opts.TAU  (1,1) double {mustBeFinite, mustBePositive} = 1.0    % s
    opts.TF   (1,1) double {mustBeFinite, mustBePositive} = 10.0   % s, sweep end
    opts.H    (1,1) double {mustBeFinite, mustBePositive} = 0.01   % s, RK2 step
    % [] means "the listing's own value".  Default samp = H reproduces the
    % listing exactly (it records every step); a larger samp thins the record.
    opts.samp   double {mustBeReal, mustBeFinite} = []
    % Listing 4.5.  VC is carried ONLY so the cancellation can be demonstrated.
    opts.mc_vc  (1,1) double {mustBeFinite, mustBePositive} = 4000.0  % ft/s
    % [] means the listing's own sweep, for TF = 1:10 s.
    opts.mc_tf  double {mustBeReal, mustBeFinite} = []
    % The book runs 50.  1000 is the default here because 50 cannot ADJUDICATE
    % anything -- see the INFO note on sample size in the header.
    %
    % THE MONTE CARLO DOMINATES THE COST of this file (the adjoint is one pass
    % of TF/H steps; the Monte Carlo is sum(mc_tf)/H steps on a vector of
    % mc_runs).  If it needs to be cheaper inside a suite, LOWER THIS rather
    % than turning 'montecarlo' off: the band widens with it automatically
    % (about 3.3% per SE at 400 runs against about 2.1% at 1000), so the row
    % stays honest, whereas switching the Monte Carlo off deletes the only
    % check that compares the two listings at all.
    opts.mc_runs (1,1) double {mustBeFinite, mustBePositive} = 1000
    opts.seed    (1,1) double {mustBeFinite, mustBeNonnegative} = 20260823
    % 5, not 3.  The agreement row tests TEN flight times at once against a
    % FIXED seed, so a k that is merely "significant" buys a standing chance
    % that a CORRECT port errors on its first run.  At k = 3 that chance is
    % about 3%; at 4, 0.3%; at 5, 4e-5.  It costs almost nothing: the two
    % faults this row exists to catch move sigma by 56% and 32% against a
    % 5-SE band of about 10% at the default sample size.
    opts.ksig    (1,1) double {mustBeFinite, mustBePositive} = 5.0
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options, MATLAB reports nargin = 0 no matter
    % how many were passed (measured, R2025b), so an `if nargin == 0` here
    % would be a mode that never fires.
    opts.montecarlo (1,1) logical = true
    opts.benchmark  (1,1) logical = true
    opts.render     (1,1) logical = true
    opts.plot       (1,1) logical = false
    opts.assert     (1,1) logical = true
end

XNT = opts.XNT;  XNP = opts.XNP;  TAU = opts.TAU;
TF  = opts.TF;   H   = opts.H;

samp = opts.samp;
if isempty(samp)
    samp = H;                    % the listing records every step; see header
end
if ~isscalar(samp) || samp <= 0
    error('mgs:zarchan:shapingFilterAdjoint:samp', ...
        'samp must be a positive scalar or [] (meaning "every step"); got %s.', ...
        mat2str(samp));
end
if H > samp
    error('mgs:zarchan:shapingFilterAdjoint:step', ...
        ['integration step H = %g s is larger than the sample interval ' ...
         'samp = %g s, so the returned curve would be sampled at the step ' ...
         'rate while claiming the coarser interval in its own time vector.'], ...
        H, samp);
end
if H >= TAU
    error('mgs:zarchan:shapingFilterAdjoint:stiff', ...
        ['H = %g s is not smaller than the guidance time constant TAU = %g s. ' ...
         'The lag state would be integrated at or below one sample per time ' ...
         'constant.  RK2 does not blow up here -- it returns a smooth, ' ...
         'entirely wrong curve -- so this is refused rather than warned about.'], ...
        H, TAU);
end
if TF <= 2*H
    error('mgs:zarchan:shapingFilterAdjoint:short', ...
        'TF = %g s is not long enough to take two steps at H = %g s.', TF, H);
end

% Flight times for the Monte Carlo.  The listing's own outer loop is
% "for TF = 1:10", so that is the default, clipped to the sweep actually run.
mc_tf = opts.mc_tf(:).';
if isempty(mc_tf)
    mc_tf = 1:floor(TF);
end
mc_tf = mc_tf(mc_tf > 0 & mc_tf <= TF);

% =========================================================================
%   LISTING 4.6.  Transcribed from zarchan.txt:1330-1337.
% =========================================================================
[t, X, tgo, nEvalAdj] = adjointRun(XNP, TAU, TF, H, samp);
if numel(t) < 2
    % Fewer than two samples is refused, not warned about.  An empty curve
    % returned quietly looks like a converged result; a ONE-sample curve is
    % worse, because diff() on it is empty and several checks below would then
    % try to append an empty verdict to their pass/fail vector -- a "null
    % assignment" error thrown far from the samp value that caused it.
    error('mgs:zarchan:shapingFilterAdjoint:empty', ...
        ['the sweep recorded %d sample(s) at TF = %g s, samp = %g s.  At ' ...
         'least two are needed for any curve comparison to mean anything.'], ...
        numel(t), TF, samp);
end

x1 = X(:,1);  x2 = X(:,2);  x3 = X(:,3);  x4 = X(:,4);  x5 = X(:,5);

% The listing's sampling line, verbatim: XMUDNT = XNT*sqrt(X5/TGO).  The
% divisor is TGO (= TP + 1e-5), NOT TP.  That is where the shaping filter's
% spectral density PHI_s = nT^2/tF enters; see the header.
sigma_ft = XNT*sqrt(x5./tgo);

% ---- closed forms.  N' = 3 ONLY -- see BENCHMARK 1/2 in the header. --------
isN3 = abs(XNP - 3) < 1e-12;
if isN3
    x1_cf    = mntKernel(t, TAU);
    x2_cf    = exp(-t/TAU).*(t - t.^2/(2*TAU));
    x3_cf    = exp(-t/TAU).*(1 - 2*t/TAU + t.^2/(2*TAU^2));
    x4_cf    = exp(-t/TAU).*(t.^2/(2*TAU) - t.^3/(6*TAU^2));
    x5_cf    = (TAU^5/16)*brkKernel(t/TAU);
    % Divided by the SAME tgo the listing uses, so the 1e-5 offset cannot
    % masquerade as a modelling residual at the first few samples.
    sigma_cf = XNT*sqrt(x5_cf./tgo);
else
    nanv = nan(size(t));
    x1_cf = nanv;  x2_cf = nanv;  x3_cf = nanv;  x4_cf = nanv;  x5_cf = nanv;
    sigma_cf = nanv;
end

% =========================================================================
%   LISTING 4.5.  Transcribed from zarchan.txt:1293-1312, vectorised across
%   replications (see header).  This is the second opinion the whole file
%   exists to obtain.
% =========================================================================
MC = [];
nEvalMC = 0;
if opts.montecarlo
    if isempty(mc_tf)
        error('mgs:zarchan:shapingFilterAdjoint:mctf', ...
            ['no Monte Carlo flight times survive: mc_tf must lie in (0, TF] ' ...
             'and TF = %g s.  Running the adjoint alone and reporting a pass ' ...
             'would drop the only check that compares the two listings.'], TF);
    end
    nRuns = round(opts.mc_runs);
    if nRuns < 8
        error('mgs:zarchan:shapingFilterAdjoint:mcruns', ...
            ['mc_runs = %d.  A sample this small has no usable standard ' ...
             'error, so the agreement row would pass on anything.'], nRuns);
    end
    [MC, nEvalMC] = mcSweep(XNP, TAU, XNT, opts.mc_vc, H, mc_tf, nRuns, opts.seed);

    % Line the two listings up on the adjoint's own grid.  Nearest sample, not
    % interpolation: at samp = H the grids differ by <= H/2 and interpolating
    % would blur a real disagreement into a smaller one.
    MC.adj_ft = zeros(size(MC.tf));
    MC.cf_ft  = nan(size(MC.tf));
    MC.t_adj  = zeros(size(MC.tf));
    for k = 1:numel(MC.tf)
        [~, j] = min(abs(t - MC.tf(k)));
        MC.adj_ft(k) = sigma_ft(j);
        MC.cf_ft(k)  = sigma_cf(j);
        MC.t_adj(k)  = t(j);
    end
    MC.dev_ft = MC.sigma_ft - MC.adj_ft;
    MC.z      = MC.dev_ft ./ max(MC.se_ft, realmin);
end

R = struct();
R.t         = t;
R.tgo       = tgo;
R.x1 = x1;  R.x2 = x2;  R.x3 = x3;  R.x4 = x4;  R.x5 = x5;
R.x1_cf     = x1_cf;
R.x5_cf     = x5_cf;
R.sigma_ft  = sigma_ft;
R.sigma_cf_ft = sigma_cf;
R.mc        = MC;
R.p = struct('XNT',XNT, 'XNP',XNP, 'TAU',TAU, 'TF',TF, 'H',H, 'samp',samp, ...
             'mc_vc',opts.mc_vc, 'mc_runs',round(opts.mc_runs), ...
             'mc_tf',mc_tf, 'seed',opts.seed, 'ksig',opts.ksig, ...
             'n_rhs_adjoint',nEvalAdj, 'n_rhs_montecarlo',nEvalMC, ...
             'units','ft, ft/s, ft/s^2; g = 32.2 ft/s^2');

R.selfcheck = structuralChecks(R, XNP, TAU, XNT, opts.mc_vc, H, ...
                               x2_cf, x3_cf, x4_cf, isN3, opts.ksig);

if opts.benchmark
    R.bench = benchmarks(R, opts, isN3, H, TAU, XNT, XNP, TF);
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

% render() reads R.p, NOT opts.  R.p is what the run ACTUALLY used after the
% [] defaults were resolved (samp, mc_tf); printing opts would print the
% caller's empty placeholders beside numbers derived from the resolved values.
R.summary = render(R, isN3, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R, opts); end

% ---- adjudication.  The verdict is FORMED either way; only the reaction to
%      it depends on 'assert'.  A port that errors unconditionally cannot be
%      diagnosed -- the caller never gets R, so nobody can see WHICH side of a
%      comparison was wrong -- and benchSuite runs every port with assert off
%      for exactly that reason.  So: with 'assert' false the result comes back
%      WITH the failing check attached, and a WARNING, never silence.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated.  The closed ' ...
        'forms are N'' = 3 only and the 13.3 ft row needs the listing''s ' ...
        'own case; this run used N'' = %g, TAU = %g s, nT = %g ft/s^2, ' ...
        'TF = %g s.  An unvalidated port must not be handed back silently.'], ...
        XNP, TAU, XNT, TF);
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port ' ...
        'disagrees with ITSELF, which is a wiring fault, not a tolerance ' ...
        'question.  First: %s.'], sum(~R.selfcheck.oks), ...
        R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf(['%d of %d evaluated book values are outside ' ...
        'tolerance (first: %s, book %s, ours %s, band %s).'], ...
        numel(bad), sum(ev), R.bench(bad(1)).what, ...
        R.bench(bad(1)).book_str, R.bench(bad(1)).ours_str, ...
        R.bench(bad(1)).band);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:shapingFilterAdjoint:benchmark', ...
            '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN.  R comes back complete -- R.bench, R.selfcheck,
        % R.verdict and both curves -- so the caller can see WHICH side of a
        % comparison moved.  A port that errors here instead would hand back
        % nothing, and the last time that happened in this package nobody
        % could tell whether the book value or the integrator was at fault.
        warning('mgs:zarchan:shapingFilterAdjoint:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function [t, X, tgo, nEval] = adjointRun(XNP, TAU, TF, H, samp)
%ADJOINTRUN  Listing 4.6, zarchan.txt:1330-1337.
%
%   Every constant this needs is an ARGUMENT.  MATLAB local functions cannot
%   see the main function's workspace; a previous wave in this package put a
%   constant in the caller and read it from a local, and it threw
%   "Unrecognized function or variable" on the first line of the first run --
%   which static checking could not see beforehand.
%
%   Returns nEval, the count of right-hand-side evaluations, for the INFO row
%   that quotes zarchan.txt:1328 on adjoint-vs-Monte-Carlo cost.

% The listing's line 1, verbatim except for the plotting.
S  = 0.0;
TP = 0.0 + 0.00001;      % NOT 0: the adjoint gain is N'/tgo.  See header.
X1 = 0.0;
X2 = 0.0;
X3 = 1.0;                % <<< THE IMPULSE.  Unit area, on the miss node.
X4 = 0.0;
X5 = 0.0;

% Derivatives initialised before the loop so that the FLAG==1 branch inside it
% cannot read a value left over from anything else.
X1D = 0.0;  X2D = 0.0;  X3D = 0.0;  X4D = 0.0;  X5D = 0.0;
TGO = TP + 0.00001;

nmax = ceil(TF/min(samp, H)) + 8;
t   = zeros(nmax,1);
X   = zeros(nmax,5);
tgo = zeros(nmax,1);
n   = 0;
nEval = 0;

while TP <= (TF - 1e-5)
    X1OLD = X1;  X2OLD = X2;  X3OLD = X3;  X4OLD = X4;  X5OLD = X5;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  Pass 1 evaluates the derivatives at
    % the current state; pass 2 takes a full Euler step, advances TP by H, and
    % re-evaluates there.  TP advances INSIDE this loop, exactly once, which is
    % what makes the second evaluation happen at t+H.  Move that line out and
    % the scheme quietly degrades to first order while still running smoothly.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            X1 = X1 + H*X1D;
            X2 = X2 + H*X2D;
            X3 = X3 + H*X3D;
            X4 = X4 + H*X4D;
            X5 = X5 + H*X5D;
            TP = TP + H;
        end
        X1D = X2;
        X2D = X3;
        Y1  = (X4 - X2)/TAU;      % the single guidance lag
        TGO = TP + 0.00001;       % adjoint time IS time-to-go
        X3D = XNP*Y1/TGO;         % the time-varying PN gain N'/tgo
        X4D = -Y1;
        X5D = X1*X1;              % the shaping filter: accumulate x1^2
        FLAG = 1;
        nEval = nEval + 1;
    end
    % The listing writes "FLAG = 0" here as well.  Dropped: the outer loop
    % re-initialises FLAG on its next pass, so the assignment cannot be read.
    % Kept as a comment rather than as code so nobody re-adds it thinking a
    % state machine was lost.

    % Heun corrector: (x_old + x_pred)/2 + (H/2)*f(t+H, x_pred), which equals
    % x_old + (H/2)*(f_old + f_pred).  X1D..X5D here are the SECOND-pass
    % derivatives; averaging against X1OLD is what reintroduces the first.
    X1 = (X1OLD + X1)/2 + 0.5*H*X1D;
    X2 = (X2OLD + X2)/2 + 0.5*H*X2D;
    X3 = (X3OLD + X3)/2 + 0.5*H*X3D;
    X4 = (X4OLD + X4)/2 + 0.5*H*X4D;
    X5 = (X5OLD + X5)/2 + 0.5*H*X5D;

    % ONE increment, tested against samp.  The listing increments S twice and
    % tests "S >= .000999" -- a threshold far below one increment, so it
    % records every step.  Reproducing the literal would silently change
    % meaning for H < 0.0005; reproducing the double increment would halve any
    % samp the caller asks for.  See the header.
    S = S + H;
    if S >= samp - 0.5*H
        S = 0.0;
        n = n + 1;
        t(n)   = TP;
        X(n,:) = [X1 X2 X3 X4 X5];
        tgo(n) = TGO;
    end
end

t = t(1:n);  X = X(1:n,:);  tgo = tgo(1:n);
end


% =============================================================================
function [M, nEval] = mcSweep(XNP, TAU, XNTmag, VC, H, tfList, nRuns, seed)
%MCSWEEP  Listing 4.5's two nested loops, zarchan.txt:1293-1312.
%
%   Outer loop over flight time, inner loop over replications -- with the inner
%   loop turned into an array dimension.  See the header for why that is exact.
%
%   The uniforms come from a Lehmer generator inside this file rather than from
%   rand().  Two reasons, both practical: a benchmark that moves when someone
%   else calls rng() is not a benchmark, and setting the global stream to fix
%   that would silently reach into the caller's session -- inside a suite that
%   runs fifteen ports back to back, that is a real hazard.
%
%   The two draws per replication are taken from SEPARATE BLOCKS of the stream
%   (all the start times first, then all the signs).  Consecutive values of a
%   multiplicative generator lie on a lattice in 2-D; drawing the pair
%   consecutively would couple the maneuver's start time to its sign, which is
%   exactly the correlation that would bias the sample standard deviation.

nT = numel(tfList);
M = struct();
M.tf         = tfList(:).';
M.sigma_ft   = zeros(1,nT);
M.mean_ft    = zeros(1,nT);
M.se_ft      = zeros(1,nT);
M.sigma50_ft = nan(1,nT);
M.mean50_ft  = nan(1,nT);
M.runs       = nRuns;
M.seed       = seed;
M.kurtosis   = zeros(1,nT);

% One continuous stream across the whole sweep: 2 uniforms per replication.
u = lcgUniform(seed, 2*nRuns*nT);
nEval = 0;

for k = 1:nT
    TFk  = tfList(k);
    base = 2*nRuns*(k-1);
    sums = u(base + (1:nRuns));                 % SUM   -> start time
    pzs  = u(base + nRuns + (1:nRuns));         % PZ    -> sign of the maneuver

    % Listing 4.5: TSTART = TF*SUM;  PZ = PZ - .5;  if PZ > 0, COEF = 1, else -1.
    tstart = TFk*sums;
    coef   = ones(1,nRuns);
    coef(pzs - 0.5 <= 0) = -1;

    [Z, ne] = mcBatch(XNP, TAU, XNTmag, VC, H, TFk, tstart, coef);
    nEval = nEval + ne;

    % The book's own estimators: XMEAN over all runs, SIGMA with (RUN-1).
    XMEAN = sum(Z)/nRuns;
    d     = Z - XMEAN;
    SIGMA = sqrt(sum(d.^2)/(nRuns - 1));

    % Standard error of a SAMPLE STANDARD DEVIATION, from the sample's own
    % fourth moment.  The normal-theory form s/sqrt(2n) is WRONG here: COEF
    % makes the sample two-lobed with kurtosis about 2.7, and using 3 would
    % overstate the band by ~15%, i.e. would make the agreement row weaker
    % than it is entitled to be.
    m2 = sum(d.^2)/nRuns;
    m4 = sum(d.^4)/nRuns;
    varS2 = max(m4 - m2^2, 0)/nRuns;
    SE    = sqrt(varS2)/(2*max(SIGMA, realmin));

    M.sigma_ft(k) = SIGMA;
    M.mean_ft(k)  = XMEAN;
    M.se_ft(k)    = SE;
    M.kurtosis(k) = m4/max(m2^2, realmin);

    % The BOOK'S experiment: the first 50 replications.  Reported, never
    % scored -- a 50-run set's own sampling error is ~9% of its answer.
    if nRuns >= 50
        Z50 = Z(1:50);
        m50 = sum(Z50)/50;
        M.sigma50_ft(k) = sqrt(sum((Z50 - m50).^2)/49);
        M.mean50_ft(k)  = m50;
    end
end
end


% =============================================================================
function [Z, nEval] = mcBatch(XNP, TAU, XNTmag, VC, H, TF, tstart, coef)
%MCBATCH  Listing 4.5's inner simulation, run for every replication at once.
%
%   tstart and coef are 1-by-n.  Z is the miss Y at t = TF, one per replication.
%   Constants arrive as arguments; nothing here reads a caller's workspace.

n   = numel(tstart);
Y   = zeros(1,n);
YD  = zeros(1,n);
XNL = zeros(1,n);
T   = 0.0;
nEval = 0;

while T <= (TF - 1e-5)
    % Listing 4.5: "if T < TSTART, XNT = 0; else XNT = COEF*96.6".  The
    % direction of that comparison is the one thing the transcription could
    % not settle from shape -- see the header for the three reasons it is
    % "maneuver STARTS at TSTART", and for the analytic size of the error the
    % other choice makes.  T here is the time at the TOP of the step, which is
    % where the listing evaluates it; XNT is then held across BOTH derivative
    % evaluations, as in the listing.
    XNT = coef .* XNTmag .* double(T >= tstart);

    YOLD = Y;  YDOLD = YD;  XNLOLD = XNL;

    % ---- pass 1: derivatives at the current state -------------------------
    TGO   = TF - T + 0.00001;
    RTM   = VC*TGO;
    % "./" and not "/": RTM is a scalar today, and (row vector)/(scalar) is
    % the same thing -- but if TGO ever becomes a vector, "/" silently turns
    % into a least-squares solve that returns one number and keeps running.
    XLAMD = (RTM*YD + Y*VC) ./ (RTM^2);
    XNC   = XNP*VC*XLAMD;          % VC cancels against RTM; see header
    XNLD  = (XNC - XNL)/TAU;
    YDD   = XNT - XNL;
    nEval = nEval + n;

    % ---- predictor.  ORDER MATTERS: Y uses the OLD YD, YD then uses the OLD
    %      YDD.  Reordering these three lines is a silent change of scheme.
    Y   = Y + H*YD;
    YD  = YD + H*YDD;
    XNL = XNL + H*XNLD;
    T   = T + H;

    % ---- pass 2: derivatives at the predicted state -----------------------
    TGO   = TF - T + 0.00001;
    RTM   = VC*TGO;
    XLAMD = (RTM*YD + Y*VC) ./ (RTM^2);
    XNC   = XNP*VC*XLAMD;
    XNLD  = (XNC - XNL)/TAU;
    YDD   = XNT - XNL;
    nEval = nEval + n;

    % ---- corrector, in the listing's own algebra --------------------------
    Y   = 0.5*(YOLD   + Y   + H*YD);
    YD  = 0.5*(YDOLD  + YD  + H*YDD);
    XNL = 0.5*(XNLOLD + XNL + H*XNLD);
end

Z = Y;
end


% =============================================================================
function u = lcgUniform(seed, n)
%LCGUNIFORM  MINSTD (Lehmer) uniforms in (0,1).  Base MATLAB, deterministic,
%   and it does not touch the global random stream.
%
%   x <- 16807*x mod (2^31 - 1).  16807*x < 3.61e13 < 2^53, so every product
%   is EXACT in double precision -- no silent rounding in the recurrence.
%   x = 0 is an absorbing state, so the seed is mapped into 1..m-1.
m = 2147483647;
x = mod(floor(abs(seed)), m - 1) + 1;
u = zeros(1, n);
for k = 1:n
    x = mod(16807*x, m);
    u(k) = x/m;
end
end


% =============================================================================
function y = mntKernel(t, T)
%MNTKERNEL  zarchan.txt:1345 (and :890): MNT|N'=3 = 0.5*tF^2*exp(-tF/T).
%   The miss SENSITIVITY [s^2] to a step target maneuver.  Multiply by nT
%   [ft/s^2] to get feet.  N' = 3 ONLY -- the caller gates on that.
y = 0.5 * t.^2 .* exp(-t/T);
end


% =============================================================================
function b = brkKernel(x)
%BRKKERNEL  The bracket of the book's closed form, zarchan.txt:1355-1357:
%
%       b(x) = 3 - exp(-2x)*(2x^4 + 4x^3 + 6x^2 + 6x + 3)
%            = 4 * int_0^x s^4 exp(-2s) ds                (identical, exact)
%
%   AS PRINTED IT IS UNUSABLE FOR SMALL x.  b(0.01) is 8.0e-11 and the printed
%   form computes it as 3 minus 2.99999999992 -- every significant digit gone.
%   It does not return NaN; it returns a small plausible number, which would
%   plot as a slightly wrong curve near the origin and be believed.  Below
%   x = 1 this uses the series
%
%       b(x) = 4 x^5 * sum_k (-2x)^k / (k! (k+5))
%
%   which is the same function, exact to machine precision there.
b   = zeros(size(x));
big = x >= 1;

% Both branches are guarded with any().  An all-false logical index on the
% left of an assignment whose right side is empty is a DELETION in MATLAB, not
% a no-op assignment, and this function is called with scalars (tF = 10 s from
% the closed-form benchmark) as well as with the whole sweep.
if any(big(:))
    xb = x(big);
    b(big) = 3 - exp(-2*xb).*(2*xb.^4 + 4*xb.^3 + 6*xb.^2 + 6*xb + 3);
end

if any(~big(:))
    xs   = x(~big);
    s    = zeros(size(xs));
    term = ones(size(xs));                 % (-2x)^k / k!, starting at k = 0
    for k = 0:40
        s    = s + term/(k+5);
        term = term .* (-2*xs)/(k+1);
    end
    b(~big) = 4*xs.^5 .* s;
end

% b(x) >= 0 for x >= 0 by construction (it is an integral of a positive
% integrand).  This clamp exists only to stop a rounding-level negative from
% turning sqrt() complex at the very first sample; it cannot mask a real
% error, because at x >= 1 the value is never below 0.157.
b = max(b, 0);
end


% =============================================================================
function m = mudntKernel(tF, T, nT)
%MUDNTKERNEL  zarchan.txt:1354-1357, N' = 3 only.  Returns FEET.
%
%       MUDNT = (nT/4) * sqrt( (T^5/tF) * b(tF/T) ),   b from brkKernel
%
%   Shares no code with the integrator, which is the whole reason it is worth
%   having: it is a second opinion, not a restatement of the first.
m = (nT/4) * sqrt( (T^5 ./ tF) .* brkKernel(tF/T) );
end


% =============================================================================
function K = structuralChecks(R, XNP, TAU, XNT, VC, H, x2_cf, x3_cf, x4_cf, ...
                              isN3, ksig)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.  They are the weaker
%   claim that it agrees with itself -- which is what catches wiring faults.
K = struct();
names = {};  oks = [];  detail = {};

tolCurve = max(0.005, 4*H);          % same provenance as BENCHMARK 1/2

% 1  TGO really is the recorded time plus 1e-5.  Cheap, and it is the only
%    thing that would notice someone "cleaning up" the divisor in XMUDNT to
%    R.t (a 1e-5 change at tF = 10 s, a 0.1% change at the first sample).
K.tgo_resid = max(abs(R.tgo - (R.t + 1e-5)));
names{end+1} = 'tgo == t + 1e-5';
oks(end+1)   = K.tgo_resid <= 1e-12;
detail{end+1} = sprintf('max |tgo - (t+1e-5)| = %.3e s', K.tgo_resid);

% 2  x5 = int x1^2 cannot decrease.  A sign error on X5D, or X5D wired to a
%    state that changes sign, shows up here and nowhere else cheap.
K.x5_min_step = min(diff(R.x5));
names{end+1} = 'x5 non-decreasing (x5dot = x1^2)';
oks(end+1)   = K.x5_min_step >= -1e-15;
detail{end+1} = sprintf('min consecutive change %+.3e s^5', K.x5_min_step);

% 3-5  x2, x3, x4 against exact solutions of the TRANSCRIBED ODEs, derived in
%      the header.  These are the sharp wiring checks: a swapped derivative or
%      a flipped sign on X4D leaves x1 looking like a plausible miss curve
%      while moving these enormously.
if isN3
    K.x2_resid = maxRel(R.x2, x2_cf);
    K.x3_resid = maxRel(R.x3, x3_cf);
    K.x4_resid = maxRel(R.x4, x4_cf);
    names{end+1} = 'x2 == exp(-t/T)(t - t^2/2T)';
    oks(end+1)   = K.x2_resid <= tolCurve;
    detail{end+1} = sprintf('%.4f%% of peak (band %.4f%%)', 100*K.x2_resid, 100*tolCurve);
    names{end+1} = 'x3 == exp(-t/T)(1 - 2t/T + t^2/2T^2)';
    oks(end+1)   = K.x3_resid <= tolCurve;
    detail{end+1} = sprintf('%.4f%% of peak (band %.4f%%)', 100*K.x3_resid, 100*tolCurve);
    names{end+1} = 'x4 == exp(-t/T)(t^2/2T - t^3/6T^2)';
    oks(end+1)   = K.x4_resid <= tolCurve;
    detail{end+1} = sprintf('%.4f%% of peak (band %.4f%%)', 100*K.x4_resid, 100*tolCurve);
else
    K.x2_resid = NaN;  K.x3_resid = NaN;  K.x4_resid = NaN;
end

% 6  VC cancels out of Listing 4.5.  Two DETERMINISTIC trajectories (fixed
%    start time, fixed sign) at VC and 3*VC.  Analytically identical; see the
%    header.  Any future edit that introduces a real closing-velocity
%    dependence is invisible everywhere else in this file.
if ~isempty(R.mc)
    TFv = R.p.mc_tf(end);
    [Za, ~] = mcBatch(XNP, TAU, XNT, VC,   H, TFv, TFv/2, 1);
    [Zb, ~] = mcBatch(XNP, TAU, XNT, 3*VC, H, TFv, TFv/2, 1);
    K.vc_resid = abs(Za - Zb)/max(abs(Za), realmin);
    names{end+1} = 'Monte Carlo independent of VC';
    % 1e-6, not 1e-12.  VC cancels ALGEBRAICALLY, not bit-for-bit: RTM = VC*TGO
    % puts the two runs on different rounding paths through a step that divides
    % by TGO^2 = 1e-10.  Every plausible way to BREAK the cancellation (dropping
    % the VC from XNC, or from RTM) changes the trajectory by order 1, so this
    % band still discriminates by more than five orders of magnitude.
    oks(end+1)   = K.vc_resid <= 1e-6;
    detail{end+1} = sprintf('|dY|/|Y| = %.3e between VC = %g and %g ft/s', ...
        K.vc_resid, VC, 3*VC);

    % 7  The COEF = +/-1 sign randomisation is actually balanced.  If it is
    %    dropped or biased, the sample mean walks away from zero and SIGMA
    %    stops estimating what the adjoint computes -- a 32% error at tF = 10 s
    %    with no other symptom.  Band: ksig standard errors of the mean.
    % SE of the mean is EXACTLY sigma/sqrt(n) here, because Z = COEF*|Y| has
    % variance sigma^2 by construction.  Limit is at least 6 SE: this is a
    % STRUCTURAL row, so a false red is a hard error on a correct port, and
    % dropping COEF entirely lands at about 34 SE -- there is no daylight
    % between 6 and 34 that anything real occupies.
    seMean   = R.mc.sigma_ft/sqrt(R.mc.runs);
    meanLim  = max(ksig, 6);
    K.mean_z = max(abs(R.mc.mean_ft) ./ max(seMean, realmin));
    names{end+1} = 'Monte Carlo sample mean ~ 0 (COEF is symmetric)';
    oks(end+1)   = K.mean_z <= meanLim;
    detail{end+1} = sprintf('worst |mean| = %.2f SE (limit %.1f SE)', K.mean_z, meanLim);
else
    K.vc_resid = NaN;  K.mean_z = NaN;
end

K.names    = names;
K.oks   = logical(oks);
K.detail   = detail;
K.all_ok   = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    j = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{j}, detail{j});
end
end


function r = maxRel(a, b)
%MAXREL  Max absolute difference NORMALISED BY THE CURVE'S OWN PEAK.
%   Not pointwise relative: x2, x3 and x4 all cross zero, and a pointwise
%   relative error there divides a small number by a smaller one and
%   manufactures a failure out of nothing.
den = max(abs(b));
if ~(den > 0)
    r = Inf;                 % refuse to report "perfect" against a zero curve
    return
end
r = max(abs(a - b))/den;
end


% =============================================================================
function B = benchmarks(R, opts, isN3, H, TAU, XNT, XNP, TF)
%BENCHMARKS  Computed vs published, with the PROVENANCE of every band.
%
%   *** EVERY BOOK VALUE HERE WAS GREPPED OUT OF zarchan.txt AND THE LINE
%   NUMBER IS IN THE .source FIELD. ***  Nothing was read off a figure: the
%   extraction turned every figure into an image link, and earlier waves in
%   this package were handed "benchmarks" that were exactly such phantom
%   readings.  Where the book states a SHAPE about a figure, that prose IS
%   greppable, and an ordering check is usually stronger than a plotted value.
%
%   Bands and what each one means:
%     'text'          1%    prose, three significant figures (13.3 ft)
%     'text+scheme'   1% + 4*H   the above PLUS this integrator's O(H) bias;
%                               formed in code so it cannot be tighter than
%                               either part
%     'analytic'      max(0.5%, 4*H) of peak -- against a closed FORM
%     'statistical'   max(k*SE, 3%) -- against a finite Monte Carlo sample
%     'shape'         an ordering; no tolerance, and none is possible
%
%   Do NOT widen a band to turn a row green.  Bands are set by the provenance
%   of the book value, not by what we happen to compute.

B = emptyBench();
tolCurve = max(0.005, 4*H);
bookCase = isN3 && abs(TAU - 1) < 1e-12 && abs(XNT - 96.6) < 1e-9 && TF >= 10;
whyN3    = sprintf(['the book''s closed forms for this chapter are N'' = 3 ' ...
    'ONLY (zarchan.txt:1345, :1354); this run used N'' = %g'], XNP);
whyCase  = sprintf(['off the listing''s own case -- 13.3 ft is quoted for ' ...
    'N''=3, TAU=1 s, nT=96.6 ft/s^2, tF=10 s; this run used N''=%g, TAU=%g s, ' ...
    'nT=%g ft/s^2, TF=%g s'], XNP, TAU, XNT, TF);

% ---- 1.  x1 against the printed step-maneuver miss -------------------------
if isN3
    r1 = maxRel(R.x1, R.x1_cf);
    B = addBound(B, 'x1(t) == 0.5 t^2 exp(-t/T)  [N''=3]', ...
        sprintf('<= %.4f%% of peak', 100*tolCurve), ...
        sprintf('%.4f%% of peak', 100*r1), r1 <= tolCurve, ...
        'p.86 closed form (zarchan.txt:1345, also :890)', 's^2', ...
        ['exact solution of the transcribed ODEs; catches a misplaced ' ...
         'impulse or a swapped derivative, which the sigma row alone would not']);
else
    B = addSkip(B, 'x1(t) == 0.5 t^2 exp(-t/T)  [N''=3]', ...
        '0.5 t^2 exp(-t/T)', whyN3, ...
        'p.86 closed form (zarchan.txt:1345, also :890)', 's^2');
end

% ---- 2.  the whole sigma curve against the closed form ---------------------
if isN3
    r2 = maxRel(R.sigma_ft, R.sigma_cf_ft);
    B = addBound(B, 'sigma(tF) vs Ch.4 closed form, whole sweep', ...
        sprintf('<= %.4f%% of peak', 100*tolCurve), ...
        sprintf('%.4f%% of peak', 100*r2), r2 <= tolCurve, ...
        'p.86 closed form (zarchan.txt:1354-1357)', 'ft', ...
        ['analytic; shares no code with the integrator.  Residual is the ' ...
         'first-step O(H) offset in x3, predicted at 0.5*N''*H = ' ...
         sprintf('%.2f%%', 100*0.5*XNP*H)]);
else
    B = addSkip(B, 'sigma(tF) vs Ch.4 closed form, whole sweep', ...
        'MUDNT|N''=3', whyN3, 'p.86 closed form (zarchan.txt:1354-1357)', 'ft');
end

% ---- 3a.  the closed form against the book's printed 13.3 ft ---------------
% This row has NO integrator in it.  It asks only whether the formula and the
% constants describe the same problem the book measured.
if bookCase
    cf10 = mudntKernel(10.0, TAU, XNT);
    B = addVal(B, 'closed form at tF = 10 s vs book 13.3 ft', 13.3, cf10, ...
        'text', 0.01, 'p.98 text (zarchan.txt:1536)', 'ft', ...
        ['no integrator involved: this is the formula and the constants ' ...
         'against the printed number']);
else
    B = addSkip(B, 'closed form at tF = 10 s vs book 13.3 ft', '13.3', ...
        whyCase, 'p.98 text (zarchan.txt:1536)', 'ft');
end

% ---- 3b.  the adjoint against the book's printed 13.3 ft -------------------
if bookCase
    [~, j10] = min(abs(R.t - 10.0));
    tolBook = 0.01 + 4*H;      % the 3a band PLUS the row-2 band, formed here
    B = addVal(B, 'adjoint sigma at tF = 10 s vs book 13.3 ft', 13.3, ...
        R.sigma_ft(j10), 'text+scheme', tolBook, ...
        'p.98 text (zarchan.txt:1536)', 'ft', ...
        sprintf(['sampled at t = %.5f s.  Band is 1%% (three printed figures) ' ...
                 'plus 4*H (this scheme''s start-up bias); it can never be ' ...
                 'tighter than either'], R.t(j10)));
else
    B = addSkip(B, 'adjoint sigma at tF = 10 s vs book 13.3 ft', '13.3', ...
        whyCase, 'p.98 text (zarchan.txt:1536)', 'ft');
end

% ---- 4.  LISTING 4.6 vs LISTING 4.5.  The row this file exists for. --------
if ~isempty(R.mc)
    M    = R.mc;
    band = max(opts.ksig*M.se_ft, 0.03*abs(M.adj_ft));
    okA  = all(abs(M.dev_ft) <= band);
    [worstA, iA] = max(abs(M.dev_ft) ./ max(band, realmin));
    B = addBound(B, 'Listing 4.6 vs Listing 4.5 (adjoint vs Monte Carlo)', ...
        sprintf('|d| <= max(%.1f SE, 3%%) at all %d flight times', ...
                opts.ksig, numel(M.tf)), ...
        sprintf('worst %.2f of band, %+.4f ft at tF = %g s (%d runs)', ...
                worstA, M.dev_ft(iA), M.tf(iA), M.runs), okA, ...
        'p.84 text (zarchan.txt:1320) -- "in close proximity"', 'ft', ...
        ['the book''s claim is an adjective, so it is scored as the ' ...
         'statistically correct version of it: SE from the sample''s own ' ...
         '4th moment (kurtosis ~2.7, not 3, because COEF is +/-1)']);

    if isN3
        devB = M.sigma_ft - M.cf_ft;
        bandB = max(opts.ksig*M.se_ft, 0.03*abs(M.cf_ft));
        okB   = all(abs(devB) <= bandB);
        [worstB, iB] = max(abs(devB) ./ max(bandB, realmin));
        B = addBound(B, 'Listing 4.5 vs Ch.4 closed form', ...
            sprintf('|d| <= max(%.1f SE, 3%%) at all %d flight times', ...
                    opts.ksig, numel(M.tf)), ...
            sprintf('worst %.2f of band, %+.4f ft at tF = %g s', ...
                    worstB, devB(iB), M.tf(iB)), okB, ...
            'p.86 closed form (zarchan.txt:1354-1357)', 'ft', ...
            ['the SECOND leg of the triangle.  With both legs on the board, ' ...
             'a break lights exactly one row and names the guilty side -- ' ...
             'two implementations that match a figure but not each other is ' ...
             'the failure mode this file is exposed to']);
    else
        B = addSkip(B, 'Listing 4.5 vs Ch.4 closed form', 'MUDNT|N''=3', ...
            whyN3, 'p.86 closed form (zarchan.txt:1354-1357)', 'ft');
    end
else
    B = addSkip(B, 'Listing 4.6 vs Listing 4.5 (adjoint vs Monte Carlo)', ...
        '"in close proximity"', ...
        ['''montecarlo'' is false, so Listing 4.5 was not run.  This is the ' ...
         'only row that compares the two listings and the only scored row ' ...
         'that survives off-book parameters'], ...
        'p.84 text (zarchan.txt:1320)', 'ft');
    B = addSkip(B, 'Listing 4.5 vs Ch.4 closed form', 'MUDNT|N''=3', ...
        '''montecarlo'' is false', ...
        'p.86 closed form (zarchan.txt:1354-1357)', 'ft');
end

% ---- 5.  THE SHAPE CLAIM ---------------------------------------------------
% zarchan.txt:1279, about Fig. 4.16: "the miss distance is small for both
% small and large flight times".  Prose about a figure IS greppable, and this
% ordering is what would catch the TGO divisor being dropped from XMUDNT.
win = R.t >= 1.0;                         % the book's figure starts at tF = 1 s
tw  = R.t(win);  sw = R.sigma_ft(win);
% Three samples is the minimum at which "interior peak" is a statement about
% the curve rather than about the sampling.  A coarse samp would otherwise
% report FAIL for a perfectly correct run whose record simply has no middle.
if TF >= 5 && numel(sw) >= 3
    [pk, ip] = max(sw);
    interior = ip > 1 && ip < numel(sw);
    endsLow  = interior && (sw(1) <= 0.8*pk) && (sw(end) <= 0.8*pk);
    B = addShape(B, 'sigma small at BOTH ends, interior peak', ...
        'rise then fall over tF = 1..TF s, both ends <= 80% of peak', ...
        sprintf('peak %.3f ft at tF = %.2f s; ends %.1f%% and %.1f%% of peak', ...
                pk, tw(ip), 100*sw(1)/pk, 100*sw(end)/pk), ...
        interior && endsLow, 'p.81 text (zarchan.txt:1279)', ...
        ['the closed form puts the peak at tF/T = 3.16.  Drop the TGO ' ...
         'divisor from XMUDNT and this curve rises monotonically instead']);
else
    B = addSkip(B, 'sigma small at BOTH ends, interior peak', ...
        'rise then fall', ...
        sprintf(['TF = %g s with %d sample(s) at or past tF = 1 s.  Either ' ...
                 'the sweep does not reach past the peak at tF/T = 3.16 (no ' ...
                 'falling limb to check) or samp is too coarse to have a ' ...
                 'middle -- both would make "interior peak" a statement about ' ...
                 'the sampling rather than about the curve'], TF, numel(sw)), ...
        'p.81 text (zarchan.txt:1279)', 'ft');
end

% ---- INFO rows.  Reported, never scored. ----------------------------------
[pkAll, ipAll] = max(R.sigma_ft);
B = addInfo(B, 'peak of the sigma curve', '--', ...
    sprintf('%.3f ft at tF = %.3f s', pkAll, R.t(ipAll)), ...
    'Figs. 4.16/4.18/4.19 -- shape only', ...
    ['the book publishes no peak value; "axis([0 10 0 30])" in the listing ' ...
     'is a plot range the author chose, not a measurement']);

if ~isempty(R.mc) && any(isfinite(R.mc.sigma50_ft))
    d50 = R.mc.sigma50_ft - R.mc.adj_ft;
    [w50, i50] = max(abs(d50));
    B = addInfo(B, 'the BOOK''s own 50-run sets (Listing 4.5, RUN = 50)', ...
        '"in close proximity"', ...
        sprintf('worst |d| = %.3f ft (%.1f%%) at tF = %g s', ...
                w50, 100*w50/max(abs(R.mc.adj_ft(i50)), realmin), R.mc.tf(i50)), ...
        'p.81/p.84 text (zarchan.txt:1279, :1320)', ...
        ['NOT SCORED: a 50-run standard deviation carries ~9% sampling error ' ...
         'of its own (sqrt((kappa-1)/4n), kappa ~ 2.73), so agreement at ' ...
         'n = 50 is a statement about the draw, not about the port']);
end

if ~isempty(R.mc)
    B = addInfo(B, 'cost: one adjoint run vs ten 50-run Monte Carlo sets', ...
        '"equivalent of ten 50-run Monte Carlo sets"', ...
        sprintf('%d adjoint RHS evals vs %d Monte Carlo RHS evals (%.0fx)', ...
                R.p.n_rhs_adjoint, R.p.n_rhs_montecarlo, ...
                R.p.n_rhs_montecarlo/max(R.p.n_rhs_adjoint,1)), ...
        'p.84 text (zarchan.txt:1328)', ...
        ['NOT SCORED: the sentence is about program STRUCTURE, and the ratio ' ...
         'here depends on mc_runs, which this port raised from the book''s 50']);
end

B = addInfo(B, 'what a dispersion costs us today', '--', ...
    'one sortie per sample', ...
    'project context, not Zarchan', ...
    ['this listing is the one-run column.  It does NOT transfer directly: ' ...
     'the adjoint identity is a LINEARITY identity and our dominant error ' ...
     'source is a lag that SATURATES']);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want: a struct array
%   that quietly reordered fields would make [B.ok] mean something else.
%   Schema is the one mgs.zarchan.benchSuite's shared benchRows() consumes.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
% Tolerance arrives EXPLICITLY rather than through a band->tol lookup, because
% two of the bands here are built from H at the call site and a lookup table
% would have to keep a second copy of that arithmetic.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.4g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.5g', ours);
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
%ADDSKIP  A check that COULD NOT RUN.  Never a pass: it is counted as a check
%   that did not happen, which is a different thing from one that succeeded.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'skip';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = '--';
B(i).pct  = NaN;   B(i).band = 'skip';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end


% =============================================================================
function s = render(R, isN3, ev)
L = {};
p = R.p;
L{end+1} = 'Zarchan Listing 4.6 -- ADJOINT MODEL USING SHAPING FILTER APPROACH';
L{end+1} = '   (with Listing 4.5, the Monte Carlo it must agree with)';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['   nT = %g ft/s^2 (%.4g g)   N'' = %g   TAU = %g s   ' ...
    'TF = %g s   H = %g s   samp = %g s'], p.XNT, p.XNT/32.2, p.XNP, p.TAU, ...
    p.TF, p.H, p.samp);
if ~isempty(R.mc)
    L{end+1} = sprintf(['   Monte Carlo: %d runs at each of %d flight times ' ...
        '(seed %g, VC = %g ft/s, k = %.1f SE)'], p.mc_runs, numel(p.mc_tf), ...
        p.seed, p.mc_vc, p.ksig);
else
    L{end+1} = '   Monte Carlo: NOT RUN (''montecarlo'', false)';
end
L{end+1} = '';

% ---- the answer, on the book's own flight-time grid -----------------------
L{end+1} = 'MISS-DISTANCE STANDARD DEVIATION vs FLIGHT TIME  (the listing''s XMUDNT)';
if ~isempty(R.mc)
    L{end+1} = sprintf('%9s %12s %12s %12s %12s %10s %10s', ...
        'tF [s]', 'adjoint', 'closed form', 'MonteCarlo', 'MC 50-run', 'SE', 'd/SE');
else
    L{end+1} = sprintf('%9s %12s %12s %14s', 'tF [s]', 'adjoint', 'closed form', 'x1 [s^2]');
end

if ~isempty(R.mc)
    for k = 1:numel(R.mc.tf)
        L{end+1} = sprintf('%9.3f %12.4f %12s %12.4f %12s %10.4f %10.2f', ...
            R.mc.tf(k), R.mc.adj_ft(k), numOrDash(R.mc.cf_ft(k)), ...
            R.mc.sigma_ft(k), numOrDash(R.mc.sigma50_ft(k)), ...
            R.mc.se_ft(k), R.mc.z(k)); %#ok<AGROW>
    end
else
    stride = max(1, round(0.5/p.samp));
    rows = unique([1:stride:numel(R.t), numel(R.t)]);
    for r = rows
        L{end+1} = sprintf('%9.3f %12.4f %12s %14.6f', ...
            R.t(r), R.sigma_ft(r), numOrDash(R.sigma_cf_ft(r)), R.x1(r)); %#ok<AGROW>
    end
end
L{end+1} = '';
if ~isN3
    L{end+1} = sprintf(['NOTE: N'' = %g, so the "closed form" column is empty. ' ...
        'The Ch.4 formulas are'], p.XNP);
    L{end+1} = '      N'' = 3 ONLY (zarchan.txt:1345, :1354).  Only the adjoint-vs-';
    L{end+1} = '      Monte-Carlo row is scored at this N''.';
    L{end+1} = '';
end

% ---- structural block ----------------------------------------------------
K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-46s %s', vstr(K.oks(i)), K.names{i}, ...
        K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

% ---- benchmark board -----------------------------------------------------
if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning';
    L{end+1} = '            NOT CHECKED, which is not the same thing as "fine".';
else
    L{end+1} = 'BENCHMARKS  (book values grepped from zarchan.txt; line numbers in .source)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
            case 'info'
                L{end+1} = sprintf('   INFO %-52s %s', b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case 'skip'
                L{end+1} = sprintf('   SKIP %-52s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case {'shape','bound'}
                L{end+1} = sprintf('   %-4s %-52s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf('   %-4s %-52s book %s %s / ours %s %s (%+.3f%%, band %.4g%%)', ...
                    vstr(b.ok), b.what, b.book_str, b.units, b.ours_str, ...
                    b.units, b.pct, 100*b.tol); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    nsc = sum(ev);
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped/info -> R.pass = %s', ...
        nsc, sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: x1 is a SENSITIVITY [s^2], not a miss.  sigma_ft already';
L{end+1} = '   carries the nT and the 1/tF of the shaping filter.  And the adjoint';
L{end+1} = '   identity is a LINEARITY identity: our own dominant error source is a';
L{end+1} = '   lag that SATURATES, which no adjoint can represent.';
s = strjoin(L, newline);
end


function s = numOrDash(v)
if isfinite(v)
    s = sprintf('%12.4f', v);
else
    s = sprintf('%12s', '--');
end
end

function s = vstr(tf)
if tf
    s = 'PASS';
else
    s = 'FAIL';
end
end


% =============================================================================
function drawCurves(R, opts)
%DRAWCURVES  Base MATLAB graphics only.  The listing's own axis([0 10 0 30])
%   is applied to the first figure so the shape can be compared with Fig. 4.18
%   directly, but it is NOT allowed to clip a curve silently: if the peak
%   leaves the box, the limit is widened and the title says so.
figure;
hold on; grid on;
plot(R.t, R.sigma_ft, '-');
lg = {'Listing 4.6 adjoint'};
if any(isfinite(R.sigma_cf_ft))
    plot(R.t, R.sigma_cf_ft, '--');
    lg{end+1} = 'Ch.4 closed form';
end
if ~isempty(R.mc)
    plot(R.mc.tf, R.mc.sigma_ft, '+');
    lg{end+1} = sprintf('Listing 4.5 Monte Carlo (%d runs)', R.mc.runs);
end
xlabel('Flight Time (S)');
ylabel('Miss Dist Standard Deviation (Ft)');
top = 30;
clipped = max(R.sigma_ft) > top;
if clipped
    top = 1.1*max(R.sigma_ft);
    title('Adjoint model using shaping filter approach (y-axis WIDENED past the book''s 30 ft)');
else
    title('Adjoint model using shaping filter approach');
end
axis([0 opts.TF 0 top]);
legend(lg{:}, 'Location', 'best');
hold off;

if any(isfinite(R.x1_cf))
    figure;
    hold on; grid on;
    plot(R.t, R.x1, '-');
    plot(R.t, R.x1_cf, '--');
    xlabel('Flight Time (S)');
    ylabel('Miss sensitivity to step target maneuver (s^2)');
    title('x1 vs the book''s 0.5 t^2 exp(-t/T)   [N'' = 3]');
    legend('adjoint x1', 'closed form', 'Location', 'best');
    hold off;
end
end
