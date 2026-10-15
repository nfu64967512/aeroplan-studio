function R = airframeLinear(opts)
%MGS.ZARCHAN.AIRFRAMELINEAR  Zarchan Listing 21.2 -- LINEAR AIRFRAME
%   SIMULATION, ported line for line, with the transfer-function poles and
%   zeros taken with ROOTS instead of asserted.
%
%   R = mgs.zarchan.airframeLinear()                      % book run + board
%   R = mgs.zarchan.airframeLinear('benchmark', false, 'alt', 50000)
%   R = mgs.zarchan.airframeLinear('benchmark', false, 'deldeg', 10)
%   R = mgs.zarchan.airframeLinear('benchmark', false, ...
%           'trim', 'accel', 'xncg', 10)                  % the Ch.22 trim
%
%   The linearised twin of Listing 21.1.  Same missile (Fig. 21.3), same
%   flight condition, same 5-deg fin step -- but the aerodynamics are frozen
%   at a TRIM angle of attack, which collapses the airframe to the second
%   order transfer functions of p.485:
%
%       nL       K1 (1 - s^2/wz^2)              thd      K3 (1 + Ta s)
%       -- = ------------------------------ ,   --- = ------------------------
%       del   1 + 2 zAF s/wAF + s^2/wAF^2       del   1 + 2 zAF s/wAF + s^2/wAF^2
%
%   with wAF = sqrt(-Ma), zAF = Za*wAF/(2*Ma), wz^2 = (Ma*Zd - Za*Md)/Zd,
%   K1 = -VM(Ma*Zd - Md*Za)/(1845*Ma), Ta = Md/(Ma*Zd - Md*Za), K3 = 1845*K1/VM.
%
% ===========================================================================
%   *** UNITS ARE THE BOOK'S.  FEET, ft/s, g = 32.2 ft/s^2, DEGREES. ***
%
%   Zarchan works in feet with g = 32.2 and converts rad->deg with the literal
%   57.3.  The rest of mgs is metres.  This file is DELIBERATELY left in the
%   book's units so the published numbers compare digit for digit with no
%   conversion sitting in between to be got wrong.  "13 g" and "25.3 rad/s"
%   are plausible in either system, so a silent conversion produces a number
%   that looks right and is wrong by 3.28x.  Convert at the CALL SITE, visibly.
%
%   Inside the transfer functions: del and eps are DEGREES, thd is deg/s, nL
%   is g, wAF/wz are rad/s, Ma/Md are 1/s^2, Za/Zd are 1/s, K1 is g/deg, K3 is
%   1/s.  The mixed-unit constant 1845 is the book's rounding of 57.3*32.2 =
%   1845.06; see the SELF-CHECK note about the 3.3e-5 residual it leaves.
% ===========================================================================
%
%   WHY THIS LISTING, AND NOT 21.1
%
%   21.1 integrates the nonlinear force and moment equations.  If you want to
%   know how fast that airframe is you run it, look at the trace and FIT a
%   time constant.  That is the position this project has been stuck in: we
%   measured a gamma-tracking gain of 0.063 against a model's 1.385 and about
%   8 s of lag on the real vehicle, and we cannot DERIVE the 8 s from
%   anything -- only fit it, sortie by sortie.
%
%   21.2 is the twin where the dynamics are a POLYNOMIAL.  The characteristic
%   polynomial's roots are the airframe's poles; the numerator's roots are its
%   zeros; the time constant falls out of the pole instead of out of a curve
%   fit.  R.lag carries three numbers that are the point of the exercise:
%
%       tau_wn_s        1/wAF                  the book's own definition
%                                              (zarchan.txt:6784, p.492-493)
%       tau_env_s       1/(zAF*wAF)            how long the ringing ACTUALLY
%                                              lasts -- the pole's real part
%       tau_env_meas_s  fitted from the        the same number, measured off
%                       Listing 21.2 run       the integration, agreeing to
%                                              0.1-0.3%
%
%   Two independent computations of one lag, agreeing.  At the book's sea
%   level condition 1/wAF = 0.040 s but 1/(zAF*wAF) = 0.79 s: the airframe
%   REACHES its acceleration in 40 ms and is still ringing about it 0.8 s
%   later, because zAF = 0.0506.  Quoting 1/wAF alone as "the airframe time
%   constant" -- which the book does, and says it is doing -- understates how
%   long the transient is present by a factor of 1/zAF = 20.  Our own vehicle
%   has no published damping at all, so the analogous mistake there is not
%   even visible.
%
%   WHAT THIS DOES NOT MODEL, AND MUST NOT BE REPORTED AS IF IT DID.
%   Constant speed, constant altitude, SUPERSONIC linearised aero (the 8*S/
%   (beta*Sref) terms are a supersonic-fin approximation and beta = sqrt(M^2-1)
%   is imaginary below Mach 1), pitch plane only, no gravity, no drag, no
%   thrust, no actuator, no autopilot, no acceleration or fin limit, and the
%   aero coefficients frozen at ONE trim angle of attack.  It is an open-loop
%   fin-step response of a bare airframe, not a guidance simulation.
%
% ---------------------------------------------------------------------------
%   NO CONTROL SYSTEM TOOLBOX.  WHY THE LONG WAY ROUND.
%
%   A reader who owns the toolbox will reach for tf/ss/step/damp/bode and
%   wonder why this file does not.  mgs is base MATLAB only, so:
%
%     poles      roots(den) where den is the DENOMINATOR COEFFICIENT VECTOR in
%                descending powers of s.  The book writes the denominator as
%                1 + 2*zAF*s/wAF + s^2/wAF^2, i.e. [1/wAF^2, 2*zAF/wAF, 1].
%                ROOTS normalises by the leading coefficient itself, so the
%                monic form [1, 2*zAF*wAF, wAF^2] has the same roots; the
%                self-check computes BOTH and requires them equal, which is
%                the cheapest possible proof that the polynomial was assembled
%                in the right order.  (Reversed coefficients would give roots
%                at the RECIPROCALS -- 0.0025 +/- 0.04i instead of -1.26 +/-
%                24.94i -- and a reciprocal pole pair still looks like a
%                lightly damped second-order system on a printout.)
%     damp       wn = abs(pole), zeta = -real(pole)/abs(pole).  Recovering the
%                listing's own wAF/zAF from its own roots is an identity, and
%                it is checked at 1e-12 rather than assumed.
%     zeros      roots of the numerator.  For nL/del the numerator is
%                K1*(1 - s^2/wz^2) -> [-K1/wz^2, 0, K1] -> roots at +/-wz.
%                The one at +wz is the RIGHT-HALF-PLANE zero (Ch.23, p.549,
%                zarchan.txt:7687).
%     bode       G(j*w) = polyval(num, 1i*w) ./ polyval(den, 1i*w).  That is
%                all freqresp does for a SISO polynomial ratio.
%     step       the listing's own Heun integration, which is the point.
%
%   POLYVAL WITH A COMPLEX ARGUMENT IS FINE in base MATLAB.  What is NOT fine
%   is letting a complex number reach the TIME domain: see the wAF guard.
%
% ---------------------------------------------------------------------------
%   PROVENANCE, AND THE OCR DAMAGE, ITEM BY ITEM
%
%   Transcription: matlab/zarchan/_listings/listing_21_02.m, from
%   zarchan.txt:6748-6769 (book pp.488-491).
%
%   THIS LISTING CONTAINS NO VARIABLE-TO-VARIABLE COMPARISON.  Every damaged
%   comparison in it is variable-to-CONSTANT and was forced by shape; nothing
%   was left for me to resolve by guessing.  For the record:
%
%     while T,1.99999    -> while T < 1.99999     T grows from 0 by H
%     while STEP,=1      -> while STEP <= 1       STEP is 1, then set to 2
%                                                 ('<' would never enter)
%     if S.=.0099999     -> if S >= .0099999      S accumulates and is reset
%                                                 ('<=' would fire every step)
%     if ALT,=30000.     -> if ALT <= 30000.      see the paragraph below
%
%   THE ATMOSPHERE BRANCH IS THE ONE WORTH ARGUING ABOUT.  The direction is
%   forced: at ALT = 0 the true branch must give RHO = .002378 (standard sea
%   level density), so the test is a LOW-altitude test, not a high one.  Get
%   the sense backwards and sea level runs at .0034 slug/ft^3, +43% dynamic
%   pressure, +20% on wAF, and every number in this file is quietly wrong
%   while still looking like a missile.  What is NOT forced is '<' vs '<=',
%   and it only matters at exactly 30000 ft, where the two branches give
%   8.7482e-4 and 8.6948e-4 -- 0.61% apart in RHO, 0.31% in wAF.  The
%   altitude sweep below deliberately INCLUDES 30000 ft so the choice is
%   exercised rather than untested.  All seven listings in the book that carry
%   this preamble (21.1, 21.2, 22.1, 22.2, 23.1, 23.2, 23.3) were repaired the
%   same way, so this port is at least consistent with its siblings.
%
%   The book's very long semicolon-separated lines are the original
%   formatting, not damage.  XK2=XK1 is in the listing and is never used; it
%   is kept, and named, so a reader diffing against p.489 does not go looking
%   for it.
%
%   PULLED IN, AND SAID SO -- TWICE:
%
%   (1) LISTING 21.1 (zarchan.txt:6510-6526), verbatim, as NONLINEARTWIN().
%       21.2 does not need it to run.  It is here because EVERY published
%       statement about Listing 21.2 is a statement about how the linear
%       result compares to the NONLINEAR one, and without 21.1 in the same
%       file there is nothing to compare to -- the port would be scored
%       against itself.  See the section on what the book actually publishes.
%
%   (2) THE TRIM SOLVE FROM LISTING 22.1 (zarchan.txt:6958), as
%       TRIMACCEL().  21.2 trims on a given FIN DEFLECTION; 22.1 trims on a
%       given ACCELERATION.  It is here for one reason: Table 22.1 and Table
%       22.2 publish wAF, zAF, wz, K1, Ta and K3 for this exact airframe at
%       two altitudes, and those six numbers are the only HARD numeric
%       benchmark that exists for the quantities this listing computes.  They
%       are quoted at a 10-g trim, so reaching them needs 22.1's quadratic.
%       Twelve table entries, all reproduced inside 1%.  Nothing else from
%       Chapter 22 was taken -- no autopilot, no gain, no closed loop.
%
% ---------------------------------------------------------------------------
%   THE OCR ATE A MINUS SIGN, AND IT MATTERS -- IN BOTH DIRECTIONS
%
%   In this extraction a leading minus sign sometimes survives as the digit
%   '2'.  It is demonstrable, not a guess:
%
%     zarchan.txt:6972  "more negative than 20.005"   -> -0.005
%     zarchan.txt:6972  "more positive than 0.04"     ->  0.04   (untouched)
%     Table 22.1        K1 = "23.07 g/deg"            -> -3.07
%     Table 22.2        K3 = "21.89 1/s"              -> -1.89
%
%   The last two PROVE each other.  The listing defines K3 = 1845*K1/VM with
%   VM = 3000, so K3 must be 0.615*K1.  Read as printed: 1845*23.07/3000 =
%   14.19, not 21.89 -- the table would be internally inconsistent.  Read with
%   the minus restored: 1845*(-3.07)/3000 = -1.888 vs the published -1.89, and
%   at 50 kft 1845*(-0.559)/3000 = -0.3438 vs -0.344.  Both to three figures.
%   The benchmark board scores against the RESTORED values and says so on
%   every row.  Scoring against "+23.07" would fail a correct port by 850%.
%
%   THE SAME DAMAGE HITS THE PROSE, and here it changed what the sentence
%   MEANS.  zarchan.txt:6527 reads:
%
%     "...causes the missile to build up to a NEGATIVE angle of attack.  The
%      angle of attack initially approaches 28 deg, oscillates, and finally
%      approaches a steady-state or trim value of approximately 25 deg..."
%
%   and Fig. 21.4's caption (zarchan.txt:6530) says "approximately 25-deg
%   angle of attack".  Taken at face value that is a 25-deg trim, which for
%   this airframe would be roughly 5x the acceleration the very next paragraph
%   quotes.  They are -8 deg and -5 deg.  The sentence says "negative" in
%   words two clauses earlier; our transcription of the book's OWN Listing
%   21.1 peaks at -8.32 deg and trims at -4.45 deg; and Listing 21.2's trim
%   quadratic, solved analytically, gives -4.4457 deg.  The board scores
%   |alpha| against 8 and 5 and prints the reasoning in the row detail.
%
%   ONE PLACE THE MINUS DID SURVIVE AS NOTHING AT ALL: "the steady-state
%   acceleration due to a 5-deg fin deflection is approximately 13 g"
%   (zarchan.txt:6528) has no '2' in front of it even though nL is negative
%   here (-12.97 g).  So the corruption is not systematic and you cannot
%   pattern-match your way through it -- each number has to be settled against
%   something computable.  Every restored sign in this file is justified by an
%   identity or by the book's own simulation, never by "it looks like a minus".
%
% ---------------------------------------------------------------------------
%   WHAT THE BOOK PUBLISHES ABOUT LINEAR-vs-NONLINEAR: A DIRECTION, NEVER A
%   NUMBER.  This was the specific question, so it gets a specific answer.
%
%   Grepping every sentence in Ch.21 that compares the two models:
%
%     zarchan.txt:6747  Fig. 21.9  (sea level, 5 deg)
%         "a near perfect match to the nonlinear airframe results"     ADJECTIVE
%     zarchan.txt:6770  Fig. 21.10 (sea level, 10 deg)
%         "the linear approximation to the airframe starts to
%          deteriorate"                                               DIRECTION
%     zarchan.txt:6770  the stated MECHANISM
%         "The angle of attack squared terms ... prevent the linear
%          model from being perfect.  The linear model is a less
%          accurate representation of reality when the angle of
%          attack is large."                                          DIRECTION
%     zarchan.txt:6771  Fig. 21.11 (50 kft, 5 deg)
%         "we would expect our linear model to be less accurate ...
%          the linear model still approximates reality very well"     DIRECTION
%                                                                     + ADJECTIVE
%
%   There is no published number for the agreement.  Not a percentage, not an
%   rms, not a peak error, not anywhere in the chapter.  The four figures that
%   would carry one became image links in the extraction.
%
%   So the linear-vs-nonlinear rows on the board are ORDERINGS, which is what
%   the book actually asserts, and they are stronger than a number read off a
%   plot would have been:
%
%       rms|linear - nonlinear| / |K1*del|      sea level  5 deg   6.60%
%                                               50 kft     5 deg  11.61%
%                                               sea level 10 deg  16.93%
%
%   SCORED:  10-deg > 5-deg at sea level         ("starts to deteriorate")
%   SCORED:  50-kft 5-deg > sea-level 5-deg      ("expect ... less accurate")
%   NOT SCORED: "a near perfect match" and "still ... very well" are adjectives
%       with no number attached, and they are reported as INFO carrying the
%       measured figure.  Note that the second one points slightly the other
%       way: at 50 kft the normalised error is nearly DOUBLE sea level's, which
%       is exactly what the book's own reasoning predicts one paragraph
%       earlier.  This file does not widen "very well" to cover 11.6%; it
%       prints both and lets the reader see the tension.
%
% ---------------------------------------------------------------------------
%   THE OTHER THING WORTH KNOWING: FIG. 21.6 IS NOT ABOUT TRIM
%
%   "at higher altitudes the missile will pull more angle of attack ... for a
%   given fin deflection" (zarchan.txt:6529).  For a given fin deflection that
%   is FALSE at trim, and the listing itself is the proof: the trim condition
%   is CM = 0, and CM is Q*Sref*d*(...) -- Q divides straight out.  The trim
%   angle of attack therefore depends on MACH and geometry only, and this port
%   computes -4.4457 deg at sea level and -4.4457 deg at 50 kft, identical to
%   the last bit (the self-check requires the residual to be exactly 0).
%
%   What IS true, and what Fig. 21.6 shows over its 2-s window, is that the
%   TRANSIENT is bigger: peak |alpha| goes 8.32 -> 8.70 deg, +4.6%, because
%   wAF has dropped from 25.0 to 9.6 rad/s and the first overshoot has not
%   been damped yet.  The board scores the peak, not the trim, and the row
%   says so -- the margin is only 4.6% and a reader is entitled to know that.
%
%   The book's sentence becomes unambiguously true the moment the trim is set
%   by an ACCELERATION instead of a fin angle, which is what Ch.22 does: at a
%   10-g command the trim goes 2.81 deg -> 14.74 deg between sea level and
%   50 kft, and the fin needed goes -3.26 deg -> -17.87 deg (past any real fin
%   limit, which is the point the chapter is making).  Both framings are on
%   the board, separately labelled, because they are different claims.
%
% ---------------------------------------------------------------------------
%   BENCHMARK MODE IS A FLAG.  IT IS NOT nargin.
%
%   In a function whose arguments block declares ONLY name-value options,
%   MATLAB reports nargin = 0 NO MATTER HOW MANY were passed (measured on
%   R2025b).  Three ports in an earlier wave used `if nargin == 0`: one became
%   infinite recursion and two silently replaced the caller's arguments with
%   the benchmark run.  'benchmark' is the ONLY switch here.
%
%   It defaults TRUE, so a bare call is a CHECKED call.  Benchmark mode never
%   overrides anything the caller passed; instead every book value that
%   depends on a parameter the caller moved is SKIPPED with the reason
%   printed.  That is why the arguments-block defaults are [] and the book
%   constants live in BOOKNOMINAL(): "the caller supplied this" is then
%   exactly "the field is non-empty", with no second copy of the constants to
%   drift out of agreement.
%
%   WITH benchmark = false, R.pass is FALSE, not true.  Nothing was checked;
%   "not checked" and "checked and correct" must not print alike.
%
% ---------------------------------------------------------------------------
%   FOUR THINGS THIS FUNCTION REFUSES TO DO
%
%   Each of these produces a PLAUSIBLE WRONG ANSWER if left alone, which is
%   the failure mode this repo keeps getting burned by:
%
%   1. MACH <= 1.  beta = sqrt(XMACH^2 - 1) goes imaginary and MATLAB
%      propagates complex numbers through the whole aero block without a
%      murmur.  real(nL) comes out finite and roughly missile-shaped.
%   2. Ma >= 0.  wAF = sqrt(-Ma) is then imaginary.  Zarchan says in the text
%      that "a positive Ma indicates that the bare airframe is unstable"
%      (zarchan.txt:6799) -- an unstable airframe has a REAL right-half-plane
%      pole, not a natural frequency, and the second-order form in the listing
%      does not describe it.  The integration would still run and still
%      produce numbers.
%   3. NO REAL TRIM (negative discriminant).  Same story: complex ALFTR.
%   4. STEP SIZE TOO LARGE FOR THE POLE.  This is the subtle one.  Heun on a
%      purely oscillatory mode has amplification |1 + z + z^2/2| =
%      sqrt(1 + (w*h)^4/4) > 1 -- it ANTI-DAMPS, at about (w*h)^4/8 per step,
%      i.e. w^4*h^3/8 per second.  The airframe's real damping only removes
%      zAF*w per second, and zAF is 0.019 at 50 kft.  Requiring the numerical
%      anti-damping to stay under a tenth of the physical damping gives
%
%          h <= (0.8*zAF)^(1/3) / wAF
%
%      = 0.0138 s at sea level (the book runs 0.0025, a 5.5x margin) and
%      0.0260 s at 50 kft.  Measured on a bit-identical transcription of this
%      loop run outside MATLAB, the first overshoot at sea level moves
%      -28.47 g (h = 0.0025) -> -29.01 (h = 0.01) -> -30.95 (h = 0.02) ->
%      -34.67 (h = 0.03): +22% on a step size the book never suggested, with
%      no warning of any kind.  The bound is enforced, not warned about.
%
%   None of the four is a tolerance.  They are all "this quantity is not the
%   thing it is named after any more".
%
% ---------------------------------------------------------------------------
%   OUTPUT
%
%   R.fc          the nominal flight condition: .alt .vm .mach .rho .q,
%                 .trim (.mode .alftr_deg .deltr_deg .root_discarded_deg
%                 .cm_residual), .aero (.cna .cnd .cma .cmd .xma .xmd .za .zd),
%                 .tf (.waf .zaf .wz .xk1 .xk2 .ta .xk3)
%   R.tf          taken with ROOTS, not asserted: .poles .zeros_nl .zero_thd
%                 .wn .zeta .rhp_zero .rhp_zero_rad_s, .w .mag_nl .phase_nl_deg
%                 .dc_gain .peak_mag .peak_w
%   R.lag         .tau_wn_s .tau_env_s .tau_env_meas_s .n_peaks .cycles_in_run
%   R.lin         Listing 21.2: .t .xnl .thd  (200 samples, 0.01 s apart)
%   R.non         Listing 21.1: .t .xnl_g .alf_deg  (same grid, same length)
%   R.cmp         one entry per book case (SL 5 deg, SL 10 deg, 50 kft 5 deg):
%                 .rms .rms_norm .maxabs .alf_peak_deg .alftr_deg .nl_ss
%   R.sweep       .alt / .vm grids with waf zaf wz ta xma xmd xk1 alftr_deg
%   R.tbl22       the 10-g-trim reproduction of Tables 22.1 and 22.2
%   R.bench       computed vs book, one row per published value, each with
%                 .source (table/figure/page + the zarchan.txt line)
%   R.pass        true iff every EVALUATED row passed
%   R.selfcheck   structural invariants and their measured residuals
%   R.cfg/.offbook/.nominal/.summary
%
%   Bench rows carry BOTH numeric (.book/.ours, NaN on shape rows) and text
%   (.book_str/.ours_str, always populated), the same schema
%   MGS.ZARCHAN.ADJOINTWITHAUTOPILOT uses, so BENCHSUITE's adaptAutopilot
%   logic reads these rows unchanged.
%
%   See also MGS.ZARCHAN.BENCHSUITE, MGS.ZARCHAN.ADJOINTWITHAUTOPILOT.

arguments
    % [] means "use the book value" -- see BOOKNOMINAL().  Non-empty means
    % "the caller chose this", which is what gates the benchmark rows.  Do NOT
    % replace these with literals: that puts a second copy of every book
    % constant in the file and destroys the only mechanism this port has for
    % telling an off-book run apart from a broken one.
    opts.vm      double {mustBeReal, mustBeFinite} = []   % VM     [ft/s]
    opts.alt     double {mustBeReal, mustBeFinite} = []   % ALT    [ft]
    opts.deldeg  double {mustBeReal, mustBeFinite} = []   % DEL*57.3 [deg] fin step
    opts.xncg    double {mustBeReal, mustBeFinite} = []   % XNCG   [g] only for trim 'accel'
    opts.wgt     double {mustBeReal, mustBeFinite} = []   % WGT    [lb]
    opts.a       double {mustBeReal, mustBeFinite} = []   % A      [ft/s] speed of sound
    opts.h       double {mustBeReal, mustBeFinite} = []   % H      [s]
    opts.tstop   double {mustBeReal, mustBeFinite} = []   % run length [s]
    opts.altgrid double {mustBeReal, mustBeFinite} = []   % for the Fig. 21.12-21.17 rows
    opts.vmgrid  double {mustBeReal, mustBeFinite} = []   % ditto
    % 'fin' is Listing 21.2's own trim (given a fin angle).  'accel' is
    % Listing 22.1's (given an acceleration).  NOT a mode switch on nargin.
    % Class is `char` with NO size spec on purpose: the default must be ''
    % (that is how "the caller did not supply this" is detected), and '' is
    % 0-by-0, so a (1,:) constraint would reject this function's own default.
    %
    % 'accel' changes WHERE THE AERO IS LINEARISED, not what fin step the
    % simulation applies -- deldeg still drives the run.  To reproduce
    % Listing 22.1's open-loop response as well, pass deldeg equal to the
    % resulting R.fc.trim.deltr_deg.
    opts.trim    char {mustBeMember(opts.trim, {'fin','accel',''})} = ''
    opts.benchmark (1,1) logical = true    % THE ONLY SWITCH.  Not nargin.
    opts.nonlinear (1,1) logical = true    % run the Listing 21.1 twin
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

GRAV_FT = 32.2;    % g, the book's value.  Not 9.80665, not 9.81.
DEG     = 57.3;    % the book's literal.  Not 180/pi = 57.29578.

N = bookNominal();
[cfg, offbook] = resolveConfig(opts, N);

R = struct();
R.units   = 'ft, ft/s, deg, deg/s, g = 32.2 ft/s^2 -- Zarchan''s units, NOT SI';
R.listing = 'Zarchan Listing 21.2 (zarchan.txt:6748, pp.488-491), linear airframe';
R.opts    = opts;
R.cfg     = cfg;
R.offbook = offbook;
R.nominal = N;

% ---- the nominal flight condition: geometry -> trim -> transfer functions --
R.fc = flightCondition(cfg.alt, cfg.vm, cfg, GRAV_FT, DEG);

% ---- poles, zeros and the frequency response, all by hand -----------------
R.tf = tfAnalysis(R.fc.tf);

% ---- Listing 21.2 itself ---------------------------------------------------
R.lin = simLinear(R.fc.tf, cfg.deldeg, cfg.h, cfg.tstop);

% ---- Listing 21.1, pulled in so there is something to compare against -----
if cfg.nonlinear
    R.non = simNonlinear(R.fc.geom, cfg.deldeg, cfg.vm, cfg.h, cfg.tstop, GRAV_FT, DEG);
else
    R.non = [];
end

% ---- the lag summary: the reason this file exists -------------------------
R.lag = lagSummary(R.fc.tf, R.lin, cfg);

if cfg.benchmark
    R.cmp       = compareCases(cfg, GRAV_FT, DEG);
    R.sweep     = sweeps(cfg, GRAV_FT, DEG);
    R.tbl22     = table22(cfg, GRAV_FT, DEG);
    R.selfcheck = structuralCheck(R, cfg, GRAV_FT, DEG);
    R.bench     = benchmarks(R, cfg, offbook);
    ev          = ~[R.bench.skipped];
    R.pass      = any(ev) && all([R.bench(ev).ok]);
else
    R.cmp       = [];
    R.sweep     = [];
    R.tbl22     = [];
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
    % whose parameters put every book value out of reach has checked NOTHING,
    % and that is the shape of green light this repo has been burned by.
    if ~any(ev)
        error('mgs:zarchan:airframeLinear:noBenchmark', ...
            ['not one book value could be evaluated.  The published numbers ' ...
             'live at VM = 3000 ft/s on the Fig. 21.3 missile (WGT = 1000 lb, ' ...
             'A = 1000 ft/s); this run moved %s.  Refusing to hand back an ' ...
             'unvalidated port silently -- pass ''benchmark'', false if you ' ...
             'meant to run off-book.'], offbookStr(offbook));
    end
    if ~R.pass
        bad = find(~[R.bench.ok] & ev);
        error('mgs:zarchan:airframeLinear:benchmark', ...
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
%   Straight off zarchan.txt:6748 (the first line of Listing 21.2), plus the
%   grids the Experiments section (pp.492-497) sweeps.  Change a value here
%   and you have changed what "the book" means for every benchmark row in this
%   file, which is the intended blast radius -- there is no second copy.
N = struct();
N.vm      = 3000;      % VM=3000.
N.alt     = 0;         % ALT=0.
N.deldeg  = 5;         % DEL=5./57.3   (the listing stores radians; we keep deg)
N.xncg    = 10;        % Table 22.1's 10-g command, for trim 'accel'
N.wgt     = 1000;      % WGT=1000.
N.a       = 1000;      % A=1000.  The book flags this as a 10% approximation.
N.h       = 0.0025;    % H=.0025
N.tstop   = 2;         % while T,1.99999  ->  T < TSTOP - 1e-5
N.trim    = 'fin';     % Listing 21.2 trims on the FIN ANGLE
% 30000 is deliberately IN the altitude grid: it is the exact breakpoint of
% the RHO branch whose comparison operator the OCR destroyed.
N.altgrid = [0 10000 20000 30000 40000 50000];
% Mach 2 to Mach 4 at A = 1000.  The figures' own axis limits did not survive
% extraction (see the INFO rows), so this grid is OURS and is labelled as such
% on every row that uses it -- the DIRECTION claims are the book's, the grid
% they are tested on is not.
N.vmgrid  = [2000 2500 3000 3500 4000];
end


% =============================================================================
function [cfg, offbook] = resolveConfig(opts, N)
%RESOLVECONFIG  Fill every empty option from the book, and record which ones
%   the caller moved.  OFFBOOK is what gates the benchmark rows.
cfg = struct();
offbook = {};
f = {'vm','alt','deldeg','xncg','wgt','a','h','tstop','altgrid','vmgrid','trim'};
for i = 1:numel(f)
    v = opts.(f{i});
    if isempty(v)
        cfg.(f{i}) = N.(f{i});
    else
        cfg.(f{i}) = v;
        offbook{end+1} = f{i}; %#ok<AGROW>
    end
end
cfg.benchmark = opts.benchmark;
cfg.nonlinear = opts.nonlinear;
cfg.render    = opts.render;
cfg.plot      = opts.plot;
cfg.assert    = opts.assert;

if ~isscalar(cfg.vm) || ~isscalar(cfg.alt) || ~isscalar(cfg.deldeg)
    error('mgs:zarchan:airframeLinear:scalar', ...
        ['vm, alt and deldeg name ONE flight condition and must be scalars. ' ...
         'To sweep, pass altgrid/vmgrid -- those feed the Fig. 21.12-21.17 ' ...
         'direction rows and are handled with the sweep''s own bookkeeping.']);
end
if cfg.h <= 0 || cfg.tstop <= 0
    error('mgs:zarchan:airframeLinear:positive', 'h and tstop must be positive.');
end
end


% =============================================================================
%   AERODYNAMICS -- Listing 21.2's preamble, byte for byte
% =============================================================================
function G = geometry(alt, vm, cfg, GRAV_FT)
%GEOMETRY  The Fig. 21.3 missile.  Everything down to Q is common to Listings
%   21.1, 21.2, 22.1, 22.2, 23.1, 23.2 and 23.3 -- identical in all seven.
%
%   The dimensions are HARD-CODED on purpose.  They are the missile, not a
%   parameter: change CRW and "the book's numbers" stop meaning anything, and
%   there would be no way for the benchmark gate to notice.  vm/alt/wgt/a ARE
%   options because the book itself sweeps or discusses all four.
G = struct();
G.DIAM = 1;    G.FR  = 3;     G.XL  = 20;
G.CTW  = 0;    G.CRW = 6;     G.HW  = 2;      % wing, triangular
G.CTT  = 0;    G.CRT = 2;     G.HT  = 2;      % tail, triangular
G.XN   = 4;    G.XCG = 10;    G.XHL = 19.5;
G.WGT  = cfg.wgt;   G.A = cfg.a;   G.ALT = alt;   G.VM = vm;

% Exponential atmosphere, Chapter 10.  See the header for what hangs on '<='.
if alt <= 30000
    G.RHO = .002378*exp(-alt/30000);
else
    G.RHO = .0034*exp(-alt/22000);
end

G.SWING = .5*G.HW*(G.CTW + G.CRW);
G.STAIL = .5*G.HT*(G.CTT + G.CRT);
G.SREF  = 3.1416*G.DIAM*G.DIAM/4;     % the book's 3.1416, not pi
G.XLP   = G.FR*G.DIAM;
G.SPLAN = (G.XL - G.XLP)*G.DIAM + 1.33*G.XLP*G.DIAM/2;
G.XCPN  = 2*G.XLP/3;
G.AN    = .67*G.XLP*G.DIAM;
G.AB    = (G.XL - G.XLP)*G.DIAM;
G.XCPB  = (.67*G.AN*G.XLP + G.AB*(G.XLP + .5*(G.XL - G.XLP)))/(G.AN + G.AB);
G.XCPW  = G.XLP + G.XN + .7*G.CRW - .2*G.CTW;
G.XMACH = vm/G.A;
G.XIYY  = G.WGT*(3*((G.DIAM/2)^2) + G.XL*G.XL)/(12*GRAV_FT);
G.TMP1  = (G.XCG - G.XCPW)/G.DIAM;
G.TMP2  = (G.XCG - G.XHL)/G.DIAM;
G.TMP3  = (G.XCG - G.XCPB)/G.DIAM;
G.TMP4  = (G.XCG - G.XCPN)/G.DIAM;

% GUARD 1.  beta = sqrt(M^2-1) is the Ackeret/supersonic-fin factor; the whole
% 8*S/(beta*Sref) family of terms is meaningless subsonic and IMAGINARY below
% Mach 1.  MATLAB would carry the complex value silently through Q, CNA, XMA,
% WAF and the integration and hand back a finite real(nL).
if G.XMACH <= 1
    error('mgs:zarchan:airframeLinear:subsonic', ...
        ['Mach %.4g (VM = %g ft/s, A = %g ft/s): beta = sqrt(M^2 - 1) is not ' ...
         'real.  The linearised fin/wing terms in this listing are a ' ...
         'SUPERSONIC approximation and there is no subsonic answer to give. ' ...
         'The book runs Mach 3 (3000/1000).  MATLAB will happily return ' ...
         'complex accelerations here, which is why this is an error and not ' ...
         'a warning.'], G.XMACH, vm, G.A);
end
G.B = sqrt(G.XMACH^2 - 1);
G.Q = .5*G.RHO*vm*vm;
end


% =============================================================================
function T = trimFin(G, delrad)
%TRIMFIN  Listing 21.2's own trim: CM = y1*a + y2*a^2 + y3*del = 0 solved for
%   the trim angle of attack (book pp.487-488, zarchan.txt:6748).
%
%   NOTE Q DOES NOT APPEAR.  CM is a coefficient; the trim condition is CM = 0,
%   so dynamic pressure divides straight out and the trim angle depends on
%   MACH and geometry only -- not on altitude.  This is not a simplification
%   made here, it is what the equation says, and it is the thing that makes
%   Fig. 21.6's caption easy to misread.  See the header.
T = struct();
T.mode = 'fin';
T.Y1 = 2*G.TMP4 + 8*G.SWING*G.TMP1/(G.B*G.SREF) + 8*G.STAIL*G.TMP2/(G.B*G.SREF);
T.Y2 = 1.5*G.SPLAN*G.TMP3/G.SREF;
T.Y3 = 8*G.STAIL*G.TMP2*delrad/(G.B*G.SREF);

disc = T.Y1*T.Y1 - 4*T.Y2*T.Y3;
% GUARD 3.
if disc < 0
    error('mgs:zarchan:airframeLinear:noTrim', ...
        ['the trim quadratic has no real root (discriminant %.4g) at this ' ...
         'fin deflection: there is no angle of attack at which the moment ' ...
         'balances, so "the trim angle of attack" does not exist and every ' ...
         'aero derivative below would be complex.'], disc);
end
T.alftr      = (-T.Y1 - sqrt(disc))/(2*T.Y2);   % the listing's root
T.alftr_other = (-T.Y1 + sqrt(disc))/(2*T.Y2);  % "the unrealistic root", p.487
T.delrad     = delrad;
end


function T = trimAccel(G, xncg)
%TRIMACCEL  PULLED IN FROM LISTING 22.1 (zarchan.txt:6958), verbatim.
%   Trim on a commanded ACCELERATION instead of a fin angle.  This is not part
%   of Listing 21.2; it is here only because Tables 22.1/22.2 -- the only hard
%   numbers published for wAF, zAF, wz, K1, Ta, K3 on this airframe -- are
%   quoted at a 10-g trim.  See the header.
T = struct();
T.mode = 'accel';
T.P1 = G.WGT*xncg/(G.Q*G.SREF);
T.Y1 = 2 + 8*G.SWING/(G.B*G.SREF) + 8*G.STAIL/(G.B*G.SREF);
T.Y2 = 1.5*G.SPLAN/G.SREF;
T.Y3 = 8*G.STAIL/(G.B*G.SREF);
T.Y4 = 2*G.TMP4 + 8*G.SWING*G.TMP1/(G.B*G.SREF) + 8*G.STAIL*G.TMP2/(G.B*G.SREF);
T.Y5 = 1.5*G.SPLAN*G.TMP3/G.SREF;
T.Y6 = 8*G.STAIL*G.TMP2/(G.B*G.SREF);
T.P2 = T.Y2 - T.Y3*T.Y5/T.Y6;
T.P3 = T.Y1 - T.Y3*T.Y4/T.Y6;

disc = T.P3*T.P3 + 4*T.P2*T.P1;
if disc < 0
    error('mgs:zarchan:airframeLinear:noTrimAccel', ...
        ['no real trim at %g g: the airframe cannot produce that much ' ...
         'acceleration at this flight condition (discriminant %.4g).'], xncg, disc);
end
T.alftr       = (-T.P3 + sqrt(disc))/(2*T.P2);
T.alftr_other = (-T.P3 - sqrt(disc))/(2*T.P2);
T.delrad      = -T.Y4*T.alftr/T.Y6 - T.Y5*T.alftr*T.alftr/T.Y6;
T.xncg        = xncg;
end


% =============================================================================
function A = aeroDerivatives(G, alftr, vm, GRAV_FT)
%AERODERIVATIVES  The linearisation, Listing 21.2 lines 2-3 of the preamble.
%   CNA and CMA depend on the TRIM angle of attack -- that is the whole
%   content of "linearise about trim" and the reason the alpha^2 term is what
%   limits the model (zarchan.txt:6770).
A = struct();
A.CNA  = 2 + 1.5*G.SPLAN*alftr/G.SREF + 8*G.SWING/(G.B*G.SREF) + 8*G.STAIL/(G.B*G.SREF);
A.CND  = 8*G.STAIL/(G.B*G.SREF);
A.CMAP = 2*G.TMP4 + 1.5*G.SPLAN*alftr*G.TMP3/G.SREF + 8*G.SWING*G.TMP1/(G.B*G.SREF);
A.CMA  = A.CMAP + 8*G.STAIL*G.TMP2/(G.B*G.SREF);
A.CMD  = 8*G.STAIL*G.TMP2/(G.B*G.SREF);
A.XMA  = G.Q*G.SREF*G.DIAM*A.CMA/G.XIYY;
A.XMD  = G.Q*G.SREF*G.DIAM*A.CMD/G.XIYY;
A.ZA   = -GRAV_FT*G.Q*G.SREF*A.CNA/(G.WGT*vm);
A.ZD   = -GRAV_FT*G.Q*G.SREF*A.CND/(G.WGT*vm);
end


function F = transferFunctions(A, vm)
%TRANSFERFUNCTIONS  p.485-486.  1845 is the book's rounding of 57.3*32.2.
F = struct();

% GUARD 2.  See the header.  This is the guard that matters most: an unstable
% airframe is a perfectly ordinary design (Zarchan discusses it two pages
% later) and the ONLY thing that goes wrong is that this listing's algebra
% stops describing it -- silently, in complex arithmetic.
if A.XMA >= 0
    error('mgs:zarchan:airframeLinear:unstableAirframe', ...
        ['Ma = %+.4g >= 0: this airframe is statically UNSTABLE (Zarchan, ' ...
         'p.493: "A positive Ma indicates that the bare airframe is ' ...
         'unstable").  wAF = sqrt(-Ma) is then imaginary and the second-order ' ...
         'form of Listing 21.2 does not describe the vehicle -- an unstable ' ...
         'airframe has a REAL right-half-plane pole and needs the pole/zero ' ...
         'form, not a natural frequency and a damping ratio.  MATLAB would ' ...
         'integrate the complex equations without complaint.'], A.XMA);
end
if A.ZD == 0
    error('mgs:zarchan:airframeLinear:noControl', ...
        'Zd = 0: the fin produces no normal force, so wz and K1 are undefined.');
end

F.WZ  = sqrt((A.XMA*A.ZD - A.ZA*A.XMD)/A.ZD);
F.WAF = sqrt(-A.XMA);
F.ZAF = .5*F.WAF*A.ZA/A.XMA;
F.XK1 = -vm*(A.XMA*A.ZD - A.XMD*A.ZA)/(1845*A.XMA);
F.XK2 = F.XK1;      % in the listing, never used.  Kept so a diff against
                    % p.489 does not send the reader hunting for it.
F.TA  = A.XMD/(A.XMA*A.ZD - A.XMD*A.ZA);
F.XK3 = 1845*F.XK1/vm;
F.vm  = vm;
end


function FC = flightCondition(alt, vm, cfg, GRAV_FT, DEG)
%FLIGHTCONDITION  geometry -> trim -> derivatives -> transfer functions.
FC = struct();
FC.alt = alt;  FC.vm = vm;
FC.geom = geometry(alt, vm, cfg, GRAV_FT);
FC.mach = FC.geom.XMACH;
FC.rho  = FC.geom.RHO;
FC.q    = FC.geom.Q;

switch cfg.trim
case 'fin'
    T = trimFin(FC.geom, cfg.deldeg/DEG);
case 'accel'
    T = trimAccel(FC.geom, cfg.xncg);
otherwise
    error('mgs:zarchan:airframeLinear:trimMode', ...
        'trim must be ''fin'' (Listing 21.2) or ''accel'' (Listing 22.1).');
end

FC.trim = struct();
FC.trim.mode              = T.mode;
FC.trim.alftr             = T.alftr;
FC.trim.alftr_deg         = T.alftr*DEG;
FC.trim.deltr             = T.delrad;
FC.trim.deltr_deg         = T.delrad*DEG;
FC.trim.root_discarded_deg = T.alftr_other*DEG;
% CM at the trim point must be zero.  Recomputed from the NONLINEAR moment
% coefficient, not from the quadratic that produced alftr, so it is a real
% check and not a restatement.
FC.trim.cm_residual = cmNonlinear(FC.geom, T.alftr, T.delrad);

FC.aero = aeroDerivatives(FC.geom, T.alftr, vm, GRAV_FT);
FC.tf   = transferFunctions(FC.aero, vm);
FC.tf.deldeg = cfg.deldeg;
end


function CM = cmNonlinear(G, alf, del)
%CMNONLINEAR  Listing 21.1's moment coefficient, used only to verify trim.
CM = 2*alf*G.TMP4 + 1.5*G.SPLAN*alf*alf*G.TMP3/G.SREF + ...
     8*G.SWING*alf*G.TMP1/(G.B*G.SREF) + 8*G.STAIL*(alf + del)*G.TMP2/(G.B*G.SREF);
end


function CN = cnNonlinear(G, alf, del)
%CNNONLINEAR  Listing 21.1's normal force coefficient.
CN = 2*alf + 1.5*G.SPLAN*alf*alf/G.SREF + 8*G.SWING*alf/(G.B*G.SREF) + ...
     8*G.STAIL*(alf + del)/(G.B*G.SREF);
end


% =============================================================================
%   POLES, ZEROS AND FREQUENCY RESPONSE -- base MATLAB, no toolbox
% =============================================================================
function P = tfAnalysis(F)
%TFANALYSIS  What tf/damp/zero/bode would have told us, taken by hand.
%
%   The book writes the denominator in the "1 + ... " form.  ROOTS normalises
%   by the leading coefficient, so the monic form has identical roots -- both
%   are built here and the self-check requires them to agree, which is the
%   cheapest available proof that the coefficient vector is in DESCENDING
%   powers of s and not reversed.
P = struct();
P.den_book  = [1/F.WAF^2, 2*F.ZAF/F.WAF, 1];       % 1 + 2z s/w + s^2/w^2
P.den_monic = [1, 2*F.ZAF*F.WAF, F.WAF^2];         % s^2 + 2zw s + w^2
P.poles       = roots(P.den_book);
P.poles_monic = roots(P.den_monic);

% damp(), by hand.  For a complex pair wn = |p| and zeta = -Re(p)/|p|.
P.wn   = abs(P.poles(1));
P.zeta = -real(P.poles(1))/P.wn;
P.wd   = abs(imag(P.poles(1)));                    % damped frequency
P.stable = all(real(P.poles) < 0);

% nL/del numerator: K1*(1 - s^2/wz^2).  Roots at +/-wz; the one at +wz is in
% the RIGHT half plane.  Zarchan, Ch.23 p.549 (zarchan.txt:7687): "the
% negative sign in the numerator ... indicates that the airframe zero is in
% the right-half plane".
P.num_nl  = [-F.XK1/F.WZ^2, 0, F.XK1];
P.zeros_nl = roots(P.num_nl);
P.rhp_zero = any(real(P.zeros_nl) > 0);
P.rhp_zero_rad_s = max(real(P.zeros_nl));

% thd/del numerator: K3*(1 + Ta*s).  One zero at -1/Ta, in the LEFT half
% plane -- the body rate channel does NOT share the acceleration channel's
% non-minimum-phase behaviour, which is why rate feedback is the cheap fix.
P.num_thd  = [F.XK3*F.TA, F.XK3];
P.zero_thd = roots(P.num_thd);
if isempty(P.zero_thd)
    % Ta == 0 degrades the numerator to a constant, so there is no zero.  NaN
    % rather than an empty, so the printout says "NaN" instead of silently
    % printing the line with a field missing.
    P.zero_thd = NaN;
end

% freqresp(), by hand.  polyval accepts a complex argument.
P.w = logspace(log10(F.WAF/100), log10(F.WAF*100), 4001).';
gj  = polyval(P.num_nl, 1i*P.w) ./ polyval(P.den_book, 1i*P.w);
P.mag_nl       = abs(gj);
P.phase_nl_deg = unwrap(angle(gj))*180/pi;
P.dc_gain      = polyval(P.num_nl, 0)/polyval(P.den_book, 0);   % == K1

[P.peak_mag, ip] = max(P.mag_nl);
P.peak_w = P.w(ip);
% Closed form for the resonant peak of this particular numerator/denominator
% pair.  At w = wAF the denominator is exactly 2*zAF*j, and the numerator
% 1 - (j w)^2/wz^2 = 1 + (wAF/wz)^2 -- the RHP zero ADDS magnitude here rather
% than subtracting it, which is easy to get backwards.
P.peak_mag_form = abs(F.XK1)*(1 + (F.WAF/F.WZ)^2)/(2*F.ZAF);
end


% =============================================================================
%   THE INTEGRATIONS
% =============================================================================
function S = simLinear(F, deldeg, h, tstop)
%SIMLINEAR  LISTING 21.2, line for line (zarchan.txt:6749-6769).
%
%   Second-order Heun: predictor pass, then trapezoidal corrector, with the
%   book's FLAG/STEP construction preserved exactly rather than rewritten into
%   a for-loop -- a reader diffing against p.490 must find the same lines.
%
%   eps ("E") is the intermediate state of the split transfer function:
%       E/del  = 1/(1 + 2 zAF s/wAF + s^2/wAF^2)   -> Edd = wAF^2(del - E - 2 zAF Ed/wAF)
%       nL/E   = K1 (1 - s^2/wz^2)                 -> nL  = K1 (E - Edd/wz^2)
%       thd/E  = K3 (1 + Ta s)                     -> thd = K3 (E + Ta Ed)
%   E is in DEGREES because del enters as DEL*57.3.
checkStep(F, h);

WAF = F.WAF;  ZAF = F.ZAF;  WZ = F.WZ;
XK1 = F.XK1;  TA  = F.TA;   XK3 = F.XK3;
DELDEG = deldeg;               % the listing's DEL*57.3

E = 0; ED = 0; T = 0; S_ACC = 0;
EDD = 0;                        % set on every inner pass before use; declared
                                % here only so the corrector below can never
                                % read an undefined variable if the loop body
                                % is ever edited.
XNL = 0; THD = 0;

nmax = ceil(tstop/0.01) + 2;    % the book grows ArrayT element by element,
Tarr = zeros(nmax,1);           % which is O(n^2) and makes MATLAB warn.  Same
XNLarr = zeros(nmax,1);         % numbers, preallocated.
THDarr = zeros(nmax,1);
n = 0;

while T < (tstop - 1e-5)        % the book's literal 1.99999 for tstop = 2
    EOLD = E; EDOLD = ED; STEP = 1; FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            E  = E  + h*ED;
            ED = ED + h*EDD;
            T  = T  + h;
        end
        EDD = WAF*WAF*(DELDEG - E - 2*ZAF*ED/WAF);
        XNL = XK1*(E - EDD/WZ^2);
        THD = XK3*(E + TA*ED);
        FLAG = 1;
    end
    FLAG = 0; %#ok<NASGU>   kept: the listing resets it here
    E  = .5*(EOLD  + E  + h*ED);
    ED = .5*(EDOLD + ED + h*EDD);
    S_ACC = S_ACC + h;
    if S_ACC >= .0099999        % the book's 0.01 - 1e-7; the slop is there
        S_ACC = 0;              % because 4*0.0025 is 0.0099999999999999985
        n = n + 1;
        Tarr(n) = T; XNLarr(n) = XNL; THDarr(n) = THD;
    end
end

S = struct('t', Tarr(1:n), 'xnl', XNLarr(1:n), 'thd', THDarr(1:n), ...
           'h', h, 'tstop', tstop, 'deldeg', deldeg, ...
           'nl_dc', XK1*deldeg, 'thd_dc', XK3*deldeg);
% NOTE the book records XNL and THD from the PREDICTED point, before the
% corrector runs -- that is what the listing does and it is kept.  The
% difference is O(h^2) and the steady state is unaffected.
end


function S = simNonlinear(G, deldeg, vm, h, tstop, GRAV_FT, DEG)
%SIMNONLINEAR  LISTING 21.1 (zarchan.txt:6510-6526), verbatim.  PULLED IN.
%
%   Not part of Listing 21.2.  Present because every published claim about
%   21.2 is a claim about how it compares to 21.1, and a port scored against
%   itself is not scored at all.  Same H, same TSTOP, same 0.01-s recorder, so
%   the two traces are index-for-index comparable with no interpolation.
DEL = deldeg/DEG;
THD = 0; ALF = 0; T = 0; S_ACC = 0;
THDD = 0; ALFD = 0; XNL = 0;

nmax = ceil(tstop/0.01) + 2;
Tarr = zeros(nmax,1); Garr = zeros(nmax,1); Aarr = zeros(nmax,1);
n = 0;

while T < (tstop - 1e-5)
    THDOLD = THD; ALFOLD = ALF; STEP = 1; FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            THD = THD + h*THDD;
            ALF = ALF + h*ALFD;
            T   = T   + h;
        end
        CN   = cnNonlinear(G, ALF, DEL);
        CM   = cmNonlinear(G, ALF, DEL);
        THDD = G.Q*G.SREF*G.DIAM*CM/G.XIYY;
        XNL  = GRAV_FT*G.Q*G.SREF*CN/G.WGT;     % ft/s^2 here, /32.2 on record
        ALFD = THD - XNL/vm;
        FLAG = 1;
    end
    FLAG = 0; %#ok<NASGU>
    THD = .5*(THDOLD + THD + h*THDD);
    ALF = .5*(ALFOLD + ALF + h*ALFD);
    S_ACC = S_ACC + h;
    if S_ACC >= .0099999
        S_ACC = 0;
        n = n + 1;
        Tarr(n) = T; Garr(n) = XNL/GRAV_FT; Aarr(n) = ALF*DEG;
    end
end

S = struct('t', Tarr(1:n), 'xnl_g', Garr(1:n), 'alf_deg', Aarr(1:n), ...
           'h', h, 'tstop', tstop, 'deldeg', deldeg);
end


function checkStep(F, h)
%CHECKSTEP  GUARD 4.  See the header for the derivation and the measured
%   numbers.  Heun ANTI-DAMPS an oscillatory mode; the airframe's own damping
%   is only 0.02-0.06, so there is very little to eat into.
hmax = (0.8*F.ZAF)^(1/3)/F.WAF;
if h > hmax
    error('mgs:zarchan:airframeLinear:stepSize', ...
        ['h = %.5g s is too coarse for this airframe (wAF = %.4g rad/s, ' ...
         'zAF = %.4g).  Heun''s amplification on a purely oscillatory mode ' ...
         'is sqrt(1 + (w h)^4/4) > 1: it INJECTS damping of the wrong sign ' ...
         'at about w^4 h^3/8 per second, against a physical decay of only ' ...
         'zAF*wAF = %.4g per second.  Keeping the numerical term under a ' ...
         'tenth of the physical one requires h <= (0.8 zAF)^(1/3)/wAF = ' ...
         '%.5g s.  The book runs h = 0.0025.  Measured outside MATLAB on a ' ...
         'bit-identical loop, the sea-level first overshoot drifts -28.47 g ' ...
         '(h=0.0025) -> -29.01 (0.01) -> -30.95 (0.02) -> -34.67 (0.03) with ' ...
         'no other symptom, so this is enforced rather than warned about.'], ...
        h, F.WAF, F.ZAF, F.ZAF*F.WAF, hmax);
end
end


% =============================================================================
%   THE LAG -- derived from the pole, and measured off the run
% =============================================================================
function L = lagSummary(F, lin, cfg)
%LAGSUMMARY  The reason this file exists.  Two independent routes to the same
%   time constant: the pole's real part, and the decay of the simulated
%   ringing.  If they disagree, one of them is not what it claims to be.
L = struct();
L.tau_wn_s  = 1/F.WAF;              % the book's definition, zarchan.txt:6784
L.tau_env_s = 1/(F.ZAF*F.WAF);      % -1/Re(pole): how long it actually rings
L.ratio     = L.tau_env_s/L.tau_wn_s;   % == 1/zAF
L.period_s  = 2*pi/(F.WAF*sqrt(1 - F.ZAF^2));
L.cycles_in_run = cfg.tstop/L.period_s;

[L.tau_env_meas_s, L.n_peaks, L.period_meas_s] = ...
    envelopeFit(lin.t, lin.xnl, lin.nl_dc);

L.note = ['1/wAF is how fast the airframe REACHES its acceleration; ' ...
          '1/(zAF*wAF) is how long it is still ringing about it.  They ' ...
          'differ by 1/zAF, which is 20x at sea level.'];
end


function [tau, npk, period] = envelopeFit(t, x, ss)
%ENVELOPEFIT  Fit |x - ss| ~ A*exp(-t/tau) through the local extrema.
%   polyfit/polyval are base MATLAB.  Returns NaN rather than a number if
%   there are too few peaks to fit -- a two-point "fit" through a transient
%   that has not rung yet is exactly the plausible wrong answer to refuse.
d = x - ss;
ip = find(abs(d(2:end-1)) >= abs(d(1:end-2)) & ...
          abs(d(2:end-1)) >  abs(d(3:end))) + 1;
ip = ip(abs(d(ip)) > 1e-9*max(1, abs(ss)));
npk = numel(ip);
if npk < 4
    tau = NaN; period = NaN; return
end
c = polyfit(t(ip), log(abs(d(ip))), 1);
tau = -1/c(1);
period = 2*mean(diff(t(ip)));   % successive extrema are half a period apart
end


% =============================================================================
%   THE THREE PUBLISHED CASES, LINEAR vs NONLINEAR
% =============================================================================
function C = compareCases(cfg, GRAV_FT, DEG)
%COMPARECASES  Figs. 21.9 (SL, 5 deg), 21.10 (SL, 10 deg), 21.11 (50 kft,
%   5 deg).  These are run at the BOOK's conditions regardless of cfg.alt and
%   cfg.deldeg, because these three points ARE the published comparison -- but
%   they still inherit cfg.vm/wgt/a, which is what the benchmark gate keys on.
cases = struct('name', {'Fig.21.9  SL   5 deg', ...
                        'Fig.21.10 SL  10 deg', ...
                        'Fig.21.11 50k  5 deg'}, ...
               'alt',  {0, 0, 50000}, ...
               'deldeg', {5, 10, 5});
C = struct('name',{},'alt',{},'deldeg',{},'rms',{},'rms_norm',{},'maxabs',{}, ...
           'alf_peak_deg',{},'alf_mean_deg',{},'alftr_deg',{},'nl_ss',{}, ...
           'nl_mean_lin',{},'nl_mean_non',{},'waf',{},'zaf',{},'ta',{},'ok',{});

sub = cfg; sub.trim = 'fin';
for k = 1:numel(cases)
    sub.alt = cases(k).alt; sub.deldeg = cases(k).deldeg;
    FC  = flightCondition(sub.alt, sub.vm, sub, GRAV_FT, DEG);
    lin = simLinear(FC.tf, sub.deldeg, sub.h, sub.tstop);
    i = numel(C) + 1;
    C(i).name   = cases(k).name;
    C(i).alt    = cases(k).alt;
    C(i).deldeg = cases(k).deldeg;
    C(i).nl_ss  = FC.tf.XK1*cases(k).deldeg;      % linear DC gain * step
    C(i).alftr_deg   = FC.trim.alftr_deg;
    C(i).nl_mean_lin = mean(lin.xnl);
    C(i).waf = FC.tf.WAF;  C(i).zaf = FC.tf.ZAF;  C(i).ta = FC.tf.TA;
    C(i).ok  = cfg.nonlinear;

    if ~cfg.nonlinear
        % Every remaining field NEEDS the Listing 21.1 twin.  NaN, not zero
        % and not a linear-only stand-in: a zero rms would print as a perfect
        % match, which is the exact wrong answer to hand back.
        [C(i).rms, C(i).rms_norm, C(i).maxabs, C(i).alf_peak_deg, ...
         C(i).alf_mean_deg, C(i).nl_mean_non] = deal(NaN);
        continue
    end

    non = simNonlinear(FC.geom, sub.deldeg, sub.vm, sub.h, sub.tstop, GRAV_FT, DEG);
    m = min(numel(lin.xnl), numel(non.xnl_g));
    e = lin.xnl(1:m) - non.xnl_g(1:m);
    C(i).rms = sqrt(mean(e.^2));
    % NORMALISED because the three cases differ by 7x in signal size: at
    % 50 kft the raw rms is 0.22 g against sea level's 0.86, which would make
    % the high-altitude case look FOUR TIMES BETTER when it is in fact worse.
    % The book's claims are about model fidelity, not about g.
    C(i).rms_norm = C(i).rms/abs(C(i).nl_ss);
    C(i).maxabs   = max(abs(e));
    C(i).alf_peak_deg = max(abs(non.alf_deg));
    C(i).alf_mean_deg = mean(non.alf_deg);
    C(i).nl_mean_non  = mean(non.xnl_g);
end
end


% =============================================================================
%   THE EXPERIMENTS SWEEPS -- Figs. 21.12 to 21.17
% =============================================================================
function S = sweeps(cfg, GRAV_FT, DEG)
%SWEEPS  Direction rows only.  The FIGURES' axis limits did not survive the
%   extraction, so the grid is ours (see BOOKNOMINAL) and every row that uses
%   it says so.  What is the book's is the DIRECTION of each trend, which is
%   in prose and greppable.
%
%   The sweeps are run at the fin trim (5 deg), i.e. Chapter 21's own
%   operating point, NOT at Chapter 22's 10-g trim.  Evidence that this is the
%   right reading: the book quotes Ta ~ 4 s at 50 kft/3000 ft/s from Fig.
%   21.16, and the 5-deg-fin trim gives 3.78 s (-5%) while the 10-g trim gives
%   2.40 s (-40%).
S = struct();
sub = cfg; sub.trim = 'fin'; sub.deldeg = 5;

S.alt = cfg.altgrid(:);
S.vm_for_alt = cfg.vm;
[S.waf_alt, S.zaf_alt, S.wz_alt, S.ta_alt, S.xma_alt, S.xmd_alt, ...
 S.xk1_alt, S.alftr_alt] = deal(nan(numel(S.alt),1));
for i = 1:numel(S.alt)
    FC = flightCondition(S.alt(i), cfg.vm, sub, GRAV_FT, DEG);
    S.waf_alt(i)=FC.tf.WAF; S.zaf_alt(i)=FC.tf.ZAF; S.wz_alt(i)=FC.tf.WZ;
    S.ta_alt(i)=FC.tf.TA;   S.xma_alt(i)=FC.aero.XMA; S.xmd_alt(i)=FC.aero.XMD;
    S.xk1_alt(i)=FC.tf.XK1; S.alftr_alt(i)=FC.trim.alftr_deg;
end

S.vm = cfg.vmgrid(:);
S.alt_for_vm = [0; 50000];
nv = numel(S.vm);
[S.waf_vm, S.zaf_vm, S.wz_vm, S.ta_vm, S.xma_vm, S.xmd_vm] = ...
    deal(nan(nv, numel(S.alt_for_vm)));
for j = 1:numel(S.alt_for_vm)
    for i = 1:nv
        FC = flightCondition(S.alt_for_vm(j), S.vm(i), sub, GRAV_FT, DEG);
        S.waf_vm(i,j)=FC.tf.WAF; S.zaf_vm(i,j)=FC.tf.ZAF; S.wz_vm(i,j)=FC.tf.WZ;
        S.ta_vm(i,j)=FC.tf.TA;   S.xma_vm(i,j)=FC.aero.XMA; S.xmd_vm(i,j)=FC.aero.XMD;
    end
end
end


function T = table22(cfg, GRAV_FT, DEG)
%TABLE22  Tables 22.1 and 22.2 (zarchan.txt:6932-6937, 6976-6977) reproduced.
%
%   Fixed at the BOOK's own conditions -- 3000 ft/s, 10 g, sea level and
%   50 kft -- irrespective of cfg.alt and cfg.xncg, because those conditions
%   ARE what the tables are.  It still inherits cfg.vm/wgt/a, so a caller who
%   moves the missile gets these rows SKIPPED rather than quietly rescored.
T = struct();
T.alt  = [0; 50000];
T.vm   = cfg.vm;
T.xncg = 10;
sub = cfg; sub.trim = 'accel'; sub.xncg = 10;
n = numel(T.alt);
[T.waf, T.zaf, T.wz, T.xk1, T.xk3, T.ta, T.alftr_deg, T.deltr_deg] = deal(nan(n,1));
for i = 1:n
    FC = flightCondition(T.alt(i), cfg.vm, sub, GRAV_FT, DEG);
    T.waf(i)=FC.tf.WAF; T.zaf(i)=FC.tf.ZAF; T.wz(i)=FC.tf.WZ;
    T.xk1(i)=FC.tf.XK1; T.xk3(i)=FC.tf.XK3; T.ta(i)=FC.tf.TA;
    T.alftr_deg(i)=FC.trim.alftr_deg; T.deltr_deg(i)=FC.trim.deltr_deg;
    T.cm_residual(i,1) = FC.trim.cm_residual;
    % nL delivered at the trim point, recomputed from the NONLINEAR CN.  Must
    % come back as exactly XNCG or the 22.1 quadratic was mis-transcribed.
    CN = cnNonlinear(FC.geom, FC.trim.alftr, FC.trim.deltr);
    T.nl_at_trim(i,1) = FC.geom.Q*FC.geom.SREF*CN/FC.geom.WGT;
    % Round trip: feed the accel-trim FIN ANGLE back into Listing 21.2's own
    % fin-trim quadratic.  It must return the same alpha.  This is the single
    % strongest check that the two trims are the same equation -- they share
    % Y4 == 21.2's Y1 and Y5 == 21.2's Y2 -- and it exercises 21.2's root
    % selection at a NEGATIVE fin angle, which the book never runs.
    Tf = trimFin(FC.geom, FC.trim.deltr);
    T.roundtrip_rel(i,1) = abs(Tf.alftr/FC.trim.alftr - 1);
end
end


% =============================================================================
%   STRUCTURAL SELF-CHECK -- wiring, not the book.  A failure here voids
%   everything below it, so it is printed above the board.
% =============================================================================
function K = structuralCheck(R, cfg, GRAV_FT, DEG)
K = struct(); K.skipped = false;
F = R.fc.tf; P = R.tf;

% 1. The polynomial was assembled in descending powers.  Reversed, the roots
%    would be the RECIPROCALS -- 0.0025 +/- 0.04i -- and a reciprocal pole
%    pair still prints like a lightly damped second-order system.
K.poly_resid = max(abs(sort(P.poles) - sort(P.poles_monic)));
K.poly_tol   = 1e-9*max(1, abs(F.WAF));
K.poly_ok    = K.poly_resid <= K.poly_tol;

% 2. roots() must give back the listing's own wAF and zAF.  An identity,
%    checked rather than assumed.
K.damp_resid = max(abs([P.wn/F.WAF - 1, P.zeta/F.ZAF - 1]));
K.damp_tol   = 1e-12;
K.damp_ok    = K.damp_resid <= K.damp_tol;

% 3. DC gain of nL/del is K1 by construction.
K.dc_resid = abs(P.dc_gain/F.XK1 - 1);
K.dc_tol   = 1e-12;
K.dc_ok    = K.dc_resid <= K.dc_tol;

% 4. THE ONE THAT MATTERS: the ringing measured off the time-domain
%    integration against the pole's real part from roots().  Two entirely
%    different computations of the same lag.
K.lag_form = R.lag.tau_env_s;
K.lag_meas = R.lag.tau_env_meas_s;
K.lag_tol  = 0.01;
% "Could not be measured" is n/a, not FAIL.  A run too short to ring cannot
% disprove the pole, and printing FAIL for it would train a reader to ignore
% this section.
K.lag_applies = ~isnan(K.lag_meas);
if ~K.lag_applies
    K.lag_rel = NaN; K.lag_ok = false;
    K.lag_why = sprintf(['only %d extrema in a %.3g s run (%.2f cycles at ' ...
        'wAF = %.3g rad/s) -- too few to fit an envelope'], ...
        R.lag.n_peaks, cfg.tstop, R.lag.cycles_in_run, F.WAF);
else
    K.lag_rel = abs(K.lag_meas/K.lag_form - 1);
    K.lag_ok  = K.lag_rel <= K.lag_tol;
    K.lag_why = '';
end

% 5. Measured oscillation period vs 2*pi/wd from the pole's imaginary part.
K.per_form = R.lag.period_s;
K.per_meas = R.lag.period_meas_s;
K.per_tol  = 0.02;          % the recorder samples at 0.01 s; at wAF = 25 that
                            % is 4% of a period, so this cannot be tighter
K.per_applies = ~isnan(K.per_meas);
if ~K.per_applies
    K.per_rel = NaN; K.per_ok = false;
else
    K.per_rel = abs(K.per_meas/K.per_form - 1);
    K.per_ok  = K.per_rel <= K.per_tol;
end

% 6. Resonant peak against the closed form.  Catches a sign error in the
%    numerator: if the zero were read as (1 + s^2/wz^2) the peak would come
%    out at |K1|(1 - (wAF/wz)^2)/(2 zAF), which at sea level is 15.2 instead
%    of 36.1 -- both perfectly plausible-looking resonances.
K.peak_resid = abs(P.peak_mag/P.peak_mag_form - 1);
K.peak_tol   = 0.01;
K.peak_ok    = K.peak_resid <= K.peak_tol;
K.peakw_resid = abs(P.peak_w/F.WAF - 1);
K.peakw_tol   = 0.02;
K.peakw_ok    = K.peakw_resid <= K.peakw_tol;

% 7. CM = 0 at the trim point, recomputed from the NONLINEAR moment.
K.trim_resid = abs(R.fc.trim.cm_residual);
K.trim_tol   = 1e-10;
K.trim_ok    = K.trim_resid <= K.trim_tol;

% 8. The discarded root really is unrealistic (p.487).
K.otherroot_deg = R.fc.trim.root_discarded_deg;
K.otherroot_ok  = abs(K.otherroot_deg) > 90;

% 9. K3 == 1845*K1/VM exactly (the listing's own definition), AND the physical
%    identity thd_ss == 57.3*32.2*nL_ss/VM only to 3.3e-5 -- because the book's
%    1845 is 57.3*32.2 = 1845.06 ROUNDED.  Two different tolerances on purpose:
%    the first is a transcription check, the second measures the book's own
%    rounding and must NOT be tightened to 1e-12 or a correct port fails.
K.k3_resid = abs(F.XK3/(1845*F.XK1/cfg.vm) - 1);
K.k3_tol   = 1e-12;
K.k3_ok    = K.k3_resid <= K.k3_tol;
thd_phys   = DEG*GRAV_FT*(F.XK1*cfg.deldeg)/cfg.vm;
K.rate_resid = abs((F.XK3*cfg.deldeg)/thd_phys - 1);
K.rate_tol   = 1e-4;
K.rate_ok    = K.rate_resid <= K.rate_tol;

% 10. The linear steady state, taken off the RUN, against K1*del (final value
%     theorem).  Uses the last half of the record so the transient is out.
i0 = max(1, floor(numel(R.lin.xnl)/2));
K.ss_meas  = mean(R.lin.xnl(i0:end));
K.ss_resid = abs(K.ss_meas/R.lin.nl_dc - 1);
K.ss_tol   = 0.01;
K.ss_ok    = K.ss_resid <= K.ss_tol;
% Only meaningful once the ringing has actually decayed.  Two envelope time
% constants leaves 13.5% of the initial amplitude, which averaging over the
% last half of the window removes; the book's sea-level run clears this at
% 2 s vs 1.58 s and the residual is 6.3e-4.  At 50 kft the envelope constant
% is 5.4 s and the same 2-s run is nowhere near settled -- the window mean is
% 5.5% off K1*del, which is the TRANSIENT, not a broken final value theorem.
% Scoring it there would put a red light on a correct port.
K.ss_applies = cfg.tstop >= 2*R.lag.tau_env_s;

% 11. Fin trim is altitude-invariant at fixed Mach.  Exactly, not nearly --
%     Q divides out of CM = 0.  This is the fact behind the Fig. 21.6 row.
sub = cfg; sub.trim = 'fin';
FC0  = flightCondition(0,     cfg.vm, sub, GRAV_FT, DEG);
FC50 = flightCondition(50000, cfg.vm, sub, GRAV_FT, DEG);
t0   = FC0.trim.alftr_deg;
t50  = FC50.trim.alftr_deg;
K.trimalt_resid = abs(t50 - t0);
K.trimalt_tol   = 0;             % ZERO.  Not a tolerance -- an identity.
K.trimalt_ok    = K.trimalt_resid <= K.trimalt_tol;
K.trimalt_deg   = t0;

% 12. The Ch.22 trim round-trips through Ch.21's quadratic, and delivers
%     exactly the commanded g.  Both computed in TABLE22.
if isfield(R, 'tbl22') && ~isempty(R.tbl22)
    K.rt_resid = max(R.tbl22.roundtrip_rel);
    K.rt_tol   = 1e-9;
    K.rt_ok    = K.rt_resid <= K.rt_tol;
    K.g_resid  = max(abs(R.tbl22.nl_at_trim/R.tbl22.xncg - 1));
    K.g_tol    = 1e-9;
    K.g_ok     = K.g_resid <= K.g_tol;
else
    K.rt_resid = NaN; K.rt_tol = NaN; K.rt_ok = false;
    K.g_resid  = NaN; K.g_tol  = NaN; K.g_ok  = false;
end

% 13. The nonlinear twin settles where the linear trim says it should.  This
%     is the delete-experiment for the trim quadratic: get the quadratic
%     wrong and the linear model still runs, still oscillates, still looks
%     like a missile -- and sits at the wrong angle of attack.
%     Only meaningful on the FIN trim: the nonlinear twin is stepped with
%     cfg.deldeg, so on the 'accel' trim it is settling at a different fin
%     angle than the linearisation point and the two are not supposed to agree.
K.trimnl_applies = cfg.nonlinear && ~isempty(R.non) && strcmp(R.fc.trim.mode, 'fin');
if K.trimnl_applies
    K.trimnl_meas  = mean(R.non.alf_deg);
    K.trimnl_form  = R.fc.trim.alftr_deg;
    K.trimnl_resid = abs(K.trimnl_meas/K.trimnl_form - 1);
    K.trimnl_tol   = 0.02;      % the window mean of a decaying oscillation
    K.trimnl_ok    = K.trimnl_resid <= K.trimnl_tol;
else
    K.trimnl_resid = NaN; K.trimnl_tol = NaN; K.trimnl_ok = false;
end

% Also only meaningful on the fin trim -- see the bench row that cites it.
K.otherroot_applies = strcmp(R.fc.trim.mode, 'fin');

% The unconditional ones are the algebraic identities -- they hold at every
% flight condition and every step size, so there is no legitimate reason for
% any of them to be n/a.  The conditional ones are MEASUREMENTS off the run,
% and a run too short to measure them cannot disprove anything.
K.ok = K.poly_ok && K.damp_ok && K.dc_ok && K.peak_ok && K.peakw_ok && ...
       K.trim_ok && K.k3_ok && K.rate_ok && K.trimalt_ok && K.rt_ok && K.g_ok && ...
       (K.lag_ok       || ~K.lag_applies) && ...
       (K.per_ok       || ~K.per_applies) && ...
       (K.ss_ok        || ~K.ss_applies) && ...
       (K.otherroot_ok || ~K.otherroot_applies) && ...
       (K.trimnl_ok    || ~K.trimnl_applies);
% How much of the self-check actually ran.  A caller that switched everything
% into n/a must be able to see that, rather than reading K.ok = true as proof.
K.n_applicable = 11 + K.lag_applies + K.per_applies + K.ss_applies + ...
                 K.otherroot_applies + K.trimnl_applies;
K.n_total = 16;
end


% =============================================================================
%   THE BOARD
% =============================================================================
function B = benchmarks(R, cfg, offbook)
%BENCHMARKS  Computed vs published, with a band, a reason for the band, and
%   the table/figure/page each value came from.
%
%   FOUR TOLERANCE CLASSES, and the difference is PROVENANCE, not taste:
%     'table'  1%   printed in a table to three significant figures
%     'text'   2%   printed in running prose to two or three figures
%     'approx' 5%   the book says "approximately"
%     'graph' 15%   the number is the author reading his own plot
%   plus 'shape' and 'bound', which are inequalities and orderings taken from
%   prose and carry no tolerance at all.
%
%   Do NOT widen a band to turn a row green.
%
%   EVERY BOOK NUMBER ON THIS BOARD WAS GREPPED OUT OF zarchan.txt.  Values
%   that exist only inside a figure did not survive extraction and are not
%   here at all; where the book's only statement is an adjective the row is
%   INFO and carries the measured number instead of a verdict.

B = emptyBench();

% GATES.  Note what is NOT in them: 'alt', 'deldeg', 'trim' and 'xncg' do not
% gate the published-case rows, because COMPARECASES, SWEEPS and TABLE22 all
% pin those to the book's own values internally (each says so in its header).
% Gating on a parameter the row does not actually use would SKIP rows that are
% perfectly evaluable, and a board that skips silently is the failure this
% whole convention exists to stop.
GATE_AERO = {'vm','wgt','a'};                    % moves every derivative
GATE_SIM  = [GATE_AERO, {'h','tstop'}];          % ... and the integration
GATE_GRID = [GATE_AERO, {'altgrid','vmgrid'}];   % ... and the sweep points

wA = gateWhy(offbook, GATE_AERO);
wS = gateWhy(offbook, GATE_SIM);
wG = gateWhy(offbook, GATE_GRID);

T = R.tbl22;  C = R.cmp;  S = R.sweep;

% -------------------------------------------------------------------------
% A.  TABLES 22.1 and 22.2 -- the only HARD numbers published for the six
%     quantities Listing 21.2 exists to compute.  Quoted at a 10-g trim, so
%     reaching them needs Listing 22.1's quadratic (pulled in, see header).
%     K1 and K3 are scored against the SIGN-RESTORED values; the row detail
%     carries the proof.
% -------------------------------------------------------------------------
S221 = 'Table 22.1, p.502 (zarchan.txt:6932-6937)';
S222 = 'Table 22.2, p.507 (zarchan.txt:6976-6977)';
signNote = ['book prints "23.07"/"21.89"; the leading 2 is an eaten minus ' ...
            '(K3 = 1845*K1/VM proves it: 1845*(-3.07)/3000 = -1.888 = the ' ...
            'published -1.89, whereas +23.07 would give +14.19)'];
rows = { ...
  'wAF  @  0 kft (10-g trim)', 25.3,    'rad/s', 'waf', 1, S221, 'wAF = sqrt(-Ma)'
  'wAF  @ 50 kft (10-g trim)', 10.0,    'rad/s', 'waf', 2, S221, 'wAF = sqrt(-Ma)'
  'zAF  @  0 kft (10-g trim)', 0.058,   '-',     'zaf', 1, S221, 'zAF = Za*wAF/(2*Ma)'
  'zAF  @ 50 kft (10-g trim)', 0.027,   '-',     'zaf', 2, S221, 'zAF = Za*wAF/(2*Ma)'
  'wz   @  0 kft (10-g trim)', 43.2,    'rad/s', 'wz',  1, S221, 'the RHP airframe zero'
  'wz   @ 50 kft (10-g trim)', 18.9,    'rad/s', 'wz',  2, S221, 'the RHP airframe zero'
  'K1   @  0 kft (10-g trim)', -3.07,   'g/deg', 'xk1', 1, S221, signNote
  'K1   @ 50 kft (10-g trim)', -0.559,  'g/deg', 'xk1', 2, S221, signNote
  'Ta   @  0 kft (10-g trim)', 0.457,   's',     'ta',  1, S222, 'turning rate time constant'
  'Ta   @ 50 kft (10-g trim)', 2.40,    's',     'ta',  2, S222, 'turning rate time constant'
  'K3   @  0 kft (10-g trim)', -1.89,   '1/s',   'xk3', 1, S222, signNote
  'K3   @ 50 kft (10-g trim)', -0.344,  '1/s',   'xk3', 2, S222, signNote };
for k = 1:size(rows,1)
    if ~isempty(wA)
        B = addSkip(B, rows{k,1}, sprintf('%g', rows{k,2}), wA, rows{k,6}, rows{k,3});
    else
        B = addVal(B, rows{k,1}, rows{k,2}, T.(rows{k,4})(rows{k,5}), 'table', ...
                   rows{k,6}, rows{k,3}, rows{k,7});
    end
end

% -------------------------------------------------------------------------
% B.  The nominal case, Figs. 21.4/21.5 (zarchan.txt:6527-6528, 6530, 6540).
%     Prose numbers, but the author is reading his own plot -> band 'graph'.
%     The two angle-of-attack values carry restored minus signs; the detail
%     says why on each row.
% -------------------------------------------------------------------------
i1 = 1;   % Fig. 21.9 case in R.cmp == sea level, 5 deg
alphaNote = ['book prints "28 deg"/"25 deg" but the same sentence says ' ...
             '"NEGATIVE angle of attack" -- the leading 2 is an eaten minus. ' ...
             'Scored on magnitude; the book''s own Listing 21.1, transcribed ' ...
             'here, peaks at -8.32 and trims at -4.45'];
% Two of these four rows need the Listing 21.1 twin and two do not, so they
% carry different gates.  Lumping them would skip the LINEAR trim angle -- the
% number this listing actually computes -- for a reason that does not apply.
wNL = wS;
if isempty(wNL) && ~cfg.nonlinear
    wNL = 'the Listing 21.1 twin was switched off (''nonlinear'', false)';
end
if ~isempty(wNL)
    B = addSkip(B, 'peak |alpha|, SL 5-deg fin', '8', wNL, ...
        'Fig. 21.4 text, p.481 (zarchan.txt:6527)', 'deg');
else
    B = addVal(B, 'peak |alpha|, SL 5-deg fin', 8, C(i1).alf_peak_deg, 'graph', ...
        'Fig. 21.4 text, p.481 (zarchan.txt:6527)', 'deg', alphaNote);
end
if ~isempty(wS)
    B = addSkip(B, 'trim |alpha|, SL 5-deg fin', '5', wS, ...
        'Fig. 21.4 caption, p.481 (zarchan.txt:6530)', 'deg');
else
    B = addVal(B, 'trim |alpha|, SL 5-deg fin', 5, abs(C(i1).alftr_deg), 'graph', ...
        'Fig. 21.4 caption, p.481 (zarchan.txt:6530)', 'deg', ...
        [alphaNote '.  Ours is the ANALYTIC root of 21.2''s trim quadratic']);
end
if ~isempty(wNL)
    B = addSkip(B, 'steady-state |nL| nonlinear', '13', wNL, ...
        'Fig. 21.5 text, p.481 (zarchan.txt:6528)', 'g');
else
    B = addVal(B, 'steady-state |nL| nonlinear', 13, abs(C(i1).nl_mean_non), 'graph', ...
        'Fig. 21.5 text, p.481 (zarchan.txt:6528)', 'g', ...
        ['window mean of the Listing 21.1 trace -- Fig. 21.5 IS the ' ...
         'nonlinear run.  Note this one has NO eaten minus even though nL ' ...
         'is negative, so the corruption is not systematic']);
end
if ~isempty(wS)
    B = addSkip(B, 'steady-state |nL| linear (K1*del)', '13', wS, ...
        'Fig. 21.9 text, p.488 (zarchan.txt:6747)', 'g');
else
    B = addVal(B, 'steady-state |nL| linear (K1*del)', 13, abs(C(i1).nl_ss), 'graph', ...
        'Fig. 21.9 text, p.488 (zarchan.txt:6747)', 'g', ...
        'the linear DC gain K1*del -- the number Fig. 21.9 overlays on 21.5');
end

% -------------------------------------------------------------------------
% C.  LINEAR vs NONLINEAR.  The book states these as ORDERINGS, never as a
%     number -- see the header section.  Scored as orderings.
% -------------------------------------------------------------------------
srcDet = 'Fig. 21.10 text, p.491 (zarchan.txt:6770)';
srcAlt = 'Fig. 21.11 text, p.491 (zarchan.txt:6771)';
if ~isempty(wS) || ~cfg.nonlinear
    why = wS; if isempty(why), why = 'the Listing 21.1 twin was switched off (''nonlinear'', false)'; end
    B = addSkip(B, 'fidelity worse at 10 deg than 5', '10deg > 5deg', why, srcDet, '-');
    B = addSkip(B, 'fidelity worse at 50 kft than SL', '50kft > SL', why, srcAlt, '-');
else
    ok1 = C(2).rms_norm > C(1).rms_norm;
    B = addShape(B, 'fidelity worse at 10 deg than 5', ...
        '"starts to deteriorate"', ...
        sprintf('%.2f%% > %.2f%%', 100*C(2).rms_norm, 100*C(1).rms_norm), ok1, srcDet, ...
        ['normalised rms|linear - nonlinear| over the 2-s window.  ' ...
         'NORMALISED because the raw rms would rank the cases by signal ' ...
         'size, not by fidelity.  The book''s stated mechanism is the ' ...
         'alpha^2 term, and |alpha| trim goes 4.45 -> 9.06 deg']);
    ok2 = C(3).rms_norm > C(1).rms_norm;
    B = addShape(B, 'fidelity worse at 50 kft than SL', ...
        '"expect ... less accurate"', ...
        sprintf('%.2f%% > %.2f%%', 100*C(3).rms_norm, 100*C(1).rms_norm), ok2, srcAlt, ...
        ['the book PREDICTS this one sentence before calling the result ' ...
         '"very well" -- the prediction is scored, the adjective is an ' ...
         'INFO row below']);
end

% -------------------------------------------------------------------------
% D.  Figs. 21.6 / 21.7 -- what changes with altitude at a fixed fin angle.
% -------------------------------------------------------------------------
src67 = 'Figs. 21.6/21.7 text, p.481 (zarchan.txt:6529)';
if ~isempty(wS) || ~cfg.nonlinear
    why = wS; if isempty(why), why = 'the Listing 21.1 twin was switched off'; end
    B = addSkip(B, 'more alpha at 50 kft (transient)', '50kft > SL', why, src67, 'deg');
else
    okA = C(3).alf_peak_deg > C(1).alf_peak_deg;
    B = addBound(B, 'more alpha at 50 kft (transient)', '"more angle of attack"', ...
        sprintf('%.3f > %.3f deg', C(3).alf_peak_deg, C(1).alf_peak_deg), okA, ...
        src67, 'deg', ...
        ['PEAK, not trim.  At a fixed fin angle the TRIM alpha is ' ...
         'altitude-invariant (Q divides out of CM = 0; the self-check ' ...
         'requires the residual to be exactly 0), so the only sense in ' ...
         'which 50 kft pulls more alpha is the undamped first overshoot. ' ...
         'MARGIN IS ONLY 4.6% -- read the row, do not just read the tick']);
end
if ~isempty(wA)
    B = addSkip(B, 'less accel available at 50 kft', '50kft < SL', wA, src67, 'g/deg');
    B = addSkip(B, 'more alpha at 50 kft (10-g trim)', '50kft > SL', wA, src67, 'deg');
else
    okB = abs(T.xk1(2)) < abs(T.xk1(1));
    B = addBound(B, 'less accel available at 50 kft', '"less acceleration available"', ...
        sprintf('|K1| %.4f < %.4f g/deg', abs(T.xk1(2)), abs(T.xk1(1))), okB, ...
        src67, 'g/deg', 'the acceleration gain per degree of fin, Fig. 21.7');
    okC = abs(T.alftr_deg(2)) > abs(T.alftr_deg(1));
    B = addBound(B, 'more alpha at 50 kft (10-g trim)', '"more angle of attack"', ...
        sprintf('%.3f > %.3f deg', abs(T.alftr_deg(2)), abs(T.alftr_deg(1))), okC, ...
        src67, 'deg', ...
        ['the SAME sentence, read the Ch.22 way: hold the ACCELERATION ' ...
         'fixed instead of the fin, and the claim becomes unambiguous ' ...
         '(2.81 -> 14.74 deg, and the fin needed goes -3.26 -> -17.87 deg, ' ...
         'past any real fin limit -- which is the point the book is making)']);
end

% -------------------------------------------------------------------------
% E.  The Experiments sweeps, Figs. 21.12-21.17.  DIRECTIONS from prose.
%     The grid is OURS (the figures' axes did not survive extraction) and
%     every row says so.
% -------------------------------------------------------------------------
gridNote = ['direction is the book''s, the grid is ours: alt %s ft, ' ...
            'VM %s ft/s at the 5-deg fin trim'];
gn = sprintf(gridNote, mat2str(S.alt(:).'), mat2str(S.vm(:).'));
trends = { ...
 'wAF falls with altitude',   'Fig. 21.12, p.492 (zarchan.txt:6784)', @() allDown(S.waf_alt)
 'wAF rises with speed',      'Fig. 21.12, p.492 (zarchan.txt:6784)', @() allUp(S.waf_vm(:,1)) && allUp(S.waf_vm(:,2))
 'zAF falls with altitude',   'Fig. 21.13, p.494 (zarchan.txt:6798)', @() allDown(S.zaf_alt)
 'zAF falls with speed',      'Fig. 21.13, p.494 (zarchan.txt:6798)', @() allDown(S.zaf_vm(:,1)) && allDown(S.zaf_vm(:,2))
 'wz  falls with altitude',   'Fig. 21.17, p.497 (zarchan.txt:6835)', @() allDown(S.wz_alt)
 'wz  rises with speed',      'Fig. 21.17, p.497 (zarchan.txt:6835)', @() allUp(S.wz_vm(:,1)) && allUp(S.wz_vm(:,2))
 'Ta  rises with altitude',   'Fig. 21.16, p.495 (zarchan.txt:6800)', @() allUp(S.ta_alt)
 'Ta  rises with speed',      'Fig. 21.16, p.495 (zarchan.txt:6800)', @() allUp(S.ta_vm(:,1)) && allUp(S.ta_vm(:,2))
 '|Ma| falls with altitude',  'Fig. 21.14, p.495 (zarchan.txt:6811)', @() allDown(abs(S.xma_alt))
 '|Ma| rises with speed',     'Fig. 21.14, p.495 (zarchan.txt:6811)', @() allUp(abs(S.xma_vm(:,1))) && allUp(abs(S.xma_vm(:,2)))
 '|Md| falls with altitude',  'Fig. 21.15, p.495 (zarchan.txt:6812)', @() allDown(abs(S.xmd_alt))
 '|Md| rises with speed',     'Fig. 21.15, p.495 (zarchan.txt:6812)', @() allUp(abs(S.xmd_vm(:,1))) && allUp(abs(S.xmd_vm(:,2)))
 '|K1| falls with altitude',  'Fig. 21.7, p.483 (zarchan.txt:6529)',  @() allDown(abs(S.xk1_alt)) };
magNote = ['book plots Ma and Md and says they "get smaller"; for this ' ...
           'missile both are NEGATIVE (statically stable), so "smaller" is ' ...
           'scored on MAGNITUDE -- read as a signed number, Ma going ' ...
           '-624 -> -91.9 got larger, which is the opposite verdict'];
for k = 1:size(trends,1)
    dtl = gn;
    if contains(trends{k,1}, 'Ma') || contains(trends{k,1}, 'Md')
        dtl = [gn '.  ' magNote];
    end
    if ~isempty(wG)
        B = addSkip(B, trends{k,1}, 'monotone', wG, trends{k,2}, '-');
    else
        f  = trends{k,3};
        ok = f();
        B = addShape(B, trends{k,1}, 'monotone (prose)', vstrMono(ok), ok, ...
                     trends{k,2}, dtl);
    end
end

% Ta ~ 4 s at 50 kft, 3000 ft/s -- a NUMBER, in prose, off Fig. 21.16.
srcTa = 'Fig. 21.16 text, p.495 (zarchan.txt:6810)';
if ~isempty(wG)
    B = addSkip(B, 'Ta at 50 kft, 3000 ft/s', '4', wG, srcTa, 's');
else
    i50 = find(S.alt == 50000, 1);
    if isempty(i50)
        B = addSkip(B, 'Ta at 50 kft, 3000 ft/s', '4', ...
            'altgrid does not contain 50000 ft', srcTa, 's');
    else
        B = addVal(B, 'Ta at 50 kft, 3000 ft/s', 4, S.ta_alt(i50), 'graph', ...
            srcTa, 's', ...
            ['at the 5-DEG FIN trim, which is the evidence that Ch.21''s ' ...
             'figures use that trim: the 10-g trim would give 2.40 s (-40%)']);
    end
end

% -------------------------------------------------------------------------
% F.  Sign and structure claims that are stated flatly in prose.
% -------------------------------------------------------------------------
B = addBound(B, 'airframe zero is right-half-plane', '"right-half plane"', ...
    sprintf('zero at %+.4f rad/s', R.tf.rhp_zero_rad_s), R.tf.rhp_zero, ...
    'Ch.23 p.549 (zarchan.txt:7687)', 'rad/s', ...
    ['roots([-K1/wz^2, 0, K1]) -> +/-wz.  Taken with ROOTS, not asserted: ' ...
     'the sign of the s^2 term in the numerator is exactly what a ' ...
     'transcription error would flip, and the time response barely changes. ' ...
     'This zero is the "wrong-way tail effect" (p.497, zarchan.txt:6835): ' ...
     'the fin''s own lift pushes the nose the wrong way first']);

B = addBound(B, 'bare airframe is statically stable', 'Ma < 0 (p.493)', ...
    sprintf('Ma = %+.2f, poles Re %+.4f', R.fc.aero.XMA, real(R.tf.poles(1))), ...
    R.fc.aero.XMA < 0 && R.tf.stable, ...
    'p.493 (zarchan.txt:6799)', '1/s^2', ...
    ['"A positive Ma indicates that the bare airframe is unstable" -- for ' ...
     'this missile Ma < 0, which is also the precondition for wAF = ' ...
     'sqrt(-Ma) being real at all (see the guard)']);

B = addBound(B, 'damping is low (zAF well under 1)', '"quite low"', ...
    sprintf('zAF = %.5f', R.fc.tf.ZAF), R.fc.tf.ZAF < 0.1, ...
    'Fig. 21.13 text, p.494 (zarchan.txt:6798)', '-', ...
    'and the consequence: the step response overshoots and rings, next row');

% max(abs(.)) rather than min(.): a caller running a NEGATIVE fin step would
% make min(xnl) about zero and this row would fail on a correct port.
os = max(abs(R.lin.xnl))/abs(R.lin.nl_dc);
B = addBound(B, 'step response overshoots and rings', '"decaying oscillations"', ...
    sprintf('overshoot x%.3f, %.1f cycles', os, R.lag.cycles_in_run), ...
    os > 1.5 && R.lag.cycles_in_run > 2, ...
    'Fig. 21.4 text, p.481 (zarchan.txt:6527)', '-', ...
    ['a bare airframe at zAF = 0.05 overshoots to 2.2x its own steady ' ...
     'state -- this is the row that makes the "an autopilot is usually not ' ...
     'required to speed up the airframe" remark (p.493) misleading if read ' ...
     'as "no autopilot required"']);

% Gated on the trim mode, not on the aero: "the unrealistic root" is a remark
% about CHAPTER 21's quadratic.  Chapter 22's is a different quadratic with a
% different discarded root, and scoring that one against this sentence would
% be citing a page that does not say it.
if ~strcmp(R.fc.trim.mode, 'fin')
    B = addSkip(B, 'discarded trim root is unphysical', '"unrealistic root"', ...
        'the run is on Ch.22''s acceleration trim; the remark is about Ch.21''s', ...
        'p.487 (zarchan.txt:6683)', 'deg');
else
    B = addBound(B, 'discarded trim root is unphysical', '"unrealistic root"', ...
        sprintf('%.1f deg', R.fc.trim.root_discarded_deg), ...
        abs(R.fc.trim.root_discarded_deg) > 90, ...
        'p.487 (zarchan.txt:6683)', 'deg', ...
        'the book says it eliminates a root but does not print it; it is -244.8 deg');
end

if ~isempty(wA)
    B = addSkip(B, 'zAF in the quoted 0.02-0.065 band', '0.02..0.065', wA, ...
        'Fig. 21.13 text, p.494 (zarchan.txt:6798)', '-');
else
    okZ = all(T.zaf >= 0.02 & T.zaf <= 0.065);
    B = addBound(B, 'zAF in the quoted 0.02-0.065 band', '0.02 .. 0.065', ...
        sprintf('%.5f and %.5f', T.zaf(1), T.zaf(2)), okZ, ...
        'Fig. 21.13 text, p.494 (zarchan.txt:6798)', '-', ...
        ['containment of the two TABULATED values.  Scoreable here (and NOT ' ...
         'for wAF, see the INFO row) because 0.02 and 0.065 are not round ' ...
         'numbers and are therefore not the tabulated values rounded -- so ' ...
         'this tests the physics, not the printing']);
end

% -------------------------------------------------------------------------
% G.  REPORTED, NEVER SCORED.  Each says exactly why.
% -------------------------------------------------------------------------
if ~isempty(C) && ~isnan(C(1).rms_norm)
    B = addInfo(B, '"a near perfect match" (Fig 21.9)', 'adjective, no number', ...
        sprintf('rms %.2f%% of ss, max %.2f g', 100*C(1).rms_norm, C(1).maxabs), ...
        'Fig. 21.9 text, p.488 (zarchan.txt:6747)', ...
        ['the book attaches NO number to the agreement anywhere in Ch.21 -- ' ...
         'not a percentage, not an rms, not a peak error.  Scoring an ' ...
         'adjective means inventing the threshold, so the measured figures ' ...
         'are printed and the ORDERING rows above carry the verdict.  For ' ...
         'the record the worst discrepancy is at the FIRST OVERSHOOT ' ...
         '(-28.47 linear vs -25.91 nonlinear), not in the steady state']);
    B = addInfo(B, '"still ... very well" at 50 kft', 'adjective, no number', ...
        sprintf('rms %.2f%% of ss (SL is %.2f%%)', ...
                100*C(3).rms_norm, 100*C(1).rms_norm), ...
        'Fig. 21.11 text, p.491 (zarchan.txt:6771)', ...
        ['NOT SCORED, and the reason is worth stating: normalised, the ' ...
         '50-kft error is nearly DOUBLE sea level''s.  That is precisely ' ...
         'what the book predicts one sentence earlier, so the adjective and ' ...
         'the arithmetic point slightly different ways.  This file prints ' ...
         'both rather than widening "very well" to cover 11.6%']);
end
B = addInfo(B, 'wAF sweep range 10-30 rad/s', '10 .. 30 rad/s', ...
    sprintf('tabulated %.4f and %.4f', T.waf(1), T.waf(2)), ...
    'Fig. 21.12 text, p.492-493 (zarchan.txt:6784)', ...
    ['NOT SCORED for a specific reason: the endpoints are ROUND, and our ' ...
     '50-kft value 9.953 is the book''s own "10.0" before rounding.  ' ...
     'Testing containment would test the rounding, not the port -- it ' ...
     'would fail a correct port by 0.5%.  The figure''s actual sweep ' ...
     'limits became an image link and did not survive extraction.']);
B = addInfo(B, 'airframe tau range 0.033-0.1 s', '0.033 .. 0.1 s', ...
    sprintf('1/wAF = %.5f and %.5f', 1/T.waf(1), 1/T.waf(2)), ...
    'Fig. 21.12 text, p.492-493 (zarchan.txt:6784)', ...
    ['the reciprocal of the row above and NOT SCORED for the same reason ' ...
     '(0.033 = 1/30, 0.1 = 1/10 exactly -- it is the book doing arithmetic ' ...
     'on its own rounded endpoints).  R.lag carries the number this port ' ...
     'thinks is the useful one: 1/(zAF*wAF), which is 20x longer.']);
B = addInfo(B, 'Ta 5 s -> 4 s from adding wing+tail', '5 s (Ch.18)', ...
    sprintf('%.3f s with wing+tail', R.fc.tf.TA), ...
    'p.495 (zarchan.txt:6810)', ...
    ['NOT REPRODUCIBLE from this listing.  The 5 s is a DIFFERENT model ' ...
     '(Ch.18, nose and body only); you cannot get it here by zeroing SWING ' ...
     'and STAIL because STAIL = 0 kills Md, and with Md = 0 there is no ' ...
     'control moment and no transfer function left to have a Ta.']);
B = addInfo(B, 'Figs. 21.12-21.17 axis limits', 'not in the text', 'grid is ours', ...
    'Figs. 21.12-21.17, pp.492-497', ...
    ['every figure in the extraction became an image link, so the sweep ' ...
     'ranges the book actually plotted are unrecoverable.  The DIRECTION ' ...
     'rows above are the book''s claims tested on OUR grid, which is stated ' ...
     'on each of those rows rather than left implicit.']);
end


% =============================================================================
%   BENCH ROW PLUMBING  --  schema identical to MGS.ZARCHAN.ADJOINTWITHAUTOPILOT
%   so BENCHSUITE's adaptAutopilot reads these rows with no new adapter code.
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
case 'approx', t = 0.05;   % the book says "approximately"
case 'graph',  t = 0.15;   % the author read it off his own plot
otherwise
    error('mgs:zarchan:airframeLinear:band', ...
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
    w = sprintf('the caller moved %s away from the book value', strjoin(hit, ', '));
end
end

function s = offbookStr(offbook)
if isempty(offbook), s = 'nothing'; else, s = strjoin(offbook, ', '); end
end

function t = allUp(v)
%ALLUP  STRICTLY increasing.  Not issorted(): issorted() calls a flat vector
%   sorted, so a trend that does not move at all would pass as "rises with".
t = all(diff(v(:)) > 0);
end

function t = allDown(v)
t = all(diff(v(:)) < 0);
end

function s = vstrMono(ok)
if ok, s = 'monotone, sign correct'; else, s = 'NOT monotone'; end
end


% =============================================================================
%   RENDER
% =============================================================================
function s = render(R, cfg, offbook)
W = 92;
L = {};
L{end+1} = repmat('=', 1, W);
L{end+1} = 'mgs.zarchan.airframeLinear  --  Zarchan Listing 21.2, LINEAR AIRFRAME SIMULATION';
L{end+1} = '  units: ft, ft/s, deg, deg/s, g = 32.2 -- the book''s, NOT SI';
L{end+1} = sprintf('  flight condition: VM %g ft/s (Mach %.3f), ALT %g ft, RHO %.5e, Q %.1f psf', ...
    R.fc.vm, R.fc.mach, R.fc.alt, R.fc.rho, R.fc.q);
L{end+1} = sprintf('  trim (%s): alpha %+.4f deg, fin %+.4f deg, CM residual %.2e', ...
    R.fc.trim.mode, R.fc.trim.alftr_deg, R.fc.trim.deltr_deg, R.fc.trim.cm_residual);
if ~isempty(offbook)
    L{end+1} = sprintf('  OFF-BOOK: %s -- rows depending on these are SKIPPED, not rescored', ...
        offbookStr(offbook));
end
L{end+1} = repmat('-', 1, W);

L{end+1} = 'AERO DERIVATIVES AND TRANSFER FUNCTION PARAMETERS';
L{end+1} = sprintf('  CNa %8.4f   CNd %8.4f   CMa %8.4f   CMd %8.4f', ...
    R.fc.aero.CNA, R.fc.aero.CND, R.fc.aero.CMA, R.fc.aero.CMD);
L{end+1} = sprintf('  Ma %9.3f    Md %9.3f    Za %8.5f   Zd %8.5f  [1/s^2, 1/s]', ...
    R.fc.aero.XMA, R.fc.aero.XMD, R.fc.aero.ZA, R.fc.aero.ZD);
L{end+1} = sprintf('  wAF %7.4f rad/s   zAF %8.5f   wz %8.4f rad/s', ...
    R.fc.tf.WAF, R.fc.tf.ZAF, R.fc.tf.WZ);
L{end+1} = sprintf('  K1 %8.5f g/deg    K3 %8.5f 1/s    Ta %8.5f s', ...
    R.fc.tf.XK1, R.fc.tf.XK3, R.fc.tf.TA);
L{end+1} = repmat('-', 1, W);

L{end+1} = 'POLES AND ZEROS  --  roots(), not tf/damp (no Control System Toolbox)';
L{end+1} = sprintf('  poles      %+.6f %+.6fi   (and conjugate)', ...
    real(R.tf.poles(1)), abs(imag(R.tf.poles(1))));
L{end+1} = sprintf('  recovered  wn %.6f rad/s   zeta %.6f   wd %.6f rad/s', ...
    R.tf.wn, R.tf.zeta, R.tf.wd);
L{end+1} = sprintf('  nL/del zeros  %+.4f and %+.4f rad/s   -> RHP zero: %s', ...
    min(real(R.tf.zeros_nl)), max(real(R.tf.zeros_nl)), tf2s(R.tf.rhp_zero));
L{end+1} = sprintf('  thd/del zero  %+.4f rad/s (= -1/Ta, LEFT half plane)', R.tf.zero_thd(1));
L{end+1} = sprintf('  |nL/del| DC %.5f g/deg (= K1)   resonant peak %.4f at %.4f rad/s', ...
    abs(R.tf.dc_gain), R.tf.peak_mag, R.tf.peak_w);
L{end+1} = repmat('-', 1, W);

L{end+1} = 'THE LAG  --  what a linear model gives you that a fitted curve does not';
L{end+1} = sprintf('  1/wAF                    %8.5f s   the book''s "airframe time constant"', ...
    R.lag.tau_wn_s);
L{end+1} = sprintf('  1/(zAF*wAF)              %8.5f s   -1/Re(pole): how long it RINGS', ...
    R.lag.tau_env_s);
if isnan(R.lag.tau_env_meas_s)
    L{end+1} = sprintf('  measured off the run          --       only %d extrema in %.3g s', ...
        R.lag.n_peaks, cfg.tstop);
else
    L{end+1} = sprintf('  measured off the run     %8.5f s   envelope fit, %d extrema', ...
        R.lag.tau_env_meas_s, R.lag.n_peaks);
end
L{end+1} = sprintf('  ratio 1/zAF              %8.2fx   the factor by which quoting 1/wAF alone', ...
    R.lag.ratio);
L{end+1} = '                                        understates how long the transient is present';
L{end+1} = repmat('-', 1, W);

% ---- structural self-check ----------------------------------------------
if isfield(R.selfcheck, 'ok')
    K = R.selfcheck;
    L{end+1} = sprintf(['STRUCTURAL SELF-CHECK (wiring, not the book -- a failure ' ...
        'here voids everything)   %d/%d apply'], K.n_applicable, K.n_total);
    L{end+1} = sprintf('  polynomial in descending powers of s         : %9.3e  tol %.0e  %s', ...
        K.poly_resid, K.poly_tol, tf2s(K.poly_ok));
    L{end+1} = sprintf('  roots() returns the listing''s own wAF, zAF   : %9.3e  tol %.0e  %s', ...
        K.damp_resid, K.damp_tol, tf2s(K.damp_ok));
    L{end+1} = sprintf('  DC gain of nL/del == K1                      : %9.3e  tol %.0e  %s', ...
        K.dc_resid, K.dc_tol, tf2s(K.dc_ok));
    L{end+1} = sprintf('  measured ringing tau vs 1/(zAF*wAF)          : %9.3e  tol %.0e  %s', ...
        K.lag_rel, K.lag_tol, okOrNA(K.lag_ok, K.lag_applies));
    if ~K.lag_applies
        L{end+1} = sprintf('      n/a because %s', K.lag_why);
    end
    L{end+1} = sprintf('  measured period vs 2*pi/wd                   : %9.3e  tol %.0e  %s', ...
        K.per_rel, K.per_tol, okOrNA(K.per_ok, K.per_applies));
    L{end+1} = sprintf('  resonant peak vs |K1|(1+(wAF/wz)^2)/(2 zAF)  : %9.3e  tol %.0e  %s', ...
        K.peak_resid, K.peak_tol, tf2s(K.peak_ok));
    L{end+1} = sprintf('  resonant peak sits at wAF                    : %9.3e  tol %.0e  %s', ...
        K.peakw_resid, K.peakw_tol, tf2s(K.peakw_ok));
    L{end+1} = sprintf('  CM == 0 at trim (nonlinear CM, not the quad) : %9.3e  tol %.0e  %s', ...
        K.trim_resid, K.trim_tol, tf2s(K.trim_ok));
    L{end+1} = sprintf('  discarded trim root %8.1f deg, |.| > 90   : %9s  %s', ...
        K.otherroot_deg, '', okOrNA(K.otherroot_ok, K.otherroot_applies));
    L{end+1} = sprintf('  K3 == 1845*K1/VM                            : %9.3e  tol %.0e  %s', ...
        K.k3_resid, K.k3_tol, tf2s(K.k3_ok));
    L{end+1} = sprintf('  thd_ss == 57.3*32.2*nL_ss/VM (1845 rounding) : %9.3e  tol %.0e  %s', ...
        K.rate_resid, K.rate_tol, tf2s(K.rate_ok));
    L{end+1} = sprintf('  simulated steady state vs K1*del             : %9.3e  tol %.0e  %s', ...
        K.ss_resid, K.ss_tol, okOrNA(K.ss_ok, K.ss_applies));
    if ~K.ss_applies
        L{end+1} = sprintf(['      n/a: run is %.3g s but the ringing decays with ' ...
            'tau = %.3g s -- not settled'], cfg.tstop, R.lag.tau_env_s);
    end
    L{end+1} = sprintf('  fin trim is altitude-invariant (Q divides out): %9.3e  tol %g       %s', ...
        K.trimalt_resid, K.trimalt_tol, tf2s(K.trimalt_ok));
    L{end+1} = sprintf('  Ch.22 trim round-trips through Ch.21''s quad  : %9.3e  tol %.0e  %s', ...
        K.rt_resid, K.rt_tol, tf2s(K.rt_ok));
    L{end+1} = sprintf('  Ch.22 trim delivers exactly the commanded g  : %9.3e  tol %.0e  %s', ...
        K.g_resid, K.g_tol, tf2s(K.g_ok));
    L{end+1} = sprintf('  nonlinear twin settles at the linear trim    : %9.3e  tol %.0e  %s', ...
        K.trimnl_resid, K.trimnl_tol, okOrNA(K.trimnl_ok, K.trimnl_applies));
else
    L{end+1} = 'STRUCTURAL SELF-CHECK  ***NOT RUN*** (benchmark = false) -- nothing is verified';
end
L{end+1} = repmat('-', 1, W);

% ---- linear vs nonlinear -------------------------------------------------
if ~isempty(R.cmp)
    L{end+1} = 'LINEAR vs NONLINEAR (Listing 21.1 pulled in) -- the book states these as';
    L{end+1} = 'ORDERINGS ONLY.  No number for the agreement is published anywhere in Ch.21.';
    L{end+1} = sprintf('  %-22s %9s %9s %9s %9s %9s', ...
        'case', 'rms[g]', 'rms/ss', 'max[g]', 'pk|alf|', 'ss[g]');
    for i = 1:numel(R.cmp)
        c = R.cmp(i);
        L{end+1} = sprintf('  %-22s %9.4f %8.2f%% %9.4f %9.3f %9.3f', ...
            c.name, c.rms, 100*c.rms_norm, c.maxabs, c.alf_peak_deg, c.nl_ss); %#ok<AGROW>
    end
    L{end+1} = repmat('-', 1, W);
end

% ---- Tables 22.1 / 22.2 --------------------------------------------------
if ~isempty(R.tbl22)
    T = R.tbl22;
    L{end+1} = 'TABLES 22.1 / 22.2 REPRODUCED (10-g trim, Listing 22.1''s quadratic pulled in)';
    L{end+1} = sprintf('  %8s %9s %9s %9s %10s %9s %9s %9s', ...
        'alt[ft]', 'wAF', 'zAF', 'wz', 'K1', 'K3', 'Ta', 'alf[deg]');
    for i = 1:numel(T.alt)
        L{end+1} = sprintf('  %8g %9.4f %9.5f %9.4f %10.5f %9.5f %9.5f %9.4f', ...
            T.alt(i), T.waf(i), T.zaf(i), T.wz(i), T.xk1(i), T.xk3(i), ...
            T.ta(i), T.alftr_deg(i)); %#ok<AGROW>
    end
    L{end+1} = '  book: 25.3/10.0  0.058/0.027  43.2/18.9  -3.07/-0.559  -1.89/-0.344  0.457/2.40';
    L{end+1} = '  (K1 and K3 sign-restored -- the printed "23.07"/"21.89" have an eaten minus;';
    L{end+1} = '   K3 = 1845*K1/VM is the proof, see R.bench detail on those rows)';
    L{end+1} = repmat('-', 1, W);
end

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
s = strjoin(L, newline);
end


function drawCurves(R, cfg)
%DRAWCURVES  The listing's own two plots, plus what the frequency domain adds.
figure('Name','Zarchan Listing 21.2 -- linear airframe');
subplot(2,2,1);
plot(R.lin.t, R.lin.xnl, 'LineWidth', 1.2); grid on; hold on;
if ~isempty(R.non), plot(R.non.t, R.non.xnl_g, '--', 'LineWidth', 1.2); end
yline(R.lin.nl_dc, ':');
xlabel('Time (Sec)'); ylabel('Missile Acceleration (G)');
if ~isempty(R.non), legend('linear (21.2)','nonlinear (21.1)','K1*del','Location','best');
else, legend('linear (21.2)','K1*del','Location','best'); end
title(sprintf('ALT %g ft, fin %g deg', cfg.alt, cfg.deldeg));

subplot(2,2,2);
plot(R.lin.t, R.lin.thd, 'LineWidth', 1.2); grid on;
xlabel('Time (Sec)'); ylabel('Body Rate (Deg/Sec)');

subplot(2,2,3);
plot(real(R.tf.poles), imag(R.tf.poles), 'x', 'MarkerSize', 12, 'LineWidth', 2);
hold on; grid on;
plot(real(R.tf.zeros_nl), imag(R.tf.zeros_nl), 'o', 'MarkerSize', 10, 'LineWidth', 2);
xline(0, 'k:');
xlabel('Real (1/s)'); ylabel('Imag (1/s)');
title('poles (x) and nL/del zeros (o) -- one zero is in the RIGHT half plane');

subplot(2,2,4);
loglog(R.tf.w, R.tf.mag_nl, 'LineWidth', 1.2); grid on; hold on;
xline(R.fc.tf.WAF, ':'); xline(R.fc.tf.WZ, '--');
xlabel('\omega (rad/s)'); ylabel('|nL/\delta| (g/deg)');
title('freq response by polyval at j\omega (no bode)');
end


% =============================================================================
function s = tf2s(b)
if b, s = 'PASS'; else, s = 'FAIL'; end
end

function s = okOrNA(b, applies)
%OKORNA  'n/a' is NOT 'PASS'.  A check that does not apply in this mode must
%   print as neither, or a mode that switches every check off reads green.
if ~applies, s = 'n/a '; elseif b, s = 'PASS'; else, s = 'FAIL'; end
end

function s = bandTag(band, tol)
switch lower(band)
case 'table',  s = 'tbl 1%';
case 'text',   s = 'txt 2%';
case 'approx', s = 'apx 5%';
case 'graph',  s = 'fig 15%';
otherwise,     s = sprintf('%.0f%%', 100*tol);
end
end

function s = trunc(str, n)
str = char(str);
if numel(str) > n, s = [str(1:n-1) '~']; else, s = str; end
end
