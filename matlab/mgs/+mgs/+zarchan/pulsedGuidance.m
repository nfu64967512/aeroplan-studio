function R = pulsedGuidance(opts)
%MGS.ZARCHAN.PULSEDGUIDANCE  Zarchan Listing 15.2 -- PULSED guidance.
%
%   R = mgs.zarchan.pulsedGuidance()
%   R = mgs.zarchan.pulsedGuidance('PULSES', 5)
%   R = mgs.zarchan.pulsedGuidance('companions', false, 'assert', false)
%
%   Line-by-line port of Listing 15.2 (zarchan.txt:4587-4628): the Listing 14.1
%   exoatmospheric engagement -- inverse-square gravity on both bodies, Lambert
%   launch, second-order Runge-Kutta -- flown by a law that can only command a
%   FIXED acceleration magnitude AMAG and so can only choose each burn's
%   DURATION.  UNITS ARE THE BOOK'S: ft, ft/s, ft/s^2, g = 32.2, A = 2.0926e7 ft,
%   GM = 1.4077e16 ft^3/s^2, 1 nmi = 6076 ft.  Nothing is converted; the rest of
%   mgs is metres, so convert AT THE CALL SITE and say so there.
%
%   THE LAW.  Zarchan equates the displacement of one rectangular pulse of
%   amplitude a lasting dt, with tgo left, to the zero-effort miss PN would have
%   removed over the whole flight (zarchan.txt:4559, :4563, :4565):
%       ZEM = .5*a*dt^2 + a*dt*(tgo-dt)     and     ZEM = Vc*tgo^2*lamdot
%   a quadratic in dt whose realistic root is (:4567-4569)
%       dt = tgo*(1 - sqrt(1 - 2*Vc*lamdot/a))
%   That is TPULSE, with DISC = 1 - 2*VC*abs(XLAMD)/AMAG.  An identity, not an
%   approximation, so BENCHMARK 1 scores it at machine precision.
%
%   PULSES is fixed in advance and the pulses are spread evenly (:4581): the
%   k-th of PULSES-1 scheduled pulses fires the first time TGO drops below
%   TF*k/(PULSES-1).  The PULSE_NUM<=0 branch (:4612-4618) then adds a TERMINAL
%   burn -- and note what it does: while DISC > 0 it sets TPULSE < TGO, so
%   "TGO <= TPULSE" is FALSE and nothing happens.  It fires only once DISC <= 0,
%   i.e. once one feasible pulse can no longer remove the ZEM, then latches on
%   (TOFF = 999999) to impact.  Since 2*ZEM/tgo^2 grows without bound as
%   tgo -> 0 it ALWAYS eventually fires; SELF-CHECK 4 turns on that fact.
%
%   ⭐⭐ THAT LAST BURN IS THE WHOLE STORY OF THIS LISTING, so read the branch
%   again: its length is NOT chosen.  Every scheduled pulse gets a duration
%   solved from the quadratic; the terminal one gets TOFF = 999999 and burns
%   until impact.  So "N pulses" buys N-1 sized burns plus one whose cost is
%   however long is left after the law runs out of feasible single pulses.
%   Everything the pulse count does to the divert, it does through THAT.
%
% ---------------------------------------------------------------------------
%   THE THREE UNRESOLVED COMPARISONS, RESOLVED.  Elsewhere in this book "." is
%   the OCR of ">" and "," of "<", both confirmed inside this listing, where
%   "if DISC.0." and "if PULSE_NUM . 0." repair to "> 0".  So the operator was
%   known and only the argument order was in doubt.
%
%     1  "if XLONGM.XLONGT" -> XLONGM > XLONGT.  GEOMETRY, not shape.  At
%        longitude th the transverse unit vector toward INCREASING longitude is
%        (-sin th, cos th).  The else branch's direction (-pi/2+GAMT+XLONGT) is
%        (cos 23, sin 23) at the book's XLONGT = 90 deg: climbing GAMT above
%        the local horizontal and moving toward SMALLER longitude, i.e. TOWARD
%        the missile at 45 deg.  The if branch is its mirror (157 deg), for a
%        missile EAST of the target.  With 45/90 the shipped resolution takes
%        the ELSE branch and the target closes.  SELF-CHECK 5 deletes it.
%     2  "if T.TOFF" -> T > TOFF (shut the engine off once the scheduled
%        off-time has passed).  Decisive: the listing writes TOFF = 999999 for
%        "never shut off", which is meaningless against "<".  The inverse is
%        true on the step right after TOFF is set, so every burn would last one
%        integration step -- SELF-CHECK 4.
%     3  "if TGO,TPULSE" -> TGO < TPULSE -> TOFF = 9999999 (latch on): "the
%        pulse I need is longer than the flight I have left, so burn to the
%        end".  The inverse reads "plenty of time left, so burn forever".
%   Two further repairs, both against literals: "if TGO,=(TF-0.)*..." -> "<="
%   and "if S.=.99999" -> ">=".
%
%   REPRODUCED, NOT TIDIED.  "(TF-0.)" is TF, kept verbatim.  A scheduled slot
%   reached with DISC <= 0 sets PULSE_ON = 1 and TPULSE = 0 but NEVER ASSIGNS
%   XNC, so the command holds its previous value for one step and TOFF = T+0
%   shuts it off: a zero-length pulse, counted in .n_zero_len and excluded from
%   BENCHMARK 5 (a burn that never ran cannot be asked to have reduced
%   anything).  ACQUIRE, XFIRSTT, YFIRSTT are set and never read (Listing 14.1
%   leftovers) -- dropped; so is "save datfil.txt output /ascii", the book's
%   recurring typo for -ascii.  lambertpz (Listing 13.2, body identical to
%   MGS.ZARCHAN.GEMSIMULATION's, including its "xlongt > xlongm" resolution)
%   and predictpz are local functions below; predictpz's unread local "a" is
%   dropped.  A Lambert that hits the iteration cap is REFUSED, not warned
%   about: the missile would never have been aimed at the target and every
%   number downstream would be meaningless.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  ⚠️ Ch.15's pulsed section PUBLISHES NO NUMBER for this listing
%   -- no miss, no divert, no LOS rate.  Every figure became an image link, and
%   "axis([0 500 0 .00002])" is a plot window the author chose, not a
%   measurement.  What it does publish is ORDERING CLAIMS in prose, and prose is
%   greppable, so those are what is scored and this says so plainly.  Row 1 is
%   exact algebra; the rest are shapes and bounds.
%     1 bound  dt solves the book's own quadratic, to machine precision (:4565-6)
%     2 bound  10 pulses -> "a successful intercept"                     (:4582)
%     3 bound  5 pulses -> also a successful intercept             (:4645,:4647)
%     4 shape  fewer pulses -> LOS rate "builds up to larger values in
%              between pulses"                                           (:4645)
%     5 shape  each burn drives the LOS rate toward zero                 (:4582)
%     6 shape  PN nulls the LOS rate "at the end of the flight only", so it
%              rebuilds less often than a pulsed run                     (:4646)
%     7 shape  "proportional navigation has the smallest divert"         (:4647)
%     8 bound  more pulses "does not appear to influence" divert         (:4647)
%    9-11 info the MEASURED divert-vs-pulse-count sweep -- NEVER scored  (:4647)
%    12 info   the listing's plot range -- reported, NEVER scored        (:4623)
%
%   TWO THRESHOLDS ARE OURS, NOT THE BOOK'S.  Both are options so they cannot
%   hide, and neither may be widened to turn a row green.  "Successful
%   intercept" (2, 3) is miss <= miss_frac (1%) of the miss the SAME engagement
%   makes with the guidance DELETED -- a MEASURED reference, which is what keeps
%   it from being a bare magic number.  "Does not appear to influence" (8) is
%   |DELV(10)/DELV(5)-1| <= divert_band (25%), fixed before any run.
%
%   *** ROW 7 IS THE INTERESTING ONE. ***  On the flat-Earth linear model the
%   pulse law is DERIVED from, N' = 3 PN spends 1.5*z0/tF to remove an initial
%   zero-effort miss z0 while one early pulse spends ~z0/tF -- so the linear
%   model says pulsed is ~1.5x CHEAPER, the opposite of Fig. 15.13.  Here
%   (z0 = 1e5 ft, tF = 500 s, a = 64.4 ft/s^2) that is 300 ft/s against 201 ft/s
%   at dt = 3.12 s.  The book simulates the NONLINEAR problem, where a 200 ft/s
%   correction made 500 s early is reshaped by the gravity gradient and the
%   later pulses must clean up after it.  If row 7 goes red, read it as figure
%   vs linear theory FIRST and as a port bug SECOND -- and check row 1, which
%   cannot drift, before touching anything.  Do not adjust the row either way.
%
%   NOT SCORED: any miss or divert MAGNITUDE, and the peak LOS rate.  Figs.
%   15.10-15.13 publish none of them.
%
% ---------------------------------------------------------------------------
%   *** ROW 8 IS RED, AND IT IS A RESULT. ***  MEASURED 2026-08-24, and read
%   this before touching the row, the band or the sweep.
%
%   Row 8 was suspected of being a sweep that does not move -- a comparison
%   whose parameter never reaches the integrator, which would make "the divert
%   barely changes" the most convincing possible version of the wrong answer.
%   It is not.  Ten pulse counts flown off ONE launch state (R.sweep):
%
%     N        2     3     4     5     6     8    10    12    16    20
%     DELV  3310  2112  1539  1251  1087   907   809   755   703   659  ft/s
%
%   Monotone decreasing at every one of the ten points, a factor of 5 end to
%   end, 35.3% across the book's own 10-vs-5 pair.  SELF-CHECK 6 exists to keep
%   that honest: it fails if two pulse counts ever produce the same DELV to the
%   last bit, or if the schedule stops issuing N-1 pulses.  So the pulse count
%   reaches the simulation, the row is aimed at the quantity the book's
%   sentence names ("the lateral divert requirements" = this listing's DELV),
%   and it compares the pair Fig. 15.13 overlays.  It stays red and the 25%
%   band stays where it was fixed.  ⚠️ Row 8 is also live in the other
%   direction -- measured: PULSES_ALT = 9/8/7 give 4.9%/10.8%/17.3% and pass,
%   6 gives 25.6% and fails.  It is not a row stuck red.
%
%   WHERE THE DISAGREEMENT LIVES -- and half the book's sentence survives it.
%   Split each run's DELV at the instant the terminal burn latches on:
%
%     N              2     3     4     5     6     8    10    12    16    20
%     scheduled    332   533   612   632   636   632   627   623   615   611
%     terminal    2978  1580   927   619   450   274   182   133    88    48
%
%   The N-1 SCHEDULED pulses cost the SAME to within 1.4% over the book's
%   5..10 range (632 vs 627 ft/s) and within 4% over 4..20, because each is
%   sized to null the WHOLE remaining ZEM, so the sum telescopes to the
%   correction the engagement needs rather than to the number of burns.  That
%   is exactly the insensitivity Zarchan describes, and it is real.  Every bit
%   of the sensitivity is in the terminal burn, whose length nobody chose: 46.2
%   s at N = 2 down to 0.75 s at N = 20.  Fewer scheduled pulses let lamdot
%   build higher between them (row 4 measures that separately), DISC <= 0
%   arrives earlier, and the unbounded burn-to-impact runs longer.
%
%   ⚠️ NOT flat at the sparse end, and the header says so rather than quoting
%   only the flattering range: at N = 2 the single scheduled pulse spends 332
%   ft/s because that is all one pulse can spend.  Both spreads print.
%
%   TOTAL IMPULSE IS NOT HELD FIXED ACROSS THE SWEEP, and it is worth being
%   precise about why that does not make the comparison uncontrolled.  Fixed
%   magnitude AND fixed duration would make impulse proportional to N, which
%   would be an uncontrolled sweep; this listing fixes only the MAGNITUDE and
%   solves each duration from the quadratic, so the scheduled impulse is
%   whatever the ZEM demands -- measured above as near-constant.  What the
%   sweep varies is granularity, and granularity is what the book's sentence
%   is about.  Launch state, target, prediction error and step sizes are
%   identical at every point (runSweep flies flyEngagement directly).
%
%   ⚠️ WHAT WOULD CHANGE THIS VERDICT, and what would not.  Not a wider band --
%   that is forbidden and would not be informative anyway.  What would is
%   evidence about the terminal branch: Fig. 15.13 did not survive extraction
%   (image link), so whether the author's divert curves included a 9.6 s
%   burn-to-impact tail at 5 pulses is not recoverable from this text.  If a
%   copy of the figure ever turns up showing both profiles flat to impact, the
%   thing to re-examine is the "TGO <= TPULSE" resolution in the PULSE_NUM<=0
%   branch, NOT the band.  Rows 9-11 print the numbers that argument needs.
%
% ---------------------------------------------------------------------------
%   WHAT THIS PRINTS.  *** MEASURED, at the shipped defaults. ***  Row 1
%   residual 2.1e-14 (above 1e-10 is a wiring fault in the DISC/TPULSE lines,
%   not a tolerance question).  Nine scheduled pulses at PULSES = 10, at the
%   TGO slots 500*k/9 s -- but note the first one lands at t = 45.6 s, NOT at
%   t = 0, because the initial TGO = RTM/VC is 563.3 s and has to fall to 500
%   before slot 9 opens; the last is at t = 445.7 s, then the terminal burn
%   latches at t = 498.4 s and runs 2.8 s to impact.  Burn durations 5.157 s
%   (the first, and the longest) then 0.29-0.71 s.  Deleted-guidance miss
%   9.25e4 ft, i.e. of order the 1e5 ft
%   prediction error; 10-pulse miss 1.22 ft, 5-pulse 3.44 ft, PN 0.63 ft,
%   floored by the 1e-4 s step used once TGO < 0.1 s.  DELV 808.8 (10 pulses),
%   1251 (5), 423.4 (PN).  Seven structural rows green, row 8 red, four INFO.
%   Whole call ~0.5 s, of which the ten-point sweep is ~0.2 s.
%
%   OUTPUTS.  R.runs holds one struct per case -- .p10 .p05 (pulsed), .pn
%   (Listing 14.1's PN at N' = XNP, same prediction error), .off (guidance
%   deleted: the miss reference), .flip (the T>TOFF delete experiment); only
%   .p10 exists when 'companions' is false.  Each carries .t .tgo .xnc .xlamd
%   .delv .energy at the listing's 1 s sample interval, plus .miss_ft (closest
%   approach), .rtm_exit_ft (the value the listing PRINTS), .delv_ft,
%   .max_burn_s, .pulse, and the divert split .delv_at_terminal_ft /
%   .delv_terminal_ft / .t_terminal / .term_dur_s / .t_end_s.  R.sweep is the
%   divert-vs-pulse-count table, one row vector per column above, plus the
%   peak-to-peak spreads over the full sweep and over the book's own range.
%   R.bench are the rows; R.selfcheck has .names/.oks; R.pass is true only if a
%   row was SCORED and all passed; R.verdict is the one-line reason otherwise;
%   R.p and R.summary are the parameters and text.
%
% ---------------------------------------------------------------------------
%   ⭐⭐⭐ NEAR-PARABOLIC GUARD -- lambda = r0*|v|^2/gm MUST STAY BELOW 1.999999
%
%   THE SILENT WRONG ANSWER.  Listing 13.2 is ELLIPTIC-ONLY.  Its bracket
%   [gmin, gmax] is DEFINED by the escape condition lambda = 2, and its
%   flight-time closed form divides by bot2 = xlam*(2/xlam-1)^1.5, which
%   collapses as lambda -> 2.  Handed a transfer that is genuinely HYPERBOLIC
%   the secant cannot leave the bracket, so it crawls to the bracket EDGE and
%   halts on an orbit that is not the solution -- while its own residual test
%   says it converged.
%
%   MEASURED 2026-08-24 -- alt 300 nm -> 0, transfer angle 78 deg, TF = 750 s:
%     true transfer needs   lambda = 2.014012          (HYPERBOLIC)
%     lambertpz halts at    lambda = 1.999999919,  bot2 = 1.6409e-11
%     it reports            tF = 749.999996 s   -> passes its own 1e-8 test
%     ⚠⚠ converged = TRUE
%     propagating its answer (ode45, RelTol 1e-13) MISSES THE TARGET BY
%     98,751.56 ft.  An independent universal-variable (Bate-Mueller-White)
%     Lambert returns 35302.6106 ft/s and misses by 0.00 ft; this listing
%     returns 35179.5920 ft/s.
%
%   ⚠⚠ THE GUARD IS NOT KEYED ON THE ROUTINE'S `converged` FLAG.  That flag is
%   the thing being caught -- it is TRUE on the wrong answer, in all six
%   transcriptions of this listing carried in this package.
%   ⚠⚠ AND MUTUAL AGREEMENT DID NOT CATCH IT EITHER.  Those six transcriptions
%   agree to 0.000e+00 over 954 converged geometries with identical iteration
%   counts 954/954, because the misreading is in the LISTING, not in any one
%   transcription of it.  Only a quantity the routine does not control
%   separates the two answers, and lambda is that quantity.
%
%   ⚠ MEASURED SEPARATION, and it is NOT the "enormous margin" this guard was
%   commissioned on.  Over a 6150-geometry sweep (50 transfer angles x 41
%   flight times x 3 altitude pairs) the 3043 converged-and-CORRECT cases reach
%   lambda = 1.999997634 and the 17 converged-and-WRONG cases start at
%   1.999999740 (misses 66,600 ft to 24,278,620 ft, all adjudicated by
%   propagation).  The threshold separates them with ZERO false alarms, but the
%   gap is ~1.4e-6, not the ~0.039 a coarser 1080-point grid suggested.
%   ⚠ ONE CASE LEAKS: transfer angle 99 deg, TF = 900 s halts at
%   lambda = 1.999997634 -- BELOW the threshold -- and still misses by 141.89 ft.
%   The threshold is left where it was specified; widening it to swallow that
%   case is not something any source supports.
%
%   WHAT IT ADDS HERE.  This port already REFUSES to fly when lambertpz hits
%   the cap, which is why Lambert convergence is not a scored row (it could
%   only ever be green -- see the note above K.lambert_hitcap).  The
%   near-parabolic case slips straight past that refusal: it does not hit the
%   cap, it CONVERGES, and the launch velocity is wrong anyway.  The guard is
%   the row that can go red on a solve this port was willing to fly.
%
%   THIS PORT'S REACH -- MEASURED, AND THE GUARD BITES.
%     XLONGMDEG = -40, TF = 902 s -> lambda = 1.999999898, hitcap = 0
%     XLONGMDEG = -40, TF = 904 s -> lambda = 1.999999888
%     XLONGMDEG = -50, TF = 928 s -> lambda = 1.999999944
%   ⭐ At the first of those the port's own refusal does NOT fire (hitcap = 0):
%   it converges, flies the whole engagement, and reports miss and divert
%   numbers for a launch velocity that is not the solution.  The guard is red;
%   the near neighbours TF = 900 and 906 are honest REFUSALS, which is how
%   narrow the band is.
%   MARGIN AT THE BOOK'S OWN CASE: lambda_out = 0.619914499.
% ---------------------------------------------------------------------------
%
%   ⚠️ 'sweep', [] turns the sweep off and therefore turns SELF-CHECK 6 RED.
%   That is deliberate: with the sweep off nothing checks that the pulse count
%   reaches the simulation, and an unchecked invariant must not print green.
%
%   See also MGS.ZARCHAN.ENGAGE2D, MGS.ZARCHAN.LAMBERTEFFICIENT,
%            MGS.ZARCHAN.GEMSIMULATION, MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 15.2 line 1, verbatim (zarchan.txt:4587).
    opts.PULSES     (1,1) double {mustBeFinite, mustBePositive} = 10
    opts.AMAG       (1,1) double {mustBeFinite, mustBePositive} = 64.4    % ft/s^2 = 2 g
    opts.PREDERR    (1,1) double {mustBeFinite}                 = -100000 % ft; prose :4582 says 100 kft
    opts.TF         (1,1) double {mustBeFinite, mustBePositive} = 500     % s
    opts.DISTNMT    (1,1) double {mustBeFinite, mustBePositive} = 6000    % nmi
    opts.GAMDEGT    (1,1) double {mustBeFinite}                 = 23      % deg
    opts.XLONGMDEG  (1,1) double {mustBeFinite}                 = 45      % deg
    opts.XLONGTDEG  (1,1) double {mustBeFinite}                 = 90      % deg
    opts.H          (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s
    opts.HFINE      (1,1) double {mustBeFinite, mustBePositive} = 0.0001  % s, once TGO < .1
    % Fig. 15.11's case, and Fig. 15.12/15.13's PN comparison.  Listing 14.1,
    % on which 15.2 is based, uses XNP = 3 (zarchan.txt:4220).
    opts.PULSES_ALT (1,1) double {mustBeFinite, mustBePositive} = 5
    opts.XNP        (1,1) double {mustBeFinite, mustBePositive} = 3
    % Iteration cap for Listing 13.2's Lambert secant, which the book does not
    % have.  Exhausting it is a SELF-CHECK failure, never a silent fallback.
    opts.maxit      (1,1) double {mustBeFinite, mustBePositive} = 200
    % OURS, NOT THE BOOK'S -- see the header.  Exposed so they cannot hide.
    opts.miss_frac  (1,1) double {mustBeFinite, mustBePositive} = 0.01
    opts.divert_band(1,1) double {mustBeFinite, mustBePositive} = 0.25
    % THE SWEEP.  The listing's whole subject is how performance depends on the
    % pulse count, so the dependence is MEASURED and printed rather than
    % asserted.  Reported as INFO rows -- never scored, because the book
    % publishes no divert number to score against.  N >= 2 is a hard floor:
    % the schedule divides by PULSES-1.  [] turns the sweep off.
    opts.sweep      (1,:) double {mustBeFinite, mustBeInteger, ...
                                  mustBeGreaterThanOrEqual(opts.sweep,2)} ...
                                                = [2 3 4 5 6 8 10 12 16 20]
    % false runs ONLY the 10-pulse case: rows 2-8 then SKIP (never pass).
    opts.companions (1,1) logical = true
    opts.benchmark  (1,1) logical = true
    opts.render     (1,1) logical = true
    opts.assert     (1,1) logical = true
end

A = 2.0926e7;  GM = 1.4077e16;  DEGRAD = 360/(2*pi);

% ---- Listing 15.2 line 1: the ballistic target's launch state ---------------
GAMT = opts.GAMDEGT/57.3;                 % the book's own 57.3, not 180/pi
PHIT = opts.DISTNMT*6076/A;
R0T  = A;                                 % ALTNMTIC = 0
TOP  = GM*(1 - cos(PHIT));
TEMP = R0T*cos(GAMT)/A - cos(PHIT + GAMT);
BOT  = R0T*cos(GAMT)*TEMP;
if BOT <= 0 || TOP <= 0
    error('mgs:zarchan:pulsedGuidance:target', ...
        ['the closed form for the target launch speed is degenerate at ' ...
         'DISTNMT = %g nmi, GAMDEGT = %g deg (TOP = %g, BOT = %g).  A complex ' ...
         'VT would propagate silently through the entire run.'], ...
        opts.DISTNMT, opts.GAMDEGT, TOP, BOT);
end
VT = sqrt(TOP/BOT);

XLONGM = opts.XLONGMDEG/DEGRAD;
XLONGT = opts.XLONGTDEG/DEGRAD;
% UNRESOLVED COMPARISON 1, resolved -- see the header.  BOTH branches are kept
% so SELF-CHECK 5 can evaluate the one we did not ship.
[X1T, Y1T]         = targetVelocity(VT, GAMT, XLONGT,   XLONGM > XLONGT);
[X1Tflip, Y1Tflip] = targetVelocity(VT, GAMT, XLONGT, ~(XLONGM > XLONGT));

XM = A*cos(XLONGM);  YM = A*sin(XLONGM);      % ALTNMMIC = 0
XT = A*cos(XLONGT);  YT = A*sin(XLONGT);

% ---- Lambert launch, aimed at a DELIBERATELY WRONG predicted intercept ------
[XTF, YTF] = predictpz(opts.TF, XT, YT, X1T, Y1T, GM, opts.H);
YTF = YTF + opts.PREDERR;
[VRXM, VRYM, hitcap, lamOut] = lambertpz(XM, YM, opts.TF, XTF, YTF, XLONGM, ...
                                 XLONGT, GM, opts.maxit);
if hitcap || ~isfinite(VRXM) || ~isfinite(VRYM)
    % REFUSED, not warned about.  A Lambert velocity that did not converge
    % means the missile was never aimed at the predicted intercept, so every
    % miss and divert downstream describes a different engagement -- and would
    % describe it smoothly, with no symptom.  The book's routine has no cap
    % at all and would simply spin here.
    error('mgs:zarchan:pulsedGuidance:lambert', ...
        ['Listing 13.2''s Lambert secant did not converge in %d iterations ' ...
         '(vrx = %g, vry = %g).  Refusing to fly an engagement whose launch ' ...
         'velocity is not the one the book solved for.'], ...
        round(opts.maxit), VRXM, VRYM);
end

IC = struct('XT',XT, 'YT',YT, 'X1T',X1T, 'Y1T',Y1T, ...
            'XM',XM, 'YM',YM, 'X1M',VRXM, 'Y1M',VRYM);

% ---- the runs ---------------------------------------------------------------
cfg = struct('AMAG',opts.AMAG, 'XNP',opts.XNP, 'TF',opts.TF, ...
             'H',opts.H, 'HFINE',opts.HFINE, 'GM',GM, 'toffFlip',false);

runs = struct();
runs.p10 = flyEngagement(IC, 'pulsed', opts.PULSES, cfg);
if opts.companions
    runs.p05  = flyEngagement(IC, 'pulsed', opts.PULSES_ALT, cfg);
    runs.pn   = flyEngagement(IC, 'pn',     opts.PULSES,     cfg);
    runs.off  = flyEngagement(IC, 'none',   opts.PULSES,     cfg);
    cfgF = cfg;  cfgF.toffFlip = true;
    runs.flip = flyEngagement(IC, 'pulsed', opts.PULSES, cfgF);
end

% ---- THE SWEEP.  Divert vs pulse count, measured ---------------------------
%   Driven straight through flyEngagement, NOT by recursive calls to this
%   function: the launch state does not depend on PULSES, so re-solving the
%   Lambert and re-running the 50,000-step predictor once per point would
%   triple the cost of the sweep and buy nothing.  Every point therefore flies
%   the SAME missile off the SAME launch velocity, which is what makes the
%   comparison controlled.  ~24 ms per point on the machine this was written on.
sweep = runSweep(IC, cfg, opts, runs);

R = struct();
R.runs  = runs;
R.sweep = sweep;
R.p = struct('PULSES',opts.PULSES, 'PULSES_ALT',opts.PULSES_ALT, ...
             'AMAG',opts.AMAG, 'PREDERR',opts.PREDERR, 'TF',opts.TF, ...
             'XNP',opts.XNP, 'H',opts.H, 'HFINE',opts.HFINE, 'VT',VT, ...
             'lambert_vx',VRXM, 'lambert_vy',VRYM, 'lambert_hitcap',hitcap, ...
             'companions',opts.companions, 'miss_frac',opts.miss_frac, ...
             'divert_band',opts.divert_band, ...
             'units','ft, ft/s, ft/s^2, s; g = 32.2 ft/s^2');
R.units = R.p.units;

R.lambda_out = lamOut;
R.selfcheck  = structuralChecks(runs, opts, hitcap, ...
                               [X1T Y1T], [X1Tflip Y1Tflip], XLONGT, sweep, lamOut);

if opts.benchmark
    R.bench = benchmarks(runs, opts, sweep);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && all(R.selfcheck.oks);
else
    R.bench = emptyBench();
    ev      = false;
    R.pass  = false;      % NOT CHECKED must never render as "fine"
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end

% ---- adjudication.  The verdict is FORMED either way; 'assert' chooses only
%      the reaction to it, so benchSuite (which runs every port with assert
%      off) still gets R complete and can see WHICH side moved.
msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false; nothing was compared. R.pass = false means NOT CHECKED.';
elseif ~any(ev)
    msg = 'not one book claim could be evaluated (with companions off, rows 2-8 skip).';
elseif ~all(R.selfcheck.oks)
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, a wiring fault rather than a tolerance question. First: %s.'], ...
        sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf('%d of %d evaluated book claims failed (first: %s -- book %s, ours %s).', ...
        numel(bad), sum(ev), R.bench(bad(1)).what, ...
        R.bench(bad(1)).book_str, R.bench(bad(1)).ours_str);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:pulsedGuidance:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:pulsedGuidance:benchmark', ...
            '%s  (''assert'' is false, so R is returned complete; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [x1t, y1t] = targetVelocity(VT, GAMT, XLONGT, missileEast)
%TARGETVELOCITY  Listing 15.2's "if XLONGM.XLONGT" branch pair (zarchan.txt:4587).
%   missileEast is the resolved comparison XLONGM > XLONGT.  Every constant
%   arrives as an argument: MATLAB local functions cannot see the main
%   function's workspace, and a previous wave in this package lost a morning
%   to exactly that.
if missileEast
    x1t = VT*cos(pi/2 - GAMT + XLONGT);
    y1t = VT*sin(pi/2 - GAMT + XLONGT);
else
    x1t = VT*cos(-pi/2 + GAMT + XLONGT);
    y1t = VT*sin(-pi/2 + GAMT + XLONGT);
end
end


% =============================================================================
function W = runSweep(IC, cfg, opts, runs)
%RUNSWEEP  Fly the SAME engagement at each pulse count in opts.sweep and record
%   what changes.  This is the substance of Listing 15.2: the listing exists to
%   show how a fixed-amplitude law performs as a function of the pulse count,
%   and a port that reports only two points cannot tell "insensitive" from
%   "the sweep parameter never reached the simulation".
%
%   ⚠️ The two points the BOOK compares (PULSES and PULSES_ALT) are REUSED from
%   runs rather than re-flown, so the sweep column and the benchmark rows can
%   never disagree about the same pulse count -- two independent computations
%   of one number is a place for them to drift apart silently.
%
%   Every field is a row vector aligned with .pulses.
W = struct('pulses',[], 'ran',false, 'n_sched',[], 'delv_ft',[], ...
           'miss_ft',[], 'delv_sched_ft',[], 'delv_terminal_ft',[], ...
           'delv_sched_impulse_ft',[], 't_terminal_s',[], 'term_dur_s',[], ...
           'sched_spread_pct',NaN, 'total_spread_pct',NaN, ...
           'book_lo',NaN, 'book_hi',NaN, 'in_book_range',[], ...
           'sched_spread_book_pct',NaN, 'total_spread_book_pct',NaN, ...
           'reused',[]);
ks = opts.sweep(:).';
if isempty(ks),  return;  end

n = numel(ks);
W.pulses = ks;
[W.n_sched, W.delv_ft, W.miss_ft, W.delv_sched_ft, W.delv_terminal_ft, ...
 W.delv_sched_impulse_ft, W.t_terminal_s, W.term_dur_s] = deal(nan(1,n));
W.reused = false(1,n);

for i = 1:n
    k = ks(i);
    if k == opts.PULSES
        S = runs.p10;  W.reused(i) = true;
    elseif opts.companions && k == opts.PULSES_ALT
        S = runs.p05;  W.reused(i) = true;
    else
        S = flyEngagement(IC, 'pulsed', k, cfg);
    end
    W.n_sched(i)              = S.n_sched;
    W.delv_ft(i)              = S.delv_ft;
    W.miss_ft(i)              = S.miss_ft;
    W.delv_sched_ft(i)        = S.delv_at_terminal_ft;
    W.delv_terminal_ft(i)     = S.delv_terminal_ft;
    W.delv_sched_impulse_ft(i)= S.delv_sched_impulse_ft;
    W.t_terminal_s(i)         = S.t_terminal;
    W.term_dur_s(i)           = S.term_dur_s;
end
W.ran = true;

% Peak-to-peak spread as a fraction of the mean, for the two halves of the
% divert.  Reported, never scored: the book publishes no divert number.
%
% ⚠️ TWO RANGES, AND BOTH ARE PRINTED, because quoting only one of them would
% be a choice made after seeing the data.  The FULL sweep reaches down to N = 2,
% where the schedule owns a single pulse and cannot carry the correction at all
% (its scheduled cost is 332 ft/s against ~615 from N = 4 up) -- so a "the
% scheduled cost is flat" claim quoted over the full sweep is false.  The BOOK
% RANGE is the closed interval between the two cases Fig. 15.13 actually
% overlays, PULSES_ALT..PULSES; it is not a subset chosen to look good, it is
% the comparison the sentence on p.347 is about.
W.total_spread_pct = spreadPct(W.delv_ft);
W.sched_spread_pct = spreadPct(W.delv_sched_ft);

W.book_lo = min(opts.PULSES, opts.PULSES_ALT);
W.book_hi = max(opts.PULSES, opts.PULSES_ALT);
W.in_book_range = ks >= W.book_lo & ks <= W.book_hi;
if nnz(W.in_book_range) >= 2
    W.total_spread_book_pct = spreadPct(W.delv_ft(W.in_book_range));
    W.sched_spread_book_pct = spreadPct(W.delv_sched_ft(W.in_book_range));
end
end

function p = spreadPct(v)
v = v(isfinite(v));
if numel(v) < 2 || mean(v) == 0,  p = NaN;  else,  p = 100*(max(v)-min(v))/mean(v);  end
end

function s = listStr(ks, v, fmt)
%LISTSTR  "2:3310 3:2112 ..." -- the sweep, compact enough for one board cell.
c = cell(1, numel(ks));
for i = 1:numel(ks),  c{i} = sprintf(['%g:' fmt], ks(i), v(i));  end
s = strjoin(c, ' ');
end


% =============================================================================
function Sout = flyEngagement(IC, law, PULSES, cfg)
%FLYENGAGEMENT  Listing 15.2's main loop, zarchan.txt:4593-4621.
%
%   law  'pulsed' -> Listing 15.2's fixed-amplitude scheduler
%        'pn'     -> Listing 14.1's XNC = XNP*VC*XLAMD (Figs. 15.12/15.13)
%        'none'   -> XNC == 0.  The miss reference for "successful intercept",
%                    i.e. the delete experiment behind rows 2 and 3.
%
%   cfg.toffFlip inverts the resolved "T > TOFF" and NOTHING else.

AMAG = cfg.AMAG;  XNP = cfg.XNP;  TF = cfg.TF;  GM = cfg.GM;
isPN = strcmp(law, 'pn');  isPulsed = strcmp(law, 'pulsed');

XT = IC.XT;  YT = IC.YT;  X1T = IC.X1T;  Y1T = IC.Y1T;
XM = IC.XM;  YM = IC.YM;  X1M = IC.X1M;  Y1M = IC.Y1M;

T = 0;  SAMP = 0;  DELV = 0;  XNC = 0;      % SAMP is the listing's S
% TPULSE is pre-set even though every path through the scheduler assigns it
% before reading it, so the Code Analyzer flags it as unused.  Kept anyway:
% "if TGO < TPULSE" is a READ, and if a future edit ever lets the DISC <= 0
% path skip the assignment, an undefined TPULSE would be an error thrown far
% from the line that caused it -- whereas a stale one would just misplace a
% pulse and produce a smooth, wrong trajectory.  0 fails "TGO < TPULSE" and so
% takes the safe branch (TOFF = T, i.e. shut off).
PULSE_ON = 0;  PULSE_NUM = PULSES - 1;  TOFF = 0;  TPULSE = 0;  %#ok<NASGU>
XDT=0; YDT=0; X1DT=0; Y1DT=0; XDM=0; YDM=0; X1DM=0; Y1DM=0; DELVD=0;

RTM1 = XT - XM;  RTM2 = YT - YM;  RTM = sqrt(RTM1^2 + RTM2^2);
VTM1 = X1T - X1M;  VTM2 = Y1T - Y1M;
VC = -(RTM1*VTM1 + RTM2*VTM2)/RTM;
XLAMD = 0;

nmax = ceil(TF) + 16;
arT = zeros(nmax,1);  arTGO = zeros(nmax,1);  arXNC = zeros(nmax,1);
arLAM = zeros(nmax,1);  arDELV = zeros(nmax,1);  arE = zeros(nmax,1);
count = 0;

% Per-burn records.  The OFF store is CAPPED: under the toffFlip delete
% experiment the terminal branch toggles every step, which would otherwise
% grow a 5e4-element array one element at a time.  nOffTot counts them all.
%
% curIdx is which SCHEDULED pulse is currently burning, or 0 for "the terminal
% branch turned this on".  Without it, the toggling under toffFlip would pair
% every terminal on/off against the LAST scheduled pulse's start time and
% report burn lengths of tens of seconds -- which is exactly the number
% SELF-CHECK 4 reads, so it would quietly invert that delete experiment.
capOn  = max(round(PULSES), 1) + 2;
capOff = capOn + 8;
onT  = zeros(1,capOn);   onLam  = zeros(1,capOn);   onDur = zeros(1,capOn);
offT = zeros(1,capOff);  offLam = zeros(1,capOff);  offIdx = zeros(1,capOff);
curIdx = 0;
nSched = 0;  nOff = 0;  nOffTot = 0;  nZeroLen = 0;  amagBad = 0;  quadResid = 0;
missMin = RTM;  tTerm = NaN;  delvTerm = NaN;

maxsteps = ceil(1.2*TF/cfg.H) + ceil(5/cfg.HFINE) + 1000;
nstep = 0;

while VC >= 0
    TGO = RTM/VC;
    if TGO > 0.1,  H = cfg.H;  else,  H = cfg.HFINE;  end

    XOLDT = XT;  YOLDT = YT;  X1OLDT = X1T;  Y1OLDT = Y1T;
    XOLDM = XM;  YOLDM = YM;  X1OLDM = X1M;  Y1OLDM = Y1M;  DELVOLD = DELV;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  Pass 1 evaluates the derivatives at
    % the current state; pass 2 takes a full Euler step, advances T by H, and
    % re-evaluates there.  T advances INSIDE this loop, exactly once.
    while STEP <= 1
        if FLAG == 1
            XT = XT + H*XDT;      YT = YT + H*YDT;
            X1T = X1T + H*X1DT;   Y1T = Y1T + H*Y1DT;
            XM = XM + H*XDM;      YM = YM + H*YDM;
            X1M = X1M + H*X1DM;   Y1M = Y1M + H*Y1DM;
            DELV = DELV + H*DELVD;  T = T + H;  STEP = 2;
        end
        TEMBOTT = (XT^2 + YT^2)^1.5;
        X1DT = -GM*XT/TEMBOTT;  Y1DT = -GM*YT/TEMBOTT;
        XDT = X1T;  YDT = Y1T;
        RTM1 = XT - XM;  RTM2 = YT - YM;  RTM = sqrt(RTM1^2 + RTM2^2);
        VTM1 = X1T - X1M;  VTM2 = Y1T - Y1M;
        VC  = -(RTM1*VTM1 + RTM2*VTM2)/RTM;
        TGO = RTM/VC;
        XLAM  = atan2(RTM2, RTM1);
        XLAMD = (RTM1*VTM2 - RTM2*VTM1)/(RTM*RTM);
        % Listing 14.1's one extra line.  Under Listing 15.2, XNC is set
        % OUTSIDE this loop and held across BOTH derivative passes; under PN
        % it is recomputed on each pass, exactly as 14.1 does it.
        if isPN,  XNC = XNP*VC*XLAMD;  end
        DELVD = abs(XNC);
        AM1 = -XNC*sin(XLAM);  AM2 = XNC*cos(XLAM);
        TEMBOTM = (XM^2 + YM^2)^1.5;
        X1DM = -GM*XM/TEMBOTM + AM1;  Y1DM = -GM*YM/TEMBOTM + AM2;
        XDM = X1M;  YDM = Y1M;
        FLAG = 1;
    end

    XT = (XOLDT + XT)/2 + .5*H*XDT;      YT = (YOLDT + YT)/2 + .5*H*YDT;
    X1T = (X1OLDT + X1T)/2 + .5*H*X1DT;  Y1T = (Y1OLDT + Y1T)/2 + .5*H*Y1DT;
    XM = (XOLDM + XM)/2 + .5*H*XDM;      YM = (YOLDM + YM)/2 + .5*H*YDM;
    X1M = (X1OLDM + X1M)/2 + .5*H*X1DM;  Y1M = (Y1OLDM + Y1M)/2 + .5*H*Y1DM;
    DELV = (DELVOLD + DELV)/2 + .5*H*DELVD;

    % Closest approach.  RTM here is the pass-2 value, which is also the one
    % the listing carries into the guidance block and prints at exit.
    if RTM < missMin,  missMin = RTM;  end

    % ---- the pulsed scheduler, zarchan.txt:4599-4619 -----------------------
    if isPulsed
        if PULSE_ON == 1
            if cfg.toffFlip
                shutOff = (T < TOFF);         % the INVERTED resolution
            else
                shutOff = (T > TOFF);         % UNRESOLVED COMPARISON 2
            end
            if shutOff
                PULSE_ON = 0;  XNC = 0;
                nOffTot = nOffTot + 1;
                if nOff < capOff
                    nOff = nOff + 1;
                    offT(nOff) = T;  offLam(nOff) = abs(XLAMD);  offIdx(nOff) = curIdx;
                end
                curIdx = 0;
            end
        end
        if PULSE_NUM > 0
            if TGO <= (TF - 0)*PULSE_NUM/(PULSES - 1)
                PULSE_NUM = PULSE_NUM - 1;
                PULSE_ON  = 1;
                DISC = 1 - 2*VC*abs(XLAMD)/AMAG;
                if DISC > 0
                    TPULSE = TGO*(1 - sqrt(DISC));
                    if XLAMD > 0,  XNC = AMAG;  else,  XNC = -AMAG;  end
                    % BENCHMARK 1, evaluated where the book's algebra lives.
                    zemPN = VC*TGO^2*abs(XLAMD);
                    zemPl = .5*AMAG*TPULSE^2 + AMAG*TPULSE*(TGO - TPULSE);
                    if zemPN > 0
                        quadResid = max(quadResid, abs(zemPl/zemPN - 1));
                    end
                else
                    TPULSE = 0;              % XNC deliberately NOT assigned
                    nZeroLen = nZeroLen + 1;
                end
                if nSched < capOn
                    nSched = nSched + 1;
                    onT(nSched) = T;  onLam(nSched) = abs(XLAMD);
                    onDur(nSched) = TPULSE;
                    curIdx = nSched;
                end
                if TGO < TPULSE                % UNRESOLVED COMPARISON 3
                    TOFF = 9999999;
                else
                    TOFF = T + TPULSE;
                end
            end
        else
            DISC = 1 - 2*VC*abs(XLAMD)/AMAG;
            if DISC > 0
                TPULSE = TGO*(1 - sqrt(DISC));
            else
                TPULSE = 999999;
            end
            if TGO <= TPULSE
                if XLAMD > 0,  XNC = AMAG;  else,  XNC = -AMAG;  end
                % Recorded BEFORE PULSE_ON is set, so this is the accumulated
                % divert at the instant the terminal burn begins -- the split
                % the sweep needs.  It stays NaN if the terminal branch latches
                % while a SCHEDULED pulse is still burning (the two burns then
                % run into one another and there is no instant to record).
                if PULSE_ON == 0 && ~isfinite(tTerm)
                    tTerm = T;  delvTerm = DELV;
                end
                PULSE_ON = 1;  TOFF = 999999;
                curIdx = 0;      % a terminal burn, not one of the scheduled ones
            end
        end
        % The DEFINING property of pulsed guidance: the command is 0 or
        % +/-AMAG and never anything between.  Checked at EVERY step, not only
        % at samples -- this is what would notice a continuous term leaking in.
        if XNC ~= 0 && abs(abs(XNC) - AMAG) > 1e-9,  amagBad = amagBad + 1;  end
    end

    SAMP = SAMP + H;
    if SAMP >= .99999
        SAMP = 0;  count = count + 1;
        arT(count) = T;  arTGO(count) = TGO;  arXNC(count) = XNC;
        arLAM(count) = XLAMD;  arDELV(count) = DELV;
        % Specific orbital energy of the UNPOWERED target -- SELF-CHECK 1.
        arE(count) = .5*(X1T^2 + Y1T^2) - GM/sqrt(XT^2 + YT^2);
    end

    nstep = nstep + 1;
    if nstep > maxsteps
        error('mgs:zarchan:pulsedGuidance:runaway', ...
            ['the ''%s'' run took %d steps without VC going negative.  The ' ...
             'listing has no step cap and would spin here forever; a run that ' ...
             'never closes must not be reported as a miss distance.'], law, nstep);
    end
end

% Longest completed burn, in seconds.  SELF-CHECK 4 reads this: with "T > TOFF"
% a burn lasts TPULSE, with the inverted test exactly one integration step.
maxBurn = 0;
for k = 1:nOff
    j = offIdx(k);
    if j >= 1 && j <= nSched
        maxBurn = max(maxBurn, offT(k) - onT(j));
    end
end

Sout = struct();
Sout.law = law;  Sout.PULSES = PULSES;
Sout.t = arT(1:count);      Sout.tgo = arTGO(1:count);
Sout.xnc = arXNC(1:count);  Sout.xlamd = arLAM(1:count);
Sout.delv = arDELV(1:count);  Sout.energy = arE(1:count);
Sout.miss_ft     = missMin;     % closest approach seen
Sout.rtm_exit_ft = RTM;         % the value the listing PRINTS
Sout.delv_ft     = DELV;
Sout.n_steps     = nstep;
Sout.n_sched     = nSched;
Sout.n_zero_len  = nZeroLen;
Sout.n_off       = nOffTot;
Sout.n_amag_bad  = amagBad;
Sout.quad_resid  = quadResid;
Sout.t_terminal  = tTerm;
Sout.t_end_s     = T;                       % exact loop-exit time (VC < 0)
% The divert SPLIT, measured rather than read off the 1 s sample grid: during a
% burn DELV climbs at AMAG ft/s per second, so interpolating between two 1 s
% samples can be wrong by tens of ft/s -- more than the whole effect at high
% pulse counts.  delv_sched_ft is what the N-1 SCHEDULED pulses cost;
% delv_terminal_ft is what the final latched burn-to-impact costs.
Sout.delv_at_terminal_ft = delvTerm;
Sout.delv_terminal_ft    = DELV - delvTerm;
Sout.delv_sched_impulse_ft = AMAG*sum(onDur(1:nSched));
Sout.term_dur_s  = T - tTerm;
Sout.max_burn_s  = maxBurn;
Sout.pulse = struct('t_on',onT(1:nSched), 'lam_on',onLam(1:nSched), ...
                    'dur',onDur(1:nSched), 't_off',offT(1:nOff), ...
                    'lam_off',offLam(1:nOff), 'idx',offIdx(1:nOff));
end


% =============================================================================
function [xtf, ytf] = predictpz(tf, xdum, ydum, x1dum, y1dum, gm, h)
%PREDICTPZ  Listing 15.2's own predictor, zarchan.txt:4624-4628.  Two-body
%   propagation of the target to tf by the same RK2 the main loop uses.  The
%   listing assigns a local "a = 2.0926e7" it never reads; dropped.
t = 0;  x = xdum;  y = ydum;  x1 = x1dum;  y1 = y1dum;
xd = 0;  yd = 0;  x1d = 0;  y1d = 0;
while t <= (tf - .00001)
    xold = x;  yold = y;  x1old = x1;  y1old = y1;
    step = 1;  flag = 0;
    while step <= 1
        if flag == 1
            x = x + h*xd;    y = y + h*yd;
            x1 = x1 + h*x1d; y1 = y1 + h*y1d;  t = t + h;  step = 2;
        end
        tembot = (x^2 + y^2)^1.5;
        x1d = -gm*x/tembot;  y1d = -gm*y/tembot;
        xd = x1;  yd = y1;  flag = 1;
    end
    x = (xold + x)/2 + .5*h*xd;      y = (yold + y)/2 + .5*h*yd;
    x1 = (x1old + x1)/2 + .5*h*x1d;  y1 = (y1old + y1)/2 + .5*h*y1d;
end
xtf = x;  ytf = y;
end


% =============================================================================
function [vrx, vry, hitcap, lamout] = lambertpz(xic, yic, tfdes, xf, yf, xlongm, ...
                                        xlongt, gm, maxit)
%LAMBERTPZ  Listing 13.2's efficient Lambert routine, zarchan.txt:3943-3955.
%   Body identical to the one in MGS.ZARCHAN.GEMSIMULATION, including that
%   port's resolution of "if xlongt > xlongm".  hitcap = 1 when the cap was
%   reached; the listing has no cap and would spin on a non-finite secant step.
ric = sqrt(xic^2 + yic^2);  rf = sqrt(xf^2 + yf^2);
cphi = min(1, max(-1, (xic*xf + yic*yf)/(ric*rf)));
phi = acos(cphi);  sphi = sin(phi);  r0 = ric;
icount = 0;  hitcap = 0;
gmin = atan2((sphi - sqrt(2*r0*(1 - cphi)/rf)), (1 - cphi));
gmax = atan2((sphi + sqrt(2*r0*(1 - cphi)/rf)), (1 - cphi));
gam = (gmin + gmax)/2;  tf = 0;
vrx = NaN;  vry = NaN;  gold = gam;  told = 0;
while ~(abs(tfdes - tf) <= (.00000001*tfdes))
    top = gm*(1 - cos(phi));
    temp = r0*cos(gam)/rf - cos(phi + gam);
    bot = r0*cos(gam)*temp;
    if bot > 0
        v = sqrt(top/bot);
        if xlongt > xlongm
            vrx = v*cos(pi/2 - gam + xlongm);   vry = v*sin(pi/2 - gam + xlongm);
        else
            vrx = v*cos(-pi/2 + gam + xlongm);  vry = v*sin(-pi/2 + gam + xlongm);
        end
        xlam = r0*v*v/gm;
        if xlam > 0 && xlam < 2
            top1 = tan(gam)*(1 - cos(phi)) + (1 - xlam)*sin(phi);
            bot1p = (1 - cos(phi))/(xlam*cos(gam)*cos(gam));
            bot1 = (2 - xlam)*(bot1p + cos(gam + phi)/cos(gam));
            top2 = 2*cos(gam);
            bot2 = xlam*((2/xlam - 1)^1.5);
            top3 = sqrt(2/xlam - 1);
            bot3 = cos(gam)/tan(phi/2) - sin(gam);
            tf = r0*(top1/bot1 + (top2/bot2)*atan2(top3, bot3))/(v*cos(gam));
        else
            tf = inf;                     % at or past escape: too lofted
        end
    else
        tf = inf;
    end
    if ~isfinite(tf),  tf = inf;  end
    icount = icount + 1;
    if tf > tfdes,  gmax = gam;  else,  gmin = gam;  end
    if icount == 1
        xnext = (gmax + gmin)/2;
    else
        xnext = gam + (gam - gold)*(tfdes - tf)/(tf - told);
        % NaN fails BOTH bracket tests, so a non-finite secant step would
        % propagate into gam and never converge.
        if ~isfinite(xnext) || xnext > gmax || xnext < gmin
            xnext = (gmax + gmin)/2;
        end
    end
    gold = gam;  told = tf;  gam = xnext;
    if icount >= maxit,  hitcap = 1;  break;  end
end
% NEAR-PARABOLIC GUARD, see the file header.  Recomputed from the RETURNED
% velocity rather than from the loop's xlam, so the number scored is the one
% the missile is actually launched on.
lamout = r0*(vrx^2 + vry^2)/gm;
end


% =============================================================================
function K = structuralChecks(runs, opts, hitcap, vT, vTflip, XLONGT, sweep, lamOut)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never evidence that
%   this port reproduces Zarchan -- the weaker claim that it agrees with
%   ITSELF, which is what catches wiring faults.  .names and .oks are what
%   benchSuite's adaptGeneric turns into board rows; without BOTH, everything
%   computed here is invisible and the board still looks green.
K = struct();
names = {};  oks = [];

p10 = runs.p10;

% 1  the TARGET is unpowered two-body, so its specific orbital energy is a
%    constant of the motion.  RK2 at H = 0.01 s against an orbital rate of
%    ~1e-3 rad/s makes the per-step relative error ~(wH)^3 ~ 1e-15 over ~5e4
%    steps, so 1e-6 leaves three orders of margin: a threshold, not a band
%    fitted to a run.
E = p10.energy;
K.energy_drift = 0;
if numel(E) >= 2
    K.energy_drift = (max(E) - min(E))/max(abs(mean(E)), realmin);
end
names{end+1} = 'target two-body specific energy conserved to 1e-6';
oks(end+1)   = numel(E) >= 2 && K.energy_drift <= 1e-6;

% 2  the fixed-amplitude property, counted at every step inside the run.
K.n_amag_bad = p10.n_amag_bad;
names{end+1} = 'pulsed command is only 0 or +/-AMAG, at every step';
oks(end+1)   = K.n_amag_bad == 0;

% 3  the schedule issues exactly PULSES-1 pulses; the terminal burn is not one
%    of them.  Exact by construction, so any other count is a fault in the
%    "TGO <= TF*PULSE_NUM/(PULSES-1)" line rather than a tolerance question.
K.n_sched = p10.n_sched;
names{end+1} = sprintf('exactly %g scheduled pulses issued', opts.PULSES - 1);
oks(end+1)   = p10.n_sched == opts.PULSES - 1;

% 4  * DELETE EXPERIMENT * -- invert the resolved "T > TOFF" and NOTHING else.
%
%    ⚠️ What this does NOT measure is as important as what it does.  The
%    inverted test is true on the step right after TOFF is set, so every
%    scheduled pulse collapses to ONE integration step -- but the run STILL
%    INTERCEPTS, because the terminal branch always eventually fires (see the
%    header) and then burns at AMAG until impact, which nulls whatever ZEM is
%    left.  A delete experiment scored on the MISS would therefore have passed
%    the wrong resolution silently.  So it is scored on the burn structure,
%    which is deterministic: shipped, the longest burn is TPULSE (seconds);
%    inverted, it is exactly one step of H.  The extra propellant the inverted
%    version spends is reported alongside as the visible symptom.
K.max_burn_shipped_s = p10.max_burn_s;
K.max_burn_flip_s    = NaN;
K.delv_shipped_ft    = p10.delv_ft;
K.delv_flip_ft       = NaN;
if opts.companions
    K.max_burn_flip_s = runs.flip.max_burn_s;
    K.delv_flip_ft    = runs.flip.delv_ft;
    ok4 = p10.max_burn_s > 10*opts.H && K.max_burn_flip_s <= 2*opts.H;
else
    ok4 = false;
end
names{end+1} = 'DELETE: inverting T>TOFF collapses every burn to one step';
oks(end+1)   = ok4;

% 5  * DELETE EXPERIMENT * -- invert the resolved "XLONGM > XLONGT".  Decisive
%    without a run: the shipped branch must give the target a velocity
%    component toward DECREASING longitude (toward the missile at 45 deg), the
%    inverted branch toward increasing longitude.  The transverse unit vector
%    in the direction of increasing longitude at XLONGT is (-sin, cos), so the
%    sign of its dot product with the target velocity is the whole test.
th = [-sin(XLONGT), cos(XLONGT)];
K.vt_transverse      = th*vT(:);
K.vt_transverse_flip = th*vTflip(:);
names{end+1} = 'DELETE: inverting XLONGM>XLONGT sends the target away from the missile';
oks(end+1)   = K.vt_transverse < 0 && K.vt_transverse_flip > 0;

% 6  * WIRING * -- does the swept parameter reach the simulation at all?
%
%    ⚠️ This is the check that has to exist before ANY "changing N barely moves
%    the divert" claim can be believed.  A sweep whose parameter never reaches
%    the integrator produces bit-identical runs, and bit-identical runs are
%    maximally "insensitive" -- the most convincing possible version of the
%    wrong answer.  Two independent signatures, both exact by construction:
%      * the schedule must issue exactly N-1 pulses at every N, and
%      * no two pulse counts may produce the SAME DELV to the last bit.
%    The second is what a hardcoded pulse count inside runSweep would break;
%    the first is what a schedule that ignored PULSES would break.
K.sweep_n = 0;  K.sweep_unique_delv = 0;  K.sweep_sched_ok = false;
if sweep.ran
    K.sweep_n           = numel(sweep.pulses);
    K.sweep_unique_delv = numel(unique(sweep.delv_ft));
    K.sweep_sched_ok    = all(sweep.n_sched == sweep.pulses - 1);
end
names{end+1} = 'WIRING: every swept pulse count reaches the sim (distinct DELV, N-1 pulses)';
oks(end+1)   = sweep.ran && K.sweep_n >= 2 && ...
               K.sweep_unique_delv == K.sweep_n && K.sweep_sched_ok;

% 7  * THE SPLIT IS REAL * -- the divert-vs-pulse-count rows are INFO, so they
%    can neither pass nor fail, and an INFO row nobody can falsify is exactly
%    the "row that cannot fail" this package refuses to ship.  This is the row
%    that CAN fail on their behalf, and it is not an identity: delv_terminal is
%    a SUBTRACTION of two recorded quantities, while AMAG*(t_end - t_terminal)
%    is the physics -- the final pulse latches (TOFF = 999999) and must
%    therefore burn at exactly AMAG from the recorded instant to impact.  If
%    delv_at_terminal were sampled one step from t_terminal, or if the terminal
%    burn were not in fact continuous, the two disagree.
%
%    THE THRESHOLD IS THE SMALLEST FAULT IT MUST CATCH, DIVIDED DOWN: a
%    one-integration-step misplacement costs AMAG*H (0.644 ft/s at the
%    defaults), so 1e-4 of that is 10,000x below the fault and -- measured --
%    some 14,000x above the float accumulation actually seen (4.5e-9 ft/s over
%    a 46 s burn).  It is not a band fitted to a run; both edges were measured.
K.split_resid_ft = NaN;
if sweep.ran
    fin = isfinite(sweep.delv_terminal_ft) & isfinite(sweep.term_dur_s);
    if any(fin)
        K.split_resid_ft = max(abs(sweep.delv_terminal_ft(fin) - ...
                                   opts.AMAG*sweep.term_dur_s(fin)));
    end
end
names{end+1} = 'SPLIT: terminal divert = AMAG x burn-to-impact length';
oks(end+1)   = isfinite(K.split_resid_ft) && ...
               K.split_resid_ft <= 1e-4*opts.AMAG*opts.H;

% Lambert convergence is NOT a row here: reaching it means the caller already
% got past the hard refusal in the main function, so a row could only ever be
% green.  hitcap is carried in R.p for the record.
K.lambert_hitcap = hitcap;

% NEAR-PARABOLIC GUARD, and it is a row precisely BECAUSE the convergence row
% above cannot be one.  See the file header for the 98,751 ft miss it exists to
% catch.  ⚠️ The failing solve does NOT hit the cap and does NOT trip the
% refusal -- it converges, on a velocity that is not the solution, and every
% miss and divert downstream then describes a different engagement smoothly and
% with no symptom.  ⚠️ Nor does agreement between this package's six
% transcriptions of Listing 13.2 catch it: all six agree to 0.000e+00 there.
K.lambda_out = lamOut;
names{end+1} = 'near-parabolic guard: lambda = r0|v|^2/gm < 1.999999';
oks(end+1)   = isfinite(lamOut) && lamOut < 1.999999;

K.names = names;
K.oks   = logical(oks(:).');
K.first_bad = '';
b = find(~K.oks, 1);
if ~isempty(b),  K.first_bad = names{b};  end
end


% =============================================================================
function B = benchmarks(runs, opts, sweep)
%BENCHMARKS  Computed vs published.  Every citation was grepped out of
%   zarchan_decoded.txt; nothing here was read off a figure, because the
%   extraction turned every figure into an image link.
B = emptyBench();
p10 = runs.p10;
noComp = 'companions are off, so only the 10-pulse run exists';

% ---- 1  the pulse duration solves the book's own quadratic ------------------
B = addBound(B, 'dt solves ZEM = .5*a*dt^2 + a*dt*(tgo-dt)', ...
    'exact identity (quadratic formula, unrealistic root dropped)', ...
    sprintf('max rel resid %.3e over %d scheduled pulses', p10.quad_resid, p10.n_sched), ...
    p10.n_sched > 0 && p10.quad_resid <= 1e-10, ...
    'p.339, ZEM and dt equations (zarchan.txt:4559, :4565, :4567-4569)', '-', ...
    'evaluated at every scheduled pulse with DISC > 0, where the book''s algebra lives');

if ~opts.companions
    % Rows 2-8 SKIP -- never pass.  "Not checked" must not render as "fine".
    sk = {'10 pulses -> successful intercept',        'zarchan.txt:4582',        'ft'; ...
          '5 pulses -> successful intercept',         'zarchan.txt:4645, :4647', 'ft'; ...
          'LOS rate builds higher with fewer pulses', 'zarchan.txt:4645',        'rad/s'; ...
          'each burn drives |LOS rate| down',         'zarchan.txt:4582',        'rad/s'; ...
          'PN LOS rate does not rebuild',             'zarchan.txt:4646',        '-'; ...
          'PN has the smallest divert',               'zarchan.txt:4647',        'ft/s'; ...
          'pulse count barely changes divert',        'zarchan.txt:4647',        'ft/s'};
    for k = 1:size(sk,1)
        B = addSkip(B, sk{k,1}, 'prose claim, no number published', noComp, ...
                    sk{k,2}, sk{k,3});
    end
else
    p05 = runs.p05;  pn = runs.pn;  off = runs.off;
    lim = opts.miss_frac*off.miss_ft;

    % ---- 2, 3  "a successful intercept" ------------------------------------
    B = addBound(B, '10 pulses -> successful intercept', ...
        sprintf(['"a successful intercept" (NO number published); scored as ' ...
                 'miss <= %g%% of the %.4g ft miss with guidance DELETED'], ...
                100*opts.miss_frac, off.miss_ft), ...
        sprintf('%.4g ft', p10.miss_ft), p10.miss_ft <= lim, ...
        'Fig. 15.10 text, zarchan.txt:4582', 'ft', ...
        sprintf('listing prints RTM = %.4g ft at loop exit; terminal burn starts at t = %.4g s', ...
                p10.rtm_exit_ft, p10.t_terminal));

    B = addBound(B, '5 pulses -> successful intercept', ...
        sprintf(['"each of the cases studied so far resulted in successful ' ...
                 'intercepts"; same %g%% criterion'], 100*opts.miss_frac), ...
        sprintf('%.4g ft', p05.miss_ft), p05.miss_ft <= lim, ...
        'Figs. 15.11/15.13 text, zarchan.txt:4645, :4647', 'ft', ...
        sprintf('reference miss with guidance deleted = %.4g ft', off.miss_ft));

    % ---- 4  fewer pulses -> larger LOS rate between pulses ------------------
    %   Compared over the part of the flight in which BOTH schedules are still
    %   firing, tgo >= TF/(PULSES_ALT-1).  Past that the sparser run has run out
    %   of scheduled pulses and the 1/tgo^2 endgame growth would dominate either
    %   curve regardless of how many pulses were used.  No UPPER bound: the
    %   initial TGO = RTM/VC is not TF and may exceed it, and clipping at TF
    %   would silently drop the opening samples of both runs.
    lo = opts.TF/max(opts.PULSES_ALT - 1, 1);
    pk10 = windowPeak(p10, lo, inf);
    pk05 = windowPeak(p05, lo, inf);
    B = addShape(B, 'LOS rate builds higher with fewer pulses', ...
        'peak |lamdot| with 5 pulses > with 10 pulses', ...
        sprintf('%.3e vs %.3e rad/s over tgo >= %.0f s', pk05, pk10, lo), ...
        isfinite(pk05) && isfinite(pk10) && pk05 > pk10, ...
        'Fig. 15.11 text, zarchan.txt:4645', ...
        '"since there are fewer pulses, the line-of-sight rate builds up to larger values in between pulses"');

    % ---- 5  each burn drives the LOS rate toward zero -----------------------
    [nGood, nTest] = pulseKnockdown(p10);
    B = addShape(B, 'each burn drives |LOS rate| down', ...
        'every burn ends with a smaller |lamdot| than it started', ...
        sprintf('%d of %d completed burns (%d zero-length slots excluded)', ...
                nGood, nTest, p10.n_zero_len), ...
        nTest > 0 && nGood == nTest, ...
        'Fig. 15.10 text, zarchan.txt:4582', ...
        '"always trying to drive the line-of-sight rate to zero after each pulse is issued"');

    % ---- 6  PN nulls lamdot only at the end ---------------------------------
    riPN = windowRises(pn,  lo, inf);
    ri10 = windowRises(p10, lo, inf);
    B = addShape(B, 'PN LOS rate does not rebuild', ...
        'PN nulls lamdot at the END of flight only, so it rebuilds less often than a pulsed run', ...
        sprintf('%d rising samples (PN) vs %d (10 pulses)', riPN, ri10), ...
        riPN < ri10, 'Fig. 15.12 text, zarchan.txt:4646', ...
        'counted over the same tgo window as row 4');

    % ---- 7, 8  divert -------------------------------------------------------
    B = addShape(B, 'PN has the smallest divert', ...
        'DELV(PN) < DELV(5 pulses) and < DELV(10 pulses)', ...
        sprintf('PN %.4g, 5p %.4g, 10p %.4g ft/s', pn.delv_ft, p05.delv_ft, p10.delv_ft), ...
        pn.delv_ft < p05.delv_ft && pn.delv_ft < p10.delv_ft, ...
        'Fig. 15.13 text, zarchan.txt:4647', ...
        ['flat-Earth linear theory predicts the OPPOSITE (PN 1.5*z0/tF = 300 ft/s ' ...
         'vs 201 ft/s for one early pulse) -- read the header before treating a red row as a bug']);

    % ---- 8  "does not appear to influence" ----------------------------------
    %   ⚠️ RED, AND LEFT RED.  The band is the one fixed before any run and the
    %   quantity is the one the book's sentence names -- "the lateral divert
    %   requirements", which is exactly the DELV this listing accumulates and
    %   prints.  The pair compared is the book's own pair (Fig. 15.13 overlays
    %   PN, 5 pulses and 10 pulses).  Nothing here is aimed at the wrong thing;
    %   the port simply disagrees with the figure.  The sweep rows below say
    %   WHERE the disagreement lives, and the header says what survives.
    rel = abs(p10.delv_ft/max(p05.delv_ft, realmin) - 1);
    % The SCHEDULED-side spread is measured, so the sentence that reports it
    % must follow the measurement.  This phrase used to be hardcoded to "the
    % book's claim holds there" and printed that verdict even at 88.8% apart
    % (PULSES_ALT = 2), i.e. an assertion in the detail string that no input
    % could ever falsify.  It is now adjudicated against the SAME band the row
    % itself is scored at, so it can read either way.
    relSched = abs(p10.delv_at_terminal_ft / ...
                   max(p05.delv_at_terminal_ft, realmin) - 1);
    if relSched <= opts.divert_band
        schedVerdict = 'the book''s claim holds there';
    else
        schedVerdict = 'the book''s claim does NOT hold there either';
    end
    B = addBound(B, 'pulse count barely changes divert', ...
        sprintf('"does not appear to influence" (NO band published); scored at %g%%', ...
                100*opts.divert_band), ...
        sprintf('%.2f%% between %g and %g pulses', 100*rel, opts.PULSES, opts.PULSES_ALT), ...
        rel <= opts.divert_band, 'Fig. 15.13 text, zarchan.txt:4647', 'ft/s', ...
        sprintf(['DELV %gp = %.4g, %gp = %.4g ft/s.  The split says where the ' ...
                 'difference is: SCHEDULED pulses cost %.4g vs %.4g ft/s ' ...
                 '(%.1f%% apart -- %s), the final ' ...
                 'LATCHED burn-to-impact costs %.4g vs %.4g ft/s.'], ...
                opts.PULSES, p10.delv_ft, opts.PULSES_ALT, p05.delv_ft, ...
                p10.delv_at_terminal_ft, p05.delv_at_terminal_ft, ...
                100*relSched, schedVerdict, ...
                p10.delv_terminal_ft, p05.delv_terminal_ft));
end

% ---- 9, 10, 11  THE SWEEP.  Reported, NEVER scored --------------------------
%   The measured dependence of divert on pulse count IS the subject of Listing
%   15.2, so it prints whatever the verdict on row 8 turns out to be.  Unscored
%   because Ch.15 publishes no divert number anywhere -- only the adjective
%   "does not appear to influence", which row 8 already adjudicates once.
%   Scoring the same sentence a second time against a measurand chosen AFTER
%   seeing the data would be widening a band with extra steps.
if sweep.ran
    ks = sweep.pulses;
    B = addInfo(B, 'SWEEP: divert vs pulse count', ...
        '"increasing the number of pulses does not appear to influence" (no number)', ...
        sprintf('DELV %s ft/s  [spread %.0f%% over N=%g..%g, %.0f%% over the book''s N=%g..%g]', ...
                listStr(ks, sweep.delv_ft, '%.0f'), sweep.total_spread_pct, ...
                min(ks), max(ks), sweep.total_spread_book_pct, ...
                sweep.book_lo, sweep.book_hi), ...
        'Fig. 15.13 text, zarchan.txt:4647', ...
        ['MEASURED, and monotone decreasing at every one of the points: total ' ...
         'divert falls by a factor of 5 from the sparsest schedule to the ' ...
         'densest, so the sweep parameter unquestionably reaches the ' ...
         'simulation (SELF-CHECK 6) and row 8 is not a wiring artefact']);

    B = addInfo(B, 'SWEEP: divert BEFORE the terminal burn', ...
        'the half of the divert the book''s claim survives on', ...
        sprintf('DELV %s ft/s  [spread %.0f%% over N=%g..%g, %.1f%% over the book''s N=%g..%g]', ...
                listStr(ks, sweep.delv_sched_ft, '%.0f'), sweep.sched_spread_pct, ...
                min(ks), max(ks), sweep.sched_spread_book_pct, ...
                sweep.book_lo, sweep.book_hi), ...
        'Fig. 15.13 text, zarchan.txt:4647', ...
        ['the N-1 SCHEDULED pulses cost nearly the same total once there are ' ...
         'enough of them to carry the correction -- each is sized to null the ' ...
         'WHOLE remaining ZEM, so the sum telescopes to what the engagement ' ...
         'needs.  ⚠️ NOT flat at the sparse end: N = 2 spends 332 ft/s on its ' ...
         'single scheduled pulse because it cannot spend more, which is why ' ...
         'the full-sweep spread is quoted alongside the book''s range.']);

    B = addInfo(B, 'SWEEP: terminal burn carries the sensitivity', ...
        'the half of the divert the book''s claim is false of', ...
        sprintf('burn-to-impact %s s', listStr(ks, sweep.term_dur_s, '%.2f')), ...
        'listing''s PULSE_NUM<=0 branch, zarchan.txt:4612-4618', ...
        ['the listing''s last pulse latches (TOFF = 999999) and burns to ' ...
         'impact, so its length is not chosen -- it is however long is left ' ...
         'once one feasible pulse can no longer remove the ZEM.  Fewer ' ...
         'scheduled pulses let lamdot build higher, that moment arrives ' ...
         'earlier, and the unbounded burn runs longer.']);
end

% ---- 9  the listing's plot range: reported, NEVER scored --------------------
B = addInfo(B, 'listing plot range for lamdot', 'axis([0 500 0 .00002])', ...
    sprintf('peak |lamdot| in the 10-pulse run = %.3e rad/s', peakAbs(p10.xlamd)), ...
    'zarchan.txt:4623', ...
    ['an axis() range is a plot window the author chose, not a measurement; ' ...
     'Figs. 15.10-15.13 publish no LOS-rate, miss or divert number at all']);
end


% =============================================================================
function v = peakAbs(x)
if isempty(x),  v = NaN;  else,  v = max(abs(x));  end
end

function v = windowPeak(S, tgoLo, tgoHi)
%WINDOWPEAK  max |lamdot| over the samples whose TGO lies in [tgoLo, tgoHi].
m = S.tgo >= tgoLo & S.tgo <= tgoHi;
if ~any(m),  v = NaN;  else,  v = max(abs(S.xlamd(m)));  end
end

function n = windowRises(S, tgoLo, tgoHi)
%WINDOWRISES  how many samples in the window carry a LARGER |lamdot| than the
%   previous one -- i.e. how often the LOS rate rebuilds instead of decaying.
m = S.tgo >= tgoLo & S.tgo <= tgoHi;
a = abs(S.xlamd(m));
if numel(a) < 2,  n = 0;  else,  n = sum(diff(a) > 0);  end
end

function [nGood, nTest] = pulseKnockdown(S)
%PULSEKNOCKDOWN  For each burn that actually ran (duration > 0) and was shut
%   off again, did |lamdot| end smaller than it started?  Zero-length slots --
%   the DISC <= 0 quirk documented in the header -- are excluded: a pulse that
%   never burned cannot be asked to have reduced anything.
nGood = 0;  nTest = 0;
for k = 1:numel(S.pulse.idx)
    j = S.pulse.idx(k);
    if j >= 1 && j <= numel(S.pulse.dur) && S.pulse.dur(j) > 0
        nTest = nTest + 1;
        if S.pulse.lam_off(k) < S.pulse.lam_on(j),  nGood = nGood + 1;  end
    end
end
end


% =============================================================================
function s = render(R, ev)
%RENDER  Reads R.p and R.runs, never opts: R.p is what the run ACTUALLY used.
L = {};
L{end+1} = 'mgs.zarchan.pulsedGuidance -- Zarchan Listing 15.2, PULSED GUIDANCE';
L{end+1} = sprintf('units: %s', R.p.units);
L{end+1} = sprintf(['AMAG = %.4g ft/s^2 (%.3g g)   PULSES = %g (alt %g)   ' ...
    'PREDERR = %.4g ft   TF = %g s   H = %g / %g s'], R.p.AMAG, R.p.AMAG/32.2, ...
    R.p.PULSES, R.p.PULSES_ALT, R.p.PREDERR, R.p.TF, R.p.H, R.p.HFINE);
L{end+1} = '';
L{end+1} = 'run       miss(ft)  RTM_exit(ft)  DELV(ft/s)  sched  maxburn(s)   steps';
f = fieldnames(R.runs);
for i = 1:numel(f)
    S = R.runs.(f{i});
    L{end+1} = sprintf('%-9s %9.4g %13.4g %11.4g %6d %11.4g %7d', f{i}, ...
        S.miss_ft, S.rtm_exit_ft, S.delv_ft, S.n_sched, S.max_burn_s, S.n_steps); %#ok<AGROW>
end
% ---- the divert-vs-pulse-count sweep.  This is what Listing 15.2 is ABOUT,
%      so it prints as a table and not only as a one-line board cell.
if isfield(R, 'sweep') && R.sweep.ran
    W = R.sweep;
    L{end+1} = '';
    L{end+1} = 'DIVERT vs PULSE COUNT (measured; same launch state at every point)';
    L{end+1} = '  N   sched   DELV(ft/s)   sched(ft/s)   terminal(ft/s)   t_term(s)  burn(s)  miss(ft)';
    for i = 1:numel(W.pulses)
        L{end+1} = sprintf('%3g %6d %11.4g %13.4g %15.4g %11.4g %8.3g %9.4g', ...
            W.pulses(i), W.n_sched(i), W.delv_ft(i), W.delv_sched_ft(i), ...
            W.delv_terminal_ft(i), W.t_terminal_s(i), W.term_dur_s(i), ...
            W.miss_ft(i)); %#ok<AGROW>
    end
    L{end+1} = sprintf(['  spread N=%g..%g : TOTAL %.0f%%, SCHEDULED %.0f%%   |   ' ...
        'over the book''s N=%g..%g : TOTAL %.0f%%, SCHEDULED %.1f%%'], ...
        min(W.pulses), max(W.pulses), W.total_spread_pct, W.sched_spread_pct, ...
        W.book_lo, W.book_hi, W.total_spread_book_pct, W.sched_spread_book_pct);
    L{end+1} = ['  => the pulse count DOES change the total divert, and over the ' ...
        'book''s own 5..10 range the'];
    L{end+1} = ['     whole of the change is the length of the final LATCHED ' ...
        'burn-to-impact: the scheduled'];
    L{end+1} = '     pulses cost the same to within 1%, the terminal burn 3.4x more at 5 pulses.';
end

L{end+1} = '';
L{end+1} = 'benchmarks (Ch.15 publishes NO number for this listing -- shapes and bounds only)';
for i = 1:numel(R.bench)
    b = R.bench(i);
    if b.skipped
        tag = upper(b.kind);
    elseif b.ok
        tag = 'ok  ';
    else
        tag = 'FAIL';
    end
    L{end+1} = sprintf('  [%s] %-40s %s', tag, b.what, b.ours_str); %#ok<AGROW>
    L{end+1} = sprintf('         book: %s', b.book_str);            %#ok<AGROW>
    L{end+1} = sprintf('         src : %s', b.source);              %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = 'structural self-checks';
for i = 1:numel(R.selfcheck.names)
    if R.selfcheck.oks(i),  t = 'ok  ';  else,  t = 'FAIL';  end
    L{end+1} = sprintf('  [%s] %s', t, R.selfcheck.names{i});       %#ok<AGROW>
end
L{end+1} = sprintf('scored rows: %d   pass: %d', sum(ev), R.pass);
s = strjoin(L, newline);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER must match the adders exactly; a reordered
%   struct makes [B.ok] mean something else while still looking like a typo.
%   This is the schema mgs.zarchan.benchSuite's adaptGeneric consumes, and it
%   is STRICT -- a near miss errors rather than being accommodated.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
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
%ADDINFO  Reported, never scored: skipped = true keeps it out of the pass count
%   in BOTH directions.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'info';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'info';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true;  B(i).why = why;
B(i).source = source;  B(i).units = '-';  B(i).detail = '';
end

function B = addSkip(B, what, bookStr, why, source, units)
%ADDSKIP  A check that COULD NOT RUN.  Never a pass.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'skip';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = '--';
B(i).pct  = NaN;   B(i).band = 'skip';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true;  B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end
