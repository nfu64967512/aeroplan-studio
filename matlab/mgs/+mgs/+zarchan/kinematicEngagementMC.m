function R = kinematicEngagementMC(opts)
%MGS.ZARCHAN.KINEMATICENGAGEMENTMC  Zarchan Listing 32.2 -- TWO-DIMENSIONAL
%   KINEMATIC MULTIPLE-RUN ENGAGEMENT SIMULATION: an impulsive interceptor
%   against an impulsive ICBM, Keplerian round non-rotating Earth.
%
%   R = mgs.zarchan.kinematicEngagementMC()
%   R = mgs.zarchan.kinematicEngagementMC('engage', false)   % skip the loop
%   R = mgs.zarchan.kinematicEngagementMC('zkm', 9000, 'assert', false)
%
%   Book source (the DECODED file; line numbers are shared with the raw one):
%     zarchan_decoded.txt:12200 LISTING 32.2 header, :12201-12238 the code
%     (predict32 at :12234-12238), :12157 tFME = 252 + .223*DRkm - 5.44e-6*DRkm^2,
%     :12172 the 7.19/7.33/7.23 km/s burnout velocities, :12194 the success gate
%     in prose, :12199 the sweep in prose, :12239 Fig. 32.3, :12241 Fig. 32.4.
%   Transcription ported: matlab/zarchan/_listings/listing_32_02.m
%
%   *** "MC" HERE IS MULTIPLE-CASE, NOT MONTE CARLO. ***  The name was given;
%   the listing is a SWEEP.  No rand, no randn, no noise, no dispersion -- the
%   outer loop is "for TF=(TLAUNCH+60):20:(TFTOT-20)" (:12201).  Nothing is
%   seeded because nothing is drawn, and R.p.seed is a STRING saying so rather
%   than a number that would imply otherwise.
%
%   UNITS ARE THE BOOK'S: ft, ft/s, ft/s^2; km via /3280; deg via 57.3;
%   a = 2.0926e7 ft, gm = 1.4077e16 ft^3/s^2.  Nothing is converted to SI.
%
% ---------------------------------------------------------------------------
%   THE ONE UNRESOLVED COMPARISON, AND THE OCR-DAMAGED ONES AROUND IT
%
%   listing_32_02.m reports ONE unresolved variable-to-variable comparison:
%   "if T.TLAUNCH" ahead of the missile's gravity terms.  Resolved T > TLAUNCH,
%   not T < TLAUNCH, forced by its own else branch (XMDD = 0, YMDD = 0): the
%   missile is the body that is INERT before launch and gravitating after it.
%   The other direction has the missile fall for 300 s and then freeze, and it
%   would still run.
%
%   ⚠️ MINUS SIGNS.  :12157 extracts as "tFME = 252+ 0.223DRkm  5.44106DR2 km":
%   BOTH minus signs are gone -- the one before 5.44 and the one in 10^-6.  The
%   listing's own "-(5.44E-6)*RDESKM*RDESKM" settles it, and 252+2230-544 =
%   1938 matches the figure annotation at :12185.  Read the formula off the
%   prose alone and the answer is 3026 s, which is also a plausible ICBM
%   flight time.
%
%   The rest are this book's "," = "<" and "." = ">", and here the PROSE states
%   the answer.  :12194 -- flights succeed if "the required missile velocity is
%   less than 8 km/s, the miss is less than 1000 ft, and the altitude of
%   intercept is greater than 50 km" -- fixes all three directions in
%   "if (VBOM,8. & RTM,1000. & ALTTKM.50.)".  The loop guard
%   "~((T.(TLAUNCH+50.)) & VC,0. & RTM,10000.)" reads the same way: stop after
%   launch+50 s once the range has opened (VC < 0) inside 10,000 ft.
%
%   Dropped, not "fixed" into code: the transcription's lines 53 onward, which
%   are the book's prose and the Figs. 32.3/32.4 axis text leaking in (:12239
%   onward).  Recorded and dropped: "save datfil.txt output /ascii" (typo for
%   -ascii); "XMDOLD=VRXM; YMDOLD=VRYM" in the launch block, dead because the
%   corrector has already run and the next iteration overwrites it;
%   "TGOLAMM=TF-T" written twice; BL, BT and S inside LAMBERT3D, assigned and
%   never read; predict32 declaring eleven arguments and reading seven.
%
% ---------------------------------------------------------------------------
%   ⭐ WHAT THIS PORT RESTRUCTURES, AND WHAT LICENSES IT
%
%   1  VBOM -- the only quantity Fig. 32.3 plots -- is computed AT LAUNCH by
%      LAMBERT3D(XM,YM,ZM,TF-TLAUNCH,XTF,YTF,ZTF).  The missile is stationary
%      until then, so XM,YM are still the initial ones; the engagement
%      integration that follows changes nothing about VBOM, only the MISS.
%   2  predict32 integrates the TARGET from 0 to TF at h = .01, and the target
%      trajectory depends on neither TF nor the interceptor.  Seventy-eight
%      calls re-walk one trajectory.  This port walks it ONCE and samples it.
%   So all of Fig. 32.3 -- three launch separations x 78 flight times -- costs
%   one target pass plus one Lambert call per point.  SELFCHECK 2 makes that a
%   claim rather than an assumption: it runs predict32 VERBATIM, its own loop
%   and its own Lambert call, and requires the sampled answer to equal it.
%
%   NOT reproduced by default: the 1000-ft miss gate across the full sweep --
%   78 runs x ~1.1e5 RK2 steps ~ 9e6 steps, for a miss the book itself calls
%   unreliable at this step size (:12194).  'engage' (default true) instead
%   runs the main loop VERBATIM for the two cases Fig. 32.4 names and reports
%   them as INFO.
%
%   TWO ADDED GUARDS, both deviations.  (a) The book's while condition needs
%   RTM < 10,000 ft to fire, so a run that misses by more NEVER TERMINATES;
%   'tmax_pad' seconds past TF the run stops and is flagged.  (b) When the
%   target at TF is nearly overhead the launch site THETA -> 0 and LAMBERT3D's
%   C2 = MAG1*SINA/CROSSMAG divides by zero -- for z = 9000 km the target
%   crosses overhead near TF = 1718 s, between two sweep points.  Points inside
%   'theta_min' come back NaN and counted, not silently plotted.
%
% ---------------------------------------------------------------------------
% ---------------------------------------------------------------------------
%   ⭐⭐⭐ THE NEAR-PARABOLIC GUARD DOES **NOT** APPLY TO THIS PORT, AND HERE IS
%   THE MEASUREMENT THAT SAYS SO  (2026-08-24)
%
%   THE FAULT IT WOULD HAVE GUARDED.  Listing 13.2's lambertpz -- carried in
%   SIX other files in this package -- is ELLIPTIC-ONLY.  Its bracket is
%   DEFINED by the escape condition lambda = r0|v|^2/gm = 2, so a transfer that
%   is genuinely HYPERBOLIC cannot be reached: the secant crawls to the bracket
%   EDGE and halts there.  Measured at alt 300 nm -> 0, transfer angle 78 deg,
%   TF = 750 s: it stops at lambda = 1.999999919 where bot2 = 1.6409e-11,
%   reports tF = 749.999996 s, PASSES its own 1e-8 residual test, returns
%   converged = TRUE -- and its answer misses the target by 98,751.56 ft when
%   propagated at RelTol 1e-13.  An independent universal-variable
%   (Bate-Mueller-White) Lambert misses by 0.00 ft.  ⚠⚠ All six transcriptions
%   agree to 0.000e+00 on that geometry with identical iteration counts, so
%   mutual agreement could not find it either.  Those six got a structural row
%   asserting lambda < 1.999999.
%
%   ⚠⚠ THIS FILE DOES NOT CARRY THAT ROUTINE.  LAMBERT3D is Listing 28.2, and
%   it has an EXPLICIT HYPERBOLIC BRANCH (LAMBDA > 2.0000001, the log form) and
%   an explicit near-parabolic branch between 1.9999999 and 2.0000001.  For it,
%   lambda above 2 is a correct answer, not a symptom.
%
%   MEASURED, over 9564 Lambert answers across 36 configurations of this port's
%   own options (RDESKM x TLOFT x dTF):
%       elliptic  (lambda < 1.9999999) : 7848
%       HYPERBOLIC (lambda > 2.0000001): 1716   <- 17.9%, all of them correct
%       inside the 2e-7 parabolic collar :  0
%   At the defaults alone it is 39 hyperbolic answers of 236, reaching
%   lambda = 220.35.  A "lambda < 1.999999" row here would therefore be a
%   FALSE ALARM ON 17.9% OF CORRECT ANSWERS.  It is not added.
%
%   ⚠ NOR IS A NEAR-PARABOLIC COLLAR ROW ADDED.  The obvious substitute -- "no
%   returned answer sits in [1.9999999, 2.0000001] where the routine abandons
%   both exact closed forms" -- is TRUE here, and it is true because nothing
%   comes near: the closest any of those 9564 answers got to lambda = 2 is
%   8.235e-04, which is 8235x the collar's half-width.  A row that cannot be
%   driven red is a row nobody has verified, so it is not scored.  What is
%   kept instead is the DATA: R.p.lambda_all, .lambda_max and .lambda_min, so a
%   later reader can see where this port actually sat rather than trust a
%   green tick.
%
%   THIS PORT'S OWN LAMBERT FAULT IS A DIFFERENT ONE and is already scored:
%   the bracket COLLAPSE erratum documented above, caught by structural
%   invariant 3.  It is a failure of the bracket in GAMMA, not of the branch
%   in lambda.
% ---------------------------------------------------------------------------
%
%   ⭐⭐⭐ THE ONE LAMBERT CALL THAT DOES NOT CONVERGE  (measured 2026-08-24)
%
%   Structural invariant 3 is RED at the defaults.  EXACTLY ONE of the 237
%   Lambert calls a default run makes fails, and it is always the same one:
%
%       z = 9000 km, TF = 1560 s (flight 1260 s), theta = 10.977 deg
%       101 iterations (the cap), |TF-T|/TF = 1.5137
%
%   The three obvious explanations were each tested and each REFUTED:
%
%   (a) NOT a mis-transcription.  LAMBERT3D's three bracket updates match
%       zarchan_decoded.txt:10440/:10442/:10443 character for character, including
%       that all three are separate un-elsed "if"s and that the cap is the
%       book's own ":10450  if ITERS.100 break".
%
%   (b) NOT an infeasible geometry.  A bisection on the ROUTINE'S OWN T(gamma)
%       finds T rising monotonically from 0 at G_MIN = 27.2550 deg to +Inf at
%       the parabolic boundary, with a UNIQUE crossing of TF = 1260 s at
%       gamma = 74.7798 deg, |v| = 17225.77 ft/s = 5.2518 km/s.  That value
%       sits exactly on the smooth curve between this sweep's own neighbours,
%       5.2519 km/s at 1240 s and 5.2540 km/s at 1280 s.  The solution exists;
%       the solver does not reach it.
%
%   (c) NOT a cap that is too low.  Caps of 100 / 200 / 1000 / 100000 all
%       return the SAME residual 1.5137 and the SAME wrong speed, because
%       GAMMA stops changing at iteration 3.  Raising it buys nothing and
%       would be the same sin as widening a band.
%
%   WHAT IT ACTUALLY IS: an erratum in Listing 28.2.  Iteration 1 sits at the
%   bracket midpoint 58.6275 deg where T = 643.4 s, short of the 1260 s asked.
%   T(gamma) is steeply convex near the parabolic boundary, so the secant step
%   extrapolates to 88.6956 deg -- past escape, but still inside G_MAX = 90 deg,
%   so the "NEXT >= G_MAX" clamp does not fire.  There T comes back NEGATIVE
%   (-647.26 s, off the hyperbolic branch).  Now the second update fires
%   (T < 0 and GAMMA < G_MAX  =>  G_MAX = GAMMA) AND SO DOES THE THIRD
%   (T < TF and GAMMA > G_MIN  =>  G_MIN = GAMMA).  Both ends land on 88.6956
%   deg in the same pass, the bracket has zero width, NEXT = (G_MIN+G_MAX)/2 is
%   GAMMA again, and the iteration is a fixed point until the cap.
%
%   The book calls 28.2 "a straightforward extension" of the 2-D routine in
%   Ch.13 (:10407).  It is not, in this one respect: Listing 13.2 updates the
%   bracket with a MUTUALLY EXCLUSIVE "if tf > tfdes gmax=gam; else gmin=gam;
%   end" (:3948-:3949), which cannot collapse.  Feeding this exact geometry to
%   Listing 13.2's routine -- all six independent transcriptions of it in this
%   package -- returns 17225.77 ft/s in 8 iterations, residual 2e-8 relative.
%
%   NOT REPAIRED.  Repairing the listing would make this port stop being a port.
%   What changed instead: a non-converged call now returns NaN rather than the
%   velocity belonging to whatever GAMMA the cap caught it on (16.3434 km/s
%   here, against a true 5.2518), and the failing call is reported WITH ITS
%   GEOMETRY in R.p.lambert_failed, in invariant 3's detail and in the summary.
%   Nothing that was scored moved: all nine scored benchmarks still pass, the
%   point was already outside the VBOM < 8 km/s gate, and Fig. 32.3's shape row
%   is decided by the 117.38 km/s at the shortest flight time.
%
%   ⚠️ IT IS NOT ONE UNLUCKY POINT.  The failure is a property of the geometry
%   family, not of this scenario: RDESKM = 8000 km moves it to TWO other points
%   (TF 1580 s and 1640 s, both z = 9000 km).  Anything that changes the sweep
%   changes WHICH calls collapse, so a run that comes back 0-of-N has not
%   proved the routine sound -- it has missed the band.  The measured trigger
%   is a first secant step that lands between the parabolic gamma and G_MAX,
%   which the "NEXT >= G_MAX" clamp does not cover.
%
%   Fault injection, 2026-08-24, so the row is known to be live in BOTH
%   directions rather than permanently red:
%     dTF = 32 s (1560 s off the grid)          0 of 150 fail, invariant GREEN
%     zkm = [1000 5000]                         0 of 159 fail, invariant GREEN
%     cap 100 -> 6                            105 of 237 fail, invariant RED
%     third if made exclusive of T < 0 (the     0 of 237 fail, GREEN, and the
%       13.2 rule; NOT KEPT, run on a copy)     1260 s point returns 5.2518
%                                               km/s -- the bisection answer
%     conv forced true                          0 reported, still RED on the
%                                               residual, 16.3434 km/s is back
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED WHEN WRITTEN; the values below were
%   CONFIRMED BY EXECUTION on 2026-08-24 (R2025b), and where measurement and
%   derivation differ the measurement is written next to it. ***
%     TFTOT = 252 + 2230 - 544 = 1938.0 s exactly; sweep 360:20:1900, 78 points
%     VBOT ~ 7.193 km/s.  Min-energy closed form v^2 = (gm/a)*2*sin(phi)/
%       (1+sin(phi)), 2*phi = 3.28e7/2.0926e7 rad, gives 23594.6 ft/s =
%       7.1934 km/s; TF = 1938 s is 5 s past the true minimum-energy 1932.8 s
%       and the speed is at a MINIMUM there, so the excess is second-order.
%       Book prints 7.19.
%     z = 9000 km: ~6.5 km/s at 800 s of flight, ~5.3 km/s at 1400 s.
%     Engagement misses at H = .01/.0001: order tens to hundreds of feet.
%   A residual near a band EDGE is not a pass to be accepted.
%
%   OUTPUTS.  R.tf/.flight_s; R.zkm; R.vbom_kms (nz x ntf, NaN where guarded);
%   R.gamma_deg, R.apogee_km (interceptor, closed form from the Lambert
%   solution's orbital elements -- no integration); R.alt_int_km (target
%   altitude at TF, the 50-km gate); R.gate; R.vbot_kms/.vbot_me_kms/
%   .vbot_loft_kms/.vbot_depr_kms; R.tftot; R.target; R.engage; R.bench,
%   R.selfcheck, R.units, R.pass, R.verdict, R.p, R.summary.  'assert' chooses
%   the reaction to a verdict and nothing else: true errors, false WARNS and
%   still returns R complete.
arguments
    opts.TLAUNCH   (1,1) double {mustBeFinite, mustBeNonnegative} = 300.
    opts.RDESKM    (1,1) double {mustBeFinite, mustBePositive}    = 10000.
    opts.TLOFT     (1,1) double {mustBeFinite}                    = 0.
    opts.TS        (1,1) double {mustBeFinite, mustBePositive}    = 1.
    opts.H         (1,1) double {mustBeFinite, mustBePositive}    = .01
    opts.ALTMKMIC  (1,1) double {mustBeFinite, mustBeNonnegative} = 0.
    % The book's nominal is 1000 km (XLONGMDEGICKM); :12239 adds 5000 and 9000.
    opts.zkm       (1,:) double {mustBeFinite, mustBeNonnegative} = [1000 5000 9000]
    opts.dTF       (1,1) double {mustBeFinite, mustBePositive}    = 20.
    % rad.  See the near-radial guard in the header.
    opts.theta_min (1,1) double {mustBeFinite, mustBePositive}    = 1e-3
    opts.engage      (1,1) logical = true
    opts.engage_zkm  (1,1) double {mustBeFinite, mustBeNonnegative} = 9000.
    opts.engage_tf   (1,:) double {mustBeFinite, mustBePositive} = [1100. 1700.]
    opts.tmax_pad    (1,1) double {mustBeFinite, mustBePositive} = 120.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.assert    (1,1) logical = true
end

A  = 2.0926E7;      % ft   -- listing line 2
GM = 1.4077E16;     % ft^3/s^2

TLAUNCH = opts.TLAUNCH;  RDESKM = opts.RDESKM;  TLOFT = opts.TLOFT;
TS = opts.TS;  H = opts.H;

stride = TS/H;
if abs(stride - round(stride)) > 1e-9
    error('mgs:zarchan:kinematicEngagementMC:stride', ...
        ['TS = %g s is not an integer number of steps of H = %g s, so the ' ...
         'sampled target pass could not land exactly on a sweep time and the ' ...
         'predicted intercept point would be off by up to H/2 -- silently.'], TS, H);
end
stride = round(stride);
if abs(opts.dTF/TS - round(opts.dTF/TS)) > 1e-9
    error('mgs:zarchan:kinematicEngagementMC:grid', ...
        'dTF = %g s must be a whole number of TS = %g s samples.', opts.dTF, TS);
end

% ---- the time-of-flight formula, zarchan.txt:12157 -------------------------
TFTOT_ME = 252. + .223*RDESKM - (5.44E-6)*RDESKM*RDESKM;
TFTOT    = TFTOT_ME + TLOFT;

% ---- target initial conditions and its Lambert burn (listing :12201-12232) --
XLONGFDEG = 57.3*RDESKM*3280./A;
XLONGF = XLONGFDEG/57.3;
XF = A*cos(XLONGF);  YF = A*sin(XLONGF);  ZF = 0.;
XT0 = A;  YT0 = 0.;  ZT0 = 0.;
[VRX, VRY, ~, itT, errT, convT, lamT] = lambert3d(XT0, YT0, ZT0, TFTOT, XF, YF, ZF, 0, GM);
lamAll = lamT;                          % near-parabolic census, see the header
XTD0 = VRX;  YTD0 = VRY;
VBOT = sqrt(XTD0^2 + YTD0^2)/3280.;
% The target's own burn.  If THIS one failed nothing downstream means anything:
% the whole pass, every sampled intercept point and every sweep row are built
% on it, and they would all come out NaN rather than wrong -- but silently NaN
% is still worse than saying so.
lamFail = struct('where', {}, 'zkm', {}, 'tf', {}, 'flight_s', {}, ...
                 'theta_deg', {}, 'iters', {}, 'terr', {});
if ~convT
    lamFail(end+1) = struct('where','target burn (TFTOT)', 'zkm',NaN, ...
        'tf',TFTOT, 'flight_s',TFTOT, 'theta_deg',NaN, 'iters',itT, 'terr',errT);
end

% The three trajectories of :12172, one Lambert call each.  VBOT above is the
% RUN's, at TFTOT = TFTOT_ME + TLOFT; VBOT_ME is explicitly the minimum-energy
% one, so that benchmark 3 cannot be scored against a lofted run by accident.
% Each of these is a SCORED benchmark (7.19 / 7.33 / 7.23 km/s), so a
% non-converged one must not reach addVal as a number: NaN there is recorded as
% "the computed value is not finite" and the row skips instead of scoring a
% velocity that solves a different flight time.
if TLOFT == 0.
    VBOT_ME = VBOT;
else
    [vm1, vm2, ~, itm, erm, cvm, lamM] = lambert3d(XT0, YT0, ZT0, TFTOT_ME, XF, YF, ZF, 0, GM);
    lamAll(end+1) = lamM;
    VBOT_ME = sqrt(vm1^2 + vm2^2)/3280.;
    if ~cvm, lamFail(end+1) = lf('minimum-energy target burn', TFTOT_ME, itm, erm); end
end
[vl1, vl2, ~, itl, erl, cvl, lamL] = lambert3d(XT0, YT0, ZT0, TFTOT_ME + 500., XF, YF, ZF, 0, GM);
lamAll(end+1) = lamL;
VBOT_LOFT = sqrt(vl1^2 + vl2^2)/3280.;
if ~cvl, lamFail(end+1) = lf('lofted target burn (TLOFT=500)', TFTOT_ME+500., itl, erl); end
[vd1, vd2, ~, itd, erd, cvd, lamD] = lambert3d(XT0, YT0, ZT0, TFTOT_ME - 200., XF, YF, ZF, 0, GM);
lamAll(end+1) = lamD;
VBOT_DEPR = sqrt(vd1^2 + vd2^2)/3280.;
if ~cvd, lamFail(end+1) = lf('depressed target burn (TLOFT=-200)', TFTOT_ME-200., itd, erd); end

% ---- ONE target pass; see the header.  This IS predict32, for every TF. -----
[trec, TX, TY, TXD, TYD] = targetPass(XT0, YT0, XTD0, YTD0, H, TFTOT, stride, GM);

% ---- the sweep, listing :12201 --------------------------------------------
tf  = (TLAUNCH + 60.):opts.dTF:(TFTOT - 20.);
nz  = numel(opts.zkm);
ntf = numel(tf);
ALTM = opts.ALTMKMIC*3280.;

vbom   = nan(nz, ntf);
gam    = nan(nz, ntf);
apog   = nan(nz, ntf);
theta  = nan(nz, ntf);
altint = nan(1, ntf);
lamErr = errT;  lamIt = itT;  nRadial = 0;

% ⚠️ The sweep reads the shared pass by INDEX.  If a sweep time is not a whole
% number of TS samples the index silently names a DIFFERENT time, the Lambert
% call is then solved to the wrong intercept point, and every VBOM on the board
% is plausibly wrong with no symptom.  Checked once, up front, against the
% recorded times themselves rather than against the arithmetic that built them.
kIdx = round(tf/TS);
if any(kIdx < 1) || any(kIdx > numel(trec)) || ...
        max(abs(trec(min(max(kIdx,1),numel(trec))) - tf)) > 0.5*H + 1e-9
    error('mgs:zarchan:kinematicEngagementMC:sample', ...
        ['a sweep time is not a whole number of TS = %g s samples of the ' ...
         'target pass, so the predicted intercept point would be taken from ' ...
         'the wrong instant.  Sweep %g:%g:%g s.'], TS, tf(1), opts.dTF, tf(end));
end

for i = 1:ntf
    k = kIdx(i);
    XTF = TX(k);  YTF = TY(k);
    altint(i) = (sqrt(XTF^2 + YTF^2) - A)/3280.;
    for j = 1:nz
        XLONGM = (opts.zkm(j)/111.)/57.3;
        XM0 = (A + ALTM)*cos(XLONGM);  YM0 = (A + ALTM)*sin(XLONGM);
        ct = (XM0*XTF + YM0*YTF)/(sqrt(XM0^2+YM0^2)*sqrt(XTF^2+YTF^2));
        theta(j,i) = acos(max(min(ct,1),-1));
        if theta(j,i) < opts.theta_min
            nRadial = nRadial + 1;      % near-radial: see the header
            continue
        end
        [vx, vy, ~, it, er, cv, lamS] = lambert3d(XM0, YM0, 0., tf(i)-TLAUNCH, ...
                                            XTF, YTF, 0., 0, GM);
        lamErr = max(lamErr, er);  lamIt = max(lamIt, it);
        lamAll(end+1) = lamS; %#ok<AGROW>
        if ~cv
            % NOT continue-with-a-number.  vbom/gam/apog stay NaN and the call
            % is recorded WITH ITS GEOMETRY, because "worst iteration count 101"
            % tells a reader that something failed and nothing about where.
            lamFail(end+1) = struct('where','sweep', 'zkm',opts.zkm(j), ...
                'tf',tf(i), 'flight_s',tf(i)-TLAUNCH, ...
                'theta_deg',57.3*theta(j,i), 'iters',it, 'terr',er); %#ok<AGROW>
            continue
        end
        vbom(j,i) = sqrt(vx^2 + vy^2)/3280.;
        [gam(j,i), apog(j,i)] = elements(XM0, YM0, vx, vy, GM, A);
    end
end

% The two gates evaluable without the engagement integration (:12194).
gate = (vbom < 8.) & repmat(altint > 50., nz, 1);

% ---- the listing's main loop, verbatim, for the Fig. 32.4 cases ------------
ENG = struct('tf', {}, 'flight_s', {}, 'zkm', {}, 'vbom_kms', {}, ...
             'miss_ft', {}, 'alt_int_km', {}, 'apogee_km', {}, ...
             'tstop', {}, 'timeout', {});
if opts.engage
    XLONGME = (opts.engage_zkm/111.)/57.3;
    XME = (A + ALTM)*cos(XLONGME);  YME = (A + ALTM)*sin(XLONGME);
    for j = 1:numel(opts.engage_tf)
        TFe = opts.engage_tf(j);
        ke  = round(TFe/TS);
        % Same index hazard as the sweep, same refusal -- but here the case is
        % dropped rather than the whole run, because engage_tf is a diagnostic
        % list and a bad entry in it should not take the board down.
        if ke < 1 || ke > numel(trec) || abs(trec(ke) - TFe) > 0.5*H + 1e-9
            continue
        end
        ENG(end+1) = engageRun(XT0, YT0, XTD0, YTD0, XME, YME, TFe, TLAUNCH, ...
            TX(ke), TY(ke), H, TS, GM, A, TFe + opts.tmax_pad, ...
            opts.engage_zkm); %#ok<AGROW>
    end
end

R = struct();
R.tf         = tf;
R.flight_s   = tf - TLAUNCH;
R.zkm        = opts.zkm;
R.vbom_kms   = vbom;
R.gamma_deg  = gam;
R.apogee_km  = apog;
R.theta_rad  = theta;
R.alt_int_km = altint;
R.gate       = gate;
R.vbot_kms   = VBOT;
R.vbot_me_kms   = VBOT_ME;
R.vbot_loft_kms = VBOT_LOFT;
R.vbot_depr_kms = VBOT_DEPR;
R.tftot      = TFTOT;
R.tftot_me   = TFTOT_ME;
R.target     = struct('t', trec, 'x', TX, 'y', TY, 'xd', TXD, 'yd', TYD);
R.engage     = ENG;
R.units      = ['ft, ft/s, ft/s^2 (g = 32.2); km via /3280; deg via 57.3; ' ...
                'a = 2.0926e7 ft, gm = 1.4077e16 ft^3/s^2'];
R.p = struct('TLAUNCH',TLAUNCH, 'RDESKM',RDESKM, 'TLOFT',TLOFT, 'TS',TS, ...
             'H',H, 'ALTMKMIC',opts.ALTMKMIC, 'dTF',opts.dTF, ...
             'theta_min',opts.theta_min, 'n_radial_skipped',nRadial, ...
             'lambda_all',lamAll, 'lambda_max',maxFinite(lamAll), ...
             'lambda_min',minFinite(lamAll), ...
             'lambert_maxreltime',lamErr, 'lambert_maxiters',lamIt, ...
             'lambert_failed',{lamFail}, 'n_lambert_failed',numel(lamFail), ...
             'lambert_calls',1 + 2 + double(TLOFT~=0.) + nz*ntf - nRadial, ...
             'engage',opts.engage, 'engage_zkm',opts.engage_zkm, ...
             'seed','none (deterministic sweep -- no rand in Listing 32.2)', ...
             'units',R.units);

R.selfcheck = structuralChecks(R, XT0, YT0, XF, YF, TS, H, GM, A, TFTOT, ...
                               RDESKM, TLAUNCH, opts.dTF, lamErr, lamIt, lamFail);

if opts.benchmark
    R.bench = benchmarks(R, TFTOT_ME, TLAUNCH, RDESKM);
    ev      = ~[R.bench.skipped] & ~strcmpi({R.bench.kind}, 'info');
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine".
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false: R.pass is false meaning NOT CHECKED.';
elseif ~any(ev)
    msg = 'not one book value could be evaluated at these parameters.';
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question.  ' ...
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
        error('mgs:zarchan:kinematicEngagementMC:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:kinematicEngagementMC:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [trec, X, Y, XD, YD] = targetPass(XT, YT, XTD, YTD, H, TEND, stride, GM)
%TARGETPASS  predict32's integrator (zarchan.txt:12235-12237), walked ONCE to
%   TEND and sampled every 'stride' steps.  Constants arrive as ARGUMENTS:
%   MATLAB local functions cannot see the caller's workspace.
%
%   The corrector is predict32's own algebra, "xt=(xtold+xt)/2+.5*h*xtd", NOT
%   the main loop's ".5*(XTOLD+XT+H*XTD)".  The two are algebraically equal and
%   differ only in rounding -- and it is predict32 this pass stands in for, so
%   it is predict32's form that is used.
n = floor(round(TEND/H)/stride);
trec = zeros(1,n);  X = zeros(1,n);  Y = zeros(1,n);
XD = zeros(1,n);    YD = zeros(1,n);
T = 0.;  XTDD = 0.;  YTDD = 0.;  m = 0;
for k = 1:(n*stride)
    XTOLD = XT;  YTOLD = YT;  XTDOLD = XTD;  YTDOLD = YTD;
    STEP = 1;  FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            XT = XT + H*XTD;  YT = YT + H*YTD;
            XTD = XTD + H*XTDD;  YTD = YTD + H*YTDD;
            T = T + H;  STEP = 2;
        end
        TEMBOT = (XT^2 + YT^2)^1.5;
        XTDD = -GM*XT/TEMBOT;  YTDD = -GM*YT/TEMBOT;
        FLAG = 1;
    end
    XT  = (XTOLD  + XT )/2 + .5*H*XTD;
    YT  = (YTOLD  + YT )/2 + .5*H*YTD;
    XTD = (XTDOLD + XTD)/2 + .5*H*XTDD;
    YTD = (YTDOLD + YTD)/2 + .5*H*YTDD;
    if mod(k, stride) == 0
        m = m + 1;
        trec(m) = T;  X(m) = XT;  Y(m) = YT;  XD(m) = XTD;  YD(m) = YTD;
    end
end
end


% =============================================================================
function [xtf, ytf] = predict32(tp, xtp, ytp, tf, tftot, xf, yf, gm, h)
%PREDICT32  zarchan.txt:12234-12238, VERBATIM, as its own loop with its own
%   Lambert call.  Used ONLY by selfcheck 2, whose whole job is to prove that
%   the sampled shared pass returns what this returns.
%
%   The book declares eleven arguments and READS seven.  xtinit/ytinit are
%   never mentioned again, and xtdp/ytdp are overwritten on the fourth line by
%   this routine's OWN Lambert call before anything reads them -- so passing
%   the target's velocity in is decoration.  Four dropped, seven carried.
%
%   'h' IS AN ARGUMENT HERE AND A LITERAL ".01" IN THE BOOK.  At the default
%   they are the same number.  Away from it the BOOK is inconsistent -- its
%   main loop steps at H while its predictor stays at .01, so the intercept
%   point stops being the point the engagement flies to -- and reproducing that
%   would make selfcheck 2 fail for a reason that has nothing to do with this
%   port.  The book's literal is recorded here instead of being obeyed.
t = tp;  xt = xtp;  yt = ytp;  zt = 0.;  zf = 0.;
tgolam = tftot - t;
[vrx, vry] = lambert3d(xt, yt, zt, tgolam, xf, yf, zf, 0, gm);
xtd = vrx;  ytd = vry;  xtdd = 0.;  ytdd = 0.;
while t <= (tf - .00001)
    xtold = xt;  ytold = yt;  xtdold = xtd;  ytdold = ytd;
    step = 1;  flag = 0;
    while step <= 1
        if flag == 1
            xt = xt + h*xtd;  yt = yt + h*ytd;
            xtd = xtd + h*xtdd;  ytd = ytd + h*ytdd;
            t = t + h;  step = 2;
        end
        tembot = (xt^2 + yt^2)^1.5;
        xtdd = -gm*xt/tembot;  ytdd = -gm*yt/tembot;
        flag = 1;
    end
    xt  = (xtold  + xt )/2 + .5*h*xtd;
    yt  = (ytold  + yt )/2 + .5*h*ytd;
    xtd = (xtdold + xtd)/2 + .5*h*xtdd;
    ytd = (ytdold + ytd)/2 + .5*h*ytdd;
end
xtf = xt;  ytf = yt;
end


% =============================================================================
function E = engageRun(XT, YT, XTD, YTD, XM, YM, TF, TLAUNCH, XTFACT, YTFACT, ...
                       H0, TS, GM, A, TMAX, zkm)
%ENGAGERUN  The listing's main while loop, zarchan.txt:12213-12233, verbatim.
%
%   The comparisons are resolved as the header says.  XTF/YTF are initialised
%   before the loop to the values the listing's own "if T >= TLAUNCH" block
%   assigns them; identical, and it removes a read-before-write that only the
%   ordering of two adjacent ifs protects.
%
%   TMAX IS THIS PORT'S, NOT THE BOOK'S.  The book's guard needs RTM < 10000 ft
%   to fire, so a run that misses by more integrates forever.
XTDD = 0.;  YTDD = 0.;  XMD = 0.;  YMD = 0.;  XMDD = 0.;  YMDD = 0.;
ZM = 0.;  T = 0.;  S = 0.;  H = H0;  QBOOSTM = 1;  VBOM = NaN;
XTF = XTFACT;  YTF = YTFACT;  ZTF = 0.;
RTM1 = XT - XM;  RTM2 = YT - YM;  RTM = sqrt(RTM1^2 + RTM2^2);
VTM1 = XTD - XMD;  VTM2 = YTD - YMD;
VC = -(RTM1*VTM1 + RTM2*VTM2)/RTM;
ALTMKM = (sqrt(XM^2 + YM^2) - A)/3280.;
ALTTKM = (sqrt(XT^2 + YT^2) - A)/3280.;
apo = 0.;  timeout = false;

while ~((T > (TLAUNCH + 50.)) && VC < 0. && RTM < 10000.)
    if T > TMAX, timeout = true;  break;  end
    if RTM < 1000., H = .0001;  else, H = H0;  end
    XTOLD = XT;  YTOLD = YT;  XTDOLD = XTD;  YTDOLD = YTD;
    XMOLD = XM;  YMOLD = YM;  XMDOLD = XMD;  YMDOLD = YMD;
    STEP = 1;  FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            XT = XT + H*XTD;  YT = YT + H*YTD;
            XTD = XTD + H*XTDD;  YTD = YTD + H*YTDD;
            XM = XM + H*XMD;  YM = YM + H*YMD;
            XMD = XMD + H*XMDD;  YMD = YMD + H*YMDD;
            T = T + H;
        end
        TEMPBOTT = (XT^2 + YT^2)^1.5;
        XTDD = -GM*XT/TEMPBOTT;  YTDD = -GM*YT/TEMPBOTT;
        RTM1 = XT - XM;  RTM2 = YT - YM;
        VTM1 = XTD - XMD;  VTM2 = YTD - YMD;
        RTM = sqrt(RTM1^2 + RTM2^2);
        VC = -(RTM1*VTM1 + RTM2*VTM2)/RTM;
        if T > TLAUNCH                       % <<< THE UNRESOLVED COMPARISON
            TEMPBOTM = (XM^2 + YM^2)^1.5;
            XMDD = -GM*XM/TEMPBOTM;  YMDD = -GM*YM/TEMPBOTM;
        else
            XMDD = 0.;  YMDD = 0.;
        end
        ALTMKM = (sqrt(XM^2 + YM^2) - A)/3280.;
        ALTTKM = (sqrt(XT^2 + YT^2) - A)/3280.;
        FLAG = 1;
    end
    XT  = .5*(XTOLD  + XT  + H*XTD);   YT  = .5*(YTOLD  + YT  + H*YTD);
    XTD = .5*(XTDOLD + XTD + H*XTDD);  YTD = .5*(YTDOLD + YTD + H*YTDD);
    XM  = .5*(XMOLD  + XM  + H*XMD);   YM  = .5*(YMOLD  + YM  + H*YMD);
    XMD = .5*(XMDOLD + XMD + H*XMDD);  YMD = .5*(YMDOLD + YMD + H*YMDD);
    S = S + H;
    if (T >= TLAUNCH) && QBOOSTM == 1
        QBOOSTM = 0;
        TGOLAMM = TF - T;
        [VRXM, VRYM] = lambert3d(XM, YM, ZM, TGOLAMM, XTF, YTF, ZTF, 0, GM);
        XMD = VRXM;  YMD = VRYM;
        VBOM = sqrt(XMD^2 + YMD^2)/3280.;
    end
    if S >= (TS - .0001)
        S = 0.;
        apo = max(apo, ALTMKM);
    end
end

E = struct('tf', TF, 'flight_s', TF - TLAUNCH, 'zkm', zkm, 'vbom_kms', VBOM, ...
           'miss_ft', RTM, 'alt_int_km', ALTTKM, 'apogee_km', apo, ...
           'tstop', T, 'timeout', timeout);
end


% =============================================================================
function [vrx, vry, vrz, iters, terr, conv, lamout] = lambert3d(XT, YT, ZT, ...
                                                TF, XF, YF, ZF, SWITCH1, GM)
%LAMBERT3D  Listing 28.2's Lambert routine (zarchan.txt:10428, listing header
%   :10408; transcription listing_28_02.m), which Listing 32.2 calls and does
%   not reprint -- its own comment points at Listing 28.3, but the routine is
%   printed in 28.2.  PI, DEG_PER_RAD, EARTH_RADIUS, FT_PER_KM and RAD_PER_SEC
%   are assigned on its first line; only PI is read, so pi is used directly.
%
%   Comparisons resolved with "," = "<" and
%   "." = ">"; the ceiling drops when the flight time comes out too long or
%   negative, the floor rises when it comes out too short, which is the only
%   consistent reading.  "/. . ." in the transcription is a line continuation.
%   BL, BT and S are assigned and never read in the book and are dropped.
%
% ---------------------------------------------------------------------------
%   ⭐⭐⭐ ERRATUM IN LISTING 28.2: THE BRACKET CAN COLLAPSE.  See the file
%   header, section "THE ONE LAMBERT CALL THAT DOES NOT CONVERGE".  The three
%   bracket updates below are the book's, verbatim and un-elsed (:10440/:10442/:10443),
%   and the second and third BOTH fire on an iterate whose flight time comes
%   back negative -- G_MAX <- GAMMA and G_MIN <- GAMMA in the same pass.  The
%   bracket is then a single point, NEXT = (G_MIN+G_MAX)/2 = GAMMA, and the
%   iteration is a fixed point until the cap.  Listing 13.2, which :10407 calls
%   this routine "a straightforward extension" of, does NOT have this: its
%   update is a mutually exclusive "if tf > tfdes gmax=gam; else gmin=gam; end"
%   (:3948-:3949).  NOT REPAIRED HERE -- the port reproduces the book, and structural
%   invariant 3 scores the consequence.
HALFPI = pi/2.;
RF0X = XF-XT;  RF0Y = YF-YT;  RF0Z = ZF-ZT;
R0DOTRF  = XT*XF + YT*YF + ZT*ZF;
R0DOTRF0 = XT*RF0X + YT*RF0Y + ZT*RF0Z;
R0MAG  = sqrt(XT^2+YT^2+ZT^2);
RFMAG  = sqrt(XF^2+YF^2+ZF^2);
RF0MAG = sqrt(RF0X^2+RF0Y^2+RF0Z^2);
RATIO = R0MAG/RFMAG;  GMDIVR0 = GM/R0MAG;
COS_T = R0DOTRF/(R0MAG*RFMAG);
VNUMER = GMDIVR0*(1.-COS_T);
T_MIN = 0.;
if SWITCH1 == 0
    G_MIN = HALFPI - acos(R0DOTRF0/(R0MAG*RF0MAG));  G_MAX = HALFPI;
    THETA = acos(COS_T);
else
    G_MIN = -HALFPI;  G_MAX = -HALFPI + acos(R0DOTRF0/(R0MAG*RF0MAG));
    THETA = 2.*pi - acos(COS_T);
end
SIN_T = sin(THETA);  COT_HALFT = 1./tan(THETA/2.);
GAMMA = (G_MAX+G_MIN)/2.;  GOLD = G_MIN;  TOLD = 0.;  T = 0.;  iters = 1;
RV0MAG = NaN;
while abs(TF-T) > .00000001*TF
    SIN_G = sin(GAMMA);  COS_G = cos(GAMMA);  TAN_G = SIN_G/COS_G;
    COS_TPLUSG = cos(THETA+GAMMA-2.*pi);
    TERM1 = (RATIO*COS_G-COS_TPLUSG)*COS_G;
    RV0MAG = sqrt(VNUMER/TERM1);
    % ⚠️ PORT ADDITION, and it is not cosmetic.  The book guards neither
    % sqrt(negative) here nor log(negative) in the escape branch.  MATLAB does
    % not error on either -- it returns a COMPLEX number, and every subsequent
    % ">" then silently compares real parts only.  A complex T is treated the
    % same way an invalid one is: hand it to the bisection leg.
    if ~isreal(RV0MAG) || ~isfinite(RV0MAG) || RV0MAG <= 0
        T = -1.;
    else
        LAMBDA = RV0MAG*RV0MAG/GMDIVR0;
        if LAMBDA < 1.9999999
            TERM0 = sqrt(2./LAMBDA-1.);
            TERM1 = (TAN_G*(1.-COS_T)+(1.-LAMBDA)*SIN_T)/((2.-LAMBDA)*RATIO);
            TERM2 = (COS_G+COS_G)/(LAMBDA*TERM0*TERM0*TERM0);
            TERM3 = atan2(TERM0, (COS_G*COT_HALFT-SIN_G));
            T = (R0MAG/(RV0MAG*COS_G))*(TERM1+TERM2*TERM3);
        elseif LAMBDA > 2.0000001
            TERM0 = sqrt(1.-2./LAMBDA);
            TERM1 = (TAN_G*(1.-COS_T)+(1.-LAMBDA)*SIN_T)/((2.-LAMBDA)*RATIO);
            TERM2 = COS_G/(LAMBDA*TERM0*TERM0*TERM0);
            TERM3 = SIN_G-COS_G*COT_HALFT;
            TERM3 = log((TERM3-TERM0)/(TERM3+TERM0));
            T = (R0MAG/(RV0MAG*COS_G))*(TERM1-TERM2*TERM3);
        else
            TERM0 = COS_G*COT_HALFT;  TERM1 = TERM0-SIN_G;
            TERM0 = (3*TERM0*TERM1+1.)/(TERM1*TERM1*TERM1);
            T = TERM0*(2.*R0MAG)/(3.*RV0MAG);
        end
        if ~isreal(T) || ~isfinite(T), T = -1.;  end
    end
    if T > TF  && GAMMA < G_MAX, G_MAX = GAMMA;  end
    if T < 0.  && GAMMA < G_MAX, G_MAX = GAMMA;  end
    if T < TF  && GAMMA > G_MIN, G_MIN = GAMMA;  T_MIN = T;  end
    if T < 0.
        NEXT = (G_MIN+G_MAX)/2.;  GOLD = G_MIN;  TOLD = T_MIN;
    else
        NEXT = GAMMA + (TF-T)*(GAMMA-GOLD)/(T-TOLD);
        if NEXT >= G_MAX
            NEXT = (GAMMA+G_MAX)/2.;
        elseif NEXT <= G_MIN
            NEXT = (GAMMA+G_MIN)/2.;
        end
        GOLD = GAMMA;  TOLD = T;
    end
    GAMMA = NEXT;  iters = iters + 1;
    % ⚠️ 100 IS THE BOOK'S OWN CAP, not this port's: ":10450  if ITERS.100
    % break".  Raising it is not an option and would not help anyway -- MEASURED
    % 2026-08-24, caps of 100 / 200 / 1000 / 100000 on the one geometry that
    % fails all return the IDENTICAL iters-at-cap, the identical residual
    % 1.5137 and the identical wrong speed, because GAMMA stops moving at
    % iteration 3.  See the file header.
    if iters > 100, break;  end
end
terr = abs(TF-T)/TF;
% The routine's OWN convergence test, re-evaluated rather than inferred from
% which way the loop exited.
conv = isreal(terr) && isfinite(terr) && terr <= .00000001;
if ~conv || ~isreal(RV0MAG) || ~isfinite(RV0MAG)
    % ⚠️ PORT ADDITION, and the second half of it is NEW.  The book returns the
    % velocity belonging to the last GAMMA it happened to be sitting on when
    % the cap fired -- a finite, plottable, ordinary-looking number that is not
    % a solution to the flight time that was asked for.  At the one sweep point
    % where this fires it is 16.3434 km/s against a true 5.2518, and it lands
    % in R.vbom_kms next to numbers that ARE solutions.  Returning NaN keeps
    % terr (the real residual, not Inf) available to invariant 3 while making
    % the bad point impossible to mistake for a good one.
    vrx = NaN;  vry = NaN;  vrz = NaN;  lamout = NaN;
    return
end

if SWITCH1 == 0, ANGLE = HALFPI-GAMMA;  else, ANGLE = GAMMA-HALFPI;  end
SINA = sin(ANGLE);  COSA = cos(ANGLE);
V1X = XT;  V1Y = YT;  V1Z = ZT;  V2X = XF;  V2Y = YF;  V2Z = ZF;
MAG1 = sqrt(V1X*V1X+V1Y*V1Y+V1Z*V1Z);
DOTMAG = V1X*V2X+V1Y*V2Y+V1Z*V2Z;
CROSSX = V1Y*V2Z-V1Z*V2Y;  CROSSY = V1Z*V2X-V1X*V2Z;  CROSSZ = V1X*V2Y-V1Y*V2X;
CROSSMAG = sqrt(CROSSX*CROSSX+CROSSY*CROSSY+CROSSZ*CROSSZ);
C2 = MAG1*SINA/CROSSMAG;
C1 = COSA/MAG1 - DOTMAG*C2/(MAG1*MAG1);
VUNITX = C1*V1X + C2*V2X;
VUNITY = C1*V1Y + C2*V2Y;
VUNITZ = C1*V1Z + C2*V2Z;
vrx = RV0MAG*VUNITX;  vry = RV0MAG*VUNITY;  vrz = RV0MAG*VUNITZ;
% lambda = r0*|v|^2/gm, recomputed from the RETURNED velocity.  See the
% near-parabolic section in the file header for what this is and, just as
% importantly, for what it is NOT here: unlike Listing 13.2, THIS routine has
% an explicit hyperbolic branch, so lambda above 2 is a legitimate answer.
lamout = R0MAG*(vrx^2 + vry^2 + vrz^2)/GM;
end


% =============================================================================
function m = maxFinite(v)
v = v(isfinite(v));
if isempty(v), m = NaN; else, m = max(v); end
end


% =============================================================================
function m = minFinite(v)
v = v(isfinite(v));
if isempty(v), m = NaN; else, m = min(v); end
end


% =============================================================================
function DISTKM = distance3dkm(XT, YT, ZT, XF, YF, ZF, A)
%DISTANCE3DKM  Listing 28.2's great-circle distance at radius A, in km.
R = sqrt(XT^2+YT^2+ZT^2);  RF = sqrt(XF^2+YF^2+ZF^2);
CBETA = (XT*XF+YT*YF+ZT*ZF)/(R*RF);
if CBETA <= 1.
    BETA = acos(max(CBETA,-1));  DISTKM = A*BETA/3280.;
else
    DISTKM = (XF-XT)/3280.;
end
end


% =============================================================================
function [gam_deg, apo_km] = elements(x, y, vx, vy, GM, A)
%ELEMENTS  Flight-path angle and apogee ALTITUDE from one position/velocity
%   pair, in closed form.  No integration: this is what lets the lofted-vs-
%   direct row of Fig. 32.4 be scored for the price of a Lambert call.
r = sqrt(x^2+y^2);  v = sqrt(vx^2+vy^2);
gam_deg = 57.3*asin(max(min((x*vx + y*vy)/(r*v), 1), -1));
En = v*v/2 - GM/r;
if En >= 0
    apo_km = Inf;  return          % escape: no apogee
end
h = abs(x*vy - y*vx);
a = -GM/(2*En);
e = sqrt(max(1 + 2*En*h*h/(GM*GM), 0));
apo_km = (a*(1+e) - A)/3280.;
end


% =============================================================================
function K = structuralChecks(R, XT0, YT0, XF, YF, TS, H, GM, A, TFTOT, ...
                              RDESKM, TLAUNCH, dTF, lamErr, lamIt, lamFail)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.
K = struct();
names = {};  oks = [];  detail = {};

% 1  Lambert + RK2 + the tFME formula, closed as a loop: the target must land
%    RDESKM downrange at radius A after TFTOT seconds.
kend = round(TFTOT/TS);
kend = max(1, min(kend, numel(R.target.t)));
xe = R.target.x(kend);  ye = R.target.y(kend);
dkm  = distance3dkm(xe, ye, 0., XT0, YT0, 0., A);
altk = (sqrt(xe^2+ye^2) - A)/3280.;
names{end+1} = 'target lands RDESKM downrange at zero altitude';
oks(end+1)   = abs(dkm - RDESKM) <= 10. && abs(altk) <= 5.;
detail{end+1} = sprintf('range %.3f km (want %g), altitude %+.3f km', ...
                        dkm, RDESKM, altk);

% 2  ⭐ THE ONE THAT LICENSES THE RESTRUCTURING.  predict32 run VERBATIM, as
%    its own loop with its own Lambert call, against the sample the sweep used.
tf1 = TLAUNCH + 60.;
[px, py] = predict32(0., XT0, YT0, tf1, TFTOT, XF, YF, GM, H);
k1 = round(tf1/TS);
d1 = hypot(px - R.target.x(k1), py - R.target.y(k1));
names{end+1} = 'sampled pass == predict32 run verbatim';
oks(end+1)   = d1 <= 1e-6;
detail{end+1} = sprintf('|dr| = %.3e ft at TF = %g s (same ops, same order)', ...
                        d1, tf1);

% 3  Every Lambert call met the routine's OWN convergence test.
%
%    ⭐⭐ THIS ROW IS RED AT THE DEFAULTS AND THAT IS THE RESULT, not a
%    tolerance to be relaxed.  Diagnosed 2026-08-24, and all three of the usual
%    explanations were tested and REFUTED:
%      * not a mis-transcription -- the three bracket updates in LAMBERT3D
%        match zarchan_decoded.txt:10440/:10442/:10443 character for character;
%      * not an infeasible geometry -- a bisection on the routine's OWN T(gamma)
%        finds a unique solution at gamma = 74.7798 deg, |v| = 5.2518 km/s,
%        which sits between the 1240 s (5.2519) and 1280 s (5.2540) neighbours;
%      * not a cap that is too low -- 100 is the BOOK's cap (:10450) and caps of
%        200 / 1000 / 100000 return the identical residual, because GAMMA is a
%        fixed point from iteration 3 on.
%    The cause is an erratum in Listing 28.2; see the LAMBERT3D docstring.
%    ⚠️ The band is the routine's own convergence test.  Do not widen it, and do
%    not raise the cap: both would print green over a solver that still does not
%    solve.  The row goes GREEN on any sweep that never asks the collapsing
%    geometry -- verified by moving the grid off it -- so it is a live row, not
%    a permanent red.
%    ⚠️ THREE detectors, deliberately not one.  The residual, the iteration
%    count and the per-call convergence flag are all checked, because a fault
%    that disables the FLAG (the obvious way to make this row green) still has
%    to get past the other two.  Verified by injection 2026-08-24: forcing
%    conv = true gives "0 of 237 reported failure" and the row stays RED on the
%    residual, with the wrong 16.3434 km/s visible again in R.vbom_kms.
lamOk = lamErr <= 1e-8 && lamIt <= 100 && isempty(lamFail);
names{end+1} = 'every Lambert call converged inside 100 iterations';
oks(end+1)   = lamOk;
if ~isempty(lamFail)
    detail{end+1} = sprintf('%d of %d calls FAILED -- %s', numel(lamFail), ...
        R.p.lambert_calls, lamFailStr(lamFail));
elseif lamOk
    detail{end+1} = sprintf(['worst |TF-T|/TF = %.3e, worst iteration count ' ...
        '%d, 0 of %d calls failed'], lamErr, lamIt, R.p.lambert_calls);
else
    % Cannot happen at the defaults: conv is DERIVED from terr, so a residual
    % over the band implies a recorded failure.  If it ever prints, the
    % convergence flag has been decoupled from the residual it is made of.
    detail{end+1} = sprintf(['INCONSISTENT: 0 of %d calls reported failure, ' ...
        'yet worst |TF-T|/TF = %.3e and worst iteration count %d are outside ' ...
        'the routine''s own test -- the conv flag no longer follows the ' ...
        'residual'], R.p.lambert_calls, lamErr, lamIt);
end

% 4  Specific orbital energy along the pass.  Heun is not symplectic, so this
%    drifts, and the drift is O(h^2) -- so the tolerance is too, anchored at
%    1e-8 for the listing's H = .01 s.  Against a ~3990 s orbital period the
%    drift predicted there is of order 1e-11, i.e. the band is ~3 decades wide
%    and this is a real check rather than a formality.  A FIXED 1e-8 would have
%    turned into a false alarm the moment anyone coarsened H.
r  = hypot(R.target.x, R.target.y);
v2 = R.target.xd.^2 + R.target.yd.^2;
En = v2/2 - GM./r;
edrift = max(abs(En - En(1)))/abs(En(1));
etol   = 1e-8*(H/0.01)^2;
names{end+1} = 'target specific energy conserved along the pass';
oks(end+1)   = edrift <= etol;
detail{end+1} = sprintf('max relative drift %.3e over %d samples (band %.3e)', ...
                        edrift, numel(En), etol);

% 5  The sweep grid is the listing's own (:12201, prose :12199).
want = (TLAUNCH+60.):dTF:(TFTOT-20.);
names{end+1} = 'sweep grid = (TLAUNCH+60):dTF:(TFTOT-20)';
oks(end+1)   = numel(R.tf) == numel(want) && max(abs(R.tf - want)) < 1e-9;
detail{end+1} = sprintf('%d points, %g -> %g s in steps of %g s', ...
                        numel(R.tf), R.tf(1), R.tf(end), dTF);

K.names = names;
K.oks   = logical(oks(:).');
K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    b = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{b}, detail{b});
end
end


% =============================================================================
function B = benchmarks(R, TFTOT_ME, TLAUNCH, RDESKM)
%BENCHMARKS  Ten rows.  Every book value was grepped out of the DECODED file;
%   the line number is in .source.  Nothing here was read off a figure by this
%   port -- where the BOOK read a figure and said so in prose, the row carries
%   the reading precision in its band and says that too.
%
%   ⚠️ EVERY published number in Ch.32's two-dimensional section belongs to ONE
%   scenario: a 10,000-km minimum-energy ICBM, interceptor launched at 300 s.
%   Off that scenario the rows are SKIPPED, not scored against numbers that no
%   longer describe the run.  Scoring 7.19 km/s against a 5000-km trajectory
%   would still print a percentage.
B = emptyBench();
onRange = abs(RDESKM - 10000.) < 1e-9;
has9000 = any(abs(R.zkm - 9000.) < 1e-9);
onGeom  = onRange && abs(TLAUNCH - 300.) < 1e-9 && has9000;
offRangeWhy = sprintf(['RDESKM = %g km, but every published number in this ' ...
    'section is for the 10,000-km trajectory.'], RDESKM);
offGeomWhy = sprintf(['needs the book''s scenario: RDESKM = 10000 (got %g), ' ...
    'TLAUNCH = 300 s (got %g) and z = 9000 km in the sweep (%s).'], ...
    RDESKM, TLAUNCH, yn(has9000));

% 0  ⭐⭐ AN ERRATUM IN THE BOOK, reported and NOT adjudicated.  There is no
%    published number to score it against -- Zarchan does not say his Lambert
%    routine fails, and the sweep he plots in Fig. 32.3 is 78 points per curve
%    where this port's is 78 too, so whatever he got at the collapsing point he
%    plotted.  INFO is the honest kind: computed, stated, not turned into a
%    pass or a fail.  Structural invariant 3 is where it costs something.
B = addInfo(B, 'Listing 28.2 LAMBERT3D bracket collapse (book erratum)', ...
    ['13.2 (:3948-:3949) updates the bracket with a mutually exclusive if/else; ' ...
     '28.2 (:10440/:10442/:10443) uses three un-elsed ifs and can set G_MIN = G_MAX'], ...
    sprintf('%d of %d Lambert calls did not converge -- %s', ...
        R.p.n_lambert_failed, R.p.lambert_calls, lamFailStr(R.p.lambert_failed)), ...
    'zarchan.txt:10440/:10442/:10443 vs :3948-:3949; the cap :10450', '-', ...
    ['the second and third if both fire when the flight time comes back ' ...
     'negative, collapsing the bracket to a point; GAMMA is then a fixed ' ...
     'point until the book''s own 100-iteration cap.  Not repaired: this port ' ...
     'reproduces the listing.  A caller that needs the point can bisect the ' ...
     'routine''s own T(gamma), which has a unique root there.']);

if ~onRange
    B = addSkip(B, 'flight time, 10,000-km minimum-energy ICBM', '1938 s', ...
        offRangeWhy, 'zarchan.txt:12157, :12185', 's');
    B = addSkip(B, 'TLOFT shifts the flight time one-for-one', ...
        '2438 / 1738 s', offRangeWhy, 'zarchan.txt:12186, :12187', 's');
    B = addSkip(B, 'target burnout velocities (3 trajectories)', ...
        '7.19 / 7.33 / 7.23 km/s', offRangeWhy, 'zarchan.txt:12172', 'km/s');
else

% 1  the time-of-flight formula, evaluated at the book's own inputs
B = addVal(B, 'flight time, 10,000-km minimum-energy ICBM', 1938., R.tftot_me, ...
    '0.05 s absolute', 0.05/1938., ...
    'formula zarchan.txt:12157; Fig. 32.1 annotation :12185', 's', ...
    '252 + 0.223*10000 - 5.44e-6*1e8 = 252 + 2230 - 544');

% 2  TLOFT shifts it one-for-one
lo = TFTOT_ME + 500.;  de = TFTOT_ME - 200.;
B = addShape(B, 'TLOFT shifts the flight time one-for-one', ...
    'lofted 2438 s, depressed 1738 s', sprintf('%.1f / %.1f s', lo, de), ...
    abs(lo-2438.) <= 0.05 && abs(de-1738.) <= 0.05, ...
    'Fig. 32.1 annotations zarchan.txt:12186, :12187', ...
    'the listing does TFTOT = TFTOT + TLOFT, so this is arithmetic, not physics');

% 3-5  the required TARGET burnout velocities, three trajectories, prose
B = addVal(B, 'target burnout velocity, minimum energy', 7.19, R.vbot_me_kms, ...
    '0.005 km/s = half the last printed digit', 0.005/7.19, ...
    'zarchan.txt:12172', 'km/s', ...
    'one LAMBERT3D call over TFTOT_ME; no integration is involved');
B = addVal(B, 'target burnout velocity, lofted (TLOFT=500)', 7.33, ...
    R.vbot_loft_kms, '0.005 km/s', 0.005/7.33, 'zarchan.txt:12172', 'km/s', ...
    'same call at TFTOT = 2438 s');
B = addVal(B, 'target burnout velocity, depressed (TLOFT=-200)', 7.23, ...
    R.vbot_depr_kms, '0.005 km/s', 0.005/7.23, 'zarchan.txt:12172', 'km/s', ...
    'same call at TFTOT = 1738 s');
end

if ~onGeom
    B = addSkip(B, 'required interceptor velocity, z=9000 km (2 points)', ...
        '5.3 km/s at 1400 s, 6.5 km/s at 800 s', offGeomWhy, ...
        'zarchan.txt:12239, :12241', 'km/s');
    B = addSkip(B, 'short flights cost most; z=9000 bottoms out near 1400 s', ...
        'shape of Fig. 32.3', offGeomWhy, 'zarchan.txt:12239', '-');
    B = addSkip(B, 'the 1400-s interceptor is lofted, the 800-s one is not', ...
        'shape of Fig. 32.4', offGeomWhy, 'zarchan.txt:12241', '-');
    B = addInfo(B, 'the success gate, and what was not run', ...
        'VBOM < 8 km/s, miss < 1000 ft, intercept altitude > 50 km', ...
        gateStr(R, TLAUNCH), 'zarchan.txt:12194', '-', ...
        'reported at off-book parameters; nothing above it was scored');
    return
end

% 6-7  the two points Fig. 32.4 names, z = 9000 km.  BAND PROVENANCE: the
%      abscissas are stated flatly (":12241", and the figure annotations
%      :12269/:12270), so only the ORDINATE is a figure reading -- Fig. 32.3's
%      y axis is labelled 0/2/4/6/8 (:12242-:12246) and the author quoted one
%      decimal, so 0.15 km/s is ~1.5x the finest credible read.  It is not
%      widened past that; a red row here is a result.
[v14, d14] = curveAt(R, 9000., 1400.);
B = addVal(B, 'required interceptor velocity, z=9000 km, 1400 s flight', 5.3, ...
    v14, '0.15 km/s (figure read to 0.1)', 0.15/5.3, ...
    'zarchan.txt:12239 and :12241; Fig. 32.4 annotation :12270', 'km/s', d14);
[v08, d08] = curveAt(R, 9000., 800.);
B = addVal(B, 'required interceptor velocity, z=9000 km, 800 s flight', 6.5, ...
    v08, '0.15 km/s (figure read to 0.1)', 0.15/6.5, ...
    'zarchan.txt:12241; Fig. 32.4 annotation :12269', 'km/s', d08);

% 8  the SHAPE of Fig. 32.3, in two clauses
[shapeOk, shapeStr, shapeDet] = fig323Shape(R);
B = addShape(B, 'short flights cost most; z=9000 bottoms out near 1400 s', ...
    ['max at the shortest flight time for every z; 5.3 km/s quoted as the ' ...
     'z=9000 minimum at ~1400 s'], shapeStr, shapeOk, ...
    'zarchan.txt:12239', shapeDet);

% 9  Fig. 32.4's actual claim: the LONG flight is the lofted one
[lofOk, lofStr, lofDet] = loftShape(R);
B = addShape(B, 'the 1400-s interceptor is lofted, the 800-s one is not', ...
    'apogee(1400 s flight) > apogee(800 s flight)', lofStr, lofOk, ...
    'zarchan.txt:12241', lofDet);

% 10  the success gate, reported and NOT adjudicated
B = addInfo(B, 'the success gate, and what was not run', ...
    'VBOM < 8 km/s, miss < 1000 ft, intercept altitude > 50 km', ...
    gateStr(R, TLAUNCH), 'zarchan.txt:12194', '-', ...
    ['the 1000-ft miss gate needs the engagement integration: 78 runs x ' ...
     '~1.1e5 RK2 steps ~ 9e6 steps, for a miss the book itself calls ' ...
     'unreliable at this step size.  Two named cases are run instead.']);
end


% =============================================================================
function [v, d] = curveAt(R, zkm, flight)
%CURVEAT  One point of the sweep, with its two neighbours reported so that the
%   local slope -- i.e. how much the book's abscissa ambiguity could be worth
%   -- is visible next to the number rather than assumed away.
v = NaN;  d = 'launch separation or flight time not on the sweep grid';
j = find(abs(R.zkm - zkm) < 1e-9, 1);
i = find(abs(R.flight_s - flight) < 1e-9, 1);
if isempty(j) || isempty(i), return; end
v = R.vbom_kms(j,i);
if i > 1
    lo = sprintf('%.3f', R.vbom_kms(j,i-1));
else
    lo = 'n/a';
end
if i < numel(R.flight_s)
    hi = sprintf('%.3f', R.vbom_kms(j,i+1));
else
    hi = 'n/a';
end
d = sprintf('neighbours at %+g s: %s / %s km/s; target altitude at TF %.1f km', ...
            R.p.dTF, lo, hi, R.alt_int_km(i));
end


function [ok, s, d] = fig323Shape(R)
%FIG323SHAPE  Clause 1 on the UNGATED curve (the gate removes a prefix of short
%   flight times, so the plotted curve's left edge is where VBOM drops under
%   8 km/s -- the claim is about the rise, which is ungated).  Clause 2 is
%   stated as "1400 s is at the bottom" rather than "argmin == 1400 s",
%   because a flat minimum makes argmin jitter and flatness is the point.
ok1 = true;  parts = {};
for j = 1:numel(R.zkm)
    c = R.vbom_kms(j,:);
    good = isfinite(c);
    ok1 = ok1 && any(good) && (c(1) >= max(c(good)) - 1e-12);
    parts{end+1} = sprintf('z=%g: %.2f -> min %.3f km/s', R.zkm(j), c(1), ...
                           min(c(good))); %#ok<AGROW>
end
ok2 = false;  ratio = NaN;
j = find(abs(R.zkm - 9000.) < 1e-9, 1);
i = find(abs(R.flight_s - 1400.) < 1e-9, 1);
if ~isempty(j) && ~isempty(i)
    c = R.vbom_kms(j,:);
    g = R.gate(j,:) & isfinite(c);
    if any(g) && isfinite(c(i))
        ratio = c(i)/min(c(g));
        ok2 = ratio <= 1.02;
    end
end
ok = ok1 && ok2;
s  = sprintf('%s | 1400 s sits %.2f%% above the gated z=9000 minimum', ...
             strjoin(parts, ', '), 100*(ratio-1));
d  = sprintf(['clause 1 (max at the shortest flight) %s; clause 2 (1400 s ' ...
              'within 2%% of the gated minimum) %s'], tf2s(ok1), tf2s(ok2));
end


function [ok, s, d] = loftShape(R)
%LOFTSHAPE  Apogee from the Lambert solution's orbital elements -- see
%   ELEMENTS.  Both interceptors leave the same point, so this is a comparison
%   of two trajectories and not of two geometries.
ok = false;  s = 'z=9000 km not on the sweep';  d = '';
j = find(abs(R.zkm - 9000.) < 1e-9, 1);
i14 = find(abs(R.flight_s - 1400.) < 1e-9, 1);
i08 = find(abs(R.flight_s -  800.) < 1e-9, 1);
if isempty(j) || isempty(i14) || isempty(i08), return; end
a14 = R.apogee_km(j,i14);  a08 = R.apogee_km(j,i08);
g14 = R.gamma_deg(j,i14);  g08 = R.gamma_deg(j,i08);
ok = isfinite(a14) && isfinite(a08) && a14 > a08;
s  = sprintf('apogee %.0f km (1400 s) vs %.0f km (800 s)', a14, a08);
d  = sprintf(['initial flight-path angle %.1f deg vs %.1f deg; both from the ' ...
              'same launch point, closed form, no integration'], g14, g08);
end


function s = gateStr(R, TLAUNCH)
n = 0;  tot = 0;  parts = {};
for j = 1:numel(R.zkm)
    g = R.gate(j,:) & isfinite(R.vbom_kms(j,:));
    n = n + sum(g);  tot = tot + numel(g);
    if any(g)
        parts{end+1} = sprintf('z=%g: %d pts, flight %g-%g s', R.zkm(j), ...
            sum(g), min(R.flight_s(g)), max(R.flight_s(g))); %#ok<AGROW>
    else
        parts{end+1} = sprintf('z=%g: none', R.zkm(j)); %#ok<AGROW>
    end
end
e = '';
for k = 1:numel(R.engage)
    E = R.engage(k);
    % NOT built inside [ ] directly: whitespace is significant between square
    % brackets, so a continued sprintf there is a parse error, not a concat.
    one = sprintf(['; engaged TF=%g s (flight %g s): miss %.1f ft, intercept ' ...
        'altitude %.1f km, VBOM %.3f km/s%s'], E.tf, E.flight_s, E.miss_ft, ...
        E.alt_int_km, E.vbom_kms, tmoStr(E.timeout));
    e = [e one]; %#ok<AGROW>
end
s = sprintf('%d of %d sweep points pass the two evaluable gates (%s)%s', ...
            n, tot, strjoin(parts, ', '), e);
if TLAUNCH ~= 300., s = [s ' (off-book TLAUNCH)']; end
end

function s = tmoStr(t)
if t, s = ' [TIMED OUT -- see tmax_pad]'; else, s = ''; end
end

function s = tf2s(b)
if b, s = 'holds'; else, s = 'VIOLATED'; end
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER are benchSuite's adaptGeneric schema.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.9g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.9g', ours);
B(i).band = band;  B(i).tol = tol;
B(i).units = units;  B(i).source = source;  B(i).detail = detail;
if isempty(ours) || ~isfinite(ours) || book == 0
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

function B = addInfo(B, what, bookStr, oursStr, source, units, why)
%ADDINFO  Computed and REFUSED adjudication.  Never a pass, never a fail.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'info';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'info';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = false;  B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = why;
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

function s = yn(b)
if b, s = 'present'; else, s = 'absent'; end
end

function F = lf(where, tf, iters, terr)
%LF  One non-converged Lambert call, in the shape lamFail collects.
F = struct('where', where, 'zkm', NaN, 'tf', tf, 'flight_s', tf, ...
           'theta_deg', NaN, 'iters', iters, 'terr', terr);
end

function s = lamFailStr(F)
%LAMFAILSTR  The failing calls, each with the geometry that produced it.  A
%   count alone sends the next reader back to the debugger.
%
%   Listed in full up to SHOWN, then counted.  The cap is on the PRINTING only;
%   R.p.lambert_failed always carries every one.  A 105-failure injection
%   produced a 10-kB invariant line, which is a different way of hiding.
SHOWN = 6;
if isempty(F), s = 'none';  return;  end
n = numel(F);
more = '';
if n > SHOWN
    more = sprintf(' (+%d more; all of them in R.p.lambert_failed)', n - SHOWN);
    F = F(1:SHOWN);
end
p = cell(1, numel(F));
for k = 1:numel(F)
    if isnan(F(k).zkm)
        p{k} = sprintf('%s at TF %g s (%d iters, |TF-T|/TF %.4g)', ...
            F(k).where, F(k).tf, F(k).iters, F(k).terr);
    else
        p{k} = sprintf(['z=%g km, TF %g s (flight %g s, theta %.3f deg): ' ...
            '%d iters, |TF-T|/TF %.4g'], F(k).zkm, F(k).tf, F(k).flight_s, ...
            F(k).theta_deg, F(k).iters, F(k).terr);
    end
end
s = [strjoin(p, '; ') more];
end


% =============================================================================
function s = render(R, ev)
L = {};
L{end+1} = 'Zarchan Listing 32.2 -- TWO-DIMENSIONAL KINEMATIC MULTIPLE-RUN ENGAGEMENT';
L{end+1} = sprintf('   %s', R.units);
L{end+1} = '   "MC" = MULTIPLE-CASE.  The listing draws no random numbers; nothing is seeded.';
L{end+1} = sprintf(['   TFTOT = %.1f s (minimum energy %.1f + TLOFT %g), ' ...
    'target burnout %.4f km/s'], R.tftot, R.tftot_me, R.p.TLOFT, R.vbot_kms);
L{end+1} = sprintf(['   sweep %g:%g:%g s (%d points) x %d launch separations; ' ...
    'H = %g s, %d point(s) skipped as near-radial'], R.tf(1), R.p.dTF, ...
    R.tf(end), numel(R.tf), numel(R.zkm), R.p.H, R.p.n_radial_skipped);
L{end+1} = sprintf('   Lambert: %d call(s), %d did not converge -- %s', ...
    R.p.lambert_calls, R.p.n_lambert_failed, lamFailStr(R.p.lambert_failed));
if R.p.n_lambert_failed > 0
    L{end+1} = ['   ^ book erratum in Listing 28.2, not a tolerance: those ' ...
                'points are NaN, never a number.'];
end
L{end+1} = '';
L{end+1} = '   flight [s]   required interceptor velocity [km/s], by launch separation';
hdr = '      ';
for j = 1:numel(R.zkm), hdr = [hdr sprintf('  z=%-8g', R.zkm(j))]; end %#ok<AGROW>
L{end+1} = hdr;
step = max(1, round(numel(R.tf)/12));
for i = 1:step:numel(R.tf)
    ln = sprintf('   %8g   ', R.flight_s(i));
    for j = 1:numel(R.zkm)
        if R.gate(j,i), mk = '*'; else, mk = ' '; end
        ln = [ln sprintf('  %8.3f%s', R.vbom_kms(j,i), mk)]; %#ok<AGROW>
    end
    L{end+1} = ln; %#ok<AGROW>
end
L{end+1} = '   (* = passes the two gates evaluable without the engagement loop)';
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-52s %s', vstr(K.oks(i)), K.names{i}, K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning';
    L{end+1} = '            NOT CHECKED, which is not the same thing as "fine".';
else
    L{end+1} = 'BENCHMARKS  (book values grepped from zarchan_decoded.txt; lines in .source)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
            case 'skip'
                L{end+1} = sprintf('   SKIP %-52s want %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.why); %#ok<AGROW>
            case 'info'
                L{end+1} = sprintf('   INFO %-52s %s', b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.why); %#ok<AGROW>
            case 'shape'
                L{end+1} = sprintf('   %-4s %-52s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf(['   %-4s %-52s book %s %s / ours %s %s ' ...
                    '(%+.4f%%, band %.4g%%)'], vstr(b.ok), b.what, b.book_str, ...
                    b.units, b.ours_str, b.units, b.pct, 100*b.tol); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d info/skipped -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: KINEMATICS ONLY.  Impulsive burns, point masses, no drag, no';
L{end+1} = '   thrust profile, no autopilot, no seeker, no guidance law after launch,';
L{end+1} = '   and a round non-rotating Earth (QEARTH = 0).  This listing answers';
L{end+1} = '   "how fast must the interceptor be", not "can it be guided there".';
s = strjoin(L, newline);
end

function s = vstr(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end
