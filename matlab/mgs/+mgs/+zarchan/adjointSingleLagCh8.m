function R = adjointSingleLagCh8(opts)
%MGS.ZARCHAN.ADJOINTSINGLELAGCH8  Zarchan Listing 8.1 -- PN vs AUGMENTED PN
%   miss adjoint for a single-lag guidance system.
%
%   R = mgs.zarchan.adjointSingleLagCh8()              book case, prints, checks
%   R = mgs.zarchan.adjointSingleLagCh8('TAU', 2.0)    sweep the guidance lag
%   R = mgs.zarchan.adjointSingleLagCh8('APN', 1)      augmented PN alone
%   R = mgs.zarchan.adjointSingleLagCh8('render',false,'assert',false)
%
%   Line-by-line port of Listing 8.1 (zarchan.txt:2463-2470), the adjoint of
%   the Fig. 8.11 homing loop, integrated by the book's own second-order
%   Runge-Kutta (Heun) scheme.  Fig. 8.11's caption is the whole point of the
%   listing: "we have proportional navigation if APN = 0 and augmented
%   proportional navigation if APN = 1" (:2451).  The book ran it twice
%   (:2461) and plotted the pair as Fig. 8.12, so this port runs BOTH by
%   default and hands back both columns.
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, g = 32.2 ft/s^2.  NOT SI.
%
%   The rest of mgs is metres.  This file is deliberately NOT converted, so
%   every number it prints can be laid straight against a printed figure with
%   no factor in between to be got wrong.  A silent ft->m here would turn 12 ft
%   of miss into "3.7" and it would still look like a perfectly reasonable miss
%   distance.  Convert AT THE CALL SITE and say so there.
% ===========================================================================
%
% ---------------------------------------------------------------------------
%   *** READ THIS FIRST: HOW THIS RELATES TO MGS.ZARCHAN.ADJOINTSINGLELAG ***
%
%   Listing 3.1 (Ch.3) and Listing 8.1 (Ch.8) are NOT the same program.  They
%   are also not independent.  Both statements matter, so both are proved here
%   rather than asserted.
%
%   THEY DIFFER AS CODE.  Side by side, Listing 3.1 (:760) and Listing 8.1
%   (:2467):
%
%     3.1:  X1D = X2;                    8.1:  X1D = X2 + X4*XNP*APN/(2*TAU);
%           X2D = X3;                          X2D = X3 + XNP*X4/(TAU*TP);
%           Y1  = (X4-X2)/TAU;                 X3D = XNP*X4/(TAU*TP*TP);
%           TGO = TP + .00001;                 X4D = -X4/TAU - X2;
%           X3D = XNP*Y1/TGO;
%           X4D = -Y1;
%
%   Three differences, and only one of them is cosmetic:
%     (a) Listing 8.1 carries the APN feed-forward term on X1D.  That term is
%         NEW; nothing in Ch.3 has it.
%     (b) the two use DIFFERENT STATE REALISATIONS of the same loop -- 8.1
%         spreads the PN gain over X2D and X3D as N'/tgo and N'/tgo^2, where
%         3.1 funnels everything through the single node Y1.
%     (c) 8.1 divides by TP directly where 3.1 forms TGO = TP + 1e-5 first.
%         Cosmetic: TP already starts at 1e-5, so neither divides by zero.
%
%   THEY AGREE AS SYSTEMS, at APN = 0.  This is the part worth proving,
%   because "port it twice and hope" is how twins drift.  Differentiate 8.1's
%   x2 equation and the time-varying gain cancels exactly:
%
%       x2'  = x3 + N' x4/(T t)
%       x2'' = x3' + N' x4'/(T t) - N' x4/(T t^2)
%            = N' x4/(T t^2) + N' x4'/(T t) - N' x4/(T t^2)
%            = N' x4'/(T t)
%
%   Listing 3.1 says x2'' = x3' = N'(x4 - x2)/(T tgo).  The two are the same
%   equation iff  x4' = x4^(3.1) - x2, i.e. iff
%
%       x4^(3.1) = -x4^(8.1) / T
%
%   and that mapping also satisfies 3.1's own x4 equation identically.  Both
%   start from x1 = x2 = x4 = 0, x3 = 1, so the two programs produce the SAME
%   x1(t) and x2(t) -- hence the same miss curves -- at APN = 0.
%
%   *** THE TWO ARE INDISTINGUISHABLE ON THE BOOK'S DEFAULT RUN (TAU = 1). ***
%   Every candidate mapping x4^(3.1) = -x4^(8.1)*T^k collapses to the same
%   thing at T = 1.  If you ever want to check the power of T in that
%   relationship, you MUST do it at TAU ~= 1.  The T^2-normalization benchmark
%   below is the check that can see it; it exists for that reason.
%
%   CONSEQUENCE FOR THIS FILE.  It is NOT a wrapper.  A wrapper could not
%   produce the APN column, which is the entire content of Ch.8, and the twin
%   cannot check that column (see the fault table below: every APN-specific
%   fault is INERT in the PN column).  So the APN=0 column is computed here and
%   then CROSS-CHECKED against mgs.zarchan.adjointSingleLag -- benchmark 4 --
%   which is what stops the pair drifting apart without a wrapper's cost.
%
% ---------------------------------------------------------------------------
%   WHAT AUGMENTED PN IS, AND WHY THE TERM LANDS ON x1
%
%   The book's law, printed at :11556:  nc|APN = N' Vc lambda-dot + 0.5 N' nT.
%   Equivalently, from the zero-effort-miss form at :2286 and :6264,
%
%       nc = N'( y + ydot*tgo + 0.5*APN*nT*tgo^2 ) / tgo^2
%
%   With APN = 0 that is exactly N' Vc lambda-dot, i.e. the Ch.3 loop; the
%   augmentation is the constant N'/2 feed-forward of measured target
%   acceleration.  Constant, because 0.5*N'*nT*tgo^2/tgo^2 has no tgo left in
%   it -- which is why the adjoint image of that path (X4*XNP*APN/(2*TAU)) is
%   the ONE term in the whole listing with no 1/TP in it.  A 1/TP creeping into
%   that term is a plausible-looking typo; it would still integrate.
%
%   The term lands on X1D and nowhere else, and that has a consequence worth
%   stating because it is also a test:
%
%     *** THE HEADING-ERROR CHANNEL IS IDENTICAL FOR PN AND APN. ***
%
%   x2, x3 and x4 never reference APN or x1, so MHE is bit-for-bit the same in
%   both columns -- not "close", EXACTLY equal.  Physically that is obvious in
%   hindsight (with no target maneuver there is nothing to feed forward) and it
%   is checked as an exact-zero benchmark, because a stray coupling from x1
%   back into the loop would break it on the first step.
%
% ---------------------------------------------------------------------------
%   THE OUTPUTS ARE SENSITIVITIES.  YOU MUST MULTIPLY THEM.
%
%     R.x1(:,j)  [s^2]  ->  miss from a step target maneuver   =  x1 * nT
%     R.x2(:,j)  [s]    ->  miss from an initial heading error =  x2 * (-VM*HE)
%
%   Both products are formed for you as R.miss_nt_ft and R.miss_he_ft.  The raw
%   sensitivities are returned because the loop is linear: doubling nT doubles
%   the miss and nothing has to be recomputed.
%
%   The heading-error scale factor is -VM*HE, NOT 2*VM*HE.  The Ch.3 prose at
%   :749 reads "multiply by 2VM HE"; that "2" is an OCR'd minus sign, and the
%   same corruption appears at :750 as "220 deg of heading error" for a listing
%   whose own data statement says HEDEG = -20.  Listing 8.1 itself (:2469)
%   computes ArrayXMHE(n) = -VM*HE*X2.  The listing wins.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Nine rows.  A failure ERRORS unless 'assert' is false.
%
%   1  PN column vs the p.55 CLOSED FORM the book prints (:846-849, :888-895):
%          Miss/nT |N'=4  =  tF^2 e^(-tF/T) (0.5 - tF/(6T))
%      Reconstructed for general integer N' >= 3 from 1-H(s) = [s/(s+1/T)]^N'
%      (:828-831) and MNT/nT = (1-H)/s^3 (:833-839).  Benchmark 8 proves the kernel
%      reproduces ALL of the book's printed forms exactly, so this is the
%      book's own formula and not a lookalike.
%      Measured residual at the book's H = 0.01: 2.53% of peak.
%
%   2  APN column vs the SAME closed form EXTENDED -- see the derivation
%      below.  *** THIS ONE IS DERIVED, NOT PRINTED IN THE BOOK. ***  Zarchan
%      gives no closed form for the lagged APN miss; Fig. 8.12 is a picture,
%      and the extraction turned every figure into an image link, so there is
%      no APN number anywhere in the text to grep.  What makes the derived form
%      trustworthy is not the algebra, it is that it reduces to the book's
%      printed form term-for-term at APN = 0 (checked, benchmark 8) and that
%      the integrated adjoint converges ONTO it at the same rate and to the
%      same 2.53%-of-peak residual as the PN column does onto the book's.
%      A wrong closed form does not get converged onto.
%
%      DERIVATION (all of it, so it can be audited).  Only x1 sees APN, so
%      with c = N'*APN/(2T),  x1' = x2 + c x4  and  x4' = -x4/T - x2:
%          X4(s) = -X2(s)/(s + a),          a = 1/T
%          X1(s) =  X2(s)/s + c X4(s)/s  =  X2/s - c X2/(s(s+a))
%      and X2(s) = (1-H)/s^2 = s^(N'-2)/(s+a)^N' from the book, so
%
%        MNT/nT (s) = s^(N'-3)/(s+a)^N' - (N' a APN/2) s^(N'-3)/(s+a)^(N'+1)
%
%      Both terms are of the form s^p/(s+a)^(p+n0), inverted exactly by
%      CFKERNEL below.  At APN = 0 the second term vanishes and the first is
%      the book's (1-H)/s^3.  At N' = 4, T = 1, APN = 1 it evaluates to
%          MNT/nT = e^-t (t^2/2 - t^3/2 + t^4/12).
%
%   3  Fig. 3.13 / p.47 (:751): "a 3-g maneuver causes nearly 12 ft of miss if
%      the flight time is only 1 s".  That is a CH.3 number and it is used here
%      DELIBERATELY and only on the APN = 0 column, because the equivalence
%      proof above says that column IS the Ch.3 system.  Tolerance +-1 ft: 12
%      is read off a printed graph, not a published digit.  Gated on the book's
%      full nominal parameter set.
%
%   4  APN = 0 column vs mgs.zarchan.adjointSingleLag, the Ch.3 port.  Two
%      different state realisations of one system, integrated separately.
%      Catches drift in either file.  Measured 0.65% of peak at H = 0.01.
%
%   5  MHE identical across APN.  Required EXACTLY zero (see above).
%
%   6  Fig. 8.12 prose (:2461): "Augmented proportional navigation yields
%      smaller miss distances for shorter flight times, whereas proportional
%      navigation yields smaller miss distances for longer flight times."
%      Turned into two sign checks.
%
%      *** THAT SENTENCE IS A SUMMARY, NOT AN EVERYWHERE-STATEMENT, AND THE
%      EXCEPTION IS RECORDED HERE SO NOBODY "DISCOVERS" IT LATER AND FILES A
%      BUG. ***  Evaluated on the EXACT closed forms at N' = 4, T = 1, the
%      ordering |APN| < |PN| flips at tF/T = 2.001, flips back at 4.001, and
%      flips again at 6.000 -- because both curves oscillate through zero (PN
%      crosses at 3.001, APN at 1.268 and 4.733) and whichever is nearer its
%      own zero crossing wins on absolute value there.  The book's claim holds
%      cleanly at the two ENDS, so that is what is checked:
%          short:  0 < tF <= 1.5 T  ->  |APN| < |PN|   (14/14 samples; the
%                                       margin shrinks to 0 as tF->0 by
%                                       O(t^3), so the SIGN is what is asserted)
%          long:      tF >= 6.5 T   ->  |PN| < |APN|   (36/36 samples)
%      Both windows are stated in units of T because benchmark 9 proves the
%      curves depend on tF/T only.  Gated on N' = 4; VM, HE and nT do not
%      enter, since both columns carry the same nT factor.
%
%   7  Prose at :2473: "Generally, larger guidance system time constants yield
%      larger miss distances", and :2471: "in a zero-lag system both guidance
%      laws would always yield zero miss distance".  Checked as strict
%      monotonicity of peak |MNT| over T in {T/8, T/4, T/2, T}, per law.
%
%   8  CFKERNEL vs the four closed forms the book PRINTS, to machine precision:
%        N'=3 maneuver (:1345, :888)  0.5 tF^2 e^(-tF/T)
%        N'=4 maneuver (:846-849)     tF^2 e^(-tF/T) (0.5 - tF/(6T))
%        N'=5 maneuver (:888-895)     tF^2 e^(-tF/T) (0.5 - tF/(3T) + tF^2/(24T^2))
%        N'=4 heading  (:872-876)     -VM tF e^(-tF/T) (1 - tF/T + tF^2/(6T^2))
%      This is the row that makes benchmarks 1 and 2 mean anything: it proves
%      the analytic side is the book's, not something shaped like it.
%
%   9  T^2 NORMALIZATION.  Miss/(nT T^2) is a function of tF/T alone, for BOTH
%      laws (both terms of the derived transform carry T^2 exactly).  Re-run on
%      a matched normalized grid; residual must stay under 1% of the normalized
%      peak.  Measured 0.20%.  THIS IS THE ONLY CHECK THAT CAN SEE A WRONG
%      POWER OF T, because the book's default run has TAU = 1 where every power
%      looks the same.
%
%   TOLERANCE for benchmarks 1 and 2:  max(1% of peak, 6*H).
%   The 6*H slope tracks discretization against the 1/tgo singularity at the
%   start of the run; measured residual is 11.5% at H=0.05, 4.88% at 0.02,
%   2.53% at 0.01, 1.31% at 0.005, 0.30% at 0.001 -- i.e. it falls at roughly
%   H^0.93, giving a steady 2.2x-3.4x margin across that whole range.  The 1%
%   FLOOR is there so that refining H below ~2e-3 cannot make a CORRECT run
%   fail once the residual stops falling as fast as a purely linear band does.
%   Punishing someone for using a smaller step is the worst failure this file
%   could have, so the floor is not negotiable.
%
%   The floor is safe because the check has enormous margin.  Fault injection
%   at H = 0.01, residual as % of peak (correct = 2.53% in both columns):
%
%                                          PN col      APN col
%       impulse moved to x1                   810%         942%
%       impulse moved to x2                  3008%        2917%
%       impulse moved to x4              2.1e+10 %    2.1e+10 %
%       N'x4/(T tgo) dropped from X2D        6907%        5880%
%       X3D uses 1/tgo instead of 1/tgo^2     453%         432%
%       sign flipped on X4D               5.9e+6 %    1.5e+7  %
%       APN term loses the /2                2.53%         158%
%       APN term sign flipped                2.53%         309%
%       APN term dropped entirely            2.53%         154%
%
%   Read the last three rows carefully.  EVERY APN-SPECIFIC FAULT IS EXACTLY
%   INERT IN THE PN COLUMN -- 2.53%, the correct value, to three figures.  That
%   is the whole argument for benchmark 2 existing: without a closed form for
%   the APN column, three distinct ways of getting Ch.8's only new term wrong
%   would leave no trace anywhere in this file.
%
% ---------------------------------------------------------------------------
%   WHAT THIS IS NOT.  A LINEARISED, PLANAR, constant-closing-speed homing loop
%   with a FIRST-ORDER lag.  Our vehicle has gravity, drag, thrust, an
%   asymmetric nz clamp, a LOS floor, and a lag that SATURATES.  Do not read a
%   miss distance out of here and call it a prediction for an aircraft.
%
%   It is, however, the closest thing in mgs to the question Ch.8 asks, and
%   that question is live for us: our measured gamma-tracking gain is 0.063
%   against the model's 1.385, ~8 s of lag, saturating on 26 of 32 sorties.
%   Ch.8's answer -- that the lag costs miss distance and that augmentation
%   does NOT buy it back -- is why "add a feed-forward term" is not a fix for
%   what we have.  Note also that our bullseye is STATIONARY: nT = 0 makes the
%   APN term identically zero, so on our own problem the two columns coincide
%   and the heading-error channel is the only one that speaks.
%
% ---------------------------------------------------------------------------
%   R.t            adjoint time [s] = original-system FLIGHT TIME tF, and also
%                  readable as "time to go at which the maneuver starts"
%   R.APN          1xK laws that were run, in column order (0 = PN, 1 = APN)
%   R.x1, R.x2     MxK sensitivities [s^2], [s]
%   R.miss_nt_ft   MxK  nT * x1        -- miss from the step maneuver [ft]
%   R.miss_he_ft   MxK  -VM*HE * x2    -- miss from heading error [ft]
%                       (all columns identical -- see benchmark 5)
%   R.cf_nt_ft     MxK  closed-form counterpart (NaN if not available)
%   R.cf_he_ft     Mx1  closed-form counterpart, law-independent
%   R.he_scale     the -VM*HE factor actually used [ft/s]
%   R.pn, R.apn    column index of APN=0 / APN=1, or [] if not run
%   R.bench        struct array, computed vs book, with verdicts
%   R.pass         true iff every EVALUATED benchmark row passed
%   R.verified     true iff the closed-form rows actually ran and passed
%   R.p            the parameters actually used
%   R.summary      the rendered text block
%
%   TO PUT THIS ON THE BOARD in mgs.zarchan.benchSuite, three edits, all
%   required together (its manifest guard errors if you do two of the three):
%     1. AUDITED_TOTAL 5 -> 6
%     2. P(end+1) = struct('key','adjointSingleLagCh8', 'listing','Listing 8.1', ...
%          'entry','adjointSingleLagCh8(''render'',false,''assert'',false)', ...
%          'call', @() mgs.zarchan.adjointSingleLagCh8('render',false,'assert',false), ...
%          'adapt', @adaptAdjointSingleLagCh8, 'chapter','Ch.8  PN vs APN single-lag adjoint');
%     3. an adapter looping R.bench, plus a cite() case.  Every row carries its
%        own .source string already, so the cite() table can just read it.
%   NOTE the 'assert',false in the entry: benchSuite wants a full board, not a
%   first-failure exception.  R.bench still carries every verdict.
%
%   See also MGS.ZARCHAN.ADJOINTSINGLELAG, MGS.ZARCHAN.ADJOINTBINOMIAL5,
%   MGS.ZARCHAN.BENCHSUITE, MGS.ZARCHAN.CROSSCHECK.

arguments
    % Book nominal case, Listing 8.1 data statement (zarchan.txt:2463).
    % Feet and seconds throughout.
    opts.VM     (1,1) double {mustBeFinite, mustBePositive} = 3000.0  % ft/s
    opts.HEDEG  (1,1) double {mustBeFinite}                 = -20.0   % deg
    opts.XNT    (1,1) double {mustBeFinite}                 = 96.6    % ft/s^2 = 3 g
    opts.TAU    (1,1) double {mustBeFinite, mustBePositive} = 1.0     % s
    opts.XNP    (1,1) double {mustBeFinite, mustBePositive} = 4.0     % N-prime
    opts.TF     (1,1) double {mustBeFinite, mustBePositive} = 10.0    % s, sweep end
    opts.H      (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s, RK2 step
    opts.samp   (1,1) double {mustBeFinite, mustBePositive} = 0.1     % s, record every
    % The listing's APN switch.  Only 0 and 1 are defined by the book (Fig.
    % 8.11 caption, :2451).  A fractional value would be a plausible model of
    % imperfect target-acceleration estimation, and this function REFUSES it
    % rather than quietly returning a curve that no figure describes and no
    % benchmark here covers.
    opts.APN    (1,:) double {mustBeMember(opts.APN, [0 1])} = [0 1]
    opts.render (1,1) logical = true    % print the table
    opts.benchmark (1,1) logical = true % compute R.bench
    opts.assert (1,1) logical = true    % ERROR if an evaluated row fails
    opts.twin   (1,1) logical = true    % cross-check vs the Ch.3 port
    opts.lagsweep (1,1) logical = true  % extra runs at T/8, T/4, T/2
end

% *** DO NOT ADD "if nargin == 0" TO THIS FILE. ***
% In a function whose arguments block declares ONLY name-value options, MATLAB
% reports nargin = 0 NO MATTER HOW MANY were passed (measured, R2025b).  Three
% ports in an earlier wave used that idiom: one became infinite recursion, two
% silently replaced the caller's arguments with the benchmark run.  Every
% switch in this file is an explicit logical option, and that is the only
% mechanism permitted here.

VM  = opts.VM;   TAU = opts.TAU;  XNP = opts.XNP;
XNT = opts.XNT;  TF  = opts.TF;   H   = opts.H;   samp = opts.samp;
laws = opts.APN(:).';

% The book's own conversion.  57.3 is not 180/pi; swapping in the exact value
% moves HE in the 4th digit -- harmless, but it would put every printed number
% slightly off the published ones for no gain.  Kept as the book has it.
HE = opts.HEDEG / 57.3;                                   % rad

if numel(unique(laws)) ~= numel(laws)
    error('mgs:zarchan:adjointSingleLagCh8:dupLaw', ...
        ['''APN'' lists the same law twice.  Duplicate columns would make the ' ...
         'PN-vs-APN comparison rows compare a curve with itself and pass ' ...
         'trivially -- refusing rather than returning a board of free ticks.']);
end
if H > samp
    error('mgs:zarchan:adjointSingleLagCh8:step', ...
        ['integration step H = %g s is larger than the sample interval ' ...
         'samp = %g s, so the returned curve would be sampled at the step ' ...
         'rate and silently disagree with its own time vector.'], H, samp);
end
if H >= TAU
    error('mgs:zarchan:adjointSingleLagCh8:stiff', ...
        ['H = %g s is not smaller than the guidance time constant TAU = %g s. ' ...
         'The lag state would be integrated at or below one sample per time ' ...
         'constant; RK2 still returns a smooth, entirely wrong curve here ' ...
         'rather than blowing up, so this is refused instead of warned about.'], ...
        H, TAU);
end

% ---- the runs -------------------------------------------------------------
K = numel(laws);
tRef = [];
X1 = [];  X2 = [];
for j = 1:K
    S = adjointRun(laws(j), TAU, XNP, TF, H, samp);
    if isempty(tRef)
        tRef = S.t;
        X1 = zeros(numel(tRef), K);
        X2 = zeros(numel(tRef), K);
    elseif numel(S.t) ~= numel(tRef) || max(abs(S.t - tRef)) > 1e-9
        % APN touches x1 only, so the TP sequence CANNOT depend on it.  If it
        % does, the loop has been rewired and every column-to-column
        % comparison below is comparing different abscissae.
        error('mgs:zarchan:adjointSingleLagCh8:grid', ...
            ['the time grid changed between APN = %g and APN = %g.  APN enters ' ...
             'only the x1 derivative, so it cannot move TP; something in the ' ...
             'integration loop has been rewired.'], laws(1), laws(j));
    end
    X1(:,j) = S.x1;
    X2(:,j) = S.x2;
end
M = numel(tRef);
if M == 0
    error('mgs:zarchan:adjointSingleLagCh8:empty', ...
        ['the sweep recorded no samples: TF = %g s with samp = %g s. ' ...
         'Returning an empty curve would look like a converged result.'], ...
        TF, samp);
end

he_scale = -VM*HE;                                   % ft/s, = +1047.120 nominal
miss_nt  = XNT * X1;                                 % ft
miss_he  = he_scale * X2;                            % ft

% ---- closed forms ---------------------------------------------------------
isIntNp  = abs(XNP - round(XNP)) < 1e-12;
Np       = round(XNP);
a        = 1.0/TAU;
cf_nt_ok = isIntNp && (Np >= 3);      % the (1-H)/s^3 route needs N' >= 3
cf_he_ok = isIntNp && (Np >= 2);      % the (1-H)/s^2 route needs N' >= 2
cf_nt = nan(M, K);
if cf_nt_ok
    for j = 1:K
        cf_nt(:,j) = XNT * cfMiss(tRef, a, Np, laws(j));
    end
end
if cf_he_ok
    cf_he = he_scale * cfKernel(tRef, a, Np-2, 2);
else
    cf_he = nan(M,1);
end

R = struct();
R.listing    = 'Zarchan Listing 8.1 (zarchan.txt:2463-2470), adjoint of Fig. 8.11';
R.units      = 'feet, ft/s, ft/s^2; g = 32.2 ft/s^2 -- Zarchan''s units, NOT SI';
R.t          = tRef;
R.APN        = laws;
R.x1         = X1;
R.x2         = X2;
R.miss_nt_ft = miss_nt;
R.miss_he_ft = miss_he;
R.cf_nt_ft   = cf_nt;
R.cf_he_ft   = cf_he;
R.he_scale   = he_scale;
R.pn         = find(laws == 0, 1);
R.apn        = find(laws == 1, 1);
R.p = struct('VM',VM, 'HEDEG',opts.HEDEG, 'HE',HE, 'XNT',XNT, 'TAU',TAU, ...
             'XNP',XNP, 'TF',TF, 'H',H, 'samp',samp, 'APN',laws, ...
             'units','ft, ft/s, ft/s^2; g = 32.2 ft/s^2');

if opts.benchmark
    R.bench = benchmarks(R, opts, cf_nt_ok, cf_he_ok, isIntNp);
    ev = ~[R.bench.skipped];
    R.pass     = any(ev) && all([R.bench(ev).ok]);
    R.verified = R.pass && cf_nt_ok;
else
    % NOT the same as "passed with nothing to check".  Spelled out so a caller
    % that forgot the flag cannot mistake an unchecked run for a clean one.
    R.bench    = emptyBench();
    R.pass     = false;
    R.verified = false;
end

R.summary = render(R, opts, cf_nt_ok, cf_he_ok, isIntNp);
if opts.render
    fprintf('%s\n', R.summary);
end

if opts.assert
    if ~opts.benchmark
        error('mgs:zarchan:adjointSingleLagCh8:assertNoBench', ...
            ['''assert'' is on but ''benchmark'' is off, so there is nothing to ' ...
             'assert.  A run that checks nothing must not be able to report ' ...
             'success -- set one of the two.']);
    end
    ev = ~[R.bench.skipped];
    if ~any(ev)
        error('mgs:zarchan:adjointSingleLagCh8:noBenchmark', ...
            ['not one benchmark could be evaluated with these parameters ' ...
             '(N'' = %g, APN = %s).  Refusing to hand back an unvalidated ' ...
             'port silently -- pass ''assert'', false if you meant to run ' ...
             'off-book.\n\n%s'], XNP, mat2str(laws), R.summary);
    end
    if ~R.pass
        bad = find(~[R.bench.ok] & ev);
        error('mgs:zarchan:adjointSingleLagCh8:benchmark', ...
            ['%d of %d evaluated benchmarks FAILED (first: "%s", book %.6g, ' ...
             'ours %.6g, band %s).  These compare against published values and ' ...
             'against closed-form solutions of the very ODE being integrated, ' ...
             'so a red row means the port is wrong, not that the band was ' ...
             'tight.  Full board:\n\n%s'], ...
            numel(bad), sum(ev), R.bench(bad(1)).what, R.bench(bad(1)).book, ...
            R.bench(bad(1)).ours, R.bench(bad(1)).band, R.summary);
    end
end
end


% =============================================================================
function S = adjointRun(APN, TAU, XNP, TF, H, samp)
%ADJOINTRUN  THE LISTING.  Transcribed from zarchan.txt:2463-2469.
%
%   The OCR of that block renders "<=" as "," and ">=" as "."; the three
%   affected lines are  while TP,=(TF-1e-5) ,  while STEP,=1  and  if S.=.0999 .
%   All three have a NUMERIC on one side, so the repair is forced by shape and
%   nothing is left to judge -- this listing contains no variable-to-variable
%   comparison of the kind that cannot be resolved.  Those are the ONLY edits
%   to the control flow.
%
%   The listing's trailing line in the extracted file ("Of course in a zero-lag
%   system both guidance laws would always yield zero miss distance...") is
%   BOOK PROSE that leaked into the code block, not code.  It is dropped here,
%   and its content is used where it belongs -- as benchmark 7's shape check.
%
%   NEITHER VM NOR XNT IS AN ARGUMENT, deliberately.  They scale the OUTPUTS
%   (miss = nT*x1, miss = -VM*HE*x2) and do not enter the loop at all, so the
%   trajectory cannot depend on them.  Passing them in would invite someone to
%   "use" them somewhere inside.
%
%   Fig. 8.11 states, all four of them:
%     x1  <- adjoint image of the nT input        [s^2]  (the APN term is here)
%     x2  <- adjoint image of the ydot(0) input   [s]
%     x3  <- carries the unit impulse             [-]
%     x4  <- the guidance lag state
%   Apart from the two outputs none of these has physical meaning (the same
%   warning the book gives for Fig. 3.12 at :742); do not try to interpret x3
%   or x4 as anything.

% TP starts at +1e-5, not 0.  That is not sloppiness: the adjoint gains are
% N'/tgo and N'/tgo^2 and tgo IS the adjoint time, so an exact zero divides by
% zero on the very first derivative evaluation.  The offset is small enough not
% to move the published numbers and is part of what reproduces them.
%
% Note Listing 8.1 divides by TP DIRECTLY, where Listing 3.1 first forms
% TGO = TP + .00001.  Both are protected by this same initial offset; the
% difference is cosmetic and is left as each listing has it.
TP = 0.0 + 0.00001;

X1 = 0.0;
X2 = 0.0;
X3 = 1.0;   % <<< THE IMPULSE.  Unit area on the miss node.  The book says so in
            %     words (:2452): "an initial condition of unity is applied to the x3
            %     integrator to make a miss distance adjoint".  Put it on x1,
            %     x2 or x4 instead and the program STILL RUNS, producing a
            %     smooth curve of the right order of magnitude that answers a
            %     different question.  See the fault table in the header.
X4 = 0.0;
Sacc = 0.0;

nmax = ceil(TF/samp) + 4;
A_TP = zeros(nmax,1);  A_X1 = zeros(nmax,1);  A_X2 = zeros(nmax,1);
n = 0;

% Derivatives are initialised before the loop so the FLAG==1 branch inside
% cannot read a stale value left over from anything else.
X1D = 0.0;  X2D = 0.0;  X3D = 0.0;  X4D = 0.0;

while TP <= (TF - 1e-5)
    X1OLD = X1;  X2OLD = X2;  X3OLD = X3;  X4OLD = X4;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  Pass 1 evaluates the derivatives at
    % the current state; pass 2 takes a full Euler step, advances TP by H, and
    % re-evaluates there.  TP is advanced INSIDE this loop, exactly once, which
    % is what makes the second evaluation happen at t+H -- move that line out
    % and the scheme quietly degrades to first order while still running.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            X1 = X1 + H*X1D;
            X2 = X2 + H*X2D;
            X3 = X3 + H*X3D;
            X4 = X4 + H*X4D;
            TP = TP + H;
        end
        % ---- the four adjoint differential equations, verbatim -------------
        % X1D's second term is the augmentation and the ONLY line in the
        % listing that differs in KIND from Ch.3.  It carries no 1/TP, because
        % the augmentation gain 0.5*N'*nT*tgo^2/tgo^2 is constant in tgo.  A
        % 1/TP added here would look entirely reasonable and would still
        % integrate; benchmark 2 is what catches it.
        X1D = X2 + X4*XNP*APN/(2.*TAU);
        X2D = X3 + XNP*X4/(TAU*TP);
        X3D = XNP*X4/(TAU*TP*TP);
        X4D = -X4/TAU - X2;
        FLAG = 1;
    end
    FLAG = 0;

    % Heun corrector: (x_old + x_pred)/2 + (H/2)*f(t+H, x_pred), which equals
    % x_old + (H/2)*(f_old + f_pred).  X1D..X4D here are the SECOND-pass
    % derivatives; averaging with X1OLD is what reintroduces the first.
    X1 = (X1OLD + X1)/2 + 0.5*H*X1D;
    X2 = (X2OLD + X2)/2 + 0.5*H*X2D;
    X3 = (X3OLD + X3)/2 + 0.5*H*X3D;
    X4 = (X4OLD + X4)/2 + 0.5*H*X4D;

    % Book writes this as "S >= .0999", a threshold hard-wired to H = 0.01 and
    % samp = 0.1 that mis-samples at any other step size.  The form below is
    % bit-identical at the book's values and stays correct elsewhere.
    Sacc = Sacc + H;
    if Sacc >= samp - 0.5*H
        Sacc = 0.0;
        n = n + 1;
        A_TP(n) = TP;
        A_X1(n) = X1;
        A_X2(n) = X2;
    end
end

S = struct('t', A_TP(1:n), 'x1', A_X1(1:n), 'x2', A_X2(1:n));
end


% =============================================================================
function y = cfMiss(t, a, Np, APN)
%CFMISS  MNT/nT for the single-lag loop, PN and augmented PN.
%
%     MNT/nT (s) = s^(Np-3)/(s+a)^Np - (Np*a*APN/2) * s^(Np-3)/(s+a)^(Np+1)
%
%   First term: the book's own (1-H)/s^3 with 1-H = [s/(s+a)]^Np (:828-831).
%   Second term: the augmentation, derived in the header.  At APN = 0 it is
%   identically absent, so this function IS the book's formula there -- not an
%   approximation to it.
p = Np - 3;
if p < 0
    error('mgs:zarchan:adjointSingleLagCh8:cfOrder', ...
        ['the maneuver closed form needs N'' >= 3, got %d.  There is no ' ...
         'sensible extrapolation below it.'], Np);
end
y = cfKernel(t, a, p, 3);
if APN ~= 0
    y = y - (Np*a*APN/2) * cfKernel(t, a, p, 4);
end
end


% =============================================================================
function y = cfKernel(t, a, p, n0)
%CFKERNEL  Inverse Laplace transform of  s^p / (s + a)^(p + n0).
%
%   Expand the numerator about the pole,  s^p = ((s+a) - a)^p, then invert term
%   by term using  L^-1{1/(s+a)^n} = t^(n-1) e^(-at)/(n-1)!.  EXACT for integer
%   p -- the sum is FINITE, there is no truncation here.
%
%   Requires INTEGER p >= 0.  For a fractional p the binomial is an infinite
%   series and this routine would silently return a truncation of it, which is
%   not a closed form and must not be printed beside one.  The caller gates on
%   integrality; do not remove that gate.
%
%   Benchmark 8 checks this against all four closed forms the book prints, so
%   it is verified as the book's, not merely as self-consistent.
if p < 0 || abs(p - round(p)) > 0
    error('mgs:zarchan:adjointSingleLagCh8:cfKernel', ...
        'cfKernel needs an integer p >= 0, got %g.', p);
end
y = zeros(size(t));
for k = 0:p
    % nchoosek and factorial are base MATLAB -- no toolbox is pulled in here.
    y = y + nchoosek(p,k) * (-a)^k * t.^(n0-1+k) / factorial(n0-1+k);
end
y = y .* exp(-a*t);
end


% =============================================================================
function B = benchmarks(R, opts, cf_nt_ok, cf_he_ok, isIntNp)
%BENCHMARKS  The nine rows.  Nothing here recomputes a book value.
B = emptyBench();
t   = R.t;
TAU = opts.TAU;  XNP = opts.XNP;  H = opts.H;
tolCF = max(0.01, 6.0*H);        % see TOLERANCE in the header

% --- 1 & 2: adjoint vs closed form, per law --------------------------------
% Residual is normalised by the curve's own PEAK, not pointwise.  Both curves
% cross zero (PN at tF/T = 3.0, APN at 1.27 and 4.73), and a pointwise relative
% error there divides a small number by a smaller one and manufactures a
% failure out of nothing.
for j = 1:numel(R.APN)
    law = R.APN(j);
    if law == 0
        nm  = 'PN  column vs p.55 closed form';
        src = 'p.52-55 (zarchan.txt:828-849), Fig. 3.16';
        bnd = 'closed form (BOOK)';
        det = 'analytic inverse transform; shares no code with the RK2 loop';
    else
        nm  = 'APN column vs extended closed form';
        src = 'DERIVED (see header); reduces to p.55 at APN=0';
        bnd = 'closed form (DERIVED)';
        det = 'the only check that can see a wrong APN term -- see fault table';
    end
    if ~cf_nt_ok
        if ~isIntNp
            why = sprintf(['N'' = %g is not an integer.  The inversion is a ' ...
                'finite binomial only for integer N''; a truncated infinite ' ...
                'series is not a closed form and will not be printed as one.'], XNP);
        else
            why = sprintf(['N'' = %g is below 3, so (1-H)/s^3 has no inverse ' ...
                'of this form.'], XNP);
        end
        B = addSkip(B, nm, 'value', 0, 'frac of peak', bnd, src, why);
        continue;
    end
    scale = max(abs(R.cf_nt_ft(:,j)));
    if scale <= 0
        error('mgs:zarchan:adjointSingleLagCh8:degenerate', ...
            ['closed-form curve for APN = %g is identically zero, so any ' ...
             'residual would compare as perfect -- refusing to report a pass.'], law);
    end
    resid = max(abs(R.miss_nt_ft(:,j) - R.cf_nt_ft(:,j))) / scale;
    B = addRow(B, nm, 'value', 0, resid, 'frac of peak', bnd, tolCF, ...
        resid <= tolCF, src, det);
end

% --- 3: Fig. 3.13 / p.47, "nearly 12 ft" -- PN column only -----------------
% Only meaningful on the book's own case.  Running it against user-supplied
% parameters would compare our answer to a figure describing a different
% engagement: a false red light, and sometimes a false green one.
nominal = (opts.VM == 3000.0) && (opts.HEDEG == -20.0) && (opts.XNT == 96.6) ...
       && (TAU == 1.0) && (XNP == 4.0);
src313 = 'Fig. 3.13 / p.47 prose (zarchan.txt:751), via the APN=0 equivalence';
if isempty(R.pn)
    B = addSkip(B, 'PN miss at tF = 1 s, 3-g step', 'value', 12.0, 'ft', ...
        '+/-1 ft (read off a graph)', src313, ...
        'APN = 0 was not run, so there is no PN column to read');
elseif ~(nominal && opts.TF >= 1.0)
    B = addSkip(B, 'PN miss at tF = 1 s, 3-g step', 'value', 12.0, 'ft', ...
        '+/-1 ft (read off a graph)', src313, ...
        ['parameters are off the book''s nominal case, so Fig. 3.13 does not ' ...
         'describe them']);
else
    [~, i1] = min(abs(t - 1.0));
    v = R.miss_nt_ft(i1, R.pn);
    B = addRow(B, 'PN miss at tF = 1 s, 3-g step', 'value', 12.0, v, 'ft', ...
        '+/-1 ft (read off a graph)', 1.0, abs(v - 12.0) <= 1.0, src313, ...
        sprintf(['read at t = %.5f s (Zarchan starts TP at +1e-5).  A Ch.3 ' ...
                 'number, valid here only because APN=0 IS the Ch.3 system.'], t(i1)));
end

% --- 4: cross-check against the Ch.3 port ----------------------------------
srcTwin = 'mgs.zarchan.adjointSingleLag (Listing 3.1) -- NOT a book value';
tolTwin = max(0.01, 3.0*H);
if ~opts.twin
    B = addSkip(B, 'APN=0 column vs Listing 3.1 port', 'crossport', 0, ...
        'frac of peak', 'twin agreement', srcTwin, '''twin'' is false');
elseif isempty(R.pn)
    B = addSkip(B, 'APN=0 column vs Listing 3.1 port', 'crossport', 0, ...
        'frac of peak', 'twin agreement', srcTwin, 'APN = 0 was not run');
elseif isempty(which('mgs.zarchan.adjointSingleLag'))
    B = addSkip(B, 'APN=0 column vs Listing 3.1 port', 'crossport', 0, ...
        'frac of peak', 'twin agreement', srcTwin, ...
        'mgs.zarchan.adjointSingleLag is not on the path');
else
    try
        A = mgs.zarchan.adjointSingleLag('render', false, 'VM', opts.VM, ...
            'HEDEG', opts.HEDEG, 'XNT', opts.XNT, 'TAU', TAU, 'XNP', XNP, ...
            'TF', opts.TF, 'H', H, 'samp', opts.samp);
        if numel(A.t) ~= numel(t) || max(abs(A.t - t)) > 1e-9
            error('mgs:zarchan:adjointSingleLagCh8:twinGrid', ...
                ['the Ch.3 port returned a different time grid (%d samples vs ' ...
                 '%d here).  Its sampler changed; the curves are no longer ' ...
                 'comparable point by point.'], numel(A.t), numel(t));
        end
        sc = max(abs(R.miss_nt_ft(:, R.pn)));
        d  = max(abs(R.miss_nt_ft(:, R.pn) - A.miss_nt_ft)) / sc;
        B = addRow(B, 'APN=0 column vs Listing 3.1 port', 'crossport', 0, d, ...
            'frac of peak', 'twin agreement', tolTwin, d <= tolTwin, srcTwin, ...
            ['two state realisations of one system, integrated separately; ' ...
             'the residuals differ because the discretizations differ']);
    catch ME
        % A FAIL, not a skip.  But say plainly which file might be at fault:
        % the Ch.3 port ERRORS by design when its own checks fail, and that is
        % its failure, not necessarily this one's.
        B = addRow(B, 'APN=0 column vs Listing 3.1 port', 'crossport', 0, NaN, ...
            'frac of peak', 'twin agreement', tolTwin, false, srcTwin, ...
            sprintf(['the Ch.3 port RAISED: %s || that port errors by design ' ...
                     'when its own benchmarks fail, so this row failing may ' ...
                     'mean IT is broken, not this file.  Run ' ...
                     'mgs.zarchan.adjointSingleLag directly to tell them apart.'], ...
                     oneLine(ME.message)));
    end
end

% --- 5: heading-error channel is identical across APN ----------------------
srcHE = 'structural: APN enters x1 only (zarchan.txt:2467) -- NOT a book value';
if numel(R.APN) < 2
    B = addSkip(B, 'MHE identical across APN', 'structural', 0, 'ft', ...
        'EXACTLY 0', srcHE, 'only one law was run; there is nothing to compare');
else
    dhe = max(max(abs(R.miss_he_ft - R.miss_he_ft(:,1))));
    % Exact zero, deliberately: x2/x3/x4 never reference APN or x1, so the
    % columns are produced by bit-identical arithmetic.  A tolerance here would
    % hide a real coupling from x1 back into the loop.
    B = addRow(B, 'MHE identical across APN', 'structural', 0, dhe, 'ft', ...
        'EXACTLY 0', 0, dhe == 0, srcHE, ...
        'a nonzero value means x1 has been coupled back into the loop');
end

% --- 6: Fig. 8.12 ordering, both ends --------------------------------------
% Windows are in units of T because benchmark 9 proves the curves depend on
% tF/T alone.  Gated on N' = 4 only: VM, HE and nT do not enter the ORDERING,
% because both columns carry the same nT factor.  See the header for the
% intermediate band where the book's summary does not hold.
src812 = 'Fig. 8.12 prose (zarchan.txt:2461)';
if isempty(R.pn) || isempty(R.apn)
    why = 'both APN = 0 and APN = 1 are needed to compare the two laws';
    B = addSkip(B, 'Fig 8.12: |APN| < |PN|, short flights', 'shape', 0, ...
        'violations', 'sign only', src812, why);
    B = addSkip(B, 'Fig 8.12: |PN| < |APN|, long flights', 'shape', 0, ...
        'violations', 'sign only', src812, why);
elseif XNP ~= 4
    why = sprintf(['the book states it for N'' = 4; at N'' = %g the zero ' ...
        'crossings move and the windows below are not the right ones'], XNP);
    B = addSkip(B, 'Fig 8.12: |APN| < |PN|, short flights', 'shape', 0, ...
        'violations', 'sign only', src812, why);
    B = addSkip(B, 'Fig 8.12: |PN| < |APN|, long flights', 'shape', 0, ...
        'violations', 'sign only', src812, why);
else
    mn = abs(R.miss_nt_ft(:, R.pn)) - abs(R.miss_nt_ft(:, R.apn));   % >0 = APN wins
    wS = t <= 1.5*TAU;
    wL = t >= 6.5*TAU;
    if ~any(wS)
        B = addSkip(B, 'Fig 8.12: |APN| < |PN|, short flights', 'shape', 0, ...
            'violations', 'sign only', src812, ...
            sprintf('no sample lands in 0 < tF <= %g s', 1.5*TAU));
    else
        nv = sum(mn(wS) <= 0);
        B = addRow(B, 'Fig 8.12: |APN| < |PN|, short flights', 'shape', 0, nv, ...
            'violations', 'sign only', 0, nv == 0, src812, ...
            sprintf(['%d samples in 0 < tF <= %.3g s; smallest margin %.4f ft ' ...
                     '(the margin goes to zero as tF->0 by O(t^3), by ' ...
                     'construction -- the SIGN is what is asserted)'], ...
                     sum(wS), 1.5*TAU, min(mn(wS))));
    end
    if ~any(wL)
        B = addSkip(B, 'Fig 8.12: |PN| < |APN|, long flights', 'shape', 0, ...
            'violations', 'sign only', src812, ...
            sprintf('TF = %g s does not reach tF = %g s', opts.TF, 6.5*TAU));
    else
        nv = sum(mn(wL) >= 0);
        B = addRow(B, 'Fig 8.12: |PN| < |APN|, long flights', 'shape', 0, nv, ...
            'violations', 'sign only', 0, nv == 0, src812, ...
            sprintf('%d samples at tF >= %.3g s; smallest margin %.4f ft', ...
                    sum(wL), 6.5*TAU, min(-mn(wL))));
    end
end

% --- 7 & 9: the lag sweep --------------------------------------------------
% One sweep, two rows.  Ratios are taken from the caller's own run so the
% NORMALIZED grid is identical at every T; that is what makes row 9 sharp
% rather than approximate.
srcMono = 'prose at zarchan.txt:2473 and :2471 (zero lag => zero miss)';
srcT2   = 'structural: both transform terms carry T^2 -- NOT a book value';
if ~opts.lagsweep
    B = addSkip(B, 'larger T gives larger miss', 'shape', 0, 'violations', ...
        'strict ordering', srcMono, '''lagsweep'' is false');
    B = addSkip(B, 'T^2 normalization holds', 'structural', 0, 'frac of peak', ...
        '<= 1% of peak', srcT2, '''lagsweep'' is false');
else
    fac    = [0.125 0.25 0.5 1.0];
    hRatio = H/TAU;  sRatio = opts.samp/TAU;  fRatio = opts.TF/TAU;
    nL  = numel(R.APN);
    pk  = zeros(numel(fac), nL);
    relT2 = 0;
    for j = 1:nL
        ref = [];  pkn = 0;  dev = 0;
        for i = 1:numel(fac)
            Tk = TAU*fac(i);
            Sk = adjointRun(R.APN(j), Tk, XNP, fRatio*Tk, hRatio*Tk, sRatio*Tk);
            mk = opts.XNT * Sk.x1;
            pk(i,j) = max(abs(mk));
            nrm = mk / (opts.XNT * Tk^2);        % the T^2 normalization
            if isempty(ref)
                ref = nrm;
                pkn = max(abs(ref));
            else
                q = min(numel(nrm), numel(ref));
                dev = max(dev, max(abs(nrm(1:q) - ref(1:q))));
            end
        end
        if pkn <= 0
            error('mgs:zarchan:adjointSingleLagCh8:normDegenerate', ...
                ['the normalized reference curve for APN = %g is identically ' ...
                 'zero, so any deviation would compare as perfect.'], R.APN(j));
        end
        relT2 = max(relT2, dev/pkn);             % worst law, not the average
    end
    nv = sum(sum(diff(pk,1,1) <= 0));
    B = addRow(B, 'larger T gives larger miss', 'shape', 0, nv, 'violations', ...
        'strict ordering', 0, nv == 0, srcMono, ...
        sprintf(['peak |MNT| over T = [%s]*%g s, per law: %s ft.  The T^2 law ' ...
                 '(next row) is WHY zero lag gives zero miss.'], ...
                 num2str(fac), TAU, mat2str(round(pk(:).',5))));
    B = addRow(B, 'T^2 normalization holds', 'structural', 0, relT2, ...
        'frac of peak', '<= 1% of peak', 0.01, relT2 <= 0.01, srcT2, ...
        ['the ONLY row that can see a wrong power of T: the book''s default ' ...
         'run has TAU = 1, where every power looks the same']);
end

% --- 8: cfKernel vs the four forms the book PRINTS -------------------------
% Machine precision, because both sides are analytic.  This row is what makes
% rows 1 and 2 mean anything: it proves the analytic side is the BOOK's.
srcK = 'printed closed forms at zarchan.txt:846-849, :872-876, :888-895';
if ~cf_he_ok && ~cf_nt_ok
    B = addSkip(B, 'cfKernel reproduces the printed forms', 'structural', 0, ...
        'rel err', '<= 1e-12', srcK, 'no closed form is available at this N''');
else
    % A FIXED grid, independent of the run: this row is about the analytic
    % kernel, not about anything the integrator did.
    tk = (0.05:0.05:10)';
    aK = 1.0;                                   % T = 1 s for the printed forms
    e = 0;
    e = max(e, relmax(cfKernel(tk,aK,0,3), 0.5*tk.^2.*exp(-tk)));
    e = max(e, relmax(cfKernel(tk,aK,1,3), tk.^2.*exp(-tk).*(0.5 - tk/6)));
    e = max(e, relmax(cfKernel(tk,aK,2,3), tk.^2.*exp(-tk).*(0.5 - tk/3 + tk.^2/24)));
    e = max(e, relmax(cfKernel(tk,aK,2,2), tk.*exp(-tk).*(1 - tk + tk.^2/6)));
    B = addRow(B, 'cfKernel reproduces the printed forms', 'structural', 0, e, ...
        'rel err', '<= 1e-12', 1e-12, e <= 1e-12, srcK, ...
        ['N''=3,4,5 maneuver and N''=4 heading error, term for term.  If this ' ...
         'row is red, rows 1 and 2 are comparing against the wrong analytic.']);
end
end


% =============================================================================
%   BENCH PLUMBING.  Field names and order are fixed by EMPTYBENCH; a struct
%   array that quietly reordered fields would make [B.ok] mean something else.
% =============================================================================
function B = emptyBench()
B = struct('what',{}, 'kind',{}, 'book',{}, 'ours',{}, 'units',{}, ...
           'band',{}, 'tol',{}, 'ok',{}, 'skipped',{}, 'why',{}, ...
           'source',{}, 'detail',{});
end

function B = addRow(B, what, kind, book, ours, units, band, tol, ok, source, detail)
B(end+1) = struct('what',what, 'kind',kind, 'book',book, 'ours',ours, ...
    'units',units, 'band',band, 'tol',tol, 'ok',logical(ok), ...
    'skipped',false, 'why','', 'source',source, 'detail',detail);
end

function B = addSkip(B, what, kind, book, units, band, source, why)
B(end+1) = struct('what',what, 'kind',kind, 'book',book, 'ours',NaN, ...
    'units',units, 'band',band, 'tol',NaN, 'ok',false, ...
    'skipped',true, 'why',why, 'source',source, 'detail','');
end

function e = relmax(got, want)
%RELMAX  max|got-want| normalised by the PEAK of WANT.  Peak, not pointwise:
%   every one of these curves passes through zero.
s = max(abs(want));
if s <= 0
    error('mgs:zarchan:adjointSingleLagCh8:relmax', ...
        'reference curve is identically zero; any residual would look perfect.');
end
e = max(abs(got - want)) / s;
end

function s = oneLine(msg)
s = regexprep(msg, '\s+', ' ');
if numel(s) > 200
    s = [s(1:197) '...'];
end
end


% =============================================================================
function s = render(R, opts, cf_nt_ok, cf_he_ok, isIntNp)
W = 92;
L = {};
L{end+1} = 'Zarchan Listing 8.1 -- ADJOINT: PROPORTIONAL vs AUGMENTED PROPORTIONAL NAVIGATION';
L{end+1} = '*** UNITS ARE THE BOOK''S: feet, ft/s, ft/s^2, g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf('  VM = %g ft/s   HE = %g deg   nT = %g ft/s^2 (%.2f g)   TAU = %g s   N'' = %g', ...
    opts.VM, opts.HEDEG, opts.XNT, opts.XNT/32.2, opts.TAU, opts.XNP);
L{end+1} = sprintf('  TF = %g s   H = %g s   samp = %g s   laws: APN = %s   (0 = PN, 1 = augmented PN)', ...
    opts.TF, opts.H, opts.samp, mat2str(R.APN));
L{end+1} = sprintf('  heading-error scale -VM*HE = %+.3f ft/s   (NOT 2*VM*HE -- see header)', ...
    R.he_scale);
L{end+1} = '  integrator: Zarchan''s two-pass predictor/corrector, transcribed';
L{end+1} = repmat('-', 1, W);

% ---- the curve table ------------------------------------------------------
hdr = sprintf('%9s', 'tF [s]');
un  = sprintf('%9s', '');
for j = 1:numel(R.APN)
    hdr = [hdr sprintf('%12s%12s', lawName(R.APN(j)), 'closed')]; %#ok<AGROW>
    un  = [un  sprintf('%12s%12s', '[ft]', '[ft]')];               %#ok<AGROW>
end
hdr = [hdr sprintf('%12s%12s', 'HE miss', 'HE closed')];
un  = [un  sprintf('%12s%12s', '[ft]', '[ft]')];
L{end+1} = hdr;
L{end+1} = un;

stride = max(1, round(0.5/opts.samp));
rows = unique([1:stride:numel(R.t), numel(R.t)]);
if opts.TF >= 1.0
    [~, i1] = min(abs(R.t - 1.0));
    rows = unique([rows, i1]);
end
for r = rows
    ln = sprintf('%9.5f', R.t(r));
    for j = 1:numel(R.APN)
        ln = [ln sprintf('%12.4f', R.miss_nt_ft(r,j))]; %#ok<AGROW>
        if cf_nt_ok
            ln = [ln sprintf('%12.4f', R.cf_nt_ft(r,j))]; %#ok<AGROW>
        else
            ln = [ln sprintf('%12s', '--')]; %#ok<AGROW>
        end
    end
    ln = [ln sprintf('%12.4f', R.miss_he_ft(r,1))]; %#ok<AGROW>
    if cf_he_ok
        ln = [ln sprintf('%12.4f', R.cf_he_ft(r))]; %#ok<AGROW>
    else
        ln = [ln sprintf('%12s', '--')]; %#ok<AGROW>
    end
    L{end+1} = ln; %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = '  The HE columns are printed ONCE: the heading-error channel is identical for';
L{end+1} = '  PN and APN (APN feeds forward nT, and a heading error has no nT).  That is';
L{end+1} = '  asserted as an EXACT zero below, not as a tolerance.';
if ~cf_nt_ok
    L{end+1} = '';
    if ~isIntNp
        L{end+1} = sprintf('  CLOSED FORM UNAVAILABLE: N'' = %g is not an integer.  The inversion is a', opts.XNP);
        L{end+1} = '  finite binomial only for integer N''; for a fractional one it becomes an';
        L{end+1} = '  infinite series, and a truncation of that is NOT a closed form.  Rather';
        L{end+1} = '  than print an approximation under the word "closed", none is computed.';
    else
        L{end+1} = sprintf('  CLOSED FORM UNAVAILABLE: N'' = %g is below 3.', opts.XNP);
    end
    L{end+1} = '  R.verified = false.  THE RETURNED CURVES ARE UNCHECKED.';
end
L{end+1} = repmat('-', 1, W);

% ---- the board ------------------------------------------------------------
if isempty(R.bench)
    L{end+1} = 'BENCHMARKS ***NOT COMPUTED*** (''benchmark'' is false).  R.pass is false and';
    L{end+1} = 'means "not checked", NOT "checked and fine".';
else
    L{end+1} = sprintf('%-38s %13s %13s %9s  %s', 'BENCHMARK', 'book', 'ours', 'band', 'verdict');
    for i = 1:numel(R.bench)
        b = R.bench(i);
        if b.skipped
            L{end+1} = sprintf('%-38s %13.6g %13s %9s  SKIP', ...
                b.what, b.book, '--', '--'); %#ok<AGROW>
            L{end+1} = sprintf('     skipped: %s', b.why); %#ok<AGROW>
        else
            L{end+1} = sprintf('%-38s %13.6g %13.6g %9.4g  %s', ...
                b.what, b.book, b.ours, b.tol, passStr(b.ok)); %#ok<AGROW>
            L{end+1} = sprintf('     %s  [%s]', b.detail, b.source); %#ok<AGROW>
        end
    end
    ev = ~[R.bench.skipped];
    L{end+1} = '';
    L{end+1} = sprintf('  %d evaluated, %d passed, %d failed, %d skipped   ->  R.pass = %d', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(ev & ~[R.bench.ok]), sum(~ev), R.pass);
end
L{end+1} = repmat('-', 1, W);
L{end+1} = 'REMINDER: x1 and x2 are SENSITIVITIES, not misses.';
L{end+1} = '   miss from target maneuver = x1 * nT        (already done: R.miss_nt_ft)';
L{end+1} = '   miss from heading error   = x2 * (-VM*HE)  (already done: R.miss_he_ft)';
L{end+1} = 'The APN closed form is DERIVED, not printed in the book -- see the header.';
s = strjoin(L, newline);
end

function s = lawName(a)
if a == 0
    s = 'PN miss';
else
    s = 'APN miss';
end
end

function s = passStr(tf)
if tf
    s = 'PASS';
else
    s = 'FAIL';
end
end
