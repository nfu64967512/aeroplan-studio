function A = adjointSingleLag(opts)
%MGS.ZARCHAN.ADJOINTSINGLELAG  Zarchan Listing 3.1 -- single-lag miss adjoint.
%
%   A = mgs.zarchan.adjointSingleLag()            book case, prints, returns curve
%   A = mgs.zarchan.adjointSingleLag('TAU', 2.0)  sweep the guidance lag
%   A = mgs.zarchan.adjointSingleLag('render', false)
%
%   Line-by-line port of Listing 3.1 (zarchan.txt:756-765), the adjoint of the
%   Fig. 3.12 proportional-navigation homing loop with a single guidance time
%   constant TAU, integrated by the book's own second-order Runge-Kutta (Heun)
%   scheme.
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, g = 32.2 ft/s^2.  NOT SI.
%
%   The rest of mgs is metres.  This file is deliberately NOT converted, so
%   that every number it prints can be laid straight against a printed figure
%   without a factor floating around to be got wrong.  A silent ft->m here
%   would turn 12 ft of miss into "3.7" and it would still look like a
%   perfectly reasonable miss distance -- which is the precise failure mode
%   this package exists to prevent.  If you need metres, convert AT THE CALL
%   SITE and say so there.
% ===========================================================================
%
%   WHY THIS ONE MATTERS HERE
%
%   The adjoint gets miss-distance sensitivity for EVERY flight time out of a
%   SINGLE run.  This project currently obtains that curve by flying SITL
%   sorties -- 32 of them for the truth set.  More to the point, mgs scores
%   32/32 at 0.03 m where the real aircraft score 25/32 at 0.93 m, and the
%   measured cause is a guidance-system LAG the model does not have (measured
%   gamma-tracking gain 0.063 against the model's 1.385, ~8 s of it, saturating
%   on 26 of 32 sorties).  Ch. 3 is the machinery for exactly that: what a time
%   constant T does to miss.  This port is the entry point to it.
%
%   Two things it buys, and they are different things:
%     1. An INDEPENDENT CHECK of our integrator and PN kernel, because the book
%        publishes the answers (Figs. 3.13, 3.16, 3.18 and the p.55/p.56
%        normalized formulas).  See BENCHMARKS below.
%     2. The lag -> miss relationship itself, which nothing else in mgs has.
%
%   *** WHAT IT IS NOT. ***  This is a LINEARISED, PLANAR, constant-closing-
%   speed homing loop with a FIRST-ORDER lag.  Our vehicle has gravity, drag,
%   thrust, an asymmetric nz clamp, a LOS floor, and a lag that SATURATES.
%   Do not read a miss distance out of here and call it a prediction for an
%   aircraft.  Read it as the answer to "what would a clean loop with this much
%   lag do", which is the baseline our failures are measured against.
%
% ---------------------------------------------------------------------------
%   THE OUTPUTS ARE SENSITIVITIES.  YOU MUST MULTIPLY THEM.
%
%   This is where people go wrong, so it is stated as a table.  x1 and x2 are
%   NOT miss distances.  Each one becomes a miss only after multiplication:
%
%     A.x1   [s^2]  ->  miss from a step target maneuver   =  x1 * nT
%     A.x2   [s]    ->  miss from an initial heading error =  x2 * (-VM*HE)
%
%   Both products are already formed for you as A.miss_nt_ft and A.miss_he_ft,
%   so ordinary use never has to touch the scale factors.  The raw x1/x2 are
%   returned because the SENSITIVITY, not the miss, is what you want when
%   re-scaling to a different maneuver level: the loop is linear, so doubling
%   nT doubles the miss and the curve does not have to be recomputed.
%
%   The UNITS in that table are the cheapest available check that the impulse
%   is on the right state (see below).  nT is ft/s^2 and a miss is ft, so x1
%   must carry s^2; the heading-error scale VM*HE is ft/s, so x2 must carry s.
%   x1 = int(x2) is exactly that one-power-of-time relationship.
%
% ---------------------------------------------------------------------------
%   THE HEADING-ERROR SCALE FACTOR IS  -VM*HE,  NOT  2*VM*HE.
%
%   The prose at zarchan.txt:749 reads "(multiply by 2VM HE to get miss)".
%   That "2" is an OCR artifact of a MINUS SIGN, and the same corruption is
%   visible one paragraph later at :750, which reads "there is 220 deg of
%   heading error" for a listing whose own source says HEDEG = -20.  Minus
%   became "2" twice in the same passage.
%
%   Four independent confirmations that the factor is -VM*HE:
%     - the listing itself (:762) computes  ArrayXMHE(n) = -VM*HE*X2;
%     - the generalized transfer function at :884 is MHE/HE = -VM*[1-H(s)]/s^2;
%     - the p.56 normalized formulas are tabulated as  Miss/(VM*HE)  -- there
%       is no 2 anywhere in the normalizing group;
%     - sign and size.  With HE = -20 deg, -VM*HE = +1047 ft/s and the miss
%       curve is positive with a 179 ft peak, matching Fig. 3.14.  2*VM*HE
%       would be -2094: the wrong sign AND twice the magnitude, and it would
%       plot as a perfectly smooth, perfectly wrong curve.
%
%   Taking the prose literally is the trap.  It is recorded here because the
%   number it produces is plausible and nothing downstream would flinch.
%
% ---------------------------------------------------------------------------
%   WHY THE IMPULSE GOES ON x3, AND NOWHERE ELSE
%
%   Adjoint rules swap inputs and outputs: the quantity you wanted to observe
%   in the original system becomes the DRIVING IMPULSE of the adjoint, and the
%   original system's disturbance inputs become the adjoint's outputs.  We want
%   MISS, which is y(tF) -- the output of the second integrator of the original
%   loop.  So the impulse belongs at the adjoint image of that node.
%
%   Trace the chain.  In the original system nT enters at y-double-dot and
%   reaches y through TWO integrations:  nT -> ydd -> yd -> y.  The adjoint
%   runs it backwards, and the code says so:  X1D = X2  and  X2D = X3.  So
%   x3 -> x2 -> x1 is that same chain reversed, with x3 sitting at the y end.
%   x3 is therefore the miss node; x2 lands on the image of yd(0) (which is
%   what a heading error IS -- an initial condition on lateral velocity); and
%   x1 lands on the image of nT.  Hence x1 pairs with nT and x2 with -VM*HE,
%   the pairing asserted in the table above.
%
%   *** PUT THE IMPULSE ON x1, x2 OR x4 AND THE PROGRAM STILL RUNS. ***  It
%   integrates cleanly, produces a smooth decaying curve of the right general
%   shape and the right order of magnitude, and answers a DIFFERENT QUESTION --
%   the sensitivity of miss RATE, or of acceleration, instead of miss.  There
%   is no symptom.  That is why the impulse state is hard-coded below and is
%   not an option: an argument named 'impulse_state' would be an invitation to
%   generate confident nonsense.  The closed-form check (BENCHMARK 2) is what
%   would actually catch it, and it is on by default for that reason.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS -- both run automatically, and a failure is an ERROR
%
%   1  Fig. 3.13 / p.47 prose: a 3-g target maneuver causes "nearly 12 ft" of
%      miss at tF = 1 s.  This port gives 12.0712 ft.  Tolerance +-1.0 ft,
%      because 12 is a value read off a printed graph, not a published digit.
%      Only checked when the parameters are the book's nominal set; change TAU,
%      XNP, VM, XNT or HEDEG and it is skipped, because the figure no longer
%      describes the case being run.
%
%   2  The p.55 closed form, which the book states agrees with this listing
%      (Figs. 3.16 and 3.18).  It IS reconstructible, and is implemented in
%      cfKernel below:
%
%        1 - H(s) = [ s/(s + 1/T) ]^N'
%        miss from step maneuver:  MNT/nT = (1-H)/s^3
%        miss from heading error:  MHE/HE = -VM*(1-H)/s^2
%
%      whose inverse transform for integer N' is the finite sum in cfKernel.
%      For N' = 4 that reduces to the book's printed result
%
%        MNT/nT = t^2 * exp(-t/T) * (0.5 - t/(6T))         [11.85 ft at t=T=1]
%        MHE/HE = -VM * t * exp(-t/T) * (1 - t/T + t^2/(6T^2))
%
%      This is a REAL second opinion: it never touches the integrator or the
%      loop wiring, so it catches a mis-placed impulse, a swapped derivative or
%      a botched Heun corrector, none of which BENCHMARK 1 would.
%
%      Confidence that cfKernel is not invented: it reproduces ALL SIX closed
%      forms the book prints at p.55-56 -- target maneuver at N' = 3, 4, 5 and
%      heading error at N' = 3, 4, 5 -- term for term, including the 1.5 and
%      the /24 in the N' = 5 heading-error case.  It was also checked against
%      this integrator at N' = 3,4,5,6 and TAU = 0.5, 1.0.
%
%      TOLERANCE, and why it has both a slope AND a floor.  At the book's
%      H = 0.01 the adjoint sits 1.88% of peak above the closed form (12.0712
%      vs 11.8458 ft at tF = 1 s, which is +1.90% pointwise).  That gap is
%      discretization against the 1/tgo singularity at the start of the run,
%      NOT a modelling error -- but it converges at only about H^0.67, NOT at
%      first order, precisely because of that singularity.  Hence
%
%           tol = max(0.5% of peak, 4*H)
%
%      The 4*H slope tracks the error where it dominates: measured margin is
%      2.46x at H = 0.05, 2.13x at 0.01, 1.99x at 0.005.  The 0.5% FLOOR is
%      there because a purely linear band eventually dips UNDER a residual
%      that is only falling as H^0.67 -- without it, someone who refined H to
%      around 1.5e-4 to get a BETTER answer would be rewarded with a spurious
%      ERROR on a perfectly correct run.  That is the worst failure this file
%      could have, so the floor is not negotiable.
%
%      The floor is safe because the check has enormous margin.  Fault
%      injection at H = 0.01, residual as % of peak:
%           correct                          1.88%
%           impulse moved to x1            810%
%           impulse moved to x2        106,253%
%           impulse moved to x4        107,049%
%           X2D wired to x4 instead of x3  100%
%           sign flipped on X4D         47,758%
%      Every structural break lands at 100% or more -- 200x the floor.  There
%      is no daylight between "this passes" and "this is right".
%
%   If XNP is not an integer the closed form cannot be formed by this route.
%   The function then says so in plain words, sets A.verified = false, and
%   returns the curve UNCHECKED rather than substituting an approximation.
%
% ---------------------------------------------------------------------------
%   A.t            adjoint time [s] = original-system FLIGHT TIME tF, and also
%                  readable as "time to go at which the maneuver starts"
%   A.x1, A.x2     the two sensitivities described above
%   A.miss_nt_ft   XNT * x1    -- miss from the step target maneuver [ft]
%   A.miss_he_ft   -VM*HE * x2 -- miss from the initial heading error [ft]
%   A.cf_nt_ft     closed-form counterpart of miss_nt_ft (NaN if unavailable)
%   A.cf_he_ft     closed-form counterpart of miss_he_ft (NaN if unavailable)
%   A.he_scale     the -VM*HE factor actually used [ft/s]
%   A.verified     true only if the closed-form check actually ran and passed
%   A.check        benchmark detail
%   A.p            the parameters actually used
%   A.summary      the rendered text block
%
%   See also MGS.ZARCHAN.CROSSCHECK, MGS.ZARCHAN.SIZING, MGS.ZARCHAN.ADDITIVELAW.

arguments
    % Book nominal case, Listing 3.1 line 1.  Feet and seconds throughout.
    opts.VM     (1,1) double {mustBeFinite, mustBePositive} = 3000.0  % ft/s
    opts.HEDEG  (1,1) double {mustBeFinite}                 = -20.0   % deg
    opts.XNT    (1,1) double {mustBeFinite}                 = 96.6    % ft/s^2 = 3 g
    opts.TAU    (1,1) double {mustBeFinite, mustBePositive} = 1.0     % s
    opts.XNP    (1,1) double {mustBeFinite, mustBePositive} = 4.0     % N-prime, dimensionless
    opts.TF     (1,1) double {mustBeFinite, mustBePositive} = 10.0    % s, sweep end
    opts.H      (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s, RK2 step
    opts.samp   (1,1) double {mustBeFinite, mustBePositive} = 0.1     % s, record every
    opts.render (1,1) logical                               = true
end

VM  = opts.VM;   TAU  = opts.TAU;  XNP  = opts.XNP;
XNT = opts.XNT;  TF   = opts.TF;   H    = opts.H;    samp = opts.samp;

% The book's own conversion.  57.3 is not 180/pi, and swapping in the exact
% value shifts HE in the 4th digit -- harmless, but it would put every printed
% number slightly off the published ones for no gain.  Kept as the book has it.
HE = opts.HEDEG / 57.3;                                   % rad

if H > samp
    error('mgs:zarchan:adjointSingleLag:step', ...
        ['integration step H = %g s is larger than the sample interval ' ...
         'samp = %g s, so the returned curve would be sampled at the step ' ...
         'rate and silently disagree with its own time vector.'], H, samp);
end
if H >= TAU
    error('mgs:zarchan:adjointSingleLag:stiff', ...
        ['H = %g s is not smaller than the guidance time constant TAU = %g s. ' ...
         'The lag state would be integrated at or below one sample per time ' ...
         'constant; RK2 still returns a smooth, entirely wrong curve here ' ...
         'rather than blowing up, so this is refused instead of warned about.'], ...
        H, TAU);
end

% =========================================================================
%   THE LISTING.  Transcribed from zarchan.txt:757-765.  The OCR of that block
%   renders "<=" as ",=" and ">=" as ".="; those are restored here and are the
%   ONLY edits to the control flow.
%
%   Fig. 3.12 states, all four of them:
%     x1  <- int x2          adjoint image of the nT input      [s^2]
%     x2  <- int x3          adjoint image of the yd(0) input   [s]
%     x3  <- driven by N'*y1/tgo, and carries the unit impulse  [-]
%     x4  <- the guidance lag state
%   The book notes that apart from the two outputs none of these has physical
%   meaning; do not try to interpret x3 or x4 as anything.
% =========================================================================
T0 = 0.0;
S  = 0.0;

% TP starts at +1e-5, not 0, and TGO carries a second +1e-5.  That is not
% sloppiness: the adjoint gain is N'/tgo and tgo IS the adjoint time, so an
% exact zero divides by zero on the very first derivative evaluation.  The
% offset is small enough not to move the published numbers and is part of what
% reproduces them, so it is kept rather than "cleaned up".
TP = T0 + 0.00001;

X1 = 0.0;
X2 = 0.0;
X3 = 1.0;   % <<< THE IMPULSE.  Unit area, on the miss node.  See header.
X4 = 0.0;

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
        X1D = X2;
        X2D = X3;
        Y1  = (X4 - X2)/TAU;        % the single lag
        TGO = TP + 0.00001;         % adjoint time IS time-to-go
        X3D = XNP*Y1/TGO;           % the time-varying PN gain N-prime/tgo
        X4D = -Y1;
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
    S = S + H;
    if S >= samp - 0.5*H
        S = 0.0;
        n = n + 1;
        A_TP(n) = TP;
        A_X1(n) = X1;
        A_X2(n) = X2;
    end
end

A_TP = A_TP(1:n);  A_X1 = A_X1(1:n);  A_X2 = A_X2(1:n);
if n == 0
    error('mgs:zarchan:adjointSingleLag:empty', ...
        ['the sweep recorded no samples: TF = %g s with samp = %g s. ' ...
         'Returning an empty curve would look like a converged result.'], ...
        TF, samp);
end

% ---- form the miss distances -----------------------------------------------
% The two products from the header table, formed here once so that no caller
% has to remember the scale factors -- particularly the -VM*HE one.
he_scale = -VM*HE;                                   % ft/s, = +1047.120 nominal
miss_nt  = XNT * A_X1;                               % ft
miss_he  = he_scale * A_X2;                          % ft

% ---- BENCHMARK 2 material: the closed form ---------------------------------
isIntNp = abs(XNP - round(XNP)) < 1e-12;
cf_ok   = isIntNp && (round(XNP) >= 3);              % the /s^3 route needs N-prime >= 3
if cf_ok
    cf_nt = XNT      * cfKernel(A_TP, TAU, round(XNP), 3);
    cf_he = he_scale * cfKernel(A_TP, TAU, round(XNP), 2);
else
    cf_nt = nan(size(A_TP));
    cf_he = nan(size(A_TP));
end

A = struct();
A.t          = A_TP;
A.x1         = A_X1;
A.x2         = A_X2;
A.miss_nt_ft = miss_nt;
A.miss_he_ft = miss_he;
A.cf_nt_ft   = cf_nt;
A.cf_he_ft   = cf_he;
A.he_scale   = he_scale;
A.p = struct('VM',VM, 'HEDEG',opts.HEDEG, 'HE',HE, 'XNT',XNT, 'TAU',TAU, ...
             'XNP',XNP, 'TF',TF, 'H',H, 'samp',samp, ...
             'units','ft, ft/s, ft/s^2; g = 32.2 ft/s^2');

[A.check, A.verified] = runChecks(A, opts, cf_ok, isIntNp);
A.summary = render(A, opts, cf_ok, isIntNp);
if opts.render
    fprintf('%s\n', A.summary);
end
end

% =============================================================================
function y = cfKernel(t, T, Np, nInt)
%CFKERNEL  Inverse Laplace transform of  s^(Np-nInt) / (s + 1/T)^Np.
%
%   nInt = 3 gives the target-maneuver sensitivity (the book: miss is "after
%   three integrations"), nInt = 2 the heading-error one ("after two").  Both
%   follow from 1 - H(s) = [s/(s+1/T)]^Np, the result derived at p.52.
%
%   Expand the numerator about the pole,  s^p = ((s+a) - a)^p  with a = 1/T and
%   p = Np - nInt, then invert term by term.  EXACT for integer p -- the sum is
%   finite, there is no truncation here.
%
%   Requires INTEGER Np.  For a fractional Np the binomial above is an infinite
%   series and this routine would silently return a truncation of it, which is
%   not a closed form and must not be printed beside one.  The caller gates on
%   integrality; do not remove that gate.
a = 1.0/T;
p = Np - nInt;
if p < 0
    error('mgs:zarchan:adjointSingleLag:cfOrder', ...
        ['closed form needs N'' >= %d for this channel, got %d.  There is no ' ...
         'sensible extrapolation below it.'], nInt, Np);
end
y = zeros(size(t));
for k = 0:p
    % nchoosek and factorial are base MATLAB -- no toolbox is pulled in here.
    y = y + nchoosek(p,k) * (-a)^k * t.^(nInt-1+k) / factorial(nInt-1+k);
end
y = y .* exp(-a*t);
end

% =============================================================================
function [C, verified] = runChecks(A, opts, cf_ok, isIntNp)
%RUNCHECKS  The two benchmarks.  A failure ERRORS: an adjoint that is wrong but
%   returns its curve anyway is the worst possible outcome here, because the
%   curve is smooth, plausible, and indistinguishable from an answer.
C = struct();
C.cf_available = cf_ok;
C.integer_np   = isIntNp;
C.cf_resid = NaN;  C.cf_tol = NaN;  C.cf_pass = false;

% --- BENCHMARK 2: adjoint vs closed form, over the whole sweep --------------
verified = false;
if cf_ok
    % Residual is normalised by the curve's own PEAK, not pointwise.  The
    % closed form crosses zero (near tF = 3T at N' = 4), and a pointwise
    % relative error there divides a small number by a smaller one and
    % manufactures a failure out of nothing.
    scale = max(abs(A.cf_nt_ft));
    if scale <= 0
        error('mgs:zarchan:adjointSingleLag:degenerate', ...
            ['closed-form maneuver curve is identically zero, so any residual ' ...
             'would compare as perfect -- refusing to report a pass.']);
    end
    resid = max(abs(A.miss_nt_ft - A.cf_nt_ft)) / scale;
    % The 4*H slope tracks discretization where it dominates.  The 0.5% FLOOR
    % stops a CORRECT run from failing once H is small enough that the residual
    % -- which converges at only ~H^0.67 against the 1/tgo singularity -- stops
    % falling as fast as a linear band does.  Measured: without the floor this
    % starts throwing spuriously near H = 1.5e-4, i.e. it would punish someone
    % for refining the step.  The floor costs nothing, because every structural
    % break measured (impulse on the wrong state, swapped derivative, flipped
    % sign) lands at 100% of peak or more.  See BENCHMARK 2 in the header.
    tol   = max(0.005, 4.0*opts.H);
    C.cf_resid = resid;
    C.cf_tol   = tol;
    C.cf_pass  = (resid <= tol);
    if ~C.cf_pass
        error('mgs:zarchan:adjointSingleLag:closedForm', ...
            ['adjoint disagrees with the p.55 closed form by %.3f%% of peak ' ...
             '(tolerance %.3f%% at H = %g).  These two share no code -- the ' ...
             'closed form is analytic, the adjoint is integrated -- so a gap ' ...
             'this size means the loop wiring, the impulse state or the Heun ' ...
             'corrector is wrong, NOT that the step size is coarse. ' ...
             'Discretization alone is only 1.88%% of peak at H = 0.01, while ' ...
             'every structural break measured (impulse on the wrong state, ' ...
             'swapped derivative, flipped sign) lands at 100%% or more.'], ...
            100*resid, 100*tol, opts.H);
    end
    verified = true;
end

% --- BENCHMARK 1: Fig. 3.13, "nearly 12 ft" at tF = 1 s ---------------------
% Only meaningful on the book's own case.  Running it against user-supplied
% parameters would compare our answer to a figure describing a different
% engagement -- a false red light, and worse, sometimes a false green one.
nominal = (opts.VM == 3000.0) && (opts.HEDEG == -20.0) && (opts.XNT == 96.6) ...
       && (opts.TAU == 1.0)   && (opts.XNP == 4.0);
C.fig313_applicable = nominal && (opts.TF >= 1.0);
C.fig313_book = 12.0;
C.fig313_tol  = 1.0;                       % a value read off a printed graph
C.idx_tf1 = [];  C.t_tf1 = NaN;
C.fig313_adj = NaN;  C.fig313_cf = NaN;  C.fig313_pass = false;
if C.fig313_applicable
    [~, j] = min(abs(A.t - 1.0));
    C.idx_tf1     = j;
    C.t_tf1       = A.t(j);
    C.fig313_adj  = A.miss_nt_ft(j);
    C.fig313_cf   = A.cf_nt_ft(j);
    C.fig313_pass = abs(C.fig313_adj - C.fig313_book) <= C.fig313_tol;
    if ~C.fig313_pass
        error('mgs:zarchan:adjointSingleLag:fig313', ...
            ['tF = 1 s miss from the 3-g target maneuver came out %.4f ft; ' ...
             'Fig. 3.13 shows nearly %.0f ft (tolerance %.1f ft).  The port ' ...
             'does not reproduce the book.'], ...
            C.fig313_adj, C.fig313_book, C.fig313_tol);
    end
end
end

% =============================================================================
function s = render(A, opts, cf_ok, isIntNp)
L = {};
L{end+1} = sprintf('Zarchan Listing 3.1 -- single-lag miss adjoint (RK2, H = %g s)', opts.H);
L{end+1} = '  UNITS ARE THE BOOK''S: feet, ft/s, ft/s^2.  Nothing here is SI.';
L{end+1} = sprintf('  VM = %g ft/s   HE = %g deg   nT = %g ft/s^2 (%.2f g)   TAU = %g s   N'' = %g', ...
    opts.VM, opts.HEDEG, opts.XNT, opts.XNT/32.2, opts.TAU, opts.XNP);
L{end+1} = sprintf('  heading-error scale -VM*HE = %+.3f ft/s   (NOT 2*VM*HE -- see header)', ...
    A.he_scale);
L{end+1} = '';
L{end+1} = sprintf('%9s %12s %14s %14s %12s %14s', ...
    'tF [s]', 'x1 [s^2]', 'miss_nT [ft]', 'closed [ft]', 'x2 [s]', 'miss_HE [ft]');

stride = max(1, round(0.5/opts.samp));
rows = unique([1:stride:numel(A.t), numel(A.t)]);
if ~isempty(A.check.idx_tf1)
    rows = unique([rows, A.check.idx_tf1]);
end
for r = rows
    if cf_ok
        cfs = sprintf('%14.4f', A.cf_nt_ft(r));
    else
        cfs = sprintf('%14s', '--');
    end
    L{end+1} = sprintf('%9.4f %12.6f %14.4f %s %12.6f %14.4f', ...
        A.t(r), A.x1(r), A.miss_nt_ft(r), cfs, A.x2(r), A.miss_he_ft(r)); %#ok<AGROW>
end
L{end+1} = '';

% --- benchmark 1 ---
if A.check.fig313_applicable
    L{end+1} = 'BENCHMARK 1  Fig. 3.13 -- 3-g target maneuver at tF = 1 s';
    L{end+1} = sprintf('   adjoint       %9.4f ft   at tF = %.5f s', ...
        A.check.fig313_adj, A.check.t_tf1);
    L{end+1} = sprintf('   book          ~%.0f ft (read off the figure)', A.check.fig313_book);
    L{end+1} = sprintf('   |difference|  %9.4f ft   tol %.2f ft   -> %s', ...
        abs(A.check.fig313_adj - A.check.fig313_book), A.check.fig313_tol, ...
        passStr(A.check.fig313_pass));
else
    L{end+1} = 'BENCHMARK 1  SKIPPED -- parameters are not the book''s nominal case,';
    L{end+1} = '             so Fig. 3.13 does not describe this engagement.';
end
L{end+1} = '';

% --- benchmark 2 ---
if cf_ok
    L{end+1} = 'BENCHMARK 2  p.55 closed form (shares no code with the integrator)';
    if A.check.fig313_applicable
        L{end+1} = sprintf('   closed form   %9.4f ft at tF = 1 s   [t^2 e^(-t/T) (0.5 - t/6T)]', ...
            A.check.fig313_cf);
    end
    L{end+1} = sprintf('   max |adjoint - closed| = %.4f%% of peak   tol %.4f%% (max(0.5%%, 4*H))   -> %s', ...
        100*A.check.cf_resid, 100*A.check.cf_tol, passStr(A.check.cf_pass));
    L{end+1} = '   That residual is discretization against the 1/tgo singularity, not';
    L{end+1} = '   error: 1.88% of peak at the book''s H = 0.01.  Band = max(0.5%, 4*H).';
    L{end+1} = '   This is the check that would catch the impulse being put on the';
    L{end+1} = '   wrong state -- BENCHMARK 1 alone would not.';
else
    L{end+1} = 'BENCHMARK 2  SKIPPED -- CLOSED FORM NOT AVAILABLE.';
    if ~isIntNp
        L{end+1} = sprintf('             N'' = %g is not an integer.  The p.55 inversion is a', opts.XNP);
        L{end+1} = '             finite binomial only for integer N''; for a fractional one it';
        L{end+1} = '             becomes an infinite series, and a truncation of that is NOT a';
        L{end+1} = '             closed form.  Rather than print an approximation next to the';
        L{end+1} = '             words "closed form", this function declines to compute one.';
    else
        L{end+1} = sprintf('             N'' = %g is below 3, so the (1-H)/s^3 maneuver channel', opts.XNP);
        L{end+1} = '             has no inverse of this form.';
    end
    L{end+1} = '             A.verified = false.  THE RETURNED CURVE IS UNCHECKED.';
end

L{end+1} = '';
L{end+1} = 'REMINDER: x1 and x2 are SENSITIVITIES, not misses.';
L{end+1} = '   miss from target maneuver = x1 * nT        (already done: miss_nt_ft)';
L{end+1} = '   miss from heading error   = x2 * (-VM*HE)  (already done: miss_he_ft)';
s = strjoin(L, newline);
end

function s = passStr(tf)
if tf
    s = 'PASS';
else
    s = 'FAIL';
end
end
