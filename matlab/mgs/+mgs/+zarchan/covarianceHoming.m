function R = covarianceHoming(opts)
%MGS.ZARCHAN.COVARIANCEHOMING  Zarchan Listing 5.2 -- homing-loop covariance
%   analysis, ported line for line from the book's own source.
%
%   R = mgs.zarchan.covarianceHoming()                 % the book's run, prints
%   R = mgs.zarchan.covarianceHoming('xnp', [3 4 5])
%   R = mgs.zarchan.covarianceHoming('xnt', 5*32.2)    % 5-g target, not 3
%   R = mgs.zarchan.covarianceHoming('render', false, 'assert', false)
%
%   Propagates the state covariance of the single-lag homing loop of Fig. 5.4
%   through the matrix differential equation
%
%       Xdot = F(t) X + X F(t)'  +  Q
%
%   with fourth-order Runge-Kutta, and returns the standard-deviation
%   HISTORIES.  The miss dispersion falls out as sigma_y(tF), with NO Monte
%   Carlo: one deterministic run replaces the whole ensemble.
%
% ===========================================================================
%   *** UNITS ARE THE BOOK'S.  FEET, ft/s, ft/s^2, g = 32.2.  NOT SI. ***
%
%   Zarchan works in feet.  The rest of mgs is metres.  This file is
%   DELIBERATELY left in the book's units so the published numbers can be
%   compared digit for digit with no conversion sitting in between to be got
%   wrong.  A dispersion of "13.3" is plausible in either unit, so a silent
%   conversion returns a number that looks right and is wrong by 3.28x.
%   Convert at the CALL SITE, visibly.  One benchmark row below exists purely
%   to catch a units edit: it asserts 15*g == 483, which only holds at 32.2.
% ===========================================================================
%
%   WHY WE WANT THIS ONE
%
%   We currently price a parameter against dispersion by flying 32 SITL
%   sorties per change.  This listing computes dispersion analytically.  That
%   is the whole attraction, so the limits below are the important part of
%   this header, not decoration.
%
%   *** WHAT IT CANNOT PRICE: ARRIVAL-TIME SPREAD. ***
%   Our acceptance criterion is a 3 s arrival spread.  This model CANNOT
%   speak to it, and the proof is inside the file: closing velocity Vc
%   cancels out of the loop exactly (structural check 4 measures the
%   cancellation at ~1e-14).  A model in which the closing speed provably
%   does not affect the answer has no along-track channel at all; every
%   sigma here is LOS-NORMAL.  So a number off this file is a MISS
%   dispersion, never an arrival-time dispersion.  Reporting one as the
%   other would be the "right topic, wrong direction" failure with a book
%   citation attached to make it convincing.
%
%   WHAT IT CAN PRICE, and this is real:
%     * miss dispersion from a random target maneuver, one run per error
%       source (the book's own point: run it once per source for a budget);
%     * miss dispersion from a RELEASE-SIDE INITIAL CONDITION, via 'x0'.
%       Our measured discriminant is release lateral/heading error |eps0|,
%       which enters here as a variance on state 2 (ydot).  Use
%       R.x0_from_heading_error(vm_fts, sigma_rad) to build that x0 rather
%       than filling the matrix by hand -- putting it in the wrong slot
%       returns a plausible number instead of an error.
%     * the missile-acceleration sigma sigma_nL(t), i.e. how much authority
%       the dispersion demands, which is the quantity our nz clamp bounds.
%
%   NONE OF THIS IS OUR VEHICLE.  No gravity, no drag, no thrust, no
%   asymmetric nz clamp, no LOS floor, and above all the single lag here is
%   linear and NEVER SATURATES -- saturation is exactly what our own data
%   says dominates (gamma-tracking gain 0.063 against the model's 1.385).
%
% ---------------------------------------------------------------------------
%   *** THE TERMINAL SAMPLE IS NOT A RESULT.  READ BEFORE QUOTING ANY g. ***
%
%   Zarchan divides by tgo and dodges tgo = 0 with TGO = tF - t + 1e-5.  At
%   the LAST recorded sample tgo IS that 1e-5.  Consequences, all measured
%   here and all asserted by the self-checks:
%
%     sigma_y(tF)   SURVIVES.  13.297 ft at the book's settings.  It moves
%                   0.28% when the 1e-5 is changed to 1e-4 and it agrees
%                   with the Chapter 4 CLOSED FORM to 0.53%.  This is the
%                   number the book publishes and the number to use.
%
%     sigma_nL(tF)  IS GARBAGE.  It comes out 1.2366e+05 g.  It is NOT a
%                   step-size artifact -- it is identical to four digits at
%                   h/2, h/4 and h/8 -- it is N'*sigma_y/(T*eps) and so it
%                   scales as 1/eps: 1.24e5 at eps=1e-5, 1.24e4 at 1e-4,
%                   1256 at 1e-3.  It is a function of Zarchan's fudge, not
%                   of the flight.  Structural check 6 ASSERTS that 1/eps
%                   scaling so nobody can quietly start quoting it.  The
%                   book hides it with axis([0 10 0 20]); we reproduce that
%                   axis in the plot, and we return R.sig_nl_g_masked with
%                   that sample set to NaN so an accidental (end) fails.
%
%     THE WHOLE X   at that sample is not a covariance matrix: its
%                   correlations reach 2.97 and its correlation matrix has
%                   an eigenvalue of -2.99.  Every sample BEFORE it is
%                   properly PSD (worst normalized eigenvalue +1.15e-4, at
%                   t = 5.6 s).  Structural check 3 tests the record with
%                   the final sample excluded and SAYS SO, instead of
%                   quietly widening a tolerance until it passes.
%
%   sigma_ydot at the final sample is corrupted the same way and does not
%   converge under h refinement (2345, 1759, 1351 ft/s at h, h/2, h/4).
%   R.sig_ydot is returned because it is useful over the flight; its last
%   column is NaN in R.sig_ydot_masked for the same reason.
%
% ---------------------------------------------------------------------------
%   TRANSCRIPTION NOTES -- what the OCR damaged and how it was resolved.
%   Source: matlab/zarchan/_listings/listing_05_02.m, from zarchan.txt:1521.
%
%   1. `while ~(T .= (TF-.0001))`.  The extractor's '.'-for-'>' repair does
%      not fire when the right-hand side is an expression, so this one
%      arrived unresolved.  It is '>=', and not by preference: with '<=' the
%      test reads ~(0 <= 9.9999) = false and the loop body NEVER EXECUTES.
%      Only '>=' produces a program at all.  The same idiom two lines down,
%      `if S . = .09999`, WAS auto-repaired to '>=' and confirms the mapping.
%      There are no variable-to-variable comparisons in this listing.
%
%   2. PHIS = XNT*XNT/TF.  The prose renders the definition as "Fs = n2T tF"
%      (zarchan.txt:1290 and :1442) because the extraction lost the FRACTION
%      BAR, which then reads as a multiply.  The code says divide and the
%      code is right: with nT^2*tF the answer would be 10x too large (133 ft
%      instead of 13.3), and the Chapter 4 closed form carries the same 1/tF.
%      An OCR artifact, not a book erratum.
%
%   3. `XD=(F*X)+(F*X)'+Q` is transcribed as written.  Note it is NOT the
%      textbook Lyapunov form F*X + X*F' + Q: it is a SYMMETRIZER, equal to
%      the Lyapunov form only while X stays symmetric.  It does stay
%      symmetric -- exactly, to the last bit, which structural check 2
%      asserts with a tolerance of ZERO.  A non-symmetric x0 would silently
%      mean something else, so x0 is checked and REJECTED rather than
%      quietly symmetrised.
%
%   4. The paragraph trailing the transcribed file ("The homing loop
%      covariance analysis program of Listing 5.2 was run, ...") is PROSE
%      that leaked into the code block during extraction.  Dropped, not
%      "repaired" into code.  Its content is the 13.3 ft benchmark below.
%
%   5. `save datfil.txt output /ascii` is not reproduced: this function
%      returns data, it does not write files into the caller's pwd.
%
% ---------------------------------------------------------------------------
%   THE MODEL (Fig. 5.4; state space derived at zarchan.txt:1450-1500)
%
%     x = [ y ; ydot ; ydotdot_T ; delta ]
%
%     y          relative separation normal to the LOS          [ft]
%     ydot       its rate                                       [ft/s]
%     ydotdot_T  target acceleration, white noise * integrator  [ft/s^2]
%     delta      first-order-lagged LOS angle                   [rad]
%
%     F = [ 0              1   0   0
%           -N'/(T*tgo)    0   1   N'*Vc/T
%           0              0   0   0
%           1/(T*Vc*tgo)   0   0   -1/T ]
%
%     Q(3,3) = PHIS = nT^2/tF          (every other entry zero)
%     nL     = A*x,  A = [ N'/(T*tgo)  0  0  -N'*Vc/T ]
%
%   delta is the LAG STATE, so nL = N'*Vc*deltadot: the LOS rate is formed by
%   the washout s/(1+sT), which is why the "lag" sits where it does.  The
%   only error source is the 3-g uniformly distributed target maneuver
%   (zarchan.txt:1517) unless the caller adds one through x0 or phis.
%
% ---------------------------------------------------------------------------
%   OUTPUT   (K = numel(xnp), M = number of 0.1-s samples)
%
%   R.t                1xM      sample times                        [s]
%   R.tgo              1xM      tF - t + eps_tgo, what F was built from
%   R.sig_y            KxM      sqrt(X(1,1)) relative separation     [ft]
%   R.sig_ydot         KxM      sqrt(X(2,2))                        [ft/s]
%   R.sig_nt           KxM      sqrt(X(3,3)) target acceleration     [ft/s^2]
%   R.sig_delta        KxM      sqrt(X(4,4))                        [rad]
%   R.sig_nl           KxM      sqrt(A*X*A') missile acceleration   [ft/s^2]
%   R.sig_nl_g         KxM      the same in g -- the book's ArraySIGNLG
%   R.sig_nl_g_masked  KxM      final column NaN.  USE THIS ONE.
%   R.sig_ydot_masked  KxM      final column NaN.
%   R.Xs               4x4xMxK  the covariance matrices themselves
%   R.miss_ft          Kx1      sigma_y(tF), the miss dispersion    [ft]
%   R.xnp              1xK
%   R.closed_form            handle: Ch.4 p.85-86 formula, N'=3 only
%   R.x0_from_heading_error  handle: builds an x0 from a heading sigma
%   R.bench            struct array, computed vs book, with verdicts
%   R.selfcheck        structural invariants and their residuals
%   R.pass             true iff every SCORED benchmark passed
%   R.summary          the printed text
%
%   BENCHSUITE WIRING (not done here -- benchSuite guards its own manifest):
%       key    'covarianceHoming'    listing 'Listing 5.2'
%       entry  covarianceHoming('assert',false,'render',false)
%       adapt  read R.bench rows: .what .book_s .ours_s .pct .band .tol .ok
%              .skipped .scored .why .detail .source  (.source already
%              carries the figure/page citation, so no cite() table row is
%              needed -- do NOT add a second copy of it there)
%       and bump AUDITED_TOTAL from 5 to 6.
%
%   See also MGS.ZARCHAN.BENCHSUITE, MGS.ZARCHAN.ADJOINTSINGLELAG,
%   MGS.ZARCHAN.ADJOINTBINOMIAL5.

arguments
    % Book names are UPPERCASE; these are the same variables, lowercased.
    % Defaults are Listing 5.2's own data statement -- INCLUDING xnp = 3,
    % which the listing hard-codes.  This is a transcription: the default
    % run is the book's run, not a sweep somebody thought more interesting.
    opts.xnp     (1,:) double {mustBeNonnegative, mustBeFinite} = 3       % XNP, N'
    opts.tau     (1,1) double {mustBePositive,    mustBeFinite} = 1.0     % TAU, T  [s]
    opts.tf      (1,1) double {mustBePositive,    mustBeFinite} = 10.0    % TF      [s]
    opts.h       (1,1) double {mustBePositive,    mustBeFinite} = 0.01    % H       [s]
    opts.vc      (1,1) double {mustBePositive,    mustBeFinite} = 4000.0  % VC  [ft/s]
    opts.xnt     (1,1) double {mustBeNonnegative, mustBeFinite} = 96.6    % XNT [ft/s^2] = 3 g
    % PHIS override.  NaN means "derive it from xnt, as the book does".  It
    % is NOT validated as nonnegative/finite because those validators reject
    % NaN, which is the sentinel.  Checked by hand below.
    opts.phis    (1,1) double = NaN                                       % PHIS
    % Initial covariance.  Zero in the book.  Non-zero is how a release-side
    % initial condition (our |eps0|) gets priced; see x0_from_heading_error.
    opts.x0      (4,4) double = zeros(4)
    % Zarchan's L'Hopital dodge, TGO = tF - t + eps.  Exposed because the
    % terminal acceleration sigma is a pure function of it (see header) and
    % structural check 6 has to vary it.  Changing it changes the answer.
    opts.eps_tgo (1,1) double {mustBePositive, mustBeFinite} = 1e-5
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
    % Internal.  The structural checks re-run this function with altered
    % vc / xnt / eps_tgo / h; those inner runs must not self-check again or
    % the recursion never bottoms out.  Turning it off is turning off the
    % only tests that still bite when the caller runs off-book.
    opts.selfcheck (1,1) logical = true
end

% There is deliberately NO `if nargin == 0` anywhere in this file.  In a
% function whose arguments block declares ONLY name-value options MATLAB
% reports nargin = 0 no matter how many were passed (measured, R2025b), so
% such a test is ALWAYS TRUE.  Three ports in the last wave used it: one
% became infinite recursion, two silently discarded the caller's arguments.
% Every switch here is an explicit flag.

GRAV_FT = 32.2;                 % g in the book's units.  Do not "modernise".
EPS_TGO = opts.eps_tgo;

R = struct();
R.units   = 'feet, ft/s, ft/s^2, g = 32.2 -- Zarchan''s units, NOT SI';
R.listing = 'Zarchan Listing 5.2 (zarchan.txt:1521), homing loop of Fig. 5.4';
R.opts    = opts;
R.xnp     = opts.xnp;
R.grav_ft = GRAV_FT;

% ---- input guards ---------------------------------------------------------
if isempty(opts.xnp)
    error('mgs:zarchan:covarianceHoming:noXnp', ...
        '''xnp'' is empty, so there is nothing to run.  Pass at least one N''.');
end

% x0 must be a real, symmetric, PSD covariance.  It is REJECTED rather than
% symmetrised: the propagation uses Zarchan's (F*X)+(F*X)'+Q, which
% symmetrises silently, so an asymmetric x0 would be quietly reinterpreted
% as something the caller did not write and would still return plausible ft.
if ~isreal(opts.x0)
    error('mgs:zarchan:covarianceHoming:x0Complex', '''x0'' must be real.');
end
asym0 = max(max(abs(opts.x0 - opts.x0.')));
if asym0 > 0
    error('mgs:zarchan:covarianceHoming:x0Asym', ...
        ['''x0'' is not symmetric (max |x0 - x0''| = %.3g).  A covariance ' ...
         'matrix is symmetric by definition, and the book''s update would ' ...
         'symmetrise yours without telling you.  Fix the matrix; do not ' ...
         'rely on the integrator to fix it.'], asym0);
end
if any(diag(opts.x0) < 0)
    error('mgs:zarchan:covarianceHoming:x0NegVar', ...
        '''x0'' has a negative variance on its diagonal; that is not a covariance.');
end
if any(opts.x0(:) ~= 0)
    ev0 = eig(opts.x0);
    if min(ev0) < -1e-10*max([abs(ev0); 1])
        error('mgs:zarchan:covarianceHoming:x0NotPSD', ...
            ['''x0'' is not positive semidefinite (min eigenvalue %.3g).  ' ...
             'Propagating it produces negative variances downstream and ' ...
             'sqrt() then hands back complex "standard deviations" that ' ...
             'print exactly like real ones.'], min(ev0));
    end
end

% PHIS: chosen once, recorded, never recomputed anywhere below.
phisOverridden = ~isnan(opts.phis);
if phisOverridden
    if ~isfinite(opts.phis) || opts.phis < 0
        error('mgs:zarchan:covarianceHoming:phis', ...
            '''phis'' must be finite and nonnegative (got %g).', opts.phis);
    end
    PHIS = opts.phis;
else
    PHIS = opts.xnt*opts.xnt/opts.tf;     % PHIS=XNT*XNT/TF -- see note 2
end
R.phis = PHIS;
R.phis_overridden = phisOverridden;

% An all-zero problem integrates to an all-zero answer.  That is arithmetic,
% not a result, and "sigma = 0" reads as "no dispersion" -- exactly the shape
% of green light this package exists to prevent.
if PHIS == 0 && all(opts.x0(:) == 0)
    error('mgs:zarchan:covarianceHoming:noSource', ...
        ['there is no error source: phis = 0 and x0 = 0, so every standard ' ...
         'deviation would be identically zero.  Give the loop a maneuver ' ...
         '(xnt / phis) or an initial condition (x0).']);
end

% h must divide tf.  If it does not, the book's loop test overshoots tF on
% the final step and TGO = tF - t + eps goes NEGATIVE: F(2,1) and F(4,1) flip
% sign and the run returns confident nonsense.  The in-loop TGO guard would
% catch it, but the reason is clearer stated here.
nsteps = round(opts.tf/opts.h);
if abs(nsteps*opts.h - opts.tf) > 1e-9*max(1, opts.tf)
    error('mgs:zarchan:covarianceHoming:stepDivide', ...
        ['h = %g does not divide tf = %g (%.6g steps).  Zarchan''s loop ' ...
         'test ~(T >= tF - 1e-4) then steps PAST tF, making tgo negative ' ...
         'and flipping the sign of the guidance terms.  Choose h so that ' ...
         'tf/h is an integer.'], opts.h, opts.tf, opts.tf/opts.h);
end

% The book samples every 0.1 s via `if S >= .09999`, kept verbatim below.  If
% h does not divide 0.1 the curves are still right, only the abscissa is
% coarser.  Nothing downstream assumes the spacing, so this degrades visibly.
rr = rem(0.1, opts.h);
if abs(rr) > 1e-12 && abs(rr - opts.h) > 1e-12
    warning('mgs:zarchan:covarianceHoming:sampleGrid', ...
        ['h = %g does not divide 0.1 s, so the output grid is not the ' ...
         'book''s 0.1-s grid.  Values are unaffected; only the sampling is.'], ...
        opts.h);
end

% ---- the runs -------------------------------------------------------------
K = numel(opts.xnp);
P = cell(1, K);
for j = 1:K
    P{j} = propagate(opts.xnp(j), opts.tau, opts.tf, opts.vc, opts.h, ...
                     PHIS, opts.x0, EPS_TGO);
end
M = numel(P{1}.t);
if M < 3
    error('mgs:zarchan:covarianceHoming:noSamples', ...
        ['the run produced %d samples (tf = %g, h = %g).  A dispersion ' ...
         'HISTORY cannot be built from that, and handing back a two-point ' ...
         'curve would plot as almost nothing at all.'], M, opts.tf, opts.h);
end

R.t   = P{1}.t;
R.tgo = P{1}.tgo;
R.sig_y     = zeros(K, M);   R.sig_ydot  = zeros(K, M);
R.sig_nt    = zeros(K, M);   R.sig_delta = zeros(K, M);
R.sig_nl    = zeros(K, M);   R.Xs        = zeros(4, 4, M, K);
for j = 1:K
    R.sig_y(j,:)     = P{j}.sig_y;
    R.sig_ydot(j,:)  = P{j}.sig_ydot;
    R.sig_nt(j,:)    = P{j}.sig_nt;
    R.sig_delta(j,:) = P{j}.sig_delta;
    R.sig_nl(j,:)    = P{j}.sig_nl;
    R.Xs(:,:,:,j)    = P{j}.X;
end
R.sig_nl_g = R.sig_nl / GRAV_FT;      % the book's ArraySIGNLG
R.miss_ft  = R.sig_y(:, end);         % sMiss = sy(tF)

% The masked copies.  NaN, not a clipped value: NaN fails every comparison
% and shows up as a gap in a plot, whereas a clipped 20 g would silently
% become a quotable number.  See the terminal-sample block in the header.
R.sig_nl_g_masked = R.sig_nl_g;    R.sig_nl_g_masked(:, end) = NaN;
R.sig_ydot_masked = R.sig_ydot;    R.sig_ydot_masked(:, end) = NaN;

R.closed_form           = @closedFormMiss;
R.x0_from_heading_error = @x0HeadingError;

% ---- structural self-check ------------------------------------------------
if opts.selfcheck
    R.selfcheck = structuralCheck(opts, R, PHIS);
else
    R.selfcheck = struct('skipped', true);
end

% ---- book benchmarks ------------------------------------------------------
R.bench = benchmarks(R, opts, phisOverridden, GRAV_FT);
sc = [R.bench.scored] & ~[R.bench.skipped];
R.pass = any(sc) && all([R.bench(sc).ok]);

R.summary = render(R, opts, PHIS, GRAV_FT);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawFigures(R, opts, GRAV_FT); end

if opts.assert
    % "Nothing was evaluated" must NOT read as "everything passed".  A run
    % whose parameters put every benchmark out of reach has checked NOTHING,
    % and that is the shape of green light this repo has been burned by.
    if ~any(sc)
        error('mgs:zarchan:covarianceHoming:noBenchmark', ...
            ['not one book benchmark could be evaluated.  They need N'' = 3 ' ...
             'in ''xnp'' and the listing''s own case (tau = 1, tf = 10, ' ...
             'xnt = 96.6, x0 = 0, phis derived, eps_tgo = 1e-5).  Refusing ' ...
             'to hand back an unvalidated port silently -- pass ''assert'', ' ...
             'false if you meant to run off-book.']);
    end
    if ~R.pass
        bad = find(sc & ~[R.bench.ok]);
        error('mgs:zarchan:covarianceHoming:benchmark', ...
            ['%d of %d scored book benchmarks are outside tolerance (first: ' ...
             '%s, book %s, ours %s, band %s).  Either the port broke or the ' ...
             'parameters left the region the book covers.  Full table:\n\n%s'], ...
            numel(bad), sum(sc), R.bench(bad(1)).what, R.bench(bad(1)).book_s, ...
            R.bench(bad(1)).ours_s, R.bench(bad(1)).band, R.summary);
    end
end
end


% =============================================================================
function S = propagate(XNP, TAU, TF, VC, H, PHIS, X0, EPSTGO)
%PROPAGATE  Listing 5.2 itself.  The book's own statements are quoted to the
%   right of each line.  Do NOT replace this with ode45 or with lyap(): the
%   published number is what THIS scheme at THIS step size produces, and
%   lyap() is a Control System Toolbox function this package must not need.
%
%   Two things look like bugs and are not:
%     * TGO = TF - TNEW + 1e-5.  Zarchan's dodge for the 1/tgo terms at
%       tgo = 0.  Remove it and the last step divides by zero.  Keep it and
%       the last SAMPLE is evaluated at tgo = 1e-5 -- see the header.
%     * XD = (F*X) + (F*X)' + Q, not F*X + X*F' + Q.  Equal while X is
%       symmetric, which it is, exactly.

F = zeros(4,4);  X = X0;  Q = zeros(4,4);
K0 = zeros(4,4); K1 = zeros(4,4); K2 = zeros(4,4);   % preallocation, as the book
T = 0.0;  TNEW = T;  Sacc = 0.0;                     % T=0.; TNEW=T; S=0.;
TGO = TF - T + EPSTGO;                               % TGO=TF-T+.00001

F(1,2) = 1.;                                         % F(1,2)=1.
F(2,1) = -XNP/(TAU*TGO);                             % F(2,1)=-XNP/(TAU*TGO)
F(2,3) = 1.;                                         % F(2,3)=1.
F(2,4) = XNP*VC/TAU;                                 % F(2,4)=XNP*VC/TAU
F(4,1) = 1./(TAU*VC*TGO);                            % F(4,1)=1./(TAU*VC*TGO)
F(4,4) = -1./TAU;                                    % F(4,4)=-1./TAU
Q(3,3) = PHIS;                                       % Q(3,3)=PHIS

nMax = ceil(TF/H) + 4;
tA   = zeros(1, nMax);   tgoA = zeros(1, nMax);
syA  = zeros(1, nMax);   sydA = zeros(1, nMax);
sntA = zeros(1, nMax);   sdA  = zeros(1, nMax);
snlA = zeros(1, nMax);   XA   = zeros(4, 4, nMax);
n    = 0;

while ~(T >= (TF - 0.0001))                          % while ~(T >= (TF-.0001))
    Sacc = Sacc + H;                                 % S=S+H
    XOLD = X;                                        % XOLD=X
    STEP = 1;
    while (STEP == 1) || (STEP == 2) || (STEP == 3)
        TGO = TF - TNEW + EPSTGO;
        checkTgo(TGO, TNEW, TF, H);
        F(2,1) = -XNP/(TAU*TGO);
        F(4,1) = 1./(TAU*VC*TGO);
        FX = F*X;  XD = FX + FX.' + Q;               % XD=(F*X)+(F*X)'+Q
        if (STEP == 1)
            STEP = 2;  K0 = XD;  TNEW = T + .5*H;  X = XOLD + .5.*H.*K0;
        elseif (STEP == 2)
            STEP = 3;  K1 = XD;  TNEW = T + .5*H;  X = XOLD + .5.*H.*K1;
        else
            STEP = 4;  K2 = XD;  TNEW = T + H;     X = XOLD + H.*K2;
        end
    end
    TGO = TF - TNEW + EPSTGO;
    checkTgo(TGO, TNEW, TF, H);
    F(2,1) = -XNP/(TAU*TGO);
    F(4,1) = 1./(TAU*VC*TGO);
    FX = F*X;  XD = FX + FX.' + Q;
    K3 = XD;                                         % K3=XD
    T  = TNEW;                                       % T=TNEW
    X  = XOLD + H.*(K0 + 2.*(K1 + K2) + K3)./6;      % the RK4 combination

    if Sacc >= .09999                                % if S >= .09999
        Sacc = 0.;
        % A is rebuilt every sample because TGO changes.  A(1,2) and A(1,3)
        % are written out as explicit zeros, as the book does, so that a
        % later edit adding a term has to DELETE a visible zero rather than
        % fill a slot nobody knew was there.
        A = zeros(1,4);
        A(1,1) = XNP/(TAU*TGO);                      % A(1,1)=XNP/(TAU*TGO)
        A(1,2) = 0.;                                 % A(1,2)=0.
        A(1,3) = 0.;                                 % A(1,3)=0.
        A(1,4) = -XNP*VC/TAU;                        % A(1,4)=-XNP*VC/TAU
        AXAT = A*X*A.';                              % AXAT=A*X*A'
        n = n + 1;
        tA(n)   = T;                                 % ArrayT(count)=T
        tgoA(n) = TGO;
        syA(n)  = sqrtVar(X(1,1),    'X(1,1) relative position', T, X, A);
        sydA(n) = sqrtVar(X(2,2),    'X(2,2) relative rate',     T, X, A);
        sntA(n) = sqrtVar(X(3,3),    'X(3,3) target accel',      T, X, A);
        sdA(n)  = sqrtVar(X(4,4),    'X(4,4) lagged LOS angle',  T, X, A);
        snlA(n) = sqrtVar(AXAT(1,1), 'A*X*A'' missile accel',    T, X, A);
        XA(:,:,n) = X;
    end
end

S.t      = tA(1:n);     S.tgo       = tgoA(1:n);
S.sig_y  = syA(1:n);    S.sig_ydot  = sydA(1:n);
S.sig_nt = sntA(1:n);   S.sig_delta = sdA(1:n);
S.sig_nl = snlA(1:n);   S.X         = XA(:,:,1:n);
end


function checkTgo(TGO, TNEW, TF, H)
%CHECKTGO  tgo <= 0 means the loop stepped past tF and every guidance term
%   has just changed sign.  The run would carry on and return feet.  It does
%   not: refusing to return a plausible wrong answer is the whole point.
if TGO <= 0
    error('mgs:zarchan:covarianceHoming:tgoNonPositive', ...
        ['tgo = %.6g at t = %.6g with tf = %g, h = %g.  The integration ' ...
         'stepped past the intercept, which flips the sign of F(2,1) and ' ...
         'F(4,1).  Continuing would return a plausible, wrong dispersion.'], ...
        TGO, TNEW, TF, H);
end
end


function s = sqrtVar(v, what, t, X, A)
%SQRTVAR  sqrt() of a variance, with the negative case made loud.
%   A covariance that has lost positive semidefiniteness yields a small
%   negative variance; sqrt() then returns a COMPLEX number that propagates
%   through max(), plot() and fprintf() looking exactly like a real one.
if v >= 0
    s = sqrt(v);
    return;
end
% Scale to judge "small" against: the positive part of the same quadratic
% form, so the threshold means something at both 1e-4 rad^2 and 1e5 ft^2.
scale = max([abs(diag(X)).', abs(A).^2 * abs(diag(X)), eps]);
if v > -1e-9*scale
    s = 0;                 % rounding of a true zero, not a loss of PSD
    return;
end
error('mgs:zarchan:covarianceHoming:negativeVariance', ...
    ['%s came out NEGATIVE (%.6g) at t = %.4f s: the covariance has lost ' ...
     'positive semidefiniteness.  sqrt() would return a complex "standard ' ...
     'deviation" that prints like a real one.  Reduce h, or check x0.'], ...
    what, v, t);
end


% =============================================================================
function m = closedFormMiss(nT_ftps2, T_s, tF_s)
%CLOSEDFORMMISS  Zarchan Ch.4, p.85-86 (zarchan.txt:1354-1360).  The rms miss
%   due to a uniformly distributed target maneuver, VALID FOR N' = 3 ONLY:
%
%       M = (nT/4) * sqrt( (T^5/tF) * [3 - exp(-2x)(2x^4+4x^3+6x^2+6x+3)] )
%       x = tF/T
%
%   This shares NO CODE with the Riccati integrator, which is what makes it
%   worth having: it is a second opinion, not a restatement of the first.
%
%   The OCR lost the minus signs in "3 -" and in exp(-2x); both are recovered
%   from the requirement that M -> 0 as tF -> 0.  With "+" the bracket tends
%   to 6 rather than 0 and the formula would predict a finite miss for a
%   zero-length flight.  The recovered form lands on the integrator to 0.53%
%   at the book's case and to better than 1% across tF = 1..10 s and
%   T = 0.5..2 s, which no wrong sign choice does.
%
%   Returns FEET.  nT in ft/s^2; T and tF in seconds.
arguments
    nT_ftps2 (1,1) double {mustBeNonnegative, mustBeFinite}
    T_s      (1,1) double {mustBePositive,    mustBeFinite}
    tF_s     (1,1) double {mustBePositive,    mustBeFinite}
end
x   = tF_s/T_s;
brk = 3 - exp(-2*x)*(2*x^4 + 4*x^3 + 6*x^2 + 6*x + 3);
if brk < 0
    % Cannot happen for x > 0, but if it ever does the caller gets an error
    % rather than a complex "miss distance".
    error('mgs:zarchan:covarianceHoming:closedFormNeg', ...
        'closed-form bracket went negative (%.3g) at tF/T = %g.', brk, x);
end
m = nT_ftps2/4 * sqrt(T_s^5/tF_s * brk);
end


function X0 = x0HeadingError(vm_ftps, sigma_rad)
%X0HEADINGERROR  Build an initial covariance for a RELEASE HEADING ERROR.
%
%   A heading error of sigma_HE radians at a speed of Vm ft/s is a lateral
%   RATE error of Vm*sigma_HE ft/s: a variance on STATE 2 (ydot) and on
%   nothing else.  This exists so the caller does not put it on state 1 (a
%   position offset -- a different channel with a different answer) or
%   forget to square it.  Both of those return plausible feet.
%
%   Pair it with 'phis', 0 to isolate the channel; the book benchmarks then
%   correctly report themselves SKIPPED, which is the visible signal that
%   you are no longer running the book's case.
%
%       x0 = R.x0_from_heading_error(3000, 0.010);      % 3000 ft/s, 10 mr
%       Q  = mgs.zarchan.covarianceHoming('x0', x0, 'phis', 0, 'assert', false);
%
%   *** FEET PER SECOND. *** Our own vehicle speeds are quoted in m/s.
%   Convert here, at the call site, visibly.
arguments
    vm_ftps   (1,1) double {mustBePositive,    mustBeFinite}
    sigma_rad (1,1) double {mustBeNonnegative, mustBeFinite}
end
if sigma_rad > 0.5
    error('mgs:zarchan:covarianceHoming:headingUnits', ...
        ['sigma_rad = %g.  This argument is RADIANS; %g rad is %.0f deg, ' ...
         'which is almost certainly degrees typed into a radian slot.  ' ...
         'Convert first.'], sigma_rad, sigma_rad, sigma_rad*57.3);
end
X0 = zeros(4);
X0(2,2) = (vm_ftps*sigma_rad)^2;
end


% =============================================================================
function C = structuralCheck(opts, R, PHIS)
%STRUCTURALCHECK  Invariants that must hold or every number above is garbage.
%
%   These are NOT benchmarks.  They test the WIRING, using properties the
%   loop has for structural reasons, and they are the only tests in this file
%   that still work when the caller runs off-book.  Each says what it catches.

C = struct();  C.skipped = false;  C.notes = {};
zeroX0 = all(opts.x0(:) == 0);

% -- 1. THE SHAPING FILTER IS EXACT.  State 3 is white noise through an
%    integrator and nothing feeds it, so X(3,3)(t) = x0(3,3) + PHIS*t
%    EXACTLY: RK4 integrates a constant with no truncation error at all.
%    This pins Q's placement, the RK4 weights and the time base against
%    ANALYTIC truth with no figure reading anywhere in it.  It also catches
%    the note-2 hazard directly: PHIS = nT^2*tF instead of nT^2/tF shows up
%    here as a factor of tF^2, long before it becomes 133 ft of "miss".
%    Measured residual at the book's case: 1.7e-14.
ref33 = opts.x0(3,3) + PHIS*R.t;
C.shaping_rel = max(abs(R.sig_nt(1,:).^2 - ref33) ./ max(ref33, eps));
C.shaping_tol = 1e-10;
C.shaping_ok  = C.shaping_rel <= C.shaping_tol;

% -- 2. SYMMETRY IS EXACT.  (F*X)+(F*X)' is symmetric bit for bit, Q is
%    symmetric, and every RK4 combination of symmetric matrices is too.  The
%    tolerance is ZERO on purpose: a nonzero residual means the shape of the
%    update line changed, and there is no honest reason for a small one.
asym = 0;
for i = 1:size(R.Xs, 3)
    Xi   = R.Xs(:,:,i,1);
    asym = max(asym, max(max(abs(Xi - Xi.'))));
end
C.symmetry_res = asym;
C.symmetry_tol = 0;
C.symmetry_ok  = (asym <= 0);

% -- 3. POSITIVE SEMIDEFINITENESS -- FINAL SAMPLE EXCLUDED, AND SAID SO.
%    Tested on the CORRELATION-normalized matrix, because the four states
%    differ by ~10 orders of magnitude in scale (y ~ 4e2 ft, delta ~ 1e-1
%    rad) and a raw eigenvalue is meaningless across that spread.  Only the
%    states that actually CARRY variance are included: with 'phis', 0 the
%    target-acceleration state is identically zero, and dividing by its zero
%    sigma would make the whole check unevaluable rather than failing it.
%
%    The final sample is excluded because it sits at tgo = eps_tgo, where the
%    book's own dodge produces correlations of 2.97 and a normalized
%    eigenvalue of -2.99.  Excluding it is a STATED LIMITATION, not a
%    widened tolerance.
%
%    The tolerance is -1e-5, and the reason it is not -1e-15 is measured, not
%    guessed: near intercept the correlations run to +/-1 and the matrix goes
%    near-rank-deficient, so the smallest eigenvalue floats at the 1e-6 level
%    (book case +1.15e-4 at t = 5.6 s; heading-error-only case -5.9e-7 at
%    t = 9.9 s).  A REAL loss of PSD in this system is order 0.1 to 3 -- the
%    excluded final sample is -2.99 -- so -1e-5 still separates the two by
%    five orders of magnitude.
mn = Inf;  mnT = NaN;  ntest = 0;
for i = 1:(size(R.Xs, 3) - 1)
    Xi = R.Xs(:,:,i,1);
    d  = sqrt(diag(Xi));
    kk = find(d > 0);
    if numel(kk) < 2, continue; end   % 0 or 1 live state: sqrtVar already covers it
    ds = d(kk);
    Ci = Xi(kk,kk) ./ (ds*ds.');
    ev = eig((Ci + Ci.')/2);
    if ~isreal(ev)
        error('mgs:zarchan:covarianceHoming:complexEig', ...
            ['eig() of a symmetrised correlation matrix returned complex ' ...
             'values at t = %g s.  That cannot happen for a real symmetric ' ...
             'matrix, so the matrix is not what this code thinks it is.'], R.t(i));
    end
    ntest = ntest + 1;
    if min(ev) < mn, mn = min(ev);  mnT = R.t(i); end
end
C.psd_min   = mn;
C.psd_min_t = mnT;
C.psd_ntest = ntest;
C.psd_tol   = -1e-5;
if ntest == 0
    % Not evaluable, and printed as NOT EVALUABLE rather than as a tick.
    % Every individual variance is still guarded by sqrtVar, which errors on
    % a negative one, so nothing here is unchecked -- only the CROSS terms
    % are, and with fewer than two live states there are none.
    C.psd_min = NaN;  C.psd_ok = true;
    C.notes{end+1} = ['PSD check not evaluable: fewer than two states carry ' ...
                      'variance at every sample, so there are no cross terms ' ...
                      'to test.  sqrtVar still guards each variance.'];
else
    C.psd_ok = (mn >= C.psd_tol);
end

% -- 4. Vc CANCELS EXACTLY.  Rescaling delta by Vc removes Vc from both the
%    y-dynamics and from nL, so NEITHER sigma_y NOR sigma_nL may move.  It is
%    a very sharp test (~1e-14) and it is also the PROOF, quoted in the
%    header, that this model carries no along-track information and therefore
%    cannot price an arrival-time spread.
%    Skipped when x0 is non-zero: a variance placed on delta does not scale
%    with Vc, so the invariance genuinely fails there and asserting it would
%    be asserting something false.
if zeroX0
    Rv = quietRun(opts, 'vc', opts.vc*7);
    C.vc_rel = maxRel([Rv.sig_y(1,:), Rv.sig_nl(1,:)], ...
                      [R.sig_y(1,:),  R.sig_nl(1,:)]);
    C.vc_tol = 1e-9;
    C.vc_ok  = C.vc_rel <= C.vc_tol;
    C.vc_skipped = false;
else
    C.vc_rel = NaN;  C.vc_tol = 1e-9;  C.vc_ok = true;  C.vc_skipped = true;
    C.notes{end+1} = ['Vc-cancellation check skipped: x0 is non-zero, and a ' ...
                      'variance on delta does not scale with Vc.'];
end

% -- 5. LINEARITY IN THE MANEUVER LEVEL.  With x0 = 0 the covariance is
%    linear in Q and Q is quadratic in nT, so every sigma scales EXACTLY
%    with nT.  Catches a dropped square or a stray sqrt.  It is also the
%    licence for a caller to sweep maneuver level WITHOUT re-running -- valid
%    only at x0 = 0, which is exactly why the skip below exists.
if zeroX0 && opts.xnt > 0 && isnan(opts.phis)
    f  = 2.5;
    Rn = quietRun(opts, 'xnt', opts.xnt*f);
    C.lin_rel = maxRel([Rn.sig_y(1,:), Rn.sig_nl(1,:)]/f, ...
                       [R.sig_y(1,:),  R.sig_nl(1,:)]);
    C.lin_tol = 1e-9;
    C.lin_ok  = C.lin_rel <= C.lin_tol;
    C.lin_skipped = false;
else
    C.lin_rel = NaN;  C.lin_tol = 1e-9;  C.lin_ok = true;  C.lin_skipped = true;
    C.notes{end+1} = ['maneuver-linearity check skipped: it needs x0 = 0, ' ...
                      'xnt > 0 and phis derived from xnt.'];
end

% -- 6. THE TERMINAL ACCELERATION SIGMA IS A FUNCTION OF eps_tgo, NOT OF THE
%    FLIGHT.  Re-run with eps_tgo x10 and assert that sigma_nL(tF) falls by
%    10x.  This is the guard that stops the 1.24e5 g terminal sample ever
%    being quoted as a result: it is ASSERTED to be an artifact, not merely
%    warned about in a comment somewhere.
%
%    ONLY THE RATIO IS SCORED.  sigma_y's insensitivity to eps is measured
%    and printed alongside, but it is NOT a pass criterion: it degrades when
%    the run itself is under-resolved (3.4% at tau = 0.2 s with h = 0.01),
%    and gating on it there would raise a red flag on check 6 for a fault
%    that is check 7's to report.  A tolerance that fails for the wrong
%    reason teaches people to ignore it.
%
%    Band 5%, measured 9.81 to 9.99 across tau = 0.2..2, tf = 1..10, N' =
%    3..5 and the heading-error case.  The alternative hypothesis this is
%    discriminating against is "no 1/eps scaling", i.e. a ratio near 1.
%
%    Skipped when the dodge is not small: at eps_tgo comparable to h the
%    terminal sample is no longer dominated by 1/eps and the 10x is not
%    expected to hold.
epsGate = opts.eps_tgo*10 <= min(1e-3, opts.h/10);
if epsGate
    Re = quietRun(opts, 'eps_tgo', opts.eps_tgo*10);
    C.eps_ratio     = R.sig_nl(1,end) / Re.sig_nl(1,end);       % expect ~10
    C.eps_ratio_tol = 0.05;                                     % relative to 10
    C.eps_sigy_rel  = abs(Re.sig_y(1,end)/R.sig_y(1,end) - 1);  % reported only
    C.eps_ok        = abs(C.eps_ratio/10 - 1) <= C.eps_ratio_tol;
    C.eps_skipped   = false;
else
    C.eps_ratio = NaN;  C.eps_ratio_tol = 0.05;  C.eps_sigy_rel = NaN;
    C.eps_ok = true;    C.eps_skipped = true;
    C.notes{end+1} = sprintf(['1/eps_tgo check skipped: eps_tgo = %g is not ' ...
        'small against h = %g, so the terminal sample is not dominated by ' ...
        '1/eps and the 10x is not expected.'], opts.eps_tgo, opts.h);
end

% -- 7. STEP-SIZE CONVERGENCE of the one number we publish.  Halving h must
%    move sigma_y(tF) by well under the benchmark band, or that band is
%    measuring the integrator rather than the port.
%
%    The convergence is O(h), NOT O(h^4): the error is dominated by the final
%    steps inside the eps_tgo region, where RK4 has no accuracy to offer.
%    Measured: 0.24% at the book's case, 0.44% at tau = 0.5 s, and 27% at
%    tau = 0.2 s -- where 20 steps per guidance time constant is genuinely
%    not enough and failing is the correct outcome, not an over-tight band.
%    The 1% band is a "the integrator has not diverged" claim; the sharp
%    accuracy statement is benchmark row 2, against the closed form.
Rh = quietRun(opts, 'h', opts.h/2);
C.h_rel = abs(Rh.sig_y(1,end)/R.sig_y(1,end) - 1);
C.h_tol = 1e-2;
C.h_ok  = C.h_rel <= C.h_tol;

C.ok = C.shaping_ok && C.symmetry_ok && C.psd_ok && C.vc_ok && ...
       C.lin_ok && C.eps_ok && C.h_ok;
if ~C.ok
    error('mgs:zarchan:covarianceHoming:selfcheck', ...
        ['a STRUCTURAL invariant failed, so every number this function ' ...
         'computed is wrong:\n' ...
         '  1 shaping filter X(3,3) == phis*t : rel %10.3e (tol %.1e) %s\n' ...
         '  2 X symmetric, bit for bit        : res %10.3e (tol %.1e) %s\n' ...
         '  3 X PSD (final sample excluded)   : min %10.3e (tol %.1e) %s\n' ...
         '  4 Vc cancels out of the loop      : rel %10.3e (tol %.1e) %s\n' ...
         '  5 sigma scales linearly with nT   : rel %10.3e (tol %.1e) %s\n' ...
         '  6 sigma_nL(tF) scales as 1/eps    : x%9.4f (want 10)   %s\n' ...
         '  7 sigma_y(tF) converged in h      : rel %10.3e (tol %.1e) %s\n' ...
         'These test the wiring, not the book.  A failure in 1-6 means the ' ...
         'transcription of Listing 5.2 has been damaged.  A failure in 7 ' ...
         'alone usually means h is too coarse for tau -- this scheme is ' ...
         'O(h) near the intercept, so halve h and try again.'], ...
        C.shaping_rel, C.shaping_tol, tf2s(C.shaping_ok), ...
        C.symmetry_res, C.symmetry_tol, tf2s(C.symmetry_ok), ...
        C.psd_min, C.psd_tol, tf2s(C.psd_ok), ...
        C.vc_rel, C.vc_tol, tf2s(C.vc_ok), ...
        C.lin_rel, C.lin_tol, tf2s(C.lin_ok), ...
        C.eps_ratio, tf2s(C.eps_ok), ...
        C.h_rel, C.h_tol, tf2s(C.h_ok));
end
end


function Q = quietRun(opts, varargin)
%QUIETRUN  Re-run with overrides: no printing, no assert, and NO SELF-CHECK,
%   which is what stops the recursion.  Every option is forwarded EXPLICITLY;
%   a forgotten one would make the inner run a different problem and the
%   invariant would then be comparing two unrelated answers and passing.
a = opts;
for i = 1:2:numel(varargin), a.(varargin{i}) = varargin{i+1}; end
Q = mgs.zarchan.covarianceHoming( ...
    'xnp', a.xnp, 'tau', a.tau, 'tf', a.tf, 'h', a.h, 'vc', a.vc, ...
    'xnt', a.xnt, 'phis', a.phis, 'x0', a.x0, 'eps_tgo', a.eps_tgo, ...
    'render', false, 'plot', false, 'assert', false, 'selfcheck', false);
end


function r = maxRel(a, b)
r = max(abs(a - b) ./ max(abs(b), eps));
end


% =============================================================================
function B = benchmarks(R, opts, phisOverridden, GRAV_FT)
%BENCHMARKS  Computed vs published, with a band and the PROVENANCE of it.
%
%   *** EVERY BOOK VALUE HERE WAS GREPPED OUT OF zarchan.txt AND THE LINE
%   NUMBER IS IN THE .source FIELD. ***  The extraction turned every figure
%   into an image link, so no number that lives only on a plot survives into
%   the text -- and on the previous wave three "benchmarks" handed to porting
%   agents were exactly such phantom figure readings.  Where the book states
%   a SHAPE about a figure ("monotonically increasing") that prose IS
%   greppable, and a shape check is often stronger than a value eyeballed off
%   a plot anyway.
%
%   FIVE TOLERANCE CLASSES:
%     'text'      1%    printed in the running text to three figures (13.3 ft)
%     'analytic'  2%    compared against a closed-form FORMULA from the book,
%                       evaluated here; 2% covers this scheme's O(h) terminal
%                       error, measured at +0.53%
%     'graph'     n/a   read off a figure by the author; scored as a BRACKET,
%                       never as a value
%     'shape'     n/a   a prose claim about a figure; scored as an ordering
%     'units'     0.5%  the book's own arithmetic, used as a units tripwire
%
%   Do NOT widen a band to turn a row green.  The bands are set by the
%   PROVENANCE of the book value, not by what we happen to compute.

B = emptyBench();

j3 = find(opts.xnp == 3, 1);
bookCase = ~isempty(j3) && opts.tau == 1 && opts.tf == 10 && ...
           abs(opts.xnt - 96.6) < 1e-9 && ~phisOverridden && ...
           all(opts.x0(:) == 0) && abs(opts.eps_tgo - 1e-5) < 1e-15;
whyNot = ['off the listing''s own case (needs N''=3 in xnp, tau=1, tf=10, ' ...
          'xnt=96.6, x0=0, phis derived, eps_tgo=1e-5)'];

% ---- 1.  THE HEADLINE ----------------------------------------------------
% zarchan.txt:1536 -- "the covariance analysis program indicates that the
% standard deviation of the miss distance is 13.3 ft, which is identical to
% the adjoint results of Chapter 4 (see Fig. 4.18)."
i = numel(B) + 1;
B(i) = mkbench('sigma_y(tF), 3-g uniform maneuver', 13.3, 'ft', 'text', ...
               0.01, 'p.98 text (zarchan.txt:1536)');
if bookCase
    B(i) = scoreNum(B(i), R.sig_y(j3, end), 'ft', ...
        'sMiss = sy(tF).  The book quotes this to three figures.');
else
    B(i) = markSkipped(B(i), whyNot);
end

% ---- 2.  THE SECOND OPINION ----------------------------------------------
% zarchan.txt:1354-1360, the Ch.4 closed form for N' = 3.  It shares no code
% with the Riccati loop, so this is the row with real teeth: a transcription
% error anywhere in F, Q or the RK4 weights moves it.
i  = numel(B) + 1;
cf = NaN;
if bookCase, cf = closedFormMiss(opts.xnt, opts.tau, opts.tf); end
B(i) = mkbench('sigma_y(tF) vs Ch.4 closed form', cf, 'ft', 'analytic', ...
               0.02, 'p.85-86 closed form (zarchan.txt:1354)');
if bookCase
    B(i) = scoreNum(B(i), R.sig_y(j3, end), 'ft', ...
        'analytic solution of the same problem; shares no code with the integrator');
else
    B(i) = markSkipped(B(i), whyNot);
end

% ---- 3.  THE SHAPE CLAIM -------------------------------------------------
% zarchan.txt:1556 -- "the standard deviation of the missile acceleration is
% monotonically increasing for the 10-s flight."  Prose about Fig. 5.6, so it
% is greppable and usable.  Scored over the WHOLE record, terminal sample
% included, because that is the interval the sentence names.
i = numel(B) + 1;
B(i) = mkbench('sigma_nL monotonically increasing', NaN, '', 'shape', NaN, ...
               'p.99 text (zarchan.txt:1556)');
if ~isempty(j3) && all(opts.x0(:) == 0)
    dmin = min(diff(R.sig_nl_g(j3,:)));
    B(i) = scoreShape(B(i), dmin > 0, 'increasing throughout', ...
        sprintf('min consecutive rise %+0.4e g', dmin), ...
        'the whole 10-s record, which is the interval the sentence names');
else
    B(i) = markSkipped(B(i), ['needs N''=3 and x0 = 0: with an initial ' ...
        'condition the maneuver is no longer the only error source']);
end

% ---- 4.  THE FIGURE READING, SCORED AS A BRACKET -------------------------
% zarchan.txt:1556 -- "68% of the time 15 g (that is, 483 ft/s2) or 5 times
% the acceleration of the target is required to avoid acceleration
% saturation."  The 15 g is READ OFF Fig. 5.6, so it is NOT scored as a
% value.  What IS scored is where the curve crosses 483 ft/s^2: it must
% cross, and it must cross only in the last half second, which is what a
% monotone curve that leaves the top of a 20-g axis looks like.  A port whose
% acceleration sigma were 15% low would never cross at all; one 2% high would
% cross before 9.7 s.  That is the honest strength of a figure reading.
i = numel(B) + 1;
B(i) = mkbench('sigma_nL crosses 15 g only at the end', NaN, '', 'graph', ...
               NaN, 'Fig. 5.6 / p.99 text (zarchan.txt:1556)');
if bookCase
    g15 = 15;                                  % g, the book's reading
    gg  = R.sig_nl_g(j3,:);
    kk  = find(gg <= g15, 1, 'last');
    if isempty(kk) || kk >= numel(gg)
        B(i) = scoreShape(B(i), false, ...
            sprintf('crossing in (%.1f, %.1f) s', opts.tf-0.5, opts.tf), ...
            'never crosses 15 g', ...
            'a monotone curve that never reaches the book''s reading');
    else
        tx  = R.t(kk) + (g15 - gg(kk))/(gg(kk+1) - gg(kk))*(R.t(kk+1) - R.t(kk));
        okx = tx > (opts.tf - 0.5) && tx < opts.tf;
        B(i) = scoreShape(B(i), okx, ...
            sprintf('crossing in (%.1f, %.1f) s', opts.tf-0.5, opts.tf), ...
            sprintf('crosses at t = %.4f s', tx), ...
            sprintf('brackets: %.2f g at %.1f s, %.2f g at %.1f s', ...
                    gg(kk), R.t(kk), gg(kk+1), R.t(kk+1)));
    end
else
    B(i) = markSkipped(B(i), whyNot);
end

% ---- 5.  THE UNITS TRIPWIRE ----------------------------------------------
% Same sentence: "15 g (that is, 483 ft/s2) ... 5 times the acceleration of
% the target".  This row checks OUR CONSTANTS, not the simulation.  It fails
% the instant somebody converts this file to SI (15*9.81 = 147, not 483) and
% its detail line re-derives the book's "5 times" from xnt, so an unannounced
% change of target level shows up there too.  Cheap, and it guards the one
% mistake this package would find hardest to see.
i = numel(B) + 1;
B(i) = mkbench('book arithmetic 15 g = 483 ft/s^2', 483, 'ft/s^2', 'units', ...
               0.005, 'p.99 text (zarchan.txt:1556)');
B(i) = scoreNum(B(i), 15*GRAV_FT, 'ft/s^2', ...
    sprintf('and 15 g / target %.4g g = %.4g (the book says 5 times)', ...
            opts.xnt/GRAV_FT, 15/max(opts.xnt/GRAV_FT, eps)));

% ---- INFO rows.  Reported, never scored. ---------------------------------
i = numel(B) + 1;
[pk, ipk] = max(R.sig_y(1,:));
B(i) = mkinfo('peak sigma_y over the flight', ...
    sprintf('%.2f ft at t = %.2f s', pk, R.t(ipk)), ...
    'Fig. 5.5 shape only -- the book publishes no peak value', ...
    'the MISS is sigma_y at tF, which is NOT this peak');

i = numel(B) + 1;
B(i) = mkinfo('terminal sigma_nL (ARTIFACT)', ...
    sprintf('%.4e g at tgo = %.0e s', R.sig_nl_g(1,end), R.tgo(end)), ...
    'not a book value and not a result', ...
    'equals N''*sigma_y/(T*eps_tgo); structural check 6 asserts the 1/eps scaling');

if numel(opts.xnp) > 1
    for j = 1:numel(opts.xnp)
        i = numel(B) + 1;
        B(i) = mkinfo(sprintf('N''=%g miss / sigma_nL at 0.5 s to go', opts.xnp(j)), ...
            sprintf('%.4f ft / %.3f g', R.sig_y(j,end), R.sig_nl_g(j, max(1,end-5))), ...
            'no published value for this sweep', ...
            'reported because the run produced it; the listing hard-codes XNP = 3');
    end
end
end


function B = emptyBench()
B = struct('what',{}, 'book',{}, 'book_s',{}, 'ours',{}, 'ours_s',{}, ...
           'pct',{}, 'band',{}, 'tol',{}, 'ok',{}, 'skipped',{}, ...
           'scored',{}, 'why',{}, 'detail',{}, 'source',{}, 'unit',{});
end

function b = mkbench(what, book, unit, band, tol, source)
%MKBENCH  Field names and ORDER must match emptyBench exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want: a struct
%   array that quietly reordered fields would make [B.ok] mean something else.
b = struct('what',what, 'book',book, 'book_s','', 'ours',NaN, 'ours_s','', ...
           'pct',NaN, 'band',band, 'tol',tol, 'ok',false, 'skipped',false, ...
           'scored',true, 'why','', 'detail','', 'source',source, 'unit',unit);
if isfinite(book)
    b.book_s = sprintf('%.4g %s', book, unit);
else
    b.book_s = '(shape, see verdict)';
end
end

function b = mkinfo(what, ours_s, source, detail)
b = struct('what',what, 'book',NaN, 'book_s','--', 'ours',NaN, ...
           'ours_s',ours_s, 'pct',NaN, 'band','info', 'tol',NaN, ...
           'ok',false, 'skipped',false, 'scored',false, 'why','', ...
           'detail',detail, 'source',source, 'unit','');
end

function b = scoreNum(b, ours, unit, detail)
if ~isfinite(b.book)
    error('mgs:zarchan:covarianceHoming:bench', ...
        ['benchmark "%s" has no numeric book value to score against, so ' ...
         'scoring it as a number would invent one.'], b.what);
end
b.ours   = ours;
b.ours_s = sprintf('%.4f %s', ours, unit);
b.pct    = 100*(ours/b.book - 1);
b.ok     = abs(ours/b.book - 1) <= b.tol;
b.detail = detail;
end

function b = scoreShape(b, ok, wantStr, gotStr, detail)
b.book_s = wantStr;
b.ours_s = gotStr;
b.ok     = logical(ok);
b.detail = detail;
end

function b = markSkipped(b, why)
% Skipped rows are NOT passes.  R.pass requires at least one SCORED row and
% these are excluded from the count entirely.
b.ours = NaN;  b.ours_s = 'skipped';  b.pct = NaN;
b.ok   = false;  b.skipped = true;  b.why = why;
end


% =============================================================================
function s = render(R, opts, PHIS, GRAV_FT)
W = 100;
L = {};
L{end+1} = 'Zarchan Listing 5.2 -- HOMING LOOP COVARIANCE ANALYSIS';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['N'' = %s   tau = %g s   tF = %g s   h = %g s   ' ...
    'Vc = %g ft/s   nT = %g ft/s^2 (%.4g g)'], mat2str(R.xnp), opts.tau, ...
    opts.tf, opts.h, opts.vc, opts.xnt, opts.xnt/GRAV_FT);
L{end+1} = sprintf('PHIS = %.6g (ft/s^2)^2/s [%s]    eps_tgo = %g s    x0 = %s', ...
    PHIS, ternary(R.phis_overridden, 'CALLER OVERRIDE', 'nT^2/tF, as the book'), ...
    opts.eps_tgo, ternary(all(opts.x0(:) == 0), 'zero (book)', 'NON-ZERO (caller)'));
L{end+1} = 'integrator: Zarchan''s 4th-order Runge-Kutta on Xdot = FX + XF'' + Q, transcribed';
L{end+1} = repmat('-', 1, W);

if isfield(R.selfcheck, 'ok')
    C = R.selfcheck;
    L{end+1} = 'STRUCTURAL SELF-CHECK  (wiring, not the book -- a failure here voids everything)';
    L{end+1} = sprintf('  1 shaping filter X(3,3) == x0(3,3)+phis*t : rel %10.3e  tol %.0e  %s', ...
        C.shaping_rel, C.shaping_tol, tf2s(C.shaping_ok));
    L{end+1} = sprintf('  2 X symmetric, bit for bit                : res %10.3e  tol %.0e  %s', ...
        C.symmetry_res, C.symmetry_tol, tf2s(C.symmetry_ok));
    if C.psd_ntest == 0
        L{end+1} = '  3 X PSD, FINAL SAMPLE EXCLUDED            : NOT EVALUABLE (<2 live states)'; %#ok<AGROW>
    else
        L{end+1} = sprintf(['  3 X PSD, FINAL SAMPLE EXCLUDED            : min %10.3e  ' ...
            'tol %.0e  %s  (worst at t = %.2f s, %d samples)'], ...
            C.psd_min, C.psd_tol, tf2s(C.psd_ok), C.psd_min_t, C.psd_ntest); %#ok<AGROW>
    end
    if C.vc_skipped
        L{end+1} = '  4 Vc cancels out of the loop              : SKIPPED (x0 non-zero)';
    else
        L{end+1} = sprintf('  4 Vc cancels out of the loop (x7)         : rel %10.3e  tol %.0e  %s', ...
            C.vc_rel, C.vc_tol, tf2s(C.vc_ok));
    end
    if C.lin_skipped
        L{end+1} = '  5 sigma scales linearly with nT           : SKIPPED (needs x0 = 0, phis derived)';
    else
        L{end+1} = sprintf('  5 sigma scales linearly with nT (x2.5)    : rel %10.3e  tol %.0e  %s', ...
            C.lin_rel, C.lin_tol, tf2s(C.lin_ok));
    end
    if C.eps_skipped
        L{end+1} = '  6 sigma_nL(tF) is 1/eps_tgo, not physics  : SKIPPED (eps_tgo not small against h)'; %#ok<AGROW>
    else
        L{end+1} = sprintf(['  6 sigma_nL(tF) is 1/eps_tgo, not physics  : x%9.4f  ' ...
            'want 10 +/-%.0f%%   %s  (sigma_y moved %.3f%%, reported not scored)'], ...
            C.eps_ratio, 100*C.eps_ratio_tol, tf2s(C.eps_ok), 100*C.eps_sigy_rel); %#ok<AGROW>
    end
    L{end+1} = sprintf('  7 sigma_y(tF) converged in h (h/2)        : rel %10.3e  tol %.0e  %s', ...
        C.h_rel, C.h_tol, tf2s(C.h_ok));
    for k = 1:numel(C.notes)
        L{end+1} = ['    note: ' C.notes{k}]; %#ok<AGROW>
    end
else
    L{end+1} = 'STRUCTURAL SELF-CHECK  ***SKIPPED*** (selfcheck=false) -- nothing here is verified';
end
L{end+1} = repmat('-', 1, W);

L{end+1} = 'BOOK BENCHMARKS  (every book value was grepped from zarchan.txt; the line is in ''source'')';
L{end+1} = sprintf('  %-36s %-26s %-26s %-9s %s', 'what', 'book', 'ours', 'band', 'verdict');
for i = 1:numel(R.bench)
    b = R.bench(i);
    if b.skipped
        L{end+1} = sprintf('  %-36s %-26s %-26s %-9s SKIP', ...
            b.what, b.book_s, 'skipped', b.band); %#ok<AGROW>
        L{end+1} = sprintf('  %-36s   why: %s', '', b.why); %#ok<AGROW>
    elseif ~b.scored
        L{end+1} = sprintf('  %-36s %-26s %-26s %-9s INFO', ...
            b.what, b.book_s, b.ours_s, b.band); %#ok<AGROW>
    elseif isfinite(b.pct)
        L{end+1} = sprintf('  %-36s %-26s %-26s %-9s %s  (%+.2f%%, tol %.1f%%)', ...
            b.what, b.book_s, b.ours_s, b.band, tf2s(b.ok), b.pct, 100*b.tol); %#ok<AGROW>
    else
        L{end+1} = sprintf('  %-36s %-26s %-26s %-9s %s', ...
            b.what, b.book_s, b.ours_s, b.band, tf2s(b.ok)); %#ok<AGROW>
    end
    if ~isempty(b.detail)
        L{end+1} = sprintf('  %-36s   %s', '', b.detail); %#ok<AGROW>
    end
    L{end+1} = sprintf('  %-36s   source: %s', '', b.source); %#ok<AGROW>
end
L{end+1} = '  bands: text 1% (3 figures in the running text) | analytic 2% (book formula,';
L{end+1} = '  evaluated here) | graph & shape (figure reading / prose, scored as brackets and';
L{end+1} = '  orderings, never as values) | units 0.5% (tripwire on g = 32.2)';
L{end+1} = repmat('-', 1, W);

L{end+1} = 'STANDARD DEVIATION HISTORIES  (1-s grid; R.t / R.sig_* carry the full 0.1-s record)';
for j = 1:numel(R.xnp)
    L{end+1} = sprintf('  N'' = %g', R.xnp(j)); %#ok<AGROW>
    L{end+1} = sprintf('    %6s %14s %14s %14s %16s', 't [s]', 'sig_y [ft]', ...
        'sig_ydot', 'sig_nL [g]', 'sig_nL [ft/s2]'); %#ok<AGROW>
    for x = 1:floor(R.t(end))
        k = nearestIdx(R.t, x);
        L{end+1} = sprintf('    %6.1f %14.4f %14.4f %14.4f %16.2f', ...
            R.t(k), R.sig_y(j,k), R.sig_ydot(j,k), R.sig_nl_g(j,k), R.sig_nl(j,k)); %#ok<AGROW>
    end
    L{end+1} = sprintf('    MISS  sigma_y(tF) = %.4f ft', R.miss_ft(j)); %#ok<AGROW>
    L{end+1} = ''; %#ok<AGROW>
end

L{end+1} = repmat('-', 1, W);
L{end+1} = 'READ THIS BEFORE QUOTING ANY NUMBER ABOVE';
L{end+1} = sprintf(['  * THE LAST ROW OF THE TABLE SITS AT tgo = %g s.  sigma_y there IS the ' ...
    'miss and is'], opts.eps_tgo);
L{end+1} = '    sound.  sigma_nL and sigma_ydot there are NOT: they read through 1/tgo and are';
L{end+1} = '    a function of eps_tgo (self-check 6 measures the 1/eps scaling).  Use';
L{end+1} = '    R.sig_nl_g_masked / R.sig_ydot_masked, whose last column is NaN by design.';
L{end+1} = '  * Vc cancels EXACTLY (self-check 4).  A model in which closing speed provably';
L{end+1} = '    does not matter has no along-track channel, so nothing here is an ARRIVAL-TIME';
L{end+1} = '    dispersion.  Every sigma is LOS-normal.  Do not price a 3-s arrival spread';
L{end+1} = '    against it.';
L{end+1} = '  * The only error source is the target maneuver unless you passed x0 or phis.';
L{end+1} = '    Our own discriminant is release-side |eps0|, an initial condition: build it';
L{end+1} = '    with R.x0_from_heading_error(vm_fts, sigma_rad) and pass ''phis'', 0.';
L{end+1} = '  * No gravity, no drag, no thrust, no nz clamp, no LOS floor, and the single lag';
L{end+1} = '    NEVER SATURATES.  Saturation is what our flight data says dominates.  A miss';
L{end+1} = '    distance off this board is not a sortie prediction.';
s = strjoin(L, newline);
end


function drawFigures(R, opts, GRAV_FT)
%DRAWFIGURES  Figs. 5.5 and 5.6, including the book's own axis limits.
figure('Name', 'Zarchan Fig. 5.5 -- relative separation');
plot(R.t, R.sig_y', 'LineWidth', 1.2); grid on;
xlabel('Time (s)'); ylabel('Standard Deviation of Relative Position (Ft)');
title(sprintf('Covariance analysis, tau = %g s, %.4g-g uniform target maneuver', ...
    opts.tau, opts.xnt/GRAV_FT));
legend(arrayfun(@(v) sprintf('N'' = %g', v), R.xnp, 'UniformOutput', false), ...
    'Location', 'best');

figure('Name', 'Zarchan Fig. 5.6 -- missile acceleration');
plot(R.t, R.sig_nl_g', 'LineWidth', 1.2); grid on;
xlabel('Time (s)'); ylabel('Standard Deviation of Acceleration (G)');
% axis([0 10 0 20]) is the BOOK'S OWN LINE, and it is the reason Fig. 5.6
% looks finite: it crops the terminal 1/eps_tgo sample clean off the top.
% Kept, with this note in the title, so the plot matches the book AND nobody
% concludes from the picture that the curve is bounded.
axis([0 opts.tf 0 20]);
title('Book axis [0 tF 0 20]: the terminal 1/eps\_tgo sample is CROPPED, not small');
legend(arrayfun(@(v) sprintf('N'' = %g', v), R.xnp, 'UniformOutput', false), ...
    'Location', 'best');
end


% =============================================================================
function k = nearestIdx(t, x)
[~, k] = min(abs(t - x));
end

function s = tf2s(b)
if b, s = 'PASS'; else, s = 'FAIL'; end
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
