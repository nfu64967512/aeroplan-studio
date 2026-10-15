function R = adjointBinomial5(opts)
%MGS.ZARCHAN.ADJOINTBINOMIAL5  Zarchan Listing 6.1 -- adjoint of the
%   fifth-order binomial guidance system, ported line for line.
%
%   R = mgs.zarchan.adjointBinomial5()          % book defaults, prints the table
%   R = mgs.zarchan.adjointBinomial5('xnp', [3 4 5 6])
%   R = mgs.zarchan.adjointBinomial5('render', false, 'assert', false)
%
%   Produces the NORMALIZED miss-distance design curves of Figs. 6.6-6.8
%   against three target-maneuver shapes -- step, ramp, parabolic -- and the
%   optimal-evasive-maneuver coefficients of Figs. 6.9-6.11.
%
% ===========================================================================
%   *** UNITS ARE THE BOOK'S.  FEET.  NOT SI. ***
%
%   Zarchan works in feet and ft/s with g = 32.2 ft/s^2.  The rest of mgs is
%   metres.  This file is DELIBERATELY left in the book's units so the
%   published numbers can be compared digit for digit without a conversion
%   sitting in between to be got wrong.
%
%   Everything this function returns or prints is feet.  There is no metre
%   anywhere in it and it must not acquire one: a miss distance of "12" is
%   plausible in either unit, so a silent conversion would produce a number
%   that looks right and is wrong by 3.28x.  That is exactly the class of
%   failure this package exists to catch.  Convert at the CALL SITE, visibly.
% ===========================================================================
%
%   WHY THIS PORT IS WORTH HAVING
%
%   Our own model scores 32/32 hits at 0.03 m where the real flight set scores
%   25/32 at 0.93 m.  The measured difference is guidance-system LAG: the real
%   vehicle tracks flight-path angle with a gain of 0.063 against the model's
%   1.385, roughly 8 s of lag, saturating on 26 of 32 sorties.  The model has
%   no such lag, so it cannot build a failure population at all.
%
%   Zarchan Ch. 6 is exactly this subject and gives us two things:
%
%   (1) AN INDEPENDENT CHECK.  The book publishes the answers.  If our
%       integrator and our PN kernel are right, this port lands on them.  It
%       does -- see the benchmark table.  That check is worth more than the
%       curves, because it is the only part of this file falsifiable against
%       something we did not write ourselves.
%
%   (2) THE LAG MACHINERY.  A single-lag model FLATTERS a real airframe.  The
%       fifth-order binomial system -- five equal lags of T/5, so the lags sum
%       to exactly T -- is Zarchan's realistic stand-in for seeker + noise
%       filter + flight-control system together (zarchan.txt:1648).  Miss
%       distances from it run an order of magnitude above the single-lag
%       answer (:5357).  With a measured T of ~8 s and dive times of ~35-47 s
%       we sit at tF/T ~ 4.4-5.9, which on these curves is the steep part.
%
%   WHAT THIS DOES *NOT* MODEL, AND MUST NOT BE REPORTED AS IF IT DID.
%   Listing 6.1 has exactly THREE disturbance channels and all three are
%   TARGET MANEUVER: step, ramp, parabolic.  Our bullseye is STATIONARY, and
%   our own data says the discriminating disturbance is release-side heading
%   error |eps0| -- an initial-condition channel that does not appear in this
%   adjoint at all.  So this file validates the loop and prices what lag costs
%   against a maneuvering target.  It is NOT a model of our failure mode, and
%   a miss distance computed here is NOT a prediction about a sortie.
%   Zarchan's heading-error channel is a different adjoint.
%
% ---------------------------------------------------------------------------
%   THE DIMENSIONS DIFFER PER SHAPE.  THIS IS THE EASY THING TO GET WRONG.
%
%       step        miss_ft = c * T^2 * nT [g]
%       ramp        miss_ft = c * T^3 * rT [g/s]
%       parabolic   miss_ft = c * T^4 * pT [g/s^2]
%
%   A normalized coefficient applied with the wrong power of T is off by a
%   factor of T and still comes out looking like a plausible number of feet.
%   So the power is NOT written in this comment and then retyped in the code:
%   it lives in exactly one place, SHAPETABLE below, and every consumer -- the
%   curve normalization, the benchmark table, and the public converter R.miss
%   -- reads it from there.  Editing the power in one place changes it
%   everywhere or fails loudly; it cannot go half-changed.
%
%   R.miss is that converter, exposed so callers do not reimplement it:
%       R.miss('ramp', 120, 0.5, 1.6)   ->  24    (ft)
%   It ERRORS on an unknown shape name rather than defaulting to a power.
%
% ---------------------------------------------------------------------------
%   OUTPUT
%
%   R.tf_over_T   1xM abscissa, normalized flight time tF/T (== normalized
%                 time-to-go at which the disturbance strikes -- for this
%                 system the two are interchangeable, zarchan.txt:1733)
%   R.xnp         1xK effective navigation ratios that were run
%   R.shapes      1x3 struct: .name .power .level -- the single source of T^n
%   R.curve(k)    k = 1..3 in R.shapes order
%                   .name .power
%                   .y      KxM normalized coefficient, row j is R.xnp(j)
%   R.opt(j)      optimal evasive maneuver, per navigation ratio
%                   .xnp .coeff .extrema_tf_over_T .extrema_value .tv_exact
%   R.bench       struct array, computed vs book, with verdicts
%   R.selfcheck   structural invariants and their measured residuals
%   R.pass        true iff every evaluable benchmark row passed
%   R.summary     the printed text
%
%   See also MGS.ZARCHAN.CROSSCHECK, MGS.ZARCHAN.SIZING, PUREPN2D.

arguments
    % Book names are UPPERCASE; these are the same variables, lowercased.
    opts.xnp    (1,:) double {mustBeNonnegative, mustBeFinite} = [3 4 5]  % XNP,  N'
    opts.tau    (1,1) double {mustBePositive, mustBeFinite}    = 1.0      % TAU,  T  [s]
    opts.tf     (1,1) double {mustBePositive, mustBeFinite}    = 10.0     % TF       [s]
    opts.h      (1,1) double {mustBePositive, mustBeFinite}    = 0.01     % H        [s]
    opts.vc     (1,1) double {mustBePositive, mustBeFinite}    = 4000.0   % VC   [ft/s]
    opts.render (1,1) logical = true
    opts.plot   (1,1) logical = false
    opts.assert (1,1) logical = true
    % Internal.  The structural self-check re-runs this function with altered
    % vc/tau; those inner runs must not self-check again or the recursion never
    % bottoms out.  Callers have no reason to touch this -- turning it off is
    % turning off the only test that survives off-book parameters.
    opts.selfcheck (1,1) logical = true
end

% g in the book's units.  XNT/XNTD/XNTDD are all 32.2 in Listing 6.1 because
% they are 1 g, 1 g/s and 1 g/s^2 respectively -- the SAME NUMBER carrying
% THREE DIFFERENT UNITS.  That coincidence is why the normalization powers
% have to be tracked structurally rather than by eye.
GRAV_FT = 32.2;

% The book's sampler is `if S >= .0999`, kept verbatim below.  It emits one
% sample per 0.1 s of adjoint time only when h divides 0.1.  Other step sizes
% still integrate correctly, they just land on a coarser grid -- the curves
% stay right, only the abscissa spacing changes.  Nothing downstream assumes
% the spacing, so this degrades visibly instead of silently.
rr = rem(0.1, opts.h);
if abs(rr) > 1e-12 && abs(rr - opts.h) > 1e-12
    warning('mgs:zarchan:adjointBinomial5:sampleGrid', ...
        ['h = %g does not divide 0.1 s, so the output grid is not the book''s ' ...
         '0.1-s grid.  Curve values are unaffected; only the sampling is.'], opts.h);
end

R = struct();
R.units   = 'feet, ft/s, g = 32.2 ft/s^2 -- Zarchan''s units, NOT SI';
R.listing = 'Zarchan Listing 6.1 (zarchan.txt:1671), adjoint of Fig. 6.5';
R.opts    = opts;
R.shapes  = shapeTable();
R.miss    = @missFt;
R.xnp     = opts.xnp;

% ---- the runs -------------------------------------------------------------
K = numel(opts.xnp);
A = cell(1, K);
for j = 1:K
    A{j} = adjointRun(opts.xnp(j), opts.tau, opts.tf, opts.vc, opts.h, ...
                      GRAV_FT, GRAV_FT, GRAV_FT);
end
R.tf_over_T = A{1}.tp / opts.tau;         % tF/T, the book's abscissa
M = numel(R.tf_over_T);

S = R.shapes;
for k = 1:numel(S)
    Y = zeros(K, M);
    for j = 1:K
        % THE POWER COMES FROM THE TABLE, NEVER FROM A LITERAL HERE.
        % With tau = 1 (the book's normalizing choice) this divide is a no-op,
        % which is precisely why a wrong power would go unnoticed on the
        % default run.  The tau-scaling self-check below exists so that a
        % wrong power fails even though the default run cannot see it.
        Y(j, :) = A{j}.y(k, :) / opts.tau^S(k).power;
    end
    R.curve(k).name  = S(k).name;
    R.curve(k).power = S(k).power;
    R.curve(k).y     = Y;
end

% ---- optimal evasive maneuver (Figs. 6.9-6.11) ----------------------------
for j = 1:K
    R.opt(j) = optimalEvasive(opts.xnp(j), R.tf_over_T, R.curve(1).y(j, :));
end

% ---- structural self-check ------------------------------------------------
if opts.selfcheck
    R.selfcheck = structuralCheck(opts, GRAV_FT, R);
else
    R.selfcheck = struct('skipped', true);
end

% ---- book benchmarks ------------------------------------------------------
R.bench = benchmarks(R, opts);
ev = ~[R.bench.skipped];
R.pass = any(ev) && all([R.bench(ev).ok]);

R.summary = render(R, opts);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R, opts); end

if opts.assert
    % "Nothing was evaluated" must NOT read as "everything passed".  A run
    % whose xnp or tf put every benchmark out of reach has checked NOTHING,
    % and that is the shape of green light this repo has been burned by.
    if ~any(ev)
        error('mgs:zarchan:adjointBinomial5:noBenchmark', ...
            ['not one book benchmark could be evaluated: they need N'' = 3 in ' ...
             '''xnp'' and tF/T = 5 inside the horizon (tf/tau = %g here).  ' ...
             'Refusing to hand back an unvalidated port silently -- pass ' ...
             '''assert'', false if you meant to run off-book.'], opts.tf/opts.tau);
    end
    if ~R.pass
        bad = find(~[R.bench.ok] & ev);
        error('mgs:zarchan:adjointBinomial5:benchmark', ...
            ['%d of %d book benchmarks are outside tolerance (first: %s, book ' ...
             '%.4g, ours %.4g, band %s).  Either the port broke or the ' ...
             'parameters left the region the book covers.  Full table:\n\n%s'], ...
            numel(bad), sum(ev), R.bench(bad(1)).what, R.bench(bad(1)).book, ...
            R.bench(bad(1)).ours, R.bench(bad(1)).band, R.summary);
    end
end
end


% =============================================================================
function S = shapeTable()
%SHAPETABLE  The ONE place the power of T is written down.
%
%   miss_ft = coefficient * T^power * level,  level in the units named.
%   Row order here is the row order of A.y in ADJOINTRUN and of R.curve.
S = struct( ...
    'name',  {'step',     'ramp',      'parab'}, ...
    'power', {2,          3,           4}, ...
    'level', {'nT [g]',   'rT [g/s]',  'pT [g/s^2]'}, ...
    'book',  {'MNT (X1)', 'MNTD (X9)', 'MNTDD (X10)'});
end


% =============================================================================
function m = missFt(shape, coeff, T_s, level)
%MISSFT  Normalized coefficient -> actual miss distance IN FEET.
%
%   m = missFt('step',  12, 0.5, 4.0)     % 4 g step      -> 12 ft
%   m = missFt('ramp', 120, 0.5, 1.6)     % 1.6 g/s ramp  -> 24 ft
%   m = missFt('parab',300, 0.5, 0.64)    % 0.64 g/s^2    -> 12 ft
%
%   T_s in seconds; level in g, g/s or g/s^2 to match the shape.
%   Returns FEET, because the coefficient is in feet (see the file header).
S = shapeTable();
k = find(strcmpi({S.name}, shape), 1);
if isempty(k)
    % No default power.  Guessing would return feet-shaped nonsense off by a
    % factor of T -- unnoticeable at the T near 1 s the book uses, and
    % catastrophic at the T ~ 8 s we actually measure.
    error('mgs:zarchan:adjointBinomial5:shape', ...
        ['unknown maneuver shape "%s".  Known shapes: %s.  Refusing to pick a ' ...
         'power of T on your behalf -- step/ramp/parabolic normalize by ' ...
         'T^2/T^3/T^4 and a wrong choice still returns a plausible number.'], ...
        char(string(shape)), strjoin({S.name}, ', '));
end
m = coeff .* T_s.^S(k).power .* level;
end


% =============================================================================
function A = adjointRun(XNP, TAU, TF, VC, H, XNT, XNTD, XNTDD)
%ADJOINTRUN  Listing 6.1 itself.  Book variable names kept in the comments.
%
%   State mapping, Fig. 6.5 integrator outputs x1..x10 -> x(1)..x(10):
%       x(1)   MNT node   miss sensitivity to a STEP target maneuver
%       x(2)              miss sensitivity to an IMPULSE; this closes the loop
%       x(3)              the injected impulse lives here as an IC of 1
%       x(4),x(5)         the fifth lag, written so it also forms the N'*Vc*s
%       x(6),x(7),x(8)    the remaining lags of the binomial chain
%       x(9)   MNTD       miss sensitivity to a RAMP      (integral of x1)
%       x(10)  MNTDD      miss sensitivity to a PARABOLIC (integral of x9)
%
%   Two things in here look like bugs and are not:
%     * TGO = tp + 1e-5.  Zarchan's L'Hopital dodge for the 1/tgo term at
%       tgo = 0 (zarchan.txt:1662).  Remove it and the first step divides by
%       zero.
%     * x(3) starts at 1.  That unity initial condition IS the impulse that
%       makes this a miss-distance adjoint.  Zero it and every curve comes out
%       flat zero, which reads as "no miss" rather than "no simulation".
%
%   Do not replace the hand-rolled integrator with ode45.  The published
%   numbers are the numbers this scheme at this step size produces, and the
%   whole point of the file is to land on them.

nMax = ceil(TF/H) + 4;
tpA  = zeros(1, nMax);
yA   = zeros(3, nMax);

x    = zeros(10, 1);
x(3) = 1;                        % X3=1 -- the miss-distance adjoint impulse
tp   = 0.0 + 0.00001;            % TP=T+.00001
Sacc = 0.0;
n    = 0;

while tp <= (TF - 1e-5)
    xold = x;
    % Zarchan's two-pass predictor/corrector (Heun), structure verbatim: pass
    % one takes derivatives at tp, pass two at the Euler-predicted point, and
    % the corrector averages.  Note tp IS advanced between the passes and is
    % NOT rolled back, so the 1/tgo term is evaluated at both times.  That
    % detail is part of why the port lands on the book.
    xd1 = deriv(x, tp, XNP, TAU, VC);
    x   = x + H*xd1;                                  % Euler predictor
    tp  = tp + H;                                     % TP=TP+H
    xd2 = deriv(x, tp, XNP, TAU, VC);
    x   = 0.5*(xold + x + H*xd2);                     % X=.5*(XOLD+X+H*XD)

    Sacc = Sacc + H;
    if Sacc >= 0.0999                                 % if S>=.0999
        Sacc = 0.0;
        n = n + 1;
        tpA(n)  = tp;
        yA(1,n) = XNT   * x(1);                       % ArrayXMNT
        yA(2,n) = XNTD  * x(9);                       % ArrayXMNTD
        yA(3,n) = XNTDD * x(10);                      % ArrayXMNTDD
    end
end

if n < 2
    error('mgs:zarchan:adjointBinomial5:noSamples', ...
        ['the adjoint produced %d samples (tf = %g, h = %g).  A design curve ' ...
         'cannot be built from that, and returning it would hand the caller ' ...
         'an empty curve that plots as nothing at all.'], n, TF, H);
end
A.tp = tpA(1:n);
A.y  = yA(:, 1:n);            % rows in SHAPETABLE order: step, ramp, parab
end


% -----------------------------------------------------------------------------
function xd = deriv(x, tp, XNP, TAU, VC)
%DERIV  The ten differential equations, transcribed from Listing 6.1.
%   The book's own line is quoted on the right of each.  Do not "tidy" these:
%   the 5/TAU factors ARE the five lags of TAU/5, and folding them changes the
%   order of the system while leaving it looking like a working guidance loop.
xd     = zeros(10, 1);
xd(1)  = x(2);                                        % X1D=X2
xd(2)  = x(3);                                        % X2D=X3
Y1     = 5.*(5.*x(5)/TAU + x(4))/TAU;                 % Y1=5.*(5.*X5/TAU+X4)/TAU
TGO    = tp + 0.00001;                                % TGO=TP+.00001
xd(3)  = Y1/(VC*TGO);                                 % X3D=Y1/(VC*TGO)
xd(4)  = -Y1;                                         % X4D=-Y1
xd(5)  = -5.*x(5)/TAU + 5.*x(6)*XNP*VC/TAU;           % X5D
xd(6)  = -5.*x(6)/TAU + 5.*x(7)/TAU;                  % X6D
xd(7)  = -5.*x(7)/TAU + 5.*x(8)/TAU;                  % X7D
xd(8)  = -5.*x(8)/TAU - x(2);                         % X8D
xd(9)  = x(1);                                        % X9D=X1
xd(10) = x(9);                                        % X10D=X9
end


% =============================================================================
function O = optimalEvasive(xnp, tn, ystep)
%OPTIMALEVASIVE  Figs. 6.9-6.11, "Optimal Target Evasive Maneuvers" (:1733).
%
%   The target's best policy is bang-bang: hold +nT, then roll 180 deg at each
%   interior extremum of the step-miss curve, so the missile sees an apparent
%   2*nT step at the worst possible time-to-go.  The book's construction is
%
%       coeff = 2 * SUM |interior extrema of the step curve|
%
%   evaluated in the text as 41.4*2 = 82.8 at N'=3, (19.1+29.8)*2 = 97.8 at
%   N'=4, and (8.4+29.9+24)*2 = 124.6 at N'=5.
%
%   That formula quietly ASSUMES the step curve is zero at BOTH ends of the
%   horizon.  It is zero at tF/T = 0, but at N'=5 it has only fallen to about
%   2.7 by tF/T = 10 -- the tail has not died yet.  The exact bang-bang answer
%   is the total variation of the curve, reported as .tv_exact.  The two agree
%   at N'=3 and separate at N'=5.  Read .coeff when comparing against the
%   book and .tv_exact when you want the truth; do not silently swap one for
%   the other, and do not "fix" .coeff to match .tv_exact -- .coeff is what
%   the published 82.8 is, and the benchmark tests it as such.
d  = diff(ystep);
ix = find(d(1:end-1) .* d(2:end) < 0) + 1;

O.xnp               = xnp;
O.extrema_tf_over_T = tn(ix);
O.extrema_value     = ystep(ix);
if isempty(ix)
    % NaN, not 0.  A zero here would print as "the target can induce no miss",
    % which is a claim, and a false one.  NaN fails every tolerance test and
    % surfaces as FAIL instead of as a suspiciously excellent result.
    O.coeff    = NaN;
    O.tv_exact = NaN;
else
    O.coeff    = 2 * sum(abs(ystep(ix)));
    O.tv_exact = sum(abs(diff([0, ystep(ix), ystep(end)])));
end
end


% =============================================================================
function C = structuralCheck(opts, GRAV_FT, R)
%STRUCTURALCHECK  Invariants that must hold or every number above is garbage.
%
%   These are NOT benchmarks.  They test the WIRING, using properties the loop
%   has for structural reasons, and they are the only tests in this file that
%   still work when the caller runs off-book parameters.
%
%   1. OPEN LOOP.  With N' = 0 the guidance loop is cut -- the XNP factor is
%      the only path from x6 into x5 -- so x1/x9/x10 must be the unguided
%      closed forms tau^2/2, tau^3/6, tau^4/24 exactly.  This pins the double
%      integrator, the two extra integrators and the integration scheme
%      against ANALYTIC truth, with no book reading involved.  Checked only
%      for tF/T >= 1: near tF/T = 0 the curve values are ~1e-3 and a relative
%      test there measures the 1e-5 L'Hopital offset, not the integrator.
%
%   2. Vc CANCELS.  The book states closing velocity does not matter (:1663):
%      Vc multiplies in x5d and divides in x3d.  If a transcription drops
%      either one the curves move.  This is a very sharp test -- it passes at
%      ~1e-14, so any real error is unmissable.
%
%   3. TAU SCALES.  Doubling tau, with tf and h doubled to keep the same
%      normalized grid, must leave the NORMALIZED curves unchanged, because
%      T^2/T^3/T^4 is exactly what the normalization removes.  This is the
%      ONLY check that can see a wrong power in SHAPETABLE, because the
%      default run has tau = 1 where every power is a no-op.

C = struct();
C.skipped = false;

% -- 1. open loop against the analytic answer
A0  = adjointRun(0.0, opts.tau, opts.tf, opts.vc, opts.h, GRAV_FT, GRAV_FT, GRAV_FT);
t0  = A0.tp;
sel = t0 >= opts.tau;
if ~any(sel)
    error('mgs:zarchan:adjointBinomial5:horizon', ...
        ['tf = %g is shorter than tau = %g, so not one sample reaches ' ...
         'tF/T = 1 and the open-loop invariant cannot be evaluated.  An ' ...
         'unevaluated invariant is not a passing one, so this is an error ' ...
         'rather than a skip.'], opts.tf, opts.tau);
end
ref = [GRAV_FT*t0.^2/2; GRAV_FT*t0.^3/6; GRAV_FT*t0.^4/24];
C.openloop_rel = max(max(abs(A0.y(:,sel) - ref(:,sel)) ./ abs(ref(:,sel))));
C.openloop_tol = 2e-3;
C.openloop_ok  = C.openloop_rel <= C.openloop_tol;

% -- 2. Vc cancellation
b0 = benchScalars(R);
C.vc_rel = maxRel(benchScalars(quietRun(opts, 'vc', opts.vc*7)), b0);
C.vc_tol = 1e-9;
C.vc_ok  = C.vc_rel <= C.vc_tol;

% -- 3. tau scaling
Rt = quietRun(opts, 'tau', opts.tau*2, 'tf', opts.tf*2, 'h', opts.h*2);
C.tau_rel = maxRel(benchScalars(Rt), b0);
C.tau_tol = 5e-3;
C.tau_ok  = C.tau_rel <= C.tau_tol;

C.ok = C.openloop_ok && C.vc_ok && C.tau_ok;
if ~C.ok
    error('mgs:zarchan:adjointBinomial5:selfcheck', ...
        ['a STRUCTURAL invariant of the adjoint failed, so every curve this ' ...
         'function computed is wrong:\n' ...
         '  open loop vs analytic tau^n      : rel %.3e (tol %.1e) %s\n' ...
         '  Vc cancellation                  : rel %.3e (tol %.1e) %s\n' ...
         '  T^2/T^3/T^4 normalization (tau)  : rel %.3e (tol %.1e) %s\n' ...
         'These test the wiring, not the book.  A failure here means the ' ...
         'transcription of Listing 6.1 or of SHAPETABLE has been damaged.'], ...
        C.openloop_rel, C.openloop_tol, tf2s(C.openloop_ok), ...
        C.vc_rel, C.vc_tol, tf2s(C.vc_ok), ...
        C.tau_rel, C.tau_tol, tf2s(C.tau_ok));
end
end


function Q = quietRun(opts, varargin)
%QUIETRUN  Re-run with overrides: no printing, no assert, and no self-check,
%   which is what stops the recursion.
a = opts;
for i = 1:2:numel(varargin), a.(varargin{i}) = varargin{i+1}; end
Q = mgs.zarchan.adjointBinomial5( ...
    'xnp', a.xnp, 'tau', a.tau, 'tf', a.tf, 'h', a.h, 'vc', a.vc, ...
    'render', false, 'plot', false, 'assert', false, 'selfcheck', false);
end


function v = benchScalars(Rx)
%BENCHSCALARS  The four numbers the self-checks compare, for the FIRST xnp.
%   Deliberately scalars rather than whole curves: relative error on a curve
%   that crosses zero is meaningless, and a norm over the curve would hide
%   which shape moved.
v = [readAt(Rx, 1, 1, 5), readAt(Rx, 2, 1, 5), readAt(Rx, 3, 1, 5), Rx.opt(1).coeff];
end


function r = maxRel(a, b)
r = max(abs(a - b) ./ max(abs(b), eps));
end


function y = readAt(Rx, k, j, x)
%READAT  Curve k, navigation-ratio row j, evaluated at tF/T = x.
t = Rx.tf_over_T;
if x < t(1) || x > t(end)
    y = NaN;   % out of horizon -> NaN so it fails loudly; never extrapolated
    return;
end
y = interp1(t, Rx.curve(k).y(j, :), x, 'linear');
end


% =============================================================================
function B = benchmarks(R, opts)
%BENCHMARKS  Computed vs published, with a band and a reason for the band.
%
%   TWO TOLERANCE CLASSES, and the difference matters:
%
%   'graph' 15%  -- Zarchan READ these off Figs. 6.6-6.8 by eye and printed
%                   them as round numbers (12, 120, 300, 42, 40, 150, 75).
%                   They carry one or two significant figures.  Demanding 1%
%                   of a number the author eyeballed off a plot would fail a
%                   correct port, so the band is honest about what the source
%                   actually is.
%   'text'   1%  -- printed in the running text to three figures (41.4, 29.8,
%                   19.1, 8.4, 29.9) and used in arithmetic there.  These are
%                   SHARP, and the optimal-evasive rows are the real test of
%                   this port.
%
%   Do NOT widen a band to turn a row green.  The bands are set by the
%   PROVENANCE of the book value, not by what we happen to compute.

T  = 0.5;      % the book's FIRST worked example: guidance time constant 0.5 s
T2 = 1.0;      % its SECOND: same tF = 2.5 s, doubled T, hence tF/T = 2.5
NT = 4.0;      % 4 g step
RT = 1.6;      % g/s   ramp      reaching 4 g in 2.5 s
PT = 0.64;     % g/s^2 parabola  reaching 4 g in 2.5 s
X5 = 5.0;      % tF/T = 2.5/0.5

% The tF/T = 2.5 row below is deliberately given T2 = 1 s and NOT T.  It is
% the book's own second example -- "doubling the guidance system time constant
% increased the miss by more than an order of magnitude", 42*1^2*4 = 168 ft
% (p.115).  Scoring that row at T = 0.5 s would print a book column of 42 ft,
% a number that appears nowhere in Zarchan and that the next reader would
% waste an afternoon failing to find.  The normalized columns are the actual
% test; the feet columns have to stay quotable.

% For whoever reads the printed table: at T = 0.5 s and nT = 4 g the product
% T^2*nT is exactly 1.0, so on the step and optimal rows the miss-in-feet
% equals the coefficient.  That is a coincidence of the book's chosen example,
% NOT an identity -- the ramp and parabolic rows show the powers doing work.

j3 = find(opts.xnp == 3, 1);
j4 = find(opts.xnp == 4, 1);
j5 = find(opts.xnp == 5, 1);

B = emptyBench();
B = addRow(B, R, 'step  N''=3 @ tF/T=5',   1, j3, X5,  12,  'graph', T, NT, 'step',  '4 g');
B = addRow(B, R, 'ramp  N''=3 @ tF/T=5',   2, j3, X5,  120, 'graph', T, RT, 'ramp',  '1.6 g/s');
B = addRow(B, R, 'parab N''=3 @ tF/T=5',   3, j3, X5,  300, 'graph', T, PT, 'parab', '0.64 g/s^2');
B = addOpt(B, R, 'opt.evasive N''=3',         j3,      82.8, 'text',  T, NT);

% Secondary published values -- free, from the same run, and they exercise
% N' = 4 and 5 as well.
B = addRow(B, R, 'step  N''=3 @ tF/T=2.5', 1, j3, 2.5, 42,  'graph', T2, NT, 'step', '4 g, T=1s');
B = addRow(B, R, 'ramp  N''=4 @ tF/T=5',   2, j4, X5,  40,  'graph', T, RT, 'ramp',  '1.6 g/s');
B = addRow(B, R, 'parab N''=4 @ tF/T=5',   3, j4, X5,  150, 'graph', T, PT, 'parab', '0.64 g/s^2');
B = addRow(B, R, 'parab N''=5 @ tF/T=5',   3, j5, X5,  75,  'graph', T, PT, 'parab', '0.64 g/s^2');
B = addOpt(B, R, 'opt.evasive N''=4',         j4,      97.8,  'text', T, NT);
B = addOpt(B, R, 'opt.evasive N''=5',         j5,      124.6, 'text', T, NT);
end


function B = emptyBench()
B = struct('what', {}, 'level', {}, 'book', {}, 'ours', {}, 'pct', {}, ...
           'band', {}, 'tol', {}, 'ok', {}, 'skipped', {}, 'why', {}, ...
           'miss_book', {}, 'miss_ours', {});
end


function B = addRow(B, R, what, k, j, x, book, band, T, level, shape, levelStr)
i = numel(B) + 1;
B(i).what = what;  B(i).level = levelStr;  B(i).book = book;
B(i).band = band;  B(i).tol   = bandTol(band);
if isempty(j)
    B = markSkipped(B, i, 'N'' not in xnp');
    return;
end
ours = readAt(R, k, j, x);
if ~isfinite(ours)
    B = markSkipped(B, i, sprintf('tF/T = %g outside horizon', x));
    return;
end
B(i).ours    = ours;
B(i).pct     = 100*(ours/book - 1);
B(i).ok      = abs(ours/book - 1) <= B(i).tol;
B(i).skipped = false;
B(i).why     = '';
% The converter is CALLED here, not reimplemented.  If SHAPETABLE's power were
% wrong these miss columns would be wrong too -- and the tau-scaling
% self-check would already have errored out before execution reached here.
B(i).miss_book = missFt(shape, book, T, level);
B(i).miss_ours = missFt(shape, ours, T, level);
end


function B = addOpt(B, R, what, j, book, band, T, level)
i = numel(B) + 1;
B(i).what = what;  B(i).level = sprintf('%g g', level);  B(i).book = book;
B(i).band = band;  B(i).tol   = bandTol(band);
if isempty(j)
    B = markSkipped(B, i, 'N'' not in xnp');
    return;
end
ours = R.opt(j).coeff;
if ~isfinite(ours)
    B = markSkipped(B, i, 'no interior extremum in the step curve');
    return;
end
B(i).ours    = ours;
B(i).pct     = 100*(ours/book - 1);
B(i).ok      = abs(ours/book - 1) <= B(i).tol;
B(i).skipped = false;
B(i).why     = '';
% The optimal evasive maneuver is a STEP (rolled 180 deg), so it normalizes
% by T^2 like any other step -- hence 'step' here and not a fourth shape.
B(i).miss_book = missFt('step', book, T, level);
B(i).miss_ours = missFt('step', ours, T, level);
end


function B = markSkipped(B, i, why)
B(i).ours = NaN;  B(i).pct = NaN;  B(i).ok = false;
B(i).skipped = true;  B(i).why = why;
B(i).miss_book = NaN;  B(i).miss_ours = NaN;
end


function t = bandTol(band)
switch lower(band)
case 'graph', t = 0.15;   % author read it off a figure: 1-2 significant figures
case 'text',  t = 0.01;   % printed to 3 figures in the running text
otherwise
    error('mgs:zarchan:adjointBinomial5:band', ...
        ['unknown tolerance band "%s".  Bands encode the PROVENANCE of the ' ...
         'book value; inventing one silently sets an arbitrary pass criterion.'], band);
end
end


% =============================================================================
function s = render(R, opts)
W = 96;
L = {};
L{end+1} = 'Zarchan Listing 6.1 -- ADJOINT OF THE FIFTH-ORDER BINOMIAL GUIDANCE SYSTEM';
L{end+1} = '*** UNITS ARE FEET, ft/s, g = 32.2 ft/s^2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf('tau = %g s   tF = %g s   h = %g s   Vc = %g ft/s   N'' = %s', ...
    opts.tau, opts.tf, opts.h, opts.vc, mat2str(R.xnp));
L{end+1} = 'integrator: Zarchan''s two-pass predictor/corrector, transcribed';
L{end+1} = repmat('-', 1, W);

if isfield(R.selfcheck, 'ok')
    C = R.selfcheck;
    L{end+1} = 'STRUCTURAL SELF-CHECK  (wiring, not the book -- a failure here voids everything)';
    L{end+1} = sprintf('  open loop N''=0 vs analytic tau^2/2, tau^3/6, tau^4/24 : rel %9.2e  tol %.0e  %s', ...
        C.openloop_rel, C.openloop_tol, tf2s(C.openloop_ok));
    L{end+1} = sprintf('  Vc cancels out of the loop (x7)                        : rel %9.2e  tol %.0e  %s', ...
        C.vc_rel, C.vc_tol, tf2s(C.vc_ok));
    L{end+1} = sprintf('  T^2/T^3/T^4 normalization holds under tau (x2)         : rel %9.2e  tol %.0e  %s', ...
        C.tau_rel, C.tau_tol, tf2s(C.tau_ok));
else
    L{end+1} = 'STRUCTURAL SELF-CHECK  ***SKIPPED*** (selfcheck=false) -- nothing here is verified';
end
L{end+1} = repmat('-', 1, W);

L{end+1} = 'BOOK BENCHMARKS      tF = 2.5 s, T = 0.5 s => tF/T = 5   (Zarchan pp. 114-119)';
L{end+1} = '  feet columns use the T named in the level column; T = 0.5 s where none is named';
L{end+1} = sprintf('  %-22s %-11s %7s %8s %8s | %8s %8s | %-5s %s', ...
    'what', 'level', 'book', 'ours', 'diff', 'book ft', 'ours ft', 'band', 'verdict');
for i = 1:numel(R.bench)
    if i == 5
        L{end+1} = ['  ' repmat('.', 1, W-4)]; %#ok<AGROW>
        L{end+1} = '  secondary published values (same run, free cross-checks)'; %#ok<AGROW>
    end
    b = R.bench(i);
    if b.skipped
        L{end+1} = sprintf('  %-22s %-11s %7.4g %8s %8s | %8s %8s | %-5s SKIP (%s)', ...
            b.what, b.level, b.book, '--', '--', '--', '--', b.band, b.why); %#ok<AGROW>
    else
        L{end+1} = sprintf('  %-22s %-11s %7.4g %8.2f %+7.1f%% | %8.2f %8.2f | %-5s %s', ...
            b.what, b.level, b.book, b.ours, b.pct, b.miss_book, b.miss_ours, ...
            b.band, tf2s(b.ok)); %#ok<AGROW>
    end
end
L{end+1} = '  band: graph = author read it off a figure (15%); text = printed to 3 figures (1%)';
L{end+1} = repmat('-', 1, W);

L{end+1} = 'OPTIMAL EVASIVE MANEUVER (Figs. 6.9-6.11): target rolls 180 deg at each extremum';
for j = 1:numel(R.opt)
    O = R.opt(j);
    L{end+1} = sprintf('  N''=%-3g coeff = 2*sum|extrema| = %7.2f   exact TV = %7.2f', ...
        O.xnp, O.coeff, O.tv_exact); %#ok<AGROW>
    L{end+1} = sprintf('         extrema at tF/T = %-22s values = %s', ...
        mat2str(round(O.extrema_tf_over_T, 2)), mat2str(round(O.extrema_value, 2))); %#ok<AGROW>
end
L{end+1} = '  coeff reproduces the book''s arithmetic; TV is the exact bang-bang answer and';
L{end+1} = '  runs lower where the step curve has not decayed by tF/T = 10 (visible at N''=5).';
L{end+1} = repmat('-', 1, W);

S = R.shapes;
for k = 1:numel(S)
    L{end+1} = sprintf('DESIGN CURVE %d/%d -- %-5s    miss_ft = c * T^%d * %s', ...
        k, numel(S), S(k).name, S(k).power, S(k).level); %#ok<AGROW>
    hdr = sprintf('    %6s', 'tF/T');
    for j = 1:numel(R.xnp)
        hdr = [hdr sprintf('  %9s', sprintf('N''=%g', R.xnp(j)))]; %#ok<AGROW>
    end
    L{end+1} = hdr; %#ok<AGROW>
    for x = 1:floor(R.tf_over_T(end))
        row = sprintf('    %6.1f', x);
        for j = 1:numel(R.xnp)
            row = [row sprintf('  %9.2f', readAt(R, k, j, x))]; %#ok<AGROW>
        end
        L{end+1} = row; %#ok<AGROW>
    end
    L{end+1} = ''; %#ok<AGROW>
end

L{end+1} = 'READ THIS BEFORE QUOTING ANY NUMBER ABOVE';
L{end+1} = '  The only disturbances in Listing 6.1 are TARGET MANEUVERS (step/ramp/parabolic).';
L{end+1} = '  Our bullseye is stationary and our measured discriminant is release heading error';
L{end+1} = '  |eps0| -- an initial-condition channel that is NOT in this adjoint.  This file';
L{end+1} = '  validates the loop and prices lag against a maneuvering target.  It does not';
L{end+1} = '  predict a sortie miss and must not be reported as if it did.';
if opts.tau ~= 1
    L{end+1} = sprintf(['  NOTE tau = %g, not 1.  The curves are still normalized (divided by ' ...
        'T^n), so they should be IDENTICAL to a tau = 1 run -- that is what the tau ' ...
        'self-check asserts.'], opts.tau);
end
s = strjoin(L, newline);
end


% =============================================================================
function drawCurves(R, opts)
S = R.shapes;
for k = 1:numel(S)
    figure('Name', sprintf('Zarchan Fig. 6.%d -- %s', 5+k, S(k).name));
    plot(R.tf_over_T, R.curve(k).y', 'LineWidth', 1.2); grid on;
    xlabel('Normalized flight time  t_F/T');
    ylabel(sprintf('Miss / (T^%d * %s)   [ft]', S(k).power, S(k).level));
    title(sprintf('%s target maneuver -- fifth-order binomial (tau = %g s)', ...
        S(k).name, opts.tau));
    legend(arrayfun(@(v) sprintf('N'' = %g', v), R.xnp, 'UniformOutput', false), ...
        'Location', 'best');
end
end


function s = tf2s(b)
if b, s = 'PASS'; else, s = 'FAIL'; end
end
