function R = rusnakNumericalGains(opts)
%MGS.ZARCHAN.RUSNAKNUMERICALGAINS  Zarchan Listing 26.3 -- the Rusnak & Meir
%   NUMERICAL route to the optimal guidance gains for a single-lag flight
%   control system, checked against the closed form the same page prints.
%
%   R = mgs.zarchan.rusnakNumericalGains()
%   R = mgs.zarchan.rusnakNumericalGains('TAU', 2.0, 'assert', false)
%
%   Line-by-line port of Listing 26.3 (zarchan.txt:9612-9628), Fig. 26.10.
%   Three scalar ODEs and a division; that is the whole method.  It replaces
%   the 4x4 matrix Riccati integration of Listing 26.1 and, per the book,
%   "will run approximately 100 times faster" because it tolerates a step of
%   0.01 s where the Riccati route needs 0.0001 s (zarchan.txt:9609).
%
% ===========================================================================
%   UNITS.  SECONDS AND PER-SECOND.  NO LENGTHS APPEAR ANYWHERE.
% ===========================================================================
%   tgo, TAU, H [s]   N', C3, C4 [-]   C1 [1/s^2]   C2 [1/s]   L(tgo) = C1
%   x1 [s]   x2 [s^3]   x3 [s^2]   GAM [s^3]
%
%   GAM is the gamma of J = y^2(tF) + gamma*int nc^2 dt, so it carries s^3 and
%   SCALES WITH TAU^3.  Change TAU without changing GAM and the small-tgo end
%   of the curve moves; that is physics, not a bug.  The gains multiply y [ft],
%   ydot [ft/s], nT and nL [ft/s^2] at the CALL SITE -- this file computes
%   gains, never a trajectory, so g = 32.2 never appears.
%
%   WHAT IT CANNOT SAY HERE.  Every gain of nc = C1*y + C2*ydot + C3*nT +
%   C4*nL is scheduled on TIME TO GO, and our own tgo predictor is wrong by
%   40-50%.  Zarchan's Appendix A.1 puts the break-even at a tgo scale error
%   of 0.9 -- optimal guidance loses to ordinary PN well before our error.  So
%   this is not a law to fly.  It is (a) an independent check on our RK2
%   integrator against a published closed form and (b) the derivation of why
%   fixed-N PN at N' = 3 is exactly the far-field limit of the optimal law.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_26_03.m reports NO unresolved variable-to-
%   variable comparisons and that is correct -- every comparison in 26.3 is
%   against a literal.  Four forced repairs, none a judgement call:
%     1  "while ~(T .= (10.-.0001))" -> "while ~(T >= (TF - 1e-4))".  "." is
%        the OCR of ">" throughout this book; T only increases, so "<=" never
%        terminates and "==" never fires.  TF is parameterised (the listing
%        hard-codes 10); the guard is 1e-4 as printed, not Ch.4's 1e-5.
%     2  "while STEP,=1" -> "while STEP <= 1"; the body sets STEP=2 to leave.
%     3  "if S.=.09999" -> "if S >= samp - 0.5*H".  S resets on each record,
%        so "<=" would record forever.  The literal .09999 is 0.1 - 1e-5, i.e.
%        "samp = 0.1 s with a float guard"; keeping it would silently change
%        meaning at any other H.
%     4  "DERIVING NEW GUIDANCE LAW FOR CUBIC FLIGHT CONTROL SYSTEM" trailing
%        the transcription is the next SECTION HEADING (zarchan.txt:9629)
%        leaking in.  Dropped, not "fixed" into code.
%   Quirks: "save datfil.txt output /ascii" is a typo for -ascii (nothing is
%   written to disk here, so it is recorded and dropped); the "FLAG=0" after
%   the inner loop is dead and is kept as a comment, not code; and --
%     * D = X2 + GAM IS FORMED INSIDE THE DERIVATIVE BLOCK, so the D used at
%       sampling holds the PREDICTOR-stage x2 while PZ = X1/D uses the
%       CORRECTED x1.  KEPT.  D never feeds the dynamics so it cannot
%       accumulate; it is a per-sample offset of about H^2*x1*x1dot/(x2+GAM)
%       -- +0.10% at tgo = 1 s, +1.4% at 0.2 s, 4e-6 at 10 s.  R.d and R.x2
%       expose both stages so it is measurable rather than assumed, and
%       selfcheck 4 removes it to show what is left.
%
% ---------------------------------------------------------------------------
%   THE TWO REGULARIZATIONS ARE DIFFERENT NUMBERS.  This is the one thing in
%   this listing that will fool you.  The numerical branch divides by X2 + GAM
%   with GAM = 1e-5 [s^3].  The printed formula divides by BOT + 1e-4 where
%   BOT = 6*X2/TAU^3, i.e. by X2 + TAU^3*1e-4/6 = X2 + 1.667e-5 at TAU = 1.
%   Ch.30 confirms the second one IS a gamma: "a division by zero was avoided
%   by adding the term g to the denominator... g was set to 0.0001"
%   (zarchan.txt:11468, :11469).  So the two curves are the SAME function at
%   gammas differing by 5/3, and they part company wherever gamma rivals the
%   integral: XNP/XNPTH = (X2 + TAU^3*1e-4/6)/(X2 + GAM), which is 1.64 at
%   tgo = 0.1 s, 1.06 at 0.3 s, 1.0002 at 1 s.  Scoring "excellent agreement"
%   (:9610) over the whole sweep would fail for a reason that has nothing to
%   do with the port, so the two agreement rows start at 'agree_from'
%   (default 1 s, 90% of the sweep) and the divergence below it is an INFO row
%   carrying its own predicted value.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS -- every book value grepped out of zarchan.txt; the line number
%   is in each row's .source.  Nothing was read off a figure.
%     1  N' -> 3 as tgo/T -> infinity             value  (:2526, :8938)
%     2  numerical N' vs printed formula, tgo>=1  bound  (:9610, Fig 26.11)
%     3  numerical C4 vs printed formula, tgo>=1  bound  (:9610, Fig 26.12)
%     4  N' grows toward intercept, ->3 far out   shape  (:2526, :8938)
%     5  raw denominator -> 0 as tgo -> 0         shape  (:9356)
%   Rows 2 and 3 carry a 0.5% band.  Predicted residual is ~0.10% at tgo = 1 s
%   (0.078% predictor-stage D + 0.022% gamma mismatch + 0.013% RK2), falling
%   below 0.01% past tgo = 3 s.  If a run lands near 0.5% the mechanism above
%   says it should not have -- do NOT widen the band.
%   NOT SCORED (INFO): the 100x step ratio (arithmetic on two printed step
%   sizes, true by construction, measures no runtime); the peak of the N'
%   curve ("axis([0 10 .1 40])" is a plot range, not a measurement); and the
%   small-tgo divergence, for which the book publishes no number.
%
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED. ***  This file has
%   never been executed -- MATLAB is the caller's, not this author's:
%      tgo     XNP      XNPTH        C4        C4TH
%      0.1    4.64      2.822    -2.244     -1.365
%      0.3   32.74     30.64    -14.85     -13.90
%      1.0   12.31     12.294    -4.527     -4.523
%      5.0    4.5327    4.5327   -0.7265    -0.7264
%     10.0    3.6911    3.6911   -0.33220   -0.33220
%   100 samples (tgo = 0.1 .. 10.0 by 0.1), 1000 RK2 steps, 2000 RHS evals.
%   Row 1 should read 3.000003; rows 2 and 3 ~0.10% of a 0.5% band.
%
%   OUTPUTS.  R.t [s]; R.x1 R.x2 R.x3 the states; R.d the listing's D
%   (predictor-stage x2 + GAM, where R.x2 is corrected); R.pz = L(tgo);
%   R.xnp / R.xnpth numerical and formula N'; R.c1..R.c4 and R.c1th..R.c4th;
%   R.bench R.selfcheck R.pass R.verdict R.p R.summary R.units.  'assert'
%   false WARNS and still returns R complete -- benchSuite runs every port
%   that way so one red row cannot hide the board.
%
%   See also MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 26.3 line 1, verbatim.  TF is the listing's hard-coded 10.
    opts.TAU   (1,1) double {mustBeFinite, mustBePositive} = 1.0     % s
    opts.GAM   (1,1) double {mustBeFinite, mustBeNonnegative} = 1e-5 % s^3
    opts.TF    (1,1) double {mustBeFinite, mustBePositive} = 10.0    % s
    opts.H     (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s
    opts.samp  (1,1) double {mustBeFinite, mustBePositive} = 0.1     % s
    % The regularizer the PRINTED FORMULA carries: "+.0001" on its denominator.
    % Separate from GAM on purpose -- they are different numbers and the whole
    % small-tgo story above is that difference.  0 is allowed so the raw
    % formula can be inspected; it is singular as tgo -> 0.
    opts.GAMTH (1,1) double {mustBeFinite, mustBeNonnegative} = 1e-4
    % Where the two agreement rows start.  1 s is not a fudge: below it the
    % two gammas dominate and the comparison is between different problems.
    opts.agree_from (1,1) double {mustBeFinite, mustBePositive} = 1.0
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so an `if nargin == 0` here would be a mode that never fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

TAU = opts.TAU;  GAM = opts.GAM;  TF = opts.TF;  H = opts.H;
samp = opts.samp;  GAMTH = opts.GAMTH;

if H >= TAU
    error('mgs:zarchan:rusnakNumericalGains:stiff', ...
        ['H = %g s is not smaller than the lag time constant TAU = %g s.  ' ...
         'RK2 does not blow up here, it returns a smooth and entirely wrong ' ...
         'curve, so this is refused rather than warned about.'], H, TAU);
end
if samp < H
    error('mgs:zarchan:rusnakNumericalGains:samp', ...
        ['samp = %g s is finer than the integration step H = %g s, so the ' ...
         'returned curve would be sampled at the step rate while claiming ' ...
         'the finer interval in its own time vector.'], samp, H);
end
if TF <= 2*H
    error('mgs:zarchan:rusnakNumericalGains:short', ...
        'TF = %g s is not long enough to take two steps at H = %g s.', TF, H);
end

% =========================================================================
%   LISTING 26.3.  Transcribed from zarchan.txt:9612-9628.
% =========================================================================
[t, x1, x2pred, x2, x3, nEval] = sweepRun(TAU, TF, H, samp);
if numel(t) < 2
    % Refused, not warned about.  A one-sample curve makes diff() empty and
    % several checks below would then append an empty verdict -- a "null
    % assignment" error thrown far from the samp value that caused it.
    error('mgs:zarchan:rusnakNumericalGains:empty', ...
        ['the sweep recorded %d sample(s) at TF = %g s, samp = %g s.  At ' ...
         'least two are needed for any curve comparison to mean anything.'], ...
        numel(t), TF, samp);
end

% ---- the listing's sampling block, verbatim ------------------------------
% D holds the PREDICTOR-stage x2 (see the header); PZ divides the CORRECTED
% x1 by it.  That asymmetry is the listing's, and it is kept.
d   = x2pred + GAM;
pz  = x1 ./ d;                       % L(tgo), zarchan.txt:9592
xnp = pz .* t.^2;                    % N'(tgo) = L(tgo)*tgo^2,  :9600
c1  = pz;
c2  = pz .* t;
c3  = 0.5 * pz .* t.^2;
c4  = -x3 .* pz;

% ---- the closed forms the same listing prints ----------------------------
xs    = t / TAU;
xnpth = nprimeFormula(xs, GAMTH);                       % :9341, :2525
c4th  = -xnpth .* (exp(-xs) + xs - 1) ./ (xs.^2);       % :9352-9354
% Listing 26.1's first three formula gains (zarchan.txt:9347-9351), carried so
% the numerical C1..C3 have a published counterpart too.  They share xnpth, so
% they are NOT independent of row 2 -- reported, not separately scored.
c1th = xnpth ./ t.^2;
c2th = xnpth ./ t;
c3th = 0.5 * xnpth;

R = struct();
R.t = t;  R.x1 = x1;  R.x2 = x2;  R.x3 = x3;  R.d = d;
R.pz = pz;  R.xnp = xnp;  R.xnpth = xnpth;
R.c1 = c1;  R.c2 = c2;  R.c3 = c3;  R.c4 = c4;
R.c1th = c1th;  R.c2th = c2th;  R.c3th = c3th;  R.c4th = c4th;
R.units = ['seconds; N''/C3/C4 dimensionless, C1 [1/s^2], C2 [1/s], ' ...
           'x1 [s], x2 [s^3], x3 [s^2], GAM [s^3]; no lengths appear'];
R.p = struct('TAU',TAU, 'GAM',GAM, 'GAMTH',GAMTH, 'TF',TF, 'H',H, ...
             'samp',samp, 'agree_from',opts.agree_from, 'n_samples',numel(t), ...
             'n_rhs',nEval, 'units',R.units);

R.selfcheck = structuralChecks(t, x1, x2, x3, xnp, xnpth, c1, c2, c3, c4, ...
                               TAU, GAMTH);

if opts.benchmark
    R.bench = benchmarks(R, TAU, GAM, GAMTH, H, opts.agree_from);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine" -- that is the shape of green light this repo keeps paying for.
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R); end

msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = 'not one book value could be evaluated.';
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port ' ...
        'disagrees with ITSELF, which is a wiring fault, not a tolerance ' ...
        'question.  First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
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
        error('mgs:zarchan:rusnakNumericalGains:benchmark', '%s\n\n%s', ...
            msg, R.summary);
    else
        % WARN AND RETURN.  R comes back complete so the caller can see WHICH
        % side of a comparison moved.  A port that errors here hands back
        % nothing and cannot be diagnosed.
        warning('mgs:zarchan:rusnakNumericalGains:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function [t, X1a, X2Pa, X2a, X3a, nEval] = sweepRun(TAU, TF, H, samp)
%SWEEPRUN  Listing 26.3's integration loop, zarchan.txt:9612-9628.
%
%   Every constant arrives as an ARGUMENT.  MATLAB local functions cannot see
%   the main function's workspace; a previous wave in this package read a
%   constant from the caller and threw on the first line of the first run.
%
%   Returns the predictor-stage x2 (X2Pa) ALONGSIDE the corrected one (X2a).
%   Both come free from one sweep because D never feeds the dynamics -- it is
%   read only at sampling.  That is what lets selfcheck 4 remove the listing's
%   D asymmetry without a second integration.

S = 0.0;  T = 0.0;
X1 = 0.0;  X2 = 0.0;  X3 = 0.0;
% Derivatives pre-initialised so the FLAG==1 branch cannot read a leftover.
X1D = 0.0;  X2D = 0.0;  X3D = 0.0;  X2PRED = 0.0;

nmax = ceil(TF/min(samp, H)) + 8;
t = zeros(nmax,1);  X1a = t;  X2a = t;  X3a = t;  X2Pa = t;
n = 0;  nEval = 0;

while ~(T >= (TF - 1e-4))            % listing: while ~(T .= (10.-.0001))
    S = S + H;
    X1OLD = X1;  X2OLD = X2;  X3OLD = X3;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  Pass 1 evaluates at the current
    % state; pass 2 takes a full Euler step, advances T by H and re-evaluates
    % there.  T advances INSIDE this loop, exactly once, which is what makes
    % the second evaluation happen at t+H.  Move that line out and the scheme
    % degrades to first order while still running perfectly smoothly.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            X1 = X1 + H*X1D;  X2 = X2 + H*X2D;  X3 = X3 + H*X3D;
            T  = T + H;
        end
        X1D    = (T - X1)/TAU;   % single lag driven by the ramp tgo
        X2D    = X1^2;           % the integral in the denominator of L(tgo)
        X2PRED = X2;             % the listing forms D = X2 + GAM RIGHT HERE
        X3D    = -X3/TAU + T;    % same lag, scaled: x3 == TAU*x1 exactly
        FLAG   = 1;
        nEval  = nEval + 1;
    end
    % The listing writes "FLAG=0" here too.  Dropped: the outer loop
    % reinitialises FLAG on its next pass, so the assignment cannot be read.

    % Heun corrector, in the listing's own algebra.
    X1 = 0.5*(X1OLD + X1 + H*X1D);
    X2 = 0.5*(X2OLD + X2 + H*X2D);
    X3 = 0.5*(X3OLD + X3 + H*X3D);

    if S >= samp - 0.5*H         % listing: if S .= .09999  (= 0.1 - 1e-5)
        S = 0.0;
        n = n + 1;
        t(n) = T;  X1a(n) = X1;  X2a(n) = X2;  X3a(n) = X3;  X2Pa(n) = X2PRED;
    end
end

t = t(1:n);  X1a = X1a(1:n);  X2a = X2a(1:n);  X3a = X3a(1:n);  X2Pa = X2Pa(1:n);
end


% =============================================================================
function y = nprimeFormula(x, gamth)
%NPRIMEFORMULA  zarchan.txt:9341 (and :2525, :8933), with the listing's own
%   "+.0001" regularizer carried as an argument:
%
%     N'(x) = 6x^2(e^-x - 1 + x) / (2x^3 + 3 + 6x - 6x^2 - 12x e^-x - 3 e^-2x
%                                   + gamth),      x = tgo/T
%
%   The denominator cancels catastrophically for small x -- it is O(0.3 x^5)
%   formed as a difference of terms near 3.  Unlike Ch.4's bracket this needs
%   NO series expansion, because gamth (1e-4) swamps the true value everywhere
%   the cancellation bites (below x ~ 0.35), which is exactly why Zarchan put
%   it there.  Pass gamth = 0 and that protection is gone as well.
y = 6*x.^2.*(exp(-x) - 1 + x) ./ ...
    (2*x.^3 + 3 + 6*x - 6*x.^2 - 12*x.*exp(-x) - 3*exp(-2*x) + gamth);
end


% =============================================================================
function K = structuralChecks(t, x1, x2, x3, xnp, xnpth, c1, c2, c3, c4, ...
                              TAU, GAMTH)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.
K = struct();
names = {};  oks = [];  detail = {};

% 1  x3 == TAU*x1 is an EXACT identity of the transcribed ODEs: substituting
%    u = TAU*x1 into x1' = (t-x1)/TAU gives u' = -u/TAU + t, which is X3D
%    term for term.  This is the only cheap thing that catches a wiring fault
%    in X3D, and C4 is built entirely out of x3.
scale = max(abs(TAU*x1));
r1 = max(abs(x3 - TAU*x1)) / max(scale, realmin);
names{end+1} = 'x3 == TAU*x1 (exact identity of the ODEs)';
oks(end+1)   = r1 <= 1e-9;
detail{end+1} = sprintf('max |x3 - TAU*x1| / peak = %.3e', r1);

% 2  x2 = int x1^2 cannot decrease.  A sign error on X2D shows up here and
%    nowhere else cheap.
mn = min(diff(x2));
names{end+1} = 'x2 non-decreasing (x2dot = x1^2)';
oks(end+1)   = mn >= -1e-15;
detail{end+1} = sprintf('min consecutive change %+.3e s^3', mn);

% 3  The gain definitions of zarchan.txt:9347-9351 and :9592: C1 = N'/tgo^2,
%    C2 = N'/tgo, C3 = 0.5N'.  Tautological given the listing's own lines, so
%    it proves nothing about the book -- it catches a typo in THIS file.
r3 = max([ max(abs(c1 - xnp./t.^2) ./ max(abs(c1), realmin)), ...
           max(abs(c2 - xnp./t)    ./ max(abs(c2), realmin)), ...
           max(abs(c3 - 0.5*xnp)   ./ max(abs(c3), realmin)) ]);
names{end+1} = 'C1 = N''/tgo^2, C2 = N''/tgo, C3 = 0.5N''';
oks(end+1)   = r3 <= 1e-12;
detail{end+1} = sprintf('worst relative deviation %.3e', r3);

% 4  ⭐ THE SHARP ONE.  Remove the TWO known differences between the numerical
%    branch and the printed formula -- the gamma mismatch (use the formula's
%    own TAU^3*GAMTH/6) and the predictor-stage D (use the corrected x2) --
%    and the two must collapse onto each other at EVERY sample, small tgo
%    included.  What is left is pure RK2 truncation: predicted worst ~0.2%
%    near tgo = 0.25 s, where trapezoid error on int x1^2 is still comparable
%    to the denominator, falling to ~1e-5 by tgo = 3 s.  Band 1%.
%    If this row goes red, x1 or x2 is wired wrong; if it is green and rows
%    2/3 are red, the difference really is the two regularizations.
xnp_star = x1 .* t.^2 ./ (x2 + TAU^3*GAMTH/6);
r4 = max(abs(xnp_star ./ xnpth - 1));
names{end+1} = 'matched gamma + corrected x2 collapses onto the formula';
oks(end+1)   = r4 <= 0.01;
detail{end+1} = sprintf('max |N''_matched/N''_formula - 1| = %.4f%%', 100*r4);

% 5  Signs.  N' is a positive navigation ratio; C4 multiplies nL in a
%    subtraction (nc = ... - nL*T^2(e^-x + x - 1)*N'/tgo^2), so it is negative
%    everywhere -- which is why the book plots "-C4".
names{end+1} = 'N'' > 0 and C4 < 0 throughout';
oks(end+1)   = all(xnp > 0) && all(c4 < 0);
detail{end+1} = sprintf('min N'' = %.4f, max C4 = %+.4f', min(xnp), max(c4));

K.names = names;
K.oks   = logical(oks);
K.detail = detail;
K.all_ok = all(K.oks);
bad = find(~K.oks, 1);
if isempty(bad), K.first_bad = ''; else, K.first_bad = names{bad}; end
end


% =============================================================================
function B = benchmarks(R, TAU, GAM, GAMTH, H, agree_from)
%BENCHMARKS  Computed vs published.  EVERY book value here was grepped out of
%   zarchan.txt and the line number is in the .source field.  Nothing was read
%   off a figure -- the extraction turned every figure into an image link.
B = emptyBench();
t = R.t;  win = t >= agree_from;
tolCurve = 0.005;         % 0.5%; predicted residual ~0.10% -- see the header

% ---- 1.  the far-field limit ---------------------------------------------
% "at the beginning of the flight (long time to go) the effective navigation
%  ratio is approximately constant and is approaching 3" -- zarchan.txt:2526.
% Evaluated at x = 1e6 through the SAME formula the listing prints; gamth is
% 1e-4 against a denominator of 2e18 there, so the regularizer is irrelevant.
lim = nprimeFormula(1e6, GAMTH);
B = addVal(B, 'N'' as tgo/T -> infinity', 3.0, lim, 'text', 0.01, ...
    'p.179 text (zarchan.txt:2526), restated at :8938', '-', ...
    ['the ONE row that survives any TAU, GAM or TF.  It is also why fixed-N ' ...
     'PN at N'' = 3 is the far-field limit of the optimal law']);

% ---- 2.  numerical N' vs the printed formula -----------------------------
if any(win)
    r = max(abs(R.xnp(win)./R.xnpth(win) - 1));
    B = addBound(B, sprintf('numerical N'' vs formula, tgo >= %g s', agree_from), ...
        sprintf('<= %.3f%%', 100*tolCurve), sprintf('%.4f%%', 100*r), ...
        r <= tolCurve, ...
        'p.669 text (zarchan.txt:9610) + Fig. 26.11 caption (:9613)', '-', ...
        ['"excellent agreement for the effective navigation ratio".  ' ...
         'Predicted ~0.10% at tgo = 1 s: 0.078% predictor-stage D + 0.022% ' ...
         'gamma mismatch + 0.013% RK2, all falling with tgo']);
else
    B = addSkip(B, 'numerical N'' vs formula', '"excellent agreement"', ...
        sprintf('agree_from = %g s is past the end of the sweep (TF = %g s)', ...
                agree_from, R.p.TF), 'zarchan.txt:9610', '-');
end

% ---- 3.  numerical C4 vs the printed formula -----------------------------
if any(win)
    r = max(abs(R.c4(win)./R.c4th(win) - 1));
    B = addBound(B, sprintf('numerical C4 vs formula, tgo >= %g s', agree_from), ...
        sprintf('<= %.3f%%', 100*tolCurve), sprintf('%.4f%%', 100*r), ...
        r <= tolCurve, ...
        'p.670 text (zarchan.txt:9610) + Fig. 26.12 caption (:9623)', '-', ...
        ['"accurate fourth control gain".  C4/C4TH = (N''/N''_th)*(x1/x1_exact) ' ...
         'analytically, so this row adds x1''s own truncation on top of row 2 ' ...
         'and independently checks the sign and the x^2 divisor of C4TH']);
else
    B = addSkip(B, 'numerical C4 vs formula', '"accurate fourth control gain"', ...
        'agree_from is past the end of the sweep', 'zarchan.txt:9610', '-');
end

% ---- 4.  the shape claim -------------------------------------------------
% ":2526 -- approximately constant and approaching 3 [far out].  As we get
%  closer to intercept... grows considerably."  Scored as TWO orderings, which
%  needs no threshold pulled out of the air: N' falls monotonically with tgo,
%  and |N' - 3| falls monotonically with tgo as well.
if sum(win) >= 3
    v = R.xnp(win);
    grows   = all(diff(v) < 0);              % rises as tgo falls
    settles = all(diff(abs(v - 3)) < 0);     % and settles toward 3
    B = addShape(B, 'N'' grows toward intercept and approaches 3 far out', ...
        'N'' and |N'' - 3| both strictly decreasing in tgo', ...
        sprintf('N'' = %.4f at tgo = %g s down to %.4f at %g s; |N''-3| %.4f -> %.4f', ...
                v(1), t(find(win,1)), v(end), t(end), abs(v(1)-3), abs(v(end)-3)), ...
        grows && settles, 'p.179 text (zarchan.txt:2526), also :8938', ...
        ['two orderings rather than a threshold.  A sign error in the ' ...
         'denominator leaves N'' plausible in magnitude but breaks monotonicity']);
else
    B = addSkip(B, 'N'' grows toward intercept and approaches 3 far out', ...
        'monotone in tgo', ...
        sprintf(['only %d sample(s) at or past tgo = %g s; "monotone" would ' ...
                 'be a statement about the sampling'], sum(win), agree_from), ...
        'zarchan.txt:2526', '-');
end

% ---- 5.  why the regularizer is there at all -----------------------------
% ":9356 -- the denominator in the closed-form solution for the effective
%  navigation ratio goes to zero as time to go approaches zero.  To avoid that
%  singularity, 0.0001 was added to the denominator."
xs  = t/TAU;
raw = 2*xs.^3 + 3 + 6*xs - 6*xs.^2 - 12*xs.*exp(-xs) - 3*exp(-2*xs);
% MONOTONICITY IS ONLY MEANINGFUL WHERE THE VALUE IS ABOVE THE CANCELLATION
% FLOOR.  raw is O(0.3*x^5) formed as a difference of terms near 3, so its
% float noise is ~7e-16 absolute; below ~1e-10 the samples are noise and would
% fail an ordering test for a perfectly correct run at a fine samp.  At the
% listing's own samp = 0.1 s every sample clears it (raw(0.1) = 2.8e-6).
good = raw > 1e-10;
if sum(good) >= 3
    B = addShape(B, 'raw formula denominator -> 0 as tgo -> 0', ...
        'positive, strictly increasing in tgo, first sample < 1e-5 of last', ...
        sprintf('%.4e at tgo = %g s vs %.4e at %g s (ratio %.2e)', ...
                raw(1), t(1), raw(end), t(end), raw(1)/max(raw(end), realmin)), ...
        all(diff(raw(good)) > 0) && raw(1) < 1e-5*raw(end), ...
        'p.654 text (zarchan.txt:9356)', ...
        ['the stated reason the "+.0001" exists.  It also happens to mask the ' ...
         'catastrophic cancellation in this expression, which is why no series ' ...
         'expansion is needed here (unlike Ch.4''s bracket)']);
else
    B = addSkip(B, 'raw formula denominator -> 0 as tgo -> 0', ...
        'positive and strictly increasing', ...
        sprintf(['only %d of %d samples clear the 1e-10 cancellation floor, ' ...
                 'so an ordering test would be measuring float noise'], ...
                sum(good), numel(raw)), 'zarchan.txt:9356', '-');
end

% ---- INFO rows.  Reported, never scored. ---------------------------------
lo = ~win;
if any(lo)
    [wr, iw] = max(R.xnp(lo)./R.xnpth(lo));
    tl = t(lo);
    pred = (R.x2(lo) + TAU^3*GAMTH/6) ./ (R.x2(lo) + GAM);
    B = addInfo(B, 'small-tgo divergence between the two branches', '--', ...
        sprintf('worst ratio %.4f at tgo = %g s (gamma alone predicts %.4f)', ...
                wr, tl(iw), pred(iw)), ...
        'mechanism: zarchan.txt:11468-11469 (Ch.30 names the "+.0001" a gamma)', ...
        ['NOT SCORED: the book publishes no number for it.  The numerical ' ...
         'branch uses GAM = ' sprintf('%g', GAM) ' s^3 and the formula uses ' ...
         'TAU^3*' sprintf('%g', GAMTH) '/6 = ' sprintf('%g', TAU^3*GAMTH/6) ...
         ' s^3.  Same function, two gammas -- so this is a real difference ' ...
         'between two published expressions, not a port defect']);
end

B = addInfo(B, 'step-size ratio vs Listing 26.1 (Riccati route)', ...
    '"approximately 100 times faster"', ...
    sprintf('%g / %g = %.0f integration steps', 1e-4, H, 1e-4/H), ...
    'p.669 text (zarchan.txt:9609); 26.1''s 0.0001 s at :9356', ...
    ['NOT SCORED: arithmetic on two printed step sizes, true by construction, ' ...
     'and it measures no runtime.  The real ratio is LARGER -- 26.1 does 4x4 ' ...
     'matrix products per step where this does three scalar lines']);

[pk, ip] = max(R.xnp);
B = addInfo(B, 'peak of the N'' curve on the sampled grid', '--', ...
    sprintf('%.3f at tgo = %g s', pk, t(ip)), ...
    'Figs. 26.11/26.12 -- shape only', ...
    ['NOT SCORED: the book publishes no peak.  "axis([0 10 .1 40])" in the ' ...
     'listing is a plot range the author chose, not a measurement -- though ' ...
     'the peak landing under 40 is consistent with it']);

% ---- the cross-check the brief asked for, and cannot have yet ------------
B = addSkip(B, 'cross-check vs the Listing 26.1 Riccati port', ...
    'the two routes agree (Fig. 26.6, Fig. 26.11)', ...
    ['mgs.zarchan.optimalGainsSingleLag (Listing 26.1) is NOT PORTED as of ' ...
     'this writing -- `which` finds nothing and no file in +mgs/+zarchan ' ...
     'mentions Listing 26.1.  When it exists, the row is: run it at the same ' ...
     'TAU and TF, and require its numerical C1..C4 and N'' to match R.c1..R.c4 ' ...
     'and R.xnp on a common tgo grid.  Until then the only cross-check ' ...
     'available is against the CLOSED FORM both listings print, which is ' ...
     'exactly what rows 2 and 3 do -- a weaker claim, honestly labelled.'], ...
    'p.669 text (zarchan.txt:9610)', '-');
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want.  A struct array
%   whose fields quietly reordered would make [B.ok] mean something else and
%   would fail with "Subscripted assignment between dissimilar structures",
%   which reads like a typo rather than a schema break.  This is the schema
%   mgs.zarchan.benchSuite's adaptGeneric consumes, and it is STRICT.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.4g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.7g', ours);
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

function B = addBound(B, what, bookStr, oursStr, ok, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'bound';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'bound';  B(i).tol = NaN;
B(i).ok   = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = units;  B(i).detail = detail;
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
%ADDSKIP  A check that COULD NOT RUN.  Never a pass: a check that did not
%   happen is a different thing from one that succeeded.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'skip';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = '--';
B(i).pct  = NaN;   B(i).band = 'skip';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end


% =============================================================================
function s = render(R, ev)
L = {};  p = R.p;
L{end+1} = 'Zarchan Listing 26.3 -- RUSNAK & MEIR NUMERICAL GUIDANCE GAINS';
L{end+1} = '   (single-lag flight control system; Fig. 26.10)';
L{end+1} = ['*** UNITS: ' R.units ' ***'];
L{end+1} = sprintf(['   TAU = %g s   GAM = %g s^3   formula gamma = TAU^3*%g/6 ' ...
    '= %g s^3'], p.TAU, p.GAM, p.GAMTH, p.TAU^3*p.GAMTH/6);
L{end+1} = sprintf('   TF = %g s   H = %g s   samp = %g s   %d samples, %d RHS evals', ...
    p.TF, p.H, p.samp, p.n_samples, p.n_rhs);
L{end+1} = '';
L{end+1} = sprintf('%8s %10s %10s %11s %11s %11s', ...
    'tgo [s]', 'N'' num', 'N'' form', 'C4 num', 'C4 form', 'C1 num');
% Row 1 (the smallest tgo, where the two branches disagree most) plus one row
% per second.  Starting the stride AT the stride, not at 1, is what puts whole
% seconds in the table instead of 1.1, 2.1, ...
stride = max(1, round(1.0/p.samp));
rows = unique([1, stride:stride:numel(R.t), numel(R.t)]);
for r = rows
    L{end+1} = sprintf('%8.2f %10.4f %10.4f %11.5f %11.5f %11.4f', ...
        R.t(r), R.xnp(r), R.xnpth(r), R.c4(r), R.c4th(r), R.c1(r)); %#ok<AGROW>
end
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-52s %s', vstr(K.oks(i)), K.names{i}, ...
        K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false';
    L{end+1} = '            meaning NOT CHECKED, which is not "fine".';
else
    L{end+1} = 'BENCHMARKS vs zarchan.txt  (line numbers are in R.bench.source)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        if b.skipped && strcmpi(b.kind, 'info'), mark = 'info';
        elseif b.skipped,                        mark = 'SKIP';
        else,                                    mark = vstr(b.ok);
        end
        L{end+1} = sprintf('   %-4s %-46s book %-34s ours %s', mark, ...
            b.what, b.book_str, b.ours_str); %#ok<AGROW>
    end
    L{end+1} = sprintf('   %d of %d rows scored.', sum(ev), numel(R.bench));
end
s = strjoin(L, newline);
end

function v = vstr(ok)
if ok, v = 'ok'; else, v = 'FAIL'; end
end


% =============================================================================
function drawCurves(R)
%DRAWCURVES  The listing's two figures: semilogy of N' and of -C4, numerical
%   against formula, on the listing's own axis limits.
figure;
semilogy(R.t, R.xnp, R.t, R.xnpth); grid on
xlabel('Time to go (s)'); ylabel('NP'); axis([0 R.p.TF 0.1 40])
legend('numerical (Listing 26.3)', 'formula', 'Location', 'best')
title('Fig. 26.11 -- effective navigation ratio')

figure;
semilogy(R.t, -R.c4, R.t, -R.c4th); grid on
xlabel('Time to go (s)'); ylabel('-C4'); axis([0 R.p.TF 0.1 20])
legend('numerical (Listing 26.3)', 'formula', 'Location', 'best')
title('Fig. 26.12 -- fourth control gain')
end
