function R = differenceEquation(opts)
%MGS.ZARCHAN.DIFFERENCEEQUATION  Zarchan Listing 1.2 -- difference-equation
%   simulation of a first-order fading-memory filter, checked against the
%   closed form the book derives on the facing page.
%
%   R = mgs.zarchan.differenceEquation()
%   R = mgs.zarchan.differenceEquation('G', 0.2)            other filter gain
%   R = mgs.zarchan.differenceEquation('assert', false)     warn, still return R
%
%   Line-by-line port of Listing 1.2 (zarchan.txt:317-318) -- the smallest
%   listing in the book, two source lines, no numerical integration at all.
%   It solves  y(n+1) = y(n) + G*(x(n+1) - y(n))  (zarchan.txt:283) for a unit
%   step input and prints it beside  y(n) = 1 - (1-G)^n  (zarchan.txt:315).
%
%   UNITS: none.  y, x and G are dimensionless; the only dimensional quantity
%   is the sampling time TS = 0.1 s, with t = n*TS (zarchan.txt:281).  No g and
%   no ft appear -- the ft-vs-m hazard governing the rest of this package does
%   not arise here.
%
% ---------------------------------------------------------------------------
%   *** THIS CHAPTER PUBLISHES ALMOST NOTHING SCOREABLE.  SAYING SO IS THE
%   CORRECT OUTCOME, NOT A FAILURE. ***  The whole passage (:282-329) holds
%   exactly ONE independent number this port can be held to: the 63% at :324.
%   The rest is inputs we set ourselves (:317), a loop bound (:323), or an
%   identity the listing already asserts (:315).  Fig. 1.3 publishes no numbers
%   -- it became an image link at :321/:331 -- so nothing is read off it.  Seven
%   rows: five adjudicated, one INFO, one SKIP; two of the five can only pass or
%   skip and are locks on the default, which their .detail says outright.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_01_02.m reports 11 curly-quote repairs and NO
%   unresolved comparisons -- correct: the listing contains no comparison at
%   all, and no prose leaked into it.
%
%   In THIS passage the extraction DROPPED minus signs rather than mapping them
%   to the digit 2 (the third damage class): ":291 a = 1 G" is a = 1 - G, and
%   ":315 yn = 1 (1 G)n" is y(n) = 1 - (1-G)^n.  Both readings are forced --
%   a = 1 + G puts the pole outside the unit circle, and the printed factor
%   G/(1-a) at :294 would then be -1, giving y -> -1 against a unit step.  The
%   passage prints no signed decimals, so nothing here is at minus->2 risk.
%
%   Three deliberate departures from the literal transcription:
%     1  YTHEORY is X*(1-(1-G)^n), not the listing's 1-(1-G)^n.  The closed
%        form is derived for a UNIT step (":283 ... that is, x(n+1) = 1") and
%        the listing hard-codes X = 1, so they agree exactly at the default.
%        Off X = 1 the literal form would compare a unit-step theory against a
%        scaled simulation and redden the agreement row for a reason having
%        nothing to do with the recursion.  R.y_theory_unit keeps it visible.
%     2  The pre-loop "YTHEORY = 1.-(1.-G)^N" with N = 0 is dead code -- the
%        for loop overwrites N immediately.  It states that y(0) = 0, matching
%        Y = 0., and is recorded here rather than executed.
%     3  The arrays are preallocated (the listing grows them), and "save datfil
%        output -ascii" is dropped: nothing is written to disk.
%   The listing's "count" index is kept though it equals n, because the grid
%   starts at t = TS -- n = 0 is never recorded.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS -- book values grepped from the DECODED book, line number in
%   each row's .source.  Nothing read off a figure.
%     1 BOUND  max|y - X(1-(1-G)^n)| <= 1e-12.  ":324 the simulation results
%       agree with the closed-form solution" against the closed form at :315.
%       At G = 0.5, X = 1 every value is dyadic, so 0 is expected EXACTLY.
%     2 VALUE  20 iterations (":323 ... solves the difference equation 20
%       times").  Band 0.  Pass-or-skip: a lock, not a discriminating check.
%     3 VALUE  THE ONE REAL ROW.  ":324 the amount of time it takes the filter
%       output to reach 63% of its steady-state value is the filter time
%       constant."  tau = -TS/ln(1-G) = 0.14427 s here.  Scored as the fraction
%       of steady state reached at tau, from the decay ratio MEASURED off the
%       simulation -- so a broken recursion moves it (ratio 0.6 reads 52.1%).
%       Band 0.5 pp / 63 = 0.794%, the rounding of a two-figure percentage and
%       nothing else.  Expected +0.34%: 1 - 1/e = 63.212% against a printed 63.
%       NOT interpolated between samples -- tau falls between n = 1 (0.50) and
%       n = 2 (0.75), where interpolation reads 61.1%, a pure grid artefact.
%       The same 63% convention is used by the book at :7710.
%     4 SHAPE  ":324 the filter output eventually matches the filter input" --
%       monotone toward X, no overshoot.  Valid at any G in (0,1).
%     5 BOUND  |y_N - X|/|X| <= 1e-5, the convergence half of that sentence.
%       SKIPPED with the arithmetic when (1-G)^N > 1e-5; band never widened.
%     6 INFO   the data statement (:317) and the resulting tau.
%     7 SKIP   tau in SECONDS -- Ch.1 prints only the 63% definition, never a
%       number, and Fig. 1.3 carries no scale.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED. ***  This file has
%   never been executed; MATLAB is the caller's.  At the defaults:
%     y(n) = 1 - 2^-n, y(20) = 0.99999904632568;  max|y - YTHEORY| = 0 EXACTLY
%       (a small nonzero is already a rewrite, not creeping rounding)
%     tau = 0.1442695 s; fraction of steady state at tau = 63.2121% against the
%       book's 63, i.e. +0.337% against a 0.794% band
%     |y(20) - 1| = 9.5367e-07 against the 1e-5 bound
%     delete experiments: dropping the -y feedback gives y(20) = 10 (residual
%       9.0); shifting the theory exponent gives 0.5
%   A residual near a band edge is not a pass to accept.  Do not widen a band.
%
% ---------------------------------------------------------------------------
%   OUTPUTS  R.n R.t R.y R.y_theory R.resid (the listing's arrays);
%     R.y_theory_unit the literal 1-(1-G)^n; R.tau_s; R.ratio_hat (the decay
%     ratio measured off the simulation, should be 1-G); R.units R.p R.bench
%     R.selfcheck R.pass R.verdict R.summary.
%   'assert' chooses the REACTION to a failure and nothing else: true errors,
%   false warns and returns R complete.  It is never silent.
%
%   See also MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 1.2's data statement, verbatim (zarchan.txt:317).
    opts.G  (1,1) double {mustBeFinite} = 0.5
    opts.X  (1,1) double {mustBeFinite} = 1.0
    opts.TS (1,1) double {mustBeFinite, mustBePositive} = 0.1
    % "for N=1:20".
    opts.N  (1,1) double {mustBeFinite, mustBePositive, mustBeInteger} = 20
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so an "if nargin == 0" mode here would never fire.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

G = opts.G;  X = opts.X;  TS = opts.TS;  N = opts.N;

if ~(G > 0 && G < 1)
    % ":283 the filter gain is a designer chosen number between zero and
    % unity".  Outside it the closed form still holds algebraically, but tau is
    % undefined at G = 1 and the shape rows describe a different signal.
    % Refused, not warned about: it would return a smooth, plausible curve.
    error('mgs:zarchan:differenceEquation:gain', ...
        ['G = %g.  The book''s fading-memory gain lies strictly between 0 ' ...
         'and 1 (zarchan.txt:283); outside that range the 63%%-of-steady-' ...
         'state and monotonicity rows describe a different signal.'], G);
end
if N < 2
    % With one sample diff(y) is EMPTY, so the ordering test evaluates to []
    % rather than false and appending it throws "null assignment" far from the
    % N that caused it.  Refused rather than warned about.
    error('mgs:zarchan:differenceEquation:short', ...
        ['N = %d.  A single sample has no ordering and no measurable decay ' ...
         'ratio, so the shape and 63%%-of-steady-state rows would be empty ' ...
         'rather than false.'], N);
end
if X == 0
    error('mgs:zarchan:differenceEquation:input', ...
        ['X = 0.  The steady state is then 0 and "63%% of steady state" is ' ...
         'undefined, so every scored row would be vacuous.']);
end

% =========================================================================
%   LISTING 1.2.  Transcribed from zarchan.txt:317.
% =========================================================================
[t, y, y_theory] = runListing(G, X, TS, N);
y_theory_unit = 1 - (1-G).^(1:N)';       % the listing's literal form, X = 1
resid = y - y_theory;

% Decay ratio MEASURED off the simulation, not assumed.  X - y(n) = X(1-G)^n,
% so this is (1-G) for a correct recursion and something else for a broken one.
% Endpoints only; constancy is a separate structural check.
remN = (X - y(N))/X;
if remN > 0
    ratio_hat = remN^(1/N);
else
    % Underflow (large N) or a broken recursion that overshoots X.  NaN, not a
    % fabricated value: benchmark 3 skips itself on this and says why.
    ratio_hat = NaN;
end
tau_s = -TS/log(1-G);

R = struct();
R.n     = (1:N)';
R.t     = t;
R.y     = y;
R.y_theory      = y_theory;
R.y_theory_unit = y_theory_unit;
R.resid = resid;
R.tau_s = tau_s;
R.ratio_hat = ratio_hat;
R.units = 'dimensionless (y, x, G); time in seconds via t = n*TS; no g, no ft';
R.p = struct('G',G, 'X',X, 'TS',TS, 'N',N, 'tau_s',tau_s, 'units',R.units);

R.selfcheck = structuralChecks(t, y, y_theory, G, X, TS, N);

if opts.benchmark
    R.bench = benchmarks(y, resid, ratio_hat, tau_s, G, X, TS, N);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false(1,0);
    % NOT true.  "Not checked" must never render as "fine".
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R);            end

% ---- adjudication.  The verdict is FORMED either way; 'assert' picks only the
%      reaction, because benchSuite runs every port with assert off and a port
%      that refuses to return is a port nobody can diagnose.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated at G = %g, ' ...
        'X = %g, TS = %g s, N = %d.  An unvalidated port must not be ' ...
        'handed back silently.'], G, X, TS, N);
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
        error('mgs:zarchan:differenceEquation:benchmark', '%s\n\n%s', ...
            msg, R.summary);
    else
        warning('mgs:zarchan:differenceEquation:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function [t, y, ytheory] = runListing(G, X, TS, N)
%RUNLISTING  Listing 1.2, zarchan.txt:317.  The loop is kept as a loop: it IS
%   the listing, and 20 iterations is not a performance question.
%
%   Every constant arrives as an ARGUMENT -- MATLAB local functions cannot see
%   the main function's workspace, and an earlier wave in this package shipped
%   a local that read one from the caller and threw on its first line.
Y = 0.0;                                  % Y=0. in the data statement
count = 0;
t       = zeros(N,1);
y       = zeros(N,1);
ytheory = zeros(N,1);
for n = 1:N
    Y       = Y + G*(X - Y);              % the difference equation, :283
    T       = n*TS;                       % t = n*Ts, :281
    YTHEORY = X*(1 - (1-G)^n);            % :315, scaled by X -- see header
    count   = count + 1;
    t(count)       = T;
    y(count)       = Y;
    ytheory(count) = YTHEORY;
end
end


% =============================================================================
function K = structuralChecks(t, y, ytheory, G, X, TS, N)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never to be quoted
%   as evidence that this port reproduces Zarchan.  Two of the five are DELETE
%   EXPERIMENTS: they break the recursion on purpose and require the agreement
%   row's own threshold to notice -- a guard nobody has tried to break is not a
%   guard.  .names and .oks are what put these on the board; an invariant
%   stored under any other field name is an invariant nobody ran.
K = struct();
names = {};  oks = [];  detail = {};
tolExact = 1e-12*max(1, abs(X));          % the same threshold benchmark 1 uses

% 1  the time axis.  The listing sets T = n*TS; a cumulative "T = T + TS" would
%    drift by ~N*eps and look identical on a plot.
r1 = max(abs(t - (1:N)'*TS));
names{end+1} = 't(n) == n*TS exactly';
oks(end+1)   = r1 <= 8*eps*N*TS;
detail{end+1} = sprintf('max |t - n*TS| = %.3e s', r1);

% 2  the error decays by exactly (1-G) every step.  This is the recursion's
%    signature: an additive constant, a wrong gain, or a missing feedback term
%    all break it, and it needs no closed form to state.
e  = X - y;
r2 = max(abs(e(2:end) - (1-G)*e(1:end-1)))/abs(X);
names{end+1} = 'error ratio == (1-G) every step';
oks(end+1)   = r2 <= 1e-13;
detail{end+1} = sprintf('max |e(n+1)-(1-G)e(n)|/|X| = %.3e', r2);

% 3  the first sample.  Catches an off-by-one that starts the closed form at
%    n = 0 or the loop at n = 2 -- both of which still give a smooth curve.
r3 = abs(y(1) - G*X)/abs(X);
names{end+1} = 'first sample y(1) == G*X';
oks(end+1)   = r3 <= 8*eps;
detail{end+1} = sprintf('|y(1) - G*X|/|X| = %.3e', r3);

% 4  DELETE EXPERIMENT: drop the -y feedback (y = y + G*x), the single most
%    likely slip in a two-line listing.  The agreement row MUST go loud, so
%    this requires 1e6x its threshold, not merely "different".
yBad = zeros(N,1);  Yb = 0.0;
for n = 1:N
    Yb = Yb + G*X;
    yBad(n) = Yb;
end
r4 = max(abs(yBad - ytheory));
names{end+1} = 'DELETE feedback -> residual goes loud';
oks(end+1)   = r4 > 1e6*tolExact;
detail{end+1} = sprintf('mutated residual %.4g vs threshold %.1e', r4, tolExact);

% 5  DELETE EXPERIMENT: shift the closed form's exponent by one.  This is what
%    a "tidied" YTHEORY looks like, and it is invisible on a plot.
r5 = max(abs(y - X*(1 - (1-G).^((1:N)'-1))));
names{end+1} = 'DELETE exponent alignment -> residual goes loud';
oks(end+1)   = r5 > 1e6*tolExact;
detail{end+1} = sprintf('mutated residual %.4g vs threshold %.1e', r5, tolExact);

K.names  = names;
K.oks    = logical(oks);
K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    j = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{j}, detail{j});
end
end


% =============================================================================
function B = benchmarks(y, resid, ratio_hat, tau_s, G, X, TS, N)
%BENCHMARKS  Computed vs published, with the PROVENANCE of every band:
%     1e-12   the residual of an algebraic identity; 0 expected
%     0       a counted quantity
%     0.794%  0.5 pp / 63, the rounding of a two-figure percentage, and
%             nothing else went into it
%     shape   an ordering; no tolerance, and none is possible
%   Do NOT widen a band to turn a row green.
B = emptyBench();
tolExact = 1e-12*max(1, abs(X));
sgn = sign(X);

% ---- 1.  the listing's own claim: simulation == closed form ----------------
r1 = max(abs(resid));
B = addBound(B, 'simulation == closed form  max|y - YTHEORY|', ...
    sprintf('<= %.1e', tolExact), sprintf('%.4e', r1), r1 <= tolExact, ...
    'p.10 prose + closed form (zarchan.txt:324, :315)', '-', ...
    ['at the listing''s own G=0.5, X=1 every value is a dyadic rational, ' ...
     'so this should be EXACTLY 0 -- a small nonzero is already a rewrite']);

% ---- 2.  the loop bound ----------------------------------------------------
if N == 20
    B = addVal(B, 'iterations of the difference equation', 20, N, ...
        'integer (band 0)', 0, 'p.10 (zarchan.txt:323)', 'count', ...
        ['a LOCK on the default, not a discriminating check: it can only ' ...
         'pass or skip']);
else
    B = addSkip(B, 'iterations of the difference equation', '20', ...
        sprintf(['the book''s 20 is Listing 1.2''s own loop bound; this run ' ...
                 'used N = %d, so it is not reproducing that listing'], N), ...
        'p.10 (zarchan.txt:323)', 'count');
end

% ---- 3.  THE ONE INDEPENDENT NUMBER IN THE PASSAGE -------------------------
%   63% of steady state at one filter time constant.  Band = 0.5 pp on a
%   two-figure percentage, expressed relative to 63 so that addVal's relative
%   test means exactly that and nothing wider.
tol63 = 0.5/63;
if isfinite(ratio_hat) && ratio_hat > 0
    pct63 = 100*(1 - ratio_hat^(tau_s/TS));
    B = addVal(B, 'output at one time constant [% of steady state]', ...
        63, pct63, '+/-0.5 pp on a 2-figure percentage', tol63, ...
        'p.10 (zarchan.txt:324); same convention at :7710', '%', ...
        sprintf(['decay ratio %.6f MEASURED off the simulation, ' ...
                 'tau = -TS/ln(1-G) = %.5f s; NOT interpolated between ' ...
                 'samples (tau lies between n=1 and n=2, where interpolation ' ...
                 'reads 61.1%% -- a grid artefact)'], ratio_hat, tau_s));
else
    B = addSkip(B, 'output at one time constant [% of steady state]', '63', ...
        sprintf(['the measured decay ratio is not usable: (X-y_N)/X = %.3e, ' ...
                 'so ((X-y_N)/X)^(1/N) is not a positive real'], ...
                (X - y(N))/X), 'p.10 (zarchan.txt:324)', '%');
end

% ---- 4.  shape: rises to the input, no overshoot ---------------------------
dmin = min(sgn*diff(y));
over = max(sgn*(y - X));
epsX = 8*eps*abs(X);
B = addShape(B, 'output rises to the input and never overshoots', ...
    'monotone toward X, no sample beyond X', ...
    sprintf('min step %+.3e, max overshoot %+.3e', dmin, over), ...
    dmin >= -epsX && over <= epsX, ...
    'p.10 (zarchan.txt:324, Fig. 1.3 caption :329)', ...
    ['an ordering, not a plotted value -- Fig. 1.3 became an image link at ' ...
     ':321/:331 and carries no readable number']);

% ---- 5.  convergence half of the same sentence -----------------------------
relEnd  = abs(y(N) - X)/abs(X);
predEnd = (1-G)^N;
if predEnd <= 1e-5
    B = addBound(B, 'output has converged to the input  |y_N - X|/|X|', ...
        '<= 1e-5', sprintf('%.4e', relEnd), relEnd <= 1e-5, ...
        'p.10 (zarchan.txt:324)', '-', ...
        ['"eventually matches" read as five decimals; the ordering row ' ...
         'above carries the shape half of the same sentence']);
else
    B = addSkip(B, 'output has converged to the input  |y_N - X|/|X|', ...
        '<= 1e-5', ...
        sprintf(['(1-G)^N = %.3g > 1e-5 at G = %g, N = %d, so this run has ' ...
                 'not reached steady state and the claim is not testable on ' ...
                 'it.  The band is NOT widened to absorb that.'], ...
                predEnd, G, N), 'p.10 (zarchan.txt:324)', '-');
end

% ---- 6/7.  reported, never scored ------------------------------------------
B = addInfo(B, 'listing data statement', 'G=.5; X=1.; TS=.1', ...
    sprintf('G=%g X=%g TS=%g N=%d -> tau=%.5f s', G, X, TS, N, tau_s), ...
    'p.9 listing (zarchan.txt:317)', ...
    'inputs we set, not results: scoring them would only check our own typing');
B = addSkip(B, 'filter time constant [s]', 'not printed', ...
    ['Chapter 1 states only the 63% DEFINITION (:324) and never a number ' ...
     'for tau; Fig. 1.3 carries no scale, so there is nothing to score ' ...
     'against.  tau is reported on the INFO row above instead.'], ...
    'p.10 (zarchan.txt:324, :329)', 's');
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want.  Schema is the
%   one mgs.zarchan.benchSuite's shared benchRows() consumes.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
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
%ADDINFO  Reported, never scored: skipped = true keeps it out of the pass count
%   in BOTH directions -- an INFO row can neither pass nor fail.
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
L = {};
p = R.p;
L{end+1} = 'Zarchan Listing 1.2 -- DIFFERENCE EQUATION SIMULATION';
L{end+1} = '   first-order fading-memory filter, step response vs closed form';
L{end+1} = sprintf('   G = %g   X = %g   TS = %g s   N = %d   tau = %.5f s', ...
    p.G, p.X, p.TS, p.N, p.tau_s);
L{end+1} = sprintf('   units: %s', p.units);
L{end+1} = '';
L{end+1} = sprintf('%5s %9s %14s %14s %13s', 'n', 't [s]', 'y (sim)', ...
    'YTHEORY', 'y - YTHEORY');
stride = max(1, ceil(p.N/20));
rows   = unique([1:stride:p.N, p.N]);
for r = rows
    L{end+1} = sprintf('%5d %9.3f %14.10f %14.10f %13.3e', ...
        R.n(r), R.t(r), R.y(r), R.y_theory(r), R.resid(r)); %#ok<AGROW>
end
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-44s %s', vstr(K.oks(i)), K.names{i}, ...
        K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning';
    L{end+1} = '            NOT CHECKED, which is not the same thing as "fine".';
else
    L{end+1} = 'BENCHMARKS  (book values grepped from zarchan.txt; line numbers in .source)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
            case 'info'
                L{end+1} = sprintf('   INFO %-50s %s', b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case 'skip'
                L{end+1} = sprintf('   SKIP %-50s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case {'shape','bound'}
                L{end+1} = sprintf('   %-4s %-50s %s', vstr(b.ok), b.what, ...
                    b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf(['   %-4s %-50s book %s %s / ours %s %s ' ...
                    '(%+.3f%%, band %.4g%%)'], vstr(b.ok), b.what, ...
                    b.book_str, b.units, b.ours_str, b.units, b.pct, ...
                    100*b.tol); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = sprintf('   %d row(s) adjudicated, %d reported only.', ...
        sum(ev), numel(R.bench) - sum(ev));
end
s = strjoin(L, newline);
end

function s = vstr(tf)
if tf
    s = 'PASS';
else
    s = 'FAIL';
end
end


% =============================================================================
function drawCurves(R)
%DRAWCURVES  The listing's own figure: both curves, its own labels.
figure;
plot(R.t, R.y, '-o', R.t, R.y_theory, '--'); grid on
title('Output');  xlabel('T (S)');  ylabel('Y');
legend('simulation', 'closed form', 'Location', 'southeast');
end
