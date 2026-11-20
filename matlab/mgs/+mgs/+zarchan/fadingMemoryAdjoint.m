function R = fadingMemoryAdjoint(opts)
%MGS.ZARCHAN.FADINGMEMORYADJOINT  Zarchan Listing 7.3 -- the ADJOINT of a
%   homing loop containing a two-state digital FADING MEMORY filter.
%
%   R = mgs.zarchan.fadingMemoryAdjoint()
%   R = mgs.zarchan.fadingMemoryAdjoint('BETA', 0.3)
%   R = mgs.zarchan.fadingMemoryAdjoint('TS', 0.05, 'assert', false)
%
%   Line-by-line port of Listing 7.3 (zarchan.txt:2160-2203).  ONE run gives
%   the miss-vs-flight-time curve for BOTH disturbances of Fig. 7.1 at every
%   flight time at once: target-maneuver miss (Fig. 7.17) and measurement-noise
%   miss (Fig. 7.16).  The book got the same two curves from Listing 7.2 by
%   flying 1000 sorties -- 50 runs at each of 20 flight times (:2064).
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, rad for the LOS angle, g = 32.2
%   ft/s^2.  NOT SI.  XNT = 96.6 ft/s^2 is 3 g; SIGNOISE = 0.001 rad is the
%   book's "1 mr" of seeker noise.  Nothing is converted, so every printed
%   number lies straight against a published one.  Convert AT THE CALL SITE.
% ===========================================================================
%
%   *** WHAT IT IS NOT. ***  A LINEARISED, PLANAR loop with constant closing
%   velocity, a UNITY (lag-free) flight control system, no gravity, drag,
%   acceleration limit or LOS floor.  The only dynamics in the loop are the
%   sampled filter itself.  A clean-loop baseline, not an aircraft prediction.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_07_03.m does not parse.  Three repairs, all
%   forced by shape, none a judgement call, plus one deletion:
%
%     1  "while TP , = (TF-1e-5)" -> "while TP <= (TF-1e-5)".  This book's
%        extraction renders "<" as ",".  TP only increases, so ">=" never
%        terminates and "==" never fires.
%     2  "while STEP , =1" -> "while STEP <= 1".  Same substitution; the body
%        sets STEP = 2 to leave, which only makes sense against "<=".
%     3  "if S . =(TS-.0001)" -> "if S >= (TS-.0001)".  "." is the OCR of ">".
%        S resets to 0 inside the block, so "<=" would fire every step and run
%        the filter at the INTEGRATION rate.  Corroborated: Listing 7.1
%        (:2046) carries the same test in a program whose prose (:2026) says
%        the difference equations are solved every Ts.  SELF-CHECK 5 measures
%        the consequence -- it is the row that would catch the wrong choice.
%     4  The extractor flags one line of prose inside the code: it is the
%        heading "SOME PROPERTIES OF FADING MEMORY FILTERS" (:2207) and the two
%        paragraphs under it, leaking in after the listing ends at :2203.
%        Dropped, not "fixed" into code -- those paragraphs supply rows 5-7,
%        which is how they were identified.
%
%   Book quirks reproduced deliberately:
%     * THERE IS NO X4.  The listing carries X1, X2, X3, X5; x4 belongs to
%       Fig. 7.15's numbering and never becomes a state.  A renumbered state is
%       a renamed benchmark, so it is left alone.
%     * TP starts at +1e-5 and TGO carries a SECOND +1e-5.  Kept: the adjoint
%       gains are 1/(Vc*tgo) and 1/(Vc*tgo^2), so an exact zero divides by zero
%       on the first derivative evaluation.
%     * "count=1" with the first store at count=2 leaves index 1 at zero.  That
%       is MATLAB auto-fill, not a measurement -- but it IS the right value
%       (zero flight time, zero miss), so the leading (0,0,0) sample is kept
%       and flagged rather than silently dropped.
%     * Curves are recorded ONLY inside the sampler block, so the returned
%       vectors are at the SAMPLE times (100 points at the book's settings),
%       not at the 1000 integration steps.  That is what Figs. 7.16/7.17 plot.
%
%   NOT the book's: the sampler literal ".0001" is kept, but H < 5e-4 s is
%   REFUSED rather than silently changing that test's meaning (at H = 1e-4 it
%   fires one step early every time and the filter drifts ahead of the plant
%   with no symptom).  The book's own "Ts/H must be a large integer" (:2026,
%   repeated for the adjoint at :2177) is enforced the same way, as an error,
%   because every violation returns a smooth, plotable, entirely wrong curve.
%
%   THE IMPULSE IS ON X3, AND THE BOOK SAYS SO (:2176, general rule at :1697).
%   Trace it: X1D = X2 and X2D = X3 + ..., so x3 -> x2 -> x1 reverses
%   nT -> ydd -> yd -> y.  Put it on x1 or x2 and the program still runs,
%   smoothly, at a plausible magnitude, answering a different question.  No
%   symptom -- which is what SELF-CHECK 1 is for.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS -- honest headline: CHAPTER 7 PUBLISHES ALMOST NO NUMBERS.
%   Every miss distance in it lives on Figs. 7.4-7.5 and 7.16-7.22, and the
%   extraction turned every figure into an image link.  Two values were
%   greppable; the rest of what the chapter asserts is ORDERINGS, so orderings
%   are what is scored.  Nothing here was read off a figure.
%
%     1,2  G = 1-b^2 and H = (1-b)^2, Table 7.1 two-state row (:2011-2012):
%          0.36 and 0.04 at the book's b = 0.8, which is the b this listing
%          sets (:2196).  Listing 7.1 (:2044) writes the same two EXPRESSIONS
%          but at b = 0.3, so it corroborates the formulae and not the values.
%          Skipped off-nominal.
%     3    SKIPPED DELIBERATELY: adjoint vs the Listing 7.2 Monte Carlo.  ":2187
%          -- both methods yield approximately the same answers" is the
%          chapter's headline and would be the strongest row here, but the
%          Monte Carlo curves exist only on Figs. 7.16/7.17, so scoring it
%          needs Listing 7.2 PORTED (50 runs x 20 flight times, Gaussian seeker
%          noise) -- a different listing.  Emitted as a SKIP carrying that
%          arithmetic, NOT replaced by a weaker row wearing its name.
%     4    ":2064 -- the noise-induced miss is small compared to the
%          target-maneuver-induced miss for most flight times" at b = 0.8.
%          Scored as |noise| < |maneuver| at more than half the tF >= 1 s
%          samples.
%     5,6  Figs. 7.18/7.19 (:2208): the faster filter (b = 0.3) gives less
%          maneuver miss, and -- the book calls this surprising -- less noise
%          miss too.  Row 6 corroborated from Ch.9 (:2885), which says that for
%          THIS filter "decreasing the filter bandwidth always appeared to
%          increase miss distance".
%     7    That sentence's ADVERBS: "dramatically" (maneuver) against
%          "slightly" (noise), scored as an ordering of the two fractional
%          improvements -- the only defensible reading of an adverb.
%     8,9  Figs. 7.21/7.22 (:2218, :2230): halving Ts from 0.1 to 0.05 s cuts
%          the maneuver miss and the noise miss.  Row 9 holds sigma FIXED,
%          which is the experiment Ch.7 actually ran; the appendix says so when
%          it comes back to correct it (:14373).
%     10   Fig. A.6 (:14383-14384): under constant noise SPECTRAL DENSITY sigma
%          must rise 41.4% when Ts halves (the book prints "2.5 = 1.414", i.e.
%          2^0.5 with the exponent lost in extraction) and the noise miss
%          "still decreases ... although not as dramatically".  Scored as
%          sqrt(2)*noise(Ts/2) < noise(Ts) AND that gain smaller than row 9's.
%
%   ⚠️ Rows 4-10 are ORDERINGS, weak taken alone, not weak collectively: 5 and
%   6 point the same way for different reasons, 7 constrains their relative
%   size, 8-10 repeat the exercise over a different parameter.  A transcription
%   error in the difference-equation block moves all of them.
%
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED -- this file has never
%   been executed; MATLAB is the caller's. ***  The default run must give 100
%   filter updates (TF/Ts) out of 1000 steps (TF/H), a 101-point curve counting
%   the leading (0,0,0) sample, and a noise-miss curve that never decreases
%   (y5 is a running sum of squares).  SELF-CHECK 1 must return x1 = t^2/2 to
%   roundoff, i.e. 50 s^2 at tF = 10 s, hence a deleted-feedback maneuver miss
%   of 96.6*50 = 4830 ft.  The SCORED rows are orderings whose book side is an
%   inequality, so NO numeric prediction is offered for them -- do not read one
%   into a green board.
%
% ---------------------------------------------------------------------------
%   OUTPUTS
%     R.t          adjoint time [s].  This IS the original system's FLIGHT
%                  TIME: one sweep, every flight time, one run.
%     R.mnt_ft     XNT*x1 -- miss due to a constant target maneuver [ft]
%     R.noise_ft   SIGNOISE*sqrt(y5) -- miss due to LOS angle noise [ft]
%     R.x1 x2 x3 x5, R.y5   states at the sample times (there is no x4)
%     R.fast R.rate  the b = beta_fast and Ts/2 runs ([] if those rows skipped)
%     R.bench      benchmark rows, schema shared with the other ports
%     R.selfcheck  .names/.oks/.detail -- structural, NOT book values
%     R.pass       true only if at least one row was SCORED and all passed
%     R.verdict    '' when everything passed, else the one-line reason.  ALWAYS
%                  FORMED; 'assert' chooses errors vs warn-and-return and
%                  nothing else, so a failing run still hands back the
%                  evidence.  benchSuite runs every port with assert off.
%     R.units      one line naming the units
%
%   See also MGS.ZARCHAN.SHAPINGFILTERADJOINT (Listing 4.6 -- the continuous
%   one-run dispersion), MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 7.3 line 1, verbatim (zarchan.txt:2160).
    opts.XNT      (1,1) double {mustBeFinite, mustBePositive}    = 96.6   % ft/s^2 = 3 g
    opts.XNP      (1,1) double {mustBeFinite, mustBeNonnegative} = 3.0    % N-prime
    opts.TF       (1,1) double {mustBeFinite, mustBePositive}    = 10.0   % s, sweep end
    opts.TS       (1,1) double {mustBeFinite, mustBePositive}    = 0.1    % s, sampling
    opts.BETA     (1,1) double {mustBeFinite}                    = 0.8    % fading memory b
    opts.SIGNOISE (1,1) double {mustBeFinite, mustBeNonnegative} = 0.001  % rad = 1 mr
    opts.VC       (1,1) double {mustBeFinite, mustBePositive}    = 4000.0 % ft/s
    opts.H        (1,1) double {mustBeFinite, mustBePositive}    = 0.01   % s, RK2 step
    % The book's OWN comparison point for Figs. 7.18/7.19: zarchan.txt:2001
    % reduces b "from 0.8 to 0.3".  Rows 5-7 compare this run against BETA.
    opts.beta_fast (1,1) double {mustBeFinite} = 0.3
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options, MATLAB reports nargin = 0 no matter how
    % many were passed, so an `if nargin == 0` here would never fire.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

XNT = opts.XNT;  XNP = opts.XNP;  TF = opts.TF;  TS = opts.TS;
BETA = opts.BETA;  SIGNOISE = opts.SIGNOISE;  VC = opts.VC;  H = opts.H;

if ~(BETA > 0 && BETA < 1)
    error('mgs:zarchan:fadingMemoryAdjoint:beta', ...
        ['BETA = %g.  Table 7.1 (zarchan.txt:1964) defines b as "a constant ' ...
         'between zero and unity"; outside that the gains G = 1-b^2 and ' ...
         'H = (1-b)^2 stop describing a filter.'], BETA);
end
checkGrid(TS, H, TF);

% =========================================================================
%   LISTING 7.3.  Transcribed from zarchan.txt:2160-2203.
% =========================================================================
N = runAdj(XNT, XNP, TF, TS, BETA, SIGNOISE, VC, H, true);

% Structural companions.  One sweep each, neither is a book value; see
% structuralChecks for what each one is entitled to prove.
O = runAdj(XNT, XNP, TF, TS, BETA, SIGNOISE, VC,   H, false);  % feedback deleted
V = runAdj(XNT, XNP, TF, TS, BETA, SIGNOISE, 3*VC, H, true);   % Vc scaling law

% The book's two comparison runs (Figs. 7.18/7.19 and 7.21/7.22).  The Ts/2 run
% is taken only when Ts/2 is still a large integer multiple of H; if it is not,
% rows 8-10 SKIP rather than being scored on a grid the book did not use.
A = [];
if opts.beta_fast > 0 && opts.beta_fast < 1 && opts.beta_fast < BETA
    A = runAdj(XNT, XNP, TF, TS, opts.beta_fast, SIGNOISE, VC, H, true);
end
B2 = [];
if gridOK(TS/2, H)
    B2 = runAdj(XNT, XNP, TF, TS/2, BETA, SIGNOISE, VC, H, true);
end

R = struct();
R.t        = N.tp;
R.mnt_ft   = N.mnt;
R.noise_ft = N.noi;
R.x1 = N.x1;  R.x2 = N.x2;  R.x3 = N.x3;  R.x5 = N.x5;  R.y5 = N.y5;
R.fast = A;   R.rate = B2;
R.units = 'ft, ft/s, ft/s^2, rad (LOS angle), s; g = 32.2 ft/s^2';
R.p = struct('XNT',XNT, 'XNP',XNP, 'TF',TF, 'TS',TS, 'BETA',BETA, ...
             'SIGNOISE',SIGNOISE, 'VC',VC, 'H',H, ...
             'GFILTER',1-BETA^2, 'HFILTER',(1-BETA)^2, ...
             'beta_fast',opts.beta_fast, 'n_samples',N.nsamp, ...
             'n_steps',N.nstep, 'n_rhs',N.neval, 'units',R.units);

R.selfcheck = structuralChecks(N, O, V, XNT, TF, TS, H);

if opts.benchmark
    R.bench = benchmarks(N, A, B2, BETA, TS, opts.beta_fast);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine" -- that is the shape of green light this repo has been burned
    % by before.
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R); end

% ---- adjudication.  The verdict is FORMED either way; 'assert' chooses the
%      reaction to it and nothing else.  A port that errors unconditionally
%      cannot be diagnosed, because the caller never gets R.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['not one book claim could be evaluated at b = %g, ' ...
        'Ts = %g s, N'''' = %g.  An unvalidated port must not be handed back ' ...
        'silently.'], BETA, TS, XNP);
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port ' ...
        'disagrees with ITSELF, which is a wiring fault, not a tolerance ' ...
        'question.  First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf(['%d of %d evaluated book claims failed (first: %s -- ' ...
        'book %s, ours %s).'], numel(bad), sum(ev), R.bench(bad(1)).what, ...
        R.bench(bad(1)).book_str, R.bench(bad(1)).ours_str);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:fadingMemoryAdjoint:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN.  R comes back complete so the caller can see WHICH
        % side of a comparison moved.
        warning('mgs:zarchan:fadingMemoryAdjoint:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function ok = gridOK(TS, H)
%GRIDOK  Is Ts/H "a large integer" in the book's sense (zarchan.txt:1990)?
r  = TS/H;
ok = abs(r - round(r)) <= 1e-9*max(r,1) && round(r) >= 5 && H >= 5e-4;
end


% =============================================================================
function checkGrid(TS, H, TF)
%CHECKGRID  Refuse, do not warn.  Every failure here returns a smooth,
%   plotable, entirely wrong curve.
r = TS/H;
if abs(r - round(r)) > 1e-9*max(r,1) || round(r) < 5
    error('mgs:zarchan:fadingMemoryAdjoint:grid', ...
        ['Ts/H = %g.  The book requires this ratio to be "a large integer so ' ...
         'that effects in between sampling instants are treated properly and ' ...
         'accurately" (zarchan.txt:1990; repeated for the adjoint at :2177). ' ...
         'Ts = %g s, H = %g s.'], r, TS, H);
end
if H < 5e-4
    error('mgs:zarchan:fadingMemoryAdjoint:step', ...
        ['H = %g s.  The listing tests its sample accumulator against the ' ...
         'LITERAL "TS - .0001", which is kept here; below H = 5e-4 s that ' ...
         'literal fires one step early on every sample and the filter drifts ' ...
         'ahead of the plant with no symptom.'], H);
end
if TF < 2*TS
    error('mgs:zarchan:fadingMemoryAdjoint:short', ...
        'TF = %g s does not contain two samples at Ts = %g s.', TF, TS);
end
end


% =============================================================================
function U = runAdj(XNT, XNP, TF, TS, BETA, SIGNOISE, VC, H, fbOn)
%RUNADJ  Listing 7.3, zarchan.txt:2160-2203, verbatim except as documented in
%   the header.  Every constant is an ARGUMENT: MATLAB local functions cannot
%   see the main function's workspace, and a previous wave in this package
%   learned that at runtime rather than at lint time.
%
%   fbOn = false DELETES the Y4 feedback from the two derivative lines, opening
%   the adjoint loop.  It exists for SELF-CHECKS 1 and 2 and for nothing else;
%   the book has no such switch.

GFILTER = 1 - BETA^2;          % Table 7.1, two-state row (zarchan.txt:1975)
HFILTER = (1 - BETA)^2;        % Table 7.1, two-state row (zarchan.txt:1976)
fb = double(logical(fbOn));

T = 0.0;  S = 0.0;  TP = T + 0.00001;   % NOT 0: the gains are 1/(Vc*tgo)
X1 = 0;  X2 = 0;  X3 = 1;  X5 = 0.0;    % <<< THE IMPULSE, on x3 (:2176)
Y1OLD = 0.0;  Y2OLD = 0.0;  Y3OLD = 0.0;  Y4OLD = 0.0;  Y5OLD = 0.0;

% Derivatives initialised before the loop so the FLAG==1 branch inside it
% cannot read a value left over from anything else.
X1D = 0.0;  X2D = 0.0;  X3D = 0.0;  X5D = 0.0;

nmax = ceil(TF/TS) + 2;
tp = zeros(nmax,1);  mnt = zeros(nmax,1);  noi = zeros(nmax,1);
x1 = zeros(nmax,1);  x2  = zeros(nmax,1);  x3  = zeros(nmax,1);
x5 = zeros(nmax,1);  y5  = zeros(nmax,1);
% The book's "count=1" with the first store at count=2 leaves index 1 at zero.
% Kept -- it is the correct (0 s, 0 ft, 0 ft) sample.  See the header.
count = 1;
nstep = 0;  neval = 0;

while TP <= (TF - 1e-5)
    X1OLD = X1;  X2OLD = X2;  X3OLD = X3;  X5OLD = X5;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  Pass 1 evaluates the derivatives at
    % the current state; pass 2 takes a full Euler step, advances TP by H and
    % re-evaluates there.  TP advances INSIDE this loop, exactly once, which is
    % what makes the second evaluation happen at t+H.  Move that line out and
    % the scheme degrades to first order while still running smoothly.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            X1 = X1 + H*X1D;  X2 = X2 + H*X2D;
            X3 = X3 + H*X3D;  X5 = X5 + H*X5D;
            TP = TP + H;
        end
        TGO = TP + 0.00001;              % adjoint time IS time-to-go
        X1D = X2;
        X2D = X3 + fb*Y4OLD/(VC*TGO);
        X3D = fb*Y4OLD/(VC*TGO*TGO);
        X5D = -X2;                       % x5 is the adjoint LOS-angle node
        FLAG = 1;
        neval = neval + 1;
    end
    % The listing writes "FLAG=0" here as well.  Dropped: the outer loop
    % re-initialises FLAG on its next pass, so that assignment cannot be read.
    % Recorded here rather than as code so nobody re-adds it thinking a state
    % machine was lost.

    % Heun corrector, in the listing's own algebra: (x_old + x_pred)/2 plus
    % (H/2)*f(t+H, x_pred).  X1D..X5D here are the SECOND-pass derivatives.
    X1 = (X1OLD + X1)/2 + 0.5*H*X1D;
    X2 = (X2OLD + X2)/2 + 0.5*H*X2D;
    X3 = (X3OLD + X3)/2 + 0.5*H*X3D;
    X5 = (X5OLD + X5)/2 + 0.5*H*X5D;
    nstep = nstep + 1;

    S = S + H;
    if S >= (TS - 0.0001)                % OCR "." -> ">"; header repair 3
        S = 0.0;
        % ---- the two-state fading memory filter, ADJOINTED ----------------
        TEMP1 = (X5 - Y1OLD)*XNP*VC;
        TEMP2 = HFILTER*(Y2OLD + TEMP1)/TS + GFILTER*Y3OLD;
        Y1NEW = X5;
        Y2NEW = TEMP1 + Y2OLD + TS*(Y3OLD - TEMP2);
        Y3NEW = Y3OLD - TEMP2;
        Y4NEW = Y4OLD + TEMP2;
        Y5NEW = Y5OLD + TEMP2*TEMP2;     % square AND SUM -- the noise is digital
        Y1OLD = Y1NEW;  Y2OLD = Y2NEW;  Y3OLD = Y3NEW;
        Y4OLD = Y4NEW;  Y5OLD = Y5NEW;

        count = count + 1;
        tp(count)  = TP;
        mnt(count) = XNT*X1;
        noi(count) = SIGNOISE*sqrt(Y5NEW);
        x1(count) = X1;  x2(count) = X2;  x3(count) = X3;
        x5(count) = X5;  y5(count) = Y5NEW;
    end
end

U = struct('tp',tp(1:count), 'mnt',mnt(1:count), 'noi',noi(1:count), ...
           'x1',x1(1:count), 'x2',x2(1:count), 'x3',x3(1:count), ...
           'x5',x5(1:count), 'y5',y5(1:count), ...
           'nsamp',count-1, 'nstep',nstep, 'neval',neval, ...
           'TS',TS, 'BETA',BETA, 'VC',VC);
end


% =============================================================================
function K = structuralChecks(N, O, V, XNT, TF, TS, H)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.  They are the weaker
%   claim that it agrees with itself -- which is what catches wiring faults.
%
%   ⚠️ .names and .oks are what put these on the board.  Two earlier ports in
%   this package computed a full set of invariants -- delete experiments
%   included -- stored them under private field names, and contributed ZERO
%   structural rows to a board that then looked green.  Keep both fields, keep
%   them the same length.
names = {};  oks = [];  detail = {};

% 1  OPEN LOOP IS EXACT.  With the Y4 feedback deleted the adjoint collapses to
%    x3' = 0, x2' = x3, x1' = x2 with x3(0) = 1 -- so x1 = t^2/2, and Heun
%    integrates that exactly (trapezoid on a linear integrand).  This is the
%    sharpest cheap test of BOTH the impulse placement and the integrator: put
%    the impulse on x1 or x2 and this residual explodes, while the closed-loop
%    curve stays entirely plausible-looking.
tOpen  = O.tp(2:end) - 1e-5;            % elapsed time; TP is offset by 1e-5
xExact = 0.5*tOpen.^2;
K.open_resid = max(abs(O.x1(2:end) - xExact))/max(xExact);
names{end+1}  = 'open loop (feedback deleted): x1 == t^2/2 exactly';
oks(end+1)    = K.open_resid <= 1e-12;
detail{end+1} = sprintf('max |x1 - t^2/2| / peak = %.3e  (x1(tF) = %.6f s^2)', ...
                        K.open_resid, O.x1(end));

% 2  DELETE EXPERIMENT.  Restoring that feedback must change the answer by a
%    lot.  If it does not, the sampled filter is not in the loop at all and
%    every benchmark below is scoring a bare double integrator.  Open-loop miss
%    is XNT*TF^2/2 = 4830 ft at the book's settings, against a guided miss of
%    order tens of feet.
K.delete_ratio = abs(O.mnt(end))/max(abs(N.mnt(end)), realmin);
names{end+1}  = 'DELETE: removing the Y4 feedback changes maneuver miss > 3x';
oks(end+1)    = K.delete_ratio > 3;
detail{end+1} = sprintf('open %.1f ft vs closed %.4g ft (ratio %.1f; XNT*TF^2/2 = %.1f ft)', ...
                        O.mnt(end), N.mnt(end), K.delete_ratio, XNT*TF^2/2);

% 3  THE VC SCALING LAW.  Vc cancels out of the CONTINUOUS states -- TEMP1, and
%    hence Y2..Y4, scale with Vc, and they re-enter only as Y4/(Vc*tgo) and
%    Y4/(Vc*tgo^2) -- but it survives in the DISCRETE accumulator, where
%    y5 = sum TEMP2^2 scales as Vc^2.  So the maneuver miss must be independent
%    of Vc and the noise miss exactly linear in it.  One run at 3*Vc tests
%    both, and it is the only thing that would notice a future edit
%    introducing a real Vc dependence.
K.vc_mnt   = maxRel(V.mnt, N.mnt);
K.vc_noise = maxRel(V.noi, 3*N.noi);
names{end+1}  = 'Vc cancels from maneuver miss; noise miss is linear in Vc';
oks(end+1)    = K.vc_mnt <= 1e-9 && K.vc_noise <= 1e-9;
detail{end+1} = sprintf('maneuver %.2e, noise-vs-3x %.2e (both vs curve peak)', ...
                        K.vc_mnt, K.vc_noise);

% 4  y5 IS A RUNNING SUM OF SQUARES, so the noise miss cannot decrease with
%    flight time.  A sign error on TEMP2's accumulation shows up here and
%    nowhere else cheap.
K.noise_min_step = min(diff(N.noi));
names{end+1}  = 'noise miss non-decreasing (y5 accumulates TEMP2^2)';
oks(end+1)    = K.noise_min_step >= -1e-15;
detail{end+1} = sprintf('min consecutive change %+.3e ft', K.noise_min_step);

% 5  THE REPAIRED SAMPLER TEST (header repair 3) must fire exactly TF/Ts times
%    out of TF/H integration steps.  This is the invariant that would catch
%    "<=" having been chosen instead of ">=": that resolution fires on EVERY
%    step and would give TF/H samples, not TF/Ts.
K.nsamp = N.nsamp;
K.nstep = N.nstep;
names{end+1}  = 'sampler fired round(TF/Ts) times, not once per step';
oks(end+1)    = K.nsamp == round(TF/TS) && K.nstep == round(TF/H);
detail{end+1} = sprintf('%d samples / %d steps (expected %d / %d)', ...
                        K.nsamp, K.nstep, round(TF/TS), round(TF/H));

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
function r = maxRel(a, b)
%MAXREL  Max absolute difference NORMALISED BY THE REFERENCE CURVE'S PEAK.
%   Not pointwise relative: these curves cross zero, and a pointwise relative
%   error there divides a small number by a smaller one and manufactures a
%   failure out of nothing.
den = max(abs(b));
if ~(den > 0)
    r = Inf;                 % refuse to report "perfect" against a zero curve
    return
end
r = max(abs(a - b))/den;
end


% =============================================================================
function B = benchmarks(N, A, Bq, BETA, TS, beta_fast)
%BENCHMARKS  Ten rows: two values, one deliberate skip, seven orderings --
%   which is what Chapter 7 actually publishes.  See the header.
B = emptyBench();
SRC_TAB = 'Table 7.1, two-state row, p.138 (zarchan.txt:1975-2012)';
SRC_MC  = 'Figs. 7.16/7.17, p.152 (zarchan.txt:2151)';
SRC_B   = 'Figs. 7.18/7.19 prose, p.155 (zarchan.txt:2172)';
SRC_R1  = 'Fig. 7.21 prose, p.156 (zarchan.txt:2182)';
SRC_R2  = 'Fig. 7.22 prose, p.157 (zarchan.txt:2194)';
SRC_A6  = 'Fig. A.6 prose, p.960-961 (zarchan.txt:14196-14384)';

% ---- 1,2  the published filter gains ------------------------------------
if abs(BETA - 0.8) < 1e-12
    B = addVal(B, 'G = 1 - b^2 at b = 0.8', 0.36, 1-BETA^2, '1e-10', 1e-10, ...
        SRC_TAB, '-', 'corroborated by the listing''s own line, zarchan.txt:2160');
    B = addVal(B, 'H = (1 - b)^2 at b = 0.8', 0.04, (1-BETA)^2, '1e-10', 1e-10, ...
        SRC_TAB, '-', ['the EXPRESSION (not this value) is corroborated by ' ...
        'Listing 7.1, zarchan.txt:2008, which writes the same HFILTER at b = 0.3']);
else
    B = addSkip(B, 'G = 1 - b^2 at b = 0.8', '0.36', sprintf( ...
        ['this run used b = %g.  Table 7.1 publishes the two gains, not a ' ...
         'curve, so there is nothing to score off-nominal.'], BETA), SRC_TAB, '-');
    B = addSkip(B, 'H = (1 - b)^2 at b = 0.8', '0.04', ...
        sprintf('this run used b = %g.', BETA), SRC_TAB, '-');
end

% ---- 3  the row this chapter is really about, and cannot supply ---------
B = addSkip(B, 'adjoint vs the Listing 7.2 Monte Carlo', ...
    '"approximately the same answers"', ...
    ['the 1000-run Monte Carlo curves exist only on Figs. 7.16/7.17 and the ' ...
     'extraction turned every figure into an image link, so there is no ' ...
     'published number to score against.  Scoring it needs Listing 7.2 ' ...
     'PORTED -- 50 runs at each of 20 flight times with Gaussian seeker ' ...
     'noise -- which is a different listing.  Deliberately NOT replaced by a ' ...
     'weaker row wearing its name.'], SRC_MC, 'ft');

% ---- 4  noise miss is the small one at b = 0.8 --------------------------
sel = N.tp >= 1.0;
if any(sel)
    frac = sum(abs(N.noi(sel)) < abs(N.mnt(sel)))/sum(sel);
    med  = median(abs(N.noi(sel))./max(abs(N.mnt(sel)), realmin));
    B = addShape(B, 'noise miss < maneuver miss for most flight times (b = 0.8)', ...
        '"the noise-induced miss is small compared to the target-maneuver-induced miss for most flight times"', ...
        sprintf('holds at %.0f%% of the tF >= 1 s samples', 100*frac), frac > 0.5, ...
        'Figs. 7.4/7.5 prose, p.142 (zarchan.txt:2028)', ...
        sprintf('median |noise|/|maneuver| = %.3f over %d sample times', med, sum(sel)));
else
    B = addSkip(B, 'noise miss < maneuver miss for most flight times', ...
        '"small compared to"', 'no sample time reaches tF = 1 s at these settings', ...
        'zarchan.txt:2028', '-');
end

% ---- 5,6,7  the faster filter, Figs. 7.18/7.19 --------------------------
if ~isempty(A)
    impM = 1 - abs(A.mnt(end))/max(abs(N.mnt(end)), realmin);
    impN = 1 - abs(A.noi(end))/max(abs(N.noi(end)), realmin);
    B = addShape(B, sprintf('faster filter (b %g -> %g): less MANEUVER miss', BETA, beta_fast), ...
        '"dramatically improved miss distance results for the wider bandwidth filter"', ...
        sprintf('%.4g ft vs %.4g ft (%.0f%% less)', A.mnt(end), N.mnt(end), 100*impM), ...
        abs(A.mnt(end)) < abs(N.mnt(end)), SRC_B, ...
        'compared at tF = TF, the listing''s own flight time');
    B = addShape(B, sprintf('faster filter (b %g -> %g): less NOISE miss', BETA, beta_fast), ...
        '"slightly less miss distance due to line-of-sight angle noise for the faster filter"', ...
        sprintf('%.4g ft vs %.4g ft (%.0f%% less)', A.noi(end), N.noi(end), 100*impN), ...
        abs(A.noi(end)) < abs(N.noi(end)), SRC_B, ...
        'corroborated from Ch.9: "decreasing the filter bandwidth always appeared to increase miss distance" (zarchan.txt:2849)');
    B = addShape(B, '"dramatically" (maneuver) beats "slightly" (noise)', ...
        'maneuver improvement > noise improvement', ...
        sprintf('%.0f%% vs %.0f%%', 100*impM, 100*impN), impM > impN, SRC_B, ...
        'the only defensible reading of the two adverbs in one sentence');
else
    B = addSkip(B, 'faster filter comparison (Figs. 7.18/7.19)', 'less miss at lower b', ...
        sprintf('beta_fast = %g is not a usable faster filter against BETA = %g', ...
                beta_fast, BETA), SRC_B, 'ft');
end

% ---- 8,9,10  the higher data rate, Figs. 7.21/7.22 and A.6 --------------
if ~isempty(Bq)
    impM = 1 - abs(Bq.mnt(end))/max(abs(N.mnt(end)), realmin);
    impN = 1 - abs(Bq.noi(end))/max(abs(N.noi(end)), realmin);
    % Constant SPECTRAL DENSITY: sigma rises as 1/sqrt(Ts) -- by 41.4% when Ts
    % is halved (zarchan.txt:14196).  The noise miss is exactly linear in
    % SIGNOISE, so this is a scaling of the curve, not a rerun.
    kSig = sqrt(TS/Bq.TS);
    impC = 1 - kSig*abs(Bq.noi(end))/max(abs(N.noi(end)), realmin);
    B = addShape(B, sprintf('halving Ts (%g -> %g s): less MANEUVER miss', TS, Bq.TS), ...
        '"doubling the sampling rate ... dramatically reduces the miss sensitivity due to target maneuver"', ...
        sprintf('%.4g ft vs %.4g ft (%.0f%% less)', Bq.mnt(end), N.mnt(end), 100*impM), ...
        abs(Bq.mnt(end)) < abs(N.mnt(end)), SRC_R1, 'compared at tF = TF');
    B = addShape(B, sprintf('halving Ts (%g -> %g s): less NOISE miss, sigma fixed', TS, Bq.TS), ...
        '"the miss due to noise decreases with increasing sampling rate"', ...
        sprintf('%.4g ft vs %.4g ft (%.0f%% less)', Bq.noi(end), N.noi(end), 100*impN), ...
        abs(Bq.noi(end)) < abs(N.noi(end)), SRC_R2, ...
        'sigma held constant -- the experiment Ch.7 actually ran, as the appendix says at zarchan.txt:14186');
    B = addShape(B, 'constant spectral density: noise miss STILL falls, by less', ...
        '"still decreases with increasing data rate, although not as dramatically"', ...
        sprintf('%.0f%% less at sigma x %.3f (vs %.0f%% at fixed sigma)', ...
                100*impC, kSig, 100*impN), impC > 0 && impC < impN, SRC_A6, ...
        sprintf('the book''s own multiplier is +41.4%% ("2.5 = 1.414", i.e. 2^0.5); ours is %.4f', kSig));
else
    B = addSkip(B, 'higher data rate comparison (Figs. 7.21/7.22/A.6)', ...
        'less miss at Ts/2', sprintf( ...
        ['Ts/2 = %g s is not a large-integer multiple of H = the listing''s ' ...
         'step, so the halved-rate run would sit on a grid the book did not use'], ...
        TS/2), SRC_R1, 'ft');
end
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want: a struct array
%   that quietly reordered fields would make [B.ok] mean something else.  This
%   is the schema benchSuite's adaptGeneric consumes, and it is STRICT: a
%   near-miss (book_s for book_str) errors rather than being accommodated.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.4g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.6g', ours);
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
p = R.p;
L = {};
L{end+1} = 'mgs.zarchan.fadingMemoryAdjoint -- Zarchan Listing 7.3 (zarchan.txt:2160-2203)';
L{end+1} = 'adjoint of a homing loop with a two-state digital fading memory filter';
L{end+1} = sprintf('  units      : %s', R.units);
L{end+1} = sprintf('  params     : nT=%g ft/s^2  N''=%g  Vc=%g ft/s  b=%g  Ts=%g s  H=%g s  sigma=%g rad', ...
                   p.XNT, p.XNP, p.VC, p.BETA, p.TS, p.H, p.SIGNOISE);
L{end+1} = sprintf('  gains      : G = 1-b^2 = %.6g   H = (1-b)^2 = %.6g', ...
                   p.GFILTER, p.HFILTER);
L{end+1} = sprintf('  grid       : %d samples from %d steps (Ts/H = %g), %d rhs evaluations', ...
                   p.n_samples, p.n_steps, p.TS/p.H, p.n_rhs);
L{end+1} = sprintf('  at tF=%g s : maneuver miss %.4g ft   noise miss %.4g ft', ...
                   R.t(end), R.mnt_ft(end), R.noise_ft(end));
L{end+1} = '  -- structural invariants (self-consistency, NOT book values) --';
for k = 1:numel(R.selfcheck.names)
    L{end+1} = sprintf('   [%s] %s -- %s', tick(R.selfcheck.oks(k)), ...
                       R.selfcheck.names{k}, R.selfcheck.detail{k}); %#ok<AGROW>
end
L{end+1} = '  -- book rows --';
for k = 1:numel(R.bench)
    b = R.bench(k);
    if b.skipped
        L{end+1} = sprintf('   [--] %s -- %s: %s', b.what, upper(b.kind), b.why); %#ok<AGROW>
    else
        L{end+1} = sprintf('   [%s] %s -- book %s | ours %s', tick(b.ok), ...
                           b.what, b.book_str, b.ours_str); %#ok<AGROW>
    end
end
if any(ev)
    L{end+1} = sprintf('  scored %d row(s), %d passed.', sum(ev), sum([R.bench(ev).ok]));
else
    L{end+1} = '  NOTHING WAS SCORED.  R.pass is false meaning NOT CHECKED.';
end
s = strjoin(L, newline);
end

function c = tick(ok)
if ok, c = 'ok'; else, c = 'XX'; end
end


% =============================================================================
function drawCurves(R)
figure; plot(R.t, R.mnt_ft, 'LineWidth', 1.2); grid on
xlabel('Flight Time (Sec)'); ylabel('Target Maneuver Miss (Ft)');
title('Listing 7.3 -- Fig. 7.17');
figure; plot(R.t, R.noise_ft, 'LineWidth', 1.2); grid on
xlabel('Flight Time (Sec)'); ylabel('Noise Miss (Ft)');
title('Listing 7.3 -- Fig. 7.16');
end
