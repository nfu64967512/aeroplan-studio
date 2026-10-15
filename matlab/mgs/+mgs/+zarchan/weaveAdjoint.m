function R = weaveAdjoint(opts)
%MGS.ZARCHAN.WEAVEADJOINT  Zarchan Listing 20.2 -- adjoint of a single time
%   constant PN guidance system driven by a WEAVING (sinusoidal) target.
%
%   R = mgs.zarchan.weaveAdjoint()
%   R = mgs.zarchan.weaveAdjoint('W', 1.5)                 lower weave frequency
%   R = mgs.zarchan.weaveAdjoint('TAU', 0.5, 'assert', false)
%
%   Line-by-line port of Listing 20.2 (zarchan.txt:5999-6010), integrated by the
%   book's own second-order Runge-Kutta (Heun) scheme.
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: FEET, ft/s^2, rad/s, s.  g = 32.2 ft/s^2.  NOT SI.
%   nT = 193.2 ft/s^2 is 6 g (zarchan.txt:5943).  The rest of mgs is metres;
%   nothing is converted here, so every number printed can be laid straight
%   against a published one.  Convert AT THE CALL SITE and say so there.
% ===========================================================================
%
%   WHAT IT IS.  The target flies nT*sin(w*t).  The adjoint generates that by
%   driving a SECOND-ORDER SHAPING NETWORK (x6'' + w^2 x6 = x2) with the
%   adjoint impulse response, and the miss is XMWEAVE = nT*w*x6.  Everything
%   upstream of x5/x6 is identical to Listing 4.6's guidance loop, so the two
%   ports share their x2/x3/x4 closed forms; the resonator is the entire
%   difference and it is what the structural checks pin down.
%
%   *** WHAT IT IS NOT. ***  Linearised, planar, constant closing velocity,
%   FIRST-ORDER lag, no gravity, no drag, no acceleration limit, no LOS floor.
%   Ch.20 itself says the single time constant model UNDERSTATES the miss
%   (zarchan.txt:5938); the fifth-order figures are 4-5x larger (:6175, :6181).
%   Do not read a miss out of here and call it a prediction for an aircraft.
%
%   WHY IT MATTERS HERE.  One headline: for a weaving target the miss does NOT
%   decay with flight time (zarchan.txt:5959) -- it oscillates forever at the
%   TARGET's frequency, so whether you hit is "a matter of luck" (:6076).  A
%   step maneuver's miss dies out; a periodic one does not.  Our own weave work
%   fails by SATURATION, which is a different mechanism entirely -- this file is
%   the clean-loop baseline that saturation is measured against, not a model of
%   it.  The adjoint identity is a LINEARITY identity.
%
%   TWO DIFFERENT PEAKS, AND THEY ARE NOT THE SAME NUMBER.  This trips people:
%     * 28 ft  -- the GLOBAL peak over the flight-time sweep, transient
%                 included, at tF ~ 1.15 s (BENCHMARK 1).
%     * 18.3 ft -- the STEADY-STATE amplitude at the same settings, which is
%                 what every closed form in the chapter computes (BENCHMARK 2).
%   Comparing the 28 against the closed form "fails" by 53% and means nothing.
%
% ---------------------------------------------------------------------------
%   TRANSCRIPTION.  matlab/zarchan/_listings/listing_20_02.m; its header reports
%   NO unresolved variable-to-variable comparisons, and that is correct -- every
%   comparison in 20.2 is against a literal.  Three repairs, none a judgement:
%     1  "while TP , =(TF-1e-5)" -> "while TP <= (TF-1e-5)".  ',' is this book's
%        OCR of '<'.  Shape-forced: TP only increases, so '>=' never terminates.
%     2  Trailing line "CLOSED-FORM SOLUTIONS FOR MISS DISTANCE" is the next
%        SECTION HEADING leaking in (the listing ends :6010, the heading is
%        :6011).  Dropped, not "fixed" into code.
%     3  "save datfil.txt output/ascii" is a typo for -ascii (Listing 20.3, two
%        pages later at :6170, has it right).  Nothing is written to disk here.
%   Reproduced deliberately: TP starts at +1e-5 and TGO carries a SECOND +1e-5,
%   because the adjoint gain is N'/tgo and an exact zero divides by zero.
%   NOT reproduced: the listing records every 0.1 s ("if S >= .09999").  This
%   port defaults samp = H, i.e. every step.  Reason: peak detection and the
%   finite-difference structural checks both need the fine grid, and the record
%   interval affects the OUTPUT ARRAY ONLY -- never the integration.  Pass
%   'samp', 0.1 to get the listing's own record back, value for value.
%   VC = 4000 is set on the listing's first line and REFERENCED NOWHERE.  It is
%   vestigial (copied from the listings whose x3D carries a 1/VC).  Carried in
%   R.p so its deadness is visible rather than silently tidied away.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Every book value was grepped out of zarchan.txt and the line
%   number is in that row's .source.  Nothing was read off a figure: extraction
%   turned every figure into an image link.  Where the book states a SHAPE about
%   a figure, that prose IS greppable and is scored as an ordering.
%
%   The closed form the chapter publishes (:6027, :6093-6096, :6104-6107, :6108-6111) is
%       1 - H(s) = [s/(s + 1/T)]^N'   =>   miss/nT  <-  s^(N'-2)/(s + 1/T)^N'
%   so, with x = w*T,
%       peak steady-state miss = nT * T^2 * x^(N'-2) / (1 + x^2)^(N'/2)
%       phase                  = (N'-2)*pi/2 - N'*atan(x)
%   The book prints that magnitude for N' = 3, 4 and 5 separately (with an
%   explicit 32.2 because ITS nT is in g; ours is in ft/s^2, so no 32.2) and
%   prints the phase for N' = 3 only.  Rows 2/3 run at the CALLER's parameters
%   against this general form -- they are the only rows that survive off-book
%   settings.  Rows 1 and 4-9 PIN the book's own cases, so the board is always
%   the book's board; that means a caller who changes W or TAU gets a green
%   board that says nothing about their run, and the summary says so out loud.
%
%   BANDS -- every one is built from its part's PROVENANCE, in code:
%     integrator  3 * (0.5*N'*H/TAU).  MECHANISM: on the first step Y1 is exactly
%                 0 while the true x3'(0) is -N'/T, so Heun takes half the first
%                 slope and the corrector lands on 1 - 1.5H/T where the truth is
%                 1 - 3H/T (N'=3).  The response is linear in the impulse, so it
%                 becomes a CONSTANT RELATIVE bias of ~+0.5*N'*H/TAU.  Verified
%                 four ways below; the sibling shapingFilterAdjoint measures
%                 +1.88% against the same formula's +2.0%.  Band = 3x the
%                 prediction, so a correct run sits at a THIRD of its band and a
%                 run at the EDGE is a fault, not a pass.
%     phase       3 * (TF*w^3*H^2/6), Heun's accumulated phase error on the
%                 resonator (arg(1+jth-th^2/2) = th + th^3/6 per step).  Only
%                 BENCHMARK 3 needs it; amplitude rows take max|.| over a whole
%                 period, where phase cannot move the answer.
%     read-off    5%, rows 4-6 ONLY: those three numbers are the author reading
%                 Fig. 20.8's ordinate, and he reads 5.5 and 11.5 where his own
%                 printed formula gives 5.7601 and 11.3844 (-4.5%, -1.0%).
%     rounding    0.5/book, rows 1 and 4-6.  "8 ft" is a rounded 8.25.
%   Rows 4-6 are therefore WEAK by construction (bands 11%, 17%, 20%) -- on the
%   board because they are what the chapter publishes as numbers, not because
%   they discriminate.  Rows 2, 3, 7, 8 are the sharp ones.  Do NOT widen a band
%   to make a row green; every row states its expected residual in .detail.
%
%   NOT SCORED: Fig. 20.8's ordinate itself, and Figs. 20.9-20.12 with the
%   fifth-order numbers at :6175/:6181/:6185 -- those are Listing 20.3, a
%   different port.  No rand/randn anywhere here, so there is no seed to record.
%
% ---------------------------------------------------------------------------
%   WHAT THIS PRINTS.  Predicted from the closed form and the bias mechanism
%   BEFORE the first run, then MEASURED (R2025b, 2026-08-24).  Both columns are
%   given because the agreement is the evidence that the bias mechanism above is
%   a mechanism and not a story:
%                                              predicted   measured
%     global peak over tF = 0..10 s              ~27.45      27.44 ft
%     steady-state amplitude (closed form 18.33)   +1.5%     +1.49%
%     BENCHMARK 3 waveform residual                ~1.7%      1.81% of amplitude
%     BENCHMARK 4 / 5 / 6  vs 33 / 17 / 8 ft   +6.3/+3.5/+11.2  +6.3/+3.2/+11.1%
%     BENCHMARK 7 / 8  (N' = 4, 5)               +2.0/+2.5%  +1.9/+2.3%
%     structural x2 and x4 residuals               ~1.5%      1.44 / 1.44%
%   Off-book too: at N' = 4, TAU = 0.5 the formula predicts +4.0% and the run
%   gives +3.6%.  A residual near a band EDGE is a fault to investigate, not a
%   pass -- every one of these sits at about a third of its band by design.
%
%   FAULT INJECTION, measured, because a green board proves nothing on its own:
%     X6D = X2 instead of X5   -> the d(x6)/dt invariant reads 99.98% (band
%                                 0.1%) and ALL NINE scored rows go red
%     impulse on X2 not X3     -> two invariants and 8 of 9 rows go red
%     W dropped from XMWEAVE   -> every invariant stays green (it is an output
%                                 scaling, not a wiring fault) and 8 of 9 rows
%                                 go red.  This is what rows 4-8 are for, and
%                                 it is the fault that found the w = 1 blind
%                                 spot described at rows 7-8.
%
%   OUTPUTS.  R.t is adjoint time [s] = the original system's FLIGHT TIME: one
%   sweep gives every flight time, which is the whole point.  R.x2..R.x6 are the
%   listing's five states (there is no x1 in 20.2).  R.miss_ft = XNT*W*x6 is its
%   XMWEAVE [ft], THE ANSWER; R.miss_ss_ft is the printed steady-state sinusoid
%   on the same grid; R.peak_ft is this sweep's global peak, transient included.
%   R.ss_amp_cf_ft / R.ss_phase_rad are the PRINTED amplitude and phase.  There
%   is deliberately no MEASURED steady-state amplitude in R: TF = 10 s is not
%   always long enough for the transient to be gone, and a field that is
%   sometimes steady-state and sometimes transient is worse than no field --
%   BENCHMARK 2 measures it on a run long enough to mean it.
%   Also R.bench R.selfcheck R.pass R.verdict R.p R.summary R.units.  R.pass is
%   true only if a row was SCORED and all scored rows passed; 'benchmark', false
%   makes it FALSE meaning NOT CHECKED, never "fine".  'assert' picks the
%   REACTION to R.verdict and nothing else: true errors, false WARNS and still
%   returns R complete.  benchSuite runs every port with it off.
%
%   See also MGS.ZARCHAN.SHAPINGFILTERADJOINT (Listing 4.6 -- the same guidance
%   loop, a stochastic shaping filter instead of a resonator),
%   MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 20.2 line 1, verbatim.  Feet, seconds, rad/s.
    opts.XNT (1,1) double {mustBeFinite, mustBePositive} = 193.2  % ft/s^2 = 6 g
    opts.XNP (1,1) double {mustBeFinite, mustBePositive} = 3.0    % N-prime
    opts.TAU (1,1) double {mustBeFinite, mustBePositive} = 1.0    % s
    opts.TF  (1,1) double {mustBeFinite, mustBePositive} = 10.0   % s, sweep end
    opts.W   (1,1) double {mustBeFinite, mustBePositive} = 3.0    % rad/s
    opts.H   (1,1) double {mustBeFinite, mustBePositive} = 0.01   % s, RK2 step
    % Set on the listing's first line and referenced NOWHERE in it.  Carried so
    % that its deadness is on the record; see the transcription note.
    opts.VC  (1,1) double {mustBeFinite, mustBePositive} = 4000.0 % ft/s, UNUSED
    % [] means "every step".  The listing's own literal is 0.1 s; see header.
    opts.samp double {mustBeReal, mustBeFinite} = []
    % Time constants of transient decay demanded before a steady-state window is
    % measured.  18 puts the residual transient at ~1e-4 of the amplitude even
    % for the N' = 5 case, whose transient carries a t^4.
    opts.ss_tau (1,1) double {mustBeFinite, mustBePositive} = 18
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so an "if nargin == 0" here would be a mode that never fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

XNT = opts.XNT;  XNP = opts.XNP;  TAU = opts.TAU;
TF  = opts.TF;   W   = opts.W;    H   = opts.H;

samp = opts.samp;
if isempty(samp), samp = H; end
if ~isscalar(samp) || samp <= 0
    error('mgs:zarchan:weaveAdjoint:samp', ...
        'samp must be a positive scalar or [] (meaning every step); got %s.', ...
        mat2str(samp));
end
if H > samp
    error('mgs:zarchan:weaveAdjoint:step', ...
        ['H = %g s exceeds the sample interval samp = %g s, so the returned ' ...
         'curve would be sampled at the step rate while its own time vector ' ...
         'claims the coarser interval.'], H, samp);
end
if H >= TAU
    error('mgs:zarchan:weaveAdjoint:stiff', ...
        ['H = %g s is not smaller than the guidance time constant TAU = %g s. ' ...
         'RK2 does not blow up here -- it returns a smooth, entirely wrong ' ...
         'curve -- so this is refused rather than warned about.'], H, TAU);
end
if W*H >= 0.5
    error('mgs:zarchan:weaveAdjoint:resonator', ...
        ['W*H = %g.  The second-order shaping network is integrated at fewer ' ...
         'than ~13 steps per weave period, which distorts the amplitude this ' ...
         'listing exists to measure.  Refused rather than warned about.'], W*H);
end
if TF <= 2*H
    error('mgs:zarchan:weaveAdjoint:short', ...
        'TF = %g s is not long enough to take two steps at H = %g s.', TF, H);
end

% =========================================================================
%   LISTING 20.2, at the caller's parameters.  This is what R returns.
% =========================================================================
M = weaveCase(XNP, TAU, W, XNT, TF, H, samp, 0);
if numel(M.t) < 2
    % Refused, not warned about.  An empty curve returned quietly looks like a
    % converged result, and a ONE-sample curve is worse: diff() on it is empty
    % and the structural checks below would then append an empty result to a
    % logical vector -- a "null assignment" error thrown far from the cause.
    error('mgs:zarchan:weaveAdjoint:empty', ...
        ['the sweep recorded %d sample(s) at TF = %g s, samp = %g s.  At ' ...
         'least two are needed for any curve comparison to mean anything.'], ...
        numel(M.t), TF, samp);
end

R = struct();
R.t  = M.t;
R.x2 = M.X(:,1);  R.x3 = M.X(:,2);  R.x4 = M.X(:,3);
R.x5 = M.X(:,4);  R.x6 = M.X(:,5);
R.tgo        = M.tgo;
R.miss_ft    = M.miss;
R.miss_ss_ft = M.cf_curve;
R.peak_ft    = M.peak_all;
R.ss_amp_cf_ft = M.cf_amp;
R.ss_phase_rad = M.cf_phase;
R.units = 'ft, ft/s^2, rad/s, s; g = 32.2 ft/s^2 (nT = 193.2 ft/s^2 = 6 g)';
R.p = struct('XNT',XNT, 'XNP',XNP, 'TAU',TAU, 'TF',TF, 'W',W, 'H',H, ...
             'samp',samp, 'VC_unused',opts.VC, 'ss_tau',opts.ss_tau, ...
             'x_normalised_weave_freq', W*TAU, 'units', R.units);

R.selfcheck = structuralChecks(M, XNP, TAU, W, XNT, H, samp);

if opts.benchmark
    R.bench = benchmarks(XNP, TAU, W, XNT, H, samp, opts.ss_tau);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine" -- that is the shape of green light this repo has been burned
    % by more than once.
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R); end

% ---- adjudication.  The verdict is FORMED either way; 'assert' picks the
%      reaction.  A port that errors unconditionally cannot be diagnosed --
%      the caller never gets R, so nobody can see which side moved.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the book. ' ...
           'R.pass is false meaning NOT CHECKED.'];
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question. ' ...
        'First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf(['%d of %d scored book values are outside tolerance ' ...
        '(first: %s, book %s, ours %s, band %s).'], numel(bad), sum(ev), ...
        R.bench(bad(1)).what, R.bench(bad(1)).book_str, ...
        R.bench(bad(1)).ours_str, R.bench(bad(1)).band);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:weaveAdjoint:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:weaveAdjoint:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [t, X, miss, tgo] = weaveRun(XNP, TAU, W, XNT, TF, H, samp)
%WEAVERUN  Listing 20.2, zarchan.txt:5999-6010.
%
%   EVERY constant this needs is an ARGUMENT.  MATLAB local functions cannot see
%   the main function's workspace; a previous wave in this package read one from
%   the caller and threw on the first line of the first run, which no amount of
%   static checking saw beforehand.
%
%   States are the listing's own X2..X6.  There is no X1: Fig. 20.3's adjoint
%   takes the miss at the resonator output, not at an integrator chain end.

S  = 0.0;
TP = 0.0 + 0.00001;      % NOT 0: the adjoint gain is N'/tgo.
X2 = 0.0;
X3 = 1.0;                % <<< THE IMPULSE.  Unit area, on the miss node.
X4 = 0.0;
X5 = 0.0;
X6 = 0.0;

% Initialised before the loop so the FLAG==1 branch inside cannot read a value
% left over from anything else.
X2D = 0.0;  X3D = 0.0;  X4D = 0.0;  X5D = 0.0;  X6D = 0.0;
TGO = TP + 0.00001;

nmax = ceil(TF/min(samp, H)) + 8;
t   = zeros(nmax,1);
X   = zeros(nmax,5);
tgo = zeros(nmax,1);
n   = 0;

while TP <= (TF - 1e-5)
    X2OLD = X2;  X3OLD = X3;  X4OLD = X4;  X5OLD = X5;  X6OLD = X6;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  Pass 1 evaluates at the current state;
    % pass 2 takes a full Euler step, advances TP by H, and re-evaluates there.
    % TP advances INSIDE this loop, exactly once -- move that line out and the
    % scheme quietly degrades to first order while still running smoothly.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            X2 = X2 + H*X2D;
            X3 = X3 + H*X3D;
            X4 = X4 + H*X4D;
            X5 = X5 + H*X5D;
            X6 = X6 + H*X6D;
            TP = TP + H;
        end
        X2D = X3;
        Y1  = (-X2 + X4)/TAU;     % the single guidance lag
        TGO = TP + 0.00001;       % adjoint time IS time-to-go
        X3D = Y1*XNP/TGO;         % the time-varying PN gain N'/tgo
        X4D = -Y1;
        X5D = X2 - W*W*X6;        % <<< the weave shaping network, half 1
        X6D = X5;                 % <<< and half 2:  x6'' + w^2 x6 = x2
        FLAG = 1;
    end
    % The listing writes "FLAG = 0" here too.  Dropped: the outer loop
    % re-initialises FLAG on its next pass, so the assignment cannot be read.
    % Left as a comment so nobody re-adds it thinking a state machine was lost.

    % Heun corrector: (x_old + x_pred)/2 + (H/2)*f(t+H, x_pred).
    X2 = (X2OLD + X2)/2 + 0.5*H*X2D;
    X3 = (X3OLD + X3)/2 + 0.5*H*X3D;
    X4 = (X4OLD + X4)/2 + 0.5*H*X4D;
    X5 = (X5OLD + X5)/2 + 0.5*H*X5D;
    X6 = (X6OLD + X6)/2 + 0.5*H*X6D;

    S = S + H;
    if S >= samp - 0.5*H
        S = 0.0;
        n = n + 1;
        t(n)   = TP;
        X(n,:) = [X2 X3 X4 X5 X6];
        tgo(n) = TGO;
    end
end

t = t(1:n);  X = X(1:n,:);  tgo = tgo(1:n);
miss = XNT*W*X(:,5);          % the listing's XMWEAVE = XNT*W*X6
end


% =============================================================================
function S = weaveCase(XNP, TAU, W, XNT, TF, H, samp, nCyc)
%WEAVECASE  One run plus the two quantities the chapter publishes about it.
%
%   nCyc = 0  -> transient case: only .peak_all is meaningful.
%   nCyc > 0  -> steady-state case: .ss_amp and .wave_resid are measured over
%                the LAST nCyc whole weave periods, by which time the transient
%                (which decays as exp(-t/TAU) times a polynomial) is gone.
[t, X, miss, tgo] = weaveRun(XNP, TAU, W, XNT, TF, H, samp);

S = struct();
S.XNP = XNP;  S.TAU = TAU;  S.W = W;  S.XNT = XNT;  S.TF = TF;
S.t = t;  S.X = X;  S.miss = miss;  S.tgo = tgo;
S.peak_all = max(abs(miss));

% The chapter's closed form, general in N'.  See the header for the derivation
% from 1 - H(s) = [s/(s+1/T)]^N' (zarchan.txt:6027).  The book's printed forms
% carry an explicit 32.2 because ITS nT is in g; XNT here is already ft/s^2.
x = W*TAU;
S.cf_amp   = XNT * x^(XNP-2) * TAU^2 / (1 + x^2)^(XNP/2);
S.cf_phase = (XNP-2)*pi/2 - XNP*atan(x);
S.cf_curve = S.cf_amp * sin(W*t + S.cf_phase);

Tper = 2*pi/W;
S.ss_ok = false;  S.ss_amp = NaN;  S.wave_resid = NaN;  S.nsel = 0;
if nCyc > 0 && numel(t) > 1 && (t(end) - nCyc*Tper) > 0
    sel = t >= (t(end) - nCyc*Tper) - 0.5*H;
    S.nsel = sum(sel);
    if S.nsel >= 8
        S.ss_amp     = max(abs(miss(sel)));
        S.wave_resid = max(abs(miss(sel) - S.cf_curve(sel)))/max(S.cf_amp, realmin);
        S.ss_ok      = true;
    end
end
end


% =============================================================================
function K = structuralChecks(M, XNP, TAU, W, XNT, H, samp)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.  They are the weaker
%   claim that it agrees with itself -- which is what catches wiring faults.
K = struct();
names = {};  oks = [];  detail = {};

t  = M.t;  x2 = M.X(:,1);  x4 = M.X(:,3);  x5 = M.X(:,4);  x6 = M.X(:,5);

% 1  TGO really is the recorded time plus 1e-5.  The only thing that would
%    notice someone "cleaning up" the divisor in X3D to TP.
K.tgo_resid = max(abs(M.tgo - (t + 1e-5)));
names{end+1}  = 'tgo == t + 1e-5';
oks(end+1)    = K.tgo_resid <= 1e-12;
detail{end+1} = sprintf('max |tgo - (t+1e-5)| = %.3e s', K.tgo_resid);

% 2  x2 against the EXACT solution of the transcribed guidance ODEs at N' = 3
%    (derived in mgs.zarchan.shapingFilterAdjoint's header from the printed
%    step-maneuver miss 0.5 t^2 exp(-t/T), zarchan.txt:1345).  Put the impulse
%    on x2 or x4 instead of x3 and the program still runs, smoothly, at the
%    right order of magnitude, answering a different question.  This is what
%    catches that; the miss rows alone would not.
tolCurve = max(0.005, 1.5*XNP*H/TAU);
if abs(XNP - 3) < 1e-12
    K.x2_resid = maxRel(x2, exp(-t/TAU).*(t - t.^2/(2*TAU)));
    K.x4_resid = maxRel(x4, exp(-t/TAU).*(t.^2/(2*TAU) - t.^3/(6*TAU^2)));
    names{end+1}  = 'x2 == exp(-t/T)(t - t^2/2T)   [N''=3]';
    oks(end+1)    = K.x2_resid <= tolCurve;
    detail{end+1} = sprintf('%.4f%% of peak (band %.3f%%)', 100*K.x2_resid, 100*tolCurve);
    names{end+1}  = 'x4 == exp(-t/T)(t^2/2T - t^3/6T^2)   [N''=3]';
    oks(end+1)    = K.x4_resid <= tolCurve;
    detail{end+1} = sprintf('%.4f%% of peak (band %.3f%%)', 100*K.x4_resid, 100*tolCurve);
else
    K.x2_resid = NaN;  K.x4_resid = NaN;
end

% 3-4  THE RESONATOR, which is the entire difference between this listing and
%      Listing 4.6.  Central differences on the RECORDED states, so these check
%      what was actually written out, not what the integrator believed.
%      Band comes from the SAMPLING: a central difference of a signal whose
%      dominant frequency is W has a relative error of (W*hs)^2/6, and the band
%      is three times that with a floor.
%      hs is MEASURED, not assumed to be samp.  The listing records when an
%      accumulator passes a threshold, so for a samp that is not a whole
%      multiple of H the true spacing is the next multiple up -- dividing by the
%      requested samp there would inject a fixed scale error into d6 and d5 and
%      fail two correct checks.
dts = diff(t);
hs  = median(dts);
uniform = numel(t) >= 5 && hs > 0 && max(abs(dts - hs)) <= 1e-9*hs;
tolFD = max(1e-3, (W*hs)^2/2);
if uniform
    d6 = centralDiff(x6, hs);
    d5 = centralDiff(x5, hs);
    in = 2:(numel(t)-1);
    K.x6d_resid = maxRel(d6(in), x5(in));
    K.x5d_resid = maxRel(d5(in), x2(in) - W*W*x6(in));
else
    % Not a pass.  Inf reports "this could not be measured on a non-uniform
    % grid", which is a different statement from "the wiring is wrong", and the
    % detail string below says which.
    K.x6d_resid = Inf;  K.x5d_resid = Inf;
end
if uniform
    fdNote = sprintf(' at h = %g s', hs);
else
    fdNote = ' -- NOT MEASURABLE: the recorded grid is not uniform';
end
names{end+1}  = 'd(x6)/dt == x5';
oks(end+1)    = K.x6d_resid <= tolFD;
detail{end+1} = sprintf('%.4f%% of peak (band %.3f%%)%s', ...
    100*K.x6d_resid, 100*tolFD, fdNote);
names{end+1}  = 'd(x5)/dt == x2 - W^2 x6   (shaping network)';
oks(end+1)    = K.x5d_resid <= tolFD;
detail{end+1} = sprintf('%.4f%% of peak (band %.3f%%)%s', ...
    100*K.x5d_resid, 100*tolFD, fdNote);

% 5  XNT is a pure OUTPUT SCALING: it appears in XMWEAVE = XNT*W*X6 and nowhere
%    in the loop, so the miss must be exactly linear in it.  Two short runs.  If
%    a future edit puts nT inside the dynamics (an acceleration limit, say, which
%    is exactly what Listing 20.3 does) the miss curve stays plausible and every
%    other check here still passes.
TFs = min(M.TF, 3.0);
[~, ~, m1] = weaveRun(XNP, TAU, W, XNT,   TFs, H, samp);
[~, ~, m3] = weaveRun(XNP, TAU, W, 3*XNT, TFs, H, samp);
K.lin_resid = maxRel(3*m1, m3);
names{end+1}  = 'miss exactly linear in XNT';
oks(end+1)    = K.lin_resid <= 1e-12;
detail{end+1} = sprintf('|3*miss(nT) - miss(3nT)|/peak = %.3e', K.lin_resid);

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


function d = centralDiff(y, h)
%CENTRALDIFF  Interior points only; the ends are set to their neighbours so the
%   vector keeps its length and the caller can index it against t.
d = zeros(size(y));
if numel(y) >= 3
    d(2:end-1) = (y(3:end) - y(1:end-2))/(2*h);
    d(1) = d(2);  d(end) = d(end-1);
end
end


function r = maxRel(a, b)
%MAXREL  Max absolute difference NORMALISED BY THE REFERENCE'S OWN PEAK.  Not
%   pointwise relative: every state here crosses zero, and a pointwise relative
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
function B = benchmarks(XNP, TAU, W, XNT, H, samp, ssTau)
%BENCHMARKS  Computed vs published, with the PROVENANCE of every band.
%
%   *** EVERY BOOK VALUE HERE WAS GREPPED OUT OF zarchan.txt AND THE LINE
%   NUMBER IS IN THE .source FIELD. ***  Nothing was read off a figure.
%
%   Rows 2 and 3 run at the CALLER's parameters; rows 1 and 4-9 PIN the book's
%   own cases so the board is always the book's board.  See the header.
B = emptyBench();
nCyc = 2;

tolInt = @(np, tau) max(0.005, 1.5*np*H/tau);   % 3x the predicted +0.5*N'*H/T
readOff = 0.05;                                  % Fig. 20.8 ordinate read-off
rnd     = @(v) 0.5/abs(v);                       % the book's printed rounding
tfSS    = @(tau, w) ssTau*tau + nCyc*2*pi/w;

% ---- 1.  the global peak over the book's own flight-time sweep -------------
% zarchan.txt:5959 is about Fig. 20.1 (the NONLINEAR Listing 20.1); :5977 says
% the adjoint results are "virtually identical" to it, which is what makes it a
% value for THIS listing.  That transitive step is book-sanctioned but
% unquantified, hence the 5% model allowance in the band -- stated, not hidden.
C1 = weaveCase(3, 1.0, 3.0, 193.2, 10.0, H, samp, 0);
tol1 = readOff + rnd(28) + tolInt(3,1) + 0.05;
B = addVal(B, 'peak miss over tF = 0..10 s   [N''=3, TAU=1, w=3, nT=6g]', ...
    28, C1.peak_all, sprintf('+/-%.1f%%', 100*tol1), tol1, ...
    'p.442 text on Fig. 20.1, with Fig. 20.4 (zarchan.txt:5959, :5977)', 'ft', ...
    ['TRANSIENT peak at tF ~ 1.15 s, NOT the steady-state amplitude (18.3 ft). ' ...
     'Exact value of the transcribed ODEs is 27.04 ft; expect ~-2%.  Band = ' ...
     '5% read-off + 1.8% rounding + 4.5% integrator + 5% linear-vs-nonlinear']);

% ---- 2.  steady-state amplitude vs the printed magnitude, CALLER'S CASE ----
C2 = weaveCase(XNP, TAU, W, XNT, tfSS(TAU,W), H, samp, nCyc);
tol2 = tolInt(XNP, TAU);
if C2.ss_ok
    B = addVal(B, 'steady-state peak miss vs printed magnitude [caller''s case]', ...
        C2.cf_amp, C2.ss_amp, sprintf('+/-%.1f%%', 100*tol2), tol2, ...
        'p.450 Peak Miss formula (zarchan.txt:6077, formula at :6093-6096)', 'ft', ...
        sprintf(['nT*T^2*x^(N''-2)/(1+x^2)^(N''/2) at x = w*T = %.4g.  ' ...
                 'Expect about +%.2f%% (the Heun start-up bias).  This row and ' ...
                 'row 3 are the only ones that survive off-book parameters.'], ...
                 W*TAU, 100*tol2/3));
else
    B = addSkip(B, 'steady-state peak miss vs printed magnitude [caller''s case]', ...
        'nT*T^2*x^(N''-2)/(1+x^2)^(N''/2)', ...
        sprintf(['the steady-state window (last %d weave periods after %g ' ...
            'time constants) held only %d samples'], nCyc, ssTau, C2.nsel), ...
        'p.450 Peak Miss formula (zarchan.txt:6077, formula at :6093-6096)', 'ft');
end

% ---- 3.  the whole steady-state WAVEFORM: amplitude AND phase -------------
% Heun's phase error on the resonator is +th^3/6 per step, th = W*H, so
% TF*W^3*H^2/6 radians accumulated; the band carries three times that.
tolPh = 3*(C2.TF * W^3 * H^2 / 6);
tol3  = tol2 + tolPh;
if C2.ss_ok
    B = addBound(B, 'steady-state miss == A sin(w tF + phase)  [caller''s case]', ...
        sprintf('<= %.2f%% of amplitude', 100*tol3), ...
        sprintf('%.3f%% of amplitude', 100*C2.wave_resid), ...
        C2.wave_resid <= tol3, ...
        'p.448 time-domain solution (zarchan.txt:6056-6059, :6065-6070), Fig. 20.7 (:6072)', ...
        'fraction of amplitude', ...
        sprintf(['phase = (N''-2)pi/2 - N'' atan(wT) = %.4f rad; the book ' ...
            'prints this for N''=3 only.  Scores FREQUENCY too (:5959 -- the ' ...
            'miss oscillates at the TARGET''s frequency).  Band = %.1f%% ' ...
            'amplitude + %.1f%% accumulated Heun phase.'], ...
            C2.cf_phase, 100*tol2, 100*tolPh));
else
    B = addSkip(B, 'steady-state miss == A sin(w tF + phase)  [caller''s case]', ...
        'A sin(w tF + (N''-2)pi/2 - N'' atan(wT))', ...
        'the steady-state window was too short to measure (see row 2)', ...
        'p.448 time-domain solution (zarchan.txt:6056-6059, :6065-6070)', 'fraction');
end

% ---- 4-6.  the chapter's three worked numbers, Fig. 20.8 read-offs ---------
% Each is (ordinate read off Fig. 20.8) x nT[g] x T^2, and the book writes "~".
% Weak by construction; the bands are decomposed in each .detail.
% Columns: w [rad/s], T [s], book's peak miss [ft], the ordinate the book SAYS
% it read off Fig. 20.8, source.  The ordinate is quoted, not back-computed:
% 11.5 x 6 x 0.5^2 is 17.25, which the book prints as "17", so recovering the
% ordinate from the printed answer would report 11.33 and misattribute the
% book's rounding to its eyesight.
ex = { 2.0, 1.0, 33,  5.5, 'p.451 worked example (zarchan.txt:6120-6123)'
       2.0, 0.5, 17, 11.5, 'p.451 worked example (zarchan.txt:6124-6127)'
       4.0, 0.5,  8,  5.5, 'p.451 worked example (zarchan.txt:6128-6131)' };
for k = 1:size(ex,1)
    Wk = ex{k,1};  Tk = ex{k,2};  bk = ex{k,3};  ordk = ex{k,4};
    Ck = weaveCase(3, Tk, Wk, 193.2, tfSS(Tk,Wk), H, samp, nCyc);
    tk = readOff + rnd(bk) + tolInt(3, Tk);
    whatk = sprintf('peak steady-state miss, 6g target, w=%g rad/s, T=%g s', Wk, Tk);
    if Ck.ss_ok
        B = addVal(B, whatk, bk, Ck.ss_amp, sprintf('+/-%.1f%%', 100*tk), tk, ...
            ex{k,5}, 'ft', ...
            sprintf(['the book read %.1f off Fig. 20.8 where its own printed ' ...
                'formula gives %.4f, then wrote %g x 6 x %g^2 ~ %g ft.  The ' ...
                'formula itself gives %.3f ft.  Band = 5%% read-off + %.1f%% ' ...
                'rounding + %.1f%% integrator.'], ...
                ordk, 32.2*(Wk*Tk)/(1+(Wk*Tk)^2)^1.5, ordk, Tk, bk, ...
                Ck.cf_amp, 100*rnd(bk), 100*tolInt(3,Tk)));
    else
        B = addSkip(B, whatk, sprintf('%g ft', bk), ...
            'steady-state window too short', ex{k,5}, 'ft');
    end
end

% ---- 7-8.  the N' = 4 and N' = 5 magnitude formulas ------------------------
% Printed separately at :6104 and :6108.  These rows are why XNP is a real
% parameter of this port and not a constant.
%
% ⚠️ w = 2 rad/s, T = 1 s, NOT the tidier w = 1, T = 1.  Both give x = wT = 1,
% but at w = 1 the factor W in XMWEAVE = XNT*W*X6 is UNITY, so dropping it
% entirely is invisible on these two rows.  Measured: with W deleted from the
% miss, w = 1 leaves rows 7 and 8 GREEN while six other rows go red.  x = 2 is
% off the peak of the normalised curve and scores just as well.
np = {4, 'p.450 N''=4 formula 32.2 x^2/(1+x^2)^2 (zarchan.txt:6104-6107)'
      5, 'p.450 N''=5 formula 32.2 x^3/(1+x^2)^2.5 (zarchan.txt:6108-6111)'};
for k = 1:size(np,1)
    nk = np{k,1};
    Ck = weaveCase(nk, 1.0, 2.0, 193.2, tfSS(1.0,2.0), H, samp, nCyc);
    tk = tolInt(nk, 1.0);
    whatn = sprintf('peak steady-state miss at N''=%d, x = wT = 2', nk);
    if Ck.ss_ok
        B = addVal(B, whatn, Ck.cf_amp, Ck.ss_amp, ...
            sprintf('+/-%.1f%%', 100*tk), tk, np{k,2}, 'ft', ...
            sprintf(['book''s own formula at nT = 6 g, T = 1 s, w = 2 rad/s; ' ...
                'expect about +%.1f%%.'], 100*tk/3));
    else
        B = addSkip(B, whatn, 'printed formula', ...
            'steady-state window too short', np{k,2}, 'ft');
    end
end

% ---- 9.  SHAPE: lower weave frequency => larger miss -----------------------
% zarchan.txt:5978 and the Fig. 20.5 caption at :5992.  "Significantly" is an
% adjective, so what is scored is the ordering, with a 20% margin so a numerical
% tie cannot pass.  This is the row that catches the resonator being driven at
% the wrong frequency, or W dropping out of XMWEAVE = XNT*W*X6.
Ca = weaveCase(3, 1.0, 1.5, 193.2, tfSS(1.0,1.5), H, samp, nCyc);
Cb = weaveCase(3, 1.0, 3.0, 193.2, tfSS(1.0,3.0), H, samp, nCyc);
if Ca.ss_ok && Cb.ss_ok
    ratio = Ca.ss_amp/max(Cb.ss_amp, realmin);
    B = addShape(B, 'w 3 -> 1.5 rad/s increases the miss', ...
        'miss increases significantly', ...
        sprintf('%.4g ft vs %.4g ft (ratio %.2f)', Ca.ss_amp, Cb.ss_amp, ratio), ...
        ratio > 1.2, ...
        'p.443 text and Fig. 20.5 caption (zarchan.txt:5978, :5992)', ...
        ['closed form predicts 49.46 vs 18.33 ft, ratio 2.70.  DELIBERATELY ' ...
         'WEAK: an ordering survives any fault that scales both sides, and ' ...
         'measured, it stayed green under two injected faults that reddened ' ...
         'eight and six other rows.  It is here to catch the resonator being ' ...
         'driven at the wrong frequency, not to certify anything on its own.']);
else
    B = addSkip(B, 'w 3 -> 1.5 rad/s increases the miss', 'increases', ...
        'steady-state window too short at one of the two frequencies', ...
        'p.443 text and Fig. 20.5 caption (zarchan.txt:5978, :5992)', 'ft');
end

% ---- INFO: what the adjoint buys ------------------------------------------
B = addInfo(B, 'cost: one adjoint run vs the brute-force sweep', ...
    '"only one run has to be made"', ...
    'Listing 20.1 sweeps RT1IC = 500:500:40000, i.e. 80 nonlinear runs; this is 1', ...
    'p.442-443 text (zarchan.txt:5943, :5968)', ...
    ['NOT SCORED: the sentence is about program STRUCTURE.  The 80 is counted ' ...
     'from Listing 20.1''s own loop bounds, not measured here.']);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw "Subscripted assignment between dissimilar
%   structures" -- which is the behaviour we want: a struct array that quietly
%   reordered fields would make [B.ok] mean something else.  Schema is the one
%   mgs.zarchan.benchSuite's adaptGeneric consumes, and it is STRICT.
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
%ADDINFO  Reported, never scored.  skipped = true keeps it out of the pass count
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
%ADDSKIP  A check that COULD NOT RUN.  Never a pass: a check that did not happen
%   is a different thing from one that succeeded.
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
L{end+1} = 'Zarchan Listing 20.2 -- ADJOINT, SINGLE TIME CONSTANT GUIDANCE vs WEAVING TARGET';
L{end+1} = '*** UNITS ARE FEET, ft/s^2, rad/s.  g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['   nT = %g ft/s^2 (%.4g g)   N'' = %g   TAU = %g s   ' ...
    'w = %g rad/s   x = wT = %.4g'], p.XNT, p.XNT/32.2, p.XNP, p.TAU, p.W, ...
    p.x_normalised_weave_freq);
L{end+1} = sprintf('   TF = %g s   H = %g s   samp = %g s   (VC = %g ft/s is DEAD in this listing)', ...
    p.TF, p.H, p.samp, p.VC_unused);
L{end+1} = '';
L{end+1} = 'MISS vs FLIGHT TIME (the listing''s XMWEAVE).  It does NOT decay with';
L{end+1} = sprintf(['flight time: the steady-state amplitude is %.4f ft at ' ...
    'phase %.4f rad.'], R.ss_amp_cf_ft, R.ss_phase_rad);
L{end+1} = sprintf('%9s %14s %16s', 'tF [s]', 'miss [ft]', 'steady state [ft]');
stride = max(1, round(0.5/p.samp));
rows = unique([1:stride:numel(R.t), numel(R.t)]);
for r = rows
    L{end+1} = sprintf('%9.3f %14.4f %16.4f', R.t(r), R.miss_ft(r), R.miss_ss_ft(r)); %#ok<AGROW>
end
L{end+1} = sprintf('   global peak over this sweep: %.4f ft', R.peak_ft);
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-44s %s', vstr(K.oks(i)), K.names{i}, K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning NOT CHECKED.';
else
    L{end+1} = 'BENCHMARKS  (rows 1 and 4-9 pin the BOOK''S cases; rows 2-3 use yours)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch b.kind
            case 'value'
                if b.skipped
                    L{end+1} = sprintf('   %-4s %-58s %s', 'SKIP', b.what, b.why); %#ok<AGROW>
                else
                    L{end+1} = sprintf('   %-4s %-58s book %8s  ours %8s  %+7.2f%%  band %s', ...
                        vstr(b.ok), b.what, b.book_str, b.ours_str, b.pct, b.band); %#ok<AGROW>
                end
            case {'bound','shape'}
                L{end+1} = sprintf('   %-4s %-58s %s  vs  %s', ...
                    vstr(b.ok), b.what, b.ours_str, b.book_str); %#ok<AGROW>
            case 'skip'
                L{end+1} = sprintf('   %-4s %-58s %s', 'SKIP', b.what, b.why); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf('   %-4s %-58s %s', 'INFO', b.what, b.ours_str); %#ok<AGROW>
        end
        L{end+1} = sprintf('        %s', b.source); %#ok<AGROW>
    end
    L{end+1} = sprintf('   scored rows: %d', sum(ev));
    if abs(p.XNP - 3) > 1e-12 || abs(p.TAU - 1) > 1e-12 || abs(p.W - 3) > 1e-12
        L{end+1} = '   NOTE: you changed the case.  Rows 1 and 4-9 still ran the BOOK''s';
        L{end+1} = '   parameters, so a green board there says nothing about YOUR run --';
        L{end+1} = '   rows 2 and 3 and the structural invariants are what cover it.';
    end
end
s = strjoin(L, newline);
end

function v = vstr(ok)
if ok, v = 'ok'; else, v = 'FAIL'; end
end

function drawCurves(R)
figure('Name','Zarchan Listing 20.2 -- weaving target adjoint');
plot(R.t, R.miss_ft, 'LineWidth', 1.2); hold on
plot(R.t, R.miss_ss_ft, '--', 'LineWidth', 1.0); grid on
xlabel('Flight Time (s)'); ylabel('Miss (ft)');
legend('adjoint (Listing 20.2)', 'closed-form steady state', 'Location','best');
title(sprintf('nT = %g ft/s^2, N'' = %g, TAU = %g s, w = %g rad/s', ...
    R.p.XNT, R.p.XNP, R.p.TAU, R.p.W));
end
