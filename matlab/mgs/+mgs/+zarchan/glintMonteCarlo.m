function R = glintMonteCarlo(opts)
%MGS.ZARCHAN.GLINTMONTECARLO  Zarchan Listing 30.3 -- Monte Carlo of a
%   three-state Kalman filter and the optimal guidance law in a homing loop
%   driven by UNCORRELATED (white) GLINT NOISE and a uniformly distributed
%   target maneuver.
%
%   R = mgs.zarchan.glintMonteCarlo()
%   R = mgs.zarchan.glintMonteCarlo('runs', 400, 'assert', false)
%   R = mgs.zarchan.glintMonteCarlo('tf', 0.2:0.2:10)     % the book's own sweep
%   R = mgs.zarchan.glintMonteCarlo('experiments', false) % skip Figs 30.16/30.17
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, g = 32.2 ft/s^2.  NOT SI.
%   Phis in (ft/s^2)^2/Hz, Phin in ft^2/Hz.  Listing 30.1 is Chapter 30's
%   metric exception; this is not it.  Convert at the call site and say so.
% ===========================================================================
%
%   THREE ROUTES TO ONE NUMBER, as Fig. 30.15 superimposes them (:11417):
%   the continuous bound sqrt(P11) = sqrt(2*Phis^(1/6)*Phin^(5/6)); the discrete
%   Riccati recursion (Listing 30.2, run inline because 30.3 runs it inline --
%   it supplies the gains the Monte Carlo flies); and the Monte Carlo itself.
%   The bound alone proves nothing; the port is worth having because the other
%   two are produced by machinery that shares no code with it.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_30_03.m does not parse.
%
%     1  OCR "," is "<" and "." is ">" throughout this book: "while T ,= (TF -
%        1e-5)" and "while STEP ,=1" -> "<="; "if S.=(TS - 1e-5)" -> ">=";
%        "if PZ . 0" -> "> 0".  All shape-forced: T and S only increase, STEP
%        is set to 2 in the body to leave, and PZ has had 0.5 subtracted.
%     2  UNRESOLVED VARIABLE-TO-VARIABLE #1: "if T,TSTART / XNTC=0 / else
%        XNTC=COEF*XNTIC" -> "if T < TSTART", i.e. THE MANEUVER STARTS AT
%        TSTART AND STAYS ON.  Because the prose at :11382 says each trial has
%        "a different maneuver starting time", and because Phis = nTMAX^2/tF
%        (:11278) is the spectral density of a constant-amplitude maneuver with
%        a uniformly distributed START time, nT(t) = nT*u(t - TSTART).  The
%        other direction is a different signal with a different Phis, which
%        would invalidate every bound below.
%        NOT MEASURED HERE, and that is a limitation: the optimal estimator
%        plus optimal law absorbs either switch shape, so this port cannot
%        adjudicate its own resolution the way the Ch.4 twin can (in
%        shapingFilterAdjoint the same flip moves sigma by 56%).
%     3  UNRESOLVED VARIABLE-TO-VARIABLE #2: "if XNC.AMAX, XNC=AMAX" -> ">",
%        shape-forced by its partner "if XNC < -AMAX" two lines later: a
%        symmetric limiter.  UNREACHABLE at nominal settings -- AMAXG =
%        99999999 g is how the book writes "the acceleration command is not
%        limited" (:11372) -- so this one is asserted, not tested.
%     4  Prose leaked into the transcription's lines 37-40 (book body text at
%        :11417-11423, including the ligature wreckage of the sqrt(P11)
%        formula).  Dropped, not "fixed" into code.
%     5  "save datfil.txt output /ascii" is the book's recurring typo for
%        -ascii.  Nothing is written to disk here.
%
%   DEAD CODE IN THE LISTING, KEPT.  RTM = VC*TGO and XLAM = Y/(VC*TGO) are
%   computed every pass and never read; XNEW, XNP and YIC are never read either.
%   ==> THE SIMULATION IS INDEPENDENT OF CLOSING VELOCITY.  RTM/XLAM are still
%   evaluated so self-check 2 exercises a real path -- that check cannot fail
%   today and says so.  The book's own loose end, recorded not fixed: P22 is
%   seeded from a 1047 ft/s heading-error uncertainty while the truth model
%   starts at YD = 0 with no heading error injected.
%
%   VECTORISED ACROSS REPLICATIONS, RICCATI HOISTED.  "for JJ=1:RUN" becomes an
%   array index; the covariance recursion does not depend on the measurements,
%   so it runs ONCE per flight time.  Both exact, not approximations.
%   RANDOMNESS: a private RandStream('mt19937ar') seeded from 'seed' +
%   round(1000*tF) -- never the global stream, and the per-flight-time seed
%   makes the TAU and gamma variants PAIRED on identical draws.  Draws are taken
%   in blocks rather than interleaved as in the listing; bit-agreement with the
%   book's 1990s rand is impossible anyway.
%
%   BOOK SELF-CONTRADICTION, recorded so nobody "fixes" it: the bound's exponent
%   is printed .83333 in Listing 30.2 (:11337), .8333 in 30.3 (:11415), 5/6 in
%   the derivation (:11267).  This port uses exact 5/6.  It changes nothing at
%   the nominal case because Phin is exactly 1 there -- which is why such a typo
%   survives printing.
%
% ---------------------------------------------------------------------------
%   THE CH.18 CROSS-CHECK, AND WHY IT IS AN INFO ROW.  noiseMissAdjoint
%   (Listing 18.1) carries sigma = 1.44*T^(-0.5)*PhiGL^0.5 at N' = 3
%   (:5288, where the minus sign is lost in extraction; :5290 settles it --
%   reducing the guidance time constant INCREASES the glint miss).  THE TWO
%   SETUPS DO NOT MATCH, and that is the result:  Ch.18's T exponent is -0.5
%   while the Ch.30 bound contains no T AT ALL (self-check 1 measures that);
%   Ch.18 is PhiGL^0.5 while Ch.30 is Phin^(5/12) on a DISCRETE density
%   Phin = sigma_GL^2*Ts; and Ch.18 is fixed N' with no estimator against
%   Ch.30's optimal N'(tgo/T) plus optimal filter -- a LOWER BOUND the first
%   cannot reach.  No equality between them is scoreable.  Benchmark 6 scores
%   the only half that survives: the Ch.30 exponent, measured out of the
%   discrete recursion, against the published 5/12.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** THIS FILE HAS NEVER BEEN EXECUTED. ***  MATLAB
%   is the caller's.  These come from an INDEPENDENT REPLICA of the same
%   recursions run outside MATLAB -- a prediction that can itself be wrong, not
%   a measurement.
%     Phis(tF=10) = 2592.1 (book truncates to 2592), Phin = 1 exactly
%     closed-form bound       2.7226 ft vs the book's printed 2.72   (+0.10%)
%     discrete Riccati Ts=0.01, tF=10:  2.6729 ft, gap to formula     1.82%
%     discrete Riccati Ts=0.1,  tF=10:  6.2969 ft vs formula 7.1064, 11.39%
%     measured glint exponent 0.41972 vs 5/12 = 0.41667              (+0.73%)
%     Monte Carlo rms [ft] / ratio to sqrt(P11), 400 replications:
%                                tF = 6      tF = 8      tF = 10
%       TAU 0.2, gamma 1e-4  3.220/1.156 3.070/1.128 3.005/1.124   nominal
%       TAU 1.0, gamma 1e-4  5.780/2.076 5.433/1.996 5.076/1.899   Fig 30.16
%       TAU 1.0, gamma 1e-8  3.097/1.112 2.994/1.100 2.923/1.094   Fig 30.17
%
%   ==> THE HONEST HEADLINE: this reproduction lands 12-16% ABOVE the bound in
%   the nominal case, not on top of it.  The book calls that "very close
%   agreement" and prints no number, which is why benchmark 6 scores only the
%   ONE-SIDED claim the book states as fact and the closeness goes on an INFO
%   row.  At the default 100 replications an rms carries ~7-9% sampling error,
%   the same size as the offset, so do not read one run's ratio as a
%   measurement of it.  DO NOT widen a band to make a row green.
%
%   OUTPUTS
%     R.tf .rms .sigma .mean .sp11 .form   the nominal sweep [ft]
%     R.exp        the TAU=1 experiments (Figs 30.16/30.17), [] if switched off
%     R.ric        Ts = 0.01 vs Ts = 0.1 discrete-vs-formula comparison
%     R.bench      benchmark rows (schema shared with every other port)
%     R.selfcheck  .names/.oks/.detail -- structural, NOT book values
%     R.pass       true only if a row was SCORED and everything passed
%     R.verdict    '' on success; otherwise the one-line reason.  ALWAYS formed;
%                  'assert' chooses error-vs-warn and nothing else.
%
%   See also MGS.ZARCHAN.DISCRETERICCATI, MGS.ZARCHAN.OPTIMALGUIDANCEMC,
%   MGS.ZARCHAN.NOISEMISSADJOINT, MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 30.3's own first line, verbatim.  Feet and seconds.
    opts.VC       (1,1) double {mustBeFinite, mustBePositive} = 5*3280   % ft/s
    opts.XNTIC    (1,1) double {mustBeFinite, mustBePositive} = 161      % 5 g
    opts.VM       (1,1) double {mustBeFinite, mustBePositive} = 3000     % ft/s
    opts.HEDEG    (1,1) double {mustBeFinite} = 20                       % deg
    opts.SIGNOISE (1,1) double {mustBeFinite, mustBePositive} = 10       % ft
    opts.TS       (1,1) double {mustBeFinite, mustBePositive} = 0.01     % s
    opts.TAU      (1,1) double {mustBeFinite, mustBePositive} = 0.2      % s
    opts.PZ1      (1,1) double {mustBeFinite, mustBePositive} = 1e-4     % gamma
    opts.AMAXG    (1,1) double {mustBeFinite, mustBePositive} = 99999999 % g
    opts.H        (1,1) double {mustBeFinite, mustBePositive} = 0.001    % s
    opts.runs     (1,1) double {mustBeFinite, mustBePositive} = 100
    % The book sweeps 0.2:0.2:10 (50 flight times x 100 runs = 5000 runs,
    % :11382) = 255 s of simulated flight against 30 s here, i.e. 8.5x the work
    % for no extra benchmark.  Pass 'tf', 0.2:0.2:10 to reproduce Fig. 30.15.
    opts.tf       double {mustBeFinite, mustBePositive} = [2 4 6 8 10]
    opts.seed     (1,1) double {mustBeFinite, mustBeNonnegative} = 20260824
    % The book's two controlled experiments (TAU 0.2 -> 1 s, then gamma
    % 1e-4 -> 1e-8).  Switching them off deletes benchmarks 7 and 8, which are
    % the only rows that test the guidance law's ENDGAME rather than its filter.
    opts.experiments (1,1) logical = true
    opts.benchmark   (1,1) logical = true
    opts.render      (1,1) logical = true
    opts.assert      (1,1) logical = true
end

P = struct('VC',opts.VC, 'XNTIC',opts.XNTIC, 'VM',opts.VM, 'HEDEG',opts.HEDEG, ...
           'SIGNOISE',opts.SIGNOISE, 'TS',opts.TS, 'H',opts.H, ...
           'AMAX',opts.AMAXG*32.2, 'runs',round(opts.runs), 'seed',opts.seed);
tf = sort(opts.tf(:).');

if P.H > P.TS + 1e-12
    error('mgs:zarchan:glintMonteCarlo:step', ...
        ['H = %g s exceeds the filter sample time TS = %g s, so the Kalman ' ...
         'update would fire less often than once per sample while the ' ...
         'recursion still assumes a TS-long propagation.  Refused rather ' ...
         'than warned about: it returns a smooth, wrong curve.'], P.H, P.TS);
end
if abs(P.TS/P.H - round(P.TS/P.H)) > 1e-9
    error('mgs:zarchan:glintMonteCarlo:ratio', ...
        ['TS/H = %.6f is not an integer.  The sample trigger "S >= TS - 1e-5" ' ...
         'would then drift against the Riccati gain index and the two would ' ...
         'come apart a few samples into the run, silently.'], P.TS/P.H);
end
if P.runs < 8
    error('mgs:zarchan:glintMonteCarlo:runs', ...
        ['runs = %d.  A sample this small has no usable standard error, so ' ...
         'the agreement row would pass on anything.'], P.runs);
end

% ---- the nominal sweep -----------------------------------------------------
S0 = mcSweep(P, tf, opts.TAU, opts.PZ1);

% ---- Listing 30.2 alone, Ts = 0.01 vs Ts = 0.1 (Fig. 30.13) ---------------
ric = struct();
ric.tf = 10;
for j = 1:2
    ts = 0.01*10^(j-1);
    ric.ts(j)   = ts;
    ric.sp11(j) = riccatiRun(ric.tf, ts, P.XNTIC^2/ric.tf, P.SIGNOISE, ...
                             P.VM, P.HEDEG, P.XNTIC);
    ric.form(j) = boundFormula(P.XNTIC^2/ric.tf, P.SIGNOISE^2*ts);
end
ric.gap = abs(ric.sp11./ric.form - 1);

% ---- glint exponent, measured out of the discrete recursion ---------------
% Phin = sigma_GL^2*Ts, so scaling sigma_GL by sqrt(2) doubles Phin.
sgA = P.SIGNOISE/sqrt(2);  sgB = P.SIGNOISE*sqrt(2);
spA = riccatiRun(10, P.TS, P.XNTIC^2/10, sgA, P.VM, P.HEDEG, P.XNTIC);
spB = riccatiRun(10, P.TS, P.XNTIC^2/10, sgB, P.VM, P.HEDEG, P.XNTIC);
glintExp = log(spB/spA) / log((sgB^2*P.TS)/(sgA^2*P.TS));

% ---- the book's two controlled experiments (Figs 30.16, 30.17) ------------
EX = [];
if opts.experiments
    tfe = tf(tf >= 6);
    if isempty(tfe), tfe = tf(end); end
    EX = struct();
    EX.tf     = tfe;
    EX.nom    = pickAt(S0, tfe);                       % TAU = 0.2, gamma = 1e-4
    EX.tau1   = mcSweep(P, tfe, 1.0, opts.PZ1);        % Fig. 30.16
    EX.tau1g8 = mcSweep(P, tfe, 1.0, 1e-8);            % Fig. 30.17
end

R = struct();
R.tf = S0.tf;  R.rms = S0.rms;  R.sigma = S0.sigma;  R.mean = S0.mean;
R.sp11 = S0.sp11;  R.form = S0.form;  R.rel_se = S0.rel_se;
R.exp = EX;
R.ric = ric;
R.glint_exp = glintExp;
R.units = 'ft, ft/s, ft/s^2; Phis (ft/s^2)^2/Hz, Phin ft^2/Hz; g = 32.2 ft/s^2';
R.p = struct('VC',P.VC, 'XNTIC',P.XNTIC, 'VM',P.VM, 'HEDEG',P.HEDEG, ...
             'SIGNOISE',P.SIGNOISE, 'TS',P.TS, 'TAU',opts.TAU, ...
             'PZ1',opts.PZ1, 'AMAXG',opts.AMAXG, 'H',P.H, 'runs',P.runs, ...
             'tf',tf, 'seed',P.seed, 'units',R.units);

R.selfcheck = structuralChecks(P, opts.TAU, opts.PZ1, S0);

if opts.benchmark
    R.bench = benchmarks(P, opts, S0, ric, glintExp, EX);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  "Not checked" must never render as "fine".
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false, so nothing was compared against the book.';
elseif ~any(ev)
    msg = 'not one book value could be evaluated.';
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question. ' ...
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
        error('mgs:zarchan:glintMonteCarlo:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN COMPLETE.  benchSuite runs every port with assert
        % off precisely so one red row cannot hide the board.
        warning('mgs:zarchan:glintMonteCarlo:benchmark', ...
            '%s  (''assert'' is false; see R.verdict)', msg);
    end
end
end


% =============================================================================
function S = mcSweep(P, tfList, TAU, PZ1)
%MCSWEEP  Listing 30.3's outer "for TF" loop, one Monte Carlo set per flight
%   time.  Every constant arrives as an argument: MATLAB local functions cannot
%   see the caller's workspace.
n = numel(tfList);
% Built field by field.  struct() would be fine for these NUMERIC vectors --
% only a CELL value makes struct() return a 1-by-n struct array -- but writing
% it out means nobody has to remember which rule applies, and a future edit
% that puts a cell in here cannot silently turn S into an array whose later
% "S.rms(k) = " writes land in different elements.
S = struct();
S.tf = tfList;  S.tau = TAU;  S.pz1 = PZ1;
S.rms = zeros(1,n);  S.sigma = zeros(1,n);  S.mean = zeros(1,n);
S.sp11 = zeros(1,n); S.form  = zeros(1,n);  S.rel_se = zeros(1,n);
for k = 1:n
    TF   = tfList(k);
    PHIS = P.XNTIC*P.XNTIC/TF;              % nTMAX^2/tF, zarchan.txt:11278
    PHIN = P.SIGNOISE*P.SIGNOISE*P.TS;      % sigma_GL^2*Ts, :11281

    [sp11, K1, K2, K3] = riccatiRun(TF, P.TS, PHIS, P.SIGNOISE, P.VM, ...
                                    P.HEDEG, P.XNTIC);
    Z = mcBatch(P, TF, TAU, PZ1, K1, K2, K3);

    % The listing's own estimators: an (n-1) divisor on BOTH the rms and the
    % standard deviation.  Kept -- it inflates the rms by sqrt(n/(n-1)),
    % 0.5% at 100 runs, and is what the book's Fig. 30.15 plots.
    m = numel(Z);
    S.rms(k)   = sqrt(sum(Z.^2)/(m-1));
    S.mean(k)  = sum(Z)/m;
    S.sigma(k) = sqrt(sum((Z - S.mean(k)).^2)/(m-1));
    S.sp11(k)  = sp11;
    S.form(k)  = boundFormula(PHIS, PHIN);

    % Relative standard error of an rms estimate, from the sample's OWN fourth
    % moment: var(m2) = (m4 - m2^2)/m, and d sqrt(m2) = d m2 /(2 sqrt(m2)).
    % Normal theory (1/sqrt(2m)) is not assumed -- COEF = +/-1 makes the miss
    % sample two-lobed and its kurtosis is not 3.
    m2 = sum(Z.^2)/m;  m4 = sum(Z.^4)/m;
    S.rel_se(k) = sqrt(max(m4 - m2^2, 0)/m) / (2*max(m2, realmin));
end
end


% =============================================================================
function Z = mcBatch(P, TF, TAU, PZ1, K1, K2, K3)
%MCBATCH  Listing 30.3's inner "for JJ = 1:RUN" loop, run for every replication
%   at once.  Returns the miss Y at t = TF, one per replication.
nr = P.runs;
H  = P.H;  TS = P.TS;

rs   = RandStream('mt19937ar', 'Seed', mod(P.seed + round(1000*TF), 2^31-1));
u1   = rand(rs, 1, nr);                       % SUM  -> maneuver start time
u2   = rand(rs, 1, nr);                       % PZ   -> sign of the maneuver
nsm  = round(TF/TS) + 2;
W    = randn(rs, nsm, nr);                    % YNOISE/SIGNOISE, one per sample

TSTART = TF*u1;
COEF   = ones(1, nr);
COEF(u2 - 0.5 <= 0) = -1;                     % "PZ=PZ-.5; if PZ > 0, COEF=1"

Y = zeros(1,nr);  YD = zeros(1,nr);  XNL = zeros(1,nr);  XNC = zeros(1,nr);
YH = zeros(1,nr); YDH = zeros(1,nr); XNTH = zeros(1,nr);
T = 0.0;  S = 0.0;  isamp = 0;

while T <= (TF - 1e-5)
    YOLD = Y;  YDOLD = YD;  XNLOLD = XNL;

    % ---- pass 1 (FLAG == 0): derivatives at the current state -------------
    XNT  = COEF .* P.XNTIC .* double(T >= TSTART);   % "if T < TSTART, XNTC=0"
    % RTM and XLAM are computed by the listing on every pass and never read.
    % Kept so that self-check 2 (VC independence) exercises a real code path.
    RTM  = P.VC*(TF - T + 0.00001);
    XLAM = Y ./ RTM;  %#ok<NASGU>

    XNLD = (XNC - XNL)/TAU;
    YDD  = XNT - XNL;

    % ---- predictor.  ORDER MATTERS: Y uses the OLD YD, YD the OLD YDD. ----
    Y   = Y + H*YD;
    YD  = YD + H*YDD;
    XNL = XNL + H*XNLD;
    T   = T + H;

    % ---- pass 2: T has advanced, so XNT is re-evaluated, as in the listing
    XNT  = COEF .* P.XNTIC .* double(T >= TSTART);
    XNLD = (XNC - XNL)/TAU;
    YDD  = XNT - XNL;

    % ---- corrector, in the listing's own algebra --------------------------
    Y   = 0.5*(YOLD   + Y   + H*YD);
    YD  = 0.5*(YDOLD  + YD  + H*YDD);
    XNL = 0.5*(XNLOLD + XNL + H*XNLD);

    S = S + H;
    if S >= (TS - 1e-5)
        S = 0.0;
        isamp = min(isamp + 1, numel(K1));
        TGO = TF - T + 0.000001;      % NOTE: 1e-6 here, 1e-5 above.  As printed.

        % ---- three-state Kalman update.  Gains are hoisted (see header). ---
        YSTAR = Y + P.SIGNOISE*W(isamp,:);
        RES   = YSTAR - YH - TS*YDH - 0.5*TS*TS*(XNTH - XNL);
        YH    = K1(isamp)*RES + YH + TS*YDH + 0.5*TS*TS*(XNTH - XNL);
        YDH   = K2(isamp)*RES + YDH + TS*(XNTH - XNL);
        XNTH  = K3(isamp)*RES + XNTH;

        % ---- optimal guidance law, Chapter 8 (:11374-11380) ---------------
        X    = TGO/TAU;
        ZEM  = YH + YDH*TGO - XNL*TAU*TAU*(exp(-X) + X - 1) + 0.5*XNTH*TGO*TGO;
        TOP  = 6*X*X*(exp(-X) - 1 + X);
        BOT1 = 2*X*X*X + 3 + 6*X - 6*X*X;
        BOT2 = -12*X*exp(-X) - 3*exp(-2*X);
        % PZ1 is the book's gamma guard (:11467): BOT1+BOT2 -> 0.3*X^5 as
        % X -> 0, so without it N' blows up like 10/X at intercept.  Evaluated
        % AS PRINTED, cancellation and all -- the residual noise is ~7e-16
        % absolute and gamma is 1e-8 at its smallest here, so it never bites.
        XNPP = TOP/(PZ1 + BOT1 + BOT2);
        XNC  = XNPP*ZEM/(TGO*TGO);
        XNC  = min(max(XNC, -P.AMAX), P.AMAX);   % "if XNC > AMAX" -- see header
    end
end
Z = Y;
end


% =============================================================================
function [sp11, K1, K2, K3] = riccatiRun(TF, TS, PHIS, SIGNOISE, VM, HEDEG, XNTIC)
%RICCATIRUN  The discrete matrix Riccati recursion, Listing 30.2 (:11333-11336)
%   and the identical block inside 30.3.  Independent of the measurements, so
%   one pass serves every replication.
TS2 = TS*TS;  TS3 = TS2*TS;  TS4 = TS3*TS;  TS5 = TS4*TS;
SIGN2 = SIGNOISE^2;
P11 = SIGN2;  P12 = 0;  P13 = 0;
P22 = (VM*HEDEG/57.3)^2;  P23 = 0;  P33 = XNTIC*XNTIC;

n  = round(TF/TS);
K1 = zeros(1,n);  K2 = zeros(1,n);  K3 = zeros(1,n);
for i = 1:n
    M11 = P11 + TS*P12 + 0.5*TS2*P13 + TS*(P12 + TS*P22 + 0.5*TS2*P23);
    M11 = M11 + 0.5*TS2*(P13 + TS*P23 + 0.5*TS2*P33) + TS5*PHIS/20;
    M12 = P12 + TS*P22 + 0.5*TS2*P23 + TS*(P13 + TS*P23 + 0.5*TS2*P33);
    M12 = M12 + TS4*PHIS/8;
    M13 = P13 + TS*P23 + 0.5*TS2*P33 + PHIS*TS3/6;
    M22 = P22 + TS*P23 + TS*(P23 + TS*P33) + PHIS*TS3/3;
    M23 = P23 + TS*P33 + 0.5*TS2*PHIS;
    M33 = P33 + PHIS*TS;

    k1 = M11/(M11 + SIGN2);  k2 = M12/(M11 + SIGN2);  k3 = M13/(M11 + SIGN2);
    P11 = (1 - k1)*M11;  P12 = (1 - k1)*M12;  P13 = (1 - k1)*M13;
    P22 = -k2*M12 + M22; P23 = -k2*M13 + M23; P33 = -k3*M13 + M33;

    K1(i) = k1;  K2(i) = k2;  K3(i) = k3;
end
sp11 = sqrt(P11);
end


% =============================================================================
function f = boundFormula(PHIS, PHIN)
%BOUNDFORMULA  zarchan.txt:11267-11272, with EXACT exponents.
%   sqrt(P11) = sqrt(2*Phis^(1/6)*Phin^(5/6)).  The listings print .16667 and
%   .8333/.83333; see the header on that contradiction.
f = sqrt(2*(PHIS^(1/6))*(PHIN^(5/6)));
end


% =============================================================================
function S = pickAt(S0, tfWant)
%PICKAT  The nominal sweep restricted to the experiment's flight times.
[~, idx] = ismember(tfWant, S0.tf);
S = S0;
for f = {'tf','rms','sigma','mean','sp11','form','rel_se'}
    S.(f{1}) = S0.(f{1})(idx);
end
end


% =============================================================================
function K = structuralChecks(P, TAU, PZ1, S0)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.
K = struct();
names = {};  oks = [];  detail = {};

% Three tiny sweeps at 16 replications, sharing one flight time.  They are the
% cheapest way to run these as REAL variations through the production path
% rather than as assertions about the source.
Pa = P;  Pa.runs = 16;
Pb = Pa; Pb.VC = 3*P.VC;
zBase = mcSweep(Pa, 2, TAU,     PZ1);
zVC   = mcSweep(Pb, 2, TAU,     PZ1);
zTAU  = mcSweep(Pa, 2, 5*TAU,   1e-8);

% 1  The estimator never sees the airframe: sqrt(P11) must not move when TAU
%    and gamma BOTH change.  This is the fact that makes the Ch.18 cross-check
%    a category error rather than a disagreement, so it is measured through the
%    same call path the sweep uses, not asserted from the argument list.
names{end+1} = 'sqrt(P11) carries no TAU and no gamma';
oks(end+1)   = zBase.sp11 == zTAU.sp11;
detail{end+1} = sprintf('sqrt(P11) = %.10g ft at (TAU %g, g %g) and %.10g at (TAU %g, g 1e-8)', ...
                        zBase.sp11, TAU, PZ1, zTAU.sp11, 5*TAU);

% 2  VC cancels.  RTM and XLAM are computed every pass and never read, so the
%    miss cannot depend on closing velocity.  TODAY THIS CANNOT FAIL -- it is a
%    guard for the future edit that wires XLAM into the command (which is what
%    PN would do), and it is stated that way rather than sold as a measurement.
names{end+1} = 'miss is independent of closing velocity VC';
oks(end+1)   = zBase.rms == zVC.rms;
detail{end+1} = sprintf('rms %.10g vs %.10g ft at VC and 3*VC (guard, not a measurement)', ...
                        zBase.rms, zVC.rms);

% 3  COEF = +/-1 is load-bearing.  Delete it and every run maneuvers the same
%    way; the sample mean stops being zero and the rms stops estimating the
%    quantity the Riccati equations predict.  The mean's own standard error is
%    sigma/sqrt(n), so 4 of them is the band.
z = abs(S0.mean) ./ max(S0.sigma/sqrt(P.runs), realmin);
names{end+1} = 'COEF = +/-1 keeps the sample mean at zero';
oks(end+1)   = all(z <= 4);
detail{end+1} = sprintf('max |mean|/SE = %.2f over %d flight times (band 4)', ...
                        max(z), numel(z));

% 4  The transcribed optimal navigation ratio must tend to 3 as tgo/T -> inf.
%    x = 1e4, not 100: N'(x) approaches 3 like 3*(1 + 2/x), so N'(100) is
%    3.061 and a 1% band there would fail a CORRECT transcription.
n3 = optNav(1e4, 0);
names{end+1} = 'optimal N''(x) -> 3 as tgo/T -> inf';
oks(end+1)   = abs(n3 - 3) < 0.01;
detail{end+1} = sprintf('N''(1e4) = %.6f', n3);

% 5  ...and the gamma guard must collapse it as tgo -> 0, which is the whole
%    mechanism behind Figs 30.16 and 30.17.  Ungoverned, the denominator is
%    0.3*x^5 and N' would be ~10/x = 1e4 here; the threshold discriminates by
%    six orders at gamma = 1e-8 and by ten at 1e-4.
ng = optNav(1e-3, PZ1);
names{end+1} = 'gamma guard collapses N'' as tgo -> 0';
oks(end+1)   = ng < 0.01;
detail{end+1} = sprintf('N''(1e-3) = %.3e at gamma = %g (ungoverned it would be ~1e4)', ...
                        ng, PZ1);

K.names = names;
K.oks   = logical(oks);
K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    i = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{i}, detail{i});
end
end


% =============================================================================
function v = optNav(X, gamma)
%OPTNAV  The listing's own N'(x), evaluated standalone for the structural rows.
TOP  = 6*X*X*(exp(-X) - 1 + X);
BOT1 = 2*X*X*X + 3 + 6*X - 6*X*X;
BOT2 = -12*X*exp(-X) - 3*exp(-2*X);
v = TOP/(gamma + BOT1 + BOT2);
end


% =============================================================================
function B = emptyBench()
B = struct('what',{}, 'kind',{}, 'book',{}, 'book_str',{}, 'ours',{}, ...
           'ours_str',{}, 'pct',{}, 'band',{}, 'tol',{}, 'ok',{}, ...
           'skipped',{}, 'why',{}, 'source',{}, 'units',{}, 'detail',{});
end

function B = addRow(B, what, kind, book, book_str, ours, ours_str, band, tol, ...
                    ok, skipped, why, source, units, detail)
%ADDROW  THE ONLY CONSTRUCTOR.  Field ORDER is the contract benchSuite's
%   adaptGeneric enforces strictly, so it lives in exactly one place.
i = numel(B) + 1;
B(i).what     = what;
B(i).kind     = kind;
B(i).book     = book;
B(i).book_str = book_str;
B(i).ours     = ours;
B(i).ours_str = ours_str;
if isfinite(book) && book ~= 0 && isfinite(ours)
    B(i).pct = 100*(ours/book - 1);
else
    B(i).pct = NaN;
end
B(i).band    = band;
B(i).tol     = tol;
B(i).ok      = logical(ok);
B(i).skipped = logical(skipped);
B(i).why     = why;
B(i).source  = source;
B(i).units   = units;
B(i).detail  = detail;
end


% =============================================================================
function B = benchmarks(P, opts, S0, ric, glintExp, EX)
%BENCHMARKS  Eleven rows: nine scored, two INFO (the closeness of "very close
%   agreement", and the Ch.18 cross-check).  With 'experiments' false two of
%   the nine become SKIP, leaving seven.  Every book value was grepped out of
%   zarchan_decoded.txt and the line is in .source.  Nothing was read off a
%   figure -- extraction turned every figure into an image link, and every
%   figure claim below is scored as an ORDERING taken from the prose that
%   describes it.
B = emptyBench();

% 1  Phis = nTMAX^2/tF = 161^2/10.  The book prints 2592; the arithmetic is
%    2592.1, so the printed value is TRUNCATED, not a different quantity.  The
%    band is the truncation and nothing else -- widening it further would stop
%    it testing anything.
phis10 = P.XNTIC^2/10;
B = addRow(B, 'Phis at tF = 10 s', 'value', 2592, '2592', phis10, ...
    sprintf('%.4f', phis10), '+/-0.5 (book truncates)', 0.5, ...
    abs(phis10-2592) <= 0.5, false, '', ...
    'p.792 eq for Phis; zarchan.txt:11278-11280', '(ft/s^2)^2/Hz', ...
    '161^2/10 = 2592.1; XNTIC = 161 ft/s^2 is the book''s 5-g maneuver');

% 2  Phin = sigma_GL^2*Ts = 10^2*0.01 = 1
phin = P.SIGNOISE^2*P.TS;
B = addRow(B, 'Phin (glint) at Ts = 0.01 s', 'value', 1, '1', phin, ...
    sprintf('%.6f', phin), 'exact', 1e-9, abs(phin-1) <= 1e-9, false, '', ...
    'p.792 eq for PhiN; zarchan.txt:11281', 'ft^2/Hz', '10^2*0.01');

% 3  sqrt(P11) = sqrt(2*Phis^(1/6)*Phin^(5/6)) = 2.72 ft
cf = boundFormula(phis10, phin);
tol3 = 0.005;
B = addRow(B, 'smallest possible rms miss, closed form, tF = 10 s', 'value', ...
    2.72, '2.72 ft', cf, sprintf('%.4f ft', cf), '0.5%', tol3, ...
    abs(cf/2.72 - 1) <= tol3, false, '', ...
    'p.792 worked example; zarchan.txt:11284-11288', 'ft', ...
    'band 0.5% = ~3x the half-ulp of a 3-figure printed value');

% 4  Fig. 30.13, first clause: at Ts = 0.01 the discrete solution and the
%    formula are "virtually identical".  An ADJECTIVE, so it is scored as a
%    shape: the gap must be under 5%.
okA = ric.gap(1) <= 0.05;
B = addRow(B, 'Ts = 0.01: discrete Riccati ~ formula', 'shape', NaN, ...
    '"virtually identical"', ric.gap(1), sprintf('gap %.2f%%', 100*ric.gap(1)), ...
    'gap <= 5%', 0.05, okA, false, '', ...
    'Fig. 30.13 prose; zarchan.txt:11340', '', ...
    sprintf('discrete %.4f ft vs formula %.4f ft', ric.sp11(1), ric.form(1)));

% 5  Same sentence, second and third clauses: at Ts = 0.1 there is MORE of a
%    difference, and performance is WORSE.  Pure ordering -- stronger than any
%    number eyeballed off the figure.
okB = (ric.gap(2) > ric.gap(1)) && (ric.sp11(2) > ric.sp11(1));
B = addRow(B, 'Ts = 0.1: bigger gap AND bigger miss', 'shape', NaN, ...
    'gap(0.1) > gap(0.01) and miss(0.1) > miss(0.01)', NaN, ...
    sprintf('gap %.2f%% > %.2f%%, miss %.3f > %.3f ft', 100*ric.gap(2), ...
            100*ric.gap(1), ric.sp11(2), ric.sp11(1)), 'ordering', NaN, ...
    okB, false, '', 'Fig. 30.13 prose; zarchan.txt:11340', '', ...
    'both clauses of one sentence; either alone would be weaker');

% 6  sqrt(P11) IS A LOWER BOUND ON THE MISS, and the book states it as a fact,
%    not as an adjective: "we can do no better with any other guidance law"
%    (:11421-11423).  So the scored claim is ONE-SIDED -- the Monte Carlo must
%    not come in UNDER the bound by more than sampling error.  It is deliberately
%    not turned into a two-sided percentage band: "very close agreement" and
%    "nearly able to achieve" are adjectives and the book prints no number for
%    them, so how close we actually land is reported as an INFO row below
%    rather than adjudicated against a figure this port would have invented.
%    The upper side is carried by rows 7 and 8, which measure the same distance
%    against the book's OWN declared-broken case.
tail = S0.tf >= 6;
if ~any(tail), tail = false(size(S0.tf));  tail(end) = true; end
rtail  = S0.rms(tail)./S0.sp11(tail);
setail = S0.rel_se(tail);
okC = all(rtail >= 1 - 4*setail);
B = addRow(B, 'Monte Carlo rms does not beat sqrt(P11), tF >= 6 s', 'bound', ...
    NaN, 'rms >= sqrt(P11) ("we can do no better")', NaN, ...
    sprintf('ratios %s', vecStr(rtail)), 'ratio >= 1 - 4 SE', NaN, ...
    okC, false, '', 'zarchan.txt:11421-11423 (also Fig. 30.15 caption :11441)', ...
    'ratio', sprintf('4 SE = %s%% at %d runs; one-sided by construction', ...
                     vecStr(400*setail), P.runs));

% INFO  How close "very close agreement" actually comes out.  NOT adjudicated:
%       the book prints an adjective and a figure, and a figure did not survive
%       extraction.  Put on the board so a reader can judge it, never scored.
B = addRow(B, 'how close the Monte Carlo gets to the bound, tF >= 6 s', 'info', ...
    NaN, '"very close agreement" / "nearly able to achieve"', NaN, ...
    sprintf('rms/sqrt(P11) = %s', vecStr(rtail)), 'n/a', NaN, false, true, ...
    ['an ADJECTIVE with no printed number behind it, so no band this port ' ...
     'invented could be honest.  For scale: an independent replica of these ' ...
     'recursions, run outside MATLAB at 400 replications, puts the nominal ' ...
     'case 12-16% above the bound at tF = 6, 8 and 10 s, against 90-108% ' ...
     'above for the TAU = 1 s case the book itself calls wrong.  The book ' ...
     'runs 100 x 50 = 5000 (:11382) and at 100 replications the rms carries ' ...
     'about 7-9% of sampling error, which is the same size as the offset.'], ...
    'Fig. 30.15 prose; zarchan.txt:11417-11421, :11441', 'ratio', ...
    sprintf('%d runs x %d flight times in the tail', P.runs, sum(tail)));

% 6  The Ch.30 glint exponent, measured out of the discrete recursion, against
%    the published 5/6 on Phin inside a square root, i.e. 5/12 on the rms.
tol8 = 0.03;
B = addRow(B, 'glint exponent d log(rms)/d log(Phin)', 'value', 5/12, ...
    '5/12 = 0.41667', glintExp, sprintf('%.5f', glintExp), '3%', tol8, ...
    abs(glintExp/(5/12) - 1) <= tol8, false, '', ...
    'P11 = 2 Phis^(1/6) Phin^(5/6); zarchan.txt:11267-11272', 'exponent', ...
    ['measured over a factor of 4 in Phin at tF = 10 s, Ts = ' ...
     sprintf('%g s; band 3%% is discretisation, not modelling', P.TS)]);

% 7/8  The book's two controlled experiments, its most informative rows: the
%       only ones that test the guidance law's ENDGAME rather than the filter.
if isempty(EX)
    why = '''experiments'' is false, so the TAU = 1 s sweeps were not run.';
    B = addRow(B, 'Fig. 30.16: TAU 0.2 -> 1 s raises the Monte Carlo rms', ...
        'skip', NaN, 'rms(TAU=1) > rms(TAU=0.2)', NaN, 'SKIP', 'n/a', NaN, ...
        false, true, why, 'zarchan.txt:11424', '', '');
    B = addRow(B, 'Fig. 30.17: gamma 1e-4 -> 1e-8 pulls it back to the Riccati', ...
        'skip', NaN, '|rms/sp11 - 1| shrinks', NaN, 'SKIP', 'n/a', NaN, ...
        false, true, why, 'zarchan.txt:11470', '', '');
else
    ok7 = all(EX.tau1.rms > EX.nom.rms);
    B = addRow(B, 'Fig. 30.16: TAU 0.2 -> 1 s raises the Monte Carlo rms', ...
        'shape', NaN, 'rms(TAU=1) > rms(TAU=0.2) at every tF', NaN, ...
        sprintf('%s vs %s ft', vecStr(EX.tau1.rms), vecStr(EX.nom.rms)), ...
        'ordering', NaN, ok7, false, '', ...
        'Fig. 30.16 prose; zarchan.txt:11424', 'ft', ...
        ['paired: the same seed per flight time, so the two sweeps share ' ...
         'their draws and the difference is the TAU change alone']);

    d4 = abs(EX.tau1.rms  ./EX.tau1.sp11   - 1);
    d8 = abs(EX.tau1g8.rms./EX.tau1g8.sp11 - 1);
    ok8 = all(d8 < d4);
    B = addRow(B, 'Fig. 30.17: gamma 1e-4 -> 1e-8 pulls it back to the Riccati', ...
        'shape', NaN, '|rms/sp11 - 1| shrinks at every tF', NaN, ...
        sprintf('%s vs %s', vecStr(100*d8), vecStr(100*d4)), 'ordering', NaN, ...
        ok8, false, '', 'Fig. 30.17 prose; zarchan.txt:11470', '%', ...
        ['the guard, not TAU, is the cause; sp11 does not move between these ' ...
         'two runs (self-check 1)']);
end

% INFO  The Ch.18 cross-check the brief asked for.  Not scoreable: the setups
%       do not match, and that is the finding.
B = addRow(B, 'cross-check vs noiseMissAdjoint (Listing 18.1) glint scaling', ...
    'info', NaN, 'sigma = 1.44 T^(-0.5) PhiGL^0.5, N'' = 3', NaN, ...
    sprintf('Ch.30: no T at all, Phin^%.5f', glintExp), 'n/a', NaN, ...
    false, true, ...
    ['SETUPS DO NOT MATCH, so no equality is scoreable.  Ch.18 is fixed ' ...
     'N'' = 3 with no estimator and a T exponent of -0.5 (a FASTER guidance ' ...
     'system misses MORE).  Ch.30''s bound contains no T whatsoever and ' ...
     'scales as Phin^(5/12) = 0.41667 on a DISCRETE density Phin = ' ...
     'sigma_GL^2*Ts, not on continuous PhiGL.  Ch.30 is also a LOWER BOUND ' ...
     'that Ch.18''s law cannot reach.  Row 6 scores the half that survives.'], ...
    'zarchan.txt:5288 + :5290 (Ch.18) vs :11267-11272 (Ch.30)', '', ...
    sprintf('TAU = %g s here; self-check 1 shows sqrt(P11) does not move with it', ...
            opts.TAU));
end


% =============================================================================
function s = vecStr(v)
s = ['[' strtrim(sprintf('%.3f ', v)) ']'];
end


% =============================================================================
function s = render(R, ev)
L = {'mgs.zarchan.glintMonteCarlo  --  Zarchan Listing 30.3 (Ch.30, pp.797-800)'};
L{end+1} = sprintf('  TAU %g s  TS %g s  H %g s  gamma %g  runs %d  seed %d', ...
    R.p.TAU, R.p.TS, R.p.H, R.p.PZ1, R.p.runs, R.p.seed);
L{end+1} = ['  units: ' R.units];
L{end+1} = '';
L{end+1} = '  tF [s]     rms MC     sigma      mean    sqrt(P11)    formula';
for i = 1:numel(R.tf)
    L{end+1} = sprintf('  %6.2f  %9.4f  %9.4f  %8.4f  %9.4f  %9.4f', ...
        R.tf(i), R.rms(i), R.sigma(i), R.mean(i), R.sp11(i), R.form(i)); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = 'STRUCTURAL CHECKS  (self-consistency, NOT book values)';
for i = 1:numel(R.selfcheck.names)
    if R.selfcheck.oks(i), tag = 'holds'; else, tag = 'BROKEN'; end
    L{end+1} = sprintf('   %-44s %-7s %s', R.selfcheck.names{i}, tag, ...
        R.selfcheck.detail{i}); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = 'BOOK BENCHMARKS';
for i = 1:numel(R.bench)
    b = R.bench(i);
    if b.skipped
        L{end+1} = sprintf('   %-6s %-52s %s', upper(b.kind), b.what, b.why); %#ok<AGROW>
    else
        if b.ok, tag = 'ok  '; else, tag = 'FAIL'; end
        L{end+1} = sprintf('   %s  %-52s book %-14s ours %-22s band %s', ...
            tag, b.what, b.book_str, b.ours_str, b.band); %#ok<AGROW>
    end
end
L{end+1} = '';
if any(ev)
    L{end+1} = sprintf('  %d of %d rows scored.', sum(ev), numel(R.bench));
else
    L{end+1} = '  NOTHING WAS SCORED.';
end
s = strjoin(L, newline);
end
