function R = weaveKalmanThreeState(opts)
%MGS.ZARCHAN.WEAVEKALMANTHREESTATE  Zarchan Listing 25.1 -- the ORIGINAL
%   three-state linear Kalman filter flown against a WEAVING target.
%
%   R = mgs.zarchan.weaveKalmanThreeState()
%   R = mgs.zarchan.weaveKalmanThreeState('SIGRIN', 1e-4, 'assert', false)
%   R = mgs.zarchan.weaveKalmanThreeState('APN', 2)      % optimal guidance
%
%   Line-by-line port of Listing 25.1 (zarchan.txt:8584-8602).  This is the
%   BASELINE that Listing 25.2 (the four-state weave filter) improves on, so
%   the pair is the point: 25.1 is a filter built for a CONSTANT target
%   maneuver, asked to track a sinusoid.  The book's own verdict, :8620, is
%   that it does so badly at 1 mr and well at 0.1 mr.
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, g = 32.2 ft/s^2.  NOT SI.
%   LOS angle noise is in RADIANS; the book quotes it in mr (1 mr = 0.001).
% ===========================================================================
%
%   WHAT THE CHAPTER ACTUALLY PUBLISHES -- READ THIS BEFORE ADDING ROWS
%
%   Almost nothing.  Chapter 25's treatment of Listing 25.1 is four figures
%   (25.2-25.5) and no printed number except the parameter set.  Every figure
%   became an image link in the extraction, so there is nothing to read off
%   them.  That leaves: Table 25.1's five nominal values (zarchan.txt:8952-
%   8956), the prose parameter restatement (:8582), the "SIGRIN = .0001"
%   variant (:8621) and exactly ONE quantitative claim about a figure --
%   "approximately a half-second lag" (:8621).  This port therefore scores
%   FIVE values and THREE orderings, and says so rather than manufacturing a
%   ninth.  A thin board here is the correct outcome, not a failure.
%
%   THE BOARD, with the line each book value was grepped from:
%     1 value  target maneuver amplitude   3 g       :8955 / :8582
%     2 value  target weave frequency      2 r/s     :8956 / :8582
%     3 value  flight control time const.  .5 s      :8952 / :8948
%     4 value  closing velocity            9000 ft/s :8954 / :8582
%     5 value  estimate lag at 0.1 mr      ~0.5 s    :8621   <- the only
%              quantitative claim the chapter makes about any of its figures
%     6 shape  0.1 mr beats 1 mr on amplitude        :8621
%     7 shape  1 mr estimate improves late in flight :8620
%     8 shape  PHIN = 0 delete experiment            :8620
%     9 info   the 25.2 comparison, which needs both listings   :8842
%    10 skip   rms miss vs flight time -- figures only, nothing printed :8949
%   Rows 3 and 4 are RECOVERED FROM THE RUN (the airframe's own step
%   response; the measurement-noise scale), not echoed back from the input
%   struct: an echo would pass even if the constant never reached the loop.
%
%   THE TRANSCRIPTION, AND THE THREE REPAIRS
%
%   matlab/zarchan/_listings/listing_25_01.m reports NO unresolved variable-
%   to-variable comparisons, and that is right -- every comparison in 25.1 is
%   against a literal.  Three repairs, each forced:
%
%     1  "if S.=(TS-.0001)"  ->  "if S >= (TS-.0001)".  "." is this book's OCR
%        of ">".  S is reset to 0 inside the block, so "<=" would fire on
%        every integration step: the Kalman filter would then run at H, not
%        TS, while its own transition and process-noise terms still carried
%        TS = 0.01.  That is a ten-times-wrong filter that runs smoothly.
%     2  "while STEP ,=1"  ->  "while STEP <= 1".  The body sets STEP = 2 to
%        leave, which only makes sense against "<=".
%     3  "while T,=TF"  ->  "while T <= (TF - 1e-5)".  The OCR of the operator
%        is unambiguous ("<="), but the BOUND is changed here and that is a
%        judgement call, so: with H = 0.001 the accumulated T lands a few ulp
%        BELOW TF after 10000 additions, so the literal takes one further step
%        to T = TF + H, where TGO = TF - T + 1e-5 is NEGATIVE and
%        XNC = N'(yh + ydh*tgo)/tgo^2 is evaluated through a negative tgo.
%        Zarchan's own Listing 9.2 (:2813), which 25.1 is a modification of,
%        writes "while T <= (TF - 1e-5)".  Adopted from the sibling listing.
%
%   Reproduced deliberately: TGO carries +1e-5 in the derivative block and
%   +1e-6 in the sampled block.  The two differ, so YSTAR is not exactly
%   Y + noise but (tgo+1e-6)/(tgo+1e-5)*Y + noise.  That factor is 1-9e-4 at
%   tgo = 0.01 s and 0.1 at the FINAL sample (tgo = 1e-6): the last few
%   samples of YSTAR are not measurements of anything.  Every windowed
%   statistic below therefore stops at tgo >= 0.1 s, and says so.
%   Dropped: XLAMDH (computed, never used), the two figures, and
%   "save datfil.txt output /ascii" (a typo for -ascii; nothing is written).
%   Noted, not changed: HEDEGFIL = 20 enters only as (VM*HEDEGFIL/57.3)^2, so
%   the book's minus-sign OCR damage class cannot reach it.
%
%   RANDOMNESS.  The listing calls randn.  A benchmark that moves when someone
%   else calls rng() is not a benchmark, so the draws come from a Lehmer
%   generator in this file (Box-Muller, the two uniform blocks taken from
%   SEPARATE halves of the stream so the pairs are not consecutive lattice
%   points).  The global stream is never touched.  No run of the book's own
%   listing reproduces another run of it either, so nothing is lost.
%   THE SAME DRAWS ARE USED FOR ALL FOUR RUNS -- the 1 mr / 0.1 mr comparison
%   is then a controlled experiment on the noise SCALE, not two draws.
%
%   FOUR RUNS, AND WHY
%     nom   SIGRIN = 1e-3   the book's nominal case (Figs. 25.2, 25.3)
%     low   SIGRIN = 1e-4   the book's variant at :8621 (Figs. 25.4, 25.5)
%     noq   PHIN = 0        DELETE EXPERIMENT for the prose at :8620, "the
%                           presence of process noise in the filter enables
%                           the Kalman filter to track all types of target
%                           maneuvers".  Scored as BENCHMARK 8.
%     alt   nom, other seed STRUCTURAL: the Riccati sees no measurement, so
%                           the gain sequences must be BIT-IDENTICAL to nom.
%
%   *** WHY rms(estimate error) IS NOT USED AS A QUALITY METRIC. ***  Against
%   a sinusoid, an estimate at FULL amplitude and 59 deg of lag has an rms
%   error of 2*sin(29.5 deg) = 0.98 of the signal's own rms -- i.e. barely
%   better than estimating zero.  The book knows this and talks about LAG.
%   Every ordering row therefore scores the AMPLITUDE and the PHASE of the
%   W rad/s component of the estimate, obtained by projecting onto sin/cos
%   over a WHOLE number of weave periods.  This is not a stylistic
%   preference: an rms version of row 7 was written first and came back RED,
%   late rms above mid rms, while the fitted amplitude climbed 0.91 -> 1.62
%   -> 2.51 g and the lag fell 1.36 -> 1.15 -> 0.94 s across the same flight.
%   rms would have recorded the book as wrong about its own figure.  See the
%   comment on row 7.
%
%   WHAT THIS SHOULD PRINT.  *** NOT A MATLAB RUN. ***  This file has never
%   been executed -- MATLAB is the caller's, not this author's.  The numbers
%   below come from an INDEPENDENT scalar replica of the recurrences in this
%   file (same listing, same Lehmer/Box-Muller stream, same seed 20260824),
%   written to check the benchmark shapes before they were fixed.  Treat them
%   as an expectation to compare a first run against, not as a measurement of
%   this file, and REPORT ANY DISAGREEMENT rather than adjusting a band:
%
%     fitted W-component of XNTH over [2.00 8.28] s (628 samples)
%        1 mr    1.715 g, lag +1.099 s        (truth 3 g)
%        0.1 mr  2.893 g, lag +0.513 s   <- BENCHMARK 5, book "~half-second"
%        PHIN=0  0.297 g, lag +1.534 s   <- BENCHMARK 8, the delete experiment
%     one-period fits at 1 mr, BENCHMARK 7:
%        early [1.00 4.14]  0.909 g, lag +1.358 s
%        late  [6.76 9.90]  2.501 g, lag +0.938 s   (improves, still lags)
%     recovered constants: amplitude 2.999988 g, W 2.000000 rad/s,
%        TAU 0.500017 s, VC 8934 ft/s (-0.74%, band 10%)
%     K1 spans [0.0366, 1.0]; sqrt(P33) falls 2.89 -> 0.58 g over the flight
%     1000 samples, the last at t = 10 s where tgo = 1e-6 and XNC ~ 1.8e12 --
%        that sample is why every window here stops at tgo >= 0.1 s
%
%   A lag outside 0.25-0.75 s at 0.1 mr, or a late window that does not beat
%   the early one, is a real disagreement worth chasing.  Do NOT widen a band.
%
%   OUTPUTS
%     R.nom .low .noq   per-run records, all sampled at TS: .t .tgo .XNT .XNTH
%                       .Y .YSTAR .YH .YDH .XNL .XNC .K1 .K2 .K3 .P11 .P22
%                       .P33.  ** .XNC(end) is evaluated at tgo = 1e-6 and is
%                       ~1e10; it never acts on the airframe because the loop
%                       exits, but do not plot it. **
%     R.fit             fitted amplitude / lag / phase of the W rad/s
%                       component of XNTH: .nom .low .noq over two periods,
%                       plus .early and .late (one period each, 1 mr)
%     R.bench           benchmark rows, schema shared with the other ports
%     R.selfcheck       .names/.oks/.detail structural invariants
%     R.units R.p R.pass R.verdict R.summary
%
%   See also MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 25.1 line 1, verbatim.  Feet, seconds, radians.
    opts.TAU      (1,1) double {mustBeFinite, mustBePositive} = 0.5      % s
    opts.APN      (1,1) double {mustBeMember(opts.APN,[0 1 2])} = 0      % 0=PN
    opts.VC       (1,1) double {mustBeFinite, mustBePositive} = 9000     % ft/s
    opts.XNTREAL  (1,1) double {mustBeFinite, mustBePositive} = 96.6     % ft/s^2
    opts.XNTMAX   (1,1) double {mustBeFinite, mustBePositive} = 96.6     % ft/s^2
    opts.W        (1,1) double {mustBeFinite, mustBePositive} = 2.0      % rad/s
    opts.VM       (1,1) double {mustBeFinite, mustBePositive} = 3000     % ft/s
    opts.HEDEG    (1,1) double {mustBeFinite} = 0.0                      % deg
    opts.HEDEGFIL (1,1) double {mustBeFinite} = 20.0                     % deg
    opts.XNP      (1,1) double {mustBeFinite, mustBePositive} = 3.0
    opts.SIGRIN   (1,1) double {mustBeFinite, mustBePositive} = 0.001    % rad
    % The book's own variant, :8621 "namely, SIGRIN = .0001".
    opts.SIGLOW   (1,1) double {mustBeFinite, mustBePositive} = 0.0001   % rad
    opts.TS       (1,1) double {mustBeFinite, mustBePositive} = 0.01     % s
    opts.TF       (1,1) double {mustBeFinite, mustBePositive} = 10.0     % s
    opts.H        (1,1) double {mustBeFinite, mustBePositive} = 0.001    % s
    opts.seed     (1,1) double {mustBeFinite, mustBeNonnegative} = 20260824
    % NOT nargin: in a function whose arguments block declares only name-value
    % options MATLAB reports nargin = 0 however many were passed, so an
    % "if nargin == 0" mode switch here would never fire.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

TF = opts.TF;  TS = opts.TS;  H = opts.H;  W = opts.W;

if TS <= 2*H
    error('mgs:zarchan:weaveKalmanThreeState:sample', ...
        ['TS = %g s must be more than two integration steps of H = %g s.  ' ...
         'The listing''s own record test is "S >= TS - 1e-4"; below that the ' ...
         'test fires every step and the filter silently runs at H while its ' ...
         'own transition terms still carry TS.'], TS, H);
end
if abs(TS/H - round(TS/H)) > 1e-9
    error('mgs:zarchan:weaveKalmanThreeState:commensurate', ...
        ['TS = %g s is not an integer number of steps of H = %g s, so the ' ...
         'sample instants would drift through the flight.'], TS, H);
end
if TF < 4*pi/W + 2.5
    error('mgs:zarchan:weaveKalmanThreeState:short', ...
        ['TF = %g s leaves no room for the two-period (%.3f s) amplitude/' ...
         'phase window after the initial transient, at W = %g rad/s.'], ...
        TF, 4*pi/W, W);
end

% One noise stream, shared by every run.  See the header: this makes the
% 1 mr / 0.1 mr comparison a controlled experiment on the SCALE.
nDraw = ceil(TF/TS) + 8;
G     = gaussLehmer(opts.seed, nDraw);
Galt  = gaussLehmer(opts.seed + 977, nDraw);

PHIN = opts.XNTMAX*opts.XNTMAX/TF;      % listing: PHIN = XNTMAX*XNTMAX/TF
nom = runEngagement(packParams(opts, opts.SIGRIN, PHIN), G);
low = runEngagement(packParams(opts, opts.SIGLOW, PHIN), G);
noq = runEngagement(packParams(opts, opts.SIGRIN, 0.0),  G);
alt = runEngagement(packParams(opts, opts.SIGRIN, PHIN), Galt);

% ---- windows.  All of them stop at tgo >= 0.1 s; see the header on the
%      1e-5 / 1e-6 TGO mismatch, which makes the last few YSTAR samples
%      something other than measurements.
tSafe  = TF - 0.1;
Pw     = 2*pi/W;                  % one weave period
tFit0  = 2.0;                     % after the initial covariance transient
tFit1  = tFit0 + 2*Pw;            % exactly two weave periods
% ONE period each, as early and as late as the flight allows.  Whole periods
% are what makes the sin/cos projection orthogonal; an arbitrary window leaks
% the sin term into the cos term and manufactures a phase.
tEarly = [1.0, 1.0 + Pw];
tLate  = [tSafe - Pw, tSafe];

fit = struct();
fit.nom   = weaveFit(nom.t, nom.XNTH, W, tFit0, tFit1);
fit.low   = weaveFit(low.t, low.XNTH, W, tFit0, tFit1);
fit.noq   = weaveFit(noq.t, noq.XNTH, W, tFit0, tFit1);
fit.early = weaveFit(nom.t, nom.XNTH, W, tEarly(1), tEarly(2));
fit.late  = weaveFit(nom.t, nom.XNTH, W, tLate(1),  tLate(2));
fit.window = [tFit0 tFit1];

R = struct();
R.nom = nom;  R.low = low;  R.noq = noq;
R.fit = fit;
R.units = 'ft, ft/s, ft/s^2, s, rad/s; g = 32.2 ft/s^2; LOS noise in rad (1 mr = .001)';
R.p = struct('TAU',opts.TAU, 'APN',opts.APN, 'VC',opts.VC, ...
    'XNTREAL',opts.XNTREAL, 'XNTMAX',opts.XNTMAX, 'W',W, 'VM',opts.VM, ...
    'HEDEG',opts.HEDEG, 'HEDEGFIL',opts.HEDEGFIL, 'XNP',opts.XNP, ...
    'SIGRIN',opts.SIGRIN, 'SIGLOW',opts.SIGLOW, 'TS',TS, 'TF',TF, 'H',H, ...
    'PHIN',PHIN, 'seed',opts.seed, 'tsafe',tSafe, 'fit_window',[tFit0 tFit1], ...
    'early_window',tEarly, 'late_window',tLate, 'n_rhs',nom.nrhs, ...
    'units',R.units);

R.selfcheck = structuralChecks(nom, alt, opts, tSafe);

if opts.benchmark
    R.bench = benchmarks(R, opts, tSafe, tEarly, tLate);
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
if opts.plot,   drawCurves(R); end

% ---- adjudication.  The verdict is FORMED either way; 'assert' chooses only
%      the reaction to it.  benchSuite runs every port with assert off, so a
%      failing port must still RETURN, complete, with the evidence attached.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = 'not one book value could be evaluated on this parameter set.';
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
        error('mgs:zarchan:weaveKalmanThreeState:benchmark', '%s\n\n%s', ...
            msg, R.summary);
    else
        warning('mgs:zarchan:weaveKalmanThreeState:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function P = packParams(opts, SIGRIN, PHIN)
%PACKPARAMS  Everything runEngagement needs, as ONE argument.  MATLAB local
%   functions cannot see the caller's workspace; this package has been bitten
%   by a local reading a constant that was only defined in the main function.
P = struct('TAU',opts.TAU, 'APN',opts.APN, 'VC',opts.VC, ...
    'XNTREAL',opts.XNTREAL, 'XNTMAX',opts.XNTMAX, 'W',opts.W, 'VM',opts.VM, ...
    'HEDEG',opts.HEDEG, 'HEDEGFIL',opts.HEDEGFIL, 'XNP',opts.XNP, ...
    'SIGRIN',SIGRIN, 'TS',opts.TS, 'TF',opts.TF, 'H',opts.H, 'PHIN',PHIN);
end


% =============================================================================
function O = runEngagement(P, G)
%RUNENGAGEMENT  Listing 25.1, zarchan.txt:8584-8602, statement for statement.
%
%   G is the pre-drawn unit-normal stream, one draw per Kalman update.  It is
%   an ARGUMENT so that two runs can be given identical noise.

TS = P.TS;  TS2 = TS*TS;  TS3 = TS2*TS;  TS4 = TS3*TS;  TS5 = TS4*TS;
H  = P.H;   TF = P.TF;    VC = P.VC;     TAU = P.TAU;
XNP = P.XNP;  W = P.W;    XNTREAL = P.XNTREAL;
SIGRIN = P.SIGRIN;  PHIN = P.PHIN;  APN = P.APN;

RTM = VC*TF;  SIGNOISE = SIGRIN;  SIGPOS = RTM*SIGNOISE;  SIGN2 = SIGPOS^2;
P11 = SIGN2;  P12 = 0.;  P13 = 0.;
P22 = (P.VM*P.HEDEGFIL/57.3)^2;  P23 = 0.;  P33 = P.XNTMAX*P.XNTMAX;

Y  = 0.;                          % listing: YIC = 0
YD = -P.VM*P.HEDEG/57.3;
T  = 0.;  S = 0.;
YH = 0.;  YDH = 0.;  XNTH = 0.;  XNC = 0.;  XNL = 0.;
% Initialised before the loop so the FLAG==1 branch cannot read a stale value.
YDD = 0.;  XNLD = 0.;  XLAM = 0.;  XNT = 0.;  YSTAR = 0.;  %#ok<NASGU>
K1 = 0.;  K2 = 0.;  K3 = 0.;                               %#ok<NASGU>

nmax = ceil(TF/TS) + 8;
z = zeros(nmax,1);
O = struct('t',z, 'tgo',z, 'XNT',z, 'XNTH',z, 'Y',z, 'YSTAR',z, 'YH',z, ...
           'YDH',z, 'XNL',z, 'XNC',z, 'K1',z, 'K2',z, 'K3',z, ...
           'P11',z, 'P22',z, 'P33',z, 'n',0, 'nrhs',0);
n = 0;  nrhs = 0;  kdraw = 0;

% See the header, repair 3: the literal bound is TF, changed to TF - 1e-5.
while T <= (TF - 1e-5)
    YOLD = Y;  YDOLD = YD;  XNLOLD = XNL;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor.  T advances INSIDE this loop, exactly
    % once, which is what makes the second evaluation happen at t+H.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            Y   = Y + H*YD;
            YD  = YD + H*YDD;
            XNL = XNL + H*XNLD;
            T   = T + H;
        end
        XNT   = XNTREAL*sin(W*T);          % the weaving target
        TGO   = TF - T + .00001;
        % The listing's own line.  It feeds XLAMD, which 25.1 computes and
        % never uses (the guidance reads YH/YDH, not the raw LOS rate), so
        % XLAMD is dropped and RTM is dead HERE -- as it is in the book.
        RTM   = VC*TGO;                    %#ok<NASGU>
        XLAM  = Y/(VC*TGO);
        XNLD  = (XNC - XNL)/TAU;           % single-time-constant airframe
        YDD   = XNT - XNL;                 % ACHIEVED accel enters, not XNC
        FLAG  = 1;
        nrhs  = nrhs + 1;
    end
    % The listing also writes FLAG = 0 here; the outer loop re-initialises it,
    % so the assignment cannot be read.  Kept as a comment, not as code.

    Y   = .5*(YOLD + Y + H*YD);
    YD  = .5*(YDOLD + YD + H*YDD);
    XNL = .5*(XNLOLD + XNL + H*XNLD);

    S = S + H;
    if S >= (TS - .0001)               % repair 1: ">=", not "<="
        S = 0.;
        TGO = TF - T + .000001;        % NOTE: 1e-6 here, 1e-5 above.  Kept.
        RTM = VC*TGO;
        SIGNOISE = SIGRIN;  SIGPOS = RTM*SIGNOISE;  SIGN2 = SIGPOS^2;

        % ---- Riccati.  Depends on TGO and PHIN ONLY -- never on a
        %      measurement.  Structural check 1 is exactly this claim.
        M11 = P11 + TS*P12 + .5*TS2*P13 + TS*(P12 + TS*P22 + .5*TS2*P23);
        M11 = M11 + .5*TS2*(P13 + TS*P23 + .5*TS2*P33) + TS5*PHIN/20.;
        M12 = P12 + TS*P22 + .5*TS2*P23 + TS*(P13 + TS*P23 + .5*TS2*P33) ...
              + TS4*PHIN/8.;
        M13 = P13 + TS*P23 + .5*TS2*P33 + PHIN*TS3/6.;
        M22 = P22 + TS*P23 + TS*(P23 + TS*P33) + PHIN*TS3/3.;
        M23 = P23 + TS*P33 + .5*TS2*PHIN;
        M33 = P33 + PHIN*TS;
        K1 = M11/(M11 + SIGN2);
        K2 = M12/(M11 + SIGN2);
        K3 = M13/(M11 + SIGN2);
        P11 = (1.-K1)*M11;  P12 = (1.-K1)*M12;  P13 = (1.-K1)*M13;
        P22 = -K2*M12 + M22;
        P23 = -K2*M13 + M23;
        P33 = -K3*M13 + M33;

        % ---- measurement and update.  ORDER MATTERS: RES and the YH/YDH
        %      updates all read the OLD XNTH; XNTH is updated last.
        kdraw = kdraw + 1;
        XLAMNOISE = SIGNOISE*G(kdraw);
        YSTAR = RTM*(XLAM + XLAMNOISE);
        RES  = YSTAR - YH - TS*YDH - .5*TS*TS*(XNTH - XNL);
        YH   = K1*RES + YH + TS*YDH + .5*TS*TS*(XNTH - XNL);
        YDH  = K2*RES + YDH + TS*(XNTH - XNL);
        XNTH = K3*RES + XNTH;

        % ---- guidance.  XLAMDH is computed in the listing and never used;
        %      dropped rather than carried as dead code.
        if APN == 0
            XNC = XNP*(YH + YDH*TGO)/(TGO*TGO);
        elseif APN == 1
            XNC = XNP*(YH + YDH*TGO + .5*XNTH*TGO*TGO)/(TGO*TGO);
        else
            XS = TGO/TAU;
            TOP  = 6.*XS*XS*(exp(-XS) - 1. + XS);
            BOT1 = 2*XS*XS*XS + 3. + 6.*XS - 6.*XS*XS;
            BOT2 = -12.*XS*exp(-XS) - 3.*exp(-2.*XS);
            XNPP = TOP/(.0001 + BOT1 + BOT2);
            C1 = XNPP/(TGO*TGO);  C2 = XNPP/TGO;  C3 = .5*XNPP;
            C4 = -XNPP*(exp(-XS) + XS - 1.)/(XS*XS);
            XNC = C1*YH + C2*YDH + C3*XNTH + C4*XNL;
        end

        n = n + 1;
        O.t(n) = T;      O.tgo(n) = TGO;   O.XNT(n) = XNT;   O.XNTH(n) = XNTH;
        O.Y(n) = Y;      O.YSTAR(n) = YSTAR;
        O.YH(n) = YH;    O.YDH(n) = YDH;   O.XNL(n) = XNL;   O.XNC(n) = XNC;
        O.K1(n) = K1;    O.K2(n) = K2;     O.K3(n) = K3;
        O.P11(n) = P11;  O.P22(n) = P22;   O.P33(n) = P33;
    end
end

f = {'t','tgo','XNT','XNTH','Y','YSTAR','YH','YDH','XNL','XNC', ...
     'K1','K2','K3','P11','P22','P33'};
for i = 1:numel(f)
    O.(f{i}) = O.(f{i})(1:n);
end
O.n = n;  O.nrhs = nrhs;
end


% =============================================================================
function F = weaveFit(t, x, W, t0, t1)
%WEAVEFIT  Amplitude, phase and LAG of the W rad/s component of x, by
%   projection onto sin(Wt) and cos(Wt) over [t0, t1].
%
%   The truth is XNTREAL*sin(W t) with zero phase, so a fitted
%   x ~ A*sin(W t + phi) has phi < 0 and a time lag of -phi/W.
%
%   The window is TWO WHOLE PERIODS, which is what makes the projection
%   orthogonal; a window of arbitrary length leaks the sin term into the cos
%   term and manufactures a phase.  It is NOT wrapped: a POSITIVE phase comes
%   back as a NEGATIVE lag, and that is a signal, not something to fold away.
m = t >= t0 & t <= t1;
F = struct('amp',NaN, 'phase',NaN, 'lag',NaN, 'n',sum(m), ...
           't0',t0, 't1',t1, 'ok',false);
if F.n < 8
    return
end
tt = t(m);  xx = x(m);
cs = 2*mean(xx .* sin(W*tt));      % ~ A cos(phi)
cc = 2*mean(xx .* cos(W*tt));      % ~ A sin(phi)
F.amp   = hypot(cs, cc);
F.phase = atan2(cc, cs);
F.lag   = -F.phase/W;
F.ok    = true;
end


% =============================================================================
function g = gaussLehmer(seed, n)
%GAUSSLEHMER  Unit normals from MINSTD (x <- 16807 x mod 2^31-1) plus
%   Box-Muller.  Base MATLAB, deterministic, and it does not touch the global
%   random stream -- inside a suite that runs many ports back to back, setting
%   rng() would silently reach into the caller's session.
%
%   16807*x < 3.61e13 < 2^53, so every product is EXACT in double precision.
%   x = 0 is absorbing, so the seed is mapped into 1..m-1.
%
%   The two uniforms of each Box-Muller pair are taken from SEPARATE HALVES of
%   the stream.  Consecutive MINSTD values lie on a coarse 2-D lattice, and
%   Box-Muller on consecutive pairs turns that lattice into a visible spiral.
m = 2147483647;
x = mod(floor(abs(seed)), m - 1) + 1;
np = 2*ceil(n/2);
u = zeros(1, np);
for k = 1:np
    x = mod(16807*x, m);
    u(k) = x/m;                    % in (0,1); never 0, so log() is safe
end
half = np/2;
u1 = u(1:half);
u2 = u(half + (1:half));
r  = sqrt(-2*log(u1));
g  = zeros(1, np);
g(1:2:end) = r .* cos(2*pi*u2);
g(2:2:end) = r .* sin(2*pi*u2);
g = g(1:n);
end


% =============================================================================
function K = structuralChecks(nom, alt, opts, tSafe)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.  They are the weaker
%   claim that it agrees with ITSELF, which is what catches wiring faults.
%
%   .names and .oks are BOTH required and must be the same length: benchSuite
%   only puts a structural row on the board when it finds both, and a port
%   that computes twenty invariants under private field names puts ZERO rows
%   up while looking green.
K = struct();
names = {};  oks = [];  detail = {};

% 1  DELETE/PERTURB EXPERIMENT.  The Riccati depends on TGO and PHIN only, so
%    changing the NOISE REALIZATION must not move a single gain, bit for bit.
%    This is the one check that notices a measurement leaking into the
%    covariance recursion -- which would still produce a smooth, plausible,
%    wrong filter.
nk = min(numel(nom.K1), numel(alt.K1));
dK = max([max(abs(nom.K1(1:nk) - alt.K1(1:nk))), ...
          max(abs(nom.K2(1:nk) - alt.K2(1:nk))), ...
          max(abs(nom.K3(1:nk) - alt.K3(1:nk)))]);
names{end+1} = 'Riccati gains independent of the noise draw (bit-exact)';
oks(end+1)   = (nk == numel(nom.K1)) && (dK == 0);
detail{end+1} = sprintf('max |dK| = %.3e over %d updates, second seed %g', ...
    dK, nk, opts.seed + 977);

% 2  Covariance diagonals cannot go negative.  A sign slip in
%    P22 = -K2*M12 + M22 (or P33) lands here and nowhere else cheap.
mnP = min([min(nom.P11), min(nom.P22), min(nom.P33)]);
names{end+1} = 'covariance diagonals P11, P22, P33 stay >= 0';
oks(end+1)   = mnP >= 0;
detail{end+1} = sprintf('min diagonal = %.6g ft^2 / (ft/s)^2 / (ft/s^2)^2', mnP);

% 3  K1 = M11/(M11+SIGN2) with H = [1 0 0] and both terms non-negative, so
%    the position gain is mathematically confined to (0, 1].  A value outside
%    means M11 went negative or SIGN2 did.
kmin = min(nom.K1);  kmax = max(nom.K1);
names{end+1} = 'position gain K1 in (0, 1] at every update';
oks(end+1)   = kmin > 0 && kmax <= 1;
detail{end+1} = sprintf('K1 in [%.6f, %.6f]', kmin, kmax);

% 4  The target model is the listing's, exactly: XNT = XNTREAL*sin(W*t) at the
%    RECORDED instants.  Catches a mis-transcribed amplitude or frequency, and
%    catches XNT being evaluated at the wrong pass of the predictor.
dT = max(abs(nom.XNT - opts.XNTREAL*sin(opts.W*nom.t)));
names{end+1} = 'XNT == XNTREAL*sin(W*t) at every sample';
oks(end+1)   = dT <= 1e-9*opts.XNTREAL;
detail{end+1} = sprintf('max |XNT - %g sin(%g t)| = %.3e ft/s^2', ...
    opts.XNTREAL, opts.W, dT);

% 5  The measurement noise is really VC*tgo*SIGRIN on relative position.
%    Normalise the residual by its own predicted sigma and the sample sd must
%    be 1.  Catches a dropped VC, a dropped tgo, or SIGNOISE used where
%    SIGPOS belongs -- each of which leaves a perfectly smooth run.
%    Restricted to tgo >= 0.1 s: see the header on the 1e-5/1e-6 TGO mismatch.
sel = nom.tgo >= 0.1;
zres = (nom.YSTAR(sel) - nom.Y(sel)) ./ ...
       (opts.VC * nom.tgo(sel) * opts.SIGRIN);
sdz = sqrt(mean(zres.^2));
names{end+1} = 'measurement noise scale == VC*tgo*SIGRIN';
oks(end+1)   = abs(sdz - 1) <= 0.20;
detail{end+1} = sprintf(['rms of normalised residual = %.4f (want 1 +/- 0.20; ' ...
    'n = %d, sampling error ~%.1f%%)'], sdz, sum(sel), ...
    100/sqrt(2*max(sum(sel),1)));

K.dK_gain    = dK;
K.min_P_diag = mnP;
K.K1_range   = [kmin kmax];
K.XNT_resid  = dT;
K.noise_sd   = sdz;
K.tsafe      = tSafe;

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
function B = benchmarks(R, opts, tSafe, tEarly, tLate)
%BENCHMARKS  Every book value below was grepped out of
%   matlab/zarchan/_source/zarchan_decoded.txt and its line number is in the
%   row's .source field.  NOTHING here was read off a figure: the extraction
%   turned every figure in this book into an image link, so a "figure
%   reading" in this package is by construction a fabrication.
%
%   Chapter 25 publishes almost no number for Listing 25.1.  Five values and
%   three orderings is the entire harvest; the two unscored rows record what
%   was left on the table and why.
%
%   Constants arrive as ARGUMENTS -- a MATLAB local function cannot see the
%   main function's workspace.

nom = R.nom;  fit = R.fit;
G   = 32.2;
rows = {};

% ---------------------------------------------------------------- 1  value
% Measured back out of the RECORDED XNT, not echoed from the input.  A TS
% grid that straddles the sinusoid peak can only lose (W*TS/2)^2/2 = 5e-5 of
% it, so 0.5% is two orders loose on discretization and still fails hard on a
% mis-transcribed 96.6 (the next plausible typo, 9.66 or 966, is 10x out).
ampG = max(nom.XNT)/G;
tol1 = 0.005;
rows{end+1} = mkRow('target maneuver amplitude', 'value', ...
    3.0, '3 g', ampG, sprintf('%.4f g', ampG), 100*(ampG - 3)/3, ...
    '0.5%', tol1, abs(ampG - 3)/3 <= tol1, false, ...
    'max(XNT)/32.2 over the recorded samples; band is discretization x200', ...
    ['Table 25.1 "Target acceleration level 3 g" zarchan.txt:8955; prose ' ...
     '"3-g weaving target" zarchan.txt:8582'], ...
    'g (= 32.2 ft/s^2)', ...
    sprintf('XNTREAL = %g ft/s^2 = %.4f g', opts.XNTREAL, opts.XNTREAL/G));

% ---------------------------------------------------------------- 2  value
% From the upward zero crossings of the recorded XNT, linearly interpolated.
% This is what notices W being read as a HERTZ, or the sin() argument losing
% its W -- both of which leave a perfectly smooth run.
wMeas = measWeaveFreq(nom.t, nom.XNT);
tol2  = 0.005;
ok2   = isfinite(wMeas) && abs(wMeas - 2)/2 <= tol2;
rows{end+1} = mkRow('target weave frequency', 'value', ...
    2.0, '2 r/s', wMeas, sprintf('%.5f rad/s', wMeas), ...
    100*(wMeas - 2)/2, '0.5%', tol2, ok2, false, ...
    'mean full period between interpolated upward zero crossings of XNT', ...
    ['Table 25.1 "Target weave frequency 2 r/s" zarchan.txt:8956; prose ' ...
     '"weave frequency of 2 rad/s" zarchan.txt:8582'], ...
    'rad/s', ...
    sprintf('W input = %g rad/s, period %.4f s', opts.W, 2*pi/opts.W));

% ---------------------------------------------------------------- 3  value
% Recovered from the AIRFRAME'S OWN RESPONSE, not from the input: XNC is held
% constant across each TS interval, so (XNL(k+1)-XNL(k))/TS = (XNC(k) - the
% interval mean of XNL)/TAU exactly, and a least-squares slope over the
% flight returns TAU.  The midpoint stands in for that mean, an
% O((TS/TAU)^2/12) = 3e-5 relative error, so 2% is loose by nearly three
% orders.  The endgame (tgo < 1 s) is excluded because XNC ~ 1/tgo^2 there
% and the very last sample is evaluated at tgo = 1e-6.
[tauMeas, nTau] = measTau(nom.t, nom.XNC, nom.XNL, nom.tgo, 1.0);
tol3 = 0.02;
ok3  = isfinite(tauMeas) && abs(tauMeas - 0.5)/0.5 <= tol3;
rows{end+1} = mkRow('flight control system time constant', 'value', ...
    0.5, '.5 s', tauMeas, sprintf('%.5f s', tauMeas), ...
    100*(tauMeas - 0.5)/0.5, '2%', tol3, ok3, false, ...
    sprintf(['least-squares slope of dXNL/dt against (XNC - XNL) over %d ' ...
             'samples with tgo >= 1 s'], nTau), ...
    ['Table 25.1 "Autopilot time constant .5 s" zarchan.txt:8952; prose ' ...
     '"The flight control system time constant is set at .5 s" ' ...
     'zarchan.txt:8948'], ...
    's', ...
    sprintf('TAU input = %g s; single-lag airframe XNLD = (XNC-XNL)/TAU', ...
            opts.TAU));

% ---------------------------------------------------------------- 4  value
% Recovered from the MEASUREMENT NOISE SCALE.  YSTAR - Y is VC*tgo*SIGRIN
% times a unit normal, so the rms of (YSTAR-Y)/(tgo*SIGRIN) estimates VC.
% This is a SCALE row, not a precision row: it catches a dropped VC, a
% dropped tgo, or radians confused with milliradians -- each a factor of 1e3
% or more -- and its band is set by its own sampling error, not by whatever
% would make it green.
[vcMeas, nVc] = measVc(nom.Y, nom.YSTAR, nom.tgo, opts.SIGRIN, 0.1);
se4  = 1/sqrt(2*max(nVc,1));            % relative se of an rms of n normals
tol4 = max(0.10, 4*se4);
ok4  = isfinite(vcMeas) && abs(vcMeas - opts.VC)/opts.VC <= tol4;
rows{end+1} = mkRow('closing velocity', 'value', ...
    9000, '9000 ft/s', vcMeas, sprintf('%.1f ft/s', vcMeas), ...
    100*(vcMeas - 9000)/9000, sprintf('%.1f%%', 100*tol4), tol4, ok4, false, ...
    sprintf(['4 standard errors of an rms over n = %d samples (1 se = ' ...
             '1/sqrt(2n) = %.2f%%), floored at 10%%'], nVc, 100*se4), ...
    ['Table 25.1 "Closing velocity 9000 ft/s" zarchan.txt:8954; prose ' ...
     '"the closing velocity has been increased to 9000 ft/s" ' ...
     'zarchan.txt:8582'], ...
    'ft/s', ...
    sprintf('rms((YSTAR-Y)/(tgo*SIGRIN)), SIGRIN = %g rad, tgo >= 0.1 s', ...
            opts.SIGRIN));

% ---------------------------------------------------------------- 5  value
% THE ONLY QUANTITATIVE CLAIM THE CHAPTER MAKES ABOUT ANY OF THESE FIGURES.
% zarchan.txt:8621 -- "The estimate of the target maneuver is nearly perfect,
% except there is approximately a half-second lag between the actual maneuver
% and the estimate."  Scored at the book's own 0.1 mr variant.
%
% Band 0.25 s, i.e. +/-50%, because "approximately a half-second" is one
% significant figure at best.  It is NOT a band chosen to pass: the two
% failure modes it exists to catch are a filter with no lag at all (0 s, out)
% and one lagging a quarter weave period (pi/2W = 0.785 s, out).  A tighter
% band would be reading precision into an adverb; a wider one would admit
% both faults.
lagLow = fit.low.lag;
tol5   = 0.25;
ok5    = isfinite(lagLow) && abs(lagLow - 0.5) <= tol5;
rows{end+1} = mkRow('estimate lag behind the maneuver, 0.1 mr', 'value', ...
    0.5, 'approximately a half-second', lagLow, sprintf('%.3f s', lagLow), ...
    100*(lagLow - 0.5)/0.5, '+/-0.25 s', tol5, ok5, false, ...
    ['phase of the W rad/s component of XNTH, projected on sin/cos over ' ...
     'exactly two weave periods; lag = -phase/W, NOT wrapped'], ...
    'prose about Fig. 25.5, zarchan.txt:8621', ...
    's', ...
    sprintf('fit window [%.2f %.2f] s, %d samples, fitted amplitude %.2f g', ...
            fit.low.t0, fit.low.t1, fit.low.n, fit.low.amp/G));

% ---------------------------------------------------------------- 6  shape
% zarchan.txt:8621 -- "As a consequence of the reduced measurement noise we
% can see from Fig. 25.5 that the filter's estimate of the target maneuver
% now better approximates the sinusoidal nature of the maneuver."  An
% ORDERING, which is all the sentence supports: no threshold is invented.
eA_nom = abs(fit.nom.amp - opts.XNTREAL);
eA_low = abs(fit.low.amp - opts.XNTREAL);
ok6    = isfinite(eA_nom) && isfinite(eA_low) && eA_low < eA_nom;
rows{end+1} = mkRow('0.1 mr tracks the weave amplitude better than 1 mr', ...
    'shape', NaN, 'better (ordering; no number published)', NaN, ...
    sprintf('|dA| %.2f g at 0.1 mr < %.2f g at 1 mr', eA_low/G, eA_nom/G), ...
    NaN, 'strict ordering', NaN, ok6, false, ...
    ['rms(estimate error) is deliberately NOT the metric: against a ' ...
     'sinusoid, a FULL-amplitude estimate lagging 59 deg already has an ' ...
     'rms error of 0.98 of the signal rms, so rms cannot separate lag from ' ...
     'amplitude.  The fitted W-component amplitude can.'], ...
    'prose and Fig. 25.5 caption, zarchan.txt:8621', ...
    'g (fitted amplitude of the W rad/s component of XNTH)', ...
    sprintf('fitted amplitude %.3f g at 1 mr, %.3f g at 0.1 mr, truth %.3f g', ...
            fit.nom.amp/G, fit.low.amp/G, opts.XNTREAL/G));

% ---------------------------------------------------------------- 7  shape
% zarchan.txt:8620 -- "Near the end of the flight, where the effective
% measurement noise on relative position is diminished, the filter estimate
% improves but lags the actual target maneuver."  BOTH halves of that
% sentence are scored: the estimate must get BETTER late, and it must still
% LAG late.
%
% *** rms(XNTH - XNT) IS NOT THE METRIC HERE EITHER, AND THAT IS NOT A
% CONVENIENCE. ***  A first draft of this row used rms over a mid window
% against a late window and came back RED -- late rms LARGER than mid --
% while every other measure of the estimate improved monotonically.  The
% reason is the one already argued at the top of this file: with a large
% standing lag the error is dominated by the phase term, and as the estimate
% AMPLITUDE grows toward the truth the phase term grows with it, so rms can
% rise while the estimate is plainly getting better.  Scoring the fitted
% amplitude and lag separates the two; scoring rms would have recorded the
% book as wrong about its own figure.  The rejected rms row is left in the
% record deliberately: it is why this one is shaped the way it is.
%
% SIGPOS = VC*tgo*SIGRIN falls linearly to zero over the flight, so the
% prose's stated mechanism is in the code, not in the draw.
fE = R.fit.early;  fL = R.fit.late;
dA_E = abs(fE.amp - opts.XNTREAL);
dA_L = abs(fL.amp - opts.XNTREAL);
ok7  = isfinite(dA_E) && isfinite(dA_L) && isfinite(fE.lag) && ...
       isfinite(fL.lag) && dA_L < dA_E && fL.lag < fE.lag && fL.lag > 0;
rows{end+1} = mkRow('1 mr: estimate improves late, and still lags', 'shape', ...
    NaN, 'improves but lags (ordering; no number published)', NaN, ...
    sprintf('late |dA| %.2f g < early %.2f g; lag %.2f s < %.2f s, > 0', ...
            dA_L/G, dA_E/G, fL.lag, fE.lag), ...
    NaN, 'strict ordering, both clauses', NaN, ok7, false, ...
    sprintf(['one whole weave period each, [%.2f %.2f] s and [%.2f %.2f] s; ' ...
             'the late window stops at t = %.2f s so the listing 1e-5 / ' ...
             '1e-6 TGO mismatch cannot reach it'], tEarly(1), tEarly(2), ...
             tLate(1), tLate(2), tSafe), ...
    'prose about Fig. 25.3, zarchan.txt:8620', ...
    'g (fitted amplitude of the W rad/s component of XNTH) and s (lag)', ...
    sprintf('early %.3f g / %+.3f s, late %.3f g / %+.3f s, truth %.3f g', ...
            fE.amp/G, fE.lag, fL.amp/G, fL.lag, opts.XNTREAL/G));

% ---------------------------------------------------------------- 8  shape
% DELETE EXPERIMENT for zarchan.txt:8620 -- "the presence of process noise in
% the filter enables the Kalman filter to track all types of target
% maneuvers.  Process noise lets the filter know that its model of the real
% world may be in error."  Setting PHIN = 0 drives the Riccati gains toward
% zero, the estimate freezes, and the W rad/s component of XNTH collapses.
% Without this row PHIN could be mis-transcribed (XNTMAX^2*TF instead of
% XNTMAX^2/TF is a factor of 100 here) and nothing else would notice.
ok8 = isfinite(fit.noq.amp) && isfinite(fit.nom.amp) && ...
      fit.noq.amp < fit.nom.amp;
rows{end+1} = mkRow('process noise is what lets the filter track', 'shape', ...
    NaN, 'process noise enables tracking (ordering)', NaN, ...
    sprintf('amp %.4f g with PHIN = 0 vs %.4f g with PHIN = %.4g', ...
            fit.noq.amp/G, fit.nom.amp/G, R.p.PHIN), ...
    NaN, 'strict ordering', NaN, ok8, false, ...
    ['delete experiment: same noise draws, same measurement noise, only ' ...
     'PHIN zeroed'], ...
    'prose, zarchan.txt:8620', ...
    'g (fitted amplitude of the W rad/s component of XNTH)', ...
    sprintf('PHIN nominal = XNTMAX^2/TF = %.4f (ft/s^2)^2/s', R.p.PHIN));

% ---------------------------------------------------------------- 9  info
rows{end+1} = mkRow('Listing 25.2 is the point of this baseline', 'info', ...
    NaN, ['"much better than the general purpose linear three-state ' ...
          'Kalman filter when the measurement noise is 1 mr"'], ...
    NaN, 'not evaluated here', NaN, 'n/a', NaN, false, true, ...
    ['the comparison needs BOTH listings, so it belongs to the four-state ' ...
     'port, not to this one.  Recorded here so the pair is visible from ' ...
     'either side.'], ...
    'prose, zarchan.txt:8842', '', ...
    'see mgs.zarchan.weaveKalmanFilter (Listing 25.2, zarchan.txt:8875)');

% ---------------------------------------------------------------- 10  skip
rows{end+1} = mkRow('rms miss distance vs flight time', 'skip', ...
    NaN, 'published as Figs. 25.14-25.16 only', NaN, 'NOT SCOREABLE', ...
    NaN, 'n/a', NaN, false, true, ...
    ['the experiment is "Twenty-five Monte Carlo sets ... for flight times ' ...
     'ranging from .5 s to 10 s in steps of .5 s" (zarchan.txt:8949) and ' ...
     'its results appear ONLY on figures, which the extraction turned into ' ...
     'image links.  No printed number survives, so there is nothing to ' ...
     'score against.  Widening a band to absorb a figure reading would be ' ...
     'inventing the book value.'], ...
    'zarchan.txt:8949 (the experiment), Figs. 25.14-25.16', '', ...
    'would need 25 x 20 = 500 runs of this listing plus a miss metric');

if isempty(rows)
    B = emptyBench();
else
    B = [rows{:}];
end
end


% =============================================================================
function r = mkRow(what, kind, book, book_str, ours, ours_str, pct, band, ...
                   tol, ok, skipped, why, source, units, detail)
%MKROW  THE ONLY constructor for a benchmark row in this file.
%
%   benchSuite's adaptGeneric is strict: a near-miss schema (book_s for
%   book_str, ok_vec for oks) ERRORS rather than being accommodated, and a
%   field ORDER that drifts between two rows makes them refuse to
%   concatenate at all.  One constructor is how that is made impossible.
%
%   Field order, fixed: what kind book book_str ours ours_str pct band tol
%                       ok skipped why source units detail
r = struct( ...
    'what',     what, ...
    'kind',     kind, ...
    'book',     book, ...
    'book_str', book_str, ...
    'ours',     ours, ...
    'ours_str', char(ours_str), ...
    'pct',      pct, ...
    'band',     band, ...
    'tol',      tol, ...
    'ok',       logical(ok), ...
    'skipped',  logical(skipped), ...
    'why',      why, ...
    'source',   source, ...
    'units',    units, ...
    'detail',   detail);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  A struct array carrying the benchmark schema with no rows, so
%   that [B.ok] and [B.skipped] are [] rather than an error when nothing was
%   scored.  Built through mkRow so its field order cannot diverge from that
%   of a real row.
B = mkRow('', 'info', NaN, '', NaN, '', NaN, '', NaN, false, true, ...
          '', '', '', '');
B(1) = [];
end


% =============================================================================
function w = measWeaveFreq(t, x)
%MEASWEAVEFREQ  Weave frequency from the interpolated upward zero crossings
%   of the RECORDED target acceleration.  Returns NaN, never 0, if there are
%   fewer than two crossings -- a zero would read as a result.
s   = sign(x(:));
idx = find(s(1:end-1) < 0 & s(2:end) >= 0);
if numel(idx) < 2
    w = NaN;
    return
end
tt  = t(:);  xx = x(:);
den = xx(idx+1) - xx(idx);
tc  = tt(idx) - xx(idx).*(tt(idx+1) - tt(idx))./den;
w   = 2*pi/mean(diff(tc));
end


% =============================================================================
function [tau, n] = measTau(t, XNC, XNL, tgo, tgomin)
%MEASTAU  Recover the airframe time constant from the run itself.
%
%   XNC is updated once per TS and then HELD, so on [t_k, t_k + TS] the exact
%   solution of XNLD = (XNC - XNL)/TAU gives
%
%       (XNL(k+1) - XNL(k))/TS = (XNC(k) - mean_[k,k+1] XNL)/TAU
%
%   The midpoint stands in for that mean; the error is O((TS/TAU)^2/12).
%   Least squares through the origin: TAU = sum(x^2)/sum(x*y).
sel = tgo >= tgomin;              % tgo decreases, so this selects a prefix
i   = find(sel);
n   = numel(i);
if n < 20
    tau = NaN;
    return
end
i   = i(1:end-1);
dt  = t(i+1) - t(i);
y   = (XNL(i+1) - XNL(i))./dt;                   % dXNL/dt at the midpoint
xm  = XNC(i) - 0.5*(XNL(i) + XNL(i+1));          % (XNC - XNL) at the midpoint
den = sum(xm.*y);
if den == 0
    tau = NaN;
else
    tau = sum(xm.^2)/den;
end
end


% =============================================================================
function [vc, n] = measVc(Y, YSTAR, tgo, SIGRIN, tgomin)
%MEASVC  Recover the closing velocity from the measurement noise scale.
%   YSTAR - Y is VC*tgo*SIGRIN times a unit normal, plus a 9e-6*Y/tgo term
%   from the listing's 1e-5 / 1e-6 TGO mismatch -- which is under 1e-4 of the
%   noise for tgo >= 0.1 s, and is why the window stops there.
sel = tgo >= tgomin;
n   = sum(sel);
if n < 30
    vc = NaN;
    return
end
z  = (YSTAR(sel) - Y(sel)) ./ (tgo(sel)*SIGRIN);
vc = sqrt(mean(z.^2));
end


% =============================================================================
function s = render(R, ev)
%RENDER  The text block.  Reads R.p -- what the run ACTUALLY used -- and
%   never opts, whose [] placeholders are not what was flown.
L = {};
L{end+1} = 'mgs.zarchan.weaveKalmanThreeState -- Zarchan Listing 25.1';
L{end+1} = '  ORIGINAL three-state linear Kalman filter vs a WEAVING target';
L{end+1} = sprintf('  units: %s', R.units);
L{end+1} = sprintf(['  TAU %.3g s | APN %d | VC %g ft/s | nT %g ft/s^2 ' ...
    '(%.3g g) | W %g rad/s | N'' %g'], R.p.TAU, R.p.APN, R.p.VC, ...
    R.p.XNTREAL, R.p.XNTREAL/32.2, R.p.W, R.p.XNP);
L{end+1} = sprintf(['  TS %g s | H %g s | TF %g s | PHIN %.4g | SIGRIN %g ' ...
    'rad (%g mr) | low %g rad (%g mr) | seed %g'], R.p.TS, R.p.H, R.p.TF, ...
    R.p.PHIN, R.p.SIGRIN, 1000*R.p.SIGRIN, R.p.SIGLOW, 1000*R.p.SIGLOW, ...
    R.p.seed);
L{end+1} = sprintf(['  fitted W-component of XNTH over [%.2f %.2f] s:  ' ...
    '1 mr %.3f g / lag %+.3f s | 0.1 mr %.3f g / lag %+.3f s | PHIN=0 ' ...
    '%.4f g / lag %+.3f s'], R.fit.window(1), R.fit.window(2), ...
    R.fit.nom.amp/32.2, R.fit.nom.lag, R.fit.low.amp/32.2, R.fit.low.lag, ...
    R.fit.noq.amp/32.2, R.fit.noq.lag);
L{end+1} = '';
L{end+1} = ['  BENCHMARKS (book values grepped from zarchan_decoded.txt; ' ...
            'no figure was read)'];

for i = 1:numel(R.bench)
    b = R.bench(i);
    if b.skipped
        mark = upper(b.kind);
    elseif b.ok
        mark = 'PASS';
    else
        mark = 'FAIL';
    end
    L{end+1} = sprintf('   %-5s %-44s book %-30s ours %s', ...
        mark, trunc(b.what,44), trunc(b.book_str,30), b.ours_str); %#ok<AGROW>
    if ~b.skipped && isfinite(b.pct)
        L{end+1} = sprintf('           %+.2f%% against a band of %s   [%s]', ...
            b.pct, b.band, b.source); %#ok<AGROW>
    else
        L{end+1} = sprintf('           [%s]', b.source); %#ok<AGROW>
    end
end

L{end+1} = '';
L{end+1} = ['  STRUCTURAL (self-consistency only -- NOT evidence of ' ...
            'agreement with the book)'];
for i = 1:numel(R.selfcheck.names)
    if R.selfcheck.oks(i)
        mark = 'ok  ';
    else
        mark = 'BAD ';
    end
    L{end+1} = sprintf('   %s %-50s %s', mark, ...
        trunc(R.selfcheck.names{i},50), R.selfcheck.detail{i}); %#ok<AGROW>
end

nEval = sum(ev);
nBad  = sum(ev & ~[R.bench.ok]);
L{end+1} = '';
L{end+1} = sprintf(['  %d row(s) evaluated, %d outside tolerance; %d ' ...
    'structural invariant(s), %d violated.'], nEval, nBad, ...
    numel(R.selfcheck.oks), sum(~R.selfcheck.oks));
L{end+1} = ['  NOTE: Chapter 25 prints no miss distance, no rms and no ' ...
            'gain for Listing 25.1.  A thin'];
L{end+1} = ['        board here is the correct outcome; the skip row says ' ...
            'what was left on the table.'];

s = strjoin(L, newline);
end


% =============================================================================
function s = trunc(s, n)
%TRUNC  Fixed-width column helper.  Never errors on a short string.
s = char(s);
if numel(s) > n
    s = [s(1:n-3) '...'];
end
end


% =============================================================================
function drawCurves(R)
%DRAWCURVES  The listing's own two figures, plus the 0.1 mr variant.
%   The FINAL sample is dropped from every plot: it is evaluated at
%   tgo = 1e-6, where XNC is of order 1e10 and YSTAR is not a measurement of
%   anything.
G = 32.2;
k = 1:(numel(R.nom.t) - 1);

figure;
plot(R.nom.t(k), R.nom.XNT(k)/G, '-', R.nom.t(k), R.nom.XNTH(k)/G, '-');
grid on;
title(sprintf('Acceleration Estimate (%g mr)', 1000*R.p.SIGRIN));
xlabel('Time (Sec)'); ylabel('Acceleration (G)');
legend('actual nT', 'estimate', 'Location', 'best');

figure;
plot(R.nom.t(k), R.nom.Y(k), '-', R.nom.t(k), R.nom.YSTAR(k), '-');
grid on;
title(sprintf('Measurement and Signal (%g mr)', 1000*R.p.SIGRIN));
xlabel('Time (Sec)'); ylabel('Y (Ft)');
legend('Y', 'Y*', 'Location', 'best');

figure;
plot(R.low.t(k), R.low.XNT(k)/G, '-', R.low.t(k), R.low.XNTH(k)/G, '-');
grid on;
title(sprintf('Acceleration Estimate (%g mr)', 1000*R.p.SIGLOW));
xlabel('Time (Sec)'); ylabel('Acceleration (G)');
legend('actual nT', 'estimate', 'Location', 'best');
end
