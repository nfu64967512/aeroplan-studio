function R = weaveKalmanFilter(opts)
%MGS.ZARCHAN.WEAVEKALMANFILTER  Zarchan Listing 25.2 -- the FOUR-STATE WEAVE
%   Kalman filter, flown inside the homing loop against a real weaving target.
%
%   R = mgs.zarchan.weaveKalmanFilter()
%   R = mgs.zarchan.weaveKalmanFilter('SIGRIN', 1e-4)       the .1 mr case
%   R = mgs.zarchan.weaveKalmanFilter('APN', 4, 'assert', false)  comp. weave
%   R = mgs.zarchan.weaveKalmanFilter('W', 4)               filter mistuned 2x
%
%   Line-by-line port of Listing 25.2 (zarchan.txt:8875-8920).  UNITS ARE THE
%   BOOK'S: ft, ft/s, ft/s^2, ft/s^3, g = 32.2 ft/s^2, angles in rad (the book
%   quotes seeker noise in mr; 1 mr = .001 rad).  Nothing is converted here --
%   convert AT THE CALL SITE and say so there.
%
%   WHY THIS ONE MATTERS HERE.  weaveDesignCurve and weaveCompensated cover the
%   GUIDANCE side of a weave.  This is the ESTIMATION side: from noisy relative
%   position alone, recover the target's sinusoidal acceleration AND its jerk,
%   by putting the sinusoid into the filter's own model.  State is
%   [y, ydot, yddT, ydddT]; Fk is the exact discrete harmonic oscillator at a
%   KNOWN frequency w.  Sibling mgs.zarchan.weaveKalmanThreeState (Listing
%   25.1) is the same engagement with a CONSTANT-maneuver filter, and the
%   book's whole point (:8842) is that comparison.
%
%   *** WHAT IT IS NOT. ***  Planar, linearised, constant closing velocity, one
%   first-order flight-control lag, NO acceleration limit, no gravity or drag,
%   and w known EXACTLY.  Our own weave completes 0.21 of a period, which is
%   the regime where "estimate the sinusoid" has least to give.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_25_02.m reports NO unresolved variable-to-
%   variable comparisons, and that is right: every comparison in 25.2 is
%   against a literal.  Two OCR repairs, both shape-forced, plus two drops:
%     1  "while T,=(TF-.0001)" -> "while T <= (TF-.0001)".  "," is this book's
%        OCR of "<".  T only increases, so ">=" never terminates.  The BOUND is
%        the listing's own 1e-4; nothing is adopted from a sibling listing.
%     2  "if S.=(TS-.00001)"   -> "if S >= (TS-.00001)".  "." is the OCR of
%        ">".  S is reset to 0 inside the block, so "<=" would fire every step
%        and run the filter at H while its own Fk and Qk still carried TS -- a
%        ten-times-wrong filter that runs perfectly smoothly.
%     3  Dropped: the trailing "MISS DISTANCE ANALYSIS", which is the book's
%        next section heading (:8924) leaking one line past the listing.
%     4  Dropped: "FLAG=0" after the predictor (the outer loop re-initialises
%        it, so it cannot be read) and the four figures.
%   NOT a repair, kept: "while STEP <= 1" already reads correctly.
%   PHASE is assigned on line 1 and never read.  Listing 25.3 (:9147), which is
%   built from this one, uses it as sin(WREAL*T + PHASE); this port applies it,
%   which is a NO-OP at the listing's PHASE = 0.  A declared option that
%   silently does nothing is the exact failure mode this repo keeps paying for.
%
%   Reproduced deliberately:
%     * BOTH TGO expressions carry +1e-6 (unlike Listing 25.1, where the
%       derivative block uses 1e-5 and the sampled block 1e-6).  So YSTAR here
%       really is a measurement of y, all the way in.
%     * XLAM is formed in the SECOND derivative pass, from the PREDICTED Y, and
%       is not recomputed after the corrector.  YSTAR is therefore built on a
%       Y that is O(H^2) away from the recorded one -- the gap is -.5*H^2*ydd,
%       1e-4 ft against a noise sigma of 9 ft at tgo = 1 s.  Measured EXACTLY
%       by structural check 6, which subtracts the recorded draw.
%       ⚠️ That ratio inverts at the very end.  The last sample sits on the
%       listing's tgo = +1e-6 epsilon, where the noise sigma is 9e-6 ft and the
%       same gap has grown to 1.5e-3 ft: 168 sigma.  Nothing downstream uses
%       it (see XNC(end) below), but it is why check 6 is not a plain rms.
%     * Q(1,1) uses the book's ".333", not 1/3.  At the nominal x = w*Ts = .02
%       that rounding is FIVE ORDERS OF MAGNITUDE larger than the term it
%       rounds and it drives Qk(1,1) NEGATIVE.  It is also numerically inert.
%       Kept, measured, and reported as an INFO row; 'q11_exact' switches it.
%     * inv() on a 1-by-1 is done as a division.  Same number, no lu().
%
%   RANDOMNESS.  The listing calls randn.  A benchmark that moves when someone
%   else calls rng() is not a benchmark, so draws come from a Lehmer generator
%   in this file and the global stream is never touched.  THE SAME DRAWS FEED
%   THE 1 mr, .1 mr AND MISTUNED RUNS: each comparison moves one variable.
%
%   *** WHY rms(estimate error) IS NEVER USED AS AN ABSOLUTE QUALITY METRIC. ***
%   Against a sinusoid, an estimate at full amplitude and 60 deg of lag has an
%   rms error equal to the signal's own rms -- no better than estimating zero.
%   Benchmark 8 therefore scores AMPLITUDE and LAG of the w component (sin/cos
%   projection over whole periods).  Benchmarks 7 and 9 do use rms, but only to
%   compare one run against another under identical noise, where the metric's
%   absolute meaning does not matter.
%
%   WHAT THIS PRINTS.  *** MEASURED IN MATLAB, 2026-08-24.  Until that date
%   this block said "DERIVED, NOT MEASURED -- this file has never been
%   executed"; it has now been run and every number below is what it produces.
%   The derivation was right everywhere it committed to a digit. ***
%     Qk(1,1) as printed:  -3.111e-7 ft^2  (exact 1/3 gives +5.917e-13 ft^2)
%     N' at tgo/TAU = 1e4: 3.0006         (the "approaches 3" row, +0.02%)
%     N' at tgo/TAU = 1:   12.3           (it rises as the missile closes)
%     w-component of the estimate: 1 mr amp 70.31 ft/s^2 (0.728 of truth),
%       lag +0.013 s; .1 mr amp 98.03 (1.015 of truth), lag -0.001 s
%     containment inside +/-sqrt(P33), +/-sqrt(P44): 0.9264 / 0.9256 over
%       5000 samples -- ABOVE the book's 68%, and expected to be: the filter
%       assumes a target that MIGHT start weaving (Phis = w^2 n^2/tF) while
%       the truth weaves deterministically, so P is an upper bound.
%       ⚠️ THAT IS THE SAME DISAGREEMENT kalmanHomingLoop CARRIES AS INFO
%       (0.919 vs 0.68 on the identical sentence at :2806).  Here it is still
%       SCORED, and the reason is measured, not asserted -- see benchmark 6.
%     at .1 mr the estimate should be near-exact: amplitude ratio within a few
%       percent of 1 and |lag| well under the 0.5 s the THREE-state filter
%       shows at :8621.
%   Do NOT widen a band to absorb a disagreement with any of this.
%
%   OUTPUTS
%     R.nom .low .hi .lo   per-run records sampled at TS: .t .tgo .YTDD .YTDDH
%                          .YTDDD .YTDDDH .SP33 .SP44 .Y .YSTAR .YH .YDH .XNL
%                          .XNC .gdraw .GAIN(:,4) .Pdiag(:,4) .miss .dprop
%                          .gdraw is the unit normal that measurement actually
%                          consumed -- recorded so the measurement-scale
%                          invariant is an identity, not a statistic.
%                          ** .XNC(end) is evaluated at tgo = 1e-6 and is of
%                          order 1e15; it never reaches the airframe because
%                          the loop exits, but do not plot it. **
%     R.ens                the nominal ensemble's pooled 1-sigma containment
%     R.fit                amplitude / phase / lag of the w component of YTDDH
%     R.rms                rms estimate errors, per run, over the fit window
%     R.phi .q .q_exact    the model matrices actually used
%     R.bench R.selfcheck R.units R.p R.pass R.verdict R.summary
%
%   See also MGS.ZARCHAN.WEAVEKALMANTHREESTATE, MGS.ZARCHAN.WEAVECOMPENSATED,
%   MGS.ZARCHAN.WEAVEEXTENDEDKALMAN, MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 25.2 line 1 (zarchan.txt:8876), verbatim.  Feet, seconds, rad.
    opts.TAU      (1,1) double {mustBeFinite, mustBePositive} = 0.5     % s
    opts.APN      (1,1) double {mustBeMember(opts.APN,[0 1 2 3 4])} = 0 % 0 = PN
    opts.MVR      (1,1) double {mustBeMember(opts.MVR,[0 1])} = 1       % 1 = weave
    opts.VC       (1,1) double {mustBeFinite, mustBePositive} = 9000    % ft/s
    opts.W        (1,1) double {mustBeFinite, mustBePositive} = 2.0     % rad/s, FILTER (listing: WH = W)
    opts.WREAL    (1,1) double {mustBeFinite, mustBePositive} = 2.0     % rad/s, TRUTH
    opts.XNT      (1,1) double {mustBeFinite, mustBePositive} = 96.6    % ft/s^2, filter's assumed level
    opts.XNTREAL  (1,1) double {mustBeFinite, mustBePositive} = 96.6    % ft/s^2, truth
    opts.TS       (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s
    opts.YIC      (1,1) double {mustBeFinite} = 0.0                     % ft
    opts.VM       (1,1) double {mustBeFinite, mustBePositive} = 3000    % ft/s
    opts.HEDEG    (1,1) double {mustBeFinite} = 0.0                     % deg
    opts.HEDEGFIL (1,1) double {mustBeFinite} = 20.0                    % deg
    opts.XNP      (1,1) double {mustBeFinite, mustBePositive} = 3.0
    opts.SIGRIN   (1,1) double {mustBeFinite, mustBePositive} = 0.001   % rad (1 mr)
    % The book's variant at :8852, "reduced by an order of magnitude to .1 mr".
    opts.SIGLOW   (1,1) double {mustBeFinite, mustBePositive} = 0.0001  % rad
    opts.SIGGL    (1,1) double {mustBeFinite, mustBeNonnegative} = 0.0  % ft (glint)
    opts.RA       (1,1) double {mustBeFinite, mustBePositive} = 21000   % ft
    opts.SRN      (1,1) double {mustBeFinite, mustBeNonnegative} = 0.0
    opts.TF       (1,1) double {mustBeFinite, mustBePositive} = 10.0    % s
    opts.H        (1,1) double {mustBeFinite, mustBePositive} = 0.001   % s
    opts.QPERFECT (1,1) double {mustBeMember(opts.QPERFECT,[0 1])} = 0
    opts.PHASE    (1,1) double {mustBeFinite} = 0.0                     % rad
    % false = the book's printed ".333" in Qk(1,1).  See the header.
    opts.q11_exact (1,1) logical = false
    % Independent noise realisations for the containment row.  The book's own
    % Figs. 25.9/25.10 are ONE run; more runs only sharpen the same statistic.
    opts.runs     (1,1) double {mustBeFinite, mustBePositive} = 5
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
    error('mgs:zarchan:weaveKalmanFilter:sample', ...
        ['TS = %g s must be more than two steps of H = %g s.  Below that the ' ...
         'listing''s record test fires every step and the filter silently ' ...
         'runs at H while Fk and Qk still carry TS.'], TS, H);
end
if abs(TS/H - round(TS/H)) > 1e-9
    error('mgs:zarchan:weaveKalmanFilter:commensurate', ...
        'TS = %g s is not a whole number of steps of H = %g s.', TS, H);
end
% The fit window is built on the TRUE weave frequency, not the filter's: the
% mistuned runs of benchmark 9 are measured against the same truth.
WR   = opts.WREAL;
nPer = floor((TF - 0.2 - 2.0)/(2*pi/WR));     % whole weave periods after t = 2 s
if nPer < 1
    error('mgs:zarchan:weaveKalmanFilter:short', ...
        ['TF = %g s at WREAL = %g rad/s leaves no WHOLE weave period (%.3f s) ' ...
         'between t = 2 s and TF - 0.2 s.  A partial window leaks the sin ' ...
         'projection into the cos one and manufactures a phase.'], TF, WR, 2*pi/WR);
end
tFit  = [2.0, 2.0 + min(nPer,2)*(2*pi/WR)];
nRuns = round(opts.runs);

% ---- the filter's model.  PHIS = w^2 n^2 / tF is the book's spectral density
%      (zarchan.txt:8757): "the target MIGHT start weaving", not "does".
PHIS = W*W*opts.XNT*opts.XNT/TF;
[PHI, Q]  = weaveModel(W, TS, PHIS, opts.q11_exact);
[~, Qex]  = weaveModel(W, TS, PHIS, true);

% ---- runs.  One noise stream per ensemble member; member 1's stream is
%      re-used by every variant so each comparison moves ONE thing.
nDraw = ceil(TF/TS) + 8;
G = zeros(nRuns, nDraw);
for k = 1:nRuns
    G(k,:) = gaussLehmer(opts.seed + 977*(k-1), nDraw);
end

base = packParams(opts, opts.SIGRIN, W);
ens  = cell(1, nRuns);
for k = 1:nRuns
    ens{k} = runEngagement(base, PHI, Q, G(k,:));
end
nom = ens{1};
low = runEngagement(packParams(opts, opts.SIGLOW, W), PHI, Q, G(1,:));

% Mistuned filters: the book's own experiment at :8975 -- "twice as large or
% half as small".  Fk, Qk, PHIS and the update all retune with WH.
[PHIhi, Qhi] = weaveModel(2*W, TS, (2*W)^2*opts.XNT^2/TF, opts.q11_exact);
[PHIlo, Qlo] = weaveModel(0.5*W, TS, (0.5*W)^2*opts.XNT^2/TF, opts.q11_exact);
hi = runEngagement(packParams(opts, opts.SIGRIN, 2*W),   PHIhi, Qhi, G(1,:));
lo = runEngagement(packParams(opts, opts.SIGRIN, 0.5*W), PHIlo, Qlo, G(1,:));

% ---- pooled 1-sigma containment over the ensemble (benchmark 6) ------------
inA = 0;  inJ = 0;  nS = 0;
for k = 1:numel(ens)
    e = ens{k};
    inA = inA + sum(abs(e.YTDD - e.YTDDH)  <= e.SP33);
    inJ = inJ + sum(abs(e.YTDDD - e.YTDDDH) <= e.SP44);
    nS  = nS + numel(e.t);
end

R = struct();
R.nom = nom;  R.low = low;  R.hi = hi;  R.lo = lo;
R.ens = struct('runs',numel(ens), 'samples',nS, 'frac_acc',inA/nS, ...
               'frac_jerk',inJ/nS);
R.fit = struct('nom', weaveFit(nom.t, nom.YTDDH, opts.WREAL, tFit), ...
               'low', weaveFit(low.t, low.YTDDH, opts.WREAL, tFit), ...
               'window', tFit);
R.rms = struct( ...
    'nom_acc',  rmsWin(nom.t, (nom.YTDD-nom.YTDDH)/32.2, tFit), ...
    'low_acc',  rmsWin(low.t, (low.YTDD-low.YTDDH)/32.2, tFit), ...
    'hi_acc',   rmsWin(hi.t,  (hi.YTDD-hi.YTDDH)/32.2,  tFit), ...
    'lo_acc',   rmsWin(lo.t,  (lo.YTDD-lo.YTDDH)/32.2,  tFit), ...
    'nom_jerk', rmsWin(nom.t, (nom.YTDDD-nom.YTDDDH)/32.2, tFit), ...
    'low_jerk', rmsWin(low.t, (low.YTDDD-low.YTDDDH)/32.2, tFit));
R.phi = PHI;  R.q = Q;  R.q_exact = Qex;
R.units = ['ft, ft/s, ft/s^2, ft/s^3, s; g = 32.2 ft/s^2; LOS noise in rad ' ...
           '(1 mr = .001); w in rad/s'];
R.p = struct('TAU',opts.TAU, 'APN',opts.APN, 'MVR',opts.MVR, 'VC',opts.VC, ...
    'W',W, 'WREAL',opts.WREAL, 'XNT',opts.XNT, 'XNTREAL',opts.XNTREAL, ...
    'TS',TS, 'VM',opts.VM, 'HEDEGFIL',opts.HEDEGFIL, 'XNP',opts.XNP, ...
    'SIGRIN',opts.SIGRIN, 'SIGLOW',opts.SIGLOW, 'RA',opts.RA, 'TF',TF, ...
    'H',H, 'PHIS',PHIS, 'X',W*TS, 'QPERFECT',opts.QPERFECT, ...
    'PHASE',opts.PHASE, 'q11_exact',opts.q11_exact, 'runs',nRuns, ...
    'seed',opts.seed, 'fit_window',tFit, 'n_rhs',nom.nrhs, 'units',R.units);

R.selfcheck = structuralChecks(R, opts, PHI, Qex, PHIS);

if opts.benchmark
    R.bench = benchmarks(R, opts);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    R.pass  = false;      % "not checked" must never render as "fine"
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R); end

% ---- adjudication.  The verdict is FORMED either way; 'assert' chooses only
%      the reaction.  benchSuite runs every port with assert off, so a failing
%      port must still RETURN, complete, with the evidence attached.
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
        error('mgs:zarchan:weaveKalmanFilter:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:weaveKalmanFilter:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function [PHI, Q] = weaveModel(WH, TS, PHIS, exact11)
%WEAVEMODEL  Fk and Qk of Listing 25.2, zarchan.txt:8881 (printed again as
%   prose at :8766-8772 and :8737-8756).  x = w*Ts.
%
%   Q(1,1)'s leading coefficient is the book's ".333", NOT 1/3, in BOTH places
%   the book prints it.  The bracket's true value is x^7/252 = 5.08e-15 at
%   x = .02 while the rounding error is (1/3 - .333)*x^3 = -2.67e-9, so the
%   printed form returns a NEGATIVE Qk(1,1) five orders of magnitude too big.
%   It is inert (M(1,1) never gets near 3e-7 ft^2), and it is kept, because
%   quietly "fixing" a published listing is how a port stops being one.
X = WH*TS;
c = cos(X);  s = sin(X);  s2 = sin(2*X);

PHI = zeros(4);
PHI(1,1) = 1;   PHI(1,2) = TS;  PHI(1,3) = (1-c)/(WH*WH);  PHI(1,4) = (X-s)/(WH^3);
PHI(2,2) = 1;   PHI(2,3) = s/WH;  PHI(2,4) = (1-c)/(WH*WH);
PHI(3,3) = c;   PHI(3,4) = s/WH;
PHI(4,3) = -WH*s;  PHI(4,4) = c;

if exact11
    a11 = 1/3;
else
    a11 = .333;                        % <<< the book's, verbatim
end
Q = zeros(4);
Q(1,1) = PHIS*(a11*X^3 - 2*s + 2*X*c + .5*X - .25*s2)/(WH^5);
Q(1,2) = PHIS*(.5*X*X - X*s + .5*s*s)/(WH^4);            Q(2,1) = Q(1,2);
Q(1,3) = PHIS*(s - X*c - .5*X + .25*s2)/(WH^3);          Q(3,1) = Q(1,3);
Q(1,4) = PHIS*(c + X*s - .5*s*s - 1)/(WH*WH);            Q(4,1) = Q(1,4);
Q(2,2) = PHIS*(1.5*X - 2*s + .25*s2)/(WH^3);
Q(2,3) = PHIS*(1 - c - .5*s*s)/(WH*WH);                  Q(3,2) = Q(2,3);
Q(2,4) = PHIS*(s - .5*X - .25*s2)/WH;                    Q(4,2) = Q(2,4);
Q(3,3) = PHIS*(.5*X - .25*s2)/WH;
Q(3,4) = .5*PHIS*s*s;                                    Q(4,3) = Q(3,4);
Q(4,4) = WH*PHIS*(.5*X + .25*s2);
end


% =============================================================================
function P = packParams(opts, SIGRIN, WH)
%PACKPARAMS  Everything runEngagement needs, as ONE argument.  MATLAB local
%   functions cannot see the caller's workspace, and this package has already
%   been bitten by a local reading a constant defined only in the main function.
P = struct('TAU',opts.TAU, 'APN',opts.APN, 'MVR',opts.MVR, 'VC',opts.VC, ...
    'WH',WH, 'WREAL',opts.WREAL, 'XNT',opts.XNT, 'XNTREAL',opts.XNTREAL, ...
    'TS',opts.TS, 'YIC',opts.YIC, 'VM',opts.VM, 'HEDEG',opts.HEDEG, ...
    'HEDEGFIL',opts.HEDEGFIL, 'XNP',opts.XNP, 'SIGRIN',SIGRIN, ...
    'SIGGL',opts.SIGGL, 'RA',opts.RA, 'SRN',opts.SRN, 'TF',opts.TF, ...
    'H',opts.H, 'QPERFECT',opts.QPERFECT, 'PHASE',opts.PHASE);
end


% =============================================================================
function O = runEngagement(P, PHI, Q, G)
%RUNENGAGEMENT  Listing 25.2, zarchan.txt:8876-8920, statement for statement.
%
%   G is the pre-drawn unit-normal stream, one draw per Kalman update, passed
%   in so that two configurations can be given IDENTICAL noise.

TS = P.TS;  H = P.H;  TF = P.TF;  VC = P.VC;  TAU = P.TAU;  XNP = P.XNP;
WH = P.WH;  X  = WH*TS;  APN = P.APN;  MVR = P.MVR;
SIGRIN = P.SIGRIN;  SIGGL = P.SIGGL;  SRN = P.SRN;  RA = P.RA;

RTM = VC*TF;
SIGNOISE = sqrt(SIGRIN^2 + (SIGGL/RTM)^2 + (SRN*RTM*RTM/(RA*RA))^2);
SIGPOS = RTM*SIGNOISE;  SIGN2 = SIGPOS^2;

Pm = zeros(4);
Pm(1,1) = SIGN2;
Pm(2,2) = (P.VM*P.HEDEGFIL/57.3)^2;
Pm(3,3) = P.XNT*P.XNT;
Pm(4,4) = WH*WH*P.XNT*P.XNT;
HMAT = [1 0 0 0];  HT = HMAT.';  PHIT = PHI.';  IDNP = eye(4);
GK = [.5*TS*TS; TS; 0; 0];        % discrete control matrix, :8712

T = 0.;  S = 0.;  XNC = 0.;  XNL = 0.;
Y = P.YIC;  YD = -P.VM*P.HEDEG/57.3;
XLAM = Y/RTM;
[YTDD, YTDDD] = targetAccel(MVR, P.XNTREAL, P.WREAL, P.PHASE, T);
if P.QPERFECT == 1
    YH = Y;  YDH = YD;  YTDDH = YTDD;  YTDDDH = YTDDD;
else
    YH = 0.;  YDH = 0.;  YTDDH = 0.;  YTDDDH = 0.;
end
% Initialised before the loop so the FLAG==1 branch cannot read a stale value.
% (The analyzer calls three of them unused; they are the point.)
YDD = 0.;  XNLD = 0.;  GAIN = zeros(4,1);  YSTAR = 0.;  TGO = TF;  %#ok<NASGU>

nmax = ceil(TF/TS) + 8;
z = zeros(nmax,1);
O = struct('t',z, 'tgo',z, 'YTDD',z, 'YTDDH',z, 'YTDDD',z, 'YTDDDH',z, ...
           'SP33',z, 'SP44',z, 'Y',z, 'YSTAR',z, 'YH',z, 'YDH',z, 'XNL',z, ...
           'XNC',z, 'gdraw',z, 'GAIN',zeros(nmax,4), 'Pdiag',zeros(nmax,4), ...
           'n',0, 'nrhs',0, 'miss',NaN, 'dprop',0);
n = 0;  nrhs = 0;  kdraw = 0;  dprop = 0;

while T <= (TF - .0001)                    % repair 1
    YOLD = Y;  YDOLD = YD;  XNLOLD = XNL;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor.  T advances INSIDE this loop, exactly once,
    % which is what makes the second evaluation happen at t+H.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            Y = Y + H*YD;  YD = YD + H*YDD;  XNL = XNL + H*XNLD;  T = T + H;
        end
        TGO  = TF - T + .000001;
        RTM  = VC*TGO;                     %#ok<NASGU> the listing's own dead
        XLAM = Y/(VC*TGO);                 % store; the sampled block redoes it
        YTDD = targetAccel(MVR, P.XNTREAL, P.WREAL, P.PHASE, T);
        XNLD = (XNC - XNL)/TAU;            % single-time-constant airframe
        YDD  = YTDD - XNL;                 % ACHIEVED accel enters, not XNC
        FLAG = 1;
        nrhs = nrhs + 1;
    end
    % The listing also writes FLAG = 0 here; the outer loop re-initialises it,
    % so the assignment cannot be read.  Kept as a comment, not as code.

    Y   = .5*(YOLD + Y + H*YD);
    YD  = .5*(YDOLD + YD + H*YDD);
    XNL = .5*(XNLOLD + XNL + H*XNLD);

    S = S + H;
    if S >= (TS - .00001)                  % repair 2
        S = 0.;
        TGO = TF - T + .000001;            % same 1e-6 as above; see the header
        RTM = VC*TGO;
        SIGNOISE = sqrt(SIGRIN^2 + (SIGGL/RTM)^2 + (SRN*RTM*RTM/(RA*RA))^2);
        SIGPOS = RTM*SIGNOISE;  SIGN2 = SIGPOS^2;

        % ---- Riccati.  Depends on TGO and the model ONLY, never on a
        %      measurement -- and that is not a comment any more: SELF-CHECK 7
        %      re-runs these four lines from Fk, Qk and the recorded tgo alone
        %      and requires Pdiag and GAIN back to the last bit.  inv() on the
        %      1-by-1 HMHTR is a division.
        M      = PHI*Pm*PHIT + Q;
        HMHTR  = HMAT*M*HT + SIGN2;
        GAIN   = (M*HT)/HMHTR;
        Pm     = (IDNP - GAIN*HMAT)*M;

        [YTDD, YTDDD] = targetAccel(MVR, P.XNTREAL, P.WREAL, P.PHASE, T);

        kdraw = kdraw + 1;
        XLAMNOISE = SIGNOISE*G(kdraw);
        YSTAR = RTM*(XLAM + XLAMNOISE);

        % ---- the four scalar filter equations, listing :8902-8907, derived as
        %      prose at :8818-8834.  ORDER MATTERS:
        %      RES, YH and YDH all read the OLD YTDDH/YTDDDH, and YTDDDH reads
        %      the OLD YTDDH -- hence YTDDHNEW.
        prop = PHI*[YH; YDH; YTDDH; YTDDDH] - GK*XNL;   % the SAME propagation,
        RES = YSTAR - YH - TS*YDH - (1-cos(X))*YTDDH/(WH*WH) ...
              - (X-sin(X))*YTDDDH/(WH*WH*WH) + .5*TS*TS*XNL;
        YH  = YH + TS*YDH + (1-cos(X))*YTDDH/(WH*WH) ...
              + (X-sin(X))*YTDDDH/(WH*WH*WH) + GAIN(1)*RES - .5*TS*TS*XNL;
        YDH = YDH + sin(X)*YTDDH/WH + (1-cos(X))*YTDDDH/(WH*WH) ...
              + GAIN(2)*RES - TS*XNL;
        YTDDHNEW = cos(X)*YTDDH + sin(X)*YTDDDH/WH + GAIN(3)*RES;
        YTDDDH   = -WH*sin(X)*YTDDH + cos(X)*YTDDDH + GAIN(4)*RES;
        YTDDH    = YTDDHNEW;
        % ... written out by hand in the listing.  Structural check 3 is that
        % the hand-written copy and PHI agree; the listing carries BOTH.
        dprop = max(dprop, max(abs([YH;YDH;YTDDH;YTDDDH] - GAIN*RES - prop)));

        % ---- guidance, :8907-8916.  APN = 0..4 = PN, APN, optimal, weave,
        %      compensated weave (:8841).  Note APN=3 is APN=4 with a CONSTANT
        %      N' and no lag term: (1-cos XP)/XP^2 == (1-cos w*tgo)/(w*tgo)^2.
        if APN == 0
            XNC = XNP*(YH + YDH*TGO)/(TGO*TGO);
        elseif APN == 1
            XNC = XNP*(YH + YDH*TGO + .5*YTDDH*TGO*TGO)/(TGO*TGO);
        elseif APN == 2
            XS = TGO/TAU;  XNPP = navRatio(XS);
            C1 = XNPP/(TGO*TGO);  C2 = XNPP/TGO;  C3 = .5*XNPP;
            C4 = -XNPP*(exp(-XS) + XS - 1.)/(XS*XS);
            XNC = C1*YH + C2*YDH + C3*YTDDH + C4*XNL;
        elseif APN == 3
            XP = WH*TGO;
            XNC = XNP*(YH + YDH*TGO)/(TGO*TGO) ...
                  + XNP*YTDDH*(1.-cos(XP))/XP^2 ...
                  + XNP*YTDDDH*(XP-sin(XP))/(XP*XP*WH);
        else
            XS = TGO/TAU;  XNPP = navRatio(XS);
            C1 = XNPP/(TGO*TGO);  C2 = XNPP/TGO;
            C3 = XNPP*(1.-cos(WH*TGO))/(WH*WH*TGO*TGO);
            C4 = -XNPP*(exp(-XS) + XS - 1.)/(XS*XS);
            C5 = XNPP*(WH*TGO - sin(WH*TGO))/(WH*WH*WH*TGO*TGO);
            XNC = C1*YH + C2*YDH + C3*YTDDH + C4*XNL + C5*YTDDDH;
        end

        n = n + 1;
        O.t(n) = T;          O.tgo(n) = TGO;
        O.YTDD(n) = YTDD;    O.YTDDH(n) = YTDDH;
        O.YTDDD(n) = YTDDD;  O.YTDDDH(n) = YTDDDH;
        O.SP33(n) = sqrt(max(Pm(3,3),0));  O.SP44(n) = sqrt(max(Pm(4,4),0));
        O.Y(n) = Y;          O.YSTAR(n) = YSTAR;
        O.YH(n) = YH;        O.YDH(n) = YDH;
        O.XNL(n) = XNL;      O.XNC(n) = XNC;
        % The unit normal this measurement actually consumed.  Recorded so the
        % measurement-scale invariant can be checked EXACTLY rather than as an
        % rms: without it the check has to infer the draw from the residual,
        % which is what let a collapsed denominator masquerade as a scale error.
        O.gdraw(n) = G(kdraw);
        O.GAIN(n,:) = GAIN.';
        O.Pdiag(n,:) = [Pm(1,1) Pm(2,2) Pm(3,3) Pm(4,4)];
    end
end

f = {'t','tgo','YTDD','YTDDH','YTDDD','YTDDDH','SP33','SP44','Y','YSTAR', ...
     'YH','YDH','XNL','XNC','gdraw'};
for i = 1:numel(f)
    O.(f{i}) = O.(f{i})(1:n);
end
O.GAIN = O.GAIN(1:n,:);  O.Pdiag = O.Pdiag(1:n,:);
O.n = n;  O.nrhs = nrhs;  O.miss = Y;  O.dprop = dprop;
end


% =============================================================================
function [YTDD, YTDDD] = targetAccel(MVR, XNTREAL, WREAL, PHASE, T)
%TARGETACCEL  The listing's target: constant if MVR = 0, weaving if MVR = 1.
%   PHASE is declared on line 1 of the listing and never read there; it is
%   applied here and is a NO-OP at the listing's PHASE = 0 (see the header).
if MVR == 0
    YTDD = XNTREAL;  YTDDD = 0.;
else
    YTDD  = XNTREAL*sin(WREAL*T + PHASE);
    YTDDD = XNTREAL*WREAL*cos(WREAL*T + PHASE);
end
end


% =============================================================================
function XNPP = navRatio(XS)
%NAVRATIO  The optimal-guidance effective navigation ratio, zarchan.txt:8933,
%   with the listing's own .0001 guard in the denominator.  XS = tgo/TAU.
TOP  = 6.*XS.*XS.*(exp(-XS) - 1. + XS);
BOT1 = 2*XS.^3 + 3. + 6.*XS - 6.*XS.*XS;
BOT2 = -12.*XS.*exp(-XS) - 3.*exp(-2.*XS);
XNPP = TOP./(.0001 + BOT1 + BOT2);
end


% =============================================================================
function F = weaveFit(t, x, W, win)
%WEAVEFIT  Amplitude, phase and LAG of the W rad/s component of x, by
%   projection onto sin(Wt) and cos(Wt) over a window of WHOLE periods -- which
%   is what makes the projection orthogonal.  The truth is XNTREAL*sin(W t), so
%   a fitted A*sin(W t + phi) has phi < 0 and a lag of -phi/W.  NOT wrapped: a
%   positive phase comes back as a negative lag, and that is a signal.
m = t >= win(1) & t <= win(2);
F = struct('amp',NaN, 'phase',NaN, 'lag',NaN, 'n',sum(m), 'ok',false);
if F.n < 8
    return
end
tt = t(m);  xx = x(m);
cs = 2*mean(xx .* sin(W*tt));
cc = 2*mean(xx .* cos(W*tt));
F.amp = hypot(cs, cc);  F.phase = atan2(cc, cs);  F.lag = -F.phase/W;
F.ok = true;
end


% =============================================================================
function v = rmsWin(t, e, win)
%RMSWIN  rms of e over the window.  NaN (never 0) if empty -- a zero would read
%   as a perfect result.
m = t >= win(1) & t <= win(2);
if ~any(m)
    v = NaN;
else
    v = sqrt(mean(e(m).^2));
end
end


% =============================================================================
function g = gaussLehmer(seed, n)
%GAUSSLEHMER  Unit normals from MINSTD (x <- 16807 x mod 2^31-1) plus
%   Box-Muller.  Deterministic, base MATLAB, and it never touches the global
%   random stream -- inside a suite that runs many ports back to back, calling
%   rng() would silently reach into the caller's session.
%   16807*x < 3.61e13 < 2^53, so every product is EXACT.  x = 0 is absorbing,
%   so the seed is mapped into 1..m-1.  The two uniforms of each Box-Muller
%   pair come from SEPARATE HALVES of the stream: consecutive MINSTD values lie
%   on a coarse 2-D lattice and pairing them turns it into a visible spiral.
m = 2147483647;
x = mod(floor(abs(seed)), m - 1) + 1;
np = 2*ceil(n/2);
u = zeros(1, np);
for k = 1:np
    x = mod(16807*x, m);
    u(k) = x/m;                        % in (0,1); never 0, so log() is safe
end
half = np/2;
r = sqrt(-2*log(u(1:half)));
g = zeros(1, np);
g(1:2:end) = r .* cos(2*pi*u(half + (1:half)));
g(2:2:end) = r .* sin(2*pi*u(half + (1:half)));
g = g(1:n);
end


% =============================================================================
function Qn = qQuadrature(WH, TS, PHIS, n)
%QQUADRATURE  Qk = int_0^Ts Phi(tau) Qc Phi(tau)' dtau by composite Simpson,
%   with Qc = diag(0,0,0,w^2*PHIS) from the book's continuous process-noise
%   matrix (:8720-8728).  Only column 4 of Phi is needed.  Shares NO code with
%   weaveModel, which is the whole reason it is worth having.
if mod(n,2) == 1, n = n + 1; end
tau = linspace(0, TS, n+1);
w = ones(1,n+1);  w(2:2:n) = 4;  w(3:2:n-1) = 2;  w = w*(TS/n)/3;
v = [ (WH*tau - sin(WH*tau))/WH^3
      (1 - cos(WH*tau))/(WH*WH)
      sin(WH*tau)/WH
      cos(WH*tau) ];
Qn = zeros(4);
for i = 1:4
    for j = 1:4
        Qn(i,j) = WH*WH*PHIS*sum(w .* v(i,:) .* v(j,:));
    end
end
end


% =============================================================================
function K = structuralChecks(R, opts, PHI, Qex, PHIS)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan -- they are the weaker
%   claim that it agrees with ITSELF, which is what catches wiring faults.
%
%   .names and .oks are BOTH required and must be the same length: benchSuite
%   only puts a structural row on the board when it finds both, and a port that
%   computes twenty invariants under private field names puts ZERO rows up
%   while looking green.
K = struct();
names = {};  oks = [];  detail = {};
W = opts.W;  TS = opts.TS;  nom = R.nom;

% 1  Fk == expm(Fc*Ts).  The book DERIVES Fk from the state equations; this
%    recomputes it from those equations by an unrelated route.  Catches a
%    transposed or mis-signed entry, which nothing else here would notice.
Fc = [0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 -W*W 0];
dPhi = max(max(abs(PHI - expm(Fc*TS))));
names{end+1} = 'Fk == expm(Fc*Ts)';
oks(end+1)   = dPhi <= 1e-12;
detail{end+1} = sprintf('max |dFk| = %.3e', dPhi);

% 2  Qk (EXACT 1/3) == the quadrature, everything but the (1,1) entry.
Qn = qQuadrature(W, TS, PHIS, 4000);
rel = abs(Qex - Qn)./max(abs(Qn), realmin);
r11 = rel(1,1);  rel(1,1) = 0;
worst = max(rel(:));
names{end+1} = 'Qk == integral(Fk Qc Fk''), entries other than (1,1)';
oks(end+1)   = worst <= 1e-5;
detail{end+1} = sprintf('worst relative %.3e (band 1e-5; Q12 alone loses ~3e-7 to cancellation)', worst);

% 3  The (1,1) entry, separately and loosely, because the printed bracket is
%    cancellation-limited: terms of order .04 produce x^7/252 = 5.1e-15 at
%    x = .02, so ~13 digits go, leaving ~0.2% of the answer.
names{end+1} = 'Qk(1,1) == quadrature (cancellation-limited)';
oks(end+1)   = r11 <= 2e-2;
detail{end+1} = sprintf('relative %.3e (band 2e-2), exact-1/3 form', r11);

% 4  The listing writes the state propagation TWICE -- once as Fk for the
%    Riccati, once by hand in the four scalar update lines.  This is the check
%    that the two copies are the same matrix.  A divergence would leave a
%    filter whose gains were computed for a model it does not run.
names{end+1} = 'hand-written update == Fk*xhat + Gk*(-nL)';
oks(end+1)   = nom.dprop <= 1e-6*max(1, max(abs(nom.YH)));
detail{end+1} = sprintf('max |scalar - matrix| = %.3e ft (over %d updates)', ...
    nom.dprop, nom.n);

% 5  Covariance diagonals cannot go negative, and K1 = M11/(M11+SIGN2) with
%    both terms non-negative is confined to (0,1].  A sign slip in the update
%    lands here and nowhere else cheap.
mnP = min(nom.Pdiag(:));
k1  = nom.GAIN(:,1);
names{end+1} = 'P diagonals >= 0 and K1 in (0,1]';
oks(end+1)   = mnP >= -1e-9*max(nom.Pdiag(:)) && min(k1) > 0 && max(k1) <= 1;
detail{end+1} = sprintf('min diag = %.4g, K1 in [%.6f, %.6f]', mnP, min(k1), max(k1));

% 6  The measurement really is VC*tgo*SIGNOISE on relative position.  Catches a
%    dropped VC, a dropped tgo, SIGPOS used where SIGNOISE belongs, or the
%    wrong draw -- each of which leaves a perfectly smooth run.
%
%    *** WHY THIS IS TWO CLAUSES AND NOT ONE rms. ***  It WAS one rms over
%    every sample, and it read 5.42 against a band of 1 +/- 0.20 and stood
%    VIOLATED on the board with no detail.  The wiring was never wrong.  The
%    listing's TGO carries a +1e-6 s epsilon so its divisions survive T = TF
%    (:8893 derivative block, :8899 sampled block), and the FINAL sample
%    lands exactly there: tgo = 1e-6 s makes the
%    measurement scale VC*tgo*SIGNOISE = 9e-6 ft at the defaults, six orders
%    below the trapezoid's OWN predictor-corrector gap of 1.5e-3 ft.  That one
%    sample scored |z| = 168.5 and supplied 28.40 of the 29.38 mean square; the
%    other 999, spanning a 1000:1 range of the denominator, gave rms 0.9899
%    with a textbook normal shape (median |z| 0.683 vs 0.6745, p99 2.58 vs
%    2.576).  So the failure was a denominator collapsing to nothing, not a
%    scale error -- and an rms cannot tell those apart, which is exactly why
%    the row carried no usable detail.  Re-aimed, NOT widened: the band on the
%    statistical clause is still 1 +/- 0.20, and the terminal sample is not
%    dropped, it is moved to the clause that can actually test it.
%
%    (a) EXACT, over EVERY sample including the terminal one.  YSTAR is built
%        as RTM*(XLAM + SIGNOISE*g) from the PREDICTED Y while the recorded Y
%        is the corrected one, so the identity is
%            YSTAR - Y  ==  VC*tgo*SIGNOISE*g  -  .5*H^2*ydd
%        with the second term the O(H^2) gap the header describes.  Subtract
%        the noise term the invariant claims and what is left must BE that gap,
%        which is bounded by .5*H^2*max|ydd|.  Measured ratio 0.83-1.00 over
%        23 parameter variants (H, TS, TF, TAU, W, VC, APN 0-4, MVR, SIGRIN,
%        seed); the 1.5 is headroom, not a fitted band.  Any of the wiring
%        faults above leaves a residue of order the MEASUREMENT -- tens of
%        feet, not 1e-3 -- so the clause has ~4 orders of discrimination.
%    (b) STATISTICAL, over the samples where the measurement still carries
%        noise: tgo > H, at least one integration step of flight remaining.
%        TS > 2H is enforced above, so this can exclude at most one sample.
%        This is the clause that would catch a scale error common to both
%        sides of (a) -- e.g. VC wrong in the run AND in the recorded tgo.
rtm  = opts.VC * nom.tgo;
sig  = sqrt(opts.SIGRIN^2 + (opts.SIGGL./rtm).^2 + (opts.SRN*rtm.*rtm/(opts.RA^2)).^2);
den  = rtm .* sig;                                  % VC*tgo*SIGNOISE, per sample
resid = (nom.YSTAR - nom.Y) - den .* nom.gdraw;     % must be the trapezoid gap
trunc = 0.5*opts.H^2 * max(abs(nom.YTDD - nom.XNL));
okA   = max(abs(resid)) <= 1.5*trunc + 1e-12;
keep  = nom.tgo > opts.H;
sdz   = sqrt(mean(((nom.YSTAR(keep) - nom.Y(keep))./den(keep)).^2));
okB   = any(keep) && abs(sdz - 1) <= 0.20;
[~, jc] = min(nom.tgo);                             % the closest-in sample
zc = (nom.YSTAR(jc) - nom.Y(jc))/den(jc);
names{end+1} = 'measurement noise scale == VC*tgo*SIGNOISE';
oks(end+1)   = okA && okB;
detail{end+1} = sprintf([ ...
    'exact, all %d samples: max |(YSTAR-Y) - VC*tgo*SIGNOISE*g| = %.3e ft vs ' ...
    'the trapezoid''s own .5 H^2 max|ydd| = %.3e ft (band 1.5x) -> %s.  ' ...
    'rms of normalised residual = %.4f over the %d samples with tgo > H ' ...
    '(want 1 +/- 0.20, sampling error ~%.1f%%) -> %s.  ' ...
    'Excluded from the rms: %d sample(s) at the listing''s tgo = +1e-6 epsilon, ' ...
    'where the two sides read %.3e ft (residual) against %.3e ft ' ...
    '(VC*tgo*SIGNOISE), i.e. |z| = %.1f from a collapsed denominator'], ...
    nom.n, max(abs(resid)), trunc, vstr(okA), ...
    sdz, sum(keep), 100/sqrt(2*max(sum(keep),1)), vstr(okB), ...
    sum(~keep), nom.YSTAR(jc)-nom.Y(jc), den(jc), abs(zc));

% 7  ⭐⭐ THE RECORDED COVARIANCE IS THE ONE THE MODEL PRESCRIBES.  Added
%    2026-08-24, and it exists because of a measured hole, not a hunch.
%
%    The comment on the Riccati block says the covariance "depends on TGO and
%    the model ONLY, never on a measurement".  That is a strong claim and
%    nothing tested it: the recursion could be re-run from OUTSIDE the loop,
%    from Fk, Qk and the recorded tgo alone, and must reproduce Pdiag and GAIN
%    sample for sample.  Until this check existed it did not have to.
%
%    THE HOLE, measured by injection: deleting Q from `M = PHI*Pm*PHIT + Q`
%    leaves containment at 0.705 / 0.706, benchmark 6 GREEN, every other row
%    green, and R.pass = 1.  Shrinking Phis by 100x does the same (0.755 /
%    0.741).  A four-state filter with NO PROCESS NOISE passed this entire
%    port.  It fails here, because Q enters the recomputation and the
%    residual is then Q itself, not a rounding.
%
%    ⚠️ WHAT THIS CANNOT DO, so nobody quotes it for more than it is: it
%    reuses the SAME formulas, so it cannot catch a wrong formula.  That job
%    belongs to checks 1-3, which rebuild Fk from expm and Qk from quadrature
%    by unrelated routes.  This one catches tampering with the recursion as it
%    RUNS -- a dropped term, a scaled P, gains that did not come from this M.
%    The band is 1e-12 because the two paths do identical arithmetic in
%    identical order: the measured residual is at the last bit, not near an
%    edge, so the band is headroom and not a tolerance fitted to a result.
VCq  = opts.VC;
RTM0 = VCq*opts.TF;
sig0 = sqrt(opts.SIGRIN^2 + (opts.SIGGL/RTM0)^2 + (opts.SRN*RTM0*RTM0/(opts.RA^2))^2);
Pc = zeros(4);
Pc(1,1) = (RTM0*sig0)^2;
Pc(2,2) = (opts.VM*opts.HEDEGFIL/57.3)^2;
Pc(3,3) = opts.XNT*opts.XNT;
Pc(4,4) = W*W*opts.XNT*opts.XNT;
HMq = [1 0 0 0];  HTq = HMq.';  PTq = PHI.';  I4 = eye(4);
nq = numel(nom.tgo);
Pref = zeros(nq,4);  Gref = zeros(nq,4);
for k = 1:nq
    rtmk = VCq*nom.tgo(k);
    sigk = sqrt(opts.SIGRIN^2 + (opts.SIGGL/rtmk)^2 + (opts.SRN*rtmk*rtmk/(opts.RA^2))^2);
    s2k  = (rtmk*sigk)^2;
    Mk   = PHI*Pc*PTq + R.q;                 % R.q is the Q ACTUALLY used
    Gk   = (Mk*HTq)/(HMq*Mk*HTq + s2k);
    Pc   = (I4 - Gk*HMq)*Mk;
    Pref(k,:) = diag(Pc).';  Gref(k,:) = Gk.';
end
% Normalised PER COLUMN by that column's own peak, not pointwise: P11 falls by
% ten orders over the flight and a pointwise relative error at the last sample
% would divide a rounding by a smaller rounding and invent a failure.
sP = max(abs(Pref), [], 1);  sG = max(abs(Gref), [], 1);
dP = max(max(abs(Pref - nom.Pdiag) ./ max(sP, realmin)));
dG = max(max(abs(Gref - nom.GAIN)  ./ max(sG, realmin)));
K.ric_dP = dP;  K.ric_dG = dG;
names{end+1} = 'Pdiag and GAIN == Riccati re-run from Fk, Qk and tgo alone';
oks(end+1)   = dP <= 1e-12 && dG <= 1e-12;
detail{end+1} = sprintf(['max relative dP %.3e, dGAIN %.3e over %d updates ' ...
    '(band 1e-12; the two paths do identical arithmetic, so the measured ' ...
    'residual is 0 exactly, not merely small).  Independent of every ' ...
    'measurement, which is the claim being tested.  Injected: Q dropped from ' ...
    'the Riccati -> dP 0.33, dGAIN 1.16 (green in every bench row); Pm x1.05 ' ...
    '-> dP 242; Pm x4 -> dP 6.2e12'], dP, dG, nq);

% ── PHIS matches the book's printed formula ────────────────────────────────
% ⚠️⚠️ READ WHAT THIS DOES AND DOES NOT CATCH BEFORE TRUSTING IT.
%
%   The book prints the process-noise spectral density at
%   zarchan_decoded.txt:8757-8758 as   Fs = w^2 * nT_MAX^2 / tF,   where
%   T_MAX is a SUBSCRIPT on nT.  That subscript has already caused one
%   documented slip in this very file: a comment restated it as
%   "Fs = w^2 n^2 / T_MAX", which is a different formula.
%
%   ⭐ SO THIS IS A TRANSCRIPTION CHECK, NOT A PHYSICS CHECK.  It catches a
%   dropped or duplicated factor -- W*XNT*XNT/TF instead of W*W*XNT*XNT/TF,
%   or TF moved into the numerator.  It does NOT catch a deliberate scale
%   error, because it recomputes the same product.
%
% ⚠️⚠️ AND THAT SCALE ERROR IS A REAL, MEASURED BLIND SPOT OF THIS PORT.
%   Multiplying PHIS by a constant and re-running the whole file:
%
%       PHIS x     rms(z) accel   rms(z) jerk   containment   any row red?
%       -----     ------------   -----------   -----------   ------------
%        1/100        0.839          0.896         75.5%          NO
%        1/10         0.747          0.805         87.7%          NO
%         1  (nom)    0.621          0.589         92.6%           -
%         10          0.579          0.351         93.7%          NO
%        100          0.627          0.252         91.0%          NO
%
%   A HUNDREDFOLD error in the process-noise density leaves every row in this
%   port green.  The containment row cannot see it: its band is ">= 0.68 and
%   < 1", and 0.755 and 0.910 both sit inside.
%
%   ⚠️ Why no band was added to close this.  rms(z) would separate 1/100
%   (0.839) and 100 (jerk 0.252) from the legitimate spread measured over
%   eight variants (seed 7/99/2024, TF 8/12, SIGRIN 5e-4, TS 0.05, runs 8):
%   accel 0.529-0.650, jerk 0.392-0.753.  But the margin to the nearest fault
%   is 3.5% on the jerk ceiling, off EIGHT samples.  A band with that margin
%   drawn from that few points is an invented edge, and this suite has spent
%   several waves removing those.  ⇒ The hole is DOCUMENTED WITH ITS NUMBERS
%   rather than papered over.  Closing it properly needs a reference for what
%   P *should* be, and this configuration does not have one: the filter is
%   conservative even at nominal (rms(z) 0.62, not 1), which is the same
%   phenomenon mgs.zarchan.kalmanHomingLoop documents for PHIN.
phis_book = opts.W^2 * opts.XNT^2 / opts.TF;
K.phis_rel = abs(PHIS - phis_book) / max(abs(phis_book), realmin);
names{end+1} = 'PHIS == w^2 nT_MAX^2 / tF, the book''s printed formula';
oks(end+1)   = K.phis_rel <= 1e-12;
detail{end+1} = sprintf(['relative %.3e (band 1e-12).  ⚠️ TRANSCRIPTION check ' ...
    'only: it recomputes the same product, so it catches a dropped factor ' ...
    '(W*XNT*XNT/TF -> 1.0 relative) but NOT a scale error.  A PHIS scaled by ' ...
    '1/100 or 100 leaves every row in this port green -- see the measured ' ...
    'table above the check.'], K.phis_rel);

K.dPhi = dPhi;  K.q_rel = worst;  K.q11_rel = r11;  K.dprop = nom.dprop;
K.minP = mnP;   K.noise_sd = sdz;
K.noise_resid = max(abs(resid));  K.noise_trunc = trunc;
K.noise_n = sum(keep);  K.terminal_z = zc;  K.terminal_den = den(jc);
K.terminal_resid = nom.YSTAR(jc) - nom.Y(jc);
K.names = names;  K.oks = logical(oks);  K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    j = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{j}, detail{j});
end
end


% =============================================================================
function B = benchmarks(R, opts)
%BENCHMARKS  Computed vs published, with the PROVENANCE of every band.
%
%   *** EVERY BOOK VALUE WAS GREPPED OUT OF zarchan.txt AND THE LINE NUMBER IS
%   IN THE .source FIELD. ***  Nothing was read off a figure: the extraction
%   turned every figure into an image link.  Where the book states a SHAPE
%   about a figure, that prose IS greppable, and an ordering check is usually
%   stronger than a plotted value.
%
%   Chapter 25 publishes almost no numbers for this listing -- four figures,
%   one parameter table, and four prose claims.  Nine scored rows is what is
%   actually there.  Do NOT widen a band to turn a row green.
B = emptyBench();
W = opts.W;  TAU = opts.TAU;

% ---- 1, 2.  The target the book says it flew.  Stated twice, independently:
%      in prose at :8841 and in Table 25.1 at :8955/:8956.
B = addVal(B, 'target maneuver amplitude', 3.0, opts.XNTREAL/32.2, 'text', 0.005, ...
    'p.616 text + Table 25.1 (zarchan.txt:8841, :8955)', 'g', ...
    'XNTREAL = 96.6 ft/s^2 on listing line 1 (:8876); 96.6/32.2 = 3 exactly');
B = addVal(B, 'target weave frequency', 2.0, opts.WREAL, 'text', 0.005, ...
    'p.616 text + Table 25.1 (zarchan.txt:8841, :8956)', 'rad/s', ...
    'WREAL on listing line 1; the FILTER''s WH is scored separately by row 9');

% ---- 3.  Fk against the SAME matrix printed as prose seven pages earlier.
%      Two independent transcriptions of one matrix; they must be identical.
Pb = bookPhi(W, opts.TS);
dP = max(max(abs(R.phi - Pb)));
B = addBound(B, 'Fk == the printed discrete fundamental matrix', ...
    'identical, elementwise', sprintf('max |dFk| = %.3e', dP), dP <= 1e-14, ...
    'p.615 matrix equation (zarchan.txt:8766-8772)', '-', ...
    ['the listing (:8881) and the derivation (:8766) are separate ' ...
     'transcriptions of one matrix; this is the only row that compares them']);

% ---- 4.  The APN = 4 branch against the printed compensated-weave law.
%      This is the law the project cares about (see weaveCompensated).
[dLaw, sLaw] = lawResidual(W, TAU);   % N' is XNPP here, NOT XNP -- see detail
B = addBound(B, 'APN=4 coefficients == printed nc(weave,lag)', ...
    'identical for every tgo', sprintf('max relative %.3e over %s', dLaw, sLaw), ...
    dLaw <= 1e-10, 'p.626 equation (zarchan.txt:8939-8946)', '-', ...
    ['C1..C5 in the listing vs the closed form: C3 = N''(1-cos w tgo)/' ...
     '(w tgo)^2, C5 = N''(w tgo - sin w tgo)/(w^3 tgo^2), C4 = -N''(e^-x+x-1)/x^2. ' ...
     'APN=3 is the same law with N'' fixed at XNP and no C4 term']);

% ---- 5.  N' -> 3 for large tgo.  A LIMIT, evaluated far out.
B = addVal(B, 'effective navigation ratio at tgo/TAU = 1e4', 3.0, navRatio(1e4), ...
    'text', 0.001, 'p.626 text (zarchan.txt:8938)', '-', ...
    sprintf(['"the effective navigation ratio approaches 3" as tgo grows, ' ...
             'and "increases to a larger number" as the missile closes: ' ...
             'N''(1) = %.2f, N''(10) = %.3f, N''(100) = %.4f'], ...
            navRatio(1), navRatio(10), navRatio(100)));

% ---- 6.  THE ONE CLAIM THE CHAPTER MAKES ABOUT FILTER CONSISTENCY. ---------
%      :8851 -- errors in the estimates of target acceleration and jerk "lie
%      within the theoretical bounds (that is, +/- square root of P33 and P44,
%      respectively) approximately 68% of the time".
%
%      ⭐⭐⭐ RE-EDGED 2026-08-24 AFTER FAULT INJECTION.  The band was 45-99%,
%      and BOTH edges were ours.  The sibling kalmanHomingLoop scored the
%      identical sentence with an invented 0.995 upper edge that could not
%      fail in the direction of its own disagreement, and that row has now
%      been deleted there; the question put to this one was whether 45-99% is
%      the same defect wearing the same clothes.
%
%      IT IS NOT, AND THE ANSWER IS MEASURED.  Each fault was applied to this
%      source and rerun (containment accel / jerk):
%        Pm x4          1.000 / 1.000  RED     Pm x100      1.000 / 1.000  RED
%        Pm /4          0.003 / 0.022  RED     P33,P44 /9   0.008 / 0.011  RED
%        gains dropped
%          from the state update
%                       0.514 / 0.415  RED (jerk only, at the OLD 0.45 floor)
%        Q dropped from the Riccati
%                       0.705 / 0.706  green -- and the WHOLE PORT stayed
%                                      green (R.pass = 1).  Now caught by
%                                      self-check 7, added for this.
%        Phis /100      0.755 / 0.741  green -- ⚠️ STILL GREEN, and still
%                                      green everywhere else in the port
%                                      (R.pass = 1).  See below.
%      So the old band did exclude all three faults the brief named.  What it
%      did NOT exclude was a filter with no process noise at all.
%
%      ⚠️⚠️ ONE HOLE IS LEFT OPEN ON PURPOSE, AND NAMED RATHER THAN PAPERED
%      OVER.  Shrinking Phis by 100x passes every scored row in this port,
%      including self-check 7 -- because check 7 recomputes the recursion from
%      the Q the port BUILT, so a Phis that is consistently wrong everywhere
%      is invisible to it, exactly as checks 1-3 are blind to it for the same
%      reason.  Closing it needs a row on Phis itself against the book's
%      Fs = w^2 nT_MAX^2 / tF at :8757 (the extraction reads "Fs = v2n2TMAX"
%      on :8757 with "tF" carried onto :8758; T_MAX is a SUBSCRIPT on the
%      target acceleration, not the denominator -- this line said
%      "w^2 n^2 / T_MAX" until 2026-08-24), which is a different row from this one
%      and is not smuggled in here.  Widening or narrowing THIS band would not
%      have touched it.
%
%      TWO EDGES CHANGED, AND NEITHER CHANGE WAS MADE TO MOVE A VERDICT --
%      this row is green at 0.926 / 0.926 before and after:
%        * FLOOR 0.45 -> 0.68.  0.45 came from nowhere; 0.68 is the book's own
%          number in the sentence being scored, and the port's own header
%          already said containment is expected AT OR ABOVE it.  This is a
%          TIGHTENING and it adds teeth: dropping the gains from the state
%          update now trips the ACCEL clause too (0.514), not the jerk clause
%          alone.
%        * CEILING 0.99 -> strictly < 1.  0.99 also came from nowhere, and it
%          was turning THIS row red on a documented call for no book reason:
%          weaveKalmanFilter('SIGRIN', 1e-4), the book's own .1 mr case at
%          :8852, gives jerk 0.9950.  `< 1` is not a chosen tolerance -- it is
%          the saturation boundary: a containment of exactly 1.000 over 5000
%          samples means the bound never once failed to contain, which no
%          covariance that predicts anything produces.
%      MEASURED COST OF THE CEILING CHANGE, stated rather than hoped: scaling
%      Pm by s gives accel/jerk 0.965/0.9998, 0.964/0.996, 0.969/0.994,
%      0.976/0.997, 0.995/0.9998, 1.000/1.000 at s = 1.05, 1.1, 1.25, 1.5, 2,
%      3, so this row now catches only s = 3 where 0.99 caught them all.  That
%      loss is COVERED, and not by luck: SELF-CHECK 7 recomputes the whole
%      covariance recursion from Fk, Qk and the recorded tgo and fails at
%      every one of those s to machine precision.  R.pass was already 0 at
%      each of them via benchmark 9 before check 7 existed.
fa = R.ens.frac_acc;  fj = R.ens.frac_jerk;
okC = fa >= 0.68 && fa < 1 && fj >= 0.68 && fj < 1;
B = addBound(B, 'estimate errors inside +/-sqrt(P33), +/-sqrt(P44)', ...
    '"approximately 68% of the time", both states; scored as >= 0.68 and < 1', ...
    sprintf('accel %.1f%%, jerk %.1f%% (%d samples, %d runs)', ...
            100*fa, 100*fj, R.ens.samples, R.ens.runs), okC, ...
    'p.617 text (zarchan.txt:8851)', '-', ...
    ['NOT 68 +/- something: the filter assumes a target that MIGHT start ' ...
     'weaving (Phis = w^2 n^2/tF) while the truth weaves deterministically, ' ...
     'so P is an upper bound and containment is expected ABOVE 68%.  The ' ...
     'floor is the book''s own number and the ceiling is the saturation ' ...
     'boundary; neither edge is a digit chosen to fit a measurement.  ' ...
     'MEASURED discrimination: Pm x4 -> 1.000, Pm /4 -> 0.003, P33/P44 /9 ' ...
     '-> 0.008, gains dropped from the state update -> 0.514 accel / 0.415 ' ...
     'jerk; all four RED.  ⚠️ MEASURED BLIND SPOTS, stated because a row''s ' ...
     'limits belong on the row: Q dropped from the Riccati reads 0.705 / ' ...
     '0.706 GREEN here (self-check 7 catches it), and Phis /100 reads 0.755 ' ...
     '/ 0.741 GREEN here AND green in every other row of this port -- that ' ...
     'one is still uncaught, and no edge on this row would catch it.  ' ...
     '⚠️ This row is scored on the TUNED filter ' ...
     'that :8851 describes.  A caller who mistunes it or stops the target ' ...
     'weaving is outside that sentence and should expect red: measured, ' ...
     '''W'',4 gives 0.686 / 0.818, ''W'',1 gives 0.281 / 0.147, ''MVR'',0 ' ...
     'gives 0.139 / 0.327']);

% ---- 7.  Reducing the noise to .1 mr improves BOTH estimates. --------------
okN = R.rms.low_acc < R.rms.nom_acc && R.rms.low_jerk < R.rms.nom_jerk;
B = addShape(B, '.1 mr improves the acceleration AND jerk estimates', ...
    'both estimate errors fall when SIGRIN drops by 10x', ...
    sprintf('accel rms %.4f -> %.4f g, jerk rms %.4f -> %.4f g/s', ...
            R.rms.nom_acc, R.rms.low_acc, R.rms.nom_jerk, R.rms.low_jerk), ...
    okN, 'p.617/618 text (zarchan.txt:8852, :8864)', ...
    ['same noise DRAWS in both runs, only the scale changes, so this is a ' ...
     'controlled experiment.  rms is used only to compare a run against ' ...
     'another run -- see the header on why it is not a quality metric']);

% ---- 8.  "nearly perfect in the low-noise environment" (:8864). ------------
%      Scored as amplitude and lag of the w component, because "nearly perfect"
%      about a sinusoid is a statement about BOTH.
F = R.fit.low;
ratio = F.amp/opts.XNTREAL;
okP = F.ok && abs(ratio - 1) <= 0.20 && abs(F.lag) <= 0.25;
B = addBound(B, 'at .1 mr the acceleration estimate is "nearly perfect"', ...
    'amplitude within 20% of 3 g and |lag| <= 0.25 s', ...
    sprintf('amplitude %.4g ft/s^2 (%.3f of truth), lag %+.3f s over %.1f-%.1f s', ...
            F.amp, ratio, F.lag, R.fit.window(1), R.fit.window(2)), okP, ...
    'p.618 text (zarchan.txt:8864)', '-', ...
    ['0.25 s is HALF the only lag figure the chapter publishes -- the ' ...
     'THREE-state filter''s "approximately a half-second lag" at :8621.  ' ...
     'A four-state filter that models the sinusoid exactly must beat that ' ...
     'comfortably or it is not doing what the chapter says it does']);

% ---- 9.  Mistuning the filter's weave frequency degrades it. ---------------
%      :8975 -- "when the estimated weave frequency is either twice as large or
%      half as small as the actual target weave frequency, significant
%      performance degradation may occur", and the same sentence names the
%      FILTER as one of the two things degraded.  This doubles as the delete
%      experiment: knowledge of w is the only thing this filter has that the
%      three-state one does not.
okM = R.rms.hi_acc > R.rms.nom_acc && R.rms.lo_acc > R.rms.nom_acc;
B = addShape(B, 'mistuned weave frequency (2x, 0.5x) degrades the filter', ...
    'both mistuned filters worse than the tuned one', ...
    sprintf('accel rms %.4f g tuned -> %.4f g at 2w (%.2fx), %.4f g at w/2 (%.2fx)', ...
            R.rms.nom_acc, R.rms.hi_acc, R.rms.hi_acc/R.rms.nom_acc, ...
            R.rms.lo_acc, R.rms.lo_acc/R.rms.nom_acc), okM, ...
    'p.629 text (zarchan.txt:8975; same experiment at :10015)', ...
    ['identical noise draws; Fk, Qk and Phis all retune with WH, as they do ' ...
     'in the book.  Ordering only -- "significant" is an adjective, and the ' ...
     'ratios are reported rather than adjudicated']);

% ---- SKIP.  The one place the book prints a miss number for this listing. --
B = addSkip(B, 'max rms miss 5 ft (w known) / 20 ft (w low by 1 r/s)', ...
    '5 ft and 20 ft', ...
    ['NOT SCOREABLE from the published listing: those figures come from ' ...
     '"a modified version of Listing 25.2" (:10015) flown as TWENTY-FIVE ' ...
     'Monte Carlo sets over flight times .5-10 s (:8949) WITH a 10 g ' ...
     'acceleration limit (:8963).  Listing 25.2 as printed has no ' ...
     'acceleration limit, no Monte Carlo loop and one flight time, and the ' ...
     'noise level behind Fig. 25.17 is not stated.  Three unpublished ' ...
     'inputs; widening a band to absorb them would be a fabrication'], ...
    'p.646 text (zarchan.txt:10015), Figs. 25.15-25.17', 'ft');

% ---- INFO.  Reported, never scored. ---------------------------------------
B = addInfo(B, 'the book''s ".333" in Qk(1,1)', ...
    'Fs/w^5 [.333 x^3 - 2 sin x + 2x cos x + .5x - .25 sin 2x]', ...
    sprintf('as printed %.4g ft^2; with exact 1/3 %.4g ft^2', R.q(1,1), R.q_exact(1,1)), ...
    'p.614 equation (zarchan.txt:8737-8738) and the listing (:8881)', ...
    ['NOT SCORED, but not a transcription error either: the bracket is ' ...
     'x^7/252 = 5.1e-15 at x = .02 while (1/3 - .333)x^3 = -2.7e-9, so the ' ...
     'printed rounding drives Qk(1,1) NEGATIVE and 5e5 times too large.  It ' ...
     'is inert -- M(1,1) never approaches 3e-7 ft^2 -- and it is kept.  ' ...
     '''q11_exact'', true switches it']);

% ---- INFO.  The terminal sample, kept ON THE BOARD. -----------------------
%      Structural check 6 used to fail here and the board showed only
%      "VIOLATED" with no number.  The check is now aimed at the quantity it
%      names, so the row is green -- and this INFO row is what stops the
%      measured number disappearing with it.
K = R.selfcheck;
B = addInfo(B, 'measurement scale at the listing''s tgo = +1e-6 epsilon', ...
    'no claim -- the epsilon exists to keep the divisions alive at T = TF', ...
    sprintf('residual %.3e ft vs VC*tgo*SIGNOISE = %.3e ft -> |z| = %.1f', ...
            K.terminal_resid, K.terminal_den, abs(K.terminal_z)), ...
    'listing line TGO = TF - T + .000001 (zarchan.txt:8893 and :8899)', ...
    ['NOT SCORED and NOT a defect.  At T = TF the measurement scale collapses ' ...
     'to microfeet while the trapezoid''s own O(H^2) predictor-corrector gap ' ...
     'stays put, so this one sample carries no information about the noise ' ...
     'scale and unbounded leverage on an rms of it.  It is the sample that ' ...
     'used to make structural check 6 read 5.42 against a band of 1 +/- 0.20.  ' ...
     'It is still checked EXACTLY there (check 6 clause a); what it is no ' ...
     'longer asked to do is carry a statistic it cannot inform.  The same ' ...
     'sample is why R.nom.XNC(end) is of order 1e15 and must not be plotted']);
end


% =============================================================================
function Pb = bookPhi(WH, TS)
%BOOKPHI  The fundamental matrix as PRINTED IN PROSE at zarchan.txt:8766-8772,
%   typed from the book rather than from the listing.  x = w*Ts.
x = WH*TS;
Pb = [1, TS, (1-cos(x))/WH^2, (x-sin(x))/WH^3
      0,  1, sin(x)/WH,       (1-cos(x))/WH^2
      0,  0, cos(x),          sin(x)/WH
      0,  0, -WH*sin(x),      cos(x)];
end


% =============================================================================
function [d, s] = lawResidual(WH, TAU)
%LAWRESIDUAL  The listing's APN = 4 branch against the printed closed form
%   nc = N'/tgo^2 [y + yd*tgo + (1-cos w tgo)/w^2 * yddT
%                    + (w tgo - sin w tgo)/w^3 * ydddT - nL*TAU^2*(e^-x+x-1)],
%   zarchan.txt:8939-8946, with N' from :8933 and x = tgo/TAU.
%
%   One arbitrary but fixed state vector, swept over tgo.  Normalised by the
%   sweep's peak, not pointwise: nc crosses zero and a pointwise relative error
%   there divides a small number by a smaller one and invents a failure.
tgo = linspace(0.05, 10, 400);
y = 100;  yd = -50;  a = 60;  j = 80;  nL = 20;
XS = tgo/TAU;  N = navRatio(XS);
listingNc = N./(tgo.*tgo)*y + N./tgo*yd ...
          + N.*(1-cos(WH*tgo))./(WH*WH*tgo.*tgo)*a ...
          - N.*(exp(-XS) + XS - 1)./(XS.*XS)*nL ...
          + N.*(WH*tgo - sin(WH*tgo))./(WH^3*tgo.*tgo)*j;
bookNc = N./(tgo.*tgo) .* ( y + yd*tgo + (1-cos(WH*tgo))/(WH*WH)*a ...
          + (WH*tgo - sin(WH*tgo))/(WH^3)*j - nL*TAU*TAU*(exp(-XS) + XS - 1) );
d = max(abs(listingNc - bookNc))/max(abs(bookNc));
s = sprintf('tgo = %.2f..%g s', tgo(1), tgo(end));
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want.  Schema is the
%   one benchSuite's shared benchRows() consumes.
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
L = {};  p = R.p;
L{end+1} = 'Zarchan Listing 25.2 -- FOUR-STATE WEAVE KALMAN FILTER AND WEAVING TARGET';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, ft/s^3, g = 32.2.  NOT SI. ***';
L{end+1} = sprintf(['   nT = %g ft/s^2 (%.4g g)   w_true = %g rad/s   w_filter = %g rad/s' ...
    '   TAU = %g s   TF = %g s'], p.XNTREAL, p.XNTREAL/32.2, p.WREAL, p.W, p.TAU, p.TF);
L{end+1} = sprintf(['   APN = %d   VC = %g ft/s   SIGRIN = %g rad (%.1f mr)   ' ...
    'TS = %g s   H = %g s   Phis = %.4g'], p.APN, p.VC, p.SIGRIN, 1000*p.SIGRIN, ...
    p.TS, p.H, p.PHIS);
L{end+1} = sprintf('   %d nominal run(s), seed %g; fit window %.2f-%.2f s', ...
    p.runs, p.seed, p.fit_window(1), p.fit_window(2));
L{end+1} = '';
L{end+1} = 'TARGET ACCELERATION AND ESTIMATE (the listing''s two datfil columns)';
L{end+1} = sprintf('%8s %10s %10s %10s %10s %10s', ...
    't [s]', 'yddT [g]', 'est [g]', 'jerk[g/s]', 'est[g/s]', 'sqrtP33[g]');
stride = max(1, round(1.0/p.TS));
for k = 1:stride:numel(R.nom.t)
    L{end+1} = sprintf('%8.2f %10.4f %10.4f %10.4f %10.4f %10.4f', ...
        R.nom.t(k), R.nom.YTDD(k)/32.2, R.nom.YTDDH(k)/32.2, ...
        R.nom.YTDDD(k)/32.2, R.nom.YTDDDH(k)/32.2, R.nom.SP33(k)/32.2); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = sprintf(['   fitted w-component of the estimate: 1 mr amp %.4g ft/s^2 ' ...
    'lag %+.3f s | .1 mr amp %.4g lag %+.3f s'], R.fit.nom.amp, R.fit.nom.lag, ...
    R.fit.low.amp, R.fit.low.lag);
L{end+1} = sprintf('   linearised miss (no accel limit): %+.3f ft at 1 mr, %+.3f ft at .1 mr', ...
    R.nom.miss, R.low.miss);
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-46s %s', vstr(K.oks(i)), K.names{i}, K.detail{i}); %#ok<AGROW>
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
                L{end+1} = sprintf('   %-4s %-50s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf('   %-4s %-50s book %s %s / ours %s %s (%+.3f%%, band %.4g%%)', ...
                    vstr(b.ok), b.what, b.book_str, b.units, b.ours_str, b.units, ...
                    b.pct, 100*b.tol); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped/info -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: the weave frequency is KNOWN here.  Listing 25.3 is the same';
L{end+1} = '   engagement with w as a fifth state, and :8975 is the price of getting';
L{end+1} = '   w wrong.  Our own weave completes 0.21 of a period, so nothing in this';
L{end+1} = '   file transfers to it without saying so out loud.';
s = strjoin(L, newline);
end

function s = vstr(tf)
if tf, s = 'PASS'; else, s = 'FAIL'; end
end


% =============================================================================
function drawCurves(R)
%DRAWCURVES  The listing's own four figures, base MATLAB graphics only.
figure; grid on; hold on;
plot(R.nom.t, R.nom.YTDD/32.2, '-', R.nom.t, R.nom.YTDDH/32.2, '--');
xlabel('Time (Sec)'); ylabel('Acceleration and Estimate (G)');
legend('target', 'estimate', 'Location','best'); hold off;

figure; grid on; hold on;
plot(R.nom.t, R.nom.YTDDD/32.2, '-', R.nom.t, R.nom.YTDDDH/32.2, '--');
xlabel('Time (Sec)'); ylabel('Jerk and Estimate (G/S)');
legend('target', 'estimate', 'Location','best'); hold off;

figure; grid on; hold on;
plot(R.nom.t, (R.nom.YTDD - R.nom.YTDDH)/32.2, '-', ...
     R.nom.t,  R.nom.SP33/32.2, '--', R.nom.t, -R.nom.SP33/32.2, '--');
xlabel('Time (Sec)'); ylabel('Error in Estimate of Acceleration (G)');
legend('error', '+sqrt(P33)', '-sqrt(P33)', 'Location','best'); hold off;

figure; grid on; hold on;
plot(R.nom.t, (R.nom.YTDDD - R.nom.YTDDDH)/32.2, '-', ...
     R.nom.t,  R.nom.SP44/32.2, '--', R.nom.t, -R.nom.SP44/32.2, '--');
xlabel('Time (Sec)'); ylabel('Error in Estimate of Jerk (G/S)');
legend('error', '+sqrt(P44)', '-sqrt(P44)', 'Location','best'); hold off;
end
