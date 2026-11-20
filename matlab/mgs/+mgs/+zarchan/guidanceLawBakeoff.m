function R = guidanceLawBakeoff(opts)
%MGS.ZARCHAN.GUIDANCELAWBAKEOFF  Zarchan Listing 26.6 -- four guidance laws
%   flown head to head against the same cubic flight control system.
%
%   R = mgs.zarchan.guidanceLawBakeoff()
%   R = mgs.zarchan.guidanceLawBakeoff('wz', 5, 'laws', [0 3])
%   R = mgs.zarchan.guidanceLawBakeoff('assert', false, 'render', false)
%
%   Line-by-line port of Listing 26.6 (zarchan.txt:9856-9892), including the
%   GENERATEGAINS routine inside it (:9885-9892) that solves the 6-state matrix
%   Riccati equation for the cubic guidance law's gains.
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, g = 32.2 ft/s^2; W and WZ in
%   rad/s; seconds.  NOT SI.  The rest of mgs is metres and nothing is
%   converted here, so that every number printed lies straight against a
%   published one.  Convert AT THE CALL SITE and say so there.
% ===========================================================================
%
%   WHAT IT IS.  One linearised homing loop, one disturbance (a 0.5-g target
%   maneuver, on from t = 0), perfect state knowledge, no noise -- and four
%   guidance laws selected by APN (:9854): 0 proportional navigation, 1
%   augmented PN, 2 optimal for a SINGLE-LAG flight control system, 3 optimal
%   for the CUBIC one (gains from GENERATEGAINS).  The plant is the same in all
%   four cases: the cubic transfer function printed at :9841-9847,
%
%       nL/nc = (1 - s^2/WZ^2) / [ (1 + TAU s)(1 + 2 Z s/W + s^2/W^2) ]
%
%   realised as EDDD = W^2(XNC - E - (2Z/W+TAU)ED - (2Z TAU/W + 1/W^2)EDD)/TAU
%   with XNL = E - EDD/WZ^2.  Expanding that denominator reproduces the three
%   coefficients exactly, which is how the OCR mush at :9843-9847 was read.  WZ
%   is a RIGHT-half-plane zero (1 - s^2/WZ^2 = (1-s/WZ)(1+s/WZ)) -- the
%   "wrong-way tail effect" of :9901.
%
%   WHY IT SWEEPS WZ AND NOT ONLY THE LAWS.  The book runs the listing three
%   times, at WZ = 100, 10 and 5 rad/s, and the ORDERING BETWEEN THE LAWS
%   REVERSES.  At WZ = 100 the single-lag optimal law is "far superior" to PN
%   (:9893); at WZ = 5 it is "much worse than proportional navigation" (:9901)
%   while the cubic law is still "nearly perfect" (:9909).  A law that is
%   optimal for the WRONG plant model loses to plain PN.  Orderings, not miss
%   numbers, are what this chapter publishes and what this port scores.
%
%   *** WHAT IT IS NOT. ***  Linearised, planar, constant closing velocity,
%   PERFECT state feedback (laws 2 and 3 read the achieved acceleration and the
%   flight-control system's internal states; law 1 is handed the true target
%   maneuver), no noise, no estimator, no gravity, drag, thrust or LOS floor,
%   and no acceleration limit that binds.  It prices nothing of ours directly.
%   What it answers cleanly is the one question we have: what does a guidance
%   law lose when its internal model of the airframe is wrong -- our measured
%   gamma-tracking gain is 0.063 against a model 1.385.
%
% ---------------------------------------------------------------------------
%   TRANSCRIPTION.  listing_26_06.m does not parse.  Repairs:
%     1  "while T , (TF-.00001)" -> "<=" (both loops).  The OCR renders "<" as
%        "," throughout this book; T only increases, so ">=" never terminates.
%     2  "while STEP , =1" -> "<=".  Both two-pass loops are written here as
%        "for FLAG = 0:1" -- the same control flow with the book's STEP/FLAG
%        bookkeeping removed, not reinterpreted.
%     3  "if S .=(TS-.0001)" -> ">=".  "." is the OCR of ">"; S resets to 0 on
%        every sample, so "<=" would sample forever.
%     4  ⚠️ THE ONE UNRESOLVED VARIABLE-TO-VARIABLE COMPARISON the extractor
%        flagged: "if XNC.XLIM, XNC=XLIM; elseif XNC < -XLIM, XNC=-XLIM".
%        Resolved as XNC > XLIM.  Forced: the elseif arm is the symmetric lower
%        branch, so the pair is a saturation, and "<" would make the first arm
%        an inverse limiter clamping everything EXCEPT large commands.
%     5  the plotting tail and "save datfil.txt output /ascii" (the book's own
%        typo for -ascii) are dropped; nothing is written to disk.
%   DEAD IN THE LISTING, so absent here and not worth hunting for: VC = 4000
%   and VM = 3000 are never used (VC feeds RTM -> XLAM, which is never read),
%   so the miss does not depend on closing velocity at all; and XNCG = XNC/32.2
%   is computed twice per step and never read.
%
%   TWO THINGS THAT LOOK LIKE BUGS AND ARE NOT (both checked by arithmetic):
%     * "if S >= TS - 0.0001" with S += H.  At the book's H = 1e-4 the sum of
%       99 increments is 0.009899999999999995, three ULP BELOW 0.0099, so the
%       sample fires at 100 steps -- exactly the intended TS.  The epsilon is
%       sized to absorb that drift without firing a step early, and any H that
%       divides 0.01 gives the same 0.01 s interval: H is not secretly the
%       guidance sample rate.
%     * The gain table holds round(TFMAX/TS) = 1000 entries and JJ =
%       fix(TGO/TS)+1 reaches exactly 1000 (the first guidance sample is at
%       T = TS, not at 0).  It fits EXACTLY, so a one-step floating-point
%       difference in the gain loop's termination would turn it into an index
%       error.  This port integrates the Riccati TWO SAMPLES PAST TFMAX for
%       that reason and makes "no lookup past entry 1000" a structural row.
%       The bias it does not fix: JJ rounds tgo UP one grid step -- 0.2% at
%       tgo = 5 s, a factor 2 at the last sample.
%
%   WHAT THIS PORT CHANGES.  The book's "for TF=.1:.1:10" is 100 independent
%   runs, so the whole (flight time x law x WZ) ensemble is ONE time loop with
%   a per-element active mask, on a 24-point default grid dense below 2 s where
%   the book's short-flight-time claim lives ('tf', 0.1:0.1:10 restores the
%   book's own).  H, gain_H and step_H are separate: the Riccati genuinely
%   needs 1e-4 (:9356, and Listing 26.3 exists because this approach is 100x
%   slower than a 0.01 s step, :9609); a step-response shape check does not.
%   COST at the defaults, MEASURED (R2025b, 2026-08-24, and no longer the
%   arithmetic guess that used to stand here): 0.74 s, median of three, for the
%   benchSuite call; 1.17 s on the book's own 'tf', 0.1:0.1:10 grid.  That buys
%   3 Riccati sweeps of 1e5 RK2 steps on a 6x6 (1002 gain rows recorded each)
%   plus 1e5 ensemble steps carrying 288 elements.  The knobs are 'wz' (one
%   Riccati sweep each) and 'gain_H'; cutting flight times saves almost nothing,
%   because the ensemble loop runs to max(tf) whatever the grid between.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  ⚠️ CH.26 PUBLISHES NO MISS NUMBER FOR THIS LISTING.  Figs.
%   26.19-26.21 ARE the results, and every figure became an image link in the
%   extraction.  What the chapter publishes is orderings, in prose, and prose
%   about a figure is greppable -- so: ONE value row (the 0.5-g maneuver level,
%   :9854) and TEN shape/bound rows, each carrying its own sentence and line
%   number in .source.  ⚠️ EVERY THRESHOLD IN THOSE TEN IS MINE.  The book's
%   words are adjectives; each row states in .band what it turned the adjective
%   into --
%       "far superior" / "nearly perfect"   peak |miss| ratio <= 0.25 / <= 0.05
%       "much worse"                        peak |miss| ratio >= 3
%       "not significant" / "significant"   ⚠️ NO LONGER A NUMBER.  Rows 8 and
%                                           11 used to be a matched pair of
%                                           ABSOLUTE bands (|ratio - 1| <= 0.25
%                                           and ratio >= 1.25) on PN's peak
%                                           |miss|.  Row 8 failed at 1.6053 and
%                                           was the only row on this board that
%                                           disagreed with its chapter.  It was
%                                           the BAND that was wrong, and it is
%                                           gone: both rows are now the
%                                           orderings the prose actually states.
%                                           The measurements that forced that,
%                                           including the retired 1.6053, are in
%                                           the block comment above row 8 and
%                                           printed inside the row.
%   They are loose enough that only a real reordering trips them.  Do not widen
%   one to make a row green; if a row fails, ask which ordering moved -- and if
%   the answer is that the chapter never stated an ordering there, delete the
%   number rather than tuning it, which is what happened to rows 8 and 11.
%   One row carries an OCR caveat: :9894 prints "flight times less than 25",
%   which cannot be a flight time in a sweep ending at 10 s (TFMAX = 10 at
%   :9861).  It is "2 s" with the space lost, and the row is written as an
%   ordering so that 2 versus 2.5 cannot change its verdict.
%   NOT SCORED: the peak-miss table itself (nothing published to score it
%   against), augmented PN (this chapter never discusses it), and the XLIM fire
%   count -- which is NOT an invariant, because at XLIM = 9999999 the limiter
%   still fires on the last sample of nearly every run, where TGO is 1e-4 and
%   the command is applied for zero remaining flight time.
%
%   WHAT THIS PRINTS.  *** MEASURED 2026-08-24 on R2025b, replacing the
%   derived-from-the-prose paragraph that used to stand here.  Two of its
%   numbers were wrong. ***  Peak |miss| [ft] at WZ = 100 / 10 / 5, on the
%   default 24-point grid:
%       PN         8.251  13.24   55.67        APN        4.691  7.541  31.83
%       opt-1lag   0.2890 34.26   2.097e+05    opt-cubic  0.2734 0.3476 0.2609
%   On the book's own 'tf', 0.1:0.1:10 the peaks are 8.351 / 13.41 / 56.41 for
%   PN, all three at tF = 2.2 s.  So: cubic ~ 0 everywhere; single-lag optimal
%   ~ 0 at WZ = 100, wrecked at WZ = 10 for short flight times, and 2.1e5 ft at
%   WZ = 5 (the caption at :9917 says "goes unstable") -- finite but enormous is
%   the book's result, not an overflow, because XLIM bounds the command.  NaN
%   anywhere is a fault and is a structural row.
%   ⚠️ THE TWO CORRECTIONS.  (1) PN is NOT "roughly unchanged 100 -> 10": its
%   peak rises by a factor 1.606, +61%, and that is what rows 8 and 11 had to be
%   rewritten around.  (2) the WZ = 5 step response dips to -0.3467, not to the
%   "about -0.24" this header used to assert from an area identity; the analytic
%   partial-fraction step response of the printed transfer function gives
%   -0.346769 and the port's RK2 gives -0.346731, so the -0.24 was simply wrong,
%   not a step-size effect.
%
%   OUTPUTS.  R.miss [nTF x nLaw x nWZ] signed miss [ft]; R.tf .laws .wz the
%   grids in that index order; R.peak .med [4 x 3] by law code 0..3 and by WZ
%   in the book's order [100 10 5], NaN where not run; R.step the Fig.
%   26.17/26.18 step responses; R.gains one 6-by-N table per WZ; R.bench
%   R.selfcheck R.pass R.verdict R.p R.summary R.units.  'assert' true errors
%   on a failed row, false WARNS and returns R COMPLETE -- which is how
%   benchSuite runs it, because a port that refuses to return is a port nobody
%   can diagnose.
%
%   See also MGS.ZARCHAN.ADJOINTOPTIMAL (Ch.8 optimal guidance, and the App.
%   A.1 result on what a tgo scale error costs an optimal law),
%   MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 26.6 lines 1-2, verbatim (zarchan.txt:9856, :9861).
    opts.TAU    (1,1) double {mustBeFinite, mustBePositive} = 1.0
    opts.W      (1,1) double {mustBeFinite, mustBePositive} = 20.0
    opts.Z      (1,1) double {mustBeFinite, mustBePositive} = 0.7
    opts.XNT    (1,1) double {mustBeFinite} = 16.1      % 0.5 g, ft/s^2
    opts.TS     (1,1) double {mustBeFinite, mustBePositive} = 0.01
    opts.GAM    (1,1) double {mustBeFinite, mustBePositive} = 0.00001
    opts.XLIM   (1,1) double {mustBeFinite, mustBePositive} = 9999999.0
    opts.TFMAX  (1,1) double {mustBeFinite, mustBePositive} = 10.0
    opts.XNP    (1,1) double {mustBeFinite, mustBePositive} = 3.0
    opts.H      (1,1) double {mustBeFinite, mustBePositive} = 0.0001
    % The Riccati step.  The book's, and it is not decoration -- see :9356.
    opts.gain_H (1,1) double {mustBeFinite, mustBePositive} = 0.0001
    % The step-response check only.  Deliberately NOT tied to H: it is a shape
    % check on a plant whose fastest mode is W = 20 rad/s, and 1e-3 is 300
    % steps per period of it.  H is set by the guidance endgame, not the plant.
    opts.step_H (1,1) double {mustBeFinite, mustBePositive} = 0.001
    % [] means the book's own three cases / grid / law set.
    opts.wz     double {mustBeReal, mustBeFinite} = []   % rad/s, [100 10 5]
    opts.tf     double {mustBeReal, mustBeFinite} = []   % s
    opts.laws   double {mustBeReal, mustBeFinite} = []   % subset of 0:3
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so an "if nargin == 0" here would be a mode that never fires.
    opts.assert    (1,1) logical = true
end

TAU = opts.TAU;  W = opts.W;  Z = opts.Z;  XNT = opts.XNT;
TS  = opts.TS;   GAM = opts.GAM;  XLIM = opts.XLIM;
TFMAX = opts.TFMAX;  XNP = opts.XNP;  H = opts.H;

wz = opts.wz(:).';
if isempty(wz), wz = [100 10 5]; end        % the book's three cases
tf = opts.tf(:).';
if isempty(tf), tf = [0.25:0.25:2, 2.5:0.5:TFMAX]; end
laws = opts.laws(:).';
if isempty(laws), laws = [0 1 2 3]; end

if any(wz <= 0)
    error('mgs:zarchan:guidanceLawBakeoff:wz', 'wz must be positive rad/s.');
end
if numel(unique(wz)) ~= numel(wz)
    error('mgs:zarchan:guidanceLawBakeoff:wzdup', ...
        'wz has repeated values; each one costs a full Riccati sweep.');
end
if isempty(tf) || any(tf <= 0) || any(tf > TFMAX + 1e-12)
    error('mgs:zarchan:guidanceLawBakeoff:tf', ...
        ['tf must be non-empty and lie in (0, TFMAX], and TFMAX = %g s.  The ' ...
         'cubic law''s gain table is generated out to TFMAX only, so a longer ' ...
         'flight time would silently reuse the last stored gain set.'], TFMAX);
end
if ~all(ismember(laws, 0:3))
    error('mgs:zarchan:guidanceLawBakeoff:laws', ...
        'laws must be a subset of [0 1 2 3]; see the header for what each is.');
end
laws = unique(laws, 'stable');
if H >= TS || opts.gain_H >= TS
    error('mgs:zarchan:guidanceLawBakeoff:step', ...
        ['H = %g s and gain_H = %g s must both be SMALLER than the guidance ' ...
         'update interval TS = %g s.  At or above it the sample test fires ' ...
         'every step and the zero-order hold disappears -- silently.'], ...
        H, opts.gain_H, TS);
end
if H >= TAU/10
    error('mgs:zarchan:guidanceLawBakeoff:stiff', ...
        ['H = %g s is not small against TAU = %g s.  RK2 does not blow up ' ...
         'here, it returns a smooth and entirely wrong curve.'], H, TAU);
end

% =========================================================================
%   GENERATEGAINS (zarchan.txt:9885-9892), once per WZ.  Only if a run needs
%   it: the Riccati sweep is the dominant cost of this file.
% =========================================================================
nW = numel(wz);
gains  = cell(1, nW);
nrec   = zeros(1, nW);
symres = nan(1, nW);
needGains = any(laws == 3);
for k = 1:nW
    if needGains
        % Two samples past TFMAX -- see the EXACT FIT note in the header.
        [gains{k}, nrec(k), symres(k)] = generateGains(TAU, W, Z, wz(k), ...
            GAM, TFMAX + 2*TS, TS, opts.gain_H);
        if nrec(k) < 1
            error('mgs:zarchan:guidanceLawBakeoff:nogains', ...
                ['the Riccati sweep at WZ = %g recorded no gain rows, so the ' ...
                 'cubic law has no table to read.'], wz(k));
        end
    else
        gains{k} = zeros(6,0);
    end
end
% round(), not floor(): 10/0.01 evaluates to 999.99999999999998 in binary
% floating point, and floor() would silently declare the book's own last table
% entry out of range.
jjMax = round(TFMAX/TS);      % largest index a book-legal run may produce

% =========================================================================
%   Figs. 26.17 / 26.18: the step response of the transcribed plant.  E, ED
%   and EDD do not depend on WZ (it enters only the readout XNL = E -
%   EDD/WZ^2), so one scalar integration serves every case.
% =========================================================================
[stepT, stepE, stepEDD] = fcsStep(TAU, W, Z, opts.step_H, 8*TAU);
stepXNL   = stepE - stepEDD*(1./(wz.^2));         % [nt x nW], implicit expand
singleLag = 1 - exp(-stepT/TAU);
stepDev   = max(abs(stepXNL - singleLag), [], 1); % [1 x nW]
stepMin   = min(stepXNL, [], 1);
stepEnd   = stepXNL(end, :);

% =========================================================================
%   LISTING 26.6's own loop, over the whole ensemble at once.
% =========================================================================
[TFg, LAWg]  = ndgrid(tf(:), laws(:));
[~, ~, WZig] = ndgrid(tf(:), laws(:), 1:nW);
TFv  = repmat(TFg(:),  nW, 1);
LAWv = repmat(LAWg(:), nW, 1);
WZiv = WZig(:);
[missV, nClampXNC, nClampJJ, nSteps] = sweepMisses(TFv, LAWv, WZiv, wz(:), ...
    gains, jjMax, TAU, W, Z, XNT, XNP, TS, XLIM, H, TFMAX);
miss = reshape(missV, numel(tf), numel(laws), nW);

% ---- the 4 x 3 summary, indexed by LAW CODE and by the book's WZ order ----
wzRef = [100 10 5];
PK = nan(4,3);  MD = nan(4,3);
for j = 1:numel(laws)
    for k = 1:nW
        c = find(wzRef == wz(k), 1);
        if isempty(c), continue; end
        a = abs(miss(:,j,k));
        PK(laws(j)+1, c) = max(a);
        MD(laws(j)+1, c) = median(a);
    end
end
% Row 5 needs the short/long split of the single-lag optimal law at WZ = 10.
short2 = NaN;  long2 = NaN;
j2 = find(laws == 2, 1);  k10 = find(wz == 10, 1);
if ~isempty(j2) && ~isempty(k10)
    a = abs(miss(:,j2,k10));
    if any(tf < 2),  short2 = max(a(tf < 2));  end
    if any(tf >= 2), long2  = max(a(tf >= 2)); end
end
% Step-response statistics in the book's WZ order, NaN where not run.
SDV = nan(1,3);  SMN = nan(1,3);
for k = 1:nW
    c = find(wzRef == wz(k), 1);
    if ~isempty(c), SDV(c) = stepDev(k);  SMN(c) = stepMin(k);  end
end

R = struct();
R.tf   = tf(:).';
R.laws = laws(:).';
R.wz   = wz(:).';
R.miss = miss;
R.peak = PK;
R.med  = MD;
R.step = struct('t', stepT, 'xnl', stepXNL, 'single_lag', singleLag, ...
                'dev', stepDev, 'min_xnl', stepMin, 'final_xnl', stepEnd);
R.gains = gains;
R.units = 'ft, ft/s, ft/s^2 (g = 32.2 ft/s^2); s; W and WZ in rad/s';
R.p = struct('TAU',TAU, 'W',W, 'Z',Z, 'XNT',XNT, 'XNT_g',XNT/32.2, 'TS',TS, ...
    'GAM',GAM, 'XLIM',XLIM, 'TFMAX',TFMAX, 'XNP',XNP, 'H',H, ...
    'gain_H',opts.gain_H, 'step_H',opts.step_H, 'n_gain_rows',nrec, ...
    'n_steps',nSteps, 'n_xnc_clamped',nClampXNC, 'n_jj_clamped',nClampJJ, ...
    'units',R.units);

R.selfcheck = structuralChecks(miss, stepEnd, symres, nClampJJ, needGains);

if opts.benchmark
    R.bench = benchmarks(PK, MD, SDV, SMN, short2, long2, XNT, tf, wz, laws);
    ev = ~[R.bench.skipped];
    % A pass needs at least one row that actually EXERCISED the sweep.  Row 1
    % compares two of the listing's own constants and would score green on a
    % run whose WZ cases are all off-book -- every ordering row then SKIPs, and
    % "1 scored, 10 skipped" must not read as a validated port.
    sim = ev & ~strcmpi({R.bench.kind}, 'value');
    R.pass = any(sim) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev = false;  sim = false;
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine" -- that is the shape of green light this repo has been burned
    % by more than once.
    R.pass = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R); end

% ---- adjudication.  The verdict is FORMED either way; only the reaction to
%      it depends on 'assert'.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(sim)
    msg = ['not one ORDERING row could be evaluated: every one of them needs a ' ...
           '(law, WZ) pair the book actually ran, and only the parameter row ' ...
           'scored.  Check ''laws'' and ''wz'' -- the book''s cases are WZ = ' ...
           '100, 10 and 5 rad/s with laws 0..3.'];
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port ' ...
        'disagrees with ITSELF, which is a wiring fault, not a tolerance ' ...
        'question.  First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf(['%d of %d evaluated book claims are outside tolerance ' ...
        '(first: %s, want %s, ours %s).'], numel(bad), sum(ev), ...
        R.bench(bad(1)).what, R.bench(bad(1)).book_str, R.bench(bad(1)).ours_str);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:guidanceLawBakeoff:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN.  R comes back complete so the caller can see WHICH
        % side of a comparison moved.
        warning('mgs:zarchan:guidanceLawBakeoff:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [Cg, nrec, symResid] = generateGains(TAU, W, Z, WZ, GAM, TFg, TS, Hg)
%GENERATEGAINS  zarchan.txt:9885-9892.  Numerically integrates the 6-state
%   matrix Riccati equation forward in TIME-TO-GO and stores -C(1,:) every TS.
%
%   The state ordering is fixed by the book's F: x = [Y, YD, XNT, E, ED, EDD].
%   Check it against the plant -- it is the only way to know the gain vector is
%   being applied to the right things back in the simulation:
%       F(1,2)=1                            -> Y'  = YD
%       F(2,3)=1, F(2,4)=-1, F(2,6)=1/WZ^2  -> YD' = XNT - (E - EDD/WZ^2)
%       F(4,5)=1, F(5,6)=1                  -> E' = ED, ED' = EDD
%       F(6,4:6), G(6,1)=W^2/TAU            -> the cubic FCS denominator
%   and the simulation's XNC = C1*Y + C2*YD + C3*XNT + C4*E + C5*ED + C6*EDD
%   is that same ordering.  The sign convention is u = -R^-1 B' S x, so the
%   listing's Ci = -C(1,i) makes XNC = sum(Ci*xi).
%
%   Every constant is an ARGUMENT.  MATLAB local functions cannot see the main
%   function's workspace, and a previous wave in this package lost a run to
%   exactly that.
F = zeros(6,6);
Smat = zeros(6,6);
G = zeros(6,1);
G(6,1) = W*W/TAU;
F(1,2) = 1;  F(2,3) = 1;  F(2,4) = -1;  F(2,6) = 1/WZ^2;
F(4,5) = 1;  F(5,6) = 1;
F(6,4) = -W*W/TAU;
F(6,5) = -W*W*(2*Z/W + TAU)/TAU;
F(6,6) = -W*W*(1/W^2 + 2*Z*TAU/W)/TAU;
Smat(1,1) = 1;               % terminal weight on the miss, integrated in tgo

GAMINV = 1/GAM;
T  = 0;
S1 = 0;
SD = zeros(6,6);
C  = zeros(1,6);
Cg = zeros(6, ceil(TFg/TS) + 8);
nrec = 0;
it = 0;
itmax = ceil(TFg/Hg) + 16;

while T <= TFg - 0.0001
    it = it + 1;
    if it > itmax
        error('mgs:zarchan:guidanceLawBakeoff:riccati', ...
            'the Riccati loop did not terminate after %d steps.', itmax);
    end
    S1 = S1 + Hg;
    SOLD = Smat;
    for FLAG = 0:1                       % the book's STEP/FLAG two-pass loop
        if FLAG == 1
            Smat = Smat + Hg*SD;         % HSD = H*SD; S = S + HSD
            T = T + Hg;
        end
        SF = Smat*F;
        C  = GAMINV*(G.'*Smat);          % GTS = G'*S; C = GAMINV*GTS   (1x6)
        SD = SF + SF.' - GAM*(C.'*C);    % SFSFT - CTBC
    end
    Smat = 0.5*(SOLD + Smat) + 0.5*Hg*SD;
    if S1 >= TS - 0.0001
        S1 = 0;
        nrec = nrec + 1;
        % The listing stores the SECOND-pass C, i.e. the one evaluated at the
        % predicted S rather than at the corrected S.  Kept.
        Cg(:, nrec) = -C(1,:).';
    end
end
Cg = Cg(:, 1:nrec);
% S must stay symmetric: SD = SF + SF' - C'C is symmetric by construction, so
% any asymmetry is an integration or indexing fault, not a tolerance question.
symResid = max(max(abs(Smat - Smat.'))) / max(1, max(max(abs(Smat))));
end


% =============================================================================
function [t, Ev, EDDv] = fcsStep(TAU, W, Z, Hs, Tend)
%FCSSTEP  Unit step into the transcribed cubic flight control system, same
%   Heun scheme as the simulation.  Returns E and EDD; the achieved
%   acceleration is XNL = E - EDD/WZ^2, and WZ enters ONLY there -- which is
%   why one integration covers every WZ case.
n = max(2, round(Tend/Hs));
t    = (0:n).' * Hs;
Ev   = zeros(n+1, 1);
EDDv = zeros(n+1, 1);
k0 = W*W/TAU;
k1 = 2*Z/W + TAU;
k2 = 2*Z*TAU/W + 1/W^2;
E = 0;  ED = 0;  EDD = 0;
for i = 1:n
    EOLD = E;  EDOLD = ED;  EDDOLD = EDD;
    EDDD = k0*(1 - E - k1*ED - k2*EDD);
    E   = E   + Hs*ED;
    ED  = ED  + Hs*EDD;
    EDD = EDD + Hs*EDDD;
    EDDD = k0*(1 - E - k1*ED - k2*EDD);
    E   = 0.5*(EOLD   + E   + Hs*ED);
    ED  = 0.5*(EDOLD  + ED  + Hs*EDD);
    EDD = 0.5*(EDDOLD + EDD + Hs*EDDD);
    Ev(i+1)   = E;
    EDDv(i+1) = EDD;
end
end


% =============================================================================
function [Y, nClampXNC, nClampJJ, nSteps] = sweepMisses(TFv, LAWv, WZiv, wz, ...
    gains, jjMax, TAU, W, Z, XNT, XNP, TS, XLIM, H, TFMAX)
%SWEEPMISSES  Listing 26.6's two nested loops, with the outer loop over flight
%   time turned into an array dimension and the law and WZ cases carried
%   alongside it.  Every element integrates its OWN flight; an element whose
%   flight has ended is frozen by a mask, and its Y is the miss.
%
%   The frozen elements are excluded from the guidance update rather than
%   multiplied by zero.  A frozen element's TGO is within one step of zero, so
%   its PN command goes as 1/TGO^2 and can be enormous; enormous times zero is
%   a number, but if it ever reached Inf it would be NaN, and a NaN in a state
%   vector spreads to nothing at all and is then indistinguishable from a
%   modelling result.  Masking the guidance means the arithmetic is not done.
%
%   The corrector for a frozen element is 0.5*(x + x + 0), which returns it
%   bit-for-bit: the predictor could not have moved it.

nE  = numel(TFv);
wz2 = wz(WZiv).^2;
wz2 = wz2(:);

E = zeros(nE,1);  ED = zeros(nE,1);  EDD = zeros(nE,1);
Y = zeros(nE,1);  YD = zeros(nE,1);  XNC = zeros(nE,1);
T = zeros(nE,1);
S = 0;

k0 = W*W/TAU;
k1 = 2*Z/W + TAU;
k2 = 2*Z*TAU/W + 1/W^2;

isL0 = (LAWv == 0);  isL1 = (LAWv == 1);
isL2 = (LAWv == 2);  isL3 = (LAWv == 3);
nW = numel(wz);
isL3k = cell(1, nW);
for k = 1:nW, isL3k{k} = isL3 & (WZiv == k); end

nClampXNC = 0;  nClampJJ = 0;  nSteps = 0;
itmax = ceil(TFMAX/H) + 16;

while true
    act = (T <= TFv - 0.00001);          % the listing's own loop condition
    if ~any(act), break; end
    nSteps = nSteps + 1;
    if nSteps > itmax
        error('mgs:zarchan:guidanceLawBakeoff:sweep', ...
            'the flight loop did not terminate after %d steps.', itmax);
    end
    hA = H*double(act);
    S = S + H;
    EOLD = E;  EDOLD = ED;  EDDOLD = EDD;  YOLD = Y;  YDOLD = YD;

    % ---- pass 1: derivatives at the current state -------------------------
    EDDD = k0*(XNC - E - k1*ED - k2*EDD);
    YDD  = XNT - (E - EDD./wz2);
    % ---- predictor (the listing's FLAG==1 branch) -------------------------
    E   = E   + hA.*ED;
    ED  = ED  + hA.*EDD;
    EDD = EDD + hA.*EDDD;
    Y   = Y   + hA.*YD;
    YD  = YD  + hA.*YDD;
    T   = T   + hA;
    % ---- pass 2: derivatives at the predicted state -----------------------
    EDDD = k0*(XNC - E - k1*ED - k2*EDD);
    XNL  = E - EDD./wz2;
    YDD  = XNT - XNL;
    % ---- corrector, in the listing's own algebra --------------------------
    E   = 0.5*(EOLD   + E   + hA.*ED);
    ED  = 0.5*(EDOLD  + ED  + hA.*EDD);
    EDD = 0.5*(EDDOLD + EDD + hA.*EDDD);
    Y   = 0.5*(YOLD   + Y   + hA.*YD);
    YD  = 0.5*(YDOLD  + YD  + hA.*YDD);

    % ---- the guidance update, every TS ------------------------------------
    if S >= TS - 0.0001
        S = 0;
        % TGO as the second derivative pass left it: T has already advanced.
        TGO = TFv - T + 0.0001;

        m = isL0 & act;                                     % APN = 0, PN
        if any(m)
            g = TGO(m);
            XNC(m) = XNP*(Y(m) + YD(m).*g)./(g.*g);
        end
        m = isL1 & act;                                     % APN = 1
        if any(m)
            g = TGO(m);
            XNC(m) = XNP*(Y(m) + YD(m).*g + 0.5*XNT*g.*g)./(g.*g);
        end
        m = isL2 & act;                                     % APN = 2
        if any(m)
            g   = TGO(m);
            XS  = g/TAU;
            EX  = exp(-XS);
            TOP  = 6.*XS.*XS.*(EX - 1 + XS);
            BOT1 = 2*XS.^3 + 3 + 6.*XS - 6.*XS.*XS;
            BOT2 = -12.*XS.*EX - 3.*exp(-2.*XS);
            % The 0.0001 in the denominator is the book's (:9356 explains the
            % same device in Listing 26.1).  It is load-bearing for a second
            % reason: BOT1+BOT2 cancels to O(XS^5) out of terms of O(1), so
            % below XS ~ 0.01 it is pure rounding noise.  The 0.0001 then
            % dominates and the effective navigation ratio collapses to zero
            % near intercept instead of being garbage.
            XNPP = TOP./(0.0001 + BOT1 + BOT2);
            C1P = XNPP./(g.*g);
            C2P = XNPP./g;
            C3P = 0.5*XNPP;
            C4P = -XNPP.*(EX + XS - 1)./(XS.*XS);
            XNC(m) = C1P.*Y(m) + C2P.*YD(m) + C3P*XNT + C4P.*XNL(m);
        end
        for k = 1:nW                                        % APN = 3
            m = isL3k{k} & act;
            if ~any(m), continue; end
            Cg = gains{k};
            JJ = fix(TGO(m)/TS) + 1;
            nClampJJ = nClampJJ + sum(JJ > jjMax);
            JJ = min(max(JJ, 1), size(Cg, 2));
            c = Cg(:, JJ);                                  % 6 x numel(JJ)
            XNC(m) = c(1,:).'.*Y(m) + c(2,:).'.*YD(m) + c(3,:).'*XNT ...
                   + c(4,:).'.*E(m) + c(5,:).'.*ED(m) + c(6,:).'.*EDD(m);
        end

        % The limiter, with the unresolved comparison resolved as XNC > XLIM
        % (header repair 4).  COUNTED, NOT ASSERTED: at XLIM = 9999999 it
        % still fires on the last sample of nearly every run, where TGO is
        % 1e-4 and the command is applied for zero remaining flight time.
        hot = act & (abs(XNC) > XLIM);
        nClampXNC = nClampXNC + sum(hot);
        XNC = min(max(XNC, -XLIM), XLIM);
    end
end
end


% =============================================================================
function K = structuralChecks(miss, stepEnd, symres, nClampJJ, needGains)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.  They are the weaker
%   claim that it agrees with itself -- which is what catches wiring faults.
K = struct();
names = {};  oks = [];  detail = {};

% 1  No NaN or Inf anywhere.  The WZ=5 single-lag case is EXPECTED to be huge
%    (the book calls it unstable); XLIM is what keeps huge from becoming Inf.
bad = sum(~isfinite(miss(:)));
mx  = max(abs(miss(isfinite(miss))));
if isempty(mx), mx = NaN; end
names{end+1} = 'every miss is finite';
oks(end+1)   = (bad == 0);
detail{end+1} = sprintf('%d of %d non-finite; max |miss| = %.4g ft', ...
    bad, numel(miss), mx);

% 2  DC gain of the transcribed flight control system is 1.  That is what the
%    printed transfer function says (numerator and denominator both -> 1 at
%    s = 0) and it is the cheapest thing that would notice a sign error in
%    XNL = E - EDD/WZ^2 or a mistyped denominator coefficient.  At 8 TAU the
%    residual exp(-8) = 3.4e-4 is still present, hence the 5e-3 band.
dcerr = max(abs(stepEnd - 1));
names{end+1} = 'FCS step response settles to 1 (DC gain)';
oks(end+1)   = (dcerr <= 5e-3);
detail{end+1} = sprintf('max |XNL(8 TAU) - 1| = %.3e', dcerr);

if needGains
    % 3  The Riccati solution stays symmetric.
    sr = max(symres(isfinite(symres)));
    if isempty(sr), sr = NaN; end
    names{end+1} = 'Riccati S stays symmetric';
    oks(end+1)   = (sr <= 1e-9);
    detail{end+1} = sprintf('max relative asymmetry %.3e', sr);

    % 4  The gain-table index never ran past the book's own last entry.  See
    %    the EXACT FIT note in the header: at the book's settings the largest
    %    index the simulation can produce equals the table length exactly.
    names{end+1} = 'gain-table index within the book''s table';
    oks(end+1)   = (nClampJJ == 0);
    detail{end+1} = sprintf('%d lookup(s) past entry round(TFMAX/TS)', nClampJJ);
end

K.names  = names;
K.oks    = logical(oks);
K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    i = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', K.names{i}, K.detail{i});
end
end


% =============================================================================
function B = benchmarks(PK, MD, SDV, SMN, short2, long2, XNT, tf, wz, laws)
%BENCHMARKS  Computed vs published.  EVERY book value here was grepped out of
%   zarchan.txt and the line number is in the .source field.  Chapter 26
%   publishes exactly ONE number for this listing (the maneuver level); the
%   rest of what it publishes is orderings, in prose, about figures that did
%   not survive extraction.  Thresholds are named in the header and repeated
%   in each row's .band / .detail.
B = emptyBench();
wzName = {'WZ=100', 'WZ=10', 'WZ=5'};
c100 = 1;  c10 = 2;  c5 = 3;            % columns of PK/MD, book's WZ order
PN = 1;  APN = 2;  OPT1 = 3;  CUB = 4;  % rows of PK/MD = law code + 1

% NOISE.  Rows 8 and 11 compare two ratios, and a BARE ">" between them is not a
% guard -- it is satisfied by rounding dust.  MEASURED 2026-08-24: run this port
% with 'wz', [1e8 1e7 1e6], where the term EDD/WZ^2 is inert to machine
% precision, and PN's three peaks come back 8.2115492088280586,
% ...8319826, ...92202746 -- equal to 12 figures but not bit-identical, so the
% two ratios differ by 4.7e-11 and a bare ">" REPORTS PASS ON A PLANT THAT HAS
% NO ZERO IN IT.  This floor is 1e-5 times the smallest real margin either row
% carries (2.62x on row 11, 100x on row 8) and 1e5 times the measured dust, so
% it can separate the two without being a threshold fitted to the answer.  It is
% a numerical floor, NOT a reading of any adjective in the chapter.
NOISE = 1e-6;

% ---- 1.  the only published number -----------------------------------------
B = addVal(B, 'target maneuver XNT / g', 0.5, XNT/32.2, 'text', 1e-6, ...
    'p.682 prose "a 0.5-g target maneuver" (zarchan.txt:9854); XNT=16.1 at :9861', ...
    'g', 'the listing''s only disturbance; 16.1/32.2 = 0.5 exactly');

% ---- 2.  the wrong-way tail effect -----------------------------------------
if isfinite(SMN(c5)) && isfinite(SMN(c100))
    ok = (SMN(c5) <= -0.05) && (SMN(c100) >= -0.02);
    B = addShape(B, 'step response goes wrong-way at WZ=5, not at WZ=100', ...
        'min XNL <= -0.05 at WZ=5 and >= -0.02 at WZ=100', ...
        sprintf('min XNL = %.4f (WZ=5), %.4f (WZ=100)', SMN(c5), SMN(c100)), ok, ...
        'p.682 "the significant wrong-way tail effect" (zarchan.txt:9901); Fig. 26.17 caption :9849', ...
        ['the RHP zero 1-s^2/WZ^2 is the subject of the whole chapter; a sign ' ...
         'slip in XNL = E - EDD/WZ^2 deletes it and nothing else notices']);
else
    B = addSkip(B, 'step response goes wrong-way at WZ=5, not at WZ=100', ...
        'undershoot present at 5, absent at 100', ...
        'needs both WZ = 5 and WZ = 100 in ''wz''', ...
        'zarchan.txt:9901; Fig. 26.17 caption :9849', '-');
end

% ---- 3.  "virtually identical" at WZ = 100 ---------------------------------
if isfinite(SDV(c100)) && isfinite(SDV(c5)) && SDV(c5) > 0
    ok = SDV(c100) <= 0.25*SDV(c5);
    B = addBound(B, 'WZ=100 step response matches the single lag', ...
        '<= 0.25 x the WZ=5 deviation', ...
        sprintf('%.4f vs %.4f (ratio %.3f)', SDV(c100), SDV(c5), SDV(c100)/SDV(c5)), ...
        ok, 'p.683 "virtually identical" (zarchan.txt:9853); Fig. 26.18 caption :9857', ...
        '-', ['max |XNL - (1-exp(-t/TAU))| over 8 TAU; the 0.25 is this port''s ' ...
              'reading of "virtually identical" against "totally different" (:9848)']);
else
    B = addSkip(B, 'WZ=100 step response matches the single lag', ...
        'virtually identical', 'needs both WZ = 100 and WZ = 5', ...
        'zarchan.txt:9853', '-');
end

% ---- 4.  WZ = 100: optimal single-lag far superior to PN --------------------
B = ratioRow(B, PK, OPT1, c100, PN, c100, 0.25, 'below', ...
    'WZ=100: single-lag optimal far superior to PN', ...
    'p.686 "very good and far superior to the performance of proportional navigation" (zarchan.txt:9893); Fig. 26.19 caption :9902', ...
    'peak |miss| ratio; "far superior" read as at most a quarter of PN''s worst miss');

% ---- 5.  WZ = 10: the single-lag law fails at SHORT flight times ------------
if isfinite(short2) && isfinite(long2)
    ok = short2 > long2;
    B = addShape(B, 'WZ=10: single-lag optimal is worst at SHORT flight times', ...
        'max|miss| over tF < 2 s exceeds max|miss| over tF >= 2 s', ...
        sprintf('%.4g ft (tF<2) vs %.4g ft (tF>=2)', short2, long2), ok, ...
        'p.686 "deteriorates significantly for flight times less than 2 s" (zarchan.txt:9894; the OCR prints "25")', ...
        ['written as an ordering so that it does not depend on whether the ' ...
         'lost character makes the threshold 2 or 2.5 s']);
else
    B = addSkip(B, 'WZ=10: single-lag optimal is worst at SHORT flight times', ...
        'short flight times are the worst', ...
        'needs law 2 at WZ = 10 and flight times on both sides of 2 s', ...
        'zarchan.txt:9894', 'ft');
end

% ---- 6.  WZ = 10: the cubic law is "always zero" ----------------------------
B = ratioRow(B, PK, CUB, c10, PN, c10, 0.05, 'below', ...
    'WZ=10: cubic optimal miss is essentially zero', ...
    'p.686 "the miss distance is always zero" (zarchan.txt:9894); Fig. 26.20 caption :9903', ...
    ['scored against PN''s peak rather than against 0 ft: the gain table is ' ...
     'quantised to TS and indexed one grid step high, so exact zero is not reachable']);

% ---- 7.  WZ = 10: PN generally worse than both ------------------------------
if all(isfinite([MD(PN,c10) MD(OPT1,c10) MD(CUB,c10)]))
    ok = (MD(PN,c10) > MD(OPT1,c10)) && (MD(PN,c10) > MD(CUB,c10));
    B = addShape(B, 'WZ=10: PN worse than BOTH optimal laws', ...
        'median |miss| of PN above both', ...
        sprintf('PN %.4g, single-lag %.4g, cubic %.4g ft', ...
            MD(PN,c10), MD(OPT1,c10), MD(CUB,c10)), ok, ...
        'p.687 "the proportional-navigation results are generally worse than that of both other guidance laws" (zarchan.txt:9900)', ...
        ['"generally" scored on the median, not the peak: the single-lag law ' ...
         'beats PN at most flight times here but not at the shortest ones (row 5)']);
else
    B = addSkip(B, 'WZ=10: PN worse than BOTH optimal laws', ...
        'PN median above both', 'needs laws 0, 2 and 3 at WZ = 10', ...
        'zarchan.txt:9900', 'ft');
end

% ---- 8.  first half of the pair on PN's sensitivity to the zero -------------
%   ⚠️⚠️ REWRITTEN 2026-08-24, AND THE OLD BAND IS GONE ON PURPOSE.  What stood
%   here scored the book's "does not significantly influence" as an ABSOLUTE
%   two-sided band, |peak ratio - 1| <= 0.25.  It measured 1.6053 and was the
%   only row on this board that disagreed with its chapter.  Four measurements
%   killed each of the three ways that could have happened:
%     (a) NOT a port fault.  PN's miss agrees with an independent ode45
%         (RelTol 1e-10, same zero-order hold at TS) to 5e-6 relative or better
%         at every WZ -- 8.35122 / 13.40928 / 56.41401 ft against this port's
%         8.35122 / 13.40927 / 56.41374, worst case 4.8e-6 at WZ = 5, which is
%         the port's H = 1e-4 and not a disagreement about the model.  The
%         cubic plant agrees with the analytic
%         partial-fraction step response of the printed transfer function to
%         five decimals at all three WZ (min XNL -0.000296 / -0.082601 /
%         -0.346769).  Poles -1 and -14 +/- 14.2829i, as printed.
%     (b) NOT the statistic, and not the summary of it.  Peak 1.606, median
%         1.563, rms 1.600, and the ratio at EVERY one of the book's own 100
%         flight times lies in [1.180, 1.606] -- nothing about the miss lands
%         inside 25%.  Augmented PN moves 1.609 over the same change, so this
%         is the PLANT's sensitivity to its own numerator zero, not PN's.
%     (c) NOT the step size.  1.6053 at H = 1e-4, 1.6058 at 5e-5, 1.6098 at
%         2e-5: it converges the wrong way for a numerical artefact.
%     (d) AND NO ABSOLUTE THRESHOLD EXISTS TO BE TUNED TO.  PN's peak |miss| is
%         a smooth monotone function of WZ with log-log slope -0.02 at WZ = 100,
%         -0.32 at 20, -1.21 at 10, -2.29 at 7, -4.27 at 5, diverging near
%         WZ ~ 3 (peak 8.31 ft at WZ = 1000 -> 8.35 at 100 -> 13.4 at 10 ->
%         56.4 at 5 -> 1516 at 3).  There is no knee between 10 and 5 for a wall
%         to stand on; the old 25% wall was placed where the physics has none.
%   THE CHAPTER IS NOT WRONG.  It calibrates its own adjective inside this one
%   paragraph: the same 100 -> 10 decrease that it calls "not significant" for
%   PN (x1.606) it calls "deteriorates significantly" for the single-lag optimal
%   law (x161.1 on the book's own 0.1:0.1:10 grid, x118.6 on this port's default
%   24-point one).  Both readings sit on one consistent scale -- the author's.
%   Nothing here says the chapter miscounted; it says an unquantified adjective
%   was turned into a two-sided number that the chapter never wrote down.  The
%   band was this port's invention, and the text supports the COMPARISON it
%   actually draws instead, which is also this file's own stated principle at
%   the top: "Orderings, not miss numbers, are what this chapter publishes and
%   what this port scores."  So the row is scored as that ordering, and the
%   retired 1.6053 is PRINTED INSIDE IT rather than deleted.
if all(isfinite([PK(PN,c100) PK(PN,c10) PK(OPT1,c100) PK(OPT1,c10)])) ...
        && PK(PN,c100) > 0 && PK(OPT1,c100) > 0
    rPN  = PK(PN,c10)/PK(PN,c100);
    rOPT = PK(OPT1,c10)/PK(OPT1,c100);
    B = addShape(B, 'WZ 100->10 moves PN far less than the single-lag optimal', ...
        'PN''s peak-|miss| ratio below the single-lag law''s over the same change', ...
        sprintf('PN x%.4g vs opt-1lag x%.4g  (PN peak %.4g -> %.4g ft, %+.0f%%)', ...
            rPN, rOPT, PK(PN,c100), PK(PN,c10), 100*(rPN-1)), ...
        rOPT > rPN*(1 + NOISE), ...
        'p.686-687 "deteriorates significantly ... The decrease of the right-half-plane zero does not significantly influence the" / "proportional-navigation results" (zarchan.txt:9894 running into :9900)', ...
        ['the contrast that sentence pair draws, scored as the ordering it is. ' ...
         'REPORTED, NOT SCORED, because the chapter never quantifies it: the ' ...
         'literal absolute reading of "not significantly" DOES NOT HOLD -- PN''s ' ...
         'peak rises 61% at the defaults (the live figure is printed above), ' ...
         'converged (1.6053 at H=1e-4, 1.6098 at 2e-5), with every flight time ' ...
         'in the book''s own 0.1:0.1:10 grid moving by ' ...
         '1.180-1.606x.  See the block comment above this row for why no band ' ...
         'replaces it: on the same grid the author calls the single-lag law''s ' ...
         'x161 "significantly" in the same breath as calling PN''s x1.61 "not ' ...
         'significant", and peak |miss| vs WZ is knee-free.  â ï¸â ï¸ THIS ROW''S ' ...
         'TEETH, RE-MEASURED ON THIS PORT 2026-08-24 (earlier text quoted an ' ...
         'ode45 replica and overstated them): removing the WZ term from BOTH ' ...
         'places it enters sweepMisses does fail the row -- both ratios come ' ...
         'back exactly 1.  But (a) removing it from the pass-2 XNL line ONLY ' ...
         'leaves this row GREEN at x1.267 vs x1.8, and (b) SLIPPING ITS SIGN ' ...
         'leaves this row GREEN at x0.6232 vs x0.7928 -- the ordering survives ' ...
         'even though every peak moved.  Row 9 catches (a) and row 11 catches ' ...
         '(b).  So this row alone is a WEAK detector; it is the pair with rows ' ...
         '9 and 11 that has teeth, and none of the three pins a magnitude.']);
else
    B = addSkip(B, 'WZ 100->10 moves PN far less than the single-lag optimal', ...
        'PN''s ratio below the single-lag law''s', ...
        'needs laws 0 AND 2 at both WZ = 100 and WZ = 10', ...
        'zarchan.txt:9894', '-');
end

% ---- 9.  WZ = 5: THE REVERSAL ----------------------------------------------
B = ratioRow(B, PK, OPT1, c5, PN, c5, 3.0, 'above', ...
    'WZ=5: single-lag optimal MUCH WORSE than PN', ...
    'p.687-688 "terrible and in fact is much worse than proportional navigation" (zarchan.txt:9901, continuing at :9909); Fig. 26.21 caption :9917 "goes unstable"', ...
    ['the ordering reversal against row 4 -- the chapter''s point: a law that ' ...
     'is optimal for the WRONG plant model loses to plain PN']);

% ---- 10.  WZ = 5: the cubic law still works ---------------------------------
B = ratioRow(B, PK, CUB, c5, PN, c5, 0.05, 'below', ...
    'WZ=5: cubic optimal still essentially zero', ...
    'p.688 "the cubic guidance law yields nearly perfect performance" (zarchan.txt:9909); Fig. 26.21 caption :9917', ...
    'this is the listing''s own nominal case (APN=3, WZ=5 at zarchan.txt:9861)');

% ---- 11.  second half of the pair, and the book's operative word is "NOW" ----
%   :9909 reads "The decrease in the right-half-plane zero to 5 r/s NOW
%   significantly affects the performance of proportional navigation" -- "now"
%   being explicitly against the 100 -> 10 case the chapter has just called not
%   significant.  That comparative is the whole testable content of the
%   sentence; the "peak ratio >= 1.25" that used to be here was, exactly like
%   row 8's 25%, a number this port invented and the chapter does not contain.
%   Scored as the comparison "now" makes, so the pair still lands on opposite
%   sides of something -- of EACH OTHER, which is what the chapter claims.  It
%   is a weaker check than the absolute pair it replaces and is labelled so; it
%   is not a weaker check than an arbitrary threshold that happens to be green.
if all(isfinite([PK(PN,c100) PK(PN,c10) PK(PN,c5)])) ...
        && PK(PN,c100) > 0 && PK(PN,c10) > 0
    rLo = PK(PN,c10)/PK(PN,c100);
    rHi = PK(PN,c5)/PK(PN,c10);
    B = addShape(B, 'PN moves MORE over 10 -> 5 than it did over 100 -> 10', ...
        'peak-|miss| ratio over 10->5 above the ratio over 100->10', ...
        sprintf('x%.4g (10->5) vs x%.4g (100->10); margin %.2fx', rHi, rLo, rHi/rLo), ...
        rHi > rLo*(1 + NOISE), ...
        'p.688 "The decrease in the right-half-plane zero to 5 r/s NOW significantly affects the performance of proportional navigation" (zarchan.txt:9909)', ...
        ['teeth CHECKED BY MUTATION ON THIS PORT, not on a replica and not ' ...
         'asserted (re-measured 2026-08-24; the earlier text quoted an ' ...
         'independent ode45 model and its numbers were 1% off this file''s): ' ...
         'baseline rLo 1.605 / rHi 4.203 PASS; remove the WZ term from BOTH ' ...
         'places it enters sweepMisses and the peaks are 8.212 ft at every WZ ' ...
         'so both ratios are 1.000 exactly, FAIL; slip its sign and the peaks ' ...
         'fall 8.172 -> 5.093 -> 1.221 ft, rLo 0.6232 / rHi 0.2398, FAIL.  ' ...
         'â ï¸ Removing it from the pass-2 XNL line ONLY does NOT fail this row ' ...
         '(rLo 1.267, rHi 2.049, margin 1.62x); row 9 is what catches that.  At ' ...
         'the defaults the margin is 2.62x and the two changes are +61% and ' ...
         '+321%; the LIVE margin is the number printed above, and it is the whole ' ...
         'of what the chapter asserts here.']);
else
    B = addSkip(B, 'PN moves MORE over 10 -> 5 than it did over 100 -> 10', ...
        'the 10->5 change is the larger of the two', ...
        'needs law 0 at ALL THREE of WZ = 100, 10 and 5 -- the claim is comparative', ...
        'zarchan.txt:9909', '-');
end

% ---- INFO / SKIP -----------------------------------------------------------
B = addInfo(B, 'peak |miss| by law and WZ [ft]', 'no published value', ...
    peakStr(PK), 'Figs. 26.19-26.21 (zarchan.txt:9902, :9903, :9917)', ...
    ['the bakeoff result itself.  NOT scored: Ch.26 gives no miss number in ' ...
     'prose, and every figure became an image link in the extraction, so there ' ...
     'is nothing to compare a number against -- only the orderings above.']);
B = addInfo(B, 'augmented PN (APN=1) peak |miss| [ft]', 'no claim in Ch.26', ...
    sprintf('%.4g / %.4g / %.4g at %s / %s / %s', PK(APN,1), PK(APN,2), ...
        PK(APN,3), wzName{1}, wzName{2}, wzName{3}), ...
    'the law is offered at zarchan.txt:9854 and coded at :9874', ...
    ['the listing implements it and this chapter never discusses it; augmented ' ...
     'PN is Chapter 8''s subject, and against this chapter''s plant it is ' ...
     'simply another entry in the table.']);
B = addSkip(B, 'miss distance at any particular flight time', ...
    'Figs. 26.19-26.21 (curves only)', ...
    ['not evaluable: Ch.26 prints no miss-distance value anywhere in prose, ' ...
     'and the figures did not survive extraction.  A number invented for this ' ...
     'row would be exactly the phantom benchmark this package exists to keep ' ...
     'off the board.'], ...
    'zarchan.txt:9893-9917', 'ft');
B = addInfo(B, 'grid actually run', '-', ...
    sprintf('%d flight times %.2f-%.2f s, laws [%s], WZ [%s] rad/s', ...
        numel(tf), min(tf), max(tf), num2str(laws), num2str(wz)), ...
    'the book''s own sweep is "for TF=.1:.1:10" (zarchan.txt:9862)', ...
    ['flight times are independent runs, so the grid is a resolution choice; ' ...
     'pass ''tf'', 0.1:0.1:10 for the book''s own figure grid']);
end


function B = ratioRow(B, PK, rNum, cNum, rDen, cDen, thr, side, what, src, detail)
%RATIOROW  One peak-|miss| ratio against a threshold, or a SKIP if either case
%   was not run.  Every "far superior" / "much worse" / "nearly perfect" row
%   goes through here so the ratio is formed the same way each time.
num = PK(rNum, cNum);  den = PK(rDen, cDen);
if ~isfinite(num) || ~isfinite(den) || den == 0
    B = addSkip(B, what, sprintf('peak ratio %s %g', side, thr), ...
        'the (law, WZ) pair this row compares was not part of this run', ...
        src, '-');
    return
end
r = num/den;
if strcmp(side, 'below')
    ok = r <= thr;
    want = sprintf('peak |miss| ratio <= %g', thr);
else
    ok = r >= thr;
    want = sprintf('peak |miss| ratio >= %g', thr);
end
B = addBound(B, what, want, sprintf('%.4g / %.4g ft = %.4g', num, den, r), ...
    ok, src, 'ft', detail);
end


function s = peakStr(PK)
lab = {'PN', 'APN', 'opt-1lag', 'opt-cubic'};
parts = cell(1,4);
for i = 1:4
    parts{i} = sprintf('%s %.3g/%.3g/%.3g', lab{i}, PK(i,1), PK(i,2), PK(i,3));
end
s = ['(WZ=100/10/5)  ' strjoin(parts, ' | ')];
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want: a struct array
%   whose fields reordered would make [B.ok] mean something else.  This is the
%   schema benchSuite's adaptGeneric consumes, and it is STRICT: a near-miss
%   (book_s for book_str) errors rather than being accommodated.
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
L = {};
p = R.p;
lab = {'PN', 'APN', 'opt-1lag', 'opt-cubic'};
L{end+1} = 'Zarchan Listing 26.6 -- SIMULATION FOR EVALUATING DIFFERENT GUIDANCE LAWS';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf('   TAU = %g s   W = %g r/s   Z = %g   XNT = %g ft/s^2 (%.3g g)   N'' = %g', ...
    p.TAU, p.W, p.Z, p.XNT, p.XNT_g, p.XNP);
L{end+1} = sprintf('   TS = %g s   GAM = %g   XLIM = %g ft/s^2   H = %g s   gain_H = %g s', ...
    p.TS, p.GAM, p.XLIM, p.H, p.gain_H);
L{end+1} = sprintf('   WZ cases: [%s] r/s   laws: [%s]   %d flight times %.2f-%.2f s', ...
    num2str(R.wz), num2str(R.laws), numel(R.tf), min(R.tf), max(R.tf));
L{end+1} = '';
L{end+1} = 'PEAK |MISS| OVER THE FLIGHT-TIME SWEEP [ft]   (median in brackets)';
L{end+1} = sprintf('%12s %20s %20s %20s', 'law', 'WZ=100', 'WZ=10', 'WZ=5');
for i = 1:4
    L{end+1} = sprintf('%12s %20s %20s %20s', lab{i}, ...
        numBr(R.peak(i,1), R.med(i,1)), numBr(R.peak(i,2), R.med(i,2)), ...
        numBr(R.peak(i,3), R.med(i,3))); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = sprintf('FCS STEP RESPONSE (Figs. 26.17/26.18) at WZ = [%s]:', num2str(R.wz));
L{end+1} = sprintf('   min XNL (wrong-way dip)          %s', mat2str(round(R.step.min_xnl, 4)));
L{end+1} = sprintf('   max deviation from 1-exp(-t/TAU) %s', mat2str(round(R.step.dev, 4)));
L{end+1} = sprintf('   XNC limiter fired on %d guidance samples (not an invariant -- see header)', ...
    p.n_xnc_clamped);
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
    L{end+1} = 'BENCHMARKS  (book claims grepped from zarchan.txt; line numbers in .source)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
            case 'info'
                L{end+1} = sprintf('   INFO %-52s %s', b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case 'skip'
                L{end+1} = sprintf('   SKIP %-52s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case {'shape','bound'}
                L{end+1} = sprintf('   %-4s %-52s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf('   %-4s %-52s book %s %s / ours %s %s (%+.3f%%, band %.4g%%)', ...
                    vstr(b.ok), b.what, b.book_str, b.units, b.ours_str, ...
                    b.units, b.pct, 100*b.tol); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped/info -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: perfect state feedback, no noise, no estimator, no acceleration';
L{end+1} = '   limit that binds.  What this chapter measures is the cost of a guidance';
L{end+1} = '   law whose internal model of the airframe is WRONG -- read the WZ=5 row of';
L{end+1} = '   opt-1lag as that cost, not as a miss distance for an aircraft.';
s = strjoin(L, newline);
end

function s = numBr(pk, md)
if isfinite(pk)
    s = sprintf('%.4g [%.4g]', pk, md);
else
    s = '--';
end
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
%DRAWCURVES  Base MATLAB graphics only.  One panel per WZ case, which is how
%   Figs. 26.19-26.21 are laid out.
lab = {'PN', 'APN', 'opt-1lag', 'opt-cubic'};
figure;
for k = 1:numel(R.wz)
    subplot(numel(R.wz), 1, k);
    hold on; grid on;
    lg = {};
    for j = 1:numel(R.laws)
        plot(R.tf, R.miss(:,j,k), '-');
        lg{end+1} = lab{R.laws(j)+1}; %#ok<AGROW>
    end
    xlabel('Flight Time (s)');
    ylabel('Miss (ft)');
    title(sprintf('Listing 26.6, WZ = %g r/s', R.wz(k)));
    legend(lg{:}, 'Location', 'best');
    hold off;
end

figure;
hold on; grid on;
plot(R.step.t, R.step.xnl, '-');
plot(R.step.t, R.step.single_lag, '--');
xlabel('Time (s)');
ylabel('Achieved acceleration / commanded');
title('Figs. 26.17/26.18: cubic FCS step response vs the single-lag representation');
hold off;
end
