function R = optimalGuidanceMC(opts)
%MGS.ZARCHAN.OPTIMALGUIDANCEMC  Zarchan Listing 9.3 -- Monte Carlo engagement
%   simulation testing OPTIMAL GUIDANCE against proportional navigation, with a
%   three-state digital Kalman filter and a single-lag flight-control system.
%
%   R = mgs.zarchan.optimalGuidanceMC()
%   R = mgs.zarchan.optimalGuidanceMC('runs', 50)          the book's own set
%   R = mgs.zarchan.optimalGuidanceMC('TAU', 0.1, 'assert', false)
%   R = mgs.zarchan.optimalGuidanceMC('filter_accel', 'XNC')   the printed page
%
%   Line-by-line port of Listing 9.3 (zarchan.txt:2894-2919).  UNITS ARE THE
%   BOOK'S: FEET, ft/s, ft/s^2, g = 32.2.  NOT SI.  Convert at the call site.
%
% --- WHAT TARGET DOES THIS FLY?  (read before quoting any law ordering) ------
%   YDD = XNT - XNL with XNT = 96.6 ft/s^2 held CONSTANT for the whole flight
%   (:2894, :2902), confirmed in prose at :2892: "a constant 3-g target maneuver
%   occurring at the beginning of flight".  The target DOES maneuver.  That
%   matters, because an earlier wave in this package measured augmented PN as
%   WORSE than plain PN on a listing whose target was non-maneuvering, where the
%   augmentation term is pure estimator noise.  Not the case here.
%   The two disturbances are orthogonal by construction and :2892 says so: the
%   only RANDOM input is 1 mr of seeker noise, the only DETERMINISTIC input is
%   the 3-g step, so SIGMA of the miss = noise-induced miss and MEAN of the miss
%   = maneuver-induced miss.  SELF-CHECK 4 is the delete experiment for that.
%   The book compares APN = 0 (PN) and APN = 2 (optimal) only; APN = 1
%   (augmented PN) is run here and REPORTED, never scored.
%
% --- THE TRANSCRIPTION, AND WHAT WAS DONE TO IT -----------------------------
%   listing_09_03.m does not parse.  Its header reports ONE unresolved
%   variable-to-variable comparison; THREE comparisons in fact arrive damaged
%   (repairs 1-3), and the pure variable-against-variable one -- the only one
%   with no shape cue of its own -- is repair 3, "if XNC.XLIM".  All three are
%   resolved below, in decreasing order of judgement:
%    1  "if S.=(TS - 1e-5)"  ->  "if S >= (TS - 1e-5)".
%       Resolved as ">=", three
%       reasons: (a) S is zeroed INSIDE the branch, so "<=" fires every step and
%       the "discrete section" would run at the integration rate; (b) :2791 says
%       the discrete section is entered "every sampling interval Ts" and that
%       "Ts/H must be an integer" -- meaningless unless S counts UP to TS;
%       (c) it is measurable: "<=" multiplies the number of filter updates by
%       TS/H = 10 and moves the miss by order 1.
%    2  "while T,= (TF - 1e-5)" -> "<=", "while STEP ,=1" -> "<=".  This book's
%       OCR renders "<" as ",".  Shape-forced: T only increases; STEP=2 leaves.
%    3  "if XNC.XLIM" -> "if XNC > XLIM".  "." is the OCR of ">"; XLIM is a
%       positive symmetric limit and the paired branch already reads "< -XLIM".
%    4  "XNEW=XNPP*XNL*(EXP(-X)+X-1.)/(X*X)" -> lowercase exp.  MATLAB is
%       case-sensitive and EXP is undefined; the same printed line (:2910) uses
%       lowercase exp three times.  A book typo, not a judgement call.
%    5  Dropped: the plot block, "save datfil.txt output /ascii" (a typo for
%       -ascii; nothing is written to disk here), and the dead %Preallocation
%       line, which assigns TF and I before both loops overwrite them.
%
%   QUIRKS REPRODUCED DELIBERATELY -- do not "tidy" these:
%    Q1 TGO carries +1e-5 in the continuous section and +1e-6 in the discrete
%       one (:2902).  At the FINAL sample (T = TF) that mismatch scales the
%       measurement by 0.1, but the while test fails immediately afterwards, so
%       it never reaches Y.  Elsewhere it is a 1e-4 relative effect.  That same
%       final update divides XLAMDH by VC*TGO^2 = 4e-9, so its XNC leaves for
%       ~1e12 ft/s^2 and IS clipped by XLIM.  It is dead, so SELF-CHECK 5 scores
%       the LIVE updates only and reports the dead one -- counting it would fail
%       the invariant on a correct port.
%    Q2 XLAM is evaluated at the PREDICTOR state, inside the inner loop, and
%       then read by the discrete section after the corrector has overwritten Y
%       -- a half-step-stale measurement, O(H^2).
%    Q3 XLAMD is computed on every pass and NEVER READ: the laws use XLAMDH from
%       the filter.  Kept so a reader diffing against the page finds it.
%    Q4 The APN==1 branch writes "APN*.5*XNP*XNTH"; APN is 1 there, so the
%       factor is 0.5.  Kept literally.
%    Q5 XNPP = TOP/(.0001+BOT1+BOT2).  The 1e-4 is a REGULARISER, not physics.
%       BOT1+BOT2 vanishes to O(x^5) (= 0.3x^5) as x = tgo/TAU -> 0 while TOP ->
%       3x^4, so unguarded XNPP ~ 10/x diverges at intercept; with the guard it
%       COLLAPSES to ~0 below x ~ 0.2 (tgo < 0.1 s at TAU = 0.5 s).
%    Q6 VM = 3000 and HEDEG = 20 enter ONLY the filter's P22 = (VM*HEDEG/57.3)^2.
%       The plant has YIC = 0 and YD(0) = 0: there is NO heading error in the
%       truth.  The filter is deliberately initialised pessimistically.  Equally,
%       PHIN = XNT^2/TF models a maneuver with UNIFORMLY DISTRIBUTED start time
%       (:2785) while the truth steps at t = 0.  Both mismatches are the book's.
%
% --- THE BOOK CONTRADICTS ITSELF, AND IT MATTERS ENORMOUSLY -----------------
%   :2891 says "the achieved missile acceleration nL rather than the commanded
%   acceleration nc is fed back into the filter".  The listing at :2910 writes
%   nc in all three filter lines.  This is NOT a cosmetic difference:
%
%     fed nc  : the optimal law DIVERGES.  Its noise miss reaches 258 ft over
%               the book's sweep and the acceleration limiter bites on 9.4% of
%               live updates -- 65x outside the axis([0,10,0,4]) box that the
%               same listing draws at :2923 for a figure carrying both laws.
%     fed nL  : every ordering the book states appears.  See MEASURED, below.
%
%   So :2910 cannot be the code that drew Figs. 9.14/9.15, and this port
%   DEFAULTS TO THE PROSE (filter_accel = 'XNL').  Row 9 on the board scores
%   that choice; 'XNC' still reproduces the printed page, divergence included.
%   The mechanism is visible in the book: Listing 9.2 has NO flight-control lag
%   (its plant is YDD = XNT - XNC, :2817), so there nc IS the achieved
%   acceleration and the identical filter lines at :2821 are correct.  9.3 adds
%   the lag (YDD = XNT - XNL, :2902) and those three lines came across
%   unchanged.  Both other lagged listings with this filter -- 25.1 (:8595) and
%   29.3 (:10778) -- write XNL.
%
%   *** WHY THE DIFFERENCE IS SO VIOLENT.  The filter propagates its estimate
%   with .5*TS^2*(XNTH - a).  Under PN the command is tens of g and using the
%   wrong 'a' is a small error.  The optimal law runs an effective navigation
%   ratio of 12-24 in the endgame (Q5), commands thousands of g, and feeding
%   THAT back as though the airframe had flown it corrupts the estimate that
%   generated it.  The loop closes on its own error.  This is the single
%   highest-leverage line in the listing and it has no symptom under APN = 0,
%   which is the case the listing ships set to.
%
%   OUTPUTS: R.tf R.laws R.sigma_ft R.mean_ft (nLaws-by-nTF) R.se_sigma_ft
%   R.se_mean_ft R.miss_ft (raw, nLaws-by-nTF-by-runs) R.clip_frac
%   R.clip_dead_frac (Q1's dead final update, reported not scored)
%   R.sigma50_ft R.mean50_ft (the book's own first 50 runs) R.xnpp (Q5 sweep)
%   R.feedback (the nL-vs-nc probe) R.bench R.selfcheck R.pass R.verdict R.p
%   R.units R.summary.
%
% --- MEASURED, on the defaults (200 runs, seed 20260824, tF = .5:.5:10) -----
%   Run on 2026-08-24 in the caller's own MATLAB.  Reproducible bit for bit:
%   the generator is the Lehmer stream in this file, so nothing here depends on
%   rng() state or on anyone else's session.  ~0.2 s for all three laws.
%
%     tF [s]        0.5    1.0    2.0    5.0    10.0
%     sigma  PN    0.905  1.166  1.780  1.637  1.758     (noise miss)
%     sigma  opt   0.910  0.972  0.832  0.769  0.657
%     mean   PN    8.765 17.554 11.705 -0.556  0.012     (maneuver miss)
%     mean   opt   0.901 -0.590 -0.360  0.055  0.063
%
%   Board: 10 scored rows, all pass, 5 structural invariants pass.
%
%   *** ROW 6 WAS RE-AIMED ON 2026-08-24, AND THE OLD NOTE HERE WAS WRONG. ***
%   It used to score the SIGMA gap against ":2892 -- the differences between
%   the guidance laws is greatest for the smaller flight times", read 0.589 ->
%   0.972 ft (widest at LONG tF), stood permanently red, and was filed as "a
%   result, not a defect".  It was neither: it was MIS-AIMED.  :2892's subject
%   is the MISS DISTANCE -- "optimal guidance yields smaller MISS DISTANCES ...
%   The differences between the guidance laws is greatest for the smaller
%   flight times" -- and :2924 nails the quantity down: "the performance of
%   both ... is about the same, FROM A MISS DISTANCE POINT OF VIEW, as can be
%   seen from Figs. 9.14 AND 9.15."  Sigma and mean are the two components the
%   same paragraph splits that miss into, so the quantity is their rms
%   combination, this book's standing measure (:8925, :10015).  Measured on
%   it: the gap runs 11.278 -> 0.974 ft, a 11.6x collapse, peak 16.46 ft at
%   tF = 1 s.  Fig. 9.14's own caption (:2925) claims only that optimal has
%   the smaller noise miss and says nothing about flight time; that claim is
%   row 3 and it passes 19/20.
%   NOTHING WAS WIDENED AND NOTHING WAS DELETED: the sigma-gap number is still
%   on the board, as the INFO row beside it, with the mechanism -- the
%   maneuver miss decays to nothing for both laws by 10 s so its gap must
%   close, while PN's noise miss sits on a plateau near 1.7 ft against
%   optimal's 0.7 ft, so THAT gap is widest at long tF.  It is a real
%   observation about a component; it just is not what :2892 asserts.
%
%   See also MGS.ZARCHAN.RICCATISOLVER (Listing 9.1 -- the same Riccati
%   recursion, scored against the book's MATRIX form), MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 9.3 line 1, verbatim (zarchan.txt:2894, :2896, :2898).
    opts.VC       (1,1) double {mustBeFinite, mustBePositive}    = 4000.0  % ft/s
    opts.XNT      (1,1) double {mustBeFinite, mustBeNonnegative} = 96.6    % ft/s^2
    opts.YIC      (1,1) double {mustBeFinite}                    = 0.0     % ft
    opts.VM       (1,1) double {mustBeFinite, mustBePositive}    = 3000.0  % ft/s
    opts.HEDEG    (1,1) double {mustBeFinite, mustBeNonnegative} = 20.0    % deg
    opts.XNP      (1,1) double {mustBeFinite, mustBePositive}    = 3.0
    opts.SIGNOISE (1,1) double {mustBeFinite, mustBeNonnegative} = 0.001   % rad
    opts.TS       (1,1) double {mustBeFinite, mustBePositive}    = 0.1     % s
    opts.TAU      (1,1) double {mustBeFinite, mustBePositive}    = 0.5     % s
    opts.H        (1,1) double {mustBeFinite, mustBePositive}    = 0.01    % s
    opts.XLIM     (1,1) double {mustBeFinite, mustBePositive}    = 999999  % ft/s^2
    opts.NOISE    (1,1) logical = true
    % [] means the listing's own sweep, TF = .5:.5:10 s.
    opts.tf   double {mustBeReal, mustBeFinite} = []
    opts.laws double {mustBeReal, mustBeFinite} = [0 1 2]
    % The book runs 50 (:2892).  200 here because a 50-run sigma carries ~10%
    % sampling error of its own answer and cannot ADJUDICATE an ordering; the
    % first-50 subset is reported as an INFO row so the book's experiment stays
    % visible.  This is the cost knob: cost is linear in runs.
    opts.runs (1,1) double {mustBeFinite, mustBePositive} = 200
    opts.seed (1,1) double {mustBeFinite, mustBeNonnegative} = 20260824
    % 'XNL' = the prose (:2891) -- THE DEFAULT, and see the header for the
    % measurement that forced it.  'XNC' = the listing as printed (:2910),
    % kept so the printed page can still be reproduced exactly.
    opts.filter_accel (1,:) char {mustBeMember(opts.filter_accel,{'XNC','XNL'})} = 'XNL'
    % Standard errors allowed before an ordering counts as violated.  4, not 2:
    % the ordering rows test 20 flight times on TWO figures at once, so a k that
    % is merely "significant" buys a standing chance that a CORRECT port lights
    % up on the draw.  At k = 2 that chance is ~60% over 40 comparisons, at 3
    % ~10%, at 4 ~0.3%.  It costs nothing: every gap the book calls "clear" is
    % 10-200 SE wide, so k = 4 still separates them.
    opts.ksig (1,1) double {mustBeFinite, mustBePositive} = 4.0
    % THE ONLY SWITCHES.  Not nargin: with an arguments block declaring only
    % name-value options MATLAB reports nargin = 0 however many were passed, so
    % an `if nargin == 0` mode here would never fire.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

ratio = opts.TS/opts.H;
if abs(ratio - round(ratio)) > 1e-9
    error('mgs:zarchan:optimalGuidanceMC:tsh', ...
        ['TS/H = %g is not an integer.  zarchan.txt:2791 states the ' ...
         'requirement outright; violating it does not crash, it silently ' ...
         'jitters the filter update instants and returns a smooth wrong curve.'], ratio);
end
tf = opts.tf(:).';
if isempty(tf), tf = 0.5:0.5:10.0; end
tf = tf(tf > 0);
if isempty(tf)
    error('mgs:zarchan:optimalGuidanceMC:tf', 'no positive flight time survives in ''tf''.');
end
laws = unique(round(opts.laws(:).'));
if isempty(laws) || any(laws < 0 | laws > 2)
    error('mgs:zarchan:optimalGuidanceMC:laws', ...
        'laws must be a non-empty subset of [0 1 2] (PN, augmented PN, optimal); got %s.', ...
        mat2str(opts.laws));
end
nRuns = round(opts.runs);
if nRuns < 8
    error('mgs:zarchan:optimalGuidanceMC:runs', ...
        ['runs = %d.  A sample this small has no usable standard error, so ' ...
         'every ordering row would pass on the draw rather than on the port.'], nRuns);
end

P = struct('VC',opts.VC, 'XNP',opts.XNP, 'TAU',opts.TAU, 'TS',opts.TS, ...
           'H',opts.H, 'SIGNOISE',opts.SIGNOISE, 'XLIM',opts.XLIM, ...
           'VM',opts.VM, 'HEDEG',opts.HEDEG, 'YIC',opts.YIC, ...
           'XNT_truth',opts.XNT, 'XNT_model',opts.XNT, ...
           'noiseOn',opts.NOISE, 'useXNL',strcmp(opts.filter_accel,'XNL'));
% XNT_truth/XNT_model are ONE variable in the listing.  The split exists only
% so SELF-CHECK 4 can delete the deterministic disturbance without also
% deleting the filter's process-noise model -- see the header.

% ---- noise, drawn once and SHARED by every law (common random numbers) -----
% Same realisations for PN, augmented PN and optimal, so the law-to-law rows
% score a paired difference.  That reduces the variance of the DIFFERENCE, which
% is the only quantity the book's figure captions make a claim about.
nTF = numel(tf);  nL = numel(laws);
G = cell(1,nTF);  st = mod(floor(abs(opts.seed)), 2147483646) + 1;
for k = 1:nTF
    [G{k}, st] = gaussBlock(st, round(tf(k)/opts.TS) + 2, nRuns);
end

sigma_ft = zeros(nL,nTF);  mean_ft = zeros(nL,nTF);  clip_frac = zeros(nL,nTF);
dead_frac = zeros(nL,nTF);
sigma50  = nan(nL,nTF);    mean50  = nan(nL,nTF);
se_sig   = zeros(nL,nTF);  se_mean = zeros(nL,nTF);
% The RAW misses are kept.  They are what makes the ordering rows a PAIRED
% comparison: the laws share their random numbers, so the standard error of the
% law-to-law difference is std(Z_a - Z_b)/sqrt(n) and is several times smaller
% than the unpaired form.  Costs nL*nTF*runs doubles.
miss_ft  = zeros(nL,nTF,nRuns);
for a = 1:nL
    for k = 1:nTF
        [Z, ncl, ntot, ndd] = engagement(laws(a), tf(k), G{k}, P);
        [sigma_ft(a,k), mean_ft(a,k)] = bookStats(Z);
        [se_sig(a,k), se_mean(a,k)]   = seOf(Z);
        miss_ft(a,k,:) = Z;
        clip_frac(a,k) = ncl/max(ntot,1);
        dead_frac(a,k) = ndd/nRuns;
        if nRuns >= 50
            [sigma50(a,k), mean50(a,k)] = bookStats(Z(1:50));
        end
    end
end

% ---- THE BOOK'S CONTRADICTION, MEASURED RATHER THAN ASSERTED ---------------
% :2891 (prose) feeds the ACHIEVED acceleration nL to the filter; :2910 (the
% printed listing) feeds the COMMANDED nc.  Both are flown here, at one flight
% time, with the SAME random numbers, so the choice this port makes is a scored
% row instead of an opinion.  Four engagements; the probe flight time is 4*TAU,
% near where the noise miss peaks, and it does not depend on which laws the
% caller asked for.
[~, kp] = min(abs(tf - 4*opts.TAU));
Pc = P;  Pc.useXNL = false;
Pl = P;  Pl.useXNL = true;
FB = struct('tf', tf(kp));
FB.sig_pn_xnc  = bookStats(engagement(0, tf(kp), G{kp}, Pc));
FB.sig_opt_xnc = bookStats(engagement(2, tf(kp), G{kp}, Pc));
FB.sig_pn_xnl  = bookStats(engagement(0, tf(kp), G{kp}, Pl));
FB.sig_opt_xnl = bookStats(engagement(2, tf(kp), G{kp}, Pl));

R = struct();
R.tf = tf;  R.laws = laws;  R.feedback = FB;
R.sigma_ft = sigma_ft;  R.mean_ft = mean_ft;  R.clip_frac = clip_frac;
R.clip_dead_frac = dead_frac;
R.sigma50_ft = sigma50; R.mean50_ft = mean50;
R.se_sigma_ft = se_sig;  R.se_mean_ft = se_mean;  R.miss_ft = miss_ft;
% Q5 made visible: the optimal law's own effective navigation ratio.
R.xnpp_x = [0.05 0.2 1 3 10 1000];
R.xnpp   = arrayfun(@xnppOf, R.xnpp_x);
R.units  = 'ft, ft/s, ft/s^2 (g = 32.2 ft/s^2); LOS in rad, time in s';
R.p = struct('VC',opts.VC,'XNT',opts.XNT,'VM',opts.VM,'HEDEG',opts.HEDEG, ...
    'XNP',opts.XNP,'SIGNOISE',opts.SIGNOISE,'TS',opts.TS,'TAU',opts.TAU, ...
    'H',opts.H,'XLIM',opts.XLIM,'NOISE',opts.NOISE,'runs',nRuns, ...
    'seed',opts.seed,'rng','MINSTD Lehmer + Box-Muller, local to this file', ...
    'filter_accel',opts.filter_accel,'tf',tf,'laws',laws,'units',R.units);

R.selfcheck = structuralChecks(R, P, G, nRuns);

if opts.benchmark
    R.bench = benchmarks(R, opts);
    ev = ~[R.bench.skipped];
    R.pass = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();  ev = false;
    % NOT true.  "not checked" must never render as "fine".
    R.pass = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R);            end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false, so nothing was compared against the book. R.pass = NOT CHECKED.';
elseif ~any(ev)
    msg = 'not one book value could be evaluated; an unvalidated port must not be handed back silently.';
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question. First: %s.'], ...
        sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf('%d of %d evaluated book values are outside tolerance (first: %s, book %s, ours %s).', ...
        numel(bad), sum(ev), R.bench(bad(1)).what, R.bench(bad(1)).book_str, R.bench(bad(1)).ours_str);
end
R.verdict = msg;
if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:optimalGuidanceMC:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN COMPLETE.  benchSuite runs every port with assert off
        % precisely so one red row cannot withhold the evidence behind it.
        warning('mgs:zarchan:optimalGuidanceMC:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [Z, nclip, ntot, ndead] = engagement(law, TF, G, P)
%ENGAGEMENT  Listing 9.3's two inner loops, zarchan.txt:2898-2913, VECTORISED
%   ACROSS REPLICATIONS.  Every replication at a flight time shares the time
%   grid and the (deterministic) Riccati recursion and they are independent, so
%   the book's "for I = 1:RUN" becomes an element index.  The arithmetic per
%   replication, and its ORDER, is the listing's.
%
%   Every constant arrives as an ARGUMENT: MATLAB local functions cannot see the
%   main function's workspace, and a previous wave in this package shipped a
%   local that read one from the caller and threw on the first line of the run.

VC = P.VC;  XNP = P.XNP;  TAU = P.TAU;  TS = P.TS;  H = P.H;
SIGNOISE = P.SIGNOISE;  XLIM = P.XLIM;
n = size(G,2);

TS2 = TS*TS;  TS3 = TS2*TS;  TS4 = TS3*TS;  TS5 = TS4*TS;
PHIN = P.XNT_model*P.XNT_model/TF;
RTM = VC*TF;  SIGPOS = RTM*SIGNOISE;  SIGN2 = SIGPOS^2;
P11 = SIGN2;  P12 = 0.;  P13 = 0.;
P22 = (P.VM*P.HEDEG/57.3)^2;  P23 = 0.;  P33 = P.XNT_model*P.XNT_model;

Y = P.YIC*ones(1,n);  YD = zeros(1,n);  XNL = zeros(1,n);  XNC = zeros(1,n);
YH = zeros(1,n);  YDH = zeros(1,n);  XNTH = zeros(1,n);
YDD = zeros(1,n);  XNLD = zeros(1,n);  XLAM = zeros(1,n);
T = 0.;  S = 0.;  k = 0;  nclip = 0;  ntot = 0;  ndead = 0;

while T <= (TF - 1e-5)
    YOLD = Y;  YDOLD = YD;  XNLOLD = XNL;
    STEP = 1;  FLAG = 0;
    % Zarchan's two-pass predictor loop.  T advances INSIDE it, exactly once,
    % which is what puts the second derivative evaluation at t+H.  Moving that
    % line out degrades the scheme to first order while still running smoothly.
    while STEP <= 1
        if FLAG == 1
            Y = Y + H*YD;  YD = YD + H*YDD;  XNL = XNL + H*XNLD;
            T = T + H;  STEP = 2;
        end
        TGO  = TF - T + .00001;              % Q1: 1e-5 here, 1e-6 below
        XLAM = Y ./ (VC*TGO);                % Q2: read later, post-corrector
        % XLAMD = (VC*TGO*YD + Y*VC)/(VC*TGO)^2 -- computed by the listing,
        % never read (Q3).  Not evaluated here; see the header before restoring.
        XNLD = (XNC - XNL)/TAU;
        YDD  = P.XNT_truth - XNL;            % CONSTANT 3-g target maneuver
        FLAG = 1;
    end
    % Corrector, in the listing's own algebra.
    Y   = .5*(YOLD   + Y   + H*YD);
    YD  = .5*(YDOLD  + YD  + H*YDD);
    XNL = .5*(XNLOLD + XNL + H*XNLD);

    S = S + H;
    if S >= (TS - 1e-5)                      % REPAIR 1 -- see header
        S = 0.;
        TGO = TF - T + .000001;              % Q1
        RTM = VC*TGO;  SIGPOS = RTM*SIGNOISE;  SIGN2 = SIGPOS^2;

        M11 = P11 + TS*P12 + .5*TS2*P13 + TS*(P12 + TS*P22 + .5*TS2*P23);
        M11 = M11 + .5*TS2*(P13 + TS*P23 + .5*TS2*P33) + TS5*PHIN/20.;
        M12 = P12 + TS*P22 + .5*TS2*P23 + TS*(P13 + TS*P23 + .5*TS2*P33) + TS4*PHIN/8.;
        M13 = P13 + TS*P23 + .5*TS2*P33 + PHIN*TS3/6.;
        M22 = P22 + TS*P23 + TS*(P23 + TS*P33) + PHIN*TS3/3.;
        M23 = P23 + TS*P33 + .5*TS2*PHIN;
        M33 = P33 + PHIN*TS;
        K1 = M11/(M11+SIGN2);  K2 = M12/(M11+SIGN2);  K3 = M13/(M11+SIGN2);
        P11 = (1.-K1)*M11;  P12 = (1.-K1)*M12;  P13 = (1.-K1)*M13;
        P22 = -K2*M12 + M22;  P23 = -K2*M13 + M23;  P33 = -K3*M13 + M33;

        k = k + 1;
        if k > size(G,1)
            % Refused, not wrapped.  Reusing a noise row would correlate two
            % filter updates and quietly shrink the sample dispersion.
            error('mgs:zarchan:optimalGuidanceMC:noiseblock', ...
                ['the noise block holds %d rows but update %d was reached at ' ...
                 'tF = %g s, TS = %g s.'], size(G,1), k, TF, TS);
        end
        if P.noiseOn, XLAMNOISE = SIGNOISE*G(k,:); else, XLAMNOISE = zeros(1,n); end
        YSTAR = RTM*(XLAM + XLAMNOISE);
        if P.useXNL, AFB = XNL; else, AFB = XNC; end   % :2891 prose vs :2910 code
        RES  = YSTAR - YH - TS*YDH - .5*TS*TS*(XNTH - AFB);
        YH   = K1*RES + YH + TS*YDH + .5*TS*TS*(XNTH - AFB);
        YDH  = K2*RES + YDH + TS*(XNTH - AFB);
        XNTH = K3*RES + XNTH;
        XLAMDH = (YH + YDH*TGO) ./ (VC*TGO*TGO);

        if law == 0
            XNC = XNP*VC*XLAMDH;                              % proportional nav
        elseif law == 1
            XNC = XNP*VC*XLAMDH + 1*.5*XNP*XNTH;              % Q4: APN == 1 here
        else
            X    = TGO/TAU;
            TOP  = 6.*X*X*(exp(-X) - 1. + X);
            BOT1 = 2*X*X*X + 3. + 6.*X - 6.*X*X;
            BOT2 = -12.*X*exp(-X) - 3.*exp(-2.*X);
            XNPP = TOP/(.0001 + BOT1 + BOT2);                 % Q5: 1e-4 guard
            XNEW = XNPP*XNL*(exp(-X) + X - 1.)/(X*X);         % REPAIR 4
            XNC  = XNPP*VC*XLAMDH + .5*XNPP*XNTH - XNEW;      % optimal guidance
        end
        % LIVE vs DEAD clipping.  The FINAL update runs at T = TF, so its
        % TGO is 1e-6 (Q1) and XLAMDH divides by VC*TGO^2 = 4e-9: XNC leaves
        % for ~1e12 ft/s^2 and is clipped on essentially every replication.
        % That XNC is DEAD -- the while test fails immediately afterwards, so
        % it never reaches Y.  Counting it would make the "limiter never
        % binds" invariant FAIL ON A CORRECT PORT, so the two counts are kept
        % apart and only the live one is scored.  The dead one is reported.
        nbite = sum(XNC > XLIM | XNC < -XLIM);
        if T <= (TF - 1e-5)
            nclip = nclip + nbite;  ntot = ntot + n;
        else
            ndead = ndead + nbite;
        end
        XNC = min(max(XNC, -XLIM), XLIM);                     % REPAIR 3
    end
end
Z = Y;
end


% =============================================================================
function [SIGMA, XMEAN] = bookStats(Z)
%BOOKSTATS  The listing's own estimators, zarchan.txt:2915-2918: XMEAN = Z1/I
%   with I = RUN, and SIGMA with the (RUN-1) divisor.
N = numel(Z);
XMEAN = sum(Z)/N;
SIGMA = sqrt(sum((Z - XMEAN).^2)/(N-1));
end

function [seS, seM] = seOf(Z)
%SEOF  Standard errors of the sample SIGMA and the sample MEAN.
%   SE(s) comes from the sample's OWN fourth moment, not from the normal-theory
%   s/sqrt(2n): the miss here is a linear functional of Gaussian noise PLUS a
%   deterministic offset, and assuming kurtosis 3 would misstate the band in a
%   direction nobody checks.
N = numel(Z);  m = sum(Z)/N;  d = Z - m;
m2 = sum(d.^2)/N;  m4 = sum(d.^4)/N;
s  = sqrt(sum(d.^2)/(N-1));
seS = sqrt(max(m4 - m2^2, 0)/N)/(2*max(s, realmin));
seM = s/sqrt(N);
end

function v = xnppOf(x)
%XNPPOF  The listing's effective navigation ratio, :2910, as a function of
%   x = tgo/TAU.  Shares no line with the loop, on purpose -- SELF-CHECK 1.
TOP  = 6.*x*x*(exp(-x) - 1. + x);
BOT1 = 2*x*x*x + 3. + 6.*x - 6.*x*x;
BOT2 = -12.*x*exp(-x) - 3.*exp(-2.*x);
v = TOP/(.0001 + BOT1 + BOT2);
end


% =============================================================================
function [Gm, state] = gaussBlock(state, nrow, ncol)
%GAUSSBLOCK  N(0,1) draws from a MINSTD (Lehmer) stream plus Box-Muller.
%   Deterministic, base MATLAB, and it does not touch the caller's global random
%   stream -- a benchmark that moves when someone else calls rng() is not a
%   benchmark, and setting the global stream to fix that reaches into a session
%   that runs fifteen other ports back to back.
%   x <- 16807*x mod (2^31-1); 16807*x < 3.61e13 < 2^53, so the recurrence is
%   EXACT in double precision.  x = 0 is absorbing, hence the 1..m-1 mapping.
m = 2147483647;  x = state;
np = ceil(nrow*ncol/2);
u = zeros(1, 2*np);
for i = 1:2*np
    x = mod(16807*x, m);
    u(i) = x/m;                       % in (0,1): log() and the sqrt are safe
end
state = x;
u1 = u(1:np);  u2 = u(np + (1:np));   % SEPARATE BLOCKS, not consecutive pairs:
r  = sqrt(-2*log(u1));                % consecutive values of a multiplicative
z  = [r.*cos(2*pi*u2); r.*sin(2*pi*u2)];   % generator lie on a 2-D lattice
Gm = reshape(z(1:nrow*ncol), nrow, ncol);
end


% =============================================================================
function K = structuralChecks(R, P, G, nRuns)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never evidence that
%   this port reproduces Zarchan -- the weaker claim that it agrees with itself,
%   which is what catches wiring faults.  .names and .oks are what put these on
%   the board; an invariant nobody can see is an invariant that did not run.
K = struct();  names = {};  oks = [];  detail = {};

% 1  The optimal law degenerates to PN as tgo/TAU -> infinity.  Analytic:
%    TOP -> 6x^3, BOT -> 2x^3, so XNPP -> 3.  Catches a mangled TOP/BOT1/BOT2.
v = xnppOf(1000);
names{end+1} = 'XNPP -> 3 as tgo/TAU -> inf (optimal degenerates to PN)';
oks(end+1)   = abs(v - 3) <= 0.01;
detail{end+1} = sprintf('XNPP(1000) = %.5f; XNPP at x = 0.05/0.2/1/3/10 = %s (Q5 guard)', ...
    v, mat2str(round(arrayfun(@xnppOf,[0.05 0.2 1 3 10]),3)));

% 2  The ONLY stochastic input is XLAMNOISE.  With NOISE = 0 every replication
%    must be bit-identical, so the sample dispersion is exactly zero.
Pq = P;  Pq.noiseOn = false;
Zq = engagement(0, R.tf(end), G{end}, Pq);
spread = max(abs(Zq - Zq(1)))/max(abs(Zq(1)), realmin);
names{end+1} = 'NOISE = 0 => zero run-to-run dispersion';
oks(end+1)   = spread <= 1e-12;
detail{end+1} = sprintf('max relative spread over %d runs = %.3e at tF = %g s', ...
    nRuns, spread, R.tf(end));

% 3  VC and SIGNOISE enter ONLY as their PRODUCT.  SIGPOS = VC*TGO*SIGNOISE and
%    YSTAR = RTM*(XLAM+XLAMNOISE) both carry the product; VC cancels out of
%    XNC = N'*VC*XLAMDH because XLAMDH divides by VC.  Exact, not approximate.
%    Every plausible way to BREAK it moves the trajectory by order 1.
Pa = P;  Pb = P;  Pb.VC = 2*P.VC;  Pb.SIGNOISE = P.SIGNOISE/2;
Za = engagement(2, R.tf(end), G{end}, Pa);
Zb = engagement(2, R.tf(end), G{end}, Pb);
rel = max(abs(Za - Zb))/max(max(abs(Za)), realmin);
names{end+1} = 'only the product VC*SIGNOISE matters (optimal law)';
oks(end+1)   = rel <= 1e-9;
detail{end+1} = sprintf('|dZ|/|Z| = %.3e between (VC,sig) and (2VC,sig/2)', rel);

% 4  DELETE EXPERIMENT for the book's own decomposition at :2892: the MEAN is
%    the maneuver-induced miss, the SIGMA is the noise-induced miss.  Remove the
%    3-g step from the TRUTH ONLY (the filter keeps its process-noise model) and
%    the mean must collapse while the dispersion survives.
[~, kD] = min(abs(R.tf - 1.0));            % small tF: where the step miss lives
aD = 1;  lawD = R.laws(aD);                % whichever law this run actually flew
Pd = P;  Pd.XNT_truth = 0;
Zd = engagement(lawD, R.tf(kD), G{kD}, Pd);
[sD, mD] = bookStats(Zd);
m0 = R.mean_ft(aD,kD);  s0 = R.sigma_ft(aD,kD);
seD = sD/sqrt(nRuns);
names{end+1} = 'DELETE target maneuver => mean collapses, sigma survives';
oks(end+1)   = abs(mD) <= max(0.10*abs(m0), 3*seD) && sD >= 0.5*s0;
detail{end+1} = sprintf(['APN = %d, tF = %g s: mean %+.4f -> %+.4f ft (limit %.4f), ' ...
    'sigma %.4f -> %.4f ft (floor %.4f)'], lawD, R.tf(kD), m0, mD, ...
    max(0.10*abs(m0), 3*seD), s0, sD, 0.5*s0);

% 5  XLIM = 999999 must NOT bind.  That is what makes Figs 9.14/9.15 the LINEAR
%    comparison :2892 describes; the saturation study is Fig. 9.16 (:2933).
%    If a future default lets the limiter bite, every ordering row silently
%    starts measuring a different experiment.
worstClip = max(R.clip_frac(:));
names{end+1} = 'acceleration limit never binds at the listing''s XLIM';
oks(end+1)   = worstClip == 0;
detail{end+1} = sprintf(['worst LIVE clipped fraction = %.4g (XLIM = %g ft/s^2); the DEAD ' ...
    'final update (Q1, TGO = 1e-6) clipped %.4g of replications and cannot reach Y'], ...
    worstClip, P.XLIM, max(R.clip_dead_frac(:)));

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
%BENCHMARKS  Computed vs published.  EVERY book value was grepped out of
%   zarchan.txt and the line number is in .source.  Nothing was read off a
%   figure -- the extraction turned every figure into an image link, and earlier
%   waves here were handed exactly such phantom readings.  Where the book states
%   a SHAPE about a figure, that prose IS greppable, and an ordering check is
%   stronger than a plotted value.
B = emptyBench();
iPN = find(R.laws == 0, 1);  iAP = find(R.laws == 1, 1);  iOP = find(R.laws == 2, 1);
pair = ~isempty(iPN) && ~isempty(iOP);
whyPair = sprintf(['both APN = 0 and APN = 2 are needed for a law ordering; ' ...
    'this run flew laws %s'], mat2str(R.laws));

% ---- 1, 2.  The two UNIT CONVERSIONS the prose and the listing cross-check --
B = addVal(B, 'target maneuver amplitude', 3.0, opts.XNT/32.2, 'text', 0.005, ...
    'p.204 text "a constant 3-g target maneuver" (zarchan.txt:2892) vs XNT = 96.6 (:2894)', 'g', ...
    'two independent statements of the same constant; this row is the ft/s^2 <-> g conversion');
B = addVal(B, 'seeker measurement noise', 1.0, opts.SIGNOISE*1000, 'text', 0.005, ...
    'p.204 text "1 mr of measurement noise" (zarchan.txt:2892) vs SIGNOISE = .001 (:2896)', 'mr', ...
    'the rad <-> milliradian conversion, cross-checked against the listing constant');

% ---- 3.  Fig. 9.14 -- noise-induced miss ordering --------------------------
%   :2892 "optimal guidance yields smaller miss distances".  Scored against the
%   MEASURED standard error of this sample rather than as an exact inequality,
%   and the reason is not comfort: at the shortest flight times both laws land
%   within a few thousandths of a foot of each other, well inside the sampling
%   error of either estimate, and declaring a winner there would be adjudicating
%   the DRAW of the random numbers rather than the port.  The row therefore asks
%   the two questions the prose actually asserts -- optimal is never
%   SIGNIFICANTLY worse, and IS significantly better somewhere -- and prints the
%   count and the worst z so a shrinking margin is visible before it flips.
if pair
    dS  = R.sigma_ft(iPN,:) - R.sigma_ft(iOP,:);
    % Unpaired combination, which OVERSTATES the error here: the two laws share
    % their random numbers, so the estimates are positively correlated and the
    % true SE of the difference is smaller.  Said out loud because it makes the
    % "never worse" half easier and the "better somewhere" half harder.
    seS = sqrt(R.se_sigma_ft(iPN,:).^2 + R.se_sigma_ft(iOP,:).^2);
    zS  = dS ./ max(seS, realmin);
    okS = ~any(zS < -opts.ksig) && any(zS > opts.ksig);
    [~, iw] = min(zS);
    B = addShape(B, 'Fig. 9.14: sigma(optimal) < sigma(PN)', ...
        sprintf('optimal never worse by >%.0f SE, better somewhere', opts.ksig), ...
        sprintf('better at %d/%d tF (%d beyond %.0f SE); worst %+.4f ft = %+.2f SE at tF = %g s', ...
                sum(dS > 0), numel(dS), sum(zS > opts.ksig), opts.ksig, dS(iw), zS(iw), R.tf(iw)), ...
        okS, 'Fig. 9.14 caption (zarchan.txt:2925) + p.204 text (:2892)', ...
        'sigma is the noise-induced miss because seeker noise is the only random input (:2892)');
else
    B = addSkip(B, 'Fig. 9.14: sigma(optimal) < sigma(PN)', 'optimal smaller', ...
        whyPair, 'Fig. 9.14 caption (zarchan.txt:2925)', 'ft');
end

% ---- 4.  Fig. 9.15 -- maneuver-induced miss ordering -----------------------
%   The same claim on the other figure.  Here the standard error IS the paired
%   one, std(Z_PN - Z_opt)/sqrt(n), which the stored raw misses make exact.
if pair
    dM  = abs(R.mean_ft(iPN,:)) - abs(R.mean_ft(iOP,:));
    dZ  = reshape(R.miss_ft(iPN,:,:) - R.miss_ft(iOP,:,:), numel(R.tf), []);
    seM = std(dZ, 0, 2).'/sqrt(size(dZ,2));
    zM  = dM ./ max(seM, realmin);
    okM = ~any(zM < -opts.ksig) && any(zM > opts.ksig);
    [~, iw] = min(zM);
    B = addShape(B, 'Fig. 9.15: |mean|(optimal) < |mean|(PN)', ...
        sprintf('optimal never worse by >%.0f SE, better somewhere', opts.ksig), ...
        sprintf('better at %d/%d tF (%d beyond %.0f SE); worst %+.4f ft = %+.2f SE at tF = %g s', ...
                sum(dM > 0), numel(dM), sum(zM > opts.ksig), opts.ksig, dM(iw), zM(iw), R.tf(iw)), ...
        okM, 'Fig. 9.15 caption (zarchan.txt:2934) + p.204 text (:2892)', ...
        ['the mean is the maneuver-induced miss; |.| because the book plots ' ...
         'magnitude.  Where BOTH laws have already decayed to tenths of a foot ' ...
         'the book itself calls them equal (:2924), so the SE gate is what ' ...
         'keeps this row from adjudicating a 0.05 ft difference']);
else
    B = addSkip(B, 'Fig. 9.15: |mean|(optimal) < |mean|(PN)', 'optimal smaller', ...
        whyPair, 'Fig. 9.15 caption (zarchan.txt:2934)', 'ft');
end

% ---- 5, 6.  "the differences ... is greatest for the smaller flight times" --
%   :2892, restated at :2924 as "for a 10-s flight the performance of both is
%   about the same".  SCORED ONCE PER FIGURE, deliberately.  The claim is made
%   about both figures in one sentence, but it is two measurements, and a single
%   conjunctive row would report a failure without naming the guilty half.
if pair
    q  = max(1, round(numel(R.tf)/4));
    lo = 1:q;  hi = (numel(R.tf)-q+1):numel(R.tf);
    gM = abs(R.mean_ft(iPN,:)) - abs(R.mean_ft(iOP,:));
    gS = R.sigma_ft(iPN,:) - R.sigma_ft(iOP,:);
    B = addShape(B, 'Fig. 9.15: the PN-vs-optimal gap is largest at SMALL tF', ...
        'gap(low tF) > gap(high tF)', ...
        sprintf('|mean| gap %.3f -> %.3f ft (first vs last %d of %d tF)', ...
                mean(gM(lo)), mean(gM(hi)), q, numel(R.tf)), ...
        mean(gM(lo)) > mean(gM(hi)), ...
        'p.204 text (zarchan.txt:2892), restated at p.208 (:2924)', ...
        'the mechanism the book gives: PN is already effective once tF >> TAU, so there is less left to win');
    % ---- 6.  RE-AIMED 2026-08-24.  This row used to score the SIGMA gap and
    %      stood permanently red at 0.589 -> 0.972 ft.  It was mis-aimed, and
    %      the book says so in two places:
    %        :2892  "Both figures clearly show that ... optimal guidance yields
    %                smaller MISS DISTANCES, even in the presence of
    %                measurement noise errors.  The differences between the
    %                guidance laws is greatest for the smaller flight times."
    %        :2924  "This flight time was chosen because the performance of
    %                both proportional navigation and optimal guidance is
    %                about the same, FROM A MISS DISTANCE POINT OF VIEW, as
    %                can be seen from Figs. 9.14 AND 9.15."
    %      The subject is the MISS DISTANCE, read off the two figures together.
    %      Sigma and mean are the two components the same paragraph decomposes
    %      it into ("we can interpret the standard deviation of the miss to be
    %      the noise-induced miss and the mean of the miss to be the
    %      target-maneuver-induced miss", :2892), so the quantity the sentence
    %      is about is their rms combination -- "rms miss distance", which is
    %      this book's standing performance measure (e.g. :8925, :10015).
    %      Fig. 9.14's OWN caption (:2925) claims only that optimal has the
    %      smaller noise miss; it makes no claim about flight-time dependence,
    %      and that caption is already scored by row 3 above.
    %      The sigma-only measurement is NOT lost: it is the INFO row below,
    %      because it is a real and interesting observation -- it just is not
    %      what :2892 asserts.
    gT  = sqrt(R.sigma_ft(iPN,:).^2 + R.mean_ft(iPN,:).^2) ...
        - sqrt(R.sigma_ft(iOP,:).^2 + R.mean_ft(iOP,:).^2);
    B = addShape(B, 'miss distance: the PN-vs-optimal gap is largest at SMALL tF', ...
        'gap(low tF) > gap(high tF) on rms miss = sqrt(mean^2 + sigma^2)', ...
        sprintf('rms-miss gap %.3f -> %.3f ft (first vs last %d of %d tF), peak %.3f ft at tF = %g s', ...
                mean(gT(lo)), mean(gT(hi)), q, numel(R.tf), ...
                max(gT), R.tf(find(gT == max(gT), 1))), ...
        mean(gT(lo)) > mean(gT(hi)), ...
        'p.204 text (zarchan.txt:2892), restated at p.208 (:2924)', ...
        sprintf(['the two components the book decomposes the miss into, ' ...
         'recombined.  A %.1fx collapse, against a pooled standard error of ' ...
         'about %.3f ft per flight time, so the ordering is not a sampling ' ...
         'accident.  It is carried by the maneuver miss: sigma alone goes the ' ...
         'OTHER way (INFO row below), but sigma is 1.8 ft where the miss at ' ...
         'small tF is 17 ft, so it cannot set the miss distance there'], ...
         mean(gT(lo))/max(mean(gT(hi)), realmin), ...
         mean(sqrt(R.se_sigma_ft(iPN,:).^2 + R.se_sigma_ft(iOP,:).^2))));

    B = addInfo(B, 'Fig. 9.14 alone: the sigma gap is widest at LONG tF', ...
        '(the book makes no flight-time claim about the noise miss alone)', ...
        sprintf('sigma gap %.3f -> %.3f ft (first vs last %d of %d tF)', ...
                mean(gS(lo)), mean(gS(hi)), q, numel(R.tf)), ...
        'measured; the nearest book sentence is p.204 (zarchan.txt:2892), which is about the miss distance', ...
        ['REPORTED, NEVER SCORED, and kept because it is the observation that ' ...
         'exposed the mis-aim.  The maneuver miss decays to nothing for both ' ...
         'laws by tF = 10 s so its gap must close; the NOISE miss does not -- ' ...
         'PN sits on a plateau near 1.7 ft while optimal stays flat near ' ...
         '0.7 ft, so the sigma gap is widest at the LONG flight times.  This ' ...
         'was a SCORED row against :2892 and permanently red at 0.589 -> ' ...
         '0.972 ft.  It was not widened and it was not deleted: it was aimed ' ...
         'at a component of the quantity :2892 and :2924 actually name, and ' ...
         'the row that replaced it scores that quantity instead']);
else
    B = addSkip(B, 'Fig. 9.15: the PN-vs-optimal gap is largest at SMALL tF', ...
        'gap(low tF) > gap(high tF)', whyPair, 'p.204 text (zarchan.txt:2892)', 'ft');
    B = addSkip(B, 'miss distance: the PN-vs-optimal gap is largest at SMALL tF', ...
        'gap(low tF) > gap(high tF)', whyPair, 'p.204 text (zarchan.txt:2892)', 'ft');
end

% ---- 6b.  optimal is ~flat in flight time, PN is not -----------------------
if pair
    cv = @(v) std(v)/max(mean(abs(v)), realmin);
    cS = [cv(R.sigma_ft(iPN,:)) cv(R.sigma_ft(iOP,:))];
    B = addShape(B, 'optimal is approximately independent of flight time, PN is not', ...
        'CV(optimal) < CV(PN) on the noise miss', ...
        sprintf('sigma CV %.3f (PN) vs %.3f (optimal); |mean| CV %.3f vs %.3f (reported only)', ...
                cS(1), cS(2), cv(abs(R.mean_ft(iPN,:))), cv(abs(R.mean_ft(iOP,:)))), ...
        cS(2) < cS(1), ...
        'p.204 text (zarchan.txt:2892) -- "optimal guidance performance, unlike that of PN, is approximately independent of flight time"', ...
        ['scored on the NOISE miss only.  The maneuver-miss CV is printed but ' ...
         'not scored: both of those curves decay through zero, so a coefficient ' ...
         'of variation there divides by a mean that is nearly zero and measures ' ...
         'the decay rather than the flatness']);
else
    B = addSkip(B, 'optimal is approximately independent of flight time, PN is not', ...
        'CV(optimal) < CV(PN)', whyPair, 'p.204 text (zarchan.txt:2892)', '-');
end


% ---- 7, 8.  the listing's own plot ranges, as CONTAINMENT bounds -----------
%   APN = 0 at :2896, so both plots at :2923 are the PN curves and their y-axes
%   were chosen to hold them.  An axis is the AUTHOR'S choice of range, not a
%   measurement: this is necessary-not-sufficient, and it is NOT a fitted band.
if ~isempty(iPN)
    mS = max(R.sigma_ft(iPN,:));
    B = addBound(B, 'PN noise miss fits the listing''s sigma plot range', ...
        '0 < max sigma <= 4 ft', sprintf('max %.4f ft at tF = %g s', mS, ...
        R.tf(find(R.sigma_ft(iPN,:) == mS,1))), mS > 0 && mS <= 4, ...
        'listing axis([0,10,0,4]) (zarchan.txt:2923), APN = 0 at :2896', 'ft', ...
        'a plot range, not a published measurement -- containment only; would catch an order-of-magnitude wiring error');
    mM = max(abs(R.mean_ft(iPN,:)));
    B = addBound(B, 'PN maneuver miss fits the listing''s mean plot range', ...
        '0 < max |mean| <= 60 ft', sprintf('max %.4f ft at tF = %g s', mM, ...
        R.tf(find(abs(R.mean_ft(iPN,:)) == mM,1))), mM > 0 && mM <= 60, ...
        'listing axis([0,10,0,60]) (zarchan.txt:2923), APN = 0 at :2896', 'ft', ...
        'same provenance and same caveat as the row above');
else
    B = addSkip(B, 'PN curves fit the listing''s plot ranges', '4 ft / 60 ft', ...
        'law 0 (PN) was not run', 'listing axis(...) (zarchan.txt:2923)', 'ft');
end

% ---- 9.  WHICH ACCELERATION THE FILTER IS FED -- the book contradicts itself
%   :2891 says nL ("the achieved missile acceleration nL rather than the
%   commanded acceleration nc is fed back into the filter"); the listing at
%   :2910 writes nc in all three filter lines.  House rule for a book that
%   contradicts itself: score one, cite both, say why.  What decides it is the
%   book's OWN plot box.  Fig. 9.14 carries both laws and the program that drew
%   it sets axis([0,10,0,4]) at :2923, so the optimal law's noise miss must fit
%   inside 4 ft.  Fed nc it does not, by two orders of magnitude, and the
%   acceleration limiter starts biting; fed nL it lands under the PN curve
%   exactly as the caption claims.  So :2910 cannot be the code that produced
%   the figure, and the port defaults to the prose.
%   The mechanism is visible too: Listing 9.2 (:2817) has NO flight-control lag
%   -- its plant is YDD = XNT - XNC -- so there nc IS the achieved acceleration
%   and its identical filter lines (:2821) are correct.  Listing 9.3 adds the
%   lag (YDD = XNT - XNL at :2902) and the three filter lines were carried over
%   unchanged.  Listings 25.1 (:8595) and 29.3 (:10778), both lagged, write XNL.
FB = R.feedback;
rXNC = FB.sig_opt_xnc/max(FB.sig_pn_xnc, realmin);
rXNL = FB.sig_opt_xnl/max(FB.sig_pn_xnl, realmin);
B = addBound(B, 'the filter must be fed nL (:2891), not nc (:2910)', ...
    'sigma(optimal)/sigma(PN) < 1 with nL and >> 1 with nc', ...
    sprintf('at tF = %g s: nL gives %.3f, nc gives %.1f (%.3f vs %.1f ft against PN %.3f/%.3f)', ...
            FB.tf, rXNL, rXNC, FB.sig_opt_xnl, FB.sig_opt_xnc, FB.sig_pn_xnl, FB.sig_pn_xnc), ...
    rXNL < 1 && rXNC > 10, ...
    'prose zarchan.txt:2891 vs listing :2910; adjudicated by axis([0,10,0,4]) at :2923', 'ratio', ...
    ['the printed listing is not merely different, it is outside the axis box ' ...
     'the same listing draws.  Scored rather than left as a comment so that ' ...
     'flipping filter_accel back cannot pass quietly']);

% ---- INFO / SKIP -----------------------------------------------------------
if ~isempty(iAP) && ~isempty(iPN)
    nbS = sum(R.sigma_ft(iAP,:) < R.sigma_ft(iPN,:));
    nbM = sum(abs(R.mean_ft(iAP,:)) < abs(R.mean_ft(iPN,:)));
    B = addInfo(B, 'augmented PN (APN = 1), reported not scored', ...
        'not published', ...
        sprintf(['vs PN: noise miss smaller at %d/%d tF (peak %.3f vs %.3f ft), ' ...
                 'maneuver miss smaller at %d/%d tF (peak %.3f vs %.3f ft)'], ...
                nbS, numel(R.tf), max(R.sigma_ft(iAP,:)), max(R.sigma_ft(iPN,:)), ...
                nbM, numel(R.tf), max(abs(R.mean_ft(iAP,:))), max(abs(R.mean_ft(iPN,:)))), ...
        'listing branch at zarchan.txt:2910', ...
        ['the book''s figures compare APN = 0 and APN = 2 only (:2892), so there ' ...
         'is nothing here to score against.  Reported because a previous wave ' ...
         'measured augmented PN as WORSE than plain PN and attributed it to a ' ...
         'NON-maneuvering target, where the augmentation term is pure estimator ' ...
         'noise.  This listing''s target does maneuver (constant 3 g from t = 0, ' ...
         ':2892) and the augmented law STILL measures worse on the noise miss, ' ...
         'so that explanation does not carry over: the .5*N''*XNTH term injects ' ...
         'the filter''s target-acceleration estimate, noise and all, straight ' ...
         'into the command.  What it buys is maneuver miss, and only where the ' ...
         'maneuver miss is large.  Optimal guidance beats both because it uses ' ...
         'the same estimate AND cancels the airframe lag']);
end
B = addInfo(B, 'which side of the contradiction this run flew', ...
    'prose :2891 = nL / listing :2910 = nc', ...
    sprintf('filter_accel = ''%s''%s', opts.filter_accel, ...
            repmat(' (the printed page, NOT the figures)', 1, double(strcmp(opts.filter_accel,'XNC')))), ...
    'prose zarchan.txt:2891 vs listing :2910', ...
    ['row 9 above is where that choice is adjudicated.  Pass ' ...
     'filter_accel = ''XNC'' to reproduce the printed page exactly, including ' ...
     'the divergence -- which is a legitimate thing to want and a ruinous ' ...
     'thing to get by accident']);
if opts.runs >= 50 && ~isempty(iPN)
    B = addInfo(B, 'the book''s OWN 50-run set (first 50 replications)', ...
        '50 runs (zarchan.txt:2892, RUN = 50 at :2896)', ...
        sprintf('PN sigma %.3f ft / %d-run %.3f ft at tF = %g s', ...
                R.sigma50_ft(iPN,end), opts.runs, R.sigma_ft(iPN,end), R.tf(end)), ...
        'p.204 text (zarchan.txt:2892)', ...
        ['reported, never adjudicated: a 50-run sample standard deviation carries ' ...
         '~10% sampling error of its own answer, so "does the 50-run set match" ' ...
         'is a question about the draw, not about the port']);
end
B = addSkip(B, 'the miss values plotted on Figs 9.14/9.15', 'not published', ...
    ['no number from either figure survives extraction -- every figure became an ' ...
     'image link, and the surrounding prose (:2892, :2924) states only ORDERINGS. ' ...
     'Rows 3-6 score those orderings; inventing a figure reading is what this ' ...
     'skip exists to refuse'], 'Figs. 9.14/9.15 (zarchan.txt:2925, :2934)', 'ft');
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want.  Schema is the
%   one mgs.zarchan.benchSuite's adaptGeneric consumes; it is STRICT.
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
p = R.p;  L = {};
L{end+1} = 'Zarchan Listing 9.3 -- MONTE CARLO ENGAGEMENT TESTING OPTIMAL GUIDANCE';
L{end+1} = ['*** UNITS: ' R.units '.  NOTHING HERE IS SI. ***'];
L{end+1} = sprintf(['   VC = %g ft/s  nT = %g ft/s^2 (%.4g g)  N'' = %g  TAU = %g s  ' ...
    'TS = %g s  H = %g s'], p.VC, p.XNT, p.XNT/32.2, p.XNP, p.TAU, p.TS, p.H);
L{end+1} = sprintf(['   sigma_noise = %g rad (%.4g mr)  XLIM = %g ft/s^2  runs = %d  ' ...
    'seed = %d  filter feedback = %s'], p.SIGNOISE, 1000*p.SIGNOISE, p.XLIM, ...
    p.runs, p.seed, p.filter_accel);
L{end+1} = sprintf('   tF sweep: %g : %g : %g s (%d points), laws APN = %s', ...
    R.tf(1), (numel(R.tf)>1)*(R.tf(min(2,end))-R.tf(1)), R.tf(end), numel(R.tf), mat2str(R.laws));
nm = {'PN (APN=0)','augmented PN (APN=1)','optimal (APN=2)'};
L{end+1} = '   miss at the sweep ends:';
for a = 1:numel(R.laws)
    L{end+1} = sprintf('     %-22s sigma %8.4f -> %8.4f ft   mean %+9.4f -> %+9.4f ft', ...
        nm{R.laws(a)+1}, R.sigma_ft(a,1), R.sigma_ft(a,end), R.mean_ft(a,1), R.mean_ft(a,end)); %#ok<AGROW>
end
L{end+1} = sprintf(['   filter fed nL vs nc at tF = %g s: sigma(optimal) = %.3f vs %.1f ft ' ...
    '(PN %.3f / %.3f)'], R.feedback.tf, R.feedback.sig_opt_xnl, R.feedback.sig_opt_xnc, ...
    R.feedback.sig_pn_xnl, R.feedback.sig_pn_xnc);
L{end+1} = '   STRUCTURAL:';
for i = 1:numel(R.selfcheck.names)
    L{end+1} = sprintf('     [%s] %s -- %s', tick(R.selfcheck.oks(i)), ...
        R.selfcheck.names{i}, R.selfcheck.detail{i}); %#ok<AGROW>
end
L{end+1} = '   BENCHMARKS:';
for i = 1:numel(R.bench)
    b = R.bench(i);
    if b.skipped
        L{end+1} = sprintf('     [%-4s] %s -- %s', upper(b.kind), b.what, b.why); %#ok<AGROW>
    else
        L{end+1} = sprintf('     [%s] %-62s book %-28s ours %s   (%s)', tick(b.ok), ...
            b.what, b.book_str, b.ours_str, b.source); %#ok<AGROW>
    end
end
if any(ev)
    L{end+1} = sprintf('   %d rows scored, %d passed, %d info/skip.', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev));
end
s = strjoin(L, newline);
end

function c = tick(ok)
if ok, c = ' OK '; else, c = 'FAIL'; end
end

function drawCurves(R)
nm = {'PN','augmented PN','optimal'};
figure; subplot(2,1,1); hold on; grid on;
for a = 1:numel(R.laws), plot(R.tf, R.sigma_ft(a,:), '-o', 'DisplayName', nm{R.laws(a)+1}); end
xlabel('Flight Time (s)'); ylabel('Noise Miss Std Dev (ft)'); legend show;
title(sprintf('Listing 9.3, %d runs, seed %d', R.p.runs, R.p.seed));
subplot(2,1,2); hold on; grid on;
for a = 1:numel(R.laws), plot(R.tf, R.mean_ft(a,:), '-o', 'DisplayName', nm{R.laws(a)+1}); end
xlabel('Flight Time (s)'); ylabel('Mean Miss (ft)'); legend show;
end
