function R = weaveExtendedKalman(opts)
%MGS.ZARCHAN.WEAVEEXTENDEDKALMAN  Zarchan Listing 25.3 -- the FIVE-STATE
%   EXTENDED Kalman filter in the homing loop.  Listing 25.2 is TOLD the
%   target's weave frequency; this one has to ESTIMATE it, from line-of-sight
%   angle alone, and the chapter is about what that costs.
%
%   R = mgs.zarchan.weaveExtendedKalman()
%   R = mgs.zarchan.weaveExtendedKalman('SIGRIN', .001)        1 mr, Fig. 25.18
%   R = mgs.zarchan.weaveExtendedKalman('WHIC', 3)             Fig. 27.2
%   R = mgs.zarchan.weaveExtendedKalman('aux', false)          main run only
%
%   Line-by-line port of Listing 25.3 (zarchan.txt:9147-9163), including its
%   subroutine PROJECT, which brute-force Euler-integrates the NONLINEAR state
%   equations forward one sampling interval because the fundamental matrix is
%   only a two-term Taylor series.  The book says exactly that at :9100.
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, g = 32.2 ft/s^2; rad/s ("r/s"
%   in the book); LOS noise in rad (1 mr = .001).  NOT SI.  The rest of mgs is
%   metres -- convert AT THE CALL SITE and say so there.
% ===========================================================================
%
%   TRANSCRIPTION REPAIRS.  listing_25_03.m does not parse.  All forced by
%   shape except #4, which is a judgement and is argued:
%     1  "while T ,=(TF-.0001)"  -> "<=".  "," is this book's OCR of "<"; T
%        only increases, so ">=" never terminates.
%     2  "if S .=(TS-.00001)"    -> ">=".  "." is the OCR of ">"; S is zeroed
%        on every record, so "<=" would record every integration step.
%        SELF-CHECK 4 counts the updates and pins this to 1000, not 10000.
%     3  "while T ,=(TS-.0001)" inside PROJECT -> "<=".  Ten Euler steps.
%     4  *** THE ONE UNRESOLVED COMPARISON *** the transcription header flags:
%        "if XNC .XNCLIM, XNC=XNCLIM".  Resolved as "XNC > XNCLIM" because the
%        NEXT line is "if XNC < -XNCLIM, XNC = -XNCLIM": the pair is a
%        symmetric limiter, and "<" would clamp the command to the limit from
%        below on every step.  At the listing's own XNCLIM = 9999999 neither
%        branch can fire, so this cannot move a scored row here -- SELF-CHECK 5
%        counts the clamps and shows the count is 0.  It matters only to a
%        caller who sets a real limit (the book's 10-g cases, Figs 25.30/32).
%     5  "X=W*T" on the init line is DEAD (X is 25.2's fundamental-matrix
%        argument; 25.3 builds PHI from TS).  PHASE/PHASEDEG, QEKF, TS2..TS7
%        are likewise never referenced.  Dropped, not "fixed" into code.
%     6  PROJECT's first argument TP is dead (its first statement is T = 0).
%        Dropped from the signature.  Plot/save tail dropped; nothing here
%        writes to disk.
%     7  The listing's inv() of a 1-by-1 is written as a division.  Identical.
%
%   *** MINUS-SIGN DAMAGE, DECODED AND CONFIRMED. ***  Prose at :9101 reads
%   "initialized wrong to 21 r/s (that is, WHIC = 21)".  That is the book's
%   minus sign mis-mapped to the digit 2: it is -1 r/s.  Not a guess -- the
%   listing's own first line, :9147, prints "WHIC=-1.".  BENCHMARK 1 scores it
%   because it is the cheapest standing proof the decode was done, not skipped.
%   The other number in the passage, "2.5 r/s rather than 2 r/s" (:9102), was
%   checked for the same damage and is NOT damaged: "-.5" also renders as
%   "2.5", but the sentence calls the value "nearly correct" against a truth of
%   2 r/s, which -0.5 is not.  The sign is discussed separately in the same
%   sentence ("but the sign is wrong"), so the book's steady-state estimate is
%   about -2.5 rad/s.
%
%   *** ONE CONTRADICTION, SCORED ONE WAY. ***  The printed listing sets
%   SIGRIN = .0001 (0.1 mr), but the prose immediately after calls the nominal
%   case "1 mr" (:9102) and only later reduces it "by an order of magnitude to
%   .1 mr" (:9112).  The PRINTED listing is therefore already the reduced-noise
%   variant.  This port defaults to the printed line and runs 1 mr as an
%   AUXILIARY case, because the 2.5 rad/s value belongs to 1 mr.  Both lines
%   are cited on the row.
%
%   *** THE BOARD WAS WRONG HERE UNTIL 2026-08-24, AND THE NEXT READER NEEDS
%   TO KNOW IT.  ROWS 5 AND 6 WERE SCORING ONE NOISE DRAW. ***  They used to
%   run WHIC = 3 and WHIC = 4 ONCE EACH, at this file's single default seed,
%   and they printed the book's two outcomes INVERTED: WHIC = 3 -- the SMALLER
%   initialisation error, which :9952 says recovers -- never locked, while
%   WHIC = 4 -- the LARGER error, which :9952 says diverges -- locked at
%   2.95 s.  That is the opposite of the book and it has the exact signature
%   of a crossed pair of rows.  IT IS NOT ONE.  Measured, not re-derived:
%     * THE WIRING IS RIGHT.  RUNS.whic3 really begins at w_hat = +3.0000 and
%       RUNS.whic4 at +4.0000, against a truth W = +2 (the listing's own
%       "W=2." at :9147, and Ch.27 restates it at :9952: "the actual target
%       weave frequency was 2 r/s").  So +1 and +2 rad/s of error, exactly as
%       the two rows assume: both rows were aimed at the right experiment.
%     * THE TRANSCRIPTION IS RIGHT.  PHI(4,5) = -2*WH*YTDDH*TS, the Q entries,
%       RES = RTM*XLAMS - YB and the two-pass predictor all match :9147-9163
%       entry for entry.
%     * THE LOCK GATE IS NOT THE CULPRIT.  "Thereafter" gates can turn a
%       lock-then-wander run into Inf, but that is not what happens here: at
%       the default seed w_hat for WHIC = 3 crosses the [1.5, 2.5] gate only
%       as a TRANSIENT -- 29 of 1000 samples, all inside t = 1.76-2.25 s --
%       and then settles: for t >= 3.5 s it stays in [3.096, 3.456] and
%       never re-enters the gate.  So the Inf is not the gate discarding a
%       lock, it is a genuine failure to converge on that draw.  (CORRECTED
%       2026-08-24: this block previously claimed the trace 'sits between
%       +3.1 and +3.6 for the whole flight and never enters [1.5, 2.5] at
%       all'.  Measured, the full-flight range is [+1.119, +4.521] and 261
%       of 1000 samples lie outside [3.1, 3.6].  The conclusion is
%       unchanged -- the settled value is +3.20, not +2 -- but the
%       sentence supporting it was false as written.)
%     * THE SIGN IS NOT THE CULPRIT either.  Both cases stay POSITIVE; the
%       wrong-sign behaviour is the WHIC = -1 nominal, and rows 2/3 already
%       handle it by scoring |w_hat| and the sign separately.
%   WHAT IS ACTUALLY TRUE IS THAT WHICH WAY EACH CASE GOES IS A PROPERTY OF
%   THE NOISE DRAW, NOT OF THE INITIALISATION ALONE.  Over 100 seeds:
%   WHIC = 3 recovers on 58 of them and ends with median |w_hat_ss - W| =
%   0.22 rad/s; WHIC = 4 recovers on 4 and ends at median +7.82 rad/s, 291% of
%   W out.  PAIRED seed by seed, WHIC = 3 ends closer to the truth than
%   WHIC = 4 on 97 of 100 draws, and among the WHIC = 3 draws that do recover
%   the median recovery time is 3.00 s -- which is the number :9952 states.
%   The default seed 20260824 is one of the ~2% of draws that lands in BOTH
%   tails at once.  Zarchan's Fig. 27.2 is likewise ONE realisation of HIS
%   randn, which cannot be reproduced from a different generator, so a
%   one-draw row could never have tested that sentence in the first place.
%   A SECOND CONTRADICTION ON THE SAME PAGE, MEASURED AND FOUND INERT:
%   Ch.27's Table 27.1 (:9939-9940) lists the missile acceleration limit as
%   10 g for these runs, while the Listing 25.3 that :9952 says was run prints
%   XNCLIM = 9999999, i.e. no limit at all.  It does not matter here -- at
%   XNCLIM = 322 ft/s^2 (10 g) the ensemble is 19/24 recoveries at a median
%   2.97 s against 19/24 at 2.98 s unlimited, and the WHIC = 4 side is
%   unchanged at 1/24 and 5.70 rad/s.  Measured, not assumed; the port stays
%   on the printed listing.
%   ROWS 5 AND 6 ARE THEREFORE RE-AIMED at a fixed 24-seed ensemble (see
%   whicEnsemble).  The default seed is deliberately LEFT ALONE: moving it
%   until the single draw came out green would be scoring a different draw,
%   which is the same mistake with a nicer colour.  Both rows still print
%   their inverted single-draw number, and so does the summary.
%
%   WHAT IT IS NOT: a linearised planar homing loop, constant closing velocity,
%   single-lag flight control, no gravity/drag/seeker dynamics, target weave
%   amplitude known.  Nothing here predicts an aircraft.
%
%   WHAT THIS PRINTS (*** MEASURED 2026-08-24; the "derived, never executed"
%   disclaimer this block used to carry is retired ***): 1000 Kalman updates,
%   0 command clamps, w_hat starting at -1 and settling NEGATIVE at -2.085
%   rad/s (0.1 mr) and -2.314 rad/s (1 mr), so the |w_hat| error is 0.085 vs
%   0.314 -- smaller at the lower noise, as :9112 says.  Default call is about
%   120 ms, of which about 100 ms is the 24-seed x 2-case WHIC ensemble.
%   Every band below is stated in the row; do not widen one to turn a row
%   green, and do not move the seed to do it either.
%
%   See also MGS.ZARCHAN.WEAVEKALMANFILTER (25.2, frequency known),
%   MGS.ZARCHAN.WEAVEKALMANTHREESTATE (25.1), MGS.ZARCHAN.BENCHSUITE.

arguments
    opts.SIGRIN (1,1) double {mustBeFinite, mustBeNonnegative} = 1e-4  % rad
    opts.WHIC   (1,1) double {mustBeFinite} = -1.0                     % rad/s
    opts.PHIS2  (1,1) double {mustBeFinite, mustBeNonnegative} = 0.0
    opts.W      (1,1) double {mustBeFinite, mustBePositive} = 2.0      % rad/s
    opts.XNT    (1,1) double {mustBeFinite, mustBePositive} = 96.6     % 3 g
    opts.TAU    (1,1) double {mustBeFinite, mustBePositive} = 0.5      % s
    opts.TF     (1,1) double {mustBeFinite, mustBePositive} = 10.0     % s
    opts.TS     (1,1) double {mustBeFinite, mustBePositive} = 0.01     % s
    opts.H      (1,1) double {mustBeFinite, mustBePositive} = 0.001    % s
    opts.HP     (1,1) double {mustBeFinite, mustBePositive} = 0.001    % s
    opts.VC     (1,1) double {mustBeFinite, mustBePositive} = 9000     % ft/s
    opts.VM     (1,1) double {mustBeFinite, mustBePositive} = 3000     % ft/s
    opts.XNP    (1,1) double {mustBeFinite, mustBePositive} = 3.0
    opts.APN    (1,1) double {mustBeMember(opts.APN,[0 1 2 3 4])} = 4
    opts.XNCLIM (1,1) double {mustBeFinite, mustBePositive} = 9999999
    opts.SIGGL  (1,1) double {mustBeFinite, mustBeNonnegative} = 0
    opts.SRN    (1,1) double {mustBeFinite, mustBeNonnegative} = 0
    opts.RA     (1,1) double {mustBeFinite, mustBePositive} = 21000    % ft
    opts.HEDEG  (1,1) double {mustBeFinite} = 0                        % deg
    opts.QPERFECT (1,1) logical = false
    % Deterministic LCG + Box-Muller inside this file.  rand/randn would move
    % when someone else calls rng(), and setting the global stream would reach
    % into the caller's session mid-suite.
    opts.seed   (1,1) double {mustBeFinite, mustBeNonnegative} = 20260824
    % THE ONLY SWITCHES.  Not nargin: with an arguments block that declares
    % only name-value options MATLAB reports nargin = 0 however many were
    % passed, so an "if nargin == 0" mode would never fire.
    opts.aux       (1,1) logical = true     % the six comparison runs
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.assert    (1,1) logical = true
end

if opts.H > opts.TS
    error('mgs:zarchan:weaveExtendedKalman:step', ...
        ['H = %g s exceeds the sampling interval TS = %g s, so the filter ' ...
         'would update less often than once per integration step and the ' ...
         'engagement would still run smoothly and wrongly.'], opts.H, opts.TS);
end
if opts.HP > opts.TS
    error('mgs:zarchan:weaveExtendedKalman:project', ...
        'HP = %g s exceeds TS = %g s: PROJECT would take zero Euler steps.', ...
        opts.HP, opts.TS);
end

p = opts;
p.freq_known = false;   % pin w_hat at the truth (the 25.2-style twin)
p.kill_jac   = false;   % delete PHI(4,5), the frequency Jacobian
p.strict     = true;    % refuse to continue on a non-finite command

A = runEKF(p);                                  % the printed listing, 0.1 mr

% NOT struct('nominal', A): struct() expands a struct ARRAY on the right into
% an array of results, so that idiom is one refactor away from silently
% returning something of the wrong size.
RUNS = struct();  RUNS.nominal = A;
if opts.aux
    q = p;  q.strict = false;
    b = q;  b.SIGRIN = 1e-3;                    RUNS.mr1     = runEKF(b);
    c = q;  c.WHIC   = 3;                       RUNS.whic3   = runEKF(c);
    d = q;  d.WHIC   = 4;                       RUNS.whic4   = runEKF(d);
    e = q;  e.freq_known = true;                RUNS.known   = runEKF(e);
    f = q;  f.PHIS2 = 1;                        RUNS.phis2   = runEKF(f);
    g = q;  g.kill_jac = true;                  RUNS.nojac   = runEKF(g);
end

R = struct();
R.t = A.t;  R.WH = A.WH;  R.err_w = opts.W - A.WH;
R.acc_g = A.acc_g;  R.acc_hat_g = A.accH_g;
R.jerk_g = A.jerk_g;  R.jerk_hat_g = A.jerkH_g;
R.sp55 = A.sp55;  R.miss_ft = A.miss_ft;
R.wss_signed = A.wss_signed;  R.wss = A.wss;
R.rms_acc_g = A.rms_acc_g;
R.runs = RUNS;
R.units = ['ft, ft/s, ft/s^2, s, rad/s; g = 32.2 ft/s^2; ' ...
           'LOS noise in rad (1 mr = .001)'];
R.p = struct('SIGRIN',opts.SIGRIN, 'WHIC',opts.WHIC, 'PHIS2',opts.PHIS2, ...
    'W',opts.W, 'XNT',opts.XNT, 'TAU',opts.TAU, 'TF',opts.TF, 'TS',opts.TS, ...
    'H',opts.H, 'HP',opts.HP, 'VC',opts.VC, 'VM',opts.VM, 'XNP',opts.XNP, ...
    'APN',opts.APN, 'XNCLIM',opts.XNCLIM, 'PHIS1',A.PHIS1, 'seed',opts.seed, ...
    'aux',opts.aux, 'n_updates',A.n, 'n_clamps',A.nclip, ...
    'win_ss',[0.5*opts.TF 0.9*opts.TF], 'win_rms',[0.4*opts.TF opts.TF], ...
    'units',R.units);

R.selfcheck = structuralChecks(p, A, RUNS, opts.aux);

if opts.benchmark
    [R.bench, R.whic_ensemble] = benchmarks(A, RUNS, opts, p);
    ev = ~[R.bench.skipped];
    R.pass = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();  ev = false;
    R.whic_ensemble = struct('ran', false);
    R.pass = false;          % NOT CHECKED must never render as fine
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false: nothing was compared. R.pass = NOT CHECKED.';
elseif ~any(ev)
    msg = 'no book value could be evaluated (run with ''aux'' true).';
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port ' ...
        'disagrees with ITSELF, a wiring fault, not a tolerance question. ' ...
        'First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf('%d of %d evaluated book values outside tolerance (first: %s, book %s, ours %s, band %s).', ...
        numel(bad), sum(ev), R.bench(bad(1)).what, R.bench(bad(1)).book_str, ...
        R.bench(bad(1)).ours_str, R.bench(bad(1)).band);
end
R.verdict = msg;
if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:weaveExtendedKalman:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:weaveExtendedKalman:benchmark', ...
            '%s  (''assert'' is false; R is returned complete, see R.verdict)', msg);
    end
end
end


% =============================================================================
function O = runEKF(p)
%RUNEKF  Listing 25.3, zarchan.txt:9147-9163.  Every constant is an ARGUMENT
%   via p: MATLAB local functions cannot see the caller's workspace.
XNT=p.XNT; W=p.W; TF=p.TF; TS=p.TS; H=p.H; HP=p.HP; VC=p.VC; VM=p.VM;
TAU=p.TAU; XNP=p.XNP; APN=p.APN; XNCLIM=p.XNCLIM;
SIGRIN=p.SIGRIN; SIGGL=p.SIGGL; SRN=p.SRN; RA=p.RA; PHIS2=p.PHIS2;
PHIS1 = W*W*XNT*XNT/TF;              % the listing's own line 1
ORDER = 5;

T=0; S=0; Y=0; YD = -XNT/W - VM*p.HEDEG/57.3;
YTDD = XNT*sin(W*T);  YTDDD = XNT*W*cos(W*T);
XNC=0; XNL=0; XNLD=0; YDD=0;
WH = p.WHIC;
if p.QPERFECT
    YH=Y; YDH=YD; YTDDH=YTDD; YTDDDH=YTDDD; WH=W;
else
    YH=0; YDH=0; YTDDH=0; YTDDDH=0;
end
if p.freq_known, WH = W; end

% The listing's "PHI=zeros(ORDER); Q=zeros(ORDER)" is dropped: it exists only
% because the listing assigns a FIXED subset of entries each cycle and relies
% on the rest staying zero, and phiMat/qMat build the whole matrix from zeros
% every call.  Same values, and it cannot rot into a stale-entry bug.
P=zeros(ORDER); IDNP=eye(ORDER);
RTM = VC*TF;
SIGNOISE = noiseSigma(SIGRIN, SIGGL, SRN, RTM, RA);
YNOISE = SIGNOISE*RTM;
P(1,1)=YNOISE*YNOISE; P(2,2)=(VM*20/57.3)^2; P(3,3)=XNT*XNT;
P(4,4)=(W*XNT)^2;     P(5,5)=W*W;
if p.freq_known, P(5,5)=0; end
HMAT=[1 0 0 0 0]; HT=HMAT.';

nMax = ceil(TF/TS)+4;
z = lcgNormal(p.seed, nMax);
t=zeros(nMax,1); aWH=t; aA=t; aAH=t; aJ=t; aJH=t; aS55=t;
n=0; nclip=0; blew=false; p55rise=0;

while T <= (TF - 0.0001)
    YOLD=Y; YDOLD=YD; XNLOLD=XNL;
    STEP=1; FLAG=0;
    while STEP <= 1                     % Zarchan's two-pass predictor loop
        if FLAG==1
            STEP=2;
            Y=Y+H*YD; YD=YD+H*YDD; XNL=XNL+H*XNLD; T=T+H;
        end
        YTDD = XNT*sin(W*T);
        TGO  = TF-T+0.00001;
        XNLD = (XNC-XNL)/TAU;
        YDD  = YTDD-XNL;
        FLAG = 1;
    end
    Y   = 0.5*(YOLD+Y+H*YD);            % Heun corrector, the listing's algebra
    YD  = 0.5*(YDOLD+YD+H*YDD);
    XNL = 0.5*(XNLOLD+XNL+H*XNLD);

    S = S + H;
    if S >= (TS - 0.00001)              % repair 2; SELF-CHECK 4 pins the count
        S = 0;
        YTDD  = XNT*sin(W*T);
        YTDDD = XNT*W*cos(W*T);
        PHI = phiMat(WH, YTDDH, TS, p.kill_jac);
        Q   = qMat(WH, YTDDH, TS, PHIS1, PHIS2);
        M   = PHI*P*PHI.' + Q;
        RTM = VC*TGO;
        SIGNOISE = noiseSigma(SIGRIN, SIGGL, SRN, RTM, RA);
        YNOISE   = SIGNOISE*RTM;
        GAIN = (M*HT)/(HMAT*M*HT + YNOISE^2);       % inv() of a 1-by-1
        P55old = P(5,5);
        P      = (IDNP - GAIN*HMAT)*M;
        p55rise = max(p55rise, P(5,5) - P55old);

        n = n + 1;
        XLAM  = Y/RTM;
        XNOISE = SIGNOISE*z(n);
        XLAMS = XLAM + XNOISE;
        [YB,YDB,YTDDB,YTDDDB] = project(TS, YH, YDH, YTDDH, YTDDDH, HP, XNL, WH);
        RES   = RTM*XLAMS - YB;
        YH    = YB     + GAIN(1)*RES;
        YDH   = YDB    + GAIN(2)*RES;
        YTDDH = YTDDB  + GAIN(3)*RES;
        YTDDDH= YTDDDB + GAIN(4)*RES;
        if ~p.freq_known
            WH = WH + GAIN(5)*RES;
        end

        XNC = guidanceLaw(APN, XNP, TGO, TAU, YH, YDH, YTDDH, YTDDDH, XNL, WH);
        if ~isfinite(XNC)
            % The book's APN 3/4 kernels divide by WH^2 and WH^3.  A w_hat that
            % reaches 0 (or a diverged run) makes XNC NaN, and a NaN command
            % propagates silently through the whole engagement.  Refuse rather
            % than continue -- for aux runs 'strict' is off and the run is
            % marked, which is itself the evidence a divergence row wants.
            if p.strict
                error('mgs:zarchan:weaveExtendedKalman:nonfinite', ...
                    ['guidance command is not finite at t = %.3f s ' ...
                     '(w_hat = %g rad/s, APN = %d).'], T, WH, APN);
            end
            % n was advanced to index the noise draw but nothing has been
            % recorded at it yet; step it back so the returned arrays contain
            % no unwritten (zero) sample masquerading as data.
            n = n - 1;  blew = true;  break
        end
        if XNC >  XNCLIM, XNC =  XNCLIM; nclip = nclip+1; end   % repair 4
        if XNC < -XNCLIM, XNC = -XNCLIM; nclip = nclip+1; end

        t(n)=T; aWH(n)=WH; aA(n)=YTDD/32.2; aAH(n)=YTDDH/32.2;
        aJ(n)=YTDDD/32.2; aJH(n)=YTDDDH/32.2; aS55(n)=sqrt(max(P(5,5),0));
    end
end

O = struct();
O.t=t(1:n); O.WH=aWH(1:n); O.acc_g=aA(1:n); O.accH_g=aAH(1:n);
O.jerk_g=aJ(1:n); O.jerkH_g=aJH(1:n); O.sp55=aS55(1:n);
O.n=n; O.nclip=nclip; O.blew=blew; O.p55rise=p55rise; O.PHIS1=PHIS1;
O.miss_ft=abs(Y); O.WHIC=p.WHIC; O.SIGRIN=SIGRIN; O.PHIS2=PHIS2;

% Steady state = MEDIAN over [0.5 tF, 0.9 tF].  Median because the endgame
% (TGO -> 1e-5) drives the measurement variance to ~1e-10 and the gains to
% their zero-noise limit, so a mean over the whole flight is dominated by the
% last few samples.  The window ends at 0.9 tF for the same reason.
w1 = O.t >= 0.5*TF & O.t <= 0.9*TF;
if any(w1)
    O.wss_signed = median(O.WH(w1));
    O.wss = abs(O.wss_signed);
else
    O.wss_signed = NaN;  O.wss = NaN;
end
O.werr = abs(O.wss - W);

w2 = O.t >= 0.4*TF;                    % "after a transient period", :9112-9124
if any(w2)
    O.rms_acc_g  = sqrt(mean((O.acc_g(w2)  - O.accH_g(w2)).^2));
    O.rms_jerk_g = sqrt(mean((O.jerk_g(w2) - O.jerkH_g(w2)).^2));
else
    O.rms_acc_g = NaN;  O.rms_jerk_g = NaN;
end
O.t_lock = lockTime(O.t, O.WH, W, 0.25, 0.9*TF);
O.wh_frozen_dev = max(abs(O.WH - p.WHIC));
end


% =============================================================================
function [YB,YDB,YTDDB,YTDDDB] = project(TS, YPH, YDPH, YTDDPH, YTDDDPH, HP, XNLP, WPH)
%PROJECT  The listing's own subroutine.  Euler, SEQUENTIAL (each line uses the
%   value the previous line just wrote).  Reordering these six lines is a
%   silent change of scheme; SELF-CHECK 1 pins the order to machine precision.
T=0; Y=YPH; YD=YDPH; YTDD=YTDDPH; YTDDD=YTDDDPH; W=WPH; XNL=XNLP; H=HP;
while T <= (TS - 0.0001)
    YTDDDD = -W*W*YTDD;
    YTDDD  = YTDDD + H*YTDDDD;
    YTDD   = YTDD  + H*YTDDD;
    YDD    = YTDD  - XNL;
    YD     = YD    + H*YDD;
    Y      = Y     + H*YD;
    T      = T + H;
end
YB=Y; YDB=YD; YTDDB=YTDD; YTDDDB=YTDDD;
end


% =============================================================================
function PHI = phiMat(WH, YTDDH, TS, killJac)
%PHIMAT  The two-term Taylor fundamental matrix, evaluated at the CURRENT
%   estimates.  (4,3) = -w_hat^2*Ts is EVEN in w_hat; (4,5) is the frequency
%   Jacobian and is the only entry that carries the sign -- which is why
%   deleting it (SELF-CHECK 3) freezes the frequency estimate dead.
PHI = zeros(5);
PHI(1,1)=1; PHI(1,2)=TS; PHI(2,2)=1; PHI(2,3)=TS; PHI(3,3)=1; PHI(3,4)=TS;
PHI(4,3)=-WH*WH*TS; PHI(4,4)=1; PHI(4,5)=-2*WH*YTDDH*TS; PHI(5,5)=1;
if killJac, PHI(4,5)=0; end
end

function Q = qMat(WH, YTDDH, TS, PHIS1, PHIS2)
%QMAT  Discrete process noise, zarchan.txt:9080-9090, at the current estimates.
Q = zeros(5);
Q(3,3)=PHIS1*TS*TS*TS/3;
Q(3,4)=PHIS1*TS*TS/2;   Q(4,3)=Q(3,4);
Q(4,4)=4*WH*WH*YTDDH*YTDDH*PHIS2*TS*TS*TS/3 + PHIS1*TS;
Q(4,5)=-WH*YTDDH*TS*TS*PHIS2;  Q(5,4)=Q(4,5);
Q(5,5)=PHIS2*TS;
end

function s = noiseSigma(SIGRIN, SIGGL, SRN, RTM, RA)
s = sqrt(SIGRIN^2 + (SIGGL/RTM)^2 + (SRN*RTM*RTM/(RA*RA))^2);
end

function XNC = guidanceLaw(APN, XNP, TGO, TAU, YH, YDH, YTDDH, YTDDDH, XNL, WH)
%GUIDANCELAW  The listing's five options: PN, APN, optimal, weave, compensated
%   weave (APN = 0..4).  The listing's cancellation-prone kernels are kept AS
%   PRINTED -- (1-cos x)/x^2 at x ~ 2.5e-5 loses about 6 digits of 16 and the
%   book runs it anyway; "fixing" it would stop this being a transcription.
switch APN
    case 0
        XNC = XNP*(YH+YDH*TGO)/(TGO*TGO);
    case 1
        XNC = XNP*(YH+YDH*TGO+0.5*YTDDH*TGO*TGO)/(TGO*TGO);
    case 2
        XS = TGO/TAU;  XNPP = optimalRatio(XS);
        C1 = XNPP/(TGO*TGO);
        C2 = XNPP/TGO;
        C3 = 0.5*XNPP;
        C4 = -XNPP*(exp(-XS)+XS-1)/(XS*XS);
        XNC = C1*YH + C2*YDH + C3*YTDDH + C4*XNL;
    case 3
        XP = WH*TGO;
        XNC = XNP*(YH+YDH*TGO)/(TGO*TGO) + XNP*YTDDH*(1-cos(XP))/XP^2 ...
            + XNP*YTDDDH*(XP-sin(XP))/(XP*XP*WH);
    otherwise
        XS = TGO/TAU;  XNPP = optimalRatio(XS);
        C1 = XNPP/(TGO*TGO);
        C2 = XNPP/TGO;
        C3 = XNPP*(1-cos(WH*TGO))/(WH*WH*TGO*TGO);
        C4 = -XNPP*(exp(-XS)+XS-1)/(XS*XS);
        C5 = XNPP*(WH*TGO-sin(WH*TGO))/(WH*WH*WH*TGO*TGO);
        XNC = C1*YH + C2*YDH + C3*YTDDH + C4*XNL + C5*YTDDDH;
end
end

function XNPP = optimalRatio(XS)
TOP  = 6*XS*XS*(exp(-XS)-1+XS);
BOT1 = 2*XS*XS*XS+3+6*XS-6*XS*XS;
BOT2 = -12*XS*exp(-XS)-3*exp(-2*XS);
XNPP = TOP/(0.0001+BOT1+BOT2);      % the .0001 guard is the book's
end

function tl = lockTime(t, WH, W, frac, tEnd)
%LOCKTIME  Earliest time after which |w_hat - W| stays within frac*W for the
%   rest of the window.  Inf if it never does.
k = find(t <= tEnd, 1, 'last');
if isempty(k), tl = Inf; return, end
gate = abs(WH(1:k) - W) <= frac*W;
bad = find(~gate, 1, 'last');
if isempty(bad),  tl = t(1);
elseif bad < k,   tl = t(bad+1);
else,             tl = Inf;
end
end

function z = lcgNormal(seed, n)
%LCGNORMAL  Deterministic standard normals, base MATLAB, and it does NOT touch
%   the global random stream.  MINSTD x <- 16807*x mod (2^31-1) is exact in
%   double.  Box-Muller pairs come from SEPARATE HALVES of the stream:
%   consecutive values of a multiplicative generator lie on a 2-D lattice and
%   feeding a consecutive pair into Box-Muller puts a visible spiral in it.
m = 2147483647;
x = mod(floor(abs(seed)), m-1) + 1;
np = 2*ceil(n/2);
u = zeros(1,np);
for i = 1:np
    x = mod(16807*x, m);
    u(i) = x/m;
end
h  = np/2;
r  = sqrt(-2*log(u(1:h)));
th = 2*pi*u(h+1:np);
z  = zeros(1,np);
z(1:h) = r.*cos(th);  z(h+1:np) = r.*sin(th);
z = z(1:n);
end


% =============================================================================
function K = structuralChecks(p, A, RUNS, aux)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and must never be quoted
%   as evidence this port reproduces Zarchan.  .names and .oks are what
%   benchSuite puts on the board -- an invariant computed without them is an
%   invariant nobody can see, which is the same as not running it.
names = {};  oks = [];  detail = {};

% 1  PROJECT's sequential Euler, in closed form.  With the target states zero
%    and XNL held constant the six lines reduce to YD_k = YD0 - k*h*XNL and
%    Y_k = Y0 + k*h*YD0 - h^2*XNL*k(k+1)/2 -- EXACT for this discrete scheme,
%    so machine precision or broken.  The k(k+1)/2 (not k(k-1)/2) is precisely
%    the fingerprint of "Y uses the YD the previous line just wrote".
h = p.HP;  k = floor((p.TS - 0.0001)/h) + 1;
[YB,YDB,YTDDB,YTDDDB] = project(p.TS, 11.0, -3.0, 0, 0, h, 7.0, p.W);
Yex  = 11.0 + k*h*(-3.0) - h*h*7.0*k*(k+1)/2;
YDex = -3.0 - k*h*7.0;
d1 = max(abs([YB-Yex, YDB-YDex, YTDDB, YTDDDB]))/max(1, abs(Yex));
names{end+1}  = 'PROJECT == its own sequential-Euler closed form';
oks(end+1)    = d1 <= 1e-12;
detail{end+1} = sprintf('%d Euler steps; max deviation %.3e', k, d1);

% 2  SIGN STRUCTURE.  PHI(-w) must equal PHI(w) except entry (4,5), which
%    flips; in Q, (4,5) flips and (4,4)/(5,5) do not.  This is what makes the
%    filter's propagation blind to the sign of the frequency -- the mechanism
%    behind the book's "magnitude right, sign wrong" result -- and a lost
%    minus or a stray square in either matrix breaks it.
P1 = phiMat(2.5, 40.0, p.TS, false);  P2 = phiMat(-2.5, 40.0, p.TS, false);
Q1 = qMat(2.5, 40.0, p.TS, 100, 0.7); Q2 = qMat(-2.5, 40.0, p.TS, 100, 0.7);
Pd = P1;  Pd(4,5) = -Pd(4,5);
Qd = Q1;  Qd(4,5) = -Qd(4,5);  Qd(5,4) = -Qd(5,4);
d2 = max([max(abs(Pd-P2),[],'all')/max(abs(P1),[],'all'), ...
          max(abs(Qd-Q2),[],'all')/max(abs(Q1),[],'all')]);
names{end+1}  = 'PHI/Q even in w_hat except the (4,5) Jacobian';
oks(end+1)    = d2 <= 1e-14;
detail{end+1} = sprintf('max relative deviation %.3e', d2);

% 3  DELETE EXPERIMENT.  PHI(4,5) = -2*w_hat*ytddhat*Ts is the ONLY path from
%    the frequency state into the others.  Remove it and P(1:4,5) can never
%    become nonzero, so GAIN(5) stays 0 and w_hat is frozen at WHIC forever.
%    Nothing else in the run reports this: the engagement still flies.
if aux
    dev = RUNS.nojac.wh_frozen_dev;
    names{end+1}  = 'DELETE: removing PHI(4,5) freezes w_hat at WHIC';
    oks(end+1)    = dev <= 1e-12;
    detail{end+1} = sprintf(['max |w_hat - WHIC| = %.3e rad/s without the ' ...
        'Jacobian, against %.4g rad/s with it'], dev, A.wh_frozen_dev);
end

% 4  UPDATE CADENCE.  Repair 2 turned "if S .=(TS-.00001)" into ">=".  The
%    wrong direction records EVERY integration step: 10000 instead of 1000,
%    a filter running at 1 kHz instead of 100 Hz, and it would still produce a
%    plausible curve.  This is the row that would catch it.
want = round(p.TF/p.TS);
names{end+1}  = 'exactly round(TF/TS) Kalman updates (repair 2)';
oks(end+1)    = A.n == want;
detail{end+1} = sprintf('%d updates, expected %d', A.n, want);

% 5  With PHIS2 = 0 the frequency state gets no process noise, so P(5,5) is
%    non-increasing (M(5,5) = P(5,5), and the update subtracts
%    M(5,1)*M(1,5)/HMHTR >= 0).  The threshold is ABSOLUTE, not relative,
%    because a real fault here -- Q(5,5) wired to PHIS1 instead of PHIS2 --
%    adds PHIS1*Ts ~ 37 per step, eight orders above the 1e-6 allowed for
%    rounding in (I-KH)M, which is not symmetric-by-construction.  Also
%    confirms repair 4 is inert at this XNCLIM: zero clamps.
names{end+1}  = 'PHIS2 = 0 => P(5,5) non-increasing; XNCLIM never fires';
oks(end+1)    = (p.PHIS2 > 0 || A.p55rise <= 1e-6) && A.nclip == 0;
detail{end+1} = sprintf(['largest single-step rise in P(5,5) %.3e (limit 1e-6); ' ...
    'sqrt(P55) %.4g -> %.4g rad/s; %d command clamps'], A.p55rise, ...
    firstOr(A.sp55, NaN), lastOr(A.sp55, NaN), A.nclip);

K = struct('names',{names}, 'oks',logical(oks), 'detail',{detail});
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    b = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{b}, detail{b});
end
end


% =============================================================================
function [B, E] = benchmarks(A, RUNS, opts, p)
%BENCHMARKS  Rows 1-6 and 9 are about the LISTING AS PRINTED; if the caller has
%   moved off it they would be about the caller instead, so they collapse into
%   ONE skip row rather than being quietly re-pointed at a different
%   experiment.  Rows 7 (algebra) and 8 (paired, same seed) survive any
%   settings -- row 8 is what makes this file usable for pricing a parameter
%   rather than only for reproducing page 632.
B = emptyBench();
E = struct('ran', false);       % the WHIC ensemble; only the stock branch pays
W = opts.W;
stock = opts.SIGRIN == 1e-4 && opts.WHIC == -1 && opts.PHIS2 == 0 && ...
        opts.APN == 4 && opts.W == 2 && opts.XNT == 96.6 && opts.TAU == 0.5 && ...
        opts.TF == 10 && opts.TS == 0.01 && ~opts.QPERFECT;

if ~stock
    B = addSkip(B, 'the seven printed-listing rows (WHIC; |w| at 1 mr; sign; noise ordering; WHIC 3/4; Fs2 = 1)', ...
        'see the cited lines', sprintf(['the caller moved off the printed ' ...
        'listing (SIGRIN %g, WHIC %g, PHIS2 %g, APN %d, W %g, TF %g s), so ' ...
        'these rows would be about the caller and not about the book'], ...
        opts.SIGRIN, opts.WHIC, opts.PHIS2, opts.APN, opts.W, opts.TF), ...
        'zarchan.txt:9101, :9102, :9112, :9136, :9147, :9952', '-');
elseif ~opts.aux
    B = addSkip(B, 'the seven printed-listing rows', 'see the cited lines', ...
        '''aux'' is false: the 1 mr / WHIC / Fs2 / known-frequency runs did not happen', ...
        'zarchan.txt:9101-9136, :9952', '-');
else
    M1 = RUNS.mr1;  F = RUNS.phis2;  C3 = RUNS.whic3;  C4 = RUNS.whic4;

    % 1  WHIC = -1 rad/s.  Prose and listing agree ONLY after the minus-sign
    %    decode, which is the whole reason this row exists.
    B = addVal(B, 'WHIC, the deliberately wrong initial w_hat', -1.0, opts.WHIC, ...
        'exact', 1e-12, ['prose zarchan.txt:9101 ("WHIC = 21" = -1, minus-sign ' ...
        'damage) confirmed by the listing line zarchan.txt:9147 ("WHIC=-1.")'], ...
        'rad/s', 'the book initialises the EKF wrong on purpose, to test robustness');

    % 2  |w_hat| in the steady state at the book's NOMINAL 1 mr.  The 25% band
    %    is set by REALISATION SPREAD, not printing precision: our noise draw
    %    is not the book's and a tight band would be scoring the draw.  It
    %    still red-lights both failures that matter -- "never learned"
    %    (|w_hat| stays 1, -60%) and "diverged" (|w_hat| >> 3).
    B = addVal(B, '|w_hat| steady state at 1 mr', 2.5, M1.wss, '25%', 0.25, ...
        'prose zarchan.txt:9102 (Fig. 25.18): "2.5 r/s rather than 2 r/s"', 'rad/s', ...
        sprintf(['median |w_hat| over [%.1f, %.1f] s, truth W = %.3g.  The ' ...
        'printed listing (:9147) has SIGRIN = .0001 = 0.1 mr but the prose ' ...
        'calls the nominal case 1 mr, so this row is the AUX run'], ...
        0.5*opts.TF, 0.9*opts.TF, W));

    % 3  The SIGN is wrong, at both noise levels.  Ordering, not a value.
    B = addShape(B, 'sign of w_hat is wrong (negative) at 1 mr AND 0.1 mr', ...
        'magnitude nearly correct, but the sign is wrong; still unable to estimate the sign', ...
        sprintf('w_hat_ss %+.3f (1 mr), %+.3f (0.1 mr) rad/s vs W = +%.3g', ...
        M1.wss_signed, A.wss_signed, W), ...
        M1.wss_signed < 0 && A.wss_signed < 0, 'prose zarchan.txt:9102 and :9112', ...
        'both start at WHIC = -1; the book keeps the wrong sign and flies it');

    % 4  Less noise, better magnitude.  PAIRED: same seed, so the two runs
    %    differ by the noise level and by nothing else.
    B = addShape(B, '0.1 mr improves the |w_hat| estimate over 1 mr', ...
        'reducing the measurement noise improves the estimate of the magnitude', ...
        sprintf('| |w_hat|-W | = %.3f at 0.1 mr vs %.3f at 1 mr', A.werr, M1.werr), ...
        A.werr < M1.werr, 'prose zarchan.txt:9112 (Fig. 25.21)', ...
        'paired: identical seed and identical noise-sequence index');

    % 5/6  WHIC = 3 recovers, WHIC = 4 does not: the book's robustness limit,
    %      and the reason Ch.27 exists at all.  *** RE-AIMED 2026-08-24: these
    %      two rows used to score ONE draw each and came out INVERTED.  See the
    %      header block.  Which way a single draw falls is a property of the
    %      noise realisation; the book's sentence is about the filter.  The
    %      ensemble is the smallest thing that tests the sentence, and the
    %      single-draw number stays printed in ours_str so the inversion is
    %      still visible on the board rather than quietly deleted. ***
    E = whicEnsemble(p, 24);
    B = addShape(B, 'WHIC = 3 (error +1 rad/s): w_hat recovers, at ~3 s', ...
        'the extended Kalman filter is able to estimate the target weave frequency after 3 s', ...
        sprintf(['median recovery %.2f s over the %d of %d draws that recover ' ...
                 '(WHIC = 4 recovers on %d); single default draw %.4g s'], ...
        E.tlock3, E.nlock3, E.n, E.nlock4, C3.t_lock), ...
        isfinite(E.tlock3) && E.tlock3 >= 2.25 && E.tlock3 <= 3.75 && ...
        E.nlock3 > E.nlock4, 'prose zarchan.txt:9952 (Fig. 27.2)', ...
        ['band 3 s +/- 25%, the tolerance the book''s own "nearly correct" ' ...
         '(2.5 vs 2 at :9102) implies; recovery = |w_hat-W| <= 25% of W ' ...
         'thereafter.  Also requires WHIC = 3 to recover on more draws than ' ...
         'WHIC = 4, so a lucky single recovery cannot carry the row']);
    e4 = abs(lastOr(C4.WH, NaN) - W);       % a diverged run can end early
    e3 = abs(lastOr(C3.WH, NaN) - W);
    B = addShape(B, 'WHIC = 4 (error +2 rad/s): the estimate diverges', ...
        'the filter''s estimate diverges from the truth and is in considerable error', ...
        sprintf(['median |w_hat_ss-W| %.2f rad/s (%.0f%% of W) vs %.2f at ' ...
                 'WHIC = 3; closer at WHIC = 3 on %d of %d paired draws; ' ...
                 'single default draw %.3f rad/s (final) vs %.3f'], ...
        E.err4, 100*E.err4/W, E.err3, E.paired, E.n, e4, e3), ...
        E.err4 > 0.25*W && E.paired >= ceil(0.75*E.n), ...
        'prose zarchan.txt:9952 (Fig. 27.2)', ...
        ['"considerable error" scored as OUTSIDE the book''s own 25%-of-W ' ...
         '"nearly correct" band, plus a PAIRED per-seed comparison (same ' ...
         'seed, same noise sequence, only WHIC differs) so the row cannot be ' ...
         'passed by a noisy ensemble that is bad everywhere']);

    % 9  Fs2 = 1.  "Can actually diverge" is scored as its weaker, checkable
    %    half: worse than Fs2 = 0 on the same draw.
    B = addShape(B, 'Fs2 = 1 degrades the frequency estimate', ...
        'if we add too much process noise (Fs2 = 1) ... the estimate can actually diverge', ...
        sprintf('| |w_hat|-W | = %.3f at Fs2 = 1 vs %.3f at Fs2 = 0%s', F.werr, ...
        A.werr, ternary(F.blew, ' (run went non-finite)', '')), ...
        F.blew || F.werr > A.werr, 'prose zarchan.txt:9136 (Fig. 25.25)', ...
        'the checkable half of "can diverge"; paired, same seed');
end

% 7  THE BOOK'S OWN SYMMETRY CLAIM, and it is exact algebra rather than a
%    realisation: in compensated weave guidance C3 ~ (1-cos wT)/(w^2 T^2) and
%    C5 ~ (wT - sin wT)/(w^3 T^2) are both EVEN in w.  This is why the book is
%    content to fly a wrong-signed frequency estimate, and it is the row that
%    would catch a lost power of w in either kernel.
dmax = 0;
for tg = [0.25 1.0 3.0 7.5]
    for wq = [0.7 2.0 2.5 4.0]
        ap = guidanceLaw(4, opts.XNP, tg, opts.TAU, 12, -30, 55, -70, 8,  wq);
        am = guidanceLaw(4, opts.XNP, tg, opts.TAU, 12, -30, 55, -70, 8, -wq);
        dmax = max(dmax, abs(ap-am)/max(abs(ap), realmin));
    end
end
B = addBound(B, 'compensated weave command is EVEN in w_hat', ...
    'the sign of the target weave frequency does not influence the guidance command', ...
    sprintf('max relative |XNC(w) - XNC(-w)| = %.3e over 16 (tgo, w) points', dmax), ...
    dmax <= 1e-12, 'prose zarchan.txt:9189', '-', 'exact in IEEE arithmetic');

% 8  THE COMPARISON THIS PORT EXISTS FOR: frequency ESTIMATED costs accuracy
%    against frequency KNOWN.  *** READ THE CAVEAT. ***  The book compares
%    against Listing 25.2, which carries the EXACT weave fundamental matrix.
%    The twin here is THE SAME FILTER TOLD THE TRUTH (w_hat pinned at W,
%    P(5,5) = 0, GAIN(5) never applied) -- a four-state filter carrying 25.3's
%    TAYLOR PHI.  That isolates the cost of ESTIMATING from the cost of the
%    Taylor approximation, a cleaner experiment than the book's, but it is NOT
%    the book's and must not be quoted as reproducing Figs 25.11/25.12.
if opts.aux
    K = RUNS.known;
    B = addShape(B, 'estimating w costs accuracy vs being told it (paired)', ...
        'these estimates are not quite as good as those obtained ... when the target weave frequency was known precisely', ...
        sprintf('post-transient rms accel-estimate error %.4f g estimated vs %.4f g known (ratio %.3f)', ...
        A.rms_acc_g, K.rms_acc_g, A.rms_acc_g/max(K.rms_acc_g, realmin)), ...
        A.rms_acc_g >= K.rms_acc_g, ...
        'prose zarchan.txt:9124 (Figs 25.22/25.23 vs 25.11/25.12)', ...
        'same seed and noise sequence; twin = this filter with w_hat pinned at W');
end

% NOT SCORED.
B = addSkip(B, 'Fs2 = .1 "slightly improves" the magnitude', 'slightly improves', ...
    ['a SLIGHT improvement at ONE noise realisation is below the resolution of ' ...
     'this experiment; scoring it would be scoring the draw, and widening a ' ...
     'band to absorb it is exactly what must not happen'], ...
    'prose zarchan.txt:9136 (Fig. 25.24)', 'rad/s');
B = addSkip(B, 'rms miss distance vs weave frequency', '--', ...
    ['Figs 25.29-25.32 publish rms miss as FIGURES; no number for this ' ...
     'configuration survived extraction.  (Ch.27''s 5 ft / 20 ft at :10015 is ' ...
     'Listing 25.2 over a frequency sweep, not this run.)  This run''s terminal ' ...
     '|Y| is in R.miss_ft and is NOT a published value'], ...
    'zarchan.txt:9195-9236, :10015', 'ft');
end

function E = whicEnsemble(p, nSeeds)
%WHICENSEMBLE  Fig. 27.2 is a pair of traces from ONE of Zarchan's randn draws,
%   and this filter's convergence at a given WHIC is a property of the draw:
%   over 100 seeds WHIC = 3 recovers on 58 and WHIC = 4 on 4, so a one-draw row
%   has roughly a 1-in-50 chance of printing the pair INVERTED -- which is
%   precisely what the default seed did, and what rows 5/6 used to report.
%   This runs the pair over a FIXED panel derived from the caller's own single
%   declared seed, so the panel cannot be quietly hand-picked and a caller who
%   changes 'seed' genuinely re-tests instead of re-rolling.  The contrast is
%   not knife-edge: over six independent base seeds the median WHIC = 3
%   recovery time stayed in 2.87-3.09 s, the WHIC = 4 steady-state error in
%   253-330% of W, and the paired count in 22-24 of 24.
%
%   'strict' is off: a diverged WHIC = 4 run CAN drive w_hat through zero and
%   make the APN = 4 kernel's 1/w^3 non-finite.  That is the divergence, not an
%   error, and .nblew reports it.
E = struct('ran', true, 'n', nSeeds);
E.seeds = p.seed + 1000*(1:nSeeds);
W = p.W;
lock = nan(2, nSeeds);  wss = nan(2, nSeeds);  blew = false(2, nSeeds);
for j = 1:2
    for i = 1:nSeeds
        q = p;  q.strict = false;  q.WHIC = 2 + j;   % j = 1 -> 3, j = 2 -> 4
        q.seed = E.seeds(i);
        O = runEKF(q);
        lock(j,i) = O.t_lock;  wss(j,i) = O.wss_signed;  blew(j,i) = O.blew;
    end
end
E.lock3 = lock(1,:);  E.lock4 = lock(2,:);
E.wss3  = wss(1,:);   E.wss4  = wss(2,:);
E.nlock3 = sum(isfinite(E.lock3));  E.nlock4 = sum(isfinite(E.lock4));
% median over the draws that recover AT ALL.  Conditioning is deliberate: the
% book states WHEN it recovers, not how often, and the how-often is scored
% separately by requiring nlock3 > nlock4 on the same row.  NaN if none do,
% which addShape's condition then fails rather than passes.
E.tlock3 = median(E.lock3(isfinite(E.lock3)));
E.tlock4 = median(E.lock4(isfinite(E.lock4)));
E.err3 = median(abs(E.wss3 - W), 'omitnan');
E.err4 = median(abs(E.wss4 - W), 'omitnan');
E.wmed3 = median(E.wss3, 'omitnan');  E.wmed4 = median(E.wss4, 'omitnan');
E.paired = sum(abs(E.wss3 - W) < abs(E.wss4 - W));
E.nblew  = sum(blew(:));
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end

function v = firstOr(x, dflt)
if isempty(x), v = dflt; else, v = x(1); end
end

function v = lastOr(x, dflt)
%LASTOR  Last element, or a default if the run ended before recording one.
%   A diverged aux run can stop early; indexing (end) on it would throw far
%   from the divergence that caused it.
if isempty(x), v = dflt; else, v = x(end); end
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want.  This is the
%   schema mgs.zarchan.benchSuite's shared benchRows() consumes.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.6g', book);
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

function B = addBound(B, what, bookStr, oursStr, ok, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'bound';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'bound';  B(i).tol = NaN;
B(i).ok   = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = units;  B(i).detail = detail;
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
p = R.p;  L = {};
L{end+1} = 'mgs.zarchan.weaveExtendedKalman -- Zarchan Listing 25.3 (zarchan.txt:9147-9163)';
L{end+1} = sprintf('  units: %s', R.units);
L{end+1} = sprintf(['  W %.3g rad/s  WHIC %.3g  nT %.4g ft/s^2 (%.2f g)  TAU %.3g s  ' ...
    'VC %.4g ft/s  TF %.3g s  TS %.3g s  H %.4g s'], p.W, p.WHIC, p.XNT, ...
    p.XNT/32.2, p.TAU, p.VC, p.TF, p.TS, p.H);
L{end+1} = sprintf('  APN %d  N'' %.3g  SIGRIN %.4g mr  PHIS1 %.6g  PHIS2 %.3g  seed %d', ...
    p.APN, p.XNP, 1000*p.SIGRIN, p.PHIS1, p.PHIS2, p.seed);
L{end+1} = sprintf('  %d updates, %d command clamps; w_hat_ss %+.3f rad/s; rms accel err %.4f g; terminal |Y| %.3f ft', ...
    p.n_updates, p.n_clamps, R.wss_signed, R.rms_acc_g, R.miss_ft);
if isfield(R.runs,'whic3') && isfield(R.runs,'whic4')
    % The single draw, kept visible on purpose: at the default seed it prints
    % the book's WHIC 3/4 outcomes INVERTED.  Rows 5/6 score the ensemble.
    L{end+1} = sprintf(['  single draw (seed %d): WHIC 3 -> w_ss %+.3f, recovery %.4g s;  ' ...
        'WHIC 4 -> w_ss %+.3f, recovery %.4g s   <- one realisation, NOT scored'], ...
        p.seed, R.runs.whic3.wss_signed, R.runs.whic3.t_lock, ...
        R.runs.whic4.wss_signed, R.runs.whic4.t_lock);
end
if isfield(R,'whic_ensemble') && isstruct(R.whic_ensemble) && R.whic_ensemble.ran
    E = R.whic_ensemble;
    L{end+1} = sprintf(['  WHIC ensemble (%d seeds): recovery %d/%d at WHIC 3 (median %.2f s) ' ...
        'vs %d/%d at WHIC 4; median |w_ss-W| %.2f vs %.2f rad/s; paired %d/%d; %d run(s) went non-finite'], ...
        E.n, E.nlock3, E.n, E.tlock3, E.nlock4, E.n, E.err3, E.err4, E.paired, E.n, E.nblew);
end
L{end+1} = '  --- structural ---';
for i = 1:numel(R.selfcheck.names)
    L{end+1} = sprintf('   [%s] %s: %s', ternary(R.selfcheck.oks(i),'ok','BAD'), ...
        R.selfcheck.names{i}, R.selfcheck.detail{i}); %#ok<AGROW>
end
L{end+1} = '  --- book ---';
for i = 1:numel(R.bench)
    b = R.bench(i);
    if b.skipped
        L{end+1} = sprintf('   [%-4s] %s -- %s', upper(b.kind), b.what, b.why); %#ok<AGROW>
    else
        L{end+1} = sprintf('   [%s] %s: book %s, ours %s (%s)', ...
            ternary(b.ok,'ok ','BAD'), b.what, b.book_str, b.ours_str, b.band); %#ok<AGROW>
    end
end
if ~any(ev), L{end+1} = '  NOTHING WAS SCORED.'; end
s = strjoin(L, newline);
end
