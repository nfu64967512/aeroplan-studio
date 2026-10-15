function R = dualRateAdjoint(opts)
%MGS.ZARCHAN.DUALRATEADJOINT  Zarchan Listing 29.2 -- adjoint of a homing loop
%   with TWO SAMPLERS RUNNING AT DIFFERENT RATES, plus its Listing 29.1 twin.
%
%   R = mgs.zarchan.dualRateAdjoint()
%   R = mgs.zarchan.dualRateAdjoint('BETA', 0.3)           faster filter
%   R = mgs.zarchan.dualRateAdjoint('forward', false)      adjoint only, fast
%   R = mgs.zarchan.dualRateAdjoint('assert', false)       warn, still return R
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, s, g = 32.2 ft/s^2.  NOT SI.
%   The rest of mgs is metres; nothing is converted here so that every printed
%   number can be laid straight against a published one.  Convert AT THE CALL
%   SITE and say so there.
% ===========================================================================
%
%   WHAT IT IS.  A LINEAR, planar, constant-closing-velocity homing loop
%   (Fig. 29.2) whose seeker data arrives at 50 Hz but whose two-state fading
%   memory filter runs at 10 Hz; the surplus measurements are averaged by a
%   moving-average preprocessor.  Listing 29.2 is its ADJOINT: ONE integration
%   gives the miss at EVERY flight time, where the forward model (Listing 29.1,
%   ported here too) needs one run each.  Sole error source: a constant 3-g
%   target maneuver from t = 0.  It is the book's only adjoint of a MULTIRATE
%   loop, which is the shape our own stack has.  *** IT DOES NOT TRANSFER TO
%   OUR SPREAD PROBLEM: *** the adjoint identity is a LINEARITY identity and
%   our dominant error is a gamma-tracking lag that SATURATES (gain 0.063 vs a
%   model 1.385, at its stop on 26 of 32 sorties).  Read it as machinery.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  Source: matlab/zarchan/_listings/listing_29_02.m (book
%   code at zarchan.txt:10664-10680).  Its header reports
%   NO unresolved variable-to-variable comparisons and that is right -- every
%   comparison in 29.2 is against a literal.  Three forced repairs:
%
%     1  "while TP , = (TF-1e-5)" -> "while TP <= ...".  Shape-forced: TP
%        starts at 1e-5 and only increases, so ">=" never enters and "=="
%        never fires.  (The extractor fixed the identical "while STEP , =1"
%        but not this one, because of the "(" after the "=".)
%     2  "if S . = (TS-.0001)" -> "if S >= ...", same for S2.  "." is this
%        book's OCR of ">" (also at :1300 "if PZ . 0", :12847 "if XK1.K(1,1)").
%        DIRECTION is forced: S is RESET to 0 inside the block, so "<=" fires
%        every step and the two sampling rates in the title would not exist.
%     3  Transcription lines 24-31 are the next paragraph plus Fig. 29.4 axis
%        text leaking in ("A lti tu de ( km )" -- not even this figure's axis).
%        Dropped, not "fixed" into code.
%
%   Reproduced deliberately: TP starts at +1e-5 and TGO = TP (the adjoint gain
%   is 1/(Vc*tgo); an exact zero divides by zero on the first derivative), and
%   "count=1" before the first store, which leaves ArrayTP(1)=ArrayXMNT(1)=0 so
%   the published curve begins at the origin.  Dropped: plotting and the save.
%   From Listing 29.1: XLAMD = (RTM*YD+Y*VC)/RTM^2 is computed and NEVER USED
%   (the command comes from the FILTER's XLAMDHNEW) -- dead code in the book,
%   see zarchan.txt:10641, so it is not carried here.
%
% ---------------------------------------------------------------------------
%   THE BOOK CONTRADICTS ITSELF ABOUT THE PREPROCESSOR, AND THE CODE WINS
%
%   Prose (zarchan.txt:10600): the extra data is handled by "adding up five
%   measurements and then dividing by five" -- 5 taps, dc gain 1.
%   Listing 29.1 (:10653) sums SIX and multiplies by 0.2, dc gain 1.2:
%       PZ = .2*(Y5OLD + Y5NEW+Y4NEW+Y3NEW+Y2NEW+XLAM)
%   The extra term is NOT a typo, and the adjoint proves it.  Unroll 29.2 with
%   u(k) = .2*(Y6NEW-Y6OLD):  Y10 = u(k-1)+u(k), Y9 = u(k-2)+..., Y8 = ...,
%   Y5 = u(k-4)+...+u(k), and Y4 += Y5OLD + u(k) = u(k)+...+u(k-5) -- SIX.
%   Both listings implement the same six-tap gain-0.2 filter; only the prose
%   says five.  This port transcribes the CODE, reports the contradiction as an
%   INFO row citing both sides, and MEASURES it: SELFCHECK 5 drops the sixth
%   tap and requires the peak miss to move.  It is not inert -- the cross-check
%   below puts the five-tap peak at 126.86 ft against 101.59 ft, and the
%   five-tap forward at tF = 10 s at +0.69 ft against -15.30 ft.  The choice
%   between the prose and the code changes the SIGN of the answer.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Every book value was grepped from the DECODED book; the
%   line number in .source is the grep/sed/Read line and resolves in BOTH book
%   files.  Nothing was read off a figure.
%
%   ⚠️⚠️ CORRECTED.  This used to say the raw and decoded files "do NOT share
%   line numbering (15120 vs 15307), so one citation alone would be a dead
%   reference", and every row therefore carried TWO numbers.  Measured, that is
%   wrong in a way that made the second number actively harmful:
%
%     * The two book files ARE aligned -- both hold 15120 LF records and 15307
%       CR records, and the same sentence sits at the same index in both.  ONE
%       citation resolves in either file.
%     * The +187 is ONE FILE READ TWO WAYS: grep / sed / Read split on LF and
%       see 15120 lines; MATLAB readlines() splits on a bare CR as well and
%       sees 15308.  The gap accumulates through the file -- 0 at the front,
%       +36 by Ch.7, +187 by Ch.29.
%     * So the "decoded :NNNN" halves were readlines INDICES wearing the wrong
%       label.  All 8 pairs differed by exactly 187.  Following one of them
%       into zarchan_decoded.txt lands on unrelated text.  They are removed;
%       the surviving number is the grep one, and it was verified to land on
%       the cited material.
%
%     1-2  G = 1-b^2 and H = (1-b)^2 at b = 0.8 (zarchan.txt:10609-10610).
%          Both minus signs vanished in extraction ("G = 1 b2") and are
%          recovered from the listing's own GFILTER/HFILTER -- not guessed.
%     3    Ts/Ts2 = 5, why the preprocessor exists (:10600; Table 29.1).
%     4    THE ROW THIS FILE EXISTS FOR: 29.2 vs 29.1, zarchan.txt:10681 --
%          "exact agreement between the adjoint and forward simulations".
%          "Exact" is an adjective about a plot, so the band is this port's own
%          floor: max(0.5% of peak, 4*H), normalised by the PEAK because the
%          miss curve crosses zero and a pointwise relative error there
%          manufactures a failure out of nothing.  Both schemes are Heun at the
%          same H with aligned samplers, so only the O(H) truncation DIFFERENCE
%          survives.  A residual near the EDGE is not a pass to accept: it
%          means the loops stopped being transposes, and that is a ~16 ft
%          effect, not a ~0.5 ft one.
%     5    SHAPE (:10693): at b = 0.3 "the guidance system performance
%          improves".  An ordering on peak |miss| -- stronger than a number
%          eyeballed off Fig. 29.5.
%
%   NOT SCORED: the miss magnitudes of Figs. 29.4/29.5 (no number is printed),
%   and the Fig. 29.6 claim that adjoint time doubles as the tgo of a maneuver
%   onset -- that needs 29.1 rewritten to sweep the maneuver START time, so it
%   is left as an explicit SKIP rather than asserted.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** THIS FILE HAS NEVER BEEN EXECUTED -- MATLAB is
%   the caller's. ***  There is no closed form for this loop, so these do not
%   come from one: they come from an INDEPENDENT re-implementation of the same
%   two loops in another language, written to test the repairs above before
%   handing the port over.  Evidence about the TRANSCRIPTION, not about this
%   file running:
%     peak |miss|         101.59 ft at tF = 2.32 s  (b = 0.8)
%     miss at tF = 1..10  42.27  97.64  85.76  32.24  -13.40  -33.71
%                         -35.19  -28.77  -21.28  -15.30 ft
%     adjoint vs forward  worst 2.0e-4 ft = 2e-6 of peak -- the book's "exact
%                         agreement", four orders inside the band
%     peak at b = 0.3     16.55 ft, 6x better than b = 0.8
%     five-tap variant    126.86 ft, a 24.9% move (SELFCHECK 5 wants > 2%)
%     selfchecks 1,2,3    exact identities: EXACTLY zero, not "small"
%     cadence             100 low-rate fires, 500 high, 501 samples, t(end) =
%                         10.00001 s
%   That curve spans -35.2..+101.6 ft and Listing 29.1's plot box is
%   axis([00,10,-40,100]): the author sized the box to the answer.  Still not a
%   measurement, so still not scored.
%
%   OUTPUTS
%     R.t, R.miss_ft   the adjoint sweep: flight time [s] and miss [ft]
%     R.x1             miss/nT, the sensitivity [s^2]
%     R.fwd            Listing 29.1: .tf .miss_ft .adj_ft .dev_ft   ([] if off)
%     R.beta_alt       the b = 0.3 comparison run
%     R.bench .selfcheck .pass .verdict .p .units .summary
%
%   See also MGS.ZARCHAN.SHAPINGFILTERADJOINT, MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 29.2 line 1 and Table 29.1 (zarchan.txt:10631-10639), verbatim.
    opts.XNT  (1,1) double {mustBeFinite, mustBePositive} = 96.6    % ft/s^2, 3 g
    opts.XNP  (1,1) double {mustBeFinite, mustBePositive} = 3.0     % N-prime
    opts.TAP  (1,1) double {mustBeFinite, mustBePositive} = 0.2     % s, FCS lag
    opts.VC   (1,1) double {mustBeFinite, mustBePositive} = 4000.0  % ft/s
    opts.BETA (1,1) double {mustBeFinite, mustBePositive} = 0.8     % filter beta
    opts.TS   (1,1) double {mustBeFinite, mustBePositive} = 0.1     % s, filter
    opts.TS2  (1,1) double {mustBeFinite, mustBePositive} = 0.02    % s, system
    opts.TF   (1,1) double {mustBeFinite, mustBePositive} = 10.0    % s
    opts.H    (1,1) double {mustBeFinite, mustBePositive} = 0.001   % s, RK2
    % The preprocessor.  SIX taps at gain 0.2 -- that is what BOTH listings do;
    % only the prose says five.  Read the header before "correcting" this.
    opts.ntap   (1,1) double {mustBeFinite} = 6
    opts.ppgain (1,1) double {mustBeFinite} = 0.2
    % Flight times for the forward model.  [] means 1:1:floor(TF).  The book
    % runs 0.1:0.1:10 (100 runs); ten adjudicates agreement at a tenth of the
    % cost.  Each must be a whole multiple of TS -- see mustDivide.
    opts.fwd_tf double {mustBeReal, mustBeFinite} = []
    % THE ONLY SWITCHES.  Not nargin: with an arguments block declaring only
    % name-value options MATLAB reports nargin = 0 however many were passed,
    % so an "if nargin == 0" here would be a mode that never fires.
    opts.forward   (1,1) logical = true
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

XNT=opts.XNT; XNP=opts.XNP; TAP=opts.TAP; VC=opts.VC; BETA=opts.BETA;
TS=opts.TS;   TS2=opts.TS2; TF=opts.TF;   H=opts.H;
ntap = round(opts.ntap);  ppgain = opts.ppgain;

if ntap < 2
    error('mgs:zarchan:dualRateAdjoint:ntap', ...
        ['ntap = %d.  The preprocessor needs at least two taps to be a ' ...
         'preprocessor: the adjoint chain would have no delay states left.'], ntap);
end
mustDivide(TS2, H,   'TS2', 'H');
mustDivide(TS,  TS2, 'TS',  'TS2');
mustDivide(TF,  TS,  'TF',  'TS');

fwd_tf = opts.fwd_tf(:).';
if isempty(fwd_tf), fwd_tf = 1:1:floor(TF); end
fwd_tf = fwd_tf(fwd_tf > 0 & fwd_tf <= TF);
for q = fwd_tf
    mustDivide(q, TS, 'each fwd_tf', 'TS');
end

% =========================================================================
%   LISTING 29.2.  One run; every flight time.
% =========================================================================
[t, miss_ft, nLow, nHigh, nEvalA] = adjointRun(XNT,XNP,TAP,VC,BETA,TS,TS2, ...
                                               TF,H,ntap,ppgain,1.0);
if numel(t) < 3
    error('mgs:zarchan:dualRateAdjoint:empty', ...
        ['the sweep recorded %d sample(s) at TF = %g s, Ts2 = %g s.  An ' ...
         'empty or one-point curve returned quietly looks like a result.'], ...
        numel(t), TF, TS2);
end
[peak_ft, ipk] = max(abs(miss_ft));

% =========================================================================
%   LISTING 29.1.  One run PER flight time -- the second opinion, and the
%   only thing that makes the transposition above checkable at all.
% =========================================================================
FWD = [];  nEvalF = 0;
if opts.forward && ~isempty(fwd_tf)
    [fm, nEvalF] = forwardMiss(XNT,XNP,TAP,VC,BETA,TS,TS2,H,fwd_tf,ntap,ppgain);
    FWD = struct('tf',fwd_tf, 'miss_ft',fm, 'adj_ft',zeros(1,numel(fwd_tf)), ...
                 'dev_ft',zeros(1,numel(fwd_tf)));
    for k = 1:numel(fwd_tf)
        % Nearest recorded sample, NOT interpolation: the two grids agree to
        % 1e-5 s here, and interpolating would blur a real disagreement into a
        % smaller one.
        [~, j] = min(abs(t - fwd_tf(k)));
        FWD.adj_ft(k) = miss_ft(j);
    end
    FWD.dev_ft = FWD.miss_ft - FWD.adj_ft;
end

% The b = 0.3 comparison the book makes in Fig. 29.5.
ALT = [];
if opts.benchmark && abs(BETA - 0.3) > 1e-12
    [ta, ma] = adjointRun(XNT,XNP,TAP,VC,0.3,TS,TS2,TF,H,ntap,ppgain,1.0);
    ALT = struct('BETA',0.3, 't',ta, 'miss_ft',ma, 'peak_ft',max(abs(ma)));
end

R = struct();
R.t = t;  R.miss_ft = miss_ft;  R.x1 = miss_ft/XNT;
R.peak_ft = peak_ft;  R.peak_t = t(ipk);
R.fwd = FWD;  R.beta_alt = ALT;
R.units = 'ft, ft/s, ft/s^2, s; g = 32.2 ft/s^2 (the book''s units, unconverted)';
R.p = struct('XNT',XNT, 'XNP',XNP, 'TAP',TAP, 'VC',VC, 'BETA',BETA, 'TS',TS, ...
             'TS2',TS2, 'TF',TF, 'H',H, 'ntap',ntap, 'ppgain',ppgain, ...
             'fwd_tf',fwd_tf, 'n_low_fires',nLow, 'n_high_fires',nHigh, ...
             'n_rhs_adjoint',nEvalA, 'n_rhs_forward',nEvalF, 'units',R.units);

R.selfcheck = structuralChecks(XNT,XNP,TAP,VC,BETA,TS,TS2,TF,H,ntap,ppgain, ...
                               t,miss_ft,nLow,nHigh,peak_ft);

if opts.benchmark
    R.bench = benchmarks(R, BETA, TS, TS2, peak_ft, ALT, opts.forward);
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

% ---- adjudication.  The verdict is FORMED either way; only the reaction to
%      it depends on 'assert'.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port ' ...
        'disagrees with ITSELF, which is a wiring fault, not a tolerance ' ...
        'question.  First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf(['%d of %d evaluated book values are outside tolerance ' ...
        '(first: %s, want %s, ours %s).'], numel(bad), sum(ev), ...
        R.bench(bad(1)).what, R.bench(bad(1)).book_str, R.bench(bad(1)).ours_str);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:dualRateAdjoint:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN COMPLETE.  benchSuite runs every port with assert
        % off precisely so that one red row cannot hide the board, and a port
        % that refuses to return is a port nobody can diagnose.
        warning('mgs:zarchan:dualRateAdjoint:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [t, miss, nLow, nHigh, nEval] = adjointRun(XNT,XNP,TAP,VC,BETA,TS, ...
                                                    TS2,TF,H,ntap,ppgain,x3ic)
%ADJOINTRUN  Listing 29.2, zarchan.txt:10664-10680.
%
%   Every constant is an ARGUMENT.  MATLAB local functions cannot see the main
%   function's workspace; a previous wave in this package read a caller's
%   constant from a local and threw on the first line of the first run.
GFILTER = 1.0 - BETA^2;          % zarchan.txt:10609, minus recovered from code
HFILTER = (1.0 - BETA)^2;        % zarchan.txt:10610

S = 0;  S2 = 0;  TP = 0 + 0.00001;   % NOT 0: the adjoint gain is 1/(Vc*tgo)
X1=0; X2=0; X3=x3ic; X4=0; X5=0;     % <<< THE IMPULSE is on X3, nowhere else
X1D=0; X2D=0; X3D=0; X4D=0; X5D=0;
Y1OLD=0; Y2OLD=0; Y3OLD=0; Y7=0; Y6OLD=0; Y4=0;
Z = zeros(1, ntap-1);                % the listing's Y11,Y10,Y9,Y8,Y5

nmax = ceil(TF/TS2) + 4;
t = zeros(nmax,1);  miss = zeros(nmax,1);
n = 1;                               % count=1 leaves sample 1 at (0,0)
nLow = 0;  nHigh = 0;  nEval = 0;

while TP <= (TF - 1e-5)
    S = S + H;  S2 = S2 + H;         % the listing increments at the TOP
    X1OLD=X1; X2OLD=X2; X3OLD=X3; X4OLD=X4; X5OLD=X5;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  Pass 1 evaluates at the current
    % state; pass 2 takes a full Euler step, advances TP by H and re-evaluates
    % there.  TP advances INSIDE this loop, exactly once -- move that line out
    % and the scheme quietly degrades to first order while still running.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            X1=X1+H*X1D; X2=X2+H*X2D; X3=X3+H*X3D; X4=X4+H*X4D; X5=X5+H*X5D;
            TP = TP + H;
        end
        TGO = TP;                    % adjoint time IS time-to-go
        X1D = X2;
        X2D = X3 + Y4/(VC*TGO);
        X3D = Y4/(VC*TGO*TGO);
        X4D = -X2 - X4/TAP;
        X5D = X4/TAP;
        FLAG = 1;
        nEval = nEval + 1;
    end
    % Heun corrector, in the listing's own algebra.
    X1 = (X1OLD+X1)/2 + .5*H*X1D;
    X2 = (X2OLD+X2)/2 + .5*H*X2D;
    X3 = (X3OLD+X3)/2 + .5*H*X3D;
    X4 = (X4OLD+X4)/2 + .5*H*X4D;
    X5 = (X5OLD+X5)/2 + .5*H*X5D;

    % ---- LOW rate (Ts) FIRST.  Signal flow is reversed in the adjoint, so
    %      the low-rate sampler is met first -- the opposite of Listing 29.1
    %      (zarchan.txt:10661).  Swapping these two blocks is silent.
    if S >= (TS - 0.0001)
        S = 0;
        Y1NEW = X5;
        TEMP1 = (Y1NEW - Y1OLD)*XNP*VC;
        TEMP2 = HFILTER*(Y2OLD + TEMP1)/TS + GFILTER*Y3OLD;
        Y2NEW = TEMP1 + Y2OLD + TS*(Y3OLD - TEMP2);
        Y3NEW = Y3OLD - TEMP2;
        Y7    = TEMP2 + Y7;                  % Y7NEW = TEMP2 + Y7OLD
        Y1OLD = Y1NEW;  Y2OLD = Y2NEW;  Y3OLD = Y3NEW;
        nLow = nLow + 1;
    end

    % ---- HIGH rate (Ts2): the preprocessor's adjoint, and the record.
    if S2 >= (TS2 - 0.0001)
        S2 = 0;
        Y6NEW = Y7;
        u = ppgain*(Y6NEW - Y6OLD);
        [Z, dY4] = preprocTaps(Z, u);
        Y4 = Y4 + dY4;                       % Y4NEW = Y4OLD + Y5OLD + Y11NEW
        Y6OLD = Y6NEW;
        n = n + 1;
        t(n) = TP;  miss(n) = XNT*X1;        % XMNT = XNT*X1
        nHigh = nHigh + 1;
    end
end
t = t(1:n);  miss = miss(1:n);
end


% =============================================================================
function [Zn, dY4] = preprocTaps(Zo, u)
%PREPROCTAPS  The adjoint of the moving-average preprocessor, ONE step.
%
%   Zo = [Y11 Y10 Y9 Y8 Y5] of the listing.  Unrolled, the increment this hands
%   the accumulator is u(k) + u(k-1) + ... + u(k-ntap+1) -- SIX terms at the
%   listing's ntap = 6, which is exactly what makes it the transpose of
%   Listing 29.1's six-term PZ.  SELFCHECK 1 probes THIS function with an
%   impulse: a check that restates the recursion instead of calling it proves
%   nothing about the code that runs.
dY4 = Zo(end) + u;
Zn  = [u, Zo(1:end-1) + u];
end


% =============================================================================
function [miss, nEval] = forwardMiss(XNT,XNP,TAP,VC,BETA,TS,TS2,H,tfList, ...
                                     ntap,ppgain)
%FORWARDMISS  Listing 29.1, zarchan.txt:10645-10658.
%   One run per flight time; the miss is Y at t = TF.  XLAMD is dead code in
%   the book and is not carried -- see the header.
GFILTER = 1.0 - BETA^2;
HFILTER = (1.0 - BETA)^2;
miss  = zeros(1, numel(tfList));
nEval = 0;
for k = 1:numel(tfList)
    TF = tfList(k);
    Y=0; YD=0; YDD=0; XNL=0; XNLD=0; XNC=0; T=0; S=0; S2=0;
    XLAMHOLD=0; XLAMDHOLD=0; XLAM=0; PZ=0;
    lam = zeros(1, ntap);            % the listing's Y1..Y5 PLUS its Y5OLD
    while T <= (TF - 1e-5)
        YOLD=Y; YDOLD=YD; XNLOLD=XNL;
        STEP = 1;  FLAG = 0;
        while STEP <= 1
            if FLAG == 1
                Y=Y+H*YD; YD=YD+H*YDD; XNL=XNL+H*XNLD; T=T+H; STEP=2;
            end
            TGO  = TF - T + 0.00001;
            XLAM = Y/(VC*TGO);
            XNLD = (XNC - XNL)/TAP;
            YDD  = XNT - XNL;
            FLAG = 1;
            nEval = nEval + 1;
        end
        Y   = .5*(YOLD   + Y   + H*YD);
        YD  = .5*(YDOLD  + YD  + H*YDD);
        XNL = .5*(XNLOLD + XNL + H*XNLD);
        S = S + H;  S2 = S2 + H;
        % ---- HIGH rate FIRST in the forward model (zarchan.txt:10630): the
        %      high-rate sampler is the one the signal meets first.
        if S2 >= (TS2 - 1e-5)
            S2  = 0;
            lam = [XLAM, lam(1:end-1)];
            PZ  = ppgain*sum(lam);   % the listing's six-term PZ, one gain
        end
        if S >= (TS - 1e-5)
            S = 0;
            RES       = PZ - (XLAMHOLD + TS*XLAMDHOLD);
            XLAMHNEW  = GFILTER*RES + XLAMHOLD + TS*XLAMDHOLD;
            XLAMDHNEW = HFILTER*RES/TS + XLAMDHOLD;
            XNC       = XNP*VC*XLAMDHNEW;
            XLAMHOLD  = XLAMHNEW;  XLAMDHOLD = XLAMDHNEW;
        end
    end
    miss(k) = Y;
end
end


% =============================================================================
function K = structuralChecks(XNT,XNP,TAP,VC,BETA,TS,TS2,TF,H,ntap,ppgain, ...
                              t,miss_ft,nLow,nHigh,peak_ft)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never evidence that
%   this port reproduces Zarchan -- only the weaker claim that it agrees with
%   itself, which is what catches wiring faults.
K = struct();  names = {};  oks = [];  detail = {};

% 1  THE TRANSPOSE IDENTITY.  Probe the adjoint preprocessor with a unit
%    impulse and compare its increment sequence against the FIR the forward
%    model applies.  This is the only check that would notice the two listings
%    drifting apart at the preprocessor -- which is precisely where the book's
%    own prose disagrees with its own code.
Z = zeros(1, ntap-1);  h = zeros(1, ntap+3);
for i = 1:numel(h)
    [Z, h(i)] = preprocTaps(Z, double(i == 1));
end
K.fir_resid = max(abs(ppgain*h - [ppgain*ones(1,ntap), zeros(1,3)]));
names{end+1} = 'adjoint preprocessor == forward FIR (six taps at 0.2)';
oks(end+1)   = K.fir_resid == 0;
detail{end+1} = sprintf('impulse response [%s] vs %d taps of %g; resid %.3e', ...
    strtrim(sprintf('%g ', ppgain*h(1:min(numel(h),8)))), ntap, ppgain, K.fir_resid);

% 2  VC CANCELS.  Y4 is proportional to VC (TEMP1 carries XNP*VC) and enters
%    the states only as Y4/(VC*tgo).  Doubling VC is EXACT in binary floating
%    point at every step of that chain, so this must come back at exactly 0.
%    Any nonzero result means a genuine closing-velocity dependence has
%    appeared, and it is invisible everywhere else in this file.
[~, mv2] = adjointRun(XNT,XNP,TAP,2*VC,BETA,TS,TS2,TF,H,ntap,ppgain,1.0);
K.vc_resid = max(abs(mv2 - miss_ft));
names{end+1} = 'VC cancels (VC -> 2*VC leaves the miss bit-identical)';
oks(end+1)   = K.vc_resid == 0;
detail{end+1} = sprintf('max |dmiss| = %.3e ft between VC = %g and %g ft/s', ...
    K.vc_resid, VC, 2*VC);

% 3  LINEARITY.  This is the adjoint of a LINEAR loop: doubling the impulse on
%    X3 must double every sample, bit for bit.  Any nonlinearity smuggled in
%    later (a limit, an abs, a saturation) breaks this and nothing else here
%    would see it -- the curve would stay smooth and plausible.
[~, mx2] = adjointRun(XNT,XNP,TAP,VC,BETA,TS,TS2,TF,H,ntap,ppgain,2.0);
K.lin_resid = max(abs(mx2 - 2*miss_ft));
names{end+1} = 'linear in the impulse (X3(0)=2 doubles the miss exactly)';
oks(end+1)   = K.lin_resid == 0;
detail{end+1} = sprintf('max |miss(2u) - 2 miss(u)| = %.3e ft', K.lin_resid);

% 4  CADENCE.  The whole point of the listing is TWO rates.  An off-by-one in
%    the S/S2 accumulators still produces a smooth, plausible curve.
K.n_low = nLow;  K.n_high = nHigh;
names{end+1} = 'sampler cadence: TF/Ts low-rate, TF/Ts2 high-rate fires';
oks(end+1)   = (nLow == round(TF/TS)) && (nHigh == round(TF/TS2)) && ...
               (abs(t(end) - TF) <= 2*H) && (numel(t) == nHigh + 1);
detail{end+1} = sprintf('%d low (want %d), %d high (want %d), last t = %.5f s', ...
    nLow, round(TF/TS), nHigh, round(TF/TS2), t(end));

% 5  DELETE EXPERIMENT.  Drop the sixth preprocessing tap -- the PROSE reading
%    (five samples, dc gain 1.0).  If that made no difference, the header's
%    whole argument about a six-tap filter would be decoration.  Threshold is
%    2% of peak: the dc gain moves 1.2 -> 1.0, a 20% loop-gain change in a
%    deliberately sluggish sampled loop, so a result NEAR 2% would itself be
%    the surprise.
if ntap >= 3
    [~, m5] = adjointRun(XNT,XNP,TAP,VC,BETA,TS,TS2,TF,H,ntap-1,ppgain,1.0);
    pk5 = max(abs(m5));
    K.tap_delta = abs(pk5 - peak_ft);
    names{end+1} = 'the sixth preprocessing tap is load-bearing';
    oks(end+1)   = ~isfinite(pk5) || K.tap_delta > 0.02*peak_ft;
    detail{end+1} = sprintf(['peak %.4f ft at %d taps vs %.4f ft at %d taps ' ...
        '(%.2f%% of peak; want > 2%%)'], peak_ft, ntap, pk5, ntap-1, ...
        100*K.tap_delta/max(peak_ft, realmin));
else
    K.tap_delta = NaN;
end

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
function B = benchmarks(R, BETA, TS, TS2, peak_ft, ALT, wantFwd)
%BENCHMARKS  Computed vs published, with the PROVENANCE of every band.
%
%   *** EVERY BOOK VALUE HERE WAS GREPPED OUT OF THE DECODED BOOK; THE LINE
%   NUMBER IN .source IS THE grep LINE AND RESOLVES IN BOTH FILES. ***  See the
%   header note: the former second "decoded :NNNN" number was a MATLAB
%   readlines() index mislabelled, +187 through this chapter, and was removed.
%   Nothing was read off a figure.
B = emptyBench();
srcG  = 'p.753 eq (zarchan.txt:10609) + Table 29.1 (:10636)';
srcH  = 'p.753 eq (zarchan.txt:10610) + Table 29.1 (:10636)';
srcTS = 'p.752 text (zarchan.txt:10600) + Table 29.1 (:10638-10639)';

% ---- 1-2  the fading memory gains ------------------------------------------
if abs(BETA - 0.8) < 1e-12
    B = addVal(B, 'fading memory G = 1 - b^2 at b = 0.8', 0.36, 1 - BETA^2, ...
        'exact', 1e-12, srcG, '-', ...
        ['the minus sign vanished in extraction ("G = 1 b2") and is recovered ' ...
         'from the listing''s own GFILTER=1.-BETA^2, not guessed']);
    B = addVal(B, 'fading memory H = (1-b)^2 at b = 0.8', 0.04, (1-BETA)^2, ...
        'exact', 1e-12, srcH, '-', ...
        ['same recovery from HFILTER=(1.-BETA)^2.  H is the gain that gets ' ...
         'divided by TS in the filter update, so a swap with G is not subtle ' ...
         'in the answer but is invisible in the source']);
else
    B = addSkip(B, 'fading memory G = 1 - b^2 at b = 0.8', '0.36', ...
        sprintf('this run used b = %g; Table 29.1 quotes b = 0.8', BETA), ...
        srcG, '-');
    B = addSkip(B, 'fading memory H = (1-b)^2 at b = 0.8', '0.04', ...
        sprintf('this run used b = %g; Table 29.1 quotes b = 0.8', BETA), ...
        srcH, '-');
end

% ---- 3  the reason the preprocessor exists at all --------------------------
B = addVal(B, 'data-rate ratio Ts/Ts2', 5, TS/TS2, 'exact', 1e-12, srcTS, '-', ...
    ['"we have five times as much data as can be used by the digital noise ' ...
     'filter" -- 50 Hz in, a 10 Hz filter']);

% ---- 4  THE ROW THIS FILE EXISTS FOR ---------------------------------------
if ~isempty(R.fwd)
    F    = R.fwd;
    % max(0.5% of peak, 4*H) -- the same pairing the sibling adjoint ports use.
    % The 4*H floor stops a refined H from eventually punishing a correct run.
    band = max(0.005*peak_ft, 4*R.p.H);
    [worst, iw] = max(abs(F.dev_ft));
    B = addBound(B, 'Listing 29.2 vs Listing 29.1 (adjoint vs forward)', ...
        sprintf('|d| <= max(0.5%% of peak, 4H) = %.4f ft at all %d flight times', ...
                band, numel(F.tf)), ...
        sprintf('worst %+.4f ft (%.2f%% of peak) at tF = %g s', ...
                F.dev_ft(iw), 100*worst/max(peak_ft, realmin), F.tf(iw)), ...
        worst <= band, ...
        'p.760 text (zarchan.txt:10681) -- "exact agreement"', ...
        'ft', ...
        ['the book''s word is an adjective about a plot, so the band is this ' ...
         'port''s own O(H) floor, normalised by the PEAK because the miss ' ...
         'curve crosses zero.  Both schemes are Heun at the same H with ' ...
         'aligned samplers, so the expected residual is ~2e-4 ft; a result ' ...
         'near the edge means they have stopped being transposes']);
else
    B = addSkip(B, 'Listing 29.2 vs Listing 29.1 (adjoint vs forward)', ...
        '"exact agreement"', ...
        sprintf(['Listing 29.1 was not run (''forward'' is %s, %d flight ' ...
                 'times survived the TF clip).  This is the ONLY row that ' ...
                 'checks the transposition itself'], ...
                mat2str(wantFwd), numel(R.p.fwd_tf)), ...
        'p.760 text (zarchan.txt:10681)', 'ft');
end

% ---- 5  SHAPE: the b = 0.3 claim -------------------------------------------
if ~isempty(ALT)
    B = addShape(B, 'b = 0.3 beats b = 0.8 (faster filter, less miss)', ...
        'peak |miss| at b = 0.3  <  peak |miss| at b = 0.8', ...
        sprintf('%.4f ft at b = 0.3 vs %.4f ft at b = %g', ...
                ALT.peak_ft, peak_ft, BETA), ...
        ALT.peak_ft < peak_ft, ...
        'p.761 text (zarchan.txt:10693)', ...
        ['"when the bandwidth of the fading memory filter is increased ' ...
         '(b = 0.3) the guidance system performance improves".  An ORDERING, ' ...
         'not a figure reading -- Fig. 29.5 publishes no number']);
else
    B = addSkip(B, 'b = 0.3 beats b = 0.8 (faster filter, less miss)', ...
        'peak(b=0.3) < peak(b=0.8)', ...
        'this run IS b = 0.3, so there is nothing to compare it against', ...
        'p.761 text (zarchan.txt:10693)', 'ft');
end

% ---- INFO / SKIP.  Reported, never adjudicated. ---------------------------
B = addInfo(B, 'the book contradicts itself about the preprocessor', ...
    'prose: five taps / code: six', ...
    sprintf('ported as %d taps at gain %g (dc gain %g)', ...
            R.p.ntap, R.p.ppgain, R.p.ntap*R.p.ppgain), ...
    'zarchan.txt:10600 (prose) vs :10653 and :10664-10680 (both listings)', ...
    ['NOT SCORED as a value because both sides ARE the book.  The CODE is ' ...
     'ported: Listing 29.1 sums six samples, and Listing 29.2''s unrolled ' ...
     'chain increments its accumulator with six -- they are transposes of ' ...
     'each other, which the prose''s five cannot be.  SELFCHECK 5 measures ' ...
     'what the sixth tap is worth']);

B = addInfo(B, 'peak miss over the sweep', ...
    '"miss distances can be quite large"', ...
    sprintf('%.4f ft at tF = %.3f s', peak_ft, R.peak_t), ...
    'p.761 text (zarchan.txt:10693)', ...
    ['NOT SCORED: no number is printed for Fig. 29.4, and Listing 29.1''s ' ...
     'axis([00,10,-40,100]) is a plot range the author chose, not a ' ...
     'measurement -- even though it does bracket this curve almost exactly. ' ...
     'This package has been handed phantom figure readings before']);

B = addSkip(B, 'adjoint time == tgo of the maneuver onset (Fig. 29.6)', ...
    'same curve as Fig. 29.4', ...
    ['NOT IMPLEMENTED: the claim needs Listing 29.1 modified to hold tF at ' ...
     '10 s and sweep the maneuver START time instead, which is a different ' ...
     'program.  Scoreable, and deliberately left unscored rather than asserted'], ...
    'p.761 text (zarchan.txt:10694)', 'ft');
end


% =============================================================================
function mustDivide(a, b, na, nb)
%MUSTDIVIDE  a must be a whole multiple of b.  A sampler whose period is not an
%   integer number of steps drifts by one step at a time and produces a
%   perfectly smooth, entirely wrong curve, so this is refused, not warned about.
q = a/b;
if abs(q - round(q)) > 1e-9
    error('mgs:zarchan:dualRateAdjoint:cadence', ...
        ['%s = %g is not a whole multiple of %s = %g (ratio %.6f).  The ' ...
         'sampler would drift a step at a time and the curve would stay ' ...
         'smooth while being wrong.'], na, a, nb, b, q);
end
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER are the schema benchSuite's adaptGeneric
%   consumes.  One constructor per kind, so the order cannot drift.
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
%ADDINFO  Reported, never adjudicated: it can neither pass nor fail.
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
L{end+1} = 'Zarchan Listing 29.2 -- ADJOINT OF A HOMING LOOP WITH TWO SAMPLING RATES';
L{end+1} = '   (with Listing 29.1, the forward model it must reproduce)';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['   nT = %g ft/s^2 (%.4g g)   N'' = %g   T = %g s   ' ...
    'Vc = %g ft/s   b = %g'], p.XNT, p.XNT/32.2, p.XNP, p.TAP, p.VC, p.BETA);
L{end+1} = sprintf(['   Ts = %g s (%.0f Hz)   Ts2 = %g s (%.0f Hz)   ' ...
    'preprocessor %d taps x %g   TF = %g s   H = %g s'], ...
    p.TS, 1/p.TS, p.TS2, 1/p.TS2, p.ntap, p.ppgain, p.TF, p.H);
L{end+1} = '';
L{end+1} = 'MISS vs FLIGHT TIME  (the listing''s XMNT; ONE run, every flight time)';
if ~isempty(R.fwd)
    L{end+1} = sprintf('%9s %14s %14s %12s', 'tF [s]', 'adjoint 29.2', ...
        'forward 29.1', 'd [ft]');
    for k = 1:numel(R.fwd.tf)
        L{end+1} = sprintf('%9.3f %14.4f %14.4f %12.5f', R.fwd.tf(k), ...
            R.fwd.adj_ft(k), R.fwd.miss_ft(k), R.fwd.dev_ft(k)); %#ok<AGROW>
    end
else
    stride = max(1, round(0.5/p.TS2));
    rows = unique([1:stride:numel(R.t), numel(R.t)]);
    L{end+1} = sprintf('%9s %14s', 'tF [s]', 'adjoint 29.2');
    for r = rows
        L{end+1} = sprintf('%9.3f %14.4f', R.t(r), R.miss_ft(r)); %#ok<AGROW>
    end
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
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning';
    L{end+1} = '            NOT CHECKED, which is not the same thing as "fine".';
else
    L{end+1} = 'BENCHMARKS  (book values grepped from zarchan.txt; lines in .source)';
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
                L{end+1} = sprintf('   %-4s %-52s %s', vstr(b.ok), b.what, ...
                    b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf(['   %-4s %-52s book %s %s / ours %s %s ' ...
                    '(%+.3f%%, band %.4g%%)'], vstr(b.ok), b.what, b.book_str, ...
                    b.units, b.ours_str, b.units, b.pct, 100*b.tol); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped/info -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: the adjoint identity is a LINEARITY identity.  This loop has';
L{end+1} = '   no acceleration limit, no drag and no saturation -- our own dominant';
L{end+1} = '   error source is a lag that SATURATES, which no adjoint can represent.';
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
%DRAWCURVES  Base MATLAB graphics only.
figure;  hold on;  grid on;
plot(R.t, R.miss_ft, '-');
lg = {'Listing 29.2 adjoint'};
if ~isempty(R.fwd)
    plot(R.fwd.tf, R.fwd.miss_ft, 'o');
    lg{end+1} = 'Listing 29.1 forward';
end
if ~isempty(R.beta_alt)
    plot(R.beta_alt.t, R.beta_alt.miss_ft, '--');
    lg{end+1} = sprintf('adjoint, b = %g', R.beta_alt.BETA);
end
xlabel('Flight Time (Sec)');
ylabel('Target Maneuver Miss (Ft)');
title('Homing loop with two samplers operating at different rates');
legend(lg{:}, 'Location', 'best');
hold off;
end
