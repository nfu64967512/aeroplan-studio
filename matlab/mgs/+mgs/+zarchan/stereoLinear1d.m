function R = stereoLinear1d(opts)
%MGS.ZARCHAN.STEREOLINEAR1D  Zarchan Listing 34.1 -- ONE-DIMENSIONAL
%   ENGAGEMENT SIMULATION BASED ON LINEARIZED GEOMETRY (Ch.34, boost-phase
%   kill-vehicle sizing).
%
%   R = mgs.zarchan.stereoLinear1d()                 the listing's nominal case
%   R = mgs.zarchan.stereoLinear1d('APN', 1)         augmented PN
%   R = mgs.zarchan.stereoLinear1d('IOPTION', 1)     actual thrust/weight nT
%   R = mgs.zarchan.stereoLinear1d('ITGT', 2)        the two-stage ICBM threat
%   R = mgs.zarchan.stereoLinear1d('assert', false)  warn instead of erroring
%
%   Line-by-line port of Listing 34.1 (zarchan.txt:13193-13220), integrated by
%   the book's own second-order Runge-Kutta (Heun) scheme.  Listing 34.1 IS
%   Listing 2.2 with three things added: a target-acceleration MODEL (parabola
%   or real thrust/weight), an APN term, and a divert accumulator DELV.
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, g = 32.2 ft/s^2.  NOT SI.
%   Two conversions are the LISTING'S OWN and are reproduced, not corrected:
%     PRED = <km> * 3280      PIP error, km -> ft
%     DELV / 3.28             divert, ft/s -> m/s
%   3.28 is not 3.28084.  Every divert benchmark below is a RATIO in which the
%   3.28 cancels, so the book's rounded factor cannot leak into a score.
% ===========================================================================
%
%   WHY THIS ONE MATTERS HERE.  Ch.34 sizes a kill vehicle by DIVERT -- the
%   integral of |acceleration| -- against a target whose maneuver is a known
%   function of time, and it gets two closed forms out of that (rows 9, 10).
%   That is the same shape as our own "how much authority does this profile
%   actually need" question.  What it is NOT: no lag, no drag, no gravity, no
%   LOS floor, constant closing velocity, and a maneuver that is a function of
%   TIME rather than of geometry.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION, AND WHAT WAS DONE TO IT
%
%   The OCR of this book maps '<' -> ',' and '>' -> '.' inside code.  Listing
%   2.2, four hundred pages earlier, prints "while T,=(TF-1e-5)", which can
%   only be "<=", and that fixes the mapping.  Applying it:
%
%     1  "while T,(TF-.0001)"  -> "while T < (TF-.0001)".  Single comma, so
%        "<" and not "<=".  Immaterial here: T is an exact multiple of H and
%        TF-0.0001 is not, so both forms stop on the same step.
%     2  "while STEP,=1"       -> "while STEP <= 1".  Body sets STEP=2 to leave.
%     3  "if T,180." / "if T,120." / "elseif T,240."  -> "<".  Stage boundaries.
%     4  "if S.=.09999"        -> "if S >= .09999".  S resets to 0 on record,
%        so "<=" would record forever.
%
%     5  *** THE ONE UNRESOLVED COMPARISON ***  The extractor flags exactly one
%        variable-to-variable comparison it could not settle from shape:
%        "if XNC.XNCMAX".  RESOLVED AS  if XNC > XNCMAX, XNC = XNCMAX.
%        Three reasons, strongest last:
%          a  the character mapping above already says '.' is '>';
%          b  the very next line is "if XNC < -XNCMAX, XNC=-XNCMAX", and only
%             ">" completes a symmetric two-sided limiter;
%          c  it is MEASURABLE.  With "<" the test is true on essentially every
%             step (XNC is below 966 ft/s^2 almost always), so XNC would be
%             pinned at +30 g and DELV would come out as 966*tF: benchmark 9
%             would read 17,671 m/s against the book's 540, a factor of 33.
%             That is a scored row, not a shape argument.
%
%     6  The trailing "figure plot(...) ... save datfil.txt output /ascii disp
%        'simulation finished' Y DELV/3.28" is the listing's I/O.  "/ascii" is
%        a typo for "-ascii" (Listing 2.2 has it right); nothing is written to
%        disk here at all.  The two quantities the book prints by hand at the
%        end -- Y and DELV/3.28 -- come back as R.Y_ft and R.dv_m.
%     7  "FLAG=0" after the inner loop is dead (the outer loop re-initialises
%        it).  Recorded here as a comment, not as code.
%     8  SUM and XN accumulate a mean target acceleration the listing never
%        uses.  Kept, and returned as R.xnt_mean_g, so their absence cannot
%        later look like a transcription loss.
%     9  No minus-sign damage was found in the listing: -212, -2622, -642,
%        -57.3 and -XNCMAX all survive as minus signs, and each of 44000 /
%        54100 / 440660 / 725850 / 168120 / 182250 is confirmed by a scored
%        row (1, 2, 3).  The damage IS present in the two worked examples the
%        divert rows use -- "(170 110)" and "(223 180)" are "170-110" and
%        "223-180" -- and both readings are confirmed arithmetically there.
%
%   VM is dead algebra: HEDEG = -57.3*PRED/(VM*TF) then YD = -VM*HEDEG/57.3,
%   so YD = PRED/TF and VM cancels.  VC cancels too (structural row 2).  Both
%   are kept because deleting them would change what this file transcribes.
%
%   *** THE LAST RECORDED SAMPLE IS ROUND-OFF, AND IT LOOKS FINE. ***  The
%   listing's final record lands at T = TF, where TGO = 1e-5 and the PN gain
%   3/TGO^2 is 3e10.  The zero-effort miss it multiplies, Y + YD*TGO, is a
%   difference of two numbers of order 0.1 whose accumulated round-off after
%   180,000 additions is comparable to the difference itself.  The quotient is
%   finite, smooth-looking and meaningless.  Every ACCELERATION read below
%   comes from the last sample with TGO >= 'tgo_floor' (default 0.05 s).  DELV
%   is an integral of a bounded quantity and is not affected: the contaminated
%   last step contributes H*|XNC|, about 1e-5 of the total.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS -- every book value was grepped out of zarchan_decoded.txt and
%   the line number is in its .source field.  Nothing was read off a figure;
%   where the book states a SHAPE about a figure, that prose is greppable and
%   is scored as an ordering.  Rows 1, 2, 3 and 7 are printed by the book to
%   ONE significant figure, so their band is +/- 0.5 in that figure -- the
%   correct comparison for a 1-s.f. number, not a band chosen to fit us.
%
%   OUTPUTS
%     R.t R.tgo R.xnt_g R.xnc_g R.delv_m   the listing's Array* records
%     R.Y_ft R.dv_m R.xnt_mean_g           what the listing prints at the end
%     R.cases      the benchmark sub-runs ([] if 'benchmark' is false)
%     R.bench      benchmark rows, schema shared with the other ports
%     R.selfcheck  .names/.oks/.detail -- structural, NOT book values
%     R.pass       true only if a row was SCORED and everything passed
%     R.verdict    '' when clean; ALWAYS formed.  'assert' picks error vs warn.
%
%   See also MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 34.1 line 1, verbatim.
    opts.ITGT    (1,1) double {mustBeMember(opts.ITGT,[1 2])}    = 1
    opts.IOPTION (1,1) double {mustBeMember(opts.IOPTION,[0 1])} = 0
    opts.APN     (1,1) double {mustBeMember(opts.APN,[0 1])}     = 0
    opts.XNTAV   (1,1) double {mustBeFinite} = 117.6      % ft/s^2, ITGT=2 mean
    opts.XNTMAX  (1,1) double {mustBeFinite, mustBeNonnegative} = 9*32.2
    opts.XNCMAX  (1,1) double {mustBeFinite, mustBePositive}    = 966.0  % 30 g
    opts.XNP     (1,1) double {mustBeFinite, mustBePositive}    = 3.0
    opts.VC      (1,1) double {mustBeFinite, mustBePositive}    = 18000.0
    opts.VM      (1,1) double {mustBeFinite, mustBePositive}    = 9000.0
    % PIP error in KILOMETRES.  The listing writes "PRED=0.*3280." -- the
    % leading 0 is the km, 3280 is the km->ft factor.
    opts.PRED_KM (1,1) double {mustBeFinite} = 0.0
    % [] means the listing's own rule: 180 s for the IRBM, 240 s for the ICBM.
    opts.TF      double {mustBeReal, mustBeFinite} = []
    opts.H       (1,1) double {mustBeFinite, mustBePositive} = 0.001
    % The listing tests "S >= .09999" -- 0.1 s minus a hair.  Keeping the
    % literal would silently change meaning for H > 1e-5; this tests against
    % samp - H/2, which records every 100th step at the listing's own H.
    opts.samp    (1,1) double {mustBeFinite, mustBePositive} = 0.1
    % Samples closer to intercept than this are excluded from every
    % ACCELERATION reading -- see the round-off note in the header.  Setting
    % it to 0 restores the listing's raw last point, and its round-off.
    opts.tgo_floor (1,1) double {mustBeFinite, mustBeNonnegative} = 0.05
    opts.ft_per_m  (1,1) double {mustBeFinite, mustBePositive} = 3.28
    % THE ONLY SWITCHES.  Not nargin: with an arguments block that declares
    % only name-value options MATLAB reports nargin = 0 however many were
    % passed, so "if nargin == 0" would be a mode that never fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

TF = opts.TF;
if isempty(TF)
    if opts.ITGT == 1
        TF = 180.0;                      % the listing's own if/else
    else
        TF = 240.0;
    end
end
if ~isscalar(TF) || TF <= 0
    error('mgs:zarchan:stereoLinear1d:tf', ...
        'TF must be a positive scalar or [] (the listing''s own rule); got %s.', ...
        mat2str(opts.TF));
end
if opts.H > opts.samp
    error('mgs:zarchan:stereoLinear1d:step', ...
        ['H = %g s is larger than the sample interval samp = %g s, so the ' ...
         'returned record would be sampled at the step rate while its own ' ...
         'time vector claimed the coarser one.'], opts.H, opts.samp);
end
if TF <= 20*opts.H
    error('mgs:zarchan:stereoLinear1d:short', ...
        ['TF = %g s is under twenty steps at H = %g s.  The 1/TGO^2 gain ' ...
         'makes a coarse run smooth and wrong rather than obviously broken, ' ...
         'so this is refused rather than warned about.'], TF, opts.H);
end
if opts.IOPTION == 1 && ~(abs(TF - 180) < 1e-9 || abs(TF - 240) < 1e-9)
    warning('mgs:zarchan:stereoLinear1d:burnout', ...
        ['IOPTION = 1 (real thrust/weight) with TF = %g s.  The listing ' ...
         'HARD-CODES the burnout times 180 s (IRBM) and 120/240 s (ICBM); ' ...
         'they do NOT follow TF, so the target burns out at the wrong ' ...
         'fraction of this flight.  Only the parabola scales with TF.'], TF);
end

PRED = opts.PRED_KM * 3280.0;                    % the listing's km -> ft

S0 = runEngagement(opts.ITGT, opts.IOPTION, opts.APN, TF, opts.XNTMAX, ...
                   opts.XNTAV, opts.XNCMAX, opts.XNP, opts.VC, opts.VM, ...
                   PRED, opts.H, opts.samp, opts.ft_per_m);
if S0.n < 3
    error('mgs:zarchan:stereoLinear1d:empty', ...
        ['the run recorded %d sample(s) at TF = %g s, samp = %g s.  A curve ' ...
         'with no interior cannot support any of the shape rows.'], ...
        S0.n, TF, opts.samp);
end

R = struct();
R.t          = S0.t;
R.tgo        = S0.tgo;
R.xnt_g      = S0.xnt_g;
R.xnc_g      = S0.xnc_g;
R.delv_m     = S0.delv_m;
R.Y_ft       = S0.Y_ft;
R.dv_m       = S0.dv_m;
R.xnt_mean_g = S0.xnt_mean_g;
R.units      = ['ft, ft/s, ft/s^2, g = 32.2 ft/s^2; DELV in m/s via the ' ...
                'listing''s own 3.28 ft/m, PIP in km via 3280 ft/km'];
R.p = struct('ITGT',opts.ITGT, 'IOPTION',opts.IOPTION, 'APN',opts.APN, ...
             'TF',TF, 'XNTMAX',opts.XNTMAX, 'XNTAV',opts.XNTAV, ...
             'XNCMAX',opts.XNCMAX, 'XNP',opts.XNP, 'VC',opts.VC, ...
             'VM',opts.VM, 'PRED_ft',PRED, 'PRED_KM',opts.PRED_KM, ...
             'H',opts.H, 'samp',opts.samp, 'tgo_floor',opts.tgo_floor, ...
             'ft_per_m',opts.ft_per_m, 'units',R.units);

R.selfcheck = structuralChecks(R, opts.XNP, opts.VC, opts.VM, opts.XNCMAX, ...
                               opts.H, opts.samp, opts.ft_per_m, opts.XNTAV);

if opts.benchmark
    R.cases = caseRuns(opts.XNP, opts.VC, opts.VM, opts.XNCMAX, opts.H, ...
                       opts.samp, opts.ft_per_m, opts.XNTAV);
    R.bench = benchmarks(R.cases, opts.H, opts.tgo_floor, opts.XNTAV, opts.XNCMAX);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.cases = [];
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
%      it depends on 'assert'.  benchSuite runs every port with assert off so
%      that one red row cannot hide the rest of the board.
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
        '(first: %s, book %s, ours %s, band %s).'], numel(bad), sum(ev), ...
        R.bench(bad(1)).what, R.bench(bad(1)).book_str, ...
        R.bench(bad(1)).ours_str, R.bench(bad(1)).band);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:stereoLinear1d:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN, complete, so the caller can see WHICH side moved.
        warning('mgs:zarchan:stereoLinear1d:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function out = runEngagement(ITGT, IOPTION, APN, TF, XNTMAX, XNTAV, XNCMAX, ...
                             XNP, VC, VM, PRED, H, samp, ftPerM)
%RUNENGAGEMENT  Listing 34.1, zarchan.txt:13193-13220, verbatim except I/O.
%
%   Every constant arrives as an ARGUMENT.  MATLAB local functions cannot see
%   the caller's workspace, and a previous wave in this package shipped a local
%   that read one anyway -- it threw on the first line of the first run, which
%   no amount of static checking had caught beforehand.

count = 0;
HEDEG = -57.3*PRED/(VM*TF);
YD    = -VM*HEDEG/57.3;          % = PRED/TF.  VM cancels; see the header.
Y     = 0.0;
T     = 0.0;
S     = 0.0;
DELV  = 0.0;
SUM   = 0.0;
XN    = 0.0;

% Initialised before the loop so that the FLAG==1 branch inside it cannot read
% a value left over from anything else.
YDD = 0.0;  DELVD = 0.0;  XNC = 0.0;  XNT = 0.0;
TGO = TF - T + 0.00001;

nmax      = ceil(TF/samp) + 8;
ArrayT    = zeros(nmax,1);
ArrayXNT  = zeros(nmax,1);
ArrayXNC  = zeros(nmax,1);
ArrayDELV = zeros(nmax,1);
ArrayTGO  = zeros(nmax,1);

while T < (TF - 0.0001)
    YOLD = Y;  YDOLD = YD;  DELVOLD = DELV;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  Pass 1 evaluates the derivatives at
    % the current state; pass 2 takes a full Euler step, advances T by H, and
    % re-evaluates there.  T advances INSIDE this loop, exactly once, which is
    % what makes the second evaluation happen at t+H.  Move that line out and
    % the scheme quietly degrades to first order while still running smoothly.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            Y    = Y + H*YD;
            YD   = YD + H*YDD;
            DELV = DELV + H*DELVD;
            T    = T + H;
        end
        TGO   = TF - T + 0.00001;
        XNT   = targetAccel(T, ITGT, IOPTION, TF, XNTMAX, XNTAV);
        XLAMD = (Y + YD*TGO)/(VC*TGO*TGO);
        XNC   = XNP*VC*XLAMD + 0.5*APN*XNP*XNT;   % APN = 0 -> plain PN
        if XNC > XNCMAX                           % <<< the resolved comparison
            XNC = XNCMAX;
        end
        if XNC < -XNCMAX
            XNC = -XNCMAX;
        end
        DELVD = abs(XNC);                         % divert = int |nc| dt
        YDD   = XNT - XNC;
        FLAG  = 1;
    end
    % The listing writes "FLAG = 0" here as well.  Dropped: the outer loop
    % re-initialises FLAG on its next pass, so the assignment cannot be read.
    % Kept as a comment so nobody re-adds it thinking a state machine was lost.

    % Heun corrector, in the listing's own algebra.
    Y    = 0.5*(YOLD    + Y    + H*YD);
    YD   = 0.5*(YDOLD   + YD   + H*YDD);
    DELV = 0.5*(DELVOLD + DELV + H*DELVD);

    S = S + H;
    if S >= samp - 0.5*H
        S     = 0.0;
        SUM   = SUM + XNT;      % the listing accumulates these and never uses
        XN    = XN + 1.0;       % them; kept so that their absence cannot look
        count = count + 1;      % like a transcription loss
        ArrayT(count)    = T;
        ArrayXNT(count)  = XNT/32.2;
        ArrayXNC(count)  = XNC/32.2;
        ArrayDELV(count) = DELV/ftPerM;
        ArrayTGO(count)  = TGO;
    end
end

out           = struct();
out.n         = count;
out.t         = ArrayT(1:count);
out.xnt_g     = ArrayXNT(1:count);
out.xnc_g     = ArrayXNC(1:count);
out.delv_m    = ArrayDELV(1:count);
out.tgo       = ArrayTGO(1:count);
out.Y_ft      = Y;                  % what the listing prints as "Y"
out.DELV_ft   = DELV;
out.dv_m      = DELV/ftPerM;        % what the listing prints as "DELV/3.28"
if XN > 0
    out.xnt_mean_g = SUM/XN/32.2;
else
    out.xnt_mean_g = NaN;
end
end


% =============================================================================
function XNT = targetAccel(T, ITGT, IOPTION, TF, XNTMAX, XNTAV)
%TARGETACCEL  The apparent target maneuver, zarchan.txt:13199-13213.
%
%   ONE source of truth: the integrator and the profile benchmarks call this
%   same function, so a row cannot pass against a second copy of the numbers.
%
%   NOTE the burnout times are HARD-CODED (180 s IRBM; 120/240 s ICBM) and do
%   NOT scale with TF.  Only the PARABOLA follows TF.  That is the listing's,
%   not an omission, and the caller warns when IOPTION = 1 meets a foreign TF.
if ITGT == 1
    if IOPTION == 0
        XNT = XNTMAX*(T/TF)^2;                        % nT = nTMAX (t/tF)^2
    else
        if T < 180.0
            WGT = -212.0*T + 44000.0;   TRST = 54100.0;
        else
            WGT = 3300.0;               TRST = 0.0;
        end
        XNT = 32.2*TRST/WGT;
    end
else
    if IOPTION == 0
        XNT = XNTAV;                                  % constant, not parabolic
    else
        if T < 120.0
            WGT = -2622.0*T + 440660.0; TRST = 725850.0;
        elseif T < 240.0
            WGT = -642.0*T + 168120.0;  TRST = 182250.0;
        else
            WGT = 5500.0;               TRST = 0.0;
        end
        XNT = 32.2*TRST/WGT;
    end
end
end


% =============================================================================
function C = caseRuns(XNP, VC, VM, XNCMAX, H, samp, ftPerM, XNTAV)
%CASERUNS  The five book experiments the benchmark rows score.  Each is the
%   listing with the switches the book says it used, and nothing else.
C = struct();

% Fig. 34.3 -- PN (APN = 0) against the REAL thrust/weight IRBM, tF = 180 s.
C.pn_actual  = runEngagement(1, 1, 0, 180.0, 9*32.2, XNTAV, XNCMAX, XNP, ...
                             VC, VM, 0.0, H, samp, ftPerM);
% Fig. 34.4 -- APN against the parabola, and against the real profile.
C.apn_parab  = runEngagement(1, 0, 1, 180.0, 9*32.2, XNTAV, XNCMAX, XNP, ...
                             VC, VM, 0.0, H, samp, ftPerM);
C.apn_actual = runEngagement(1, 1, 1, 180.0, 9*32.2, XNTAV, XNCMAX, XNP, ...
                             VC, VM, 0.0, H, samp, ftPerM);
% p.919 worked example: DVMVR = 0.25 * 36 m/s^2 * (170-110) s = 540 m/s.
% 36 m/s^2 in the listing's feet is 36*3.28; the 3.28 cancels in the ratio.
C.dv_mvr     = runEngagement(1, 0, 1, 60.0, 36.0*ftPerM, XNTAV, XNCMAX, XNP, ...
                             VC, VM, 0.0, H, samp, ftPerM);
% p.921 worked example: DVPIP = 1.5 * 89,000 m / (223-180) s = 3.1 km/s.
% Maneuver switched OFF (XNTMAX = 0) so the row sees the PIP term alone.
C.dv_pip     = runEngagement(1, 0, 1, 43.0, 0.0, XNTAV, XNCMAX, XNP, ...
                             VC, VM, 89.0*3280.0, H, samp, ftPerM);
end


% =============================================================================
function B = benchmarks(C, H, tgoFloor, XNTAV, XNCMAX)
%BENCHMARKS  Computed vs published, with the PROVENANCE of every band.
%
%   Bands:
%     '1 s.f.'  +/- 0.5 in the book's single printed figure (rows 1, 2, 3, 7)
%     'text'    the printed digits (rows 6, 9, 10)
%     'shape'   an ordering; no tolerance is possible, and none is claimed
%   Do NOT widen a band to turn a row green.  A red row is a RESULT.

B = emptyBench();

% ---- 1-3.  The apparent-maneuver profiles themselves. ----------------------
% No integrator involved: these are the transcribed thrust/weight constants
% evaluated where the book quotes them.  nT = 32.2*TRST/WGT RISES inside a
% stage (thrust constant, weight falling linearly), so each stage maximum is
% the last instant before its branch, which the simulation samples at T - H.
irbm = profilePeak(1, 180.0, 180.0, H, XNTAV);
B = addVal(B, 'IRBM apparent maneuver at burnout', 9.0, irbm, '1 s.f.', ...
    0.5/9.0, 'p.895 text, Fig. 34.2 (zarchan.txt:13145)', 'g', ...
    ['32.2*54100/(44000-212*179.999) = 298.3 ft/s^2.  This row is what ' ...
     'confirms 212 / 44000 / 54100 survived the OCR']);

icbm1 = profilePeak(2, 120.0, 240.0, H, XNTAV);
B = addVal(B, 'ICBM stage-1 peak (first staging event, 120 s)', 6.0, icbm1, ...
    '1 s.f.', 0.5/6.0, 'p.928 text, Fig. 34.15 (zarchan.txt:13598)', 'g', ...
    'confirms -2622 / 440660 / 725850');

icbm2 = profilePeak(2, 240.0, 240.0, H, XNTAV);
B = addVal(B, 'ICBM stage-2 peak (second staging event, 240 s)', 13.0, icbm2, ...
    '1 s.f.', 0.5/13.0, 'p.928 text, Fig. 34.15 (zarchan.txt:13598)', 'g', ...
    'confirms -642 / 168120 / 182250; the book says "nearly 13 g"');

% ---- 4.  Fig. 34.3, as an ORDERING. ---------------------------------------
% "30 g or three times the maximum acceleration capability of the target."
% The RATIO is the part of that sentence that is a statement about the model;
% the 30 is a one-figure reading off a plot and is INFO below.
[ncEnd, ntEnd, iEnd] = terminalPair(C.pn_actual, tgoFloor);
ratio = ncEnd/ntEnd;
B = addShape(B, 'PN terminal accel = 3 x terminal target accel', ...
    'nc(tF)/nT(tF) = 3, within 5%', ...
    sprintf('%.4f   (nc = %.3f g, nT = %.3f g at t = %.2f s)', ...
            ratio, ncEnd, ntEnd, C.pn_actual.t(iEnd)), ...
    abs(ratio/3 - 1) <= 0.05, 'p.899 text, Fig. 34.3 (zarchan.txt:13223)', ...
    ['exact for N'' = 3: nc -> 3 nT as tgo -> 0.  Chapter 2 gets the same ' ...
     'answer for a step maneuver, which is why the book says the two agree']);

B = addInfo(B, 'the 30 g the book prints for that same terminal value', ...
    '30 g', sprintf('%.2f g', ncEnd), ...
    'p.899 text, Fig. 34.3 (zarchan.txt:13223)', ...
    ['NOT SCORED: one significant figure read off a plot whose axis tops out ' ...
     'at 30 and whose command limit XNCMAX = 966 ft/s^2 IS exactly 30 g.  ' ...
     'Three times the IRBM''s own 9.26 g is 27.8 g, so the book has rounded ' ...
     'the target up to 10 g.  Row 4 scores the half of that sentence which ' ...
     'is a claim about the model rather than about the plot']);

% ---- 6-7.  Fig. 34.4, the APN peaks. --------------------------------------
[pkPar, tPar, iPar, nPar] = peakAccel(C.apn_parab, tgoFloor);
B = addVal(B, 'APN peak accel, PARABOLIC maneuver approximation', 4.5, pkPar, ...
    'text', 0.05, 'p.900 text, Fig. 34.4 (zarchan.txt:13224)', 'g', ...
    ['the book glosses 4.5 as "one half the maximum acceleration capability ' ...
     'of the target" in the same sentence, i.e. a round figure.  The exact ' ...
     'coefficient is 3*max_u[u(u-1-ln u)] = 0.48594, giving 4.373 g at ' ...
     'nTMAX = 9 g; the 5% band covers that rounding and nothing more']);

[pkAct, tAct, iAct, nAct] = peakAccel(C.apn_actual, tgoFloor);
B = addVal(B, 'APN peak accel, ACTUAL thrust/weight maneuver', 3.0, pkAct, ...
    '1 s.f.', 0.5/3.0, 'p.900 text, Fig. 34.4 (zarchan.txt:13224)', 'g', ...
    ['the book''s point: APN needs about a third of the target''s own ' ...
     'capability where PN needed three times it (row 4)']);

% ---- 8.  The shape claim that goes with them. -----------------------------
endPar = C.apn_parab.xnc_g(nPar);
endAct = C.apn_actual.xnc_g(nAct);
okShape = iPar < nPar && iAct < nAct && ...
          abs(endPar) <= 0.5*pkPar && abs(endAct) <= 0.5*pkAct;
B = addShape(B, 'APN peak is NOT at the end of flight (both models)', ...
    'interior peak and terminal accel <= 50% of peak, both models', ...
    sprintf(['parabolic: peak %.3f g at t = %.1f s (%.0f%% of tF), end ' ...
             '%.3f g;  actual: peak %.3f g at t = %.1f s (%.0f%% of tF), ' ...
             'end %.3f g'], pkPar, tPar, 100*tPar/180, endPar, ...
             pkAct, tAct, 100*tAct/180, endAct), okShape, ...
    'p.900 text, Fig. 34.4 (zarchan.txt:13224)', ...
    ['the book''s reason for caring: a peak away from intercept means a ' ...
     'saturated KKV can come OUT of saturation before it matters.  Under PN ' ...
     '(row 4) the peak is at the very end, which is the opposite case']);

% ---- 9-10.  The two divert formulas, on the book's own arithmetic. --------
% Both are derived in the book FOR THE MODEL OF FIG. 34.1 (zarchan.txt:13301),
% which is this listing, so instantiating them here tests THIS model.  The
% numbers 36 m/s^2 / 60 s and 89 km / 43 s come from two worked examples whose
% printed subtractions "(170 110)" and "(223 180)" are the book's dropped
% minus signs; 0.25*36*60 = 540 and 1.5*89000/43 = 3105 confirm both readings.
% *** THIS DETAIL STRING USED TO LIE, AND PRINTED THE PROOF BESIDE ITSELF. ***
% It read "Peak command %.1f ft/s^2 against the 966 limit -- unsaturated, so
% DELV is the pure formula", and what it printed was "Peak command 966.0 ft/s^2
% against the 966 limit -- unsaturated".  966.0 against a 966 limit is exactly
% SATURATED.  The verdict was hard-coded into the sprintf and could not be
% contradicted by the number it was standing next to.
%
% The measurement, now made rather than claimed: the limiter binds on ONE
% sample of the run -- the last one, at TGO = 1e-5, where the 3/TGO^2 gain is
% 3e10 and the zero-effort miss it multiplies is pure round-off (header).  Over
% the samples the port treats as physical (TGO >= tgo_floor) the peak is 57.4
% ft/s^2, 6% of the limit.  The row's CONCLUSION survives -- that one clipped
% step is 5.1e-4 of DELV -- but it is now a number, not an adjective.
[satM, bodyM, nlimM] = satStats(C.dv_mvr, tgoFloor, XNCMAX);
B = addVal(B, 'DVMVR = 0.25 nTMAX tF   (36 m/s^2, 60 s)', 540.0, ...
    C.dv_mvr.dv_m, 'text', 0.02, ...
    'p.919 worked example (zarchan.txt:13512); formula at :13274', 'm/s', ...
    sprintf(['APN, parabolic, tF = 60 s.  The coefficient is EXACT, not the ' ...
             'curve fit the book calls it: int_0^1 3[u^2 - u - u ln u] du = ' ...
             '1/4.  Peak command %.1f ft/s^2 against the %.0f limit; %d of ' ...
             '%d samples sit ON the limit and %d of those are at ' ...
             'TGO >= %g s; peak over TGO >= %g s is %.1f ft/s^2, %.0f%% of ' ...
             'the limit'], satM, XNCMAX, nlimM(1), numel(C.dv_mvr.xnc_g), ...
            nlimM(2), tgoFloor, tgoFloor, bodyM, 100*bodyM/XNCMAX));

[satP, bodyP, nlimP] = satStats(C.dv_pip, tgoFloor, XNCMAX);
B = addVal(B, 'DVPIP = 1.5 PIP / tF   (89 km, 43 s)', 3100.0, ...
    C.dv_pip.dv_m, 'text', 0.016, ...
    'p.921 worked example (zarchan.txt:13561-13563); formula at :13302', 'm/s', ...
    sprintf(['maneuver OFF (XNTMAX = 0), the PIP entered as the listing''s ' ...
             'own heading-error equivalent PRED = 89*3280 ft.  Band 1.6%% = ' ...
             'half of the book''s last printed figure (it prints "3.1 km/s"; ' ...
             'the formula itself gives 3104.65).  Peak command %.1f ft/s^2 ' ...
             'against the %.0f limit, %d samples on it (%d at ' ...
             'TGO >= %g s); peak over TGO >= %g s is %.1f ft/s^2'], ...
            satP, XNCMAX, nlimP(1), nlimP(2), tgoFloor, tgoFloor, bodyP));
end


% =============================================================================
function [pkAll, pkBody, nAtLim] = satStats(S, tgoFloor, XNCMAX)
%SATSTATS  What the acceleration limiter actually did, as three numbers.
%   pkAll   peak |command| over the whole record          [ft/s^2]
%   pkBody  peak over the samples the port treats as physical (TGO >= tgoFloor)
%   nAtLim  [total, how many of those are at TGO >= tgoFloor] on the limit
%   Exists so that no row has to describe saturation with an adjective.
a      = 32.2*abs(S.xnc_g);
pkAll  = max(a);
w      = S.tgo >= tgoFloor;
if any(w), pkBody = max(a(w)); else, pkBody = NaN; end
nAtLim = [sum(a >= XNCMAX*(1 - 1e-12)), sum(a(w) >= XNCMAX*(1 - 1e-12))];
end


function [nc, nt, i] = terminalPair(S, tgoFloor)
%TERMINALPAIR  The last sample that is NOT round-off.  See the header.
i = find(S.tgo >= tgoFloor, 1, 'last');
if isempty(i), i = numel(S.tgo); end
nc = S.xnc_g(i);
nt = S.xnt_g(i);
end


function [pk, tpk, ipk, n] = peakAccel(S, tgoFloor)
%PEAKACCEL  Peak |command| over the samples that are not round-off, plus the
%   index of the last usable sample so a caller can ask "was the peak at the
%   end" without re-deriving the cut.
n = find(S.tgo >= tgoFloor, 1, 'last');
if isempty(n), n = numel(S.tgo); end
[pk, ipk] = max(abs(S.xnc_g(1:n)));
tpk = S.t(ipk);
end


function pk = profilePeak(ITGT, stageEnd, TF, H, XNTAV)
%PROFILEPEAK  Max apparent maneuver [g] in the stage ending at stageEnd, read
%   on the simulation's own 0.1-s grid PLUS the last step before the branch.
%   Calls the SAME targetAccel the integrator calls -- one source of truth.
tg = [0:0.1:stageEnd, stageEnd - H];
tg = tg(tg < stageEnd);
if ITGT == 2 && stageEnd > 200
    tg = tg(tg >= 120);                 % second stage only
end
v = zeros(size(tg));
for k = 1:numel(tg)
    v(k) = targetAccel(tg(k), ITGT, 1, TF, 0.0, XNTAV);
end
pk = max(v)/32.2;
end


% =============================================================================
function K = structuralChecks(R, XNP, VC, VM, XNCMAX, H, samp, ftPerM, XNTAV)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never to be quoted
%   as evidence that this port reproduces Zarchan -- they are the weaker claim
%   that it agrees with ITSELF, which is what catches wiring faults.
%
%   .names and .oks are what benchSuite puts on the board.  A structural check
%   stored under any other field name is a check nobody can see, which is the
%   same as a check that was never run.
K = struct();
names = {};  oks = [];  detail = {};

TFs = 20.0;                        % short runs: these are WIRING tests
nT9 = 9*32.2;
pip = 2.0*3280.0;

% 1  DELVD = |XNC|, so the divert record cannot go backwards.  A sign slip on
%    DELVD, or DELVD wired to XNC instead of abs(XNC), shows up here and
%    nowhere else cheap.
K.delv_min_step = min(diff(R.delv_m));
names{end+1}  = 'DELV non-decreasing (DELVD = |XNC|)';
oks(end+1)    = K.delv_min_step >= -1e-15;
detail{end+1} = sprintf('min consecutive change %+.3e m/s', K.delv_min_step);

% 2  VC CANCELS.  XLAMD divides by VC*TGO^2 and XNC multiplies by VC again, so
%    closing velocity drops out of the command entirely.  Two runs at VC and
%    3*VC must agree to round-off.  If a future edit puts a REAL VC dependence
%    in, this row is the only thing that will notice -- a 3x change against a
%    1e-8 band discriminates by eight orders of magnitude.
A  = runEngagement(1, 0, 1, TFs, nT9, XNTAV, XNCMAX, XNP, VC,   VM, 0.0, H, samp, ftPerM);
Bv = runEngagement(1, 0, 1, TFs, nT9, XNTAV, XNCMAX, XNP, 3*VC, VM, 0.0, H, samp, ftPerM);
K.vc_resid = maxRelPeak(A.xnc_g, Bv.xnc_g);
names{end+1}  = 'VC cancels out of the command (VC vs 3*VC)';
oks(end+1)    = K.vc_resid <= 1e-8;
detail{end+1} = sprintf('max |dXNC|/peak = %.3e between VC = %g and %g ft/s', ...
                        K.vc_resid, VC, 3*VC);

% 3  PIP-ONLY closed form.  With nT = 0 and N' = 3 the exact solution is
%    nc(t) = 3*PIP*tgo/tF^3 -- LINEAR in tgo with a known slope.  Checked as a
%    slope so it never has to divide by a vanishing nc near intercept; samples
%    with tgo < 1 s are excluded for the round-off reason in the header.
Cp  = runEngagement(1, 0, 1, TFs, 0.0, XNTAV, XNCMAX, XNP, VC, VM, pip, H, samp, ftPerM);
sel = Cp.tgo >= 1.0;
if any(sel)
    slope = 32.2*Cp.xnc_g(sel)./Cp.tgo(sel);
    want  = XNP*pip/TFs^3;
    K.pip_slope_err = max(abs(slope/want - 1));
else
    K.pip_slope_err = Inf;  want = NaN;
end
names{end+1}  = 'PIP-only: nc = 3 PIP tgo / tF^3 (exact, N''=3)';
oks(end+1)    = K.pip_slope_err <= 1e-3;
detail{end+1} = sprintf('max slope error %.3e over %d samples (want %.5f ft/s^3)', ...
                        K.pip_slope_err, sum(sel), want);

% 4  DELETE / INJECT EXPERIMENT on the acceleration limit.  Rerun case A with
%    XNCMAX pulled down to 30 ft/s^2, well under its ~141 ft/s^2 peak: the
%    command must sit exactly ON the new limit AND the divert must change.
%    Without the second half, a limiter that clipped the recorded command but
%    was never integrated would still pass.
lim = 30.0;
D = runEngagement(1, 0, 1, TFs, nT9, XNTAV, lim, XNP, VC, VM, 0.0, H, samp, ftPerM);
K.clamp_max = 32.2*max(abs(D.xnc_g));
K.clamp_dv  = abs(D.dv_m/A.dv_m - 1);
names{end+1}  = 'XNCMAX limiter binds AND changes the divert';
oks(end+1)    = K.clamp_max <= lim*(1 + 1e-12) && K.clamp_max >= lim*0.999 ...
                && K.clamp_dv >= 0.01;
detail{end+1} = sprintf(['peak %.5f ft/s^2 against limit %.1f; divert moves ' ...
                         '%.1f%% (%.2f -> %.2f m/s)'], K.clamp_max, lim, ...
                        100*K.clamp_dv, A.dv_m, D.dv_m);

% 5-6  THE CHAPTER'S DELIVERABLE FORMULA -- AND WHAT THE SENTENCE ACTUALLY
%    SAYS.  zarchan_decoded.txt:13303-13305 verbatim (13304 and 13305 are ONE
%    two-line display; the multiplication sign arrived as a raw 0x04 byte and
%    the fraction bar did not survive extraction at all):
%
%      :13303  "Here it can be observed that for a given PIP error, more
%               divert will be required for shorter KKV action times tF. Thus
%               the total divert required, under worse case conditions, is
%               simply given by"
%      :13304  "DVTOTAPN = 1.5<x>PIP tF"
%      :13305  "+ 0.25<x>nTMAXtF"
%
%    so the formula is  DVTOTAPN = 1.5 PIP / tF + 0.25 nTMAX tF.  The DIVISION
%    is invisible in both source files and is recovered three independent
%    ways, all arithmetic: the prose at :13303 ("more divert ... for SHORTER
%    action times"); the worked example at :13561-13563, 1.5*89000/(223-180) =
%    3104.65 -> the printed "3.1 km/s"; and Table 34.1 at :13568-13575, where
%    1.5*20400/(170-110) = 510 reproduces the printed DVFormula column to the
%    digit in all seven rows.
%
%    *** THE ROW THAT USED TO LIVE HERE ASSERTED SOMETHING THE BOOK DOES NOT
%    CLAIM, ON A PREMISE THAT IS MEASURABLY FALSE. ***  It was
%    "DVTOTAPN = DVPIP + DVMVR (superposition)", scored |E/(A+Cp) - 1| <= 1e-6,
%    justified as "the system is linear and BOTH contributions are
%    non-negative throughout, so int|sum| is the sum of the two integrals,
%    exactly".  Both halves of that are wrong:
%
%      a  :13303 says the sum is the total UNDER WORSE CASE CONDITIONS.  That
%         is an UPPER BOUND, attained when the two contributions drive the
%         same way -- not an unconditional equality.  Divert is int|nc| dt, so
%         the general statement is the triangle inequality,
%         int|a + b| <= int|a| + int|b|.
%      b  The contributions are NOT both non-negative throughout.  The
%         PIP-only command goes negative on exactly one sample: the last one,
%         at TGO = 1e-5, where the PN gain 3/TGO^2 = 3e10 multiplies a
%         zero-effort miss that is by then pure round-off (see the header).
%         Its analytic value there is 3*PIP*tgo/tF^3 = 2.5e-5 ft/s^2; the
%         listing computes -18.0 ft/s^2.  The sign is arbitrary.
%
%    The measured consequence, at the production limit: the old row read
%    4.6981e-06, i.e. it missed its 1e-6 band by a factor of 4.7 -- and it
%    missed it in the direction the triangle inequality requires,
%    E = 592.103985 < 592.106767 = A + Cp, a deficit of 2.78e-03 m/s that
%    appears ENTIRELY between t = 19.9 s and t = 20.0 s (the gap is -2.1e-11
%    at every earlier sample).  Raising XNCMAX until the limiter provably
%    cannot bind does NOT fix it -- the residual gets WORSE, 6.8040e-06 --
%    which is what rules saturation out as the cause and leaves the sign.
%
%    So the identity is split into the two separate claims it was conflating.
%    Row 5 is the LINEARITY that makes a two-term decomposition legitimate at
%    all, and it is exact.  Row 6 is the divert statement in the form :13303
%    puts it: equality where the two commands are co-signed, and the bound
%    everywhere.

% 5  COMMANDS SUPERPOSE, EXACTLY.  Y and YD are linear in (PIP, nT) and
%    XNC = N' VC XLAMD + 0.5 APN N' XNT is linear in (Y, YD, XNT), so with the
%    limiter out of the loop the both-inputs run must equal the sum of the two
%    single-input runs to round-off, over the WHOLE record -- no window, no
%    exclusion.  XNCfree is checked to be provably non-binding in the same
%    row, otherwise a run that clipped every sample flat would pass this by
%    agreeing with itself.
XNCfree = 1e12;
A2 = runEngagement(1, 0, 1, TFs, nT9, XNTAV, XNCfree, XNP, VC, VM, 0.0, H, samp, ftPerM);
C2 = runEngagement(1, 0, 1, TFs, 0.0, XNTAV, XNCfree, XNP, VC, VM, pip, H, samp, ftPerM);
E2 = runEngagement(1, 0, 1, TFs, nT9, XNTAV, XNCfree, XNP, VC, VM, pip, H, samp, ftPerM);
K.cmd_super_err = maxRelPeak(E2.xnc_g, A2.xnc_g + C2.xnc_g);
K.cmd_peak_ft   = 32.2*max(abs(E2.xnc_g));
names{end+1}  = 'commands superpose: nc(PIP+MVR) = nc(PIP) + nc(MVR), limiter out of the loop';
oks(end+1)    = K.cmd_super_err <= 1e-9 && K.cmd_peak_ft <= 0.5*XNCfree;
% Requirement and measurement, not a verdict: "so the limiter provably never
% binds" was a conclusion inside the sprintf and would have printed unchanged
% beside a peak that reached the limit.
detail{end+1} = sprintf(['max |dnc|/peak = %.3e over all %d samples ' ...
                         '(must be <= 1e-9); peak %.4g ft/s^2 against the ' ...
                         '%.3g limit, which the row requires to stay under ' ...
                         'half'], K.cmd_super_err, numel(E2.t), ...
                        K.cmd_peak_ft, XNCfree);

% 6  THE DIVERT STATEMENT.  Two conjuncts, from :13303.
%    (i)  EQUALITY on the leading run of samples where the two component
%         commands share a sign.  That window is defined from the SIGNS
%         THEMSELVES, not from a time cut, so there is no knob to turn; the
%         row prints how long it is and where it ends.  It is required to
%         cover at least 90% of the record -- a guard against the window
%         collapsing to nothing, which would make the equality unfalsifiable,
%         NOT a tolerance on the physics.
%    (ii) THE BOUND over the full flight, including the contaminated last
%         sample: E <= A + Cp.  This one is a theorem (clipping is a
%         contraction and the triangle inequality survives it), so a violation
%         cannot be round-off -- it can only be a wiring fault.
E = runEngagement(1, 0, 1, TFs, nT9, XNTAV, XNCMAX, XNP, VC, VM, pip, H, samp, ftPerM);
sameLen = isequal(numel(A.xnc_g), numel(Cp.xnc_g), numel(E.xnc_g));
opp = (sign(A.xnc_g).*sign(Cp.xnc_g) < 0);
kco = find(opp, 1, 'first') - 1;
if isempty(kco), kco = numel(opp); end
K.cosign_n   = kco;
K.cosign_all = numel(opp);
if kco >= 1 && kco < numel(opp)
    K.cosign_tgo = E.tgo(kco+1);        % TGO at the first disagreement
else
    K.cosign_tgo = NaN;
end
if sameLen && kco >= 1
    K.super_err = abs(E.delv_m(kco)/(A.delv_m(kco) + Cp.delv_m(kco)) - 1);
else
    K.super_err = Inf;
end
K.super_full = E.dv_m/(A.dv_m + Cp.dv_m) - 1;      % SIGNED.  Must be <= 0.
kshow = max(kco, 1);
names{end+1}  = 'DVTOT = DVPIP + DVMVR while co-signed, <= it overall (zarchan.txt:13303-13305)';
oks(end+1)    = sameLen && kco >= 0.9*numel(opp) && K.super_err <= 1e-9 ...
                && K.super_full <= 1e-12;
detail{end+1} = sprintf(['co-signed on %d/%d samples (first disagreement at ' ...
                         'TGO = %.5g s): %.6f vs %.6f + %.6f m/s, error ' ...
                         '%.3e.  Full flight %+.4e, must be <= 0 because ' ...
                         ':13303 says "under worse case conditions"'], ...
                        kco, numel(opp), K.cosign_tgo, E.delv_m(kshow), ...
                        Cp.delv_m(kshow), A.delv_m(kshow), ...
                        K.super_err, K.super_full);

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


function r = maxRelPeak(a, b)
%MAXRELPEAK  Max absolute difference normalised by the curve's own PEAK.  Not
%   pointwise relative: these curves pass through zero, and a pointwise
%   relative error there divides a small number by a smaller one and
%   manufactures a failure out of nothing.
den = max(abs(b));
if ~(den > 0)
    r = Inf;                     % refuse to report "perfect" against a zero curve
    return
end
r = max(abs(a - b))/den;
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER must match the adders exactly.  Schema is
%   the one mgs.zarchan.benchSuite consumes; a near-miss ERRORS there rather
%   than being accommodated, so every row is built through one constructor.
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

function B = addInfo(B, what, bookStr, oursStr, source, why)
%ADDINFO  Reported, never scored.  skipped = true keeps it out of the pass
%   count in BOTH directions: an INFO row can neither pass nor fail.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'info';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'info';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = '-';  B(i).detail = '';
end


% =============================================================================
function s = render(R, ev)
L = {};
p = R.p;
L{end+1} = 'Zarchan Listing 34.1 -- ONE-DIMENSIONAL ENGAGEMENT, LINEARIZED GEOMETRY';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2.  DELV is m/s via 3.28. ***';
L{end+1} = sprintf('   ITGT = %d (%s)   IOPTION = %d (%s)   %s   N'' = %g', ...
    p.ITGT, tgtName(p.ITGT), p.IOPTION, manName(p.IOPTION, p.ITGT), ...
    lawName(p.APN), p.XNP);
L{end+1} = sprintf(['   TF = %g s   nTMAX = %g ft/s^2 (%.3g g)   XNCMAX = %g ' ...
    'ft/s^2 (%.3g g)   PIP = %g km   H = %g s'], p.TF, p.XNTMAX, ...
    p.XNTMAX/32.2, p.XNCMAX, p.XNCMAX/32.2, p.PRED_KM, p.H);
L{end+1} = '';
L{end+1} = sprintf('   miss Y = %.4f ft      divert DELV = %.2f m/s      mean nT = %.4f g', ...
    R.Y_ft, R.dv_m, R.xnt_mean_g);
L{end+1} = '';

L{end+1} = 'RECORD  (the listing''s Array*, every samp seconds)';
L{end+1} = sprintf('%9s %9s %10s %10s %12s', 't [s]', 'tgo [s]', 'nT [g]', ...
    'nc [g]', 'DELV [m/s]');
stride = max(1, round(numel(R.t)/12));
rows = unique([1:stride:numel(R.t), numel(R.t)]);
for r = rows
    L{end+1} = sprintf('%9.2f %9.3f %10.4f %10.4f %12.3f', ...
        R.t(r), R.tgo(r), R.xnt_g(r), R.xnc_g(r), R.delv_m(r)); %#ok<AGROW>
end
L{end+1} = ['   (the final row sits at TGO = 1e-5, where nc is ROUND-OFF -- ' ...
            'excluded from every scored acceleration; see the header)'];
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
    L{end+1} = 'BENCHMARKS  (book values grepped from zarchan.txt; line numbers in .source)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
            case 'info'
                L{end+1} = sprintf('   INFO %-50s %s', b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case 'shape'
                L{end+1} = sprintf('   %-4s %-50s %s', vstr(b.ok), b.what, ...
                    b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf(['   %-4s %-50s book %s %s / ours %s %s ' ...
                    '(%+.3f%%, band %.3g%%)'], vstr(b.ok), b.what, ...
                    b.book_str, b.units, b.ours_str, b.units, b.pct, ...
                    100*b.tol); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d info -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: no lag, no drag, no gravity, no LOS floor, constant Vc, and a';
L{end+1} = '   maneuver that is a function of TIME.  Divert here is int|nc|dt on that';
L{end+1} = '   model -- a SIZING number, not a prediction for an aircraft.';
s = strjoin(L, newline);
end

function s = tgtName(i)
if i == 1
    s = 'one-stage IRBM, 180 s';
else
    s = 'two-stage ICBM, 240 s';
end
end

function s = manName(io, itgt)
if io == 0
    if itgt == 1
        s = 'parabolic approximation';
    else
        s = 'constant XNTAV';
    end
else
    s = 'actual thrust/weight';
end
end

function s = lawName(a)
if a == 0
    s = 'PN (APN = 0)';
else
    s = 'APN (APN = 1)';
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
%DRAWCURVES  Base MATLAB graphics only.  Axis labels are the listing's own.
figure;
subplot(2,1,1);
hold on; grid on;
plot(R.t, R.xnc_g, '-');
plot(R.t, R.xnt_g, '--');
xlabel('Missile Flight Time (s)');
ylabel('KKV Acceleration (g)');
legend('KKV command nc', 'apparent target nT', 'Location', 'best');
title(sprintf('Listing 34.1:  ITGT %d, IOPTION %d, APN %d', ...
      R.p.ITGT, R.p.IOPTION, R.p.APN));
hold off;

subplot(2,1,2);
grid on;
plot(R.t, R.delv_m, '-');
xlabel('Missile Flight Time (s)');
ylabel('KKV Lateral Divert (m/s)');
end
