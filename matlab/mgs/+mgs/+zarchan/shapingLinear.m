function R = shapingLinear(opts)
%MGS.ZARCHAN.SHAPINGLINEAR  Zarchan Listing 24.1 -- LINEAR ENGAGEMENT
%   SIMULATION FOR GUIDANCE LAW COMPARISON (trajectory shaping vs PN).
%
%   R = mgs.zarchan.shapingLinear()
%   R = mgs.zarchan.shapingLinear('benchmark',false,'XLAMFDEG',-45)
%   R = mgs.zarchan.shapingLinear('assert',false,'render',false)   % in a suite
%
%   Line-by-line port of Listing 24.1, Ch. 24, pp. 577-579.  Sources in repo:
%       matlab/zarchan/_source/zarchan.txt:8133  the listing
%       matlab/zarchan/_source/zarchan.txt:8112  the prose that sets it up
%       matlab/zarchan/_source/zarchan.txt:8285  the general closed form
%       matlab/zarchan/_source/zarchan.txt:8312  the three published checks
%       matlab/zarchan/_listings/listing_24_01.m the transcription ported here
%
%   The DESIGN side of mgs.zarchan.shapingNonlinear (Listing 24.2): 24.1 is the
%   linearised zero-lag homing loop of Fig. 24.2 where the law was first
%   checked, 24.2 drops the same law into the nonlinear engagement of Listing
%   2.1.  24.1 is the one where the book publishes CLOSED FORMS, which is why
%   this file can score exact agreement and its sibling cannot.
%
% ===========================================================================
%   UNITS ARE FEET: ft, ft/s, ft/s^2, g = 32.2, degrees via the book's 57.3
%   (NOT 180/pi).  Nothing is converted.  Listing 30.1 (mgs.zarchan.shaping2d)
%   is this package's metric exception; this file is not it.
% ===========================================================================
%
%   THE LAW, with PN = 0 (the flag is defined at zarchan.txt:8114):
%       nc = 4*Vc*lamdot + nT + 2*Vc*(lam - lamF)/tgo
%   augmented PN at an effective navigation ratio of 4, plus a term driving
%   the LOS angle to a designer-chosen final value (zarchan.txt:8111).  With
%   PN = 1 it degenerates to nc = N'*Vc*lamdot.
%
%   WHY IT MATTERS HERE.  mgs.zarchan.additiveLaw runs a shaping term in this
%   repo, and that term was found in Aug 2026 to have written the CONSTANT
%   setpoint as the CURRENT angle -- an attractor with the wrong sign.  The
%   listing is the antidote: constant lamF, both terms scaled by the CLOSING
%   velocity, clamp applied AFTER the terms are summed.
%
%   *** WHAT IT IS NOT. ***  Linearised, planar, ZERO LAG, constant Vc, tF an
%   INPUT rather than an estimate; no gravity, drag or thrust.  Every failure
%   mode this project actually has (a gamma loop of measured tracking gain
%   0.063, an nz clamp, a saturating pace loop) is absent by construction.
%   The law's definition and a clean-loop baseline, not a prediction for an
%   aircraft.
%
% ---------------------------------------------------------------------------
%   TRANSCRIPTION.  listing_24_01.m does not parse; its header reports ONE
%   unresolved variable-to-variable comparison.  Four repairs, none a
%   judgement call:
%     1  "while T,=(TF-.0001)" -> "<=".  The OCR renders "<" as ",".  T only
%        increases, so ">=" never terminates and "==" never fires.
%     2  "while STEP ,=1"      -> "<=".  The body sets STEP = 2 to leave.
%     3  "if XNC.XNCLIM"       -> "if XNC > XNCLIM".  *** THE UNRESOLVED ONE.
%        "." is this book's OCR of ">".  Direction forced by the next line,
%        "if XNC < -XNCLIM, XNC = -XNCLIM": the pair is a symmetric limiter,
%        and "<" here would raise every in-limit command TO the limit, making
%        the nominal case a full-scale square wave instead of the +25 g/-30 g
%        ramp the book prints at :8132.
%     4  "if S.=.09999"        -> ">=".  S is reset on every record, so "<="
%        would record forever.
%   Reproduced deliberately: TGO = TF - T + 1e-5 (the law divides by tgo), and
%   the recorded XNC/XLAM come from the SECOND derivative pass, i.e. the
%   EULER-PREDICTED state, while the recorded Y is the CORRECTED one -- the
%   listing's own mismatch, O(H^2) except at the last sample.  Dropped: the
%   plotting block and "save datfil.txt output /ascii" (a typo for -ascii).
%   Nothing is written to disk and no figure is drawn.
%
%   TWO SIGNS THE OCR ATE, RESOLVED FROM THE LISTING, NOT GUESSED.  Every
%   minus in this chapter's prose became a "2" ("220 deg", "230 g"), so the
%   printed closed forms lost theirs.  Book "ydot(0) = VM*HE": the listing's
%   line 1 is YD = -VM*HEDEG/57.3, so ydot(0) = -VM*HE.  Book "ydot(tF) =
%   Vc*lam(tF)": the listing defines XLAM = Y/(VC*TGO), so y = Vc*tgo*lam and
%   ydot(tF) = -Vc*lam(tF).  Both are confirmed by the SIGN the book reports
%   at :8132 (+25 g opening for HE = -20 with lamF = -30); flip either and the
%   opening command is 0 g, not 25 g.
%
% ---------------------------------------------------------------------------
%   *** THE FINAL RECORDED SAMPLE IS AN ARTIFACT.  IT IS NOT SCORED. ***
%   At the last record T = tF, so the true time-to-go is ZERO while the
%   listing hands the law TGO = 1e-5.  XLAM = Y/(Vc*TGO) and XLAMD =
%   (Y+YD*TGO)/(Vc*TGO^2) are then 0/0 forms on a predictor state carrying a
%   local error of about (H^2/2)*ydotdot.  The shaping law turns a state error
%   dY into a command error 6*dY/tgo^2, so 1e-4 ft at tgo = 1e-5 becomes an
%   order-1e9 ft/s^2 command -- big enough that XNCLIM, inert for the whole
%   flight, fires on that one step; the LOS angle collapses toward 0 there for
%   the same reason.  The book's own axis([0 10 -40 30]) and axis([0 10 -30
%   10]) clip it out of Figs. 24.4 and 24.5, which is why the printed figures
%   look clean.  So every scored quantity is read inside t <= tF - tguard
%   (default 0.1 s, the listing's own sample interval).  Sizing: the command
%   error stays under 0.5% of peak while tguard >= H*sqrt(3*ydotdot/(0.005*
%   peak)) = 0.017 s at the book's values, so 0.1 s carries ~6x in tguard,
%   ~36x in the error.  The recorded Y is NOT affected (its corrector uses YD,
%   not YDD), so the miss row still reads the true last sample.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Every book value was grepped out of zarchan.txt and the line
%   number is in the row's .source.  Nothing was read off a figure -- the
%   extraction turned every figure into an image link -- but prose ABOUT a
%   figure is greppable and is scored as an ordering.  Bands:
%     'analytic'  max(0.5%, 4*H) of peak, against a printed CLOSED FORM where
%                 the book claims "exact agreement": discretization only.
%     'text'      10%, stated plainly but read off the book's own plot and
%                 sampled tguard before tF (the LOS angle still moves 0.75 deg
%                 of the -30 goal, 0.28 of PN's 14, over that last 0.1 s).
%     'figure'    15%, a one-significant-figure eyeball reading quoted in
%                 prose ("25 g", "approximately -30 g near the end").
%   The book's own three checks (:8312-8326) are the strongest rows in the
%   chapter -- it publishes the CONFIGURATION and the FORMULA and asserts
%   exact agreement -- and all three are special cases of the general form at
%   :8285, which is what this file evaluates.  NOT SCORED (INFO): the Fig.
%   24.7 levels (book 20 / -20 g, PN "nearly 20 g"), because that case is the
%   SUPERPOSITION of the nT and lamF rows, whose formula is already scored
%   exactly.  Reported with numbers, not adjudicated.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED FROM THE CLOSED FORMS, NOT MEASURED
%   -- this file has never been executed; MATLAB is the caller's. ***  In g
%   and deg, at the first record (t = 0.1 s) and the last in-window one (9.9):
%
%     case                     nc(0.1)  nc(9.9)  lam(9.9)  y(10)
%     A  shape HE-20 lamF-30    +25.43   -31.93   -29.25    ~0
%     B  PN    HE-20            + 9.66   + 0.10   + 7.50    ~0
%     C  shape nT 6g  lamF-30   +18.62   -19.63   -29.40    ~0
%     D  PN    nT 6g            + 0.18   +17.82   +13.56    ~0
%     E  shape nT 6g  lamF 0    + 6.000  + 6.000  +  0.00    0
%     F  shape HE-20  lamF 0    +12.81   - 6.31   +  0.15   ~0
%     G  shape        lamF-30   +12.62   -25.63   -29.40    ~0
%
%   Expected residuals against the book: +1.7% (25 g), +6.5% (-30 g),
%   -2.5% (-30 deg), -6.3% (8 deg), -3.1% (14 deg); the three exact-agreement
%   rows should land near 0.01-0.05% of peak against a 0.5% band.  A residual
%   near a band EDGE is not a pass to be accepted -- the gap between 0.05%
%   and 0.5% is the entire safety margin.  Do not widen a band.
%
%   OUTPUTS.  R.case.A..G (each .t .y .ncg .lamdeg .tgo .clamped .cfg .title
%   .i1 .i2, the first and last IN-WINDOW record), R.run (the caller's own
%   configuration flown), R.bench, R.selfcheck, R.pass, R.units, R.p,
%   R.summary, and R.verdict -- ALWAYS FORMED; 'assert' picks the reaction and
%   nothing else: true errors, false WARNS and still returns R complete, which
%   is how benchSuite can see which side of a comparison moved.
%
%   See also MGS.ZARCHAN.SHAPINGNONLINEAR, MGS.ZARCHAN.SHAPING2D,
%   MGS.ZARCHAN.ADDITIVELAW, MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 24.1 line 1, verbatim (zarchan.txt:8133).
    opts.XNT      (1,1) double {mustBeFinite} = 0        % ft/s^2 target accel
    opts.HEDEG    (1,1) double {mustBeFinite} = -20      % deg heading error
    opts.XNCLIM   (1,1) double {mustBeFinite, mustBePositive} = 999999
    % 0 = trajectory shaping, 1 = PN (zarchan.txt:8114).  Kept numeric and
    % kept the book's name; it is not a logical in the book.
    opts.PN       (1,1) double {mustBeMember(opts.PN,[0 1])} = 0
    opts.XLAMFDEG (1,1) double {mustBeFinite} = -30      % deg, desired final LOS
    opts.VC       (1,1) double {mustBeFinite, mustBePositive} = 4000   % ft/s
    opts.VM       (1,1) double {mustBeFinite, mustBePositive} = 3000   % ft/s
    opts.TF       (1,1) double {mustBeFinite, mustBePositive} = 10     % s
    opts.XNP      (1,1) double {mustBeFinite, mustBePositive} = 3      % N'
    opts.H        (1,1) double {mustBeFinite, mustBePositive} = 0.001  % s
    opts.samp     (1,1) double {mustBeFinite, mustBePositive} = 0.1    % s
    % How far before tF the scored window ends; see THE FINAL RECORDED SAMPLE.
    opts.tguard   (1,1) double {mustBeFinite, mustBePositive} = 0.1    % s
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so an `if nargin == 0` mode here would never fire.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.assert    (1,1) logical = true
end

DEG  = 57.3;      % the book's, not 180/pi
GRAV = 32.2;      % ft/s^2

if opts.samp < opts.H
    error('mgs:zarchan:shapingLinear:samp', ...
        ['samp = %g s is smaller than the step H = %g s, so the record would ' ...
         'claim a finer time axis than the run has.'], opts.samp, opts.H);
end
if opts.tguard < 10*opts.H
    error('mgs:zarchan:shapingLinear:guard', ...
        ['tguard = %g s is under 10*H = %g s.  Inside that the recorded ' ...
         'command is a predictor error divided by tgo^2, so a closed-form ' ...
         'comparison there measures the endgame, not the law.'], ...
        opts.tguard, 10*opts.H);
end
if opts.TF <= 4*opts.tguard
    error('mgs:zarchan:shapingLinear:short', ...
        'TF = %g s leaves no scored window at tguard = %g s.', opts.TF, opts.tguard);
end

base = struct('XNT',opts.XNT, 'HEDEG',opts.HEDEG, 'XNCLIM',opts.XNCLIM, ...
    'PN',opts.PN, 'XLAMFDEG',opts.XLAMFDEG, 'VC',opts.VC, 'VM',opts.VM, ...
    'TF',opts.TF, 'XNP',opts.XNP, 'H',opts.H, 'samp',opts.samp, ...
    'tguard',opts.tguard);

[CS, offbook] = bookCases(base, GRAV);
CASE = struct();
for k = 1:numel(CS)
    CASE.(CS(k).key) = flyOne(CS(k).cfg, DEG, GRAV, CS(k).title);
end

R = struct();
R.case  = CASE;
R.run   = flyOne(base, DEG, GRAV, 'caller configuration');
R.units = 'ft, ft/s, ft/s^2; g = 32.2 ft/s^2; deg via the book''s 57.3';
R.p     = base;
R.p.units   = R.units;
R.p.offbook = offbook;
R.selfcheck = structuralChecks(CASE, base, DEG, GRAV);

if opts.benchmark
    R.bench = benchmarks(CASE, base, offbook, GRAV);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine".
    R.pass  = false;
end

R.summary = render(R, ev, offbook);
if opts.render, fprintf('%s\n', R.summary); end

msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated: %s.  An ' ...
        'unvalidated port must not be handed back silently.'], offbook);
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port ' ...
        'disagrees with ITSELF, a wiring fault rather than a tolerance ' ...
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
        error('mgs:zarchan:shapingLinear:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:shapingLinear:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function E = flyOne(C, DEG, GRAV, ttl)
%FLYONE  Listing 24.1, zarchan.txt:8133, one engagement.
%   EVERY constant arrives as an ARGUMENT: MATLAB local functions cannot see
%   the main function's workspace, and a previous wave in this package read a
%   constant from the caller and threw on the first line of its first run.

XNT = C.XNT;  XNCLIM = C.XNCLIM;  PN = C.PN;  VC = C.VC;
TF  = C.TF;   XNP = C.XNP;  H = C.H;  samp = C.samp;

XLAMF = C.XLAMFDEG/DEG;
Y  = 0.0;
YD = -C.VM*C.HEDEG/DEG;            % the listing's own line 1
T  = 0.0;
S  = 0.0;
% Initialised before the loop so the FLAG==1 branch cannot read a leftover.
YDD = 0.0;  XNC = 0.0;  XLAM = 0.0;  TGO = TF;  clipped = false;

nmax = ceil(TF/samp) + 4;
tA = zeros(nmax,1);  yA = tA;  gA = tA;  lA = tA;  goA = tA;
cA = false(nmax,1);
n  = 0;  nClamp = 0;

while T <= (TF - 0.0001)
    YOLD = Y;  YDOLD = YD;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  Pass 1 evaluates at the current
    % state; pass 2 takes a full Euler step, advances T by H and re-evaluates
    % THERE.  T advances INSIDE this loop, exactly once -- move that line out
    % and the scheme silently degrades to first order while still running.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            Y  = Y + H*YD;
            YD = YD + H*YDD;
            T  = T + H;
        end
        TGO   = TF - T + 0.00001;              % never exactly 0; see header
        XLAM  = Y/(VC*TGO);
        XLAMD = (Y + YD*TGO)/(VC*TGO*TGO);
        if PN == 1
            XNC = XNP*VC*XLAMD;
        else
            XNC = 4.*VC*XLAMD + XNT + 2.*VC*(XLAM - XLAMF)/TGO;
        end
        raw = XNC;
        if XNC >  XNCLIM, XNC =  XNCLIM; end   % <<< the resolved comparison
        if XNC < -XNCLIM, XNC = -XNCLIM; end
        clipped = (XNC ~= raw);
        if clipped, nClamp = nClamp + 1; end
        YDD  = XNT - XNC;
        FLAG = 1;
    end
    % The listing writes "FLAG = 0" here too.  Dropped: the outer loop
    % re-initialises FLAG, so the assignment cannot be read.  Noted rather
    % than re-added, so nobody thinks a state machine was lost.

    Y  = 0.5*(YOLD  + Y  + H*YD);
    YD = 0.5*(YDOLD + YD + H*YDD);

    % The listing tests "S >= .09999" against samp = 0.1, H = 0.001, i.e. half
    % a step below the interval.  Written that way so a caller changing samp
    % gets the interval asked for rather than the book's literal.
    S = S + H;
    if S >= samp - 0.5*H
        S = 0.0;
        n = n + 1;
        tA(n) = T;  yA(n) = Y;  goA(n) = TGO;
        gA(n) = XNC/GRAV;                      % the listing's XNCG
        lA(n) = XLAM*DEG;                      % the listing's XLAMDEG
        cA(n) = clipped;
    end
end

E = struct('title',ttl, 'cfg',C, 't',tA(1:n), 'y',yA(1:n), 'ncg',gA(1:n), ...
    'lamdeg',lA(1:n), 'tgo',goA(1:n), 'clamped',cA(1:n), 'n_clamp',nClamp, ...
    'i1',1, 'i2',[]);

w = E.t <= (TF - C.tguard + 1e-9);             % the scored window; see header
E.i2 = find(w, 1, 'last');
if isempty(E.i2) || E.i2 < 2
    error('mgs:zarchan:shapingLinear:window', ...
        ['the run recorded %d sample(s), %d of them inside t <= tF - tguard. ' ...
         'At least two are needed for a curve comparison to mean anything.'], ...
        n, sum(w));
end
end


% =============================================================================
function ncg = tsClosedForm(t, TF, yd0, XNT, ydF, GRAV)
%TSCLOSEDFORM  The book's GENERAL closed form, zarchan.txt:8285, with y(0) = 0
%   (which Listing 24.1 always has):
%     nc(t) = [tF*ydot(0)*(6tgo-2tF) + tF^3*nT(0) + tF*ydot(tF)*(6tgo-4tF)]/tF^3
%   The three published special cases (:8291 nT, :8305 HE, :8310 lamF) are
%   this with the other terms zeroed, so one evaluation scores all three
%   without three transcriptions to keep in step.  Returns g.
tgo = TF - t;
ncg = (TF*yd0.*(6*tgo - 2*TF) + TF^3*XNT + TF*ydF.*(6*tgo - 4*TF))/TF^3/GRAV;
end

function ncg = pnClosedForm(t, TF, yd0, XNT, GRAV)
%PNCLOSEDFORM  Zero-lag PN at N' = 3.  *** DERIVED HERE, not a book value ***
%   -- which is why it is used only in the STRUCTURAL block:
%     nc(t) = 3*ydot(0)/tF*(1 - t/tF) + 3*nT*t/tF
%   Both halves satisfy nc = 3*(y + ydot*tgo)/tgo^2 term for term with
%   y(0) = 0, so this is an exact solution of the transcribed ODEs, not an
%   approximation.  It is what catches the PN branch being mis-wired while
%   every shaping row still passes.
ncg = (3*yd0/TF*(1 - t/TF) + 3*XNT*t/TF)/GRAV;
end


% =============================================================================
function [CS, offbook] = bookCases(base, GRAV)
%BOOKCASES  The seven configurations the book actually publishes.  Only XNT /
%   HEDEG / PN / XLAMFDEG are overridden, so changing a SHARED parameter takes
%   every case off-book and the rows SKIP rather than being scored against
%   numbers that no longer describe the run.  (XLAMFDEG is irrelevant when
%   PN = 1, so B and D are on-book whatever it is.)
nT6 = 6*GRAV;                                  % 193.2 ft/s^2, the book's 6 g
CS = struct('key',{},'title',{},'cfg',{});
CS = addCase(CS, base, 'A', 'shaping, HE -20 deg, lamF -30 deg (Figs 24.3-24.5)',   0, -20, 0, -30);
CS = addCase(CS, base, 'B', 'PN, HE -20 deg                    (Figs 24.3-24.5)',   0, -20, 1, -30);
CS = addCase(CS, base, 'C', 'shaping, 6 g target, lamF -30 deg (Figs 24.6-24.8)', nT6,   0, 0, -30);
CS = addCase(CS, base, 'D', 'PN, 6 g target                    (Figs 24.6-24.8)', nT6,   0, 1, -30);
CS = addCase(CS, base, 'E', 'shaping, 6 g target, lamF 0       (Fig 24.13)',      nT6,   0, 0,   0);
CS = addCase(CS, base, 'F', 'shaping, HE -20 deg, lamF 0       (Fig 24.14)',        0, -20, 0,   0);
CS = addCase(CS, base, 'G', 'shaping, lamF -30 deg only        (Fig 24.15)',        0,   0, 0, -30);

b = struct('VC',4000, 'VM',3000, 'TF',10, 'XNP',3, 'XNCLIM',999999, ...
           'H',0.001, 'samp',0.1, 'tguard',0.1);
f = fieldnames(b);
bad = {};
for i = 1:numel(f)
    if base.(f{i}) ~= b.(f{i})
        bad{end+1} = sprintf('%s = %g (book %g)', f{i}, base.(f{i}), b.(f{i})); %#ok<AGROW>
    end
end
if isempty(bad)
    offbook = '';
else
    offbook = sprintf(['off the listing''s own case: %s.  The book''s numbers ' ...
        'describe VC=4000, VM=3000, TF=10, N''=3, XNCLIM=999999, H=0.001, ' ...
        'samp=0.1'], strjoin(bad, ', '));
end
end

function CS = addCase(CS, base, key, ttl, XNT, HEDEG, PN, XLAMFDEG)
% Field-by-field, not struct(): struct() with a struct-valued field is one of
% the places MATLAB's cell-expansion rule surprises people.
c = base;
c.XNT = XNT;  c.HEDEG = HEDEG;  c.PN = PN;  c.XLAMFDEG = XLAMFDEG;
i = numel(CS) + 1;
CS(i).key = key;  CS(i).title = ttl;  CS(i).cfg = c;
end


% =============================================================================
function K = structuralChecks(CASE, base, DEG, GRAV)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never evidence that
%   this port reproduces Zarchan -- only the weaker claim that it agrees with
%   itself, which is what catches wiring faults.
names = {};  oks = [];  detail = {};
K = struct();
A = CASE.A;  Bp = CASE.B;  C = CASE.C;  D = CASE.D;
Ec = CASE.E; F = CASE.F;   G = CASE.G;
tolCurve = max(0.005, 4*base.H);

% 1  XNCLIM inert inside the scored window.  If it fires there, every
%    closed-form row is comparing against a saturated loop and superposition
%    is gone with it.  (It DOES fire on the final sample -- see the header --
%    which is outside the window and is reported as an INFO row.)
inWin = 0;
for c = {A,Bp,C,D,Ec,F,G}
    e = c{1};
    inWin = inWin + sum(e.clamped(1:e.i2));
end
K.clamp_in_window = inWin;
names{end+1}  = 'XNCLIM inert inside the scored window';
oks(end+1)    = inWin == 0;
detail{end+1} = sprintf('%d clamped sample(s) at t <= tF - tguard', inWin);

% 2  SUPERPOSITION.  The loop is affine in the state with coefficients that
%    depend on t alone, so A = F + G and C = E + G must hold to ROUNDING on
%    the identical grid.  The sharpest check here: it catches any
%    state-dependent term creeping into the law, and it is what would notice
%    the clamp firing where it should not.
K.superpose = max( ...
    maxRel(A.ncg(1:A.i2), F.ncg(1:F.i2)  + G.ncg(1:G.i2)), ...
    maxRel(C.ncg(1:C.i2), Ec.ncg(1:Ec.i2) + G.ncg(1:G.i2)));
names{end+1}  = 'superposition: A = F+G and C = E+G';
oks(end+1)    = K.superpose <= 1e-9;
detail{end+1} = sprintf('worst %.3e of peak (limit 1e-9)', K.superpose);

% 3  The PN branch really is N'*Vc*lamdot, against the exact zero-lag solution
%    derived in pnClosedForm.  Every shaping row would still pass with the PN
%    branch broken; this is the only check that would not.
if abs(base.XNP - 3) < 1e-12
    yd0 = -base.VM*Bp.cfg.HEDEG/DEG;
    K.pn_resid = max( ...
        maxRel(Bp.ncg(1:Bp.i2), pnClosedForm(Bp.t(1:Bp.i2), base.TF, yd0, Bp.cfg.XNT, GRAV)), ...
        maxRel(D.ncg(1:D.i2),  pnClosedForm(D.t(1:D.i2),  base.TF, 0,   D.cfg.XNT,  GRAV)));
    ok3 = K.pn_resid <= tolCurve;
    d3  = sprintf('%.4f%% of peak (band %.4f%%)', 100*K.pn_resid, 100*tolCurve);
else
    K.pn_resid = NaN;
    ok3 = true;
    d3  = sprintf('not applicable at N'' = %g (the form is N''=3)', base.XNP);
end
names{end+1}  = 'PN branch == 3*ydot(0)/tF*(1-t/tF) + 3*nT*t/tF';
oks(end+1)    = ok3;
detail{end+1} = d3;

% 4  The record is on the grid the caller asked for.  The listing accumulates
%    S by H against a literal threshold; an off-by-one there plots perfectly
%    smoothly on a time axis that is simply wrong.
want = (base.samp:base.samp:base.TF).';
K.grid_n = numel(A.t);
K.grid_resid = Inf;
if numel(A.t) == numel(want)
    K.grid_resid = max(abs(A.t - want));
end
names{end+1}  = 'record on t = samp:samp:TF';
oks(end+1)    = (K.grid_n == numel(want)) && (K.grid_resid <= 0.5*base.H);
detail{end+1} = sprintf('%d samples (want %d), max |dt| = %.3e s', ...
    K.grid_n, numel(want), K.grid_resid);

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

function r = maxRel(a, b)
%MAXREL  Max absolute difference NORMALISED BY THE REFERENCE CURVE'S PEAK.
%   Not pointwise relative: every command curve here crosses zero, and a
%   pointwise relative error there divides a small number by a smaller one
%   and manufactures a failure out of nothing.
den = max(abs(b));
if ~(den > 0)
    r = Inf;                % refuse to report "perfect" against a zero curve
    return
end
r = max(abs(a - b))/den;
end


% =============================================================================
function B = benchmarks(CASE, base, offbook, GRAV)
%BENCHMARKS  Computed vs published.  *** EVERY BOOK VALUE WAS GREPPED OUT OF
%   zarchan.txt AND THE LINE NUMBER IS IN .source. ***  Band provenance is in
%   the header; do not widen one to turn a row green.
B = emptyBench();
A = CASE.A;  Bp = CASE.B;  C = CASE.C;  D = CASE.D;
Ec = CASE.E; F = CASE.F;   G = CASE.G;

tolCurve = max(0.005, 4*base.H);
yd0 = -base.VM*(-20)/57.3;                     % the HE cases' ydot(0), ft/s
ydF = -base.VC*(-30)/57.3;                     % the lamF cases' ydot(tF), ft/s
on  = isempty(offbook);

S13 = 'Fig. 24.13 / p.588 text (zarchan.txt:8313)';
S14 = 'Fig. 24.14 / p.588 text (zarchan.txt:8317)';
S15 = 'Fig. 24.15 / p.589 text (zarchan.txt:8326)';
S04 = 'Fig. 24.4 / p.577 text (zarchan.txt:8132)';
S05 = 'Fig. 24.5 / p.579 text (zarchan.txt:8153)';
S07 = 'Fig. 24.7 / p.581 text (zarchan.txt:8162)';
S08 = 'Fig. 24.8 / p.581 text (zarchan.txt:8163)';
SMS = 'p.577 and p.581 text (zarchan.txt:8131, :8161)';
WANT = sprintf('<= %.3f%% of peak, book says "exact agreement"', 100*tolCurve);

% ---- 1-3.  THE BOOK'S OWN THREE CHECKS: it publishes the configuration AND
%      the formula AND asserts exact agreement.
if on
    r1 = maxRel(Ec.ncg(1:Ec.i2), tsClosedForm(Ec.t(1:Ec.i2), base.TF, 0, 6*GRAV, 0, GRAV));
    B = addBound(B, 'nc|nT == nT(0)   (XNT=193.2, HE=0, lamF=0)', WANT, ...
        sprintf('%.4f%% of peak; nc = %.4f g vs 6.000 g', 100*r1, Ec.ncg(Ec.i2)), ...
        r1 <= tolCurve, S13, 'g', ...
        ['the missile simply matches the target: this row is what pins the ' ...
         'nT term''s coefficient at 1 in the transcribed law']);
    r2 = maxRel(F.ncg(1:F.i2), tsClosedForm(F.t(1:F.i2), base.TF, yd0, 0, 0, GRAV));
    B = addBound(B, 'nc|HE == (2 ydot0/tF)(2-3t/tF)   (HE=-20, lamF=0)', WANT, ...
        sprintf('%.4f%% of peak', 100*r2), r2 <= tolCurve, S14, 'g', ...
        ['ydot(0) = -VM*HE is the listing''s own line 1, which is how the ' ...
         'sign the OCR ate is recovered rather than guessed']);
    r3 = maxRel(G.ncg(1:G.i2), tsClosedForm(G.t(1:G.i2), base.TF, 0, 0, ydF, GRAV));
    B = addBound(B, 'nc|lamF == (2 ydot(tF)/tF)(1-3t/tF)   (lamF=-30)', WANT, ...
        sprintf('%.4f%% of peak', 100*r3), r3 <= tolCurve, S15, 'g', ...
        ['ydot(tF) = -Vc*lamF follows from the listing''s XLAM = Y/(VC*TGO); ' ...
         'this is the row that adjudicates the shaping term itself']);
else
    B = addSkip(B, 'nc|nT == nT(0)   (XNT=193.2, HE=0, lamF=0)', 'nT(0)', offbook, S13, 'g');
    B = addSkip(B, 'nc|HE == (2 ydot0/tF)(2-3t/tF)   (HE=-20, lamF=0)', 'formula', offbook, S14, 'g');
    B = addSkip(B, 'nc|lamF == (2 ydot(tF)/tF)(1-3t/tF)   (lamF=-30)', 'formula', offbook, S15, 'g');
end

% ---- 4-5.  Fig. 24.4, the nominal shaping case. ---------------------------
if on
    B = addVal(B, 'shaping: command at start of flight', 25, A.ncg(A.i1), ...
        'figure', 0.15, S04, 'g', sprintf(['read at the first record, ' ...
        't = %.2f s, where the book''s own plot starts.  Expected +1.7%%'], A.t(A.i1)));
    B = addVal(B, 'shaping: command near end of flight', -30, A.ncg(A.i2), ...
        'figure', 0.15, S04, 'g', sprintf(['book says "approximately"; read ' ...
        'at t = %.2f s, one sample before the artifact at tF.  Expected ' ...
        '+6.5%%'], A.t(A.i2)));
    % ---- 6-8.  The final LOS angle, the whole point of the law. -----------
    B = addVal(B, 'shaping: final LOS meets the -30 deg design goal', -30, ...
        A.lamdeg(A.i2), 'text', 0.10, S05, 'deg', sprintf(['sampled at ' ...
        't = %.2f s: the angle is still moving over the last %.2f s (0.75 ' ...
        'deg) and the record AT tF is an artifact.  Expected -2.5%%'], ...
        A.t(A.i2), base.tguard));
    B = addVal(B, 'PN: final LOS is uncontrolled (~8 deg)', 8, ...
        Bp.lamdeg(Bp.i2), 'text', 0.10, S05, 'deg', ...
        ['the book''s point is that PN does not control this angle at all -- ' ...
         '"a matter of luck".  Expected -6.3%']);
    B = addVal(B, 'PN: final LOS vs 6 g maneuvering target', 14, ...
        D.lamdeg(D.i2), 'text', 0.10, S08, 'deg', ...
        'the same uncontrolled angle for the other disturbance.  Expected -3.1%');
else
    B = addSkip(B, 'shaping: command at start of flight', '25', offbook, S04, 'g');
    B = addSkip(B, 'shaping: command near end of flight', '-30', offbook, S04, 'g');
    B = addSkip(B, 'shaping: final LOS meets the -30 deg design goal', '-30', offbook, S05, 'deg');
    B = addSkip(B, 'PN: final LOS is uncontrolled (~8 deg)', '8', offbook, S05, 'deg');
    B = addSkip(B, 'PN: final LOS vs 6 g maneuvering target', '14', offbook, S08, 'deg');
end

% ---- 9.  "y(10) = 0 in both cases", said twice (:8131 heading error, :8161
%      target maneuver).  A bound, because zero has no percentage.
mis = max(abs([A.y(end), Bp.y(end), C.y(end), D.y(end)]));
B = addBound(B, 'miss is zero for BOTH laws and BOTH disturbances', ...
    '|y(tF)| <= 1 ft', sprintf('worst |y(tF)| = %.4g ft over the 4 cases', mis), ...
    mis <= 1.0, SMS, 'ft', ...
    ['1 ft is 1e-4 of the heading-error excursion ydot(0)*tF = 10471 ft and ' ...
     '0.25 ms of closing at Vc.  Anything that actually breaks the intercept ' ...
     '-- a wrong navigation ratio, a dropped term -- lands in the hundreds']);

% ---- 10.  THE CHAPTER'S HEADLINE ORDERING, from prose about two figures. --
mA = max(abs(A.ncg(1:A.i2)));   mB = max(abs(Bp.ncg(1:Bp.i2)));
mC = max(abs(C.ncg(1:C.i2)));   mD = max(abs(D.ncg(1:D.i2)));
B = addShape(B, 'shaping costs more acceleration than PN, both disturbances', ...
    'max|nc| shaping > max|nc| PN for HE and for target maneuver', ...
    sprintf('HE: %.2f > %.2f g;  6 g target: %.2f > %.2f g', mA, mB, mC, mD), ...
    (mA > mB) && (mC > mD), ...
    'zarchan.txt:8132 and :8162 ("significantly more", "requires more")', ...
    ['an ordering out of prose is stronger than a number eyeballed off a ' ...
     'plot, and it is the one scored row that survives off-book parameters']);

% ---- INFO.  Reported, never adjudicated. ---------------------------------
B = addInfo(B, 'Fig. 24.7 levels (6 g target): book 20 / -20 g, PN "nearly 20 g"', ...
    '20 / -20 / ~20', sprintf('shaping %.2f and %.2f g; PN ends at %.2f g', ...
    C.ncg(C.i1), C.ncg(C.i2), D.ncg(D.i2)), S07, ...
    ['NOT SCORED: the same KIND of claim as the Fig. 24.4 rows, and this ' ...
     'case is the SUPERPOSITION of the nT and lamF rows, whose formula is ' ...
     'already scored exactly and whose sum the structural block verifies']);
B = addInfo(B, 'the final recorded sample (t = tF) is an ARTIFACT', ...
    'clipped out of the book''s own axis() boxes', ...
    sprintf(['case A: nc = %.4g g and LOS = %.3g deg at tF, against %.2f g ' ...
    'and %.2f deg one sample earlier; XNCLIM fired %d time(s) across the ' ...
    'seven runs'], A.ncg(end), A.lamdeg(end), A.ncg(A.i2), A.lamdeg(A.i2), ...
    A.n_clamp + Bp.n_clamp + C.n_clamp + D.n_clamp + Ec.n_clamp + ...
    F.n_clamp + G.n_clamp), ...
    'mechanism derived in this file''s header, not in the book', ...
    ['NOT SCORED, and not a defect of the port: TGO = tF - t + 1e-5 hands ' ...
     'the law a 1e-5 s time-to-go when the true one is zero, so XLAM and ' ...
     'XLAMD become 0/0 forms on a predictor state.  Everything scored above ' ...
     'is read at or before tF - tguard']);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly or the
%   appends throw.  A struct array whose fields reordered would make [B.ok]
%   mean something else.  This is the schema benchSuite's adaptGeneric
%   consumes; it is deliberately STRICT and a near-miss errors.
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

function B = addBound(B, what, bookStr, oursStr, ok, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'bound';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'bound';  B(i).tol = NaN;
B(i).ok   = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = units;  B(i).detail = detail;
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
%ADDINFO  Reported, never scored: it can neither pass nor fail.
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
function s = render(R, ev, offbook)
L = {};
p = R.p;
L{end+1} = 'Zarchan Listing 24.1 -- LINEAR ENGAGEMENT, TRAJECTORY SHAPING vs PN';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2, deg via 57.3.  NOT SI. ***';
L{end+1} = sprintf(['   VC = %g   VM = %g ft/s   TF = %g s   N'' = %g   ' ...
    'XNCLIM = %g ft/s^2   H = %g   samp = %g   tguard = %g s'], ...
    p.VC, p.VM, p.TF, p.XNP, p.XNCLIM, p.H, p.samp, p.tguard);
if ~isempty(offbook)
    L{end+1} = ['   OFF-BOOK: ' offbook];
end
L{end+1} = '';
L{end+1} = 'THE SEVEN PUBLISHED CONFIGURATIONS.  g and deg, at the first record and';
L{end+1} = 'at the last one inside t <= tF - tguard; the record AT tF is an artifact.';
L{end+1} = sprintf('%4s %-48s %8s %8s %8s %9s', 'case', 'configuration', ...
    'nc first', 'nc last', 'LOS last', 'y(tF) ft');
k = fieldnames(R.case);
for i = 1:numel(k)
    E = R.case.(k{i});
    L{end+1} = sprintf('%4s %-48s %8.3f %8.3f %8.3f %9.3g', k{i}, E.title, ...
        E.ncg(E.i1), E.ncg(E.i2), E.lamdeg(E.i2), E.y(end)); %#ok<AGROW>
end
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-48s %s', vstr(K.oks(i)), K.names{i}, ...
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
                L{end+1} = sprintf('   INFO %-56s %s', b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why);                    %#ok<AGROW>
            case 'skip'
                L{end+1} = sprintf('   SKIP %-56s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why);                    %#ok<AGROW>
            case {'shape','bound'}
                L{end+1} = sprintf('   %-4s %-56s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str);         %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf('   %-4s %-56s book %s %s / ours %s %s (%+.3f%%, band %.4g%%)', ...
                    vstr(b.ok), b.what, b.book_str, b.units, b.ours_str, ...
                    b.units, b.pct, 100*b.tol);                             %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped/info -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: ZERO LAG, constant Vc, linearised geometry, tF given as an';
L{end+1} = '   input.  The law''s DEFINITION and a clean-loop baseline -- not a';
L{end+1} = '   prediction for an airframe with a saturating gamma loop.';
s = strjoin(L, newline);
end

function s = vstr(tf)
if tf
    s = 'PASS';
else
    s = 'FAIL';
end
end
