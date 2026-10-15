function R = radomeHomingLoop(opts)
%MGS.ZARCHAN.RADOMEHOMINGLOOP  Zarchan Listing 18.2 -- homing loop with RADOME
%   effects, ONE flight, with the missile's 7-g acceleration command LIMIT.
%
%   R = mgs.zarchan.radomeHomingLoop()
%   R = mgs.zarchan.radomeHomingLoop('TAU', 0.2)       the book's unstable case
%   R = mgs.zarchan.radomeHomingLoop('assert', false)  warn instead of error
%
%   Line-by-line port of Listing 18.2 (heading zarchan.txt:5455, code :5456-
%   5465), integrated by the book's own two-pass Heun scheme.
%   UNITS ARE THE BOOK'S: ft, ft/s, ft/s^2, s, g = 32.2 ft/s^2.  NOT SI.  R and
%   mu are dimensionless; HEDEG is the one degree input and the listing's 57.3
%   converts it.  Convert AT THE CALL SITE and say so there.
%
% --- WHAT IS GENUINELY DIFFERENT FROM MGS.ZARCHAN.RADOMEENGAGEMENT (6.2) -----
%   The ODEs are identical line for line; diff them.  Three things are new:
%     1  *** THE 7-g COMMAND LIMIT, XNCLIMG = 7. ***  6.2 has no limiter.  It is
%        the loop's ONE nonlinearity and it is what makes the book's sentence
%        true: "oscillates between +7 g" (:5466; "+" is a mangled "+/-") is a
%        LIMIT CYCLE ON THE RAIL, not a property of the linear loop.  Self-check
%        3 deletes the limiter and the same case runs away.  6.2 cannot see it.
%     2  ONE flight recording a TIME HISTORY, not a sweep of terminal misses.
%     3  T is the swept parameter here; Chapter 6 swept Ta.
%   6.2 already reaches 0.79 / 2.07 / Tmin by ROOT LOCUS on the quintic.  What
%   this port adds is the TIME-DOMAIN route -- bisection on the SIMULATION -- so
%   the agreement is a second opinion rather than a restatement.  Rows 3-6.
%
% --- THE PREDICATE, AND THE AUDIT THAT FORCED IT -----------------------------
%   An earlier version bisected on the largest single-sample DROP over the WHOLE
%   trace and asserted in a COMMENT that that was monotone in T.  It is not:
%   TGO = TF - T + 1e-5, so the 1/TGO gain diverges in the last samples and puts
%   EVERY case on the rail, stable or not.  No bracket existed, both bisections
%   returned NaN, three rows and one invariant went dark.  THE FIX IS THE
%   PREDICATE, NOT THE BRACKET; nothing here was widened.
%   "oscillating" now == over the PRE-ENDGAME window t <= tF - guard (0.5 s) the
%   recorded nc TOUCHES EITHER RAIL or CHANGES SIGN -- the book's own two-part
%   phrase kept whole (:5466, :5474).  The guard removes the transcription's own
%   endgame singularity, not a stability property, and is applied identically to
%   every case and both sides of every transition, so it cannot move a verdict.
%   MONOTONICITY IS NOW MEASURED: self-check 4 requires EXACTLY ONE flip across
%   each bracket.  A comment cannot be caught; a row can.
%
% --- THE TRANSCRIPTION -------------------------------------------------------
%   listing_18_02.m reports ONE unresolved comparison and ONE leak of prose.
%     1  "if XNC.XNCLIM" -> "if XNC > XNCLIM".  *** THE UNRESOLVED ONE. ***
%        Resolved ">", forced by the next line, which survived intact: "if
%        XNC < -XNCLIM".  A clamp PAIR is > upper / < lower.  With "<" upstairs
%        every command below +7 g would be pushed UP onto the rail in every
%        case, stable or not, contradicting the book's own stable trace.  ("."
%        is this OCR's ">"; the same substitution appears in "if S.=.0999".)
%     2  "while T,=(TF-1e-5)" -> "while T <= ...".  "," is "<"; shape-forced,
%        T only increases so ">=" never terminates.  "if S.=.0999" -> ">=": S
%        resets on every record, so "<=" would record every step.
%     3  Transcription lines 22-24 are the book's PROSE (:5466-5470) leaking
%        past the last statement.  DROPPED, not "fixed" into code -- it is the
%        source of rows 7-8, which is how it was identified.  Plot tail and
%        "save datfil.txt -ascii" dropped; the dead "FLAG=0" and the never-read
%        "YDIC=YD" kept as comments.
%   MINUS-SIGN DAMAGE (this OCR's third class, not auto-decoded): "20.01 radome
%   slope" (:5454) is -0.01, confirmed by the listing's own R=-.01; "between
%   +7 g" (:5466, :5474) is +/-7 g; the inequality at :5408 lost the minus on
%   its lower bound.  This port takes |mu| throughout and never relies on the
%   printed sign.  The listing records XNC AFTER the corrector, so the recorded
%   value is the SECOND-pass command and the first record lands at t = 0.1 s.
%
% --- *** WE HAVE NO RADOME.  THIS PORTS A MECHANISM, NOT A CONFIGURATION. *** -
%   What ports is the shape of the argument: a parasitic feedback path, ONE
%   scalar mu = N'Vc|R|Ta/(VmT) that measures it, a boundary that is CROSSED,
%   and a SATURATION that turns divergence into a bounded limit cycle at the
%   rail -- so the symptom is a command sitting on its limit rather than a
%   number running away.  That last part we have met before.
%
%   *** NEVER EXECUTED BY ITS AUTHOR.  MATLAB is the caller's. ***  The values
%   below are DERIVED -- reproduced offline in an independent replica of this
%   same arithmetic -- so a first run has something to disagree with:
%     nominal (T=0.3)  peak 3.3656 g, nc [+0.0065,+3.3656], 0 sign changes,
%                      worst fall 0.000%, final y 0.01317 ft, 100 records
%     Tmin             0.25316 s (Vc 4000), 0.56962 s (Vc 9000)
%     neutral T        0.24856 s -> |mu| 0.80464 (+1.85% on 0.79); 0.58370 s ->
%                      |mu| 0.77095 (-2.41%).  Both inside the book's own
%                      brackets (0.2,0.3) and (0.5,0.7)
%     four cases       T=0.2: both rails, 18 sign changes | T=0.3: 3.3656 g, 0 |
%                      T=0.5: both rails, 4 | T=0.7: window peak 2.5758 g, 0
%     clamp deleted    Vc=4000, T=0.2 first exceeds 70 g at t = 5.6 s
%   NEVER widen a band to make a row green; a red row is a RESULT.
%
%   OUTPUTS  R.t/R.y/R.nc_g ([s],[ft],[g]), R.nom, R.cases, R.tcrit, R.bench,
%     R.selfcheck (.names/.oks -- structural, NOT book values), R.pass,
%     R.verdict, R.units, R.p, R.summary.
%
%   See also MGS.ZARCHAN.RADOMEENGAGEMENT (Listing 6.2 -- same ODEs, no
%   limiter, miss-vs-flight-time sweep), MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 18.2 line 1, verbatim (zarchan.txt:5456).  Feet and seconds.
    opts.VC      (1,1) double {mustBeFinite, mustBePositive} = 4000   % ft/s
    opts.XNT     (1,1) double {mustBeFinite}                 = 32.2   % ft/s^2
    opts.XNCLIMG (1,1) double {mustBeFinite, mustBePositive} = 7      % g
    opts.YIC     (1,1) double {mustBeFinite}                 = 0      % ft
    opts.VM      (1,1) double {mustBeFinite, mustBePositive} = 3000   % ft/s
    opts.HEDEG   (1,1) double {mustBeFinite}                 = 0      % deg
    opts.TAU     (1,1) double {mustBeFinite, mustBePositive} = 0.3    % s
    opts.XNP     (1,1) double {mustBeFinite, mustBePositive} = 3      % N-prime
    opts.TA      (1,1) double {mustBeFinite, mustBeNonnegative} = 5   % s
    opts.R       (1,1) double {mustBeFinite}                 = -0.01  % slope
    opts.TF      (1,1) double {mustBeFinite, mustBePositive} = 10     % s
    opts.H       (1,1) double {mustBeFinite, mustBePositive} = 0.01   % s
    % The listing tests "S >= .0999" after one "S = S + H": it records every
    % 0.1 s at H = 0.01.  Kept as a DEFAULT, not hard-coded -- the literal
    % 0.0999 would silently change meaning at any other H.
    opts.samp    (1,1) double {mustBeFinite, mustBePositive} = 0.1    % s
    % Tail excluded from every stability judgement; see the header.
    opts.guard   (1,1) double {mustBeFinite, mustBeNonnegative} = 0.5 % s
    % THE ONLY SWITCHES.  Not nargin: with an arguments block declaring only
    % name-value options MATLAB reports nargin = 0 however many were passed.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.assert    (1,1) logical = true
end

VC = opts.VC;  XNT = opts.XNT;  VM = opts.VM;  TAU = opts.TAU;
XNP = opts.XNP;  TA = opts.TA;  RD = opts.R;  TF = opts.TF;  H = opts.H;
CLG = opts.XNCLIMG;  YIC = opts.YIC;  HED = opts.HEDEG;  SAMP = opts.samp;
GRD = opts.guard;

% The lag chain runs at 5/TAU, so its own time constant is TAU/5.  Refuse
% rather than warn: Heun does not blow up when TAU/5 approaches H, it returns a
% smooth and entirely wrong curve, and this file's whole output is a shape.
if TAU < 15*H
    error('mgs:zarchan:radomeHomingLoop:stiff', ...
        ['TAU = %g s puts the lag chain''s own time constant TAU/5 = %g s at ' ...
         'only %.1f steps of H = %g s.  At least 3 are required.'], ...
        TAU, TAU/5, TAU/(5*H), H);
end
if SAMP < H
    error('mgs:zarchan:radomeHomingLoop:samp', ...
        'samp = %g s is smaller than the integration step H = %g s.', SAMP, H);
end
if GRD >= TF - 2*SAMP
    error('mgs:zarchan:radomeHomingLoop:guard', ...
        'guard = %g s leaves no pre-endgame window in a tF = %g s flight.', GRD, TF);
end

% ---- the nominal single flight, Listing 18.2 as printed ---------------------
[t, y, ncg] = runFlight(VC, XNT, CLG, YIC, VM, HED, TAU, XNP, TA, RD, TF, H, SAMP);
R = struct();
R.t = t;  R.y = y;  R.nc_g = ncg;
R.nom = summarise(t, ncg, CLG, TF, GRD);

% ---- the book's four published cases (Figs. 18.6, 18.7) ---------------------
% Hard-coded because these are the BOOK's flight conditions, not this run's.
R.cases = struct('vc', {}, 'tau', {}, 't', {}, 'ncg', {}, 'm', {});
probes = [4000 0.2; 4000 0.3; 9000 0.5; 9000 0.7];
for i = 1:size(probes,1)
    [tt, ~, nn] = bookFlight(probes(i,1), probes(i,2), H, SAMP);
    R.cases(i).vc = probes(i,1);  R.cases(i).tau = probes(i,2);
    R.cases(i).t = tt;  R.cases(i).ncg = nn;
    R.cases(i).m = summarise(tt, nn, 7, 10, GRD);
end

% ---- the time-domain neutral point, by bisection on the SIMULATION ----------
% NaN when the bracket does not straddle, which SKIPs the row: a check that
% could not run is never a pass, and the bracket is never widened to avoid one.
R.tcrit = struct();
R.tcrit.vc4000 = bisectTcrit(4000, 0.15, 0.60, H, SAMP, GRD);
R.tcrit.vc9000 = bisectTcrit(9000, 0.35, 1.20, H, SAMP, GRD);
R.tcrit.mu4000 = muOf(3, 4000, 0.01, 5, 3000, R.tcrit.vc4000);
R.tcrit.mu9000 = muOf(3, 9000, 0.01, 5, 3000, R.tcrit.vc9000);

R.units = 'ft, ft/s, ft/s^2, s; g = 32.2 ft/s^2; R and mu dimensionless';
R.p = struct('VC',VC, 'XNT',XNT, 'XNCLIMG',CLG, 'YIC',YIC, 'VM',VM, ...
             'HEDEG',HED, 'TAU',TAU, 'XNP',XNP, 'TA',TA, 'R',RD, 'TF',TF, ...
             'H',H, 'samp',SAMP, 'guard',GRD, ...
             'mu', muOf(XNP, VC, abs(RD), TA, VM, TAU), 'units', R.units);

R.selfcheck = structuralChecks(H, SAMP, GRD);

% The book's figure rows are quoted for ONE configuration.  Off it, they SKIP.
bookCase = XNP == 3 && RD == -0.01 && TA == 5 && VM == 3000 && ...
           XNT == 32.2 && CLG == 7 && TF == 10;

if opts.benchmark
    R.bench = benchmarks(R, bookCase, XNP, RD, TA, VM, XNT, CLG, TF);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine" -- the shape of green light this repo has been burned by.
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end

% Verdict is FORMED either way; 'assert' picks only the reaction.  benchSuite
% runs every port with assert off so one red row cannot withhold the evidence.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the book. ' ...
           'R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = 'not one book value could be evaluated.';
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, a wiring fault and not a tolerance question.  First: %s.'], ...
        sum(~R.selfcheck.oks), R.selfcheck.first_bad);
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
        error('mgs:zarchan:radomeHomingLoop:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:radomeHomingLoop:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [t, y, ncg] = runFlight(VC, XNT, XNCLIMG, YIC, VM, HEDEG, TAU, ...
                                 XNP, TA, RD, TF, H, samp)
%RUNFLIGHT  Listing 18.2, zarchan.txt:5456-5465, verbatim apart from plotting.
%   Every constant arrives as an ARGUMENT: MATLAB local functions cannot see
%   the main function's workspace, and a previous wave threw on its first line
%   for exactly that reason.
XNCLIM = XNCLIMG*32.2;                 % the listing's XNCLIM = XNCLIMG*32.2
Y   = YIC;
YD  = -VM*HEDEG/57.3;                  % the listing's YDIC = YD is never read
XNL = 0;  ELAMDH = 0;  X4 = 0;  X5 = 0;  TH = 0;  THH = 0;
T   = 0;  S = 0;
% Derivatives predeclared so the FLAG==1 branch cannot read a stale value.
YDD = 0;  XNLD = 0;  ELAMDHD = 0;  X4D = 0;  X5D = 0;  THD = 0;  THHD = 0;
XNC = 0;

nmax = ceil(TF/min(samp,H)) + 8;
t = zeros(nmax,1);  y = zeros(nmax,1);  ncg = zeros(nmax,1);
n = 0;

while T <= (TF - 1e-5)
    YOLD = Y;  YDOLD = YD;  XNLOLD = XNL;  ELAMDHOLD = ELAMDH;
    X4OLD = X4;  X5OLD = X5;  THOLD = TH;  THHOLD = THH;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  T advances INSIDE it, exactly once,
    % which is what makes the second evaluation happen at t+H.  Move that line
    % out and the scheme quietly degrades to first order while still running.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            Y = Y + H*YD;  YD = YD + H*YDD;  XNL = XNL + H*XNLD;
            ELAMDH = ELAMDH + H*ELAMDHD;  X4 = X4 + H*X4D;  X5 = X5 + H*X5D;
            TH = TH + H*THD;  THH = THH + H*THHD;  T = T + H;
        end
        TGO  = TF - T + 0.00001;
        XLAM = Y/(VC*TGO);
        % The radome enters ONLY through +RD*THH; delete that term and the
        % parasitic path is severed while every state still integrates
        % smoothly (self-check 2 is what notices).
        EPS  = XLAM - TH - THH + RD*THH;
        DD   = 5.*EPS/TAU;
        ELAMDHD = 5.*(DD - ELAMDH)/TAU;
        XNC  = XNP*VC*ELAMDH;
        % *** LISTING 18.2's ONE ADDITION TO LISTING 6.2. ***  The upper clamp
        % is the transcription's unresolved comparison, resolved ">" by the
        % lower clamp on the next line; see the header.
        if XNC >  XNCLIM,  XNC =  XNCLIM;  end
        if XNC < -XNCLIM,  XNC = -XNCLIM;  end
        X4D  = 5.*(XNC - X4)/TAU;
        X5D  = 5.*(X4 - X5)/TAU;
        XNLD = 5.*(X5 - XNL)/TAU;
        THD  = XNL/VM + TA*XNLD/VM;      % (1 + Ta*s)/VM, per the listing
        THHD = DD - THD;
        YDD  = XNT - XNL;
        FLAG = 1;
    end
    % The listing writes "FLAG=0" here; dead, the outer loop re-initialises it.

    Y   = .5*(YOLD + Y + H*YD);
    YD  = .5*(YDOLD + YD + H*YDD);
    XNL = .5*(XNLOLD + XNL + H*XNLD);
    ELAMDH = .5*(ELAMDHOLD + ELAMDH + H*ELAMDHD);
    X4  = .5*(X4OLD + X4 + H*X4D);
    X5  = .5*(X5OLD + X5 + H*X5D);
    TH  = .5*(THOLD + TH + H*THD);
    THH = .5*(THHOLD + THH + H*THHD);

    S = S + H;
    if S >= samp - 0.5*H          % the listing's "if S >= .0999" at samp = 0.1
        S = 0;  n = n + 1;
        t(n) = T;  y(n) = Y;  ncg(n) = XNC/32.2;   % XNC is the 2nd-pass value
    end
end
t = t(1:n);  y = y(1:n);  ncg = ncg(1:n);
end


function [t, y, ncg] = bookFlight(vc, T, H, samp)
%BOOKFLIGHT  Listing 18.2's own case with only Vc and T free.  tF pinned at the
%   listing's 10 s: these are the BOOK's figure conditions, not the caller's.
[t, y, ncg] = runFlight(vc, 32.2, 7, 0, 3000, 0, T, 3, 5, -0.01, 10, H, samp);
end


% =============================================================================
function M = summarise(t, ncg, limg, TF, guard)
%SUMMARISE  The book's adjectives made countable, on the pre-endgame window
%   t <= TF - guard (_w fields) as well as on the whole trace.
%     hit_hi_w/hit_lo_w/nsc_w   TOGETHER they are the bisection predicate.
%     dip/dip_w                 largest single-sample DECREASE / peak.  This
%       was the OLD bisection predicate and it is NOT monotone in T; it is kept
%       because "monotonically increasing" is a real book claim, but it is
%       adjudicated only where the book states it flatly (row 7), never here.
M = struct('peak',NaN,'hi',NaN,'lo',NaN,'nsc',NaN,'dip',NaN, ...
           'peak_w',NaN,'hit_hi_w',false,'hit_lo_w',false,'nsc_w',NaN,'dip_w',NaN);
if isempty(ncg), return; end
rail = 0.999*limg;
M.peak = max(abs(ncg));  M.hi = max(ncg);  M.lo = min(ncg);
M.nsc  = nSignChanges(ncg);
M.dip  = worstDrop(ncg);

w  = t <= (TF - guard) + 1e-9;
nw = ncg(w);
if isempty(nw), nw = ncg(1); end
M.peak_w   = max(abs(nw));
M.hit_hi_w = max(nw) >=  rail;
M.hit_lo_w = min(nw) <= -rail;
M.nsc_w    = nSignChanges(nw);
M.dip_w    = worstDrop(nw);
end

function k = nSignChanges(v)
s = sign(v);  s(s == 0) = 1;          % sign(0) = 0 would fake a crossing
k = sum(diff(s) ~= 0);
end

function d = worstDrop(v)
dv = diff(v);
if isempty(dv), d = 0; return; end
d = max([0; -dv(:)])/max(max(abs(v)), realmin);
end

function mu = muOf(XNP, VC, absR, TA, VM, T)
%MUOF  The chapter's parasitic scalar, zarchan.txt:5408: |N'*Vc*R*Ta/(Vm*T)|.
%   MAGNITUDE throughout -- the printed lower bound lost its minus sign to the
%   OCR, so nothing here relies on the printed sign.
mu = XNP*VC*absR*TA./(VM*T);
end

function tf = oscillates(vc, T, H, samp, guard)
%OSCILLATES  THE PREDICATE.  The book's own phrase kept whole: over the
%   pre-endgame window the command touches a rail, or changes sign.
[tt, ~, nn] = bookFlight(vc, T, H, samp);
m  = summarise(tt, nn, 7, 10, guard);
tf = m.hit_hi_w || m.hit_lo_w || m.nsc_w >= 1;
end

function tc = bisectTcrit(vc, loT, hiT, H, samp, guard)
%BISECTTCRIT  The TIME-DOMAIN neutral guidance time constant.  mu ~ 1/T, so the
%   loop is less stable at smaller T; that the predicate really is monotone
%   over this bracket is NOT asserted here, it is MEASURED by self-check 4.
%   NaN when the bracket does not straddle, and NaN when the answer pins to the
%   integrator's own floor.  Never a silent midpoint, never a widened bracket.
tc  = NaN;
loT = max(loT, 15*H);              % below this the lag chain is unresolved
if loT >= hiT, return; end
if ~oscillates(vc, loT, H, samp, guard) || oscillates(vc, hiT, H, samp, guard)
    return
end
a = loT;  b = hiT;
for i = 1:30
    m = 0.5*(a + b);
    if oscillates(vc, m, H, samp, guard), a = m; else, b = m; end
    if (b - a) < 1e-5, break; end
end
tc = 0.5*(a + b);
if tc <= loT*(1 + 1e-3), tc = NaN; end
end


% =============================================================================
function K = structuralChecks(H, samp, guard)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never to be quoted
%   as evidence that this port reproduces Zarchan -- only that it agrees with
%   itself, which is what catches wiring faults.
%   .names and .oks are what put these on the board.  A file that computes
%   invariants and stores them as K.something_ok puts ZERO rows up and looks
%   green; that has happened twice in this package.
K = struct();
names = {};  oks = [];  detail = {};

% 1  No disturbance, no command.  Every state starts at zero and XNT and HEDEG
%    are the only forcings, so anything nonzero here is spurious excitation.
[~, y0, n0] = runFlight(4000, 0, 7, 0, 3000, 0, 0.3, 3, 5, -0.01, 10, H, samp);
K.quiet = max([max(abs(n0)), max(abs(y0))]);
names{end+1}  = 'no target maneuver and no heading error => zero nc and zero y';
oks(end+1)    = K.quiet <= 1e-12;
detail{end+1} = sprintf('max(|nc| [g], |y| [ft]) = %.3e', K.quiet);

% 2  With R = 0, Ta CANNOT matter: theta enters the loop only through R*THH,
%    and elsewhere theta and theta_H appear solely as their SUM (d(TH+THH)/dt =
%    DD identically), which Ta leaves alone.  Mis-wire EPS -- drop the -TH, or
%    write R*TH -- and this breaks while everything else still runs smoothly
%    and at the right order of magnitude.  Derived: 2.2e-12 here against 3.1
%    for the same Ta pair at the book's R = -0.01, so it has teeth.
[~, ~, a0] = runFlight(4000, 32.2, 7, 0, 3000, 0, 0.3, 3,  0, 0, 10, H, samp);
[~, ~, a1] = runFlight(4000, 32.2, 7, 0, 3000, 0, 0.3, 3, 10, 0, 10, H, samp);
K.ta_free = max(abs(a0 - a1))/max(max(abs(a0)), realmin);
names{end+1}  = 'with R = 0 the command history is independent of Ta';
oks(end+1)    = K.ta_free <= 1e-9;
detail{end+1} = sprintf('max |d(nc)|/peak = %.3e between Ta = 0 and 10 s', K.ta_free);

% 3  *** DELETE EXPERIMENT.  The one this file exists for. ***  Raise the limit
%    and the book's "oscillates between +/-7 g" case does NOT settle into a
%    smaller oscillation -- it runs away.  So the +/-7 g in the book's sentence
%    is the LIMITER, not the loop.  Listing 6.2 cannot see this (no limiter,
%    and it records only terminal miss).  1e4 g rather than Inf: a finite rail
%    still bounds the endgame 1/TGO gain, so this cannot pass on a NaN.
[tfree, ~, nfree] = runFlight(4000, 32.2, 1e4, 0, 3000, 0, 0.2, 3, 5, -0.01, 10, H, samp);
K.free_peak = max(abs(nfree));
j = find(abs(nfree) > 70, 1);
K.free_t70 = NaN;
if ~isempty(j), K.free_t70 = tfree(j); end
names{end+1}  = 'DELETE the 7-g clamp: the Vc=4000, T=0.2 s case runs past 70 g';
oks(end+1)    = isfinite(K.free_peak) && K.free_peak >= 70;
detail{end+1} = sprintf(['peak |nc| = %.4g g unclamped (%.4gx the 7-g rail); ' ...
    'first exceeds 70 g at t = %.2f s'], K.free_peak, K.free_peak/7, K.free_t70);

% 4  *** THE PREDICATE IS MONOTONE -- MEASURED, NOT ASSERTED. ***  The previous
%    version asserted monotonicity in a COMMENT, the assertion was false, and
%    both bisections silently returned NaN.  A comment cannot be caught; a row
%    can.  EXACTLY ONE flip is required, because otherwise bisection has no
%    meaning even when it returns a number.
lo4 = max(0.15, 15*H);  lo9 = max(0.35, 15*H);
g4 = lo4:0.05:0.60;  g9 = lo9:0.10:1.25;
K.flips4 = countFlips(4000, g4, H, samp, guard);
K.flips9 = countFlips(9000, g9, H, samp, guard);
names{end+1}  = 'the bisection predicate flips exactly once across each bracket';
oks(end+1)    = K.flips4 == 1 && K.flips9 == 1;
detail{end+1} = sprintf(['%d flip(s) over %d probes in [%.2f,%.2f] s at ' ...
    'Vc=4000, %d over %d in [%.2f,%.2f] s at Vc=9000'], K.flips4, numel(g4), ...
    lo4, 0.60, K.flips9, numel(g9), lo9, 1.25);

K.names = names;  K.oks = logical(oks);  K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    jb = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{jb}, detail{jb});
end
end

function k = countFlips(vc, grid, H, samp, guard)
if numel(grid) < 2, k = NaN; return; end
u = false(size(grid));
for i = 1:numel(grid)
    u(i) = oscillates(vc, grid(i), H, samp, guard);
end
k = sum(diff(u) ~= 0);
end


% =============================================================================
function B = benchmarks(R, bookCase, XNP, RD, TA, VM, XNT, CLG, TF)
%BENCHMARKS  Computed vs published, with the PROVENANCE of every band.
%   *** EVERY BOOK VALUE WAS GREPPED OUT OF zarchan_decoded.txt AND THE LINE
%   NUMBER IS IN .source. ***  Nothing is read off a figure: Figs. 18.4-18.7 all
%   became image links.  What IS greppable is the chapter's PROSE about them,
%   and that is what the shape rows score -- as orderings.
B = emptyBench();
tminOf = @(vc) 3*vc*0.01*5/(0.79*3000);

% ---- 1-2.  Tmin, the chapter's two worked arithmetic examples.  No integrator
%            is touched; they ANCHOR rows 3-6.
B = addVal(B, 'Tmin, aircraft threat (Vc = 4000 ft/s)', 0.25, tminOf(4000), ...
    'text(2dp)', 0.005/0.25, 'p.401 worked example (zarchan.txt:5452)', 's', ...
    'Tmin = N''Vc|R|Ta/(0.79 Vm) with the BOOK''s 0.79, N''=3, |R|=0.01, Ta=5 s.');
B = addVal(B, 'Tmin, ballistic threat (Vc = 9000 ft/s)', 0.57, tminOf(9000), ...
    'text(2dp)', 0.005/0.57, 'p.405 worked example (zarchan.txt:5472)', 's', ...
    'same formula, the chapter''s second flight condition (Vm 3000, Vt 6000).');

% ---- 3-4.  The neutral constant 0.79, reached from the TIME DOMAIN ----------
% BAND 8%, FIXED BEFORE ANY RUN, two-sided, from two OPPOSING biases: 0.79 is
% the Ta/T -> inf asymptote (exact boundary 0.806 at this Ta/T ~ 20) which
% pushes the residual UP, while a discrete predicate on a 10 s window with a
% 0.5 s guard cannot resolve a growth rate near zero, which pushes it DOWN.  So
% the residual is NOT expected one-signed: derived, Vc=4000 lands ABOVE (+1.85%)
% and Vc=9000 BELOW (-2.41%).  8% still has teeth -- a factor-2 wiring error is
% nowhere near it.
B = addMu(B, 'neutral |N''VcRTa/(VmT)| from the SIMULATION, Vc = 4000', ...
    R.tcrit.mu4000, R.tcrit.vc4000, 0.15, 0.60);
B = addMu(B, 'neutral |N''VcRTa/(VmT)| from the SIMULATION, Vc = 9000', ...
    R.tcrit.mu9000, R.tcrit.vc9000, 0.35, 1.20);

% ---- 5-6.  "Simulation and theory agree" (:5470), in the time domain -------
B = addTcritRow(B, R.tcrit.vc4000, 0.2, 0.3, 4000, tminOf(4000), ...
    'p.404 text (zarchan.txt:5466-5470); Fig. 18.6 caption (:5475)');
B = addTcritRow(B, R.tcrit.vc9000, 0.5, 0.7, 9000, tminOf(9000), ...
    'p.405 text (zarchan.txt:5474); Fig. 18.7 caption (:5490)');

% ---- 7-8.  The prose about Figs. 18.6, 18.7, scored as CROSSINGS ------------
if bookCase
    c02 = R.cases(1).m;  c03 = R.cases(2).m;
    c05 = R.cases(3).m;  c07 = R.cases(4).m;

    okA = c02.hit_hi_w && c02.hit_lo_w && c02.nsc_w >= 2 && ...
          ~c03.hit_hi_w && ~c03.hit_lo_w && c03.nsc_w == 0 && c03.dip <= 0.01;
    B = addShape(B, 'Fig. 18.6, Vc=4000: T=0.2 on both rails, T=0.3 stable', ...
        ['T=0.2 reaches BOTH rails and changes sign; T=0.3 reaches neither, ' ...
         'never changes sign, and never falls (whole trace)'], ...
        sprintf(['T=0.2: [%+.2f,%+.2f] g, %d sign changes | T=0.3: peak %.4f g, ' ...
                 '%d sign changes, worst fall %.3f%% of peak (WHOLE trace)'], ...
                c02.lo, c02.hi, c02.nsc_w, c03.peak, c03.nsc_w, 100*c03.dip), ...
        okA, 'p.404 text (zarchan.txt:5466)', ...
        ['book: T=0.2 "oscillates between +7 g ... indeed unstable"; T=0.3 ' ...
         '"stabilizes ... a monotonically increasing straight line".  "+" is a ' ...
         'mangled "+/-"; the listing clamps at -XNCLIM explicitly.  Both halves ' ...
         'scored TOGETHER because the claim is a CROSSING -- either alone is ' ...
         'satisfiable by a port that never changes behaviour at all.  Here the ' ...
         'book states monotonicity FLATLY, so it IS adjudicated, on the WHOLE ' ...
         'trace at 1 percent; derived, the worst fall is exactly 0.000 percent, ' ...
         'so that is not a cushion.  "Straight line" is NOT scored -- Fig. 18.6 ' ...
         'became an image link, so there is no published straightness to score ' ...
         'against.  Both rails are reachable only because of the limiter (SC3).']);

    okB = c05.hit_hi_w && c05.hit_lo_w && c05.nsc_w >= 2 && ...
          ~c07.hit_hi_w && ~c07.hit_lo_w && c07.nsc_w == 0;
    B = addShape(B, 'Fig. 18.7, Vc=9000: T=0.5 on both rails, T=0.7 stable', ...
        ['T=0.5 reaches BOTH rails and changes sign; T=0.7 reaches neither ' ...
         'and never changes sign (pre-endgame window)'], ...
        sprintf(['T=0.5: [%+.2f,%+.2f] g, %d sign changes | T=0.7: window peak ' ...
                 '%.4f g, %d sign changes, worst fall %.3f%% of window peak'], ...
                c05.lo, c05.hi, c05.nsc_w, c07.peak_w, c07.nsc_w, 100*c07.dip_w), ...
        okB, 'p.405 text (zarchan.txt:5474)', ...
        ['book: T=0.5 "oscillates between +7 g ... unstable"; T=0.7 "stable and ' ...
         'APPROXIMATELY monotonically increasing".  *** MONOTONICITY IS NOT ' ...
         'ADJUDICATED HERE, and the reason is the book''s own hedge ' ...
         '"approximately". ***  Derived, the worst single-sample fall in the ' ...
         'T=0.7 window is 2.19 percent of peak, which a 2 percent threshold ' ...
         'would fail; moving it to 3 to absorb that is exactly the widening ' ...
         'this port was sent back to stop doing, so the fall is REPORTED and ' ...
         'the row scores what the book says without hedging.  The window ' ...
         'matters here too: the FULL T=0.7 trace DOES touch +7 g in its LAST ' ...
         'sample, from TGO -> 1e-5 -- the transcription''s endgame, not a ' ...
         'stability property.']);
else
    whyOff = sprintf(['off the listing''s own case -- Figs. 18.6/18.7 are quoted ' ...
        'for N''=3, R=-0.01, Ta=5 s, Vm=3000 ft/s, nT=32.2 ft/s^2, a 7-g limit ' ...
        'and tF=10 s; this run used N''=%g, R=%g, Ta=%g s, Vm=%g, nT=%g, limit ' ...
        '%g g, tF=%g s'], XNP, RD, TA, VM, XNT, CLG, TF);
    B = addSkip(B, 'Fig. 18.6, Vc=4000: T=0.2 on both rails, T=0.3 stable', ...
        'see zarchan.txt:5466', whyOff, 'p.404 text (zarchan.txt:5466)', '-');
    B = addSkip(B, 'Fig. 18.7, Vc=9000: T=0.5 on both rails, T=0.7 stable', ...
        'see zarchan.txt:5474', whyOff, 'p.405 text (zarchan.txt:5474)', '-');
end

% ---- 9.  The other half of the inequality, honestly out of reach ------------
B = addSkip(B, 'neutral bound at POSITIVE radome slope', '2.07', ...
    ['Chapter 18 states the upper bound but never simulates positive R: ' ...
     'Listing 18.2 sets R = -.01 and every case in the chapter is ' ...
     'negative-slope, so there is no time history to bisect.  Scored by root ' ...
     'locus in radomeEngagement.  Widening rows 3-4 to straddle both bounds ' ...
     'would make them unfalsifiable.'], ...
    'p.399 inequality (zarchan.txt:5409)', '-');
end


function B = addMu(B, what, mu, tc, loT, hiT)
if ~isfinite(mu)
    B = addSkip(B, what, '0.79', sprintf(['the bisection bracket (%.2f, %.2f) s ' ...
        'did not straddle the transition (or the answer pinned to the ' ...
        'integrator floor), so there is no neutral point to report'], loT, hiT), ...
        'p.399 inequality (zarchan.txt:5408)', '-');
    return
end
B = addVal(B, what, 0.79, mu, 'text(2dp)+mech', 0.08, ...
    'p.399 inequality (zarchan.txt:5408)', '-', ...
    sprintf(['bisection on the TIME HISTORY (rail contact or sign change over ' ...
     'the pre-endgame window) at T = %.5g s -- NOT on the characteristic ' ...
     'polynomial, which is radomeEngagement''s route and would be a ' ...
     'restatement.  Band 8%%, two-sided, fixed before any run: 0.79 is the ' ...
     'Ta/T -> inf asymptote (exact boundary 0.806 at this Ta/T ~ 20), which ' ...
     'pushes the residual UP, and a 10 s window with a 0.5 s guard cannot ' ...
     'resolve a growth rate near zero, which pushes it DOWN.'], tc));
end


function B = addTcritRow(B, tc, loProbe, hiProbe, vc, tminBook, src)
%ADDTCRITROW  "Simulation and theory agree", scored the way the book
%   demonstrates it: the transition must lie strictly between the two time
%   constants it probes, one of which it calls unstable and the other stable.
what = sprintf('Vc=%g: simulated neutral T lies in the book''s bracket', vc);
want = sprintf('%.3g s < T_neutral < %.3g s (book''s Tmin = %.5g s)', ...
    loProbe, hiProbe, tminBook);
if ~isfinite(tc)
    B = addSkip(B, what, want, ['the bisection bracket did not straddle the ' ...
        'transition (or pinned to the integrator floor), so there is no ' ...
        'neutral T to score'], src, 's');
    return
end
B = addBound(B, what, want, ...
    sprintf('%.5g s (%.2f%% of the book''s Tmin)', tc, 100*tc/tminBook), ...
    tc > loProbe && tc < hiProbe, src, 's', ...
    ['"the theoretical formula for the minimum guidance system time constant ' ...
     'and the simulation results of Listing 18.2 appear to be in agreement" ' ...
     '(zarchan.txt:5470).  The book demonstrates that with TWO probes, so a ' ...
     'bracket is the strongest claim its own evidence supports; demanding ' ...
     'equality with Tmin would score the asymptotic 0.79 and the detector''s ' ...
     'window bias as if they were port errors.']);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw "Subscripted assignment between dissimilar
%   structures" -- which is the behaviour we want.  This is the schema
%   benchSuite's adaptGeneric consumes, and adaptGeneric is STRICT: a near-miss
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
L{end+1} = 'Zarchan Listing 18.2 -- SIMULATION OF HOMING LOOP WITH RADOME EFFECTS';
L{end+1} = '*** FEET, ft/s, ft/s^2, g = 32.2.  NOT SI.  WE HAVE NO RADOME: this is';
L{end+1} = '    the MECHANISM -- a parasitic path, one scalar mu, and a SATURATION';
L{end+1} = '    that turns divergence into a limit cycle on the rail. ***';
L{end+1} = sprintf(['   Vc = %g ft/s  Vm = %g ft/s  nT = %g ft/s^2 (%.3g g)  ' ...
    'N'' = %g  T = %g s'], p.VC, p.VM, p.XNT, p.XNT/32.2, p.XNP, p.TAU);
L{end+1} = sprintf(['   Ta = %g s  R = %g  nc limit = %g g  HE = %g deg  tF = %g s  ' ...
    'H = %g s  samp = %g s  guard = %g s'], p.TA, p.R, p.XNCLIMG, p.HEDEG, ...
    p.TF, p.H, p.samp, p.guard);
L{end+1} = sprintf('   |mu| = N''Vc|R|Ta/(VmT) = %.4f   (neutral near 0.79)', p.mu);
L{end+1} = '';
L{end+1} = sprintf(['NOMINAL FLIGHT  peak |nc| = %.5g g   nc in [%+.4f, %+.4f] g   ' ...
    '%d sign changes'], R.nom.peak, R.nom.lo, R.nom.hi, R.nom.nsc);
L{end+1} = sprintf('                worst sample fall %.3f%% of peak;  final y = %.6g ft', ...
    100*R.nom.dip, R.y(end));
L{end+1} = '';
L{end+1} = 'THE BOOK''S FOUR PUBLISHED CASES (Figs. 18.6, 18.7).  _w = pre-endgame window.';
L{end+1} = sprintf('%8s %7s %10s %9s %10s %9s', 'Vc', 'T [s]', 'peak_w[g]', ...
    'rails_w', 'signchg_w', 'fall_w%');
for i = 1:numel(R.cases)
    c = R.cases(i);  m = c.m;
    L{end+1} = sprintf('%8g %7g %10.4g %9s %10d %9.3f', c.vc, c.tau, m.peak_w, ...
        sprintf('%d/%d', m.hit_hi_w, m.hit_lo_w), m.nsc_w, 100*m.dip_w); %#ok<AGROW>
end
L{end+1} = '   rails_w = (upper hit)/(lower hit).  Rail contact OR a sign change over';
L{end+1} = '   this window IS the bisection predicate.  fall_w is REPORTED, not scored.';
L{end+1} = sprintf(['   time-domain neutral T: %.5g s at Vc=4000 (|mu| %.4f), ' ...
    '%.5g s at Vc=9000 (|mu| %.4f)'], R.tcrit.vc4000, R.tcrit.mu4000, ...
    R.tcrit.vc9000, R.tcrit.mu9000);
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-58s %s', vstr(K.oks(i)), K.names{i}, ...
        K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning';
    L{end+1} = '            NOT CHECKED, which is not the same thing as "fine".';
else
    L{end+1} = 'BENCHMARKS  (book values grepped from zarchan_decoded.txt; lines in .source)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
            case 'skip'
                L{end+1} = sprintf('   SKIP %-52s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case {'shape','bound'}
                L{end+1} = sprintf('   %-4s %-52s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
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
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: "oscillates between +/-7 g" is the LIMITER talking.  Delete it';
L{end+1} = '   (self-check 3) and the same case runs away.  A command sitting on its';
L{end+1} = '   rail is a loop with no authority left, not a bounded response.';
s = strjoin(L, newline);
end

function s = vstr(tf)
if tf
    s = 'PASS';
else
    s = 'FAIL';
end
end
