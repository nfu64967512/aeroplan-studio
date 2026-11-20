function R = thrustWeight(opts)
%MGS.ZARCHAN.THRUSTWEIGHT  Zarchan Listing 12.1 -- two-stage booster
%   thrust-weight computations, plus the 1-D burn integration that checks them.
%
%   R = mgs.zarchan.thrustWeight()
%   R = mgs.zarchan.thrustWeight('AMAX1', 20, 'AMAX2', 20, 'assert', false)
%   R = mgs.zarchan.thrustWeight('H', 0.001)          finer RK2 step
%
%   Port of Listing 12.1 (zarchan.txt:3708-3727, page furniture interleaved).
%   The listing does two things: it SIZES a two-stage booster from (Isp, mass
%   fraction, payload, desired dV, max axial g) via the rocket equation, then
%   INTEGRATES Vdot = g*T/W through the resulting thrust-weight schedule to
%   confirm the sizing really delivers the requested dV.  Both halves are here.
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: ft, ft/s, POUNDS FORCE, s, g = 32.2 ft/s^2.  Weights
%   are lbf, so T/W is dimensionless and IS the axial acceleration in g.
%   Nothing is converted -- the rest of mgs is metres and newtons, so convert
%   AT THE CALL SITE.  A silent lb->N would turn the book's 26,950 lb of
%   first-stage thrust into "119,880" and it would still read as a perfectly
%   reasonable booster.
% ===========================================================================
%
%   THE TRANSCRIPTION.  listing_12_01.m does not parse.  Its header reports ONE
%   unresolved variable-to-variable comparison; there are three OCR'd "," (this
%   book's "<") in total, all resolved by shape, none by taste:
%
%     1  "while T ,= (TB1+TB2)" -> "while T <= (TB1+TB2)".  T starts at 0 and
%        only increases, so ">=" never enters the loop at all and "==" never
%        fires.  RUN, self-check 5: the flipped form records 0 samples.
%     2  "if T,TB1" -> "if T < TB1".   <<< THE UNRESOLVED ONE.  Three-branch
%        chain, ASCENDING thresholds: branch 1 is the first-stage schedule
%        WGT = -WP1*T/TB1 + WTOT, meaningful only on [0, TB1] (it reaches
%        WS1+WTOT2 at TB1 and goes NEGATIVE past 57.2 s), so ">" would use it
%        exactly where it is invalid.  RUN, self-check 4: the flipped form
%        returns a burnout velocity near 14,855 ft/s against 20,000 -- a 26%
%        error, from a run that completes smoothly and plots without complaint.
%     3  "elseif(T,(TB1+TB2))" -> "elseif T < (TB1+TB2)".  Same chain; the
%        "else" arm is coast (TRST = 0, WGT = WPAY), i.e. the AFTER-burnout
%        case, so the elseif must be the BEFORE-burnout test.
%
%   Dropped as page furniture, not "fixed" into code: "BOOSTERS 265" (running
%   header, :3723) and the trailing "GRAVITY TURN [1]", the next SECTION
%   HEADING (:3728).  "save datfil.txt output /ascii" is a typo for -ascii;
%   nothing is written to disk here at all.  "if S >= .99999" with H = .01
%   records once a second; this port tests against 'samp' instead so that a
%   different sample interval does not silently keep the 1 s literal.
%
%   BENCHMARKS -- every book value grepped out of zarchan_decoded.txt, line
%   number in the row's .source.  Nothing read off a figure (the extraction
%   turned every figure into an image link).
%
%     1-9  TABLE 12.1 "SIMULATION OUTPUTS", zarchan.txt:3689-3699.  BAND = HALF
%          THE BOOK'S OWN LAST PRINTED DIGIT: +/-0.5 lb on an integer weight,
%          +/-0.05 s on "32.2 s".  Tightest band the publication supports, and
%          not loose -- T1 lands +0.48 lb inside +/-0.5 lb, 96% of the band,
%          because "26,950" is quoted with a trailing zero and this port does
%          NOT read that as +/-5.  If T1 reddens, check the rounding rule first.
%    10-11 The integrator, zarchan.txt:3702.  Scored against the program's OWN
%          DELV and DELV1, not against the prose "20 kft/s"/"one-third":
%          with linear weight flow and T = Isp*wdot, integrating g*T/W gives
%          g*Isp*ln(W0/Wf), which IS the rocket equation the stage was sized
%          from, so the identity is exact and the whole residual is RK2
%          discretization.  Band 0.05%; expect about 0.001%.
%      12  zarchan.txt:3702: acceleration "varied between 4 g and 10 g during
%          the first-stage burn and ... between 2 g and 10 g during the
%          second-stage burn".  BOUND row, +/-0.5 g (book quotes 1 sig fig).
%    INFO  zarchan.txt:1929-1936 (Ch. 6, thrust vector control): T/W =
%          (nL/g)/alpha = 5/(5/57.3) = 57.3, then Talpha = VM/((T/W)g) =
%          20,000/(57.3*32.2) = 10.8 s.  The only worked ARITHMETIC on a
%          thrust-to-weight ratio in the decoded maths region -- but a different
%          chapter and a different mechanism (lateral TVC, not staging), so
%          REPORTED, NOT SCORED.  It is the book confirming in numbers the
%          identity a/g = T/W that "A = 32.2*TRST/WGT" rests on.
%
%   WHAT THIS SHOULD PRINT.  *** THIS FILE HAS NEVER BEEN EXECUTED -- MATLAB is
%   the caller's, not this author's.  The numbers below come from running the
%   SAME algorithm in a throwaway Python re-implementation, so treat them as an
%   expectation to compare a first run against, NOT as this port's output. ***
%       Wtot 6168.72  Wp1 3473.67  Ws1 613.00  Wp2 1684.74  Ws2 297.31 lb
%       T1 26950.48 lb  tB1 32.2227 s   T2 3973.07 lb  tB2 106.0100 s
%       V(tB1) 6665.14 ft/s   V(tB1+tB2) 19999.75 ft/s   burnout 138.23 s
%       stage-1 a 4.370 -> 9.999 g      stage-2 a 1.908 -> 9.999 g
%   Do not widen a band to make a row green.  A red row is a result.
%
%   OUTPUTS: R.p (sizing + parameters used), R.t/R.v/R.a_g/R.w/R.trst (full-rate
%   burn history), R.rec (the listing's own 1 s record), R.bench, R.selfcheck
%   (.names/.oks, including two DELETE experiments), R.pass (true only if a row
%   was SCORED and all passed), R.verdict (always formed; 'assert' picks error
%   vs warn and nothing else -- with assert false R comes back complete).
%
%   See also MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 12.1 line 1, verbatim.  Pounds, seconds, ft/s.
    opts.XISP1 (1,1) double {mustBeFinite, mustBePositive} = 250     % s
    opts.XISP2 (1,1) double {mustBeFinite, mustBePositive} = 250     % s
    opts.XMF1  (1,1) double {mustBeFinite, mustBePositive} = 0.85    % mf1*
    opts.XMF2  (1,1) double {mustBeFinite, mustBePositive} = 0.85    % mf2*
    opts.WPAY  (1,1) double {mustBeFinite, mustBePositive} = 100     % lb
    opts.DELV  (1,1) double {mustBeFinite, mustBePositive} = 20000   % ft/s
    % The listing writes .3333 and .6667, NOT 1/3 and 2/3.  Kept literal: they
    % sum to exactly 1.0000, so the burnout velocity target is DELV on the nose.
    opts.f1    (1,1) double {mustBeFinite, mustBePositive} = 0.3333
    opts.f2    (1,1) double {mustBeFinite, mustBePositive} = 0.6667
    opts.AMAX1 (1,1) double {mustBeFinite, mustBePositive} = 10      % g
    opts.AMAX2 (1,1) double {mustBeFinite, mustBePositive} = 10      % g
    opts.H     (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s, RK2
    opts.samp  (1,1) double {mustBeFinite, mustBePositive} = 1.0     % s, record
    opts.G     (1,1) double {mustBeFinite, mustBePositive} = 32.2    % ft/s^2
    % THE ONLY SWITCHES.  Not nargin: with an arguments block that declares only
    % name-value options MATLAB reports nargin = 0 however many were passed, so
    % an "if nargin == 0" here would be a mode that never fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

if opts.XMF1 >= 1 || opts.XMF2 >= 1
    error('mgs:zarchan:thrustWeight:mf', ...
        ['approximate fuel mass fraction must be < 1 (got %g, %g).  At 1 the ' ...
         'structural weight Wp*(1-mf)/mf is zero and the sizing degenerates.'], ...
        opts.XMF1, opts.XMF2);
end
if opts.H >= opts.samp
    error('mgs:zarchan:thrustWeight:samp', ...
        'H = %g s must be smaller than samp = %g s.', opts.H, opts.samp);
end

P = sizing(opts, opts.G);

if opts.H >= 0.01*min(P.TB1, P.TB2)
    error('mgs:zarchan:thrustWeight:step', ...
        ['H = %g s gives fewer than 100 steps in a burn phase (tB1 = %g s, ' ...
         'tB2 = %g s).  RK2 does not blow up here, it returns a smooth and ' ...
         'wrong velocity, so this is refused rather than warned about.'], ...
        opts.H, P.TB1, P.TB2);
end

% =========================================================================
%   LISTING 12.1, integration half.  zarchan.txt:3713-3727.
% =========================================================================
[t, v, a_g, w, trst, rec] = boostRun(P, opts.H, opts.samp, opts.G, +1, +1);
if numel(t) < 2
    error('mgs:zarchan:thrustWeight:empty', ...
        'the burn recorded %d sample(s); at least two are needed.', numel(t));
end

R = struct();
R.t = t;  R.v = v;  R.a_g = a_g;  R.w = w;  R.trst = trst;
R.rec = rec;
R.p = P;
R.p.H = opts.H;  R.p.samp = opts.samp;  R.p.G = opts.G;
R.p.units = 'ft, ft/s, lbf, s; a in g with g = 32.2 ft/s^2';

% Quantities the benchmarks and the render both need, formed once.
i1 = find(t <= P.TB1, 1, 'last');
i2 = find(t <= P.TB1 + P.TB2, 1, 'last');
R.v_stage1_end = v(i1);
R.v_burnout    = v(i2);
s1 = t <  P.TB1;
s2 = t >= P.TB1 & t < (P.TB1 + P.TB2);
R.a1_min = min(a_g(s1));  R.a1_max = max(a_g(s1));
R.a2_min = min(a_g(s2));  R.a2_max = max(a_g(s2));

R.selfcheck = structuralChecks(R, P, opts);

if opts.benchmark
    R.bench = benchmarks(R, P);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine".
    R.pass  = false;
end

R.summary = render(R, P, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R); end

msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the book.  ' ...
           'R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = ['not one book value could be evaluated; an unvalidated port must ' ...
           'not be handed back silently.'];
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question.  ' ...
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
        error('mgs:zarchan:thrustWeight:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN.  R comes back complete so the caller can see WHICH
        % side of a comparison moved; benchSuite runs every port with assert
        % off for exactly that reason.
        warning('mgs:zarchan:thrustWeight:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function P = sizing(o, G)
%SIZING  Listing 12.1 lines 1-2, zarchan.txt:3708, :3713.
%   Every constant arrives as an argument: MATLAB local functions cannot see the
%   caller's workspace.

DELV1 = o.f1*o.DELV;
DELV2 = o.f2*o.DELV;

% ---- second stage first: stage 1 treats "stage 2 + payload" as its payload ---
E2   = exp(DELV2/(o.XISP2*G));
TOP2 = o.WPAY*(E2 - 1);
BOT2 = 1/o.XMF2 - ((1 - o.XMF2)/o.XMF2)*E2;
mustBeFeasible(BOT2, DELV2, o.XISP2, o.XMF2, G, 2);
WP2   = TOP2/BOT2;
WS2   = WP2*(1 - o.XMF2)/o.XMF2;
WTOT2 = WP2 + WS2 + o.WPAY;
TRST2 = o.AMAX2*(o.WPAY + WS2);       % thrust set by the g limit AT BURNOUT,
TB2   = o.XISP2*WP2/TRST2;            % where the stage is lightest

% ---- first stage -------------------------------------------------------------
E1   = exp(DELV1/(o.XISP1*G));
TOP1 = WTOT2*(E1 - 1);
BOT1 = 1/o.XMF1 - ((1 - o.XMF1)/o.XMF1)*E1;
mustBeFeasible(BOT1, DELV1, o.XISP1, o.XMF1, G, 1);
WP1   = TOP1/BOT1;
WS1   = WP1*(1 - o.XMF1)/o.XMF1;
WTOT  = WP1 + WS1 + WTOT2;
TRST1 = o.AMAX1*(WTOT2 + WS1);
TB1   = o.XISP1*WP1/TRST1;

P = struct('DELV',o.DELV, 'DELV1',DELV1, 'DELV2',DELV2, ...
           'WPAY',o.WPAY, 'XISP1',o.XISP1, 'XISP2',o.XISP2, ...
           'XMF1',o.XMF1, 'XMF2',o.XMF2, 'AMAX1',o.AMAX1, 'AMAX2',o.AMAX2, ...
           'WP1',WP1, 'WS1',WS1, 'WP2',WP2, 'WS2',WS2, ...
           'WTOT',WTOT, 'WTOT2',WTOT2, ...
           'TRST1',TRST1, 'TRST2',TRST2, 'TB1',TB1, 'TB2',TB2);
end


% =============================================================================
function mustBeFeasible(BOT, DV, ISP, MF, G, stage)
%MUSTBEFEASIBLE  BOT = 0 is exactly the rocket-equation ceiling
%   dV = g*Isp*ln(1/(1-mf)).  Past it BOT goes NEGATIVE and the propellant
%   weight comes back negative -- whereupon every downstream weight, thrust and
%   burn time is a plausible-looking finite number of the wrong sign.  Refused,
%   because there is no other symptom.
if BOT <= 0
    error('mgs:zarchan:thrustWeight:infeasible', ...
        ['stage %d: dV = %g ft/s exceeds the rocket-equation ceiling ' ...
         'g*Isp*ln(1/(1-mf)) = %g ft/s at Isp = %g s, mf = %g.  The propellant ' ...
         'weight would come back NEGATIVE.'], ...
        stage, DV, G*ISP*log(1/(1-MF)), ISP, MF);
end
end


% =============================================================================
function [t, v, a_g, w, trst, rec] = boostRun(P, H, samp, G, stageCmp, loopCmp)
%BOOSTRUN  Listing 12.1, zarchan.txt:3713-3727.  Zarchan's two-pass RK2.
%
%   stageCmp  +1 = the book-resolved "if T < TB1";  -1 = the flipped "T > TB1".
%   loopCmp   +1 = the book-resolved "while T <= TB";  -1 = the flipped ">=".
%   Both alternatives are RUN, by self-checks 4 and 5, so the resolution is
%   demonstrated rather than asserted.

TB = P.TB1 + P.TB2;
nmax = ceil(TB/H) + 8;
t = zeros(nmax,1);  v = zeros(nmax,1);  a_g = zeros(nmax,1);
w = zeros(nmax,1);  trst = zeros(nmax,1);
rt = zeros(nmax,1); rvk = zeros(nmax,1); rag = zeros(nmax,1);

T = 0.0;  V = 0.0;  S = 0.0;  A = 0.0;  WGT = P.WTOT;  TRST = P.TRST1;
n = 0;  m = 0;

while loopOn(T, TB, loopCmp)
    VOLD = V;  STEP = 1;  FLAG = 0;

    % Zarchan's predictor loop.  Pass 1 evaluates A at the current T; pass 2
    % takes the Euler step, advances T by H, and re-evaluates there.  T advances
    % INSIDE this loop, exactly once -- move that line out and the scheme
    % quietly degrades to first order while still running smoothly.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            V = V + H*A;
            T = T + H;
        end
        [WGT, TRST] = schedule(T, P, stageCmp);
        A = G*TRST/WGT;
        FLAG = 1;
    end
    % The listing writes "FLAG = 0" here too; the outer loop re-initialises it,
    % so that assignment cannot be read.  Kept as a comment, not as code, so
    % nobody re-adds it thinking a state machine was lost.

    V = (VOLD + V)/2 + 0.5*H*A;

    n = n + 1;
    t(n) = T;  v(n) = V;  a_g(n) = A/G;  w(n) = WGT;  trst(n) = TRST;

    % Only the DELETE experiment can reach a non-finite state here (the flipped
    % stage test drives WGT through zero).  Bail rather than spin.
    if ~isfinite(V) || abs(V) > 1e12
        break
    end

    % ONE increment, tested against samp.  The listing increments S once and
    % tests "S >= .99999" with H = .01, i.e. records every 100th step; keeping
    % the literal would silently change meaning for any other H.
    S = S + H;
    if S >= samp - 0.5*H
        S = 0.0;
        m = m + 1;
        rt(m) = T;  rvk(m) = V/1000;  rag(m) = A/G;
    end
end

t = t(1:n);  v = v(1:n);  a_g = a_g(1:n);  w = w(1:n);  trst = trst(1:n);
rec = struct('t', rt(1:m), 'vk', rvk(1:m), 'ag', rag(1:m));
end


% =============================================================================
function on = loopOn(T, TB, loopCmp)
%LOOPON  "while T <= (TB1+TB2)" and its refuted alternative.
if loopCmp > 0
    on = T <= TB;
else
    on = T >= TB;      % refuted: T starts at 0, so this never enters at all
end
end


% =============================================================================
function [WGT, TRST] = schedule(T, P, stageCmp)
%SCHEDULE  The listing's three-branch thrust-weight profile, Fig. 12.7.
%   Linear weight burn-down within each stage; the structural weight of stage 1
%   disappears at staging.
if stageCmp > 0
    firstStage = T < P.TB1;
else
    firstStage = T > P.TB1;    % refuted: see self-check 4
end

if firstStage
    WGT  = -P.WP1*T/P.TB1 + P.WTOT;
    TRST = P.TRST1;
elseif T < (P.TB1 + P.TB2)
    WGT  = -P.WP2*T/P.TB2 + P.WTOT2 + P.WP2*P.TB1/P.TB2;
    TRST = P.TRST2;
else
    WGT  = P.WPAY;
    TRST = 0.0;
end
end


% =============================================================================
function K = structuralChecks(R, P, o)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never evidence that
%   this port reproduces Zarchan -- only that it agrees with itself.  Checks 4
%   and 5 are DELETE EXPERIMENTS: they re-run the listing with the OCR'd
%   comparisons resolved the other way and require the answer to break.
%
%   .names and .oks are what put these on the board.  A port that computes
%   invariants and stores them under bespoke field names shows ZERO structural
%   rows and looks green.
K = struct();
names = {};  oks = [];  detail = {};

% 1  the weight ledger closes, and each stage's structural fraction is 1-mf
led = abs(P.WTOT - (P.WP1 + P.WS1 + P.WP2 + P.WS2 + P.WPAY));
f1  = abs(P.WS1/(P.WP1 + P.WS1) - (1 - P.XMF1));
f2  = abs(P.WS2/(P.WP2 + P.WS2) - (1 - P.XMF2));
names{end+1}  = 'weight ledger closes and Ws/(Wp+Ws) = 1-mf';
oks(end+1)    = led <= 1e-9*P.WTOT && f1 <= 1e-12 && f2 <= 1e-12;
detail{end+1} = sprintf('ledger %.3e lb, mf residuals %.2e / %.2e', led, f1, f2);

% 2  each stage was sized BY the rocket equation, so it must satisfy it
r1 = o.G*P.XISP1*log(P.WTOT /(P.WTOT  - P.WP1)) - P.DELV1;
r2 = o.G*P.XISP2*log(P.WTOT2/(P.WTOT2 - P.WP2)) - P.DELV2;
names{end+1}  = 'rocket equation closes on each stage';
oks(end+1)    = abs(r1) <= 1e-8*P.DELV1 && abs(r2) <= 1e-8*P.DELV2;
detail{end+1} = sprintf('dV residual %+.3e / %+.3e ft/s', r1, r2);

% 3  the schedule: staging drops exactly Ws1, burnout leaves Ws2+Wpay, weight
%    never rises inside a stage, velocity never falls
eps0 = 1e-9;
wA = schedule(P.TB1 - eps0, P, +1);
wB = schedule(P.TB1 + eps0, P, +1);
wC = schedule(P.TB1 + P.TB2 - eps0, P, +1);
drop = abs((wA - wB) - P.WS1);
endw = abs(wC - (P.WS2 + P.WPAY));
inS1 = R.t <  P.TB1;
inS2 = R.t >= P.TB1 & R.t < (P.TB1 + P.TB2);
% -inf is a FLOOR, not a fudge: it keeps mono a scalar when a phase happens to
% hold fewer than two samples, so the && below cannot be handed an empty.
mono = max([reshape(diff(R.w(inS1)),[],1); reshape(diff(R.w(inS2)),[],1); -inf]);
vmon = min(diff(R.v));
names{end+1}  = 'staging drop = Ws1, burnout weight = Ws2+Wpay, W down / V up';
oks(end+1)    = drop <= 1e-6*P.WS1 && endw <= 1e-6*(P.WS2 + P.WPAY) && ...
                mono <= 1e-9 && vmon >= -1e-9;
detail{end+1} = sprintf(['drop err %.2e lb, burnout err %.2e lb, ' ...
                         'max dW %+.2e lb, min dV %+.2e ft/s'], ...
                        drop, endw, mono, vmon);

% 4  DELETE EXPERIMENT: resolve "if T,TB1" the other way round
[~, vF] = boostRun(P, o.H, o.samp, o.G, -1, +1);
if isempty(vF)
    vFend = NaN;
else
    vFend = vF(end);
end
brk = ~isfinite(vFend) || abs(vFend - P.DELV)/P.DELV > 0.05;
names{end+1}  = 'DELETE: "T > TB1" instead of "T < TB1" breaks burnout velocity';
oks(end+1)    = brk;
detail{end+1} = sprintf('flipped burnout V = %.1f ft/s vs %.1f (%.1f%% off)', ...
                        vFend, P.DELV, 100*abs(vFend - P.DELV)/P.DELV);

% 5  DELETE EXPERIMENT: resolve "while T ,= TB" the other way round
tG = boostRun(P, o.H, o.samp, o.G, +1, -1);
names{end+1}  = 'DELETE: "while T >= TB" instead of "<=" records nothing';
oks(end+1)    = isempty(tG);
detail{end+1} = sprintf('flipped loop recorded %d samples', numel(tG));

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
function B = benchmarks(R, P)
%BENCHMARKS  Every row built through ONE constructor so the field order cannot
%   drift.  Table 12.1 rows carry a band of HALF THE BOOK'S LAST PRINTED DIGIT.
tab = 'Table 12.1, p.263 (zarchan.txt:%d)';

B = mkrow('Wtot  total interceptor weight', 'value', 6169, '6169 lb', ...
        P.WTOT, sprintf('%.2f lb', P.WTOT), ...
        '+/-0.5 lb (half the printed digit)', 0.5, ...
        abs(P.WTOT - 6169) <= 0.5, false, '', sprintf(tab,3691), 'lb', ...
        'also stated in prose at zarchan.txt:3684');

B(end+1) = mkrow('Wp1  first-stage propellant weight', 'value', 3474, '3474 lb', ...
        P.WP1, sprintf('%.2f lb', P.WP1), '+/-0.5 lb', 0.5, ...
        abs(P.WP1 - 3474) <= 0.5, false, '', sprintf(tab,3692), 'lb', '');

B(end+1) = mkrow('Ws1  first-stage structural weight', 'value', 613, '613 lb', ...
        P.WS1, sprintf('%.2f lb', P.WS1), '+/-0.5 lb', 0.5, ...
        abs(P.WS1 - 613) <= 0.5, false, '', sprintf(tab,3693), 'lb', '');

B(end+1) = mkrow('Wp2  second-stage propellant weight', 'value', 1685, '1685 lb', ...
        P.WP2, sprintf('%.2f lb', P.WP2), '+/-0.5 lb', 0.5, ...
        abs(P.WP2 - 1685) <= 0.5, false, '', sprintf(tab,3694), 'lb', '');

B(end+1) = mkrow('Ws2  second-stage structural weight', 'value', 297, '297 lb', ...
        P.WS2, sprintf('%.2f lb', P.WS2), '+/-0.5 lb', 0.5, ...
        abs(P.WS2 - 297) <= 0.5, false, '', sprintf(tab,3695), 'lb', ...
        'the widest of the nine in relative terms (about +0.10%)');

B(end+1) = mkrow('T1  thrust level of first stage', 'value', 26950, '26,950 lb', ...
        P.TRST1, sprintf('%.2f lb', P.TRST1), '+/-0.5 lb', 0.5, ...
        abs(P.TRST1 - 26950) <= 0.5, false, '', sprintf(tab,3696), 'lb', ...
        ['"26,950" has a trailing zero so +/-5 would be defensible; NOT used -- ' ...
         'the row is already inside +/-0.5, and widening a green band is how ' ...
         'bands stop meaning anything']);

B(end+1) = mkrow('tB1  thrust burn time of first stage', 'value', 32.2, '32.2 s', ...
        P.TB1, sprintf('%.4f s', P.TB1), '+/-0.05 s', 0.05, ...
        abs(P.TB1 - 32.2) <= 0.05, false, '', sprintf(tab,3697), 's', ...
        'the table prints the symbol as "lB1"; an OCR slip for tB1');

B(end+1) = mkrow('T2  thrust level of second stage', 'value', 3973, '3973 lb', ...
        P.TRST2, sprintf('%.2f lb', P.TRST2), '+/-0.5 lb', 0.5, ...
        abs(P.TRST2 - 3973) <= 0.5, false, '', sprintf(tab,3698), 'lb', '');

B(end+1) = mkrow('tB2  thrust burn time of second stage', 'value', 106, '106 s', ...
        P.TB2, sprintf('%.4f s', P.TB2), '+/-0.5 s', 0.5, ...
        abs(P.TB2 - 106) <= 0.5, false, '', sprintf(tab,3699), 's', ...
        sprintf(['burnout at tB1+tB2 = %.2f s; the book puts the second weight ' ...
                 'drop at 138.2 s (zarchan.txt:3685)'], P.TB1 + P.TB2));

% --- the integrator ---------------------------------------------------------
tol1 = 5e-4*P.DELV1;
B(end+1) = mkrow('V at end of first-stage burn = one-third of dV', 'value', ...
        P.DELV1, sprintf('%.1f ft/s', P.DELV1), ...
        R.v_stage1_end, sprintf('%.2f ft/s', R.v_stage1_end), ...
        '0.05% (RK2 discretization)', tol1, ...
        abs(R.v_stage1_end - P.DELV1) <= tol1, false, '', ...
        'Fig. 12.8 prose, p.264 (zarchan.txt:3702)', 'ft/s', ...
        'scored against the program''s own DELV1 = f1*DELV; see header');

tol2 = 5e-4*P.DELV;
B(end+1) = mkrow('V at second-stage burnout = 20 kft/s goal', 'value', ...
        P.DELV, sprintf('%.1f ft/s', P.DELV), ...
        R.v_burnout, sprintf('%.2f ft/s', R.v_burnout), ...
        '0.05% (RK2 discretization)', tol2, ...
        abs(R.v_burnout - P.DELV) <= tol2, false, '', ...
        'Fig. 12.8 prose, p.264 (zarchan.txt:3702)', 'ft/s', ...
        ['the sample at the last T <= tB1+tB2; the loop takes one further step ' ...
         'past burnout, worth about 0.5*H*a_final = 1.6 ft/s']);

% --- the acceleration envelope ----------------------------------------------
dev = max([abs(R.a1_min - 4), abs(R.a1_max - 10), ...
           abs(R.a2_min - 2), abs(R.a2_max - 10)]);
B(end+1) = mkrow('axial acceleration envelope per stage', 'bound', ...
        NaN, '4-10 g (stage 1), 2-10 g (stage 2)', ...
        dev, sprintf('%.3f-%.3f g / %.3f-%.3f g (worst dev %.3f g)', ...
                     R.a1_min, R.a1_max, R.a2_min, R.a2_max, dev), ...
        '+/-0.5 g (the book quotes 1 significant figure)', 0.5, ...
        dev <= 0.5, false, '', 'Fig. 12.8 prose, p.264 (zarchan.txt:3702)', 'g', ...
        ['the maxima are AMAX1/AMAX2 by construction (thrust is set from the ' ...
         'burnout weight), so the two minima are the free numbers; the ' ...
         'stage-1 minimum is corroborated at zarchan.txt:3752, "about 4 g at ' ...
         'the beginning"']);

% --- info -------------------------------------------------------------------
tw   = 5/(5/57.3);
ttvc = 20000/(tw*32.2);
B(end+1) = mkrow('Ch.6 worked T/W arithmetic (NOT this listing)', 'info', ...
        57.3, '57.3', tw, sprintf('%.4f', tw), 'n/a', NaN, true, true, ...
        ['INFO -- different chapter and different mechanism (lateral thrust ' ...
         'vector control, not staging); reported, not adjudicated'], ...
        'Ch.6 p.132-133 (zarchan.txt:1929-1936)', 'dimensionless', ...
        sprintf(['T/W = (nL/g)/alpha = 5/(5/57.3) = 57.3; Talpha = VM/((T/W)g) ' ...
                 '= 20,000/(57.3*32.2) = 10.8 s, ours %.3f s.'], ttvc));
end


% =============================================================================
function r = mkrow(what, kind, book, book_str, ours, ours_str, band, tol, ...
                   ok, skipped, why, source, units, detail)
%MKROW  THE ONE CONSTRUCTOR.  Field order is fixed here and nowhere else:
%   what kind book book_str ours ours_str pct band tol ok skipped why source
%   units detail.  benchSuite's adaptGeneric is strict about it.
r = struct();
r.what     = what;
r.kind     = kind;
r.book     = book;
r.book_str = book_str;
r.ours     = ours;
r.ours_str = ours_str;
if isfinite(book) && book ~= 0 && isfinite(ours)
    r.pct = 100*(ours - book)/book;
else
    r.pct = NaN;
end
r.band     = band;
r.tol      = tol;
r.ok       = logical(ok);
r.skipped  = logical(skipped);
r.why      = why;
r.source   = source;
r.units    = units;
r.detail   = detail;
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  A 0x0 struct array with EXACTLY the schema mkrow produces.
B = mkrow('', '', NaN, '', NaN, '', '', NaN, false, true, '', '', '', '');
B(1) = [];
end


% =============================================================================
function s = render(R, P, ev)
L = {};
L{end+1} = 'mgs.zarchan.thrustWeight -- Zarchan Listing 12.1, two-stage booster thrust-weight';
L{end+1} = sprintf('units: %s', R.p.units);
L{end+1} = sprintf(['inputs: Isp %g/%g s, mf* %g/%g, Wpay %g lb, dV %g ft/s ' ...
                    '(%g/%g split), amax %g/%g g, H %g s'], ...
                   P.XISP1, P.XISP2, P.XMF1, P.XMF2, P.WPAY, P.DELV, ...
                   P.DELV1/P.DELV, P.DELV2/P.DELV, P.AMAX1, P.AMAX2, R.p.H);
L{end+1} = '';
L{end+1} = sprintf('  Wtot %10.2f lb  Wp1 %9.2f  Ws1 %8.2f  Wp2 %9.2f  Ws2 %7.2f', ...
                   P.WTOT, P.WP1, P.WS1, P.WP2, P.WS2);
L{end+1} = sprintf('  T1   %10.2f lb  tB1 %9.4f s  T2 %8.2f lb  tB2 %9.4f s  burnout %.2f s', ...
                   P.TRST1, P.TB1, P.TRST2, P.TB2, P.TB1 + P.TB2);
L{end+1} = sprintf('  V(tB1) %8.2f ft/s  V(burnout) %10.2f ft/s', ...
                   R.v_stage1_end, R.v_burnout);
L{end+1} = sprintf('  a stage 1 %6.3f -> %6.3f g   a stage 2 %6.3f -> %6.3f g', ...
                   R.a1_min, R.a1_max, R.a2_min, R.a2_max);
L{end+1} = '';
L{end+1} = 'BENCHMARKS (band = half the book''s last printed digit unless noted)';
for k = 1:numel(R.bench)
    b = R.bench(k);
    if b.skipped
        mark = 'INFO';
    elseif b.ok
        mark = ' ok ';
    else
        mark = 'FAIL';
    end
    if isnan(b.pct)
        pc = '       ';
    else
        pc = sprintf('%+6.3f%%', b.pct);
    end
    L{end+1} = sprintf('  [%s] %-44s book %-12s ours %-30s %s', ...
                       mark, b.what, b.book_str, b.ours_str, pc); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = 'STRUCTURAL (self-consistency, NOT book values; 4 and 5 are delete experiments)';
for k = 1:numel(R.selfcheck.names)
    if R.selfcheck.oks(k)
        mark = ' ok ';
    else
        mark = 'FAIL';
    end
    L{end+1} = sprintf('  [%s] %-62s %s', mark, R.selfcheck.names{k}, ...
                       R.selfcheck.detail{k}); %#ok<AGROW>
end
L{end+1} = '';
if any(ev)
    L{end+1} = sprintf('scored %d row(s), %d passed; structural %d/%d', ...
                       sum(ev), sum([R.bench(ev).ok]), sum(R.selfcheck.oks), ...
                       numel(R.selfcheck.oks));
else
    L{end+1} = 'NOTHING SCORED.';
end
s = strjoin(L, newline);
end


% =============================================================================
function drawCurves(R)
%DRAWCURVES  The listing's two plots, on one figure.
figure;
subplot(2,1,1); plot(R.t, R.v/1000); grid on;
xlabel('Time (Sec)'); ylabel('Velocity (KFt/Sec)');
subplot(2,1,2); plot(R.t, R.a_g); grid on;
xlabel('Time (Sec)'); ylabel('Acceleration (G)');
end
