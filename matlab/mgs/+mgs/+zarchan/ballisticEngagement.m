function R = ballisticEngagement(opts)
%MGS.ZARCHAN.BALLISTICENGAGEMENT  Zarchan Listing 16.2 -- engagement between a
%   constant-speed interceptor and a decelerating ballistic target.
%
%   R = mgs.zarchan.ballisticEngagement()
%   R = mgs.zarchan.ballisticEngagement('XNCLIMG', 20, 'assert', false)
%
%   Line-by-line port of Listing 16.2 (zarchan.txt:4893-4927, including its
%   sub-function initialpz) on the book's own second-order Runge-Kutta (Heun)
%   scheme.  The target half IS Listing 16.1 -- same exponential atmosphere,
%   same beta drag, same flat-Earth g -- and is cross-checked against
%   MGS.ZARCHAN.BALLISTICTARGETSIM as a structural row, because the target
%   channel here is completely DECOUPLED from the missile and so must
%   reproduce the sibling port to rounding.
%
%   UNITS ARE THE BOOK'S: ft, ft/s, ft/s^2, g = 32.2, rho slug/ft^3, beta
%   lb/ft^2.  NOT SI.  Nothing is converted, so every number printed can be
%   laid straight against a published one; convert AT THE CALL SITE and say so
%   there.  Angles use the listing's own 57.3 deg/rad, not 180/pi.
%
%   *** WHAT THIS IS NOT FOR. ***  A Mach-6 re-entry vehicle against an
%   interceptor with no drag, no thrust and no autopilot lag.  No number here
%   belongs in a spread or terminal-speed argument.  What transfers is the
%   SHAPE: a guidance command that saturates against an acceleration limit
%   produces a large miss, and feeding the known target acceleration forward
%   (APN) removes the saturation rather than out-muscling it.  Our limit is an
%   airframe and an autopilot, not a 7-g literal.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_16_02.m reports TWO unresolved
%   variable-to-variable comparisons.  Both are shape-forced, not chosen:
%
%     1  "if XNC.XNCLIM" -> "if XNC > XNCLIM".  The next statement is
%        "if XNC < -XNCLIM, XNC = -XNCLIM", so the pair is a symmetric
%        saturation and only ">" completes it.  "<" would clamp XNC to +XNCLIM
%        whenever the command was BELOW the limit -- i.e. on nearly every step
%        -- flying a constant-turn missile that still terminates and still
%        prints a miss.
%     2  "while rt2.rt2des" -> "while rt2 > rt2des" (in initialpz).  The
%        routine marches the target DOWN from 200 kft to 50 kft, so it must run
%        while the target is above it.  "<" exits at t = 0 and the caller's
%        next line, VM = RTMF/TFDES, divides by zero.  Arithmetic-forced.
%
%   Three OCR substitutions were already resolved by the extractor and confirm
%   against the decoded text at zarchan.txt:4894-4925: "while VC.=0." -> ">= 0",
%   "if RTM,1000." -> "< 1000", "if RT2,=30000." -> "<= 30000".  "S >= .09999"
%   records every 0.1 s at the book's H; this port tests samp - H/2 so another
%   samp is honoured instead of a threshold frozen at the book's own step.
%   Nothing is written to disk ("save datfil.txt output /ascii" is the book's
%   typo for -ascii).  The listing's ZEMPLOS/ZEM1/ZEM2 initialisations are dead
%   and are dropped: they are the fossil of the predictive-guidance option the
%   prose promises ("Three interceptor guidance options appear",
%   zarchan.txt:4885) but the printed code, with two branches, does not have.
%   Reported, not invented back in.
%
%   THE MISS is the listing's bare trailing "RTM": the range from the LAST
%   derivative evaluation, at the PREDICTED state, not recomputed from the
%   corrected one.  Kept exactly so -- it is what the book's figures were made
%   from.
%
% ---------------------------------------------------------------------------
%   FIVE CASES, AND NO 'APN' OPTION.  The section's whole argument is a
%   COMPARISON, so APN is a case label here, not a parameter -- an 'APN' switch
%   would let a caller score the PN rows against an APN run and never notice.
%   Two further cases exist only to be DELETE EXPERIMENTS on the acceleration
%   limit, which is what turns the book's causal claim into something measured:
%
%     nominal   RM1 = 170,000 ft, PN     Figs. 16.12/16.13
%     stress    RM1 =  50,000 ft, PN     Figs. 16.14/16.15
%     stressAPN RM1 =  50,000 ft, APN    Fig. 16.16
%     nominal / stress with XNCLIM removed        delete experiments 1 and 2
%
%   initialpz sees only the TARGET, so it runs ONCE and its answer is handed to
%   all five; the listing re-runs it per case because it only ever runs one.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Every book value was grepped out of zarchan_decoded.txt (the
%   maths-decoded copy) and its line is in the row's .source.  Nothing was read
%   off a figure -- the extraction turned every figure into an image link -- so
%   prose CLAIMS about figures are scored as bounds and orderings instead.
%   The one clean printed number in the passage is the stressing case's
%   interceptor speed, "3000 ft/s" (zarchan.txt:4938); it is DERIVED here as
%   RTMF/TFDES, so it exercises initialpz, the launch geometry and the target
%   model in one figure.  Band 5%: the book prints a round 3000, while a
%   reversed initialpz or a wrong BETEST moves it by tens of percent.
%
%   THE BOOK CONTRADICTS ITSELF on the acceleration limit: zarchan.txt:4885
%   calls XNCLIMG "nominally set to infinity", the listing's own line 1 at
%   :4893 says "XNCLIMG=7.".  Reported as an INFO row citing both and settled
%   by delete experiment 1 -- the nominal case never demands more than ~1.4 g,
%   so removing the limit must change its miss by EXACTLY zero.
%
%   REPORTED, NOT SCORED: the book calls this a "constant speed interceptor"
%   (zarchan.txt:4884), but the listing applies XNC normal to the LINE OF
%   SIGHT, not to the velocity.  At the stressing geometry's lead angles those
%   differ and |VM| does not stay put.  Faithful to the code, at odds with the
%   adjective, so the drift is printed and left unadjudicated.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED IN MATLAB. ***  MATLAB
%   is the caller's; this file has never been executed here.  The figures come
%   from an independent double-precision replication of the same Heun scheme,
%   so a first run has something to be compared against:
%
%     initialpz: RT1F = 133640.6 ft, RT2F = 49985.4 ft, TFDES = 32.990 s
%       nominal RM1 = 170000: GAMMDEG = 126.041 deg, VM = 1873.62 ft/s
%       stress  RM1 =  50000: GAMMDEG =  30.866 deg, VM = 2953.58 ft/s
%     nominal   T = 32.993 s, alt 49972 ft, miss  0.938 ft, |a_T| 7.805 g,
%               peak |ATPLOS| 0.658 g, peak |XNC| 1.408 g,  0 saturated
%     stress    T = 33.003 s, alt 49936 ft, miss 25.534 ft, |a_T| 7.869 g,
%               peak |ATPLOS| 3.2895 g, peak |XNC| 7.000 g, 14 saturated
%     stressAPN T = 32.976 s, alt 50039 ft, miss  0.569 ft, |a_T| 7.805 g,
%               peak |ATPLOS| 4.138 g, peak |XNC| 1.961 g,  0 saturated
%     limit removed: nominal miss 0.938 ft (bit-identical), peak  1.408 g
%                    stress  miss 0.651 ft,                 peak 10.063 g
%     rows: VM -1.55% (band 5%), target decel -2.44% (10%), intercept altitude
%           -0.06% (2%); the four bound/shape rows pass with margin.
%
%   A residual near a band EDGE is not a pass to be accepted, and NEVER widen a
%   band to make a row green -- a red row is a result.
%
% ---------------------------------------------------------------------------
%   OUTPUTS
%     R.nominal .stress .stressAPN .nominal_nolimit .stress_nolimit
%                 per case: .T .RT1K .RT2K .RM1K .RM2K .ATG .ATPLOSG .XNCG
%                 .RTM_ft .VM_fps .VT_fps, plus .miss_ft .T_end .alt_end_ft
%                 .ATG_end .peak_atplos_g .peak_xnc_g .n_sat .vm_drift
%                 .VM0_fps .launch
%     R.launch    initialpz's answer: .RT1F .RT2F .TFDES (shared by all five)
%     R.bench     benchmark rows, schema shared with the other ports
%     R.selfcheck .names/.oks/.detail -- structural, NOT book values
%     R.pass      true only if at least one row was SCORED and all passed
%     R.verdict   '' when everything passed, else the one-line reason.  ALWAYS
%                 FORMED; 'assert' picks the reaction and nothing else -- false
%                 WARNS and still returns R complete, because benchSuite runs
%                 every port with assert off so one red row cannot hide a board.
%     R.p         the parameters actually used;  R.units;  R.summary
%
%   See also MGS.ZARCHAN.BALLISTICTARGETSIM (Listing 16.1 -- the target model
%   cross-checked here), MGS.ZARCHAN.ENGAGE2D, MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 16.2 line 1, verbatim (zarchan.txt:4893).
    opts.RT1     (1,1) double {mustBeFinite} = 0            % ft, target downrange
    opts.RT2     (1,1) double {mustBeFinite, mustBePositive} = 200000 % ft, target altitude
    opts.RM1     (1,1) double {mustBeFinite} = 170000       % ft, NOMINAL missile downrange
    opts.RM2     (1,1) double {mustBeFinite} = 0            % ft, missile altitude
    opts.VT      (1,1) double {mustBeFinite, mustBePositive} = 6000   % ft/s
    opts.RT2DES  (1,1) double {mustBeFinite, mustBePositive} = 50000  % ft, desired intercept
    opts.GAMTDEG (1,1) double {mustBeFinite, mustBePositive} = 45     % deg, re-entry angle
    opts.BETA    (1,1) double {mustBeFinite, mustBePositive} = 500    % lb/ft^2, TRUE
    opts.BETEST  (1,1) double {mustBeFinite, mustBePositive} = 500    % lb/ft^2, ESTIMATED
    opts.XNP     (1,1) double {mustBeFinite, mustBePositive} = 3      % N-prime
    opts.XNCLIMG (1,1) double {mustBeFinite, mustBePositive} = 7      % g, acceleration limit
    % zarchan.txt:4938.  The book's second, stressing, launch point.
    opts.RM1_stress (1,1) double {mustBeFinite} = 50000     % ft
    opts.H       (1,1) double {mustBeFinite, mustBePositive} = 0.01   % s, RK2 step
    % The listing's own endgame refinement, "if RTM < 1000., H = .0002".  Both
    % halves are options so the miss can be probed for step dependence, but
    % they default to the listing's literals.
    opts.H_endgame     (1,1) double {mustBeFinite, mustBePositive} = 0.0002 % s
    opts.endgame_range (1,1) double {mustBeFinite, mustBePositive} = 1000   % ft
    opts.samp    (1,1) double {mustBeFinite, mustBePositive} = 0.1    % s, record interval
    % Guard only.  The listing's "while VC >= 0" has no other exit; a parameter
    % mistake that never closes would hang a suite rather than fail it.
    opts.tmax    (1,1) double {mustBeFinite, mustBePositive} = 300    % s
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so an "if nargin == 0" here would be a mode that never fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

G = 32.2;                       % the listing's own gravity, hard-coded there

if opts.samp < opts.H
    error('mgs:zarchan:ballisticEngagement:samp', ...
        ['samp = %g s is smaller than the integration step H = %g s, so the ' ...
         'returned curve would be sampled at the step rate while claiming ' ...
         'the coarser interval in its own time vector.'], opts.samp, opts.H);
end
if opts.RT2DES >= opts.RT2
    error('mgs:zarchan:ballisticEngagement:rt2des', ...
        ['RT2DES = %g ft is not below the target''s initial altitude %g ft, ' ...
         'so initialpz exits at t = 0 and VM = RTMF/TFDES divides by zero.'], ...
        opts.RT2DES, opts.RT2);
end

P = struct('RT1',opts.RT1, 'RT2',opts.RT2, 'RM2',opts.RM2, 'VT',opts.VT, ...
           'GAMTDEG',opts.GAMTDEG, 'BETA',opts.BETA, 'XNP',opts.XNP, ...
           'XNCLIMG',opts.XNCLIMG, 'H',opts.H, 'H_endgame',opts.H_endgame, ...
           'endgame_range',opts.endgame_range, 'samp',opts.samp, ...
           'tmax',opts.tmax, 'G',G);

% =========================================================================
%   LISTING 16.2.  Transcribed from zarchan.txt:4893-4927.
% =========================================================================
VT1IC =  opts.VT*cos(opts.GAMTDEG/57.3);
VT2IC = -opts.VT*sin(opts.GAMTDEG/57.3);    % negative: the target descends

% initialpz -- run ONCE; it sees only the target.  See the header.
[RT1F, RT2F, TFDES] = initialpz(opts.RT2DES, opts.RT1, opts.RT2, VT1IC, ...
                                VT2IC, opts.BETEST, opts.H, G, opts.tmax);
launch = struct('RT1F',RT1F, 'RT2F',RT2F, 'TFDES',TFDES, 'BETEST',opts.BETEST);

XNCLIM = opts.XNCLIMG*G;
nom  = flyEngagement(P, launch, opts.RM1,        0, XNCLIM, VT1IC, VT2IC);
str  = flyEngagement(P, launch, opts.RM1_stress, 0, XNCLIM, VT1IC, VT2IC);
strA = flyEngagement(P, launch, opts.RM1_stress, 1, XNCLIM, VT1IC, VT2IC);
% ---- the two DELETE EXPERIMENTS on the acceleration limit -----------------
nomU = flyEngagement(P, launch, opts.RM1,        0, Inf,    VT1IC, VT2IC);
strU = flyEngagement(P, launch, opts.RM1_stress, 0, Inf,    VT1IC, VT2IC);

R = struct();
R.launch          = launch;
R.nominal         = nom;
R.stress          = str;
R.stressAPN       = strA;
R.nominal_nolimit = nomU;
R.stress_nolimit  = strU;
R.units = ['ft, ft/s, ft/s^2 (g = 32.2 ft/s^2); rho slug/ft^3; beta lb/ft^2; ' ...
           'angles deg via the listing''s 57.3 deg/rad'];
R.p = struct('RT1',opts.RT1, 'RT2',opts.RT2, 'RM1',opts.RM1, 'RM2',opts.RM2, ...
             'VT',opts.VT, 'RT2DES',opts.RT2DES, 'GAMTDEG',opts.GAMTDEG, ...
             'BETA',opts.BETA, 'BETEST',opts.BETEST, 'XNP',opts.XNP, ...
             'XNCLIMG',opts.XNCLIMG, 'RM1_stress',opts.RM1_stress, ...
             'H',opts.H, 'H_endgame',opts.H_endgame, ...
             'endgame_range',opts.endgame_range, 'samp',opts.samp, ...
             'tmax',opts.tmax, 'units',R.units);

R.selfcheck = structuralChecks(P, R);

% The book's rows describe the book's engagement.  Off-book physics must SKIP
% them, never quietly re-score them against a different problem.
onBook = opts.RT2 == 200000 && opts.VT == 6000 && opts.GAMTDEG == 45 && ...
         opts.BETA == 500 && opts.BETEST == 500 && opts.XNP == 3 && ...
         opts.XNCLIMG == 7 && opts.RT2DES == 50000 && opts.RT1 == 0 && ...
         opts.RM2 == 0 && opts.RM1 == 170000 && opts.RM1_stress == 50000;

if opts.benchmark
    R.bench = benchmarks(R, onBook, opts);
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
elseif ~any(ev)
    msg = ['not one book value could be evaluated -- the Chapter 16 rows ' ...
           'describe the book''s own engagement and this run changed it.'];
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
        error('mgs:zarchan:ballisticEngagement:benchmark', ...
            '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN.  R comes back complete so the caller can see WHICH
        % side of a comparison moved.  Never silence.
        warning('mgs:zarchan:ballisticEngagement:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function [rt1f, rt2f, tfdes] = initialpz(rt2des, rt1ic, rt2ic, vt1ic, vt2ic, ...
                                         beta, h, G, tmax)
%INITIALPZ  Listing 16.2's sub-function, zarchan.txt:4919-4927.  A mini
%   simulation of the TARGET ALONE that predicts where and when it reaches the
%   desired intercept altitude.  It is told the ESTIMATED ballistic
%   coefficient; zarchan.txt:4884 warns that an error there becomes an
%   interceptor launch error, which is what beta ~= betest buys.
%
%   "while rt2 . rt2des" is resolved as "rt2 > rt2des" -- see the header.
%   Every constant arrives as an ARGUMENT: local functions cannot see the
%   caller's workspace, and a port here that forgot that threw on its first
%   line with no static warning.
rt1 = rt1ic;  rt2 = rt2ic;  vt1 = vt1ic;  vt2 = vt2ic;
t = 0.0;
at1 = 0.0;  at2 = 0.0;      % so the FLAG branch cannot read a stale value

while rt2 > rt2des
    rt1old = rt1;  rt2old = rt2;  vt1old = vt1;  vt2old = vt2;
    step = 1;  flag = 0;
    while step <= 1
        if flag == 1
            rt1 = rt1 + h*vt1;  rt2 = rt2 + h*vt2;
            vt1 = vt1 + h*at1;  vt2 = vt2 + h*at2;
            t = t + h;  step = 2;
        end
        rho  = rhoOf(rt2);
        vt   = sqrt(vt1^2 + vt2^2);
        q    = 0.5*rho*vt^2;
        gamt = atan2(-vt2, vt1);
        at1  = -G*q*cos(gamt)/beta;
        at2  = -G + G*q*sin(gamt)/beta;
        flag = 1;
    end
    rt1 = 0.5*(rt1old + rt1 + h*vt1);  rt2 = 0.5*(rt2old + rt2 + h*vt2);
    vt1 = 0.5*(vt1old + vt1 + h*at1);  vt2 = 0.5*(vt2old + vt2 + h*at2);
    if t > tmax
        error('mgs:zarchan:ballisticEngagement:initialpz', ...
            ['initialpz did not reach %g ft in %g s.  The listing''s loop is ' ...
             'unbounded; this is a guard, and reaching it is a parameter ' ...
             'mistake, not a result.'], rt2des, tmax);
    end
end
rt1f = rt1;  rt2f = rt2;  tfdes = t;
end


% =============================================================================
function OUT = flyEngagement(P, launch, RM1, APN, XNCLIM, VT1, VT2)
%FLYENGAGEMENT  Listing 16.2's main loop, zarchan.txt:4893-4918.
%
%   XNCLIM arrives as an ARGUMENT so that "no acceleration limit" is the SAME
%   code path with Inf passed in.  A separate no-limit branch would be a second
%   implementation, and a delete experiment run through different code proves
%   nothing about the code it was supposed to test.
%
%   Constants come in through P; local functions cannot see the caller.
G = P.G;

% ---- the launch solution: put the interceptor on a collision course --------
RTM1F   = launch.RT1F - RM1;
RTM2F   = launch.RT2F - P.RM2;
GAMMDEG = 57.3*atan2(RTM2F, RTM1F);
RTMF    = sqrt(RTM1F^2 + RTM2F^2);
VM      = RTMF/launch.TFDES;
VM1     = VM*cos(GAMMDEG/57.3);
VM2     = VM*sin(GAMMDEG/57.3);

RM1_0 = RM1;
RT1 = P.RT1;  RT2 = P.RT2;  RM2 = P.RM2;
RTM1 = RT1 - RM1;  RTM2 = RT2 - RM2;  RTM = sqrt(RTM1^2 + RTM2^2);
VTM1 = VT1 - VM1;  VTM2 = VT2 - VM2;
VC   = -(RTM1*VTM1 + RTM2*VTM2)/RTM;

% H is NOT initialised here: the first statement of every outer step picks it
% from RTM, so an initial value would be dead code posing as a default.
T = 0.0;  S = 0.0;  XNC = 0.0;
AT1 = 0.0;  AT2 = 0.0;  AM1 = 0.0;  AM2 = 0.0;  ATPLOS = 0.0;

nmax = ceil(P.tmax/P.samp) + 8;
n = 0;
Tv = zeros(nmax,1);  R1 = zeros(nmax,1);  R2 = zeros(nmax,1);
M1 = zeros(nmax,1);  M2 = zeros(nmax,1);  AG = zeros(nmax,1);
AP = zeros(nmax,1);  XG = zeros(nmax,1);  RG = zeros(nmax,1);
VMv = zeros(nmax,1); VTv = zeros(nmax,1);

while VC >= 0.0
    if RTM < P.endgame_range
        H = P.H_endgame;
    else
        H = P.H;
    end
    RT1OLD = RT1;  RT2OLD = RT2;  VT1OLD = VT1;  VT2OLD = VT2;
    RM1OLD = RM1;  RM2OLD = RM2;  VM1OLD = VM1;  VM2OLD = VM2;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop: pass 1 at the current state, pass 2
    % after a full Euler step with T advanced by H (INSIDE this loop, once).
    % NOTE the ORDER in the FLAG branch -- POSITION advances on the OLD
    % velocity, before velocity advances on the old acceleration.  Reordering
    % is a silent change of scheme that still runs perfectly smoothly.
    while STEP <= 1
        if FLAG == 1
            RT1 = RT1 + H*VT1;  RT2 = RT2 + H*VT2;
            VT1 = VT1 + H*AT1;  VT2 = VT2 + H*AT2;
            RM1 = RM1 + H*VM1;  RM2 = RM2 + H*VM2;
            VM1 = VM1 + H*AM1;  VM2 = VM2 + H*AM2;
            T = T + H;  STEP = 2;
        end
        % ---- target: Listing 16.1, unchanged ------------------------------
        RHO  = rhoOf(RT2);
        VTs  = sqrt(VT1^2 + VT2^2);
        Q    = 0.5*RHO*VTs^2;
        % GAMT is the DESCENT angle, positive going down, which is why drag
        % enters AT2 with a PLUS sign: it opposes the downward velocity.
        GAMT = atan2(-VT2, VT1);
        AT1  = -G*Q*cos(GAMT)/P.BETA;
        AT2  = -G + G*Q*sin(GAMT)/P.BETA;
        % ---- relative geometry --------------------------------------------
        RTM1 = RT1 - RM1;  RTM2 = RT2 - RM2;
        RTM  = sqrt(RTM1^2 + RTM2^2);
        VTM1 = VT1 - VM1;  VTM2 = VT2 - VM2;
        VC   = -(RTM1*VTM1 + RTM2*VTM2)/RTM;
        XLAM  = atan2(RTM2, RTM1);
        XLAMD = (RTM1*VTM2 - RTM2*VTM1)/(RTM*RTM);
        % Target deceleration PERPENDICULAR to the line of sight -- the only
        % part the interceptor sees as a maneuver.  The section is about how
        % much of it there is.
        ATPLOS = -AT1*sin(XLAM) + AT2*cos(XLAM);
        if APN == 1
            XNC = P.XNP*VC*XLAMD + 0.5*P.XNP*ATPLOS;
        else
            XNC = P.XNP*VC*XLAMD;
        end
        % Symmetric saturation.  "if XNC . XNCLIM" resolved as ">" -- header.
        if XNC >  XNCLIM,  XNC =  XNCLIM;  end
        if XNC < -XNCLIM,  XNC = -XNCLIM;  end
        % NOTE: normal to the LINE OF SIGHT, not to the missile velocity --
        % the listing's own choice, and why |VM| drifts despite the book
        % calling this a constant-speed interceptor.
        AM1 = -XNC*sin(XLAM);
        AM2 =  XNC*cos(XLAM);
        FLAG = 1;
    end
    % The listing writes "FLAG=0" here too.  Dropped: the outer loop
    % re-initialises FLAG on its next pass, so the assignment cannot be read.

    RT1 = 0.5*(RT1OLD + RT1 + H*VT1);  RT2 = 0.5*(RT2OLD + RT2 + H*VT2);
    VT1 = 0.5*(VT1OLD + VT1 + H*AT1);  VT2 = 0.5*(VT2OLD + VT2 + H*AT2);
    RM1 = 0.5*(RM1OLD + RM1 + H*VM1);  RM2 = 0.5*(RM2OLD + RM2 + H*VM2);
    VM1 = 0.5*(VM1OLD + VM1 + H*AM1);  VM2 = 0.5*(VM2OLD + VM2 + H*AM2);

    % ONE increment against samp - H/2.  The listing's literal "S >= .09999"
    % records every 0.1 s at its own H and changes meaning at any other.
    S = S + H;
    if S >= P.samp - 0.5*H
        S = 0.0;
        n = n + 1;
        Tv(n)  = T;
        R1(n)  = RT1/1000;   R2(n) = RT2/1000;
        M1(n)  = RM1/1000;   M2(n) = RM2/1000;
        AG(n)  = sqrt(AT1^2 + AT2^2)/G;
        AP(n)  = ATPLOS/G;
        XG(n)  = XNC/G;
        RG(n)  = RTM;
        VMv(n) = sqrt(VM1^2 + VM2^2);
        VTv(n) = sqrt(VT1^2 + VT2^2);
    end
    if T > P.tmax
        error('mgs:zarchan:ballisticEngagement:tmax', ...
            ['the engagement did not close in %g s (RM1 = %g ft, APN = %d).  ' ...
             'The listing''s "while VC >= 0" has no other exit; this is a ' ...
             'guard, and reaching it is a parameter mistake, not a result.'], ...
            P.tmax, RM1_0, APN);
    end
end
if n < 3
    error('mgs:zarchan:ballisticEngagement:empty', ...
        ['the run recorded %d sample(s) at samp = %g s.  At least three are ' ...
         'needed before "peak" or "monotone" say anything about the flight.'], ...
        n, P.samp);
end

OUT = struct();
OUT.T = Tv(1:n);  OUT.RT1K = R1(1:n);  OUT.RT2K = R2(1:n);
OUT.RM1K = M1(1:n);  OUT.RM2K = M2(1:n);
OUT.ATG = AG(1:n);  OUT.ATPLOSG = AP(1:n);  OUT.XNCG = XG(1:n);
OUT.RTM_ft = RG(1:n);  OUT.VM_fps = VMv(1:n);  OUT.VT_fps = VTv(1:n);
% The listing's own answer: a bare "RTM" after the loop, i.e. the second-pass
% range at the PREDICTED state.  Kept exactly so -- see the header.
OUT.miss_ft    = RTM;
OUT.T_end      = T;
OUT.alt_end_ft = RT2;
OUT.APN        = APN;
OUT.VM0_fps    = VM;
OUT.ATG_end    = OUT.ATG(end);
OUT.launch     = struct('RM1',RM1_0, 'GAMMDEG',GAMMDEG, 'VM',VM, ...
                        'RTMF',RTMF, 'RTM1F',RTM1F, 'RTM2F',RTM2F);
% Peak |ATPLOS| is taken OUTSIDE the endgame: in the last few thousandths of a
% second the line of sight swings through intercept and ATPLOS flips sign and
% spikes -- the geometry collapsing, not the maneuver Fig. 16.15 is about.  The
% cut is the listing's OWN endgame test, RTM < 1000 ft, not a threshold
% invented to make a row green.
out = OUT.RTM_ft >= P.endgame_range;
if ~any(out), out = true(size(OUT.RTM_ft)); end
OUT.peak_atplos_g   = max(abs(OUT.ATPLOSG(out)));
OUT.peak_atplos_all = max(abs(OUT.ATPLOSG));
OUT.peak_xnc_g      = max(abs(OUT.XNCG));
% With XNCLIM = Inf this is 0 by construction, which is the right answer for
% the delete-experiment cases: nothing was ever clipped.
OUT.n_sat           = sum(abs(OUT.XNCG) >= XNCLIM/G - 1e-9);
OUT.vm_drift        = max(abs(OUT.VM_fps/VM - 1));
OUT.n_samples       = n;
end


% =============================================================================
function rho = rhoOf(alt_ft)
%RHOOF  The book's two-branch exponential atmosphere, zarchan.txt:4900-4901.
%   Reversing the branches multiplies rho at 100 kft by 2.4 and the trajectory
%   still looks entirely reasonable, which is why Listing 16.1's port scores
%   the seam between them.
if alt_ft <= 30000
    rho = 0.002378*exp(-alt_ft/30000);
else
    rho = 0.0034*exp(-alt_ft/22000);
end
end


% =============================================================================
function K = structuralChecks(P, R)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never evidence that
%   this port reproduces Zarchan -- only that it agrees with itself, which is
%   what catches wiring faults.  .names and .oks are what benchSuite turns into
%   board rows; a check stored under any other field name is a check NOBODY CAN
%   SEE, and this package has shipped twenty invariants that way.
names = {};  oks = [];  detail = {};
K = struct();

% 1  DELETE EXPERIMENT.  Remove the acceleration limit from the NOMINAL case.
%    That case never demands more than ~1.4 g, so the limiter is not on the
%    signal path and the miss must not move AT ALL.  This is what reconciles
%    the book's "nominally set to infinity" (zarchan.txt:4885) with the
%    listing's own "XNCLIMG=7." (zarchan.txt:4893): at the nominal geometry
%    they are the same run.  A nonzero difference means the limiter is acting
%    where nothing asked it to.
K.nom_limit_delta_ft = abs(R.nominal.miss_ft - R.nominal_nolimit.miss_ft);
names{end+1}  = 'delete XNCLIM: nominal miss unchanged';
oks(end+1)    = K.nom_limit_delta_ft <= 1e-9;
detail{end+1} = sprintf(['miss %.4f ft limited vs %.4f ft unlimited, delta ' ...
    '%.3e ft; peak demand %.3f g against a %.1f g limit'], ...
    R.nominal.miss_ft, R.nominal_nolimit.miss_ft, K.nom_limit_delta_ft, ...
    R.nominal_nolimit.peak_xnc_g, P.XNCLIMG);

% 2  DELETE EXPERIMENT.  Remove it from the STRESSING case, where the book says
%    it IS on the signal path.  Two things must happen together: the unlimited
%    demand must exceed the limit, and the miss must collapse.  Either alone is
%    weak -- the first without the second would mean the limiter is inert, and
%    the second without the first would mean something else moved.
K.str_miss_ratio = R.stress.miss_ft / max(R.stress_nolimit.miss_ft, realmin);
K.str_demand_g   = R.stress_nolimit.peak_xnc_g;
names{end+1}  = 'delete XNCLIM: stressing miss collapses';
oks(end+1)    = K.str_miss_ratio >= 5 && K.str_demand_g > P.XNCLIMG;
detail{end+1} = sprintf(['miss %.3f ft limited vs %.3f ft unlimited (%.1fx), ' ...
    'unlimited peak demand %.3f g against the %.1f g limit'], ...
    R.stress.miss_ft, R.stress_nolimit.miss_ft, K.str_miss_ratio, ...
    K.str_demand_g, P.XNCLIMG);

% 3  CROSS-CHECK against Listing 16.1.  The target channel in 16.2 is
%    completely decoupled from the missile -- no term in AT1/AT2 depends on
%    RM1, RM2, VM1 or VM2 -- so at the same H it must reproduce the sibling
%    port to rounding.  A drifting agreement would mean the two files disagree
%    about drag, gravity or the atmosphere, and only ONE of them is scored
%    against the Chapter 16 closed forms.
[ok3, det3, K.crosscheck_rel] = crossCheck161(P, R.nominal);
names{end+1}  = 'target channel matches Listing 16.1';
oks(end+1)    = ok3;
detail{end+1} = det3;

% 4  Range closes monotonically in all three book cases, right up to the exit.
%    The loop's ONLY exit is VC < 0, so if RTM ever grew while the loop was
%    still running, VC and RTM would be describing different geometries.
cs  = {R.nominal, R.stress, R.stressAPN};
lbl = {'nominal','stress','stressAPN'};
worst = -Inf;  which = '';
for i = 1:3
    d = max(diff(cs{i}.RTM_ft));
    if d > worst, worst = d;  which = lbl{i};  end
end
K.max_rtm_step_ft = worst;
names{end+1}  = 'range closes monotonically (loop exits on VC only)';
oks(end+1)    = worst < 0;
detail{end+1} = sprintf(['worst consecutive range change %+.3f ft, in the ' ...
    '%s case'], worst, which);

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
function [ok, detail, relmax] = crossCheck161(P, S)
%CROSSCHECK161  Our target channel against MGS.ZARCHAN.BALLISTICTARGETSIM.
%
%   Only PRE-ENDGAME samples are compared: once H drops to H_endgame our record
%   phase shifts off the samp grid, and a phase difference is not a physics
%   difference.  Matching is nearest-time within half a step, so a sibling that
%   starts its record on a different sample cannot fail this row for the wrong
%   reason.  The sibling is called with 'benchmark' false, which makes it WARN;
%   that one warning is suppressed and restored.  If it cannot be called at all
%   the row FAILS with the reason -- never swallowed into a pass.
ok = false;  relmax = NaN;
try
    ws = warning('off', 'mgs:zarchan:ballisticTargetSim:benchmark');
    % cl is never read: it exists so that its destructor restores the warning
    % state when this function returns, on the error path as well as the
    % normal one.  Deleting the assignment deletes the restore.
    cl = onCleanup(@() warning(ws));
    B = mgs.zarchan.ballisticTargetSim('RT2',P.RT2, 'VT',P.VT, ...
        'GAMTDEG',P.GAMTDEG, 'BETA',P.BETA, 'H',P.H, 'samp',P.samp, ...
        'benchmark',false, 'render',false, 'assert',false);
catch ME
    detail = sprintf('could not call mgs.zarchan.ballisticTargetSim: %s', ...
                     ME.message);
    return
end

pre = find(S.RTM_ft >= P.endgame_range);
if isempty(pre) || isempty(B.T)
    detail = 'no comparable samples (empty pre-endgame record or empty sibling)';
    return
end

% The sibling always launches its target from downrange 0; ours starts at
% P.RT1, so that offset is added back before comparing.
off   = P.RT1/1000;
worst = 0;  nCmp = 0;
scale = max(abs(P.RT2), 1);
for i = pre(:).'
    [dt, j] = min(abs(B.T - S.T(i)));
    if dt > 0.5*P.H, continue, end
    e = max(abs(B.RT2K(j) - S.RT2K(i)), ...
            abs(B.RT1K(j) + off - S.RT1K(i)))*1000;
    worst = max(worst, e/scale);
    nCmp  = nCmp + 1;
end
relmax = worst;
if nCmp < 10
    detail = sprintf(['only %d sample(s) lined up with the sibling, too few ' ...
        'to mean anything'], nCmp);
    return
end
% 1e-9 sits ~7 orders above the last-bit disagreement two identical schemes can
% pick up from operation ORDER alone and ~6 orders below any real difference in
% drag, gravity or the atmosphere.  There is no band between them to argue in.
ok = worst <= 1e-9;
detail = sprintf(['worst relative position difference %.3e over %d samples ' ...
    '(tol 1e-9)'], worst, nCmp);
end


% =============================================================================
function B = benchmarks(R, onBook, opts)
B = emptyBench();
skipWhy = ['this run changed the book''s engagement, and the Chapter 16 ' ...
           'passage describes only its own'];

% ---- 1.  The one clean printed value in the passage. ----------------------
if onBook
    B = addVal(B, 'interceptor speed, stressing launch', 3000, ...
        R.stress.VM0_fps, '5%', 0.05, ...
        'p.369 text, Fig. 16.14 (zarchan.txt:4938)', 'ft/s', ...
        sprintf(['RTMF/TFDES out of initialpz: %.1f ft / %.4f s.  The book ' ...
        'prints a round 3000; a reversed initialpz or a wrong BETEST moves ' ...
        'this by tens of percent.'], R.stress.launch.RTMF, R.launch.TFDES));
else
    B = addSkip(B, 'interceptor speed, stressing launch', '3000', skipWhy, ...
        'zarchan.txt:4938', 'ft/s');
end

% ---- 2.  Target deceleration at intercept, nominal case. ------------------
if onBook
    B = addVal(B, 'target deceleration at intercept (50 kft)', 8, ...
        R.nominal.ATG_end, '10%', 0.10, ...
        'p.368 text, Fig. 16.13 (zarchan.txt:4928)', 'g', ...
        sprintf(['last recorded sample, altitude %.2f kft.  "approximately ' ...
        '8 g" is a one-figure statement, hence 10%%; the stressing case ' ...
        '"approaches 8 g" at %.3f g (zarchan.txt:4949).'], ...
        R.nominal.RT2K(end), R.stress.ATG_end));
else
    B = addSkip(B, 'target deceleration at intercept (50 kft)', '8', skipWhy, ...
        'zarchan.txt:4928', 'g');
end

% ---- 3.  The intercept happens where initialpz was asked to put it. -------
if onBook
    B = addVal(B, 'intercept altitude', opts.RT2DES, R.nominal.alt_end_ft, ...
        '2%', 0.02, 'p.364 text (zarchan.txt:4883)', 'ft', ...
        sprintf(['"have the intercept take place at 50-kft altitude ' ...
        '(RT2DES = 50,000)"; stressing case lands at %.0f ft, APN at %.0f ft'], ...
        R.stress.alt_end_ft, R.stressAPN.alt_end_ft));
else
    B = addSkip(B, 'intercept altitude', sprintf('%g', opts.RT2DES), skipWhy, ...
        'zarchan.txt:4883', 'ft');
end

% ---- 4.  "the peak value is in excess of 3 g" ----------------------------
pk = R.stress.peak_atplos_g;
B = addBound(B, 'peak target decel perpendicular to LOS, stressing', ...
    '> 3 g', sprintf('%.3f g', pk), pk > 3, ...
    'p.369 text, Fig. 16.15 (zarchan.txt:4949)', 'g', ...
    sprintf(['taken outside the listing''s own endgame (RTM >= %g ft); the ' ...
    'unfiltered peak is %.3f g and is the line of sight swinging through ' ...
    'intercept, not the target'], opts.endgame_range, R.stress.peak_atplos_all));

% ---- 5.  The ordering the whole section turns on. ------------------------
%      "there is no target deceleration perpendicular to the line of sight"
%      (nominal) against "much larger than it was for the inverse trajectory
%      case" (stressing).  Two adjectives, one measurable ordering.
rat = pk / max(R.nominal.peak_atplos_g, realmin);
B = addShape(B, 'perpendicular decel: inverse ~0, stressing much larger', ...
    'nominal ~ 0, stressing much larger', ...
    sprintf('%.3f g vs %.3f g (%.1fx)', R.nominal.peak_atplos_g, pk, rat), ...
    rat >= 3 && R.nominal.peak_atplos_g < 1, ...
    'p.368 and p.369 text (zarchan.txt:4928, :4949)', ...
    ['"From a missile point of view, the target does not appear to be ' ...
     'maneuvering" is the nominal half of this row']);

% ---- 6.  "the interceptor requires more than 7 g (missile acceleration
%          limit) to hit the apparent 3-g target maneuver" -----------------
%      Measured by REMOVING the limit and reading the demand.  With the limit
%      in place the command is 7 g by construction and the sentence would be
%      unfalsifiable -- which is exactly the sort of row that looks green and
%      tests nothing.
dem = R.stress_nolimit.peak_xnc_g;
B = addBound(B, 'unlimited PN demand exceeds the 7-g capability', ...
    sprintf('> %g g', opts.XNCLIMG), sprintf('%.3f g', dem), ...
    dem > opts.XNCLIMG, ...
    'p.369 text (zarchan.txt:4949); the 7 g is from :4948', 'g', ...
    ['peak |XNC| with XNCLIM removed -- the same code path with Inf passed ' ...
     'in, not a second implementation']);

% ---- 7.  Saturation -> large miss, and APN removes both. -----------------
B = addShape(B, 'saturation causes the miss; APN removes both', ...
    'PN saturates and misses; APN succeeds', ...
    sprintf('PN %d sat, miss %.2f ft | APN %d sat, miss %.2f ft', ...
        R.stress.n_sat, R.stress.miss_ft, R.stressAPN.n_sat, ...
        R.stressAPN.miss_ft), ...
    R.stress.n_sat > 0 && R.stressAPN.n_sat == 0 && ...
        R.stressAPN.miss_ft < R.stress.miss_ft/5, ...
    'p.369-370 text, Figs. 16.15/16.16 (zarchan.txt:4949, :4950)', ...
    sprintf(['nominal case for scale: %d saturated samples, miss %.2f ft.  ' ...
    '"Large" is the book''s adjective, so the row scores the comparison it ' ...
    'is an adjective about, not a number.'], R.nominal.n_sat, ...
    R.nominal.miss_ft));

% ---- 8.  The inverse geometry, from the prose about Fig. 16.12. ----------
tpath = sqrt((R.nominal.RT1K(end) - opts.RT1/1000)^2 + ...
             (R.nominal.RT2K(end) - opts.RT2/1000)^2);
mpath = R.nominal.VM0_fps*R.nominal.T_end/1000;
B = addShape(B, 'inverse trajectory: target path much longer than missile', ...
    'target trajectory much longer; target much faster', ...
    sprintf('%.1f vs %.1f kft (%.2fx); VT0 %.0f vs VM %.0f ft/s', ...
        tpath, mpath, tpath/max(mpath,realmin), opts.VT, R.nominal.VM0_fps), ...
    tpath > 1.5*mpath && opts.VT > 1.5*R.nominal.VM0_fps, ...
    'p.364 text, Fig. 16.12 (zarchan.txt:4891)', ...
    'chord for the target, VM*T for the nominally constant-speed missile');

% ---- INFO.  The book contradicting itself, and its own adjective. --------
B = addInfo(B, 'XNCLIMG: "nominally infinity" vs the listing''s 7', ...
    'infinity (:4885) / 7 (:4893)', sprintf('%g g', opts.XNCLIMG), ...
    'zarchan.txt:4885 vs :4893', ...
    ['the book contradicts itself.  Settled by the delete experiment in ' ...
     'R.selfcheck: at the nominal geometry the two are the SAME RUN, because ' ...
     'peak demand there is far under the limit.  The stressing case is where ' ...
     'the difference is the whole point, and there the book uses 7.']);

B = addInfo(B, '"constant speed interceptor" vs XNC normal to the LOS', ...
    'constant speed (:4884)', ...
    sprintf('|VM| drifts %.1f%% nominal, %.1f%% stressing', ...
        100*R.nominal.vm_drift, 100*R.stress.vm_drift), ...
    'zarchan.txt:4884', ...
    ['the listing applies XNC normal to the LINE OF SIGHT, not normal to the ' ...
     'velocity, so speed is only constant while the lead angle is small.  It ' ...
     'is not, in the stressing geometry.  Faithful to the code and at odds ' ...
     'with the adjective, so it is reported rather than adjudicated.']);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want.  This is the
%   schema benchSuite's adaptGeneric consumes, and it is strict on purpose.
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
L{end+1} = 'Zarchan Listing 16.2 -- BALLISTIC TARGET ENGAGEMENT SIMULATION';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['   target %g ft / %g ft/s / %g deg / beta %g (est %g)   ' ...
    'Nprime = %g   limit %g g'], p.RT2, p.VT, p.GAMTDEG, p.BETA, p.BETEST, ...
    p.XNP, p.XNCLIMG);
L{end+1} = sprintf(['   H = %g s (endgame %g s below %g ft), samp = %g s, ' ...
    'launch points %g and %g ft downrange'], p.H, p.H_endgame, ...
    p.endgame_range, p.samp, p.RM1, p.RM1_stress);
L{end+1} = sprintf(['   initialpz: target reaches %g ft at downrange %.1f ft ' ...
    'after TFDES = %.4f s'], p.RT2DES, R.launch.RT1F, R.launch.TFDES);
L{end+1} = '';
L{end+1} = 'CASES';
L{end+1} = sprintf('%-18s %9s %9s %9s %9s %10s %11s %6s', ...
    'case', 'VM0', 'T_end', 'alt_end', 'miss', '|a_T| end', 'pk|ATPLOS|', 'nsat');
nm = {'nominal (PN)','stress (PN)','stress (APN)','nominal no limit', ...
      'stress no limit'};
cs = {R.nominal, R.stress, R.stressAPN, R.nominal_nolimit, R.stress_nolimit};
for i = 1:numel(cs)
    c = cs{i};
    L{end+1} = sprintf('%-18s %9.1f %9.3f %9.0f %9.3f %10.3f %11.3f %6d', ...
        nm{i}, c.VM0_fps, c.T_end, c.alt_end_ft, c.miss_ft, c.ATG_end, ...
        c.peak_atplos_g, c.n_sat); %#ok<AGROW>
end
% The listing's eight printed arrays are NOT tabulated here: they are on
% R.<case>, and eight columns times three cases is a wall nobody reads.
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-44s %s', vstr(K.oks(i)), K.names{i}, ...
        K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning';
    L{end+1} = '            NOT CHECKED, which is not the same thing as fine.';
else
    L{end+1} = 'BENCHMARKS vs ZARCHAN CHAPTER 16';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        if b.skipped
            mark = 'INFO';
            if strcmp(b.kind, 'skip'), mark = 'SKIP'; end
            L{end+1} = sprintf('   %-4s %-54s book %-22s ours %s', ...
                mark, b.what, b.book_str, b.ours_str); %#ok<AGROW>
        elseif isnan(b.book)
            L{end+1} = sprintf('   %-4s %-54s %s | %s', ...
                vstr(b.ok), b.what, b.book_str, b.ours_str); %#ok<AGROW>
        else
            L{end+1} = sprintf('   %-4s %-54s book %-10s ours %-10s %+7.2f%% (band %s)', ...
                vstr(b.ok), b.what, b.book_str, b.ours_str, b.pct, b.band); %#ok<AGROW>
        end
    end
    L{end+1} = sprintf('   -> %d of %d scored rows pass; %d reported only.', ...
        sum([R.bench(ev).ok]), sum(ev), sum(~ev));
end
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
%DRAWCURVES  The listing's two figures, for the three book cases.  OFF by
%   default: benchSuite runs dozens of ports and a figure per port is a window
%   pile, not a result.
figure; hold on; grid on
cs = {R.nominal, R.stress, R.stressAPN};
for i = 1:3
    plot(cs{i}.RT1K, cs{i}.RT2K, cs{i}.RM1K, cs{i}.RM2K);
end
xlabel('Downrange (Kft)'); ylabel('Altitude (Kft)');
title('Listing 16.2 -- engagement geometries');

figure; grid on
plot(R.stress.T, R.stress.ATG, R.stress.T, R.stress.ATPLOSG, ...
     R.stress.T, R.stress.XNCG);
xlabel('Time (Sec)'); ylabel('Acceleration (G)');
title('Listing 16.2 -- stressing case, PN');
end
