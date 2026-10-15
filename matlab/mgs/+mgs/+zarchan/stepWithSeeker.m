function R = stepWithSeeker(opts)
%MGS.ZARCHAN.STEPWITHSEEKER  Zarchan Listing 19.2 -- nonlinear engagement with
%   SEEKER DYNAMICS and a step in target displacement.
%
%   R = mgs.zarchan.stepWithSeeker()                     % sweep + benchmarks
%   R = mgs.zarchan.stepWithSeeker('TAU',0.5)            % halve the seeker lag
%   R = mgs.zarchan.stepWithSeeker('THOM',1,'SeekerLag',false)   % Listing 19.1
%
%   Book source: matlab/zarchan/_source/zarchan.txt:5666 (listing), :5663
%   (prose); OCR-repaired transcription in _listings/listing_19_02.m.
%
%   UNITS ARE FEET.  ft, ft/s, ft/s^2, rad, s; g = 32.2 ft/s^2.  NOT SI, and
%   nothing is converted -- convert at the call site (see MGS.ZARCHAN.ENGAGE2D).
%
%   WHAT IT IS.  Two aircraft fly 400 ft apart (:5551); the missile homes on
%   their power centroid until, THOM seconds before intercept, one is resolved
%   and the target APPARENTLY JUMPS 200 ft.  Listing 19.1 flies that with a
%   zero-lag PN command; 19.2 puts the command behind a first-order SEEKER --
%   the dish D chases the sightline and drives the command by DISH RATE:
%
%       Ddot = (lam - D)/TAU ;  n_c = N' * Vc * Ddot            (:5680)
%
%   i.e. n_c/lam = N' Vc s/(1 + sT) with T = TAU.  The pair exists to price the
%   lag, so both laws live here behind one switch ('SeekerLag') and their
%   difference is a scored row.
%
%   WHAT IT CAN SAY: how much miss one guidance time constant buys for a given
%   homing time, and that it vanishes for tF/T > 3 (:5815).  WHAT IT CANNOT:
%   anything about a realistic guidance system -- the book calls a single lag
%   optimistic (:5785) and its fifth-order model needs tF/T > 8.  No gravity,
%   no drag, and NO ACCELERATION LIMIT (Listing 19.5 adds one).
%
%   TRANSCRIPTION.  OCR renders '<' as ',' and '>' as '.':  while VC.0. -> VC>0;
%   if(TGO,=THOM..) -> TGO<=THOM; while STEP,=1 -> STEP<=1; if RTM2.0 -> RTM2>0;
%   if(TGO,.3) -> TGO<0.3, i.e. refine the step NEAR intercept (:5568 renders
%   the same line in Listing 19.1 as '..' = TGO>0.3, which would refine the
%   wrong end -- 19.2's glyph is the good one).  S=0. is dead in 19.2, dropped.
%   EPS=XLAM-D is computed by the listing and never used; kept and exported.
%
%   *** THE ONE THAT CHANGES THE ANSWER: the dish initial condition. ***
%   At the switch the listing (:5674) does, IN THIS ORDER:
%       RT2=RT2+DISPLACE; RTM2=RT2-RM2; XLAM=atan2(RTM2,RTM1); D=XLAM;
%   so XLAM is recomputed AFTER the jump and the dish is placed on the NEW
%   sightline: seeker error exactly zero at the switch, n_c(0) = 0.  The prose
%   at :5663 reads as if D should keep the OLD sightline, which here is a no-op
%   (lam == D == 0 for the whole pre-step leg, exactly).  The listing is right
%   and the prose is loose.  THREE INDEPENDENT CONFIRMATIONS:
%   (a) The adjoint of this same loop, Listing 19.3, forms the dish-IC miss as
%       XIC = X4*DISP/(VC*TGO) (:5701).  Read back, that is the forward initial
%       condition D(0) = yTIC/(Vc tF) -- which IS lam_new.  lam_old would kill
%       the XIC term and leave two of the "three error sources" at :5688.
%   (b) With D(0) = lam(0), substituting yt = y - yTIC*(1 - t/tF) gives
%       yt(0)=0, ytdot(0)=yTIC/tF, Dt(0)=0 -- the Ch.3 HEADING ERROR problem at
%       VM*HE = -yTIC/tF.  Ch.3's Miss/(VM HE)|N'=3 = tF e^-x (1-0.5x) then
%       gives EXACTLY the Ch.19 formula Miss/yTIC = e^-x (1-0.5x) at :5749, and
%       explains the book's odd remark at :5698 that the heading-error miss
%       divided by homing time equals the displacement miss.
%   (c) Numerically lam_new tracks the published curve (+6% at x=1, +0.3% at
%       x=5); leaving D alone gives -38.8 ft at x=1 where the book says +36.8,
%       the WRONG SIGN.  Self-check 3 pins it so it cannot drift back.
%
%   NUMERICS -- read before quoting any small miss.  With no acceleration limit
%   the lagged command diverges as tgo -> 0 (peak 217.95 g at x=1 against
%   18.55 g with the lag off), so the endgame is stiff and MISSES BELOW ~10 ft
%   ARE NOT STEP-CONVERGED: at THOM=3, -5.02 ft at Hfine 1e-4 but -6.39 at 1e-5
%   and 1e-6; at THOM=5, -2.03 at 1e-5 but -4.48 at 1e-6.  THOM=1 is flat at
%   39.02 over 1e-4..1e-6.  Hence the small-miss end of Fig. 19.8 is scored as
%   a BOUND, never as a value, and the N'=5 row is a SKIP.
%
%   BENCHMARKS -- nine scored (:5563 :5624 :5660 :5684 :5688 :5749 :5752 :5760
%   :5762 :5785 :5815), two INFO, one SKIP.  Measured on R2025b at the defaults,
%   2026-08-24, ~1.5 s:  miss(x=1,N'=3) 39.019 ft vs 36.788 closed form (+6.1%)
%   and vs 36 figure read (+8.4%);  miss(x=1,N'=4) 12.955 vs 12.263 (+5.6%);
%   peak n_c with lag off 597.16 ft/s^2 vs 600 (-0.5%);  miss with lag off
%   0.033 ft;  max|miss| for x >= 3 is 6.39 ft against a 10 ft bound.  A
%   residual near a band edge is not a pass -- do not widen a band.
%
%   OUTPUTS.  R.THOM_s/R.miss_ft the Fig. 19.8 sweep (SIGNED per :5785);
%   R.nolag the Listing 19.1 twin; R.n4 the N'=4 side run; R.bench the rows;
%   R.selfcheck .names/.oks the structural invariants (NOT book values);
%   R.pass true only if at least one row was SCORED and all passed.  'assert'
%   chooses only the reaction -- false WARNS and still returns R complete,
%   because benchSuite runs every port with assert off.
%
%   See also MGS.ZARCHAN.ENGAGE2D, MGS.ZARCHAN.ADJOINTSINGLELAG,
%   MGS.ZARCHAN.BENCHSUITE.

arguments
    % ---- the book's data statement (:5667), verbatim -----------------------
    opts.XNP      (1,1) double {mustBeFinite, mustBePositive} = 3.0
    opts.TAU      (1,1) double {mustBeFinite, mustBePositive} = 1.0      % s
    opts.DISPLACE (1,1) double {mustBeFinite, mustBePositive} = 200.0    % ft
    opts.VM       (1,1) double {mustBeFinite, mustBePositive} = 3000.0   % ft/s
    opts.VT       (1,1) double {mustBeFinite, mustBePositive} = 1000.0   % ft/s
    opts.RM1      (1,1) double {mustBeFinite} = 0.0                      % ft
    opts.RM2      (1,1) double {mustBeFinite} = 1000.0                   % ft
    opts.RT1      (1,1) double {mustBeFinite} = 20000.0                  % ft
    opts.RT2      (1,1) double {mustBeFinite} = 1000.0                   % ft
    % The listing's own sweep is  for THOM = 0.1:0.1:5  (:5667).  Written as
    % (1:50)/10 so 0.5, 1, 2, 3 and 5 land on EXACT doubles and the benchmark
    % can index them without a fuzzy match.
    opts.THOM     (1,:) double {mustBeFinite, mustBePositive} = (1:50)/10
    opts.H        (1,1) double {mustBeFinite, mustBePositive} = 0.001    % s
    opts.Hfine    (1,1) double {mustBeFinite, mustBePositive} = 0.00001  % s
    opts.TgoFine  (1,1) double {mustBeFinite, mustBePositive} = 0.3      % s
    % false = Listing 19.1's zero-lag law (n_c = N' Vc lamdot).  The benchmark
    % runs BOTH regardless; this switch is for callers who want one.
    opts.SeekerLag (1,1) logical = true
    opts.TMax     (1,1) double {mustBeFinite, mustBePositive} = 60       % guard
    % THE ONLY MODE SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so `if nargin == 0` is a mode that fires always.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

if opts.Hfine > opts.H
    error('mgs:zarchan:stepWithSeeker:step', ...
        ['Hfine = %g s exceeds H = %g s, so the "endgame refinement" would ' ...
         'COARSEN the last %g s of every run instead.  With no acceleration ' ...
         'limit in this listing that does not blow up -- it returns a smooth, ' ...
         'plausible, wrong miss.'], opts.Hfine, opts.H, opts.TgoFine);
end
if opts.H >= opts.TAU
    error('mgs:zarchan:stepWithSeeker:stiff', ...
        ['H = %g s is not smaller than the seeker time constant TAU = %g s.  ' ...
         'The dish state would be integrated at or below one sample per time ' ...
         'constant; the trapezoidal corrector does not fail, it returns a ' ...
         'different guidance system.'], opts.H, opts.TAU);
end
if opts.VM < 400
    error('mgs:zarchan:stepWithSeeker:units', ...
        ['VM = %g.  THIS FILE IS IN FEET PER SECOND (%.0f m/s).  An SI speed ' ...
         'passed straight in does not fail here -- it simulates a slow ' ...
         'missile and hands back a miss in feet that reads as metres.'], ...
        opts.VM, opts.VM*0.3048);
end

G = 32.2;   % the book's g, ft/s^2.

% =========================== THE SWEEP =======================================
nT   = numel(opts.THOM);
miss = zeros(1,nT);
DG   = cell(1,nT);
for k = 1:nT
    [miss(k), DG{k}] = engage(opts.THOM(k), opts.TAU, opts.SeekerLag, ...
        opts.XNP, opts.DISPLACE, opts.VM, opts.VT, ...
        opts.RM1, opts.RM2, opts.RT1, opts.RT2, ...
        opts.H, opts.Hfine, opts.TgoFine, opts.TMax);
end

% The zero-lag twin (Listing 19.1) at the nominal homing time.  ALWAYS run, so
% the "what does the lag cost" row exists whatever 'SeekerLag' was set to.
[iNom, okNom] = idxOf(opts.THOM, 1.0);
thNom = 1.0;
if okNom, thNom = opts.THOM(iNom); end
[miss0, DG0] = engage(thNom, opts.TAU, false, ...
    opts.XNP, opts.DISPLACE, opts.VM, opts.VT, ...
    opts.RM1, opts.RM2, opts.RT1, opts.RT2, ...
    opts.H, opts.Hfine, opts.TgoFine, opts.TMax);

% One extra engagement at N' = 4, nominal homing time, so the row that tests the
% NAVIGATION-RATIO dependence of the miss formula (:5752, :5763) actually fires
% at the default entry instead of skipping forever.
[missN4, DGN4] = engage(thNom, opts.TAU, true, ...
    4.0, opts.DISPLACE, opts.VM, opts.VT, ...
    opts.RM1, opts.RM2, opts.RT1, opts.RT2, ...
    opts.H, opts.Hfine, opts.TgoFine, opts.TMax);

R = struct();
R.THOM_s   = opts.THOM;
R.miss_ft  = miss;                       % SIGNED, per :5785
R.norm     = miss / opts.DISPLACE;       % the Fig. 19.13 ordinate
R.diag     = DG;
R.nolag    = struct('THOM_s',thNom, 'miss_ft',miss0, ...
                    'peak_nc_ftps2',DG0.post_peak_xnc, ...
                    'peak_nc_g',DG0.post_peak_xnc/G, 'diag',DG0);
R.n4       = struct('THOM_s',thNom, 'XNP',4.0, 'miss_ft',missN4, 'diag',DGN4);
R.h_quantum_ft = opts.Hfine * DG0.vc_at_end;
R.units    = ['FEET: miss/range ft, speed ft/s, accel ft/s^2, angle rad, ' ...
              'time s; g = 32.2 ft/s^2.  NOT SI.'];
R.p        = struct('XNP',opts.XNP, 'TAU',opts.TAU, 'DISPLACE',opts.DISPLACE, ...
    'VM',opts.VM, 'VT',opts.VT, 'H',opts.H, 'Hfine',opts.Hfine, ...
    'TgoFine',opts.TgoFine, 'SeekerLag',opts.SeekerLag, 'n_runs',nT, ...
    'units',R.units);

R.selfcheck = structuralChecks(DG, DG0, DGN4, opts.DISPLACE);

if opts.benchmark
    R.bench = benchmarks(R, opts, G);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    R.pass  = false;    % NOT true.  "not checked" must never render as "fine".
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot
    figure; plot(R.THOM_s, R.miss_ft, '-o', 'MarkerSize', 3); grid on
    xlabel('Homing Time (s)'); ylabel('Miss (Ft)');
    title(sprintf('Listing 19.2: N''=%g, TAU=%g s, %g ft step', ...
        opts.XNP, opts.TAU, opts.DISPLACE));
end

msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the book. ' ...
           'R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = 'not one book value could be evaluated at these options.';
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
        error('mgs:zarchan:stepWithSeeker:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:stepWithSeeker:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [RTMP, dg] = engage(THOM, TAU, useLag, XNP, DISPLACE, VM, VT, ...
                             RM1, RM2, RT1, RT2, H0, HF, TGOFINE, TMAX)
%ENGAGE  Listing 19.2 itself (:5667-5683), the book's variable names kept so a
%   side-by-side diff against zarchan.txt stays readable.  useLag = false swaps
%   the four seeker lines for Listing 19.1's zero-lag command (:5573).
%
%   EVERY constant arrives as an argument.  MATLAB local functions cannot see
%   the caller's workspace, and a wave-3 port in this package threw on its
%   first run for exactly that reason.

QSWITCH = 0;  VT1 = -VT;  VT2 = 0.;  T = 0.;
RTM1 = RT1-RM1;  RTM2 = RT2-RM2;  RTM = sqrt(RTM1^2 + RTM2^2);
% Dead as written -- every reader of XLAM recomputes it first -- but it is the
% book's line (:5668) and removing lines that "do nothing" is how a port and a
% listing stop being diffable.  Kept, marked, not relied on.
XLAM = atan2(RTM2, RTM1); %#ok<NASGU>
VM1  = VM;  VM2 = 0.;
VTM1 = VT1-VM1;  VTM2 = VT2-VM2;
VC   = -(RTM1*VTM1 + RTM2*VTM2)/RTM;
D    = 0.;  DD = 0.;  AM1 = 0.;  AM2 = 0.;  XNC = 0.;  EPS = 0.;

if VC <= 0
    error('mgs:zarchan:stepWithSeeker:opening', ...
        ['closing velocity is %.1f ft/s at t = 0, so "while VC > 0" exits ' ...
         'immediately and the INITIAL separation (%.0f ft) comes back ' ...
         'labelled as a miss distance.'], VC, RTM);
end

dg = struct('n_steps',0, 'n_rhs',0, 't_flight',NaN, 'qswitch_n',0, ...
    'rt2_jump',NaN, 'dish_err_switch',NaN, 'lam_jump',NaN, ...
    'pre_max_rtm2',0, 'pre_max_xnc',0, 'n_pre',0, ...
    'post_first_xnc',NaN, 'post_peak_xnc',0, ...
    'vc_at_end',NaN, 'vc_final',NaN, 'eps_final',NaN, ...
    'hit_tmax',false, 'used_lag',useLag);
capture = false;

while VC > 0.
    TGO = RTM/VC;
    if TGO < TGOFINE, H = HF; else, H = H0; end

    if (TGO <= THOM) && (QSWITCH == 0)
        QSWITCH = 1;
        rt2b    = RT2;
        % Both sightlines measured INDEPENDENTLY of D, before anything moves,
        % so the invariant below is a measurement and not a tautology: writing
        % `dg.eps = XLAM - D` straight after `D = XLAM` can only ever be zero
        % and would guard nothing.
        lam_old = atan2(rt2b - RM2, RTM1);
        RT2     = RT2 + DISPLACE;
        lam_new = atan2(RT2 - RM2, RTM1);
        if useLag
            % :5674 -- XLAM recomputed AFTER the jump, then D placed ON it.
            % See the header: this is the seeker-dish initial condition, and it
            % is what makes the run reproduce the published curve.
            RTM2 = RT2-RM2;  XLAM = atan2(RTM2, RTM1);  D = XLAM;
            dg.dish_err_switch = D - lam_new;   % 0 here; = -lam_jump if the
        end                                     % order is ever reverted
        dg.lam_jump = lam_new - lam_old;
        dg.qswitch_n = dg.qswitch_n + 1;
        dg.rt2_jump  = RT2 - rt2b;
        capture = true;
    end
    pre = (QSWITCH == 0);

    RT1OLD=RT1; RT2OLD=RT2; RM1OLD=RM1; RM2OLD=RM2;
    VM1OLD=VM1; VM2OLD=VM2; DOLD=D;
    STEP = 1;  FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            RT1=RT1+H*VT1; RT2=RT2+H*VT2; RM1=RM1+H*VM1; RM2=RM2+H*VM2;
            VM1=VM1+H*AM1; VM2=VM2+H*AM2;
            D = D + H*DD;  T = T + H;
        end
        RTM1=RT1-RM1; RTM2=RT2-RM2; RTM=sqrt(RTM1^2+RTM2^2);
        VTM1=VT1-VM1; VTM2=VT2-VM2;
        VC   = -(RTM1*VTM1 + RTM2*VTM2)/RTM;
        XLAM = atan2(RTM2, RTM1);
        if useLag
            EPS = XLAM - D;              % listing computes it, never uses it
            DD  = (XLAM - D)/TAU;        % first-order seeker
        else
            % Listing 19.1 (:5573): the command rides the ANALYTIC sightline
            % rate, never a difference of XLAM -- at H = 1e-5 s differencing
            % would be pure numerical noise shaped like a guidance command.
            DD  = (RTM1*VTM2 - RTM2*VTM1)/(RTM*RTM);
        end
        XNC = XNP*VC*DD;
        AM1 = -XNC*sin(XLAM);  AM2 = XNC*cos(XLAM);
        FLAG = 1;
        dg.n_rhs = dg.n_rhs + 1;
        if capture
            dg.post_first_xnc = XNC;  capture = false;
        end
    end
    FLAG = 0; %#ok<NASGU>   the book's line; the next outer step re-zeroes it
    RT1=.5*(RT1OLD+RT1+H*VT1); RT2=.5*(RT2OLD+RT2+H*VT2);
    RM1=.5*(RM1OLD+RM1+H*VM1); RM2=.5*(RM2OLD+RM2+H*VM2);
    VM1=.5*(VM1OLD+VM1+H*AM1); VM2=.5*(VM2OLD+VM2+H*AM2);
    D  =.5*(DOLD+D+H*DD);

    dg.n_steps = dg.n_steps + 1;
    if pre
        dg.n_pre        = dg.n_pre + 1;
        dg.pre_max_rtm2 = max(dg.pre_max_rtm2, abs(RTM2));
        dg.pre_max_xnc  = max(dg.pre_max_xnc,  abs(XNC));
    else
        dg.post_peak_xnc = max(dg.post_peak_xnc, abs(XNC));
        % LAST CLOSING velocity, not the terminating one.  The loop exits on
        % VC <= 0, so taking VC unconditionally here hands back a NEGATIVE
        % "closing speed" and, downstream, a negative miss quantum -- a
        % nonsense number that still prints without complaint.
        if VC > 0, dg.vc_at_end = VC; end
    end
    if T > TMAX
        dg.hit_tmax = true;  break
    end
end

% :5683 -- the miss is SIGNED by which side of aircraft 1 the missile passed.
if RTM2 > 0, RTMP = RTM; else, RTMP = -RTM; end
dg.t_flight = T;  dg.vc_final = VC;  dg.eps_final = EPS;
end


% =============================================================================
function K = structuralChecks(DG, DG0, DGN4, DISPLACE)
%STRUCTURALCHECKS  Invariants of the PORT, not values from the book.  A failure
%   here means the file disagrees with ITSELF, so no benchmark row means
%   anything; these never enter the pass count.  Folded over EVERY engagement
%   the call ran, including the two side runs -- an invariant checked only on
%   the nominal case is an invariant the sweep can quietly break.
all_dg = [DG(:); {DG0}; {DGN4}];
nm = { ...
    'pre-step leg is exactly quiescent: RTM2 == 0 and n_c == 0', ...
    'the displacement fires exactly once, and by exactly DISPLACE', ...
    'dish sits on the NEW sightline at the switch, and lam really jumped', ...
    'two derivative evaluations per outer step (book''s trapezoid)', ...
    'every run ended on the VC sign change, not the runaway guard'};
ok = true(1, numel(nm));
for i = 1:numel(all_dg)
    d = all_dg{i};
    ok(1) = ok(1) && (d.n_pre == 0 || (d.pre_max_rtm2 == 0 && d.pre_max_xnc == 0));
    ok(2) = ok(2) && d.qswitch_n == 1 && abs(d.rt2_jump - DISPLACE) <= 1e-9;
    % Row 3 pins the dish-IC resolution argued in the header so it cannot
    % drift back to "leave D alone" -- which reverses the sign of the miss.
    % Both halves are needed: D - lam_new == 0 is the resolution, and
    % |lam_jump| > 0 proves there was a jump for it to be a resolution OF.
    % Deletion-tested: reordering the switch block to leave D on the old
    % sightline turns |D - lam_new| into 0.05 rad and this goes VIOLATED.
    ok(3) = ok(3) && (~d.used_lag || ...
            (abs(d.dish_err_switch) < 1e-12 && abs(d.lam_jump) > 1e-6));
    ok(4) = ok(4) && d.n_rhs == 2*d.n_steps;
    ok(5) = ok(5) && ~d.hit_tmax && d.vc_final <= 0;
end

K = struct('names',{nm}, 'oks',logical(ok), 'all_ok',all(ok), 'first_bad','');
if ~K.all_ok, K.first_bad = nm{find(~ok, 1)}; end
end


% =============================================================================
function B = benchmarks(R, opts, G)
%BENCHMARKS  Every book value here was grepped out of
%   matlab/zarchan/_source/zarchan.txt at the line named in `source`.
B = emptyBench();
S  = 'zarchan.txt';
yT = opts.DISPLACE;

[i1, ok1] = idxOf(R.THOM_s, 1.0);
[i2, ok2] = idxOf(R.THOM_s, 2.0);
[i3, ok3] = idxOf(R.THOM_s, 3.0);
[i5, ok5] = idxOf(R.THOM_s, 5.0);
[ip, okp] = idxOf(R.THOM_s, 0.1);
[ih, okh] = idxOf(R.THOM_s, 0.5);
m1 = pick(R.miss_ft, i1);  m2 = pick(R.miss_ft, i2);
m3 = pick(R.miss_ft, i3);  m5 = pick(R.miss_ft, i5);
mp = pick(R.miss_ft, ip);  mh = pick(R.miss_ft, ih);

isN3   = abs(opts.XNP - 3) < 1e-12;
isTau1 = abs(opts.TAU - 1) < 1e-12;
lagOn  = opts.SeekerLag;
nomOK  = ok1 && isTau1 && lagOn;
whyNom = sprintf(['needs the nominal case: THOM = 1 s in the sweep, TAU = 1 s ' ...
    '(here %g) and SeekerLag on (here %d)'], opts.TAU, lagOn);

% x = tF/T (:5760).  Closed forms at :5749 (N'=3), :5752 (N'=4), :5756 (N'=5).
x   = 1.0;
cf3 = yT*exp(-x)*(1 - 0.5*x);
cf4 = yT*exp(-x)*(1 - x + x^2/6);
cf5 = yT*exp(-x)*(1 - 1.5*x + x^2/2 - x^3/24);

B = addVal(B, 'miss at tF/T = 1, N''=3', cf3, m1, 'model', ...
    sprintf('Miss/yTIC = e^-x(1-0.5x) at x=1, %s:5749 (x = tF/T defined :5760)', S), ...
    'ft', ['closed form of the LINEARISED loop vs this NONLINEAR listing; ' ...
           'the book claims agreement only graphically (Fig. 19.11, :5710)'], ...
    nomOK && isN3, [whyNom ', and N''=3']);

B = addVal(B, 'miss at tF/T = 1 (Fig. 19.13 read)', 36, m1, 'graph', ...
    sprintf('Miss = 0.18 x 200 = 36 ft, %s:5762', S), 'ft', ...
    'the 0.18 is read off the ordinate of Fig. 19.13, hence the graph band', ...
    nomOK && isN3, [whyNom ', and N''=3']);

% Scored off R.n4, a SEPARATE engagement at N' = 4 -- not off the sweep, which
% runs at whatever XNP the caller asked for.
B = addVal(B, 'miss at tF/T = 1, N''=4 (side run)', cf4, R.n4.miss_ft, 'model', ...
    sprintf('Miss/yTIC = e^-x(1-x+x^2/6) at x=1, %s:5752; figure read 12 ft :5763', S), ...
    'ft', ['step spread 0.56 ft here (12.955 at Hfine 1e-5, 12.392 at 1e-4) ' ...
           '-- nearer the unconverged region than the N''=3 row'], ...
    ok1 && isTau1, sprintf(['needs THOM = 1 s in the sweep and TAU = 1 s ' ...
                            '(here %g); XNP is fixed at 4 for this row'], opts.TAU));

B = addVal(B, 'peak |n_c| with the seeker lag REMOVED', 600, ...
    R.nolag.peak_nc_ftps2, 'text', ...
    sprintf('n_cMAX = 3*yTIC/tF^2 = 3*200/1 = 600 ft/s^2 = 18.6 g, %s:5660; "nearly 20 g" :5581', S), ...
    'ft/s^2', ['Listing 19.1''s number, same code with SeekerLag off -- the ' ...
               'baseline the lag is priced against.  The half-per-cent ' ...
               'shortfall is finite geometry: RTM = 4005 ft, not 4000'], ...
    ok1 && isN3, 'needs THOM = 1 s in the sweep and N''=3');

B = addBound(B, 'miss with the lag REMOVED is numerical zero', '|miss| < 1 ft', ...
    sprintf('%.4f ft (one-step floor %.3f ft)', R.nolag.miss_ft, R.h_quantum_ft), ...
    abs(R.nolag.miss_ft) < 1.0, ...
    sprintf('"the miss distance y(tF) is always zero!" for the zero-lag loop, %s:5624; "no ... dynamics ... to cause miss distance" :5563', S), ...
    'ft', 'bound not value: the book''s claim is that it vanishes, not a number', ...
    ok1 && isN3, 'needs THOM = 1 s in the sweep and N''=3');

ratio = abs(m1)/max(abs(R.nolag.miss_ft), eps);
B = addShape(B, 'the seeker lag is the WHOLE miss', ...
    'zero-lag miss vanishes, single-lag miss does not', ...
    sprintf('%.3f ft with lag vs %.4f ft without (%.0fx)', m1, R.nolag.miss_ft, ratio), ...
    isfinite(ratio) && ratio > 100, ...
    sprintf('%s:5563 (no dynamics -> no miss) with :5663 (add one time constant)', S), ...
    'the point of the pair, as an ORDERING so it survives step sensitivity', ...
    nomOK && isN3, [whyNom ', and N''=3']);

bigX = [m3 m5];
B = addBound(B, 'tF/T >= 3 gives negligible miss', ...
    sprintf('|miss| <= 0.05*yTIC = %.0f ft', 0.05*yT), ...
    sprintf('max|miss| over tF/T in {3,5} = %.3f ft', max(abs(bigX))), ...
    all(isfinite(bigX)) && max(abs(bigX)) <= 0.05*yT, ...
    sprintf('"the number of guidance time constants had to be greater than three for the miss distance to be negligible", %s:5815; "for long homing times ... approximately zero" :5688', S), ...
    'ft', ['a BOUND on purpose: at this size the values are not ' ...
           'step-converged (x = 3 gives -5.02 ft at Hfine 1e-4, -6.39 at 1e-5)'], ...
    ok3 && ok5 && isTau1 && lagOn && isN3, ...
    'needs THOM = 3 s and 5 s in the sweep, TAU = 1 s, lag on, N''=3');

seq = [mp mh m1 m2];
B = addShape(B, 'miss falls with homing time and tends to yTIC', ...
    'large for short homing time, ~0 for long; unity = hitting the power centroid', ...
    sprintf('%.2f > %.2f > %.2f > %.2f ft; miss(0.1 s)/yTIC = %.3f', ...
            seq(1), seq(2), seq(3), seq(4), mp/yT), ...
    all(isfinite(seq)) && all(diff(seq) < 0) && mp/yT >= 0.8, ...
    sprintf('%s:5684 (short homing time -> large miss), :5688 (long -> ~0), :5815 (unity = power centroid)', S), ...
    'monotone on 0.1..2 s, where the closed form is monotone too (d/dx flips only at x = 3)', ...
    okp && okh && ok1 && ok2 && isTau1 && lagOn && isN3, ...
    'needs THOM = 0.1, 0.5, 1 and 2 s in the sweep, TAU = 1 s, lag on, N''=3');

B = addShape(B, 'sign convention of the miss', ...
    'positive = between aircraft 1 and the power centroid; negative = below aircraft 1', ...
    sprintf('%+.3f ft at tF/T = 1, %+.3f ft at tF/T = 3', m1, m3), ...
    isfinite(m1) && isfinite(m3) && m1 > 0 && m3 < 0, ...
    sprintf('%s:5785', S), ...
    'the RTM2 > 0 test at :5683; an unsigned miss would hide the overshoot', ...
    ok1 && ok3 && isTau1 && lagOn && isN3, ...
    'needs THOM = 1 s and 3 s in the sweep, TAU = 1 s, lag on, N''=3');

% ---- NOT SCORED ------------------------------------------------------------
B = addSkip(B, 'miss at tF/T = 1, N''=5', '3 ft (:5764) / -3.07 ft (:5756)', ...
    sprintf(['the book contradicts itself AND the quantity is below our ' ...
        'resolution.  :5764 prints "0.015 x 200 = 3 ft", POSITIVE, read off ' ...
        'Fig. 19.13; its own formula (:5756-5758) gives e^-1*(-0.0416667) = ' ...
        '-0.015331 -> %.2f ft, NEGATIVE (:5785 confirms the curve goes below ' ...
        'zero, so the figure read dropped a sign).  And our own value moves ' ...
        '1.86 ft between Hfine 1e-5 (-5.14) and 1e-4 (-3.28) on a 3 ft ' ...
        'quantity.  Widening a band to absorb either would invent a pass.'], cf5), ...
    sprintf('%s:5756 and %s:5764', S, S), 'ft');

B = addInfo(B, 'linear-vs-nonlinear residual grows as x shrinks', ...
    'Fig. 19.11 claims "excellent agreement" (:5710)', ...
    sprintf('x=1 %+.1f%%, x=0.5 %+.1f%%, x=0.1 %+.1f%%', ...
        100*(m1/cf3 - 1), ...
        100*(mh/(yT*exp(-0.5)*(1 - 0.25)) - 1), ...
        100*(mp/(yT*exp(-0.1)*(1 - 0.05)) - 1)), ...
    sprintf('%s:5710', S), ...
    ['claimed on a 0-200 ft ordinate, where 22 ft at x = 0.5 is near a line ' ...
     'width.  lam = yTIC/(Vc tF) reaches 0.5 rad at x = 0.1, so it is the ' ...
     'LINEARISATION that moves, not the simulation']);

B = addInfo(B, 'terminal command is UNBOUNDED with the lag', ...
    'no acceleration limit in this listing (one arrives in Listing 19.5, :5848)', ...
    sprintf('peak |n_c| %.1f g with lag vs %.2f g without, at tF/T = 1', ...
        pickPeak(R, i1)/G, R.nolag.peak_nc_g), ...
    sprintf('%s:5848', S), ...
    ['n_c = N'' Vc (lam - D)/TAU and lam slews without bound as tgo -> 0 -- ' ...
     'why misses below ~10 ft are step-dependent and scored as bounds']);
end


% =============================================================================
%   BENCH ROW HELPERS.  Schema consumed by mgs.zarchan.benchSuite/adaptGeneric:
%   what / kind / book / book_str / ours / ours_str / pct / band / tol / ok /
%   skipped / why / source / units / detail, IN THAT ORDER.  ONE constructor per
%   kind so the field order cannot drift -- a struct array whose fields
%   reordered makes [B.ok] mean something else and fails with "Subscripted
%   assignment between dissimilar structures", which reads like a typo.
% =============================================================================
function B = emptyBench()
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, source, units, detail, gate, whyGate)
% GATE/WHYGATE optional: gate false emits a SKIP carrying whyGate, so the book
% value is typed in exactly ONE place.  nargin is meaningful in a LOCAL with
% positional arguments; the warning in this package is about the MAIN function,
% whose arguments block declares only name-value options.
if nargin >= 9 && ~gate
    B = addSkip(B, what, sprintf('%.4g', book), whyGate, source, units);
    return
end
i = numel(B) + 1;
tol = bandTol(band);
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

function B = addShape(B, what, bookStr, oursStr, ok, source, detail, gate, whyGate)
if nargin >= 8 && ~gate
    B = addSkip(B, what, bookStr, whyGate, source, '-');
    return
end
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'shape';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'shape';  B(i).tol = NaN;
B(i).ok   = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = '-';  B(i).detail = detail;
end

function B = addBound(B, what, bookStr, oursStr, ok, source, units, detail, gate, whyGate)
if nargin >= 9 && ~gate
    B = addSkip(B, what, bookStr, whyGate, source, units);
    return
end
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
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'skip';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = '--';
B(i).pct  = NaN;   B(i).band = 'skip';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end

function t = bandTol(band)
%BANDTOL  Bands encode the PROVENANCE of the book value.  An unknown band
%   ERRORS: inventing one silently sets an arbitrary pass criterion.
switch lower(band)
case 'exact',  t = 1e-12;  % an identity, or a literal echoed from the listing
case 'text',   t = 0.02;   % printed in prose to 2-3 significant figures
case 'approx', t = 0.05;   % the book says "approximately"
case 'model',  t = 0.10;   % the value is exact but belongs to the LINEARISED
                           % model while this listing is the NONLINEAR one
case 'graph',  t = 0.15;   % the author read it off a figure's ordinate
otherwise
    error('mgs:zarchan:stepWithSeeker:band', ...
        ['unknown tolerance band "%s".  Bands encode the PROVENANCE of the ' ...
         'book value, so an unrecognised one is a stop, not a default.'], band);
end
end


% =============================================================================
function [i, ok] = idxOf(v, want)
%IDXOF  Index of WANT in V, exact to 1e-9.  Not "nearest": silently scoring a
%   0.9 s run against the 1.0 s book value is exactly the class of error this
%   package exists to catch.
d = abs(v - want);
[m, i] = min(d);
ok = isfinite(m) && m < 1e-9;
if ~ok, i = []; end
end

function y = pick(v, i)
if isempty(i), y = NaN; else, y = v(i); end
end

function y = pickPeak(R, i)
if isempty(i), y = NaN; else, y = R.diag{i}.post_peak_xnc; end
end


% =============================================================================
function s = render(R, ev)
L = {};
L{end+1} = 'Zarchan Listing 19.2 -- engagement with SEEKER DYNAMICS and a step in target displacement';
L{end+1} = '  (Ch.19 "Multiple Targets"; zarchan.txt:5666.  Zero-lag twin = Listing 19.1.)';
L{end+1} = sprintf('  %s', R.units);
L{end+1} = sprintf(['  N''=%g  TAU=%g s  step=%g ft  VM=%g  VT=%g ft/s  ' ...
                    'H=%g s -> %g s when TGO < %g s'], ...
    R.p.XNP, R.p.TAU, R.p.DISPLACE, R.p.VM, R.p.VT, R.p.H, R.p.Hfine, R.p.TgoFine);
L{end+1} = sprintf(['  %d homing times swept, %.2f..%.2f s;  one-step miss ' ...
                    'floor %.3f ft;  seeker lag %s'], ...
    R.p.n_runs, min(R.THOM_s), max(R.THOM_s), R.h_quantum_ft, onoff(R.p.SeekerLag));
L{end+1} = '';
for i = 1:numel(R.bench)
    b = R.bench(i);
    if b.skipped
        L{end+1} = sprintf('  [%-6s] %-46s book %s', upper(b.kind), b.what, b.book_str); %#ok<AGROW>
        L{end+1} = sprintf('           %s', b.why); %#ok<AGROW>
    elseif strcmp(b.kind, 'value')
        if b.ok, v = 'pass'; else, v = 'FAIL'; end
        L{end+1} = sprintf('  [%-6s] %-46s book %-10s ours %-10s %+7.2f%%  (%s)', ...
            v, b.what, b.book_str, b.ours_str, b.pct, b.band); %#ok<AGROW>
    else
        if b.ok, v = 'pass'; else, v = 'FAIL'; end
        L{end+1} = sprintf('  [%-6s] %-46s book %s', v, b.what, b.book_str); %#ok<AGROW>
        L{end+1} = sprintf('           ours %s', b.ours_str); %#ok<AGROW>
    end
end
L{end+1} = '';
for i = 1:numel(R.selfcheck.names)
    if R.selfcheck.oks(i), v = 'holds'; else, v = 'VIOLATED'; end
    L{end+1} = sprintf('  [%-8s] %s', v, R.selfcheck.names{i}); %#ok<AGROW>
end
L{end+1} = '';
if any(ev)
    L{end+1} = sprintf('  %d of %d scored rows pass; %d not scored; %d/%d invariants hold.', ...
        sum([R.bench(ev).ok]), sum(ev), sum([R.bench.skipped]), ...
        sum(R.selfcheck.oks), numel(R.selfcheck.oks));
else
    L{end+1} = '  NOTHING WAS SCORED at these options -- see the SKIP reasons above.';
end
L{end+1} = '  Misses below ~10 ft are NOT step-converged (no acceleration limit in this listing).';
s = strjoin(L, newline);
end

function s = onoff(b)
if b, s = 'ON (Listing 19.2)'; else, s = 'OFF (Listing 19.1 law)'; end
end
