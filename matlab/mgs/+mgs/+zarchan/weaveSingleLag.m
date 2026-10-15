function R = weaveSingleLag(opts)
%MGS.ZARCHAN.WEAVESINGLELAG  Zarchan Listing 20.1 -- nonlinear two-dimensional
%   engagement, single-time-constant PN, against a WEAVING target.
%
%   R = mgs.zarchan.weaveSingleLag()
%   R = mgs.zarchan.weaveSingleLag('W', 1.5)          the book's Fig. 20.5 case
%   R = mgs.zarchan.weaveSingleLag('TAU', 0.5, 'assert', false)
%
%   Line-by-line port of Listing 20.1 (zarchan.txt:5945-5957), integrated by the
%   book's own second-order Runge-Kutta (Heun) scheme.  Eighty runs: the initial
%   missile-target separation is swept 500:500:40000 ft and each is flown to
%   intercept, so the output is MISS vs FLIGHT TIME -- Fig. 20.1.
%
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, rad, rad/s; g = 32.2 ft/s^2.  NOT
%   SI.  The rest of mgs is metres; nothing is converted here, so every number
%   printed lies straight against a published one.  A silent ft->m would turn
%   the book's 28 ft of miss into "8.5" and it would still read as a perfectly
%   reasonable miss.  Convert AT THE CALL SITE and say so there.
%
%   WHAT IT IS.  Nonlinear planar geometry (real atan2 line of sight, real range
%   closure), PN through a SINGLE first-order guidance lag, and a target whose
%   flight-path angle turns at nT*sin(w_T*t)/vT -- so it weaves at CONSTANT
%   SPEED.  The command N'*Vc*lambdadot_hat acts PERPENDICULAR TO THE LINE OF
%   SIGHT, not to missile velocity, which is why missile speed drifts a few
%   percent over a run.  Miss is |R_TM| at the step where closing velocity first
%   goes non-positive, signed by which side the target ended up on (:5943).
%
%   WHAT IT IS NOT.  No gravity, drag, thrust, NO ACCELERATION LIMIT, and one
%   lag where a real airframe-plus-autopilot chain has five.  The chapter's own
%   point is that a single lag SERIOUSLY UNDERSTATES the miss (:5938); the
%   fifth-order answer is mgs.zarchan.weaveDesignCurve (Listing 20.3) and this
%   file is the baseline that one is measured against.  Do not quote a miss out
%   of here as an achievable number for anything.
%
%   THE TRANSCRIPTION.  listing_20_01.m reports ONE unresolved comparison,
%   `if(RTM , 1000.)`; the raw text (:5948) has the same `,` where this book's
%   OCR renders `<`.  Resolved as `if RTM < 1000, H = .0005, end` -- refine the
%   step in the ENDGAME, not the approach.  RTM decreases monotonically toward
%   intercept, so `<` is the one-way switch every other engagement listing uses
%   (:413 with H = .0002, :8360 with H = .00001).  And it is MEASURED: flipping
%   it outright still runs smoothly and silently and gives max|miss| = 44.8 ft
%   against the book's 28, +60% (independent scalar twin).  Removing only the
%   refinement is reachable from the options and was run here ('Rswitch', 1e-9):
%   42.1 ft, four rows red, two structural invariants violated.  BENCHMARK 3 at
%   a 2% band catches either; nothing else here would.
%     Dropped rather than copied, all four provably dead: `S=0.` (no sampling
%   logic in this listing), `TGO=RTM/VC` and the pre-loop `XLAM=atan2(...)`
%   (overwritten before any read), and the `FLAG=0` after the inner loop (the
%   outer loop re-initialises it).  `save datfil.txt` and the plotting are not
%   reproduced; 'plot' draws Fig. 20.1 instead.
%
%   BENCHMARKS.  Each row carries its own zarchan.txt citation in .source and
%   its reasoning in .detail; this is the index, not a second copy.  Nothing was
%   read off a figure -- extraction turned every figure into an image link -- so
%   where the BOOK read a number off one, the row carries the 'graph' band.
%     1-2  :5943  Vc "approximately 4000 ft/s"; the 40,000 ft run flies "10 s"
%     3    :5959  max|miss| "as large as 28 ft"
%     4    :5959  min|miss| "or as small as zero" -- an adjective, so a BOUND
%     5    :5959  "sinusoidal ... with frequency 3 rad/s", from zero crossings
%     6-7  :6056  Magnitude|N'=3 = 32.2*w_T/(w_T^2+1/T^2)^1.5   (nT in g)
%          :6058  Phase|N'=3     = pi/2 - 3*atan(w_T*T)
%     8    :6095  Peak/(nT*T^2)|N'=3 = 32.2*x/(1+x^2)^1.5 at x = 2, vs the
%                 "approximately 5.5" the book reads off Fig. 20.8 (:6122)
%     9    :5959  SHAPE: the miss "does not approach zero as the homing time
%                 increases!", unlike Ch.3/Ch.6's step maneuver
%
%   THREE THINGS IN THERE THAT ARE EASY TO GET WRONG:
%     * Rows 6-7 are fitted at FIXED w_T over the LAST FULL WEAVE PERIOD, and
%       that window is load-bearing: over [4, 4+2pi/w] the fit residual is
%       3.42 ft because the transient is still alive, over the last period it is
%       0.25 ft.  Fixing w_T matters too -- the frequency is a book claim scored
%       by row 5, and letting the fit chase it would launder that into 6-7.
%     * ⚠️ ROW 6 CARRIES 'model' (10%), A BAND THIS PORT ADDS.  The book claims
%       that formula matches the LINEARIZED ADJOINT (:6072), and separately that
%       the adjoint matches these nonlinear results; comparing this sweep
%       straight to the formula therefore spans a real modelling gap plus
%       leftover transient.  Row 7 needs no such slack -- a BOUND at 0.05 rad,
%       residual three parts in ten thousand -- so the two are scored apart and
%       a failure says which one moved.
%     * Row 8 is the only row that never touches the integrator and the only one
%       evaluable at ANY options, which is what keeps the board from ever coming
%       back empty.  Every other row is gated and SKIPS off-book.
%
%   INFO, not adjudicated: :5943 calls the sweep "equivalent to varying the
%   flight time from 0.05 s to 10 s in steps of 0.05 s".  The 10 s end is right
%   (40000/4000) but 500/4000 is 0.125 s -- a BOOK ERRATUM.  The right number is
%   not published, and a row that fails on a typo teaches nobody anything.  (The
%   worked examples at :6123/:6127/:6131 -- 33, 17, 8 ft -- are that same figure
%   read multiplied out; the formula gives 34.6, 17.1, 8.6.  That is the entire
%   justification for the 'graph' band.)
%
%   WHAT THIS PRINTS AT THE DEFAULTS.  *** MEASURED, twice: by an independent
%   scalar twin written before this file, and by running this file.  They agree
%   to every digit shown, which is the only reason the twin was worth writing.
%     runs 80, T = 0.1255 .. 10.0095 s, 77616 outer steps (2 rhs evals each)
%     max|miss| 28.0567 ft at T = 1.1275 s (book 28)     +0.20%
%     min|miss| 0.9855 ft = 3.51% of max                 (bound <= 5%)
%     zero-crossing w 3.0434 rad/s (book 3)              +1.45%
%     fit A 18.8615 ft vs closed form 18.3286            +2.91%
%     fit phase -2.17663 vs -2.17634 rad                 -0.00029 rad
%     A_late/A_early 0.950       norm miss at x = 2  5.7601
%     => 9 rows scored, all pass; 5 structural invariants hold; 2 INFO.
%   A residual near a band EDGE is not a pass to be accepted; the lines above
%   say where each should land.  Do not widen a band to make a row green.
%   Unscored cross-check: at 'W', 1.5 (Fig. 20.5) the fitted amplitude is
%   51.2 ft against 18.9 ft at w_T = 3, confirming :5978 that "the miss distance
%   increases significantly with the lower weave frequency".  The book publishes
%   only that comparative, so it stays unscored.
%
%   OUTPUTS.  R.R0_ft/R.t_s/R.miss_ft the sweep (miss is SIGNED); R.ss the
%   steady-state fit and its closed-form counterparts (NaN unless N' = 3);
%   R.bench the rows; R.selfcheck .names/.oks the structural invariants, which
%   are NOT book values; R.pass true only if at least one row was SCORED and all
%   passed; R.verdict '' on success else the one-line reason -- ALWAYS FORMED,
%   with 'assert' choosing only the reaction (true errors, false WARNS and still
%   returns R complete, because benchSuite runs every port with assert off and a
%   port that refuses to return is a port nobody can diagnose).
%
%   See also MGS.ZARCHAN.WEAVEDESIGNCURVE (Listing 20.3, the fifth-order twin),
%   MGS.ZARCHAN.ENGAGE2D (Listing 2.1, the geometry this is built on),
%   MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 20.1 line 1 and its initial-condition block, verbatim.
    opts.XNP     (1,1) double {mustBeFinite, mustBePositive} = 3.0      % N'
    opts.TAU     (1,1) double {mustBeFinite, mustBePositive} = 1.0      % s
    opts.XNT     (1,1) double {mustBeFinite, mustBePositive} = 193.2    % ft/s^2 = 6 g
    % w_T = 0 is the STEP-maneuver problem (Ch.3/Ch.6), not this chapter, and it
    % would divide by zero in every weave-period window below.  Refused, not
    % silently special-cased.
    opts.W       (1,1) double {mustBeFinite, mustBePositive} = 3.0      % rad/s
    opts.VM      (1,1) double {mustBeFinite, mustBePositive} = 3000.0   % ft/s
    opts.VT      (1,1) double {mustBeFinite, mustBePositive} = 1000.0   % ft/s
    opts.R0      (1,1) double {mustBeFinite, mustBePositive} = 500.0    % ft
    opts.dR      (1,1) double {mustBeFinite, mustBePositive} = 500.0    % ft
    opts.R1      (1,1) double {mustBeFinite, mustBePositive} = 40000.0  % ft
    opts.H       (1,1) double {mustBeFinite, mustBePositive} = 0.01     % s
    opts.Hfine   (1,1) double {mustBeFinite, mustBePositive} = 0.0005   % s, endgame
    opts.Rswitch (1,1) double {mustBeFinite, mustBePositive} = 1000.0   % ft
    % Start of the window treated as "after the initial transient" (:5959).
    opts.tss     (1,1) double {mustBeFinite, mustBePositive} = 4.0      % s
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so `if nargin == 0` would be a mode that never fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

if opts.Hfine > opts.H
    error('mgs:zarchan:weaveSingleLag:step', ...
        ['Hfine = %g s exceeds H = %g s, so the "endgame refinement" would ' ...
         'COARSEN the last 1000 ft instead.  That runs smoothly and returns a ' ...
         'plausible, wrong miss -- refused rather than warned about.'], ...
        opts.Hfine, opts.H);
end
if opts.H >= opts.TAU
    error('mgs:zarchan:weaveSingleLag:stiff', ...
        ['H = %g s is not smaller than the guidance time constant TAU = %g s; ' ...
         'the lag state would be integrated at or below one sample per time ' ...
         'constant.  RK2 does not blow up here, it returns a smooth and ' ...
         'entirely wrong curve.'], opts.H, opts.TAU);
end
if opts.R1 < opts.R0 + 3*opts.dR
    error('mgs:zarchan:weaveSingleLag:sweep', ...
        ['the sweep %g:%g:%g ft yields fewer than four runs.  Every ' ...
         'measurement here is a property OF THE CURVE, so a two-point sweep ' ...
         'would return numbers that look fine and mean nothing.'], ...
        opts.R0, opts.dR, opts.R1);
end

% =========================================================================
%   LISTING 20.1.  Transcribed from zarchan.txt:5945-5957.
% =========================================================================
[R0v, t, miss, D] = weaveSweep(opts.XNP, opts.TAU, opts.XNT, opts.W, ...
    opts.VM, opts.VT, opts.R0, opts.dR, opts.R1, ...
    opts.H, opts.Hfine, opts.Rswitch);

% ---- steady-state measurements -------------------------------------------
per = 2*pi/opts.W;                       % one weave period
SS = struct();
SS.win        = [t(end) - per, t(end)];  % the LAST full weave period
SS.win_early  = [opts.tss, opts.tss + per];
[SS.A_ft,       SS.phase_rad, SS.rms_ft,       SS.n_fit]   = ...
    fitSine(t, miss, opts.W, SS.win(1),       SS.win(2));
[SS.A_early_ft, ~,            SS.rms_early_ft, SS.n_early] = ...
    fitSine(t, miss, opts.W, SS.win_early(1), SS.win_early(2));
[SS.w_meas, SS.n_cross] = crossFreq(t, miss, opts.tss);

isN3 = abs(opts.XNP - 3) < 1e-12;
if isN3
    SS.A_cf_ft      = weaveMag(opts.XNT, opts.W, opts.TAU);
    SS.phase_cf_rad = weavePhase(opts.W, opts.TAU);
    SS.phase_resid  = wrapPi(SS.phase_rad - SS.phase_cf_rad);
else
    SS.A_cf_ft = NaN;  SS.phase_cf_rad = NaN;  SS.phase_resid = NaN;
end

[mx, imx] = max(abs(miss));

R = struct();
R.R0_ft = R0v;  R.t_s = t;  R.miss_ft = miss;
R.ss    = SS;
R.max_miss_ft = mx;   R.t_at_max_s = t(imx);
R.min_miss_ft = min(abs(miss));
R.units = ['FEET: miss/range ft, speed ft/s, accel ft/s^2, omega rad/s, ' ...
           'time s; g = 32.2 ft/s^2.  NOT SI.'];
R.p = struct('XNP',opts.XNP, 'TAU',opts.TAU, 'XNT',opts.XNT, 'W',opts.W, ...
    'VM',opts.VM, 'VT',opts.VT, 'sweep',[opts.R0 opts.dR opts.R1], ...
    'H',opts.H, 'Hfine',opts.Hfine, 'Rswitch',opts.Rswitch, 'tss',opts.tss, ...
    'n_runs',numel(R0v), 'n_steps',D.nsteps, 'units',R.units);

R.selfcheck = structuralChecks(D, t, miss, R0v);

if opts.benchmark
    R.bench = benchmarks(R, opts, D, isN3);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine".
    R.pass  = false;
end

R.summary = render(R, isN3, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot
    figure; plot(R.t_s, R.miss_ft, '-o', 'MarkerSize', 3); grid on
    xlabel('Flight Time (Sec)'); ylabel('Miss (Ft)');
    title(sprintf('Listing 20.1: N''=%g, T=%g s, w_T=%g rad/s', ...
        opts.XNP, opts.TAU, opts.W));
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
        error('mgs:zarchan:weaveSingleLag:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN, complete, so the caller can see WHICH side moved.
        warning('mgs:zarchan:weaveSingleLag:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function [R0v, t, miss, D] = weaveSweep(XNP, TAU, XNT, W, VMc, VTc, ...
                                        R0, dR, R1, H0, Hf, RSW)
%WEAVESWEEP  Listing 20.1's outer sweep and its Heun integrator, verbatim.
%
%   Every constant is an ARGUMENT.  MATLAB local functions cannot see the main
%   function's workspace; a previous wave in this package read a constant from
%   the caller and threw on the first line of the first run, which no amount of
%   static checking could have seen.
%
%   D collects the quantities the structural checks need, gathered inside the
%   loop because they are not recoverable from (t, miss) afterwards.

R0v  = (R0:dR:R1).';
nrun = numel(R0v);
t    = zeros(nrun,1);
miss = zeros(nrun,1);

D = struct('vt_resid',0, 'perp_resid',0, 'xnc_scale',0, 'miss_frac',0, ...
           'fine_at_end',true, 'vc_at_end',true, 'nsteps',0);

% A run that never closes would spin forever inside a suite.  Cap on TIME, not
% on step count, so the cap means the same thing at any H.
tcap = 5*R1/(VMc + VTc);

for k = 1:nrun
    VM = VMc;  VT = VTc;
    RM1 = 0.;  RM2 = 0.;  RT1 = R0v(k);  RT2 = 0.;  BETA = 0.;
    VT1 = -VT*cos(BETA);  VT2 = VT*sin(BETA);  T = 0.;
    RTM1 = RT1 - RM1;  RTM2 = RT2 - RM2;  RTM = sqrt(RTM1^2 + RTM2^2);
    VM1 = VM;  VM2 = 0.;  VTM1 = VT1 - VM1;  VTM2 = VT2 - VM2;
    VC  = -(RTM1*VTM1 + RTM2*VTM2)/RTM;
    XLAMH = 0.;  H = H0;
    % Derivatives initialised before the loop so the FLAG==1 branch cannot read
    % a value left over from the previous RUN.
    BETAD = 0.;  AM1 = 0.;  AM2 = 0.;  XLAMHD = 0.;  XNC = 0.;

    while VC > 0.
        if RTM < RSW, H = Hf; end          % <<< the one resolved comparison
        BETAOLD = BETA;  RT1OLD = RT1;  RT2OLD = RT2;
        RM1OLD  = RM1;   RM2OLD = RM2;  VM1OLD = VM1;  VM2OLD = VM2;
        XLAMHOLD = XLAMH;
        STEP = 1;  FLAG = 0;

        % Zarchan's two-pass predictor loop.  Pass 1 evaluates at the current
        % state; pass 2 takes a full Euler step, advances T by H, and
        % re-evaluates there.  T advances INSIDE this loop, exactly once.
        while STEP <= 1
            if FLAG == 1
                STEP = 2;
                BETA = BETA + H*BETAD;
                RT1  = RT1  + H*VT1;   RT2 = RT2 + H*VT2;
                RM1  = RM1  + H*VM1;   RM2 = RM2 + H*VM2;
                VM1  = VM1  + H*AM1;   VM2 = VM2 + H*AM2;
                XLAMH = XLAMH + H*XLAMHD;
                T = T + H;
            end
            VT1 = -VT*cos(BETA);  VT2 = VT*sin(BETA);
            BETAD = XNT*sin(W*T)/VT;                 % the WEAVE
            RTM1 = RT1 - RM1;  RTM2 = RT2 - RM2;
            RTM  = sqrt(RTM1^2 + RTM2^2);
            VTM1 = VT1 - VM1;  VTM2 = VT2 - VM2;
            VC   = -(RTM1*VTM1 + RTM2*VTM2)/RTM;
            XLAM = atan2(RTM2, RTM1);
            XLAMHD = (XLAM - XLAMH)/TAU;             % the single guidance lag
            XNC  = XNP*VC*XLAMHD;
            AM1  = -XNC*sin(XLAM);  AM2 = XNC*cos(XLAM);
            FLAG = 1;
        end
        % The listing's trailing FLAG = 0 is dropped: the outer loop
        % re-initialises FLAG, so the assignment cannot be read.

        % Heun corrector, in the listing's own algebra.
        BETA = .5*(BETAOLD + BETA + H*BETAD);
        RT1  = .5*(RT1OLD  + RT1  + H*VT1);
        RT2  = .5*(RT2OLD  + RT2  + H*VT2);
        RM1  = .5*(RM1OLD  + RM1  + H*VM1);
        RM2  = .5*(RM2OLD  + RM2  + H*VM2);
        VM1  = .5*(VM1OLD  + VM1  + H*AM1);
        VM2  = .5*(VM2OLD  + VM2  + H*AM2);
        XLAMH = .5*(XLAMHOLD + XLAMH + H*XLAMHD);

        D.vt_resid   = max(D.vt_resid,   abs(sqrt(VT1^2 + VT2^2) - VTc));
        D.perp_resid = max(D.perp_resid, abs(AM1*cos(XLAM) + AM2*sin(XLAM)));
        D.xnc_scale  = max(D.xnc_scale,  abs(XNC));
        D.nsteps     = D.nsteps + 1;

        if T > tcap
            error('mgs:zarchan:weaveSingleLag:noclose', ...
                ['run %d (R0 = %g ft) reached %g s without the closing ' ...
                 'velocity going non-positive.  The engagement never ' ...
                 'terminates at these options; a suite would hang here.'], ...
                k, R0v(k), T);
        end
    end

    if RTM2 > 0., RTMP = RTM; else, RTMP = -RTM; end
    t(k)    = T;
    miss(k) = RTMP;
    D.fine_at_end = D.fine_at_end && (H == Hf);
    D.vc_at_end   = D.vc_at_end   && (VC <= 0.);
    D.miss_frac   = max(D.miss_frac, RTM/R0v(k));
end
end


% =============================================================================
function [A, ph, rms, n] = fitSine(t, y, w, t0, t1)
%FITSINE  Least squares y ~ A*sin(w*t + ph) over [t0, t1], at FIXED w.
%   Fixing w is the point: the frequency is a BOOK CLAIM scored separately
%   (BENCHMARK 5), so letting the fit chase it would launder that claim into
%   the amplitude and phase rows.
m = (t >= t0 - 1e-12) & (t <= t1 + 1e-12);
n = sum(m);
if n < 4
    A = NaN;  ph = NaN;  rms = NaN;  return
end
C = cos(w*t(m));  S = sin(w*t(m));
ab = [C(:) S(:)] \ y(m);           % base MATLAB; no toolbox
a = ab(1);  b = ab(2);             % y ~ a*cos + b*sin = A*sin(w t + ph)
A  = hypot(a, b);
ph = atan2(a, b);
r  = y(m) - (a*C(:) + b*S(:));
rms = sqrt(sum(r.^2)/n);
end


% =============================================================================
function [w, nzc] = crossFreq(t, y, t0)
%CROSSFREQ  Weave frequency from sign changes of the miss after t0.
%   Half a period per crossing, so w = pi/mean(diff(zeros)).  Linear
%   interpolation between bracketing samples; no toolbox.
m  = t >= t0;
ts = t(m);  ys = y(m);
i  = find(ys(1:end-1).*ys(2:end) < 0);
nzc = numel(i);
if nzc < 2
    w = NaN;  return
end
zc = ts(i) + (ts(i+1) - ts(i)).*abs(ys(i))./(abs(ys(i)) + abs(ys(i+1)));
w  = pi/mean(diff(zc));
end


% =============================================================================
function m = weaveMag(XNT, W, TAU)
%WEAVEMAG  zarchan.txt:6056, Magnitude|N'=3 = 32.2*w_T/(w_T^2 + 1/T^2)^1.5 with
%   nT in g.  XNT here is in ft/s^2, so the 32.2 is already inside it.  FEET.
m = XNT*W/((W^2 + 1/TAU^2)^1.5);
end


% =============================================================================
function p = weavePhase(W, TAU)
%WEAVEPHASE  zarchan.txt:6058, Phase|N'=3 = pi/2 - 3*atan(w_T*T).  RADIANS.
p = pi/2 - 3*atan(W*TAU);
end


% =============================================================================
function v = weaveNorm3(x)
%WEAVENORM3  zarchan.txt:6095, Peak Miss/(nT*T^2)|N'=3 = 32.2*x/(1+x^2)^1.5,
%   x = w_T*T, nT in g.  Shares no code with the integrator, which is the whole
%   reason it is worth having.
v = 32.2*x./(1 + x.^2).^1.5;
end


% =============================================================================
function d = wrapPi(a)
d = mod(a + pi, 2*pi) - pi;
end


% =============================================================================
function K = structuralChecks(D, t, miss, R0v)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan -- only that it agrees
%   with itself, which is what catches wiring faults.
names = {};  oks = [];

names{end+1} = 'target speed constant (|vT| == VT)';
oks(end+1)   = D.vt_resid <= 1e-6;

% AM is BY CONSTRUCTION perpendicular to the line of sight: (-XNC sin, XNC cos)
% dotted with (cos, sin) is identically zero.  This is the only cheap thing
% that sees a swapped sin/cos in AM1/AM2 -- which would still fly, smoothly.
names{end+1} = 'PN command perpendicular to the LOS';
oks(end+1)   = D.perp_resid <= 1e-9*max(D.xnc_scale, 1);

names{end+1} = 'homing time increases with initial separation';
oks(end+1)   = all(diff(t) > 0);

names{end+1} = 'every run terminates on VC <= 0 with miss << R0';
oks(end+1)   = D.vc_at_end && D.miss_frac <= 0.02;

names{end+1} = 'endgame step refinement active at intercept in every run';
oks(end+1)   = D.fine_at_end;

K = struct();
K.names   = names;
K.oks     = logical(oks);
K.all_ok  = all(K.oks);
K.details = struct('vt_resid',D.vt_resid, 'perp_resid',D.perp_resid, ...
                   'miss_frac',D.miss_frac, 'n_runs',numel(R0v), ...
                   'miss_range',[min(miss) max(miss)]);
if K.all_ok
    K.first_bad = '';
else
    ib = find(~K.oks, 1);
    K.first_bad = names{ib};
end
end


% =============================================================================
function B = benchmarks(R, opts, D, isN3)
%BENCHMARKS  Every row carries its band, the reason for the band, and the
%   zarchan.txt line the book value was grepped from.
B = emptyBench();
SS = R.ss;
SRC1 = 'prose p.439-440, zarchan.txt:5943';
SRC2 = 'prose p.442 (Fig. 20.1), zarchan.txt:5959';
SRC3 = 'Magnitude/Phase|N''=3, zarchan.txt:6056 and :6058 (agreement claimed :6072)';
SRC4 = 'Peak Miss formula zarchan.txt:6095; Fig. 20.8 read at :6122';

% The listing's own sweep and speeds.  Rows 1-5 and 9 are properties OF THAT
% CURVE, so they are skipped rather than re-banded when the caller moves it.
nom = eqTol(opts.VM,3000) && eqTol(opts.VT,1000) && eqTol(opts.R0,500) && ...
      eqTol(opts.dR,500) && eqTol(opts.R1,40000) && eqTol(opts.XNP,3) && ...
      eqTol(opts.TAU,1) && eqTol(opts.XNT,193.2) && eqTol(opts.W,3);
whyNom = 'off-book options: this row is a property of the listing''s own sweep';

% Rows 1-5 and 9 are properties OF THE LISTING'S OWN CURVE.  Each is written
% ONCE and carries its own gate: the constructors emit a SKIP when the gate is
% false.  The earlier shape -- an if/else with a parallel block of addSkip calls
% -- had every book literal typed twice, which is precisely how a book value and
% the row that skips it drift apart without anything going red.
B = addVal(B, 'closing velocity at t = 0', 4000, opts.VM + opts.VT, ...
    'approx', SRC1, 'ft/s', ...
    'the book says "approximately 4000 ft/s"; head-on it is VM + VT exactly', ...
    nom, whyNom);
B = addVal(B, 'flight time of the 40,000 ft run', 10, R.t_s(end), ...
    'text', SRC1, 's', ...
    'book: the sweep is "equivalent to varying the flight time ... to 10 s"', ...
    nom, whyNom);
B = addVal(B, 'largest |miss| over the sweep', 28, R.max_miss_ft, ...
    'text', SRC2, 'ft', ...
    'book: "the miss distance ... can be as large as 28 ft"', nom, whyNom);
B = addBound(B, 'smallest |miss| over the sweep', ...
    '"as small as zero": min <= 5% of max', ...
    sprintf('%.4g ft = %.2f%% of max', R.min_miss_ft, ...
            100*R.min_miss_ft/max(R.max_miss_ft, realmin)), ...
    R.min_miss_ft <= 0.05*R.max_miss_ft, SRC2, 'ft', ...
    '"as small as zero" is an adjective, so it is a bound and not a value', ...
    nom, whyNom);
% 3, the literal from :5959, NOT opts.W -- the gate already forces them equal,
% and writing opts.W here would make the BOOK column a function of the caller's
% options, which is how a benchmark quietly stops being one.
B = addVal(B, 'steady-state oscillation frequency', 3, SS.w_meas, ...
    'approx', SRC2, 'rad/s', ...
    'book: "the miss is sinusoidal in nature with frequency 3 rad/s"', ...
    nom, whyNom);

% 6-7  the chapter's closed form.  N' = 3 only, and the fit must have run.
gN3   = isN3 && isfinite(SS.A_ft);
whyN3 = sprintf(['the closed form at :6056/:6058 is N'''' = 3 only (this run ' ...
    'used %g), or the last-period fit had too few samples'], opts.XNP);
B = addVal(B, 'steady-state miss amplitude vs closed form', ...
    SS.A_cf_ft, SS.A_ft, 'model', SRC3, 'ft', ...
    sprintf(['fit at fixed w over the last weave period [%.3f, %.3f] s, rms ' ...
             '%.3f ft (%.3f ft one period earlier: the transient)'], ...
            SS.win(1), SS.win(2), SS.rms_ft, SS.rms_early_ft), gN3, whyN3);
B = addBound(B, 'steady-state miss phase vs closed form', ...
    'residual <= 0.05 rad', ...
    sprintf('%+.5f rad (%.5f vs %.5f)', SS.phase_resid, SS.phase_rad, ...
            SS.phase_cf_rad), ...
    abs(SS.phase_resid) <= 0.05, SRC3, 'rad', ...
    'phase needs no model slack: nonlinear geometry shifts amplitude, not timing', ...
    gN3, whyN3);

% 8  pure formula.  UNGATED -- evaluable at ANY options, which is what keeps
%    the board from ever coming back empty.
B = addVal(B, 'normalized peak miss at x = w_T*T = 2', 5.5, weaveNorm3(2), ...
    'graph', SRC4, 'ft/(g*s^2)', ...
    ['book reads "approximately 5.5" off Fig. 20.8 and multiplies out to ' ...
     '33 ft (:6123); the printed formula gives 5.760 and 34.6 ft']);

% 9  shape
gShape = nom && isfinite(SS.A_ft) && isfinite(SS.A_early_ft) && SS.A_early_ft > 0;
rat    = SS.A_ft/max(SS.A_early_ft, realmin);
B = addShape(B, 'miss does not decay with homing time', ...
    'amplitude of the last weave period >= 60% of one period earlier', ...
    sprintf('%.3f (%.3f ft vs %.3f ft)', rat, SS.A_ft, SS.A_early_ft), ...
    rat >= 0.60, SRC2, ...
    ['book: the miss "does not approach zero as the homing time increases!", ' ...
     'unlike the step-maneuver results of Ch.3 and Ch.6'], gShape, whyNom);

% INFO: the book's own erratum, and the cost of the sweep.
B = addInfo(B, 'flight-time span of the sweep', ...
    '"from 0.05 s to 10 s in steps of 0.05 s"', ...
    sprintf('%.4f .. %.4f s in steps of %.4f s', R.t_s(1), R.t_s(end), ...
            (R.t_s(end)-R.t_s(1))/max(numel(R.t_s)-1,1)), SRC1, ...
    ['BOOK ERRATUM: 40000/4000 = 10 s is right but 500/4000 = 0.125 s, ' ...
     'not 0.05 s.  Not scored -- the correct number is not published, and a ' ...
     'row that fails on a typo teaches nobody anything.']);
B = addInfo(B, 'cost of the brute-force sweep', 'not a published quantity', ...
    sprintf('%d runs, %d outer steps, 2 rhs evaluations each', ...
            numel(R.R0_ft), D.nsteps), SRC1, ...
    'reported because Listing 20.2 exists to replace this with ONE run');
end


% =============================================================================
%   BENCH ROW HELPERS.  Schema matches what mgs.zarchan.benchSuite consumes via
%   benchRows(): what / kind / book / book_str / ours / ours_str / pct / band /
%   tol / ok / skipped / why / source / units / detail, IN THAT ORDER.  ONE
%   constructor per kind so the field order cannot drift -- a struct array whose
%   fields reordered makes [B.ok] mean something else.
% =============================================================================
function B = emptyBench()
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, source, units, detail, gate, whyGate)
% GATE/WHYGATE are optional: when gate is false the row is emitted as a SKIP
% carrying whyGate, so the book value is typed in exactly ONE place.
%   nargin is meaningful HERE.  The warning in this package is about the MAIN
%   function, whose `arguments` block declares only name-value options and
%   therefore always reports nargin = 0; a local with positional arguments is
%   ordinary MATLAB.
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
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'skip';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = '--';
B(i).pct  = NaN;   B(i).band = 'skip';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end

function t = bandTol(band)
%BANDTOL  Bands encode the PROVENANCE of the book value.  Inventing one
%   silently sets an arbitrary pass criterion, so an unknown band ERRORS.
switch lower(band)
case 'exact',  t = 1e-12;  % an identity, or a literal echoed from the listing
case 'text',   t = 0.02;   % printed in prose to 2-3 significant figures
case 'approx', t = 0.05;   % the book says "approximately"
case 'model',  t = 0.10;   % ⚠️ ADDED BY THIS PORT.  The book value is exact,
                           % but it belongs to the LINEARIZED model and this
                           % listing is the NONLINEAR one; :6072 claims
                           % agreement for the adjoint, not for this sweep.
                           % Used on ONE row (steady-state amplitude), where
                           % the measured residual is +2.9%.
case 'graph',  t = 0.15;   % the author read it off a figure's ordinate
otherwise
    error('mgs:zarchan:weaveSingleLag:band', ...
        ['unknown tolerance band "%s".  Bands encode the PROVENANCE of the ' ...
         'book value; inventing one silently sets an arbitrary pass ' ...
         'criterion.'], band);
end
end

function b = eqTol(a, x)
b = abs(a - x) <= 1e-9*max(1, abs(x));
end


% =============================================================================
function s = render(R, isN3, ev)
L = {};
L{end+1} = 'mgs.zarchan.weaveSingleLag -- Listing 20.1, weaving target vs single-lag PN';
L{end+1} = sprintf('  %s', R.units);
L{end+1} = sprintf(['  N''=%g  TAU=%g s  nT=%g ft/s^2 (%.2f g)  w_T=%g rad/s  ' ...
    'VM=%g  VT=%g ft/s'], R.p.XNP, R.p.TAU, R.p.XNT, R.p.XNT/32.2, R.p.W, ...
    R.p.VM, R.p.VT);
L{end+1} = sprintf('  sweep %g:%g:%g ft -> %d runs, %d outer steps, T = %.4f..%.4f s', ...
    R.p.sweep(1), R.p.sweep(2), R.p.sweep(3), R.p.n_runs, R.p.n_steps, ...
    R.t_s(1), R.t_s(end));
L{end+1} = sprintf('  max|miss| %.4f ft at T = %.4f s     min|miss| %.4f ft', ...
    R.max_miss_ft, R.t_at_max_s, R.min_miss_ft);
L{end+1} = sprintf('  steady state: w %.4f rad/s (%d crossings), fit A %.4f ft, phase %+.5f rad, rms %.4f ft', ...
    R.ss.w_meas, R.ss.n_cross, R.ss.A_ft, R.ss.phase_rad, R.ss.rms_ft);
if isN3
    L{end+1} = sprintf('  closed form  : A %.4f ft, phase %+.5f rad  ->  dA %+.2f%%, dphase %+.5f rad', ...
        R.ss.A_cf_ft, R.ss.phase_cf_rad, 100*(R.ss.A_ft/R.ss.A_cf_ft - 1), ...
        R.ss.phase_resid);
else
    L{end+1} = '  closed form  : N'' = 3 only, not evaluated at these options';
end
L{end+1} = '  --- benchmarks ---';
for i = 1:numel(R.bench)
    b = R.bench(i);
    switch lower(b.kind)
    case 'info', v = 'INFO';
    case 'skip', v = 'SKIP';
    otherwise
        if b.skipped, v = 'SKIP'; elseif b.ok, v = 'PASS'; else, v = 'FAIL'; end
    end
    if strcmpi(b.kind, 'value') && ~b.skipped
        L{end+1} = sprintf('  [%s] %-46s book %-10s ours %-10s %+7.2f%%  (%s)', ...
            v, b.what, b.book_str, b.ours_str, b.pct, b.band); %#ok<AGROW>
    else
        L{end+1} = sprintf('  [%s] %-46s %s | %s', ...
            v, b.what, b.book_str, b.ours_str); %#ok<AGROW>
    end
end
L{end+1} = '  --- structural (self-consistency, NOT book values) ---';
for i = 1:numel(R.selfcheck.names)
    if R.selfcheck.oks(i), v = 'holds'; else, v = 'VIOLATED'; end
    L{end+1} = sprintf('  [%-8s] %s', v, R.selfcheck.names{i}); %#ok<AGROW>
end
if any(ev)
    L{end+1} = sprintf('  scored %d row(s); pass = %d', sum(ev), R.pass);
else
    L{end+1} = '  NOTHING WAS SCORED -- R.pass = false means NOT CHECKED.';
end
s = strjoin(L, newline);
end
