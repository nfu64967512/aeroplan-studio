function R = engageLinear(opts)
%MGS.ZARCHAN.ENGAGELINEAR  Zarchan Listing 2.2 -- linearised PN engagement.
%
%   R = mgs.zarchan.engageLinear()            run the published benchmarks and
%                                             print the comparison
%   R = mgs.zarchan.engageLinear(name,value)  run ONE engagement, return history
%
%   Line-by-line port of Listing 2.2 (Zarchan, "Tactical and Strategic Missile
%   Guidance", Ch. 2).  Local full text: matlab/zarchan/_source/zarchan.txt
%     line 483  prose: "In this simulation the flight time tF is an INPUT
%                       rather than output."
%     line 484  LISTING 2.2 header
%     line 485  the data statement
%     line 486-492  the RK2 loop, transcribed below
%     line 493  Fig. 2.7 discussion (heading error: linear ~ nonlinear)
%     line 495  Fig. 2.8 discussion (3-g maneuver: linear OVER-predicts)
%
% =========================================================================
%   UNITS ARE THE BOOK'S.  FEET, ft/s, ft/s^2, g = 32.2 ft/s^2.
% =========================================================================
%   The rest of mgs is SI and this file is NOT.  That is deliberate and it is
%   the only thing that makes the port worth having: the book publishes
%   numbers, and a port you cannot compare digit-for-digit against a published
%   number is a port you have not checked.
%
%   Nothing here converts.  A quiet ft->m in this file would turn the 13.0 g
%   initial command into 4.0 g, which still looks like a missile acceleration
%   and is wrong -- exactly the silently-plausible failure this package exists
%   to catch.  If you feed R into anything SI, convert AT THE CALL SITE, in a
%   line that says so.
%
%   g = 32.2 is Zarchan's, not 32.174.  Do not "correct" it: every g-number he
%   prints was divided by 32.2, and switching moves all of them by 0.08 %.
%
% =========================================================================
%   THE MODEL, AND THE ONE THING PEOPLE GET BACKWARDS
% =========================================================================
%   Two states -- relative lateral separation y and its rate ydot -- with
%
%       lambda_dot = (y + ydot*tgo) / (Vc*tgo^2)
%       nc         = N' * Vc * lambda_dot
%       ydotdot    = nT - nc
%
%   integrated by second-order Runge-Kutta at a FIXED step H = 0.01 s.  Unlike
%   the nonlinear Listing 2.1 there is no step-size reduction near the end and
%   no closing-velocity sign test, because:
%
%   *** FLIGHT TIME IS AN INPUT HERE, NOT AN OUTPUT. ***
%
%   The linearisation is taken ABOUT A NOMINAL COLLISION COURSE.  Range is
%   replaced by RTM = Vc*(tF - t), so "how long until intercept" is one of the
%   things you linearised around -- it is a coefficient of the model, not
%   something the model can discover.  The simulation therefore stops when the
%   clock reaches TF, and the miss is DEFINED as y(TF): the residual lateral
%   separation at the instant range was assumed to reach zero.
%
%   Getting this backwards is the classic first mistake with Listing 2.2.  It
%   shows up as someone hunting for the intercept-detection branch, not finding
%   one, and adding it -- at which point the model quietly stops being the
%   linearisation that every adjoint and covariance method in Chs. 3-13 is
%   built on, and the answers stop matching the book with no error anywhere.
%
%   The corollary bites too, and it is measurable.  Against the Ch. 2 nonlinear
%   heading-error case, the book's TF = 10 s is not that engagement's time:
%   its initial range/closing-speed is 40000/3819 = 10.47 s and its realised
%   flight time is 10.58 s (the missile's curved path lengthens it).  That
%   alone puts the linear acceleration ~4.7 % high; small-angle HE vs sin(HE)
%   adds ~2.1 %.  Check D1 below measures the pair together (~8 % rms) and then
%   re-runs with TF handed the nonlinear model's own flight time, where it
%   collapses to ~2 %.  That is the header's claim made numerically rather than
%   asserted.
%
% =========================================================================
%   VC IS NOT IN THE LISTING'S DATA STATEMENT, AND IT CANCELS
% =========================================================================
%   Listing 2.2 uses VC in two places and never assigns it (zarchan.txt:485 is
%   the data statement and has no VC; :488 uses it).  In the book it survives
%   in the workspace from Listing 2.1.  Run the listing standalone in MATLAB
%   and you get "Unrecognized function or variable 'VC'".
%
%   Before picking a value, notice that it CANCELS:
%
%       XNC = XNP*VC * (Y + YD*TGO)/(VC*TGO^2)  ==  XNP*(Y + YD*TGO)/TGO^2
%
%   so the command, and therefore the whole trajectory, is independent of VC.
%   It only scales the reported LAMBDA_DOT.  This file defaults VC = 4000 =
%   VM + VT, the head-on closing speed of Listing 2.1's geometry, so R.lamdot
%   is on the book's scale -- and check C in runBenchmarks re-runs the whole
%   engagement at a VC three orders of magnitude away and PROVES the invariance
%   instead of asserting it in a comment.  (VC = 0 is the one fatal value: 0/0
%   -> NaN for every tick, and the run would still "complete".  It errors.)
%
% =========================================================================
%   THE LAST FRACTION OF A SECOND IS NOT RESOLVABLE.  MEASURED.
% =========================================================================
%   nc = N'*ZEM/tgo^2, and on these cases ZEM itself shrinks like tgo^2, so the
%   command tends to a finite limit.  Numerically it does not: the RK2 solution
%   for y carries a fixed truncation error (~0.013 ft at H = 0.01), and once
%   ZEM drops below that error the ratio is noise amplified by 1/tgo^2.
%
%   Measured on the 3-g case, |nc_sim - nc_exact| as a fraction of peak:
%
%       tgo >= 2.00 s   1.4e-05          tgo >= 0.20 s   1.4e-03
%       tgo >= 1.00 s   5.2e-05          tgo >= 0.10 s   6.1e-03
%       tgo >= 0.50 s   2.1e-04          tgo >= 0.01 s   2.0e+00  (200 % !)
%
%   Monotonicity, which is exact for this case, first breaks at tgo = 0.27 s.
%   The very last in-loop sample (tgo = 0.01) comes out at -5.97 g where the
%   answer is +6.00 g -- RIGHT MAGNITUDE, WRONG SIGN.
%
%   *** THE LISTING'S OWN RECORDED ARRAY ENDS ON A NOISE SAMPLE. ***  Its
%   0.1 s decimation records at t = TF, where TGO = 1e-5, so R.book.nc_g(end)
%   is garbage in both cases: -5.9e6 g for the 3-g case (obvious) and 0.1317 g
%   for the heading-error case (NOT obvious -- it reads like a small, plausible
%   residual acceleration and is pure amplified round-off).  R.book is kept
%   verbatim because it is the listing, but R.book.last_is_noise says so, and
%   nothing in the benchmarks reads it.  The book's own Figs. 2.7/2.8 cannot
%   have been drawn with that point.
%
%   Consequences for how this file is used:
%     - R.t stops at TF - H.  There is deliberately NO t = TF sample.
%     - every terminal check is taken at tgo = TGUARD = 0.5 s, where the
%       measured error is 2e-4 of peak, and the published tgo -> 0 limit is
%       reported next to it from the closed form.
%     - do not "fix" this by shrinking the +1e-5 in TGO.  That makes it worse.
%
% =========================================================================
%   WHY THIS FILE IS IN THIS REPO
% =========================================================================
%   Our mgs model scores 32/32 hits at 0.03 m where the real 32-flight set
%   scores 25/32 at 0.93 m.  The measured cause is guidance-system LAG:
%   flight-path-angle tracking gain 0.063 against the model's 1.385, ~8 s of
%   lag, saturating on 26 of 32 sorties.  Zarchan Ch. 3 / Ch. 6 is the theory
%   of exactly that, and Listing 2.2 is the plant every one of those chapters
%   perturbs.  Porting it buys two things:
%
%     (a) an INDEPENDENT CHECK on our integrator and PN kernel, because the
%         book publishes the answers and the zero-lag cases additionally have
%         exact closed forms to check the RK2 against;
%     (b) the object Ch. 3 attaches a time constant T to.
%
%   *** WHAT THIS FILE CANNOT SHOW YOU. ***  It has NO LAG.  Zero-lag PN with
%   N' > 2 misses by exactly zero against both a heading error and a step
%   target maneuver, and the benchmarks below confirm the port reproduces that
%   zero (0.003 ft and 0.013 ft, which is RK2 residual, not physics).  So
%   nothing in this file can reproduce our failure population, and a number out
%   of here is never evidence about our miss distances.  It is the BASELINE the
%   lag gets measured against.  The lag-to-miss machinery is a different file.
%
% -------------------------------------------------------------------------
%   OPTIONS (defaults are Listing 2.2's data statement)
%       TF        flight time, s -- AN INPUT, see above                 [10]
%       HEDEG     initial heading error, deg                            [-20]
%       XNP       effective navigation ratio N'                         [4]
%       XNT       target maneuver, ft/s^2 (96.6 = 3 g)                  [0]
%       VM        missile speed, ft/s                                   [3000]
%       VC        closing speed, ft/s -- cancels, see above             [4000]
%       Y0        initial relative separation, ft                       [0]
%       YD0       initial relative rate, ft/s; [] means -VM*HE          [[]]
%       H         fixed integration step, s                             [0.01]
%       bench     run the benchmark suite instead of one engagement     [false]
%       nonlinear include the Listing 2.1 cross-check in the suite      [true]
%       strict    error (not just report) if a benchmark check fails    [true]
%       render    print the summary                                     [false]
%
%   OUTPUT, single run (all book units)
%       R.t R.y R.yd R.ydd R.nc R.nc_g R.lamdot   history, t = 0 .. TF-H
%       R.book.t/.y/.yd/.nc_g   the listing's own 0.1 s decimation, verbatim,
%                               INCLUDING its noise sample at t = TF
%       R.book.last_is_noise    true -- see the section above
%       R.miss    y at t_end, ft -- the LINEARISED miss (not a distance
%                 formula, and SIGNED, unlike Listing 2.1's)
%       R.yd_end  ydot at t_end, ft/s
%       R.t_end   where integration stopped; equals TF only if TF/H is an
%                 integer, otherwise the last whole step before TF
%       R.nc0_g   command at t = 0, g
%
%   See also PUREPN2D, MGS.ZARCHAN.CROSSCHECK, MGS.ZARCHAN.SIZING.

arguments
    opts.TF        (1,1) double {mustBePositive, mustBeFinite} = 10.0
    opts.HEDEG     (1,1) double {mustBeFinite}                 = -20.0
    opts.XNP       (1,1) double {mustBeFinite}                 = 4.0
    opts.XNT       (1,1) double {mustBeFinite}                 = 0.0
    opts.VM        (1,1) double {mustBePositive, mustBeFinite} = 3000.0
    opts.VC        (1,1) double {mustBeFinite}                 = 4000.0
    opts.Y0        (1,1) double {mustBeFinite}                 = 0.0
    % No size constraint on YD0 on purpose: "(1,:) double = []" is rejected by
    % MATLAB's own validator, because [] is 0x0 and (1,:) demands one row.  The
    % scalar-or-empty rule is enforced explicitly below, where it can say why.
    opts.YD0             double {mustBeFinite}                 = []
    opts.H         (1,1) double {mustBePositive, mustBeFinite} = 0.01
    opts.bench     (1,1) logical                               = false
    opts.nonlinear (1,1) logical                               = true
    opts.strict    (1,1) logical                               = true
    opts.render    (1,1) logical                               = false
end

% ⚠️⚠️ NOT `nargin == 0`.  In a function whose `arguments` block declares ONLY
%    name-value options, MATLAB reports nargin = 0 no matter how many were
%    passed (measured, R2025b).  The old test here was ALWAYS TRUE, so every
%    caller silently got the benchmark run instead of the sweep it asked for.
%
%    ⚠️ It had no symptom, which is why it survived a first repair pass: the
%    only caller is benchSuite, and benchSuite passes 'bench',true anyway.
%    A masked always-true branch is not a fixed branch -- it was found later by
%    tests/test_zarchan_port_lint.py, and only after a preprocessing bug in that
%    lint (which had been eating the very lines it was meant to scan) was fixed.
%    The flag is the ONLY switch.
if opts.bench
    opts.render = true;
end

if numel(opts.YD0) > 1
    error('mgs:zarchan:engageLinear:yd0', ...
        ['YD0 must be a scalar or [] (meaning "use -VM*HE"); got %d elements. ' ...
         'A vector here would broadcast through the whole integration and ' ...
         'produce %d engagements superposed into one history, with no error.'], ...
        numel(opts.YD0), numel(opts.YD0));
end

if opts.bench
    R = runBenchmarks(opts);
    if opts.render, fprintf('%s\n', R.summary); end
    if opts.strict && ~R.pass
        bad = {R.checks(~[R.checks.ok]).name};
        error('mgs:zarchan:engageLinear:benchmark', ...
            ['%d benchmark check(s) FAILED: %s\n' ...
             'The table above has the measured values.  Do not widen the ' ...
             'tolerances to make this pass -- the closed-form checks are exact ' ...
             'solutions of the ODE being integrated, so a failure there means ' ...
             'the RK2 loop or the ZEM expression is wrong, not that the band ' ...
             'was tight.  Call with (''bench'',true,''strict'',false) to get R ' ...
             'back for inspection.'], numel(bad), strjoin(bad, ', '));
    end
    return
end

R = simulate(opts);
if opts.render, fprintf('%s\n', R.summary); end
end


% =========================================================================
function R = simulate(o)
%SIMULATE  Listing 2.2, transcribed, with the recording split two ways.
GRAV = 32.2;                      % Zarchan's g.  See the header.

if o.VC == 0
    error('mgs:zarchan:engageLinear:vcZero', ...
        ['VC = 0.  XLAMD divides by VC, so every tick would be NaN and the ' ...
         'run would still "complete", returning a NaN miss.  VC cancels out ' ...
         'of the command (see the header), so any nonzero value gives the ' ...
         'same trajectory; 4000 = VM + VT is the book''s head-on value.']);
end

TF = o.TF;  XNP = o.XNP;  XNT = o.XNT;  VM = o.VM;  VC = o.VC;  H = o.H;

% HE = HEDEG/57.3 is the book's conversion, NOT deg2rad.  57.3 vs 180/pi is
% 0.0074 % -- physically irrelevant, but keeping it means every number here is
% reproducible from the printed listing without a "why is my last digit off".
HE = o.HEDEG/57.3;

Y = o.Y0;
if isempty(o.YD0)
    YD = -VM*HE;                  % listing: YD = -VM*HEDEG/57.3
else
    YD = o.YD0;
end

T = 0.0;  S = 0.0;  n = 0;  k = 0;
YDD = 0.0;   % never read before it is written (the FLAG==0 pass writes it
             % first); initialised only so the RK2 shape does not trip static
             % analysis into flagging an undefined variable.

nmax = floor((TF - 1e-5)/H) + 3;
t_h = nan(nmax,1); y_h = nan(nmax,1); yd_h = nan(nmax,1);
nc_h = nan(nmax,1); ydd_h = nan(nmax,1); lam_h = nan(nmax,1);
bt = nan(nmax,1); by = nan(nmax,1); byd = nan(nmax,1); bnc = nan(nmax,1);

% The loop guard is the listing's:  while T <= TF - 1e-5.  The 1e-5 is load
% bearing -- T is accumulated by 1000 additions of 0.01 and lands a couple of
% ulps above 10, so a bare "T <= TF" takes one extra step at TGO = 1e-5 and
% records a sample past the end of the linearisation.
while T <= (TF - 1e-5)
    YOLD = Y;  YDOLD = YD;
    STEP = 1;  FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            Y  = Y  + H*YD;       % Euler predictor
            YD = YD + H*YDD;
            T  = T  + H;
        end
        TGO   = TF - T + .00001;  % the +1e-5 keeps TGO^2 off zero on the last
                                  % step; it is the book's guard, not ours, and
                                  % shrinking it makes the tail worse, not
                                  % better -- see the header
        XLAMD = (Y + YD*TGO)/(VC*TGO*TGO);
        XNC   = XNP*VC*XLAMD;
        YDD   = XNT - XNC;

        % Full-rate history is recorded on the FIRST derivative evaluation,
        % i.e. at the CORRECTED state, so R.t starts at exactly 0 and R.nc(1)
        % is the true t = 0 command.  The listing's own array (below) instead
        % stores the SECOND evaluation, which sits at the Euler-PREDICTED
        % point.  Mid-flight they differ by O(H^2); in the last few hundredths
        % of a second the predicted point is the worse of the two by orders of
        % magnitude, because its O(H^2) offset is what gets divided by TGO^2.
        % They are kept separate rather than reconciled: the book's figures are
        % of the second one, and a "close enough" merge would make an exact
        % comparison against the book impossible.
        if FLAG == 0
            k = k + 1;
            t_h(k) = T; y_h(k) = Y; yd_h(k) = YD;
            nc_h(k) = XNC; ydd_h(k) = YDD; lam_h(k) = XLAMD;
        end
        FLAG = 1;
    end
    FLAG = 0;
    Y  = .5*(YOLD  + Y  + H*YD);  % trapezoidal corrector
    YD = .5*(YDOLD + YD + H*YDD);

    S = S + H;
    if S >= .0999                 % the listing's 0.1 s record decimation
        S = 0.0;
        n = n + 1;
        bt(n) = T; by(n) = Y; byd(n) = YD; bnc(n) = XNC/GRAV;
    end
end

% NO TERMINAL SAMPLE IS APPENDED.  An earlier version of this file evaluated
% the command once more at the final corrected state, which sits at TGO = 1e-5,
% and got 263 g for the heading-error case whose true terminal command is 0.
% The state is kept (as R.miss / R.yd_end) because y(TF) IS the linearised miss
% and is perfectly well conditioned; only the ZEM/TGO^2 ratio is not.
R = struct();
R.t       = t_h(1:k);
R.y       = y_h(1:k);
R.yd      = yd_h(1:k);
R.ydd     = ydd_h(1:k);
R.nc      = nc_h(1:k);
R.nc_g    = nc_h(1:k)/GRAV;
R.lamdot  = lam_h(1:k);

R.book    = struct('t', bt(1:n), 'y', by(1:n), 'yd', byd(1:n), ...
                   'nc_g', bnc(1:n), 'last_is_noise', true, ...
                   'note', ['listing-verbatim 0.1 s decimation; the final ' ...
                            'sample is at TGO = 1e-5 and its nc_g is ' ...
                            'amplified round-off, not an acceleration']);

% MISS is y(TF), the linearised definition (Ch. 2).  It is SIGNED, unlike the
% nonlinear Listing 2.1 miss, which comes out of a distance formula and is
% therefore always positive.  Do not compare their signs.
R.miss    = Y;
R.yd_end  = YD;
% t_end is where the integration actually stopped.  It equals TF only when
% TF/H is an integer.  Otherwise the loop halts at the last whole step before
% TF and R.miss is y(t_end), NOT y(TF) -- a real effect when TF is handed a
% nonlinear model's flight time (check D1 does exactly that).  Reported rather
% than papered over with a short final step, which would not be the listing.
R.t_end   = T;
R.nc0_g   = R.nc_g(1);
R.ncmax_g = max(abs(R.nc_g));
R.TF = TF;  R.XNP = XNP;  R.XNT = XNT;  R.VM = VM;  R.VC = VC;  R.H = H;
R.HEDEG = o.HEDEG;  R.HE = HE;
if isempty(o.YD0), R.YD0 = -VM*HE; else, R.YD0 = o.YD0; end
R.opts = o;
R.summary = renderRun(R);
end


function s = renderRun(R)
L = {};
L{end+1} = sprintf(['Zarchan Listing 2.2 (linear)  TF=%.4g s  N''=%.4g  ' ...
                    'HEDEG=%.4g deg  XNT=%.4g ft/s^2 (%.3g g)'], ...
                   R.TF, R.XNP, R.HEDEG, R.XNT, R.XNT/32.2);
L{end+1} = sprintf('  yd(0)        = %12.4f ft/s', R.YD0);
L{end+1} = sprintf('  nc(0)        = %12.4f ft/s^2 = %8.4f g', R.nc(1), R.nc_g(1));
L{end+1} = sprintf('  nc(t=TF-H)   = %12.4f ft/s^2 = %8.4f g   <- see below', ...
                   R.nc(end), R.nc_g(end));
L{end+1} = sprintf('  max |nc|     = %12.4f g', R.ncmax_g);
L{end+1} = sprintf('  miss y(TF)   = %12.6f ft   (SIGNED; linearised, not a distance)', R.miss);
L{end+1} = '  units are FEET and ft/s^2, g = 32.2.  TF was an INPUT, not a result.';
L{end+1} = '  the last ~0.3 s of nc is amplified round-off (ZEM/tgo^2 below the';
L{end+1} = '  integrator''s own error in y).  Read terminal values at tgo >= 0.5 s.';
s = strjoin(L, newline);
end


% =========================================================================
function nc = cfHeadingError(t, TF, XNP, VM, HEDEG)
%CFHEADINGERROR  Exact zero-lag solution, heading error only.
%
%   nc(t) = -(N'*VM*HE/tF) * (tgo/tF)^(N'-2),   HE = HEDEG/57.3
%
%   Derived from the same ODE the RK2 above integrates: with Z = y + ydot*tgo,
%   dZ/dtgo = -nT*tgo + N'*Z/tgo, and Z = -VM*HE*tF at tgo = tF with nT = 0.
%   So comparing the two is a check of the INTEGRATOR and of the ZEM
%   expression, not a restatement of the code.  This is the "independent
%   check" half of why the port exists.
if XNP <= 2.0
    error('mgs:zarchan:engageLinear:np', ...
        ['N'' = %g.  The closed form carries (N''-2), and PN is not ' ...
         'stabilising at or below N'' = 2, so there is no finite answer to ' ...
         'compare against.  Refusing to return the divergent branch as if it ' ...
         'were a benchmark.'], XNP);
end
HE  = HEDEG/57.3;
tgo = max(TF - t, 0);
nc  = -(XNP*VM*HE/TF) .* (tgo/TF).^(XNP-2);
end


function nc = cfManeuver(t, TF, XNP, XNT)
%CFMANEUVER  Exact zero-lag solution, step target maneuver only.
%
%   nc(t) = [N'*nT/(N'-2)] * [1 - (tgo/tF)^(N'-2)]
%
%   tgo -> 0 limit N'*nT/(N'-2): for N' = 4 that is 2*nT, i.e. a 3-g target
%   costs the interceptor 6 g at the end.  Zero at t = 0, monotonically
%   increasing -- both are checked below, because a sign error in the ZEM term
%   reproduces the magnitude and reverses the shape, and a magnitude-only
%   check would wave it through.
if XNP <= 2.0
    error('mgs:zarchan:engageLinear:np', ...
        'N'' = %g; the closed form divides by (N''-2).  See cfHeadingError.', XNP);
end
tgo = max(TF - t, 0);
nc  = (XNP*XNT/(XNP-2)) .* (1 - (tgo/TF).^(XNP-2));
end


% =========================================================================
function R = runBenchmarks(o)
%RUNBENCHMARKS  Fig. 2.7 and Fig. 2.8, plus the exact-solution, tail-resolution
%   and VC-invariance guards.
GRAV = 32.2;
C = struct('name', {}, 'value', {}, 'want', {}, 'ok', {});
L = {};

TF = 10.0;  XNP = 4.0;  VM = 3000.0;  VC = 4000.0;  H = 0.01;
XNT3G  = 96.6;                    % 3 g in the book's units, 3*32.2
TGUARD = 0.5;                     % read terminal values at tgo >= this.
                                  % 0.5 s is where the measured tail error is
                                  % 2e-4 of peak (see the header's table); at
                                  % 0.01 s it is 200 %.  Not a taste parameter.

L{end+1} = 'Zarchan Listing 2.2 -- linearised engagement.  BOOK UNITS: ft, ft/s^2, g = 32.2';
L{end+1} = repmat('=', 1, 92);

% ---- A: heading error, the Fig. 2.7 case --------------------------------
A    = simulate(mergeOpts(o, 'TF',TF, 'HEDEG',-20.0, 'XNT',0.0, 'XNP',XNP, ...
                             'VM',VM, 'VC',VC, 'H',H, 'Y0',0.0, 'YD0',[]));
cfA  = cfHeadingError(A.t, TF, XNP, VM, -20.0);
refA = max(abs(cfA));
mA   = A.t <= (TF - TGUARD);
eA   = max(abs(A.nc(mA) - cfA(mA)))/refA;

L{end+1} = 'A.  HEADING ERROR   HEDEG = -20, XNT = 0, N'' = 4, TF = 10 s        [Fig. 2.7]';
L{end+1} = sprintf('      nc(0)                       %10.4f g   (closed form %.4f g)', ...
                   A.nc0_g, cfA(1)/GRAV);
L{end+1} = sprintf('      peak |nc|                   %10.4f g   (at t = 0; monotonically decreasing)', ...
                   A.ncmax_g);
L{end+1} = sprintf('      RK2 vs exact solution       %10.3e     (frac of peak, tgo >= %.1f s)', eA, TGUARD);
L{end+1} = sprintf('      linearised miss y(TF)       %10.6f ft  (zero lag, N''>2  =>  0)', A.miss);
L{end+1} = sprintf('      listing array, first sample %10.4f g   at t = %.2f s', ...
                   A.book.nc_g(1), A.book.t(1));
L{end+1} = sprintf('      listing array, LAST sample  %10.4f g   at t = %.2f s  <- NOISE, reads plausible', ...
                   A.book.nc_g(end), A.book.t(end));
C = addCheck(C, 'A: nc(0) matches closed form', abs(A.nc0_g - cfA(1)/GRAV), ...
             '<= 1e-3 g', abs(A.nc0_g - cfA(1)/GRAV) <= 1e-3);
C = addCheck(C, 'A: RK2 tracks exact solution', eA, '<= 2e-3 of peak', eA <= 2e-3);
C = addCheck(C, 'A: zero-lag miss is zero', abs(A.miss), '<= 0.1 ft', abs(A.miss) <= 0.1);

% ---- B: 3-g target maneuver, the Fig. 2.8 case --------------------------
B    = simulate(mergeOpts(o, 'TF',TF, 'HEDEG',0.0, 'XNT',XNT3G, 'XNP',XNP, ...
                             'VM',VM, 'VC',VC, 'H',H, 'Y0',0.0, 'YD0',[]));
cfB  = cfManeuver(B.t, TF, XNP, XNT3G);
refB = max(abs(cfB));
mB   = B.t <= (TF - TGUARD);
eB   = max(abs(B.nc(mB) - cfB(mB)))/refB;
iB   = find(mB, 1, 'last');              % the tgo = TGUARD sample
ncGuardB   = B.nc_g(iB);
cfGuardB   = cfB(iB)/GRAV;
wantLimitB = XNP*XNT3G/(XNP-2)/GRAV;     % the tgo -> 0 limit, exactly 6 g here
monoB      = all(diff(B.nc(mB)) >= -1e-9*refB);

L{end+1} = '';
L{end+1} = 'B.  3-g TARGET MANEUVER   XNT = 96.6 ft/s^2, HEDEG = 0, N'' = 4     [Fig. 2.8]';
L{end+1} = sprintf('      nc at tgo = %.1f s           %10.4f g   (closed form %.4f g)', ...
                   TGUARD, ncGuardB, cfGuardB);
L{end+1} = sprintf('      tgo -> 0 limit  N''nT/(N''-2) %10.4f g   (closed form; NOT resolvable in sim)', ...
                   wantLimitB);
L{end+1} = sprintf('      RK2 vs exact solution       %10.3e     (frac of peak, tgo >= %.1f s)', eB, TGUARD);
L{end+1} = sprintf('      monotonically increasing    %10s      (over tgo >= %.1f s)', tf2s(monoB), TGUARD);
L{end+1} = sprintf('      linearised miss y(TF)       %10.6f ft  (zero lag  =>  0)', B.miss);
L{end+1} = sprintf('      listing array, LAST sample  %10.4g g   at t = %.2f s  <- NOISE, obvious here', ...
                   B.book.nc_g(end), B.book.t(end));
C = addCheck(C, 'B: nc(tgo=0.5) matches closed form', abs(ncGuardB - cfGuardB), ...
             '<= 0.01 g', abs(ncGuardB - cfGuardB) <= 0.01);
C = addCheck(C, 'B: nc(tgo=0.5) approaches 6 g', ncGuardB, '5.90 .. 6.00 g', ...
             ncGuardB >= 5.90 && ncGuardB <= 6.00);
C = addCheck(C, 'B: RK2 tracks exact solution', eB, '<= 2e-3 of peak', eB <= 2e-3);
C = addCheck(C, 'B: nc monotonically increasing', double(monoB), 'true', monoB);
C = addCheck(C, 'B: zero-lag miss is zero', abs(B.miss), '<= 0.1 ft', abs(B.miss) <= 0.1);

% ---- B2: the tail-resolution table, printed so nobody has to rediscover it
L{end+1} = '';
L{end+1} = '    tail resolution of the 3-g case, |nc_sim - nc_exact| as a fraction of peak:';
tailRow = '      ';
for tg = [2.0 1.0 0.5 0.2 0.1 0.01]
    m = B.t <= (TF - tg);
    if any(m)
        tailRow = [tailRow sprintf('tgo>=%.2f %.1e   ', tg, ...
                   max(abs(B.nc(m) - cfB(m)))/refB)]; %#ok<AGROW>
    end
end
L{end+1} = tailRow;
L{end+1} = '      nc = N''*ZEM/tgo^2 and ZEM ~ tgo^2, so once ZEM falls under the RK2 error';
L{end+1} = '      in y (~0.013 ft) the ratio is round-off amplified by 1/tgo^2.  Not a bug';
L{end+1} = '      in the port: it is a property of the listing, and the listing''s own';
L{end+1} = '      recorded array ends inside that region.';

% ---- C: VC really does cancel -------------------------------------------
% A DELETE EXPERIMENT, not a comment.  This project has been burned by guards
% that were documented and absent, so the claim "VC cancels" is made to earn
% itself: re-run the identical engagement at a VC three orders of magnitude
% away and require the command history to match.  Not bit-identical -- IEEE
% multiply-then-divide by the same number is not an identity -- but it must
% agree to round-off, and 1e-12 of peak is round-off here (measured: 3e-16).
Cv  = simulate(mergeOpts(o, 'TF',TF, 'HEDEG',-20.0, 'XNT',0.0, 'XNP',XNP, ...
                             'VM',VM, 'VC',137.3, 'H',H, 'Y0',0.0, 'YD0',[]));
dVC = max(abs(Cv.nc - A.nc))/refA;
L{end+1} = '';
L{end+1} = 'C.  VC INVARIANCE   (the listing uses VC and never assigns it)';
L{end+1} = sprintf('      |nc(VC=137.3)-nc(VC=4000)|  %10.3e     (frac of peak; algebraic cancellation)', dVC);
C = addCheck(C, 'C: trajectory independent of VC', dVC, '<= 1e-12 of peak', dVC <= 1e-12);

R    = struct();
R.he = A;  R.man = B;  R.vc = Cv;
R.nl = [];

% ---- D: against the nonlinear Listing 2.1 -------------------------------
if ~o.nonlinear
    L{end+1} = '';
    L{end+1} = 'D.  NONLINEAR CROSS-CHECK **SKIPPED**  (called with ''nonlinear'',false).';
    L{end+1} = '    Fig. 2.7 and Fig. 2.8 are COMPARISONS.  With D skipped, A and B show';
    L{end+1} = '    only that this file integrates its own equations correctly.  They say';
    L{end+1} = '    NOTHING about whether the linearisation approximates the nonlinear';
    L{end+1} = '    model, which is the entire claim of Ch. 2.  Do not report a green';
    L{end+1} = '    suite from here as "Listing 2.2 verified".';
else
    [NLhe, NLman, callNote] = nonlinearPair(XNP, VM, XNT3G);
    L{end+1} = '';
    L{end+1} = sprintf('D.  NONLINEAR CROSS-CHECK   via %s', callNote);

    % D1 -- Fig. 2.7: the two acceleration profiles should essentially
    % coincide.  Compared on NORMALISED time t/tF, because the two models do
    % not agree on what the flight time is: TF is an input here (10 s) and an
    % output there (~10.58 s).  On raw time the tails would be sheared apart
    % for a reason that has nothing to do with the linearisation.
    tauA  = A.t/TF;
    [tauN, gN] = normTrace(NLhe);
    grid1 = linspace(0.02, 0.90, 200);
    gl    = interp1(tauA, A.nc_g, grid1, 'linear');
    gn    = interp1(tauN, gN,     grid1, 'linear');
    rel1  = (gl - gn)./max(abs(gn), eps);
    rms1  = sqrt(mean(rel1.^2));

    % The same comparison with TF handed the nonlinear model's own flight
    % time.  If the residual collapses, the gap above was the TF choice and
    % not the linearisation.  It does not collapse all the way: the small-angle
    % HE vs sin(HE) term is ~2.1 % and stays.  (Strictly the matching time is
    % the nonlinear model's INITIAL range/closing-speed, 10.47 s, not its
    % realised flight time, 10.58 s -- the missile's curved path lengthens the
    % engagement.  Using tf slightly over-corrects.  Both are printed.)
    Am    = simulate(mergeOpts(o, 'TF',NLhe.tf, 'HEDEG',-20.0, 'XNT',0.0, ...
                                   'XNP',XNP, 'VM',VM, 'VC',VC, 'H',H, ...
                                   'Y0',0.0, 'YD0',[]));
    glm   = interp1(Am.t/Am.TF, Am.nc_g, grid1, 'linear');
    rel1m = (glm - gn)./max(abs(gn), eps);
    rms1m = sqrt(mean(rel1m.^2));

    % Reported at tau = 0.02, NOT at each model's own first sample.  engage2d
    % may decimate its array the way Listing 2.1 does (every 0.1 s), in which
    % case its "first sample" is at t = 0.1 and a nc(0)-vs-nc(0) line would
    % silently compare t = 0 against t = 0.1 -- a 2 % shear that looks like a
    % modelling difference.  Both sides come off the common grid.
    L{end+1} = sprintf('    D1  heading error    nonlinear tf = %.4f s   vs linear TF = %.4f s', ...
                       NLhe.tf, TF);
    L{end+1} = sprintf('        linear nc(0)     %8.4f g   (t = 0 exactly, this model only)', A.nc0_g);
    L{end+1} = sprintf('        nc at tau=0.02   linear %8.4f g    nonlinear %8.4f g    ratio %.4f', ...
                       gl(1), gn(1), gl(1)/max(abs(gn(1)), eps));
    L{end+1} = sprintf('        nc at tau=0.02   TF=tf  %8.4f g    nonlinear %8.4f g    ratio %.4f', ...
                       glm(1), gn(1), glm(1)/max(abs(gn(1)), eps));
    L{end+1} = sprintf('        rms rel. diff    book TF = 10     %8.2f %%   (tau in [0.02,0.90])', 100*rms1);
    L{end+1} = sprintf('        rms rel. diff    TF = nonlin tf   %8.2f %%   <- the TF choice, isolated', 100*rms1m);
    C = addCheck(C, 'D1: linear ~ nonlinear (book TF)', rms1, '<= 0.12 rms rel', rms1 <= 0.12);
    C = addCheck(C, 'D1: better at matched TF', rms1m, '<= 0.05 rms rel', rms1m <= 0.05);

    % D2 -- Fig. 2.8.  THIS ONE IS A SIGN CHECK, NOT A TOLERANCE.
    % The book states a DIRECTION: the linear model OVER-predicts the terminal
    % acceleration requirement, because it holds the target's acceleration
    % perpendicular to the LOS at the full maneuver level, while in reality
    % the target turns (BETAD = nT/VT, ~55 deg over this flight) and that
    % component decays.  No number is published for the gap, so this checks the
    % inequality and REPORTS the margin.  Inventing a tolerance here would be
    % inventing a benchmark, which is worse than having none: it would go green
    % or red on a threshold nobody can trace back to the book.
    %
    % Window tau in [0.80,0.95].  Upper end 0.95 not 0.99, because past that
    % the LINEAR side is inside its own unresolvable tail (header table) and
    % the check would start measuring round-off instead of a direction.
    tauB   = B.t/TF;
    [tauNm, gNm] = normTrace(NLman);
    grid2  = linspace(0.80, 0.95, 100);
    gl2    = interp1(tauB,  B.nc_g, grid2, 'linear');
    gn2    = interp1(tauNm, gNm,    grid2, 'linear');
    marg   = gl2 - gn2;
    fracGE = mean(marg >= 0);
    medMar = median(marg);

    L{end+1} = sprintf('    D2  3-g maneuver     nonlinear tf = %.4f s', NLman.tf);
    L{end+1} = sprintf('        nc at tau=0.95   linear %8.4f g    nonlinear %8.4f g', ...
                       gl2(end), gn2(end));
    L{end+1} = sprintf('        DIRECTION        linear >= nonlinear on %5.1f %% of tau in [0.80,0.95]', ...
                       100*fracGE);
    L{end+1} = sprintf('        median margin    %+8.4f g    min %+8.4f g', medMar, min(marg));
    L{end+1} = '        (a direction, not a tolerance -- the book publishes no number here)';
    C = addCheck(C, 'D2: linear OVER-predicts terminal nc', fracGE, '>= 0.90 of window', fracGE >= 0.90);
    C = addCheck(C, 'D2: median terminal margin positive', medMar, '> 0 g', medMar > 0);

    R.nl = struct('he', NLhe, 'man', NLman, 'call', callNote, ...
                  'rms_rel_bookTF', rms1, 'rms_rel_matchedTF', rms1m, ...
                  'tau1', grid1, 'lin1', gl, 'lin1_matchedTF', glm, 'nl1', gn, ...
                  'tau2', grid2, 'lin2', gl2, 'nl2', gn2, ...
                  'frac_linear_ge', fracGE, 'median_margin_g', medMar, ...
                  'matchedTF', Am);
end

% ---- verdict ------------------------------------------------------------
L{end+1} = '';
L{end+1} = repmat('-', 1, 92);
L{end+1} = sprintf('%-42s %14s  %-22s %s', 'check', 'measured', 'want', 'verdict');
for i = 1:numel(C)
    L{end+1} = sprintf('%-42s %14.6g  %-22s %s', ...
        C(i).name, C(i).value, C(i).want, tick(C(i).ok)); %#ok<AGROW>
end
L{end+1} = repmat('-', 1, 92);
L{end+1} = 'A and B are EXACT-SOLUTION checks: the closed forms solve the same ODE the RK2';
L{end+1} = 'loop integrates, so they test the integrator, not the algebra of the listing.';
L{end+1} = 'D2 is a SIGN CHECK.  Fig. 2.8 publishes a direction, not a value.';
L{end+1} = '';
L{end+1} = 'This model has NO LAG, and zero-lag PN misses by zero.  Nothing here is';
L{end+1} = 'evidence about our 25/32 at 0.93 m -- it is the baseline that Ch.3''s guidance';
L{end+1} = 'time constant gets measured against.';

R.checks  = C;
R.pass    = all([C.ok]);
R.summary = strjoin(L, newline);
end


% =========================================================================
function [NLhe, NLman, note] = nonlinearPair(XNP, VM, XNT3G)
%NONLINEARPAIR  Run Listing 2.1 for the two Ch.2 cases via mgs.zarchan.engage2d.
%
%   ASSUMED SIGNATURE (name-value in, scalar struct out), matching the house
%   pattern of PUREPN2D and the variable names of Listing 2.1's data statement:
%
%       NL = mgs.zarchan.engage2d('VM',3000,'VT',1000,'XNT',0,'HEDEG',-20, ...
%                'XNP',4,'RM1',0,'RM2',10000,'RT1',40000,'RT2',10000,'BETA',0);
%
%   with NL carrying a time vector and a command-acceleration vector in g.
%
%   If that call is rejected we retry with only ('XNT','HEDEG','XNP') -- every
%   other value above IS Listing 2.1's data statement, so a faithful port has
%   them as defaults.  WHICH CALL SUCCEEDED IS PRINTED in the D header.  A
%   silent fallback is how you end up comparing against a different engagement
%   than the one you think you ran.
here = fileparts(mfilename('fullpath'));
if ~isfile(fullfile(here, 'engage2d.m'))
    error('mgs:zarchan:engageLinear:noEngage2d', ...
        ['mgs.zarchan.engage2d is not in %s.\n' ...
         'Figs. 2.7 and 2.8 are COMPARISONS against the nonlinear Listing 2.1; ' ...
         'without it there is nothing to compare to, and reporting A/B alone ' ...
         'as "the Ch.2 benchmarks pass" would be a false green.\n' ...
         'Either port Listing 2.1 as engage2d.m, or call\n' ...
         '    mgs.zarchan.engageLinear(''bench'',true,''nonlinear'',false)\n' ...
         'which prints the reduced suite WITH a banner saying what is missing.'], here);
end

fullHE = {'VM',VM, 'VT',1000, 'XNT',0.0,   'HEDEG',-20.0, 'XNP',XNP, ...
          'RM1',0, 'RM2',10000, 'RT1',40000, 'RT2',10000, 'BETA',0};
fullMN = {'VM',VM, 'VT',1000, 'XNT',XNT3G, 'HEDEG',0.0,   'XNP',XNP, ...
          'RM1',0, 'RM2',10000, 'RT1',40000, 'RT2',10000, 'BETA',0};
minHE  = {'XNT',0.0,   'HEDEG',-20.0, 'XNP',XNP};
minMN  = {'XNT',XNT3G, 'HEDEG',0.0,   'XNP',XNP};

try
    NLhe  = normalizeNL(mgs.zarchan.engage2d(fullHE{:}), 'heading-error');
    NLman = normalizeNL(mgs.zarchan.engage2d(fullMN{:}), '3-g maneuver');
    note  = 'mgs.zarchan.engage2d(full Listing 2.1 data statement)';
    return
catch errFull
    try
        NLhe  = normalizeNL(mgs.zarchan.engage2d(minHE{:}), 'heading-error');
        NLman = normalizeNL(mgs.zarchan.engage2d(minMN{:}), '3-g maneuver');
        note  = ['mgs.zarchan.engage2d(XNT,HEDEG,XNP only -- EVERYTHING ELSE ' ...
                 'IS ITS OWN DEFAULT, confirm they are the book''s)'];
        return
    catch errMin
        error('mgs:zarchan:engageLinear:engage2dCall', ...
            ['could not call mgs.zarchan.engage2d.  Both attempts failed.\n\n' ...
             'FULL call ("VM","VT","XNT","HEDEG","XNP","RM1","RM2","RT1",' ...
             '"RT2","BETA") said:\n    %s\n\n' ...
             'MINIMAL call ("XNT","HEDEG","XNP") said:\n    %s\n\n' ...
             'Fix the argument names in nonlinearPair() in this file to match ' ...
             'engage2d, or run with (''nonlinear'',false).  Not guessing ' ...
             'further: a wrong-but-accepted name here would compare against a ' ...
             'different engagement and the numbers would still look sane.'], ...
            errFull.message, errMin.message);
    end
end
end


function N = normalizeNL(S, what)
%NORMALIZENL  Pull (t, nc in g, tf) out of engage2d's struct without guessing.
if ~isstruct(S) || ~isscalar(S)
    error('mgs:zarchan:engageLinear:nlType', ...
        'engage2d returned a %s for the %s case; expected a scalar struct.', ...
        class(S), what);
end
f = fieldnames(S);
t = pickField(S, {'t','T','time','ArrayT','tv'}, 'time vector', what);

% Acceleration.  Field names carrying "g" are taken as g; the bare ones are
% taken as ft/s^2 and divided by 32.2.  Mixing the two up is a factor of 32,
% which shows up as 0.4 g where 13 g belongs -- loud rather than plausible --
% and the range guard below rejects it outright instead of comparing it.
[nc, wasG] = pickAccel(S, what);
if ~wasG, nc = nc/32.2; end

t = t(:);  nc = nc(:);
if numel(t) ~= numel(nc)
    error('mgs:zarchan:engageLinear:nlLen', ...
        ['engage2d (%s): time has %d samples, acceleration has %d.  Refusing ' ...
         'to truncate one to the other -- that silently shifts the whole ' ...
         'profile in normalised time.'], what, numel(t), numel(nc));
end
keep = isfinite(t) & isfinite(nc);
t = t(keep);  nc = nc(keep);
if numel(t) < 10
    error('mgs:zarchan:engageLinear:nlShort', ...
        'engage2d (%s): only %d finite samples; nothing to compare.', what, numel(t));
end
if any(diff(t) <= 0)
    error('mgs:zarchan:engageLinear:nlMono', ...
        ['engage2d (%s): time vector is not strictly increasing, so it cannot ' ...
         'be resampled onto normalised time.'], what);
end
pk = max(abs(nc));
if ~(pk > 0.1 && pk < 1000)
    error('mgs:zarchan:engageLinear:nlUnits', ...
        ['engage2d (%s): peak acceleration reads %.4g "g".  The Ch.2 cases ' ...
         'peak at 4-13 g, so this is almost certainly a units mix-up (32.2x) ' ...
         'or a different quantity entirely.  Fields present: %s'], ...
        what, pk, strjoin(f, ', '));
end

tf = NaN;
for c = {'tf','TF','t_f','tF','flight_time','tflight'}
    if isfield(S, c{1}) && isscalar(S.(c{1})) && isfinite(S.(c{1}))
        tf = double(S.(c{1}));  break
    end
end
if ~isfinite(tf)
    % Falling back to the last recorded sample.  Listing 2.1 records every
    % 0.1 s, so this can sit up to one record interval short of intercept --
    % a ~1 % error in normalised time.  Not hidden: the D header prints tf.
    tf = t(end);
end
if tf <= 0
    error('mgs:zarchan:engageLinear:nlTf', ...
        'engage2d (%s): flight time resolved to %.4g s.', what, tf);
end
N = struct('t', t, 'nc_g', nc, 'tf', tf, 'what', what, 'fields', {f});
end


function [v, wasG] = pickAccel(S, what)
gnames = {'nc_g','xnc_g','XNCG','ArrayXNCG','acc_g','nc_G','a_g','xncg'};
anames = {'nc','XNC','xnc','acc','accel'};
for c = gnames
    if isfield(S, c{1}) && ~isscalar(S.(c{1}))
        v = double(S.(c{1}));  wasG = true;  return
    end
end
for c = anames
    if isfield(S, c{1}) && ~isscalar(S.(c{1}))
        v = double(S.(c{1}));  wasG = false;  return
    end
end
error('mgs:zarchan:engageLinear:nlAccel', ...
    ['engage2d (%s) has no recognisable acceleration history.\n' ...
     'Looked for (as g): %s\n' ...
     'Looked for (as ft/s^2): %s\n' ...
     'Present: %s\n' ...
     'Add the real name to pickAccel() in engageLinear.m rather than letting ' ...
     'this pick whatever vector is handy.'], what, strjoin(gnames, ', '), ...
     strjoin(anames, ', '), strjoin(fieldnames(S), ', '));
end


function v = pickField(S, names, what, ctx)
for c = names
    if isfield(S, c{1}) && ~isscalar(S.(c{1}))
        v = double(S.(c{1}));  return
    end
end
error('mgs:zarchan:engageLinear:nlField', ...
    'engage2d (%s) has no %s.  Looked for: %s.  Present: %s.', ...
    ctx, what, strjoin(names, ', '), strjoin(fieldnames(S), ', '));
end


function [tau, g] = normTrace(N)
tau = N.t(:)/N.tf;
g   = N.nc_g(:);
[tau, iu] = unique(tau, 'stable');
g = g(iu);
end


% =========================================================================
function o = mergeOpts(o, varargin)
for i = 1:2:numel(varargin)
    o.(varargin{i}) = varargin{i+1};
end
end


function C = addCheck(C, name, value, want, ok)
C(end+1) = struct('name', name, 'value', double(value), 'want', want, ...
                  'ok', logical(ok));
end


function s = tick(ok)
if ok, s = 'ok'; else, s = '**FAIL**'; end
end


function s = tf2s(b)
if b, s = 'yes'; else, s = 'NO'; end
end
