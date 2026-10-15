function R = stepAdjointBinomial5(opts)
%MGS.ZARCHAN.STEPADJOINTBINOMIAL5  Zarchan Listing 19.4 -- adjoint of the
%   FIFTH-ORDER BINOMIAL guidance system, miss due to a STEP IN TARGET
%   DISPLACEMENT.  This listing IS Fig. 19.16.
%
%   R = mgs.zarchan.stepAdjointBinomial5()
%   R = mgs.zarchan.stepAdjointBinomial5('TAU', 0.2, 'assert', false)
%
%   CITATIONS.  The two source files do NOT share line numbering in this build
%   (zarchan.txt 15120 lines, zarchan_decoded.txt 15307).  Numbers below are the
%   DECODED file's, raw in parentheses; the offset is a constant +139 through
%   this chapter.  Listing 19.4 is :5943-5951 (raw 5804-5812).
%
% ---------------------------------------------------------------------------
%   WHAT IS NEW AGAINST MGS.ZARCHAN.ADJOINTBINOMIAL5 (Listing 6.1)
%
%   NOT the dynamics.  deriv() here is Listing 6.1's deriv() character for
%   character -- same 5/TAU binomial chain, same XNP*VC gain between X6 and X5,
%   same Y1, same TGO = TP + 1e-5 dodge.  6.1 merely carries x1, x9, x10 too.
%
%   What is new is the OUTPUT MAP, and that is the whole listing.  6.1 reads
%   miss off the integrators x1/x9/x10, the adjoint images of a step/ramp/
%   parabolic TARGET MANEUVER -- a FORCING channel.  19.4 deletes those three
%   states and reads miss off X3 and X4 instead,
%        TOT = DISP*X3 + X4*DISP/(VC*TGO)
%   which are the adjoint images of the forward problem's two INITIAL
%   CONDITIONS: the step in target displacement, and the seeker dish-angle IC
%   it forces (:5837 (raw 5698) splits it that way for the single-lag twin,
%   mgs.zarchan.stepAdjointSingleLag).  DISP = 1 here (200 ft in Listing 19.3),
%   so TOT is already the normalized miss Miss/yTIC that Fig. 19.16 plots.
%
%   UNITS: the SCORED quantity is DIMENSIONLESS -- Miss/yTIC, because DISP = 1.
%   Everything feeding it is in the book's own foot-second-g system (VC ft/s,
%   TAU s, g = 32.2 ft/s^2), and the book's prose restates each normalized miss
%   as "<value> x 200 = <feet>", so a reader converting back gets FEET.
%   ⚠️ Nothing here is metric.  Listing 30.1 is the book's one metric listing
%   (mgs.zarchan.shaping2d); if a number from that port ever meets a number
%   from this one, one of them has to be converted first, and the mixed result
%   would look perfectly plausible -- which is why this line exists.
%   Since x1/x9/x10 are fed BY x2 and feed nothing back, deleting them cannot
%   disturb X2..X8: at matched XNP/TAU/VC/H the two ports must agree state for
%   state -- the natural cross-port check, which belongs in
%   mgs.zarchan.crosscheck, not in a local that cannot see the other port.
%   The benchmark sets are disjoint: 6.1 is scored against Chapter 6's
%   target-maneuver design curves, every row here comes from Fig. 19.16.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION (matlab/zarchan/_listings/listing_19_04.m, whose header
%   declares no unresolved variable-to-variable comparison -- correct, there
%   are none).
%
%     1  "while ~(TP.(TF-.00001))" -> "while TP <= (TF - 1e-5)".  '.' is this
%        book's OCR of '>' (see repair 3).  Shape-forced too: TP only increases.
%     2  "while STEP,=1" -> "while STEP <= 1" (the extractor did this).  ',' is
%        the OCR of '<'; the body sets STEP=2 to leave.
%     3  "if S,.09999" -> "if S >= samp - H/2".  THE EXTRACTOR GOT THIS WRONG:
%        it read ',' as '<' again, but S is incremented by H once per outer
%        step and reset on every record, so "S < .09999" records EVERY step and
%        the 0.09999 could have been any number at all.  Decisive evidence for
%        ">=": the identical statement survives uncorrupted in eleven other
%        listings as "if S.=.09999" (:4738 :4953 :8322 :8554 :9809 :10020
%        :10506 :11142 :13405 ...); only :5949 and 19.3's :5846 were damaged.
%        The literal 0.09999 becomes 'samp' (default 0.1) so a caller's
%        sampling interval means what it says.
%     4  Transcription lines 20-39 are the book's following PROSE plus the
%        Fig. 19.16 AXIS LABELS leaking into the code block (the ligature
%        residue sits there, in "tF = 1").  Dropped, not "fixed" into code.
%     5  plot/xlabel/ylabel/clc/save/disp dropped; nothing is written to disk.
%     6  HE = -DISP/VM and VM = 3000 are computed and NEVER READ: 19.4 dropped
%        19.3's heading-error output (XMHE = -VM*HE*X2) but kept the two lines
%        that fed it.  Carried into R.p, unused.
%     7  States are X2..X8 with no X1 -- the book's own numbering, from 6.1
%        where X1 was the ramp/parabolic state.  The state vector is 8 long
%        with x(1) unused so that x(k) IS Xk.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Every book number was grepped out of the DECODED file; the
%   line is in each row's .source.  Nothing was read off a figure.
%
%   1-6  The six normalized misses the book prints in PROSE at :5978-5984 (raw
%        5839-5845) for Fig. 19.16 at tF/T = 1 and 2:
%             tF/T=1   N'=3 0.811   N'=4 0.748   N'=5 0.686
%             tF/T=2   N'=3 0.212   N'=4 0.0073  N'=5 0.206
%        Each is printed as "<value> x 200 = <feet>" and all six products check
%        out (162/150/137, 42/1.5/41 ft), so the digits are not OCR damage.
%        The 200-ft restatements are DISP times these; not scored twice.
%
%        SIGNS ARE NOT SCORED, MAGNITUDES ARE.  The book prints these products
%        UNSIGNED even when the number is negative -- demonstrably: at :5903
%        (raw 5764) it writes "MissN'=5 = 0.015 x 200 = 3 ft" for the SINGLE-LAG
%        system while its own closed form at :5885-5899, e^-x(1 - 1.5x + x^2/2
%        - x^3/24), gives -0.01533 at x = 1.  Same chapter, same construction,
%        sign dropped.  Fig. 19.16's axis runs 1.0 down to -0.4 (Fig. 19.13's
%        stops at 0.0), so these curves do go negative.  Each value row scores
%        |Miss/yTIC|; the sign content sits in row 7, where it is falsifiable.
%
%        BAND: 0.002 ABSOLUTE, carried as a per-row RELATIVE tolerance because
%        the shared schema's tolerance is relative.  Absolute is right because
%        the quantity CROSSES ZERO -- a percent band on 0.0073 is meaningless.
%        The book prints three significant figures, so its own quantisation is
%        0.0005 and the remaining 4x is discretisation allowance at the
%        listing's own H.  On row 5 (0.0073) that band is 27 percent in
%        relative terms and the row is correspondingly WEAK; it really only
%        asserts that the N'=4 curve crosses zero near tF/T = 2.  Do not widen
%        any of these to get green.
%
%   7    SHAPE, :5978-5984: the family is strictly ordered by N' at tF/T = 1 AND
%        at tF/T = 2 (0.811>0.748>0.686, then 0.212>0.0073>-0.206).  This row
%        carries the sign argument: reading the last value as +0.206 would put
%        N'=4 in a trough at ~0 between two neighbours at ~+0.21, one unit
%        after the family was strictly ordered.  Monotone is the only
%        self-consistent reading, and it makes the N'=5 value negative.
%
%   8    BOUND, :5954 (raw 5815): tF/T "must now be greater than eight for there
%        to be negligible miss distance.  Recall that for the single time
%        constant system the number of guidance time constants had to be
%        greater than three".  "Negligible" is an adjective, so it is given the
%        book's OWN level: the single-lag closed form at x = 3 is exactly
%        -0.5*e^-3 = -0.024894, and that is the miss the book calls negligible.
%        SCORED on the hard half only -- x > 3, and that it settles inside the
%        sweep.  The "eight" is an eyeball off a plot with ticks every two
%        units: reported, not adjudicated.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED -- this file has never
%   been executed; MATLAB is the caller's, not this author's. ***
%     99 samples at TP = 0.10001, 0.20001 ... 9.90001 s at the defaults, so
%     tF/T = 1 and 2 land exactly on samples 10 and 20.  Miss/yTIC starts at
%     essentially 1.0 (Fig. 19.16's "Power Centroid" label) and decays with
%     overshoot below zero; to leading order the departure from 1 is
%     0.87*N'*(t/T)^5, so the first sample is ~1e-3 below unity.  Self-check 4
%     (N' = 0) should print EXACTLY 0 deviation, not merely small.
%   A residual near a band edge is not a pass to be accepted.
%
%   OUTPUTS.  R.t (adjoint time = the original problem's HOMING TIME, s),
%   R.tgo (= t + 1e-5, what the listing divides by), R.x (n-by-7, X2..X8),
%   R.xmy/R.xic (the two halves), R.tot and R.tot_norm = tot/DISP, R.byN (the
%   N' = 3/4/5 sweeps), R.bench, R.selfcheck, R.pass, R.verdict, R.p,
%   R.summary, R.units, R.listing.  'assert' picks the REACTION to a bad
%   verdict and nothing else: true errors, false WARNS and still returns R
%   complete.  benchSuite runs every port with assert off so one red row
%   cannot hide the board.
%
%   See also MGS.ZARCHAN.ADJOINTBINOMIAL5, MGS.ZARCHAN.STEPADJOINTSINGLELAG,
%   MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 19.4 line 1, verbatim (:5943).
    opts.XNP  (1,1) double {mustBeFinite, mustBePositive} = 3.0     % N-prime
    opts.TAU  (1,1) double {mustBeFinite, mustBePositive} = 1.0     % s, TOTAL
    opts.TF   (1,1) double {mustBeFinite, mustBePositive} = 10.0    % s
    opts.DISP (1,1) double {mustBeFinite, mustBeNonzero}  = 1.0     % unit step
    opts.VC   (1,1) double {mustBeFinite, mustBePositive} = 4000.0  % ft/s
    opts.VM   (1,1) double {mustBeFinite, mustBePositive} = 3000.0  % ft/s, DEAD
    opts.H    (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s
    opts.samp double {mustBeReal, mustBeFinite} = []   % [] = the listing's 0.1
    % THE ONLY SWITCHES.  Not nargin: with an arguments block declaring only
    % name-value options MATLAB reports nargin = 0 however many were passed,
    % so an "if nargin == 0" mode here would never fire.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

XNP = opts.XNP;  TAU = opts.TAU;  TF = opts.TF;
DISP = opts.DISP;  VC = opts.VC;  H = opts.H;

samp = opts.samp;
if isempty(samp), samp = 0.1; end
if ~isscalar(samp) || samp <= 0
    error('mgs:zarchan:stepAdjointBinomial5:samp', ...
        'samp must be a positive scalar or [] (the listing''s 0.1 s); got %s.', ...
        mat2str(samp));
end
if H > samp
    error('mgs:zarchan:stepAdjointBinomial5:step', ...
        ['H = %g s exceeds samp = %g s, so the curve would be sampled at the ' ...
         'step rate while claiming the coarser interval.'], H, samp);
end
if H >= TAU/5
    % TAU/5, NOT TAU.  The chain is five lags of TAU/5; a step legal against
    % the total time constant is five times too coarse for the stages, and Heun
    % does not blow up here -- it returns a smooth, entirely wrong curve.  So
    % this is refused rather than warned about.
    error('mgs:zarchan:stepAdjointBinomial5:stiff', ...
        ['H = %g s is not smaller than the BINOMIAL STAGE time constant ' ...
         'TAU/5 = %g s (TAU = %g s is the sum of the five stages).'], H, TAU/5, TAU);
end
if TF <= 2*H
    error('mgs:zarchan:stepAdjointBinomial5:short', ...
        'TF = %g s is not long enough for two steps at H = %g s.', TF, H);
end

% ---- Listing 19.4 itself (:5943-5951) --------------------------------------
[t, X, tgo, xmy, xic, nEval] = adjointRun(XNP, TAU, TF, VC, DISP, H, samp);
if numel(t) < 2
    error('mgs:zarchan:stepAdjointBinomial5:empty', ...
        ['the sweep recorded %d sample(s) at TF = %g s, samp = %g s; at least ' ...
         'two are needed for any curve comparison to mean anything.'], ...
        numel(t), TF, samp);
end

R = struct();
R.listing = 'Zarchan Listing 19.4 (zarchan_decoded.txt:5943-5951), adjoint of Fig. 19.15';
R.units   = 'Miss/yTIC dimensionless (DISP=1); TAU,TF,TP,H in s; VC,VM in ft/s';
R.t = t;  R.tgo = tgo;  R.x = X;
R.xmy = xmy;  R.xic = xic;
R.tot = xmy + xic;
R.tot_norm = R.tot / DISP;
R.p = struct('XNP',XNP, 'TAU',TAU, 'TF',TF, 'DISP',DISP, 'VC',VC, ...
             'VM',opts.VM, 'H',H, 'samp',samp, 'HE',-DISP/opts.VM, ...
             'n_rhs',nEval, 'units',R.units);

[R.byN, R.selfcheck] = sweepsAndChecks(XNP, TAU, TF, VC, DISP, H, samp, R);

if opts.benchmark
    R.bench = benchmarks(R.byN, TAU, TF, samp);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    R.pass  = false;      % NOT true.  "Not checked" must never read as "fine".
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R); end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false: nothing was compared against the book, and R.pass = false means NOT CHECKED.';
elseif ~any(ev)
    msg = sprintf(['no book value could be evaluated: Fig. 19.16''s points need ' ...
        'tF/T = 1 and 2 on the sample grid, and this run has TF/TAU = %g, ' ...
        'samp/TAU = %g.'], TF/TAU, samp/TAU);
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, a wiring fault rather than a tolerance question.  ' ...
        'First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf('%d of %d evaluated book values are outside tolerance (first: %s, book %s, ours %s).', ...
        numel(bad), sum(ev), R.bench(bad(1)).what, R.bench(bad(1)).book_str, ...
        R.bench(bad(1)).ours_str);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:stepAdjointBinomial5:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:stepAdjointBinomial5:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [t, X, tgo, xmy, xic, nEval] = adjointRun(XNP, TAU, TF, VC, DISP, H, samp)
%ADJOINTRUN  Listing 19.4.  Every constant is an ARGUMENT: MATLAB locals cannot
%   see the main function's workspace, and this package has already lost a wave
%   to a local reading a caller's variable.  x(k) IS the book's Xk; x(1) is
%   unused because 19.4 has no X1 (repair 7).
nmax = ceil(TF/min(samp, H)) + 8;
t   = zeros(nmax,1);  X   = zeros(nmax,7);  tgo = zeros(nmax,1);
xmy = zeros(nmax,1);  xic = zeros(nmax,1);

x    = zeros(8,1);
x(3) = 1;             % X3=1 -- THE IMPULSE.  Zero it and every curve comes out
                      % flat, which reads as "no miss" rather than "no run".
tp   = 0.0 + 0.00001; % TP=T+.00001.  NOT 0: X3D carries a 1/TGO.
S    = 0.0;
n    = 0;  nEval = 0;

while tp <= (TF - 1e-5)
    xold = x;
    % Zarchan's two-pass predictor/corrector.  TP advances BETWEEN the passes
    % and is not rolled back, so 1/TGO is evaluated at both times.  Move that
    % line and the scheme quietly degrades to first order, still running.
    xd1 = deriv(x, tp, XNP, TAU, VC);
    x   = x + H*xd1;
    tp  = tp + H;
    xd2 = deriv(x, tp, XNP, TAU, VC);
    % X2=(X2OLD+X2)/2+.5*H*X2D, as the algebraically identical .5*(XOLD+X+H*XD)
    x   = 0.5*(xold + x + H*xd2);
    TGO = tp + 0.00001;
    nEval = nEval + 2;

    S = S + H;                        % ONE increment; ">=" per repair 3
    if S >= samp - 0.5*H
        S = 0.0;
        n = n + 1;
        t(n)   = tp;
        tgo(n) = TGO;
        X(n,:) = x(2:8).';
        xmy(n) = DISP*x(3);                 % XMY=DISP*X3
        xic(n) = x(4)*DISP/(VC*TGO);        % XIC=X4*DISP/(VC*TGO)
    end
end

t = t(1:n);  X = X(1:n,:);  tgo = tgo(1:n);  xmy = xmy(1:n);  xic = xic(1:n);
end


% =============================================================================
function xd = deriv(x, tp, XNP, TAU, VC)
%DERIV  The seven equations of Listing 19.4, from :5948; the book's own line is
%   quoted on the right.  Do not "tidy" the 5/TAU factors -- they ARE the five
%   lags of TAU/5, and folding them changes the ORDER of the system while
%   leaving it looking like a working guidance loop.  Identical to Listing
%   6.1's deriv() minus x1/x9/x10; see the header.
xd    = zeros(8,1);
xd(2) = x(3);                                       % X2D=X3
Y1    = 5.*(5.*x(5)/TAU + x(4))/TAU;                % Y1=5.*(5.*X5/TAU+X4)/TAU
TGO   = tp + 0.00001;                               % TGO=TP+.00001
xd(3) = Y1/(VC*TGO);                                % X3D=Y1/(VC*TGO)
xd(4) = -Y1;                                        % X4D=-Y1
xd(5) = -5.*x(5)/TAU + 5.*x(6)*XNP*VC/TAU;          % X5D
xd(6) = -5.*x(6)/TAU + 5.*x(7)/TAU;                 % X6D
xd(7) = -5.*x(7)/TAU + 5.*x(8)/TAU;                 % X7D
xd(8) = -5.*x(8)/TAU - x(2);                        % X8D
end


% =============================================================================
function [byN, K] = sweepsAndChecks(XNP, TAU, TF, VC, DISP, H, samp, R)
%SWEEPSANDCHECKS  The N' = 3/4/5 sweeps the benchmarks need, plus the
%   structural invariants.  Invariants are SELF-consistency, NOT book values,
%   and must never be quoted as evidence that this port reproduces Zarchan.
byN = struct('xnp', {}, 'x', {}, 'tot_norm', {});
for k = 1:3
    Nk = k + 2;
    [tk, ~, ~, mk, ck] = adjointRun(Nk, TAU, TF, VC, DISP, H, samp);
    byN(k).xnp      = Nk;
    byN(k).x        = tk(:)/TAU;
    byN(k).tot_norm = (mk(:) + ck(:))/DISP;
end

names = {};  oks = [];  detail = {};

% 1  VC CANCELS, exactly.  X5 carries the one factor of VC (the XNP*VC gain),
%    X4 inherits it through Y1, and both readouts divide it back out --
%    X3D = Y1/(VC*TGO), XIC = X4/(VC*TGO).  Nothing else sees VC.  This is the
%    only thing that would notice an edit introducing a real VC dependence.
[~, ~, ~, m2, c2] = adjointRun(XNP, TAU, TF, 7*VC, DISP, H, samp);
d1 = max(abs((m2 + c2)/DISP - R.tot_norm));
names{end+1}  = 'Miss/yTIC independent of VC (VC cancels analytically)';
oks(end+1)    = d1 <= 1e-9;
detail{end+1} = sprintf('max |dTOT| over VC -> 7*VC = %.3e', d1);

% 2  THE BOOK'S OWN VERIFIED HYPOTHESIS (:5954): the normalized curves depend
%    on tF/T alone.  Exact under t = T*s (every state scales as a power of T
%    and the powers close), so halving TAU with TF, H and samp halved must
%    reproduce the curve sample for sample.  The residual is NOT discretisation
%    -- the dimensionless step is identical -- it is the 1e-5 in TGO, which is
%    ABSOLUTE and does not scale.  Compared only for x >= 0.5 for that reason.
[t3, ~, ~, m3, c3] = adjointRun(XNP, TAU/2, TF/2, VC, DISP, H/2, samp/2);
v3  = (m3 + c3)/DISP;
x3  = t3/(TAU/2);
nn  = min(numel(v3), numel(R.tot_norm));
sel = (x3(1:nn) >= 0.5);
if nn >= 10 && any(sel)
    vv = v3(1:nn);  rr = R.tot_norm(1:nn);
    d2 = max(abs(vv(sel) - rr(sel)));
else
    d2 = NaN;
end
names{end+1}  = 'Miss/yTIC is a function of tF/T alone (book :5954)';
oks(end+1)    = isfinite(d2) && d2 <= 1e-4;
detail{end+1} = sprintf('max |dTOT| over TAU -> TAU/2 (x>=0.5) = %.3e', d2);

% 3  ZERO HOMING TIME MEANS FULL DISPLACEMENT -- Fig. 19.16's "Power Centroid"
%    label at 1.0 (:5954: "a normalized miss of unity means we are hitting the
%    power centroid").  X3(0)=1 and X4(0)=0, so TOT(0)=DISP identically.  The
%    departure is FIFTH order, ~0.87*N'*(t/T)^5 (derived here, not measured),
%    about 1e-3 at the H < TAU/5 limit -- so 0.02 is a limit check, not a band
%    fitted to an observation.
[t4, ~, ~, m4, c4] = adjointRun(XNP, TAU, 5*H, VC, DISP, H, H);
if isempty(t4)
    d3 = NaN;
else
    d3 = abs((m4(1) + c4(1))/DISP - 1);
end
names{end+1}  = 'Miss/yTIC -> 1 as homing time -> 0 (Fig. 19.16 power centroid)';
oks(end+1)    = isfinite(d3) && d3 <= 0.02;
detail{end+1} = sprintf('|TOT/DISP - 1| at t = %.4g s is %.3e', firstOr(t4, NaN), d3);

% 4  DELETE EXPERIMENT.  Remove the guidance gain (N' = 0) and the answer must
%    become EXACTLY 1 at every sample: X5 stays 0, so Y1 = 5*X4/TAU with
%    X4(0) = 0 keeps X4 at 0, so X3D = 0 and X3 stays at its impulse of 1.  A
%    missile that is not guiding hits the power centroid, every time.  This is
%    what proves the XNP*VC gain and the X4/X5 pair are WIRED INTO the readout
%    rather than merely present -- 0 deviation, not "small".
[~, ~, ~, m5, c5] = adjointRun(0.0, TAU, TF, VC, DISP, H, samp);
d4 = max(abs((m5 + c5)/DISP - 1));
names{end+1}  = 'DELETE N''=0: unguided miss is exactly the full displacement';
oks(end+1)    = d4 <= 1e-12;
detail{end+1} = sprintf('max |TOT/DISP - 1| with the gain removed = %.3e', d4);

K = struct();
K.names   = names;
K.oks     = logical(oks(:)).';
K.details = detail;
K.all_ok  = all(K.oks);
bad = find(~K.oks, 1);
if isempty(bad)
    K.first_bad = '';
else
    K.first_bad = sprintf('%s (%s)', names{bad}, detail{bad});
end
end


% =============================================================================
function B = benchmarks(byN, TAU, TF, samp)
%BENCHMARKS  Fig. 19.16, :5978-5984 (raw 5839-5845).  See the header for the
%   band's provenance and for why magnitudes, not signs, are scored.
BANDABS = 0.002;                 % ABSOLUTE in Miss/yTIC
SRC1 = 'Fig. 19.16, p.433 prose; zarchan_decoded.txt:5978-5980 (raw 5839-5841)';
SRC2 = 'Fig. 19.16, p.433 prose; zarchan_decoded.txt:5982-5984 (raw 5843-5845)';
SRC3 = 'Fig. 19.16 prose, p.432; zarchan_decoded.txt:5954 (raw 5815)';
BOOK1 = [0.811 0.748 0.686];
BOOK2 = [0.212 0.0073 0.206];

B  = emptyBench();
v1 = nan(1,3);  v2 = nan(1,3);
for k = 1:3
    v1(k) = readAt(byN(k), 1.0, samp/TAU);
    v2(k) = readAt(byN(k), 2.0, samp/TAU);
end

offGrid = sprintf('tF/T is not on the sample grid: TF/TAU = %g, samp/TAU = %g.', ...
                  TF/TAU, samp/TAU);
for k = 1:3
    what = sprintf('|Miss/yTIC| at tF/T = 1, N'' = %d', byN(k).xnp);
    if ~isfinite(v1(k))
        B = addSkip(B, what, sprintf('%.4g', BOOK1(k)), offGrid, SRC1, '-');
    else
        B = addVal(B, what, BOOK1(k), abs(v1(k)), 'abs 0.002 in Miss/yTIC', ...
            BANDABS/abs(BOOK1(k)), SRC1, '-', sprintf('signed value %+.5f', v1(k)));
    end
end
for k = 1:3
    what = sprintf('|Miss/yTIC| at tF/T = 2, N'' = %d', byN(k).xnp);
    if ~isfinite(v2(k))
        B = addSkip(B, what, sprintf('%.4g', BOOK2(k)), offGrid, SRC2, '-');
    else
        if k == 2
            % Said out loud rather than hidden, so nobody quotes this row as a
            % three-figure agreement: 0.002 absolute is 27 percent of 0.0073.
            dtl = sprintf(['signed value %+.5f -- the 0.002 band is 27 percent ' ...
                'here, so what this row really asserts is that the N''=4 curve ' ...
                'crosses zero near tF/T = 2'], v2(k));
        else
            dtl = sprintf('signed value %+.5f', v2(k));
        end
        B = addVal(B, what, BOOK2(k), abs(v2(k)), 'abs 0.002 in Miss/yTIC', ...
            BANDABS/abs(BOOK2(k)), SRC2, '-', dtl);
    end
end

% ORDERING.  This is where the SIGNS live; see the header.
if all(isfinite(v1)) && all(isfinite(v2))
    ok = all(diff(v1) < 0) && all(diff(v2) < 0);
    B = addShape(B, 'Fig. 19.16 family strictly ordered by N'' at tF/T = 1 and 2', ...
        '0.811>0.748>0.686 and 0.212>0.0073>-0.206', ...
        sprintf('[%+.4f %+.4f %+.4f] / [%+.4f %+.4f %+.4f]', v1, v2), ok, SRC1, ...
        ['reading the N''=5 x=2 value as +0.206 would put N''=4 in a trough at ' ...
         '~0 between two neighbours at ~+0.21']);
else
    B = addSkip(B, 'Fig. 19.16 family strictly ordered by N''', 'monotone in N''', ...
        offGrid, SRC1, '-');
end

% TIME CONSTANTS NEEDED BEFORE THE MISS IS "NEGLIGIBLE".
NEGL = 0.5*exp(-3);        % single-lag closed form at its own x = 3, :5885-5899
if TF/TAU >= 4 && ~isempty(byN(1).x)
    idx = find(abs(byN(1).tot_norm) > NEGL, 1, 'last');
    if isempty(idx)
        xn = 0;
    elseif idx >= numel(byN(1).x)
        xn = Inf;                          % never settles inside the sweep
    else
        xn = byN(1).x(idx);
    end
    ok = isfinite(xn) && xn > 3 && xn <= TF/TAU;
    B = addBound(B, 'guidance time constants before miss is negligible (N''=3)', ...
        '> 8 (single lag needed > 3)', sprintf('%.2f', xn), ok, SRC3, 'tF/T', ...
        sprintf(['scored on the hard half only (x > 3, and settles inside the ' ...
                 'sweep); "negligible" is fixed at |Miss/yTIC| <= %.5f, the ' ...
                 'single-lag value at its own x = 3'], NEGL));
else
    B = addSkip(B, 'guidance time constants before miss is negligible', '> 8', ...
        sprintf('sweep covers only tF/T <= %g; at least 4 is needed to test "> 3".', ...
                TF/TAU), SRC3, 'tF/T');
end
end


% =============================================================================
function v = readAt(S, xt, xtol)
%READAT  NEAREST recorded sample to tF/T = xt, not interpolation: interpolating
%   would blur a real disagreement into a smaller one.
v = NaN;
if isempty(S.x), return; end
[d, j] = min(abs(S.x - xt));
if d <= xtol + 1e-9
    v = S.tot_norm(j);
end
end

function v = firstOr(a, dflt)
if isempty(a), v = dflt; else, v = a(1); end
end


% =============================================================================
%   ROW CONSTRUCTION.  ONE constructor, so the field ORDER cannot drift --
%   benchSuite's adaptGeneric is strict and a near-miss schema errors.
% =============================================================================
function B = emptyBench()
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = mkRow(B, what, kind, book, bookStr, ours, oursStr, pct, band, ...
                   tol, ok, skipped, why, source, units, detail)
i = numel(B) + 1;
B(i).what    = what;              B(i).kind     = kind;
B(i).book    = book;              B(i).book_str = bookStr;
B(i).ours    = ours;              B(i).ours_str = oursStr;
B(i).pct     = pct;               B(i).band     = band;
B(i).tol     = tol;               B(i).ok       = logical(ok);
B(i).skipped = logical(skipped);  B(i).why      = why;
B(i).source  = source;            B(i).units    = units;
B(i).detail  = detail;
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
% Tolerance arrives EXPLICITLY: the bands here are absolute at the call site,
% and a band->tol lookup would need a second copy of that arithmetic.
if ~isfinite(ours) || book == 0
    B = mkRow(B, what, 'value', book, sprintf('%.4g', book), ours, '--', NaN, ...
        band, tol, false, true, ...
        'the computed value is not finite (or the book value is zero)', ...
        source, units, detail);
    return
end
B = mkRow(B, what, 'value', book, sprintf('%.4g', book), ours, ...
    sprintf('%.5g', ours), 100*(ours/book - 1), band, tol, ...
    abs(ours/book - 1) <= tol, false, '', source, units, detail);
end

function B = addShape(B, what, bookStr, oursStr, ok, source, detail)
B = mkRow(B, what, 'shape', NaN, bookStr, NaN, oursStr, NaN, 'shape', NaN, ...
    ok, false, '', source, '-', detail);
end

function B = addBound(B, what, bookStr, oursStr, ok, source, units, detail)
B = mkRow(B, what, 'bound', NaN, bookStr, NaN, oursStr, NaN, 'bound', NaN, ...
    ok, false, '', source, units, detail);
end

function B = addSkip(B, what, bookStr, why, source, units)
%ADDSKIP  A check that COULD NOT RUN.  Never a pass: counted as a check that
%   did not happen, which is a different thing from one that succeeded.
B = mkRow(B, what, 'skip', NaN, bookStr, NaN, '--', NaN, 'skip', NaN, ...
    false, true, why, source, units, '');
end


% =============================================================================
function s = render(R, ev)
p = R.p;
L = {};
L{end+1} = 'mgs.zarchan.stepAdjointBinomial5 -- Zarchan Listing 19.4 (Fig. 19.16)';
L{end+1} = sprintf('  N''=%g  TAU=%g s  TF=%g s  DISP=%g  VC=%g ft/s  H=%g s  samp=%g s', ...
                   p.XNP, p.TAU, p.TF, p.DISP, p.VC, p.H, p.samp);
L{end+1} = sprintf('  %d samples, %d rhs evaluations;  VM=%g, HE=%g computed by the listing and never read', ...
                   numel(R.t), p.n_rhs, p.VM, p.HE);
L{end+1} = sprintf('  units: %s', R.units);
L{end+1} = sprintf('  Miss/yTIC: first %.5f  last %.5f  min %.5f  max %.5f', ...
                   R.tot_norm(1), R.tot_norm(end), min(R.tot_norm), max(R.tot_norm));
L{end+1} = '  structural invariants (self-consistency, NOT book values):';
for i = 1:numel(R.selfcheck.names)
    L{end+1} = sprintf('    [%s] %s -- %s', tick(R.selfcheck.oks(i)), ...
                       R.selfcheck.names{i}, R.selfcheck.details{i});  %#ok<AGROW>
end
if isempty(R.bench)
    L{end+1} = '  benchmarks: NOT RUN';
else
    L{end+1} = '  benchmarks vs the book:';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        if b.skipped
            L{end+1} = sprintf('    [SKIP] %-50s %s', b.what, b.why);  %#ok<AGROW>
        else
            L{end+1} = sprintf('    [%s] %-50s book %-9s ours %-9s', ...
                tick(b.ok), b.what, b.book_str, b.ours_str);           %#ok<AGROW>
        end
    end
    L{end+1} = sprintf('  %d of %d rows evaluated', sum(ev), numel(R.bench));
end
s = strjoin(L, newline);
end

function s = tick(ok)
if ok, s = ' ok '; else, s = 'FAIL'; end
end


% =============================================================================
function drawCurves(R)
figure;
plot(R.t, R.tot_norm, 'LineWidth', 1.2);  grid on;
xlabel('Homing time t_F (s)');
ylabel('Miss / y_{TIC}');
title(sprintf('Listing 19.4, fifth-order binomial, N'' = %g, T = %g s', ...
              R.p.XNP, R.p.TAU));
end
