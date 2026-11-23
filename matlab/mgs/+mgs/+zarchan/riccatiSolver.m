function R = riccatiSolver(opts)
%MGS.ZARCHAN.RICCATISOLVER  Zarchan Listing 9.1 -- solving the RICATTI
%   EQUATIONS for the gains of the three-state discrete Kalman filter of
%   Figs. 9.1/9.2.
%
%   R = mgs.zarchan.riccatiSolver()
%   R = mgs.zarchan.riccatiSolver('SIGRIN', 0.01)          the Fig. 9.6 case
%   R = mgs.zarchan.riccatiSolver('TS', 0.01, 'assert', false)
%
%   Line-by-line port of zarchan.txt:2793, :2799-2801.  Every ":NNNN" here is a
%   zarchan.txt line; the VALUES were read from zarchan_decoded.txt, where the
%   maths font is repaired.  ⚠️ MEASURED 2026-08-24: the two files are NOW
%   line-for-line ALIGNED (15120 lines each; :2793 and :9341 carry the same
%   content in both), so a line cited from either is valid.  That was not true
%   earlier in the day -- optimalGainsSingleLag's header warns of a 187-line
%   drift, and kalmanLawOptions cites Listing 29.3 at :10953 where it now sits
%   at :10765.  The decoded file was re-extracted; RE-CHECK any older citation.
%
%   UNITS ARE THE BOOK'S: ft, ft/s, ft/s^2, s, rad; g = 32.2 ft/s^2; HEDEG in
%   DEGREES and divided by 57.3, as the listing does.  Nothing is converted.
%
%   *** THIS IS NOT THE CH.26 RICCATI EQUATION. ***  The brief asks whether this
%   is the recursion mgs.zarchan.optimalGainsSingleLag integrates.  It is not:
%   Ch.9 is ESTIMATION -- a discrete matrix DIFFERENCE equation for the
%   covariance of the errors in the estimate, stepped forward in Ts, driven by
%   process noise Q and measurement noise R, output M H'/(H M H' + R).  Ch.26
%   (:9435) is CONTROL -- a nonlinear matrix DIFFERENTIAL equation with a
%   quadratic term, no Q, no measurement noise, output B^-1 G'S.  Duals, not
%   the same object; no row here touches Ch.26.
%   The recursion IDENTICAL to this one is in Listings 9.2, 25.1, 25.2, 29.3,
%   29.4, 30.2 -- of which mgs.zarchan.kalmanLawOptions (29.3) is ported and
%   exposes R.kalman.K at the SAME parameters.  Still not a free cross-check:
%   29.3 takes its first step one sample LATER (first TGO = tF-Ts, ours tF), so
%   the two are the same recursion on schedules shifted by one step and differ
%   by up to 16%.  What they do is CONVERGE -- SELF-CHECK 5, on our own shifted
%   rerun, so that no 100-run sweep is paid for inside this file.
%
%   *** WHAT IT IS NOT. ***  Zero-lag, planar, constant closing velocity, no
%   guidance loop at all: 9.1 solves the covariance recursion open-loop and
%   off-line, and its gains are optimal only if the target really maneuvers as
%   white-noise-through-an-integrator with Phi_s = nT^2/tF (:2626).
%
% --- TRANSCRIPTION.  listing_09_01.m reports no unresolved variable-to-variable
%   comparison, and that is right -- its only comparison is against an
%   expression.  ONE repair, forced by shape: "while T , =(TF-1e-5)" -> "<=";
%   "," is this book's OCR of "<" and T only increases from 0, so ">=" never
%   terminates.  Four book quirks, deliberate: "H=.01; S=0." are set and NEVER
%   READ (9.1 has no continuous section; carried over from 9.2), dropped rather
%   than "fixed"; "SIGNOISE=SIGRIN" is loop-invariant, kept inline; ArrayT(n)=T
%   is recorded BEFORE T=T+TS, so TGO = TF-T+1e-6 runs 10.000001 down to
%   0.100001 and T is ACCUMULATED as the listing does, not linspaced; plot/clc/
%   "save datfil.txt -ascii" are I/O and nothing is written to disk.
%   MINUS-SIGN DAMAGE (the digit 2 standing in for "-"): looked for, none found
%   -- every constant is confirmed by prose at :2785 (SIGRIN=.001 is the 1 mr,
%   XNT=96.6 the 3 g = 3*32.2 exactly, TF=10 the 10-s flight), and HEDEG=20 by
%   the P0 equation at :2778 plus HEDEGFIL=20 in Listing 9.2.
%
% --- BENCHMARKS.  ⚠️ Ch.9 publishes almost NO number for this listing: Figs.
%   9.3 and 9.6 are gain PLOTS and every figure became an image link in the
%   extraction, so the gain values themselves are unscoreable (ROW 9, a SKIP).
%   That is the honest yield of the chapter, not a shortfall of the port.  What
%   IS greppable is the surrounding EQUATIONS and three prose claims:
%     1 the listing IS the scalar expansion of the published matrix recursion
%       (:2785) -- scored against an INDEPENDENT matrix implementation of
%       :2730-2754, :2758-2766, :2768-2773.  Sharpest row here: the only one
%       that checks the seven powers of Ts in the M terms.
%     2 nT = 3 g, Phi_s = nT^2/tF (:2785, :2626)
%     3 P0 = diag([sigma_noise^2, (VM*HE/57.3)^2, nT^2]) (:2778)
%     4 "the Kalman filter has time-varying gains" (:2786)
%     5 "after an initial transient period, the gains appear to be
%       monotonically increasing" (:2786) -- an ORDERING, on all three gains
%     6 "it takes about 3 s for the Kalman gains to go through their initial
%       transient period" (:2806) -- +/-1.5 s, because the word is "about"
%     7 "10 mr of angle noise rather than 1 mr ... the first Kalman gain value
%       is approximately halved" (:2807)
%   ROW 5 is tested at t >= 4 s, NOT the prose's 3 s: K1's last decrease is at
%   3.6 s, so the literal 3 s would fail a faithful port by one 0.1-s step.  4 s
%   is "about 3" plus one figure-reading tick; ROW 6 carries the 3 vs 3.6.
%   ROW 7 is a BOUND: the true median ratio is 0.62, so "halved" is itself ~25%
%   optimistic and scoring against 0.500 would pretend the prose has three
%   digits.  [0.35, 0.75] excludes ratio = 1 (SIGRIN never reaching the gain).
%   ⚠️ It does NOT catch SIGRIN reaching the running noise but not P0 (0.60).
%
% --- WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED IN MATLAB. ***  This
%   file has never been executed; the numbers come from an independent
%   transcription of the same recursion in double precision, which MATLAB
%   should reproduce to roundoff (100 steps of arithmetic, no library calls).
%     100 samples, t = 0, 0.1, ..., 9.9 s
%     K1  0.88704 at t=0, minimum 0.28801 at t=3.6 s, 0.89845 at t=9.9 s
%     K2  7.74105 at t=0, minimum 0.44224 at t=3.4 s, 4.67166 at t=9.9 s
%     K3  0.00331 at t=0, last decrease at t=3.2 s,   9.83302 at t=9.9 s
%     P0 diagonal 1600 / 1096461.17 / 9331.56;  Phi_s = 933.156
%     sqrt(P11) 37.67 -> 11.29 -> 0.379 ft at t = 0, 4.9, 9.9 s
%     ROW 1 residual 8.0e-16 on the gains, 1.5e-15 on P (band 1e-12)
%     ROW 4 max/min K1 = 3.1195   ROW 6 onset 3.6 s   ROW 7 ratio 0.6233
%     SELF-CHECK 3 (Q deleted): still falling at 6.8 s, ends 0.7292, max 52%
%     SELF-CHECK 4 (range scaling deleted): falls all flight, 0.2354, max 74%
%     SELF-CHECK 5 (one-step grid shift): 16.0% worst, 0.103% after t = 3.96 s
%   A residual near a band EDGE is not a pass to be accepted, and no band here
%   is to be widened to turn a row green.
%
% --- OUTPUTS.  R.t R.tgo [s]; R.K n-by-3 [K1 K2 K3]; R.P 3-by-3-by-n after
%   each update; R.Pdiag; R.sig_pos (the range-scaled position noise); R.P0;
%   R.hi (the 10 mr rerun: .SIGRIN .K1 .ratio, [] if disabled); R.bench
%   R.selfcheck R.pass R.verdict R.p R.units R.summary.
%
%   See also MGS.ZARCHAN.KALMANLAWOPTIONS (Listing 29.3 -- the same recursion
%   inside a homing loop), MGS.ZARCHAN.OPTIMALGAINSSINGLELAG (Listing 26.1 --
%   a DIFFERENT Riccati equation; see above), MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 9.1 line 1 (zarchan.txt:2793, :2799), verbatim.
    opts.VC     (1,1) double {mustBeFinite, mustBePositive}    = 4000.0  % ft/s
    opts.XNT    (1,1) double {mustBeFinite, mustBePositive}    = 96.6    % ft/s^2
    opts.VM     (1,1) double {mustBeFinite, mustBePositive}    = 3000.0  % ft/s
    opts.HEDEG  (1,1) double {mustBeFinite, mustBeNonnegative} = 20.0    % deg
    opts.SIGRIN (1,1) double {mustBeFinite, mustBePositive}    = 0.001   % rad
    opts.TS     (1,1) double {mustBeFinite, mustBePositive}    = 0.1     % s
    opts.TF     (1,1) double {mustBeFinite, mustBePositive}    = 10.0    % s
    % The Fig. 9.6 comparison (:2807).  [] disables it and BENCH ROW 7 skips.
    opts.SIGRIN_HI double {mustBeReal} = 0.01                            % rad
    % THE ONLY SWITCHES.  Not nargin: with an arguments block that declares
    % only name-value options, MATLAB reports nargin = 0 however many were
    % passed, so an `if nargin == 0` mode here would never fire.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

VC = opts.VC;  XNT = opts.XNT;  VM = opts.VM;  HEDEG = opts.HEDEG;
SIGRIN = opts.SIGRIN;  TS = opts.TS;  TF = opts.TF;

if TS > TF
    error('mgs:zarchan:riccatiSolver:step', ...
        ['TS = %g s exceeds the flight time TF = %g s, so the recursion would ' ...
         'take no step at all and hand back an empty gain table -- which ' ...
         'reads exactly like a converged answer.'], TS, TF);
end
hiS = opts.SIGRIN_HI;
if ~isempty(hiS) && (~isscalar(hiS) || hiS <= 0)
    error('mgs:zarchan:riccatiSolver:sigrinhi', ...
        'SIGRIN_HI must be a positive scalar or [] (meaning "do not run it"); got %s.', ...
        mat2str(hiS));
end

% =========================================================================
%   LISTING 9.1.  zarchan.txt:2793, :2799-2801.
% =========================================================================
[t, tgo, K, P3, sig_pos, P0] = riccatiRun(VC, XNT, VM, HEDEG, SIGRIN, TS, TF, ...
                                          0.0, 1.0, false);
n = numel(t);
if n < 3
    error('mgs:zarchan:riccatiSolver:short', ...
        ['the recursion recorded %d sample(s) at TF = %g s, TS = %g s.  Every ' ...
         'shape row below needs a curve, not a point.'], n, TF, TS);
end
Pdiag = [squeeze(P3(1,1,:)), squeeze(P3(2,2,:)), squeeze(P3(3,3,:))];

% The 10 mr case is a rerun of the WHOLE listing -- P0 included, because that
% is what rerunning it does.  See ROW 7 on the variant this cannot see.
HI = [];
if ~isempty(hiS)
    [~, ~, Khi] = riccatiRun(VC, XNT, VM, HEDEG, hiS, TS, TF, 0.0, 1.0, false);
    m = min(size(Khi,1), n);
    HI = struct('SIGRIN', hiS, 'K1', Khi(1:m,1), 'ratio', Khi(1:m,1)./K(1:m,1));
end

R = struct();
R.t = t;  R.tgo = tgo;  R.K = K;  R.P = P3;
R.Pdiag = Pdiag;  R.sig_pos = sig_pos;  R.P0 = P0;  R.hi = HI;
R.units = ['ft, ft/s, ft/s^2, s, rad; g = 32.2 ft/s^2; HEDEG in deg (/57.3).  ' ...
           'No random numbers are used, so there is no seed to record.'];
R.p = struct('VC',VC, 'XNT',XNT, 'VM',VM, 'HEDEG',HEDEG, 'SIGRIN',SIGRIN, ...
             'TS',TS, 'TF',TF, 'PHIN',XNT*XNT/TF, 'SIGRIN_HI',hiS, ...
             'n_updates',n, 'units',R.units);

R.selfcheck = structuralChecks(R, VC, XNT, VM, HEDEG, SIGRIN, TS, TF);

if opts.benchmark
    R.bench = benchmarks(R, VC, XNT, VM, HEDEG, SIGRIN, TS, TF, hiS);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  "Not checked" must never render as "fine".
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawGains(R); end

msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the book.  ' ...
           'R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated: the scored prose ' ...
        'rows in Ch.9 are tied to the listing''s own case, and this run used ' ...
        'VC=%g, nT=%g, VM=%g, HE=%g deg, sigrin=%g, Ts=%g, tF=%g.'], ...
        VC, XNT, VM, HEDEG, SIGRIN, TS, TF);
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
        error('mgs:zarchan:riccatiSolver:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN COMPLETE.  benchSuite runs every port with assert off
        % precisely so that one red row cannot hide the rest of the board.
        warning('mgs:zarchan:riccatiSolver:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [t, tgo, K, P3, sig_pos, P0] = riccatiRun(VC, XNT, VM, HEDEG, SIGRIN, ...
                                          TS, TF, T0, phinScale, freezeNoise)
%RICCATIRUN  Listing 9.1, zarchan.txt:2793 and :2799-2801, listing order.
%
%   Every constant is an ARGUMENT.  MATLAB local functions cannot see the main
%   function's workspace; a previous wave in this package shipped a local that
%   read one from the caller and threw on the first line of the first run.
%
%   T0          start time (0 = the listing; TS = Listing 29.3's schedule)
%   phinScale   0 DELETES the process noise            -- SELF-CHECK 3
%   freezeNoise holds SIGN2 at its t = 0 value, deleting the range scaling of
%               the measurement noise                  -- SELF-CHECK 4

TS2 = TS*TS;  TS3 = TS2*TS;  TS4 = TS3*TS;  TS5 = TS4*TS;
PHIN = phinScale*XNT*XNT/TF;

RTM = VC*TF;  SIGNOISE = SIGRIN;  SIGPOS = RTM*SIGNOISE;  SIGN2 = SIGPOS^2;
SIGN2_0 = SIGN2;                        % only read when freezeNoise is true
P11 = SIGN2;  P12 = 0.;  P13 = 0.;
P22 = (VM*HEDEG/57.3)^2;  P23 = 0.;  P33 = XNT*XNT;
P0 = [P11 P12 P13; P12 P22 P23; P13 P23 P33];

nmax = floor(max(TF - 1e-5 - T0, 0)/TS) + 2;
t = zeros(nmax,1);  tgo = zeros(nmax,1);  K = zeros(nmax,3);
P3 = zeros(3,3,nmax);  sig_pos = zeros(nmax,1);
T = T0;  n = 0;

while T <= (TF - 1e-5)
    TGO = TF - T + .000001;
    RTM = VC*TGO;  SIGNOISE = SIGRIN;  SIGPOS = RTM*SIGNOISE;  SIGN2 = SIGPOS^2;
    if freezeNoise, SIGN2 = SIGN2_0; end

    M11 = P11 + TS*P12 + .5*TS2*P13 + TS*(P12 + TS*P22 + .5*TS2*P23);
    M11 = M11 + .5*TS2*(P13 + TS*P23 + .5*TS2*P33) + TS5*PHIN/20.;
    M12 = P12 + TS*P22 + .5*TS2*P23 + TS*(P13 + TS*P23 + .5*TS2*P33) + TS4*PHIN/8.;
    M13 = P13 + TS*P23 + .5*TS2*P33 + PHIN*TS3/6.;
    M22 = P22 + TS*P23 + TS*(P23 + TS*P33) + PHIN*TS3/3.;
    M23 = P23 + TS*P33 + .5*TS2*PHIN;
    M33 = P33 + PHIN*TS;

    K1 = M11/(M11 + SIGN2);  K2 = M12/(M11 + SIGN2);  K3 = M13/(M11 + SIGN2);

    P11 = (1.-K1)*M11;  P12 = (1.-K1)*M12;  P13 = (1.-K1)*M13;
    P22 = -K2*M12 + M22;  P23 = -K2*M13 + M23;  P33 = -K3*M13 + M33;

    n = n + 1;
    t(n) = T;  tgo(n) = TGO;  K(n,:) = [K1 K2 K3];  sig_pos(n) = SIGPOS;
    P3(:,:,n) = [P11 P12 P13; P12 P22 P23; P13 P23 P33];
    T = T + TS;                         % accumulated, as the listing does
end

t = t(1:n);  tgo = tgo(1:n);  K = K(1:n,:);
P3 = P3(:,:,1:n);  sig_pos = sig_pos(1:n);
end


% =============================================================================
function [Km, Pm] = riccatiMatrix(VC, XNT, VM, HEDEG, SIGRIN, TS, TF)
%RICCATIMATRIX  The BOOK'S MATRIX equations, not the listing's scalar ones.
%
%   Phi_k and Q_k from zarchan.txt:2730-2754, K_k from :2758, P_k from :2768.
%   This shares no line with riccatiRun, which is the entire point: :2785
%   claims the listing IS this recursion "expanded to scalar form", and BENCH
%   ROW 1 is what turns that claim into a measurement.

Phi = [1 TS .5*TS*TS; 0 1 TS; 0 0 1];
Q   = (XNT*XNT/TF) * [TS^5/20  TS^4/8   TS^3/6;
                      TS^4/8   TS^3/3   TS^2/2;
                      TS^3/6   TS^2/2   TS    ];
Hm  = [1 0 0];
P   = diag([(VC*TF*SIGRIN)^2, (VM*HEDEG/57.3)^2, XNT*XNT]);

nmax = floor(max(TF - 1e-5, 0)/TS) + 2;
Km = zeros(nmax,3);  Pm = zeros(3,3,nmax);  T = 0;  n = 0;
while T <= (TF - 1e-5)
    TGO = TF - T + .000001;
    Rk  = (VC*TGO*SIGRIN)^2;
    M   = Phi*P*Phi.' + Q;
    Kk  = M*Hm.'/(Hm*M*Hm.' + Rk);      % scalar denominator; no inv() needed
    P   = (eye(3) - Kk*Hm)*M;
    n = n + 1;  Km(n,:) = Kk.';  Pm(:,:,n) = P;
    T = T + TS;
end
Km = Km(1:n,:);  Pm = Pm(:,:,1:n);
end


% =============================================================================
function tm = monoOnset(t, x)
%MONOONSET  The earliest recorded time from which x never decreases again.
j = find(diff(x) < -1e-12*max(abs(x)), 1, 'last');
if isempty(j), tm = t(1); else, tm = t(j+1); end
end

function ok = monoAfter(t, X, t0)
%MONOAFTER  Every column of X non-decreasing over t >= t0.
m = t >= t0;
ok = true;
for c = 1:size(X,2)
    d = diff(X(m,c));
    ok = ok && (isempty(d) || all(d >= -1e-12*max(abs(X(:,c)))));
end
end

function [ok, txt] = deleteBroke(t, base, mut, tlate)
%DELETEBROKE  A mutation has to be VISIBLE somewhere a reader looks.
%   Preferred evidence: the book's monotone-after-the-transient claim (BENCH
%   ROW 5) FLIPS.  If the baseline itself is not monotone there -- which only
%   happens off the listing's own case -- that test would pass VACUOUSLY, so
%   the fallback is a material change in the gain.  Never a pass by default.
relch = max(abs(mut - base)./max(abs(base), realmin));
if monoAfter(t, base, tlate)
    ok  = ~monoAfter(t, mut, tlate);
    txt = sprintf('shape row flips (%d); K1(end) %.4f -> %.4f, max change %.1f%%', ...
        ok, base(end), mut(end), 100*relch);
else
    ok  = relch >= 0.05;
    txt = sprintf(['baseline is not monotone here, so scored on size instead: ' ...
        'max change %.1f%% (>= 5%% required)'], 100*relch);
end
end


% =============================================================================
function K = structuralChecks(R, VC, XNT, VM, HEDEG, SIGRIN, TS, TF)
%STRUCTURALCHECKS  Self-consistency, and two DELETE EXPERIMENTS.  These are NOT
%   book values and must never be quoted as evidence that this port reproduces
%   Zarchan; they are the weaker claim that it agrees with itself.
%   ⚠️ .names and .oks are what benchSuite puts on the board.  A delete
%   experiment nobody can see is the same as one that was never run.
K = struct();
names = {};  oks = [];  detail = {};
t = R.t;  Kg = R.K;

% "After the transient" = the last 60% of the recorded flight.  On the
% listing's own case that is t >= 3.96 s, i.e. exactly the t >= 4 s window
% BENCH ROW 5 is scored on; writing it as a fraction keeps checks 3-5 from
% going RED for the uninteresting reason that a short TF has no t >= 4 s.
tlate = t(1) + 0.4*(t(end) - t(1));

% 1  P11 = K1*SIGN2 identically.  (1-K1) = SIGN2/(M11+SIGN2), so P11 =
%    K1*SIGN2 exactly -- an algebraic consequence of two transcribed lines that
%    a mistyped update breaks while leaving the curve entirely plausible.
p11 = squeeze(R.P(1,1,:));
res1 = max(abs(p11 - Kg(:,1).*R.sig_pos.^2) ./ max(p11, realmin));
names{end+1} = 'P11 == K1*sigma_pos^2 (exact)';
oks(end+1)   = res1 <= 1e-12;
detail{end+1} = sprintf('max relative deviation %.2e', res1);

% 2  Riccati sanity: 0 < K1 <= 1, because the innovation covariance M11+SIGN2
%    is never smaller than M11; and P stays symmetric positive definite.
mn = inf;
for i = 1:size(R.P,3)
    Pi = R.P(:,:,i);
    mn = min(mn, min(real(eig((Pi + Pi.')/2))));
end
sane = all(Kg(:,1) > 0) && all(Kg(:,1) <= 1) && mn > 0;
names{end+1} = 'Riccati sane: 0 < K1 <= 1 and P positive definite';
oks(end+1)   = sane;
detail{end+1} = sprintf('K1 in [%.4f, %.4f]; min eig(P) = %.4g', ...
    min(Kg(:,1)), max(Kg(:,1)), mn);

% 3  DELETE EXPERIMENT -- the process noise.  With Phi_s = 0 the book's
%    "monotonically increasing after the transient" (:2786) MUST fail.  If it
%    does not, the seven Q terms are not reaching the answer.
[~, ~, Kq] = riccatiRun(VC, XNT, VM, HEDEG, SIGRIN, TS, TF, 0.0, 0.0, false);
[okq, txtq] = deleteBroke(t, Kg(:,1), Kq(:,1), tlate);
names{end+1} = 'DELETE process noise (Phi_s = 0) => book shape row breaks';
oks(end+1)   = okq;
detail{end+1} = sprintf('%s; last decrease at t = %.1f s', txtq, monoOnset(t, Kq(:,1)));

% 4  DELETE EXPERIMENT -- the range scaling of the measurement noise.  :2785
%    ("this noise must be converted to a positional noise by the multiplication
%    of range") is the mechanism behind the rising gains; freezing SIGN2 at its
%    t = 0 value must destroy the same shape row.
[~, ~, Kf] = riccatiRun(VC, XNT, VM, HEDEG, SIGRIN, TS, TF, 0.0, 1.0, true);
[okf, txtf] = deleteBroke(t, Kg(:,1), Kf(:,1), tlate);
names{end+1} = 'DELETE range scaling (SIGN2 frozen) => book shape row breaks';
oks(end+1)   = okf;
detail{end+1} = sprintf('%s; monotone onset %.1f s', txtf, monoOnset(t, Kf(:,1)));

% 5  The recursion FORGETS ITS INITIAL CONDITION.  Rerun on Listing 29.3's
%    schedule (first update one sample later) and require the two to agree
%    after the transient.  This is the invariant that licenses comparing our
%    gain table with kalmanLawOptions', and it is why they differ early.
[~, ~, Ksh] = riccatiRun(VC, XNT, VM, HEDEG, SIGRIN, TS, TF, TS, 1.0, false);
nsh = min(size(Ksh,1), numel(t) - 1);
gap = abs(Kg((1:nsh)+1, 1) - Ksh(1:nsh, 1)) ./ abs(Kg((1:nsh)+1, 1));
late = t((1:nsh)+1) >= tlate;
if any(late), res5 = max(gap(late)); else, res5 = max([gap; 0]); end
K.shift_worst_pct = 100*max([gap; 0]);
K.shift_late_pct  = 100*res5;
names{end+1} = 'one-step grid shift forgotten after the transient (<= 0.5%)';
oks(end+1)   = res5 <= 0.005;
detail{end+1} = sprintf('worst %.1f%% over the flight, %.3f%% on t >= %.2f s', ...
    K.shift_worst_pct, K.shift_late_pct, tlate);

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
function B = benchmarks(R, VC, XNT, VM, HEDEG, SIGRIN, TS, TF, hiS)
%BENCHMARKS  Computed vs published.  EVERY book value was grepped out of the
%   decoded file and the RAW zarchan.txt line is in .source.  Nothing here was
%   read off a figure -- Figs. 9.3 and 9.6 are image links in the extraction,
%   which is exactly why ROW 9 is a SKIP and not a "figure reading".
B = emptyBench();
t = R.t;  Kg = R.K;

nominal = abs(XNT-96.6) < 1e-9 && abs(VC-4000) < 1e-9 && abs(VM-3000) < 1e-9 ...
    && abs(HEDEG-20) < 1e-9 && abs(SIGRIN-0.001) < 1e-12 ...
    && abs(TS-0.1) < 1e-12 && abs(TF-10) < 1e-9;
whyOff = sprintf(['prose about Fig. 9.3/9.6, which is the listing''s own case ' ...
    '(VC=4000, nT=96.6, VM=3000, HE=20 deg, sigrin=.001, Ts=.1, tF=10); this ' ...
    'run used VC=%g, nT=%g, VM=%g, HE=%g, sigrin=%g, Ts=%g, tF=%g'], ...
    VC, XNT, VM, HEDEG, SIGRIN, TS, TF);

% ---- 1.  the listing IS the scalar expansion of the matrix Riccati ---------
[Km, Pm] = riccatiMatrix(VC, XNT, VM, HEDEG, SIGRIN, TS, TF);
nc = min(size(Km,1), size(Kg,1));
dK = max(max(abs(Kg(1:nc,:) - Km(1:nc,:)) ./ max(abs(Km(1:nc,:)), realmin)));
dP = 0;
for i = 1:nc
    e  = reshape(R.P(:,:,i) - Pm(:,:,i), [], 1);
    sc = max(abs(reshape(Pm(:,:,i), [], 1)));
    dP = max(dP, max(abs(e))/max(sc, realmin));
end
B = addBound(B, 'listing == scalar expansion of the matrix Riccati equations', ...
    'exact (:2785 says so)', sprintf('gains %.1e, P %.1e', dK, dP), ...
    max(dK, dP) <= 1e-12, ...
    ['Ch.9 M/K/P equations, p.191-192 (zarchan.txt:2730-2754, :2758-2766, ' ...
     ':2768-2773); the claim itself at :2785'], '-', ...
    ['independent matrix implementation of the published Phi_k, Q_k, K_k, ' ...
     'P_k; the only row here that checks the seven powers of Ts in M']);

% ---- 2.  the process noise level ------------------------------------------
if abs(XNT - 96.6) < 1e-9
    B = addVal(B, 'target maneuver level nT/g', 3.0, XNT/32.2, ...
        '0.5% (prose, one significant figure)', 0.005, ...
        ['p.193 "a target maneuver of amplitude 3 g" (zarchan.txt:2785); ' ...
         'Phi_s = nT^2/tF at :2626'], 'g', ...
        sprintf('Phi_s = nT^2/tF = %.4f (ft/s^2)^2/s', XNT*XNT/TF));
else
    B = addSkip(B, 'target maneuver level nT/g', '3', ...
        sprintf('the 3 g is quoted for nT = 96.6 ft/s^2; this run used %g', XNT), ...
        'zarchan.txt:2785', 'g');
end

% ---- 3.  the published P0 --------------------------------------------------
p0book = [(VC*TF*SIGRIN)^2, (VM*HEDEG/57.3)^2, XNT*XNT];
p0ours = [R.P0(1,1) R.P0(2,2) R.P0(3,3)];
offd   = max(abs([R.P0(1,2) R.P0(1,3) R.P0(2,3)]));
d0     = max(abs(p0ours - p0book) ./ max(abs(p0book), realmin));
B = addBound(B, 'P0 == diag([sigma_noise^2, (VM*HE/57.3)^2, nT^2])', ...
    sprintf('[%.6g %.6g %.6g]', p0book(1), p0book(2), p0book(3)), ...
    sprintf('max rel dev %.1e, off-diagonal %.1e', d0, offd), ...
    d0 <= 1e-12 && offd == 0, ...
    ['p.193 P0 equation (zarchan.txt:2778); "converted to a positional noise ' ...
     'by the multiplication of range" at :2785'], 'ft^2, (ft/s)^2, (ft/s^2)^2', ...
    sprintf(['sigma_noise = VC*tF*sigrin = %.4f ft.  This row pins the ORDER ' ...
             'of the three entries; the RANGE SCALING of the running noise is ' ...
             'what SELF-CHECK 4 deletes'], VC*TF*SIGRIN));

% ---- 4/5/6.  the three prose claims about Fig. 9.3 -------------------------
if nominal
    rng1 = max(Kg(:,1))/min(Kg(:,1));
    B = addBound(B, 'the gains are TIME-VARYING (not a fading-memory constant)', ...
        'time-varying', sprintf('max/min K1 = %.4f', rng1), rng1 >= 1.5, ...
        ['p.193 "unlike the constant-gain digital fading memory filter, the ' ...
         'Kalman filter has time-varying gains" (zarchan.txt:2786)'], '-', ...
        'the weakest row here; the two DELETE experiments are what give it teeth');

    B = addShape(B, 'all three gains non-decreasing for t >= 4 s', ...
        'monotonically increasing after the transient', ...
        sprintf('K1/K2/K3 onset %.1f/%.1f/%.1f s', monoOnset(t,Kg(:,1)), ...
                monoOnset(t,Kg(:,2)), monoOnset(t,Kg(:,3))), ...
        monoAfter(t, Kg, 4.0), ...
        ['p.193 "after an initial transient period, the gains appear to be ' ...
         'monotonically increasing" (zarchan.txt:2786)'], ...
        ['tested at 4 s, not the prose''s 3 s: K1''s last decrease is at ' ...
         't = 3.6 s, so the literal 3 s would fail a faithful port by one step']);

    B = addVal(B, 'duration of the initial gain transient', 3.0, ...
        monoOnset(t, Kg(:,1)), '+/-1.5 s (the word is "about")', 0.5, ...
        ['p.196 "it takes about 3 s for the Kalman gains to go through their ' ...
         'initial transient period" (zarchan.txt:2806)'], 's', ...
        ['this row catches only a gross error -- 0.5 s or 7 s fail it; 3.0 ' ...
         'against 3.6 s it cannot separate, and is not claimed to']);
else
    B = addSkip(B, 'the gains are TIME-VARYING (not a fading-memory constant)', ...
        'time-varying', whyOff, 'zarchan.txt:2786', '-');
    B = addSkip(B, 'all three gains non-decreasing for t >= 4 s', ...
        'monotonically increasing', whyOff, 'zarchan.txt:2786', '-');
    B = addSkip(B, 'duration of the initial gain transient', '3', whyOff, ...
        'zarchan.txt:2806', 's');
end

% ---- 7.  Fig. 9.6: 10 mr of angle noise instead of 1 mr --------------------
if nominal && ~isempty(hiS) && abs(hiS - 0.01) < 1e-12 && ~isempty(R.hi)
    rat = median(R.hi.ratio);
    B = addBound(B, 'K1 at 10 mr / K1 at 1 mr ("approximately halved")', ...
        '~0.5, scored as [0.35, 0.75]', ...
        sprintf('median %.4f (min %.4f, max %.4f)', rat, min(R.hi.ratio), ...
                max(R.hi.ratio)), rat >= 0.35 && rat <= 0.75, ...
        ['p.196 "if we tell the filter that there is 10 mr of angle noise ' ...
         'rather than 1 mr, the first Kalman gain value is approximately ' ...
         'halved" (zarchan.txt:2807)'], '-', ...
        ['a BOUND, not a value: the true ratio is 0.62, so "halved" is ~25% ' ...
         'optimistic and 0.500 would pretend the prose has three digits.  The ' ...
         'band excludes ratio = 1 (sigrin never reaching the gain) but NOT ' ...
         'sigrin reaching the running noise and not P0, which gives 0.60']);
else
    B = addSkip(B, 'K1 at 10 mr / K1 at 1 mr ("approximately halved")', '~0.5', ...
        'needs the nominal 1 mr case AND SIGRIN_HI = 0.01 (the Fig. 9.6 pair)', ...
        'zarchan.txt:2807', '-');
end

% ---- 8/9.  reported, never scored -----------------------------------------
B = addInfo(B, 'same recursion in Listing 29.3, started one sample later', ...
    'not a comparison the book makes', ...
    sprintf('worst %.1f%% over the flight, %.3f%% once settled', ...
            R.selfcheck.shift_worst_pct, R.selfcheck.shift_late_pct), ...
    ['zarchan.txt:2793 vs :10766 (Listing 29.3, ported as ' ...
     'mgs.zarchan.kalmanLawOptions, which exposes R.kalman.K).  ⚠️ 10766 is ' ...
     'the RAW line, verified; that port''s header cites :10953'], ...
    ['the book never compares the two listings, so there is nothing to score; ' ...
     'this row exists so their difference is not mistaken for a port fault']);

B = addSkip(B, 'the gain values of Figs. 9.3 and 9.6', 'plotted, never printed', ...
    ['Ch.9 publishes NO gain number for this listing -- both figures became ' ...
     'image links in the extraction.  Ours (K1 = 0.887 at t=0, minimum 0.288 ' ...
     'at 3.6 s, 0.898 at 9.9 s) have NO book counterpart, and quoting them as ' ...
     'agreement would be a fabricated figure reading'], ...
    'Figs. 9.3, 9.6 (captions at zarchan.txt:2794, :2839; both are image links)', '-');
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly.  This is the
%   schema benchSuite's shared benchRows() consumes; a near miss ERRORS there.
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
%ADDSKIP  A check that COULD NOT RUN.  Never a pass: not-run is a different
%   thing from succeeded.
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
L{end+1} = 'Zarchan Listing 9.1 -- SOLVING THE RICATTI EQUATIONS (three-state Kalman gains)';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf('   VC = %g ft/s   nT = %g ft/s^2 (%.4g g)   VM = %g ft/s   HE = %g deg', ...
    p.VC, p.XNT, p.XNT/32.2, p.VM, p.HEDEG);
L{end+1} = sprintf(['   sigrin = %g rad (%.3g mr)   Ts = %g s   tF = %g s   ' ...
    'Phi_s = %.4f   %d updates'], p.SIGRIN, 1000*p.SIGRIN, p.TS, p.TF, ...
    p.PHIN, p.n_updates);
L{end+1} = '';
L{end+1} = sprintf('%8s %8s %10s %10s %10s %10s %10s %10s %10s', 't [s]', ...
    'tgo [s]', 'sig_pos', 'K1', 'K2', 'K3', 'sqrtP11', 'sqrtP22', 'sqrtP33');
stride = max(1, round(numel(R.t)/10));
rows = unique([1:stride:numel(R.t), numel(R.t)]);
for r = rows
    L{end+1} = sprintf('%8.2f %8.2f %10.4f %10.5f %10.5f %10.5f %10.4f %10.4f %10.4f', ...
        R.t(r), R.tgo(r), R.sig_pos(r), R.K(r,1), R.K(r,2), R.K(r,3), ...
        sqrt(R.Pdiag(r,1)), sqrt(R.Pdiag(r,2)), sqrt(R.Pdiag(r,3))); %#ok<AGROW>
end
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-58s %s', vstr(K.oks(i)), K.names{i}, ...
        K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning NOT CHECKED.';
else
    L{end+1} = 'BENCHMARKS vs THE BOOK   (value/bound/shape are scored; info/skip are not)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        if b.skipped
            mark = '--';
        elseif b.ok
            mark = 'ok';
        else
            mark = 'RED';
        end
        L{end+1} = sprintf('   %-4s %-6s %-52s book %-30s ours %s', mark, ...
            b.kind, b.what, b.book_str, b.ours_str); %#ok<AGROW>
        L{end+1} = sprintf('        %s', b.source); %#ok<AGROW>
        if ~isempty(b.why)
            L{end+1} = sprintf('        why: %s', b.why); %#ok<AGROW>
        end
    end
    L{end+1} = sprintf('   %d of %d rows scored; %d passed.', sum(ev), ...
        numel(R.bench), sum(ev & [R.bench.ok]));
end
s = strjoin(L, newline);
end

function v = vstr(ok)
if ok, v = 'ok'; else, v = 'RED'; end
end

function drawGains(R)
%DRAWGAINS  Fig. 9.3.  The listing plots ArrayK1 alone; all three are drawn
%   here because the prose claim scored above is about all three.
figure;
plot(R.t, R.K(:,1), '-', R.t, R.K(:,2), '--', R.t, R.K(:,3), ':'); grid on
xlabel('Flight Time (Sec)'); ylabel('Kalman gains');
legend('K1', 'K2', 'K3', 'Location', 'best');
title('Zarchan Fig. 9.3 -- Kalman gain profiles for nominal case');
end
