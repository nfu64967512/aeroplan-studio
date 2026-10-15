function R = discreteRiccati(opts)
%MGS.ZARCHAN.DISCRETERICCATI  Zarchan Listing 30.2 -- SOLVING THE DISCRETE
%   RICATTI EQUATIONS for the best achievable rms miss of a three-state Kalman
%   filter, swept over flight time.
%
%   R = mgs.zarchan.discreteRiccati()
%   R = mgs.zarchan.discreteRiccati('TS', 0.1)          the book's coarse case
%   R = mgs.zarchan.discreteRiccati('assert', false)    warn, still return R
%
%   Line-by-line port of zarchan.txt:11332-11338 (p.794).  Every ":NNNN" is a
%   zarchan.txt line; the VALUES were read from zarchan_decoded.txt, where the
%   maths font is repaired.  The two files are line-for-line aligned (15120
%   lines each; :11331 is the LISTING 30.2 heading in BOTH).
%
%   UNITS ARE THE BOOK'S: ft, ft/s, ft/s^2, s; g = 32.2 ft/s^2.  P11 is in
%   ft^2, so sqrt(P11) is a MISS IN FEET.  HEDEG is in DEGREES and divided by
%   57.3, as the listing does.  Nothing is converted.
%
% ---------------------------------------------------------------------------
%   IS THIS THE SAME RECURSION AS riccatiSolver (9.1) AND THE CH.26 PORTS?
%
%   9.1: YES, identically.  The M/K/P block of :11336 is character-for-
%   character the M/K/P block of :2799-2801 (riccatiSolver's own header already
%   lists 30.2 among the listings carrying it).  It is the ESTIMATION Riccati,
%   M = Phi*P*Phi' + Q, K = M*H'/(H*M*H' + R), P = (I - K*H)*M (:11290-11293),
%   expanded to scalars with symmetry exploited.  TWO differences, both matter:
%     a  9.1 measures an ANGLE, so SIGPOS = RTM*SIGRIN and R SHRINKS as tgo
%        runs out; its gains rise and never settle.  30.2 measures GLINT IN
%        FEET, so R = SIGNOISE^2 is CONSTANT -- and with RTM dead (below) the
%        recursion is TIME-INVARIANT.  That is why it reaches a fixed point,
%        and why comparing it to a STEADY-STATE formula means anything.
%     b  NAME COLLISION.  9.1 calls the PROCESS density PHIN (:2793,
%        XNT*XNT/TF).  30.2 calls that PHIS and uses PHIN for the MEASUREMENT
%        density SIGNOISE^2*TS (:11333).  Same symbol, two quantities, four
%        pages apart; copying a line across without renaming gives a smooth,
%        wrong curve.
%   Ch.26: NO.  optimalGainsSingleLag/Weave/Cubic integrate the CONTROL
%   Riccati (:9256-9276) -- a nonlinear matrix DIFFERENTIAL equation with a
%   quadratic term, no Q, no measurement noise.  Dual object; untouched here.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_30_02.m reports NO unresolved comparisons, and
%   that is right: Listing 30.2 contains no IF at all.  Six decisions:
%     1  :11336 runs "M12=P12+TS*P22+.5*TS2*P23+TS*(P13+TS*P23+.5*TS2*P33)"
%        into "M12=M12+TS4*PHIS/8.;" with NO SEMICOLON.  Restored -- forced,
%        not guessed: Listing 9.1 (:2800) writes the identical expression as
%        ONE statement ending "+TS4*PHIN/8.;", so the split is a page break.
%     2  :11337 "FORM=sqrt(...)" also lost its semicolon.  Restored; no
%        numeric effect.
%     3  "save datfil.txt output /ascii" is a typo for -ascii (9.1 at :2802 has
%        it right).  Nothing is written to disk here; recorded and dropped.
%     4  "plot(ArrayT',ArrayFORM',ArraySP11)" leaves the THIRD argument not
%        transposed -- a book bug, not OCR (30.3 at :11423 writes proper pairs).
%     5  "T=0.;" before "for T=TS:TS:TF" is dead; the loop overwrites T.
%     6  "TGO=TF-T+.000001; RTM=VC*TGO;" runs every step and RTM is NEVER READ,
%        so VC = 16400 ft/s is dead.  KEPT and reported as an INFO row rather
%        than deleted, because that deadness is the fact (a) rests on.
%   OCR on the scored values -- superscripts did not survive: :11279-11280
%   "1612 / 10 = 2592" is 161^2/10; :11281 "1020.01 = 1" is 10^2*0.01; :11287
%   "225921=615=6" is 2*2592^(1/6)*1^(5/6); :11295 lost the LEADING 1 of the
%   fundamental matrix ([1 Ts .5Ts^2; 0 1 Ts; 0 0 1], confirmed by the
%   listing's own M11).  The minus-sign-to-digit-2 damage class was looked for
%   at :11260-11265 and is NOT present: the "2" in "P2 11 = 2P12Fn" is a
%   genuine coefficient, as SELF-CHECK 2 verifies by deriving all six equations
%   from 0 = -P*H'*R^-1*H*P + P*F' + F*P + Q.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS (line numbers in each row's .source; nothing read off a figure)
%     1  Phi_s = nT^2/tF = 161^2/10 = 2592        (:11278-11280) band 0.1%
%     2  Phi_n = sigma_GL^2*Ts = 10^2*0.01 = 1    (:11281)       band 0.1%
%     3  sqrt(P11) = sqrt(2*Phi_s^(1/6)*Phi_n^(5/6)) = 2.72 ft
%                                                 (:11287-11288) band 0.5%
%     4  target maneuver 5 g = XNTIC/32.2         (:11277)       band 0.2%
%     5  BOUND from :11340 ("virtually identical when the sampling time is
%        0.01 s"): |SP11/FORM - 1| <= 5% at the longest tF.  The band is NOT
%        fitted to our answer -- the SAME sentence says Ts = 0.1 s shows "more
%        of a difference", so the threshold must SEPARATE those two cases and
%        nothing else pins it; 5% is their geometric midpoint.  Do not widen.
%     6-8  SHAPE rows from :11330 and :11340: the Ts=0.1 gap exceeds the
%        Ts=0.01 gap; the gap shrinks monotonically as Ts shrinks; SP11 itself
%        shrinks with Ts ("better performance ... with smaller sampling
%        times").  An ordering is the strongest form a figure-only claim takes.
%   NOT SCORED: curve values off Fig. 30.13 (SKIP -- it published no numbers,
%   and fabricated figure readings have been caught in this package before);
%   VC/RTM (INFO).
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED. ***  MATLAB is the
%   caller's; this file has never been executed.  The numbers below come from
%   an INDEPENDENT scalar reimplementation of the same recursion:
%     tF = 10 s, Ts = 0.01 s:  FORM 2.7226, SP11 2.6729 ft, gap  -1.83%
%     tF = 10 s, Ts = 0.1  s:  FORM 7.1065, SP11 6.2969 ft, gap -11.39%
%     gap over Ts = 0.2/0.1/0.05/0.02/0.01 s: -18.80 -11.39 -6.69 -3.21 -1.83 %
%     final P at tF = 10 s, Ts = 0.01 s: 7.1446 26.477 49.060 149.03 370.51 1385.9
%     continuous fixed point:            7.4126 27.473 50.913 152.74 377.40 1398.7
%     SELF-CHECK 4 (delete Q): SP11 2.6729 -> 0.9463 ft, i.e. 35% of nominal.
%   Short flight times are NOT converged and are not meant to be: at tF = 0.2 s
%   the recursion gets 20 steps from P11 = 100 ft^2 and lands 12.8% ABOVE the
%   formula.  Hence rows 5-8 are scored at the LONGEST tF, guarded by
%   SELF-CHECK 3.
%
%   OUTPUTS  R.tf R.t_last R.n_steps R.sp11 R.form R.gap_pct R.P R.P_cont
%            R.ts_scan(.ts .sp11 .form .gap_pct .tf) R.phi_s R.phi_n
%            R.bench R.selfcheck R.pass R.verdict R.summary R.p R.units
%
%   See also MGS.ZARCHAN.RICCATISOLVER, MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 30.2 line 1 (:11332), verbatim.  Feet and seconds.
    opts.VC       (1,1) double {mustBeFinite, mustBePositive} = 5*3280  % ft/s, DEAD
    opts.XNTIC    (1,1) double {mustBeFinite, mustBePositive} = 161     % ft/s^2 = 5 g
    opts.VM       (1,1) double {mustBeFinite, mustBePositive} = 3000    % ft/s
    opts.HEDEG    (1,1) double {mustBeFinite, mustBeNonnegative} = 20   % deg
    opts.SIGNOISE (1,1) double {mustBeFinite, mustBePositive} = 10      % ft, glint
    opts.TS       (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s
    opts.TFLIST   double {mustBeReal, mustBeFinite} = []  % [] = "for TF=.2:.2:10."
    % The book ran 0.1 and 0.01 (:11340); the rest are here so that "monotone"
    % is a claim about a trend and not about two points.  opts.TS is added too.
    opts.TSCOMPARE double {mustBeReal, mustBeFinite} = [0.2 0.1 0.05 0.02 0.01]
    % THE ONLY SWITCHES.  Not nargin: with an arguments block declaring only
    % name-value options MATLAB reports nargin = 0 however many were passed.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.assert    (1,1) logical = true
end

VC = opts.VC;  XNTIC = opts.XNTIC;  VM = opts.VM;  HEDEG = opts.HEDEG;
SIGNOISE = opts.SIGNOISE;  TS = opts.TS;

TFs = opts.TFLIST(:).';
if isempty(TFs), TFs = 0.2:0.2:10.0; end                          % :11332
if any(TFs < TS)
    error('mgs:zarchan:discreteRiccati:shortTF', ...
        ['every flight time must admit at least one sample: min(TF) = %g s ' ...
         'against TS = %g s.  A zero-step sweep returns the INITIAL ' ...
         'covariance and would read as a converged answer.'], min(TFs), TS);
end

nTF   = numel(TFs);
sp11  = zeros(1,nTF);  form  = zeros(1,nTF);
tlast = zeros(1,nTF);  nstep = zeros(1,nTF);
Plast = zeros(1,6);
for k = 1:nTF
    [Pk, sp11(k), form(k), nstep(k), tlast(k)] = ...
        riccatiSweep(XNTIC, VM, HEDEG, SIGNOISE, TS, TFs(k), 1.0, 1);
    if k == nTF, Plast = Pk; end
end

% ---- the sampling-time scan, at the LONGEST flight time --------------------
TFend  = TFs(end);
tsList = sort(unique([opts.TSCOMPARE(:).', TS]), 'descend');
tsList = tsList(tsList > 0 & tsList <= TFend);
sc = struct('ts', tsList, 'sp11', zeros(size(tsList)), ...
            'form', zeros(size(tsList)), 'gap_pct', zeros(size(tsList)), ...
            'tf', TFend);
for k = 1:numel(tsList)
    [~, sc.sp11(k), sc.form(k)] = ...
        riccatiSweep(XNTIC, VM, HEDEG, SIGNOISE, tsList(k), TFend, 1.0, 1);
end
sc.gap_pct = 100*(sc.sp11./sc.form - 1);

PHIS = XNTIC*XNTIC/TFend;
PHIN = SIGNOISE*SIGNOISE*TS;

R = struct();
R.tf = TFs;  R.t_last = tlast;  R.n_steps = nstep;
R.sp11 = sp11;  R.form = form;  R.gap_pct = 100*(sp11./form - 1);
R.P = Plast;  R.P_cont = contSteady(PHIS, PHIN);
R.ts_scan = sc;  R.phi_s = PHIS;  R.phi_n = PHIN;
R.units = 'ft, ft/s, ft/s^2, s; g = 32.2 ft/s^2; P11 in ft^2, sqrt(P11) in ft';
R.p = struct('VC',VC, 'XNTIC',XNTIC, 'VM',VM, 'HEDEG',HEDEG, ...
             'SIGNOISE',SIGNOISE, 'TS',TS, 'TF_end',TFend, 'n_tf',nTF, ...
             'ts_scan',tsList, 'units',R.units);

R.selfcheck = structuralChecks(XNTIC, VM, HEDEG, SIGNOISE, TS, TFend, ...
                               Plast, PHIS, PHIN, sp11(end));

if opts.benchmark
    R.bench = benchmarks(R, XNTIC, SIGNOISE, TS, TFend, VC);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine" -- that is a green light this repo has been burned by.
    R.bench = emptyBench();  ev = false;  R.pass = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end

msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated.  The published ' ...
        'numbers are for XNTIC = 161 ft/s^2, SIGNOISE = 10 ft, TS = 0.01 s, ' ...
        'tF = 10 s; this run used %g, %g, %g, %g.'], XNTIC, SIGNOISE, TS, TFend);
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
        error('mgs:zarchan:discreteRiccati:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN, complete.  benchSuite runs every port with assert
        % off precisely so one red row cannot hide the rest of the board.
        warning('mgs:zarchan:discreteRiccati:benchmark', ...
            '%s  (''assert'' false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [P, SP11, FORM, N, Tend] = riccatiSweep(XNTIC, VM, HEDEG, SIGNOISE, ...
                                                 TS, TF, phisScale, nRepeat)
%RICCATISWEEP  Listing 30.2's loop, zarchan.txt:11333-11337.
%
%   Every constant arrives as an ARGUMENT: MATLAB local functions cannot see
%   the main function's workspace, and a previous wave in this package shipped
%   a local that read one from the caller.
%
%   phisScale scales PHIS (1 = the listing, 0 = SELF-CHECK 4's delete
%   experiment).  nRepeat runs the same fixed-step recursion nRepeat times
%   longer for the convergence check -- legitimate ONLY because the recursion
%   is time-invariant here (RTM dead, R constant).

TS2 = TS*TS;  TS3 = TS2*TS;  TS4 = TS3*TS;  TS5 = TS4*TS;        % :11333
PHIS = phisScale*XNTIC*XNTIC/TF;
PHIN = SIGNOISE*SIGNOISE*TS;
SIGPOS = SIGNOISE;  SIGN2 = SIGPOS^2;

P11 = SIGN2;                        % :11335 -- the glint variance
P12 = 0.0;
P13 = 0.0;
P22 = (VM*HEDEG/57.3)^2;            % heading-error velocity uncertainty
P23 = 0.0;
P33 = XNTIC*XNTIC;                  % full maneuver magnitude, squared
% "T=0.;" sits here in the listing and is dead: the FOR overwrites it.

Tgrid = TS:TS:TF;                   % :11335, "for T=TS:TS:TF"
N     = numel(Tgrid);
Tend  = TF;
if N > 0, Tend = Tgrid(end); end    % ArrayT records T, not TF

for rep = 1:nRepeat
for ii = 1:N
    % TGO = TF-T+.000001; RTM = VC*TGO;  <-- RTM is never read; see header.
    % SIGPOS/SIGN2 are recomputed from constants every step in the listing and
    % are hoisted above: nothing in the loop can change them.
    M11 = P11 + TS*P12 + .5*TS2*P13 + TS*(P12 + TS*P22 + .5*TS2*P23);
    M11 = M11 + .5*TS2*(P13 + TS*P23 + .5*TS2*P33) + TS5*PHIS/20.;
    M12 = P12 + TS*P22 + .5*TS2*P23 + TS*(P13 + TS*P23 + .5*TS2*P33);
    M12 = M12 + TS4*PHIS/8.;
    M13 = P13 + TS*P23 + .5*TS2*P33 + PHIS*TS3/6.;
    M22 = P22 + TS*P23 + TS*(P23 + TS*P33) + PHIS*TS3/3.;
    M23 = P23 + TS*P33 + .5*TS2*PHIS;
    M33 = P33 + PHIS*TS;
    K1  = M11/(M11 + SIGN2);
    K2  = M12/(M11 + SIGN2);
    K3  = M13/(M11 + SIGN2);
    P11 = (1. - K1)*M11;
    P12 = (1. - K1)*M12;
    P13 = (1. - K1)*M13;
    P22 = -K2*M12 + M22;
    P23 = -K2*M13 + M23;
    P33 = -K3*M13 + M33;
end
end

SP11 = sqrt(P11);                                                  % :11336
% The listing's own literal exponents .16667/.83333, NOT 1/6 and 5/6 (1.3e-5
% relative apart at the book's case).  Reproduced: it is what the printed
% program computes, and benchmark 3 is scored against the printed program.
FORM = sqrt(2.*(PHIS^.16667)*(PHIN^.83333));                       % :11337
P    = [P11 P12 P13 P22 P23 P33];
end


% =============================================================================
function P = riccatiSweepMatrix(XNTIC, VM, HEDEG, SIGNOISE, TS, TF)
%RICCATISWEEPMATRIX  The same sweep as the book's MATRIX equations
%   (:11290-11299, :11318-11329), sharing no line with the scalar version.
%   SELF-CHECK 1 requires the two to agree.
PHIS = XNTIC*XNTIC/TF;
Rk   = SIGNOISE^2;                                  % :11301, Rk = sigma_GL^2
PHI  = [1 TS 0.5*TS^2; 0 1 TS; 0 0 1];              % :11295, leading 1 restored
Q    = PHIS*[TS^5/20 TS^4/8 TS^3/6; ...             % :11318-11329
             TS^4/8  TS^3/3 TS^2/2; ...
             TS^3/6  TS^2/2 TS];
Hm   = [1 0 0];                                     % :11294
Pm   = diag([Rk, (VM*HEDEG/57.3)^2, XNTIC*XNTIC]);
for ii = 1:numel(TS:TS:TF)
    M  = PHI*Pm*PHI.' + Q;
    K  = M*Hm.'/(Hm*M*Hm.' + Rk);
    Pm = (eye(3) - K*Hm)*M;
end
P = [Pm(1,1) Pm(1,2) Pm(1,3) Pm(2,2) Pm(2,3) Pm(3,3)];
end


% =============================================================================
function P = contSteady(PHIS, PHIN)
%CONTSTEADY  The CONTINUOUS steady state, :11260-11267.  P11 is the book's
%   printed result; the other five follow from its own six algebraic
%   equations, in the order that makes each an explicit formula.
P11 = 2*PHIS^(1/6)*PHIN^(5/6);      % :11267
P13 = sqrt(PHIS*PHIN);              % :11262
P12 = P11^2/(2*PHIN);               % :11260
P23 = P11*P13/PHIN;                 % :11264
P33 = P12*P13/PHIN;                 % :11265
P22 = P11*P12/PHIN - P13;           % :11263
P = [P11 P12 P13 P22 P23 P33];
end


function r = sixEqResidual(P, PHIS, PHIN)
%SIXEQRESIDUAL  Max residual of :11260-11265, SCALED by the left-hand side:
%   P33 is order 1400 where P11 is order 7, and an absolute residual would be
%   dominated by whichever equation carries the largest units.
P11=P(1); P12=P(2); P13=P(3); P22=P(4); P23=P(5); P33=P(6);
lhs = [P11^2,      P12^2,      P13^2,     P11*P12,        P11*P13,  P12*P13];
rhs = [2*P12*PHIN, 2*P23*PHIN, PHIS*PHIN, PHIN*(P22+P13), P23*PHIN, P33*PHIN];
r   = max(abs(lhs - rhs)./max(abs(lhs), realmin));
end


% =============================================================================
function K = structuralChecks(XNTIC, VM, HEDEG, SIGNOISE, TS, TFend, ...
                              Plast, PHIS, PHIN, sp11end)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never evidence that
%   this port reproduces Zarchan.  .names and .oks are what put these on
%   benchSuite's board; invariants stored under any other field name are an
%   experiment nobody can see.
names = {};  oks = [];  detail = {};  K = struct();

% 1  The scalar expansion IS Phi*P*Phi' + Q / (I-KH)M.  Catches a mistyped TS
%    power in any of the eight M lines -- each of which yields a smooth,
%    plausible, wrong curve.
Pm = riccatiSweepMatrix(XNTIC, VM, HEDEG, SIGNOISE, TS, TFend);
K.matrix_resid = max(abs(Plast - Pm)./max(abs(Pm), realmin));
names{end+1}  = 'scalar recursion == matrix Phi*P*Phi'' + Q form';
oks(end+1)    = K.matrix_resid <= 1e-9;
detail{end+1} = sprintf('max rel. difference over the 6 elements = %.3e', ...
                        K.matrix_resid);

% 2  The book's closed form satisfies the book's own six algebraic equations.
%    Also what rules out the minus-sign-to-2 damage class at :11260-11265: a
%    mangled sign there would not close.
K.sixeq_resid = sixEqResidual(contSteady(PHIS, PHIN), PHIS, PHIN);
names{end+1}  = 'continuous P satisfies all six algebraic equations';
oks(end+1)    = K.sixeq_resid <= 1e-10;
detail{end+1} = sprintf('max scaled residual = %.3e', K.sixeq_resid);

% 3  The recursion has actually CONVERGED at the longest flight time.  The
%    bound and shape rows compare a fixed point against a STEADY-STATE
%    formula; without this they could be measuring a transient instead.
[~, sp11long] = riccatiSweep(XNTIC, VM, HEDEG, SIGNOISE, TS, TFend, 1.0, 10);
K.converged_resid = abs(sp11long - sp11end)/max(abs(sp11end), realmin);
names{end+1}  = 'converged at the longest tF (10x the steps changes nothing)';
oks(end+1)    = K.converged_resid <= 1e-9;
detail{end+1} = sprintf('|dSP11|/SP11 = %.3e over 10x iterations', ...
                        K.converged_resid);

% 4  DELETE EXPERIMENT.  Zero the process noise and the answer must COLLAPSE:
%    with no maneuver driving it, repeated position measurements pull P11 to
%    zero and "best possible miss" becomes a fiction.  If deleting Q left the
%    answer where it was, the Q terms would not be doing the work the chapter
%    says they do -- and this whole file would be measuring nothing.
[~, sp11noQ] = riccatiSweep(XNTIC, VM, HEDEG, SIGNOISE, TS, TFend, 0.0, 1);
K.deleteQ_ratio = sp11noQ/max(sp11end, realmin);
names{end+1}  = 'DELETE Q: zeroing PHIS collapses SP11 (<= 60% of nominal)';
oks(end+1)    = K.deleteQ_ratio <= 0.60;
detail{end+1} = sprintf('SP11 %.4f -> %.4f ft (%.1f%% of nominal)', ...
                        sp11end, sp11noQ, 100*K.deleteQ_ratio);

% 5  Still a covariance: positive diagonal and Cauchy-Schwarz on all three
%    pairs.  A sign slip in the P22/P23/P33 lines (the three using -K2/-K3)
%    breaks this and nothing else cheap.
P11=Plast(1); P12=Plast(2); P13=Plast(3); P22=Plast(4); P23=Plast(5); P33=Plast(6);
K.cs_min = min([P11*P22 - P12^2, P11*P33 - P13^2, P22*P33 - P23^2]);
names{end+1}  = 'final P is a valid covariance (diag > 0, Cauchy-Schwarz)';
oks(end+1)    = all([P11 P22 P33] > 0) && K.cs_min >= 0;
detail{end+1} = sprintf('min 2x2 minor = %+.4g', K.cs_min);

K.names = names;  K.oks = logical(oks);  K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    j = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{j}, detail{j});
end
end


% =============================================================================
function B = benchmarks(R, XNTIC, SIGNOISE, TS, TFend, VC)
%BENCHMARKS  Computed vs published, with the PROVENANCE of every band.
%   Do NOT widen a band to turn a row green.  A red row is a result.
B  = emptyBench();
sc = R.ts_scan;
isMvr   = abs(XNTIC - 161) < 1e-9;
isNoise = abs(SIGNOISE - 10) < 1e-12 && abs(TS - 0.01) < 1e-12;
isTF10  = abs(TFend - 10) < 1e-9;
whyOff  = sprintf(['off the book''s case -- the published numbers are for ' ...
    'XNTIC = 161, SIGNOISE = 10, TS = 0.01, tF = 10; this run used ' ...
    '%g, %g, %g, %g'], XNTIC, SIGNOISE, TS, TFend);

if isMvr && isTF10
    B = addVal(B, 'Phi_s = nT^2/tF at tF = 10 s', 2592, R.phi_s, 'text', ...
        0.001, 'p.792 (zarchan.txt:11278-11280)', 'ft^2/s^3', ...
        '161^2/10 = 2592.1; the book prints 2592 (raw line reads "1612 / 10")');
else
    B = addNote(B, 'skip', 'Phi_s = nT^2/tF at tF = 10 s', '2592', whyOff, ...
        'p.792 (zarchan.txt:11278-11280)', 'ft^2/s^3');
end

if isNoise
    B = addVal(B, 'Phi_n = sigma_GL^2*Ts', 1, R.phi_n, 'text', 0.001, ...
        'p.792 (zarchan.txt:11281)', 'ft^2*s', ...
        '10^2*0.01 = 1; the raw line reads "1020.01" -- the exponent is lost');
else
    B = addNote(B, 'skip', 'Phi_n = sigma_GL^2*Ts', '1', whyOff, ...
        'p.792 (zarchan.txt:11281)', 'ft^2*s');
end

% Three printed figures -> the truth lies in [2.715, 2.725], so 0.5% is the
% rounding interval and not a choice of ours.
if isMvr && isNoise && isTF10
    B = addVal(B, 'sqrt(P11) steady-state formula at tF = 10 s', 2.72, ...
        R.form(end), 'text', 0.005, 'p.792 (zarchan.txt:11287-11288)', 'ft', ...
        'FORM as the listing computes it, literal exponents .16667/.83333');
else
    B = addNote(B, 'skip', 'sqrt(P11) steady-state formula at tF = 10 s', ...
        '2.72', whyOff, 'p.792 (zarchan.txt:11287-11288)', 'ft');
end

if isMvr
    B = addVal(B, 'target maneuver magnitude', 5, XNTIC/32.2, 'text', 0.002, ...
        'p.792 text (zarchan.txt:11277)', 'g', ...
        'the prose says "a 5-g target maneuver"; the listing says XNTIC = 161');
else
    B = addNote(B, 'skip', 'target maneuver magnitude', '5', whyOff, ...
        'p.792 text (zarchan.txt:11277)', 'g');
end

gapFine = abs(R.gap_pct(end));
B = addCheck(B, 'bound', 'discrete vs formula at the longest tF', ...
    '|SP11/FORM - 1| <= 5%  ("virtually identical")', ...
    sprintf('%.3f%% at tF = %g s, Ts = %g s (SP11 %.4f vs FORM %.4f ft)', ...
            gapFine, TFend, TS, R.sp11(end), R.form(end)), gapFine <= 5, ...
    'p.795 text (zarchan.txt:11340)', 'ft', ...
    ['the band separates the two cases the SAME sentence contrasts, and is ' ...
     'their geometric midpoint -- not a fit to our answer']);

[hasFine,   iFine]   = tsIndex(sc.ts, 0.01);
[hasCoarse, iCoarse] = tsIndex(sc.ts, 0.1);
if hasFine && hasCoarse
    B = addCheck(B, 'shape', ...
        'Ts = 0.1 s differs from the formula more than Ts = 0.01 s', ...
        '|gap(0.1)| > |gap(0.01)|', ...
        sprintf('%.2f%% vs %.2f%% at tF = %g s', abs(sc.gap_pct(iCoarse)), ...
                abs(sc.gap_pct(iFine)), sc.tf), ...
        abs(sc.gap_pct(iCoarse)) > abs(sc.gap_pct(iFine)), ...
        'p.795 text (zarchan.txt:11340)', '-', ...
        'the second half of the sentence whose first half the bound row scores');
else
    B = addNote(B, 'skip', ...
        'Ts = 0.1 s differs from the formula more than Ts = 0.01 s', ...
        '|gap(0.1)| > |gap(0.01)|', ...
        'TSCOMPARE does not contain both 0.1 s and 0.01 s', ...
        'p.795 text (zarchan.txt:11340)', '-');
end

% :11330 -- "as the sampling time gets smaller, the continuous and discrete
% answers ... should agree".  sc.ts is DESCENDING, so |gap| must decrease.
if numel(sc.ts) >= 3
    g = abs(sc.gap_pct);
    B = addCheck(B, 'shape', 'gap shrinks monotonically as Ts shrinks', ...
        'strictly decreasing |gap| along decreasing Ts', ...
        sprintf('%s %% over Ts = %s s', vecStr(g,'%.2f'), vecStr(sc.ts,'%g')), ...
        all(diff(g) < 0), 'p.794 text (zarchan.txt:11330)', '-', ...
        'a trend, not two points: the book states the limit, not a value');
    B = addCheck(B, 'shape', 'smaller Ts gives a smaller achievable rms miss', ...
        'SP11 strictly decreasing along decreasing Ts', ...
        sprintf('%s ft over Ts = %s s', vecStr(sc.sp11,'%.4f'), ...
                vecStr(sc.ts,'%g')), all(diff(sc.sp11) < 0), ...
        'p.795 text (zarchan.txt:11340)', 'ft', ...
        ['independent of the row above: FORM moves with Ts too, so the gap ' ...
         'could shrink while the miss itself grew']);
else
    B = addNote(B, 'skip', 'gap shrinks monotonically as Ts shrinks', ...
        'decreasing |gap|', 'fewer than three sampling times in TSCOMPARE', ...
        'p.794 text (zarchan.txt:11330)', '-');
    B = addNote(B, 'skip', 'smaller Ts gives a smaller achievable rms miss', ...
        'SP11 decreasing', 'fewer than three sampling times in TSCOMPARE', ...
        'p.795 text (zarchan.txt:11340)', 'ft');
end

B = addNote(B, 'skip', 'curve values off Fig. 30.13', '--', ...
    ['NOT SCORED: Fig. 30.13 publishes no numbers -- the extraction turned ' ...
     'it into an image link (zarchan.txt:11342-11361) and the "10 8 6 4 2 0" ' ...
     'nearby is the y-AXIS TICK LIST, not data.  Its content is scored ' ...
     'through the prose rows instead'], ...
    'Fig. 30.13 caption (zarchan.txt:11362)', 'ft');

B = addNote(B, 'info', 'VC and RTM are dead code', ...
    'VC = 5*3280 = 16400 ft/s, RTM = VC*TGO', ...
    ['NOT SCORED, but load-bearing: because RTM is never read, R stays ' ...
     'constant and the recursion is TIME-INVARIANT.  That is why it reaches ' ...
     'a fixed point and why the steady-state formula is comparable at all.  ' ...
     'In Listing 9.1 the same RTM IS read, and those gains never settle.'], ...
    'listing line (zarchan.txt:11336)', '-');
B(end).ours_str = sprintf('VC = %g ft/s, computed every step, never read', VC);
end


% =============================================================================
function [found, idx] = tsIndex(tsList, want)
[d, idx] = min(abs(tsList - want));
found = d <= 1e-12;
end

function s = vecStr(v, fmt)
c = arrayfun(@(x) sprintf(fmt, x), v(:).', 'UniformOutput', false);
s = ['[' strjoin(c, ' ') ']'];
end


% =============================================================================
%   ROW CONSTRUCTION.  ONE source of field order -- newRow() -- so a near-miss
%   schema cannot drift in from a hand-built row.  benchSuite's adaptGeneric is
%   STRICT: book_s for book_str would ERROR rather than be accommodated.
function b = newRow()
b = struct('what','', 'kind','', 'book',NaN, 'book_str','', 'ours',NaN, ...
           'ours_str','--', 'pct',NaN, 'band','', 'tol',NaN, 'ok',false, ...
           'skipped',false, 'why','', 'source','', 'units','-', 'detail','');
end

function B = emptyBench()
B = newRow();  B(:) = [];      % 0x0 with EXACTLY newRow's field order
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
b = newRow();
b.what = what;  b.kind = 'value';
b.book = book;  b.book_str = sprintf('%.4g', book);
b.ours = ours;  b.ours_str = sprintf('%.5g', ours);
b.band = band;  b.tol = tol;
b.source = source;  b.units = units;  b.detail = detail;
if ~isfinite(ours) || book == 0
    b.skipped = true;
    b.why = 'the computed value is not finite (or the book value is zero)';
else
    b.pct = 100*(ours/book - 1);
    b.ok  = abs(ours/book - 1) <= tol;
end
B(end+1) = b;
end

function B = addCheck(B, kind, what, bookStr, oursStr, ok, source, units, detail)
%ADDCHECK  'shape' (an ordering) or 'bound' (a one-sided limit).  Both are
%   SCORED; neither carries a percentage, because neither book claim has one.
b = newRow();
b.what = what;  b.kind = kind;  b.band = kind;
b.book_str = bookStr;  b.ours_str = oursStr;
b.ok = logical(ok);
b.source = source;  b.units = units;  b.detail = detail;
B(end+1) = b;
end

function B = addNote(B, kind, what, bookStr, why, source, units)
%ADDNOTE  'info' (reported, never scorable) or 'skip' (a check that could not
%   run).  skipped = true keeps both out of the pass count in BOTH directions:
%   a check that did not happen is not a check that succeeded.
b = newRow();
b.what = what;  b.kind = kind;  b.band = kind;
b.book_str = bookStr;  b.why = why;
b.skipped = true;
b.source = source;  b.units = units;
B(end+1) = b;
end


% =============================================================================
function s = render(R, ev)
p = R.p;  L = {};
L{end+1} = 'Zarchan Listing 30.2 -- SOLVING THE DISCRETE RICATTI EQUATIONS';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['   nT = %g ft/s^2 (%.4g g)   glint = %g ft   Ts = %g s   ' ...
    'VM = %g ft/s   HE = %g deg'], p.XNTIC, p.XNTIC/32.2, p.SIGNOISE, p.TS, ...
    p.VM, p.HEDEG);
L{end+1} = sprintf(['   %d flight times, longest tF = %g s   (VC = %g ft/s ' ...
    'is DEAD CODE -- see the INFO row)'], p.n_tf, p.TF_end, p.VC);
L{end+1} = '';
L{end+1} = 'BEST POSSIBLE RMS MISS vs FLIGHT TIME  (ArrayFORM / ArraySP11)';
L{end+1} = sprintf('%9s %8s %12s %12s %10s', 'tF [s]', 'steps', 'FORM [ft]', ...
                   'SP11 [ft]', 'gap [%]');
rows = unique([1:max(1,round(numel(R.tf)/10)):numel(R.tf), numel(R.tf)]);
for r = rows
    L{end+1} = sprintf('%9.2f %8d %12.4f %12.4f %+10.3f', R.tf(r), ...
        R.n_steps(r), R.form(r), R.sp11(r), R.gap_pct(r)); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = sprintf('SAMPLING-TIME SCAN at tF = %g s', R.ts_scan.tf);
L{end+1} = sprintf('%9s %12s %12s %10s', 'Ts [s]', 'FORM [ft]', 'SP11 [ft]', ...
                   'gap [%]');
for k = 1:numel(R.ts_scan.ts)
    L{end+1} = sprintf('%9g %12.4f %12.4f %+10.3f', R.ts_scan.ts(k), ...
        R.ts_scan.form(k), R.ts_scan.sp11(k), R.ts_scan.gap_pct(k)); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = sprintf('FINAL COVARIANCE at tF = %g s  [P11 P12 P13 P22 P23 P33]', ...
                   p.TF_end);
L{end+1} = sprintf('   discrete   %s', vecStr(R.P, '%.4g'));
L{end+1} = sprintf('   continuous %s   (zarchan.txt:11260-11267)', ...
                   vecStr(R.P_cont, '%.4g'));
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
    s = strjoin(L, newline);
    return
end
L{end+1} = 'BENCHMARKS  (book values grepped from zarchan.txt; lines in .source)';
for i = 1:numel(R.bench)
    b = R.bench(i);
    switch lower(b.kind)
        case {'info','skip'}
            L{end+1} = sprintf('   %-4s %-50s %s', upper(b.kind), b.what, ...
                b.ours_str); %#ok<AGROW>
            L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
        case {'shape','bound'}
            L{end+1} = sprintf('   %-4s %-50s %s', vstr(b.ok), b.what, ...
                b.ours_str); %#ok<AGROW>
            L{end+1} = sprintf('        want: %s   [%s]  %s', b.book_str, ...
                b.source, b.detail); %#ok<AGROW>
        otherwise
            L{end+1} = sprintf(['   %-4s %-50s book %s %s / ours %s %s ' ...
                '(%+.3f%%, band %.4g%%)'], vstr(b.ok), b.what, b.book_str, ...
                b.units, b.ours_str, b.units, b.pct, 100*b.tol); %#ok<AGROW>
            L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
    end
end
L{end+1} = sprintf('   %d of %d rows evaluated; the rest are SKIP/INFO.', ...
    sum(ev), numel(R.bench));
s = strjoin(L, newline);
end

function s = vstr(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end
