function R = radomeEngagement(opts)
%MGS.ZARCHAN.RADOMEENGAGEMENT  Zarchan Listing 6.2 -- engagement simulation
%   with RADOME effects: a parasitic feedback path that turns a stable homing
%   loop unstable.
%
%   R = mgs.zarchan.radomeEngagement()
%   R = mgs.zarchan.radomeEngagement('TA', 10)         turning-rate time const
%   R = mgs.zarchan.radomeEngagement('R', 0)           radome path removed
%   R = mgs.zarchan.radomeEngagement('assert', false)  warn instead of error
%
%   Line-by-line port of Listing 6.2 (zarchan.txt:1874-1890), integrated by the
%   book's own second-order Runge-Kutta (Heun) scheme.
%
% ===========================================================================
%   *** WE HAVE NO RADOME.  THIS PORTS A MECHANISM, NOT A CONFIGURATION. ***
%
%   Nothing we fly carries a radar seeker, so NO parameter here maps onto
%   anything in the aircraft and no number out of it is a prediction for one.
%   What ports is the STRUCTURE: a loop that is stable on its own acquires a
%   second, unintended path from its own output back to its own input, and past
%   a threshold in ONE scalar the whole loop goes unstable while every
%   individual block still behaves exactly as designed.  Zarchan's scalar is
%   N'*Vc*R*Ta/(Vm*T); his threshold is 0.79.  Read this for the shape of that
%   argument -- a parasitic path, one scalar that measures it, a boundary that
%   is CROSSED rather than approached -- not for a number to carry across.
%
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, s, g = 32.2 ft/s^2.  NOT SI.
%   Angles are radians inside the model; HEDEG is the one degree-valued input
%   and the listing's own 57.3 converts it.  R is dimensionless and NEGATIVE in
%   the book's case.  Convert AT THE CALL SITE and say so there.
% ===========================================================================
%
%   THE TRANSCRIPTION.  listing_06_02.m reports NO unresolved
%   variable-to-variable comparisons, and that is right: every comparison in
%   6.2 is against a literal.  Repairs, none a judgement call:
%     1  "while T,=(TF-1e-5)" -> "while T <= (TF-1e-5)".  The OCR renders "<"
%        as ",".  Shape-forced: T only increases, so ">=" never terminates and
%        "==" never fires against accumulated floating point.  ("while STEP,=1"
%        the extractor already fixed; its body sets STEP=2 to leave.)
%     2  Plotting tail and "save datfil.txt -ascii" dropped; nothing is written.
%     3  "FLAG=0" after the inner loop is dead (the outer loop re-initialises
%        it) and "YDIC=YD" is never read.  Both kept as comments, not code, so
%        nobody re-adds them thinking a state machine was lost.
%
%   TWO OCR DAMAGES IN THE PROSE, both resolved from the listing, which is the
%   authority:
%     * The book prints the aerodynamic transfer function as "1/(VM(1+Ta s))".
%       The listing says THD = XNL/VM + TA*XNLD/VM, i.e. thetadot/nL =
%       (1+Ta*s)/VM -- Ta in the NUMERATOR -- and the derivation at
%       zarchan.txt:1852 agrees.  So the parasitic term is N'*Vc*R*(1+Ta*s)/Vm,
%       and ONLY that version reproduces the book's own 0.79 and 2.07.  The
%       printed reciprocal does not.
%     * The inequality at zarchan.txt:5408-5409, "0.79 < N'VcRTa/(VmT) < 2.07",
%       lost a MINUS SIGN on the lower bound: this OCR renders "-" as "2" (the
%       same paragraph writes R = -0.01 as "20.01").  Forced by the book's own
%       Tmin, derived from that inequality FOR NEGATIVE R and saying T > Tmin
%       is stable, i.e. |N'VcRTa/(VmT)| < 0.79.  As printed it asserts the
%       opposite of the sentence that follows it.
%
%   VECTORISED ACROSS FLIGHT TIMES.  The listing's "for TF=.1:.1:10" is 100
%   independent trajectories sharing one step, so it becomes an array dimension
%   and the sweep costs max(TF)/H steps instead of sum(TF)/H.  The arithmetic
%   per trajectory, and its ORDER, is the listing's.  Flight times are sorted
%   ascending so the still-flying set is a contiguous suffix; that is what
%   replaces the per-trajectory while-loop.
%
%   WHAT IT PRINTS.  *** MEASURED (R2025b, 0.25 s for the default call), not
%   predicted.  Every band was fixed BEFORE the first run and none was moved
%   after it. ***  Nine scored rows and four structural invariants, all green:
%
%     mu bounds     0.79108, 2.07108   vs the book's 0.79 and 2.07.  These are
%                   sec^5(pi/10)/(5tan(pi/10)) and its 3pi/10 twin -- Routh on
%                   the book's quintic -- but the port BISECTS for them, so the
%                   agreement is a second opinion and not a restatement.
%     Tmin          0.25316 s, 0.56962 s   vs the printed 0.25 and 0.57
%     peak |miss|   10.31 ft at R = 0;  10.34 / 8.87 / 18.94 at Ta = 0/5/10
%     tail |miss|   0.00041 / 0.209 / 0.0856 / 18.94 ft for those four
%     sign changes  1 / 1 / 3 / 9 -- the book's "more oscillatory", counted
%     max Re(s)     -4.767 / -1.303 / -0.0152 1/s at Ta = 0/5/10
%
%   THE Ta = 10 CASE IS THE INTERESTING ONE.  Its linearised loop is still
%   (barely) STABLE: max Re(s) = -0.0152 1/s, mu = -0.800 against a boundary of
%   -0.806.  The book calls it unstable, and the SWEEP sides with the book --
%   the largest miss of the whole sweep lands in the last 20% of flight times
%   and the envelope grows 1.33x from tF in [3,5] to [8,10].  So that row
%   scores PERSISTENCE, not divergence: nothing runs away, it reaches 18.9 ft
%   and is still climbing.  Do not "fix" it into a blow-up test, and do not
%   widen a band to make a row green.
%
%   THESE CHECKS WERE PROVED TO HAVE TEETH, by mutation, not by assertion.
%   Five faults injected into copies under throwaway package names (severing
%   the radome path; flipping R's sign; dropping Ta from thetadot; dropping the
%   -TH from the boresight error; building the binomial 4th-order) each turned
%   1-2 rows red while an unmutated control stayed green.  The 4th-order fault
%   initially passed EVERYTHING -- its poles are all still at -5/T, so a
%   pole-LOCATION check cannot see a wrong ORDER -- which is why invariant 1
%   now scores the pole COUNT separately.  ⚠️ A first attempt at this test
%   silently ran the REAL file (MATLAB resolved the package name to it and
%   reported all mutants passing); confirm with which() before believing a
%   mutation result.
%
%   OUTPUTS
%     R.tf, R.miss   the nominal sweep [s], [ft] -- miss vs FLIGHT TIME, which
%                    is what the listing plots (each point is a whole flight)
%     R.cases        the Ta = 0/5/10 sweeps plus the R = 0 reference
%     R.stab         .mu_lo .mu_hi (neutral bounds), .mu, .maxre, .poles
%     R.bench        benchmark rows; schema shared with the other ports
%     R.selfcheck    structural residuals (self-consistency, NOT book values)
%     R.pass         true only if a row was SCORED and every scored row passed
%     R.verdict      '' when everything passed, else the one-line reason.
%                    ALWAYS FORMED; 'assert' picks the reaction and nothing
%                    else -- false WARNS and still returns R complete.
%     R.units, R.p, R.summary
%
%   See also MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 6.2 line 1, verbatim (zarchan.txt:1875).  Feet and seconds.
    opts.VC    (1,1) double {mustBeFinite, mustBePositive} = 4000    % ft/s
    opts.XNT   (1,1) double {mustBeFinite}                 = 32.2    % ft/s^2, 1 g
    opts.YIC   (1,1) double {mustBeFinite}                 = 0       % ft
    opts.VM    (1,1) double {mustBeFinite, mustBePositive} = 3000    % ft/s
    opts.HEDEG (1,1) double {mustBeFinite}                 = 0       % deg
    opts.TAU   (1,1) double {mustBeFinite, mustBePositive} = 0.5     % s
    opts.XNP   (1,1) double {mustBeFinite, mustBePositive} = 3       % N-prime
    opts.TA    (1,1) double {mustBeFinite, mustBeNonnegative} = 0    % s
    opts.R     (1,1) double {mustBeFinite}                 = -0.01   % radome slope
    opts.TFmax (1,1) double {mustBeFinite, mustBePositive} = 10      % s
    opts.dTF   (1,1) double {mustBeFinite, mustBePositive} = 0.1     % s
    opts.H     (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so "if nargin == 0" here would be a mode that never fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

VC = opts.VC;  XNT = opts.XNT;  VM = opts.VM;  TAU = opts.TAU;
XNP = opts.XNP;  TA = opts.TA;  RD = opts.R;  H = opts.H;

% The five guidance lags each have time constant TAU/5.  H at or above that is
% refused rather than warned about: RK2 does not blow up there, it returns a
% smooth and entirely wrong curve.
if H >= TAU/5
    error('mgs:zarchan:radomeEngagement:stiff', ...
        ['H = %g s is not smaller than the individual lag time constant ' ...
         'TAU/5 = %g s.  The fifth-order binomial would be integrated at or ' ...
         'below one sample per lag.'], H, TAU/5);
end
if opts.dTF < H
    error('mgs:zarchan:radomeEngagement:grid', ...
        'dTF = %g s is finer than the integration step H = %g s.', opts.dTF, H);
end
if opts.TFmax < 2*opts.dTF
    error('mgs:zarchan:radomeEngagement:short', ...
        'TFmax = %g s gives fewer than two flight times at dTF = %g s.', ...
        opts.TFmax, opts.dTF);
end

tfList = opts.dTF:opts.dTF:opts.TFmax;
miss = sweepMiss(VC, XNT, opts.YIC, VM, opts.HEDEG, TAU, XNP, TA, RD, H, tfList);

% ---- the loop's own characteristic polynomial, built from the listing's
%      equations (see the header for the two OCR damages this resolves).
pol = roots(charPolyS(XNP, VC, VM, TAU, TA, RD));

S = struct();
S.mu_lo = abs(bisectNeutral(-3, 0));     % negative-R bound: book prints 0.79
S.mu_hi = bisectNeutral( 5, 0);          % positive-R bound: book prints 2.07
S.mu    = XNP*VC*RD*TA/(VM*TAU);         % this run's parasitic scalar (signed)
if S.mu == 0
    % Not a no-op: a negative R with Ta = 0 (the listing's own defaults)
    % produces NEGATIVE ZERO, which renders as "-0.0000" and reads like a
    % tiny destabilising value rather than the absence of one.
    S.mu = 0;
end
S.maxre = max(real(pol));
S.poles = pol;

% ---- the book's own Ta = 0/5/10 experiment (zarchan.txt:1892-1899), plus the
%      R = 0 reference the Ta = 0 case is claimed to match.
bookCase = abs(TAU-0.5) < 1e-12 && abs(XNP-3) < 1e-12 && abs(RD+0.01) < 1e-12 ...
    && abs(VC-4000) < 1e-9 && abs(VM-3000) < 1e-9 && abs(XNT-32.2) < 1e-9 ...
    && opts.HEDEG == 0 && opts.YIC == 0 && opts.TFmax >= 10;
C = struct('ta', {}, 'peak', {}, 'tail', {}, 'nsc', {}, 'miss', {});
if bookCase
    C = addCase(C, NaN, sweepMiss(VC,XNT,0,VM,0,TAU,XNP,0,0.0,H,tfList), tfList);
    for ta = [0 5 10]
        C = addCase(C, ta, sweepMiss(VC,XNT,0,VM,0,TAU,XNP,ta,RD,H,tfList), tfList);
    end
end

R = struct();
R.tf = tfList;  R.miss = miss;  R.cases = C;  R.stab = S;
R.units = 'ft, ft/s, ft/s^2, s; radome slope dimensionless; g = 32.2 ft/s^2';
R.p = struct('VC',VC, 'XNT',XNT, 'YIC',opts.YIC, 'VM',VM, 'HEDEG',opts.HEDEG, ...
    'TAU',TAU, 'XNP',XNP, 'TA',TA, 'R',RD, 'TFmax',opts.TFmax, ...
    'dTF',opts.dTF, 'H',H, 'bookCase',bookCase, 'units',R.units);

R.selfcheck = structuralChecks(VC, XNT, VM, TAU, XNP, H, tfList);

if opts.benchmark
    R.bench = benchmarks(S, C, bookCase, TAU, XNP, RD, VC, VM);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine".
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R); end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false: R.pass is false meaning NOT CHECKED.';
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated at N''=%g, TAU=%g s, ' ...
        'R=%g.  An unvalidated port must not be handed back silently.'], ...
        XNP, TAU, RD);
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
        error('mgs:zarchan:radomeEngagement:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN, complete, so the caller can see WHICH side moved.
        warning('mgs:zarchan:radomeEngagement:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function miss = sweepMiss(VC, XNT, YIC, VM, HEDEG, TAU, XNP, TA, RD, H, tfList)
%SWEEPMISS  Listing 6.2, zarchan.txt:1875-1890, vectorised across flight times.
%   Every constant arrives as an ARGUMENT: MATLAB local functions cannot see
%   the main function's workspace, and a previous wave in this package threw on
%   its first line for exactly that reason.
[tfs, ord] = sort(tfList(:).');
n = numel(tfs);

Y   = YIC*ones(1,n);
YD  = (-VM*HEDEG/57.3)*ones(1,n);   % the listing's YDIC = YD is never read
XNL = zeros(1,n);  ELAMDH = zeros(1,n);
X4  = zeros(1,n);  X5     = zeros(1,n);
TH  = zeros(1,n);  THH    = zeros(1,n);

T = 0.0;  j0 = 1;
while j0 <= n
    % tfs ascending, so the still-flying trajectories are the suffix j0:n.
    % This is the listing's "while T <= (TF - 1e-5)", one T shared by all.
    while j0 <= n && T > tfs(j0) - 1e-5
        j0 = j0 + 1;
    end
    if j0 > n, break; end
    s = j0:n;

    y = Y(s); yd = YD(s); xnl = XNL(s); el = ELAMDH(s);
    x4 = X4(s); x5 = X5(s); th = TH(s); thh = THH(s);
    yOLD = y; ydOLD = yd; xnlOLD = xnl; elOLD = el;
    x4OLD = x4; x5OLD = x5; thOLD = th; thhOLD = thh;

    % ---- pass 1: derivatives at the current state ------------------------
    [ydd, xnld, eld, x4d, x5d, thd, thhd] = rhs(y, xnl, el, x4, x5, th, thh, ...
        tfs(s) - T + 0.00001, VC, VM, TAU, XNP, XNT, TA, RD);

    % ---- predictor.  ORDER MATTERS: y advances on the OLD yd, exactly as the
    %      listing's sequential assignments do.
    y = y + H*yd;   yd = yd + H*ydd;   xnl = xnl + H*xnld;   el = el + H*eld;
    x4 = x4 + H*x4d;  x5 = x5 + H*x5d;  th = th + H*thd;  thh = thh + H*thhd;
    Tn = T + H;                    % the listing advances T INSIDE the loop

    % ---- pass 2: derivatives at the predicted state ----------------------
    [ydd, xnld, eld, x4d, x5d, thd, thhd] = rhs(y, xnl, el, x4, x5, th, thh, ...
        tfs(s) - Tn + 0.00001, VC, VM, TAU, XNP, XNT, TA, RD);

    % ---- corrector, in the listing's own algebra.  y uses the PREDICTED yd.
    y   = 0.5*(yOLD   + y   + H*yd);
    yd  = 0.5*(ydOLD  + yd  + H*ydd);
    xnl = 0.5*(xnlOLD + xnl + H*xnld);
    el  = 0.5*(elOLD  + el  + H*eld);
    x4  = 0.5*(x4OLD  + x4  + H*x4d);
    x5  = 0.5*(x5OLD  + x5  + H*x5d);
    th  = 0.5*(thOLD  + th  + H*thd);
    thh = 0.5*(thhOLD + thh + H*thhd);
    % The listing writes "FLAG=0" here; dead, see the header.

    Y(s) = y; YD(s) = yd; XNL(s) = xnl; ELAMDH(s) = el;
    X4(s) = x4; X5(s) = x5; TH(s) = th; THH(s) = thh;
    T = Tn;
end

miss = zeros(1, n);
miss(ord) = Y;
end


function [YDD, XNLD, ELAMDHD, X4D, X5D, THD, THHD] = rhs(Y, XNL, ELAMDH, ...
    X4, X5, TH, THH, TGO, VC, VM, TAU, XNP, XNT, TA, RD)
%RHS  The block between the listing's "end" and its "FLAG=1", verbatim.
%   EPS is the boresight error: lambda - theta - theta_H + R*theta_H.  The
%   radome enters ONLY through that +R*THH -- delete it and the parasitic path
%   is severed while every state still integrates smoothly (self-check 4).
XLAM    = Y./(VC*TGO);
EPS     = XLAM - TH - THH + RD*THH;
DD      = 5.*EPS/TAU;
ELAMDHD = 5.*(DD - ELAMDH)/TAU;
XNC     = XNP*VC*ELAMDH;
X4D     = 5.*(XNC - X4)/TAU;
X5D     = 5.*(X4 - X5)/TAU;
XNLD    = 5.*(X5 - XNL)/TAU;
THD     = XNL/VM + TA*XNLD/VM;      % (1 + Ta*s)/VM -- see the header
THHD    = DD - THD;
YDD     = XNT - XNL;
end


% =============================================================================
function c = charPolyS(XNP, VC, VM, TAU, TA, RD)
%CHARPOLYS  Characteristic polynomial of the guidance loop of Fig. 6.21, in s,
%   derived from the listing's own equations:
%
%     [(1-R) + s*T/5]*(1 + s*T/5)^4 + (N'*Vc*R/Vm)*(1 + Ta*s)
%
%   The (1-R) is exact: the seeker tracking loop closes through
%   eps = lambda - (1-R)*theta_s - R*theta, so its lag is T/(5(1-R)) and its
%   gain 1/(1-R).  The book prints the R = 0 version, (1+sT/5)^5 + ..., which
%   differs by 1% at R = -0.01 -- kept exact here because self-check 1 asks the
%   polynomial to collapse to the pure fifth-order binomial at R = 0
%   (zarchan.txt:1866) and an approximation would make that check vacuous.
b = [TAU/5, 1];
q = 1;
for i = 1:4
    q = conv(q, b);
end
c = conv([TAU/5, 1-RD], q);
c = polyAdd(c, (XNP*VC*RD/VM)*[TA, 1]);
end


function c = asymPoly(mu)
%ASYMPOLY  The book's OWN transfer function, normalised by sigma = s*T, in the
%   Ta/T -> infinity limit its inequality is stated for (zarchan.txt:5405):
%
%       (1 + sigma/5)^5 + mu*sigma,     mu = N'*Vc*R*Ta/(Vm*T)
%
%   The constant part of the parasitic term is k = mu*T/Ta -> 0 at fixed mu,
%   which is what makes the bound depend on mu alone.
c = 1;
for i = 1:5
    c = conv(c, [1/5, 1]);
end
c(end-1) = c(end-1) + mu;      % the sigma^1 coefficient
end


function mu = bisectNeutral(aUnstable, bStable)
%BISECTNEUTRAL  The neutral-stability mu, by bisection on the largest real part
%   of the roots.  Deliberately NOT the closed form sec^5(phi)/(5 tan phi):
%   this has to be an independent numeric route to the book's 0.79 and 2.07,
%   not a restatement of the algebra that predicts them.
a = aUnstable;  b = bStable;
for i = 1:200
    m = 0.5*(a + b);
    if max(real(roots(asymPoly(m)))) > 0
        a = m;
    else
        b = m;
    end
    if abs(b - a) < 1e-13, break; end
end
mu = 0.5*(a + b);
end


function c = polyAdd(a, b)
n = max(numel(a), numel(b));
c = [zeros(1, n-numel(a)), a(:).'] + [zeros(1, n-numel(b)), b(:).'];
end


function C = addCase(C, ta, m, tfList)
%ADDCASE  peak = max|miss| over the sweep; tail = max|miss| over the last 20%
%   of flight times; nsc = sign changes along the sweep, which is the book's
%   "more oscillatory" made countable.
%
%   tail is ABSOLUTE, in feet, deliberately: a tail/peak FRACTION is capped at
%   1, so it could not express "the ringing never decays" as a ratio against a
%   case where it does.  At Ta = 10 s the tail IS the peak (both 18.94 ft) --
%   a fraction would have printed 1.000 there and lost the whole finding.
i = numel(C) + 1;
C(i).ta = ta;  C(i).miss = m;
C(i).peak = max(abs(m));
C(i).tail = max(abs(m(tfList >= 0.8*max(tfList))));
C(i).nsc  = sum(diff(sign(m)) ~= 0);
end


% =============================================================================
function K = structuralChecks(VC, XNT, VM, TAU, XNP, H, tfList)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.  They are the weaker
%   claim that it agrees with itself -- which is what catches wiring faults.
K = struct();
names = {};  oks = [];  detail = {};

% 1  R = 0 collapses the loop to a pure fifth-order binomial (zarchan.txt:1866):
%    FIVE poles, all at -5/T.  Band on the location is LOOSE on purpose -- a
%    root of multiplicity 5 is conditioned like eps^(1/5) ~ 7e-4 relative
%    (measured 1.4e-3), so demanding 1e-9 here would fail a correct port.
%
%    ⚠️ THE COUNT IS SCORED SEPARATELY FROM THE LOCATION, and that is not
%    belt-and-braces.  A mutation test on this file built the polynomial with
%    four conv() passes instead of five; every pole was STILL at -5/T, so a
%    location-only check passed a fourth-order loop -- and so did all nine
%    benchmark rows, because the mu bounds come from a different polynomial
%    and rows 8-9 compare orderings that survive an order change.  The count
%    is the only thing in the file that sees it.
p0 = roots(charPolyS(XNP, VC, VM, TAU, 5.0, 0.0));
K.binom_n     = numel(p0);
K.binom_resid = max(abs(p0 + 5/TAU))/(5/TAU);
names{end+1} = 'R = 0 gives the fifth-order binomial (5 poles, all at -5/T)';
oks(end+1)   = K.binom_n == 5 && K.binom_resid <= 5e-3;
detail{end+1} = sprintf('%d poles (want 5); worst off -%.4g by %.3e (band 5e-3)', ...
    K.binom_n, 5/TAU, K.binom_resid);

% 2  No disturbance, no miss.  Every state starts at zero and XNT is the only
%    forcing, so anything nonzero here is spurious excitation.
m0 = sweepMiss(VC, 0, 0, VM, 0, TAU, XNP, 5, -0.01, H, tfList);
K.quiet = max(abs(m0));
names{end+1} = 'no target maneuver and no heading error => zero miss';
oks(end+1)   = K.quiet <= 1e-12;
detail{end+1} = sprintf('max |miss| = %.3e ft', K.quiet);

% 3  The heading-error path is live and decays.  YD's initial condition
%    (-VM*HEDEG/57.3) is the only thing HEDEG touches; a dropped minus sign or
%    a dropped 57.3 leaves check 2 perfectly green.
mh = sweepMiss(VC, 0, 0, VM, 10, TAU, XNP, 0, 0.0, H, tfList);
K.he_peak = max(abs(mh));
K.he_end  = abs(mh(end));
names{end+1} = 'heading error alone produces a miss that decays with tF';
oks(end+1)   = K.he_peak > 1e-6 && K.he_end < K.he_peak;
detail{end+1} = sprintf('peak %.4g ft, tF = %g s value %.4g ft', ...
    K.he_peak, max(tfList), K.he_end);

% 4  With R = 0, Ta CANNOT matter.  theta enters the loop only through the
%    R*THH term -- elsewhere theta and theta_H appear solely as their sum, the
%    seeker's inertial pointing angle, which Ta leaves alone.  This is the
%    check that the parasitic path is wired where the book puts it: mis-wire
%    EPS (drop the -TH, or write R*TH) and this breaks while everything else
%    still runs.  1e-9 of peak, not bit-level: TH and THH are separate states
%    on different rounding paths.
ma = sweepMiss(VC, XNT, 0, VM, 0, TAU, XNP,  0, 0.0, H, tfList);
mb = sweepMiss(VC, XNT, 0, VM, 0, TAU, XNP, 10, 0.0, H, tfList);
K.ta_free = max(abs(ma - mb))/max(max(abs(ma)), realmin);
names{end+1} = 'with R = 0 the miss is independent of Ta';
oks(end+1)   = K.ta_free <= 1e-9;
detail{end+1} = sprintf('max |d(miss)|/peak = %.3e between Ta = 0 and 10 s', K.ta_free);

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
function B = benchmarks(S, C, bookCase, TAU, XNP, RD, VC, VM)
%BENCHMARKS  Computed vs published, with the PROVENANCE of every band.
%
%   *** EVERY BOOK VALUE HERE WAS GREPPED OUT OF zarchan.txt AND THE LINE
%   NUMBER IS IN THE .source FIELD. ***  Chapter 6 publishes NO miss-distance
%   number for the radome case -- Figs. 6.22/6.23/6.24 became image links in
%   the extraction, so nothing can be read off them.  What the chapter DOES
%   publish is prose about those figures, and prose is greppable; those become
%   shape rows.  The only scoreable NUMBERS for this loop live in Chapter 18,
%   which analyses the same block diagram (Fig. 6.21) analytically.
%
%   Do NOT widen a band to turn a row green.  Bands are set by the provenance
%   of the book value, not by what we happen to compute.
B = emptyBench();

% ---- 1-2.  The neutral-stability bounds ------------------------------------
whyMu = ['bisection on max(real(roots)) of the book''s own quintic; the ' ...
    'closed form sec^5(phi)/(5 tan phi) is deliberately NOT used, so this is ' ...
    'a second opinion rather than a restatement.  Printed to 2 dp, so the ' ...
    'band is half of the last digit.'];
B = addVal(B, 'neutral-stability bound |N''VcRTa/(VmT)|, R < 0', 0.79, S.mu_lo, ...
    'text(2dp)', 0.005/0.79, 'p.399 inequality (zarchan.txt:5408)', '-', ...
    [whyMu ' The printed "0.79" lost its minus sign to the OCR; see the header.']);
B = addVal(B, 'neutral-stability bound  N''VcRTa/(VmT), R > 0', 2.07, S.mu_hi, ...
    'text(2dp)', 0.005/2.07, 'p.399 inequality (zarchan.txt:5409)', '-', whyMu);

% ---- 3-4.  Tmin, the book's own two arithmetic examples ---------------------
% Chapter 18 constants, hardcoded: these are the book's worked numbers, not
% this run's parameters.  N'=3, |R|=0.01, Ta=5 s, Vm=3000 ft/s, Vc=4000/9000.
tminOf = @(vc) 3*vc*0.01*5/(0.79*3000);
B = addVal(B, 'Tmin, aircraft threat (Vc = 4000 ft/s)', 0.25, tminOf(4000), ...
    'text(2dp)', 0.005/0.25, 'p.401 worked example (zarchan.txt:5452)', 's', ...
    ['N''VcRTa/(0.79Vm) with the BOOK''s 0.79, not ours -- this row asks only ' ...
     'whether the formula and the constants are the arithmetic the book ' ...
     'printed.  No integrator and no root-finding are involved.']);
B = addVal(B, 'Tmin, ballistic threat (Vc = 9000 ft/s)', 0.57, tminOf(9000), ...
    'text(2dp)', 0.005/0.57, 'p.405 worked example (zarchan.txt:5472)', 's', ...
    'same formula, the book''s second flight condition.');

% ---- 5-7.  Chapter 6's own Ta = 0 / 5 / 10 experiment ----------------------
if bookCase
    ref = C(1);  c0 = C(2);  c5 = C(3);  c10 = C(4);
    d0  = max(abs(c0.miss - ref.miss))/max(ref.peak, realmin);
    B = addBound(B, 'Ta = 0: miss curve matches the no-parasitic case', ...
        '<= 20% of the R = 0 peak', sprintf('%.2f%% of peak', 100*d0), ...
        d0 <= 0.20, 'p.397 text (zarchan.txt:1899)', 'ft', ...
        ['"virtually identical to the case in which there are no parasitic ' ...
         'paths in the homing loop".  At Ta = 0 the parasitic term collapses ' ...
         'to the constant N''VcR/Vm = ' sprintf('%.3g', XNP*VC*RD/VM) ', i.e. ' ...
         'a few percent of loop gain and nothing else; measured 6.50%.  ' ...
         'Normalised by the PEAK, not pointwise, because both curves cross ' ...
         'zero -- and note what that does NOT cover: the two TAILS differ by ' ...
         '500x (0.209 vs 0.00041 ft).  Both are far under a foot against a ' ...
         '10-ft peak, which is what "virtually identical" can honestly mean ' ...
         'here, but a tail-relative reading of this row would be false.']);

    % The row's title says "oscillatory", so the row tests it: sign changes
    % along the sweep, which is the book's own word made countable.  Measured
    % 1 / 3 / 9 at Ta = 0 / 5 / 10 -- a clean monotone ladder.
    okOsc = c5.tail <= 0.5*c5.peak && c5.peak <= 10*c0.peak && c5.nsc > c0.nsc;
    B = addShape(B, 'Ta = 5 s: oscillatory, still small, tends to zero', ...
        'more sign changes than Ta = 0 AND tail <= 0.5*peak AND peak <= 10x', ...
        sprintf('%d sign changes vs %d; tail %.4g ft (%.3f*peak); peak %.4g ft (%.2fx)', ...
                c5.nsc, c0.nsc, c5.tail, c5.tail/max(c5.peak,realmin), ...
                c5.peak, c5.peak/max(c0.peak,realmin)), okOsc, ...
        'p.397 text (zarchan.txt:1899)', ...
        ['"begins to become more oscillatory, but the miss distances are ' ...
         'still small and tend to zero as the flight time increases".  All ' ...
         'three clauses are scored.  mu = -0.40 here, well inside the ' ...
         'boundary, and max Re(s) = -1.303 1/s.']);

    okUns = c10.tail >= 5*c5.tail;
    B = addShape(B, 'Ta = 10 s: the response does not settle', ...
        'tail miss >= 5x the Ta = 5 s tail miss', ...
        sprintf('%.4g ft vs %.4g ft (%.4gx); %d sign changes', c10.tail, ...
                c5.tail, c10.tail/max(c5.tail, realmin), c10.nsc), okUns, ...
        'p.397 text (zarchan.txt:1899); Fig. 6.22 caption (:1893)', ...
        ['"the miss distance response becomes unstable".  Scored as ' ...
         'PERSISTENCE, not divergence, and the distinction is real: the ' ...
         'LINEARISED loop here is still stable, max Re(s) = -0.0152 1/s at ' ...
         'mu = -0.800 against a boundary of -0.806.  The SWEEP sides with ' ...
         'the book anyway -- the largest miss of the whole sweep falls in ' ...
         'the last 20% of flight times and the envelope grows 1.33x from ' ...
         'tF in [3,5] to [8,10].  Nothing runs away: it reaches 18.9 ft and ' ...
         'is still climbing.  A row demanding blow-up would fail on a ' ...
         'correct port.']);
else
    whyOff = sprintf(['off the listing''s own case -- Figs. 6.22-6.24 are ' ...
        'quoted for N''=3, TAU=0.5 s, R=-0.01, Vc=4000, Vm=3000, ' ...
        'nT=32.2 ft/s^2, tF to 10 s; this run used N''=%g, TAU=%g s, R=%g, ' ...
        'Vc=%g, Vm=%g'], XNP, TAU, RD, VC, VM);
    for w = {'Ta = 0: miss curve matches the no-parasitic case', ...
             'Ta = 5 s: oscillatory, still small, tends to zero', ...
             'Ta = 10 s: the response does not settle'}
        B = addSkip(B, w{1}, 'see zarchan.txt:1899', whyOff, ...
            'p.397 text (zarchan.txt:1899)', 'ft');
    end
end

% ---- 8-9.  Figs. 6.23 and 6.24, scored on the loop's roots ------------------
% Both sentences are claims about STABILITY, which the characteristic
% polynomial decides.  The figures publish no numbers, and the sweep's peak
% miss is not what either sentence says, so the root real part is both sharper
% and closer to the claim.  The book's own Ta = 5 s, R = -0.01 throughout.
reOf = @(np, tau) max(real(roots(charPolyS(np, 4000, 3000, tau, 5, -0.01))));
r3 = reOf(3, 0.5);  r5 = reOf(5, 0.5);  r5s = reOf(5, 0.75);
B = addShape(B, 'larger N'' is destabilising at negative radome slope', ...
    'max Re(s) increases from N'' = 3 to N'' = 5', ...
    sprintf('%+.4f -> %+.4f 1/s', r3, r5), r5 > r3, ...
    'p.397 text (zarchan.txt:1900)', ...
    ['"the higher effective navigation ratio has a destabilizing effect ... ' ...
     'we are essentially increasing the guidance system gain".  mu goes ' ...
     '-0.40 -> -0.67 against a boundary near -0.80.']);
B = addShape(B, 'larger TAU is stabilising at negative radome slope', ...
    'max Re(s) decreases from TAU = 0.5 s to 0.75 s (at N'' = 5)', ...
    sprintf('%+.4f -> %+.4f 1/s', r5, r5s), r5s < r5, ...
    'p.397 text (zarchan.txt:1901)', ...
    ['"increasing the guidance system time constant from 0.5 s to 0.75 s has ' ...
     'a stabilizing effect".  Two effects push the same way here: mu falls ' ...
     'to -0.44, and the roots scale as 1/T.']);

% ---- INFO -------------------------------------------------------------------
B = addInfo(B, 'this run''s parasitic scalar and worst root', ...
    'no published value', ...
    sprintf('mu = %+.4f (unstable below %+.4f), max Re(s) = %+.4f 1/s', ...
            S.mu, -S.mu_lo, S.maxre), ...
    'derived from zarchan.txt:5408 and the listing''s own equations', ...
    ['reported, never scored: the book prints no mu for an arbitrary ' ...
     'parameter set.  mu < -0.79 (or > 2.07) is the unstable side.']);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw "Subscripted assignment between dissimilar
%   structures" -- which is the behaviour we want.  A struct array that quietly
%   reordered fields would make [B.ok] mean something else.  Schema is the one
%   benchSuite's adaptGeneric consumes, and adaptGeneric is STRICT.
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
%ADDINFO  Reported, never scored: an INFO row can neither pass nor fail.
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
L{end+1} = 'Zarchan Listing 6.2 -- ENGAGEMENT SIMULATION WITH RADOME EFFECTS';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = '*** WE HAVE NO RADOME: this is the MECHANISM (a parasitic feedback';
L{end+1} = '    path that turns a stable loop unstable), not a configuration. ***';
L{end+1} = sprintf(['   Vc = %g ft/s  Vm = %g ft/s  nT = %g ft/s^2 (%.3g g)  ' ...
    'N'' = %g  TAU = %g s'], p.VC, p.VM, p.XNT, p.XNT/32.2, p.XNP, p.TAU);
L{end+1} = sprintf('   Ta = %g s  R = %g  HE = %g deg  tF = %g:%g:%g s  H = %g s', ...
    p.TA, p.R, p.HEDEG, p.dTF, p.dTF, p.TFmax, p.H);
L{end+1} = '';
L{end+1} = sprintf(['STABILITY   mu = N''VcRTa/(VmT) = %+.4f   stable band ' ...
    '(%+.4f, %+.4f)   max Re(s) = %+.5f 1/s'], ...
    R.stab.mu, -R.stab.mu_lo, R.stab.mu_hi, R.stab.maxre);
[pk, ipk] = max(abs(R.miss));
L{end+1} = sprintf('            nominal sweep: peak |miss| = %.4g ft at tF = %.2f s', ...
    pk, R.tf(ipk));
L{end+1} = '';

if ~isempty(R.cases)
    L{end+1} = 'THE BOOK''S Ta EXPERIMENT (Fig. 6.22; R = -0.01 except the reference)';
    L{end+1} = sprintf('%12s %14s %16s %14s', 'Ta [s]', 'peak |miss|', ...
        'tail |miss|', 'sign changes');
    for i = 1:numel(R.cases)
        c = R.cases(i);
        if isnan(c.ta)
            nm = 'R = 0 ref';
        else
            nm = sprintf('%g', c.ta);
        end
        L{end+1} = sprintf('%12s %14.5g %16.5g %14d', nm, c.peak, c.tail, ...
            c.nsc); %#ok<AGROW>
    end
    L{end+1} = '   tail = max |miss| over the last 20% of flight times [ft].';
    L{end+1} = '   sign changes along the sweep = the book''s "more oscillatory".';
    L{end+1} = '';
end

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-52s %s', vstr(K.oks(i)), K.names{i}, ...
        K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning';
    L{end+1} = '            NOT CHECKED, which is not the same thing as "fine".';
else
    L{end+1} = 'BENCHMARKS  (book values grepped from zarchan.txt; line numbers in .source)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
            case 'info'
                L{end+1} = sprintf('   INFO %-50s %s', b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case 'skip'
                L{end+1} = sprintf('   SKIP %-50s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case {'shape','bound'}
                L{end+1} = sprintf('   %-4s %-50s %s', vstr(b.ok), b.what, ...
                    b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf(['   %-4s %-50s book %s %s / ours %s %s ' ...
                    '(%+.3f%%, band %.4g%%)'], vstr(b.ok), b.what, b.book_str, ...
                    b.units, b.ours_str, b.units, b.pct, 100*b.tol); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped/info -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: the y axis is MISS vs FLIGHT TIME -- every point is a whole';
L{end+1} = '   flight, not an instant of one.  And "unstable" here means the';
L{end+1} = '   ringing never settles, not that anything diverges within 10 s.';
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
%DRAWCURVES  Base MATLAB graphics only, with the listing's own axis labels.
figure;
hold on; grid on;
plot(R.tf, R.miss, '-');
lg = {sprintf('Ta = %g s, R = %g', R.p.TA, R.p.R)};
for i = 1:numel(R.cases)
    c = R.cases(i);
    plot(R.tf, c.miss, '--');
    if isnan(c.ta)
        lg{end+1} = 'R = 0 (no parasitic path)'; %#ok<AGROW>
    else
        lg{end+1} = sprintf('Ta = %g s', c.ta); %#ok<AGROW>
    end
end
xlabel('Flight Time (Sec)');
ylabel('Miss (Ft)');
title('Listing 6.2 -- engagement with radome effects');
legend(lg{:}, 'Location', 'best');
hold off;
end
