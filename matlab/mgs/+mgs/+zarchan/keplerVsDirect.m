function R = keplerVsDirect(opts)
%MGS.ZARCHAN.KEPLERVSDIRECT  Zarchan Listing 28.3 -- direct integration of the
%   ballistic-target equations of motion, checked against Kepler propagation.
%
%   R = mgs.zarchan.keplerVsDirect()
%   R = mgs.zarchan.keplerVsDirect('H', 0.01)        100x cheaper, same answer
%   R = mgs.zarchan.keplerVsDirect('assert', false)  warn instead of error
%
%   Port of Listing 28.3 (zarchan.txt:10475-10485) and its KEPLER1 subroutine
%   (:10486-10505).  A ballistic target leaves the Earth's surface and is
%   propagated 1000 s two independent ways -- 1,000,000 Heun steps of the
%   inverse-square field, and a universal-variable Kepler solve.  AGREEMENT IS
%   THE ENTIRE POINT, and Table 28.1 (:10467-10470) is where the book publishes
%   it.  No figure is involved.  The book's heading reads "KEPLER PROPOGATION"
%   (:10474); that is the book's typo, corrected to PROPAGATION everywhere here.
%
%   UNITS: the listing integrates in FEET and divides by 3280 to print km.
%   3280, NOT 3280.8399 -- GM = 1.4077e16 ft^3/s^2 was chosen to match
%   GMX = 398923 km^3/s^2 under THAT factor, so "correcting" it breaks the very
%   consistency the chapter demonstrates.  Everything returned is km / km/s.
%
% ---------------------------------------------------------------------------
%   *** THE FINDING: Table 28.1's difference is NOT numerical error. ***
%
%   The book says only that "the difference in answers between the two methods
%   is very small" (:10465), which invites reading the ~5 m gap as truncation.
%   It is not.  The two methods are handed DIFFERENT constants:
%       direct integration :  GM  = 1.4077e16 ft^3/s^2 = 398922.544 km^3/s^2
%       Kepler subroutine  :  GMX = 398923 km^3/s^2      (relative gap 1.14e-6)
%   The step-size sweep settles it: the gap is 0.005032 km at H = 0.001 and
%   0.005033 km at H = 0.05 -- FLAT in H, so it cannot be truncation -- while
%   the GM-matched gap falls as H^2 (1.5e-7 km at H = 0.01, 4.1e-6 at H = 0.05).
%   Selfcheck 4 runs that as an explicit DELETE EXPERIMENT.  Both methods are
%   right; the constants disagree.  That is our own recurring shape: two layers,
%   each internally correct, each carrying its own copy of a number.  A gap read
%   as "numerical noise, tighten the step" is one nobody ever closes.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_28_03.m does not parse.  Its own header claims
%   NO unresolved comparisons; one survived.  Repairs, all forced by the book's
%   OCR substitutions ("," is "<", "." is ">"):
%     1  "if (abs(DT) ,TLIMIT)" -> "abs(DT) < TLIMIT".  THE ONE THE HEADER
%        MISSED.  "<" not ">": TLIMIT = 1e-10 is a degeneracy floor and the
%        guarded body copies X0 to X1 unchanged -- right for a ZERO-length
%        propagation, nonsense for a long one.  Dead either way at DT = 1000 s.
%     2  "while T ,= TF" -> "<= TF";  "while STEP ,= 1" -> "<= 1";
%        "if ALP0 ,= 0." -> "<= 0.";  "if ALP0 , 0." -> "< 0.";
%        "if DX2 . 0." -> "> 0." (sqrt(DX2) must be real);  "if S .= 9.9999" ->
%        ">= 9.9999".
%     3  Four lines of the NEXT section's prose (:10506-10509) trail the
%        transcription.  Dropped, not "fixed" into code.
%   Deviations, each because the book's own code is inert:  S (the 9.9999 s
%   record counter) is incremented, reset and stores nothing -- dropped.
%   W = -6.283185/86400 is never read (no ECEF conversion here) -- dropped;
%   A = 2.0926e7 ft is equally unread but kept, for the altitude readout.
%   KEPLER1's degenerate-DT branch gets a RETURN: as printed it copies X0 to X1,
%   falls through, divides by DT = 0 and yields NaN, so the guard does nothing
%   at all (unreachable at DT = 1000 s).  A0 is computed and never read, kept
%   inert; DX0/DX1 likewise but wired into DX2 (same two products, same order,
%   bit for bit) so no dead line survives to be "cleaned up" by someone who
%   cannot tell it was the book's.  KN = 10 FIXED Laguerre passes with no
%   convergence test: kept, and it is what makes the routine bit-deterministic.
%
%   *** THE LOOP RUNS ONE STEP PAST TF, AND IT SHOWS IN TABLE 28.1. ***
%   "while T <= TF" with T built by 1,000,000 additions of 0.001 leaves
%   T = 999.999999983 -- systematically BELOW 1000 by 1.7e-8 -- so a 1,000,001st
%   step runs and the loop stops at 1000.001 s.  One step is |v|*H = 4.83 m, the
%   FOURTH decimal of Table 28.1, whose Integration row is the state at exactly
%   1000.000 s.  So the literal loop runs AND the step-1,000,000 sample is
%   captured: R.int (scored) is that sample, R.int_end (reported) is where the
%   loop stops.  The loop is the book's; only the choice of WHICH recorded
%   sample to lay against Table 28.1 is ours, and it is the epoch KEPLER1 is
%   itself called at (T1 = TF).
%
% ---------------------------------------------------------------------------
%   BENCHMARKS: Table 28.1 only -- a TABLE, so it survived extraction intact,
%   and Chapter 28 publishes no other number for THIS listing.  Bands: the
%   Kepler rows come back inside ONE unit of the table's last printed place
%   (band 2e-4 km = two of them); the Integration rows inside THREE, and that
%   residual is FLAT in H (identical to 6 decimals at H = 0.001, 0.01 and 0.05)
%   so it is neither our step size nor our arithmetic -- 3.5e-8 of |r|, origin
%   in the book's own run UNKNOWN, band 1e-3 km, one decade below the book's own
%   two-method spread (worst component 0.0039 km) and we sit at 28% of it.  The
%   agreement row is relative to |r| = 8062 km at 3e-6, five times the 1.14e-6
%   GM gap that FLOORS it; tighter would red-light a correct port.  What it
%   CANNOT do: one step of epoch error is also ~6e-7 of |r|, and selfcheck 5 is
%   what notices that.  Do not widen any of these to turn a row green.
%
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED. ***  MATLAB is the
%   caller's, not this author's; these came from an independent transcription of
%   the same listing in another language -- a prediction to check, not a run:
%     Kepler @1000 s   2547.414831  -3583.107397   6758.238431 km
%                        -3.5585100   -3.2456151    -0.3794558 km/s
%     Direct @1000 s   2547.417781  -3583.108080   6758.242451 km
%                        -3.5585047   -3.2456178    -0.3794471 km/s
%     |r_int - r_kep| = 0.005032 km (Table 28.1 itself implies 0.004810 km)
%     delete experiment 0.005033 km -> 1.5e-7 km, a factor of ~3.3e4
%     literal loop 1,000,001 steps, t_end = 1000.001 s
%   A ~5 m two-method gap is CORRECT and expected -- it is the constant
%   mismatch.  A gap near ZERO means somebody harmonised the two GMs and
%   selfcheck 4 has gone vacuous.
%
%   OUTPUTS: R.kep / R.int (1x6 [km, km/s] at t = TF), R.int_end (where the
%   literal loop stops), R.err (= R.int - R.kep, the listing's ERRX..ERRZD),
%   R.book (Table 28.1 as published), R.gm, R.delete_exp, R.bench, R.selfcheck,
%   R.pass, R.verdict, R.p, R.summary, R.units.
%
%   See also MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 28.3 line 1, verbatim.  FEET and ft/s, "obtained from output of
    % Listing C28L2" -- the target is on the Earth's surface at t = 0.
    opts.X0ft (1,6) double {mustBeFinite} = ...
        [14990432.9744621, 1952093.10305573, 14469752.1663352, ...
           996.773566434768, -14954.8124604715, 17528.1931768263]
    opts.TF   (1,1) double {mustBeFinite, mustBePositive} = 1000.0    % s
    opts.H    (1,1) double {mustBeFinite, mustBePositive} = 0.001     % s
    % The two constants whose 1.14e-6 disagreement IS Table 28.1's gap.
    opts.GM   (1,1) double {mustBeFinite, mustBePositive} = 1.4077e16 % ft^3/s^2
    opts.GMX  (1,1) double {mustBeFinite, mustBePositive} = 398923.0  % km^3/s^2
    opts.A    (1,1) double {mustBeFinite, mustBePositive} = 2.0926e7  % ft, radius
    opts.FTKM (1,1) double {mustBeFinite, mustBePositive} = 3280.0    % the book's
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    % assert=false WARNS and still returns R complete, so a red row can be
    % diagnosed rather than merely thrown.  benchSuite runs every port that way.
    opts.assert    (1,1) logical = true
end

TF = opts.TF;  H = opts.H;  FTKM = opts.FTKM;

nTF = round(TF/H);
if nTF < 2 || abs(nTF*H - TF) > 0.25*H
    error('mgs:zarchan:keplerVsDirect:grid', ...
        ['TF = %g s is not an integer number of steps of H = %g s (nearest is ' ...
         '%d steps = %.9g s).  The benchmark epoch would then sit a fraction ' ...
         'of a step from the epoch KEPLER1 is called at, and one step is ' ...
         '%.3g km -- visible in Table 28.1''s last digit.'], ...
        TF, H, nTF, nTF*H, H*norm(opts.X0ft(4:6))/FTKM);
end

X0km = opts.X0ft / FTKM;

% ---- Listing 28.3's two propagations ---------------------------------------
[Ikm, Iend, nStep, Tend] = runDirect(opts.X0ft, opts.GM, TF, H, FTKM, nTF);
Kkm = kepler1(X0km, 0.0, TF, opts.GMX);

R = struct();
R.kep     = Kkm;
R.int     = Ikm;
R.int_end = Iend;
R.err     = Ikm  - Kkm;        % the listing's ERRX ERRY ERRZ ERRXD ERRYD ERRZD
R.err_end = Iend - Kkm;
R.n_steps = nStep;
R.T_end   = Tend;
R.T_int   = nTF*H;
R.alt_km  = [norm(X0km(1:3)), norm(Kkm(1:3))] - opts.A/FTKM;

% Table 28.1, zarchan.txt:10469-10470.  Typed once, here, and nowhere else.
R.book = struct( ...
    'integration', [2547.4175, -3583.1082, 6758.2423, ...
                      -3.558504, -3.245617, -0.379447], ...
    'kepler',      [2547.4148, -3583.1074, 6758.2384, ...
                      -3.558509, -3.245615, -0.379455], ...
    'source', 'Table 28.1, p.737 (zarchan.txt:10467-10470)');
R.book.err = R.book.integration - R.book.kepler;

R.gm = struct('ft3', opts.GM, 'km3_implied', opts.GM/FTKM^3, ...
              'km3_kepler', opts.GMX, 'rel', opts.GM/FTKM^3/opts.GMX - 1);

R.p = struct('TF',TF, 'H',H, 'n_steps_to_TF',nTF, 'GM',opts.GM, ...
             'GMX',opts.GMX, 'A',opts.A, 'FTKM',FTKM, 'X0ft',opts.X0ft, ...
             'units','km, km/s; GM in ft^3/s^2, GMX in km^3/s^2');
R.units = ['km and km/s (the listing divides feet by the book''s 3280 ft/km); ' ...
           'GM in ft^3/s^2, GMX in km^3/s^2'];

[R.selfcheck, R.delete_exp] = structuralChecks(R, opts, TF, H, FTKM, nTF, X0km);

if opts.benchmark
    R.bench = benchmarks(R, FTKM);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    R.pass  = false;      % NOT CHECKED is not the same thing as "fine".
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false: nothing was compared against Table 28.1.';
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is wiring, not tolerance.  First: %s.'], ...
        sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf(['%d of %d scored rows are outside tolerance (first: %s, ' ...
        'book %s, ours %s).'], numel(bad), sum(ev), R.bench(bad(1)).what, ...
        R.bench(bad(1)).book_str, R.bench(bad(1)).ours_str);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:keplerVsDirect:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:keplerVsDirect:benchmark', ...
            '%s  (''assert'' false, R returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [Ktf, Kend, n, T] = runDirect(X0ft, GM, TF, H, FTKM, nTF)
%RUNDIRECT  Listing 28.3's main loop, zarchan.txt:10475-10485.  The book's own
%   predictor/corrector (Heun) on the inverse-square field, in FEET.
%
%   Every constant is an ARGUMENT: MATLAB local functions cannot see the main
%   function's workspace, and this package has already lost a wave to that.
%
%   Ktf is the sample at step nTF (t = TF exactly).  Kend is where the literal
%   "while T <= TF" loop stops -- ONE STEP LATER.  See the header.

XT  = X0ft(1);  YT  = X0ft(2);  ZT  = X0ft(3);
XTD = X0ft(4);  YTD = X0ft(5);  ZTD = X0ft(6);
XTDD = 0.0;  YTDD = 0.0;  ZTDD = 0.0;   % set before the loop so the FLAG==1
T = 0.0;  n = 0;                        % branch cannot read a stale value
Stf = nan(1,6);

while T <= TF
    XTOLD  = XT;   YTOLD  = YT;   ZTOLD  = ZT;
    XTDOLD = XTD;  YTDOLD = YTD;  ZTDOLD = ZTD;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass loop: pass 1 at the current state, pass 2 after a full
    % Euler step with T advanced by H.  T advances INSIDE here, exactly once;
    % hoisting it out silently degrades the scheme to first order.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            XT  = XT  + H*XTD;   YT  = YT  + H*YTD;   ZT  = ZT  + H*ZTD;
            XTD = XTD + H*XTDD;  YTD = YTD + H*YTDD;  ZTD = ZTD + H*ZTDD;
            T   = T + H;
        end
        TEMPBOTT = (XT^2 + YT^2 + ZT^2)^1.5;
        XTDD = -GM*XT/TEMPBOTT;
        YTDD = -GM*YT/TEMPBOTT;
        ZTDD = -GM*ZT/TEMPBOTT;
        FLAG = 1;
    end
    % The listing's trailing "FLAG=0" is dropped: the outer loop reinitialises
    % FLAG, so that assignment can never be read.

    % Corrector, in the listing's own algebra.  ORDER MATTERS: position uses the
    % PREDICTED velocity, before velocity is itself corrected.
    XT  = .5*(XTOLD  + XT  + H*XTD);
    YT  = .5*(YTOLD  + YT  + H*YTD);
    ZT  = .5*(ZTOLD  + ZT  + H*ZTD);
    XTD = .5*(XTDOLD + XTD + H*XTDD);
    YTD = .5*(YTDOLD + YTD + H*YTDD);
    ZTD = .5*(ZTDOLD + ZTD + H*ZTDD);
    % S = S + H;  if S >= 9.9999, S = 0.; end   -- vestigial; see the header.

    n = n + 1;
    if n == nTF
        Stf = [XT YT ZT XTD YTD ZTD];
    end
end

if any(isnan(Stf))
    error('mgs:zarchan:keplerVsDirect:epoch', ...
        ['the loop stopped after %d steps without reaching step %d (t = TF). ' ...
         'The benchmark epoch does not exist in this run, and scoring the ' ...
         'final state instead would silently compare two different times.'], ...
        n, nTF);
end
Ktf  = Stf / FTKM;
Kend = [XT YT ZT XTD YTD ZTD] / FTKM;
end


% =============================================================================
function X1 = kepler1(X0, T0, T1, GMX)
%KEPLER1  Listing 28.3's subroutine, zarchan.txt:10486-10505.  Universal
%   variables with Laguerre iteration (n = 5), KN = 10 fixed passes, no
%   convergence test.  km and km/s in and out.  REX is only a nondimensionalising
%   scale and cancels exactly; GMX is the sole physical constant.
REX = 6380.;  TLIMIT = 1.E-10;  KN = 10;  MUQR = 1.;
DT = T1 - T0;
if abs(DT) < TLIMIT
    X1 = X0;      % RETURN added.  As printed this falls through, and the
    return        % fall-through divides by DT = 0.  See the header.
end

TIME_FACTOR = sqrt(REX^3/GMX);
VEX = REX/TIME_FACTOR;
DT  = DT/TIME_FACTOR;
R0  = X0(1:3)/REX;
V0  = X0(4:6)/VEX;

R0MAG  = sqrt(R0(1)^2 + R0(2)^2 + R0(3)^2);
V0MAG  = sqrt(V0(1)^2 + V0(2)^2 + V0(3)^2);
D0     = R0(1)*V0(1) + R0(2)*V0(2) + R0(3)*V0(3);
SIGMA0 = D0/MUQR;
ALP0   = 2./R0MAG - V0MAG*V0MAG;      % reciprocal semi-major axis

% A0 is computed by the listing and never read.  Kept so the transcription
% reads against the page.
if ALP0 == 0.
    A0 = 1.E30;   %#ok<NASGU>
else
    A0 = 1./ALP0; %#ok<NASGU>
end

X = ALP0*DT;
if ALP0 <= 0.
    X = .1*DT/R0MAG;
end

U1 = 0.;  U2 = 0.;  U3 = 0.;  DFX = 0.;
for K = 1:KN
    if ALP0 < 0.
        Y  = ALP0*X*X;   YQR = sqrt(-Y);
        CY = (1.-cosh(YQR))/Y;
        SY = (sinh(YQR) - YQR)/(YQR^3);
    elseif ALP0 == 0.
        Y = 0.;  CY = .5;  SY = 1./6.;
    else
        Y  = ALP0*X*X;   YQR = sqrt(Y);
        CY = (1.-cos(YQR))/Y;
        SY = (YQR - sin(YQR))/(YQR^3);
    end
    U1 = X*(1. - Y*SY);
    U2 = X*X*CY;
    U3 = X*X*X*SY;
    FX   = R0MAG*U1 + SIGMA0*U2 + U3 - DT*MUQR;
    DFX  = SIGMA0*U1 + (1. - ALP0*R0MAG)*U2 + R0MAG;
    DFX2 = SIGMA0*(1. - Y*CY) + (1. - ALP0*R0MAG)*U1;
    SDFX = DFX/(abs(DFX));
    % The listing computes DX0 and DX1, never reads them, and then forms DX2
    % as "16.*DFX*DFX - 20.*FX*DFX2".  Wired through DX0/DX1 here: the same
    % two products in the same order, bit for bit, but no dead code left to be
    % "cleaned up" later by someone who cannot see it was the book's.
    DX0 = 16.*DFX*DFX;
    DX1 = 20.*FX*DFX2;
    DX2 = DX0 - DX1;
    if DX2 > 0.
        DX = 5.*FX/(DFX + SDFX*sqrt(DX2));
    else
        DX = .5*X;
    end
    X = X - DX;
end

RMAG = DFX;
F  = 1. - U2/R0MAG;
G  = DT - U3/MUQR;
DF = -MUQR*U1/(RMAG*R0MAG);
DG = 1. - U2/RMAG;
X1 = zeros(1,6);
X1(1:3) = (F*R0  + G*V0 )*REX;
X1(4:6) = (DF*R0 + DG*V0)*VEX;
end


% =============================================================================
function [K, D] = structuralChecks(R, opts, TF, H, FTKM, nTF, X0km)
%STRUCTURALCHECKS  Self-consistency.  NOT book values -- these must never be
%   quoted as evidence that the port reproduces Zarchan.  .names and .oks are
%   what puts them on the board: a delete experiment nobody can see is the same
%   as one nobody ran.
K = struct();  names = {};  oks = [];  detail = {};

muI = opts.GM/FTKM^3;     % what the direct integration actually flies
muK = opts.GMX;           % what KEPLER1 flies

% 1  KEPLER1 returns a point on a conic, so specific energy is exact.
e0 = spEnergy(X0km, muK);  e1 = spEnergy(R.kep, muK);
K.kep_energy_rel = abs(e1/e0 - 1);
names{end+1}  = 'Kepler conserves specific energy (exact conic)';
oks(end+1)    = K.kep_energy_rel <= 1e-12;
detail{end+1} = sprintf('|dE/E| = %.3e', K.kep_energy_rel);

% 2  Heun is NOT symplectic, so this one can drift.  Over 1e6 steps it must not.
f0 = spEnergy(X0km, muI);  f1 = spEnergy(R.int, muI);
K.int_energy_rel = abs(f1/f0 - 1);
names{end+1}  = 'direct integration conserves the same energy';
oks(end+1)    = K.int_energy_rel <= 1e-9;
detail{end+1} = sprintf('|dE/E| = %.3e over %d Heun steps', K.int_energy_rel, nTF);

% 3  Central force => angular momentum fixed in magnitude AND direction.  This
%    is what would catch a sign or an index swapped in the gravity term while
%    the energy check still looked healthy.
h0 = cross(X0km(1:3), X0km(4:6));
K.h_dev = max([angBetween(h0, cross(R.kep(1:3), R.kep(4:6))), ...
               angBetween(h0, cross(R.int(1:3), R.int(4:6)))]);
names{end+1}  = 'angular momentum direction fixed (both methods)';
oks(end+1)    = K.h_dev <= 1e-9;
detail{end+1} = sprintf('worst tilt %.3e rad', K.h_dev);

% 4  *** DELETE EXPERIMENT. ***  Hand the integrator the Kepler routine's OWN
%    constant.  If Table 28.1's gap is truncation this changes nothing; if it is
%    the constant mismatch, the gap collapses.  Run at 10*H, a tenth of the
%    cost: the mechanism is a physics mismatch, not a step-size effect, so the
%    coarser grid tests it just as well and the H^2 residual stays ~65x below
%    the threshold.
Hc = 10*H;  nc = round(TF/Hc);
D  = struct('H',Hc, 'gap_book_gm',NaN, 'gap_matched_gm',NaN, 'ratio',NaN);
if nc >= 2 && abs(nc*Hc - TF) <= 0.25*Hc
    Ib = runDirect(opts.X0ft, opts.GM,         TF, Hc, FTKM, nc);
    Im = runDirect(opts.X0ft, opts.GMX*FTKM^3, TF, Hc, FTKM, nc);
    D.gap_book_gm    = norm(Ib(1:3) - R.kep(1:3));
    D.gap_matched_gm = norm(Im(1:3) - R.kep(1:3));
    D.ratio = D.gap_book_gm/max(D.gap_matched_gm, realmin);
    ok4 = D.gap_book_gm >= 1e-3 && D.gap_matched_gm <= 1e-5;
    detail{end+1} = sprintf(['GM as printed %.6f km  ->  GM = GMX*%g^3 ' ...
        '%.3e km (factor %.2g), at H = %g s'], D.gap_book_gm, FTKM, ...
        D.gap_matched_gm, D.ratio, Hc);
else
    ok4 = false;
    detail{end+1} = sprintf(['not run: TF = %g s is not an integer multiple ' ...
        'of 10*H = %g s'], TF, Hc);
end
names{end+1} = 'DELETE EXP: matching GM collapses the two-method gap';
oks(end+1)   = ok4;

% 5  The scored sample really is at t = TF -- and the literal loop really does
%    overshoot.  One step is 4.83 m here, which the agreement row is too coarse
%    to see (both are ~6e-7 of |r|), so this is the only thing that notices.
K.epoch_err    = abs(R.T_int - TF);
K.overshoot_km = norm(R.int_end(1:3) - R.int(1:3));
names{end+1}   = 'scored sample is at t = TF (loop overshoot accounted for)';
oks(end+1)     = K.epoch_err <= 1e-9 && R.n_steps >= nTF;
detail{end+1}  = sprintf(['t_scored = %.9f s (|d| %.1e s); the literal loop ' ...
    'ran %d steps to t = %.6f s, worth %.6f km of position'], R.T_int, ...
    K.epoch_err, R.n_steps, R.T_end, K.overshoot_km);

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

function e = spEnergy(S, mu)
e = 0.5*norm(S(4:6))^2 - mu/norm(S(1:3));
end

function a = angBetween(u, v)
c = dot(u, v)/(norm(u)*norm(v));
a = acos(max(-1, min(1, c)));
end


% =============================================================================
function B = benchmarks(R, FTKM)
%BENCHMARKS  Table 28.1 (zarchan.txt:10467-10470) and nothing else.  Chapter 28
%   publishes no other number for THIS listing -- Figs. 28.1-28.9 belong to
%   Listings 28.1 and 28.2 -- so the row count is small on purpose.
src = R.book.source;
B   = emptyBench();
lbl = {'x', 'y', 'z'};

% ---- 1-3.  KEPLER position, component by component.  The sharp ones: four
%      decimal places on an 8062 km state is 1.2e-8 relative.
for i = 1:3
    B = addAbs(B, sprintf('Kepler %s(t=%g s)', lbl{i}, R.p.TF), ...
        R.book.kepler(i), R.kep(i), 2e-4, 'table', src, 'km', ...
        ['universal variables, KN = 10 fixed Laguerre passes; band is two ' ...
         'units of Table 28.1''s last printed place']);
end

% ---- 4.  KEPLER velocity, all three components in one bound row. -----------
dv = max(abs(R.kep(4:6) - R.book.kepler(4:6)));
B = addBound(B, 'Kepler velocity vs Table 28.1 (3 components)', ...
    'max |d| <= 3e-6 km/s (three last places)', ...
    sprintf('max |d| = %.3e km/s', dv), dv <= 3e-6, src, 'km/s', ...
    'one row, not three: the components share a single mechanism');

% ---- 5-6.  DIRECT INTEGRATION, at the step-nTF epoch. ----------------------
dr = max(abs(R.int(1:3) - R.book.integration(1:3)));
B = addBound(B, 'direct integration position vs Table 28.1', ...
    'max |d| <= 1e-3 km', sprintf('max |d| = %.3e km', dr), dr <= 1e-3, ...
    src, 'km', ...
    ['band is one decade below the book''s own two-method spread (worst ' ...
     'component 0.0039 km).  The residual is FLAT in H, so it is not our step ' ...
     'size; its origin in the book''s own run is unknown']);

dvi = max(abs(R.int(4:6) - R.book.integration(4:6)));
B = addBound(B, 'direct integration velocity vs Table 28.1', ...
    'max |d| <= 3e-6 km/s (three last places)', ...
    sprintf('max |d| = %.3e km/s', dvi), dvi <= 3e-6, src, 'km/s', ...
    'velocity is reproduced far better than position, as it must be');

% ---- 7.  THE ROW THIS LISTING EXISTS FOR: the two methods agree. -----------
rel     = norm(R.err(1:3))/norm(R.kep(1:3));
relBook = norm(R.book.err(1:3))/norm(R.book.kepler(1:3));
B = addBound(B, 'THE AGREEMENT: |r_direct - r_Kepler| / |r|', ...
    'a "very small" difference, scored as <= 3e-6 of |r|', ...
    sprintf('%.3e  (%.6f km; Table 28.1 itself implies %.3e / %.6f km)', ...
            rel, norm(R.err(1:3)), relBook, norm(R.book.err(1:3))), ...
    rel <= 3e-6, [src ' + p.736 text (zarchan.txt:10465)'], '-', ...
    ['threshold is 5x the 1.14e-6 GM gap that FLOORS this quantity; tighter ' ...
     'would red-light a correct port.  It cannot see a one-step epoch error ' ...
     '(also ~6e-7 of |r|) -- selfcheck 5 is what sees that']);

% ---- 8.  The constants themselves, which is where the gap comes from. ------
B = addAbs(B, 'GM/3280^3 vs the Kepler routine''s GMX', ...
    R.gm.km3_kepler, R.gm.km3_implied, 5e11/FTKM^3, 'quantisation', ...
    'listing line 1 + KEPLER1 (zarchan.txt:10475, :10486)', 'km^3/s^2', ...
    sprintf(['GM = 1.4077e16 is quoted to five figures, so its own ' ...
             'quantisation is +/-%.2f km^3/s^2 -- that is the band.  The two ' ...
             'constants sit %.2e apart relatively, and THAT is Table 28.1''s ' ...
             'difference'], 5e11/FTKM^3, abs(R.gm.rel)));

% ---- INFO.  Reported, never scored. ---------------------------------------
B = addInfo(B, 'the listing''s own ERRX..ERRZD, reproduced', ...
    sprintf('Table 28.1 implies [%s] km', strtrim(sprintf('%+.6f ', R.book.err(1:3)))), ...
    sprintf('[%s] km', strtrim(sprintf('%+.6f ', R.err(1:3)))), src, ...
    ['NOT SCORED: a difference of two nearly equal 8062 km numbers inherits ' ...
     'the whole of the Integration row''s last-digit noise, so it is a 5% ' ...
     'comparison dressed as a four-decimal one']);

B = addInfo(B, 'loop termination: "while T <= TF" overshoots', ...
    sprintf('step %d is t = TF', R.p.n_steps_to_TF), ...
    sprintf('%d steps, t_end = %.6f s, %.6f km past the scored sample', ...
            R.n_steps, R.T_end, norm(R.int_end(1:3) - R.int(1:3))), ...
    'zarchan.txt:10475', ...
    ['NOT SCORED: T built by summing H sits systematically 1.7e-8 s BELOW TF ' ...
     'after nTF additions, so the literal loop takes one more step.  Which ' ...
     'sample to score is ours to choose; the loop is the book''s']);

B = addInfo(B, 'altitude: surface at t = 0, high at t = TF', ...
    sprintf('A = %g ft = %.3f km (never read by the listing)', R.p.A, R.p.A/FTKM), ...
    sprintf('%.3f km -> %.3f km', R.alt_km(1), R.alt_km(2)), ...
    'listing line 1 (zarchan.txt:10475)', ...
    ['NOT SCORED: the book publishes no altitude for this case.  Reported ' ...
     'because "the target starts ON the surface" is the sanity check a reader ' ...
     'of Table 28.1 cannot otherwise make']);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER are the schema benchSuite's adaptGeneric
%   consumes, and it is STRICT -- a near-miss errors rather than being
%   accommodated.  Every row is built through the adders below so the order
%   cannot drift.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addAbs(B, what, book, ours, absBand, band, source, units, detail)
%ADDABS  A value row whose band is stated in ABSOLUTE units and converted to the
%   schema's relative tol here.  Table 28.1's precision is absolute (a fixed
%   number of decimal places); quoting a percentage instead would restate the
%   same number in a form nobody can check against the page.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.8g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.10g', ours);
B(i).band = band;
B(i).units = units;  B(i).source = source;
B(i).detail = sprintf('%s  [band %.4g %s]', detail, absBand, units);
if ~isfinite(ours) || book == 0
    B(i).tol = NaN;  B(i).pct = NaN;  B(i).ok = false;  B(i).skipped = true;
    B(i).why = 'the computed value is not finite (or the book value is zero)';
    return
end
B(i).tol = absBand/abs(book);
B(i).pct = 100*(ours/book - 1);
B(i).ok  = abs(ours - book) <= absBand;
B(i).skipped = false;  B(i).why = '';
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
%ADDINFO  skipped = true keeps an INFO row out of the pass count in BOTH
%   directions: it can neither pass nor fail.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'info';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'info';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = '-';  B(i).detail = '';
end


% =============================================================================
function s = render(R, ev)
L = {};
L{end+1} = 'Zarchan Listing 28.3 -- COMPARING DIRECT INTEGRATION WITH KEPLER';
L{end+1} = '   PROPAGATION   (the book''s own heading spells it "PROPOGATION")';
L{end+1} = sprintf(['   TF = %g s   H = %g s   %d steps to TF   GM = %g ' ...
    'ft^3/s^2   GMX = %g km^3/s^2'], R.p.TF, R.p.H, R.p.n_steps_to_TF, ...
    R.p.GM, R.p.GMX);
L{end+1} = sprintf('   units: %s', R.units);
L{end+1} = '';
L{end+1} = 'PREDICTED TARGET STATE AT t = TF            (cf. Table 28.1, p.737)';
L{end+1} = sprintf('%-14s %13s %13s %13s %11s %11s %11s', 'method', ...
    'x [km]', 'y [km]', 'z [km]', 'xd [km/s]', 'yd [km/s]', 'zd [km/s]');
L{end+1} = row6('Integration', R.int);
L{end+1} = row6('  book', R.book.integration);
L{end+1} = row6('Kepler', R.kep);
L{end+1} = row6('  book', R.book.kepler);
L{end+1} = row6('ERR (ours)', R.err);
L{end+1} = row6('ERR (book)', R.book.err);
L{end+1} = '';
L{end+1} = sprintf(['THE GAP IS THE CONSTANT, NOT THE METHOD:  GM/%g^3 = ' ...
    '%.4f  vs  GMX = %.4f km^3/s^2'], R.p.FTKM, R.gm.km3_implied, R.gm.km3_kepler);
L{end+1} = sprintf(['   relative %+.3e;  two-method position gap %.6f km ' ...
    '(%.3e of |r|)'], R.gm.rel, norm(R.err(1:3)), ...
    norm(R.err(1:3))/norm(R.kep(1:3)));
L{end+1} = '';

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
    L{end+1} = 'BENCHMARKS  (Table 28.1; line numbers in .source)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
            case 'info'
                L{end+1} = sprintf('   INFO %-46s %s', b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        book: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case 'bound'
                L{end+1} = sprintf('   %-4s %-46s %s', vstr(b.ok), b.what, ...
                    b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf('   %-4s %-46s book %s / ours %s %s (%+.3e%%)', ...
                    vstr(b.ok), b.what, b.book_str, b.ours_str, b.units, ...
                    b.pct); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d info -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: "the difference in answers between the two methods is very';
L{end+1} = '   small" is a statement about TWO GRAVITATIONAL CONSTANTS, not about';
L{end+1} = '   truncation.  Give the integrator GMX and the gap falls ~4.5 decades.';
s = strjoin(L, newline);
end

function s = row6(name, v)
s = sprintf('%-14s %13.6f %13.6f %13.6f %11.6f %11.6f %11.6f', name, v);
end

function s = vstr(tf)
if tf
    s = 'PASS';
else
    s = 'FAIL';
end
end
