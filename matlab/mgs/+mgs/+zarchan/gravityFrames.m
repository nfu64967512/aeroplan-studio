function R = gravityFrames(opts)
%MGS.ZARCHAN.GRAVITYFRAMES  Zarchan Listing 11.2 -- one gravity field, two
%   coordinate systems, integrated side by side in a single loop.
%
%   R = mgs.zarchan.gravityFrames()
%   R = mgs.zarchan.gravityFrames('V', 20000, 'GAM', 30)
%   R = mgs.zarchan.gravityFrames('assert', false, 'plot', true)
%
%   Line-by-line port of Listing 11.2 (zarchan.txt:3326-3337), integrated by
%   the book's own second-order Runge-Kutta (Heun) scheme at H = 0.01 s.
%
%   The listing exists for ONE reason, stated at zarchan.txt:3324: to
%   "demonstrate that the polar and Cartesian Earth-centered differential
%   equations are equivalent".  Both sets are advanced in the SAME outer loop
%   from the same launch condition, and the book's verdict (zarchan.txt:3339)
%   is that "the resultant trajectories are identical for all practical
%   purposes".  That is a comparison of two computed curves, not a figure
%   reading, so it is the strongest scoreable claim in the chapter and it is
%   BENCHMARKS 1-2.  The chapter publishes exactly two other numbers about
%   this run and nothing else; Ch.11 is a derivation chapter, and four
%   scoreable rows is the whole harvest.
%
%     Cartesian, Earth-centred inertial (X, Y, X1, Y1):
%         xddot = -gm*x/(x^2+y^2)^1.5 ,   yddot = -gm*y/(x^2+y^2)^1.5
%     Polar (R0, R1, PSI), derived at zarchan.txt:3299-3318:
%         rddot - r*psidot^2 + gm/r^2 = 0 ,   r^2*psidot = (a+alt)*V*cos(gam)
%   The polar form carries angular momentum as an EXACT first integral (PSID
%   is evaluated from the constant every step); the Cartesian form has to earn
%   it from the integrator.  SELFCHECK 1 measures that asymmetry.
%
%   UNITS ARE THE BOOK'S: FEET and ft/s, distances REPORTED in n.mi. at the
%   book's 6076 ft/n.mi.  Nothing is converted to metres; convert at the call
%   site and say so there.  Unlike 11.1 this listing has no G = 32.2 anywhere:
%   both of its channels are inverse-square, neither is flat-Earth.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  matlab/zarchan/_listings/listing_11_02.m does not parse
%   as MATLAB.  Its own header reports NO unresolved variable-to-variable
%   comparisons and that is correct -- every comparison in 11.2 is against a
%   LITERAL (FLAG==1, STEP<=1, S>=9.99999, ALTNM>-.0001), so nothing below is a
%   judgement call about which way an `if A.B` points.  Repairs, all forced:
%
%     1  "count=0; H=.01 A=2.0926e7;" -> ";" restored after .01; two statements
%        ran together (:3326).  11.1, one page earlier (:3245), has it.
%     2  "while ALTNM . -.0001" -> "> -.0001".  "." is this book's OCR of ">";
%        ALTNM starts at 0, so "<" never enters the loop.
%     3  "while STEP ,=1" -> "<= 1".  "," is the OCR of "<"; the body sets
%        STEP=2 to leave, which only works against "<=".
%     4  "if S.=9.99999" -> ">=".  S is zeroed on every record, so "<=" would
%        record forever.
%     5  Transcription lines 19-20 are the book's NEXT paragraph and next
%        section heading leaking into the code block -- the one unresolved item
%        its header flags.  Dropped, not "fixed" into code.  That paragraph is
%        zarchan.txt:3339 and it supplies BENCHMARKS 3 and 4, which is how it
%        was identified.
%     6  "save datfil.txt output /ascii" is the book's typo for -ascii; nothing
%        is written to disk here.  "SCOUNT=0." is dead in the listing.
%
%   ONE ERRATUM, FIXED, AND SHOWN TO BE INERT HERE.  11.1 and 11.2 both write
%   "ALT=ALTNM/6076." -- n.mi. to feet by DIVISION.  Every other listing in the
%   chapter multiplies (:3459, :3484, :3765).  This port multiplies.  At the
%   published ALTNM = 0 both readings are 0, so no scored row moves either way;
%   SELFCHECK 3 is what would catch it for a caller who sets ALTNM ~= 0.
%
%   TWO DEVIATIONS, NEITHER ABLE TO MOVE A SCORED ROW.  (a) The listing's first
%   record is at t = samp; this port also records t = 0.  Apogee is mid-flight
%   and impact is interpolated between the last two samples, so no benchmark
%   reads index 1.  (b) acos(CBETA) is clamped to [-1,1]: at t = 0 CBETA rounds
%   to 1+2e-16 and acos returns complex.  The listing never evaluates it there.
%
%   ONE ARTEFACT KEPT, because it is the ONLY thing separating the two frames.
%   The polar flight-path angle is written GAM/57.3, the Cartesian one
%   1.5708-GAM/57.3, and 1.5708 - pi/2 = +3.673e-6.  So the Cartesian channel
%   launches 3.673e-6 rad shallower, independent of GAM and of the 57.3.  Speed
%   and radius match exactly (VRX^2+VRY^2 = V^2 identically) so both channels
%   have the same energy and period; the whole frame difference is that angle,
%   worth ~3.7e-6*tan(gam) in angular momentum -- which is what BENCHMARK 5
%   measures, and why its threshold is 1e-4 and not 1e-12.  Both literals are
%   exposed ('DEG', 'HALFPI') but default to the book's.
%
%   WHAT IT SHOULD PRINT: CLOSED-FORM two-body predictions from the launch
%   condition, NOT observations -- MATLAB has not been run against this file.
%   At V = 24000 ft/s, gam = 45 deg from the surface the orbit is elliptical
%   with a1 = 1.830e7 ft, e = 0.714, giving peak altitude near 1718 n.mi.,
%   impact near 4425 n.mi. and flight time near 2724 s.  The book rounds the
%   first two to 1700 and "nearly 4500" and never publishes the third.
%   SELFCHECK 4 makes that comparison inside the file, measured not asserted.
%
%   SELFCHECKS  1 Cartesian r^2*thetadot drift | 2 Cartesian energy drift
%               3 altitude read-back (catches the /6076 erratum) | 4 closed
%               form | 5 DELETE EXPERIMENT: +1% gm in the CARTESIAN channel
%               only must break frame agreement, or BENCHMARKS 1-2 are vacuous.
%   NOT CHECKED (costs a second full flight): ANGDEG only rotates the Cartesian
%   picture and must leave every reported number alone.
%
%   See also MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 11.2 line 1, verbatim.  Feet, ft/s, seconds, degrees.
    opts.V      (1,1) double {mustBeFinite, mustBePositive}    = 24000
    opts.GAM    (1,1) double {mustBeFinite, mustBeReal}        = 45
    opts.ANGDEG (1,1) double {mustBeFinite, mustBeReal}        = 0
    opts.ALTNM  (1,1) double {mustBeFinite, mustBeNonnegative} = 0
    opts.H      (1,1) double {mustBeFinite, mustBePositive}    = 0.01
    opts.samp   (1,1) double {mustBeFinite, mustBePositive}    = 10   % S>=9.99999
    opts.A      (1,1) double {mustBeFinite, mustBePositive}    = 2.0926e7
    opts.GM     (1,1) double {mustBeFinite, mustBePositive}    = 1.4077e16
    opts.NMFT   (1,1) double {mustBeFinite, mustBePositive}    = 6076
    % The book's two rounded constants.  See the artefact note in the header.
    opts.DEG    (1,1) double {mustBeFinite, mustBePositive}    = 57.3
    opts.HALFPI (1,1) double {mustBeFinite, mustBePositive}    = 1.5708
    % The listing's while-loop has no time bound; a launch above escape speed
    % never returns and would hang it.  Guarded analytically AND by this cap.
    opts.tmax   (1,1) double {mustBeFinite, mustBePositive}    = 20000
    % Frame-agreement threshold, n.mi.  NOT tuned to our answer: Fig. 11.9 is
    % published 500 px over ~4500 n.mi. of downrange and 308 px over ~1800 of
    % altitude = 9.0 and 5.8 n.mi. per pixel, so 0.5 n.mi. is a NINETEENTH of
    % one pixel -- the strongest reading of "identical" the figure can carry.
    opts.frame_tol_nm (1,1) double {mustBeFinite, mustBePositive} = 0.5
    % SELFCHECK 5, the delete experiment: gm is scaled by (1+probe_gm) in the
    % CARTESIAN channel only, over a short probe flight.
    opts.probe_s  (1,1) double {mustBeFinite, mustBePositive} = 400
    opts.probe_gm (1,1) double {mustBeFinite, mustBePositive} = 0.01
    % Not nargin: with only name-value options MATLAB reports nargin = 0
    % however many were passed, so `if nargin == 0` would never fire.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

if opts.samp < opts.H
    error('mgs:zarchan:gravityFrames:samp', ...
        'samp (%g s) is below the step H (%g s); the record would be empty.', ...
        opts.samp, opts.H);
end

gam  = opts.GAM/opts.DEG;
ang  = opts.ANGDEG/opts.DEG;
alt0 = opts.ALTNM*opts.NMFT;          % ERRATUM FIX -- see header
r0   = opts.A + alt0;

% ---- closed-form two-body prediction, used by SELFCHECK 4 and as the guard
%      against a launch that never comes back.
eps0 = 0.5*opts.V^2 - opts.GM/r0;
if eps0 >= 0
    error('mgs:zarchan:gravityFrames:escape', ...
        ['V = %g ft/s is at or above escape speed %g ft/s at r = %g ft, so ' ...
         'the ground-stop loop never terminates.'], ...
        opts.V, sqrt(2*opts.GM/r0), r0);
end
a1  = -opts.GM/(2*eps0);
h0  = r0*opts.V*cos(gam);              % book: r^2*psidot = (a+alt)*V*cos(gam)
pSL = h0^2/opts.GM;
ecc = sqrt(max(0, 1 - pSL/a1));
if a1*(1-ecc) >= r0
    error('mgs:zarchan:gravityFrames:noreturn', ...
        ['perigee radius %g ft is not below the launch radius %g ft, so the ' ...
         'trajectory never returns to the launch altitude.'], a1*(1-ecc), r0);
end
nu0 = acos(min(1, max(-1, (pSL/r0 - 1)/ecc)));   % ascending branch: rdot > 0
CF  = struct();
CF.a1_ft      = a1;
CF.ecc        = ecc;
CF.alt_apo_nm = (a1*(1+ecc) - opts.A)/opts.NMFT;
CF.range_nm   = opts.A*2*(pi - nu0)/opts.NMFT;
CF.period_s   = 2*pi*sqrt(a1^3/opts.GM);
E0            = 2*atan2(sqrt(1-ecc)*sin(nu0/2), sqrt(1+ecc)*cos(nu0/2));
CF.tof_s      = CF.period_s*(1 - (E0 - ecc*sin(E0))/pi);

% ---- the flight, both frames, one loop.
F = flyBoth(opts.A, opts.GM, 1.0, opts.V, gam, ang, alt0, opts.H, opts.samp, ...
            opts.NMFT, opts.HALFPI, opts.tmax, true);

R = struct();
R.t          = F.T;
R.polar      = struct('s_nm', F.SPOL, 'alt_nm', F.APOL);
R.cart       = struct('s_nm', F.DIST, 'alt_nm', F.ALT);
R.d_alt_nm   = F.APOL - F.ALT;
R.d_s_nm     = F.SPOL - F.DIST;
R.h_cart     = F.HC;
R.eps_cart   = F.EC;
R.max_d_alt_nm = max(abs(R.d_alt_nm));
R.max_d_s_nm   = max(abs(R.d_s_nm));
R.peak_alt_nm  = max(F.ALT);
R.capped       = F.capped;

% impact by linear interpolation between the last above-ground sample and the
% first below-ground one -- the pair the listing's while-loop stops on.
kk = find(F.ALT(1:end-1) >= 0 & F.ALT(2:end) < 0, 1, 'last');
if isempty(kk)
    R.impact_t_s = NaN;  R.impact_dist_nm = NaN;
else
    f = F.ALT(kk)/(F.ALT(kk) - F.ALT(kk+1));
    R.impact_t_s     = F.T(kk)    + f*(F.T(kk+1)    - F.T(kk));
    R.impact_dist_nm = F.DIST(kk) + f*(F.DIST(kk+1) - F.DIST(kk));
end
R.last_dist_nm = F.DIST(end);
R.cf = CF;
R.p  = struct('V',opts.V, 'GAM',opts.GAM, 'ANGDEG',opts.ANGDEG, ...
              'ALTNM',opts.ALTNM, 'H',opts.H, 'samp',opts.samp, ...
              'A',opts.A, 'GM',opts.GM, 'NMFT',opts.NMFT, 'DEG',opts.DEG, ...
              'HALFPI',opts.HALFPI, 'h0',h0, 'n_steps',F.nstep, ...
              'n_samples',numel(F.T), 'frame_tol_nm',opts.frame_tol_nm, ...
              'probe_s',opts.probe_s, 'probe_gm',opts.probe_gm, ...
              'units','ft, ft/s; distances reported in n.mi. at 6076 ft/n.mi.');
R.units = R.p.units;

% ---- SELFCHECK 5's delete experiment: same launch, gm scaled in the CARTESIAN
%      channel only, short probe.  Compared against the same window of the
%      real flight, so the ratio is like for like.
P = flyBoth(opts.A, opts.GM, 1 + opts.probe_gm, opts.V, gam, ang, alt0, ...
            opts.H, opts.samp, opts.NMFT, opts.HALFPI, opts.probe_s, false);
R.probe_resid_nm = max(abs(P.APOL - P.ALT));
win = F.T <= opts.probe_s + 0.5*opts.H;
R.base_resid_nm  = max(abs(R.d_alt_nm(win)));

R.selfcheck = structuralChecks(R, CF, opts.ALTNM, opts.frame_tol_nm);

if opts.benchmark
    R.bench = benchmarks(R, opts, h0);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    R.pass  = false;   % NOT checked is not the same thing as fine
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R);           end

msg = '';
if R.capped
    msg = sprintf(['the flight hit the tmax cap of %g s without returning ' ...
        'to the ground; every trajectory number below is truncated.'], opts.tmax);
elseif ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = 'not one book value could be evaluated.';
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
        error('mgs:zarchan:gravityFrames:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN, complete: a port that errors here hands back
        % nothing and nobody can see which side of the comparison moved.
        warning('mgs:zarchan:gravityFrames:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function F = flyBoth(A, GM, gmScaleCart, V, gam, ang, alt0, H, samp, NMFT, ...
                     halfpi, tstop, stopGround)
%FLYBOTH  Listing 11.2's outer loop: both frames, one Heun step each, per
%   iteration.  Every constant arrives as an ARGUMENT -- MATLAB local functions
%   cannot see the caller's workspace, and a silently-zero GM here would still
%   draw a plausible straight line.  gmScaleCart scales gm in the CARTESIAN
%   channel only (1.0 for every real flight; the delete experiment uses it);
%   stopGround false runs to tstop instead of to impact.
GMC = GM*gmScaleCart;
h0  = (A + alt0)*V*cos(gam);           % the polar channel's exact first integral

R0 = A + alt0;   R1 = V*sin(gam);   PSI = 0;
X  = (A + alt0)*cos(ang);   Y = (A + alt0)*sin(ang);
XFIRST = X;  YFIRST = Y;  RF = sqrt(XFIRST^2 + YFIRST^2);
X1 = V*cos(halfpi - gam + ang);
Y1 = V*sin(halfpi - gam + ang);
T = 0;  S = 0;  nstep = 0;  capped = false;

nmax = ceil(tstop/samp) + 4;
T_ = zeros(1,nmax); SP = T_; AP = T_; DI = T_; AL = T_; HC = T_; EC = T_;
k  = 0;  ALTNM = 0;

while true
    if k == 0 || S >= samp - 1e-5
        S = 0;  k = k + 1;
        rr    = sqrt(X*X + Y*Y);
        T_(k) = T;
        SP(k) = A*PSI/NMFT;                  % polar downrange, unwrapped
        AP(k) = (R0 - A)/NMFT;
        AL(k) = (rr - A)/NMFT;
        cb    = (X*XFIRST + Y*YFIRST)/(rr*RF);
        DI(k) = A*acos(min(1, max(-1, cb)))/NMFT;   % clamp: see header
        HC(k) = X*Y1 - Y*X1;                 % Cartesian r^2*thetadot
        EC(k) = 0.5*(X1*X1 + Y1*Y1) - GM/rr; % true-GM energy, probe excepted
        ALTNM = AL(k);
    end
    % The listing tests the LAST RECORDED altitude, not the current one; kept.
    if stopGround && ~(ALTNM > -1e-4), break; end
    if T >= tstop - 0.5*H, capped = stopGround; break; end

    % ---- Heun stage 1, at the current state
    PSID   = h0/(R0*R0);
    R1D    = -GM/(R0*R0) + R0*PSID*PSID;
    R0D    = R1;
    TEMBOT = (X*X + Y*Y)^1.5;
    X1D = -GMC*X/TEMBOT;  Y1D = -GMC*Y/TEMBOT;  XD = X1;  YD = Y1;

    R0o=R0; R1o=R1; PSIo=PSI; Xo=X; Yo=Y; X1o=X1; Y1o=Y1;
    R0 = R0 + H*R0D;   R1 = R1 + H*R1D;   PSI = PSI + H*PSID;
    X  = X  + H*XD;    Y  = Y  + H*YD;
    X1 = X1 + H*X1D;   Y1 = Y1 + H*Y1D;
    T  = T + H;

    % ---- Heun stage 2, at the predictor
    PSID   = h0/(R0*R0);
    R1D    = -GM/(R0*R0) + R0*PSID*PSID;
    R0D    = R1;
    TEMBOT = (X*X + Y*Y)^1.5;
    X1D = -GMC*X/TEMBOT;  Y1D = -GMC*Y/TEMBOT;  XD = X1;  YD = Y1;

    R0 = (R0o+R0)/2 + .5*H*R0D;    R1 = (R1o+R1)/2 + .5*H*R1D;
    PSI= (PSIo+PSI)/2 + .5*H*PSID;
    X  = (Xo+X)/2   + .5*H*XD;     Y  = (Yo+Y)/2   + .5*H*YD;
    X1 = (X1o+X1)/2 + .5*H*X1D;    Y1 = (Y1o+Y1)/2 + .5*H*Y1D;

    S = S + H;  nstep = nstep + 1;
end

F = struct('T',T_(1:k), 'SPOL',SP(1:k), 'APOL',AP(1:k), 'DIST',DI(1:k), ...
           'ALT',AL(1:k), 'HC',HC(1:k), 'EC',EC(1:k), ...
           'nstep',nstep, 'capped',capped);
end


% =============================================================================
function K = structuralChecks(R, CF, ALTNM_req, frameTol)
%STRUCTURALCHECKS  Self-consistency, NOT book values.  .names and .oks are the
%   two fields benchSuite looks for; without BOTH, none of this reaches the
%   board and a delete experiment nobody can see is the same as not running it.
names = {};  oks = [];  detail = {};

K = struct();

% 1  the Cartesian channel has to EARN angular momentum; the polar one is
%    handed it.  Estimated drift for Heun at H = 0.01 over ~2700 s is ~1e-9
%    relative, so 1e-6 is three decades of headroom and still six decades
%    below any wiring fault, which is O(1).
K.h_drift  = max(abs(R.h_cart - R.h_cart(1)))/abs(R.h_cart(1));
names{end+1} = 'Cartesian r^2*thetadot conserved (rel drift < 1e-6)';
oks(end+1)   = K.h_drift < 1e-6;
detail{end+1}= sprintf('%.3g', K.h_drift);

% 2  same argument for specific energy.
K.eps_drift = max(abs(R.eps_cart - R.eps_cart(1)))/abs(R.eps_cart(1));
names{end+1} = 'Cartesian specific energy conserved (rel drift < 1e-6)';
oks(end+1)   = K.eps_drift < 1e-6;
detail{end+1}= sprintf('%.3g', K.eps_drift);

% 3  altitude read-back.  Vacuous at the published ALTNM = 0 and NOT vacuous
%    for anyone else: the book's ALT=ALTNM/6076 would put a requested 1000
%    n.mi. launch at 2.7e-5 n.mi. and nothing else would complain.
K.alt0_readback_nm = R.cart.alt_nm(1);
names{end+1} = 'initial altitude reads back as ALTNM (the /6076 erratum)';
oks(end+1)   = abs(K.alt0_readback_nm - ALTNM_req) <= max(1e-6, 1e-9*ALTNM_req);
detail{end+1}= sprintf('%.6g vs %.6g n.mi.', K.alt0_readback_nm, ALTNM_req);

% 4  against the closed-form two-body solution of the same launch condition.
%    0.1% absorbs the 10 s sampling of apogee (~0.001%) and of impact
%    (~0.003%) with two decades to spare.
K.cf_apo_rel = abs(R.peak_alt_nm/CF.alt_apo_nm - 1);
K.cf_rng_rel = abs(R.impact_dist_nm/CF.range_nm - 1);
K.cf_tof_rel = abs(R.impact_t_s/CF.tof_s - 1);
names{end+1} = 'apogee, range and time of flight match the two-body closed form (0.1%)';
oks(end+1)   = all([K.cf_apo_rel, K.cf_rng_rel, K.cf_tof_rel] < 1e-3);
detail{end+1}= sprintf('%.3g / %.3g / %.3g', K.cf_apo_rel, K.cf_rng_rel, K.cf_tof_rel);

% 5  DELETE EXPERIMENT.  If the two channels were the same numbers copied
%    twice, or if the residual were computed against itself, BENCHMARKS 1-2
%    would pass no matter what.  Break the Cartesian gravity by 1% and the
%    residual must move by more than an order of magnitude AND past the
%    threshold those rows are scored at.
K.probe_resid_nm = R.probe_resid_nm;
K.base_resid_nm  = R.base_resid_nm;
K.probe_ratio    = R.probe_resid_nm/max(R.base_resid_nm, 1e-6);
names{end+1} = 'DELETE: +1% gm in the Cartesian channel only breaks frame agreement';
oks(end+1)   = K.probe_ratio >= 20 && K.probe_resid_nm >= frameTol;
detail{end+1}= sprintf('%.4g vs %.4g n.mi. over the probe (x%.4g)', ...
                       K.probe_resid_nm, K.base_resid_nm, K.probe_ratio);

K.names  = names;
K.oks    = logical(oks);
K.detail = detail;
K.all_ok = all(K.oks);
K.first_bad = '';
if ~K.all_ok
    j = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{j}, detail{j});
end
end


% =============================================================================
function B = benchmarks(R, opts, h0)
B = emptyBench();
srcClaim = 'text after Listing 11.2, p.241 -- zarchan.txt:3339 (Fig. 11.9, :3342)';

% 1-2  THE row this listing exists for.  Not a figure reading: the book's
%      sentence is a comparison of two computed curves and we recompute both.
B = addShape(B, 'polar vs Cartesian altitude identical', ...
    sprintf('"identical for all practical purposes" (<= %.2f n.mi.)', opts.frame_tol_nm), ...
    sprintf('max |d| = %.4g n.mi. over %d samples', R.max_d_alt_nm, numel(R.t)), ...
    R.max_d_alt_nm <= opts.frame_tol_nm, srcClaim, ...
    ['threshold is a nineteenth of one pixel of the published figure; the ' ...
     'only launch-condition difference between the channels is the 1.5708 ' ...
     'literal, worth 3.673e-6 rad of flight-path angle']);

B = addShape(B, 'polar vs Cartesian downrange identical', ...
    sprintf('"identical for all practical purposes" (<= %.2f n.mi.)', opts.frame_tol_nm), ...
    sprintf('max |d| = %.4g n.mi. over %d samples', R.max_d_s_nm, numel(R.t)), ...
    R.max_d_s_nm <= opts.frame_tol_nm, srcClaim, ...
    ['A*PSI (unwrapped) against A*acos(CBETA); these are two different ' ...
     'definitions of the same swept angle and the acos one folds back above ' ...
     '180 deg, so a very long flight would redden this row correctly']);

% 3  the only ordinary number the chapter publishes about this run.
%    tol 3% is the round-to-the-nearest-hundred band the book's own "1700"
%    implies (+/-50 on 1700), not a band chosen to fit an answer.
B = addVal(B, 'peak altitude', 1700, R.peak_alt_nm, '+/-3% (book rounds to 100)', ...
    0.03, srcClaim, 'n.mi.', ...
    sprintf(['closed-form apogee for this launch is %.1f n.mi.; sampled ' ...
             'every %g s, worth about 0.02 n.mi. of loss at the peak'], ...
             R.cf.alt_apo_nm, opts.samp));

% 4  "nearly 4500" is an approach-from-below claim, so it is scored as a
%    BOUND.  Turning it into a value with book = 4500 and a wide tolerance
%    would be inventing a precision the sentence does not have.
B = addBound(B, 'total distance travelled', '"nearly 4500 n.mi." => 4000 < d < 4500', ...
    sprintf('%.1f n.mi. at impact (last sample %.1f)', R.impact_dist_nm, R.last_dist_nm), ...
    R.impact_dist_nm > 4000 && R.impact_dist_nm < 4500, srcClaim, 'n.mi.', ...
    ['interpolated to alt = 0 between the last two samples; the listing''s ' ...
     'own last record is up to one sample interval past impact, about 28 ' ...
     'n.mi. at this ground speed, which is why the raw value is not scored']);

% 5  the published first integral, tested in the channel that does NOT
%    enforce it.  Expected residual is the 1.5708 artefact, ~3.7e-6*tan(gam);
%    1e-4 is 27x that at gam = 45 deg and still catches anything structural.
hrel = abs(R.h_cart(1)/h0 - 1);
B = addShape(B, 'Cartesian channel realises r^2*thetadot = (a+alt)*V*cos(gam)', ...
    'published first integral, zarchan.txt:3314 and :3318 (rel < 1e-4)', ...
    sprintf('%.4g (h_cart = %.6g, book constant = %.6g ft^2/s)', hrel, R.h_cart(1), h0), ...
    hrel < 1e-4, 'polar derivation, p.239 -- zarchan.txt:3314, restated :3318', ...
    'residual here is the 1.5708-vs-pi/2 artefact, ~3.673e-6*tan(gam)');

% 6  the derivation's initial conditions against the listing's.  Cheap, and
%    it is the only cross-check that the code on p.240 implements the algebra
%    on p.239 rather than something adjacent to it.
ic_ok = abs(R.polar.alt_nm(1) - opts.ALTNM) <= 1e-9*max(1,opts.ALTNM) && ...
        R.polar.s_nm(1) == 0;
B = addShape(B, 'polar initial conditions r(0)=a+alt, theta(0)=0, rdot(0)=V*sin(gam)', ...
    'r(0) = a+alt, theta(0) = 0, rdot(0) = V sin gam', ...
    sprintf('R0-A = %.6g n.mi., PSI(0) = %.6g, R1(0) = %.6f ft/s', ...
            R.polar.alt_nm(1), R.polar.s_nm(1), opts.V*sin(opts.GAM/opts.DEG)), ...
    ic_ok, 'polar derivation, p.239 -- zarchan.txt:3320-3322', ...
    'listing line 1 sets R0=A+ALT, PSI=0., R1=V*sin(GAM/57.3) -- all three agree');

% 7  the experiment's own settings, reported so a caller who changed them
%    cannot mistake the rows above for the book's case.
B = addInfo(B, 'launch condition', ...
    'V = 24,000 ft/s, 45 deg from the reference, from the surface', ...
    sprintf('V = %g ft/s, GAM = %g deg, ALTNM = %g, ANGDEG = %g', ...
            opts.V, opts.GAM, opts.ALTNM, opts.ANGDEG), srcClaim, ...
    'reported, never scored');

% 8  NOT a skip: a skip is a check that could not run.  There is no book
%    claim about flight time at all, so there is nothing to score against.
B = addInfo(B, 'flight time', 'not published anywhere in Ch.11', ...
    sprintf('%.1f s (closed form %.1f s, orbital period %.1f s)', ...
            R.impact_t_s, R.cf.tof_s, R.cf.period_s), ...
    'searched zarchan.txt:3324-3345; the chapter gives altitude and distance only', ...
    'reported, never scored');
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want.  Schema is the
%   one mgs.zarchan.benchSuite's shared benchRows() consumes; adaptGeneric is
%   strict and a near-miss name errors rather than being accommodated.
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
%ADDINFO  skipped = true keeps it out of the pass count in BOTH directions --
%   an INFO row can neither pass nor fail.
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
p = R.p;
L{end+1} = 'mgs.zarchan.gravityFrames -- Zarchan Listing 11.2 (zarchan.txt:3326-3337)';
L{end+1} = sprintf('  units: %s', p.units);
L{end+1} = sprintf(['  launch  V = %g ft/s  gam = %g deg  alt = %g n.mi.  ' ...
                    'ang = %g deg   H = %g s, sample %g s'], ...
                   p.V, p.GAM, p.ALTNM, p.ANGDEG, p.H, p.samp);
L{end+1} = sprintf('  ran %d steps, %d samples', p.n_steps, p.n_samples);
L{end+1} = sprintf(['  trajectory   peak alt %.2f n.mi.   impact %.2f n.mi. ' ...
                    'at t = %.1f s'], R.peak_alt_nm, R.impact_dist_nm, R.impact_t_s);
L{end+1} = sprintf(['  closed form  peak alt %.2f n.mi.   range  %.2f n.mi. ' ...
                    'at t = %.1f s  (a1 = %.4g ft, e = %.5f)'], ...
                   R.cf.alt_apo_nm, R.cf.range_nm, R.cf.tof_s, R.cf.a1_ft, R.cf.ecc);
L{end+1} = sprintf('  frame residual  altitude %.4g n.mi.   downrange %.4g n.mi.', ...
                   R.max_d_alt_nm, R.max_d_s_nm);
L{end+1} = '  --- structural ---';
for i = 1:numel(R.selfcheck.names)
    L{end+1} = sprintf('   %s %-64s %s', tick(R.selfcheck.oks(i)), ...
                       R.selfcheck.names{i}, R.selfcheck.detail{i}); %#ok<AGROW>
end
L{end+1} = '  --- benchmarks ---';
for i = 1:numel(R.bench)
    b = R.bench(i);
    if b.skipped
        mark = ' -';
    else
        mark = tick(b.ok);
    end
    L{end+1} = sprintf('   %s %-52s book %-46s ours %s', mark, b.what, ...
                       b.book_str, b.ours_str); %#ok<AGROW>
end
if any(ev)
    L{end+1} = sprintf('  %d of %d scoreable rows pass; %d structural of %d.', ...
        sum([R.bench(ev).ok]), sum(ev), sum(R.selfcheck.oks), numel(R.selfcheck.oks));
end
s = strjoin(L, newline);
end

function c = tick(ok)
if ok, c = 'ok  '; else, c = 'FAIL'; end
end

function drawCurves(R)
figure;
plot(R.polar.s_nm, R.polar.alt_nm, '-', R.cart.s_nm, R.cart.alt_nm, '--');
grid on;  xlabel('Downrange (Nmi)');  ylabel('Altitude (Nmi)');
legend('polar', 'Earth-centred Cartesian', 'Location', 'best');
title('Zarchan Fig. 11.9 -- polar and Cartesian gravity field equations');
end
