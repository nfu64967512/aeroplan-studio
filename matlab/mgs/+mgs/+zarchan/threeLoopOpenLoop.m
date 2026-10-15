function R = threeLoopOpenLoop(opts)
%MGS.ZARCHAN.THREELOOPOPENLOOP  Zarchan Listing 23.2 -- OPEN-LOOP RESPONSE OF
%   THE THREE-LOOP AUTOPILOT.  Bode magnitude and phase, transcribed.
%
%   R = mgs.zarchan.threeLoopOpenLoop()             % book run + benchmark board
%   R = mgs.zarchan.threeLoopOpenLoop('wcr', 100)   % Fig. 23.7
%   R = mgs.zarchan.threeLoopOpenLoop('zeta', 0.4)  % Fig. 23.10
%   R = mgs.zarchan.threeLoopOpenLoop('plot', true) % the listing's two figures
%
%   *** NAME.  The task sheet asked for entry point `threeLoopOpen` in file
%   `threeLoopOpenLoop.m`.  Both cannot hold: MATLAB dispatches a package
%   function on its FILE name and ignores a mismatched declaration, so
%   `mgs.zarchan.threeLoopOpen` would be a dead name.  The file name won (all
%   23 neighbours match theirs).  If the short name is wanted, add a one-line
%   forwarder threeLoopOpen.m -- do NOT rename the declaration back.
%
%   The loop of Fig. 23.2 is broken just ahead of the actuator, and what is
%   left is (zarchan.txt:7552-7607, pp.540-541)
%
%                       K0 (1 + 2 z0 s/w0 + s^2/w0^2)
%       HG(s) = ----------------------------------------------------------
%               s (1 + 2 zAF s/wAF + s^2/wAF^2)(1 + 2 zACT s/wACT + s^2/wACT^2)
%
%   evaluated at s = jw.  No differential equations: the listing walks 159
%   frequencies and takes a magnitude and an angle at each, which is why it
%   runs instantly compared with every other program in this chapter.
%
% ===========================================================================
%   *** UNITS ARE THE BOOK'S.  rad/s, dB, DEGREES. ***
%   Upstream of the transfer function the aero is feet, ft/s, g = 32.2 ft/s^2
%   and DEGREES, exactly as in mgs.zarchan.threeLoopAutopilot.  Nothing here
%   is SI.  Convert at the CALL SITE, visibly.
% ===========================================================================
%
%   WHAT WAS REUSED
%
%   Listing 23.2's first ~60 lines are BYTE-IDENTICAL to Listing 23.1's --
%   geometry, trim solve, linearized aero, gain algorithm through XKDC.  This
%   file re-runs NONE of it.  GAINSET() calls
%
%       mgs.zarchan.threeLoopAutopilot('benchmark', false, 'crosscheck', false)
%
%   and takes w0, z0, wAF, zAF, K0 off R.fc(1).  A second transcription of
%   that preamble is the EXPENSIVE option: two copies drift and no static
%   check can see it happen.  All this file adds is the `for I=2:160` sweep,
%   and that is the only physics written here.
%
%   Price, stated: the neighbour integrates a 1-second step response on the
%   way to those gains and Listing 23.2 has no integration at all, so that
%   response is discarded (with its two advisory warnings, see GAINSET).
%   wact/zact are deliberately NOT forwarded -- they never enter the gain
%   algorithm, only the actuator factor of HG, and forwarding them would
%   expose this file to a neighbour guard (wact*h < 0.5) about integration.
%
%   CAN SAY: gain and phase crossover frequencies and the two margins of the
%   FLIGHT-CONTROL loop, versus the design goals tau / wcr / zeta and the
%   actuator.  Properties of a linear transfer function, exact.
%   CANNOT SAY: anything about a guidance loop -- Ch.23's open-loop analysis
%   has no seeker, target, noise, radome or miss distance, so a margin here is
%   not a robustness claim about a sortie.  Nor about our vehicle: ArduPilot's
%   pitch loop is not this loop.
%
%   TRANSCRIPTION NOTES (listing_23_02.m, zarchan.txt:7625-7633)
%   * The listing's residual-damage list is empty; its one unresolved item,
%     `if ALT <= 30000.`, is already resolved inside the neighbour.
%   * `for I=2:160`, W = 10^(0.025*I-1).  I=1 IS SKIPPED, so the sweep starts
%     at 10^-0.95 = 0.1122 rad/s, not the 0.1 the plot axis shows.  Kept: 159
%     points, 0.025 decade apart, ending exactly at 1000.
%   * 57.3, not 180/pi.  The book's constant.  It biases phase by 0.007%.
%   * XNCG=10 survives only as the trim point the aero is linearized about;
%     nothing is commanded, because nothing integrates.
%   * XNLLIN, XACC, XK2, DELTR, TF are dead here.  Not carried.  Neither are
%     `save datfil.txt` and `disp 'simulation finished'`; R holds the arrays.
%
%   OUTPUT
%     R.w R.gain_db R.phase_deg   the listing's 159-point sweep (ArrayW etc.)
%     R.margins                   .wc_rad_s .pm_deg .w180_rad_s .gm_db
%     R.gains                     the reused gain set from Listing 23.1
%     R.book                      the four book configurations and their margins
%     R.bench .selfcheck .pass .summary .cfg .offbook
%
%   THE BOARD DOES NOT SCORE YOUR RUN.  R.bench re-runs the FOUR configurations
%   the chapter publishes numbers for and scores those, so an off-book call
%   still reports whether the port is intact and never reports a pass for
%   parameters the book never covered.
%
%   See also MGS.ZARCHAN.THREELOOPAUTOPILOT.

arguments
    opts.alt   double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % ft
    opts.tau   double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % s
    opts.wcr   double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % rad/s
    opts.zeta  double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % -
    opts.wact  double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % rad/s
    opts.zact  double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % -
    opts.vm    double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % ft/s
    opts.wgt   double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % lb
    opts.xncg  double {mustBeReal, mustBeFinite, mustBeScalarOrEmpty} = []  % g
    opts.benchmark (1,1) logical = true   % THE ONLY MODE SWITCH.  Not nargin:
    opts.render    (1,1) logical = true   % nargin is 0 here however many
    opts.plot      (1,1) logical = false  % name-value pairs the caller passes.
    opts.assert    (1,1) logical = true
end

BOOK = bookNominal();
[cfg, offbook] = resolveConfig(opts, BOOK);

R = struct();
R.units   = 'rad/s, dB, deg (aero upstream: ft, ft/s, deg, g = 32.2 ft/s^2)';
R.listing = 'Zarchan Listing 23.2 (zarchan.txt:7624), open-loop Bode of the three-loop autopilot';
R.cfg     = cfg;
R.offbook = offbook;

% ---- the caller's run ----------------------------------------------------
F = gainSet(cfg.alt, cfg.tau, cfg.wcr, cfg.zeta, cfg.vm, cfg.wgt, cfg.xncg);
R.gains = F;
[R.w, R.gain_db, R.phase_deg] = bookSweep(F, cfg.wact, cfg.zact);
R.margins = marginsOf(F, cfg.wact, cfg.zact);

% ---- structural invariants (always; they cost nothing) -------------------
R.selfcheck = selfChecks(R.w, R.gain_db, R.phase_deg, F, cfg.wact, cfg.zact, R.margins);
if ~all(R.selfcheck.oks)
    bad = R.selfcheck.names(~R.selfcheck.oks);
    if cfg.assert
        error('mgs:zarchan:threeLoopOpenLoop:selfcheck', ...
            'structural self-check failed: %s', strjoin(bad, '; '));
    else
        % assert = false must still HAND BACK the result, or nobody can
        % diagnose which invariant broke.
        warning('mgs:zarchan:threeLoopOpenLoop:selfcheck', ...
            'structural self-check failed (assert = false, returning anyway): %s', ...
            strjoin(bad, '; '));
    end
end

% ---- the book's own four configurations, and the board -------------------
if cfg.benchmark
    R.book  = bookCases(BOOK);
    R.bench = benchmarks(R.book);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]);
else
    R.book  = [];
    R.bench = emptyBench();
    ev      = false;
    R.pass  = false;   % nothing was checked; that is NOT a pass.
end

R.summary = render(R, cfg, offbook, ev);
if cfg.render, fprintf('%s\n', R.summary); end
if cfg.plot,   drawCurves(R); end

if cfg.assert && cfg.benchmark
    if ~any(ev)
        error('mgs:zarchan:threeLoopOpenLoop:noBenchmark', ...
            ['not one book value could be evaluated -- every margin came ' ...
             'back NaN.  Refusing to hand back an unvalidated port.']);
    end
    if ~R.pass
        b = find(~[R.bench.ok] & ev, 1);
        error('mgs:zarchan:threeLoopOpenLoop:benchmark', ...
            ['%d of %d evaluated book values are outside tolerance (first: ' ...
             '%s, book %s, ours %s).  Full board:\n\n%s'], ...
            sum(~[R.bench.ok] & ev), sum(ev), R.bench(b).what, ...
            R.bench(b).book_str, R.bench(b).ours_str, R.summary);
    end
end
end


% =============================================================================
function N = bookNominal()
%BOOKNOMINAL  The data statement on the first line of Listing 23.2
%   (zarchan.txt:7625).  Identical to Listing 23.1's minus TF, which 23.2 has
%   no use for.  There is no second copy of these constants in this file.
N = struct('alt', 0, 'tau', 0.3, 'wcr', 50, 'zeta', 0.7, ...
           'wact', 150, 'zact', 0.7, 'vm', 3000, 'wgt', 1000, 'xncg', 10);
end


function [cfg, offbook] = resolveConfig(opts, N)
%RESOLVECONFIG  Empty means "use the book value"; non-empty means the caller
%   chose it.  offbook lists the names actually moved.
cfg = struct(); offbook = {};
f = fieldnames(N);
for k = 1:numel(f)
    nm = f{k};
    if isempty(opts.(nm))
        cfg.(nm) = N.(nm);
    else
        cfg.(nm) = opts.(nm);
        if cfg.(nm) ~= N.(nm), offbook{end+1} = nm; end %#ok<AGROW>
    end
end
cfg.benchmark = opts.benchmark; cfg.render = opts.render;
cfg.plot      = opts.plot;      cfg.assert = opts.assert;

if cfg.wact <= 0 || cfg.zact <= 0
    error('mgs:zarchan:threeLoopOpenLoop:actuator', ...
        ['the actuator factor is 1 + 2*zact*s/wact + s^2/wact^2; wact = %g ' ...
         'and zact = %g must both be positive or it is not a lag.'], ...
        cfg.wact, cfg.zact);
end
end


% =============================================================================
function F = gainSet(ALT, TAU, WCR, ZETA, VM, WGT, XNCG)
%GAINSET  THE REUSE.  Listing 23.2's aero-and-gain preamble is Listing 23.1's,
%   fetched rather than retyped.  Five numbers are needed: w0, z0, wAF, zAF, K0.
%
%   The two silenced warnings describe the neighbour's STEP RESPONSE, which is
%   computed on the way to the gains and thrown away here; the off-nominal
%   board cases (wcr = 100, zeta = 0.4) trip them, and a warning about a result
%   nobody reads is worse than none.  The suppression does not outlive this
%   call -- checked in a live session, not assumed.  Note when checking it
%   yourself: warning('off', id) returns the EFFECTIVE previous state, so
%   warning(ws) faithfully restores 'off' in a session that had warnings off
%   globally.  That is a correct restore, not a leak.
ws    = warning('off', 'mgs:zarchan:threeLoopAutopilot:noT63');
ws(2) = warning('off', 'mgs:zarchan:threeLoopAutopilot:notSettled');
% Not unused: onCleanup also covers the error paths below and an error thrown
% inside threeLoopAutopilot, which a trailing warning(ws) would not.
restore = onCleanup(@() warning(ws));

A = mgs.zarchan.threeLoopAutopilot('alt', ALT, 'tau', TAU, 'wcr', WCR, ...
        'zeta', ZETA, 'vm', VM, 'wgt', WGT, 'xncg', XNCG, ...
        'benchmark', false, 'crosscheck', false, 'render', false, ...
        'plot', false, 'assert', false);
F = A.fc(1);

need = {'w0', 'z0', 'waf', 'zaf', 'xk0'};
miss = need(~isfield(F, need));
if ~isempty(miss)
    error('mgs:zarchan:threeLoopOpenLoop:reuse', ...
        ['mgs.zarchan.threeLoopAutopilot no longer returns %s on R.fc.  This ' ...
         'file reuses its gain algorithm rather than duplicating it, so a ' ...
         'renamed field there breaks the Bode here.'], strjoin(miss, ', '));
end
if F.xk0 >= 0
    % XMAGTOP = -XK0*sqrt(...).  A non-negative K0 makes the magnitude zero or
    % negative and 20*log10 of it -Inf or COMPLEX, and MATLAB would carry the
    % complex numbers all the way to a plot that still looks like a Bode plot.
    error('mgs:zarchan:threeLoopOpenLoop:sign', ...
        ['K0 = %g is not negative.  Listing 23.2 writes the magnitude as ' ...
         '-K0*sqrt(...), which assumes K0 = -W^2/(tau*wAF^2) < 0.'], F.xk0);
end
end


% =============================================================================
function [w, gdb, pdeg] = bookSweep(F, WACT, ZACT)
%BOOKSWEEP  `for I=2:160, W=10^(.025*I-1)` -- the listing's grid, exactly.
I = 2:160;
w = 10.^(0.025*I - 1);
[gdb, pdeg] = hgBode(w, F.w0, F.z0, F.waf, F.zaf, F.xk0, WACT, ZACT);
end


function [gdb, pdeg] = hgBode(w, W0, Z0, WAF, ZAF, XK0, WACT, ZACT)
%HGBODE  The loop body of Listing 23.2, vectorized over w.  Every constant it
%   needs arrives as an argument: a MATLAB local function cannot see the main
%   function's workspace, and a port in an earlier wave shipped believing it
%   could.
XMAGTOP = -XK0.*sqrt((1 - (w./W0).^2).^2 + (2.*Z0.*w./W0).^2);
XMAGBOT =    w.*sqrt((1 - (w./WAF).^2).^2 + (2.*ZAF.*w./WAF).^2);
XMAG    = XMAGTOP./XMAGBOT;
XMAGACT = 1./sqrt((1 - w.*w./(WACT*WACT)).^2 + (2.*ZACT.*w./WACT).^2);

PHASETOP = atan2(2.*Z0.*w./W0,      1 - (w./W0).^2);
PHASEBOT = atan2(2.*ZAF.*w./WAF,    1 - (w./WAF).^2);
PHASEACT = atan2(2.*ZACT.*w./WACT,  1 - w.*w./(WACT*WACT));

gdb  = 20.*log10(XMAG.*XMAGACT);
pdeg = -90. + 57.3*(PHASETOP - PHASEBOT - PHASEACT);   % 57.3 is the book's
end


% =============================================================================
function M = marginsOf(F, WACT, ZACT)
%MARGINSOF  Gain/phase crossover frequencies and the two margins.
%
%   The listing's 0.025-decade grid is 5.9% coarse -- too coarse to read a
%   crossover off directly -- so the two crossings are refined by geometric
%   bisection on the SAME continuous formula the grid samples.  SELFCHECKS()
%   then verifies the refined answer sits inside the grid's own bracket, so
%   the refinement cannot quietly wander off the transcribed curve.
gfun = @(x) firstOut(x, F, WACT, ZACT, 1);
pfun = @(x) firstOut(x, F, WACT, ZACT, 2) + 180;

M = struct();
M.wc_rad_s   = crossLog(gfun, 1e-2, 1e4);          % |HG| = 0 dB
M.w180_rad_s = crossLog(pfun, 1e-2, 1e4);          % phase = -180 deg
if isfinite(M.wc_rad_s)
    M.pm_deg = 180 + firstOut(M.wc_rad_s, F, WACT, ZACT, 2);
else
    M.pm_deg = NaN;
end
if isfinite(M.w180_rad_s)
    M.gm_db = -firstOut(M.w180_rad_s, F, WACT, ZACT, 1);
else
    M.gm_db = NaN;
end
end


function v = firstOut(w, F, WACT, ZACT, sel)
%FIRSTOUT  Scalar accessor so MARGINSOF can build one-output handles.
%   sel = 1 -> gain in dB, sel = 2 -> phase in deg.  Named sel, not which:
%   `which` is a builtin and shadowing it inside a package is a latent trap.
[g, p] = hgBode(w, F.w0, F.z0, F.waf, F.zaf, F.xk0, WACT, ZACT);
if sel == 1, v = g; else, v = p; end
end


function w = crossLog(fun, wlo, whi)
%CROSSLOG  Geometric bisection for a sign change of FUN on [wlo, whi].
%   NaN, not an endpoint, when there is no bracketed crossing: an endpoint
%   returned here would be scored as a margin.
flo = fun(wlo); fhi = fun(whi);
if ~isfinite(flo) || ~isfinite(fhi) || flo*fhi > 0, w = NaN; return, end
for k = 1:100
    wm = sqrt(wlo*whi); fm = fun(wm);
    if ~isfinite(fm), w = NaN; return, end
    if flo*fm <= 0, whi = wm; else, wlo = wm; flo = fm; end
end
w = sqrt(wlo*whi);
end


% =============================================================================
function C = selfChecks(w, gdb, pdeg, F, WACT, ZACT, M)
%SELFCHECKS  Four structural invariants.  Residuals are printed INTO the
%   names, because the contract for R.selfcheck is .names and .oks only and a
%   bare false is not diagnosable.
names = {}; oks = [];

% 1. the listing's grid, not a grid that resembles it
gridOK = numel(w) == 159 && abs(w(1)/10^-0.95 - 1) < 1e-12 && ...
         abs(w(end)/1000 - 1) < 1e-12 && ...
         max(abs(diff(log10(w)) - 0.025)) < 1e-12;
names{end+1} = sprintf('grid is the listing''s I=2:160 (n=%d, %.4f..%.0f rad/s)', ...
                       numel(w), w(1), w(end));
oks(end+1) = gridOK;

% 2. low-frequency asymptote.  Below every corner HG -> K0/s, so the magnitude
%    must lie on the -K0/w line.  This is the one check that would catch a
%    transposed numerator/denominator, which no benchmark row can see.
asym = 20*log10(-F.xk0/w(1));
dAs  = abs(gdb(1) - asym);
names{end+1} = sprintf('low-frequency magnitude on the -K0/w asymptote (%.4f dB off)', dAs);
oks(end+1) = dAs < 0.05;

% 3. one crossing of each kind.  Two 0-dB crossings would make "the" gain
%    margin meaningless, and bisection would still return a number.
wd = logspace(-1, 3, 4001);
[gd, pd] = hgBode(wd, F.w0, F.z0, F.waf, F.zaf, F.xk0, WACT, ZACT);
ng = sum(gd(1:end-1).*gd(2:end) < 0);
np = sum((pd(1:end-1)+180).*(pd(2:end)+180) < 0);
names{end+1} = sprintf('exactly one 0-dB and one -180deg crossing on 0.1-1000 rad/s (%d, %d)', ng, np);
oks(end+1) = (ng == 1) && (np == 1);

% 4. BOTH refined crossings inside the brackets the TRANSCRIBED grid puts them
%    in.  This is what ties the off-grid bisection back to the listing.
[gLo, gHi, gIn] = bracket(w, gdb,      0,    M.wc_rad_s);
[pLo, pHi, pIn] = bracket(w, pdeg,  -180,  M.w180_rad_s);
names{end+1} = sprintf(['refined crossings inside the grid brackets: ' ...
                        '0 dB %.3f in [%.3f, %.3f], -180deg %.3f in [%.3f, %.3f] rad/s'], ...
                       M.wc_rad_s, gLo, gHi, M.w180_rad_s, pLo, pHi);
oks(end+1) = gIn && pIn;

% .names and .oks and NOTHING else -- adaptGeneric is strict, and an extra
% diagnostic field is the near-miss schema that errors.  Hence the residuals
% being formatted into the names above.
C = struct('names', {names}, 'oks', logical(oks(:).'));
end


function [lo, hi, in] = bracket(w, y, level, wref)
%BRACKET  The adjacent grid pair that straddles LEVEL, and whether WREF is
%   between them.  NaN bracket -> not in it: a missing bracket must fail the
%   check, never pass it by default.
i = find(y(1:end-1) >= level & y(2:end) < level, 1);
if isempty(i) || ~isfinite(wref)
    lo = NaN; hi = NaN; in = false;
else
    lo = w(i); hi = w(i+1);
    in = (wref >= lo) && (wref <= hi);
end
end


% =============================================================================
function BK = bookCases(N)
%BOOKCASES  The four configurations Chapter 23 publishes open-loop numbers
%   for.  Built from BOOKNOMINAL and NOT from the caller's cfg, so the board
%   means the same thing on an off-book call as on a bare one.
spec = { 'nominal',  N.tau, N.wcr,  N.zeta, N.wact
         'tau 0.1',  0.1,   N.wcr,  N.zeta, N.wact
         'wcr 100',  N.tau, 100,    N.zeta, N.wact
         'zeta 0.4', N.tau, N.wcr,  0.4,    N.wact };
for k = 1:size(spec, 1)
    F = gainSet(N.alt, spec{k,2}, spec{k,3}, spec{k,4}, N.vm, N.wgt, N.xncg);
    M = marginsOf(F, spec{k,5}, N.zact);
    % Built field by field, not through struct(): struct() with a struct-valued
    % field is a documented expansion trap, and the field ORDER here has to be
    % identical on every pass or BK(k) = B throws.
    B = struct();
    B.name = spec{k,1};  B.tau  = spec{k,2};  B.wcr  = spec{k,3};
    B.zeta = spec{k,4};  B.wact = spec{k,5};  B.gains = F;
    B.wc_rad_s   = M.wc_rad_s;    B.pm_deg = M.pm_deg;
    B.w180_rad_s = M.w180_rad_s;  B.gm_db  = M.gm_db;
    if k == 1, BK = B; else, BK(k) = B; end
end
end


% =============================================================================
function B = benchmarks(BK)
%BENCHMARKS  Nine scored rows and one info row.  EVERY number below was
%   grepped out of zarchan.txt; the source string carries the line.  All of
%   them are prose reporting a figure, so all of them are band 'graph'.
S4  = 'Fig. 23.4 text, p.541 -- zarchan.txt:7614';
S4b = 'Fig. 23.4 text, p.542 -- zarchan.txt:7622';
S6  = 'Fig. 23.6 text, p.544 -- zarchan.txt:7634';
S7  = 'Fig. 23.7 text, p.545 -- zarchan.txt:7641';
S10 = 'Fig. 23.10 text, p.547 -- zarchan.txt:7666';

nom = BK(1); tlo = BK(2); w100 = BK(3); z04 = BK(4);
B = emptyBench();

B = addVal(B, 'gain crossover freq, nominal', 60, nom.wc_rad_s, 'graph', S4, 'rad/s', ...
    'tau 0.3 s, wcr goal 50, zeta 0.7, wact 150 -- the goal is missed on purpose');
B = addVal(B, 'gain margin, nominal', 11, nom.gm_db, 'graph', S4b, 'dB', ...
    sprintf('at the phase crossover w180 = %.1f rad/s', nom.w180_rad_s));
B = addVal(B, 'phase margin, nominal', 45, nom.pm_deg, 'graph', S4b, 'deg', ...
    'the book quotes this same case as 43 deg on p.547; see the info row');

B = addVal(B, 'gain crossover freq, wcr goal 100', 104, w100.wc_rad_s, 'graph', S7, 'rad/s', ...
    'the approximation improves as the crossover leaves the airframe behind');
B = addVal(B, 'gain margin, wcr goal 100', 1.5, w100.gm_db, 'graph', S7, 'dB', ...
    'the actuator at 150 rad/s is now inside the loop bandwidth');
B = addVal(B, 'phase margin, wcr goal 100', 5, w100.pm_deg, 'graph', S7, 'deg', ...
    'same run as the row above');

B = addVal(B, 'gain margin, zeta 0.4', 3.8, z04.gm_db, 'graph', S10, 'dB', ...
    'tau and wcr held at the nominal 0.3 s and 50 rad/s');
B = addVal(B, 'phase margin, zeta 0.4', 6, z04.pm_deg, 'graph', S10, 'deg', ...
    'same run as the row above');

% Fig. 23.6 is a two-part claim and both parts are checkable: the crossover
% must NOT move, and the gain elsewhere must.  Scoring only the first half
% would pass a transfer function that ignored tau altogether.
dWc  = abs(tlo.wc_rad_s/nom.wc_rad_s - 1);
dK0  = abs(20*log10(tlo.gains.xk0/nom.gains.xk0));   % low-frequency gain shift
B = addShape(B, 'tau 0.3 -> 0.1: crossover unchanged, gain not', ...
    'crossover unchanged; open-loop gain changes at some frequencies', ...
    sprintf('crossover %.1f -> %.1f rad/s (%.1f%%); low-freq gain %+.1f dB', ...
            nom.wc_rad_s, tlo.wc_rad_s, 100*dWc, dK0), ...
    dWc < 0.05 && dK0 > 1, S6, ...
    'both halves must hold: <5% crossover move AND >1 dB gain move');

B = addInfo(B, 'nominal phase margin is quoted twice, differently', ...
    '45 deg (p.542) and 43 deg (p.547)', sprintf('%.2f deg', nom.pm_deg), ...
    'zarchan.txt:7622 vs zarchan.txt:7666', ...
    ['both sentences describe the SAME nominal design (tau 0.3, wcr 50, ' ...
     'zeta 0.7); our value falls between them, so neither can be scored ' ...
     'as the authoritative one and the 45 deg row above is the one used']);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER are the contract.  Every row is built
%   through the adders below so the order cannot drift: a struct array whose
%   fields reordered makes [B.ok] mean something else and fails with
%   "Subscripted assignment between dissimilar structures".
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, source, units, detail)
tol = bandTol(band);
i = numel(B) + 1;
B(i).what = what;      B(i).kind = 'value';
B(i).book = book;      B(i).book_str = sprintf('%.4g', book);
B(i).ours = ours;      B(i).ours_str = sprintf('%.5g', ours);
B(i).band = band;      B(i).tol = tol;
B(i).units = units;    B(i).source = source;  B(i).detail = detail;
if ~isfinite(ours) || book == 0
    B(i).pct = NaN; B(i).ok = false; B(i).skipped = true;
    B(i).why = 'the computed value is not finite (or the book value is zero)';
    return
end
B(i).pct = 100*(ours/book - 1);
B(i).ok  = abs(ours/book - 1) <= tol;
B(i).skipped = false;  B(i).why = '';
end

function B = addShape(B, what, bookStr, oursStr, ok, source, detail)
i = numel(B) + 1;
B(i).what = what;   B(i).kind = 'shape';
B(i).book = NaN;    B(i).book_str = bookStr;
B(i).ours = NaN;    B(i).ours_str = oursStr;
B(i).pct  = NaN;    B(i).band = 'shape';  B(i).tol = NaN;
B(i).ok   = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;     B(i).units = '-';      B(i).detail = detail;
end

function B = addInfo(B, what, bookStr, oursStr, source, why)
%ADDINFO  Reported, never scored.  skipped = true keeps it out of the pass
%   count in BOTH directions -- an info row can neither pass nor fail.
i = numel(B) + 1;
B(i).what = what;   B(i).kind = 'info';
B(i).book = NaN;    B(i).book_str = bookStr;
B(i).ours = NaN;    B(i).ours_str = oursStr;
B(i).pct  = NaN;    B(i).band = 'info';  B(i).tol = NaN;
B(i).ok   = false;  B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = '-'; B(i).detail = '';
end

function t = bandTol(band)
%BANDTOL  The package's shared provenance bands.  Same numbers as
%   threeLoopAutopilot; a band encodes WHERE the value came from, never how
%   close we happen to be.
switch lower(band)
case 'table',  t = 0.01;
case 'text',   t = 0.02;
case 'approx', t = 0.05;
case 'graph',  t = 0.15;   % the author read it off a plot
otherwise
    error('mgs:zarchan:threeLoopOpenLoop:band', ...
        'unknown tolerance band "%s".', band);
end
end


% =============================================================================
function s = render(R, cfg, offbook, ev)
L = {};
L{end+1} = 'mgs.zarchan.threeLoopOpenLoop -- Zarchan Listing 23.2, open-loop Bode of the three-loop autopilot';
L{end+1} = sprintf('units: %s', R.units);
L{end+1} = sprintf('run:   alt %g ft, tau %g s, wcr %g rad/s, zeta %g, wact %g rad/s, zact %g', ...
    cfg.alt, cfg.tau, cfg.wcr, cfg.zeta, cfg.wact, cfg.zact);
if isempty(offbook)
    L{end+1} = '       (book nominal)';
else
    L{end+1} = sprintf('       OFF BOOK: %s moved -- the board below still scores the BOOK''s runs', ...
        strjoin(offbook, ', '));
end
L{end+1} = sprintf('gains reused from Listing 23.1: w0 %.4f  z0 %.4f  wAF %.4f  zAF %.5f  K0 %.4f', ...
    R.gains.w0, R.gains.z0, R.gains.waf, R.gains.zaf, R.gains.xk0);
L{end+1} = sprintf('margins: gain crossover %.3f rad/s, phase margin %.2f deg | phase crossover %.3f rad/s, gain margin %.3f dB', ...
    R.margins.wc_rad_s, R.margins.pm_deg, R.margins.w180_rad_s, R.margins.gm_db);
L{end+1} = '';
L{end+1} = 'self-checks';
for k = 1:numel(R.selfcheck.names)
    L{end+1} = sprintf('  [%s] %s', tf2s(R.selfcheck.oks(k)), R.selfcheck.names{k}); %#ok<AGROW>
end
if isempty(R.bench)
    L{end+1} = '';
    L{end+1} = 'benchmark: NOT RUN (benchmark = false).  Nothing was checked.';
    s = strjoin(L, newline); return
end
L{end+1} = '';
L{end+1} = 'benchmark board (the book''s four configurations, not this run)';
L{end+1} = sprintf('  %-38s %10s %10s %8s  %-5s %s', 'what', 'book', 'ours', 'delta', 'stat', 'source');
for k = 1:numel(R.bench)
    b = R.bench(k);
    if b.skipped
        d = '--'; st = upper(b.kind);
    else
        if isnan(b.pct), d = '--'; else, d = sprintf('%+.1f%%', b.pct); end
        st = tf2s(b.ok);
    end
    L{end+1} = sprintf('  %-38s %10s %10s %8s  %-5s %s', ...
        trunc(b.what, 38), trunc(b.book_str, 10), trunc(b.ours_str, 10), d, st, b.source); %#ok<AGROW>
    if strcmp(b.kind, 'shape') || strcmp(b.kind, 'info')
        % A shape/info row's whole payload IS its two strings, and the columns
        % above truncate at 10 characters.  Print them in full or the row says
        % nothing.
        L{end+1} = sprintf('      book: %s', b.book_str); %#ok<AGROW>
        L{end+1} = sprintf('      ours: %s', b.ours_str); %#ok<AGROW>
    end
    if b.skipped && ~isempty(b.why)
        L{end+1} = sprintf('      why: %s', b.why); %#ok<AGROW>
    end
end
L{end+1} = sprintf('  %d of %d evaluated rows pass (%d reported but not scored)', ...
    sum([R.bench(ev).ok]), sum(ev), sum(~ev));
s = strjoin(L, newline);
end

function s = tf2s(b)
if b, s = 'PASS'; else, s = 'FAIL'; end
end

function s = trunc(str, n)
str = char(str);
if numel(str) > n, s = [str(1:n-1) '~']; else, s = str; end
end


% =============================================================================
function drawCurves(R)
%DRAWCURVES  The listing's own two figures, axis limits included.
figure; semilogx(R.w, R.gain_db); grid on
xlabel('Frequency (Rad/Sec)'); ylabel('Gain (Db)'); axis([.1 1000 -60 40])
title('Listing 23.2 -- open-loop gain')
figure; semilogx(R.w, R.phase_deg); grid on
xlabel('Frequency (Rad/Sec)'); ylabel('Phase (Deg)'); axis([.1 1000 -400 100])
title('Listing 23.2 -- open-loop phase')
end
