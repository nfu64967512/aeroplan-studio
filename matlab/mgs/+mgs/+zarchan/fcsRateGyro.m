function R = fcsRateGyro(opts)
%MGS.ZARCHAN.FCSRATEGYRO  Zarchan Listing 22.2 -- the RATE GYRO flight-control
%   system driving a LINEAR airframe, ported line for line.
%
%   R = mgs.zarchan.fcsRateGyro()                        % the book run + board
%   R = mgs.zarchan.fcsRateGyro('ALT',50000,'XKR',0.25)  % the 50-kft case
%   R = mgs.zarchan.fcsRateGyro('rategyro',false)        % bare-airframe loop
%   R = mgs.zarchan.fcsRateGyro('XKR',5,'H',1e-3)        % the book's h experiment
%
%   Source: matlab/zarchan/_listings/listing_22_02.m, extracted from
%   zarchan.txt:7041-7054.  The Ch.21 notional missile (20 ft x 1 ft, 1000 lb,
%   Mach 3, cg at 10 ft, hinge line at 19.5 ft), trimmed for a 10-g command,
%   linearised, and closed by the rate gyro autopilot of Fig. 22.7.  There is
%   NO ACTUATOR in this listing: the autopilot output IS the fin deflection.
%   A 10-g step is commanded and nL is integrated for 2 s.
%
% ==========================================================================
%   *** UNITS ARE THE BOOK'S.  FEET, ft/s, slug/ft^3, g = 32.2 ft/s^2. ***
%
%   ALFTR and DELTR are RADIANS (the slender-body "2*alpha" in CN is per
%   radian; SELFCHECK 1/2 verify that by identity rather than asserting it).
%   K1 is nevertheless g/DEG, because 1845 = 57.3*32.2 does rad->deg AND
%   ft/s^2->g in one constant.  A "K1 in g/rad" is 57.3x too big and still
%   reads as a plausible number.  The rest of mgs is metres: convert at the
%   CALL SITE, visibly, never inside this file.
% ==========================================================================
%
%   CAN SAY: what a rate-gyro inner loop buys in damping over a bare airframe,
%   what it costs in bandwidth, and -- the reason this listing earns a file
%   next to its siblings -- what an integration step does to a HIGH-GAIN loop.
%   zarchan.txt:7032 warns that at KR = 5 a 10x step makes the loop "appear to
%   go unstable" when it is not: a numerical artefact wearing the costume of a
%   control failure.
%
%   CANNOT SAY anything about our aircraft.  Linearised, planar, one flight
%   condition, no actuator, no fin or acceleration limit, no gravity, no
%   seeker.  Our gamma-tracking deficit is a SATURATING lag and a linear model
%   cannot represent saturation.  Clean-loop baseline only.
%
% --------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_22_02.m reports NO unresolved comparisons and
%   that is right: every comparison in 22.2 is against a literal, and the
%   extractor had already undone the OCR's three glyph losses (',' is '<',
%   '.' is '>', a leading '2' is a minus).  Four things decided HERE, none of
%   them from "what makes the answer come out right":
%
%     1  "if ALT <= 30000." kept as extracted.  Listing 22.4 renders the same
%        line '<', so the book disagrees with itself about the boundary -- but
%        the fits are continuous there (8.747e-4 vs 8.694e-4 slug/ft^3), so it
%        cannot change a result at either published altitude.
%     2  "if S >= .0099999" kept LITERAL, as TSAMP.  At the book's H = 1e-4 it
%        fires on step 100 (the constant sits 1e-9 under 0.01; accumulation
%        error over 100 adds is ~1e-18), so it is INERT at the book's step and
%        NOT inert in general -- which is why it is an option, not a literal.
%     3  The trailing "FLAG=0" dropped: the outer loop re-initialises FLAG, so
%        it cannot be read.  Kept as a comment so nobody re-adds it thinking a
%        state machine was lost.
%     4  "save datfil.txt" and the plotting dropped; nothing is written here.
%
%   ONE QUIRK REPRODUCED.  XNL is computed inside the derivative block, so the
%   RECORDED value is evaluated at the PREDICTOR state.  E and ED are second
%   order; the recorded XNL carries the predictor's extra term, worth ~2e-5 g
%   at the book's step (see SELFCHECK 3).  Tidying it changes what this file
%   reproduces.
%
%   NEIGHBOURS, so nothing is ported twice: airframeLinear is Listing 21.2
%   (+22.1); flightControlDelay is Listing 22.4 (this loop plus an actuator
%   plus a transport delay) and its 'none' path is explicitly NOT 22.2.
%   'rategyro', false gives 22.1's loop and is here as the REFERENCE RUN the
%   damping claim at :7025 is scored against, not as a second port of 22.1.
%
% --------------------------------------------------------------------------
%   WHAT IT PRINTS.  Every number below was DERIVED first, from the book's own
%   rounded Table 23.2 derivatives (Ma -642, Md -555, Za -2.94, Zd -0.65), and
%   then MEASURED on the book run -- so a later run diffs against a number:
%
%     sea level: wAF 25.340 rad/s, zAF 0.05801, wZ 43.207, K1 -3.0715 g/deg,
%                Ta 0.45714 s, K3 -1.8890 1/s, alpha 2.812 deg, delta -3.256
%     KR = 0.1 : w 27.630 rad/s (1.090 x wAF), zeta 1.0566, KDC -3.8707,
%                poles -19.7 and -38.7 rad/s
%     nL(t)    : 20000 steps, 200 records, nL(2 s) = 10.000000 g, NO peaks and
%                no overshoot.  t = 0 IS NEVER RECORDED (first sample 0.01 s),
%                so the instantaneous -4.09 g jump from the RHP zero does not
%                appear -- the record starts at -1.859 g and rises monotonically.
%     refs     : bare airframe 8 peaks in 2 s; t95 0.450 s at 50 kft against
%                0.180 s at sea level (only the ORDERING is scored -- the book
%                prints no settling time); KR = 5 poles -2776 and -2.42 rad/s,
%                and RK2's real-axis limit is h*|s| = 2, so h = 1e-4 gives
%                0.278 (STABLE) and h = 1e-3 gives 2.78 (growth 2.08 per step,
%                DIVERGES).  That IS :7032.  Its recorded max is 7.0 g, not the
%                36 g of the t = 0 jump: the 0.36 ms mode dies before sample 1.
%     structural residuals 3e-16, 8e-16, 1.50e-05 g (band 1e-3), 1.4e-15 s.
%
%   If a residual comes back near a band EDGE that is not a pass to be
%   accepted.  Do NOT widen a band to make a row green.
%
%   OUTPUTS
%     R.t, R.xnl, R.xncg   the listing's ArrayT / ArrayXNL / ArrayXNCG
%     R.aero               trim and linearised derivatives
%     R.fcs                closed-loop w, zeta, KDC (the book's p.510 algebra)
%     R.run                metrics of the main run
%     R.ref                the reference runs (bare airframe, 50 kft, KR = 5)
%     R.bench .selfcheck .units .pass .verdict .p .summary
%
%   See also MGS.ZARCHAN.FLIGHTCONTROLDELAY, MGS.ZARCHAN.AIRFRAMELINEAR,
%   MGS.ZARCHAN.THREELOOPAUTOPILOT, MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 22.2, line 1, verbatim.
    opts.ALT   (1,1) double {mustBeFinite, mustBeNonnegative} = 0
    opts.VM    (1,1) double {mustBeFinite, mustBePositive}    = 3000
    opts.WGT   (1,1) double {mustBeFinite, mustBePositive}    = 1000
    opts.XNCG  (1,1) double {mustBeFinite, mustBePositive}    = 10
    opts.XKR   (1,1) double {mustBeFinite, mustBePositive}    = 0.1
    opts.H     (1,1) double {mustBeFinite, mustBePositive}    = 0.0001
    % The listing's own literals: "while T < 1.99999" and "if S >= .0099999".
    opts.TSTOP (1,1) double {mustBeFinite, mustBePositive}    = 1.99999
    opts.TSAMP (1,1) double {mustBeFinite, mustBePositive}    = 0.0099999
    % THE ONLY MODE SWITCHES, and they are explicit flags.  Never nargin: in a
    % function whose arguments block declares only name-value options MATLAB
    % reports nargin = 0 however many were passed, so an "if nargin == 0"
    % here would be a mode that never fires.
    opts.rategyro  (1,1) logical = true    % false = Listing 22.1's open loop
    opts.extras    (1,1) logical = true    % the three reference runs
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

if opts.VM <= 1000
    error('mgs:zarchan:fcsRateGyro:subsonic', ...
        ['VM = %g ft/s is not supersonic against the book''s fixed speed of ' ...
         'sound (1000 ft/s at every altitude), so B = sqrt(M^2-1) is ' ...
         'imaginary and every derivative below it would come back complex ' ...
         'rather than erroring.'], opts.VM);
end
if opts.H >= opts.TSAMP
    error('mgs:zarchan:fcsRateGyro:decimate', ...
        ['H = %g s is not smaller than the record interval TSAMP = %g s, so ' ...
         'the returned curve would be sampled at the step rate while its ' ...
         'time vector claimed the coarser one.'], opts.H, opts.TSAMP);
end
if opts.TSTOP <= 2*opts.H
    error('mgs:zarchan:fcsRateGyro:short', ...
        'TSTOP = %g s is not long enough for two steps at H = %g s.', ...
        opts.TSTOP, opts.H);
end

GRAV_FT = 32.2;

F  = airframe(opts.ALT, opts.VM, opts.WGT, opts.XNCG, GRAV_FT);
FC = closedLoop(F, opts.XKR, opts.rategyro);
S  = simulate(F, FC, opts.XNCG, opts.XKR, opts.H, opts.TSTOP, opts.TSAMP, ...
              opts.rategyro);

R = struct();
R.t    = S.t;
R.xnl  = S.xnl;
R.xncg = opts.XNCG*ones(size(S.t));
R.aero = F;
R.fcs  = FC;
R.run  = S;
R.units = 'ft, ft/s, slug/ft^3, g = 32.2 ft/s^2; angles rad, K1 in g/deg';

% ---- the three reference runs the book's prose is scored against ----------
R.ref = struct('done', false);
if opts.extras
    R.ref = referenceRuns(opts, GRAV_FT);
end

R.selfcheck = structuralChecks(F, FC, S, opts);

if opts.benchmark
    R.bench = benchmarks(R, opts);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine".
    R.pass  = false;
end

R.p = struct('ALT',opts.ALT, 'VM',opts.VM, 'WGT',opts.WGT, 'XNCG',opts.XNCG, ...
             'XKR',opts.XKR, 'H',opts.H, 'TSTOP',opts.TSTOP, ...
             'TSAMP',opts.TSAMP, 'rategyro',opts.rategyro, ...
             'extras',opts.extras, 'nsteps',S.nsteps, 'units',R.units);

R.summary = render(R, opts, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurve(R); end

% ---- adjudication.  The verdict is FORMED either way; 'assert' chooses only
%      the reaction.  benchSuite runs every port with assert off so that one
%      red row cannot hide the board, and a port that refuses to return is a
%      port nobody can diagnose.
msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false, so R.pass is false meaning NOT CHECKED.';
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated at ALT = %g ft, ' ...
        'VM = %g ft/s, XNCG = %g g, XKR = %g.  An unvalidated port must not ' ...
        'be handed back silently.'], opts.ALT, opts.VM, opts.XNCG, opts.XKR);
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) did not hold.  A VIOLATED one ' ...
        'is a wiring fault, not a tolerance question; a NOT RUN one is a ' ...
        'check that could not be evaluated at these parameters and is ' ...
        'reported false rather than green.  First: %s.'], ...
        sum(~R.selfcheck.oks), R.selfcheck.first_bad);
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
        error('mgs:zarchan:fcsRateGyro:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:fcsRateGyro:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function F = airframe(ALT, VM, WGT, XNCG, GRAV_FT)
%AIRFRAME  Listing 22.2's preamble, zarchan.txt:7041-7045.  Geometry -> trim
%   -> linearised aerodynamics -> autopilot gains.
%
%   The geometry is NOT an option: it is one specific vehicle and every book
%   value on the board belongs to it.  Every constant is an ARGUMENT -- MATLAB
%   local functions cannot see the caller's workspace, and a previous wave in
%   this package read one from there and threw on its first line.

A    = 1000;                    % speed of sound, ft/s, at EVERY altitude: the
                                % book's approximation, and what makes XMACH ->
                                % B -> every derivative reproduce Table 22.1.
DIAM = 1; FR = 3; XL = 20;
CTW  = 0; CRW = 6; HW = 2;      % wing: tip chord, root chord, semi-span
CTT  = 0; CRT = 2; HT = 2;      % tail: tip chord, root chord, semi-span
XN   = 4; XCG = 10; XHL = 19.5;

if ALT <= 30000.                % transcription note 1
    RHO = .002378*exp(-ALT/30000.);
else
    RHO = .0034*exp(-ALT/22000.);
end

SWING = .5*HW*(CTW+CRW);
STAIL = .5*HT*(CTT+CRT);
SREF  = 3.1416*DIAM*DIAM/4.;    % the book's 3.1416, NOT pi (2e-6 relative --
                                % invisible, and exactly the sort of silent
                                % edit that stops a digit-for-digit comparison
                                % from being one).
XLP   = FR*DIAM;
SPLAN = (XL-XLP)*DIAM + 1.33*XLP*DIAM/2.;
XCPN  = 2*XLP/3;
AN    = .67*XLP*DIAM;
AB    = (XL-XLP)*DIAM;
XCPB  = (.67*AN*XLP + AB*(XLP + .5*(XL-XLP)))/(AN+AB);
XCPW  = XLP + XN + .7*CRW - .2*CTW;
XMACH = VM/A;
XIYY  = WGT*(3*((DIAM/2)^2) + XL*XL)/(12*GRAV_FT);
TMP1  = (XCG-XCPW)/DIAM;
TMP2  = (XCG-XHL)/DIAM;
TMP3  = (XCG-XCPB)/DIAM;
TMP4  = (XCG-XCPN)/DIAM;
B     = sqrt(XMACH^2 - 1);
Q     = .5*RHO*VM*VM;
P1    = WGT*XNCG/(Q*SREF);      % the normal-force coefficient the command needs

Y1 = 2. + 8*SWING/(B*SREF) + 8*STAIL/(B*SREF);
Y2 = 1.5*SPLAN/SREF;
Y3 = 8*STAIL/(B*SREF);
Y4 = 2*TMP4 + 8*SWING*TMP1/(B*SREF) + 8*STAIL*TMP2/(B*SREF);
Y5 = 1.5*SPLAN*TMP3/SREF;
Y6 = 8*STAIL*TMP2/(B*SREF);
P2 = Y2 - Y3*Y5/Y6;
P3 = Y1 - Y3*Y4/Y6;

% Trim: CN = P1 and CM = 0 solved simultaneously.  RADIANS -- see the units
% block.  SELFCHECK 1/2 verify both identities instead of trusting this line.
ALFTR = (-P3 + sqrt(P3*P3 + 4.*P2*P1))/(2.*P2);
DELTR = -Y4*ALFTR/Y6 - Y5*ALFTR*ALFTR/Y6;

CNA  = 2 + 1.5*SPLAN*ALFTR/SREF + 8*SWING/(B*SREF) + 8*STAIL/(B*SREF);
CND  = 8*STAIL/(B*SREF);
CMAP = 2*TMP4 + 1.5*SPLAN*ALFTR*TMP3/SREF + 8*SWING*TMP1/(B*SREF);
CMA  = CMAP + 8*STAIL*TMP2/(B*SREF);
CMD  = 8*STAIL*TMP2/(B*SREF);

XMA = Q*SREF*DIAM*CMA/XIYY;
XMD = Q*SREF*DIAM*CMD/XIYY;
ZA  = -GRAV_FT*Q*SREF*CNA/(WGT*VM);
ZD  = -GRAV_FT*Q*SREF*CND/(WGT*VM);

if XMA >= 0
    % A statically stable airframe has Ma < 0.  A positive one makes
    % WAF = sqrt(-XMA) imaginary and MATLAB would carry the complex number all
    % the way to a complex "acceleration" without a word.
    error('mgs:zarchan:fcsRateGyro:staticallyUnstable', ...
        ['Ma = %.4g >= 0: this geometry is statically UNSTABLE here, so ' ...
         'WAF = sqrt(-Ma) is imaginary and the second-order airframe form ' ...
         'stops applying.'], XMA);
end

WZ  = sqrt((XMA*ZD - ZA*XMD)/ZD);       % airframe zero, right-half-plane
WAF = sqrt(-XMA);
ZAF = .5*WAF*ZA/XMA;
XK1 = -VM*(XMA*ZD - XMD*ZA)/(1845*XMA); % 1845 = 57.3*32.2; see the units block
TA  = XMD/(XMA*ZD - XMD*ZA);
XK3 = 1845*XK1/VM;

F = struct('alt',ALT, 'vm',VM, 'wgt',WGT, 'xncg',XNCG, 'rho',RHO, 'q',Q, ...
    'xmach',XMACH, 'b',B, 'sref',SREF, 'splan',SPLAN, 'swing',SWING, ...
    'stail',STAIL, 'xiyy',XIYY, 'alftr',ALFTR, 'alftr_deg',57.3*ALFTR, ...
    'deltr',DELTR, 'deltr_deg',57.3*DELTR, 'cna',CNA, 'cnd',CND, 'cma',CMA, ...
    'cmd',CMD, 'xma',XMA, 'xmd',XMD, 'za',ZA, 'zd',ZD, 'wz',WZ, 'waf',WAF, ...
    'zaf',ZAF, 'xk1',XK1, 'ta',TA, 'xk3',XK3, 'p1',P1, 'tmp1',TMP1, ...
    'tmp2',TMP2, 'tmp3',TMP3, 'tmp4',TMP4);
end


% =============================================================================
function C = closedLoop(F, XKR, rategyro)
%CLOSEDLOOP  The book's p.510 algebra, zarchan.txt:7011-7022:
%
%       KDC  = (1 - KR*K3)/(K1*KR)
%       w    = wAF*sqrt(1 - KR*K3)
%       zeta = (w/2) * (2*zAF/wAF - KR*K3*Ta)/(1 - KR*K3)
%
%   NOT used by the simulation, which integrates the states directly.  It is
%   here because the design sentence at zarchan.txt:7025 is a claim about THIS
%   algebra, and because SELFCHECK 3 needs the poles.
%
%   The open-loop branch is Listing 22.1: KDC = 1/K1, no rate feedback, so the
%   loop reduces to the bare airframe (aK = 0, w = wAF, zeta = zAF).
C = struct();
C.rategyro = rategyro;
if rategyro
    C.xkdc = (1. - XKR*F.xk3)/(F.xk1*XKR);
    C.aK   = XKR*F.xk3;          % the rate loop's contribution
    C.gDC  = XKR*C.xkdc;         % what multiplies XNCG inside DEL
else
    C.xkdc = 1./F.xk1;
    C.aK   = 0;
    C.gDC  = C.xkdc;
end
C.w          = F.waf*sqrt(1 - C.aK);
C.zeta       = 0.5*C.w*((2*F.zaf/F.waf - C.aK*F.ta)/(1 - C.aK));
C.w_over_waf = C.w/F.waf;

% Closed-loop poles of  E'' + 2*zeta*w*E' + w^2*E = w^2*Ess.
c1   = F.waf^2*(2*F.zaf/F.waf - C.aK*F.ta);      % = 2*zeta*w
disc = c1^2/4 - C.w^2;
if disc >= 0
    C.sig_slow = c1/2 - sqrt(disc);              % the pole nearest the origin
else
    C.sig_slow = c1/2;
end
C.s1 = -c1/2 + sqrt(complex(disc));
C.s2 = -c1/2 - sqrt(complex(disc));
end


% =============================================================================
function S = simulate(F, C, XNCG, XKR, H, TSTOP, TSAMP, rategyro)
%SIMULATE  Listing 22.2's loop, transcribed from zarchan.txt:7046-7053.
%
%   Zarchan's two-pass (Heun) scheme: pass 1 evaluates the derivatives at the
%   current state, pass 2 takes a full Euler step, advances T by H INSIDE the
%   inner loop, and re-evaluates there.  Moving that "T = T + H" out degrades
%   the scheme to first order while still running perfectly smoothly.
WAF = F.waf;  ZAF = F.zaf;  WZ2 = F.wz^2;
XK1 = F.xk1;  XK3 = F.xk3;  TA  = F.ta;  XKDC = C.xkdc;

E = 0.;  ED = 0.;  T = 0;  SACC = 0;
EDD = 0.;   % assigned by pass 1 before any read; initialised so the FLAG==1
            % branch can never pick up a value left over from anything else.
XNL = 0.;

nmax = ceil(TSTOP/TSAMP) + 8;
t    = zeros(nmax,1);
xnl  = zeros(nmax,1);
n    = 0;
nsteps   = 0;
diverged = false;

while T < TSTOP
    EOLD = E;  EDOLD = ED;  STEP = 1;  FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            E = E + H*ED;  ED = ED + H*EDD;  T = T + H;
        end
        if rategyro
            THD = XK3*(E + TA*ED);
            DEL = XKR*(XKDC*XNCG + THD);
        else
            % Listing 22.1: no rate feedback, KDC cancels the aero gain.
            DEL = XKDC*XNCG;
        end
        EDD = WAF*WAF*(DEL - E - 2.*ZAF*ED/WAF);
        XNL = XK1*(E - EDD/WZ2);     % evaluated at the PREDICTOR state on
        FLAG = 1;                    % pass 2 -- see the header's QUIRK note
    end
    % The listing writes "FLAG = 0" here too.  Dropped: the outer loop
    % re-initialises FLAG on its next pass, so it cannot be read.

    E  = .5*(EOLD  + E  + H*ED);
    ED = .5*(EDOLD + ED + H*EDD);
    SACC = SACC + H;
    nsteps = nsteps + 1;
    if SACC >= TSAMP
        SACC = 0.;
        n = n + 1;
        t(n) = T;  xnl(n) = XNL;
    end
    if ~isfinite(XNL) || abs(XNL) > 1e12
        % Stop rather than let Inf turn into NaN two lines later.  The book's
        % KR = 5 / h = 1e-3 case grows by 2.08 per step and reaches this in
        % about 36 steps; carrying on would only replace a big number with a
        % NaN that no longer says which way it went.
        diverged = true;
        break
    end
end

t = t(1:n);  xnl = xnl(1:n);

S = struct('t',t, 'xnl',xnl, 'nsteps',nsteps, 'diverged',diverged, ...
    'nl_final',NaN, 'nl_max',NaN, 'nl_min',NaN, 'npeaks',NaN, 't95',NaN);
if n >= 1
    S.nl_final = xnl(end);
    S.nl_max   = max(xnl);
    S.nl_min   = min(xnl);
end
if n >= 3
    % A local maximum has to STAND ABOVE its neighbours by more than rounding
    % noise.  Without the epsilon a converged, monotone curve reports ONE peak
    % -- the last few samples agree to about 1e-15 and a strict ">" comparison
    % finds a maximum in the last bit.  Measured: the KR = 0.1 response scored
    % 1 peak that way when it has none.  1e-9 of the curve's own scale is nine
    % orders below the 0.74 g by which the bare airframe's SMALLEST peak (the
    % 8th, at t = 2 s) stands clear, so this cannot hide a real oscillation.
    eps_pk = 1e-9*max(1, max(abs(xnl)));
    S.npeaks = sum(xnl(2:end-1) > xnl(1:end-2) + eps_pk & ...
                   xnl(2:end-1) > xnl(3:end)   + eps_pk);
    S.t95    = settleTime(t, xnl, XNCG, 0.05);
end
end


% =============================================================================
function tt = settleTime(t, y, target, frac)
%SETTLETIME  First recorded time after which |y - target| never again exceeds
%   frac*|target|.  NaN if it never settles inside the record.
idx = find(abs(y - target) > frac*abs(target), 1, 'last');
if isempty(idx)
    tt = t(1);
elseif idx >= numel(t)
    tt = NaN;
else
    tt = t(idx+1);
end
end


% =============================================================================
function REF = referenceRuns(opts, GRAV_FT)
%REFERENCERUNS  The three runs the chapter's PROSE claims are scored against.
%   Each one pins its OWN altitude, gain and step size, so they stay the
%   book's runs no matter what the caller asked the main run to do.
REF = struct('done', true);

% zarchan.txt:7025 -- "increasing the autopilot gain KR increases the damping
% of the flight-control system above that of the bare airframe".
Fsl  = airframe(0, opts.VM, opts.WGT, opts.XNCG, GRAV_FT);
Cbar = closedLoop(Fsl, opts.XKR, false);
REF.bare = simulate(Fsl, Cbar, opts.XNCG, opts.XKR, 1e-4, 1.99999, 0.0099999, false);

% zarchan.txt:7055 -- "the response is more sluggish at the higher altitude",
% KR = 0.1 at sea level against KR = 0.25 at 50 kft.
Csl  = closedLoop(Fsl, 0.1, true);
REF.sea = simulate(Fsl, Csl, opts.XNCG, 0.1, 1e-4, 1.99999, 0.0099999, true);
F50  = airframe(50000, opts.VM, opts.WGT, opts.XNCG, GRAV_FT);
C50  = closedLoop(F50, 0.25, true);
REF.hi  = simulate(F50, C50, opts.XNCG, 0.25, 1e-4, 1.99999, 0.0099999, true);
REF.fcs_sea = Csl;  REF.fcs_hi = C50;

% zarchan.txt:7032 -- "If the integration interval were 10 times larger, the
% system would appear to go unstable when the autopilot gain is large
% (KR = 5)."  0.5 s is enough for both verdicts: the divergent case needs
% about 36 steps and the stable case's fast mode is dead inside 5 ms.
C5 = closedLoop(Fsl, 5, true);
REF.k5_fine   = simulate(Fsl, C5, opts.XNCG, 5, 1e-4, 0.5, 0.0099999, true);
REF.k5_coarse = simulate(Fsl, C5, opts.XNCG, 5, 1e-3, 0.5, 0.0099999, true);
REF.k5_fcs    = C5;
end


% =============================================================================
function K = structuralChecks(F, C, S, opts)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan -- they are the
%   weaker claim that it agrees with ITSELF, which is what catches wiring.
%
%   A check that CAN fail still returns; nothing here errors.
%
%   A check that could NOT RUN is recorded as false, never as true.  "n/a read
%   as green" is the precise failure this package exists to prevent, so the
%   cost is accepted: a caller who deliberately diverges the run (KR = 5 at
%   h = 1e-3) gets one NOT RUN entry and a verdict that says so in those
%   words rather than calling it a violation.
names = {};  oks = [];  det = {};

% 1/2  The trim solve.  CN and CM are written out LONGHAND here, from the
%      book's own p.500-501 expressions, so a mistyped Y-shorthand in
%      AIRFRAME cannot agree with itself.
a = F.alftr;  d = F.deltr;  bs = F.b*F.sref;
CNtr = 2*a + 1.5*F.splan*a^2/F.sref + 8*F.swing*a/bs + 8*F.stail*(a+d)/bs;
CMtr = 2*a*F.tmp4 + 1.5*F.splan*a^2*F.tmp3/F.sref + ...
       8*F.swing*a*F.tmp1/bs + 8*F.stail*(a+d)*F.tmp2/bs;
r1 = abs(CNtr/F.p1 - 1);
names{end+1} = 'trim CN(alpha,delta) == commanded CN';
oks(end+1)   = r1 <= 1e-12;
det{end+1}   = sprintf('rel residual %.3e (CN %.6f vs P1 %.6f)', r1, CNtr, F.p1);

r2 = abs(CMtr)/max(abs(2*a*F.tmp4), realmin);
names{end+1} = 'trim CM(alpha,delta) == 0';
oks(end+1)   = r2 <= 1e-12;
det{end+1}   = sprintf('|CM| / |first term| = %.3e', r2);

% 3  The integrated nL against the CLOSED-FORM step response of the same
%    linear system.  This is the check that would notice a swapped derivative
%    or a sign flip in the rate feedback, both of which leave a smooth and
%    entirely plausible curve behind.
%
%    Tolerance 1e-3 g against a predicted ~2e-5 (measured 1.50e-05): the
%    recorded XNL is a PREDICTOR value, so it carries (H^2/2)*|E''| of Euler
%    error, 4e-6 relative at w*H = 2.8e-3.  The margin is over a MECHANISM,
%    not a guess.  A wiring fault moves the curve by O(1 g), five orders up.
if S.diverged || numel(S.t) < 2
    names{end+1} = 'nL matches the closed-form step response';
    oks(end+1)   = false;
    det{end+1}   = 'NOT RUN: the run diverged or recorded fewer than 2 samples';
    K.nl_resid   = NaN;
elseif abs(C.s1 - C.s2) < 1e-9
    names{end+1} = 'nL matches the closed-form step response';
    oks(end+1)   = false;
    det{end+1}   = 'NOT RUN: repeated poles, the modal form divides by s2 - s1';
    K.nl_resid   = NaN;
else
    ya = analyticNL(F, C, opts.XNCG, S.t);
    K.nl_resid = max(abs(S.xnl - ya));
    names{end+1} = 'nL matches the closed-form step response';
    oks(end+1)   = K.nl_resid <= 1e-3;
    det{end+1}   = sprintf('max |sim - closed form| = %.3e g over %d samples', ...
                           K.nl_resid, numel(S.t));
end

% 4  The decimator.  "S = S + H" once per outer step against a literal
%    0.0099999 is inert at the book's H and NOT inert in general; this is what
%    would notice it drifting.
if numel(S.t) >= 3
    dt = diff(S.t);
    K.grid_dev = max(abs(dt - median(dt)));
    names{end+1} = 'record grid is uniform';
    oks(end+1)   = K.grid_dev <= 1.5*opts.H;
    det{end+1}   = sprintf('max deviation from median spacing %.3e s (median %.5f s)', ...
                           K.grid_dev, median(dt));
else
    K.grid_dev = NaN;
    names{end+1} = 'record grid is uniform';
    oks(end+1)   = false;
    det{end+1}   = 'NOT RUN: fewer than 3 records';
end

K.names  = names;
K.oks    = logical(oks);
K.detail = det;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    i = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{i}, det{i});
end
end


% =============================================================================
function y = analyticNL(F, C, XNCG, t)
%ANALYTICNL  Exact step response of the transcribed ODEs.
%
%       E'' + 2*zeta*w*E' + w^2*E = w^2*Ess,   E(0) = E'(0) = 0
%       nL = K1*(E - E''/wZ^2)
%
%   Shares no code with the integrator, which is the whole reason it is worth
%   having: it is a second opinion, not a restatement of the first.
Ess = C.gDC*XNCG/(1 - C.aK);
s1  = C.s1;  s2 = C.s2;
t   = t(:);
e1  = exp(s1*t);  e2 = exp(s2*t);
f   = 1 - (s2*e1 - s1*e2)/(s2 - s1);
fdd = -s1*s2*(s1*e1 - s2*e2)/(s2 - s1);
y   = real(F.xk1*Ess*(f - fdd/F.wz^2));
end


% =============================================================================
%   BOOK VALUES.  THE ONE PLACE a published number is written down.
%
%   RULE OBSERVED HERE, AT COST: a value greppable out of zarchan.txt is
%   usable; a value living only on a figure does not exist, because the
%   extraction turned every figure into an image link.  What the book says in
%   PROSE about a figure IS greppable and IS used -- as shape and ordering
%   rows, which are frequently sharper than a level read off a plot.  An
%   ADJECTIVE is reported as INFO and never scored.
% =============================================================================
function V = bookValues()
V = struct();
% Table 22.1 p.501, sea-level column.  The OCR renders a leading minus as '2'.
V.waf = 25.3;      % :6934  "vAF Airframe natural frequency 25.3 rad/s"
V.zaf = 0.058;     % :6935  "zAF Airframe damping 0.058"
% Table 22.2 p.507, sea level.  K3's sign is not inferred from the OCR'd '2':
% Listing 22.3 hard-codes it at :7150 ("K3=-1.89; TA=.457; ZAF=.058; WAF=25.3").
V.ta  = 0.457;     % :6976  "Ta Turning rate time constant 0.457 s"
V.k3  = -1.89;     % :6977  "K3 Aerodynamic body rate gain 21.89 1/s"
% The steady-state row's "book value" is the COMMAND itself: :7010-7011 says
% "to get the achieved acceleration to match the commanded acceleration in the
% steady state ... KDC = (1 - KR*K3)/(K1*KR)", which the listing sets.
% :7025 / :7055 -- KR ~ 0.1 at sea level, ~0.25 at 50 kft, for damping ~ unity.
V.kr_for_unit_zeta = 0.1;
end


% =============================================================================
function B = benchmarks(R, opts)
%BENCHMARKS  Computed vs published, with a band, a reason for the band, and
%   the table/page each value came from.
%
%   TOLERANCE CLASSES, and the difference is PROVENANCE, not taste:
%     'table'  1%   printed in a table to three significant figures
%     'text'   2%   printed in running prose, or recomputed there from
%                   already-rounded table entries
%   plus 'shape' and 'bound', which are orderings and inequalities taken from
%   prose and carry no tolerance at all.  Do NOT widen a band to go green.
V = bookValues();
B = emptyBench();

aeroOff = {};
if opts.VM   ~= 3000, aeroOff{end+1} = 'VM';   end
if opts.WGT  ~= 1000, aeroOff{end+1} = 'WGT';  end
if opts.XNCG ~= 10,   aeroOff{end+1} = 'XNCG'; end
wA = gateWhy(aeroOff);
seaLevel = (opts.ALT == 0);
whySea   = 'the published column is SEA LEVEL only (ALT = 0)';

src221 = 'Table 22.1 p.501 (zarchan.txt:6934-6935)';
src222 = 'Table 22.2 p.507 + Listing 22.3 (zarchan.txt:6976-6977, 7150)';

% ---- A. the airframe the loop is closed around ---------------------------
B = addAero(B, 'wAF airframe nat. freq', V.waf, R.aero.waf, 'rad/s', src221, ...
    wA, seaLevel, whySea, ...
    'Listing 22.3 hard-codes WAF=25.3 at :7150, independently of the table');
B = addAero(B, 'zAF airframe damping', V.zaf, R.aero.zaf, '-', src221, ...
    wA, seaLevel, whySea, ...
    'the low damping is the entire reason the rate gyro loop exists');
B = addAero(B, 'Ta turning-rate const', V.ta, R.aero.ta, 's', src222, ...
    wA, seaLevel, whySea, ...
    'Ta enters zeta directly, so this row is upstream of the damping bound');
B = addAero(B, 'K3 body-rate gain', V.k3, R.aero.xk3, '1/s', src222, ...
    wA, seaLevel, whySea, ...
    'SIGNED: Listing 22.3 hard-codes K3=-1.89, so the OCR''d minus is not guessed');

% ---- B. the KDC design equation, measured.  Valid at ANY gain and altitude:
%      KDC = (1-KR*K3)/(K1*KR) makes the closed-loop DC gain exactly 1, so
%      nL(inf) must BE the command.  Gated on the transient having decayed --
%      with the arithmetic in the reason -- not on a parameter list.
srcDC  = 'p.510 (zarchan.txt:7010-7011)';
decay  = R.fcs.sig_slow*opts.TSTOP;
if R.run.diverged
    B = addSkip(B, 'nL steady state = command', sprintf('%g', opts.XNCG), ...
        'the run diverged, so there is no steady state to read', srcDC, 'g');
elseif ~(decay >= 14)
    B = addSkip(B, 'nL steady state = command', sprintf('%g', opts.XNCG), ...
        sprintf(['the slowest closed-loop pole is %.3g 1/s and the run is ' ...
        '%.4g s, so the transient is only down to exp(-%.2f) = %.1e -- not ' ...
        'settled enough for a 1%% band'], R.fcs.sig_slow, opts.TSTOP, decay, ...
        exp(-decay)), srcDC, 'g');
else
    B = addVal(B, 'nL steady state = command', opts.XNCG, R.run.nl_final, ...
        'table', srcDC, 'g', ...
        ['an EXACT cancellation, not a tolerance question: at the book''s ' ...
         'settings the transient is down to exp(-39), so the residual sits ' ...
         'at the 1e-12 level.  A miss here is the feedback path, not the ' ...
         'integrator']);
end

% ---- C. damping ~ unity at the book's gain --------------------------------
srcDes = 'p.509/512 (zarchan.txt:7025, 7055)';
if ~isempty(wA)
    B = addSkip(B, 'FCS damping at KR = 0.1', '~1', wA, srcDes, '-');
elseif ~(seaLevel && opts.XKR == V.kr_for_unit_zeta && opts.rategyro)
    B = addSkip(B, 'FCS damping at KR = 0.1', '~1', ...
        'the published pairing is SEA LEVEL, KR = 0.1, rate gyro loop', ...
        srcDes, '-');
else
    B = addBound(B, 'FCS damping at KR = 0.1', 'approx 1 (+/-10%)', ...
        sprintf('%.4f', R.fcs.zeta), abs(R.fcs.zeta - 1) <= 0.10, srcDes, '-', ...
        ['a BOUND, not a value: the book chose the round 0.1, not the gain ' ...
         'that makes zeta exactly unity, and this airframe gives 1.0566 ' ...
         'there -- which a 5% "approximately" band would FAIL on a CORRECT ' ...
         'port.  Widening that band to 6% is the move this file refuses, so ' ...
         'the row is posed as an inequality instead']);
end

% ---- D. the three prose claims, as shape rows -----------------------------
haveRef = isfield(R.ref, 'done') && R.ref.done;
whyRef  = 'the reference runs were not made (''extras'', false)';
if ~isempty(wA)
    whyRef = wA;
    haveRef = false;
end

srcDamp = 'p.509 (zarchan.txt:7025)';
if ~haveRef
    B = addSkip(B, 'rate gyro damps the bare airframe', ...
        'FCS damping > bare airframe', whyRef, srcDamp, 'peaks');
else
    nb = R.ref.bare.npeaks;  nr = R.ref.sea.npeaks;
    B = addShape(B, 'rate gyro damps the bare airframe', ...
        'bare airframe rings, closed loop does not', ...
        sprintf('bare %d peaks in 2 s, KR=0.1 %d', nb, nr), ...
        nb >= 4 && nr <= 1, srcDamp, ...
        ['"increasing the autopilot gain KR increases the damping of the ' ...
         'flight-control system above that of the bare airframe".  Scored ' ...
         'as a PEAK COUNT off both step responses, because zeta 0.058 and ' ...
         'zeta 1.06 straddle the oscillatory/overdamped boundary and that ' ...
         'IS the claim.  Measured 8 and 0; 4 and 1 are the loosest ' ...
         'thresholds that still separate them']);
end

srcSlug = 'Fig. 22.10 / p.512 (zarchan.txt:7055)';
if ~haveRef
    B = addSkip(B, '50 kft response more sluggish', ...
        't95(50 kft, KR=0.25) > t95(sea, KR=0.1)', whyRef, srcSlug, 's');
else
    ts = R.ref.sea.t95;  th = R.ref.hi.t95;
    B = addShape(B, '50 kft response more sluggish', ...
        't95(50 kft) > t95(sea level)', ...
        sprintf('%.3f s vs %.3f s (ratio %.2f)', th, ts, th/ts), ...
        isfinite(ts) && isfinite(th) && th > ts, srcSlug, ...
        ['"the response is more sluggish at the higher altitude".  An ' ...
         'ORDERING, which is all the sentence claims -- the book prints no ' ...
         'settling time.  Each run uses the book''s own gain for its ' ...
         'altitude (0.1 and 0.25); measured 0.450 s against 0.180 s']);
end

srcStep = 'p.509 (zarchan.txt:7032)';
if ~haveRef
    B = addSkip(B, 'KR = 5 stable at h, unstable at 10h', ...
        'stable at 1e-4, diverges at 1e-3', whyRef, srcStep, '-');
else
    fine   = ~R.ref.k5_fine.diverged   && isfinite(R.ref.k5_fine.nl_max);
    coarse =  R.ref.k5_coarse.diverged;
    peakFine = max(abs([R.ref.k5_fine.nl_max, R.ref.k5_fine.nl_min]));
    B = addShape(B, 'KR = 5 stable at h, unstable at 10h', ...
        'a numerical instability, not a control one', ...
        sprintf('h=1e-4 %s (max|nL| %.4g g), h=1e-3 %s', ...
                divergeStr(R.ref.k5_fine.diverged), peakFine, ...
                divergeStr(R.ref.k5_coarse.diverged)), ...
        fine && coarse, srcStep, ...
        ['"If the integration interval were 10 times larger, the system ' ...
         'would appear to go unstable when the autopilot gain is large ' ...
         '(KR = 5).  However this instability is due simply to numerical ' ...
         'integration."  A PREDICTION, not a fit: the KR = 5 poles are ' ...
         '-2776 and -2.42 1/s and RK2''s real-axis limit is h*|s| = 2, so ' ...
         'h = 1e-4 gives 0.28 and h = 1e-3 gives 2.78 (growth 2.08/step).  ' ...
         'THE row this listing is worth porting for']);
end

% ---- E. reported, never scored -------------------------------------------
if isempty(wA) && seaLevel && opts.rategyro
    B = addInfo(B, 'FCS nat freq vs airframe''s', 'not significantly different', ...
        sprintf('w/wAF = %.3f (%.2f vs %.2f rad/s)', R.fcs.w_over_waf, ...
                R.fcs.w, R.aero.waf), srcDes, ...
        ['the book''s word is an ADJECTIVE ("does not cause the natural ' ...
         'frequency ... to be significantly different").  Turning +9% into ' ...
         'a pass or a fail would invent a threshold the book never states']);
end
end


% =============================================================================
%   BENCH-ROW CONSTRUCTORS.  Field names and ORDER match the schema
%   benchSuite's adaptGeneric requires:
%     what kind book book_str ours ours_str pct band tol ok skipped why
%     source units detail
%   Every row is built through ONE of these, so the order cannot drift.  A
%   struct array whose fields reordered makes [B.ok] mean something else and
%   fails with "Subscripted assignment between dissimilar structures", which
%   reads like a typo rather than a schema break.
% =============================================================================
function B = emptyBench()
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addAero(B, what, book, ours, units, source, whyOff, seaLevel, whySea, detail)
if ~isempty(whyOff)
    B = addSkip(B, what, sprintf('%g', book), whyOff, source, units);
elseif ~seaLevel
    B = addSkip(B, what, sprintf('%g', book), whySea, source, units);
else
    B = addVal(B, what, book, ours, 'table', source, units, detail);
end
end

function B = addVal(B, what, book, ours, band, source, units, detail)
i = numel(B) + 1;
tol = bandTol(band);
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.4g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.5g', ours);
B(i).pct  = NaN;   B(i).band = band;  B(i).tol = tol;
B(i).ok = false;   B(i).skipped = true;  B(i).why = '';
B(i).source = source;  B(i).units = units;  B(i).detail = detail;
if ~isfinite(ours) || book == 0
    B(i).why = 'the computed value is not finite (or the book value is zero)';
    return
end
B(i).pct = 100*(ours/book - 1);
B(i).ok  = abs(ours/book - 1) <= tol;
B(i).skipped = false;
end

function B = addShape(B, what, bookStr, oursStr, ok, source, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'shape';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'shape';  B(i).tol = NaN;
B(i).ok = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = '-';  B(i).detail = detail;
end

function B = addBound(B, what, bookStr, oursStr, ok, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'bound';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'bound';  B(i).tol = NaN;
B(i).ok = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = units;  B(i).detail = detail;
end

function B = addInfo(B, what, bookStr, oursStr, source, why)
%ADDINFO  Reported, never scored.  skipped = true keeps it out of the pass
%   count in BOTH directions: an INFO row can neither pass nor fail.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'info';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'info';  B(i).tol = NaN;
B(i).ok = false;   B(i).skipped = true;  B(i).why = why;
B(i).source = source;  B(i).units = '-';  B(i).detail = '';
end

function B = addSkip(B, what, bookStr, why, source, units)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'skip';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = '--';
B(i).pct  = NaN;   B(i).band = 'skip';  B(i).tol = NaN;
B(i).ok = false;   B(i).skipped = true;  B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end

function t = bandTol(band)
switch lower(band)
case 'table', t = 0.01;
case 'text',  t = 0.02;
otherwise
    error('mgs:zarchan:fcsRateGyro:band', ...
        ['unknown tolerance band "%s".  Bands encode the PROVENANCE of the ' ...
         'book value; inventing one silently sets an arbitrary pass ' ...
         'criterion.'], band);
end
end

function w = gateWhy(off)
if isempty(off)
    w = '';
else
    w = sprintf(['off-book: %s was changed, so this row would compare our ' ...
        'number against a book value for a different airframe'], strjoin(off, ', '));
end
end

function s = divergeStr(d)
if d, s = 'DIVERGED'; else, s = 'bounded'; end
end


% =============================================================================
function s = render(R, opts, ev)
L = {};
L{end+1} = '=== mgs.zarchan.fcsRateGyro -- Listing 22.2, rate gyro FCS (zarchan.txt:7041-7054)';
L{end+1} = sprintf('    units: %s', R.units);
L{end+1} = sprintf('    ALT %g ft  VM %g ft/s  WGT %g lb  command %g g  KR %g  h %g s  %s', ...
    opts.ALT, opts.VM, opts.WGT, opts.XNCG, opts.XKR, opts.H, ...
    ternary(opts.rategyro, 'rate gyro loop', 'OPEN LOOP (Listing 22.1)'));
L{end+1} = sprintf('    %d outer steps, %d records', R.run.nsteps, numel(R.t));
L{end+1} = '';
L{end+1} = sprintf('  airframe : wAF %.3f rad/s  zAF %.4f  wZ %.3f rad/s  K1 %.4f g/deg', ...
    R.aero.waf, R.aero.zaf, R.aero.wz, R.aero.xk1);
L{end+1} = sprintf('             Ta %.4f s  K3 %.4f 1/s  Ma %.4g  Md %.4g  Za %.4g  Zd %.4g', ...
    R.aero.ta, R.aero.xk3, R.aero.xma, R.aero.xmd, R.aero.za, R.aero.zd);
L{end+1} = sprintf('             trim alpha %.3f deg  delta %.3f deg  q %.1f psf', ...
    R.aero.alftr_deg, R.aero.deltr_deg, R.aero.q);
L{end+1} = sprintf('  closed   : w %.3f rad/s (%.3f x wAF)  zeta %.4f  KDC %.4f', ...
    R.fcs.w, R.fcs.w_over_waf, R.fcs.zeta, R.fcs.xkdc);
L{end+1} = sprintf('  response : nL(end) %.6f g   min %.4f g   max %.4f g   peaks %g   t95 %.3f s%s', ...
    R.run.nl_final, R.run.nl_min, R.run.nl_max, R.run.npeaks, R.run.t95, ...
    ternary(R.run.diverged, '   *** DIVERGED ***', ''));
if isfield(R.ref, 'done') && R.ref.done
    L{end+1} = sprintf('  refs     : bare airframe %g peaks | 50 kft t95 %.3f s | KR=5 h=1e-4 %s, h=1e-3 %s', ...
        R.ref.bare.npeaks, R.ref.hi.t95, divergeStr(R.ref.k5_fine.diverged), ...
        divergeStr(R.ref.k5_coarse.diverged));
end
L{end+1} = '';
L{end+1} = '  structural (self-consistency, NOT book values):';
for i = 1:numel(R.selfcheck.names)
    L{end+1} = sprintf('    [%s] %-42s %s', ...
        ternary(R.selfcheck.oks(i), 'ok', 'XX'), R.selfcheck.names{i}, ...
        R.selfcheck.detail{i}); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = '  benchmarks (book value <- zarchan.txt line in each source):';
for i = 1:numel(R.bench)
    b = R.bench(i);
    switch lower(b.kind)
    case {'skip','info'}
        L{end+1} = sprintf('    [--] %-38s book %-28s (%s)', ...
            b.what, b.book_str, trunc(b.why, 70)); %#ok<AGROW>
    case {'shape','bound'}
        L{end+1} = sprintf('    [%s] %-38s want %-28s got %s', ...
            ternary(b.ok, 'ok', 'XX'), b.what, b.book_str, b.ours_str); %#ok<AGROW>
    otherwise
        L{end+1} = sprintf('    [%s] %-38s book %-10s ours %-10s %+7.3f%%  (band %g%%)', ...
            ternary(b.ok, 'ok', 'XX'), b.what, b.book_str, b.ours_str, ...
            b.pct, 100*b.tol); %#ok<AGROW>
    end
    L{end+1} = sprintf('         source: %s', b.source); %#ok<AGROW>
end
L{end+1} = sprintf('  %d row(s) evaluated, %d passed; structural %d/%d.', ...
    sum(ev), sum(ev & [R.bench.ok]), sum(R.selfcheck.oks), numel(R.selfcheck.oks));
s = strjoin(L, newline);
end

function v = ternary(c, a, b)
if c, v = a; else, v = b; end
end

function s = trunc(s, n)
if numel(s) > n, s = [s(1:n-3) '...']; end
end

function drawCurve(R)
figure;
plot(R.t, R.xnl, '-', R.t, R.xncg, '--'); grid on
xlabel('Time (Sec)'); ylabel('Missile Acceleration (G)');
title('Listing 22.2 -- rate gyro flight-control system');
legend('achieved nL', 'commanded nc', 'Location', 'southeast');
end
