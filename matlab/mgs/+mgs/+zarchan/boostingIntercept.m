function R = boostingIntercept(opts)
%MGS.ZARCHAN.BOOSTINGINTERCEPT  Zarchan Listing 14.2 -- strategic engagement
%   against a BOOSTING target, and what the target's thrust costs the missile.
%
%   R = mgs.zarchan.boostingIntercept()
%   R = mgs.zarchan.boostingIntercept('TSTART', 0, 'assert', false)
%   R = mgs.zarchan.boostingIntercept('XNCLIM', 322)          a 10-g limit
%
%   Line-by-line port of Listing 14.2 (zarchan.txt:4343-4390): an impulsive
%   interceptor, put on a collision triangle by a Lambert solution, against a
%   two-stage booster flying a gravity turn, in an Earth-centered frame.
%   Listing 13.2's lambertpz is carried here as a local function because 14.2
%   calls it and the book prints it only once.
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: ft, ft/s, ft/s^2, lb, s, deg.  nmi = 6076 ft,
%   g = 32.2, A = 2.0926e7 ft, GM = 1.4077e16 ft^3/s^2.  NOT SI, nothing is
%   converted; convert AT THE CALL SITE and say so there.
% ===========================================================================
%
%   WHY THE 14.1/14.2 PAIR IS THE POINT
%
%   A booster does not evade, but the part of its axial thrust perpendicular
%   to the line of sight is, to the missile, indistinguishable from a target
%   maneuver -- and the missile pays for it in lateral divert.  This port
%   flies the engagement THREE times: guidance off (the book's collision-
%   triangle check, and where nT is measured), PN on, and PN on with the
%   TARGET'S THRUST DELETED and nothing else changed.  The third run is NOT
%   Listing 14.1 -- 14.1 is a 500-s, 45-deg engagement, so its divert figure
%   mixes flight time and geometry in with thrust.  Benchmark 9 is the row
%   that prices thrust alone.
%
%   *** WHAT IT IS NOT. ***  An IMPULSIVE interceptor: Lambert hands it a
%   velocity at t = 0 and after that the only forces are gravity and the
%   guidance command, applied instantly -- no lag, no drag, no attitude loop,
%   no thrust of its own.  Our aircraft has all four.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION, AND THE THREE UNRESOLVED COMPARISONS
%
%   listing_14_02.m does not parse.  The OCR renders "<" as "," and ">" as
%   ".", and its header reports THREE variable-to-variable comparisons it
%   could not settle from shape.  Each is resolved below and each has its own
%   DELETE EXPERIMENT in R.selfcheck -- a resolution nobody can watch fail is
%   a resolution nobody checked.
%
%     1  "if T , TB1"  ->  T < TB1   (stage 1 is BURNING).
%        WGT = -WP1*T/TB1 + WTOT is a linear mass depletion reaching
%        WTOT - WP1 exactly at T = TB1 and going NEGATIVE past it, and TB1 =
%        XISP1*WP1/TRST1 is by construction the time to consume WP1.  Under
%        "T > TB1" the weight crosses zero mid-flight and the thrust term
%        32.2*TRST/WGT REVERSES SIGN.  Delete experiment 2.
%
%     2  "elseif T , (TB1+TB2)"  ->  T < (TB1+TB2)   (stage 2 is BURNING).
%        Same structure one stage on; the else-branch it guards is the
%        post-burnout coast (WGT = WPAY, TRST = 0), reachable only once all
%        thrust has ended.  ⚠️ NEVER DECISIVE AT THE BOOK'S OWN SETTINGS:
%        TB1 + TB2 = 69.1 s but TF = 50 s, so the coast branch is dead code in
%        every flight here.  Delete experiment 3 exercises it on the schedule
%        as a pure function; benchmark agreement is NOT evidence for it.
%
%     3  "if XNC . XNCLIM"  ->  XNC > XNCLIM   (an UPPER limit).
%        It mirrors "if XNC < -XNCLIM, XNC = -XNCLIM" two lines later, and the
%        prose calls it "a 20-g limit ... placed on the commanded
%        acceleration" (zarchan.txt:4322).  Under "<" the pair stops being a
%        limiter and becomes a CONSTANT +20-g command generator: every command
%        below the limit, the exact zeros before TSTART included, is raised to
%        +XNCLIM.  Delete experiment 4.
%
%   Comparisons against literals were repaired by shape and are not in doubt:
%   "while VC.=0." -> VC >= 0; "if TGO..1" -> TGO > 0.1; "while t,=(tf-.00001)"
%   -> t <= tf-1e-5; "if S.=.99999" -> S >= .99999; "if T.25." -> T > 25.
%   No prose leaked into this listing.
%
%   Book quirks kept, and some dropped:
%     * X1T, Y1T ARE A UNIT VECTOR.  14.1 multiplies them by VT; 14.2 does
%       not, so the booster starts at 1 ft/s along the kick direction.  That
%       is deliberate, not a dropped factor: VEL = sqrt(X1T^2+Y1T^2) is the
%       denominator of the gravity-turn thrust direction, so an exactly
%       stationary booster would divide by zero on step one.  KEPT.
%     * DELV IS TWO DIFFERENT VARIABLES -- the 20,000 ft/s sizing budget in
%       line 1, then the divert accumulator from the loop on.  Split into
%       'DELVSIZE' and R.*.divert; nothing else changed.
%     * ATPLOS uses the TOTAL target acceleration (thrust AND gravity) though
%       the prose calls it the booster acceleration perpendicular to the LOS.
%       Kept as printed, and benchmark 4 scores it as printed.
%     * XKICKDEG and XFIRSTT/YFIRSTT are each assigned twice (reproduced
%       once); "save ... /ascii" is a typo for -ascii and predictb's "h=.01"
%       is missing its semicolon -- nothing is written or echoed here.
%     * The listing hard-codes the steps .01 and .0001.  Here they are 'H'
%       and H/100 -- exactly the listing at the default H = 0.01.
%
%   OUTPUTS  R.nominal / R.guided / R.noboost (the three runs), R.sizing (the
%   Chapter 12 booster, lb), R.bench .selfcheck .pass .verdict .p .units
%   .summary -- the schema benchSuite's adaptGeneric consumes.
%
%   See also MGS.ZARCHAN.BENCHSUITE, MGS.ZARCHAN.GEMSIMULATION.

arguments
    opts.TF        (1,1) double {mustBeFinite, mustBePositive} = 50      % s
    opts.XLONGMDEG (1,1) double {mustBeFinite} = 85                      % deg
    opts.XLONGTDEG (1,1) double {mustBeFinite} = 90                      % deg
    opts.XKICKDEG  (1,1) double {mustBeFinite} = 80                      % deg
    opts.XNP       (1,1) double {mustBeFinite, mustBePositive} = 3
    opts.XNCLIM    (1,1) double {mustBeFinite, mustBePositive} = 644     % ft/s^2
    opts.TSTART    (1,1) double {mustBeFinite, mustBeNonnegative} = 25   % s
    opts.PREDERR   (1,1) double {mustBeFinite} = 0                       % ft
    opts.DELVSIZE  (1,1) double {mustBeFinite, mustBePositive} = 20000   % ft/s
    opts.H         (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s
    opts.samp      (1,1) double {mustBeFinite, mustBePositive} = 1.0     % s
    opts.LEFT      (1,1) logical = false
    % Hard walls.  The listing's only exit is VC < 0; a run that never closes
    % would hang the suite.
    opts.tmax      (1,1) double {mustBeFinite, mustBePositive} = 200     % s
    opts.maxstep   (1,1) double {mustBeFinite, mustBePositive} = 2e6
    opts.maxit     (1,1) double {mustBeFinite, mustBePositive} = 200
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.assert    (1,1) logical = true
end

if opts.H > opts.samp
    error('mgs:zarchan:boostingIntercept:samp', ...
        ['H = %g s exceeds samp = %g s, so the record would be sampled at ' ...
         'the step rate while claiming the coarser interval.'], opts.H, opts.samp);
end
if opts.tmax <= opts.TF
    error('mgs:zarchan:boostingIntercept:tmax', ...
        'tmax = %g s must exceed TF = %g s or intercept can never be reached.', ...
        opts.tmax, opts.TF);
end
if opts.TSTART >= opts.TF
    error('mgs:zarchan:boostingIntercept:tstart', ...
        ['TSTART = %g s is not before TF = %g s, so proportional navigation ' ...
         'would never switch on and the guided run would be a second copy of ' ...
         'the nominal one.'], opts.TSTART, opts.TF);
end

P           = sizeBooster(opts.DELVSIZE);
P.A         = 2.0926e7;    P.GM        = 1.4077e16;
P.DEGRAD    = 360./(2.*pi);
P.TF        = opts.TF;         P.XLONGMDEG = opts.XLONGMDEG;
P.XLONGTDEG = opts.XLONGTDEG;  P.XKICKDEG  = opts.XKICKDEG;
P.XNP       = opts.XNP;        P.XNCLIM    = opts.XNCLIM;
P.TSTART    = opts.TSTART;     P.PREDERR   = opts.PREDERR;
P.H         = opts.H;          P.samp      = opts.samp;
P.LEFT      = opts.LEFT;       P.tmax      = opts.tmax;
P.maxstep   = opts.maxstep;    P.maxit     = opts.maxit;

N = flyEngagement(P, false, true);    % nominal: PN off, booster (Figs 14.10/11)
G = flyEngagement(P, true,  true);    % Fig. 14.12: PN on for the last 25 s
D = flyEngagement(P, true,  false);   % ⭐ target thrust DELETED, all else equal

R           = struct();
R.nominal   = N;
R.guided    = G;
R.noboost   = D;
R.sizing    = P;
R.p         = opts;
R.units     = 'ft, ft/s, ft/s^2, lb, s, deg; nmi = 6076 ft; g = 32.2 ft/s^2';
R.selfcheck = structuralChecks(P, N, G, D);

if opts.benchmark
    R.bench = benchmarks(P, N, G, D, opts);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && all(R.selfcheck.oks);
else
    R.bench = emptyBench();
    ev      = false;
    R.pass  = false;          % NOT CHECKED is not the same thing as fine
end

R.summary = render(P, N, G, D, R, ev);
if opts.render, fprintf('%s\n', R.summary); end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false; nothing was compared against the book.';
elseif ~any(ev)
    msg = 'not one book value could be evaluated at these parameters.';
elseif ~all(R.selfcheck.oks)
    bad = find(~R.selfcheck.oks, 1);
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question. ' ...
        'First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.names{bad});
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
        error('mgs:zarchan:boostingIntercept:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:boostingIntercept:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function P = sizeBooster(DELV)
%SIZEBOOSTER  Listing 14.2 line 1, verbatim (zarchan.txt:4343).  Weights in lb.
%   Stage 2 is sized against the payload, stage 1 against the whole of stage 2;
%   both are the rocket equation solved for propellant weight.  TRSTn is set so
%   the axial acceleration reaches AMAXn g at THAT stage's BURNOUT, where the
%   mass is smallest -- hence "the maximum axial booster acceleration of a
%   two-stage booster is 20 g" (zarchan.txt:4322).
P       = struct();
P.XISP1 = 250.;  P.XISP2 = 250.;                        % s
P.XMF1  = .85;   P.XMF2  = .85;                         % propellant fraction
P.WPAY  = 100.;
P.DELV  = DELV;  P.DELV1 = .3333*DELV;  P.DELV2 = .6667*DELV;
P.AMAX1 = 20.;   P.AMAX2 = 20.;                         % g

TOP2    = P.WPAY*(exp(P.DELV2/(P.XISP2*32.2))-1.);
BOT2    = 1/P.XMF2-((1.-P.XMF2)/P.XMF2)*exp(P.DELV2/(P.XISP2*32.2));
P.WP2   = TOP2/BOT2;
P.WS2   = P.WP2*(1-P.XMF2)/P.XMF2;
P.WTOT2 = P.WP2+P.WS2+P.WPAY;
P.TRST2 = P.AMAX2*(P.WPAY+P.WS2);
P.TB2   = P.XISP2*P.WP2/P.TRST2;

TOP1    = P.WTOT2*(exp(P.DELV1/(P.XISP1*32.2))-1.);
BOT1    = 1/P.XMF1-((1.-P.XMF1)/P.XMF1)*exp(P.DELV1/(P.XISP1*32.2));
P.WP1   = TOP1/BOT1;
P.WS1   = P.WP1*(1-P.XMF1)/P.XMF1;
P.WTOT  = P.WP1+P.WS1+P.WTOT2;
P.TRST1 = P.AMAX1*(P.WTOT2+P.WS1);
P.TB1   = P.XISP1*P.WP1/P.TRST1;
end


% =============================================================================
function [wgt, trst] = burnState(t, P, boost, mode)
%BURNSTATE  The listing's three-branch burn schedule.
%   mode 0 = as shipped; mode 1 inverts staging test 1; mode 2 inverts staging
%   test 2.  Modes 1 and 2 exist ONLY so R.selfcheck can run the delete
%   experiments named in the header; no flight ever uses them.
if ~boost, wgt = P.WPAY;  trst = 0.;  return, end
c1 = (t < P.TB1);
c2 = (t < (P.TB1 + P.TB2));
if mode == 1, c1 = ~c1; end
if mode == 2, c2 = ~c2; end
if c1
    wgt = -P.WP1*t/P.TB1 + P.WTOT;                        trst = P.TRST1;
elseif c2
    wgt = -P.WP2*t/P.TB2 + P.WTOT2 + P.WP2*P.TB1/P.TB2;   trst = P.TRST2;
else
    wgt = P.WPAY;                                         trst = 0.;
end
end


% =============================================================================
function [xtf, ytf] = predictb(tf, xdum, ydum, x1dum, y1dum, P, boost)
%PREDICTB  Listing 14.2's prediction routine (zarchan.txt:4380-4389).
%   Integrates the BOOSTING target forward to tf so Lambert has an intercept
%   point.  Fixed h = 0.01 s, second-order Runge-Kutta (Heun), as printed --
%   the guidance loop's endgame refinement is not in here.
h = .01;
t = 0.;  x = xdum;  y = ydum;  x1 = x1dum;  y1 = y1dum;
xd = 0;  yd = 0;  x1d = 0;  y1d = 0;
while t <= (tf-.00001)
    xold = x;  yold = y;  x1old = x1;  y1old = y1;
    step = 1;  flag = 0;
    while step <= 1
        if flag == 1
            x = x+h*xd;  y = y+h*yd;  x1 = x1+h*x1d;  y1 = y1+h*y1d;
            t = t+h;  step = 2;
        end
        [wgt, trst] = burnState(t, P, boost, 0);
        at  = 32.2*trst/wgt;
        vel = sqrt(x1^2+y1^2);
        axt = at*x1/vel;  ayt = at*y1/vel;
        tembott = (x^2+y^2)^1.5;
        x1d = -P.GM*x/tembott+axt;  y1d = -P.GM*y/tembott+ayt;
        xd  = x1;  yd = y1;  flag = 1;
    end
    x  = (xold+x)/2+.5*h*xd;     y  = (yold+y)/2+.5*h*yd;
    x1 = (x1old+x1)/2+.5*h*x1d;  y1 = (y1old+y1)/2+.5*h*y1d;
end
xtf = x;  ytf = y;
end


% =============================================================================
function S = flyEngagement(P, guide, boost)
%FLYENGAGEMENT  The main loop of Listing 14.2 (zarchan.txt:4344-4379).
%   guide = false reproduces the book's nominal collision-triangle run (XNC is
%   held at zero throughout).  boost = false deletes the target's thrust and
%   changes nothing else -- benchmark 9 and self-check 5 use it.
%
%   The aPLOS average is accumulated as a TIME INTEGRAL of |ATPLOS| over
%   t > TSTART, not as a mean of the recorded samples, so benchmark 4 does not
%   move when a caller changes 'samp'.
A = P.A;  GM = P.GM;
XLONGM = P.XLONGMDEG/P.DEGRAD;
XLONGT = P.XLONGTDEG/P.DEGRAD;
XM = A*cos(XLONGM);  YM = A*sin(XLONGM);       % ALTNMMIC = 0 in the listing
XT = A*cos(XLONGT);  YT = A*sin(XLONGT);       % ALTNMTIC = 0
XFIRSTT = XT;  YFIRSTT = YT;
if P.LEFT
    X1T = cos( pi/2. - P.XKICKDEG/P.DEGRAD + XLONGT);
    Y1T = sin( pi/2. - P.XKICKDEG/P.DEGRAD + XLONGT);
else
    X1T = cos(-pi/2. + P.XKICKDEG/P.DEGRAD + XLONGT);
    Y1T = sin(-pi/2. + P.XKICKDEG/P.DEGRAD + XLONGT);
end
T = 0.;
[XTF, YTF] = predictb(P.TF, XT, YT, X1T, Y1T, P, boost);
YTF = YTF + P.PREDERR;
[VRXM, VRYM, nit, hitcap] = lambertpz(XM, YM, P.TF, XTF, YTF, XLONGM, XLONGT, P.maxit);
X1M = VRXM;  Y1M = VRYM;

RTM1 = XT-XM;  RTM2 = YT-YM;  RTM = sqrt(RTM1^2+RTM2^2);
VTM1 = X1T-X1M;  VTM2 = Y1T-Y1M;
VC   = -(RTM1*VTM1+RTM2*VTM2)/RTM;
DELV = 0.;  Sacc = 0.;
XNC = 0.;  XNCRAW = 0.;  ATPLOS = 0.;  DELVD = 0.;
XDT = 0; YDT = 0; X1DT = 0; Y1DT = 0; XDM = 0; YDM = 0; X1DM = 0; Y1DM = 0;
apacc = 0.;  aptim = 0.;
xncmax = 0.;  rawmax = 0.;  nclamp = 0;  nrawbelow = 0;  ncmd = 0;

nrec = ceil(P.tmax/P.samp)+10;
rec  = zeros(nrec, 9);  nr = 0;  nstep = 0;  timedout = false;
rfirst = sqrt(XFIRSTT^2+YFIRSTT^2);
while VC >= 0.
    TGO = RTM/VC;
    if TGO > .1, H = P.H; else, H = P.H/100.; end
    XOLDT = XT;  YOLDT = YT;  X1OLDT = X1T;  Y1OLDT = Y1T;
    XOLDM = XM;  YOLDM = YM;  X1OLDM = X1M;  Y1OLDM = Y1M;
    DELVOLD = DELV;
    step = 1;  flag = 0;
    while step <= 1
        if flag == 1
            XT = XT+H*XDT;  YT = YT+H*YDT;  X1T = X1T+H*X1DT;  Y1T = Y1T+H*Y1DT;
            XM = XM+H*XDM;  YM = YM+H*YDM;  X1M = X1M+H*X1DM;  Y1M = Y1M+H*Y1DM;
            DELV = DELV+H*DELVD;  T = T+H;  step = 2;
        end
        [WGT, TRST] = burnState(T, P, boost, 0);
        AT  = 32.2*TRST/WGT;
        VEL = sqrt(X1T^2+Y1T^2);
        AXT = AT*X1T/VEL;  AYT = AT*Y1T/VEL;
        TEMBOTT = (XT^2+YT^2)^1.5;
        X1DT = -GM*XT/TEMBOTT+AXT;  Y1DT = -GM*YT/TEMBOTT+AYT;
        XDT  = X1T;  YDT = Y1T;
        RTM1 = XT-XM;  RTM2 = YT-YM;  RTM = sqrt(RTM1^2+RTM2^2);
        VTM1 = X1T-X1M;  VTM2 = Y1T-Y1M;
        VC   = -(RTM1*VTM1+RTM2*VTM2)/RTM;
        XLAM  = atan2(RTM2, RTM1);
        XLAMD = (RTM1*VTM2-RTM2*VTM1)/(RTM*RTM);
        ATPLOS = Y1DT*cos(XLAM)-X1DT*sin(XLAM);       % thrust AND gravity
        if guide && T > P.TSTART, XNC = P.XNP*VC*XLAMD; else, XNC = 0.; end
        XNCRAW = XNC;
        if XNC >  P.XNCLIM, XNC =  P.XNCLIM; end      % resolution 3
        if XNC < -P.XNCLIM, XNC = -P.XNCLIM; end
        DELVD = abs(XNC);
        AM1 = -XNC*sin(XLAM);  AM2 = XNC*cos(XLAM);
        TEMBOTM = (XM^2+YM^2)^1.5;
        X1DM = -GM*XM/TEMBOTM+AM1;  Y1DM = -GM*YM/TEMBOTM+AM2;
        XDM = X1M;  YDM = Y1M;  flag = 1;
    end
    XT  = (XOLDT+XT)/2+.5*H*XDT;     YT  = (YOLDT+YT)/2+.5*H*YDT;
    X1T = (X1OLDT+X1T)/2+.5*H*X1DT;  Y1T = (Y1OLDT+Y1T)/2+.5*H*Y1DT;
    XM  = (XOLDM+XM)/2+.5*H*XDM;     YM  = (YOLDM+YM)/2+.5*H*YDM;
    X1M = (X1OLDM+X1M)/2+.5*H*X1DM;  Y1M = (Y1OLDM+Y1M)/2+.5*H*Y1DM;
    DELV = (DELVOLD+DELV)/2.+.5*H*DELVD;

    ncmd   = ncmd+1;
    xncmax = max(xncmax, abs(XNC));
    rawmax = max(rawmax, abs(XNCRAW));
    if XNCRAW ~= XNC,      nclamp    = nclamp+1;    end
    if XNCRAW <  P.XNCLIM, nrawbelow = nrawbelow+1; end
    if T > P.TSTART, apacc = apacc + H*abs(ATPLOS);  aptim = aptim + H; end

    Sacc = Sacc+H;
    if Sacc >= P.samp*(1-1e-5)      % = the listing's ".99999" at samp = 1 s
        Sacc = 0.;  nr = nr+1;
        rec(nr,:) = [T, ...
            (sqrt(XT^2+YT^2)-A)/6076., A*arcAngle(XT,YT,XFIRSTT,YFIRSTT,rfirst)/6076., ...
            (sqrt(XM^2+YM^2)-A)/6076., A*arcAngle(XM,YM,XFIRSTT,YFIRSTT,rfirst)/6076., ...
            XNC/32.2, XNCRAW/32.2, DELV, ATPLOS];
    end
    nstep = nstep+1;
    if T > P.tmax || nstep >= P.maxstep, timedout = true;  break, end
end
rec = rec(1:nr,:);

S = struct();
S.t = rec(:,1);  S.altT_nmi = rec(:,2);  S.distT_nmi = rec(:,3);
S.altM_nmi = rec(:,4);  S.distM_nmi = rec(:,5);
S.xnc_g = rec(:,6);  S.xnc_raw_g = rec(:,7);
S.divert_hist = rec(:,8);  S.aplos = rec(:,9);
S.miss_ft   = RTM;                                    % the listing's bare RTM
S.divert    = DELV;                                   % the listing's bare DELV
S.t_end     = T;
S.aplos_mean = apacc/max(aptim, realmin);             % mean |aPLOS|, t > TSTART
S.aplos_time = aptim;
S.xnc_max_abs    = xncmax;
S.xncraw_max_abs = rawmax;
S.n_clamped      = nclamp;
S.n_raw_below    = nrawbelow;
S.n_cmd          = ncmd;
S.lambert_it = nit;   S.hitcap = hitcap;
S.timedout   = timedout;  S.nstep = nstep;
S.guide = guide;  S.boost = boost;
end


% =============================================================================
function b = arcAngle(x, y, xf, yf, rf)
%ARCANGLE  The listing's CBETA/BETA pair, with the acos argument CLAMPED.
%   At launch the vehicle sits on the sphere and CBETA can come back as
%   1 + 2e-16; acos of that is complex and the run then carries on smoothly
%   with complex state.  A genuine |CBETA| > 1 is unreachable from a dot
%   product divided by the product of the two norms, so the clamp masks
%   nothing.
r = sqrt(x^2+y^2);
c = (x*xf+y*yf)/(r*rf);
b = acos(min(1, max(-1, c)));
end


% =============================================================================
function [vrx, vry, icount, hitcap] = lambertpz(xic, yic, tfdes, xf, yf, ...
                                                xlongm, xlongt, maxit)
%LAMBERTPZ  Listing 13.2's efficient Lambert routine, zarchan.txt:3943-3955.
%   14.2 calls it and the book prints it only once.  Iterate the flight-path
%   angle GAM until the closed-form flight time matches tfdes; bracket from the
%   escape-velocity condition, secant step with a bisection fallback.
%   hitcap = 1 when the cap was reached: the listing has no cap and would spin
%   forever on a non-finite secant step, so exhausting it here returns the
%   current velocity and lets a structural row say so.
gm  = 1.4077e16;
ric = sqrt(xic^2+yic^2);  rf = sqrt(xf^2+yf^2);
cphi = (xic*xf+yic*yf)/(ric*rf);
cphi = min(1, max(-1, cphi));                 % see arcAngle for why
phi = acos(cphi);  sphi = sin(phi);  r0 = ric;
icount = 0;  hitcap = 0;
gmin = atan2((sphi-sqrt(2.*r0*(1.-cphi)/rf)), (1-cphi));
gmax = atan2((sphi+sqrt(2.*r0*(1.-cphi)/rf)), (1-cphi));
gam = (gmin+gmax)/2.;  tf = 0;
vrx = NaN;  vry = NaN;  gold = gam;  told = 0;
while ~(abs(tfdes-tf) <= (.00000001*tfdes))
    top  = gm*(1.-cos(phi));
    temp = r0*cos(gam)/rf-cos(phi+gam);
    bot  = r0*cos(gam)*temp;
    if bot > 0
        v = sqrt(top/bot);
        if xlongt > xlongm
            vrx = v*cos(pi/2.-gam+xlongm);   vry = v*sin(pi/2.-gam+xlongm);
        else
            vrx = v*cos(-pi/2.+gam+xlongm);  vry = v*sin(-pi/2.+gam+xlongm);
        end
        xlam = r0*v*v/gm;
        if xlam > 0 && xlam < 2
            top1  = tan(gam)*(1-cos(phi))+(1-xlam)*sin(phi);
            bot1p = (1-cos(phi))/(xlam*cos(gam)*cos(gam));
            bot1  = (2-xlam)*(bot1p+cos(gam+phi)/cos(gam));
            top2  = 2*cos(gam);
            bot2  = xlam*((2/xlam-1)^1.5);
            top3  = sqrt(2/xlam-1);
            bot3  = cos(gam)/tan(phi/2)-sin(gam);
            temp  = (top2/bot2)*atan2(top3,bot3);
            tf    = r0*(top1/bot1+temp)/(v*cos(gam));
        else
            tf = inf;                         % at or past escape: too lofted
        end
    else
        tf = inf;
    end
    if ~isfinite(tf), tf = inf; end
    icount = icount+1;
    if tf > tfdes, gmax = gam; else, gmin = gam; end
    if icount == 1
        xnext = (gmax+gmin)/2.;
    else
        xnext = gam+(gam-gold)*(tfdes-tf)/(tf-told);
        % NaN fails BOTH of the listing's bracket tests, so a non-finite
        % secant step would propagate into gam and never converge.
        if ~isfinite(xnext) || xnext > gmax || xnext < gmin
            xnext = (gmax+gmin)/2.;
        end
    end
    gold = gam;  told = tf;  gam = xnext;
    if icount >= maxit, hitcap = 1;  break, end
end
end


% =============================================================================
function K = structuralChecks(P, N, G, D)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never evidence that
%   this port reproduces Zarchan -- only the weaker claim that it agrees with
%   itself, which is what catches wiring faults.  .names and .oks are what
%   benchSuite's adaptGeneric turns into board rows; without BOTH, everything
%   computed here is invisible and the board still looks green.
K = struct();  names = {};  oks = [];

% 1  the sizing block is the rocket equation, and staging sheds exactly the
%    structural weight of the stage that just finished.
d1 = P.XISP1*32.2*log(P.WTOT/(P.WTOT-P.WP1))/P.DELV1 - 1;
d2 = P.XISP2*32.2*log(P.WTOT2/(P.WTOT2-P.WP2))/P.DELV2 - 1;
eT = 1e-9;
j1 = burnState(P.TB1-eT, P, true, 0)       - burnState(P.TB1+eT, P, true, 0);
j2 = burnState(P.TB1+P.TB2-eT, P, true, 0) - burnState(P.TB1+P.TB2+eT, P, true, 0);
K.sizing_resid = max(abs([d1 d2]));
K.stage_shed   = max(abs([j1-P.WS1, j2-P.WS2]))/P.WTOT;
names{end+1} = 'rocket equation holds in both stages, and staging sheds exactly WS1/WS2';
oks(end+1)   = K.sizing_resid <= 1e-12 && K.stage_shed <= 1e-9;

% 2  ⭐ DELETE EXPERIMENT -- unresolved comparison 1, "T < TB1".
%    The shipped resolution must keep the booster weight positive for the whole
%    flight; the inverted one must not.  If both survive, the header's argument
%    for "<" is unverified.
tg = linspace(0, P.TF, 501);
ws = arrayfun(@(t) burnState(t, P, true, 0), tg);
wi = arrayfun(@(t) burnState(t, P, true, 1), tg);
K.wgt_min_ship = min(ws);  K.wgt_min_flip1 = min(wi);
names{end+1} = 'DELETE: inverting "T < TB1" drives the booster weight negative';
oks(end+1)   = K.wgt_min_ship > 0 && K.wgt_min_flip1 < 0;

% 3  ⭐ DELETE EXPERIMENT -- unresolved comparison 2, "T < TB1+TB2".
%    Never decisive at TF = 50 s, so it is exercised on the schedule as a pure
%    function across the whole of stage 2.
ts = linspace(P.TB1+1e-6, P.TB1+P.TB2-1e-6, 201);
[~, trs] = arrayfun(@(t) burnState(t, P, true, 0), ts);
[~, tri] = arrayfun(@(t) burnState(t, P, true, 2), ts);
K.trst2_ship = min(trs);  K.trst2_flip2 = max(tri);
names{end+1} = 'DELETE: inverting "T < TB1+TB2" kills stage-2 thrust outright';
oks(end+1)   = all(trs == P.TRST2) && all(tri == 0);

% 4  ⭐ DELETE EXPERIMENT -- unresolved comparison 3, "XNC > XNCLIM".
%    Run on the guided flight's own command stream: the shipped limiter never
%    lets |XNC| past XNCLIM and DOES bite at least once, while the inverted
%    resolution would pin the command at +XNCLIM on essentially every step --
%    including the whole pre-TSTART phase, where the raw command is exactly 0.
K.xnc_max_g      = G.xnc_max_abs/32.2;
K.n_clamped      = G.n_clamped;
K.flip3_pinned   = G.n_raw_below/max(G.n_cmd, 1);
names{end+1} = 'DELETE: inverting "XNC > XNCLIM" turns the limiter into a constant +XNCLIM command';
oks(end+1)   = G.xnc_max_abs <= P.XNCLIM*(1+1e-12) && G.n_clamped > 0 && ...
               K.flip3_pinned >= 0.9;

% 5  every Lambert call converged and every run actually closed.  The
%    thrust-deleted twin is included on purpose: benchmark 9 skips itself if it
%    fails, so without this row a broken twin would be invisible.
K.hitcaps   = [N.hitcap G.hitcap D.hitcap];
K.timeouts  = [N.timedout G.timedout D.timedout];
names{end+1} = 'Lambert converged on every call and all three runs closed';
oks(end+1)   = ~any(K.hitcaps) && ~any(K.timeouts);

% 6  the "nominal" run is what the book calls it: open loop, not merely
%    lightly guided.  Config-independent on purpose -- "the booster is still
%    burning at TF" is a fact about the BOOK'S case, so it is scored by
%    benchmark 8 (which skips off-case) and only reported here.
K.nominal_cmd_max = N.xncraw_max_abs;
K.tf_vs_burnout   = P.TF - (P.TB1+P.TB2);   % reported; scored by benchmark 8
names{end+1} = 'nominal run is genuinely open loop (raw command identically zero)';
oks(end+1)   = N.xncraw_max_abs == 0;

K.names = names;
K.oks   = logical(oks(:).');
end


% =============================================================================
function B = benchmarks(P, N, G, D, opts)
%BENCHMARKS  Computed vs published.  EVERY book value was grepped out of
%   zarchan_decoded.txt and its line number is in the .source field.  Nothing
%   here was read off a figure -- the extraction turned Figs. 14.10-14.13 into
%   image links -- but the book's PROSE CLAIMS about those figures are
%   greppable, and rows 4, 5 and 8 score exactly those sentences.
%
%   Bands: 'exact' = an identity of the listing's own arithmetic; 'text ~N' =
%   the band implied by the precision the book printed N to.  No band here was
%   chosen after seeing a computed number.
B = emptyBench();
sizeCase = abs(opts.DELVSIZE-20000) < 1e-9;
bookCase = sizeCase && abs(opts.TF-50) < 1e-9 && abs(opts.XLONGMDEG-85) < 1e-9 ...
    && abs(opts.XLONGTDEG-90) < 1e-9 && abs(opts.XNP-3) < 1e-9 ...
    && abs(opts.XNCLIM-644) < 1e-9 && abs(opts.TSTART-25) < 1e-9 ...
    && abs(opts.XKICKDEG-80) < 1e-9 && opts.PREDERR == 0 && ~opts.LEFT;
whyOff = sprintf(['off the book''s own case (TF = 50 s, XLONGMDEG = 85, ' ...
    'XLONGTDEG = 90, N'' = 3, XNCLIM = 644, PN from 25 s, no prediction ' ...
    'error); this run used TF = %g, XLONGMDEG = %g, N'' = %g, XNCLIM = %g, ' ...
    'TSTART = %g, PREDERR = %g'], opts.TF, opts.XLONGMDEG, opts.XNP, ...
    opts.XNCLIM, opts.TSTART, opts.PREDERR);
win = opts.TF - opts.TSTART;      % the guided window, 25 s in the book

% ---- 1.  the Chapter 12 booster ----------------------------------------
if sizeCase
    B = addVal(B, 'total booster burn time TB1 + TB2', 69.1, P.TB1+P.TB2, ...
        'text 69.1', 1e-3, ...
        'p.313 "The total burn time is 69.1 s" (zarchan.txt:4322)', 's', ...
        sprintf(['pure sizing arithmetic, no trajectory involved.  "69.1" is ' ...
        'three significant figures, so the band is its own rounding, ' ...
        '0.05/69.1 = 0.072%%, taken as 0.1%%.  Stage 1 alone burns %.4f s'], P.TB1));
else
    B = addSkip(B, 'total booster burn time TB1 + TB2', '69.1', ...
        sprintf('DELVSIZE = %g ft/s, not the book''s 20000', opts.DELVSIZE), ...
        'p.313 (zarchan.txt:4322)', 's');
end

% ---- 2.  20 g at each burnout ------------------------------------------
if sizeCase
    at1 = 32.2*P.TRST1/(P.WTOT-P.WP1);
    at2 = 32.2*P.TRST2/(P.WTOT2-P.WP2);
    B = addVal(B, 'peak axial booster acceleration (stage-1 burnout)', ...
        20*32.2, at1, 'exact', 1e-12, ...
        'p.313 "the maximum axial booster acceleration of a two-stage booster is 20 g" (zarchan.txt:4322)', ...
        'ft/s^2', sprintf(['the schedule is linear in T, so the peak sits at ' ...
        'the end of the stage, where the mass is least.  It is an identity ' ...
        'only if WTOT - WP1 really equals WS1 + WTOT2 -- that weight ' ...
        'bookkeeping is what it tests.  Stage 2 gives %.9f by the same ' ...
        'route'], at2));
else
    B = addSkip(B, 'peak axial booster acceleration (stage-1 burnout)', '644', ...
        sprintf('DELVSIZE = %g ft/s, not the book''s 20000', opts.DELVSIZE), ...
        'p.313 (zarchan.txt:4322)', 'ft/s^2');
end

% ---- 3.  the four 14.1 -> 14.2 changes ---------------------------------
B = addShape(B, 'the 14.1 -> 14.2 configuration changes named in the prose', ...
    'TF = 50 s, XLONGMDEG = 85 deg, XNCLIM = 20 g, PN from t = 25 s', ...
    sprintf('TF = %g s, XLONGMDEG = %g deg, XNCLIM = %g ft/s^2 = %g g, PN from t = %g s', ...
        opts.TF, opts.XLONGMDEG, opts.XNCLIM, opts.XNCLIM/32.2, opts.TSTART), ...
    abs(opts.TF-50) < 1e-9 && abs(opts.XLONGMDEG-85) < 1e-9 && ...
    abs(opts.XNCLIM-644) < 1e-9 && abs(opts.TSTART-25) < 1e-9, ...
    'p.313 prose (zarchan.txt:4322) against the listing''s own first line (zarchan.txt:4343)', ...
    ['⚠️ THE BOOK CONTRADICTS ITSELF HERE, and both statements are right in ' ...
     'their own units: the prose writes "(XNCLIM = 20)" -- g -- and the code ' ...
     'writes "XNCLIM=644." -- ft/s^2.  644 = 20*32.2 exactly, so the CODE is ' ...
     'what is scored.  Taking the prose literally would cap the command at ' ...
     '20 ft/s^2, i.e. 0.62 g, and the engagement would read as a guidance ' ...
     'failure rather than as a units slip']);

% ---- 4.  the apparent target maneuver ----------------------------------
if bookCase
    B = addVal(B, 'mean |aPLOS| over the guided window (guidance OFF)', ...
        129, N.aplos_mean, 'text ~129', 0.10, ...
        'p.315 "the magnitude of the booster acceleration perpendicular to the line of sight is approximately 4 g (129 ft/s2) on the average for the last 25 s" (zarchan.txt:4332), Fig. 14.11', ...
        'ft/s^2', sprintf(['measured on the GUIDANCE-OFF run, because Fig. 14.11 ' ...
        'belongs to the nominal case at zarchan.txt:4323; the guided run gives ' ...
        '%.2f over the same window.  Band: the sentence prints the number ' ...
        'twice, as "129" (three figures, 0.4%%) and as "4 g" (one figure, ' ...
        '12.5%%), and the author read it off a figure; 10%% is between the ' ...
        'two.  ⚠️ ATPLOS carries gravity as well as thrust, as printed'], ...
        G.aplos_mean));
else
    B = addSkip(B, 'mean |aPLOS| over the guided window (guidance OFF)', '129', ...
        whyOff, 'p.315 (zarchan.txt:4332)', 'ft/s^2');
end

% ---- 5.  the divert bill -----------------------------------------------
if bookCase
    B = addVal(B, 'lateral divert, PN against the boosting target', ...
        5300, G.divert, 'text ~5300', 0.05, ...
        'p.315 "the missile lateral divert requirements for this case are approximately 5300 ft/s" (zarchan.txt:4341), Fig. 14.12', ...
        'ft/s', ['⚠️ THE BOOK GIVES THIS CASE TWICE AND DISAGREES WITH ITSELF: ' ...
        '"approximately 5300 ft/s" at :4341 and "about 5200 ft/s in the ' ...
        'proportional navigation case" at :4411.  :4341 is the sentence that ' ...
        'reports the run; :4411 restates it two pages on while introducing ' ...
        'APN, so :4341 is scored.  The two differ by 1.9%, so an honest band ' ...
        'must admit both: 5% is 2.6x that spread']);
else
    B = addSkip(B, 'lateral divert, PN against the boosting target', '~5300', ...
        whyOff, 'p.315 (zarchan.txt:4341); also :4411', 'ft/s');
end

% ---- 6.  the divert formula itself -------------------------------------
if abs(opts.XNP-3) < 1e-9 && abs(win-25) < 1e-9
    dvBook = opts.XNP/(opts.XNP-1)*129*win;
    B = addVal(B, 'theoretical divert DV|PN = N''/(N''-1)*nT*tF, book inputs', ...
        4838, dvBook, 'exact', 1e-3, ...
        'p.316 "DV|PN = N''/(N''-1) nT tF = 3*129*25/2 = 4838 ft/s" (zarchan.txt:4392-4394); the formula is derived at :4304', ...
        'ft/s', ['this row checks the FORMULA, not the simulation: nT = 129 ' ...
        'ft/s^2 and tF = 25 s are the book''s own published inputs, not ours. ' ...
        'The exact product is 4837.5 and the book prints 4838, so the band is ' ...
        'the book''s own rounding to four figures']);
else
    B = addSkip(B, 'theoretical divert DV|PN = N''/(N''-1)*nT*tF, book inputs', ...
        '4838', sprintf('needs N'' = 3 and a 25 s guided window; this run has N'' = %g and %g s', ...
        opts.XNP, win), 'p.316 (zarchan.txt:4392-4394)', 'ft/s');
end

% ---- 7.  "theory and simulation are in close agreement" ----------------
if bookCase
    dvOurs = opts.XNP/(opts.XNP-1)*N.aplos_mean*win;
    rat    = G.divert/max(dvOurs, realmin);
    B = addBound(B, 'simulated divert agrees with the divert formula', ...
        '|simulated / theoretical - 1| <= 0.25', ...
        sprintf('%.4g ft/s simulated vs %.4g ft/s theoretical (%+.1f%%)', ...
                G.divert, dvOurs, 100*(rat-1)), abs(rat-1) <= 0.25, ...
        'p.316 "In other words, theory and simulation are in close agreement" (zarchan.txt:4395)', ...
        'ft/s', ['the theory side uses OUR nT and OUR window, so a drift in ' ...
        'either the aPLOS or the divert bookkeeping ALONE breaks it -- unlike ' ...
        'rows 4 and 5, which could move together and still pass.  0.25 is not ' ...
        'a measurement: it is 2.6x the book''s own gap between its 5300 ft/s ' ...
        'simulation and its 4838 ft/s theory (9.6%)']);
else
    B = addSkip(B, 'simulated divert agrees with the divert formula', ...
        '|simulated / theoretical - 1| <= 0.25', whyOff, ...
        'p.316 (zarchan.txt:4395)', 'ft/s');
end

% ---- 8.  the nominal case really is a collision triangle ---------------
%    The sentence scored here makes TWO claims -- the missile hits, and the
%    booster is still burning when it does -- so both are in the ok, and the
%    row skips off the book's case rather than reporting a false failure.
if bookCase
    stillBurning = P.TF < (P.TB1 + P.TB2);
    B = addBound(B, 'nominal (guidance-off) run hits the still-burning booster', ...
        'terminal miss < 100 ft, and TF < TB1+TB2', ...
        sprintf('miss %.4g ft; TF %.4g s vs burnout %.4g s', N.miss_ft, P.TF, P.TB1+P.TB2), ...
        ~N.timedout && N.miss_ft < 100 && stillBurning, ...
        'p.314 "Figure 14.10 shows the missile hitting the target in the nominal 50-s flight in which the booster is accelerating the entire time" (zarchan.txt:4323)', ...
        'ft', ['THE ROW THAT TESTS predictb AND lambertpz: with guidance off ' ...
        'nothing else can bring the miss down.  The 100 ft is arithmetic, not ' ...
        'taste -- the endgame step H/100 = 1e-4 s against a closing speed of ' ...
        'order 3.6e4 ft/s covers ~3.6 ft, so 100 ft is ~27 steps and five ' ...
        'orders below the ~1.8e6 ft initial range']);
else
    B = addSkip(B, 'nominal (guidance-off) run hits the still-burning booster', ...
        'terminal miss < 100 ft, and TF < TB1+TB2', whyOff, ...
        'p.314 (zarchan.txt:4323)', 'ft');
end

% ---- 9.  ⭐ WHAT THE TARGET'S THRUST COSTS -----------------------------
if D.timedout || D.hitcap
    B = addSkip(B, 'target thrust dominates the interceptor''s divert bill', ...
        'boosting divert >= 10x thrust-deleted divert', ...
        'the thrust-deleted twin did not converge or did not close, so the pair is not comparable', ...
        'p.315 (zarchan.txt:4341) against p.311 (zarchan.txt:4255)', 'ft/s');
else
    rat9 = G.divert/max(D.divert, realmin);
    B = addShape(B, 'target thrust dominates the interceptor''s divert bill', ...
        'boosting divert >= 10x the divert against the same target with its thrust deleted', ...
        sprintf('%.4g ft/s boosting vs %.4g ft/s thrust-deleted (%.0fx)', ...
                G.divert, D.divert, rat9), rat9 >= 10, ...
        'p.315 "approximately 5300 ft/s" for the boosting case (zarchan.txt:4341) against p.311 "about 350 ft/s of lateral divert was required by the missile to intercept the ballistic target" for Listing 14.1 (zarchan.txt:4255)', ...
        ['⭐ THE PAIR ROW.  The 10x is read off the BOOK''S own pair -- ' ...
        '5300/350 = 15x -- not off this run.  ⚠️ The twin is NOT Listing ' ...
        '14.1: 14.1 is a 500-s, 45-deg engagement, so its 350 ft/s mixes ' ...
        'flight time and geometry in with thrust.  This row prices thrust ' ...
        'ALONE -- same TF, same longitudes, same predictor and Lambert, one ' ...
        'term removed from the target''s acceleration -- which makes 10x ' ...
        'conservative: a short flight against a nearly static target should ' ...
        'need LESS divert than 14.1''s 350 ft/s, not more']);
end
end


% =============================================================================
function s = render(P, N, G, D, R, ev)
L = {};
L{end+1} = 'mgs.zarchan.boostingIntercept -- Zarchan Listing 14.2, boosting-target intercept';
L{end+1} = sprintf('  units: %s', R.units);
L{end+1} = sprintf(['  booster: WTOT %.1f lb, WP1 %.1f, WP2 %.1f, TB1 %.3f s, ' ...
    'TB2 %.3f s, TB1+TB2 %.3f s'], P.WTOT, P.WP1, P.WP2, P.TB1, P.TB2, P.TB1+P.TB2);
L{end+1} = sprintf('  run                     miss(ft)   divert(ft/s)  mean|aPLOS|(ft/s^2)  t_end(s)');
runs = {N, G, D};
tags = {'nominal (PN off)', 'guided (PN on)', 'thrust DELETED '};
for i = 1:3
    r = runs{i};
    L{end+1} = sprintf('  %-22s %9.3f %13.2f %18.2f %10.3f', ...
        tags{i}, r.miss_ft, r.divert, r.aplos_mean, r.t_end); %#ok<AGROW>
end
L{end+1} = '  benchmarks';
for i = 1:numel(R.bench)
    b = R.bench(i);
    if b.skipped
        L{end+1} = sprintf('    [SKIP] %-56s  %s', b.what, b.why); %#ok<AGROW>
    elseif strcmp(b.kind, 'value')
        L{end+1} = sprintf('    [%s] %-56s book %-10s ours %-10s %+7.2f%% (band %s)', ...
            verdictTag(b.ok), b.what, b.book_str, b.ours_str, b.pct, b.band); %#ok<AGROW>
    else
        L{end+1} = sprintf('    [%s] %-56s want %s | got %s', ...
            verdictTag(b.ok), b.what, b.book_str, b.ours_str); %#ok<AGROW>
    end
end
L{end+1} = '  structural invariants';
for i = 1:numel(R.selfcheck.names)
    L{end+1} = sprintf('    [%s] %s', verdictTag(R.selfcheck.oks(i)), ...
        R.selfcheck.names{i}); %#ok<AGROW>
end
if any(ev)
    L{end+1} = sprintf('  %d of %d evaluated book values inside tolerance.', ...
        sum([R.bench(ev).ok]), sum(ev));
end
s = strjoin(L, newline);
end

function t = verdictTag(ok)
if ok, t = ' OK '; else, t = 'FAIL'; end
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER must match the adders exactly; a reordered
%   struct makes [B.ok] mean something else while still looking like a typo.
%   This is the schema benchSuite's adaptGeneric consumes, and it is STRICT --
%   a near miss errors rather than being accommodated.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.6g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.6g', ours);
B(i).pct  = NaN;   B(i).band = band;  B(i).tol = tol;
B(i).ok   = false; B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = units;  B(i).detail = detail;
if ~isfinite(ours) || book == 0
    B(i).skipped = true;
    B(i).why = 'the computed value is not finite (or the book value is zero)';
    return
end
B(i).pct = 100*(ours/book - 1);
B(i).ok  = abs(ours/book - 1) <= tol;
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

function B = addSkip(B, what, bookStr, why, source, units)
%ADDSKIP  A check that COULD NOT RUN.  Never a pass.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'skip';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = '--';
B(i).pct  = NaN;   B(i).band = 'skip';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true;  B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end
