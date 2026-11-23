function R = lambertBruteForce(opts)
%MGS.ZARCHAN.LAMBERTBRUTEFORCE  Zarchan Listing 13.1 -- Lambert by BRUTE FORCE.
%
%   R = mgs.zarchan.lambertBruteForce()
%   R = mgs.zarchan.lambertBruteForce('TF', 1500, 'assert', false)
%   R = mgs.zarchan.lambertBruteForce('step1', 1, 'twoloops', false)  % Table 13.1 row 3
%
%   Line-by-line port of Listing 13.1 (zarchan.txt:3890-3903).  Given the
%   missile's and the target's angular position and altitude and a desired
%   flight time, find the initial velocity vector that flies one to the other in
%   a 1/r^2 gravity field, by scanning the flight-path angle over all of
%   [-90,+90] deg in 0.1-deg steps and then rescanning a 1.15-deg window in
%   0.0001-deg steps.
%
%   UNITS ARE THE BOOK'S: FEET, ft/s, SECONDS, DEGREES for the search.
%   a = 2.0926e7 ft, gm = 1.4077e16 ft^3/s^2, 1 n mi = 6076 ft.  Nothing is
%   converted to SI, so every number printed can be laid straight against a
%   published one.  Convert AT THE CALL SITE and say so there.
%
%   The 13.1/13.2 PAIR is the same answer, one slow and one fast: 13.2 replaces
%   the scan with a bracketed search on the escape-velocity interval.  See NOTES
%   FOR THE 13.2 PORT at the end of this header for the division of labour.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_13_01.m does not parse.  Throughout this book
%   "," is the OCR of "<" and "." is the OCR of ">".  Three forced repairs:
%   "while SecondTimeThrough ,= 1" -> "<= 1" (the body only increments, so ">="
%   never ends); "if ~(top,0. | bot,0.)" -> "~(top < 0 || bot < 0)", the guard
%   on sqrt() of a negative; "save datfil.txt output /ascii" is the book's own
%   typo for -ascii and writes nothing here.  The transcription's trailing
%   paragraph and TABLE 13.1 are the book's next PROSE (:3910-3915) leaking in;
%   dropped as code, then used as the benchmark table -- which is how they were
%   identified.  ArrayBGAM = 57.3*gam is KEPT: 57.3 is not 180/pi, so the
%   recorded angle runs 0.0074% high, but it is a plot label only and the search
%   itself uses degrad = 360/(2*pi).  R.arrays carries both.
%
%   ITS TWO UNRESOLVED VARIABLE-TO-VARIABLE COMPARISONS, resolved, each MEASURED
%   by a delete experiment below rather than merely argued:
%
%     "if xlongt.xlongm" -> "if xlongt > xlongm".  The true branch uses
%     cos(pi/2 - gam + xlongm), the book's COUNTERCLOCKWISE formula (:3862,
%     Fig. 13.1); the else branch is its CLOCKWISE one (:3866, Fig. 13.2).  A
%     target at a LARGER angle is reached counterclockwise, so ">".  Measured
%     twice: the self-check that recovers the flight-path angle from (vrx,vry),
%     and the delete experiment that forces the other branch -- miss 3.0e7 ft,
%     more than an Earth radius, with no other symptom anywhere.
%
%     "if (tf .tfdes) break" -> "if tf > tfdes, break".  The prose says it
%     outright, :3880: "We stop the loop when the computed flight time is
%     greater than the desired flight time."  Measured: flipped, the scan breaks
%     on its first valid angle and returns gam = -11.25 deg -- a trajectory
%     THROUGH the Earth, the case :3881 says must be rejected.
%
% ---------------------------------------------------------------------------
%   THREE THINGS ABOUT TABLE 13.1, all of which change what "agrees" means.
%
%   THE MINUS SIGN.  The VRX column reads 27696, 27668, 27418 (:3913-3915).
%   Those are -7696, -7668, -7418: this extraction maps a minus sign onto the
%   digit 2.  Two independent forcings, not a guess.  GEOMETRY: vrx =
%   |V|*sin(gam - xlongm) and the solution has gam = 22.2 deg against xlongm =
%   45 deg, so vrx MUST be negative.  ENERGY: |V| would be 33,212 ft/s if VRX
%   were +27,696, against 25,937 circular and 36,680 escape at this radius -- a
%   near-escape ellipse that does not come back in 1000 s.  With the sign
%   restored |V| = 19,879 ft/s and apogee is 305 n mi.  VRY is undamaged.
%
%   ROW 3 HAS BOTH LOOPS REMOVED.  The prose (:3910) switches the first loop to
%   1-deg steps AFTER having removed the second, and the numbers confirm it: one
%   loop at 1 deg gives (-7418, 18360, 1014.26), the row to four figures, while
%   keeping the refinement pass gives (-7471, 18354, 1011.49) -- two of three
%   published cells missed by many times their last printed digit.  Two scored
%   rows turn on this reading, so it is not free.
%
%   THE BOOK IS OFF BY ONE ON ITS OWN NOMINAL COUNT.  Table 13.1 and the
%   Fig. 13.4 caption (:3885, :3889, :3913) say 1084 iterations; the listing's
%   own `count` reaches 1083, because it records only angles whose tF is still
%   BELOW target and the answer is delivered by the NEXT (breaking) visit --
%   number 1084 if you number the recorded ones 1..1083.  The other two rows of
%   the same table (335, 34) match `count` EXACTLY under that same convention,
%   so the table is internally inconsistent by one.  Scored with a +/-1 COUNT
%   band and named; the other two count rows are scored exactly.  It is not
%   floating-point noise: tF at the breaking angle is 1000.000625 s and one
%   1e-4-deg step is worth 0.00183 s, so landing on 1084 would need tF to move
%   by 6e-7 relative -- ten orders above double precision.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED. ***  MATLAB is the
%   caller's, not this author's; these come from an independent
%   reimplementation of the same arithmetic, so that a first run has something
%   to be compared against instead of merely admired:
%     nominal   vrx = -7695.5957  vry = 18329.3032  tF = 1000.000625 s
%               count = 1083   gamdeg = 22.2248 deg   lambda = 0.587458
%     one loop, 0.1 deg   -7668.5172  18332.1806  1001.3757 s  count 335
%     one loop, 1.0 deg   -7417.7659  18359.6149  1014.2637 s  count  34
%     round-trip miss at the routine's own tF ~1e-6 ft (RK4, h = 0.5 s); at the
%     DESIRED 1000 s, 12.4 ft two-loop against 27,326 ft one-loop
%   Do not widen a band to make a row green.  A red row is a result.
%
%   OUTPUTS.  R.vrx R.vry R.v R.tf R.gamdeg R.count R.lambda R.phi_deg;
%   R.arrays (the listing's ArrayICOUNT/BGAM/VRX/VRY/TF plus the exact angle and
%   a pass index); R.experiments (the three Table 13.1 conditions);
%   R.miss_at_tf_ft, R.miss_at_tfdes_ft; R.bench; R.selfcheck (.names/.oks);
%   R.pass; R.verdict; R.p; R.units; R.summary.  'assert' false WARNS and still
%   returns R complete -- benchSuite runs every port that way.
%
%   NOTES FOR THE 13.2 PORT (do not duplicate this work): 13.2 solves the SAME
%   nominal case, so its vrx/vry/tF/lambda must land on the numbers above --
%   cross-check against those rather than re-deriving them.  What 13.2 owns and
%   this file deliberately does not: Table 13.2's per-iteration convergence, the
%   "four iterations" and "more than two orders of magnitude faster" claims
%   (:3957), and the escape-velocity bracket that replaces the scan.  That speed
%   claim is a RATIO, and 1083 iterations here is its denominator.
%
%   See also MGS.ZARCHAN.BENCHSUITE.

arguments
    opts.XLONGMDEG (1,1) double {mustBeFinite} = 45     % missile longitude, deg
    opts.XLONGTDEG (1,1) double {mustBeFinite} = 90     % target  longitude, deg
    opts.ALTNMM    (1,1) double {mustBeFinite, mustBeNonnegative} = 0   % n mi
    opts.ALTNMT    (1,1) double {mustBeFinite, mustBeNonnegative} = 0   % n mi
    opts.TF        (1,1) double {mustBeFinite, mustBePositive} = 1000   % s
    opts.a         (1,1) double {mustBeFinite, mustBePositive} = 2.0926e7  % ft
    opts.gm        (1,1) double {mustBeFinite, mustBePositive} = 1.4077e16 % ft^3/s^2
    opts.step1     (1,1) double {mustBeFinite, mustBePositive} = 0.1    % deg, loop 1
    opts.step2     (1,1) double {mustBeFinite, mustBePositive} = 1e-4   % deg, loop 2
    opts.backoff   (1,1) double {mustBeFinite, mustBePositive} = 0.15   % deg
    opts.span2     (1,1) double {mustBeFinite, mustBePositive} = 1.0    % deg
    opts.twoloops  (1,1) logical = true     % false = the book's own experiment
    % RK4 step for the independent round-trip check.  0.5 s over a 1000-s flight
    % already lands within ~1e-6 ft; the propagator is not the thing under test,
    % so it is not refined further.
    opts.prop_h    (1,1) double {mustBeFinite, mustBePositive} = 0.5    % s
    % THE ONLY SWITCHES.  Not nargin: with an arguments block declaring only
    % name-value options, MATLAB reports nargin = 0 however many were passed.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

DEGRAD = 360./(2.*pi);
ALTT   = opts.ALTNMT*6076.;      ALTM   = opts.ALTNMM*6076.;
XLONGM = opts.XLONGMDEG/DEGRAD;  XLONGT = opts.XLONGTDEG/DEGRAD;
XM = (opts.a+ALTM)*cos(XLONGM);  YM = (opts.a+ALTM)*sin(XLONGM);
XT = (opts.a+ALTT)*cos(XLONGT);  YT = (opts.a+ALTT)*sin(XLONGT);

% ---- the nominal run, at whatever the caller asked for ---------------------
S = olambert(XM,YM,opts.TF,XT,YT,XLONGM,XLONGT,opts.a,opts.gm, ...
             opts.step1,opts.step2,opts.backoff,opts.span2,opts.twoloops, ...
             'book', +1);

% ---- the three conditions of TABLE 13.1 ------------------------------------
% Rows 2 and 3 fix their OWN step and drop the second loop; that is the book's
% experiment, not a caller option, so they do not read opts.step1/twoloops.
% They are run only on the published case: off it there is nothing to compare
% them against, and a coarse 1-deg scan is entitled to fail to bracket a flight
% time the 0.1-deg scan reached, which would take the whole port down with it.
bookCase = abs(opts.XLONGMDEG-45)<1e-12 && abs(opts.XLONGTDEG-90)<1e-12 && ...
           opts.ALTNMM==0 && opts.ALTNMT==0 && abs(opts.TF-1000)<1e-12 && ...
           abs(opts.a-2.0926e7)<1 && abs(opts.gm-1.4077e16)<1e6 && ...
           abs(opts.step2-1e-4)<1e-12 && abs(opts.backoff-0.15)<1e-12;
if bookCase
    S2 = olambert(XM,YM,opts.TF,XT,YT,XLONGM,XLONGT,opts.a,opts.gm, ...
                  0.1,opts.step2,opts.backoff,opts.span2,false,'book',+1);
    S3 = olambert(XM,YM,opts.TF,XT,YT,XLONGM,XLONGT,opts.a,opts.gm, ...
                  1.0,opts.step2,opts.backoff,opts.span2,false,'book',+1);
else
    S2 = notRun();  S3 = notRun();
end
E = struct('name',{},'vrx',{},'vry',{},'tf',{},'count',{},'gamdeg',{});
E = addExp(E, 'Nominal', S);
E = addExp(E, 'Remove second loop', S2);
E = addExp(E, 'One-degree increments', S3);

R = struct();
R.vrx = S.vrx;  R.vry = S.vry;  R.v = S.v;  R.tf = S.tf;
R.gamdeg = S.gamdeg;  R.count = S.count;  R.lambda = S.xlam;
R.phi_deg = S.phi*DEGRAD;
R.arrays = S.arrays;
R.experiments = E;
R.xm = XM;  R.ym = YM;  R.xt = XT;  R.yt = YT;

% Independent verification: FLY the answer.  Nothing in this shares code with
% the closed forms the listing iterates on.
sOwn = kepProp(XM,YM,S.vrx,S.vry,S.tf,    opts.gm, opts.prop_h);
sDes = kepProp(XM,YM,S.vrx,S.vry,opts.TF, opts.gm, opts.prop_h);
R.miss_at_tf_ft    = hypot(sOwn(1)-XT, sOwn(2)-YT);
R.miss_at_tfdes_ft = hypot(sDes(1)-XT, sDes(2)-YT);

R.p = struct('XLONGMDEG',opts.XLONGMDEG,'XLONGTDEG',opts.XLONGTDEG, ...
    'ALTNMM',opts.ALTNMM,'ALTNMT',opts.ALTNMT,'TF',opts.TF,'a',opts.a, ...
    'gm',opts.gm,'step1',opts.step1,'step2',opts.step2,'backoff',opts.backoff, ...
    'span2',opts.span2,'twoloops',opts.twoloops,'prop_h',opts.prop_h, ...
    'units','ft, ft/s, s, deg; a = 2.0926e7 ft, gm = 1.4077e16 ft^3/s^2');
R.units = 'ft, ft/s, s, deg (the book''s own units; NOT SI)';

% The break-test delete experiment needs its own run; it is cheap, because a
% flipped test breaks almost immediately.  It is allowed to FAIL to converge --
% that is one of the ways a broken break test shows itself -- so the failure is
% caught and reported as evidence rather than being allowed to kill the port.
try
    Sflip = olambert(XM,YM,opts.TF,XT,YT,XLONGM,XLONGT,opts.a,opts.gm, ...
                     opts.step1,opts.step2,opts.backoff,opts.span2, ...
                     opts.twoloops, 'book', -1);
    Sflip.threw = '';
catch err
    Sflip = struct('tf',NaN, 'gamdeg',NaN, 'count',NaN, 'threw',err.message);
end
R.selfcheck = structuralChecks(S, Sflip, XM, YM, XT, YT, XLONGM, XLONGT, ...
                               opts.gm, opts.TF, opts.prop_h, opts.twoloops, ...
                               R.miss_at_tf_ft);

if opts.benchmark
    R.bench = benchmarks(R, E, S2, opts, bookCase, XM, YM, XT, YT);
    ev = ~[R.bench.skipped];
    R.pass = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();  ev = false;
    R.pass = false;   % NOT true.  "not checked" must never render as "fine".
end

R.summary = render(R, E, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot
    figure; plot(R.arrays.icount, R.arrays.tf); grid on
    xlabel('iteration'); ylabel('t_F [s]');
    title('Zarchan Fig. 13.4 -- brute-force Lambert convergence');
end

msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the book. ' ...
           'R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated: Table 13.1 is ' ...
        'published for XLONGMDEG = 45, XLONGTDEG = 90, alt = 0, TF = 1000 s ' ...
        'and the listing''s own constants.  This run used %g/%g deg, TF = %g s.'], ...
        opts.XLONGMDEG, opts.XLONGTDEG, opts.TF);
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question. ' ...
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
        error('mgs:zarchan:lambertBruteForce:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN.  A port that errors here hands back nothing, and then
        % nobody can see WHICH side of a comparison moved.
        warning('mgs:zarchan:lambertBruteForce:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function S = olambert(xic,yic,tfdes,xf,yf,xlongm,xlongt,aE,gmE, ...
                      step1,step2,backoff,span2,twoloops,ccwMode,tfSign) %#ok<INUSD>
%OLAMBERT  Listing 13.1's subroutine, zarchan.txt:3894-3903.
%
%   Every constant arrives as an ARGUMENT.  MATLAB local functions cannot see
%   the caller's workspace, and the listing's own "global a gm" is precisely the
%   construct that cannot survive the translation.
%
%   aE IS UNUSED ON PURPOSE.  The listing declares "global a gm" inside
%   olambert and then never reads a: the routine works entirely from r0 = |ric|
%   and rf = |rf|, so the Earth radius only enters through the caller.  Kept in
%   the signature so the correspondence with the listing stays visible.
%
%   ccwMode : 'book' uses the listing's own xlongt > xlongm test; 'ccw'/'cw'
%             force a branch, for the delete experiment.
%   tfSign  : +1 is the listing's "break when tf > tfdes"; -1 flips it, for the
%             other delete experiment.

count=0; vrx=0.; vry=0.; tf=0.; v=0.; xlam=NaN;
ric=sqrt(xic^2+yic^2); rf=sqrt(xf^2+yf^2);
cphi=(xic*xf+yic*yf)/(ric*rf); phi=acos(cphi); r0=ric;
degrad=360./(2.*pi);

nmax = numel(-90:step1:90);
if twoloops, nmax = nmax + ceil((backoff+span2)/step2) + 2; end
Aic=zeros(nmax,1); Abg=zeros(nmax,1); Agd=zeros(nmax,1);
Avx=zeros(nmax,1); Avy=zeros(nmax,1); Atf=zeros(nmax,1); Aps=zeros(nmax,1);

broke=[false false]; cpass=[0 0]; tfStart2=NaN; seen2=false;
gamdeg=NaN; gamdegnew=NaN; gamdegfin=NaN;
SecondTimeThrough=0;
while SecondTimeThrough <= double(twoloops)          % the listing: "<= 1"
    p = SecondTimeThrough + 1;
    if SecondTimeThrough == 0
        start=-90; step=step1; stop=+90;
    else
        start=gamdegnew; step=step2; stop=gamdegfin;
    end
    gvec = start:step:stop;
    if isempty(gvec)
        error('mgs:zarchan:lambertBruteForce:emptyscan', ...
            'search pass %d has an empty grid (%g:%g:%g).', p, start, step, stop);
    end
    for gamdeg = gvec
        gam=gamdeg/degrad;
        top=gmE*(1-cos(phi));
        temp=r0*cos(gam)/rf-cos(phi+gam);
        bot=r0*cos(gam)*temp;
        if ~(top < 0. || bot < 0.)
            v=sqrt(top/bot);
            switch ccwMode
                case 'ccw', ccw = true;
                case 'cw',  ccw = false;
                otherwise,  ccw = xlongt > xlongm;    % the listing's own test
            end
            if ccw
                vrx=v*cos(pi/2 -gam+xlongm); vry=v*sin(pi/2 -gam+xlongm);
            else
                vrx=v*cos(-pi/2 +gam+xlongm); vry=v*sin(-pi/2 +gam+xlongm);
            end
            xlam=r0*v*v/gmE;
            top1=tan(gam)*(1-cos(phi))+(1-xlam)*sin(phi);
            bot1p=(1-cos(phi))/(xlam*cos(gam)*cos(gam));
            bot1=(2-xlam)*(bot1p+cos(gam+phi)/cos(gam));
            top2=2*cos(gam);
            if ~((2/xlam-1) < 0.)
                bot2=xlam*((2/xlam-1)^1.5);
                top3=sqrt(2/xlam-1);
                bot3=cos(gam)/tan(phi/2)-sin(gam);
                tmp=(top2/bot2)*atan2(top3,bot3);
                tf=r0*(top1/bot1+tmp)/(v*cos(gam));
                if p==2 && ~seen2, tfStart2=tf; seen2=true; end
                if tfSign*(tf - tfdes) > 0
                    broke(p)=true;  break                % out of the for loop
                end
                count=count+1; cpass(p)=cpass(p)+1;
                Aic(count)=count; Abg(count)=57.3*gam;   % the book's own 57.3
                Agd(count)=gamdeg;
                Avx(count)=vrx; Avy(count)=vry; Atf(count)=tf; Aps(count)=p;
            end
        end
    end
    gamdegnew=gamdeg-backoff; gamdegfin=gamdeg+span2;
    SecondTimeThrough=SecondTimeThrough+1;
end

if ~broke(1)
    error('mgs:zarchan:lambertBruteForce:noroot', ...
        ['no flight-path angle in [-90,90] deg at %g-deg steps produces a ' ...
         'flight time above the desired %g s, so the scan never stopped on a ' ...
         'solution.  Returning the last grid point would look like an answer ' ...
         'and is not one.'], step1, tfdes);
end
if twoloops && ~broke(2)
    error('mgs:zarchan:lambertBruteForce:nobracket', ...
        ['the refinement pass ran to its end (%g deg) without crossing the ' ...
         'desired flight time of %g s.'], gamdegfin, tfdes);
end

S = struct('vrx',vrx, 'vry',vry, 'v',v, 'tf',tf, 'count',count, ...
    'gamdeg',gamdeg, 'xlam',xlam, 'phi',phi, 'r0',r0, 'rf',rf, ...
    'cpass',cpass, 'broke',broke, 'tf_start2',tfStart2, ...
    'arrays', struct('icount',Aic(1:count), 'bgam',Abg(1:count), ...
                     'gamdeg',Agd(1:count), 'vrx',Avx(1:count), ...
                     'vry',Avy(1:count), 'tf',Atf(1:count), 'pass',Aps(1:count)));
end


% =============================================================================
function s = kepProp(x0,y0,vx0,vy0,T,gmE,h)
%KEPPROP  RK4 on the book's own two-body equations, zarchan.txt:3805-3808.
%   Shares no code with the closed forms, which is the whole reason it is worth
%   having: it FLIES the answer instead of restating the formula that made it.
n  = max(1, ceil(T/h));
hh = T/n;
s  = [x0; y0; vx0; vy0];
for i = 1:n
    k1 = kepRhs(s,           gmE);
    k2 = kepRhs(s+0.5*hh*k1, gmE);
    k3 = kepRhs(s+0.5*hh*k2, gmE);
    k4 = kepRhs(s+    hh*k3, gmE);
    s  = s + (hh/6)*(k1+2*k2+2*k3+k4);
end
end

function d = kepRhs(s, gmE)
r15 = (s(1)^2 + s(2)^2)^1.5;
d = [s(3); s(4); -gmE*s(1)/r15; -gmE*s(2)/r15];
end

function E = addExp(E, name, S)
i = numel(E)+1;
E(i).name=name;  E(i).vrx=S.vrx;  E(i).vry=S.vry;  E(i).tf=S.tf;
E(i).count=S.count;  E(i).gamdeg=S.gamdeg;
end

function S = notRun()
%NOTRUN  Placeholder for a Table 13.1 condition that was not run because the
%   case is off the book's.  NaN, not zero: every row that would read it is
%   skipped, and a zero would print as a result.
S = struct('vrx',NaN,'vry',NaN,'tf',NaN,'count',NaN,'gamdeg',NaN,'v',NaN);
end


% =============================================================================
function K = structuralChecks(S, Sflip, XM, YM, XT, YT, XLONGM, XLONGT, gmE, ...
                              TFdes, ph, twoloops, missOwn)
%STRUCTURALCHECKS  Self-consistency, NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.  Two of the five are
%   DELETE EXPERIMENTS on the two comparisons the transcription could not
%   settle; without them the resolutions in the header would be assertions.
names = {};  oks = [];  detail = {};
DEGRAD = 360./(2.*pi);

% 1  FLY THE ANSWER.  Exact for the routine's own tF, by construction.
names{end+1}  = 'round trip: (vrx,vry) flown for tF lands on the target';
oks(end+1)    = missOwn <= 1.0;
detail{end+1} = sprintf('miss %.4g ft at tF = %.6f s (RK4, h = %g s)', missOwn, S.tf, ph);

% 2  Recover the flight-path angle from the velocity components.  This is what
%    the direction branch is FOR, and it is the cheap half of check 3.  The
%    EXPECTED recovery differs by branch: counterclockwise returns gamdeg
%    itself, clockwise returns 180 - gamdeg (the velocity points the other way
%    round the Earth).  Comparing a clockwise run against gamdeg would light a
%    red row for a perfectly correct answer, so the branch is read here rather
%    than assumed.
ccw    = XLONGT > XLONGM;                        % the listing's own test
gamF   = S.gamdeg/DEGRAD;
vt     = -S.vrx*sin(XLONGM) + S.vry*cos(XLONGM); % local horizontal, ccw
vr     =  S.vrx*cos(XLONGM) + S.vry*sin(XLONGM); % local vertical, outward
gamRec = atan2(vr, vt)*DEGRAD;
if ccw, gamExp = S.gamdeg; else, gamExp = 180 - S.gamdeg; end
dGam = mod(gamRec - gamExp + 180, 360) - 180;    % wrapped, so +/-180 is not a miss
names{end+1}  = 'flight-path angle recovered from (vrx,vry) == the search angle';
oks(end+1)    = abs(dGam) <= 1e-9;
detail{end+1} = sprintf('recovered %.7f deg vs expected %.7f deg (%s branch)', ...
                        gamRec, gamExp, ccwStr(ccw));

% 3  DELETE EXPERIMENT on unresolved comparison #1: force the OTHER branch.
%    tf is unaffected by the branch, so only the velocity components move --
%    which is exactly why nothing inside the search itself would notice.
if ccw
    vrxF = S.v*cos(-pi/2 + gamF + XLONGM);  vryF = S.v*sin(-pi/2 + gamF + XLONGM);
else
    vrxF = S.v*cos(pi/2 - gamF + XLONGM);   vryF = S.v*sin(pi/2 - gamF + XLONGM);
end
sF    = kepProp(XM,YM,vrxF,vryF,S.tf,gmE,ph);
missF = hypot(sF(1)-XT, sF(2)-YT);
names{end+1}  = 'DELETE: the other travel-direction branch -> misses';
oks(end+1)    = missF > 1e6;
detail{end+1} = sprintf('%s instead of %s: miss %.4g ft against %.4g ft correct', ...
                        ccwStr(~ccw), ccwStr(ccw), missF, missOwn);

% 4  DELETE EXPERIMENT on unresolved comparison #2: break on tf < tfdes.
%    Refusing to converge at all counts as "wrong solution" -- that is what the
%    caught error means -- so it is scored the same way and printed verbatim.
names{end+1} = 'DELETE: break test flipped to tf < tfdes -> wrong solution';
if ~isempty(Sflip.threw)
    oks(end+1)    = true;
    detail{end+1} = sprintf('flipped test does not converge at all: %s', Sflip.threw);
else
    oks(end+1)    = abs(Sflip.tf - TFdes) > 100 || Sflip.gamdeg < 0;
    detail{end+1} = sprintf(['flipped test gives tF %.2f s (want %.2f) at ' ...
        'gam %.4f deg, count %d'], Sflip.tf, TFdes, Sflip.gamdeg, Sflip.count);
end

% 5  The refinement pass must actually BRACKET the crossing: its first angle has
%    to be on the low side.  This fails when backoff < step1 (try 'step1',1 with
%    'twoloops',true), which is why the book's own 1-deg experiment removed the
%    second loop rather than keeping it.
if twoloops
    names{end+1}  = 'second pass brackets the crossing (its first tF is below target)';
    oks(end+1)    = S.cpass(2) >= 1 && isfinite(S.tf_start2) && S.tf_start2 <= TFdes;
    detail{end+1} = sprintf('pass-2 first tF %.4f s vs desired %.4f s, %d refined steps', ...
                            S.tf_start2, TFdes, S.cpass(2));
else
    names{end+1}  = 'second pass brackets the crossing [n/a: single-pass run]';
    oks(end+1)    = true;
    detail{end+1} = '''twoloops'' is false, so there is no refinement pass to bracket anything';
end

% Assigned field by field, NOT through struct(...): struct('names',{names})
% with a 1-by-5 cell would build a 1-by-5 STRUCT ARRAY, and adaptGeneric would
% then see five scalar selfchecks with one name each instead of one with five.
K = struct();
K.names  = names;
K.oks    = logical(oks);
K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    j = find(~K.oks,1);  K.first_bad = sprintf('%s (%s)', names{j}, detail{j});
end
end


% =============================================================================
function B = benchmarks(R, E, S2, opts, bookCase, XM, YM, XT, YT)
%BENCHMARKS  Computed vs published.  EVERY book value here was grepped out of
%   zarchan_decoded.txt and its line number is in .source.  Nothing was read off
%   a figure -- the extraction turned every figure into an image link.
%
%   Bands, and where each one comes from:
%     'table-lsd'   one unit in the last digit Table 13.1 actually PRINTS.  The
%                   table is hand-typed and inconsistent about it (rows 1 and 3
%                   round, row 2 truncates), so half a unit would fail a correct
%                   port on row 2.  It is one unit, never more.
%     'count'       exact.  An iteration count has no rounding.
%     'count +/-1'  the nominal row ONLY, and only because the book contradicts
%                   itself there -- see the header.  Named, not hidden.
%     'shape'/'bound'  an ordering claim from prose; no tolerance is possible.
B = emptyBench();
nomCase = bookCase && abs(opts.step1-0.1)<1e-12 && opts.twoloops;
whyOff = sprintf(['Table 13.1 is published for 45/90 deg, alt 0, TF = 1000 s ' ...
    'and the listing''s own constants; this run used %g/%g deg, alt %g/%g n mi, ' ...
    'TF = %g s'], opts.XLONGMDEG, opts.XLONGTDEG, opts.ALTNMM, opts.ALTNMT, opts.TF);
whyNom = sprintf('%s, with step1 = %g deg and twoloops = %d', whyOff, ...
    opts.step1, opts.twoloops);
SRC  = 'Table 13.1, p.280 (zarchan.txt:3913-3915)';
SRC1 = 'Table 13.1 (zarchan.txt:3913), also Fig. 13.4 caption :3885 and text :3889';

% ---- Table 13.1 row 1, NOMINAL --------------------------------------------
B = addVal(B, nomCase, whyNom, 'VRX, nominal (Table 13.1)', -7696, E(1).vrx, ...
    'table-lsd', 1, SRC, 'ft/s', ['book prints "27696": the minus sign is ' ...
    'mis-mapped to the digit 2.  Forced twice over -- gam = 22.2 deg < ' ...
    'xlongm = 45 deg makes vrx negative, and +27696 would put |V| at ' ...
    '33.2 kft/s, a near-escape ellipse that does not return in 1000 s']);
B = addVal(B, nomCase, whyNom, 'VRY, nominal (Table 13.1)', 18329, E(1).vry, ...
    'table-lsd', 1, SRC, 'ft/s', ...
    'the one Table 13.1 velocity cell with no sign damage -- the clean anchor');
B = addVal(B, nomCase, whyNom, 'iterations, nominal (Table 13.1)', 1084, ...
    E(1).count, 'count +/-1', 1, SRC1, '-', ['the listing''s own `count` ' ...
    'reaches 1083: it records only angles whose tF is still below target, and ' ...
    'the answer arrives on the next visit.  The other two rows of this table ' ...
    'match `count` exactly, so the table is inconsistent by one.  The band is ' ...
    'one COUNT and it is named rather than absorbed']);

% ---- Table 13.1 row 2, SECOND LOOP REMOVED ---------------------------------
B = addVal(B, bookCase, whyOff, 'iterations, second loop removed', 335, ...
    E(2).count, 'count', 0, [SRC ', prose :3910 and :3889'], '-', ...
    'EXACT, no band: this is the count row the book is not confused about');
B = addVal(B, bookCase, whyOff, 'tF, second loop removed', 1001, E(2).tf, ...
    'table-lsd', 1, SRC, 's', ['the routine returns the value at the BREAKING ' ...
    'angle, not a converged one -- which is why this cell is 1001, not 1000']);

% ---- Table 13.1 row 3, ONE-DEGREE STEPS ------------------------------------
B = addVal(B, bookCase, whyOff, 'iterations, one-degree steps', 34, E(3).count, ...
    'count', 0, SRC, '-', 'EXACT, no band');
B = addVal(B, bookCase, whyOff, 'VRX, one-degree steps', -7418, E(3).vrx, ...
    'table-lsd', 1, SRC, 'ft/s', ['book prints "27418" -- same minus-sign ' ...
    'damage.  This cell is also what shows row 3 has BOTH loops removed: ' ...
    'keeping the refinement pass gives -7471, fifty times this band away']);
B = addVal(B, bookCase, whyOff, 'tF, one-degree steps', 1014, E(3).tf, ...
    'table-lsd', 1, SRC, 's', ['the second discriminator for "both loops ' ...
    'removed": with the refinement pass kept it is 1011.5, outside this band']);

% ---- SHAPE: the convergence argument, zarchan.txt:3881 ---------------------
% "flight time is smooth and monotonically increasing with increasing
% flight-path angle" -- this is WHY the scan may stop at the first crossing.
% Checked WITHIN each pass: the record as a whole steps backwards at the pass
% boundary by construction, because pass 2 restarts 0.15 deg below where pass 1
% stopped, so checking the concatenation would fail a correct run.
tfA = R.arrays.tf;  psA = R.arrays.pass;  worst = Inf;
for p = unique(psA)'
    d = diff(tfA(psA==p));
    if ~isempty(d), worst = min(worst, min(d)); end
end
B = addShape(B, 'tF increases monotonically with flight-path angle', ...
    'strictly increasing within each scan', ...
    sprintf('smallest step %+.4g s over %d recorded angles', worst, numel(tfA)), ...
    isfinite(worst) && worst > 0, 'p.277 text (zarchan.txt:3881)', ...
    'the property that licenses stopping at the FIRST crossing');

% ---- BOUND: exact for its own flight time, approximate for the desired one --
% zarchan.txt:3910 -- "the velocities are exact for a 1001-s flight but
% approximate for a Lambert solution requiring a 1000-s flight" -- and :3883,
% where the refinement loop "is required to get extremely precise answers".
if bookCase
    o1 = kepProp(XM,YM,S2.vrx,S2.vry,S2.tf,   opts.gm,opts.prop_h);
    d1 = kepProp(XM,YM,S2.vrx,S2.vry,opts.TF, opts.gm,opts.prop_h);
    mOwn1 = hypot(o1(1)-XT, o1(2)-YT);
    mDes1 = hypot(d1(1)-XT, d1(2)-YT);
    B = addBound(B, 'exact for its own tF, approximate for the desired tF', ...
        'one-loop answer exact at 1001 s but not at 1000 s; refinement "required"', ...
        sprintf('one loop %.3g ft at own tF / %.0f ft at %g s; two loops %.3g / %.1f ft', ...
                mOwn1, mDes1, opts.TF, R.miss_at_tf_ft, R.miss_at_tfdes_ft), ...
        mOwn1 <= 1.0 && R.miss_at_tf_ft <= 1.0 && ...
        mDes1 > 100*max(R.miss_at_tfdes_ft, 1e-9), ...
        'p.280 text (zarchan.txt:3910), p.277 (:3883)', 'ft', ...
        ['flown, not asserted: both answers hit the target at their OWN tF, ' ...
         'and the refinement pass is what buys the DESIRED one']);
else
    B = addSkip(B, 'exact for its own tF, approximate for the desired tF', ...
        'exact at 1001 s, approximate at 1000 s', whyOff, ...
        'p.280 text (zarchan.txt:3910)', 'ft');
end
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly; this is the
%   schema benchSuite's adaptGeneric consumes, and it is deliberately strict.
B = struct('what',{},'kind',{},'book',{},'book_str',{},'ours',{},'ours_str',{}, ...
           'pct',{},'band',{},'tol',{},'ok',{},'skipped',{},'why',{}, ...
           'source',{},'units',{},'detail',{});
end

function B = addVal(B, gate, why, what, book, ours, band, absTol, source, units, detail)
%ADDVAL  Every value row here is a cell of a PRINTED TABLE, so the tolerance is
%   ABSOLUTE -- one unit in the last printed digit -- and .tol carries the
%   relative equivalent so the board's percentage column stays honest.
%
%   GATE carries the "is this still the book's case" test, so that each row is
%   written ONCE instead of as an if/else pair.  A false gate produces a SKIP,
%   never a pass: a check that could not run is a different thing from one that
%   succeeded, and the two must stay distinguishable on the board.
if ~gate
    B = addSkip(B, what, sprintf('%.6g', book), why, source, units);
    return
end
i = numel(B)+1;
B(i).what=what;    B(i).kind='value';
B(i).book=book;    B(i).book_str=sprintf('%.6g', book);
B(i).ours=ours;    B(i).ours_str=sprintf('%.7g', ours);
B(i).band=band;    B(i).tol=absTol/max(abs(book), realmin);
B(i).units=units;  B(i).source=source;  B(i).detail=detail;
if ~isfinite(ours) || book == 0
    B(i).pct=NaN;  B(i).ok=false;  B(i).skipped=true;
    B(i).why='the computed value is not finite (or the book value is zero)';
    return
end
B(i).pct = 100*(ours/book - 1);
B(i).ok  = abs(ours - book) <= absTol;
B(i).skipped=false;  B(i).why='';
end

function B = addShape(B, what, bookStr, oursStr, ok, source, detail)
i = numel(B)+1;
B(i).what=what;  B(i).kind='shape';
B(i).book=NaN;   B(i).book_str=bookStr;
B(i).ours=NaN;   B(i).ours_str=oursStr;
B(i).pct=NaN;    B(i).band='shape';  B(i).tol=NaN;
B(i).ok=logical(ok);  B(i).skipped=false;  B(i).why='';
B(i).source=source;   B(i).units='-';    B(i).detail=detail;
end

function B = addBound(B, what, bookStr, oursStr, ok, source, units, detail)
i = numel(B)+1;
B(i).what=what;  B(i).kind='bound';
B(i).book=NaN;   B(i).book_str=bookStr;
B(i).ours=NaN;   B(i).ours_str=oursStr;
B(i).pct=NaN;    B(i).band='bound';  B(i).tol=NaN;
B(i).ok=logical(ok);  B(i).skipped=false;  B(i).why='';
B(i).source=source;   B(i).units=units;  B(i).detail=detail;
end

function B = addSkip(B, what, bookStr, why, source, units)
%ADDSKIP  A check that COULD NOT RUN.  Never a pass: a check that did not happen
%   is a different thing from one that succeeded.
i = numel(B)+1;
B(i).what=what;  B(i).kind='skip';
B(i).book=NaN;   B(i).book_str=bookStr;
B(i).ours=NaN;   B(i).ours_str='--';
B(i).pct=NaN;    B(i).band='skip';  B(i).tol=NaN;
B(i).ok=false;   B(i).skipped=true;  B(i).why=why;
B(i).source=source;  B(i).units=units;  B(i).detail='';
end


% =============================================================================
function s = render(R, E, ev)
L = {};
L{end+1} = 'Zarchan Listing 13.1 -- LAMBERT ROUTINE USING BRUTE FORCE APPROACH';
L{end+1} = '*** UNITS ARE FEET, ft/s, SECONDS, DEGREES.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf('   missile %g deg / %g n mi   target %g deg / %g n mi   TF = %g s', ...
    R.p.XLONGMDEG, R.p.ALTNMM, R.p.XLONGTDEG, R.p.ALTNMT, R.p.TF);
L{end+1} = sprintf('   scan %g deg then %g deg (back off %g, span %g); two loops = %d', ...
    R.p.step1, R.p.step2, R.p.backoff, R.p.span2, R.p.twoloops);
L{end+1} = '';
L{end+1} = sprintf('SOLUTION  vrx = %.4f ft/s   vry = %.4f ft/s   |V| = %.3f ft/s', ...
    R.vrx, R.vry, R.v);
L{end+1} = sprintf('          gamma = %.6f deg   phi = %.4f deg   lambda = %.6f  (< 2 = elliptical, :3842)', ...
    R.gamdeg, R.phi_deg, R.lambda);
L{end+1} = sprintf('          tF = %.6f s   count = %d recorded iterations', R.tf, R.count);
L{end+1} = sprintf('          flown back: miss %.4g ft at its own tF, %.3f ft at the desired %g s', ...
    R.miss_at_tf_ft, R.miss_at_tfdes_ft, R.p.TF);
L{end+1} = '';
L{end+1} = 'TABLE 13.1 ACCURACY EXPERIMENTS   (the book''s VRX column is sign-damaged; see header)';
L{end+1} = sprintf('%-22s %12s %12s %11s %8s', 'Condition','VRX','VRY','tF','Iters');
bk = [-7696 18329 1000 1084; -7668 18332 1001 335; -7418 18360 1014 34];
for k = 1:numel(E)
    L{end+1} = sprintf('%-22s %12.4f %12.4f %11.4f %8d', ...
        E(k).name, E(k).vrx, E(k).vry, E(k).tf, E(k).count);
    if k <= size(bk,1)
        L{end+1} = sprintf('%-22s %12g %12g %11g %8g', '   book', ...
            bk(k,1), bk(k,2), bk(k,3), bk(k,4));
    end
end
L{end+1} = '';
L{end+1} = 'STRUCTURAL CHECKS (self-consistency, NOT book values)';
for i = 1:numel(R.selfcheck.names)
    L{end+1} = sprintf('  [%s] %-62s %s', tick(R.selfcheck.oks(i)), ...
        R.selfcheck.names{i}, R.selfcheck.detail{i});
end
L{end+1} = '';
L{end+1} = 'BENCHMARKS vs the book';
for i = 1:numel(R.bench)
    b = R.bench(i);
    if b.skipped
        L{end+1} = sprintf('  [--] %-48s book %-8s  %s', b.what, b.book_str, b.why);
    elseif isnan(b.book)
        L{end+1} = sprintf('  [%s] %-48s want %s / got %s', tick(b.ok), b.what, ...
            b.book_str, b.ours_str);
    else
        L{end+1} = sprintf('  [%s] %-48s book %-8s ours %-11s (%+.4f%%, band %s)', ...
            tick(b.ok), b.what, b.book_str, b.ours_str, b.pct, b.band);
    end
end
if any(ev)
    L{end+1} = sprintf('  %d of %d evaluated rows pass; %d not evaluated.', ...
        sum([R.bench(ev).ok]), sum(ev), sum(~ev));
else
    L{end+1} = '  NOTHING WAS EVALUATED -- see R.verdict.';
end
s = strjoin(L, newline);
end

function c = tick(ok)
if ok, c = 'ok'; else, c = 'XX'; end
end

function s = ccwStr(ccw)
if ccw, s = 'counterclockwise'; else, s = 'clockwise'; end
end
