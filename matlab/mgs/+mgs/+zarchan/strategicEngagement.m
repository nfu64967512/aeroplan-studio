function R = strategicEngagement(opts)
%MGS.ZARCHAN.STRATEGICENGAGEMENT  Zarchan Listing 28.4 -- three-dimensional
%   strategic missile-target engagement (Monte Carlo -> Las Vegas, intercepted
%   from Atlantic City), with the LAMBERT3D and KEPLER1 subroutines it calls.
%
%   R = mgs.zarchan.strategicEngagement()
%   R = mgs.zarchan.strategicEngagement('PREDERR_NMI', 0, 'assert', false)
%   R = mgs.zarchan.strategicEngagement('aim_rotation', 'TFTOT')
%
%   Port of zarchan.txt:10527-10574 (driver), :10428 (LAMBERT3D, printed under
%   Listing 28.2), :10486 (KEPLER1, printed under Listing 28.3), :10423
%   (distance3d).  Units are the book's: FEET and ft/s for every state,
%   ft/s^2 for acceleration with g = 32.2, altitudes in n mi (6076 ft), and
%   KEPLER1 alone in km / km/s.  Nothing is converted to SI anywhere.
%
% ---------------------------------------------------------------------------
%   THE SIX UNRESOLVED COMPARISONS -- WHICH WAY, AND WHY
%
%   In this book's OCR "," is "<" and "." is ">".  listing_28_04.m leaves six
%   variable-to-variable tests unresolved.  All six resolve from the physics of
%   an impulsively launched missile sitting on a rotating Earth:
%
%     1  predictor,  "if T,TLAUNCH & QBOOSTM==1"  ->  T <  TLAUNCH.  The branch
%        body is "XM=XM; YM=YM; ..." -- it HOLDS the missile.  Holding it
%        AFTER launch would mean the missile never flies.
%     2  corrector,  "if T,TLAUNCH & QBOOSTM==1"  ->  T <  TLAUNCH.  Same test,
%        and its body re-anchors the missile to its ECEF launch site rotated by
%        W*T.  Re-anchoring after launch would teleport it home every step.
%     3  homing gate, "if TGO < 200. & T.(TLAUNCH+50.)" -> T > TLAUNCH+50.
%        The 50 s is a settling margin after boost; "<" would run the homing
%        law only in the 250 s BEFORE the missile exists.
%     4  missile accel, "if T.TLAUNCH" -> T > TLAUNCH.  The branch adds gravity
%        and the guidance command; the else sets all three to zero "so that the
%        missile remains on the ground" (zarchan.txt:10524, verbatim intent).
%     5  clamp, "if AXMGUID.XNCLIM, AXMGUID=XNCLIM"  ->  AXMGUID > XNCLIM (and
%        the three axes identically).  The elseif is already "< -XNCLIM" in the
%        transcription, so "<" here would make the two branches overlap and
%        peg EVERY command at +-XNCLIM.  Structural check 5 measures that: a
%        flipped clamp puts sqrt(3)*161/32.2 = 8.66 g on every sample, against
%        a nominal peak the book calls 0.1 g.
%     6  LAMBERT3D's search, "if T.TF ... G_MAX=GAMMA" -> T > TF.  Same secant
%        bracket as Listing 13.2, where MGS.ZARCHAN.LAMBERTEFFICIENT confirmed
%        the identical test arithmetically against the published iterate
%        sequence of Table 13.2.  Not re-derived here; that port is the
%        authority and this one inherits its resolution.
%
%   Also repaired, forced, not judgement calls: "while VC.0" -> "while VC > 0"
%   (VC is the closing velocity; the run ends at closest approach), "if
%   RTM.5000." -> RTM > 5000 (the fine step 1e-5 s belongs to the endgame, not
%   to 1000 s of cruise), "if S.=9.9999" -> S >= 9.9999 (S is reset to 0 on
%   every record), and in KEPLER1 "if ALP0,0." -> ALP0 < 0 (that branch takes
%   sqrt(-Y) and uses cosh/sinh, i.e. it IS the hyperbolic case).
%   The trailing prose block in the transcription is the book's own paragraph
%   about the nominal case (zarchan.txt:10575); dropped, and it is where four
%   of the benchmarks below come from.
%
% ---------------------------------------------------------------------------
%   TWO BOOK DEFECTS THIS PORT REPRODUCES, AND REPORTS
%
%   A  DISTRTNM IS KILOMETRES.  distance3d returns "DISTKM = A*BETA/3280",
%      i.e. arc length in km, and Listing 28.4 stores it in DISTRTNM and labels
%      the plot "Downrange (Nmi)".  Altitude (/6076) really is n mi, so
%      Fig. 28.10 plots km against n mi on one pair of axes.  The PROSE is in
%      true n mi: Monte Carlo to Las Vegas is "nearly 5000 n miles"
%      (zarchan.txt:10457) = 9418 km = 5084 n mi, not 9418 n mi.  This port
%      returns BOTH -- R.dist_t_km is the listing's own number, R.*_nmi is the
%      converted one -- and scores the n mi value, because that is what the
%      sentence being scored says.
%   B  THE AIM POINT IS ROTATED BY THE WRONG TIME.  The listing sets
%      TFTOT = 2000 (the target's total flight) and TF = 1000 (the intercept
%      time), then writes "XLONGF = XLONGF - W*TF" before calling LAMBERT3D
%      with TFTOT.  Listing 28.2, the same trajectory, uses its single TF for
%      both.  Rotating Las Vegas forward by 1000 s instead of 2000 s moves the
%      aim point 4.17 deg (~360 km) and shortens the whole trajectory a few
%      percent.  Transcribed AS PRINTED; 'aim_rotation','TFTOT' runs the other
%      reading, and structural check 4 reports the size of the difference
%      instead of hiding it.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Every book value below was grepped out of zarchan_decoded.txt;
%   the line number is in each row's .source.  Nothing was read off a figure --
%   the extraction turned every figure into an image link.
%
%   1-3  THE LAMBERT LEG, exactly.  Listing 28.3's first line (:10475) is
%        commented "Obtained from output of Listing C28L2" and prints the
%        target's ECI state to 15 digits: XTD = 996.773566434768,
%        YTD = -14954.8124604715, ZTD = 17528.1931768263 ft/s.  Those ARE
%        LAMBERT3D's output for this same Monte Carlo -> Las Vegas problem, in
%        Listing 28.2's convention (aim point rotated by the full 2000 s), so
%        rows 1-3 solve Lambert that way and compare, band 1e-5 relative.  It
%        is a published 15-digit check on the subroutine that aims the target
%        and, 800 s later, the missile.
%   4    THE KEPLER LEG.  Table 28.1 (:10467-10470) propagates those ICs to
%        t = 1000 s: 2547.4148, -3583.1074, 6758.2384 km.  Scored on x, y and z
%        in .detail.  ⚠️ MGS.ZARCHAN.KEPLERVSDIRECT (Listing 28.3) OWNS THIS
%        COMPARISON; repeated here once only because Listing 28.4 CALLS KEPLER1
%        to build the aim point, and a mis-wired Kepler moves the intercept
%        while every other row still looks plausible.  Corroboration that the
%        table and the prose describe one trajectory: |r| = 8062.4 km minus
%        REX = 6380 km is 1682 km = 908 n mi, i.e. the "target apogee is
%        approximately 900 n miles" of :10457.
%   5    HOMING STARTS AT 800 s (:10575, "200 s before intercept"), scored as
%        the exact T at which TGO = RTM/VC first drops below 200 s, band 2%.
%        A red row here would not mean the port is wrong -- it would mean
%        RTM/VC is a worse t_go 200 s out than the book's sentence assumes.
%   6    INTERCEPT ALTITUDE about 900 n mi (:10575, corroborated at :10457).
%   7    INTERCEPT DOWNRANGE about 2500 n mi (:10575), in TRUE n mi (defect A).
%        Band on 6 and 7 is 10%: both words are "about"/"approximately" and
%        both are figure readings, so a 2-significant-figure band would be
%        pretending.  10% is the loosest band that still separates the right
%        answer from the two wrong ones this listing can produce -- the km
%        value (about 4710) and the full range (5084).
%   8    PEAK COMMANDED ACCELERATION 0.1 g (:10575).  ONE significant figure,
%        so the honest band is +-0.05 g = 50% and the row catches only an
%        order-of-magnitude error; row 9 carries the weight.  Scored on the
%        10 s record, which is what Fig. 28.11 plots on axes the listing pins
%        at [0 1000 0 .5]; the instantaneous peak over every step happens in
%        the last milliseconds, is off that scale, and is reported not scored.
%   9    SHAPE: "the acceleration command was zero before that" (:10575).
%        Exactly zero, not small -- the else branch assigns 0.
%   10   INFO: the listing's last statement is the bare "RTM DELV".  Chapter 28
%        publishes neither number, so both are reported and neither adjudicated.
%
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED -- this file has never
%   been executed; MATLAB is the caller's. ***  From Table 28.1's own numbers:
%   intercept altitude near 908 n mi, downrange near 2540 n mi (half of the
%   5084 n mi great circle), homing on near 800 s, intercept near 1000 s.  The
%   guided miss should be small (the endgame step is 1e-5 s, about 0.25 ft of
%   travel) and the guidance-deleted miss of order the 60,760 ft prediction
%   error.  'assert' chooses the reaction to a verdict and nothing else --
%   false WARNS and still returns R complete, so a red row can be diagnosed.
%
%   See also MGS.ZARCHAN.LAMBERTEFFICIENT (Listing 13.2 -- owns the secant
%   Lambert search), MGS.ZARCHAN.KEPLERVSDIRECT (Listing 28.3 -- owns Table
%   28.1), MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 28.4 line 1, verbatim.  Degrees, seconds, feet.
    opts.XLONGTDEG (1,1) double {mustBeFinite} = 7.42        % Monte Carlo
    opts.XLATTDEG  (1,1) double {mustBeFinite} = 43.75
    opts.XLONGFDEG (1,1) double {mustBeFinite} = -115.136    % Las Vegas
    opts.XLATFDEG  (1,1) double {mustBeFinite} = 36.175
    opts.XLONGMDEG (1,1) double {mustBeFinite} = -74.423     % Atlantic City
    opts.XLATMDEG  (1,1) double {mustBeFinite} = 39.364
    opts.TLAUNCH   (1,1) double {mustBeFinite, mustBeNonnegative} = 200.0
    opts.TFTOT     (1,1) double {mustBeFinite, mustBePositive}    = 2000.0
    opts.TF        (1,1) double {mustBeFinite, mustBePositive}    = 1000.0
    opts.PREDERR_NMI (1,1) double {mustBeFinite} = 10.0      % listing: 10.*6076.
    opts.XNCLIM    (1,1) double {mustBeFinite, mustBePositive}    = 161.0
    opts.XNP       (1,1) double {mustBeFinite, mustBePositive}    = 3.0
    % Defect B.  'TF' is what the listing prints; 'TFTOT' is what Listing 28.2
    % does with its single flight time.  Never silently switched.
    opts.aim_rotation (1,1) string {mustBeMember(opts.aim_rotation, ["TF","TFTOT"])} = "TF"
    opts.H_far  (1,1) double {mustBeFinite, mustBePositive} = 0.01      % RTM > 5000 ft
    opts.H_near (1,1) double {mustBeFinite, mustBePositive} = 0.00001   % endgame
    opts.samp   (1,1) double {mustBeFinite, mustBePositive} = 9.9999    % listing's
    % The delete experiment (structural check 3) flies the engagement a SECOND
    % time with the homing law removed.  It roughly doubles the cost of this
    % file and it is the only check that proves the guidance channel is wired
    % to anything, so it defaults ON.
    opts.delete_experiment (1,1) logical = true
    opts.max_steps (1,1) double {mustBeFinite, mustBePositive} = 4e6
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.assert    (1,1) logical = true
end

P = struct();
P.A = 2.0926E7;  P.GM = 1.4077E16;  P.W = -6.283185/86400.;
P.XLONGTDEG = opts.XLONGTDEG;  P.XLATTDEG = opts.XLATTDEG;
P.XLONGFDEG = opts.XLONGFDEG;  P.XLATFDEG = opts.XLATFDEG;
P.XLONGMDEG = opts.XLONGMDEG;  P.XLATMDEG = opts.XLATMDEG;
P.TLAUNCH = opts.TLAUNCH;  P.TFTOT = opts.TFTOT;  P.TF = opts.TF;
P.PREDERR = opts.PREDERR_NMI*6076.;
P.XNCLIM = opts.XNCLIM;  P.XNP = opts.XNP;
P.SWITCH1 = 0;  P.SWITCHM = 0;
P.H_far = opts.H_far;  P.H_near = opts.H_near;  P.samp = opts.samp;
P.max_steps = opts.max_steps;
% Defect B: which time the aim-point longitude is advanced by.
if opts.aim_rotation == "TF", P.TROT = P.TF; else, P.TROT = P.TFTOT; end

E = flyEngagement(P, true);

ED = [];
if opts.delete_experiment
    ED = flyEngagement(P, false);
end

% ---- the two subroutine cross-checks, run stand-alone on the BOOK'S OWN
%      configuration (Listing 28.2's convention), not on this run's states.
PB = P;  PB.TROT = P.TFTOT;
[lamX, lamY, lamZ] = bookLambertLeg(PB);
% Book ICs, zarchan.txt:10475, fed to KEPLER1 exactly as Listing 28.3 does.
X0book = [14990432.9744621, 1952093.10305573, 14469752.1663352, ...
          996.773566434768, -14954.8124604715, 17528.1931768263]/3280.;
X1book = kepler1(X0book, 0., 1000.);

R = struct();
R.t          = E.t;
R.dist_t_km  = E.dist_t_km;    % the listing's OWN DISTRTNM -- km, see defect A
R.dist_t_nmi = E.dist_t_km*3280./6076.;
R.alt_t_nmi  = E.alt_t_nmi;
R.dist_m_km  = E.dist_m_km;
R.dist_m_nmi = E.dist_m_km*3280./6076.;
R.alt_m_nmi  = E.alt_m_nmi;
R.acc_g      = E.acc_g;
R.longt_deg  = E.longt;  R.latt_deg = E.latt;
R.longm_deg  = E.longm;  R.latm_deg = E.latm;
R.miss_ft        = E.miss_ft;
R.delv_fps       = E.delv_fps;
R.t_intercept    = E.t_end;
R.t_homing_on    = E.t_guid_on;
R.peak_acc_g     = E.amax_plot_g;      % 10 s record, what Fig. 28.11 plots
R.peak_acc_all_g = E.amax_step_g;      % every step, incl. the endgame
R.alt_intercept_nmi  = E.alt_t_nmi(end);
R.dist_intercept_nmi = E.dist_t_km(end)*3280./6076.;
R.target_v0    = [E.XTD0 E.YTD0 E.ZTD0];
R.aim_point_ft = [E.XTF E.YTF E.ZTF];
R.lambert_book = [lamX lamY lamZ];
R.kepler_book  = X1book;
R.deleted      = ED;
R.p            = P;
R.p.aim_rotation = char(opts.aim_rotation);
R.units = ['ft, ft/s, ft/s^2 (g = 32.2 ft/s^2); altitude in n mi (6076 ft); ' ...
           'the listing''s DISTRTNM/DISTRMNM are KM (see defect A); ' ...
           'KEPLER1 works in km, km/s.'];

R.selfcheck = structuralChecks(E, ED, P, lamX, lamY, lamZ);

if opts.benchmark
    R.bench = benchmarks(R, E, P, opts, lamX, lamY, lamZ, X1book);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    R.pass  = false;      % NOT CHECKED is not the same thing as fine.
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false, so nothing was compared against the book.';
elseif ~any(ev)
    msg = 'not one book value could be evaluated (off-book configuration).';
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
        error('mgs:zarchan:strategicEngagement:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:strategicEngagement:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [VRX, VRY, VRZ] = bookLambertLeg(P)
%BOOKLAMBERTLEG  The target's Lambert leg alone, in Listing 28.2's convention.
%   Feeds benchmarks 1-3.  Constants arrive as arguments; MATLAB local
%   functions cannot see the caller's workspace.
XLONGF = P.XLONGFDEG/57.3 - P.W*P.TROT;   XLATF = P.XLATFDEG/57.3;
XLONGT = P.XLONGTDEG/57.3;                XLATT = P.XLATTDEG/57.3;
XF = P.A*cos(XLATF)*cos(XLONGF);  YF = P.A*cos(XLATF)*sin(XLONGF);
ZF = P.A*sin(XLATF);
XT = P.A*cos(XLATT)*cos(XLONGT);  YT = P.A*cos(XLATT)*sin(XLONGT);
ZT = P.A*sin(XLATT);
[VRX, VRY, VRZ] = lambert3d(XT, YT, ZT, P.TFTOT, XF, YF, ZF, P.SWITCH1);
end


% =============================================================================
function E = flyEngagement(P, guidOn)
%FLYENGAGEMENT  Listing 28.4, zarchan.txt:10527-10574, on the book's own Heun
%   scheme.  guidOn == false is the DELETE EXPERIMENT: the homing law is
%   removed and nothing else changes.
A = P.A;  GM = P.GM;  W = P.W;  TF = P.TF;  TLAUNCH = P.TLAUNCH;
XNCLIM = P.XNCLIM;  XNP = P.XNP;
QBOOSTM = 1;

XLONGF = P.XLONGFDEG/57.3;  XLATF = P.XLATFDEG/57.3;
XLONGT = P.XLONGTDEG/57.3;  XLATT = P.XLATTDEG/57.3;
XLONGM = P.XLONGMDEG/57.3;  XLATM = P.XLATMDEG/57.3;
XLATMDEGIC = P.XLATMDEG;  XLONGMDEGIC = P.XLONGMDEG;

XLONGF = XLONGF - W*P.TROT;              % listing: -W*TF.  See defect B.
XF = A*cos(XLATF)*cos(XLONGF);  YF = A*cos(XLATF)*sin(XLONGF);  ZF = A*sin(XLATF);
XT = A*cos(XLATT)*cos(XLONGT);  YT = A*cos(XLATT)*sin(XLONGT);  ZT = A*sin(XLATT);
[XTD, YTD, ZTD] = lambert3d(XT, YT, ZT, P.TFTOT, XF, YF, ZF, P.SWITCH1);
XTINIT = XT;  YTINIT = YT;  ZTINIT = ZT;
E.XTD0 = XTD;  E.YTD0 = YTD;  E.ZTD0 = ZTD;

XM = A*cos(XLATM)*cos(XLONGM);  YM = A*cos(XLATM)*sin(XLONGM);  ZM = A*sin(XLATM);
XMD =  A*W*cos(XLATM)*sin(XLONGM);
YMD = -A*W*cos(XLATM)*cos(XLONGM);
ZMD = 0.;

RTM1 = XT-XM;  RTM2 = YT-YM;  RTM3 = ZT-ZM;
RTM = sqrt(RTM1^2 + RTM2^2 + RTM3^2);
VTM1 = XTD-XMD;  VTM2 = YTD-YMD;  VTM3 = ZTD-ZMD;
VC = -(RTM1*VTM1 + RTM2*VTM2 + RTM3*VTM3)/RTM;
if ~(VC > 0)
    error('mgs:zarchan:strategicEngagement:noclosure', ...
        ['VC = %g ft/s at t = 0, so "while VC > 0" never runs and the ' ...
         'engagement returns an empty record that would read as a converged ' ...
         'result.  The target is not closing on the launch site.'], VC);
end

% Intercept-point prediction: KEPLER1 on the target's own state, TF s ahead.
X0 = [XT YT ZT XTD YTD ZTD]/3280.;
X1 = kepler1(X0, 0., TF);
XTF = X1(1)*3280.;  YTF = X1(2)*3280.;  ZTF = X1(3)*3280.;
XTF = XTF + P.PREDERR;                    % the deterministic prediction error
E.XTF = XTF;  E.YTF = YTF;  E.ZTF = ZTF;

T = 0.;  S = 0.;  DELV = 0.;
ALTNM = (sqrt(XT^2+YT^2+ZT^2) - A)/6076.;
% Derivatives exist before the loop so the FLAG==1 branch cannot read a stale
% or undefined value on any path.
XTDD = 0.; YTDD = 0.; ZTDD = 0.;  XMDD = 0.; YMDD = 0.; ZMDD = 0.;  DELVD = 0.;
AXMGUID = 0.; AYMGUID = 0.; AZMGUID = 0.;
TGO = RTM/VC;   % defensive; recomputed every evaluation inside the loop

nmax = ceil(3*P.TFTOT/P.samp) + 64;
rec = zeros(nmax, 10);  nrec = 0;
t_guid_on = NaN;  amax_step = 0;  nclamp = 0;  nsteps = 0;  timeout = false;

while VC > 0
    if RTM > 5000., H = P.H_far; else, H = P.H_near; end

    XTOLD=XT; YTOLD=YT; ZTOLD=ZT; XTDOLD=XTD; YTDOLD=YTD; ZTDOLD=ZTD;
    XMOLD=XM; YMOLD=YM; ZMOLD=ZM; XMDOLD=XMD; YMDOLD=YMD; ZMDOLD=ZMD;
    DELVOLD = DELV;
    STEP = 1;  FLAG = 0;

    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            XT=XT+H*XTD;  YT=YT+H*YTD;  ZT=ZT+H*ZTD;
            XTD=XTD+H*XTDD;  YTD=YTD+H*YTDD;  ZTD=ZTD+H*ZTDD;
            if T < TLAUNCH && QBOOSTM == 1
                % "XM=XM; YM=YM; ..." -- the missile is held on the ground.
            else
                XM=XM+H*XMD;  YM=YM+H*YMD;  ZM=ZM+H*ZMD;
                XMD=XMD+H*XMDD;  YMD=YMD+H*YMDD;  ZMD=ZMD+H*ZMDD;
            end
            DELV = DELV + H*DELVD;
            T = T + H;                    % INSIDE the inner loop, exactly once
        end

        TEMPBOTT = (XT^2 + YT^2 + ZT^2)^1.5;
        XTDD = -GM*XT/TEMPBOTT;  YTDD = -GM*YT/TEMPBOTT;  ZTDD = -GM*ZT/TEMPBOTT;
        ALTNM = (sqrt(XT^2+YT^2+ZT^2) - A)/6076.;
        RTM1 = XT-XM;  RTM2 = YT-YM;  RTM3 = ZT-ZM;
        VTM1 = XTD-XMD;  VTM2 = YTD-YMD;  VTM3 = ZTD-ZMD;
        RTM = sqrt(RTM1^2 + RTM2^2 + RTM3^2);
        VC  = -(RTM1*VTM1 + RTM2*VTM2 + RTM3*VTM3)/RTM;
        TGO = RTM/VC;

        if guidOn && TGO < 200. && T > (TLAUNCH + 50.)
            ZEM1 = RTM1 + VTM1*TGO;  ZEM2 = RTM2 + VTM2*TGO;  ZEM3 = RTM3 + VTM3*TGO;
            ZEMDOTRTM = (ZEM1*RTM1 + ZEM2*RTM2 + ZEM3*RTM3)/RTM;
            ZEMPER1 = ZEM1 - ZEMDOTRTM*RTM1/RTM;
            ZEMPER2 = ZEM2 - ZEMDOTRTM*RTM2/RTM;
            ZEMPER3 = ZEM3 - ZEMDOTRTM*RTM3/RTM;
            AXMGUID = XNP*ZEMPER1/(TGO^2);
            AYMGUID = XNP*ZEMPER2/(TGO^2);
            AZMGUID = XNP*ZEMPER3/(TGO^2);
            if isnan(t_guid_on), t_guid_on = T; end
        else
            AXMGUID = 0.;  AYMGUID = 0.;  AZMGUID = 0.;
        end

        hit = 0;
        if AXMGUID >  XNCLIM, AXMGUID =  XNCLIM; hit = 1;
        elseif AXMGUID < -XNCLIM, AXMGUID = -XNCLIM; hit = 1; end
        if AYMGUID >  XNCLIM, AYMGUID =  XNCLIM; hit = 1;
        elseif AYMGUID < -XNCLIM, AYMGUID = -XNCLIM; hit = 1; end
        if AZMGUID >  XNCLIM, AZMGUID =  XNCLIM; hit = 1;
        elseif AZMGUID < -XNCLIM, AZMGUID = -XNCLIM; hit = 1; end
        nclamp = nclamp + hit;

        if T > TLAUNCH
            TEMPBOTM = (XM^2 + YM^2 + ZM^2)^1.5;
            XMDD = -GM*XM/TEMPBOTM + AXMGUID;
            YMDD = -GM*YM/TEMPBOTM + AYMGUID;
            ZMDD = -GM*ZM/TEMPBOTM + AZMGUID;
        else
            XMDD = 0.;  YMDD = 0.;  ZMDD = 0.;
        end
        DELVD = sqrt(AXMGUID^2 + AYMGUID^2 + AZMGUID^2);
        amax_step = max(amax_step, DELVD/32.2);
        FLAG = 1;
    end

    XT = .5*(XTOLD+XT+H*XTD);  YT = .5*(YTOLD+YT+H*YTD);  ZT = .5*(ZTOLD+ZT+H*ZTD);
    XTD = .5*(XTDOLD+XTD+H*XTDD);  YTD = .5*(YTDOLD+YTD+H*YTDD);
    ZTD = .5*(ZTDOLD+ZTD+H*ZTDD);
    if T < TLAUNCH && QBOOSTM == 1
        % Re-anchor to the ECEF launch site, rotated: W is NEGATIVE here, so
        % "-W*T" advances the longitude eastward.
        XM  = A*cos(XLATMDEGIC/57.3)*cos(XLONGMDEGIC/57.3 - W*T);
        YM  = A*cos(XLATMDEGIC/57.3)*sin(XLONGMDEGIC/57.3 - W*T);
        ZM  = A*sin(XLATMDEGIC/57.3);
        XMD =  A*W*cos(XLATMDEGIC/57.3)*sin(XLONGMDEGIC/57.3 - W*T);
        YMD = -A*W*cos(XLATMDEGIC/57.3)*cos(XLONGMDEGIC/57.3 - W*T);
        ZMD = 0.;
    else
        XM = .5*(XMOLD+XM+H*XMD);  YM = .5*(YMOLD+YM+H*YMD);  ZM = .5*(ZMOLD+ZM+H*ZMD);
        XMD = .5*(XMDOLD+XMD+H*XMDD);  YMD = .5*(YMDOLD+YMD+H*YMDD);
        ZMD = .5*(ZMDOLD+ZMD+H*ZMDD);
    end
    DELV = .5*(DELVOLD + DELV + H*DELVD);

    TGOM = TF - T;
    if T >= TLAUNCH && QBOOSTM == 1
        [XMD, YMD, ZMD] = lambert3d(XM, YM, ZM, TGOM, XTF, YTF, ZTF, P.SWITCHM);
        QBOOSTM = 0;
        XMDOLD = XMD;  YMDOLD = YMD;  ZMDOLD = ZMD;  %#ok<NASGU> (listing's line)
    end

    S = S + H;
    if S >= P.samp
        S = 0.;
        nrec = nrec + 1;
        rec(nrec,:) = sampleRow(XT,YT,ZT,XM,YM,ZM,W,T,A,XTINIT,YTINIT,ZTINIT, ...
                                ALTNM, AXMGUID, AYMGUID, AZMGUID);
    end

    nsteps = nsteps + 1;
    if nsteps >= P.max_steps, timeout = true; break; end
end

% The listing records one final unconditional sample after the loop.
nrec = nrec + 1;
rec(nrec,:) = sampleRow(XT,YT,ZT,XM,YM,ZM,W,T,A,XTINIT,YTINIT,ZTINIT, ...
                        ALTNM, AXMGUID, AYMGUID, AZMGUID);
rec = rec(1:nrec,:);

E.t         = rec(:,1);
E.dist_t_km = rec(:,2);   E.alt_t_nmi = rec(:,3);
E.dist_m_km = rec(:,4);   E.alt_m_nmi = rec(:,5);
E.acc_g     = rec(:,6);
E.longt     = rec(:,7);   E.latt = rec(:,8);
E.longm     = rec(:,9);   E.latm = rec(:,10);
E.miss_ft   = RTM;
E.delv_fps  = DELV;
E.t_end     = T;
E.t_guid_on = t_guid_on;
E.amax_step_g = amax_step;
% What Fig. 28.11 plots: the 10 s record, EXCLUDING the terminal sample that
% the listing appends after the loop (it sits at t_go -> 0, off the axis the
% listing pins at [0 1000 0 .5]).
if nrec > 1
    E.amax_plot_g = max(rec(1:end-1,6));
else
    E.amax_plot_g = rec(1,6);
end
E.acc_terminal_g = rec(end,6);
E.nclamp = nclamp;  E.nsteps = nsteps;  E.timeout = timeout;
E.nrec = nrec;
end


% =============================================================================
function row = sampleRow(XT,YT,ZT,XM,YM,ZM,W,T,A,XTINIT,YTINIT,ZTINIT, ...
                         ALTNM, AXMGUID, AYMGUID, AZMGUID)
%SAMPLEROW  The listing's recording block.  ⚠️ ALTNM arrives from the
%   derivative section (the PREDICTED target state) while DISTRTNM below is
%   computed from the CORRECTED one -- a half-step inconsistency that is the
%   book's, kept.  Columns:
%   [T, DISTRTNM(km), ALTNM(nmi), DISTRMNM(km), ALTMNM(nmi), a(g),
%    XLONGTDEG, XLATTDEG, XLONGMDEG, XLATMDEG]
XTE = XT*cos(W*T) - YT*sin(W*T);  YTE = XT*sin(W*T) + YT*cos(W*T);  ZTE = ZT;
XLATTDEG  = 57.3*atan2(ZTE, sqrt(XTE^2 + YTE^2));
XLONGTDEG = 57.3*atan2(YTE, XTE);
DISTRTNM  = distance3d(XTE, YTE, ZTE, XTINIT, YTINIT, ZTINIT, A);

XME = XM*cos(W*T) - YM*sin(W*T);  YME = XM*sin(W*T) + YM*cos(W*T);  ZME = ZM;
XLATMDEG  = 57.3*atan2(ZME, sqrt(XME^2 + YME^2));
XLONGMDEG = 57.3*atan2(YME, XME);
DISTRMNM  = distance3d(XME, YME, ZME, XTINIT, YTINIT, ZTINIT, A);

ALTMNM   = (sqrt(XM^2 + YM^2 + ZM^2) - A)/6076.;
AXMGUIDG = sqrt(AXMGUID^2 + AYMGUID^2 + AZMGUID^2)/32.2;

row = [T, DISTRTNM, ALTNM, DISTRMNM, ALTMNM, AXMGUIDG, ...
       XLONGTDEG, XLATTDEG, XLONGMDEG, XLATMDEG];
end


% =============================================================================
function DISTKM = distance3d(XT, YT, ZT, XF, YF, ZF, A)
%DISTANCE3D  zarchan.txt:10423.  ⚠️ RETURNS KILOMETRES (A*BETA/3280) even
%   though Listings 28.2/28.4 store it in a variable named ...NM and label the
%   plot "Nmi".  See defect A in the header.  The else branch is the book's and
%   is unreachable for |CBETA| <= 1; kept so nobody "restores" it later.
%   The clamp inside acos is OURS: CBETA can come back as 1+2e-16 for a
%   near-zero arc and acos would then return a complex distance that plots.
R  = sqrt(XT^2 + YT^2 + ZT^2);
RF = sqrt(XF^2 + YF^2 + ZF^2);
CBETA = (XT*XF + YT*YF + ZT*ZF)/(R*RF);
if CBETA <= 1.
    BETA = acos(max(-1, min(1, CBETA)));
    DISTKM = A*BETA/3280.;
else
    DISTKM = (XF - XT)/3280.;
end
end


% =============================================================================
function [VRX, VRY, VRZ] = lambert3d(XT, YT, ZT, TF, XF, YF, ZF, SWITCH1)
%LAMBERT3D  zarchan.txt:10428, the three-dimensional Lambert routine printed
%   under Listing 28.2.  Secant search on flight-path angle GAMMA inside a
%   bracket, then the in-plane velocity direction as C1*r0 + C2*rf.
%
%   The bracket-update comparisons ("if T > TF ... G_MAX = GAMMA") are the
%   3-D twin of Listing 13.2's, which MGS.ZARCHAN.LAMBERTEFFICIENT resolved and
%   then CONFIRMED against the published iterate sequence of Table 13.2.  Not
%   re-derived here.  The book's dead locals (BL, BT, DEG_PER_RAD,
%   EARTH_RADIUS, FT_PER_KM, RAD_PER_SEC) are dropped, not "fixed".
PI = 3.1415926535898;  GM = 1.4077E16;  HALFPI = PI/2.;
RF0X = XF-XT;  RF0Y = YF-YT;  RF0Z = ZF-ZT;
R0DOTRF  = XT*XF + YT*YF + ZT*ZF;
R0DOTRF0 = XT*RF0X + YT*RF0Y + ZT*RF0Z;
R0MAG  = sqrt(XT^2 + YT^2 + ZT^2);
RFMAG  = sqrt(XF^2 + YF^2 + ZF^2);
RF0MAG = sqrt(RF0X^2 + RF0Y^2 + RF0Z^2);
RATIO   = R0MAG/RFMAG;
GMDIVR0 = GM/R0MAG;
COS_T   = R0DOTRF/(R0MAG*RFMAG);
VNUMER  = GMDIVR0*(1. - COS_T);
T_MIN = 0.;
if SWITCH1 == 0
    G_MIN = HALFPI - acos(R0DOTRF0/(R0MAG*RF0MAG));  G_MAX = HALFPI;
    THETA = acos(COS_T);
else
    G_MIN = -HALFPI;  G_MAX = -HALFPI + acos(R0DOTRF0/(R0MAG*RF0MAG));
    THETA = 2.*PI - acos(COS_T);
end
SIN_T = sin(THETA);  COT_HALFT = 1./tan(THETA/2.);
GAMMA = (G_MAX + G_MIN)/2.;  GOLD = G_MIN;  TOLD = 0.;  T = 0.;  ITERS = 1;
RV0MAG = NaN;  NEXT = GAMMA;   % defensive; both are set on every pass below

while abs(TF - T) > (.00000001*TF)
    SIN_G = sin(GAMMA);  COS_G = cos(GAMMA);  TAN_G = SIN_G/COS_G;
    COS_TPLUSG = cos(THETA + GAMMA - 2.*PI);
    TERM1 = (RATIO*COS_G - COS_TPLUSG)*COS_G;
    RV0MAG = sqrt(VNUMER/TERM1);
    LAMBDA = RV0MAG*RV0MAG/GMDIVR0;
    if LAMBDA < 1.9999999
        TERM0 = sqrt(2./LAMBDA - 1.);
        TERM1 = (TAN_G*(1.-COS_T) + (1.-LAMBDA)*SIN_T)/((2.-LAMBDA)*RATIO);
        TERM2 = (COS_G + COS_G)/(LAMBDA*TERM0*TERM0*TERM0);
        TERM3 = atan2(TERM0, (COS_G*COT_HALFT - SIN_G));
        T = (R0MAG/(RV0MAG*COS_G))*(TERM1 + TERM2*TERM3);
    elseif LAMBDA > 2.0000001
        TERM0 = sqrt(1. - 2./LAMBDA);
        TERM1 = (TAN_G*(1.-COS_T) + (1.-LAMBDA)*SIN_T)/((2.-LAMBDA)*RATIO);
        TERM2 = COS_G/(LAMBDA*TERM0*TERM0*TERM0);
        TERM3 = SIN_G - COS_G*COT_HALFT;
        TERM3 = log((TERM3 - TERM0)/(TERM3 + TERM0));
        T = (R0MAG/(RV0MAG*COS_G))*(TERM1 - TERM2*TERM3);
    else
        TERM0 = COS_G*COT_HALFT;  TERM1 = TERM0 - SIN_G;
        TERM0 = (3*TERM0*TERM1 + 1.)/(TERM1*TERM1*TERM1);
        T = TERM0*(2.*R0MAG)/(3.*RV0MAG);
    end
    if T > TF  && GAMMA < G_MAX, G_MAX = GAMMA; end
    if T < 0.  && GAMMA < G_MAX, G_MAX = GAMMA; end
    if T < TF  && GAMMA > G_MIN, G_MIN = GAMMA;  T_MIN = T; end
    if T < 0.
        NEXT = (G_MIN + G_MAX)/2.;  GOLD = G_MIN;  TOLD = T_MIN;
    else
        NEXT = GAMMA + (TF - T)*(GAMMA - GOLD)/(T - TOLD);
        if NEXT >= G_MAX
            NEXT = (GAMMA + G_MAX)/2.;
        elseif NEXT <= G_MIN
            NEXT = (GAMMA + G_MIN)/2.;
        end
        GOLD = GAMMA;  TOLD = T;
    end
    GAMMA = NEXT;
    ITERS = ITERS + 1;
    if ITERS > 100, break; end
end

if SWITCH1 == 0
    ANGLE = HALFPI - GAMMA;
else
    ANGLE = GAMMA - HALFPI;
end
SINA = sin(ANGLE);  COSA = cos(ANGLE);
V1X=XT; V1Y=YT; V1Z=ZT;  V2X=XF; V2Y=YF; V2Z=ZF;
MAG1   = sqrt(V1X*V1X + V1Y*V1Y + V1Z*V1Z);
DOTMAG = V1X*V2X + V1Y*V2Y + V1Z*V2Z;
CROSSX = V1Y*V2Z - V1Z*V2Y;
CROSSY = V1Z*V2X - V1X*V2Z;
CROSSZ = V1X*V2Y - V1Y*V2X;
CROSSMAG = sqrt(CROSSX*CROSSX + CROSSY*CROSSY + CROSSZ*CROSSZ);
C2 = MAG1*SINA/CROSSMAG;
C1 = COSA/MAG1 - DOTMAG*C2/(MAG1*MAG1);
VUNITX = C2*V2X + C1*V1X;
VUNITY = C2*V2Y + C1*V1Y;
VUNITZ = C2*V2Z + C1*V1Z;
VRX = RV0MAG*VUNITX;  VRY = RV0MAG*VUNITY;  VRZ = RV0MAG*VUNITZ;
end


% =============================================================================
function X1 = kepler1(X0, T0, T1)
%KEPLER1  zarchan.txt:10486, printed under Listing 28.3.  Laguerre-Conway
%   iteration (n = 5) on the universal variable, KN = 10 fixed passes.
%   ⚠️ INPUTS AND OUTPUTS ARE km AND km/s -- Listing 28.4 divides by 3280 going
%   in and multiplies by 3280 coming out.
%   ⚠️ Book defect kept visible, not reproduced: the |DT| < TLIMIT branch fills
%   X1 = X0 and then FALLS THROUGH into the solver (there is no return), so the
%   copy is immediately overwritten.  Returned here instead, because falling
%   through divides by DT = 0.
%   MGS.ZARCHAN.KEPLERVSDIRECT (Listing 28.3) is the authority on this routine;
%   this copy exists because Listing 28.4 calls it to build the aim point.
GMX = 398923.;  REX = 6380.;  TLIMIT = 1.E-10;  KN = 10;  MUQR = 1.;
DT = T1 - T0;
if abs(DT) < TLIMIT
    X1 = X0(1:6);
    return
end
TIME_FACTOR = sqrt(REX^3/GMX);
VEX = REX/TIME_FACTOR;
DT  = DT/TIME_FACTOR;
R0 = X0(1:3)/REX;
V0 = X0(4:6)/VEX;
R0MAG = sqrt(R0(1)^2 + R0(2)^2 + R0(3)^2);
V0MAG = sqrt(V0(1)^2 + V0(2)^2 + V0(3)^2);
D0 = R0(1)*V0(1) + R0(2)*V0(2) + R0(3)*V0(3);
SIGMA0 = D0/MUQR;
ALP0 = 2./R0MAG - V0MAG*V0MAG;
X = ALP0*DT;
if ALP0 <= 0.
    X = .1*DT/R0MAG;
end
U1 = 0.; U2 = 0.; U3 = 0.; DFX = 1.;
for K = 1:KN
    if ALP0 < 0.
        Y = ALP0*X*X;  YQR = sqrt(-Y);
        CY = (1. - cosh(YQR))/Y;  SY = (sinh(YQR) - YQR)/(YQR^3);
    elseif ALP0 == 0.
        Y = 0.;  CY = .5;  SY = 1./6.;
    else
        Y = ALP0*X*X;  YQR = sqrt(Y);
        CY = (1. - cos(YQR))/Y;  SY = (YQR - sin(YQR))/(YQR^3);
    end
    U1 = X*(1. - Y*SY);
    U2 = X*X*CY;
    U3 = X*X*X*SY;
    FX   = R0MAG*U1 + SIGMA0*U2 + U3 - DT*MUQR;
    DFX  = SIGMA0*U1 + (1. - ALP0*R0MAG)*U2 + R0MAG;
    DFX2 = SIGMA0*(1. - Y*CY) + (1. - ALP0*R0MAG)*U1;
    SDFX = DFX/abs(DFX);
    DX2  = 16.*DFX*DFX - 20.*FX*DFX2;
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
for I = 1:3
    X1(I)   = (F*R0(I)  + G*V0(I))*REX;
    X1(I+3) = (DF*R0(I) + DG*V0(I))*VEX;
end
end


% =============================================================================
function K = structuralChecks(E, ED, P, lamX, lamY, lamZ)
%STRUCTURALCHECKS  Self-consistency.  NOT book values.  ⚠️ .names and .oks must
%   both exist and be the SAME LENGTH or benchSuite puts ZERO rows on the board.
names = {};  oks = [];  detail = {};

% 1  the run terminated at a genuine closest approach, not on the step cap.
names{end+1} = 'run ended on VC <= 0 (closest approach), not on the step cap';
oks(end+1)   = ~E.timeout;
detail{end+1} = sprintf('%d steps, t_end = %.4f s, RTM = %.2f ft', ...
    E.nsteps, E.t_end, E.miss_ft);

% 2  the missile is pinned to the rotating Earth until TLAUNCH.  Catches an
%    inverted "if T < TLAUNCH" in either the predictor or the corrector: with
%    the test flipped the missile either coasts off the ground before launch or
%    is teleported home every step after it.
pre = E.t < P.TLAUNCH;
if any(pre), altPre = max(abs(E.alt_m_nmi(pre))); else, altPre = NaN; end
names{end+1} = 'missile pinned to its ECEF launch site until TLAUNCH';
oks(end+1)   = ~isnan(altPre) && altPre <= 1e-6;
detail{end+1} = sprintf('max |ALTMNM| over %d pre-launch samples = %.3e n mi', ...
    sum(pre), altPre);

% 3  DELETE THE HOMING LAW.  The only check that proves the guidance channel is
%    connected to the missile's acceleration at all.
if isempty(ED)
    names{end+1} = 'DELETE the homing law -> miss grows to prediction-error scale';
    oks(end+1)   = false;
    detail{end+1} = 'NOT RUN (''delete_experiment'', false) -- an unrun check is not a pass';
else
    names{end+1} = 'DELETE the homing law -> miss grows to prediction-error scale';
    oks(end+1)   = ED.miss_ft > 10*E.miss_ft && ED.miss_ft > 0.25*P.PREDERR;
    detail{end+1} = sprintf(['guided miss %.1f ft -> unguided %.1f ft ' ...
        '(x%.1f); PREDERR = %.0f ft'], E.miss_ft, ED.miss_ft, ...
        ED.miss_ft/max(E.miss_ft, realmin), P.PREDERR);
end

% 4  DEFECT B, measured rather than argued: how far the aim point moves if the
%    longitude is advanced by TFTOT instead of TF.  Reported as a bound so the
%    reader sees the size; it passes as long as the two readings really do
%    differ (if they did not, the header's claim would be wrong).
dv = norm([E.XTD0 E.YTD0 E.ZTD0] - [lamX lamY lamZ]);
names{end+1} = 'defect B is real: -W*TF and -W*TFTOT give different Lambert legs';
oks(end+1)   = (abs(P.TROT - P.TFTOT) < 1e-9) || dv > 1.0;
detail{end+1} = sprintf(['|v0(TF-rotation) - v0(TFTOT-rotation)| = %.2f ft/s ' ...
    '(this run used the %s rotation)'], dv, ...
    ternary(abs(P.TROT - P.TF) < 1e-9, 'TF', 'TFTOT'));

% 5  the clamp never binds on the plotted record, so a FLIPPED clamp
%    comparison (defect 5 in the header) would peg every sample at
%    sqrt(3)*XNCLIM/32.2 g instead of leaving the book's 0.1 g curve.
flipped_g = sqrt(3)*P.XNCLIM/32.2;
names{end+1} = 'accel clamp does not bind on the plotted record (flipped clamp would peg it)';
oks(end+1)   = E.amax_plot_g < 0.25*flipped_g;
detail{end+1} = sprintf(['plotted peak %.4f g vs flipped-clamp value %.2f g; ' ...
    'clamp fired on %d of %d derivative evaluations (endgame only)'], ...
    E.amax_plot_g, flipped_g, E.nclamp, 2*E.nsteps);

K.names = names;  K.oks = logical(oks);  K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    j = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{j}, detail{j});
end
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end


% =============================================================================
function B = benchmarks(R, E, P, opts, lamX, lamY, lamZ, X1book)
%BENCHMARKS  Ten rows.  See the header for every band and where it came from.
B = emptyBench();
SRC_IC   = 'Listing 28.3 line 1 (ICs from Listing 28.2), zarchan.txt:10475';
SRC_TAB  = 'Table 28.1, Kepler row, zarchan.txt:10467-10470';
SRC_NOM  = 'nominal-case prose for Fig. 28.10/28.11, zarchan.txt:10575';

% 1-3  the Lambert leg, in Listing 28.2's convention.  Independent of this
%      run's aim_rotation choice, so they are scored unconditionally.
B = addVal(B, 'target ECI velocity XTD (Lambert, 28.2 convention)', ...
    996.773566434768, lamX, '1e-5 rel', 1e-5, SRC_IC, 'ft/s', ...
    'LAMBERT3D, aim point rotated by the full TFTOT = 2000 s');
B = addVal(B, 'target ECI velocity YTD (Lambert, 28.2 convention)', ...
    -14954.8124604715, lamY, '1e-5 rel', 1e-5, SRC_IC, 'ft/s', ...
    'same solve; sign is the book''s own minus, not OCR damage');
B = addVal(B, 'target ECI velocity ZTD (Lambert, 28.2 convention)', ...
    17528.1931768263, lamZ, '1e-5 rel', 1e-5, SRC_IC, 'ft/s', ...
    'same solve');

% 4  the Kepler leg, on the book's OWN printed ICs.
B = addVal(B, 'Kepler-propagated target x at t = 1000 s', ...
    2547.4148, X1book(1), '1e-5 rel', 1e-5, SRC_TAB, 'km', ...
    sprintf(['y = %.4f (book -3583.1074), z = %.4f (book 6758.2384); ' ...
             'owned by mgs.zarchan.keplerVsDirect, repeated here because ' ...
             'Listing 28.4 calls KEPLER1 to build the aim point'], ...
             X1book(2), X1book(3)));

% 5-9  the nominal case.  Gated: these are statements about the listing's own
%      settings, so an off-book run must SKIP them, not stretch them.
onBook = abs(P.TLAUNCH - 200) < 1e-9 && abs(P.TFTOT - 2000) < 1e-9 && ...
         abs(P.TF - 1000) < 1e-9 && abs(opts.PREDERR_NMI - 10) < 1e-9 && ...
         abs(P.XNP - 3) < 1e-9 && abs(P.XNCLIM - 161) < 1e-9;
whyOff = ['this run is not the listing''s nominal case (TLAUNCH/TFTOT/TF/' ...
          'PREDERR/N''/XNCLIM changed), and the book''s sentence describes ' ...
          'that case only'];

if onBook
    B = addVal(B, 'time homing guidance turns on', 800., E.t_guid_on, ...
        '2%', 0.02, SRC_NOM, 's', sprintf(['first T with TGO = RTM/VC < 200 s; ' ...
        'intercept at %.2f s, so %.2f s of homing (book says 200)'], ...
        E.t_end, E.t_end - E.t_guid_on));
    B = addVal(B, 'intercept altitude', 900., R.alt_intercept_nmi, ...
        '10%', 0.10, SRC_NOM, 'n mi', ...
        'corroborated by the 900 n mi apogee at zarchan.txt:10457');
    B = addVal(B, 'intercept downrange from target launch site', 2500., ...
        R.dist_intercept_nmi, '10%', 0.10, SRC_NOM, 'n mi', ...
        sprintf(['the listing''s own DISTRTNM is %.1f -- and it is KM, ' ...
                 'not n mi (defect A)'], E.dist_t_km(end)));
    B = addVal(B, 'peak commanded acceleration (10 s record, Fig. 28.11)', ...
        0.1, E.amax_plot_g, '50% (book is 1 s.f.)', 0.50, SRC_NOM, 'g', ...
        sprintf(['instantaneous peak over every step is %.3f g and the ' ...
                 'terminal sample is %.3f g -- both off the listing''s own ' ...
                 'axis([0 1000 0 .5]) and neither is scored'], ...
                 E.amax_step_g, E.acc_terminal_g));
    if isnan(E.t_guid_on)
        % The homing law never engaged.  That is not "zero before homing" --
        % it is zero for the whole flight, and passing this row on it would be
        % the exact shape of green light this package exists to prevent.
        B = addSkip(B, 'command is EXACTLY zero before homing starts', 'zero', ...
            ['homing never engaged (TGO never fell below 200 s while ' ...
             'T > TLAUNCH+50), so there is no "before" to test'], SRC_NOM, '-');
    else
        preOn = E.t < E.t_guid_on;
        zeroBefore = any(preOn) && max(E.acc_g(preOn)) == 0;
        B = addShape(B, 'command is EXACTLY zero before homing starts', ...
            'zero for every sample before t_homing_on', ...
            sprintf('max = %.3g g over %d samples', ...
                    max([0; E.acc_g(preOn)]), sum(preOn)), ...
            zeroBefore, SRC_NOM, '"the acceleration command was zero before that"');
    end
else
    B = addSkip(B, 'time homing guidance turns on', '800 s', whyOff, SRC_NOM, 's');
    B = addSkip(B, 'intercept altitude', '900 n mi', whyOff, SRC_NOM, 'n mi');
    B = addSkip(B, 'intercept downrange from target launch site', '2500 n mi', ...
        whyOff, SRC_NOM, 'n mi');
    B = addSkip(B, 'peak commanded acceleration', '0.1 g', whyOff, SRC_NOM, 'g');
    B = addSkip(B, 'command is EXACTLY zero before homing starts', 'zero', ...
        whyOff, SRC_NOM, '-');
end

% 10  the listing's own printed outputs, which the book never publishes.
B = addInfo(B, 'miss RTM and divert DELV (the listing''s last line)', ...
    'printed by "RTM DELV" -- no value published in Chapter 28', ...
    sprintf('RTM = %.2f ft, DELV = %.2f ft/s', E.miss_ft, E.delv_fps), ...
    'Listing 28.4 final statement, zarchan.txt:10574', ...
    ['the book prints both variables but reports neither anywhere in the ' ...
     'chapter, so there is nothing to score against']);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER are benchSuite's adaptGeneric schema.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.12g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.12g', ours);
B(i).band = band;  B(i).tol = tol;
B(i).units = units;  B(i).source = source;  B(i).detail = detail;
if isempty(ours) || ~isscalar(ours) || ~isfinite(ours) || book == 0
    B(i).pct = NaN;  B(i).ok = false;  B(i).skipped = true;
    B(i).why = 'the computed value is not a finite scalar (or the book value is zero)';
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
%ADDSKIP  A check that COULD NOT RUN.  Never a pass.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'skip';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = '--';
B(i).pct  = NaN;   B(i).band = 'skip';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true;  B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end


% =============================================================================
function s = render(R, ev)
L = {};
L{end+1} = 'Zarchan Listing 28.4 -- 3-D STRATEGIC MISSILE-TARGET ENGAGEMENT';
L{end+1} = sprintf('   %s', R.units);
L{end+1} = sprintf(['   target %.3f/%.3f deg -> aim %.3f/%.3f deg, TFTOT = %g s; ' ...
    'missile from %.3f/%.3f deg at TLAUNCH = %g s'], R.p.XLATTDEG, ...
    R.p.XLONGTDEG, R.p.XLATFDEG, R.p.XLONGFDEG, R.p.TFTOT, R.p.XLATMDEG, ...
    R.p.XLONGMDEG, R.p.TLAUNCH);
L{end+1} = sprintf(['   PREDERR = %.0f ft, XNCLIM = %g ft/s^2, N'''' = %g, ' ...
    'aim rotated by %s (%g s)'], R.p.PREDERR, R.p.XNCLIM, R.p.XNP, ...
    R.p.aim_rotation, R.p.TROT);
L{end+1} = sprintf(['   intercept at t = %.3f s, homing on at %.3f s, ' ...
    'alt %.1f n mi, downrange %.1f n mi (= %.1f km, the listing''s own number)'], ...
    R.t_intercept, R.t_homing_on, R.alt_intercept_nmi, R.dist_intercept_nmi, ...
    R.dist_t_km(end));
L{end+1} = sprintf('   miss RTM = %.2f ft, divert DELV = %.2f ft/s, peak a = %.4f g (plotted) / %.3f g (every step)', ...
    R.miss_ft, R.delv_fps, R.peak_acc_g, R.peak_acc_all_g);
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-62s %s', vstr(K.oks(i)), K.names{i}, K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning';
    L{end+1} = '            NOT CHECKED, which is not the same thing as "fine".';
else
    L{end+1} = 'BENCHMARKS  (book values grepped from zarchan_decoded.txt; lines in .source)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
            case 'skip'
                L{end+1} = sprintf('   SKIP %-56s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case 'info'
                L{end+1} = sprintf('   INFO %-56s %s', b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s  [%s]', b.why, b.source); %#ok<AGROW>
            case 'shape'
                L{end+1} = sprintf('   %-4s %-56s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s  [%s]', b.book_str, b.source); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf(['   %-4s %-56s book %s %s / ours %s %s ' ...
                    '(%+.4f%%, band %s)'], vstr(b.ok), b.what, b.book_str, ...
                    b.units, b.ours_str, b.units, b.pct, b.band); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped/info -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: DISTRTNM/DISTRMNM are KILOMETRES in the listing and are';
L{end+1} = '   labelled "Nmi" on its plot (defect A).  R.dist_*_nmi is the';
L{end+1} = '   converted value and is what the book''s prose quotes.';
s = strjoin(L, newline);
end

function s = vstr(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end
