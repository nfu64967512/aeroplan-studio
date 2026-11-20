function R = stereoTrackingMC(opts)
%MGS.ZARCHAN.STEREOTRACKINGMC  Zarchan Listing 34.4 -- two-dimensional MONTE
%   CARLO engagement simulation using STEREO TRACKING (boost-phase intercept).
%
%   R = mgs.zarchan.stereoTrackingMC()
%   R = mgs.zarchan.stereoTrackingMC('engage', false)      book rows only, fast
%   R = mgs.zarchan.stereoTrackingMC('RUN', 50, 'assert', false)   the book's MC
%   R = mgs.zarchan.stereoTrackingMC('PHIS', 9)            the Fig. 34.24 filter
%
%   Transcribed from zarchan.txt:13788-13891 (the listing), with predict34 from
%   :13442-13466 and LAMBERT3D from :10428 (printed under Listing 28.2, already
%   transcribed for MGS.ZARCHAN.STRATEGICENGAGEMENT and copied here unchanged).
%   205 code lines; the largest listing in the book.
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, g = 32.2.  Angles in radians,
%   sensor sigma in radians (5e-5 rad = 50 microradians).  The listing reports
%   MISS in ft and DIVERT in m/s (DELV/3.28) and so does R.  Convert at the
%   call site, not here.
% ===========================================================================
%
%   *** THIS CHAPTER PUBLISHES ALMOST NOTHING SCOREABLE FOR THIS LISTING. ***
%   Every output of 34.4 -- filter estimates, estimation errors against the
%   Riccati bounds, and the 90% KKV divert -- appears ONLY on Figs. 34.21-34.27,
%   and the extraction turned every figure into an image link.  The one numeric
%   prose claim about the output ("the divert requirements can still increase
%   more than 500 m/s", :13963) is a difference between two figure curves and
%   is emitted as a SKIP with the arithmetic, not absorbed by a wide band.
%   What IS published, and is therefore what is scored, is (a) the case setup
%   in prose at :13892, and (b) the stereo-tracking ALGEBRA at :13764-13782 --
%   the triangulation closed form and the chain-rule partials (checked by
%   COMPLEX STEP, not by differencing -- see algebraResiduals).  Nine scored
%   rows, all of them engagement-free.
%
% ---------------------------------------------------------------------------
%   THE SEVEN UNRESOLVED COMPARISONS, AND HOW EACH WAS SETTLED
%
%   The OCR maps "<" to "," and ">" to ".".  Listing 34.3 (:13396-13440) is the
%   same driver without the filters and settles five of these by printing them
%   in a context where the shape forces the direction; the other two are
%   settled by the prose at :13784.  Resolutions, with the reason:
%
%   1  "if VMRQDKM,VMRQDKMIC"  ->  VMRQDKM < VMRQDKMIC (= 4 km/s).  PROSE
%      (:13372): the loop finds "the earliest possible intercept time for an
%      impulsive interceptor whose speed is less than 4 km/s".  34.3 prints the
%      same test against the literal 4.
%   2  "while ~((T.(TF-10.)) & VC,0.)"  ->  ~(T > TF-10 & VC < 0).  Shape: T
%      only increases and VC changes sign at closest approach, so this is the
%      book's universal "stop just after intercept" guard.
%   3  "if (T.TGUID & TGO,THOM)"  ->  T > TGUID & TGO < THOM.  PROSE (:13784):
%      "During the last 10 s ... homing guidance took place and ... near-perfect
%      estimates of the target states were available at a 100-Hz rate".  This
%      block uses the TRUE states and sits in the derivative loop (rate 1/H =
%      100 Hz).  THOM = 10.
%   4  "if (T.TGUID & TGO.THOM)"  ->  T > TGUID & TGO > THOM.  Same sentence,
%      other half (:13784): "command guided ... until 10 s before intercept
%      based on the Kalman filter estimates".  This block uses XH/XDH/XDDH and
%      sits inside the TS = 0.1 s sampling block (10 Hz).  The two together are
%      why 34.4 replaced 34.3's "if T > TGUID ... else zero" with two disjoint
%      gates: between filter samples the command simply HOLDS.
%   5  "if AXMGUID.XNCLIM" (x4)  ->  AXMGUID > XNCLIM.  Shape-forced by its own
%      "elseif AXMGUID < -XNCLIM" partner.
%   6  "if T.TLAUNCH" / "if T,TUPT" / "if T,TPZ & DELVELT.500."  ->  T >
%      TLAUNCH, T < TUPT, T < TPZ & DELVELT > 500.  Shape: the missile is
%      inert before launch, the target flies vertically for the first TUPT
%      seconds, and the DELVELT branch is forced by its own elseif.
%   7  "if S.=(TS-.0001)" / "if XN,10."  ->  S >= (TS-.0001) and XN < 10.  S is
%      zeroed on every sample so "<=" would sample forever; XN < 10 selects the
%      least-squares startup gains for the first ten measurements.
%
%   Two lines of leaked prose and the "¼ð Þþ½" residue were dropped, not
%   "fixed" into code.  So were the listing's dead diagnostics -- QLAUNCH,
%   QOOMPH, PIPERR/PIPERRKM, ACCPERPTOT, ZEMPERPTOT, TBOT, DISTRTKMIC and the
%   figure/save block: assigned, never read, and none of them published.
%
%   THREE BOOK DEFECTS KEPT VISIBLE RATHER THAN REPAIRED:
%     * "count" is shared by the per-STEP recorder and the per-RUN recorder, so
%       at RUN = 1 the two overwrite each other's index.  Separate counters are
%       used here and the defect is recorded in R.p.book_defects.
%     * predict34's post-burnout branch sets qboost = 1 where the driver sets
%       QBOOST = 0.  Kept: it re-snaps xtd to the Lambert velocity every step,
%       which is the SAME ballistic arc, so it is self-consistent -- but it
%       keeps calling LAMBERT3D for the rest of the prediction, and that is
%       most of this port's runtime.
%     * The Taylor PIP (XTF/YTF) is built from the TRUE target states XT, XTD,
%       XTDD, not from the filter estimates XH, XDH, XDDH -- 34.4 did not
%       change that line from 34.3.  The prose at :13784 says the interceptor
%       was "command guided (first with Lambert guidance and then APN) ... based
%       on the Kalman filter estimates"; as printed only the APN half is.  NOT
%       silently corrected here.
%
%   CASE MISMATCH, both lines cited: the listing flies sensor 1 at 400 km and
%   sensor 2 at 500 km downrange (XLONGMDEGICKM = 400, XLONGS2DEGKM = 500,
%   :13788); the prose case for Figs. 34.21-34.24 is 800 km and 900 km
%   (:13892).  The 100-km BASELINE is common to both, and that is the only part
%   scored.  TF is likewise declared 230 s at :13788 and confirmed in prose at
%   :13892, but QFIX = 0 makes the for-loop OVERWRITE it; the searched value is
%   reported as an INFO row, and the declared 230 is what row B4 scores.
%
% ---------------------------------------------------------------------------
%   OUTPUTS
%     R.miss_ft, R.divert_ms  per-run miss and divert (the listing's ArrayRTM,
%                             ArrayDELV/3.28).  [] when 'engage' is false.
%     R.pct90                 the 90% point, the book's figure of merit
%     R.single                RUN == 1 single-flight record: .T .err_xtdd_g
%                             .sp33x_g (Fig. 34.23's three curves)
%     R.bench .selfcheck .pass .verdict .p .summary   package conventions
%
%   See also MGS.ZARCHAN.STRATEGICENGAGEMENT (LAMBERT3D),
%   MGS.ZARCHAN.KINEMATICENGAGEMENTMC, MGS.ZARCHAN.BENCHSUITE.

arguments
    opts.PHIS      (1,1) double {mustBeFinite, mustBeNonnegative} = 576     % Fig. 34.21/34.23
    opts.SIGTHET1  (1,1) double {mustBeFinite, mustBeNonnegative} = 0.00005 % rad = 50 urad
    opts.TS        (1,1) double {mustBeFinite, mustBePositive}    = 0.1     % s
    opts.H         (1,1) double {mustBeFinite, mustBePositive}    = 0.01    % s
    % The listing prints RUN = 50.  1 is the book's OWN single-flight mode
    % (:13786) and is the default here only because 50 costs 50x; nothing is
    % scored off the engagement either way.
    opts.RUN       (1,1) double {mustBeInteger, mustBePositive}   = 1
    opts.ITGT      (1,1) double {mustBeMember(opts.ITGT,[1 2])}   = 2       % 2 = ICBM
    opts.QFIX      (1,1) double {mustBeMember(opts.QFIX,[0 1])}   = 0
    opts.QPERFECT  (1,1) double {mustBeMember(opts.QPERFECT,[0 1])} = 0
    opts.QGUID     (1,1) double {mustBeMember(opts.QGUID,[0 1])}  = 1
    opts.BIAS1     (1,1) double {mustBeFinite}                    = 0       % rad
    opts.XLONGMDEGICKM (1,1) double {mustBeFinite, mustBePositive} = 400.    % km
    opts.XLONGS2DEGKM  (1,1) double {mustBeFinite, mustBePositive} = 500.    % km
    opts.TFDES     (1,1) double {mustBeFinite, mustBePositive}    = 230.    % s
    opts.seed      (1,1) double {mustBeFinite, mustBeNonnegative} = 34042026
    opts.maxsteps  (1,1) double {mustBeFinite, mustBePositive}    = 4e6
    opts.engage    (1,1) logical = true
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.assert    (1,1) logical = true
end

UNITS = ['ft, ft/s, ft/s^2 (g = 32.2 ft/s^2); angles rad; miss in ft and ' ...
         'divert in m/s (the listing''s DELV/3.28)'];

% Listing 34.4 line 1, verbatim except for the values the arguments block owns.
P = struct();
P.PHIS = opts.PHIS;   P.TLAUNCH = 80.;     P.TFDES = opts.TFDES;
P.TS   = opts.TS;     P.H = opts.H;        P.RDESKM = 10000.;
P.XLONGMDEGICKM = opts.XLONGMDEGICKM;      P.XLONGS2DEGKM = opts.XLONGS2DEGKM;
P.GAMDEG = 89.99;     P.TUPT = 20.;        P.TGUID = 100.;
P.XNCLIM = 322.;      P.XNP = 3.;          P.QPERFECT = opts.QPERFECT;
P.TLOFT = 500.;       P.ALTMKMIC = 15.;    P.QTAYLOR = 1;
P.QGUID = opts.QGUID; P.ITGT = opts.ITGT;  P.DELTF = 0.;
P.QFIX = opts.QFIX;   P.SIGTHET1 = opts.SIGTHET1;
P.THOM = 10.;         P.BIAS1 = opts.BIAS1;
P.RUN = opts.RUN;     P.VMRQDKMIC = 4.;    P.ORDER = 3;
P.seed = opts.seed;   P.maxsteps = opts.maxsteps;
P.units = UNITS;
P.book_defects = { ...
    'shared "count" between the per-step and per-run recorders (RUN==1)'; ...
    'predict34 sets qboost=1 in the branch where the driver sets QBOOST=0'; ...
    'Taylor PIP uses TRUE target states, not the filter estimates'};

% =========================================================================
%   THE ENGAGEMENT.  Guarded, and a failure becomes a RED row rather than an
%   exception: the nine scored rows below do not need it, and a port that
%   throws here hands the caller nothing to diagnose.
% =========================================================================
ENG = [];  eng_err = '';
if opts.engage
    try
        ENG = engagementSet(P);
    catch ME
        eng_err = sprintf('%s (%s)', ME.message, ME.identifier);
    end
end

R = struct();
R.miss_ft   = [];  R.divert_ms = [];  R.pct90 = struct('miss_ft',NaN,'divert_ms',NaN);
R.single    = [];  R.TF_used = NaN;   R.handover = [];
if ~isempty(ENG)
    R.miss_ft   = ENG.miss_ft;
    R.divert_ms = ENG.divert_ms;
    R.pct90     = struct('miss_ft', pct90(ENG.miss_ft), ...
                         'divert_ms', pct90(ENG.divert_ms));
    R.single    = ENG.single;
    R.TF_used   = ENG.TF;
    R.handover  = ENG.handover;
end
R.p = P;
R.p.engage_error = eng_err;

R.selfcheck = structuralChecks(P);
R.units     = UNITS;

if opts.benchmark
    R.bench = benchmarks(P, ENG, eng_err, opts.engage, UNITS);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && all(R.selfcheck.oks);
else
    R.bench = emptyBench();
    ev      = false;
    R.pass  = false;   % NOT CHECKED must never render as fine
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false, so nothing was compared; R.pass = false means NOT CHECKED.';
elseif ~any(ev)
    msg = 'not one book value could be evaluated.';
elseif ~all(R.selfcheck.oks)
    bad = find(~R.selfcheck.oks, 1);
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question.  ' ...
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
        error('mgs:zarchan:stereoTrackingMC:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:stereoTrackingMC:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function OUT = engagementSet(P)
%ENGAGEMENTSET  The JJ loop of Listing 34.4, zarchan.txt:13788-13890.
A = 2.0926E7;  GM = 1.4077E16;
if P.ITGT == 1, TPZ = 180.; else, TPZ = 240.; end
SIGTHET2 = P.SIGTHET1;  XLONGS1DEGKM = P.XLONGMDEGICKM;
ALTM = P.ALTMKMIC*3280.;
TFTOT = 252. + .223*P.RDESKM - (5.44E-6)*P.RDESKM*P.RDESKM + P.TLOFT;

TS = P.TS;  TGUID = P.TGUID;  THOM = P.THOM;  XNP = P.XNP;
TLAUNCH = P.TLAUNCH;  TUPT = P.TUPT;

% Built by dot assignment, NOT struct('miss_ft', zeros(1,RUN), ...): struct()
% given a non-scalar numeric value builds a STRUCT ARRAY of that size, which
% would turn OUT into a 1-by-RUN struct and every later OUT.x into a comma
% list.  It runs; it just stops meaning what it says.
OUT = struct();
OUT.miss_ft   = zeros(1,P.RUN);
OUT.divert_ms = zeros(1,P.RUN);
OUT.TF        = NaN;
OUT.single    = [];
OUT.handover  = struct();
st = mod(floor(abs(P.seed)), 2147483646) + 1;      % Lehmer state, carried on

nCmd = 0;  nHome = 0;  tHandover = NaN;

for JJ = 1:P.RUN
    % ---- per-run initial conditions, :13790-13792 -------------------------
    SWITCHM = 0;  SWITCH1 = 0;  W = 0.;
    XLONGFDEG = 57.3*P.RDESKM*3280./A;
    XLONGMDEG = P.XLONGMDEGICKM/111.;
    XLONGS1DEG = XLONGS1DEGKM/111.;  XLONGS2DEG = P.XLONGS2DEGKM/111.;
    QFIRST = 1;  QBOOST = 1;  T = 0.;  S = 0.;  AXT = 0.;  AYT = 0.;
    XLONGF = XLONGFDEG/57.3 - W*TFTOT;
    XF = A*cos(XLONGF);  YF = A*sin(XLONGF);  ZF = 0;
    XLONGT = 0.;  XLONGM = XLONGMDEG/57.3;
    XLONGS1 = XLONGS1DEG/57.3;  XLONGS2 = XLONGS2DEG/57.3;
    XT = A*cos(XLONGT);   YT = A*sin(XLONGT);   ZT = 0;
    XM = (A+ALTM)*cos(XLONGM);   YM = (A+ALTM)*sin(XLONGM);  ZM = 0;
    XS1 = (A+ALTM)*cos(XLONGS1); YS1 = (A+ALTM)*sin(XLONGS1);
    XS2 = (A+ALTM)*cos(XLONGS2); YS2 = (A+ALTM)*sin(XLONGS2);
    XTD = cos(1.5708 - P.GAMDEG/57.3);  YTD = sin(1.5708 - P.GAMDEG/57.3);
    ATP = 1.;  H = P.H;  XMD = 0.;  YMD = 0.;
    RTM1 = XT-XM;  RTM2 = YT-YM;  RTM = sqrt(RTM1^2 + RTM2^2);
    VTM1 = XTD-XMD;  VTM2 = YTD-YMD;
    VC = -(RTM1*VTM1 + RTM2*VTM2)/RTM;
    DELV = 0.;  AXMGUID = 0.;  AYMGUID = 0.;  XNCLIM = P.XNCLIM;
    XTDD = 0.;  YTDD = 0.;  XMDD = 0.;  YMDD = 0.;  DELVD = 0.;  TGO = 0.;
    XTF = XT;  YTF = YT;  ZTF = 0;  XTFACT = XT;  YTFACT = YT;

    % ---- pick TF.  QFIX = 1 -> one predict34; QFIX = 0 -> the search loop,
    %      which is where most of the runtime lives.  :13794-13800.
    TF = P.TFDES;
    if P.QFIX == 1
        [XTFACT, YTFACT] = predict34(T,XT,YT,XTD,YTD,TF,TFTOT,TUPT,XF,YF,P.ITGT,P.H,P.maxsteps);
    else
        for TFtry = (TLAUNCH+30.):10:(TPZ-10.)
            TF = TFtry;
            [XTFACT, YTFACT] = predict34(T,XT,YT,XTD,YTD,TF,TFTOT,TUPT,XF,YF,P.ITGT,P.H,P.maxsteps);
            TGOLAM = TF - TLAUNCH;
            [VRX, VRY] = lambert3d(XM,YM,ZM,TGOLAM,XTFACT,YTFACT,0,SWITCHM);
            VMRQDKM = sqrt(VRX^2 + VRY^2)/3280.;
            if VMRQDKM < P.VMRQDKMIC, break; end          % <- comparison 1
        end
    end
    TF = TF + P.DELTF;
    OUT.TF = TF;

    % ---- two decoupled three-state polynomial filters, :13802 -------------
    XH=0.; XDH=0.; XDDH=0.; YH=0.; YDH=0.; YDDH=0.;
    PHI = zeros(3);  PHI(1,1)=1; PHI(1,2)=TS; PHI(1,3)=.5*TS*TS;
    PHI(2,2)=1; PHI(2,3)=TS; PHI(3,3)=1;
    PHIT = PHI';  HMAT = [1. 0. 0.];  HT = HMAT';  IDNPZ = eye(3);
    PX = zeros(3);  PX(1,1)=99999999999.; PX(2,2)=99999999999.; PX(3,3)=99999999999.;
    PY = PX;
    Q = qMatrix(P.PHIS, TS);
    XN = 0.;

    nrec = 0;  recT = zeros(200000,1);  recE = recT;  recS = recT;
    nstep = 0;

    % ---- comparison 2 -----------------------------------------------------
    while ~((T > (TF-10.)) && (VC < 0.))
        nstep = nstep + 1;
        if nstep > P.maxsteps
            error('mgs:zarchan:stereoTrackingMC:runaway', ...
                'engagement exceeded %g steps at T = %g s (TF = %g s).', ...
                P.maxsteps, T, TF);
        end
        if RTM < 1000, H = .00001; else, H = P.H; end
        XTOLD=XT; YTOLD=YT; XTDOLD=XTD; YTDOLD=YTD;
        XMOLD=XM; YMOLD=YM; XMDOLD=XMD; YMDOLD=YMD; DELVOLD=DELV;
        STEP = 1;  FLAG = 0;  homedThisStep = false;
        while STEP <= 1
            if FLAG == 1
                STEP = 2;
                XT=XT+H*XTD; YT=YT+H*YTD; XTD=XTD+H*XTDD; YTD=YTD+H*YTDD;
                XM=XM+H*XMD; YM=YM+H*YMD; XMD=XMD+H*XMDD; YMD=YMD+H*YMDD;
                DELV=DELV+H*DELVD; T=T+H;
            end
            if P.ITGT == 1
                if T < 180., WGT=-212.*T+44000.;  TRST=54100.;
                else,        WGT=3300.;           TRST=0.;      end
            else
                if T < 120.,     WGT=-2622*T+440660.; TRST=725850.;
                elseif T < 240., WGT=-642.*T+168120.; TRST=182250.;
                else,            WGT=5500.;           TRST=0.;   end
            end
            ATP = 32.2*TRST/WGT;
            TEMPBOTT = (XT^2+YT^2)^1.5;
            XTDD = -GM*XT/TEMPBOTT + AXT;  YTDD = -GM*YT/TEMPBOTT + AYT;
            RTM1=XT-XM; RTM2=YT-YM; VTM1=XTD-XMD; VTM2=YTD-YMD;
            RTM = sqrt(RTM1^2+RTM2^2);
            VC  = -(RTM1*VTM1 + RTM2*VTM2)/RTM;
            TGO = RTM/VC;
            % ---- comparison 3: TRUE-state APN, last THOM seconds, 1/H rate
            if (T > TGUID) && (TGO < THOM)
                TEMPBOTM = (XM^2+YM^2)^1.5;
                XMDDGRAV = -GM*XM/TEMPBOTM;  YMDDGRAV = -GM*YM/TEMPBOTM;
                ZEM1 = RTM1 + VTM1*TGO + .5*(XTDD-XMDDGRAV)*TGO^2;
                ZEM2 = RTM2 + VTM2*TGO + .5*(YTDD-YMDDGRAV)*TGO^2;
                ZEMDOTRTM = (ZEM1*RTM1 + ZEM2*RTM2)/RTM;
                ZEMPER1 = ZEM1 - ZEMDOTRTM*RTM1/RTM;
                ZEMPER2 = ZEM2 - ZEMDOTRTM*RTM2/RTM;
                AXMGUID = XNP*ZEMPER1/(TGO^2);
                AYMGUID = XNP*ZEMPER2/(TGO^2);
                if P.QGUID == 0, XNCLIM = 0.; end
                [AXMGUID, AYMGUID] = clampAcc(AXMGUID, AYMGUID, XNCLIM);
                homedThisStep = true;
                if isnan(tHandover), tHandover = T; end
            end
            if T <= TGUID, AXMGUID = 0.; AYMGUID = 0.; end
            if T > TLAUNCH                                  % comparison 6
                TEMPBOTM = (XM^2+YM^2)^1.5;
                XMDD = -GM*XM/TEMPBOTM + AXMGUID;
                YMDD = -GM*YM/TEMPBOTM + AYMGUID;
            else
                XMDD = 0.;  YMDD = 0.;
            end
            DELVD = sqrt(AXMGUID^2 + AYMGUID^2);
            FLAG = 1;
        end
        if homedThisStep, nHome = nHome + 1; end
        XT =.5*(XTOLD+XT+H*XTD);   YT =.5*(YTOLD+YT+H*YTD);
        XTD=.5*(XTDOLD+XTD+H*XTDD); YTD=.5*(YTDOLD+YTD+H*YTDD);
        XM =.5*(XMOLD+XM+H*XMD);   YM =.5*(YMOLD+YM+H*YMD);
        XMD=.5*(XMDOLD+XMD+H*XMDD); YMD=.5*(YMDOLD+YMD+H*YMDD);
        DELV=.5*(DELVOLD+DELV+H*DELVD);
        S = S + H;

        if QBOOST == 1
            TGOLAM = TFTOT - T;
            [VTX, VTY] = lambert3d(XT,YT,ZT,TGOLAM,XF,YF,ZF,SWITCH1);
            DELVXT = VTX-XTD;  DELVYT = VTY-YTD;
            DELVELT = sqrt(DELVXT^2 + DELVYT^2);
            if (T < TPZ) && (DELVELT > 500.)                % comparison 6
                AXT = ATP*DELVXT/DELVELT;  AYT = ATP*DELVYT/DELVELT;
            elseif DELVELT < 500.
                QBOOST = 0;  AXT = 0.;  AYT = 0.;
                XTD = VTX;  YTD = VTY;
            else
                QBOOST = 0;  AXT = 0.;  AYT = 0.;
            end
        end
        if T < TUPT                                          % comparison 6
            RTMAG = sqrt(XT^2+YT^2);
            AXT = ATP*XT/RTMAG;  AYT = ATP*YT/RTMAG;
        end
        if T >= TLAUNCH
            if P.QPERFECT == 1
                XTF = XTFACT;  YTF = YTFACT;
            elseif (P.QPERFECT == 0) && (P.QTAYLOR == 1)
                TGOM = TF - T;
                XTF = XT + XTD*TGOM + .5*XTDD*TGOM*TGOM;
                YTF = YT + YTD*TGOM + .5*YTDD*TGOM*TGOM;
            end
            ZTF = 0;
        end
        if (T >= TLAUNCH) && (QFIRST == 1)
            QFIRST = 0;  TGOPZ = TF - TLAUNCH;
            [VRX, VRY] = lambert3d(XM,YM,ZM,TGOPZ,XTF,YTF,ZTF,SWITCHM);
            XMD = VRX;  YMD = VRY;
        end

        % ---- comparison 7: the TS-rate stereo measurement + filters -------
        if S >= (TS - .0001)
            S = 0.;
            [z1, z2, st] = gauss2(st);
            [XTS, YTS, THET1, THET2] = measure(XS1,YS1,XS2,YS2,XT,YT, ...
                                    P.SIGTHET1, SIGTHET2, P.BIAS1, z1, z2);
            [SIGX, SIGY] = pseudoSigma(XS1,YS1,XS2,YS2,THET1,THET2, ...
                                       P.SIGTHET1, SIGTHET2);
            XN = XN + 1.;
            XK1 = 3*(3*XN*XN-3*XN+2)/(XN*(XN+1)*(XN+2));
            XK2 = 18*(2*XN-1)/(XN*(XN+1)*(XN+2)*TS);
            XK3 = 60/(XN*(XN+1)*(XN+2)*TS*TS);

            [PX, K] = riccatiStep(PX, PHI, PHIT, Q, HMAT, HT, SIGX^2, IDNPZ);
            if XN < 10., XK1PZ=XK1; XK2PZ=XK2; XK3PZ=XK3;
            else,        XK1PZ=K(1); XK2PZ=K(2); XK3PZ=K(3); end
            RES = XTS - XH - TS*XDH - .5*TS*TS*XDDH;
            XH  = XH + XDH*TS + .5*TS*TS*XDDH + XK1PZ*RES;
            XDH = XDH + XDDH*TS + XK2PZ*RES;
            XDDH= XDDH + XK3PZ*RES;

            [PY, KP] = riccatiStep(PY, PHI, PHIT, Q, HMAT, HT, SIGY^2, IDNPZ);
            if XN < 10., XK1PZP=XK1; XK2PZP=XK2; XK3PZP=XK3;
            else,        XK1PZP=KP(1); XK2PZP=KP(2); XK3PZP=KP(3); end
            RESP = YTS - YH - TS*YDH - .5*TS*TS*YDDH;
            YH  = YH + YDH*TS + .5*TS*TS*YDDH + XK1PZP*RESP;
            YDH = YDH + YDDH*TS + XK2PZP*RESP;
            YDDH= YDDH + XK3PZP*RESP;

            % ---- comparison 4: FILTERED command APN, tgo > THOM, 1/TS rate
            if (T > TGUID) && (TGO > THOM)
                RTM1H = XH-XM;  RTM2H = YH-YM;
                RTMH  = sqrt(RTM1H^2 + RTM2H^2);
                VTM1H = XDH-XMD;  VTM2H = YDH-YMD;
                VCH   = -(RTM1H*VTM1H + RTM2H*VTM2H)/RTMH;
                TGOH  = RTMH/VCH;
                TEMPBOTM = (XM^2+YM^2)^1.5;
                XMDDGRAV = -GM*XM/TEMPBOTM;  YMDDGRAV = -GM*YM/TEMPBOTM;
                ZEM1H = RTM1H + VTM1H*TGOH + .5*(XDDH-XMDDGRAV)*TGOH^2;
                ZEM2H = RTM2H + VTM2H*TGOH + .5*(YDDH-YMDDGRAV)*TGOH^2;
                ZEMDOTRTMH = (ZEM1H*RTM1H + ZEM2H*RTM2H)/RTMH;
                ZEMPER1H = ZEM1H - ZEMDOTRTMH*RTM1H/RTMH;
                ZEMPER2H = ZEM2H - ZEMDOTRTMH*RTM2H/RTMH;
                AXMGUID = XNP*ZEMPER1H/(TGOH^2);
                AYMGUID = XNP*ZEMPER2H/(TGOH^2);
                if P.QGUID == 0, XNCLIM = 0.; end
                [AXMGUID, AYMGUID] = clampAcc(AXMGUID, AYMGUID, XNCLIM);
                nCmd = nCmd + 1;
            end
        end

        if P.RUN == 1
            nrec = nrec + 1;
            if nrec > numel(recT)
                recT(2*end) = 0; recE(2*end) = 0; recS(2*end) = 0;
            end
            recT(nrec) = T;
            recE(nrec) = (XTDD - XDDH)/32.2;      % ERRXTDD, g
            recS(nrec) = sqrt(PX(3,3))/32.2;      % SP33X,   g
        end
    end

    OUT.miss_ft(JJ)   = RTM;
    OUT.divert_ms(JJ) = DELV/3.28;
    if P.RUN == 1
        OUT.single = struct('T', recT(1:nrec), 'err_xtdd_g', recE(1:nrec), ...
                            'sp33x_g', recS(1:nrec));
    end
end
OUT.handover = struct('n_cmd_updates', nCmd, 'n_homing_steps', nHome, ...
                      't_handover_s', tHandover, 'cmd_rate_hz', 1/TS, ...
                      'homing_rate_hz', 1/P.H);
end


% =============================================================================
function [xtf, ytf] = predict34(tp,xtp,ytp,xtdp,ytdp,tf,tftot,tupt,xf,yf,itgt,h,maxsteps)
%PREDICT34  zarchan.txt:13442-13466, printed under Listing 34.3.  Integrates
%   the target forward to tf.  The post-burnout branch keeps qboost = 1 (the
%   book's own text) so LAMBERT3D is called on every step to the end; see the
%   header's book-defect list.
if itgt == 1, tpz = 180; else, tpz = 240; end
t = tp;  switch1 = 0;  xt = xtp;  yt = ytp;  zt = 0.;
xtd = xtdp;  ytd = ytdp;  zf = 0.;
gm = 1.4077E16;  qboost = 1;  axt = 0.;  ayt = 0.;
xtdd = 0.;  ytdd = 0.;  n = 0;
while t <= (tf - .00001)
    n = n + 1;
    if n > maxsteps
        error('mgs:zarchan:stereoTrackingMC:predictRunaway', ...
            'predict34 exceeded %g steps at t = %g s.', maxsteps, t);
    end
    xtold = xt;  ytold = yt;  xtdold = xtd;  ytdold = ytd;
    step = 1;  flag = 0;
    while step <= 1
        if flag == 1
            xt = xt + h*xtd;  yt = yt + h*ytd;
            xtd = xtd + h*xtdd;  ytd = ytd + h*ytdd;
            t = t + h;  step = 2;
        end
        tembot = (xt^2 + yt^2)^1.5;
        xtdd = -gm*xt/tembot + axt;  ytdd = -gm*yt/tembot + ayt;
        if itgt == 1
            if t < 180., wgt=-212.*t+44000.;  trst=54100.;
            else,        wgt=3300.;           trst=0.;      end
        else
            if t < 120,     wgt=-2622*t+440660.; trst=725850.;
            elseif t < 240., wgt=-642.*t+168120.; trst=182250.;
            else,            wgt=5500.;           trst=0.;   end
        end
        atp = 32.2*trst/wgt;  flag = 1;
    end
    xt = (xtold+xt)/2 + .5*h*xtd;   yt = (ytold+yt)/2 + .5*h*ytd;
    xtd = (xtdold+xtd)/2 + .5*h*xtdd;  ytd = (ytdold+ytd)/2 + .5*h*ytdd;
    if qboost == 1
        tgolam = tftot - t;
        [vtx, vty] = lambert3d(xt,yt,zt,tgolam,xf,yf,zf,switch1);
        delvxt = vtx-xtd;  delvyt = vty-ytd;
        delvelt = sqrt(delvxt^2 + delvyt^2);
        if (t < tpz) && (delvelt > 500.)
            axt = atp*delvxt/delvelt;  ayt = atp*delvyt/delvelt;
        elseif delvelt < 500.
            qboost = 1;  axt = 0.;  ayt = 0.;      % book prints 1, not 0
            xtd = vtx;  ytd = vty;
        else
            qboost = 0;  axt = 0.;  ayt = 0.;
        end
        if t < tupt
            rtmag = sqrt(xt^2 + yt^2);
            axt = atp*xt/rtmag;  ayt = atp*yt/rtmag;
        end
    end
end
xtf = xt;  ytf = yt;
end


% =============================================================================
function [XTS, YTS, T1, T2] = measure(XS1,YS1,XS2,YS2,XT,YT,S1,S2,BIAS1,z1,z2)
%MEASURE  The listing's measurement line: two noisy angles, then triangulation.
%   Factored out so that the BIAS1 delete experiment in structuralChecks runs
%   through the SAME code the engagement runs through.  A delete experiment on
%   a re-implementation of the path proves nothing about the path.
%   BIAS1 is on SENSOR 1 ONLY -- the listing adds it to THET1S and not THET2S.
T1  = atan2(YS1-YT, XS1-XT);
T2  = atan2(YS2-YT, XS2-XT);
T1S = T1 + S1*z1 + BIAS1;
T2S = T2 + S2*z2;
[XTS, YTS] = triangulate(XS1, YS1, XS2, YS2, T1S, T2S);
end


% =============================================================================
function [XTS, YTS] = triangulate(XS1, YS1, XS2, YS2, T1S, T2S)
%TRIANGULATE  The listing's TOP1/XTS/TOP2/YTS, which are the book's published
%   closed form for the pseudo measurements, zarchan.txt:13764-13765.  It is an
%   EXACT inverse of theta_i = atan2(ys_i - yT, xs_i - xT); BENCHMARK B7 scores
%   that at machine precision, which is why no tolerance is spent on it.
t1 = tan(T1S);  t2 = tan(T2S);  den = t2 - t1;
TOP1 = XS2*t2 - XS1*t1 + YS1 - YS2;
XTS  = TOP1/den;
TOP2 = XS2*t2*t1 - XS1*t1*t1 + t1*(YS1 - YS2);
YTS  = YS1 - XS1*t1 + TOP2/den;
end


% =============================================================================
function [SIGX, SIGY, D] = pseudoSigma(XS1, YS1, XS2, YS2, T1, T2, S1, S2)
%PSEUDOSIGMA  Chain-rule variance of the pseudo measurements, zarchan.txt:13771-13777
%   (the variance sum) and :13779-13782 (the two xT partials).  The book leaves
%   the yT partials as "found in a similar way" -- they are DERIVED here, so
%   they are checked structurally rather than scored as book values.
t1 = tan(T1);  t2 = tan(T2);  den = t2 - t1;
d1 = (cos(T1)*den)^2;  d2 = (cos(T2)*den)^2;
D.dxdt1 = (t2*(XS2-XS1) + YS1 - YS2)/d1;
D.dxdt2 = (t1*(XS1-XS2) + YS2 - YS1)/d2;
D.dydt1 = -XS1/(cos(T1)*cos(T1)) + ...
          (XS2*t2*t2 - 2.*XS1*t1*t2 + (YS1-YS2)*t2 + XS1*t1*t1)/d1;
D.dydt2 = (t1*t1*(XS1-XS2) - (YS1-YS2)*t1)/d2;
SIGX = sqrt((D.dxdt1*S1)^2 + (D.dxdt2*S2)^2);
SIGY = sqrt((D.dydt1*S1)^2 + (D.dydt2*S2)^2);
end


% =============================================================================
function Q = qMatrix(PHIS, TS)
%QMATRIX  The listing's process-noise matrix.  Selfcheck K1 verifies it against
%   a numerically integrated PHI(tau)*G*G'*PHI(tau)' -- not against itself.
Q = zeros(3);
Q(1,1)=PHIS*TS^5/20;  Q(1,2)=PHIS*TS^4/8;  Q(1,3)=PHIS*TS^3/6;
Q(2,1)=Q(1,2);        Q(2,2)=PHIS*TS^3/3;  Q(2,3)=PHIS*TS*TS/2;
Q(3,1)=Q(1,3);        Q(3,2)=Q(2,3);       Q(3,3)=PHIS*TS;
end


% =============================================================================
function [Pnew, K] = riccatiStep(P, PHI, PHIT, Q, HMAT, HT, Rvar, IDN)
%RICCATISTEP  The listing's M -> HMHTR -> K -> P chain, with the 1x1 inverse
%   done as the listing does it (a reciprocal, not inv()).
M     = PHI*P*PHIT + Q;
HMHTR = HMAT*M*HT + Rvar;
K     = (M*HT)*(1./HMHTR);
Pnew  = (IDN - K*HMAT)*M;
end


% =============================================================================
function [ax, ay] = clampAcc(ax, ay, XNCLIM)
%CLAMPACC  The listing's four limiter comparisons, resolved as "> XNCLIM" by
%   their own "elseif < -XNCLIM" partners.  Per-AXIS, not on the magnitude --
%   clamping the magnitude instead would leave a 1.41*XNCLIM corner reachable.
if ax > XNCLIM, ax = XNCLIM; elseif ax < -XNCLIM, ax = -XNCLIM; end
if ay > XNCLIM, ay = XNCLIM; elseif ay < -XNCLIM, ay = -XNCLIM; end
end


% =============================================================================
function [z1, z2, st] = gauss2(st)
%GAUSS2  Two standard normals from a MINSTD Lehmer stream by Box-Muller.  The
%   listing calls randn twice per sample; a private stream is used instead so
%   the result cannot move when someone else calls rng(), and so this port
%   never reaches into the caller's global stream inside a suite.
m = 2147483647;
st = mod(16807*st, m);  u1 = st/m;
st = mod(16807*st, m);  u2 = st/m;
r  = sqrt(-2*log(u1));
z1 = r*cos(2*pi*u2);
z2 = r*sin(2*pi*u2);
end


% =============================================================================
function v = pct90(x)
%PCT90  The listing's figure of merit: SORT(x) then take element 45 of 50, i.e.
%   the ceil(0.9*RUN)-th order statistic.  BENCHMARK B5 scores the index.
if isempty(x), v = NaN; return; end
s = sort(x(:));
v = s(min(numel(s), max(1, ceil(0.9*numel(s)))));
end


% =============================================================================
function [VRX, VRY, VRZ] = lambert3d(XT, YT, ZT, TF, XF, YF, ZF, SWITCH1)
%LAMBERT3D  zarchan.txt:10428, printed under Listing 28.2 and called by 34.3,
%   34.4 and predict34.  Copied UNCHANGED from MGS.ZARCHAN.STRATEGICENGAGEMENT,
%   whose header records how the bracket-update comparisons were resolved and
%   confirmed against the published iterate sequence of Table 13.2.
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
T_MIN = 0.;
VNUMER  = GMDIVR0*(1. - COS_T);
if SWITCH1 == 0
    G_MIN = HALFPI - acos(R0DOTRF0/(R0MAG*RF0MAG));  G_MAX = HALFPI;
    THETA = acos(COS_T);
else
    G_MIN = -HALFPI;  G_MAX = -HALFPI + acos(R0DOTRF0/(R0MAG*RF0MAG));
    THETA = 2.*PI - acos(COS_T);
end
SIN_T = sin(THETA);  COT_HALFT = 1./tan(THETA/2.);
GAMMA = (G_MAX + G_MIN)/2.;  GOLD = G_MIN;  TOLD = 0.;  T = 0.;  ITERS = 1;
RV0MAG = NaN;  NEXT = GAMMA;
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
if SWITCH1 == 0, ANGLE = HALFPI - GAMMA; else, ANGLE = GAMMA - HALFPI; end
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
VRX = RV0MAG*(C2*V2X + C1*V1X);
VRY = RV0MAG*(C2*V2Y + C1*V1Y);
VRZ = RV0MAG*(C2*V2Z + C1*V1Z);
end


% =============================================================================
function K = structuralChecks(P)
%STRUCTURALCHECKS  Self-consistency only.  NOT book values, and never evidence
%   that this port reproduces Zarchan.  .names and .oks are the same length and
%   are what benchSuite puts on the board -- an invariant with no name is an
%   invariant nobody can see.
names = {};  oks = [];  detail = {};

% A representative geometry: sensors 100 km apart at 15 km, target 400 km
% short of sensor 1, in the listing's own ft units.
A = 2.0926E7;  ALTM = P.ALTMKMIC*3280.;
l1 = (P.XLONGMDEGICKM/111.)/57.3;  l2 = (P.XLONGS2DEGKM/111.)/57.3;
XS1 = (A+ALTM)*cos(l1);  YS1 = (A+ALTM)*sin(l1);
XS2 = (A+ALTM)*cos(l2);  YS2 = (A+ALTM)*sin(l2);
XTt = (A + 60*3280)*cos(0.02);  YTt = (A + 60*3280)*sin(0.02);
th1 = atan2(YS1-YTt, XS1-XTt);   th2 = atan2(YS2-YTt, XS2-XTt);

% K1  Q(TS) against a numerically integrated process-noise matrix.
Qb = qMatrix(P.PHIS, P.TS);
nq = 20001;  tau = linspace(0, P.TS, nq);  Qn = zeros(3);
for k = 1:3
    for j = 1:3
        f = phiCol(tau,k).*phiCol(tau,j);
        Qn(k,j) = P.PHIS*trapzLocal(tau, f);
    end
end
den1 = max(abs(Qb(:)));  if den1 == 0, den1 = 1; end   % PHIS = 0 is legal
e1 = max(abs(Qb(:)-Qn(:)))/den1;
names{end+1} = 'Q(TS) == PHIS*int PHI(tau)*G*G''*PHI(tau)'' dtau';
oks(end+1)   = e1 <= 1e-6;
detail{end+1} = sprintf('max relative deviation %.3e', e1);

% K2  Riccati recursion: P stays symmetric, PSD, and P(3,3) falls from the
%     cold start.  A sign error in (I-KH)M shows up here and nowhere cheap.
PX = zeros(3);  PX(1,1)=99999999999.; PX(2,2)=99999999999.; PX(3,3)=99999999999.;
PHI = zeros(3); PHI(1,1)=1; PHI(1,2)=P.TS; PHI(1,3)=.5*P.TS*P.TS;
PHI(2,2)=1; PHI(2,3)=P.TS; PHI(3,3)=1;
[SIGX0, ~] = pseudoSigma(XS1,YS1,XS2,YS2,th1,th2,P.SIGTHET1,P.SIGTHET1);
p33_0 = PX(3,3);  asym = 0;
for k = 1:60
    PX = riccatiStep(PX, PHI, PHI', Qb, [1 0 0], [1;0;0], SIGX0^2, eye(3));
    asym = max(asym, max(abs(PX(:)-reshape(PX',9,1)))/max(abs(PX(:))));
end
psd = min(eig((PX+PX')/2)) >= -1e-6*max(abs(diag(PX)));
names{end+1} = 'Riccati P stays symmetric + PSD and P(3,3) falls from cold start';
oks(end+1)   = (asym <= 1e-9) && psd && (PX(3,3) < p33_0);
detail{end+1} = sprintf('asym %.2e, P33 %.4g -> %.4g, PSD %d', ...
                        asym, p33_0, PX(3,3), psd);

% K3  DELETE EXPERIMENT on BIAS1, run THROUGH measure() -- the same function
%     the engagement calls.  A bias of db rad must (a) move the triangulated
%     xT by dxdt1*db and (b) leave sensor 2's angle untouched.  This is the
%     only check that would notice BIAS1 wired to the wrong sensor, to both,
%     or to nothing at all.  db = 1e-6 rad, not 1e-8: at 1e-8 the roundoff
%     floor eps*xT/db is ~8e-6 of the derivative and would eat the answer.
db = 1e-6;
[~,~,D] = pseudoSigma(XS1,YS1,XS2,YS2,th1,th2,P.SIGTHET1,P.SIGTHET1);
[xp, ~, ~, t2p] = measure(XS1,YS1,XS2,YS2,XTt,YTt,0,0, db,0,0);
[xm, ~, ~, t2m] = measure(XS1,YS1,XS2,YS2,XTt,YTt,0,0,-db,0,0);
e3 = abs(((xp-xm)/(2*db) - D.dxdt1)/D.dxdt1);
names{end+1} = 'BIAS1 delete experiment: sensor-1 bias moves xT by dxdt1*bias, sensor 2 untouched';
oks(end+1)   = (e3 <= 1e-4) && (t2p == t2m) && (t2p == th2);
detail{end+1} = sprintf('(dx/dbias)/dxdt1 - 1 = %.3e; theta2 shift %.3e rad', ...
                        e3, abs(t2p - th2));

% K4  The y partials are DERIVED here, not published, so they are checked
%     against the book's own yT closed form rather than scored as book values.
%     Same change as B9 and for the same measured reason: complex step instead
%     of central differences, over the 45-geometry sweep instead of the one
%     geometry above.  The old form read 1.639e-09 against a 1e-4 band -- it
%     passed, but on a band five decades looser than the answer, which is how
%     a partial with a real error of 1e-5 would also have passed.  Band 1e-9.
[~, ~, e4, SWk] = algebraResiduals(P);
names{end+1} = 'derived dyT/dtheta partials == d(yT)/d(theta), complex step';
oks(end+1)   = e4 <= 1e-9;
detail{end+1} = sprintf(['max relative deviation %.3e over 45 geometries ' ...
                         '(complex step, h = 1e-200)'], e4);

% K6  THE DISCRIMINATOR ITSELF, SCORED.  B9's whole argument is that the
%     central-difference disagreement was an ARTEFACT OF THE STEP rather than
%     an offset in the partial, and the evidence for that is the SHAPE of the
%     error-vs-step curve.  A shape quoted in a comment is not a check, so it
%     is scored here:
%       (i)   the truncation branch falls faster than linearly in h -- for a
%             central difference it falls as h^2, so two decades of h must buy
%             at least three decades of error (measured: 1.46e+04);
%       (ii)  the round-off branch RISES as h shrinks -- two decades of h must
%             cost at least one decade of error (measured: 7.16e+01);
%       (iii) the V has a floor far below the band the row used to fail
%             (measured minimum 1.98e-07 against the old 1e-5).
%     A genuinely mis-transcribed partial pins the curve flat at its own
%     offset: (i) and (ii) collapse towards 1 and (iii) cannot reach 1e-6.
%     All three must hold, so the row cannot be satisfied by one accident.
rTrunc = SWk.err(4)/SWk.err(6);        % SWk.h(k) = 1e-k, by construction
rRound = SWk.err(13)/SWk.err(11);
[vFloor, iFloor] = min(SWk.err);
names{end+1} = 'central-difference gap is a STEP artefact, not an offset (h^2 and 1/h branches)';
oks(end+1)   = (rTrunc >= 1e3) && (rRound >= 10) && (vFloor <= 1e-6);
detail{end+1} = sprintf(['truncation branch err(1e-4)/err(1e-6) = %.3g ' ...
                         '(h^2 predicts 1e4); round-off branch ' ...
                         'err(1e-13)/err(1e-11) = %.3g (1/h predicts 1e2); ' ...
                         'V floor %.3e at h = %.0e'], rTrunc, rRound, ...
                        vFloor, SWk.h(iFloor));

% K5  DELETE EXPERIMENT on the limiter and on QGUID.  A huge command must come
%     back clamped per axis at XNCLIM with its sign intact; QGUID = 0 sets
%     XNCLIM = 0 and must therefore return exactly zero.
[cx, cy] = clampAcc(1e9, -1e9, P.XNCLIM);
[zx, zy] = clampAcc(1e9, -1e9, 0.);
names{end+1} = 'acceleration limiter clamps per axis at XNCLIM, QGUID=0 zeroes it';
oks(end+1)   = (cx == P.XNCLIM) && (cy == -P.XNCLIM) && (zx == 0) && (zy == 0);
detail{end+1} = sprintf('clamped to %+.1f / %+.1f ft/s^2; QGUID=0 -> %+.1f', cx, cy, zx);

% Every value is wrapped in a 1-by-1 cell.  struct('oks', [true false]) would
% build a 1-by-2 STRUCT ARRAY, and benchSuite would then see .names/.oks on an
% array instead of a scalar -- the exact shape that put twenty invariants on
% zero board rows in an earlier wave.
K = struct('names', {names}, 'oks', {logical(oks)}, 'detail', {detail});
end


% =============================================================================
function c = phiCol(tau, k)
%PHICOL  Row k of PHI(tau)*G for the three-state polynomial model, G = [0;0;1].
switch k
    case 1, c = tau.^2/2;
    case 2, c = tau;
    otherwise, c = ones(size(tau));
end
end


% =============================================================================
function s = trapzLocal(x, y)
s = sum((y(1:end-1) + y(2:end))/2 .* diff(x));
end


% =============================================================================
function B = benchmarks(P, ENG, eng_err, wanted, UNITS)
%BENCHMARKS  Nine scored rows, one INFO, one SKIP.  Every book value below was
%   grepped out of zarchan_decoded.txt and its line number is in .source.
%   Nothing was read off a figure: the extraction turned every figure into an
%   image link, and Figs. 34.21-34.27 are where all of this listing's OUTPUT
%   lives.  That is why the scored rows are setup and algebra, not results.

B = emptyBench();

% B1  Sensor angular accuracy.  The prose renders "50 mr", but the figure label
%     at :13998 reads "50 microradians" and the listing sets 5e-5 rad, so the
%     prose "m" is the extraction of a lost mu.  Both lines cited.
B(end+1) = brow('sensor angular accuracy', 'value', 50e-6, '50e-6 rad (50 urad)', ...
    P.SIGTHET1, sprintf('%.6g rad', P.SIGTHET1), '1e-12 rel', 1e-12, ...
    eq12(P.SIGTHET1, 50e-6), false, '', ...
    'prose zarchan.txt:13892 ("50 mr" = 50 urad), Fig. 34.25 label :13998', ...
    'rad', 'SIGTHET1 = SIGTHET2; the listing prints .00005');

% B2  Filter sampling time / measurement rate.
B(end+1) = brow('filter sampling time TS', 'value', 0.1, '0.1 s (10 samples/s)', ...
    P.TS, sprintf('%.6g s', P.TS), '1e-12 rel', 1e-12, ...
    eq12(P.TS, 0.1), false, '', ...
    'prose zarchan.txt:13892 and :13784 ("10 times per second")', ...
    's', 'sets PHI, Q and the least-squares gains XK1..XK3');

% B3  Sensor altitude.
B(end+1) = brow('sensor altitude', 'value', 15, '15 km', ...
    P.ALTMKMIC, sprintf('%.6g km', P.ALTMKMIC), '1e-12 rel', 1e-12, ...
    eq12(P.ALTMKMIC, 15), false, '', 'prose zarchan.txt:13892', ...
    'km', 'ALTMKMIC; both sensors and the interceptor share it');

% B4  Declared intercept time.  QFIX = 0 lets the search loop overwrite TF, so
%     this row scores the DECLARED value and the searched one is an INFO row.
B(end+1) = brow('declared intercept time TF', 'value', 230, '230 s', ...
    P.TFDES, sprintf('%.6g s', P.TFDES), '1e-12 rel', 1e-12, ...
    eq12(P.TFDES, 230), false, '', ...
    'listing line 1 zarchan.txt:13788, confirmed in prose :13892', ...
    's', 'overwritten by the TF search when QFIX = 0');

% B5  Kalman filter process noise.  The book runs two: 576 and 9.
B(end+1) = brow('Kalman process noise PHIS', 'value', 576, '576', ...
    P.PHIS, sprintf('%.6g', P.PHIS), '1e-12 rel', 1e-12, ...
    eq12(P.PHIS, 576), false, '', ...
    'prose zarchan.txt:13893 (prose "Fs = 576", raw ligature) + Fig. 34.21 label :13912; Fs = 9 at :13893/:13938', ...
    'ft^2/s^5', 'pass ''PHIS'', 9 for the Fig. 34.22/34.24 filter');

% B6  Homing handover.
B(end+1) = brow('homing handover THOM', 'value', 10, '10 s before intercept', ...
    P.THOM, sprintf('%.6g s', P.THOM), '1e-12 rel', 1e-12, ...
    eq12(P.THOM, 10), false, '', 'prose zarchan.txt:13784', ...
    's', 'splits filtered command APN (tgo > THOM) from true-state APN');

% B7  The 90% point.  "90% of the flights had required divert that was less
%     than the amount shown" over a 50-run set is order statistic 45, which is
%     exactly the listing's RTMSORT(45) / DELVSORT(45).
idx50 = ceil(0.9*50);
B(end+1) = brow('90% point index in a 50-run set', 'value', 45, ...
    '45 (RTMSORT(45), DELVSORT(45))', idx50, sprintf('%d', idx50), ...
    'exact', 0, idx50 == 45, false, '', ...
    'listing zarchan.txt:13888-13890, definition in prose :13919-13963', ...
    'index', 'pct90() takes the ceil(0.9*RUN)-th order statistic');

% B8  Triangulation.  The book prints the closed form for xT and yT; it must
%     invert theta1, theta2 EXACTLY, so this is scored at machine precision and
%     not with a tolerance chosen to be comfortable.
[eTri, eDx, ~, SW] = algebraResiduals(P);
B(end+1) = brow('triangulation inverts the two angle equations', 'shape', 0, ...
    'exact (closed-form solution)', eTri, sprintf('%.3e', eTri), '1e-12 rel', ...
    1e-12, eTri <= 1e-12, false, '', ...
    'published xT, yT at zarchan.txt:13764-13765', ...
    'relative', 'worst case over a sweep of target positions along the baseline');

% B9  The published xT partials, differenced WITHOUT SUBTRACTION.
%
%   *** THIS ROW USED TO BE RED, AND THE PARTIALS WERE NEVER THE REASON. ***
%   It read 3.1648e-05 against a 1e-5 band using central differences at
%   h = 1e-6 rad.  Sweeping h over fourteen decades settles it (SW below, and
%   selfcheck K6 scores the shape):
%
%       h      1e-4     1e-5     1e-6     1e-7     1e-8     1e-9     1e-10
%     err    4.6e-01  3.2e-03  3.2e-05  3.2e-07  2.0e-07  1.7e-06  2.4e-05
%
%   That is a V, not a plateau.  From 1e-4 to 1e-7 the error falls a hundred-
%   fold per decade of h -- exactly the h^2 of a central difference -- and
%   below 1e-8 it RISES tenfold per decade, exactly the eps*|xT|/h of
%   subtractive cancellation on numbers of order 2e7 ft.  A mis-transcribed
%   partial is a CONSTANT offset: it would floor the curve flat at its own
%   size and neither branch would appear.  So the disagreement was the
%   differencing, and the old h = 1e-6 was sitting two decades up the
%   truncation branch of its own V.
%
%   (The comment that chose it argued 1e-8 would be too noisy, "roundoff floor
%   eps*xT/db is ~8e-6 of the derivative".  h = 1e-8 is in fact the MINIMUM of
%   the sweep at 1.98e-07 -- it would have passed the old band by a factor of
%   50.  The estimate was an order-of-magnitude bound used as if it were the
%   error.)
%
%   The fix is not a bigger band, it is a derivative with no subtraction in
%   it: the COMPLEX STEP, f'(x) = Im f(x + ih)/h.  triangulate() is built from
%   tan, +, -, * and / only, so it is analytic and the identity is exact; with
%   no difference of nearby reals there is no cancellation and no step to
%   tune.  Measured 1.9462e-11, and FLAT from h = 1e-20 to h = 1e-200 -- 180
%   decades in which a differencing artefact could not possibly stay put.
%
%   The band is therefore TIGHTENED from 1e-5 to 1e-9: four orders of
%   magnitude stricter than the band this row used to fail, and still two
%   decades above the measured 1.9e-11 floor, which is set by the
%   conditioning of the closed form itself and not by the derivative.
B(end+1) = brow('published dxT/dtheta partials == d(xT)/d(theta)', 'shape', 0, ...
    'chain rule, exact', eDx, sprintf('%.3e', eDx), '1e-9 rel', 1e-9, ...
    eDx <= 1e-9, false, '', ...
    'published partials at zarchan.txt:13779-13782, variance sum :13771-13777', ...
    'relative', sprintf(['complex step Im f(t+ih)/h, h = 1e-200, over 45 ' ...
    'geometries -- no subtraction, so no step to tune.  Central differences ' ...
    'on the same grid: %.3e at h = 1e-6 (the old setting), best %.3e at ' ...
    'h = %.0e.  y partials are in R.selfcheck'], ...
    SW.err(6), min(SW.err), SW.h(find(SW.err == min(SW.err), 1))));

% B10  The engagement itself -- INFO, because nothing it produces is published
%      as a number.  A failure is reported as a RED row, not swallowed.
if ~wanted
    B(end+1) = brow('engagement (miss, divert)', 'info', NaN, 'figures only', ...
        NaN, 'not run', 'n/a', NaN, false, true, ...
        '''engage'' is false; the nine scored rows above do not need it', ...
        'Figs. 34.25-34.27, zarchan.txt:13963-14060', UNITS, '');
elseif ~isempty(eng_err)
    B(end+1) = brow('engagement (miss, divert)', 'value', NaN, 'figures only', ...
        NaN, 'ERRORED', 'n/a', NaN, false, false, eng_err, ...
        'Figs. 34.25-34.27, zarchan.txt:13963-14060', UNITS, ...
        'the transcription threw; see R.p.engage_error');
else
    B(end+1) = brow('engagement (miss, divert)', 'info', NaN, 'figures only', ...
        pct90(ENG.divert_ms), sprintf('miss %.1f ft, divert %.1f m/s, TF %.0f s', ...
        pct90(ENG.miss_ft), pct90(ENG.divert_ms), ENG.TF), 'n/a', NaN, ...
        true, true, 'no number is published for the printed 400/500-km case', ...
        'Figs. 34.25-34.27, zarchan.txt:13963-14060', UNITS, ...
        sprintf('%d run(s), %d filtered commands, %d homing steps', ...
        numel(ENG.miss_ft), ENG.handover.n_cmd_updates, ENG.handover.n_homing_steps));
end

% B11  SKIP, with the arithmetic.  Not absorbed into a wider band anywhere.
B(end+1) = brow('"divert increases more than 500 m/s with noise"', 'skip', 500, ...
    '> 500 m/s', NaN, 'not scoreable', 'n/a', NaN, false, true, ...
    ['it is a DIFFERENCE between two curves of Fig. 34.25 (noise+filtering ' ...
     'at Fs = 9 minus no-noise-or-filtering), and neither curve publishes a ' ...
     'number.  Reproducing it needs 4 configurations x ~5 downranges x 50 ' ...
     'runs = ~1000 full engagements, and would still be scored against a ' ...
     'figure reading.'], 'prose zarchan.txt:13963', 'm/s', '');
end


% =============================================================================
function [eTri, eDx, eDy, SW] = algebraResiduals(P)
%ALGEBRARESIDUALS  Rows B8/B9 and selfchecks K4/K6.  Sweeps the target along
%   the baseline so a single lucky geometry cannot carry any of them.
%
%   eTri  triangulation inverse residual (B8)
%   eDx   published dxT/dtheta partials vs the derivative of the published xT
%   eDy   derived    dyT/dtheta partials vs the derivative of the published yT
%   SW    .h/.err -- the CENTRAL-DIFFERENCE error as a function of step, kept
%         because it is the only thing that distinguishes "the partial is
%         wrong" from "the differencing is"; K6 scores its shape.
%
%   *** WHY THE DERIVATIVES ARE COMPLEX-STEPPED AND NOT DIFFERENCED. ***
%   xT is of order 2e7 ft and dxT/dtheta of order 4e6 ft/rad, so a central
%   difference subtracts two numbers that agree in their leading digits and
%   divides by 2h.  Its error is h^2*|f'''|/6 + eps*|xT|/h: a V in h with a
%   floor around 2e-07 near h = 1e-8, nowhere near the machine precision the
%   chain rule deserves to be checked at.  triangulate() is analytic in both
%   angles (tan, +, -, *, / and nothing else), so
%
%       f'(x) = Im f(x + i*h) / h    exactly, for any h,
%
%   with no difference of nearby reals anywhere in it.  h = 1e-200 is not a
%   small step, it is a scale factor -- and the residual not moving across
%   180 decades of it is the evidence that no step-dependent error remains.
A = 2.0926E7;  ALTM = P.ALTMKMIC*3280.;
l1 = (P.XLONGMDEGICKM/111.)/57.3;  l2 = (P.XLONGS2DEGKM/111.)/57.3;
XS1 = (A+ALTM)*cos(l1);  YS1 = (A+ALTM)*sin(l1);
XS2 = (A+ALTM)*cos(l2);  YS2 = (A+ALTM)*sin(l2);

lts  = linspace(0.002, 0.030, 15);
alts = [20 100 300];
ch   = 1e-200;
% INDEX k IS THE DECADE: SW.h(k) = 1e-k, so callers index by decade and
% never compare a float to a literal (10.^(-(1:14)) is not bit-equal to 1e-6,
% and `SW.h == 1e-6` selects nothing at all -- silently, as an empty).
SW.h = 10.^(-(1:14));
SW.err = zeros(size(SW.h));
eTri = 0;  eDx = 0;  eDy = 0;

for lt = lts
    for altkm = alts
        XTt = (A + altkm*3280)*cos(lt);  YTt = (A + altkm*3280)*sin(lt);
        t1 = atan2(YS1-YTt, XS1-XTt);  t2 = atan2(YS2-YTt, XS2-XTt);
        [xs, ys] = triangulate(XS1,YS1,XS2,YS2,t1,t2);
        eTri = max(eTri, max(abs(xs-XTt), abs(ys-YTt))/sqrt(XTt^2+YTt^2));
        [~,~,D] = pseudoSigma(XS1,YS1,XS2,YS2,t1,t2,P.SIGTHET1,P.SIGTHET1);

        % Complex step, one perturbed angle at a time.
        [cx1, cy1] = triangulate(XS1,YS1,XS2,YS2,complex(t1,ch),t2);
        [cx2, cy2] = triangulate(XS1,YS1,XS2,YS2,t1,complex(t2,ch));
        eDx = max([eDx, abs((imag(cx1)/ch - D.dxdt1)/D.dxdt1), ...
                        abs((imag(cx2)/ch - D.dxdt2)/D.dxdt2)]);
        eDy = max([eDy, abs((imag(cy1)/ch - D.dydt1)/D.dydt1), ...
                        abs((imag(cy2)/ch - D.dydt2)/D.dydt2)]);

        % The central-difference sweep, on the SAME geometries and against the
        % SAME published partials, so K6's shape argument is about this row.
        for m = 1:numel(SW.h)
            dh = SW.h(m);
            xa = triangulate(XS1,YS1,XS2,YS2,t1+dh,t2);
            xb = triangulate(XS1,YS1,XS2,YS2,t1-dh,t2);
            xc = triangulate(XS1,YS1,XS2,YS2,t1,t2+dh);
            xd = triangulate(XS1,YS1,XS2,YS2,t1,t2-dh);
            SW.err(m) = max([SW.err(m), ...
                             abs(((xa-xb)/(2*dh) - D.dxdt1)/D.dxdt1), ...
                             abs(((xc-xd)/(2*dh) - D.dxdt2)/D.dxdt2)]);
        end
    end
end
end


% =============================================================================
function t = eq12(a, b)
%EQ12  "the same number", with one part in 1e12 of slack for the fact that a
%   decimal literal is not a binary one.  It is NOT a tolerance on physics:
%   every row that uses it compares a setup constant against the same setup
%   constant printed in the book's prose, and any real disagreement is a
%   change in the LAST significant figure, i.e. ~1e-3 relative at worst.
t = abs(a - b) <= 1e-12*max(1, abs(b));
end


% =============================================================================
function b = brow(what, kind, book, book_str, ours, ours_str, band, tol, ...
                  ok, skipped, why, source, units, detail)
%BROW  THE ONLY row constructor.  benchSuite's adaptGeneric is strict about
%   field ORDER as well as field names, so every row is built here or nowhere.
if isnumeric(book) && isscalar(book) && isfinite(book) && book ~= 0 && ...
        isnumeric(ours) && isscalar(ours) && isfinite(ours)
    pct = 100*(ours - book)/abs(book);
else
    pct = NaN;
end
b = struct('what', what, 'kind', kind, 'book', book, 'book_str', book_str, ...
    'ours', ours, 'ours_str', ours_str, 'pct', pct, 'band', band, 'tol', tol, ...
    'ok', logical(ok), 'skipped', logical(skipped), 'why', why, ...
    'source', source, 'units', units, 'detail', detail);
end


% =============================================================================
function B = emptyBench()
B = brow('', 'info', NaN, '', NaN, '', '', NaN, false, true, '', '', '', '');
B(1) = [];
end


% =============================================================================
function s = render(R, ev)
L = {};
L{end+1} = 'mgs.zarchan.stereoTrackingMC -- Zarchan Listing 34.4';
L{end+1} = '  two-dimensional Monte Carlo engagement using STEREO TRACKING';
L{end+1} = sprintf('  units: %s', R.units);
L{end+1} = sprintf('  PHIS %g, SIGTHET %g rad, TS %g s, H %g s, RUN %d, ITGT %d', ...
    R.p.PHIS, R.p.SIGTHET1, R.p.TS, R.p.H, R.p.RUN, R.p.ITGT);
if ~isempty(R.miss_ft)
    L{end+1} = sprintf('  TF used %.0f s; 90%% miss %.1f ft, 90%% divert %.1f m/s', ...
        R.TF_used, R.pct90.miss_ft, R.pct90.divert_ms);
elseif ~isempty(R.p.engage_error)
    L{end+1} = sprintf('  ENGAGEMENT ERRORED: %s', R.p.engage_error);
else
    L{end+1} = '  engagement not run (''engage'' false)';
end
L{end+1} = '  ---- benchmarks -------------------------------------------------';
for k = 1:numel(R.bench)
    b = R.bench(k);
    if b.skipped, mark = 'SKIP'; elseif b.ok, mark = ' ok '; else, mark = 'FAIL'; end
    L{end+1} = sprintf('  [%s] %-46s book %-28s ours %s', ...
        mark, b.what, b.book_str, b.ours_str); %#ok<AGROW>
end
L{end+1} = '  ---- structural (self-consistency, NOT book values) -------------';
for k = 1:numel(R.selfcheck.names)
    if R.selfcheck.oks(k), mark = ' ok '; else, mark = 'FAIL'; end
    L{end+1} = sprintf('  [%s] %-58s %s', mark, R.selfcheck.names{k}, ...
        R.selfcheck.detail{k}); %#ok<AGROW>
end
L{end+1} = sprintf('  scored %d row(s); pass = %d', sum(ev), R.pass);
s = strjoin(L, newline);
end
