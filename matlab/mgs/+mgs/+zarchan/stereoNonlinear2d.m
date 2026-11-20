function R = stereoNonlinear2d(opts)
%MGS.ZARCHAN.STEREONONLINEAR2D  Zarchan Listing 34.3 -- two-dimensional
%   NONLINEAR interceptor/IRBM engagement, round Earth, boost-phase intercept.
%
%   R = mgs.zarchan.stereoNonlinear2d()
%   R = mgs.zarchan.stereoNonlinear2d('taylorcase', true)   % + Table 34.1 row 1
%   R = mgs.zarchan.stereoNonlinear2d('assert', false)
%
%   Line-by-line port of Listing 34.3 (zarchan.txt:13373-13465) including its
%   predict34.m subroutine, and of LAMBERT3D/distance3dkm (Listing 28.3, reused
%   from mgs.zarchan.impulsiveIcbm -- see that file for their own nine OCR
%   resolutions).  Target: single-stage generic IRBM (ITGT = 1), 2000-km lofted
%   trajectory, 180-s burn.  Interceptor: impulsively air-launched at 90 s from
%   15 km, APN guidance from 110 s, intercept designed for 170 s.
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, g = 32.2.  A = 2.0926e7 ft,
%   GM = 1.4077e16 ft^3/s^2, km = 3280 ft, m = 3.28 ft (the book's own
%   conversions, both slightly off SI and both kept).  Divert is reported in
%   m/s because the book reports it that way -- via the listing's own final
%   line, DELV/3.28.  Nothing here is SI-converted silently.
% ===========================================================================
%
%   THE COMPARISONS.  The transcription's header reports SEVEN unresolved
%   variable-to-variable comparisons; twelve in total had to be settled (the
%   rest are variable-to-literal).  In this book's OCR "," is "<" and "." is
%   ">"; each was read off the DECODED book and then justified from shape.
%
%     :13389  ~((T . (TF-10.)) & VC , 0.)  ->  ~((T > TF-10) & (VC < 0))
%             DECISIVE, not a preference.  The target climbs away from the
%             stationary interceptor, so VC is NEGATIVE from about t = 0.01 s
%             until launch.  With "T < TF-10" the loop exits on the SECOND
%             step and the program still "runs", printing a 0.01-s engagement.
%             The T guard exists precisely to survive that opening phase and
%             arm the VC<0 test only near intercept.
%     :13403  if T . TGUID  ->  T > TGUID.  Prose: guidance "is turned on at
%             110 s" (:13468) and Fig. 34.9 plots KKV acceleration from 110 s
%             on.  The else branch zeroes the command, which must be the
%             BEFORE state.  Inverting it is SELFCHECK 1.
%     :13405  if AXMGUID . XNCLIM  ->  ">"; likewise AYMGUID.  Its own elseif
%             (AXMGUID < -XNCLIM) survived OCR, so the pair is a symmetric
%             limiter and the missing half is the upper clamp.
%     :13411  if T . TLAUNCH  ->  T > TLAUNCH.  The else branch sets
%             XMDD = YMDD = 0; the interceptor is stationary before launch and
%             flies afterwards, not the reverse.  Caught by SELFCHECK 3.
%     :13417  if (T , TPZ & DELVELT . 500.)  ->  T < TPZ, DELVELT > 500.
%             TPZ = 180 s is powered-flight end, so "still burning" is T<TPZ;
%             the elseif is DELVELT < 500, so the if-branch is its complement.
%             Listing 34.2 carries the identical construct at :13332.
%     :13423  if T , TUPT  ->  T < TUPT.  TUPT = 15 s is the vertical-rise
%             time: thrust along the radius vector for the first 15 s only.
%             ">" flies radially forever -- no 2000-km lofted trajectory.
%     :13433  if S . = (TS-.0001)  ->  ">=".  S is zeroed on every record, so
%             "<=" records every step and TS would mean nothing.
%     :13444  while t , = (tf-.00001) -> "<=", "while step ,=1" -> "<=", and
%             predict34 repeats the TPZ/500/TUPT trio, resolved identically.
%
%   FAITHFUL WARTS, kept and NOT tidied (each is a thing a reader would "fix",
%   thereby changing the answer):
%     * predict34's boost-exhausted branch sets qboost=1 where the main loop
%       sets QBOOST=0, so the predictor re-snaps the target onto the Lambert
%       arc every step instead of once.  Idempotent on an exact Keplerian arc,
%       so the two still agree -- but it is a real difference, and it is the
%       predictor's, not ours.
%     * predict34 nests its TUPT block INSIDE "if qboost==1"; the main loop
%       does not.  Immaterial while qboost is 1 (t < 15 s), kept anyway.
%     * "TRST=0." inside the boost-end branches is dead: the next derivative
%       pass recomputes WGT and TRST from the time schedule.  Kept.
%     * The book's predict34.m declares "function [xtf,ytf]=predict(...)";
%       MATLAB dispatches on the file name, so the book's own program works.
%     * KEPT because they are executable lines whose presence a reader could
%       mistake for meaning: XTDOLD/XMDOLD written after the corrector,
%       TGOLAMM and TGO each computed twice, ATP=1 overwritten before it is
%       read, PIPERR/QLAUNCH/QOOMPH/TBOT assigned and never consulted.
%       ZEMPLOSG is kept and returned as R.nominal.zemplos_km -- the book's
%       name says "G" but the line divides by 3280, so it is KILOMETRES.
%     * DROPPED, and this is the only place they are recorded: the book's
%       dead initialisers XTINIT/YTINIT/RTINIT/XMINIT/YMINIT/RMINIT/AXM/AYM/
%       AMP/ACC/PREDERRKM/PIPERRKM/DELVELM/PIPKMBO/ZEMPERPTOT/DISTRTKMIC, and
%       the record block's unplotted VTK/ATG/VMKM/VTKM/DELVKM/ACCNEWG/
%       AXMGUIDG/AYMGUIDG.  Every one is written once and never read by any
%       branch, so none can change a number; they are arithmetic, not state.
%   NOT kept: a step-count cap (opts.maxsteps).  The endgame integrates at
%   H = 1e-5 s, so a stop condition that never fires is a hang, not an error.
%
%   BENCHMARKS: see benchmarks() below; every book value carries its
%   zarchan.txt line.  Nothing was read off a figure -- the extraction turned
%   every figure into an image link.  The two "approximately N g" rows are
%   PROSE about Fig. 34.9 (:13471), which is greppable, and their bands come
%   from the word "approximately" plus the printed precision, not from us.
%
%   WHAT THIS SHOULD PRINT.  *** DERIVED / PUBLISHED, NOT MEASURED. ***  This
%   file has never been executed; MATLAB is the caller's.  Hand arithmetic:
%   XNCLIM/32.2 = 10.000 g exactly, matching the prose's "10-g limit"
%   (:13521); the stand-off is A*(400/111)/57.3 = 401.2 km against a nominal
%   "400 km"; TFTOT = 252+.223*2000-5.44e-6*2000^2+200 = 876.24 s.  From the
%   book: divert 208 m/s, peak target accel perpendicular to LOS 3.6 g, peak
%   KKV accel 0.7 g, launch speed 3.4 km/s.  The residuals against those four
%   are UNKNOWN until this is run.  A residual near a band edge is not a pass
%   to be accepted, and no band here may be widened to make a row green.
%
%   OUTPUTS
%     R.nominal   perfect-PIP run: .t .distrt_km .alttkm .distrm_km .altmkm
%                 .atplos_g .xncplos_g .pipplos_km .pip_km .rtm_ft .vc
%                 .energy .acmd_g .accperp_g, plus .delv_mps .vmrqd_kms
%                 .t_end .rtm_end_ft .max_atplos_g .max_xncplos_g
%     R.taylor    same for the three-term-Taylor PIP case, or []
%     R.bench     benchmark rows (schema shared with mgs.zarchan.benchSuite)
%     R.selfcheck .names/.oks/.detail -- structural, NOT book values
%     R.pass .verdict .p .summary .units
%
%   See also MGS.ZARCHAN.IMPULSIVEICBM, MGS.ZARCHAN.ICBMTRAJECTORY,
%   MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 34.3 line 1, verbatim (zarchan.txt:13374).
    opts.TLAUNCH  (1,1) double {mustBeFinite} = 90.0     % s
    opts.TF       (1,1) double {mustBeFinite} = 170.0    % s, desired intercept
    opts.TS       (1,1) double {mustBeFinite, mustBePositive} = 1.0   % s
    opts.XLONGMDEGICKM (1,1) double {mustBeFinite} = 400.0   % km stand-off
    opts.RDESKM   (1,1) double {mustBeFinite, mustBePositive} = 2000.0
    opts.GAMDEG   (1,1) double {mustBeFinite} = 89.99    % deg
    opts.TUPT     (1,1) double {mustBeFinite} = 15.0     % s vertical rise
    opts.TGUID    (1,1) double {mustBeFinite} = 110.0    % s guidance on
    opts.XNCLIM   (1,1) double {mustBeFinite} = 322.0    % ft/s^2 = 10 g
    opts.XNP      (1,1) double {mustBeFinite} = 3.0
    opts.TLOFT    (1,1) double {mustBeFinite} = 200.0    % s loft bias
    opts.ALTMKMIC (1,1) double {mustBeFinite} = 15.0     % km launch altitude
    opts.ITGT     (1,1) double {mustBeMember(opts.ITGT,[1 2])} = 1   % 1 = IRBM
    opts.DELTF    (1,1) double {mustBeFinite} = 0.0
    opts.QGUID    (1,1) double {mustBeMember(opts.QGUID,[0 1])} = 1
    opts.H        (1,1) double {mustBeFinite, mustBePositive} = 0.01
    opts.HFINE    (1,1) double {mustBeFinite, mustBePositive} = 0.00001
    opts.RFINE    (1,1) double {mustBeFinite, mustBePositive} = 1000.0   % ft
    % Not the book's: without it a stop condition that never fires is a hang
    % at 1e-5 s per step.  The nominal case needs of order 24,000 steps.
    opts.maxsteps (1,1) double {mustBeFinite, mustBePositive} = 400000
    % Table 34.1 row 1 (400 km, tF = 170 s) with the three-term Taylor PIP.
    % OFF by default: it is a SECOND full engagement plus a second predict34
    % sweep, so it roughly doubles the cost of this port, and the nominal
    % perfect-PIP case owns the chapter's four headline numbers.  Turning it
    % on scores two more published values (PIP 20,400 m, divert 570 m/s).
    opts.taylorcase (1,1) logical = false
    opts.benchmark  (1,1) logical = true
    opts.render     (1,1) logical = true
    opts.assert     (1,1) logical = true
end

A     = 2.0926E7;      % ft, Earth radius (the book's value)
GM    = 1.4077E16;     % ft^3/s^2
G     = 32.2;          % ft/s^2
FTPKM = 3280.0;        % ft per km  (the book's, not 3280.84)
FTPM  = 3.28;          % ft per m   (the listing's final line, DELV/3.28)

P = opts;
P.QPERFECT = 1;  P.QTAYLOR = 1;  P.QFIX = 1;
S1 = engagement(P, A, GM, G, FTPKM, FTPM);

S2 = [];
if opts.taylorcase
    P2 = P;  P2.QPERFECT = 0;   % QFIX stays 1, at the table's published tF
    S2 = engagement(P2, A, GM, G, FTPKM, FTPM);
end

R = struct();
R.nominal = S1;
R.taylor  = S2;
R.units   = ['ft, ft/s, ft/s^2; g = 32.2 ft/s^2; A = 2.0926e7 ft, ' ...
             'GM = 1.4077e16 ft^3/s^2; km = 3280 ft, m = 3.28 ft (book''s own)'];
R.p = struct('TLAUNCH',opts.TLAUNCH, 'TF',opts.TF, 'TGUID',opts.TGUID, ...
    'XLONGMDEGICKM',opts.XLONGMDEGICKM, 'RDESKM',opts.RDESKM, ...
    'ALTMKMIC',opts.ALTMKMIC, 'XNP',opts.XNP, 'XNCLIM',opts.XNCLIM, ...
    'XNCLIM_g',opts.XNCLIM/G, 'ITGT',opts.ITGT, 'H',opts.H, ...
    'HFINE',opts.HFINE, 'RFINE',opts.RFINE, 'TS',opts.TS, ...
    'taylorcase',opts.taylorcase, 'units',R.units);

R.selfcheck = structuralChecks(S1, opts.TGUID, opts.TLAUNCH, G);

if opts.benchmark
    R.bench = benchmarks(S1, S2, opts, G, FTPM);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    R.pass  = false;    % NOT CHECKED must never render as fine
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false: nothing was compared against the book.';
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
        error('mgs:zarchan:stereoNonlinear2d:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN, with R complete, so the caller can see WHICH side
        % of a comparison moved.  benchSuite runs every port with assert off.
        warning('mgs:zarchan:stereoNonlinear2d:benchmark', ...
            '%s  (''assert'' is false; R is returned complete, see R.verdict)', msg);
    end
end
end


% =============================================================================
function S = engagement(P, A, GM, G, FTPKM, FTPM) %#ok<INUSD>
%ENGAGEMENT  Listing 34.3's main program, zarchan.txt:13374-13440.
%   Every constant arrives as an ARGUMENT: MATLAB local functions cannot see
%   the caller's workspace, and a previous wave in this package lost a day to
%   exactly that.  (G is carried for symmetry with the rest of the package;
%   the listing hard-codes 32.2 at every use and this port keeps it there.)

TLAUNCH=P.TLAUNCH; TF=P.TF; TS=P.TS; XLONGMDEGICKM=P.XLONGMDEGICKM;
RDESKM=P.RDESKM; GAMDEG=P.GAMDEG; TUPT=P.TUPT; TGUID=P.TGUID;
XNCLIM=P.XNCLIM; XNP=P.XNP; QPERFECT=P.QPERFECT; TLOFT=P.TLOFT;
ALTMKMIC=P.ALTMKMIC; QTAYLOR=P.QTAYLOR; QGUID=P.QGUID; ITGT=P.ITGT;
SWITCH1=0; SWITCHM=0; DELTF=P.DELTF; QFIX=P.QFIX;
if ITGT==1, TPZ=180.; else, TPZ=240.; end
ALTM=ALTMKMIC*FTPKM;
TFTOT=252.+.223*RDESKM-(5.44E-6)*RDESKM*RDESKM;  TFTOT=TFTOT+TLOFT;
W=0.;
XLONGFDEG=57.3*RDESKM*FTPKM/A;
XLONGMDEGIC=XLONGMDEGICKM/111.;  XLONGMDEG=XLONGMDEGIC;
QLAUNCH=0; QFIRST=1; XLONGTDEG=0.; QBOOST=1; QOOMPH=1; %#ok<NASGU>
T=0.; S_=0.; AXT=0.; AYT=0.;
XLONGF=XLONGFDEG/57.3;  XLONGF=XLONGF-W*TFTOT;
XF=A*cos(XLONGF); YF=A*sin(XLONGF); ZF=0;
XLONGT=XLONGTDEG/57.3;  XLONGM=XLONGMDEG/57.3;
XT=A*cos(XLONGT); YT=A*sin(XLONGT);
XM=(A+ALTM)*cos(XLONGM); YM=(A+ALTM)*sin(XLONGM); ZM=0;
XFIRST=XT; YFIRST=YT; ZT=0; ZFIRST=0;
XTD=cos(1.5708-GAMDEG/57.3); YTD=sin(1.5708-GAMDEG/57.3);
ATP=1.; H=P.H; XMD=0.; YMD=0.;   % ATP=1 is the book's; the first derivative
                                 % pass overwrites it before anything reads it
RTM1=XT-XM; RTM2=YT-YM; RTM=sqrt(RTM1^2+RTM2^2);
VTM1=XTD-XMD; VTM2=YTD-YMD;
VC=-(RTM1*VTM1+RTM2*VTM2)/RTM;
DELV=0.; AXMGUID=0.; AYMGUID=0.; ZEM1=0.; ZEM2=0.; ACCNEW=0.; TBOT=0.;
% Derivatives pre-seeded.  The derivative pass always runs before the
% corrector reads them, so this changes nothing; it only stops a future edit
% from reading a value left over from somewhere else.
XTDD=0.; YTDD=0.; XMDD=0.; YMDD=0.; DELVD=0.; ACCPERPTOT=0.;
XTF=NaN; YTF=NaN; ZTF=0; ZTFACT=0.;    % ZTFACT: the book sets it only on the
                                       % QFIX branch and reads it on both
maxResid = 0;  maxIters = 0;  nLam = 0;

% ---- the predicted intercept point ---------------------------------------
if QFIX==1
    [XTFACT,YTFACT,pr,pit] = predict34(T,XT,YT,XTD,YTD,TF,TFTOT,TUPT,XF,YF, ...
                                       ITGT,A,GM,P.H);
    maxResid=max(maxResid,pr); maxIters=max(maxIters,pit);
    TGOLAM=TF-TLAUNCH;
    [VRX,VRY,~,it,rs]=lambert3d(XM,YM,ZM,TGOLAM,XTFACT,YTFACT,ZTFACT,SWITCHM,GM);
    maxResid=max(maxResid,rs); maxIters=max(maxIters,it); nLam=nLam+1;
    VMRQDKM=sqrt(VRX^2+VRY^2)/FTPKM;
else
    % The book reassigns the loop variable TF itself and leans on its value
    % after "break"; TFc keeps that explicit.  Ascending TF means the FIRST
    % feasible one is the EARLIEST, which is what the prose asks for (:13566,
    % "the earliest boost-phase intercept time was selected, assuming the
    % interceptor speed could be no greater than 4 km/s").
    for TFc=(TLAUNCH+30.):10:(TPZ-10.)
        [XTFACT,YTFACT,pr,pit] = predict34(T,XT,YT,XTD,YTD,TFc,TFTOT,TUPT, ...
                                           XF,YF,ITGT,A,GM,P.H);
        maxResid=max(maxResid,pr); maxIters=max(maxIters,pit);
        TGOLAM=TFc-TLAUNCH;
        [VRX,VRY,~,it,rs]=lambert3d(XM,YM,ZM,TGOLAM,XTFACT,YTFACT,ZTFACT, ...
                                    SWITCHM,GM);
        maxResid=max(maxResid,rs); maxIters=max(maxIters,it); nLam=nLam+1;
        VMRQDKM=sqrt(VRX^2+VRY^2)/FTPKM;
        TF=TFc;
        if VMRQDKM < 4.
            break
        end
    end
end
VMRQD_PRELAUNCH = VMRQDKM;
TF=TF+DELTF;

nmax = ceil((TF+2)/TS)+8;
Z = zeros(nmax,1);
AT=Z; ADRT=Z; AALTT=Z; ADRM=Z; AALTM=Z; AATP=Z; AXNC=Z; APIPP=Z; APIP=Z;
ARTM=Z; AVC=Z; AEN=Z; AACC=Z; AAPT=Z; AZEM=Z;
count=0; nstep=0;

while ~((T > (TF-10.)) && (VC < 0.))
    nstep=nstep+1;
    if nstep > P.maxsteps
        error('mgs:zarchan:stereoNonlinear2d:runaway', ...
            ['the stop condition ~((T > TF-10) & VC < 0) has not fired after ' ...
             '%d steps (T = %.4f s, RTM = %.1f ft, VC = %.1f ft/s).  The ' ...
             'endgame integrates at H = %g s, so this is refused rather than ' ...
             'left to hang.'], nstep, T, RTM, VC, P.HFINE);
    end
    if RTM < P.RFINE, H=P.HFINE; else, H=P.H; end
    XTOLD=XT; YTOLD=YT; XTDOLD=XTD; YTDOLD=YTD;
    XMOLD=XM; YMOLD=YM; XMDOLD=XMD; YMDOLD=YMD; DELVOLD=DELV;
    STEP=1; FLAG=0;
    while STEP <= 1
        if FLAG==1
            STEP=2;
            XT=XT+H*XTD; YT=YT+H*YTD; XTD=XTD+H*XTDD; YTD=YTD+H*YTDD;
            XM=XM+H*XMD; YM=YM+H*YMD; XMD=XMD+H*XMDD; YMD=YMD+H*YMDD;
            DELV=DELV+H*DELVD; T=T+H;
        end
        if ITGT==1
            if T < 180., WGT=-212.*T+44000.; TRST=54100.;
            else,        WGT=3300.;          TRST=0.;
            end
        else
            if T < 120.,     WGT=-2622*T+440660.; TRST=725850.;
            elseif T < 240., WGT=-642.*T+168120.; TRST=182250.;
            else,            WGT=5500.;          TRST=0.;
            end
        end
        ATP=32.2*TRST/WGT;
        TEMPBOTT=(XT^2+YT^2)^1.5;
        XTDD=-GM*XT/TEMPBOTT+AXT;  YTDD=-GM*YT/TEMPBOTT+AYT;
        RTM1=XT-XM; RTM2=YT-YM; VTM1=XTD-XMD; VTM2=YTD-YMD;
        RTM=sqrt(RTM1^2+RTM2^2);
        VC=-(RTM1*VTM1+RTM2*VTM2)/RTM;
        TGO=RTM/VC;
        ACCDOTRTM=(XTDD*RTM1+YTDD*RTM2)/RTM;
        ACCPER1=XTDD-ACCDOTRTM*RTM1/RTM;  ACCPER2=YTDD-ACCDOTRTM*RTM2/RTM;
        ACCPERPTOT=sqrt(ACCPER1^2+ACCPER2^2)/32.2;
        if T > TGUID
            TEMPBOTM=(XM^2+YM^2)^1.5;
            XMDDGRAV=-GM*XM/TEMPBOTM;  YMDDGRAV=-GM*YM/TEMPBOTM;
            ZEM1=RTM1+VTM1*TGO+.5*(XTDD-XMDDGRAV)*TGO^2;
            ZEM2=RTM2+VTM2*TGO+.5*(YTDD-YMDDGRAV)*TGO^2;
            ZEMDOTRTM=(ZEM1*RTM1+ZEM2*RTM2)/RTM;
            ZEMPER1=ZEM1-ZEMDOTRTM*RTM1/RTM;  ZEMPER2=ZEM2-ZEMDOTRTM*RTM2/RTM;
            AXMGUID=XNP*ZEMPER1/(TGO^2);  AYMGUID=XNP*ZEMPER2/(TGO^2);
            TGO=RTM/VC; %#ok<NASGU>  the book computes TGO a second time here,
            % after the commands are already formed.  Dead, and kept dead.
            if QGUID==0, XNCLIM=0.; end     % one-way; the book's guidance-off
            if AXMGUID > XNCLIM,      AXMGUID=XNCLIM;
            elseif AXMGUID < -XNCLIM, AXMGUID=-XNCLIM;
            end
            if AYMGUID > XNCLIM,      AYMGUID=XNCLIM;
            elseif AYMGUID < -XNCLIM, AYMGUID=-XNCLIM;
            end
        else
            AXMGUID=0.; AYMGUID=0.;
        end
        if T > TLAUNCH
            TEMPBOTM=(XM^2+YM^2)^1.5;
            XMDD=-GM*XM/TEMPBOTM+AXMGUID;  YMDD=-GM*YM/TEMPBOTM+AYMGUID;
        else
            XMDD=0.; YMDD=0.;
        end
        ACCNEW=sqrt(AXMGUID^2+AYMGUID^2);  DELVD=ACCNEW;
        FLAG=1;
    end
    FLAG=0; %#ok<NASGU>   the outer loop re-initialises it; the book's line
    XT =.5*(XTOLD+XT+H*XTD);      YT =.5*(YTOLD+YT+H*YTD);
    XTD=.5*(XTDOLD+XTD+H*XTDD);   YTD=.5*(YTDOLD+YTD+H*YTDD);
    XM =.5*(XMOLD+XM+H*XMD);      YM =.5*(YMOLD+YM+H*YMD);
    XMD=.5*(XMDOLD+XMD+H*XMDD);   YMD=.5*(YMDOLD+YMD+H*YMDD);
    DELV=.5*(DELVOLD+DELV+H*DELVD);
    S_=S_+H;
    if QBOOST==1
        TGOLAM=TFTOT-T;
        [VRX,VRY,~,it,rs]=lambert3d(XT,YT,ZT,TGOLAM,XF,YF,ZF,SWITCH1,GM);
        maxResid=max(maxResid,rs); maxIters=max(maxIters,it); nLam=nLam+1;
        VTX=VRX; VTY=VRY;  DELVXT=VTX-XTD; DELVYT=VTY-YTD;
        DELVELT=sqrt(DELVXT^2+DELVYT^2);
        if (T < TPZ && DELVELT > 500.)
            AXT=ATP*DELVXT/DELVELT;  AYT=ATP*DELVYT/DELVELT;
        elseif DELVELT < 500.
            TRST=0.; QBOOST=0; AXT=0.; AYT=0.; %#ok<NASGU>
            XTD=VTX; XTDOLD=XTD; YTD=VTY; YTDOLD=YTD; TBOT=T; %#ok<NASGU>
        else
            QBOOST=0; QOOMPH=0; AXT=0.; AYT=0.; TBOT=T; %#ok<NASGU>
        end
    end
    if T < TUPT
        RTMAG=sqrt(XT^2+YT^2);  AXT=ATP*XT/RTMAG;  AYT=ATP*YT/RTMAG;
    end
    if T >= TLAUNCH
        TGOLAMM=TF-T; %#ok<NASGU>
        if QPERFECT==1
            XTF=XTFACT; YTF=YTFACT;
        elseif (QPERFECT==0 && QTAYLOR==1)
            TGOM=TF-T;
            XTF=XT+XTD*TGOM+.5*XTDD*TGOM*TGOM;
            YTF=YT+YTD*TGOM+.5*YTDD*TGOM*TGOM;
        end
        ZTF=0; QLAUNCH=1; %#ok<NASGU>
        PIPERR=sqrt((XTF-XTFACT)^2+(YTF-YTFACT)^2)/FTPKM; %#ok<NASGU>
    end
    TGOLAMM=TF-T; %#ok<NASGU>
    if (T >= TLAUNCH && QFIRST==1)
        QFIRST=0; TGOPZ=TF-TLAUNCH;
        [VRX,VRY,~,it,rs]=lambert3d(XM,YM,ZM,TGOPZ,XTF,YTF,ZTF,SWITCHM,GM);
        maxResid=max(maxResid,rs); maxIters=max(maxIters,it); nLam=nLam+1;
        VMXRQD=VRX; VMYRQD=VRY;
        VMRQDKM=sqrt(VMXRQD^2+VMYRQD^2)/FTPKM;
        XMD=VMXRQD; XMDOLD=XMD; YMD=VMYRQD; YMDOLD=YMD; %#ok<NASGU>
    end
    if S_ >= (TS-.0001)
        S_=0.;
        ALTTKM=(sqrt(XT^2+YT^2)-A)/FTPKM;
        DISTRTKM=distance3dkm(XT,YT,ZT,XFIRST,YFIRST,ZFIRST,A,FTPKM);
        ALTMKM=(sqrt(XM^2+YM^2)-A)/FTPKM;
        DISTRMKM=distance3dkm(XM,YM,ZM,XFIRST,YFIRST,ZFIRST,A,FTPKM);
        XLAM=atan2(RTM2,RTM1);
        ATPLOS=-XTDD*sin(XLAM)+YTDD*cos(XLAM);   ATPLOSG=ATPLOS/32.2;
        ZEMPLOS=-ZEM1*sin(XLAM)+ZEM2*cos(XLAM);  ZEMPLOSG=ZEMPLOS/FTPKM;
        XNCPLOSG=(-AXMGUID*sin(XLAM)+AYMGUID*cos(XLAM))/32.2;
        if T > TLAUNCH
            PIPKM=sqrt((XTFACT-XTF)^2+(YTFACT-YTF)^2)/FTPKM;
            PIPPLOS=-(XTFACT-XTF)*sin(XLAM)+(YTFACT-YTF)*cos(XLAM);
            PIPPLOSKM=PIPPLOS/FTPKM;
        else
            PIPKM=0.; PIPPLOSKM=0.;
        end
        count=count+1;
        AT(count)=T; ADRT(count)=DISTRTKM; AALTT(count)=ALTTKM;
        ADRM(count)=DISTRMKM; AALTM(count)=ALTMKM;
        AATP(count)=ATPLOSG; AXNC(count)=XNCPLOSG;
        APIPP(count)=PIPPLOSKM; APIP(count)=PIPKM;
        AZEM(count)=ZEMPLOSG;   % the book's name; it is KILOMETRES, not g's
        % Not the book's, these four: the structural invariants need them.
        ARTM(count)=RTM; AVC(count)=VC; AAPT(count)=ACCPERPTOT;
        AEN(count)=.5*(XMD^2+YMD^2)-GM/sqrt(XM^2+YM^2);
        AACC(count)=max(abs(AXMGUID),abs(AYMGUID))/32.2;
    end
end

if count < 2
    % Refused, not warned about.  A one-sample record makes min()/max() below
    % return EMPTY, which would then flow into the benchmark rows as an empty
    % "ours" and render as a blank rather than as a failure.
    error('mgs:zarchan:stereoNonlinear2d:empty', ...
        ['the engagement recorded %d sample(s): it ended at t = %.4f s with ' ...
         'VC = %.1f ft/s.  The stop condition fired before the run began.'], ...
        count, T, VC);
end

S = struct();
S.t=AT(1:count); S.distrt_km=ADRT(1:count); S.alttkm=AALTT(1:count);
S.distrm_km=ADRM(1:count); S.altmkm=AALTM(1:count);
S.atplos_g=AATP(1:count); S.xncplos_g=AXNC(1:count);
S.pipplos_km=APIPP(1:count); S.pip_km=APIP(1:count);
S.zemplos_km=AZEM(1:count);
S.rtm_ft=ARTM(1:count); S.vc=AVC(1:count); S.energy=AEN(1:count);
S.acmd_g=AACC(1:count); S.accperp_g=AAPT(1:count);
S.delv_ftps = DELV;
S.delv_mps  = DELV/FTPM;         % the listing's own last line, DELV/3.28
S.vmrqd_kms = VMRQDKM;
S.vmrqd_prelaunch_kms = VMRQD_PRELAUNCH;
S.t_end = T;  S.rtm_end_ft = RTM;  S.vc_end = VC;  S.tf_used = TF;
S.nsteps = nstep;  S.nlambert = nLam;
S.lambert_maxresid = maxResid;  S.lambert_maxiters = maxIters;
S.max_atplos_g   = max(abs(S.atplos_g));
S.max_xncplos_g  = max(abs(S.xncplos_g));
S.atplos_signed  = [min(S.atplos_g)  max(S.atplos_g)];
S.xncplos_signed = [min(S.xncplos_g) max(S.xncplos_g)];
S.max_acmd_g = max(S.acmd_g);
S.tbot = TBOT;
end


% =============================================================================
function [xtf,ytf,maxresid,maxiters] = predict34(tp,xtp,ytp,xtdp,ytdp,tf, ...
                                                 tftot,tupt,xf,yf,itgt,a,gm,h) %#ok<INUSL>
%PREDICT34  Listing 34.3's predict34.m, zarchan.txt:13441-13465.  Integrates
%   the target forward to tf and returns its exact position there.  The book's
%   "qboost=1" in the boost-exhausted branch (where the main program writes
%   QBOOST=0) is KEPT -- see the file header.  (a is carried because the
%   book's routine declares it; the routine never reads it.)
if itgt==1, tpz=180; else, tpz=240; end
t=tp; switch1=0; xt=xtp; yt=ytp; zt=0.; xtd=xtdp; ytd=ytdp; zf=0.;
qboost=1; axt=0.; ayt=0.;
xtdd=0.; ytdd=0.; maxresid=0; maxiters=0;
while t <= (tf-.00001)
    xtold=xt; ytold=yt; xtdold=xtd; ytdold=ytd;
    step=1; flag=0;
    while step <= 1
        if flag==1
            xt=xt+h*xtd; yt=yt+h*ytd; xtd=xtd+h*xtdd; ytd=ytd+h*ytdd;
            t=t+h; step=2;
        end
        tembot=(xt^2+yt^2)^1.5;
        xtdd=-gm*xt/tembot+axt;  ytdd=-gm*yt/tembot+ayt;
        if itgt==1
            if t < 180., wgt=-212.*t+44000.; trst=54100.;
            else,        wgt=3300.;          trst=0.;
            end
        else
            if t < 120,      wgt=-2622*t+440660.; trst=725850.;
            elseif t < 240., wgt=-642.*t+168120.; trst=182250.;
            else,            wgt=5500.;          trst=0.;
            end
        end
        atp=32.2*trst/wgt;  flag=1;
    end
    flag=0; %#ok<NASGU>
    xt =(xtold+xt)/2+.5*h*xtd;      yt =(ytold+yt)/2+.5*h*ytd;
    xtd=(xtdold+xtd)/2+.5*h*xtdd;   ytd=(ytdold+ytd)/2+.5*h*ytdd;
    if qboost==1
        tgolam=tftot-t;
        [vrx,vry,~,it,rs]=lambert3d(xt,yt,zt,tgolam,xf,yf,zf,switch1,gm);
        maxresid=max(maxresid,rs); maxiters=max(maxiters,it);
        vtx=vrx; vty=vry;  delvxt=vtx-xtd; delvyt=vty-ytd;
        delvelt=sqrt(delvxt^2+delvyt^2);
        if (t < tpz && delvelt > 500.)
            axt=atp*delvxt/delvelt;  ayt=atp*delvyt/delvelt;
        elseif delvelt < 500.
            trst=0.; qboost=1; axt=0.; ayt=0.; %#ok<NASGU>
            xtd=vtx; xtdold=xtd; ytd=vty; ytdold=ytd; %#ok<NASGU>
        else
            qboost=0; axt=0.; ayt=0.;
        end
        if t < tupt        % NESTED inside "if qboost==1" here; NOT in the
            rtmag=sqrt(xt^2+yt^2);          % main program.  The book's, kept.
            axt=atp*xt/rtmag;  ayt=atp*yt/rtmag;
        end
    end
end
xtf=xt; ytf=yt;
end


% =============================================================================
function [VRX, VRY, VRZ, ITERS, RESID] = lambert3d(XT, YT, ZT, TF, ...
                                                   XF, YF, ZF, SWITCH1, GM)
%LAMBERT3D  Zarchan Listing 28.3, zarchan.txt:10428.  Reused from
%   mgs.zarchan.impulsiveIcbm, whose header carries the nine OCR resolutions
%   and the evidence for each; the only change is dropping that port's FLIP
%   delete-experiment flag, which is exercised there.  RESID is the loop's own
%   relative convergence residual, and SELFCHECK 5 scores it -- the book's
%   version cannot tell a converged answer from its ITERS > 100 break.
PI_ = 3.1415926535898;  HALFPI = PI_/2.;
RF0X = XF-XT;  RF0Y = YF-YT;  RF0Z = ZF-ZT;
R0DOTRF  = XT*XF + YT*YF + ZT*ZF;
R0DOTRF0 = XT*RF0X + YT*RF0Y + ZT*RF0Z;
R0MAG  = sqrt(XT^2+YT^2+ZT^2);
RFMAG  = sqrt(XF^2+YF^2+ZF^2);
RF0MAG = sqrt(RF0X^2+RF0Y^2+RF0Z^2);
RATIO = R0MAG/RFMAG;  GMDIVR0 = GM/R0MAG;
COS_T = R0DOTRF/(R0MAG*RFMAG);
VNUMER = GMDIVR0*(1.-COS_T);  T_MIN = 0.;
if SWITCH1 == 0
    G_MIN = HALFPI - acos(R0DOTRF0/(R0MAG*RF0MAG));  G_MAX = HALFPI;
    THETA = acos(COS_T);
else
    G_MIN = -HALFPI;  G_MAX = -HALFPI + acos(R0DOTRF0/(R0MAG*RF0MAG));
    THETA = 2.*PI_ - acos(COS_T);
end
SIN_T = sin(THETA);  COT_HALFT = 1./tan(THETA/2.);
GAMMA = (G_MAX+G_MIN)/2.;  GOLD = G_MIN;  TOLD = 0.;  T = 0.;  ITERS = 1;
RV0MAG = NaN;
while abs(TF-T) > (0.00000001*TF)
    SIN_G = sin(GAMMA);  COS_G = cos(GAMMA);  TAN_G = SIN_G/COS_G;
    COS_TPLUSG = cos(THETA+GAMMA-2.*PI_);
    TERM1 = (RATIO*COS_G - COS_TPLUSG)*COS_G;
    RV0MAG = sqrt(VNUMER/TERM1);
    LAMBDA = RV0MAG*RV0MAG/GMDIVR0;
    if LAMBDA < 1.9999999                            % elliptic
        TERM0 = sqrt(2./LAMBDA-1.);
        TERM1 = (TAN_G*(1.-COS_T)+(1.-LAMBDA)*SIN_T)/((2.-LAMBDA)*RATIO);
        TERM2 = (COS_G+COS_G)/(LAMBDA*TERM0*TERM0*TERM0);
        TERM3 = atan2(TERM0, (COS_G*COT_HALFT-SIN_G));
        T = (R0MAG/(RV0MAG*COS_G))*(TERM1+TERM2*TERM3);
    elseif LAMBDA > 2.0000001                        % hyperbolic
        TERM0 = sqrt(1.-2./LAMBDA);
        TERM1 = (TAN_G*(1.-COS_T)+(1.-LAMBDA)*SIN_T)/((2.-LAMBDA)*RATIO);
        TERM2 = COS_G/(LAMBDA*TERM0*TERM0*TERM0);
        TERM3 = SIN_G - COS_G*COT_HALFT;
        TERM3 = log((TERM3-TERM0)/(TERM3+TERM0));
        T = (R0MAG/(RV0MAG*COS_G))*(TERM1-TERM2*TERM3);
    else                                             % parabolic
        TERM0 = COS_G*COT_HALFT;  TERM1 = TERM0 - SIN_G;
        TERM0 = (3*TERM0*TERM1+1.)/(TERM1*TERM1*TERM1);
        T = TERM0*(2.*R0MAG)/(3.*RV0MAG);
    end
    if T > TF && GAMMA < G_MAX,  G_MAX = GAMMA;               end
    if T < 0. && GAMMA < G_MAX,  G_MAX = GAMMA;               end
    if T < TF && GAMMA > G_MIN,  G_MIN = GAMMA;  T_MIN = T;   end
    if T < 0.
        NEXT = (G_MIN+G_MAX)/2.;  GOLD = G_MIN;  TOLD = T_MIN;
    else
        NEXT = GAMMA + (TF-T)*(GAMMA-GOLD)/(T-TOLD);   % secant
        if NEXT >= G_MAX
            NEXT = (GAMMA+G_MAX)/2.;                   % ...clamped to bisection
        elseif NEXT <= G_MIN
            NEXT = (GAMMA+G_MIN)/2.;
        end
        GOLD = GAMMA;  TOLD = T;
    end
    GAMMA = NEXT;  ITERS = ITERS + 1;
    if ITERS > 100
        break
    end
end
RESID = abs(TF-T)/abs(TF);
if SWITCH1 == 0, ANGLE = HALFPI - GAMMA; else, ANGLE = GAMMA - HALFPI; end
SINA = sin(ANGLE);  COSA = cos(ANGLE);
V1X = XT;  V1Y = YT;  V1Z = ZT;   V2X = XF;  V2Y = YF;  V2Z = ZF;
MAG1 = sqrt(V1X*V1X+V1Y*V1Y+V1Z*V1Z);
DOTMAG = V1X*V2X + V1Y*V2Y + V1Z*V2Z;
CROSSX = V1Y*V2Z - V1Z*V2Y;
CROSSY = V1Z*V2X - V1X*V2Z;
CROSSZ = V1X*V2Y - V1Y*V2X;
CROSSMAG = sqrt(CROSSX*CROSSX+CROSSY*CROSSY+CROSSZ*CROSSZ);
C2 = MAG1*SINA/CROSSMAG;
C1 = COSA/MAG1 - DOTMAG*C2/(MAG1*MAG1);
VRX = RV0MAG*(C2*V2X + C1*V1X);
VRY = RV0MAG*(C2*V2Y + C1*V1Y);
VRZ = RV0MAG*(C2*V2Z + C1*V1Z);
end


% =============================================================================
function DISTKM = distance3dkm(XT, YT, ZT, XF, YF, ZF, A, FTPKM)
%DISTANCE3DKM  Zarchan Listing 28.3.  Great-circle arc at the Earth radius A.
%   "if CBETA,=1." -> "<=": the else branch exists only to catch a rounding
%   overshoot that would otherwise turn acos() complex.
R  = sqrt(XT^2+YT^2+ZT^2);
RF = sqrt(XF^2+YF^2+ZF^2);
CBETA = (XT*XF+YT*YF+ZT*ZF)/(R*RF);
if CBETA <= 1.
    DISTKM = A*acos(CBETA)/FTPKM;
else
    DISTKM = (XF-XT)/FTPKM;
end
end


% =============================================================================
function K = structuralChecks(S, TGUID, TLAUNCH, G)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never evidence that
%   this port reproduces Zarchan -- the weaker claim that it agrees with
%   ITSELF, which is what catches a mis-resolved comparison.
K = struct();  names = {};  oks = [];  detail = {};

% 1  Guidance is silent before TGUID.  This is the delete experiment for the
%    ":13403 T > TGUID" resolution: invert it and this row is the only thing
%    in the file that changes character, because the divert integral would
%    still come out a plausible few hundred m/s.
%    The window stops HALF A SECOND SHORT of TGUID on purpose.  T is
%    accumulated as T+H nine thousand times, so the step that carries the
%    t = 110 sample straddles the switch: whether that one sample is recorded
%    before or after the first guidance command is a floating-point accident,
%    not a fact about the port.  Excluding it costs one sample out of twenty
%    and removes a standing chance of a false red.
pre = S.t <= TGUID - 0.5;
K.pre_guid_max = max([0; abs(S.xncplos_g(pre))]);
names{end+1}  = 'no KKV command before TGUID (T > TGUID)';
oks(end+1)    = K.pre_guid_max == 0;
detail{end+1} = sprintf('max |a_KKV| = %.3e g over %d samples at t <= %g s', ...
    K.pre_guid_max, sum(pre), TGUID-0.5);

% 2  In TWO dimensions the perpendicular-to-LOS direction is unique, so the
%    listing's two spellings of it must agree exactly:
%    |ATPLOSG| = |-xtdd sin(lam) + ytdd cos(lam)| == ACCPERPTOT.
%    A wrong XLAM, or a swapped sin/cos, breaks this and nothing else cheap.
den = max(max(S.accperp_g), realmin);
K.perp_resid = max(abs(abs(S.atplos_g) - S.accperp_g))/den;
names{end+1}  = '|ATPLOSG| == ACCPERPTOT (2-D LOS identity)';
oks(end+1)    = K.perp_resid <= 1e-10;
detail{end+1} = sprintf('max relative gap %.3e over %d samples', ...
    K.perp_resid, numel(S.t));

% 3  Between launch and TGUID the interceptor is on a pure two-body arc, so
%    v^2/2 - GM/r is constant.  Catches an inverted ":13411 T > TLAUNCH" (the
%    interceptor would coast in a straight line and the energy would walk), a
%    sign error on gravity, and a corrector wired to the wrong derivative.
%    1e-6 is far above RK2's drift over 20 s at H = 0.01 s (order 1e-10 by
%    the local-truncation estimate) and far below any structural break, which
%    is O(1).  BOTH ends are inset by half a second for the same reason as
%    check 1: the t = 90 sample straddles the launch impulse and the t = 110
%    sample straddles the first guidance command, so including either would
%    make this row a test of floating-point luck.
w = S.t >= TLAUNCH + 0.5 & S.t <= TGUID - 0.5;
if sum(w) >= 3
    e = S.energy(w);
    K.energy_drift = (max(e)-min(e))/max(abs(mean(e)), realmin);
    ok3 = K.energy_drift <= 1e-6;
else
    K.energy_drift = NaN;  ok3 = false;
end
names{end+1}  = 'two-body energy constant on the unguided coast';
oks(end+1)    = ok3;
detail{end+1} = sprintf('relative spread %.3e over %d samples in [%g, %g] s', ...
    K.energy_drift, sum(w), TLAUNCH+0.5, TGUID-0.5);

% 4  The engagement must actually close.  The listing drops to H = 1e-5 s
%    inside RTM < 1000 ft; if the run ends OUTSIDE that radius then the fine
%    step never engaged, closest approach was never resolved, and the divert
%    integral is an integral over a fly-by.
K.rtm_end = S.rtm_end_ft;
names{end+1}  = 'closes inside the fine-step radius (RTM_end <= 1000 ft)';
oks(end+1)    = K.rtm_end <= 1000.0;
detail{end+1} = sprintf('final RTM = %.1f ft at t = %.4f s (VC = %+.1f ft/s)', ...
    S.rtm_end_ft, S.t_end, S.vc_end);

% 5  Every Lambert solve converged.  A silently unconverged solve is a wrong
%    launch velocity that still flies a perfectly smooth trajectory.
K.lam_resid = S.lambert_maxresid;
names{end+1}  = 'every Lambert solve converged';
oks(end+1)    = K.lam_resid <= 1e-8 && S.lambert_maxiters <= 100;
detail{end+1} = sprintf('worst residual %.2e, worst %d iters over %d solves', ...
    S.lambert_maxresid, S.lambert_maxiters, S.nlambert);

K.names = names;  K.oks = logical(oks);  K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    j = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{j}, detail{j});
end
K.g = G;
end


% =============================================================================
function B = benchmarks(S1, S2, opts, G, FTPM) %#ok<INUSD>
%BENCHMARKS  Computed vs published.  Every book value carries its zarchan.txt
%   line in .source.  Bands come from the PROVENANCE of the book value:
%     'sim'      3%   a 3-figure number printed out of THIS listing; the only
%                     legitimate divergences are our OCR resolutions and
%                     floating point, so anything past ~1% deserves a look
%     'text~'   10%   "approximately 3.6 g" -- prose about Fig. 34.9
%     'text~1'  15%   "approximately 0.7 g" -- one decimal AND "approximately"
%     'text'     3%   "3.4 km/s", a two-figure description
%     'exact'  0.5%   listing constant against prose ("10-g limit")
%     'shape'         an ordering; no tolerance is possible
B = emptyBench();
bookCase = abs(opts.TF-170)<1e-9 && abs(opts.TLAUNCH-90)<1e-9 && ...
           abs(opts.TGUID-110)<1e-9 && abs(opts.XLONGMDEGICKM-400)<1e-9 && ...
           opts.ITGT==1 && opts.QGUID==1;
why = sprintf(['off the chapter''s own case -- the published numbers are for ' ...
    'tL=90 s, tGUID=110 s, tF=170 s, 400-km stand-off, ITGT=1; this run used ' ...
    'tL=%g, tGUID=%g, tF=%g, %g km, ITGT=%g'], opts.TLAUNCH, opts.TGUID, ...
    opts.TF, opts.XLONGMDEGICKM, opts.ITGT);

% ---- 1.  the headline: KKV lateral divert ---------------------------------
if bookCase
    B = addVal(B, 'KKV lateral divert, perfect PIP', 208, S1.delv_mps, ...
        'sim', 0.03, 'p.918 text (zarchan.txt:13471); Fig. 34.9 caption :13504', ...
        'm/s', sprintf(['the listing''s own last line, DELV/3.28.  %.1f ft/s ' ...
        'accumulated from t = %g s to %.3f s'], S1.delv_ftps, opts.TGUID, S1.t_end));
else
    B = addSkip(B, 'KKV lateral divert, perfect PIP', '208', why, ...
        'p.918 text (zarchan.txt:13471)', 'm/s');
end

% ---- 2.  peak target acceleration perpendicular to the LOS ----------------
if bookCase
    B = addVal(B, 'peak target accel perpendicular to LOS', 3.6, ...
        S1.max_atplos_g, 'text~', 0.10, 'p.918 text (zarchan.txt:13471)', 'g', ...
        sprintf(['max over the TS = %g s record, which is what Fig. 34.9 plots; ' ...
        'signed range [%+.2f, %+.2f] g'], opts.TS, S1.atplos_signed(1), ...
        S1.atplos_signed(2)));
else
    B = addSkip(B, 'peak target accel perpendicular to LOS', '3.6', why, ...
        'p.918 text (zarchan.txt:13471)', 'g');
end

% ---- 3.  peak KKV commanded acceleration ---------------------------------
if bookCase
    B = addVal(B, 'peak KKV accel perpendicular to LOS', 0.7, ...
        S1.max_xncplos_g, 'text~1', 0.15, 'p.918 text (zarchan.txt:13471)', 'g', ...
        sprintf(['signed range [%+.3f, %+.3f] g.  The limiter is %.1f g and the ' ...
        'largest component command was %.3f g, so the clamp is inactive and ' ...
        'this row measures the guidance law, not the limiter'], ...
        S1.xncplos_signed(1), S1.xncplos_signed(2), opts.XNCLIM/G, S1.max_acmd_g));
else
    B = addSkip(B, 'peak KKV accel perpendicular to LOS', '0.7', why, ...
        'p.918 text (zarchan.txt:13471)', 'g');
end

% ---- 4.  required impulsive launch speed ---------------------------------
if bookCase
    B = addVal(B, 'required interceptor speed at launch', 3.4, S1.vmrqd_kms, ...
        'text', 0.03, 'p.917 text (zarchan.txt:13467)', 'km/s', ...
        sprintf(['Lambert from the 15-km launch point to the PIP in %g s.  The ' ...
        'pre-loop QFIX solve gives %.4f km/s from the same (unmoved) position'], ...
        opts.TF-opts.TLAUNCH, S1.vmrqd_prelaunch_kms));
else
    B = addSkip(B, 'required interceptor speed at launch', '3.4', why, ...
        'p.917 text (zarchan.txt:13467)', 'km/s');
end

% ---- 5.  the acceleration limit the prose claims for this program --------
B = addVal(B, 'KKV acceleration limit', 10.0, opts.XNCLIM/G, 'exact', 0.005, ...
    'p.920 text (zarchan.txt:13521-13523, "10-g limit on KKV")', 'g', ...
    ['the prose against the listing''s own XNCLIM = 322 ft/s^2.  NOTE the ' ...
     'limiter is applied per COMPONENT, so the command MAGNITUDE can reach ' ...
     '14.1 g; that is the book''s, not a transcription slip']);

% ---- 6.  the ordering the chapter draws out of Fig. 34.9 ------------------
if bookCase && S1.max_xncplos_g > 0
    ratio = S1.max_atplos_g/S1.max_xncplos_g;
    B = addShape(B, 'KKV needs ~5x less acceleration than the target', ...
        'target/KKV peak ratio ~ 5 (3.6 g vs 0.7 g)', ...
        sprintf('%.2f (%.2f g vs %.2f g)', ratio, S1.max_atplos_g, ...
                S1.max_xncplos_g), ratio >= 4.0 && ratio <= 6.5, ...
        'p.918 text (zarchan.txt:13471)', ...
        ['scored as an ordering because "approximately five times" is what the ' ...
         'book says; the window is the ratio the two printed values themselves ' ...
         'support (3.55/0.75 = 4.7 up to 3.65/0.65 = 5.6), opened out']);
else
    B = addSkip(B, 'KKV needs ~5x less acceleration than the target', '~5', ...
        why, 'p.918 text (zarchan.txt:13471)', '-');
end

% ---- 7.  the parabolic-maneuver divert formula, and its own verdict -------
if bookCase
    dvmvr = 0.25*36*(170-110);     % :13512, 0.25*nTMAX*tF, nTMAX = 36 m/s^2
    okArith = abs(dvmvr-540) <= 0.5;
    okOver  = dvmvr > 2*S1.delv_mps;
    B = addBound(B, 'theoretical divert 540 m/s is >2x the simulated divert', ...
        '0.25*36*(170-110) = 540 m/s, "more than twice as high"', ...
        sprintf('formula %.0f m/s, ratio %.2fx our %.1f m/s', dvmvr, ...
                dvmvr/max(S1.delv_mps,realmin), S1.delv_mps), ...
        okArith && okOver, 'p.919 text (zarchan.txt:13511-13513)', 'm/s', ...
        ['nTMAX = 36 is the 3.6 g peak expressed in m/s^2, and tF = 170-110 is ' ...
         'the GUIDED time, not the flight time.  The book prints it as ' ...
         '"0.25  36  (170 110)" -- the multiply and the minus are both OCR ' ...
         'casualties, and the arithmetic is what recovers them']);
else
    B = addSkip(B, 'theoretical divert 540 m/s is >2x the simulated divert', ...
        '540', why, 'p.919 text (zarchan.txt:13511-13513)', 'm/s');
end

% ---- 8.  the BOOK against itself: DVPIP = 1.5*PIP/(tF - tGUID) -----------
%      No simulation in this row.  It scores whether Table 34.1 and the 89-km
%      worked example were transcribed correctly -- this wave's known failure
%      mode -- and it is the arithmetic that proves the table's "tF" column is
%      the intercept time while the formula's denominator is the GUIDED time.
tab = [400 170 570 20400 510
       350 160 410 12500 375
       300 150 290  7200 270
       250 140 210  3900 195
       200 130 140  1800 135
       150 120 110   700 105
       100 120 110   680 102];
dvf   = 1.5*tab(:,4)./(tab(:,2)-110);
rel   = abs(dvf-tab(:,5))./tab(:,5);
rel89 = abs(1.5*89000/(223-180) - 3100)/3100;
worst = max([rel; rel89]);
B = addBound(B, 'book''s DVPIP formula reproduces all 8 published diverts', ...
    'DVFormula = 1.5*PIP/(tF-110) across Table 34.1, and 3.1 km/s for the 89-km case', ...
    sprintf('worst relative gap %.2f%% (89-km case %.2f%%)', 100*worst, 100*rel89), ...
    worst <= 0.02, 'Table 34.1 (zarchan.txt:13567-13575); p.921 (:13524-13530)', '-', ...
    ['a TRANSCRIPTION check, not an integrator check.  The 89-km line prints ' ...
     '"(2232 180 = 43)", i.e. 223-180 = 43 with the minus lost into a digit -- ' ...
     'the damage class the decoder deliberately does not touch']);

% ---- 9.  Table 34.1 row 1, the Taylor-PIP case ---------------------------
j = [];
if opts.taylorcase && bookCase
    j = find(abs(S2.t-opts.TGUID) < 0.5001, 1);
end
if ~isempty(j)
    pipm = abs(S2.pipplos_km(j))*1000;
    B = addVal(B, 'Taylor PIP error perpendicular to LOS at tGUID', 20400, ...
        pipm, 'sim', 0.05, 'Table 34.1 row 1 (zarchan.txt:13569)', 'm', ...
        sprintf('sampled at t = %.2f s; total PIP error there %.3f km', ...
                S2.t(j), S2.pip_km(j)));
    B = addVal(B, 'KKV divert, Taylor PIP, 400 km', 570, S2.delv_mps, ...
        'sim', 0.05, 'Table 34.1 row 1 (zarchan.txt:13569)', 'm/s', ...
        ['same geometry as the rows above but QPERFECT = 0.  Band 5%, not 3%: ' ...
         'the book reached this tF through the QFIX = 0 search and we fix it ' ...
         'at the table''s published 170 s']);
else
    r = 'not run: ''taylorcase'' is false (it is a second full engagement)';
    if ~bookCase
        r = why;
    elseif opts.taylorcase
        r = sprintf(['the record has no sample within 0.5 s of tGUID = %g s ' ...
            '(TS = %g s), and the book''s PIP column is quoted AT the moment ' ...
            'guidance starts'], opts.TGUID, opts.TS);
    end
    B = addSkip(B, 'Taylor PIP error perpendicular to LOS at tGUID', '20,400', ...
        r, 'Table 34.1 row 1 (zarchan.txt:13569)', 'm');
    B = addSkip(B, 'KKV divert, Taylor PIP, 400 km', '570', r, ...
        'Table 34.1 row 1 (zarchan.txt:13569)', 'm/s');
end

% ---- 10.  the one that CANNOT be scored ----------------------------------
B = addSkip(B, 'post-boost 89-km PIP case: divert 2.8 km/s at 223 s', '2800', ...
    ['UNPUBLISHED INPUT.  The book says only that "the interceptor launch ' ...
     'point was moved further downrange" (:13520) for the tF = 230 s, ' ...
     'tGUID = 180 s case; XLONGMDEGICKM for it is never printed and Fig. 34.10 ' ...
     'is an image link.  Solving for the stand-off that returns 2.8 km/s would ' ...
     'be fitting the input to the answer'], ...
    'p.920 text (zarchan.txt:13521-13523)', 'm/s');

% ---- INFO rows.  Reported, never scored. ---------------------------------
B = addInfo(B, 'achieved intercept', '--', ...
    sprintf('t = %.4f s (desired %g s), closest approach %.1f ft, %d steps', ...
            S1.t_end, S1.tf_used, S1.rtm_end_ft, S1.nsteps), ...
    'no miss distance is published for this case', ...
    ['NOT SCORED: the chapter quotes divert, not miss.  The only achieved ' ...
     'intercept time it prints (223 s) belongs to the 89-km case, and that ' ...
     'case is skipped above for want of its launch point']);
B = addInfo(B, 'interceptor stand-off actually flown', '400 km (nominal)', ...
    sprintf('%.1f km at t = 0', S1.distrm_km(1)), ...
    'zarchan.txt:13374, XLONGMDEGICKM/111', ...
    'NOT SCORED: 111 km/deg is the listing''s own rounding, so "400 km" is 401.2 km');
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want.  Schema is the
%   one mgs.zarchan.benchSuite's shared benchRows() consumes.
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
%ADDINFO  Reported, never scored.  skipped = true keeps it out of the pass
%   count in BOTH directions -- an INFO row can neither pass nor fail.
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
L = {};  p = R.p;  S = R.nominal;
L{end+1} = 'Zarchan Listing 34.3 -- TWO-DIMENSIONAL NONLINEAR ENGAGEMENT';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['   tL = %g s   tGUID = %g s   tF = %g s   stand-off %g km' ...
    '   alt %g km   N'' = %g   nc lim %.1f g'], p.TLAUNCH, p.TGUID, p.TF, ...
    p.XLONGMDEGICKM, p.ALTMKMIC, p.XNP, p.XNCLIM_g);
L{end+1} = sprintf(['   H = %g s (%g s inside %g ft)   record every %g s   ' ...
    '%d steps, %d Lambert solves'], p.H, p.HFINE, p.RFINE, p.TS, S.nsteps, ...
    S.nlambert);
L{end+1} = '';
L{end+1} = 'ENGAGEMENT  (the listing''s own record)';
L{end+1} = sprintf('%8s %11s %10s %11s %10s %10s %10s', 't [s]', ...
    'tgt dr[km]', 'tgt h[km]', 'msl dr[km]', 'msl h[km]', 'aT_LOS[g]', 'aM_LOS[g]');
stride = max(1, round(10/p.TS));
rows = unique([1:stride:numel(S.t), numel(S.t)]);
for r = rows
    L{end+1} = sprintf('%8.2f %11.2f %10.2f %11.2f %10.2f %10.3f %10.3f', ...
        S.t(r), S.distrt_km(r), S.alttkm(r), S.distrm_km(r), S.altmkm(r), ...
        S.atplos_g(r), S.xncplos_g(r)); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = sprintf(['ANSWER: divert %.1f m/s (book 208), launch speed %.3f km/s ' ...
    '(book 3.4),'], S.delv_mps, S.vmrqd_kms);
L{end+1} = sprintf(['        peak aT_LOS %.2f g (book 3.6), peak aM_LOS %.2f g ' ...
    '(book 0.7), miss %.1f ft at %.3f s'], S.max_atplos_g, S.max_xncplos_g, ...
    S.rtm_end_ft, S.t_end);
if ~isempty(R.taylor)
    L{end+1} = sprintf('        Taylor PIP case: divert %.1f m/s (book 570)', ...
        R.taylor.delv_mps);
end
L{end+1} = '';
K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-52s %s', vstr(K.oks(i)), K.names{i}, ...
        K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';
if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN.  R.pass = false meaning NOT CHECKED.';
else
    L{end+1} = 'BENCHMARKS  (book value, its zarchan.txt line, and the band''s reason)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        if b.skipped
            mark = '--';
        else
            mark = vstr(b.ok);
        end
        if ~b.skipped && ~isnan(b.pct)
            num = sprintf('%-10s %-12s %+7.2f%%', b.book_str, b.ours_str, b.pct);
        else
            num = sprintf('%-10s %-12s', b.book_str, b.ours_str);
        end
        L{end+1} = sprintf('   %-4s %-52s %s', mark, b.what, num); %#ok<AGROW>
        L{end+1} = sprintf('        %s | %s', b.band, b.source); %#ok<AGROW>
        if b.skipped && ~isempty(b.why)
            L{end+1} = sprintf('        why: %s', b.why); %#ok<AGROW>
        end
    end
    L{end+1} = sprintf('   %d scored, %d skipped/info.', sum(ev), sum(~ev));
end
s = strjoin(L, newline);
end

function s = vstr(ok)
if ok, s = 'ok'; else, s = 'FAIL'; end
end
