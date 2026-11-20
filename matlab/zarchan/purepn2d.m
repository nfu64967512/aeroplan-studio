function out = purepn2d(varargin)
%PUREPN2D  Zarchan Ch. 30 Pure PN -- the IGUID=1 branch of Listing 30.1.
%
%       gamdot = N' * lamdot        (+ BIAS while TBEG <= T < TBEG+DELT)
%       nc     = VM * gamdot
%
%   This is PURE proportional navigation: the command is built from the
%   missile's OWN speed VM (not from the closing velocity Vc), and with
%   ICHOICE=1 it is applied PERPENDICULAR TO THE MISSILE VELOCITY VECTOR.
%   That is the law our STOT vehicle actually flies -- see
%   memory note "we-already-fly-pure-pn".
%
%   SOURCE (local full text, zarchan.txt)
%     line 10930  prose introducing Listing 30.1, the IGUID / ICHOICE
%                 options, and the stationary-target 10 km / -90 deg case
%     line 10932  "n=0 ; IGUID=0 ; ICHOICE=0 ;"  (start of the listing)
%     line 10949  the ICHOICE branch
%     line 10961  Figs. 30.2-30.4 discussion
%     line 10980  Fig. 30.5: velocity loss, verbatim --
%                 "dramatic trajectory shaping causes a severe missile
%                  velocity loss when the missile acceleration command is
%                  perpendicular to the line of sight and no velocity loss
%                  at all when the missile acceleration command is
%                  perpendicular to the velocity vector"
%
%   RELATION TO ../ref/zarchan_listing30_1.m
%   ----------------------------------------
%   ../ref holds the FULL Listing 30.1 (both IGUID branches), transcribed by
%   another workstream.  This file is the IGUID=1 branch only, kept
%   numerically identical so the two can be cross-checked against each other
%   -- demo_reproduce does exactly that (check D).  If they ever disagree,
%   one of the two transcriptions is wrong; do not "fix" either by eye.
%
%   USAGE
%       out = purepn2d()                                  % unbiased pure PN
%       out = purepn2d('BIASFORM','listing')              % biased PN, book
%       out = purepn2d('ICHOICE',0)                       % command perp. LOS
%
%   OPTIONS (defaults = Listing 30.1's data statement except BIASFORM)
%       ICHOICE  0 = a_cmd perp. LOS, 1 = perp. velocity vector       [1]
%       XNP      effective navigation ratio N'                        [3]
%       GAMDEG   initial flight-path angle (= heading error), deg     [30]
%       GAMFDEG  desired terminal flight-path angle, deg              [-90]
%       DELT     bias-on duration, s                                  [30]
%       TBEG     bias turn-on time, s                                 [0]
%       XNCLIMG  acceleration command limit, g                        [10]
%       VMIC     initial missile speed, m/s                           [250]
%       RANGEM   initial downrange to the stationary target, m        [10000]
%       RT2M     target altitude, m  (NON-BOOK, see below)            [0]
%       BIASFORM 'none' | 'listing' | 'text'                          ['none']
%       HSCALE   multiplier on the book's step H (1 = book)           [1]
%       PLOTS    reproduce Figs. 30.2 / 30.3 / 30.4 / 30.5            [false]
%
%   ======================================================================
%   ERRATUM -- THE BIAS SIGN.  SOURCE CODE WINS.
%   ======================================================================
%   Book PROSE:    bias = [  gamF*(N'-1) + N'*lamIC - gamIC ] / DELT
%   Listing 30.1:  BIASDEG=(-GAMFDEG*(XNP-1.)+XNP*XLAM*57.3-GAMDEG)/DELT
%                            ^ leading MINUS
%   They disagree in sign on the first term.  The LISTING is authoritative
%   (independently re-confirmed against the PDF page image).  With the
%   nominal numbers the listing gives BIAS = +5 deg/s and the prose -7 deg/s;
%   +5 is the one that lofts the missile so it has altitude to dive out of,
%   which is the ~3 km apogee drawn in Fig. 30.2.
%   BIASFORM='text' runs the prose variant so the difference can be seen.
%
%   UNITS
%   -----
%   The listing's INTERNAL units are FEET despite the prose at line 10930
%   saying the units "have been converted to the metric system": the initial
%   conditions are written 10000.*3.28 and 250.*3.28, and XNCLIM=32.2*XNCLIMG.
%   Only the 0.1-s log block converts back.  Kept verbatim, because 3.28 (not
%   3.28084) is the book's constant and round-tripping through it is not the
%   identity.  Inputs/outputs of this function are metric; the loop is feet.
%
%   DEVIATIONS FROM THE LISTING (all of them)
%   -----------------------------------------
%   1. Only the IGUID=1 branch is kept.  IGUID=0 (optimal trajectory
%      shaping) lives in ../ref/zarchan_listing30_1.m.
%   2. BIASFORM='none' (BIAS=0) added as the default, because "pure PN" in
%      this project means the unbiased law.  'listing' reproduces the book.
%   3. RT2M: the listing hard-codes the target at launch altitude (RT2IC=0).
%      Our engagement releases ABOVE a sea-level target.  Default 0
%      reproduces the book exactly.  Same option, same meaning as ../ref.
%   4. VMM (missile speed) added to the log block -- the printed listing
%      drops it even though the prose says missile velocity is written to
%      the file and Fig. 30.5 plots it.
%   5. Preallocation + runaway guard (T > 600 s).  Speed / safety only.
%
%   APPLICABILITY -- READ BEFORE USING THE NUMBERS
%   ----------------------------------------------
%   * ZERO lag.  XNC becomes AM in the same integration step.  We have
%     tau ~= 1.50 s, and a 1.75 s alpha lowpass on top.
%   * The only nonlinearity is the SYMMETRIC command limit +-XNCLIM applied
%     to XNC (magnitude of the command, before it is resolved into
%     components).  Our vehicle instead clamps nz = clip(cos(gam) +
%     V*gdot/g, -1.0, 3.0), which is ASYMMETRIC, is in load-factor space,
%     and is followed by a one-sided LOS floor.  The book has no
%     counterpart to the LOS floor anywhere (verified: Ch24, Ch28, Ch30,
%     App A all use an ADDITIVE terminal-angle term, never a clamp).
%     Do not read this file as a model of our saturation behaviour.
%   * Constant-speed point mass: no drag, no thrust, no gravity.  With
%     ICHOICE=1 the speed is constant by construction (a is perpendicular
%     to v), which is a KINEMATIC IDENTITY here, not a physical result.
%     Our airframe has drag and thrust, so "no velocity loss at all"
%     (line 10980) does NOT transfer to us.

%% ---- options ------------------------------------------------------------
p = struct('ICHOICE',1,'XNP',3,'GAMDEG',30,'GAMFDEG',-90, ...
           'DELT',30,'TBEG',0,'XNCLIMG',10,'VMIC',250,'RANGEM',10000, ...
           'RT2M',0,'BIASFORM','none','HSCALE',1,'PLOTS',false,'SAVEDAT','');
for k = 1:2:numel(varargin)
    f = varargin{k};
    if ~isfield(p,f), error('purepn2d: unknown option %s',f); end
    p.(f) = varargin{k+1};
end

%% =======================================================================
%  LISTING 30.1, IGUID=1 branch  (line by line)
%  =======================================================================
n=0; ICHOICE=p.ICHOICE;
GAMDEG=p.GAMDEG; GAMIC=GAMDEG/57.3; DELT=p.DELT; TBEG=p.TBEG; TEND=TBEG+DELT;
XNP=p.XNP; RM1IC=0.; RM2IC=0.; RT1IC=p.RANGEM*3.28; RT2IC=p.RT2M*3.28;
VM=p.VMIC*3.28;
XNCLIMG=p.XNCLIMG; GAMFDEG=p.GAMFDEG; H=.0001*p.HSCALE; XNCLIM=32.2*XNCLIMG;
RM1=RM1IC; RM2=RM2IC; RT1=RT1IC; RT2=RT2IC; VT1=0.; VT2=0.; T=0.; S=0.;
RTM1=RT1-RM1; RTM2=RT2-RM2; RTM=sqrt(RTM1^2+RTM2^2); XLAM=atan2(RTM2,RTM1);
VM1=VM*cos(GAMIC); VM2=VM*sin(GAMIC); VTM1=VT1-VM1; VTM2=VT2-VM2;
VC=-(RTM1*VTM1+RTM2*VTM2)/RTM;

switch lower(p.BIASFORM)
    case 'listing'   % <-- the Listing 30.1 source line.  AUTHORITATIVE.
        BIASDEG=(-GAMFDEG*(XNP-1.) + XNP*XLAM*57.3-GAMDEG)/DELT;
    case 'text'      % <-- the book's prose line (erratum, kept for study)
        BIASDEG=( GAMFDEG*(XNP-1.) + XNP*XLAM*57.3-GAMDEG)/DELT;
    case 'none'      % <-- unbiased pure PN (deviation 2)
        BIASDEG=0;
    otherwise
        error('purepn2d: BIASFORM must be none | listing | text');
end
BIAS=BIASDEG/57.3; X=0.;

NMAX=200000;
ArrayT=zeros(1,NMAX); ArrayRT1K=ArrayT; ArrayRT2K=ArrayT; ArrayRM1K=ArrayT;
ArrayRM2K=ArrayT; ArrayXNCG=ArrayT; ArrayGAMDEG=ArrayT; ArrayXM=ArrayT;
ArrayVMM=ArrayT;
AM1=0; AM2=0; XD=0; XNC=0; GAM=GAMIC;

while VC >= 0
    if RTM < 1000
        H=.00001*p.HSCALE;
    else
        H=.0001*p.HSCALE;
    end
    RM1OLD=RM1; RM2OLD=RM2; VM1OLD=VM1; VM2OLD=VM2;
    XOLD=X; STEP=1; FLAG=0;
    while STEP <= 1
        if FLAG==1
            STEP=2;
            RM1=RM1+H*VM1; RM2=RM2+H*VM2; VM1=VM1+H*AM1; VM2=VM2+H*AM2;
            X=X+H*XD; T=T+H;
        end
        GAM=atan2(VM2,VM1); VM=sqrt(VM1^2+VM2^2);
        RTM1=RT1-RM1; RTM2=RT2-RM2; RTM=sqrt(RTM1^2+RTM2^2);
        VTM1=VT1-VM1; VTM2=VT2-VM2;
        VC=-(RTM1*VTM1+RTM2*VTM2)/RTM;
        XLAM=atan2(RTM2,RTM1);
        XLAMD=(RTM1*VTM2-RTM2*VTM1)/(RTM*RTM);
        % ---- IGUID = 1 : (biased) pure proportional navigation ---------
        if T<TBEG
            GAMD=XNP*XLAMD;         XNC=VM*GAMD;
        elseif T<TEND
            GAMD=XNP*XLAMD+BIAS;    XNC=VM*GAMD;
        else
            GAMD=XNP*XLAMD;         XNC=VM*GAMD;
        end
        if XNC>XNCLIM,  XNC=XNCLIM;  end
        if XNC<-XNCLIM, XNC=-XNCLIM; end
        if ICHOICE==0
            AM1=-XNC*sin(XLAM);     AM2=XNC*cos(XLAM);     % perp. LOS
        else
            AM1=-XNC*sin(GAM);      AM2=XNC*cos(GAM);      % perp. velocity
        end
        XD=XNC*XNC; FLAG=1;
    end
    FLAG=0;
    RM1=.5*(RM1OLD+RM1+H*VM1); RM2=.5*(RM2OLD+RM2+H*VM2);
    VM1=.5*(VM1OLD+VM1+H*AM1); VM2=.5*(VM2OLD+VM2+H*AM2);
    X=.5*(XOLD+X+H*XD);
    S=S+H;
    if S >= .09999
        S=0.; n=n+1;
        RT1K=RT1/3280.; RT2K=RT2/3280.; RM1K=RM1/3280.; RM2K=RM2/3280.;
        XNCG=XNC/32.2; GAMDEG=GAM*57.3; VMM=VM/3.28; XM=X/(3.28*3.28);
        ArrayT(n)=T; ArrayRT1K(n)=RT1K; ArrayRT2K(n)=RT2K;
        ArrayRM1K(n)=RM1K; ArrayRM2K(n)=RM2K; ArrayXNCG(n)=XNCG;
        ArrayGAMDEG(n)=GAMDEG; ArrayXM(n)=XM; ArrayVMM(n)=VMM;
    end
    if T > 600 || n >= NMAX, break; end       % deviation 5
end
RTM=sqrt(RTM1^2+RTM2^2); VM=sqrt(VM1^2+VM2^2);
%  ================== end of Listing 30.1 (IGUID=1) =======================

ArrayT=ArrayT(1:n); ArrayRT1K=ArrayRT1K(1:n); ArrayRT2K=ArrayRT2K(1:n);
ArrayRM1K=ArrayRM1K(1:n); ArrayRM2K=ArrayRM2K(1:n); ArrayXNCG=ArrayXNCG(1:n);
ArrayGAMDEG=ArrayGAMDEG(1:n); ArrayXM=ArrayXM(1:n); ArrayVMM=ArrayVMM(1:n);

if p.PLOTS
    figure, plot(ArrayRM1K,ArrayRM2K,ArrayRT1K,ArrayRT2K),grid
    xlabel('Downrange (Km)'), ylabel('Altitude (Km)'), title('Fig. 30.2 style')
    figure, plot(ArrayT,ArrayXNCG),grid
    xlabel('Time (Sec)'), ylabel('XNC (G)'), title('Fig. 30.3 style')
    figure, plot(ArrayT,ArrayGAMDEG),grid
    xlabel('Time (Sec)'), ylabel('GAM (Deg)'), title('Fig. 30.4 style')
    figure, plot(ArrayT,ArrayVMM),grid
    xlabel('Time (Sec)'), ylabel('VM (m/s)'), title('Fig. 30.5 style')
end
if ~isempty(p.SAVEDAT)
    output=[ArrayT',ArrayRT1K',ArrayRT2K',ArrayRM1K',ArrayRM2K', ...
            ArrayXNCG',ArrayGAMDEG',ArrayVMM',ArrayXM']; %#ok<NASGU>
    save(p.SAVEDAT,'output','-ascii');
end

out           = struct();
out.opts      = p;
out.BIASDEG   = BIASDEG;
out.T         = ArrayT;
out.RM1K      = ArrayRM1K;  out.RM2K   = ArrayRM2K;
out.XNCG      = ArrayXNCG;  out.GAMDEG = ArrayGAMDEG;
out.XM        = ArrayXM;    out.VMM    = ArrayVMM;
out.tf        = T;
out.miss_m    = RTM/3.28;
out.gam_f_deg = GAM*57.3;
out.vm_f_ms   = VM/3.28;
out.x_perf    = X/(3.28*3.28);
out.vm_ic_ms  = ArrayVMM(1);
out.vm_min_ms = min(ArrayVMM);
out.vm_max_ms = max(ArrayVMM);
out.apogee_km = max(ArrayRM2K);
out.xnc_max_g = max(abs(ArrayXNCG));
end
