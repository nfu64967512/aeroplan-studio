function out = zarchan_listing30_1(varargin)
%ZARCHAN_LISTING30_1  Verbatim MATLAB transcription of Zarchan Listing 30.1.
%
%   "TWO-DIMENSIONAL ENGAGEMENT SIMULATION WITH OPTIMAL TRAJECTORY SHAPING
%    AND BIASED PROPORTIONAL NAVIGATION GUIDANCE OPTIONS"
%   Zarchan, Tactical and Strategic Missile Guidance, Ch. 30, pp. 780-783.
%
%   PURPOSE IN THIS REPO
%   --------------------
%   Positive control for the STOT fidelity gate.  We already have a negative
%   control (config N1: pn_los_floor=0 + pn_nz_min=0.05, which flies 0/6 in
%   SITL).  What we lacked was a reference implementation whose answer is
%   known a priori.  Listing 30.1's scenario is isomorphic to ours: stationary
%   target, 10 km downrange, terminal flight-path angle -90 deg, V0 = 250 m/s.
%   It is a ruler of known length.
%
%   USAGE
%       out = zarchan_listing30_1()                        % nominal IGUID=0 ICHOICE=0
%       out = zarchan_listing30_1('IGUID',1,'ICHOICE',1)
%       out = zarchan_listing30_1('IGUID',1,'BIASFORM','text')   % erratum variant
%
%   Name/value options (defaults = the book's nominal case):
%       IGUID    0 = optimal trajectory shaping, 1 = biased PN          [0]
%       ICHOICE  0 = a_cmd perpendicular to LOS, 1 = perp. to velocity  [0]
%       XNP      PN navigation ratio N' (biased-PN branch only)         [3]
%       GAMDEG   initial flight path angle (heading error), deg         [30]
%       GAMFDEG  desired terminal flight path angle, deg                [-90]
%       DELT     bias-on duration, s                                    [30]
%       TBEG     bias turn-on time, s                                   [0]
%       XNCLIMG  acceleration command limit, g                          [10]
%       VMIC     initial missile speed, m/s                             [250]
%       RANGEM   initial downrange to target, m                         [10000]
%       BIASFORM 'listing' (book source line) | 'text' (book prose line)['listing']
%       HSCALE   multiplier on the book's integration step H (1 = book)   [1]
%       PLOTS    true -> reproduce Figs 30.2 / 30.3 / 30.4              [false]
%       SAVEDAT  path for the datfil.txt equivalent, '' = skip          ['']
%
%   =====================================================================
%   ERRATUM -- THE BIAS SIGN   (recorded per task instruction)
%   =====================================================================
%   The book's PROSE gives the bias as
%
%           bias = [ gamF*(N-1) + N*lamIC - gamIC ] / Dt           (prose)
%
%   while the SOURCE LINE of Listing 30.1 reads
%
%           BIASDEG=(-GAMFDEG*(XNP-1.) + XNP*XLAM*57.3-GAMDEG)/DELT
%                    ^ note the leading minus                     (listing)
%
%   They disagree in sign on the first term.  Per instruction the LISTING
%   line is authoritative and is what this file implements.
%
%   Independent check performed here: the LISTING is the form that is
%   self-consistent with the derivation printed immediately above it,
%       gamF - gamIC = N*(gamF - lamIC) + bias*Dt
%   =>  bias*Dt = gamF - gamIC - N*gamF + N*lamIC
%               = -gamF*(N-1) + N*lamIC - gamIC        <-- the LISTING form.
%   With the nominal numbers (gamF=-90, gamIC=+30, lamIC=0, N=3, Dt=30) the
%   listing gives BIAS = +5 deg/s, the prose gives -7 deg/s.  +5 deg/s is also
%   the physically correct one: to arrive at -90 deg the missile must FIRST
%   LOFT (positive gamma-dot) so that it has altitude to dive out of, which is
%   exactly the ~3 km apogee drawn in Fig. 30.2.  The prose line has a dropped
%   minus sign.  Set BIASFORM='text' to run the prose variant.
%
%   TRANSCRIPTION NOTES (OCR repairs; no semantic changes)
%   ------------------------------------------------------
%     "while VC .=0"   -> while VC >= 0
%     "if RTM ,1000"   -> if RTM < 1000
%     "while STEP ,=1" -> while STEP <= 1
%     "if XNC.XNCLIM"  -> if XNC > XNCLIM
%     "if XNC,-XNCLIM" -> if XNC < -XNCLIM
%     "if T,TBEG"      -> if T < TBEG
%     "elseif T,TEND"  -> elseif T < TEND
%     "if S.= .09999"  -> if S >= .09999
%
%   The listing's INTERNAL units are FEET / FT-SEC despite the prose saying
%   "converted to the metric system": the ICs are written 10000.*3.28 and
%   250.*3.28, and XNCLIM=32.2*XNCLIMG.  Only the 0.1 s logging block converts
%   back (/3280 -> km, /3.28 -> m/s, /(3.28*3.28) -> m^2/s^3).  Kept verbatim,
%   because 3.28 (not 3.28084) is the book's constant and round-tripping
%   through it is not the identity.
%
%   The printed log block stores ArrayXM but drops the companion ArrayVMM
%   line, even though the prose says missile velocity is written to the file
%   and Fig. 30.5 plots it.  VMM is logged here; that is the only addition to
%   the logging block.
%
%   Integrator is the book's own two-pass predictor/trapezoidal-corrector
%   scheme (STEP/FLAG), retained exactly.

%% ---- options (the book's nominal case is the default) -------------------
%   NON-BOOK OPTION: RT2M.  The listing hard-codes RT2IC = 0, i.e. the target
%   sits at the launch altitude.  Our engagement releases ABOVE a sea-level
%   target, so the positive control (stot_poscontrol.m) needs the target placed
%   at RT2M = -z0.  Default 0 reproduces the book exactly -- every E3 case and
%   figure is unchanged.  This is the only line of the transcription that is not
%   in Zarchan.
p = struct('IGUID',0,'ICHOICE',0,'XNP',3,'GAMDEG',30,'GAMFDEG',-90, ...
           'DELT',30,'TBEG',0,'XNCLIMG',10,'VMIC',250,'RANGEM',10000, ...
           'BIASFORM','listing','PLOTS',false,'SAVEDAT','','HSCALE',1, ...
           'RT2M',0);
for k = 1:2:numel(varargin)
    f = varargin{k};
    if ~isfield(p,f), error('unknown option %s',f); end
    p.(f) = varargin{k+1};
end

%% =======================================================================
%  LISTING 30.1  (line by line)
%  =======================================================================
n=0 ; IGUID=p.IGUID ; ICHOICE=p.ICHOICE ;

GAMDEG=p.GAMDEG ; GAMIC=GAMDEG/57.3 ; DELT=p.DELT ; TBEG=p.TBEG ; TEND=TBEG+DELT ;
XNP=p.XNP ; RM1IC=0. ; RM2IC=0. ; RT1IC=p.RANGEM*3.28 ; RT2IC=p.RT2M*3.28 ; VM=p.VMIC*3.28 ;
XNCLIMG=p.XNCLIMG ; GAMFDEG=p.GAMFDEG ; H=.0001*p.HSCALE ; XNCLIM=32.2*XNCLIMG ;
RM1=RM1IC ; RM2=RM2IC ; RT1=RT1IC ; RT2=RT2IC ; VT1=0. ; VT2=0. ; T=0. ; S=0. ;
RTM1=RT1-RM1 ; RTM2=RT2-RM2 ; RTM=sqrt(RTM1^2+RTM2^2) ; XLAM=atan2(RTM2,RTM1) ;
VM1=VM*cos(GAMIC) ; VM2=VM*sin(GAMIC) ; VTM1=VT1-VM1 ; VTM2=VT2-VM2 ;
VC=-(RTM1*VTM1+RTM2*VTM2) / RTM ; GAMF=GAMFDEG/57.3 ;
if strcmpi(p.BIASFORM,'listing')
    BIASDEG=(-GAMFDEG*(XNP-1.) + XNP*XLAM*57.3-GAMDEG)/DELT ;   % <-- listing line
else
    BIASDEG=( GAMFDEG*(XNP-1.) + XNP*XLAM*57.3-GAMDEG)/DELT ;   % <-- prose line
end
BIAS=BIASDEG/57.3 ; X=0. ;

% preallocation (not in the book; speed only, arrays trimmed at the end)
NMAX=200000 ;
ArrayT=zeros(1,NMAX) ; ArrayRT1K=ArrayT ; ArrayRT2K=ArrayT ; ArrayRM1K=ArrayT ;
ArrayRM2K=ArrayT ; ArrayXNCG=ArrayT ; ArrayGAMDEG=ArrayT ; ArrayXM=ArrayT ;
ArrayVMM=ArrayT ;
AM1=0 ; AM2=0 ; XD=0 ; XNC=0 ; GAM=GAMIC ;

while VC >= 0
    if RTM < 1000
        H= .00001*p.HSCALE ;
    else
        H= .0001*p.HSCALE ;
    end
    RM1OLD=RM1 ; RM2OLD=RM2 ; VM1OLD=VM1 ; VM2OLD=VM2 ;
    XOLD=X ; STEP=1 ; FLAG=0 ;
    while STEP <= 1
        if FLAG==1
            STEP=2 ;
            RM1=RM1+H*VM1 ; RM2=RM2+H*VM2 ; VM1=VM1+H*AM1 ; VM2=VM2+H*AM2 ;
            X=X+H*XD ; T=T+H ;
        end
        GAM=atan2(VM2,VM1) ; VM=sqrt(VM1^2+VM2^2) ;
        RTM1=RT1-RM1 ; RTM2=RT2-RM2 ; RTM=sqrt(RTM1^2+RTM2^2) ;
        VTM1=VT1-VM1 ; VTM2=VT2-VM2 ;
        VC= -(RTM1*VTM1+RTM2*VTM2) / RTM ;
        XLAM=atan2(RTM2,RTM1) ;
        XLAMD=(RTM1*VTM2-RTM2*VTM1)/(RTM*RTM) ;
        if IGUID==0
            TGO=RTM/VC ;
            XNC=4. *VC*XLAMD+2. *VC*(XLAM-GAMF) / TGO ;
        else
            if T<TBEG
                GAMD=XNP*XLAMD ; XNC=VM*GAMD ;
            elseif T<TEND
                GAMD=XNP*XLAMD+BIAS ; XNC=VM*GAMD ;
            else
                GAMD=XNP*XLAMD ; XNC=VM*GAMD ;
            end
        end
        if XNC>XNCLIM
            XNC=XNCLIM ;
        end
        if XNC<-XNCLIM
            XNC=-XNCLIM ;
        end
        if ICHOICE==0
            AM1=-XNC*sin(XLAM) ;
            AM2=XNC*cos(XLAM) ;
        else
            AM1=-XNC*sin(GAM) ;
            AM2=XNC*cos(GAM) ;
        end
        XD=XNC*XNC ; FLAG=1 ;
    end
    FLAG=0 ;
    RM1= .5* (RM1OLD+RM1+H*VM1) ; RM2= .5* (RM2OLD+RM2+H*VM2) ;
    VM1= .5* (VM1OLD+VM1+H*AM1) ; VM2= .5* (VM2OLD+VM2+H*AM2) ;
    X= .5*(XOLD+X+H*XD) ;
    S=S+H ;
    if S >= .09999
        S=0. ; n=n+1 ;
        RT1K=RT1/3280. ; RT2K=RT2/3280. ; RM1K=RM1/3280. ; RM2K=RM2/3280. ;
        XNCG=XNC/32.2 ; GAMDEG=GAM*57.3 ; VMM=VM/3.28 ; XM=X/(3.28*3.28) ;
        ArrayT(n)=T ; ArrayRT1K(n)=RT1K ; ArrayRT2K(n)=RT2K ;
        ArrayRM1K(n)=RM1K ; ArrayRM2K(n)=RM2K ; ArrayXNCG(n)=XNCG ;
        ArrayGAMDEG(n)=GAMDEG ; ArrayXM(n)=XM ; ArrayVMM(n)=VMM ;
    end
    if T > 600 || n >= NMAX
        break ;     % runaway guard (not in the book)
    end
end

ArrayT=ArrayT(1:n) ; ArrayRT1K=ArrayRT1K(1:n) ; ArrayRT2K=ArrayRT2K(1:n) ;
ArrayRM1K=ArrayRM1K(1:n) ; ArrayRM2K=ArrayRM2K(1:n) ; ArrayXNCG=ArrayXNCG(1:n) ;
ArrayGAMDEG=ArrayGAMDEG(1:n) ; ArrayXM=ArrayXM(1:n) ; ArrayVMM=ArrayVMM(1:n) ;

if p.PLOTS
    figure
    plot(ArrayRT1K,ArrayRT2K,ArrayRM1K,ArrayRM2K),grid
    title('Engagement Geometry'), xlabel('Downrange (Km)'), ylabel('Altitude (Km)')
    figure
    plot(ArrayT,ArrayXNCG),grid
    title('Commanded Acceleration'), xlabel('Time (Sec)'), ylabel('XNC (G)')
    figure
    plot(ArrayT,ArrayGAMDEG),grid
    title('Flight Path Angle'), xlabel('Time (Sec)'), ylabel('GAM (Deg)')
end
if ~isempty(p.SAVEDAT)
    output=[ArrayT',ArrayRT1K',ArrayRT2K',ArrayRM1K',ArrayRM2K', ...
            ArrayXNCG',ArrayGAMDEG',ArrayVMM',ArrayXM'] ; %#ok<NASGU>
    save(p.SAVEDAT,'output','-ascii') ;
end
RTM=sqrt(RTM1^2+RTM2^2) ;
VM=sqrt(VM1^2+VM2^2) ;
%  ======================= end of Listing 30.1 ============================

out             = struct();
out.opts        = p;
out.BIASDEG     = BIASDEG;
out.T           = ArrayT;
out.RM1K        = ArrayRM1K;   out.RM2K   = ArrayRM2K;
out.XNCG        = ArrayXNCG;   out.GAMDEG = ArrayGAMDEG;
out.XM          = ArrayXM;     out.VMM    = ArrayVMM;
out.tf          = T;
out.miss_m      = RTM/3.28;                % terminal range to go, m
out.gam_f_deg   = GAM*57.3;
out.vm_f_ms     = VM/3.28;
out.x_perf      = X/(3.28*3.28);           % performance index, m^2/s^3
out.vm_min_ms   = min(ArrayVMM);
out.vm_max_ms   = max(ArrayVMM);
out.vm_ic_ms    = ArrayVMM(1);
out.apogee_km   = max(ArrayRM2K);
out.xnc_max_g   = max(abs(ArrayXNCG));
end
