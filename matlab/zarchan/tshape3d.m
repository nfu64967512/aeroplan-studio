function out = tshape3d(varargin)
%TSHAPE3D  Zarchan Listing A.6 -- 3-D trajectory shaping with terminal angle.
%
%       nc_i = [6*RTM_i + 6*VTM_i*tgo + nT_i*tgo^2
%               + 2*(VM_i - VM_i(tF))*tgo] / tgo^2
%       a    = nc - (nc . vhat) vhat        <-- projected PERP. TO VELOCITY
%
%   The terminal flight-path angle is imposed as an ADDITIVE term chasing a
%   TERMINAL VELOCITY VECTOR, never as a clamp on a commanded angle:
%       VM1(tF)=VM*cos(gPF)*cos(gYF)
%       VM2(tF)=VM*sin(gPF)
%       VM3(tF)=VM*cos(gPF)*sin(gYF)                       (line 14454)
%
%   SOURCE (local full text, zarchan.txt)
%     line 14414  the 2-D form, and the warning that swapping terminal
%                 RELATIVE velocity for terminal OWN velocity moves the PN
%                 coefficient from 4 to 6
%     line 14416  "LISTING A.6 TRAJECTORY SHAPING GUIDANCE LAW IN
%                  THREE-DIMENSIONAL SIMULATION"
%     lines 14417-14442  the code, transcribed below (incl. LAUNCHLOGIC)
%     line 14445  verbatim: the 3-D extension is written "by inspection",
%                 i.e. the author did NOT redo the 3-D optimisation
%     line 14446  the three-axis guidance law
%     line 14452  verbatim: "we want to make the total acceleration
%                 perpendicular to the missile velocity vector. This will
%                 ensure that the missile velocity will remain constant"
%     line 14454  the terminal velocity components above
%     line 14463  verbatim: "the price paid for the trajectory shaping is a
%                 very large commanded missile acceleration"
%
%   *** DO NOT CONFUSE THE TWO PROJECTIONS ***
%   Ch. 24 nonlinear (line 8352) projects PERPENDICULAR TO THE LOS.
%   App. A (line 14452) projects PERPENDICULAR TO THE VELOCITY VECTOR.
%   The code for the two looks nearly identical.  We fly perpendicular to
%   the velocity vector, so App. A is the relevant one.
%
%   *** THE TERMINAL-SPEED TRAP (this one will bite us) ***
%   VM1F/VM2F/VM3F are computed ONCE, before the loop, from the INITIAL VM
%   (line 14420).  That is only self-consistent because the perpendicular
%   projection makes |VM| exactly constant in this constant-speed point-mass
%   model.  Our vehicle has thrust and drag, so |VM| at impact is NOT |VM|
%   at release.  Copying this file as-is would bias the additive term
%   systematically -- the same failure already recorded in the memory note
%   "predictor-underestimates-dive".  If you port it, recompute VM*F every
%   step from a PREDICTED terminal speed.
%
%   *** 60 g IS AN AXIS LIMIT, NOT A MEASUREMENT ***
%   Line 14436 is axis([00,10,00,60]) -- a PLOT RANGE.  It is only a lower
%   bound on the peak.  The claim "very large commanded acceleration" is
%   carried by the prose at line 14463, not by that axis call.  Much of that
%   peak is the 1/tgo^2 singularity as tgo -> 0, not the nominal demand.
%
%   USAGE
%       out = tshape3d()                             % book nominal case
%       out = tshape3d('GAMFPDEG',-60,'GAMFYDEG',0)
%       out = tshape3d('XNTG',0)                     % stationary-ish target
%
%   OPTIONS (defaults = the book's data statement, line 14417)
%       XNTG      target maneuver, g                                  [4]
%       VT        target speed, ft/s                                  [1000]
%       VM        missile speed, ft/s                                 [3000]
%       RM1,RM2,RM3  missile initial position, ft            [0,10000,-1000]
%       RT1,RT2,RT3  target  initial position, ft            [30000,10000,0]
%       GAMFPDEG  desired terminal PITCH flight-path angle, deg       [-30]
%       GAMFYDEG  desired terminal YAW   flight-path angle, deg       [20]
%       BETA      initial target flight-path angle, rad               [0]
%       HSCALE    multiplier on the book's step H (1 = book)          [1]
%       PLOTS     reproduce Figs. A.14-A.18                           [false]
%
%   UNITS: feet / ft-s throughout.
%   Axis convention: 1 = downrange, 2 = altitude, 3 = crossrange (line 14452).
%
%   TRANSCRIPTION REPAIRS
%       "while ~(VC,0)"  -> while ~(VC < 0)
%       "if RTM,1000."   -> if RTM < 1000.
%       "while STEP,=1"  -> while STEP <= 1
%       "if S.=.0999"    -> if S >= .0999
%       "if VC,0"        -> if VC < 0        (inside LAUNCHLOGIC)
%   These restorations are inference, not source.
%
%   DEVIATIONS FROM THE LISTING (all of them)
%   -----------------------------------------
%   1. Wrapped in a function with name/value options; defaults = line 14417.
%   2. HSCALE added.  The book runs H = 1e-4 (1e-5 inside 1000 ft), which is
%      ~1e5 steps.  HSCALE > 1 coarsens it for quick looks; HSCALE = 1 is
%      the book and is the default.
%   3. Preallocation; runaway guard (T > 60 s).
%   4. The final log entry after the loop (the book appends one more sample
%      outside the while) is kept.
%   Nothing else.  LAUNCHLOGIC is transcribed unchanged, including its
%   0.1-s search grid and its stated validity limit of 10 s flight time.
%
%   APPLICABILITY
%   -------------
%   * ZERO lag, NO acceleration limit, NO gravity, NO drag, NO thrust.
%   * The law needs the TARGET ACCELERATION nT.  Ours is a stationary sea
%     target, nT = 0, which is the easy end of this law's assumptions.
%   * 1/tgo^2 singularity: the command diverges as tgo -> 0 whenever any
%     residual remains.  Any port must gate or floor tgo.
%   * LAUNCHLOGIC assumes a non-maneuvering target and tF < 10 s.

%% ---- options ------------------------------------------------------------
p = struct('XNTG',4.,'VT',1000.,'VM',3000., ...
           'RM1',0.,'RM2',10000.,'RM3',-1000., ...
           'RT1',30000.,'RT2',10000.,'RT3',0., ...
           'GAMFPDEG',-30.,'GAMFYDEG',20.,'BETA',0., ...
           'HSCALE',1,'PLOTS',false,'SAVEDAT','');
for k = 1:2:numel(varargin)
    f = varargin{k};
    if ~isfield(p,f), error('tshape3d: unknown option %s',f); end
    p.(f) = varargin{k+1};
end

%% =======================================================================
%  LISTING A.6  (line by line)
%  =======================================================================
n=0; XNTG=p.XNTG; VT=p.VT; VM=p.VM;
RM1=p.RM1; RM2=p.RM2; RM3=p.RM3;
RT1=p.RT1; RT2=p.RT2; RT3=p.RT3;
GAMFPDEG=p.GAMFPDEG; GAMFYDEG=p.GAMFYDEG;
XNT=32.2*XNTG; BETA=p.BETA;
VT1=-VT*cos(BETA); VT2=VT*sin(BETA); VT3=0;
GAMFP=GAMFPDEG/57.3; GAMFY=GAMFYDEG/57.3;
[VM1,VM2,VM3,TF]=LAUNCHLOGIC(RM1,RM2,RM3,RT1,RT2,RT3,VT1,VT2,VT3,VM);
H=.0001*p.HSCALE; T=0.; S=0.;
RTM1=RT1-RM1; RTM2=RT2-RM2; RTM3=RT3-RM3;
RTM=sqrt(RTM1^2+RTM2^2+RTM3^2);
VTM1=VT1-VM1; VTM2=VT2-VM2; VTM3=VT3-VM3;
VC=-(RTM1*VTM1+RTM2*VTM2+RTM3*VTM3)/RTM;
VM1F=VM*cos(GAMFP)*cos(GAMFY);      % <-- computed ONCE.  See "TRAP" above.
VM2F=VM*sin(GAMFP);
VM3F=VM*cos(GAMFP)*sin(GAMFY);

NMAX=200000;
ArrayT=zeros(1,NMAX); ArrayRM1K=ArrayT; ArrayRM2K=ArrayT; ArrayRM3K=ArrayT;
ArrayRT1K=ArrayT; ArrayRT2K=ArrayT; ArrayRT3K=ArrayT;
ArrayGAMPDEG=ArrayT; ArrayGAMFPDEG=ArrayT;
ArrayGAMYDEG=ArrayT; ArrayGAMFYDEG=ArrayT; ArrayXNCG=ArrayT;
AM1=0.; AM2=0.; AM3=0.; BETAD=XNT/VT;
GAMPDEG=0.; GAMYDEG=0.; XNCG=0.;

while ~(VC < 0)
    if RTM < 1000.
        H=.00001*p.HSCALE;
    else
        H=.0001*p.HSCALE;
    end
    BETAOLD=BETA; RT1OLD=RT1; RT2OLD=RT2; RT3OLD=RT3;
    RM1OLD=RM1; RM2OLD=RM2; RM3OLD=RM3;
    VM1OLD=VM1; VM2OLD=VM2; VM3OLD=VM3;
    STEP=1; FLAG=0;
    while STEP <= 1
        if FLAG==1
            STEP=2;
            BETA=BETA+H*BETAD;
            RT1=RT1+H*VT1; RT2=RT2+H*VT2; RT3=RT3+H*VT3;
            RM1=RM1+H*VM1; RM2=RM2+H*VM2; RM3=RM3+H*VM3;
            VM1=VM1+H*AM1; VM2=VM2+H*AM2; VM3=VM3+H*AM3;
            T=T+H;
        end
        RTM1=RT1-RM1; RTM2=RT2-RM2; RTM3=RT3-RM3;
        RTM=sqrt(RTM1^2+RTM2^2+RTM3^2);
        VTM1=VT1-VM1; VTM2=VT2-VM2; VTM3=VT3-VM3;
        VC=-(RTM1*VTM1+RTM2*VTM2+RTM3*VTM3)/RTM;
        TGO=RTM/VC;
        BETAD=XNT/VT;
        XNT1=XNT*sin(BETA); XNT2=XNT*cos(BETA); XNT3=0.;
        VT1=-VT*cos(BETA); VT2=VT*sin(BETA); VT3=0.;
        VM=sqrt(VM1*VM1+VM2*VM2+VM3*VM3);
        XNC1=((6.*RTM1+6.*VTM1*TGO)/TGO^2) +2.0*(VM1-VM1F)/(TGO)+XNT1;
        XNC2=((6.*RTM2+6.*VTM2*TGO)/TGO^2) +2.0*(VM2-VM2F)/(TGO)+XNT2;
        XNC3=((6.*RTM3+6.*VTM3*TGO)/TGO^2) +2.0*(VM3-VM3F)/(TGO)+XNT3;
        XNCG=sqrt(XNC1^2+XNC2^2+XNC3^2)/32.2;
        % ---- projection PERPENDICULAR TO THE VELOCITY VECTOR -----------
        XNCDOTVM=(XNC1*VM1+XNC2*VM2+XNC3*VM3)/VM;
        AM1=XNC1-XNCDOTVM*VM1/VM;
        AM2=XNC2-XNCDOTVM*VM2/VM;
        AM3=XNC3-XNCDOTVM*VM3/VM;
        GAMYDEG=57.3*atan2(VM3,VM1);
        GAMPDEG=57.3*atan2(VM2,sqrt(VM1^2+VM3^2));
        FLAG=1;
    end
    FLAG=0;
    BETA=.5*(BETAOLD+BETA+H*BETAD);
    RT1=.5*(RT1OLD+RT1+H*VT1); RT2=.5*(RT2OLD+RT2+H*VT2);
    RT3=.5*(RT3OLD+RT3+H*VT3);
    RM1=.5*(RM1OLD+RM1+H*VM1); RM2=.5*(RM2OLD+RM2+H*VM2);
    RM3=.5*(RM3OLD+RM3+H*VM3);
    VM1=.5*(VM1OLD+VM1+H*AM1); VM2=.5*(VM2OLD+VM2+H*AM2);
    VM3=.5*(VM3OLD+VM3+H*AM3);
    S=S+H;
    if S >= .0999
        S=0.; n=n+1;
        RT1K=RT1/1000.; RT2K=RT2/1000.; RT3K=RT3/1000.;
        RM1K=RM1/1000.; RM2K=RM2/1000.; RM3K=RM3/1000.;
        ArrayT(n)=T;
        ArrayRM1K(n)=RM1K; ArrayRM2K(n)=RM2K; ArrayRM3K(n)=RM3K;
        ArrayRT1K(n)=RT1K; ArrayRT2K(n)=RT2K; ArrayRT3K(n)=RT3K;
        ArrayGAMPDEG(n)=GAMPDEG; ArrayGAMFPDEG(n)=GAMFPDEG;
        ArrayGAMYDEG(n)=GAMYDEG; ArrayGAMFYDEG(n)=GAMFYDEG;
        ArrayXNCG(n)=XNCG;
    end
    if T > 60 || n >= NMAX-2, break; end       % deviation 3
end
n=n+1;
RT1K=RT1/1000.; RT2K=RT2/1000.; RT3K=RT3/1000.;
RM1K=RM1/1000.; RM2K=RM2/1000.; RM3K=RM3/1000.;
ArrayT(n)=T;
ArrayRM1K(n)=RM1K; ArrayRM2K(n)=RM2K; ArrayRM3K(n)=RM3K;
ArrayRT1K(n)=RT1K; ArrayRT2K(n)=RT2K; ArrayRT3K(n)=RT3K;
ArrayGAMPDEG(n)=GAMPDEG; ArrayGAMFPDEG(n)=GAMFPDEG;
ArrayGAMYDEG(n)=GAMYDEG; ArrayGAMFYDEG(n)=GAMFYDEG;
ArrayXNCG(n)=XNCG;
RTM=sqrt(RTM1^2+RTM2^2+RTM3^2);
%  ========================== end of Listing A.6 ==========================

ArrayT=ArrayT(1:n); ArrayRM1K=ArrayRM1K(1:n); ArrayRM2K=ArrayRM2K(1:n);
ArrayRM3K=ArrayRM3K(1:n); ArrayRT1K=ArrayRT1K(1:n); ArrayRT2K=ArrayRT2K(1:n);
ArrayRT3K=ArrayRT3K(1:n); ArrayGAMPDEG=ArrayGAMPDEG(1:n);
ArrayGAMFPDEG=ArrayGAMFPDEG(1:n); ArrayGAMYDEG=ArrayGAMYDEG(1:n);
ArrayGAMFYDEG=ArrayGAMFYDEG(1:n); ArrayXNCG=ArrayXNCG(1:n);

if p.PLOTS
    figure, plot(ArrayRM1K,ArrayRM2K,ArrayRT1K,ArrayRT2K),grid
    xlabel('Downrange (kft)'), ylabel('Altitude (kft)'), title('Fig. A.14')
    figure, plot(ArrayRM1K,ArrayRM3K,ArrayRT1K,ArrayRT3K),grid
    xlabel('Downrange (kft)'), ylabel('Crossrange (kft)'), title('Fig. A.15')
    figure, plot(ArrayT,ArrayGAMPDEG,ArrayT,ArrayGAMFPDEG),grid
    xlabel('Time (Sec)'), ylabel('Pitch Reentry Angle (deg)'), title('Fig. A.16')
    figure, plot(ArrayT,ArrayGAMYDEG,ArrayT,ArrayGAMFYDEG),grid
    xlabel('Time (Sec)'), ylabel('Yaw Reentry Angle (deg)'), title('Fig. A.17')
    figure, plot(ArrayT,ArrayXNCG),grid
    xlabel('Time (Sec)'), ylabel('Commanded Acceleration (g)')
    axis([00,10,00,60]), title('Fig. A.18  (60 is an AXIS LIMIT)')
end
if ~isempty(p.SAVEDAT)
    output=[ArrayT',ArrayRM1K',ArrayRM2K',ArrayRM3K',ArrayRT1K', ...
            ArrayRT2K',ArrayRT3K',ArrayGAMPDEG',ArrayGAMFPDEG', ...
            ArrayGAMYDEG',ArrayGAMFYDEG']; %#ok<NASGU>
    save(p.SAVEDAT,'output','-ascii');
end

out              = struct();
out.opts         = p;
out.TF_launch    = TF;              % LAUNCHLOGIC's collision-triangle guess
out.tf           = T;
out.miss_ft      = RTM;
out.T            = ArrayT;
out.RM1K=ArrayRM1K; out.RM2K=ArrayRM2K; out.RM3K=ArrayRM3K;
out.RT1K=ArrayRT1K; out.RT2K=ArrayRT2K; out.RT3K=ArrayRT3K;
out.GAMPDEG      = ArrayGAMPDEG;
out.GAMYDEG      = ArrayGAMYDEG;
out.XNCG         = ArrayXNCG;
out.gamP_f_deg   = GAMPDEG;
out.gamY_f_deg   = GAMYDEG;
out.xnc_max_g    = max(ArrayXNCG);
out.vm_f         = sqrt(VM1^2+VM2^2+VM3^2);
end

% =========================================================================
function [VM1,VM2,VM3,TF]=LAUNCHLOGIC(RM1,RM2,RM3,RT1,RT2,RT3, ...
                                      VT1,VT2,VT3,VM)
%LAUNCHLOGIC  Listing A.6's launch subroutine, verbatim.
%   Places the missile on a collision triangle assuming NO target maneuver.
%   Valid only for flight times below 10 s (the search grid stops there).
VM1=VM; VM2=0; VM3=0;
for TF=.1:.1:10
    RTM1=RT1-RM1; RTM2=RT2-RM2; RTM3=RT3-RM3; %#ok<NASGU>
    RT1F=RT1+VT1*TF; RT2F=RT2+VT2*TF; RT3F=RT3+VT3*TF;
    THET=asin((RT2F-RM2)/(VM*TF));
    PSI=atan2(RT3F-RM3,RT1F-RM1);
    VM1=VM*cos(THET)*cos(PSI); VM2=VM*sin(THET); VM3=VM*cos(THET)*sin(PSI);
    RM1F=RM1+VM1*TF; RM2F=RM2+VM2*TF; RM3F=RM3+VM3*TF;
    RTM1F=RT1F-RM1F; RTM2F=RT2F-RM2F; RTM3F=RT3F-RM3F;
    RTMF=sqrt(RTM1F^2+RTM2F^2+RTM3F^2);
    VTM1=VT1-VM1; VTM2=VT2-VM2; VTM3=VT3-VM3;
    VC=-(RTM1F*VTM1+RTM2F*VTM2+RTM3F*VTM3)/RTMF;
    if VC < 0
        break
    end
end
end
