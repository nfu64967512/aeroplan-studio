function out = pn2d(varargin)
%PN2D  Zarchan Listing 2.1 -- two-dimensional tactical engagement simulation.
%
%   True proportional navigation,  nc = N' * Vc * lamdot,  command applied
%   PERPENDICULAR TO THE LINE OF SIGHT.
%
%   SOURCE (local full text, zarchan.txt)
%     line  409  "LISTING 2.1 TWO-DIMENSIONAL TACTICAL MISSILE-TARGET
%                 ENGAGEMENT SIMULATION"
%     lines 410-427   the code, transcribed below line by line
%     line  428  the prose describing the listing
%     line  431  Fig. 2.2 caption: "Increasing effective navigation ratio
%                causes heading error to be removed more rapidly."
%
%   USAGE
%       out = pn2d()                       % book nominal: HEDEG=-20, XNP=4
%       out = pn2d('XNP',5)
%       out = pn2d('HEDEG',0,'XNT',96.6)   % the 3-g maneuver case (line 438)
%
%   OPTIONS (defaults are the book's nominal data statement, line 410)
%       VM      missile speed, ft/s                                  [3000]
%       VT      target  speed, ft/s                                  [1000]
%       XNT     target maneuver, ft/s^2                              [0]
%       HEDEG   heading error, deg                                   [-20]
%       XNP     effective navigation ratio N'                        [4]
%       RM1,RM2 missile initial position, ft                         [0,10000]
%       RT1,RT2 target  initial position, ft                         [40000,10000]
%       BETA    initial target flight-path angle, rad                [0]
%       PLOTS   reproduce the listing's two figures                  [false]
%       SAVEDAT path for the datfil.txt equivalent, '' = skip        ['']
%
%   OUTPUT (struct)
%       .miss_ft   RTM at termination = miss distance (always positive,
%                  it is computed from the distance formula -- line 428)
%       .tf        flight time, s
%       .T,.RT1,.RT2,.RM1,.RM2,.XNCG,.RTM   the book's 0.1-s log arrays
%       .GAMDEG    missile flight-path angle, deg  (NOT in the book's log;
%                  added so the heading-error-removal claim of Fig. 2.2 can
%                  be measured directly -- see DEVIATIONS)
%
%   UNITS: feet / ft-s throughout, exactly as the book. No metric anywhere.
%
%   TRANSCRIPTION REPAIRS
%   ---------------------
%   The PDF text layer destroys MATLAB's comparison operators: "," is "<"
%   and "." is ">".  Restored (this restoration is inference, not source):
%       "while VC .= 0"    -> while VC >= 0
%       "if RTM , 1000"    -> if RTM < 1000
%       "while STEP ,=1"   -> while STEP <= 1
%       "if S . =.09999"   -> if S >= .09999
%
%   DEVIATIONS FROM THE LISTING (all of them, and why)
%   --------------------------------------------------
%   1. Wrapped in a function with name/value options.  The book's line 410
%      is a bare data statement; every default here equals that line.
%   2. Arrays preallocated then trimmed (speed only; identical values).
%   3. GAMDEG = atan2(VM2,VM1)*57.3 added to the 0.1-s log block.  The book
%      logs XNC/32.2 and RTM but not gamma; demo_reproduce needs gamma to
%      test the Fig. 2.2 claim quantitatively.  Pure instrumentation -- it
%      is never fed back into the state.
%   4. Runaway guard (T > 600 s or array full) so a mis-specified case
%      cannot hang MATLAB.  Never trips on the book's cases.
%   Nothing else.  In particular the integrator (the book's own two-pass
%   predictor / trapezoidal-corrector "STEP/FLAG" scheme), the step-size
%   schedule (H = 0.01, and 0.0002 inside 1000 ft), and the termination
%   test (VC changes sign) are kept exactly.
%
%   APPLICABILITY -- READ BEFORE USING THE NUMBERS
%   ----------------------------------------------
%   * ZERO guidance-system lag.  There is no autopilot, no seeker, no
%     actuator; XNC becomes AM instantaneously.  Line 621 (verbatim):
%     "in the absence of acceleration saturation effects, zero miss
%     distance can be achieved if there are no lags within the homing
%     loop".  Our vehicle has tau ~= 1.50 s.  Miss distances out of this
%     file are therefore floor values (essentially the integration
%     resolution), NOT predictions of our system.
%   * NO acceleration limit.  AM is whatever the law asks for.
%   * Constant missile SPEED is not enforced: the command is perpendicular
%     to the LOS, not to the velocity vector, so |VM| drifts.  This is the
%     mechanism Ch. 30 calls out at line 10980.  Use purepn2d with
%     ICHOICE=1 if you want speed preserved.
%   * Miss-distance resolution is Vc*H_final.  Line 640 (verbatim context):
%     with H = 0.0002 s and Vc ~ 4000 ft/s that is ~0.8 ft.  Do not read
%     any miss below ~1 ft out of this file as physical.

%% ---- options ------------------------------------------------------------
p = struct('VM',3000.,'VT',1000.,'XNT',0.,'HEDEG',-20.,'XNP',4., ...
           'RM1',0.,'RM2',10000.,'RT1',40000.,'RT2',10000.,'BETA',0., ...
           'PLOTS',false,'SAVEDAT','');
for k = 1:2:numel(varargin)
    f = varargin{k};
    if ~isfield(p,f), error('pn2d: unknown option %s',f); end
    p.(f) = varargin{k+1};
end

%% =======================================================================
%  LISTING 2.1  (line by line)
%  =======================================================================
n=0; VM=p.VM; VT=p.VT; XNT=p.XNT; HEDEG=p.HEDEG; XNP=p.XNP;
RM1=p.RM1; RM2=p.RM2;
RT1=p.RT1; RT2=p.RT2; BETA=p.BETA;
VT1=-VT*cos(BETA); VT2=VT*sin(BETA); HE=HEDEG/57.3; T=0.; S=0.;
RTM1=RT1-RM1; RTM2=RT2-RM2; RTM=sqrt(RTM1*RTM1+RTM2*RTM2);
XLAM=atan2(RTM2,RTM1);
XLEAD=asin(VT*sin(BETA+XLAM)/VM);
THET=XLAM+XLEAD;
VM1=VM*cos(THET+HE); VM2=VM*sin(THET+HE);
VTM1=VT1-VM1; VTM2=VT2-VM2;
VC=-(RTM1*VTM1+RTM2*VTM2)/RTM;

% preallocation (deviation 2)
NMAX=200000;
ArrayT=zeros(1,NMAX); ArrayRT1=ArrayT; ArrayRT2=ArrayT; ArrayRM1=ArrayT;
ArrayRM2=ArrayT; ArrayXNCG=ArrayT; ArrayRTM=ArrayT; ArrayGAMDEG=ArrayT;
AM1=0.; AM2=0.; XNC=0.; BETAD=XNT/VT;

while VC >= 0
    if RTM < 1000
        H=.0002;
    else
        H=.01;
    end
    BETAOLD=BETA; RT1OLD=RT1; RT2OLD=RT2; RM1OLD=RM1; RM2OLD=RM2;
    VM1OLD=VM1; VM2OLD=VM2;
    STEP=1; FLAG=0;
    while STEP <= 1
        if FLAG==1
            STEP=2;
            BETA=BETA+H*BETAD; RT1=RT1+H*VT1; RT2=RT2+H*VT2;
            RM1=RM1+H*VM1; RM2=RM2+H*VM2;
            VM1=VM1+H*AM1; VM2=VM2+H*AM2; T=T+H;
        end
        % ---- differential equations (always before FLAG=1) -------------
        RTM1=RT1-RM1; RTM2=RT2-RM2; RTM=sqrt(RTM1*RTM1+RTM2*RTM2);
        VTM1=VT1-VM1; VTM2=VT2-VM2;
        VC=-(RTM1*VTM1+RTM2*VTM2)/RTM;
        XLAM=atan2(RTM2,RTM1);
        XLAMD=(RTM1*VTM2-RTM2*VTM1)/(RTM*RTM);
        XNC=XNP*VC*XLAMD;
        AM1=-XNC*sin(XLAM); AM2=XNC*cos(XLAM);
        VT1=-VT*cos(BETA); VT2=VT*sin(BETA); BETAD=XNT/VT;
        FLAG=1;
    end
    FLAG=0;
    BETA=.5*(BETAOLD+BETA+H*BETAD);
    RT1=.5*(RT1OLD+RT1+H*VT1); RT2=.5*(RT2OLD+RT2+H*VT2);
    RM1=.5*(RM1OLD+RM1+H*VM1); RM2=.5*(RM2OLD+RM2+H*VM2);
    VM1=.5*(VM1OLD+VM1+H*AM1); VM2=.5*(VM2OLD+VM2+H*AM2);
    S=S+H;
    if S >= .09999
        S=0.; n=n+1;
        ArrayT(n)=T; ArrayRT1(n)=RT1; ArrayRT2(n)=RT2;
        ArrayRM1(n)=RM1; ArrayRM2(n)=RM2;
        ArrayXNCG(n)=XNC/32.2; ArrayRTM(n)=RTM;
        ArrayGAMDEG(n)=atan2(VM2,VM1)*57.3;      % deviation 3
    end
    if T > 600 || n >= NMAX, break; end          % deviation 4
end
RTM = sqrt(RTM1*RTM1+RTM2*RTM2);
%  ========================= end of Listing 2.1 ===========================

ArrayT=ArrayT(1:n); ArrayRT1=ArrayRT1(1:n); ArrayRT2=ArrayRT2(1:n);
ArrayRM1=ArrayRM1(1:n); ArrayRM2=ArrayRM2(1:n); ArrayXNCG=ArrayXNCG(1:n);
ArrayRTM=ArrayRTM(1:n); ArrayGAMDEG=ArrayGAMDEG(1:n);

if p.PLOTS
    figure
    plot(ArrayRT1,ArrayRT2,ArrayRM1,ArrayRM2),grid
    title('Two-dimensional tactical missile-target engagement simulation')
    xlabel('Downrange (Ft)'), ylabel('Altitude (Ft)')
    figure
    plot(ArrayT,ArrayXNCG),grid
    title('Two-dimensional tactical missile-target engagement simulation')
    xlabel('Time (sec)'), ylabel('Acceleration of missle (G)')
end
if ~isempty(p.SAVEDAT)
    output=[ArrayT',ArrayRT1',ArrayRT2',ArrayRM1',ArrayRM2', ...
            ArrayXNCG',ArrayRTM']; %#ok<NASGU>
    save(p.SAVEDAT,'output','-ascii');
end

out          = struct();
out.opts     = p;
out.miss_ft  = RTM;
out.tf       = T;
out.T        = ArrayT;
out.RT1      = ArrayRT1;   out.RT2 = ArrayRT2;
out.RM1      = ArrayRM1;   out.RM2 = ArrayRM2;
out.XNCG     = ArrayXNCG;  out.RTM = ArrayRTM;
out.GAMDEG   = ArrayGAMDEG;
out.xnc0_g   = ArrayXNCG(1);
out.xncmax_g = max(abs(ArrayXNCG));
out.vm_f     = sqrt(VM1^2+VM2^2);
end
