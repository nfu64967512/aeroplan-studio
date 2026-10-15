function out = zem3d(varargin)
%ZEM3D  Zarchan Listing 28.1 -- three-dimensional engagement, ZEM guidance.
%
%       ZEM      = RTM + VTM*tgo                      (QPN=1, proportional nav)
%       ZEMPAR   = (ZEM . RTM/|RTM|) * RTM/|RTM|
%       ZEMPER   = ZEM - ZEMPAR                       (VECTOR subtraction)
%       AMiP     = XNP * ZEMPERi / tgo^2              (ONE XNP, all three axes)
%       AMiD     = (AMiP - AMi)/TAU                   (first-order lag)
%
%   SOURCE (local full text, zarchan.txt)
%     line 10213  the chapter's ZEM framing
%     line 10231  ZEMDOTRTM, a SCALAR that eats all three axes
%     line 10235  ZEMPER = ZEM - ZEMPAR, a VECTOR subtraction
%     line 10238  ZEM perpendicular to the LOS
%     line 10245  nc_i = N' * ZEMPER_i / tgo^2
%     line 10252  verbatim: "the effective navigation ratio N' is a constant
%                 set equal to three"   <-- SINGULAR
%     line 10296  "LISTING 28.1 THREE-DIMENSIONAL TACTICAL ENGAGEMENT
%                  SIMULATION WITH SPIRALING TARGET"
%     lines 10297-10324  the code, transcribed below
%     line 10326  Figs. 28.1-28.4 discussion (total miss ~ constant vs
%                 flight time; crossrange miss ~ zero)
%
%   *** WHY THIS FILE MATTERS TO US ***
%   This is ONE VECTOR LAW RESOLVED INTO COMPONENTS, not three independent
%   scalar channels.  Three pieces of evidence, all in the source:
%     (a) line 10252 says N' is A constant, singular;
%     (b) line 10297 declares a single XNP=3. and line 10310 uses that same
%         XNP on all three axes;
%     (c) ZEMDOTRTM is one scalar computed from all three components, so
%         the axes are NOT separable.
%   Our vehicle uses n_lat = 4.0 and n_vert = 5.0 -- two independent scalar
%   channels with DIFFERENT gains.  That has no counterpart anywhere in
%   Zarchan.  Do not cite Listing 28.1 as precedent for it.
%
%   *** SECOND TRAP: THIS EXAMPLE HAS NO ACCELERATION LIMIT ***
%   Line 10297 sets XNCLIM=9999999999.  Every performance claim Ch. 28 makes
%   is therefore made under UNLIMITED authority.  It cannot be used to argue
%   anything about our nz in [-1, 3].
%
%   *** THIRD TRAP: ZARCHAN CLAMPS THE ACHIEVED STATE, WE CLAMP THE COMMAND ***
%   Line 10310 computes AMiD = (AMiP - AMi)/TAU from the UNCLAMPED command
%   AMiP, and only afterwards (lines 10311-10318) clamps the STATE AMi.  The
%   lag keeps chasing the true demand, so authority returns the instant the
%   demand drops.  We clamp the COMMAND (nz = clip(...)), so our loop never
%   sees the excess demand at all.  Different semantics; not interchangeable.
%
%   USAGE
%       out = zem3d()                        % book nominal, full 80-case sweep
%       out = zem3d('RT3IC',40000:-2500:500) % coarse sweep (fast)
%       out = zem3d('QPN',0)                 % optimal guidance w/ lag instead
%
%   OPTIONS (defaults = the book's data statement, line 10297)
%       QPN     1 = proportional navigation, 0 = optimal guidance      [1]
%       TAU     guidance system time constant, s                       [1]
%       W       target weave frequency, rad/s                          [3]
%       AT      target maneuver amplitude, ft/s^2                      [193.2]
%       VT      target speed, ft/s                                     [1000]
%       VM      missile speed, ft/s                                    [3000]
%       XNP     effective navigation ratio (QPN=1 only)                [3]
%       XNCLIM  acceleration limit, ft/s^2                    [9999999999]
%       RT3IC   vector of target initial downrange, ft     [40000:-500:500]
%       PLOTS   reproduce Figs. 28.1-28.4                              [false]
%
%   OUTPUT (one entry per engagement in the RT3IC sweep)
%       .T, .RTM1, .RTM2, .RTM3, .RTM      miss components and total, ft
%
%   UNITS: feet / ft-s throughout.
%
%   TRANSCRIPTION REPAIRS (PDF text layer destroys the operators)
%       "while VC.=0"     -> while VC >= 0
%       "if RTM,1000"     -> if RTM < 1000
%       "while STEP,=1"   -> while STEP <= 1
%       "if AM1.XNCLIM"   -> if AM1 > XNCLIM
%       "if AM1,-XNCLIM"  -> if AM1 < -XNCLIM
%       "if S.=.09999"    -> if S >= .09999
%   These restorations are inference, not source.
%
%   DEVIATIONS FROM THE LISTING (all of them)
%   -----------------------------------------
%   1. Wrapped in a function with name/value options; defaults = line 10297.
%   2. RT3IC exposed as a vector so the sweep can be coarsened.  The book's
%      40000:-500:500 (80 engagements) is the default and is what the
%      figures were made from.
%   3. Preallocation; runaway guard (T > 60 s per engagement).
%   4. The book's 0.1-s block computes RT1K..RM3K and then throws them away
%      (it stores nothing).  Kept as-is -- it is dead code in the original
%      too -- but the values are not returned.
%   Nothing else.  In particular the clamp-the-state ordering above is kept
%   exactly as printed, because it is the semantically load-bearing part.
%
%   APPLICABILITY
%   -------------
%   * Constant missile SPEED is assumed only in the initial condition; the
%     commanded acceleration here is perpendicular to the LOS, so |VM|
%     drifts during the run.  No drag, no thrust, no gravity.
%   * The single first-order lag TAU is the ONLY dynamics.  Our loop has
%     an airframe/attitude lag (tau ~= 1.50 s when the LOS floor is not
%     firing) PLUS a 1.75 s alpha lowpass PLUS a 1.0 s PN lead PLUS 20 Hz
%     command quantisation.  tau here is not our tau.
%   * tgo = RTM/VC is exact only for non-accelerating geometry.

%% ---- options ------------------------------------------------------------
p = struct('QPN',1,'TAU',1.,'W',3.,'AT',193.2,'VT',1000.,'VM',3000., ...
           'XNP',3.,'XNCLIM',9999999999.,'RT3IC',40000:-500:500, ...
           'PLOTS',false,'SAVEDAT','');
for k = 1:2:numel(varargin)
    f = varargin{k};
    if ~isfield(p,f), error('zem3d: unknown option %s',f); end
    p.(f) = varargin{k+1};
end

%% =======================================================================
%  LISTING 28.1  (line by line)
%  =======================================================================
count=0; QPN=p.QPN; TAU=p.TAU; W=p.W; AT=p.AT; VT=p.VT; VM=p.VM;
XNP0=p.XNP; XNCLIM=p.XNCLIM;

NC=numel(p.RT3IC);
ArrayT=zeros(1,NC); ArrayRTM1=ArrayT; ArrayRTM2=ArrayT; ArrayRTM3=ArrayT;
ArrayRTM=ArrayT;

for RT3IC=p.RT3IC
    XNP=XNP0;                       % QPN=0 overwrites XNP each step
    RM1=0.; RM2=10000.; RM3=0.; RT1=0.; RT2=10000.; RT3=RT3IC;
    VT1=-AT/W; VT2=0.; VT3=-VT; T=0.; S=0.;
    RTM1=RT1-RM1; RTM2=RT2-RM2; RTM3=RT3-RM3;
    RTM=sqrt(RTM1^2+RTM2^2+RTM3^2);
    VM1=0.; VM2=0.; VM3=VM;
    VTM1=VT1-VM1; VTM2=VT2-VM2; VTM3=VT3-VM3;
    VC=-(RTM1*VTM1+RTM2*VTM2+RTM3*VTM3)/RTM;
    AM1=0.; AM2=0.; AM3=0.;
    AM1D=0.; AM2D=0.; AM3D=0.; AT1=0.; AT2=0.; AT3=0.;   % see deviation 3

    while VC >= 0
        if RTM < 1000, H=.0002; else, H=.01; end
        RT1OLD=RT1; RT2OLD=RT2; RT3OLD=RT3;
        RM1OLD=RM1; RM2OLD=RM2; RM3OLD=RM3;
        VM1OLD=VM1; VM2OLD=VM2; VM3OLD=VM3;
        VT1OLD=VT1; VT2OLD=VT2; VT3OLD=VT3;
        AM1OLD=AM1; AM2OLD=AM2; AM3OLD=AM3;
        STEP=1; FLAG=0;
        while STEP <= 1
            if FLAG==1
                STEP=2;
                RT1=RT1+H*VT1; RT2=RT2+H*VT2; RT3=RT3+H*VT3;
                RM1=RM1+H*VM1; RM2=RM2+H*VM2; RM3=RM3+H*VM3;
                VM1=VM1+H*AM1; VM2=VM2+H*AM2; VM3=VM3+H*AM3;
                VT1=VT1+H*AT1; VT2=VT2+H*AT2; VT3=VT3+H*AT3;
                AM1=AM1+H*AM1D; AM2=AM2+H*AM2D; AM3=AM3+H*AM3D;
                T=T+H;
            end
            RTM1=RT1-RM1; RTM2=RT2-RM2; RTM3=RT3-RM3;
            RTM=sqrt(RTM1^2+RTM2^2+RTM3^2);
            VTM1=VT1-VM1; VTM2=VT2-VM2; VTM3=VT3-VM3;
            VC=-(RTM1*VTM1+RTM2*VTM2+RTM3*VTM3)/RTM;
            TGO=RTM/VC;
            AT1=AT*sin(W*T); AT2=AT*cos(W*T); AT3=0.;
            if QPN==1
                ZEM1=RTM1+VTM1*TGO; ZEM2=RTM2+VTM2*TGO; ZEM3=RTM3+VTM3*TGO;
            else
                AT1D=AT*W*cos(W*T); AT2D=-AT*W*sin(W*T); AT3D=0.;
                X=TGO/TAU;
                TOP=6.*X*X*(exp(-X)-1.+X);
                BOT1=2*X*X*X+3.+6.*X-6.*X*X;
                BOT2=-12.*X*exp(-X)-3.*exp(-2.*X);
                XNP=TOP/(.0001+BOT1+BOT2);
                ZEM1=RTM1+VTM1*TGO+AT1*(1.-cos(W*TGO))/W^2+ ...
                     (AT1D*(W*TGO-sin(W*TGO))/W^3)- ...
                     AM1*TAU*TAU*(exp(-X)+X-1.);
                ZEM2=RTM2+VTM2*TGO+AT2*(1.-cos(W*TGO))/W^2+ ...
                     (AT2D*(W*TGO-sin(W*TGO))/W^3)- ...
                     AM2*TAU*TAU*(exp(-X)+X-1.);
                ZEM3=RTM3+VTM3*TGO+AT3*(1.-cos(W*TGO))/W^2+ ...
                     (AT3D*(W*TGO-sin(W*TGO))/W^3)- ...
                     AM3*TAU*TAU*(exp(-X)+X-1.);
            end
            % ---- ONE scalar projection, three components ---------------
            ZEMDOTRTM=(ZEM1*RTM1+ZEM2*RTM2+ZEM3*RTM3)/RTM;
            ZEMPER1=ZEM1-ZEMDOTRTM*RTM1/RTM;
            ZEMPER2=ZEM2-ZEMDOTRTM*RTM2/RTM;
            ZEMPER3=ZEM3-ZEMDOTRTM*RTM3/RTM;
            AM1P=XNP*ZEMPER1/(TGO*TGO);
            AM2P=XNP*ZEMPER2/(TGO*TGO);
            AM3P=XNP*ZEMPER3/(TGO*TGO);
            % lag driven by the UNCLAMPED command AMiP  (see THIRD TRAP)
            AM1D=(AM1P-AM1)/TAU; AM2D=(AM2P-AM2)/TAU; AM3D=(AM3P-AM3)/TAU;
            % then the STATE is clamped
            if AM1>XNCLIM,  AM1=XNCLIM;  end
            if AM1<-XNCLIM, AM1=-XNCLIM; end
            if AM2>XNCLIM,  AM2=XNCLIM;  end
            if AM2<-XNCLIM, AM2=-XNCLIM; end
            if AM3>XNCLIM,  AM3=XNCLIM;  end
            if AM3<-XNCLIM, AM3=-XNCLIM; end
            XNCG=sqrt(AM1^2+AM2^2+AM3^2)/32.2; %#ok<NASGU>
            FLAG=1;
        end
        FLAG=0;
        RT1=.5*(RT1OLD+RT1+H*VT1); RT2=.5*(RT2OLD+RT2+H*VT2);
        RT3=.5*(RT3OLD+RT3+H*VT3);
        RM1=.5*(RM1OLD+RM1+H*VM1); RM2=.5*(RM2OLD+RM2+H*VM2);
        RM3=.5*(RM3OLD+RM3+H*VM3);
        VM1=.5*(VM1OLD+VM1+H*AM1); VM2=.5*(VM2OLD+VM2+H*AM2);
        VM3=.5*(VM3OLD+VM3+H*AM3);
        VT1=.5*(VT1OLD+VT1+H*AT1); VT2=.5*(VT2OLD+VT2+H*AT2);
        VT3=.5*(VT3OLD+VT3+H*AT3);
        AM1=.5*(AM1OLD+AM1+H*AM1D); AM2=.5*(AM2OLD+AM2+H*AM2D);
        AM3=.5*(AM3OLD+AM3+H*AM3D);
        S=S+H;
        if S >= .09999
            S=0.;
            % the book computes RT1K..RM3K here and stores nothing (dead
            % code in the original; deviation 4)
        end
        if T > 60, break; end                    % deviation 3
    end
    count=count+1;
    ArrayT(count)=T; ArrayRTM1(count)=RTM1; ArrayRTM2(count)=RTM2;
    ArrayRTM3(count)=RTM3; ArrayRTM(count)=RTM;
end
%  ========================= end of Listing 28.1 ==========================

if p.PLOTS
    figure, plot(ArrayT,ArrayRTM1),grid
    xlabel('Time (s)'), ylabel('RTM1 (ft)'), axis([0 10 -30 30])
    figure, plot(ArrayT,ArrayRTM2),grid
    xlabel('Time (s)'), ylabel('RTM2 (ft)'), axis([0 10 -30 30])
    figure, plot(ArrayT,ArrayRTM3),grid
    xlabel('Time (s)'), ylabel('RTM3 (ft)'), axis([0 10 -30 30])
    figure, plot(ArrayT,ArrayRTM),grid
    xlabel('Time (s)'), ylabel('RTM (ft)'), axis([0 10 0 30])
end
if ~isempty(p.SAVEDAT)
    output=[ArrayT',ArrayRTM1',ArrayRTM2',ArrayRTM3',ArrayRTM']; %#ok<NASGU>
    save(p.SAVEDAT,'output','-ascii');
end

out       = struct();
out.opts  = p;
out.T     = ArrayT;
out.RTM1  = ArrayRTM1;  out.RTM2 = ArrayRTM2;  out.RTM3 = ArrayRTM3;
out.RTM   = ArrayRTM;
end
