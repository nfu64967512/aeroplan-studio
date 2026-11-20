function out = adjoint_miss(varargin)
%ADJOINT_MISS  Zarchan Listing 3.1 -- single-lag adjoint of the homing loop.
%
%   ONE run gives miss distance for EVERY flight time, for BOTH a step
%   target maneuver and an initial heading error.
%
%   =====================================================================
%   ##  READ THIS BEFORE YOU BELIEVE ANY CURVE THIS FILE DRAWS  ##
%   =====================================================================
%   The method of adjoints is only defined for a LINEAR, DETERMINISTIC,
%   TIME-VARYING system.  Line 650 verbatim:
%       "For every linear deterministic system, there exists an adjoint
%        system ... by application of three Rules"
%   This implementation adds three more assumptions on top:
%       A1  the homing loop is LINEARISED about a collision triangle
%           (small angles, constant closing velocity Vc)
%       A2  the guidance system is exactly ONE first-order lag, TAU
%       A3  there is NO saturation anywhere -- not on acceleration, not on
%           load factor, not on commanded angle
%
%   OUR SYSTEM VIOLATES A2 AND A3, AND PARTLY A1:
%     * A3 (the fatal one).  Saturation is our DOMINANT behaviour, not a
%       correction to it.  nz = clip(cos(gam) + V*gdot/g, -1.0, 3.0) fires
%       constantly, and the nz-clamp fraction correlates with touchdown
%       error at spearman +0.538 to +0.914 depending on definition.  On top
%       of that sits a ONE-SIDED LOS floor,
%           gam_cmd = min(gam_cmd, -atan2(h,d)*1.0)
%       applied AFTER the nz clamp, which fires on 43.1% of PN samples
%       overall and on 89.8-99.7% of samples for the four failing aircraft.
%       A one-sided min() has ZERO gradient with respect to its input on
%       the active side: the loop is OPEN there.  An adjoint of an open
%       loop is meaningless.  Zarchan has no counterpart to that floor
%       anywhere -- Ch.24 (line 8109), Ch.28, Ch.30 (line 10887) and App.A
%       (line 14446) all impose terminal angle as an ADDITIVE term.
%     * A2.  Our effective lag is TAU ~= 1.50 s only while the LOS floor is
%       NOT firing; when it fires the identified lag inflates to 2.5-8.0 s.
%       That is not one lag, it is a loop that switches structure.  Also we
%       have a 1.75 s alpha lowpass, a 1.0 s PN lead and 20 Hz commands on
%       top of the airframe.
%     * A1.  Our dive is a large-angle manoeuvre against a stationary
%       target with a strong terminal-angle demand; the small-angle
%       linearisation is being stretched.
%
%   WHAT THIS FILE IS THEREFORE GOOD FOR, AND ONLY THIS:
%     1. A LOWER BOUND / floor: if even the unsaturated linear loop misses
%        at some tF/TAU, we certainly do.
%     2. TREND and SCALING: how miss grows as tF/TAU shrinks, and the
%        normalisation facts (miss ~ TAU^2 for maneuver, ~ TAU for heading
%        error).  Those survive as SHAPE arguments.
%     3. Validating other code -- it is a known-correct linear reference.
%   WHAT IT IS NOT GOOD FOR:
%     Predicting our miss.  Our tF/TAU is comfortable on the face of it
%     (batch median 16.4 apparent, 27.4 on the airframe TAU), yet the four
%     aircraft that missed by 37-46 m sit at 15.1 / 7.6 / 5.5 / 4.7 -- three
%     of them below 8.  If you read this file's pretty curve you would
%     conclude those three are simply "short flights".  The measured cause
%     is that the LOS floor was firing on ~90-100% of their PN samples, so
%     they were barely doing PN at all.  Same symptom, different mechanism.
%     The adjoint cannot tell you which, because it cannot represent the
%     second one.
%   Our own acceptance criterion is RESIDUAL ALTITUDE + TOUCHDOWN ERROR
%   (err = 1.124*residual_altitude + 1.99, R^2 = 0.906, n=30).  Nothing in
%   this file speaks to that.
%   =====================================================================
%
%   SOURCE (local full text, zarchan.txt)
%     lines 651-659  the three rules for building an adjoint:
%       RULE 1  convert all system inputs to impulses
%       RULE 2  replace t by tF - t in the arguments of all time-varying
%               coefficients
%       RULE 3  reverse all signal flow, redefining nodes as summing
%               junctions and vice versa
%     line 682   verbatim: "A simulation of the adjoint block diagram will
%       yield y as a function of flight time y(tF). This means that in an
%       adjoint simulation we obtain the miss distances due to both a step
%       target maneuver and initial heading error for various flight
%       times - all obtained in one computer run!"
%     lines 684-696  h*(tF - tI, tF - tO) = h(tO, tI)
%     line  751  verbatim result used as the acceptance test in
%       demo_reproduce: "a 3-g maneuver causes nearly 12 ft of miss if the
%       flight time is only 1 s"; and "long flights (flight time large
%       compared to guidance system time constant) ... result in virtually
%       zero miss"
%     line  756  "LISTING 3.1 SINGLE-LAG ADJOINT WITH SECOND-ORDER
%                 RUNGE-KUTTA INTEGRATION"
%     lines 757-763  the code, transcribed below
%
%   STATE MEANING (four states, Fig. 3.12)
%       X1  integrates X2         -> ArrayXMNT = XNT * X1   (maneuver miss)
%       X2  integrates X3         -> ArrayXMHE = -VM*HE*X2  (heading miss)
%       X3  the lag output path;  X3D = XNP*Y1/TGO
%       X4  the lag state;        X4D = -Y1,  Y1 = (X4 - X2)/TAU
%   TP is the ADJOINT time, and it reads as FLIGHT TIME on the output axis
%   (Rule 2).  X3(0)=1 is the impulse of Rule 1.
%
%   USAGE
%       out = adjoint_miss()                  % book nominal
%       out = adjoint_miss('XNP',5,'TF',20)
%
%   OPTIONS (defaults = the book's data statement, line 757)
%       XNT    target maneuver, ft/s^2                              [96.6]
%       XNP    effective navigation ratio N'                        [4]
%       TAU    guidance system time constant, s                     [1]
%       TF     largest flight time swept, s                         [10]
%       VM     missile speed, ft/s                                  [3000]
%       HEDEG  heading error, deg                                   [-20]
%       H      integration step, s                                  [0.01]
%       PLOTS  reproduce Figs. 3.13 / 3.14                          [false]
%
%   OUTPUT
%       .TP     flight time axis, s
%       .XMNT   miss due to the step target maneuver, ft
%       .XMHE   miss due to the initial heading error, ft
%   Because the adjoint is LINEAR, superposition applies to the outputs:
%   doubling XNT doubles XMNT (line 751).  That is exactly why A3 above is
%   fatal -- superposition is the first thing saturation destroys.
%
%   TRANSCRIPTION REPAIRS
%       "while TP ,=(TF-1e-5)" -> while TP <= (TF-1e-5)
%       "while STEP ,=1"       -> while STEP <= 1
%       "if S.=.0999"          -> if S >= .0999
%   These restorations are inference, not source.
%
%   DEVIATIONS FROM THE LISTING (all of them)
%   -----------------------------------------
%   1. Wrapped in a function with name/value options; defaults = line 757.
%   2. Preallocation, trimmed at the end.
%   Nothing else.  The RK2 corrector form the book uses here is
%   X = (XOLD + X)/2 + .5*H*XD, which differs cosmetically from Listing
%   2.1's .5*(XOLD + X + H*XD) but is algebraically identical; kept as
%   printed in Listing 3.1.

%% ---- options ------------------------------------------------------------
p = struct('XNT',96.6,'XNP',4.,'TAU',1.,'TF',10,'VM',3000.,'HEDEG',-20., ...
           'H',.01,'PLOTS',false,'SAVEDAT','');
for k = 1:2:numel(varargin)
    f = varargin{k};
    if ~isfield(p,f), error('adjoint_miss: unknown option %s',f); end
    p.(f) = varargin{k+1};
end

%% =======================================================================
%  LISTING 3.1  (line by line)
%  =======================================================================
n=0; XNT=p.XNT; XNP=p.XNP; TAU=p.TAU; TF=p.TF; VM=p.VM; HEDEG=p.HEDEG;
T=0.; S=0.; TP=T+.00001; X1=0; X2=0; X3=1; X4=0; H=p.H; HE=HEDEG/57.3;

NMAX=ceil(TF/max(H,eps)*1.1)+100;
ArrayTP=zeros(1,NMAX); ArrayXMNT=ArrayTP; ArrayXMHE=ArrayTP;
X1D=0; X2D=0; X3D=0; X4D=0;

while TP <= (TF-1e-5)
    X1OLD=X1; X2OLD=X2; X3OLD=X3; X4OLD=X4;
    STEP=1; FLAG=0;
    while STEP <= 1
        if FLAG==1
            STEP=2;
            X1=X1+H*X1D; X2=X2+H*X2D; X3=X3+H*X3D; X4=X4+H*X4D;
            TP=TP+H;
        end
        X1D=X2;
        X2D=X3;
        Y1=(X4-X2)/TAU;
        TGO=TP+.00001;
        X3D=XNP*Y1/TGO;
        X4D=-Y1;
        FLAG=1;
    end
    FLAG=0;
    X1=(X1OLD+X1)/2+.5*H*X1D;
    X2=(X2OLD+X2)/2+.5*H*X2D;
    X3=(X3OLD+X3)/2+.5*H*X3D;
    X4=(X4OLD+X4)/2+.5*H*X4D;
    S=S+H;
    if S >= .0999
        S=0.; n=n+1;
        ArrayTP(n)=TP;
        ArrayXMNT(n)=XNT*X1;
        ArrayXMHE(n)=-VM*HE*X2;
    end
end
%  ========================== end of Listing 3.1 ==========================

ArrayTP=ArrayTP(1:n); ArrayXMNT=ArrayXMNT(1:n); ArrayXMHE=ArrayXMHE(1:n);

if p.PLOTS
    figure, plot(ArrayTP,ArrayXMNT),grid
    xlabel('Flight Time (Sec)'), ylabel('Target Maneuver Miss (Ft)')
    title('Fig. 3.13 style   -- LINEAR, UNSATURATED, ONE LAG')
    figure, plot(ArrayTP,ArrayXMHE),grid
    xlabel('Flight Time (Sec)'), ylabel('Heading Error Miss (Ft)')
    title('Fig. 3.14 style   -- LINEAR, UNSATURATED, ONE LAG')
end
if ~isempty(p.SAVEDAT)
    output=[ArrayTP',ArrayXMNT',ArrayXMHE']; %#ok<NASGU>
    save(p.SAVEDAT,'output','-ascii');
end

out       = struct();
out.opts  = p;
out.TP    = ArrayTP;
out.XMNT  = ArrayXMNT;
out.XMHE  = ArrayXMHE;
end
