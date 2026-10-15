function out = closedform(varargin)
%CLOSEDFORM  Zarchan Ch. 2 closed-form solutions (+ the Ch. 3 lag-miss set).
%
%   CH. 2 -- NO GUIDANCE-SYSTEM LAG.  Linearised, small-angle, no saturation.
%
%   (a) heading error only              (zarchan.txt lines 533-539)
%         ydotdot = N'*Vc*lamdot ,  y(0)=0 ,  ydot(0) = -VM*HE
%         nc(t) = -VM*HE*N'/tF * (1 - t/tF)^(N'-2)
%         miss  = y(tF) = 0
%       line 539 verbatim: "the closed-form solution for the miss distance
%       y(tF) is zero. In other words, as long as the missile has sufficient
%       acceleration capability, there is no miss due to heading error!"
%
%   (b) target maneuver only            (lines 549-559)
%         nc(t) = [N'/(N'-2)] * [1 - (1 - t/tF)^(N'-2)] * nT
%         miss  = 0
%       line 559 verbatim: "the closed-form solution indicates that the miss
%       distance due to target maneuver is exactly zero!"
%       lines 555-557: at N'=2 the division by zero is removed by
%       L'Hopital's rule, giving
%         nc(t) = -2*ln((tF - t)/tF)*nT
%       line 558-559: setting N'=2.01 or 1.99 in the general formula gives
%       "approximately the same solution", so no special case is needed in
%       code.  Both are returned here so that can be checked.
%
%   (c) zero-effort miss form           (lines 578-583)
%         lamdot = ZEM_PLOS/(RTM*tgo) ,  nc = N'*ZEM_PLOS/tgo^2
%
%   SHAPES (this is the part that matters for sizing)
%     heading error:  nc(0) proportional to HE and to VM, inversely to tF;
%                     MONOTONICALLY DECREASING, zero at tF.
%                     => guard against saturation AT THE START.
%     target maneuver: independent of tF and of VM; depends only on nT and
%                     N'.  MONOTONICALLY INCREASING.
%                     => guard against saturation AT THE END.
%     Larger N' pulls MORE acceleration at t=0 and decays FASTER -- that is
%     the analytic content of Fig. 2.2 (line 431) and Fig. 2.9 (line 541).
%
%   CH. 3 -- WITH A SINGLE FIRST-ORDER LAG T.  (lines 883-921)
%     1 - H(s) = [s/(s+1/T)]^N'
%     target maneuver:   Miss/nT
%       N'=3:  0.5*tF^2*exp(-tF/T)
%       N'=4:  tF^2*exp(-tF/T)*(0.5 - tF/(6T))
%       N'=5:  tF^2*exp(-tF/T)*(0.5 - tF/(3T) + tF^2/(24T^2))
%     heading error:     Miss/(VM*HE)
%       N'=3:  tF*exp(-tF/T)*(1 - tF/(2T))
%       N'=4:  tF*exp(-tF/T)*(1 - tF/T + tF^2/(6T^2))
%       N'=5:  tF*exp(-tF/T)*(1 - 1.5*tF/T + tF^2/(2T^2) - tF^3/(24T^3))
%
%   ===================================================================
%   ERRATUM #2 -- N'=5 HEADING ERROR, LAST TERM SIGN
%   ===================================================================
%   The local full text at lines 919-921 prints that last term as
%   "+tF^3/24T^3".  It should be MINUS.  Two independent confirmations:
%     (a) partial-fraction expansion of the book's own 1-H(s) (line 830)
%         for s^3/(s+a)^5 gives t*exp(-a t)*[1 - 1.5at + 0.5(at)^2
%         - (at)^3/24];
%     (b) numerically against the verbatim Listing 3.1 adjoint (XNP=5):
%         minus-sign version max deviation ~3.1e-3 (integration-error
%         scale), plus-sign version ~3.9e-1 -- two orders apart.
%   demo_reproduce re-runs check (b) every time.  The MINUS version is the
%   default; SIGN5='text' selects the printed (wrong) one for study.
%   NOTE: unlike the Ch. 30 BIASDEG erratum, we have no PDF page image for
%   this one, so we cannot say whether it is an OCR fault or a typesetting
%   fault in the book.  The formula itself is confirmed either way.
%
%   NORMALISATION (lines 925-926, verbatim)
%     target maneuver, Miss/(nT*T^2) vs tF/T:
%       "doubling the guidance system time constant quadruples the miss!"
%     heading error,   Miss/(VM*HE*T) vs tF/T:
%       "doubling the guidance system time constant only doubles the
%        heading error miss."
%
%   USAGE
%       out = closedform()                       % book nominal
%       out = closedform('XNP',5,'TF',10)
%
%   OPTIONS
%       TF     flight time, s                                       [10]
%       XNP    effective navigation ratio for the nc(t) profiles     [4]
%       VM     missile speed, ft/s                                  [3000]
%       HEDEG  heading error, deg                                   [-20]
%       XNT    target maneuver, ft/s^2                              [96.6]
%       TAU    guidance time constant T for the Ch. 3 block, s       [1]
%       NPTS   samples in t and in tF/T                             [1001]
%       XMAX   largest tF/T evaluated in the Ch. 3 block            [12]
%       SIGN5  'listing' (minus, correct) | 'text' (plus)     ['listing']
%
%   OUTPUT
%       .t, .nc_he, .nc_nt            Ch. 2 profiles, ft/s^2
%       .nc_nt_lhopital               the N'=2 limit evaluated at XNP=2
%       .miss_he, .miss_nt            both exactly 0 (the book's result)
%       .x                            tF/T grid
%       .miss_nt_norm(k,:)            Miss/(nT*T^2),   k = 1,2,3 -> N'=3,4,5
%       .miss_he_norm(k,:)            Miss/(VM*HE*T),  k = 1,2,3 -> N'=3,4,5
%       .missfun_nt, .missfun_he      function handles f(tF,T,NP) in ft
%
%   APPLICABILITY -- THE WHOLE FILE
%   -------------------------------
%   Everything here assumes: LINEARISED geometry (small angles about a
%   collision triangle), CONSTANT closing velocity, NO acceleration limit,
%   and -- for the Ch. 2 part -- NO lag at all.  The headline results
%   ("miss is exactly zero") are statements about an unsaturated linear
%   loop.  Our vehicle saturates: nz-clamp fraction correlates with
%   touchdown error at spearman +0.538 to +0.914 across definitions.  So
%   these formulas describe the regime we are NOT in.  Use them to size and
%   to reason about SHAPE (where in the flight the demand peaks), never to
%   predict our miss.

%% ---- options ------------------------------------------------------------
p = struct('TF',10,'XNP',4,'VM',3000,'HEDEG',-20,'XNT',96.6, ...
           'TAU',1,'NPTS',1001,'XMAX',12,'SIGN5','listing');
for k = 1:2:numel(varargin)
    f = varargin{k};
    if ~isfield(p,f), error('closedform: unknown option %s',f); end
    p.(f) = varargin{k+1};
end
HE = p.HEDEG/57.3;
tF = p.TF;  NP = p.XNP;

%% ---- Ch. 2 : acceleration profiles --------------------------------------
t = linspace(0,tF,p.NPTS);
r = 1 - t/tF;                       % (1 - t/tF), goes 1 -> 0

% (a) heading error.  Sign follows the book: ydot(0) = -VM*HE (line 536).
nc_he = -p.VM*HE*NP/tF * r.^(NP-2);

% (b) target maneuver.  General formula; NP=2 is singular (see below).
if abs(NP-2) > 1e-12
    nc_nt = (NP/(NP-2)) * (1 - r.^(NP-2)) * p.XNT;
else
    nc_nt = nan(size(t));
end
% the L'Hopital limit at N'=2 (lines 555-557), evaluated on the same grid.
% -2*ln((tF-t)/tF)*nT ; the last sample is +Inf by construction.
nc_nt_lhopital = -2*log(max(r,eps)).*p.XNT;

%% ---- Ch. 3 : miss with a single lag -------------------------------------
x = linspace(0,p.XMAX,p.NPTS);      % x = tF/T

useplus5 = strcmpi(p.SIGN5,'text');

% Miss/(nT*T^2) as a function of x = tF/T
mnt = zeros(3,numel(x));
mnt(1,:) = 0.5*x.^2 .*exp(-x);                                  % N'=3
mnt(2,:) =     x.^2 .*exp(-x).*(0.5 - x/6);                     % N'=4
mnt(3,:) =     x.^2 .*exp(-x).*(0.5 - x/3 + x.^2/24);           % N'=5

% Miss/(VM*HE*T) as a function of x = tF/T
mhe = zeros(3,numel(x));
mhe(1,:) = x.*exp(-x).*(1 - x/2);                               % N'=3
mhe(2,:) = x.*exp(-x).*(1 - x + x.^2/6);                        % N'=4
if useplus5
    mhe(3,:) = x.*exp(-x).*(1 - 1.5*x + x.^2/2 + x.^3/24);      % printed
else
    mhe(3,:) = x.*exp(-x).*(1 - 1.5*x + x.^2/2 - x.^3/24);      % correct
end

% dimensional handles, ft.  NPq must be 3, 4 or 5.
missfun_nt = @(tf,Tc,NPq) local_pick(NPq, ...
        0.5*(tf/Tc)^2*exp(-tf/Tc), ...
            (tf/Tc)^2*exp(-tf/Tc)*(0.5 - (tf/Tc)/6), ...
            (tf/Tc)^2*exp(-tf/Tc)*(0.5 - (tf/Tc)/3 + (tf/Tc)^2/24)) * Tc^2;
if useplus5
    h5 = @(z) z*exp(-z)*(1 - 1.5*z + z^2/2 + z^3/24);
else
    h5 = @(z) z*exp(-z)*(1 - 1.5*z + z^2/2 - z^3/24);
end
missfun_he = @(tf,Tc,NPq) local_pick(NPq, ...
        (tf/Tc)*exp(-tf/Tc)*(1 - (tf/Tc)/2), ...
        (tf/Tc)*exp(-tf/Tc)*(1 - (tf/Tc) + (tf/Tc)^2/6), ...
        h5(tf/Tc)) * Tc;

%% ---- output -------------------------------------------------------------
out = struct();
out.opts            = p;
out.t               = t;
out.nc_he           = nc_he;
out.nc_nt           = nc_nt;
out.nc_nt_lhopital  = nc_nt_lhopital;
out.miss_he         = 0;      % line 539
out.miss_nt         = 0;      % line 559
out.x               = x;
out.miss_nt_norm    = mnt;
out.miss_he_norm    = mhe;
out.missfun_nt      = missfun_nt;
out.missfun_he      = missfun_he;
out.NP_index        = [3 4 5];
end

% -------------------------------------------------------------------------
function v = local_pick(NPq,a,b,c)
switch NPq
    case 3, v = a;
    case 4, v = b;
    case 5, v = c;
    otherwise
        error('closedform: the book only gives lag-miss formulas for N''=3,4,5');
end
end
