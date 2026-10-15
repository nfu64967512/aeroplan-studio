function [dx, aux] = stot_plant(x, u, b, il, env)
%STOT_PLANT  Continuous 3-DOF point-mass airframe + ArduPilot inner loop.
%
%   [dx, aux] = stot_plant(x, u, b, il, env)
%
%   x(1) n      [m]     north, target-tangent plane, bullseye at the origin
%   x(2) e      [m]     east
%   x(3) z      [m]     height above the target datum (sea target => AGL = AMSL)
%   x(4) V      [m/s]   total speed (no wind, so TAS = groundspeed magnitude)
%   x(5) gam    [rad]   flight path angle, UP POSITIVE (descent is negative)
%   x(6) psi    [rad]   heading of the horizontal velocity
%   x(7) th     [rad]   pitch attitude
%   x(8) q      [rad/s] pitch rate
%   x(9) phi    [rad]   bank attitude
%   x(10) p     [rad/s] roll rate
%
%   u(1) nav_roll [rad], u(2) nav_pitch [rad], u(3) thr_pc [%]  -- all ZOH from
%   the flight controller, i.e. already through stot_fcs_clamp.
%
%   env.home_amsl [m]  offset from the target datum to AMSL, for ISA density.
%                      The bullseye is at sea level, so this is 0.
%
% ---------------------------------------------------------------------------
%   WHY THIS IS NOT sim3d.py's PLANT
%
%   M1  alpha exists.  sim3d.py:289 says outright that a point mass has no alpha
%       and uses gamma as a proxy for pitch.  Here pitch is a state, alpha is
%       th - gam, and lift comes from alpha -- so pitch and gamma are different
%       quantities and the firmware clamp lands where it really lands.
%
%   M5  the achievable normal load is an AERODYNAMIC property, not the guidance
%       clamp.  sim3d.py:343 re-clamps the ACHIEVED nz to [nz_min, 3.0], i.e. it
%       uses the guidance law's limits as the airframe's capability.  Here the
%       cap is CLmax*q*S/(m*g) and it moves with density and V^2.  This is the
%       single reason sim3d had no failure mode: an aircraft that can always
%       pull is an aircraft that always hits.
%
%   Drag and thrust are the CALIBRATED predictor.py forms (CD0 = 0.01500 from
%   n = 1617 samples).  The one difference is that CL comes from alpha here
%   rather than from the quasi-equilibrium assumption CL = W cos(gam)/(qS) --
%   the two agree in trim, and they must differ during a manoeuvre or the model
%   could not see manoeuvre-induced drag at all.
% ---------------------------------------------------------------------------

n = x(1); e = x(2); z = x(3); V = x(4); gam = x(5); psi = x(6); %#ok<ASGLU>
th = x(7); q = x(8); phi = x(9); p = x(10);

nav_roll = u(1); nav_pitch = u(2); thr_pc = u(3);

V  = max(V, 1.0);          % keep the kinematics finite; logged via aux.v_floor
g  = b.g;

% ---- atmosphere: predictor.py:52-54, h must be AMSL ------------------------
h_amsl = z + env.home_amsl;
rho    = 1.225 * (1.0 - 2.25577e-5*max(h_amsl,0.0))^4.2586;
qbar   = 0.5*rho*V*V;
qS     = qbar*b.S;

% ---- angle of attack -------------------------------------------------------
% th = gam + alpha_body*cos(phi)  (the AoA leaves the vertical plane when the
% aircraft is banked), inverted for alpha_body.  cos(phi) is floored so an
% 80 deg bank does not blow the inverse up.
cphi = cos(phi);
alpha_b = (th - gam) / max(abs(cphi), 0.2) * sign_nz(cphi);
alpha_stop = false;
if abs(alpha_b) > b.alpha_max_rad
    alpha_b = sign(alpha_b)*b.alpha_max_rad;
    alpha_stop = true;
end

% ---- lift, and the AERODYNAMIC normal-load ceiling (M5) --------------------
CL_lin = b.CLa * alpha_b;
CL     = min(b.CLmax, max(b.CLmin, CL_lin));
cl_sat = (CL_lin > b.CLmax) || (CL_lin < b.CLmin);
L      = qS*CL;
nL     = L / b.W;                       % load factor along the lift axis
nz_max_aero = b.CLmax*qS/b.W;           % what the airframe can actually pull

% ---- drag: predictor.py:57-61, same CD0 and same induced term -------------
CD = b.CD0 + CL*CL/b.piEAR;
D  = qS*CD;

% ---- thrust: predictor.py:64-73 -------------------------------------------
% The throttle law itself lives in the guidance; by the time it gets here it has
% already been clamped to [THR_MIN, THR_MAX] and slew limited by the FC.
T = (thr_pc/100) * b.Tfull;

% ---- point-mass equations of motion ---------------------------------------
dV   = (T - D)/b.m - g*sin(gam);
dgam = (g/V) * (nL*cphi - cos(gam));
cg   = cos(gam);
if abs(cg) < 0.05, cg = sign_nz(cg)*0.05; end     % vertical-flight singularity
dpsi = (g/(V*cg)) * nL*sin(phi);

dn = V*cos(gam)*cos(psi);
de = V*cos(gam)*sin(psi);
dz = V*sin(gam);

% ---- ArduPilot inner loop --------------------------------------------------
% Angle P from the firmware default (rate demand = angle error / TCONST, and
% PTCH2SRV_RMAX = 0 so there is no rate cap in the firmware); everything below
% that -- rate PID, servo, short period -- is lumped into one first-order lag.
q_dem = (nav_pitch - th) / il.tconst_p;
p_dem = (nav_roll  - phi) / il.tconst_r;
q_dem = min(max(q_dem, -il.q_max), il.q_max);
p_dem = min(max(p_dem, -il.p_max), il.p_max);
dth = q;
dq  = (q_dem - q) / il.tau_q;
dphi = p;
dp   = (p_dem - p) / il.tau_p;

dx = [dn; de; dz; dV; dgam; dpsi; dth; dq; dphi; dp];

if nargout > 1
    aux = struct('rho',rho,'qbar',qbar,'alpha_b',alpha_b,'CL',CL, ...
                 'CL_lin',CL_lin,'cl_sat',cl_sat,'alpha_stop',alpha_stop, ...
                 'L',L,'D',D,'T',T,'nL',nL, ...
                 'nz_body',nL*cphi, ...             % vertical-plane component
                 'nz_max_aero',nz_max_aero, ...
                 'aero_nz_binding', cl_sat, ...
                 'eas', V*sqrt(rho/b.rho0), ...     % M13 bookkeeping
                 'gs',  V*cos(gam), ...
                 'q_dem',q_dem,'p_dem',p_dem);
end
end

function s = sign_nz(v)
%SIGN_NZ  sign() but never zero, so a division by it stays finite.
if v < 0, s = -1; else, s = 1; end
end
