function [meas, tk] = seekerTracker(x, tgt, tk, c)
%MGS.SEEKERTRACKER  Seeker/Tracker block -- the measurement the guidance sees.
%
%   [meas, tk] = mgs.seekerTracker(x, tgt, tk, c)
%   tk = mgs.seekerTracker()            seed a tracker state (FIRST output)
%
%   x     plant state vector (see stot_plant): [n e z V gam psi th q phi p]
%   tgt   target in the same tangent plane, struct .n .e .z (usually 0,0,0)
%   tk    tracker state, carried between calls.
%   c     mgs.params()
%
%   meas  what the guidance law is allowed to read this tick:
%         .n .e .z .vn .ve .vd .pitch    the tot_node telemetry latch, with
%                                        MAVLink quantisation applied
%         .lam_dot .sig_dot              sightline rates, lateral and vertical
%         .Vc                            closing velocity
%         .src                           'analytic' | 'tracker'
%
% ---------------------------------------------------------------------------
%   THIS BLOCK IS WHERE WE DIFFER FROM THE MATHWORKS EXAMPLE, ON PURPOSE.
%
%   MathWorks "Designing a Guidance System in MATLAB and Simulink" gives the
%   Seeker/Tracker a real tracking loop: a 0.05 s time constant, an estimator
%   whose bandwidth is half the autopilot's, and it ESTIMATES the inertial
%   sightline rate because a real seeker measures a gimbal angle, not a state.
%
%   tot_node.py has no seeker.  It reads GLOBAL_POSITION_INT and computes
%
%       lam_dot = (de*vn - dn*ve) / d^2                  tot_node.py:2824
%
%   analytically from the latch.  No lag, no noise, no gimbal.  So:
%
%     c.trk.mode = 'analytic'   reproduces the flight.  DEFAULT.
%     c.trk.mode = 'tracker'    the MathWorks structure, for the question
%                               "what would a real seeker cost us?"
%
%   Do not read 'tracker' results as predictions of the current vehicle.  It
%   models hardware we do not have.
%
%   MEASUREMENT QUANTISATION is applied in both modes, because that part IS
%   real: lat/lon are int32 1e-7 deg (1.11 cm), relative_alt is int32 mm, and
%   velocities are int16 cm/s.
%
%   See also STOT_GUIDANCE, MGS.GUIDANCE.

if nargin == 0 || (nargin == 1 && isempty(x))
    % Seeding convention (see mgs.guidance): no arguments -> the FIRST output
    % is the seed state, so `tk = mgs.seekerTracker()` is right.
    meas = seed(); tk = meas; return;
end
if isempty(tk), tk = seed(); end

n = x(1); e = x(2); z = x(3); V = x(4); gam = x(5); psi = x(6); th = x(7);

% ---- the telemetry latch, quantised exactly as MAVLink quantises it --------
ms = c.meas;
vn = V*cos(gam)*cos(psi);
ve = V*cos(gam)*sin(psi);
vd = -V*sin(gam);

meas = struct();
meas.n     = q(n - tgt.n, ms.q_pos_m,  ms.enable_quant);
meas.e     = q(e - tgt.e, ms.q_pos_m,  ms.enable_quant);
meas.z     = q(z - tgt.z, ms.q_z_m,    ms.enable_quant);
meas.vn    = q(vn,        ms.q_vel_ms, ms.enable_quant);
meas.ve    = q(ve,        ms.q_vel_ms, ms.enable_quant);
meas.vd    = q(vd,        ms.q_vel_ms, ms.enable_quant);
meas.pitch = q(th,        ms.q_att_rad, ms.enable_quant);

% ---- geometry, from the quantised latch (not from truth) ------------------
dn = -meas.n;  de = -meas.e;
d  = max(1e-3, hypot(dn, de));
h  = meas.z;
R  = max(1e-3, hypot(d, h));
vh = hypot(meas.vn, meas.ve);

lam_analytic = (de*meas.vn - dn*meas.ve) / (d*d);        % tot_node.py:2824
sig_analytic = (h*vh - d*(-meas.vd)) / (R*R);            % vertical sightline rate
Vc_analytic  = -(dn*meas.vn + de*meas.ve + h*(-meas.vd)) / R;

switch lower(c.trk.mode)
    case 'analytic'
        meas.lam_dot = lam_analytic;
        meas.sig_dot = sig_analytic;
        meas.Vc      = Vc_analytic;
        meas.src     = 'analytic';

    case 'tracker'
        % MathWorks structure: the tracker holds an estimate of the sightline
        % ANGLE and drives it towards the measurement with time constant tau;
        % the rate estimate is the loop's own driving term, which is what makes
        % it lag.  Noise enters on the angle, where a seeker's noise lives.
        lam = atan2(de, dn);
        sig = atan2(h, d);
        if c.trk.sigma_noise > 0
            lam = lam + c.trk.sigma_noise*randn();
            sig = sig + c.trk.sigma_noise*randn();
        end
        if ~tk.init
            tk.lam = lam;  tk.sig = sig;  tk.init = true;
        end
        tau = max(c.trk.tau, 1e-6);
        lam_dot = wrapToPi(lam - tk.lam) / tau;
        sig_dot = (sig - tk.sig) / tau;
        dt = 1/max(c.trk.hz, 1);
        a = min(1, dt/tau);
        tk.lam = tk.lam + a*wrapToPi(lam - tk.lam);
        tk.sig = tk.sig + a*(sig - tk.sig);
        meas.lam_dot = lam_dot;
        meas.sig_dot = sig_dot;
        meas.Vc      = Vc_analytic + c.trk.vc_noise*randn();
        meas.src     = 'tracker';

    otherwise
        error('mgs:seekerTracker:mode', ...
            ['unknown c.trk.mode "%s".  Allowed: analytic | tracker.  ' ...
             'Not defaulting to analytic: a typo would then silently give ' ...
             'you the flight model when you asked for a seeker.'], c.trk.mode);
end

meas.d = d;  meas.h = h;  meas.R = R;  meas.vh = vh;
end

% -----------------------------------------------------------------------------
function tk = seed()
tk = struct('init', false, 'lam', 0.0, 'sig', 0.0);
end

function v = q(v, step, on)
if ~on || step <= 0, return; end
v = round(v/step)*step;
end
