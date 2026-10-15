function [ic, detail] = icFromState(st, c, opts)
%MGS.ICFROMSTATE  A live vehicle state -> the release IC the model wants.
%
%   [ic, detail] = mgs.icFromState(st, c)
%
%   st  .n .e .z         [m]   position in the target-tangent plane
%       .vn .ve .vd      [m/s] NED velocity, vd DOWN positive
%       .pitch_deg       [deg] measured pitch attitude  (optional)
%       .roll_deg        [deg] measured bank            (optional)
%
%   ic  the struct mgs.scenario/mgs.simSwarm consume:
%       .tag .r0 .z0 .V0 .gam0_deg .eps0_deg .theta_deg .phi0_deg
%       .th0_offset_deg .dive_T .t_release
%
% ---------------------------------------------------------------------------
%   th0_offset_deg IS THE ONE THAT MATTERS AND THE ONE NOBODY EXPECTS.
%
%   The model normally starts at aerodynamic trim.  The real vehicle does not,
%   and tot_node.py:2871 seeds its angle-of-attack low-pass with
%   (measured pitch - gamma) and then decays it with tau = 1.75 s, while 2878
%   divides that seed by max(cos(roll), 0.5).  A 10 deg seed at 60 deg bank
%   adds 20 deg to the pitch command for several seconds.
%
%   So if you seed the model at trim while the vehicle was 8 deg off trim, the
%   two open the dive differently and every downstream comparison inherits it
%   -- and it looks like a plant error rather than an initial-condition error.
%   This function computes the offset from the live attitude when it is
%   available, and SAYS SO in detail.th0_source when it is not.
%
%   See also MGS.SHADOWDIFF, MGS.SCENARIO.

arguments
    st
    c = mgs.params()
    opts.tag (1,:) char = 'LIVE'
end

n0 = st.n; e0 = st.e; z0 = st.z;
vn = st.vn; ve = st.ve; vd = st.vd;

vh = hypot(vn, ve);
V0 = hypot(vh, vd);
gam0 = atan2(-vd, max(vh, 1e-3));
psi0 = atan2(ve, vn);
los0 = atan2(-e0, -n0);
eps0 = wrapToPi(psi0 - los0);

phi0_deg = 0.0;
if isfield(st, 'roll_deg') && isfinite(st.roll_deg), phi0_deg = st.roll_deg; end

ic = struct();
ic.tag       = opts.tag;
ic.r0        = hypot(n0, e0);
ic.z0        = z0;
ic.V0        = V0;
ic.gam0_deg  = rad2deg(gam0);
ic.eps0_deg  = rad2deg(eps0);
ic.theta_deg = rad2deg(atan2(e0, n0));      % bearing of the vehicle FROM target
ic.phi0_deg  = phi0_deg;
ic.dive_T    = NaN;
ic.t_release = 0.0;

% ---- pitch offset relative to trim ----------------------------------------
th_trim = trim_pitch(ic, c);
detail = struct('th_trim_deg', rad2deg(th_trim));
if isfield(st, 'pitch_deg') && isfinite(st.pitch_deg)
    ic.th0_offset_deg = st.pitch_deg - rad2deg(th_trim);
    detail.th0_source = 'live ATTITUDE';
    detail.pitch_live_deg = st.pitch_deg;
else
    ic.th0_offset_deg = 0.0;
    detail.th0_source = 'NOT AVAILABLE -- started at trim';
    detail.pitch_live_deg = NaN;
    warning('mgs:icFromState:nopitch', ...
        ['no live pitch attitude, so the model starts at aerodynamic trim. ' ...
         'tot_node seeds its alpha low-pass from measured pitch and decays ' ...
         'it over 1.75 s, so a real offset here shows up as a plant error ' ...
         'in any comparison that follows.']);
end
detail.eps0_deg = ic.eps0_deg;
detail.V0 = V0;
end

% -----------------------------------------------------------------------------
function th0 = trim_pitch(ic, c)
%TRIM_PITCH  The same trim mgs.simSwarm uses, so the offset is measured
%   against the model's own starting point and not a different one.
gam0 = deg2rad(ic.gam0_deg);
phi0 = deg2rad(ic.phi0_deg);
rho0 = 1.225*(1 - 2.25577e-5*max(ic.z0, 0))^4.2586;
qS0  = 0.5*rho0*ic.V0^2*c.b.S;
nL0  = cos(gam0)/max(cos(phi0), 0.2);
CL0  = min(c.b.CLmax, max(c.b.CLmin, nL0*c.b.W/max(qS0, 1e-6)));
th0  = gam0 + (CL0/c.b.CLa)*cos(phi0);
end
