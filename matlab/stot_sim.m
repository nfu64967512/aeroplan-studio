function out = stot_sim(ic, c)
%STOT_SIM  One dive: 20 Hz guidance / 40 Hz telemetry / 50 Hz FC / RK4 plant.
%
%   out = stot_sim(ic, c)
%
%   ic  release state (see stot_ic32.m for how it is built from truth32.csv)
%       .tag        char
%       .r0         [m]   horizontal range to the bullseye at release
%       .z0         [m]   height above the target datum at release
%       .V0         [m/s] total speed
%       .gam0_deg   [deg] flight path angle, up positive
%       .eps0_deg   [deg] signed heading offset, >0 = velocity right of the LOS
%       .dive_T     [s]   scheduled touchdown, measured from release (t = 0)
%       .phi0_deg   [deg] optional bank at release (default 0)
%   c   cfg.params()
%
%   The aircraft is placed due north of the bullseye without loss of generality:
%   there is no wind and the tangent plane is isotropic, so only the geometry
%   (r0, z0, V0, gam0, eps0) matters, not the absolute bearing.
%
% ------------------------------- THREE CLOCKS (M6) --------------------------
%   telemetry 40 Hz  (tel_us = 25000)   -> what the node is allowed to see
%   guidance  20 Hz  (TICK_PERIOD_S)    -> _pn_cmd, reads the latest latch
%   flight ctl 50 Hz (SCHED_LOOP_RATE)  -> stot_fcs_clamp, ZOH on the command
%   plant     continuous, RK4, step <= sim.h_max, and the scheduler shortens the
%             step so that it lands exactly on every clock boundary.  sim3d.py
%             folded all of this into one 20 Hz explicit Euler step.
%
% ------------------------------- TERMINATION (the sim3d bug) ----------------
%   z = 0 is found by a secant root search on the RK4 step, and EVERY output is
%   then taken at that root -- there is one interpolation, applied to the whole
%   state vector, so "only some outputs got interpolated" is structurally
%   impossible.  sim3d.py:404 takes r_end at the end of the sub-step in which z
%   first went negative while t_go IS interpolated (line 399-401), which turns
%   its touchdown error into pure step-size residue: 0.020 -> -1.3064 m,
%   0.010 -> -0.7455, 0.005 -> -0.5546.  stot_selftest.m 'dtinv' asserts that
%   halving h here does NOT halve the touchdown error.

if nargin < 2, c = cfg.params(); end
if ~isfield(ic,'phi0_deg') || isempty(ic.phi0_deg), ic.phi0_deg = 0.0; end
% Pitch attitude at release, RELATIVE to aerodynamic trim.  truth32.csv does not
% carry it -- the node never logged ATTITUDE -- yet it drives the whole opening
% of the dive, because tot_node.py:2871 seeds its alpha low-pass with
% (pitch_meas - gam) and then decays it with tau = 1.75 s, and 2878 divides that
% seed by max(cos(roll),0.5).  A 10 deg seed at 60 deg bank therefore adds 20 deg
% to the pitch command for several seconds.  Default 0 (= trim); sweep it.
if ~isfield(ic,'th0_offset_deg') || isempty(ic.th0_offset_deg)
    ic.th0_offset_deg = 0.0;
end

pn = c.pn; fc = c.fc; b = c.b; ms = c.meas; sp = c.sim;
il = c.il;
il.tconst_p = fc.PTCH2SRV_TCONST;
il.tconst_r = fc.RLL2SRV_TCONST;
env.home_amsl = 0.0;      % the bullseye is at AMSL 0 (sea-level target datum)

% ---------------------------------------------------------------- initial --
gam0 = deg2rad(ic.gam0_deg);
eps0 = deg2rad(ic.eps0_deg);
phi0 = deg2rad(ic.phi0_deg);
n0 = ic.r0;  e0 = 0.0;                 % target at the origin, aircraft due north
los0 = atan2(-e0, -n0);                % bearing of the LOS = pi
psi0 = los0 + eps0;                    % eps > 0 = velocity clockwise of the LOS

% trim alpha so the aircraft starts neither pitching nor sinking artificially
rho0 = 1.225*(1 - 2.25577e-5*max(ic.z0,0))^4.2586;
qS0  = 0.5*rho0*ic.V0^2*b.S;
nL0  = cos(gam0)/max(cos(phi0),0.2);
CL0  = min(b.CLmax, max(b.CLmin, nL0*b.W/max(qS0,1e-6)));
alpha0 = CL0/b.CLa;
th0  = gam0 + alpha0*cos(phi0) + deg2rad(ic.th0_offset_deg);

x = [n0; e0; ic.z0; ic.V0; gam0; psi0; th0; 0.0; phi0; 0.0];

% ---------------------------------------------------------------- clocks ---
dt_gd  = pn.tick_dt;          % 0.05
dt_tel = 1/ms.tel_hz;         % 0.025
dt_fc  = 1/fc.loop_hz;        % 0.02
t = 0.0;
t_gd = 0.0; t_tel = 0.0; t_fc = 0.0;
EPS = 1e-9;

rng(ms.seed, 'twister');

gs = struct('alpha_lp',0,'has_alpha',false,'frozen',false, ...
            'roll_frz',0,'pitch_frz',0,'dive_T',ic.dive_T);

latch = latch_from(x, ms);
cmd   = struct('roll',0,'pitch',0,'thr',pn.thr_trim);
u     = [0; 0; 100*pn.thr_trim];
thr_pc_prev = NaN;
have_cmd = false;

% ---------------------------------------------------------------- logging --
nmax   = ceil((sp.t_max)/dt_fc) + 16;
% TR columns: 1 t | 2 n | 3 e | 4 z | 5 V | 6 gam | 7 psi | 8 th | 9 q
%            10 phi | 11 p | 12 nav_roll | 13 nav_pitch | 14 thr_pc
TR     = nan(nmax, 14);
ntr    = 0;
ngmax  = ceil(sp.t_max/dt_gd) + 16;
GD     = nan(ngmax, 22);
ngd    = 0;
FCF    = zeros(1,6);      % counters: fc_roll_clip fc_pitch_lo fc_pitch_hi
                          %           thr_clip thr_slew aero_nz
nfc    = 0;

td = struct('hit',false,'t',NaN,'r',NaN,'V',NaN,'gam',NaN,'gs',NaN, ...
            'psi',NaN,'th',NaN,'phi',NaN,'eas',NaN);
t_stop_at = Inf;          % set once touchdown happens: keep flying post_s more
outage_max = 0.0; outage_now = 0.0;

while t < sp.t_max
    % ---- telemetry latch (40 Hz) ------------------------------------------
    if t >= t_tel - EPS
        latch = latch_from(x, ms);
        t_tel = t_tel + dt_tel;
    end

    % ---- guidance (20 Hz) --------------------------------------------------
    if t >= t_gd - EPS
        % M7 node outage.  Measured: one ~3.6 s gap every ~35.8 s, i.e. exactly
        % one per dive, at an unknown phase.  During the gap the node emits
        % nothing and the flight controller holds the last attitude target.
        % NOTE once the gap passes GUIDED_TIMEOUT = 3.0 s the firmware stops
        % holding and hands roll to L1 (calc_nav_roll), pitch to TECS
        % (calc_nav_pitch) and throttle to TECS -- mode_guided.cpp:40,77,88.
        % That fallback is NOT modelled; out.guided_timeout_risk flags it.
        frozen_out = sp.freeze_on && ...
            (mod(t - sp.freeze_phase_s, sp.freeze_period_s) < sp.freeze_dur_s);
        if frozen_out
            outage_now = outage_now + dt_gd;      % node is not emitting
            outage_max = max(outage_max, outage_now);
        else
            outage_now = 0.0;
            [cmd, gs, dbg] = stot_guidance(latch, gs, t, pn);
            have_cmd = true;
            ngd = ngd + 1;
            GD(ngd,:) = [t, dbg.d, dbg.h, dbg.R, dbg.V, dbg.gam, ...
                         dbg.nz_raw, dbg.nz, double(dbg.nz_lo), double(dbg.nz_hi), ...
                         dbg.gam_cmd_pn, dbg.gam_los, dbg.gam_cmd, ...
                         double(dbg.los_active), double(dbg.pitch_lo), ...
                         double(dbg.pitch_hi), double(dbg.roll_node_clip), ...
                         dbg.v_req, double(dbg.thr_sat_lo), double(dbg.thr_sat_hi), ...
                         double(dbg.frozen), cmd.thr];
        end
        t_gd = t_gd + dt_gd;
    end

    % ---- flight controller (50 Hz) ----------------------------------------
    if t >= t_fc - EPS
        if have_cmd
            [nav_roll, nav_pitch, thr_pc, fl] = stot_fcs_clamp( ...
                cmd.roll, cmd.pitch, cmd.thr, x(9), x(7), thr_pc_prev, dt_fc, fc);
            thr_pc_prev = thr_pc;
            u = [nav_roll; nav_pitch; thr_pc];
            nfc = nfc + 1;
            FCF(1) = FCF(1) + fl.fc_roll_clip;
            FCF(2) = FCF(2) + fl.fc_pitch_lo;
            FCF(3) = FCF(3) + fl.fc_pitch_hi;
            FCF(4) = FCF(4) + fl.thr_clip;
            FCF(5) = FCF(5) + fl.thr_slew;
            [~, ax] = stot_plant(x, u, b, il, env);
            FCF(6) = FCF(6) + ax.aero_nz_binding;
        end
        ntr = ntr + 1;
        TR(ntr,:) = [t, x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8), ...
                     x(9), x(10), u(1), u(2), u(3)];
        t_fc = t_fc + dt_fc;
    end

    % ---- integrate to the next clock boundary ------------------------------
    hh = min([t_gd, t_tel, t_fc, t + sp.h_max, t_stop_at]) - t;
    if hh <= EPS, hh = min(sp.h_max, 1e-6); end
    xn = rk4(x, u, hh, b, il, env);

    % ---- touchdown event: ONE interpolation for the WHOLE state -----------
    if ~td.hit && x(3) > 0 && xn(3) <= 0
        hstar = solve_cross(x, u, hh, b, il, env);
        xc    = rk4(x, u, hstar, b, il, env);
        tc    = t + hstar;
        td.hit = true;
        td.t   = tc;
        td.r   = hypot(xc(1), xc(2));
        td.V   = xc(4);
        td.gam = xc(5);
        td.gs  = xc(4)*cos(xc(5));
        td.psi = xc(6); td.th = xc(7); td.phi = xc(9);
        rho_c  = 1.225*(1 - 2.25577e-5*max(xc(3)+env.home_amsl,0))^4.2586;
        td.eas = xc(4)*sqrt(rho_c/b.rho0);
        t_stop_at = tc + sp.post_s;
    end

    x = xn;  t = t + hh;

    if td.hit && (t >= t_stop_at - EPS || x(3) < sp.z_floor), break; end
    if ~td.hit && hypot(x(1),x(2)) > 20000, break; end     % escaped
    if x(4) < 5, break; end
end

TR = TR(1:ntr,:);  GD = GD(1:ngd,:);

% ---------------------------------------- closest horizontal passage -------
% Truth defines z_res as the AGL at the closest horizontal passage and d1_min as
% that passage distance, both taken on a window that runs past the crossing --
% which is why z_res can be negative.  Reproduce that by bracketing rdot = 0.
[d1_min, z_res, t_rmin] = closest_pass(TR, b, il, env, sp);

% ------------------------------------------------------------- summary -----
gdN = max(ngd,1);
% Only count guidance ticks between release and touchdown, which is the window
% fidelity_long.replay_nz uses on the measured CSVs.
if td.hit, msk = GD(:,1) <= td.t; else, msk = true(ngd,1); end
nn = max(sum(msk),1);

out = struct();
out.tag        = ic.tag;
out.hit        = td.hit;
out.t_dive     = td.t;                 % release -> AGL = 0 crossing
out.err_touch  = td.r;                 % horizontal miss AT the crossing
out.V_touch    = td.V;
out.gs_touch   = td.gs;
out.eas_touch  = td.eas;
out.gam_touch  = rad2deg(td.gam);
out.z_res      = z_res;
out.d1_min     = d1_min;
out.t_rmin     = t_rmin;
% These three are taken over release -> touchdown only, matching truth32:
% gs_peak_dive_ms excludes the expended ballistic fall, and climb_after_rel_m /
% gam_min_deg are defined on the same window.
if td.hit, wtr = TR(:,1) <= td.t; else, wtr = true(size(TR,1),1); end
out.gs_peak    = max(TR(wtr,5).*cos(TR(wtr,6)));
out.climb_after_rel = max(0, max(TR(wtr,4)) - ic.z0);
out.gam_min_deg= rad2deg(min(TR(wtr,6)));
out.roll_max_deg = rad2deg(max(abs(TR(:,10))));

out.nz_n       = nn;
out.nz_lo_frac = sum(GD(msk,9))/nn;
out.nz_hi_frac = sum(GD(msk,10))/nn;
out.nz_clip_frac = out.nz_lo_frac + out.nz_hi_frac;
out.los_frac   = sum(GD(msk,14))/nn;
out.pitch_lo_frac = sum(GD(msk,15))/nn;
out.pitch_hi_frac = sum(GD(msk,16))/nn;
out.roll_node_clip_frac = sum(GD(msk,17))/nn;
out.thr_lo_frac = sum(GD(msk,19))/nn;
out.thr_hi_frac = sum(GD(msk,20))/nn;
out.freeze_frac = sum(GD(msk,21))/nn;

out.fc_roll_clip_frac  = FCF(1)/max(nfc,1);
out.fc_pitch_lo_frac   = FCF(2)/max(nfc,1);
out.fc_pitch_hi_frac   = FCF(3)/max(nfc,1);
out.fc_thr_clip_frac   = FCF(4)/max(nfc,1);
out.fc_thr_slew_frac   = FCF(5)/max(nfc,1);
out.aero_nz_frac       = FCF(6)/max(nfc,1);
out.node_outage_max_s  = outage_max;
out.guided_timeout_risk= outage_max >= fc.GUIDED_TIMEOUT;

out.n_gd = ngd; out.n_fc = nfc;
if sp.log
    out.TR = TR;  out.GD = GD;
end
out.ic = ic;
end

% ===========================================================================
function m = latch_from(x, ms)
%LATCH_FROM  Build the telemetry the node receives, with MAVLink quantisation.
% GLOBAL_POSITION_INT: lat/lon int32 1e-7 deg, relative_alt int32 mm,
% vx/vy/vz int16 cm/s.  lam_dot and sig_dot divide by d^2 and R^2, so this
% quantisation is squared up as the range closes -- that is the mechanism M8 is
% about, and it is exactly what sim3d.py (which feeds the guidance the truth)
% cannot produce.
n = x(1); e = x(2); z = x(3); V = x(4); gam = x(5); psi = x(6);
vn = V*cos(gam)*cos(psi);
ve = V*cos(gam)*sin(psi);
vd = -V*sin(gam);
pitch = x(7);
if ms.noise_pos_m > 0
    n = n + ms.noise_pos_m*randn; e = e + ms.noise_pos_m*randn;
    z = z + ms.noise_pos_m*randn;
end
if ms.noise_vel_ms > 0
    vn = vn + ms.noise_vel_ms*randn; ve = ve + ms.noise_vel_ms*randn;
    vd = vd + ms.noise_vel_ms*randn;
end
if ms.noise_att_rad > 0
    pitch = pitch + ms.noise_att_rad*randn;
end
if ms.enable_quant
    n = qz(n, ms.q_pos_m);  e = qz(e, ms.q_pos_m);  z = qz(z, ms.q_z_m);
    vn = qz(vn, ms.q_vel_ms); ve = qz(ve, ms.q_vel_ms); vd = qz(vd, ms.q_vel_ms);
    if ms.q_att_rad > 0, pitch = qz(pitch, ms.q_att_rad); end
end
m = struct('n',n,'e',e,'z',z,'vn',vn,'ve',ve,'vd',vd,'pitch',pitch);
end

function v = qz(v, q)
if q > 0, v = round(v/q)*q; end
end

% ===========================================================================
function xn = rk4(x, u, h, b, il, env)
%RK4  Classical 4th-order step.  u is constant across the step by construction:
% the scheduler never lets a step straddle a flight-controller boundary.
k1 = stot_plant(x,             u, b, il, env);
k2 = stot_plant(x + 0.5*h*k1,  u, b, il, env);
k3 = stot_plant(x + 0.5*h*k2,  u, b, il, env);
k4 = stot_plant(x + h*k3,      u, b, il, env);
xn = x + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
end

% ===========================================================================
function hstar = solve_cross(x, u, h, b, il, env)
%SOLVE_CROSS  Sub-step time at which z hits 0, by secant on the RK4 flow.
% This is the MATLAB equivalent of odeset('Events',...): the root is found on
% the integrated solution, and the caller then evaluates the WHOLE state there.
z0 = x(3);
a = 0; fa = z0;
bb = h; fb_x = rk4(x,u,h,b,il,env); fb = fb_x(3);
hstar = a + fa*(bb-a)/(fa-fb);
for it = 1:40
    xc = rk4(x,u,hstar,b,il,env);
    fc_ = xc(3);
    if abs(fc_) < 1e-10, break; end
    if fa*fc_ < 0, bb = hstar; fb = fc_; else, a = hstar; fa = fc_; end
    den = (fa - fb);
    if abs(den) < 1e-15, hstar = 0.5*(a+bb); continue; end
    cand = a + fa*(bb-a)/den;
    if ~(cand > min(a,bb) && cand < max(a,bb)), cand = 0.5*(a+bb); end
    if abs(cand-hstar) < 1e-14, hstar = cand; break; end
    hstar = cand;
end
hstar = min(max(hstar, 0), h);
end

% ===========================================================================
function [d1_min, z_res, t_rmin] = closest_pass(TR, b, il, env, sp)
%CLOSEST_PASS  Minimum horizontal range and the height there, root-found on
% rdot = 0 rather than picked off the sample grid.
d1_min = NaN; z_res = NaN; t_rmin = NaN;
if size(TR,1) < 3, return; end
n = TR(:,2); e = TR(:,3); V = TR(:,5); gam = TR(:,6); psi = TR(:,7);
r  = hypot(n,e);
rd = (n.*(V.*cos(gam).*cos(psi)) + e.*(V.*cos(gam).*sin(psi))) ./ max(r,1e-9);
% Take the GLOBAL minimum, not the first local one.  Several release states put
% the aircraft on a heading up to 81 deg off the LOS, so r rises before it falls
% and a first-crossing search lands on the wrong stationary point (it reported
% z_res = 3409 m for RRD2-U1).
[~, imin] = min(r);
k = [];
for kk = max(imin-1,1) : min(imin, numel(rd)-1)
    if rd(kk) < 0 && rd(kk+1) >= 0, k = kk; break; end
end
if isempty(k)
    d1_min = r(imin); z_res = TR(imin,4); t_rmin = TR(imin,1);
    return;
end
% bracket [k, k+1]: one flight-controller period, so u is constant across it
x0 = TR(k,2:11).';
u0 = TR(k,12:14).';
h  = TR(k+1,1) - TR(k,1);
lo = 0; hi = h;
for it = 1:60
    mid = 0.5*(lo+hi);
    xm  = rk4(x0,u0,mid,b,il,env);
    rdm = (xm(1)*xm(4)*cos(xm(5))*cos(xm(6)) + xm(2)*xm(4)*cos(xm(5))*sin(xm(6))) ...
          / max(hypot(xm(1),xm(2)),1e-9);
    if rdm < 0, lo = mid; else, hi = mid; end
    if hi-lo < 1e-12, break; end
end
xm = rk4(x0,u0,0.5*(lo+hi),b,il,env);
d1_min = hypot(xm(1),xm(2));
z_res  = xm(3);
t_rmin = TR(k,1) + 0.5*(lo+hi);
end
