function out = simSwarm(scn, c, opts)
%MGS.SIMSWARM  Run N vehicles against one target on a shared clock.
%
%   out = mgs.simSwarm(scn, c)
%   out = mgs.simSwarm(scn, c, 'hook', @myfun, 'realtime', true)
%
%   scn   mgs.scenario(...)
%   c     mgs.params()
%
%   out.t         [1 x K]  world clock samples at the guidance rate
%   out.veh(i)    per-vehicle result
%       .tag .hit .t_impact .err_m .z_res_m .v_impact .gs_impact
%       .traj    table of the logged trajectory
%       .flags   fraction of ticks each clamp was binding
%   out.kpi       mgs.kpi(out) -- spread, hit rate, CEP, terminal speed
%   out.calibrated  false if ANY uncalibrated leg was exercised
%   out.notes     cellstr, every reason calibrated went false
%
% ---------------------------------------------------------------------------
%   FOUR CLOCKS, exactly as in stot_sim, because they are not decoration:
%       telemetry  40 Hz  (tel_us = 25000)  what a node is allowed to see
%       guidance   20 Hz  (TICK_PERIOD_S)   _pn_cmd, reads the latest latch
%       flight ctl 50 Hz  (SCHED_LOOP_RATE) the firmware clamp chain, ZOH
%       plant      continuous, RK4, sub-stepped onto every clock boundary
%
%   Folding these into one rate is what a previous simulator here did, and it
%   turned the touchdown error into pure step-size residue.
%
%   TOUCHDOWN is found by a secant root search on the RK4 step, and the WHOLE
%   state vector is interpolated to that root -- so "some outputs got
%   interpolated and some did not" is structurally impossible.
%
% ---------------------------------------------------------------------------
%   IMPACT SPEED.  out.veh.v_impact is the TOTAL speed and out.veh.gs_impact
%   is the ground speed.  Both are reported because the field pipeline once
%   silently degraded the total to the ground speed in 12 of 16 sorties (a
%   dt = 0 made vz read as zero), turning 157-215 m/s into 140-189 m/s.  Two
%   named fields cost nothing and make that substitution impossible here.
%
%   See also MGS.SCENARIO, MGS.KPI, MGS.BRIDGE.

arguments
    scn
    c
    opts.hook = []            % f(t, snapshot) at the guidance rate; the bridge
    opts.realtime (1,1) logical = false
    opts.verbose (1,1) logical = true
    opts.t_max (1,1) double = NaN
end

n = scn.n;
sp = c.sim;
t_max = sp.t_max; if ~isnan(opts.t_max), t_max = opts.t_max; end

dt_gd = c.pn.tick_dt;                 % 20 Hz guidance
dt_tel = 1/c.meas.tel_hz;             % 40 Hz telemetry latch
dt_fc  = 1/c.fc.loop_hz;              % 50 Hz flight controller

% ---- honesty bookkeeping ---------------------------------------------------
notes = {};
held = false;
if isfield(scn, 'notes'), notes = scn.notes; end
calibrated = true;
if isfield(scn, 'calibrated'), calibrated = scn.calibrated; end
if c.gam.deficit_on
    calibrated = false;
    notes{end+1} = ['gamma tracking deficit is ON and UNCALIBRATED: this is ' ...
                    'a pessimistic bound, not a prediction (see mgs.params)'];
end
if ~strcmp(c.trk.mode, 'analytic')
    calibrated = false;
end
% ⚠️⚠️ 2026-08-22:這兩個閘門先前**被承諾但沒有實作** —— 兩份檔頭都白紙黑字
%    寫著 simSwarm 會把它們標成 uncalibrated,而 simSwarm 只看 gam.deficit_on
%    與 trk.mode。實測後果:跑 zarchan_additive(從沒飛過的律)回報
%    calibrated = 1。這正是「宣告有守衛、實際沒有」那一類 —— 而且是出現在
%    **用來防止那一類的機制自己身上**,與 pn_n_lat 契約列那次同型。
if isfield(c.pn, 'law') && ~isempty(c.pn.law) && ~strcmpi(char(c.pn.law), 'node')
    calibrated = false;
    notes{end+1} = sprintf(['guidance law is "%s", NOT the flown law: none of the ' ...
        '32-flight validation transfers.  This is a hypothesis, not a prediction ' ...
        '(mgs.zarchan.additiveLaw header).'], char(c.pn.law));
end

% ---- vehicles --------------------------------------------------------------
V = init_vehicles(scn, c);

% ---- log buffers -----------------------------------------------------------
maxK = ceil(t_max/dt_gd) + 8;
tlog = nan(1, maxK);
for i = 1:n
    V(i).log = nan(maxK, 19);
    V(i).k   = 0;
end

% ---- clocks ----------------------------------------------------------------
% ACCUMULATORS, not mod() edge tests, and the same EPS as stot_sim.  A modular
% edge test drifts against floating-point t over ~800 steps and silently
% double-fires or skips a tick; matching stot_sim here is what makes the
% n = 1 parity check in mgs.selftest hold to 1e-12 instead of to "close".
t = 0.0;  K = 0;
t_gd = 0.0; t_tel = 0.0; t_fc = 0.0;
EPS = 1e-9;
if opts.realtime, wallT0 = tic; end

% =============================== main loop ===================================
while t < t_max
    % ---- telemetry latch -------------------------------------------------
    if t >= t_tel - EPS
        for i = 1:n
            if V(i).done, continue; end
            [V(i).meas, V(i).tk] = mgs.seekerTracker(V(i).x, scn.tgt, V(i).tk, c);
        end
        t_tel = t_tel + dt_tel;
    end

    % ---- guidance --------------------------------------------------------
    if t >= t_gd - EPS
        K = K + 1;  tlog(K) = t;
        for i = 1:n
            if V(i).done, continue; end
            if t + 1e-9 < V(i).t_release
                % staggered release: the vehicle is upstream, not on PN yet
                % ⚠️ 'hold' 是**沒有任何真值資料**的一段:真值集只從放行之後
                %    開始觀測。mgs.guidance 的檔頭承諾一踏進去就標 uncalibrated。
                held = true;
                V(i).gs.phase = 'hold';
            elseif strcmp(V(i).gs.phase, 'hold')
                V(i).gs.phase = 'dive';
                V(i).t_pn = t;
            end
            [V(i).cmd, V(i).gs, V(i).dbg] = ...
                mgs.guidance(V(i).meas, V(i).gs, t, c);
            V(i).have_cmd = true;
            V(i).k = V(i).k + 1;
            V(i).log(V(i).k, :) = log_row(t, V(i));
        end
        if ~isempty(opts.hook)
            opts.hook(t, snapshot(t, V, scn, c));
        end
        if opts.realtime
            pace(wallT0, t);
        end
        t_gd = t_gd + dt_gd;
    end

    % ---- flight controller ------------------------------------------------
    if t >= t_fc - EPS
        for i = 1:n
            if V(i).done || ~V(i).have_cmd, continue; end
            [V(i).u, V(i).ap, V(i).fl] = ...
                mgs.autopilot(V(i).cmd, V(i).x, V(i).ap, dt_fc, c);
        end
        t_fc = t_fc + dt_fc;
    end

    % ---- integrate to the next clock boundary ------------------------------
    h = min([t_gd, t_tel, t_fc, t + sp.h_max]) - t;
    if h <= EPS, h = min(sp.h_max, 1e-6); end

    % ---- plant ------------------------------------------------------------
    for i = 1:n
        if V(i).done, continue; end
        xn = rk4(V(i).x, V(i).u, h, c);
        if xn(3) <= 0 && V(i).x(3) > 0
            hs = solve_cross(V(i).x, V(i).u, h, c);
            xc = rk4(V(i).x, V(i).u, hs, c);
            V(i) = finish(V(i), xc, t + hs, 'impact');
            continue;
        end
        V(i).x = xn;
        if V(i).x(3) < sp.z_floor
            V(i) = finish(V(i), V(i).x, t + h, 'floor');
        end
    end

    t = t + h;
    if all([V.done]), break; end
end

% ---- anyone still flying at t_max is a MISS, and says so -------------------
for i = 1:n
    if ~V(i).done
        V(i) = finish(V(i), V(i).x, t, 'timeout');
    end
end

% ---- assemble --------------------------------------------------------------
out = struct();
out.t   = tlog(1:K);
out.tag = scn.tag;
out.c   = c;
out.scn = scn;
out.veh = pack_vehicles(V, c);
if held
    calibrated = false;
    notes{end+1} = ['at least one vehicle flew a "hold" leg, and hold is ' ...
        'UNCALIBRATED: the truth set only ever observed the vehicle from ' ...
        'release onwards (mgs.guidance header).'];
end
out.calibrated = calibrated;
out.notes = notes;
out.kpi = mgs.kpi(out);

if opts.verbose
    fprintf('%s\n', mgs.kpi(out, 'render', true));
    if ~out.calibrated
        fprintf('  *** UNCALIBRATED RUN ***\n');
        for k = 1:numel(out.notes)
            fprintf('      - %s\n', out.notes{k});
        end
    end
end
end

% =============================================================================
function V = init_vehicles(scn, c)
n = scn.n;
V = struct('tag', {}, 'x', {}, 'gs', {}, 'ap', {}, 'tk', {}, 'meas', {}, ...
           'cmd', {}, 'dbg', {}, 'u', {}, 'fl', {}, 'done', {}, 'have_cmd', {}, ...
           't_release', {}, 't_pn', {}, 'end_reason', {}, 'x_end', {}, ...
           't_end', {}, 'log', {}, 'k', {});
for i = 1:n
    ic = scn.ic(i);
    th = deg2rad(ic.theta_deg);
    n0 = ic.r0*cos(th);
    e0 = ic.r0*sin(th);
    los0 = atan2(-e0, -n0);
    gam0 = deg2rad(ic.gam0_deg);
    phi0 = deg2rad(ic.phi0_deg);
    psi0 = los0 + deg2rad(ic.eps0_deg);

    % trim so the vehicle starts neither pitching nor sinking artificially
    rho0 = 1.225*(1 - 2.25577e-5*max(ic.z0, 0))^4.2586;
    qS0  = 0.5*rho0*ic.V0^2*c.b.S;
    nL0  = cos(gam0)/max(cos(phi0), 0.2);
    CL0  = min(c.b.CLmax, max(c.b.CLmin, nL0*c.b.W/max(qS0, 1e-6)));
    th0  = gam0 + (CL0/c.b.CLa)*cos(phi0) + deg2rad(ic.th0_offset_deg);

    V(i).tag  = ic.tag;
    V(i).x    = [n0; e0; ic.z0; ic.V0; gam0; psi0; th0; 0.0; phi0; 0.0];
    V(i).gs   = mgs.guidance();          % seeding convention: first output
    V(i).gs.dive_T = ic.dive_T;
    V(i).ap   = mgs.autopilot();
    V(i).tk   = mgs.seekerTracker();
    [V(i).meas, V(i).tk] = mgs.seekerTracker(V(i).x, scn.tgt, V(i).tk, c);
    % Same seeding as stot_sim, deliberately: nav_pitch starts at 0 (not at
    % trim) and the FC is gated on have_cmd, so the first fast-loop iteration
    % cannot act on an attitude target nobody sent.  ap.thr_pc_prev = NaN
    % bypasses the slew limiter on the very first call, as it does there.
    V(i).cmd  = struct('roll', 0.0, 'pitch', 0.0, 'thr', c.pn.thr_trim);
    V(i).dbg  = struct('phase', 'dive');
    V(i).u    = [0; 0; 100*c.pn.thr_trim];
    V(i).ap.thr_pc_prev = NaN;
    V(i).fl   = struct();
    V(i).have_cmd = false;
    V(i).done = false;
    V(i).t_release = ic.t_release;
    V(i).t_pn = ic.t_release;
    V(i).end_reason = '';
    V(i).x_end = [];
    V(i).t_end = NaN;
    if ic.t_release > 0, V(i).gs.phase = 'hold'; end
end
end

% -----------------------------------------------------------------------------
function V = finish(V, x, t, reason)
V.done = true;  V.x_end = x;  V.t_end = t;  V.end_reason = reason;
end

% -----------------------------------------------------------------------------
function veh = pack_vehicles(V, c)
veh = struct('tag', {}, 'hit', {}, 't_impact', {}, 'err_m', {}, ...
             'z_res_m', {}, 'v_impact', {}, 'gs_impact', {}, ...
             'gam_impact_deg', {}, 'end_reason', {}, 'traj', {}, 'flags', {});
for i = 1:numel(V)
    x = V(i).x_end;
    veh(i).tag = V(i).tag;
    veh(i).end_reason = V(i).end_reason;
    veh(i).t_impact = V(i).t_end;
    veh(i).err_m   = hypot(x(1), x(2));
    veh(i).z_res_m = x(3);
    % TOTAL speed and GROUND speed, both named.  See the header.
    veh(i).v_impact  = x(4);
    veh(i).gs_impact = x(4)*cos(x(5));
    veh(i).gam_impact_deg = rad2deg(x(5));
    veh(i).hit = strcmp(V(i).end_reason, 'impact') && ...
                 veh(i).err_m <= 10.0 && abs(veh(i).z_res_m) <= 1.0;

    L = V(i).log(1:V(i).k, :);
    veh(i).traj = array2table(L, 'VariableNames', logNames());
    veh(i).flags = flag_fractions(L);
end
end

function nm = logNames()
nm = {'t', 'n', 'e', 'z', 'V', 'gam_deg', 'psi_deg', 'th_deg', 'phi_deg', ...
      'nav_roll_deg', 'nav_pitch_deg', 'thr_pc', 'lam_dot', 'sig_dot', ...
      'nz', 'los_active', 'frozen', 't_go', 'nz_raw'};
end

function r = log_row(t, v)
x = v.x;  u = v.u;  d = v.dbg;
r = [t, x(1), x(2), x(3), x(4), rad2deg(x(5)), rad2deg(x(6)), ...
     rad2deg(x(7)), rad2deg(x(9)), rad2deg(u(1)), rad2deg(u(2)), u(3), ...
     getf(d, 'lam_dot', NaN), getf(d, 'sig_dot', NaN), getf(d, 'nz', NaN), ...
     double(getf(d, 'los_active', false)), double(getf(d, 'frozen', false)), ...
     getf(d, 't_go', NaN), getf(d, 'nz_raw', NaN)];
end

function v = getf(s, f, dflt)
if isstruct(s) && isfield(s, f), v = double(s.(f)); else, v = dflt; end
end

function f = flag_fractions(L)
if isempty(L), f = struct('los_active', NaN, 'frozen', NaN); return; end
f = struct('los_active', mean(L(:, 16), 'omitnan'), ...
           'frozen',     mean(L(:, 17), 'omitnan'));
end

% =============================================================================
function xn = rk4(x, u, h, c)
k1 = mgs.airframe(x, u, c);
k2 = mgs.airframe(x + 0.5*h*k1, u, c);
k3 = mgs.airframe(x + 0.5*h*k2, u, c);
k4 = mgs.airframe(x + h*k3, u, c);
xn = x + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
end

function hs = solve_cross(x, u, h, c)
%SOLVE_CROSS  Sub-step time at which z hits 0, by secant on the RK4 flow.
f = @(s) subsref(rk4(x, u, s, c), substruct('()', {3}));
a = 0; fa = x(3);
b = h; fb = f(h);
hs = h;
for k = 1:24
    if abs(fb - fa) < 1e-12, break; end
    hs = b - fb*(b - a)/(fb - fa);
    hs = min(max(hs, 0), h);
    fh = f(hs);
    if abs(fh) < 1e-9, break; end
    a = b; fa = fb; b = hs; fb = fh;
end
end

% =============================================================================
function s = snapshot(t, V, scn, c)
%SNAPSHOT  The wire payload for the bridge -- one struct per tick.
%   Kept flat and JSON-shaped on purpose: mgs.Bridge encodes it verbatim.
s = struct('t', t, 'tag', scn.tag, 'n', numel(V), 'uavs', struct([]));
for i = 1:numel(V)
    x = V(i).x;
    s.uavs(i).id    = i;
    s.uavs(i).tag   = V(i).tag;
    s.uavs(i).phase = V(i).gs.phase;
    s.uavs(i).done  = V(i).done;
    s.uavs(i).n = x(1); s.uavs(i).e = x(2); s.uavs(i).z = x(3);
    s.uavs(i).V = x(4);
    s.uavs(i).gam_deg = rad2deg(x(5));
    s.uavs(i).psi_deg = rad2deg(x(6));
    s.uavs(i).th_deg  = rad2deg(x(7));
    s.uavs(i).phi_deg = rad2deg(x(9));
    s.uavs(i).t_go = getf(V(i).dbg, 't_go', NaN);
    s.uavs(i).d    = hypot(x(1), x(2));
end
s.tgt = scn.tgt;
s.tgt_lat = c.swarm.tgt_lat;
s.tgt_lon = c.swarm.tgt_lon;
s.tgt_amsl = c.swarm.tgt_amsl;
end

function pace(wallT0, t)
%PACE  Hold the sim to wall clock for live display.  If the sim is already
%   behind, do NOT try to catch up -- report the slip instead.  Silently
%   running faster than real time while claiming to be real time is the same
%   class of lie as a stale parameter.
lag = t - toc(wallT0);
if lag > 0
    pause(lag);
elseif lag < -1.0
    warning('mgs:simSwarm:slip', ...
        'realtime pacing is %.1f s behind; the stream is NOT real time', -lag);
end
end
