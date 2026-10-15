function [cmd, gs, dbg] = stot_guidance(m, gs, tnow, pn)
%STOT_GUIDANCE  Line-by-line port of tot_node.py::_pn_cmd + _pn_step freeze.
%
%   [cmd, gs, dbg] = stot_guidance(m, gs, tnow, pn)
%
%   m   measurement latch the node has in hand this tick (NOT truth):
%       .n .e .z    [m]   position in the target-tangent NED plane, target at
%                         the origin, z measured from the target datum.
%       .vn .ve .vd [m/s] NED velocity, vd DOWN positive.
%       .pitch      [rad] measured pitch attitude (ATTITUDE msg).
%   gs  guidance state, carried between ticks:
%       .alpha_lp .has_alpha .frozen .roll_frz .pitch_frz
%   tnow  [s]  node wall clock (same origin as gs.dive_T)
%   pn    guidance parameters (cfg.params().pn), plus gs.dive_T [s]
%
%   cmd .roll .pitch [rad], .thr [0..1]
%   dbg  one struct of saturation flags per tick -- these are the quantities the
%        fidelity gate compares, so every clamp gets its own field.
%
% ============================ ORDER IS PART OF THE SPEC ======================
%   nz clamp  ->  pn_lead  ->  LOS FLOOR min()  ->  alpha  ->  pitch clip
%             ->  node roll clip  ->  throttle  ->  terminal freeze
%
%   Getting the LOS floor before the nz clamp yields a model in which pn_nz_min
%   is effective.  On the real vehicle it is almost inert, because gam_los is
%   nearly always the more negative of the two and min() picks it.  See the
%   selftest 'order' in stot_selftest.m: with los_floor=1 a nz_min sweep must
%   move gam_cmd by exactly 0, and with los_floor=0 the same sweep must move it.
% =============================================================================
%   Source: scripts/wsl/sar_swarm_patches/tot_node.py @ d7562e2
%           _geom    2472-2484
%           _pn_cmd  2818-2928
%           _pn_step 2930-2952

GRAV = pn.g;                                             % tot_node.py:2820

% ---- _geom (2472-2484) -----------------------------------------------------
% dn,de point FROM the aircraft TO the bullseye.  Target sits at the origin of
% the local plane, so dn = 0 - n, de = 0 - e.
dn = -m.n;                                               % 2475
de = -m.e;                                               % 2476
d  = max(1e-3, hypot(dn, de));                           % 2477  horizontal range
h  = m.z;                                                % 2478  z - zg, zg = 0 at sea
R  = max(1e-3, hypot(d, h));                             % 2479  slant range
vn = m.vn;  ve = m.ve;  vd = m.vd;                       % 2480
vh   = hypot(vn, ve);                                    % 2481
hdot = -vd;                                              % 2482  UP positive
V    = max(1e-3, hypot(vh, hdot));                       % 2483
gam  = atan2(hdot, max(vh, 1e-3));                       % 2821  descent negative

% ---- lateral channel (2824-2836) -------------------------------------------
lam_dot = (de*vn - dn*ve) / (d*d);                       % 2824  analytic LOS rate
a_h     = pn.n_lat * vh * lam_dot;                       % 2825
if pn.itcg_on                                            % 2831
    error('stot_guidance:itcg', ...
        ['itcg_on is not implemented.  The RRE contract has itcg_on = 0 so ' ...
         'a_t == 0 for every flight in the truth set; a stub returning 0 ' ...
         'would silently look like a working ITCG.']);
end
a_t = 0.0;                                               % 2831-2833
a_h = a_h + a_t;
% Denominator is g*cos(gam), and cos(gam) itself is floored at 0.25.
roll_cmd = atan2(a_h, GRAV * max(cos(gam), 0.25));       % 2836

% ---- longitudinal channel up to the nz clamp (2840-2850) -------------------
ddot    = -(dn*vn + de*ve) / d;                          % 2840 (inlined)
sig_dot = (hdot*d - h*ddot) / (R*R);                     % 2840
gdot    = -pn.n_vert * sig_dot;                          % 2841
nz_raw  = cos(gam) + V*gdot/GRAV;                        % 2847
nz      = max(pn.nz_min, min(pn.nz_max, nz_raw));        % 2848  <- THE CLAMP
gdot    = (nz - cos(gam)) * GRAV / V;                    % 2849  clamp written back
gam_cmd_pn = gam + gdot * pn.lead;                       % 2850  1 s lead
gam_cmd = gam_cmd_pn;

% ---- LOS floor (2864-2866) -- MUST come after the nz clamp -----------------
los_active = false;
gam_los = NaN;
if pn.los_floor > 0.0
    gam_los = -atan2(h, max(d, 1e-3));                   % 2865
    gam_cap = gam_los * pn.los_floor;
    los_active = (gam_cap < gam_cmd);   % min() actually bit -> LOS is in charge
    gam_cmd = min(gam_cmd, gam_cap);                     % 2866
end

% ---- alpha estimate and attitude conversion (2869-2879) --------------------
th_meas = m.pitch;                                       % 2870  MEASURED pitch
a_raw   = th_meas - gam;                                 % 2871
if ~gs.has_alpha
    gs.alpha_lp = a_raw;      % self.alpha.get(i, a_raw) -- first tick seeds
    gs.has_alpha = true;
end
k = min(1.0, pn.tick_dt / pn.alpha_tau);                 % 2876  = 0.0285714...
gs.alpha_lp = gs.alpha_lp + k*(a_raw - gs.alpha_lp);     % 2877
% NOTE 2878 uses roll_cmd BEFORE the r_lim clip below (2886).  Do not reorder.
pitch_cmd = gam_cmd + gs.alpha_lp / max(cos(roll_cmd), 0.5);   % 2878

% ---- node-side attitude clips (2880-2886) ---------------------------------
if pn.dive_pitch_min_deg < 0.0                           % 2880-2882
    pmin = deg2rad(pn.dive_pitch_min_deg);
else
    pmin = deg2rad(-80.0);
end
pmax = deg2rad(pn.dive_pitch_max_deg);
pitch_lo = (pitch_cmd < pmin);
pitch_hi = (pitch_cmd > pmax);
pitch_cmd = max(pmin, min(pmax, pitch_cmd));

% r_lim uses th_meas (MEASURED pitch), not the pitch_cmd just computed, and the
% 80.0 is a literal in tot_node.py -- a second copy of ROLL_LIMIT_DEG.
r_lim = deg2rad(pn.node_roll_lim_deg) * abs(cos(th_meas));   % 2885
roll_node_clip = (abs(roll_cmd) > r_lim);
roll_cmd = max(-r_lim, min(r_lim, roll_cmd));            % 2886

% ---- throttle (2905-2925) --------------------------------------------------
switch lower(pn.throttle_law)
case 'clock'
    if isnan(gs.dive_T)
        t_left = 0.0;                                    % 2905, dive_T is None
    else
        t_left = gs.dive_T - tnow;
    end
    late  = (t_left <= 1.0);                             % 2906
    if ~late
        t_rem = t_left;                                  % 2907
    else
        t_rem = max(1.0, R/max(V,1e-3));
    end
    v_req = max(pn.v_req_floor, min(pn.vmax, R/t_rem));  % 2917
    if late && pn.late_full                              % 2918-2919
        v_req = pn.vmax;
    end
case 'vmax'
    % State-only variant for table building: dive_T is an absolute wall clock,
    % so the 'clock' law makes _pn_cmd not a function of the state.
    late  = false;
    v_req = pn.vmax;
otherwise
    error('stot_guidance:law','unknown throttle_law "%s"', pn.throttle_law);
end
thr_unsat = pn.thr_trim + pn.thr_gain*(v_req - V);       % 2920-2921
thr = max(0.0, min(1.0, thr_unsat));
if pn.dive_full_thr                                      % 2925
    thr = 1.0;
end

% ---- terminal freeze, _pn_step 2935-2949 ----------------------------------
% One-shot latch of roll/pitch only.  THROTTLE IS NOT FROZEN (2950).
t_go = R / max(V, 1e-3);                                 % 2935
freeze_now = (t_go <= pn.fz_tgo) || (R <= pn.fz_r);      % 2946
roll_out = roll_cmd;  pitch_out = pitch_cmd;
if freeze_now
    if ~gs.frozen
        gs.roll_frz  = roll_cmd;                         % 2947
        gs.pitch_frz = pitch_cmd;                        % 2948
        gs.frozen    = true;
    end
    roll_out  = gs.roll_frz;                             % 2949
    pitch_out = gs.pitch_frz;
end

cmd = struct('roll', roll_out, 'pitch', pitch_out, 'thr', thr);

% ---- instrumentation -------------------------------------------------------
dbg = struct( ...
    'd',d, 'h',h, 'R',R, 'V',V, 'gam',gam, 'vh',vh, 'hdot',hdot, ...
    'lam_dot',lam_dot, 'sig_dot',sig_dot, ...
    'nz_raw',nz_raw, 'nz',nz, ...
    'nz_lo', nz_raw < pn.nz_min, ...
    'nz_hi', nz_raw > pn.nz_max, ...
    'gam_cmd_pn',gam_cmd_pn, 'gam_los',gam_los, 'gam_cmd',gam_cmd, ...
    'los_active',los_active, ...
    'alpha_lp',gs.alpha_lp, ...
    'roll_cmd_raw',roll_cmd, ...     % after the node clip (what is sent)
    'pitch_lo',pitch_lo, 'pitch_hi',pitch_hi, ...
    'roll_node_clip',roll_node_clip, 'r_lim',r_lim, ...
    'v_req',v_req, 'late',late, 'thr_unsat',thr_unsat, ...
    'thr_sat_lo', thr_unsat < 0.0, 'thr_sat_hi', thr_unsat > 1.0, ...
    't_go',t_go, 'frozen',freeze_now);
end
