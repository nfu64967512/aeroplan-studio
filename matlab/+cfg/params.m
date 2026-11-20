function c = params(varargin)
%CFG.PARAMS  Effective-value snapshot for the STOT terminal-dive model.
%
%   c = cfg.params()                 RRE contract snapshot (the values that flew)
%   c = cfg.params('name',value,...) same, with dotted-path overrides, e.g.
%       cfg.params('pn.nz_min',-3.0, 'pn.los_floor',0.0)
%
%   PROVENANCE.  Every number below is either
%     (a) read back from runs/artifacts/RRE1/contract_RRE1.json  [CONTRACT],
%     (b) a shell default in scripts/wsl/swarm_ros2_up.sh        [SHELL],
%     (c) a literal in scripts/wsl/sar_swarm_patches/tot_node.py [LITERAL],
%     (d) a .parm line in scripts/wsl/swarm_sitl_up.sh           [PARM],
%     (e) an ArduPilot firmware default, repo does not override  [AP-DEFAULT],
%     (f) a calibrated airframe constant from predictor.py       [CALIB], or
%     (g) an UNCALIBRATED PRIOR                                  [PRIOR].
%
%   *** The PRIOR entries are the only free parameters in this model.  They are
%   listed together in c.meta.priors so a reviewer can find them without reading
%   the file.  They were NOT fitted to the 32-flight truth set. ***
%
%   WARNING.  Source defaults in tot_node.py are NOT what flew.  Three of them
%   differ enormously (pn_los_floor 0.0 -> 1.0, pn_nz_min +0.05 -> -1.0,
%   pn_vmax 80 -> 210).  Copying the source defaults gives a different model.

% ---------------------------------------------------------------- guidance --
pn.n_lat       = 4.0;      % SHELL    swarm_ros2_up.sh:561  (N_LAT)
pn.n_vert      = 5.0;      % SHELL    swarm_ros2_up.sh:562  (N_VERT)
pn.lead        = 1.0;      % SHELL    swarm_ros2_up.sh:563  [s]
pn.alpha_tau   = 1.75;     % SHELL    swarm_ros2_up.sh:439  [s]
pn.nz_min      = -1.0;     % CONTRACT RRE1 pn_nz_min   (source default is +0.05)
pn.nz_max      =  3.0;     % LITERAL  tot_node.py:2848  (not a parameter)
pn.los_floor   =  1.0;     % CONTRACT RRE1 pn_los_floor (source default is 0.0)
pn.vmax        = 210.0;    % CONTRACT RRE1 pn_vmax     (source default is 80.0)
pn.thr_trim    = 0.36;     % SHELL    swarm_ros2_up.sh:371
pn.thr_gain    = 0.030;    % SHELL    swarm_ros2_up.sh:372  [1/(m/s)]
pn.late_full   = true;     % SHELL    swarm_ros2_up.sh:572  (PN_LATE_FULL=True)
pn.dive_full_thr = false;  % SHELL    swarm_ros2_up.sh:571  (DIVE_FULL_THR=False)
pn.v_req_floor = 45.0;     % LITERAL  tot_node.py:2917
pn.fz_tgo      = 1.5;      % SHELL    swarm_ros2_up.sh:567  [s]
pn.fz_r        = 100.0;    % SHELL    swarm_ros2_up.sh:566  [m]
pn.dive_pitch_min_deg = -80.0;  % SHELL swarm_ros2_up.sh:317
pn.dive_pitch_max_deg =  45.0;  % SHELL swarm_ros2_up.sh:318
pn.node_roll_lim_deg  =  80.0;  % LITERAL tot_node.py:2885 -- a hard-coded 80.0,
                                %   NOT the .parm ROLL_LIMIT_DEG.  Two copies
                                %   that happen to hold the same value.
pn.itcg_on     = false;    % CONTRACT RRE1 itcg_on = 0  => a_t == 0
pn.tick_dt     = 0.05;     % LITERAL  tot_node.py:119 TICK_PERIOD_S => 20 Hz
pn.g           = 9.80665;  % LITERAL  GRAV inside _pn_cmd
pn.throttle_law = 'clock'; % 'clock' = reproduce the flight (dive_T wall clock),
                           % 'vmax'  = state-only law for table building.

% ------------------------------------------------------------ flight ctrl --
fc.PTCH_LIM_MIN_DEG = -80.0;  % PARM swarm_sitl_up.sh:217 (DIVE_PITCH_MIN)
fc.PTCH_LIM_MAX_DEG =  45.0;  % PARM swarm_sitl_up.sh:218 (DIVE_PITCH_MAX)
fc.ROLL_LIMIT_DEG   =  80.0;  % PARM swarm_sitl_up.sh:219 (DIVE_ROLL_MAX=80)
fc.STALL_PREVENTION =  0;     % PARM swarm_sitl_up.sh:220  <- asserted, see below
fc.THR_MIN          =  0;     % PARM swarm_sitl_up.sh:226
fc.THR_MAX          = 100;    % PARM swarm_sitl_up.sh:233, JET_THRMAX unset
fc.THR_SLEWRATE     = 100;    % AP-DEFAULT [%/s], repo does not set it
fc.GUIDED_TIMEOUT   =  3.0;   % AP-DEFAULT [s]
fc.loop_hz          = 50;     % AP-DEFAULT SCHED_LOOP_RATE, repo does not set it
fc.PTCH2SRV_TCONST  = 0.5;    % AP-DEFAULT AP_PitchController.cpp:35  [s]
fc.RLL2SRV_TCONST   = 0.5;    % AP-DEFAULT AP_RollController.cpp:35   [s]
fc.PTCH2SRV_RMAX    = 0.0;    % AP-DEFAULT AP_PitchController.cpp:46,55 => no cap
fc.cmd_quant_deg    = 0.01;   % GCS_MAVLink_Plane.cpp:1050/1058 centidegree int
fc.rotate_limits    = true;   % Plane.cpp:186, true for non-tailsitter

% ------------------------------------------------------------- airframe ----
b.m     = 22.0;    % CALIB predictor.py:39
b.S     = 0.45;    % CALIB predictor.py:40
b.span  = 1.88;    % CALIB predictor.py:41
b.e     = 0.9;     % CALIB predictor.py:42
b.CD0   = 0.01500; % CALIB predictor.py:43 -- inverse-solved from RPK/RPL, n=1617,
                   %       sd 0.00058.  DO NOT REFIT.
b.g     = 9.80665;
b.AR    = b.span^2 / b.S;          % 7.85422...
b.piEAR = pi * b.e * b.AR;         % 22.207290...
b.W     = b.m * b.g;               % 215.7463 N
b.Tfull = b.m * b.g / 0.7;         % 308.2090 N   predictor.py:46
b.rho0  = 1.225;
% --- PRIOR: the lift curve.  predictor.py never needed it (it assumed
%     quasi-equilibrium L = W cos gamma), so there is no calibrated value.
b.CLa    = 2*pi*b.AR / (2 + sqrt(b.AR^2 + 4));   % Helmbold straight wing, 4.884/rad
b.CLmax  = 1.2;    % PRIOR  plain wing, no flaps
b.CLmin  = -0.9;   % PRIOR  inverted / pushover capability
b.alpha_max_rad = deg2rad(25);  % PRIOR  hard AoA stop

% ------------------------------------- inner loop (the only black box, M4) --
% ArduPilot structure: angle error -> demanded body rate (P = 1/TCONST, and no
% rate cap because RMAX=0) -> rate PID -> surface -> short period.  We model the
% angle P exactly from the firmware default and lump everything downstream of it
% into one first-order lag.  The attitude loop is therefore 2nd order overall,
% which is what M4 asks for (sim3d had a single 1st-order lag).
il.tau_q = 0.10;   % PRIOR [s] pitch rate loop + actuator + short period, lumped
il.tau_p = 0.08;   % PRIOR [s] roll rate loop + actuator, lumped
il.q_max = deg2rad(200);  % PRIOR [rad/s] body-rate stop
il.p_max = deg2rad(300);  % PRIOR [rad/s]

% ------------------------------------------------------- measurement (M8) --
meas.enable_quant = true;
meas.q_pos_m   = 1.1119e-2;  % lat/lon are int32 1e-7 deg => 1.11 cm
meas.q_z_m     = 1e-3;       % GLOBAL_POSITION_INT.relative_alt is int32 mm
meas.q_vel_ms  = 1e-2;       % .vx/.vy/.vz are int16 cm/s
meas.q_att_rad = 0.0;        % ATTITUDE is float32 -- no quantisation
meas.tel_hz    = 40;         % tel_us = 25000 in swarm_sitl_up.sh
meas.noise_pos_m   = 0.0;    % PRIOR, default OFF
meas.noise_vel_ms  = 0.0;    % PRIOR, default OFF
meas.noise_att_rad = 0.0;    % PRIOR, default OFF
meas.seed = 20260819;

% -------------------------------------------------------------- simulator --
sim.h_max     = 0.005;   % [s] max RK4 step; the scheduler shortens it to land
                         %     exactly on every 20/40/50 Hz boundary.
sim.t_max     = 200.0;   % [s] after release
sim.post_s    = 4.0;     % [s] keep flying past the z=0 crossing so the closest-
                         %     horizontal-passage stats (z_res, d1_min) are taken
                         %     on the same kind of window the truth extractor uses
sim.z_floor   = -400.0;  % [m] stop the post-touchdown tail here
sim.freeze_on = false;   % M7 node outage, default OFF (a defect, not physics)
sim.freeze_period_s = 35.8;
sim.freeze_dur_s    = 3.6;
sim.freeze_phase_s  = 18.0;  % measured data fixes the period and the duration
                             % but not the phase; 18 s puts the single outage in
                             % the middle of a ~40 s dive.  Sweep it.
sim.log = true;

% ------------------------------------------------------------------ meta ---
meta.source_branch = 'fix/dccpp-geofence-2d @ d7562e2';
meta.contract      = 'runs/artifacts/RRE1/contract_RRE1.json';
meta.not_modelled  = { ...
  'M9  update_load_factor / STALL_PREVENTION -- dead while STALL_PREVENTION=0 (asserted)'; ...
  'M11 sideslip / yaw / turn-coordination error -- perfect coordination assumed'; ...
  'M12 wind -- SITL had none (no SIM_WIND anywhere in scripts/wsl)'; ...
  'compressibility -- absorbed into the calibrated CD0 at the calibration Mach'};
meta.priors = { ...
  'b.CLa, b.CLmax, b.CLmin, b.alpha_max_rad -- lift curve, never calibrated'; ...
  'il.tau_q, il.tau_p, il.q_max, il.p_max   -- inner-loop lump'; ...
  'meas.noise_* -- default 0; quantisation is exact, sensor noise is not modelled'};
meta.priors_note = [ ...
  'These are the ONLY tunable numbers.  They were set from first principles ' ...
  'before the 32-flight comparison was run and were not adjusted afterwards.'];

c = struct('pn',pn,'fc',fc,'b',b,'il',il,'meas',meas,'sim',sim,'meta',meta);

% ---- dotted-path overrides -------------------------------------------------
if mod(numel(varargin),2) ~= 0
    error('cfg:params','overrides must come in name/value pairs');
end
for k = 1:2:numel(varargin)
    pth = varargin{k};
    if ~(ischar(pth) || isstring(pth))
        error('cfg:params','override name %d is not a string', (k+1)/2);
    end
    parts = strsplit(char(pth), '.');
    if numel(parts) ~= 2
        error('cfg:params','override "%s" must be group.field', char(pth));
    end
    if ~isfield(c, parts{1}) || ~isfield(c.(parts{1}), parts{2})
        error('cfg:params','unknown override "%s"', char(pth));
    end
    c.(parts{1}).(parts{2}) = varargin{k+1};
end

% ---- assertions that guard model validity ---------------------------------
% If STALL_PREVENTION is ever turned on, Attitude.cpp:713-756 recomputes
% roll_limit_cd from (V/AIRSPEED_MIN)^2 and the whole lateral authority model
% changes.  Refuse to run silently in that case.
assert(c.fc.STALL_PREVENTION == 0, ...
    ['cfg:params: STALL_PREVENTION ~= 0.  apply_load_factor_roll_limits() stops ' ...
     'early-returning (Attitude.cpp:698-701) and rewrites roll_limit_cd.  That ' ...
     'branch is deliberately NOT modelled -- see meta.not_modelled M9.']);
assert(c.fc.PTCH2SRV_RMAX == 0, ...
    'cfg:params: PTCH2SRV_RMAX ~= 0 -- firmware would cap pitch rate; not modelled.');
assert(c.pn.nz_max == 3.0, 'cfg:params: nz_max literal changed; re-check tot_node.py:2848.');
end
