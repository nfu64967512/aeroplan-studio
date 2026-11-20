function [nav_roll, nav_pitch, thr_pc, fl] = stot_fcs_clamp( ...
        roll_cmd, pitch_cmd, thr_cmd, roll_meas, pitch_meas, thr_pc_prev, dt_fc, fc)
%STOT_FCS_CLAMP  The ArduPlane-side clamp chain that sim3d.py does not have.
%
%   [nav_roll, nav_pitch, thr_pc, fl] = stot_fcs_clamp(roll_cmd, pitch_cmd, ...
%           thr_cmd, roll_meas, pitch_meas, thr_pc_prev, dt_fc, fc)
%
%   roll_cmd, pitch_cmd [rad]  what the node sent (ZOH at 20 Hz)
%   thr_cmd             [0..1] SET_ATTITUDE_TARGET.thrust
%   roll_meas,pitch_meas[rad]  AHRS/EKF attitude ESTIMATE at the top of this
%                              fast-loop iteration -- see the note below
%   thr_pc_prev         [%]    throttle output of the previous FC iteration
%   dt_fc               [s]    fast-loop period (1/SCHED_LOOP_RATE)
%
%   Order inside ONE fast-loop iteration, ArduPlane/Plane.cpp:62-67:
%       FAST_TASK(ahrs_update)          <- computes roll_limit_cd, pitch_limit_min
%       FAST_TASK(update_control_mode)  <- ModeGuided::update applies them
%       FAST_TASK(stabilize)
%       FAST_TASK(set_servos)
%
% ---------------------------------------------------------------------------
%   *** WHICH roll does pitch_limit_min *= |cos roll| use? ***
%   ahrs.cos_roll(), i.e. the AHRS/EKF attitude ESTIMATE, not nav_roll_cd and
%   not forced_rpy_cd.  Verbatim, ArduPlane/Plane.cpp:186-190:
%
%       if (rotate_limits) {
%           roll_limit_cd *= ahrs.cos_pitch();
%           pitch_limit_min *= fabsf(ahrs.cos_roll());
%       }
%
%   It is therefore NOT an algebraic loop: the limits are computed from the
%   attitude measured at the top of the loop and applied later in the same loop,
%   which is a one-plant-step delayed state feedback.  This function must be
%   handed the PLANT state, never this tick's command.  stot_selftest.m has a
%   check ('noloop') that swaps roll_meas for roll_cmd and asserts the result
%   changes -- if it does not change, someone wired up the command-feedback
%   version by mistake.
%
%   Numbers: roll 65 deg  =>  pitch_limit_min = -80 * 0.4226 = -33.8 deg.
%   Measured |roll| peaks at 65.1 deg on the truth set, i.e. right on the line.
%
%   Note the deliberate asymmetry, copied as-is:
%     roll_limit_cd   *= cos_pitch()          (SIGNED)
%     pitch_limit_min *= fabsf(cos_roll())    (ABS)
% ---------------------------------------------------------------------------

% ---- command quantisation: forced_rpy_cd is int centidegrees ---------------
% GCS_MAVLink_Plane.cpp:1050/1058, then constrain_int32 at mode_guided.cpp:42/80
if fc.cmd_quant_deg > 0
    qd        = fc.cmd_quant_deg;
    roll_cmd  = deg2rad(round(rad2deg(roll_cmd)/qd)*qd);
    pitch_cmd = deg2rad(round(rad2deg(pitch_cmd)/qd)*qd);
end

% ---- (a) ahrs_update(): Plane.cpp:178-190 ---------------------------------
roll_limit = deg2rad(fc.ROLL_LIMIT_DEG);
pitch_min  = deg2rad(fc.PTCH_LIM_MIN_DEG);
pitch_max  = deg2rad(fc.PTCH_LIM_MAX_DEG);
if fc.rotate_limits
    roll_limit = roll_limit * cos(pitch_meas);        % :188  SIGNED, no abs
    pitch_min  = pitch_min  * abs(cos(roll_meas));    % :189  ABS
end
% constrain_int32 with lo > hi would silently return hi.  We never get there
% (pitch stays inside [-80, 45] so cos_pitch > 0) but say so out loud.
assert(roll_limit >= 0, ...
    ['stot_fcs_clamp: roll_limit_cd went negative (|pitch| > 90 deg).  ' ...
     'constrain_int32 would return the high bound; that branch is not modelled.']);

% ---- (b) ModeGuided::update(): mode_guided.cpp:41 -------------------------
nav_roll = min(max(roll_cmd, -roll_limit), roll_limit);
% mode_guided.cpp:42 calls plane.update_load_factor(), which reaches
% Attitude.cpp:698-701 and early-returns because STALL_PREVENTION == 0.
% cfg.params asserts that, so there is nothing to do here.

% ---- (c) ModeGuided::update(): mode_guided.cpp:80 -------------------------
nav_pitch = min(max(pitch_cmd, pitch_min), pitch_max);

% ---- (d) throttle: mode_guided.cpp:94 -> servos.cpp:538-539,606 -----------
thr_pc = 100 * thr_cmd;                                   % forced_throttle
thr_pc = min(max(thr_pc, fc.THR_MIN), fc.THR_MAX);        % apply_throttle_limits
% set_slew_rate(k_throttle, THR_SLEWRATE, 100, G_Dt) -- servos.cpp:60.
% THR_SLEWRATE is %/s of full range, so 100 %/s means 0 -> 100 takes 1 s.
slew_hit = false;
if fc.THR_SLEWRATE > 0 && ~isnan(thr_pc_prev)
    dmax = fc.THR_SLEWRATE * dt_fc;
    dv   = thr_pc - thr_pc_prev;
    if abs(dv) > dmax
        thr_pc  = thr_pc_prev + sign(dv)*dmax;
        slew_hit = true;
    end
end

fl = struct( ...
    'fc_roll_clip',  abs(roll_cmd)  > roll_limit + 1e-12, ...
    'fc_pitch_lo',   pitch_cmd < pitch_min - 1e-12, ...
    'fc_pitch_hi',   pitch_cmd > pitch_max + 1e-12, ...
    'roll_limit',    roll_limit, ...
    'pitch_min',     pitch_min, ...
    'thr_clip',      (100*thr_cmd > fc.THR_MAX) || (100*thr_cmd < fc.THR_MIN), ...
    'thr_slew',      slew_hit);
end
