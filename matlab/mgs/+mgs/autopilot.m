function [u, ap, fl] = autopilot(cmd, x, ap, dt_fc, c)
%MGS.AUTOPILOT  Autopilot block -- the ArduPlane clamp chain, plus two defects.
%
%   [u, ap, fl] = mgs.autopilot(cmd, x, ap, dt_fc, c)
%   ap = mgs.autopilot()                 seed the autopilot state (FIRST output)
%
%   cmd    .roll .pitch [rad] .thr [0..1]   what the guidance sent (ZOH 20 Hz)
%   x      plant state, for the MEASURED attitude the clamps feed back on
%   dt_fc  [s] fast-loop period, 1/SCHED_LOOP_RATE
%
%   u      [nav_roll; nav_pitch; thr_pc]  the plant's input
%   fl     clamp flags out of stot_fcs_clamp, plus the two below
%
% ---------------------------------------------------------------------------
%   WHERE THIS SITS RELATIVE TO THE MATHWORKS EXAMPLE
%
%   The example's autopilot is a classical THREE-LOOP acceleration autopilot:
%   an accelerometer ahead of the CG closes the outer loop, a rate gyro adds
%   damping, and the guidance commands a LATERAL ACCELERATION.
%
%   Ours is not that, and pretending otherwise would mislead every comparison:
%   tot_node commands an ATTITUDE (SET_ATTITUDE_TARGET roll/pitch/thrust) and
%   ArduPlane closes an angle loop on it -- rate demand = angle error / TCONST,
%   then a rate PID.  The acceleration loop does not exist.  That is why our
%   normal-load limits are aerodynamic and firmware clamps rather than an
%   autopilot gain, and why pn_nz_min turned out to be nearly inert.
%
%   stot_fcs_clamp is the firmware clamp chain; the angle loop itself lives in
%   stot_plant (it is a state, so it has to integrate with the airframe).
%
% ---------------------------------------------------------------------------
%   TWO DEFECTS ARE MODELLED HERE, BOTH OFF BY DEFAULT
%
%   1. c.fc.AIRSPEED_MAX REFUSAL.  A speed request above AIRSPEED_MAX is not
%      clipped by the firmware -- the whole command is REJECTED with
%      MAV_RESULT=4 and nothing downstream changes.  This once meant a
%      catch-up channel commanded at 157 m/s against AIRSPEED_MAX 150 never
%      existed at all, silently, for many sorties.  fl.speed_refused reports it.
%
%   2. c.gam.deficit_on -- THE GAMMA TRACKING SHORTFALL.  This model tracks a
%      commanded flight-path angle better than the vehicles appear to, which
%      is why every simulator in this project hits every time.
%
%      *** The famous 0.063 gain / 8 s lag pair is an ACCOUNTING ARTEFACT. ***
%      The LOS floor is applied after the nz clamp as a min(), its output is
%      an ANGLE command that bypasses the clamp, and dividing that angle step
%      by pn_lead = 1.0 s books it as a rate demand of up to -61 deg/s.  The
%      dataflash shows the inner loop was fine: attitude error 1.5-4.8 deg,
%      pitch-rate error 0.000, no limiting active.  Refitted as the angle loop
%      it is, the feasible ticks give 0.2916 / 3.43 s -- an ordinary
%      first-order angle tracker.  Those are the defaults in mgs.params.
%
%      The injection below is still PHENOMENOLOGICAL: it shapes the pitch
%      path's input-output behaviour, it does not explain it, and nobody has
%      yet checked that it reproduces the measured miss population.  Hence
%      the gate.
%
%   See also STOT_FCS_CLAMP, MGS.GUIDANCE, MGS.AIRFRAME.

if nargin == 0
    % Seeding convention (see mgs.guidance): no arguments -> the FIRST output
    % is the seed state, so `ap = mgs.autopilot()` is right.
    u = seed(); ap = u; fl = struct(); return;
end
if isempty(ap), ap = seed(); end

roll_meas  = x(9);
pitch_meas = x(7);

% ---- defect 1: the speed request is refused, not clipped -------------------
speed_refused = false;
thr_cmd = cmd.thr;
if isfield(c.fc, 'AIRSPEED_MAX') && ~isempty(ap.v_req)
    if ap.v_req > c.fc.AIRSPEED_MAX
        speed_refused = true;
        thr_cmd = ap.thr_last;      % the command never landed; nothing changed
    end
end

% ---- defect 2: gamma tracking deficit, injected on the pitch path ----------
pitch_cmd = cmd.pitch;
gam_now = x(5);                       % flight path angle IS a plant state here
if c.gam.deficit_on
    if ~c.gam.acknowledge_uncalibrated
        error('mgs:autopilot:gam_uncalibrated', ...
            ['c.gam.deficit_on = true, but the deficit block HAS NOT BEEN ' ...
             'CALIBRATED AGAINST OUTCOMES.\n' ...
             'The structure is right and the defaults are the angle-loop ' ...
             'refit (0.2916, 3.43 s), NOT the rate-space artefact pair ' ...
             '(0.063, 8.0 s) -- that pair is an accounting artefact of the ' ...
             'post-clamp LOS floor, and fed in directly it gives 0/8 at ' ...
             '800-960 m while RRE actually flew 15/16.\n' ...
             'But nobody has yet checked that the refit reproduces the ' ...
             'measured MISS POPULATION, so this knob still answers no ' ...
             'question.  Either\n' ...
             '  (a) fit gam.gain and gam.lag_s against ' ...
             'matlab/gammadiag/chain32.csv + matlab/data/truth32.csv until ' ...
             'the simulated miss population matches the measured one, and ' ...
             'record the fit, or\n' ...
             '  (b) set gam.acknowledge_uncalibrated = true to run it anyway ' ...
             'as a bound, knowing the output is not a prediction.']);
    end
    % First-order lag with a DC gain, referred to the model's own unaided gain
    % so the two are commensurable.  gam.gain and gam.lag_s are THE two fit
    % parameters; nothing else in this block is free.
    if ~ap.pitch_eff_init
        ap.pitch_eff = pitch_meas;    % start at trim, not at zero, or the
        ap.pitch_eff_init = true;     % first tick injects a step of its own
    end
    k = c.gam.gain / max(c.gam.model_gain, 1e-6);
    tau = max(c.gam.lag_s, 1e-3);
    target = gam_now + k*(pitch_cmd - gam_now);
    a = min(1, dt_fc/tau);
    ap.pitch_eff = ap.pitch_eff + a*(target - ap.pitch_eff);
    pitch_cmd = ap.pitch_eff;
else
    ap.pitch_eff = pitch_cmd;
    ap.pitch_eff_init = true;
end

% ---- the firmware clamp chain (unchanged, validated) ----------------------
[nav_roll, nav_pitch, thr_pc, fl] = stot_fcs_clamp( ...
    cmd.roll, pitch_cmd, thr_cmd, roll_meas, pitch_meas, ap.thr_pc_prev, ...
    dt_fc, c.fc);

ap.thr_pc_prev = thr_pc;
ap.thr_last    = thr_cmd;

fl.speed_refused = speed_refused;
fl.gam_deficit   = c.gam.deficit_on;
fl.pitch_cmd_in  = cmd.pitch;
fl.pitch_cmd_eff = pitch_cmd;

u = [nav_roll; nav_pitch; thr_pc];
end

% -----------------------------------------------------------------------------
function ap = seed()
ap = struct('thr_pc_prev', 0.0, 'thr_last', 0.0, 'pitch_eff', 0.0, ...
            'pitch_eff_init', false, 'v_req', []);
end
