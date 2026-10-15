function c = params(varargin)
%MGS.PARAMS  Effective-value snapshot for the MULTI-vehicle guidance sim.
%
%   c = mgs.params()                      RRE contract snapshot, n = 2
%   c = mgs.params('swarm.n', 8, ...)     dotted-path overrides
%
%   This is cfg.params() -- the validated single-vehicle snapshot -- plus the
%   four sections a swarm needs and a single vehicle does not:
%
%       c.swarm   how many, where they start, when they are released
%       c.trk     Seeker/Tracker  (the MathWorks example's first block)
%       c.up      the upstream that delivers vehicles to the release point
%       c.io      the ROS 2 / TCP bridge
%
%   PROVENANCE TAGS carry over from cfg.params:
%       [CONTRACT] read back from runs/artifacts/*/contract_*.json
%       [SHELL]    a default in scripts/wsl/swarm_ros2_up.sh
%       [LITERAL]  a literal in tot_node.py
%       [PRIOR]    UNCALIBRATED.  Listed in c.meta.priors.
%
%   *** Everything new in this file is [PRIOR] unless tagged otherwise. ***
%   The single-vehicle dive chain is calibrated against 32 flights; the swarm
%   layer around it is NOT, and mgs.simSwarm stamps out.calibrated = false
%   whenever an uncalibrated section is actually exercised.
%
%   See also CFG.PARAMS, MGS.FROMCONTRACT, MGS.SIMSWARM.

c = cfg.params();          % everything validated, untouched

% ---- two flight-controller PARMs cfg.params does not carry ------------------
% They are in the contract table, and they bite: catch_speed 157 once exceeded
% AIRSPEED_MAX 150 and the WHOLE command was rejected with MAV_RESULT=4, so the
% catch-up channel never existed.  A speed request above AIRSPEED_MAX is not
% clipped by the firmware -- it is refused.  mgs.autopilot models the refusal.
c.fc.AIRSPEED_MIN  =  25.0;        % [CONTRACT] AIRSPEED_MIN
c.fc.AIRSPEED_MAX  = 230.0;        % [CONTRACT] AIRSPEED_MAX

% ---- which terminal law -----------------------------------------------------
% 'node'             the flight law.  stot_guidance, the line-by-line port of
%                    tot_node.py::_pn_cmd, validated on 32 sorties.  DEFAULT.
% 'zarchan_additive' Zarchan's ADDITIVE terminal-angle term (zarchan.txt:10887,
%                    nc = 4*Vc*lamdot + 2*Vc*(lam-gamF)/tgo) instead of our
%                    one-sided min().  A min() has zero gradient on its active
%                    side, so the loop is OPEN there -- and the floor is active
%                    on 43.1% of PN samples, 89.8-99.7% on the four aircraft
%                    that missed.  NEVER FLOWN.  Any run using it is marked
%                    uncalibrated.
% ⚠️ THE SECOND CLAMP (tot_node.py:3102).  1 = re-clamp the back-solved nz
% AFTER the LOS floor, which is what closes the bypass; 0 = the shipped
% order, where the floor's output bypasses [nz_min, nz_max] entirely.
% ⚠️⚠️ stot_guidance.m does NOT implement this branch -- its header pins the
% port to commit d7562e2 and the branch is newer.  So mgs.simSwarm cannot
% simulate it; only mgs.lawCheck can, against a real log.  Setting it here
% and expecting simSwarm to honour it would be a silent no-op, so simSwarm
% raises instead.
c.pn.nz_clamp_after_floor = false;
c.pn.law        = 'node';
% ⚠️⚠️ Zarchan's GAMFDEG: the DEMANDED terminal flight-path angle (deg), a
% CONSTANT SETPOINT.  His own value is -90 (zarchan.txt:10934).  It is what
% makes the shaping term a terminal-angle law: an earlier import used the
% CURRENT gamma here and the term became an attractor with no impact-angle
% demand at all (opposite sign on 32/32 release states).
c.pn.gam_f_deg  = -90.0;
c.pn.shape_gain = 0.0;             % Zarchan's coefficient; his is 2.0 paired
                                   % with N' = 4, and we fly n_vert = 5.  The
                                   % default is 0 (term off) because adopting
                                   % a gain nobody set is exactly the failure
                                   % this project is organised against.

% ============================================================ swarm ==========
% n = 2 is what the truth set flew.  The programme target is 16 (8 x 2 groups);
% see the note in mgs.scenario about what stops being true above n = 2.
c.swarm.n          = 2;            % [CONTRACT] RRE flew 2 aircraft
c.swarm.tgt_lat    = 23.5000;      % [PRIOR] demo bullseye, overwritten by the
c.swarm.tgt_lon    = 119.5000;     %         bridge / by mgs.scenario
c.swarm.tgt_amsl   = 0.0;          % [CONTRACT] sea-level target datum.  Do NOT
                                   %   take this from a terrain library: water
                                   %   tiles return garbage (17 m of spread).
c.swarm.T_arr      = [];           % [s] common time over target; [] = use the
                                   %     per-vehicle dive_T from the scenario
c.swarm.stagger_s  = 0.0;          % [PRIOR] deliberate release stagger

% ==================================================== release geometry =======
% The fan / perch / release numbers.  mgs.scenario turns these into one release
% state per vehicle; mgs.fromContract overwrites them from a flight readback.
c.scen.r_fan            = 35000.0;  % [CONTRACT] fan radius
c.scen.half_angle_deg   =    25.0;  % [CONTRACT] half-angle of the fan
c.scen.z0               =  3100.0;  % [CONTRACT] perch_alt, height over the datum
c.scen.dive_angle_deg   =    27.0;  % [CONTRACT] commanded dive angle (magnitude)
c.scen.dive_speed       =   210.0;  % [CONTRACT] dive_speed
c.scen.vmax             =   150.0;  % [CONTRACT] vmax (AIRSPEED_MAX guard)
c.scen.V0               =   157.0;  % [MEASURED] release GROUNDSPEED at 3100 m,
                                    %   = EAS 135.  Feeding 157 as an EAS
                                    %   overshoots the thrust ceiling by 17%.
c.scen.eps0_deg         =    25.0;  % [MEASURED] release heading offset; the
                                    %   validated operating point is 25-40 deg
                                    %   with R ~ 6760 m (spread 2.47 s).
c.scen.r0               =  6760.0;  % [MEASURED] release range
c.scen.spread_mode      = 'even';   % 'even' | 'mirror' | 'fromTruth'

% ============================================== Seeker / Tracker =============
% MathWorks "Designing a Guidance System in MATLAB and Simulink": the
% Seeker/Tracker estimates the inertial sightline rate through a tracking loop
% with a 0.05 s time constant, and sets the estimator bandwidth at half the
% autopilot bandwidth.
%
% tot_node.py does NOT do that.  It differentiates the LOS ANALYTICALLY from
% the position/velocity latch (tot_node.py:2824), so it carries no tracker lag
% and no tracker noise.  Both are implemented here; 'analytic' is the default
% because 'analytic' is what flew.  Switching to 'tracker' changes the answer
% and has to be a deliberate act.
c.trk.mode         = 'analytic';   % 'analytic' [CONTRACT] | 'tracker' [PRIOR]
c.trk.tau          = 0.05;         % [s] MathWorks tracker loop time constant
c.trk.sigma_noise  = 0.0;          % [rad] [PRIOR] sightline noise, 1-sigma
c.trk.vc_noise     = 0.0;          % [m/s] [PRIOR] closing-velocity noise
c.trk.hz           = c.meas.tel_hz;   % a tracker cannot run faster than the
                                      % telemetry feeding it (40 Hz)

% ================================================== upstream model ===========
% How a vehicle GETS to the release point.  Three settings, not interchangeable:
%
%   'ic'     take the release state straight from the scenario.  This is the
%            validated path -- identical to what stot_sim does, with the dive
%            chain downstream checked against 32 flights.
%
%   'ring'   fly a hold ring and release on the gate.  UNCALIBRATED, and wrong
%            in a known direction: the commanded ring (95 m/s, 1300 m) was
%            never flown.  The vehicles actually did 124-186 m/s on a ~5867 m
%            circle and cut straight through.  c.up.ring_honest = true models
%            the ring that was flown instead of the one that was ordered.
%            Either way out.calibrated goes false.
%
%   'ros'    the release state arrives over the bridge from the live WSL stack.
c.up.mode          = 'ic';
c.up.ring_r        = 1300.0;       % [CONTRACT] hold_radius, commanded
c.up.ring_v        = 95.0;         % [CONTRACT] vmin, commanded
c.up.ring_honest   = false;        % [PRIOR] use the measured ring instead
c.up.ring_r_meas   = 5867.0;       % [MEASURED] what the ring actually was
c.up.ring_v_meas   = 155.0;        % [MEASURED] midpoint of 124-186 m/s
c.up.gate_r        = 4700.0;       % [CONTRACT] gate_r
c.up.release_at_ring = true;       % [CONTRACT] release_at_ring = 1

% ======================================================= bridge / io =========
c.io.transport     = 'tcp';        % 'tcp' | 'ros2' | 'none'
c.io.host          = '127.0.0.1';  % WSL2 NAT forwards Windows->WSL on loopback
c.io.port          = 5799;         % mgs_bridge_node.py default
c.io.domain_id     = 0;            % 'ros2' transport only
c.io.mode          = 'shadow';     % 'shadow' | 'sil' | 'offline'
c.io.rate_hz       = 20;           % publish rate of the shadow stream
c.io.timeout_s     = 2.0;
c.io.strict        = true;         % a dropped link ABORTS.  Never silently
                                   % fall back to the offline model -- that is
                                   % the exact failure shape this project bans.

% ===================================================== gamma tracking ========
% *** READ THIS BEFORE TRUSTING ANY RESULT OUT OF THIS SIM. ***
%
% This model tracks a commanded flight path angle far better than the vehicles
% appear to, so it hits every time and reproduces none of the observed
% failures.  This section is the HOOK for injecting the shortfall: a
% first-order lag with a DC gain on the pitch-command path, with gain and
% lag_s the only two free parameters.
%
% THE HEADLINE NUMBERS HAVE BEEN REFUTED, AND THE CORRECTION MATTERS HERE.
% The often-quoted pair (rate-tracking gain 0.063, lag 8.0 s, 26/32 at the
% search ceiling) is largely an ACCOUNTING ARTEFACT.  In tot_node the LOS
% floor is applied AFTER the nz clamp as a min(), and its output is an ANGLE
% command that bypasses the clamp entirely; dividing that angle step by
% pn_lead = 1.0 s records it as a rate demand of up to -61 deg/s that the
% vehicle was never going to meet.  The dataflash says the inner loop was
% fine: attitude error 1.5-4.8 deg, pitch-rate loop error 0.000, no limiting
% active anywhere.
%
% Refitted as the ANGLE loop it actually is:
%     all ticks        k = 0.0661  ->  tau = 15.1 s   (this is why the lag fit
%                                                      pinned at its 8 s cap)
%     feasible ticks   k = 0.2916  ->  tau =  3.43 s, corr +0.552
% i.e. an ordinary first-order angle-tracking loop.  The defaults below are
% the FEASIBLE-TICK refit, not the artefact pair.
%
% *** STILL NOT CALIBRATED AGAINST OUTCOMES, AND IT REFUSES TO RUN. ***
% Independent confirmation that the artefact pair is unusable as block
% parameters: fed in directly it gives 0/8 hits at 800-960 m, and even with
% the gain opened to 1.0 the miss is still 205 m -- the 8 s lag alone leaves
% the vehicle ballistic at gamma_0.  RRE actually flew 15/16.
%
% To use it: fit gain and lag_s against matlab/gammadiag/chain32.csv plus
% matlab/data/truth32.csv until the simulated miss population matches the
% measured one -- or set acknowledge_uncalibrated to run it as a deliberately
% pessimistic bound, knowing the output is not a prediction.
c.gam.deficit_on   = false;
c.gam.acknowledge_uncalibrated = false;
c.gam.gain         = 0.2916;       % [REFIT] angle loop, feasible ticks
c.gam.lag_s        = 3.43;         % [REFIT] tau from the same fit
c.gam.model_gain   = 1.0;          % the refit is already referred to the
                                   % angle loop, so no rescaling is needed
c.gam.artefact_gain = 0.063;       % [ARTEFACT] kept only so the number is
c.gam.artefact_lag_s = 8.0;        % [ARTEFACT] recognisable when quoted
c.gam.note = ['0.063 / 8.0 s is a rate-space accounting artefact of the ' ...
              'post-clamp LOS floor; the angle-loop refit is 0.2916 / ' ...
              '3.43 s on feasible ticks.  See the memory note ' ...
              'gamma-tracking-deficit and matlab/gammadiag/.'];

% ============================================================ overrides ======
% cfg.params returns a COLUMN cell; append as a column or horzcat throws.
c.meta.priors = [c.meta.priors(:); ...
    {'trk.sigma_noise, trk.vc_noise -- seeker noise, hardware we do not have'; ...
     'up.ring_* -- the hold ring was never instrumented'; ...
     'swarm.stagger_s -- deliberate release stagger'; ...
     'scen.spread_mode, scen.eps0_deg walk -- fan layout above n = 2'; ...
     'gam.* -- phenomenological, and only when c.gam.deficit_on'}];
c = apply_overrides(c, varargin);
end

% -----------------------------------------------------------------------------
function c = apply_overrides(c, kv)
%APPLY_OVERRIDES  'a.b.c',value pairs -> c.a.b.c = value, with an existence
%   check.  A typo in an override name is a silent no-op in most codebases;
%   here it is an error, because "I set it and nothing happened" is the single
%   most expensive bug class in this project.
if isempty(kv), return; end
if mod(numel(kv), 2) ~= 0
    error('mgs:params:pairs', 'overrides must be name,value pairs');
end
for k = 1:2:numel(kv)
    name = kv{k};  val = kv{k+1};
    if ~(ischar(name) || isstring(name))
        error('mgs:params:name', 'override %d: name must be char/string', (k+1)/2);
    end
    parts = strsplit(char(name), '.');
    if ~has_path(c, parts)
        error('mgs:params:unknown', ...
            ['override "%s" does not name an existing field.  Refusing to ' ...
             'create it: a typo here would look exactly like the override ' ...
             'having worked.'], char(name));
    end
    c = setfield(c, parts{:}, val);   %#ok<SFLD>
end
end

function tf = has_path(s, parts)
tf = false;
for k = 1:numel(parts)
    if ~isstruct(s) || ~isfield(s, parts{k}), return; end
    s = s.(parts{k});
end
tf = true;
end
