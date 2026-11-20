function mdl = build(c, scn, opts)
%MGS.SLX.BUILD  Generate the Simulink multi-missile guidance model.
%
%   mdl = mgs.slx.build()                      defaults, n = 2
%   mdl = mgs.slx.build(c, scn)                your parameters and scenario
%   mdl = mgs.slx.build(c, scn, 'open', true)  ... and open it
%
%   Builds matlab/mgs/models/mgs_multi.slx with the block decomposition of the
%   MathWorks example "Designing a Guidance System in MATLAB and Simulink":
%
%       Guidance System
%           Seeker/Tracker   40 Hz   mgs.slx.SeekerTracker
%           Guidance         20 Hz   mgs.slx.Guidance
%           Autopilot        50 Hz   mgs.slx.Autopilot
%       Airframe
%           Aero & EOM       cont.   mgs.slx.Airframe  -> Integrator
%
%   WHERE OURS DIFFERS FROM THE EXAMPLE, AND WHY IT IS NOT AN OVERSIGHT
%
%     Fin actuator          absent.  The example models a fin servo because
%                           its autopilot commands fin deflection.  ArduPlane
%                           closes an angle loop and we command attitude, so
%                           the servo lives inside the lumped inner-loop lag
%                           in stot_plant.  Adding a fin block here would be
%                           inventing hardware.
%     Aero lookup tables    absent.  The example interpolates 2-D tables over
%                           incidence and Mach for a Mach 2-4 missile.  Ours
%                           is a subsonic jet UAV with an analytic polar whose
%                           CD0 was inverse-solved from 1617 flight samples.
%     Three-loop autopilot  replaced by the ArduPlane clamp chain.  There is
%                           no accelerometer outer loop on this vehicle.
%     Stateflow mode logic  replaced by the phase switch in mgs.guidance.
%     N vehicles            every signal is N x k, and the blocks loop.  The
%                           example flies one missile.
%
%   *** NUMERICAL FIDELITY. ***  This model uses a Simulink solver; mgs.simSwarm
%   uses the fixed-step RK4 and secant touchdown search copied from stot_sim,
%   and reproduces it bit for bit.  The two will not agree in the last digits.
%   Use this model to study and present STRUCTURE.  Use mgs.simSwarm for any
%   number that has to line up with the 32-flight truth set.  mgs.selftest
%   compares them against a stated tolerance rather than assuming they match.
%
%   See also MGS.SIMSWARM, MGS.SLX.SEEKERTRACKER, MGS.SELFTEST.

arguments
    c = mgs.params()
    scn = []
    opts.name (1,:) char = 'mgs_multi'
    opts.open (1,1) logical = false
    opts.stopTime (1,1) double = 60
end

if isempty(scn), scn = mgs.scenario(c); end
n = scn.n;

mdl = opts.name;
if bdIsLoaded(mdl), close_system(mdl, 0); end
% The .slx we are about to overwrite is on the path, so new_system warns about
% shadowing.  That is exactly what regenerating a generated model means, so the
% warning is suppressed here and only here, and restored on the way out.
ws = warning('off', 'Simulink:Engine:MdlFileShadowing');
restoreWarn = onCleanup(@() warning(ws));
new_system(mdl);

% ---- the model needs these in the base workspace, and says so --------------
% A Simulink model that silently depends on undeclared workspace variables is
% a model that runs differently for the next person.  Everything the diagram
% references is written here, under names prefixed mgs_, and listed in the
% model annotation.
assignin('base', 'mgs_c',  c);
assignin('base', 'mgs_N',  n);
assignin('base', 'mgs_X0', initial_state(scn, c));
assignin('base', 'mgs_diveT',  [scn.ic.dive_T]);
assignin('base', 'mgs_tRel',   [scn.ic.t_release]);

set_param(mdl, 'StopTime', num2str(opts.stopTime), ...
               'Solver', 'ode45', ...
               'RelTol', '1e-8', 'AbsTol', '1e-10', ...
               'SolverType', 'Variable-step', ...
               'MaxStep', '0.005');

P = @(x, y) sprintf('[%d %d %d %d]', x, y, x+160, y+70);

% ---- blocks ----------------------------------------------------------------
add_msblock(mdl, 'Seeker Tracker', 'mgs.slx.SeekerTracker', P(340, 60), ...
    {'Params', 'mgs_c'; 'N', 'mgs_N'; 'SampleTime', num2str(1/c.meas.tel_hz)});

add_msblock(mdl, 'Guidance', 'mgs.slx.Guidance', P(560, 60), ...
    {'Params', 'mgs_c'; 'N', 'mgs_N'; 'SampleTime', num2str(c.pn.tick_dt); ...
     'DiveT', 'mgs_diveT'; 'TRelease', 'mgs_tRel'});

add_msblock(mdl, 'Autopilot', 'mgs.slx.Autopilot', P(780, 60), ...
    {'Params', 'mgs_c'; 'N', 'mgs_N'; 'SampleTime', num2str(1/c.fc.loop_hz)});

add_msblock(mdl, 'Airframe', 'mgs.slx.Airframe', P(780, 220), ...
    {'Params', 'mgs_c'; 'N', 'mgs_N'});

add_block('simulink/Continuous/Integrator', [mdl '/States'], ...
    'Position', P(560, 220), 'InitialCondition', 'mgs_X0');

add_block('simulink/Signal Routing/Goto', [mdl '/X out'], ...
    'Position', '[400 240 460 260]', 'GotoTag', 'X');
add_block('simulink/Signal Routing/From', [mdl '/X in 1'], ...
    'Position', '[260 80 320 100]', 'GotoTag', 'X');
add_block('simulink/Signal Routing/From', [mdl '/X in 2'], ...
    'Position', '[700 140 760 160]', 'GotoTag', 'X');

logs = {'X', 'CMD', 'U', 'DBG', 'FL', 'AUX'};
for k = 1:numel(logs)
    add_block('simulink/Sinks/To Workspace', [mdl '/log ' logs{k}], ...
        'Position', sprintf('[%d %d %d %d]', 1020, 40 + 60*(k-1), 1090, 70 + 60*(k-1)), ...
        'VariableName', ['mgs_log_' logs{k}], 'SaveFormat', 'Structure With Time');
end

% ---- wiring ----------------------------------------------------------------
add_line(mdl, 'X in 1/1',        'Seeker Tracker/1', 'autorouting', 'on');
add_line(mdl, 'Seeker Tracker/1','Guidance/1',       'autorouting', 'on');
add_line(mdl, 'Guidance/1',      'Autopilot/1',      'autorouting', 'on');
add_line(mdl, 'X in 2/1',        'Autopilot/2',      'autorouting', 'on');
add_line(mdl, 'Autopilot/1',     'Airframe/2',       'autorouting', 'on');
add_line(mdl, 'X in 2/1',        'Airframe/1',       'autorouting', 'on');
add_line(mdl, 'Airframe/1',      'States/1',         'autorouting', 'on');
add_line(mdl, 'States/1',        'X out/1',          'autorouting', 'on');

add_line(mdl, 'X in 2/1',         'log X/1',   'autorouting', 'on');
add_line(mdl, 'Guidance/1',       'log CMD/1', 'autorouting', 'on');
add_line(mdl, 'Autopilot/1',      'log U/1',   'autorouting', 'on');
add_line(mdl, 'Guidance/2',       'log DBG/1', 'autorouting', 'on');
add_line(mdl, 'Autopilot/2',      'log FL/1',  'autorouting', 'on');
add_line(mdl, 'Airframe/2',       'log AUX/1', 'autorouting', 'on');

% ---- annotation ------------------------------------------------------------
note = sprintf([ ...
    'mgs multi-missile guidance -- generated by mgs.slx.build, DO NOT hand edit\n' ...
    'n = %d vehicles.  Every signal is N x k.\n\n' ...
    'rates:  Seeker/Tracker %g Hz | Guidance %g Hz | Autopilot %g Hz | ' ...
    'Airframe continuous\n\n' ...
    'base workspace dependencies: mgs_c mgs_N mgs_X0 mgs_diveT mgs_tRel\n\n' ...
    'This model is for STRUCTURE.  For numbers that must match the 32-flight\n' ...
    'truth set use mgs.simSwarm -- it reproduces stot_sim bit for bit, and a\n' ...
    'Simulink solver does not.'], ...
    n, c.meas.tel_hz, 1/c.pn.tick_dt, c.fc.loop_hz);
a = Simulink.Annotation([mdl '/note']);
a.Text = note;
a.Position = [260 340 980 470];

% ---- save ------------------------------------------------------------------
here = fileparts(fileparts(fileparts(mfilename('fullpath'))));   % .../matlab/mgs
outDir = fullfile(here, 'models');
if ~isfolder(outDir), mkdir(outDir); end
save_system(mdl, fullfile(outDir, [mdl '.slx']));

if opts.open, open_system(mdl); end
end

% =============================================================================
function add_msblock(mdl, name, cls, pos, props)
%ADD_MSBLOCK  A MATLAB System block, in INTERPRETED execution.
%   Interpreted, not code generation: the underlying functions raise on bad
%   configuration (an unknown tracker mode, an uncalibrated deficit) and those
%   errors are the point.  Code generation would strip them.
blk = [mdl '/' name];
add_block('simulink/User-Defined Functions/MATLAB System', blk, 'Position', pos);
set_param(blk, 'System', cls);
set_param(blk, 'SimulateUsing', 'Interpreted execution');
for k = 1:size(props, 1)
    set_param(blk, props{k, 1}, props{k, 2});
end
end

% -----------------------------------------------------------------------------
function X0 = initial_state(scn, c)
%INITIAL_STATE  Same trim as mgs.simSwarm, so the two start identically.
n = scn.n;
X0 = zeros(n, 10);
for i = 1:n
    ic = scn.ic(i);
    th = deg2rad(ic.theta_deg);
    n0 = ic.r0*cos(th);  e0 = ic.r0*sin(th);
    gam0 = deg2rad(ic.gam0_deg);
    phi0 = deg2rad(ic.phi0_deg);
    psi0 = atan2(-e0, -n0) + deg2rad(ic.eps0_deg);
    rho0 = 1.225*(1 - 2.25577e-5*max(ic.z0, 0))^4.2586;
    qS0  = 0.5*rho0*ic.V0^2*c.b.S;
    nL0  = cos(gam0)/max(cos(phi0), 0.2);
    CL0  = min(c.b.CLmax, max(c.b.CLmin, nL0*c.b.W/max(qS0, 1e-6)));
    th0  = gam0 + (CL0/c.b.CLa)*cos(phi0) + deg2rad(ic.th0_offset_deg);
    X0(i, :) = [n0, e0, ic.z0, ic.V0, gam0, psi0, th0, 0, phi0, 0];
end
end
