%DEMO_LIVE_SHADOW  Run the model alongside the live WSL stack, and stream it.
%
%   Prerequisite, inside WSL:
%       source /opt/ros/humble/setup.bash
%       python3 matlab/mgs/bridge/mgs_bridge_node.py --port 5799 --n 6
%
%   Then, here:   mgs_setup; demo_live_shadow
%
%   Nothing in this demo commands an aircraft.  c.io.mode = 'shadow' makes
%   sendCmd raise, and the bridge refuses to forward a target unless it was
%   started with --allow-target.  Both of those are deliberate: the model
%   running next to the vehicles and the model flying the vehicles are
%   different activities and should not be one keystroke apart.

mgs_setup();

% ---- 1. check everything the link depends on, before depending on it ------
rep = mgs.preflight();
if ~rep.pass
    error('demo:preflight', ...
        ['preflight found %d hard failures -- fix them first.  Running ' ...
         'anyway would give you a model that looks live and is not.'], ...
        rep.hardFail);
end

% ---- 2. connect -----------------------------------------------------------
c = mgs.params('swarm.n', 4, 'io.mode', 'shadow');
b = mgs.Bridge(c);
b.connect();
fprintf('link up: schema %d, ROS %s, rtt %.0f ms\n', ...
    b.ack.schema, b.ack.ros_distro, 1000*b.ping());

% ---- 3. take the LIVE effective values and run the model on them ----------
% This is the part that makes the comparison mean something.  The parameters
% come off the running node and flight controller by readback -- not off the
% launch command, and not off this file's defaults.
fprintf('\nreading the live contract back...\n');
[values, raw] = b.contract();
if isempty(fieldnames(values))
    warning(['the live stack returned no effective values -- the ROS 2 nodes ' ...
        'are probably not up.  Continuing on mgs.params() defaults, and ' ...
        'SAYING SO: any comparison below is model-vs-model, not model-vs-vehicle.']);
else
    c = mgs.fromContract(values, 'strict', false, 'base', c);
    fprintf('model is now running the values the aircraft are running.\n');
end

% ---- 4. stream the model into /mgs/shadow while it flies ------------------
scn = mgs.scenario(c, 'tag', 'SHADOW');
out = mgs.simSwarm(scn, c, 'hook', b.simHook(), 'realtime', false);

fprintf('\nstreamed %d guidance ticks to /mgs/shadow (tx %d, errors %d)\n', ...
    numel(out.t), b.stats.tx, b.stats.errors);
fprintf('watch it from WSL with:\n    ros2 topic echo /mgs/shadow --field data\n');

% ---- 5. what the live stack thinks it is doing ----------------------------
b.poll();
st = b.latest('status', 10);
if isempty(st)
    fprintf('\nno /swarm/tot_status seen -- tot_node is not publishing.\n');
else
    fprintf('\nlive tot_status: %s\n', jsonencode(st.data));
end

b.close();
