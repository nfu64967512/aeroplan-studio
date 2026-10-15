%DEMO_OFFLINE  "If I set it like THIS, how does the model run it?"
%
%   No ROS, no WSL, no live stack.  This is the loop to sit in when you want
%   to know what a configuration does before spending a sortie on it.
%
%   Run:  mgs_setup; demo_offline

mgs_setup();
fprintf('\n=================== 1. the contract snapshot as flown ==========\n');
c = mgs.params('swarm.n', 8);
scn = mgs.scenario(c, 'tag', 'BASE');
base = mgs.simSwarm(scn, c);

fprintf('\n=================== 2. change ONE thing =======================\n');
% pn_los_floor is the switch tot_node.py calls "the only effective one for a
% steep dive".  Turn it off and watch what moves.
c0 = mgs.params('swarm.n', 8, 'pn.los_floor', 0.0);
off = mgs.simSwarm(mgs.scenario(c0, 'tag', 'FLOOR-OFF'), c0);

fprintf('\n=================== 3. sweep it ===============================\n');
floors = [0.0 0.25 0.5 0.75 1.0];
fprintf('%-10s %-6s %-10s %-10s %-10s\n', ...
    'los_floor', 'hits', 'med err m', 'spread s', 'LOS active');
for f = floors
    cf = mgs.params('swarm.n', 8, 'pn.los_floor', f);
    of = mgs.simSwarm(mgs.scenario(cf, 'tag', sprintf('F%.2f', f)), cf, ...
                      'verbose', false);
    k = of.kpi;
    losfrac = mean(arrayfun(@(v) mean(v.traj.los_active, 'omitnan'), of.veh));
    fprintf('%-10.2f %-6d %-10.2f %-10.2f %-9.1f%%\n', ...
        f, sum(k.hit), k.err_median, k.spread_s, 100*losfrac);
end

fprintf(['\nREAD THIS BEFORE ACTING ON THE TABLE.\n' ...
    '  The gamma tracking deficit is OFF in all of the above, so the model\n' ...
    '  tracks a commanded flight path angle far better than the vehicles do\n' ...
    '  (gain 1.385 against a measured 0.063).  It therefore hits every time,\n' ...
    '  and a sweep of hit COUNT here carries very little.  What the sweep is\n' ...
    '  good for is the shape of the other columns -- spread, and how much of\n' ...
    '  the dive the LOS floor was in charge of.\n']);
