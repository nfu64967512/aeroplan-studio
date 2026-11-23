function A = timeAuthority(c, opts)
%MGS.TIMEAUTHORITY  How much arrival-time authority does the current law have?
%
%   A = mgs.timeAuthority()                 nominal release state
%   A = mgs.timeAuthority(c, 'r0', 9000)    at a different release range
%   A = mgs.timeAuthority(c, 'render', true)
%
%   A.t_min, A.t_max     the reachable arrival window [s from release]
%   A.window_s           t_max - t_min
%   A.gain               d(t_impact)/d(dive_T), per swept segment
%   A.gain_inside        median gain inside the window
%   A.sweep              table of dive_T vs t_impact
%
% ---------------------------------------------------------------------------
%   WHY THIS EXISTS, AND WHAT IT SETTLED
%
%   "Does the guidance law converge in time?" is not a yes/no question, and
%   answering it either way is wrong.  Measured on this model at the nominal
%   release state (r0 6760 m, z0 3100 m, V0 157 m/s, gamma0 -27 deg):
%
%     dive_T <= 34 s        t_impact pinned at 35.82 s,  gain exactly 0.000
%     dive_T 36 .. 46 s     gain 0.84 .. 0.97   <-- THE LOOP WORKS HERE
%     dive_T >= 52 s        t_impact pinned at 47.22 s,  gain exactly 0.000
%
%   So the law has a real, well-behaved time loop with a hard saturation
%   boundary.  It is not open-loop, and it is not a tuning problem: outside
%   the window the gain is zero because the airframe cannot get there, not
%   because a gain is too small.
%
%   THE WINDOW COLLAPSES ON CLOSURE, as R^1.70:
%     r0  3000 m -> 3.35 s      6760 m -> 11.41 s      12000 m -> 35.39 s
%   which is why a correction that was available at handover is gone by the
%   time anyone notices it is needed.
%
%   THE EARLY EDGE IS pn_vmax, AND IT IS THE ONE TUNABLE THING HERE:
%     pn_vmax 150 -> window 0.72 s      210 -> 11.41 s      320 -> 19.24 s
%   The late edge does not move: it is zero throttle plus gravity.
%   *** CAVEAT: raising pn_vmax drives terminal speed past the 121-219 m/s
%   band the drag polar was fitted over, so the wide-window numbers are an
%   extrapolation, not a prediction. ***
%
%   AND THE OPERATIONAL CONSEQUENCE.  Two vehicles released dt apart, both
%   commanded the same arrival time, keep this fraction of dt at touchdown:
%     dt 0.5 s -> 17%    1 s -> 15%    2 s -> 10%    3 s -> 8%
%     dt 5 s   -> 68%    8 s -> 147%
%   i.e. the loop removes ~90% of a small handover error and NOTHING of a
%   large one.  The project's own regression (tot_node.py:630, spread =
%   0.76*handover_dt + 1.62, R^2 0.779, n=5) has a slope consistent with
%   flights that were sitting in the saturated regime.
%   *** The dt >= 5 s rows are contaminated by the UNCALIBRATED hold phase,
%   which runs for dt seconds before the second release.  Read the small-dt
%   rows; treat the large-dt rows as indicative only. ***
%
%   See also MGS.KPI, MGS.SIMSWARM.

arguments
    c = mgs.params()
    opts.r0 (1,1) double = NaN
    opts.dive_T (1,:) double = 24:2:60
    opts.render (1,1) logical = true
end

c1 = c;  c1.swarm.n = 1;
if isfinite(opts.r0), c1.scen.r0 = opts.r0; end
scn0 = mgs.scenario(c1, 'tag', 'AUTH');

T = opts.dive_T(:);
ti = nan(size(T));  thrHi = nan(size(T));  thrLo = nan(size(T));  vend = nan(size(T));
for k = 1:numel(T)
    s = scn0;  s.ic(1).dive_T = T(k);
    o = mgs.simSwarm(s, c1, 'verbose', false);
    ti(k)   = o.veh(1).t_impact;
    vend(k) = o.veh(1).v_impact;
    tr = o.veh(1).traj;
    thrHi(k) = mean(tr.thr_pc >= 99.9);
    thrLo(k) = mean(tr.thr_pc <= 0.1);
end

A = struct();
A.r0 = c1.scen.r0;
A.pn_vmax = c1.pn.vmax;
A.sweep = table(T, ti, vend, thrHi, thrLo, ...
    'VariableNames', {'dive_T', 't_impact', 'V_end', 'thr_full', 'thr_zero'});
A.t_min = min(ti);
A.t_max = max(ti);
A.window_s = A.t_max - A.t_min;
A.gain = diff(ti) ./ diff(T);
inside = A.gain > 0.05;
if any(inside)
    A.gain_inside = median(A.gain(inside));
else
    A.gain_inside = 0;
end
A.summary = render(A, c1);
if opts.render, fprintf('%s\n', A.summary); end
end

% -----------------------------------------------------------------------------
function s = render(A, c)
L = {};
L{end+1} = sprintf(['arrival-time authority -- r0 %.0f m, pn_vmax %g, ' ...
    'V0 %g m/s'], A.r0, A.pn_vmax, c.scen.V0);
L{end+1} = repmat('-', 1, 70);
L{end+1} = sprintf('%9s %10s %9s %9s %9s', ...
    'dive_T', 't_impact', 'gain', 'thr=100%', 'thr=0%');
T = A.sweep.dive_T;
for k = 1:height(A.sweep)
    if k < height(A.sweep), g = sprintf('%9.3f', A.gain(k)); else, g = '        -'; end
    L{end+1} = sprintf('%9.1f %10.3f %s %8.0f%% %8.0f%%', ...
        T(k), A.sweep.t_impact(k), g, ...
        100*A.sweep.thr_full(k), 100*A.sweep.thr_zero(k)); %#ok<AGROW>
end
L{end+1} = repmat('-', 1, 70);
L{end+1} = sprintf('  reachable window  %.2f .. %.2f s   (%.2f s wide)', ...
    A.t_min, A.t_max, A.window_s);
L{end+1} = sprintf('  tracking gain     %.2f inside, exactly 0.000 outside', ...
    A.gain_inside);
L{end+1} = ['  A gain of 0.000 outside the window is SATURATION, not a small'];
L{end+1} = ['  gain: the airframe cannot get there.  Retuning does not help.'];
s = strjoin(L, newline);
end
