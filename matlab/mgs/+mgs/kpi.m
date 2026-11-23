function k = kpi(out, opts)
%MGS.KPI  Acceptance metrics for a swarm run.
%
%   k = mgs.kpi(out)
%   s = mgs.kpi(out, 'render', true)     printable block
%
%   THE ACCEPTANCE THRESHOLDS, as set for this programme:
%       1  launch interval        3 s
%       2  formation forms
%       3  hit error              <= 10 m
%       4  touchdown spread       <= 3 s   ... over EIGHT aircraft, not sixteen
%       5  cruise speed           157 m/s   GROUND SPEED at 3100 m (= EAS 135)
%       6  terminal speed         >= 204 m/s   GROUND SPEED
%
%   *** 5 AND 6 ARE GROUND SPEEDS, AND THAT IS NOT A DETAIL. ***
%   157 is the ground speed at 3100 m; the corresponding EAS is 135.  Feeding
%   157 in as an EAS overshoots the thrust ceiling by 17 % and turns the STOT
%   synchroniser open-loop.  204 is likewise a ground speed -- and the design
%   thrust ceiling is only 182 m/s, so it is not automatically reachable.
%
%   k.v_impact is the TOTAL speed and k.gs_impact is the GROUND speed.  Only
%   gs_impact is graded against 204.  The field pipeline once silently
%   degraded the total to the ground speed in 12 of 16 sorties (a dt = 0 made
%   vz read zero), which is why the two are never merged here.
%
%   and one ordering that overrides all of them: per-vehicle touchdown
%   RELIABILITY comes before spread.  Spread computed over a population that
%   half misses is not a number about spread.  k.spread_valid is false unless
%   the hit rate clears k.reliability_gate (0.90 by default).
%
%   When spread and terminal speed are in tension, spread wins.
%
% ---------------------------------------------------------------------------
%   k.hit_rate           fraction meeting BOTH the horizontal and the altitude
%                        test.  A horizontal-only "hit" is not a hit: a 6/6
%                        "all hit" once turned out to be six high passes.
%   k.err_m              per-vehicle horizontal error at touchdown
%   k.z_res_m            RESIDUAL ALTITUDE at closest passage.  On the subset
%                        where the LOS floor is binding >= 85 % of the time,
%                        err ~ 1.12 * residual (R^2 0.97).  On vehicles that
%                        are really doing PN, that relation has no power
%                        (rho = -0.26) -- do not apply it blind.
%   k.spread_s           max - min of touchdown time
%   k.spread_release_s   the part already present at release
%   k.spread_dive_s      the part accumulated during the dive
%                        *** This decomposition is the only thing that says
%                        whether to fix upstream or downstream. ***
%   k.v_impact / k.gs_impact   total and ground speed, separately
%
%   THE TIME LOOP IS REAL BUT SATURATION-BOUNDED.  Two separate statements,
%   and conflating them (as an earlier version of this comment did) gets the
%   conclusion backwards:
%
%   1. The PN channel itself carries NO time information.  Under pure PN
%      against a stationary target, d(t_go)/dt = -1 exactly to O(sigma^2), so
%      the time-to-go error derivative is identically zero and a release-time
%      difference passes straight through.  Measured here: with no deadline
%      commanded at all, a 1-5 s release offset comes out at touchdown
%      AMPLIFIED to 176-219% of itself.
%
%      *** SOURCING -- RESTORED 2026-08-22 after the source was located. ***
%      Guo, Li, Zhang, Cai and He, "Data-Driven Method for Impact Time
%      Control Based on Proportional Navigation Guidance" (Beijing Inst. of
%      Technology).  Eq. (31) t_f = t + t_go^PNG, Eq. (32) d/dt t_go^PNG =
%      -1, hence Eq. (33) d/dt e_tgo = 0.  Verbatim: "if only PNG is
%      adopted, the etgo keeps unchanged; that is, the etgo at the beginning
%      of PNG trajectory is the same as the etgo at the end of the PNG
%      trajectory; etgo cannot be eliminated."
%
%      An earlier version of this comment called the citation unsourced
%      because the PDF is not in the repository.  That was checking the
%      wrong shelf: it is source c3644548-7fb0-4d9e-a336-59e7b10cdb51
%      (guo2020.pdf, 55813 chars) of the STOT-plan notebook.  The result is
%      also derivable in three lines, which is why the derivation is kept
%      above -- but it is NOT ours, and the attribution belongs here.
%
%   2. Our law is NOT pure PN: it has a throttle time channel
%      (v_req = R/t_rem, tot_node.py:3168 calls it "the only impact-time
%      channel").  That channel WORKS, inside a window bounded by saturation.
%      Measured on this model at the nominal release state (r0 6760, V0 157):
%        reachable arrival window  35.82 .. 47.22 s
%        tracking gain inside it   d(t_impact)/d(dive_T) = 0.84 .. 0.97
%        tracking gain outside it  exactly 0.000
%        window scaling            as R^1.70, so authority collapses on closure
%        early edge set by pn_vmax 150 -> 0.72 s window;  210 -> 11.41 s
%
%   So k.spread_release_s exceeding the budget is recoverable IF the required
%   correction lands inside that window, and not otherwise.  k.notes says
%   which case you are in.
%
%   See also MGS.SIMSWARM.

arguments
    out
    opts.render (1,1) logical = false
    opts.err_max_m (1,1) double = 10.0
    opts.spread_max_s (1,1) double = 3.0
    opts.reliability_gate (1,1) double = 0.90
    opts.gs_min (1,1) double = 204.0      % terminal GROUND speed spec
    opts.v_calib (1,2) double = [121 219] % band the drag polar was fitted over
end

v = out.veh;
n = numel(v);

k = struct();
k.n = n;
k.tag = out.tag;
k.err_m     = [v.err_m];
k.z_res_m   = [v.z_res_m];
k.t_impact  = [v.t_impact];
k.v_impact  = [v.v_impact];
k.gs_impact = [v.gs_impact];
k.hit       = logical([v.hit]);
k.end_reason = {v.end_reason};

k.hit_rate = mean(k.hit);
k.err_median = median(k.err_m(k.hit), 'omitnan');
k.err_max    = max(k.err_m(k.hit));
if isempty(k.err_max), k.err_max = NaN; end
k.cep50 = prctile_safe(k.err_m, 50);

% ---- spread, and its decomposition ----------------------------------------
tt = k.t_impact(k.hit);
if numel(tt) >= 2
    k.spread_s = max(tt) - min(tt);
else
    k.spread_s = NaN;
end

t_rel = arrayfun(@(s) s.t_release, out.scn.ic);
sel = k.hit;
if numel(t_rel) == n && sum(sel) >= 2
    k.spread_release_s = max(t_rel(sel)) - min(t_rel(sel));
    dive_dur = k.t_impact(sel) - t_rel(sel);
    k.spread_dive_s = max(dive_dur) - min(dive_dur);
else
    k.spread_release_s = NaN;
    k.spread_dive_s = NaN;
end

k.reliability_gate = opts.reliability_gate;
k.spread_valid = k.hit_rate >= opts.reliability_gate && numel(tt) >= 2;

% ---- terminal GROUND speed, criterion 6 ------------------------------------
k.gs_min_spec = opts.gs_min;
k.gs_median = median(k.gs_impact, 'omitnan');
k.gs_worst  = min(k.gs_impact);
k.n_gs_ok   = sum(k.gs_impact >= opts.gs_min);
k.pass_gs   = k.n_gs_ok == k.n;
% A pass bought by running the airframe outside the band its drag polar was
% fitted over is an extrapolation, not a result.  Say which one it is.
k.gs_extrapolated = any(k.v_impact > opts.v_calib(2)) || ...
                    any(k.v_impact < opts.v_calib(1));

% ---- verdicts --------------------------------------------------------------
k.pass_err    = ~isnan(k.err_max) && k.err_max <= opts.err_max_m;
k.pass_spread = k.spread_valid && k.spread_s <= opts.spread_max_s;
k.pass = k.pass_err && k.pass_spread && k.pass_gs;

k.notes = {};
if ~k.spread_valid
    k.notes{end+1} = sprintf(['spread is NOT valid: hit rate %.0f%% is below ' ...
        'the %.0f%% reliability gate.  Fix touchdown reliability first.'], ...
        100*k.hit_rate, 100*opts.reliability_gate);
end
if ~isnan(k.spread_release_s) && k.spread_release_s > opts.spread_max_s
    k.notes{end+1} = sprintf(['%.2f s of the spread is ALREADY PRESENT at ' ...
        'release.  The throttle time channel can absorb a release offset ' ...
        'only while the correction stays inside the reachable arrival ' ...
        'window (measured 35.82-47.22 s at the nominal release state, and ' ...
        'shrinking as R^1.70).  Past that its gain is exactly 0.000 and the ' ...
        'offset passes through, so the fix is upstream.'], k.spread_release_s);
end
if ~isnan(k.spread_dive_s) && ~isnan(k.spread_release_s) && ...
        k.spread_dive_s > k.spread_release_s
    k.notes{end+1} = sprintf(['the dive contributes more spread (%.2f s) ' ...
        'than release does (%.2f s) -- the fix is downstream.'], ...
        k.spread_dive_s, k.spread_release_s);
end
if ~k.pass_gs
    k.notes{end+1} = sprintf(['terminal GROUND speed %.1f m/s median ' ...
        '(worst %.1f) is below the %.0f m/s spec -- %d of %d aircraft made ' ...
        'it.  The lever is pn_vmax, not the dive angle: measured here, ' ...
        'pn_vmax 210 -> 193 m/s and 260 -> 231, while sweeping the dive ' ...
        'angle 27-55 deg moves it only +3 m/s.'], ...
        k.gs_median, k.gs_worst, opts.gs_min, k.n_gs_ok, k.n);
end
if k.gs_extrapolated
    k.notes{end+1} = sprintf(['terminal TOTAL speed left the %g-%g m/s band ' ...
        'the drag polar was fitted over -- these speeds are an ' ...
        'extrapolation, not a prediction.'], opts.v_calib(1), opts.v_calib(2));
end
if n < 8
    k.notes{end+1} = sprintf(['n = %d.  The 3 s threshold is defined over ' ...
        'EIGHT aircraft; a spread measured on %d is not that number.'], n, n);
end
if isfield(out, 'calibrated') && ~out.calibrated
    k.notes{end+1} = 'run is UNCALIBRATED (see out.notes)';
end

if opts.render
    k = render(k, opts);
end
end

% -----------------------------------------------------------------------------
function s = render(k, opts)
L = {};
L{end+1} = sprintf('== %s : n = %d ==', k.tag, k.n);
L{end+1} = sprintf('  hit          %d/%d  (%.0f%%)   [horizontal <= %g m AND |z| <= 1 m]', ...
    sum(k.hit), k.n, 100*k.hit_rate, opts.err_max_m);
L{end+1} = sprintf('  error        median %.2f m, max %.2f m', ...
    k.err_median, k.err_max);
L{end+1} = sprintf('  z residual   %s m', vec(k.z_res_m));
L{end+1} = sprintf('  touchdown    %s s', vec(k.t_impact));
if k.spread_valid
    L{end+1} = sprintf('  spread       %.2f s   (release %.2f + dive %.2f)', ...
        k.spread_s, k.spread_release_s, k.spread_dive_s);
else
    L{end+1} = '  spread       -- not valid, see notes';
end
L{end+1} = sprintf('  impact speed total  %s m/s', vec(k.v_impact));
L{end+1} = sprintf('               GROUND %s m/s   <- this is the one graded', ...
    vec(k.gs_impact));
L{end+1} = sprintf('               spec >= %.0f m/s ground: %d/%d', ...
    k.gs_min_spec, k.n_gs_ok, k.n);
L{end+1} = sprintf('  end reason   %s', strjoin(k.end_reason, ', '));
L{end+1} = sprintf('  VERDICT      err %s   spread %s   term gs %s', ...
    yn(k.pass_err), yn(k.pass_spread), yn(k.pass_gs));
for i = 1:numel(k.notes)
    L{end+1} = sprintf('  ! %s', k.notes{i}); %#ok<AGROW>
end
s = strjoin(L, newline);
end

function s = yn(tf)
if tf, s = 'PASS'; else, s = 'FAIL'; end
end

function s = vec(v)
s = ['[' strjoin(arrayfun(@(x) sprintf('%.2f', x), v, ...
    'UniformOutput', false), ' ') ']'];
end

function p = prctile_safe(v, q)
v = sort(v(~isnan(v)));
if isempty(v), p = NaN; return; end
if numel(v) == 1, p = v; return; end
idx = 1 + (q/100)*(numel(v)-1);
lo = floor(idx); hi = ceil(idx);
p = v(lo) + (idx-lo)*(v(hi)-v(lo));
end
