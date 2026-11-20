function D = stot_gate_diag()
%STOT_GATE_DIAG  Why the gate failed: the model flies a beam, not a dive.
%
%   Reads matlab/out/gate.mat (written by stot_gate) and asks one question:
%   is the simulated flight-path angle simply the release LOS depression angle
%   held constant?  If it is, pn_los_floor = 1.0 has turned the model into a
%   straight-line beam rider, every release state converges on the bullseye by
%   construction, and no configuration change can produce a miss -- which is
%   exactly the symptom the gate detected.

L = load(fullfile(fileparts(mfilename('fullpath')),'out','gate.mat'));
G = L.G; S = G.S; M = G.M; D = struct();
p = @(varargin) fprintf(varargin{:});

losdep = -atan2d(M.z_rel_m, M.r_rel_m);
D.losdep = losdep;

p('\n--- BEAM-RIDER TEST: is sim terminal gamma just the release LOS depression? ---\n');
p('  sim  gam_touch vs release LOS depression: pearson %+.4f, median |diff| %.3f deg\n', ...
    corr_(S.gam_touch_deg, losdep), median(abs(S.gam_touch_deg - losdep)));
p('  meas gam_touch vs release LOS depression: pearson %+.4f, median |diff| %.3f deg\n', ...
    corr_(M.row_gamma_touch_deg, losdep), median(abs(M.row_gamma_touch_deg - losdep)));
p('  LOS-floor binding fraction, median over 32 = %.1f %%\n', 100*median(S.los_frac));
p('  measured 3-D path length / straight-line slant range, median = %.4f\n', ...
    median(M.path_len_m ./ M.R_slant_rel_m));

p('\n--- the spread the model does and does not produce ---\n');
p('  quantity          sim min     sim med     sim p90     sim max  |  meas min  meas med  meas p90  meas max\n');
p('  err_touch [m]  %10.4f  %10.4f  %10.4f  %10.4f  | %9.3f %9.3f %9.3f %9.2f\n', ...
    min(S.err_touch_m), median(S.err_touch_m), pctl(S.err_touch_m,90), max(S.err_touch_m), ...
    min(M.err_touch_m), median(M.err_touch_m), pctl(M.err_touch_m,90), max(M.err_touch_m));
p('  z_res     [m]  %10.4f  %10.4f  %10.4f  %10.4f  | %9.3f %9.3f %9.3f %9.2f\n', ...
    min(S.z_res_m), median(S.z_res_m), pctl(S.z_res_m,90), max(S.z_res_m), ...
    min(M.z_res_m), median(M.z_res_m), pctl(M.z_res_m,90), max(M.z_res_m));
p('  sd:            sim %.4f m vs meas %.4f m  (ratio 1 : %.0f)\n', ...
    std(S.z_res_m), std(M.z_res_m), std(M.z_res_m)/max(std(S.z_res_m),1e-12));

NC = G.NC;
p('\n--- how little the negative controls move anything (32-flight medians) ---\n');
p('  %-10s %10s %10s %10s %10s %10s\n','config','err med','err p90','t_dive','V_touch','los_frac');
p('  %-10s %10.4f %10.4f %10.3f %10.2f %9.1f%%\n','baseline', ...
    median(S.err_touch_m), pctl(S.err_touch_m,90), median(S.t_dive_s), ...
    median(S.V_touch_ms), 100*median(S.los_frac));
for k = 1:numel(NC)
    Sk = NC(k).S;
    p('  %-10s %10.4f %10.4f %10.3f %10.2f %9.1f%%\n', NC(k).name, ...
        median(Sk.err_touch_m), pctl(Sk.err_touch_m,90), median(Sk.t_dive_s), ...
        median(Sk.V_touch_ms), 100*median(Sk.los_frac));
end
D.G = G;
end

function r = corr_(x, y)
m = isfinite(x) & isfinite(y);
x = x(m) - mean(x(m)); y = y(m) - mean(y(m));
r = sum(x.*y)/sqrt(sum(x.^2)*sum(y.^2));
end
function v = pctl(x, q)
x = sort(x(isfinite(x))); n = numel(x);
i = (q/100)*(n-1) + 1; lo = floor(i); hi = ceil(i); f = i - lo;
v = x(lo)*(1-f) + x(hi)*f;
end
