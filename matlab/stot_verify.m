function V = stot_verify()
%STOT_VERIFY  Everything, in one call, including the tests this model FAILS.
%
%   V = stot_verify()
%
%   Runs, in order:
%     1  stot_selftest      internal consistency (order, dt invariance, clamps)
%     2  stot_run32         32 measured release states vs truth32.csv
%     3  negative control   N1 = pn_los_floor 0 + pn_nz_min +0.05
%     4  stot_lagcheck      measured vs modelled gamma-tracking lag
%
%   It prints a verdict per gate quantity.  Read the FAIL lines: this model does
%   not yet reproduce the miss population, and pretending otherwise by moving a
%   prior would be the exact failure mode the negative control exists to catch.

V = struct();
fprintf('\n############ 1. SELFTEST ############\n');
V.selftest = stot_selftest();

fprintf('\n############ 2. FIDELITY, RRE CONTRACT ############\n');
[S, M, F] = stot_run32('label','verify');
V.S = S; V.M = M; V.F = F;

fprintf('\n############ 3. NEGATIVE CONTROL N1 ############\n');
% Measured N1 (pn_los_floor = 0, pn_nz_min = +0.05) on the RPQ batch: 0/6 reached
% the water, errors out to 2423 m.  sim3d passed it 32/32 at a 0.90 m median,
% i.e. it called a configuration that destroyed the vehicle BETTER than the one
% that flew.  Any model that does the same has no discriminating power.
cN = cfg.params('pn.los_floor',0.0,'pn.nz_min',0.05);
[SN, ~, FN] = stot_run32('label','N1','cfg',cN,'quiet',true);
V.SN = SN; V.FN = FN;
fprintf('  N1: err median %.2f m, err max %.2f m, reached water %d/32, z_res sd %.3f m\n', ...
    FN.err_med_sim, max(SN.err_touch_m), sum(SN.hit), FN.zres_sd_sim);
fprintf('  baseline (contract): err median %.2f m, err max %.2f m, reached water %d/32\n', ...
    F.err_med_sim, max(S.err_touch_m), sum(S.hit));
n1_separates = (FN.err_med_sim > 10*max(F.err_med_sim, 1e-6)) || (sum(SN.hit) < 32);
fprintf('  does N1 separate from the baseline?  %s\n', yn(n1_separates));
V.n1_separates = n1_separates;

fprintf('\n############ 4. GAMMA TRACKING LAG ############\n');
V.L = stot_lagcheck(false, 8.0);

fprintf('\n############ VERDICT ############\n');
g = {};
g{end+1} = check('dive duration',   abs(F.tdive_bias_med) < 2.0 && F.tdive_mae < 3.0, ...
    sprintf('bias %+.2f s, MAE %.2f s, rho %+.3f', F.tdive_bias_med, F.tdive_mae, F.tdive_spearman));
g{end+1} = check('terminal speed',  abs(F.V_bias_med) < 10, ...
    sprintf('bias %+.1f m/s, rho %+.3f', F.V_bias_med, F.V_spearman));
g{end+1} = check('residual alt spread', F.zres_sd_sim > 0.25*F.zres_sd_meas, ...
    sprintf('sd %.4f m vs measured %.4f m  (sim3d was 0.0002)', F.zres_sd_sim, F.zres_sd_meas));
g{end+1} = check('touchdown error', F.err_med_sim > 0.3*F.err_med_meas && F.err_hit10_sim <= F.err_hit10_meas + 2, ...
    sprintf('median %.2f m vs %.2f m; <=10 m %d/32 vs %d/32', ...
        F.err_med_sim, F.err_med_meas, F.err_hit10_sim, F.err_hit10_meas));
g{end+1} = check('nz clamp fraction', F.nz_spearman > 0.5 && F.nz_frac_sim_med > 0.3*F.nz_frac_meas_med, ...
    sprintf('median %.2f %% vs %.2f %%, rho %+.3f, zeros %d/32 vs %d/32 (sim3d 29/32)', ...
        100*F.nz_frac_sim_med, 100*F.nz_frac_meas_med, F.nz_spearman, ...
        F.nz_zero_count_sim, F.nz_zero_count_meas));
g{end+1} = check('negative control N1', n1_separates, ...
    'N1 must not look as good as, or better than, the configuration that flew');
g{end+1} = check('gamma tracking lag', ...
    median(V.L.lag_sim_s,'omitnan') > 0.5*median(V.L.lag_meas_s,'omitnan'), ...
    sprintf('model %.2f s / vehicle %.2f s; residual %.2f deg vs %.2f deg', ...
        median(V.L.lag_sim_s,'omitnan'), median(V.L.lag_meas_s,'omitnan'), ...
        median(V.L.rms_sim_deg,'omitnan'), median(V.L.rms_meas_deg,'omitnan')));
V.gates = g;
fprintf('\n');
end

function s = check(name, ok, detail)
if ok, v = 'PASS'; else, v = '**FAIL**'; end
fprintf('  %-22s %-8s %s\n', name, v, detail);
s = struct('name',name,'pass',ok,'detail',detail);
end
function s = yn(b)
if b, s = 'yes'; else, s = 'NO  <- same defect sim3d had'; end
end
