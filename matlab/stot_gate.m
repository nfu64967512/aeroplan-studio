function G = stot_gate()
%STOT_GATE  Task G: run the MATLAB model through the SAME gate that killed
%           scripts/research/ditcg/sim3d.py.  Verbatim rules, not relaxed.
%
%   G = stot_gate()
%
%   1  forward reproduction, 32 airframes, RRE contract configuration
%   2  negative controls  N1 (los_floor 0 + nz_min +0.05), N2 (n_vert 3),
%                         N3 (n_vert 7)
%   3  the pre-registered gate criteria (a)..(e)
%   4  dt sensitivity  h = 0.02 / 0.01 / 0.005
%
%   NOTHING in this file adjusts a model parameter.  If a criterion fails it is
%   reported as a failure.  The gate is the point of the exercise; moving it
%   would destroy the only meaningful comparison against sim3d.

t_all = tic;
G = struct();
p = @(varargin) fprintf(varargin{:});

% ======================================================================== 1
p('\n================ 1. FORWARD REPRODUCTION (32 flights, RRE contract) ================\n');
c0 = cfg.params();
[S, M, F] = stot_run32('label','gate_base','cfg',c0,'quiet',true);
G.S = S; G.M = M; G.F = F;

p('  quantity                          measured        simulated     rho(sim,meas)\n');
p('  ----------------------------------------------------------------------------\n');
p('  (a) nz clamp frac  median [%%]     %8.3f        %8.3f        %+.3f\n', ...
    100*F.nz_frac_meas_med, 100*F.nz_frac_sim_med, F.nz_spearman);
p('      nz clamp frac  range  [%%]  %6.2f-%6.2f   %6.2f-%6.2f\n', ...
    100*F.nz_frac_meas_min, 100*F.nz_frac_meas_max, 100*F.nz_frac_sim_min, 100*F.nz_frac_sim_max);
p('      nz clamp frac  == 0            %3d/32          %3d/32          (sim3d 29/32)\n', ...
    F.nz_zero_count_meas, F.nz_zero_count_sim);
p('  (b) residual alt   median [m]     %8.3f        %8.3f        %+.3f\n', ...
    F.zres_med_meas, F.zres_med_sim, F.zres_spearman);
p('  (e) residual alt   sd     [m]     %8.4f        %8.4f\n', F.zres_sd_meas, F.zres_sd_sim);
p('  (c) touchdown err  median [m]     %8.3f        %8.3f        %+.3f\n', ...
    F.err_med_meas, F.err_med_sim, F.err_spearman);
p('      touchdown err  p90    [m]     %8.3f        %8.3f\n', ...
    pct(M.err_touch_m,90), pct(S.err_touch_m,90));
p('      touchdown err  <=10 m          %3d/32          %3d/32\n', F.err_hit10_meas, F.err_hit10_sim);
p('  (d) dive duration  median [s]     %8.2f        %8.2f        %+.3f\n', ...
    F.tdive_sim_med_meas, F.tdive_med_sim, F.tdive_spearman);
p('      dive duration  bias / MAE [s]                  %+7.2f / %.2f\n', F.tdive_bias_med, F.tdive_mae);
p('  (e) terminal gamma median [deg]   %8.2f        %8.2f        %+.3f   DIAGNOSTIC ONLY\n', ...
    median(M.row_gamma_touch_deg,'omitnan'), median(S.gam_touch_deg,'omitnan'), ...
    spear(S.gam_touch_deg, M.row_gamma_touch_deg));
p('      terminal speed median [m/s]   %8.1f        %8.1f        %+.3f\n', ...
    F.V_med_meas, F.V_med_sim, F.V_spearman);

% per-flight residual-altitude absolute error, the (b) statistic
zres_abs = abs(S.z_res_m - M.z_res_m);
G.zres_medabs = median(zres_abs,'omitnan');
p('\n      (b) statistic: median |z_res_sim - z_res_meas| = %.3f m   (sim3d: 1.70)\n', G.zres_medabs);

% ======================================================================== 2
p('\n================ 2. NEGATIVE CONTROLS ================\n');
NC = struct('name',{},'cfgstr',{},'measured',{},'S',{},'F',{});

cN1 = cfg.params('pn.los_floor',0.0,'pn.nz_min',0.05);
cN2 = cfg.params('pn.n_vert',3.0);
cN3 = cfg.params('pn.n_vert',7.0);
specs = { 'N1', 'los_floor=0.0, nz_min=+0.05', 'RPQ 0/6 reached water, errors to 2423 m', cN1
          'N2', 'n_vert=3',                    'errors 2.64-5.35 m (baseline 0.84-2.64)', cN2
          'N3', 'n_vert=7',                    'p90 error 4.14 -> 6.52 m (worse)',        cN3 };
for k = 1:size(specs,1)
    [Sk, ~, Fk] = stot_run32('label',['gate_' specs{k,1}],'cfg',specs{k,4},'quiet',true);
    NC(k).name = specs{k,1}; NC(k).cfgstr = specs{k,2}; NC(k).measured = specs{k,3};
    NC(k).S = Sk; NC(k).F = Fk;
end
G.NC = NC;

p('  config    override                          err med   err p90    err max   <=10m   water   z_res sd\n');
p('  ------------------------------------------------------------------------------------------------\n');
p('  %-8s  %-32s %8.3f  %8.3f  %9.3f   %2d/32   %2d/32   %8.4f\n', ...
    'baseline', 'los_floor=1.0 nz_min=-1.0 n_vert=5', ...
    F.err_med_sim, pct(S.err_touch_m,90), max(S.err_touch_m), F.err_hit10_sim, sum(S.hit), F.zres_sd_sim);
for k = 1:numel(NC)
    Sk = NC(k).S; Fk = NC(k).F;
    p('  %-8s  %-32s %8.3f  %8.3f  %9.3f   %2d/32   %2d/32   %8.4f\n', ...
        NC(k).name, NC(k).cfgstr, Fk.err_med_sim, pct(Sk.err_touch_m,90), ...
        max(Sk.err_touch_m), Fk.err_hit10_sim, sum(Sk.hit), Fk.zres_sd_sim);
end
p('\n  what the vehicle actually did:\n');
for k = 1:numel(NC), p('    %s  %s\n', NC(k).name, NC(k).measured); end

% ======================================================================== 3
p('\n================ 3. PRE-REGISTERED GATE ================\n');
gates = {};

% (a) spearman(sim nz clamp frac, measured) >= 0.60         sim3d: +0.359 FAIL
gates{end+1} = g_('(a) nz clamp rank corr', F.nz_spearman >= 0.60, ...
    sprintf('rho = %+.3f  (need >= +0.600; sim3d was +0.359)', F.nz_spearman));

% (b) median absolute residual-altitude error < 10 m        sim3d: 1.70 (hollow)
gates{end+1} = g_('(b) residual alt med|err|', G.zres_medabs < 10.0, ...
    sprintf('%.3f m  (need < 10 m; sim3d 1.70 -- see (e), that PASS was hollow)', G.zres_medabs));

% (c) N1 must break                                          sim3d: 32/32 clean FAIL
S1 = NC(1).S; F1 = NC(1).F;
n1_ratio = F1.err_med_sim / max(F.err_med_sim, 1e-9);
n1_broke = (sum(S1.hit) <= 16) || (F1.err_med_sim >= 100.0) || (n1_ratio >= 10.0);
gates{end+1} = g_('(c) N1 must break', n1_broke, ...
    sprintf(['err med %.3f m (x%.2f baseline), max %.1f m, water %d/32  ' ...
             '(need water<=16/32 OR med>=100 m OR >=10x baseline)'], ...
            F1.err_med_sim, n1_ratio, max(S1.err_touch_m), sum(S1.hit)));

% (d) N2 and N3 must BOTH be worse than baseline
S2 = NC(2).S; F2 = NC(2).F;  S3 = NC(3).S; F3 = NC(3).F; %#ok<NASGU>
n2_worse = F2.err_med_sim > F.err_med_sim;
n3_worse = pct(S3.err_touch_m,90) > pct(S.err_touch_m,90);
gates{end+1} = g_('(d) N2 worse (n_vert=3)', n2_worse, ...
    sprintf('err med %.4f vs baseline %.4f m  (also p90 %.4f vs %.4f)', ...
            F2.err_med_sim, F.err_med_sim, pct(S2.err_touch_m,90), pct(S.err_touch_m,90)));
gates{end+1} = g_('(d) N3 worse (n_vert=7)', n3_worse, ...
    sprintf('err p90 %.4f vs baseline %.4f m  (also med %.4f vs %.4f)', ...
            pct(S3.err_touch_m,90), pct(S.err_touch_m,90), F3.err_med_sim, F.err_med_sim));

% (e) discriminating power: sim z_res sd >= measured sd / 3
thr_e = 11.773/3;                      % the literal threshold in the task brief
gates{end+1} = g_('(e) z_res sd discrim', F.zres_sd_sim >= thr_e, ...
    sprintf('sim sd %.4f m  (need >= %.3f = 11.773/3; this run measures %.4f => %.3f)  sim3d 0.0002', ...
            F.zres_sd_sim, thr_e, F.zres_sd_meas, F.zres_sd_meas/3));

G.gates = gates;

% ======================================================================== 4
p('\n================ 4. dt SENSITIVITY (the sim3d r_end bug) ================\n');
hs = [0.02 0.01 0.005];
dt = struct('h',hs,'err_med',nan(size(hs)),'err_mean',nan(size(hs)), ...
            'tdive_med',nan(size(hs)),'zres_sd',nan(size(hs)));
for k = 1:numel(hs)
    ck = cfg.params('sim.h_max',hs(k));
    [Sk, ~, Fk] = stot_run32('label',sprintf('gate_h%g',hs(k)),'cfg',ck,'quiet',true,'save',false);
    dt.err_med(k)   = Fk.err_med_sim;
    dt.err_mean(k)  = mean(Sk.err_touch_m,'omitnan');
    dt.tdive_med(k) = Fk.tdive_med_sim;
    dt.zres_sd(k)   = Fk.zres_sd_sim;
end
G.dt = dt;
p('     h [s]      err median [m]    err mean [m]   t_dive med [s]   z_res sd [m]\n');
for k = 1:numel(hs)
    p('   %8.4f    %14.8f  %14.8f %16.8f %14.8f\n', ...
        hs(k), dt.err_med(k), dt.err_mean(k), dt.tdive_med(k), dt.zres_sd(k));
end
r1 = dt.err_med(2)/max(dt.err_med(1),1e-12);
r2 = dt.err_med(3)/max(dt.err_med(2),1e-12);
p('\n   ratio on halving h:  %.4f  then  %.4f   (a proportional-to-h bug gives ~0.50, 0.50)\n', r1, r2);
p('   total spread over the 4x step change: %.3e m\n', max(dt.err_med)-min(dt.err_med));
p('   sim3d.py on the same kind of sweep: r_end -1.3064 -> -0.7455 -> -0.5546 m\n');
halves = (abs(r1-0.5) < 0.15) && (abs(r2-0.5) < 0.15);
gates{end+1} = g_('(dt) not step-size residue', ~halves, ...
    sprintf('ratios %.4f / %.4f -- the error must NOT halve when h halves', r1, r2));
G.gates = gates;

% ======================================================================== V
p('\n================ VERDICT ================\n');
npass = 0;
for k = 1:numel(gates)
    if gates{k}.pass, v = 'PASS'; npass = npass + 1; else, v = '**FAIL**'; end
    p('  %-26s %-9s %s\n', gates{k}.name, v, gates{k}.detail);
end
G.n_pass = npass; G.n_gate = numel(gates);
G.verdict = 'FAIL'; if npass == numel(gates), G.verdict = 'PASS'; end
p('\n  %d/%d criteria pass  =>  GATE %s\n', npass, numel(gates), G.verdict);
p('  wall time %.0f s\n\n', toc(t_all));

here = fileparts(mfilename('fullpath'));
od = fullfile(here,'out'); if ~isfolder(od), mkdir(od); end
save(fullfile(od,'gate.mat'),'G');
end

% ===========================================================================
function s = g_(name, ok, detail)
s = struct('name',name,'pass',logical(ok),'detail',detail);
end
function v = pct(x, q)
x = sort(x(isfinite(x)));
if isempty(x), v = NaN; return; end
n = numel(x); i = (q/100)*(n-1) + 1;
lo = floor(i); hi = ceil(i); f = i - lo;
v = x(lo)*(1-f) + x(hi)*f;
end
function r = spear(a, bb)
m = isfinite(a) & isfinite(bb);
if sum(m) < 4, r = NaN; return; end
x = rnk(a(m)); y = rnk(bb(m));
x = x - mean(x); y = y - mean(y);
d = sqrt(sum(x.^2)*sum(y.^2));
if d == 0, r = NaN; else, r = sum(x.*y)/d; end
end
function r = rnk(v)
[~, i] = sort(v); r = zeros(size(v)); r(i) = 1:numel(v);
u = unique(v);
for k = 1:numel(u)
    m = (v == u(k));
    if sum(m) > 1, r(m) = mean(r(m)); end
end
end
