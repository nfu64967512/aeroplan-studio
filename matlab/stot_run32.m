function [S, M, F] = stot_run32(varargin)
%STOT_RUN32  Fly the 32 measured release states and compare against truth32.csv.
%
%   [S, M, F] = stot_run32()
%   [S, M, F] = stot_run32('bank','zero', 'cfg', cfg.params(...), 'label','N1')
%
%   S  per-aircraft simulated results (table)
%   M  per-aircraft measured truth      (table, from matlab/data/truth32.csv)
%   F  the fidelity comparison, one struct of scalars (this is what the gate reads)
%
%   Options
%     'bank'   'traj' (default) | 'zero'   how the release bank angle is set
%     'cfg'    a cfg.params() struct       defaults to the RRE contract snapshot
%     'label'  char                        printed, and stored in F.label
%     'save'   true (default) | false      writes matlab/out/run32_<label>.csv
%     'quiet'  false (default)
%
%   The five gate quantities are dive duration, residual altitude, touchdown
%   error, nz clamp fraction and terminal speed.  Nothing in this file adjusts a
%   parameter to improve any of them.

o = parse_opts(varargin, struct('bank','traj','cfg',[],'label','RRE-contract', ...
                                'save',true,'quiet',false));
if isempty(o.cfg), o.cfg = cfg.params(); end
c = o.cfg;

[ics, M] = stot_ic32(o.bank);
N = numel(ics);

tag = strings(N,1);
t_dive = nan(N,1); err = nan(N,1); zres = nan(N,1); d1 = nan(N,1);
Vtd = nan(N,1); gstd = nan(N,1); eastd = nan(N,1); gamtd = nan(N,1);
nzc = nan(N,1); nzlo = nan(N,1); nzhi = nan(N,1);
los = nan(N,1); pmin_node = nan(N,1); rollnode = nan(N,1);
fcpl = nan(N,1); fcph = nan(N,1); fcrc = nan(N,1);
thrlo = nan(N,1); thrhi = nan(N,1); slew = nan(N,1); aero = nan(N,1);
climb = nan(N,1); gspk = nan(N,1); rollmax = nan(N,1); hit = false(N,1);

t0 = tic;
for i = 1:N
    r = stot_sim(ics(i), cfg_nolog(c));
    tag(i)   = string(r.tag);
    hit(i)   = r.hit;
    t_dive(i)= r.t_dive;   err(i) = r.err_touch;
    zres(i)  = r.z_res;    d1(i)  = r.d1_min;
    Vtd(i)   = r.V_touch;  gstd(i)= r.gs_touch;  eastd(i) = r.eas_touch;
    gamtd(i) = r.gam_touch;
    nzc(i)   = r.nz_clip_frac; nzlo(i) = r.nz_lo_frac; nzhi(i) = r.nz_hi_frac;
    los(i)   = r.los_frac;  pmin_node(i) = r.pitch_lo_frac;
    rollnode(i) = r.roll_node_clip_frac;
    fcpl(i)  = r.fc_pitch_lo_frac; fcph(i) = r.fc_pitch_hi_frac;
    fcrc(i)  = r.fc_roll_clip_frac;
    thrlo(i) = r.thr_lo_frac; thrhi(i) = r.thr_hi_frac;
    slew(i)  = r.fc_thr_slew_frac; aero(i) = r.aero_nz_frac;
    climb(i) = r.climb_after_rel; gspk(i) = r.gs_peak;
    rollmax(i) = r.roll_max_deg;
end
wall = toc(t0);

S = table(tag, hit, t_dive, err, zres, d1, Vtd, gstd, eastd, gamtd, ...
          nzc, nzlo, nzhi, los, pmin_node, rollnode, fcpl, fcph, fcrc, ...
          thrlo, thrhi, slew, aero, climb, gspk, rollmax, ...
   'VariableNames', {'tag','hit','t_dive_s','err_touch_m','z_res_m','d1_min_m', ...
     'V_touch_ms','gs_touch_ms','eas_touch_ms','gam_touch_deg', ...
     'nz_clip_frac','nz_lo_frac','nz_hi_frac','los_frac','node_pitch_lo_frac', ...
     'node_roll_clip_frac','fc_pitch_lo_frac','fc_pitch_hi_frac','fc_roll_clip_frac', ...
     'thr_lo_frac','thr_hi_frac','thr_slew_frac','aero_nz_frac', ...
     'climb_after_rel_m','gs_peak_ms','roll_max_deg'});

% ---- align measured to simulated by tag ------------------------------------
[tf_, loc] = ismember(cellstr(tag), M.tag);
assert(all(tf_), 'stot_run32: tag mismatch between ics and truth32');
M = M(loc,:);

F = compare(S, M, o, c, wall);
if ~o.quiet, report(S, M, F, c); end
if o.save
    here = fileparts(mfilename('fullpath'));
    od = fullfile(here,'out'); if ~isfolder(od), mkdir(od); end
    writetable(S, fullfile(od, sprintf('run32_%s.csv', safe(o.label))));
    mc = {'t_dive_sim_s','t_dive_wall_s','err_touch_m','z_res_m','V_touch_ms', ...
          'nz_clip_frac','nz_lo_frac','nz_hi_frac','climb_after_rel_m', ...
          'agl_min_m','d1_min_m','gs_peak_dive_ms'};
    Ms = M(:, mc);
    Ms.Properties.VariableNames = cellfun(@(s)['meas_' s], mc, 'UniformOutput', false);
    writetable([S, Ms], fullfile(od, sprintf('compare32_%s.csv', safe(o.label))));
end
end

% ===========================================================================
function F = compare(S, M, o, c, wall)
F = struct('label', o.label, 'bank', o.bank, 'n', height(S), 'wall_s', wall);
F.throttle_law = c.pn.throttle_law;
F.los_floor = c.pn.los_floor;  F.nz_min = c.pn.nz_min;

isr = startsWith(S.tag, "RRE");

% -- gate quantity 1: dive duration.  Compare against t_dive_sim_s (freeze
%    removed) because the model has no node outage by default.
F.tdive_sim_med_meas = median(M.t_dive_sim_s);
F.tdive_med_sim      = median(S.t_dive_s, 'omitnan');
F.tdive_bias_med     = median(S.t_dive_s - M.t_dive_sim_s, 'omitnan');
F.tdive_mae          = mean(abs(S.t_dive_s - M.t_dive_sim_s), 'omitnan');
F.tdive_spearman     = spear(S.t_dive_s, M.t_dive_sim_s);

% -- gate quantity 2: residual altitude.  The 0.0002 m spread is what killed
%    sim3d: a model whose residual altitude never varies has zero discriminating
%    power, whatever its median is.
F.zres_sd_meas = std(M.z_res_m, 'omitnan');
F.zres_sd_sim  = std(S.z_res_m, 'omitnan');
F.zres_med_meas= median(M.z_res_m, 'omitnan');
F.zres_med_sim = median(S.z_res_m, 'omitnan');
F.zres_spearman= spear(S.z_res_m, M.z_res_m);

% -- gate quantity 3: touchdown error
F.err_med_meas = median(M.err_touch_m, 'omitnan');
F.err_med_sim  = median(S.err_touch_m, 'omitnan');
F.err_hit10_meas = sum(M.err_touch_m <= 10);
F.err_hit10_sim  = sum(S.err_touch_m <= 10);
F.err_spearman = spear(S.err_touch_m, M.err_touch_m);
F.err_med_meas_rre = median(M.err_touch_m(isr), 'omitnan');
F.err_med_sim_rre  = median(S.err_touch_m(isr), 'omitnan');

% -- gate quantity 4: nz clamp fraction (THE headline number)
F.nz_frac_meas_min = min(M.nz_clip_frac);
F.nz_frac_meas_max = max(M.nz_clip_frac);
F.nz_frac_meas_med = median(M.nz_clip_frac);
F.nz_frac_sim_min  = min(S.nz_clip_frac);
F.nz_frac_sim_max  = max(S.nz_clip_frac);
F.nz_frac_sim_med  = median(S.nz_clip_frac);
F.nz_zero_count_sim  = sum(S.nz_clip_frac == 0);
F.nz_zero_count_meas = sum(M.nz_clip_frac == 0);
F.nz_spearman = spear(S.nz_clip_frac, M.nz_clip_frac);

% -- gate quantity 5: terminal speed
F.V_med_meas = median(M.V_touch_ms, 'omitnan');
F.V_med_sim  = median(S.V_touch_ms, 'omitnan');
F.V_bias_med = median(S.V_touch_ms - M.V_touch_ms, 'omitnan');
F.V_spearman = spear(S.V_touch_ms, M.V_touch_ms);

% -- structural fingerprint: sim3d cannot produce a post-release climb at all
F.climb_med_meas = median(M.climb_after_rel_m);
F.climb_med_sim  = median(S.climb_after_rel_m);
F.climb_nonzero_sim  = sum(S.climb_after_rel_m > 1);
F.climb_nonzero_meas = sum(M.climb_after_rel_m > 1);
end

% ===========================================================================
function report(S, M, F, c)
p = @(varargin) fprintf(varargin{:});
p('\n=== stot_run32  [%s]  bank=%s  law=%s  los_floor=%.1f  nz_min=%+.2f ===\n', ...
  F.label, F.bank, F.throttle_law, F.los_floor, F.nz_min);
p('    %d airframes, %.1f s wall\n\n', F.n, F.wall_s);

p('  quantity                       measured           simulated        note\n');
p('  ---------------------------------------------------------------------------\n');
p('  dive duration  median [s]      %8.2f           %8.2f      bias %+.2f, MAE %.2f, rho %+.3f\n', ...
  F.tdive_sim_med_meas, F.tdive_med_sim, F.tdive_bias_med, F.tdive_mae, F.tdive_spearman);
p('  residual alt   median [m]      %8.3f           %8.3f      rho %+.3f\n', ...
  F.zres_med_meas, F.zres_med_sim, F.zres_spearman);
p('  residual alt   sd     [m]      %8.4f           %8.4f      <- sim3d had 0.0002\n', ...
  F.zres_sd_meas, F.zres_sd_sim);
p('  touchdown err  median [m]      %8.2f           %8.2f      rho %+.3f\n', ...
  F.err_med_meas, F.err_med_sim, F.err_spearman);
p('  touchdown err  <= 10 m         %5d/%d            %5d/%d\n', ...
  F.err_hit10_meas, F.n, F.err_hit10_sim, F.n);
p('  nz clamp frac  median [%%]      %8.2f           %8.2f      rho %+.3f\n', ...
  100*F.nz_frac_meas_med, 100*F.nz_frac_sim_med, F.nz_spearman);
p('  nz clamp frac  range  [%%]   %6.2f-%6.2f      %6.2f-%6.2f\n', ...
  100*F.nz_frac_meas_min, 100*F.nz_frac_meas_max, ...
  100*F.nz_frac_sim_min,  100*F.nz_frac_sim_max);
p('  nz clamp frac  == 0 count      %5d/%d            %5d/%d   <- sim3d had 29/32\n', ...
  F.nz_zero_count_meas, F.n, F.nz_zero_count_sim, F.n);
p('  terminal speed median [m/s]    %8.1f           %8.1f      bias %+.1f, rho %+.3f\n', ...
  F.V_med_meas, F.V_med_sim, F.V_bias_med, F.V_spearman);
p('  climb after release median [m] %8.1f           %8.1f      nonzero %d/%d vs %d/%d\n', ...
  F.climb_med_meas, F.climb_med_sim, F.climb_nonzero_meas, F.n, F.climb_nonzero_sim, F.n);

p('\n  clamp activity in the model (fraction of ticks, median over 32):\n');
q = @(v) 100*median(v);
p('    node  nz low %5.2f %%   nz high %5.2f %%   LOS floor binding %5.1f %%\n', ...
  q(S.nz_lo_frac), q(S.nz_hi_frac), q(S.los_frac));
p('    node  pitch at -80 %5.2f %%   roll at 80|cos th| %5.2f %%\n', ...
  q(S.node_pitch_lo_frac), q(S.node_roll_clip_frac));
p('    FC    pitch_min (-80|cos roll|) %5.2f %%   pitch_max %5.2f %%   roll %5.2f %%\n', ...
  q(S.fc_pitch_lo_frac), q(S.fc_pitch_hi_frac), q(S.fc_roll_clip_frac));
p('    thr   sat low %5.2f %%   sat high %5.2f %%   slew rate %5.2f %%\n', ...
  q(S.thr_lo_frac), q(S.thr_hi_frac), q(S.thr_slew_frac));
p('    aero  CL ceiling binding %5.2f %%   peak |roll| median %5.1f deg\n', ...
  q(S.aero_nz_frac), median(S.roll_max_deg));
p('\n  priors in this model (the only free numbers): \n');
for k = 1:numel(c.meta.priors), p('    - %s\n', c.meta.priors{k}); end
p('\n');
end

% ===========================================================================
function r = spear(a, bb)
m = isfinite(a) & isfinite(bb);
if sum(m) < 4, r = NaN; return; end
r = corr_p(tiedrank_(a(m)), tiedrank_(bb(m)));
end
function r = corr_p(x, y)
x = x - mean(x); y = y - mean(y);
d = sqrt(sum(x.^2)*sum(y.^2));
if d == 0, r = NaN; else, r = sum(x.*y)/d; end
end
function r = tiedrank_(v)
% plain average-rank, so no Statistics Toolbox dependency
[~, i] = sort(v); r = zeros(size(v)); r(i) = 1:numel(v);
u = unique(v);
for k = 1:numel(u)
    m = (v == u(k));
    if sum(m) > 1, r(m) = mean(r(m)); end
end
end
function c = cfg_nolog(c)
c.sim.log = false;
end
function s = safe(s)
s = regexprep(char(s), '[^A-Za-z0-9_.-]', '_');
end
function o = parse_opts(args, o)
for k = 1:2:numel(args)
    f = lower(char(args{k}));
    if ~isfield(o, f), error('stot_run32: unknown option "%s"', f); end
    o.(f) = args{k+1};
end
end
