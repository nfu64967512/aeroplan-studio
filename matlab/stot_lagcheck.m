function L = stot_lagcheck(quiet, lagmax)
%STOT_LAGCHECK  How fast does the real aircraft follow the commanded gamma, and
%               how fast does the model?
%
%   L = stot_lagcheck()
%
%   For every measured flight this replays the LONGITUDINAL half of _pn_cmd on
%   the flown state (matlab/data/traj/<tag>.csv), producing the gam_cmd the node
%   would have issued at each row, then finds the time shift that best lines
%   gam_cmd up with the gamma the aircraft actually flew.  The same computation
%   is run on stot_sim output.  If the model tracks faster than the vehicle, the
%   difference in these two lags is a number, not an opinion.
%
%   Everything here uses the CSV's own gs and gam_deg columns, which are
%   bit-for-bit what the node saw; only ddot is differentiated, over a +-0.5 s
%   centred window on the de-quantised clock (the raw t is quantised to 0.1 s and
%   about half the adjacent rows share a stamp).

if nargin < 1, quiet = false; end
if nargin < 2, lagmax = 20.0; end
here = fileparts(mfilename('fullpath'));
c   = cfg.params();
ics = stot_ic32('traj');
lags = 0:0.05:lagmax;

tag = strings(numel(ics),1);
lag_meas = nan(numel(ics),1);  rms_meas = nan(numel(ics),1);  r0_meas = nan(numel(ics),1);
lag_sim  = nan(numel(ics),1);  rms_sim  = nan(numel(ics),1);  r0_sim  = nan(numel(ics),1);
nzf_meas = nan(numel(ics),1);  nzf_sim  = nan(numel(ics),1);
gain_meas= nan(numel(ics),1);  corr_meas= nan(numel(ics),1);
gain_sim = nan(numel(ics),1);  corr_sim = nan(numel(ics),1);

cs = c; cs.sim.log = true;
for i = 1:numel(ics)
    tag(i) = string(ics(i).tag);
    f = fullfile(here,'data','traj',[strrep(ics(i).tag,'-','_') '.csv']);
    A = readtable(f);
    keep = A.post_touch == 0;
    t = A.t_dq(keep); r = A.r(keep); z = A.z(keep);
    gam = deg2rad(A.gamma_deg(keep)); gs = A.gs(keep);
    if numel(t) < 30, continue; end
    [gcmd, nzlo, nzhi] = replay_gamcmd(t, r, z, gam, gs, c.pn);
    nzf_meas(i) = (nzlo+nzhi)/numel(t);
    [lag_meas(i), rms_meas(i)] = best_lag(t, gcmd, t, gam, lags);
    [~, r0_meas(i)] = best_lag(t, gcmd, t, gam, 0);
    [gain_meas(i), corr_meas(i)] = rate_gain(t, gam, gcmd, c.pn.lead);

    o = stot_sim(ics(i), cs);
    ts = o.GD(:,1); zs = o.GD(:,3); rs = o.GD(:,2);
    gams = o.GD(:,6); gss = o.GD(:,5).*cos(gams);
    m = ts <= o.t_dive;
    [gcs, l2, h2] = replay_gamcmd(ts(m), rs(m), zs(m), gams(m), gss(m), c.pn);
    nzf_sim(i) = (l2+h2)/max(sum(m),1);
    [lag_sim(i), rms_sim(i)] = best_lag(ts(m), gcs, ts(m), gams(m), lags);
    [~, r0_sim(i)] = best_lag(ts(m), gcs, ts(m), gams(m), 0);
    [gain_sim(i), corr_sim(i)] = rate_gain(ts(m), gams(m), gcs, c.pn.lead);
end

L = table(tag, lag_meas, lag_sim, rms_meas*180/pi, rms_sim*180/pi, r0_meas*180/pi, r0_sim*180/pi, ...
          nzf_meas, nzf_sim, gain_meas, gain_sim, corr_meas, corr_sim, ...
    'VariableNames', {'tag','lag_meas_s','lag_sim_s','rms_meas_deg','rms_sim_deg', ...
                      'rms0_meas_deg','rms0_sim_deg','nz_replay_meas','nz_replay_sim', ...
                      'rate_gain_meas','rate_gain_sim','rate_corr_meas','rate_corr_sim'});
if ~quiet
    fprintf('\n=== gamma tracking lag: vehicle vs model ===\n');
    fprintf('  best-fit lag  [s]   measured median %.2f  (range %.2f-%.2f)\n', ...
        median(lag_meas,'omitnan'), min(lag_meas), max(lag_meas));
    fprintf('                      model    median %.2f  (range %.2f-%.2f)\n', ...
        median(lag_sim,'omitnan'), min(lag_sim), max(lag_sim));
    fprintf('  residual rms  [deg] measured median %.2f   model median %.2f\n', ...
        median(rms_meas,'omitnan')*180/pi, median(rms_sim,'omitnan')*180/pi);
    fprintf('  nz clamp frac (this replay) measured median %.2f %%   model median %.2f %%\n', ...
        100*median(nzf_meas,'omitnan'), 100*median(nzf_sim,'omitnan'));
    fprintf('  residual at lag 0 [deg] measured median %.2f   model median %.2f\n', ...
        median(r0_meas,'omitnan')*180/pi, median(r0_sim,'omitnan')*180/pi);
    fprintf('  at the %.0f s search ceiling: %d/%d measured, %d/%d model\n', ...
        lagmax, sum(lag_meas >= lagmax-1e-9), numel(ics), ...
        sum(lag_sim >= lagmax-1e-9), numel(ics));
    fprintf('  lag ratio (model / vehicle) median %.2f\n', ...
        median(lag_sim,'omitnan')/max(median(lag_meas,'omitnan'),1e-9));
    fprintf('  gamma-rate gain (achieved/demanded)  measured median %.3f   model median %.3f\n', ...
        median(gain_meas,'omitnan'), median(gain_sim,'omitnan'));
    fprintf('  corr(achieved rate, demanded rate)  measured median %+.3f  model median %+.3f\n', ...
        median(corr_meas,'omitnan'), median(corr_sim,'omitnan'));
    fprintf(['  READ THIS AS: the flown gamma is not a delayed copy of gam_cmd at all.\n' ...
             '  A longer search window keeps buying a longer "best" lag while the\n' ...
             '  residual stays several degrees, which is the signature of a different\n' ...
             '  signal, not a late one.  The model, by contrast, tracks its own command\n' ...
             '  to a quarter of a degree.\n\n']);
end
end

% ---------------------------------------------------------------------------
function [gcmd, nlo, nhi] = replay_gamcmd(t, r, z, gam, gs, pn)
%REPLAY_GAMCMD  The longitudinal half of _pn_cmd, evaluated on a flown state.
n = numel(t);
gcmd = nan(n,1); nlo = 0; nhi = 0;
hdot = gs.*tan(gam);
V    = max(1e-3, hypot(gs, hdot));
ddot = local_slope(t, r, 0.5);
for k = 1:n
    d = max(1e-3, r(k)); h = z(k); R = max(1e-3, hypot(d,h));
    sig_dot = (hdot(k)*d - h*ddot(k)) / (R*R);
    gdot = -pn.n_vert*sig_dot;
    nz_raw = cos(gam(k)) + V(k)*gdot/pn.g;
    if nz_raw < pn.nz_min, nlo = nlo + 1; end
    if nz_raw > pn.nz_max, nhi = nhi + 1; end
    nz = max(pn.nz_min, min(pn.nz_max, nz_raw));
    gdot = (nz - cos(gam(k)))*pn.g/V(k);
    gc = gam(k) + gdot*pn.lead;
    if pn.los_floor > 0
        gc = min(gc, -atan2(h, max(d,1e-3))*pn.los_floor);
    end
    gcmd(k) = gc;
end
end

function [gain, cc] = rate_gain(t, gam, gcmd, lead)
%RATE_GAIN  Regress the ACHIEVED gamma rate on the DEMANDED one.
%   demanded gamma rate = (gam_cmd - gam)/lead, which is exactly what the node
%   asked for this tick.  A gain near 1 means the vehicle did what it was told;
%   a gain near 0 with a low correlation means it was not following the
%   longitudinal command at all, which is a different diagnosis from "late".
gain = NaN; cc = NaN;
[tu, iu] = unique(t(:), 'stable');
gam = gam(iu); gcmd = gcmd(iu);
if numel(tu) < 30, return; end
gd_meas = local_slope(tu, gam, 0.5);
gd_cmd  = (gcmd - gam)/lead;
m = isfinite(gd_meas) & isfinite(gd_cmd);
if sum(m) < 20, return; end
x = gd_cmd(m); y = gd_meas(m);
gain = (x'*y)/max(x'*x, 1e-12);
xc = x - mean(x); yc = y - mean(y);
cc = (xc'*yc)/max(sqrt((xc'*xc)*(yc'*yc)), 1e-12);
end

function s = local_slope(t, y, half)
n = numel(t); s = zeros(n,1);
for k = 1:n
    m = abs(t - t(k)) <= half;
    if sum(m) < 3, s(k) = 0; continue; end
    tt = t(m) - t(k);
    p = [tt, ones(sum(m),1)] \ y(m);
    s(k) = p(1);
end
end

function [lag, rmsv] = best_lag(tc, gcmd, tg, gam, lags)
%BEST_LAG  Shift that minimises rms(gam(t+lag) - gam_cmd(t)).
best = inf; lag = NaN; rmsv = NaN;
ok = isfinite(gcmd);
tk = tc(ok);  gk = gcmd(ok);
% t_dq de-quantises stamps but is not guaranteed strictly increasing when more
% than two rows share a 0.1 s stamp, and interp1 refuses duplicates.
[tg, iu] = unique(tg(:), 'stable');  gam = gam(iu);
[tg, is] = sort(tg);                 gam = gam(is);
for L = lags
    gi = interp1(tg, gam, tk + L, 'linear', NaN);
    m  = isfinite(gi);
    if sum(m) < 20, continue; end
    e = sqrt(mean((gi(m) - gk(m)).^2));
    if e < best, best = e; lag = L; rmsv = e; end
end
end
