function R = sweep_band3s(Rs, Vs, band_s)
%SWEEP_BAND3S  How fast can a 6-8 ship group be if it must arrive within band_s?
%
%   R = sweep_band3s()                       defaults below
%   R = sweep_band3s([6760 9200], [210 250], 3.0)
%
%   The user's objective, in order:
%     1. make the arrival spread as SHORT as possible
%     2. subject to that, make the terminal speed as FAST as possible
%
%   So the figure of merit is NOT "is 204 m/s reachable somewhere in the
%   window".  It is: pack the whole group into a band_s slice at the FAST end
%   of the reachable arrival window, and report the SLOWEST ship in it -- that
%   ship is what the acceptance criterion actually grades.
%
%   With 6-8 aircraft in a 3 s band the ships sit ~0.43-0.60 s apart, so the
%   slowest one is the one commanded to arrive at t_early + band_s.
%
% ---------------------------------------------------------------------------
%   ⚠️⚠️ WHAT THIS CAN AND CANNOT SAY.
%
%   The reachable window and the speed-vs-arrival-time curve come from the
%   drag polar (CD0 = 0.01500, inverse-solved from 1617 flight samples), which
%   is the CALIBRATED part of this model.  Hit rate and touchdown error are
%   NOT: mgs scores 32/32 at 0.03 m where the flight set scores 25/32 at
%   0.93 m.
%
%   And the absolute speeds do not transfer.  Measured on LAT1 (2026-08-23):
%   terminal GROUND speed 150.3 / 147.7 m/s, while this model at the same
%   release range gives 193.1 at the window's fast edge -- it over-predicts by
%   about 43 m/s.  Use the ORDERING and the DIRECTION; do not quote a band
%   width or a speed from here as a flight prediction.
%
%   ⚠️ The window is a CAPACITY, not a schedule.  A 20 s window does not make
%   a 19 s release-time spread disappear; it means the throttle channel could
%   absorb it IF the release scheduler commands a different dive_T per ship.
%
%   See also MGS.SIMSWARM, MGS.KPI.

if nargin < 1 || isempty(Rs),     Rs = [6760 9200 12000 14000]; end
if nargin < 2 || isempty(Vs),     Vs = [210 230 250]; end
if nargin < 3 || isempty(band_s), band_s = 3.0; end

R = struct('r0', {}, 'vmax', {}, 'win_s', {}, 'gs_fast', {}, 'gs_slow', {}, ...
           't_early', {}, 'feasible', {});

fprintf('\n群塞進窗最早端的 %.1f s 切片(6-8 台) => 最慢一架的終端地速\n', band_s);
fprintf('%7s |', 'r0');
fprintf(' vmax %-9d', Vs); fprintf('  %9s %9s\n', '窗寬 s', 'gam_los');

for r0 = Rs
    fprintf('%7d |', r0);
    win = NaN; gl = NaN;
    for vm = Vs
        c = mgs.params(); c.swarm.n = 1; c.sim.t_max = 500; c.pn.vmax = vm;
        s0 = one_ship(c, r0);
        [tE, tL] = window_edges(c, s0);
        if ~isfinite(tE)
            fprintf(' %14s', 'no-window');
            R(end+1) = row(r0, vm, NaN, NaN, NaN, NaN, false); %#ok<AGROW>
            continue;
        end
        win = tL - tE;
        gsF = gs_at(c, s0, tE);
        tS  = min(tE + band_s, tL);
        gsS = gs_at(c, s0, tS);
        ok  = (tL - tE) >= band_s;
        fprintf(' %14.1f', gsS);
        R(end+1) = row(r0, vm, win, gsF, gsS, tE, ok); %#ok<AGROW>
    end
    gl = atand(c.scen.z0 / sqrt(max(r0^2 - c.scen.z0^2, 1)));
    fprintf('  %9.1f %9.1f\n', win, gl);
end
fprintf('\n⚠️ 模型在 LAT1 的射程上高估終端地速約 43 m/s(193.1 vs 實機 150.3/147.7)\n');
fprintf('   ⇒ 只讀方向與排序,不要把數字當飛行預測。\n');
end

% =============================================================================
function s = one_ship(c, r0)
scn = mgs.scenario(c, 'tag', 'B3');
s = scn; s.n = 1; s.ic = scn.ic(1); s.ic.r0 = r0;
end

function t = arrive(c, s0, dive_T)
%ARRIVE  Touchdown time for a commanded dive_T, or NaN if it does not impact.
s = s0; s.ic.dive_T = dive_T;
try
    o = mgs.simSwarm(s, c, 'verbose', false);
    if strcmp(o.veh(1).end_reason, 'impact'), t = o.veh(1).t_impact; else, t = NaN; end
catch
    t = NaN;
end
end

function g = gs_impact(c, s0, dive_T)
s = s0; s.ic.dive_T = dive_T;
try
    o = mgs.simSwarm(s, c, 'verbose', false);
    if strcmp(o.veh(1).end_reason, 'impact'), g = o.veh(1).gs_impact; else, g = NaN; end
catch
    g = NaN;
end
end

function [tE, tL] = window_edges(c, s0)
%WINDOW_EDGES  The reachable arrival interval, by bisection on the saturation.
%   ⚠️ Bisection, not a dense grid: a grid fine enough to resolve the edges
%   costs ~100 sims per cell and the last attempt at that timed out at 1800 s.
lo = 10; hi = 260;
tLo = arrive(c, s0, lo);  tHi = arrive(c, s0, hi);
if ~isfinite(tLo) || ~isfinite(tHi), tE = NaN; tL = NaN; return; end
tE = tLo; tL = tHi;
% early edge: largest dive_T that still saturates at tE
a = lo; b = hi;
for k = 1:14
    m = 0.5*(a+b);
    if abs(arrive(c, s0, m) - tE) < 0.05, a = m; else, b = m; end
end
tE = arrive(c, s0, a);
% late edge: smallest dive_T that already saturates at tL
a = lo; b = hi;
for k = 1:14
    m = 0.5*(a+b);
    if abs(arrive(c, s0, m) - tL) < 0.05, b = m; else, a = m; end
end
tL = arrive(c, s0, b);
end

function g = gs_at(c, s0, t_want)
%GS_AT  Terminal ground speed for the dive_T that arrives at t_want.
a = 10; b = 260;
for k = 1:14
    m = 0.5*(a+b);
    t = arrive(c, s0, m);
    if ~isfinite(t), b = m; continue; end
    if t < t_want, a = m; else, b = m; end
end
g = gs_impact(c, s0, 0.5*(a+b));
end

function r = row(r0, vm, w, gf, gsl, tE, ok)
r = struct('r0', r0, 'vmax', vm, 'win_s', w, 'gs_fast', gf, ...
           'gs_slow', gsl, 't_early', tE, 'feasible', ok);
end
