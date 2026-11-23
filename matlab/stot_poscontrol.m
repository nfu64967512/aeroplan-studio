function P = stot_poscontrol()
%STOT_POSCONTROL  Positive control: our guidance law vs Zarchan Listing 30.1
%                 flown from IDENTICAL initial conditions.
%
%   P = stot_poscontrol()
%
%   DIAGNOSTIC ONLY -- this does not enter the gate verdict.  sim3d.py was never
%   run against a reference of known answer; the point here is to find out WHERE
%   our law departs from a textbook law, and by how much, on the same geometry.
%
%   COMMON INITIAL CONDITION.  The median RRE release state out of truth32.csv:
%   horizontal range r0 to a sea-level bullseye, altitude z0, total speed V0,
%   flight-path angle gam0, heading dead at the target (eps0 = 0, wings level)
%   so the engagement is planar and Listing 30.1's 2-D kinematics apply without
%   any projection.
%
%   Zarchan's frame: missile at the origin, target at (RT1, RT2) = (r0, -z0).
%   That is what the RT2M option (the only non-book line in the transcription)
%   is for; RT2M = 0 leaves every E3 case bit-identical.
%
%   THE MODELS ARE NOT THE SAME PHYSICS, AND THAT IS THE POINT:
%     Listing 30.1   no gravity, no drag, no thrust, no attitude loop, no
%                    actuator; a_cmd applied instantly, limited only by XNCLIM.
%     ours           gravity, calibrated drag, throttle law, alpha low-pass,
%                    2nd-order attitude loop, ArduPilot clamp chain, and the
%                    nz in [-1, 3] + LOS-floor pair.
%   So any divergence is expected.  The question the number answers is whether
%   our law is in the same family as a working terminal-angle law, or somewhere
%   else entirely.

p = @(varargin) fprintf(varargin{:});
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'ref'));

% ------------------------------------------------- common initial condition
[ics, T] = stot_ic32('zero');
isr = startsWith(string(T.tag), "RRE");
r0   = median(T.r_rel_m(isr));
z0   = median(T.z_rel_m(isr));
V0   = median(T.V_rel_ms(isr));
gam0 = median(T.gamma_rel_deg(isr));
dT   = median(T.t_rem0_s(isr));

ic = ics(1);
ic.tag = 'POSCTRL-medianRRE';
ic.r0 = r0; ic.z0 = z0; ic.V0 = V0; ic.gam0_deg = gam0;
ic.eps0_deg = 0.0; ic.phi0_deg = 0.0; ic.dive_T = dT;
P.ic = ic;

p('\n================ 5. POSITIVE CONTROL vs ZARCHAN LISTING 30.1 ================\n');
p('  common IC (median RRE release state, planar, wings level):\n');
p('    r0 = %.1f m   z0 = %.1f m   V0 = %.1f m/s   gam0 = %+.2f deg   LOS dep = %.2f deg\n', ...
    r0, z0, V0, gam0, rad2deg(atan2(z0,r0)));
p('    dive_T = %.2f s (ours only; Listing 30.1 has no time constraint)\n\n', dT);

% ------------------------------------------------------------------- ours
c = cfg.params();
o = stot_sim(ic, c);
P.ours = o;

% ours: horizontal range to go, altitude, gamma, along the flown track
r_o   = hypot(o.TR(:,2), o.TR(:,3));
z_o   = o.TR(:,4);
V_o   = o.TR(:,5);
g_o   = rad2deg(o.TR(:,6));
t_o   = o.TR(:,1);
k_o   = t_o <= o.t_dive + 1e-9;

% node-side normal acceleration proxy: the nz the guidance law asked for
nz_cmd = o.GD(:,7);  nz_app = o.GD(:,8);
P.nz_raw_min = min(nz_cmd);  P.nz_raw_max = max(nz_cmd);

% ---------------------------------------------------------------- Zarchan
% Case Z1: optimal trajectory shaping (IGUID 0), a_cmd perpendicular to the LOS
%          -- the book's nominal.  Terminal angle demand -90 deg, same as the
%          steepest thing our LOS floor can ask for.
% Case Z2: same but a_cmd perpendicular to V (ICHOICE 1), which is the variant
%          that conserves speed -- structurally closer to a lift-generated
%          normal acceleration, i.e. closer to what an aircraft does.
% Case Z3: Z2 with the acceleration limit cut from the book's 10 g to 3 g,
%          which is OUR pn.nz_max.  This is the run that isolates authority.
zo = {'RANGEM', r0/1000*1000, 'RT2M', -z0, 'VMIC', V0, 'GAMDEG', gam0, 'GAMFDEG', -90};
Z = struct('name',{},'opts',{},'out',{});
Z(1) = mk('Z1 optimal shaping, a perp LOS, 10 g', [zo, {'IGUID',0,'ICHOICE',0,'XNCLIMG',10}]);
Z(2) = mk('Z2 optimal shaping, a perp V,   10 g', [zo, {'IGUID',0,'ICHOICE',1,'XNCLIMG',10}]);
Z(3) = mk('Z3 optimal shaping, a perp V,    3 g', [zo, {'IGUID',0,'ICHOICE',1,'XNCLIMG',3}]);
Z(4) = mk('Z4 biased PN N=3,   a perp V,   10 g', [zo, {'IGUID',1,'ICHOICE',1,'XNCLIMG',10,'DELT',dT}]);
for k = 1:numel(Z), Z(k).out = zarchan_listing30_1(Z(k).opts{:}); end
P.Z = Z;

p('  terminal comparison\n');
p('  law                                       tf [s]   miss [m]   gam_f [deg]   V_f [m/s]  a_max [g]\n');
p('  --------------------------------------------------------------------------------------------\n');
p('  OURS (full model, LOS floor + nz clamp)  %7.2f  %9.3f     %+8.2f    %8.1f     %s\n', ...
    o.t_dive, o.err_touch, o.gam_touch, o.V_touch, sprintf('%.2f*', max(abs(nz_app))));
for k = 1:numel(Z)
    zk = Z(k).out;
    p('  %-40s %7.2f  %9.3f     %+8.2f    %8.1f     %.2f\n', ...
        Z(k).name, zk.tf, zk.miss_m, zk.gam_f_deg, zk.vm_f_ms, zk.xnc_max_g);
end
p('  * ours is load factor nz (body normal, gravity included), not |a|/g -- see note below\n');

% -------------------------------------------------- where do they diverge
% Common independent variable = horizontal range to go, from r0 down to 0.
% Both trajectories are monotone in it here, so altitude is a function of it.
p('\n  divergence in the vertical plane, as a function of horizontal range to go\n');
p('  law     alt sep @ 3/4 r0   @ 1/2 r0   @ 1/4 r0    max sep    r where sep first > 100 m\n');
p('  --------------------------------------------------------------------------------------\n');
rq = r0*[0.75 0.50 0.25];
P.div = struct('name',{},'sep_q',{},'sep_max',{},'r_100',{});
for k = 1:numel(Z)
    zk = Z(k).out;
    rz = r0 - zk.RM1K*1000;          % Zarchan downrange -> range to go
    hz = zk.RM2K*1000;               % altitude above the RELEASE point
    hz = hz + z0;                    % -> altitude above the target datum
    ok = rz > 1 & isfinite(hz);
    [ru, hu] = mono(rz(ok), hz(ok));
    % ours on the same grid
    [ro2, zo2] = mono(r_o(k_o), z_o(k_o));
    gr = linspace(min(max(ru),max(ro2)), 1.0, 2000).';
    a = interp1(ru, hu, gr, 'linear', NaN);
    bq = interp1(ro2, zo2, gr, 'linear', NaN);
    sep = abs(a - bq);
    sq = interp1(gr, sep, rq, 'linear', NaN);
    i100 = find(sep > 100 & isfinite(sep), 1, 'first');
    r100 = NaN; if ~isempty(i100), r100 = gr(i100); end
    p('  %-6s %12.1f %11.1f %10.1f %11.1f %14s\n', Z(k).name(1:2), sq(1), sq(2), sq(3), ...
      max(sep(isfinite(sep))), rstr(r100));
    P.div(k) = struct('name',Z(k).name,'sep_q',sq,'sep_max',max(sep(isfinite(sep))),'r_100',r100);
end

% ------------------------------------------------------- gamma histories
p('\n  flight-path angle: how steep each law actually gets\n');
p('  law     gam at 3/4 r0  1/2 r0   1/4 r0   min gam   terminal gam\n');
p('  ---------------------------------------------------------------\n');
[ro2, go2] = mono(r_o(k_o), g_o(k_o));
gq = interp1(ro2, go2, rq, 'linear', NaN);
p('  OURS   %12.2f %8.2f %8.2f %9.2f %13.2f\n', gq(1), gq(2), gq(3), min(g_o(k_o)), o.gam_touch);
for k = 1:numel(Z)
    zk = Z(k).out;
    rz = r0 - zk.RM1K*1000;
    ok = rz > 1;
    [ru, gu] = mono(rz(ok), zk.GAMDEG(ok));
    gqz = interp1(ru, gu, rq, 'linear', NaN);
    p('  %-6s %12.2f %8.2f %8.2f %9.2f %13.2f\n', Z(k).name(1:2), gqz(1), gqz(2), gqz(3), ...
        min(zk.GAMDEG), zk.gam_f_deg);
end

p(['\n  READING THIS.  Listing 30.1 has no gravity, no drag and no attitude loop, so\n' ...
   '  its acceleration limit is the ONLY authority constraint it has, and it can hold\n' ...
   '  any commanded gamma-dot instantly.  Ours spends most of its vertical authority\n' ...
   '  holding 1 g against gravity: nz in [-1, +3] leaves only +2 g of manoeuvre and\n' ...
   '  -2 g of pushover, and pn_los_floor then overrides the PN branch outright.  The\n' ...
   '  separation numbers above are therefore an upper bound on "how much of our miss\n' ...
   '  behaviour is the guidance law" -- most of it is the airframe and the clamp chain.\n']);

od = fullfile(here,'out'); if ~isfolder(od), mkdir(od); end
save(fullfile(od,'poscontrol.mat'),'P');
end

% ===========================================================================
function s = mk(name, opts)
s = struct('name',name,'opts',{opts},'out',[]);
end
function [xs, ys] = mono(x, y)
% strictly increasing sample points, averaging duplicates -- interp1 needs this
x = x(:); y = y(:);
m = isfinite(x) & isfinite(y);
x = x(m); y = y(m);
[x, i] = sort(x); y = y(i);
[xs, ~, ic] = unique(x);
ys = accumarray(ic, y, [], @mean);
end
function s = rstr(v)
if isnan(v), s = 'never'; else, s = sprintf('%.0f m', v); end
end
