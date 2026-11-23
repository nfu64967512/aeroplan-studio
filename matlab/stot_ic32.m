function [ics, T] = stot_ic32(bank_mode)
%STOT_IC32  Build the 32 release states from matlab/data/truth32.csv.
%
%   ics = stot_ic32()          bank at release estimated from the flown track
%   ics = stot_ic32('zero')    bank at release forced to 0 (sensitivity control)
%   ics = stot_ic32('traj')    same as the default
%
%   truth32.csv gives r, z, V, gamma and the SIGNED heading offset eps at the
%   release row, plus the scheduled touchdown dive_T.  It does not give the bank
%   angle, because the node never logged one.  Bank matters: several airframes
%   release with eps up to -81 deg, i.e. pointing far off the bullseye, and a
%   model that starts them wings-level has to spend the first seconds rolling in
%   that the real aircraft had already spent.  'traj' recovers it from the flown
%   lat/lon in matlab/data/traj/<tag>.csv via the coordinated-turn relation
%   tan(phi) = chidot*vh/g.  That is measurement, not tuning, but it is a
%   derived quantity, so stot_run32 reports the 'zero' control alongside it.

if nargin < 1 || isempty(bank_mode), bank_mode = 'traj'; end
here = fileparts(mfilename('fullpath'));
T = readtable(fullfile(here,'data','truth32.csv'));

ics = struct('tag',{},'trial',{},'sysid',{},'r0',{},'z0',{},'V0',{}, ...
             'gam0_deg',{},'eps0_deg',{},'dive_T',{},'phi0_deg',{});
for i = 1:height(T)
    s.tag      = char(T.tag{i});
    s.trial    = char(T.trial{i});
    s.sysid    = T.sysid(i);
    s.r0       = T.r_rel_m(i);
    s.z0       = T.z_rel_m(i);
    s.V0       = T.V_rel_ms(i);
    s.gam0_deg = T.gamma_rel_deg(i);
    s.eps0_deg = T.eps_rel_deg(i);
    s.dive_T   = T.t_rem0_s(i);       % dive_T - now at release, in sim time
    s.phi0_deg = 0.0;
    if strcmpi(bank_mode,'traj')
        s.phi0_deg = bank_from_traj(here, s.tag);
    end
    ics(end+1) = s; %#ok<AGROW>
end
end

% ===========================================================================
function phi_deg = bank_from_traj(here, tag)
phi_deg = 0.0;
% truth32 tags are RRE1-U1 but the per-flight files are RRE1_U1.csv.
f = fullfile(here,'data','traj',[strrep(tag,'-','_') '.csv']);
if ~isfile(f)
    % Loud, not silent: a missing file here used to return 0 for all 32
    % airframes and look exactly like "the aircraft were all wings level".
    error('stot_ic32:notraj','no trajectory file for %s (%s)', tag, f);
end
A = readtable(f);
% Use the de-quantised clock: the raw t column is quantised to 0.1 s and ~50 %
% of adjacent rows share a stamp, so differentiating on t alone is meaningless.
t  = A.t_dq;  la = A.lat;  lo = A.lon;  gs = A.gs;
k  = find(A.t_rel >= 0, 1, 'first');
if isempty(k)
    error('stot_ic32:norelease','%s has no row at or after release', tag);
end
w  = (t >= t(k) - 0.1) & (t <= t(k) + 1.2);
idx = find(w);
if numel(idx) < 5
    error('stot_ic32:shortwindow','%s: only %d rows in the release window', tag, numel(idx));
end
lat0 = mean(la(idx));
mlon = 111320*cosd(lat0);
n = (la(idx)-lat0)*111320;
e = (lo(idx)-mean(lo(idx)))*mlon;
% course, unwrapped, then a straight-line fit for the rate
chi = unwrap(atan2(diff(e), diff(n)));
tm  = 0.5*(t(idx(1:end-1)) + t(idx(2:end)));
if numel(chi) < 4 || (max(tm)-min(tm)) < 0.2
    error('stot_ic32:shortfit','%s: cannot fit a course rate at release', tag);
end
pfit = polyfit(tm, chi, 1);
chidot = pfit(1);
vh = mean(gs(idx));
phi = atan(chidot*vh/9.80665);
assert(isfinite(phi), 'stot_ic32: non-finite bank estimate for %s', tag);
phi_deg = rad2deg(max(min(phi, deg2rad(80)), deg2rad(-80)));
end
