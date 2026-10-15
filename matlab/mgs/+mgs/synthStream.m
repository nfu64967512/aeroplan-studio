function [path, truth] = synthStream(path, c, opts)
%MGS.SYNTHSTREAM  A fake live recording with a KNOWN divergence, for testing.
%
%   [path, truth] = mgs.synthStream('out/synth.ndjson', c, ...
%                       'inject', 'clamp', 'at_s', 15, 'deg', 6)
%
%   Runs the model, converts its trajectory back to lat/lon, and writes it out
%   in exactly the format mgs_bridge_node.py emits.  Then injects a known
%   perturbation into ONE layer starting at a known time.
%
%   truth.layer   which layer was perturbed  ('guidance' | 'clamp' | 'plant')
%   truth.at_s    when, in seconds after release
%   truth.deg     by how much
%
% ---------------------------------------------------------------------------
%   WHY THIS EXISTS
%
%   mgs.shadowDiff's whole job is ATTRIBUTION -- saying which layer parted
%   company first.  A tool that attributes cannot be trusted just because it
%   runs; it has to be shown to get a known answer right.  Without the live
%   WSL stack there is no other way to check that, and shipping an unchecked
%   attribution tool into a project whose central problem is plausible-looking
%   wrong answers would be a poor trade.
%
%   The stream is generated from the model itself, so with no injection the
%   diff must report ZERO divergence in every layer.  That is the null test,
%   and it is at least as important as the injection test: if the null case
%   shows divergence, the frame conversion or the resampling is broken.
%
%   See also MGS.SHADOWDIFF, MGS.SELFTEST.

arguments
    path (1,:) char
    c = mgs.params()
    opts.inject (1,:) char = 'none'      % none | guidance | clamp | plant
    opts.at_s (1,1) double = 15.0
    opts.deg (1,1) double = 6.0
    opts.liveParams (1,:) cell = {}      % see LIVEPARAMS below
    opts.tgt_lat (1,1) double = 23.5
    opts.tgt_lon (1,1) double = 119.5
    opts.hz (1,1) double = 20            % >= the node's 20 Hz tick; see
                                         % the note in mgs.shadowDiff
end

c1 = c; c1.swarm.n = 1;
scn = mgs.scenario(c1, 'tag', 'SYNTH');

% LIVEPARAMS: generate the "live" side from a model with DIFFERENT parameters.
% This is the physically consistent injection, and the one closest to what a
% real divergence looks like: change the inner-loop lag or the drag and the
% whole trajectory responds, so gamma moves too.
%
% The additive injections below (guidance/clamp/plant) are kinematically
% INCONSISTENT on purpose -- they bump an angle without re-integrating, which
% is what makes them a clean unit test of the attribution arithmetic.  Their
% cost is that L3b (flight path angle) stays at 0.00, because velocity was
% never touched.  Do not read that as "gamma agreed".
if ~isempty(opts.liveParams)
    cLive = mgs.params(opts.liveParams{:});
    cLive.swarm.n = 1;
    MLive = mgs.simSwarm(mgs.scenario(cLive, 'tag', 'SYNTH'), cLive, 'verbose', false);
    T = MLive.veh(1).traj;
    icForCmd = scn.ic(1);
    cForCmd = cLive;
else
    M = mgs.simSwarm(scn, c1, 'verbose', false);
    T = M.veh(1).traj;
    icForCmd = scn.ic(1);
    cForCmd = c1;
end

% the pre-clamp command, recovered the same way shadowDiff does
[cmdPitch, cmdRoll] = replay_cmd(T, cForCmd, icForCmd);

t0 = 1.7e9;                                   % an arbitrary but fixed epoch
dt = 1/opts.hz;
ts = (0:dt:max(T.t)).';

get = @(col) interp1(T.t, T.(col), ts, 'linear', 'extrap');
n = get('n'); e = get('e'); z = get('z');
V = get('V'); gam = get('gam_deg'); psi = get('psi_deg');
th = get('th_deg'); phi = get('phi_deg');
navP = get('nav_pitch_deg'); navR = get('nav_roll_deg');
thrPc = get('thr_pc');
cmdP = interp1(T.t, cmdPitch, ts, 'linear', 'extrap');
cmdR = interp1(T.t, cmdRoll,  ts, 'linear', 'extrap');

% ---- inject -----------------------------------------------------------------
mask = ts >= opts.at_s;
switch lower(opts.inject)
    case 'none'
    case 'guidance'
        % the node asked for something different: perturb the pre-clamp command
        % AND everything downstream of it, because in a real vehicle it would
        % propagate.  A perturbation that stops at one layer is not a test of
        % attribution, it is a test of arithmetic.
        cmdP(mask) = cmdP(mask) + opts.deg;
        navP(mask) = navP(mask) + opts.deg;
        th(mask)   = th(mask)   + opts.deg;
    case 'clamp'
        navP(mask) = navP(mask) + opts.deg;
        th(mask)   = th(mask)   + opts.deg;
    case 'plant'
        th(mask)   = th(mask)   + opts.deg;
    otherwise
        error('mgs:synthStream:inject', ...
            'inject must be none | guidance | clamp | plant (got "%s")', opts.inject);
end

% ---- back to lat/lon, with the SAME flat-earth constant -------------------
EARTH = 111320.0;
lat = opts.tgt_lat + n/EARTH;
lon = opts.tgt_lon + e./(EARTH*cos(deg2rad(lat)));

vn = V.*cosd(gam).*cosd(psi);
ve = V.*cosd(gam).*sind(psi);
vd = -V.*sind(gam);

d = fileparts(path);
if ~isempty(d) && ~isfolder(d), mkdir(d); end
fid = fopen(path, 'w', 'n', 'UTF-8');
closer = onCleanup(@() fclose(fid));

wl(fid, struct('type','_meta','t',t0,'schema',1, ...
               'synthetic',true,'inject',opts.inject,'at_s',opts.at_s, ...
               'deg',opts.deg, ...
               'liveParams', strjoin(cellfun(@(x) char(string(x)), ...
                                             opts.liveParams, 'UniformOutput', false), ' ')));
wl(fid, struct('type','gate','t',t0, 'data', ...
    struct('target', struct('lat',opts.tgt_lat,'lon',opts.tgt_lon,'alt',c.swarm.tgt_amsl))));
wl(fid, struct('type','status','t',t0, ...
    'data', struct('phase','DIVE','T_arr', max(T.t))));

for k = 1:numel(ts)
    u = struct('id',1,'lat',lat(k),'lon',lon(k),'amsl',z(k)+c.swarm.tgt_amsl, ...
               'vn',vn(k),'ve',ve(k),'vd',vd(k),'V',V(k), ...
               'roll_deg',phi(k),'pitch_deg',th(k),'yaw_deg',psi(k), ...
               'cmd_roll_deg',cmdR(k),'cmd_pitch_deg',cmdP(k),'cmd_thrust',thrPc(k)/100,'throttle_pc',thrPc(k), ...
               'nav_roll_deg',navR(k),'nav_pitch_deg',navP(k), ...
               'aspd_error',0.0,'alt_error',0.0,'age_s',0.05);
    wl(fid, struct('type','state','t',t0+ts(k),'uavs',u));
end

truth = struct('layer', lower(opts.inject), 'at_s', opts.at_s, ...
               'deg', opts.deg, 'n_samples', numel(ts), 'path', path, ...
               'liveParams', {opts.liveParams});
end

% -----------------------------------------------------------------------------
function wl(fid, s)
fprintf(fid, '%s\n', jsonencode(s));
end

function [pitch_deg, roll_deg] = replay_cmd(mt, c, ic)
n = height(mt);
pitch_deg = nan(n,1); roll_deg = nan(n,1);
gs = mgs.guidance(); gs.dive_T = ic.dive_T; gs.phase = 'dive';
for k = 1:n
    V = mt.V(k); gam = deg2rad(mt.gam_deg(k)); psi = deg2rad(mt.psi_deg(k));
    m = struct('n', mt.n(k), 'e', mt.e(k), 'z', mt.z(k), ...
               'vn', V*cos(gam)*cos(psi), 've', V*cos(gam)*sin(psi), ...
               'vd', -V*sin(gam), 'pitch', deg2rad(mt.th_deg(k)));
    m.d = max(1e-3, hypot(m.n, m.e)); m.h = m.z;
    m.R = max(1e-3, hypot(m.d, m.h)); m.vh = hypot(m.vn, m.ve);
    m.lam_dot = 0; m.sig_dot = 0; m.Vc = 0;
    [cmd, gs] = mgs.guidance(m, gs, mt.t(k), c);
    pitch_deg(k) = rad2deg(cmd.pitch);
    roll_deg(k)  = rad2deg(cmd.roll);
end
end
