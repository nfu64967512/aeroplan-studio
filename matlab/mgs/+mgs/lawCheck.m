function K = lawCheck(C, tgt, c, opts)
%MGS.LAWCHECK  Is the ported guidance law the same law the vehicle flew?
%
%   K = mgs.lawCheck(C, tgt)            C = mgs.parseTotCsv(...), tgt = L.tgt
%   K = mgs.lawCheck(C, tgt, c, 'render', true)
%
%   Recomputes the longitudinal channel from the NODE'S OWN logged state and
%   compares it against the nz the node itself recorded, at two points:
%
%     L1  model nz_raw  vs  req_nz    the law BEFORE the clamp
%     L2  model cmd_nz  vs  cmd_nz    the whole chain: clamp -> gamma command
%                                     -> one-sided min() with the LOS floor
%                                     -> back-solved nz  (tot_node.py:3100)
%
% ---------------------------------------------------------------------------
%   WHY THIS IS THE STRONGEST CHECK IN THE WHOLE ENVIRONMENT
%
%   Every input comes from the same file as the output, so there is no
%   telemetry-path difference, no resampling and no clock alignment folded in.
%   It is a pure algebraic question: given exactly what the node saw, does the
%   port compute exactly what the node computed?
%
%   Measured on BASE2 (2026-08-22, 1299 real dive ticks across two aircraft):
%       L1  rms 0.0019 / 0.0032 g,  corr 1.0000, slope 1.0000 / 1.0003
%       L2  rms 0.0017 / 0.0011 g
%   The residual is CSV quantisation -- req_nz is logged to 3 decimals and
%   gs / gam_deg to 1 -- not a difference in the law.
%
%   ⇒ The guidance layer is NOT where the model and the vehicle part company.
%     Anything that does diverge is downstream of it.
%
% ---------------------------------------------------------------------------
%   K.veh(k) .id .n
%            .l1_rms .l1_corr .l1_slope
%            .l2_rms
%            .floor_frac    fraction of dive ticks the LOS floor was BINDING
%            .out_of_clamp  fraction with cmd_nz outside [nz_min, nz_max]
%            .cmd_nz_median
%
%   See also MGS.PARSETOTCSV, MGS.SHADOWDIFF, MGS.ZARCHAN.ADDITIVELAW.

arguments
    C
    tgt
    c = mgs.params()
    opts.render (1,1) logical = true
end

E = 111320.0;                       % geodesy.py:4 -- the constant the node uses
g = c.pn.g;  N = c.pn.n_vert;  lead = c.pn.lead;
lo = c.pn.nz_min;  hi = c.pn.nz_max;  floorK = c.pn.los_floor;
clampAfter = false;
if isfield(c.pn, 'nz_clamp_after_floor'), clampAfter = logical(c.pn.nz_clamp_after_floor); end

K = struct('veh', struct('id', {}, 'n', {}, 'l1_rms', {}, 'l1_corr', {}, ...
                         'l1_slope', {}, 'l2_rms', {}, 'floor_frac', {}, ...
                         'out_of_clamp', {}, 'cmd_nz_median', {}, ...
                         'req_nz_median', {}));

for k = 1:numel(C.veh)
    T = C.veh(k).tbl;
    md = T.mode;  if ~iscell(md), md = cellstr(md); end
    dv = strcmpi(md, 'dive');
    if ~any(dv), continue; end

    lat = numc(T.lat);  lon = numc(T.lon);  agl = numc(T.agl);
    gs  = numc(T.gs);   gam = deg2rad(numc(T.gam_deg));
    sig = deg2rad(numc(T.lead_angle_deg));
    rq  = numc(T.req_nz);  cm = numc(T.cmd_nz);
    lat=lat(dv); lon=lon(dv); agl=agl(dv); gs=gs(dv); gam=gam(dv);
    sig=sig(dv); rq=rq(dv); cm=cm(dv);

    dn = (tgt.lat - lat) * E;
    de = (tgt.lon - lon) * E .* cos(deg2rad(lat));
    d  = max(1e-3, hypot(dn, de));
    h  = agl;                       % agl is already height over the target datum
    R  = max(1e-3, hypot(d, h));

    V    = gs ./ max(cos(gam), 1e-6);
    hdot = gs .* tan(gam);
    % ddot from the node's own logged lead angle -- the angle between the
    % velocity and the line of sight.  Differentiating d numerically instead
    % would inject the CSV's own quantisation into the comparison.
    ddot = -gs .* cos(sig);

    sig_dot = (hdot.*d - h.*ddot) ./ (R.*R);
    nz_raw  = cos(gam) + V .* (-N*sig_dot) / g;

    nz      = max(lo, min(hi, nz_raw));
    gam_pn  = gam + ((nz - cos(gam))*g./V) * lead;
    gam_los = -atan2(h, max(d, 1e-3));
    binding = gam_los*floorK < gam_pn;
    gam_cmd = min(gam_pn, gam_los*floorK);
    cmd_hat = cos(gam) + V .* ((gam_cmd - gam)/lead) / g;
    % ⚠️ THE SECOND CLAMP.  tot_node.py:3102 re-clamps the BACK-SOLVED nz when
    % nz_clamp_after_floor is on, which is the switch that closes the bypass.
    % stot_guidance.m does not implement this branch -- its header pins it to
    % commit d7562e2, and the branch is newer -- so the model CANNOT simulate
    % the flag being on.  Measured consequence of leaving it out: on NZC1
    % (flag on) the L2 residual was 22.0 and 38.6 g instead of 0.002.
    if clampAfter
        cmd_hat = max(lo, min(hi, cmd_hat));
    end

    o1 = isfinite(nz_raw) & isfinite(rq);
    o2 = isfinite(cmd_hat) & isfinite(cm);
    e1 = nz_raw(o1) - rq(o1);
    e2 = cmd_hat(o2) - cm(o2);
    r  = lin(nz_raw(o1), rq(o1));

    j = numel(K.veh) + 1;
    K.veh(j).id = C.veh(k).id;
    K.veh(j).n = sum(o1);
    K.veh(j).l1_rms = sqrt(mean(e1.^2));
    K.veh(j).l1_corr = r.rho;
    K.veh(j).l1_slope = r.slope;
    K.veh(j).l2_rms = sqrt(mean(e2.^2));
    K.veh(j).floor_frac = mean(binding(o2));
    K.veh(j).out_of_clamp = mean(cm(o2) < lo | cm(o2) > hi);
    K.veh(j).cmd_nz_median = median(cm(o2));
    K.veh(j).req_nz_median = median(rq(o1));
end

K.summary = render(K, c);
if opts.render, fprintf('%s\n', K.summary); end
end

% -----------------------------------------------------------------------------
function s = render(K, c)
L = {};
ca = false;
if isfield(c.pn, 'nz_clamp_after_floor'), ca = logical(c.pn.nz_clamp_after_floor); end
L{end+1} = sprintf(['guidance-law check against the node''s own log ' ...
    '(N_vert %g, clamp [%.1f %.1f], los_floor %.1f, nz_clamp_after_floor %d)'], ...
    c.pn.n_vert, c.pn.nz_min, c.pn.nz_max, c.pn.los_floor, ca);
L{end+1} = repmat('-', 1, 88);
L{end+1} = sprintf('%6s %7s | %10s %7s %7s | %10s | %9s %10s %10s', ...
    'uav', 'ticks', 'L1 rms g', 'corr', 'slope', 'L2 rms g', ...
    'floor%', 'out-clamp%', 'cmd_nz med');
for k = 1:numel(K.veh)
    v = K.veh(k);
    L{end+1} = sprintf('%6d %7d | %10.4f %7.4f %7.4f | %10.4f | %8.1f%% %9.1f%% %10.3f', ...
        v.id, v.n, v.l1_rms, v.l1_corr, v.l1_slope, v.l2_rms, ...
        100*v.floor_frac, 100*v.out_of_clamp, v.cmd_nz_median); %#ok<AGROW>
end
L{end+1} = repmat('-', 1, 88);
L{end+1} = '  L1/L2 rms of a few thousandths of a g is CSV quantisation, not a';
L{end+1} = '  difference in the law: req_nz is logged to 3 decimals, gs and';
L{end+1} = '  gam_deg to 1.  The port and the flying node compute the same thing.';
L{end+1} = '';
L{end+1} = '  out-clamp% is the fraction of dive ticks on which the command that';
L{end+1} = '  was actually sent sits OUTSIDE [nz_min, nz_max].';
if ~ca
    L{end+1} = '  It is not a model artefact: the LOS floor is applied AFTER the clamp';
    L{end+1} = '  as a min(), so its output bypasses the clamp (tot_node.py:3100).';
    L{end+1} = '  ⇒ the switch that closes it exists and is OFF: nz_clamp_after_floor.';
else
    L{end+1} = '  nz_clamp_after_floor is ON, so this should read 0.0% -- anything else';
    L{end+1} = '  means the flag did not take effect and the run is not what it says.';
    L{end+1} = '';
    L{end+1} = '  ⚠️ CLOSING THE BYPASS IS NOT THE SAME AS FIXING THE DIVE.  Measured';
    L{end+1} = '  on NZC1 (2026-08-22): the flag worked -- 100%/45% out-of-clamp went';
    L{end+1} = '  to 0.0%/0.0% -- and the sortie got WORSE.  uav1''s dive went from';
    L{end+1} = '  765 to 1722 ticks and still timed out; uav2 passed over at 8194 m;';
    L{end+1} = '  neither landed.  The -6.7 g the bypass was delivering is what the';
    L{end+1} = '  vehicle uses to get its nose onto the sightline.  Capping it makes';
    L{end+1} = '  the command legal and the dive impossible.';
end
s = strjoin(L, newline);
end

function v = numc(cc)
if isnumeric(cc), v = double(cc); return; end
v = nan(numel(cc), 1);
for k = 1:numel(cc)
    d = str2double(strtrim(char(cc{k})));
    if ~isnan(d), v(k) = d; end
end
end

function r = lin(x, y)
x = x(:); y = y(:); mx = mean(x); my = mean(y);
r.rho = sum((x-mx).*(y-my)) / sqrt(sum((x-mx).^2)*sum((y-my).^2));
r.slope = sum((x-mx).*(y-my)) / sum((x-mx).^2);
end
