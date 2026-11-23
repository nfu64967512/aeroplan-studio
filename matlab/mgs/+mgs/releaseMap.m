function M = releaseMap(c, opts)
%MGS.RELEASEMAP  How much release dispersion can the terminal law absorb?
%
%   M = mgs.releaseMap()
%   M = mgs.releaseMap(c, 'dz', -1000:250:1000, 'gam0', -27:5:13)
%
%   Sweeps the release state over the dispersion the run-in actually produces
%   and reports, for each cell, what the law DEMANDS.
%
% ---------------------------------------------------------------------------
%   WHY DEMAND AND NOT HIT/MISS
%
%   This model hits essentially every case (median error 0.03 m, 32/32), so
%   hit rate here has no resolution -- and the handoff of 2026-08-21 says the
%   same thing from the flight side: hit rate has no resolution at n = 16
%   either (16/16 against a 26/32 baseline is still only p = 0.16).  Scoring
%   this sweep by hits would produce a uniformly green map that means nothing.
%
%   So the discriminant is the DEMAND side, which is what the handoff says
%   MATLAB is actually for: "given this (already broken) release state, what
%   does the law ASK FOR".  Reported per cell:
%
%     req_p90      Zarchan Ch.2 required/available normal acceleration, 90th
%                  percentile, against the GUIDANCE clamp (the ceiling that
%                  actually binds -- the aerodynamic one is tens of g here)
%     nz_frac      fraction of the dive with the nz clamp on a stop
%     los_frac     fraction of the dive with the one-sided LOS floor in
%                  charge, i.e. the PN loop OPEN
%     z_res        residual altitude at closest passage
%     err          touchdown error, reported but NOT used to grade
%
% ---------------------------------------------------------------------------
%   THE AXES COME FROM MEASUREMENT, NOT FROM ROUND NUMBERS
%
%   dz    release altitude error about the 3100 m perch.  Measured: bimodal,
%         with 14 of 32 sorties between 205 and 1014 m off.  The programme's
%         own acceptance criterion is |alt - 3100| <= 100 m, against a 18/32
%         = 56 % baseline.
%
%   gam0  flight path angle when PN takes over.  This is the discriminator
%         the flight data found: over the 8.1 s between mode=="dive" and the
%         first PN command, the hit group lost a median 81 m and the failure
%         group GAINED 269 m.  At 157 m/s those are gamma ~ -3.7 deg and
%         ~ +12.4 deg.  All 6 observed failures were in the climbing group;
%         all 17 that were descending hit.
%
%   *** WHAT THIS CANNOT DO. ***  The model starts at release, and the root
%   cause (a steep-dive TECS block armed 143-210 s early, TECS_SPDWEIGHT = 0,
%   energy sloshing through the whole 30 km run-in) happens BEFORE that.  This
%   sweep takes the dispersion as an INPUT.  It can say how much of it the
%   terminal law absorbs; it cannot say anything about where it came from, and
%   it cannot be used to argue the run-in does not need fixing.
%
%   See also MGS.ZARCHAN.SIZING, MGS.TIMEAUTHORITY, MGS.SIMSWARM.

arguments
    c = mgs.params()
    opts.dz (1,:) double = -1000:250:1000
    opts.gam0 (1,:) double = [-27 -20 -12 -6 -3.7 0 6 12.4]
    opts.render (1,1) logical = true
end

c1 = c;  c1.swarm.n = 1;
nz_lim = max(abs(c1.pn.nz_max), abs(c1.pn.nz_min));

nD = numel(opts.dz);  nG = numel(opts.gam0);
M = struct();
M.dz = opts.dz;  M.gam0 = opts.gam0;
M.req_p90  = nan(nG, nD);
M.nz_frac  = nan(nG, nD);
M.los_frac = nan(nG, nD);
M.z_res    = nan(nG, nD);
M.err      = nan(nG, nD);
M.t_dive   = nan(nG, nD);
M.reached  = false(nG, nD);

for a = 1:nG
    for b = 1:nD
        cc = c1;  cc.scen.z0 = 3100 + opts.dz(b);
        scn = mgs.scenario(cc, 'tag', 'RMAP');
        scn.ic(1).gam0_deg = opts.gam0(a);
        scn.ic(1).theta_deg = 0;
        scn.ic(1).eps0_deg = cc.scen.eps0_deg;
        o = mgs.simSwarm(scn, cc, 'verbose', false);
        T = o.veh(1).traj;
        v = o.veh(1);

        gam = deg2rad(T.gam_deg);
        req = abs(T.nz_raw - cos(gam));
        M.req_p90(a,b)  = prc(req, 90) / nz_lim;
        M.nz_frac(a,b)  = mean(T.nz <= cc.pn.nz_min + 1e-9 | ...
                               T.nz >= cc.pn.nz_max - 1e-9, 'omitnan');
        M.los_frac(a,b) = mean(T.los_active, 'omitnan');
        M.z_res(a,b)    = v.z_res_m;
        M.err(a,b)      = v.err_m;
        M.t_dive(a,b)   = v.t_impact;
        M.reached(a,b)  = strcmp(v.end_reason, 'impact');
    end
end

M.summary = render(M, c1);
if opts.render, fprintf('%s\n', M.summary); end
end

% -----------------------------------------------------------------------------
function s = render(M, c)
L = {};
L{end+1} = sprintf(['release-dispersion map -- what the law DEMANDS ' ...
    '(perch %g m, r0 %g m, V0 %g m/s)'], 3100, c.scen.r0, c.scen.V0);
L{end+1} = '';
L{end+1} = 'req/clamp p90  (Zarchan Ch.2 required/available, 1.0 = at the guidance clamp)';
L{end+1} = hdr(M);
for a = 1:numel(M.gam0)
    L{end+1} = row(M.gam0(a), M.req_p90(a,:), '%8.2f'); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = 'LOS floor binding  [% of dive]   -- the PN loop is OPEN on these ticks';
L{end+1} = hdr(M);
for a = 1:numel(M.gam0)
    L{end+1} = row(M.gam0(a), 100*M.los_frac(a,:), '%8.1f'); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = 'nz clamp on a stop [% of dive]';
L{end+1} = hdr(M);
for a = 1:numel(M.gam0)
    L{end+1} = row(M.gam0(a), 100*M.nz_frac(a,:), '%8.1f'); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = 'touchdown error [m]  -- REPORTED, NOT A GRADE.  This model hits everything.';
L{end+1} = hdr(M);
for a = 1:numel(M.gam0)
    L{end+1} = row(M.gam0(a), M.err(a,:), '%8.2f'); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = sprintf('  reached the datum: %d of %d cells', sum(M.reached(:)), numel(M.reached));
s = strjoin(L, newline);
end

function h = hdr(M)
h = sprintf('%8s', 'gam0\dz');
for b = 1:numel(M.dz), h = [h sprintf('%8d', M.dz(b))]; end %#ok<AGROW>
end

function r = row(g, v, fmt)
r = sprintf('%8.1f', g);
for b = 1:numel(v), r = [r sprintf(fmt, v(b))]; end %#ok<AGROW>
end

function p = prc(v, q)
v = sort(v(~isnan(v)));
if isempty(v), p = NaN; return; end
if numel(v) == 1, p = v; return; end
i = 1 + (q/100)*(numel(v)-1);
lo = floor(i); hi = ceil(i);
p = v(lo) + (i-lo)*(v(hi)-v(lo));
end
