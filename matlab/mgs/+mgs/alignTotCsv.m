function A = alignTotCsv(L, C, uavId, opts)
%MGS.ALIGNTOTCSV  Put the node's own CSV on the recording's clock, and say how
%   well the two sources actually agree.
%
%   A = mgs.alignTotCsv(L, C, 1)
%
%   A.offset     [s] add this to the CSV's t to get the recording's Unix t
%   A.pos_rms    [m] horizontal position disagreement after alignment
%   A.pos_p95    [m]
%   A.alt_rms    [m]
%   A.n          overlapping samples used
%   A.ok         true if the two sources agree well enough to trust L1
%   A.why        printable
%
% ---------------------------------------------------------------------------
%   THE RESIDUAL IS THE POINT, NOT THE OFFSET.
%
%   The CSV's t is relative to node start; the recording's t is Unix.  Finding
%   the offset is easy.  What matters is what is LEFT once they are aligned:
%
%   The node computes its nz demand from ITS OWN position latch, taken off a
%   direct 40 Hz flight-controller link.  The recording sees MAVROS, a
%   different link at a different rate.  If those two disagree, then comparing
%   "the model driven by the MAVROS state" against "the nz the node computed
%   from its own state" folds a STATE-SOURCE difference into what would be
%   reported as a guidance-law divergence.
%
%   So this measures the disagreement and refuses to bless L1 when it is large.
%   Reporting a law divergence that is really a telemetry difference is
%   exactly the class of wrong answer this whole tool exists to avoid.
%
%   See also MGS.PARSETOTCSV, MGS.SHADOWDIFF.

arguments
    L
    C
    uavId (1,1) double
    opts.pos_tol_m (1,1) double = 25.0   % see the note on the default
    opts.search_s (1,1) double = 5.0     % refinement window around the coarse offset
end

A = struct('offset', NaN, 'pos_rms', NaN, 'pos_p95', NaN, 'alt_rms', NaN, ...
           'n', 0, 'ok', false, 'why', '');

ic = find([C.veh.id] == uavId, 1);
il = find([L.uav.id] == uavId, 1);
if isempty(ic) || isempty(il)
    A.why = sprintf('uav %d not in both sources', uavId);
    return;
end
Ct = C.veh(ic).tbl;
Lt = L.uav(il).tbl;

ct = numcol(Ct, 't');
clat = numcol(Ct, 'lat');  clon = numcol(Ct, 'lon');  calt = numcol(Ct, 'alt');
ok = isfinite(ct) & isfinite(clat) & isfinite(clon);
ct = ct(ok); clat = clat(ok); clon = clon(ok); calt = calt(ok);
if numel(ct) < 20
    A.why = 'too few usable CSV rows'; return;
end

% ---- coarse: line the two mode==dive transitions up ------------------------
cmode = Ct.mode(ok);
kd = find(strcmpi(cmode, 'dive'), 1, 'first');
if isempty(kd)
    A.why = 'no dive rows in the CSV'; return;
end
tc_dive = ct(kd);

td_dive = NaN;
if isfield(L, 'mode') && ~isempty(L.mode)
    sel = L.mode.id == uavId & strcmpi(L.mode.mode, 'dive');
    if any(sel), td_dive = L.mode.t(find(sel, 1, 'first')); end
end
if ~isfinite(td_dive)
    A.why = ['the recording has no per-vehicle mode==dive to anchor on ' ...
             '(tot_status uavs field missing?)'];
    return;
end
off0 = td_dive - tc_dive;

% ---- refine: minimise horizontal position disagreement --------------------
% Only over the dive, where both sources are dense and the geometry moves fast
% enough for the alignment to be well conditioned.
win = ct >= tc_dive - 5 & ct <= tc_dive + 60;
cs = struct('t', ct(win), 'lat', clat(win), 'lon', clon(win), 'alt', calt(win));

best = struct('off', off0, 'rms', Inf);
for d = -opts.search_s:0.05:opts.search_s
    r = resid(cs, Lt, off0 + d);
    if isfinite(r.rms) && r.rms < best.rms
        best.off = off0 + d;  best.rms = r.rms;
    end
end
r = resid(cs, Lt, best.off);

A.offset  = best.off;
A.pos_rms = r.rms;
A.pos_p95 = r.p95;
A.alt_rms = r.alt_rms;
A.n       = r.n;
A.coarse_offset = off0;
A.ok = isfinite(A.pos_rms) && A.pos_rms <= opts.pos_tol_m;
A.why = sprintf(['offset %+.2f s (coarse %+.2f); position disagreement rms ' ...
    '%.1f m, p95 %.1f m, alt rms %.1f m over %d samples; tolerance %.0f m'], ...
    A.offset, A.coarse_offset, A.pos_rms, A.pos_p95, A.alt_rms, A.n, ...
    opts.pos_tol_m);
if ~A.ok
    A.why = [A.why sprintf([' -- TOO LARGE.  The node and the recording do ' ...
        'not agree on where the aircraft was, so any nz difference between ' ...
        'them would be a state-source artefact as much as a law difference.'])];
end
end

% -----------------------------------------------------------------------------
function r = resid(cs, Lt, off)
r = struct('rms', NaN, 'p95', NaN, 'alt_rms', NaN, 'n', 0);
tt = cs.t + off;
lat = interp1(Lt.t, Lt.lat, tt, 'linear', NaN);
lon = interp1(Lt.t, Lt.lon, tt, 'linear', NaN);
amsl = interp1(Lt.t, Lt.amsl, tt, 'linear', NaN);
ok = isfinite(lat) & isfinite(lon);
if sum(ok) < 20, return; end
% Same flat-earth constant tot_node uses, so this residual is in the frame the
% guidance actually works in.
E = 111320.0;
dn = (cs.lat(ok) - lat(ok)) * E;
de = (cs.lon(ok) - lon(ok)) * E .* cos(deg2rad(lat(ok)));
d = hypot(dn, de);
r.rms = sqrt(mean(d.^2));
r.p95 = prc(d, 95);
r.n = sum(ok);
% alt in the CSV is relative to HOME, amsl in the recording is AMSL -- they
% differ by the home elevation, so compare their VARIATION, not their level.
a = cs.alt(ok) - amsl(ok);
r.alt_rms = std(a, 'omitnan');
end

function p = prc(v, q)
v = sort(v(~isnan(v)));
if isempty(v), p = NaN; return; end
if numel(v) == 1, p = v; return; end
i = 1 + (q/100)*(numel(v)-1);
lo = floor(i); hi = ceil(i);
p = v(lo) + (i-lo)*(v(hi)-v(lo));
end

function v = numcol(T, name)
c = T.(name);
if isnumeric(c), v = double(c); return; end
v = nan(numel(c), 1);
for k = 1:numel(c)
    s = strtrim(char(c{k}));
    if isempty(s), continue; end
    d = str2double(s);
    if ~isnan(d), v(k) = d; end
end
end
