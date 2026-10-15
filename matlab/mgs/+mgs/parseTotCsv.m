function C = parseTotCsv(path)
%MGS.PARSETOTCSV  tot_node's own per-tick log -> per-vehicle tables.
%
%   C = mgs.parseTotCsv('runs/shadow/tot_BASE2.csv')
%
%   C.veh(k).id    sysid
%   C.veh(k).tbl   the node's own record, one row per tick per vehicle
%   C.cols         every column name found
%   C.have         which of the columns mgs.shadowDiff needs are populated
%
% ---------------------------------------------------------------------------
%   WHY THIS FILE AND NOT MAVLINK
%
%   The guidance layer needs what the NODE COMMANDED, and there is no way to
%   get that over MAVLink on this vehicle:
%
%     - tot_node sends SET_ATTITUDE_TARGET on its OWN direct flight-controller
%       link, so MAVROS never sees the command go out.
%     - ATTITUDE_TARGET (msg 83), which would report it back, is accepted by
%       ArduPlane (`set_message_interval` returns success=True) and then NEVER
%       SENT -- measured over a whole sortie including the dive.  Accepted but
%       unimplemented, and from the ROS side indistinguishable from a quiet
%       topic.
%
%   The node's own CSV has it, at the node's own tick, with no resampling:
%
%     req_nz        the demand BEFORE the [nz_min, nz_max] clamp
%     cmd_nz        after it
%     nz_max_true   the aerodynamic ceiling
%     gam_deg v_cmd agl lat lon alt gs
%
%   So the guidance layer is compared on nz, not on a pitch command.  That is
%   the better comparison anyway: nz is what the law actually computes, and
%   the pitch command is a downstream conversion through the alpha low-pass.
%
%   ⚠️ `alpha_cmd_deg` exists as a column but is EMPTY in this configuration
%   (0 of 1303 dive rows).  C.have tells you which columns really carry data
%   so an empty column cannot be mistaken for a zero one.
%
%   ⚠️ THE TIME COLUMN IS RELATIVE, not Unix: it starts at 0 when the node
%   starts.  mgs.shadowDiff aligns it against the recording; see ALIGNTOTCSV.
%
%   See also MGS.SHADOWDIFF, MGS.PARSESTREAM.

if ~isfile(path)
    error('mgs:parseTotCsv:missing', 'no such file: %s', path);
end

T = readtable(path, 'TextType', 'char');
C = struct();
C.path = path;
C.cols = T.Properties.VariableNames;

need = {'t', 'sysid', 'mode', 'lat', 'lon', 'alt', 'gs', 'gam_deg', 'agl', ...
        'req_nz', 'cmd_nz'};
miss = need(~ismember(need, C.cols));
if ~isempty(miss)
    error('mgs:parseTotCsv:cols', ...
        ['%s is missing %s.  Refusing to guess: a tot CSV without these is ' ...
         'a different schema and every downstream number would be silently ' ...
         'wrong.'], path, strjoin(miss, ', '));
end

sysid = T.sysid;
if iscell(sysid), sysid = cellfun(@str2double, sysid); end
ids = unique(sysid(isfinite(sysid)));

C.veh = struct('id', {}, 'tbl', {});
for k = 1:numel(ids)
    sel = sysid == ids(k);
    C.veh(k).id = ids(k);
    C.veh(k).tbl = T(sel, :);
end

% ---- which of the columns we care about actually carry data ---------------
% An all-empty column reads as zeros once str2double gets hold of it, and a
% zero nz demand is a perfectly legal value -- so "empty" and "zero" have to
% be told apart HERE, not downstream.
C.have = struct();
for f = {'req_nz', 'cmd_nz', 'nz_max_true', 'alpha_cmd_deg', 'alpha_est_deg', ...
         'gam_deg', 'v_cmd', 'agl'}
    nm = f{1};
    if ~ismember(nm, C.cols), C.have.(nm) = false; continue; end
    v = num_col(T, nm);
    C.have.(nm) = any(isfinite(v));
end
C.n_rows = height(T);
C.t_span = [min(num_col(T, 't')), max(num_col(T, 't'))];
end

% -----------------------------------------------------------------------------
function v = num_col(T, name)
c = T.(name);
if isnumeric(c), v = double(c); return; end
if iscell(c)
    v = nan(numel(c), 1);
    for k = 1:numel(c)
        s = strtrim(char(c{k}));
        if isempty(s), continue; end
        d = str2double(s);
        if ~isnan(d), v(k) = d; end
    end
    return;
end
v = double(c);
end
