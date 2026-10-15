function L = parseStream(src)
%MGS.PARSESTREAM  A recorded WSL stream -> per-vehicle time series.
%
%   L = mgs.parseStream('runs/shadow/RRX.ndjson')
%   L = mgs.parseStream(cellOfMessages)
%
%   L.meta        the _meta header, if the file has one
%   L.tgt         .lat .lon .alt from the last /swarm/tot_gate seen
%   L.uav(i)      per-vehicle table, one row per 'state' message:
%                   t lat lon amsl vn ve vd
%                   roll_deg pitch_deg yaw_deg
%                   cmd_roll_deg cmd_pitch_deg cmd_thrust       (command layer)
%                   nav_roll_deg nav_pitch_deg aspd_error       (clamp layer)
%                   throttle_pc airspeed groundspeed             (VFR_HUD)
%                   age_s
%   L.status      table of tot_status: t phase T_arr  (SWARM-level phase)
%   L.mode        table of PER-VEHICLE mode: t id mode  (climb/hold_ring/dive/...)
%   L.have        which layers are actually present -- see below
%
% ---------------------------------------------------------------------------
%   L.have IS NOT COSMETIC.  A recording taken without mavros_msgs has no
%   command layer, and every command-layer column comes back NaN.  A diff that
%   then reported "no divergence in the guidance layer" would be reporting the
%   absence of data as the absence of a problem.  mgs.shadowDiff refuses to
%   score a layer whose L.have flag is false.
%
%   See also MGS.SHADOWDIFF, MGS.RECORDSTREAM.

if ischar(src) || isstring(src)
    txt = fileread(char(src));
    lines = strsplit(txt, newline);
    msgs = {};
    for k = 1:numel(lines)
        s = strtrim(lines{k});
        if isempty(s), continue; end
        try
            msgs{end+1} = jsondecode(s); %#ok<AGROW>
        catch
            warning('mgs:parseStream:badline', 'line %d is not JSON, skipped', k);
        end
    end
elseif iscell(src)
    msgs = src;
else
    error('mgs:parseStream:type', 'src must be a path or a cell of messages');
end

L = struct('meta', struct(), 'tgt', struct('lat', NaN, 'lon', NaN, 'alt', NaN), ...
           'uav', struct([]), 'status', table(), 'have', struct());

COLS = {'t','lat','lon','amsl','vn','ve','vd','V', ...
        'roll_deg','pitch_deg','yaw_deg', ...
        'cmd_roll_deg','cmd_pitch_deg','cmd_thrust', ...
        'nav_roll_deg','nav_pitch_deg','aspd_error','alt_error', ...
        'throttle_pc','airspeed','groundspeed','age_s'};
acc = containers.Map('KeyType', 'double', 'ValueType', 'any');
stT = []; stPhase = {}; stArr = [];
modeT = []; modeId = []; modeS = {};

for k = 1:numel(msgs)
    m = msgs{k};
    if ~isstruct(m) || ~isfield(m, 'type'), continue; end
    switch m.type
        case '_meta'
            L.meta = m;

        case 'gate'
            if isfield(m, 'data') && isstruct(m.data) && isfield(m.data, 'target') ...
                    && isstruct(m.data.target)
                L.tgt.lat = getf(m.data.target, 'lat', NaN);
                L.tgt.lon = getf(m.data.target, 'lon', NaN);
                L.tgt.alt = getf(m.data.target, 'alt', NaN);
            end

        case 'status'
            if isfield(m, 'data') && isstruct(m.data)
                stT(end+1,1) = getf(m, 't', NaN); %#ok<AGROW>
                stPhase{end+1,1} = char(string(getf(m.data, 'phase', ''))); %#ok<AGROW>
                a = getf(m.data, 'T_arr', NaN);
                if isempty(a), a = NaN; end
                stArr(end+1,1) = double(a); %#ok<AGROW>
                % PER-VEHICLE mode, kept separately from the swarm phase.
                % tot_node.py:4448 flips `self.phase` to DIVE when the FIRST
                % aircraft dives, so the swarm phase says nothing about when
                % vehicle 2 released.  The per-vehicle mode does.
                % ⚠️ And mode=="dive" is still ~8 s EARLIER than the first PN
                % command (median 8.1 s, 16/16) -- that gap is flown by
                % L1/TECS.  Use it to MEASURE that gap, not as the handover.
                u = getf(m.data, 'uavs', struct());
                if isstruct(u) && ~isempty(fieldnames(u))
                    f = fieldnames(u);
                    for q = 1:numel(f)
                        id = str2double(regexprep(f{q}, '^x', ''));
                        md = '';
                        if isstruct(u.(f{q})) && isfield(u.(f{q}), 'mode')
                            md = char(string(u.(f{q}).mode));
                        end
                        if isfinite(id) && ~isempty(md)
                            modeT(end+1,1) = getf(m, 't', NaN); %#ok<AGROW>
                            modeId(end+1,1) = id; %#ok<AGROW>
                            modeS{end+1,1} = md; %#ok<AGROW>
                        end
                    end
                end
            end

        case 'state'
            if ~isfield(m, 'uavs'), continue; end
            U = m.uavs;
            if isstruct(U) && numel(U) >= 1
                for j = 1:numel(U)
                    add_row(U(j), getf(m, 't', NaN));
                end
            elseif iscell(U)
                for j = 1:numel(U)
                    add_row(U{j}, getf(m, 't', NaN));
                end
            end
    end
end

ids = sort(cell2mat(acc.keys));
for j = 1:numel(ids)
    R = acc(ids(j));
    L.uav(j).id = ids(j);
    L.uav(j).tbl = array2table(R, 'VariableNames', COLS);
end
if ~isempty(stT)
    L.status = table(stT, stPhase, stArr, 'VariableNames', {'t','phase','T_arr'});
end
if ~isempty(modeT)
    L.mode = table(modeT, modeId, modeS, 'VariableNames', {'t','id','mode'});
else
    L.mode = table();
end

% ---- which layers actually have data ---------------------------------------
L.have.state    = ~isempty(L.uav);
L.have.command  = any_finite(L, 'cmd_pitch_deg');
L.have.clamp    = any_finite(L, 'nav_pitch_deg');
L.have.throttle = any_finite(L, 'throttle_pc');
L.have.attitude = any_finite(L, 'pitch_deg');
L.have.target   = isfinite(L.tgt.lat) && isfinite(L.tgt.lon);
L.have.status   = ~isempty(stT);

    function add_row(u, tmsg)
        if ~isstruct(u) || ~isfield(u, 'id'), return; end
        id = double(u.id);
        if ~isKey(acc, id), acc(id) = zeros(0, numel(COLS)); end
        r = nan(1, numel(COLS));
        r(1) = pick(u, 't', tmsg);
        for q = 2:numel(COLS)
            r(q) = pick(u, COLS{q}, NaN);
        end
        % V is derived when the bridge did not send it
        if isnan(r(8)) && all(isfinite(r(5:7)))
            r(8) = norm(r(5:7));
        end
        acc(id) = [acc(id); r];
    end
end

% -----------------------------------------------------------------------------
function tf = any_finite(L, col)
tf = false;
for j = 1:numel(L.uav)
    T = L.uav(j).tbl;
    if ismember(col, T.Properties.VariableNames) && any(isfinite(T.(col)))
        tf = true; return;
    end
end
end

function v = pick(s, f, dflt)
if isstruct(s) && isfield(s, f) && ~isempty(s.(f)) && isnumeric(s.(f))
    v = double(s.(f));
else
    v = dflt;
end
end

function v = getf(s, f, dflt)
if isstruct(s) && isfield(s, f), v = s.(f); else, v = dflt; end
end
