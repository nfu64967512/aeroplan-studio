function [c, rep] = fromContract(src, opts)
%MGS.FROMCONTRACT  Build sim parameters FROM a flight's effective-value readback.
%
%   c = mgs.fromContract('RRE8')                      a trial in runs/artifacts
%   c = mgs.fromContract('path/to/contract_X.json')   an explicit file
%   c = mgs.fromContract(struct('gate_r',4700, ...))  a hypothetical setting
%   [c, rep] = mgs.fromContract(..., 'strict', false) do not abort on violations
%
%   This is the answer to "if I set it like THIS, how does MATLAB run it?".
%   The same JSON that scripts/wsl/contract_verify.py reads back off the live
%   node and flight controller is what parameterises the model, so a run in
%   here and a run out there are talking about the same numbers.
%
%   THREE THINGS HAPPEN, IN THIS ORDER, AND NONE OF THEM IS OPTIONAL:
%
%   1. The values are scored against the contract table (mgs.contractCheck).
%      Foam-era defaults, out-of-band values and non-numerics abort by default.
%
%   2. Mapped names are written into the parameter struct.
%
%   3. UNMAPPED names are recorded in c.meta.contract.unmapped and, if
%      'warnUnmapped' is true (the default), warned about by name.
%      *** This is the point of the function. ***  "I set dive_pace and the
%      model ignored it" is the same failure as "I set dive_pace and the node
%      ignored it", and it has to be just as loud.  A parameter this model
%      does not consume is stated, never dropped.
%
%   c.meta.contract  .source .values .sources .mapped .unmapped .report
%
%   See also MGS.CONTRACTCHECK, MGS.PARAMS, MGS.SIMSWARM.

arguments
    src
    opts.strict (1,1) logical = true
    opts.warnUnmapped (1,1) logical = true
    opts.base = []
end

[values, sources, srcName] = load_values(src);

% ---- 1. gate ---------------------------------------------------------------
rep = mgs.contractCheck(values, 'strict', opts.strict);

% ---- 2. map ----------------------------------------------------------------
if isempty(opts.base), c = mgs.params(); else, c = opts.base; end
M = mapping();
mapped = {};  unmapped = {};
names = fieldnames(values);

for k = 1:numel(names)
    nm = names{k};
    if ~isfield(M, nm)
        unmapped{end+1} = nm; %#ok<AGROW>
        continue;
    end
    spec = M.(nm);
    [x, ok] = to_double(values.(nm));
    if ~ok
        unmapped{end+1} = nm; %#ok<AGROW>
        continue;
    end
    if ~isempty(spec.xform), x = spec.xform(x); end
    parts = strsplit(spec.path, '.');
    c = setfield(c, parts{:}, x); %#ok<SFLD>
    mapped{end+1} = nm; %#ok<AGROW>
end

% ---- 3. account for everything --------------------------------------------
c.meta.contract = struct( ...
    'source',   srcName, ...
    'values',   values, ...
    'sources',  sources, ...
    'mapped',   {mapped}, ...
    'unmapped', {unmapped}, ...
    'report',   rep);

if opts.warnUnmapped && ~isempty(unmapped)
    warning('mgs:fromContract:unmapped', ...
        ['%d of %d contract values are NOT consumed by this model and were ' ...
         'DROPPED:\n    %s\n' ...
         'They are listed in c.meta.contract.unmapped.  If one of them is ' ...
         'the knob you are studying, this model cannot answer the question ' ...
         'and you must not read its output as if it could.'], ...
        numel(unmapped), numel(names), strjoin(unmapped, ', '));
end
end

% =============================================================================
function M = mapping()
%MAPPING  contract parameter name -> where it lands in the parameter struct.
%
%   Every entry is a deliberate claim that this model actually USES the value.
%   Do not add a row to silence the unmapped warning: an entry that writes a
%   field nothing reads is worse than the warning, because it looks handled.

M = struct();

% ---- guidance law: consumed by stot_guidance, line for line ----------------
M.pn_los_floor = e('pn.los_floor');
M.pn_nz_min    = e('pn.nz_min');
M.pn_vmax      = e('pn.vmax');
M.itcg_on      = e('pn.itcg_on', @(x) logical(x));

% ---- flight controller: consumed by stot_fcs_clamp ------------------------
M.THR_MAX      = e('fc.THR_MAX');
M.AIRSPEED_MIN = e('fc.AIRSPEED_MIN');
M.AIRSPEED_MAX = e('fc.AIRSPEED_MAX');

% ---- release geometry: consumed by mgs.scenario ---------------------------
M.gate_r          = e('up.gate_r');
M.hold_radius     = e('up.ring_r');
M.vmin            = e('up.ring_v');
M.release_at_ring = e('up.release_at_ring', @(x) logical(x));
M.r_fan           = e('scen.r_fan');
M.half_angle      = e('scen.half_angle_deg');
M.perch_alt       = e('scen.z0');
M.dive_angle      = e('scen.dive_angle_deg');
M.dive_speed      = e('scen.dive_speed');
M.vmax            = e('scen.vmax');
end

function s = e(path, xform)
if nargin < 2, xform = []; end
s = struct('path', path, 'xform', xform);
end

% =============================================================================
function [values, sources, srcName] = load_values(src)
sources = struct();
if isstruct(src)
    values = src;  srcName = '<struct>';  return;
end
src = char(src);
p = resolve_path(src);
raw = jsondecode(fileread(p));
if isfield(raw, 'values')
    values = raw.values;
    if isfield(raw, 'sources'), sources = raw.sources; end
else
    % a bare {name: value} map is fine too
    values = raw;
end
srcName = p;
end

function p = resolve_path(src)
if isfile(src), p = src; return; end
here = fileparts(fileparts(mfilename('fullpath')));      % .../matlab/mgs
root = fileparts(fileparts(here));                       % .../aeroplan-studio
cand = { ...
    fullfile(root, src), ...
    fullfile(root, 'runs', 'artifacts', src, ['contract_' src '.json']), ...
    fullfile(root, 'runs', 'artifacts', src, 'contract.json')};
for k = 1:numel(cand)
    if isfile(cand{k}), p = cand{k}; return; end
end
error('mgs:fromContract:notfound', ...
    ['cannot resolve "%s" to a contract file.  Tried:\n    %s'], ...
    src, strjoin(cand, sprintf('\n    ')));
end

function [x, ok] = to_double(v)
ok = true;
if isnumeric(v) && isscalar(v), x = double(v); return; end
if islogical(v) && isscalar(v), x = double(v); return; end
if ischar(v) || isstring(v)
    x = str2double(v);  ok = ~isnan(x);  return;
end
x = NaN; ok = false;
end
