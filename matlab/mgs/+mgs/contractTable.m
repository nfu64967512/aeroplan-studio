function T = contractTable(force)
%MGS.CONTRACTTABLE  Load the effective-value contract table (cached).
%
%   T = mgs.contractTable()        load, cached for the session
%   T = mgs.contractTable(true)    force a re-read
%
%   T.meta       .source .source_sha1 .n_contracts .n_invariants
%   T.byName     containers.Map name -> struct(name,source,lo,hi,foam,why,evidence)
%   T.table      struct array, same rows
%   T.envFor     containers.Map param name -> batch-script env var
%   T.jetRef     containers.Map param name -> jet-configuration reference value
%   T.boolParams cellstr, params expressed as 0/1
%   T.intParams  cellstr
%   T.unproven   cellstr, contracts NOT yet pinned by measurement -- a violation
%                on one of these is a hint, not evidence
%   T.invariants struct array of NAMES ONLY (see below)
%
% ---------------------------------------------------------------------------
%   THE TABLE HAS EXACTLY ONE HOME, AND IT IS NOT THIS FILE.
%
%   contract_table.json is GENERATED from
%       scripts/wsl/sar_swarm_patches/contract_table.py
%   by  matlab/mgs/bridge/export_contract_table.py.
%
%   Nothing here hand-copies a bound or a foam value.  Two copies of a table
%   drift, and a drifted table still runs, still prints numbers, and is still
%   wrong -- the exact failure shape the contract table was built to end.
%
%   STALENESS is checked, not assumed: the JSON carries the sha1 of the .py it
%   came from, and this function re-hashes the .py and ABORTS on a mismatch.
%
%   INVARIANTS ARE NOT CHECKED HERE.  The 17 relations in contract_table.py are
%   Python callables; only their names cross into the JSON.  Half a check is
%   worse than none, so mgs.contractCheck reports "N invariants not evaluated
%   in MATLAB" and points at scripts/wsl/contract_verify.py rather than
%   pretending to have cleared them.
%
%   See also MGS.CONTRACTCHECK, MGS.FROMCONTRACT.

persistent CACHE
if nargin < 1, force = false; end
if ~force && ~isempty(CACHE), T = CACHE; return; end

here = fileparts(fileparts(mfilename('fullpath')));       % .../matlab/mgs
jsonPath = fullfile(here, 'contract_table.json');
repoRoot = fileparts(fileparts(here));                    % .../aeroplan-studio
pyPath = fullfile(repoRoot, 'scripts', 'wsl', 'sar_swarm_patches', 'contract_table.py');
genCmd = 'python matlab/mgs/bridge/export_contract_table.py';

if ~isfile(jsonPath)
    error('mgs:contractTable:missing', ...
        ['%s does not exist.  Generate it:\n    %s\n' ...
         'It is deliberately a generated artefact -- the table lives in ' ...
         'contract_table.py and nowhere else.'], jsonPath, genCmd);
end

raw = jsondecode(fileread(jsonPath));

% ---- staleness gate --------------------------------------------------------
if isfile(pyPath)
    have = lower(mgs.util.sha1file(pyPath));
    want = lower(raw.meta.source_sha1);
    if ~strcmp(have, want)
        error('mgs:contractTable:stale', ...
            ['contract_table.json was generated from a DIFFERENT ' ...
             'contract_table.py.\n  json says sha1 %s\n  file  is  sha1 %s\n' ...
             'Re-generate it:\n    %s\n' ...
             'Refusing to run against a stale table: every bound and every ' ...
             'foam value below would be a guess.'], ...
            want(1:12), have(1:12), genCmd);
    end
else
    warning('mgs:contractTable:nosource', ...
        ['%s not found, so the table could not be checked for staleness. ' ...
         'This is expected only if you copied matlab/ out of the repo.'], pyPath);
end

% ---- normalise -------------------------------------------------------------
T = struct();
T.meta       = raw.meta;
T.table      = normalise_rows(raw.table);
T.byName     = containers.Map({T.table.name}, num2cell(T.table));
T.envFor     = map_from_struct(raw.env_for);
T.jetRef     = map_from_struct(raw.jet_reference);
T.boolParams = cellstr_of(raw.bool_params);
T.intParams  = cellstr_of(raw.int_params);
T.unproven   = cellstr_of(raw.unproven);
T.invariants = raw.invariants;
T.jsonPath   = jsonPath;

CACHE = T;
end

% -----------------------------------------------------------------------------
function rows = normalise_rows(r)
%NORMALISE_ROWS  jsondecode gives a struct array when every row has the same
%   fields and a cell array of structs otherwise.  Force the struct array, and
%   force lo/hi/foam to double-or-empty ([] means "this side is not bounded",
%   which is how contract_table.py spells None).
if iscell(r), r = [r{:}]; end
for k = 1:numel(r)
    r(k).lo   = num_or_empty(r(k).lo);
    r(k).hi   = num_or_empty(r(k).hi);
    r(k).foam = num_or_empty(r(k).foam);
end
rows = r;
end

function v = num_or_empty(v)
if isempty(v), v = []; return; end
if ischar(v) || isstring(v)
    d = str2double(v);
    if isnan(d)
        v = [];                        % a non-numeric bound is not a bound
    else
        v = d;
    end
    return;
end
v = double(v);
end

function m = map_from_struct(s)
if isempty(s) || ~isstruct(s)
    m = containers.Map('KeyType', 'char', 'ValueType', 'any'); return;
end
f = fieldnames(s);
m = containers.Map('KeyType', 'char', 'ValueType', 'any');
for k = 1:numel(f)
    % jsondecode mangles field names that are not valid MATLAB identifiers by
    % prefixing 'x'.  None of our parameter names need it, but check anyway --
    % a silently renamed key is a lookup that quietly misses.
    m(f{k}) = s.(f{k});
end
end

function c = cellstr_of(v)
if isempty(v), c = {}; return; end
if ischar(v), c = {v}; return; end
if iscell(v), c = cellfun(@char, v, 'UniformOutput', false); return; end
c = cellstr(v);
end
