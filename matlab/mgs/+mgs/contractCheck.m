function rep = contractCheck(values, opts)
%MGS.CONTRACTCHECK  Score a set of effective values against the contract table.
%
%   rep = mgs.contractCheck(values)
%   rep = mgs.contractCheck(values, 'require', {'pn_los_floor','gate_r'}, ...
%                                   'strict', true)
%
%   values  struct or containers.Map, parameter name -> value.  Strings are
%           accepted because that is how contract_*.json stores them.
%
%   rep.ok            true iff there are no hard violations
%   rep.violations    struct array .kind .name .got .want .why .unproven
%                       kind: 'missing' | 'foam' | 'range' | 'nonnumeric'
%   rep.checked       cellstr of names that were scored
%   rep.unchecked     cellstr of table entries absent from `values`
%   rep.invariantGap  how many relations were NOT evaluated (always 17 today)
%   rep.summary       one printable block
%
% ---------------------------------------------------------------------------
%   THE THREE KINDS, AND WHY 'foam' IS SEPARATE FROM 'range'
%
%   Every declare_parameter default in tot_node.py is a FOAM-ERA value
%   (vmin 14, vmax 30, dive_speed 42, perch_alt 280, r_fan 2000).  They are
%   legal, correctly typed, and raise nothing.  If any feed path breaks, the
%   value silently reverts to one of them.  A range check does not catch that,
%   because the range may happen to be wide enough to admit it.
%
%   So hitting a foam value is judged ON ITS OWN: it means "nobody ever set
%   this", regardless of whether the number is inside the allowed band.
%
%   'unproven' contracts (formation_alt, patrol_alt, itcg_gain, itcg_a_max_g)
%   are flagged but do NOT set rep.ok = false.  Their bounds have not been
%   pinned by measurement, so a violation there is a hint, not evidence.
%
%   See also MGS.CONTRACTTABLE, MGS.FROMCONTRACT.

arguments
    values
    opts.require (1,:) cell = {}
    opts.strict (1,1) logical = true
    opts.table = []
end

if isempty(opts.table), T = mgs.contractTable(); else, T = opts.table; end
V = as_map(values);

viol = struct('kind', {}, 'name', {}, 'got', {}, 'want', {}, 'why', {}, ...
              'unproven', {});
checked = {};
unchecked = {};

for k = 1:numel(T.table)
    con = T.table(k);
    name = con.name;
    isUnproven = any(strcmp(name, T.unproven));

    if ~isKey(V, name)
        if any(strcmp(name, opts.require))
            viol(end+1) = mkv('missing', name, [], ...
                'present in the readback', con.why, isUnproven); %#ok<AGROW>
        else
            unchecked{end+1} = name; %#ok<AGROW>
        end
        continue;
    end

    [x, okNum] = as_double(V(name));
    if ~okNum
        viol(end+1) = mkv('nonnumeric', name, V(name), 'a number', ...
            con.why, isUnproven); %#ok<AGROW>
        continue;
    end
    checked{end+1} = name; %#ok<AGROW>

    % ---- foam first.  It is independent of the band, on purpose. ----------
    if ~isempty(con.foam) && isequal_tol(x, con.foam)
        viol(end+1) = mkv('foam', name, x, ...
            sprintf('anything but the foam-era default %g', con.foam), ...
            con.why, isUnproven); %#ok<AGROW>
        continue;                    % one verdict per parameter, foam wins
    end

    % ---- band -------------------------------------------------------------
    lo = con.lo; hi = con.hi;
    below = ~isempty(lo) && x < lo;
    above = ~isempty(hi) && x > hi;
    if below || above
        viol(end+1) = mkv('range', name, x, band_text(lo, hi), ...
            con.why, isUnproven); %#ok<AGROW>
    end
end

hard = false(1, numel(viol));
for k = 1:numel(viol), hard(k) = ~viol(k).unproven; end

rep = struct();
rep.ok           = ~any(hard);
rep.violations   = viol;
rep.hardCount    = sum(hard);
rep.softCount    = sum(~hard);
rep.checked      = checked;
rep.unchecked    = unchecked;
rep.invariantGap = numel(T.invariants);
rep.tableSha1    = T.meta.source_sha1;
rep.summary      = render(rep, T);

if opts.strict && ~rep.ok
    error('mgs:contract:violation', '%s', rep.summary);
end
end

% -----------------------------------------------------------------------------
function v = mkv(kind, name, got, want, why, unproven)
v = struct('kind', kind, 'name', name, 'got', got, 'want', want, ...
           'why', why, 'unproven', unproven);
end

function s = band_text(lo, hi)
if isempty(lo) && isempty(hi), s = 'unbounded';
elseif isempty(lo),            s = sprintf('<= %g', hi);
elseif isempty(hi),            s = sprintf('>= %g', lo);
else,                          s = sprintf('[%g, %g]', lo, hi);
end
end

function tf = isequal_tol(a, b)
% Foam values are exact literals, but they arrive through JSON as text and back
% through str2double.  A relative tolerance keeps 42 == 42.0 == "42.000000".
tf = abs(a - b) <= max(1e-9, 1e-9 * abs(b));
end

function m = as_map(values)
if isa(values, 'containers.Map'), m = values; return; end
if ~isstruct(values)
    error('mgs:contractCheck:type', 'values must be a struct or containers.Map');
end
f = fieldnames(values);
m = containers.Map('KeyType', 'char', 'ValueType', 'any');
for k = 1:numel(f), m(f{k}) = values.(f{k}); end
end

function [x, ok] = as_double(v)
ok = true;
if isnumeric(v) && isscalar(v), x = double(v); return; end
if islogical(v) && isscalar(v), x = double(v); return; end
if ischar(v) || isstring(v)
    x = str2double(v);
    ok = ~isnan(x);
    if ~ok, x = NaN; end
    return;
end
x = NaN; ok = false;
end

function s = render(rep, T)
L = {};
L{end+1} = sprintf(['contract check: %d hard, %d soft (unproven) violations; ' ...
                    '%d parameters scored, %d not present'], ...
                   rep.hardCount, rep.softCount, numel(rep.checked), ...
                   numel(rep.unchecked));
L{end+1} = sprintf('  table %s  sha1 %s', T.meta.source, rep.tableSha1(1:12));
for k = 1:numel(rep.violations)
    v = rep.violations(k);
    tag = 'HARD'; if v.unproven, tag = 'soft'; end
    L{end+1} = sprintf('  [%s] %-8s %-22s got %s, want %s', ...
        tag, v.kind, v.name, fmt(v.got), v.want); %#ok<AGROW>
    if ~isempty(v.why)
        L{end+1} = sprintf('           why: %s', one_line(v.why)); %#ok<AGROW>
    end
end
if rep.invariantGap > 0
    L{end+1} = sprintf(['  NOTE: %d cross-parameter invariants were NOT ' ...
        'evaluated here.'], rep.invariantGap);
    L{end+1} = ['        They are Python callables in contract_table.py.  ' ...
                'Run scripts/wsl/contract_verify.py'];
    L{end+1} = '        for the full gate; this check covers single values only.';
end
s = strjoin(L, newline);
end

function s = fmt(v)
if isempty(v),        s = '<absent>';
elseif isnumeric(v),  s = sprintf('%g', v);
else,                 s = ['"' char(string(v)) '"'];
end
end

function s = one_line(s)
s = regexprep(char(s), '\s+', ' ');
if numel(s) > 96, s = [s(1:93) '...']; end
end
