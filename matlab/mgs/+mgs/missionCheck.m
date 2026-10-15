function K = missionCheck(src, opts)
%MGS.MISSIONCHECK  Phase-by-phase check of a COMPLETE flown sortie.
%
%   K = mgs.missionCheck('runs/artifacts/LAT1/tot_LAT1.csv')
%   K = mgs.missionCheck(C)                 C = mgs.parseTotCsv(...)
%
%   Walks the whole flight -- climb, cruise, weave, run-in, wait, run-up,
%   dive, expended -- and for each leg reports what happened and, separately,
%   whether this environment's MODEL covers that leg at all.
%
% ---------------------------------------------------------------------------
%   ⚠️⚠️ READ THIS BEFORE CALLING IT AN END-TO-END VALIDATION.
%
%   THE MODEL COVERS ONE LEG.  mgs.guidance's own header says it verbatim:
%   "*** PHASES OTHER THAN 'dive' ARE UNCALIBRATED. ***  The truth set only
%   ever observed the vehicle from release onwards."  There is no rocket
%   boost, no takeoff, no TECS climb, no formation layer and no L1 waypoint
%   navigation in mgs.  Before PN engages, the real aircraft is flown by
%   ArduPlane's L1/TECS -- a different controller this environment does not
%   reproduce, and deliberately so (memory architecture-boundary-with-ardupilot:
%   we take its physics and its MAVLink bridge, never its terminal guidance).
%
%   So this function does NOT simulate the mission.  It CHECKS a flown one,
%   from the node's own per-tick log, and prints a coverage column that says
%   which legs a model result could ever speak to.  Building the missing legs
%   into mgs would produce a single continuous trajectory that LOOKS like an
%   end-to-end validation while four of its five legs had no truth data
%   behind them -- which is the exact failure this whole environment exists
%   to make impossible.
%
% ---------------------------------------------------------------------------
%   K.veh(k).id
%           .seg(j)  .mode .t0 .t1 .dur .agl_lo .agl_hi .gs_lo .gs_hi
%                    .gam_lo .gam_hi .n .flags
%           .flags   every anomaly found, with the number that triggered it
%   K.summary  the printed report
%
%   See also MGS.VIZ6DOF, MGS.PARSETOTCSV, MGS.KPI.

arguments
    src
    opts.render (1,1) logical = true
    opts.cfg = []            % contract snapshot values, for the thresholds
end

if isstruct(src) && isfield(src, 'veh')
    C = src;
else
    C = mgs.parseTotCsv(char(src));
end

% ── thresholds.  ⚠️ Read from the run's own contract snapshot when we can:
%    hardcoding vmin/AIRSPEED_MIN here would silently grade one sortie by
%    another sortie's configuration.
T = thresholds(opts.cfg, C);

K = struct('veh', struct('id', {}, 'seg', {}, 'flags', {}), 'thr', T);

for k = 1:numel(C.veh)
    Tk = C.veh(k).tbl;
    t   = numc(Tk.t);
    md  = Tk.mode;  if ~iscell(md), md = cellstr(md); end
    agl = numc(Tk.agl);  gs = numc(Tk.gs);  gam = numc(Tk.gam_deg);
    [t, o] = sort(t);
    [t, iu] = unique(t, 'last');  o = o(iu);
    md = md(o); agl = agl(o); gs = gs(o); gam = gam(o);

    seg = struct('mode', {}, 't0', {}, 't1', {}, 'dur', {}, 'agl_lo', {}, ...
                 'agl_hi', {}, 'gs_lo', {}, 'gs_hi', {}, 'gam_lo', {}, ...
                 'gam_hi', {}, 'n', {}, 'flags', {});
    b = seg_bounds(md);
    for j = 1:size(b, 1)
        r = b(j,1):b(j,2);
        s = struct();
        s.mode = md{b(j,1)};
        s.t0 = t(r(1));  s.t1 = t(r(end));  s.dur = s.t1 - s.t0;  s.n = numel(r);
        [s.agl_lo, s.agl_hi] = rng(agl(r));
        [s.gs_lo,  s.gs_hi ] = rng(gs(r));
        [s.gam_lo, s.gam_hi] = rng(gam(r));
        s.flags = leg_flags(s, T);
        seg(end+1) = s; %#ok<AGROW>
    end
    K.veh(k).id = C.veh(k).id;
    K.veh(k).seg = seg;
    K.veh(k).flags = [seg.flags];
end

K.summary = render(K, C);
if opts.render, fprintf('%s\n', K.summary); end
end

% =============================================================================
function T = thresholds(cfg, C)
%THRESHOLDS  ⚠️ Prefer the run's OWN contract snapshot; never silently mix runs.
T = struct('vmin', NaN, 'vmax', NaN, 'as_min', NaN, 'perch_alt', NaN, ...
           'src', 'defaults (no snapshot found)');
v = cfg;
if isempty(v) && isfield(C, 'path') && ~isempty(C.path)
    [d, b, ~] = fileparts(char(C.path));
    trial = regexprep(b, '^tot_', '');
    f = fullfile(d, ['contract_' trial '.json']);
    if isfile(f)
        j = jsondecode(fileread(f));
        if isfield(j, 'values'), v = j.values; T.src = f; end
    end
end
if ~isempty(v)
    T.vmin      = getnum(v, 'vmin');
    T.vmax      = getnum(v, 'vmax');
    T.as_min    = getnum(v, 'AIRSPEED_MIN');
    T.perch_alt = getnum(v, 'perch_alt');
end
end

function x = getnum(v, f)
x = NaN;
if isstruct(v) && isfield(v, f)
    x = str2double(string(v.(f)));
end
end

% -----------------------------------------------------------------------------
function b = seg_bounds(md)
%SEG_BOUNDS  Contiguous runs of the same mode string.
b = zeros(0, 2);  i = 1;
while i <= numel(md)
    j = i;
    while j < numel(md) && strcmp(md{j+1}, md{i}), j = j + 1; end
    b(end+1, :) = [i j]; %#ok<AGROW>
    i = j + 1;
end
end

function [lo, hi] = rng(x)
x = x(isfinite(x));
if isempty(x), lo = NaN; hi = NaN; else, lo = min(x); hi = max(x); end
end

% -----------------------------------------------------------------------------
function f = leg_flags(s, T)
%LEG_FLAGS  ⚠️ Every flag carries the NUMBER that triggered it.  A bare
%   "warning" that does not say what it saw cannot be checked or refuted.
f = string.empty(1, 0);

% ⚠️⚠️ Below AIRSPEED_MIN the flight controller rejects the WHOLE command
%    (MAV_RESULT=4) -- not the speed part, the whole thing.  Memory
%    catch-speed-rejected-by-fc: a catch_speed of 157 against AIRSPEED_MAX 150
%    made an entire channel silently inert for dozens of sorties.
% ⚠️ `expended` 一定要排除:觸地之後飛機已經在水裡,地速 0 是**應該的**,
%    不是指令被拒收。第一版沒排除,RA1 就誤報了一次 —— 一個會誤報的旗標
%    比沒有旗標更貴,因為下一個人會照它做決策(這一輪已經在「打短」判準上
%    踩過同樣的坑)。
if ~strcmp(s.mode, 'expended') && ...
        isfinite(T.as_min) && isfinite(s.gs_lo) && s.gs_lo < T.as_min
    f(end+1) = sprintf("地速 %.0f < AIRSPEED_MIN %.0f:該段指令有被整發拒收的風險", ...
                       s.gs_lo, T.as_min);
end
% ⚠️ Below the guidance layer's own vmin the speed command itself is out of
%    contract, and dive-pace-saturates-vmin records what pinning at vmin costs:
%    it destroys spread AND terminal speed at the same time.
if isfinite(T.vmin) && isfinite(s.gs_lo) && s.gs_lo < T.vmin && ...
        any(strcmp(s.mode, {'runin', 'wait', 'runup', 'dive'}))
    f(end+1) = sprintf("地速 %.0f < vmin %.0f(%s 段)", s.gs_lo, T.vmin, s.mode);
end
% Climb that loses height, or cruise that is nowhere near perch.
if strcmp(s.mode, 'climb') && isfinite(s.agl_hi) && isfinite(T.perch_alt) && ...
        s.agl_hi < 0.5 * T.perch_alt
    f(end+1) = sprintf("climb 只到 %.0f m,不到 perch_alt %.0f 的一半", ...
                       s.agl_hi, T.perch_alt);
end
if strcmp(s.mode, 'dive') && isfinite(s.gam_hi) && s.gam_hi > 0
    f(end+1) = sprintf("俯衝段出現爬升(γ 最高 %+.1f°)—— pre-PN 爬升是本專案" + ...
                       "已知的主導曝險", s.gam_hi);
end
end

% =============================================================================
function s = render(K, C)
%RENDER  ⚠️ The coverage column is the point of this table.
COV = { ...
 'climb',    'NO',   'ArduPlane TECS 爬升。mgs 沒有這一段'; ...
 'straight', 'NO',   'L1 航點巡航。mgs 沒有這一段'; ...
 'weave',    'NO',   '蛇形吸時。mgs 沒有這一段'; ...
 'runin',    'NO',   '轉場飛向環。mgs 的 hold 相位是**未校準**的近似'; ...
 'wait',     'NO',   '環上等待。同上,且守環從未被實際飛出來過'; ...
 'runup',    'NO',   '內切助跑。mgs 沒有這一段'; ...
 'hold',     'PART', 'mgs.guidance 有 hold 相位,但檔頭自述**未校準**'; ...
 'dive',     'YES',  'mgs.simSwarm ≡ stot_sim 逐位元;律對 32 架次驗證過'; ...
 'expended', 'N/A',  '已消耗,不再下令'};

L = {};
L{end+1} = '══ 完整流程逐階段檢查(真實航次,不是模擬)';
L{end+1} = sprintf('   來源 %s', tern(isfield(C,'path'), char(getdef(C,'path','?')), '?'));
L{end+1} = sprintf('   門檻來源 %s   vmin %.0f  vmax %.0f  AIRSPEED_MIN %.0f  perch_alt %.0f', ...
    K.thr.src, K.thr.vmin, K.thr.vmax, K.thr.as_min, K.thr.perch_alt);
L{end+1} = '';
for k = 1:numel(K.veh)
    L{end+1} = sprintf('── uav%d', K.veh(k).id); %#ok<AGROW>
    L{end+1} = sprintf('  %-10s %8s %8s %7s %14s %13s %15s  %-5s', ...
        'mode', 't0', 't1', 'dur', 'AGL', '地速', 'γ deg', '模型'); %#ok<AGROW>
    for j = 1:numel(K.veh(k).seg)
        s = K.veh(k).seg(j);
        c = cov_of(COV, s.mode);
        L{end+1} = sprintf('  %-10s %8.1f %8.1f %7.1f %6.0f..%-6.0f %5.0f..%-6.0f %+6.1f..%+6.1f  %-5s', ...
            s.mode, s.t0, s.t1, s.dur, s.agl_lo, s.agl_hi, ...
            s.gs_lo, s.gs_hi, s.gam_lo, s.gam_hi, c); %#ok<AGROW>
        for q = 1:numel(s.flags)
            L{end+1} = sprintf('               ⚠️ %s', s.flags(q)); %#ok<AGROW>
        end
    end
    L{end+1} = ''; %#ok<AGROW>
end
L{end+1} = '模型涵蓋:';
for r = 1:size(COV, 1)
    L{end+1} = sprintf('  %-10s [%-4s] %s', COV{r,1}, COV{r,2}, COV{r,3}); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = '⚠️⚠️ 九段裡模型只涵蓋 dive 一段。這張表是**對飛過的資料做檢查**,';
L{end+1} = '   不是端到端模擬 —— 把缺的四段補進 mgs 會畫出一條看起來像端到端';
L{end+1} = '   驗證、實際上五段中有四段沒有真值背書的軌跡。';
s = strjoin(L, newline);
end

function c = cov_of(COV, m)
i = find(strcmp(COV(:,1), m), 1);
if isempty(i), c = '?'; else, c = COV{i,2}; end
end

function v = getdef(s, f, d)
if isstruct(s) && isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end

function v = tern(c, a, b)
if c, v = a; else, v = b; end
end

function v = numc(cc)
if isnumeric(cc), v = double(cc); return; end
v = nan(numel(cc), 1);
for k = 1:numel(cc)
    d = str2double(strtrim(char(cc{k})));
    if ~isnan(d), v(k) = d; end
end
end
