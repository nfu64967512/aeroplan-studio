function A = flightMovie(trial, opts)
%MGS.FLIGHTMOVIE  一整趟航次的影片:**起飛 → 編隊巡航 → 決斷圈 → 俯衝 → 命中**。
%
%   A = mgs.flightMovie("FORMC1")
%   A = mgs.flightMovie("FORMC1", file="C:\Users\me\Downloads\sortie.mp4")
%   A = mgs.flightMovie("FORMC1", export="none")     % 只看不存
%
% ---------------------------------------------------------------------------
%   為什麼需要這一支(而不是直接用 mgs.viz6dof)
%
%   `mgs.viz6dof` 讀 `tot_*.csv`,而那份 CSV **從「收到目標」才開始** ——
%   起飛與編隊巡航整段不在裡面。要「從起飛到命中」就必須把兩份接起來:
%
%       起飛 + 編隊巡航   runs/artifacts/<T>/formation/formation_*.csv
%       決斷圈 + 俯衝     runs/artifacts/<T>/tot_<T>.csv
%
%   編隊段的記錄器是 2026-08-24 才建的(見 scripts/wsl/sar_swarm_patches/
%   formation_log.py)。在那之前這一段**不存在於任何資料裡**,所以更早的
%   航次只能從收到目標畫起 —— 本函式會明說,不會假裝有。
%
% ---------------------------------------------------------------------------
%   ⚠️⚠️⚠️ 兩段的高度基準**不同**,接縫要對齊而且要講出來
%
%     編隊段  backup 走 `mav.position()[2]` = odom 的區域 ENU z
%             master 走 `MasterPublisher.lat/lon/alt`(GeoPose)
%     打擊段  tot_node 的 `agl` = alt − z_g(紅心地表)
%
%   實測(FORMB1/FORMC1):兩個 master 來源在地面靜止時差 **+50.77 m**。
%   直接串接會在接縫上出現一個**看不見的高度跳階** —— 影片會顯示飛機
%   在收到目標的瞬間往上或往下跳幾十公尺,而那從來沒有發生過。
%
%   ⇒ 本函式在接縫處**量出偏移並補平**,而且把補了多少寫進 A.splice
%     與圖上的標題。偏移大於 `max_splice_m` 就**報錯而不是硬接** ——
%     一個悄悄對齊的接縫比一個看得見的跳階更糟。
%
% ---------------------------------------------------------------------------
%   OUTPUTS
%     A.splice.offset_m   接縫補平量(公尺;正 = 編隊段被抬高)
%     A.splice.t_target   收到目標的時刻(接縫)
%     A.splice.n_form     編隊段的取樣數(0 = 那一段沒有資料)
%     A.file              寫出的檔案路徑
%
%   See also MGS.VIZ6DOF, MGS.PARSETOTCSV.

arguments
    trial (1,1) string
    opts.file    (1,1) string = ""
    opts.export  (1,1) string {mustBeMember(opts.export, ["none","mp4","gif"])} = "mp4"
    opts.fps     (1,1) double = 25
    opts.speed   (1,1) double = 30      % ⚠️ 整趟有 600 s 編隊 + 300 s 打擊,
                                        %    4x 會變成 4 分鐘的片子。
    opts.model   (1,1) string = "glyph"
    opts.show    (1,1) logical = true
    % 接縫偏移的容忍上限。超過就報錯 —— 那表示兩段根本不是同一趟,
    % 或某一段的基準理解錯了,硬接只會產出一個看起來合理的假影片。
    opts.max_splice_m (1,1) double = 400
    % ⚠️⚠️ 結尾在**最後一架觸地**，不是 CSV 的最後一拍。
    %    tot_node 在命中之後還會繼續記錄（expended / 盤旋 / 殘骸）——
    %    FORMC1 實測：命中在 t≈260 s，而 CSV 到 **759.7 s**。
    %    不裁的話影片有三分之一在放已經落地的飛機。
    opts.end_at  (1,1) string {mustBeMember(opts.end_at, ["impact","all"])} = "impact"
    opts.tail_s  (1,1) double = 6      % 命中後多留幾秒(看得到觸地那一刻)
end

root = mgs.repoRoot();
art  = fullfile(root, 'runs', 'artifacts', char(trial));
tot  = fullfile(art, sprintf('tot_%s.csv', trial));
assert(isfile(tot), 'mgs:flightMovie:noTot', ...
       '找不到 %s —— 沒有打擊段就沒有「到命中」可畫', tot);

C = mgs.parseTotCsv(tot);

% ── 編隊段 ────────────────────────────────────────────────────────────
formdir = fullfile(art, 'formation');
F = read_formation(formdir);
A_splice = struct('offset_m', 0, 't_target', NaN, 'n_form', 0, 'note', '');

if isempty(fieldnames(F))
    % ⚠️ 這不是「編隊沒問題」,是**那一段沒有留下資料**。
    %    2026-08-24 之前的航次全部如此。說清楚,不要靜靜地只畫後半段。
    A_splice.note = sprintf(['⚠️ %s 沒有 formation/ 資料 —— 影片**從收到目標開始**,' ...
                             '不含起飛與編隊巡航。' ...
                             '(編隊記錄器是 2026-08-24 才建的)'], trial);
    warning('mgs:flightMovie:noFormation', '%s', A_splice.note);
else
    [C, A_splice] = splice_formation(C, F, opts.max_splice_m);
end

% ── 裁到命中 ────────────────────────────────────────────────────────
t_end = NaN;
if opts.end_at == "impact"
    t_end = last_impact_t(C);
    if isfinite(t_end)
        t_end = t_end + opts.tail_s;
        for k = 1:numel(C.veh)
            T = C.veh(k).tbl;
            C.veh(k).tbl = T(todouble(T.t) <= t_end, :);
        end
    else
        % ⚠️ 找不到觸地 ⇒ **說出來**，不要默默畫整段。
        %    那通常表示這一趢沒有命中（或 agl 欄壞了）。
        warning('mgs:flightMovie:noImpact', ...
            ['⚠⚠ %s 找不到任何「DIVE 且 agl<=0」的拍 ' ...
             '⇒ 畫整段。這一趢可能沒有命中。'], trial);
    end
end

% ── 目標 ──────────────────────────────────────────────────────────────
% ⚠️ 走 viz6dof 自己的 *_geom.csv 尋找邏輯,不要另外猜一個 datum。
if opts.file == ""
    dl = fullfile(getenv('USERPROFILE'), 'Downloads');
    if ~isfolder(dl), dl = tempdir; end
    opts.file = string(fullfile(dl, sprintf('%s_takeoff_to_impact.mp4', trial)));
end

ttl = sprintf('%s — 起飛→編隊→決斷圈→俯衝→命中', trial);
if A_splice.n_form > 0
    ttl = sprintf('%s   [接縫補平 %+.1f m]', ttl, A_splice.offset_m);
end

A = mgs.viz6dof(C, export=opts.export, file=opts.file, fps=opts.fps, ...
                speed=opts.speed, model=opts.model, show=opts.show, ...
                focus="all", title=string(ttl));
A.splice = A_splice;
A.file   = opts.file;

fprintf('\n');
if A_splice.n_form > 0
    fprintf('編隊段 %d 取樣;接縫 @t=%.1f;高度基準補平 %+.1f m\n', ...
            A_splice.n_form, A_splice.t_target, A_splice.offset_m);
else
    fprintf('%s\n', A_splice.note);
end
if opts.export ~= "none"
    fprintf('影片:%s\n', opts.file);
end
end


% ══════════════════════════════════════════════════════════════════════
function F = read_formation(d)
%READ_FORMATION  把 formation_*.csv 讀成 id -> table。找不到就回空 struct。
F = struct();
if ~isfolder(d), return; end
L = dir(fullfile(d, 'formation_*.csv'));
for k = 1:numel(L)
    p = fullfile(L(k).folder, L(k).name);
    try
        T = readtable(p, 'TextType', 'string', 'VariableNamingRule', 'preserve');
    catch
        continue
    end
    if isempty(T) || ~ismember('uav_id', T.Properties.VariableNames), continue; end
    % ⚠️⚠️ 這個檔會一直寫到航次結束後好幾小時,而且**打擊期間編隊層讓權**
    %    ⇒ 中間有一個大空隙。空隙之後是殘骸資料,接進影片會讓飛機在
    %    命中之後又飛起來。只取**第一段**。
    t = todouble(T.t);
    keep = true(height(T), 1);
    if numel(t) > 2
        gaps = find(diff(t) > 30, 1, 'first');
        if ~isempty(gaps), keep(gaps+1:end) = false; end
    end
    T = T(keep, :);
    if isempty(T), continue; end
    ids = unique(todouble(T.uav_id));
    for i = 1:numel(ids)
        f = sprintf('u%d', ids(i));
        sub = T(todouble(T.uav_id) == ids(i), :);
        if isfield(F, f), F.(f) = [F.(f); sub]; else, F.(f) = sub; end
    end
end
end


% ══════════════════════════════════════════════════════════════════════
function [C, S] = splice_formation(C, F, max_splice)
%SPLICE_FORMATION  把編隊段接到打擊段前面,並對齊高度基準。
S = struct('offset_m', 0, 't_target', NaN, 'n_form', 0, 'note', '');
S.per_veh = struct('id', {}, 'offset_m', {});

% 打擊段的 t 是**相對收到目標**的秒數(tot_node 自己歸零);
% 編隊段的 t 是**牆鐘**。要用牆鐘把兩者對起來。
t_tot0 = inf;
for k = 1:numel(C.veh)
    t_tot0 = min(t_tot0, min(todouble(C.veh(k).tbl.t)));
end
assert(abs(t_tot0) < 5, 'mgs:flightMovie:totNotZeroed', ...
    ['打擊段的 t 不是從 0 起算(最小 %.1f)—— 接縫的時間對齊會錯。' ...
     '這通常表示 CSV 格式變了。'], t_tot0);

% 編隊段的最後一個牆鐘時刻 = 收到目標的那一刻(讓權點)
t_wall_end = -inf;
fn = fieldnames(F);
for i = 1:numel(fn)
    t_wall_end = max(t_wall_end, max(todouble(F.(fn{i}).t)));
end
S.t_target = 0;

offs = [];
for k = 1:numel(C.veh)
    f = sprintf('u%d', C.veh(k).id);
    if ~isfield(F, f), continue; end
    Tf = F.(f);
    Tt = C.veh(k).tbl;

    tf = todouble(Tf.t) - t_wall_end;          % 收到目標 = 0,之前為負
    af = todouble(Tf.alt);
    ok = isfinite(tf) & isfinite(af);
    tf = tf(ok); af = af(ok); Tf = Tf(ok, :);
    if numel(tf) < 20, continue; end

    % ⭐ 接縫偏移:用**接縫兩側各 5 s** 的中位數相減。
    %    用單點會被一個跳點決定整段的偏移。
    at = todouble(Tt.agl); tt = todouble(Tt.t);
    lhs = median(af(tf > -5), 'omitnan');
    rhs = median(at(tt < 5),  'omitnan');
    if ~isfinite(lhs) || ~isfinite(rhs), continue; end
    off = rhs - lhs;
    % ⚠️⚠️ **逐機**的偏移，不可以用跨機中位數。
    %    實測（FORMB1/FORMC1）：
    %        航次      UAV2（備長）   UAV1（長機）
    %        FORMB1     +269.1        +37.9
    %        FORMC1      -95.9        -20.0
    %    連**正負號都會變** ⇒ 這不是一個固定的基準差，
    %    而是接縫兩側真實的高度變化加上不同的 datum。
    %    拿跨機中位數去補平會把其中一架擺錯 200 多公尺。
    assert(abs(off) <= max_splice, 'mgs:flightMovie:spliceTooBig', ...
        ['UAV%d 的接縫高度差 %.1f m 超過上限 %.1f —— **不硬接**。\n' ...
         '    這表示兩段不是同一趟,或某一段的高度基準理解錯了。\n' ...
         '    悄悄對齊一個大偏移會產出一個看起來合理的假影片。'], ...
        C.veh(k).id, off, max_splice);
    offs(end+1) = off; %#ok<AGROW>
    S.per_veh(end+1) = struct('id', C.veh(k).id, 'offset_m', off);

    % 造一張與 tot 同欄位的表
    n = height(Tf);
    G = table();
    G.t   = tf;
    G.lat = todouble(Tf.lat);
    G.lon = todouble(Tf.lon);
    G.agl = af + off;                       % ⭐ 補平到打擊段的基準
    G.gs  = todouble(Tf.gs);
    % γ [KIN]:由高度率與地速反推。編隊段沒有記 gam_deg。
    % ⚠️ 用**平滑後**的高度率:逐拍差分被遙測抖動主導(實測中位 −13 m/s
    %    而整段平均 +3),那個量拿來畫姿態會讓飛機在影片裡瘋狂抖動。
    dz = smooth_rate(tf, G.agl, 5.0);
    G.gam_deg = rad2deg(atan2(dz, max(G.gs, 1)));
    G.sysid = repmat(C.veh(k).id, n, 1);
    G.mode  = repmat("form", n, 1);
    G.phase = repmat("FORMATION", n, 1);

    % 補齊 tot 表有而編隊段沒有的欄(留 NaN,**不要填 0**)
    for c = string(Tt.Properties.VariableNames)
        if ismember(c, string(G.Properties.VariableNames)), continue; end
        v = Tt.(char(c));
        if isnumeric(v), G.(char(c)) = nan(n, 1);
        else,            G.(char(c)) = repmat("", n, 1);
        end
    end
    G = G(:, Tt.Properties.VariableNames);   % 欄序對齊
    C.veh(k).tbl = [G; Tt];
    S.n_form = S.n_form + n;
end
if ~isempty(offs)
    S.offset_m = median(offs);
    % ⚠️⚠️ 逐機偏移差得多時，兩段的**絕對高度不可比** ——
    %    每一架自己連續了，但「誰比誰高」在接縫兩側不是同一件事。
    %    這一行必須大聲 —— 一個默默對齊的影片看起來完全合理。
    spread = max(offs) - min(offs);
    if spread > 50
        S.note = sprintf(['⚠⚠ 逐機接縫偏移相差 %.0f m ' ...
            '(%s) ⇒ 兩段的**絕對高度不可比**；' ...
            '每一架自己連續，但跨接縫的機間高度差不是真的。'], ...
            spread, strjoin(arrayfun(@(x) sprintf('UAV%d %+.0f', x.id, x.offset_m), ...
                                     S.per_veh, 'UniformOutput', false), ', '));
        warning('mgs:flightMovie:datumSpread', '%s', S.note);
    end
end
end


% ══════════════════════════════════════════════════════════════════════
function r = smooth_rate(t, y, win_s)
%SMOOTH_RATE  以 win_s 秒的窗算變化率。⚠️ 不用逐拍差分 —— 見呼叫端說明。
r = zeros(size(y));
for k = 1:numel(t)
    j = t >= t(k) - win_s/2 & t <= t(k) + win_s/2;
    if nnz(j) < 3, continue; end
    tt = t(j); yy = y(j);
    d = tt(end) - tt(1);
    if d > 0.5, r(k) = (yy(end) - yy(1)) / d; end
end
end


function t = last_impact_t(C)
%LAST_IMPACT_T  最後一架觸地的時刻。找不到回 NaN。
%   判準 = phase=="DIVE" 且 agl <= 0 的**第一**拍，逐機取完再取最大。
%   ⚠️ 不可以取「agl<=0 的最後一拍」—— SITL 不終結落海機體，
%      那會拿到幾百秒後沉在水裡的那一拍。
t = NaN;
for k = 1:numel(C.veh)
    T = C.veh(k).tbl;
    if ~ismember('agl', T.Properties.VariableNames), continue; end
    a = todouble(T.agl);
    ph = string(T.phase);
    j = find(ph == "DIVE" & a <= 0, 1, 'first');
    if isempty(j), continue; end
    ti = todouble(T.t(j));
    if isnan(t) || ti > t, t = ti; end
end
end


function v = todouble(x)
if isnumeric(x), v = double(x); return; end
v = str2double(string(x));
end
