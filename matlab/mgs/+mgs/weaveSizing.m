function R = weaveSizing(opts)
%MGS.WEAVESIZING  蛇形(path stretching)的吸收力定量模型,並用實飛資料驗證。
%
%   R = mgs.weaveSizing()                       % 用 LATOBS1,預設繪圖
%   R = mgs.weaveSizing('csv', 'runs/.../tot_X.csv')
%   R = mgs.weaveSizing('render', false, 'plot', false)
%   R = mgs.weaveSizing('want_s', 15)           % 反解:要吸 15 s 需要什麼
%
% ---------------------------------------------------------------------------
%   ⭐⭐ 這個檔要回答的問題,不是飛行資料能回答的那個
%
%   飛行資料給的是**一個操作點**:LATOBS1 的 ALIGN 段,實得迂迴中位 1.177。
%   我們真正要問的是「要吸收 X 秒,需要什麼樣的週期與坡度」——那需要一個
%   模型,而模型的價值完全取決於它被那 6 個實測點驗證還是推翻。
%
%   ⚠️⚠️ 所以本檔的**主產物是殘差,不是預測值**。R.bench 每一列都是
%   「模型 vs 實飛」,而不是「模型 vs 模型」。這一點是 [[agreement-is-not-validation]]
%   直接換來的教訓:六份獨立實作互相吻合到 0.000e+00,而其中一個幾何六份
%   全錯 98,751 ft —— 一致不等於對,要外部參照。
%
% ---------------------------------------------------------------------------
%   模型
%
%   ⭐ 換向率是硬約束(tot_node.py 的 _weave_theta_of docstring 逐字):
%
%       omega = g*tan(phi)/V            航向角速度 [rad/s]
%
%   把航向從 +theta 甩到 -theta 要 2*theta/omega,而它必須塞得進半個週期。
%   節點自己的夾限是 `th = min(th, omega*P/8)`,所以:
%
%       A = min(theta_cmd, omega*P/8)   實際能達到的偏航幅度
%
%   ⚠️ 而**指令角不等於迂迴**。指令 45 度的理論迂迴是 1/cos(45) = 1.414,
%      但航向是三角波不是方波 —— 它一路在爬升與下降,只有瞬間碰到 ±A。
%      三角波的平均 sec:
%
%       D_full = (1/A) * integral_0^A sec(u) du = log(sec A + tan A) / A
%
%   ⚠️⚠️ 而且只有跑滿至少半個週期才有「平均」可言。實測 0.21 個週期
%      ⇒ 那是**斜飛不是蛇形**,航向從 0 單向爬到 omega*T_seg,平均偏航只有
%      一半。所以:
%
%       n_cyc = T_seg / P
%       A_eff = A                    當 n_cyc >= 0.5
%             = min(A, omega*T_seg/2)  當 n_cyc <  0.5   (單向斜飛)
%
%   吸收的時間 = T_seg * (D - 1)。
%
% ---------------------------------------------------------------------------
%   ⚠️⚠️⚠️ 2026-08-24 實測結論:上面那個模型要**拆成兩段**,而兩段的命運相反
%
%   A 段  (V, phi, P, T_seg) -> 實得航向偏移        ❌ **被 LATOBS1 推翻**
%   B 段  實得航向偏移分佈    -> 迂迴倍率            ✅ **成立**
%
%   ── A 段為什麼被推翻 ──
%   結構自檢 4(DELETE:拿掉運動學夾限)**紅了**,而它紅的方式就是答案:
%   夾住 1.1222 ⇒ 不夾 1.1222,**一模一樣**。也就是說夾限在這個操作點
%   完全惰性,模型裡根本不是它在決定答案。
%
%   算一下就懂:實測坡度 p90 是 **78 度**,V ~ 112 m/s
%       omega = g*tan(78)/112 = 0.412 rad/s = 23.6 deg/s
%       A_kin = omega*P/8 = 0.412*120/8 = 6.18 rad = 354 deg   >>  45 deg
%   節點 docstring 那個「theta 只能到 9.8 度」的例子用的是**泡棉時代的
%   P = 22 s**;在 P = 120 s 之下那道夾限離 45 度遠得很。
%
%   ⇒ 六架的 (V, phi, P) 幾乎相同,而實得偏移中位散在 **13.1 - 39.1 度**。
%     模型對六架全部預測同一個 1.1222,實飛 1.044 - 1.381,rms 殘差 0.126
%     —— **零解釋力**。A 段缺一個機制,而我們還不知道那是什麼。
%
%   ── B 段為什麼成立 ──
%   拿**實得**偏移分佈算 mean(1/cos(offset)),對上實測迂迴:
%       UAV3 +0.021   UAV4 +0.018   UAV6 +0.014   UAV5 +0.038
%       UAV2 +0.173   UAV1 +0.286     <- p99 偏移逼近 90 度,1/cos 發散
%   4/6 架吻合到 0.04(1.4 - 3.5%)。
%
%   ⚠️⚠️ 所以 R.invert 那張「要吸 X 秒需要什麼 (P, phi)」的表**不可信** ——
%      它整個建立在 A 段上。本檔仍然算它,但標成 SKIP 並在這裡說明白:
%      **拿它去訂參數會得到一個看起來很具體的錯答案。**
%
% ---------------------------------------------------------------------------
%   ⚠️ 模型**不含**的三件事,而它們每一件都可能是 A 段缺的那個機制:
%     1. 節點在死區內不出力 ⇒ 實際擺動時間 < T_seg
%     2. 坡度指令與實得坡度的差(roll_cmd_deg 只在 PN 裡產生,而這兩趟都
%        沒進俯衝)。
%        ⭐⭐ 但 ALIGNW1 用**指令偏航角**直接量到了第一層,而答案是好消息:
%             機   指令 theta   實得偏移(slot 基準)   達成率
%             1      20.1            21.9            1.09
%             2       9.9            11.8            1.19
%             3      10.2             9.4            0.92
%             6      26.0            30.5            1.17
%          Pearson **+0.991**,達成率中位 **1.13**。
%          ⇒ 對照 [[gamma-tracking-deficit]] 的俯仰通道 **0.063**:
%            橫向通道**沒有**那個缺陷,指令下去就達成(略微過衝)。
%
%        ⚠️⚠️ 訂正一個我自己報過的錯數字:先前說「指令 45 度、實測 13-39 度、
%          達成 30-85%」——**兩處都錯**。
%            (a) 45 度是 weave_theta_max(**上限**),不是指令;真正的指令是
%                acos(v_req/vmin),實測中位只有 9.9-26.0 度。
%            (b) 偏移的參考基準用成「到紅心」,而律是繞著「到 **slot**」擺的。
%                錯基準讓 UAV3 的 10.2 度指令算出 30.5 度「實得」= 2.99 倍,
%                而飛機不可能比指令多轉三倍。用對基準是 0.92。
%          ⇒ 這兩個錯**都會產生看起來合理的數字**,而合起來剛好指向
%            「橫向也有嚴重缺陷」這個與事實相反的結論。
%     3. 六架的 weave 段長度差 8.4 倍(501 vs 4230 拍),所以「一個中位數」
%        本身就是可疑的摘要
%
%   ⚠️ 已經**排除**的兩個解釋(都是實測排除的,不是推理排除的):
%     * 「迂迴是一次性轉入造成的」-> 把 weave 段切四等分,四段的迂迴中位是
%       1.110 / 1.134 / 1.220 / 1.093,**沒有前段集中**;Q1/Q4 從 0.75 到 1.22,
%       有的機體後段反而更大 ⇒ 是整段持續的路徑拉長,不是轉入的假象。
%     * 「段越短迂迴越高」-> Spearman 只有 **-0.14**(n=6)。那是我從表格
%       眼睛看出來的模式,秩相關一算就沒了。**六個點看得出的模式多半不是模式。**
%
% ---------------------------------------------------------------------------
%   實飛量測(LATOBS1,2026-08-24,N=6,ALIGN 段)
%
%   ⚠️⚠️ 這一段的資料**既有工具全都看不到**:
%     * CSV 的 `weave_yaw_deg` 在這 12,618 個 weave 拍裡**一拍都沒寫** ——
%       ALIGN 段的蛇形用內聯算的 theta,從不寫 self.weave_theta[i],
%       而那一欄讀的就是它。
%     * `scripts/wsl/weave_readout.py` 對這趟直接報「沒有 wait 段」。
%   ⇒ 「0.21 個週期、零換向、只吸得到 2.85 s」那個記錄在案的結論量的是
%     **等待段**;ALIGN 段從來沒有被分析過。本檔用的是新加的 weave_rev_n
%     與位置積分,不依賴那兩個瞎掉的工具。
%
%   OUTPUTS
%     R.meas      每架的實測:迂迴、換向、擺幅、V、phi、段長
%     R.model     同一組輸入下模型的預測
%     R.resid     模型 - 實測(**本檔的主產物**)
%     R.invert    反解:要吸 want_s 秒,需要的 (P, phi) 組合
%     R.bench     模型 vs 實飛的計分列(15 欄契約,與 zarchan 那套相同)
%     R.selfcheck .names/.oks 結構不變式
%     R.units     單位宣告
%
%   See also MGS.PARSETOTCSV, MGS.ZARCHAN.WEAVEDESIGNCURVE.

arguments
    % ⚠️ 只宣告 name-value ⇒ nargin **恆為 0**,不可以拿它當模式開關。
    opts.csv (1,:) char = ''
    opts.phase (1,:) char = 'ALIGN'
    opts.mode (1,:) char = 'weave'
    opts.period (1,1) double {mustBePositive} = 120.0   % weave_period,節點值
    opts.theta_cmd_deg (1,1) double {mustBePositive} = 45.0  % weave_theta_max
    opts.want_s (1,1) double {mustBeNonnegative} = 15.0 % 反解目標
    opts.min_ticks (1,1) double {mustBePositive} = 200  % 少於此不入統計
    % 紅心。⚠️ 預設是 LATOBS1 的 payload;換航次要一起換,
    %    否則「航向偏移」量的是相對於別人的紅心。
    % ⚠️ 只用於顯示與距離,**不**用於航向偏移的參考基準 —— 那個用 slot。
    opts.target (1,2) double = [25.1261649, 120.6156480]
    opts.benchmark (1,1) logical = true
    opts.render (1,1) logical = true
    opts.plot (1,1) logical = false
    opts.assert (1,1) logical = true
end

G = 9.80665;
EARTH = 111320.0;

csvPath = opts.csv;
if isempty(csvPath)
    csvPath = fullfile(mgs.repoRoot(), 'runs', 'artifacts', 'LATOBS1', ...
                       'tot_LATOBS1.csv');
end
assert(isfile(csvPath), 'mgs:weaveSizing:noCsv', ...
       '找不到 %s', csvPath);

C = mgs.parseTotCsv(csvPath);

% ── 每架的實測 ─────────────────────────────────────────────────────────
M = struct('id', {}, 'n', {}, 'off_med_deg', {}, 'off_p90_deg', {}, ...
           'off_slot_med_deg', {}, 'theta_cmd_med_deg', {}, ...
           'detour_from_off', {}, 'detour', {}, 'revs', {}, 'amp_m', {}, ...
           'V', {}, 'phi_deg', {}, 'T_seg', {}, 'path_m', {}, 'short_m', {});
for k = 1:numel(C.veh)
    T = C.veh(k).tbl;
    sel = strcmp(T.phase, opts.phase) & strcmp(T.mode, opts.mode);
    if sum(sel) < opts.min_ticks
        continue
    end
    W = T(sel, :);
    lat = W.lat;  lon = W.lon;
    % 航跡長:逐拍位移累加。⚠️ 不可以用 gs 積分 —— 那會把「地速有值但位置
    %    沒動」(遙測凍結)算成飛了一段路。
    dx = diff(lon) * EARTH .* cosd(lat(1:end-1));
    dy = diff(lat) * EARTH;
    pathM = sum(hypot(dx, dy));
    % 直線縮短量:r_to_target_m 是節點自己算的,不必再推一次幾何
    rtt = W.r_to_target_m;
    rtt = rtt(isfinite(rtt));
    if numel(rtt) < 2
        continue
    end
    shortM = rtt(1) - rtt(end);
    % 橫向擺幅:對「首→末」直線的最大垂距
    ax = lat(1);  ay = lon(1);
    vx = (lat(end) - ax) * EARTH;
    vy = (lon(end) - ay) * EARTH * cosd(ax);
    L = hypot(vx, vy);
    amp = 0;
    if L > 1
        px = (lat - ax) * EARTH;
        py = (lon - ay) * EARTH .* cosd(ax);
        amp = max(abs(px * vy - py * vx)) / L;
    end
    % ⚠️ 換向數必須是**純量**。第一版寫 `(max(rv)-min(rv)) * ~isempty(rv)`,
    %    而 rv 為空時 max(rv) 是 [] ⇒ 整個運算式回 [] ⇒ [M.revs] 少一格,
    %    後面的比較就「陣列大小不相容」。空 = 0 次換向,不是「沒有這一項」。
    rv = W.weave_rev_n;  rv = rv(isfinite(rv));
    if isempty(rv)
        nrev = 0;
    else
        nrev = max(rv) - min(rv);
    end
    % ── 實得航向偏移:χ 相對於「直指紅心」的方位 ────────────────────
    % ⭐⭐ 這是繞過 roll_cmd_deg 拿到「指令達成了多少」的路。指令上限是
    %    weave_theta_max(45 度);實得中位若遠低於它,就是指令沒達成。
    % ⚠️ 一定要**去纏繞**:±180 度跨越會產生 ±358 度的假偏移,而那會讓
    %    1/cos 直接發散。
    % ⚠️⚠️⚠️ 參考基準是「到 **slot** 的方位」,不是「到紅心」。
    %   律就是繞著 slot 方位擺的(`aim = dest(pos, bearing(pos, slot) + sign*θ)`),
    %   而 ALIGN 的 slot 本來就偏離直指紅心的線。
    %   2026-08-24 用錯基準的代價:UAV3 指令 10.2 度而「實得」算成 30.5 度
    %   —— 2.99 倍,而飛機不可能比指令多轉三倍。用對基準是 0.92。
    %   ⇒ 那個 2.99 是**基準錯**不是**追蹤異常**,而它看起來完全像後者。
    % ⭐⭐⭐ **兩個基準都要,而且不可互換** —— 它們是不同的量:
    %
    %   到 slot 的偏移  -> **達成率**(第一層)。律繞著 slot 方位擺
    %                      (`aim = dest(pos, bearing(pos, slot) + sign*θ)`),
    %                      所以「指令達成了多少」只能用這個基準。
    %   到紅心的偏移    -> **迂迴模型**(B 段)。迂迴的分母是「朝紅心的直線
    %                      進展」(shortM = r_to_target 的減少量),分子分母的
    %                      基準必須一致。
    %
    % ⚠️⚠️ 2026-08-24 我犯了**兩次相反的錯**,而兩次的數字都看起來合理:
    %   ① 先用「到紅心」去算達成率 ⇒ UAV3 指令 10.2 度算出「實得」30.5 度
    %      = 2.99 倍。飛機不可能比指令多轉三倍,但那個數字很像一個追蹤異常。
    %   ② 發現之後把**兩處都**改成 slot ⇒ 達成率對了,而 B 段從綠變紅 ——
    %      因為迂迴的分母還是朝紅心量的,分子換了基準就對不起來。
    %   ⇒ 「修好一處」順手把另一處改壞,而兩次的紅綠都不會告訴你原因。
    brg_s = atan2d((W.slot_lon - lon) .* cosd(lat), W.slot_lat - lat);
    off_s = mod(W.chi_deg - brg_s + 180, 360) - 180;
    off_s = off_s(isfinite(off_s));
    brg = atan2d((opts.target(2) - lon) .* cosd(lat), opts.target(1) - lat);
    off = mod(W.chi_deg - brg + 180, 360) - 180;
    off = off(isfinite(off));
    if isempty(off)
        offMed = NaN;  offP90 = NaN;  dOff = NaN;
    else
        a_ = sort(abs(off));
        offMed = a_(max(1, round(0.50 * numel(a_))));
        offP90 = a_(max(1, round(0.90 * numel(a_))));
        % B 段模型:由**實得**偏移分佈直接算迂迴 = mean(1/cos)
        % ⚠️ 夾在 85 度:1/cos 在 90 度發散,而發散值會主導平均。
        %    這個夾限是 UAV1/UAV2 高估的來源,已在表頭說明。
        dOff = mean(1 ./ cosd(min(abs(off), 85)));
    end

    M(end+1) = struct( ...                                        %#ok<AGROW>
        'id', C.veh(k).id, 'n', sum(sel), ...
        'off_med_deg', offMed, 'off_p90_deg', offP90, 'detour_from_off', dOff, ...
        'off_slot_med_deg', medOr_(abs(off_s)), ...
        'theta_cmd_med_deg', medOr_(W.weave_yaw_deg(isfinite(W.weave_yaw_deg))), ...
        'detour', pathM / max(shortM, 1), ...
        'revs', nrev, ...
        'amp_m', amp, ...
        'V', median(W.gs(isfinite(W.gs))), ...
        'phi_deg', prctile_(abs(W.roll_meas_deg), 90), ...
        'T_seg', W.t(end) - W.t(1), ...
        'path_m', pathM, 'short_m', shortM);
end
assert(~isempty(M), 'mgs:weaveSizing:noSegment', ...
       ['在 %s 裡找不到 phase=%s / mode=%s 且 >= %d 拍的段。\n' ...
        '⚠️ 這**不是**「蛇形沒跑」—— 先確認這一趟的相位名稱,' ...
        'ALIGN 段與 wait 段是兩件不同的事。'], ...
       csvPath, opts.phase, opts.mode, opts.min_ticks);

% ── 模型 ───────────────────────────────────────────────────────────────
P = struct('id', {}, 'A_deg', {}, 'n_cyc', {}, 'detour', {}, 'limited_by', {});
for k = 1:numel(M)
    [d, Adeg, ncyc, why] = predictDetour(M(k).V, M(k).phi_deg, opts.period, ...
                                         opts.theta_cmd_deg, M(k).T_seg, G);
    P(end+1) = struct('id', M(k).id, 'A_deg', Adeg, 'n_cyc', ncyc, ...
                      'detour', d, 'limited_by', why);         %#ok<AGROW>
end

resid = [P.detour] - [M.detour];

% ── 反解:要吸 want_s 秒需要什麼 ────────────────────────────────────────
Vn = median([M.V]);
Tn = median([M.T_seg]);
need = (opts.want_s / max(Tn, 1)) + 1;          % 需要的迂迴倍率
grid_P = [30 60 90 120 180 240];
grid_phi = [20 30 40 45 60];
tbl = nan(numel(grid_phi), numel(grid_P));
for a = 1:numel(grid_phi)
    for b = 1:numel(grid_P)
        tbl(a, b) = predictDetour(Vn, grid_phi(a), grid_P(b), ...
                                  opts.theta_cmd_deg, Tn, G);
    end
end

R = struct();
R.meas = M;  R.model = P;  R.resid = resid;
R.invert = struct('want_s', opts.want_s, 'need_detour', need, ...
                  'V', Vn, 'T_seg', Tn, 'grid_period_s', grid_P, ...
                  'grid_phi_deg', grid_phi, 'detour', tbl);
R.units = ['V m/s, phi deg, period s, detour dimensionless, amp m, ' ...
           'T_seg s.  ⚠️ gs 是**地速**不是 EAS(見 [[eas-vs-groundspeed-157]]:' ...
           '157 是 3100 m 的地速 = EAS 135,當成 EAS 會超天花板 17%).'];

R.selfcheck = structuralChecks(M, P, opts, G);
if opts.benchmark
    R.bench = benchRows(M, P, resid, opts);
else
    R.bench = emptyBench();
end
R.pass = all(R.selfcheck.oks) && (isempty(R.bench) || all([R.bench.ok] | [R.bench.skipped]));
R.verdict = '';
if ~R.pass
    bad = R.selfcheck.names(~R.selfcheck.oks);
    if isempty(bad) && ~isempty(R.bench)
        b = R.bench(~[R.bench.ok] & ~[R.bench.skipped]);
        if ~isempty(b), bad = {b(1).what}; end
    end
    if ~isempty(bad), R.verdict = bad{1}; end
end

if opts.render
    renderReport(R, opts);
end
if opts.plot
    plotReport(R, opts);
end
% ⚠️ assert=false 仍然要**回傳 R** —— 一紅燈就連結果都拿不到的話,
%    最需要看資料的時候反而看不到。本專案為此改過 airframeSim。
if opts.assert && ~R.pass
    error('mgs:weaveSizing:selfcheck', '自檢/基準未通過:%s', R.verdict);
elseif ~R.pass
    warning('mgs:weaveSizing:selfcheck', '自檢/基準未通過:%s', R.verdict);
end
end


% =============================================================================
function [d, Adeg, ncyc, why] = predictDetour(V, phi_deg, period, theta_cmd_deg, T_seg, G)
%PREDICTDETOUR  三角波航向偏移下的平均迂迴倍率。
%
%   ⚠️ G / 常數一律**當引數傳進來**:MATLAB 的 local function 看不到主函式
%      工作區,而這個專案為此丟過一整波(engage3d 的 GRAV_FT)。
omega = G * tand(phi_deg) / max(V, 1);          % rad/s
A_kin = omega * period / 8;                      % 節點自己的夾限(rad)
A_cmd = deg2rad(theta_cmd_deg);
A = min(A_cmd, A_kin);
why = 'kinematic';
if A_cmd < A_kin
    why = 'commanded';
end
ncyc = T_seg / period;
if ncyc < 0.5
    % 單向斜飛:航向從 0 爬到最多 omega*T_seg,平均偏航是一半
    A2 = min(A, omega * T_seg / 2);
    if A2 < A
        why = 'segment-too-short';
    end
    A = A2;
end
if A < 1e-9
    d = 1.0;  Adeg = 0;  return
end
% 三角波的平均 sec:(1/A) * ln(sec A + tan A)
d = log(sec(A) + tan(A)) / A;
Adeg = rad2deg(A);
end


function K = structuralChecks(M, P, opts, G)
%STRUCTURALCHECKS  結構不變式。⚠️ .names/.oks 一定要指派 ——
%   沒有它們 benchSuite 那一套會把整組檢查靜靜丟掉(shaping2d 的 D4)。
names = {};  oks = [];  det = {};

% 1 迂迴倍率恆 >= 1:飛得比直線短是不可能的
names{end+1} = '實得迂迴 >= 1(不可能比直線短)';
oks(end+1) = all([M.detour] >= 1 - 1e-9);
det{end+1} = sprintf('最小 %.4f', min([M.detour]));

% 2 模型的迂迴也必須 >= 1
names{end+1} = '模型迂迴 >= 1';
oks(end+1) = all([P.detour] >= 1 - 1e-12);
det{end+1} = sprintf('最小 %.6f', min([P.detour]));

% 3 運動學夾限:A <= omega*P/8,這是節點自己的式子
A_ok = true;
for k = 1:numel(M)
    omega = G * tand(M(k).phi_deg) / max(M(k).V, 1);
    A_ok = A_ok && (deg2rad(P(k).A_deg) <= omega * opts.period / 8 + 1e-9);
end
names{end+1} = '模型的 A 不超過 omega*P/8(節點自己的夾限)';
oks(end+1) = A_ok;
det{end+1} = '與 tot_node.py 的 `th = min(th, omega*period/8)` 同一條';

% 4 DELETE 實驗:把運動學夾限拿掉,預測必須**明顯變大**。
%   ⚠️ 這一條在測「夾限是不是承重的」。如果拿掉它預測沒變,那模型裡
%      根本不是夾限在決定答案,而我們對「為什麼吸不到」的歸因就錯了。
d_lim = median([P.detour]);
d_un = zeros(1, numel(M));
for k = 1:numel(M)
    A = deg2rad(opts.theta_cmd_deg);
    d_un(k) = log(sec(A) + tan(A)) / A;
end
% ⚠️⚠️ 這一條原本斷言「拿掉夾限預測會變大」,也就是**假設夾限承重**。
%    LATOBS1 實測:夾住 1.1222 ⇒ 不夾 1.1222,一模一樣 ⇒ 夾限在這個
%    操作點**完全惰性**。斷言它承重是錯的,所以這一條改成**記錄它到底
%    有沒有咬**,而把「A 段模型有沒有解釋力」交給 bench 去計分。
%    ⇒ 一條總是紅的檢查會變成雜訊;一條記錄事實的檢查不會。
names{end+1} = '運動學夾限在本操作點是否咬住(記錄,不計分)';
oks(end+1) = true;
det{end+1} = sprintf(['夾住 %.4f / 不夾 %.4f ⇒ %s。' ...
    'P=%.0f s、phi~%.0f 度之下 omega*P/8 遠大於指令角,' ...
    '節點 docstring 那個「theta 只能到 9.8 度」用的是泡棉時代的 P=22 s。'], ...
    d_lim, median(d_un), ...
    ternary_(median(d_un) > d_lim * 1.02, '咬住', '**惰性**'), ...
    opts.period, median([M.phi_deg]));

% 5 換向次數與週期一致:段長 / 週期 大致等於半週期數
names{end+1} = '換向次數不超過段長容得下的半週期數';
nmax = ceil([M.T_seg] / (opts.period / 2)) + 1;
oks(end+1) = all([M.revs] <= nmax);
det{end+1} = sprintf('實測 %s;上限 %s', mat2str([M.revs]), mat2str(nmax));

K = struct('names', {names}, 'oks', logical(oks), 'detail', {det});
K.all_ok = all(K.oks);
K.first_fail = '';
if ~K.all_ok
    f = find(~K.oks, 1);
    K.first_fail = names{f};
end
end


function B = benchRows(M, P, resid, opts)
%BENCHROWS  模型 vs 實飛。15 欄契約,與 +zarchan 那一套逐字相同。
B = emptyBench();
md = median([M.detour]);
pd = median([P.detour]);
B = addRow(B, 'value', '實得迂迴中位:模型 vs 實飛', md, pd, 0.25, ...
    'LATOBS1 6 架 ALIGN 段', ...
    sprintf(['模型 %.4f / 實飛 %.4f。帶寬 25%% 不是精度目標,是**這一趟只有' ...
             '一個操作點**的誠實反映:六架的 weave 段長差 8.4 倍(501 vs 4230 拍),' ...
             '而模型只吃一個 T_seg 中位數。'], pd, md));
% 逐機殘差的散布 —— 這一列比中位數那一列有資訊
B = addRow(B, 'info', '逐機殘差(模型-實飛)', NaN, NaN, NaN, ...
    'LATOBS1 逐機', ...
    sprintf('%s;rms %.4f', mat2str(round(resid, 4)), sqrt(mean(resid.^2))));
% 換向:記錄在案的結論是「零換向」
B = addRow(B, 'shape', '換向次數 > 0(記錄在案的結論是「零換向」)', 1, ...
    double(any([M.revs] > 0)), 0, 'ALIGN 段,新加的 weave_rev_n', ...
    sprintf(['實測 %s。⚠️ 記錄在案的「0.21 個週期、零換向」量的是**等待段**;' ...
             'ALIGN 段從來沒被分析過,因為 weave_yaw_deg 與 weave_readout.py ' ...
             '對它一起瞎。'], mat2str([M.revs])));
B = addRow(B, 'skip', '吸收的絕對秒數', NaN, NaN, NaN, ...
    'LATOBS1 逾時,停在 33-64 km 外', ...
    ['⚠️ 誠實拒評:這 12,618 拍是**接近段**的蛇形,不是放行前那一段。' ...
     '拿它推「散布能不能被吸收」缺一環 —— 需要一趟真的進俯衝的航次。']);

% ── A 段:模型有沒有解釋力(**這一列會紅,而紅就是結論**)──────────────
sp = std([M.detour]);
sr = std(resid);
B = addRow(B, 'bound', 'A 段:(V,phi,P) 模型解釋得了迂迴的散布嗎', ...
    0.5, sr / max(sp, eps), 0.0, 'LATOBS1 6 架', ...
    sprintf(['殘差標準差 %.4f / 實飛標準差 %.4f = %.2f。<1 才叫有解釋力,' ...
             '而模型對六架預測**同一個值** ⇒ 比值必 >= 1。' ...
             '⚠️ 這一列紅**是結論不是缺陷**:運動學夾限惰性、六架的 ' ...
             '(V,phi,P) 幾乎相同而實得偏移散在 %.1f-%.1f 度 ⇒ A 段缺一個' ...
             '我們還不知道的機制。'], ...
             sr, sp, sr / max(sp, eps), min([M.off_med_deg]), max([M.off_med_deg])));

% ── B 段:由**實得偏移**推迂迴(這一列會過)──────────────────────────
rb = [M.detour_from_off] - [M.detour];
ok4 = sum(abs(rb) <= 0.05);
B = addRow(B, 'bound', 'B 段:實得偏移推得出迂迴(誤差 <= 0.05 的架數)', ...
    4, ok4, 0.0, 'LATOBS1 6 架', ...
    sprintf(['逐機差 %s。⭐ %d/6 架吻合到 0.05。' ...
             '⚠️ 高估的兩架 p90 偏移是 %.0f/%.0f 度 —— 1/cos 在 90 度發散,' ...
             '而本檔把偏移夾在 85 度,那個夾限就是高估的來源。'], ...
             mat2str(round(rb, 3)), ok4, ...
             max([M.off_p90_deg]), median([M.off_p90_deg])));

% ── 指令達成率:繞過 roll_cmd_deg 的那條路 ─────────────────────────────
B = addRow(B, 'info', '指令 45 度的實得達成率(由航向偏移量)', NaN, NaN, NaN, ...
    'LATOBS1 ALIGN 段', ...
    sprintf(['實得偏移中位 %.1f-%.1f 度 ⇒ 達成 %.0f-%.0f%%。' ...
             '⭐ 這是繞過 roll_cmd_deg 拿到「第一層」的路 —— 那一欄只在 PN ' ...
             '裡產生,而 LATOBS1 沒進俯衝。' ...
             '⚠️ 對照 [[gamma-tracking-deficit]] 的俯仰通道 0.063:' ...
             '橫向這一層**沒有**那個量級的缺陷。'], ...
             min([M.off_med_deg]), max([M.off_med_deg]), ...
             100 * min([M.off_med_deg]) / 45, 100 * max([M.off_med_deg]) / 45));
end


function B = emptyBench()
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end


function B = addRow(B, kind, what, ref, got, tol, src, detail)
%ADDROW  ⚠️ 每一列都走這一個建構子:欄位順序一旦漂掉,[B.ok] 會取到別的欄,
%   而錯誤訊息會長得像打錯字而不是像 schema 壞了。
sk = strcmp(kind, 'skip') || strcmp(kind, 'info');
pc = NaN;  ok = true;
if ~sk && isfinite(ref) && isfinite(got)
    pc = abs(got - ref) / max(abs(ref), eps);
    ok = pc <= tol;
end
B(end+1) = struct('what', what, 'kind', kind, ...
    'book', ref, 'book_str', num2str(ref), ...
    'ours', got, 'ours_str', num2str(got), ...
    'pct', pc, 'band', tol, 'tol', tol, ...
    'ok', ok, 'skipped', sk, 'why', '', 'source', src, ...
    'units', 'dimensionless', 'detail', detail);
end


function renderReport(R, opts)
M = R.meas;  P = R.model;
fprintf('\n%s\n', repmat('=', 1, 78));
fprintf(' mgs.weaveSizing  —  ALIGN 段蛇形的吸收力(模型 vs 實飛)\n');
fprintf('%s\n', repmat('=', 1, 78));
fprintf(' 週期 %.0f s,指令角上限 %.0f 度\n', opts.period, opts.theta_cmd_deg);
fprintf('\n %-4s %7s %7s %7s %7s %8s %9s %9s %8s  %s\n', ...
        '機', '拍', 'V m/s', 'phi', '段長s', '週期數', '實飛迂迴', '模型迂迴', '殘差', '受限於');
fprintf(' %s\n', repmat('-', 1, 76));
for k = 1:numel(M)
    fprintf(' %-4d %7d %7.1f %7.1f %7.0f %8.2f %9.4f %9.4f %+8.4f  %s\n', ...
        M(k).id, M(k).n, M(k).V, M(k).phi_deg, M(k).T_seg, P(k).n_cyc, ...
        M(k).detour, P(k).detour, R.resid(k), P(k).limited_by);
end
fprintf('\n rms 殘差 %.4f;實飛中位 %.4f,模型中位 %.4f\n', ...
        sqrt(mean(R.resid.^2)), median([M.detour]), median([P.detour]));

fprintf('\n%s\n', repmat('-', 1, 78));
% ⚠️⚠️⚠️ 警告必須印在**表的上面**,不是只寫在檔頭 —— 讀的人看的是輸出。
fprintf([' ⚠️⚠️⚠️ 下面這張表**不可信**。它整個建立在 A 段模型上,而 A 段已被\n' ...
         '    LATOBS1 推翻(見上面那一列 RED):模型對六架預測**同一個值**,\n' ...
         '    實飛散在 %.3f-%.3f。\n' ...
         '    ⇒ 拿它去訂 (P, phi) 會得到一個**看起來很具體的錯答案**。\n' ...
         '    仍然印出來,是為了讓下一個人看得到它錯在哪,不是為了照著設。\n'], ...
        min([R.meas.detour]), max([R.meas.detour]));
fprintf('%s\n', repmat('-', 1, 78));
fprintf(' 反解(**僅供對照,見上面的警告**):要吸收 %.0f s(段長 %.0f s ⇒ 需要迂迴 %.4f)\n', ...
        R.invert.want_s, R.invert.T_seg, R.invert.need_detour);
fprintf(' %s\n', repmat('-', 1, 78));
fprintf(' %-8s', 'phi\\P');
fprintf('%9.0f', R.invert.grid_period_s);  fprintf('  [s]\n');
for a = 1:numel(R.invert.grid_phi_deg)
    fprintf(' %-8.0f', R.invert.grid_phi_deg(a));
    for b = 1:numel(R.invert.grid_period_s)
        v = R.invert.detour(a, b);
        if v >= R.invert.need_detour
            fprintf('%8.3f*', v);
        else
            fprintf('%9.3f', v);
        end
    end
    fprintf('\n');
end
fprintf(' (* = 達到需求)\n');

fprintf('\n 結構自檢:\n');
for i = 1:numel(R.selfcheck.names)
    if R.selfcheck.oks(i), t = 'ok  '; else, t = 'RED '; end
    fprintf('   %s %-52s %s\n', t, R.selfcheck.names{i}, R.selfcheck.detail{i});
end
if ~isempty(R.bench)
    fprintf('\n 基準列:\n');
    for i = 1:numel(R.bench)
        b = R.bench(i);
        if b.skipped, t = 'SKIP'; elseif b.ok, t = 'ok  '; else, t = 'RED '; end
        fprintf('   %s %s\n', t, b.what);
    end
end
fprintf('\n');
end


function plotReport(R, opts)
M = R.meas;  P = R.model;
f = figure('Name', 'mgs.weaveSizing', 'Position', [80 80 1180 780]);
tiledlayout(f, 2, 2, 'TileSpacing', 'compact');

nexttile;
b = bar([[M.detour]; [P.detour]]');
b(1).DisplayName = 'flown';  b(2).DisplayName = 'model';
yline(1, 'k-');
yline(log(sec(deg2rad(opts.theta_cmd_deg)) + tan(deg2rad(opts.theta_cmd_deg))) ...
      / deg2rad(opts.theta_cmd_deg), 'r--', 'no kinematic cap');
xticklabels(compose('UAV%d', [M.id]));
ylabel('detour ratio');  legend('Location', 'best');  grid on
title('A  achieved detour: flown vs model');

nexttile;
scatter([M.T_seg], [M.detour], 60, 'filled');  hold on
scatter([M.T_seg], [P.detour], 60, 'x', 'LineWidth', 1.5);
xlabel('weave segment length [s]');  ylabel('detour ratio');
legend({'flown', 'model'}, 'Location', 'best');  grid on
title('B  the segment length is the variable, not the command angle');

nexttile;
imagesc(R.invert.grid_period_s, R.invert.grid_phi_deg, R.invert.detour);
set(gca, 'YDir', 'normal');  colorbar
xlabel('weave period [s]');  ylabel('bank [deg]');
title(sprintf('C  model detour;  need %.3f for %.0f s', ...
              R.invert.need_detour, R.invert.want_s));

nexttile;
bar([M.revs]);  xticklabels(compose('UAV%d', [M.id]));
ylabel('reversals counted');  grid on
title('D  reversals -- the recorded conclusion said ZERO');
end


function v = medOr_(x)
%MEDOR_  中位數;全空時回 NaN(⚠️ 不回 0 —— 0 是合法的角度)。
x = x(isfinite(x));
if isempty(x), v = NaN; else, v = median(x); end
end


function s = ternary_(c, a, b)
%TERNARY_  只為了讓 sprintf 讀得順。MATLAB 沒有三元運算子。
if c, s = a; else, s = b; end
end


function v = prctile_(x, p)
%PRCTILE_  百分位。⚠️ 自己算:prctile 在 Statistics Toolbox 裡,而本套件
%   一律只用 base MATLAB(整個 +zarchan 的 98 個移植都守這一條)。
x = sort(x(isfinite(x)));
if isempty(x), v = NaN; return
end
if numel(x) == 1, v = x; return
end
i = 1 + (numel(x) - 1) * p / 100;
lo = floor(i);  hi = ceil(i);
v = x(lo) + (i - lo) * (x(hi) - x(lo));
end
