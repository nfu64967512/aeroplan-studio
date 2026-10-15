function files = animate(R, videoRoot)
%GSURVEY.ANIMATE  把每一條律的「發射 → 命中」錄成 MP4。
%
%   files = gsurvey.animate(R)
%   files = gsurvey.animate(R, 'C:\Users\joy46\Videos\aeroplan_guidance_survey')
%
%   每一條律一支影片,三格同步播放:
%       左   3D 立體航跡(含拖尾與當前機位)
%       右上 俯衝剖面:高度 vs 距紅心水平距   ← 這一格是重點
%       右下 指令加速度 vs 時間(含我們的 ~2 g 坡度預算線)
%   另外產一支 compare.mp4:所有律的第 1 架疊在一起。
%
%   ⚠️ 各機的觸地時刻不同。已觸地的機體**留在落點**不消失 ——
%      讓「誰先到、誰後到」在畫面上看得出來,那正是散布。

arguments
    R struct
    videoRoot char = fullfile(getenv('USERPROFILE'), 'Videos', 'aeroplan_guidance_survey')
end

if ~isfolder(videoRoot), mkdir(videoRoot); end
p = R.p; nL = numel(R.law);
files = {};

FPS   = 30;
STEP  = 0.10;                 % 模擬時間的取樣間隔 [s]

% ── 全域座標範圍(所有律共用,影片之間才可比)────────────────────
allN = []; allE = []; allA = []; allT = []; allG = [];
for k = 1:nL
    for i = 1:numel(R.law(k).out.veh)
        T = R.law(k).out.veh(i).traj;
        allN = [allN; T.pn]; allE = [allE; T.pe]; %#ok<AGROW>
        allA = [allA; -T.pd]; allT = [allT; T.t]; %#ok<AGROW>
        allG = [allG; T.aCmd / p.g];              %#ok<AGROW>
    end
end
LIM.n = [min(allN) max(allN)] / 1000;
LIM.e = [min(allE) max(allE)] / 1000;
LIM.a = [0 max(allA) * 1.05];
LIM.t = [0 max(allT) * 1.02];
LIM.r = [0 max(hypot(allN, allE)) / 1000 * 1.02];
LIM.g = [0 min(max(allG) * 1.1, p.aMaxG * 1.6)];   % 需求會爆到幾十 g,截掉

for k = 1:nL
    L = R.law(k);
    slug = i_slug(L.name);
    dirK = fullfile(videoRoot, sprintf('%02d_%s', k, slug));
    if ~isfolder(dirK), mkdir(dirK); end

    fn = fullfile(dirK, sprintf('%02d_%s.mp4', k, slug));
    vw = VideoWriter(fn, 'MPEG-4'); vw.FrameRate = FPS; vw.Quality = 95;
    open(vw);

    f = figure('Position', [60 60 1280 720], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax3 = nexttile(tl, 1, [2 1]);
    axP = nexttile(tl, 2);
    axG = nexttile(tl, 4);

    tEnd = max(arrayfun(@(v) v.tImpact, L.out.veh));
    ts   = 0:STEP:tEnd;
    cmap = lines(numel(L.out.veh));

    for f_i = 1:numel(ts)
        tNow = ts(f_i);
        % ---- 3D ----
        cla(ax3); hold(ax3, 'on'); grid(ax3, 'on'); box(ax3, 'on');
        for i = 1:numel(L.out.veh)
            T = L.out.veh(i).traj;
            m = T.t <= tNow;
            if ~any(m), continue; end
            plot3(ax3, T.pe(m)/1000, T.pn(m)/1000, -T.pd(m), ...
                'Color', cmap(i,:), 'LineWidth', 1.3);
            j = find(m, 1, 'last');
            plot3(ax3, T.pe(j)/1000, T.pn(j)/1000, -T.pd(j), 'o', ...
                'MarkerSize', 6, 'MarkerFaceColor', cmap(i,:), ...
                'MarkerEdgeColor', 'k');
        end
        plot3(ax3, 0, 0, 0, 'rp', 'MarkerSize', 20, 'MarkerFaceColor', 'r');
        xlim(ax3, LIM.e); ylim(ax3, LIM.n); zlim(ax3, LIM.a);
        xlabel(ax3, '東 [km]'); ylabel(ax3, '北 [km]'); zlabel(ax3, '高度 [m]');
        view(ax3, [-40 22]);
        title(ax3, sprintf('%s   t = %5.1f s', L.name, tNow), ...
            'Interpreter', 'none', 'FontWeight', 'bold');

        % ---- 俯衝剖面 ----
        cla(axP); hold(axP, 'on'); grid(axP, 'on');
        for i = 1:numel(L.out.veh)
            T = L.out.veh(i).traj; m = T.t <= tNow;
            if ~any(m), continue; end
            dh = hypot(T.pn(m), T.pe(m)) / 1000;
            plot(axP, dh, -T.pd(m), 'Color', cmap(i,:), 'LineWidth', 1.2);
            j = find(m, 1, 'last');
            plot(axP, dh(end), -T.pd(j), 'o', 'MarkerSize', 5, ...
                'MarkerFaceColor', cmap(i,:), 'MarkerEdgeColor', 'k');
        end
        plot(axP, 0, 0, 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
        set(axP, 'XDir', 'reverse');
        xlim(axP, LIM.r); ylim(axP, LIM.a);
        xlabel(axP, '距紅心水平距 [km]'); ylabel(axP, '高度 [m]');
        title(axP, '俯衝剖面');

        % ---- 指令加速度 ----
        cla(axG); hold(axG, 'on'); grid(axG, 'on');
        for i = 1:numel(L.out.veh)
            T = L.out.veh(i).traj; m = T.t <= tNow;
            if ~any(m), continue; end
            plot(axG, T.t(m), T.aCmd(m)/p.g, 'Color', cmap(i,:), 'LineWidth', 1.1);
        end
        yline(axG, 2.0, 'k:', '我們的坡度預算 ~2 g', 'LabelHorizontalAlignment', 'left');
        yline(axG, p.aMaxG, 'r--');
        xlim(axG, LIM.t); ylim(axG, LIM.g);
        xlabel(axG, '時間 [s]'); ylabel(axG, '指令 [g]');
        title(axG, '權限需求');

        writeVideo(vw, getframe(f));
    end
    % 末幀停留 1 秒
    for r = 1:FPS, writeVideo(vw, getframe(f)); end
    close(vw); close(f);

    % 同一個資料夾放靜態圖與這一條律的 KPI
    src = fullfile(R.outDir, sprintf('law_%02d_%s.png', k, slug));
    if isfile(src), copyfile(src, fullfile(dirK, sprintf('%02d_%s.png', k, slug))); end
    i_writeKpi(fullfile(dirK, 'kpi.txt'), L, p);

    files{end+1} = fn; %#ok<AGROW>
    fprintf('  ✓ %s\n', fn);
end

% ── 總覽影片:所有律的第 1 架 ───────────────────────────────────────
dirC = fullfile(videoRoot, '00_compare');
if ~isfolder(dirC), mkdir(dirC); end
fn = fullfile(dirC, '00_compare.mp4');
vw = VideoWriter(fn, 'MPEG-4'); vw.FrameRate = FPS; vw.Quality = 95; open(vw);
f = figure('Position', [60 60 1280 620], 'Color', 'w', 'Visible', 'off');
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axA = nexttile(tl, 1); axB = nexttile(tl, 2);
cl = lines(nL);
tEnd = max(arrayfun(@(L) max([L.out.veh.tImpact]), R.law));
for tNow = 0:STEP:tEnd
    cla(axA); hold(axA, 'on'); grid(axA, 'on');
    cla(axB); hold(axB, 'on'); grid(axB, 'on');
    h = gobjects(1, nL);
    for k = 1:nL
        T = R.law(k).out.veh(1).traj; m = T.t <= tNow;
        if ~any(m), h(k) = plot(axA, nan, nan, 'Color', cl(k,:)); continue; end
        dh = hypot(T.pn(m), T.pe(m)) / 1000;
        h(k) = plot(axA, dh, -T.pd(m), 'Color', cl(k,:), 'LineWidth', 1.7);
        plot(axA, dh(end), -T.pd(find(m,1,'last')), 'o', 'MarkerSize', 5, ...
            'MarkerFaceColor', cl(k,:), 'MarkerEdgeColor', 'k');
        plot(axB, T.t(m), T.aCmd(m)/p.g, 'Color', cl(k,:), 'LineWidth', 1.3);
    end
    plot(axA, 0, 0, 'rp', 'MarkerSize', 16, 'MarkerFaceColor', 'r');
    set(axA, 'XDir', 'reverse'); xlim(axA, LIM.r); ylim(axA, LIM.a);
    xlabel(axA, '距紅心水平距 [km]'); ylabel(axA, '高度 [m]');
    title(axA, sprintf('所有律的俯衝剖面(第 1 架)   t = %5.1f s', tNow), 'FontWeight','bold');
    yline(axB, 2.0, 'k:', '~2 g');
    xlim(axB, LIM.t); ylim(axB, LIM.g);
    xlabel(axB, '時間 [s]'); ylabel(axB, '指令 [g]'); title(axB, '權限需求');
    legend(axA, h, {R.law.name}, 'Location', 'northeast', 'Interpreter', 'none');
    writeVideo(vw, getframe(f));
end
for r = 1:FPS, writeVideo(vw, getframe(f)); end
close(vw); close(f);
src = fullfile(R.outDir, 'compare_all.png');
if isfile(src), copyfile(src, fullfile(dirC, '00_compare.png')); end
files{end+1} = fn;
fprintf('  ✓ %s\n', fn);
end

% ═══════════════════════════════════════════════════════════════════
function i_writeKpi(fn, L, p)
fid = fopen(fn, 'w', 'n', 'UTF-8');
k = L.kpi;
fprintf(fid, '%s\n%s\n\n', k.name, L.note);
fprintf(fid, '命中時刻散布   %.3f s\n', k.spread);
fprintf(fid, '水平落點誤差   中位 %.2f m / 最大 %.2f m\n', k.missH, k.missHmax);
fprintf(fid, '落角           %.1f deg  (期望 %.0f deg)\n', k.gamF, rad2deg(p.gammaF));
fprintf(fid, '指令加速度     中位 %.2f g / 峰值 %.2f g\n', k.aMed, k.aMax);
fprintf(fid, '飽和比例       %.1f %%   (上限 %.1f g)\n', k.sat, p.aMaxG);
fprintf(fid, '航跡比         %.3f   (實際航程 / 直線距離)\n', k.track);
fprintf(fid, '終端速度       %.1f m/s\n\n', k.Vf);
fprintf(fid, '場景:n=%d、放行斜距 %.0f m(散度 %.0f m)、高度 %.0f m、速率 %.1f m/s\n', ...
    p.n, p.R0, p.R0spread, p.alt0, p.V0);
fprintf(fid, '純運動學,無飛控、無夾制鏈、無姿態迴路(見 matlab/+gsurvey/Contents.m)\n');
fclose(fid);
end

function s = i_slug(x)
s = regexprep(x, '[^A-Za-z0-9]+', '_');
s = regexprep(s, '^_|_$', '');
if isempty(s), s = 'law'; end
end
