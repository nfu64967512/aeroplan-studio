function plotRuns(R)
%GSURVEY.PLOTRUNS  軌跡圖 —— 這才是本次要看的東西:**發射到命中的軌跡變化**。
%
%   每一條律一張四格圖:
%       (1) 側視:高度 vs 距紅心水平距   ← 俯衝剖面,最重要的一張
%       (2) 俯視:水平航跡              ← 橫向繞路看得見
%       (3) 指令加速度 vs 時間           ← 權限需求的時間分佈
%       (4) 航跡角 γ vs 時間             ← 落角是怎麼長出來的
%   外加一張總覽:所有律的俯衝剖面疊在一起。

p = R.p; outDir = R.outDir;
nL = numel(R.law);
cmap = lines(max(numel(R.law(1).out.veh), 8));

% ── 每條律一張 ─────────────────────────────────────────────────────
for k = 1:nL
    L = R.law(k);
    f = figure('Position', [80 80 1180 760], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, sprintf('%s   —   %s   (n=%d)', L.name, L.note, p.n), ...
        'FontWeight', 'bold', 'Interpreter', 'none');

    % (1) 側視:高度 vs 水平距
    ax1 = nexttile(tl, 1); hold(ax1, 'on'); grid(ax1, 'on');
    for i = 1:numel(L.out.veh)
        T = L.out.veh(i).traj;
        dh = hypot(T.pn, T.pe);
        plot(ax1, dh/1000, -T.pd, 'Color', cmap(i,:), 'LineWidth', 1.1);
    end
    plot(ax1, 0, 0, 'rp', 'MarkerSize', 13, 'MarkerFaceColor', 'r');
    set(ax1, 'XDir', 'reverse');
    xlabel(ax1, '距紅心水平距 [km]'); ylabel(ax1, '高度 [m]');
    title(ax1, '① 俯衝剖面(側視)');

    % (2) 俯視
    ax2 = nexttile(tl, 2); hold(ax2, 'on'); grid(ax2, 'on'); axis(ax2, 'equal');
    for i = 1:numel(L.out.veh)
        T = L.out.veh(i).traj;
        plot(ax2, T.pe/1000, T.pn/1000, 'Color', cmap(i,:), 'LineWidth', 1.1);
    end
    plot(ax2, 0, 0, 'rp', 'MarkerSize', 13, 'MarkerFaceColor', 'r');
    xlabel(ax2, '東 [km]'); ylabel(ax2, '北 [km]');
    title(ax2, '② 水平航跡(俯視)');

    % (3) 指令加速度
    ax3 = nexttile(tl, 3); hold(ax3, 'on'); grid(ax3, 'on');
    for i = 1:numel(L.out.veh)
        T = L.out.veh(i).traj;
        plot(ax3, T.t, T.aCmd/p.g, 'Color', cmap(i,:), 'LineWidth', 1.0);
    end
    yline(ax3, p.aMaxG, 'r--', sprintf('上限 %.1f g', p.aMaxG));
    yline(ax3, 2.0, 'k:', '我們的坡度預算 ~2 g');
    xlabel(ax3, '時間 [s]'); ylabel(ax3, '指令加速度 [g]');
    title(ax3, '③ 權限需求 vs 時間');

    % (4) 航跡角
    ax4 = nexttile(tl, 4); hold(ax4, 'on'); grid(ax4, 'on');
    for i = 1:numel(L.out.veh)
        T = L.out.veh(i).traj;
        gam = -atan2d(diff([T.pd; T.pd(end)]), hypot(diff([T.pn; T.pn(end)]), ...
                                                     diff([T.pe; T.pe(end)])));
        plot(ax4, T.t, gam, 'Color', cmap(i,:), 'LineWidth', 1.0);
    end
    yline(ax4, -27, 'k:', '生產 dive_angle 27°');
    xlabel(ax4, '時間 [s]'); ylabel(ax4, '航跡角 \gamma [deg]');
    title(ax4, '④ 落角是怎麼長出來的');

    fn = fullfile(outDir, sprintf('law_%02d_%s.png', k, i_slug(L.name)));
    exportgraphics(f, fn, 'Resolution', 130);
    close(f);
end

% ── 總覽:所有律的俯衝剖面(取第 1 架)────────────────────────────
f = figure('Position', [80 80 1180 520], 'Color', 'w', 'Visible', 'off');
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('所有律的比較(第 1 架,n=%d,R_0=%.1f km,V=%.0f m/s)', ...
    p.n, p.R0/1000, p.V0), 'FontWeight', 'bold');
cl = lines(nL);

axA = nexttile(tl, 1); hold(axA, 'on'); grid(axA, 'on');
axB = nexttile(tl, 2); hold(axB, 'on'); grid(axB, 'on');
h = gobjects(1, nL);
for k = 1:nL
    T = R.law(k).out.veh(1).traj;
    dh = hypot(T.pn, T.pe);
    h(k) = plot(axA, dh/1000, -T.pd, 'Color', cl(k,:), 'LineWidth', 1.6);
    plot(axB, T.t, T.aCmd/p.g, 'Color', cl(k,:), 'LineWidth', 1.4);
end
plot(axA, 0, 0, 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
set(axA, 'XDir', 'reverse');
xlabel(axA, '距紅心水平距 [km]'); ylabel(axA, '高度 [m]');
title(axA, '俯衝剖面');
yline(axB, 2.0, 'k:', '我們的坡度預算 ~2 g');
xlabel(axB, '時間 [s]'); ylabel(axB, '指令加速度 [g]');
title(axB, '權限需求');
legend(axA, h, {R.law.name}, 'Location', 'northeast', 'Interpreter', 'none');

exportgraphics(f, fullfile(outDir, 'compare_all.png'), 'Resolution', 130);
close(f);
end

function s = i_slug(x)
s = regexprep(x, '[^A-Za-z0-9]+', '_');
s = regexprep(s, '^_|_$', '');
if isempty(s), s = 'law'; end
end
