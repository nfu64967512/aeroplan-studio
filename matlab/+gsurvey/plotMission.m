function plotMission(out, fname)
%GSURVEY.PLOTMISSION  全任務六格圖:起飛 → AUTO → 決斷圈 → GUIDED → 命中。

arguments
    out struct
    fname char = ''
end
p = out.p; n = p.n; cm = lines(n);
PH = {'TAKEOFF','CLIMB','TRANSIT','LOITER','DIVE'};

f = figure('Position',[40 40 1500 900],'Color','w','Visible','off');
tl = tiledlayout(f,2,3,'TileSpacing','compact','Padding','compact');
title(tl, sprintf(['AUTO 航線全任務  n=%d  ‧  決斷圈 %.0f m(auto\\_ring: perch %.0f / tan %.0f°)' ...
    '  ‧  盤旋指令 %.0f m  vs  最小迴轉 %.0f m  ‧  閘門 t=%.0f s'], ...
    n, out.ringR, p.perchAlt, rad2deg(p.diveAngle), p.autoLoiterR, out.turnRmin, out.tGate), ...
    'FontWeight','bold');

% ① 俯視全景
ax = nexttile(tl,1); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
th = linspace(0,2*pi,200);
plot(ax, out.ringR*cos(th)/1000, out.ringR*sin(th)/1000, 'k--', 'LineWidth',1.2);
for i=1:n
    Tj = out.veh(i).traj;
    plot(ax, Tj.pe/1000, Tj.pn/1000, 'Color', cm(i,:), 'LineWidth',1.0);
end
plot(ax,0,0,'rp','MarkerSize',16,'MarkerFaceColor','r');
xlabel(ax,'東 [km]'); ylabel(ax,'北 [km]'); title(ax,'① 俯視:起飛 → 決斷圈 → 命中');
legend(ax, {'決斷圈'}, 'Location','southwest');

% ② 決斷圈特寫
ax = nexttile(tl,2); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
plot(ax, out.ringR*cos(th)/1000, out.ringR*sin(th)/1000, 'k--','LineWidth',1.2);
for i=1:n
    Tj = out.veh(i).traj; m = Tj.phase>=4;
    if any(m), plot(ax, Tj.pe(m)/1000, Tj.pn(m)/1000, 'Color', cm(i,:), 'LineWidth',1.2); end
    ip = out.veh(i).IP; plot(ax, ip(2)/1000, ip(1)/1000, 'o', 'MarkerSize',5, ...
        'MarkerFaceColor',cm(i,:), 'MarkerEdgeColor','k');
end
plot(ax,0,0,'rp','MarkerSize',16,'MarkerFaceColor','r');
xlabel(ax,'東 [km]'); ylabel(ax,'北 [km]'); title(ax,'② 決斷圈盤旋 + 俯衝(特寫)');

% ③ 高度剖面
ax = nexttile(tl,3); hold(ax,'on'); grid(ax,'on');
for i=1:n
    Tj = out.veh(i).traj;
    plot(ax, hypot(Tj.pn,Tj.pe)/1000, -Tj.pd, 'Color', cm(i,:), 'LineWidth',1.0);
end
xline(ax, out.ringR/1000, 'k--', '決斷圈');
yline(ax, p.perchAlt, 'k:', sprintf('perch %.0f m', p.perchAlt));
plot(ax,0,0,'rp','MarkerSize',14,'MarkerFaceColor','r');
set(ax,'XDir','reverse');
xlabel(ax,'距紅心水平距 [km]'); ylabel(ax,'高度 [m]'); title(ax,'③ 高度剖面');

% ④ 相位時序
ax = nexttile(tl,4); hold(ax,'on'); grid(ax,'on');
for i=1:n
    Tj = out.veh(i).traj;
    plot(ax, Tj.t, Tj.phase + (i-1)*0.06, 'Color', cm(i,:), 'LineWidth',1.6);
end
xline(ax, out.tGate, 'r-', '全員到齊閘門', 'LineWidth',1.4);
yticks(ax,1:5); yticklabels(ax, PH);
xlabel(ax,'時間 [s]'); title(ax,'④ 相位時序');

% ⑤ 航跡角
ax = nexttile(tl,5); hold(ax,'on'); grid(ax,'on');
for i=1:n
    Tj = out.veh(i).traj;
    plot(ax, Tj.t, Tj.gam, 'Color', cm(i,:), 'LineWidth',1.0);
end
xline(ax, out.tGate,'r-'); yline(ax,-27,'k:','dive\_angle 27°');
xlabel(ax,'時間 [s]'); ylabel(ax,'\gamma [deg]'); title(ax,'⑤ 航跡角');

% ⑥ R0 —— 這一趟的頭號量
ax = nexttile(tl,6); hold(ax,'on'); grid(ax,'on');
R0 = [out.veh.R0]; ids = 1:n;
b = bar(ax, ids, R0, 0.6); b.FaceColor='flat'; b.CData = cm;
yline(ax, out.ringR, 'k--', sprintf('決斷圈 %.0f m', out.ringR), 'LineWidth',1.2);
yline(ax, 7000, 'g--', 'R0≥7000:殘餘超標 0.0%', 'LineWidth',1.2);
yline(ax, 4000, 'r--', 'R0<4000:殘餘超標 71.4%', 'LineWidth',1.2);
text(ax, 0.5, max(R0)*0.30, sprintf('R0 全距 %.0f m\n中位 %.0f m', ...
    max(R0)-min(R0), median(R0)), 'FontWeight','bold', 'BackgroundColor',[1 1 .85]);
xlabel(ax,'機號'); ylabel(ax,'放行斜距 R_0 [m]');
title(ax,'⑥ 放行距離 —— 殘餘高度的唯一自變數');

if isempty(fname)
    fname = fullfile(fileparts(fileparts(mfilename('fullpath'))),'out','gsurvey','mission.png');
end
d = fileparts(fname); if ~isfolder(d), mkdir(d); end
exportgraphics(f, fname, 'Resolution', 130); close(f);
fprintf('圖:%s\n', fname);
end
