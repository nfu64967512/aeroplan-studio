function A = viz6dof(src, opts)
%MGS.VIZ6DOF  Animated 6-DOF POSE view of a multi-vehicle terminal engagement.
%
%   A = mgs.viz6dof(out)                        out from mgs.simSwarm
%   A = mgs.viz6dof('runs/shadow/tot_BASE2.csv')  a real flown sortie
%   A = mgs.viz6dof(out, export="mp4", file="runs/artifacts/RUN/engage.mp4")
%
% ---------------------------------------------------------------------------
%   ⚠️⚠️ READ THIS BEFORE PUTTING A SCREENSHOT IN A REPORT.
%
%   This renders SIX DEGREES OF FREEDOM OF POSE -- three positions and three
%   Euler angles, all animated.  It does NOT come from a 6-DOF rigid-body
%   simulation, and calling it one in a review would not survive the first
%   question.  What is actually underneath (mgs.airframe, verbatim in its own
%   header: "3-DOF point mass + ArduPilot angle loop"):
%
%     n, e, z   DYNAMIC   translational EOM, analytic CD0 + induced-drag polar,
%                         CD0 = 0.01500 inverse-solved from 1617 flight samples
%     theta     DYNAMIC   first-order angle loop, tconst = fc.PTCH2SRV_TCONST
%     phi       DYNAMIC   first-order angle loop, tconst = fc.RLL2SRV_TCONST
%     psi       KINEMATIC coordinated turn; not an independent state
%     p, q, r   ABSENT    no body rates, no moment equations, no inertia tensor
%     beta      ABSENT    no sideslip; the model is bank-to-turn by assumption
%
%   In the standard taxonomy that is a FIVE-DOF model drawn as a six-DOF pose.
%   The figure states this on itself (the DOF ledger panel) so a screenshot
%   carries its own caveat -- that is deliberate, not decoration.
%
%   Going to true 6-DOF would need an inertia tensor and control-surface
%   derivatives for this airframe.  We do not have them: memory
%   `barracuda500-model-facts` records that the CAD body and the body SITL
%   actually flies are not the same object.  Adding invented ones would also
%   break the bit-exact mgs.simSwarm == stot_sim parity that everything else
%   in this environment is validated against.
%
% ---------------------------------------------------------------------------
%   PROVENANCE, PER DEGREE OF FREEDOM
%
%   Every drawn quantity is tagged in the ledger as one of:
%       [DYN]  integrated by the model
%       [MEAS] read from the vehicle's own logged telemetry
%       [KIN]  algebraic from other logged quantities (formula shown)
%       [CMD]  what the guidance layer asked for, not what happened
%
%   The CSV path matters here.  A flown tot_*.csv logs gam_deg, alpha_est_deg,
%   lat/lon/alt/gs -- but roll was NEVER logged before 2026-08-22, so for older
%   sorties phi has to be reconstructed from the coordinated-turn relation
%   tan(phi) = V*psidot / (g*cos(gam)), and it is tagged [KIN], not [MEAS].
%   Runs made after the "2-lite" instrument landed carry roll_cmd_deg and
%   roll_out_deg, and those are tagged [CMD] -- still not measured attitude.
%
%   See also MGS.SIMSWARM, MGS.PARSETOTCSV, MGS.KPI.

arguments
    src
    opts.export  (1,1) string {mustBeMember(opts.export, ["none","mp4","gif","png"])} = "none"
    opts.file    (1,1) string = ""
    opts.fps     (1,1) double = 25
    opts.speed   (1,1) double = 4        % playback x real time
    opts.trail_s (1,1) double = 12       % seconds of trail behind each vehicle
    opts.view    (1,2) double = [-37.5 22]
    opts.title   (1,1) string = ""
    opts.show    (1,1) logical = true
    % ⭐ 單張定格:報告要的是一張構圖好的圖,不是最後一格(最後一格通常
    %    所有機體都已經觸地,HUD 全是 "--"、航跡全空)。NaN = 播完整段。
    opts.at      (1,1) double = NaN
    opts.glyph   (1,1) double = 1.0        % 機身圖示縮放
    % ⚠️ 目標座標。CSV 路徑會自動找同目錄的 *_geom.csv;找不到就**報錯**
    %    而不是猜 —— 每一個畫出來的位置都是相對它算的,datum 錯了整張圖
    %    看起來仍然完全正常。海上紅心是 0 m AMSL(sea-level-target-datum)。
    opts.tgt     (1,2) double = [NaN NaN]  % [lat lon]
    % ⭐ 時間視窗。真實航次含 ~60 km 的轉場,整段一起畫會把**終端交戰**
    %    (唯一有導引律在跑的那一段)壓成一個點。
    %    "dive" = 只留 mode=="dive" 的那一段(飛行資料預設);
    %    "all"  = 整段;或直接給 [t0 t1]。
    opts.focus            = "dive"
    % ⭐ 機身外觀。"glyph" = 內建示意體(零相依、最快);
    %    "cad" = ui/resources/barracuda500.glb;或直接給 .glb 路徑。
    % ⚠️⚠️ CAD **不是飛出這些資料的機體**:所有航次都是 ArduPilot SIM_Plane
    %    的通用 -jet(m=22.0 kg、b=1.88 m、S=0.45 m²),而 CAD 展長 2.325 m、
    %    全長 3.786 m。畫 CAD 等於畫一架**氣動從沒被模擬過**的飛機 ——
    %    當插圖沒問題(帳上會寫),當「已驗證模型」就不行。
    opts.model   (1,1) string = "glyph"
    % ⭐ "dash" = 完整儀表板(預設);"simple" = 只有 3D 場景 + 一行但書,
    %    素色背景,適合當報告的主視覺。
    opts.style   (1,1) string {mustBeMember(opts.style, ["dash","simple"])} = "dash"
    opts.vvec    (1,1) logical = true       % 畫速度向量箭頭
    % ⭐ probe = only load + focus, then return.  A caller that needs the time
    %    window (to choose a freeze frame, say) should not have to render a
    %    whole animation to find out what it is.
    opts.probe   (1,1) logical = false
end

A = load_source(src, opts.tgt);
if opts.title ~= "", A.title = char(opts.title); end
A = apply_focus(A, opts.focus);
if opts.probe, return; end

% ── time base ────────────────────────────────────────────────────────────
t0 = min(arrayfun(@(v) v.t(1),   A.veh));
t1 = max(arrayfun(@(v) v.t(end), A.veh));
if isfinite(opts.at)
    if opts.at < t0 || opts.at > t1
        error('mgs:viz6dof:atRange', ...
            'at = %g s is outside the run [%.2f, %.2f] s.', opts.at, t0, t1);
    end
    tk = opts.at;                       % 單張定格
else
    dt = opts.speed / opts.fps;
    tk = t0:dt:t1;
    if numel(tk) < 2
        error('mgs:viz6dof:tooShort', 'need at least two frames');
    end
end

% ── figure ───────────────────────────────────────────────────────────────
vis = "on"; if ~opts.show, vis = "off"; end
S = palette(opts.style);
simple = (opts.style == "simple");
f = figure('Color', S.bg, 'Position', [60 60 1600 900], ...
           'Visible', vis, 'InvertHardcopy', 'off');
if simple
    tl = tiledlayout(f, 12, 1, 'TileSpacing', 'none', 'Padding', 'compact');
    ax3 = nexttile(tl, 1, [11 1]);  hold(ax3, 'on');
    axP = gobjects(1, 0);
    axL = nexttile(tl, 12, [1 1]);  axis(axL, 'off');
else
    tl = tiledlayout(f, 4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax3 = nexttile(tl, 1, [3 2]);   hold(ax3, 'on');  grid(ax3, 'on');
    axP = gobjects(1, 3);
    for k = 1:3
        axP(k) = nexttile(tl, 3*k);  hold(axP(k), 'on');  grid(axP(k), 'on');
    end
    axL = nexttile(tl, 10, [1 2]);  axis(axL, 'off');
end
style_axes([ax3 axP], S);

% ── static scene ─────────────────────────────────────────────────────────
lim = scene_limits(A);
% ⚠️ 機身要在 ledger **之前**建好:CAD 那一行的來源說明由 make_body 產生,
%    先畫帳再建機身的話那一行永遠是空的(而空白看起來就像「沒有但書」)。
[L, A.model_note] = make_body(opts, scene_scale(lim));
draw_ledger(axL, A, S, simple);
draw_ground(ax3, lim, A, S, simple);
for i = 1:numel(A.veh)
    % 完整航跡當底圖。simple 樣式畫實線(那是主角),dash 樣式畫淡的。
    if simple
        plot3(ax3, A.veh(i).e, A.veh(i).n, A.veh(i).z, '-', ...
              'Color', A.veh(i).col, 'LineWidth', 1.4);
    else
        plot3(ax3, A.veh(i).e, A.veh(i).n, A.veh(i).z, '-', ...
              'Color', [A.veh(i).col 0.22], 'LineWidth', 0.75);
    end
end
axis(ax3, 'equal');
xlim(ax3, lim.e); ylim(ax3, lim.n); zlim(ax3, lim.z);
view(ax3, opts.view);
if simple
    axis(ax3, 'off');
else
    xlabel(ax3, 'East (m)'); ylabel(ax3, 'North (m)'); zlabel(ax3, 'Up (m)');
end
title(ax3, sprintf('%s  --  6-DOF pose', A.title), 'Color', S.fg, ...
      'FontWeight', 'bold', 'Interpreter', 'none');
if ~simple, draw_panels(axP, A); end
% CAD 網格沒有打光就是一團色塊,看不出姿態。
if opts.model ~= "glyph"
    camlight(ax3, 'headlight'); material(ax3, 'dull'); lighting(ax3, 'gouraud');
end

% ── animated handles ─────────────────────────────────────────────────────
G = struct([]);
for i = 1:numel(A.veh)
    c = A.veh(i).col;
    G(i).body  = patch(ax3, 'Vertices', zeros(1,3), 'Faces', 1, ...
                       'FaceColor', c, 'EdgeColor', 'none', 'FaceAlpha', 0.95);
    G(i).trail = plot3(ax3, nan, nan, nan, '-', 'Color', c, 'LineWidth', 2.0);
    G(i).los   = plot3(ax3, nan, nan, nan, ':', 'Color', [c 0.5], 'LineWidth', 1.0);
    G(i).drop  = plot3(ax3, nan, nan, nan, '-', 'Color', [S.fg 0.14], 'LineWidth', 0.6);
    for k = 1:3
        G(i).ax(k) = plot3(ax3, nan, nan, nan, '-', 'LineWidth', 1.6, ...
                           'Color', axis_color(k), 'Visible', tf(~simple));
    end
    % ⭐ 速度向量。⚠️ 這是**量測/積分出來的速度方向**,不是指令方向;
    %    兩者在俯衝段差很多(那正是 γ 追蹤缺口),不要當成同一件事。
    G(i).vv = plot3(ax3, nan, nan, nan, '-', 'LineWidth', 2.2, ...
                    'Color', S.vvec, 'Visible', tf(opts.vvec));
    G(i).vh = plot3(ax3, nan, nan, nan, '-', 'LineWidth', 2.2, ...
                    'Color', S.vvec, 'Visible', tf(opts.vvec));
    % simple 樣式沒有時序面板 => 沒有游標可畫。gobjects(1,0) 讓後面的
    % set([... G(i).mark]) 仍然合法(空陣列串接不會爆)。
    G(i).mark = gobjects(1, numel(axP));
    for k = 1:numel(axP)
        G(i).mark(k) = plot(axP(k), nan, nan, 'o', 'MarkerSize', 5, ...
                            'MarkerFaceColor', c, 'MarkerEdgeColor', S.fg);
    end
end
hud = text(ax3, 0.015, 0.985, '', 'Units', 'normalized', 'Color', S.fg, ...
           'FontName', 'Consolas', 'FontSize', 10, ...
           'VerticalAlignment', 'top', 'Interpreter', 'none', ...
           'Visible', tf(~simple));

% ── writer ───────────────────────────────────────────────────────────────
W = open_writer(opts, A);
cleanup = onCleanup(@() close_writer(W));

% ── animate ──────────────────────────────────────────────────────────────
for q = 1:numel(tk)
    tnow = tk(q);
    for i = 1:numel(A.veh)
        s = sample(A.veh(i), tnow);
        if isempty(s)
            set([G(i).body G(i).trail G(i).los G(i).drop G(i).ax G(i).mark ...
                 G(i).vv G(i).vh], 'Visible', 'off');
            continue;
        end
        set([G(i).body G(i).trail G(i).los G(i).drop], 'Visible', 'on');
        set(G(i).ax, 'Visible', tf(~simple));
        set([G(i).vv G(i).vh], 'Visible', tf(opts.vvec));
        R = dcm(s.phi, s.th, s.psi);
        Vx = (R * L.v')';                       % body -> NED
        P  = [Vx(:,2) + s.e, Vx(:,1) + s.n, -Vx(:,3) + s.z];
        set(G(i).body, 'Vertices', P, 'Faces', L.f);
        I3 = eye(3);
        for k = 1:3
            a = R * (L.len * I3(:, k));
            set(G(i).ax(k), 'XData', [s.e s.e + a(2)], ...
                            'YData', [s.n s.n + a(1)], ...
                            'ZData', [s.z s.z - a(3)]);
        end
        m = A.veh(i).t <= tnow & A.veh(i).t >= tnow - opts.trail_s;
        set(G(i).trail, 'XData', A.veh(i).e(m), 'YData', A.veh(i).n(m), ...
                        'ZData', A.veh(i).z(m));
        set(G(i).los,  'XData', [s.e 0], 'YData', [s.n 0], 'ZData', [s.z 0]);
        set(G(i).drop, 'XData', [s.e s.e], 'YData', [s.n s.n], 'ZData', [s.z lim.z(1)]);
        if opts.vvec
            % 速度方向 = (γ, ψ) 決定的單位向量,長度取機身的 2.5 倍
            Lv = 2.5 * L.len;
            ux = cos(deg2rad(s.gam_deg)) * cos(s.psi);
            uy = cos(deg2rad(s.gam_deg)) * sin(s.psi);
            uz = sin(deg2rad(s.gam_deg));
            set(G(i).vv, 'XData', [s.e s.e + Lv*uy], ...
                         'YData', [s.n s.n + Lv*ux], ...
                         'ZData', [s.z s.z + Lv*uz]);
            % 箭頭:在末端再畫一小段回頭的線段,避免相依 quiver3 的動態更新
            set(G(i).vh, 'XData', s.e + Lv*[0.78 1 0.78]*uy - 0.10*Lv*[1 0 -1]*ux, ...
                         'YData', s.n + Lv*[0.78 1 0.78]*ux + 0.10*Lv*[1 0 -1]*uy, ...
                         'ZData', s.z + Lv*[0.78 1 0.78]*uz);
        end
        if ~simple
            for k = 1:3
                set(G(i).mark(k), 'XData', tnow, 'YData', s.panel(k), 'Visible', 'on');
            end
        end
    end
    set(hud, 'String', hud_text(A, tnow));
    drawnow limitrate;
    grab(W, f);
end

A.fig = f;
end

% =============================================================================
%  SOURCES
% =============================================================================
function A = load_source(src, tgt)
if isstruct(src) && isfield(src, 'veh') && isfield(src.veh, 'traj')
    A = from_sim(src);
elseif isstruct(src) && isfield(src, 'veh') && isfield(src.veh, 'tbl')
    A = from_csv_struct(src, tgt);
elseif (ischar(src) || isstring(src))
    A = from_csv_struct(mgs.parseTotCsv(char(src)), tgt);
    [~, b, e] = fileparts(char(src));
    A.title = [b e];
else
    error('mgs:viz6dof:src', ...
        ['src must be the output of mgs.simSwarm, the output of ' ...
         'mgs.parseTotCsv, or a path to a tot_*.csv.']);
end
end

% -----------------------------------------------------------------------------
function A = from_sim(out)
%FROM_SIM  Every DOF is a model state -- nothing is reconstructed here.
A = base_struct();
A.title  = 'mgs.simSwarm';
A.source = 'model';
A.prov = { ...
 'n, e, z',  'DYN',  'translational EOM, CD0 = 0.01500 (1617 flight samples)'; ...
 'theta',    'DYN',  'first-order angle loop, tconst = fc.PTCH2SRV_TCONST'; ...
 'phi',      'DYN',  'first-order angle loop, tconst = fc.RLL2SRV_TCONST'; ...
 'psi',      'KIN',  'coordinated turn; not an independent state'; ...
 'p, q, r',  'NONE', 'no body rates / moment equations / inertia tensor'; ...
 'beta',     'NONE', 'no sideslip -- bank-to-turn by assumption'};
for i = 1:numel(out.veh)
    T = out.veh(i).traj;
    A.veh(i).id  = i;
    A.veh(i).tag = out.veh(i).tag;
    A.veh(i).t   = T.t;
    A.veh(i).n   = T.n;   A.veh(i).e = T.e;   A.veh(i).z = T.z;
    A.veh(i).phi = deg2rad(T.phi_deg);
    A.veh(i).th  = deg2rad(T.th_deg);
    A.veh(i).psi = deg2rad(T.psi_deg);
    A.veh(i).gam_deg = T.gam_deg;
    A.veh(i).nz  = T.nz;
    A.veh(i).nz_raw = T.nz_raw;
    A.veh(i).V   = T.V;
    A.veh(i).gs  = T.V .* cos(deg2rad(T.gam_deg));
    A.veh(i).col = vehicle_color(i);
end
end

% -----------------------------------------------------------------------------
function A = from_csv_struct(C, tgt_ll)
%FROM_CSV_STRUCT  A real flown sortie.  ⚠️ phi is NOT logged before 2026-08-22.
A = base_struct();
A.title  = 'flown sortie';
A.source = 'flight';
E = 111320.0;                          % geodesy.py:4 -- the node's own constant
g = 9.80665;

tgt = target_of(C, tgt_ll);
have_roll = false;

for k = 1:numel(C.veh)
    T = C.veh(k).tbl;
    t   = numc(T.t);
    lat = numc(T.lat);  lon = numc(T.lon);
    agl = numc(T.agl);  gs = numc(T.gs);
    gam = deg2rad(numc(T.gam_deg));
    [t, o] = sort(t);
    % ⚠️ 節點的 t 欄有**重複值**(20 Hz 記錄但時戳有捨入)⇒ interp1 會直接
    %    報「Sample points must be unique」。取每個時戳的**最後一筆**,
    %    而不是平均:平均會把兩個不同拍的姿態混成一個不存在的姿態。
    [t, iu] = unique(t, 'last');
    o = o(iu);
    lat = lat(o); lon = lon(o); agl = agl(o); gs = gs(o); gam = gam(o);

    n = (lat - tgt.lat) * E;
    e = (lon - tgt.lon) * E .* cos(deg2rad(lat));
    z = agl;

    % ψ [KIN]: track angle from the position history.  ⚠️ NOT a logged heading.
    psi = unwrap(atan2(gradient(e), gradient(n)));
    V   = gs ./ max(cos(gam), 1e-6);

    % θ [KIN]: pitch = flight-path + angle of attack.  alpha_est_deg IS logged.
    if ismember('alpha_est_deg', T.Properties.VariableNames)
        al = deg2rad(numc(T.alpha_est_deg)); al = al(o);
        al(~isfinite(al)) = 0;
    else
        al = zeros(size(gam));
    end
    th = gam + al;

    % φ: logged only on runs made after the 2-lite instrument landed.
    if ismember('roll_out_deg', T.Properties.VariableNames)
        ph = deg2rad(numc(T.roll_out_deg)); ph = ph(o);
        if any(isfinite(ph)), have_roll = true; end
        ph(~isfinite(ph)) = 0;
    else
        % [KIN] coordinated turn: tan(phi) = V*psidot / (g*cos gam)
        % ⚠️⚠️ psi 本身是**從位置微分**來的,再微分一次就是二階差分 ——
        %    20 Hz 的 lat/lon 量化雜訊會被放大到不可信(未平滑時巡航段
        %    反解出 -78.8 deg,實際巡航幾乎是平飛)。先把 psi 平滑到
        %    ~1 s 再微分,結果再平滑一次,最後夾到韌體的 r_lim 80 deg
        %    並**回報夾到的比例** —— 夾了不講就變成一條看起來很乾淨的假曲線。
        dt_med = median(diff(t), 'omitnan');
        k1 = max(3, round(1.0 / max(dt_med, 1e-3)));
        psis = smooth_k(unwrap(psi), k1);
        psid = gradient(psis) ./ max(gradient(t), 1e-3);
        ph = atan2(V .* psid, g * max(cos(gam), 0.25));
        ph = smooth_k(ph, k1);
        rlim = deg2rad(80);
        sat = mean(abs(ph) > rlim, 'omitnan');
        ph = max(-rlim, min(rlim, ph));
        % ⚠️ 健全性檢查**不在這裡做**。這一段算的是整趟(含 ~60 km 巡航),
        %    而畫出來的通常只有俯衝段 —— 巡航坡度小會把俯衝段的雜訊稀釋掉,
        %    在整趟上判「合理」然後畫一段其實不合理的。判斷移到
        %    validate_phi(),在 focus 裁切**之後**、對真正要畫的那一段做。
        if sat > 0.005, phi_sat = sat; end
    end

    A.veh(k).id  = C.veh(k).id;
    A.veh(k).tag = sprintf('uav%d', C.veh(k).id);
    A.veh(k).t = t; A.veh(k).n = n; A.veh(k).e = e; A.veh(k).z = z;
    A.veh(k).phi = ph; A.veh(k).th = th; A.veh(k).psi = psi;
    A.veh(k).gam_deg = rad2deg(gam);
    A.veh(k).nz     = col_or_nan(T, 'cmd_nz', o);
    A.veh(k).nz_raw = col_or_nan(T, 'req_nz', o);
    A.veh(k).V = V; A.veh(k).gs = gs;
    A.veh(k).col = vehicle_color(k);
    md = T.mode; if ~iscell(md), md = cellstr(md); end
    A.dive{k} = strcmpi(md(o), 'dive');
end

if have_roll
    phi_row = {'phi', 'CMD', 'roll_out_deg -- the command AFTER the r_lim clamp, not measured attitude'};
    A.phi_kind = 'cmd';
elseif exist('phi_sat', 'var')
    phi_row = {'phi', 'KIN', sprintf(['tan(phi)=V psidot/(g cos gam), 1 s smoothed; ' ...
        '%.1f%% of samples hit the 80 deg limit and were clipped -- treat phi as indicative only'], ...
        100*phi_sat)};
else
    phi_row = {'phi', 'KIN', 'tan(phi)=V psidot/(g cos gam), 1 s smoothed -- roll was never logged on this run'};
end
if ~isfield(A, 'phi_kind') || isempty(A.phi_kind), A.phi_kind = 'kin'; end
A.prov = [ ...
 {'n, e, z',  'MEAS', 'lat/lon/agl from the node''s own per-tick log'}; ...
 {'theta',    'KIN',  'theta = gam_deg + alpha_est_deg (both logged)'}; ...
 phi_row; ...
 {'psi',      'KIN',  'track angle from the position history'}; ...
 {'p, q, r',  'NONE', 'never logged; MAVROS never sees the direct-link command'}; ...
 {'beta',     'NONE', 'not observable from this log'}];
end

% -----------------------------------------------------------------------------
function A = apply_focus(A, focus)
%APPLY_FOCUS  Trim every vehicle to a time window.
%   ⚠️ Trimming is stated in the title, never silent: a picture of the last
%      45 s of a 608 s sortie that is captioned as the sortie is a lie by
%      omission, and this project has paid for that shape before.
if isstring(focus) || ischar(focus)
    switch lower(char(focus))
        case 'all', return;
        case 'dive'
            if ~strcmp(A.source, 'flight'), return; end   % 模型本來就只有俯衝
            w = dive_window(A);
            if isempty(w), return; end
        otherwise
            error('mgs:viz6dof:focus', 'focus must be "all", "dive", or [t0 t1]');
    end
else
    w = double(focus(:))';
    if numel(w) ~= 2, error('mgs:viz6dof:focus', 'focus = [t0 t1]'); end
end
keep = false(1, numel(A.veh));
for i = 1:numel(A.veh)
    m = A.veh(i).t >= w(1) & A.veh(i).t <= w(2);
    if nnz(m) < 2, continue; end
    keep(i) = true;
    for f = ["t","n","e","z","phi","th","psi","gam_deg","nz","nz_raw","V","gs"]
        A.veh(i).(f) = A.veh(i).(f)(m);
    end
end
% ⚠️⚠️ A vehicle with <2 samples in the window is dropped.  Until 2026-08-23
%    that happened SILENTLY -- and silently is exactly what the comment at the
%    top of this function forbids.  The case is not hypothetical: SB3's uav1
%    never entered "dive" (it fell out of runup, flew runin at 120-430 m AGL,
%    and ended sitting in the sea 28.5 km short), so focus="dive" rendered a
%    two-ship sortie as a clean one-ship engagement with nothing to say it had
%    dropped the ship that failed.  That is the picture you would put in a
%    report, and it would be the opposite of what happened.
dropped = A.veh(~keep);
% ⚠️ arrayfun over an EMPTY struct array returns [] (double), not an empty
%    string array -- callers doing cellstr(A.dropped) then error out on the
%    normal, nothing-was-dropped path.  Seed the right type first.
A.dropped = strings(1, 0);
if ~isempty(dropped)
    A.dropped = arrayfun(@(v) string(v.tag), dropped);
end
if ~isempty(dropped)
    tags = strjoin(A.dropped, ', ');
    warning('mgs:viz6dof:vehicleDropped', ...
        ['focus [%g %g] keeps no samples for %s -- that vehicle is NOT in ' ...
         'this figure. It is named in the title and the ledger.'], ...
        w(1), w(2), tags);
end
A.veh = A.veh(keep);
if isempty(A.veh)
    error('mgs:viz6dof:focus', 'the focus window [%g %g] keeps no samples', w);
end
A = validate_phi(A);
A.title = sprintf('%s   [t %.1f-%.1f s]', A.title, w(1), w(2));
if ~isempty(dropped)
    A.title = sprintf('%s   (%s NOT SHOWN)', A.title, ...
        strjoin(arrayfun(@(v) string(v.tag), dropped), '/'));
end
A.prov(end+1, :) = {'window', 'CUT', sprintf(['this figure shows t = %.1f-%.1f s only ' ...
                    '(focus), not the whole sortie'], w(1), w(2))};
if ~isempty(dropped)
    A.prov(end+1, :) = {'dropped', 'CUT', sprintf(['%s had <2 samples in the window ' ...
        'and is absent from this figure -- it is NOT a one-ship sortie'], ...
        strjoin(arrayfun(@(v) string(v.tag), dropped), ', '))};
end
end

function A = validate_phi(A)
%VALIDATE_PHI  ⚠️⚠️ A reconstruction that judges itself, on the window that
%   will actually be drawn.
%
%   The node's own commanded |roll| has a median of 0.93 deg across 32
%   sorties.  If differentiating a differentiated heading gives tens of
%   degrees instead, that is quantisation noise amplified twice, not bank.
%   Drawing it anyway is worse than drawing nothing: a smooth wrong attitude
%   curve reads as data, and nobody re-derives it.
if ~strcmp(A.source, 'flight') || ~strcmp(A.phi_kind, 'kin'), return; end
v = [];
for i = 1:numel(A.veh), v = [v; abs(A.veh(i).phi(:))]; end %#ok<AGROW>
v = v(isfinite(v));
if isempty(v), return; end
med = median(v);  sat = mean(v >= deg2rad(80) - 1e-9);
if med <= deg2rad(45) && sat <= 0.20, return; end
for i = 1:numel(A.veh), A.veh(i).phi = zeros(size(A.veh(i).phi)); end
A.phi_kind = 'none';
r = find(strcmp(A.prov(:,1), 'phi'), 1);
% ⚠️ 來源字串一律用**純底線**:draw_ledger 會統一做 tex 跳脫。
%    在這裡先手動加反斜線會被跳脫兩次,tex 讀成換行 + 下標
%    (實測 roll_out_deg 被印成 'roll\out\deg' 的亂碼)。
msg = [sprintf(['reconstruction REJECTED on this window: median %.0f deg, ' ...
    '%.0f%% at the 80 deg limit, against a node-commanded median of 0.93 deg ' ...
    'over 32 sorties. Drawn as zero -- roll is simply not observable in this ' ...
    'log. Runs after 2026-08-22 carry '], rad2deg(med), 100*sat), ...
    'roll_out_deg.'];
if isempty(r), A.prov(end+1,:) = {'phi', 'NONE', msg};
else,          A.prov(r,:)     = {'phi', 'NONE', msg}; end
end

function w = dive_window(A)
w = [];
if ~isfield(A, 'dive'), return; end
a = inf; b = -inf;
for i = 1:numel(A.dive)
    d = A.dive{i};
    if any(d), a = min(a, A.veh(i).t(find(d,1))); b = max(b, A.veh(i).t(find(d,1,'last'))); end
end
if isfinite(a) && isfinite(b) && b > a, w = [a - 5, b + 2]; end
end

% -----------------------------------------------------------------------------
function A = base_struct()
A = struct('title', '', 'source', '', 'prov', {{}}, 'fig', [], 'dive', {{}}, ...
           'phi_kind', '', ...
           ... % ⚠️ tags of vehicles the focus window excluded.  Empty is the
           ... % normal case; non-empty means the figure is NOT the whole sortie
           ... % and the caller must not describe it as one.
           'dropped', {{}}, ...
           'veh', struct('id', {}, 'tag', {}, 't', {}, 'n', {}, 'e', {}, ...
                         'z', {}, 'phi', {}, 'th', {}, 'psi', {}, ...
                         'gam_deg', {}, 'nz', {}, 'nz_raw', {}, 'V', {}, ...
                         'gs', {}, 'col', {}));
end

% -----------------------------------------------------------------------------
function tgt = target_of(C, tgt_ll)
%TARGET_OF  ⚠️⚠️ Refuse to guess.  Every plotted position is relative to this;
%   a wrong datum shifts the whole engagement and the picture still looks fine.
if all(isfinite(tgt_ll))
    tgt = struct('lat', tgt_ll(1), 'lon', tgt_ll(2)); return;
end
if isfield(C, 'tgt') && isstruct(C.tgt) && isfield(C.tgt, 'lat')
    tgt = C.tgt; return;
end
% 同目錄的 <base>_geom.csv,節點自己寫的(tot_node._write_geom_csv)
if isfield(C, 'path') && ~isempty(C.path)
    [d, b, ~] = fileparts(char(C.path));
    g = fullfile(d, [b '_geom.csv']);
    if isfile(g)
        G = readtable(g, 'TextType', 'string');
        r = find(strcmpi(string(G.kind), "target"), 1);
        if ~isempty(r)
            tgt = struct('lat', double(G.lat(r)), 'lon', double(G.lon(r)));
            return;
        end
    end
end
error('mgs:viz6dof:noTarget', ...
    ['no target position: %s has no sibling *_geom.csv with a "target" row, ' ...
     'and this function will not invent one -- every plotted position is ' ...
     'relative to it, and a wrong datum still draws a perfectly plausible ' ...
     'picture.  Either point at the artifacts copy (which keeps its _geom.csv) ' ...
     'or pass tgt=[lat lon].'], char(getfielddef(C, 'path', '<struct>')));
end

function v = getfielddef(s, f, d)
if isstruct(s) && isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end

% =============================================================================
%  SCENE
% =============================================================================
function lim = scene_limits(A)
n = []; e = []; z = [];
for i = 1:numel(A.veh)
    n = [n; A.veh(i).n(:)]; e = [e; A.veh(i).e(:)]; z = [z; A.veh(i).z(:)]; %#ok<AGROW>
end
pad = @(v, f) [min(v) - f*range_or(v), max(v) + f*range_or(v)];
lim.n = pad(n, 0.08); lim.e = pad(e, 0.08);
lim.z = [min(0, min(z)), max(z)*1.05 + 1];
end

function r = range_or(v)
r = max(v) - min(v); if r < 1, r = 1; end
end

function s = scene_scale(lim)
% ⚠️ 圖示是**示意**不是比例模型:真機約 3 m,在 7 km 的場景裡畫成 3 m
%    等於看不見。取場景尺度的 0.7%(7 km -> 約 50 m),大到看得出姿態、
%    小到不會遮住紅心。報告要別的比例就用 glyph= 覆蓋。
s = 0.007 * max([diff(lim.n), diff(lim.e)]);
s = max(s, 6);
end

function draw_ground(ax, lim, A, S, simple)
%DRAW_GROUND  Sea plane at the TARGET DATUM.  ⚠️ z = 0 IS the bullseye plane:
%   the red heart is at sea level, 0 m AMSL (memory sea-level-target-datum).
[X, Y] = meshgrid(linspace(lim.e(1), lim.e(2), 2), linspace(lim.n(1), lim.n(2), 2));
surf(ax, X, Y, zeros(size(X)), 'FaceColor', S.sea, ...
     'EdgeColor', S.seaEdge, 'FaceAlpha', 0.55);
th = linspace(0, 2*pi, 180);
for r = [10 50 200]
    plot3(ax, r*cos(th), r*sin(th), zeros(size(th)), '-', ...
          'Color', [S.ring 0.85], 'LineWidth', 1.2);
end
plot3(ax, 0, 0, 0, 'p', 'MarkerSize', 14, 'MarkerFaceColor', S.ring, ...
      'MarkerEdgeColor', S.fg);
if ~simple
    text(ax, 0, 0, 0.02*diff(lim.z), '  target (0 m AMSL)', 'Color', S.ring, ...
         'FontSize', 9);
end
for i = 1:numel(A.veh)
    plot3(ax, A.veh(i).e(1), A.veh(i).n(1), A.veh(i).z(1), '^', ...
          'MarkerSize', 7, 'MarkerFaceColor', A.veh(i).col, 'MarkerEdgeColor', S.fg);
end
end

function draw_panels(axP, A)
for i = 1:numel(A.veh)
    v = A.veh(i);
    plot(axP(1), v.t, v.gam_deg, '-', 'Color', v.col, 'LineWidth', 1.1);
    plot(axP(1), v.t, rad2deg(v.th), '--', 'Color', [v.col 0.55], 'LineWidth', 0.9);
    if any(isfinite(v.nz_raw))
        plot(axP(2), v.t, v.nz_raw, '--', 'Color', [v.col 0.5], 'LineWidth', 0.9);
    end
    if any(isfinite(v.nz))
        plot(axP(2), v.t, v.nz, '-', 'Color', v.col, 'LineWidth', 1.1);
    end
    plot(axP(3), v.t, v.gs, '-',  'Color', v.col, 'LineWidth', 1.1);
    plot(axP(3), v.t, v.V,  '--', 'Color', [v.col 0.55], 'LineWidth', 0.9);
end
title(axP(1), '\gamma (solid) / \theta (dashed)   [deg]', 'Color', 'w');
title(axP(2), 'n_z   req (dashed) / cmd (solid)   [g]', 'Color', 'w');
title(axP(3), 'speed   ground (solid) / total (dashed)   [m/s]', 'Color', 'w');
yline(axP(2), -1, ':', 'clamp', 'Color', [1 0.6 0.3], 'LabelHorizontalAlignment', 'left');
yline(axP(2),  3, ':', '',       'Color', [1 0.6 0.3]);
% ⚠️ 末端 cmd_nz 由 γ 指令反解,凍結前會噴到 ±20 g 以上(BASE2 實測最低
%    −20.96 g)。讓 autoscale 跟著它跑會把整段有訊息的區間壓成一條線。
%    用穩健分位數定範圍,並**明講被裁掉多少**,不要靜靜地切掉。
clip_to_robust(axP(2), A, 'nz', [-1 3]);
% ⚠️ Both acceptance speeds are GROUND speeds (memory eas-vs-groundspeed-157,
%    terminal-204-is-groundspeed).  Drawing them against total speed would
%    silently flatter the result by ~10-25 m/s.
yline(axP(3), 157, ':', 'cruise 157 (ground)', 'Color', [0.5 1 0.6], ...
      'LabelHorizontalAlignment', 'left');
yline(axP(3), 204, ':', 'terminal 204 (ground)', 'Color', [1 0.9 0.4], ...
      'LabelHorizontalAlignment', 'left');
xlabel(axP(3), 'node time (s)', 'Color', 'w');
% ⚠️ n_z 只在俯衝段有值 ⇒ autoscale 會給它一個和另外兩格不同的 x 範圍,
%    讀圖的人會以為那是同一段時間。三格一律共用 x。
t0 = min(arrayfun(@(v) v.t(1), A.veh));  t1 = max(arrayfun(@(v) v.t(end), A.veh));
for k = 1:3, xlim(axP(k), [t0 t1]); end
linkaxes(axP, 'x');
end

function clip_to_robust(ax, A, fld, keep)
v = [];
for i = 1:numel(A.veh)
    x = A.veh(i).(fld); v = [v; x(isfinite(x))]; %#ok<AGROW>
end
if numel(v) < 10, return; end
lo = min(prctile(v, 1), keep(1) - 0.5);
hi = max(prctile(v, 99), keep(2) + 0.5);
if hi <= lo, return; end
ylim(ax, [lo hi]);
out = mean(v < lo | v > hi);
if out > 0
    text(ax, 0.99, 0.03, sprintf('%.1f%% of samples outside this range', 100*out), ...
         'Units', 'normalized', 'HorizontalAlignment', 'right', ...
         'Color', [1 0.75 0.4], 'FontSize', 7.5);
end
end

function draw_ledger(ax, A, S, simple)
if simple
    % ⚠️ 即使是主視覺也**必須**帶但書。一張沒有但書的 6-DOF 圖進了報告,
    %    第一個問題就會問「所以是六自由度剛體嗎」,而答案是不是。
    one = ['6 DOF of POSE (3-DOF point mass + ArduPilot angle loop; no body ' ...
           'rates, no moment equations) -- 5-DOF in the standard taxonomy.'];
    if isfield(A, 'model_note') && ~isempty(A.model_note)
        one = [one '   Airframe drawn: ' A.model_note];
    end
    w = wrap_at(strrep(one, '_', '\_'), 175);
    text(ax, 0.5, 1.0, w, 'Units', 'normalized', 'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'top', 'Color', S.fg, 'FontName', 'Consolas', ...
         'FontSize', 7.5, 'Interpreter', 'tex');
    return;
end
txt = { ...
 sprintf('\\bfDEGREE-OF-FREEDOM LEDGER\\rm   (source: %s)', A.source), ...
 '[DYN] integrated by the model   [MEAS] logged telemetry   [KIN] algebraic from logged quantities   [CMD] asked for, not achieved   [NONE] not modelled'};
for r = 1:size(A.prov, 1)
    % ⚠️ tex 直譯器會把 `PTCH2SRV_TCONST` 的底線讀成下標 ⇒ 參數名在報告裡
    %    印錯。逐字跳脫,不要為了省事改用 none(那樣就沒有粗體標題了)。
    body = strrep(A.prov{r,3}, '_', '\_');
    % ⚠️ 一定要折行:被否決的重建那一行有百來字,不折就會被圖框切掉,
    %    而**切掉的正好是「為什麼不可信」那半句** —— 只剩前半看起來像
    %    它還是可信的。
    w = wrap_at(body, 132);
    txt{end+1} = sprintf('  %-10s [%s]  %s', A.prov{r,1}, A.prov{r,2}, w{1}); %#ok<AGROW>
    for q = 2:numel(w)
        txt{end+1} = sprintf('  %-10s      %s', '', w{q}); %#ok<AGROW>
    end
end
if isfield(A, 'model_note') && ~isempty(A.model_note)
    wm = wrap_at(strrep(A.model_note, '_', '\_'), 132);
    for q = 1:numel(wm)
        if q == 1, lbl = 'airframe'; else, lbl = ''; end
        txt{end+1} = sprintf('  %-10s %-5s %s', lbl, tern(q==1,'[CAD]',''), wm{q}); %#ok<AGROW>
    end
end
txt{end+1} = ['\bf⚠\rm  6 DOF of POSE, not a 6-DOF rigid-body simulation: no body rates, ' ...
              'no moment equations, no sideslip.  In the standard taxonomy this is a 5-DOF model.'];
text(ax, 0, 1, txt, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
     'Color', S.fg, 'FontName', 'Consolas', 'FontSize', 8.5, ...
     'Interpreter', 'tex');
end

% =============================================================================
%  GEOMETRY
% =============================================================================
function R = dcm(phi, th, psi)
%DCM  Body -> NED, 3-2-1 (yaw-pitch-roll).  Body x fwd, y right, z down.
cp = cos(phi); sp = sin(phi); ct = cos(th); st = sin(th);
cs = cos(psi); ss = sin(psi);
R = [ ct*cs, sp*st*cs - cp*ss, cp*st*cs + sp*ss; ...
      ct*ss, sp*st*ss + cp*cs, cp*st*ss - sp*cs; ...
        -st,            sp*ct,            cp*ct ];
end

function [L, note] = make_body(opts, s)
%MAKE_BODY  Pick the airframe drawn at each vehicle.
%   ⚠️ 兩者都是**示意尺寸**:真機 3.786 m 在 7 km 的場景裡等於看不見,所以
%      一律縮放到場景尺度。CAD 版本保留真實**比例**,但不是真實大小。
persistent CACHE
note = '';
if opts.model == "glyph"
    L = airframe_glyph(opts.glyph * s);
    return;
end
if opts.model == "cad"
    g = fullfile(mgs.repoRoot(), 'ui', 'resources', 'barracuda500.glb');
else
    g = char(opts.model);
end
if isempty(CACHE) || ~strcmp(CACHE.path, g)
    CACHE = mgs.loadGlb(g, 'faces', 1500, 'frame', "body");
end
M = CACHE;
% 等比放大到與內建示意體同一個「視覺長度」,姿態才看得出來
want_len = 6.0 * opts.glyph * s;                 % 內建示意體全長 ~6s
have_len = M.bbox.max(1) - M.bbox.min(1);
k = want_len / max(have_len, 1e-9);
L.v = M.v * k;
L.f = M.f;
L.len = 2.2 * opts.glyph * s;                    % 機體三軸的長度,與示意體一致
[~, b, e] = fileparts(g);
note = sprintf(['%s%s, %d tri (from %d), scaled x%.0f -- CAD geometry, NOT the ' ...
    'airframe that flew: every sortie used SIM_Plane -jet (m 22.0 kg, b 1.88 m, ' ...
    'S 0.45 m2) while this CAD spans 2.325 m and is 3.786 m long'], ...
    b, e, size(M.f,1), M.n_tri_in, k);
end

function L = airframe_glyph(s)
%AIRFRAME_GLYPH  Cruciform-wing body, body frame (x fwd, y right, z down).
L.len = 2.2*s;
nose = [ 3.0  0    0];  tail = [-1.6  0    0];
wr   = [ 0.2  1.8  0];  wl   = [ 0.2 -1.8  0];
wrb  = [-0.9  1.5  0];  wlb  = [-0.9 -1.5  0];
fin  = [-1.1  0   -1.2]; finb = [-1.7  0   -0.2];
hr   = [-1.3  0.9  0];  hl   = [-1.3 -0.9  0];
L.v = s * [nose; tail; wr; wl; wrb; wlb; fin; finb; hr; hl];
L.f = [1 3 5; 1 5 2; 1 4 6; 1 6 2; 1 7 8; 2 7 8; 2 9 1; 2 10 1];
end

function S = palette(style)
%PALETTE  ⚠️ 兩條驗收線與夾制線在兩種樣式下都必須清楚可見 —— 主視覺好看
%   不能以看不見門檻為代價。
if style == "simple"
    S = struct('bg', [0.94 0.94 0.94], 'ax', [0.94 0.94 0.94], ...
               'fg', [0.15 0.15 0.15], 'grid', [0.55 0.55 0.60], ...
               'sea', [0.80 0.86 0.94], 'seaEdge', [0.55 0.65 0.78], ...
               'vvec', [0.85 0.10 0.10], 'ring', [0.85 0.15 0.15]);
else
    S = struct('bg', [0.06 0.07 0.10], 'ax', [0.09 0.10 0.14], ...
               'fg', [0.90 0.93 0.97], 'grid', [0.45 0.50 0.60], ...
               'sea', [0.10 0.16 0.26], 'seaEdge', [0.20 0.30 0.45], ...
               'vvec', [1.00 0.35 0.30], 'ring', [0.95 0.35 0.35]);
end
end

function v = tern(c, a, b)
if c, v = a; else, v = b; end
end

function v = tf(b)
if b, v = 'on'; else, v = 'off'; end
end

function c = axis_color(k)
switch k
    case 1, c = [1.00 0.30 0.30];      % body x -- forward
    case 2, c = [0.35 1.00 0.45];      % body y -- right wing
    otherwise, c = [0.40 0.65 1.00];   % body z -- down
end
end

function c = vehicle_color(i)
P = [0.30 0.75 1.00; 1.00 0.72 0.25; 0.55 1.00 0.55; 1.00 0.50 0.85;
     0.70 0.70 1.00; 1.00 0.95 0.45; 0.45 0.95 0.90; 0.95 0.55 0.45];
c = P(mod(i-1, size(P,1)) + 1, :);
end

% =============================================================================
%  SAMPLING / HUD
% =============================================================================
function s = sample(v, tnow)
if tnow < v.t(1) || tnow > v.t(end), s = []; return; end
q = @(y) interp1(v.t, y, tnow, 'linear');
s.n = q(v.n); s.e = q(v.e); s.z = q(v.z);
s.phi = q(unwrap(v.phi)); s.th = q(v.th); s.psi = q(unwrap(v.psi));
s.panel = [q(v.gam_deg), nan_safe(v.t, v.nz, tnow), q(v.gs)];
s.gam_deg = q(v.gam_deg);
s.V = q(v.V); s.gs = q(v.gs);
s.R = hypot(hypot(s.n, s.e), s.z);
end

function y = nan_safe(t, x, tq)
m = isfinite(x);
if nnz(m) < 2, y = NaN; return; end
y = interp1(t(m), x(m), tq, 'linear', NaN);
end

function s = hud_text(A, tnow)
L = {sprintf('t = %7.2f s', tnow)};
for i = 1:numel(A.veh)
    p = sample(A.veh(i), tnow);
    if isempty(p)
        L{end+1} = sprintf('%-6s   --', A.veh(i).tag); %#ok<AGROW>
    else
        L{end+1} = sprintf('%-6s  AGL %7.1f m   R %8.1f m   gs %6.1f  V %6.1f m/s   phi %+6.1f  th %+6.1f  psi %+6.1f deg', ...
            A.veh(i).tag, p.z, p.R, p.gs, p.V, rad2deg(p.phi), ...
            rad2deg(p.th), rad2deg(wrapToPi_(p.psi))); %#ok<AGROW>
    end
end
s = strjoin(L, newline);
end

function a = wrapToPi_(a)
a = mod(a + pi, 2*pi) - pi;
end

% =============================================================================
%  EXPORT
% =============================================================================
function W = open_writer(opts, A)
W = struct('kind', char(opts.export), 'obj', [], 'file', '', 'first', true);
if opts.export == "none", return; end
file = char(opts.file);
if isempty(file)
    d = fullfile('runs', 'artifacts', 'viz');
    if ~exist(d, 'dir'), mkdir(d); end
    file = fullfile(d, sprintf('engage_%s.%s', matlab.lang.makeValidName(A.title), ...
                               char(opts.export)));
end
W.file = file;
[p, ~, ~] = fileparts(file);
if ~isempty(p) && ~exist(p, 'dir'), mkdir(p); end
switch char(opts.export)
    case 'png'
        % 無狀態:實際寫檔在 grab()。這裡只確保副檔名沒說謊。
        [~, ~, ext] = fileparts(file);
        if ~strcmpi(ext, '.png')
            error('mgs:viz6dof:pngExt', ...
                'export="png" but file="%s" does not end in .png', file);
        end
    case 'mp4'
        W.obj = VideoWriter(file, 'MPEG-4');
        W.obj.FrameRate = opts.fps; W.obj.Quality = 95;
        open(W.obj);
    case 'gif'
        W.delay = 1/opts.fps;
end
end

function grab(W, f)
switch W.kind
    % ⚠️⚠️ 2026-08-23:'png' 這一支**先前不存在**,而 open_writer 的
    %    mustBeMember 卻接受 "png" ⇒ 呼叫端拿到「成功」、磁碟上什麼都沒有,
    %    一行警告都沒有。RA1 的補圖就是這樣「成功」了兩次才被發現
    %    (函式回報 2 個檔,目錄裡 0 個)。
    %    這正是本專案最忌諱的形態:宣告支援、靜默不做。
    %    PNG 是單格輸出 ⇒ 每一格都覆寫,單張定格(at=)時剛好只寫一次;
    %    給了 at 以外的呼叫則留下最後一格,與 gif/mp4 的語意一致。
    case 'png'
        exportgraphics(f, W.file, 'Resolution', 150, ...
                       'BackgroundColor', get(f, 'Color'));
    case 'mp4'
        writeVideo(W.obj, getframe(f));
    case 'gif'
        [im, cm] = rgb2ind(frame2im(getframe(f)), 256);
        if W.first
            imwrite(im, cm, W.file, 'gif', 'LoopCount', inf, 'DelayTime', W.delay);
        else
            imwrite(im, cm, W.file, 'gif', 'WriteMode', 'append', 'DelayTime', W.delay);
        end
end
end

function close_writer(W)
if strcmp(W.kind, 'mp4') && ~isempty(W.obj)
    try, close(W.obj); catch, end
end
end

% =============================================================================
%  SMALL HELPERS
% =============================================================================
function style_axes(axs, S)
for a = axs
    set(a, 'Color', S.ax, 'XColor', S.fg, 'YColor', S.fg, 'ZColor', S.fg, ...
           'GridColor', S.grid, 'GridAlpha', 0.28, 'FontSize', 8.5);
end
end

function v = numc(cc)
if isnumeric(cc), v = double(cc); return; end
v = nan(numel(cc), 1);
for k = 1:numel(cc)
    d = str2double(strtrim(char(cc{k})));
    if ~isnan(d), v(k) = d; end
end
end

function y = col_or_nan(T, name, o)
if ismember(name, T.Properties.VariableNames)
    y = numc(T.(name)); y = y(o);
else
    y = nan(numel(o), 1);
end
end

function w = wrap_at(str, n)
%WRAP_AT  Greedy word wrap; never truncates.
w = {}; cur = '';
for tok = strsplit(str, ' ')
    t = tok{1};
    if isempty(cur), cand = t; else, cand = [cur ' ' t]; end
    if numel(cand) > n && ~isempty(cur)
        w{end+1} = cur; cur = t; %#ok<AGROW>
    else
        cur = cand;
    end
end
if ~isempty(cur), w{end+1} = cur; end
if isempty(w), w = {''}; end
end

function y = smooth_k(x, k)
if numel(x) < k, y = x; return; end
y = movmean(x, k, 'omitnan');
end
