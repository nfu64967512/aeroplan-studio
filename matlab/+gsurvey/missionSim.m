function out = missionSim(p)
%GSURVEY.MISSIONSIM  全任務:起飛 → AUTO 航線 → 決斷圈盤旋 → 閘門 → GUIDED 命中。
%
%   out = gsurvey.missionSim(gsurvey.missionParams('n', 8))
%
%   相位機**刻意與 tot_node 同名**,方便把模擬對回航次 log:
%
%     TAKEOFF   地面加速到 Vto 後離地            (飛控自己)
%     CLIMB     爬到 perch_alt(爬升角受限)      AUTO
%     TRANSIT   飛任務航點 → 決斷圈             AUTO
%     LOITER    在決斷圈上盤旋等人               AUTO(NAV_LOITER_UNLIM)
%     ── 全員到齊閘門 `_gate_all` ──
%     DIVE      交回 GUIDED,PPN + LOS 地板       GUIDED
%     DONE      觸地
%
%   ⚠️⚠️ 這是**運動學**模擬:轉彎受坡度上限、爬升受爬升角上限、速率固定。
%      它回答的是「這條航線的**幾何**成不成立」——不是「飛控飛不飛得出來」。
%      後者要 SITL。兩者問的是不同的問題,不要互相取代。

arguments
    p struct
end

n = p.n;
T = [0; 0; 0];                            % 紅心在原點、地面
dt = p.dt;

% ── 每機的扇形方位、決斷圈點、決斷圈高度 ──────────────────────────
if n <= 1, off = 0; else, off = linspace(-p.fanHalf, p.fanHalf, n); end
brg  = mod(p.approach + off, 2*pi);
ringR = p.ringR * ones(1, n);
if p.autoIpR > 0, ringR(:) = p.autoIpR; end
ipAlt = p.perchAlt;
if p.autoIpAlt > 0, ipAlt = p.autoIpAlt; end

IP = zeros(3, n);
for i = 1:n
    IP(:, i) = [ ringR(i)*cos(brg(i)); ringR(i)*sin(brg(i)); -ipAlt ];
end

% ── 起飛點:在進場方位的**遠端**,橫向排開 ────────────────────────
home = zeros(3, n);
hb = p.approach + p.homeBrgOff;
for i = 1:n
    lat = (i - (n+1)/2) * p.launchSep;
    home(:, i) = [ p.homeR*cos(hb) - lat*sin(hb);
                   p.homeR*sin(hb) + lat*cos(hb);
                   0 ];
end

% ── 狀態 ───────────────────────────────────────────────────────────
X     = home;                              % 位置
V     = zeros(1, n);                       % 速率
psi   = zeros(1, n);                       % 航向(從北,rad)
gam   = zeros(1, n);                       % 航跡角
phase = repmat("PREP", 1, n);
tRel  = (0:n-1) * p.stagger;               % 彈射時刻(驗收① 3 s 間隔)
wpi   = ones(1, n);
tIP   = nan(1, n);                         % 抵達決斷圈的時刻
tImp  = nan(1, n);
missH = nan(1, n);
gamF  = nan(1, n);
R0    = nan(1, n);                         % ⭐ 放行距離 —— 這一趟要看的頭號量
loitT = zeros(1, n);
dPrev = inf(1, n);      % 上一拍的水平距(抵環放行要看穿越方向)
frz   = nan(2, n);
log   = cell(1, n);
for i = 1:n, log{i} = zeros(0, 8); end

% 航點:每機 [進場點(選配) → 決斷圈]
WP = cell(1, n);
for i = 1:n
    w = [];
    if p.autoTransitR > 0
        w = [w, [p.autoTransitR*cos(brg(i)); p.autoTransitR*sin(brg(i)); -ipAlt]];
    end
    w = [w, IP(:, i)];
    WP{i} = w;
end

gateOpen = false; tGate = nan;
t = 0; K = ceil(p.tMax/dt);

for k = 1:K
    if all(phase == "DONE"), break; end

    % ── 全員到齊閘門 ────────────────────────────────────────────────
    % ⚠️⚠️ 用 `_gate_all` 的語意:分母是**還活著的**,而且全滅時不通過。
    %    寫成 `sum(arrived) >= n` 在有機體損失時**數學上不可能成立**
    %    —— 那正是 ALIGNW1 等 1551 s 的死結([[kill-deadlocks-the-sortie]])。
    if ~gateOpen
        live = phase ~= "DONE";
        arrived = ~isnan(tIP) & live;
        if any(live) && all(arrived(live))
            gateOpen = true; tGate = t;
            % ⚠️⚠️ 閘門開啟**不等於**放行。各機在盤旋圈上的相位是隨機的,
            %    立刻翻 DIVE 會讓 R0 = 環半徑 ± 盤旋半徑 ⇒ 實測全距 1987 m,
            %    而 R0 是殘餘高度的唯一自變數([[residual-is-release-range]])。
            %    `releaseAtRing` 打開時改成「等下一次穿越環半徑再放行」,
            %    這與生產碼的 `release_at_ring` 是同一個機制。
            if ~p.releaseAtRing
                for i = 1:n
                    if phase(i) == "LOITER"
                        phase(i) = "DIVE"; R0(i) = norm(T - X(:, i));
                    end
                end
            end
        end
    end

    for i = 1:n
        switch phase(i)

        % ── 等彈射 ────────────────────────────────────────────────
        case "PREP"
            if t >= tRel(i), phase(i) = "TAKEOFF"; psi(i) = hb + pi; end
            % 航向 hb+pi = 朝紅心(起飛點在遠端)

        % ── 地面加速 → 離地 ───────────────────────────────────────
        case "TAKEOFF"
            V(i) = V(i) + p.accel*dt;
            X(1:2, i) = X(1:2, i) + V(i)*dt*[cos(psi(i)); sin(psi(i))];
            if V(i) >= p.Vto, phase(i) = "CLIMB"; gam(i) = p.climbMax; end

        % ── 爬升到 perch_alt ──────────────────────────────────────
        case "CLIMB"
            V(i) = min(p.V0, V(i) + p.accel*dt);
            [psi(i), gam(i)] = i_steer(X(:, i), WP{i}(:, wpi(i)), psi(i), ...
                                       V(i), p, true);
            X(:, i) = i_move(X(:, i), V(i), psi(i), gam(i), dt);
            if -X(3, i) >= ipAlt - 20
                gam(i) = 0; X(3, i) = -ipAlt; phase(i) = "TRANSIT";
            end

        % ── AUTO 航線 ─────────────────────────────────────────────
        case "TRANSIT"
            % ⭐⭐ ring_pace 的**正確語意**:在進場段調速,讓全隊同時抵環,
            %    於是**根本不必盤旋等待**(tot_node 的 `t_ring = max_j(d_go_j)/vmax`)。
            % ⚠️ 分母必須是 vmax(能力上限)不是實測地速 —— 用實測會形成
            %    正回饋棘輪:慢的更慢。這一條在生產碼裡是明寫的。
            % ⚠️⚠️ 第一版把 ring_pace 誤實作成「在盤旋圈裡調速」,結果更糟
            %    (139.55 → 155.80 s)。原因是要全員同時穿越,速度比必須覆蓋
            %    弧長比,而 Vmax/Vmin = 1.58,弧長比接近整圈 ⇒ 權限差一個
            %    數量級。**盤旋一旦開始就補不回來了,必須在進場段就對齊。**
            if p.ringPace
                dgo = zeros(1, n);
                for j = 1:n
                    if phase(j) == "TRANSIT" || phase(j) == "CLIMB"
                        dgo(j) = norm(X(1:2, j) - IP(1:2, j));
                    end
                end
                tRing = max(dgo) / p.Vmax;
                V(i) = max(p.Vmin, min(p.Vmax, ...
                       norm(X(1:2,i) - IP(1:2,i)) / max(tRing, 1e-3)));
            else
                V(i) = min(p.V0, V(i) + p.accel*dt);
            end
            tgt = WP{i}(:, wpi(i));
            [psi(i), gam(i)] = i_steer(X(:, i), tgt, psi(i), V(i), p, false);
            X(:, i) = i_move(X(:, i), V(i), psi(i), gam(i), dt);
            if norm(X(1:2, i) - tgt(1:2)) < p.wpAccept
                if wpi(i) < size(WP{i}, 2)
                    wpi(i) = wpi(i) + 1;
                else
                    phase(i) = "LOITER";
                end
            end

        % ── 決斷圈盤旋(NAV_LOITER_UNLIM)──────────────────────────
        case "LOITER"
            % 半徑受坡度上限約束:飛不出比 turnRmin 更小的圈。
            % ⚠️ 指令半徑小於 turnRmin 時飛機會用**它做得到的**半徑繞,
            %    而我們會以為半徑是指令的那個 —— hold-ring-is-unflyable。
            Rl = max(p.autoLoiterR, p.turnRmin);
            [psi(i), gam(i)] = i_loiter(X(:, i), IP(:, i), Rl, psi(i), V(i), p);
            X(:, i) = i_move(X(:, i), V(i), psi(i), gam(i), dt);
            loitT(i) = loitT(i) + dt;
            % 抵達判定:離決斷圈圓心 < auto_ip_reach
            % ⚠️ 必須 > 盤旋半徑,否則**數學上永遠不成立**(死結)。
            if isnan(tIP(i)) && norm(X(1:2, i) - IP(1:2, i)) < p.autoIpReach
                tIP(i) = t;
            end
            % 抵環放行:閘門已開 + 本機正**向內**穿越環半徑
            if gateOpen && p.releaseAtRing
                dNow = norm(T(1:2) - X(1:2, i));
                if dNow <= ringR(i) && dPrev(i) > ringR(i)
                    phase(i) = "DIVE"; R0(i) = norm(T - X(:, i));
                end
            end
            dPrev(i) = norm(T(1:2) - X(1:2, i));

        % ── GUIDED:PPN + LOS 地板 ────────────────────────────────
        case "DIVE"
            r = T - X(:, i);  R = max(norm(r), 1e-6);
            v = V(i) * [cos(gam(i))*cos(psi(i)); cos(gam(i))*sin(psi(i)); -sin(gam(i))];
            if R <= p.fzR || R/max(V(i),1) <= p.fzTgo
                if isnan(frz(1, i)), frz(:, i) = [psi(i); gam(i)]; end
                psi(i) = frz(1, i); gam(i) = frz(2, i);
            else
                [psi(i), gam(i)] = i_pn(X(:, i), T, psi(i), gam(i), V(i), p, dt);
            end
            X(:, i) = i_move(X(:, i), V(i), psi(i), gam(i), dt);
            if X(3, i) >= 0                       % 觸地
                X(3, i) = 0;
                tImp(i) = t; missH(i) = norm(X(1:2, i));
                gamF(i) = gam(i); phase(i) = "DONE";
            end
        end

        if phase(i) ~= "PREP" && phase(i) ~= "DONE"
            log{i}(end+1, :) = [t, X(:, i).', V(i), rad2deg(psi(i)), ...
                                rad2deg(gam(i)), i_phaseNum(phase(i))];
        end
    end
    t = t + dt;
end

% ── 結算 ───────────────────────────────────────────────────────────
for i = n:-1:1
    out.veh(i).id     = i;
    out.veh(i).tIP    = tIP(i);
    out.veh(i).tImp   = tImp(i);
    out.veh(i).missH  = missH(i);
    out.veh(i).gamF   = rad2deg(gamF(i));
    out.veh(i).R0     = R0(i);
    out.veh(i).loiter = loitT(i);
    out.veh(i).IP     = IP(:, i);
    out.veh(i).traj   = array2table(log{i}, 'VariableNames', ...
        {'t','pn','pe','pd','V','psi','gam','phase'});
end
out.p = p; out.tGate = tGate; out.target = T;
out.ringR = p.ringR; out.turnRmin = p.turnRmin;
end

% ═══════════════════════════════════════════════════════════════════
function [psi, gam] = i_steer(X, tgt, psi, V, p, climbing)
% 朝航點轉向,轉率受坡度上限;爬升角受上限。
d = tgt(1:2) - X(1:2);
want = atan2(d(2), d(1));
psi = i_turn(psi, want, V, p);
if climbing
    gam = p.climbMax;
else
    dz = (-tgt(3)) - (-X(3));                      % 需要爬/降的高度
    dh = max(norm(d), 1);
    gam = max(-p.climbMax, min(p.climbMax, atan2(dz, dh)));
end
end

function [psi, gam] = i_loiter(X, C, R, psi, V, p)
% 盤旋:朝「圓上最近的切點方向」轉。
d = X(1:2) - C(1:2); r = max(norm(d), 1);
tang = [-d(2); d(1)] / r;                          % 逆時針切向
radial = d / r;
err = (r - R) / max(R, 1);                         % >0 = 在圈外,要往內
aim = tang - 1.2 * err * radial;
want = atan2(aim(2), aim(1));
psi = i_turn(psi, want, V, p);
gam = max(-p.climbMax, min(p.climbMax, ((-C(3)) - (-X(3))) / max(V, 1)));
end

function [psi, gam] = i_pn(X, T, psi, gam, V, p, dt)
% PPN + LOS 地板 —— 與 tot_node._pn_cmd 同結構(見那裡的完整論證)。
r = T - X;                                  % NED:目標在下方時 r(3) > 0
d = max(hypot(r(1), r(2)), 1e-6);
h = -X(3);
R = max(norm(r), 1e-6);
v = V * [cos(gam)*cos(psi); cos(gam)*sin(psi); -sin(gam)];

% ── 橫向:λ̇ 解析求得,a_h = N·vh·λ̇ ──────────────────────────────
vh = max(hypot(v(1), v(2)), 1e-6);
lamdot = (r(1)*v(2) - r(2)*v(1)) / (d*d);
a_h = -p.Nlat * vh * lamdot;
aMaxLat = p.g * tan(p.bankMax);             % 坡度上限 ⇒ 橫向加速度上限
a_h = max(-aMaxLat, min(aMaxLat, a_h));
psi = psi + (a_h / vh) * dt;

% ── 縱向 ────────────────────────────────────────────────────────
% ⚠️⚠️ 視線仰角 ε = atan2(-r3, d)(向上為正,目標在下方時為負),
%    其角速率必須用**解析式**:
%        ε̇ = ( d·(-ṙ₃) + r₃·ḋ ) / R²,靜止目標 ṙ = −v
%    第一版把這一段寫錯,γ 被推過 −90° 之後 cos(γ) 變號、
%    整條迴路正回饋 ⇒ 落角跑出 **−150° ~ −176°** 這種不存在的值。
rdot = -v;
ddot = (r(1)*rdot(1) + r(2)*rdot(2)) / d;
epsdot = (d*(-rdot(3)) + r(3)*ddot) / (R*R);

gdot = p.Nvert * epsdot;
nz = cos(gam) + V*gdot/p.g;
nz = max(p.nzMin, min(p.nzMax, nz));
gdot = (nz - cos(gam)) * p.g / V;
gam_cmd = gam + gdot * 1.0;                 % pn_lead = 1.0

if p.losFloor > 0
    % ⚠️ 幾何否決:指令俯角不准比視線更淺。取 min = 更負 = 更陡。
    %    這一步**繞過**上面的 n_z 夾制 —— 與生產碼同一個結構,
    %    也同一個代價(見 time-term-yes-lateral-no:59.57% 的拍由它決定)。
    gam_los = -atan2(h, max(d, 1e-3));
    gam_cmd = min(gam_cmd, gam_los * p.losFloor);
end

% ⚠️ 真正的**角速率**限制(rad/s × dt),不是每步的固定夾值。
%    第一版用 ±0.35 rad **每步**,dt=0.05 之下等於 7 rad/s —— 形同沒有限制。
gdotMax = p.g * (p.nzMax + 1) / V;          % 由過載預算換算的 γ̇ 上限
dgam = max(-gdotMax*dt, min(gdotMax*dt, gam_cmd - gam));
gam = gam + dgam;
% ⚠️ 夾在 ±85°:γ 穿過 ±90° 會讓 cos(γ) 變號,整條縱向解翻面。
gam = max(-deg2rad(85), min(deg2rad(85), gam));
end

function psi = i_turn(psi, want, V, p)
% 轉率受坡度上限:psidot = g tan(phi) / V
e = mod(want - psi + pi, 2*pi) - pi;
rmax = p.g * tan(p.bankMax) / max(V, 1) * p.dt;
psi = psi + max(-rmax, min(rmax, e));
end

function X = i_move(X, V, psi, gam, dt)
X = X + V*dt*[cos(gam)*cos(psi); cos(gam)*sin(psi); -sin(gam)];
end

function arc = i_arcToRing(X, C, Rl, ringR, psi)
% 盤旋圈上到「下一次向內穿越環半徑」的弧長 [m]。
% 幾何:d^2 = ringR^2 + Rl^2 + 2·ringR·Rl·cos(θ),d = ringR 時
%       cos(θ) = −Rl/(2·ringR)  ⇒ 兩個穿越點 θ = ±θc。
% ⚠️ 只有其中一個是**向內**穿越,取逆時針方向的下一個。
c = -Rl / (2*ringR);
if c < -1 || c > 1, arc = 0; return; end      % 幾何上不相交 ⇒ 立刻放行
thc = acos(c);
% 目標→IP 的外向徑向,當作 θ=0
u0 = -C(1:2) / max(norm(C(1:2)), 1e-9);       % IP 指向目標的反向 = 外向
d  = X(1:2) - C(1:2);
th = atan2(d(2)*u0(1) - d(1)*u0(2), d(1)*u0(1) + d(2)*u0(2));
th = mod(th, 2*pi);
% 向內穿越發生在 θ = +thc(逆時針繞行時 d 由大變小的那一側)
tgt = mod(thc, 2*pi);
dth = mod(tgt - th, 2*pi);
arc = dth * Rl;
end

function k = i_phaseNum(ph)
switch ph
  case "TAKEOFF", k = 1; case "CLIMB", k = 2; case "TRANSIT", k = 3;
  case "LOITER",  k = 4; case "DIVE",  k = 5; otherwise, k = 0;
end
end
