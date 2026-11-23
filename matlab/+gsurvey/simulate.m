function out = simulate(lawFn, scn, p)
%GSURVEY.SIMULATE  純運動學積分:n 機 × 一條導引律。
%
%   out = gsurvey.simulate(@gsurvey.laws.ppn, scn, p)
%
%   質點運動學,沒有姿態迴路、沒有夾制鏈、沒有飛控。
%       ṗ = v
%       v̇ = a_cmd(+ 重力) − (速率保持時)沿 v 的分量
%   ⚠️ 這是**刻意**的:見 Contents.m。要含飛控請用 mgs.simSwarm。
%
%   命中判定:RK4 走一步之後若斜距由減轉增(通過最近點)或 R < hitR,
%   就用**割線法**在該步內求根,並把整個狀態向量內插到那個根上。
%   ⚠️ 「一部分輸出內插、一部分沒有」在結構上不可能發生 —— 全部一起插。
%
%   out.veh(i).hit .tImpact .miss .gammaF .Vf .aStats .track .traj
%   out.law, out.p, out.scn

arguments
    lawFn function_handle
    scn struct
    p struct
end

n = numel(scn.veh);
g = [0; 0; p.g];                      % NED:重力朝下(+D)

% ── 狀態 ───────────────────────────────────────────────────────────
X = zeros(6, n);                       % [p; v]
for i = 1:n
    X(:, i) = [scn.veh(i).p0; scn.veh(i).v0];
end
alive  = true(1, n);
tImp   = nan(1, n);
Xf     = nan(6, n);
% ⚠️ 用 **cell** 不是 struct array:不同的律會往狀態裡塞不同的欄位,
%    而 struct array 要求所有元素欄位一致 ⇒ 第二架塞了第一架沒有的欄位
%    就 "Subscripted assignment between dissimilar structures"。
sLaw   = repmat({struct()}, 1, n);     % 每機獨立的律狀態
aPrev  = zeros(3, n);                  % 執行器遲滯用
aLog   = cell(1, n);
trkLen = zeros(1, n);
traj   = cell(1, n);
term   = repmat({'timeout'}, 1, n);
for i = 1:n, aLog{i} = []; traj{i} = []; end

t = 0;
K = ceil(p.tMax / p.dt);

for k = 1:K
    if ~any(alive), break; end

    % 這一拍所有存活機體的協同觀測(協同律要;含自己)
    peers = i_peers(X, alive, p);

    for i = 1:n
        if ~alive(i), continue; end
        o = i_observe(t, X(:, i), scn.target, p, i, peers);

        % ── 放行時刻之前:直飛,不受導引 ────────────────────────────
        % ⚠️ 這一段就是「放行時刻散布」的模型。ring_pace 修的是它,
        %    而 spread-decomposition 說它與「俯衝耗時差」是可分離的兩項。
        if t < scn.veh(i).t0
            aC = [0; 0; 0];
            preRelease = true;
        else
            [aC, sLaw{i}] = i_call(lawFn, o, p, sLaw{i});
            preRelease = false;
        end
        % ── ⭐ 終端凍結(照 tot_node._pn_step)────────────────────────
        %  R 或 t_go 夠小就鎖住指令。理由是 λ̇ = (r × ṙ)/R² 在 R→0 時
        %  分母爆掉 —— 那是**轉向**通道的奇異點,不是律的性質。
        if ~preRelease && (o.R <= p.fzR || o.tgo <= p.fzTgo)
            if isfield(sLaw{i}, 'aFrz') && ~isempty(sLaw{i}.aFrz)
                aC = sLaw{i}.aFrz;
            else
                sLaw{i}.aFrz = aC(:);
            end
        end
        if numel(aC) ~= 3
            error('gsurvey:lawShape', ...
                '%s 回傳 %d 個分量,介面要求 3x1(見 gsurvey/Contents.m)', ...
                func2str(lawFn), numel(aC));
        end
        aC = aC(:);
        if any(~isfinite(aC))
            aC = [0; 0; 0];            % 律吐 NaN 一律當零,但要記下來
            sLaw{i}.nanTicks = i_get(sLaw{i}, 'nanTicks', 0) + 1;
        end

        % ── 指令上限(記錄飽和率,這是本測試台最重要的輸出之一)──
        aMag = norm(aC);
        sat = aMag > p.aMax + 1e-9;
        if sat, aC = aC * (p.aMax / aMag); end
        % ⚠️ 放行前的拍不計進需求統計:那一段律沒在跑,把零算進中位數
        %    會把「這條律要多少 g」稀釋成一個假的小數字。
        if ~preRelease
            aLog{i}(end+1, :) = [aMag, double(sat)];   %#ok<AGROW>
        end

        % ── 執行器遲滯(0 = 理想,預設)────────────────────────────
        if p.tau_a > 0
            lam = p.dt / (p.tau_a + p.dt);
            aC = aPrev(:, i) + lam * (aC - aPrev(:, i));
        end
        aPrev(:, i) = aC;

        % ── RK4 一步 ──────────────────────────────────────────────
        x0 = X(:, i);
        R0 = norm(scn.target - x0(1:3));
        f  = @(x) i_deriv(x, aC, g, p);
        k1 = f(x0);
        k2 = f(x0 + 0.5*p.dt*k1);
        k3 = f(x0 + 0.5*p.dt*k2);
        k4 = f(x0 +     p.dt*k3);
        x1 = x0 + (p.dt/6) * (k1 + 2*k2 + 2*k3 + k4);
        R1 = norm(scn.target - x1(1:3));

        trkLen(i) = trkLen(i) + norm(x1(1:3) - x0(1:3));
        traj{i}(end+1, :) = [t, x0(1:3).', norm(x0(4:6)), R0, aMag]; %#ok<AGROW>

        % ── 終點三條:觸地 / 進入 hitR / 通過最近點 ────────────────
        % ⚠️⚠️ **觸地要排在最前面。** 沒有地面約束時,不含重力補償的律
        %    會沉到地面以下再拉回來,而「回到 R=hitR」那一拍量到的落角
        %    是**正的(爬升)** —— 一個完全不存在的飛行狀態。
        %    第一版就是這樣:實測落角 +17.2°,而軌跡最低點在地面下 117 m。
        hitGround = p.ground && x1(3) >= 0 && x0(3) < 0;
        if hitGround
            [tr, xr] = i_ground(x0, aC, g, p);
        elseif R1 > R0 || R1 < p.hitR
            [tr, xr] = i_root(x0, aC, g, p, scn.target, R0, R1);
        else
            X(:, i) = x1;
            continue;
        end
        tImp(i)  = t + tr;
        Xf(:, i) = xr;
        alive(i) = false;
        term{i}  = ternIf(hitGround, 'ground', 'cpa');
        traj{i}(end+1, :) = [tImp(i), xr(1:3).', norm(xr(4:6)), ...
                             norm(scn.target - xr(1:3)), aMag];
    end
    t = t + p.dt;
end

% 逾時未結束的機體:用最後狀態結算,並標記
for i = 1:n
    if alive(i)
        tImp(i)  = t;
        Xf(:, i) = X(:, i);
        alive(i) = false;
        sLaw{i}.timedOut = true;
    end
end

% ── 逐機結算 ───────────────────────────────────────────────────────
for i = n:-1:1
    xf   = Xf(:, i);
    miss = norm(scn.target - xf(1:3));
    vf   = xf(4:6);
    a    = aLog{i};
    out.veh(i).id      = i;
    out.veh(i).term    = term{i};
    % 水平落點誤差:觸地時這才是「打得準不準」,斜距會把殘餘高度算進去
    out.veh(i).missH   = hypot(xf(1), xf(2));
    out.veh(i).zRes    = -xf(3);
    out.veh(i).hit     = miss <= max(p.hitR, 1e-6) * 5;
    out.veh(i).tImpact = tImp(i);
    out.veh(i).miss    = miss;
    out.veh(i).Vf      = norm(vf);
    % 落角:向下為負(NED 的 D 分量為正代表在下降)
    out.veh(i).gammaF  = -atan2(vf(3), hypot(vf(1), vf(2)));
    out.veh(i).aMed    = median(a(:, 1));
    out.veh(i).aP95    = prctile(a(:, 1), 95);
    out.veh(i).aMax    = max(a(:, 1));
    out.veh(i).satFrac = mean(a(:, 2));
    out.veh(i).nanTicks = i_get(sLaw{i}, 'nanTicks', 0);
    out.veh(i).timedOut = i_get(sLaw{i}, 'timedOut', false);
    % 航跡比 = 實際航程 / 起點到目標的直線距離
    % ⚠️ 這一欄很重要:release_at_ring 把它從 1.9 壓到 1.05,
    %    而真觸地率跟著 7/14 → 30/32。時間控制律買時間就是在花它。
    out.veh(i).track   = trkLen(i) / norm(scn.target - scn.veh(i).p0);
    out.veh(i).traj    = array2table(traj{i}, 'VariableNames', ...
        {'t', 'pn', 'pe', 'pd', 'V', 'R', 'aCmd'});
end
out.law = func2str(lawFn);
out.p   = p;
out.scn = scn;
end

% ═══════════════════════════════════════════════════════════════════
function d = i_deriv(x, aC, g, p)
v = x(4:6);
a = aC;
if p.useGravity
    gv = g;
    if p.gravComp
        % 配平:抵銷重力**垂直於速度**的分量(升力做的事)。
        % 沿速度的分量留著 —— 那是俯衝加速/爬升減速的真實物理。
        V = norm(v);
        if V > 1e-6, gv = (g.' * v / V^2) * v; end
    end
    a = a + gv;
end
if p.speedHold
    % 速率保持:把沿速度方向的分量扣掉 ⇒ 只轉向不加減速。
    % ⚠️ 這是「有動力機體維持巡航速度」的理想化。要研究速度-時間
    %    兌換的律(DD-ITCG 那一族)請把 speedHold 關掉。
    V = norm(v);
    if V > 1e-6, a = a - (a.' * v / V^2) * v; end
end
d = [v; a];
end

% ═══════════════════════════════════════════════════════════════════
function [tr, xr] = i_root(x0, aC, g, p, tgt, R0, R1)
% 割線法在 [0, dt] 內找 dR/dt = 0 或 R = hitR 的根,再 RK4 積到那裡。
lo = 0; hi = p.dt;
f  = @(x) i_deriv(x, aC, g, p);
step = @(h) i_rk4(x0, f, h);
if R1 < p.hitR
    % 進入命中球:找 R = hitR
    obj = @(h) norm(tgt - subsref(step(h), substruct('()', {1:3}))) - p.hitR;
else
    % 通過最近點:找 dR/dh = 0,用數值差分
    obj = @(h) i_dR(step, tgt, h, p.dt);
end
a0 = obj(lo); b0 = obj(hi);
tr = hi;
if isfinite(a0) && isfinite(b0) && a0 * b0 < 0
    for it = 1:60
        m = 0.5 * (lo + hi);
        fm = obj(m);
        if ~isfinite(fm), break; end
        if a0 * fm <= 0, hi = m; b0 = fm; else, lo = m; a0 = fm; end
        if hi - lo < 1e-9, break; end
    end
    tr = 0.5 * (lo + hi);
end
xr = step(tr);
if R1 <= R0 && R1 < p.hitR, tr = min(tr, p.dt); end
end

function [tr, xr] = i_ground(x0, aC, g, p)
% 觸地:在 [0, dt] 內二分找 pd = 0(高度 0)。
f = @(x) i_deriv(x, aC, g, p);
step = @(h) i_rk4(x0, f, h);
lo = 0; hi = p.dt;
for it = 1:60
    m = 0.5 * (lo + hi);
    xm = step(m);
    if xm(3) >= 0, hi = m; else, lo = m; end
    if hi - lo < 1e-9, break; end
end
tr = 0.5 * (lo + hi);
xr = step(tr);
end

function v = ternIf(c, a, b)
if c, v = a; else, v = b; end
end

function d = i_dR(step, tgt, h, dt)
e = max(dt * 1e-4, 1e-7);
h1 = max(h - e, 0); h2 = h + e;
x1 = step(h1); x2 = step(h2);
d = (norm(tgt - x2(1:3)) - norm(tgt - x1(1:3))) / (h2 - h1);
end

function x = i_rk4(x0, f, h)
if h <= 0, x = x0; return; end
k1 = f(x0); k2 = f(x0 + 0.5*h*k1); k3 = f(x0 + 0.5*h*k2); k4 = f(x0 + h*k3);
x = x0 + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
end

% ═══════════════════════════════════════════════════════════════════
function o = i_observe(t, x, tgt, p, id, peers)
pos = x(1:3); v = x(4:6);
r  = tgt - pos;
R  = max(norm(r), 1e-6);
V  = max(norm(v), 1e-6);
los = r / R;
rdot = -v;                              % 靜止目標:ṙ = vt − v = −v
o.t = t; o.id = id;
o.r = r; o.R = R; o.v = v; o.V = V; o.vt = [0; 0; 0];
o.los = los;
o.omega = cross(r, rdot) / R^2;         % 視線角速度向量
o.Vc = -(rdot.' * los);                 % 接近速率(正 = 在接近)
o.tgo = R / max(o.Vc, 1e-3);
o.sigma = real(acos(max(-1, min(1, (v.' * los) / V))));
o.gamma = -atan2(v(3), hypot(v(1), v(2)));
o.peers = peers;
end

function peers = i_peers(X, alive, p)
idx = find(alive);
if isempty(idx)
    peers = struct('id', {}, 'R', {}, 'tgo', {}, 'V', {});
    return;
end
for j = numel(idx):-1:1
    i = idx(j);
    pos = X(1:3, i); v = X(4:6, i);
    r = -pos; R = max(norm(r), 1e-6);
    Vc = max((v.' * (r / R)), 1e-3);
    peers(j).id  = i;
    peers(j).R   = R;
    peers(j).tgo = R / Vc;
    peers(j).V   = norm(v);
end
end

% ═══════════════════════════════════════════════════════════════════
function [a, s] = i_call(lawFn, o, p, s)
% 律可以回 (a) 或 (a, s)。兩種都吃,但**不要**讓律用 persistent。
if nargout(lawFn) >= 2
    [a, s] = lawFn(o, p, s);
else
    a = lawFn(o, p, s);
end
end

function v = i_get(s, f, d)
if isstruct(s) && isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end
