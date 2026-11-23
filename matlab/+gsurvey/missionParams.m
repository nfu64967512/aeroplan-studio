function p = missionParams(varargin)
%GSURVEY.MISSIONPARAMS  全任務(起飛→AUTO→決斷圈→GUIDED)的參數。
%
%   ⚠️ 這一組刻意與 `tot_node` 的生產參數**同名同值**,方便把模擬結果
%      直接對回契約快照。對不上的地方在下面逐項標了。

ip = inputParser;

% ── 機隊 ───────────────────────────────────────────────────────────
ip.addParameter('n',          8);        % 6–8
ip.addParameter('homeR',  40000.0);      % 起飛點距紅心 [m]
ip.addParameter('homeBrgOff', 0.0);      % 起飛點相對進場方位的偏置 [deg]
ip.addParameter('launchSep', 120.0);     % 起飛點之間的橫向間隔 [m]
ip.addParameter('stagger',    3.0);      % 彈射間隔 [s](使用者定調的驗收①)

% ── 目標與扇形幾何(= tot_node 生產值)──────────────────────────
ip.addParameter('approach',  90.0);      % 進場方位 [deg 從北]
ip.addParameter('fanHalf',   25.0);      % half_angle
ip.addParameter('perchAlt', 3100.0);     % perch_alt
ip.addParameter('diveAngle', 27.0);      % dive_angle → auto_ring 反推環半徑
ip.addParameter('diveAlt',    0.0);      % dive_alt(海平面紅心)

% ── AUTO 段(= 我今天加進 tot_node 的那一組)────────────────────
ip.addParameter('autoIpR',      0.0);    % auto_ip_r;0 = 用環半徑 ring_of(i)
ip.addParameter('autoIpAlt',    0.0);    % auto_ip_alt;0 = perch_of(i)
ip.addParameter('autoIpReach',5000.0);   % auto_ip_reach
ip.addParameter('autoLoiterR',3300.0);   % auto_loiter_r
ip.addParameter('autoTransitR',   0.0);  % auto_transit_r;0 = 不放進場航點
% ⭐ release_at_ring:閘門開啟後**等下一次向內穿越環半徑**才放行。
%    代價是最多半個盤旋週期的等待;買到的是 R0 收斂成一個常數。
ip.addParameter('releaseAtRing', false);
% ⭐⭐ ring_pace:閘門開啟後在盤旋段**調速**,讓全隊同時穿越環。
%    ⚠️ 這一顆治的是 releaseAtRing 帶來的 modulo 假影:各機在圈上的相位
%       隨機 ⇒ 有的剛過穿越點要再繞一整圈。實測等待 [109..143] 對 [2.5]。
ip.addParameter('ringPace',   false);
ip.addParameter('Vmin',        95.0);    % 調速下限(AIRSPEED_MIN 之上)
ip.addParameter('Vmax',       150.0);    % 調速上限

% ── 飛行包絡 ───────────────────────────────────────────────────────
ip.addParameter('V0',       161.7);      % 巡航速率 [m/s](實測)
ip.addParameter('Vto',       45.0);      % 離地速度 [m/s]
ip.addParameter('bankMax',   45.0);      % 坡度上限 [deg] ⇒ 決定迴轉半徑
ip.addParameter('climbMax',  12.0);      % 最大爬升角 [deg](T/W 0.274~0.38 的現實)
ip.addParameter('accel',      3.0);      % 起飛加速度 [m/s^2]

% ── 導引段(= tot_node 生產值)──────────────────────────────────
ip.addParameter('Nlat',       4.0);      % pn_n_lat
ip.addParameter('Nvert',      5.0);      % pn_n_vert
ip.addParameter('losFloor',   1.0);      % pn_los_floor
ip.addParameter('nzMin',     -1.0);      % pn_nz_min
ip.addParameter('nzMax',      3.0);      % nz_max_g
ip.addParameter('fzR',      100.0);      % pn_freeze_r
ip.addParameter('fzTgo',      1.5);      % pn_freeze_tgo

% ── 積分 ───────────────────────────────────────────────────────────
ip.addParameter('dt',        0.05);      % = TICK_PERIOD_S
ip.addParameter('tMax',     900.0);
ip.addParameter('wpAccept', 400.0);      % 航點接受半徑 [m]

ip.parse(varargin{:});
p = ip.Results;
p.g = 9.80665;
p.approach  = deg2rad(p.approach);
p.fanHalf   = deg2rad(p.fanHalf);
p.diveAngle = deg2rad(p.diveAngle);
p.bankMax   = deg2rad(p.bankMax);
p.climbMax  = deg2rad(p.climbMax);
p.homeBrgOff = deg2rad(p.homeBrgOff);

% ⭐ auto_ring:環半徑由 (perch_alt, dive_alt, dive_angle) 反推 —— 與
%    tot_node.__init__ 的 `r_need = (perch_alt - dive_alt)/tan(dive_angle)` 同式。
p.ringR = (p.perchAlt - p.diveAlt) / tan(p.diveAngle);

% ⚠️ 最小迴轉半徑 = V^2/(g tan phi)。這一個數字決定決斷圈**飛不飛得出來**,
%    而 hold-ring-is-unflyable 記載的失效就是指令半徑小於它。
p.turnRmin = p.V0^2 / (p.g * tan(p.bankMax));
end
