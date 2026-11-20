function scn = scenario(p)
%GSURVEY.SCENARIO  n 機的放行初始條件(扇形展開,共同紅心)。
%
%   scn = gsurvey.scenario(gsurvey.params('n', 8))
%
%   目標在原點且**在地面上**(海平面紅心,見 sea-level-target-datum)。
%   各機在斜距 R0 的球面上,方位均勻分佈在 approach ± fanHalf 的扇形內。
%
% ===========================================================================
%  ⚠️⚠️⚠️ 為什麼**必須**有散度 —— 這一段是本檔最重要的部分
% ===========================================================================
%  第一版讓八架的 R0 / σ / 放行時刻完全相同。結果是八條逐位元相同的軌跡、
%  **散布恆等於 0.000 s**,於是每一條時間控制律都「完美達標」。
%  那是 [[instrument-that-cannot-fire]] 的教科書案例:被測量在結構上
%  不可能非零,而恆零正好長得像我們想要的答案。
%
%  真實的散布有**兩個**來源,而且它們是可分離的(見 spread-decomposition):
%      ① 放行**時刻**差   —— ring_pace 修的就是這個(33.9 → 5.6 s)
%      ② 放行**幾何**差   —— R0 與航向;實測 R0 差 500 m ⇒ 耗時差 3.3 s
%  兩個都要模型化,否則「這條律修得掉散布嗎」這個問題沒有意義。
%
%  預設值取實測量級:R0spread 500 m、tStagger 5 s(ring_pace 之後的實測)。
%  要重現 ring_pace **之前**的情況把 tStagger 設 33。
%
%   scn.veh(i).p0 [3x1] NED 位置(目標在原點,D 向下為正 ⇒ 高度是 −pd)
%   scn.veh(i).v0 [3x1] NED 速度
%   scn.veh(i).t0 [1]   本機的**放行時刻**;在此之前直飛不受導引

arguments
    p struct
end

n = p.n;
scn.target = [0; 0; 0];
scn.p = p;

% 扇形方位:n=1 取中心,否則均勻鋪在 ±fanHalf
if n <= 1, off = 0; else, off = linspace(-p.fanHalf, p.fanHalf, n); end

% 散度:**確定性**不是隨機 —— 同一組參數必須每次跑出同一個答案,
% 否則律與律之間的差異會混進取樣雜訊(本專案已經因為牆鐘不確定性
% 吃過虧,見 conftest.py 的 solver 預算那一段)。
if n <= 1
    fR = 0; fT = 0;
else
    u  = linspace(-1, 1, n);                 % −1 … +1
    fR = u;                                  % R0 散度線性鋪開
    fT = u([1:2:n, 2:2:n]);                  % 時刻散度**交錯**鋪開,
end                                          % 免得 R0 與 t0 完全相關

for i = n:-1:1
    brg = p.approach + off(i);
    R0i = p.R0 + 0.5 * p.R0spread * fR(i);
    % ⚠️ R0 是**斜距**不是水平距(RRE 實測 6854 m 是斜距)。
    d0  = sqrt(max(R0i^2 - p.alt0^2, 1));

    scn.veh(i).id = i;
    scn.veh(i).p0 = [ d0*cos(brg); d0*sin(brg); -p.alt0 ];
    % ⚠️ 這裡**不要**在迴圈內做正規化:迴圈是 n→1 倒著跑的,
    %    `[scn.veh.t0]` 只讀得到已填的那幾格 ⇒ 減到的是一個部分最小值,
    %    結果是 8 架裡有 5 架的 t0 被壓成 0,散度悄悄消失一大半。
    %    正規化統一放在迴圈**之後**。
    scn.veh(i).t0 = 0.5 * p.tStagger * fT(i);

    % 速度:直指目標,再在水平面內偏開 sigma0(前置角)
    losv = scn.target - scn.veh(i).p0;
    losv = losv / norm(losv);
    hn = [-losv(2); losv(1); 0];
    if norm(hn) < 1e-9, hn = [1; 0; 0]; else, hn = hn / norm(hn); end
    % 前置角朝扇形**外**側:內外側機的 σ 反號,八架才不會全往同一邊繞
    sgn = sign(off(i)); if sgn == 0, sgn = 1; end
    dirv = cos(p.sigma0)*losv + sin(p.sigma0)*sgn*hn;
    scn.veh(i).v0 = p.V0 * dirv / norm(dirv);
end

% t0 正規化(上面的 min 在迴圈裡取不到還沒填的元素)
t0s = [scn.veh.t0];
for i = 1:n, scn.veh(i).t0 = scn.veh(i).t0 - min(t0s); end
end
