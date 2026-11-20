function [a, s] = mpng(o, p, s)
%GSURVEY.LAWS.MPNG  改進型比例導引(Jiang 2021, Aerospace 8(8):231)。
%
%   無奇異點的解析式時間控制:把時間誤差**反解成需要的前置角**,
%   再用 PN 的回復項把飛機推到那個前置角上,而不是直接加一個加速度偏置。
%
%       σ_need = asin( clamp( a_req·R / (N·V²), −1, 1) )
%       a      = a_PN + N·V²·(sin σ_need − sin σ)/R · ŝ
%
%   ⭐ 與 ITCG 的差別:ITCG 的偏置量與 e_t 成正比、**沒有上界**;
%      MPNG 先把需求轉成一個**有界的角度**再驅動,所以它在 R → 0 時
%      不會發散 —— 那正是「無奇異點」的意思。
%
%   ⚠️ 我們的實作(tot_node.py `_itcg_bias`)用的是這一支的 surplus 形式,
%      而且**誤差用公尺不用秒**(Kim & Shin 2023 eq 5):
%          e_s = V·(t_d − t) − R
%      理由是秒的形式要估 t_go,而 t_go 是本專案吃過兩次虧的量
%      (時基污染吐出 17.8 億秒;靜態幾何式系統性低估 2 倍)。
%      公尺形式的 V、R、t_d 三個量全部直接可測。
%
%   ⚠️ 2026-08-26 量到的事實:在我們的工作點上,這條律要求的加速度
%      中位 0.89–2.79 g,而坡度預算只有 1.94–2.73 g **且與 PN 共用**
%      ⇒ 47–100% 的拍會飽和。看模擬時請盯 satFrac 那一欄。

vhat = o.v / o.V;
aPN = p.N * o.V * cross(o.omega, vhat);

if p.td > 0
    td = p.td;
else
    if ~isfield(s, 'td') || isempty(s.td)
        s.td = max([o.peers.tgo]) + o.t;
    end
    td = s.td;
end

% ⭐ 公尺形式的餘裕:> 0 表示「剩下的時間夠飛比 R 更遠」⇒ 要繞路
eS = o.V * (td - o.t) - o.R;
if eS <= 0
    a = aPN; s.eS = eS; s.sigNeed = 0;
    return;
end

% 需要的前置角:繞路長度 ≈ R·(1/cos σ − 1) ⇒ 反解 σ
% ⚠️ 用 acos 的反解,並夾在 [0, 80°] —— 不夾的話 eS 大時會要求 90°,
%    那是「垂直於視線飛」= 完全不接近目標。
cosNeed = o.R / max(o.R + eS, 1e-6);
sigNeed = min(acos(max(-1, min(1, cosNeed))), deg2rad(80));

% ⚠️⚠️ 鎖存的是**符號**,不是向量本身。第一版把整個單位向量латch 在
%    慣性系裡,繞路一長它就與視線脫節 ⇒ 偏置指到不相干的方向,
%    實測 missH 67–134 m、落角被拉平到 −14°。
%    文獻要防的是 sign(σ) 在 σ≈0 的翻面抖動(同型失效見
%    stot-altitude-gate-chatter:UAV5 翻 20 次 → 末端誤差 35.6 km),
%    所以鎖符號、方向每拍隨視線重算。
hn = cross(o.los, [0; 0; -1]);               % 水平面內、視線的法向
if norm(hn) < 1e-9, hn = [1; 0; 0]; else, hn = hn / norm(hn); end
if ~isfield(s, 'sgn') || isempty(s.sgn)
    s.sgn = 1;
end
sideVec = s.sgn * hn;

% 驅動項:把實際 σ 推向 σ_need。回復項與 PN 同尺度 ⇒ 兩者可比。
aMag = p.itcgGain * p.N * o.V^2 * (sin(sigNeed) - sin(o.sigma)) / max(o.R, 1);
aT = aMag * sideVec;
aT = aT - (aT.' * vhat) * vhat;

a = aPN + aT;
s.eS = eS; s.sigNeed = sigNeed;
end
