function [a, s] = itcg(o, p, s)
%GSURVEY.LAWS.ITCG  撞擊時間控制導引(Impact Time Control Guidance)。
%
%       a = a_PN  +  a_time
%       a_PN   = N·V·(ω × v̂)                       ← 保證命中
%       a_time = K·(V²/R)·(t_d − t − t_go)          ← 橫向偏置,吸收時間誤差
%
%   施加在**橫向**通道(Jeon/Lee 2006、Guo 2020 都是這樣選的)。
%   ⚠️ Guo 選橫向的理由是逐字的:「To avoid complex coordination of the
%      two channels」—— 那是**實作簡便**,不是橫向權限比較大。
%
% ===========================================================================
%  ⚠️⚠️ 這條律只能**延後**,不能提前
% ===========================================================================
%       de_t/dt = −V·(1 − cos σ) ≤ 0
%   σ 是前置角,cos σ ≤ 1 ⇒ 導數恆 ≤ 0 ⇒ 時間誤差單調不增。
%   ⇒ **已經遲到(e_t ≤ 0)時任何橫向機動都救不回來**,直接回 0。
%   ⇒ 也因此共同命中時刻 t_d 必須取 `max_j`,取平均會讓最慢那架永遠追不上。
%
%   ⚠️ 側向要**鎖存**:文獻的 sign(σ) 在 σ=0 有奇異點,抖動會讓偏置左右
%      翻面,而翻面頻率會直接寫進航跡(同型失效見 stot-altitude-gate-chatter,
%      UAV5 翻 20 次 → 末端誤差 35.6 km)。

vhat = o.v / o.V;
aPN = p.N * o.V * cross(o.omega, vhat);

% ── 共同命中時刻:0 = 自動取全隊最慢者(我們的 dive_T 作法)──────
if p.td > 0
    td = p.td;
else
    if ~isfield(s, 'td') || isempty(s.td)
        s.td = max([o.peers.tgo]) + o.t;     % 放行當拍鎖定一次
    end
    td = s.td;
end

eT = (td - o.t) - o.tgo;                     % > 0 = 太早到 ⇒ 要繞路
if eT <= 0
    a = aPN;                                 % 遲到了,橫向救不回來
    s.eT = eT; s.aT = 0;
    return;
end

% ── 側向鎖存 ───────────────────────────────────────────────────────
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

aMag = p.itcgGain * (o.V^2 / max(o.R, 1)) * eT;
aT = aMag * sideVec;
aT = aT - (aT.' * vhat) * vhat;              % 只留垂直於速度的分量

a = aPN + aT;
s.eT = eT; s.aT = norm(aT);
end
