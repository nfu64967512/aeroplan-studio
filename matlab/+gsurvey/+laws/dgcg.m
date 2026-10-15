function [a, s] = dgcg(o, p, s)
%GSURVEY.LAWS.DGCG  分佈式群組協同導引(兩階段:共識 → 純 PN)。
%
%   第一階段(遠端):對**剩餘距離**取群體共識,靠橫向偏置把落後/超前
%                    的機體拉齊。
%   第二階段(近端):共識達成後**整條退回純 PN**,不再有時間項。
%
%       if |R_i − R̄| ≥ tol :  a = a_PN + k·(R_i − R̄)·(V²/R)·ŝ
%       else                :  a = a_PN
%
%   ⭐⭐ **這一條最貼我們的架構**,而且是有文獻先例的:
%      Zhu / Wang / Lin, "Consensus-Based Impact-Time-Control Guidance Law
%      for Cooperative Attack of Multiple Missiles" 的 Eq.(11) 就是這個
%      形狀 —— 用**距離誤差**當切換判準,而不是時間誤差。
%
%   ⭐ 對我們特別合適的理由:**距離我們量得準**,而時間誤差要過預測器,
%      而我們的預測器低估中位 +18.7 s、27/27 單邊為正。用量得準的東西
%      當閘門。`tot_node._handover_criteria` 已經在算 `range_err_m` 了,
%      只是門檻(`gate_range_err_m`)預設 0 = 停用。
%
%   ⚠️ 論文的切換門檻 30 在我們手上的抽取文字裡**單位不明**(模擬表用 km,
%      但 30 km 對那個場景太大)⇒ 這裡只採用它的**形式**,門檻用
%      p.dgcgTol(預設 200 m)並且應該實測訂定,不照抄那個數字。

vhat = o.v / o.V;
aPN = p.N * o.V * cross(o.omega, vhat);

tol = 200;                              % [m] 共識容差,見上面的警告
Rbar = mean([o.peers.R]);
dR = o.R - Rbar;                        % > 0 = 我比平均遠 ⇒ 要抄近路/別繞

if abs(dR) < tol
    a = aPN;                            % 第二階段:純 PN,乾淨收尾
    s.phase = 2; s.dR = dR;
    return;
end

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

% 比平均近的(dR < 0)要繞路拖時間;比平均遠的不繞(它已經最慢)。
% ⚠️ 只能延後不能提前 —— 與 ITCG 同一個限制,所以 dR > 0 時不出力。
k = 0.5;
aMag = k * min(dR, 0) * (o.V^2 / max(o.R, 1)) / max(Rbar, 1);
aT = aMag * sideVec;
aT = aT - (aT.' * vhat) * vhat;

a = aPN + aT;
s.phase = 1; s.dR = dR;
end
