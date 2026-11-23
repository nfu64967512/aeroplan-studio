function [a, s] = ppn(o, p, s)
%GSURVEY.LAWS.PPN  純比例導引(Pure Proportional Navigation)。
%
%   a = N · V_m · (ω × v̂)
%
%   指令加速度**嚴格垂直於本機瞬時速度向量** —— 這是 PPN 與 TPN 的
%   唯一分野,也是它適合定翼機的理由:垂直於速度的加速度由升力提供,
%   而升力正是舵面能給的東西;垂直於**視線**(TPN)則會要求一個沿軸向
%   的分量,定翼機沒有那個執行器。
%
%   ⚠️ 這就是我們現在真的在飛的那一條(tot_node.py:4501 `a_h = n_lat*vh*lam_dot`
%      用的是**本機速率** vh 而不是接近率 Vc ⇒ Pure PN,不是 True PN)。
%      ⚠️ 注意 tot_node.py:4499 的註解寫「橫向:真 PN」—— 那是中文的
%      「真正的 PN」不是術語 True PN,容易讀反。
%
%   參考:Zarchan《Tactical and Strategic Missile Guidance》Ch.30 對
%   「靜止目標 + 陡撞角」明文推薦 Pure PN。

vhat = o.v / o.V;
a = p.N * o.V * cross(o.omega, vhat);
end
