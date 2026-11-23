function [a, s] = apn(o, p, s)
%GSURVEY.LAWS.APN  增強比例導引(Augmented Proportional Navigation)。
%
%   a = N·V·(ω × v̂)  +  (N/2)·a_T⊥
%
%   在 PPN 之上加一個補償項,對抗**目標機動**或**已知的外加加速度**。
%   我們的紅心是靜止的,所以 a_T = 0;這裡把補償項用在**重力**上 ——
%   那是我們唯一已知的、持續作用的外加加速度。
%
%   ⭐ 這條的價值只有在 `gravComp = false` 時看得到:配平關掉之後,
%      PN/PPN 會一路下沉(第一版實測軌跡穿到地面下 117 m 再拉回),
%      而 APN 的 (N/2)·g⊥ 項正好把那個偏差前饋掉。
%      `gravComp = true`(預設)時它退化成 PPN,那本身就是一個結論:
%      **對一台配平過的定翼機,APN 的重力項是多餘的。**

vhat = o.v / o.V;
a = p.N * o.V * cross(o.omega, vhat);

if ~p.gravComp && p.useGravity
    gv = [0; 0; p.g];
    gperp = gv - (gv.' * vhat) * vhat;   % 重力垂直於速度的分量
    a = a - 0.5 * p.N * gperp;           % 前饋掉它(負號 = 抵銷)
end
end
