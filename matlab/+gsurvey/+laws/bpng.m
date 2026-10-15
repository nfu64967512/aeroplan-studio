function [a, s] = bpng(o, p, s)
%GSURVEY.LAWS.BPNG  偏置比例導引 —— **落角約束**(Impact Angle Control)。
%
%   縱向:a_γ = N·V·σ̇_LOS  +  bias,  bias 把終端航跡角逼向 gammaF
%   常見的閉式(Kim/Ryoo 型,靜止目標):
%
%       a = (N·V²/R)·[ (N−1)·λ̇·R/V  +  ... ]  的等價角度形式:
%       a_perp = (V²/R) · [ K1·(λ − γ) + K2·(γ_f − γ) ]
%       K1 = N,  K2 = N − 2      ← N ≥ 3 才有解
%
%   ⭐ **這條最貼我們的任務**:我們要的不只是命中,是「以夠陡的角度命中」——
%      殘餘高度被 1/tan(γ) 放大成縱向落點誤差,而那正是 residual-is-release-range
%      那張分箱表在講的事。純 PN 的終端角是**積出來的副產品**,不是被要求的量
%      (tot_node.py:4554-4557 明文記著「把 DIVE_ANGLE 改成 45 卻得不到 45°」)。
%
%   ⚠️ 我們現行的作法**不是** BPNG,是一個幾何否決(`pn_los_floor` 的
%      `min(γ_cmd, γ_los)`)。兩者的差別值得看:BPNG 是連續的偏置,
%      地板是離散的 min() —— 而後者會整個繞過 n_z 夾制(59.57% 的拍
%      由它決定指令,見 time-term-yes-lateral-no)。

vhat = o.v / o.V;

% 縱向平面內的視線傾角與航跡角(都以「向下為負」為正號慣例)
% ⚠️⚠️ 符號要與 o.gamma 同一個慣例(向上為正、俯衝為負)。
%    o.los(3) 在「目標在下方」時是**正**的(NED 的 D 向下),
%    所以 lamV = -atan2(los(3), 水平分量)。第一版寫成
%    -atan2(-los(3), ...) 是雙重負號 ⇒ 視線俯角變成 +26.8° ⇒
%    落角項把飛機**往地面推**,實測 missH 1133 m、落角 +20°(爬升)。
lamV = -atan2(o.los(3), hypot(o.los(1), o.los(2)));    % 視線俯角(俯衝為負)
gam  = o.gamma;                                        % 航跡角
gamF = p.gammaF;

% ⚠️⚠️ 第二項是 (λ − γ_f),**不是** (γ_f − γ)。
%    物理:要達到比視線**更陡**的落角,必須先把飛機拉到視線**上方**
%    (先抬頭)再插下來 —— 那個「過頂」才是落角約束律的特徵形狀。
%    寫成 (γ_f − γ) 會讓它一開始就往下壓 ⇒ 提早插地。
%    第一版實測:missH 295 m、落角 −14.9°(比純 PN 的 −26.8° 還淺),
%    而 track 0.968 < 1 就是「還沒飛到就落地」的指紋。
K1 = 4; K2 = 2;                        % 正典的 (4,2) 落角約束家族

% 縱向加速度量值(垂直於速度、在鉛垂平面內)
aV = (o.V^2 / max(o.R, 1)) * ( K1*(lamV - gam) + K2*(lamV - gamF) );

% 鉛垂平面內、垂直於速度的單位向量(向上為正)
up = [0; 0; -1];
uV = up - (up.' * vhat) * vhat;
if norm(uV) < 1e-9, uV = [0; 0; 0]; else, uV = uV / norm(uV); end

% 橫向仍走 PPN(BPNG 只管落角,不管方位)
aH = p.N * o.V * cross(o.omega, vhat);
aH = aH - (aH.' * uV) * uV;            % 把縱向那一份讓給 aV,避免重複計價

a = aV * uV + aH;
end
