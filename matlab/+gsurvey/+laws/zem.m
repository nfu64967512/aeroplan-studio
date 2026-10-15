function [a, s] = zem(o, p, s)
%GSURVEY.LAWS.ZEM  零力脫靶量導引(Zero-Effort-Miss)。
%
%       ZEM = r + ṙ·t_go            (靜止目標、無外力時的「放手不管會差多少」)
%       a   = N · ZEM_⊥ / t_go²
%
%   這是現代軌跡整形的橋樑:把導引問題化成「把 ZEM 歸零」的最佳控制問題。
%   在等速、無遲滯的極限下它與 TPN **數學上等價**(N=3 時是最小控制能量解),
%   所以它在這個測試台上應該跑出與 TPN 幾乎重疊的軌跡 —— 那本身是一條
%   有用的自我檢查:如果它們差很多,是我的哪一支寫錯了。
%
%   ⭐ ZEM 形式的價值不在這裡的表現,在它**可擴充**:落角約束、
%      終端加速度約束、目標機動,都是往 ZEM 上加項,而不是重推一條律。

tgo = max(o.tgo, 1e-3);
rdot = o.vt - o.v;
ZEM = o.r + rdot * tgo;

% 只取垂直於視線的分量 —— 沿視線的那一份改變的是抵達時刻,不是脫靶量
ZEMperp = ZEM - (ZEM.' * o.los) * o.los;

a = p.N * ZEMperp / tgo^2;

% 定翼機只有升力:再投影掉沿速度的分量(否則等於要求推力調變)
vhat = o.v / o.V;
a = a - (a.' * vhat) * vhat;
end
