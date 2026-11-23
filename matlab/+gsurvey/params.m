function p = params(varargin)
%GSURVEY.PARAMS  測試台預設參數 —— 取自**我們自己的實測工作點**。
%
%   p = gsurvey.params()
%   p = gsurvey.params('n', 8, 'aMaxG', 2.0)
%
%   ⚠️ 這裡的數字不是拍的。來源逐項標在下面,絕大多數可以在
%      tests/test_mpng_value_locks.py:60-63 與 contract_table.py 的
%      JET_REFERENCE 找到同一組值。用實測工作點的用意是:
%      模擬出來的「這條律要多少 g」可以直接對上飛行資料的同一欄。

ip = inputParser; ip.KeepUnmatched = false;

% ── 幾何(放行當拍)────────────────────────────────────────────────
% R0/TGO0/V0/SIG0 = 6854 m / 41.3 s / 161.7 m/s / 15.3° 是 RRE 實測中位,
% 逐字同 tests/test_mpng_value_locks.py:63「這些不是拍的,是量的」。
ip.addParameter('n',        8);        % 機數(使用者要 6–8)
ip.addParameter('R0',    6854.0);      % 放行斜距 [m]
ip.addParameter('alt0',  3100.0);      % 放行高度 [m](= perch_alt 生產值)
ip.addParameter('V0',     161.7);      % 放行速率 [m/s]
ip.addParameter('sigma0',  15.3);      % 放行前置角 [deg]
ip.addParameter('fanHalf', 25.0);      % 扇形半角 [deg](= half_angle 生產值)
ip.addParameter('approach', 90.0);     % 進場方位 [deg,從北]
% ── 放行散度 ⚠️ 沒有散度 = 散布恆 0 = 每條時間律都假達標,見 scenario.m
ip.addParameter('R0spread', 500.0);    % 放行斜距全距 [m](實測 500 ⇒ 耗時差 3.3 s)
% ⚠️ 預設 0:使用者要看的是**軌跡形狀**,不是散布數字。放行時刻散度
%    會把八條軌跡在時間上錯開卻不改變形狀,徒增判讀負擔。要研究散布
%    再打開(ring_pace 之前實測 33.9 s、之後 5.6 s)。
ip.addParameter('tStagger',   0.0);    % 放行時刻全距 [s]

% ── 比例導引 ───────────────────────────────────────────────────────
% ⚠️ 參數名用 `Nnav` 不是 `N`:MATLAB 的 inputParser **不分大小寫**,
%    `'N'` 會與機數 `'n'` 撞名並在解析時就報錯。解析後再展開成 p.N,
%    律裡仍然寫 p.N(讀起來是導航常數該有的樣子)。
ip.addParameter('Nnav',     4.0);      % 導航常數(= pn_n_lat 生產值)
ip.addParameter('Nvert',    5.0);      % 縱向增益(我們兩通道不同增益)
ip.addParameter('Nsingle',  4.0);      % 單一增益律用

% ── 執行器 ─────────────────────────────────────────────────────────
% ⚠️ aMaxG 是**這個測試台的**橫向上限。生產坡度預算實測 1.94–2.73 g
%    且與 PN 共用(見 time-term-yes-lateral-no)。預設給 3.0 讓律先
%    跑在不飽和的狀態,再用 sweep 收緊看誰先垮。
ip.addParameter('aMaxG',    3.0);      % 指令加速度上限 [g]
ip.addParameter('tau_a',    0.0);      % 執行器一階遲滯 [s];0 = 理想
ip.addParameter('useGravity', true);   % 物理:重力是否作用
% ⚠️⚠️ `gravComp` 模擬「配平過的機體」:把重力**垂直於速度**的分量抵銷掉。
%    真機是配平的(θ = γ + α,升力平衡重力),所以導引律不必自己扛重力。
%    關掉它會讓所有不含重力補償項的律(PN/PPN/TPN…)一路下沉再拉回,
%    軌跡穿到地面以下 —— 那不是律的缺點,是少了配平。
%    ⭐ 把它關掉正是 **APN 的重力補償項**存在的理由,要展示 APN 的價值
%      就掃這一顆。預設 true = 公平比較「導引」本身。
ip.addParameter('gravComp', true);
ip.addParameter('ground',   true);     % 高度 ≤ 0 即觸地(海平面紅心)

% ── 積分 ───────────────────────────────────────────────────────────
ip.addParameter('dt',      0.02);      % 步長 [s]
ip.addParameter('tMax',    200.0);     % 逾時 [s]
ip.addParameter('hitR',     2.0);      % 判定命中的距離 [m]
% ── ⭐ 終端凍結 —— 生產值,見 tot_node._pn_step ────────────────────
% ⚠️⚠️ 沒有它,每一條含 V²/R 的律都會在 R→0 時發散:第一版實測
%    TPG aMax **703 g**、DGCG **1966 g**。那不是律的性質,是奇異點。
%    真機的作法是 t_go 或距離夠小就**鎖住 roll/pitch 指令**(但不鎖油門)。
ip.addParameter('fzR',    100.0);      % [m]   = pn_freeze_r 生產值
ip.addParameter('fzTgo',    1.5);      % [s]   = pn_freeze_tgo 生產值

% ── 速度通道(有動力機體,不是無動力滑翔)────────────────────────
% ⚠️ 我們**有動力**,而多數 ITCG 文獻是無動力滑翔 —— 那正是
%    ditcg-time-speed-exchange 記載「不可照抄」的原因。
ip.addParameter('speedHold', true);    % true = 速率固定在 V0
ip.addParameter('Vmin',      95.0);
ip.addParameter('Vmax',     210.0);    % = pn_vmax 生產值

% ── 時間控制律共用 ─────────────────────────────────────────────────
% td = 共同指定命中時刻。0 = 由 max_j(tgo_j) 自動取(我們的 dive_T 作法,
% 而且**必須取 max 不能取平均**,否則最慢那架永遠追不上)。
ip.addParameter('td',        0.0);
ip.addParameter('itcgGain',  1.0);

% ── 落角約束律共用 ─────────────────────────────────────────────────
% ⚠️ 落角約束律要求的角度**必須是幾何上到得了的**。從 R0=6854 m /
%    alt0=3100 m 放行,視線俯角只有 26.9° —— 要求 −60° 等於命令飛機
%    在紅心前 1.2 km 就插進地面(第一版實測 missH 1216 m、track 0.85)。
%    這與我們自己的實測一致:實得俯角 23.3–33.9°,天花板 ~36°。
%    預設取 −40°(比視線陡、但還在可達帶內)。
ip.addParameter('gammaF',  -40.0);     % 期望終端航跡角 [deg]

ip.parse(varargin{:});
p = ip.Results;
p.N = p.Nnav;                          % 律裡一律讀 p.N
p.g = 9.80665;
p.aMax = p.aMaxG * p.g;
p.sigma0 = deg2rad(p.sigma0);
p.fanHalf = deg2rad(p.fanHalf);
p.approach = deg2rad(p.approach);
p.gammaF = deg2rad(p.gammaF);
end
