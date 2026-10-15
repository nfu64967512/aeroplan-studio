function R = runAll(varargin)
%GSURVEY.RUNALL  跑一組導引律,產比較表 + 軌跡圖。
%
%   R = gsurvey.runAll()
%   R = gsurvey.runAll('n', 6, 'outDir', 'out/gsurvey')
%
%   ⭐ 選進來的**只有對我們這台機體有意義**的律。挑掉的與理由:
%       RPN / TCPN        尾追導引;我們的紅心靜止
%       Beam Rider        沒有波束
%       Weave Guidance    打蛇形機動目標;我們的目標不動
%       WTG / NOG         滑模 / 最佳控制,需要我們沒有的狀態回授頻寬
%       PINN-PCG / DD-ITCG 需要神經網路或離線資料庫,解的是我們沒有的問題
%       IVG / LHG / PHG   與 PPN 在靜止目標上退化成同一件事
%
%   輸出:R.law(k).name .out .kpi,以及 outDir 底下的 PNG。

ip = inputParser;
ip.addParameter('n', 8);
ip.addParameter('outDir', fullfile(gsurveyRoot(), 'out', 'gsurvey'));
ip.KeepUnmatched = true;
ip.parse(varargin{:});
extra = namedargs2cell(ip.Unmatched);

p   = gsurvey.params('n', ip.Results.n, extra{:});
scn = gsurvey.scenario(p);
outDir = ip.Results.outDir;
if ~isfolder(outDir), mkdir(outDir); end

LAWS = {
  'Pure Pursuit',  @gsurvey.laws.pursuit, '無前置量的對照組'
  'PPN',           @gsurvey.laws.ppn,     '★ 我們現在飛的'
  'TPN',           @gsurvey.laws.tpn,     '垂直於視線,定翼機做不出軸向分量'
  'APN',           @gsurvey.laws.apn,     '加重力/目標機動補償'
  'ZEM',           @gsurvey.laws.zem,     '零力脫靶量;N=3 時是最小能量解'
  'BPNG',          @gsurvey.laws.bpng,    '★ 落角約束 —— 我們真正缺的'
  'TPG(多項式)',  @gsurvey.laws.tpg,     '落角 + 終端加速度趨零'
  'ITCG',          @gsurvey.laws.itcg,    '★ 撞擊時間控制(橫向偏置)'
  'MPNG',          @gsurvey.laws.mpng,    '★ Jiang 2021,無奇異點時間控制'
  'DGCG',          @gsurvey.laws.dgcg,    '★ 兩階段:距離共識 → 純 PN'
};

fprintf('\n%-14s %8s %9s %8s %8s %8s %7s %7s  %s\n', ...
    'law', 'spread', 'missH', 'gamF', 'aMed', 'aMax', 'sat%', 'track', 'note');
fprintf('%s\n', repmat('-', 1, 96));

for k = 1:size(LAWS, 1)
    out = gsurvey.simulate(LAWS{k, 2}, scn, p);
    v   = out.veh;
    ts  = [v.tImpact];
    kpi = struct( ...
        'name',    LAWS{k, 1}, ...
        'spread',  max(ts) - min(ts), ...
        'missH',   median([v.missH]), ...
        'missHmax',max([v.missH]), ...
        'gamF',    rad2deg(median([v.gammaF])), ...
        'aMed',    median([v.aMed]) / p.g, ...
        'aMax',    max([v.aMax]) / p.g, ...
        'sat',     100 * mean([v.satFrac]), ...
        'track',   median([v.track]), ...
        'Vf',      median([v.Vf]));
    fprintf('%-14s %8.3f %9.2f %8.1f %8.2f %8.2f %7.1f %7.3f  %s\n', ...
        kpi.name, kpi.spread, kpi.missH, kpi.gamF, kpi.aMed, kpi.aMax, ...
        kpi.sat, kpi.track, LAWS{k, 3});
    R.law(k).name = LAWS{k, 1};
    R.law(k).note = LAWS{k, 3};
    R.law(k).out  = out;
    R.law(k).kpi  = kpi;
end
R.p = p; R.scn = scn; R.outDir = outDir;

fprintf('\n單位:spread [s] · missH [m 水平] · gamF [deg 落角] · aMed/aMax [g] · track [航程/直線]\n');

gsurvey.plotRuns(R);
fprintf('圖已寫到 %s\n', outDir);
end

function r = gsurveyRoot()
r = fileparts(fileparts(mfilename('fullpath')));
end
