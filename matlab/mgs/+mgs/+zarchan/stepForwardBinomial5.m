function R = stepForwardBinomial5(opts)
%MGS.ZARCHAN.STEPFORWARDBINOMIAL5  Zarchan Listing 19.5 -- the FORWARD model of
%   a step in target displacement through a FIFTH-ORDER BINOMIAL guidance
%   system WITH AN ACCELERATION LIMIT.  Twin of Listing 19.4, the adjoint.
%
%   R = mgs.zarchan.stepForwardBinomial5()
%   R = mgs.zarchan.stepForwardBinomial5('XNCLIM', 20*32.2)     % 20 g limit
%   R = mgs.zarchan.stepForwardBinomial5('benchmark',true,'render',false)
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, rad, g = 32.2 ft/s^2.  NOT SI.
%   Nothing here is converted, so every number printed lies straight against a
%   published one.  A miss of "12" is plausible in either unit, so a silent
%   conversion would be wrong by 3.28x and still look right.  CONVERT AT THE
%   CALL SITE, visibly.
% ===========================================================================
%
%   WHAT THE LISTING IS (zarchan.txt:5848-5865, p.433-435)
%
%   Two aircraft fly in formation; the seeker sees their power centroid until,
%   THOM seconds before intercept, it resolves aircraft 1 and the target
%   APPEARS to jump 200 ft.  Listing 19.1/19.2 fly that nonlinearly, 19.4 is
%   its adjoint through five equal lags of TAU/5, and 19.5 -- this file -- is
%   the forward twin of 19.4.  It is the ONLY member of the family carrying an
%   acceleration limit (XNCLIM), which an adjoint cannot represent at all.
%
%   The step enters as TWO simultaneous initial conditions, and both matter:
%       Y = Y + DISPLACE               the position jump
%       D = XLAM = Y/(VC*TGO)          the seeker filter pre-loaded to the new
%                                      sightline, so ELAMDH starts at zero
%   Listing 19.4's output TOT = XMY + XIC is exactly those two channels summed
%   (XMY = DISP*X3 for the Y jump, XIC = X4*DISP/(VC*TGO) for the D jump).
%   That correspondence is why Fig. 19.17 can claim the two agree, and it is
%   what benchmark row 7 measures.
%
%   HOW THE TRANSCRIPTION WAS RESOLVED.  Three comparison glyphs were damaged.
%     (a) while ~(T.(TF-.0001))  ->  T > TF-.0001.  '<' would make the loop
%         body unreachable (T = 0 at entry), so the direction is forced.
%     (b) if XNC.XNCLIM          ->  XNC > XNCLIM.  The very next line reads
%         `if XNC < -XNCLIM, XNC = -XNCLIM`, so this is the upper half of a
%         symmetric limiter; '<' would pin every command to the limit.
%         Self-check 4 is built to catch exactly that mistake.
%     (c) Listing 19.4's `if S,.09999` sampling guard is ambiguous -- '<'
%         records every step, '>=' records every 0.1 s.  This port records
%         EVERY step, a superset of both, so the ambiguity cannot reach any
%         published number.  The dense record is also what row 7 interpolates.
%   VM = 3000 and S = 0 are set by the listing and never read; they are kept in
%   R.p as dead constants rather than quietly dropped.
%
%   THE ONE-STEP-LATE STEP, WHICH IS THE LISTING'S AND IS KEPT.  TGO carries a
%   +1e-5 offset, so a step requested at TGO = THOM fires at the NEXT outer
%   step and the run's effective homing time is THOM - H + 1e-5.  At the book's
%   own TAU = 0.2 that is 5% of a guidance time constant -- several feet of
%   miss on the steep part of the curve.  Nothing is nudged to hide it:
%   R.tf_eff reports what each run actually flew, and row 7 reads the adjoint
%   at the EFFECTIVE homing time while also printing the naive number.
%
%   BANDS.  Rows 2-5 are values the book READ OFF Fig. 19.16 and printed as
%   prose.  The band is +/-0.02 in normalized miss -- the finest a printed
%   ordinate about 1.4 units tall supports -- carried onto each row as
%   0.02*DISPLACE/book.  It comes from the figure, not from our answer.
%
%   The disturbance is a target DISPLACEMENT, not our |eps0| handover error,
%   and the target does not manoeuvre (XNT = 0).  No seed: nothing is random.
%
%   OUTPUT
%     R.THOM R.miss R.norm R.tf_eff  the listing's own sweep, in book units
%     R.fig1917 .THOM .fwd .adj .tf_eff      forward vs Listing 19.4 adjoint
%     R.fig1918 .THOM .limit_g .miss .meanabs   the acceleration-limit family
%     R.bench R.selfcheck R.pass R.units R.summary
%
%   See also MGS.ZARCHAN.STEPDISPLACEMENT, MGS.ZARCHAN.STEPWITHSEEKER,
%            MGS.ZARCHAN.STEPADJOINTSINGLELAG.

arguments
    % Listing 19.5 line 1 (zarchan.txt:5850-5851), verbatim.
    opts.XNP      (1,1) double {mustBeFinite, mustBePositive}    = 3.0        % N'
    opts.TAU      (1,1) double {mustBeFinite, mustBePositive}    = 0.2        % s
    opts.TF       (1,1) double {mustBeFinite, mustBePositive}    = 10.0       % s
    opts.VC       (1,1) double {mustBeFinite, mustBePositive}    = 4000.0     % ft/s
    opts.XNT      (1,1) double {mustBeFinite, mustBeReal}        = 0.0        % ft/s^2
    opts.DISPLACE (1,1) double {mustBeFinite, mustBeReal}        = 200.0      % ft
    opts.XNCLIM   (1,1) double {mustBeFinite, mustBePositive}    = 99999999.0 % ft/s^2
    opts.H        (1,1) double {mustBeFinite, mustBePositive}    = 0.01       % s
    opts.THOM     (1,:) double {mustBeFinite, mustBePositive}    = 0.1:0.1:10 % s
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so an `if nargin == 0` mode here would never fire.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.assert    (1,1) logical = true
end

G = 32.2;

% ---- the listing itself, at whatever was asked for -----------------------
[R.miss, R.tf_eff] = sweepForward(opts.THOM, opts.XNP, opts.TAU, opts.XNCLIM, ...
                                  opts.DISPLACE, opts.VC, opts.XNT, opts.TF, opts.H);
R.THOM  = opts.THOM;
R.norm  = R.miss / opts.DISPLACE;
R.p     = opts;
R.p.VM  = 3000.0;         % set by the listing, never read.  Kept, not dropped.
R.p.G   = G;
R.units = 'feet, ft/s, ft/s^2, g = 32.2 ft/s^2 -- Zarchan''s units, NOT SI';

% ---- the two book figures, on BOOK-FIXED parameters ----------------------
% Fixed on purpose: a benchmark row must say the same thing about the book no
% matter which options this call carried.  Only the INFO row looks at opts.
if opts.benchmark
    BK        = bookRuns(G);
    R.fig1917 = BK.f1917;
    R.fig1918 = BK.f1918;
else
    BK        = [];
    R.fig1917 = struct([]);
    R.fig1918 = struct([]);
end

R.selfcheck = structuralChecks(G);

if opts.benchmark
    R.bench = benchmarks(BK, opts, G);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false(1,0);
    R.pass  = R.selfcheck.all_ok;
end

R.summary = render(R, ev);
if opts.render
    fprintf('%s\n', R.summary);
end

% assert = false must still RETURN a complete R.  Warn, do not error.
if ~R.pass
    msg = 'stepForwardBinomial5: something did not hold -- see R.bench / R.selfcheck.';
    if ~R.selfcheck.all_ok
        msg = sprintf('%s  First structural failure: %s', msg, R.selfcheck.first_bad);
    end
    if opts.assert
        error('mgs:zarchan:stepForwardBinomial5:failed', '%s', msg);
    else
        warning('mgs:zarchan:stepForwardBinomial5:failed', '%s', msg);
    end
end
end


% =============================================================================
function [miss, tfEff] = sweepForward(THOM, XNP, TAU, XNCLIM, DISPLACE, VC, XNT, TF, H)
%SWEEPFORWARD  Listing 19.5's outer `for THOM = 0.1:0.1:10` loop.
miss  = zeros(1, numel(THOM));
tfEff = zeros(1, numel(THOM));
for i = 1:numel(THOM)
    [miss(i), dg] = forwardRun(THOM(i), XNP, TAU, XNCLIM, DISPLACE, VC, XNT, TF, H);
    tfEff(i) = dg.tf_eff;
end
end


function [Y, dg] = forwardRun(THOM, XNP, TAU, XNCLIM, DISPLACE, VC, XNT, TF, H)
%FORWARDRUN  Listing 19.5, zarchan.txt:5850-5864, on the book's own Heun step.
%   Local function: it can see NOTHING of the caller, so every constant it uses
%   arrives as an argument.  The book's variable names are kept.
QSWITCH = 0;
Y = 0; YD = 0; XNL = 0; D = 0; ELAMDH = 0; X4 = 0; X5 = 0; T = 0;
YDD = 0; XNLD = 0; ELAMDHD = 0; DD = 0; X4D = 0; X5D = 0;
dg = struct('tf_eff', NaN, 'nc_raw_peak', 0, 'nc_clip_peak', 0, 'nc_min_abs', Inf);

while ~(T > (TF - 0.0001))
    TGO = TF - T + 0.00001;
    if TGO <= THOM && QSWITCH == 0
        QSWITCH   = 1;
        Y         = Y + DISPLACE;
        XLAM      = Y/(VC*TGO);
        D         = XLAM;
        dg.tf_eff = TGO;          % what this run ACTUALLY flew -- see header
    end
    YOLD = Y; YDOLD = YD; XNLOLD = XNL; DOLD = D;
    ELAMDHOLD = ELAMDH; X4OLD = X4; X5OLD = X5;
    STEP = 1; FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            STEP   = 2;
            Y      = Y   + H*YD;     YD  = YD  + H*YDD;   XNL = XNL + H*XNLD;
            ELAMDH = ELAMDH + H*ELAMDHD;                  D   = D   + H*DD;
            X4     = X4  + H*X4D;    X5  = X5  + H*X5D;   T   = T + H;
        end
        TGO     = TF - T + 0.00001;
        XLAM    = Y/(VC*TGO);
        DD      = 5.*(XLAM - D)/TAU;
        ELAMDHD = 5.*(DD - ELAMDH)/TAU;
        XNC     = XNP*VC*ELAMDH;
        XNCRAW  = XNC;                                  % pre-limiter, for dg
        if XNC >  XNCLIM, XNC =  XNCLIM; end            % '.' resolved to '>'
        if XNC < -XNCLIM, XNC = -XNCLIM; end
        X4D  = 5.*(XNC - X4)/TAU;
        X5D  = 5.*(X4 - X5)/TAU;
        XNLD = 5.*(X5 - XNL)/TAU;
        YDD  = XNT - XNL;
        FLAG = 1;
        if QSWITCH == 1
            dg.nc_raw_peak  = max(dg.nc_raw_peak,  abs(XNCRAW));
            dg.nc_clip_peak = max(dg.nc_clip_peak, abs(XNC));
            dg.nc_min_abs   = min(dg.nc_min_abs,   abs(XNC));
        end
    end
    FLAG   = 0; %#ok<NASGU>
    Y      = .5*(YOLD + Y + H*YD);
    YD     = .5*(YDOLD + YD + H*YDD);
    XNL    = .5*(XNLOLD + XNL + H*XNLD);
    D      = .5*(DOLD + D + H*DD);
    ELAMDH = .5*(ELAMDHOLD + ELAMDH + H*ELAMDHD);
    X4     = .5*(X4OLD + X4 + H*X4D);
    X5     = .5*(X5OLD + X5 + H*X5D);
end
if ~isfinite(dg.nc_min_abs), dg.nc_min_abs = NaN; end
end


function [TP, TOT] = adjointRun(XNP, TAU, TF, DISP, VC, H)
%ADJOINTRUN  Listing 19.4, zarchan.txt:5804-5812 -- the twin Fig. 19.17 claims
%   agreement with.  TOT(TP) is the miss for a homing time of TP seconds.
%   Recorded EVERY step (header note (c)), not every 0.1 s.
X2 = 0; X3 = 1; X4 = 0; X5 = 0; X6 = 0; X7 = 0; X8 = 0;
X2D = 0; X3D = 0; X4D = 0; X5D = 0; X6D = 0; X7D = 0; X8D = 0;
TP  = 0 + 0.00001;
n   = ceil((TF - 0.00001)/H) + 2;
TPa = zeros(1,n); TOTa = zeros(1,n); k = 0;
while ~(TP > (TF - 0.00001))
    X2OLD = X2; X3OLD = X3; X4OLD = X4; X5OLD = X5;
    X6OLD = X6; X7OLD = X7; X8OLD = X8;
    STEP = 1; FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            X2 = X2 + H*X2D; X3 = X3 + H*X3D; X4 = X4 + H*X4D; X5 = X5 + H*X5D;
            X6 = X6 + H*X6D; X7 = X7 + H*X7D; X8 = X8 + H*X8D; TP = TP + H;
        end
        X2D = X3;
        Y1  = 5.*(5.*X5/TAU + X4)/TAU;
        TGO = TP + 0.00001;
        X3D = Y1/(VC*TGO);
        X4D = -Y1;
        X5D = -5.*X5/TAU + 5.*X6*XNP*VC/TAU;
        X6D = -5.*X6/TAU + 5.*X7/TAU;
        X7D = -5.*X7/TAU + 5.*X8/TAU;
        X8D = -5.*X8/TAU - X2;
        FLAG = 1;
    end
    FLAG = 0; %#ok<NASGU>
    X2 = (X2OLD+X2)/2 + .5*H*X2D;   X3 = (X3OLD+X3)/2 + .5*H*X3D;
    X4 = (X4OLD+X4)/2 + .5*H*X4D;   X5 = (X5OLD+X5)/2 + .5*H*X5D;
    X6 = (X6OLD+X6)/2 + .5*H*X6D;   X7 = (X7OLD+X7)/2 + .5*H*X7D;
    X8 = (X8OLD+X8)/2 + .5*H*X8D;
    TGO     = TP + 0.00001;
    k       = k + 1;
    TPa(k)  = TP;
    TOTa(k) = DISP*X3 + X4*DISP/(VC*TGO);          % XMY + XIC
end
TP = TPa(1:k); TOT = TOTa(1:k);
end


% =============================================================================
function BK = bookRuns(G)
%BOOKRUNS  Every run a benchmark row scores.  BOOK-FIXED parameters only.
BK.DISPLACE = 200.0; BK.VC = 4000.0; BK.XNT = 0.0; BK.TF = 10.0;
BK.H = 0.01; BK.INF = 99999999.0;

% -- Fig. 19.16 reads (zarchan.txt:5838-5845).  T = 1 s at tF/T = 1; the book
%    then HALVES the time constant to reach tF/T = 2, so T = 0.5 s there.
BK.xnp = [3 4 5];
BK.miss_tf1 = zeros(1,3); BK.tfe_tf1 = zeros(1,3);
BK.miss_tf2 = zeros(1,3); BK.tfe_tf2 = zeros(1,3);
for k = 1:3
    [BK.miss_tf1(k), d1] = forwardRun(1.0, BK.xnp(k), 1.0, BK.INF, ...
        BK.DISPLACE, BK.VC, BK.XNT, BK.TF, BK.H);
    BK.tfe_tf1(k) = d1.tf_eff;
    [BK.miss_tf2(k), d2] = forwardRun(1.0, BK.xnp(k), 0.5, BK.INF, ...
        BK.DISPLACE, BK.VC, BK.XNT, BK.TF, BK.H);
    BK.tfe_tf2(k) = d2.tf_eff;
end

% -- Fig. 19.17: forward vs adjoint, N' = 3, T = 0.2 s, 200 ft (:5866).
%    Run at H = TAU/40, not the listing's 0.01: the book's claim is about the
%    two MODELS, so both are taken far enough toward convergence that what is
%    left is structural disagreement and not two different RK2 truncations.
TAUn = 0.2;  Hc = TAUn/40;  th = 0.2:0.2:5;
[mf, tfe]   = sweepForward(th, 3.0, TAUn, BK.INF, BK.DISPLACE, BK.VC, BK.XNT, BK.TF, Hc);
[tpA, totA] = adjointRun(3.0, TAUn, BK.TF, BK.DISPLACE, BK.VC, Hc);
BK.f1917 = struct('THOM', th, 'fwd', mf, 'tf_eff', tfe, 'H', Hc, ...
                  'adj',       interp1(tpA, totA, tfe, 'linear'), ...
                  'adj_naive', interp1(tpA, totA, th,  'linear'));

% -- Fig. 19.18: the acceleration-limit family at T = 0.2 s (:5868).
lim_g = [Inf 20 10 5 1];
mm    = zeros(numel(lim_g), numel(th));
mabs  = zeros(1, numel(lim_g));
for j = 1:numel(lim_g)
    if isinf(lim_g(j)), lim = BK.INF; else, lim = lim_g(j)*G; end
    mm(j,:) = sweepForward(th, 3.0, TAUn, lim, BK.DISPLACE, BK.VC, BK.XNT, BK.TF, BK.H);
    mabs(j) = mean(abs(mm(j,:)));
end
BK.f1918 = struct('THOM', th, 'limit_g', lim_g, 'miss', mm, 'meanabs', mabs, ...
                  'TAU', TAUn);
end


% =============================================================================
function B = benchmarks(BK, opts, G)
%BENCHMARKS  Nine rows: four scored values, two scored shapes, three INFO.
B      = emptyBench();
figTol = 0.02;                    % normalized readout resolution of Fig. 19.16
yT     = BK.DISPLACE;

% ---- 1  INFO: which run the book is describing --------------------------
B = addInfo(B, 'nominal parameters of Listing 19.5', ...
    'yT = 200 ft, T = 0.2 s, N'' = 3, acceleration limit infinite', ...
    sprintf('this call: yT = %g ft, T = %g s, N'' = %g, XNCLIM = %.4g ft/s^2 (%.4g g)', ...
            opts.DISPLACE, opts.TAU, opts.XNP, opts.XNCLIM, opts.XNCLIM/G), ...
    'p.433 prose + Listing 19.5 line 1 (zarchan.txt:5848, 5850)', ...
    ['configuration, never scored.  The scored rows below run the BOOK''s ' ...
     'parameters whatever this call asked for, so the board says the same ' ...
     'thing about Zarchan every time.']);

% ---- 2-4  Fig. 19.16 reads at tF/T = 1 ----------------------------------
bk1 = [162 150 137];  cf1 = [0.811 0.748 0.686];
for k = 1:3
    B = addVal(B, sprintf('miss at tF/T = 1, N'' = %d', BK.xnp(k)), ...
        bk1(k), abs(BK.miss_tf1(k)), ...
        sprintf('+/-%.2f normalized = +/-%.1f ft', figTol, figTol*yT), ...
        figTol*yT/bk1(k), ...
        sprintf('p.432 prose read off Fig. 19.16 (zarchan.txt:%d)', 5838+k), ...
        'ft', sprintf(['book %.4g x 200; T = 1 s, tF requested 1 s and flown ' ...
                       '%.5g s; signed value %+.5g ft'], cf1(k), ...
                       BK.tfe_tf1(k), BK.miss_tf1(k)));
end

% ---- 5  Fig. 19.16 read at tF/T = 2, N' = 3 -----------------------------
B = addVal(B, 'miss at tF/T = 2, N'' = 3', 42, abs(BK.miss_tf2(1)), ...
    sprintf('+/-%.2f normalized = +/-%.1f ft', figTol, figTol*yT), ...
    figTol*yT/42, 'p.432 prose read off Fig. 19.16 (zarchan.txt:5843)', 'ft', ...
    sprintf(['book 0.212 x 200, reached by HALVING T to 0.5 s (:5842); flown ' ...
             'tF = %.5g s; signed value %+.5g ft.  MAGNITUDE is scored: ' ...
             'Fig. 19.16''s ordinate runs down to -0.4 and the prose prints ' ...
             'these unsigned, so the sign is not recoverable from the text.'], ...
             BK.tfe_tf2(1), BK.miss_tf2(1)));

% ---- 6  INFO: the two reads that cannot carry a band --------------------
B = addInfo(B, 'the other two reads at tF/T = 2', ...
    'book 0.0073 x 200 = 1.5 ft (N''=4) and 0.206 x 200 = 41 ft (N''=5)', ...
    sprintf('this port gives %+.4g ft and %+.4g ft', BK.miss_tf2(2), BK.miss_tf2(3)), ...
    'p.432 prose read off Fig. 19.16 (zarchan.txt:5844-5845)', ...
    ['not scored.  0.0073 is two significant figures read off a printed curve ' ...
     'about 1.4 units tall -- an order of magnitude finer than the +/-0.02 ' ...
     'band rows 2-5 carry, so any band that could contain it would contain ' ...
     'nearly everything.  Reported so row 5''s two neighbours stay visible.']);

% ---- 7  Fig. 19.17: forward and adjoint agree ---------------------------
F     = BK.f1917;
dev   = max(abs(F.fwd - F.adj));
rmsd  = sqrt(mean((F.fwd - F.adj).^2));
naive = max(abs(F.fwd - F.adj_naive));
lim7  = figTol*yT;
B = addShape(B, 'forward (19.5) and adjoint (19.4) agree', ...
    sprintf('identical curves; scored as max |fwd - adj| <= %.1f ft', lim7), ...
    sprintf('max %.4g ft, rms %.4g ft (naive abscissa %.4g ft); peak miss %.4g ft', ...
            dev, rmsd, naive, max(abs(F.fwd))), ...
    dev <= lim7, 'Fig. 19.17, p.435 prose (zarchan.txt:5866)', ...
    sprintf(['N'' = 3, T = 0.2 s, 200 ft, tF = 0.2:0.2:5 s, both models at ' ...
             'H = %g s.  The adjoint is read at each run''s EFFECTIVE homing ' ...
             'time (THOM - H + 1e-5); the naive number is the same comparison ' ...
             'against THOM, and the gap between the two is the listing''s ' ...
             'one-step-late step, not a disagreement between the models.'], F.H));

% ---- 8  Fig. 19.18: tighter limit, worse miss ---------------------------
L   = BK.f1918;
ok8 = all(diff(L.meanabs) >= -1e-9);
B = addShape(B, 'miss grows as the acceleration limit tightens', ...
    'mean |miss| non-decreasing across limits inf, 20 g, 10 g, 5 g, 1 g', ...
    sprintf('%.4g, %.4g, %.4g, %.4g, %.4g ft', L.meanabs(1), L.meanabs(2), ...
            L.meanabs(3), L.meanabs(4), L.meanabs(5)), ...
    ok8, 'Fig. 19.18, p.436 prose (zarchan.txt:5868)', ...
    ['the mean over tF = 0.2:0.2:5 s at T = 0.2 s, not a single point: the ' ...
     'book''s claim is that the CURVE degrades, and saturation need not hurt ' ...
     'at every individual homing time.  This is the only row that exercises ' ...
     'XNCLIM, which is the whole reason Listing 19.5 exists next to 19.4.']);

% ---- 9  INFO: the chapter's headline claim ------------------------------
x  = L.THOM/L.TAU;
nb = abs(L.miss(1,:))/yT;
B = addInfo(B, '"tF/T > 8 for negligible miss"', ...
    'fifth-order needs more than EIGHT guidance time constants (three sufficed at first order)', ...
    sprintf('max |miss|/yT is %.4g for tF/T > 8 and %.4g for tF/T < 8', ...
            max([nb(x > 8) 0]), max([nb(x < 8) 0])), ...
    'p.430 prose read off Fig. 19.16 (zarchan.txt:5815)', ...
    ['"negligible" is an adjective, so this is reported and not adjudicated ' ...
     '-- the same treatment stepAdjointSingleLag gives the first-order "> 3" ' ...
     'claim.  Infinite-limit curve, T = 0.2 s.']);
end


% =============================================================================
function K = structuralChecks(G)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.  They are the weaker
%   claim that it agrees with itself, which is what catches wiring faults.
names = {}; oks = []; detail = {};
INF = 99999999.0;

% 1  DELETE EXPERIMENT on the forcing.  With no displacement the loop has no
%    input at all (XNT = 0 too), so the miss must be identically zero.  Any
%    stray drive -- an initial condition left in a state, a sign flip that
%    lets the seeker self-excite -- lights here and nowhere else cheap.
m0 = sweepForward([0.5 1 2], 3.0, 0.2, INF, 0.0, 4000.0, 0.0, 10.0, 0.01);
names{end+1}  = 'no displacement, no miss (delete the forcing)';
oks(end+1)    = max(abs(m0)) <= 1e-12;
detail{end+1} = sprintf('max |miss| = %.3e ft over tF = 0.5, 1, 2 s', max(abs(m0)));

% 2  With the limiter out of reach the loop is LINEAR, so doubling the step
%    must double the miss exactly.  This is what would notice a limiter that
%    is quietly active in the "infinite" case, or a nonlinearity that crept
%    into the sightline.
mA = forwardRun(1.0, 3.0, 1.0, INF, 200.0, 4000.0, 0.0, 10.0, 0.01);
mB = forwardRun(1.0, 3.0, 1.0, INF, 400.0, 4000.0, 0.0, 10.0, 0.01);
r2 = mB/mA;
names{end+1}  = 'miss linear in DISPLACE when the limit is out of reach';
oks(end+1)    = abs(r2 - 2) <= 1e-12;
detail{end+1} = sprintf('miss(400)/miss(200) = %.12g (%.4g and %.4g ft)', r2, mA, mB);

% 3  VC CANCELS.  XLAM = Y/(VC*TGO) divides by it and XNC = XNP*VC*ELAMDH
%    multiplies it straight back, so the miss cannot depend on closing
%    velocity -- and nothing else in this file would notice if it started to.
%    1e-9 and not 1e-15: the cancellation is algebraic, not bitwise, and the
%    two runs take different rounding paths through a divide by TGO -> 1e-5.
mC = forwardRun(1.0, 3.0, 1.0, INF, 200.0, 12000.0, 0.0, 10.0, 0.01);
r3 = abs(mC - mA)/max(abs(mA), realmin);
names{end+1}  = 'miss independent of VC (VC cancels through the loop)';
oks(end+1)    = r3 <= 1e-9;
detail{end+1} = sprintf('|dmiss|/|miss| = %.3e between VC = 4000 and 12000 ft/s', r3);

% 4  THE LIMITER'S DIRECTION.  `if XNC.XNCLIM` arrived damaged and was read as
%    '>'.  Read as '<' instead, every command BELOW the limit would be forced
%    UP to it -- the clipped peak would still sit on XNCLIM and look right, so
%    the peak alone does not test this.  The MINIMUM does: with a 1 g limit
%    the command starts at exactly zero (ELAMDH = 0 at the step), and the
%    flipped comparison would drag that zero up to 32.2 ft/s^2.
lim1g    = 1*G;
[m4, d4] = forwardRun(1.0, 3.0, 0.2, lim1g, 200.0, 4000.0, 0.0, 10.0, 0.01);
ok4 = d4.nc_raw_peak > lim1g && abs(d4.nc_clip_peak - lim1g) <= 1e-9 ...
      && d4.nc_min_abs < lim1g - 1e-9;
names{end+1}  = 'limiter clips AT the limit and passes small commands through';
oks(end+1)    = ok4;
detail{end+1} = sprintf(['demand peak %.4g, clipped peak %.4g (limit %.4g), ' ...
    'min |cmd| %.4g ft/s^2'], d4.nc_raw_peak, d4.nc_clip_peak, lim1g, d4.nc_min_abs);

% 5  ... and the clipping must CHANGE THE ANSWER.  A limiter that is wired,
%    binds, and still leaves the miss untouched is a limiter feeding nothing.
m4inf = forwardRun(1.0, 3.0, 0.2, INF, 200.0, 4000.0, 0.0, 10.0, 0.01);
d5    = abs(m4 - m4inf);
names{end+1}  = 'clipping changes the miss (XNCLIM is not dead code)';
oks(end+1)    = d5 > 1.0;
detail{end+1} = sprintf('1 g limit gives %.4g ft vs %.4g ft unlimited (d = %.4g ft)', ...
                        m4, m4inf, d5);

K.names  = names;
K.oks    = logical(oks);
K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    j = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{j}, detail{j});
end
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER must match the adders exactly or the
%   B(i) = ... appends throw -- which is the behaviour we want.  This is the
%   schema mgs.zarchan.benchSuite's adaptGeneric consumes; a near miss errors
%   there rather than being accommodated.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.4g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.5g', ours);
B(i).band = band;  B(i).tol = tol;
B(i).units = units;  B(i).source = source;  B(i).detail = detail;
if ~isfinite(ours) || book == 0
    B(i).pct = NaN;  B(i).ok = false;  B(i).skipped = true;
    B(i).why = 'the computed value is not finite (or the book value is zero)';
    return
end
B(i).pct = 100*(ours/book - 1);
B(i).ok  = abs(ours/book - 1) <= tol;
B(i).skipped = false;  B(i).why = '';
end

function B = addShape(B, what, bookStr, oursStr, ok, source, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'shape';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'shape';  B(i).tol = NaN;
B(i).ok   = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = '-';  B(i).detail = detail;
end

function B = addInfo(B, what, bookStr, oursStr, source, why)
%ADDINFO  Reported, never scored: it can neither pass nor fail.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'info';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'info';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = '-';  B(i).detail = '';
end


% =============================================================================
function s = render(R, ev)
L = {};
L{end+1} = 'Zarchan Listing 19.5 -- FORWARD MODEL, STEP IN TARGET DISPLACEMENT,';
L{end+1} = '   FIFTH-ORDER BINOMIAL GUIDANCE WITH AN ACCELERATION LIMIT';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['   yT = %g ft   N'' = %g   TAU = %g s   TF = %g s   ' ...
    'H = %g s   XNCLIM = %.4g g'], R.p.DISPLACE, R.p.XNP, R.p.TAU, R.p.TF, ...
    R.p.H, R.p.XNCLIM/R.p.G);
[pk, ipk] = max(abs(R.miss));
L{end+1} = sprintf(['   sweep: %d homing times, %g to %g s; peak |miss| ' ...
    '%.4g ft at tF = %g s'], numel(R.THOM), R.THOM(1), R.THOM(end), pk, R.THOM(ipk));
L{end+1} = '';
if isempty(R.bench)
    L{end+1} = '   benchmarks: not run (benchmark = false)';
else
    L{end+1} = '   BENCHMARKS (book vs this port)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
            case 'value'
                L{end+1} = sprintf('   %-4s %-38s book %-6s ours %-9s %+7.2f%%  %s', ...
                    vstr(b.ok), b.what, b.book_str, b.ours_str, b.pct, b.band); %#ok<AGROW>
            case {'shape','bound'}
                L{end+1} = sprintf('   %-4s %-38s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf('   %-4s %-38s %s', 'info', b.what, b.ours_str); %#ok<AGROW>
        end
        L{end+1} = sprintf('        source: %s', b.source); %#ok<AGROW>
    end
end
L{end+1} = '';
L{end+1} = '   STRUCTURAL (self-consistency only -- NOT evidence against the book)';
for i = 1:numel(R.selfcheck.names)
    L{end+1} = sprintf('   %-4s %-52s %s', vstr(R.selfcheck.oks(i)), ...
        R.selfcheck.names{i}, R.selfcheck.detail{i}); %#ok<AGROW>
end
L{end+1} = '';
if isempty(R.bench)
    L{end+1} = sprintf('   %d structural checks, %d passed.   OVERALL %s', ...
        numel(R.selfcheck.oks), sum(R.selfcheck.oks), vstr(R.pass));
else
    L{end+1} = sprintf(['   %d scored rows, %d passed; %d reported unscored; ' ...
        '%d structural, %d passed.   OVERALL %s'], sum(ev), ...
        sum(ev & [R.bench.ok]), sum(~ev), numel(R.selfcheck.oks), ...
        sum(R.selfcheck.oks), vstr(R.pass));
end
s = strjoin(L, newline);
end

function v = vstr(ok)
if ok, v = 'ok'; else, v = 'FAIL'; end
end
