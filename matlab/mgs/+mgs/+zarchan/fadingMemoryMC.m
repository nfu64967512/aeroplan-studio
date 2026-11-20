function R = fadingMemoryMC(opts)
%MGS.ZARCHAN.FADINGMEMORYMC  Zarchan Listing 7.2 -- Monte Carlo sweep of the
%   homing loop with a two-state digital fading memory filter.
%
%   R = mgs.zarchan.fadingMemoryMC()
%   R = mgs.zarchan.fadingMemoryMC('runs', 10)        cheaper, wider mean-z
%   R = mgs.zarchan.fadingMemoryMC('BETA', 0.3, 'assert', false)
%
%   Line-by-line port of Listing 7.2 (zarchan.txt:2030-2051): the Monte Carlo
%   twin of Listing 7.1.  50 runs at each of 20 flight times (tF = 0.5:0.5:10),
%   1000 engagements, and for each flight time the sample MEAN and sample
%   STANDARD DEVIATION of the miss.
%
%   ⚠️ LINE-NUMBER CONVENTION, CORRECTED 2026-08-24 BY MEASUREMENT.  Every
%   citation below is now a grep/sed/Read line of zarchan.txt.  They used to be
%   MATLAB readlines() indices, +36 through Ch.7 (":2087" for the axis call
%   that Read shows at 2051).  The two book files share numbering EXACTLY --
%   15120 LF records each, same sentence at the same index in both -- so the
%   +36 was never a raw-vs-decoded offset: it is one file read two ways, since
%   grep/sed/Read split on LF (15120 lines) and readlines() splits on bare CR
%   as well (15308).  All 12 citations were re-verified against the text on
%   conversion.  Full note in kalmanLawOptions.m's header.
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, rad, s, g = 32.2 ft/s^2.  The
%   rest of mgs is metres; nothing is converted here, so every number printed
%   lies straight against a published one.  Convert AT THE CALL SITE.
% ===========================================================================
%
%   WHY THIS LISTING IS WORTH A PORT.  It is the book's cleanest demonstration
%   that a miss distribution SPLITS.  Two disturbances only: a 3-g target
%   maneuver, DETERMINISTIC, and 1 mr of LOS angle noise, zero-mean RANDOM.
%   The loop is linear, so (zarchan.txt:2028) the sample standard deviation of
%   the miss IS the noise-induced miss and the sample mean IS the
%   maneuver-induced miss -- and the chapter's headline conclusion is an
%   ORDERING between those two, not a number.  It splits ONLY because the loop
%   is linear, which is why the superposition self-check below is not
%   decoration: our own gamma channel SATURATES, and a saturating channel does
%   not decompose at all.
%
%   *** WHAT IT IS NOT. ***  Planar, linearised, constant closing velocity, a
%   PERFECT flight-control system (unity gain, no lag), no gravity, drag,
%   thrust, acceleration limit or LOS floor.  The only lag in the loop is the
%   filter itself.  Do not read an aircraft prediction out of it.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION, AND WHAT WAS DONE TO IT
%
%   matlab/zarchan/_listings/listing_07_02.m does not parse.  Its header
%   reports one unresolved item ("prose wedged into the code"); the comparison
%   damage is the usual OCR substitution and is resolved here, with reasons:
%
%     1  "while T ,= (TF - 1e-5)"  -> "while T <= (TF - 1e-5)".  "," is the
%        OCR of "<" throughout this book.  Shape-forced: T starts at 0 and only
%        increases, so ">=" never enters and "==" never fires.
%     2  "while STEP ,=1"          -> "while STEP <= 1".  Same substitution;
%        the body sets STEP = 2 to leave, which only parses against "<=".
%     3  "if S . =(TS - 1e-5)"     -> "if S >= (TS - 1e-5)".  "." is the OCR of
%        ">".  S accumulates H and is reset to 0 inside the branch; with "<="
%        the branch fires on EVERY step, running the digital filter at 100 Hz
%        instead of the stated 10 Hz and making the book's own rule "the ratio
%        Ts/H must be a large integer" (zarchan.txt:1990) pointless.  Benchmark
%        5 puts the resulting sample COUNT on the board, so the resolution is
%        visible rather than asserted.
%     4  "gaussc7(SIGNOISE)" is an unpublished helper.  Listing 7.1, the same
%        loop four pages earlier (zarchan.txt:2009), writes SIGNOISE*randn in
%        the same slot, and the text calls it "zero-mean Gaussian noise ...
%        with standard deviation SIGNOISE" (zarchan.txt:1969).  Resolved as
%        sigma*randn.  Drawn from an explicit RandStream, never the global one.
%     5  The paragraph trailing the transcription ("To generate the data of
%        Figs. 7.4 and 7.5 ...") is the book's next page leaking in; it ends at
%        zarchan.txt:2051.  Dropped, not "fixed" into code.
%
%   Kept deliberately, because they are the book's and they are load-bearing:
%   TGO carries +1e-5 so the last step cannot divide by zero; XNC is HELD
%   between sample instants (that zero-order hold IS the discrete part of the
%   loop); the postprocessor divides by (RUN-1) about the SAMPLE mean; and
%   XMEAN is assigned inside the run loop, so it is a running mean whose final
%   value -- at I = RUN -- is the one the book plots.  Not "tidied".
%   Recorded and dropped: "save datfil.txt output /ascii" (a typo for -ascii;
%   nothing is written to disk here) and the preallocations the loops overwrite.
%
%   ⚠️ MINUS-SIGN DAMAGE, class 3, present in this chapter and NOT auto-fixed:
%   "z21" is z^-1 and "Replace n by N 2 n" is N - n.  Neither is a value used
%   here, but nothing in Ch. 7 shaped like "2<digit>" is a numeral until the
%   surrounding prose says so.
%
% ---------------------------------------------------------------------------
%   WHAT IS SCOREABLE, AND WHAT IS NOT
%
%   Chapter 7 publishes almost no numbers for this listing.  Figs. 7.4 and 7.5
%   -- the sigma and mean curves -- became image links, so there is no
%   per-flight-time value to score and none is invented (one SKIP row says so).
%   Greppable: the listing's parameters, Table 7.1's gain formulas, the
%   run-count arithmetic, the ordering claim at zarchan.txt:2028, and the axis
%   limits the author set by hand.
%
%   *** IS AN AXIS LIMIT A MEASUREMENT?  ONE OF THESE TWO IS, ONE IS NOT, AND
%   THE DIFFERENCE WAS MEASURED, NOT ARGUED (2026-08-24). ***
%     The CEILING of axis([00,10,00,60]) is scored.  Sweeping b through the
%     port and reading the NOISE-FREE peak mean back out gives 46.52 ft at
%     b = 0.780, 57.62 ft at b = 0.800 and 60.65 ft at b = 0.805.  A ceiling of
%     60 that the book's own b fills to 96.0% therefore pins b to better than
%     1%, and with it Table 7.1's two-state gain row -- which is the live
%     transcription risk here (the first-order row is G = 1-b, the three-state
%     row G = 1-b^3).  The author sized that window to his own curve; the
%     number carries information and it is kept as a scored bound.
%     The FLOOR of the same axis is NOT scored.  It is reported instead, with
%     its measured number, as an INFO row -- see benchmark 9 for the four
%     measurements that took it off the board.  Short version: at b = 0.8 the
%     noise-free mean is POSITIVE at all 20 flight times, so the book's floor
%     is right about the physics; what dips below zero is the 50-run SAMPLE
%     mean, by -0.40 ft, which is 0.66% of a 60 ft frame and happened on 12 of
%     12 seeds tried.  That floor scores the random-number seed, not the port.
%     ⚠️ The Fig. 7.4 row below still carries "sigma >= 0" as a conjunct.  That
%     is deliberate and it is NOT the same thing: a sample standard deviation
%     cannot be negative, so there the floor is an identity that can never
%     change a verdict, not a claim that can fail on a seed.
%
%   See also MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 7.2's own case, zarchan.txt:2030 and :2035.
    opts.VC       (1,1) double {mustBeFinite, mustBePositive} = 4000    % ft/s
    opts.VM       (1,1) double {mustBeFinite, mustBePositive} = 3000    % ft/s
    opts.XNT      (1,1) double {mustBeFinite} = 96.6                    % ft/s^2
    opts.HEDEG    (1,1) double {mustBeFinite} = 0                       % deg
    opts.BETA     (1,1) double {mustBeFinite} = 0.8
    opts.XNP      (1,1) double {mustBeFinite, mustBePositive} = 3
    opts.SIGNOISE (1,1) double {mustBeFinite, mustBeNonnegative} = 0.001 % rad
    opts.TS       (1,1) double {mustBeFinite, mustBePositive} = 0.1     % s
    opts.H        (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s
    opts.runs     (1,1) double {mustBeFinite, mustBePositive} = 50
    opts.tf       (1,:) double {mustBeFinite, mustBePositive} = 0.5:0.5:10
    % Every engagement gets seed + 100000*itf + irun, so any single run is
    % reproducible on its own and the superposition check can re-fly one run
    % with the SAME noise stream.  itf <= 20 and runs < 100000 here, so the
    % (itf, irun) map is injective.
    opts.seed     (1,1) double {mustBeFinite, mustBeNonnegative} = 20260824
    % 4.5 SE, not 3: the mean-z check tests 20 flight times at once, so k = 3
    % would leave a correct port a standing ~5% chance of a false red.
    opts.ksig     (1,1) double {mustBeFinite, mustBePositive} = 4.5
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.assert    (1,1) logical = true
end

if opts.runs >= 1e5
    error('mgs:zarchan:fadingMemoryMC:runs', ...
        'runs must be < 1e5 or the per-engagement seed map stops being injective.');
end

R.units = 'feet, ft/s, ft/s^2, rad, s; g = 32.2 ft/s^2 (the book''s units, unconverted)';
R.p     = opts;
R.seed  = opts.seed;

% ---- the sweep the book runs: 50 runs x 20 flight times ---------------------
[R.sigma, R.mean, last] = mcSweep(opts, opts.SIGNOISE, opts.XNT, opts.runs, opts.seed);
R.tf            = opts.tf;
R.n_engagements = numel(opts.tf) * opts.runs;
R.last          = last;      % readback from the LAST engagement actually flown

% ---- noise OFF: the deterministic (maneuver-only) miss, and a delete test ---
% 2 runs is enough: with SIGNOISE = 0 the runs are bit-identical, so SIGMA must
% come out EXACTLY zero.  This sweep also supplies the deterministic miss the
% decomposition checks are measured against.  (The seed offset is cosmetic --
% at SIGNOISE = 0 drawNoise returns zeros whatever the stream.)
[sigDet, misDet] = mcSweep(opts, 0, opts.XNT, 2, opts.seed + 7000000);

R.det_mean  = misDet;
R.det_sigma = sigDet;

R.selfcheck = structuralChecks(opts, R, sigDet, misDet);

if opts.benchmark
    R.bench = benchmarks(opts, R);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();  ev = false;  R.pass = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false, so nothing was compared against the book.  R.pass = false means NOT CHECKED.';
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault and not a tolerance question.  ' ...
        'First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf('%d of %d evaluated book values are outside tolerance (first: %s, book %s, ours %s, band %s).', ...
        numel(bad), sum(ev), R.bench(bad(1)).what, R.bench(bad(1)).book_str, ...
        R.bench(bad(1)).ours_str, R.bench(bad(1)).band);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:fadingMemoryMC:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN, complete: the caller must be able to see WHICH side
        % of a comparison moved.  benchSuite runs every port with assert off.
        warning('mgs:zarchan:fadingMemoryMC:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [Y, G, Hf, nSamp] = engage(VC, VM, HEDEG, XNT, XNP, BETA, TS, TF, H, xnoise)
%ENGAGE  One flight.  Listing 7.2's inner two loops, transcribed.
%   ⚠️ Every constant arrives as an ARGUMENT.  MATLAB local functions cannot
%   see the caller's workspace, and a previous wave in this package lost a
%   whole run to exactly that assumption.
Y  = 0;                        % YIC
YD = -VM*HEDEG/57.3;           % YDIC; HEDEG = 0 in the book's case
T  = 0;  S = 0;
G   = 1 - BETA^2;              % Table 7.1, TWO-state row (zarchan.txt:1975)
Hf  = (1 - BETA)^2;            %                          (zarchan.txt:1976)
XLAMH = 0;  XLAMDH = 0;  XNC = 0;
YDD = 0;                       % written before it is read; set for safety only
nSamp = 0;

while T <= (TF - 1e-5)
    YOLD = Y;  YDOLD = YD;  STEP = 1;  FLAG = 0;
    while STEP <= 1                       % second-order Runge-Kutta (Heun)
        if FLAG == 1
            Y  = Y  + H*YD;
            YD = YD + H*YDD;
            T  = T  + H;
            STEP = 2;
        end
        TGO   = TF - T + 0.00001;
        RTM   = VC*TGO;
        XLAM  = Y/(VC*TGO);
        XLAMD = (RTM*YD + Y*VC)/(RTM^2); %#ok<NASGU> % 7.2 computes, 7.1 plots
        YDD   = XNT - XNC;               % perfect flight control: unity gain
        FLAG  = 1;
    end
    FLAG = 0; %#ok<NASGU> the book's own dead store; kept for fidelity
    Y  = 0.5*(YOLD  + Y  + H*YD);
    YD = 0.5*(YDOLD + YD + H*YDD);
    S  = S + H;
    if S >= (TS - 1e-5)                   % the sampler; see header item 3
        S = 0;
        nSamp = nSamp + 1;
        XLAMNOISE = xnoise(nSamp);        % pre-drawn: sigma*randn, header item 4
        RES    = XLAM - (XLAMH + TS*XLAMDH) + XLAMNOISE;
        XLAMH  = G*RES + XLAMH + TS*XLAMDH;
        XLAMDH = Hf*RES/TS + XLAMDH;
        XNC    = XNP*VC*XLAMDH;           % held until the next sample
    end
end
end


function [SIGMA, XMEAN, last] = mcSweep(opts, signoise, xnt, nRun, seedOff)
%MCSWEEP  The book's two outer loops plus its postprocessor, per flight time.
nTf   = numel(opts.tf);
SIGMA = zeros(1, nTf);  XMEAN = zeros(1, nTf);
last  = struct('G', NaN, 'Hf', NaN, 'XNT', xnt, 'nSamp', NaN, 'TF', NaN);
for itf = 1:nTf
    TF = opts.tf(itf);
    Z  = zeros(1, nRun);  Z1 = 0;  mu = 0;
    G  = NaN;  Hf = NaN;  nS = NaN;   % so the readback exists even for nRun = 0
    for I = 1:nRun
        xn = drawNoise(opts, signoise, TF, seedOff + 100000*itf + I);
        [Z(I), G, Hf, nS] = engage(opts.VC, opts.VM, opts.HEDEG, xnt, ...
            opts.XNP, opts.BETA, opts.TS, TF, opts.H, xn);
        Z1 = Z(I) + Z1;
        mu = Z1/I;                      % the book's running XMEAN; see header
    end
    sg = 0;  Z1 = 0;                    % the book's postprocessor, verbatim
    for I = 1:nRun
        Z1 = (Z(I) - mu)^2 + Z1;
        if I == 1, sg = 0; else, sg = sqrt(Z1/(I-1)); end
    end
    SIGMA(itf) = sg;  XMEAN(itf) = mu;
    last.G = G;  last.Hf = Hf;  last.nSamp = nS;  last.TF = TF;
end
end


function xn = drawNoise(opts, signoise, TF, seed)
%DRAWNOISE  One flight's measurement-noise stream, pre-drawn so that the same
%   stream can be re-flown with the maneuver deleted (the superposition check).
%   An explicit RandStream, so the caller's global rng state is never touched.
n  = ceil(TF/opts.TS) + 4;
rs = RandStream('mt19937ar', 'Seed', mod(seed, 2^32-1));
xn = signoise * randn(rs, 1, n);
end


% =============================================================================
function K = structuralChecks(opts, R, sigDet, misDet)
%STRUCTURALCHECKS  Invariants of the PORT, not values from the book.
%   ⚠️ .names and .oks are what put a structural row on the board.  A check
%   stored under any other field name is a check nobody can see.
names = {};  oks = [];  detail = {};

% 1. The book's own rule for a mixed continuous/discrete simulation.
ratio = opts.TS/opts.H;
names{end+1} = 'Ts/H is a large integer (book: zarchan.txt:1990)';
oks(end+1)   = abs(ratio - round(ratio)) < 1e-9 && ratio >= 10;
detail{end+1} = sprintf('Ts/H = %.6g', ratio);

% 2. DELETE EXPERIMENT.  Take the noise out; the runs become identical, so the
%    sample standard deviation must be EXACTLY zero.  If it is not, the "noise"
%    in this port has a second source and sigma is not the noise-induced miss.
names{end+1} = 'delete the noise => sigma is exactly 0 at every flight time';
oks(end+1)   = all(sigDet == 0);
detail{end+1} = sprintf('max sigma with SIGNOISE = 0: %.3g ft', max(sigDet));

% 3. SUPERPOSITION.  The loop is linear, so miss(maneuver + noise) must equal
%    miss(maneuver) + miss(noise) run on the SAME noise stream.  This is the
%    assumption the whole chapter's mean/sigma split rests on; without it the
%    decomposition benchmark below means nothing.
%    ⚠️ The response to the INITIAL CONDITION (heading error) appears in both
%    sub-runs, so it is subtracted once.  It is identically zero at the book's
%    HEDEG = 0; the term is here so the check stays correct if HEDEG is not.
sub  = unique(min(max(round([0.2 0.5 1.0]*numel(opts.tf)), 1), numel(opts.tf)));
resid = 0;  scale = 0;
for itf = sub
    TF = opts.tf(itf);
    z0 = drawNoise(opts, 0, TF, 0);          % an all-zero stream
    yIC = engage(opts.VC, opts.VM, opts.HEDEG, 0, opts.XNP, ...
        opts.BETA, opts.TS, TF, opts.H, z0);
    yDet = engage(opts.VC, opts.VM, opts.HEDEG, opts.XNT, opts.XNP, ...
        opts.BETA, opts.TS, TF, opts.H, z0);
    for I = 1:3
        xn = drawNoise(opts, opts.SIGNOISE, TF, opts.seed + 100000*itf + I);
        yFull = engage(opts.VC, opts.VM, opts.HEDEG, opts.XNT, opts.XNP, ...
            opts.BETA, opts.TS, TF, opts.H, xn);
        yNoi  = engage(opts.VC, opts.VM, opts.HEDEG, 0, opts.XNP, ...
            opts.BETA, opts.TS, TF, opts.H, xn);
        resid = max(resid, abs(yFull - (yDet + yNoi - yIC)));
        scale = max([scale, abs(yFull), abs(yDet), abs(yNoi)]);
    end
end
K.superpos_resid = resid;
names{end+1} = 'superposition: miss(maneuver+noise) = miss(maneuver) + miss(noise)';
oks(end+1)   = resid <= 1e-8*max(scale, 1);
detail{end+1} = sprintf('worst residual %.3g ft on a %.3g ft scale', resid, scale);

% 4. The chapter's claim made falsifiable: if the mean really is the
%    MANEUVER-induced miss, then the full-sweep mean must sit on the
%    noise-free miss to within the standard error of the mean.
se   = max(R.sigma/sqrt(opts.runs), realmin);
K.mean_z = max(abs(R.mean - misDet)./se);
names{end+1} = 'sample mean sits on the noise-free miss (noise is zero-mean)';
oks(end+1)   = K.mean_z <= opts.ksig;
detail{end+1} = sprintf('worst |mean - deterministic| = %.2f SE (limit %.1f SE)', ...
    K.mean_z, opts.ksig);

% 5. Seed discipline: the same (flight time, run index) must re-fly identically.
TFr = opts.tf(end);
xr  = drawNoise(opts, opts.SIGNOISE, TFr, opts.seed + 100000*numel(opts.tf) + 1);
y1  = engage(opts.VC, opts.VM, opts.HEDEG, opts.XNT, opts.XNP, opts.BETA, ...
    opts.TS, TFr, opts.H, xr);
xr2 = drawNoise(opts, opts.SIGNOISE, TFr, opts.seed + 100000*numel(opts.tf) + 1);
y2  = engage(opts.VC, opts.VM, opts.HEDEG, opts.XNT, opts.XNP, opts.BETA, ...
    opts.TS, TFr, opts.H, xr2);
names{end+1} = 'seeded re-flight of one run reproduces the miss bit-for-bit';
oks(end+1)   = isequal(y1, y2);
detail{end+1} = sprintf('tF = %g s, run 1: %.12g vs %.12g ft', TFr, y1, y2);

K.names = names;  K.oks = logical(oks);  K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    j = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{j}, detail{j});
end
end


% =============================================================================
function B = benchmarks(opts, R)
B = emptyBench();
S72 = 'Listing 7.2, zarchan.txt:2030-2051';

% 1. Two book locations, one quantity: the prose says "3-g", the code says 96.6.
B = addVal(B, 'target maneuver nT', 3*32.2, R.last.XNT, '3 g exactly', 1e-9, ...
    ['prose p.141 "a constant 3-g target maneuver" (zarchan.txt:2028) vs ' S72 ' XNT=96.6'], ...
    'ft/s^2', 'readback of the value the last engagement actually integrated');

% 2/3. Table 7.1 has THREE filter rows; picking the wrong one is the live
%      transcription risk (the first-order row is G = 1-b, not 1-b^2, and the
%      three-state row is G = 1-b^3).  The book side of these two rows is
%      Table 7.1's FORMULA written out here, independently of engage(); the
%      ours side is read back OUT of the flown loop.  Evaluated at the actual
%      b so the rows keep their meaning if b is swept.
b = opts.BETA;
B = addVal(B, 'fading memory gain G', 1 - b^2, R.last.G, 'exact', 1e-12, ...
    'Table 7.1 two-state row, zarchan.txt:1975 (G = 1-b^2); b = 0.8 from Listing 7.2, zarchan.txt:2035', ...
    '-', sprintf('b = %g here; the book''s nominal b = 0.8 gives G = 0.36', b));
B = addVal(B, 'fading memory gain H', (1 - b)^2, R.last.Hf, 'exact', 1e-12, ...
    'Table 7.1 two-state row, zarchan.txt:1976 (H = (1-b)^2)', ...
    '-', sprintf('b = %g here; the book''s nominal b = 0.8 gives H = 0.04', b));

% 4. The chapter's own arithmetic, counted rather than asserted.
B = addVal(B, 'engagements flown in the sweep', 1000, R.n_engagements, 'exact', 0, ...
    'prose "50-run Monte Carlo set ... 20 different values of flight time ... a total of 1000 runs!", zarchan.txt:2028', ...
    'runs', sprintf('%d runs x %d flight times, counted from the sweep', opts.runs, numel(opts.tf)));

% 5. Puts the "if S . =" resolution on the board: at Ts = 0.1 s a 10 s flight
%    must sample 100 times.  The rejected reading ("<=") gives 1000.
tfLast = R.last.TF;
B = addVal(B, sprintf('filter samples in the tF = %g s flight', tfLast), ...
    round(tfLast/opts.TS), R.last.nSamp, 'exact', 0, ...
    ['N = tF/Ts, zarchan.txt:2058 (prints as "N = tF=Ts": the fraction bar ' ...
     'also extracted as "="); Ts and the tF sweep from ' S72], ...
    'samples', sprintf(['counted inside the loop, not asserted; the discarded ' ...
    'OCR reading of the sampler test would give %g'], round(tfLast/opts.H)));

% 6. THE DECOMPOSITION ROW.  The chapter's headline claim is an ORDERING.
lessN = sum(R.sigma < abs(R.mean));
B = addShape(B, 'noise-induced miss is small vs maneuver-induced miss', ...
    'sigma < |mean| at MOST flight times', ...
    sprintf('%d of %d flight times', lessN, numel(R.tf)), ...
    lessN > numel(R.tf)/2, ...
    'prose p.141 "the noise-induced miss is small compared to the target-maneuver-induced miss for most flight times", zarchan.txt:2028', ...
    sprintf('median sigma %.3g ft vs median |mean| %.3g ft', median(R.sigma), median(abs(R.mean))));

% 7. Fig. 7.4's hand-set axis.  WEAKER EVIDENCE than a printed number -- an
%    axis bounds the curve only because the author would have noticed it
%    clipping.  Labelled 'bound', not 'value', for that reason.  The floor
%    here is an IDENTITY (a sample standard deviation cannot be negative), so
%    unlike Fig. 7.5's floor it can never decide a verdict; it is left in.
okS = all(R.sigma >= 0) && max(R.sigma) <= 4;
B = addBound(B, 'sigma curve fits the Fig. 7.4 axis', '0 <= sigma <= 4', ...
    sprintf('%.4g .. %.4g', min(R.sigma), max(R.sigma)), okS, ...
    'axis([00,10,00,4]) in Listing 7.2, zarchan.txt:2051; Fig. 7.4 caption zarchan.txt:2022', ...
    'ft', 'the figure itself did not survive extraction; only its axis command did');

% 8. Fig. 7.5's hand-set axis, CEILING ONLY.  The floor is benchmark 9, which
%    reports it rather than scoring it and says why.
%    This half stays scored because it was MEASURED to discriminate.  Read the
%    NOISE-FREE peak mean back out of this port at neighbouring b: 46.52 ft at
%    b = 0.780, 57.62 ft at b = 0.800, 60.65 ft at b = 0.805, 253.80 ft at
%    b = 0.900.  A ceiling of 60 filled to 96.0% by the book's own b therefore
%    excludes b >= 0.805 -- under 1% away -- and with it the wrong rows of
%    Table 7.1, which is the live transcription risk in benchmarks 2 and 3.
okM = max(R.mean) <= 60;
B = addBound(B, 'mean curve stays under the Fig. 7.5 axis ceiling', 'mean <= 60', ...
    sprintf('peak sample mean %.4g ft; noise-free peak %.4g ft = %.1f%% of the ceiling', ...
        max(R.mean), max(R.det_mean), 100*max(R.det_mean)/60), okM, ...
    'axis([00,10,00,60]) in Listing 7.2, zarchan.txt:2051; Fig. 7.5 caption zarchan.txt:2031', ...
    'ft', sprintf(['measured discrimination, not assertion: the noise-free peak ' ...
    'crosses this ceiling between b = 0.800 (%.2f ft) and b = 0.805 (60.65 ft), ' ...
    'so the author''s hand-typed 60 pins the filter bandwidth to better than 1%%'], ...
    max(R.det_mean)));

% 9. THE FLOOR OF THE SAME AXIS, DEMOTED -- reported, never scored.
%    An axis limit is a plotting decision; whether it is also a measurement has
%    to be tested, and this one failed the test four ways (all 2026-08-24, all
%    through this port):
%      a  At the book's b = 0.8 the NOISE-FREE mean is positive at every one of
%         the 20 flight times (minimum +0.0282 ft at tF = 10 s).  The book's
%         floor is right about the physics; nothing here disagrees with it.
%      b  What goes negative is the 50-run SAMPLE mean, and only in the tail
%         where the true mean has decayed to a few hundredths of a foot while
%         sigma is still ~1.9 ft.  *** CORRECTED 2026-08-24 BY RE-MEASUREMENT:
%         this line used to read "-0.23, -1.80 and -0.23 standard errors". ***
%         The three negative points are tF = 8, 9 and 10 s, and they sit
%         -1.82, -2.34 and -0.34 standard errors from their own noise-free
%         values (sample -0.0627/-0.3979/-0.0624 ft against noise-free
%         +0.4375/+0.1209/+0.0282 ft, SE = sigma/sqrt(50) =
%         0.2756/0.2214/0.2665 ft).  The worst is 2.34 SE, not 1.8 -- still
%         inside the noise for a MINIMUM taken over 20 flight times, which is
%         the statistic the row reports, but it is 2.34 and the file said 1.8.
%      c  It is not this seed: min(mean) < 0 on 12 of 12 seeds tried.
%         ⚠️ SEEDS MUST BE SPACED, and the original sweep was not.  Engagement
%         (itf, irun) draws seed + 100000*itf + irun, so two seeds closer than
%         ~2e6 SHARE most of their noise streams and are not independent
%         trials: seeds 1,2,3,7,99,555,8,12345 give a mixed 5-of-10 that says
%         nothing.  Re-measured 2026-08-24 on seeds spaced 1e7 apart -- 12 of
%         12 negative, values -0.3979 -0.0468 -0.0885 -0.1477 -0.4668 -0.2015
%         -0.1108 -0.1855 -0.0583 -0.2584 -0.0246 -0.1767 ft, mean -0.180 --
%         so the conclusion survives, on evidence that is actually 12 samples.
%      d  It is not fixable by running more, either.  50 -> 200 -> 1000 runs
%         gives -0.3979 -> -0.0727 -> -0.0649 ft, because the standard error
%         falls as 1.9/sqrt(N) while the quantity it has to clear is 0.028 ft:
%         ~4.5e3 runs PER flight time, against the book's 50.  The author's own
%         1000-run experiment could not have made this floor reliably true.
%    And the excursion is invisible: -0.40 ft on a 0..60 ft frame is 0.66% of
%    the frame height, thinner than the plotted line, so the author would never
%    have seen his own axis clip it.  Left on the board with its number so that
%    nobody re-adds it as a scored bound next year.
B = addInfo(B, 'Fig. 7.5 axis FLOOR (mean >= 0) -- reported, not scored', ...
    'axis([00,10,00,60]) asserts a floor of 0', ...
    sprintf('min sample mean %.4g ft; min noise-free mean %+.4g ft', ...
        min(R.mean), min(R.det_mean)), ...
    'axis([00,10,00,60]) in Listing 7.2, zarchan.txt:2051; Fig. 7.5 caption zarchan.txt:2031', ...
    'ft', sprintf(['an axis floor is not a measurement here: the noise-free ' ...
    'mean is positive at all %d flight times (min %+.4g ft), the negative ' ...
    'sample means are within 2.34 standard errors of it (measured -1.82, ' ...
    '-2.34, -0.34 at tF = 8, 9, 10 s), min(mean) < 0 on 12 of 12 ' ...
    'INDEPENDENT seeds (spaced 1e7; closer seeds share noise streams), and ' ...
    '50 -> 1000 runs only moves it -0.3979 -> -0.0649 ' ...
    'ft.  This bound scored the random-number seed.  The %+.4g ft excursion is ' ...
    '0.66%% of the 60 ft frame -- below the width of the plotted line.'], ...
    numel(R.tf), min(R.det_mean), min(R.mean)));

% 10. What cannot be scored, said out loud.
B = addSkip(B, 'sigma and mean at each individual flight time', 'Figs. 7.4 and 7.5', ...
    ['both curves became image links in extraction, so not one per-flight-time ' ...
     'value is published anywhere in the text.  Reading numbers off a figure ' ...
     'that no longer exists is how earlier waves fabricated benchmarks; the ' ...
     'shape and bound rows above are what the prose and the axis commands ' ...
     'genuinely support.'], ...
    'Fig. 7.4 zarchan.txt:2022, Fig. 7.5 zarchan.txt:2031', 'ft');
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER are the shared schema; every row is built
%   through mkRow so the order cannot drift between adders.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = mkRow(B, what, kind, book, book_str, ours, ours_str, pct, band, ...
                   tol, ok, skipped, why, source, units, detail)
i = numel(B) + 1;
B(i).what = what;          B(i).kind = kind;
B(i).book = book;          B(i).book_str = book_str;
B(i).ours = ours;          B(i).ours_str = ours_str;
B(i).pct  = pct;           B(i).band = band;
B(i).tol  = tol;           B(i).ok = logical(ok);
B(i).skipped = logical(skipped);  B(i).why = why;
B(i).source = source;      B(i).units = units;  B(i).detail = detail;
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
if ~isfinite(ours) || book == 0
    B = mkRow(B, what, 'value', book, sprintf('%.6g', book), ours, ...
        sprintf('%.6g', ours), NaN, band, tol, false, true, ...
        'the computed value is not finite (or the book value is zero)', ...
        source, units, detail);
    return
end
pct = 100*(ours/book - 1);
B = mkRow(B, what, 'value', book, sprintf('%.6g', book), ours, ...
    sprintf('%.6g', ours), pct, band, tol, abs(ours/book - 1) <= tol, ...
    false, '', source, units, detail);
end

function B = addShape(B, what, bookStr, oursStr, ok, source, detail)
B = mkRow(B, what, 'shape', NaN, bookStr, NaN, oursStr, NaN, 'shape', NaN, ...
    ok, false, '', source, '-', detail);
end

function B = addBound(B, what, bookStr, oursStr, ok, source, units, detail)
B = mkRow(B, what, 'bound', NaN, bookStr, NaN, oursStr, NaN, 'bound', NaN, ...
    ok, false, '', source, units, detail);
end

function B = addInfo(B, what, bookStr, oursStr, source, units, why)
%ADDINFO  REPORTED, NEVER SCORED -- and NOT the same thing as ADDSKIP.  A skip
%   is a check that could not run; an info row is a number that CAN be computed
%   and deliberately is not adjudicated, because the book side of it was
%   measured to carry no information.  skipped = true keeps it out of the pass
%   count in BOTH directions, so it can neither pass nor fail.
%   ⚠️ It exists so that a demotion is not a quiet deletion: the measured
%   number stays on the board, with the reason it stopped being a bound, so
%   nobody re-adds it as a scored bound next year.
B = mkRow(B, what, 'info', NaN, bookStr, NaN, oursStr, NaN, 'info', NaN, ...
    false, true, why, source, units, '');
end

function B = addSkip(B, what, bookStr, why, source, units)
%ADDSKIP  A check that COULD NOT RUN.  Never a pass: not the same thing.
B = mkRow(B, what, 'skip', NaN, bookStr, NaN, '--', NaN, 'skip', NaN, ...
    false, true, why, source, units, '');
end


% =============================================================================
function s = render(R, ev)
L = {};
L{end+1} = 'mgs.zarchan.fadingMemoryMC -- Zarchan Listing 7.2 (Monte Carlo, fading memory filter)';
L{end+1} = sprintf('  units: %s', R.units);
L{end+1} = sprintf('  b = %g, Ts = %g s, H = %g s, N'''' = %g, sigma_noise = %g rad, nT = %g ft/s^2', ...
    R.p.BETA, R.p.TS, R.p.H, R.p.XNP, R.p.SIGNOISE, R.p.XNT);
L{end+1} = sprintf('  %d runs x %d flight times = %d engagements, seed %d', ...
    R.p.runs, numel(R.tf), R.n_engagements, R.seed);
L{end+1} = '';
L{end+1} = '  tF(s)   sigma(ft)   mean(ft)   det-mean(ft)';
for i = 1:numel(R.tf)
    L{end+1} = sprintf('  %5.1f   %9.4g   %8.4g   %12.4g', ...
        R.tf(i), R.sigma(i), R.mean(i), R.det_mean(i)); %#ok<AGROW>
end
L{end+1} = '';
K = R.selfcheck;
for i = 1:numel(K.names)
    L{end+1} = sprintf('  [%s] %s -- %s', tick(K.oks(i)), K.names{i}, K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';
if isempty(R.bench)
    L{end+1} = '  (no benchmarks: ''benchmark'' was false)';
else
    for i = 1:numel(R.bench)
        b = R.bench(i);
        if b.skipped
            % An INFO row is a demotion, not a deletion, so its MEASURED value
            % has to print here too -- a row that shows only the book side is
            % exactly the quiet deletion this rendering is meant to prevent.
            L{end+1} = sprintf('  [%s] %-46s %s', upper(b.kind(1:min(4,end))), ...
                b.what, b.book_str); %#ok<AGROW>
            if ~strcmp(b.ours_str, '--')
                L{end+1} = sprintf('       ours %s', b.ours_str); %#ok<AGROW>
            end
            L{end+1} = sprintf('       %s', b.why); %#ok<AGROW>
        else
            L{end+1} = sprintf('  [%s] %-46s book %-12s ours %-12s (%s)', ...
                tick(b.ok), b.what, b.book_str, b.ours_str, b.band); %#ok<AGROW>
        end
    end
    L{end+1} = sprintf('  %d/%d structural, %d/%d book rows evaluated, %d not scored -- pass: %s', ...
        sum(K.oks), numel(K.oks), sum(ev & [R.bench.ok]), sum(ev), ...
        sum(~ev), tick(R.pass));
end
s = strjoin(L, newline);
end

function c = tick(tf)
if tf, c = 'ok'; else, c = 'XX'; end
end
