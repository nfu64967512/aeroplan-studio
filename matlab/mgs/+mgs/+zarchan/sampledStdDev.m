function R = sampledStdDev(opts)
%MGS.ZARCHAN.SAMPLEDSTDDEV  Zarchan Listing 4.3 -- the sampled standard
%   deviation "the hard way", as a function of sample size.
%
%   R = mgs.zarchan.sampledStdDev()
%   R = mgs.zarchan.sampledStdDev('seed', 7, 'assert', false)
%   R = mgs.zarchan.sampledStdDev('nbig', 0)      % the listing's 100 draws only
%
%   Line-by-line port of Listing 4.3 (zarchan.txt:1077-1089).  100 Gaussian
%   draws are built the book's way -- twelve uniforms summed, 6 subtracted
%   (Listing 4.1, zarchan.txt:1012-1016) -- and the standard deviation is
%   recomputed after each additional sample.  The CURVE is the point: Fig. 4.7
%   is the book showing that a statistic from a finite sample is itself a
%   random variable.  It is also where our "50 runs" habit comes from
%   (zarchan.txt:1095), so the price of that habit is an INFO row below.
%
%   UNITS: THERE ARE NONE.  Z, XMEAN and SIGMA are pure numbers.  Chapter 4's
%   ft/s^2 quantities are in Listings 4.5/4.6, not here; nothing in this file
%   may be quoted in feet.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_04_03.m reports NO unresolved variable-to-
%   variable comparisons and that is correct -- its only comparison is
%   "if I == 1", against a literal.  Three things were done to it:
%
%     1  The trailing figure/plot/save block (zarchan.txt:1089) is dropped.
%        "save datfil.txt output -ascii" is spelt CORRECTLY here, unlike four
%        other listings (4.6 at :1337 has "/ascii"); recorded, not written.
%     2  ArrayI/ArraySIGMA are preallocated.  No value changes.
%     3  rand(1) IS REPLACED.  The file's one real deviation -- see below.
%
%   *** THE LOAD-BEARING QUIRK: XMEAN IS THE FULL-SAMPLE MEAN. ***
%   The first loop reassigns "XMEAN = Z1/I" every pass, so what SURVIVES it is
%   Z1/100, the mean of ALL 100 draws -- and the second loop uses that one
%   value for every I.  SIGMA(I) is therefore NOT the sample sd of the first I
%   draws; it is their spread about the mean of all 100.  Kept, because it is
%   what the book ran, and because it is not a rounding-level difference:
%   E[SIGMA(I)^2] = (1 - 1/N)*I/(I-1), so at I = 2 it sits about 40% HIGH --
%   exactly where Fig. 4.7's "large errors" are.  Tidying it into a running
%   mean gives a smooth, plausible, different curve with no symptom, which is
%   why SELF-CHECK 4 is a DELETE EXPERIMENT that measures the gap.
%
%   *** THE RNG.  DELIBERATE, AND THE ONLY DEVIATION. ***
%   The book calls unseeded rand(1).  This port uses a MINSTD (Lehmer)
%   generator written into this file: a benchmark that moves when someone else
%   calls rng() is not a benchmark, and fixing that via the global stream would
%   reach into the caller's session mid-suite.  The seed is an argument and is
%   in R.p.seed.  Consequence, stated plainly: NO ROW SCORES OUR DRAW AGAINST
%   THE BOOK'S, because the book's seed is not published.  Two rows are SKIPs
%   for exactly that reason, with the arithmetic in their .why.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Nine rows: five value, one shape, two skip, one info.  Every
%   book value was grepped from zarchan_decoded.txt (equations are invisible to
%   a grep of the raw file); the line is in each row's .source and the band's
%   provenance is in its .detail.  Nothing was read off a figure.
%
%   ⚠️ LINE NUMBERS RE-DERIVED 2026-08-24.  zarchan_decoded.txt was regenerated
%   part-way through this port's authoring and every Ch.4 citation moved by -36
%   lines.  These were re-grepped afterwards and cross-checked against
%   zarchan.txt (same 15120 lines), so a cite here resolves in either file.  If
%   one ever lands on a UUID or an image link, re-grep the sentence -- do not
%   "adjust" it by an offset.
%
%   The two SKIPs are the honest half of this listing: the book's 0.67 and its
%   "more than 100 samples for 5%" are statements about ITS draw.  sd(SIGMA
%   (100)) = 1/sqrt(2*99) = 0.071, so the 5% claim lands either way on about
%   52% of draws; scoring the 0.67 at 3 sd would need a +-63% band.  Neither is
%   scoreable, and NEVER widen a band to turn a row green.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED. ***  This file has
%   never been executed -- MATLAB is the caller's, not this author's.
%     rows 1-3 land well inside their 4-SE bands of 0.47%, 0.73% and 1.94%
%     row 4 lands at -0.50% (4/6 = 0.66667 against the book's printed 0.67)
%     row 5 lands at exactly 0;  row 7's ratio somewhere around 2 to 6
%     SIGMA(1) = 0 exactly;  SIGMA(N) is the sd of all N draws, within a few
%        percent of 1;  SIGMA(2) about 1.3 in expectation but it is one draw
%     SELF-CHECK 4 should report TENS of percent.  Near 1% means the two means
%        have been conflated and checks 2 and 3 have lost their power.
%
% ---------------------------------------------------------------------------
%   OUTPUTS
%     R.I, R.SIGMA       the listing's ArrayI / ArraySIGMA -- Fig. 4.7
%     R.Z, R.XMEAN       the draws, and the FULL-sample mean loop 2 uses
%     R.ptp, R.sapprox   peak-to-peak of R.Z and the book's eyeball estimate
%     R.u_mean, R.u_var, R.x_mean, R.x_sigma   generator statistics
%     R.bench, R.selfcheck, R.pass, R.verdict, R.p, R.units, R.summary
%
%   See also MGS.ZARCHAN.MONTECARLOHOMING (Listing 4.5), MGS.ZARCHAN.BENCHSUITE.
arguments
    % Listing 4.3's own "for I=1:100" and "Z=zeros(1,100)".
    opts.N    (1,1) double {mustBeFinite, mustBeInteger, mustBePositive} = 100
    % Listing 4.1: twelve uniforms, minus six.  The 6 is NSUM/2 and is written
    % that way below so an off-book NSUM cannot silently keep the book's 6.
    opts.NSUM (1,1) double {mustBeFinite, mustBeInteger, mustBePositive} = 12
    % An AUXILIARY sample, drawn from the same stream after the listing's own
    % N.  It exists so that rows 1-3 test the GENERATOR (a claim about the
    % distribution) at a useful band instead of at the +-28% that N = 100
    % alone would force.  nbig = 0 reproduces the listing exactly and widens
    % those three bands automatically; it does not turn them off.
    opts.nbig (1,1) double {mustBeFinite, mustBeInteger, mustBeNonnegative} = 20000
    opts.seed (1,1) double {mustBeFinite, mustBeNonnegative} = 20260824
    % THE ONLY SWITCHES.  Not nargin: with only name-value options declared,
    % MATLAB reports nargin = 0 however many were passed, so "if nargin == 0"
    % here would be a mode that never fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

N = opts.N;  NSUM = opts.NSUM;  nbig = opts.nbig;
if N < 3
    error('mgs:zarchan:sampledStdDev:short', ...
        ['N = %d.  Two reasons, and both are refusals rather than warnings: ' ...
         'SIGMA(1) is SET to zero by the listing rather than estimated, so a ' ...
         'run of one sample contains no estimate at all; and the delete ' ...
         'experiment in SELF-CHECK 4 needs an INTERIOR I (the running-mean ' ...
         'and full-mean curves coincide at I = N by construction), which ' ...
         'first exists at N = 3.'], N);
end

% One continuous stream: the listing's N draws first, the auxiliary sample
% after.  Column j of U is the NSUM uniforms of draw j.
nDraw = N + nbig;
u     = lcgUniform(opts.seed, NSUM*nDraw);
U     = reshape(u, NSUM, nDraw);
Xall  = sum(U, 1) - NSUM/2;          % the listing's "X = SUM - 6"

Z = Xall(1:N);

% =========================================================================
%   LISTING 4.3, FIRST LOOP.  zarchan.txt:1077-1080.
%   XMEAN is reassigned every pass; the value that SURVIVES is Z1/N, the mean
%   of ALL N samples, and that is what the second loop uses for every I.
% =========================================================================
Z1       = 0;
XMEANrun = zeros(1, N);              % kept ONLY for the delete experiment
XMEAN    = 0;
for I = 1:N
    Z1          = Z(I) + Z1;
    XMEAN       = Z1/I;
    XMEANrun(I) = XMEAN;
end

% =========================================================================
%   LISTING 4.3, SECOND LOOP.  zarchan.txt:1081-1088.
% =========================================================================
Z1       = 0;
Iarr     = zeros(1, N);
SIGMAarr = zeros(1, N);
for I = 1:N
    Z1 = (Z(I) - XMEAN)^2 + Z1;
    if I == 1
        SIGMA = 0;                   % the listing's own I == 1 branch
    else
        SIGMA = sqrt(Z1/(I-1));
    end
    Iarr(I)     = I;
    SIGMAarr(I) = SIGMA;
end

% The book's "eyeballed" estimate off Fig. 4.3, zarchan.txt:1021-1022.
ptp     = max(Z) - min(Z);
sapprox = eyeball(ptp);

% Generator statistics, over everything drawn.
nU     = numel(u);
u_mean = sum(u)/nU;
u_var  = sum((u - u_mean).^2)/(nU - 1);
x_mean = sum(Xall)/nDraw;
x_sig  = sqrt(sum((Xall - x_mean).^2)/(nDraw - 1));

R = struct();
R.I       = Iarr;
R.SIGMA   = SIGMAarr;
R.Z       = Z;
R.XMEAN   = XMEAN;
R.ptp     = ptp;
R.sapprox = sapprox;
R.u_mean  = u_mean;
R.u_var   = u_var;
R.x_mean  = x_mean;
R.x_sigma = x_sig;
R.units   = ['DIMENSIONLESS: Z, XMEAN and SIGMA are pure numbers (a unity-' ...
             'variance Gaussian and its statistics).  No feet anywhere.'];
R.p = struct('N', N, 'NSUM', NSUM, 'nbig', nbig, 'seed', opts.seed, ...
             'n_draws', nDraw, 'n_uniforms', nU, 'units', R.units);

R.selfcheck = structuralChecks(Z, XMEAN, XMEANrun, SIGMAarr, u, Xall, ...
                               N, NSUM, opts.seed);

if opts.benchmark
    R.bench = benchmarks(R, N, NSUM, nDraw, nU);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine".
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurve(R); end

% ---- adjudication.  The verdict is FORMED either way; 'assert' chooses only
%      the reaction to it.  With assert false the result comes back COMPLETE,
%      with the failing row attached, and a warning -- never silence.  Every
%      port in this package is run by benchSuite with assert off for exactly
%      that reason.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated at N = %d, ' ...
        'NSUM = %d.  An unvalidated port must not be handed back silently.'], ...
        N, NSUM);
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port ' ...
        'disagrees with ITSELF, which is a wiring fault, not a tolerance ' ...
        'question.  First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf(['%d of %d evaluated book values are outside tolerance ' ...
        '(first: %s, book %s, ours %s, band %s).'], numel(bad), sum(ev), ...
        R.bench(bad(1)).what, R.bench(bad(1)).book_str, ...
        R.bench(bad(1)).ours_str, R.bench(bad(1)).band);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:sampledStdDev:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:sampledStdDev:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function s = eyeball(ptp)
%EYEBALL  zarchan.txt:1022 -- sapprox = (peak to peak)/6.
%   The 6 is SIX STANDARD DEVIATIONS (+-3s covers 99% of a Gaussian,
%   zarchan.txt:1001, :1021).  It is NOT the "- 6" of X = SUM - 6, which is
%   NSUM/2 and happens to share the digit.  Merging the two constants would be
%   invisible at NSUM = 12 and wrong everywhere else.
s = ptp/6;
end


% =============================================================================
function u = lcgUniform(seed, n)
%LCGUNIFORM  MINSTD (Lehmer) uniforms in (0,1).  Base MATLAB, deterministic,
%   and it does not touch the global random stream.
%
%   x <- 16807*x mod (2^31 - 1).  16807*x < 3.61e13 < 2^53, so every product is
%   EXACT in double precision.  x = 0 is absorbing, so the seed is mapped into
%   1..m-1.
m = 2147483647;
x = mod(floor(abs(seed)), m - 1) + 1;
u = zeros(1, n);
for k = 1:n
    x    = mod(16807*x, m);
    u(k) = x/m;
end
end


% =============================================================================
function K = structuralChecks(Z, XMEAN, XMEANrun, SIGMAarr, u, Xall, N, NSUM, seed)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.
%
%   Every constant arrives as an ARGUMENT.  MATLAB local functions cannot see
%   the main function's workspace, and a previous wave in this package shipped
%   a local that read one from the caller.
K = struct();
names = {};  oks = [];  detail = {};

% 1  The listing's I == 1 branch.  Exact zero, not "small".
names{end+1}  = 'SIGMA(1) == 0 exactly (the listing''s I == 1 branch)';
oks(end+1)    = SIGMAarr(1) == 0;
detail{end+1} = sprintf('SIGMA(1) = %g', SIGMAarr(1));
K.sigma1 = SIGMAarr(1);

% 2  XMEAN is the FULL-sample mean.  This is the transcription's load-bearing
%    quirk; see the header.
K.xmean_resid = abs(XMEAN - sum(Z)/N);
names{end+1}  = 'XMEAN is the FULL N-sample mean, not a running mean';
oks(end+1)    = K.xmean_resid <= 1e-12*max(1, abs(XMEAN));
detail{end+1} = sprintf('|XMEAN - mean(Z)| = %.3e', K.xmean_resid);

% 3  The Z1 recursion equals the direct formula, for every I >= 2.  Catches an
%    accumulator that was reset, doubled, or fed the wrong term.
cum    = cumsum((Z - XMEAN).^2);
direct = sqrt(cum(2:N)./((2:N) - 1));
K.recursion_resid = max(abs(SIGMAarr(2:N) - direct))/max(max(direct), realmin);
names{end+1}  = 'Z1 recursion == sqrt(sum((Z(1:I)-XMEAN).^2)/(I-1))';
oks(end+1)    = K.recursion_resid <= 1e-12;
detail{end+1} = sprintf('max relative gap %.3e over I = 2..%d', ...
                        K.recursion_resid, N);

% 4  *** DELETE EXPERIMENT. ***  Recompute the curve with the RUNNING mean --
%    the "tidied" version any reader would write -- and require it to DIFFER.
%    If it does not, checks 2 and 3 have no power and the quirk in the header
%    is a story rather than a fact.  The two curves agree AT I = N by
%    construction (XMEANrun(N) == XMEAN), so the gap is measured over I < N.
srun = zeros(1, N);
for I = 2:N
    srun(I) = sqrt(sum((Z(1:I) - XMEANrun(I)).^2)/(I - 1));
end
den = max(max(SIGMAarr(2:N)), realmin);
K.delete_gap = max(abs(SIGMAarr(2:N-1) - srun(2:N-1)))/den;
names{end+1}  = 'DELETE EXPT: running-mean variant differs (the quirk is real)';
oks(end+1)    = K.delete_gap > 0.01;
detail{end+1} = sprintf(['max gap %.1f%% of peak SIGMA over I = 2..%d ' ...
    '(they coincide at I = %d by construction)'], 100*K.delete_gap, N-1, N);

% 5  The stream.  Strictly inside (0,1), bounded X, and REPRODUCIBLE -- the
%    last clause is what notices if rand() is ever swapped back in.
u2 = lcgUniform(seed, min(64, numel(u)));
K.u_min = min(u);  K.u_max = max(u);
K.repro = isequal(u2, u(1:numel(u2)));
K.x_absmax = max(abs(Xall));
names{end+1}  = 'stream in (0,1), |X| <= NSUM/2, and reproducible from seed';
oks(end+1)    = K.u_min > 0 && K.u_max < 1 && K.repro && ...
                K.x_absmax <= NSUM/2 + 1e-12;
detail{end+1} = sprintf('u in [%.3e, %.6f], max|X| = %.4f (limit %g), repro %d', ...
                        K.u_min, K.u_max, K.x_absmax, NSUM/2, K.repro);

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
function B = benchmarks(R, N, NSUM, nDraw, nU)
%BENCHMARKS  Computed vs published.  Every book value was grepped out of
%   zarchan_decoded.txt and the line number is in the row's .source.  Nothing
%   here was read off a figure -- the extraction turned every figure into an
%   image link.  Where the book states a SHAPE about a figure, that prose IS
%   greppable, and the ordering is scored instead.
%
%   Bands, and the provenance of each:
%     'sampling'  4 standard errors, SE from the DISTRIBUTION's own moments
%                 (never from the draw), so the band cannot move with the seed
%     'text'         the book's printed rounding
%     'closed form'  closed arithmetic, no sampling in it at all -- tol = 0
%     'shape'        an ordering; no tolerance, and none is possible
B = emptyBench();

% ---- 1.  uniform mean = 1/2 ------------------------------------------------
seMean = sqrt((1/12)/nU);                      % Var(U) = 1/12, exactly
B = addVal(B, 'uniform mean = 1/2', 0.5, R.u_mean, 'sampling', ...
    4*seMean/0.5, 'p.60 text (zarchan.txt:986)', '-', ...
    sprintf(['pooled over all %d uniforms drawn; band is 4 SE with ' ...
             'SE = sqrt((1/12)/n) = %.2e, from the distribution, not the draw'], ...
            nU, seMean));

% ---- 2.  uniform variance = 1/12 -------------------------------------------
% Var(s^2) = (m4 - m2^2)/n with m4 = 1/80 and m2 = 1/12 for U(0,1), both exact.
m2u    = 1/12;
m4u    = 1/80;
seVar  = sqrt((m4u - m2u^2)/nU);
B = addVal(B, 'uniform variance = 1/12', m2u, R.u_var, 'sampling', ...
    4*seVar/m2u, 'p.60 text and formula (zarchan.txt:983-986)', '-', ...
    sprintf(['band is 4 SE with SE = sqrt((1/80 - 1/144)/%d) = %.2e; the ' ...
             'book''s formula sigma^2 = (b-a)^2/12 is at :983-985'], nU, seVar));

% ---- 3.  twelve uniforms minus six has UNITY standard deviation ------------
if NSUM == 12
    % Excess kurtosis of a sum of n iid uniforms is -1.2/n, so kappa = 2.9 at
    % n = 12.  SE(s) = s*sqrt((kappa-1)/(4n)).  Theory again, not the draw.
    kappa = 3 - 1.2/NSUM;
    seSig = sqrt((kappa - 1)/(4*nDraw));
    B = addVal(B, 'sigma of (12 uniforms - 6) = 1', 1.0, R.x_sigma, 'sampling', ...
        4*seSig, 'p.62 text (zarchan.txt:1010)', '-', ...
        sprintf(['over %d draws; band is 4 SE with SE = sqrt((kappa-1)/4n), ' ...
                 'kappa = 2.9 (excess kurtosis of a sum of 12 uniforms is ' ...
                 '-1.2/12).  Raise ''nbig'' to tighten it'], nDraw));
else
    B = addSkip(B, 'sigma of (12 uniforms - 6) = 1', '1', ...
        sprintf(['the book''s unity sigma is the TWELVE-uniform claim ' ...
                 '("the variance of 12 must be 1"); at NSUM = %d the ' ...
                 'theoretical sigma is sqrt(%d/12) = %.4f, which is a ' ...
                 'different number and not one the book prints'], ...
                NSUM, NSUM, sqrt(NSUM/12)), ...
        'p.62 text (zarchan.txt:1010)', '-');
end

% ---- 4/5.  the eyeball estimator, on the BOOK's OWN inputs -----------------
% These score sapprox = (peak to peak)/6 at the peak-to-peak values the book
% publishes (4 for the 100-number figure, 6 for the 1000-number figure).  They
% do NOT score our draw -- see row 6.
B = addVal(B, 'eyeball sigma at the book''s ptp = 4:  4/6', 0.67, eyeball(4), ...
    'text', 0.01, 'p.63 text (zarchan.txt:1022)', '-', ...
    ['scores the ESTIMATOR at the book''s own published peak-to-peak, not ' ...
     'our draw.  4/6 = 0.66667; the book prints 0.67, so -0.50% is the ' ...
     'book''s rounding and the band is that rounding']);

B = addVal(B, 'eyeball sigma at the book''s ptp = 6:  6/6', 1.0, eyeball(6), ...
    'closed form', 0, 'p.65 text (zarchan.txt:1047)', '-', ...
    ['the 1000-number case, where the book says the eyeball estimate hits ' ...
     'the theoretically correct value.  Closed arithmetic, so exact -- and ' ...
     'it is what catches a hard-coded 0.67']);

% ---- 6.  SKIP: our own eyeballed sigma vs the book's 0.67 ------------------
B = addSkip(B, 'our eyeballed sigma vs the book''s 0.67', '0.67', ...
    sprintf(['needs the book''s UNPUBLISHED rand seed.  Ours is %.4f (ptp ' ...
             '= %.3f over %d draws).  Arithmetic: for %d unit-normal samples ' ...
             'E[range] = 5.02 with sd 0.59, so sapprox = range/6 has mean ' ...
             '0.84 and sd 0.098; the book''s 0.67 means ITS range was 4.0, ' ...
             '-1.7 sd from its own expectation.  Two independent draws differ ' ...
             'with sd 0.139, so scoring this at 3 sd would need a +-63%% band ' ...
             '-- a band that passes anything.  NOT SCORED.'], ...
            R.sapprox, R.ptp, N, N), ...
    'p.63 text (zarchan.txt:1022)', '-');

% ---- 7.  SHAPE: large errors below 20 samples ------------------------------
% I = 1 is excluded: the listing SETS SIGMA(1) = 0 rather than estimating it.
if N >= 100
    early = 2:20;
    late  = 50:N;
    rmsE  = sqrt(sum((R.SIGMA(early) - 1).^2)/numel(early));
    rmsL  = sqrt(sum((R.SIGMA(late)  - 1).^2)/numel(late));
    ratio = rmsE/max(rmsL, realmin);
    B = addShape(B, 'scatter is larger below 20 samples than above 50', ...
        'rms|SIGMA-1| over I = 2..20  >  rms over I = 50..100', ...
        sprintf('%.4f vs %.4f, ratio %.2f', rmsE, rmsL, ratio), ...
        ratio > 1, 'p.68 text (zarchan.txt:1090)', ...
        ['theory puts the ratio near 4 (E[(SIGMA_I-1)^2] ~ 1/(2(I-1)) plus ' ...
         'the full-mean bias); the row demands only > 1, which is the ' ...
         'literal claim.  This is the row that would catch SIGMA being ' ...
         'divided by I instead of I-1']);
else
    B = addSkip(B, 'scatter is larger below 20 samples than above 50', ...
        'rms early > rms late', ...
        sprintf(['the book''s windows are ABSOLUTE (20 samples, 100 ' ...
                 'samples) and this run used N = %d, so the two windows ' ...
                 'would overlap or be empty'], N), ...
        'p.68 text (zarchan.txt:1090)', '-');
end

% ---- 8.  SKIP: the 5%-at-100-samples claim ---------------------------------
if N >= 100
    dev100 = abs(R.SIGMA(100) - 1);
    B = addSkip(B, '"more than 100 samples needed to be within 5 pct of unity"', ...
        '|SIGMA(100) - 1| > 0.05', ...
        sprintf(['a statement about the book''s own draw.  Ours is %.4f. ' ...
                 'sd(SIGMA(100)) = 1/sqrt(2*99) = 0.0711, so |SIGMA(100)-1| ' ...
                 '<= 0.05 happens on about 52%% of draws -- a coin flip ' ...
                 'cannot adjudicate a port, in either direction.  NOT ' ...
                 'SCORED.'], dev100), ...
        'p.68 text (zarchan.txt:1090)', '-');
else
    B = addSkip(B, '"more than 100 samples needed to be within 5 pct of unity"', ...
        '|SIGMA(100) - 1| > 0.05', ...
        sprintf('N = %d, so there is no SIGMA(100).', N), ...
        'p.68 text (zarchan.txt:1090)', '-');
end

% ---- 9.  INFO: the book's working compromise -------------------------------
if N >= 50
    o50 = sprintf('SIGMA(50) = %.4f, i.e. %+.1f%% from unity', ...
                  R.SIGMA(50), 100*(R.SIGMA(50) - 1));
else
    o50 = sprintf('N = %d, no SIGMA(50)', N);
end
B = addInfo(B, '"50 runs sufficient" -- what that costs', ...
    '50 runs, "sufficient in the tradeoff"', o50, ...
    'p.69 text (zarchan.txt:1095)', ...
    ['NOT SCORED: a policy sentence, not a measurement.  Its price is ' ...
     'sqrt((kappa-1)/4n) = about 10% sampling error on any 50-run standard ' ...
     'deviation -- which is the number to remember before quoting a ' ...
     '50-sortie spread to three figures.']);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want.  Schema is the
%   one mgs.zarchan.benchSuite's adaptGeneric consumes.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
% Tolerance is RELATIVE (a fraction of the book value) -- benchSuite's bandPct
% renders it as a percentage.
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
%ADDINFO  Reported, never scored.  skipped = true keeps it out of the pass
%   count in BOTH directions -- an INFO row can neither pass nor fail.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'info';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'info';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = '-';  B(i).detail = '';
end

function B = addSkip(B, what, bookStr, why, source, units)
%ADDSKIP  A check that COULD NOT RUN.  Never a pass: a check that did not
%   happen is a different thing from one that succeeded.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'skip';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = '--';
B(i).pct  = NaN;   B(i).band = 'skip';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end


% =============================================================================
function s = render(R, ev)
L = {};
p = R.p;
L{end+1} = 'Zarchan Listing 4.3 -- SAMPLED STANDARD DEVIATION (THE HARD WAY)';
L{end+1} = ['*** ' R.units ' ***'];
L{end+1} = sprintf(['   N = %d samples   NSUM = %d uniforms per draw   ' ...
    'auxiliary nbig = %d   seed = %g   (%d uniforms in total)'], ...
    p.N, p.NSUM, p.nbig, p.seed, p.n_uniforms);
L{end+1} = sprintf(['   XMEAN = %+.5f (the FULL %d-sample mean -- the second ' ...
    'loop uses it for every I)'], R.XMEAN, p.N);
L{end+1} = sprintf('   peak-to-peak = %.4f  ->  eyeballed sigma = %.4f', ...
    R.ptp, R.sapprox);
L{end+1} = '';

L{end+1} = 'SIGMA vs NUMBER OF SAMPLES  (the listing''s ArraySIGMA -- Fig. 4.7)';
L{end+1} = sprintf('%12s %14s', 'I', 'SIGMA');
picks = unique(min([1 2 3 5 10 20 50 100 p.N], p.N));
for k = picks
    L{end+1} = sprintf('%12d %14.5f', k, R.SIGMA(k)); %#ok<AGROW>
end
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-58s %s', vstr(K.oks(i)), K.names{i}, ...
        K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning';
    L{end+1} = '            NOT CHECKED, which is not the same thing as "fine".';
else
    L{end+1} = 'BENCHMARKS  (book values grepped from zarchan_decoded.txt; lines in .source)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
            case 'info'
                L{end+1} = sprintf('   INFO %-52s %s', b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case 'skip'
                L{end+1} = sprintf('   SKIP %-52s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case 'shape'
                L{end+1} = sprintf('   %-4s %-52s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf('   %-4s %-52s book %s / ours %s (%+.3f%%, band %.4g%%)', ...
                    vstr(b.ok), b.what, b.book_str, b.ours_str, b.pct, ...
                    100*b.tol); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped/info -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: SIGMA(I) here is the spread of the first I draws about the';
L{end+1} = '   mean of ALL N -- the listing''s own quirk, kept.  And the RNG is a';
L{end+1} = '   seeded LCG, not the book''s unseeded rand, so no row scores our draw';
L{end+1} = '   against the book''s draw.  Two rows are SKIPs for that reason.';
s = strjoin(L, newline);
end

function s = vstr(tf)
if tf
    s = 'PASS';
else
    s = 'FAIL';
end
end


% =============================================================================
function drawCurve(R)
%DRAWCURVE  Base MATLAB graphics only.  Fig. 4.7's own labels.
figure;
plot(R.I, R.SIGMA, '-');
grid on;
hold on;
plot([R.I(1) R.I(end)], [1 1], '--');
xlabel('Number of Samples');
ylabel('Calculated Standard Deviation');
title('Sampled Standard Deviation');
legend('SIGMA(I)', 'theoretical unity', 'Location', 'best');
hold off;
end
