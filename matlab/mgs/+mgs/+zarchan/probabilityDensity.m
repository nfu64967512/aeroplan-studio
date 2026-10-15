function R = probabilityDensity(opts)
%MGS.ZARCHAN.PROBABILITYDENSITY  Zarchan Listing 4.2 -- a probability density
%   function built from samples, scored against the ANALYTIC Gaussian.
%
%   R = mgs.zarchan.probabilityDensity()
%   R = mgs.zarchan.probabilityDensity('N', 1000)          the book's Fig. 4.6
%   R = mgs.zarchan.probabilityDensity('NSUM', 6)          the book's own shortcut
%   R = mgs.zarchan.probabilityDensity('tmp_exact', true, 'assert', false)
%
%   Line-by-line port of Listing 4.2 (zarchan.txt:1026-1039): 100 random numbers
%   are formed by adding 12 uniforms and subtracting 6, dropped into 50 bins over
%   [-6, 6], and plotted against the theoretical zero-mean unit-variance
%   Gaussian.  Listing 4.1 is the same draw without the binning, so it is not a
%   separate port -- it is the X vector returned here.
%
%   UNITS: NONE.  x is in standard deviations, p(x) in 1/x.  Chapter 4's ft/s^2
%   quantities do not appear in this listing at all.
%
% ---------------------------------------------------------------------------
%   WHAT IS SCORED, AND WHAT CANNOT BE
%
%   This listing's own output is Fig. 4.4, and the extraction turned every
%   figure into an image link, so there is no published curve to compare
%   against.  The PROSE around it is greppable, and that is the board:
%
%     * uniform generator: mean 1/2, variance 1/12          (zarchan.txt:986)
%     * "add 12 ... and subtract 6 ... zero-mean ... unity standard deviation";
%       "the variance of 12 must be 1"                      (zarchan.txt:1010)
%     * 68% / 95% / 99% within 1 / 2 / 3 sd                 (zarchan.txt:1001)
%     * the listing's own normaliser TMP = 1./sqrt(6.28)    (zarchan.txt:1026)
%     * the ORDERING between Fig. 4.4 and Fig. 4.6: 100 samples do not obviously
%       follow the Gaussian (:1040), 1000 do "closely follow" it (:1048-1049)
%
%   NOT scored, and the SKIP row says so: sigma_approx = (peak-to-peak)/6 =
%   4/6 = 0.67 (:1022) is the author EYEBALLING Fig. 4.3 -- the peak-to-peak of
%   his own draw, which is nowhere published.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_04_02.m reports ONE unresolved comparison:
%
%     1  "if K . BIN, K=BIN" -> "if K > BIN, K = BIN"  (:1035).  "." is this
%        book's OCR of ">"; its companion "if K , 1" was auto-repaired to "<"
%        because 1 is a literal.  Resolved by CONSEQUENCE: with ">" it is the
%        upper clamp facing the lower one and fires only for X >= XMAX; with
%        "<" it fires for EVERY in-range sample and sweeps the whole histogram
%        into bin 50 -- a spike, not a slightly wrong curve.  Run as a MUTATION
%        EXPERIMENT in structural check 4, not asserted in a comment.
%     2  "K=fix(...)+.99" indexes H with a non-integer.  The book prints its own
%        fix on the next line ("%CORRECTION HERE ... K=round(K);", :1036), and
%        it is applied where the book puts it: AFTER both clamps.  round(fix(v)
%        +0.99) == fix(v)+1, so the effective index is fix(...)+1 clamped.
%     3  fix() rounds TOWARDS ZERO -- floor here only because the argument is
%        non-negative for every in-range sample.
%
%   *** THE HALF-BIN OFFSET IS THE BOOK'S, AND IT IS KEPT. ***  A sample lands
%   in bin K when x is in [XMIN+(K-1)dx, XMIN+K dx), but the listing labels that
%   bin AB = XMIN + K*dx -- its UPPER EDGE -- so the plotted histogram sits
%   dx/2 = 0.12 RIGHT of the curve it is drawn against.  Not corrected: this is
%   a transcription, and the offset is part of why Fig. 4.4 looks as ragged as
%   it does.  Structural check 3 locks it.
%
%   THE RANDOM STREAM.  The listing calls rand(1), so this calls rand, seeds
%   with rng(seed,'twister'), and RESTORES the caller's state on exit via
%   onCleanup -- a port that leaves the global stream moved makes some later
%   port's benchmark irreproducible.  Draws are rand(NSUM,n) summed DOWN
%   COLUMNS: MATLAB fills column-major, so (j,i) is draw NSUM*(i-1)+j, exactly
%   the nested loop's order.  Structural check 1 asserts that bit-for-bit.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED -- this file has never
%   been executed; MATLAB is the caller's. ***  At the defaults: TMP = 0.399044
%   (+0.0253% above the true 1/sqrt(2*pi)); uniform mean 0.5 +/- 2.6e-4 (1 SE)
%   and variance 0.08333 +/- 6.8e-5; X mean 0 +/- 0.0032, variance 1 +/- 0.0045,
%   excess kurtosis -0.100; coverage ~68.3 / 95.5 / 99.75%; rms ratio 100:1000
%   about 2.6x against a 1.5x threshold; dx = 0.24, PDF area exactly 1, no
%   sample out of range, last bin 0% as resolved and 100% under the mutation.
%   THE 3-SD ROW IS THE TIGHTEST on the board (band 1.08 points, expected
%   residual 0.73).  If anything lands near an edge it is that one, and it is
%   not to be widened.
%
%   OUTPUTS: R.X R.H R.AB R.PDF R.TH R.output R.rms R.big R.stat R.p R.units
%   R.bench R.selfcheck R.pass R.verdict R.summary.  'assert' chooses only the
%   reaction to a failure: true errors, false WARNS and still returns R
%   complete.  benchSuite runs every port with it off.
%
%   See also MGS.ZARCHAN.BENCHSUITE, MGS.ZARCHAN.SHAPINGFILTERADJOINT.

arguments
    % Listing 4.2 line 1 (zarchan.txt:1026), verbatim.
    opts.N     (1,1) double {mustBeFinite, mustBePositive, mustBeInteger} = 100
    opts.BIN   (1,1) double {mustBeFinite, mustBePositive, mustBeInteger} = 50
    opts.XMIN  (1,1) double {mustBeFinite} = -6
    opts.XMAX  (1,1) double {mustBeFinite} = 6
    % Fig. 4.5/4.6's sample size.  Used ONLY by the ordering row.
    opts.NBIG  (1,1) double {mustBeFinite, mustBePositive, mustBeInteger} = 1000
    % Uniforms per Gaussian draw.  12 is the listing's; the book's closing
    % sentence (zarchan.txt:1059) says the REST of the book uses 6.  Changing
    % it SKIPS the rows whose book value assumes 12 rather than rescoring them.
    opts.NSUM  (1,1) double {mustBeFinite, mustBePositive, mustBeInteger} = 12
    % Sample for the moment and coverage rows.  N = 100 cannot adjudicate
    % 68/95/99 -- its own sampling error on those is about 4.7 points.
    opts.nstat (1,1) double {mustBeFinite, mustBePositive, mustBeInteger} = 100000
    opts.seed  (1,1) double {mustBeFinite, mustBeNonnegative} = 20260824
    % false = the listing's 1./sqrt(6.28).  true swaps in 1/sqrt(2*pi) and
    % SKIPS the row that locks it, rather than quietly passing a row that
    % claims to check a constant this run did not use.
    opts.tmp_exact  (1,1) logical = false
    opts.benchmark  (1,1) logical = true
    opts.render     (1,1) logical = true
    opts.plot       (1,1) logical = false
    opts.assert     (1,1) logical = true
end

XMIN = opts.XMIN;  XMAX = opts.XMAX;  RANGE = XMAX - XMIN;
BIN  = opts.BIN;   NSUM = opts.NSUM;  N = opts.N;  NBIG = opts.NBIG;

if RANGE <= 0
    error('mgs:zarchan:probabilityDensity:range', ...
        'XMAX = %g must exceed XMIN = %g; RANGE is the bin-width divisor.', XMAX, XMIN);
end
if NBIG < N
    error('mgs:zarchan:probabilityDensity:nbig', ...
        ['NBIG = %d is below N = %d.  The ordering row asks whether MORE samples ' ...
         'fit better; reversed it would score the opposite of the book''s claim ' ...
         'while still printing a tick.'], NBIG, N);
end
if opts.nstat < 1000
    error('mgs:zarchan:probabilityDensity:nstat', ...
        ['nstat = %d.  Below ~1000 the standard errors exceed the band the ' ...
         'moment rows are entitled to, so they pass on almost anything.'], opts.nstat);
end

if opts.tmp_exact
    TMP = 1/sqrt(2*pi);
else
    TMP = 1./sqrt(6.28);        % the listing's own constant, verbatim
end

% restoreRng must stay a live NAMED variable: onCleanup fires when it goes out
% of scope, so dropping the name would restore the stream immediately and the
% seeding below would leak out into the caller's session.
prevRng    = rng;
restoreRng = onCleanup(@() rng(prevRng));

% Structural check 1 runs FIRST and re-seeds, so the literal loop sees the same
% stream the vectorised draw below will see.
rng(opts.seed, 'twister');
mLit = min(N, 20);
Xlit = literalLoop(mLit, NSUM);

% =========================================================================
%   LISTING 4.2.  Transcribed from zarchan.txt:1026-1039.
% =========================================================================
rng(opts.seed, 'twister');
Ubig = rand(NSUM, NBIG);
Xbig = sum(Ubig, 1).' - NSUM/2;      % the listing's "SUM - 6" (NSUM = 12)
X    = Xbig(1:N);
loopResid = max(abs(Xlit(:) - X(1:mLit)));

% The statistics sample continues the SAME stream AFTER both book samples, so
% changing nstat cannot move the reproduction of Fig. 4.4.
Ustat = rand(NSUM, opts.nstat);
Xstat = sum(Ustat, 1).' - NSUM/2;

[H,  nOut] = binCounts(X,    XMIN, XMAX, RANGE, BIN, false);
[Hb, ~]    = binCounts(Xbig, XMIN, XMAX, RANGE, BIN, false);
[Hf, ~]    = binCounts(X,    XMIN, XMAX, RANGE, BIN, true);   % mutation test

dx   = RANGE/BIN;
Kv   = 1:BIN;
AB   = XMIN + Kv*dx;                 % <<< the listing's label: the UPPER EDGE
PDF  = (H /N   )*BIN/RANGE;
PDFb = (Hb/NBIG)*BIN/RANGE;
TH   = TMP*exp(-AB.*AB/2.);

rmsN   = sqrt(mean((PDF  - TH).^2));
rmsBIG = sqrt(mean((PDFb - TH).^2));
S      = sampleStats(Xstat, Ustat);

R = struct();
R.X = X(:).';  R.H = H;  R.AB = AB;  R.PDF = PDF;  R.TH = TH;
R.output = [AB(:), PDF(:), TH(:)];   % the listing's own "output" matrix
R.rms    = rmsN;
R.n_out_of_range = nOut;
R.big   = struct('N',NBIG, 'X',Xbig(:).', 'H',Hb, 'PDF',PDFb, 'rms',rmsBIG);
R.stat  = S;
R.units = 'dimensionless: x in standard deviations, p(x) in 1/x; no ft in this listing';
R.p = struct('N',N, 'NBIG',NBIG, 'BIN',BIN, 'XMIN',XMIN, 'XMAX',XMAX, ...
             'RANGE',RANGE, 'dx',dx, 'NSUM',NSUM, 'TMP',TMP, ...
             'tmp_exact',opts.tmp_exact, 'nstat',opts.nstat, ...
             'seed',opts.seed, 'generator','twister', 'units',R.units);

R.selfcheck = structuralChecks(loopResid, mLit, PDF, dx, AB, XMIN, XMAX, ...
                               BIN, nOut, NSUM, H, Hf, N, S);

if opts.benchmark
    R.bench = benchmarks(opts, TMP, S, rmsN, rmsBIG, X, Xbig);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  "Not checked" must never render as "fine".
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R); end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false: nothing was compared. R.pass = false means NOT CHECKED.';
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated at NSUM = %d, ' ...
        'tmp_exact = %d.  An unvalidated port must not be handed back silently.'], ...
        NSUM, opts.tmp_exact);
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, a wiring fault rather than a tolerance question.  ' ...
        'First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
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
        error('mgs:zarchan:probabilityDensity:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN, with R complete, so the caller can see WHICH side
        % moved.  A port that refuses to return is a port nobody can diagnose.
        warning('mgs:zarchan:probabilityDensity:benchmark', ...
            '%s  (''assert'' is false; result returned anyway, see R.verdict)', msg);
    end
end
end


% =============================================================================
function X = literalLoop(n, NSUM)
%LITERALLOOP  Listing 4.1/4.2's nested draw with no vectorising, so the
%   vectorised path has something to be checked against.  Constants arrive as
%   ARGUMENTS: MATLAB local functions cannot see the caller's workspace.
X = zeros(n,1);
for I = 1:n
    SUM = 0;
    for J = 1:NSUM
        RAND = rand(1);
        SUM  = SUM + RAND;
    end
    X(I) = SUM - NSUM/2;        % the listing's "-6" at NSUM = 12
end
end


% =============================================================================
function [Hc, nOut] = binCounts(X, XMIN, XMAX, RANGE, BIN, flipClamp)
%BINCOUNTS  Listing 4.2's binning loop, zarchan.txt:1034-1037.
%
%   flipClamp = true swaps the resolved "if K > BIN" for the OTHER reading of
%   the OCR'd comparison.  It exists only for the mutation experiment: a
%   resolution nobody can watch fail is a resolution nobody has justified.
%
%   nOut counts samples outside [XMIN, XMAX) -- the ones the clamps genuinely
%   relocate.  It does NOT count bin-1 samples, which trip "K < 1" only because
%   the listing's K carries a +0.99.
Hc = zeros(1, BIN);  nOut = 0;
for I = 1:numel(X)
    K = fix(((X(I)-XMIN)/RANGE)*BIN) + .99;
    if K < 1,  K = 1;  end
    if flipClamp
        if K < BIN, K = BIN; end
    else
        if K > BIN, K = BIN; end
    end
    K = round(K);                          % the listing's own %CORRECTION HERE
    Hc(K) = Hc(K) + 1;
    if X(I) < XMIN || X(I) >= XMAX,  nOut = nOut + 1;  end
end
end


% =============================================================================
function S = sampleStats(X, U)
%SAMPLESTATS  Moments of the Gaussian sample X and of the raw uniforms U.
%   Divisors are written out: variances use (n-1), standard errors use (n).
%   Base MATLAB only -- no Statistics Toolbox anywhere in this file.
n = numel(X);
S.n = n;  S.mean = sum(X)/n;
d  = X - S.mean;
m2 = sum(d.^2)/n;  m4 = sum(d.^4)/n;
S.var = sum(d.^2)/(n-1);  S.sd = sqrt(S.var);
S.se_mean = sqrt(m2/n);
S.se_var  = sqrt(max(m4 - m2^2, 0)/n);
S.kurt_ex = m4/max(m2^2, realmin) - 3;
S.se_kurt = sqrt(24/n);              % normal theory; this sample is near-normal
S.ptp     = max(X) - min(X);

% Coverage within 1, 2, 3 SAMPLE standard deviations of the SAMPLE mean.  "of
% the mean" is the book's own wording (zarchan.txt:1001), and measuring about
% the sample mean keeps these rows independent of the zero-mean row: a wrong
% subtraction then shows up there and only there.
a = abs(d);  S.p = zeros(1,3);  S.se_p = zeros(1,3);
for k = 1:3
    pk = sum(a <= k*S.sd)/n;
    S.p(k)    = 100*pk;
    S.se_p(k) = 100*sqrt(max(pk*(1-pk), 0)/n);
end

nu = numel(U);  S.u_n = nu;  S.u_mean = sum(U(:))/nu;
du  = U(:) - S.u_mean;
mu2 = sum(du.^2)/nu;  mu4 = sum(du.^4)/nu;
S.u_var     = sum(du.^2)/(nu-1);
S.u_se_mean = sqrt(mu2/nu);
S.u_se_var  = sqrt(max(mu4 - mu2^2, 0)/nu);
end


% =============================================================================
function K = structuralChecks(loopResid, mLit, PDF, dx, AB, XMIN, XMAX, ...
                              BIN, nOut, NSUM, H, Hf, N, S)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never to be quoted
%   as evidence that this port reproduces Zarchan -- only that it agrees with
%   itself, which is what catches wiring faults.
%
%   .names and .oks are what benchSuite puts on the board.  An invariant stored
%   under any other field name is one nobody can see, which is the same as one
%   that never ran.
K = struct();  names = {};  oks = [];  detail = {};

% 1  The vectorised draw IS the listing's nested loop, value for value.
K.loop_resid = loopResid;
names{end+1} = 'rand(NSUM,n) column-sum == the literal nested loop';
oks(end+1)   = loopResid == 0;
detail{end+1} = sprintf('max |diff| = %.3e over the first %d samples', loopResid, mLit);

% 2  The sampled PDF integrates to 1 -- catches the (H/N)*BIN/RANGE scale.
%    Clamping preserves total count, so the integral alone cannot see relocated
%    mass; the out-of-range count is folded in when the support fits inside the
%    range, which at NSUM = 12 over [-6,6] it exactly does.
K.pdf_area = sum(PDF)*dx;  K.n_out = nOut;
supportFits = (-NSUM/2 >= XMIN) && (NSUM/2 <= XMAX);
names{end+1} = 'sample PDF integrates to 1 (and nothing clamped)';
oks(end+1)   = abs(K.pdf_area - 1) <= 1e-12 && (~supportFits || nOut == 0);
detail{end+1} = sprintf('area = %.15g, %d sample(s) outside [%g,%g)%s', ...
    K.pdf_area, nOut, XMIN, XMAX, tern(supportFits, '', ' [support exceeds range]'));

% 3  THE HALF-BIN OFFSET IS KEPT.  A LOCK: if someone re-centres the labels the
%    listing has changed, and this goes red instead of the curve quietly
%    getting better.
Kv = 1:BIN;
K.label_edge_resid   = max(abs(AB - (XMIN + Kv*dx)));
K.label_centre_shift = max(abs((AB - (XMIN + (Kv-0.5)*dx)) - dx/2));
names{end+1} = 'AB labels are bin UPPER edges (+dx/2 offset kept)';
oks(end+1)   = K.label_edge_resid == 0 && K.label_centre_shift <= 1e-12;
detail{end+1} = sprintf('offset from centres = %+.4g (= dx/2); edge residual %.3e', ...
    dx/2, K.label_edge_resid);

% 4  MUTATION EXPERIMENT on the one unresolved comparison (zarchan.txt:1035).
%    "K > BIN" is the resolution; under "K < BIN" nearly all mass lands in the
%    last bin.  This row says that out loud instead of leaving it in a comment.
K.tail_frac_ok = H(BIN)/N;  K.tail_frac_flip = Hf(BIN)/N;
names{end+1} = 'clamp reads "K > BIN" (mutation: "K < BIN" collapses it)';
oks(end+1)   = K.tail_frac_ok < 0.10 && K.tail_frac_flip > 0.90;
detail{end+1} = sprintf('last bin holds %.1f%% as resolved vs %.1f%% flipped', ...
    100*K.tail_frac_ok, 100*K.tail_frac_flip);

% 5  Excess kurtosis of a sum of NSUM uniforms is -6/(5*NSUM) EXACTLY (each
%    uniform contributes -6/5; the excess kurtosis of a sum of iid terms
%    divides by their number).  DERIVED HERE, not a book value.  Its job is to
%    catch NSUM changing underneath a run that still claims the book's case:
%    12 gives -0.100, 6 gives -0.200, and the band is 5 SE.
K.kurt_target = -6/(5*NSUM);  K.kurt_ex = S.kurt_ex;
kurtLim = 5*S.se_kurt;
names{end+1} = 'excess kurtosis == -6/(5*NSUM)';
oks(end+1)   = abs(S.kurt_ex - K.kurt_target) <= kurtLim;
detail{end+1} = sprintf('%+.4f vs %+.4f target (limit %.4f = 5 SE, n = %d)', ...
    S.kurt_ex, K.kurt_target, kurtLim, S.n);

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
function B = benchmarks(opts, TMP, S, rmsN, rmsBIG, X, Xbig)
%BENCHMARKS  Computed vs published, with the PROVENANCE of every band.
%
%   *** EVERY BOOK VALUE WAS GREPPED OUT OF zarchan.txt AND THE LINE NUMBER IS
%   IN THE .source FIELD. ***  Nothing was read off a figure.
%
%   Bands:  'listing'       exact -- a constant printed in the listing itself
%           'sampling'      5 SE of our own estimate.  5, not 3: the rows share
%                           one fixed seed, so a merely-significant k buys a
%                           standing chance a CORRECT port errors on sight.
%           'text+sampling' the book's printed precision PLUS 5 SE, summed here
%                           in code so it can never be tighter than either part
%           'shape'         an ordering; no tolerance, and none is possible
%
%   Do NOT widen a band to turn a row green.
B = emptyBench();
NSUM = opts.NSUM;

% ---- 1.  the listing's Gaussian normaliser ---------------------------------
if ~opts.tmp_exact
    B = addVal(B, 'TMP = 1./sqrt(6.28)', 1./sqrt(6.28), TMP, 'listing', 0, ...
        'Listing 4.2 line 1 (zarchan.txt:1026)', '-', ...
        sprintf(['a LOCK on the transcription, not a physics check: the book ' ...
            'writes 6.28 for 2*pi, so its theory curve sits %+.4f%% above the ' ...
            'true Gaussian at every x.  Tidying it to 1/sqrt(2*pi) turns this ' ...
            'row red, which is the point'], 100*(1/sqrt(6.28)*sqrt(2*pi) - 1)));
else
    B = addSkip(B, 'TMP = 1./sqrt(6.28)', '1./sqrt(6.28)', ...
        ['''tmp_exact'' is true, so the listing''s constant was deliberately ' ...
         'replaced by 1/sqrt(2*pi).  Scored NOT CHECKED rather than passed ' ...
         'against a value this run did not use'], ...
        'Listing 4.2 line 1 (zarchan.txt:1026)', '-');
end

% ---- 2, 3.  the uniform generator ------------------------------------------
B = addVal(B, 'mean of the uniform generator', 0.5, S.u_mean, 'sampling', ...
    5*S.u_se_mean/0.5, 'p.60 text (zarchan.txt:986)', '-', ...
    sprintf('"the mean ... should be 1/2".  n = %d draws, SE = %.3g, band 5 SE', ...
        S.u_n, S.u_se_mean));

B = addVal(B, 'variance of the uniform generator', 1/12, S.u_var, 'sampling', ...
    5*S.u_se_var/(1/12), 'p.60 text (zarchan.txt:986)', '-', ...
    sprintf(['"and the variance should be 1/12".  SE %.3g from the sample''s ' ...
        'OWN 4th moment -- these are uniforms (kurtosis 1.8) and normal ' ...
        'theory would misstate the band'], S.u_se_var));

% ---- 4.  zero mean.  A bound row: addVal cannot carry a book value of zero,
%          because a relative tolerance against 0 is meaningless.  This is also
%          the ONLY row that would notice the listing's "- 6" being wrong.
zMean = abs(S.mean)/max(S.se_mean, realmin);
B = addBound(B, 'sum of 12 uniforms minus 6 has ZERO MEAN', '|mean| <= 5 SE', ...
    sprintf('mean = %+.5f  (%.2f SE, SE = %.3g, n = %d)', S.mean, zMean, ...
        S.se_mean, S.n), zMean <= 5, 'p.62 text (zarchan.txt:1010)', '-', ...
    ['"add 12 uniformly distributed random variables and subtract 6 in order ' ...
     'to obtain a zero-mean Gaussian variable".  Subtract 5.5 instead and this ' ...
     'is the only row that moves']);

% ---- 5.  unity variance ----------------------------------------------------
if NSUM == 12
    B = addVal(B, 'variance of the 12-uniform sum is UNITY', 1.0, S.var, ...
        'sampling', 5*S.se_var, 'p.62 text (zarchan.txt:1010)', '-', ...
        sprintf(['"as the variance of one uniformly distributed random variable ' ...
            'is 1/12, the variance of 12 must be 1".  SE %.3g from the ' ...
            'sample''s own 4th moment; band 5 SE'], S.se_var));
else
    B = addSkip(B, 'variance of the 12-uniform sum is UNITY', '1', ...
        sprintf(['NSUM = %d, not 12, so the book''s own arithmetic gives ' ...
            'NSUM/12 = %.4f, not 1.  Scoring 1 here would be scoring a ' ...
            'sentence this run does not implement'], NSUM, NSUM/12), ...
        'p.62 text (zarchan.txt:1010)', '-');
end

% ---- 6, 7, 8.  the 68 / 95 / 99 rule ---------------------------------------
% Band = 1 percentage point (the book prints WHOLE percents against exact
% values of 68.27 / 95.45 / 99.73, so one point is its own printed precision,
% not a courtesy) PLUS 5 SE of our estimate, summed here in code.
bookP = [68, 95, 99];  sdName = {'one', 'two', 'three'};
for k = 1:3
    what = sprintf('%% within %s standard deviation(s) of the mean', sdName{k});
    if NSUM >= 6
        B = addVal(B, what, bookP(k), S.p(k), 'text+sampling', ...
            (1.0 + 5*S.se_p(k))/bookP(k), 'p.61 text (zarchan.txt:1001)', '%', ...
            sprintf(['exact Gaussian %.2f%%; the book prints whole percents.  ' ...
                'Band = 1 point + 5 SE (%.3g) = %.3g points'], ...
                gaussPct(k), S.se_p(k), 1.0 + 5*S.se_p(k)));
    else
        B = addSkip(B, what, sprintf('%d%%', bookP(k)), ...
            sprintf(['NSUM = %d.  These figures describe a GAUSSIAN, and the ' ...
                'central-limit argument that makes this sum Gaussian needs a ' ...
                'sum, not one or two uniforms'], NSUM), ...
            'p.61 text (zarchan.txt:1001)', '%');
    end
end

% ---- 9.  THE SHAPE CLAIM ---------------------------------------------------
% zarchan.txt:1040 on Fig. 4.4 -- with 100 numbers "it is not immediately
% obvious that the computer-generated probability density function follows a
% Gaussian distribution".  zarchan.txt:1048-1049 on Fig. 4.6 -- at 1000 numbers
% it "closely follows the theoretical bell-shaped curve".  That is an ORDERING,
% and prose about a figure is greppable even though the figure is not.
%
% THRESHOLD, derived before the run and not tuned to it: histogram sampling
% error falls as 1/sqrt(n), so the pure-sampling expectation for 100 -> 1000 is
% sqrt(10) = 3.16.  The book's half-bin label offset adds a SYSTEMATIC error of
% about 0.013 in these units that does not shrink with n, diluting that to
% about 2.6.  An rms over 50 bins has a relative spread near 10%, so the
% ratio's is near 14%; 1.5 sits about 3 sd below 2.6.
bookCase = opts.N == 100 && opts.NBIG == 1000 && opts.BIN == 50 && ...
           opts.XMIN == -6 && opts.XMAX == 6 && NSUM == 12;
if bookCase && rmsBIG > 0
    ratio = rmsN/rmsBIG;
    B = addShape(B, 'more samples fit the Gaussian better (Fig. 4.4 vs 4.6)', ...
        'rms(PDF - theory) at N = 100 is at least 1.5x that at N = 1000', ...
        sprintf('rms %.5f (N=100) / %.5f (N=1000) = %.2fx', rmsN, rmsBIG, ratio), ...
        ratio >= 1.5, 'p.65 text (zarchan.txt:1040, :1048-1049)', ...
        ['expected 2.6x.  The two samples are NESTED (the 100 are the first of ' ...
         'the 1000), which takes "we drew a lucky pair" out of the row']);
else
    B = addSkip(B, 'more samples fit the Gaussian better (Fig. 4.4 vs 4.6)', ...
        'rms(100) >= 1.5 * rms(1000)', ...
        sprintf(['off the book''s own case (N=%d, NBIG=%d, BIN=%d, range ' ...
            '[%g,%g], NSUM=%d).  The 1.5x threshold was derived for 100 vs ' ...
            '1000 over 50 bins; carrying it elsewhere would reuse a number ' ...
            'whose derivation no longer applies'], opts.N, opts.NBIG, ...
            opts.BIN, opts.XMIN, opts.XMAX, NSUM), ...
        'p.65 text (zarchan.txt:1040, :1048-1049)', '-');
end

% ---- 10.  THE ROW THAT CANNOT BE SCORED ------------------------------------
B = addSkip(B, 'sigma_approx = (peak-to-peak)/6 = 4/6 = 0.67', '0.67', ...
    sprintf(['NOT SCOREABLE: the author is EYEBALLING Fig. 4.3 -- the ' ...
        'peak-to-peak of HIS 100-number draw, which the book never prints.  A ' ...
        'ptp of 4.0 means all 100 samples fell inside +/-2 sd, probability ' ...
        '0.9545^100 = 0.0095, so it is a property of one draw and not of the ' ...
        'method.  Ours: ptp/6 = %.3f at N=100 and %.3f at N=1000 (book 0.67 ' ...
        'and 6/6 = 1).  Widening a band to absorb this would be scoring the ' ...
        'draw'], (max(X)-min(X))/6, (max(Xbig)-min(Xbig))/6), ...
    'p.63 text (zarchan.txt:1022, also :1047)', '-');
end


% =============================================================================
function p = gaussPct(k)
%GAUSSPCT  Exact Gaussian coverage within k sd, percent.  erf is base MATLAB.
p = 100*erf(k/sqrt(2));
end

function s = tern(c, a, b)
if c, s = a; else, s = b; end
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want: a struct array
%   that quietly reordered fields would make [B.ok] mean something else.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
% Tolerance arrives EXPLICITLY rather than through a band->tol lookup: most
% bands here are built from the sample's own standard error at the call site,
% and a lookup table would need a second copy of that arithmetic.
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
B(i).ok = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = '-';  B(i).detail = detail;
end

function B = addBound(B, what, bookStr, oursStr, ok, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'bound';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'bound';  B(i).tol = NaN;
B(i).ok = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = units;  B(i).detail = detail;
end

function B = addSkip(B, what, bookStr, why, source, units)
%ADDSKIP  A check that COULD NOT RUN.  Never a pass: a check that did not
%   happen is a different thing from one that succeeded.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'skip';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = '--';
B(i).pct  = NaN;   B(i).band = 'skip';  B(i).tol = NaN;
B(i).ok = false;   B(i).skipped = true;  B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end


% =============================================================================
function s = render(R, ev)
L = {};  p = R.p;
L{end+1} = 'Zarchan Listing 4.2 -- PROBABILITY DENSITY FUNCTION FROM SAMPLES';
L{end+1} = sprintf('   %s', R.units);
L{end+1} = sprintf('   N = %d   BIN = %d over [%g, %g]   dx = %g   NSUM = %d   TMP = %.6f%s', ...
    p.N, p.BIN, p.XMIN, p.XMAX, p.dx, p.NSUM, p.TMP, ...
    tern(p.tmp_exact, '  (1/sqrt(2*pi), NOT the listing''s)', '  (= 1./sqrt(6.28))'));
L{end+1} = sprintf('   seed = %g (%s; caller''s stream restored on exit)   NBIG = %d   nstat = %d', ...
    p.seed, p.generator, p.NBIG, p.nstat);
L{end+1} = '';
L{end+1} = 'BINS  (AB is the bin''s UPPER EDGE -- the listing''s own label)';
L{end+1} = sprintf('%6s %9s %7s %11s %11s %11s', 'K', 'AB', 'H', 'PDF', 'theory', 'PDF-theory');
stride = max(1, round(p.BIN/13));
for r = unique([1:stride:p.BIN, p.BIN])
    L{end+1} = sprintf('%6d %9.3f %7d %11.5f %11.5f %+11.5f', r, R.AB(r), ...
        R.H(r), R.PDF(r), R.TH(r), R.PDF(r) - R.TH(r)); %#ok<AGROW>
end
L{end+1} = sprintf('   rms(PDF - theory) = %.5f at N = %d, %.5f at N = %d   (%.2fx)', ...
    R.rms, p.N, R.big.rms, R.big.N, R.rms/max(R.big.rms, realmin));
L{end+1} = '';
K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-52s %s', vstr(K.oks(i)), K.names{i}, K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';
if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false means NOT';
    L{end+1} = '            CHECKED, which is not the same thing as "fine".';
else
    L{end+1} = 'BENCHMARKS  (book values grepped from zarchan.txt; line numbers in .source)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
            case 'skip'
                L{end+1} = sprintf('   SKIP %-52s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case {'shape','bound'}
                L{end+1} = sprintf('   %-4s %-52s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf('   %-4s %-52s book %s %s / ours %s %s (%+.3f%%, band %.4g%%)', ...
                    vstr(b.ok), b.what, b.book_str, b.units, b.ours_str, ...
                    b.units, b.pct, 100*b.tol); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: Fig. 4.4 is not a benchmark -- the extraction turned every';
L{end+1} = '   figure into an image link.  Scored here: the PROSE around the listing,';
L{end+1} = '   plus the analytic Gaussian the listing computes for itself.';
s = strjoin(L, newline);
end

function s = vstr(tf)
if tf, s = 'PASS'; else, s = 'FAIL'; end
end


% =============================================================================
function drawCurves(R)
%DRAWCURVES  The listing's own plot (Fig. 4.4) and its 1000-sample twin
%   (Fig. 4.6).  Base MATLAB graphics only.
sets = {R.PDF, R.p.N; R.big.PDF, R.big.N};
for k = 1:2
    figure;
    plot(R.AB, sets{k,1}, R.AB, R.TH), grid on
    title(sprintf('Sample Gaussian distribution (N = %d)', sets{k,2}));
    xlabel('X'); ylabel('Probability Density Function');
    legend('sampled', 'theory', 'Location', 'best');
end
end
