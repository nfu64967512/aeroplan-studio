function R = lowPassWhiteNoise(opts)
%MGS.ZARCHAN.LOWPASSWHITENOISE  Zarchan Listing 4.4 -- a first-order low-pass
%   filter driven by pseudo-white noise, scored against the chapter's own
%   closed form for the output standard deviation.
%
%   R = mgs.zarchan.lowPassWhiteNoise()
%   R = mgs.zarchan.lowPassWhiteNoise('TAU', 1.0, 'assert', false)
%   R = mgs.zarchan.lowPassWhiteNoise('nrep', 2000)      cheaper, wider band
%
%   UNITS: no feet and no g here.  PHI is in "units squared per Hertz", Y in
%   "units"; TAU, H and t are seconds.  Nothing is converted -- there is
%   nothing to convert.
%
%   WHAT IS SCOREABLE.  Figs. 4.9/4.10 became image links in the extraction and
%   publish no ordinate.  Seven published things survive, and they are the
%   whole board: sigma_x = sqrt(PHI/h) (dec:1187-1190); E[y^2(t)] =
%   PHI(1-e^(-2t/T))/(2T) (dec:1181-1182); E[y^2(inf)] = PHI/(2T)
%   (dec:1183-1185); "within the +/-sigma bounds approximately 68% of the time"
%   (dec:1200, and again for T = 1 s at dec:1209); "Increasing filter time
%   constant reduces the noise transmission" (dec:1202); "tends to correlate
%   the noise" (dec:1209).  LEAN ON PURPOSE: this chapter is a baseline the
%   noise-driven adjoint ports lean on, not one that bears on our problem.
%
%   ⚠️⚠️ CITATION NOTE.  The two source files are NOT line-aligned here, against
%   the usual convention: zarchan.txt has 15120 lines, zarchan_decoded.txt has
%   15307, and in THIS region the decoded file runs exactly 36 lines later
%   (checked on six separate sentences).  Every .source cites both, as
%   "dec:N / raw:M".  One number alone would send a reader to Fig. 4.11.
%
%   THE TRANSCRIPTION.  listing_04_04.m reports no unresolved
%   variable-to-variable comparisons, and that is right: both of its
%   comparisons are against literals.  Repairs, none a judgement call:
%     1  "while T ,= 5.0" -> "while T <= 5.0" and "while STEP ,=1" ->
%        "while STEP <= 1".  The OCR renders "<" as "," throughout this book;
%        T only increases, and the inner body sets STEP=2 to leave.
%     2  Trailing "ADJOINTS FOR NOISE-DRIVEN SYSTEMS" is the book's next
%        SECTION HEADING leaking in (dec:1220; the listing ends dec:1219).
%        Dropped, not "fixed" into code.
%     3  "FLAG=0" after the inner loop is dead (the outer loop re-initialises
%        it).  Recorded, dropped.  So is "save datfil.txt output -ascii".
%   Two things ARE judgement calls, so they are stated:
%     * TIME.  The listing accumulates T += H and tests "T <= 5.0" at the top:
%       exactly, 501 records at t = 0.01..5.01 s; in floating point 500
%       additions of 0.01 overshoot 5.0 by 4e-15, so the literal loop records
%       500 on some machines and 501 on others.  This port fixes the COUNT
%       (nsteps = floor(TEND/H + 0.5) + 1 = 501) and otherwise leaves T alone:
%       it is still accumulated by heunStep and still fed to SIGPLUS, exactly
%       as the listing does.  Only the termination ambiguity is removed, and it
%       is an ambiguity about one record, not about the physics.
%     * randn -> a PRIVATE RandStream with a recorded seed.  Not rng(): rng()
%       reaches into the caller's session and benchSuite runs 49 ports.
%   VECTORISED ACROSS REPLICATIONS: the book flies one trajectory and looks at
%   a figure, and a figure cannot adjudicate a standard deviation.  The
%   arithmetic per replication, and its order, is the listing's.
%
%   THE DISCRETE CLOSED FORM -- what makes this port sharp.  Push a scalar
%   through the transcribed two-pass step by hand.  With a = H/TAU it collapses
%   EXACTLY to Y_k = A*Y_(k-1) + B*X_(k-1), A = 1 - a + a^2/2, B = a - a^2/2,
%   A + B = 1 identically (unity DC gain; A is the RK2 truncation of exp(-a)).
%   The X_k being independent with variance PHI/H,
%        var(Y_k) = SIG^2 * B^2/(1 - A^2) * (1 - A^(2k))
%   the EXACT ensemble variance OF THE CODE AS TRANSCRIBED, against the book's
%   continuous PHI(1-e^(-2t/T))/(2T).  At TAU = 0.2, H = 0.01 the two differ by
%   -0.028%: far inside the sampling band, but knowing the number means a
%   residual can be attributed rather than excused.  Self-check 2 scores the
%   measured curve against the DISCRETE form while the benchmark rows score it
%   against the BOOK's, so a failure says which one moved.  ⚠️ a = 2 makes B
%   exactly 0 -- the filter stops hearing the noise and returns a smooth zero,
%   with no symptom.  H >= TAU is refused for that.
%
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED. ***  Never executed --
%   MATLAB is the caller's.  At the defaults:
%     SIG = sqrt(PHI/h) = 10 exactly; over 1.002e7 draws, 10.000 +/- 0.002
%     sigma_ss(TAU=0.2): book 1.58114, discrete-exact 1.58070 (-0.028%)
%     sigma(t=TAU)     : book 1.47026, discrete-exact 1.46966 (-0.041%)
%     fraction inside +/-sigma: 68.3% (Gaussian truth 68.27%; book says 68)
%     sigma_ss(TAU=1)  : book 0.707107, discrete at t=5.01 s 0.707082
%     lag-1 autocorrelation: 0.9512 at TAU=0.2, 0.9900 at TAU=1 (both = A)
%     delete 1 (SIG missing 1/sqrt(h)): sigma ~ 0.158, -90%  -> must go RED
%     delete 2 (noise frozen for the run): sigma ~ 10.0, +532% -> must go RED
%   A residual landing near a band EDGE is not a pass to be accepted; the
%   discrete form says where it should land.  Do not widen a band to go green.
%
%   OUTPUTS: R.t R.sigma R.sigma_theory R.sigma_disc R.mean R.y_one (the TAU
%   run); R.run2 (the Fig. 4.10 run); R.disc (A, B, discrete/continuous ratio);
%   R.bench; R.selfcheck with .names/.oks (REQUIRED by adaptGeneric); R.pass,
%   true only if a row was SCORED and all passed; R.verdict, '' when clean;
%   R.p; R.summary; R.units.
%
%   See also MGS.ZARCHAN.SHAPINGFILTERADJOINT, MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 4.4 line 1, verbatim (dec:1212).
    opts.TAU  (1,1) double {mustBeFinite, mustBePositive} = 0.2    % s
    opts.PHI  (1,1) double {mustBeFinite, mustBePositive} = 1.0    % units^2/Hz
    opts.H    (1,1) double {mustBeFinite, mustBePositive} = 0.01   % s
    opts.TEND (1,1) double {mustBeFinite, mustBePositive} = 5.0    % s
    % Fig. 4.10: "the correlation time constant is increased from 0.2 s to 1 s".
    opts.TAU2 (1,1) double {mustBeFinite, mustBePositive} = 1.0    % s
    % Replications.  THE DOMINANT COST.  Lower it freely inside a suite -- the
    % bands widen with it automatically (ksig/sqrt(2*nrep)), so the rows stay
    % honest: 20000 gives 2%, 2000 gives 6.3%.
    opts.nrep (1,1) double {mustBeFinite, mustBePositive} = 20000
    opts.seed (1,1) double {mustBeFinite, mustBeNonnegative} = 20260824
    % 4, not 3: five value rows are tested at once against a FIXED seed, so a k
    % that is merely "significant" buys a standing chance that a CORRECT port
    % fails on its first run.  The two faults these rows exist to catch move
    % sigma by 90% and 532%, against a 4-SE band of 2%.
    opts.ksig (1,1) double {mustBeFinite, mustBePositive} = 4.0
    % THE ONLY SWITCHES.  Not nargin: with an arguments block declaring only
    % name-value options MATLAB reports nargin = 0 however many were passed.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

TAU = opts.TAU;  PHI = opts.PHI;  H = opts.H;  TEND = opts.TEND;
nrep = round(opts.nrep);

if H >= min(TAU, opts.TAU2)
    error('mgs:zarchan:lowPassWhiteNoise:stiff', ...
        ['H = %g s is not smaller than the filter time constant (%g s).  The ' ...
         'chapter requires h << T (dec:1186); worse, a = H/TAU = 2 makes the ' ...
         'input coefficient B EXACTLY zero, so the filter stops hearing the ' ...
         'noise and returns a smooth zero with no symptom.'], ...
        H, min(TAU, opts.TAU2));
end
nsteps = floor(TEND/H + 0.5) + 1;
if nsteps < 10
    error('mgs:zarchan:lowPassWhiteNoise:short', ...
        'TEND = %g s at H = %g s gives %d steps; too few to measure anything.', ...
        TEND, H, nsteps);
end
if nrep < 30
    error('mgs:zarchan:lowPassWhiteNoise:nrep', ...
        ['nrep = %d.  A sample this small has no usable standard error, so ' ...
         'every value row would pass on almost anything.'], nrep);
end

% Autocorrelation window: steady state only, so the transient cannot masquerade
% as correlation.  Three time constants, or half the run, whichever is sooner.
acfAt = @(T) max(2, min(round(3*T/H), round(0.5*nsteps)));

S1 = ensembleRun(TAU,       PHI, H, nsteps, nrep, mkStream(opts.seed,0), 1.0, false, acfAt(TAU));
S2 = ensembleRun(opts.TAU2, PHI, H, nsteps, nrep, mkStream(opts.seed,1), 1.0, false, acfAt(opts.TAU2));

% The two DELETE EXPERIMENTS, at reduced cost.  They are RUN, and their
% verdicts reach the board through R.selfcheck.names/.oks -- a delete
% experiment nobody can see is the same as not running one.
nmut = min(nrep, 2000);
D1 = ensembleRun(TAU, PHI, H, nsteps, nmut, mkStream(opts.seed,2), sqrt(H), false, acfAt(TAU));
D2 = ensembleRun(TAU, PHI, H, nsteps, nmut, mkStream(opts.seed,3), 1.0,     true,  acfAt(TAU));

R = struct();
R.t = S1.t;  R.sigma = S1.sigma;  R.sigma_theory = S1.sigma_theory;
R.sigma_disc = S1.sigma_disc;  R.mean = S1.mean;  R.y_one = S1.y_one;
R.run2 = S2;
R.disc = struct('a', H/TAU, 'A', S1.A, 'B', S1.B, ...
                'sigma_ss_disc', sqrt(S1.var_ss_disc), ...
                'sigma_ss_book', sqrt(PHI/(2*TAU)), ...
                'ratio_disc_over_book', sqrt(S1.var_ss_disc)/sqrt(PHI/(2*TAU)));
R.units = ['dimensionless "units": PHI [units^2/Hz], Y and sigma [units], ' ...
           'TAU/H/t [s].  Ch.4''s low-pass example carries no ft and no g.'];
R.p = struct('TAU',TAU, 'PHI',PHI, 'H',H, 'TEND',TEND, 'TAU2',opts.TAU2, ...
             'nsteps',nsteps, 'nrep',nrep, 'nrep_mutant',nmut, ...
             'seed',opts.seed, 'ksig',opts.ksig, ...
             'generator','mt19937ar, Ziggurat, private RandStream', ...
             'units',R.units);

% Bands, as RELATIVE fractions (benchSuite's bandPct prints 100*tol).
tolX = max(opts.ksig/sqrt(2*nsteps*nrep), 0.002);   % std of the noise draws
tolS = max(opts.ksig/sqrt(2*nrep),        0.005);   % std of the filter output
tolF = 0.03;                                        % the book's own "68"

R.selfcheck = structuralChecks(S1, D1, D2, PHI, TAU, H, tolS, opts.ksig, nrep);

if opts.benchmark
    R.bench = benchmarks(S1, S2, PHI, TAU, opts.TAU2, H, tolX, tolS, tolF);
    ev      = ~[R.bench.skipped] & ~strcmpi({R.bench.kind}, 'info');
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    R.pass  = false;   % NOT true: "not checked" must never render as "fine".
end

R.summary = render(R, S1, S2, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(S1); end

msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the book. ' ...
           'R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = 'not one book value could be evaluated.';
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, a wiring fault rather than a tolerance question.  ' ...
        'First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf(['%d of %d evaluated book values are outside tolerance ' ...
        '(first: %s, book %s, ours %s, band %.3g%%).'], numel(bad), sum(ev), ...
        R.bench(bad(1)).what, R.bench(bad(1)).book_str, ...
        R.bench(bad(1)).ours_str, 100*R.bench(bad(1)).tol);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:lowPassWhiteNoise:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN, complete.  benchSuite runs every port with assert
        % off precisely so that one red row cannot hide the board.
        warning('mgs:zarchan:lowPassWhiteNoise:benchmark', ...
            '%s  (''assert'' is false; the result is returned anyway, see R.verdict)', msg);
    end
end
end


% =============================================================================
function rs = mkStream(seed, offset)
%MKSTREAM  A PRIVATE stream.  Never rng(): that reaches into the caller's
%   session, and benchSuite runs 49 ports back to back.
rs = RandStream('mt19937ar', 'Seed', mod(round(seed) + offset, 2^31 - 1), ...
                'NormalTransform', 'Ziggurat');
end


% =============================================================================
function [Ynew, Tnew] = heunStep(Y, X, TAU, H, T)
%HEUNSTEP  Listing 4.4's inner STEP/FLAG loop, dec:1213-1216.  Every constant
%   arrives as an ARGUMENT: MATLAB local functions cannot see the main
%   function's workspace.
%
%   The loop runs EXACTLY twice.  Pass 1 (FLAG==0) evaluates YD at the current
%   state; pass 2 takes a full Euler step, advances T by H once, and
%   re-evaluates there.  X is held across both passes, as in the listing.
YOLD = Y;
STEP = 1;
FLAG = 0;
YD   = zeros(size(Y));      % never read before assignment; initialised anyway
while STEP <= 1
    if FLAG == 1
        STEP = 2;
        Y = Y + H*YD;
        T = T + H;
    end
    YD   = (X - Y)/TAU;
    FLAG = 1;
end
% The listing writes "FLAG=0" after this loop.  Dead -- the outer loop
% re-initialises FLAG -- so it is recorded here rather than reproduced.
Ynew = (YOLD + Y)/2 + 0.5*H*YD;
Tnew = T;
end


% =============================================================================
function S = ensembleRun(TAU, PHI, H, nsteps, nrep, rs, sigScale, freezeNoise, acfStart)
%ENSEMBLERUN  Listing 4.4 for nrep independent replications at once.
%   sigScale and freezeNoise exist ONLY for the delete experiments (1.0/false
%   is the listing).  They are parameters rather than edited copies so that the
%   mutants run the SAME transcribed step function the nominal case does.
SIG = sigScale*sqrt(PHI/H);
Y   = zeros(1, nrep);
T   = 0.0;

t = zeros(nsteps,1);  sg_ens = zeros(nsteps,1);  mn_ens = zeros(nsteps,1);
sg_th = zeros(nsteps,1);  y_one = zeros(nsteps,1);
nInAll = 0;  nInOne = 0;  sx = 0;  sxx = 0;  cAB = 0;  cAA = 0;  cBB = 0;

% Drawn only when it is going to be used, so the nominal run does not silently
% consume nrep normals of stream state that a reader cannot account for.
Xfrozen = [];
if freezeNoise, Xfrozen = SIG*randn(rs, 1, nrep); end
for k = 1:nsteps
    if freezeNoise
        X = Xfrozen;                       % DELETE EXPERIMENT 2
    else
        X = SIG*randn(rs, 1, nrep);        % the listing: a fresh draw per step
    end
    sx = sx + sum(X);  sxx = sxx + sum(X.*X);

    Yprev  = Y;
    [Y, T] = heunStep(Y, X, TAU, H, T);

    m  = sum(Y)/nrep;
    s2 = sum((Y - m).^2)/(nrep - 1);
    sg = sqrt(PHI*(1 - exp(-2*T/TAU))/(2*TAU));   % the listing's own SIGPLUS

    t(k) = T;  mn_ens(k) = m;  sg_ens(k) = sqrt(s2);  sg_th(k) = sg;
    y_one(k) = Y(1);
    nInAll = nInAll + sum(abs(Y) <= sg);
    nInOne = nInOne + double(abs(Y(1)) <= sg);

    if k >= acfStart
        cAB = cAB + sum(Y.*Yprev);
        cAA = cAA + sum(Y.*Y);
        cBB = cBB + sum(Yprev.*Yprev);
    end
end

a = H/TAU;
S = struct();
S.t = t;  S.sigma = sg_ens;  S.mean = mn_ens;  S.sigma_theory = sg_th;
S.y_one = y_one;
S.A = 1 - a + a^2/2;
S.B = a - a^2/2;
S.var_ss_disc = SIG^2 * S.B^2/(1 - S.A^2);
if freezeNoise
    % That recursion assumes INDEPENDENT draws.  Returning it anyway would be a
    % closed form quietly evaluated for a process it does not describe.
    S.sigma_disc = nan(nsteps,1);
else
    S.sigma_disc = sqrt(S.var_ss_disc*(1 - S.A.^(2*(1:nsteps)')));
end
nx = nsteps*nrep;
S.frac_in = nInAll/nx;  S.frac_in_one = nInOne/nsteps;
S.acf1    = cAB/sqrt(max(cAA*cBB, realmin));
S.x_std   = sqrt(max(sxx - sx^2/nx, 0)/(nx - 1));
S.x_mean  = sx/nx;
S.SIG = SIG;  S.nrep = nrep;  S.TAU = TAU;
end


% =============================================================================
function K = structuralChecks(S1, D1, D2, PHI, TAU, H, tolS, ksig, nrep)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never to be quoted
%   as evidence that this port reproduces Zarchan.
names = {};  oks = [];  detail = {};
K = struct();

% 1  The transcribed step really is Y <- A*Y + B*X with A + B = 1.  Measured on
%    the SAME function the ensemble uses: (Y=1,X=0) returns A, (Y=0,X=1) is B.
Ameas = heunStep(1, 0, S1.TAU, H, 0);
Bmeas = heunStep(0, 1, S1.TAU, H, 0);
K.gain_resid = abs(Ameas + Bmeas - 1);
K.A_resid    = abs(Ameas - S1.A);
names{end+1} = 'Heun step is Y <- A*Y + B*X with A + B == 1 (unity DC gain)';
oks(end+1)   = K.gain_resid <= 1e-12 && K.A_resid <= 1e-12;
detail{end+1} = sprintf('A = %.9g (closed form %.9g), B = %.9g, |A+B-1| = %.3e', ...
                        Ameas, S1.A, Bmeas, K.gain_resid);

% 2  Measured ensemble sigma against the EXACT discrete closed form for the
%    transcribed scheme -- not the book's continuous one, so a failure here is
%    a WIRING fault and not a discretisation residual.
K.disc_max_rel = max(abs(S1.sigma./S1.sigma_disc - 1));
tolCurve = max(1.25*ksig/sqrt(2*nrep), 0.004);   % max over a correlated sweep
names{end+1} = 'ensemble sigma(t) == exact discrete closed form';
oks(end+1)   = K.disc_max_rel <= tolCurve;
detail{end+1} = sprintf('max |sigma/sigma_disc - 1| = %.3f%% (band %.3f%%)', ...
                        100*K.disc_max_rel, 100*tolCurve);

% 3  Zero mean.  A bias in the noise handling shows here and nowhere else cheap.
K.mean_max_rel = max(abs(S1.mean)./max(S1.sigma, realmin));
tolMean = 1.25*ksig/sqrt(nrep);
names{end+1} = 'ensemble mean == 0 (in units of sigma)';
oks(end+1)   = K.mean_max_rel <= tolMean;
detail{end+1} = sprintf('max |mean|/sigma = %.3f%% (band %.3f%%)', ...
                        100*K.mean_max_rel, 100*tolMean);

% 4  DELETE EXPERIMENT.  SIG without the 1/sqrt(h).  The chapter's whole point
%    (dec:1187-1190) is that the draw must be scaled by 1/sqrt(h) to deliver
%    spectral density PHI; drop it and the run is smooth, plausible and 90%
%    low.  What is checked is that BENCHMARK 2's band catches it.
book_ss = sqrt(PHI/(2*TAU));
K.del_noscale_dev = D1.sigma(end)/book_ss - 1;
names{end+1} = 'DELETE: SIG missing 1/sqrt(h) is caught by the sigma_ss band';
oks(end+1)   = abs(K.del_noscale_dev) > tolS;
detail{end+1} = sprintf('mutant sigma_ss = %.4g vs book %.4g (%+.1f%%, band %.3g%%)', ...
                        D1.sigma(end), book_ss, 100*K.del_noscale_dev, 100*tolS);

% 5  DELETE EXPERIMENT.  One noise draw held for the whole run -- the exact
%    mis-transcription of hoisting "X=SIG*randn" out of the outer loop.  The
%    filter then converges to its own input and sigma -> SIG.
K.del_frozen_dev = D2.sigma(end)/book_ss - 1;
names{end+1} = 'DELETE: noise frozen for the run is caught by the sigma_ss band';
oks(end+1)   = abs(K.del_frozen_dev) > tolS;
detail{end+1} = sprintf('mutant sigma_ss = %.4g vs book %.4g (%+.1f%%, band %.3g%%)', ...
                        D2.sigma(end), book_ss, 100*K.del_frozen_dev, 100*tolS);

K.names  = names(:)';
K.oks    = logical(oks(:)');
K.detail = detail(:)';
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    i = find(~K.oks, 1);
    K.first_bad = sprintf('%s [%s]', K.names{i}, K.detail{i});
end
end


% =============================================================================
function B = benchmarks(S1, S2, PHI, TAU, TAU2, H, tolX, tolS, tolF)
%BENCHMARKS  Every book value was grepped out of zarchan_decoded.txt and the
%   line number is in the .source field of its row.  Nothing was read off a
%   figure: the extraction turned every figure into an image link.
U = 'units';

% 1  sigma_x = sqrt(PHI/h), scored as the MEASURED std of the draws rather than
%    the constant we assigned -- the latter is a tautology, and this is the row
%    delete experiment 4 aims at.
b1 = mkValue('noise draw std = sqrt(PHI/h)', sqrt(PHI/H), S1.x_std, tolX, ...
    'Eq. p.72, dec:1187-1190 / raw:1151-1154; listing line 1, dec:1212', U, ...
    sprintf('%d draws, mean %+.3g', S1.nrep*numel(S1.t), S1.x_mean));

% 2  Steady state, E[y^2(inf)] = PHI/(2T).
b2 = mkValue(sprintf('sigma_ss = sqrt(PHI/(2*TAU)), TAU = %g s', TAU), sqrt(PHI/(2*TAU)), ...
    S1.sigma(end), tolS, 'Eq. p.72, dec:1183-1185 / raw:1147-1149', U, ...
    sprintf(['discrete-exact value for the transcribed scheme is %.6g ' ...
             '(%.3f%% under the continuous form); the rest is sampling'], ...
            sqrt(S1.var_ss_disc), 100*(1 - sqrt(S1.var_ss_disc)/sqrt(PHI/(2*TAU)))));

% 3  The TRANSIENT, at one time constant.  Row 2 cannot see it.
[~, kT] = min(abs(S1.t - TAU));
b3 = mkValue('sigma(t = TAU) = sqrt(PHI(1-e^-2)/(2*TAU))', ...
    sqrt(PHI*(1 - exp(-2))/(2*TAU)), S1.sigma(kT), tolS, ...
    'Eq. p.72, dec:1181-1182 / raw:1145-1146; restated dec:1196-1199', U, ...
    sprintf('sampled at t = %.4g s (index %d)', S1.t(kT), kT));

% 4  "the simulation results lay within the +/-sigma bounds approximately 68% of
%    the time", scored over the ENSEMBLE.  The book justifies 68 for a Gaussian
%    at dec:1037 / raw:1001; the exact Gaussian figure is 68.27%.
b4 = mkValue(sprintf('fraction inside +/-sigma bounds, TAU = %g s', TAU), 68, ...
    100*S1.frac_in, tolF, ...
    'Prose p.73, dec:1200 / raw:1164; Gaussian 68% at dec:1037 / raw:1001', ...
    '%', 'ensemble, all replications x all steps');

% 5  The same claim made a second time, for Fig. 4.10's TAU = 1 s.
b5 = mkValue(sprintf('fraction inside +/-sigma bounds, TAU = %g s', TAU2), 68, ...
    100*S2.frac_in, tolF, 'Prose p.73, dec:1209 / raw:1173', '%', ...
    sprintf('TAU2 = %g s', TAU2));

% 6  The second parameter's steady state, same published formula.
b6 = mkValue(sprintf('sigma_ss = sqrt(PHI/(2*TAU)), TAU = %g s', TAU2), sqrt(PHI/(2*TAU2)), ...
    S2.sigma(end), tolS, ...
    'Eq. p.72, dec:1183-1185 / raw:1147-1149; TAU2 from dec:1201 / raw:1165', ...
    U, sprintf('transient residual at t = %.4g s is %+.3g%%', S1.t(end), ...
               100*(sqrt(1 - exp(-2*S1.t(end)/TAU2)) - 1)));

% 7  SHAPE.  "Increasing filter time constant reduces the noise transmission."
b7 = mkShape('larger TAU reduces noise transmission', ...
    S2.sigma(end) < S1.sigma(end), ...
    sprintf('sigma_ss(%g s) < sigma_ss(%g s)', TAU2, TAU), ...
    sprintf('%.4g < %.4g', S2.sigma(end), S1.sigma(end)), ...
    'Fig. 4.10 caption + prose, dec:1201-1202 / raw:1165-1166', ...
    sprintf('ratio %.4g, closed form sqrt(TAU/TAU2) = %.4g', ...
            S2.sigma(end)/S1.sigma(end), sqrt(TAU/TAU2)));

% 8  SHAPE.  "tends to correlate the noise ... the randomness of the noise is
%    starting to disappear as the filter time constant increases."
b8 = mkShape('larger TAU correlates the output more', S2.acf1 > S1.acf1, ...
    sprintf('acf1(%g s) > acf1(%g s)', TAU2, TAU), ...
    sprintf('%.4f > %.4f', S2.acf1, S1.acf1), ...
    'Prose p.73, dec:1201 + dec:1209 / raw:1165 + raw:1173', ...
    sprintf('closed forms A = %.4f and %.4f', S2.A, S1.A));

% 9  INFO.  The book's own experiment is ONE trajectory (Fig. 4.9).  Its
%    samples are correlated over TAU, so only about TEND/(2*TAU) are
%    effectively independent and the proportion carries a ~13-point standard
%    error of its own.  Reported so the book's actual experiment is visible;
%    refused adjudication because it is a question about the draw.
neff = max(S1.t(end)/(2*TAU), 1);
b9 = mkInfo('single-run fraction inside +/-sigma (Fig. 4.9)', '~68 %', ...
    sprintf('%.1f %%', 100*S1.frac_in_one), ...
    sprintf(['one trajectory of %d correlated samples: ~%.0f effectively ' ...
             'independent, so this proportion''s own SE is ~%.0f points.'], ...
            numel(S1.t), neff, 100*sqrt(0.68*0.32/neff)), ...
    'Fig. 4.9, dec:1203 / raw:1167 -- the figure publishes no ordinate');

B = [b1 b2 b3 b4 b5 b6 b7 b8 b9];
end


% =============================================================================
%   ROW CONSTRUCTORS.  ONE struct() call, so the field ORDER cannot drift --
%   benchSuite's adaptGeneric is strict and a near-miss schema errors.
function b = mkRow(what, kind, book, book_str, ours, ours_str, pct, band, tol, ...
                   ok, skipped, why, source, units, detail)
b = struct('what', what, 'kind', kind, 'book', book, 'book_str', book_str, ...
           'ours', ours, 'ours_str', ours_str, 'pct', pct, 'band', band, ...
           'tol', tol, 'ok', ok, 'skipped', skipped, 'why', why, ...
           'source', source, 'units', units, 'detail', detail);
end

function b = mkValue(what, book, ours, tol, source, units, detail)
fin = isfinite(ours) && isfinite(book);
pct = 100*(ours/book - 1);
ok  = fin && abs(ours - book) <= tol*abs(book);
why = '';
if ~fin, why = 'computed value is not finite'; pct = NaN; end
b = mkRow(what, 'value', book, sprintf('%.6g', book), ours, ...
          sprintf('%.6g', ours), pct, '+/-', tol, ok, ~fin, why, source, ...
          units, detail);
end

function b = mkShape(what, ok, book_str, ours_str, source, detail)
b = mkRow(what, 'shape', NaN, book_str, NaN, ours_str, NaN, 'ordering', NaN, ...
          logical(ok), false, '', source, '', detail);
end

function b = mkInfo(what, book_str, ours_str, why, source)
b = mkRow(what, 'info', NaN, book_str, NaN, ours_str, NaN, 'not scored', NaN, ...
          false, false, why, source, '', '');
end

function B = emptyBench()
b = mkRow('', 'value', NaN, '', NaN, '', NaN, '', NaN, false, true, '', '', '', '');
B = b([]);
end


% =============================================================================
function s = render(R, S1, S2, ev)
L = {'mgs.zarchan.lowPassWhiteNoise -- Zarchan Listing 4.4 (Ch.4), Figs. 4.8-4.10'};
L{end+1} = sprintf('  %s', R.units);
L{end+1} = sprintf(['  TAU %g s, PHI %g units^2/Hz, H %g s, TEND %g s, %d steps ' ...
    'x %d reps, seed %d (%s)'], R.p.TAU, R.p.PHI, R.p.H, R.p.TEND, R.p.nsteps, ...
    R.p.nrep, R.p.seed, R.p.generator);
L{end+1} = sprintf(['  discrete scheme A %.6f, B %.6f: sigma_ss %.6g vs book ' ...
    '%.6g (%+.3f%%)'], R.disc.A, R.disc.B, R.disc.sigma_ss_disc, ...
    R.disc.sigma_ss_book, 100*(R.disc.ratio_disc_over_book - 1));
for S = [S1 S2]
    L{end+1} = sprintf(['  TAU %g s: sigma_ss %.5g, acf1 %.4f, inside bounds ' ...
        '%.1f%%'], S.TAU, S.sigma(end), S.acf1, 100*S.frac_in); %#ok<AGROW>
end
L{end+1} = '  --- benchmarks ------------------------------------------------';
for i = 1:numel(R.bench)
    b = R.bench(i);
    if any(strcmpi(b.kind, {'shape','info'}))
        L{end+1} = sprintf('   [%s] %-44s %s', tick(b.ok, b.kind), b.what, ...
            b.ours_str); %#ok<AGROW>
    else
        L{end+1} = sprintf('   [%s] %-44s book %-8s ours %-8s %+7.2f%% (band %.3g%%)', ...
            tick(b.ok, b.kind), b.what, b.book_str, b.ours_str, b.pct, ...
            100*b.tol); %#ok<AGROW>
    end
end
L{end+1} = '  --- structural ------------------------------------------------';
for i = 1:numel(R.selfcheck.names)
    L{end+1} = sprintf('   [%s] %-44s %s', tick(R.selfcheck.oks(i), 'x'), ...
        R.selfcheck.names{i}, R.selfcheck.detail{i}); %#ok<AGROW>
end
if any(ev)
    L{end+1} = sprintf('  %d of %d evaluated rows pass; structural %d of %d.', ...
        sum([R.bench(ev).ok]), sum(ev), sum(R.selfcheck.oks), numel(R.selfcheck.oks));
else
    L{end+1} = '  NOTHING ADJUDICATED -- R.pass is false meaning NOT CHECKED.';
end
s = strjoin(L, newline);
end

function c = tick(ok, kind)
%TICK  'info' rows are neither pass nor fail -- printing XX for one would put a
%   red mark against a number this port deliberately refuses to adjudicate.
if strcmpi(kind, 'info'), c = '--'; elseif ok, c = 'ok'; else, c = 'XX'; end
end


% =============================================================================
function drawCurves(S1)
%DRAWCURVES  Fig. 4.9's own plot: one trajectory against +/-sigma_theory.
%   The listing's "save datfil.txt output -ascii" is deliberately NOT
%   reproduced -- a port that writes into the caller's working directory as a
%   side effect of being benchmarked is a surprise nobody asked for.
figure;
plot(S1.t, S1.y_one, '-', S1.t, S1.sigma_theory, '--', ...
     S1.t, -S1.sigma_theory, '--', S1.t, S1.sigma, ':', 'LineWidth', 1);
grid on;
title(sprintf('Low-pass filter driven by white noise, TAU = %g s', S1.TAU));
xlabel('Time (S)');
ylabel('Y');
legend('one run', '+sigma theory', '-sigma theory', 'ensemble sigma', ...
       'Location', 'best');
end
