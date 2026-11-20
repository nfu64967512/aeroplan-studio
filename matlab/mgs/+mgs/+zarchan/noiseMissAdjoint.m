function R = noiseMissAdjoint(opts)
%MGS.ZARCHAN.NOISEMISSADJOINT  Zarchan Listing 18.1 -- adjoint of the
%   fifth-order binomial guidance system for NOISE miss distances.
%
%   R = mgs.zarchan.noiseMissAdjoint()                  % reproduces Table 18.1
%   R = mgs.zarchan.noiseMissAdjoint('tf', 10, 'assert', false)   % AS PRINTED
%   R.miss('gl', 3000, 0.5, 40000, 1e-4)                % -> sigma in ft
%
%   One integration gives, for every flight time at once, the standard
%   deviation of the miss from each of FOUR white-noise sources: range-
%   independent (receiver), semiactive range-dependent, active range-dependent
%   and glint.  The listing sets Vc = T = RA = Phi = 1 (:5356) so its outputs
%   ARE the Table 18.1 coefficients, which are DIMENSIONLESS.  The book's units
%   are ft, ft/s, s with g = 32.2 ft/s^2; R.miss() is the only sanctioned way
%   to turn a coefficient into feet, because it reads the exponents from
%   CHANTABLE, the one place they exist.  Applied with the wrong power of T a
%   coefficient is off by a factor of T and still looks like plausible feet.
%
%   OVERLAP WITH MGS.ZARCHAN.ADJOINTBINOMIAL5 (Listing 6.1).  The PLANT IS THE
%   SAME SEVEN LINES: X2D/Y1/X3D/X4D/X5D/X6D/X7D/X8D are character-for-
%   character Listing 6.1 (:1723-1724), already carried at adjointBinomial5.m
%   lines 333-341.  Listing 18.1 deletes X1 and the deterministic step/ramp/
%   parabolic channels, drops VC from 4000 to 1, and reuses X9..X12 as four
%   noise accumulators, each an integral of a squared signal proportional to
%   the adjoint variable y1 (Fig. 18.3).  So if adjointBinomial5 is right and
%   this is wrong, the fault is in those four lines and nowhere else.  The
%   plant is deliberately NOT shared: each file transcribes its own page, and a
%   shared kernel would let an edit to one silently rewrite the other's book
%   comparison.
%
%   TRANSCRIPTION REPAIRS to listing_18_01.m.  (1) "while TP,=(TF-1e-5)" ->
%   "<=";  "," is this book's OCR of "<", and shape forces it: TP only
%   increases, so ">=" never terminates.  (2) Transcription lines 11-28 are
%   TABLE 18.1 printed across the listing -- dropped from the code and
%   harvested as the benchmark, which is how they were identified; the three
%   "ligature" residues live there.  (3) The trailing "MINIMUM GUIDANCE SYSTEM
%   TIME CONSTANT" is the next section heading (:5385) leaking in.  (4) "FLAG=0"
%   after the inner loop is dead -- recorded, not coded, so nobody re-adds it
%   thinking a state machine was lost.
%
%   THE GLINT NORMALIZATION IS A FLATTENED FRACTION AND THE SIGN MATTERS.
%   Table 18.1's glint row reads "sMiss T0:5 / F0:5 GL" over two lines
%   (:5378-5379): sigma ~ T^(-0.5), the ONLY channel where a FASTER guidance
%   system gives a BIGGER miss.  The prose formula at :5287 flattens to
%   "1.44T0.5F0.5", which reads as T^(+0.5) -- the opposite.  Resolved as
%   NEGATIVE three ways: the book says so in words at :5290 (reduce the time
%   constant and glint miss increases); H(t) ~ 1/T so int H^2 dt ~ 1/T exactly;
%   and it is MEASURED, by self-check 4, out of the transcribed ODEs.  Getting
%   it backwards costs nothing at T = 1 s, which is why it is scored.
%
%   THE LISTING AS PRINTED DOES NOT REPRODUCE ITS OWN TABLE.  It sets TF = 10.
%   Table 18.1 is the STEADY-STATE table, and 10 s is not steady state for
%   N' >= 4: the active channel weights the integrand by tgo^4, so its tail is
%   still arriving at t = 10.  Measured deficit at N' = 5: FN -0.04%, RN
%   -0.29%, RNA -1.38%, GL -0.002%.  Against the book's 182 the TF = 10 answer
%   is -1.55% -- outside the band -- where the converged answer is -0.17%.
%   No symptom: the short run returns a smooth, plausible, slightly small
%   number.  So 'tf' DEFAULTS TO 25, and self-check 1 measures convergence
%   rather than trusting it.  Pass 'tf', 10 for the as-printed run.
%
%   BENCHMARKS -- ten rows, six scored; each row cites its own zarchan.txt
%   line, so only the bands are worth stating here.
%
%   ⚠️ WHERE THE BOOK VALUES CAME FROM, because the two files differ and the
%   split is NOT the obvious one.  The TABLE 18.1 block (:5363-5380) is one the
%   decoder did NOT reach: :5366 and :5368 are identical in zarchan.txt and
%   zarchan_decoded.txt.  Its twelve coefficient digits are clean in both, but
%   its column headings ("N0 ¼ 3" = N' = 3) and all four normalization factors
%   ("VcT0:5F 0:5 FN" = Vc*T^0.5*Phi_FN^0.5) still carry the raw ligatures, so
%   they were decoded BY HAND against the documented "¼"->"=" and colon->
%   decimal-point substitutions.  Without that step a grep gets the numbers but
%   cannot say which column is which N' nor what to divide by.  The SINGLE-LAG
%   coefficients behind rows 8 and 9 are the opposite case and DO need the
%   decoded file: :5308 is "¼ 0:532VcT 0:5F0:5" raw against "= 0.532VcT 0.5F0.5"
%   decoded, so a naive grep for 0.532 in the raw file finds nothing at all.
%     1-4  Table 18.1 at N' = 3, one row per channel.  Band 'text' = 1%, this
%          package's convention for a value printed to three figures and the
%          band adjointBinomial5 uses for the same book.  Residuals -0.018 /
%          +0.035 / +0.051 / -0.086%.
%     5-6  The N' = 4 and N' = 5 columns, one AGGREGATE row each (max relative
%          deviation over the four channels, same band, every pair named in
%          .detail).  Measured maxima 0.335% and 0.273%.  Aggregated to keep
%          this port lean; .detail names the guilty channel when one goes red.
%     7-8  SHAPE rows on two prose claims about N' sensitivity.  Row 8 is
%          scored as a COMPARISON against the book's own single-lag numbers,
%          which is what makes "strong influence" measurable at all.
%     9-10 INFO.  Row 9 is the book's "an order of magnitude larger" claim:
%          measured 5.71x, 8.94x, 8.87x -- and GLINT 1.17x, so the claim is
%          FALSE for one channel in four and any band that passed it would be
%          one the book itself violates.  Row 10 is the TF = 10 truncation
%          above: a finding about the book, not a published number.
%
%   WHY 1% AND NOT TIGHTER.  The book's single-lag coefficients (used only in
%   rows 8 and 9) were checked BY HAND against exact integrals while writing
%   this file -- NOT by this code, which computes none of them: glint 1.4361 vs
%   printed 1.44, semiactive 1.06066 vs 1.06, range-independent 0.53033 vs
%   0.532 (book high 0.31%), active 4.5928 vs 4.66 (book high 1.5%).  This
%   book's third digit is good to about a percent, not to a part in a thousand.
%
%   WHAT THIS PRINTS.  Executed in MATLAB on 2026-08-24 at the defaults
%   (xnp = [3 4 5], tau = 1, tf = 25, h = 0.01) this file produced
%
%     N'=3   3.0394    9.4734    41.321    1.6786
%     N'=4   5.0761   18.338     89.353    2.3478
%     N'=5   8.1816   33.608    181.68     3.2033
%
%   with six scored rows inside band (worst 0.335%) and five self-checks
%   holding (tail 1.4e-10, Vc 7.6e-15, RA 3.3e-15, T 7.4e-6, monotone 0.0).
%   ONE run on ONE machine, recorded so a later run has something to disagree
%   with -- not a guarantee.  If a residual lands near a band edge instead of
%   near these, something moved: the gap between 0.34% and 1% is the entire
%   safety margin.  Do not widen a band to turn a row green.
%
%   Three claims were exercised rather than assumed: 'tf', 10 turns self-check
%   1 RED and fails 2 of 8 scored rows (1.42% and 1.55%); 'xnp', 3 scores four
%   rows and SKIPS four rather than passing quietly on a smaller board; and
%   tau 0.4, Vc 3000, RA 50000, phi [2 3 4 5] -- every normalization factor off
%   book at once -- returns the SAME coefficients to five figures, which a
%   wrong exponent could not survive, glint's -0.5 included.
%
%   OUTPUTS.  R.t adjoint time [s] (== the original system's FLIGHT TIME);
%   R.xnp, R.chan the 1x4 channel table; R.coef Kx4 STEADY-STATE coefficients,
%   rows following R.xnp; R.coef_tf10 the same read at TP = 10 s, the listing's
%   own TF; R.curve(k) the whole sweep (.coef normalized, .sigma not);
%   R.miss(key, Vc, T, RA, Phi) -> sigma [ft], ONE VALUE PER R.xnp, scalar only
%   when xnp is; R.bench rows in the shared schema; R.selfcheck .names/.oks
%   (NOT book values); R.pass true only if a row was scored and all passed;
%   R.verdict '' when clean but ALWAYS FORMED, with 'assert' choosing error vs
%   warn; R.p the parameters actually used; R.summary the rendered text.
%
%   See also MGS.ZARCHAN.ADJOINTBINOMIAL5, MGS.ZARCHAN.BENCHSUITE.

arguments
    opts.xnp   (1,:) double {mustBeFinite, mustBePositive} = [3 4 5]  % XNP, N'
    opts.tau   (1,1) double {mustBeFinite, mustBePositive} = 1.0      % TAU [s]
    % NOT the listing's 10 -- see the truncation section above.
    opts.tf    (1,1) double {mustBeFinite, mustBePositive} = 25.0     % TF  [s]
    opts.h     (1,1) double {mustBeFinite, mustBePositive} = 0.01     % H   [s]
    opts.vc    (1,1) double {mustBeFinite, mustBePositive} = 1.0      % VC
    opts.ra    (1,1) double {mustBeFinite, mustBePositive} = 1.0      % RA
    % PHIFN PHIRN PHIRNA PHIGL, in the listing's order.
    opts.phi   (1,4) double {mustBeFinite, mustBePositive} = [1 1 1 1]
    % The listing tests "S >= .0999" with H = .01, i.e. every tenth step.
    opts.samp  (1,1) double {mustBeFinite, mustBePositive} = 0.1      % [s]
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.assert    (1,1) logical = true
end

% nargin is ALWAYS 0 in a function whose arguments block declares only
% name-value options (measured, R2025b), so it is never a mode switch here.

if opts.h >= opts.tau/5
    error('mgs:zarchan:noiseMissAdjoint:stiff', ...
        ['h = %g s is not smaller than one lag of the binomial, tau/5 = %g s. ' ...
         'Heun does not blow up here -- it returns a smooth, wrong curve -- ' ...
         'so this is refused rather than warned about.'], opts.h, opts.tau/5);
end
if opts.tf <= 2*opts.h
    error('mgs:zarchan:noiseMissAdjoint:short', ...
        'tf = %g s is not long enough for two steps at h = %g s.', opts.tf, opts.h);
end
if opts.samp < opts.h
    error('mgs:zarchan:noiseMissAdjoint:samp', ...
        ['samp = %g s is finer than the step h = %g s, so the returned curve ' ...
         'would be sampled at the step rate while claiming samp in its own ' ...
         'time vector.'], opts.samp, opts.h);
end

C   = chanTable();
xnp = opts.xnp(:).';
K   = numel(xnp);
if K == 0
    error('mgs:zarchan:noiseMissAdjoint:noxnp', ...
        ['xnp is empty, so nothing would be integrated and R.pass would ' ...
         'report on an empty board.  Table 18.1 publishes N'' = 3, 4 and 5.']);
end

% The normalization factors of Table 18.1, from the ONE exponent table.
% Dividing by these is what makes R.coef comparable with the book at any
% Vc/T/RA/Phi -- and what makes self-checks 2-4 a test of the exponents.
nf = zeros(1,4);
for c = 1:4
    nf(c) = opts.vc^C(c).p_vc * opts.tau^C(c).p_tau * sqrt(opts.phi(c)) ...
            / opts.ra^C(c).p_ra;
end

curve = struct('xnp',{}, 'coef',{}, 'sigma',{});
coef     = zeros(K,4);
coefTF10 = nan(K,4);
tail     = zeros(K,4);
monoMin  = inf;
for k = 1:K
    [t, SIG, sigEnd] = adjointRun(xnp(k), opts.tau, opts.tf, opts.h, ...
                                  opts.vc, opts.ra, opts.phi, opts.samp);
    if numel(t) < 2
        error('mgs:zarchan:noiseMissAdjoint:empty', ...
            ['the sweep recorded %d sample(s) at tf = %g s, samp = %g s.  At ' ...
             'least two are needed for any curve comparison to mean anything.'], ...
            numel(t), opts.tf, opts.samp);
    end
    CO = SIG ./ nf;                        % nf is 1x4, broadcast down the rows
    curve(k).xnp   = xnp(k);
    curve(k).coef  = CO;
    curve(k).sigma = SIG;
    coef(k,:) = sigEnd ./ nf;              % final STATE, not the final sample

    % The listing's own TF, read off the same curve -- no second integration.
    if opts.tf >= 10
        [~, j10] = min(abs(t - 10));
        coefTF10(k,:) = CO(j10,:);
    end

    % Self-check inputs.  Tail = relative change over the last 20% of the
    % sweep; that is the quantity the book's TF = 10 fails.
    j = find(t >= 0.8*opts.tf, 1);
    if isempty(j), j = 1; end
    tail(k,:) = abs(CO(end,:)./max(CO(j,:), realmin) - 1);
    monoMin   = min(monoMin, min(min(diff(CO,1,1))));
end

R = struct();
R.t         = t;                            % the last sweep's grid; all share it
R.xnp       = xnp;
R.chan      = C;
R.coef      = coef;
R.coef_tf10 = coefTF10;
R.curve     = curve;
R.miss      = @(key, vc, tau, ra, phi) missOf(coef, xnp, key, vc, tau, ra, phi);
R.p = struct('xnp',xnp, 'tau',opts.tau, 'tf',opts.tf, 'h',opts.h, ...
             'vc',opts.vc, 'ra',opts.ra, 'phi',opts.phi, 'samp',opts.samp, ...
             'normfactor',nf, ...
             'units','dimensionless; book units ft, ft/s, s, g = 32.2 ft/s^2');
R.units = ['dimensionless coefficients on the Table 18.1 normalization ' ...
           'factors; the book''s own units are ft, ft/s, s with g = 32.2 ft/s^2'];

R.selfcheck = structuralChecks(coef, tail, monoMin, xnp, opts.tau, opts.tf, ...
                               opts.h, opts.samp, opts.phi);

if opts.benchmark
    R.bench = benchmarks(coef, coefTF10, xnp);
    ev      = ~[R.bench.skipped] & ~strcmpi({R.bench.kind}, 'info');
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  "not checked" must never render as "fine".
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end

msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the book. ' ...
           'R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['no book value could be evaluated: Table 18.1 publishes ' ...
        'N'' = 3, 4 and 5 only and this run used %s.'], mat2str(xnp));
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question. ' ...
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
        error('mgs:zarchan:noiseMissAdjoint:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN COMPLETE.  benchSuite runs every port with assert
        % off precisely so one red row cannot hide the board.
        warning('mgs:zarchan:noiseMissAdjoint:benchmark', ...
            '%s  (''assert'' is false; see R.verdict)', msg);
    end
end
end


% =============================================================================
function C = chanTable()
%CHANTABLE  The four noise channels and the Table 18.1 normalization exponents
%   (:5367-5380).  sigma = coef * Vc^p_vc * T^p_tau * Phi^0.5 / RA^p_ra.
%
%   ⚠️ p_tau for GLINT is NEGATIVE.  That is not a typo -- see the header.
%   THIS IS THE ONLY PLACE THESE EXPONENTS EXIST.  R.miss reads them, and so
%   do self-checks 2-4, which re-run the integrator at altered Vc/RA/T and
%   require the coefficient to come back unchanged.  Edit a number here and
%   those checks go red; retype one somewhere else and nothing notices.
C = struct( ...
    'key',   {'fn', 'rn', 'rna', 'gl'}, ...
    'name',  {'range-independent noise', 'semiactive range-dependent noise', ...
              'active range-dependent noise', 'glint noise'}, ...
    'p_vc',  {1,   2,   3,   0}, ...
    'p_tau', {0.5, 1.5, 2.5, -0.5}, ...
    'p_ra',  {0,   1,   2,   0});
end


% =============================================================================
function [t, SIG, sigEnd] = adjointRun(XNP, TAU, TF, H, VC, RA, PHI, SAMP)
%ADJOINTRUN  Listing 18.1, zarchan.txt:5362-5372 (code lines only).
%
%   Every constant is an ARGUMENT: MATLAB local functions cannot see the main
%   function's workspace, and a previous wave in this package lost a morning
%   to exactly that.
%
%   SIG is n-by-4 [XMFN XMRN XMRNA XMGL], the listing's own sampled outputs.
%   sigEnd is the FINAL INTEGRATOR STATE.  The listing echoes the last SAMPLE
%   instead; at the book's settings the two are the same value, because
%   TF/H is a whole multiple of the sample stride, and where they differ they
%   differ by a tail below 1e-15 of the total.  The state is used because it
%   does not depend on 'samp'.

S  = 0.0;
TP = 0.0 + 0.00001;         % NOT 0: TGO is the divisor in X3D and X12D.
X2 = 0;  X3 = 1;  X4 = 0;   % <<< THE IMPULSE, on x3.  Everything else zero.
X5 = 0.; X6 = 0.; X7 = 0.; X8 = 0.;
X9 = 0.; X10 = 0.; X11 = 0.; X12 = 0.;

% Initialised before the loop so the FLAG==1 branch cannot read a stale value.
X2D=0; X3D=0; X4D=0; X5D=0; X6D=0; X7D=0; X8D=0; X9D=0; X10D=0; X11D=0; X12D=0;

nmax = ceil(TF/SAMP) + 8;
t    = zeros(nmax,1);
SIG  = zeros(nmax,4);
n    = 0;

while TP <= (TF - 1e-5)
    X2O=X2; X3O=X3; X4O=X4; X5O=X5; X6O=X6; X7O=X7; X8O=X8;
    X9O=X9; X10O=X10; X11O=X11; X12O=X12;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor.  TP advances INSIDE this loop, exactly
    % once, which is what puts the second evaluation at t+H.  Move that line
    % out and the scheme degrades to first order while still running smoothly.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            X2=X2+H*X2D; X3=X3+H*X3D; X4=X4+H*X4D; X5=X5+H*X5D;
            X6=X6+H*X6D; X7=X7+H*X7D; X8=X8+H*X8D; X9=X9+H*X9D;
            X10=X10+H*X10D; X11=X11+H*X11D; X12=X12+H*X12D;
            TP = TP + H;
        end
        % ---- the plant.  IDENTICAL to Listing 6.1 (:1723-1724). ------------
        X2D = X3;
        Y1  = 5.*(5.*X5/TAU + X4)/TAU;
        TGO = TP + 0.00001;                       % adjoint time IS time-to-go
        X3D = Y1/(VC*TGO);
        X4D = -Y1;
        X5D = -5.*X5/TAU + 5.*X6*XNP*VC/TAU;      % N' enters ONLY here
        X6D = -5.*X6/TAU + 5.*X7/TAU;
        X7D = -5.*X7/TAU + 5.*X8/TAU;
        X8D = -5.*X8/TAU - X2;
        % ---- the four noise accumulators.  ALL THAT 18.1 ADDS TO 6.1. ------
        X9D  = Y1^2;                              % range independent
        X10D = (Y1*VC*TGO/RA)^2;                  % semiactive, ~ range
        X11D = (Y1*((VC*TGO/RA)^2))^2;            % active,     ~ range^2
        X12D = (Y1/(VC*TGO))^2;                   % glint
        FLAG = 1;
    end

    % Heun corrector, in the listing's own algebra.
    X2=.5*(X2O+X2+H*X2D);    X3=.5*(X3O+X3+H*X3D);    X4=.5*(X4O+X4+H*X4D);
    X5=.5*(X5O+X5+H*X5D);    X6=.5*(X6O+X6+H*X6D);    X7=.5*(X7O+X7+H*X7D);
    X8=.5*(X8O+X8+H*X8D);    X9=.5*(X9O+X9+H*X9D);    X10=.5*(X10O+X10+H*X10D);
    X11=.5*(X11O+X11+H*X11D); X12=.5*(X12O+X12+H*X12D);

    % The listing: "S=S+H; if S >= .0999".  With H = .01 that is every tenth
    % step.  Keeping the literal .0999 would silently change meaning for a
    % different H, so it is expressed against SAMP; the -0.5*H guard is what
    % makes S = 0.1 clear a 0.1 threshold in floating point.
    S = S + H;
    if S >= SAMP - 0.5*H
        S = 0.0;
        n = n + 1;
        t(n)     = TP;
        SIG(n,:) = [sqrt(X9*PHI(1)), sqrt(X10*PHI(2)), ...
                    sqrt(X11*PHI(3)), sqrt(X12*PHI(4))];
    end
end

t = t(1:n);  SIG = SIG(1:n,:);
sigEnd = [sqrt(X9*PHI(1)), sqrt(X10*PHI(2)), sqrt(X11*PHI(3)), sqrt(X12*PHI(4))];
end


% =============================================================================
function s = missOf(coef, xnp, key, vc, tau, ra, phi)
%MISSOF  R.miss: turn a coefficient into feet.  Exponents from CHANTABLE only.
C = chanTable();
c = find(strcmpi(key, {C.key}), 1);
if isempty(c)
    error('mgs:zarchan:noiseMissAdjoint:chan', ...
        ['unknown noise channel "%s".  Known: %s.  This errors rather than ' ...
         'defaulting, because every channel here carries a DIFFERENT power ' ...
         'of T and a wrong one still returns a plausible number of feet.'], ...
        char(string(key)), strjoin({C.key}, ', '));
end
s = coef(:,c) .* vc.^C(c).p_vc .* tau.^C(c).p_tau .* sqrt(phi) ./ ra.^C(c).p_ra;
if isscalar(xnp), s = s(1); end
end


% =============================================================================
function K = structuralChecks(coef, tail, monoMin, xnp, TAU, TF, H, SAMP, PHI)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never evidence that
%   this port reproduces Zarchan -- only that it agrees with itself.
%
%   Checks 2-4 are the sharp ones.  Each re-runs the integrator with ONE of
%   Vc, RA, T altered and requires the NORMALIZED coefficient to come back
%   unchanged.  That is a direct measurement of the Table 18.1 exponents out
%   of the transcribed ODEs, and it is what settles the flattened glint
%   fraction: if p_tau for glint were +0.5 instead of -0.5, check 4 would come
%   back off by a factor of TAU, not by 1e-6.
K = struct();
names = {};  oks = [];  detail = {};

n1 = xnp(1);
ref = coef(1,:);

% 1  Steady state actually reached.  THE ONE THE BOOK'S OWN TF = 10 FAILS.
K.tail = max(tail(:));
names{end+1}  = 'steady state reached by tf';
oks(end+1)    = K.tail <= 1e-8;
detail{end+1} = sprintf('max rel change over last 20%% of sweep = %.3e (tol 1e-8)', K.tail);

% 2  Vc exponents 1, 2, 3, 0.
[~, ~, sB] = adjointRun(n1, TAU, TF, H, 7.0, 1.0, PHI, SAMP);
K.vc_rel = maxRel(sB, normOf(7.0, TAU, 1.0, PHI), ref);
names{end+1}  = 'coefficients invariant under Vc (exponents 1,2,3,0)';
oks(end+1)    = K.vc_rel <= 1e-9;
detail{end+1} = sprintf('Vc 1 -> 7: max rel change %.3e (tol 1e-9)', K.vc_rel);

% 3  RA exponents 0, 1, 2, 0.
[~, ~, sC] = adjointRun(n1, TAU, TF, H, 1.0, 3.0, PHI, SAMP);
K.ra_rel = maxRel(sC, normOf(1.0, TAU, 3.0, PHI), ref);
names{end+1}  = 'coefficients invariant under RA (exponents 0,1,2,0)';
oks(end+1)    = K.ra_rel <= 1e-9;
detail{end+1} = sprintf('RA 1 -> 3: max rel change %.3e (tol 1e-9)', K.ra_rel);

% 4  T exponents 0.5, 1.5, 2.5 and -0.5.  h/tf/samp scale with tau so the
%    DISCRETE problem is the same one; the residual left over is the two 1e-5
%    offsets on TP and TGO, which do not scale.  That is why 5e-5 and not 1e-9.
[~, ~, sD] = adjointRun(n1, 2*TAU, 2*TF, 2*H, 1.0, 1.0, PHI, 2*SAMP);
K.tau_rel = maxRel(sD, normOf(1.0, 2*TAU, 1.0, PHI), ref);
names{end+1}  = 'coefficients invariant under T (glint exponent is -0.5)';
oks(end+1)    = K.tau_rel <= 5e-5;
detail{end+1} = sprintf('T 1 -> 2 with h,tf,samp scaled: max rel change %.3e (tol 5e-5)', K.tau_rel);

% 5  The accumulators integrate SQUARES, so no curve may fall.  A sign error
%    on any of X9D..X12D shows up here and nowhere else cheap.  The test is
%    >= 0, not > 0: in the converged tail the increments underflow to exactly
%    zero, which is correct behaviour, not a stalled integrator.
K.mono_min = monoMin;
names{end+1}  = 'noise accumulators non-decreasing';
oks(end+1)    = monoMin >= -1e-15;
detail{end+1} = sprintf('min consecutive change over all channels = %+.3e', monoMin);

K.names   = names;
K.oks     = logical(oks);
K.detail  = detail;
K.all_ok  = all(K.oks);
bad       = find(~K.oks, 1);
if isempty(bad), K.first_bad = ''; else, K.first_bad = detail{bad}; end
end

function nf = normOf(vc, tau, ra, phi)
%NORMOF  The Table 18.1 normalization factors, from CHANTABLE.
C  = chanTable();
nf = zeros(1,4);
for c = 1:4
    nf(c) = vc^C(c).p_vc * tau^C(c).p_tau * sqrt(phi(c)) / ra^C(c).p_ra;
end
end

function r = maxRel(sig, nf, ref)
r = max(abs((sig ./ nf) ./ ref - 1));
end


% =============================================================================
function B = emptyBench()
B = struct('what',{}, 'kind',{}, 'book',{}, 'book_str',{}, 'ours',{}, ...
           'ours_str',{}, 'pct',{}, 'band',{}, 'tol',{}, 'ok',{}, ...
           'skipped',{}, 'why',{}, 'source',{}, 'units',{}, 'detail',{});
end

function B = addRow(B, what, kind, book, book_str, ours, ours_str, band, tol, ...
                    ok, skipped, why, source, detail)
%ADDROW  THE ONLY CONSTRUCTOR.  Field ORDER is the contract benchSuite's
%   adaptGeneric enforces strictly, so it lives in exactly one place.
i = numel(B) + 1;
B(i).what     = what;
B(i).kind     = kind;
B(i).book     = book;
B(i).book_str = book_str;
B(i).ours     = ours;
B(i).ours_str = ours_str;
if isfinite(book) && book ~= 0 && isfinite(ours)
    B(i).pct = 100*(ours/book - 1);
else
    B(i).pct = NaN;
end
B(i).band    = band;
B(i).tol     = tol;
B(i).ok      = logical(ok);
B(i).skipped = logical(skipped);
B(i).why     = why;
B(i).source  = source;
B(i).units   = '';          % the coefficients are dimensionless
B(i).detail  = detail;
end


% =============================================================================
function B = benchmarks(coef, coefTF10, xnp)
%BENCHMARKS  Ten rows, six scored.  See the header for every band's provenance.
%   Do NOT widen a band to turn a row green.
B = emptyBench();
C = chanTable();

% Table 18.1, rows = channel, columns = N' 3/4/5  (zarchan.txt:5367-5380).
BK5 = [3.04 5.08  8.19 ; ...
       9.47 18.4  33.7 ; ...
       41.3 89.4  182  ; ...
       1.68 2.35  3.21];
% The book's SINGLE-LAG coefficients, same layout (:5290-5344).  Used only as
% the comparison in the two unscored/shape rows below.
BK1 = [0.532 0.561 0.588; ...
       1.06  1.10  1.15 ; ...
       4.66  4.68  4.82 ; ...
       1.44  1.71  1.94];
SRC = {'Table 18.1 p.396 range-independent row; zarchan.txt:5367-5369', ...
       'Table 18.1 p.396 semiactive row; zarchan.txt:5370-5373', ...
       'Table 18.1 p.396 active row; zarchan.txt:5374-5377', ...
       'Table 18.1 p.396 glint row; zarchan.txt:5378-5380'};
TOL = 0.01;                                   % band 'text': printed to 3 figures

% ---- rows 1-4: the N' = 3 column, one row per channel ----------------------
k3 = find(abs(xnp - 3) < 1e-12, 1);
for c = 1:4
    what = sprintf('Table 18.1 coefficient, %s, N''=3', C(c).name);
    if isempty(k3)
        B = addRow(B, what, 'value', BK5(c,1), sprintf('%.3g', BK5(c,1)), ...
            NaN, '--', 'text', TOL, false, true, ...
            sprintf('N''=3 was not run (xnp = %s)', mat2str(xnp)), SRC{c}, '');
    else
        ours = coef(k3,c);
        B = addRow(B, what, 'value', BK5(c,1), sprintf('%.3g', BK5(c,1)), ...
            ours, sprintf('%.5g', ours), 'text', TOL, ...
            abs(ours/BK5(c,1) - 1) <= TOL, false, '', SRC{c}, ...
            sprintf('sigma = %.5g * Vc^%g * T^%g * Phi^0.5 / RA^%g', ...
                    ours, C(c).p_vc, C(c).p_tau, C(c).p_ra));
    end
end

% ---- rows 5-6: the N' = 4 and N' = 5 columns, aggregated -------------------
for col = 2:3
    NP   = col + 2;
    what = sprintf('Table 18.1 column N''=%d, all four channels', NP);
    src  = sprintf(['Table 18.1 p.396, N''=%d column; zarchan.txt:5366 ' ...
                    '(heading, "N0 ¼ %d", hand-decoded) and :5369,5373,5377,5380'], ...
                   NP, NP);
    kk = find(abs(xnp - NP) < 1e-12, 1);
    if isempty(kk)
        B = addRow(B, what, 'skip', NaN, sprintf('%.3g/%.3g/%.3g/%.3g', BK5(:,col)), ...
            NaN, '--', 'all four within 1%', TOL, false, true, ...
            sprintf('N''=%d was not run (xnp = %s)', NP, mat2str(xnp)), src, '');
        continue
    end
    dev = abs(coef(kk,:).'./BK5(:,col) - 1);
    d   = '';
    for c = 1:4
        d = [d sprintf('%s %.5g vs %.3g (%+.3f%%)  ', C(c).key, coef(kk,c), ...
                       BK5(c,col), 100*(coef(kk,c)/BK5(c,col) - 1))]; %#ok<AGROW>
    end
    B = addRow(B, what, 'bound', 0, ...
        sprintf('%.3g / %.3g / %.3g / %.3g', BK5(:,col)), ...
        max(dev), sprintf('max |dev| %.3f%%', 100*max(dev)), ...
        'all four within 1%', TOL, max(dev) <= TOL, false, '', src, strtrim(d));
end

% ---- row 7: SHAPE, monotone in N' -----------------------------------------
haveAll = numel(xnp) >= 2;
if haveAll
    up = all(all(diff(coef(:,:),1,1) > 0));
    B = addRow(B, 'coefficients increase with N'' in all four channels', ...
        'shape', NaN, 'increasing', double(up), boolStr(up), 'ordering', 0, ...
        up, false, '', ...
        ['prose p.395: "Increasing the effective navigation ratio ' ...
         'significantly increases the miss distance due to noise"; ' ...
         'zarchan.txt:5360'], ...
        sprintf('xnp = %s', mat2str(xnp)));
else
    B = addRow(B, 'coefficients increase with N'' in all four channels', ...
        'skip', NaN, 'increasing', NaN, '--', 'ordering', 0, false, true, ...
        'needs at least two navigation ratios in xnp', 'zarchan.txt:5360', '');
end

% ---- row 8: SHAPE, N' matters MORE here than for a single lag --------------
k5 = find(abs(xnp - 5) < 1e-12, 1);
if ~isempty(k3) && ~isempty(k5)
    r5 = coef(k5,:).' ./ coef(k3,:).';        % ours,  N'=5 over N'=3
    r1 = BK1(:,3) ./ BK1(:,1);                % book's single-lag, same ratio
    ok8 = all(r5 > r1);
    dd = '';
    for c = 1:4
        dd = [dd sprintf('%s %.3f vs single-lag %.3f  ', C(c).key, r5(c), r1(c))]; %#ok<AGROW>
    end
    B = addRow(B, 'N'' sensitivity exceeds the single-lag system''s', 'shape', ...
        NaN, 'ours > single-lag, all 4', double(ok8), boolStr(ok8), ...
        'ordering', 0, ok8, false, '', ...
        ['prose p.395: "unlike the single time constant guidance system, the ' ...
         'effective navigation ratio has a strong influence"; zarchan.txt:5357-5360, ' ...
         'single-lag coefficients :5290-5344'], strtrim(dd));
else
    B = addRow(B, 'N'' sensitivity exceeds the single-lag system''s', 'skip', ...
        NaN, 'ours > single-lag, all 4', NaN, '--', 'ordering', 0, false, true, ...
        'needs both N''=3 and N''=5 in xnp', 'zarchan.txt:5357-5360', '');
end

% ---- row 9: INFO, the "order of magnitude" claim ---------------------------
if ~isempty(k3)
    rr = coef(k3,:).' ./ BK1(:,1);
    B = addRow(B, 'fifth-order vs single-lag coefficient ratio, N''=3', 'info', ...
        NaN, '"an order of magnitude larger"', NaN, ...
        sprintf('fn %.2fx  rn %.2fx  rna %.2fx  gl %.2fx', rr), ...
        'not scored', NaN, false, false, ...
        ['NOT ADJUDICATED: the book''s claim holds for three channels and ' ...
         'fails for GLINT (about 1.2x, not 10x), so any band that passed ' ...
         'this row would be a band the book itself violates.'], ...
        'prose p.395; zarchan.txt:5357, single-lag values :5290-5344', '');
end

% ---- row 10: INFO, the listing's own TF = 10 -------------------------------
if any(isfinite(coefTF10(:)))
    dq = abs(coefTF10./coef - 1);
    [~, iw] = max(dq(:));
    [rw, cw] = ind2sub(size(dq), iw);
    % SIGNED, not the abs used to locate it: the direction is the finding.
    sg = 100*(coefTF10(rw,cw)/coef(rw,cw) - 1);
    B = addRow(B, 'Listing 18.1 as printed (TF = 10) vs converged', 'info', ...
        NaN, 'the listing prints TF=10.', NaN, ...
        sprintf('worst %s at N''=%g: %.5g vs %.5g (%+.3f%%)', ...
                C(cw).key, xnp(rw), coefTF10(rw,cw), coef(rw,cw), sg), ...
        'not scored', NaN, false, false, ...
        ['NOT ADJUDICATED: a finding about the book, not a published number. ' ...
         'TF = 10 is not steady state for N'' >= 4 -- the active channel ' ...
         'weights the integrand by tgo^4 -- so the listing run exactly as ' ...
         'printed misses its own Table 18.1 by up to about 1.5%, silently.'], ...
        'Listing 18.1 line 1; zarchan.txt:5362', ...
        sprintf('tf default is 25 for this reason; pass ''tf'',10 to reproduce'));
end
end

function s = boolStr(b)
if b, s = 'holds'; else, s = 'VIOLATED'; end
end


% =============================================================================
function s = render(R, ev)
C = chanTable();
L = {'mgs.zarchan.noiseMissAdjoint  --  Zarchan Listing 18.1 (Ch.18, p.396)'};
L{end+1} = sprintf('  tau %g s  tf %g s  h %g s  samp %g s  Vc %g  RA %g  phi %s', ...
    R.p.tau, R.p.tf, R.p.h, R.p.samp, R.p.vc, R.p.ra, mat2str(R.p.phi));
L{end+1} = ['  units: ' R.units];
L{end+1} = '';
L{end+1} = 'TABLE 18.1 COEFFICIENTS   sigma = coef * Vc^a * T^b * Phi^0.5 / RA^c';
L{end+1} = sprintf('   %-4s %12s %12s %12s %12s', 'N''', C.key);
for k = 1:numel(R.xnp)
    L{end+1} = sprintf('   %-4g %12.5g %12.5g %12.5g %12.5g', R.xnp(k), R.coef(k,:)); %#ok<AGROW>
end
L{end+1} = sprintf('   a,b,c    %s', ...
    sprintf('%g/%g/%g  ', [[C.p_vc]; [C.p_tau]; [C.p_ra]]));
L{end+1} = '';
L{end+1} = 'STRUCTURAL CHECKS  (self-consistency, NOT book values)';
for i = 1:numel(R.selfcheck.names)
    L{end+1} = sprintf('   %-52s %-8s %s', R.selfcheck.names{i}, ...
        boolStr(R.selfcheck.oks(i)), R.selfcheck.detail{i}); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = 'BOOK BENCHMARKS';
for i = 1:numel(R.bench)
    b = R.bench(i);
    if strcmpi(b.kind, 'info')
        L{end+1} = sprintf('   INFO  %-46s %s', b.what, b.ours_str); %#ok<AGROW>
    elseif strcmpi(b.kind, 'skip') || b.skipped
        L{end+1} = sprintf('   SKIP  %-46s %s', b.what, b.why); %#ok<AGROW>
    elseif any(strcmpi(b.kind, {'shape','bound'}))
        L{end+1} = sprintf('   %-5s %-46s book %-26s ours %s', upper(b.kind), ...
            b.what, b.book_str, b.ours_str); %#ok<AGROW>
    else
        L{end+1} = sprintf('   %-5s %-46s book %-8s ours %-10s %+7.3f%%  band %g%%', ...
            boolStr(b.ok), b.what, b.book_str, b.ours_str, b.pct, 100*b.tol); %#ok<AGROW>
    end
end
if ~any(ev)
    L{end+1} = '   *** nothing was adjudicated -- R.pass false means NOT CHECKED ***';
end
s = strjoin(L, newline);
end
