function R = fcsBode(opts)
%MGS.ZARCHAN.FCSBODE  Zarchan Listing 22.3 -- open-loop Bode response of the
%   rate-gyro flight-control system, and the stability margins read off it.
%
%   R = mgs.zarchan.fcsBode()
%   R = mgs.zarchan.fcsBode('KR', 0.2, 'assert', false)   % the Fig. 22.18 case
%   R = mgs.zarchan.fcsBode('plot', true)
%
%   Line-by-line port of Listing 22.3 (zarchan.txt:7150-7152).  The listing
%   sweeps 159 log-spaced frequencies, W = 10^(0.025*I - 1) for I = 2:160, and
%   at each one evaluates the FACTORED magnitude and phase of
%
%       HG(jw) = K3*KR*(1 + jw*Ta)
%                -----------------------------------------------------------
%                [1 - w^2/wAF^2  + j*2*zAF*w/wAF][1 - w^2/wACT^2 + j*2*zACT*w/wACT]
%
%   printed as two separate equations on p.517 (zarchan.txt:7124, :7148).  No
%   numerical integration is involved, which is the book's own remark about
%   this program (zarchan.txt:7158).  bode() is NOT used: base MATLAB only, so
%   the polynomial is evaluated at s = jw here, exactly as the listing does.
%
% ===========================================================================
%   UNITS.  rad/s, dB (20*log10), degrees.  The transfer function itself is
%   DIMENSIONLESS -- no length appears anywhere in this listing; the constants
%   come from the linearised sea-level airframe of Ch.21/22 for the book's
%   3000 ft/s missile, so they are ft-derived even though no foot is visible.
%   Degrees use the LISTING'S OWN factor 57.3, not 180/pi: that inflates every
%   angle by 0.0074% and moves the phase crossover by 0.03 rad/s.  Kept.
% ===========================================================================
%
%   WHAT IT CANNOT SAY.  A LINEAR, FROZEN-COEFFICIENT, SINGLE-FLIGHT-CONDITION
%   response of the INNER loop only, broken at the actuator input where the
%   designer controls KR (Fig. 22.15).  No guidance loop, no seeker, no
%   airframe nonlinearity, no rate or deflection limit.  These are margins of
%   the AUTOPILOT, not of a homing loop, and they are the margins of a system
%   ASSUMED TO STAY LINEAR -- they say nothing about a channel whose real
%   failure mode is saturation.  Its use here is as the frequency-domain twin
%   of MGS.ZARCHAN.FLIGHTCONTROLDELAY (Listing 22.4), which takes the 55 deg
%   and 11.5 dB as given; this file COMPUTES them, sharing no code with it.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_22_03.m reports no unresolved
%   variable-to-variable comparisons, and that is correct -- Listing 22.3
%   contains no comparison at all.  Three repairs, none a judgement call:
%   the run-together plotting tail is split into its two figures (under
%   'plot'); "save datfil.txt output /ascii" is the book's recurring typo for
%   -ascii and nothing is written to disk here at all, so it is recorded and
%   dropped; "clc" and "disp 'simulation finished'" are interactive noise.
%
%   Two of the listing's quirks are reproduced DELIBERATELY:
%
%     * GAIN = 20*log10(-K3*KR*...).  K3 = -1.89, so -K3*KR = +0.189.  THE
%       MINUS SIGN IS LOAD-BEARING: "fix" it to K3 and log10 of a negative
%       returns a COMPLEX gain, which semilogx plots by silently discarding
%       the imaginary part.  The sign it removes is the negative-feedback
%       convention -- which is also why the listing's PHASE starts at 0 deg
%       and not 180: PHASE is the phase of -HG(jw), so "phase = -180" is the
%       stability boundary.  Structural check 2 states exactly that; the
%       argument is also refused up front rather than allowed to go complex.
%     * The sweep starts at W = 10^-0.95 = 0.1122 rad/s (I = 2), not at the
%       0.1 the plot axis suggests.  The axis limit is a plot range the author
%       chose, not a datum.
%
%   ⚠️ THE GAIN CROSSES 0 dB TWICE -- up at about 9.5 rad/s on the way to the
%   airframe resonance, back down at about 64.  The book's crossover is the
%   SECOND.  This port takes the LAST crossing and reports the count; taking
%   the first returns 9.5 rad/s, a perfectly plausible number that then fails
%   against 64 for a reason nobody would guess from the symptom.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS -- every book value was grepped out of zarchan.txt and the line
%   number is in its row's .source.  Nothing was read off a figure; where the
%   book itself quotes a figure, the row says so and carries a READ-OFF band,
%   not a precision band.  Phase at ω_gc = -125 deg and gain at ω_180 =
%   -11.5 dB (both :7159); ω_gc = 64 and ω_180 = 150 rad/s (:7164);
%   K_UNSTABLE = 0.376 (:7166-7173); approximate ω_CR = 55.2 rad/s (:7224);
%   ω_gc at KR = 0.2 is 103 rad/s (:7241).
%
%   The two MARGINS the book headlines -- 55 deg and 11.5 dB, both :7159 --
%   are the arithmetic complements of the first two.  render() prints them;
%   they are NOT scored, because a second row for the same measurement would
%   double-count it and inflate the pass tally.
%
%   ⚠️ AN ERRATUM, and why 60 rad/s is NOT scored.  The book gives the actual
%   gain crossover TWICE and the two disagree: 64 rad/s on p.519 (:7164, from
%   Fig. 22.16) and 60 rad/s on p.525/p.526 (:7225, :7241, from Fig. 22.18).
%   This listing computes 64.4, so p.519 is the right one.  The consequence is
%   downstream: the claim at :7229 that the approximate formula is "in error
%   by slightly less than 10%" holds against the 60 (-8.0%) and is FALSE
%   against the crossover the listing produces (-14.2%).  That bound is
%   reported as INFO with its arithmetic, NOT widened into a passing row.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED BY HAND, NOT MEASURED. ***  This file
%   has never been executed; MATLAB is the caller's.  A first run can be laid
%   against the expectation instead of merely admired:
%
%     gain crossover     64.45 rad/s   (book 64)      +0.7%
%     phase at crossover -125.30 deg   (book -125)    +0.24%   -> PM 54.70 deg
%     phase crossover    150.6 rad/s   (book 150)     +0.4%
%     gain at -180 deg   -11.42 dB     (book -11.5)   -0.7%    -> GM 11.42 dB
%     K_UNSTABLE         0.3722        (book 0.376)   -1.0%
%     approx formula     55.29 rad/s   (book 55.2)    +0.17%
%     ω_gc at KR = 0.2   105.9 rad/s   (book 103)     +2.8%
%     resonant peak      about +25.5 dB near 25.1 rad/s (book publishes none)
%     gain at 1000 rad/s about -58 dB, slope about -60 dB/decade
%
%   A residual near a band EDGE is not a pass to be accepted.  Do not widen a
%   band to make a row green.
%
%   OUTPUTS
%     R.w R.gain_db R.phase_deg   the listing's three output columns
%     R.wgc R.phase_at_wgc R.pm_deg      gain crossover and phase margin
%     R.wpc R.gain_at_wpc R.gm_db        phase crossover and gain margin
%     R.k_unstable                 KR*10^(GM/20)
%     R.wcr_approx                 KR*|K3|*Ta*wAF^2 (the book's p.525 formula)
%     R.wgc2                       gain crossover at opts.KR2
%     R.bench R.selfcheck R.pass R.verdict R.p R.summary R.units
%
%   'assert' chooses the reaction to a bad verdict and nothing else: true
%   errors, false WARNS and still returns R complete.  benchSuite runs every
%   port with assert off so that one red row cannot hide the board.
%
%   See also MGS.ZARCHAN.FLIGHTCONTROLDELAY (Listing 22.4 -- the time-domain
%   confirmation of these margins), MGS.ZARCHAN.AIRFRAMELINEAR (where K3, Ta,
%   zAF and wAF come from), MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 22.3 line 1, verbatim.
    opts.ZACT (1,1) double {mustBeFinite, mustBePositive} = 0.7
    opts.WACT (1,1) double {mustBeFinite, mustBePositive} = 150.0   % rad/s
    opts.K3   (1,1) double {mustBeFinite}                 = -1.89   % NEGATIVE
    opts.TA   (1,1) double {mustBeFinite, mustBePositive} = 0.457   % s
    opts.ZAF  (1,1) double {mustBeFinite, mustBePositive} = 0.058
    opts.WAF  (1,1) double {mustBeFinite, mustBePositive} = 25.3    % rad/s
    opts.KR   (1,1) double {mustBeFinite, mustBePositive} = 0.1
    % The second autopilot gain of Fig. 22.18.  A SECOND FULL SWEEP, not a
    % rescale: the crossover moves, so the interpolation must be redone.
    opts.KR2  (1,1) double {mustBeFinite, mustBePositive} = 0.2
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so an "if nargin == 0" here would be a mode that never fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

DEG = 57.3;                 % the listing's own rad->deg factor.  See header.

% The listing evaluates 20*log10(-K3*KR*...).  REFUSED, not warned about:
% log10 of a non-positive number does not error in MATLAB, it returns a
% COMPLEX number (or -Inf), and every margin below would then come off a curve
% offset by 27.3 dB with no symptom anywhere.
for kr = [opts.KR, opts.KR2]
    if ~(-opts.K3*kr > 0)
        error('mgs:zarchan:fcsBode:sign', ...
            ['-K3*KR must be POSITIVE, because the listing takes its log10. ' ...
             'With K3 = %g and KR = %g it is %g.  Zarchan''s K3 is NEGATIVE ' ...
             '(K3 = -1.89); the minus sign in the listing is the ' ...
             'negative-feedback convention, not a typo to be tidied away.'], ...
            opts.K3, kr, -opts.K3*kr);
    end
end

% =========================================================================
%   LISTING 22.3.  Transcribed from zarchan.txt:7150-7152.
% =========================================================================
[w, gain_db, phase_deg] = bodeSweep(opts.K3, opts.KR, opts.TA, ...
                                    opts.ZAF, opts.WAF, opts.ZACT, opts.WACT, DEG);

% ---- margins, by linear interpolation in LOG frequency between the
%      listing's own samples -- the same operation a reader performs on
%      Fig. 22.16, and it uses no information the listing did not produce.
xg   = crossings(w, gain_db, 0.0);        % may be more than one; see header
xp   = crossings(w, phase_deg, -180.0);
wgc  = lastOf(xg);                        % the DOWNWARD 0 dB crossing
wpc  = firstOf(xp);
phase_at_wgc = interpLog(w, phase_deg, wgc);
gain_at_wpc  = interpLog(w, gain_db,   wpc);
pm_deg = 180 + phase_at_wgc;
gm_db  = -gain_at_wpc;
k_unstable = opts.KR * 10^(gm_db/20);

% p.525 (zarchan.txt:7224): wCR ~ KR*K3*Ta*wAF^2, with |K3|.
wcr_approx = opts.KR * abs(opts.K3) * opts.TA * opts.WAF^2;

% Fig. 22.18: the same sweep at the doubled autopilot gain.
[w2, gain2_db] = bodeSweep(opts.K3, opts.KR2, opts.TA, ...
                           opts.ZAF, opts.WAF, opts.ZACT, opts.WACT, DEG);
wgc2 = lastOf(crossings(w2, gain2_db, 0.0));

R = struct();
R.w = w;  R.gain_db = gain_db;  R.phase_deg = phase_deg;
R.wgc = wgc;  R.phase_at_wgc = phase_at_wgc;  R.pm_deg = pm_deg;
R.wpc = wpc;  R.gain_at_wpc = gain_at_wpc;    R.gm_db = gm_db;
R.k_unstable = k_unstable;
R.wcr_approx = wcr_approx;
R.wgc2 = wgc2;
R.n_gain_crossings  = numel(xg);
R.n_phase_crossings = numel(xp);
R.units = ['rad/s, dB (20 log10), deg via 57.3; HG is dimensionless -- ' ...
           'constants are from Zarchan''s ft/s airframe, NOT SI'];
R.p = struct('ZACT',opts.ZACT, 'WACT',opts.WACT, 'K3',opts.K3, 'TA',opts.TA, ...
             'ZAF',opts.ZAF, 'WAF',opts.WAF, 'KR',opts.KR, 'KR2',opts.KR2, ...
             'DEG',DEG, 'n_points',numel(w), 'units',R.units);

R.selfcheck = structuralChecks(R, opts, DEG);

% "the book's own case" gates.  plantCase leaves KR free, so changing only the
% autopilot gain still leaves the SHAPE row scored; bookCase pins KR too.
plantCase = abs(opts.K3 + 1.89) < 1e-12 && abs(opts.TA - 0.457) < 1e-12 && ...
            abs(opts.ZAF - 0.058) < 1e-12 && abs(opts.WAF - 25.3) < 1e-12 && ...
            abs(opts.ZACT - 0.7) < 1e-12 && abs(opts.WACT - 150) < 1e-12;
bookCase  = plantCase && abs(opts.KR - 0.1) < 1e-12;

if opts.benchmark
    R.bench = benchmarks(R, opts, bookCase, plantCase);
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
if opts.plot,   drawCurves(R); end

% ---- adjudication.  The verdict is FORMED either way.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated: every published ' ...
        'number in this section is for the sea-level rate-gyro FCS ' ...
        '(K3=-1.89, Ta=0.457, zAF=0.058, wAF=25.3, zACT=0.7, wACT=150, ' ...
        'KR=0.1).  This run used KR = %g and K3 = %g.'], opts.KR, opts.K3);
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
        error('mgs:zarchan:fcsBode:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN, complete.  A port that errors here hands back
        % nothing, and then nobody can see which side of a comparison moved.
        warning('mgs:zarchan:fcsBode:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [W, GAIN, PHASE] = bodeSweep(K3, KR, TA, ZAF, WAF, ZACT, WACT, DEG)
%BODESWEEP  Listing 22.3's loop, zarchan.txt:7150-7152, verbatim.
%
%   Every constant arrives as an ARGUMENT.  MATLAB local functions cannot see
%   the main function's workspace; a previous wave in this package read a
%   constant from the caller and threw on the first line of the first run,
%   which static checking could not see beforehand.
%
%   The loop is left as a loop rather than vectorised: 159 iterations cost
%   nothing, and the book's remark (zarchan.txt:7158) that this program "runs
%   extremely rapidly because numerical integration is not involved" is about
%   this shape.

count = 0;
n     = 159;                                   % I = 2:160
W     = zeros(n,1);  GAIN = zeros(n,1);  PHASE = zeros(n,1);

for I = 2:160
    Wi     = 10^(0.025*I - 1);
    XMAG1  = sqrt(1 + (Wi*TA)^2);
    XMAG2  = sqrt((1 - (Wi/WAF)^2)^2  + (2*ZAF*Wi/WAF)^2);
    XMAG3  = sqrt((1 - (Wi/WACT)^2)^2 + (2*ZACT*Wi/WACT)^2);
    % -K3 and not K3: K3 is negative and log10 of a negative is COMPLEX, not
    % an error.  The sign removed here is the negative-feedback convention.
    GAINi  = 20*log10(-K3*KR*XMAG1/(XMAG2*XMAG3));
    PHASE1 = DEG*atan2(Wi*TA, 1.0);
    PHASE2 = DEG*atan2(2*ZAF*Wi/WAF,   1 - (Wi/WAF)^2);
    PHASE3 = DEG*atan2(2*ZACT*Wi/WACT, 1 - (Wi/WACT)^2);
    PHASEi = PHASE1 - PHASE2 - PHASE3;

    count = count + 1;
    W(count) = Wi;  GAIN(count) = GAINi;  PHASE(count) = PHASEi;
end
W = W(1:count);  GAIN = GAIN(1:count);  PHASE = PHASE(1:count);
end


% =============================================================================
function x = crossings(W, Y, level)
%CROSSINGS  Every crossing of Y through `level`, interpolated LINEARLY IN
%   log10(W) -- the grid is logarithmic, so interpolating in W itself would
%   bias every crossing towards the upper sample.  Returns ALL of them: the
%   caller picks, and says which.  The gain curve crosses 0 dB twice and
%   silently returning the first is a wrong answer with no symptom.
d = Y(:).' - level;
s = d >= 0;                 % >= not >, so a sample exactly on the level is
                            % one crossing rather than two adjacent ones
x = zeros(1,0);
for k = 1:(numel(d)-1)
    if s(k) ~= s(k+1)
        f = d(k)/(d(k) - d(k+1));                    % denominator cannot be 0
        lw = log10(W(k)) + f*(log10(W(k+1)) - log10(W(k)));
        x(end+1) = 10^lw; %#ok<AGROW>
    end
end
end

function v = firstOf(x)
if isempty(x), v = NaN; else, v = x(1); end
end

function v = lastOf(x)
if isempty(x), v = NaN; else, v = x(end); end
end

% =============================================================================
function y = interpLog(W, Y, wq)
%INTERPLOG  Y at wq, linear in log10(W) between the listing's own samples.
%   NaN outside the swept band rather than an extrapolation.  The band test
%   carries 1e-9 RELATIVE slack: crossings() returns 10^(log10(W(k)) + f*d),
%   which at f = 0 can land an ulp below W(1), and a bare "wq < W(1)" would
%   then let find() return EMPTY -- an empty k propagates as an EMPTY y, which
%   is not NaN, trips no isfinite guard, and makes a row a 0x0 struct field.
y = NaN;
if ~isfinite(wq) || wq < W(1)*(1-1e-9) || wq > W(end)*(1+1e-9), return, end
wq = min(max(wq, W(1)), W(end));
k = find(W <= wq, 1, 'last');
if isempty(k), y = Y(1); return, end
if k >= numel(W), y = Y(end); return, end
f = (log10(wq) - log10(W(k)))/(log10(W(k+1)) - log10(W(k)));
y = Y(k) + f*(Y(k+1) - Y(k));
end


% =============================================================================
function K = structuralChecks(R, opts, DEG)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.
K = struct();
names = {};  oks = [];  detail = {};
w = R.w;

% 1  The frequency grid IS the listing's: 159 points, 40 per decade, ending
%    exactly at 1000 rad/s.  Cheap, and it is the only thing that notices
%    someone "tidying" the loop bounds to 1:160 or the exponent to 0.025*I.
dl = diff(log10(w));
K.grid_resid = max([abs(numel(w) - 159), ...
                    abs(log10(w(1)) + 0.95), ...
                    abs(log10(w(end)) - 3.0), ...
                    max(abs(dl - 0.025))]);
names{end+1}  = 'grid is 10^(0.025 I - 1), I = 2:160';
oks(end+1)    = K.grid_resid <= 1e-12;
detail{end+1} = sprintf('%d points, %.4f .. %.1f rad/s, worst resid %.3e', ...
    numel(w), w(1), w(end), K.grid_resid);

% 2  THE FACTORED FORM IS THE TRANSFER FUNCTION.  Rebuild HG(jw) as a complex
%    ratio and compare.  This is the row that states what the sign convention
%    MEANS: GAIN is 20log10|HG| (K3's sign does not survive an absolute value)
%    and PHASE is the phase of -HG, which is why it starts at 0 deg and why
%    -180 is the stability boundary.  Compared modulo a full turn, because
%    atan2's principal value wraps and the listing's running sum does not --
%    and a full turn in the listing's own "degrees" is DEG*2*pi = 360.0265,
%    NOT 360.  A literal 360 leaves a 0.027 deg sawtooth past the crossover:
%    small enough to read as rounding, 2.7e5 times the real residual.
num = 1 + 1i*w*opts.TA;
d1  = 1 - (w/opts.WAF).^2  + 1i*2*opts.ZAF*w/opts.WAF;
d2  = 1 - (w/opts.WACT).^2 + 1i*2*opts.ZACT*w/opts.WACT;
HG  = opts.K3*opts.KR*num./(d1.*d2);
gRes   = max(abs(20*log10(abs(HG)) - R.gain_db));
pExact = DEG*atan2(imag(-HG), real(-HG));
dp     = R.phase_deg - pExact;
period = DEG*2*pi;
pRes   = max(abs(mod(dp + period/2, period) - period/2));
K.tf_gain_resid  = gRes;
K.tf_phase_resid = pRes;
names{end+1}  = 'factored GAIN/PHASE == 20log10|HG| and angle(-HG)';
oks(end+1)    = gRes <= 1e-9 && pRes <= 1e-7;
detail{end+1} = sprintf('max |dGain| = %.3e dB, max |dPhase| = %.3e deg', gRes, pRes);

% 3  Ordering: 0 < w_gc < w_pc.  Equivalent to "both margins are positive",
%    i.e. the designed system is stable -- the premise of the whole section.
%    Fails loudly if the crossing PICK (last for gain, first for phase) is
%    ever swapped, which is otherwise a silent 9.5-rad/s answer.
K.wgc = R.wgc;  K.wpc = R.wpc;
names{end+1}  = 'w_gc < w_pc (both margins positive)';
oks(end+1)    = isfinite(R.wgc) && isfinite(R.wpc) && R.wgc > 0 && R.wgc < R.wpc;
detail{end+1} = sprintf('w_gc = %.3f, w_pc = %.3f rad/s (%d and %d crossings found)', ...
    R.wgc, R.wpc, R.n_gain_crossings, R.n_phase_crossings);

% 4  crossings() and interpLog() are two implementations of the same
%    log-frequency interpolation and each margin uses ONE OF EACH, so put the
%    second back through the first's answer: gain at w_gc must return 0 dB and
%    phase at w_180 must return -180 deg.  An off-by-one in interpLog's
%    bracket search hands back the NEIGHBOURING sample -- a plausible-looking
%    wrong phase with no other symptom.  max() OMITS NaN in MATLAB, so both
%    sides are tested for finiteness explicitly rather than through the max.
K.gain_at_wgc  = interpLog(R.w, R.gain_db,   R.wgc);
K.phase_at_wpc = interpLog(R.w, R.phase_deg, R.wpc);
K.round_trip   = max(abs(K.gain_at_wgc), abs(K.phase_at_wpc + 180));
names{end+1}  = 'crossings() and interpLog() agree on the same bracket';
oks(end+1)    = isfinite(K.gain_at_wgc) && isfinite(K.phase_at_wpc) && ...
                K.round_trip <= 1e-8;
detail{end+1} = sprintf('gain at w_gc = %.3e dB, phase at w_180 = %+.9f deg', ...
    K.gain_at_wgc, K.phase_at_wpc);

% 5  High-frequency roll-off: one zero over four poles is -60 dB/decade.
%    Catches a whole denominator factor being dropped -- which moves the
%    margins but leaves the curve looking entirely reasonable.
%    ⚠️ ONLY MEANINGFUL once the sweep is past both natural frequencies, so it
%    is GATED and the gate is reported.  At the book's constants that is
%    900 <= 1000 and the slope lands at -59.9.
K.hf_slope    = (R.gain_db(end) - R.gain_db(end-1)) / ...
                (log10(R.w(end)) - log10(R.w(end-1)));
hfGate        = 6*max(opts.WAF, opts.WACT);
K.hf_scored   = R.w(end) >= hfGate;
names{end+1}  = 'high-frequency slope == -60 dB/decade';
oks(end+1)    = ~K.hf_scored || abs(K.hf_slope + 60) <= 1.5;
if K.hf_scored
    detail{end+1} = sprintf('%.2f dB/decade at the top (%.3f dB at %.0f rad/s)', ...
        K.hf_slope, R.gain_db(end), R.w(end));
else
    detail{end+1} = sprintf(['NOT SCORED: sweep ends at %.0f rad/s, below ' ...
        '6*max(wAF,wACT) = %.0f, so the asymptote is outside the band. ' ...
        'Slope there is %.2f dB/decade'], R.w(end), hfGate, K.hf_slope);
end

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
function B = benchmarks(R, opts, bookCase, plantCase)
%BENCHMARKS  Computed vs published, with the PROVENANCE of every band.
%     'text'      0.5%  prose arithmetic printed with its own factors
%     'fig-text'  2%    2-3 significant figures quoted in PROSE from a figure;
%                       half a unit in the last digit of "64" is 0.78%
%     'read-off'  3-5%  the book reading a curve off a plot.  A READ-OFF BAND,
%                       NOT A PRECISION CLAIM.
%     'shape'           an ordering; no tolerance, and none is possible
%   Do NOT widen a band to turn a row green.
B = emptyBench();
whyCase = sprintf(['off the listing''s own case: every published number in ' ...
    'this section is for K3=-1.89, Ta=0.457, zAF=0.058, wAF=25.3, zACT=0.7, ' ...
    'wACT=150, KR=0.1.  This run used KR=%g, K3=%g, Ta=%g, zAF=%g, wAF=%g, ' ...
    'zACT=%g, wACT=%g'], opts.KR, opts.K3, opts.TA, opts.ZAF, opts.WAF, ...
    opts.ZACT, opts.WACT);

% ---- 1-4.  The four INDEPENDENT numbers the book prints about Fig. 22.16.
% The margins (55 deg, 11.5 dB) are their complements: render() prints them,
% nothing scores them.  See the header.
if bookCase
    B = addVal(B, 'gain crossover frequency w_gc', 64.0, R.wgc, ...
        'fig-text', 0.02, 'p.519 text (zarchan.txt:7164)', 'rad/s', ...
        sprintf(['LAST of %d zero-dB crossings -- the curve also rises ' ...
                 'through 0 dB at about 9.5 rad/s before the resonance'], ...
                 R.n_gain_crossings));
    B = addVal(B, 'phase at w_gc', -125.0, R.phase_at_wgc, ...
        'fig-text', 0.02, 'p.519 text (zarchan.txt:7159)', 'deg', ...
        sprintf('phase margin follows as 180 %+.2f = %.2f deg (book 55, :7159)', ...
                R.phase_at_wgc, R.pm_deg));
    B = addVal(B, 'phase crossover frequency w_180', 150.0, R.wpc, ...
        'fig-text', 0.02, 'p.519 text (zarchan.txt:7164)', 'rad/s', ...
        ['essentially the actuator natural frequency: the airframe pair has ' ...
         'already contributed nearly its full 180 deg by here']);
    B = addVal(B, 'gain at w_180', -11.5, R.gain_at_wpc, ...
        'read-off', 0.03, 'p.519 text (zarchan.txt:7159)', 'dB', ...
        sprintf(['gain margin follows as %.2f dB (book 11.5, :7159).  NOTE ' ...
                 'the band is RELATIVE ON THE dB NUMBER, as the shared row ' ...
                 'schema scores: 3%% of -11.5 dB is 0.345 dB'], R.gm_db));
else
    B = addSkip(B, 'gain crossover frequency w_gc', '64', whyCase, ...
        'p.519 text (zarchan.txt:7164)', 'rad/s');
    B = addSkip(B, 'phase at w_gc', '-125', whyCase, ...
        'p.519 text (zarchan.txt:7159)', 'deg');
    B = addSkip(B, 'phase crossover frequency w_180', '150', whyCase, ...
        'p.519 text (zarchan.txt:7164)', 'rad/s');
    B = addSkip(B, 'gain at w_180', '-11.5', whyCase, ...
        'p.519 text (zarchan.txt:7159)', 'dB');
end

% ---- 5.  The gain that destabilises the loop -------------------------------
if bookCase
    B = addVal(B, 'K_R that destabilises the loop', 0.376, R.k_unstable, ...
        'read-off', 0.02, 'p.520 text (zarchan.txt:7166-7173)', '-', ...
        ['NOT independent of the w_180 row: it is KR*10^(GM/20), the same ' ...
         'measurement pushed through the book''s own dB arithmetic.  Scored ' ...
         'anyway because the book prints it as its own number and the ' ...
         'dB->ratio step is one this port actually performs']);
else
    B = addSkip(B, 'K_R that destabilises the loop', '0.376', whyCase, ...
        'p.520 text (zarchan.txt:7166-7173)', '-');
end

% ---- 6.  The approximate crossover formula ---------------------------------
% Printed WITH its factors (0.1 x 1.89 x 0.457 x 25.3^2 = 55.2), so this is
% arithmetic, not a reading.  It scores the seven constants, not the sweep.
if bookCase
    B = addVal(B, 'approximate w_CR = KR |K3| Ta wAF^2', 55.2, R.wcr_approx, ...
        'text', 0.005, 'p.525 text (zarchan.txt:7224)', 'rad/s', ...
        ['no sweep involved: this row scores the CONSTANTS.  The book ' ...
         'truncates 55.29 to 55.2.  How well it approximates the actual ' ...
         'crossover is the INFO row below, not this one']);
else
    B = addSkip(B, 'approximate w_CR = KR |K3| Ta wAF^2', '55.2', whyCase, ...
        'p.525 text (zarchan.txt:7224)', 'rad/s');
end

% ---- 7.  Fig. 22.18: doubling the autopilot gain ---------------------------
if plantCase && abs(opts.KR2 - 0.2) < 1e-12
    B = addVal(B, 'gain crossover at KR = 0.2 (Fig. 22.18)', 103.0, R.wgc2, ...
        'read-off', 0.05, 'p.526 text (zarchan.txt:7241)', 'rad/s', ...
        ['5 per cent is a READ-OFF BAND, not a precision claim: the book is ' ...
         'quoting a curve off Fig. 22.18, and it quotes the KR = 0.1 value ' ...
         'as 60 in the same sentence where p.519 says 64 -- see the INFO row']);
else
    B = addSkip(B, 'gain crossover at KR = 0.2 (Fig. 22.18)', '103', ...
        sprintf(['needs the book''s airframe/actuator and KR2 = 0.2; this ' ...
                 'run used KR2 = %g'], opts.KR2), ...
        'p.526 text (zarchan.txt:7241)', 'rad/s');
end

% ---- 8.  THE SHAPE CLAIM ---------------------------------------------------
% zarchan.txt:7159, about Fig. 22.16: the gain "peaks due to the low airframe
% damping (zAF = 0.058) and then is quickly attenuated due to the dynamics of
% the actuator".  Prose about a figure IS greppable, and this ordering is
% independent of KR (raising the gain lifts the whole curve without moving the
% peak), so it survives when the value rows skip.
if plantCase
    [pk, ip] = max(R.gain_db);
    interior = ip > 1 && ip < numel(R.gain_db);
    nearAF   = interior && abs(R.w(ip)/opts.WAF - 1) <= 0.20;
    attenuated = interior && (R.gain_db(end) <= pk - 60);
    B = addShape(B, 'gain peaks at the airframe resonance, then falls hard', ...
        'interior peak within 20% of wAF, and >= 60 dB down by 1000 rad/s', ...
        sprintf('peak %+.2f dB at %.2f rad/s (wAF = %.1f); %+.2f dB at %.0f rad/s (%.1f dB down)', ...
                pk, R.w(ip), opts.WAF, R.gain_db(end), R.w(end), pk - R.gain_db(end)), ...
        nearAF && attenuated, 'p.519 text (zarchan.txt:7159)', ...
        ['the book publishes NO number for the peak, so it is an ordering ' ...
         'and not a value.  Drop either actuator pole and the "falls hard" ' ...
         'half of this row is what notices']);
else
    B = addSkip(B, 'gain peaks at the airframe resonance, then falls hard', ...
        'interior peak near wAF', whyCase, 'p.519 text (zarchan.txt:7159)', 'dB');
end

% ---- INFO.  Reported, never scored. ----------------------------------------
if bookCase
    errNominal = 100*(R.wcr_approx/60 - 1);
    errActual  = 100*(R.wcr_approx/R.wgc - 1);
    B = addInfo(B, 'ERRATUM: the book gives w_gc as BOTH 64 and 60 rad/s', ...
        '64 (p.519) vs 60 (p.525, p.526)', ...
        sprintf('this listing computes %.2f rad/s', R.wgc), ...
        'zarchan.txt:7164 vs :7225 and :7241', ...
        sprintf(['NOT SCORED: the two published values are inconsistent and ' ...
            'p.519 is the one this listing reproduces.  The consequence is ' ...
            'downstream -- the claim at :7229 that the approximate formula ' ...
            'is "in error by slightly less than 10%%" holds against the 60 ' ...
            '(%.1f%%) and FAILS against the crossover this listing actually ' ...
            'produces (%.1f%%).  That bound is therefore not scored either, ' ...
            'and the band was NOT widened to absorb it.  At KR = 0.2 the ' ...
            'approximation is genuinely better, as the book says: %.1f%% ' ...
            'against our %.1f rad/s'], errNominal, errActual, ...
            100*(2*R.wcr_approx/R.wgc2 - 1), R.wgc2));
end
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw.  A struct array that quietly reordered fields
%   would make [B.ok] mean something else.  Schema is the one benchSuite's
%   shared benchRows() consumes, and adaptGeneric is deliberately strict
%   about it.
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
L{end+1} = 'Zarchan Listing 22.3 -- OPEN-LOOP BODE RESPONSE, RATE GYRO FLIGHT-CONTROL SYSTEM';
L{end+1} = sprintf(['  K3 = %g   Ta = %g s   zAF = %g   wAF = %g rad/s   ' ...
                    'zACT = %g   wACT = %g rad/s   KR = %g'], ...
                    p.K3, p.TA, p.ZAF, p.WAF, p.ZACT, p.WACT, p.KR);
L{end+1} = sprintf('  %d points, %.4f .. %.0f rad/s (40/decade); deg via %.4g', ...
                    p.n_points, R.w(1), R.w(end), p.DEG);
L{end+1} = '';
L{end+1} = sprintf('  gain crossover   w_gc = %8.3f rad/s   phase there %8.2f deg   PHASE MARGIN %6.2f deg', ...
                    R.wgc, R.phase_at_wgc, R.pm_deg);
L{end+1} = sprintf('  phase crossover  w_180= %8.3f rad/s   gain  there %8.2f dB    GAIN  MARGIN %6.2f dB', ...
                    R.wpc, R.gain_at_wpc, R.gm_db);
L{end+1} = sprintf('  KR that destabilises the loop: %.4f      approximate w_CR: %.2f rad/s', ...
                    R.k_unstable, R.wcr_approx);
L{end+1} = sprintf('  gain crossover at KR = %g: %.2f rad/s', p.KR2, R.wgc2);
L{end+1} = sprintf('  (%d zero-dB crossings found, %d phase crossings; the LAST and FIRST are used)', ...
                    R.n_gain_crossings, R.n_phase_crossings);
L{end+1} = '';
L{end+1} = sprintf('  units: %s', R.units);

if ~isempty(R.bench)
    L{end+1} = '';
    L{end+1} = '  BENCHMARKS (book value / ours / band):';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
        case 'skip'
            L{end+1} = sprintf('    SKIP  %-46s book %-10s  %s', ...
                b.what, b.book_str, b.why); %#ok<AGROW>
        case 'info'
            L{end+1} = sprintf('    INFO  %-46s %s  |  %s', ...
                b.what, b.ours_str, b.book_str); %#ok<AGROW>
        case 'shape'
            L{end+1} = sprintf('    %-5s %-46s %s', ...
                tickStr(b.ok), b.what, b.ours_str); %#ok<AGROW>
        otherwise
            if b.skipped
                L{end+1} = sprintf('    SKIP  %-46s book %-10s  %s', ...
                    b.what, b.book_str, b.why); %#ok<AGROW>
            else
                L{end+1} = sprintf('    %-5s %-46s book %-9s ours %-9s %+7.2f%%  (band %s %.3g%%)', ...
                    tickStr(b.ok), b.what, b.book_str, b.ours_str, b.pct, ...
                    b.band, 100*b.tol); %#ok<AGROW>
            end
        end
    end
end

L{end+1} = '';
L{end+1} = '  STRUCTURAL (self-consistency, NOT book values):';
for i = 1:numel(R.selfcheck.names)
    L{end+1} = sprintf('    %-5s %-46s %s', tickStr(R.selfcheck.oks(i)), ...
        R.selfcheck.names{i}, R.selfcheck.detail{i}); %#ok<AGROW>
end

L{end+1} = '';
if any(ev)
    L{end+1} = sprintf('  %d of %d evaluated book values inside tolerance.', ...
        sum([R.bench(ev).ok]), sum(ev));
else
    L{end+1} = '  NO BOOK VALUE ADJUDICATED.';
end
s = strjoin(L, newline);
end

function s = tickStr(ok)
if ok, s = ' ok  '; else, s = 'FAIL '; end
end


% =============================================================================
function drawCurves(R)
%DRAWCURVES  The listing's own plotting tail, split into its two figures and
%   given the listing's own axis limits.
figure;
semilogx(R.w, R.gain_db); grid on
xlabel('Frequency (Rad/Sec)'); ylabel('Gain (Db)');
axis([.1 1000 -60 40]);
title('Fig. 22.16 -- open-loop gain, rate gyro FCS');

figure;
semilogx(R.w, R.phase_deg); grid on
xlabel('Frequency (Rad/Sec)'); ylabel('Phase (Deg)');
axis([.1 1000 -400 100]);
title('Fig. 22.16 -- open-loop phase, rate gyro FCS');
end
