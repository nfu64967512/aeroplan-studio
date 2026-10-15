function R = ballisticTargetSim(opts)
%MGS.ZARCHAN.BALLISTICTARGETSIM  Zarchan Listing 16.1 -- ballistic target
%   simulation: a re-entering threat under drag and flat-Earth gravity only.
%
%   R = mgs.zarchan.ballisticTargetSim()
%   R = mgs.zarchan.ballisticTargetSim('VT', 8000, 'BETA', 1000)
%   R = mgs.zarchan.ballisticTargetSim('assert', false, 'render', false)
%
%   Line-by-line port of Listing 16.1 (zarchan.txt:4695-4703) on the book's
%   own second-order Runge-Kutta (Heun) scheme, plus the Chapter 16 closed
%   forms (zarchan.txt:4792, :4803-4805, :4826) it is compared against.
%
%   UNITS ARE THE BOOK'S: ft, ft/s, ft/s^2, g = 32.2, rho slug/ft^3, beta
%   lb/ft^2.  NOT SI.  Nothing is converted, so every number printed can be
%   laid straight against a published one; convert AT THE CALL SITE and say so
%   there.  Angles use the listing's own 57.3 deg/rad, not 180/pi (45/57.3 rad
%   is 45.0028 deg) -- kept, because the closed-form rows below are scored
%   against numbers the book computed with that same constant.
%
%   *** WHAT THIS IS NOT FOR. ***  Chapter 16 is the BALLISTIC half of the
%   book: an unpowered, unguided point mass falling out of the exo-atmosphere
%   at Mach ~6, whose only "acceleration" is drag it cannot steer.  THIS
%   PROJECT DOES NOT FLY THAT, and no number out of this file belongs in a
%   spread, terminal-speed or miss-distance argument.  It is here for two
%   narrow reasons: it is the cleanest instance of the exponential atmosphere
%   + ballistic-coefficient drag model that Listing 16.2, Chapter 17 and the
%   Chapter 10 aerodynamics all reuse, and it is one of the few places where
%   the book prints an ANALYTIC answer beside the simulation, so the port can
%   be scored against arithmetic instead of against a picture.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_16_01.m reports NO unresolved
%   variable-to-variable comparisons and that is correct -- every comparison
%   in 16.1 is against a literal.  Three OCR repairs, all forced by the
%   decoded text at zarchan.txt:4695-4702, none a judgement call:
%     1  "while RT2 . = 0."  -> "while RT2 >= 0."  ("." is this book's OCR of
%        ">").  Direction confirmed by the prose at zarchan.txt:4693: the run
%        "stops when the ballistic threat hits the ground (RT2 < 0)".
%     2  "while STEP , = 1" -> "while STEP <= 1"  ("," is the OCR of "<"); the
%        body sets STEP = 2 to leave, which only makes sense against "<=".
%     3  "if RT2 , = 30000." -> "if RT2 <= 30000."  Same substitution, and the
%        branch it guards is the BELOW-30-kft density law, which the prose at
%        zarchan.txt:4690 names as 0.002378*exp(-h/30,000).  Reversed, rho at
%        100 kft comes out 2.4x too large and the trajectory still looks fine.
%   "S >= .09999" records every tenth of a second at the book's H = 0.01
%   (zarchan.txt:4693); this port tests against samp - H/2 instead, so a
%   caller asking for samp = 0.5 gets 0.5 rather than a threshold frozen at
%   the book's own H.  Nothing is written to disk ("save datfil.txt output
%   /ascii" is the book's typo for -ascii); 'plot' draws its three figures.
%
%   THE MINUS-SIGN DAMAGE CLASS appears twice, both resolved by arithmetic:
%   zarchan.txt:4792 prints "0.5 = 0.606VTIC", having lost the "e^-" of
%   e^(-0.5) -- exp(-0.5) = 0.60653 confirms it to three figures.  And
%   zarchan.txt:4826 prints "= 2.6  107 V2TIC sin gT", which is 2.6e-7 with
%   the exponent minus dropped: 2.6e-7*6000^2*sin45 = 6.6 g is the right order
%   for Fig. 16.3, where 2.6e+7 would be 9e14 g.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Every book value was grepped out of zarchan_decoded.txt and
%   its line is in the row's .source; nothing was read off a figure, since the
%   extraction turned every figure into an image link.  The Fig. 16.3 numbers
%   are PROSE claims about that figure (zarchan.txt:4710), which is a
%   different thing and is greppable.  Each row's .detail carries its reason;
%   in summary:
%     1  |a| at 100 kft = 1 g, band 10%.  The WIDTH is the finding: the prose
%        credits it to gravity alone, but rho(100 kft) = 3.61e-5 gives 1.30 g
%        of drag here.  What leaves |a| near 1 g is CANCELLATION against
%        gravity at gamma = 45 deg, so a tight band would be scoring the
%        book's explanation rather than its number.
%     2  peak deceleration "nearly 8 g" -- a BOUND row, 7..8 g, because the
%        number wears an adjective.
%     3  altitude of the peak, "approximately 40 kft", band 10%.
%     4  speed there, "3800 = 0.63 * 6000" ft/s, band 3%.  ONE row: the book
%        writes speed and ratio as a single statement.
%     5  closed form VT|maxQ/VTIC = 0.606 (:4792), band 0.5%.  The next line
%        calls the same quantity "61%" (:4793); scored against the
%        three-figure form, both cited.
%     6  closed-form constant 2409 = 0.0034*22,000*g (:4805), band 0.1%.
%     7  closed-form coefficient 2.6e-7 (:4826), band 1%.
%     8  SHAPE (:4709): the trajectory "is approximately a straight line" --
%        max deviation from the chord <= 5% of the chord.
%     9  SHAPE (:4731, caption :4733): peak |a| "approximately independent
%        of ballistic coefficient" AND the peak altitude falling as beta
%        rises.  Both halves, one row.
%    10  SHAPE (:4693): H = 0.01 s is "sufficiently small" -- step-halving.
%   NOT SCORED: the book calls its closed forms "in excellent agreement" with
%   the simulation (:4807, :4828).  That is an ADJECTIVE, and the two differ
%   by 8% in altitude and 17% in magnitude because the derivation drops
%   gravity and the sub-30-kft atmosphere -- which those same lines say.  Both
%   gaps are INFO rows, with their signs.  Inventing a threshold for
%   "excellent" would be scoring our own opinion and calling it Zarchan.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED. ***  MATLAB is the
%   caller's; this file has never been executed here.  The figures below come
%   from an independent re-implementation of the same arithmetic, so a first
%   run has something to be compared against:
%     nominal (100 kft, 6000 ft/s, 45 deg, beta 500, H 0.01, samp 0.1):
%       429 samples, impact at T = 42.9 s, downrange 85.77 kft
%       |a| first sample 0.938 g (book 1, -6.2%);  peak 7.756 g (in 7..8)
%       peak altitude 38.75 kft (book ~40, -3.1%); speed 3810.5 ft/s (+0.3%)
%       ratio 0.6351;  straightness 3.0% of chord (limit 5%)
%       beta 250/500/1000: peaks 8.00/7.76/7.49 g (spread 6.5%), altitudes
%       54.4/38.8/29.7 kft;  step halving ~7e-7 relative (limit 1e-3)
%     closed forms: 42.21 kft, 6.61 g, ratio 0.60653
%   A residual near a band EDGE is not a pass to be accepted -- the numbers
%   above say where each should land.  Do not widen a band to make a row green.
%
% ---------------------------------------------------------------------------
%   OUTPUTS
%     R.T R.RT1K R.RT2K R.ATG R.VT  the listing's five recorded arrays
%                                   (s, kft, kft, g, ft/s)
%     R.VT1 R.VT2 R.AT1 R.AT2  components there; the structural rows need them
%     R.peak    .atg_g .alt_kft .speed_fps .ratio .T_s .index
%     R.cf      .alt_maxQ_kft .amax_g .vratio .const .coef .rho_maxQ
%     R.sweep   the Fig. 16.5 experiment: .beta .peak_g .alt_kft
%     R.conv    the step-halving test: .H_half .peak_g .rel
%     R.bench / R.selfcheck / R.units / R.p / R.summary as in the sibling
%               ports; .selfcheck carries .names and .oks, which is what
%               benchSuite turns into structural rows
%     R.pass    true only if at least one row was SCORED and all passed
%     R.verdict '' when everything passed, else the one-line reason.  ALWAYS
%               FORMED; 'assert' picks the reaction and nothing else -- false
%               WARNS and still returns R complete, because benchSuite runs
%               every port with assert off so one red row cannot hide a board.
%
%   See also MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 16.1 line 1, verbatim.  Feet, ft/s, degrees, lb/ft^2, seconds.
    opts.RT2     (1,1) double {mustBeFinite, mustBePositive} = 100000  % ft, initial altitude
    opts.VT      (1,1) double {mustBeFinite, mustBePositive} = 6000    % ft/s, initial speed
    opts.GAMTDEG (1,1) double {mustBeFinite, mustBePositive} = 45      % deg, re-entry angle
    opts.BETA    (1,1) double {mustBeFinite, mustBePositive} = 500     % lb/ft^2
    opts.H       (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s, RK2 step
    opts.samp    (1,1) double {mustBeFinite, mustBePositive} = 0.1     % s, record interval
    % Fig. 16.5's experiment.  Three points is the minimum at which "spread"
    % and "decreasing" are statements about a trend rather than about the
    % sample.  beta = 2000 is left out of the default because its peak falls
    % to 9 kft -- below the 30-kft atmosphere switch and close enough to the
    % ground that the run ends near the peak.  That is a different regime,
    % not a harder test of this one.
    opts.beta_sweep (1,:) double {mustBeFinite, mustBePositive} = [250 500 1000]
    % Guard only.  A ballistic target that has not hit the ground in this long
    % is a parameter mistake, and an unbounded "while RT2 >= 0" would hang the
    % whole suite rather than fail it.
    opts.tmax    (1,1) double {mustBeFinite, mustBePositive} = 600     % s
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options, MATLAB reports nargin = 0 no matter
    % how many were passed, so an "if nargin == 0" here would never fire.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

G = 32.2;                       % the listing's own gravity, hard-coded there

if opts.samp < opts.H
    error('mgs:zarchan:ballisticTargetSim:samp', ...
        ['samp = %g s is smaller than the integration step H = %g s, so the ' ...
         'returned curve would be sampled at the step rate while claiming ' ...
         'the finer interval in its own time vector.'], opts.samp, opts.H);
end

% =========================================================================
%   LISTING 16.1.  Transcribed from zarchan.txt:4695-4703.
% =========================================================================
S0 = flyTarget(opts.RT2, opts.VT, opts.GAMTDEG, opts.BETA, opts.H, ...
               opts.samp, opts.tmax, G);
if numel(S0.T) < 3
    % Refused, not warned about: below three samples "peak", "both ends" and
    % "monotone" are statements about the sampling, not about the flight.
    error('mgs:zarchan:ballisticTargetSim:empty', ...
        ['the run recorded %d sample(s) at samp = %g s.  At least three are ' ...
         'needed before any of the shape rows mean anything.'], ...
        numel(S0.T), opts.samp);
end

[pkATG, ip] = max(S0.ATG);
peak = struct('atg_g', pkATG, 'alt_kft', S0.RT2K(ip), ...
              'speed_fps', S0.VT(ip), 'ratio', S0.VT(ip)/opts.VT, ...
              'T_s', S0.T(ip), 'index', ip);

% ---- the Chapter 16 closed forms.  See the header for the two exponents
%      that had to be recovered by arithmetic. ------------------------------
sg = sin(opts.GAMTDEG/57.3);
cf = struct();
cf.const        = 0.0034*22000*G;                  % the book prints 2409
cf.coef         = 0.5*exp(-1)/(22000*G);           % the book prints 2.6e-7
cf.vratio       = exp(-0.5);                       % the book prints 0.606
cf.alt_maxQ_kft = 22000*log(cf.const/(opts.BETA*sg))/1000;
cf.amax_g       = cf.coef*opts.VT^2*sg;
cf.rho_maxQ     = opts.BETA*sg/(22000*G);

% ---- the Fig. 16.5 experiment, and the step-halving test ------------------
nb = numel(opts.beta_sweep);
sweep = struct('beta', opts.beta_sweep, 'peak_g', zeros(1,nb), ...
               'alt_kft', zeros(1,nb));
for k = 1:nb
    Sk = flyTarget(opts.RT2, opts.VT, opts.GAMTDEG, opts.beta_sweep(k), ...
                   opts.H, opts.samp, opts.tmax, G);
    [pk, ik] = max(Sk.ATG);
    sweep.peak_g(k)  = pk;
    sweep.alt_kft(k) = Sk.RT2K(ik);
end

Sh = flyTarget(opts.RT2, opts.VT, opts.GAMTDEG, opts.BETA, opts.H/2, ...
               opts.samp, opts.tmax, G);
halfPeak = max(Sh.ATG);
convRel  = abs(halfPeak - pkATG)/max(pkATG, realmin);

R = struct();
R.T   = S0.T;    R.RT1K = S0.RT1K;  R.RT2K = S0.RT2K;
R.ATG = S0.ATG;  R.VT   = S0.VT;
R.VT1 = S0.VT1;  R.VT2  = S0.VT2;   R.AT1 = S0.AT1;  R.AT2 = S0.AT2;
R.peak  = peak;
R.cf    = cf;
R.sweep = sweep;
R.conv  = struct('H_half', opts.H/2, 'peak_g', halfPeak, 'rel', convRel);
R.units = ['ft, ft/s, ft/s^2 (g = 32.2 ft/s^2); rho slug/ft^3; ' ...
           'beta lb/ft^2; angles deg via the listing''s 57.3 deg/rad'];
R.p = struct('RT2',opts.RT2, 'VT',opts.VT, 'GAMTDEG',opts.GAMTDEG, ...
             'BETA',opts.BETA, 'H',opts.H, 'samp',opts.samp, ...
             'beta_sweep',opts.beta_sweep, 'tmax',opts.tmax, ...
             'n_samples',numel(S0.T), 'impact_T_s',S0.T(end), ...
             'impact_downrange_kft',S0.RT1K(end), ...
             'impact_speed_fps',S0.VT(end), 'units',R.units);

R.selfcheck = structuralChecks(S0, G);

if opts.benchmark
    R.bench = benchmarks(S0, peak, cf, sweep, convRel, opts);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine" -- that is the shape of green light this repo has been burned
    % by more than once.
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R); end

% ---- adjudication.  The verdict is FORMED either way; only the reaction to
%      it depends on 'assert'.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = 'not one book value could be evaluated.';
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
        error('mgs:zarchan:ballisticTargetSim:benchmark', ...
            '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN.  R comes back complete so the caller can see WHICH
        % side of a comparison moved.  Never silence.
        warning('mgs:zarchan:ballisticTargetSim:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function S = flyTarget(RT2, VT, GAMTDEG, BETA, H, samp, tmax, G)
%FLYTARGET  Listing 16.1, zarchan.txt:4695-4703.
%
%   Every constant arrives as an ARGUMENT: MATLAB local functions cannot see
%   the main function's workspace, and a port in this package that forgot that
%   threw on the first line of its first run with no static warning.

RT1  =  0.0;
VT1  =  VT*cos(GAMTDEG/57.3);     % the listing's 57.3, not 180/pi
VT2  = -VT*sin(GAMTDEG/57.3);     % negative: the target is descending
T    =  0.0;
Sacc =  0.0;

% Derivatives initialised before the loop so that the FLAG==1 branch inside it
% cannot read a value left over from anything else.
AT1 = 0.0;  AT2 = 0.0;

nmax = ceil(tmax/samp) + 8;
Tv = zeros(nmax,1);  R1 = zeros(nmax,1);  R2 = zeros(nmax,1);
Ag = zeros(nmax,1);  Vv = zeros(nmax,1);
V1 = zeros(nmax,1);  V2 = zeros(nmax,1);
A1 = zeros(nmax,1);  A2 = zeros(nmax,1);
n = 0;

stepMax = ceil(tmax/H);
nstep   = 0;

while RT2 >= 0.0
    nstep = nstep + 1;
    if nstep > stepMax
        error('mgs:zarchan:ballisticTargetSim:runaway', ...
            ['not on the ground after tmax = %g s (altitude %g ft, speed ' ...
             '%g ft/s).  The listing''s loop is "while RT2 >= 0" with no ' ...
             'other exit, so this is refused rather than left to hang.'], ...
            tmax, RT2, hypot(VT1, VT2));
    end

    RT1OLD = RT1;  RT2OLD = RT2;  VT1OLD = VT1;  VT2OLD = VT2;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop: pass 1 at the current state, pass 2
    % after a full Euler step with T advanced by H.  NOTE the listing's ORDER
    % inside the FLAG branch -- POSITION advances on the OLD velocity, before
    % velocity advances on the old acceleration.  Reordering those four lines
    % is a silent change of scheme that still runs perfectly smoothly.
    while STEP <= 1
        if FLAG == 1
            RT1 = RT1 + H*VT1;
            RT2 = RT2 + H*VT2;
            VT1 = VT1 + H*AT1;
            VT2 = VT2 + H*AT2;
            T   = T + H;
            STEP = 2;
        end
        RHO = rhoOf(RT2);
        VT  = sqrt(VT1^2 + VT2^2);
        Q   = 0.5*RHO*VT^2;
        % atan2(-VT2, VT1): GAMT is the DESCENT angle, positive going down,
        % which is why the drag term enters AT2 with a PLUS sign below --
        % drag opposes the downward velocity, so it points up.
        GAMT = atan2(-VT2, VT1);
        AT1  = -G*Q*cos(GAMT)/BETA;
        AT2  = -G + G*Q*sin(GAMT)/BETA;
        FLAG = 1;
    end
    % The listing writes "FLAG=0" here as well.  Dropped: the outer loop
    % re-initialises FLAG on its next pass, so the assignment cannot be read.
    % Kept as a comment so nobody re-adds it thinking a state machine was lost.

    RT1 = 0.5*(RT1OLD + RT1 + H*VT1);
    RT2 = 0.5*(RT2OLD + RT2 + H*VT2);
    VT1 = 0.5*(VT1OLD + VT1 + H*AT1);
    VT2 = 0.5*(VT2OLD + VT2 + H*AT2);

    % ONE increment, tested against samp - H/2.  The listing increments once
    % and tests "S >= .09999", which at its own H = 0.01 records every tenth
    % of a second (zarchan.txt:4693).  Keeping the literal would silently
    % change meaning at any other H.
    Sacc = Sacc + H;
    if Sacc >= samp - 0.5*H
        Sacc = 0.0;
        n = n + 1;
        Tv(n) = T;
        R1(n) = RT1/1000;
        R2(n) = RT2/1000;
        Ag(n) = sqrt(AT1^2 + AT2^2)/G;
        Vv(n) = sqrt(VT1^2 + VT2^2);
        V1(n) = VT1;  V2(n) = VT2;
        A1(n) = AT1;  A2(n) = AT2;
    end
end

S = struct('T',Tv(1:n), 'RT1K',R1(1:n), 'RT2K',R2(1:n), 'ATG',Ag(1:n), ...
           'VT',Vv(1:n), 'VT1',V1(1:n), 'VT2',V2(1:n), ...
           'AT1',A1(1:n), 'AT2',A2(1:n));
end


% =============================================================================
function rho = rhoOf(alt_ft)
%RHOOF  The book's two-branch exponential atmosphere, zarchan.txt:4688-4690.
%   Below 30,000 ft: 0.002378*exp(-h/30,000).  Above: 0.0034*exp(-h/22,000).
%   The branch test is the OCR repair described in the header; reversed, rho
%   at 100 kft comes out 2.4x too large and the trajectory stays plausible.
if alt_ft <= 30000
    rho = 0.002378*exp(-alt_ft/30000);
else
    rho = 0.0034*exp(-alt_ft/22000);
end
end


% =============================================================================
function K = structuralChecks(S, G)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never evidence that
%   this port reproduces Zarchan -- only the weaker claim that it agrees with
%   itself, which is what catches wiring faults.
%
%   .names and .oks are what benchSuite turns into board rows.  A check stored
%   under any other field name is a check nobody can see, which has happened
%   in this package before and made a board look green.
names = {};  oks = [];  detail = {};
K = struct();

% 1  Geometry: the target falls and travels downrange, at every sample.
dAlt  = diff(S.RT2K);
dDown = diff(S.RT1K);
K.max_dAlt  = max(dAlt);
K.min_dDown = min(dDown);
names{end+1}  = 'altitude falls, downrange grows';
oks(end+1)    = K.max_dAlt < 0 && K.min_dDown > 0;
detail{end+1} = sprintf('max altitude step %+.4f kft, min downrange step %+.4f kft', ...
                        K.max_dAlt, K.min_dDown);

% 2  Drag is ANTI-PARALLEL to velocity.  Gravity removed, what is left must
%    point straight back down the velocity vector; a sign flip on either AT1
%    or AT2 turns this cosine positive while the trajectory still looks like a
%    re-entry.  Not exactly -1: the recorded AT's are the SECOND-pass
%    derivatives against the CORRECTED state, an O(H) mismatch.
d1 = S.AT1;
d2 = S.AT2 + G;
cosang = (d1.*S.VT1 + d2.*S.VT2) ./ ...
         max(hypot(d1,d2).*hypot(S.VT1,S.VT2), realmin);
K.worst_cos = max(cosang);
names{end+1}  = 'drag anti-parallel to velocity';
oks(end+1)    = K.worst_cos <= -0.99;
detail{end+1} = sprintf('worst cos(drag, V) = %+.6f (exactly -1 in the model)', ...
                        K.worst_cos);

% 3  Atmosphere: monotone in altitude, and the two branches meet at 30 kft.
%    The 0.61% step at the seam is the book's own model, not a port artefact;
%    it is checked so that a SWAPPED branch -- which reverses the sign of that
%    step and multiplies rho at 100 kft by 2.4 -- cannot pass unnoticed.
h  = 0:250:120000;
rr = arrayfun(@rhoOf, h);
K.rho_max_step = max(diff(rr));
K.rho_seam_pct = 100*abs(0.0034*exp(-30000/22000) - 0.002378*exp(-1)) / ...
                 (0.002378*exp(-1));
names{end+1}  = 'rho falls with altitude, branches meet at 30 kft';
oks(end+1)    = K.rho_max_step < 0 && K.rho_seam_pct < 1.0;
detail{end+1} = sprintf('max rho step %+.3e slug/ft^3, seam mismatch %.3f%%', ...
                        K.rho_max_step, K.rho_seam_pct);

% 4  Speed falls throughout.  At these conditions drag exceeds the along-track
%    component of gravity from the very first step; if the speed ever rose,
%    the acceleration signs or the atmosphere would be the reason.
K.max_dV = max(diff(S.VT));
names{end+1}  = 'speed strictly decreasing';
oks(end+1)    = K.max_dV < 0;
detail{end+1} = sprintf('max speed step %+.4f ft/s', K.max_dV);

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
function B = benchmarks(S, peak, cf, sweep, convRel, opts)
B = emptyBench();

% ---- 1.  |a| at 100 kft.  See the header for why the band is 10%. ---------
rho0    = rhoOf(opts.RT2);
Q0      = 0.5*rho0*opts.VT^2;
drag0_g = Q0/opts.BETA;            % = g*Q/beta expressed in g
B = addVal(B, '|a| at 100-kft altitude', 1.0, S.ATG(1), '10%', 0.10, ...
    'p.352 text (zarchan.txt:4710)', 'g', ...
    sprintf(['first recorded sample, altitude %.2f kft.  The prose credits ' ...
        'this to gravity alone, but drag here is %.2f g (rho = %.3e, ' ...
        'Q = %.0f lb/ft^2): what leaves it near 1 g is CANCELLATION against ' ...
        'gravity at gamma = %g deg, not an absent atmosphere'], ...
        S.RT2K(1), drag0_g, rho0, Q0, opts.GAMTDEG));

% ---- 2.  Peak deceleration: "nearly 8 g" is a number wearing an adjective.
B = addBound(B, 'peak deceleration -- "nearly 8 g"', '7 <= |a|max <= 8', ...
    sprintf('%.4f g at %.3f kft, T = %.2f s', peak.atg_g, peak.alt_kft, peak.T_s), ...
    peak.atg_g >= 7.0 && peak.atg_g <= 8.0, ...
    'p.352 text (zarchan.txt:4710)', 'g', ...
    ['a BOUND and not a value row: "nearly 8" is qualified prose, and ' ...
     'turning it into a value would put a band of our own choosing on the ' ...
     'book''s hedge.  This includes gravity, exactly as the listing''s ATG does']);

% ---- 3/4.  Altitude and speed at the peak. --------------------------------
B = addVal(B, 'altitude of peak deceleration', 40.0, peak.alt_kft, '10%', 0.10, ...
    'p.352 text (zarchan.txt:4710)', 'kft', ...
    sprintf(['the book says "approximately 40 kft".  The Ch.16 closed form ' ...
        '(zarchan.txt:4805) puts it at %.2f kft -- higher, because it drops ' ...
        'gravity'], cf.alt_maxQ_kft));

B = addVal(B, 'target speed at peak deceleration', 3800.0, peak.speed_fps, ...
    '3%', 0.03, 'p.352 text (zarchan.txt:4710)', 'ft/s', ...
    sprintf(['the book writes it as "3800 = 0.63 * 6000"; ours is a ratio ' ...
        'of %.4f against an initial %.0f ft/s.  Scored once, not twice: the ' ...
        'speed and the ratio are one statement'], peak.ratio, opts.VT));

% ---- 5/6/7.  The closed-form constants.  These score the TRANSCRIPTION of
%      the equations, which is where the dropped exponent minus signs live.
B = addVal(B, 'closed form VT|maxQ / VTIC = exp(-0.5)', 0.606, cf.vratio, ...
    '0.5%', 0.005, 'p.357 (zarchan.txt:4792; same value as "61%" at :4793)', '-', ...
    ['the printed "0.5 = 0.606VTIC" lost the "e^-" of e^(-0.5).  Scored ' ...
     'against the three-figure 0.606 rather than against the next line''s ' ...
     'rounded "61%", with both lines cited -- the book states one quantity twice']);

B = addVal(B, 'closed-form constant 0.0034*22,000*g', 2409, cf.const, ...
    '0.1%', 0.001, 'p.357 (zarchan.txt:4805)', 'lb/ft^2', ...
    'the "2409" inside RT2|maxQ = 22,000 ln(2409 / (beta sin gamma))');

B = addVal(B, 'closed-form peak-deceleration coefficient', 2.6e-7, cf.coef, ...
    '1%', 0.01, 'p.359 (zarchan.txt:4826)', '1/(ft/s)^2', ...
    sprintf(['printed as "2.6  107" -- exponent minus dropped.  2.6e-7 gives ' ...
        '%.2f g here; 2.6e+7 would give 9e14.  Ours is 0.5*exp(-1)/(22,000*g); ' ...
        'the book''s own route through 0.606^2 (:4821) gives 2.5920e-7, and ' ...
        'both round to the printed 2.6e-7'], cf.amax_g));

% ---- 8.  SHAPE: the trajectory is approximately a straight line. ----------
x = S.RT1K;  y = S.RT2K;
chord = hypot(x(end)-x(1), y(end)-y(1));
dev = abs((x(end)-x(1))*(y(1)-y) - (x(1)-x)*(y(end)-y(1)))/max(chord, realmin);
maxdev = max(dev);
B = addShape(B, 'trajectory is approximately a straight line', ...
    'max deviation from chord <= 5% of chord', ...
    sprintf('%.4f kft on a %.2f kft chord = %.2f%%', maxdev, chord, ...
            100*maxdev/max(chord, realmin)), ...
    maxdev <= 0.05*chord, 'p.352 text and Fig. 16.2 (zarchan.txt:4709)', ...
    ['the book exploits this straightness later in the chapter, which is ' ...
     'what makes it worth a row.  5% is our operationalisation of ' ...
     '"approximately"; the curvature is real and lives at low altitude']);

% ---- 9.  SHAPE: the Fig. 16.5 ballistic-coefficient experiment. -----------
if numel(sweep.beta) >= 3
    spread  = (max(sweep.peak_g) - min(sweep.peak_g))/mean(sweep.peak_g);
    ordered = all(diff(sweep.beta(:).') > 0) && all(diff(sweep.alt_kft(:).') < 0);
    B = addShape(B, 'peak |a| ~independent of beta; peak ALTITUDE falls with beta', ...
        'spread of peak |a| <= 12% and altitude strictly decreasing', ...
        sprintf('peaks [%s] g (spread %.1f%%), altitudes [%s] kft at beta [%s]', ...
            strjoin(arrayfun(@(v) sprintf('%.2f',v), sweep.peak_g, 'UniformOutput', false), ' '), ...
            100*spread, ...
            strjoin(arrayfun(@(v) sprintf('%.1f',v), sweep.alt_kft, 'UniformOutput', false), ' '), ...
            strjoin(arrayfun(@(v) sprintf('%g',v), sweep.beta, 'UniformOutput', false), ' ')), ...
        spread <= 0.12 && ordered, ...
        'p.354-355 text and Fig. 16.5 (zarchan.txt:4731, caption :4733)', ...
        ['BOTH halves of the book''s sentence in one row.  The independence ' ...
         'half is the surprising one: more drag at low beta is very nearly ' ...
         'cancelled by the peak moving up into thinner air']);
else
    B = addSkip(B, 'peak |a| ~independent of beta; peak ALTITUDE falls with beta', ...
        'spread <= 12%, altitude decreasing', ...
        sprintf(['beta_sweep has %d entr(y/ies).  Below three, "spread" and ' ...
            '"decreasing" describe the sample rather than the trend'], ...
            numel(sweep.beta)), ...
        'p.354-355 text (zarchan.txt:4731, caption :4733)', 'g');
end

% ---- 10.  SHAPE: H = 0.01 s is "sufficiently small". ----------------------
B = addShape(B, 'H = 0.01 s is "sufficiently small" (RK2 converged)', ...
    'peak |a| moves < 0.1% when H is halved', ...
    sprintf('%.3e relative, H = %g -> %g s', convRel, opts.H, opts.H/2), ...
    convRel < 1e-3, 'p.351 text (zarchan.txt:4693)', ...
    ['the book asserts the step size is adequate and never shows it; this ' ...
     'is that assertion, measured.  RK2 on this problem is far inside the ' ...
     'tolerance -- the residual should come back around 1e-6 relative, so a ' ...
     'result anywhere near 1e-3 means something else moved']);

% ---- INFO rows.  Reported, never scored. ---------------------------------
B = addInfo(B, 'closed-form altitude of max deceleration vs simulation', ...
    sprintf('%.2f kft (theory)', cf.alt_maxQ_kft), ...
    sprintf('%.2f kft (simulation), %+.1f%%', peak.alt_kft, ...
            100*(peak.alt_kft/max(cf.alt_maxQ_kft, realmin) - 1)), ...
    'p.358 text (zarchan.txt:4807)', ...
    ['NOT SCORED: the book''s claim here is the adjective "in excellent ' ...
     'agreement".  The gap is the derivation''s own two omissions, named on ' ...
     'that very line -- gravity, and the sub-30-kft atmosphere']);

B = addInfo(B, 'closed-form peak deceleration vs simulation', ...
    sprintf('%.2f g (theory)', cf.amax_g), ...
    sprintf('%.2f g (simulation), %+.1f%%', peak.atg_g, ...
            100*(peak.atg_g/max(cf.amax_g, realmin) - 1)), ...
    'p.359 text (zarchan.txt:4828)', ...
    ['NOT SCORED: same adjective, same two omissions.  The simulation runs ' ...
     'HIGHER because gravity has been adding speed all the way down and ' ...
     'deceleration goes as the square of it']);

B = addInfo(B, 'what this predicts about our vehicle', '--', 'nothing', ...
    'project context, not Zarchan', ...
    ['NOT SCORED: an unpowered, unguided point mass at Mach 6 shares no ' ...
     'failure mode with a guided, lift-limited, speed-floored aircraft.  ' ...
     'The reusable part of this chapter is the atmosphere model, not the ' ...
     'trajectory']);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want.  This is the
%   schema benchSuite's adaptGeneric consumes, and it is strict on purpose.
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

function B = addBound(B, what, bookStr, oursStr, ok, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'bound';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'bound';  B(i).tol = NaN;
B(i).ok   = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = units;  B(i).detail = detail;
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
L{end+1} = 'Zarchan Listing 16.1 -- BALLISTIC TARGET SIMULATION';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['   altitude %g ft   VT = %g ft/s   gamma = %g deg   ' ...
    'beta = %g lb/ft^2   H = %g s   samp = %g s'], ...
    p.RT2, p.VT, p.GAMTDEG, p.BETA, p.H, p.samp);
L{end+1} = sprintf(['   %d samples; ground impact at T = %.2f s, downrange ' ...
    '%.2f kft, speed %.1f ft/s'], p.n_samples, p.impact_T_s, ...
    p.impact_downrange_kft, p.impact_speed_fps);
L{end+1} = '';
L{end+1} = 'TRAJECTORY  (the listing''s five printed arrays)';
L{end+1} = sprintf('%9s %12s %12s %10s %12s', ...
    'T [s]', 'downrange', 'altitude', '|a| [g]', 'VT [ft/s]');
stride = max(1, round(numel(R.T)/14));
rows = unique([1:stride:numel(R.T), numel(R.T)]);
for r = rows
    L{end+1} = sprintf('%9.2f %12.3f %12.3f %10.4f %12.1f', ...
        R.T(r), R.RT1K(r), R.RT2K(r), R.ATG(r), R.VT(r)); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = sprintf(['PEAK DECELERATION  %.4f g at %.3f kft, T = %.2f s, ' ...
    'V = %.1f ft/s (%.4f of initial)'], R.peak.atg_g, ...
    R.peak.alt_kft, R.peak.T_s, R.peak.speed_fps, R.peak.ratio);
L{end+1} = sprintf(['CLOSED FORM        %.4f g at %.3f kft, V/VTIC = %.5f ' ...
    '  (gravity and the sub-30-kft atmosphere dropped)'], ...
    R.cf.amax_g, R.cf.alt_maxQ_kft, R.cf.vratio);
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-48s %s', vstr(K.oks(i)), K.names{i}, ...
        K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning';
    L{end+1} = '            NOT CHECKED, which is not the same thing as fine.';
else
    L{end+1} = 'BENCHMARKS vs ZARCHAN CHAPTER 16';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        if b.skipped
            mark = 'INFO';
            if strcmp(b.kind, 'skip'), mark = 'SKIP'; end
            L{end+1} = sprintf('   %-4s %-52s book %-16s ours %s', ...
                mark, b.what, b.book_str, b.ours_str); %#ok<AGROW>
        elseif isnan(b.book)
            L{end+1} = sprintf('   %-4s %-52s %s | %s', ...
                vstr(b.ok), b.what, b.book_str, b.ours_str); %#ok<AGROW>
        else
            L{end+1} = sprintf('   %-4s %-52s book %-10s ours %-10s %+7.2f%% (band %s)', ...
                vstr(b.ok), b.what, b.book_str, b.ours_str, b.pct, b.band); %#ok<AGROW>
        end
    end
    L{end+1} = sprintf('   -> %d of %d scored rows pass; %d reported only.', ...
        sum([R.bench(ev).ok]), sum(ev), sum(~ev));
end
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
function drawCurves(R)
%DRAWCURVES  The listing's three figures.  OFF by default: benchSuite runs
%   dozens of ports and a figure per port is a window pile, not a result.
figure; plot(R.RT1K, R.RT2K); grid on
xlabel('Downrange (Kft)'); ylabel('Altitude (Kft)');
title('Listing 16.1 -- ballistic target trajectory');
figure; plot(R.RT2K, R.ATG); grid on
xlabel('Altitude (Kft)'); ylabel('Acceleration (G)');
figure; plot(R.RT2K, R.VT); grid on
xlabel('Altitude (Kft)'); ylabel('Velocity (Ft/Sec)');
end
