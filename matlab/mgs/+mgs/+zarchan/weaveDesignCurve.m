function W = weaveDesignCurve(opts)
%MGS.ZARCHAN.WEAVEDESIGNCURVE  Zarchan Listing 20.3 -- normalized peak miss vs
%a WEAVING TARGET for a fifth-order binomial guidance system, with saturation.
%
%   W = mgs.zarchan.weaveDesignCurve()            bare call = book self-check
%   W = mgs.zarchan.weaveDesignCurve('xnp',3, 'ratio',3, 'x',1.0)
%   W = mgs.zarchan.weaveDesignCurve('plot',true)
%
%   Brute-force sweep of the normalized weave frequency x = omega_T * T.  For
%   each x it flies many flight times TF, reads the terminal separation Y, and
%   keeps the largest POSITIVE value over the steady-state window.  The result
%   is divided by T^2 to give the book's normalized miss, whose units are
%   ft per (g . s^2) -- see UNITS below, that factor of g is a live trap.
%
% ===========================================================================
%   UNITS: FEET.  DO NOT CONVERT.
% ===========================================================================
%   VC = 4000 ft/s, nT = 32.2 ft/s^2 (= 1 g), miss in ft, g = 32.2 ft/s^2.
%   The rest of mgs is SI.  These are kept in the book's units DELIBERATELY so
%   the published figures compare digit-for-digit.  Converting to metres here
%   would leave every number looking perfectly reasonable and every comparison
%   against Figs. 20.9-20.12 silently wrong, which is the exact failure mode
%   this package exists to catch.  If you need metres, convert AT THE CALL
%   SITE, after you have checked the reproduction.
%
% ===========================================================================
%   WHAT THIS IS AND -- IMPORTANT -- WHAT IT IS NOT
% ===========================================================================
%   Chapter 20 is about a TARGET that weaves while the interceptor tries to
%   hit it.  We are building the opposite: OUR OWN vehicle weaves, on purpose,
%   to burn time and control arrival.  Those are different problems and this
%   file does NOT model ours.  Do not quote a number out of here as a
%   prediction about our weave.  It will not survive contact with the flight
%   data, and this project has already been burned once by exactly that move
%   (the ring_pace anecdote extrapolated into a claim, then refuted at
%   r = 0.942 with 44.41% of sorties saturated).
%
%   What DOES transfer, and is the reason to have this file:
%
%   (1) An INDEPENDENT CHECK on our integrator and PN kernel.  The book
%       publishes the answers, so a disagreement here is our bug, not a
%       modelling opinion.  Little else in mgs has that property.
%
%   (2) The LAG -> MISS machinery we are missing.  Our model scores 32/32 at
%       0.03 m where the real set scores 25/32 at 0.93 m, and the measured
%       cause is a guidance lag the model does not have (gamma-tracking gain
%       0.063 measured vs 1.385 modelled, ~8 s of lag, saturating on 26/32).
%       Ch. 20 is the cleanest place in the book where a fifth-order lag and
%       a hard acceleration limit are driven by a SINUSOID and the steady-
%       state answer is published.  That is the shape of our open question:
%       what a lag-limited, acceleration-limited system actually achieves
%       versus what it was commanded.
%
%   (3) The qualitative result that matters to anyone designing a weave:
%       against a sinusoid the loop is WORST near x = 1, and above x ~ 0.7
%       guiding is worse than not guiding at all.  Both fall out below.
%
% ===========================================================================
%   CORRECTION TO THE BRIEF THIS FILE WAS WRITTEN FROM
% ===========================================================================
%   The task that commissioned this port asked for the saturated benchmark as
%       "N' = 5, ratio nLIM/nT = 3, x = 1: normalized miss 60 -> 70, Fig 20.12"
%   That attribution is WRONG, and it is wrong in a way that would have
%   produced a permanently red self-check that looked like OUR bug.  The
%   book's worked example for 60 -> 70 is at N' = 3, read off Fig. 20.10.
%   Verbatim from zarchan.txt, just after Fig. 20.11:
%
%       "...a 6-g target weave maneuver with weave frequency of 2 rad/s and a
%        proportional navigation missile guidance system with overall time
%        constant of 0.5 s and effective navigation ratio of 3 (nT = 6,
%        wT = 2, T = 0.5, N' = 3)... Peak Miss_g = 60 nT T^2 = 90 ft...
%        Ratio = nLIM/nT = 18/6 = 3... Peak Miss_18g = 70 nT T^2 = 105 ft"
%
%   N' = 5 at x = 1 is nowhere near 60/70: this file computes 87.5 unlimited
%   and 114.0 at ratio 3, and the book publishes no worked number for it.  The
%   benchmark block below reproduces the N' = 3 ladder (which it matches to
%   better than 3%) AND prints the N' = 5 point the brief cited, labelled as
%   NOT a book benchmark.  Both are shown rather than one quietly substituted:
%   "the numbers moved and nobody said why" is how this repo loses days.
%
%   Two further errata found while checking, both harmless but worth knowing
%   if you read the source text (same category as the two errata already noted
%   in mgs.zarchan.sizing):
%     - "Peak Miss_6g = 85 nT T^2 = 75 x 6 x 0.5^2 = 128 ft" -- the 75 is a
%       typo for 85; 85*6*0.25 = 127.5, the 75 does not give 128.
%     - "Peak Miss|N'=3 = 63 nT T^2 = 63 x 6 x 0.1 ~ 4 ft" -- the 0.1 should
%       be 0.1^2; 63*6*0.01 = 3.78.
%
% ===========================================================================
%   THE NORMALIZATION HIDES A FACTOR OF g
% ===========================================================================
%   The vertical axis of Figs. 20.9-20.12 is  miss[ft] / (nT[g] . T^2[s^2]).
%   The numerator is in FEET but nT in the denominator is in G UNITS, not
%   ft/s^2.  Check against the book's own arithmetic: 20 * 6 * 1^2 = 120 ft
%   for a 6-g target -- if nT were 193.2 ft/s^2 that would read 3864 ft.
%
%   This bites in exactly one place, the no-guidance reference curve.  The
%   coast miss is the weave displacement nT/omega_T^2, so normalized it is
%
%       W.no_guidance = XNT / x^2        and NOT  1 / x^2
%
%   with XNT = 32.2.  Get that wrong and the reference sits a factor of 32
%   too low, PN appears to lose to coasting at every frequency, and the
%   headline conclusion of Fig. 20.9 inverts.  With the factor right the
%   crossover lands between x = 0.7 and x = 0.8, which is the book's stated
%   0.7, and the two denormalized coast numbers the book prints (48.4 ft at
%   x = 1, 12.1 ft at x = 2) come back to three figures.
%
% ===========================================================================
%   "STEADY STATE" IS A CHOICE, AND THIS IS THE CHOICE
% ===========================================================================
%   The miss against a weaving target does NOT decay with flight time -- it
%   oscillates forever at the weave frequency (Fig. 20.1).  So there is no
%   converged single number; the answer is the AMPLITUDE of that oscillation,
%   and it can only be read after the LOOP transient has died.
%
%   Zarchan's rule, kept verbatim: run TF = 0.2 : 0.2 : 20 s, and only let
%   TF > 10 s contribute to the peak.  In the two currencies that matter:
%
%     guidance transient discarded : tf_min / T  time constants
%                                    = 10 T for x >= 0.5 (T is pinned to 1 s)
%                                    = 10/x T for x <  0.5 (>= 20 T)
%     weave periods SEARCHED       : (tf_max - tf_min) * omega_T / (2 pi)
%                                    = 1.59 for x <  0.5 (omega_T pinned to 1)
%                                    = 10x/(2 pi) for x >= 0.5
%                                    -> 0.80 at x = 0.5, 6.37 at x = 4
%
%   Ten time constants is comfortable for the transient.  The thin part is the
%   SEARCH: at x = 0.5 the window holds only 0.80 of a weave period, so the
%   peak of the miss-vs-TF sinusoid is not guaranteed to be inside it.  Both
%   numbers are returned per-x (W.tau_discarded, W.periods_searched) so this
%   is visible rather than assumed.  Measured sensitivity: sliding tf_min over
%   6..14 s moves the benchmark points by 0 to 0.7%, so the transient really
%   is dead by 10 s -- a checked claim, not a hope.
%
%   The peak accumulator starts at 0 and only ever takes POSITIVE samples, so
%   a window that happens to hold no positive miss returns exactly 0 -- a
%   number that reads as "no miss" and means "the search found nothing".  That
%   is refused with an error below rather than returned.
%
% ===========================================================================
%   WHY THE INPUT IS A RATIO AND NOT AN ABSOLUTE LIMIT
% ===========================================================================
%   Saturation is nonlinear, so you cannot normally scale a saturated result.
%   It is however positively homogeneous: sat_{k L}(k u) = k sat_L(u).  Scale
%   the weave amplitude AND the limit together and the whole trajectory scales,
%   so the NORMALIZED miss depends on nLIM and nT only through nLIM/nT.  That
%   is what lets the book publish one family of curves per N', and it is why
%   running at nT = 1 g answers a question about a 6-g target.
%
%   The homogeneity holds only if the two are scaled together, so this
%   function takes 'ratio' and computes XNCLIM = ratio * XNT internally.  An
%   absolute XNCLIM is deliberately NOT accepted: paired with a non-default
%   'xnt' it would silently produce the curve for some other advantage ratio,
%   correctly labelled with the ratio you thought you asked for.
%
%   See also MGS.ZARCHAN.CROSSCHECK, MGS.ZARCHAN.SIZING, PUREPN2D.

arguments
    opts.xnp       (1,1) double = 3.0            % N', effective navigation ratio
    opts.ratio     (1,1) double = Inf            % nLIM / nT, Inf = no limit
    opts.x         (1,:) double = 0.1:0.1:4.0    % omega_T * T
    opts.tf        (1,:) double = 0.2:0.2:20.0   % flight times, s
    opts.tf_min    (1,1) double = 10.0           % discard TF <= this, s
    opts.h         (1,1) double = 0.01           % integration step, s
    opts.vc        (1,1) double = 4000.0         % closing velocity, ft/s
    opts.xnt       (1,1) double = 32.2           % weave amplitude, ft/s^2 (1 g)
    opts.benchmark (1,1) logical = false
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
end

% ⚠️⚠️ NOT `nargin == 0`.  In a function whose `arguments` block declares ONLY
%    name-value options, MATLAB reports **nargin = 0 no matter how many were
%    passed** -- the single packed `opts` struct is not counted as a positional
%    argument.  Measured on R2025b, 2026-08-23:
%        ntest()            -> nargin = 0
%        ntest('A',5)       -> nargin = 0
%        ntest('A',5,'B',1) -> nargin = 0
%    So `nargin == 0` is ALWAYS TRUE here.  In engage2d that made the public
%    entry call the benchmark, which calls the public entry, and the first
%    bare call died with MATLAB:lang:StackOverflow.  In this file the failure
%    would have been quieter and worse: every caller's sweep silently replaced
%    by the benchmark run.
%    ⇒ the flag is the ONLY switch. A bare call runs the nominal single case.

% ---- input guards ---------------------------------------------------------
if ~(opts.ratio > 0)
    error('mgs:zarchan:weaveDesignCurve:ratio', ...
        ['ratio (nLIM/nT) = %g, must be > 0.\n' ...
         'ratio = 0 is NOT the book''s coast curve.  With XNC pinned to zero ' ...
         'the target''s SINE acceleration (it starts weaving from rest at ' ...
         't = 0) integrates to a net velocity nT/omega_T, so Y drifts ' ...
         'LINEARLY and the "peak" simply grows with TF -- you would get a ' ...
         'large number that means nothing.  The book''s no-guidance reference ' ...
         'is the analytic oscillatory amplitude, returned as W.no_guidance; ' ...
         'it is not obtained by running this simulation.'], opts.ratio);
end
if ~(opts.xnp > 0)
    error('mgs:zarchan:weaveDesignCurve:xnp', ...
        'xnp (N'') = %g, must be > 0.', opts.xnp);
end
if ~all(opts.x > 0)
    error('mgs:zarchan:weaveDesignCurve:x', ...
        'x (= omega_T * T) must be strictly positive; got a minimum of %g.', ...
        min(opts.x));
end
if opts.tf_min >= max(opts.tf)
    error('mgs:zarchan:weaveDesignCurve:window', ...
        ['tf_min = %g s but max(tf) = %g s, so NO flight time can contribute ' ...
         'to the peak and every answer would come back as the accumulator''s ' ...
         'initial 0.'], opts.tf_min, max(opts.tf));
end

% TGO = TF - T + 1e-5 and XLAM = Y/(VC*TGO).  That 1e-5 is Zarchan's guard
% against dividing by zero on the final step, and it is only ~3 orders of
% margin.  If TF is not an exact multiple of H the last Heun PREDICTOR lands
% past TF, TGO goes NEGATIVE, and XLAM flips sign and explodes -- corrupting a
% run that otherwise looks fine.  On the book's own grid (TF multiples of 0.2,
% H = 0.01) the only error is float drift of ~1e-13, which the 1e-5 absorbs
% with 8 orders to spare.  Off that grid it is not safe, so refuse rather than
% integrate garbage.
q = opts.tf ./ opts.h;
if any(abs(q - round(q)) > 1e-6)
    bad = find(abs(q - round(q)) > 1e-6, 1);
    error('mgs:zarchan:weaveDesignCurve:grid', ...
        ['tf(%d) = %g is not an integer multiple of h = %g (tf/h = %.9g).\n' ...
         'The final Heun predictor would then evaluate past TF, making ' ...
         'TGO = TF - T + 1e-5 negative and XLAM = Y/(VC*TGO) blow up with a ' ...
         'sign flip.  Choose h so that every tf/h is an integer.'], ...
        bad, opts.tf(bad), opts.h, q(bad));
end

% ---- the sweep ------------------------------------------------------------
W = struct();
W.tag   = sprintf('Listing 20.3  N''=%g  ratio=%g', opts.xnp, opts.ratio);
W.units = 'FEET: miss ft, VC ft/s, nT ft/s^2, g = 32.2 ft/s^2.  NOT SI.';
W.xnp   = opts.xnp;   W.ratio  = opts.ratio;
W.vc    = opts.vc;    W.xnt    = opts.xnt;
W.h     = opts.h;     W.tf_min = opts.tf_min;   W.tf_max = max(opts.tf);
W.x     = opts.x;

nx = numel(opts.x);
W.norm_miss        = nan(1, nx);
W.tau              = nan(1, nx);
W.omega_t          = nan(1, nx);
W.tau_discarded    = nan(1, nx);   % transient discarded, in time constants
W.periods_searched = nan(1, nx);   % weave periods inside the peak-search window
W.steps_per_lag    = nan(1, nx);   % h against the individual lag TAU/5

for i = 1:nx
    [W.norm_miss(i), d] = weavePoint(opts.x(i), opts);
    W.tau(i)              = d.tau;
    W.omega_t(i)          = d.w;
    W.tau_discarded(i)    = opts.tf_min / d.tau;
    W.periods_searched(i) = (W.tf_max - opts.tf_min) * d.w / (2*pi);
    W.steps_per_lag(i)    = (d.tau/5) / opts.h;
end

% See the header: the g inside nT[g] is why this is XNT/x^2 and not 1/x^2.
W.no_guidance = opts.xnt ./ opts.x.^2;

[W.peak, ip] = max(W.norm_miss);
W.x_peak = opts.x(ip);

% Highest x at which guiding still beats coasting -- Fig. 20.9's headline.
better = W.norm_miss < W.no_guidance;
if all(better)
    W.x_pn_beats_coast_upto = Inf;
elseif ~any(better)
    W.x_pn_beats_coast_upto = 0;
else
    W.x_pn_beats_coast_upto = opts.x(find(better, 1, 'last'));
end

% ---- optional book reproduction ------------------------------------------
if opts.benchmark
    W.bench = runBenchmarks(opts);
else
    W.bench = [];
end

W.summary = render(W);
if opts.render, fprintf('%s\n', W.summary); end
if opts.plot,   drawCurve(W);              end
end

% =============================================================================
function [norm_miss, d] = weavePoint(x, opts)
%WEAVEPOINT  One point of the design curve: Listing 20.3's inner two loops.

% Zarchan's branch.  For small x he does NOT shrink omega_T (a weave period of
% 2*pi/0.1 = 63 s would not fit inside a 20 s run, so steady state could never
% be reached); he pins omega_T = 1 and shrinks T instead, then undoes the
% normalization by dividing by TAU^2.  The book only divides in the x < 0.5
% branch because TAU is exactly 1 in the other; dividing always is identical
% arithmetic and one fewer branch to get wrong.
if x < 0.5
    d.w = 1.0;   d.tau = x / d.w;
else
    d.w = x;     d.tau = 1.0;
end

% Heun on a lag of time constant TAU/5 is stable only for h <= 0.4*TAU
% (|h*lambda| <= 2).  At the book's h = 0.01 the tightest point of the default
% sweep is x = 0.1, TAU = 0.1, which is 4x inside the bound but only 2.0 steps
% per lag -- crude, and it shows: refining h from 0.01 to 0.002 moves x = 0.1
% by 2.6% while the benchmark points at x = 1 and x = 2 move by under 0.05%.
% Treat the low-x end of the curve as good to a couple of percent, not to
% three figures.  Past the bound the integration does not merely lose accuracy,
% it diverges into a large finite entirely fictitious "miss", so refuse.
if opts.h > 0.4 * d.tau
    error('mgs:zarchan:weaveDesignCurve:step', ...
        ['x = %g gives TAU = %g, so the five lags each have time constant ' ...
         'TAU/5 = %g s.  h = %g exceeds the Heun stability bound 0.4*TAU = ' ...
         '%g and the integration would diverge into a large, finite, ' ...
         'entirely fictitious "miss".  Reduce h to at most %g.'], ...
        x, d.tau, d.tau/5, opts.h, 0.4*d.tau, 0.4*d.tau);
end

xnclim = opts.ratio * opts.xnt;      % Inf*xnt = Inf, so the clip is a no-op
k5     = 5.0 / d.tau;
h      = opts.h;
tf     = opts.tf(:).';
n      = numel(tf);

% The TF loop is VECTORISED: each column is an independent engagement running
% exactly the book's scalar arithmetic, and they advance together because they
% all start at T = 0 and share H.  Nothing is coupled across columns -- the
% only per-column quantity, TGO = TF - T + 1e-5, is elementwise.  This is a
% large speedup over the transcribed nested loops and changes no number.
% A column that reaches its own TF stops being written and holds its final Y,
% which is precisely what the book's `while T <= TF-1e-5` does.
Y  = zeros(1,n);  YD = zeros(1,n);  XNL = zeros(1,n);
D  = zeros(1,n);  EL = zeros(1,n);  X4  = zeros(1,n);  X5 = zeros(1,n);
T  = 0.0;
nmax = ceil(max(tf)/h) + 5;

% nmax is a backstop only -- the loop is meant to exit on the `act` test below
% once the longest TF is reached.  Exiting on nmax instead would mean T failed
% to advance as expected, and the short columns would report a separation from
% part-way through their run as if it were the terminal value.  Checked for
% real after the loop; this comment is not the guard.
for it = 1:nmax
    act = (T <= tf - 1e-5);
    if ~any(act), break; end

    Y0 = Y;  YD0 = YD;  XNL0 = XNL;  D0 = D;  EL0 = EL;  X40 = X4;  X50 = X5;

    [dD, dEL, dX4, dX5, dXNL, dYD] = ...
        derivs(T, Y, XNL, D, EL, X4, X5, tf, opts, d.w, k5, xnclim);

    Yp  = Y  + h*YD;     YDp  = YD  + h*dYD;    XNLp = XNL + h*dXNL;
    ELp = EL + h*dEL;    Dp   = D   + h*dD;
    X4p = X4 + h*dX4;    X5p  = X5  + h*dX5;

    [dD2, dEL2, dX42, dX52, dXNL2, dYD2] = ...
        derivs(T+h, Yp, XNLp, Dp, ELp, X4p, X5p, tf, opts, d.w, k5, xnclim);

    Yn   = 0.5*(Y0   + Yp   + h*YDp  );
    YDn  = 0.5*(YD0  + YDp  + h*dYD2 );
    XNLn = 0.5*(XNL0 + XNLp + h*dXNL2);
    Dn   = 0.5*(D0   + Dp   + h*dD2  );
    ELn  = 0.5*(EL0  + ELp  + h*dEL2 );
    X4n  = 0.5*(X40  + X4p  + h*dX42 );
    X5n  = 0.5*(X50  + X5p  + h*dX52 );

    Y(act)  = Yn(act);    YD(act) = YDn(act);   XNL(act) = XNLn(act);
    D(act)  = Dn(act);    EL(act) = ELn(act);
    X4(act) = X4n(act);   X5(act) = X5n(act);
    T = T + h;
end

% The backstop promised above, actually enforced.
if any(T <= tf - 1e-5)
    error('mgs:zarchan:weaveDesignCurve:backstop', ...
        ['x = %g: the integration loop hit its nmax backstop of %d steps ' ...
         'with %d of %d flight times still short of TF.  T did not advance ' ...
         'as expected, and the affected columns hold a mid-run separation ' ...
         'that would have been scored as terminal.'], ...
        x, nmax, sum(T <= tf - 1e-5), n);
end

if ~all(isfinite(Y))
    error('mgs:zarchan:weaveDesignCurve:diverged', ...
        ['x = %g produced a non-finite terminal separation on %d of %d ' ...
         'flight times.  The step-size guard passed, so suspect the TGO ' ...
         'sign flip described in this file''s header rather than lag ' ...
         'stability.'], x, sum(~isfinite(Y)), n);
end

% Zarchan's peak logic, kept as-is.  The `> XMWEAVEOLD` test is implied by
% `> XMWEAVEMAX` (a running maximum), so this is just a max over the window --
% but it is transcribed rather than simplified so a reader can diff it against
% the listing line for line.
xmweave_old = 0.0;
xmweave_max = 0.0;
for j = 1:n
    xm = Y(j);
    if xm > xmweave_old && xm > xmweave_max && tf(j) > opts.tf_min
        xmweave_max = xm;
    end
    xmweave_old = xm;
end

% Exactly 0 means the accumulator was never assigned: no POSITIVE miss
% anywhere in the search window.  Returned as a number it reads "this
% configuration has no miss", which is the opposite of the truth -- the search
% window was in the wrong place (usually too few weave periods; see
% W.periods_searched).  Refuse it.
if xmweave_max == 0.0
    error('mgs:zarchan:weaveDesignCurve:nopeak', ...
        ['x = %g: no positive miss at any TF in (%g, %g] s, so the peak ' ...
         'accumulator never fired and the answer would be a spurious 0.\n' ...
         'That window spans %.2f weave periods; the miss oscillates about ' ...
         'zero, so a window shorter than one period can sit entirely in a ' ...
         'trough.  Raise max(tf) or lower tf_min.'], ...
        x, opts.tf_min, max(tf), (max(tf)-opts.tf_min)*d.w/(2*pi));
end

norm_miss = xmweave_max / d.tau^2;
end

% =============================================================================
function [dD, dEL, dX4, dX5, dXNL, dYD] = derivs(t, Y, XNL, D, EL, X4, X5, ...
                                                 tf, opts, w, k5, xnclim)
%DERIVS  Fifth-order binomial homing loop on linearized geometry.
%
%   Five lags of TAU/5 each, so the TOTAL guidance time constant is TAU:
%     D             seeker
%     EL            noise filter; its state is the estimated LOS rate
%     X4, X5, XNL   flight-control system (aerodynamics plus autopilot)
%
%   The limit is applied to the COMMAND XNC, upstream of the last three lags --
%   so the ACHIEVED acceleration XNL is not what was clipped, and can both lag
%   it and overshoot it.  That gap is the entire subject of the saturated
%   curves, and it is the same gap our own gamma-tracking deficit lives in.
%   Clipping XNL instead would be a different and much better behaved system,
%   so do not "tidy" this into a limit on the output.

YTDD = opts.xnt * sin(w*t);              % target weave, nT*sin(omega_T*t)
TGO  = tf - t + 1e-5;                    % see the grid guard on that 1e-5
XLAM = Y ./ (opts.vc * TGO);             % LOS angle, small-angle geometry
dD   = k5 * (XLAM - D);                  % seeker lag; dD doubles as LOS rate
dEL  = k5 * (dD - EL);                   % noise-filter lag
XNC  = opts.xnp * opts.vc * EL;          % N' * Vc * lambda_dot
XNC  = max(-xnclim, min(xnclim, XNC));   % acceleration limit
dX4  = k5 * (XNC - X4);
dX5  = k5 * (X4  - X5);
dXNL = k5 * (X5  - XNL);                 % achieved acceleration
dYD  = YTDD - XNL;                       % relative separation dynamics
end

% =============================================================================
function B = runBenchmarks(opts)
%RUNBENCHMARKS  The points for which the book publishes a number.

B = struct('label', {}, 'xnp', {}, 'ratio', {}, 'x', {}, ...
           'computed', {}, 'book', {}, 'is_book', {});

% Fig. 20.9 / 20.10, N' = 3.  The book's worked example ladder is a 6-g target
% at omega_T = 2 rad/s against a T = 0.5 s loop, i.e. x = 1, denormalizing as
% miss[ft] = norm * 6 * 0.5^2.
B = addCase(B, opts, 'N''=3  unlimited        x=2', 3.0, Inf, 2.0, 20.0, true);
B = addCase(B, opts, 'N''=3  unlimited        x=1', 3.0, Inf, 1.0, 60.0, true);
B = addCase(B, opts, 'N''=3  nLIM/nT = 3      x=1', 3.0, 3.0, 1.0, 70.0, true);
B = addCase(B, opts, 'N''=3  nLIM/nT = 2      x=1', 3.0, 2.0, 1.0, 75.0, true);
B = addCase(B, opts, 'N''=3  nLIM/nT = 1      x=1', 3.0, 1.0, 1.0, 85.0, true);

% The configuration the commissioning brief cited.  NOT a book benchmark: see
% the CORRECTION block in the header.  Printed so the discrepancy is on the
% page rather than rediscovered by the next person who reads the brief.
B = addCase(B, opts, 'N''=5  unlimited        x=1', 5.0, Inf, 1.0, NaN, false);
B = addCase(B, opts, 'N''=5  nLIM/nT = 3      x=1', 5.0, 3.0, 1.0, NaN, false);
end

% -----------------------------------------------------------------------------
function B = addCase(B, opts, label, xnp, ratio, x, book, is_book)
% The benchmark deliberately inherits the caller's tf / tf_min / h / vc / xnt
% so that a caller who changes the numerics sees the book comparison move too.
% A benchmark pinned to its own private settings would keep reporting a match
% while the sweep beside it drifted.
p = opts;
p.xnp = xnp;
p.ratio = ratio;
v = weavePoint(x, p);
B(end+1) = struct('label', label, 'xnp', xnp, 'ratio', ratio, 'x', x, ...
                  'computed', v, 'book', book, 'is_book', is_book);
end

% =============================================================================
function s = render(W)
L = {};
L{end+1} = 'Zarchan Listing 20.3 -- weaving-target normalized design curve';
L{end+1} = sprintf('  %s', W.units);
L{end+1} = sprintf('  fifth-order binomial loop, N'' = %g, nLIM/nT = %g, VC = %g ft/s', ...
    W.xnp, W.ratio, W.vc);
L{end+1} = sprintf('  steady state: TF in (%g, %g] s, h = %g s', ...
    W.tf_min, W.tf_max, W.h);
L{end+1} = '';

if ~isempty(W.bench)
    L{end+1} = 'BOOK REPRODUCTION';
    L{end+1} = repmat('-', 1, 78);
    L{end+1} = sprintf('%-32s %11s %9s %9s', 'case', 'computed', 'book', 'ratio');
    for k = 1:numel(W.bench)
        b = W.bench(k);
        if b.is_book
            L{end+1} = sprintf('%-32s %11.3f %9.0f %9.3f', ...
                b.label, b.computed, b.book, b.computed/b.book); %#ok<AGROW>
        else
            L{end+1} = sprintf('%-32s %11.3f %9s %9s  <-- not a book number', ...
                b.label, b.computed, '--', '--'); %#ok<AGROW>
        end
    end
    L{end+1} = repmat('-', 1, 78);
    L{end+1} = 'Denormalized to the book''s worked example (nT = 6 g, T = 0.5 s,';
    L{end+1} = 'omega_T = 2 rad/s, so x = 1):  peak miss [ft] = norm * 6 * 0.5^2';
    for k = 1:numel(W.bench)
        b = W.bench(k);
        if b.is_book && b.x == 1.0
            L{end+1} = sprintf('   %-30s %8.1f ft    book %5.0f ft', ...
                b.label, b.computed*6*0.25, b.book*6*0.25); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = 'The N''=5 rows are the configuration the commissioning brief called';
    L{end+1} = '"Fig 20.12, 60 -> 70".  The book''s 60 -> 70 example is N''=3 read off';
    L{end+1} = 'Fig 20.10 (see the CORRECTION block in this file''s header).  The';
    L{end+1} = 'N''=5 numbers are printed, not suppressed, and not called a match.';
    L{end+1} = '';
end

L{end+1} = sprintf('DESIGN CURVE  (N'' = %g, nLIM/nT = %g)', W.xnp, W.ratio);
L{end+1} = repmat('-', 1, 78);
nx = numel(W.x);
for a = 1:10:nx
    b = min(a+9, nx);
    L{end+1} = ['  x          ' sprintf('%7.2f', W.x(a:b))];           %#ok<AGROW>
    L{end+1} = ['  norm miss  ' sprintf('%7.2f', W.norm_miss(a:b))];   %#ok<AGROW>
    L{end+1} = ['  no guid.   ' sprintf('%7.2f', W.no_guidance(a:b))]; %#ok<AGROW>
    L{end+1} = '';                                                     %#ok<AGROW>
end
L{end+1} = sprintf('  peak normalized miss %.3f at x = %.2f     (book: max near x = 0.7, ~63)', ...
    W.peak, W.x_peak);
L{end+1} = sprintf('  PN beats coasting up to x = %.2f           (book: ~0.7)', ...
    W.x_pn_beats_coast_upto);
L{end+1} = '';
L{end+1} = 'STEADY-STATE DIAGNOSTICS (worst point of this sweep)';
[pw, iw] = min(W.periods_searched);
L{end+1} = sprintf('  fewest weave periods searched : %.2f at x = %.2f', pw, W.x(iw));
L{end+1} = sprintf('  transient discarded there     : %.1f guidance time constants', ...
    W.tau_discarded(iw));
[sp, is] = min(W.steps_per_lag);
L{end+1} = sprintf('  coarsest integration          : %.1f steps per lag (TAU/5) at x = %.2f', ...
    sp, W.x(is));
L{end+1} = '';
L{end+1} = 'READ THIS BEFORE QUOTING ANY NUMBER ABOVE:';
L{end+1} = 'Chapter 20 is a weaving TARGET.  Our weave is a weaving PURSUER.  They';
L{end+1} = 'are different problems.  What transfers is the machinery -- a fifth-order';
L{end+1} = 'lag driven by a sinusoid, and what a hard acceleration limit does to it';
L{end+1} = '-- not the miss distances.';
s = strjoin(L, newline);
end

% =============================================================================
function drawCurve(W)
figure;
plot(W.x, W.norm_miss, 'LineWidth', 1.6); hold on; grid on;
plot(W.x, W.no_guidance, '--', 'LineWidth', 1.2);
xlabel('normalized weave frequency  x = \omega_T T');
ylabel('normalized peak miss   [ft / (g \cdot s^2)]');
title(sprintf('Zarchan Listing 20.3   N'' = %g,  nLIM/nT = %g   [FEET]', ...
    W.xnp, W.ratio));
legend('proportional navigation', 'no guidance  n_T/\omega_T^2', ...
    'Location', 'northeast');
% The coast reference goes to XNT/x^2 -> 3220 at x = 0.1 and would flatten the
% PN curve to a line if the axis were left to autoscale around it.
ylim([0, max(W.norm_miss)*1.6]);
end
