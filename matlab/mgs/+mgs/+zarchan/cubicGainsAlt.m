function R = cubicGainsAlt(opts)
%MGS.ZARCHAN.CUBICGAINSALT  Zarchan Listing 26.5 -- the Rusnak & Meir route to
%   the six control gains of the CUBIC flight-control-system guidance law.
%
%   R = mgs.zarchan.cubicGainsAlt()
%   R = mgs.zarchan.cubicGainsAlt('WZ', 100)           push the RHP zero away
%   R = mgs.zarchan.cubicGainsAlt('assert', false)     warn instead of error
%   R = mgs.zarchan.cubicGainsAlt('singlelag', false)  drop the Listing 26.3 rows
%
%   Line-by-line port of Listing 26.5 (zarchan.txt:9820-9839), which programs
%   the block diagram of Fig. 26.15.
%
% ===========================================================================
%   UNITS.  *** THIS LISTING HAS NO LENGTH UNIT AT ALL. ***  Everything here
%   is time and gain: tgo in SECONDS, N' dimensionless, C1 [1/s^2], C2 [1/s],
%   C3 dimensionless, C4/C5/C6 in the units of the flight-control-system state
%   each multiplies.  No feet, no metres, no g.  Nothing needs converting at
%   the call site, and nothing here may be assumed to be feet because the rest
%   of the book is.
% ===========================================================================
%
%   WHAT IT IS.  Chapter 26 derives the SAME six gains twice.  Listing 26.4
%   integrates the 6x6 matrix Riccati equation (the optimal-control route,
%   numerically fragile enough to need H = 1e-4).  Listing 26.5 instead
%   evaluates the Rusnak & Meir quantity (:9581, :9592, :9597, :9600)
%
%        L(tgo) = N/D,   N = L^-1{ (1/s^2) nL(s)/nc(s) }(tgo)
%                        D = gamma + int_0^tgo N(t)^2 dt
%        N'(tgo) = L(tgo)*tgo^2
%        C1 = N'/tgo^2,  C2 = N'/tgo,  C3 = 0.5*N'        (:9799-9809, :9366)
%        C_{3+i}(tgo) = -L(tgo) * L^-1{ (1/s^2) nL(s)/p_i(0) }(tgo)
%
%   i.e. N is the plant's RAMP RESPONSE, D the running integral of its square,
%   and each flight-control-system state contributes a gain equal to -L times
%   the ramp response measured with the ramp injected at THAT state's
%   derivative.  Four ramp responses, one quadrature, no matrices, H = 0.01 --
%   "the integration step size H is again 100 times larger than the step size
%   required for the optimal control technique" (:9818).
%
%   The plant is the cubic tail-controlled model of :9754-9761,
%
%        nL/nc = (1 - s^2/wz^2) / [ (1 + s*TAU)(1 + 2*Z*s/W + s^2/W^2) ]
%
%   wz = 5 r/s, TAU = 1 s, Z = 0.7, W = 20 r/s.  With (p1,p2,p3) = (e,ed,edd):
%   p1' = p2,  p2' = p3,  nL = p1 - p3/wz^2, and
%   p3' = (W^2/TAU)*[nc - p1 - (2Z/W+TAU)*p2 - (1/W^2+2*Z*TAU/W)*p3].
%   The MINUS in the output line is the wrong-way tail effect, and it is why
%   N' goes NEGATIVE near intercept.
%
%   *** WHAT IT CANNOT SAY. ***  This computes GAINS, not trajectories: no
%   engagement, no target, no miss distance, no acceleration limit, no
%   saturation, no noise.  It is the design-table generator that Listing 26.6
%   then flies.  Nothing out of here is a miss distance, and nothing out of
%   here prices a manoeuvre.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_26_05.m does not parse.  Four repairs, none of
%   them a judgement call:
%     1  "while ~(T .= (10.-.0001))" -> ">=".  "." is this book's OCR of ">";
%        shape-forced, since T only increases.  The listing hard-codes 10 and
%        defines no TF at all; exposed here as opts.TF.
%     2  "while STEP,=1" -> "<=".  "," is the OCR of "<"; the body sets STEP=2
%        to leave, which only makes sense against "<=".
%     3  "if S.=.09999" -> ">=".  S is zeroed on every record.
%     4  transcription lines 23-27 ("PERFORMANCE COMPARISON OF GUIDANCE
%        LAWS...", "nL nc", "1 s2", "52", and the stray ligature run) are the
%        next SECTION of the book leaking in at :9840.  Dropped, not "fixed".
%
%   The transcription reports NO unresolved variable-to-variable comparison,
%   and that is right: every comparison in 26.5 is against a literal.  What it
%   DOES leave open is which of E1/E2/E2D, E3/E3D/E4, E5/E5D/E5DD is which
%   state.  Settled by algebra, not by guessing: reading each triple as
%   (p1,p2,p3) makes all three chains identical to the main chain with the ramp
%   moved -- E1D = E2+T injects at p1', E3DD = E4+T at p2', and the loose "T -"
%   term of E5DDD at p3' -- and under that reading every feedback coefficient
%   and every output line is term-for-term the main chain's.  No other
%   assignment does that.  SELF-CHECKS 2-5 are the proof that it closed.
%
%   Reproduced deliberately: the recorded C4/C5/C6/XNP are computed inside the
%   SECOND derivative pass, i.e. at the PREDICTED state, before the corrector.
%   That is where the listing computes them; it costs ~1e-4 relative at
%   H = 0.01, and moving them after the corrector would "improve" the port away
%   from the book by more than the benchmark band.  Dropped: "save datfil.txt
%   output /ascii" (a typo for -ascii; nothing is written to disk here) and the
%   trailing "FLAG=0", which the outer loop re-initialises before any read.
%
%   ONE re-association, algebraic only.  The listing writes each gain as a
%   single expression ("C4=-(E1-E2D/WZ^2)*PZ", and "C6=PZ*(-E5+E5DD/WZ^2)",
%   which is the same sign convention written the other way round).  This port
%   names the bracket first (XN4 = E1-E2D/WZ^2, then C4 = -XN4*PZ): identical
%   arithmetic in the same order, and it is what lets the three injected-ramp
%   responses be returned and checked against their asymptotes.  Recovering
%   them afterwards as -C4/PZ would divide by a PZ that passes through zero.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS, and the honest state of the headline claim.
%
%   The book's claim FOR THIS LISTING is :9819 -- N' from the Ref. 6 method "is
%   completely equivalent to that generated by the optimal control method".
%   That comparison lives entirely on Fig. 26.16; every figure became an image
%   link in the extraction, and Listing 26.4 publishes no number for the cubic
%   N'.  It is therefore EMITTED AS A SKIP carrying the arithmetic of what
%   reproducing it would cost -- never scored against a widened band.
%
%   The scored stand-in is the same claim one section earlier, where the
%   optimal-control answer exists in CLOSED FORM.  Listing 26.3 (:9611-9628)
%   runs this identical machinery on the SINGLE-LAG plant and prints, in code
%   (:9627), the closed forms the Riccati/Schwartz route produced:
%
%        XS = tgo/TAU
%        N'_th = 6*XS^2*(e^-XS - 1 + XS) /
%                (2XS^3 + 3 + 6XS - 6XS^2 - 12*XS*e^-XS - 3*e^-2XS + 1e-4)
%        C4_th = -N'_th*(e^-XS + XS - 1)/XS^2
%
%   and :9610 reports the result: "excellent agreement ... when compared to the
%   closed-form solutions".  So this file runs the single-lag case too and
%   scores it.  It exercises exactly what 26.5 reuses -- L = N/D, N' = L*tgo^2,
%   and the injected-ramp construction behind C4.  It is a WEAKER claim than
%   the one on Fig. 26.16 and must never be reported as that one.
%
%   *** THOSE ROWS ARE SCORED ONLY FOR tgo >= TAU, and not for tidiness. ***
%   The two expressions regularise the same singularity DIFFERENTLY: the
%   listing adds GAM = 1e-5 to D, the closed form adds 1e-4 to a denominator
%   equal to 6*D/TAU^3, i.e. an effective gamma of 1.667e-5.  Below tgo ~ TAU
%   that 67% difference IS the answer -- at tgo = 0.1*TAU the two differ by
%   about 64%, and BOTH are far from the unregularised value.  Widening the
%   band to swallow that would be scoring the fudge factor rather than the
%   method, so the window is excluded and named in the row's detail instead.
%
%   The rest are prose about Figs. 26.14/26.16 turned into shape/bound rows:
%     :9766       "approach 3 when the time to go ... is very large", "get
%                 large near intercept", "go to zero at intercept", and the new
%                 one "also goes negative near intercept in an attempt to
%                 compensate for the right-half-plane zero"; also :9773.
%     :9760-9761  "the right-half-plane zero is at the low frequency of 5 r/s.
%                 The dominant time constant is 1 s, and the damping and
%                 natural frequency ... are 0.7 and 20 r/s".  Scored by
%                 expanding the book's FACTORED denominator with conv() and
%                 comparing it against the coefficients the transcribed EDDD
%                 line actually uses, then taking roots().  The numerator half
%                 catches "E + EDD/WZ^2", which would put both zeros on the
%                 imaginary axis and leave no RHP zero at all.
%     :9839       axis([0 10 -10 50]), the window BOTH 26.4 and 26.5 plot N'
%                 in.  A plot range is an author's choice, not a measurement,
%                 so it is a BOUND row.  Worth having because the curve fills
%                 it: about 10% margin at the floor and 16% at the ceiling.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED. ***  MATLAB is the
%   caller's; this file has never been executed here.  The numbers below come
%   from an independent prototype of the same recurrence, so that a first run
%   can be compared against an expectation instead of merely admired:
%
%     nominal case, H = 0.01, samp = 0.1 -> 100 records, tgo = 0.1 .. 10 s
%       N' negative only at tgo = 0.1, 0.2, 0.3   (min -8.96 at tgo = 0.2)
%       N' peak +42.2 at tgo = 0.5, falling monotonically after it
%       N'(10 s) = 3.7495   C4 = -0.3625   C5 = -0.03937   C6 = -8.371e-4
%       and on to 3.35 at tgo = 20 s, 3.17 at 40 s (-> 3, slowly)
%     single-lag cross-check, tgo >= 1 s
%       max |N'/N'_th - 1| ~ 0.017%,  max |C4/C4_th - 1| ~ 0.019%
%       -- and NEITHER falls with H.  The residual is the gamma mismatch above,
%       not discretization; a run where it DOES fall with H means something
%       else is wrong.
%     self-checks 2-5 (ramp-response asymptotes) land near 5e-6 relative.
%
%   A residual near a band EDGE is not a pass to be accepted: the mechanisms
%   above say where each one should land.  Do not widen a band to make a row
%   green.
%
% ---------------------------------------------------------------------------
%   OUTPUTS
%     R.tgo           time-to-go [s], the listing's T
%     R.C1 .. R.C6    the six control gains
%     R.NP            effective navigation ratio N' = L*tgo^2 [-]
%     R.L R.N R.D     the listing's PZ, XN and X+GAM
%     R.N4 R.N5 R.N6  the three injected-ramp responses (C_{3+i} = -L*N_{3+i})
%     R.sl            Listing 26.3 cross-check: .tgo .NP .NP_th .C4 .C4_th
%                     .relNP .relC4   ([] if 'singlelag' is false)
%     R.bench         benchmark rows, schema shared with the other ports
%     R.selfcheck     .names/.oks/.detail/.all_ok -- structural, NOT book values
%     R.pass          true only if a row was SCORED and every one passed
%     R.verdict       '' when clean, else the one-line reason.  ALWAYS formed;
%                     'assert' picks the reaction and nothing else -- true
%                     errors, false WARNS and still returns R complete, so the
%                     failing side stays visible.  benchSuite runs every port
%                     with assert off precisely so one red row cannot hide the
%                     board.
%     R.p             parameters actually used;  R.units;  R.summary
%
%   See also MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 26.5 line 1, verbatim.  Seconds and rad/s.
    opts.TAU (1,1) double {mustBeFinite, mustBePositive} = 1.0     % s
    opts.GAM (1,1) double {mustBeFinite, mustBePositive} = 1e-5    % regulariser
    opts.WZ  (1,1) double {mustBeFinite, mustBePositive} = 5.0     % r/s, RHP zero
    opts.W   (1,1) double {mustBeFinite, mustBePositive} = 20.0    % r/s
    opts.Z   (1,1) double {mustBeFinite, mustBePositive} = 0.7     % damping
    % The listing hard-codes 10 in its while test and defines no TF.
    opts.TF  (1,1) double {mustBeFinite, mustBePositive} = 10.0    % s
    opts.H   (1,1) double {mustBeFinite, mustBePositive} = 0.01    % s, RK2 step
    % The listing's "if S >= .09999" at H = 0.01, stated as an INTERVAL so it
    % keeps its meaning when H changes.  .09999 is the book's guard against
    % floating-point accumulation, not a different sample rate.
    opts.samp (1,1) double {mustBeFinite, mustBePositive} = 0.1    % s
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options, MATLAB reports nargin = 0 however many
    % were passed, so "if nargin == 0" would be a mode that never fires.
    opts.singlelag (1,1) logical = true
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

TAU = opts.TAU;  GAM = opts.GAM;  WZ = opts.WZ;  W = opts.W;  Z = opts.Z;
TF  = opts.TF;   H   = opts.H;    samp = opts.samp;

if H >= samp
    error('mgs:zarchan:cubicGainsAlt:samp', ...
        ['H = %g s is not smaller than the record interval samp = %g s, so ' ...
         'the returned curve would be sampled at the step rate while its own ' ...
         'time vector claimed the coarser one.'], H, samp);
end
if H*W >= 1
    % RK2 does not blow up at W*H ~ 1; it returns a smooth, wrong curve for the
    % quadratic, which is the failure mode this package exists to stop.
    error('mgs:zarchan:cubicGainsAlt:stiff', ...
        ['H = %g s gives W*H = %g >= 1: fewer than one sample per radian of ' ...
         'the quadratic at W = %g r/s.'], H, H*W, W);
end
if TF <= 2*H
    error('mgs:zarchan:cubicGainsAlt:short', ...
        'TF = %g s is not long enough for two steps at H = %g s.', TF, H);
end

% =========================================================================
%   LISTING 26.5.  Transcribed from zarchan.txt:9821-9839.
% =========================================================================
Q = rmCubic(TAU, GAM, WZ, W, Z, TF, H, samp);
if numel(Q.tgo) < 3
    % Refused, not warned about.  The shape rows need an interior peak; two
    % samples would make max() land on an endpoint and pass for the wrong
    % reason.
    error('mgs:zarchan:cubicGainsAlt:empty', ...
        ['the sweep recorded %d sample(s) at TF = %g s, samp = %g s.  The ' ...
         'shape rows need an interior extremum, so at least three are ' ...
         'required for any of this to mean anything.'], numel(Q.tgo), TF, samp);
end

% =========================================================================
%   LISTING 26.3 (zarchan.txt:9611-9628) -- the SAME machinery on the
%   single-lag plant, where the optimal-control answer IS published in closed
%   form.  This is the only book-value comparison available to this file.
% =========================================================================
SL = [];
if opts.singlelag
    SL = rmSingleLag(TAU, GAM, TF, H, samp);
end

R = struct();
R.tgo = Q.tgo;
R.C1 = Q.C1;  R.C2 = Q.C2;  R.C3 = Q.C3;
R.C4 = Q.C4;  R.C5 = Q.C5;  R.C6 = Q.C6;
R.NP = Q.NP;  R.L  = Q.L;   R.N  = Q.N;   R.D = Q.D;
R.N4 = Q.N4;  R.N5 = Q.N5;  R.N6 = Q.N6;
R.sl = SL;
R.units = ['seconds and gains: tgo [s], N'' [-], C1 [1/s^2], C2 [1/s], ' ...
           'C3 [-], C4/C5/C6 in the units of their FCS state.  NO length unit.'];
R.p = struct('TAU',TAU, 'GAM',GAM, 'WZ',WZ, 'W',W, 'Z',Z, 'TF',TF, ...
             'H',H, 'samp',samp, 'n_rec',numel(Q.tgo), 'n_rhs',Q.nEval, ...
             'H_riccati_26_4',1e-4, 'units',R.units);

R.selfcheck = structuralChecks(Q, TAU, WZ, W, Z, TF);

if opts.benchmark
    R.bench = benchmarks(Q, SL, TAU, GAM, WZ, W, Z, TF, H);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false(1,0);
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine" -- that is the shape of green light this repo has been burned
    % by more than once.
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R); end

msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated.  Every scored row ' ...
        'needs the listing''s own plant (TAU = 1 s, WZ = 5, W = 20, Z = 0.7, ' ...
        'GAM = 1e-5, TF = 10 s); this run used TAU = %g, WZ = %g, W = %g, ' ...
        'Z = %g, GAM = %g, TF = %g.  An unvalidated port must not be handed ' ...
        'back silently.'], TAU, WZ, W, Z, GAM, TF);
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault and not a tolerance question.  ' ...
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
        error('mgs:zarchan:cubicGainsAlt:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN, complete.  A port that errors here hands back
        % nothing, and then nobody can tell which side of the comparison moved.
        warning('mgs:zarchan:cubicGainsAlt:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function Q = rmCubic(TAU, GAM, WZ, W, Z, TF, H, samp)
%RMCUBIC  Listing 26.5, zarchan.txt:9821-9839.
%
%   Every constant arrives as an ARGUMENT.  MATLAB local functions cannot see
%   the main function's workspace; an earlier wave in this package read a
%   constant from the caller and threw on the first line of the first run,
%   which static checking could not see beforehand.
%
%   Four independent copies of the same cubic plant run side by side:
%     E,  ED,  EDD    driven by the ramp command nc = T   -> N, and D = int N^2
%     E1, E2,  E2D    ramp injected at p1'                -> N4
%     E3, E3D, E4     ramp injected at p2'                -> N5
%     E5, E5D, E5DD   ramp injected at p3'                -> N6
%
%   The two long coefficients are written INLINE, exactly as the listing writes
%   them, in all four places.  They are deliberately NOT hoisted into a shared
%   constant: hoisting would make SELF-CHECKS 2-5 test one expression four
%   times instead of four expressions once.

T = 0.0;  S = 0.0;
X  = 0.0;
E  = 0.0;  ED  = 0.0;  EDD  = 0.0;
E1 = 0.0;  E2  = 0.0;  E2D  = 0.0;
E3 = 0.0;  E3D = 0.0;  E4   = 0.0;
E5 = 0.0;  E5D = 0.0;  E5DD = 0.0;

% Derivatives pre-initialised so the FLAG==1 branch cannot read a leftover
% value from anything else on the first pass.
XD  = 0.0;  EDDD = 0.0;  E1D = 0.0;  E2DD = 0.0;
E4D = 0.0;  E3DD = 0.0;  E5DDD = 0.0;
XN  = 0.0;  PZ = 0.0;  XNP = 0.0;
XN4 = 0.0;  XN5 = 0.0;  XN6 = 0.0;
C4  = 0.0;  C5 = 0.0;  C6 = 0.0;

nmax = ceil(TF/min(samp,H)) + 8;
tgo = zeros(nmax,1);  aC4 = zeros(nmax,1);  aC5 = zeros(nmax,1);
aC6 = zeros(nmax,1);  aNP = zeros(nmax,1);  aL  = zeros(nmax,1);
aN  = zeros(nmax,1);  aD  = zeros(nmax,1);
aN4 = zeros(nmax,1);  aN5 = zeros(nmax,1);  aN6 = zeros(nmax,1);
n = 0;  nEval = 0;

while ~(T >= (TF - 0.0001))
    S = S + H;
    XOLD  = X;   EOLD  = E;    EDOLD = ED;   EDDOLD = EDD;
    E1OLD = E1;  E2OLD = E2;   E2DOLD = E2D;
    E3OLD = E3;  E3DOLD = E3D; E4OLD = E4;
    E5OLD = E5;  E5DOLD = E5D; E5DDOLD = E5DD;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  T advances INSIDE it, exactly once,
    % which is what makes the second evaluation happen at t+H.  Move that line
    % out and the scheme degrades to first order while still running smoothly.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            X  = X + H*XD;      E   = E + H*ED;       ED  = ED + H*EDD;
            EDD = EDD + H*EDDD; E1  = E1 + H*E1D;     E2  = E2 + H*E2D;
            E2D = E2D + H*E2DD; E3  = E3 + H*E3D;     E3D = E3D + H*E3DD;
            E4  = E4 + H*E4D;   E5  = E5 + H*E5D;     E5D = E5D + H*E5DD;
            E5DD = E5DD + H*E5DDD;
            T = T + H;
        end
        % --- main chain: the plant driven by the ramp command nc = T --------
        EDDD = W*W*(T - (1./W^2 + 2.*Z*TAU/W)*EDD - (2.*Z/W + TAU)*ED - E)/TAU;
        XN   = E - EDD/WZ^2;        % nL.  The MINUS is the wrong-way tail.
        XD   = XN^2;
        D    = X + GAM;
        PZ   = XN/D;                % L(tgo)
        XNP  = T*T*PZ;              % N'(tgo)
        % --- C4 chain: ramp injected at p1' ---------------------------------
        E2DD = W*W*(-(1./W^2 + 2.*Z*TAU/W)*E2D - (2.*Z/W + TAU)*E2 - E1)/TAU;
        E1D  = E2 + T;
        XN4  = E1 - E2D/WZ^2;
        C4   = -XN4*PZ;
        % --- C5 chain: ramp injected at p2' ---------------------------------
        E4D  = W*W*(-(1./W^2 + 2.*Z*TAU/W)*E4 - (2.*Z/W + TAU)*E3D - E3)/TAU;
        E3DD = E4 + T;
        XN5  = E3 - E4/WZ^2;
        C5   = -XN5*PZ;
        % --- C6 chain: ramp injected at p3' ---------------------------------
        E5DDD = T - W*W*((1./W^2 + 2.*Z*TAU/W)*E5DD + (2.*Z/W + TAU)*E5D + E5)/TAU;
        XN6   = E5 - E5DD/WZ^2;
        C6    = -XN6*PZ;
        FLAG  = 1;
        nEval = nEval + 1;
    end
    % The listing's trailing "FLAG=0" is dropped: the outer loop re-initialises
    % FLAG, so the assignment cannot be read.  Recorded here rather than
    % deleted silently, so nobody re-adds it thinking a state machine was lost.

    % Heun corrector, in the listing's own algebra: .5*(old + predicted + H*f),
    % which equals old + (H/2)*(f_old + f_pred).
    X    = .5*(XOLD + X + H*XD);
    E    = .5*(EOLD + E + H*ED);
    ED   = .5*(EDOLD + ED + H*EDD);
    EDD  = .5*(EDDOLD + EDD + H*EDDD);
    E1   = .5*(E1OLD + E1 + H*E1D);
    E2   = .5*(E2OLD + E2 + H*E2D);
    E2D  = .5*(E2DOLD + E2D + H*E2DD);
    E3   = .5*(E3OLD + E3 + H*E3D);
    E3D  = .5*(E3DOLD + E3D + H*E3DD);
    E4   = .5*(E4OLD + E4 + H*E4D);
    E5   = .5*(E5OLD + E5 + H*E5D);
    E5D  = .5*(E5DOLD + E5D + H*E5DD);
    E5DD = .5*(E5DDOLD + E5DD + H*E5DDD);

    if S >= samp - 0.5*H
        S = 0.0;
        n = n + 1;
        % SECOND-PASS values, i.e. computed at the predicted state.  That is
        % where the listing computes them; see the header before "fixing" it.
        tgo(n) = T;    aC4(n) = C4;   aC5(n) = C5;   aC6(n) = C6;
        aNP(n) = XNP;  aL(n)  = PZ;   aN(n)  = XN;   aD(n)  = X + GAM;
        aN4(n) = XN4;  aN5(n) = XN5;  aN6(n) = XN6;
    end
end

Q = struct();
Q.tgo = tgo(1:n);   Q.NP = aNP(1:n);   Q.L  = aL(1:n);
Q.N   = aN(1:n);    Q.D  = aD(1:n);
Q.C4  = aC4(1:n);   Q.C5 = aC5(1:n);   Q.C6 = aC6(1:n);
Q.N4  = aN4(1:n);   Q.N5 = aN5(1:n);   Q.N6 = aN6(1:n);
% zarchan.txt:9799-9809 (and :9366, where Listing 26.1 prints the same three):
% C1 = N'/tgo^2, C2 = N'/tgo, C3 = 0.5*N'.  Written the way Listing 26.3 writes
% them (:9627) so that no division by tgo appears as tgo -> 0.
Q.C1 = Q.L;
Q.C2 = Q.L .* Q.tgo;
Q.C3 = 0.5 * Q.L .* Q.tgo.^2;
Q.nEval = nEval;
end


% =============================================================================
function SL = rmSingleLag(TAU, GAM, TF, H, samp)
%RMSINGLELAG  Listing 26.3, zarchan.txt:9611-9628.  The SAME Rusnak & Meir
%   machinery on the single-lag plant, plus the closed forms the book prints
%   beside it -- which came from the optimal-control route, and are therefore
%   the only published optimal-control answer this file can be scored against.
T = 0.0;  S = 0.0;
X1 = 0.0;  X2 = 0.0;  X3 = 0.0;
X1D = 0.0; X2D = 0.0; X3D = 0.0;

nmax = ceil(TF/min(samp,H)) + 8;
tg  = zeros(nmax,1);  NP = zeros(nmax,1);  NPth = zeros(nmax,1);
C4v = zeros(nmax,1);  C4th = zeros(nmax,1);
n = 0;

while ~(T >= (TF - 0.0001))
    S = S + H;
    X1OLD = X1;  X2OLD = X2;  X3OLD = X3;
    STEP = 1;  FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            X1 = X1 + H*X1D;  X2 = X2 + H*X2D;  X3 = X3 + H*X3D;
            T = T + H;
        end
        X1D = (T - X1)/TAU;    % ramp response of the lag
        X2D = X1^2;            % running int N^2
        X3D = -X3/TAU + T;     % ramp injected at the lag state's derivative
        FLAG = 1;
    end
    X1 = .5*(X1OLD + X1 + H*X1D);
    X2 = .5*(X2OLD + X2 + H*X2D);
    X3 = .5*(X3OLD + X3 + H*X3D);
    if S >= samp - 0.5*H
        S = 0.0;
        n = n + 1;
        PZ = X1/(X2 + GAM);
        tg(n) = T;  NP(n) = PZ*T*T;  C4v(n) = -X3*PZ;
        % The book's closed forms, verbatim from :9627.  The +.0001 is the
        % book's OWN regulariser and is NOT the listing's GAM -- see header.
        XS   = T/TAU;
        TOP  = 6.*XS*XS*(exp(-XS) - 1. + XS);
        BOT1 = 2*XS*XS*XS + 3. + 6.*XS - 6.*XS*XS;
        BOT2 = -12.*XS*exp(-XS) - 3.*exp(-2.*XS);
        NPth(n) = TOP/(BOT1 + BOT2 + .0001);
        C4th(n) = -NPth(n)*(exp(-XS) + XS - 1.)/(XS*XS);
    end
end

SL = struct();
SL.tgo = tg(1:n);   SL.NP = NP(1:n);   SL.NP_th = NPth(1:n);
SL.C4  = C4v(1:n);  SL.C4_th = C4th(1:n);
SL.relNP = SL.NP ./ SL.NP_th - 1;
SL.relC4 = SL.C4 ./ SL.C4_th - 1;
end


% =============================================================================
function K = structuralChecks(Q, TAU, WZ, W, Z, TF)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never evidence that
%   this port reproduces Zarchan -- only the weaker claim that it agrees with
%   itself, which is what catches wiring faults.
%
%   Checks 2-5 are the ones that matter.  Each of the four chains is a linear
%   plant driven by a ramp, so each ramp response has the exact asymptote
%   f(0)*t + f'(0), taken from the transfer function the BOOK prints and
%   evaluated here by hand-differentiated logarithmic derivatives.  They share
%   no code with the integrator, and each depends on a DIFFERENT combination of
%   the two long coefficients, so a typo in any ONE of the four transcribed
%   copies moves exactly one of them.  The residual is the undecayed transient,
%   which is why the tolerance carries exp(-TF/TAU) and why it does NOT fall
%   with H.
K = struct();
names = {};  oks = [];  det = {};

a1  = 2*Z/W + TAU;            % the coefficient on edot
a2  = 1/W^2 + 2*Z*TAU/W;      % the coefficient on eddot
Kg  = W*W/TAU;
wz2 = WZ^2;
tol = max(2e-3, 20*exp(-TF/TAU));    % envelope of the leftover transient
K.tol_asym = tol;

% 1  D = GAM + int N^2 cannot decrease.  A sign error on XD, or XD wired to a
%    signal that changes sign, shows up here and nowhere else cheap.
K.D_min_step = min(diff(Q.D));
names{end+1} = 'D = gamma + int N^2 non-decreasing';
oks(end+1)   = K.D_min_step >= -1e-15;
det{end+1}   = sprintf('min consecutive change %+.3e', K.D_min_step);

% 2-5  ramp-response asymptotes, one per chain:
%      main  nL/nc         f(0) = 1,           f'(0) = -a1
%      C4    ramp at p1'   f(0) = a1,          f'(0) = (a2+1/wz2) - a1^2
%      C5    ramp at p2'   f(0) = a2 + 1/wz2,  f'(0) = (1+Kg*a1/wz2)/Kg - f(0)*a1
%      C6    ramp at p3'   f(0) = 1/Kg,        f'(0) = -a1/Kg
t   = Q.tgo(end);
f0  = [1, a1, a2 + 1/wz2, 1/Kg];
f1  = [-a1, (a2 + 1/wz2) - a1^2, ...
       (1 + Kg*a1/wz2)/Kg - (a2 + 1/wz2)*a1, -a1/Kg];
got = [Q.N(end), Q.N4(end), Q.N5(end), Q.N6(end)];
nm  = {'main ramp response N', 'C4 chain ramp response', ...
       'C5 chain ramp response', 'C6 chain ramp response'};
K.asym_rel = zeros(1,4);
for i = 1:4
    pred = f0(i)*t + f1(i);
    rel  = (got(i) - pred)/max(abs(pred), realmin);
    K.asym_rel(i) = rel;
    names{end+1} = sprintf('%s -> %.6g*t %+.6g', nm{i}, f0(i), f1(i)); %#ok<AGROW>
    oks(end+1)   = abs(rel) <= tol;                                    %#ok<AGROW>
    det{end+1}   = sprintf('got %.9g, asymptote %.9g, rel %+.2e (tol %.1e)', ...
                           got(i), pred, rel, tol);                    %#ok<AGROW>
end

K.names  = names;
K.oks    = logical(oks);
K.detail = det;
K.ok     = all(K.oks);
K.all_ok = K.ok;
if K.all_ok
    K.first_bad = '';
else
    j = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{j}, det{j});
end
end


% =============================================================================
function B = benchmarks(Q, SL, TAU, GAM, WZ, W, Z, TF, H)
%BENCHMARKS  Ten rows.  Every book number carries its zarchan.txt line number
%   in .source.  Nothing here was read off a figure: the extraction turned
%   every figure into an image link.
B = emptyBench();

bookPlant = abs(TAU-1) < 1e-12 && abs(WZ-5) < 1e-12 && ...
            abs(W-20) < 1e-12 && abs(Z-0.7) < 1e-12;
bookCase  = bookPlant && abs(GAM-1e-5) < 1e-18 && abs(TF-10) < 1e-12;

NP = Q.NP;  tg = Q.tgo;
[pk, ipk] = max(NP);
[mn, imn] = min(NP);

% ---- 1  the plant itself, against the prose that names its poles and zero ---
if bookPlant
    % The book's FACTORED denominator, expanded independently with conv():
    denFac   = conv([TAU 1], [1/W^2, 2*Z/W, 1]);
    % The coefficients the transcribed EDDD line actually uses:
    denTrans = [TAU/W^2, (1./W^2 + 2.*Z*TAU/W), (2.*Z/W + TAU), 1];
    coefDev  = max(abs(denFac - denTrans))/max(abs(denFac));
    pr   = roots(denFac);
    rre  = pr(abs(imag(pr)) < 1e-9);      % the real pole
    cpx  = pr(imag(pr) >  1e-9);          % one of the quadratic pair
    zr   = roots([-1/WZ^2, 0, 1]);        % from nL = e - eddot/wz^2
    zrp  = max(real(zr));                 % the RIGHT-half-plane zero
    tauR = -1/real(rre(1));
    wR   = abs(cpx(1));
    zeR  = -real(cpx(1))/abs(cpx(1));
    dev  = max([coefDev, abs(tauR/1 - 1), abs(wR/20 - 1), ...
                abs(zeR/0.7 - 1), abs(zrp/5 - 1)]);
    B = addBound(B, 'cubic FCS poles and RHP zero vs prose', ...
        'tau=1 s, zeta=0.7, w=20 r/s, RHP zero=+5 r/s', ...
        sprintf('tau=%.9g, zeta=%.9g, w=%.9g, zero=%+.9g', tauR, zeR, wR, zrp), ...
        dev <= 1e-8, ...
        'zarchan.txt:9760-9761 (prose); plant restated numerically at :9812-9817', 's, -, r/s', ...
        sprintf(['max rel dev %.2e over {expansion, tau, zeta, w, RHP zero}. ' ...
        'The expansion term (%.2e) compares conv([TAU 1],[1/W^2 2Z/W 1]) ' ...
        'against the two inline coefficients of the transcribed EDDD line; ' ...
        'the zero term catches "E + EDD/WZ^2", which would move both zeros ' ...
        'onto the imaginary axis and leave no RHP zero at all.'], dev, coefDev));
else
    B = addSkip(B, 'cubic FCS poles and RHP zero vs prose', ...
        'tau=1 s, zeta=0.7, w=20 r/s, RHP zero=+5 r/s', 'exact', ...
        'zarchan.txt:9760-9761', 's, -, r/s', ...
        sprintf(['off-book plant (TAU=%g, Z=%g, W=%g, WZ=%g): the prose names ' ...
        'the nominal case only.'], TAU, Z, W, WZ));
end

% ---- 2-4  Listing 26.3 against the closed forms the Riccati route produced --
% Band: the two expressions regularise D differently -- GAM = 1e-5 against an
% effective 1e-4*TAU^3/6 = 1.667e-5 at TAU = 1.  That 6.67e-6 mismatch sits
% against int N^2 = 0.0299*TAU^3 at tgo = TAU, i.e. 2.2e-4/TAU^3, so the band
% is that envelope with a 1e-3 floor and the window starts at tgo = TAU.
slTol   = max(1e-3, 5e-4/TAU^3);
src26_3 = ['zarchan.txt:9610 (prose "excellent agreement") and :9627 ' ...
           '(the closed forms, printed inside Listing 26.3)'];
if ~isempty(SL) && abs(GAM-1e-5) < 1e-18
    w = SL.tgo >= TAU - 1e-9;
    B = addVal(B, 'single-lag N'' at tgo = TF vs closed form', ...
        SL.NP_th(end), SL.NP(end), 'closed form', slTol, src26_3, '-', ...
        sprintf(['tgo = %.4g s.  This is the SUBSTITUTE for the Fig. 26.16 ' ...
        'claim: same machinery, single-lag plant, where the optimal-control ' ...
        'answer is published.'], SL.tgo(end)));
    B = addVal(B, 'single-lag C4 at tgo = TF vs closed form', ...
        SL.C4_th(end), SL.C4(end), 'closed form', slTol, src26_3, '-', ...
        sprintf(['tgo = %.4g s.  C4 is the injected-ramp construction that ' ...
        'Listing 26.5 reuses three times for the cubic plant.'], SL.tgo(end)));
    if any(w)
        rn = max(abs(SL.relNP(w)));
        rc = max(abs(SL.relC4(w)));
        [~, jn] = max(abs(SL.relNP(w)));
        tw = SL.tgo(w);
        B = addBound(B, 'single-lag N'' and C4 over tgo >= TAU', ...
            sprintf('agree to < %.2f%%', 100*slTol), ...
            sprintf('N'' %.4f%%, C4 %.4f%%', 100*rn, 100*rc), ...
            max(rn, rc) <= slTol, src26_3, '-', ...
            sprintf(['worst N'' deviation at tgo = %.3g s.  BELOW tgo = TAU ' ...
            'the two regularisers differ by 67%% and the comparison is ' ...
            'meaningless (at tgo = 0.1*TAU they disagree by about 64%%, and ' ...
            'both are far from the unregularised value) -- that window is ' ...
            'excluded, not absorbed into a wider band.'], tw(jn)));
    end
else
    if isempty(SL)
        why = 'the ''singlelag'' switch is off, so Listing 26.3 was not run';
    else
        why = sprintf(['GAM = %g, not the listing''s 1e-5.  The book''s closed ' ...
            'form carries its own hard-coded regulariser (1e-4), so the two ' ...
            'stop describing the same problem.'], GAM);
    end
    nmS = {'single-lag N'' at tgo = TF vs closed form', ...
           'single-lag C4 at tgo = TF vs closed form', ...
           'single-lag N'' and C4 over tgo >= TAU'};
    for i = 1:numel(nmS)
        B = addSkip(B, nmS{i}, 'closed form', 'closed form', src26_3, '-', why);
    end
end

% ---- 5-8  prose about Figs. 26.14 / 26.16 ----------------------------------
srcFig = ['zarchan.txt:9766 (prose) and :9773 (Fig. 26.14 caption, ' ...
          '"New guidance-law effective navigation ratio goes negative near ' ...
          'intercept")'];
if bookCase
    tail = NP(max(1,end-29):end);
    approaches3 = NP(end) > 3 && NP(end) < 4.5 && all(diff(tail) < 0);
    B = addBound(B, 'N'' approaches 3 at large tgo', '-> 3 from above', ...
        sprintf('%.4f at tgo = %.4g s, tail decreasing', NP(end), tg(end)), ...
        approaches3, srcFig, '-', ...
        ['scored as a bound and NOT as a value against 3: "approach 3 when ' ...
         'the time to go ... is very large" is an asymptote, and tgo = ' ...
         '10*TAU is not large.  The approach is slow -- an off-book ' ...
         'extension gives 3.35 at tgo = 20 s and 3.17 at 40 s -- so a value ' ...
         'row against 3 would need a 25% band, which is not a check.']);

    % any() first: with no sample at tgo >= TAU, min([]) is EMPTY and "&&"
    % errors on an empty operand -- a crash thrown far from the samp value
    % that caused it.  Short-circuiting turns that into an honest false.
    negNear = mn < 0 && tg(imn) < TAU && any(tg >= TAU) && ...
              min(NP(tg >= TAU)) > 0;
    B = addShape(B, 'N'' goes NEGATIVE near intercept', ...
        'negative near intercept (RHP-zero compensation)', ...
        sprintf('min %.4f at tgo = %.4g s; positive for every tgo >= TAU', ...
                mn, tg(imn)), ...
        negNear, srcFig, ...
        ['this is the row that adjudicates the SIGN of the output line. ' ...
         '"E + EDD/WZ^2" deletes the negative excursion while leaving a ' ...
         'perfectly plausible curve behind, and nothing else here notices.']);

    bigNear = tg(ipk) < TAU && pk > 5*NP(end);
    B = addShape(B, 'N'' gets LARGE near intercept', ...
        'large near intercept, zero at intercept', ...
        sprintf('peak %.3f at tgo = %.4g s = %.1fx N''(TF)', ...
                pk, tg(ipk), pk/NP(end)), ...
        bigNear, srcFig, ...
        sprintf(['"goes to zero at intercept" is deliberately NOT scored as a ' ...
        'separate row: N'' = L*tgo^2 is zero at tgo = 0 by construction, so ' ...
        'it would be a tautology dressed as a check.  The first record here ' ...
        'is tgo = %.4g s.'], tg(1)));

    B = addBound(B, 'N'' inside the plotted window', ...
        'axis([0 10 -10 50])', sprintf('[%.3f, %.3f]', mn, pk), ...
        mn >= -10 && pk <= 50, ...
        ['zarchan.txt:9839 (Listing 26.5''s own axis) and :9785 (Listing ' ...
         '26.4, identical limits)'], '-', ...
        ['a plot range is the author''s choice, not a measurement -- hence a ' ...
         'bound row, not a value row.  It is worth having because the curve ' ...
         'fills it: the margins are about 10% at the floor and 16% at the ' ...
         'ceiling, so a gross transcription error leaves the window.']);
else
    nmF = {'N'' approaches 3 at large tgo', 'N'' goes NEGATIVE near intercept', ...
           'N'' gets LARGE near intercept', 'N'' inside the plotted window'};
    for i = 1:numel(nmF)
        B = addSkip(B, nmF{i}, 'Fig. 26.14 prose', 'shape', srcFig, '-', ...
            sprintf(['off-book case (TAU=%g, WZ=%g, W=%g, Z=%g, GAM=%g, ' ...
            'TF=%g): the prose describes the nominal cubic plant only.'], ...
            TAU, WZ, W, Z, GAM, TF));
    end
end

% ---- 9  step size: reported, never scored ----------------------------------
B = addInfo(B, 'H is 100x the optimal-control step', ...
    'H(26.5) = 100 * H(26.4)', ...
    sprintf('%.4g / %.4g = %.4g', H, 1e-4, H/1e-4), ...
    ['zarchan.txt:9818 (prose), :9826 (Listing 26.5, H=.01), :9777 ' ...
     '(Listing 26.4, H=.0001)'], ...
    ['a statement about program structure, not a quantity the book measured. ' ...
     'It is the whole practical point of the Ref. 6 route, and it is why this ' ...
     'port costs milliseconds where a 26.4 port would cost seconds.']);

% ---- 10  the headline claim, honestly skipped ------------------------------
B = addSkip(B, 'cubic N'': Ref. 6 route == optimal-control route', ...
    'Fig. 26.16: "completely equivalent"', 'graph', ...
    'zarchan.txt:9819 (prose about Fig. 26.16)', '-', ...
    ['NOT SCOREABLE HERE.  The comparison lives only on Fig. 26.16, and every ' ...
     'figure became an image link in the extraction; Listing 26.4 publishes ' ...
     'no number for the cubic N''.  Reproducing it needs 26.4''s 6x6 matrix ' ...
     'Riccati integration at its own H = 1e-4: 1e5 steps x 2 passes x a 6x6 ' ...
     'matmul, order seconds against this board''s entire budget -- and the ' ...
     'Riccati cannot simply be run coarser, because its quadratic term ' ...
     'carries a 1/GAM = 1e5 factor that makes explicit RK2 unstable at any ' ...
     'cheaper step.  The scored stand-in is rows 2-4: the SAME machinery on ' ...
     'the single-lag plant, where the optimal-control answer IS published. ' ...
     'That is a weaker claim and must not be reported as this one.']);
end


% =============================================================================
%   ROW CONSTRUCTORS.  Every row is built through mkrow so the field ORDER
%   cannot drift: a struct array whose fields reordered makes [B.ok] mean
%   something else, and benchSuite's adaptGeneric is strict by design.
% =============================================================================
function B = emptyBench()
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = mkrow(B, what, kind, book, bookStr, ours, oursStr, pct, band, ...
                   tol, ok, skipped, why, source, units, detail)
i = numel(B) + 1;
B(i).what    = what;      B(i).kind     = kind;
B(i).book    = book;      B(i).book_str = bookStr;
B(i).ours    = ours;      B(i).ours_str = oursStr;
B(i).pct     = pct;       B(i).band     = band;
B(i).tol     = tol;       B(i).ok       = logical(ok);
B(i).skipped = logical(skipped);
B(i).why     = why;       B(i).source   = source;
B(i).units   = units;     B(i).detail   = detail;
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
if ~isfinite(ours) || ~isfinite(book) || book == 0
    B = mkrow(B, what, 'value', book, sprintf('%.5g', book), ours, ...
        sprintf('%.5g', ours), NaN, band, tol, false, true, ...
        'the computed value is not finite, or the book value is zero', ...
        source, units, detail);
    return
end
B = mkrow(B, what, 'value', book, sprintf('%.5g', book), ours, ...
    sprintf('%.5g', ours), 100*(ours/book - 1), band, tol, ...
    abs(ours/book - 1) <= tol, false, '', source, units, detail);
end

function B = addShape(B, what, bookStr, oursStr, ok, source, detail)
B = mkrow(B, what, 'shape', NaN, bookStr, NaN, oursStr, NaN, 'shape', NaN, ...
    ok, false, '', source, '-', detail);
end

function B = addBound(B, what, bookStr, oursStr, ok, source, units, detail)
B = mkrow(B, what, 'bound', NaN, bookStr, NaN, oursStr, NaN, 'bound', NaN, ...
    ok, false, '', source, units, detail);
end

function B = addInfo(B, what, bookStr, oursStr, source, why)
% Reported, never scored: an INFO row can neither pass nor fail.
B = mkrow(B, what, 'info', NaN, bookStr, NaN, oursStr, NaN, 'info', NaN, ...
    false, true, why, source, '-', '');
end

function B = addSkip(B, what, bookStr, band, source, units, why)
% A check that could NOT run.  Distinct from INFO, and it must stay distinct:
% INFO means "computed but refused to adjudicate", SKIP means "did not happen".
B = mkrow(B, what, 'skip', NaN, bookStr, NaN, '--', NaN, band, NaN, ...
    false, true, why, source, units, why);
end


% =============================================================================
function s = render(R, ev)
L = {};
L{end+1} = ['mgs.zarchan.cubicGainsAlt -- Zarchan Listing 26.5 ' ...
            '(Rusnak & Meir gains, cubic flight control system)'];
L{end+1} = sprintf(['  plant: TAU=%g s  WZ=%g r/s  W=%g r/s  Z=%g  GAM=%g   |   ' ...
    'sweep: TF=%g s  H=%g s  samp=%g s  (%d records, %d rhs evals)'], ...
    R.p.TAU, R.p.WZ, R.p.W, R.p.Z, R.p.GAM, R.p.TF, R.p.H, R.p.samp, ...
    R.p.n_rec, R.p.n_rhs);
L{end+1} = sprintf('  units: %s', R.units);
[pk, ipk] = max(R.NP);
[mn, imn] = min(R.NP);
L{end+1} = sprintf(['  N'': min %.4f at tgo=%.4g s | peak %.3f at tgo=%.4g s | ' ...
    'N''(%.4g s)=%.4f'], mn, R.tgo(imn), pk, R.tgo(ipk), R.tgo(end), R.NP(end));
L{end+1} = sprintf(['  gains at tgo=%.4g s: C1=%.5g C2=%.5g C3=%.5g ' ...
    'C4=%.5g C5=%.5g C6=%.5g'], R.tgo(end), R.C1(end), R.C2(end), ...
    R.C3(end), R.C4(end), R.C5(end), R.C6(end));
if ~isempty(R.sl)
    w = R.sl.tgo >= R.p.TAU - 1e-9;
    if any(w)
        % max([]) is [] and sprintf would then silently truncate its own
        % format string rather than complaining.
        L{end+1} = sprintf(['  Listing 26.3 cross-check (tgo>=TAU): ' ...
            'max|N''/N''_th-1| = %.4f%%, max|C4/C4_th-1| = %.4f%%'], ...
            100*max(abs(R.sl.relNP(w))), 100*max(abs(R.sl.relC4(w))));
    else
        L{end+1} = sprintf(['  Listing 26.3 cross-check: NO sample at ' ...
            'tgo >= TAU = %g s (samp = %g s), so nothing was compared.'], ...
            R.p.TAU, R.p.samp);
    end
end
L{end+1} = '  --- benchmarks ---';
for i = 1:numel(R.bench)
    b = R.bench(i);
    switch lower(b.kind)
        case 'info', tag = 'INFO';
        case 'skip', tag = 'SKIP';
        otherwise
            if b.skipped
                tag = 'SKIP';
            elseif b.ok
                tag = ' ok ';
            else
                tag = 'FAIL';
            end
    end
    if strcmpi(b.kind, 'value') && ~b.skipped
        L{end+1} = sprintf(['   [%s] %-44s book %-10s ours %-10s ' ...
            '(%+.3f%%, tol %.2g)'], tag, b.what, b.book_str, b.ours_str, ...
            b.pct, b.tol); %#ok<AGROW>
    else
        L{end+1} = sprintf('   [%s] %-44s book %-30s ours %s', ...
            tag, b.what, b.book_str, b.ours_str); %#ok<AGROW>
    end
end
L{end+1} = '  --- structural (self-consistency, NOT book values) ---';
for i = 1:numel(R.selfcheck.names)
    if R.selfcheck.oks(i)
        tag = ' ok ';
    else
        tag = 'FAIL';
    end
    L{end+1} = sprintf('   [%s] %-46s %s', tag, R.selfcheck.names{i}, ...
        R.selfcheck.detail{i}); %#ok<AGROW>
end
L{end+1} = sprintf('  scored rows: %d   pass: %d', sum(ev), R.pass);
s = strjoin(L, newline);
end


% =============================================================================
function drawCurves(R)
figure;
plot(R.tgo, R.NP);  grid on;
xlabel('Time to go (s)');  ylabel('NP');
axis([0 R.p.TF -10 50]);        % the listing's own axis, zarchan.txt:9839
title('Listing 26.5: effective navigation ratio, cubic flight control system');
end
