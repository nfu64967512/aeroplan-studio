function R = optimalGainsWeave(opts)
%MGS.ZARCHAN.OPTIMALGAINSWEAVE  Zarchan Listing 26.2 -- the matrix Riccati
%   equation integrated to get the OPTIMAL CONTROL GAINS AGAINST A WEAVING
%   (sinusoidal) TARGET, scored against the book's closed-form gains.
%
%   R = mgs.zarchan.optimalGainsWeave()
%   R = mgs.zarchan.optimalGainsWeave('W', 2.0)             other weave freq
%   R = mgs.zarchan.optimalGainsWeave('H', 1e-3)            10x faster, coarser
%   R = mgs.zarchan.optimalGainsWeave('assert', false, 'render', false)
%
%   Line-by-line port of Listing 26.2 (zarchan.txt:9466-9477), integrated by
%   the book's own two-pass STEP/FLAG second-order Runge-Kutta (Heun).  The
%   equation is the 4x4 matrix Riccati equation
%
%       dS/dtgo = S*F + F'*S - S*G*B^-1*G'*S,      S(0) = SF = diag(1,0,0,0)
%       C       = B^-1*G'*S,   nc = C1*y + C2*ydot + C3*nT + C4*nTdot
%
%   with the weaving-target model of Fig. 26.7 (zarchan.txt:9419-9440): states
%   [y, ydot, nT, nTdot], nTddot = -W^2*nT, B = GAM, and a PERFECT (zero-lag)
%   flight control system.
%
%   THE LOOP RUNS FORWARD IN TIME-TO-GO.  The usual Riccati sweep is backward
%   from tF; substituting tgo = tF - t turns it into the forward sweep the
%   listing writes, started from the terminal weighting SF.  So the x axis of
%   Figs. 26.8/26.9, labelled "Time (s)", is TIME TO GO -- not time from
%   launch -- and R.tgo is named accordingly.
%
% ===========================================================================
%   UNITS.  THIS LISTING HAS NO LENGTH UNIT IN IT AT ALL; it produces gains.
%     tgo, TF, H   s        W   rad/s        GAM   s^3
%     C1 [1/s^2]   C2 [1/s]   C3 [-]   C4 [s]        N'  [-]
%   GAM is s^3 because J = y^2(tF) + GAM*int(nc^2)dt with y in ft and nc in
%   ft/s^2 (zarchan.txt:9416-9417).  Nothing here needs g = 32.2 and nothing is
%   converted; the gains apply unchanged in metres.
% ===========================================================================
%
%   WHY THIS ONE MATTERS HERE -- AND WHAT IT CANNOT SAY
%
%   It is the book deriving an optimal law against a SINUSOID from first
%   principles, and the answer is plain: the navigation ratio stays at THREE
%   (zarchan.txt:9448) and the sinusoid buys exactly two extra zero-effort-miss
%   terms, C3*nT + C4*nTdot.  Adding a weave to the target model does not touch
%   the PN core -- the same conclusion this project reached by measurement.
%
%   DIRECTION, and it is the trap.  The book's sinusoid is a TARGET maneuver
%   being countered.  Our weave is a MISSILE-side path maneuver used to BURN
%   TIME against a stationary aim point.  Same trigonometry, opposite role:
%   nothing in C3/C4 sizes a time-absorbing S-turn, and the two must not be
%   quoted at each other.
%
%   THE FLIGHT CONTROL SYSTEM HERE IS PERFECT -- stated at zarchan.txt:9384,
%   and it is why the algebra closes at all.  These gains are a LAG-FREE
%   CEILING, not a prediction for an airframe whose measured gamma-tracking
%   gain is 0.063 against a model 1.385.  Listing 26.1 is the neighbouring
%   listing that does carry a time constant.
%
%   One transferable mechanism: GAM is the only thing keeping the gains finite
%   as tgo -> 0.  Every gain is the book's formula times tgo^3/(3GAM + tgo^3)
%   -- exactly, see below.  A PN law without such a term is bounded by clamps
%   instead, which is a different endgame.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION, AND WHAT WAS DONE TO IT
%
%   listing_26_02.m does not parse.  Five repairs, none a judgement call except
%   the fourth, which is a CONTRADICTION INSIDE THE BOOK:
%
%     1  "while ~(T .= (TF-.0001))" -> "while ~(T >= (TF - H))".  "." is this
%        book's OCR of ">".  Shape-forced: T only increases, and with "<=" the
%        negation is false on the first test so the loop never runs at all.
%        The literal .0001 IS this listing's own H, so at the book's settings
%        "TF - H" is the same expression; written as H so a caller who changes
%        H still stops one step short of TF instead of overshooting it.
%     2  "while STEP,=1" -> "while STEP <= 1" (already resolved upstream).  The
%        body sets STEP=2 to leave, which only makes sense against "<=".
%     3  "if S1.=.009999" -> "if S1 >= samp - 0.5*H".  S1 accumulates H once
%        per outer step and is reset on every record, so .009999 means "record
%        every 0.01 s" at H = 1e-4 -- 100 steps.  Written against 'samp'
%        because keeping the literal would silently change meaning at other H;
%        at the listing's own settings the two fire on exactly the same step
%        (S1 = 0.0099 at step 99, 0.0100 at step 100).
%     4  G(2,1).  THE BOOK PRINTS TWO DIFFERENT SIGNS.  The state-space matrix
%        at zarchan.txt:9426-9430 is G = [0; 1; 0; 0]; the listing at
%        zarchan.txt:9466 sets G(2,1) = -1.  THE LISTING IS FOLLOWED, because
%        it is the only one of the two that reproduces the book's own gain
%        formulas.  S itself cannot tell them apart -- only G*G' enters the
%        Riccati equation -- but C = G'*S/GAM flips sign with G, and the
%        listing records C1 = -C(1,1).  With G(2,1) = -1 that gives
%        C1 = +3/tgo^2, matching zarchan.txt:9456; with +1 it gives -3/tgo^2,
%        the wrong sign on every published gain.  The printed +1 belongs to the
%        convention ydd = nT + nc, the listing's -1 to ydd = nT - nc (relative
%        acceleration), which is what Fig. 26.7 and Chapter 8 use.  A wrong
%        choice here is INVISIBLE in S, in the Riccati residual, and in any
%        plot of |C|; only the sign of a printed gain catches it.
%     5  Dropped, not "fixed" into code: the paragraph the extractor pulled
%        into the listing body (it is zarchan.txt:9478-9479, the prose that
%        supplies the agreement rows), and "save datfil.txt output /ascii" -- a
%        typo for -ascii, and nothing is written to disk here in any case.
%
%   Two loop-invariants (GT = G', GAMINV = 1/GAM) are hoisted out of the inner
%   loop: identical values, ~200,000 fewer interpreter round trips.
%
%   COST, AND WHY H IS NOT A COMFORT SETTING.  The listing's own H = 0.0001 s
%   over TF = 10 s is 100,000 outer steps, each doing two full 4x4 passes;
%   expect a second or two.  H is an option, but the scheme's error here obeys
%
%        relative drift  ~  H^2 * tgo / (8*GAM)                          (*)
%
%   which is 0.125% at the listing's own settings and grows LINEARLY IN tgo --
%   it is an accumulated global error, not a start-up transient.  (*) was
%   measured, not guessed: it reproduces to three digits across GAM = 1e-6,
%   1e-5, 1e-4, W = 0.5, 1, 2, H = 5e-5 .. 2e-4 and TF = 5, 10, in an
%   independent re-implementation of this same recursion.  So H = 1e-3 does NOT
%   buy a cheap check -- it would put the residual at 12.5% -- and every
%   tolerance below is built from (*) rather than from a constant somebody
%   liked.  An H coarse enough to make the band exceed 5% is REFUSED, with the
%   arithmetic in the message, instead of quietly passing on a widened band.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Every book value was grepped out of zarchan.txt and the line
%   number is in the .source field of its row.  Chapter 26 publishes very
%   little that is scoreable for this listing: Figs. 26.8/26.9 became image
%   links in the extraction (zarchan.txt:9481-9484), so NO gain value can be
%   read off them.  What the chapter does state plainly is one number, four
%   formulas and an agreement claim, and that is what is scored:
%
%     1    N' = 3.  zarchan.txt:9448, prose: "the effective navigation ratio is
%          three"; the listing hard-codes XNPP = 3. at :9475.  Ours is the
%          listing's own NP = C2*tgo at the last sample.
%     2-5  The four closed forms, zarchan.txt:9456-9462:
%             C1 = 3/tgo^2      C2 = 3/tgo
%             C3 = (3/tgo^2)(1-cos(W*tgo))/W^2
%             C4 = (3/tgo^2)(W*tgo-sin(W*tgo))/W^3
%          scored as the agreement claimed for them at :9478-9479 ("in near
%          perfect agreement") and in the captions of Fig. 26.8 (:9490) and
%          Fig. 26.9 (:9491).  An adjective is not a tolerance, so each is a
%          SHAPE row: max deviation <= max(0.5%, 4x the drift law (*)).
%          tFit IS NOT A FUDGE.  The book's formulas are the GAM -> 0 limit;
%          the listing runs at GAM = 1e-5 and the exact ratio between them is
%          tgo^3/(3GAM + tgo^3).  Below tgo ~ 0.14 s the two curves genuinely
%          differ by more than 1% and the book is not claiming otherwise --
%          that is 1.4% of the width of its own figure.  tFit sits where the
%          ratio is within 0.05%, ten times inside the band, and defaults to
%          1.0 s.  Row 6 REPORTS the knee rather than hiding it.
%          C3 is scored PEAK-NORMALISED and the other three POINTWISE: C3 is
%          the one gain with a genuine ZERO (at W*tgo = 2*pi), and a pointwise
%          relative error there divides a small number by a smaller one.  Left
%          pointwise it reports 3.9% for a curve that is right to 2.5e-4 of its
%          own peak.
%     6    INFO: the GAM knee, predicted (297*GAM)^(1/3) against measured.
%     7-8  BOUND: the listing's own axis boxes, axis([0 10 0 1.5]) for C3 and
%          axis([0 10 0 1]) for C4 (zarchan.txt:9477).  These are the AUTHOR'S
%          PLOT RANGES, not measurements -- but they are in the listing, they
%          are greppable, and a curve outside them would have been clipped in
%          figures the book says agree.  They are also two different KINDS of
%          statement, which is why the C4 row is gated and the C3 row is not:
%            C3 = 3*(1-cos(W*tgo))/(W*tgo)^2 <= 1.5 for EVERY W and tgo, so
%              1.5 is the analytic limit of the curve as tgo -> 0 and the box
%              holds always;
%            C4 peaks at 3/(pi*W) at tgo = pi/W -- 0.9549 at the book's
%              W = 1 rad/s, but 1.910 at W = 0.5, OUTSIDE the box.  So that
%              axis is a statement about W = 1 and the row SKIPS when
%              3/(pi*W) > 1, with the arithmetic in the reason.
%     9    SKIP: any C3/C4 value at a stated tgo.  Emitted as a SKIP rather
%          than absorbed into a widened band.
%
%   STRUCTURAL (self-consistency, NOT book values).  This Riccati equation has
%   an EXACT closed-form solution, derived and verified here, and it is what
%   makes the port sharp at every tgo instead of only at tgo >= tFit:
%
%       S(tgo) = p*z*z',   p = 3GAM/(3GAM + tgo^3),
%       z = [1; tgo; (1-cos(W*tgo))/W^2; (W*tgo-sin(W*tgo))/W^3]
%
%   z satisfies zdot = F'*z by inspection, so Sdot - (S*F + F'*S) = pdot*z*z';
%   and G = -e2 gives S*G*G'*S = p^2*tgo^2*z*z'.  The matrix equation therefore
%   collapses to the scalar pdot = -p^2*tgo^2/GAM, whose solution is the p
%   above.  Hence C_j = 3*tgo*z_j/(3GAM + tgo^3): the book's gains times
%   tgo^3/(3GAM+tgo^3), THE SAME FACTOR FOR ALL FOUR.  Checks 1-5 score that.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  ⚠️ THIS FILE HAS NEVER BEEN RUN IN MATLAB -- the
%   MATLAB session is the caller's, not this author's.  The numbers below come
%   from an INDEPENDENT re-implementation of the same recursion, in another
%   language, at the listing's own settings.  It performs the same double
%   arithmetic in the same order, so the port should land on them to many
%   digits; treat a mismatch as a finding about the port, not about the list.
%
%     1000 samples, tgo = 0.0100 .. 10.0000 s, 100,000 outer steps
%       (the last step lands on the loop guard by a rounding-level margin; a
%        differently-rounded accumulation of T would give 999 / 9.99 s instead,
%        which changes nothing that is scored)
%     N' = C2*tgo at the last sample  3.0012483   (book 3, +0.0416%)
%       -- it OVERSHOOTS.  The exact solution approaches 3 strictly from below;
%       what carries it past is the drift (*), which is why this is not scored
%       as an identity.
%     book rows, max deviation over tgo >= 1 s:
%       C1 1.248e-3   C2 4.161e-4   C3 2.521e-4 (peak-norm)   C4 8.036e-4
%       against a band of 5.0e-3 -- a factor of 4.0 on the worst of them
%     max C3 1.489758 (formula curve 1.499988, limit 1.5); min C3 9.485e-6
%     max C4 0.954803 (formula curve 0.954929 = 3/pi at tgo = pi)
%     GAM knee 0.1437 s predicted, first sample at or above 99% is 0.1500 s
%     structural: asymmetry EXACTLY 0 (the recursion is symmetric in floating
%       point, not merely nearly so), S vs exact 1.325e-3, C vs exact 1.248e-3,
%       ratio spread 1.569e-3, S(1,1) largest step -9.04e-11 (i.e. decreasing)
%
%   Every one of those residuals is the SAME drift (*), which is why they move
%   together.  A residual near a band EDGE is not a pass to be accepted: (*)
%   says where it should land, and a factor of 4 is the whole margin.  Do not
%   widen a band to make a row green -- raise H's guard instead, or find out
%   why the drift changed.
%
%   OUTPUTS
%     R.tgo        time-to-go grid [s] (the listing's ArrayT)
%     R.C          n-by-4 numeric gains [C1 C2 C3 C4]
%     R.Cth        n-by-4 the book's closed forms (the listing's C1TH..C4TH)
%     R.Cex        n-by-4 the exact GAM-regularised gains derived above
%     R.S, R.Sex   4-by-4-by-n recorded Riccati solution, and its exact form
%     R.NP         the listing's NP = C2*tgo (effective navigation ratio)
%     R.units R.p R.bench R.selfcheck R.pass R.verdict R.summary
%   R.pass is TRUE only if a row was scored AND every scored row passed AND
%   every structural check held.  'benchmark' false leaves it FALSE meaning NOT
%   CHECKED.  'assert' chooses the reaction to a bad verdict and nothing else:
%   true errors, false WARNS and still returns R complete, so a red row can be
%   diagnosed rather than merely reported.
%
%   See also MGS.ZARCHAN.WEAVEADJOINT (Listing 20.2, the miss this law exists
%   to remove), MGS.ZARCHAN.WEAVECOMPENSATED (Listing 20.4, the same law flown
%   through a real lag), MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 26.2 line 1, verbatim (zarchan.txt:9466-9468).
    opts.W    (1,1) double {mustBeFinite, mustBePositive} = 1.0     % rad/s
    opts.GAM  (1,1) double {mustBeFinite, mustBePositive} = 1e-5    % s^3
    opts.TF   (1,1) double {mustBeFinite, mustBePositive} = 10.0    % s of tgo
    opts.H    (1,1) double {mustBeFinite, mustBePositive} = 1e-4    % s
    % [] means the listing's own record interval, 0.01 s (its ".009999").
    opts.samp double {mustBeReal, mustBeFinite} = []
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so an `if nargin == 0` here is a mode that never fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

W = opts.W;  GAM = opts.GAM;  TF = opts.TF;  H = opts.H;

samp = opts.samp;
if isempty(samp)
    samp = 0.01;                     % the listing's ".009999" threshold
end
if ~isscalar(samp) || samp <= 0
    error('mgs:zarchan:optimalGainsWeave:samp', ...
        'samp must be a positive scalar or [] (the listing''s 0.01 s); got %s.', ...
        mat2str(samp));
end
if H > samp
    error('mgs:zarchan:optimalGainsWeave:step', ...
        ['H = %g s is larger than the record interval samp = %g s, so the ' ...
         'returned curve would be sampled at the step rate while claiming ' ...
         'the coarser interval in its own time vector.'], H, samp);
end
if TF <= 2*samp
    error('mgs:zarchan:optimalGainsWeave:short', ...
        'TF = %g s gives fewer than two records at samp = %g s.', TF, samp);
end

% THE ERROR LAW (*) FROM THE HEADER, and every band below is built from it.
% Relative drift of the gains at the end of the sweep ~ H^2*tgo/(8*GAM); it
% accumulates LINEARLY in tgo, so it is a property of the whole run and not of
% the first sample.  1.25e-3 at the listing's own settings.
driftPred = H^2 * TF / (8*GAM);
if 4*driftPred > 0.05
    error('mgs:zarchan:optimalGainsWeave:coarse', ...
        ['H = %g s at GAM = %g s^3, TF = %g s puts the predicted ' ...
         'discretization drift at %.3g (H^2*TF/(8*GAM)), so every band here ' ...
         'would have to open past 5%% and the rows would pass on anything.  ' ...
         'The listing''s own H is 1e-4, which gives 1.25e-3.  Refused rather ' ...
         'than run on a widened band.'], H, GAM, TF, driftPred);
end

% =========================================================================
%   LISTING 26.2.  Transcribed from zarchan.txt:9466-9477.
% =========================================================================
[tgo, C, S, nStep] = ricattiRun(W, GAM, TF, H, samp);
if numel(tgo) < 2
    error('mgs:zarchan:optimalGainsWeave:empty', ...
        ['the sweep recorded %d sample(s) at TF = %g s, samp = %g s.  At ' ...
         'least two are needed for any curve comparison to mean anything.'], ...
        numel(tgo), TF, samp);
end

Cth = bookGains(tgo, W);                % the listing's C1TH..C4TH, GAM -> 0
[Cex, Sex] = exactGains(tgo, W, GAM);   % the exact solution derived in the header
NP = C(:,2).*tgo;                       % the listing's NP = C2*T

% The window where the book's GAM -> 0 formulas are entitled to be believed:
% the exact ratio is tgo^3/(3GAM+tgo^3), so this is where it is within 0.05%.
tFit = max(1.0, (3*GAM/5e-4)^(1/3));

R = struct();
R.tgo = tgo;  R.C = C;  R.Cth = Cth;  R.Cex = Cex;
R.S = S;  R.Sex = Sex;  R.NP = NP;
R.units = ['tgo, TF, H in s; W in rad/s; GAM in s^3; gains C1 [1/s^2] ' ...
           'C2 [1/s] C3 [-] C4 [s]; N'' dimensionless.  Listing 26.2 ' ...
           'carries no length unit at all.'];
R.p = struct('W',W, 'GAM',GAM, 'TF',TF, 'H',H, 'samp',samp, 'tFit',tFit, ...
             'drift_pred',driftPred, 'n_samples',numel(tgo), ...
             'n_steps',nStep, 'units',R.units);

R.selfcheck = structuralChecks(C, Cth, Cex, S, Sex, driftPred);

if opts.benchmark
    R.bench = benchmarks(tgo, C, Cth, W, GAM, TF, H, samp, nStep, NP, tFit, driftPred);
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

% ---- adjudication.  The verdict is FORMED either way; only the reaction to it
%      depends on 'assert'.  benchSuite runs every port with assert off so that
%      one red row cannot hide the board.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = 'not one book value could be evaluated at these options.';
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port ' ...
        'disagrees with ITSELF, which is a wiring fault, not a tolerance ' ...
        'question.  First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf('%d of %d scored rows are outside tolerance (first: %s).', ...
        numel(bad), sum(ev), R.bench(bad(1)).what);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:optimalGainsWeave:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN, complete, so the caller can see WHICH side moved.
        warning('mgs:zarchan:optimalGainsWeave:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function [tOut, Cout, Sout, nStep] = ricattiRun(W, GAM, TF, H, samp)
%RICATTIRUN  Listing 26.2, zarchan.txt:9466-9475.
%
%   Every constant this needs is an ARGUMENT: MATLAB local functions cannot see
%   the main function's workspace, and a previous wave in this package shipped
%   a port that read one from the caller and threw on its first line.

% ---- the listing's line 1, verbatim except for the plotting -----------------
F = zeros(4,4);  S = zeros(4,4);  G = zeros(4,1);
G(1,1) = 0;  G(2,1) = -1.;  G(3,1) = 0;  G(4,1) = 0.;   % -1: see header item 4
F(1,2) = 1;  F(2,3) = 1;  F(3,4) = 1.;  F(4,3) = -W*W;
S(1,1) = 1;
T = 0;  S1 = 0;

% Loop-invariant, hoisted: identical values, ~200k fewer interpreter calls.
GT = G';  GAMINV = 1./GAM;

% Defined before the loop so the FLAG==1 branch cannot read a leftover.
SD = zeros(4,4);  C = zeros(1,4);

nmax = ceil(TF/samp) + 8;
tOut = zeros(nmax,1);  Cout = zeros(nmax,4);  Sout = zeros(4,4,nmax);
n = 0;  nStep = 0;

while ~(T >= (TF - H))
    S1 = S1 + H;
    SOLD = S;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  Pass 1 evaluates at the current S;
    % pass 2 takes a full Euler step, advances T by H, and re-evaluates there.
    % T advances INSIDE this loop, exactly once, which is what makes the second
    % evaluation happen at T+H.  Move that line out and the scheme quietly
    % degrades to first order while still running smoothly.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            HSD = H*SD;  S = S + HSD;
            T = T + H;
        end
        SF   = S*F;       GTS   = GT*S;      C   = GAMINV*GTS;
        SFT  = SF';       CT    = C';        CTC = CT*C;
        CTBC = GAM*CTC;   SFSFT = SF + SFT;  SD  = SFSFT - CTBC;
        FLAG = 1;
    end
    % The listing writes FLAG = 0 here as well.  Dropped: the outer loop
    % re-initialises FLAG on its next pass, so the assignment cannot be read.
    % Kept as a comment so nobody re-adds it thinking a state machine was lost.

    % Heun corrector, in the listing's own algebra.
    H2 = .5*H;  HSDP = H2*SD;  SS = SOLD + S;  SSP = .5*SS;  S = SSP + HSDP;
    nStep = nStep + 1;

    % C recorded here is the SECOND-pass value, i.e. evaluated at the
    % EULER-PREDICTED S rather than at the corrected S.  That is the listing's
    % own order and it is kept: both are at the same time T, and they differ by
    % 0.5*H*(SD1 - SD2) = O(H^2), so it costs nothing at second order.
    if S1 >= samp - 0.5*H              % the listing's "if S1 >= .009999"
        S1 = 0;
        n = n + 1;
        tOut(n)     = T;
        Cout(n,:)   = [-C(1,1), -C(1,2), -C(1,3), -C(1,4)];
        Sout(:,:,n) = S;
    end
end

tOut = tOut(1:n);  Cout = Cout(1:n,:);  Sout = Sout(:,:,1:n);
end


% =============================================================================
function Cth = bookGains(t, W)
%BOOKGAINS  zarchan.txt:9456-9462, coded as the listing codes them at :9475.
%   The GAM -> 0 limit: these diverge as tgo -> 0 and the numerical gains do
%   not.  N' is 3 by fiat here -- the listing's own XNPP = 3.
XNPP = 3.;
Cth = [ XNPP./(t.*t), ...
        XNPP./t, ...
        XNPP*((1.-cos(W*t))/W^2)./t.^2, ...
        XNPP*((W*t-sin(W*t))/W^3)./t.^2 ];
end


% =============================================================================
function [Cex, Sex] = exactGains(t, W, GAM)
%EXACTGAINS  The EXACT solution of the transcribed Riccati equation, derived
%   and verified analytically in the header.  NOT A BOOK VALUE:
%
%       S = p*z*z',  p = 3GAM/(3GAM+tgo^3),  C_j = 3*tgo*z_j/(3GAM+tgo^3)
%
%   so every gain is the book's formula times tgo^3/(3GAM+tgo^3).
z   = [ ones(numel(t),1), t, (1-cos(W*t))/W^2, (W*t-sin(W*t))/W^3 ];
den = 3*GAM + t.^3;
Cex = (3*t./den) .* z;

p   = 3*GAM./den;
Sex = zeros(4,4,numel(t));
for k = 1:numel(t)
    Sex(:,:,k) = p(k) * (z(k,:)' * z(k,:));
end
end


% =============================================================================
function K = structuralChecks(C, Cth, Cex, S, Sex, driftPred)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.  They are the weaker
%   claim that it agrees with itself -- which is what catches wiring faults.
K = struct();
names = {};  oks = [];  detail = {};

% ALL of these residuals are the same accumulated drift (*) from the header, so
% they are all banded off it: 4x the prediction, floored at 0.5%.  Measured at
% the listing's own settings: S 1.325e-3, C 1.248e-3 against a 5e-3 band.
tolS = max(0.005, 4*driftPred);

% 1  S stays symmetric.  Sdot = S*F + (S*F)' - GAM*C'*C is symmetric by
%    construction, so any asymmetry is a transposition error in the port.
asym = 0;
for k = 1:size(S,3)
    a = max(max(abs(S(:,:,k) - S(:,:,k).')));
    asym = max(asym, a/max(max(max(abs(S(:,:,k)))), realmin));
end
K.sym = asym;
names{end+1}  = 'S stays symmetric';
oks(end+1)    = asym <= 1e-10;
detail{end+1} = sprintf('worst |S-S''|/|S| = %.3e', asym);

% 2  S against the exact rank-1 solution p*z*z'.  The sharpest check in the
%    file: it covers every tgo, including the GAM knee where the book's own
%    formulas do not apply and rows 2-5 therefore cannot look.
rS = 0;
for k = 1:size(S,3)
    rS = max(rS, norm(S(:,:,k) - Sex(:,:,k), 'fro') / ...
                 max(norm(Sex(:,:,k), 'fro'), realmin));
end
K.S_resid = rS;
names{end+1}  = 'S == p*z*z'' (exact Riccati solution)';
oks(end+1)    = rS <= tolS;
detail{end+1} = sprintf('worst %.3e (band %.3e)', rS, tolS);

% 3  The four gains against that same exact solution.  Column 3 is scored
%    PEAK-NORMALISED (floor 1) because C3 has a genuine zero at W*tgo = 2*pi;
%    pointwise it reports 3.9% for a curve right to 2.5e-4 of its own peak.
rC = relDev(C, Cex, [0 0 1 0]);
K.C_resid = rC;
names{end+1}  = 'C == 3*tgo*z/(3GAM+tgo^3), all four gains';
oks(end+1)    = rC <= tolS;
detail{end+1} = sprintf('worst %.3e (band %.3e)', rC, tolS);

% 4  The ratio numeric/formula is COMMON to the gains -- they are four entries
%    of ONE row of S, scaled by a z that is exact in the book's formula.  This
%    one uses only the BOOK's formulas and no derivation of this file's, so it
%    still stands if the exact solution above is wrong; it is what catches a
%    mis-indexed or swapped gain column, which moves the ratio for two gains
%    and not for four.
%    ⚠️ C3 IS EXCLUDED, not masked.  Its zero at W*tgo = 2*pi makes the ratio
%    C3/C3TH meaningless nearby (it reads 26 there), and no threshold on |C3TH|
%    fixes that without also deleting the whole far field, where C3TH decays as
%    1/tgo^2.  C3 is covered by check 3 and by its own book row.
sub = C(:,[1 2 4])./Cth(:,[1 2 4]);
K.ratio_spread = max(max(sub,[],2) - min(sub,[],2));
tolR = max(0.01, 8*driftPred);
names{end+1}  = 'numeric/formula ratio identical for C1, C2, C4';
oks(end+1)    = K.ratio_spread <= tolR;
detail{end+1} = sprintf('worst spread %.3e (band %.3e)', K.ratio_spread, tolR);

% 5  S(1,1) = 3GAM/(3GAM+tgo^3) is strictly decreasing in tgo.  A sign error on
%    SD leaves a smooth curve of the right order of magnitude and fails here.
K.S11_max_rise = max(diff(squeeze(S(1,1,:))));
names{end+1}  = 'S(1,1) non-increasing in tgo';
oks(end+1)    = K.S11_max_rise <= 1e-14;
detail{end+1} = sprintf('largest rise %+.3e', K.S11_max_rise);

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
function r = relDev(a, b, floorFrac)
%RELDEV  Max relative deviation, column by column, with each column's
%   denominator floored at floorFrac of that column's own peak.  floorFrac may
%   be a scalar or one value per column.
%
%   floorFrac = 0 is POINTWISE relative, floorFrac = 1 is PEAK-NORMALISED, and
%   the difference is not cosmetic.  C1 spans four decades over this sweep, so
%   peak-normalising it would leave the whole far field untested; C3 passes
%   through ZERO at W*tgo = 2*pi, so scoring IT pointwise divides a small
%   number by a smaller one and reports 3.9% for a curve that is right to
%   2.5e-4 of its own peak.  Hence [0 0 1 0] at every call site here.
den = abs(b);
pk  = max(den, [], 1);
if any(~(pk > 0))
    r = Inf;                 % refuse to report "perfect" against a zero curve
    return
end
den = max(den, floorFrac.*pk);
r = max(max(abs(a - b)./den));
end


% =============================================================================
function B = benchmarks(t, C, Cth, W, GAM, TF, H, samp, nStep, NP, tFit, driftPred)
%BENCHMARKS  Computed vs published, with the PROVENANCE of every band.
%
%   *** EVERY BOOK VALUE HERE WAS GREPPED OUT OF zarchan.txt AND THE LINE
%   NUMBER IS IN THE .source FIELD. ***  Nothing was read off a figure: the
%   extraction turned every figure in this chapter into an image link, and
%   earlier waves in this package were handed "benchmarks" that were exactly
%   such phantom readings.
B    = emptyBench();
win  = t >= tFit;
% 4x the drift law (*), floored at 0.5%.  At the listing's own settings the
% floor wins and the worst measured row (C1, 1.248e-3) sits 4.0x inside it.
tolC = max(0.005, 4*driftPred);

% ---- 1.  N' = 3 ------------------------------------------------------------
% Gate: NP -> 3 only once tgo^3 >> 3*GAM, the same GAM knee as everywhere else
% in this file.  1e4 is four decades inside it.
gateNP = TF^3 >= 1e4*3*GAM;
B = addVal(B, 'effective navigation ratio N'' = C2*tgo', 3.0, NP(end), ...
    'formula', 'p.660 text (zarchan.txt:9448); listing XNPP=3. (:9475)', '-', ...
    sprintf(['the listing''s own NP, at the last sample tgo = %.4f s.  The ' ...
        'EXACT value is 3*tgo^3/(3GAM+tgo^3) = %.9f, which approaches 3 ' ...
        'strictly from below; the run OVERSHOOTS it because the drift (*) is ' ...
        'larger than the GAM deficit by four orders of magnitude here, so ' ...
        'this row measures the integrator and not the model'], ...
        t(end), 3*t(end)^3/(3*GAM + t(end)^3)), ...
    gateNP, sprintf(['TF^3 = %.4g is not four decades above 3*GAM = %.4g, so ' ...
        'the GAM regularisation still holds N'' away from 3 at every sample'], ...
        TF^3, 3*GAM));

% ---- 2-5.  the four closed-form gains --------------------------------------
gainName = {'C1 == 3/tgo^2', 'C2 == 3/tgo', ...
            'C3 == (3/tgo^2)(1-cos(W tgo))/W^2', ...
            'C4 == (3/tgo^2)(W tgo - sin(W tgo))/W^3'};
gainSrc  = {'p.660 closed form (zarchan.txt:9456); agreement claim :9478-9479', ...
            'p.660 closed form (zarchan.txt:9457); agreement claim :9478-9479', ...
            'p.660 closed form (zarchan.txt:9458-9460); Fig. 26.8 caption :9490', ...
            'p.660 closed form (zarchan.txt:9461-9462); Fig. 26.9 caption :9491'};
% POINTWISE (floor 0) for C1, C2 and C4, which are strictly positive here;
% PEAK-NORMALISED (floor 1) for C3, which is ZERO at W*tgo = 2*pi -- see
% relDev.  This is a real choice and it is visible in the row's ours_str.
gainFloor = [0 0 1 0];
gainNorm  = {'pointwise', 'pointwise', 'peak-norm', 'pointwise'};
enough    = sum(win) >= 3;
for j = 1:4
    if enough
        d = relDev(C(win,j), Cth(win,j), gainFloor(j));
    else
        d = NaN;
    end
    B = addShape(B, gainName{j}, ...
        sprintf('"in near perfect agreement" -> <= %.2f%%', 100*tolC), ...
        sprintf('max dev %.3e (%s) over tgo >= %.3g s, %d samples', ...
            d, gainNorm{j}, tFit, sum(win)), ...
        d <= tolC, gainSrc{j}, ...
        ['the book''s claim is an ADJECTIVE, so it is scored as a bound.  The ' ...
         'window starts at tFit because the book''s formula is the GAM -> 0 ' ...
         'limit and the exact ratio between them is tgo^3/(3GAM+tgo^3); the ' ...
         'GAM knee row reports where that matters'], ...
        enough, sprintf(['only %d sample(s) at tgo >= %.3g s -- an agreement ' ...
            'claim needs a curve, not a point'], sum(win), tFit));
end

% ---- 6.  INFO: where the two curves actually part company ------------------
% Derived, not published: the deviation is 3GAM/(3GAM+tgo^3), which is 1% at
% tgo = (297*GAM)^(1/3).
knee = (297*GAM)^(1/3);
rat  = C(:,1)./Cth(:,1);
im   = find(rat >= 0.99, 1);
if isempty(im), kneeMeas = NaN; else, kneeMeas = t(im); end
B = addInfo(B, 'GAM knee: tgo where numeric/formula reaches 99%', ...
    sprintf('(297*GAM)^(1/3) = %.4f s', knee), ...
    sprintf('first sample at %.4f s (%.2f%% of the figure''s 0-10 s axis)', ...
        kneeMeas, 10*kneeMeas), ...
    'GAM = .00001 (zarchan.txt:9464, :9466)', ...
    ['not a book number -- it is WHY "near perfect agreement" survives a ' ...
     'figure drawn from tgo = 0: the disagreement lives in the first ~1.4% ' ...
     'of the x axis and is invisible at that scale']);

% ---- 7-8.  BOUND: the listing's own axis boxes -----------------------------
% ⚠️ The two boxes are DIFFERENT KINDS of statement, so only one is gated:
%   C3 = 3*(1-cos(W*tgo))/(W*tgo)^2 <= 1.5 for every W and tgo -- 1.5 is the
%        curve's analytic limit as tgo -> 0, so the box holds always;
%   C4 peaks at 3/(pi*W) at tgo = pi/W: 0.9549 at the book's W = 1, but 1.910
%        at W = 0.5, outside the box.  That axis is a statement about W = 1.
lim   = [1.5 1.0];
gate4 = 3/(pi*W) <= 1.0;
for j = 3:4
    cn = C(:,j);
    ok = max(cn) <= lim(j-2) && min(cn) >= -1e-6*max(abs(cn));
    B = addBound(B, sprintf('C%d inside the listing''s axis box [0 %g]', j, lim(j-2)), ...
        sprintf('axis([0 10 0 %g])', lim(j-2)), ...
        sprintf('C%d in [%.5e, %.5f]; formula curve peaks at %.5f', ...
            j, min(cn), max(cn), max(Cth(:,j))), ok, ...
        'listing plot block (zarchan.txt:9477)', '-', ...
        ['an axis is the AUTHOR''S PLOT RANGE, not a measurement -- but it is ' ...
         'in the listing and a curve outside it would be clipped in a figure ' ...
         'the book says agrees.  Scored on the SIMULATION curve only: the ' ...
         'formula curve reaches its own limit (1.5 for C3) only as tgo -> 0, ' ...
         'where at a very fine samp its margin shrinks into the cancellation ' ...
         'error of (1-cos) -- a statement about arithmetic, not about the port'], ...
        j == 3 || gate4, ...
        sprintf(['C4 peaks at 3/(pi*W) = %.4f, above the axis top of 1, so ' ...
            'axis([0 10 0 1]) is a statement about the book''s W = 1 rad/s ' ...
            'and not about W = %g'], 3/(pi*W), W));
end

% ---- 9.  SKIP: any gain VALUE at a stated tgo ------------------------------
B = addSkip(B, 'C3 or C4 at a stated tgo (Figs. 26.8, 26.9)', ...
    'no number printed', ...
    ['Figs. 26.8/26.9 are the only place Ch.26 shows a weave-gain value, and ' ...
     'both became image links in the extraction (zarchan.txt:9481-9484).  ' ...
     'Their captions (:9490, :9491) and the prose (:9478-9479) claim ' ...
     'AGREEMENT and nothing else, which is what rows 2-5 score.  Recorded as ' ...
     'a SKIP rather than absorbed by widening a band'], ...
    'Figs. 26.8/26.9 (zarchan.txt:9490-9491)', '-');

% ---- INFO: the run itself, so the board can see whether it was the book's ---
B = addInfo(B, 'nominal case of Listing 26.2', ...
    'W=1 r/s, TF=10 s, GAM=.00001, H=.0001 s', ...
    sprintf(['W=%g, TF=%g, GAM=%g, H=%g, samp=%g -> %d samples, %d steps; ' ...
        'predicted drift H^2*TF/(8GAM) = %.3e, bands %.3e / %.3e'], ...
        W, TF, GAM, H, samp, numel(t), nStep, driftPred, tolC, max(0.01, 8*driftPred)), ...
    'p.661 text (zarchan.txt:9464)', ...
    ['reported so that an off-book run cannot be mistaken for the book''s ' ...
     'own, and so the bands can be read against the law that set them']);
end


% =============================================================================
%   ROW CONSTRUCTORS.  ONE each, so the field ORDER cannot drift: a struct
%   array whose fields reordered makes [B.ok] mean something else, and it fails
%   with "Subscripted assignment between dissimilar structures", which reads
%   like a typo rather than a schema break.
% =============================================================================
function B = emptyBench()
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, source, units, detail, gate, whyGate)
% GATE/WHYGATE are optional: when gate is false the row is emitted as a SKIP
% carrying whyGate, so the book value is typed in exactly ONE place.
%   nargin is meaningful HERE.  The warning in this package is about the MAIN
%   function, whose `arguments` block declares only name-value options and
%   therefore always reports nargin = 0; a local with positional arguments is
%   ordinary MATLAB.
if nargin >= 9 && ~gate
    B = addSkip(B, what, sprintf('%.4g', book), whyGate, source, units);
    return
end
i = numel(B) + 1;
tol = bandTol(band);
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.4g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.8g', ours);
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

function B = addShape(B, what, bookStr, oursStr, ok, source, detail, gate, whyGate)
if nargin >= 8 && ~gate
    B = addSkip(B, what, bookStr, whyGate, source, '-');
    return
end
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'shape';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'shape';  B(i).tol = NaN;
B(i).ok   = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = '-';  B(i).detail = detail;
end

function B = addBound(B, what, bookStr, oursStr, ok, source, units, detail, gate, whyGate)
if nargin >= 9 && ~gate
    B = addSkip(B, what, bookStr, whyGate, source, units);
    return
end
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
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'skip';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = '--';
B(i).pct  = NaN;   B(i).band = 'skip';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end

function t = bandTol(band)
%BANDTOL  Bands encode the PROVENANCE of the book value.  Inventing one
%   silently sets an arbitrary pass criterion, so an unknown band ERRORS.
switch lower(band)
case 'exact',   t = 1e-12;  % an identity, or a literal echoed from the listing
case 'formula', t = 0.005;  % ADDED BY THIS PORT.  The book value is an EXACT
                            % constant of a closed form (N' = 3), not a printed
                            % measurement, so the only error in our side is
                            % this integrator's -- predicted at ~1e-5 relative.
                            % 'text' (2%) would be four hundred times looser
                            % than the quantity deserves.  Used on ONE row.
case 'text',    t = 0.02;   % printed in prose to 2-3 significant figures
case 'approx',  t = 0.05;   % the book says "approximately"
case 'graph',   t = 0.15;   % the author read it off a figure's ordinate
otherwise
    error('mgs:zarchan:optimalGainsWeave:band', ...
        ['unknown tolerance band "%s".  Bands encode the PROVENANCE of the ' ...
         'book value; inventing one silently sets an arbitrary pass ' ...
         'criterion.'], band);
end
end


% =============================================================================
function s = render(R, ev)
L = {};
L{end+1} = 'mgs.zarchan.optimalGainsWeave -- Listing 26.2, optimal gains vs a WEAVING target';
L{end+1} = sprintf('  %s', R.units);
L{end+1} = sprintf(['  W=%g rad/s  GAM=%g s^3  TF=%g s  H=%g s  samp=%g s   ' ...
    '(predicted drift H^2*TF/(8GAM) = %.3e)'], ...
    R.p.W, R.p.GAM, R.p.TF, R.p.H, R.p.samp, R.p.drift_pred);
L{end+1} = sprintf(['  %d outer steps -> %d samples, tgo = %.4f .. %.4f s ' ...
    '(TIME TO GO, not time from launch)'], R.p.n_steps, R.p.n_samples, ...
    R.tgo(1), R.tgo(end));
L{end+1} = sprintf('  N'' = C2*tgo : %.7f at tgo = %.3f s   (book 3)', ...
    R.NP(end), R.tgo(end));
L{end+1} = sprintf('  C1 %.5g..%.5g   C2 %.5g..%.5g   C3 max %.5f   C4 max %.5f', ...
    R.C(1,1), R.C(end,1), R.C(1,2), R.C(end,2), max(R.C(:,3)), max(R.C(:,4)));
L{end+1} = '  --- benchmarks ---';
for i = 1:numel(R.bench)
    b = R.bench(i);
    switch lower(b.kind)
    case 'info', v = 'INFO';
    case 'skip', v = 'SKIP';
    otherwise
        if b.skipped, v = 'SKIP'; elseif b.ok, v = 'PASS'; else, v = 'FAIL'; end
    end
    if strcmpi(b.kind, 'value') && ~b.skipped
        L{end+1} = sprintf('  [%s] %-44s book %-8s ours %-12s %+9.2e%%  (%s)', ...
            v, b.what, b.book_str, b.ours_str, b.pct, b.band); %#ok<AGROW>
    else
        L{end+1} = sprintf('  [%s] %-44s %s | %s', ...
            v, b.what, b.book_str, b.ours_str); %#ok<AGROW>
    end
end
L{end+1} = '  --- structural (self-consistency, NOT book values) ---';
for i = 1:numel(R.selfcheck.names)
    if R.selfcheck.oks(i), v = 'holds'; else, v = 'VIOLATED'; end
    L{end+1} = sprintf('  [%-8s] %-42s %s', v, R.selfcheck.names{i}, ...
        R.selfcheck.detail{i}); %#ok<AGROW>
end
if any(ev)
    L{end+1} = sprintf('  scored %d row(s); pass = %d', sum(ev), R.pass);
else
    L{end+1} = '  NOTHING WAS SCORED -- R.pass = false means NOT CHECKED.';
end
s = strjoin(L, newline);
end


% =============================================================================
function drawCurves(R)
%DRAWCURVES  Figs. 26.8 and 26.9, with the listing's own axis boxes.
for j = 3:4
    figure;
    plot(R.tgo, R.C(:,j), R.tgo, R.Cth(:,j));
    grid on;
    xlabel('Time to go (s)');       % the listing says "Time (s)"; it IS tgo
    ylabel(sprintf('C%d', j));
    if j == 3, axis([0 10 0 1.5]); else, axis([0 10 0 1]); end
    legend('Riccati', 'Formula');
end
end
