function R = stepDisplacement(opts)
%MGS.ZARCHAN.STEPDISPLACEMENT  Zarchan Listing 19.1 -- two-dimensional
%   NONLINEAR engagement with a STEP IN TARGET DISPLACEMENT, zero-lag PN.
%
%   R = mgs.zarchan.stepDisplacement()
%   R = mgs.zarchan.stepDisplacement('THOM', 0.5)        resolve later
%   R = mgs.zarchan.stepDisplacement('DISPLACE', 400, 'assert', false)
%   R = mgs.zarchan.stepDisplacement('scaling', false)   skip the 2 extra runs
%
%   Line-by-line port of Listing 19.1 (zarchan.txt:5565-5579) on the book's own
%   Heun scheme.  Two aircraft fly in formation at 1200 ft and 800 ft; the
%   interceptor homes on their power centroid at 1000 ft until, THOM seconds
%   before intercept, the seeker resolves aircraft 1 and the target APPEARS to
%   jump 200 ft (zarchan.txt:5554).
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, rad, g = 32.2 ft/s^2.  NOT SI.
%   The rest of mgs is metres and nothing here is converted, so that every
%   number printed lies straight against a published one.  CONVERT AT THE CALL
%   SITE and say so there.
% ===========================================================================
%
%   WHY THIS ONE MATTERS HERE, AND HOW FAR THE ANALOGY REACHES
%
%   Our dominant failure predictor is |eps0|, the LATERAL ERROR AT HANDOVER.
%   Fig. 19.4 puts a step in target displacement into the linearised loop as
%   "an initial condition on the integrator with output y ... y = yT - yM and
%   there is no initial yM" (zarchan.txt:5588).  |eps0| is an initial condition
%   on the SAME integrator, signed on the yM side.  At the LINEAR level they
%   are one disturbance, so this chapter's sizing law is ours:
%
%        nc(t) = N'*(1 - t/tF)^(N'-2) * yTIC / tF^2      (zarchan.txt:5622)
%        nc(0) = N'*yTIC/tF^2                            peak, AT handover
%
%   THAT LICENSES THREE THINGS AND ONLY THESE.  The demand is largest AT
%   handover and falls monotonically after (:5630), so a lateral-authority
%   budget is sized at the handover instant and nowhere later.  It is LINEAR in
%   the offset and INVERSE-SQUARE in the homing time left (:5628) -- the shape
%   of our own cliff.  And tF = 2*yTIC/(Vc*BW) (:5652) is how a sensor spec
%   turns into a homing-time budget.
%
%   *** WHAT IT DOES NOT LICENSE. ***  ZERO LAG, no acceleration limit, no
%   gravity, drag, thrust, LOS floor or nz clamp.  The miss here is EXACTLY
%   ZERO by construction and the book says so (:5624), so this file can never
%   predict a miss distance for an aircraft and a number out of it must never
%   be quoted as one.  Our |eps0| sorties do not fail because the demand is
%   large; they fail because the AUTHORITY to meet it is absent (measured
%   gamma-tracking gain 0.063 against a model 1.385, ~8 s of it, saturating on
%   26 of 32 sorties).  Lag is Listing 19.2's subject and beyond.  Read this as
%   a SIZING criterion and as the clean-loop denominator -- never as a
%   prediction.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION: TWO BARE OCR OPERATORS, AND ONE TRAP OF OUR OWN MAKING
%
%   listing_19_01.m does not parse.  Its header claims no unresolved
%   comparisons; two bare '.' operators survive anyway.
%
%   1  "if(TGO..3) H=.00005"  ->  "if TGO < 0.3".  '.' is the OCR of BOTH '<'
%      and '>' in this book, so shape cannot settle it and neither reading
%      crashes.  *** THE BOOK CONTRADICTS ITSELF HERE. ***  At :5571 the
%      character reads '>'; at :5669 the same statement in Listing 19.2 --
%      introduced as 19.1 "modified", with the CHANGED statements in bold, and
%      this is not one of them -- is "if(TGO,.3)", where ',' is unambiguously
%      '<'.  Chosen '<', on :5669.  Tell: H is set to .005 before the loop,
%      which is dead code under '>' (overwritten on iteration 1, never
%      restored).  LOW STAKES, and stated rather than assumed -- under '>' the
%      run is simply H = 5e-5 throughout, ~100,000 steps instead of 6,902, same
%      switch instant, same answers on a finer endgame.  'TGO_FINE' is exposed
%      so the rejected reading can be run.
%   2  "if S..049999"  ->  "if S > 0.049999" (strict, no '='), the record
%      interval.  Here '.' is '>': the idiom keeps its '=' in a dozen other
%      listings ("if S .=.000999" at :264, :1335; "if S.=.0999" at :491, :763,
%      :2468).  S resets to 0 at every record, so '<' would record every step
%      and leave the constant 0.05 meaning nothing at all.
%
%   3  NOT the book's fault -- ours.  The listing records only every 'samp'
%      (0.05 s) and THE PEAK IS NOT ON THAT GRID: the jump fires at the top of
%      a step whose record has just been taken.  ArrayXNCG's largest value is
%      567.88 ft/s^2 = 17.64 g, so *** Fig. 19.3 CANNOT SHOW 18.6 g AND DOES
%      NOT ***; the book's "nearly 20 g" (:5581) is the formula's 600 read back
%      onto the figure.  So this port also samples XNC at the resolution
%      instant -- an ADDITION, changing no integrated state.  AND THE OBVIOUS
%      WAY TO DO THAT IS WRONG: "max|XNC| over every evaluation" returns 2288.8
%      ft/s^2 (71 g), FOUR TIMES the book, and it is not a bug to be fixed --
%      it lands on the last step at RTM = 0.19 ft, where the LOS rate is
%      genuinely unbounded because one step straddles closest approach.
%      Smooth, plausible, no symptom.  Hence THREE peaks kept apart and only
%      the first scored: nc_res (at resolution, which is where :5657 says ncMAX
%      is), nc_max_win (RTM above 20 endgame steps of range), and nc_max_all
%      (disclosed as an INFO row, never scored).
%
%   Also deliberate: unlike 19.2, 19.1 does NOT recompute RTM2/XLAM after the
%   jump.  It does not need to -- the derivative block recomputes both on its
%   next pass; 19.2 needs it only to seed the seeker dish angle D = XLAM.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Every book value was grepped from zarchan.txt and the line
%   number is in that row's .source.  NOTHING was read off a figure (extraction
%   turned every figure into an image link); prose CLAIMS about figures are
%   greppable and are scored as shape rows.  Each band's provenance is in its
%   own row's .detail -- read that before touching one.
%
%     1  Vc(0) = 4000 ft/s                             :5654, :5661
%     2  ncMAX = 3*yTIC/tF^2 = 600 ft/s^2 = 18.6 g     :5658-5660, at :5657's
%        instant.  Band 2%; expect -0.374%, which is exactly the exact-geometry
%        factor (1+(yTIC/(Vc*tF))^2)^-1.5.
%     3  miss ~ 0 ("hits aircraft 1" :5563, "y(tF) is always zero!" :5624).
%        THE ONLY ROW THAT SURVIVES OFF-BOOK PARAMETERS.
%     4  missile ends at aircraft 1's altitude, 1200 ft   :5551, :5554.  Catches
%        a DISPLACE sign flip, which row 3 cannot.
%     5  nc(t) vs the printed closed form              :5622-5623, claim :5629
%     6  y(t)  vs the printed closed form              :5619-5620
%     7  nc peaks AT resolution (:5657), then decreases monotonically (:5630)
%     8  doubling yTIC doubles nc(0)                   :5628
%     9  doubling tF quarters nc(0)                    :5628
%        8 and 9 cost one extra engagement each and carry the chapter for us.
%
%   INFO, never scored: tF = 2*yTIC/(Vc*BW) (:5652-5656), because BW is an
%   input we choose; and the endgame whip.  Nothing else on Figs. 19.2/19.3/
%   19.6 is scoreable -- the book publishes no numbers off them.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** THIS FILE HAS NEVER BEEN RUN IN MATLAB --
%   MATLAB IS THE CALLER'S. ***  What follows is from an independent replica of
%   the same loop, written in another language while this port was being
%   written and cross-checked against the exact geometry at the switch (RTM1 =
%   4000, RTM2 = 200, VTM1 = -4000, VTM2 = 0).  They agree to the digits shown.
%   A second implementation is still not this implementation:
%
%     Vc(0)          4000.000000000000 ft/s exactly
%     resolution     T = 4.000000000 s, TGO = 1.000000000000 s exactly
%     ncMAX at res   597.757 ft/s^2 = 18.5639 g   (book 600 / 18.6;  -0.374%)
%     endgame whip   2288.8 ft/s^2 at RTM = 0.1929 ft   *** ARTEFACT ***
%     recorded peak  567.880 ft/s^2 = 17.636 g   (all Fig. 19.3 can show)
%     miss           0.192942 ft      final altitude 1200.0145 ft (+0.0012%)
%     flight time    5.003050 s       downrange at intercept 14997.14 ft
%     row 5 resid    0.3534% of peak      row 6 resid 0.4488% of yTIC
%     row 8 ratio    1.977764 (-1.112%)   row 9 ratio 0.250703 (+0.281%)
%     6902 outer steps, 100 recorded samples
%
%   A residual near a band EDGE is not a pass to be accepted; the mechanisms
%   above say where each should land.  Do not widen a band to make a row green.
%
% ---------------------------------------------------------------------------
%   OUTPUTS
%     R.T .. R.RT2   ArrayT/ArrayRM1/ArrayRM2/ArrayRT1/ArrayRT2 [s, ft]
%     R.NC, R.NCG    commanded acceleration [ft/s^2] and [g] (ArrayXNCG)
%     R.nc_res       command AT RESOLUTION.  The book's ncMAX, and the only one
%                    of the four peaks that is scored.  R.nc_max_win (RTM >=
%                    R.rtmFloor), R.nc_max_all (the whip -- INFO only) and
%                    R.nc_rec_max (the most Fig. 19.3 could show) are the rest.
%     R.miss         the listing's final RTM [ft].  THE ANSWER.
%     R.t_switch     when resolution fired [s]; R.tgo_switch the t_go then
%     R.cf           closed forms on the post-resolution window: .t .u .nc .y
%     R.bench        benchmark rows, schema shared with the other ports
%     R.selfcheck    structural residuals (self-consistency, NOT book values)
%     R.units        one line naming the units
%     R.pass         true only if at least one row was SCORED and all passed
%     R.verdict      '' when everything passed, else the one-line reason.
%                    ALWAYS FORMED.  'assert' chooses the REACTION only: true
%                    errors, false WARNS and still returns R complete -- a port
%                    that refuses to return is a port nobody can diagnose, and
%                    benchSuite runs every port with assert off for that reason.
%
%   See also MGS.ZARCHAN.BENCHSUITE, MGS.ZARCHAN.SHAPINGFILTERADJOINT.

arguments
    % Listing 19.1 line 1, verbatim.  Feet, ft/s, seconds.
    opts.XNP      (1,1) double {mustBeFinite, mustBePositive} = 3.0
    opts.DISPLACE (1,1) double {mustBeFinite}                 = 200.0
    opts.THOM     (1,1) double {mustBeFinite, mustBePositive} = 1.0
    opts.VM       (1,1) double {mustBeFinite, mustBePositive} = 3000.0
    opts.VT       (1,1) double {mustBeFinite, mustBeNonnegative} = 1000.0
    opts.RM1      (1,1) double {mustBeFinite} = 0.0
    opts.RM2      (1,1) double {mustBeFinite} = 1000.0
    opts.RT1      (1,1) double {mustBeFinite} = 20000.0
    opts.RT2      (1,1) double {mustBeFinite} = 1000.0
    opts.H        (1,1) double {mustBeFinite, mustBePositive} = 0.005
    % The endgame step and its trigger.  See transcription note 1: TGO_FINE is
    % exposed precisely so the rejected reading of "if(TGO..3)" can be run.
    opts.H_END    (1,1) double {mustBeFinite, mustBePositive} = 0.00005
    opts.TGO_FINE (1,1) double {mustBeFinite, mustBePositive} = 0.3
    opts.samp     (1,1) double {mustBeFinite, mustBePositive} = 0.05
    % Seeker beamwidth [rad].  INFO row only -- nothing is simulated with it.
    opts.BW       (1,1) double {mustBeFinite, mustBePositive} = 0.1
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so an "if nargin == 0" here would be a mode that never fires.
    opts.scaling   (1,1) logical = true    % rows 8/9: two extra engagements
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

XNP = opts.XNP;  DISP = opts.DISPLACE;  THOM = opts.THOM;
VM  = opts.VM;   VT   = opts.VT;

if opts.H_END > opts.H
    error('mgs:zarchan:stepDisplacement:step', ...
        ['H_END = %g s is larger than H = %g s.  The endgame branch exists ' ...
         'to REFINE the step so the miss can be resolved; inverted, it ' ...
         'coarsens exactly where the answer is read off, and the run still ' ...
         'completes smoothly.'], opts.H_END, opts.H);
end
if opts.samp < opts.H
    error('mgs:zarchan:stepDisplacement:samp', ...
        ['samp = %g s is below the integration step H = %g s, so the ' ...
         'returned curve would be sampled at the step rate while claiming ' ...
         'the finer interval in its own time vector.'], opts.samp, opts.H);
end

% =========================================================================
%   LISTING 19.1.  Transcribed from zarchan.txt:5565-5579.
% =========================================================================
E = flyEngagement(XNP, DISP, THOM, VM, VT, opts.RM1, opts.RM2, ...
                  opts.RT1, opts.RT2, opts.H, opts.H_END, opts.TGO_FINE, ...
                  opts.samp);

if ~(E.VC0 > 0)
    error('mgs:zarchan:stepDisplacement:opening', ...
        ['the closing velocity at t = 0 is %g ft/s, so the listing''s own ' ...
         '"while VC > 0" loop never executes and every array comes back ' ...
         'empty.  The missile must start on a CLOSING geometry.'], E.VC0);
end
if E.nswitch ~= 1
    error('mgs:zarchan:stepDisplacement:switch', ...
        ['the target displacement fired %d time(s), not once.  THOM = %g s ' ...
         'against a total flight time of about %g s (= RTM0/Vc0): if THOM is ' ...
         'not strictly inside the engagement there is no post-resolution ' ...
         'window and every scored row would be measuring an empty interval.'], ...
        E.nswitch, THOM, E.TGO0);
end
if ~all(isfinite([E.miss, E.nc_res, E.nc_max_win, E.NC(:).']))
    error('mgs:zarchan:stepDisplacement:nonfinite', ...
        ['the engagement produced a non-finite value.  XLAMD divides by ' ...
         'RTM^2, so this is what a range collapse looks like -- it is ' ...
         'refused rather than returned, because a NaN miss renders as a ' ...
         'blank rather than as a failure.']);
end

% ---- the book's two closed forms, on the post-resolution window ------------
% zarchan.txt:5619-5620 and :5622-5623.  Shares no code with the integrator,
% which is the whole reason it is worth having.
post   = E.T > E.t_switch + 1e-9;
tPost  = E.T(post) - E.t_switch;
uPost  = max(0, 1 - tPost/THOM);
cf     = struct();
cf.t   = tPost;
cf.u   = uPost;
cf.nc  = ncStepCF(uPost, THOM, XNP, DISP);
cf.y   = ySepCF(uPost, XNP, DISP);
cf.nc_meas = E.NC(post);
cf.y_meas  = E.RT2(post) - E.RM2(post);      % relative altitude = the linear y

R = struct();
R.T   = E.T;    R.RM1 = E.RM1;  R.RM2 = E.RM2;
R.RT1 = E.RT1;  R.RT2 = E.RT2;
R.NC  = E.NC;   R.NCG = E.NC/32.2;
R.nc_res     = E.nc_res;              % THE BOOK'S ncMAX.  See the header.
R.nc_res_g   = E.nc_res/32.2;
R.nc_max_win = E.nc_max_win;
R.nc_max_all = E.nc_max_all;
R.nc_max_T   = E.nc_max_T;
R.nc_max_RTM = E.nc_max_RTM;
R.nc_rec_max = E.nc_rec_max;
R.rtmFloor   = E.rtmFloor;
R.miss       = E.miss;
R.t_switch   = E.t_switch;
R.tgo_switch = E.tgo_switch;
R.VC0        = E.VC0;
R.RTM0       = E.RTM0;
R.TGO0       = E.TGO0;
R.T_final    = E.T_final;
R.steps      = E.steps;
R.cf         = cf;
R.units      = 'ft, ft/s, ft/s^2, rad; g = 32.2 ft/s^2 (the book''s units, NOT SI)';
R.p = struct('XNP',XNP, 'DISPLACE',DISP, 'THOM',THOM, 'VM',VM, 'VT',VT, ...
             'RM1',opts.RM1, 'RM2',opts.RM2, 'RT1',opts.RT1, 'RT2',opts.RT2, ...
             'H',opts.H, 'H_END',opts.H_END, 'TGO_FINE',opts.TGO_FINE, ...
             'samp',opts.samp, 'BW',opts.BW, 'scaling',opts.scaling, ...
             'units',R.units);

% ---- rows 8/9 need two more engagements ----------------------------------
R.scale = [];
if opts.scaling
    E2 = flyEngagement(XNP, 2*DISP, THOM, VM, VT, opts.RM1, opts.RM2, ...
                       opts.RT1, opts.RT2, opts.H, opts.H_END, ...
                       opts.TGO_FINE, opts.samp);
    E3 = flyEngagement(XNP, DISP, 2*THOM, VM, VT, opts.RM1, opts.RM2, ...
                       opts.RT1, opts.RT2, opts.H, opts.H_END, ...
                       opts.TGO_FINE, opts.samp);
    % Both variants must resolve ONCE and at the t_go they were asked for.
    % "nswitch == 1" alone is not enough: if 2*THOM exceeds the whole flight
    % the jump fires on the first step, which counts as one fire but gives a
    % homing window of TGO0, not 2*THOM -- and the ratio row would then be
    % scoring a different experiment while looking perfectly healthy.
    ok2 = E2.nswitch == 1 && abs(E2.tgo_switch - THOM)   <= 1.5*opts.H;
    ok3 = E3.nswitch == 1 && abs(E3.tgo_switch - 2*THOM) <= 1.5*opts.H;
    % Ratios of nc_res, NOT of nc_max_all: zarchan.txt:5628 is a claim about
    % "the INITIAL missile acceleration", and nc_max_all is the endgame whip.
    R.scale = struct('nc_2disp', E2.nc_res, 'nc_2tf', E3.nc_res, ...
                     'ratio_disp', abs(E2.nc_res)/max(abs(E.nc_res), realmin), ...
                     'ratio_tf',   abs(E3.nc_res)/max(abs(E.nc_res), realmin), ...
                     'tgo2', E2.tgo_switch, 'tgo3', E3.tgo_switch, ...
                     'ok2', ok2, 'ok3', ok3);
end

R.selfcheck = structuralChecks(E, DISP, THOM, opts.H);

if opts.benchmark
    R.bench = benchmarks(R, E, cf, opts);
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

msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated at N'' = %g, ' ...
        'yTIC = %g ft, THOM = %g s.  An unvalidated port must not be handed ' ...
        'back silently.'], XNP, DISP, THOM);
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
        error('mgs:zarchan:stepDisplacement:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN.  R comes back complete so the caller can see WHICH
        % side of a comparison moved.
        warning('mgs:zarchan:stepDisplacement:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function E = flyEngagement(XNP, DISPLACE, THOM, VM, VT, RM1, RM2, RT1, RT2, ...
                           H0, HEND, TGOFINE, samp)
%FLYENGAGEMENT  Listing 19.1, zarchan.txt:5565-5579.
%
%   Every constant this needs is an ARGUMENT.  MATLAB local functions cannot
%   see the main function's workspace; an earlier wave in this package read a
%   constant from the caller and threw on the first line of the first run.

% ---- the listing's line 1, verbatim except for the plotting ---------------
n = 0;
QSWITCH = 0;
VT1 = -VT;  VT2 = 0.0;
T = 0.0;  S = 0.0;
RTM1 = RT1 - RM1;  RTM2 = RT2 - RM2;
RTM  = sqrt(RTM1^2 + RTM2^2);
XLAM = atan2(RTM2, RTM1);                                            %#ok<NASGU>
VM1  = VM;  VM2 = 0.0;
VTM1 = VT1 - VM1;  VTM2 = VT2 - VM2;
VC   = -(RTM1*VTM1 + RTM2*VTM2)/RTM;
TGO  = RTM/VC;
H    = H0;

E = struct();
E.VC0 = VC;  E.RTM0 = RTM;  E.TGO0 = TGO;
% Is the missile on a collision triangle?  At the listing's own ICs both of
% these are EXACTLY zero, which is why the command is exactly zero until
% resolution (zarchan.txt:5555).  Recorded so that structural check 2 can tell
% "the wiring is wrong" apart from "the caller moved the missile off-course".
E.rtm2_0   = RTM2;
E.vtm2_0   = VTM2;
E.collision = abs(RTM2) <= 1e-9 && abs(VTM2) <= 1e-9;

% AM1/AM2 are read by the FLAG==1 branch.  The listing never initialises them
% and does not need to (the branch is not taken on the very first pass), but a
% zero here is both the correct pre-switch physics and one less thing that can
% carry a stale value in from a future edit.
AM1 = 0.0;  AM2 = 0.0;
XNC = 0.0;

nrec = ceil(2*max(TGO, 0)/samp) + 16;
E.T   = zeros(nrec,1);  E.RM1 = zeros(nrec,1);  E.RM2 = zeros(nrec,1);
E.RT1 = zeros(nrec,1);  E.RT2 = zeros(nrec,1);  E.NC  = zeros(nrec,1);

% THREE different "peaks", and they are NOT interchangeable -- see the header.
%   nc_res      the command at the instant of resolution.  The book's ncMAX.
%   nc_max_win  max |XNC| while RTM is still above rtmFloor.  The homing peak.
%   nc_max_all  max over EVERY evaluation.  Contaminated by the endgame.
% rtmFloor is twenty endgame steps of range.  Below that a single step
% straddles closest approach, where the LOS rate is unbounded for any nonzero
% miss; the listing never records or plots that region, and a "peak" read from
% it is a property of the step size, not of the guidance.
rtmFloor   = max(1.0, 20*abs(VC)*HEND);
nc_max_all = 0.0;  nc_max_win = 0.0;
nc_max_T   = NaN;  nc_max_RTM = NaN;
nc_pre_max = 0.0;          % max |XNC| BEFORE resolution -- selfcheck 2
nc_res     = 0.0;  gotRes = false;
t_switch   = NaN;  tgo_switch = NaN;  nswitch = 0;
rtm2_before = NaN; rtm2_after = NaN;

maxSteps = 4e6;
steps    = 0;

while VC > 0.0
    steps = steps + 1;
    if steps > maxSteps
        error('mgs:zarchan:stepDisplacement:runaway', ...
            ['%d integration steps without the closing velocity going ' ...
             'negative.  The loop test is the listing''s own "while VC > 0", ' ...
             'which has no other exit.'], steps);
    end

    TGO = RTM/VC;
    % Transcription note 1.  "if(TGO..3)" -> TGO < TGO_FINE; see the header
    % for the 5571-vs-5669 contradiction and why 5669 wins.
    if TGO < TGOFINE
        H = HEND;
    end
    % zarchan.txt:5573.  The apparent step in target position.
    if TGO <= THOM && QSWITCH == 0
        QSWITCH  = 1;
        rtm2_before = RT2 - RM2;
        RT2      = RT2 + DISPLACE;
        rtm2_after  = RT2 - RM2;
        nswitch  = nswitch + 1;
        t_switch = T;  tgo_switch = TGO;
    end

    RT1OLD = RT1;  RT2OLD = RT2;  RM1OLD = RM1;  RM2OLD = RM2;
    VM1OLD = VM1;  VM2OLD = VM2;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  Pass 1 evaluates the derivatives at
    % the current state; pass 2 takes a full Euler step, advances T by H, and
    % re-evaluates there.  T advances INSIDE this loop, exactly once.  Move
    % that line out and the scheme quietly degrades to first order while still
    % running perfectly smoothly.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            RT1 = RT1 + H*VT1;   RT2 = RT2 + H*VT2;
            RM1 = RM1 + H*VM1;   RM2 = RM2 + H*VM2;
            VM1 = VM1 + H*AM1;   VM2 = VM2 + H*AM2;
            T   = T + H;
        end
        RTM1 = RT1 - RM1;  RTM2 = RT2 - RM2;
        RTM  = sqrt(RTM1^2 + RTM2^2);
        VTM1 = VT1 - VM1;  VTM2 = VT2 - VM2;
        VC    = -(RTM1*VTM1 + RTM2*VTM2)/RTM;
        XLAM  = atan2(RTM2, RTM1);
        XLAMD = (RTM1*VTM2 - RTM2*VTM1)/(RTM*RTM);
        XNC   = XNP*VC*XLAMD;
        AM1   = -XNC*sin(XLAM);
        AM2   =  XNC*cos(XLAM);
        FLAG  = 1;

        % NOT in the listing.  The peak lives at the resolution instant, which
        % is never on the record grid -- see the header.  Additions only; none
        % of this changes any state the listing integrates.
        if abs(XNC) > nc_max_all
            nc_max_all = abs(XNC);  nc_max_T = T;  nc_max_RTM = RTM;
        end
        if RTM >= rtmFloor
            nc_max_win = max(nc_max_win, abs(XNC));
        end
        if QSWITCH == 0
            nc_pre_max = max(nc_pre_max, abs(XNC));
        elseif ~gotRes
            nc_res = XNC;             % the FIRST command after resolution
            gotRes = true;
        end
    end
    % The listing writes "FLAG=0" here too.  Dropped: the outer loop
    % re-initialises FLAG on its next pass, so the assignment cannot be read.

    % Heun corrector, in the listing's own algebra.
    RT1 = .5*(RT1OLD + RT1 + H*VT1);
    RT2 = .5*(RT2OLD + RT2 + H*VT2);
    RM1 = .5*(RM1OLD + RM1 + H*VM1);
    RM2 = .5*(RM2OLD + RM2 + H*VM2);
    VM1 = .5*(VM1OLD + VM1 + H*AM1);
    VM2 = .5*(VM2OLD + VM2 + H*AM2);

    % Transcription note 2.  The listing's literal is "if S > .049999", i.e.
    % strictly greater, with the constant one part in 1e5 below samp; that
    % slack is what absorbs the floating-point accumulation of S.
    S = S + H;
    if S > samp - 1e-6
        S = 0.0;
        n = n + 1;
        E.T(n)   = T;
        E.RM1(n) = RM1;  E.RM2(n) = RM2;
        E.RT1(n) = RT1;  E.RT2(n) = RT2;
        E.NC(n)  = XNC;                 % the listing stores XNC/32.2
    end
end

E.T = E.T(1:n);  E.RM1 = E.RM1(1:n);  E.RM2 = E.RM2(1:n);
E.RT1 = E.RT1(1:n);  E.RT2 = E.RT2(1:n);  E.NC = E.NC(1:n);
E.miss       = RTM;                     % the bare "RTM" the listing displays
E.nc_res     = nc_res;
E.nc_max_win = nc_max_win;
E.nc_max_all = nc_max_all;
E.nc_max_T   = nc_max_T;
E.nc_max_RTM = nc_max_RTM;
E.rtmFloor   = rtmFloor;
E.nc_rec_max = max([abs(E.NC); 0]);     % what the listing's own plot can show
E.nc_pre_max = nc_pre_max;
E.t_switch   = t_switch;
E.tgo_switch = tgo_switch;
E.nswitch    = nswitch;
E.rtm2_jump  = rtm2_after - rtm2_before;
E.T_final    = T;
E.RM2_final  = RM2;
E.RM1_final  = RM1;
E.steps      = steps;
end


% =============================================================================
function nc = ncStepCF(u, tF, N, yTIC)
%NCSTEPCF  zarchan.txt:5622-5623.  nc = N'*(1-t/tF)^(N'-2) * yTIC/tF^2.
%   u = 1 - t/tF, clamped at 0 by the caller: for non-integer N' a negative
%   base raised to a fractional power is COMPLEX in MATLAB, and complex
%   accelerations propagate silently into a real-looking abs() later.
nc = N * (u.^(N-2)) * yTIC / tF^2;
end


% =============================================================================
function y = ySepCF(u, N, yTIC)
%YSEPCF  zarchan.txt:5619-5620.  y = [N'(1-t/tF) - (1-t/tF)^N'] yTIC/(N'-1).
%   y(tF) = 0 identically -- the book's "the miss distance y(tF) is always
%   zero!" (:5624).  Undefined at N' = 1; the caller gates on that.
y = (N*u - u.^N) * yTIC / (N - 1);
end


% =============================================================================
function K = structuralChecks(E, DISPLACE, THOM, H)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.  They are the weaker
%   claim that it agrees with itself -- which is what catches wiring faults.
K = struct();
names = {};  oks = [];  detail = {};

% 1  The jump fires once, on the first step at or below THOM.  At the book's
%    own ICs it should land on TGO = THOM EXACTLY (RTM1 falls by exactly 20 ft
%    from exactly 20000), and BENCHMARK 2's band is sized on that.
firedOK = E.nswitch == 1 && isfinite(E.tgo_switch) && ...
          E.tgo_switch <= THOM + 1e-9 && E.tgo_switch > THOM - 1.5*H;
names{end+1}  = 'resolution fires once, in (THOM-H, THOM]';
oks(end+1)    = firedOK;
detail{end+1} = sprintf('%d fire(s); TGO = %.9f s vs THOM = %g s (T = %.5f s)', ...
                        E.nswitch, E.tgo_switch, THOM, E.t_switch);

% 2  No command before resolution.  zarchan.txt:5555 -- "no acceleration
%    commands will be required for most of the engagement" -- because the
%    missile is on a collision triangle, so XLAMD is identically zero.  A
%    nonzero value here means the initial geometry is not a collision course
%    and every post-resolution number is contaminated by a transient.
names{end+1}  = 'zero command before resolution (collision triangle)';
if E.collision
    oks(end+1)    = E.nc_pre_max <= 1e-9;
    detail{end+1} = sprintf('max |XNC| before the jump = %.3e ft/s^2', E.nc_pre_max);
else
    % NOT a failure: the caller moved the missile off the collision triangle,
    % so a pre-resolution command is the correct behaviour, not a wiring fault.
    % Reported rather than adjudicated -- a structural row that fails whenever
    % anyone changes an IC stops being read.
    oks(end+1)    = true;
    detail{end+1} = sprintf(['N/A -- ICs are not a collision triangle ' ...
        '(RTM2(0) = %g ft, VTM2(0) = %g ft/s); max |XNC| before the jump ' ...
        'was %.4g ft/s^2'], E.rtm2_0, E.vtm2_0, E.nc_pre_max);
end

% 3  The jump moves the relative altitude by exactly DISPLACE.  Cheap, and the
%    only thing that would notice DISPLACE being applied to the wrong state.
jumpErr = abs(E.rtm2_jump - DISPLACE);
names{end+1}  = 'jump moves RT2-RM2 by exactly DISPLACE';
oks(end+1)    = jumpErr <= 1e-9*max(1, abs(DISPLACE));
detail{end+1} = sprintf('d(RT2-RM2) = %+.9f ft, want %+g ft', E.rtm2_jump, DISPLACE);

% 4  Total flight time against the constant-closing-speed prediction RTM0/Vc0.
%    Derived from the listing's own ICs, NOT published: the turn costs a little
%    closing speed, so the run should be slightly LONGER, never shorter.
dT = E.T_final - E.TGO0;
names{end+1}  = 'flight time >= RTM0/Vc0 (the turn costs closing speed)';
oks(end+1)    = dT >= -2*H && dT <= 0.02*E.TGO0 + 2*H;
detail{end+1} = sprintf('T_final = %.5f s vs RTM0/Vc0 = %.5f s (%+.5f s)', ...
                        E.T_final, E.TGO0, dT);

K.names = names;
K.oks   = logical(oks);
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
function B = benchmarks(R, E, cf, opts)
%BENCHMARKS  Computed vs published, with the PROVENANCE of every band.
%
%   *** EVERY BOOK VALUE HERE WAS GREPPED OUT OF zarchan.txt AND THE LINE
%   NUMBER IS IN THE .source FIELD. ***  Nothing was read off a figure.  Where
%   the book states a SHAPE about a figure, that prose IS greppable, and an
%   ordering check is usually stronger than a plotted value.
B = emptyBench();
p = R.p;

% The book's own case.  Rows 1, 2 and 4 quote numbers that belong to THESE
% constants; off them, the printed value is simply about a different problem.
bookCase = abs(p.XNP-3) < 1e-12 && abs(p.DISPLACE-200) < 1e-9 && ...
           abs(p.THOM-1) < 1e-12 && abs(p.VM-3000) < 1e-9 && ...
           abs(p.VT-1000) < 1e-9 && abs(p.RM1) < 1e-9 && ...
           abs(p.RM2-1000) < 1e-9 && abs(p.RT1-20000) < 1e-9 && ...
           abs(p.RT2-1000) < 1e-9;
whyCase = sprintf(['off the listing''s own case -- the printed number is for ' ...
    'N''=3, yTIC=200 ft, THOM=1 s, VM=3000, VT=1000 ft/s from (0,1000) to ' ...
    '(20000,1000) ft; this run used N''=%g, yTIC=%g ft, THOM=%g s, VM=%g, ' ...
    'VT=%g ft/s'], p.XNP, p.DISPLACE, p.THOM, p.VM, p.VT);

% Linearisation validity.  The exact nonlinear command at the switch is
% nc_lin*(1 + (yTIC/(Vc*tF))^2)^-1.5, so the closed-form rows are meaningful
% only while that factor is close to 1.  0.15 caps the correction at -3.3%.
dr    = abs(p.DISPLACE)/max(R.VC0*p.THOM, realmin);
linOK = dr <= 0.15 && p.XNP > 1 + 1e-9 && numel(cf.t) >= 3;
whyLin = sprintf(['the linear closed forms need yTIC/(Vc*tF) small and ' ...
    'N'' > 1; this run has yTIC/(Vc*tF) = %.4f (cap 0.15), N'' = %g, and %d ' ...
    'post-resolution sample(s)'], dr, p.XNP, numel(cf.t));

ncPeakRef = p.XNP*abs(p.DISPLACE)/p.THOM^2;   % the book's linear nc(0)

% ---- 1.  closing velocity --------------------------------------------------
if bookCase
    B = addVal(B, 'closing velocity Vc at t = 0', 4000, R.VC0, 'text', 1e-9, ...
        'p.419 text (zarchan.txt:5654, :5661)', 'ft/s', ...
        ['exact from the ICs: head-on, VM = 3000 + VT = 1000.  Band is ' ...
         'machine epsilon because no integration has happened yet']);
else
    B = addSkip(B, 'closing velocity Vc at t = 0', '4000', whyCase, ...
        'p.419 text (zarchan.txt:5654, :5661)', 'ft/s');
end

% ---- 2.  the peak command.  THE headline number of the chapter. ------------
if bookCase
    B = addVal(B, 'ncMAX at resolution vs book 600 ft/s^2', 600, R.nc_res, ...
        'text+geometry', 0.02, 'p.419 text (zarchan.txt:5658-5660)', 'ft/s^2', ...
        sprintf(['book: ncMAX = 3*yTIC/tF^2 = 600 ft/s^2 = 18.6 g; ours ' ...
                 '%.4f g.  Measured AT RESOLUTION, which is where :5657 says ' ...
                 'the maximum is -- NOT as a max over the run; see the INFO ' ...
                 'row on the endgame.  Predicted residual -0.374%%: the exact ' ...
                 'geometry gives nc_nl/nc_lin = (1+(yTIC/(Vc*tF))^2)^-1.5. ' ...
                 'Band also covers the switch landing up to H early ' ...
                 '(2H/THOM = %.2f%%), which at the book''s ICs it does not'], ...
                 R.nc_res_g, 100*2*p.H/p.THOM));
else
    B = addSkip(B, 'ncMAX at resolution vs book 600 ft/s^2', '600', ...
        whyCase, 'p.419 text (zarchan.txt:5658-5660)', 'ft/s^2');
end

% ---- 3.  the miss.  The only row that survives off-book parameters. -------
missTol = 2.0;
B = addBound(B, 'miss distance ~ 0 (zero-lag PN hits the target)', ...
    sprintf('final RTM <= %.2f ft', missTol), ...
    sprintf('%.4f ft', R.miss), R.miss <= missTol, ...
    'p.413/417 text (zarchan.txt:5563, :5624)', 'ft', ...
    sprintf(['"the missile hits aircraft 1" (:5563) and "the miss distance ' ...
             'y(tF) is always zero!" (:5624).  The listing''s own endgame ' ...
             'resolution is Vc*H_END = %.3f ft (the book states that ' ...
             'reasoning for this idiom at :646), so the band is ~%.0fx that'], ...
             R.VC0*p.H_END, missTol/max(R.VC0*p.H_END, realmin)));

% ---- 4.  where the missile ends up ----------------------------------------
if bookCase
    B = addVal(B, 'missile final altitude = aircraft 1 altitude', 1200, ...
        E.RM2_final, 'text', 0.005, 'p.411 text (zarchan.txt:5551, :5554)', 'ft', ...
        ['aircraft 1 is at 1200 ft, aircraft 2 at 800 ft, power centroid ' ...
         'halfway at 1000 ft.  Flip the sign of DISPLACE and the miss row ' ...
         'still passes (it intercepts the other way); this row does not']);
else
    B = addSkip(B, 'missile final altitude = aircraft 1 altitude', '1200', ...
        whyCase, 'p.411 text (zarchan.txt:5551, :5554)', 'ft');
end

% ---- 5.  nc(t) against the printed closed form ----------------------------
if linOK
    r5 = max(abs(cf.nc_meas - cf.nc))/max(ncPeakRef, realmin);
    B = addBound(B, 'nc(t) vs closed form over the homing window', ...
        '<= 3.00% of the book''s linear peak', ...
        sprintf('%.4f%% of peak (%d samples)', 100*r5, numel(cf.t)), ...
        r5 <= 0.03, 'p.416/418 (zarchan.txt:5622-5623, claim at :5629)', 'ft/s^2', ...
        ['Fig. 19.5: the closed form "approximates the nonlinear results ' ...
         'very well".  Normalised by the BOOK''s peak N''*yTIC/tF^2, never ' ...
         'by our own max, so a wrong port cannot shrink its own denominator. ' ...
         'Expect ~0.4%, dominated by the (1+(yTIC/(Vc*tF))^2)^-1.5 factor']);
else
    B = addSkip(B, 'nc(t) vs closed form over the homing window', ...
        'N''(1-t/tF)^(N''-2) yTIC/tF^2', whyLin, ...
        'p.416 (zarchan.txt:5622-5623)', 'ft/s^2');
end

% ---- 6.  the relative separation against the OTHER printed closed form ----
if linOK
    r6 = max(abs(cf.y_meas - cf.y))/max(abs(p.DISPLACE), realmin);
    B = addBound(B, 'y(t) vs closed form (relative separation)', ...
        '<= 3.00% of yTIC', sprintf('%.4f%% of yTIC', 100*r6), r6 <= 0.03, ...
        'p.416 (zarchan.txt:5619-5620)', 'ft', ...
        ['the SECOND printed formula, and the one that makes "y(tF) is ' ...
         'always zero" a curve rather than an endpoint.  Compared against ' ...
         'RT2-RM2, which is the linear model''s y for a nearly-constant LOS']);
else
    B = addSkip(B, 'y(t) vs closed form (relative separation)', ...
        '[N''(1-t/tF) - (1-t/tF)^N''] yTIC/(N''-1)', whyLin, ...
        'p.416 (zarchan.txt:5619-5620)', 'ft');
end

% ---- 7.  THE SHAPE CLAIM ---------------------------------------------------
% zarchan.txt:5630 about Fig. 19.6: "the missile acceleration is always
% monotonically decreasing as the flight progresses".
if numel(cf.t) >= 3
    rise   = max(diff(abs(cf.nc_meas)));
    atRes  = abs(R.nc_max_win - abs(R.nc_res)) <= 1e-9*max(1, abs(R.nc_res));
    B = addShape(B, '|nc| peaks AT resolution, then decreases monotonically', ...
        'peak at the resolution instant; no later rise above 1% of the linear peak', ...
        sprintf('peak at resolution: %d; largest rise %+.4f%% of peak (%d samples from t_go = %.3f s)', ...
                atRes, 100*rise/max(ncPeakRef, realmin), numel(cf.t), R.tgo_switch), ...
        atRes && rise <= 0.01*ncPeakRef, ...
        'p.418/419 text (zarchan.txt:5630, :5657)', ...
        ['two prose claims, one geometric statement: ":5657 the maximum ' ...
         'acceleration occurs at the time of resolution" and ":5630 the ' ...
         'missile acceleration is always monotonically decreasing".  The ' ...
         'window is post-resolution ONLY -- before the jump the command is ' ...
         'identically zero, so the full record is not monotone and never was. ' ...
         '1% of peak is ~1/5 of one sample''s true decrement at samp = 0.05 s']);
else
    B = addSkip(B, '|nc| peaks AT resolution, then decreases monotonically', ...
        'monotone decreasing from the resolution instant', ...
        sprintf(['only %d post-resolution sample(s) at samp = %g s -- ' ...
                 '"monotone" would be a statement about the sampling'], ...
                 numel(cf.t), p.samp), ...
        'p.418/419 text (zarchan.txt:5630, :5657)', 'ft/s^2');
end

% ---- 8, 9.  THE SIZING LAW.  The two rows that carry the chapter for us. ---
% zarchan.txt:5628: "Doubling the apparent target displacement will double the
% initial missile acceleration, whereas doubling the flight time ... will
% quarter the initial missile acceleration."
% The 2*yTIC run doubles yTIC/(Vc*tF), so it needs HALF the cap that linOK
% applies -- otherwise the variant could sit outside the linearisation while
% the nominal run sits comfortably inside it.
scaleOK = linOK && dr <= 0.075;
if opts.scaling && scaleOK && ~isempty(R.scale) && R.scale.ok2 && R.scale.ok3
    B = addVal(B, 'doubling yTIC doubles ncMAX', 2.0, R.scale.ratio_disp, ...
        'text+geometry', 0.03, 'p.417 text (zarchan.txt:5628)', '-', ...
        ['exact-geometry prediction 1.9777 (-1.115%): the correction factor ' ...
         '(1+(yTIC/(Vc*tF))^2)^-1.5 is applied at 2*yTIC in the numerator and ' ...
         'at yTIC in the denominator, so it does not cancel']);
    B = addVal(B, 'doubling tF quarters ncMAX', 0.25, R.scale.ratio_tf, ...
        'text+geometry', 0.03, 'p.417 text (zarchan.txt:5628)', '-', ...
        ['exact-geometry prediction 0.25070 (+0.282%).  This is the row our ' ...
         'own lateral-authority budget rests on: handover t_go enters as an ' ...
         'INVERSE SQUARE']);
else
    whyScale = sprintf(['%s; the scaling rows additionally cap ' ...
        'yTIC/(Vc*tF) at 0.075 because the variant run doubles it'], whyLin);
    if ~opts.scaling
        whyScale = ['''scaling'' is false, so the two extra engagements were ' ...
            'not run.  These are the rows that score the chapter''s sizing law'];
    elseif scaleOK && ~isempty(R.scale) && ~(R.scale.ok2 && R.scale.ok3)
        whyScale = sprintf(['a variant engagement did not resolve at the ' ...
            't_go it was asked for (wanted %g and %g s, got %.4f and %.4f s) ' ...
            '-- 2*THOM must still be strictly inside a flight of %.3f s'], ...
            p.THOM, 2*p.THOM, R.scale.tgo2, R.scale.tgo3, R.TGO0);
    end
    B = addSkip(B, 'doubling yTIC doubles ncMAX', '2', whyScale, ...
        'p.417 text (zarchan.txt:5628)', '-');
    B = addSkip(B, 'doubling tF quarters ncMAX', '0.25', whyScale, ...
        'p.417 text (zarchan.txt:5628)', '-');
end

% ---- INFO.  Reported, never scored. ---------------------------------------
% THE ENDGAME WHIP.  Measured, not theorised: at the book's own settings the
% max over every derivative evaluation is 2288.8 ft/s^2 (71 g) and it lands on
% the LAST step, at RTM = 0.19 ft.  It is not roundoff -- as RTM -> 0 the LOS
% rate is genuinely unbounded for any nonzero miss, and one step straddles
% closest approach.  Disclosed rather than clipped, because a "peak
% acceleration" read off this listing without the window is wrong by ~4x and
% looks entirely plausible.
B = addInfo(B, 'endgame whip (max over ALL evaluations)', ...
    'not a book quantity', ...
    sprintf(['%.1f ft/s^2 = %.1f g at T = %.5f s, RTM = %.4f ft; windowed ' ...
             '(RTM >= %.2f ft) peak is %.4f ft/s^2, and the listing''s own ' ...
             'recorded array maxes at %.4f ft/s^2 = %.4f g'], ...
             R.nc_max_all, R.nc_max_all/32.2, R.nc_max_T, R.nc_max_RTM, ...
             R.rtmFloor, R.nc_max_win, R.nc_rec_max, R.nc_rec_max/32.2), ...
    'no source -- an artefact of the 1/RTM^2 LOS rate at zero range', ...
    ['NOTE FOR ANYONE READING Fig. 19.3: the recorded array cannot reach ' ...
     '18.6 g either.  ArrayXNCG is sampled every samp seconds and the ' ...
     'resolution instant is never on that grid, so the listing''s own plotted ' ...
     'peak is ~17.6 g.  The book''s "nearly 20 g" (:5581) is the FORMULA''s ' ...
     '600 ft/s^2 being read back onto the figure, not a number off the curve']);

tF_bw = 2*abs(p.DISPLACE)/max(R.VC0*p.BW, realmin);
B = addInfo(B, 'homing time from seeker beamwidth', 'tF = 2*yTIC/(Vc*BW)', ...
    sprintf('%.4f s at BW = %g rad (%.2f deg), yTIC = %g ft, Vc = %.1f ft/s', ...
            tF_bw, p.BW, p.BW*180/pi, abs(p.DISPLACE), R.VC0), ...
    'p.419 text (zarchan.txt:5652-5656)', ...
    ['BW is an input we choose, so this is a design relation being reported, ' ...
     'not a measurement being reproduced.  The book''s instance: 0.1 rad, ' ...
     '400 ft aircraft spacing (yTIC = 200 ft), Vc = 4000 ft/s -> 1 s']);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want: a struct array
%   that quietly reordered fields would make [B.ok] mean something else, and it
%   fails as "Subscripted assignment between dissimilar structures", which
%   reads like a typo rather than a schema break.  Schema is the one
%   mgs.zarchan.benchSuite's adaptGeneric consumes.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.4g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.6g', ours);
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
L{end+1} = 'Zarchan Listing 19.1 -- NONLINEAR ENGAGEMENT WITH STEP IN TARGET DISPLACEMENT';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['   N'' = %g   yTIC = %g ft   THOM = %g s   VM = %g ft/s   ' ...
    'VT = %g ft/s'], p.XNP, p.DISPLACE, p.THOM, p.VM, p.VT);
L{end+1} = sprintf(['   H = %g s (%g s once TGO < %g s)   samp = %g s   ' ...
    '%d steps, %d samples'], p.H, p.H_END, p.TGO_FINE, p.samp, R.steps, numel(R.T));
L{end+1} = '';
L{end+1} = 'ENGAGEMENT';
L{end+1} = sprintf('   Vc(0)         %12.4f ft/s      RTM(0) %10.1f ft', R.VC0, R.RTM0);
L{end+1} = sprintf('   resolution at T = %.5f s, TGO = %.6f s', R.t_switch, R.tgo_switch);
L{end+1} = sprintf('   ncMAX at res  %12.4f ft/s^2    = %8.4f g', R.nc_res, R.nc_res_g);
L{end+1} = sprintf('   windowed peak %12.4f ft/s^2    (RTM >= %.2f ft)', ...
    R.nc_max_win, R.rtmFloor);
L{end+1} = sprintf('   endgame whip  %12.1f ft/s^2    at RTM = %.4f ft -- ARTEFACT', ...
    R.nc_max_all, R.nc_max_RTM);
L{end+1} = sprintf('   recorded peak %12.4f ft/s^2    = %8.4f g  (what Fig. 19.3 can show)', ...
    R.nc_rec_max, R.nc_rec_max/32.2);
L{end+1} = sprintf('   miss (RTM)    %12.5f ft', R.miss);
L{end+1} = sprintf('   flight time   %12.5f s         (RTM0/Vc0 = %.5f s)', ...
    R.T_final, R.TGO0);
if ~isempty(R.scale)
    L{end+1} = sprintf('   ncMAX at 2*yTIC / 2*tF: %.4f / %.4f ft/s^2  (ratios %.5f, %.5f)', ...
        R.scale.nc_2disp, R.scale.nc_2tf, R.scale.ratio_disp, R.scale.ratio_tf);
end
L{end+1} = '';

% ---- the post-resolution window, nonlinear vs the two closed forms --------
L{end+1} = 'POST-RESOLUTION WINDOW  (nonlinear vs the book''s closed forms)';
L{end+1} = sprintf('%9s %10s %12s %12s %12s %12s', ...
    't-tR [s]', '1-t/tF', 'nc [ft/s^2]', 'nc closed', 'y [ft]', 'y closed');
nsh = numel(R.cf.t);
if nsh > 0
    stride = max(1, round(nsh/12));
    for k = unique([1:stride:nsh, nsh])
        L{end+1} = sprintf('%9.4f %10.5f %12.4f %12.4f %12.4f %12.4f', ...
            R.cf.t(k), R.cf.u(k), R.cf.nc_meas(k), R.cf.nc(k), ...
            R.cf.y_meas(k), R.cf.y(k)); %#ok<AGROW>
    end
else
    L{end+1} = '   (no post-resolution samples)';
end
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-50s %s', vstr(K.oks(i)), K.names{i}, ...
        K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning';
    L{end+1} = '            NOT CHECKED, which is not the same thing as "fine".';
else
    L{end+1} = 'BENCHMARKS  (book values grepped from zarchan.txt; line numbers in .source)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
            case 'info'
                L{end+1} = sprintf('   INFO %-50s %s', b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case 'skip'
                L{end+1} = sprintf('   SKIP %-50s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case {'shape','bound'}
                L{end+1} = sprintf('   %-4s %-50s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf(['   %-4s %-50s book %s %s / ours %s %s ' ...
                    '(%+.3f%%, band %.4g%%)'], vstr(b.ok), b.what, b.book_str, ...
                    b.units, b.ours_str, b.units, b.pct, 100*b.tol); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail); %#ok<AGROW>
        end
    end
    L{end+1} = sprintf('   %d of %d scored rows pass; %d not scored.', ...
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
%DRAWCURVES  The listing's own two figures.  Base MATLAB graphics only.
figure;
plot(R.RM1, R.RM2, '-', R.RT1, R.RT2, '--');
grid on;
xlabel('Downrange (Ft)');
ylabel('Altitude (Ft)');
title('Missile first guides on power centroid and then guides on aircraft 1');
legend('missile', 'target', 'Location', 'best');

figure;
hold on; grid on;
plot(R.T, R.NCG, '-');
if ~isempty(R.cf.t)
    plot(R.cf.t + R.t_switch, R.cf.nc/32.2, '--');
    legend('Listing 19.1 nonlinear', 'Ch.19 closed form', 'Location', 'best');
end
xlabel('Time (Sec)');
ylabel('Acceleration (G)');
title(sprintf('Peak %.2f g at resolution to take out %g ft of target displacement', ...
    R.nc_res_g, abs(R.p.DISPLACE)));
hold off;
end
