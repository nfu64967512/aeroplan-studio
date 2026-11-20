function R = shapingNonlinear(opts)
%MGS.ZARCHAN.SHAPINGNONLINEAR  Zarchan Listing 24.2 -- NONLINEAR ENGAGEMENT
%   SIMULATION TO TEST TRAJECTORY SHAPING GUIDANCE.
%
%   R = mgs.zarchan.shapingNonlinear()                       % board + verdicts
%   R = mgs.zarchan.shapingNonlinear('benchmark',false, ...  % one engagement
%           'XLAMFDEG',-45,'APN',1)
%
%   Line-by-line port of Listing 24.2, Zarchan, Tactical and Strategic Missile
%   Guidance, Ch. 24, pp. 592-594.  Book source in this repo:
%       matlab/zarchan/_source/zarchan.txt:8358   (the listing)
%       matlab/zarchan/_source/zarchan.txt:8338   (the prose that sets it up)
%       matlab/zarchan/_source/zarchan.txt:8353   (the nominal case)
%       matlab/zarchan/_source/zarchan.txt:8382   (the published results)
%   Transcription this port was made from:
%       matlab/zarchan/_listings/listing_24_02.m
%
% ===========================================================================
%   UNITS ARE FEET.  ALL OF THEM.  NOTHING IS CONVERTED.
% ===========================================================================
%   ft, ft/s, ft/s^2, g = 32.2 ft/s^2, degrees via the book's 57.3 (NOT
%   180/pi).  This is the ft/s branch of the book, not the metric one.
%   Listing 30.1 -- which the book itself describes as "Listing 24.2 ...
%   modified for the case of the stationary target" (zarchan.txt:10930) -- IS
%   metric.  THIS FILE IS NOT.  If you diff the two, convert at the call site
%   and say so there; a 3.28 dropped between them turns a 30 kft engagement
%   into a 9 km one and every number still reads plausibly.
%
% ===========================================================================
%   WHAT THIS LISTING IS FOR, AND WHY IT MATTERS HERE SPECIFICALLY
% ===========================================================================
%   Ch. 24 derives a guidance law that hits the target AND drives the final
%   line-of-sight angle to a designer-chosen value (zarchan.txt:8111).  This
%   listing is the book's own NONLINEAR test of that law: Listing 2.1's
%   engagement with the shaping law dropped in and an APN flag to switch back
%   to plain PN for comparison.
%
%   We already run a trajectory-shaping term in this repo --
%   mgs.zarchan.additiveLaw -- and in August 2026 that term was found NOT to
%   be Zarchan's law at all: it had written the constant setpoint GAMF as the
%   CURRENT gamma, making it an attractor with the wrong sign on 32 of 32
%   release states.  It was corrected.  This file is the test that would have
%   caught it on day one, because the book publishes what the shaped
%   trajectory does and the wrong-sign version cannot produce it: under
%   Zarchan's law the initial command is POSITIVE (+22 g, nose up, lofting)
%   and the line-of-sight angle walks from -18.4 deg to -90 deg.  An attractor
%   on the current gamma commands ~0 on a collision triangle and the engagement
%   stays a straight line.  Those two are not close; one board row separates
%   them.
%
%   HOW THIS PORT AND ADDITIVELAW DIFFER ABOUT THE LAW'S FORM -- read this
%   before treating either as authority for the other.  They agree on the
%   things that were wrong before (additive not min(), a CONSTANT setpoint,
%   clamped AFTER the terms are summed, and the sign) and they disagree on
%   four things that are not cosmetic:
%
%   (1) VELOCITY.  Zarchan scales BOTH terms with VC, the CLOSING velocity --
%       zarchan.txt:8109 is the derivation, :8367 is the listing's own line:
%           XNC = 4.*VC*XLAMD + XNTPLOS + 2.*VC*(XLAM-XLAMF)/TGO
%       additiveLaw scales both with V, the MISSILE SPEED, and uses
%       t_go = R/V.  Substituting VC -> V leaves the RATIO of the two terms
%       unchanged but scales the whole demand by (V/VC)^2.  During a loft --
%       which is the entire mechanism of this law -- VC collapses while V does
%       not, so additiveLaw over-demands exactly where the book under-demands.
%       Neither file is "the correct one" for the other's problem; this one is
%       the book, and the difference is stated here so nobody has to rediscover
%       it from a trajectory.
%
%   (2) WHAT THE SETPOINT IS.  In Ch. 24 the setpoint is XLAMF, a final LINE
%       OF SIGHT angle (zarchan.txt:8106 derives it as lambda(tF)).  In Ch. 30
%       the SAME slot in the SAME expression is called GAMF, a final FLIGHT
%       PATH angle (zarchan.txt:10942) -- because Ch. 30 is the stationary
%       target special case, where the two coincide at intercept.  additiveLaw
%       took its name and its comment from Ch. 30 ("a DESIGNER-CHOSEN FINAL
%       flight-path angle").  That is right for a stationary target and only
%       for a stationary target.  Our red heart is stationary, so the naming is
%       currently benign; against anything that moves it is a silent aliasing
%       of two different angles.
%
%   (3) THE GAIN PAIRING.  additiveLaw's header flags shape_gain = 2.0 as
%       "paired with N' = 4" and unpaired with the n_vert = 5 we fly.  This
%       listing is stronger than that: THE 4 AND THE 2 ARE HARD LITERALS.
%       There is no XNP in the shaping branch at all -- XNP=3. exists in the
%       data statement and is used ONLY by the APN==0 (plain PN) branch.  The
%       book says why at zarchan.txt:8111: the law "appears to be a form of
%       augmented proportional navigation (with an effective navigation ratio
%       of 4 ...) plus an extra term".  Both numbers fall out of one Schwartz
%       inequality solution.  They are not two knobs; treating 2.0 as sweepable
%       against N' = 5 is a departure from the book, not a tuning of it.
%
%   (4) THE TARGET ACCELERATION TERM.  Zarchan's law carries + XNTPLOS, the
%       component of target acceleration perpendicular to the line of sight,
%       with coefficient EXACTLY 1 (not the 0.5*N' of ordinary augmented PN --
%       the book calls this out as "a different multiplier", :8111).
%       additiveLaw has no such term.  Against a stationary target that is
%       exactly equivalent; against anything else it is a missing input.
%
%   ⚠️ TRANSCRIPTION NOTE -- A SIGN THE PROSE LOST AND THE CODE KEPT.  The
%      extracted prose at zarchan.txt:8343 reads "nTPLOS = nT1 sin l + nT2
%      cos l".  The listing itself reads
%           XNTPLOS = -XNT1*sin(XLAM) + XNT2*cos(XLAM)
%      with a MINUS on the first term, and the listing is right: the missile
%      command is applied along (-sin XLAM, cos XLAM) two lines later
%      (AM1 = -XNC*sin(XLAM), AM2 = XNC*cos(XLAM)), so that is the unit vector
%      the target acceleration has to be projected on too.  The extraction
%      dropped a leading minus.  This port follows the CODE.  Taking the prose
%      would flip the sign of the augmentation and would show up only on
%      maneuvering-target cases -- i.e. only on the three cases at the bottom
%      of the board.
%
% ===========================================================================
%   THE OCR DAMAGE THIS PORT HAD TO RESOLVE
% ===========================================================================
%   The transcription's header declares one unresolved variable-to-variable
%   comparison.  It is this line:
%
%       if XNC.XNCLIM, XNC=XNCLIM; end            listing_24_02.m:19
%
%   RESOLVED AS  `if XNC > XNCLIM`.  Three independent reasons, in order of
%   strength:
%
%   (a) THE GLYPH MAP IS CONSISTENT INSIDE THIS ONE LISTING.  In the raw text
%       (zarchan.txt:8358-8380) ',' is '<' and '.' is '>', and five other
%       comparisons in the same listing fix that mapping by shape alone:
%           while VC .= 0     -> VC >= 0        (the engagement loop)
%           if RTM , 1000     -> RTM < 1000     (the fine-step trigger)
%           while STEP ,=1    -> STEP <= 1      (the two-pass integrator)
%           if XNC,-XNCLIM    -> XNC < -XNCLIM  (the LOWER clamp, next line)
%           if S.=.09999      -> S >= .09999    (the 0.1 s log trigger)
%       '.' is '>' in all of them.  The unresolved one is the same glyph.
%   (b) THE PAIRED LINE.  The very next statement clamps the LOWER side with
%       '<' and assigns -XNCLIM.  A pair that assigns +XNCLIM and -XNCLIM must
%       be an upper/lower clamp, and assigning the POSITIVE limit is only
%       meaningful when the command EXCEEDS it.
%   (c) THIS REPO ALREADY HOLDS THE ANSWER.  Listing 30.1 contains the same
%       two lines (zarchan.txt:10943), and the audited copy at
%       matlab/ref/zarchan_listing30_1.m:156 reads `if XNC>XNCLIM`.  Listing
%       24.1, the LINEAR twin of this one, carries the identical damaged glyph
%       at :8144 -- so the same pair appears three times in the book and is
%       already resolved once in this repo.
%
%   AND IT IS NOT LEFT AS AN ARGUMENT.  structuralCheck() below RUNS THE
%   COUNTERFACTUAL: the inverted reading `if XNC < XNCLIM, XNC = XNCLIM`
%   raises every sub-limit command TO the limit, so the command is pinned at
%   +XNCLIM for the whole flight, the missile flies a constant-g circle and
%   never closes.  The board carries that as a delete experiment with its own
%   row, because "the comment says the guard is there" is not evidence in this
%   repo ([[promised-guards-that-do-not-exist]]).
%
%   The transcription also declares 44 auto-repairs (quotes, relayout) and no
%   prose leakage.  One line of the transcription IS prose-adjacent and is
%   dropped here: `RTM figure plot(...)` -- the bare `RTM` is the listing's
%   final display of the miss distance, and it is returned as R.run.miss_ft
%   rather than echoed.
%
% ===========================================================================
%   WHAT THE BOOK PUBLISHES, AND WHAT SURVIVED EXTRACTION
% ===========================================================================
%   Every figure in Ch. 24 became an image link with no digits on it.  Nothing
%   below is read off a plot by this port; every scored row quotes a sentence.
%   The greppable set, with line numbers:
%
%     :8383  "reaching -90 deg at the end of the flight" (shaping goal met)
%     :8383  "proportional navigation ... ended up with a final line of sight
%             angle of -18.4 deg"
%     :8395  "the amount of guidance time for trajectory shaping guidance is
%             13.6 s"
%     :8390  "the flight time was nearly 4 s shorter" (PN vs shaping)
%     :8382  "requires more than 20 g of acceleration at the beginning of the
%             flight and nearly [-]10 g at the end"
%     :8382  PN "does not require any acceleration to hit the target in this
%             scenario because it is already on a collision triangle"
%     :8353  "we would like to hit the target vertically"; PN "essentially
%             travels in a straight line"; shaping trajectory "entirely
%             different"
%     :8405  final LOS goal -30 deg, stationary: "flight time ... reduces to
%             10.6 s"
%     :8435  moving+maneuvering: "closing velocity is approximately 4000 ft/s
%             while the flight time turns out to be 14.5 s"
%     :8436  same case at 3 g: "the time of flight reduced to 11.4 s"
%     :8418  -30 deg intercept "maximum positive acceleration was 30 g while
%             the maximum negative acceleration was -10 g"; +30 deg intercept
%             "only 10 g" / "-5 g"
%     :8406  "intercepts were achieved in both cases"
%     :8418  "the design goals for the final line of sight angles for both
%             cases were met"
%
%   Two more bounds come from the LISTING'S OWN CODE rather than from prose,
%   and are scored as bounds with that provenance stated on the row:
%       axis([0 14 -20 25])    -> the plotted command stays in [-20, 25] g
%       axis([0 14 -100 0])    -> the plotted LOS angle stays in [-100, 0] deg
%   These are weaker than a quoted number (a plot CAN clip data) and they are
%   labelled 'axis' so nobody mistakes them for published values.
%
%   ⚠️ THE OCR EATS MINUS SIGNS IN PROSE.  "230 deg", "220 deg", "210 g",
%      "290 deg" in the extracted text are -30, -20, -10, -90.  Every one of
%      the numbers above was read with that in mind; where the sign mattered
%      it was cross-checked against a line that kept its en-dash (:8383 and
%      :8406 both did).
%
%   ⚠️ ONE INFERRED PARAMETER, AND IT IS CHECKED BEFORE IT IS USED.  The three
%      "moving and maneuvering target" cases (:8406) never state VT.  This port
%      uses VT = 1000 ft/s, the book's standing target speed since Ch. 2, and
%      the inference is TESTED by its own row: on the collision triangle that
%      geometry gives VC = 3932 ft/s against the book's "approximately
%      4000" (:8435).  VT = 2000 would give ~4880 and VT = 500 ~3460, so the
%      sentence pins VT to about +/-150 ft/s.  If that row goes red, the three
%      cases below it are SKIPPED rather than scored -- a wrong inference must
%      not cascade into four false failures.
%
%   ⚠️ NOT SCORED, DELIBERATELY.  The book says the PN flight time was "nearly
%      4 s shorter" (:8390).  Its own two numbers -- 13.6 s shaping (:8395) and
%      the straight-line 31,623/3000 = 10.54 s -- give 3.06 s.  "Nearly 4"
%      is the author rounding his own figure generously; a band wide enough to
%      hold both 3.06 and 4.0 would not discriminate anything the 13.6 s row
%      does not already discriminate.  So the ORDERING is scored and the
%      MAGNITUDE is an INFO row.  Same treatment for the four Fig. 24.22
%      acceleration levels (30 / -10 / 10 / -5 g): the ORDERINGS between the
%      two cases are scored, the one-significant-figure levels are INFO.
%
% ===========================================================================
%   WHAT IS NOT MODELLED
% ===========================================================================
%   No gravity, no drag, no thrust, no autopilot lag, no seeker, no ground.
%   With HEDEG = -20 the missile's altitude goes NEGATIVE partway through the
%   maneuvering cases and the simulation does not care; neither does the book.
%   Do not read a miss distance off this file as a sortie prediction.
%
%   The commanded acceleration is perpendicular to the LINE OF SIGHT, not to
%   the velocity vector, so it DOES work on the missile and |VM| DRIFTS.  That
%   is the listing's own behaviour (Ch. 30 makes the choice a switch, ICHOICE;
%   Ch. 24 does not).  The drift is reported on the board as an INFO row --
%   it is the reason path length is integrated rather than taken as VM*tF.
%
%   See also MGS.ZARCHAN.ENGAGE2D (Listing 2.1, this listing's parent),
%   MGS.ZARCHAN.ADDITIVELAW, MGS.ZARCHAN.BENCHSUITE, ZARCHAN_LISTING30_1.

arguments
    % ---- the book's data statement (Listing 24.2, first line), verbatim ----
    %      Defaults are [] ON PURPOSE: empty means "the book value", non-empty
    %      means "the caller chose this".  That distinction is the ONLY thing
    %      that lets the benchmark refuse to score a run it was not written
    %      for.  Do not replace them with literals -- that puts a second copy
    %      of every book constant in the file, which is the drift this package
    %      is organised against.  The book values live in BOOKNOMINAL().
    opts.XNTG     double {mustBeReal, mustBeFinite} = []  % target maneuver [g]
    opts.HEDEG    double {mustBeReal, mustBeFinite} = []  % heading error [deg]
    opts.XNP      double {mustBeReal, mustBeFinite} = []  % N' -- APN==0 BRANCH ONLY
    opts.RM1IC    double {mustBeReal, mustBeFinite} = []  % missile downrange [ft]
    opts.RM2IC    double {mustBeReal, mustBeFinite} = []  % missile altitude [ft]
    opts.RT1IC    double {mustBeReal, mustBeFinite} = []  % target downrange [ft]
    opts.RT2IC    double {mustBeReal, mustBeFinite} = []  % target altitude [ft]
    opts.VM       double {mustBeReal, mustBeFinite} = []  % missile speed [ft/s]
    opts.VT       double {mustBeReal, mustBeFinite} = []  % target speed [ft/s]
    opts.XNCLIMG  double {mustBeReal, mustBeFinite} = []  % accel limit [g]
    opts.APN      double {mustBeReal, mustBeFinite} = []  % 1 = shaping, 0 = PN
    opts.XLAMFDEG double {mustBeReal, mustBeFinite} = []  % final LOS goal [deg]
    % ---- integration.  NOT model parameters, so these do NOT gate the -----
    %      benchmark; they are printed loudly instead, and the board carries a
    %      measured step-sensitivity row that says what moving them costs.
    opts.H        double {mustBeReal, mustBePositive} = []  % coarse step [s]
    opts.HFINE    double {mustBeReal, mustBePositive} = []  % terminal step [s]
    opts.RFINE    double {mustBeReal, mustBePositive} = []  % range at which H drops [ft]
    opts.TMAX     double {mustBeReal, mustBePositive} = []  % runaway guard [s], NOT in the book
    % ---- housekeeping -----------------------------------------------------
    opts.benchmark (1,1) logical = true    % THE ONLY SWITCH.  Never nargin.
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

% ⚠️⚠️ NOT `nargin == 0`.  In a function whose arguments block declares ONLY
%    name-value options, MATLAB reports nargin = 0 NO MATTER HOW MANY were
%    passed -- the packed opts struct is not a positional argument.  Measured
%    on R2025b and documented the same way in engage2d.m and engage3d.m.  A
%    `nargin == 0` mode switch here would mean every caller's single
%    engagement was silently replaced by the benchmark board.

GRAV_FT = 32.2;   % the book's g.  NOT 9.80665, NOT 9.81.
DEG     = 57.3;   % the book's rad<->deg constant.  NOT 180/pi (57.29578):
                  % round-tripping through 57.3 is not the identity and the
                  % published numbers were made with 57.3.  XLAMFDEG = -90
                  % becomes -1.5706806 rad here, not -pi/2, and that 6e-5 rad
                  % is deliberately kept.

N = bookNominal();
[cfg, offbook] = resolveConfig(opts, N);

R = struct();
R.units   = 'ft, ft/s, ft/s^2, g = 32.2, deg via 57.3 -- Zarchan''s units, NOT SI';
R.listing = 'Zarchan Listing 24.2 (zarchan.txt:8358, pp.592-594)';
R.entry   = 'shapingNonlinear(''benchmark'',true,''assert'',false,''render'',false)';
R.opts    = opts;
R.cfg     = cfg;
R.offbook = offbook;
R.nominal = N;

if cfg.benchmark
    % A caller who moved a MODEL parameter and asked for the board would get a
    % board built from the book cases, with their parameter silently ignored.
    % That is a lie the board would keep printing green, so it is a stop.
    if ~isempty(offbook)
        error('mgs:zarchan:shapingNonlinear:offbookBench', ...
            ['the benchmark board runs the BOOK''S OWN cases and would ' ...
             'ignore the parameter(s) you set (%s).  It would still print a ' ...
             'full green board, which is worse than failing.  Pass ' ...
             '''benchmark'', false to fly your own case, or drop the ' ...
             'parameter to check the port.'], strjoin(sort(offbook), ', '));
    end
    CASES     = bookCases(cfg, N, DEG);
    R.cases   = runCases(CASES, GRAV_FT, DEG);
    R.run     = R.cases.A;
    R.selfcheck = structuralCheck(R.cases, N, DEG);
    R.bench   = benchmarks(R.cases, R.selfcheck);
    ev        = ~[R.bench.skipped];
    R.pass    = any(ev) && all([R.bench(ev).ok]);
else
    R.cases     = struct();
    R.run       = flyOne(cfg, GRAV_FT, DEG);
    R.selfcheck = struct('skipped', true);
    R.bench     = emptyBench();
    ev          = false;
    R.pass      = false;   % NOT true.  Nothing was checked and the summary
                           % prints that sentence rather than a blank space.
end

R.summary = render(R, cfg, offbook, ev);
if cfg.render, fprintf('%s\n', R.summary); end
if cfg.plot,   drawCurves(R, cfg);        end

if cfg.benchmark && ~R.pass
    bad = find(~[R.bench.ok] & ev);
    if isempty(bad)
        % R.pass went false with nothing actually failing, which can only mean
        % EVERY row came back skipped.  "Nothing was evaluated" must not read
        % as "everything passed", and it must not read as a mismatch either.
        msg = ['not one book value could be evaluated -- every row is ' ...
               'SKIPPED.  A board that checks nothing is not a green board.'];
    else
        msg = sprintf(['%d of %d evaluated book values are outside tolerance ' ...
                       '(first: %s -- book %s, ours %s).  Full board in ' ...
                       'R.summary.'], ...
                      numel(bad), sum(ev), R.bench(bad(1)).what, ...
                      R.bench(bad(1)).book_str, R.bench(bad(1)).ours_str);
    end
    if cfg.assert
        error('mgs:zarchan:shapingNonlinear:benchmark', '%s', msg);
    else
        % ⚠️ assert = false STILL RETURNS THE RESULT with the verdict attached.
        %    A port that errored unconditionally here is why the previous wave
        %    could not tell which side of a mismatch was wrong.
        warning('mgs:zarchan:shapingNonlinear:benchmark', '%s', msg);
    end
end
if cfg.benchmark && isfield(R.selfcheck, 'ok') && ~R.selfcheck.ok
    msg = sprintf(['a STRUCTURAL self-check failed: %s.  These are wiring ' ...
                   'invariants, not book values -- a failure here voids every ' ...
                   'benchmark row above it.'], R.selfcheck.first_fail);
    if cfg.assert
        error('mgs:zarchan:shapingNonlinear:structural', '%s', msg);
    else
        warning('mgs:zarchan:shapingNonlinear:structural', '%s', msg);
    end
end
end


% =============================================================================
%   THE BOOK CONSTANTS.  ONE COPY, HERE.
% =============================================================================
function N = bookNominal()
%BOOKNOMINAL  Every literal on the first line of Listing 24.2
%   (zarchan.txt:8358), plus the integration settings from its loop head.
%
%   XNP = 3 is in the data statement but the SHAPING branch never reads it --
%   see the header, item (3).  It is kept at the book's value anyway so the
%   APN = 0 comparison case is the book's comparison case.
N = struct();
N.XNTG     = 0;         % XNTG=0.      -- stationary, non-maneuvering nominal
N.HEDEG    = 0;         % HEDEG=0.     -- ON the collision triangle
N.XNP      = 3;         % XNP=3.       -- APN==0 branch only
N.RM1IC    = 0;         % RM1IC=0.
N.RM2IC    = 10000;     % RM2IC=10000. -- missile at 10 kft
N.RT1IC    = 30000;     % RT1IC=30000. -- target 30 kft downrange
N.RT2IC    = 0;         % RT2IC=0.     -- target on the deck
N.VM       = 3000;      % VM=3000.
N.VT       = 0;         % VT=0.        -- stationary target
N.XNCLIMG  = 9999999;   % XNCLIMG=9999999.  i.e. no limit at all
N.APN      = 1;         % APN=1        -- 1 = shaping, 0 = plain PN (:8352)
N.XLAMFDEG = -90;       % XLAMFDEG=-90.  hit vertically (:8353)
% ---- the loop head, zarchan.txt:8365-8367 --------------------------------
N.H        = 0.0001;    % H=.0001      -- coarse
N.HFINE    = 0.00001;   % H=.00001     -- inside RFINE
N.RFINE    = 1000;      % if RTM < 1000
% ---- NOT in the book -----------------------------------------------------
N.TMAX     = 60;        % runaway guard.  The longest book case is 14.5 s.
end


function [C, offbook] = resolveConfig(opts, N)
%RESOLVECONFIG  Book value where the caller passed nothing; record which model
%   parameters the caller moved.  Integration settings are deliberately NOT in
%   the off-book list -- moving them changes the discretisation, not the
%   engagement, and the board measures exactly how much (see structuralCheck).
MODEL = {'XNTG','HEDEG','XNP','RM1IC','RM2IC','RT1IC','RT2IC','VM','VT', ...
         'XNCLIMG','APN','XLAMFDEG'};
INTEG = {'H','HFINE','RFINE','TMAX'};
C = struct();
offbook = {};
for i = 1:numel(MODEL)
    f = MODEL{i};
    if isempty(opts.(f))
        C.(f) = N.(f);
    else
        C.(f) = opts.(f);
        offbook{end+1} = f; %#ok<AGROW>
    end
end
C.integ_moved = {};
for i = 1:numel(INTEG)
    f = INTEG{i};
    if isempty(opts.(f))
        C.(f) = N.(f);
    else
        C.(f) = opts.(f);
        C.integ_moved{end+1} = f;
    end
end
C.benchmark = opts.benchmark;
C.render    = opts.render;
C.plot      = opts.plot;
C.assert    = opts.assert;

% Internal knobs, never exposed as options.  clamp_gt is set to FALSE by
% exactly one case -- H, in bookCases() -- which is the counterfactual for the
% OCR-damaged comparison; stop_on_tmax lets that case and the clamped case G
% report "never closed" as DATA instead of throwing and losing the board.
% Neither is reachable from the argument list, deliberately: a caller who
% could flip clamp_gt could produce a full green board off the wrong reading.
C.clamp_gt    = true;
C.stop_on_tmax = false;
C.label       = 'caller';

% The book's own guard makes this reachable and this port will not refuse it,
% but it is a contradiction worth printing: with VT == 0 the listing forces
% BETAD = 0 (zarchan.txt:8338 -- "target flight path rate has been modified so
% there is no division by zero"), so a stationary target NEVER MOVES no matter
% what XNTG says, yet XNTG still feeds XNTPLOS into the guidance law.  The
% missile is then compensating for a maneuver that is not happening.
C.inconsistent_target = (C.VT == 0 && C.XNTG ~= 0);
end


function CS = bookCases(C, N, DEG)
%BOOKCASES  The six engagements the Ch. 24 prose reports, plus the four runs
%   the structural checks need.  Each is built from the RESOLVED config, so a
%   caller-supplied H/HFINE/RFINE reaches every case (that is the supported way
%   to make the board cheaper) while the model parameters come from N.
%
%   VT = 1000 on cases D/E/F is the ONE inferred number in this file; the
%   header explains it and benchmarks() checks it before using it.
CS = struct('key',{},'title',{},'cfg',{});

% -- A.  THE NOMINAL (zarchan.txt:8353).  Stationary target 30 kft downrange,
%        missile at 10 kft on the collision triangle, hit vertically.
CS(end+1) = mkcase('A', 'shaping, XLAMF = -90, stationary target (the nominal)', ...
    setf(C, 'APN',1, 'XLAMFDEG',-90));

% -- B.  THE COMPARISON (zarchan.txt:8352 "by simply setting APN = 0").
CS(end+1) = mkcase('B', 'plain PN, N'' = 3, same geometry', ...
    setf(C, 'APN',0, 'XNP',N.XNP));

% -- C.  LESS SHAPING (zarchan.txt:8396) -- the -30 deg goal, still stationary.
CS(end+1) = mkcase('C', 'shaping, XLAMF = -30, stationary target', ...
    setf(C, 'APN',1, 'XLAMFDEG',-30));

% -- D/E/F.  MOVING AND MANEUVERING (zarchan.txt:8406).  HEDEG = -20 and a
%        6 g target; two final-LOS goals of opposite sign, then 3 g.
CS(end+1) = mkcase('D', 'shaping, XLAMF = -30, HE = -20, 6 g moving target', ...
    setf(C, 'APN',1, 'XLAMFDEG',-30, 'HEDEG',-20, 'XNTG',6, 'VT',1000));
CS(end+1) = mkcase('E', 'shaping, XLAMF = +30, HE = -20, 6 g moving target', ...
    setf(C, 'APN',1, 'XLAMFDEG', 30, 'HEDEG',-20, 'XNTG',6, 'VT',1000));
CS(end+1) = mkcase('F', 'shaping, XLAMF = -30, HE = -20, 3 g moving target', ...
    setf(C, 'APN',1, 'XLAMFDEG',-30, 'HEDEG',-20, 'XNTG',3, 'VT',1000));

% -- G.  DELETE EXPERIMENT on the clamp, CORRECT reading.  XNCLIMG drops from
%        the book's 9999999 to 15 g, which is BELOW the nominal's 22 g opening
%        demand and ABOVE its ~10 g terminal demand, so the clamp MUST fire
%        early and MUST release later.  15 rather than 20 because the margin is
%        what makes the row a measurement rather than a coin toss.
%        stop_on_tmax: a clamped run that fails to close is DATA for this
%        experiment, not a reason to throw and lose the whole board.
Gc = setf(C, 'APN',1, 'XLAMFDEG',-90, 'XNCLIMG',15);
Gc.stop_on_tmax = true;  Gc.TMAX = 40;
CS(end+1) = mkcase('G', 'nominal with XNCLIMG = 15 (clamp made live)', Gc);

% -- H.  DELETE EXPERIMENT on the clamp, INVERTED reading -- the OCR ambiguity
%        actually flown.  Coarse on purpose: the outcome is qualitative
%        (never closes) and a 250,000-step divergent run buys nothing.
Hc = setf(C, 'APN',1, 'XLAMFDEG',-90, 'XNCLIMG',15);
Hc.clamp_gt = false;  Hc.stop_on_tmax = true;
Hc.H = max(C.H, 1e-3);  Hc.HFINE = max(C.HFINE, 1e-4);  Hc.TMAX = 20;
CS(end+1) = mkcase('H', 'INVERTED clamp reading (if XNC < XNCLIM), coarse', Hc);

% -- I.  DELETE EXPERIMENT on the SHAPING TERM.  Set the goal to the angle the
%        geometry already has: XLAM - XLAMF is then 0, the shaping term
%        vanishes, and on a collision triangle XLAMD is 0 too, so the whole
%        command is 0 and the law must reproduce the PN straight line.  If it
%        does not, the shaping term is not the only thing steering.
XLAM0 = atan2(N.RT2IC-N.RM2IC, N.RT1IC-N.RM1IC);
CS(end+1) = mkcase('I', 'shaping with XLAMF = XLAM(0) -- shaping term deleted', ...
    setf(C, 'APN',1, 'XLAMFDEG', XLAM0*DEG));

% -- J.  STEP SENSITIVITY.  The nominal at 10x the coarse step.  This is the
%        row that licenses a caller to speed the whole board up, so it has to
%        be measured rather than asserted.
CS(end+1) = mkcase('J', 'nominal at 10x coarser step', ...
    setf(C, 'APN',1, 'XLAMFDEG',-90, 'H',C.H*10, 'HFINE',C.HFINE*10));
end


function c = mkcase(key, title, cfg)
cfg.label = key;
c = struct('key',key, 'title',title, 'cfg',cfg);
end


function C = setf(C, varargin)
%SETF  Copy C with fields overridden.  Name-value, checked -- a typo'd field
%   name would otherwise ADD a field that nothing reads and the case would run
%   at the book value while looking like it had been changed.
for i = 1:2:numel(varargin)
    f = varargin{i};
    if ~isfield(C, f)
        error('mgs:zarchan:shapingNonlinear:setField', ...
            ['"%s" is not a config field.  A typo here would silently leave ' ...
             'the case at its book value.'], f);
    end
    C.(f) = varargin{i+1};
end
end


function S = runCases(CS, GRAV_FT, DEG)
S = struct();
for i = 1:numel(CS)
    E = flyOne(CS(i).cfg, GRAV_FT, DEG);
    E.title = CS(i).title;
    S.(CS(i).key) = E;
end
end


% =============================================================================
%   THE INITIAL GEOMETRY.  Factored out of flyOne because the board needs to
%   evaluate it at HEDEG = 0 (the collision triangle) WITHOUT flying anything:
%   the book's "closing velocity is approximately 4000 ft/s" (:8435) is the
%   collision-triangle value, not the value with the -20 deg heading error in.
% =============================================================================
function g = initGeometry(C, DEG)
RM1 = C.RM1IC;  RM2 = C.RM2IC;  RT1 = C.RT1IC;  RT2 = C.RT2IC;
BETA = 0;                                   % the listing sets BETA=0. flat
VT1 = -C.VT*cos(BETA);  VT2 = C.VT*sin(BETA);
HE  = C.HEDEG/DEG;
RTM1 = RT1-RM1;  RTM2 = RT2-RM2;  RTM = sqrt(RTM1^2+RTM2^2);
if RTM <= 0
    error('mgs:zarchan:shapingNonlinear:coincident', ...
        'missile and target start at the same point; there is no engagement.');
end
XLAM = atan2(RTM2, RTM1);

% asin of an argument above 1 returns a COMPLEX number in MATLAB and says
% nothing about it.  Every state downstream then goes complex and the run
% finishes with a complex "miss distance" whose real part reads like an answer.
% engage2d.m refuses the same thing for the same reason.
sinL = C.VT*sin(BETA+XLAM)/C.VM;
if abs(sinL) > 1
    error('mgs:zarchan:shapingNonlinear:noLead', ...
        ['no collision triangle exists: the asin argument is %.4f (VT = %g, ' ...
         'VM = %g).  asin() would go COMPLEX and the run would not fail -- it ' ...
         'would return a complex miss distance.'], sinL, C.VT, C.VM);
end
XLEAD = asin(sinL);
THET  = XLAM + XLEAD;
VM1   = C.VM*cos(THET+HE);   VM2 = C.VM*sin(THET+HE);
VTM1  = VT1-VM1;             VTM2 = VT2-VM2;
VC    = -(RTM1*VTM1+RTM2*VTM2)/RTM;

g = struct('RM1',RM1,'RM2',RM2,'RT1',RT1,'RT2',RT2,'BETA',BETA, ...
           'VT1',VT1,'VT2',VT2,'HE',HE,'RTM1',RTM1,'RTM2',RTM2,'RTM',RTM, ...
           'XLAM',XLAM,'XLEAD',XLEAD,'THET',THET,'VM1',VM1,'VM2',VM2, ...
           'VTM1',VTM1,'VTM2',VTM2,'VC',VC);
end


% =============================================================================
%   LISTING 24.2 ITSELF
% =============================================================================
function E = flyOne(C, GRAV_FT, DEG)
%FLYONE  The listing, with the book's variable names kept so a side-by-side
%   diff against zarchan.txt:8358 stays readable.
%
%   ⚠️ GRAV_FT and DEG are ARGUMENTS, not inherited.  MATLAB local functions do
%      NOT see the main function's workspace; a previous wave put g = 32.2 in
%      the main function, read it from a local one, and it threw
%      "Unrecognized function or variable" on the first line of execution.

XNCLIM = GRAV_FT*C.XNCLIMG;      % XNCLIM=32.2*XNCLIMG
XLAMF  = C.XLAMFDEG/DEG;         % XLAMF=XLAMFDEG/57.3
XNT    = GRAV_FT*C.XNTG;         % XNT=32.2*XNTG
XNP    = C.XNP;
APN    = C.APN;
VT     = C.VT;
VM0    = C.VM;
CLAMP_GT = C.clamp_gt;           % see structuralCheck(); TRUE is the book

g0   = initGeometry(C, DEG);
RM1  = g0.RM1;  RM2  = g0.RM2;  RT1 = g0.RT1;  RT2 = g0.RT2;
BETA = g0.BETA; VT1  = g0.VT1;  VT2 = g0.VT2;
RTM1 = g0.RTM1; RTM2 = g0.RTM2; RTM = g0.RTM;
XLAM = g0.XLAM; VM1  = g0.VM1;  VM2 = g0.VM2;
VTM1 = g0.VTM1; VTM2 = g0.VTM2; VC  = g0.VC;
T = 0;  S = 0;

if VC <= 0
    error('mgs:zarchan:shapingNonlinear:opening', ...
        ['closing velocity is %.1f ft/s at t = 0, so "while VC >= 0" exits ' ...
         'immediately and the INITIAL separation (%.0f ft) would come back ' ...
         'labelled as a miss distance.'], VC, RTM);
end

% The opening command, straight from the ICs.  Reported separately from the
% logged history because it is INTEGRATOR-INDEPENDENT -- pure algebra -- which
% makes it the one row that can falsify the "more than 20 g at the beginning"
% claim without any argument about step size.  Reported UNCLAMPED as well as
% clamped, so a limit that is quietly eating the demand is visible.
XLAMD0 = (RTM1*VTM2-RTM2*VTM1)/(RTM*RTM);
TGO0   = RTM/VC;
if APN == 0
    XNC0raw = XNP*VC*XLAMD0;
    XNTPLOS0 = 0;
else
    XNTPLOS0 = -XNT*sin(BETA)*sin(XLAM) + XNT*cos(BETA)*cos(XLAM);
    XNC0raw  = 4.*VC*XLAMD0 + XNTPLOS0 + 2.*VC*(XLAM-XLAMF)/TGO0;
end
XNC0 = clampCmd(XNC0raw, XNCLIM, CLAMP_GT);

% Preallocation.  The listing grows ArrayT(n) inside the loop; at 0.1 s
% cadence over TMAX that reallocation is the single largest avoidable cost in
% a file that already runs ~10^5 steps per case.
NMAX = ceil(C.TMAX/0.1) + 10;
z = zeros(1, NMAX);
ArrayT = z; ArrayRT1 = z; ArrayRT2 = z; ArrayRM1 = z; ArrayRM2 = z;
ArrayXNCG = z; ArrayXLAMDEG = z; ArrayRTM = z; ArrayVC = z;
ArrayGAMDEG = z; ArrayVMAG = z; ArrayTGO = z; ArrayXNTPLOSG = z;
ArrayAtLim = false(1, NMAX);

% Explicit zero-init of the derivative-block outputs.  The listing relies on
% these existing by the time FLAG==1; being explicit stops a future edit from
% picking up a stale workspace variable of the same name.
AM1 = 0; AM2 = 0; BETAD = 0; XNC = XNC0; XNTPLOS = XNTPLOS0;
n = 0; nsteps = 0; PATH = 0; rtm_min = RTM; apogee = RM2;
diverged = false; tgo_flip = false;

% =========================== LISTING 24.2 ==================================
while VC >= 0
    if RTM < C.RFINE                       % if RTM , 1000  ->  RTM < 1000
        H = C.HFINE;                       % H=.00001
    else
        H = C.H;                           % H=.0001
    end

    BETAOLD=BETA; RT1OLD=RT1; RT2OLD=RT2; RM1OLD=RM1; RM2OLD=RM2;
    VM1OLD=VM1; VM2OLD=VM2;
    STEP=1; FLAG=0;

    % Two derivative evaluations per outer step: the first at the old state,
    % the second at the Euler-predicted state.  The book's own
    % predictor/trapezoidal-corrector spelling of RK2 (Heun), kept
    % structurally identical rather than refactored into something tidier.
    while STEP <= 1                        % while STEP ,=1  ->  STEP <= 1
        if FLAG==1
            STEP=2;
            BETA=BETA+H*BETAD; RT1=RT1+H*VT1; RT2=RT2+H*VT2;
            RM1=RM1+H*VM1; RM2=RM2+H*VM2;
            VM1=VM1+H*AM1; VM2=VM2+H*AM2; T=T+H;
        end
        RTM1=RT1-RM1; RTM2=RT2-RM2; RTM=sqrt(RTM1*RTM1+RTM2*RTM2);
        VTM1=VT1-VM1; VTM2=VT2-VM2;
        VC=-(RTM1*VTM1+RTM2*VTM2)/RTM;
        XLAM=atan2(RTM2,RTM1);
        % LOS rate ANALYTICALLY from the relative kinematics, never by
        % differencing XLAM: at H = 1e-5 the difference quotient would be
        % numerical noise of the same order as the signal, and the resulting
        % command would still look like a guidance command.
        XLAMD=(RTM1*VTM2-RTM2*VTM1)/(RTM*RTM);
        TGO=RTM/VC;
        if APN==0
            XNC=XNP*VC*XLAMD;              % plain PN, the comparison case
        else
            % zarchan.txt:8109 / the listing's own line.  The 4 and the 2 are
            % LITERALS -- XNP is not read here.  See header item (3).
            XNT1=XNT*sin(BETA); XNT2=XNT*cos(BETA);
            % MINUS on the first term -- the extracted prose at :8343 lost it;
            % the listing kept it and the listing is right (the command is
            % applied along (-sin XLAM, cos XLAM) three lines below).
            XNTPLOS=-XNT1*sin(XLAM)+XNT2*cos(XLAM);
            XNC=4.*VC*XLAMD+XNTPLOS+2.*VC*(XLAM-XLAMF)/TGO;
        end
        % OCR-DAMAGED PAIR, RESOLVED.  `if XNC.XNCLIM` -> `if XNC > XNCLIM`;
        % see the header for the three reasons and structuralCheck() for the
        % counterfactual that is actually flown.
        if CLAMP_GT
            if XNC > XNCLIM,  XNC = XNCLIM;  end
            if XNC < -XNCLIM, XNC = -XNCLIM; end
        else
            if XNC < XNCLIM,  XNC = XNCLIM;  end   % the inverted reading
            if XNC < -XNCLIM, XNC = -XNCLIM; end
        end
        % Perpendicular to the LINE OF SIGHT, not to the velocity vector.
        % Ch. 30 makes that a switch (ICHOICE); Ch. 24 does not.  Swapping
        % XLAM for the flight-path angle here is a DIFFERENT guidance law that
        % still flies and still hits, so it would not announce itself.
        AM1=-XNC*sin(XLAM); AM2=XNC*cos(XLAM);
        VT1=-VT*cos(BETA); VT2=VT*sin(BETA);
        if VT==0.
            BETAD=0.;      % zarchan.txt:8338 -- Ch.24's own divide-by-zero fix
        else
            BETAD=XNT/VT;
        end
        FLAG=1;
    end
    % DEAD IN THE LISTING TOO, AND KEPT ANYWAY.  The next outer iteration sets
    % STEP=1; FLAG=0; before anything reads FLAG, so this assignment can never
    % be observed -- Code Analyzer says so.  It is Zarchan's line and removing
    % it would make the side-by-side diff against zarchan.txt:8374 lie about
    % what the book does, which is a worse trade than one suppressed warning.
    FLAG=0; %#ok<NASGU>

    BETA=.5*(BETAOLD+BETA+H*BETAD);
    RT1=.5*(RT1OLD+RT1+H*VT1); RT2=.5*(RT2OLD+RT2+H*VT2);
    RM1=.5*(RM1OLD+RM1+H*VM1); RM2=.5*(RM2OLD+RM2+H*VM2);
    % NOTE the ORDER: RM is corrected using the PREDICTED VM, and only then is
    % VM corrected.  That is the listing's own order and it is load-bearing at
    % the last step -- see the tgo_flip note below.
    VM1=.5*(VM1OLD+VM1+H*AM1); VM2=.5*(VM2OLD+VM2+H*AM2);

    nsteps = nsteps + 1;
    if RTM < rtm_min, rtm_min = RTM; end
    if RM2 > apogee,  apogee  = RM2; end
    PATH = PATH + H*0.5*(sqrt(VM1OLD*VM1OLD+VM2OLD*VM2OLD) + ...
                         sqrt(VM1*VM1+VM2*VM2));

    S=S+H;
    if S >= .09999                          % if S.=.09999  ->  S >= .09999
        S=0.; n=n+1;
        if n > NMAX
            error('mgs:zarchan:shapingNonlinear:logOverflow', ...
                ['log overflow at t = %.2f s.  Raise TMAX, or the run is not ' ...
                 'terminating and the trajectory should not be trusted.'], T);
        end
        ArrayT(n)=T;
        ArrayRT1(n)=RT1; ArrayRT2(n)=RT2; ArrayRM1(n)=RM1; ArrayRM2(n)=RM2;
        ArrayXNCG(n)=XNC/32.2;              % the listing's own literal 32.2
        ArrayXLAMDEG(n)=XLAM*57.3;          % ... and its own literal 57.3
        ArrayRTM(n)=RTM; ArrayVC(n)=VC; ArrayTGO(n)=TGO;
        ArrayGAMDEG(n)=atan2(VM2,VM1)*57.3;
        ArrayVMAG(n)=sqrt(VM1*VM1+VM2*VM2);
        ArrayXNTPLOSG(n)=XNTPLOS/32.2;
        ArrayAtLim(n) = (XNC == XNCLIM) || (XNC == -XNCLIM);
    end

    % NOT IN THE BOOK.  A non-finite state here means TGO went through zero
    % inside the last derivative evaluation and the shaping term blew up; the
    % loop test would then compare NaN and exit for the wrong reason, or not
    % exit at all.  Detected rather than papered over: the listing's arithmetic
    % is not altered.
    if ~isfinite(RTM) || ~isfinite(VC)
        tgo_flip = true;
        break
    end
    if T > C.TMAX
        if C.stop_on_tmax
            diverged = true;               % the delete experiment wants this
            break                          % as DATA, not as an exception
        end
        error('mgs:zarchan:shapingNonlinear:noIntercept', ...
            ['no intercept within TMAX = %g s (VC still %.1f ft/s, separation ' ...
             '%.0f ft).  Refusing to report a miss distance for an engagement ' ...
             'that never closed.'], C.TMAX, VC, RTM);
    end
end
RTM = sqrt(RTM1*RTM1+RTM2*RTM2);   % the listing's bare `RTM` echo = the miss
% ========================= end of Listing 24.2 =============================

E = struct();
E.cfg = C;
E.diverged = diverged;
E.tgo_flip = tgo_flip;
E.nsteps = nsteps;
E.nlog = n;

if n == 0
    % Every summary below would otherwise be read off an empty array.
    error('mgs:zarchan:shapingNonlinear:noLog', ...
        ['the engagement ended at t = %.4f s, before the first 0.1 s log ' ...
         'point, so there is no time history at all.'], T);
end
k = 1:n;
E.T = ArrayT(k); E.RT1 = ArrayRT1(k); E.RT2 = ArrayRT2(k);
E.RM1 = ArrayRM1(k); E.RM2 = ArrayRM2(k);
E.XNCG = ArrayXNCG(k); E.XLAMDEG = ArrayXLAMDEG(k);
E.RTM = ArrayRTM(k); E.VC = ArrayVC(k); E.TGO = ArrayTGO(k);
E.GAMDEG = ArrayGAMDEG(k); E.VMAG = ArrayVMAG(k);
E.XNTPLOSG = ArrayXNTPLOSG(k); E.at_lim = ArrayAtLim(k);

E.tf_s      = T;
E.miss_ft   = RTM;        % separation at the first step PAST closest approach,
                          % at the PREDICTED state -- the corrector updates
                          % RM1/RM2 but does not refresh RTM, which is the
                          % listing's own behaviour and engage2d's too.
E.miss_min_ft = rtm_min;  % closest RTM actually seen.  Always <= miss_ft;
                          % quote miss_ft against the book, this one only as a
                          % resolution note.
E.h_quantum_ft = C.HFINE * abs(E.VC(end));   % the miss's own error bar.  Uses
                          % the last LOGGED VC, not the terminal VC: the loop
                          % exits on VC having just swept through zero, so the
                          % terminal value is worthless as a scale.
E.xnc0_g       = XNC0/GRAV_FT;
E.xnc0_raw_g   = XNC0raw/GRAV_FT;
E.tgo0_s       = TGO0;
E.vc0          = g0.VC;
E.xlam0_deg    = g0.XLAM*DEG;
E.gam0_deg     = atan2(g0.VM2, g0.VM1)*DEG;

% END-OF-FLIGHT VALUES ARE TAKEN FROM THE LAST LOGGED SAMPLE, NOT FROM THE
% LOOP EXIT, AND THAT IS NOT FUSSINESS.  At exit RTM is ~1e-2 ft and it is the
% MISS VECTOR, which at closest approach points roughly PERPENDICULAR to the
% closing velocity -- so atan2 of it can return an angle ~90 deg away from the
% sightline the book is talking about.  Fig. 24.18 plots the 0.1 s log, so the
% log is what the book's "-90 deg at the end of the flight" refers to.
E.xlam_end_deg = E.XLAMDEG(end);
E.gam_end_deg  = E.GAMDEG(end);
E.xnc_end_g    = E.XNCG(end);
E.xlam_exit_deg = XLAM*DEG;     % reported, never scored -- see above

% Extremes over the WHOLE log, and over the log OUTSIDE the terminal zone.
% RFINE is the listing's own marker for the numerically delicate region (it is
% where the book drops H by 10x), so "outside RFINE" is a criterion taken from
% the listing rather than invented here.  The two are reported side by side so
% that a terminal spike is visible instead of silently setting a maximum.
out = E.RTM >= C.RFINE;
E.xnc_max_g = max(E.XNCG);   E.xnc_min_g = min(E.XNCG);
if any(out)
    E.xnc_max_out_g = max(E.XNCG(out));
    E.xnc_min_out_g = min(E.XNCG(out));
else
    E.xnc_max_out_g = NaN;  E.xnc_min_out_g = NaN;
end
E.xlam_max_deg = max(E.XLAMDEG);  E.xlam_min_deg = min(E.XLAMDEG);
E.apogee_ft    = apogee;
E.path_ft      = PATH;
E.path_ratio   = PATH / g0.RTM;   % 1.000 = straight line to the target
E.vm_drift     = E.VMAG(end)/VM0 - 1;
E.at_lim_frac  = mean(E.at_lim);
E.inside_frac  = mean(~E.at_lim);
E.xntplos_max_g = max(abs(E.XNTPLOSG));
end


function y = clampCmd(x, lim, gt)
%CLAMPCMD  The listing's clamp pair, as a function, ONLY for the reported
%   t = 0 command.  The flight loop spells it out inline instead: a function
%   call per derivative evaluation is ~2e6 calls per board.
if gt
    if x > lim,  y = lim;  elseif x < -lim, y = -lim; else, y = x; end
else
    if x < lim,  y = lim;  elseif x < -lim, y = -lim; else, y = x; end
end
end


% =============================================================================
%   STRUCTURAL SELF-CHECKS.  Wiring invariants, NOT book values.  A failure
%   here voids every benchmark row, because it means the thing being scored is
%   not the thing the book describes.
% =============================================================================
function K = structuralCheck(S, N, DEG)
K = struct();
K.skipped = false;

% (1) On the collision triangle with a stationary target the missile velocity
%     lies exactly along the sightline, so VC(0) must equal VM EXACTLY.  If it
%     does not, XLEAD/THET/HE were composed wrongly and every case is wrong.
K.vc0_err = abs(S.A.vc0 - N.VM);
K.vc0_tol = 1e-9;
K.vc0_ok  = K.vc0_err <= K.vc0_tol;

% (2) ... and the LOS rate is then identically zero, which is what makes the
%     PN case a straight line.  This is the algebraic statement of :8382
%     ("does not require any acceleration ... already on a collision
%     triangle") and it is checked before the flight, not after it.
K.pn_xnc0_g = S.B.xnc0_g;
K.pn_xnc0_ok = abs(K.pn_xnc0_g) <= 1e-9;

% (3) DELETE EXPERIMENT ON THE SHAPING TERM.  Case I sets XLAMF = XLAM(0), so
%     (XLAM - XLAMF) starts at 0 and, XLAMD being 0 too, the command is 0 and
%     stays 0.  The shaped law must then be indistinguishable from the PN
%     straight line.  Anything else means something OTHER than the shaping
%     term is steering, and the loft on case A would be attributable to the
%     wrong thing.
%     ⚠️ THE COMMAND IS MEASURED OUTSIDE RFINE, and that is not slack.  On
%     this case the physical command is EXACTLY zero, so what is left is pure
%     round-off amplified by the law's own 1/TGO: the shaping term degrades to
%     2*VC^2*dlam/RTM, and with dlam ~ 4e-10 rad of accumulated integration
%     drift that is 7e-3/RTM ft/s^2.  At RTM = 1000 ft (the listing's own
%     fine-step boundary) it is 2e-7 g; at the 1 ft the last log point can
%     easily sit at, it is 2e-4 g.  A 1e-6 tolerance on the full log would
%     therefore fail at random depending on where the last sample landed --
%     a flaky row, which is worse than no row.
K.delete_shape_tf_err = abs(S.I.tf_s - S.B.tf_s);
K.delete_shape_xnc    = max(abs([S.I.xnc_max_out_g, S.I.xnc_min_out_g]));
K.delete_shape_tol    = 1e-6;
K.delete_shape_ok     = K.delete_shape_tf_err <= 1e-4 && ...
                        K.delete_shape_xnc    <= K.delete_shape_tol;

% (4) DELETE EXPERIMENT ON THE CLAMP -- both readings of the OCR-damaged line
%     actually flown.  Under the RESOLVED reading (case G) a 15 g limit sits
%     BELOW the 22 g opening demand and ABOVE the ~10 g terminal demand, so
%     the clamp must FIRE early and must later RELEASE.  Under the INVERTED
%     reading (case H) `if XNC < XNCLIM` raises every SUB-limit command TO
%     +XNCLIM, so the command is pinned positive for essentially every sample
%     and the missile arcs away instead of steering.  Those two signatures --
%     "fires and releases" versus "pinned" -- cannot be confused, which is the
%     point: the resolution is DEMONSTRATED, not argued.
K.clamp_fire_frac   = S.G.at_lim_frac;
K.clamp_inside_frac = S.G.inside_frac;
K.clamp_live_ok     = K.clamp_fire_frac > 0 && K.clamp_inside_frac > 0;
K.inverted_diverged = S.H.diverged;
K.inverted_miss     = S.H.miss_ft;
K.inverted_tf       = S.H.tf_s;
K.inverted_pinned   = mean(S.H.XNCG >= S.H.cfg.XNCLIMG - 1e-9);
% THE VERDICT IS "PINNED", NOT "DIVERGED".  Pinning is what the inverted
% reading DOES -- every sub-limit command raised to +XNCLIM -- and it is true
% by construction whatever the trajectory then does.  Whether the circling
% missile happens to sweep back past the target inside TMAX is luck, so it is
% reported (inverted_miss / inverted_tf) and not scored.  Scoring luck is how
% a delete experiment becomes a flaky row that people start ignoring.
K.inverted_ok       = K.inverted_pinned > 0.99;
K.clamp_ok          = K.clamp_live_ok && K.inverted_ok;

% (5) The book's limit is inert.  XNCLIMG = 9999999 must never be touched on
%     any scored case, or those cases are measuring the clamp and not the law.
scored = {'A','B','C','D','E','F'};
hit = false;
for i = 1:numel(scored)
    hit = hit || any(S.(scored{i}).at_lim);
end
K.limit_inert_ok = ~hit;

% (6) STEP SENSITIVITY.  The board's flight times are quoted to 0.1 s against
%     the book; if 10x the step moved them by more than that, the numbers
%     would be a property of the integrator.  Measured rather than asserted --
%     and it is this row that licenses passing a coarser 'H' to speed the
%     board up.
K.step_tf_shift = abs(S.J.tf_s - S.A.tf_s);
K.step_tol      = 0.01;
K.step_ok       = K.step_tf_shift <= K.step_tol;

% (7) THE INFERRED VT, CHECKED.  The three maneuvering cases need a target
%     speed the book never prints.  On the collision triangle (HEDEG = 0) the
%     inferred VT = 1000 gives the closing velocity the book DOES print,
%     "approximately 4000 ft/s" (:8435).  Evaluated with no simulation at all.
Dc = S.D.cfg;  Dc.HEDEG = 0;
gD = initGeometry(Dc, DEG);
K.vc_collision = gD.VC;
K.vc_collision_ok = abs(K.vc_collision/4000 - 1) <= 0.05;
K.vt_inferred = S.D.cfg.VT;

% (8) No case ended with a non-finite state (TGO through zero inside the last
%     derivative evaluation).  If one did, its end-of-flight numbers are the
%     product of a division by a closing velocity that had just changed sign.
K.tgo_flip_any = false;
for i = 1:numel(scored)
    K.tgo_flip_any = K.tgo_flip_any || S.(scored{i}).tgo_flip;
end
K.tgo_flip_ok = ~K.tgo_flip_any;

% (9) Speed drift, REPORTED not scored.  The command is perpendicular to the
%     sightline, not to the velocity, so it does work on the missile.  There
%     is no book value for it; it is here because "path length" below has to
%     be integrated rather than taken as VM*tF, and this is the number that
%     says by how much that matters.
K.vm_drift_A = S.A.vm_drift;
K.vm_drift_D = S.D.vm_drift;

% ⚠️⚠️ 這份清單**漏了兩條**:clamp_live_ok 與 inverted_ok。兩個都算出來了,
%   但沒進 names ⇒ 連 K.ok 都沒把它們算進去,更沒有上板。而它們正是這裡最
%   該被看見的兩條:clamp_live_ok 證明夾制真的**觸發過又釋放過**(不是惰性),
%   inverted_ok 是**刪除實驗** —— 反向的 OCR 讀法會把 >99% 的指令釘在上限。
%   跑了刪除實驗卻沒有人看得到結果,等於沒跑。
names = {'vc0_ok','pn_xnc0_ok','delete_shape_ok','clamp_live_ok', ...
         'inverted_ok','clamp_ok','limit_inert_ok','step_ok', ...
         'vc_collision_ok','tgo_flip_ok'};

% 上板用的人話名字。⚠️ 與 names **逐項對應**,順序不可分開改。
labels = {'closing velocity at t=0', 'PN command at t=0', ...
          'DELETE: shaping term removed', 'clamp fires and releases', ...
          'DELETE: inverted clamp reading pins the command', ...
          'clamp resolution', 'limit inert at the book value', ...
          'step response timing', 'no closing-velocity collision', ...
          'tgo never flips sign'};
assert(numel(labels) == numel(names), ...
    'mgs:zarchan:shapingNonlinear:labels', ...
    '⚠️ labels 與 names 長度不一致 ⇒ 板上的名字會對到別條檢查。');

% ⚠️ benchSuite 的 adaptGeneric 需要 .names/.oks 才會把結構檢查放上板。
%   以前 names 只是區域變數 ⇒ 這十條一條都沒上過板,而板面全綠。
K.names = labels;
K.oks   = logical(cellfun(@(n) K.(n), names));

K.ok = true;  K.first_fail = '';
for i = 1:numel(names)
    if ~K.(names{i})
        K.ok = false;
        if isempty(K.first_fail), K.first_fail = names{i}; end
    end
end
end


% =============================================================================
%   COMPUTED vs PUBLISHED
% =============================================================================
function B = benchmarks(S, K)
%BENCHMARKS  Every row carries the band, the reason for the band, and the
%   zarchan.txt line the book value came from.  Nothing here is read off a
%   figure by this port: Ch.24's figures all became image links.
B = emptyBench();

SRC_NOM  = 'p.590 nominal case (zarchan.txt:8353)';
SRC_PN   = 'p.594 results (zarchan.txt:8382)';
SRC_LOS  = 'p.594-596 (zarchan.txt:8383)';
SRC_TF   = 'p.596 (zarchan.txt:8395)';
SRC_SHRT = 'p.596 (zarchan.txt:8390)';
SRC_C30  = 'p.596-597 (zarchan.txt:8396, :8405)';
SRC_MAN  = 'p.597 (zarchan.txt:8406)';
SRC_MACC = 'p.597 (zarchan.txt:8418)';
SRC_M45  = 'p.598 (zarchan.txt:8435)';
SRC_M3G  = 'p.598 (zarchan.txt:8436)';
SRC_AX1  = 'Listing 24.2 own plot limits axis([0 14 -20 25]) (zarchan.txt:8380)';
SRC_AX2  = 'Listing 24.2 own plot limits axis([0 14 -100 0]) (zarchan.txt:8380)';

A = S.A;  Bp = S.B;  Cc = S.C;  D = S.D;  E = S.E;  F = S.F;

% -------------------------------------------------------------------------
% A.  THE NOMINAL, SHAPING LAW, XLAMF = -90 (the case Figs 24.16-24.18 show)
% -------------------------------------------------------------------------
B = addVal(B, 'A shaping: final LOS angle', -90, A.xlam_end_deg, 'text', ...
    SRC_LOS, 'deg', ...
    ['"the line of sight angle for trajectory shaping guidance achieved the ' ...
     'design goal by reaching -90 deg at the end of the flight".  Taken at ' ...
     'the LAST LOGGED SAMPLE, which is what Fig. 24.18 plots -- the loop-exit ' ...
     'value is the direction of a ~0.03 ft residual vector and is meaningless ' ...
     '(it is reported as an INFO row below so the difference is visible)']);

B = addVal(B, 'A shaping: flight time', 13.6, A.tf_s, 'text', SRC_TF, 's', ...
    ['"the amount of guidance time for trajectory shaping guidance is ' ...
     '13.6 s".  The 2% band is 0.27 s; the step-sensitivity self-check ' ...
     'measures the integrator''s own contribution at 10x the step']);

B = addBound(B, 'A shaping: opening command > 20 g', 'XNC(0) > 20 g', ...
    sprintf('%.2f g', A.xnc0_g), A.xnc0_g > 20, SRC_PN, 'g', ...
    ['"trajectory shaping guidance requires more than 20 g of acceleration ' ...
     'at the beginning of the flight".  ALGEBRAIC, not integrated: with ' ...
     'XLAMD(0) = 0 on the collision triangle the whole command is ' ...
     '2*VC*(XLAM-XLAMF)/TGO.  A wrong-signed shaping term (the bug ' ...
     'additiveLaw carried until 2026-08-22) gives ~0 here, not 22']);

B = addBound(B, 'A shaping: terminal command near -10 g', ...
    '-15 <= XNC(end) <= -5 g', sprintf('%.2f g', A.xnc_end_g), ...
    A.xnc_end_g <= -5 && A.xnc_end_g >= -15, SRC_PN, 'g', ...
    ['"nearly [-]10 g at the end of the flight" (the OCR prints "210 g"; the ' ...
     'minus is eaten throughout this chapter).  SCORED, even though it is an ' ...
     'END-of-flight value, on ONE ground: the command is NOT DIVERGING there. ' ...
     'The book''s closed form for this term (zarchan.txt:8392) is ' ...
     'nc = 2*Vc*lamF/tF*(1 - 3t/tF), which is FINITE at t = tF -- so the 0/0 ' ...
     'in 2*VC*(XLAM-XLAMF)/TGO resolves to a number rather than a spike.  ' ...
     'That is an argument about SHAPE only: the book itself says the formula''s ' ...
     'VALUE is "not very accurate in this example" (:8396), and its sign ' ...
     'convention for lamF is opposite to the listing''s, so it is NOT the ' ...
     'source of the -10.  The -10 is prose (:8382) and the band is +/-5 g ' ...
     'around a one-significant-figure claim.  Integration round-off reaches ' ...
     'this row as 2*VC^2*dlam/RTM ~ 7e-3/RTM ft/s^2, i.e. under 0.01 g unless ' ...
     'the last log point lands inside ~0.02 ft of intercept']);

B = addShape(B, 'A shaping: command starts + and ends -', ...
    'starts +, ends -', ...
    sprintf('%+.1f -> %+.1f g, %d crossings', A.XNCG(1), A.XNCG(end), ...
            nnz(diff(sign(A.XNCG)) ~= 0)), ...
    A.XNCG(1) > 0 && A.XNCG(end) < 0, ...
    SRC_PN, ...
    ['the shape behind "more than 20 g at the beginning ... [-]10 g at the ' ...
     'end".  This is the row that separates Zarchan''s law from the attractor ' ...
     'additiveLaw used to implement: a level-flight attractor on a collision ' ...
     'triangle commands ~0 and never crosses anything.  The crossing COUNT is ' ...
     'printed but NOT scored -- "exactly once" is this port''s reading of ' ...
     'Fig. 24.17, and the figure did not survive extraction']);

B = addBound(B, 'A shaping: hits near vertically', 'final GAM steeper than -75 deg', ...
    sprintf('%.2f deg', A.gam_end_deg), A.gam_end_deg <= -75, SRC_NOM, 'deg', ...
    ['"in this application we would like to hit the target vertically for ' ...
     'lethality reasons ... it indeed appears that the trajectory shaping ' ...
     'guidance law is enabling the missile to hit the target near ' ...
     'vertically".  Missile FLIGHT PATH angle, from the last logged sample']);

B = addBound(B, 'A shaping: trajectory is not the PN one', 'path/range > 1.10', ...
    sprintf('%.4f', A.path_ratio), A.path_ratio > 1.10, SRC_NOM, '-', ...
    ['"the trajectory shaping guidance law trajectory is entirely different ' ...
     'than the proportional navigation guidance trajectory".  Path length is ' ...
     'INTEGRATED, not VM*tF: the command is perpendicular to the sightline ' ...
     'rather than to the velocity, so |VM| drifts (see the INFO row)']);

B = addBound(B, 'A shaping: command inside the book''s own plot box', ...
    '-20 <= XNC <= 25 g', sprintf('[%.2f, %.2f] g', A.xnc_min_g, A.xnc_max_g), ...
    A.xnc_min_g >= -20 && A.xnc_max_g <= 25, SRC_AX1, 'g', ...
    ['from the LISTING, not from prose: the author set axis([0 14 -20 25]) ' ...
     'for his own data.  Weaker than a quoted number -- a plot CAN clip -- ' ...
     'which is why the band is the axis and nothing tighter.  A failure here ' ...
     'localises to the terminal sample; compare xnc_max_out_g']);

B = addBound(B, 'A shaping: LOS inside the book''s own plot box', ...
    '-100 <= XLAM <= 0 deg, tF <= 14 s', ...
    sprintf('[%.2f, %.2f] deg, tF %.2f s', A.xlam_min_deg, A.xlam_max_deg, A.tf_s), ...
    A.xlam_min_deg >= -100 && A.xlam_max_deg <= 0 && A.tf_s <= 14, ...
    SRC_AX2, 'deg', ...
    'same provenance: axis([0 14 -100 0]).  The 14 s abscissa also bounds tF');

B = addBound(B, 'A shaping: intercept (miss vs terminal quantum)', ...
    sprintf('miss < 10 x HFINE*VC = %.4f ft', 10*A.h_quantum_ft), ...
    sprintf('%.4f ft', A.miss_ft), A.miss_ft < 10*A.h_quantum_ft, SRC_NOM, 'ft', ...
    ['zero lag, no limit, stationary target: the miss here is numerical noise ' ...
     'by construction and its floor is HFINE*VC.  Quoted as a resolution ' ...
     'statement, never as a performance result']);

% -------------------------------------------------------------------------
% B.  THE COMPARISON, PLAIN PN (APN = 0)
% -------------------------------------------------------------------------
B = addVal(B, 'B PN: final LOS angle', -18.4, Bp.xlam_end_deg, 'text', ...
    SRC_LOS, 'deg', ...
    ['"proportional navigation ... ended up with a final line of sight angle ' ...
     'of -18.4 deg" (:8383, repeated at :8395).  With zero heading error ' ...
     'against a stationary target XLAMD is identically zero, so this is ' ...
     'atan2(-10000, 30000)*57.3 = -18.436 and it does not move all flight']);

B = addBound(B, 'B PN: no acceleration required', 'max |XNC| < 0.01 g', ...
    sprintf('%.3e g', max(abs(Bp.XNCG))), max(abs(Bp.XNCG)) < 0.01, ...
    SRC_PN, 'g', ...
    ['"proportional navigation ... does not require any acceleration to hit ' ...
     'the target in this scenario because it is already on a collision ' ...
     'triangle".  Exactly zero by algebra; the 0.01 g band is round-off ' ...
     'headroom, not slack']);

B = addBound(B, 'B PN: essentially a straight line', '|path/range - 1| < 1e-3', ...
    sprintf('%.6f', Bp.path_ratio), abs(Bp.path_ratio-1) < 1e-3, SRC_NOM, '-', ...
    '"the missile essentially travels in a straight line to the target"');

B = addShape(B, 'B PN: flight time shorter than shaping', 'tF(PN) < tF(shaping)', ...
    sprintf('%.3f < %.3f s', Bp.tf_s, A.tf_s), Bp.tf_s < A.tf_s, SRC_SHRT, ...
    ['"because proportional navigation enabled the missile to fly directly ' ...
     'to the target, the flight time was nearly 4 s shorter".  The ORDERING ' ...
     'is scored; the 4 s is an INFO row -- see its detail for why']);

B = addInfo(B, 'B PN: how much shorter (book says "nearly 4 s")', '~4 s', ...
    sprintf('%.3f s', A.tf_s - Bp.tf_s), SRC_SHRT, ...
    ['NOT SCORED.  The book''s own two numbers -- 13.6 s (:8395) and the ' ...
     'straight-line 31623/3000 = 10.54 s -- give 3.06 s, so "nearly 4" is the ' ...
     'author rounding his own figure generously.  A band holding both 3.06 ' ...
     'and 4.0 discriminates nothing the 13.6 s row does not already catch']);

% -------------------------------------------------------------------------
% C.  LESS SHAPING: XLAMF = -30, still a stationary target
% -------------------------------------------------------------------------
B = addVal(B, 'C shaping -30: flight time', 10.6, Cc.tf_s, 'text', SRC_C30, 's', ...
    ['"Because there is less trajectory shaping, the flight time reduces to ' ...
     '10.6 s" (:8396 continued at :8405)']);

B = addVal(B, 'C shaping -30: final LOS angle', -30, Cc.xlam_end_deg, 'text', ...
    SRC_C30, 'deg', ...
    ['the case is defined by its goal ("the desired final line of sight angle ' ...
     'was -30 deg") and the chapter closes on "the trajectory shaping ' ...
     'guidance law also works in the nonlinear world" (:8437)']);

B = addShape(B, 'C shaping -30: quicker than shaping to -90', 'tF(-30) < tF(-90)', ...
    sprintf('%.3f < %.3f s', Cc.tf_s, A.tf_s), Cc.tf_s < A.tf_s, SRC_C30, ...
    '"Because there is less trajectory shaping, the flight time reduces..."');

% -------------------------------------------------------------------------
% D/E/F.  MOVING AND MANEUVERING TARGET.
%   GATED on the inferred VT.  If the collision-triangle closing velocity does
%   not reproduce the book's "approximately 4000 ft/s" then VT = 1000 is the
%   wrong inference and these rows are measuring the wrong engagement -- they
%   are SKIPPED rather than allowed to cascade into four false failures.
% -------------------------------------------------------------------------
B = addVal(B, 'D/E/F: collision-triangle closing velocity', 4000, ...
    K.vc_collision, 'approx', SRC_M45, 'ft/s', ...
    ['"the closing velocity is approximately 4000 ft/s".  THIS ROW IS THE ' ...
     'GATE ON AN INFERENCE: the book never states VT for these cases and this ' ...
     'port assumes 1000 ft/s (its standing value since Ch.2).  Evaluated at ' ...
     'HEDEG = 0 because the book''s 4000 is the nominal closing velocity fed ' ...
     'to a LINEAR formula in which heading error is a separate input; the ' ...
     'actual VC(0) with the -20 deg error in is reported as INFO below']);

gateVT = '';
if ~K.vc_collision_ok
    gateVT = sprintf(['the inferred VT = %g ft/s does not reproduce the ' ...
                      'book''s "approximately 4000 ft/s" (got %.0f), so the ' ...
                      'geometry of these cases is not the book''s'], ...
                     K.vt_inferred, K.vc_collision);
end

% ⚠️⚠️ THE GATE MUST ALSO CHECK THE RANGE, AND ITS FIRST VERSION DID NOT.
%
%   Closing velocity alone does not pin the case down: tF = R0/Vc, so a
%   correct Vc with a wrong R0 passes the Vc gate and then fails the flight
%   time -- which is what happened.  The listing's own data statement has
%   RT1IC = 30000, RM2IC = 10000, i.e. R0 = 31,623 ft; at the book's Vc ~ 4000
%   that is 7.9 s, and at VM = 3000 against a stationary target 10.5 s.
%   The book says the D/E/F flights take 14.5 s and 11.4 s, i.e. R0 ~ 58,000
%   and ~46,000 ft.  ⇒ THE BOOK'S D/E/F CASES ARE NOT THE LISTING'S NOMINAL
%   GEOMETRY, and the book never publishes what they are.
%
%   That is an UNPUBLISHED INPUT, not a failed reproduction.  Scoring it would
%   score our guess at R0 against the author's, which is the same mistake as
%   scoring a number read off a diverging curve.  ⇒ SKIP, with the arithmetic
%   in the reason so nobody has to redo it.
%
%   ⚠️ Do NOT "fix" this by widening the 10% band to admit 35%.  A band that
%   admits 35% on a flight time stops detecting anything at all.
if isempty(gateVT)
    r0_book_d = 4000 * 14.5;
    % ⚠️ 直接用 listing 自己 data statement 上的值 —— 這一段沒有 C 在作用域裡,
    %    而這四個數是書上印出來的(RM1IC=0. RM2IC=10000. RT1IC=30000. RT2IC=0.),
    %    不是推斷的。抄它們不會引進新的假設。
    r0_listing = hypot(30000 - 0, 0 - 10000);
    if abs(r0_listing - r0_book_d) / r0_book_d > 0.15
        gateVT = sprintf(['the listing''s geometry gives R0 = %.0f ft, but the ' ...
            'book''s D/E/F flight times (14.5 s, 11.4 s) at its own "approximately ' ...
            '4000 ft/s" need R0 ~ %.0f ft and ~%.0f ft. The book never publishes ' ...
            'the geometry for those cases, so their flight times cannot be ' ...
            'reproduced from anything it prints -- an unpublished input, not a ' ...
            'failed reproduction'], r0_listing, r0_book_d, 4000 * 11.4);
    end
end

if isempty(gateVT)
    B = addVal(B, 'D 6 g, XLAMF -30: flight time', 14.5, D.tf_s, 'infer', ...
        SRC_M45, 's', ...
        ['"the flight time turns out to be 14.5 s".  Band 10% because one ' ...
         'input (VT) is inferred rather than published -- the row above is ' ...
         'what makes that inference testable']);
    B = addBound(B, 'D 6 g, XLAMF -30: intercept achieved', 'miss < 10 ft', ...
        sprintf('%.3f ft', D.miss_ft), D.miss_ft < 10, SRC_MAN, 'ft', ...
        '"when trajectory shaping guidance was used intercepts were achieved in both cases"');
    B = addVal(B, 'D 6 g, XLAMF -30: final LOS angle', -30, D.xlam_end_deg, ...
        'approx', SRC_MACC, 'deg', ...
        ['"the design goals for the final line of sight angles for both cases ' ...
         'were met".  5% band: against a maneuvering target the last 0.1 s of ' ...
         'sightline is moving fast']);

    B = addBound(B, 'E 6 g, XLAMF +30: intercept achieved', 'miss < 10 ft', ...
        sprintf('%.3f ft', E.miss_ft), E.miss_ft < 10, SRC_MAN, 'ft', ...
        'the second of the "both cases"');
    B = addVal(B, 'E 6 g, XLAMF +30: final LOS angle', 30, E.xlam_end_deg, ...
        'approx', SRC_MACC, 'deg', ...
        'the +30 deg goal -- opposite sign, so it also checks the term is signed');

    B = addShape(B, 'D vs E: -30 needs more positive g than +30', ...
        'max +XNC(-30) > max +XNC(+30)', ...
        sprintf('%.2f > %.2f g', D.xnc_max_out_g, E.xnc_max_out_g), ...
        D.xnc_max_out_g > E.xnc_max_out_g, SRC_MACC, ...
        ['"for the -30 deg intercept the maximum positive acceleration was ' ...
         '30 g ... for the 30-deg intercept the maximum positive acceleration ' ...
         'was only 10 g".  The ORDERING is scored, the levels are INFO.  ' ...
         'Maxima are taken OUTSIDE RFINE -- the listing''s own marker for the ' ...
         'delicate zone -- so a single terminal sample cannot set them']);
    B = addShape(B, 'D vs E: -30 needs more negative g than +30', ...
        'min XNC(-30) < min XNC(+30)', ...
        sprintf('%.2f < %.2f g', D.xnc_min_out_g, E.xnc_min_out_g), ...
        D.xnc_min_out_g < E.xnc_min_out_g, SRC_MACC, ...
        '"-10 g" against "-5 g", same sentence, same treatment');

    B = addInfo(B, 'D/E: Fig. 24.22 acceleration levels', ...
        '+30 / -10 g  and  +10 / -5 g', ...
        sprintf('D %+.1f/%+.1f   E %+.1f/%+.1f g', D.xnc_max_out_g, ...
                D.xnc_min_out_g, E.xnc_max_out_g, E.xnc_min_out_g), SRC_MACC, ...
        ['NOT SCORED as magnitudes: four one-significant-figure levels the ' ...
         'author read off Fig. 24.22, on cases whose VT is inferred.  The two ' ...
         'ORDERINGS between them are scored instead, which is the stronger ' ...
         'statement anyway']);

    B = addVal(B, 'F 3 g, XLAMF -30: flight time', 11.4, F.tf_s, 'infer', ...
        SRC_M3G, 's', ...
        ['"the maneuver level decreased to 3 g ... the time of flight reduced ' ...
         'to 11.4 s".  Same inferred VT, same 10% band']);
    B = addShape(B, 'F vs D: 3 g target closes sooner than 6 g', ...
        'tF(3 g) < tF(6 g)', sprintf('%.3f < %.3f s', F.tf_s, D.tf_s), ...
        F.tf_s < D.tf_s, SRC_M3G, ...
        ['"the time of flight REDUCED to 11.4 s".  This pair is also the ' ...
         'delete experiment on XNTPLOS: the only difference between D and F ' ...
         'is XNTG, so if the target-acceleration term were unwired these two ' ...
         'flights would be identical']);
else
    B = addSkip(B, 'D 6 g, XLAMF -30: flight time',      '14.5', gateVT, SRC_M45, 's');
    B = addSkip(B, 'D 6 g, XLAMF -30: intercept achieved','miss ~ 0', gateVT, SRC_MAN, 'ft');
    B = addSkip(B, 'D 6 g, XLAMF -30: final LOS angle',  '-30',  gateVT, SRC_MACC, 'deg');
    B = addSkip(B, 'E 6 g, XLAMF +30: intercept achieved','miss ~ 0', gateVT, SRC_MAN, 'ft');
    B = addSkip(B, 'E 6 g, XLAMF +30: final LOS angle',  '30',   gateVT, SRC_MACC, 'deg');
    B = addSkip(B, 'D vs E: acceleration orderings',     'see :8418', gateVT, SRC_MACC, 'g');
    B = addSkip(B, 'F 3 g, XLAMF -30: flight time',      '11.4', gateVT, SRC_M3G, 's');
    B = addSkip(B, 'F vs D: 3 g closes sooner',          'yes',  gateVT, SRC_M3G, 's');
end

% -------------------------------------------------------------------------
% REPORTED, NEVER SCORED
% -------------------------------------------------------------------------
B = addInfo(B, 'A: LOS at loop exit vs at last log point', ...
    'Fig. 24.18 plots the log', ...
    sprintf('log %.2f deg / exit %.2f deg (miss %.4f ft)', ...
            A.xlam_end_deg, A.xlam_exit_deg, A.miss_ft), SRC_LOS, ...
    ['NOT SCORED.  At loop exit RTM is the MISS VECTOR, which at closest ' ...
     'approach points roughly perpendicular to the closing velocity, so its ' ...
     'atan2 can sit ~90 deg from the sightline.  Scoring the exit value ' ...
     'instead of the log would be a benchmark measuring round-off geometry']);

B = addInfo(B, 'A: actual VC(0) with heading error in', 'n/a', ...
    sprintf('%.1f ft/s (D case: %.1f)', A.vc0, D.vc0), SRC_M45, ...
    'reported so the collision-triangle gate row above cannot be misread as this');

B = addInfo(B, 'A: apogee vs launch altitude', 'Fig. 24.16 (image link only)', ...
    sprintf('%.0f ft vs %.0f ft launch', A.apogee_ft, A.cfg.RM2IC), SRC_NOM, ...
    ['NOT SCORED.  Whether the shaped trajectory climbs ABOVE its launch ' ...
     'altitude or merely flattens is only visible on Fig. 24.16, which ' ...
     'survived extraction as an image link with no digits.  The scored form ' ...
     'of "entirely different trajectory" is the path-ratio row']);

B = addInfo(B, 'missile speed drift (accel perp. to LOS, not to V)', ...
    'not modelled by the book', ...
    sprintf('A %+.2f%%, D %+.2f%%', 100*K.vm_drift_A, 100*K.vm_drift_D), ...
    'structural', ...
    ['NOT SCORED -- there is no book value.  It is here because the command ' ...
     'is perpendicular to the SIGHTLINE, so it does work on the missile and ' ...
     '|VM| is not constant.  That is why path length is integrated above ' ...
     'rather than taken as VM*tF']);

B = addInfo(B, 'terminal-zone command extremes (inside vs outside RFINE)', ...
    'n/a', sprintf('A all %.2f/%.2f, outside %.2f/%.2f g', ...
                   A.xnc_max_g, A.xnc_min_g, A.xnc_max_out_g, A.xnc_min_out_g), ...
    'structural', ...
    ['NOT SCORED.  Printed so that a plot-box row going red is immediately ' ...
     'attributable to a terminal spike rather than to the law']);
end


% =============================================================================
%   RENDER
% =============================================================================
function s = render(R, C, offbook, ev)
W = 104;
L = {};
L{end+1} = 'Zarchan Listing 24.2 -- NONLINEAR ENGAGEMENT SIMULATION TO TEST TRAJECTORY SHAPING';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2, deg via 57.3.  NOTHING HERE IS SI. ***';
L{end+1} = '    (Listing 30.1 is the METRIC restatement of this same listing -- do not mix them)';
L{end+1} = sprintf(['law: XNC = 4*VC*XLAMD + XNTPLOS + 2*VC*(XLAM - XLAMF)/TGO   ' ...
                    '(zarchan.txt:8367; the 4 and the 2 are LITERALS)']);
L{end+1} = sprintf(['nominal: missile (%g, %g) ft at %g ft/s, target (%g, %g) ft at %g ft/s, ' ...
                    'XLAMF = %g deg'], ...
    C.RM1IC, C.RM2IC, C.VM, C.RT1IC, C.RT2IC, C.VT, C.XLAMFDEG);
L{end+1} = sprintf(['integrator: the book''s two-pass predictor/corrector (Heun), ' ...
                    'H = %g s, %g s inside RTM < %g ft'], C.H, C.HFINE, C.RFINE);
if isempty(C.integ_moved)
    L{end+1} = 'discretisation: the listing''s own (H = .0001 / .00001, RTM < 1000)';
else
    L{end+1} = sprintf(['discretisation: MOVED by the caller (%s) -- the step-sensitivity ' ...
                        'self-check below is what licenses that'], ...
        strjoin(sort(C.integ_moved), ', '));
end
if C.inconsistent_target
    L{end+1} = ['*** VT = 0 with XNTG ~= 0: the listing forces BETAD = 0 (zarchan.txt:8338), ' ...
                'so the target NEVER MOVES while XNTPLOS still feeds the law ***'];
end
if isempty(offbook)
    L{end+1} = 'configuration: BOOK NOMINAL (no model parameter moved)';
else
    L{end+1} = sprintf('configuration: OFF-BOOK -- caller moved %s', ...
        strjoin(sort(offbook), ', '));
end
L{end+1} = repmat('-', 1, W);

if ~C.benchmark
    r = R.run;
    L{end+1} = 'SINGLE ENGAGEMENT (benchmark = false).  NOTHING WAS CHECKED AGAINST THE BOOK.';
    L{end+1} = sprintf('  flight time %.4f s, miss %.4f ft (quantum %.4f ft), %d steps, %d log points', ...
        r.tf_s, r.miss_ft, r.h_quantum_ft, r.nsteps, r.nlog);
    L{end+1} = sprintf('  XNC(0) %.3f g (unclamped %.3f), final LOS %.3f deg, final GAM %.3f deg', ...
        r.xnc0_g, r.xnc0_raw_g, r.xlam_end_deg, r.gam_end_deg);
    L{end+1} = sprintf('  command range over the log [%.3f, %.3f] g; path/range %.4f; |VM| drift %+.2f%%', ...
        r.xnc_min_g, r.xnc_max_g, r.path_ratio, 100*r.vm_drift);
    L{end+1} = 'R.pass is FALSE and means "not checked", not "failed".';
    s = strjoin(L, newline);
    return
end

% ---- the cases ----------------------------------------------------------
K = R.selfcheck;
L{end+1} = 'THE BOOK''S CASES';
L{end+1} = sprintf('  %-3s %-52s %9s %10s %10s %9s', ...
    'key', 'case', 'tF [s]', 'LOS_f[deg]', 'miss [ft]', 'XNC0 [g]');
keys = {'A','B','C','D','E','F'};
for i = 1:numel(keys)
    e = R.cases.(keys{i});
    L{end+1} = sprintf('  %-3s %-52s %9.4f %10.3f %10.4f %9.3f', ...
        keys{i}, e.title, e.tf_s, e.xlam_end_deg, e.miss_ft, e.xnc0_g); %#ok<AGROW>
end
L{end+1} = repmat('-', 1, W);

% ---- structural ---------------------------------------------------------
L{end+1} = 'STRUCTURAL SELF-CHECK  (wiring, not the book -- a failure here voids everything above)';
L{end+1} = sprintf('  %-54s %-30s %s', 'VC(0) = VM exactly on the collision triangle', ...
    sprintf('err %.2e (tol %.0e)', K.vc0_err, K.vc0_tol), tf2s(K.vc0_ok));
L{end+1} = sprintf('  %-54s %-30s %s', 'PN command is algebraically zero at t = 0', ...
    sprintf('%.2e g', K.pn_xnc0_g), tf2s(K.pn_xnc0_ok));
L{end+1} = sprintf('  %-54s %-30s %s', 'DELETE EXPT: XLAMF = XLAM(0) reproduces PN', ...
    sprintf('dtF %.2e s, max|XNC| %.1e g', K.delete_shape_tf_err, K.delete_shape_xnc), ...
    tf2s(K.delete_shape_ok));
L{end+1} = sprintf('  %-54s %-30s %s', 'DELETE EXPT: clamp at 20 g fires AND releases', ...
    sprintf('%.3f at limit / %.3f inside', K.clamp_fire_frac, K.clamp_inside_frac), ...
    tf2s(K.clamp_live_ok));
L{end+1} = sprintf('  %-54s %-30s %s', 'DELETE EXPT: INVERTED clamp pins the command', ...
    sprintf('pinned %.3f (miss %.0f ft)', K.inverted_pinned, K.inverted_miss), ...
    tf2s(K.inverted_ok));
L{end+1} = '      ^ these two rows RESOLVE the OCR-damaged `if XNC.XNCLIM`.  Under the inverted';
L{end+1} = '        reading every sub-limit command is RAISED to +XNCLIM, so the command is pinned';
L{end+1} = '        positive for the whole flight and the missile arcs away.  Not an argument -- a run.';
L{end+1} = sprintf('  %-54s %-30s %s', 'book''s XNCLIMG = 9999999 never touched on A-F', ...
    'no scored case at the limit', tf2s(K.limit_inert_ok));
L{end+1} = sprintf('  %-54s %-30s %s', 'flight time insensitive to a 10x coarser step', ...
    sprintf('%.2e s (tol %.0e)', K.step_tf_shift, K.step_tol), tf2s(K.step_ok));
L{end+1} = sprintf('  %-54s %-30s %s', 'inferred VT reproduces the book''s VC ~ 4000 ft/s', ...
    sprintf('VT = %g -> %.1f ft/s', K.vt_inferred, K.vc_collision), ...
    tf2s(K.vc_collision_ok));
L{end+1} = sprintf('  %-54s %-30s %s', 'no case ended with TGO through zero', ...
    sprintf('any flip: %d', K.tgo_flip_any), tf2s(K.tgo_flip_ok));
L{end+1} = repmat('-', 1, W);

% ---- the board ----------------------------------------------------------
L{end+1} = 'COMPUTED vs PUBLISHED';
L{end+1} = sprintf('  %-46s %-24s %-24s %s', 'quantity', 'book', 'ours', 'verdict');
L{end+1} = repmat('-', 1, W);
for i = 1:numel(R.bench)
    b = R.bench(i);
    switch lower(b.kind)
    case 'info'
        v = 'INFO';
        bk = b.book_str;  ou = b.ours_str;
    case 'skip'
        v = 'SKIP';
        bk = b.book_str;  ou = '--';
    otherwise
        v = tf2s(b.ok);
        bk = strtrim([b.book_str ' ' b.units]);
        ou = strtrim([b.ours_str ' ' b.units]);
    end
    L{end+1} = sprintf('  %-46s %-24s %-24s %s', b.what, bk, ou, v); %#ok<AGROW>
end
L{end+1} = repmat('-', 1, W);
nev = sum(ev);
L{end+1} = sprintf('VERDICT: %d/%d evaluated book values inside tolerance; %d not scored.', ...
    sum([R.bench(ev).ok]), nev, numel(R.bench) - nev);
L{end+1} = '';
L{end+1} = 'HOW THIS PORT AND mgs.zarchan.additiveLaw DISAGREE ABOUT THE LAW''S FORM:';
L{end+1} = '  (1) Zarchan scales BOTH terms with VC (closing velocity) and uses TGO = RTM/VC;';
L{end+1} = '      additiveLaw uses V (missile speed) and t_go = R/V.  Same ratio between the two';
L{end+1} = '      terms, whole demand scaled by (V/VC)^2 -- which diverges exactly during a loft.';
L{end+1} = '  (2) Ch.24 calls the setpoint XLAMF, a final LINE OF SIGHT angle.  Ch.30 calls the';
L{end+1} = '      same slot GAMF, a final FLIGHT PATH angle, and additiveLaw took its comment from';
L{end+1} = '      Ch.30.  Those coincide ONLY for a stationary target.';
L{end+1} = '  (3) The 4 and the 2 are LITERALS in this listing -- XNP is read only by the PN';
L{end+1} = '      branch.  additiveLaw''s "shape_gain 2.0 paired with N''=4" understates it: the';
L{end+1} = '      book does not expose them as gains at all (zarchan.txt:8111).';
L{end+1} = '  (4) Zarchan carries + XNTPLOS with coefficient exactly 1; additiveLaw has no target-';
L{end+1} = '      acceleration term.  Equivalent against a stationary target, a missing input';
L{end+1} = '      against anything else.';
L{end+1} = '';
L{end+1} = 'NOT MODELLED: gravity, drag, thrust, autopilot lag, seeker, ground.  With HEDEG = -20';
L{end+1} = 'the missile''s altitude goes NEGATIVE partway through cases D-F and neither this file';
L{end+1} = 'nor the book cares.  A miss distance from here is not a sortie prediction.';
s = strjoin(L, newline);
end


function drawCurves(R, C)
%DRAWCURVES  The listing's own three figures, for the nominal case, with the
%   PN comparison overlaid the way Figs 24.16-24.18 present it.
if ~C.benchmark
    e = R.run;
    figure; plot(e.RT1/1000, e.RT2/1000, e.RM1/1000, e.RM2/1000); grid on
    title('Listing 24.2 -- Engagement Geometry');
    xlabel('Downrange (Kft)'); ylabel('Altitude (Kft)'); legend('target','missile');
    figure; plot(e.T, e.XNCG); grid on
    title('Listing 24.2 -- Commanded Acceleration');
    xlabel('Time (Sec)'); ylabel('XNC (G)');
    figure; plot(e.T, e.XLAMDEG); grid on
    title('Listing 24.2 -- Line-of-Sight Angle');
    xlabel('Time (Sec)'); ylabel('XLAM (Deg)');
    return
end
A = R.cases.A;  B = R.cases.B;
figure; plot(A.RT1/1000, A.RT2/1000, A.RM1/1000, A.RM2/1000, ...
             B.RM1/1000, B.RM2/1000); grid on
title('Fig. 24.16 -- Trajectory shaping also works in the nonlinear world');
xlabel('Downrange (Kft)'); ylabel('Altitude (Kft)');
legend('target','missile: shaping','missile: PN');
figure; plot(A.T, A.XNCG, B.T, B.XNCG); grid on
title('Fig. 24.17 -- Commanded Acceleration');
xlabel('Time (Sec)'); ylabel('XNC (G)'); axis([0 14 -20 25]);
legend('shaping','PN');
figure; plot(A.T, A.XLAMDEG, B.T, B.XLAMDEG); grid on
title('Fig. 24.18 -- Line-of-Sight Angle');
xlabel('Time (Sec)'); ylabel('XLAM (Deg)'); axis([0 14 -100 0]);
legend('shaping','PN');
end


% =============================================================================
%   BENCH ROW HELPERS.  Schema matches what mgs.zarchan.benchSuite consumes
%   through its shared benchRows() adapter: what / kind / book / book_str /
%   ours / ours_str / pct / band / tol / ok / skipped / why / source / units /
%   detail.  Do not rename a field here without changing that adapter.
% =============================================================================
function B = emptyBench()
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end


function B = addVal(B, what, book, ours, band, source, units, detail)
i = numel(B) + 1;
tol = bandTol(band);
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
case 'exact',  t = 1e-12;  % an identity, or a literal echoed from the listing
case 'text',   t = 0.02;   % printed in prose to 2-3 significant figures
case 'approx', t = 0.05;   % the book says "approximately"
case 'infer',  t = 0.10;   % published value, but one INPUT is inferred here;
                           % the inference has its own gating row on the board
case 'graph',  t = 0.15;   % the author read it off a figure's ordinate
otherwise
    error('mgs:zarchan:shapingNonlinear:band', ...
        ['unknown tolerance band "%s".  Bands encode the PROVENANCE of the ' ...
         'book value; inventing one silently sets an arbitrary pass ' ...
         'criterion.'], band);
end
end


function s = tf2s(b)
if b, s = 'PASS'; else, s = 'FAIL'; end
end
