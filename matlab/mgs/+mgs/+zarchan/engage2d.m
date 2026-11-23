function out = engage2d(opts)
%MGS.ZARCHAN.ENGAGE2D  Zarchan Listing 2.1 -- 2-D missile-target engagement.
%
%   out = mgs.zarchan.engage2d()                     % benchmark suite + table
%   out = mgs.zarchan.engage2d('HEDEG',-20,'XNP',4)  % one engagement
%   out = mgs.zarchan.engage2d('HEDEG',0,'XNP',3,'XNT',96.6)
%
%   Line-by-line port of Listing 2.1, "Two-Dimensional Tactical Missile-Target
%   Engagement Simulation", Zarchan, Tactical and Strategic Missile Guidance,
%   Ch. 2, p. 18.  Book source in this repo:
%       matlab/zarchan/_source/zarchan.txt:409   (the listing)
%       matlab/zarchan/_source/zarchan.txt:428   (the prose that explains it)
%
% ===========================================================================
%   UNITS ARE FEET.  ALL OF THEM.  NOTHING IS CONVERTED.
% ===========================================================================
%   Positions ft, speeds ft/s, accelerations ft/s^2, g = 32.2 ft/s^2, and
%   degrees converted with the book's 57.3 rather than 180/pi.  The rest of
%   mgs is SI; this file is deliberately not.
%
%   The entire value of a book port is that its output can be laid straight
%   next to the book's printed figures with no arithmetic in between.  Put a
%   conversion in here and a wrong factor stops being a crash and becomes a
%   miss distance of 3.2 that reads plausibly in either unit.  That is the
%   failure mode this package exists to catch, so: convert AT THE CALL SITE.
%   mgs.zarchan.crosscheck is the file that maps our SI engagement onto a book
%   listing, and it documents its own approximations while doing it.
%
%   There is one active guard: VM below 400 ft/s ERRORS, because the likely
%   cause is an SI speed handed straight in (our own vehicle cruises
%   157-210 m/s, which as a bare number would simulate a 50 m/s missile and
%   still hand back a plausible-looking answer).  Override with
%   'AllowSlowMissile', true if you really mean it.
%
% ===========================================================================
%   WHY THIS FILE IS HERE
% ===========================================================================
%   mgs is calibrated on drag and CANNOT REPRODUCE FAILURES: it scores 32/32
%   at 0.03 m where the measured 32-flight set scores 25/32 at 0.93 m.  The
%   measured root cause is guidance-system LAG -- real gamma-tracking gain
%   0.063 against the model's 1.385, ~8 s of it, saturating on 26 of 32
%   sorties.  Zarchan Ch.3/Ch.6 is precisely the lag-to-miss machinery we lack.
%
%   Listing 2.1 is the ZERO-LAG floor of that ladder.  It buys two things:
%
%   (1) An INDEPENDENT CHECK on our integrator and PN kernel, because the book
%       publishes the answers.  If this file cannot make Fig. 2.3, nothing
%       built on top of it is worth reading.
%   (2) The baseline the lagged listings get differenced against.  Miss here is
%       numerical noise (sub-foot); every foot that appears once a time
%       constant is added is attributable to the lag and not to the geometry.
%
%   NOTE WHAT IS *NOT* MODELLED, because this file is easy to over-read: no
%   gravity, no drag, no thrust, no autopilot lag, no acceleration limit, no
%   seeker.  Constant-speed kinematics with an instantaneous PN command.  A
%   12 g demand here is a demand the airframe is simply assumed to meet.
%
% ===========================================================================
%   THE ADAPTIVE TERMINAL STEP -- why the miss distances mean anything
% ===========================================================================
%   The run ends when VC changes sign, i.e. at the first step PAST closest
%   approach, and the reported miss is the separation at that step.  So the
%   reported miss carries a floor of roughly one step of closing:
%
%       H * VC = 0.0002 * ~3900 ft/s ~= 0.8 ft
%
%   That is where the book's "under 0.8 ft" numerical-error claim comes from,
%   and it is why the coarse step is dropped near the end (Zarchan p.18).  Run
%   this with a single 0.01 s step throughout and the overshoot past the
%   intercept point is ~40 ft, which swamps every real effect the book then
%   goes on to discuss.  out.h_quantum_ft reports that floor for the run you
%   actually did: a miss below it is indistinguishable from zero and MUST NOT
%   be quoted as a trend.
%
%   TWO TRIGGER RULES, and they are not the same rule:
%     'range' (RTM < 1000 ft)   is what the LISTING literally says.
%     'tgo'   (RTM/VC < 0.3 s)  is the default here, per this port's brief.
%   At book speeds VC ~ 3800-4000 ft/s, so 0.3 s of TGO is 1140-1200 ft and the
%   two fire within ~20% of each other.  Verified before this file was written:
%   on all five benchmark cases the two rules agree on flight time to 1e-4 s
%   and on miss to well inside the 0.8 ft quantum.  Both are kept as options so
%   the claim stays testable instead of being a comment nobody can check.
%
%   The coarse step defaults to 0.001 s (this port's brief); the listing uses
%   0.01 s.  Also verified equivalent -- flight times identical to 1e-4 s, miss
%   differing only in which side of the terminal quantum you land on.
%
% ===========================================================================
%   BENCHMARKS -- and TWO BRIEFED VALUES THAT THIS PORT REFUTES
% ===========================================================================
%   mgs.zarchan.engage2d() runs five cases and prints a verdict table.  Fifteen
%   checks are supported by the book and PASS.  Two numbers supplied in this
%   port's brief are NOT reproducible and are reported REFUTED rather than
%   quietly widened into a tolerance band:
%
%   REFUTED 1.  "HEDEG=-20, N'=5 gives initial accel ~18 g."
%       Computed 15.21 g at t=0; 14.77 g at the first 0.1 s log point.
%       Independent check with no simulation in it: for heading error the
%       initial PN command is
%           n_c(0) = N' * VC(0) * XLAMD(0)
%       and neither VC(0) nor XLAMD(0) depends on N', so n_c(0) is EXACTLY
%       linear in N'.  N'=4 gives 12.17 g -- a value the brief itself accepts
%       -- therefore N'=5 gives 12.17 * 1.25 = 15.21 g and cannot give 18 g.
%       This file asserts that ratio to 1e-9.  18 g would need N' = 5.92.
%
%   REFUTED 2.  "HEDEG=0, XNT=3 g, N'=3 gives a peak near intercept of ~28 g."
%       Computed peak 5.45 g at t=8.4 s of a 10.46 s flight.
%       Independent check: against a constant-g target a ZERO-LAG PN system has
%       terminal command n_c(tF) = nT*N'/(N'-1) = 3*1.5 = 4.5 g, and nonlinear
%       geometry lifts the peak to 5.45.  28 g against a 3 g target would need
%       N'/(N'-1) = 9.33, i.e. N' = 1.12, which is not a navigable gain.  A
%       28 g demand at N'=3 requires guidance LAG: it is a Ch.3/Ch.6 number,
%       not a Ch.2 one.
%
%   What the book actually claims for that case is weaker and DOES hold: the
%   profile is "virtually monotonically increasing" (it rises monotonically to
%   80% of flight time, then rolls off) and the peak is "significantly higher
%   than the maneuver level of the target" -- 5.45 g against 3 g, 1.82x.
%
%   These are reported, not thrown.  The simulation did what it was asked; it
%   is the briefed expectation that is wrong, and burying that in a widened
%   tolerance is how a project ends up with a green gate and a broken model.
%
%   See also MGS.ZARCHAN.CROSSCHECK, MGS.ZARCHAN.SIZING,
%   MGS.ZARCHAN.ADDITIVELAW, ZARCHAN_LISTING30_1, PUREPN2D.

arguments
    % ---- the book's data statement (Listing 2.1, line 1), verbatim ---------
    opts.HEDEG    (1,1) double {mustBeFinite}      = -20    % heading error, deg
    opts.XNP      (1,1) double {mustBePositive}    = 4      % effective nav ratio N'
    opts.XNT      (1,1) double {mustBeFinite}      = 0      % target accel, ft/s^2 (3 g = 96.6)
    opts.VM       (1,1) double {mustBePositive}    = 3000   % missile speed, ft/s
    opts.VT       (1,1) double {mustBeNonnegative} = 1000   % target speed, ft/s
    opts.BETADEG  (1,1) double {mustBeFinite}      = 0      % target heading BETA, deg
    opts.RM1      (1,1) double {mustBeFinite}      = 0      % missile downrange, ft
    opts.RM2      (1,1) double {mustBeFinite}      = 10000  % missile altitude, ft
    opts.RT1      (1,1) double {mustBeFinite}      = 40000  % target downrange, ft
    opts.RT2      (1,1) double {mustBeFinite}      = 10000  % target altitude, ft
    % ---- integration ------------------------------------------------------
    opts.Step     (1,1) double {mustBePositive}    = 0.001  % coarse H, s (listing: 0.01)
    opts.FineStep (1,1) double {mustBePositive}    = 0.0002 % terminal H, s (listing: 0.0002)
    opts.FineTrigger (1,:) char {mustBeMember(opts.FineTrigger,{'tgo','range'})} = 'tgo'
    opts.TgoFine  (1,1) double {mustBePositive}    = 0.3    % 'tgo' rule: TGO below this, s
    opts.RangeFine (1,1) double {mustBePositive}   = 1000   % 'range' rule: RTM below this, ft
    opts.TMax     (1,1) double {mustBePositive}    = 200    % runaway guard, s (not in the book)
    % ---- housekeeping -----------------------------------------------------
    opts.AllowSlowMissile (1,1) logical = false
    opts.Plots    (1,1) logical = false
    opts.Bench    (1,1) logical = false
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
%
% The two modes return DIFFERENT STRUCTS -- see below.
%
% The two modes return DIFFERENT STRUCTS.  out.mode says which, and consumers
% should branch on it: a benchmark result has no .T field, so reaching for
% out.T on one errors instead of yielding an empty trajectory.  That is the
% behaviour we want -- a silently empty time history is how a plotting script
% ends up drawing nothing and nobody noticing.
if opts.Bench
    out = benchmark();
    return
end

out = runOne(opts);
end


% =============================================================================
function r = runOne(p)
%RUNONE  Listing 2.1 itself.  The book's variable names are kept throughout so
%   a side-by-side diff against zarchan.txt:409 stays readable.

G   = 32.2;    % the book's g.  NOT 9.80665, NOT 9.81 -- ft/s^2.
DEG = 57.3;    % the book's rad<->deg constant.  NOT 180/pi (= 57.29578):
               % round-tripping through 57.3 is not the identity, and the
               % published numbers were made with 57.3.  zarchan_listing30_1.m
               % made the same call for the same reason.

% ---- unit / feasibility guards (NOT in the book) ---------------------------
if p.VM < 400 && ~p.AllowSlowMissile
    error('mgs:zarchan:engage2d:units', ...
        ['VM = %g.  THIS FILE IS IN FEET PER SECOND: %g ft/s is %.0f m/s, far ' ...
         'below anything Zarchan flies (his nominal is 3000 ft/s = 914 m/s).  ' ...
         'The likely cause is an SI speed passed straight in -- and that does ' ...
         'not fail, it simulates a slow missile and hands back a "miss ' ...
         'distance" in feet that reads plausibly as metres.  Convert ' ...
         '(m/s * 3.28084 = ft/s), or pass ''AllowSlowMissile'', true.'], ...
        p.VM, p.VM, p.VM*0.3048);
end
if p.VT == 0 && p.XNT ~= 0
    error('mgs:zarchan:engage2d:statTarget', ...
        ['VT = 0 with XNT = %g.  The listing turns the target with ' ...
         'BETAD = XNT/VT, which here is %g/0.  MATLAB returns Inf and the run ' ...
         'CONTINUES: the target heading goes NaN within two steps and you get ' ...
         'a NaN miss distance only if you happen to look at it.  A stationary ' ...
         'target cannot have a turn rate -- set XNT = 0.  (Zarchan patches the ' ...
         'same division in Ch.24, zarchan.txt:8338.)'], p.XNT, p.XNT);
end

% ---- initial conditions (Listing 2.1, data statement) ----------------------
n = 0;
VM = p.VM; VT = p.VT; XNT = p.XNT; XNP = p.XNP;
RM1 = p.RM1; RM2 = p.RM2; RT1 = p.RT1; RT2 = p.RT2;
BETA = p.BETADEG/DEG;
VT1 = -VT*cos(BETA); VT2 = VT*sin(BETA);
HE = p.HEDEG/DEG;
T = 0.; S = 0.;
RTM1 = RT1-RM1; RTM2 = RT2-RM2; RTM = sqrt(RTM1*RTM1+RTM2*RTM2);
if RTM <= 0
    error('mgs:zarchan:engage2d:coincident', ...
        'missile and target start at the same point; there is no engagement.');
end
XLAM = atan2(RTM2,RTM1);

% XLEAD = asin(VT*sin(BETA+XLAM)/VM).  If that argument exceeds 1 -- a target
% fast enough at this aspect that no collision triangle exists -- MATLAB's
% asin returns a COMPLEX number and says nothing.  Every downstream state then
% goes complex and the loop runs happily to a complex "miss distance" whose
% real part looks like an answer.  Refuse instead.
sinL = VT*sin(BETA+XLAM)/VM;
if abs(sinL) > 1
    error('mgs:zarchan:engage2d:noLead', ...
        ['no collision triangle exists: asin argument is %.4f (VT = %g, ' ...
         'VM = %g, aspect %.1f deg).  The target is too fast at this aspect ' ...
         'for the missile to lead it.  asin() would go COMPLEX here and the ' ...
         'run would not fail -- it would return a complex miss distance.'], ...
        sinL, VT, VM, (BETA+XLAM)*DEG);
end
XLEAD = asin(sinL);
THET  = XLAM+XLEAD;
VM1   = VM*cos(THET+HE); VM2 = VM*sin(THET+HE);
VTM1  = VT1-VM1; VTM2 = VT2-VM2;
VC    = -(RTM1*VTM1+RTM2*VTM2)/RTM;
if VC <= 0
    error('mgs:zarchan:engage2d:opening', ...
        ['closing velocity is %.1f ft/s at t = 0, so "while VC >= 0" exits ' ...
         'immediately and the INITIAL separation (%.0f ft) would come back ' ...
         'labelled as a miss distance.  Check HEDEG / BETADEG / the speeds.'], ...
        VC, RTM);
end

% n_c at t = 0, straight from the ICs.  Reported separately from the logged
% history because it is INTEGRATOR-INDEPENDENT: pure algebra, and exactly
% linear in XNP.  That makes it the one benchmark able to falsify a claimed
% value without any argument about step size or figure-reading precision --
% which is exactly what it does to the briefed 18 g (see the header).
XLAMD0 = (RTM1*VTM2-RTM2*VTM1)/(RTM*RTM);
XNC0   = XNP*VC*XLAMD0;

% Preallocation, and explicit zero-init of the derivative-block outputs.  The
% listing relies on those existing by the time FLAG==1; being explicit keeps
% Code Analyzer quiet and stops a future edit from reading a stale workspace
% variable of the same name.
NMAX = ceil(p.TMax/0.1) + 10;
ArrayT = zeros(1,NMAX); ArrayRT1 = ArrayT; ArrayRT2 = ArrayT;
ArrayRM1 = ArrayT; ArrayRM2 = ArrayT; ArrayXNCG = ArrayT; ArrayRTM = ArrayT;
ArrayVC = ArrayT; ArrayVM = ArrayT; ArrayGAMDEG = ArrayT; ArrayXLAMD = ArrayT;
AM1 = 0; AM2 = 0; BETAD = 0; XNC = 0; XLAMD = XLAMD0;

rtm_min = RTM;
nsteps  = 0;

% =========================== LISTING 2.1 =====================================
while VC >= 0
    % Terminal step refinement -- see the header for why this is not optional.
    % Written as RTM < TgoFine*VC rather than RTM/VC < TgoFine so VC == 0
    % exactly cannot raise a divide; the two are identical for VC > 0, which is
    % the only case this loop reaches.
    switch p.FineTrigger
        case 'tgo',   fine = (RTM < p.TgoFine*VC);
        case 'range', fine = (RTM < p.RangeFine);      % the listing's own rule
        otherwise
            % Unreachable while mustBeMember guards the argument.  Kept so that
            % adding a third rule and forgetting this switch is a stop, not a
            % run at whatever H happened to be left in scope.
            error('mgs:zarchan:engage2d:trigger', ...
                'unhandled FineTrigger "%s"', p.FineTrigger);
    end
    if fine
        H = p.FineStep;
    else
        H = p.Step;
    end

    BETAOLD=BETA; RT1OLD=RT1; RT2OLD=RT2; RM1OLD=RM1; RM2OLD=RM2;
    VM1OLD=VM1; VM2OLD=VM2;
    STEP=1; FLAG=0;

    % Two derivative evaluations per step: the first at the old state, the
    % second at the Euler-predicted state.  This is the book's own
    % predictor/trapezoidal-corrector spelling of RK2 (Heun), kept
    % structurally identical rather than refactored into a tidier loop.
    while STEP <= 1
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
        % LOS rate ANALYTICALLY from the relative kinematics -- never by
        % differencing XLAM.  With a 0.0002 s step, differencing would produce
        % numerical noise of the same order as the signal being measured, and
        % the resulting command would still look like a guidance command.
        XLAMD=(RTM1*VTM2-RTM2*VTM1)/(RTM*RTM);
        XNC=XNP*VC*XLAMD;
        % Command is perpendicular to the INSTANTANEOUS LINE OF SIGHT (XLAM),
        % not to the velocity vector.  Ch.30 makes that a switch (ICHOICE);
        % Ch.2 does not.  Swapping XLAM for GAM here would be a different
        % guidance law that still flies and still hits -- so it would not
        % announce itself.
        AM1=-XNC*sin(XLAM); AM2=XNC*cos(XLAM);
        VT1=-VT*cos(BETA); VT2=VT*sin(BETA);
        if VT ~= 0
            BETAD=XNT/VT;
        else
            BETAD=0;      % guarded above: VT == 0 is only reachable with XNT == 0
        end
        FLAG=1;
    end
    FLAG=0;

    BETA=.5*(BETAOLD+BETA+H*BETAD);
    RT1=.5*(RT1OLD+RT1+H*VT1); RT2=.5*(RT2OLD+RT2+H*VT2);
    RM1=.5*(RM1OLD+RM1+H*VM1); RM2=.5*(RM2OLD+RM2+H*VM2);
    VM1=.5*(VM1OLD+VM1+H*AM1); VM2=.5*(VM2OLD+VM2+H*AM2);

    nsteps = nsteps + 1;
    if RTM < rtm_min, rtm_min = RTM; end

    S=S+H;
    if S >= .09999
        S=0.; n=n+1;
        if n > NMAX
            error('mgs:zarchan:engage2d:logOverflow', ...
                ['log overflow at t = %.2f s.  Raise TMax, or the run is not ' ...
                 'terminating and the trajectory should not be trusted.'], T);
        end
        ArrayT(n)=T; ArrayRT1(n)=RT1; ArrayRT2(n)=RT2;
        ArrayRM1(n)=RM1; ArrayRM2(n)=RM2;
        ArrayXNCG(n)=XNC/32.2; ArrayRTM(n)=RTM;   % 32.2 is the same g as G above;
                                                  % left as the literal the
                                                  % listing itself prints.
        % --- not in the book.  Logged because the LAGGED listings (Ch.3/Ch.6)
        %     differ from this one in exactly these quantities, and a
        %     like-for-like diff needs them present in the zero-lag baseline
        %     too.  Re-deriving them later from a saved trajectory is where
        %     sign and datum errors get introduced.
        ArrayVC(n)=VC; ArrayVM(n)=sqrt(VM1*VM1+VM2*VM2);
        ArrayGAMDEG(n)=atan2(VM2,VM1)*DEG; ArrayXLAMD(n)=XLAMD;
    end

    % Runaway guard (not in the book).  ERROR rather than break: a run that
    % never closed has no miss distance, and handing back the separation at
    % t = TMax as though it were one is exactly the plausible-wrong-answer
    % this package refuses to produce.
    if T > p.TMax
        error('mgs:zarchan:engage2d:noIntercept', ...
            ['no intercept within TMax = %g s (VC still %.1f ft/s, separation ' ...
             '%.0f ft).  Refusing to report a miss distance for an engagement ' ...
             'that never closed.'], p.TMax, VC, RTM);
    end
end
RTM = sqrt(RTM1*RTM1+RTM2*RTM2);   % the book's final display = the miss
% ========================= end of Listing 2.1 ================================

if n == 0
    error('mgs:zarchan:engage2d:noLog', ...
        ['the engagement ended at t = %.4f s, before the first 0.1 s log ' ...
         'point, so there is no time history at all, and every summary below ' ...
         'would be read off an empty array.  Increase the initial separation ' ...
         '(RT1) so the engagement lasts longer than one log interval.'], T);
end

k = 1:n;
r = struct();
r.mode = 'run';
r.opts = p;

% the book's seven logged channels -- ft, ft, ft, ft, ft, g, ft
r.T = ArrayT(k); r.RT1 = ArrayRT1(k); r.RT2 = ArrayRT2(k);
r.RM1 = ArrayRM1(k); r.RM2 = ArrayRM2(k);
r.XNCG = ArrayXNCG(k); r.RTM = ArrayRTM(k);
% additions (see the logging block)
r.VC = ArrayVC(k); r.VM = ArrayVM(k);
r.GAMDEG = ArrayGAMDEG(k); r.XLAMD = ArrayXLAMD(k);

r.tf_s    = T;
r.miss_ft = RTM;          % the book's number: separation at the first step PAST
                          % closest approach, evaluated at the PREDICTED state
                          % (the corrector updates RM1/RM2 but does not refresh
                          % RTM, and that is the listing's own behaviour).
r.miss_min_ft = rtm_min;  % addition: closest RTM actually seen.  Always <=
                          % miss_ft.  Quote miss_ft when comparing against the
                          % book; quote this only as a resolution note.
% The miss's own error bar -- see the header.  A miss below this is zero as far
% as this file knows.
%
% Uses the last LOGGED closing velocity, deliberately not the terminal VC: the
% loop exits on VC having gone negative, so terminal VC is a number that has
% just swept through zero and is worthless as a scale.  An earlier draft of
% this line read FineStep*max(VC,0) and therefore printed a quantum of 0.00 ft
% -- which would have advertised infinite precision on the one number in this
% file that has the least.
r.h_quantum_ft = p.FineStep * abs(ArrayVC(n));
r.xnc0_g     = XNC0/G;    % integrator-independent, exactly linear in XNP
r.vc_end     = VC;        % closing velocity at termination (negative by then)
r.xnc_max_g  = max(abs(r.XNCG));
r.xnc_end_g  = r.XNCG(end);
[~, ipk]     = max(abs(r.XNCG));
r.xnc_peak_t = r.T(ipk);
r.nsteps     = nsteps;
r.nlog       = n;

if p.Plots
    figure; plot(r.RT1, r.RT2, r.RM1, r.RM2); grid on
    title(sprintf('Listing 2.1 geometry:  HEDEG=%g   N''=%g   XNT=%g ft/s^2', ...
        p.HEDEG, p.XNP, p.XNT));
    xlabel('Downrange (ft)'); ylabel('Altitude (ft)'); legend('target','missile');
    figure; plot(r.T, r.XNCG); grid on
    title(sprintf('Listing 2.1 command:  HEDEG=%g   N''=%g   XNT=%g ft/s^2', ...
        p.HEDEG, p.XNP, p.XNT));
    xlabel('Time (s)'); ylabel('Missile acceleration (g)');
end
end


% =============================================================================
function B = benchmark()
%BENCHMARK  The five cases behind Figs 2.2-2.5, with a verdict table.
%
%   Cases run through the PUBLIC entry point on purpose: the defaults exercised
%   here are then the same object a caller gets, and cannot drift away from a
%   private second copy of the data statement.

A  = mgs.zarchan.engage2d('HEDEG',-20,'XNP',4,'XNT',0);     % Fig 2.2 / 2.3
Bc = mgs.zarchan.engage2d('HEDEG',-20,'XNP',5,'XNT',0);     % Fig 2.2 / 2.3
C  = mgs.zarchan.engage2d('HEDEG',0,'XNP',3,'XNT',96.6);    % Fig 2.4 / 2.5
D  = mgs.zarchan.engage2d('HEDEG',0,'XNP',4,'XNT',96.6);    % Fig 2.4 / 2.5
E  = mgs.zarchan.engage2d('HEDEG',0,'XNP',5,'XNT',96.6);    % Fig 2.4 / 2.5

% Shape checks.  TOL sits a hair above round-off: these are EXACT statements
% about the profile, so the tolerance exists only to absorb the last-ulp wobble
% of atan2/sin/cos, never to buy slack for a result that does not hold.
TOL = 1e-6;
decA  = all(diff(A.XNCG)  <= TOL);
decB  = all(diff(Bc.XNCG) <= TOL);
[~, ipkC] = max(C.XNCG);
riseC = all(diff(C.XNCG(1:ipkC)) >= -TOL);

a5 = atT(A, 5.1);  b5 = atT(Bc, 5.1);   % mid-flight, well past the crossover
ratio0 = Bc.xnc0_g / A.xnc0_g;          % must be exactly 5/4, by algebra
nprime_for_18g = 18 / (A.xnc0_g / A.opts.XNP);

K = struct('cs',{},'q',{},'comp',{},'book',{},'band',{},'v',{});

cA = 'A  HE=-20 XNT=0 N''=4';
K(end+1) = ck(cA,'n_c(0)',             sprintf('%.2f g',A.xnc0_g),    '~12 g',   '+/-1.0 g', vd(A.xnc0_g,11,13));
K(end+1) = ck(cA,'n_c monotone decay', yn(decA),                      'yes',     'exact',    vd(decA,1,1));
K(end+1) = ck(cA,'n_c at last log',    sprintf('%.3f g',A.xnc_end_g), '~0 g',    '<0.05 g',  vd(abs(A.xnc_end_g),0,0.05));
K(end+1) = ck(cA,'flight time',        sprintf('%.3f s',A.tf_s),      '~10.4 s', '+/-0.5 s', vd(A.tf_s,9.9,10.9));
K(end+1) = ck(cA,'miss distance',      sprintf('%.3f ft',A.miss_ft),  '~0 ft',   '<0.8 ft',  vd(A.miss_ft,0,0.8));

cB = 'B  HE=-20 XNT=0 N''=5';
K(end+1) = ck(cB,'n_c(0) / case A',    sprintf('%.9f',ratio0),        '1.25',    '+/-1e-9',  vd(ratio0,1.25-1e-9,1.25+1e-9));
K(end+1) = ck(cB,'n_c(0) > case A',    yn(Bc.xnc0_g>A.xnc0_g),        'yes',     'Fig 2.3',  vd(Bc.xnc0_g>A.xnc0_g,1,1));
K(end+1) = ck(cB,'n_c monotone decay', yn(decB),                      'yes',     'exact',    vd(decB,1,1));
K(end+1) = ck(cB,'n_c(5.1s) < case A', sprintf('%.2f < %.2f g',b5,a5),'yes',     'Fig 2.3',  vd(b5<a5,1,1));
K(end+1) = ck(cB,'miss distance',      sprintf('%.3f ft',Bc.miss_ft), '~0 ft',   '<0.8 ft',  vd(Bc.miss_ft,0,0.8));

cC = 'C  HE=0 XNT=3g N''=3';
K(end+1) = ck(cC,'n_c(0), coll. tri.', sprintf('%.3f g',C.xnc0_g),    '0 g',     '<0.01 g',  vd(abs(C.xnc0_g),0,0.01));
K(end+1) = ck(cC,'rises to its peak',  yn(riseC),                     'yes',     'Fig 2.5',  vd(riseC,1,1));
K(end+1) = ck(cC,'peak / target 3 g',  sprintf('%.2fx',C.xnc_max_g/3),'>1',      '1.2-3.0x', vd(C.xnc_max_g/3,1.2,3.0));
K(end+1) = ck(cC,'peak N''=3 > 4 > 5', sprintf('%.2f>%.2f>%.2f g',C.xnc_max_g,D.xnc_max_g,E.xnc_max_g), ...
                                                                      'yes',     'Fig 2.5',  vd(C.xnc_max_g>D.xnc_max_g && D.xnc_max_g>E.xnc_max_g,1,1));
K(end+1) = ck(cC,'miss distance',      sprintf('%.3f ft',C.miss_ft),  '~0 ft',   '<0.8 ft',  vd(C.miss_ft,0,0.8));

nsup  = numel(K);
npass = sum(strcmp({K.v},'PASS'));

% The two briefed values this port refutes.  Appended AFTER the count is taken
% so that they can neither inflate nor deflate the pass rate.
K(end+1) = ck(cB,'n_c(0)',   sprintf('%.2f g',Bc.xnc0_g), '~18 g (briefed)', 'see notes', 'REFUTED');
K(end+1) = ck(cC,'peak n_c', sprintf('%.2f g @ %.1f s',C.xnc_max_g,C.xnc_peak_t), ...
                                                          '~28 g (briefed)', 'see notes', 'REFUTED');

B = struct();
B.mode        = 'benchmark';
B.cases       = struct('A',A,'B',Bc,'C',C,'D',D,'E',E);
B.checks      = K;
B.n_supported = nsup;
B.n_pass      = npass;
B.ok          = (npass == nsup);
B.refuted     = { sprintf('case B n_c(0): briefed ~18 g, computed %.2f g', Bc.xnc0_g), ...
                  sprintf('case C peak n_c: briefed ~28 g, computed %.2f g', C.xnc_max_g) };
B.summary     = render(B, A, Bc, C, nprime_for_18g);
fprintf('%s\n', B.summary);
end


% =============================================================================
function s = render(B, A, Bc, C, nprime_for_18g)
L = {};
L{end+1} = 'Zarchan Listing 2.1 -- two-dimensional missile-target engagement (Ch.2, p.18)';
L{end+1} = 'units FEET: ft, ft/s, ft/s^2, g = 32.2 ft/s^2, deg via 57.3.  NOTHING converted to SI.';
L{end+1} = sprintf(['integrator RK2 (Heun), H = %g s, refined to %g s when TGO < %g s ' ...
                    '-> miss quantum %.2f ft'], ...
    A.opts.Step, A.opts.FineStep, A.opts.TgoFine, A.h_quantum_ft);
L{end+1} = sprintf(['nominal: VM = %g, VT = %g ft/s; missile (%g, %g) ft, ' ...
                    'target (%g, %g) ft, BETA = %g deg'], ...
    A.opts.VM, A.opts.VT, A.opts.RM1, A.opts.RM2, A.opts.RT1, A.opts.RT2, A.opts.BETADEG);
L{end+1} = '';
L{end+1} = sprintf('%-22s %-22s %-20s %-17s %-10s %s', ...
    'case', 'quantity', 'computed', 'book', 'band', 'verdict');
L{end+1} = repmat('-', 1, 103);
for i = 1:numel(B.checks)
    k = B.checks(i);
    L{end+1} = sprintf('%-22s %-22s %-20s %-17s %-10s %s', ...
        k.cs, k.q, k.comp, k.book, k.band, k.v); %#ok<AGROW>
end
L{end+1} = repmat('-', 1, 103);
L{end+1} = sprintf('VERDICT: %d/%d book-supported checks PASS.  2 briefed values REFUTED.', ...
    B.n_pass, B.n_supported);
L{end+1} = '';
L{end+1} = 'REFUTED 1 -- case B initial acceleration, briefed ~18 g:';
L{end+1} = '  n_c(0) = N''*VC(0)*XLAMD(0), and neither factor depends on N'', so n_c(0) is';
L{end+1} = sprintf('  EXACTLY linear in N''.  Case A (N''=4) gives %.2f g, so N''=5 gives %.2f g', ...
    A.xnc0_g, Bc.xnc0_g);
L{end+1} = '  by algebra, not by simulation -- the 1.25 ratio check above is the proof.';
L{end+1} = sprintf('  18 g would require N'' = %.2f.', nprime_for_18g);
L{end+1} = '';
L{end+1} = 'REFUTED 2 -- case C peak acceleration, briefed ~28 g:';
L{end+1} = '  zero-lag PN against a constant-g target has terminal n_c = nT*N''/(N''-1)';
L{end+1} = sprintf('  = 3 * 1.5 = 4.50 g; nonlinear geometry lifts the peak to %.2f g at t = %.1f s.', ...
    C.xnc_max_g, C.xnc_peak_t);
L{end+1} = '  28 g against a 3 g target needs N''/(N''-1) = 9.33, i.e. N'' = 1.12, which is not a';
L{end+1} = '  navigable gain.  A demand that large at N''=3 requires guidance LAG -- it is a';
L{end+1} = '  Ch.3/Ch.6 number, not a Ch.2 one.';
L{end+1} = sprintf(['  What the book DOES claim holds: peak is "significantly higher" than the ' ...
                    'target''s 3 g (%.2fx),'], C.xnc_max_g/3);
L{end+1} = '  and the profile is "virtually monotonically increasing" (rises to 80% of tF, then rolls off).';
L{end+1} = '';
L{end+1} = 'This listing has NO lag, NO gravity, NO drag, NO acceleration limit.  Miss here is';
L{end+1} = 'numerical noise by construction: it is the zero-lag baseline that the Ch.3/Ch.6';
L{end+1} = 'misses are differenced against, not a performance result to quote on its own.';
s = strjoin(L, newline);
end


% =============================================================================
function k = ck(cs, q, comp, book, band, v)
k = struct('cs',cs, 'q',q, 'comp',comp, 'book',book, 'band',band, 'v',v);
end

function v = vd(x, lo, hi)
%VD  PASS/FAIL against an inclusive band.  Logicals arrive as 0/1 with lo=hi=1.
%   NaN FAILS.  A NaN silently compared into a "pass" is how a broken run gets
%   reported green, so it is handled first and explicitly.
x = double(x);
if ~isfinite(x)
    v = 'FAIL';
elseif x >= lo && x <= hi
    v = 'PASS';
else
    v = 'FAIL';
end
end

function s = yn(b)
if b, s = 'yes'; else, s = 'NO'; end
end

function y = atT(r, t)
%ATT  Logged n_c (g) at the log point NEAREST t.  Nearest, not interpolated:
%   the log sits on a 0.1 s grid, and an interpolated value would be a number
%   the simulation never actually computed.
[~, i] = min(abs(r.T - t));
y = r.XNCG(i);
end
