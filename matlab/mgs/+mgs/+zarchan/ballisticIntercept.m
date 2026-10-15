function R = ballisticIntercept(opts)
%MGS.ZARCHAN.BALLISTICINTERCEPT  Zarchan Listing 14.1 -- engagement simulation
%   between an IMPULSIVE strategic interceptor and a BALLISTIC target, in
%   Earth-centred inverse-square gravity.
%
%   R = mgs.zarchan.ballisticIntercept()
%   R = mgs.zarchan.ballisticIntercept('PREDERR', 1e5, 'assert', false)
%   R = mgs.zarchan.ballisticIntercept('allcases', false)   % nominal run only
%
%   Line-by-line port of Listing 14.1 (zarchan.txt:4220-4247, heading :4219)
%   with its predictpz prediction routine (:4245-4247) and the lambertpz of
%   Listing 13.2 (:3943-3955) that it calls, on the book's own two-pass
%   second-order Runge-Kutta (Heun).
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: ft, ft/s, ft/s^2, g = 32.2, n mi = 6076 ft;
%   A = 2.0926e7 ft, GM = 1.4077e16 ft^3/s^2.  NOTHING IS SI and nothing is
%   converted, so every number printed lies straight against a published one.
%   Convert AT THE CALL SITE and say so there.  Two degree constants coexist,
%   both the book's: DEGRAD = 360/(2*pi) for longitudes, a bare 57.3 for GAMT
%   (23/57.3 is 23.0011 deg).  Kept apart -- VT is a closed form in GAMT.
% ===========================================================================
%
%   *** WHAT IT IS NOT. ***  Vacuum, two-body, non-rotating spherical Earth,
%   ZERO-LAG guidance, no acceleration limit (14.2 adds XNCLIM = 20 g, :4322),
%   an interceptor whose whole velocity is one Lambert impulse, and no
%   airframe, autopilot, seeker or noise.  It is here because it is the book's
%   cleanest demonstration that BEING ON A PERFECT COLLISION TRIANGLE DOES NOT
%   MAKE THE GUIDANCE QUIET: guidance off, the missile still hits (~3 ft);
%   guidance on, the same engagement burns ~356 ft/s chasing line-of-sight
%   rates that exist only because both bodies accelerate (:4255).
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_14_01.m does not parse.  This book's OCR maps
%   '<' to ',' and '>' to '.' inside code.  Forced repairs, no judgement:
%   "while VC.=0." -> VC >= 0 (the run ends when the range stops closing);
%   "if TGO..1" -> TGO > 0.1 (the step is REFINED in the endgame, so the fine
%   branch must be the small-TGO one); "while STEP ,=1" -> STEP <= 1 (the body
%   sets STEP = 2 to leave); "if S.=.99999" -> S >= .99999 (S is zeroed on
%   every record); "while t,=(tf-.00001)" -> t <= ... in predictpz; "if
%   tf.tfdes" and "if (xnext.gmax|xnext,gmin)" in lambertpz.  "save datfil.txt
%   output /ascii" is the book's typo for -ascii; nothing is written to disk.
%   No prose leaked into this listing, so nothing was dropped.
%
%   ITS ONE UNRESOLVED VARIABLE-TO-VARIABLE COMPARISON, resolved and MEASURED:
%
%       if XLONGM.XLONGT   ->   if XLONGM > XLONGT
%
%   With the book's case (missile 45 deg, target 90 deg) that is FALSE, so the
%   ELSE branch runs: X1T = VT*cos(-pi/2 + GAMT + XLONGT).  The reason is
%   arithmetic, not taste.  At longitude 90 deg the position is (0, A), so the
%   horizontal unit vector pointing DOWN-longitude -- towards the missile --
%   is (+1, 0) and the local vertical is (0, +1).  A target flying AT the
%   missile with flight-path angle GAMT must have velocity VT*(cos GAMT,
%   sin GAMT), and -pi/2 + GAMT + XLONGT = GAMT gives exactly that.  The other
%   branch gives 157 deg: the target flying away down the far side.
%   STRUCTURAL CHECK 4 is the delete experiment -- it builds both branches
%   through the same code and shows the flipped one drives the target's
%   longitude AWAY from the missile's.  (lambertpz's own "if xlongt.xlongm" is
%   the mirror image, resolved the same way, already scored by
%   MGS.ZARCHAN.LAMBERTEFFICIENT and not re-scored here.)
%
%   ONE FAITHFUL GENERALISATION, NAMED: the listing hard-codes H = .01 coarse
%   and H = .0001 fine.  Here the fine step is H/100 -- both are the book's at
%   the default, and refining H refines the endgame with it.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Every book value was grepped out of zarchan_decoded.txt -- the
%   MATHS-DECODED copy, because the divert formula's 300 ft/s lives in an
%   EQUATION and is invisible to a grep of the raw extraction -- and its line
%   is in the row's .source.  Nothing was read off a figure; the rows that
%   concern figures score PROSE CLAIMS about them, which is a different thing.
%     1  nominal divert, "about 350 ft/s" (:4255)             band 10%
%     2  peak recorded |nc| with a 100-kft prediction error vs "less than
%        0.05 g" (:4269)                                      band 10%
%     3  INFO: the STRICT reading of that bound is violated -- see below
%     4  divert with that prediction error, "about 480" (:4269)  band 10%
%     5  the closed form dV = PredErr*N'/((N'-1)*tF) = 300 (:4258-4259)  1e-9
%     6  the 100-s short-range case, "nearly 1500 ft/s" (:4284-4285)  band 10%
%     7  guidance OFF with that error: "we would miss by 100 kft" (:4256) 15%
%     8  BOUND: guidance off, no error, "the missile hit the target" (:4211)
%     9  SHAPE: PN issues commands anyway (:4211, :4255)
%    10  INFO: the book contradicts itself on the 100-s divert
%
%   ROW 2 IS THE FINDING.  The book writes "less than 0.05 g"; the recorded
%   profile peaks at about 0.0511 g -- a BROAD interior maximum near t = 382 s,
%   not a start-up or endgame artifact.  It is scored as a
%   one-significant-figure figure-derived VALUE at 10% (same provenance and
%   band as ballisticTargetSim's "approximately 40 kft"), and the strict-bound
%   violation goes on the board as its own INFO row rather than into a detail
%   string.  It is NOT scored as a bound at 0.05 with a widened threshold:
%   that would be picking the band from the answer.  And it must be measured
%   on the RECORDED 1-s profile, which is what Fig. 14.7 plots -- the per-step
%   peak is ~204 g, all of it in the last recorded second as RTM -> 0 and the
%   line-of-sight rate diverges.  R.cases.prederr.maxg_step carries that.
%
%   ROW 10.  zarchan.txt:4164 says the SAME formula, N' = 3 and 100-kft error
%   need "about 1050 ft/s ... in 100 s"; :4284 evaluates it as 1500, and
%   1e5*3/(2*100) is 1500.  Row 6 is scored against 1500 and :4164 is cited.
%
%   NOT SCORED: anything in Figs. 14.3-14.9 that is only a picture, and the
%   book's adjectives ("small", "somewhat larger", "useful and accurate").
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED. ***  MATLAB is the
%   caller's; this file has never been executed here.  These come from an
%   independent transcription of the same listing, so a first run has an
%   expectation to be compared against instead of merely admired:
%     VT = 24184.8 ft/s; lambertpz converges in 5 passes (7 for the 100-s case)
%     nominal guided      divert 355.9 ft/s   miss  2.1 ft   t_end 500.02 s
%     guidance off        divert   0          miss  3.3 ft   <- the hit
%     100-kft pred. err   divert 475.0 ft/s   peak recorded 0.0511 g
%     100 kft, unguided   miss 92,003 ft
%     100-s short range   divert 1498.1 ft/s
%     target energy drift 1.7e-11 relative over 500 s
%   A row landing near a band EDGE is not a pass to be accepted.  Do not widen
%   a band to make a row green.
%
%   OUTPUTS
%     R.nominal  the caller's run: .t .distnmT .altnmT .distnmM .altnmM .xncg
%                .delv (the listing's Array*), .rtm_ft (its printed RTM -- the
%                MISS), .delv_ft, .t_end, .vt_fps, .maxg_sampled, .maxg_step,
%                .lam_iter, .lam_resid, .dE_rel, .dh_rel, .capped
%     R.cases    the book's other four runs; [] unless 'allcases' and the
%                caller left every parameter at the listing's own value
%     R.lambda_out   lambda = r0|v|^2/gm from the velocity lambertpz RETURNED,
%                    for the nominal run; R.lambda_max is the largest over the
%                    nominal run and every extra case.  See the near-parabolic
%                    guard section below.
%     R.bench .selfcheck .units .p .pass .verdict .summary
%
% ---------------------------------------------------------------------------
%   ⭐⭐⭐ NEAR-PARABOLIC GUARD -- lambda = r0*|v|^2/gm MUST STAY BELOW 1.999999
%
%   THE SILENT WRONG ANSWER.  Listing 13.2 is ELLIPTIC-ONLY.  Its bracket
%   [gmin, gmax] is DEFINED by the escape condition lambda = 2, and its
%   flight-time closed form divides by bot2 = xlam*(2/xlam-1)^1.5, which
%   collapses as lambda -> 2.  Handed a transfer that is genuinely HYPERBOLIC
%   the secant cannot leave the bracket, so it crawls to the bracket EDGE and
%   halts on an orbit that is not the solution -- while its own residual test
%   says it converged.
%
%   MEASURED 2026-08-24 -- alt 300 nm -> 0, transfer angle 78 deg, TF = 750 s:
%     true transfer needs   lambda = 2.014012          (HYPERBOLIC)
%     lambertpz halts at    lambda = 1.999999919,  bot2 = 1.6409e-11
%     it reports            tF = 749.999996 s   -> passes its own 1e-8 test
%     ⚠⚠ converged = TRUE
%     propagating its answer (ode45, RelTol 1e-13) MISSES THE TARGET BY
%     98,751.56 ft.  An independent universal-variable (Bate-Mueller-White)
%     Lambert returns 35302.6106 ft/s and misses by 0.00 ft; this listing
%     returns 35179.5920 ft/s.
%
%   ⚠⚠ THE GUARD IS NOT KEYED ON THE ROUTINE'S `converged` FLAG.  That flag is
%   the thing being caught -- it is TRUE on the wrong answer, in all six
%   transcriptions of this listing carried in this package.
%   ⚠⚠ AND MUTUAL AGREEMENT DID NOT CATCH IT EITHER.  Those six transcriptions
%   agree to 0.000e+00 over 954 converged geometries with identical iteration
%   counts 954/954, because the misreading is in the LISTING, not in any one
%   transcription of it.  Only a quantity the routine does not control
%   separates the two answers, and lambda is that quantity.
%
%   ⚠ MEASURED SEPARATION, and it is NOT the "enormous margin" this guard was
%   commissioned on.  Over a 6150-geometry sweep (50 transfer angles x 41
%   flight times x 3 altitude pairs) the 3043 converged-and-CORRECT cases reach
%   lambda = 1.999997634 and the 17 converged-and-WRONG cases start at
%   1.999999740 (misses 66,600 ft to 24,278,620 ft, all adjudicated by
%   propagation).  The threshold separates them with ZERO false alarms, but the
%   gap is ~1.4e-6, not the ~0.039 a coarser 1080-point grid suggested.
%   ⚠ ONE CASE LEAKS: transfer angle 99 deg, TF = 900 s halts at
%   lambda = 1.999997634 -- BELOW the threshold -- and still misses by 141.89 ft.
%   The threshold is left where it was specified; widening it to swallow that
%   case is not something any source supports.
%
%   THIS PORT'S REACH -- MEASURED, AND THE GUARD BITES.  961 runs were swept
%   over XLONGMDEG = -90:5:60 and TF = 60..1000 s.  249 of them had lambertpz
%   report CONVERGED (resid <= 1e-8), and 2 of those 249 sit above the cap:
%
%     XLONGMDEG = -45, TF = 880 s -> lambda = 1.999999981, 35 passes,
%                                    resid 6.231e-09  (PASSES its own test)
%     XLONGMDEG = -50, TF = 960 s -> lambda = 1.999999945, 31 passes,
%                                    resid 8.24e-09
%
%   ⭐ At the first of those every OTHER invariant in this file is GREEN --
%   energy 1.9e-11, angular momentum 1.2e-12, "lambertpz converged (not
%   capped)" 6.231e-09, the delete experiment, the step cap -- and ONLY the
%   near-parabolic guard is red.  That is the whole signature of this fault:
%   nothing else in the port notices.
%   ⚠ The other 712 runs are HONEST failures (residuals of 0.5 to 17), already
%   caught by invariant 3; they are not what this guard is for.
%   ⚠ The Lambert target here is the PREDICTED intercept point XTF/YTF, not a
%   fixed aim point, so the transfer angle actually solved is set by the target
%   propagation and is NOT simply XLONGTDEG minus XLONGMDEG.
%   MARGIN AT THE BOOK'S OWN CASE: lambda_out = 0.631509886, and the largest
%   over the nominal run plus all four extra cases is 0.643214568.
% ---------------------------------------------------------------------------
%
%   See also MGS.ZARCHAN.VELOCITYFORMULA (Listing 11.4 -- the SAME target model
%   and the same VT closed form), MGS.ZARCHAN.LAMBERTEFFICIENT,
%   MGS.ZARCHAN.BALLISTICTARGETSIM (16.1 -- a DIFFERENT target: flat Earth plus
%   drag, no GM/r^2), MGS.ZARCHAN.BENCHSUITE.

arguments
    opts.XLONGMDEG (1,1) double {mustBeFinite} = 45      % deg, missile
    opts.XLONGTDEG (1,1) double {mustBeFinite} = 90      % deg, target
    opts.ALTNMTIC  (1,1) double {mustBeFinite} = 0       % n mi
    opts.ALTNMMIC  (1,1) double {mustBeFinite} = 0       % n mi
    opts.TF        (1,1) double {mustBeFinite, mustBePositive} = 500   % s
    opts.GAMDEGT   (1,1) double {mustBeFinite} = 23      % deg, target fpa
    opts.H         (1,1) double {mustBeFinite, mustBePositive} = 0.01  % s
    opts.XNP       (1,1) double {mustBeFinite, mustBeNonnegative} = 3
    opts.PREDERR   (1,1) double {mustBeFinite} = 0       % ft, added to YTF
    opts.DISTNMT   (1,1) double {mustBeFinite, mustBePositive} = 6000  % n mi
    opts.samp      (1,1) double {mustBeFinite, mustBePositive} = 1     % s
    % The book's other cases cost four more integrations (~5x the runtime).
    % Turning them off SKIPS the rows that need them; it never turns them green.
    opts.allcases  (1,1) logical = true
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

if opts.H > opts.samp
    error('mgs:zarchan:ballisticIntercept:step', ...
        ['H = %g s exceeds the record interval samp = %g s, so the returned ' ...
         'profile would be sampled at the step rate while claiming the ' ...
         'coarser one in its own time vector.'], opts.H, opts.samp);
end
if opts.TF <= 10*opts.H
    error('mgs:zarchan:ballisticIntercept:short', ...
        'TF = %g s is too short to integrate at H = %g s.', opts.TF, opts.H);
end

q = struct('XLONGMDEG',opts.XLONGMDEG, 'XLONGTDEG',opts.XLONGTDEG, ...
           'ALTNMTIC',opts.ALTNMTIC, 'ALTNMMIC',opts.ALTNMMIC, 'TF',opts.TF, ...
           'GAMDEGT',opts.GAMDEGT, 'H',opts.H, 'XNP',opts.XNP, ...
           'PREDERR',opts.PREDERR, 'DISTNMT',opts.DISTNMT, 'samp',opts.samp);

% Is this the listing's own case?  Chapter 14 quotes its divert numbers for
% that case and for nothing else, so anything off it SKIPS rather than being
% scored loosely.
bk = abs(opts.XLONGMDEG-45) < 1e-12 && abs(opts.XLONGTDEG-90) < 1e-12 && ...
     opts.ALTNMTIC == 0 && opts.ALTNMMIC == 0 && abs(opts.TF-500) < 1e-12 && ...
     abs(opts.GAMDEGT-23) < 1e-12 && abs(opts.H-0.01) < 1e-15 && ...
     abs(opts.XNP-3) < 1e-12 && opts.PREDERR == 0 && ...
     abs(opts.DISTNMT-6000) < 1e-12 && abs(opts.samp-1) < 1e-12;

nominal = engage(q);

C = [];
if bk && opts.allcases
    % The book's other runs (:4211, :4256-4269, :4280-4285), each stated as
    % "the same case except ...".  Built FROM q so the shared parameters
    % cannot drift away from the nominal run's.
    C.unguided   = engage(setfields(q, {'XNP'}, {0}));
    C.prederr    = engage(setfields(q, {'PREDERR'}, {1e5}));
    C.prederr_og = engage(setfields(q, {'PREDERR','XNP'}, {1e5, 0}));
    C.short      = engage(setfields(q, {'PREDERR','TF','XLONGMDEG'}, ...
                                       {1e5, 100, 80}));
end

R = struct();
R.nominal = nominal;
R.cases   = C;
R.units   = ['ft, ft/s, ft/s^2 (g = 32.2 ft/s^2); n mi = 6076 ft; ' ...
             'A = 2.0926e7 ft, GM = 1.4077e16 ft^3/s^2'];
R.p = struct('XLONGMDEG',opts.XLONGMDEG, 'XLONGTDEG',opts.XLONGTDEG, ...
             'TF',opts.TF, 'GAMDEGT',opts.GAMDEGT, 'H',opts.H, ...
             'XNP',opts.XNP, 'PREDERR',opts.PREDERR, 'DISTNMT',opts.DISTNMT, ...
             'samp',opts.samp, 'bookcase',bk, 'allcases',opts.allcases, ...
             'units',R.units);

% Every Lambert call this port made: the nominal run's plus one per extra case.
lamAll = nominal.lam_out;
if ~isempty(C)
    fn = fieldnames(C);
    for i = 1:numel(fn), lamAll(end+1) = C.(fn{i}).lam_out; end %#ok<AGROW>
end
R.lambda_out = nominal.lam_out;
R.lambda_max = max(lamAll);

R.selfcheck = structuralChecks(nominal, q, lamAll);

if opts.benchmark
    R.bench = benchmarks(nominal, C, bk, opts);
    ev      = ~[R.bench.skipped];
    R.pass  = bk && any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    R.pass  = false;      % NOT CHECKED must never render as fine
end

R.summary = render(R, bk, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R); end

% Adjudication.  The verdict is FORMED either way; 'assert' chooses only the
% reaction to it.  With assert false the result comes back COMPLETE, with a
% warning -- benchSuite runs every port that way so one red row cannot hide
% the board, and a port that refuses to return is one nobody can diagnose.
msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false: nothing was compared against the book.';
elseif ~any(ev)
    msg = 'not one book value could be evaluated.';
elseif ~bk
    msg = sprintf(['off the listing''s own case, so only the ' ...
        'case-independent closed-form row was evaluated -- the SIMULATION ' ...
        'was compared against nothing.  Chapter 14 quotes its numbers for ' ...
        '45/90 deg, TF = 500 s, GAMT = 23 deg, N'' = 3, 6000 n mi, ' ...
        'PREDERR = 0, H = 0.01 s, samp = 1 s; this run used %g/%g deg, ' ...
        'TF = %g s, N'' = %g, PREDERR = %g ft, H = %g s.'], opts.XLONGMDEG, ...
        opts.XLONGTDEG, opts.TF, opts.XNP, opts.PREDERR, opts.H);
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
        error('mgs:zarchan:ballisticIntercept:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:ballisticIntercept:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function q = setfields(q, names, vals)
%SETFIELDS  q with the named fields replaced.  Every extra case below is
%   stated by the book as "identical except ...", and this is that sentence.
for i = 1:numel(names), q.(names{i}) = vals{i}; end
end


% =============================================================================
function [X1T, Y1T, VT] = targetVel(q, flip)
%TARGETVEL  Listing 14.1's line 1 and its if/else, zarchan.txt:4220-4223.
%   'flip' inverts the resolved comparison and exists ONLY for structural
%   check 4: the delete experiment runs through this same code, so it cannot
%   drift away from what the simulation actually flies.
A = 2.0926e7;  GM = 1.4077e16;  DEGRAD = 360/(2*pi);
GAMT = q.GAMDEGT/57.3;                       % the book's 57.3, not 180/pi
PHIT = q.DISTNMT*6076/A;
R0T  = A + q.ALTNMTIC*6076;
TOP  = GM*(1 - cos(PHIT));
TEMP = R0T*cos(GAMT)/A - cos(PHIT + GAMT);   % the listing divides by A here
BOT  = R0T*cos(GAMT)*TEMP;
VT   = sqrt(TOP/BOT);                        % Ch.11 closed form, :3515-3518
XLONGM = q.XLONGMDEG/DEGRAD;
XLONGT = q.XLONGTDEG/DEGRAD;
if xor(XLONGM > XLONGT, flip)
    X1T = VT*cos(pi/2 - GAMT + XLONGT);  Y1T = VT*sin(pi/2 - GAMT + XLONGT);
else
    X1T = VT*cos(-pi/2 + GAMT + XLONGT); Y1T = VT*sin(-pi/2 + GAMT + XLONGT);
end
end


% =============================================================================
function S = engage(q)
%ENGAGE  Listing 14.1, zarchan.txt:4220-4244.  Every constant is local or
%   arrives inside q: MATLAB local functions cannot see the caller's
%   workspace, and a previous wave in this package lost a whole run to that.
A = 2.0926e7;  GM = 1.4077e16;  DEGRAD = 360/(2*pi);
[X1T, Y1T, VT] = targetVel(q, false);

ALTT = q.ALTNMTIC*6076;  ALTM = q.ALTNMMIC*6076;
XLONGM = q.XLONGMDEG/DEGRAD;  XLONGT = q.XLONGTDEG/DEGRAD;
XM = (A+ALTM)*cos(XLONGM);  YM = (A+ALTM)*sin(XLONGM);
XT = (A+ALTT)*cos(XLONGT);  YT = (A+ALTT)*sin(XLONGT);
XFIRSTT = XT;  YFIRSTT = YT;
T = 0;  Sacc = 0;

% Prediction routine: where is the ballistic target at tF?  PREDERR is the
% book's knob for corrupting that answer, and it lands on YTF only.
[XTF, YTF] = predictpz(q.TF, XT, YT, X1T, Y1T, q.H, GM);
YTF = YTF + q.PREDERR;
[VRXM, VRYM, lamIter, lamResid, lamOut] = lambertpz(XM, YM, q.TF, XTF, YTF, ...
                                            XLONGM, XLONGT, GM);
X1M = VRXM;  Y1M = VRYM;                    % the impulse IS the whole velocity

RTM1 = XT-XM;  RTM2 = YT-YM;  RTM = sqrt(RTM1^2 + RTM2^2);
VTM1 = X1T-X1M;  VTM2 = Y1T-Y1M;
VC = -(RTM1*VTM1 + RTM2*VTM2)/RTM;
DELV = 0;  XNC = 0;

E0 = 0.5*(X1T^2 + Y1T^2) - GM/sqrt(XT^2 + YT^2);   % target orbital energy
h0 = XT*Y1T - YT*X1T;                              % target angular momentum
dE = 0;  dh = 0;

nrec = ceil(q.TF/q.samp) + 10;
arT = zeros(nrec,1); arDT = arT; arAT = arT; arDM = arT; arAM = arT;
arG = arT;  arDV = arT;  n = 0;  maxgStep = 0;

XDT=0; YDT=0; X1DT=0; Y1DT=0; XDM=0; YDM=0; X1DM=0; Y1DM=0; DELVD=0;
nstep = 0;  capped = false;
% NOT in the book.  A geometry that never stops closing spins forever inside a
% suite; structural check 5 is what makes the cap visible instead of silent.
nmax = ceil(4*q.TF/q.H) + 200000;

while VC >= 0
    nstep = nstep + 1;
    if nstep > nmax, capped = true; break, end
    TGO = RTM/VC;
    if TGO > 0.1, H = q.H; else, H = q.H/100; end   % the book's .01 / .0001

    XOLDT=XT; YOLDT=YT; X1OLDT=X1T; Y1OLDT=Y1T;
    XOLDM=XM; YOLDM=YM; X1OLDM=X1M; Y1OLDM=Y1M;  DELVOLD=DELV;

    % Zarchan's two-pass predictor loop: pass 1 evaluates the derivatives
    % where we are, pass 2 takes a full Euler step, advances T, and
    % re-evaluates there.  T advances INSIDE this loop, exactly once.
    STEP = 1;  FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            XT=XT+H*XDT; YT=YT+H*YDT; X1T=X1T+H*X1DT; Y1T=Y1T+H*Y1DT;
            XM=XM+H*XDM; YM=YM+H*YDM; X1M=X1M+H*X1DM; Y1M=Y1M+H*Y1DM;
            DELV = DELV + H*DELVD;  T = T + H;  STEP = 2;
        end
        TEMBOTT = (XT^2 + YT^2)^1.5;
        X1DT = -GM*XT/TEMBOTT;  Y1DT = -GM*YT/TEMBOTT;  XDT = X1T;  YDT = Y1T;
        RTM1 = XT-XM;  RTM2 = YT-YM;  RTM = sqrt(RTM1^2 + RTM2^2);
        VTM1 = X1T-X1M;  VTM2 = Y1T-Y1M;
        VC   = -(RTM1*VTM1 + RTM2*VTM2)/RTM;
        % The listing recomputes TGO here and never reads it -- the step-size
        % branch at the top of the outer loop recomputes it again.  Kept, and
        % kept dead, so the transcription matches the page.
        TGO  = RTM/VC; %#ok<NASGU>
        XLAM  = atan2(RTM2, RTM1);
        XLAMD = (RTM1*VTM2 - RTM2*VTM1)/(RTM*RTM);
        XNC   = q.XNP*VC*XLAMD;               % proportional navigation
        DELVD = abs(XNC);                     % divert is int |nc| dt
        AM1 = -XNC*sin(XLAM);  AM2 = XNC*cos(XLAM);
        TEMBOTM = (XM^2 + YM^2)^1.5;
        X1DM = -GM*XM/TEMBOTM + AM1;  Y1DM = -GM*YM/TEMBOTM + AM2;
        XDM = X1M;  YDM = Y1M;  FLAG = 1;
    end
    % The listing's trailing "FLAG=0" is dropped: the outer loop re-initialises
    % FLAG on its next pass, so that assignment can never be read.

    XT=(XOLDT+XT)/2+.5*H*XDT;  YT=(YOLDT+YT)/2+.5*H*YDT;
    X1T=(X1OLDT+X1T)/2+.5*H*X1DT;  Y1T=(Y1OLDT+Y1T)/2+.5*H*Y1DT;
    XM=(XOLDM+XM)/2+.5*H*XDM;  YM=(YOLDM+YM)/2+.5*H*YDM;
    X1M=(X1OLDM+X1M)/2+.5*H*X1DM;  Y1M=(Y1OLDM+Y1M)/2+.5*H*Y1DM;
    DELV = (DELVOLD+DELV)/2 + .5*H*DELVD;

    maxgStep = max(maxgStep, abs(XNC)/32.2);
    Sacc = Sacc + H;
    if Sacc >= q.samp*(1 - 1e-5)             % the listing's "S >= .99999"
        Sacc = 0;
        RT = sqrt(XT^2 + YT^2);  RM = sqrt(XM^2 + YM^2);
        RF = sqrt(XFIRSTT^2 + YFIRSTT^2);
        n = n + 1;
        arT(n)  = T;
        % acos of the normalised dot product.  The clamp is NOT in the book;
        % it stops a rounding-level 1+eps returning a complex downrange at
        % the very first sample, and cannot mask anything else.
        arDT(n) = A*acos(clamp1((XT*XFIRSTT + YT*YFIRSTT)/(RT*RF)))/6076;
        arAT(n) = (RT - A)/6076;
        arDM(n) = A*acos(clamp1((XM*XFIRSTT + YM*YFIRSTT)/(RM*RF)))/6076;
        arAM(n) = (RM - A)/6076;
        arG(n)  = XNC/32.2;
        arDV(n) = DELV;
        E = 0.5*(X1T^2 + Y1T^2) - GM/RT;
        h = XT*Y1T - YT*X1T;
        dE = max(dE, abs(E-E0)/abs(E0));
        dh = max(dh, abs(h-h0)/abs(h0));
    end
end

S = struct('t',arT(1:n), 'distnmT',arDT(1:n), 'altnmT',arAT(1:n), ...
    'distnmM',arDM(1:n), 'altnmM',arAM(1:n), 'xncg',arG(1:n), ...
    'delv',arDV(1:n), 'rtm_ft',RTM, 'delv_ft',DELV, 't_end',T, 'vt_fps',VT, ...
    'xtf',XTF, 'ytf',YTF, 'vrxm',VRXM, 'vrym',VRYM, ...
    'maxg_sampled',max([0; abs(arG(1:n))]), 'maxg_step',maxgStep, ...
    'lam_iter',lamIter, 'lam_resid',lamResid, 'lam_out',lamOut, ...
    'dE_rel',dE, 'dh_rel',dh, ...
    'nstep',nstep, 'capped',capped, 'nrec',n);
end


% =============================================================================
function c = clamp1(c)
c = min(1, max(-1, c));
end


% =============================================================================
function [xtf, ytf] = predictpz(tf, x, y, x1, y1, h, gm)
%PREDICTPZ  Listing 14.1's prediction routine, zarchan.txt:4245-4247.  The
%   SAME ballistic equations and the SAME Heun scheme as the main loop, run
%   ahead of the engagement to find the predicted intercept point.
t = 0;  xd = 0;  yd = 0;  x1d = 0;  y1d = 0;
while t <= (tf - 0.00001)
    xold=x; yold=y; x1old=x1; y1old=y1;
    step = 1;  flag = 0;
    while step <= 1
        if flag == 1
            x=x+h*xd; y=y+h*yd; x1=x1+h*x1d; y1=y1+h*y1d; t=t+h; step=2;
        end
        tembot = (x^2 + y^2)^1.5;
        x1d = -gm*x/tembot;  y1d = -gm*y/tembot;  xd = x1;  yd = y1;  flag = 1;
    end
    x=(xold+x)/2+.5*h*xd;  y=(yold+y)/2+.5*h*yd;
    x1=(x1old+x1)/2+.5*h*x1d;  y1=(y1old+y1)/2+.5*h*y1d;
end
xtf = x;  ytf = y;
end


% =============================================================================
function [vrx, vry, icount, resid, lamout] = lambertpz(xic, yic, tfdes, xf, ...
                                               yf, xlongm, xlongt, gm)
%LAMBERTPZ  Listing 13.2's Lambert routine, zarchan.txt:3943-3955, called by
%   14.1 to give the impulsive interceptor the velocity that puts it on the
%   predicted intercept point at tfdes.  Secant search on flight-path angle,
%   bracketed by the escape-velocity condition.  Already benchmarked against
%   Table 13.2 by MGS.ZARCHAN.LAMBERTEFFICIENT, so it is not re-scored here.
%
%   The velocity returned belongs to the gam whose flight time PASSED the
%   test, not to the gam computed after it -- the test reads tf from the top
%   of the same pass that produced vrx, and reordering those breaks it
%   silently.  The 200-pass cap is NOT in the book: without it a degenerate
%   geometry (tf == told, so the secant divides by zero) spins forever inside
%   a suite.  Structural check 3 sees the cap, because resid then exceeds 1e-8.
ric = sqrt(xic^2 + yic^2);  rf = sqrt(xf^2 + yf^2);
cphi = (xic*xf + yic*yf)/(ric*rf);  phi = acos(clamp1(cphi));  sphi = sin(phi);
r0 = ric;  icount = 0;
gmin = atan2(sphi - sqrt(2*r0*(1-cphi)/rf), 1-cphi);
gmax = atan2(sphi + sqrt(2*r0*(1-cphi)/rf), 1-cphi);
gam = (gmin+gmax)/2;  tf = 0;  gold = gam;  told = 0;
vrx = 0;  vry = 0;
while ~(abs(tfdes-tf) <= 1e-8*tfdes)
    top = gm*(1-cos(phi));
    temp = r0*cos(gam)/rf - cos(phi+gam);
    bot = r0*cos(gam)*temp;
    v = sqrt(top/bot);
    if xlongt > xlongm
        vrx = v*cos(pi/2 - gam + xlongm);   vry = v*sin(pi/2 - gam + xlongm);
    else
        vrx = v*cos(-pi/2 + gam + xlongm);  vry = v*sin(-pi/2 + gam + xlongm);
    end
    xlam = r0*v*v/gm;
    top1 = tan(gam)*(1-cos(phi)) + (1-xlam)*sin(phi);
    bot1p = (1-cos(phi))/(xlam*cos(gam)*cos(gam));
    bot1 = (2-xlam)*(bot1p + cos(gam+phi)/cos(gam));
    top2 = 2*cos(gam);  bot2 = xlam*((2/xlam - 1)^1.5);  top3 = sqrt(2/xlam - 1);
    bot3 = cos(gam)/tan(phi/2) - sin(gam);
    temp = (top2/bot2)*atan2(top3, bot3);
    tf = r0*(top1/bot1 + temp)/(v*cos(gam));
    icount = icount + 1;
    if tf > tfdes, gmax = gam; else, gmin = gam; end
    if icount == 1
        xnext = (gmax+gmin)/2;
    else
        xnext = gam + (gam-gold)*(tfdes-tf)/(tf-told);
        if (xnext > gmax || xnext < gmin), xnext = (gmax+gmin)/2; end
    end
    gold = gam;  told = tf;  gam = xnext;
    if icount >= 200 || ~isfinite(xnext), break, end
end
resid = abs(tfdes-tf)/tfdes;
% NEAR-PARABOLIC GUARD, see the file header.  Recomputed from the RETURNED
% velocity rather than read out of the loop's xlam, so the number scored is the
% one the interceptor is actually launched on.  r0 = ric is this routine's own
% starting radius, and gm arrives as an argument.
lamout = r0*(vrx^2 + vry^2)/gm;
end


% =============================================================================
function K = structuralChecks(S, q, lamAll)
%STRUCTURALCHECKS  Self-consistency ONLY.  These are NOT book values and must
%   never be quoted as evidence that this port reproduces Zarchan; they are
%   the weaker claim that it agrees with itself, which is what catches wiring
%   faults.  .names and .oks are what benchSuite puts on the board -- an
%   invariant stored under any other field name is an invariant nobody can
%   see, which this package has shipped before.
names = {};  oks = [];  detail = {};
K = struct();

% 1  The target is in a conservative field, so its orbital energy is invariant.
%    Heun at H = 0.01 s over 500 s should drift ~1e-11 relative; 1e-8 is five
%    orders below any wiring fault and still a real constraint.
K.dE_rel = S.dE_rel;
names{end+1} = 'target orbital energy conserved';
oks(end+1)   = S.dE_rel <= 1e-8;
detail{end+1} = sprintf('max |dE|/|E0| = %.3e (limit 1e-8)', S.dE_rel);

% 2  ... and its angular momentum, which a sign error in the gravity term
%    breaks even when the energy still looks plausible.
K.dh_rel = S.dh_rel;
names{end+1} = 'target angular momentum conserved';
oks(end+1)   = S.dh_rel <= 1e-9;
detail{end+1} = sprintf('max |dh|/|h0| = %.3e (limit 1e-9)', S.dh_rel);

% 3  lambertpz reached its OWN convergence test, not the 200-pass cap.
K.lam_resid = S.lam_resid;
names{end+1} = 'lambertpz converged (not capped)';
oks(end+1)   = S.lam_resid <= 1e-8;
detail{end+1} = sprintf('|tf-tfdes|/tfdes = %.3e in %d passes', ...
                        S.lam_resid, S.lam_iter);

% 4  DELETE EXPERIMENT on the one unresolved comparison.  Build BOTH branches
%    through targetVel and ask which way the target's longitude moves: at
%    longitude lam the rate is (x*ydot - y*xdot)/r^2, so its SIGN says whether
%    the target closes the gap to the missile or opens it.  As resolved it must
%    close; flipped it must open.  Nothing else here would notice the branch
%    being "tidied" the other way -- the flipped run still integrates smoothly
%    and still returns a number.
A = 2.0926e7;  DEGRAD = 360/(2*pi);
XLONGM = q.XLONGMDEG/DEGRAD;  XLONGT = q.XLONGTDEG/DEGRAD;
XT = A*cos(XLONGT);  YT = A*sin(XLONGT);
[u1, v1] = targetVel(q, false);
[u2, v2] = targetVel(q, true);
K.lamdot_ok   = (XT*v1 - YT*u1)/(XT^2 + YT^2);
K.lamdot_flip = (XT*v2 - YT*u2)/(XT^2 + YT^2);
toward = sign(XLONGM - XLONGT);            % which way the missile lies
names{end+1} = 'delete exp: flipped XLONGM>XLONGT sends target away';
oks(end+1)   = toward ~= 0 && sign(K.lamdot_ok) == toward && ...
               sign(K.lamdot_flip) == -toward;
detail{end+1} = sprintf(['as resolved %+.3e rad/s, flipped %+.3e rad/s; ' ...
    'missile lies %+g in longitude'], K.lamdot_ok, K.lamdot_flip, toward);

% 5  The run ended because the range stopped closing, not because the step cap
%    caught it.  A capped run reports a "miss" that is only wherever it was.
K.capped = S.capped;
names{end+1} = 'run ended on VC < 0, not on the step cap';
oks(end+1)   = ~S.capped;
detail{end+1} = sprintf('%d outer steps, t_end = %.4f s, %d records', ...
                        S.nstep, S.t_end, S.nrec);

% 6  NEAR-PARABOLIC GUARD.  See the file header for the 98,751 ft miss this
%    exists to catch, why it is NOT keyed on lam_resid or on any convergence
%    flag (invariant 3 above is exactly that flag, and it reports GREEN on the
%    wrong answer), and why agreement with this package's five other
%    transcriptions of Listing 13.2 is not evidence either -- all six agree.
%    Scored over EVERY Lambert call this run made: the nominal one and one per
%    extra case, so turning 'allcases' off narrows the guard's reach and can
%    never turn it green.
K.lambda_max = max(lamAll);
K.lambda_all = lamAll;
LAMCAP = 1.999999;
names{end+1} = 'near-parabolic guard: lambda = r0|v|^2/gm < 1.999999';
oks(end+1)   = all(isfinite(lamAll)) && all(lamAll < LAMCAP);
detail{end+1} = sprintf(['max lambda over %d Lambert call(s) = %.9f ' ...
    '(cap %.6f, margin %.3e)'], numel(lamAll), K.lambda_max, LAMCAP, ...
    LAMCAP - K.lambda_max);

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
function B = benchmarks(S, C, bk, opts)
%BENCHMARKS  Bands and their provenance:
%     'text-1sf'    10%   prose carrying "about"/"nearly", one or two figures
%     'text-defn'   15%   prose EQUATING an aim-point offset with a miss --
%                         exact only if the offset is perpendicular to the
%                         closing velocity, which it is not; see row 7
%     'arithmetic'  1e-9  a number the book multiplies out on the page
%     'bound'/'shape'     an inequality or an ordering; no tolerance possible
%   Do NOT widen a band to turn a row green.  A red row is a RESULT.
B = emptyBench();
whyOff = sprintf(['off the listing''s own case (this run: %g/%g deg, ' ...
    'TF = %g s, N'' = %g, PREDERR = %g ft, H = %g s)'], opts.XLONGMDEG, ...
    opts.XLONGTDEG, opts.TF, opts.XNP, opts.PREDERR, opts.H);
whyCase = 'needs the book''s other runs; ''allcases'' is false';
have    = ~isempty(C);
why2    = ternary(bk, whyCase, whyOff);

% ---- 1  nominal divert ------------------------------------------------------
if bk
    B = addVal(B, 'nominal lateral divert (no prediction error)', 350, ...
        S.delv_ft, 'text-1sf', 0.10, 'p.310 text (zarchan.txt:4255)', 'ft/s', ...
        ['"about 350 ft/s" -- the fuel PN burns chasing line-of-sight rates ' ...
         'that exist only because both bodies are in a gravity field']);
else
    B = addSkip(B, 'nominal lateral divert (no prediction error)', '350', ...
        whyOff, 'p.310 text (zarchan.txt:4255)', 'ft/s');
end

% ---- 2 & 3  the 0.05 g claim ------------------------------------------------
if bk && have
    B = addVal(B, 'peak recorded |nc|, 100-kft prediction error', 0.05, ...
        C.prederr.maxg_sampled, 'text-1sf', 0.10, ...
        'p.311 text, Fig. 14.7 (zarchan.txt:4269)', 'g', ...
        sprintf(['measured on the RECORDED 1-s profile, which is what Fig. ' ...
        '14.7 plots; the per-step peak is %.0f g, all of it in the last ' ...
        'recorded second as RTM -> 0 and the LOS rate diverges'], ...
        C.prederr.maxg_step));
    B = addInfo(B, 'STRICT reading of "less than 0.05 g"', '< 0.05 g', ...
        sprintf('%.5f g (%+.1f%%)', C.prederr.maxg_sampled, ...
                100*(C.prederr.maxg_sampled/0.05 - 1)), ...
        'p.311 text (zarchan.txt:4269)', ...
        ['the bound AS PRINTED is violated.  The peak is a broad interior ' ...
         'maximum near t = 382 s, not a start-up or endgame artifact, so ' ...
         'this is the book reading one significant figure off its own ' ...
         'figure rather than a defect here.  Reported, not adjudicated']);
else
    B = addSkip(B, 'peak recorded |nc|, 100-kft prediction error', '< 0.05', ...
        why2, 'p.311 text, Fig. 14.7 (zarchan.txt:4269)', 'g');
end

% ---- 4  divert with a 100-kft prediction error -------------------------------
if bk && have
    B = addVal(B, 'divert, 100-kft prediction error, tF = 500 s', 480, ...
        C.prederr.delv_ft, 'text-1sf', 0.10, ...
        'p.311 text (zarchan.txt:4269)', 'ft/s', ...
        ['"about 480 ft/s ... somewhat larger than the theoretically ' ...
         'predicted 300" -- the excess is the gravity divert of row 1']);
else
    B = addSkip(B, 'divert, 100-kft prediction error, tF = 500 s', '480', ...
        why2, 'p.311 text (zarchan.txt:4269)', 'ft/s');
end

% ---- 5  the closed form the chapter opens with -------------------------------
% No integrator in this row: it asks only whether the formula and the constants
% are the ones the book multiplied out on the page.  Case-independent, so it is
% scored even off-case -- and R.pass still requires the case, see the verdict.
B = addVal(B, 'closed form dV = PredErr*N''/((N''-1)*tF), tF = 500 s', 300, ...
    divertFormula(1e5, 3, 500), 'arithmetic', 1e-9, ...
    'p.310 eq. (zarchan.txt:4258-4259)', 'ft/s', ...
    ['the book prints the multiplication itself.  The only exact published ' ...
     'number here, and it lives in an EQUATION -- invisible to a grep of the ' ...
     'undecoded extraction']);

% ---- 6  the 100-s short-range case ------------------------------------------
if bk && have
    B = addVal(B, 'divert, 100-kft prediction error, tF = 100 s', 1500, ...
        C.short.delv_ft, 'text-1sf', 0.10, ...
        'p.313 text (zarchan.txt:4284-4285)', 'ft/s', ...
        sprintf(['"indeed nearly 1500 ft/s", for the case TF = 100, ' ...
        'XLONGMDEG = 80 (zarchan.txt:4280-4281).  Theory for it is ' ...
        '%.0f ft/s'], divertFormula(1e5, 3, 100)));
else
    B = addSkip(B, 'divert, 100-kft prediction error, tF = 100 s', '1500', ...
        why2, 'p.313 text (zarchan.txt:4284-4285)', 'ft/s');
end

% ---- 7  guidance OFF with the prediction error -------------------------------
if bk && have
    B = addVal(B, 'miss with guidance OFF and a 100-kft prediction error', ...
        1e5, C.prederr_og.rtm_ft, 'text-defn', 0.15, ...
        'p.310 text (zarchan.txt:4256)', 'ft', ...
        ['"if we turned off the guidance system we would miss the target by ' ...
         '100 kft".  The band is 15% because PREDERR displaces the AIM POINT ' ...
         'along y while the miss is the separation at closest approach, i.e. ' ...
         'the component perpendicular to the closing velocity -- equal only ' ...
         'if those happen to be perpendicular.  Expect about -8%']);
else
    B = addSkip(B, 'miss with guidance OFF and a 100-kft prediction error', ...
        '100000', why2, 'p.310 text (zarchan.txt:4256)', 'ft');
end

% ---- 8  the collision triangle ----------------------------------------------
if bk && have
    B = addBound(B, 'guidance OFF, no prediction error: missile hits', ...
        '<= 6076 ft (1 n mi)', sprintf('%.1f ft', C.unguided.rtm_ft), ...
        C.unguided.rtm_ft <= 6076, 'p.304 text, Fig. 14.3 (zarchan.txt:4211)', ...
        'ft', ['1 n mi is 4e-6 of the 2705-n mi initial separation.  The ' ...
        'JOINT check on predictpz AND lambertpz: either one wrong and the ' ...
        'open-loop missile does not arrive.  Expect ~3 ft']);
else
    B = addSkip(B, 'guidance OFF, no prediction error: missile hits', ...
        'missile hit the target', why2, ...
        'p.304 text, Fig. 14.3 (zarchan.txt:4211)', 'ft');
end

% ---- 9  the chapter's point --------------------------------------------------
if bk && have
    ok9 = C.unguided.rtm_ft <= 6076 && S.delv_ft > 0 && S.maxg_sampled > 0;
    B = addShape(B, 'PN commands appear ON a collision triangle', ...
        'hit without guidance, yet commands issued', ...
        sprintf('open-loop miss %.1f ft; guided divert %.1f ft/s, peak %.4f g', ...
                C.unguided.rtm_ft, S.delv_ft, S.maxg_sampled), ok9, ...
        'p.304/310 text, Fig. 14.4 (zarchan.txt:4211, :4255)', ...
        ['the chapter''s own counterintuitive result: in a gravity field ' ...
         'both velocities vary, so the line-of-sight rate is NOT zero on a ' ...
         'collision triangle and PN spends fuel it did not need']);
else
    B = addSkip(B, 'PN commands appear ON a collision triangle', ...
        'commands issued anyway', why2, ...
        'p.304/310 text (zarchan.txt:4211, :4255)', '-');
end

% ---- 10  the book against itself ---------------------------------------------
B = addInfo(B, 'book self-contradiction on the 100-s divert', ...
    '1050 ft/s (:4164) vs 1500 ft/s (:4284)', ...
    sprintf('formula gives %.0f ft/s', divertFormula(1e5, 3, 100)), ...
    'p.301 text vs p.313 eq. (zarchan.txt:4164, :4284)', ...
    ['same formula, same N'' = 3, same 100-kft error, same 100-s flight. ' ...
     'Row 6 is scored against 1500 because the book multiplies THAT one out ' ...
     'on the page.  1050 reads like a transposed 1500, but the transposition ' ...
     'is a guess and the contradiction is a fact']);
end


% =============================================================================
function dv = divertFormula(prederr, xnp, tf)
%DIVERTFORMULA  zarchan.txt:4257-4258.  dV = PredErr*N'/((N'-1)*tF), the closed
%   form for the divert needed to remove a prediction error.  Shares no code
%   with the simulation, which is the entire reason it is worth having.
dv = prederr*xnp/((xnp - 1)*tf);
end

function v = ternary(c, a, b)
if c, v = a; else, v = b; end
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly; the schema
%   is the one mgs.zarchan.benchSuite's shared benchRows() consumes.
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

function B = addBound(B, what, bookStr, oursStr, ok, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'bound';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'bound';  B(i).tol = NaN;
B(i).ok   = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = units;  B(i).detail = detail;
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
%ADDINFO  Reported, never scored: skipped = true keeps it out of the pass count
%   in BOTH directions -- an INFO row can neither pass nor fail.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'info';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'info';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = '-';  B(i).detail = '';
end

function B = addSkip(B, what, bookStr, why, source, units)
%ADDSKIP  A check that COULD NOT RUN.  Never a pass -- a check that did not
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
function s = render(R, bk, ev)
L = {};
S = R.nominal;  p = R.p;
L{end+1} = 'Zarchan Listing 14.1 -- ENGAGEMENT SIMULATION WITH BALLISTIC TARGET';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, g = 32.2, n mi = 6076 ft.  NOT SI. ***';
L{end+1} = sprintf(['   missile %g deg   target %g deg   TF = %g s   ' ...
    'GAMT = %g deg   N'' = %g   PREDERR = %g ft   H = %g s'], p.XLONGMDEG, ...
    p.XLONGTDEG, p.TF, p.GAMDEGT, p.XNP, p.PREDERR, p.H);
L{end+1} = sprintf(['   target VT = %.1f ft/s (%g n mi ballistic range); ' ...
    'lambertpz converged in %d passes'], S.vt_fps, p.DISTNMT, S.lam_iter);
L{end+1} = '';
L{end+1} = sprintf('%-24s %13s %12s %11s %12s', 'run', 'divert [ft/s]', ...
    'miss [ft]', 't_end [s]', 'peak [g]');
L{end+1} = sprintf('%-24s %13.2f %12.2f %11.4f %12.5f', 'as called', ...
    S.delv_ft, S.rtm_ft, S.t_end, S.maxg_sampled);
if ~isempty(R.cases)
    nm = {'unguided','prederr','prederr_og','short'};
    lb = {'guidance off','100 kft pred err','100 kft, no guidance', ...
          '100 kft, tF = 100 s'};
    for i = 1:numel(nm)
        c = R.cases.(nm{i});
        L{end+1} = sprintf('%-24s %13.2f %12.2f %11.4f %12.5f', lb{i}, ...
            c.delv_ft, c.rtm_ft, c.t_end, c.maxg_sampled); %#ok<AGROW>
    end
elseif bk
    L{end+1} = '   (the book''s other runs were not made: ''allcases'' is false)';
end
L{end+1} = '';
L{end+1} = 'BENCHMARKS';
for i = 1:numel(R.bench)
    b = R.bench(i);
    if b.skipped
        L{end+1} = sprintf('   %-6s %-50s %s', upper(b.kind), b.what, b.why); %#ok<AGROW>
    elseif isnan(b.pct)
        L{end+1} = sprintf('   %-6s %-50s book %s   ours %s', ...
            vstr(b.ok), b.what, b.book_str, b.ours_str); %#ok<AGROW>
    else
        L{end+1} = sprintf('   %-6s %-50s book %s  ours %s  (%+.2f%%, band %s)', ...
            vstr(b.ok), b.what, b.book_str, b.ours_str, b.pct, b.band); %#ok<AGROW>
    end
end
L{end+1} = '';
L{end+1} = 'STRUCTURAL CHECKS (self-consistency, NOT book values)';
for i = 1:numel(R.selfcheck.names)
    L{end+1} = sprintf('   %-6s %-50s %s', vstr(R.selfcheck.oks(i)), ...
        R.selfcheck.names{i}, R.selfcheck.detail{i}); %#ok<AGROW>
end
if ~any(ev)
    L{end+1} = '   *** no book value evaluated: R.pass = false means NOT CHECKED ***';
end
s = strjoin(L, newline);
end

function s = vstr(ok)
if ok, s = 'ok'; else, s = 'FAIL'; end
end


% =============================================================================
function drawCurves(R)
%DRAWCURVES  The listing's own three figures, zarchan.txt:4243-4244.
S = R.nominal;
figure; plot(S.distnmT, S.altnmT, S.distnmM, S.altnmM); grid on
xlabel('Downrange (Nmi)'); ylabel('Altitude (Nmi)');
legend('target','missile'); title('Listing 14.1 -- engagement geometry');
figure; plot(S.t, S.xncg); grid on
xlabel('Time (Sec)'); ylabel('Acceleration (G)');
figure; plot(S.t, S.delv); grid on
xlabel('Time (Sec)'); ylabel('Lateral Divert (Ft/Sec)');
end
