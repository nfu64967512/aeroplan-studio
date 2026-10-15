function R = kalmanLawOptions(opts)
%MGS.ZARCHAN.KALMANLAWOPTIONS  Zarchan Listing 29.3 -- forward model of the
%   homing loop with a DISCRETE THREE-STATE KALMAN FILTER and three guidance
%   law options: PN (APN=0), augmented PN (APN=1), optimal (APN=2).
%
%   R = mgs.zarchan.kalmanLawOptions()
%   R = mgs.zarchan.kalmanLawOptions('APN', 0, 'assert', false)
%
%   Line-by-line port of zarchan.txt:10766-10784.  UNITS ARE THE BOOK'S: ft,
%   ft/s, ft/s^2, g = 32.2 ft/s^2; nothing is converted.  tF is fixed at 10 s
%   and TSTART = 0.1:0.1:10 s sweeps the instant at which a 3-g step target
%   maneuver begins.  The book's abscissa is the TIME TO GO at which it begins,
%   tgo = tF - TSTART, running 9.9 s down to 0 (:10760).
%
% ---------------------------------------------------------------------------
%   ⚠️⚠️ WHICH LINE NUMBERING THE CITATIONS USE.  EVERY citation in this file
%   -- header and .source alike -- is a grep/sed/Read line of matlab/zarchan/
%   _source/zarchan.txt.  It used to be an offset number, +187 through Ch.29,
%   and the ONE-LINE REASON WAS DIAGNOSED WRONG TWICE before it was measured.
%   *** CORRECTED 2026-08-24 BY MEASUREMENT, replacing a paragraph that said
%   "the two files do not share numbering, 15120 against 15307". ***
%     * THE TWO BOOK FILES DO SHARE NUMBERING, EXACTLY.  Measured: both have
%       15120 LF records and 15307 bare CR records, and the same sentence sits
%       at the same index in both under either counting ("the miss distance
%       results get better" is line 10837 of zarchan.txt AND of
%       zarchan_decoded.txt).  There is no raw-vs-decoded offset in any
%       chapter.  The brief's rule is right; the two headers that denied it
%       (this one, and kalmanLawAdjoint.m:40-42) were wrong.
%     * WHAT THE +187 REALLY IS: two READERS of the SAME file, not two files.
%       These extractions carry bare CR characters inside paragraphs.  grep,
%       sed and the agent Read tool split on LF only and see 15120 lines;
%       MATLAB readlines() splits on CR as well and sees 15308.  So Ch.29 text
%       at Read line 10837 is readlines index 11024, +187 -- and Ch.7 is +36,
%       Ch.20 +175, purely because that many CR splits precede them.  Nothing
%       moved; two tools counted differently.
%     * CONSEQUENCE FOR THE PACKAGE.  Cite the Read/grep number, because that
%       is what a reader checking the row will use.  This file, dualRateForward
%       and fadingMemoryMC now do (the latter two were converted 2026-08-24,
%       every line re-verified by text).  ⚠️ kalmanLawAdjoint.m and
%       dualRateAdjoint.m still carry readlines indices labelled
%       "zarchan_decoded.txt:NNNN"; those resolve only through readlines().
%   So: READ the decoded file (the maths is only legible there), CITE by the
%   Read/grep line, and locate the line by its TEXT, never by a remembered
%   number.  Every citation below was re-verified by text on 2026-08-24.
%
% ---------------------------------------------------------------------------
%   TRANSCRIPTION.  listing_29_03.m does not parse and reports ONE unresolved
%   variable-to-variable comparison.  Four repairs, three of them forced:
%     1  "while T,=(TF-1e-5)" -> "T <= (TF-1e-5)".  "," is this book's OCR of
%        "<"; T only increases, so ">=" would never terminate.
%     2  "while STEP,=1"      -> "STEP <= 1".  The body sets STEP=2 to leave.
%     3  "if S.=(TS-1e-5)"    -> "S >= (TS-1e-5)".  "." is the OCR of ">"; S is
%        zeroed on every sample, so "<=" would sample forever.
%     4  *** THE UNRESOLVED ONE.  "if T.TSTART, YDD=XNT-XNL; else YDD=0." ***
%        Resolved as  T > TSTART => MANEUVER IS ON.  Reasons, strongest last:
%          a  :10760 calls TSTART a STARTING time ("the time to go at which the
%             3-g target maneuver BEGINS is made a parameter").
%          b  "." and ".=" both occur in THIS listing and are distinct, so "."
%             alone is ">" (repair 3 fixes the sense of ".=" independently).
%          c  TSTART = 10 is plotted at tgo = 0.  Under "<" that run would have
%             the maneuver on for the WHOLE 10 s flight while being labelled
%             "maneuver occurs at tgo = 0", which is nonsense; under ">" it has
%             no maneuver and lands at zero miss, which is what an abscissa of
%             0 s-to-go has to mean.
%        Sized, not asserted: structural check 3 is a DELETE EXPERIMENT that
%        flips the comparison and reports how far the answer moves.
%
%   Reproduced deliberately, or dropped deliberately:
%     * The else branch is YDD = 0 where Listing 4.5 writes YDD = -XNL.  They
%       are identical here: before TSTART nothing disturbs the loop, so Y = YD
%       = 0 => XLAM = 0 => every residual is 0 => XNC = 0 => XNL stays exactly
%       0.  Structural check 1 measures that.
%     * XLAMD, XLAMDH and XNEW are ASSIGNED AND NEVER READ in the listing --
%       vestiges of the LOS-rate form of Listing 29.1.  Dropped; recorded here
%       so nobody re-adds them thinking a signal was lost.
%     * HEDEGF = 20 deg enters ONLY the filter's initial covariance P22; there
%       is no heading-error disturbance in this listing.
%     * XLAM is formed from the PREDICTED Y (the corrector runs after it), and
%       the filter rescales it by a DIFFERENT tgo epsilon, 1e-6 against the
%       derivative loop's 1e-5.  Harmless except on the final sample, where it
%       is Y*0.1 -- and that sample's XNC is never read.  Kept.
%     * The .0001 guard in XNPP's denominator is NOT cosmetic near intercept:
%       the true denominator goes as 0.3*x^5, so at x = tgo/T = 0.2 the guard
%       more than halves N' (24.16 against 52.24).  Kept.
%     * The Riccati is run ONCE and the 100 runs carried as a vector.  Exact,
%       not an approximation, and the book says so at :10760 -- "the flight
%       time is fixed and the gains will be the same from run to run".
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Every book value was grepped from the DECODED zarchan.txt and
%   CITED in the raw one (see the note above).  This chapter publishes almost
%   nothing but ORDERING for Listing 29.3, so ordering is most of what is
%   scored -- plus two closed-form FLOORS out of Chs. 3 and 4.
%
% ===========================================================================
%   ⭐⭐⭐ THE ORDERING, AND WHY IT IS NOW FOUR ROWS INSTEAD OF TWO.
%   *** REWRITTEN 2026-08-24 ON MEASUREMENT.  The paragraph that stood here
%   predicted the PN-vs-APN PEAK row would be the one at risk.  It is the
%   opposite: peak is green, rms is red, and the reason is in the book. ***
%
%   Ch.29 says, once, in one sentence (p.774, :10837):
%       "We can also see from the three cases that the miss distance results
%        get better as the guidance law becomes more advanced."
%   Ch.8 says, about EXACTLY this pair of laws under EXACTLY this disturbance
%   -- a 3-g step target maneuver into a single-lag flight-control system --
%   the opposite (Fig. 8.12 caption p.175, :2454, and text p.176, :2461):
%       "Fig. 8.12 Both guidance laws are comparable in terms of miss
%        distance."
%       "We can see from Fig. 8.12 that neither guidance law is superior from
%        a miss distance point of view (assuming the system is linear and we
%        do not have acceleration saturation).  Augmented proportional
%        navigation yields smaller miss distances for shorter flight times,
%        whereas proportional navigation yields smaller miss distances for
%        longer flight times."
%   The two abscissae are the same variable, and the book is the one that says
%   so (:10786): "adjoint time can either be interpreted as missile flight time
%   or time to go at which the target maneuver started" (again at :1733, "flight
%   time and time to go are interchangeable for this system").
%
%   WHAT THIS PORT MEASURES, on the listing's own 100-run sweep:
%       law     rms miss    peak |miss|      signed range
%       PN        8.640 ft     24.297 ft     [ -2.68,  24.30]
%       APN       9.689 ft     19.883 ft     [-19.88,  19.66]
%       optimal   0.892 ft      3.270 ft     [ -1.16,   3.27]
%   so PEAK is monotone (:10837 holds) and RMS is not (:10837 fails on the
%   PN->APN half).  |APN| is below |PN| for tgo <= ~2.4 s and above it for
%   larger tgo -- Ch.8's crossover, in Ch.29's own configuration.
%
%   THE FOUR CAUSES WERE SEPARATED BY MEASUREMENT, not by argument:
%     (a) NOT a mis-implemented APN.  The listing's own algebra is C3 = 0.5 N'
%         on the FILTER's nThat (:10740-10746, and the listing at :10782).
%         Substituting the TRUE target acceleration at that one point -- one
%         line, nothing else touched -- gives APN rms 5.190 ft and peak 11.31
%         ft, i.e. BELOW PN's 8.640 / 24.297 and monotone, which is what the
%         book asserts.  The law is wired right; the estimator is what costs.
%     (b) CROSS-PORT AGREEMENT on the same point.  Ch.26's independent bakeoff
%         (mgs.zarchan.guidanceLawBakeoff, Listing 26.6, PERFECT state feedback
%         and the true maneuver handed to the law) measures APN peak below PN
%         at every one of its three plant cases: 4.691/7.541/31.83 ft against
%         8.251/13.24/55.67, ratio 0.568/0.570/0.572.  Two ports, two chapters,
%         one answer: with nT known, APN wins; with nT estimated, it does not.
%     (c) NOT the step size.  rms (8.640, 9.689, 0.892) at H = 0.01 s becomes
%         (8.639, 9.684, 0.896) at H = 5e-4.  Converged; the ordering never
%         moves.
%     (d) NOT the statistic being un-booklike.  ⚠️ The tempting reading -- "rms
%         over a swept parameter is not a quantity Zarchan forms" -- IS FALSE
%         and was checked before it was used.  He names it: MUDNT, the miss due
%         to a target maneuver with UNIFORMLY DISTRIBUTED starting time,
%         MUDNT = nT sqrt((1/tF) int_0^tF MNT^2 dt) (:1340-1343), which is the
%         rms of exactly this curve.  So the rms row is a fair reading of
%         :10837 and it is red on the merits.
%   Hence: three SCORED rows for the parts of the claim the book supports (the
%   optimal law beating both; the envelope ordering; Ch.8's crossover), and one
%   INFO row that prints the rms failure with both chapters cited.  THE RED
%   RESULT IS NOT BANDED AWAY -- it is reported as what it is, the book
%   contradicting itself, with the more specific chapter winning.
%
%   ROW TEETH, MEASURED BY MUTATION 2026-08-24 (not asserted):
%       mutation of the APN term      crossover row   peak-ordering row
%       delete it (C3 = 0)                FAIL             FAIL
%       flip its sign (C3 = -0.5 N')      FAIL             FAIL
%       wrong factor (C3 = 1.0 N')        pass             FAIL
%   The two rows together catch all three; neither does alone.  The
%   "optimal beats both" row survives all three and is a WEAK detector -- it is
%   carried because it is what both chapters state, not because it guards.
% ===========================================================================
%
%   THE TWO FLOORS ARE BOOK EQUATIONS, and they replace the figure-frame rows.
%   Drive the same control gains with PERFECT states and no sampler and Ch.3
%   gives the closed form MNT|N'=3 = 0.5 tF^2 exp(-tF/T) (:1345, also :888-890),
%   peaking at tgo = 2T -> 6.537 ft here; Ch.4 integrates it into
%   MUDNT|N'=3 = (nT/4) sqrt(T^5/tF [3 - e^-2x (2x^4+4x^3+6x^2+6x+3)]),
%   x = tF/T (:1355-1357) -> 2.338 ft, and that closed form agrees with a
%   quadrature of :1345 to 2.2e-16.  Estimates cannot beat perfect information,
%   so both are LOWER bounds on the PN curve.  Measured: peak 24.297 = 3.72x
%   the peak floor, rms 8.640 = 3.69x the rms floor -- the sampler-plus-filter
%   cost is the SAME factor under both statistics, which is a coherence check
%   the port would fail if either statistic were wrong.  N' = 3 only; off-book
%   XNP SKIPS both rather than quoting a formula that does not apply.
%
%   ⚠️⚠️ THE FIGURE FRAMES ARE NOT SCOREABLE, AND THAT IS A MEASUREMENT.
%   Three rows used to score "peak |miss| <= the top y-axis tick of Fig. 29.9 /
%   29.10 / 29.11".  Two were red.  They are gone -- not widened, DELETED --
%   because the outermost extracted tick is a labelled gridline, not an axis
%   limit, and Chapter 29 lets you prove it.  Five figures in this chapter have
%   an extracted tick block and a port that computes the curve:
%       fig / ticks            port                          measured range
%       29.5   -5 .. 15        dualRateAdjoint beta = 0.3    [ -5.97,  16.55]
%       29.6  -20 .. 100       dualRateAdjoint beta = 0.8    [-36.11, 101.59]
%       29.9    0 .. 20        this port, PN                 [ -2.68,  24.30]
%       29.10 -20 .. 10        this port, APN                [-19.88,  19.66]
%       29.11  -1 .. 3         this port, optimal            [ -1.16,   3.27]
%   NINE of the ten tick extremes are exceeded by the curve, on two independent
%   ports, in the same direction every time; the tenth (-20 against -19.88) is
%   inside by 0.6%.  A containment test that no case in the chapter passes is
%   not a benchmark.
%
%   ⭐ TWO FACTS SETTLE IT, AND BOTH ARE IN THE BOOK'S OWN SOURCE CODE, not in
%   an argument about extraction quality:
%     1  LISTINGS 29.3 AND 29.4 CONTAIN NO axis() CALL (:10784, :10819), so
%        Figs. 29.9-29.11 are AUTO-SCALED and their limits were never typed by
%        anybody.  There is no author decision there to score.  Contrast
%        Listing 29.1 (:10659) and Listing 7.2 (:2051), which DO hand-type
%        axis([00,10,-40,100]) and axis([00,10,00,4]) -- an author's axis()
%        argument IS a claim about his own curve, and the ports that own those
%        figures (mgs.zarchan.dualRateForward, mgs.zarchan.fadingMemoryMC)
%        keep it as a scored row.  Two kinds of "frame", one scoreable.
%     2  THE EXTRACTION PROVABLY DROPS TICK ENTRIES.  Listing 29.1 types
%        axis([00,10,-40,100]); the tick block for its figure extracts as
%        "100 80 60 40 20 0 -20" -- the -40 the author himself wrote is NOT
%        THERE.  So a missing outer tick is demonstrated, not hypothesised,
%        and the "-20" in the 29.6 row of the table above is an artefact of
%        the same loss.  (dualRateAdjoint's -36.11 is inside the author's real
%        -40; its 101.59 is 1.6% over a ceiling he hard-clipped at 100.)
%   Corroborating: the y-axis LABEL in all five blocks reads "Altitude (km)"
%   where every one of these listings writes ylabel('Miss (Ft)') -- it is on 22
%   figure blocks in this file, i.e. an extraction artefact and not a book
%   error -- and Fig. 29.10's tick list is the only one printed bottom-to-top,
%   ending at 10 while its own curve reaches -19.88.  The ticks are kept as an
%   INFO row beside our ranges.  mgs.zarchan.kalmanLawAdjoint reached the same
%   verdict independently and SKIPS them; the package now says one thing, not
%   two.  ⚠️ THIS PORT DOES NOT ADD ITS OWN SKIP ROW for those curves --
%   benchSuite renders a SKIP as an unresolved row, the sibling already carries
%   one for the same three figures, and two unresolved rows for one fact reads
%   on the board as two problems.
%
%   C4's SIGN is a book-versus-book case.  Ch.29 (:10753) and Ch.8 (:2540) both
%   print C4 without a minus, but the law they come from is printed in full at
%   :2520 as nc = (N'/tgo^2)[y + y'tgo + 0.5 nT tgo^2 - nL T^2(e^-x + x - 1)],
%   where the minus survives -- and the two chapters then agree, since
%   T^2/tgo^2 = 1/x^2.  Scored on :2520, both lines cited.
%
%   The N' limit (:2526, "approaching 3") is scored at x = 1e4, where the .0001
%   guard is negligible and N' = 3 + 6/x gives 3.0006.  NOT at the sweep's own
%   x = 19.8, where N' = 3.33 and calling that "3" would need an 11% band.
%
%   THE LISTING 29.4 ADJOINT (:10820) IS PORTED after all, as
%   mgs.zarchan.kalmanLawAdjoint, which runs its own Listing 29.3 twin and
%   scores the adjoint-vs-forward identity per law.  Still an INFO row here --
%   this file does not re-run it -- but it now points at the port instead of
%   claiming the comparison is impossible.  MEASURED 2026-08-24: that port's
%   independent 29.3 agrees with this one to every printed digit at all nine of
%   its integer tgo (PN 21.8923 / APN 19.3250 / optimal 1.8941 at tgo = 1 s),
%   which is the check that says the numbers above are the listing's and not
%   this file's.
%
%   WHAT THIS PRINTS.  *** MEASURED 2026-08-24 on R2025b, 0.05 s, replacing a
%   DERIVED paragraph whose central prediction was wrong. ***  Arithmetic rows
%   green to the digit shown (100 exactly; -0.367879; 3.0006 against 3,
%   +0.02%).  Six structural invariants hold.  Ordering: peak green, crossover
%   green, optimal-beats-both green, rms INFO and FALSE.  Both floors green at
%   3.7x.  R.pass = 1.
%
%   OUTPUTS
%     R.tstart, R.tgos   the sweep [s]; tgos = TF - tstart, the book's abscissa
%     R.law(k)           .apn .name .miss_ft .rms_ft .peak_ft .peak_tgo_s .gains
%     R.kalman           .K, .Pdiag (3-by-nUpd), .t_s
%     R.bench            benchmark rows; R.selfcheck has .names .oks .detail
%     R.pass             true only if a row was SCORED and everything passed
%     R.verdict          '' when clean; ALWAYS FORMED.  'assert' picks the
%                        reaction only: true errors, false WARNS and returns R
%                        complete, which is how benchSuite runs it.
%     R.units, R.p, R.summary
%
%   See also MGS.ZARCHAN.MONTECARLOHOMING, MGS.ZARCHAN.ADJOINTOPTIMAL,
%   MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 29.3 line 1, verbatim.
    opts.VM       (1,1) double {mustBeFinite, mustBePositive} = 3000.0
    opts.VC       (1,1) double {mustBeFinite, mustBePositive} = 4000.0
    opts.XNT      (1,1) double {mustBeFinite} = 96.6      % ft/s^2 = 3 g
    opts.YIC      (1,1) double {mustBeFinite} = 0.0
    opts.HEDEGF   (1,1) double {mustBeFinite} = 20.0      % deg, COVARIANCE only
    opts.XNP      (1,1) double {mustBeFinite, mustBePositive} = 3.0
    opts.SIGNOISE (1,1) double {mustBeFinite, mustBeNonnegative} = 0.001  % rad
    opts.TS       (1,1) double {mustBeFinite, mustBePositive} = 0.1       % s
    opts.TAU      (1,1) double {mustBeFinite, mustBePositive} = 0.5       % s
    opts.TF       (1,1) double {mustBeFinite, mustBePositive} = 10.0      % s
    opts.H        (1,1) double {mustBeFinite, mustBePositive} = 0.01      % s
    % [] means the listing's own sweep, TSTART = 0.1:0.1:TF.
    opts.tstart   double {mustBeReal, mustBeFinite} = []
    % Which law options to run.  0 = PN, 1 = augmented PN, 2 = optimal.  The
    % ordering rows need all three; asking for fewer SKIPS them rather than
    % silently scoring a two-way comparison as if it were the book's three-way.
    opts.APN      double {mustBeMember(opts.APN, [0 1 2])} = [0 1 2]
    % NOT nargin: with an arguments block that declares only name-value
    % options MATLAB reports nargin = 0 however many were passed, so an
    % "if nargin == 0" mode switch here would never fire.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

TF = opts.TF;  H = opts.H;  TS = opts.TS;  TAU = opts.TAU;  XNP = opts.XNP;

if H >= TAU
    error('mgs:zarchan:kalmanLawOptions:stiff', ...
        ['H = %g s is not smaller than the flight-control time constant ' ...
         'TAU = %g s.  RK2 does not blow up here, it returns a smooth and ' ...
         'entirely wrong curve, so this is refused rather than warned about.'], ...
        H, TAU);
end
if abs(TS/H - round(TS/H)) > 1e-9
    error('mgs:zarchan:kalmanLawOptions:sampler', ...
        ['the sampling interval TS = %g s is not an integer number of ' ...
         'integration steps H = %g s.  The listing tests "S >= TS - 1e-5" on ' ...
         'an accumulator incremented by H, so a non-multiple slips a sample ' ...
         'every few periods -- a jitter that plots perfectly smoothly.'], TS, H);
end
tstart = opts.tstart(:).';
if isempty(tstart)
    tstart = TS:TS:TF;                 % the listing's own for-loop, 0.1:0.1:10
end
if any(tstart <= 0) || any(tstart > TF)
    error('mgs:zarchan:kalmanLawOptions:tstart', ...
        'every TSTART must lie in (0, TF]; TF = %g s.', TF);
end
laws = unique(opts.APN(:).', 'stable');
if isempty(laws)
    error('mgs:zarchan:kalmanLawOptions:nolaw', ...
        ['no guidance law option was requested.  Returning an empty result ' ...
         'would render as a port that ran and found nothing wrong.']);
end

% =========================================================================
%   LISTING 29.3, once per law option.  Vectorised across TSTART: the Riccati
%   recursion does not see TSTART, and the book says so at :10760.
% =========================================================================
lawNames = {'PN', 'APN', 'OPT'};
LAW = struct('apn', {}, 'name', {}, 'miss_ft', {}, 'rms_ft', {}, ...
             'peak_ft', {}, 'peak_tgo_s', {}, 'gains', {});
KG = [];  PD = [];  nUpd = 0;
tgos = TF - tstart;
for k = 1:numel(laws)
    [miss, KGk, PDk, nUpd] = forwardSweep(laws(k), opts.VM, opts.VC, opts.XNT, ...
        opts.YIC, opts.HEDEGF, XNP, opts.SIGNOISE, TS, TAU, TF, H, tstart, false);
    [pk, ipk] = max(abs(miss));
    LAW(k).apn        = laws(k);
    LAW(k).name       = lawNames{laws(k)+1};
    LAW(k).miss_ft    = miss;
    LAW(k).rms_ft     = sqrt(sum(miss.^2)/numel(miss));
    LAW(k).peak_ft    = pk;
    LAW(k).peak_tgo_s = tgos(ipk);
    LAW(k).gains      = KGk;
    if k == 1
        KG = KGk;  PD = PDk;
    end
end

R = struct();
R.tstart = tstart;
R.tgos   = tgos;
R.law    = LAW;
R.kalman = struct('K', KG, 'Pdiag', PD, 't_s', (1:nUpd)*TS);
R.units  = 'ft, ft/s, ft/s^2, s; g = 32.2 ft/s^2 (HEDEGF in deg, /57.3)';
R.p = struct('VM',opts.VM, 'VC',opts.VC, 'XNT',opts.XNT, 'YIC',opts.YIC, ...
    'HEDEGF',opts.HEDEGF, 'XNP',XNP, 'SIGNOISE',opts.SIGNOISE, 'TS',TS, ...
    'TAU',TAU, 'TF',TF, 'H',H, 'n_runs',numel(tstart), 'n_updates',nUpd, ...
    'APN',laws, 'units',R.units);

R.selfcheck = structuralChecks(R, opts, laws, tstart, TS, TAU, TF, XNP, nUpd);

if opts.benchmark
    R.bench = benchmarks(R, laws, tstart, tgos, TS, TAU, TF, opts.XNT, XNP);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  "Not checked" must never render as "fine".
    R.pass  = false;
end

R.summary = render(R, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(R); end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false, so nothing was compared against the book.';
elseif ~any(ev)
    msg = 'not one book value could be evaluated.';
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question. ' ...
        'First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf(['%d of %d evaluated book values are outside tolerance ' ...
        '(first: %s, book %s, ours %s).'], numel(bad), sum(ev), ...
        R.bench(bad(1)).what, R.bench(bad(1)).book_str, R.bench(bad(1)).ours_str);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:kalmanLawOptions:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN COMPLETE.  benchSuite runs every port with assert
        % off precisely so one red row cannot hide the rest of the board.
        warning('mgs:zarchan:kalmanLawOptions:benchmark', ...
            '%s  (''assert'' is false; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [miss, KG, PD, nUpd] = forwardSweep(APN, VM, VC, XNT, YIC, HEDEGF, ...
        XNP, SIGNOISE, TS, TAU, TF, H, tstart, flipMan)
%FORWARDSWEEP  Listing 29.3, zarchan.txt:10766-10784.  Every constant arrives
%   as an ARGUMENT: MATLAB local functions cannot see the caller's workspace.
%
%   flipMan is the DELETE EXPERIMENT for the one unresolved comparison.  false
%   is the book (maneuver ON for T > TSTART); true is the rejected reading.

n = numel(tstart);
Y = YIC*ones(1,n);  YD = zeros(1,n);  XNL = zeros(1,n);
YH = zeros(1,n);  YDH = zeros(1,n);  XNTH = zeros(1,n);  XNC = zeros(1,n);
YDD = zeros(1,n);  XNLD = zeros(1,n);  XLAM = zeros(1,n);
T = 0.0;  S = 0.0;

TS2 = TS*TS;  TS3 = TS2*TS;  TS4 = TS3*TS;  TS5 = TS4*TS;
PHIN = XNT*XNT/TF;
RTM = VC*TF;  SIGPOS = RTM*SIGNOISE;  SIGN2 = SIGPOS^2;
P11 = SIGN2;  P12 = 0.;  P13 = 0.;
P22 = (VM*HEDEGF/57.3)^2;  P23 = 0.;  P33 = XNT*XNT;

nmax = ceil(TF/TS) + 2;
KG = zeros(3,nmax);  PD = zeros(3,nmax);  nUpd = 0;

while T <= (TF - 1e-5)
    YOLD = Y;  YDOLD = YD;  XNLOLD = XNL;

    % Zarchan's two-pass predictor loop.  Pass 2 takes a full Euler step and
    % advances T INSIDE the loop, which is what makes the second derivative
    % evaluation happen at t+H.  Moving that line out silently degrades the
    % scheme to first order while still running smoothly.
    for pass = 1:2
        if pass == 2
            % ORDER MATTERS: Y uses the OLD YD, YD then uses the OLD YDD.
            Y = Y + H*YD;  YD = YD + H*YDD;  XNL = XNL + H*XNLD;  T = T + H;
        end
        TGO  = TF - T + 0.00001;
        XLAM = Y/(VC*TGO);
        XNLD = (XNC - XNL)/TAU;
        % The unresolved comparison, resolved as "maneuver STARTS at TSTART".
        % The else branch is the listing's own YDD = 0, not -XNL; identical
        % here because XNL is provably 0 before TSTART (see the header).
        if flipMan
            on = T < tstart;
        else
            on = T > tstart;
        end
        YDD = double(on) .* (XNT - XNL);
    end

    Y   = 0.5*(YOLD   + Y   + H*YD);
    YD  = 0.5*(YDOLD  + YD  + H*YDD);
    XNL = 0.5*(XNLOLD + XNL + H*XNLD);

    S = S + H;
    if S >= (TS - 1e-5)
        S = 0.;
        % NOTE the second, DIFFERENT tgo epsilon -- 1e-6 here against 1e-5 in
        % the derivative loop.  The book's, kept; see the header.
        TGO = TF - T + 0.000001;  RTM = VC*TGO;
        SIGPOS = RTM*SIGNOISE;  SIGN2 = SIGPOS^2;

        % ---- Riccati.  Scalar: it does not depend on TSTART (:10760). ------
        M11 = P11 + TS*P12 + .5*TS2*P13 + TS*(P12 + TS*P22 + .5*TS2*P23);
        M11 = M11 + .5*TS2*(P13 + TS*P23 + .5*TS2*P33) + TS5*PHIN/20.;
        M12 = P12 + TS*P22 + .5*TS2*P23 + TS*(P13 + TS*P23 + .5*TS2*P33) ...
              + TS4*PHIN/8.;
        M13 = P13 + TS*P23 + .5*TS2*P33 + PHIN*TS3/6.;
        M22 = P22 + TS*P23 + TS*(P23 + TS*P33) + PHIN*TS3/3.;
        M23 = P23 + TS*P33 + .5*TS2*PHIN;
        M33 = P33 + PHIN*TS;
        K1 = M11/(M11 + SIGN2);  K2 = M12/(M11 + SIGN2);  K3 = M13/(M11 + SIGN2);
        P11 = (1.-K1)*M11;  P12 = (1.-K1)*M12;  P13 = (1.-K1)*M13;
        P22 = -K2*M12 + M22;  P23 = -K2*M13 + M23;  P33 = -K3*M13 + M33;

        % ---- measurement and three-state update ----------------------------
        XLAMNOISE = 0.;                       % the listing's own zero
        YSTAR = RTM*(XLAM + XLAMNOISE);
        RES  = YSTAR - YH - TS*YDH - .5*TS*TS*(XNTH - XNL);
        YH   = K1*RES + YH + TS*YDH + .5*TS*TS*(XNTH - XNL);
        YDH  = K2*RES + YDH + TS*(XNTH - XNL);
        XNTH = K3*RES + XNTH;

        [C1, C2, C3, C4] = controlGains(APN, TGO, TAU, XNP);
        XNC = C1*YH + C2*YDH + C3*XNTH + C4*XNL;

        nUpd = nUpd + 1;
        KG(:,nUpd) = [K1; K2; K3];
        PD(:,nUpd) = [P11; P22; P33];
    end
end

miss = Y;
KG = KG(:,1:nUpd);  PD = PD(:,1:nUpd);
end


% =============================================================================
function [C1, C2, C3, C4, XNPP] = controlGains(APN, TGO, TAU, XNP)
%CONTROLGAINS  nc = C1*yhat + C2*ydothat + C3*nThat + C4*nL, zarchan.txt:10714
%   with the gains at :10716-10721 (PN), :10740-10746 (augmented PN, C3 =
%   0.5 N'), :10747-10753 (optimal).  ⚠️ APN's C3 multiplies the FILTER's
%   nThat, not the truth -- that is the listing's own algebra (:10782) and it
%   is why the rms ordering comes out the way it does; see the header.
%
%   C4's SIGN comes from :2520, where the optimal law is printed in full with
%   "- nL*T^2*(e^-x + x - 1)" inside the zero-effort-miss bracket.  Both places
%   that print C4 on its own (:2540, :10753) lost the minus in extraction.
if APN == 0
    XNPP = XNP;
    C1 = XNP/TGO^2;  C2 = XNP/TGO;  C3 = 0.;  C4 = 0.;
elseif APN == 1
    XNPP = XNP;
    C1 = XNP/TGO^2;  C2 = XNP/TGO;  C3 = .5*XNP;  C4 = 0.;
else
    X    = TGO/TAU;
    TOP  = 6.*X*X*(exp(-X) - 1. + X);
    BOT1 = 2*X*X*X + 3. + 6.*X - 6.*X*X;
    BOT2 = -12.*X*exp(-X) - 3.*exp(-2.*X);
    % The .0001 is the book's guard against BOT -> 0 as x -> 0.  It is NOT
    % negligible near intercept: BOT ~ 0.3*x^5, so it dominates below x ~ 0.2.
    XNPP = TOP/(.0001 + BOT1 + BOT2);
    C1 = XNPP/TGO^2;  C2 = XNPP/TGO;  C3 = .5*XNPP;
    C4 = -XNPP*(exp(-X) + X - 1.)/(X*X);
end
end


% =============================================================================
function K = structuralChecks(R, opts, laws, tstart, TS, TAU, TF, XNP, nUpd)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never to be quoted
%   as evidence that this port reproduces Zarchan -- only that it agrees with
%   itself, which is what catches wiring faults.
%
%   .names and .oks are what benchSuite reads.  A port that computes twenty
%   invariants and stores them under private field names puts ZERO structural
%   rows on the board, and the board then looks green.  Both fields, same
%   length, every time.
K = struct();
names = {};  oks = [];  detail = {};

% 1  The no-maneuver run.  TSTART = TF means the comparison T > TSTART never
%    fires, so there is NO disturbance anywhere in the loop and the miss must
%    be EXACTLY zero -- not small, zero.  This is also the proof that the
%    listing's "else YDD = 0" and Listing 4.5's "else YDD = -XNL" coincide.
j = find(abs(tstart - TF) < 1e-12, 1);
names{end+1} = 'no-maneuver run gives identically zero miss';
if isempty(j)
    K.zero_miss = NaN;
    oks(end+1)   = false;
    detail{end+1} = 'no TSTART = TF run in the sweep, so the check cannot run';
else
    K.zero_miss = max(arrayfun(@(L) abs(L.miss_ft(j)), R.law));
    oks(end+1)   = K.zero_miss == 0;
    detail{end+1} = sprintf('worst |miss| at TSTART = TF is %.3e ft', K.zero_miss);
end

% 2  The sampler fires exactly TF/TS times.  An off-by-one in "S >= TS - 1e-5"
%    halves or doubles the filter rate and still produces a smooth curve.
K.n_updates = nUpd;
names{end+1} = 'sampler fires exactly TF/TS times';
oks(end+1)   = nUpd == round(TF/TS);
detail{end+1} = sprintf('%d updates, expected %d', nUpd, round(TF/TS));

% 3  DELETE EXPERIMENT on the one unresolved comparison.  Flip T > TSTART to
%    T < TSTART and require the answer to MOVE.  If it does not, the header's
%    resolution is untested and the argument for it is decoration.
mflip = forwardSweep(laws(1), opts.VM, opts.VC, opts.XNT, opts.YIC, ...
    opts.HEDEGF, XNP, opts.SIGNOISE, TS, TAU, TF, opts.H, tstart, true);
den = max(abs(R.law(1).miss_ft));
K.flip_rel = max(abs(mflip - R.law(1).miss_ft))/max(den, realmin);
names{end+1} = 'DELETE EXPERIMENT: flipping T > TSTART changes the answer';
oks(end+1)   = K.flip_rel > 0.25;
detail{end+1} = sprintf(['flipped sweep differs by %.1f%% of peak miss ' ...
    '(law %s); below 25%% would mean the resolution is untested'], ...
    100*K.flip_rel, R.law(1).name);

% 4  Riccati sanity.  K1 is a gain on an innovation whose covariance is
%    M11 + SIGN2 >= M11, so 0 < K1 <= 1 identically; a diagonal of P going
%    negative is the classic covariance-update transcription error.
K.k1_min = min(R.kalman.K(1,:));  K.k1_max = max(R.kalman.K(1,:));
K.p_min  = min(R.kalman.Pdiag(:));
names{end+1} = 'Riccati sane: 0 < K1 <= 1 and diag(P) > 0';
oks(end+1)   = K.k1_min > 0 && K.k1_max <= 1 && K.p_min > 0;
detail{end+1} = sprintf('K1 in [%.4f, %.4f], min diag(P) = %.4g', ...
    K.k1_min, K.k1_max, K.p_min);

% 5  The Riccati does not see the control law.  Bit-identical gain sequences
%    across the law options, or the filter has been coupled to the guidance.
if numel(R.law) >= 2
    d = 0;
    for i = 2:numel(R.law)
        d = max(d, max(abs(R.law(i).gains(:) - R.law(1).gains(:))));
    end
    K.gain_spread = d;
    names{end+1} = 'all law options share a bit-identical Kalman gain sequence';
    oks(end+1)   = d == 0;
    detail{end+1} = sprintf('max gain difference across laws = %.3e', d);
end

% 6  The optimal effective navigation ratio falls monotonically with tgo, over
%    the region where the .0001 guard is negligible (x >= 1).
xg = linspace(1, 40, 400);
ng = arrayfun(@(x) nprimeOf(x, TAU, XNP), xg);
K.nprime_max_rise = max(diff(ng));
names{end+1} = 'optimal N'' decreases monotonically in tgo for x >= 1';
oks(end+1)   = K.nprime_max_rise <= 0;
detail{end+1} = sprintf('N''(x=1) = %.4f, N''(x=40) = %.4f, worst rise %.3e', ...
    ng(1), ng(end), K.nprime_max_rise);

K.names  = names;
K.oks    = logical(oks);
K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    jb = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{jb}, detail{jb});
end
end


function np = nprimeOf(x, TAU, XNP)
%NPRIMEOF  The optimal effective navigation ratio at x = tgo/T, through the
%   SAME code path the simulation uses -- including the book's .0001 guard.
[~, ~, ~, ~, np] = controlGains(2, x*TAU, TAU, XNP);
end


% =============================================================================
function B = benchmarks(R, laws, tstart, tgos, TS, TAU, TF, XNT, XNP)
%BENCHMARKS  Computed against published, with the provenance of every band.
%   Every book value was grepped from the DECODED zarchan.txt; the line number
%   is in .source.  Nothing here is a curve read off a figure.
B = emptyBench();

% ---- 1/2  the sweep itself.  Both rows describe the LISTING'S OWN sweep, so
%           an off-book tF or tstart SKIPS them: scoring "100 runs" against a
%           sweep the caller chose would be a red row that says nothing about
%           the port, and a red row that means nothing is worse than no row.
bookSweep = abs(TF - 10) < 1e-12 && abs(TS - 0.1) < 1e-12 && ...
            numel(tstart) == 100 && max(abs(tstart - (0.1:0.1:10))) < 1e-9;
if bookSweep
    B = addVal(B, 'forward-model runs', 100, numel(tstart), 'exact', 0, ...
        'text p.764, zarchan.txt:10760', 'runs', ...
        '"One hundred runs are made in the forward model"');
    okSweep = abs(max(tgos) - 9.9) < 1e-9 && abs(min(tgos)) < 1e-9 && ...
              abs(abs(tgos(2) - tgos(1)) - 0.1) < 1e-9;
    B = addShape(B, 'tgo sweep endpoints and step', ...
        '9.9 s down to 0.0 s, step 0.1 s', ...
        sprintf('%.4g -> %.4g s, step %.4g s', max(tgos), min(tgos), ...
                abs(tgos(2) - tgos(1))), okSweep, ...
        'text p.764, zarchan.txt:10760', ...
        '"varying between 9.9 and 0 s in steps of 0.1s"');
else
    B = addSkip(B, 'the listing''s own 100-run sweep', ...
        '100 runs, tgo 9.9 -> 0.0 s, step 0.1 s', ...
        sprintf(['this run used tF = %g s, TS = %g s and %d start times, ' ...
        'which is not the sweep the sentence describes'], TF, TS, ...
        numel(tstart)), 'text p.764, zarchan.txt:10760', 's');
end

% =========================================================================
%   THE ORDERING.  FOUR rows, because THE BOOK CONTRADICTS ITSELF about the
%   PN-vs-APN half and only about that half.  Ch.29 :10837 asserts a monotone
%   improvement; Ch.8 :2454 and :2461 assert the two are "comparable" and
%   "neither guidance law is superior", with a crossover in flight time.  Both
%   are scored, both are cited, and the rms reading -- which is the one that
%   fails -- is reported in full rather than banded.  See the header for the
%   measurements that separated the four possible causes.
%
%   ⚠️ GATED ON THE NOMINAL CASE.  Ch.29's sentence is about "the three cases"
%   of Figs. 29.9-29.11, which are the listing's own N' = 3 on the listing's
%   own 100-run sweep.  MEASURED 2026-08-24: at N' = 4 the peak ordering
%   reverses (PN 16.16, APN 20.24 ft) -- so an ungated row would report "the
%   book is wrong" about a run the book never made, and a red row that means
%   nothing is worse than no row.  Rows 1 and 2 therefore SKIP off-book, on
%   the same reasoning as the sweep rows above.  Row 3 is Ch.8's claim and
%   Ch.8's own case is N' = 4, so it carries no N' gate; measured, it holds at
%   both N' = 3 and N' = 4.
% =========================================================================
nominalCase = bookSweep && abs(XNP - 3) < 1e-12;
if isequal(sort(laws), [0 1 2])
    iPN = find(laws == 0, 1);  iAP = find(laws == 1, 1);  iOP = find(laws == 2, 1);
    rmsv  = [R.law(iPN).rms_ft,  R.law(iAP).rms_ft,  R.law(iOP).rms_ft];
    peakv = [R.law(iPN).peak_ft, R.law(iAP).peak_ft, R.law(iOP).peak_ft];
    pnm   = abs(R.law(iPN).miss_ft);
    apm   = abs(R.law(iAP).miss_ft);

    offBook = sprintf(['Figs. 29.9-29.11 are the listing''s nominal case, ' ...
        'N'''' = 3 on tstart = 0.1:0.1:10 s with tF = 10 s; this run used ' ...
        'N'''' = %g and %d start times, and the ordering is known to reverse ' ...
        'off-book (N'''' = 4 measures PN 16.16 / APN 20.24 ft)'], ...
        XNP, numel(tstart));

    % ---- 1  the half of :10837 that Ch.8 states as well, unconditionally --
    if ~nominalCase
    B = addSkip(B, 'optimal beats BOTH PN and APN, on peak AND rms', ...
        'optimal below both other laws under both statistics', offBook, ...
        'text p.774 zarchan.txt:10837; Ch.8 :2574 and :2579', 'ft');
    else
    B = addShape(B, 'optimal beats BOTH PN and APN, on peak AND rms', ...
        'optimal below both other laws under both statistics', ...
        sprintf('peak %.4g vs %.4g / %.4g ft;  rms %.4g vs %.4g / %.4g ft', ...
            peakv(3), peakv(1), peakv(2), rmsv(3), rmsv(1), rmsv(2)), ...
        peakv(3) < min(peakv(1:2)) && rmsv(3) < min(rmsv(1:2)), ...
        ['text p.774 zarchan.txt:10837; Ch.8 p.183 :2574 and Fig. 8.18 ' ...
         'caption :2579'], ...
        ['the uncontested part of the claim: :2574 says "the optimal ' ...
         'guidance law always yields zero miss distance" and the Fig. 8.18 ' ...
         'caption at :2579 says "Optimal guidance system does not have miss ' ...
         'distance".  ⚠️ WEAK DETECTOR, measured: it survives deleting the ' ...
         'APN term, flipping its sign and doubling its factor.  It is here ' ...
         'because both chapters state it, not because it guards anything.']);
    end

    % ---- 2  the literal Ch.29 sentence on the ENVELOPE, which is what Figs.
    %         29.9-29.11 actually plot ------------------------------------
    if ~nominalCase
    B = addSkip(B, 'peak |miss| improves PN > APN > optimal', ...
        'monotone improvement with law sophistication', offBook, ...
        'text p.774, zarchan.txt:10837', 'ft');
    else
    B = addShape(B, 'peak |miss| improves PN > APN > optimal', ...
        'monotone improvement with law sophistication', ...
        sprintf('%.4g / %.4g / %.4g ft', peakv(1), peakv(2), peakv(3)), ...
        peakv(1) > peakv(2) && peakv(2) > peakv(3), ...
        'text p.774, zarchan.txt:10837', ...
        ['"the miss distance results get better as the guidance law becomes ' ...
         'more advanced", read on the envelope of the plotted curve.  THE ' ...
         'DETECTOR ON THIS BOARD, measured by mutation: deleting the APN ' ...
         'term, flipping its sign and setting its factor to 1.0 instead of ' ...
         '0.5 each turn this row red (APN peak 24.30 / 32.95 / 39.18 ft ' ...
         'against PN 24.30).']);
    end

    % ---- 3  Ch.8 on THIS PAIR, and it is the opposite claim ---------------
    %   Threshold-free on purpose.  The book says APN wins at short times and
    %   PN at long ones, so the test is that BOTH regions exist and that the
    %   APN-wins region sits at systematically shorter tgo.  No split point is
    %   chosen, because any split point would be this port's and not the
    %   book's; the tgo <= 2 s figures in the row text are REPORTED, not used.
    dd   = apm - pnm;
    win  = dd < 0;              % APN closer
    lose = dd > 0;              % PN closer
    % mw/ml are formed WHATEVER the outcome so that a failing row still says
    % which region collapsed; a row that prints NaN NaN on failure hides the
    % one number a reader needs.
    if any(win),  mw = mean(tgos(win));   else, mw = NaN; end
    if any(lose), ml = mean(tgos(lose));  else, ml = NaN; end
    okX = any(win) && any(lose) && mw < ml;
    if ~bookSweep
    B = addSkip(B, 'PN and APN are NOT ordered; APN wins at short tgo', ...
        'neither law superior; APN better for shorter times, PN for longer', ...
        sprintf(['the claim is about where on the abscissa each law wins, so ' ...
        'it needs the listing''s own sweep across both regimes; this run had ' ...
        '%d start times over tF = %g s'], numel(tstart), TF), ...
        'zarchan.txt:2454 and :2461', 'ft');
    else
    B = addShape(B, 'PN and APN are NOT ordered; APN wins at short tgo', ...
        'neither law superior; APN better for shorter times, PN for longer', ...
        sprintf(['APN closer on %d runs (mean tgo %.2f s), PN on %d ' ...
                 '(mean tgo %.2f s); reported split at 2 s: max|miss| ' ...
                 '%.4g vs %.4g ft below, %.4g vs %.4g ft above'], ...
            sum(win), mw, sum(lose), ml, ...
            max([apm(tgos <= 2) -Inf]), max([pnm(tgos <= 2) -Inf]), ...
            max([apm(tgos >  2) -Inf]), max([pnm(tgos >  2) -Inf])), ...
        okX, ...
        ['Fig. 8.12 caption p.175 zarchan.txt:2454 and text p.176 :2461; ' ...
         'abscissa equivalence :10786 (also :1733)'], ...
        ['"neither guidance law is superior from a miss distance point of ' ...
         'view ... Augmented proportional navigation yields smaller miss ' ...
         'distances for shorter flight times, whereas proportional ' ...
         'navigation yields smaller miss distances for longer flight times" ' ...
         '(:2461), on the same disturbance (3-g step maneuver, single-lag ' ...
         'FCS) and the same abscissa (:10786, "adjoint time can either be ' ...
         'interpreted as missile flight time or time to go at which the ' ...
         'target maneuver started").  ⚠️ Ch.8''s run is N'' = 4, T = 1 s and ' ...
         'PERFECT states; this one is N'' = 3, T = 0.5 s with a Kalman ' ...
         'filter, so the row scores the SHAPE and not any number of Ch.8''s. ' ...
         'MEASURED: the crossover survives replacing the filter estimate ' ...
         'with the true nT, so it is not an artefact of the estimator.  ' ...
         'TEETH, by mutation: deleting the APN term or flipping its sign ' ...
         'turns this row red.']);
    end

    % ---- 4  the rms reading of :10837.  IT FAILS.  Reported, not scored. --
    B = addInfo(B, 'rms |miss| ordering PN > APN > optimal', ...
        '"the miss distance results get better as the law becomes advanced"', ...
        sprintf('FALSE for PN->APN:  %.4g / %.4g / %.4g ft', ...
            rmsv(1), rmsv(2), rmsv(3)), ...
        'text p.774 zarchan.txt:10837, against Ch.8 :2454 and :2461', ...
        ['⚠️⚠️ THE BOOK DISAGREES WITH ITSELF AND THIS IS THE ROW THAT SAYS ' ...
         'SO.  rms over a uniformly swept maneuver onset IS a Zarchan ' ...
         'quantity -- MUDNT, :1340-1343 -- so this is a fair reading of ' ...
         ':10837 and it is red on the merits.  Row 3 scores Ch.8''s explicit ' ...
         'denial of the same ordering and is green.  NOT a port fault, ' ...
         'measured three ways: (a) feeding the APN term the TRUE nT instead ' ...
         'of the filter''s estimate, one line changed, gives APN rms 5.190 ' ...
         'and peak 11.31 ft, below PN and monotone; (b) Ch.26''s independent ' ...
         'bakeoff, which hands its APN the true maneuver, measures APN peak ' ...
         'at 0.568-0.572x PN''s at all three of its plant cases; (c) H = ' ...
         '0.01 -> 5e-4 moves the three rms values to 8.639 / 9.684 / 0.896, ' ...
         'so it is converged.  The extra term is only worth its noise once ' ...
         'the estimate has converged, which on this sweep is tgo <~ 2.4 s.  ' ...
         'INFO and not a scored red row because the SAME sentence is already ' ...
         'scored twice above and the contradiction, not the arithmetic, is ' ...
         'the finding.']);
else
    B = addSkip(B, 'miss ordering PN > APN > optimal', ...
        'monotone improvement with law sophistication', ...
        sprintf(['the ordering is a three-way comparison and only %d law ' ...
        'option(s) were run; a two-way version would not be the book''s ' ...
        'claim'], numel(laws)), 'text p.774, zarchan.txt:10837', '-');
end

% =========================================================================
%   PERFECT-INFORMATION FLOORS.  BOOK EQUATIONS, and they are what replaced
%   the figure-frame containment rows (see the header for why those went).
%   Both are N' = 3 ONLY: off-book XNP SKIPS them rather than quoting a
%   formula that does not apply, which would put a fictitious number on the
%   board dressed as a published one.
% =========================================================================
iPN = find(laws == 0, 1);
if isempty(iPN)
    B = addSkip(B, 'PN miss above the perfect-information floors', ...
        'peak >= MNT|N''=3, rms >= MUDNT|N''=3', ...
        'APN = 0 was not among the law options run', ...
        'zarchan.txt:1345 and :1355-1357', 'ft');
elseif abs(XNP - 3) >= 1e-12
    B = addSkip(B, 'PN miss above the perfect-information floors', ...
        'peak >= MNT|N''=3, rms >= MUDNT|N''=3', ...
        sprintf(['both closed forms are derived for N'' = 3 and this run ' ...
        'uses N'' = %g'], XNP), 'zarchan.txt:1345 and :1355-1357', 'ft');
else
    % Ch.3 step-maneuver sensitivity, at its peak tgo = 2T.
    peakFloor = XNT * 0.5 * (2*TAU)^2 * exp(-2);
    % Ch.4 uniformly distributed onset: the rms of the SAME curve, in closed
    % form.  Verified against a quadrature of :1345 to 2.2e-16.
    xn   = TF/TAU;
    poly = 2*xn^4 + 4*xn^3 + 6*xn^2 + 6*xn + 3;
    rmsFloor = (XNT/4) * sqrt( TAU^5/TF * (3 - exp(-2*xn)*poly) );
    B = addBound(B, 'PN peak |miss| above the perfect-information floor', ...
        sprintf('peak |miss| >= %.4g ft', peakFloor), ...
        sprintf('%.4g ft at tgo = %.2f s  (%.2fx the floor)', ...
            R.law(iPN).peak_ft, R.law(iPN).peak_tgo_s, ...
            R.law(iPN).peak_ft/peakFloor), ...
        R.law(iPN).peak_ft >= peakFloor, ...
        'closed form MNT|N''=3 = 0.5 tF^2 exp(-tF/T), zarchan.txt:1345 (also :888-890)', ...
        'ft', ...
        ['the SAME control gains driven by PERFECT states with no sampler, ' ...
         'peaking at tgo = 2T.  Estimates cannot beat perfect information, ' ...
         'so this is a floor.  An information argument, not a theorem -- ' ...
         'hence a bound and not a value.  ⚠️ A LOOSE DETECTOR AT THE BOOK''S ' ...
         'DEFAULTS (3.72x) -- it catches a gross scale error and nothing ' ...
         'finer.  ⭐ BUT IT IS THE RIGHT FLOOR, MEASURED 2026-08-24 by ' ...
         'walking this port toward perfect information: (TS, SIGNOISE) = ' ...
         '(0.1, 1e-3) -> 24.29 ft (3.72x), (0.05, 1e-4) -> 11.85 (1.81x), ' ...
         '(0.02, 1e-6) -> 8.33 (1.27x), (0.01, 1e-9) -> 7.54 (1.15x), ' ...
         '(0.005, 1e-9) -> 7.13 ft (1.09x).  Monotone down to the closed ' ...
         'form and never through it, which is what a floor has to do.']);
    B = addBound(B, 'PN rms |miss| above the book''s MUDNT floor', ...
        sprintf('rms |miss| >= %.4g ft', rmsFloor), ...
        sprintf('%.4g ft  (%.2fx the floor)', ...
            R.law(iPN).rms_ft, R.law(iPN).rms_ft/rmsFloor), ...
        R.law(iPN).rms_ft >= rmsFloor, ...
        ['MUDNT = nT sqrt((1/tF) int_0^tF MNT^2 dt), zarchan.txt:1340-1343; ' ...
         'closed form :1355-1357'], 'ft', ...
        ['"the miss due to a target maneuver with uniformly distributed ' ...
         'starting time" (:1339) -- the rms of the perfect-information curve ' ...
         'over exactly this sweep, so the perfect-information counterpart of ' ...
         'the rms this port reports.  ⭐ COHERENCE: the measured peak and rms ' ...
         'stand at the SAME multiple of their respective floors (3.72x and ' ...
         '3.69x at the book''s defaults), which is the sampler-plus-filter ' ...
         'cost and would not hold if either statistic were computed wrongly.  ' ...
         'That equality SURVIVES THE WALK toward perfect information ' ...
         '(2026-08-24): at (TS, SIGNOISE) = (0.005, 1e-9) the peak is 1.091x ' ...
         'its floor and the rms 1.095x its own -- two statistics, two ' ...
         'independent closed forms, one ratio, all the way down.']);
end

% ---- the sign and shape of C4 for optimal guidance -----------------------
[~, ~, ~, C4x1, NPx1] = controlGains(2, TAU, TAU, XNP);      % x = tgo/TAU = 1
B = addVal(B, 'optimal C4/N'' at x = 1', -exp(-1), C4x1/NPx1, ...
    '1e-9 (exact algebra)', 1e-9, ...
    'text p.179 zarchan.txt:2520 and :2540; cf. Ch.29 :10753', '-', ...
    ['C4 = -N''(e^-x + x - 1)/x^2, so C4/N'' at x = 1 is -exp(-1) exactly. ' ...
     'Both printings of C4 alone (:2540, :10753) lost the minus; the full ' ...
     'law at :2520 keeps it, and the listing agrees with :2520.']);

% ---- the effective navigation ratio's limit ------------------------------
NPbig = nprimeOf(1e4, TAU, XNP);
B = addVal(B, 'optimal N'' as tgo/T -> infinity', 3, NPbig, '0.1%', 1e-3, ...
    'text p.178, zarchan.txt:2526 (also :6373)', '-', ...
    ['"...approximately constant and is approaching 3".  At x = 1e4 the ' ...
     '.0001 guard is negligible and N'' = 3 + 6/x gives 3.0006.  NOT scored ' ...
     'at the sweep''s x = 19.8, where N'' = 3.33 needs an 11% band.']);

% ---- info / skip --------------------------------------------------------
B = addInfo(B, 'published y-axis ticks of Figs. 29.9-29.11', ...
    'Fig 29.9: 20 15 10 5 0 | Fig 29.10: -20 -10 0 10 | Fig 29.11: 3 2 1 0 -1', ...
    sprintf('PN [%.3g, %.3g] | APN [%.3g, %.3g] | optimal [%.3g, %.3g] ft', ...
        rangeOf(R, laws, 0), rangeOf(R, laws, 1), rangeOf(R, laws, 2)), ...
    ['zarchan.txt:10821-10825, :10838-10841, :10850-10854; Listing 29.3''s own ' ...
     'plot call, which sets NO axis limits, is :10784'], ...
    ['⚠️ REPORTED, NEVER SCORED, and it used to be three scored containment ' ...
     'rows of which two were red.  DELETED, not widened: the outermost ' ...
     'extracted tick is a labelled gridline, not an axis limit, and Ch.29 ' ...
     'proves it -- across the five figures in this chapter that have both a ' ...
     'tick block and a port (29.5 and 29.6 via mgs.zarchan.dualRateAdjoint, ' ...
     'measuring [-5.97, 16.55] against ticks -5..15 and [-36.11, 101.59] ' ...
     'against -20..100; 29.9/29.10/29.11 here), NINE of the ten tick ' ...
     'extremes are exceeded by the curve, always outward.  A containment ' ...
     'test that no case in the chapter passes is not a benchmark.  The label ' ...
     'in all five blocks reads "Altitude (km)" where each listing writes ' ...
     'ylabel(''Miss (Ft)'') -- 22 figure blocks in this file carry it, so it ' ...
     'is an extraction artefact -- and Fig. 29.10''s block is the only one ' ...
     'printed bottom-to-top and stops at 10 while its own curve reaches ' ...
     '-19.88, i.e. it demonstrably lost an entry.  ' ...
     '⭐ AND THE MECHANICAL REASON, WHICH SEPARATES THIS CHAPTER''S TWO KINDS ' ...
     'OF "FRAME": Listing 29.3''s plot call (:10784) and Listing 29.4''s ' ...
     '(:10819) contain NO axis() call at all, so these three figures are ' ...
     'AUTO-SCALED and their limits were never typed by anybody -- whereas ' ...
     'Listing 7.2 (:2051) and Listing 29.1 (:10659) DO carry hand-typed ' ...
     'axis([...]) calls that hard-clip.  An author''s axis() argument is a ' ...
     'decision about his own curve and can be scored (mgs.zarchan.fadingMemoryMC ' ...
     'and mgs.zarchan.dualRateForward each keep one, on measured ' ...
     'discrimination); an auto-scaled tick label is not, and this is the row ' ...
     'that says so.  mgs.zarchan.kalmanLawAdjoint reached the same verdict ' ...
     'independently.']);

% NO SKIP ROW FOR THE CURVES THEMSELVES, deliberately.  The figures became
% image links in the extraction and no curve value survived, so no point on
% them is scoreable -- but mgs.zarchan.kalmanLawAdjoint already carries a SKIP
% row saying exactly that about exactly these three figures, and benchSuite
% renders a SKIP as an unresolved row.  Two unresolved rows for one fact makes
% the board look like two problems.  The fact stays visible once, there; the
% INFO row above is where THIS port records it.

B = addInfo(B, 'agreement with the Listing 29.4 adjoint', ...
    '"exact agreement between the adjoint and forward simulations"', ...
    'scored in mgs.zarchan.kalmanLawAdjoint, not re-run here', ...
    'text p.773, zarchan.txt:10820', ...
    ['Listing 29.4 IS ported -- mgs.zarchan.kalmanLawAdjoint -- and it runs ' ...
     'its own Listing 29.3 twin and scores the identity per guidance law ' ...
     '(measured 2026-08-24: residuals 0.075 / 0.131 / 0.004 ft against ' ...
     'peaks 21.89 / 19.33 / 1.89 ft).  That twin also agrees with THIS file ' ...
     'to every printed digit at all nine of its integer tgo, which is the ' ...
     'independent check that the miss values above are the listing''s and ' ...
     'not this port''s.  ⚠️ Its peaks are lower than the ones here only ' ...
     'because its forward grid is tgo = 1:9 s and misses this sweep''s peaks ' ...
     'at tgo = 1.3 s (PN) and 0.6 s (optimal).  Still INFO: this file does ' ...
     'not re-run it, so the absence of the chapter''s headline claim FROM ' ...
     'THIS BOARD stays visible rather than being quietly implied.']);
end


function s = rangeOf(R, laws, apn)
%RANGEOF  [min max] of one law's signed miss, or [NaN NaN] if it was not run.
i = find(laws == apn, 1);
if isempty(i)
    s = [NaN NaN];
else
    s = [min(R.law(i).miss_ft), max(R.law(i).miss_ft)];
end
end


% =============================================================================
%   ROW CONSTRUCTION.  Every row is built from blankRow(), so the field ORDER
%   cannot drift between adders -- benchSuite's adaptGeneric is strict and a
%   near-miss schema errors rather than being accommodated.
% =============================================================================
function r = blankRow()
r = struct('what','', 'kind','info', 'book',NaN, 'book_str','--', ...
           'ours',NaN, 'ours_str','--', 'pct',NaN, 'band','info', 'tol',NaN, ...
           'ok',false, 'skipped',true, 'why','', 'source','', 'units','-', ...
           'detail','');
end

function B = emptyBench()
f = fieldnames(blankRow()).';
c = [f; repmat({{}}, 1, numel(f))];
B = struct(c{:});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
r = blankRow();
r.what = what;  r.kind = 'value';
r.book = book;  r.book_str = sprintf('%.6g', book);
r.ours = ours;  r.ours_str = sprintf('%.6g', ours);
r.band = band;  r.tol = tol;  r.units = units;
r.source = source;  r.detail = detail;
if ~isfinite(ours) || book == 0
    r.pct = NaN;  r.ok = false;  r.skipped = true;
    r.why = 'the computed value is not finite (or the book value is zero)';
else
    r.pct = 100*(ours/book - 1);
    r.ok  = abs(ours/book - 1) <= tol;
    r.skipped = false;
end
B(end+1) = r;
end

function B = addShape(B, what, bookStr, oursStr, ok, source, detail)
r = blankRow();
r.what = what;  r.kind = 'shape';  r.band = 'shape';
r.book_str = bookStr;  r.ours_str = oursStr;
r.ok = logical(ok);  r.skipped = false;
r.source = source;  r.detail = detail;
B(end+1) = r;
end

function B = addBound(B, what, bookStr, oursStr, ok, source, units, detail)
r = blankRow();
r.what = what;  r.kind = 'bound';  r.band = 'bound';
r.book_str = bookStr;  r.ours_str = oursStr;
r.ok = logical(ok);  r.skipped = false;
r.source = source;  r.units = units;  r.detail = detail;
B(end+1) = r;
end

function B = addInfo(B, what, bookStr, oursStr, source, why)
% Reported, never scored: skipped = true keeps it out of the pass count in
% BOTH directions -- an INFO row can neither pass nor fail.
r = blankRow();
r.what = what;  r.kind = 'info';  r.band = 'info';
r.book_str = bookStr;  r.ours_str = oursStr;
r.source = source;  r.why = why;
B(end+1) = r;
end

function B = addSkip(B, what, bookStr, why, source, units)
% A check that COULD NOT RUN.  Never a pass: a check that did not happen is a
% different thing from one that succeeded.
r = blankRow();
r.what = what;  r.kind = 'skip';  r.band = 'skip';
r.book_str = bookStr;  r.why = why;
r.source = source;  r.units = units;
B(end+1) = r;
end


% =============================================================================
function s = render(R, ev)
L = {};
p = R.p;
L{end+1} = 'mgs.zarchan.kalmanLawOptions -- Zarchan Listing 29.3';
L{end+1} = sprintf(['  tF = %g s, TS = %g s, TAU = %g s, H = %g s, ' ...
    'N'''' = %g, nT = %g ft/s^2'], p.TF, p.TS, p.TAU, p.H, p.XNP, p.XNT);
L{end+1} = sprintf('  %d runs, %d filter updates.  units: %s', ...
    p.n_runs, p.n_updates, p.units);
L{end+1} = '  law         rms miss     peak |miss|      at tgo';
for i = 1:numel(R.law)
    L{end+1} = sprintf('  %-8s %9.3f ft %11.3f ft %8.2f s', R.law(i).name, ...
        R.law(i).rms_ft, R.law(i).peak_ft, R.law(i).peak_tgo_s); %#ok<AGROW>
end
L{end+1} = '  structural checks:';
for i = 1:numel(R.selfcheck.names)
    if R.selfcheck.oks(i)
        m = 'ok  ';
    else
        m = 'FAIL';
    end
    L{end+1} = sprintf('   [%s] %s -- %s', m, R.selfcheck.names{i}, ...
        R.selfcheck.detail{i}); %#ok<AGROW>
end
L{end+1} = '  benchmarks:';
for i = 1:numel(R.bench)
    b = R.bench(i);
    if b.skipped
        m = upper(b.kind);
    elseif b.ok
        m = 'ok  ';
    else
        m = 'FAIL';
    end
    if isnan(b.pct)
        pc = '';
    else
        pc = sprintf('  (%+.2f%%)', b.pct);
    end
    L{end+1} = sprintf('   [%s] %-42s book %-24s ours %s%s', m, b.what, ...
        b.book_str, b.ours_str, pc); %#ok<AGROW>
    L{end+1} = sprintf('          %s', b.source); %#ok<AGROW>
end
if any(ev)
    L{end+1} = sprintf('  scored %d row(s); pass = %d', sum(ev), R.pass);
else
    L{end+1} = '  NOTHING WAS SCORED -- pass = 0 means NOT CHECKED.';
end
s = strjoin(L, newline);
end


% =============================================================================
function drawCurves(R)
figure;
hold on;
for i = 1:numel(R.law)
    plot(R.tgos, R.law(i).miss_ft, 'DisplayName', R.law(i).name);
end
grid on;
legend show;
title('Miss for various tgo maneuver starting times (Listing 29.3)');
xlabel('Time to go at which maneuver occurs (s)');
ylabel('Miss (ft)');
end
