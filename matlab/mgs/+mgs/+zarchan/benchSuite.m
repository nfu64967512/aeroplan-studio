function [rows, nfail, S] = benchSuite(opts)
%MGS.ZARCHAN.BENCHSUITE  One board for every audited Zarchan port.
%
%   mgs.zarchan.benchSuite()                    run everything, print the board
%   [rows, nfail] = mgs.zarchan.benchSuite('render', false)
%   [rows, nfail, S] = mgs.zarchan.benchSuite('only', {'engage2d'})
%
%   rows   struct array, ONE ELEMENT PER BENCHMARK.  Carries .name/.ok/
%          .skipped/.detail so mgs.selftest can splice it straight into its
%          own R{} list, plus .port/.book/.computed/.delta/.source/.band.
%   nfail  number of scored checks that FAILED.  Zero is the only good value.
%   S      aggregate: counts, per-port timing/stdout, exclusions, .summary.
%
%   Wiring for mgs.selftest, two lines:
%       [rw, nf] = mgs.zarchan.benchSuite('render', false);
%       R{end+1} = chk('zarchan book benchmarks', nf == 0, ...
%                      sprintf('%d rows, %d failed', numel(rw), nf));
%   or splice rw directly -- its rows already have name/ok/skipped/detail.
%
%   IT DOES NOT THROW ON A MISMATCH.  Every port runs even if an earlier one
%   blew up, because the question this file answers is "what is the state of
%   the board", not "what is the first thing that went wrong".  A port that
%   errors becomes one FAIL row carrying its message.  Pass ('strict',true)
%   to get an error AFTER the whole board has printed -- never before it.
%
% ---------------------------------------------------------------------------
%   UNITS: FEET.  ft, ft/s, ft/s^2, g = 32.2, degrees via the book's 57.3.
%   Every number on this board is in Zarchan's units and NOTHING is converted
%   to SI, because the entire point is to compare against published values.
%   If you carry a number out of here into metric mgs code, convert at the
%   call site and say so there.
% ---------------------------------------------------------------------------
%
%   WHAT THIS FILE IS, AND WHAT IT DELIBERATELY IS NOT
%
%   It is an ADAPTER.  Each port already computes its own book comparison and
%   already owns its tolerances, and this file re-renders those verdicts side
%   by side.  It does NOT keep a second copy of the book's numbers, because a
%   second copy is a thing that drifts out of agreement with the first while
%   both keep printing green.
%
%   TWO documented exceptions, both marked in the source column of the rows
%   they affect:
%     * mgs.zarchan.weaveDesignCurve computes its benchmark points but does
%       NOT adjudicate them -- its bench struct has no ok field.  The +/-5%
%       band for those rows therefore lives HERE.  If adjudication is ever
%       added to that port, DELETE it from here rather than keeping two bands
%       that can disagree.
%     * five extra weave rows (curve maximum, its location, the PN-beats-
%       coasting crossover, and the two coast displacements) are scored here
%       against values quoted in the Ch.20 text.  They include the sharpest
%       check in that port and it publishes no verdict for any of them.
%
%   The figure/page citation per row lives here too -- most ports do not store
%   one.  The mapping is keyed on the port's own row NAME.  Rename a check in
%   a port and its citation goes to "** UNMAPPED **" in the source column and
%   is counted at the bottom.  That is deliberate: a board that keeps printing
%   a stale figure number beside a renamed check is worse than one that admits
%   it lost the reference.
%
%   TWO CITATION MECHANISMS, AND WHICH PORT USES WHICH.  The five original
%   ports (Listings 2.1, 2.2, 3.1, 6.1, 20.3) do NOT store a citation, so
%   cite() below holds theirs, keyed on row name.  The TEN ports added since --
%   second wave Listings 5.2, 5.3, 8.1, 8.2, 23.3, third wave Listings 21.1,
%   21.2, 22.4, 23.1, 28.1 -- each carry a .source field on every row that
%   already names the figure/page AND the zarchan.txt line, so their adapters
%   read THAT through selfCite(), all ten of them via the shared benchRows(),
%   and cite() has no table for them -- deliberately.  Adding one would be a
%   second copy of a citation that is already correct at the source, i.e. the
%   drift this file exists to avoid, and cite() therefore ERRORS with that instruction if it is ever
%   called with one of their keys.  selfCite() still routes an EMPTY .source to
%   "** UNMAPPED **", so the unmapped counter covers both mechanisms.
%
% ---------------------------------------------------------------------------
%   THE TRAP THE COMMISSIONING NOTES WALK STRAIGHT INTO, WIRED SHUT BELOW
%
%   Two of these ports do NOT run their benchmarks on a bare call, despite the
%   commissioning notes saying they do:
%
%       mgs.zarchan.engage2d()          -> ONE nominal engagement, no checks
%       mgs.zarchan.weaveDesignCurve()  -> the 40-point sweep, W.bench = []
%
%   The other thirteen DO run their benchmarks bare.  The manifest passes the
%   flag to all fifteen anyway, because a manifest that depends on a default
%   is a manifest that a later edit can silently empty.
%
%   Both files say so in their own headers ("the flag is the ONLY switch"),
%   because `nargin == 0` is ALWAYS TRUE in a function whose arguments block
%   declares only name-value options, so neither could use nargin to detect a
%   bare call.  A suite that called them bare would run for half a minute,
%   print a full-looking board, and CHECK NOTHING.  The calls below pass
%   'Bench',true and 'benchmark',true, and the adapters ERROR if the returned
%   struct lacks the benchmark field.  A port that adjudicates nothing is
%   recorded as a FAILURE here, never as zero failures.
%
%   Related: engage2d's benchmark path prints unconditionally (that branch has
%   no render flag), so every port runs through evalc and its output is kept
%   in S.ports(i).stdout.  Pass ('echo',true) to see it.
%
% ---------------------------------------------------------------------------
%   WHY THIS BOARD MATTERS TO THIS PROJECT
%
%   mgs scores 32/32 hits at 0.03 m where the flight set scores 25/32 at
%   0.93 m.  The measured cause is a guidance-system LAG the model does not
%   have: real gamma-tracking gain 0.063 against the model's 1.385, ~8 s of
%   lag, saturating on 26 of 32 sorties.  Zarchan Ch.3 and Ch.6 are about
%   exactly that -- what a guidance time constant T does to miss distance --
%   and the book publishes the answers, so these ports are an INDEPENDENT
%   CHECK on our integrator and PN kernel before we trust the lag machinery
%   they bring with them.
%
%   None of these listings is a model of our vehicle.  There is no gravity,
%   no drag, no thrust, no asymmetric nz clamp, no LOS floor, and above all
%   no SATURATING lag.  A miss distance off this board is not a sortie
%   prediction, and the printed footer says so on every run precisely so that
%   it cannot be quoted without that sentence attached.
%
% ---------------------------------------------------------------------------
%   RUNTIME.  THE BOARD MEASURES ITSELF: every run prints a per-port elapsed
%   time and a WALL CLOCK line naming the three slowest ports, and offers a
%   subset by name once the total passes 20 s.  Read that line rather than
%   this paragraph -- it is measured, and this paragraph is not.
%
%   What the third wave adds, by step count rather than by stopwatch (MATLAB
%   was deliberately not run while these adapters were written, so these are
%   estimates and are labelled as such):
%       airframeSim         2 flight conditions x 2 s at h = 1e-4        fast
%       airframeLinear      2 sims + 2 sweeps + Table 22 + 2 twins       fast
%       flightControlDelay  ~30 bisection runs of 20,000 steps       ~4 s
%       threeLoopAutopilot  3 step responses + 4 variants + a full run
%                           of adjointWithAutopilot for its cross-check ~5 s
%       engage3d            2 laws x 80 engagements x ~2200 steps, plus
%                           two more sweeps in the structural block   ~5-15 s
%   so budget roughly 15-30 s of new work on top of whatever the first ten
%   cost.  adjointWithAutopilot (Listing 23.3) remains the single biggest
%   entry: ~75 adjoint integrations of 50,000 steps plus 5 step responses.
%   Its own header notes that 'h',1e-3 cuts that tenfold and moves every
%   Table 23.5 entry by under 1 part in 1e4, and that h is deliberately not
%   gated as an off-book parameter -- but the entry below deliberately does
%   NOT pass it, so the board reproduces the audited numbers digit for digit
%   rather than numbers from a configuration nobody audited.  Same rule for
%   the third wave: threeLoopAutopilot's 'crosscheck' and
%   flightControlDelay's 'margins' would both buy seconds and both remove the
%   row their listing exists to produce, so neither is passed.  If you need a
%   fast board, run ('only', {...}) and SAY WHICH PORTS YOU SKIPPED.
%
%   See also MGS.SELFTEST, MGS.ZARCHAN.ENGAGE2D, MGS.ZARCHAN.ENGAGELINEAR,
%   MGS.ZARCHAN.ADJOINTSINGLELAG, MGS.ZARCHAN.ADJOINTBINOMIAL5,
%   MGS.ZARCHAN.WEAVEDESIGNCURVE, MGS.ZARCHAN.COVARIANCEHOMING,
%   MGS.ZARCHAN.ACCELMISSADJOINT, MGS.ZARCHAN.ADJOINTSINGLELAGCH8,
%   MGS.ZARCHAN.ADJOINTOPTIMAL, MGS.ZARCHAN.ADJOINTWITHAUTOPILOT,
%   MGS.ZARCHAN.AIRFRAMESIM, MGS.ZARCHAN.AIRFRAMELINEAR,
%   MGS.ZARCHAN.FLIGHTCONTROLDELAY, MGS.ZARCHAN.THREELOOPAUTOPILOT,
%   MGS.ZARCHAN.ENGAGE3D.

arguments
    opts.render (1,1) logical = true    % print the board
    opts.echo   (1,1) logical = false   % also print each port's own output
    opts.strict (1,1) logical = false   % error AFTER printing if nfail > 0
    % No size constraint on 'only'.  "opts.only (1,:) cell = {}" is REJECTED
    % by MATLAB's own validator -- {} is 0x0 and (1,:) demands one row -- the
    % same trap engageLinear.m documents for YD0.  The contents are checked
    % explicitly below, where the message can say what went wrong.
    opts.only   cell = {}               % subset of port keys, {} = all
end

% ---------------------------------------------------------------------------
%   THE MANIFEST.  Ports that PASSED audit, and ports that did not.
%
%   AUDITED_TOTAL is the number of ports that have been through audit at all.
%   The guard below refuses to run if PORTS + EXCLUDED does not account for
%   every one of them.  Without it, deleting a port from P would silently
%   shrink the board and nothing would say a benchmark had gone missing --
%   the same shape of failure as a guard that is documented and absent, which
%   this repo hit three times in one day on 2026-08-22.
% ---------------------------------------------------------------------------
%   2026-08-23: five more ports came out of adversarial audit, all PASSED,
%   none FAILED or unverified.  AUDITED_TOTAL 5 -> 10.  The X table below is
%   still empty and the renderer says so by name rather than by silence.
%   2026-08-23 (later): the third wave -- Listings 21.1, 21.2, 22.4, 23.1 and
%   28.1 -- also came back all PASSED, none FAILED, none unverified.
%   AUDITED_TOTAL 10 -> 15, X still empty.
% ⚠️ wave 4 = 19, but shaping2d''s AUDIT AGENT DIED (spend limit) before it ran.
%    It is on the board because it executes and self-cites, but it has NOT been
%    through adversarial audit -- the one check that has caught fabricated book
%    values on every previous wave.  Counted here as unaudited, deliberately.
%   2026-08-24: wave 5 wrote ten LEAN ports and audited them in pairs.  The
%   first registration pass wired FOUR of them -- weaveSingleLag, weaveAdjoint,
%   rk4SecondOrder, radomeEngagement -- and bumped AUDITED_TOTAL 18 -> 22.
% ⚠️ That pass ALSO left a note here saying the other six (weaveCompensated,
%   fcsOpenLoop, fcsRateGyro, fcsBode, threeLoopOpenLoop, shapingLinear) "did
%   NOT clear audit".  THAT NOTE WAS WRONG and is deleted.  All ten cleared
%   audit; the join between the audit results and the registration step was
%   broken, so six passing ports were recorded as unverified.  The six are
%   wired below in the same wave-5 block.  AUDITED_TOTAL 22 -> 28.
%   The distinction the wrong note blurred is worth keeping: "failed audit"
%   belongs in X with a characterised reason, "never audited" belongs in
%   UNAUDITED, and "audited, passed, not yet wired" is not a category at all --
%   it is a bookkeeping error, which is what this was.
%   2026-08-24 (later): wave 6 wrote TEN more lean ports and audited them in
%   pairs -- Ch.26 Listings 26.1-26.6, Ch.19 Listings 19.1-19.3, and Ch.31
%   Listing 31.1.  All ten PASSED, none FAILED, none unverified; X stays empty.
%   AUDITED_TOTAL 28 -> 38 for those ten, then 38 -> 39 because shaping2d's
%   audit FINALLY RAN (verdict: honest, four defects, none moving a scored
%   verdict) so it moves out of UNAUDITED and into the audited count.  That
%   empties UNAUDITED for the first time since wave 4.  The guard therefore
%   reads 39 + 0 == 39 + 0.  Read the UNAUDITED block below before trusting
%   shaping2d's biased-PN magnitudes.  Its D4 defect -- no structural row of
%   its reached this board at all -- was CLOSED on 2026-08-24; the UNAUDITED
%   block below records what it was and what was fixed on both sides.
%   2026-08-24 (later still): wave 7 wrote TEN more lean ports and audited them
%   in pairs -- Ch.7 Listings 7.1-7.4, Ch.29 Listings 29.1-29.4, and Ch.19
%   Listings 19.4-19.5.  All ten PASSED, none FAILED, none unverified, so both
%   X and UNAUDITED stay empty.  AUDITED_TOTAL 39 -> 49 and the guard reads
%   49 + 0 == 49 + 0.
% ⚠️ RESOLVED since this note was written.  The six wave-6 FAILs it refers to
%   (optimalGainsSingleLag 4, optimalGainsCubic 1, guidanceLawBakeoff 1) are all
%   GREEN now; see the corrected account further down.  The red rows on the
%   board are wave-7's: kalmanLawOptions (3), fadingMemoryMC (1),
%   dualRateForward (1).
%   2026-08-24 (wave 8): TEN more lean ports, audited in pairs.  NINE PASSED --
%   Ch.1 Listings 1.1/1.2, Ch.4 Listings 4.1-4.4, Ch.9 Listing 9.1, Ch.10
%   Listing 10.1 and Ch.18 Listing 18.1 -- and are wired in the wave-8 block
%   below.  AUDITED_TOTAL 49 -> 58 (+9, one per port ADDED TO P), and the guard
%   reads 58 + 0 == 58 + 0.
% ⚠️⚠️ THE TENTH, radomeHomingLoop (Listing 18.2), FAILED ITS AUDIT and was NOT
%   wired, NOT in X, and NOT counted in AUDITED_TOTAL.  That was a deliberate
%   third position -- X means "excluded, with a characterised reason, and we are
%   done with it", and that port was fix-or-cut with the decision still open.
%   ✅ RESOLVED ON WAVE 9.  It was FIXED, RE-AUDITED, and the repair PASSED, so
%   it took the first of the two exits that note named: it is in P, in the
%   wave-9 block below, and AUDITED_TOTAL moves for it.  The wave-8 block still
%   carries the original finding, because what it failed on is the reason the
%   repaired predicate looks the way it does; the wave-9 block carries the fix.
%   2026-08-24 (wave 9): TEN more lean ports, audited in pairs, ALL TEN PASSED --
%   Ch.25 Listings 25.1-25.3, Ch.27 Listing 27.1, Ch.9 Listings 9.2/9.3, Ch.30
%   Listings 30.2/30.3, Ch.16 Listing 16.1 and Ch.17 Listing 17.1.  None FAILED
%   and none is unverified, so X and UNAUDITED both stay empty.  ELEVEN entries
%   go into P -- those ten plus the radomeHomingLoop repair -- so AUDITED_TOTAL
%   58 -> 69 (+10 newly audited, +1 for the port that had already been audited
%   but was uncounted while its disposition was open) and the guard reads
%   69 + 0 == 69 + 0.
%   2026-08-24 (wave 10): TEN more lean ports, audited in pairs, ALL TEN PASSED
%   -- Ch.11 Listings 11.1-11.4, Ch.12 Listings 12.1/12.2 and Ch.13 Listings
%   13.1-13.4.  None FAILED and none is unverified, so X and UNAUDITED both
%   stay empty.  TEN entries go into P, so AUDITED_TOTAL 69 -> 79 (+10, one per
%   port ADDED TO P) and the guard reads 79 + 0 == 79 + 0.  Unlike wave 9 there
%   is no carried-over port to account for: the +10 here is exactly the ten
%   files wired in the wave-10 block below, and nothing else moved.
%   2026-08-24 (wave 11): TEN more lean ports, audited in pairs, ALL TEN
%   PASSED -- Ch.14 Listings 14.1/14.2, Ch.15 Listings 15.1/15.2, Ch.16 Listing
%   16.2, Ch.28 Listings 28.2-28.4 and Ch.32 Listings 32.1/32.2.  None FAILED
%   and none is unverified, so X and UNAUDITED both stay empty.  TEN entries go
%   into P, so AUDITED_TOTAL 79 -> 89 (+10, one per port ADDED TO P) and the
%   guard reads 89 + 0 == 89 + 0.  As in wave 10 there is no carried-over port
%   to account for: the +10 is exactly the ten files wired in the wave-11 block
%   below, and nothing else moved.
%   2026-08-24 (wave 12): NINE more lean ports, audited in pairs, ALL NINE
%   PASSED -- Ch.32 Listings 32.3/32.4/32.5, Ch.33 Listings 33.1/33.2 and
%   Ch.34 Listings 34.1-34.4.  None FAILED and none is unverified, so X and
%   UNAUDITED both stay empty.  NINE entries go into P, so AUDITED_TOTAL
%   89 -> 98 (+9, one per port ADDED TO P) and the guard reads 98 + 0 == 98 + 0.
%   As in waves 10 and 11 there is no carried-over port to account for: the +9
%   is exactly the nine files wired in the wave-12 block below, and nothing
%   else moved.
%   ⭐⭐⭐ THIS WAVE FINISHES THE BOOK.  Counted in this file, not assumed:
%   P now holds 98 entries, and their 'listing' labels name 98 DISTINCT
%   numbered listings, which is exactly the set of listing_NN_MM.m files in
%   matlab/zarchan/_listings (98 files).  Before this wave the set difference
%   was precisely {32.3, 32.4, 32.5, 33.1, 33.2, 34.1, 34.2, 34.3, 34.4};
%   after it, empty both ways.  (Appendix A.1 is not a numbered listing and
%   rides along inside the label 'Listing 8.2 (+A.1)'.)
AUDITED_TOTAL = 98;

P = struct('key',{},'listing',{},'entry',{},'call',{},'adapt',{},'chapter',{});
P(end+1) = struct('key','engage2d', 'listing','Listing 2.1', ...
    'entry','engage2d(''Bench'',true)', ...
    'call', @() mgs.zarchan.engage2d('Bench',true), ...
    'adapt', @adaptEngage2d, 'chapter','Ch.2  nonlinear 2-D engagement');
P(end+1) = struct('key','engageLinear', 'listing','Listing 2.2', ...
    'entry','engageLinear(''bench'',true,''strict'',false)', ...
    'call', @() mgs.zarchan.engageLinear('bench',true,'strict',false, ...
                                         'render',false,'nonlinear',true), ...
    'adapt', @adaptEngageLinear, 'chapter','Ch.2  linearised engagement');
P(end+1) = struct('key','adjointSingleLag', 'listing','Listing 3.1', ...
    'entry','adjointSingleLag(''render'',false)', ...
    'call', @() mgs.zarchan.adjointSingleLag('render',false), ...
    'adapt', @adaptAdjointSingleLag, 'chapter','Ch.3  single-lag miss adjoint');
P(end+1) = struct('key','adjointBinomial5', 'listing','Listing 6.1', ...
    'entry','adjointBinomial5(''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.adjointBinomial5('assert',false,'render',false, ...
                                             'plot',false), ...
    'adapt', @adaptAdjointBinomial5, 'chapter','Ch.6  5th-order binomial adjoint');
P(end+1) = struct('key','weaveDesignCurve', 'listing','Listing 20.3', ...
    'entry','weaveDesignCurve(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.weaveDesignCurve('benchmark',true,'render',false, ...
                                             'plot',false), ...
    'adapt', @adaptWeave, 'chapter','Ch.20 weaving-target design curve');

% -- second wave.  Every one of these carries its own .source per row, so
%    their adapters call selfCite() and NOT cite(); see the header.
%
%    Each entry passes the port's benchmark switch EXPLICITLY.  None of these
%    files has an `if nargin == 0` (they all say why in their own headers: in
%    a function whose arguments block is name-value only, MATLAB reports
%    nargin = 0 however many were passed), so a bare call is not a checked
%    call for the ones whose benchmark flag defaults false, and the adapters
%    below ERROR if the benchmark field comes back missing or empty.
P(end+1) = struct('key','covarianceHoming', 'listing','Listing 5.2', ...
    'entry','covarianceHoming(''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.covarianceHoming('assert',false,'render',false, ...
                                             'plot',false), ...
    'adapt', @adaptCovarianceHoming, 'chapter','Ch.5  homing-loop covariance analysis');
P(end+1) = struct('key','accelMissAdjoint', 'listing','Listing 5.3', ...
    'entry','accelMissAdjoint(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.accelMissAdjoint('benchmark',true,'assert',false, ...
                                             'render',false,'plot',false), ...
    'adapt', @adaptAccelMissAdjoint, 'chapter','Ch.5  adjoint miss + acceleration');
P(end+1) = struct('key','adjointSingleLagCh8', 'listing','Listing 8.1', ...
    'entry','adjointSingleLagCh8(''render'',false,''assert'',false)', ...
    'call', @() mgs.zarchan.adjointSingleLagCh8('render',false,'assert',false), ...
    'adapt', @adaptSingleLagCh8, 'chapter','Ch.8  PN vs augmented PN adjoint');
P(end+1) = struct('key','adjointOptimal', 'listing','Listing 8.2 (+A.1)', ...
    'entry','adjointOptimal(''assert'',false,''render'',false,''plot'',false)', ...
    'call', @() mgs.zarchan.adjointOptimal('assert',false,'render',false, ...
                                           'plot',false), ...
    'adapt', @adaptAdjointOptimal, 'chapter','Ch.8  optimal guidance + App. A tgo error');
% SLOW ON PURPOSE -- see the RUNTIME note in the header.  Do not "speed it up"
% by passing 'h' here without re-auditing: the rows below are the audited ones.
P(end+1) = struct('key','adjointWithAutopilot', 'listing','Listing 23.3', ...
    'entry','adjointWithAutopilot(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.adjointWithAutopilot('benchmark',true,'assert',false, ...
                                                 'render',false,'plot',false), ...
    'adapt', @adaptAutopilot, 'chapter','Ch.23 homing loop with a detailed FCS');

% -- third wave.  Ch.21 -> Ch.23 is one chain: the bare airframe, its
%    linearisation, the flight-control system built on it, and the autopilot
%    that sets its time constant.  Ch.28 is the same PN kernel we already
%    check in 2-D, flown in three dimensions.  Like the second wave, every one
%    of these carries its own .source on every row, so their adapters call
%    selfCite() through the shared benchRows() and NOT cite().
%
%    All five default 'benchmark' TRUE, so a bare call IS a checked call for
%    them -- but the flag is passed explicitly anyway, for the same reason the
%    second wave passes it: the manifest must not depend on a default that a
%    later edit could flip.  benchOrDie() refuses an empty bench array rather
%    than letting a port that adjudicated nothing report zero failures.
P(end+1) = struct('key','airframeSim', 'listing','Listing 21.1', ...
    'entry','airframeSim(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.airframeSim('benchmark',true,'assert',false, ...
                                        'render',false,'plot',false), ...
    'adapt', @adaptAirframeSim, 'chapter','Ch.21 nonlinear airframe, two flight conditions');
P(end+1) = struct('key','airframeLinear', 'listing','Listing 21.2 (+22.1)', ...
    'entry','airframeLinear(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.airframeLinear('benchmark',true,'assert',false, ...
                                           'render',false,'plot',false), ...
    'adapt', @adaptAirframeLinear, 'chapter','Ch.21 linearised airframe + Tables 22.1/22.2');
P(end+1) = struct('key','flightControlDelay', 'listing','Listing 22.4', ...
    'entry','flightControlDelay(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.flightControlDelay('benchmark',true,'assert',false, ...
                                               'render',false,'plot',false), ...
    'adapt', @adaptFlightControlDelay, 'chapter','Ch.22 rate-gyro FCS with a pure delay');
% RUNS ITS NEIGHBOUR.  Its cross-check calls mgs.zarchan.adjointWithAutopilot
% to prove the byte-identical Ch.23 preamble has not drifted between the two
% ports, so this entry costs the neighbour's AIRFRAME a second time.  Left ON:
% that row is the only delete-experiment covering the duplication, and it is
% also the only row in that port that still bites at off-book parameters.
P(end+1) = struct('key','threeLoopAutopilot', 'listing','Listing 23.1', ...
    'entry','threeLoopAutopilot(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.threeLoopAutopilot('benchmark',true,'assert',false, ...
                                               'render',false,'plot',false), ...
    'adapt', @adaptThreeLoop, 'chapter','Ch.23 three-loop autopilot step response');
P(end+1) = struct('key','engage3d', 'listing','Listing 28.1', ...
    'entry','engage3d(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.engage3d('benchmark',true,'assert',false, ...
                                     'render',false,'plot',false), ...
    'adapt', @adaptEngage3d, 'chapter','Ch.28 3-D engagement vs a weaving target');

% ── wave 4.  These use adaptGeneric -- see its header for why the bespoke
%    adapters stop here.  ⚠️ shaping2d is METRIC; every other port is in feet.
P(end+1) = struct('key','shaping2d', 'listing','Listing 30.1', ...
    'entry','shaping2d(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.shaping2d('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('shaping2d','Listing 30.1',E), ...
    'chapter','Ch.30 trajectory shaping vs biased PN, STATIONARY target, METRIC');
P(end+1) = struct('key','shapingNonlinear', 'listing','Listing 24.2', ...
    'entry','shapingNonlinear(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.shapingNonlinear('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('shapingNonlinear','Listing 24.2',E), ...
    'chapter','Ch.24 nonlinear test of the trajectory-shaping law');
P(end+1) = struct('key','monteCarloHoming', 'listing','Listing 4.5', ...
    'entry','monteCarloHoming(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.monteCarloHoming('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('monteCarloHoming','Listing 4.5',E), ...
    'chapter','Ch.4 Monte Carlo miss dispersion');
P(end+1) = struct('key','shapingFilterAdjoint', 'listing','Listing 4.6', ...
    'entry','shapingFilterAdjoint(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.shapingFilterAdjoint('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('shapingFilterAdjoint','Listing 4.6',E), ...
    'chapter','Ch.4 the same dispersion in ONE run, via a shaping filter');

% ── wave 5.  DELIBERATELY LEAN ports: a faithful transcription, 3-10 scored
%    rows and 2-5 structural checks each, rather than the 30-60 rows waves 1-4
%    carried.  That depth was right for the chapters that bear on this
%    project's problem and those are done; these four are baselines the earlier
%    waves leaned on without ever checking on their own.  A lean port that
%    scores three plainly-stated numbers is not a weaker port -- it is a
%    smaller claim, honestly sized.  All four use adaptGeneric.
P(end+1) = struct('key','weaveSingleLag', 'listing','Listing 20.1', ...
    'entry','weaveSingleLag(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.weaveSingleLag('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('weaveSingleLag','Listing 20.1',E), ...
    'chapter','Ch.20 weaving target vs single-lag PN, nonlinear brute force');
P(end+1) = struct('key','weaveAdjoint', 'listing','Listing 20.2', ...
    'entry','weaveAdjoint(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.weaveAdjoint('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('weaveAdjoint','Listing 20.2',E), ...
    'chapter','Ch.20 adjoint of that loop -- every flight time in one run');
P(end+1) = struct('key','rk4SecondOrder', 'listing','Listing 5.1', ...
    'entry','rk4SecondOrder(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.rk4SecondOrder('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('rk4SecondOrder','Listing 5.1',E), ...
    'chapter','Ch.5  second-order network by fourth-order Runge-Kutta');
P(end+1) = struct('key','radomeEngagement', 'listing','Listing 6.2', ...
    'entry','radomeEngagement(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.radomeEngagement('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('radomeEngagement','Listing 6.2',E), ...
    'chapter','Ch.6  radome-slope parasitic feedback in a homing loop');
% -- the other six wave-5 ports.  Same wave, same leanness, same adaptGeneric;
%    they are separated only because an earlier registration pass missed them.
%    ⚠️ shapingLinear has NO 'plot' option -- do not add one to its call by
%    copy-paste from its neighbours; an unmatched name-value pair is an
%    arguments-block error, not a silently ignored one.
P(end+1) = struct('key','weaveCompensated', 'listing','Listing 20.4', ...
    'entry','weaveCompensated(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.weaveCompensated('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('weaveCompensated','Listing 20.4',E), ...
    'chapter','Ch.20 PN vs weave vs compensated-weave through a 5th-order lag');
P(end+1) = struct('key','fcsOpenLoop', 'listing','Listing 22.1', ...
    'entry','fcsOpenLoop(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.fcsOpenLoop('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('fcsOpenLoop','Listing 22.1',E), ...
    'chapter','Ch.22 open-loop FCS, 10-g step at sea level and 50 kft');
P(end+1) = struct('key','fcsRateGyro', 'listing','Listing 22.2', ...
    'entry','fcsRateGyro(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.fcsRateGyro('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('fcsRateGyro','Listing 22.2',E), ...
    'chapter','Ch.22 rate-gyro FCS, no actuator, Heun two-pass integration');
P(end+1) = struct('key','fcsBode', 'listing','Listing 22.3', ...
    'entry','fcsBode(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.fcsBode('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('fcsBode','Listing 22.3',E), ...
    'chapter','Ch.22 open-loop Bode of that FCS, and its gain/phase margins');
P(end+1) = struct('key','threeLoopOpenLoop', 'listing','Listing 23.2', ...
    'entry','threeLoopOpenLoop(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.threeLoopOpenLoop('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('threeLoopOpenLoop','Listing 23.2',E), ...
    'chapter','Ch.23 three-loop autopilot broken ahead of the actuator: margins vs tau/wcr/zeta');
P(end+1) = struct('key','shapingLinear', 'listing','Listing 24.1', ...
    'entry','shapingLinear(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.shapingLinear('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('shapingLinear','Listing 24.1',E), ...
    'chapter','Ch.24 linear zero-lag homing loop: trajectory shaping vs PN');

% -- wave 6.  Ten more LEAN ports, audited in pairs, all PASSED; none FAILED
%    and none unverified, so X stays empty and UNAUDITED empties out below.
%    Three chapter groups, not one:
%      Ch.26 (26.1 -> 26.6) is a single chain -- the matrix Riccati route to
%      the optimal gains, the Rusnak & Meir route to the SAME gains as an
%      independent check, both repeated for the cubic FCS, then the four-law
%      bake-off those gains feed.  cubicGainsAlt exists to disagree with
%      optimalGainsCubic; if the two ever agree for the wrong reason they are
%      wrong together and the bake-off inherits it, which is why BOTH are
%      wired rather than whichever looked cleaner.
%      Ch.19 (19.1 -> 19.3) is the apparent-target-jump chain: the nonlinear
%      engagement, the same engagement with a seeker lag, and the adjoint that
%      produces every homing time in one run.
%      Ch.31 (31.1) stands alone -- a seeded Monte Carlo ranking four guidance
%      laws against four random target maneuvers.
%    All ten use adaptGeneric.  All ten default 'benchmark' TRUE and the flag
%    is still passed explicitly, for the reason the third wave gives above.
% (!) lawEffectivenessMC has NO 'plot' option and IS SEEDED -- opts.seed
%    (20260824) is what makes its Monte Carlo rows reproducible.  Do not pass
%    a different seed here without re-auditing: the audited rows are the ones
%    that seed produces.  It is also the only wave-6 port that draws random
%    numbers at all.
%
% ⭐⭐ WHAT WIRING WAVE 6 TAUGHT US.  It took the board from 685 rows / 0 FAIL
%    to 838 rows / 6 FAIL, all six in wave-6 ports.  The six were left red and
%    characterised here rather than patched, because a red row is a RESULT and
%    because changing a port DEFAULT changes the audited configuration.  They
%    were then investigated properly and all six are green -- two by a measured
%    truncation envelope plus a new order-of-accuracy self-check, one by moving
%    a default onto the listing's own step, and one by discovering the row was
%    testing an absolute band the chapter never states.
%    ⚠️ NOT ONE of the six was fixed by widening a band.  If a later wave finds
%    itself widening one, that is the moment to stop and re-read this block.
%    ⭐ ALL SIX ARE NOW GREEN.  What follows is the corrected account; the
%    original one is kept only where it was wrong, because HOW it was wrong is
%    the part worth keeping.
%
%      optimalGainsSingleLag  4 rows -> 0.  ⚠️ The original note here said the
%        residual is "clean O(H)".  It is O(H^2): measured log-log slopes over
%        H = 3e-3 .. 1e-5 are 1.42, 1.912, 1.991, 2.000, 2.000.  The bands are
%        now a MEASURED truncation envelope keyed on (H/TAU)^2*(TAU^3/GAM),
%        capped and floored, and a new self-check 6 re-runs the whole
%        integration at 2H and scores the residual RATIO -- that is the row
%        which proves the residual is numerical rather than structural.
%        ⭐⭐ THE FINDING WORTH REMEMBERING, measured by deletion on the
%        shipped file: a 0.02 % typo in F(2,3) (1 -> 1.0002) passes EVERY
%        band, and makes the C3 row GREENER (6.379e-4 -> 4.380e-4).  Only
%        self-check 6 catches it.  At 1.00005 the whole port goes green.
%        ⇒ At the book's own H the numeric bands do NOT detect a small
%        transcription error; the STRUCTURAL check does.  Anyone tempted to
%        judge a port by its green benchmark rows should read that sentence
%        twice.
%      optimalGainsCubic      1 row -> 0.  cross_h default 5e-4 -> 1e-4, which
%        is Listing 26.1's OWN step (zarchan.txt:9359).  No band was widened;
%        the row now measures 0.1102 % against the unchanged 1 %.  Bit-identity
%        with optimalGainsSingleLag('H',1e-4) verified at max rel diff 0.  A new
%        row scores the Richardson ratio (measures 3.990, i.e. O(H^2)) and
%        SKIPS in the four cases where the ratio would not be a statement
%        about H at all.
%      guidanceLawBakeoff     1 row -> 0, and this one was NOT numerical.
%        The retired row tested "PN barely changes, 100 -> 10 rad/s" as an
%        ABSOLUTE band |ratio - 1| <= 0.25; we measure 1.605, converged.  The
%        chapter never makes an absolute claim: at zarchan.txt:9894/:9900 it
%        says the zero's decrease "does not significantly influence the
%        proportional-navigation results" IN CONTRAST TO the optimal law, whose
%        ratio over the same change is x118.6.  So the row now tests the
%        ORDERING the sentence pair actually draws, and the retired 1.6053 is
%        PRINTED INSIDE THE ROW with an explicit note that the literal absolute
%        reading does not hold.  ⚠️ A WZ sweep at 15 points, 1000 -> 3 rad/s,
%        is smooth and monotone: no threshold could sit between 10 and 5, so
%        "barely / significantly" was never a knee to be found.
%    None of the three is grounds for X: X is for a port whose AUDIT failed --
%    fabricated or ungreppable book values -- and all ten of these cleared that.
%    A red row is a result.  Hiding it by not wiring the port is not.
P(end+1) = struct('key','optimalGainsSingleLag', 'listing','Listing 26.1', ...
    'entry','optimalGainsSingleLag(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.optimalGainsSingleLag('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('optimalGainsSingleLag','Listing 26.1',E), ...
    'chapter','Ch.26 matrix Riccati in time-to-go -> optimal gains C1..C4, single-lag plant, vs the Ch.8 closed forms');
P(end+1) = struct('key','optimalGainsWeave', 'listing','Listing 26.2', ...
    'entry','optimalGainsWeave(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.optimalGainsWeave('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('optimalGainsWeave','Listing 26.2',E), ...
    'chapter','Ch.26 the same Riccati sweep for optimal WEAVE guidance gains, sinusoidal target, zero-lag FCS');
P(end+1) = struct('key','rusnakNumericalGains', 'listing','Listing 26.3', ...
    'entry','rusnakNumericalGains(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.rusnakNumericalGains('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('rusnakNumericalGains','Listing 26.3',E), ...
    'chapter','Ch.26 Rusnak & Meir route (three scalar ODEs, Fig. 26.10) to C1..C4 and N-prime(tgo), vs the closed form');
P(end+1) = struct('key','optimalGainsCubic', 'listing','Listing 26.4', ...
    'entry','optimalGainsCubic(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.optimalGainsCubic('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('optimalGainsCubic','Listing 26.4',E), ...
    'chapter','Ch.26 Riccati sweep for the SIX gains of a cubic FCS, and the Fig. 26.14 effective navigation ratio');
P(end+1) = struct('key','cubicGainsAlt', 'listing','Listing 26.5', ...
    'entry','cubicGainsAlt(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.cubicGainsAlt('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('cubicGainsAlt','Listing 26.5',E), ...
    'chapter','Ch.26 Rusnak & Meir (Ref. 6) route to those same six cubic gains -- the book''s own check on 26.4');
P(end+1) = struct('key','guidanceLawBakeoff', 'listing','Listing 26.6', ...
    'entry','guidanceLawBakeoff(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.guidanceLawBakeoff('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('guidanceLawBakeoff','Listing 26.6',E), ...
    'chapter','Ch.26 four guidance laws head to head through a cubic FCS, swept over the numerator zero WZ = 100/10/5 rad/s');
P(end+1) = struct('key','stepDisplacement', 'listing','Listing 19.1', ...
    'entry','stepDisplacement(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.stepDisplacement('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('stepDisplacement','Listing 19.1',E), ...
    'chapter','Ch.19 2-D nonlinear zero-lag PN with the target APPARENTLY jumping 200 ft at THOM s to go');
P(end+1) = struct('key','stepWithSeeker', 'listing','Listing 19.2', ...
    'entry','stepWithSeeker(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.stepWithSeeker('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('stepWithSeeker','Listing 19.2',E), ...
    'chapter','Ch.19 that same jump flown with a first-order SEEKER LAG, homing time swept for the Fig. 19.8 miss curve');
P(end+1) = struct('key','stepAdjointSingleLag', 'listing','Listing 19.3', ...
    'entry','stepAdjointSingleLag(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.stepAdjointSingleLag('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('stepAdjointSingleLag','Listing 19.3',E), ...
    'chapter','Ch.19 adjoint of Fig. 19.10: step-displacement miss vs homing time, and the Fig. 19.12 same-curve claim');
P(end+1) = struct('key','lawEffectivenessMC', 'listing','Listing 31.1', ...
    'entry','lawEffectivenessMC(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.lawEffectivenessMC('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('lawEffectivenessMC','Listing 31.1',E), ...
    'chapter','Ch.31 seeded Monte Carlo ranking PN / optimal / differential-game / hybrid against four random target maneuvers');

% -- wave 7.  TEN more LEAN ports, audited in pairs, all PASSED; none FAILED
%    and none unverified, so X and UNAUDITED both stay empty.  Three chapter
%    groups again:
%      Ch.7 (7.1 -> 7.4) is the fading-memory chain: the digital filter in the
%      homing loop, its Monte Carlo split of miss into a noise term and a
%      maneuver term, the adjoint that produces every flight time in one run,
%      and the three-state version that estimates target acceleration.
%      Ch.29 (29.1 -> 29.4) is the multiple-sampling-rate chain: the forward
%      loop with a filter sampler and a faster system sampler, its adjoint, the
%      three-state Kalman filter with PN / augmented-PN / optimal law options,
%      and that law-option model's adjoint.
%      Ch.19 (19.4, 19.5) closes the wave-6 Ch.19 chain: the fifth-order
%      binomial step-displacement adjoint and its forward twin.  The twin
%      exists to carry the ACCELERATION LIMIT that an adjoint cannot, so the
%      two are wired together on purpose -- 19.4 alone cannot see that mode.
%    All ten use adaptGeneric, all ten return .bench AND a .selfcheck carrying
%    BOTH .names and .oks (checked before wiring: the shaping2d/shapingNonlinear
%    D4 gap above is where a port computes its invariants and never puts them on
%    the board).  All ten default 'benchmark' TRUE and the flag is still passed
%    explicitly, for the reason the third wave gives above.
% (!) THREE of them -- fadingMemoryHoming, fadingMemoryMC, stepForwardBinomial5 --
%    have NO 'plot' option, and the wave-5 shapingLinear note applies: an
%    unmatched name-value pair is an arguments-block ERROR, not a silently
%    ignored one.  These calls pass only benchmark/assert/render, which all ten
%    accept, so the shape below is uniform and safe to copy.
% (!) THREE of them are SEEDED -- fadingMemoryHoming, fadingMemoryMC and
%    fadingMemoryThreeState each default opts.seed = 20260824, and that is what
%    makes their noise rows reproducible.  Do not pass a different seed here
%    without re-auditing: the audited rows are the ones that seed produces.
% (!) dualRateAdjoint and kalmanLawAdjoint each RUN THEIR OWN FORWARD TWIN
%    ('forward' defaults true) so the adjoint can be scored against it.  That is
%    the row those ports exist to produce, so the flag is deliberately not
%    turned off to save time -- same rule as threeLoopAutopilot's crosscheck.
P(end+1) = struct('key','fadingMemoryHoming', 'listing','Listing 7.1', ...
    'entry','fadingMemoryHoming(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.fadingMemoryHoming('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('fadingMemoryHoming','Listing 7.1',E), ...
    'chapter','Ch.7  digital fading-memory filter in the homing loop, noisy LOS rate');
P(end+1) = struct('key','fadingMemoryMC', 'listing','Listing 7.2', ...
    'entry','fadingMemoryMC(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.fadingMemoryMC('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('fadingMemoryMC','Listing 7.2',E), ...
    'chapter','Ch.7  Monte Carlo split of miss into noise (sigma) vs maneuver (mean)');
P(end+1) = struct('key','fadingMemoryAdjoint', 'listing','Listing 7.3', ...
    'entry','fadingMemoryAdjoint(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.fadingMemoryAdjoint('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('fadingMemoryAdjoint','Listing 7.3',E), ...
    'chapter','Ch.7  adjoint of a two-state digital fading memory filter in the homing loop');
P(end+1) = struct('key','fadingMemoryThreeState', 'listing','Listing 7.4', ...
    'entry','fadingMemoryThreeState(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.fadingMemoryThreeState('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('fadingMemoryThreeState','Listing 7.4',E), ...
    'chapter','Ch.7  three-state fading memory filter estimating target maneuver');
P(end+1) = struct('key','dualRateForward', 'listing','Listing 29.1', ...
    'entry','dualRateForward(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.dualRateForward('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('dualRateForward','Listing 29.1',E), ...
    'chapter','Ch.29 forward homing loop with two samplers at different rates');
P(end+1) = struct('key','dualRateAdjoint', 'listing','Listing 29.2', ...
    'entry','dualRateAdjoint(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.dualRateAdjoint('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('dualRateAdjoint','Listing 29.2',E), ...
    'chapter','Ch.29 adjoint of that dual-rate loop -- every flight time in one run, scored against its forward twin');
P(end+1) = struct('key','kalmanLawOptions', 'listing','Listing 29.3', ...
    'entry','kalmanLawOptions(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.kalmanLawOptions('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('kalmanLawOptions','Listing 29.3',E), ...
    'chapter','Ch.29 forward model with a three-state Kalman filter and PN / augmented-PN / optimal guidance-law options');
P(end+1) = struct('key','kalmanLawAdjoint', 'listing','Listing 29.4 (+29.3)', ...
    'entry','kalmanLawAdjoint(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.kalmanLawAdjoint('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('kalmanLawAdjoint','Listing 29.4 (+29.3)',E), ...
    'chapter','Ch.29 adjoint of the Kalman/law-option loop, with Listing 29.3''s forward twin transcribed in the same file');
P(end+1) = struct('key','stepAdjointBinomial5', 'listing','Listing 19.4', ...
    'entry','stepAdjointBinomial5(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.stepAdjointBinomial5('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('stepAdjointBinomial5','Listing 19.4',E), ...
    'chapter','Ch.19 fifth-order binomial miss from a step in target displacement');
P(end+1) = struct('key','stepForwardBinomial5', 'listing','Listing 19.5', ...
    'entry','stepForwardBinomial5(''benchmark'',true,''render'',false)', ...
    'call', @() mgs.zarchan.stepForwardBinomial5('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('stepForwardBinomial5','Listing 19.5',E), ...
    'chapter','Ch.19 forward twin of the 19.4 adjoint: a 200 ft step through a 5th-order binomial system, WITH the acceleration limit an adjoint cannot carry');

% -- wave 8.  TEN lean ports were written and audited in pairs; NINE PASSED and
%    are wired below.  Three chapter groups plus three singles:
%      Ch.1 (1.1, 1.2) is the book's own numerical-methods opener -- the RK2
%      second-order network and the difference-equation propagator.  Every
%      integration on this board rests on those two and neither had ever been
%      checked on its own.
%      Ch.4 (4.1 -> 4.4) is the random-number chain: twelve uniforms minus six,
%      the density built from those samples, the sampled standard deviation
%      against sample size, and white noise through a low-pass filter.
%      Ch.9 (9.1) is the discrete Riccati route to the Kalman gains; Ch.10
%      (10.1) is the ballistic flyout through an exponential atmosphere; Ch.18
%      (18.1) is the fifth-order binomial adjoint for NOISE miss distances
%      (range-independent, semiactive, active, glint).
%    All nine use adaptGeneric.  All nine default 'benchmark' TRUE and the flag
%    is still passed explicitly, for the reason the third wave gives above.
%    All nine were checked for .selfcheck.names AND .selfcheck.oks before being
%    wired -- the shaping2d/shapingNonlinear D4 gap is what that check exists
%    for -- and all nine assign both from their structuralChecks local.
% (!) noiseMissAdjoint has NO 'plot' option.  The wave-5 shapingLinear note
%    applies: an unmatched name-value pair is an arguments-block ERROR, not a
%    silently ignored one.  The calls below pass only benchmark/assert/render,
%    which all nine accept, so the shape is uniform and safe to copy.
% (!) FOUR are SEEDED -- gaussianGenerator, probabilityDensity, sampledStdDev
%    and lowPassWhiteNoise each default opts.seed = 20260824, and that is what
%    makes their distributional rows reproducible.  Do not pass a different
%    seed here without re-auditing: the audited rows are the ones that seed
%    produces.  They are also the only wave-8 ports that draw random numbers.
% ✅ RESOLVED ON WAVE 9 -- READ THIS BEFORE THE PARAGRAPH IT INTRODUCES.
%    THE TENTH PORT, radomeHomingLoop (Listing 18.2), FAILED AUDIT ON WAVE 8 and
%    was deliberately NOT WIRED, NOT in X, and NOT counted in AUDITED_TOTAL,
%    because "fix or cut" was undecided.  It was FIXED, RE-AUDITED and the
%    repair PASSED, so it is now in P in the WAVE-9 BLOCK below and IS counted.
%    ⇒ Everything from here to the end of this paragraph is the HISTORICAL
%    finding, kept because it is what the repaired predicate was built against,
%    NOT a description of the file on disk today.  The current file and the fix
%    are described in the wave-9 block.
%    What the wave-8 audit found, in one line each:
%      * every citation is REAL -- the grep evidence, the minus-sign class in
%        both directions, and the ODEs all check out against listing_18_02.m.
%      * bisectTcrit returns NaN at BOTH closing velocities, so rows 3/5/6 --
%        the three rows that are the port's entire reason to exist beside
%        radomeEngagement -- all SKIP, self-check 5 goes red, and R.pass = 0.
%        Its own comment claims the dip predicate is monotone in T; measured, it
%        is not (T = 0.22/0.26/0.28/0.30 give 0.639/0.053/1.608/0.000), and the
%        metric is dominated by the terminal 1/tgo transient, so no bracket
%        exists to widen to.
%      * the header prints DERIVED expectations ("neutral T near 0.24-0.25 s")
%        that the run refutes, and with 3/5/6 skipped the only surviving value
%        rows evaluate the book's printed formula with the book's printed
%        constants -- no simulation state is touched, so no implementation could
%        fail them.
%    ⇒ The two exits offered were: discriminate on sign changes / rail contact
%    over the PRE-ENDGAME window and exclude the last ~0.5 s from the dip; or
%    delete rows 3/5/6 and self-check 5 and say the time-domain route was not
%    achieved.  ⭐ WAVE 9 TOOK THE FIRST and re-audited.  END OF HISTORICAL NOTE.
P(end+1) = struct('key','secondOrderSim', 'listing','Listing 1.1', ...
    'entry','secondOrderSim(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.secondOrderSim('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('secondOrderSim','Listing 1.1',E), ...
    'chapter','Ch.1  second-order network by second-order Runge-Kutta -- the integrator every other port rests on');
P(end+1) = struct('key','differenceEquation', 'listing','Listing 1.2', ...
    'entry','differenceEquation(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.differenceEquation('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('differenceEquation','Listing 1.2',E), ...
    'chapter','Ch.1  difference-equation propagation of a first-order lag, against its closed form');
P(end+1) = struct('key','gaussianGenerator', 'listing','Listing 4.1', ...
    'entry','gaussianGenerator(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.gaussianGenerator('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('gaussianGenerator','Listing 4.1',E), ...
    'chapter','Ch.4  Gaussian random numbers the hard way: twelve uniforms, subtract six');
P(end+1) = struct('key','probabilityDensity', 'listing','Listing 4.2', ...
    'entry','probabilityDensity(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.probabilityDensity('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('probabilityDensity','Listing 4.2',E), ...
    'chapter','Ch.4  a probability density function built from samples, scored against the analytic Gaussian');
P(end+1) = struct('key','sampledStdDev', 'listing','Listing 4.3', ...
    'entry','sampledStdDev(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.sampledStdDev('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('sampledStdDev','Listing 4.3',E), ...
    'chapter','Ch.4  sampled standard deviation as a function of sample size');
P(end+1) = struct('key','lowPassWhiteNoise', 'listing','Listing 4.4', ...
    'entry','lowPassWhiteNoise(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.lowPassWhiteNoise('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('lowPassWhiteNoise','Listing 4.4',E), ...
    'chapter','Ch.4  white noise through a first-order low-pass filter, and the sigma it settles to');
P(end+1) = struct('key','riccatiSolver', 'listing','Listing 9.1', ...
    'entry','riccatiSolver(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.riccatiSolver('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('riccatiSolver','Listing 9.1',E), ...
    'chapter','Ch.9  Kalman gains from the discrete Riccati equations');
P(end+1) = struct('key','trajectorySim', 'listing','Listing 10.1', ...
    'entry','trajectorySim(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.trajectorySim('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('trajectorySim','Listing 10.1',E), ...
    'chapter','Ch.10 ballistic flyout: gravity plus zero-lift drag through an exponential atmosphere');
P(end+1) = struct('key','noiseMissAdjoint', 'listing','Listing 18.1', ...
    'entry','noiseMissAdjoint(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.noiseMissAdjoint('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('noiseMissAdjoint','Listing 18.1',E), ...
    'chapter','Ch.18 adjoint of the 5th-order binomial loop for NOISE miss: range-independent, semiactive, active and glint');

% -- wave 9.  TEN more LEAN ports were written and audited in pairs; ALL TEN
%    PASSED, none FAILED, none unverified.  ELEVEN entries go in below, because
%    the eleventh is the REPAIR of radomeHomingLoop (Listing 18.2) -- see the
%    wave-8 block above for what it failed on, and the note at the end of this
%    block for what changed.  Chapter groups:
%      Ch.25 (25.1 -> 25.3) is the weaving-target ESTIMATION chain: the
%      three-state linear filter that does NOT know the target weaves, the
%      four-state filter that does, and the extended filter that estimates the
%      weave FREQUENCY as a fifth state.  25.1 exists to be beaten by 25.2;
%      wiring only 25.2 would leave "improves on" as an unmeasured adjective.
%      Ch.27 (27.1) is the fixed MMAE bank of three of those four-state filters
%      at 1/2/4 r/s with Bayes' rule blending them -- the same estimation
%      problem when the frequency is unknown but bracketed.
%      Ch.9 (9.2, 9.3) is the Kalman-filter homing loop and the Monte Carlo
%      bake-off of PN / augmented PN / optimal guidance behind it.  Ch.9's
%      Listing 9.1 (the discrete Riccati gains) went in on wave 8; these two are
%      what those gains are FOR.
%      Ch.30 (30.2, 30.3) is the glint pair: the discrete Riccati sweep that
%      sizes the filter, and the Monte Carlo that flies it.  Listing 30.1
%      (shaping2d) is already on the board.
%      Ch.16/Ch.17 (16.1, 17.1) are the ballistic-target pair: the drag +
%      flat-Earth re-entry itself, and the extended Kalman filter that estimates
%      its ballistic coefficient.  17.1 cross-checks against 16.1's closed form,
%      so the two belong together and are wired together.
%    All eleven use adaptGeneric, and all eleven were checked for
%    .selfcheck.names AND .selfcheck.oks BEFORE being wired -- the
%    shaping2d/shapingNonlinear D4 gap is exactly what that check exists for,
%    and a port whose invariants never reach the board is a port whose delete
%    experiments nobody can see.  All eleven default 'benchmark' TRUE and the
%    flag is still passed explicitly, for the reason the third wave gives above.
% (!) THE CALL SHAPE IS UNIFORM AND WAS CHECKED, NOT ASSUMED.  Every one of the
%    eleven accepts benchmark / assert / render; only five (optimalGuidanceMC,
%    weaveKalmanThreeState, weaveKalmanFilter, ballisticCoeffEKF,
%    ballisticTargetSim) also accept 'plot'.  The calls below therefore pass
%    ONLY benchmark/assert/render, which all eleven take.  The wave-5
%    shapingLinear note applies verbatim: an unmatched name-value pair is an
%    arguments-block ERROR, not a silently ignored one, so do not add 'plot'
%    here by copy-paste from a neighbour.
% (!) EIGHT ARE SEEDED -- kalmanHomingLoop, optimalGuidanceMC,
%    weaveKalmanThreeState, weaveKalmanFilter, weaveExtendedKalman,
%    mmaeWeaveFilters, glintMonteCarlo and ballisticCoeffEKF each default
%    opts.seed = 20260824, and that is what makes their distributional rows
%    reproducible.  Do not pass a different seed here without re-auditing: the
%    audited rows are the ones that seed produces.  discreteRiccati,
%    ballisticTargetSim and radomeHomingLoop draw no random numbers at all.
% (!) THREE CARRY A COST KNOB and none of them is passed here, for the reason
%    the RUNTIME note in the header gives about adjointWithAutopilot's 'h':
%    optimalGuidanceMC runs 200 (the book runs 50, and that port's own header
%    explains why a 50-run sigma cannot ADJUDICATE an ordering -- the first-50
%    subset is kept as an INFO row so the book's experiment stays visible),
%    glintMonteCarlo runs 100 over five flight times rather than the book's
%    fifty, and kalmanHomingLoop medians over nseed = 9 flights because one
%    flight's settle time moves 1.8-4.6 s on noise realisation alone.  Changing
%    any of the three changes the audited configuration.
% (!) RUNTIME, BY STEP COUNT AND NOT BY STOPWATCH.  MATLAB was deliberately not
%    run while these entries were written, so what follows is an ESTIMATE and is
%    labelled as one.  Ch.25/27 integrate at H = 1e-3 over 10 s, and
%    optimalGuidanceMC is 200 runs x 3 laws x 20 flight times.  Expect this wave
%    to be the most expensive one on the board.  Read the WALL CLOCK line the
%    board prints -- that one is measured, and this paragraph is not.  If it is
%    too slow, run ('only', {...}) and SAY WHICH PORTS YOU SKIPPED.
P(end+1) = struct('key','kalmanHomingLoop', 'listing','Listing 9.2', ...
    'entry','kalmanHomingLoop(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.kalmanHomingLoop('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('kalmanHomingLoop','Listing 9.2',E), ...
    'chapter','Ch.9  homing loop closed around a Kalman filter, matched and mismatched heading-error variance');
P(end+1) = struct('key','optimalGuidanceMC', 'listing','Listing 9.3', ...
    'entry','optimalGuidanceMC(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.optimalGuidanceMC('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('optimalGuidanceMC','Listing 9.3',E), ...
    'chapter','Ch.9  Monte Carlo bake-off of PN / augmented PN / optimal guidance behind that filter');
P(end+1) = struct('key','weaveKalmanThreeState', 'listing','Listing 25.1', ...
    'entry','weaveKalmanThreeState(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.weaveKalmanThreeState('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('weaveKalmanThreeState','Listing 25.1',E), ...
    'chapter','Ch.25 the ORIGINAL three-state linear Kalman filter vs a weaving target -- the baseline Listing 25.2 improves on');
P(end+1) = struct('key','weaveKalmanFilter', 'listing','Listing 25.2', ...
    'entry','weaveKalmanFilter(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.weaveKalmanFilter('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('weaveKalmanFilter','Listing 25.2',E), ...
    'chapter','Ch.25 four-state weave Kalman filter -- the ESTIMATION side of a weaving target');
P(end+1) = struct('key','weaveExtendedKalman', 'listing','Listing 25.3', ...
    'entry','weaveExtendedKalman(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.weaveExtendedKalman('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('weaveExtendedKalman','Listing 25.3',E), ...
    'chapter','Ch.25 EXTENDED weave filter: the weave frequency becomes a fifth state, its magnitude estimated and its sign not');
P(end+1) = struct('key','mmaeWeaveFilters', 'listing','Listing 27.1', ...
    'entry','mmaeWeaveFilters(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.mmaeWeaveFilters('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('mmaeWeaveFilters','Listing 27.1',E), ...
    'chapter','Ch.27 fixed MMAE bank: three four-state weave filters at 1/2/4 r/s blended by Bayes rule into the compensated weave law');
P(end+1) = struct('key','discreteRiccati', 'listing','Listing 30.2', ...
    'entry','discreteRiccati(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.discreteRiccati('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('discreteRiccati','Listing 30.2',E), ...
    'chapter','Ch.30 discrete Riccati sweep sizing the glint filter, over flight time and over sampling time');
P(end+1) = struct('key','glintMonteCarlo', 'listing','Listing 30.3', ...
    'entry','glintMonteCarlo(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.glintMonteCarlo('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('glintMonteCarlo','Listing 30.3',E), ...
    'chapter','Ch.30 Monte Carlo miss against GLINT noise, plus the book''s two controlled experiments on tau and on the process-noise gamma');
P(end+1) = struct('key','ballisticTargetSim', 'listing','Listing 16.1', ...
    'entry','ballisticTargetSim(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.ballisticTargetSim('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('ballisticTargetSim','Listing 16.1',E), ...
    'chapter','Ch.16 ballistic target properties -- drag + flat-Earth gravity re-entry, swept over ballistic coefficient');
P(end+1) = struct('key','ballisticCoeffEKF', 'listing','Listing 17.1', ...
    'entry','ballisticCoeffEKF(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.ballisticCoeffEKF('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('ballisticCoeffEKF','Listing 17.1',E), ...
    'chapter','Ch.17 extended Kalman filter for ballistic coefficient estimation (Gelb''s 1-D example), cross-checked against Ch.16''s closed form');
% ⭐ THE REPAIR.  radomeHomingLoop (Listing 18.2) was written on wave 8, FAILED
%    its audit, and was deliberately left off the board in a third position that
%    was neither P nor X -- "fix or cut, undecided".  It was FIXED and RE-
%    AUDITED, the repair PASSED, and it is wired here.  AUDITED_TOTAL moves for
%    it exactly as the wave-8 note said it would.
%    WHAT THE FIX WAS, because that is the part worth keeping: the failure was
%    the PREDICATE, not the bracket, and NOTHING WAS WIDENED.  The old version
%    bisected on the largest single-sample DROP over the WHOLE trace; since
%    TGO = TF - T + 1e-5 the 1/TGO gain diverges in the last samples and puts
%    EVERY case on the rail, stable or not, so no bracket existed, both
%    bisections returned NaN, and the three rows that are this port's entire
%    reason to exist beside radomeEngagement all went dark.  "Oscillating" is
%    now judged over the PRE-ENDGAME window t <= tF - guard (0.5 s), where it
%    means the recorded command TOUCHES EITHER RAIL or CHANGES SIGN -- the
%    book's own two-part phrase kept whole (:5466, :5474).  The guard is applied
%    identically to every case and to both sides of every transition, so it
%    cannot move a verdict by itself.
%    ⭐⭐ AND THE CLAIM THAT WAS ONLY A COMMENT IS NOW A ROW.  The old file
%    asserted monotonicity in T in a COMMENT and the assertion was false
%    (T = 0.22/0.26/0.28/0.30 measured 0.639/0.053/1.608/0.000).  Self-check 4
%    now REQUIRES exactly one flip across each bracket.  A comment cannot be
%    caught; a row can.  That is the same lesson as the promised-guards-that-do-
%    not-exist finding this repo hit three times in one day, and it is why this
%    port was worth repairing rather than cutting.
P(end+1) = struct('key','radomeHomingLoop', 'listing','Listing 18.2', ...
    'entry','radomeHomingLoop(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.radomeHomingLoop('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('radomeHomingLoop','Listing 18.2',E), ...
    'chapter','Ch.18 radome homing loop WITH the 7-g command limit: the TIME-DOMAIN route to the critical time constant Ch.6 got by root locus');

% ── WAVE 10 ────────────────────────────────────────────────────────────
%    2026-08-24: TEN more lean ports, audited in pairs, ALL TEN PASSED --
%    Ch.11 Listings 11.1/11.2/11.3/11.4, Ch.12 Listings 12.1/12.2 and Ch.13
%    Listings 13.1/13.2/13.3/13.4.  None FAILED and none is unverified, so X
%    and UNAUDITED both stay empty.  AUDITED_TOTAL 69 -> 79 (+10, one per port
%    ADDED TO P) and the guard reads 79 + 0 == 79 + 0.
%    ⭐ THE THREE CHAPTERS CHAIN, and that is why they were taken together:
%    Ch.11 builds the inverse-square field and the ballistic velocity formula,
%    Ch.12 sizes the two-stage booster and flies its gravity turn, Ch.13 steers
%    that same booster -- first with Lambert, then with GEM.
%    ⚠️ Listing 13.2's lambertpz appears in THREE of these files: lambertEfficient
%    boards it on its own, and boosterLambert and gemSimulation each carry a
%    local copy, because MATLAB local functions cannot be shared across files.
%    They are therefore three INDEPENDENT transcriptions of one routine, not one
%    shared one -- if they ever disagree, that is a finding, not a refactor.
%    ⚠️ ENTRY STRINGS EXPANDED, NOT CHANGED.  Three of these were audited by
%    their bare call (gravityFrames, thrustWeight, gravityTurn).  All ten are
%    wired here with the suite's standard triple: 'benchmark',true (already the
%    default), 'assert',false and 'render',false.  assert=false is the one that
%    matters -- the runner try/catches, so a port that ERRORS on a failing
%    invariant loses EVERY one of its rows and the board shows one red line
%    where a dozen should be.  Warn-and-return puts the failure on the board as
%    a row instead.  Neither flag touches a computed value.
P(end+1) = struct('key','gravityField', 'listing','Listing 11.1', ...
    'entry','gravityField(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.gravityField('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('gravityField','Listing 11.1',E), ...
    'chapter','Ch.11 gravity field: flat Earth vs Newton, one launch flown twice');
P(end+1) = struct('key','gravityFrames', 'listing','Listing 11.2', ...
    'entry','gravityFrames(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.gravityFrames('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('gravityFrames','Listing 11.2',E), ...
    'chapter','Ch.11 one gravity field, two coordinate systems, integrated side by side in a single loop');
P(end+1) = struct('key','orbitGenerator', 'listing','Listing 11.3', ...
    'entry','orbitGenerator(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.orbitGenerator('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('orbitGenerator','Listing 11.3',E), ...
    'chapter','Ch.11 orbit generator: the lambda = r0*V^2/GM family, circle through ellipse to escape and hyperbola');
P(end+1) = struct('key','velocityFormula', 'listing','Listing 11.4', ...
    'entry','velocityFormula(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.velocityFormula('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('velocityFormula','Listing 11.4',E), ...
    'chapter','Ch.11 the ballistic velocity formula, simulation vs closed form');
P(end+1) = struct('key','thrustWeight', 'listing','Listing 12.1', ...
    'entry','thrustWeight(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.thrustWeight('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('thrustWeight','Listing 12.1',E), ...
    'chapter','Ch.12 two-stage booster thrust-weight sizing, plus the 1-D burn integration that checks it');
P(end+1) = struct('key','gravityTurn', 'listing','Listing 12.2', ...
    'entry','gravityTurn(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.gravityTurn('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('gravityTurn','Listing 12.2',E), ...
    'chapter','Ch.12 booster sizing plus the gravity-turn trajectory that booster actually flies');
P(end+1) = struct('key','lambertBruteForce', 'listing','Listing 13.1', ...
    'entry','lambertBruteForce(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.lambertBruteForce('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('lambertBruteForce','Listing 13.1',E), ...
    'chapter','Ch.13 Lambert''s problem solved by brute-force scan of the flight-path angle');
P(end+1) = struct('key','lambertEfficient', 'listing','Listing 13.2', ...
    'entry','lambertEfficient(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.lambertEfficient('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('lambertEfficient','Listing 13.2',E), ...
    'chapter','Ch.13 the EFFICIENT Lambert routine -- secant search on flight-path angle, bracketed by the escape-velocity condition');
P(end+1) = struct('key','boosterLambert', 'listing','Listing 13.3', ...
    'entry','boosterLambert(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.boosterLambert('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('boosterLambert','Listing 13.3',E), ...
    'chapter','Ch.13 two-stage booster steered by Lambert guidance -- the first listing that closes the Lambert solver in a loop, plus Listing 13.2''s lambertpz and its Table 13.2');
P(end+1) = struct('key','gemSimulation', 'listing','Listing 13.4', ...
    'entry','gemSimulation(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.gemSimulation('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('gemSimulation','Listing 13.4',E), ...
    'chapter','Ch.13 GEM (general energy management) booster steering, with Listing 13.3''s Lambert-steered twin for the Fig. 13.12/13.13 comparisons');

% ── WAVE 11 ────────────────────────────────────────────────────────────
%    2026-08-24: TEN more lean ports, audited in pairs, ALL TEN PASSED --
%    Ch.14 Listings 14.1/14.2, Ch.15 Listings 15.1/15.2, Ch.16 Listing 16.2,
%    Ch.28 Listings 28.2/28.3/28.4 and Ch.32 Listings 32.1/32.2.  None FAILED
%    and none is unverified, so X and UNAUDITED both stay empty.  AUDITED_TOTAL
%    79 -> 89 (+10, one per port ADDED TO P) and the guard reads 89 + 0 == 89 + 0.
%    ⭐ THIS WAVE IS THE EXO-ATMOSPHERIC SPINE.  Wave 10 built the field (Ch.11),
%    sized the booster (Ch.12) and steered it (Ch.13); this wave INTERCEPTS with
%    it.  Ch.14 shoots at a ballistic and then at a boosting target, Ch.15
%    replaces the guidance law with predictive and then with pulsed, Ch.16 brings
%    the engagement back down into the atmosphere, Ch.28 lifts the whole thing to
%    three dimensions on a rotating Earth, and Ch.32 asks the kinematic question
%    the other eight assume an answer to -- how much interceptor velocity a given
%    flight time actually costs.
%    ⚠️ LISTING 13.2's lambertpz IS NOW IN SIX FILES.  Wave 10 noted three;
%    ballisticIntercept, predictiveGuidance and pulsedGuidance each carry a local
%    copy too, because MATLAB local functions cannot be shared across files.
%    They remain INDEPENDENT transcriptions of one routine -- if any two of the
%    six ever disagree, that is a finding, not a refactor.
%    ⚠️⚠️ UNITS ARE NOT UNIFORM ACROSS THIS WAVE and the board must not be read
%    as though they were.  Ch.14/15/16/28 use the book's usual ENGLISH units
%    (ft, ft/s, g = 32.2, A = 2.0926e7 ft); CHAPTER 32 IS ONE OF THE BOOK'S
%    METRIC CORNERS (m, m/s, km).  Each port declares its own R.units and the
%    renderer prints it -- do not compare a Ch.32 magnitude with a Ch.14 one
%    without converting first.
%    ⚠️ ENTRY STRINGS use the suite's standard triple: 'benchmark',true (already
%    the default), 'assert',false and 'render',false.  assert=false is the one
%    that matters -- the runner try/catches, so a port that ERRORS on a failing
%    invariant loses EVERY one of its rows and the board shows one red line where
%    a dozen should be.  Warn-and-return puts the failure on the board as a row
%    instead.  Neither flag touches a computed value.
P(end+1) = struct('key','ballisticIntercept', 'listing','Listing 14.1', ...
    'entry','ballisticIntercept(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.ballisticIntercept('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('ballisticIntercept','Listing 14.1',E), ...
    'chapter','Ch.14 strategic intercept of a BALLISTIC target -- impulsive Lambert-launched interceptor in Earth-centred inverse-square gravity, carrying Listing 13.2''s lambertpz and the book''s own predictpz prediction routine');
P(end+1) = struct('key','boostingIntercept', 'listing','Listing 14.2', ...
    'entry','boostingIntercept(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.boostingIntercept('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('boostingIntercept','Listing 14.2',E), ...
    'chapter','Ch.14 strategic intercept of a BOOSTING target -- impulsive Lambert-placed interceptor vs a two-stage gravity-turn booster, with Listing 13.2''s lambertpz carried in and a thrust-deleted twin that prices what target thrust costs the interceptor');
P(end+1) = struct('key','predictiveGuidance', 'listing','Listing 15.1', ...
    'entry','predictiveGuidance(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.predictiveGuidance('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('predictiveGuidance','Listing 15.1',E), ...
    'chapter','Ch.15 PREDICTIVE guidance against a BOOSTING target -- impulsively launched ballistic interceptor vs a two-stage booster in ECI, with the predictor-step and tgo-scale sweeps that price prediction error');
P(end+1) = struct('key','pulsedGuidance', 'listing','Listing 15.2', ...
    'entry','pulsedGuidance(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.pulsedGuidance('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('pulsedGuidance','Listing 15.2',E), ...
    'chapter','Ch.15 PULSED guidance -- the Listing 14.1 exoatmospheric engagement flown by a law that can only fix the acceleration MAGNITUDE and so must choose each burn''s DURATION instead');
P(end+1) = struct('key','ballisticEngagement', 'listing','Listing 16.2', ...
    'entry','ballisticEngagement(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.ballisticEngagement('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('ballisticEngagement','Listing 16.2',E), ...
    'chapter','Ch.16 ballistic target ENGAGEMENT -- PN and APN interceptor against a re-entering threat, on the inverse and the stressing geometry, with delete experiments on the 7-g acceleration limit');
P(end+1) = struct('key','icbmTrajectory', 'listing','Listing 28.2', ...
    'entry','icbmTrajectory(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.icbmTrajectory('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('icbmTrajectory','Listing 28.2',E), ...
    'chapter','Ch.28 ballistic-missile trajectory generator -- a three-dimensional Lambert solve picks the impulsive launch velocity, then a two-body ECI integration flies Monte Carlo -> Las Vegas in 2000 s');
P(end+1) = struct('key','keplerVsDirect', 'listing','Listing 28.3', ...
    'entry','keplerVsDirect(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.keplerVsDirect('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('keplerVsDirect','Listing 28.3',E), ...
    'chapter','Ch.28 direct integration of the ballistic-target equations of motion, checked against KEPLER1 propagation -- one trajectory, two independent propagators, plus a delete experiment');
P(end+1) = struct('key','strategicEngagement', 'listing','Listing 28.4', ...
    'entry','strategicEngagement(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.strategicEngagement('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('strategicEngagement','Listing 28.4',E), ...
    'chapter','Ch.28 three-dimensional strategic engagement -- Lambert-boosted missile from Atlantic City intercepting a Monte Carlo -> Las Vegas ballistic target, aim point from Kepler');
P(end+1) = struct('key','impulsiveIcbm', 'listing','Listing 32.1', ...
    'entry','impulsiveIcbm(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.impulsiveIcbm('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('impulsiveIcbm','Listing 32.1',E), ...
    'chapter','Ch.32 two-dimensional IMPULSIVE ICBM trajectory flown from a Lambert-solved launch velocity, minimum-energy and lofted -- METRIC units (m, m/s, km), NOT the book''s usual ft');
P(end+1) = struct('key','kinematicEngagementMC', 'listing','Listing 32.2', ...
    'entry','kinematicEngagementMC(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.kinematicEngagementMC('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('kinematicEngagementMC','Listing 32.2',E), ...
    'chapter','Ch.32 kinematics of intercepting a ballistic target -- required interceptor velocity vs flight time, impulsive interceptor against the impulsive ICBM of Listing 32.1');

% ── WAVE 12 ──────────────────────────────────────────────────────
%    2026-08-24: NINE more lean ports, audited in pairs, ALL NINE PASSED --
%    Ch.32 Listings 32.3/32.4/32.5, Ch.33 Listings 33.1/33.2 and Ch.34
%    Listings 34.1/34.2/34.3/34.4.  None FAILED and none is unverified, so X
%    and UNAUDITED both stay empty.  AUDITED_TOTAL 89 -> 98 (+9, one per port
%    ADDED TO P) and the guard reads 98 + 0 == 98 + 0.
%    ⭐⭐⭐ THIS IS THE LAST WAVE.  Every numbered listing in the book is now
%    wired.  That is a COUNT TAKEN IN THIS FILE, not an expectation: the 98
%    'listing' labels in P name 98 distinct NN.MM listings, and the _listings
%    directory holds 98 listing_NN_MM.m files; the set difference is empty in
%    both directions.  Wave 11 left exactly these nine outstanding.
%    ⭐⭐ WHAT THIS WAVE ADDS, structurally.  Ch.32's remaining three listings
%    (32.3/32.4/32.5) are ONE success predicate asked three different
%    questions -- sweep the INTERCEPTOR BASE (operationalAreas3d), sweep the
%    TARGET LAUNCH SITE (launchAreaDenied), sweep the TARGET IMPACT POINT
%    (defendedArea3d).  They share a predicate, so if any two of the three
%    ever disagree about a cell all three visit, that is a finding, not a
%    refactor.  Ch.33 is a truth model (nominalIcbm) plus the filters that
%    track it (icbmTrackingFilters) -- the same pairing Ch.14/15 used, and
%    again the chapter's claim is an ORDERING between two filters, not either
%    filter's absolute error.  Ch.34 is the boost-phase stereo chapter: a
%    linearised 1-D engagement (stereoLinear1d), a truth-model booster
%    generator (stereoTargetGen), the nonlinear 2-D engagement
%    (stereoNonlinear2d) and the Monte Carlo with stereo tracking
%    (stereoTrackingMC).
%    ⚠⚠ UNITS SPLIT INSIDE THIS WAVE, exactly as they did in wave 11, and the
%    board must not be read as though they did not.  CH.32 IS METRIC (km,
%    km/s, deg); CH.33 AND CH.34 ARE THE BOOK'S USUAL ENGLISH UNITS (ft, ft/s,
%    g = 32.2, A = 2.0926e7 ft), with Ch.34 reporting DIVERT in m/s via the
%    listing's own /3.28.  Each port declares its own R.units and the renderer
%    prints it.  Do not compare a Ch.32 magnitude with a Ch.34 one unconverted.
%    ⚠⚠ TWO ENTRY STRINGS WERE NORMALISED from what the audit ran, and
%    neither change touches a computed value:
%      * nominalIcbm was audited as the bare call with no options at all.
%        Left alone it would run with assert = TRUE, and .entry is printed by
%        the renderer as 'mgs.zarchan.%s', so the audit's own spelling
%        'mgs.zarchan.nominalIcbm' would have rendered as
%        'mgs.zarchan.mgs.zarchan.nominalIcbm'.
%      * defendedArea3d was audited without 'assert', which also defaults TRUE.
%    assert = TRUE is the flag that matters, for the reason wave 11 already
%    gave: the runner try/catches, so a port that ERRORS on a failing
%    invariant loses EVERY one of its rows and the board shows one red line
%    where a dozen should be.  Warn-and-return puts the failure on the board
%    as a row instead.  All nine functions declare benchmark/assert/render
%    (checked in their arguments blocks), so all nine now take the suite's
%    standard triple.
%    ⚠ nominalIcbm's listing label was given by the audit as the bare '33.1';
%    it is written 'Listing 33.1' here because every other label in P has that
%    shape and the coverage count above is taken off these strings.
P(end+1) = struct('key','operationalAreas3d', 'listing','Listing 32.3', ...
    'entry','operationalAreas3d(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.operationalAreas3d('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('operationalAreas3d','Listing 32.3',E), ...
    'chapter','Ch.32 three-dimensional OPERATIONAL AREAS -- where a velocity-limited impulsive interceptor can be based and still reach a 10,000-km minimum-energy ICBM, swept over longitude, latitude, launch time and intercept time with no differential equation anywhere');
P(end+1) = struct('key','launchAreaDenied', 'listing','Listing 32.4', ...
    'entry','launchAreaDenied(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.launchAreaDenied('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('launchAreaDenied','Listing 32.4',E), ...
    'chapter','Ch.32 three-dimensional LAUNCH AREA DENIED -- possible target launch sites that a 5 km/s impulsive interceptor at 60-deg longitude can deny, swept over longitude/latitude with the interceptor launch and intercept times varied; the 32.3/32.4/32.5 slice of one shared success predicate');
P(end+1) = struct('key','defendedArea3d', 'listing','Listing 32.5', ...
    'entry','defendedArea3d(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.defendedArea3d('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('defendedArea3d','Listing 32.5',E), ...
    'chapter','Ch.32 defended area: target launch site and interceptor fixed, target IMPACT POINT swept; which impact points a 5-km/s interceptor 6660 km downrange can defend');
P(end+1) = struct('key','nominalIcbm', 'listing','Listing 33.1', ...
    'entry','nominalIcbm(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.nominalIcbm('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('nominalIcbm','Listing 33.1',E), ...
    'chapter','Ch.33 trajectory generator for a NOMINAL ICBM -- two-stage APS booster: 20 s straight up, an open-loop gravity turn to 100 s, then closed-loop Lambert steering onto a 7000-km aim point; the TRUTH MODEL that Listing 33.2''s boost-phase Kalman filters track');
P(end+1) = struct('key','icbmTrackingFilters', 'listing','Listing 33.2', ...
    'entry','icbmTrackingFilters(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.icbmTrackingFilters('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('icbmTrackingFilters','Listing 33.2',E), ...
    'chapter','Ch.33 boost-phase filtering options -- one ICBM trajectory tracked by two decoupled linear polynomial Kalman filters, a two-state one handed a perfect acceleration template and a three-state one handed nothing; the chapter''s argument is the ORDERING between them once the template is even slightly wrong');
P(end+1) = struct('key','stereoLinear1d', 'listing','Listing 34.1', ...
    'entry','stereoLinear1d(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.stereoLinear1d('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('stereoLinear1d','Listing 34.1',E), ...
    'chapter','Ch.34 one-dimensional boost-phase engagement on LINEARIZED geometry -- PN vs APN against an IRBM/ICBM apparent maneuver (parabola or real thrust/weight), and the two KKV lateral-divert formulas DVMVR = 0.25 nTMAX tF and DVPIP = 1.5 PIP/tF that Ch.34 sizes a kill vehicle with; feet and g = 32.2, divert reported in m/s via the listing''s own 3.28');
P(end+1) = struct('key','stereoTargetGen', 'listing','Listing 34.2', ...
    'entry','stereoTargetGen(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.stereoTargetGen('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('stereoTargetGen','Listing 34.2',E), ...
    'chapter','Ch.34 target trajectory generator for boost-phase intercept -- Lambert-steered single-stage IRBM (ITGT=1) and two-stage APS ICBM (ITGT=2) boosters flown to impact on a round, non-rotating, atmosphere-free Earth; a TRUTH MODEL, so it is scored on conservation and closed forms, not on figures');
P(end+1) = struct('key','stereoNonlinear2d', 'listing','Listing 34.3', ...
    'entry','stereoNonlinear2d(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.stereoNonlinear2d('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('stereoNonlinear2d','Listing 34.3',E), ...
    'chapter','Ch.34 two-dimensional NONLINEAR interceptor/IRBM engagement -- round Earth, Newtonian gravity, impulsively air-launched KKV with APN guidance against a boosting 2000-km IRBM; scores lateral divert, the two Fig. 34.9 acceleration peaks and the required launch speed.  FEET, g = 32.2, divert reported in m/s via the listing''s own DELV/3.28');
P(end+1) = struct('key','stereoTrackingMC', 'listing','Listing 34.4', ...
    'entry','stereoTrackingMC(''benchmark'',true,''assert'',false,''render'',false)', ...
    'call', @() mgs.zarchan.stereoTrackingMC('benchmark',true,'assert',false,'render',false), ...
    'adapt', @(E) adaptGeneric('stereoTrackingMC','Listing 34.4',E), ...
    'chapter','Ch.34 two-dimensional MONTE CARLO engagement using STEREO TRACKING -- the book''s largest listing (205 code lines).  Almost every output of it lives ONLY on Figs. 34.21-34.27, so what is scored is the prose case setup and the stereo triangulation/partials ALGEBRA; the 500 m/s divert claim is a difference between two figure curves and is emitted as a SKIP, not absorbed by a wide band');

% Ports that FAILED or could not be verified in audit.  EMPTY TODAY, and the
% renderer prints that fact rather than printing nothing -- "no exclusions"
% and "the exclusion block was forgotten" must not look identical.
%
% To exclude a port later: move its entry out of P and add a row here with a
% concrete reason and what breaks if you trust it anyway.  Do NOT simply
% delete it from P; an absent port and a sound port look the same from
% outside, which is how a suite quietly stops covering something.
X = struct('key',{},'listing',{},'why',{},'breaks',{});

% ── ⚠️⚠️ RUNS BUT WAS NEVER ADVERSARIALLY AUDITED ──────────────────────
%
% A third category, and it is not a bookkeeping convenience.  Excluding a port
% costs its rows; counting it as audited is false.  Neither was acceptable for
% shaping2d, which is the closest listing in the whole book to the problem this
% project actually has (stationary target, terminal flight-path-angle
% constraint, launch heading error, metric, perpendicular-to-velocity option).
%
% The audit is the check that has caught FABRICATED BOOK VALUES on every wave
% that ran one -- three on wave 1 alone, including a "28 g" that appears nowhere
% in the book and an "18 g" lifted out of a different chapter. A port that
% skipped it is not equivalent to one that passed it, and the difference must be
% visible on the board rather than living in a commit message.
%
% ⇒ To clear an entry: run the audit, then delete the row and bump AUDITED_TOTAL.
%
% EMPTY AS OF 2026-08-24.  shaping2d's audit finally ran and came back HONEST:
% the auditor had MATLAB, re-grepped all fourteen SRC_ constants against
% zarchan.txt independently, re-derived the bias-sign erratum algebraically,
% re-ran the three OCR delete experiments, and found NO fabricated value and
% none lifted from another chapter.  It is counted in AUDITED_TOTAL now.
%
% ⚠️ Four defects came out of that audit.  None moves a scored verdict, so
% none is grounds to exclude the port -- but three of them are wrong TEXT
% sitting next to right numbers, and the fourth is a live SCHEMA GAP:
%   D1  a `detail` string on a scored row cites zarchan.txt:8338 for a claim
%       about Listing 24.2's BETAD line.  :8338 says nothing about BETAD; the
%       real line is 8372-8374, and there BETAD is already guarded
%       (`if VT==0. BETAD=0.; else ...`), so "it divides by zero for a
%       stationary target" is not why it was dropped.  Row verdict unaffected.
%   D2  the Fig. 30.11 INFO row quotes a "-5..1 g" printed ordinate.  Fig.
%       30.11's ticks (:11153-11157) run -3..1; -5..1 is Fig. 30.7's.  The
%       argument survives because the minor division is 1 g either way.
%   D3  an OCR-comparison comment justifies "the clamp is inert at 10 g" with
%       "the logged peak demand is under 3 g".  Measured peaks are 0.688 /
%       2.698 / 2.805 / 5.040 g -- the TBEG=10 case is 5.04 g.  The conclusion
%       still holds and the code tests `< c.xnclimg`, so only the comment lies.
% ⭐ D4  CLOSED 2026-08-24, and it was worth the trouble it caused.
%       WHAT IT WAS: shaping2d's structuralCheck built `names` and `oks` as
%       LOCALS and never assigned K.names / K.oks, so adaptGeneric returned
%       early and the port contributed ZERO structural rows -- all ten of its
%       invariants, including the four DELETE EXPERIMENTS its header presents
%       as its central evidence, were invisible.  shapingNonlinear was the
%       same, and its own `names` list additionally omitted clamp_live_ok and
%       inverted_ok, so even its aggregate K.ok never saw the delete
%       experiment.  Running a delete experiment nobody can see is the same as
%       not running it.
%       FIXED ON BOTH SIDES:
%         * the PORTS now assign K.names / K.oks (shapingNonlinear gained the
%           two missing checks and a label list, asserted equal in length);
%           each contributes TEN structural rows, verified on the board, and a
%           hand-flipped `oks` entry was confirmed to turn a row red.
%         * ⭐⭐ adaptGeneric no longer FAILS SILENTLY.  Its docstring always
%           claimed to be "deliberately STRICT ... rather than being quietly
%           accommodated", but this one branch did the opposite.  It now
%           distinguishes three cases and only the first is quiet:
%             no `selfcheck` field  -> the port declares it has none.  Quiet.
%             schema mismatch       -> RED row naming the fields it did find.
%             names/oks length gap  -> RED row.
%           So the next port that computes invariants without publishing them
%           turns the board red instead of losing them.
%
% ONE LEAD, not a defect: the port reads extracted figure AXIS TICKS as
% evidence three times, always on the acceleration ordinates and never on the
% time or altitude axes -- which is where they disagree.  Its optimal /
% perpendicular-velocity cases sit inside the book's axes; its BIASED-PN and
% perpendicular-LOS cases overrun them by 7-16 percent (92.92 s against an
% 0..80 s axis, 63.96 s against 0..60 s, a 5.466 km apogee against a 5 km
% axis).  Axes can clip and nothing published contradicts the port, but do not
% treat its biased-PN MAGNITUDES as book-validated until someone looks at that.
UNAUDITED = struct('key',{},'why',{},'exposure',{});

if numel(P) + numel(X) ~= AUDITED_TOTAL + numel(UNAUDITED)
    error('mgs:zarchan:benchSuite:manifest', ...
        ['manifest accounts for %d ports (%d wired + %d excluded) but %d have ' ...
         'been audited and %d are wired-but-unaudited.  A port was added or ' ...
         'removed without updating this file, so the board is no longer the ' ...
         'whole board.'], ...
        numel(P)+numel(X), numel(P), numel(X), AUDITED_TOTAL, numel(UNAUDITED));
end

allKeys = {P.key};
if numel(unique(allKeys)) ~= numel(allKeys)
    error('mgs:zarchan:benchSuite:dupKey', ...
        'duplicate port key in the manifest; rows could not be attributed.');
end

% Subset selection.  A key that matches nothing is an ERROR, not an empty run:
% a typo that silently checks zero benchmarks is exactly the green-light-for-
% nothing outcome this suite exists to prevent.
sel = 1:numel(P);
if ~isempty(opts.only)
    for i = 1:numel(opts.only)
        if ~(ischar(opts.only{i}) || isstring(opts.only{i}))
            error('mgs:zarchan:benchSuite:only', ...
                '''only'' takes a cell array of port keys; element %d is a %s.', ...
                i, class(opts.only{i}));
        end
    end
    want = cellfun(@char, opts.only, 'UniformOutput', false);
    bad  = want(~ismember(want, allKeys));
    if ~isempty(bad)
        error('mgs:zarchan:benchSuite:onlyKey', ...
            'unknown port key(s): %s.  Valid keys: %s.', ...
            strjoin(bad, ', '), strjoin(allKeys, ', '));
    end
    sel = find(ismember(allKeys, want));
end

here = fileparts(mfilename('fullpath'));

% ---------------------------------------------------------------------------
%   RUN
% ---------------------------------------------------------------------------
rows = emptyRows();
ports = struct('key',{},'listing',{},'entry',{},'chapter',{},'elapsed_s',{}, ...
               'ok',{},'err',{},'stdout',{},'n_rows',{});

for ii = 1:numel(sel)
    p = P(sel(ii));
    t0 = tic;
    errmsg = '';
    txt = '';
    n0 = numel(rows);

    f = fullfile(here, [p.key '.m']);
    if ~isfile(f)
        % A missing file is a FAIL ROW, not an error: the rest of the board is
        % still worth seeing, and "the port is not there" is itself a result.
        rows(end+1) = mkrow(p.key, p.listing, 'PORT FILE MISSING', ...
            'n/a', 'n/a', '--', 'FAIL', false, false, true, 'n/a', 'n/a', ...
            sprintf('expected %s', f)); %#ok<AGROW>
    else
        try
            [res, txt] = quiet(p.call);
            rows = [rows, p.adapt(res)]; %#ok<AGROW>
        catch ME
            % Two of these ports ERROR BY DESIGN when a benchmark fails
            % (adjointSingleLag on either of its checks, adjointBinomial5 on
            % a structural invariant).  That is correct behaviour for them and
            % must not take the rest of the board down with it.
            errmsg = ME.message;
            rows(end+1) = mkrow(p.key, p.listing, 'PORT RAISED AN ERROR', ...
                'suite completes', ME.identifier, '--', 'FAIL', ...
                false, false, true, 'n/a', 'n/a', oneLine(ME.message)); %#ok<AGROW>
        end
    end

    % A port that produced no EVALUATED row has checked nothing, and nothing
    % must never be counted as success.  Same rule adjointBinomial5 applies to
    % itself when no benchmark is within reach of the parameters it was given.
    added = rows(n0+1:end);
    if isempty(added) || ~any([added.scored] & ~[added.skipped])
        rows(end+1) = mkrow(p.key, p.listing, 'NO BENCHMARK EVALUATED', ...
            '>= 1 row', '0 rows', '--', 'FAIL', false, false, true, 'n/a', ...
            'n/a', ['the port ran but adjudicated nothing -- check that its ' ...
                    'benchmark flag is still honoured']); %#ok<AGROW>
    elseif ~any([added.scored] & ~[added.skipped] & ...
                ~strcmp({added.source}, STRUCTSRC()))
        % THE STRONGER FORM OF THE SAME RULE, and the hole the third wave
        % opened.  Structural rows are SCORED, and eleven of them arrive with
        % engage3d alone -- so a port whose book rows were all gated out by
        % off-book parameters would still satisfy the test above on the
        % strength of its own self-consistency.  "The port agrees with itself"
        % is a weaker claim than "the port reproduces Zarchan", and this board
        % is only ever quoted as the second one.  At the manifest's own
        % parameters this never fires; it fires when someone changes them.
        rows(end+1) = mkrow(p.key, p.listing, 'NO BOOK VALUE ADJUDICATED', ...
            '>= 1 book row', 'structural rows only', '--', 'FAIL', false, ...
            false, true, 'n/a', 'n/a', ['every row that cites the book was ' ...
                    'skipped or gated; only self-consistency was ' ...
                    'checked']); %#ok<AGROW>
    end

    ports(end+1) = struct('key',p.key, 'listing',p.listing, 'entry',p.entry, ...
        'chapter',p.chapter, 'elapsed_s',toc(t0), 'ok',isempty(errmsg), ...
        'err',errmsg, 'stdout',txt, 'n_rows',numel(rows)-n0); %#ok<AGROW>
end

% ---------------------------------------------------------------------------
%   TALLY
% ---------------------------------------------------------------------------
sc = [rows.scored];  okv = [rows.ok];  sk = [rows.skipped];
S = struct();
S.n_rows     = numel(rows);
S.n_scored   = sum(sc & ~sk);
S.n_pass     = sum(sc & okv & ~sk);
S.n_fail     = sum(sc & ~okv & ~sk);
S.n_skip     = sum(sk);
S.n_info     = sum(strcmp({rows.verdict}, 'INFO'));
S.n_refuted  = sum(strcmp({rows.verdict}, 'REFUTED'));
S.n_unmapped = sum(strcmp({rows.source}, UNMAPPED()));
S.ports      = ports;
S.excluded   = X;
S.audited_total = AUDITED_TOTAL;
S.units      = 'FEET: ft, ft/s, ft/s^2, g = 32.2.  Nothing converted to SI.';
nfail        = S.n_fail;

S.summary = render(rows, S, ports, X, opts);
if opts.render
    fprintf('%s\n', S.summary);
    if opts.echo
        for i = 1:numel(ports)
            fprintf('\n---- %s stdout %s\n%s\n', ports(i).key, ...
                repmat('-', 1, max(0, 60-numel(ports(i).key))), ports(i).stdout);
        end
    end
end

if opts.strict && nfail > 0
    error('mgs:zarchan:benchSuite:failed', ...
        ['%d of %d scored Zarchan benchmarks FAILED.  The full board is above ' ...
         '-- read it before touching a tolerance.  These compare against ' ...
         'published values and, in several places, against closed-form ' ...
         'solutions of the very ODE being integrated, so a red row means the ' ...
         'port is wrong, not that the band was tight.'], nfail, S.n_scored);
end
end


% =============================================================================
%   ADAPTERS.  One per port.  Each takes that port's own result struct and
%   returns rows.  None of them recomputes a book value.
% =============================================================================

function R = adaptEngage2d(B)
%ADAPTENGAGE2D  Listing 2.1.  The port owns both the numbers and the bands,
%   and its book column is prose ("~12 g", "0 g", "yes"), so the delta column
%   here reads "band": the verdict comes from the port's own inclusive band
%   and there is no numeric book value to subtract.  The scalars themselves
%   are in B.cases (xnc0_g, tf_s, miss_ft, xnc_max_g) if you need arithmetic.
R = emptyRows();
if ~isfield(B, 'checks')
    error('mgs:zarchan:benchSuite:engage2dMode', ...
        ['engage2d returned a single-run struct, not a benchmark struct. ' ...
         'The ''Bench'',true flag is the ONLY switch into the benchmark ' ...
         'path -- a bare call runs one nominal engagement and checks nothing.']);
end
for i = 1:numel(B.checks)
    k = B.checks(i);
    caseLetter = k.cs(1);                       % 'A', 'B' or 'C'
    name = sprintf('%s | %s', strtrim(k.cs), k.q);
    src  = cite('engage2d', sprintf('%c|%s', caseLetter, k.q));
    switch k.v
    case 'PASS'
        R(end+1) = mkrow('engage2d','Listing 2.1',name,k.book,k.comp,'band', ...
            'PASS', true, false, true, src, k.band, ''); %#ok<AGROW>
    case 'FAIL'
        R(end+1) = mkrow('engage2d','Listing 2.1',name,k.book,k.comp,'band', ...
            'FAIL', false, false, true, src, k.band, ''); %#ok<AGROW>
    otherwise
        % REFUTED rows are the two briefed values this port disproved.  They
        % are carried so the disagreement stays visible, and scored = false so
        % that they can move the pass count in neither direction.
        R(end+1) = mkrow('engage2d','Listing 2.1',name,k.book,k.comp,'--', ...
            'REFUTED', false, false, false, src, k.band, ...
            'briefed value disproved by the port; see its header'); %#ok<AGROW>
    end
end
end


function R = adaptEngageLinear(L)
%ADAPTENGAGELINEAR  Listing 2.2.  Its checks publish a BOUND, not a value: A
%   and B compare against closed-form solutions of the same ODE the RK2 loop
%   integrates (ideal residual 0), C is a delete-experiment proving VC cancels,
%   and D is the Fig. 2.7/2.8 comparison against the nonlinear listing.  So the
%   book column carries the bound and the delta column reads "bound".
%
%   D RUNS ENGAGE2D.  If engage2d is broken the D rows here go red as well --
%   that is one fault showing twice, not two independent faults.
R = emptyRows();
if ~isfield(L, 'checks')
    error('mgs:zarchan:benchSuite:engageLinearMode', ...
        'engageLinear returned no checks struct; the ''bench'' flag was not honoured.');
end
for i = 1:numel(L.checks)
    c = L.checks(i);
    src = cite('engageLinear', c.name);
    R(end+1) = mkrow('engageLinear','Listing 2.2', c.name, c.want, ...
        sprintf('%.6g', c.value), 'bound', vstr(c.ok), logical(c.ok), ...
        false, true, src, c.want, ''); %#ok<AGROW>
end
if isempty(L.nl)
    R(end+1) = mkrow('engageLinear','Listing 2.2', ...
        'D: nonlinear cross-check SKIPPED', 'Figs 2.7/2.8', 'not run', '--', ...
        'SKIP', false, true, true, 'Fig. 2.7 / Fig. 2.8', 'n/a', ...
        ['A/B/C alone show only that the file integrates its own equations; ' ...
         'Figs 2.7/2.8 are COMPARISONS and were not made']);
end
end


function R = adaptAdjointSingleLag(A)
%ADAPTADJOINTSINGLELAG  Listing 3.1.  This port ERRORS on a failed check
%   instead of returning a red flag, so if we are here both of its checks
%   passed; these rows exist to put the NUMBERS on the board, not just a tick.
R = emptyRows();
C = A.check;

% Fig. 3.13, the headline: "a 3-g maneuver causes nearly 12 ft of miss if the
% flight time is only 1 s".  Book value and tolerance both come from the port.
if C.fig313_applicable
    R(end+1) = mkrow('adjointSingleLag','Listing 3.1', ...
        'miss at tF = 1 s, 3-g step', sprintf('%.4g ft', C.fig313_book), ...
        sprintf('%.4f ft', C.fig313_adj), ...
        deltaStr(C.fig313_book, C.fig313_adj), vstr(C.fig313_pass), ...
        logical(C.fig313_pass), false, true, ...
        cite('adjointSingleLag','fig313'), sprintf('+/-%.1f ft', C.fig313_tol), ...
        sprintf('read at t = %.5f s (Zarchan starts TP at +1e-5)', C.t_tf1));
else
    R(end+1) = mkrow('adjointSingleLag','Listing 3.1', ...
        'miss at tF = 1 s, 3-g step', '~12 ft', 'not applicable', '--', ...
        'SKIP', false, true, true, cite('adjointSingleLag','fig313'), 'n/a', ...
        'parameters are off the book''s nominal case, so Fig. 3.13 does not describe them');
end

% The second opinion: an analytic inverse transform sharing NO CODE with the
% integrator.  This is the check with real teeth -- fault injection in the port
% put every structural break at 100%-107,000% of peak, against an honest
% discretization residual of 1.9%.
if C.cf_available
    R(end+1) = mkrow('adjointSingleLag','Listing 3.1', ...
        'adjoint vs p.55 closed form', '0 (identical)', ...
        sprintf('%.4g of peak', C.cf_resid), deltaStr(0, C.cf_resid), ...
        vstr(C.cf_pass), logical(C.cf_pass), false, true, ...
        cite('adjointSingleLag','closedform'), sprintf('<= %.4g', C.cf_tol), ...
        'analytic inverse transform; shares no code with the RK2 loop');
else
    R(end+1) = mkrow('adjointSingleLag','Listing 3.1', ...
        'adjoint vs p.55 closed form', '0 (identical)', 'not evaluable', '--', ...
        'SKIP', false, true, true, cite('adjointSingleLag','closedform'), 'n/a', ...
        'non-integer N'': the binomial is an infinite series, so there is no closed form to compare');
end

% Reported, not scored.  The heading-error channel is the one that maps onto
% our own measured discriminant (release-side |eps0|), so its peak is worth
% having on the board even though the port gates only the maneuver channel.
[pk, ipk] = max(abs(A.miss_he_ft));
R(end+1) = mkrow('adjointSingleLag','Listing 3.1', ...
    'heading-error channel peak', 'shape only', ...
    sprintf('%.2f ft @ tF=%.1f s', pk, A.t(ipk)), '--', 'INFO', ...
    false, false, false, cite('adjointSingleLag','he'), 'not gated', ...
    'the book publishes a shape, not a value; the closed form is in A.cf_he_ft');
end


function R = adaptAdjointBinomial5(Q)
%ADAPTADJOINTBINOMIAL5  Listing 6.1.  Two tolerance classes, both the port's:
%   'graph' 15% for the seven values Zarchan read off Figs 6.6-6.8 by eye, and
%   'text' 1% for the optimal-evasive coefficients he printed to three figures
%   and then did arithmetic with.  The text rows are the real test of the port.
R = emptyRows();
if ~isfield(Q, 'bench')
    error('mgs:zarchan:benchSuite:binom5Mode', ...
        'adjointBinomial5 returned no bench struct.');
end
for i = 1:numel(Q.bench)
    b = Q.bench(i);
    src  = cite('adjointBinomial5', b.what);
    band = sprintf('%s %.0f%%', b.band, 100*b.tol);
    if b.skipped
        R(end+1) = mkrow('adjointBinomial5','Listing 6.1', b.what, ...
            sprintf('%.4g', b.book), 'skipped', '--', 'SKIP', false, true, ...
            true, src, band, b.why); %#ok<AGROW>
    else
        R(end+1) = mkrow('adjointBinomial5','Listing 6.1', b.what, ...
            sprintf('%.4g', b.book), sprintf('%.4f', b.ours), ...
            deltaStr(b.book, b.ours), vstr(b.ok), logical(b.ok), false, true, ...
            src, band, sprintf('%.4g ft vs book %.4g ft at the worked example', ...
            b.miss_ours, b.miss_book)); %#ok<AGROW>
    end
end

% The structural invariants.  NOT book values, and labelled as such -- but
% they are the only checks in that port that still bite off-book, and the
% open-loop one pins the integrator against analytic truth with no figure
% reading anywhere in it.  The port ERRORS if any of them fails, so reaching
% this code at all means they held; the rows put the residuals on the board.
if isfield(Q, 'selfcheck') && isfield(Q.selfcheck, 'ok')
    C = Q.selfcheck;
    R(end+1) = mkrow('adjointBinomial5','Listing 6.1', ...
        'structural: open loop vs tau^n', '0 (analytic)', ...
        sprintf('%.3e rel', C.openloop_rel), deltaStr(0, C.openloop_rel), ...
        vstr(C.openloop_ok), logical(C.openloop_ok), false, true, ...
        cite('adjointBinomial5','sc_openloop'), sprintf('<= %.1e', C.openloop_tol), ...
        'N''=0 cuts the loop, so x1/x9/x10 must be tau^2/2, tau^3/6, tau^4/24');
    R(end+1) = mkrow('adjointBinomial5','Listing 6.1', ...
        'structural: Vc cancels', '0 (algebraic)', ...
        sprintf('%.3e rel', C.vc_rel), deltaStr(0, C.vc_rel), ...
        vstr(C.vc_ok), logical(C.vc_ok), false, true, ...
        cite('adjointBinomial5','sc_vc'), sprintf('<= %.1e', C.vc_tol), ...
        're-run at 7x Vc; a dropped multiply or divide would move the curves');
    R(end+1) = mkrow('adjointBinomial5','Listing 6.1', ...
        'structural: T^n normalization', '0 (by construction)', ...
        sprintf('%.3e rel', C.tau_rel), deltaStr(0, C.tau_rel), ...
        vstr(C.tau_ok), logical(C.tau_ok), false, true, ...
        cite('adjointBinomial5','sc_tau'), sprintf('<= %.1e', C.tau_tol), ...
        'the ONLY check that can see a wrong power of T: the default run has tau = 1');
end
end


function R = adaptWeave(W)
%ADAPTWEAVE  Listing 20.3.  THIS PORT DOES NOT ADJUDICATE.  Its bench struct
%   carries computed and book but no ok field, so the bands below are the only
%   ones there are and they live here.  Provenance of the +/-5%: the BOOK side,
%   not ours -- 20/60/70/75/85 are read off Figs 20.9/20.10 to one significant
%   figure, while our own step-size sensitivity at those points is 0.02-0.15%.
%
%   The two N'=5 rows are the configuration the commissioning brief cited.  The
%   book publishes NO worked number for Fig. 20.12, so they are carried as INFO
%   and scored by nobody.  The brief's "60 -> 70" is the N'=3 ladder.
BAND = 0.05;
R = emptyRows();
if isempty(W.bench)
    error('mgs:zarchan:benchSuite:weaveMode', ...
        ['weaveDesignCurve returned W.bench = [], so the ''benchmark'',true ' ...
         'flag was not honoured.  A bare call runs the sweep and checks nothing.']);
end
for i = 1:numel(W.bench)
    b = W.bench(i);
    src = cite('weaveDesignCurve', b.label);
    if b.is_book
        ok = isfinite(b.computed) && abs(b.computed/b.book - 1) <= BAND;
        R(end+1) = mkrow('weaveDesignCurve','Listing 20.3', b.label, ...
            sprintf('%.4g', b.book), sprintf('%.4f', b.computed), ...
            deltaStr(b.book, b.computed), vstr(ok), ok, false, true, src, ...
            sprintf('+/-%.0f%% (set here)', 100*BAND), ...
            'the port computes this row but publishes no verdict'); %#ok<AGROW>
    else
        R(end+1) = mkrow('weaveDesignCurve','Listing 20.3', b.label, ...
            'none published', sprintf('%.4f', b.computed), '--', 'INFO', ...
            false, false, false, src, 'not scored', ...
            'the brief cited this configuration; Fig. 20.12 carries no worked number'); %#ok<AGROW>
    end
end

% Ch.20 text values that the port computes but does not put in its bench
% struct.  The crossover is the sharpest single check in the whole file: it is
% a CROSSING, so it is immune to an overall scale error in the curve and
% extremely sensitive to the hidden g in the coast reference (no_guidance =
% XNT/x^2, not 1/x^2).  Get that factor wrong and PN never beats coasting,
% which inverts the chapter's headline claim.
okPk = abs(W.peak/63.0 - 1) <= BAND;
R(end+1) = mkrow('weaveDesignCurve','Listing 20.3', 'curve max, N''=3 unlimited', ...
    '63', sprintf('%.4f', W.peak), deltaStr(63.0, W.peak), vstr(okPk), okPk, ...
    false, true, cite('weaveDesignCurve','peak'), '+/-5% (set here)', '');

okXp = abs(W.x_peak - 0.7) <= 0.15;
R(end+1) = mkrow('weaveDesignCurve','Listing 20.3', 'x at curve max', ...
    '0.7', sprintf('%.2f', W.x_peak), deltaStr(0.7, W.x_peak), vstr(okXp), ...
    okXp, false, true, cite('weaveDesignCurve','xpeak'), '+/-0.15 in x (here)', ...
    'the curve is flat over x = 0.7-0.9, so the argmax can legitimately land one grid step off');

xc   = W.x_pn_beats_coast_upto;
okXc = isfinite(xc) && xc >= 0.7 && xc <= 0.8;
R(end+1) = mkrow('weaveDesignCurve','Listing 20.3', 'PN beats coasting up to x', ...
    '0.7', sprintf('%.2f', xc), deltaStr(0.7, xc), vstr(okXc), okXc, false, ...
    true, cite('weaveDesignCurve','crossover'), '0.7 .. 0.8 (set here)', ...
    'a crossing, not a level: insensitive to curve scale, sensitive to the g in the coast reference');

% The coast reference denormalized to the book's own 6-g / T = 0.5 s example.
% Analytic on both sides, hence 1% rather than 5%.
R = addCoastRow(R, W, 1.0, 48.4, cite('weaveDesignCurve','coast1'));
R = addCoastRow(R, W, 2.0, 12.1, cite('weaveDesignCurve','coast2'));
end


function R = addCoastRow(R, W, x, book, src)
%ADDCOASTROW  Coast displacement at one x, in feet, for nT = 6 g and T = 0.5 s.
%   EXACT grid match only.  Interpolating would invent a value the port never
%   computed and then score it against the book.
NT_G = 6.0;  T_S = 0.5;
i = find(abs(W.x - x) < 1e-9, 1);
name = sprintf('coast miss at x=%g (6 g, T=0.5 s)', x);
if isempty(i)
    R(end+1) = mkrow('weaveDesignCurve','Listing 20.3', name, ...
        sprintf('%.4g ft', book), 'x not on grid', '--', 'SKIP', false, true, ...
        true, src, '+/-1% (set here)', ...
        'the x grid does not contain this point and it will not be interpolated');
    return
end
ft = W.no_guidance(i) * NT_G * T_S^2;
ok = abs(ft/book - 1) <= 0.01;
R(end+1) = mkrow('weaveDesignCurve','Listing 20.3', name, ...
    sprintf('%.4g ft', book), sprintf('%.3f ft', ft), deltaStr(book, ft), ...
    vstr(ok), ok, false, true, src, '+/-1% (set here)', ...
    'analytic both sides; catches a wrong g in the normalization that the normalized rows cannot see');
end


% =============================================================================
%   SECOND-WAVE ADAPTERS.  Listings 5.2, 5.3, 8.1, 8.2 and 23.3.
%
%   All five ports carry a .source on every row that already names the figure
%   or page AND the zarchan.txt line, so these read it through selfCite() and
%   never through cite().  See the citation note in the file header.
%
%   All five also ERROR out of their own structural self-checks instead of
%   returning a flag, so reaching one of these adapters already means the
%   invariants held.  The structural rows they emit are therefore not a
%   verdict that could still change -- they exist to put the RESIDUAL on the
%   board, because the residual is the number that moves first when a
%   transcription breaks, long before a benchmark leaves its band.
%
%   NONE of these rows is a book value dressed up as one: the structural rows
%   print "0 (structural)" in the book column and say so again in the source
%   column, because a self-consistency residual quoted as "the port reproduces
%   the book" is a strictly weaker claim wearing a stronger claim's clothes.
% =============================================================================

function R = adaptCovarianceHoming(C)
%ADAPTCOVARIANCEHOMING  Listing 5.2, the homing-loop covariance (Riccati) run.
%   Its bench rows already carry .scored, so its two INFO rows come across
%   with no translation.  One of them matters: the terminal sigma_nL of
%   1.24e5 g is a pure function of Zarchan's eps_tgo fudge factor and NOT of
%   the flight (it scales exactly as 1/eps, which the port asserts in its
%   self-check 6).  It stays labelled ARTIFACT here for the same reason it is
%   labelled there -- so it cannot be lifted off a board as a result.
R = emptyRows();
if ~isfield(C, 'bench') || isempty(C.bench)
    error('mgs:zarchan:benchSuite:covarianceMode', ...
        ['covarianceHoming returned no bench rows.  Its benchmarks run on ' ...
         'every call, so an empty struct means the file changed shape.']);
end
for i = 1:numel(C.bench)
    b = C.bench(i);
    src = selfCite(b.source);
    if ~b.scored
        R(end+1) = mkrow('covarianceHoming','Listing 5.2', b.what, b.book_s, ...
            b.ours_s, '--', 'INFO', false, false, false, src, 'not scored', ...
            b.detail); %#ok<AGROW>
    elseif b.skipped
        R(end+1) = mkrow('covarianceHoming','Listing 5.2', b.what, b.book_s, ...
            'skipped', '--', 'SKIP', false, true, true, src, ...
            bandPct(b.band, b.tol), b.why); %#ok<AGROW>
    else
        % Shape rows carry NaN in .book/.ours and their want/got text in
        % .book_s/.ours_s, so deltaStr correctly prints '--' for them.
        R(end+1) = mkrow('covarianceHoming','Listing 5.2', b.what, b.book_s, ...
            b.ours_s, deltaStr(b.book, b.ours), vstr(b.ok), logical(b.ok), ...
            false, true, src, bandPct(b.band, b.tol), b.detail); %#ok<AGROW>
    end
end

if ~isfield(C, 'selfcheck') || ~isfield(C.selfcheck, 'ok')
    return
end
K = C.selfcheck;
S = STRUCTSRC();
R = addStructRow(R, 'covarianceHoming','Listing 5.2', ...
    'structural: shaping filter exact', K.shaping_rel, K.shaping_tol, ...
    K.shaping_ok, S, ...
    'X(3,3) == x0(3,3)+phis*t; sees PHIS = nT^2*tF (the lost fraction bar) as tF^2');
R = addStructRow(R, 'covarianceHoming','Listing 5.2', ...
    'structural: X symmetric bit for bit', K.symmetry_res, K.symmetry_tol, ...
    K.symmetry_ok, S, 'tolerance is ZERO on purpose: there is no honest small residual');
if K.psd_ntest == 0
    R(end+1) = mkrow('covarianceHoming','Listing 5.2', ...
        'structural: X PSD (final sample out)', '>= 0', 'not evaluable', '--', ...
        'SKIP', false, true, true, S, 'n/a', ...
        'fewer than two states carry variance, so there is nothing to normalise');
else
    R(end+1) = mkrow('covarianceHoming','Listing 5.2', ...
        'structural: X PSD (final sample out)', '>= 0', ...
        sprintf('%.4g min eig', K.psd_min), deltaStr(0, K.psd_min), ...
        vstr(K.psd_ok), logical(K.psd_ok), false, true, S, ...
        sprintf('>= %.1e', K.psd_tol), ...
        sprintf(['worst at t = %.2f s over %d samples.  THE FINAL SAMPLE IS ' ...
                 'EXCLUDED and is genuinely not a covariance matrix -- its ' ...
                 'correlation matrix has an eigenvalue of -2.99'], ...
                 K.psd_min_t, K.psd_ntest));
end
if K.vc_skipped
    R(end+1) = mkrow('covarianceHoming','Listing 5.2', 'structural: Vc cancels', ...
        '0 (algebraic)', 'skipped', '--', 'SKIP', false, true, true, S, 'n/a', ...
        'needs x0 = 0');
else
    R = addStructRow(R, 'covarianceHoming','Listing 5.2', ...
        'structural: Vc cancels (x7)', K.vc_rel, K.vc_tol, K.vc_ok, S, ...
        ['a model in which closing speed provably cancels has NO along-track ' ...
         'channel: this file yields miss dispersion, never arrival spread']);
end
if K.lin_skipped
    R(end+1) = mkrow('covarianceHoming','Listing 5.2', ...
        'structural: sigma linear in nT', '0 (algebraic)', 'skipped', '--', ...
        'SKIP', false, true, true, S, 'n/a', 'needs x0 = 0 and phis derived');
else
    R = addStructRow(R, 'covarianceHoming','Listing 5.2', ...
        'structural: sigma linear in nT (x2.5)', K.lin_rel, K.lin_tol, ...
        K.lin_ok, S, 'the loop is linear, so every sigma must scale with the drive');
end
if K.eps_skipped
    R(end+1) = mkrow('covarianceHoming','Listing 5.2', ...
        'structural: sigma_nL(tF) is 1/eps_tgo', 'x10', 'skipped', '--', ...
        'SKIP', false, true, true, S, 'n/a', 'eps_tgo is not small against h');
else
    R(end+1) = mkrow('covarianceHoming','Listing 5.2', ...
        'structural: sigma_nL(tF) is 1/eps_tgo', 'x10 per decade', ...
        sprintf('x%.4f', K.eps_ratio), deltaStr(10, K.eps_ratio), ...
        vstr(K.eps_ok), logical(K.eps_ok), false, true, S, ...
        sprintf('10 +/-%.0f%%', 100*K.eps_ratio_tol), ...
        sprintf(['ASSERTS that the terminal acceleration sigma is Zarchan''s ' ...
                 'fudge factor and not physics; sigma_y moved %.3f%% ' ...
                 '(reported, not scored)'], 100*K.eps_sigy_rel));
end
R = addStructRow(R, 'covarianceHoming','Listing 5.2', ...
    'structural: sigma_y(tF) converged in h', K.h_rel, K.h_tol, K.h_ok, S, ...
    're-run at h/2; this scheme is O(h) near the intercept, not O(h^4)');
end


function R = adaptAccelMissAdjoint(A)
%ADAPTACCELMISSADJOINT  Listing 5.3.  Deliberately runs the BOOK's parameters
%   for its benchmark table whatever it was called with, so these rows are
%   never a wall of SKIPs reading as green.
%
%   THIS PORT AND COVARIANCEHOMING ARE THE TWO HALVES OF THE BOOK'S OWN
%   CROSS-CHECK (":1548  Covariance analysis miss distance results agree with
%   adjoint").  They should agree to about 1.5%, NOT to machine precision:
%   the covariance side runs RK4 and lands on the analytic 13.2274 ft, this
%   one runs the book's RK2 against a 1/tgo singularity and lands on 13.4166.
%   Agreement to 1e-12 would mean one of them is not what it claims to be.
R = emptyRows();
if ~isfield(A, 'bench') || isempty(A.bench)
    error('mgs:zarchan:benchSuite:accelMode', ...
        ['accelMissAdjoint returned no bench rows, so the ''benchmark'' flag ' ...
         'was not honoured and this run adjudicated nothing.']);
end
for i = 1:numel(A.bench)
    b = A.bench(i);
    src = selfCite(b.source);
    if b.skipped
        R(end+1) = mkrow('accelMissAdjoint','Listing 5.3', b.what, ...
            sprintf('%.4g %s', b.book, b.unit), 'skipped', '--', 'SKIP', ...
            false, true, true, src, bandPct(b.band, b.tol), b.why); %#ok<AGROW>
    else
        R(end+1) = mkrow('accelMissAdjoint','Listing 5.3', b.what, ...
            sprintf('%.4g %s', b.book, b.unit), ...
            sprintf('%.4f %s', b.ours, b.unit), deltaStr(b.book, b.ours), ...
            vstr(b.ok), logical(b.ok), false, true, src, ...
            bandPct(b.band, b.tol), ...
            sprintf('read at t = %.5f s', b.t_read)); %#ok<AGROW>
    end
end

% The two PROSE claims.  These are SCORED, not INFO: the port adjudicates them
% and they gate its own R.pass.  An ordering has no tolerance to argue about
% and is often the sharper test -- neither could have been read off a figure,
% because no figure value survived the extraction at all.
if isfield(A, 'shape')
    for i = 1:numel(A.shape)
        sh = A.shape(i);
        % The CLAIM goes into .detail, not into the book column: it is a
        % sentence, and a sentence in a 15-character field pushes the rest of
        % that row's columns out of line with every other row on the board.
        det = strtrim(sprintf('%s -- %s', sh.claim, sh.detail));
        if sh.skipped
            R(end+1) = mkrow('accelMissAdjoint','Listing 5.3', ...
                ['shape: ' sh.what], 'prose ordering', 'not evaluable', '--', ...
                'SKIP', false, true, true, selfCite(sh.source), 'ordering', ...
                det); %#ok<AGROW>
        else
            R(end+1) = mkrow('accelMissAdjoint','Listing 5.3', ...
                ['shape: ' sh.what], 'prose ordering', holdStr(sh.ok), '--', ...
                vstr(sh.ok), logical(sh.ok), false, true, ...
                selfCite(sh.source), 'ordering, exact', det); %#ok<AGROW>
        end
    end
end

if ~isfield(A, 'selfcheck') || ~isfield(A.selfcheck, 'ok')
    return
end
K = A.selfcheck;
S = STRUCTSRC();
if K.cf_integer
    R = addStructRow(R, 'accelMissAdjoint','Listing 5.3', ...
        'structural: miss vs analytic closed form', K.cf_resid, K.cf_tol, ...
        K.cf_ok, S, ...
        ['the honest RK2 residual against the 1/tgo singularity; it grows as ' ...
         'about 0.48 percent times N'' times (H/0.01)']);
else
    R(end+1) = mkrow('accelMissAdjoint','Listing 5.3', ...
        'structural: miss vs analytic closed form', '0 (analytic)', 'skipped', ...
        '--', 'SKIP', false, true, true, S, 'n/a', ...
        'non-integer N'': a truncated infinite series is not a closed form');
end
if K.pub_run
    R = addStructRow(R, 'accelMissAdjoint','Listing 5.3', ...
        'structural: route IS the book''s :1355 form', K.pub_resid, ...
        K.pub_tol, K.pub_ok, S, ...
        ['the printed form is unstable below tF/T ~ 0.1 (3 minus something ' ...
         'tending to 3), so it identifies the stable route rather than being it']);
end
if K.scale_accel_run
    R = addStructRow(R, 'accelMissAdjoint','Listing 5.3', ...
        'structural: accel invariant under time scale', K.scale_accel_rel, ...
        K.scale_accel_tol, K.scale_accel_ok, S, ...
        sprintf(['x2 time scaling at tgo = %g s.  With the TAU^2 row, the ONLY ' ...
                 'check that can see a wrong TAU inside x3 = N''/(TAU*tgo)'], ...
                 K.scale_accel_tgo));
end
R = addStructRow(R, 'accelMissAdjoint','Listing 5.3', ...
    'structural: miss scales as TAU^2', K.scale_miss_rel, K.scale_miss_tol, ...
    K.scale_miss_ok, S, ...
    'the default run has TAU = 1, where every power of T looks identical');
end


function R = adaptSingleLagCh8(Q)
%ADAPTSINGLELAGCH8  Listing 8.1, PN vs augmented PN.
%
%   .band here is a PROVENANCE STRING, not a number ('closed form (BOOK)',
%   'closed form (DERIVED)', '+/-1 ft (read off a graph)', 'twin agreement'),
%   and .tol is the number.  BOTH are printed: dropping the string would lose
%   the one thing that separates the port's DERIVED closed form -- which the
%   book does not publish -- from the book's own.
%
%   TWO ROWS HERE CAN GO RED FOR A REASON THAT IS NOT THIS FILE'S FAULT.
%   'PN miss at tF = 1 s' is a Chapter 3 number (Fig. 3.13, "nearly 12 ft"),
%   admissible only because the port proves APN=0 IS the Listing 3.1 system;
%   and 'APN=0 column vs Listing 3.1 port' runs that other port, which ERRORS
%   by design on its own benchmark failures.  A red row in either can mean the
%   Ch.3 port broke.  Tell them apart with ('only', {'adjointSingleLag'}).
R = emptyRows();
if ~isfield(Q, 'bench') || isempty(Q.bench)
    error('mgs:zarchan:benchSuite:ch8Mode', ...
        ['adjointSingleLagCh8 returned no bench rows, so the ''benchmark'' ' ...
         'flag was not honoured and R.pass means "not checked".']);
end
for i = 1:numel(Q.bench)
    b = Q.bench(i);
    src  = selfCite(b.source);
    band = b.band;
    if isfinite(b.tol)
        band = sprintf('%s | %.4g', band, b.tol);
    end
    if b.skipped
        R(end+1) = mkrow('adjointSingleLagCh8','Listing 8.1', b.what, ...
            sprintf('%.6g %s', b.book, b.units), 'skipped', '--', 'SKIP', ...
            false, true, true, src, band, b.why); %#ok<AGROW>
    else
        R(end+1) = mkrow('adjointSingleLagCh8','Listing 8.1', b.what, ...
            sprintf('%.6g %s', b.book, b.units), ...
            sprintf('%.6g %s', b.ours, b.units), deltaStr(b.book, b.ours), ...
            vstr(b.ok), logical(b.ok), false, true, src, band, ...
            b.detail); %#ok<AGROW>
    end
end

% Reported, not scored.  R.verified is FALSE whenever the closed-form rows did
% not actually run -- a state in which the returned curves are unchecked but
% nothing in the pass/fail columns above would say so.
if isfield(Q, 'verified')
    R(end+1) = mkrow('adjointSingleLagCh8','Listing 8.1', ...
        'port''s own R.verified flag', 'true', boolStr(Q.verified), '--', ...
        'INFO', false, false, false, 'port state - NOT a book value', ...
        'not scored', ...
        ['false means the closed-form rows did not run and THE RETURNED CURVES ' ...
         'ARE UNCHECKED, even if every row above is green']);
end
end


function R = adaptAdjointOptimal(O)
%ADAPTADJOINTOPTIMAL  Listing 8.2 plus the Listing A.1 (Appendix) extension.
%
%   BANDS HERE ARE ABSOLUTE, in each row's own units -- 'kernel', 'graph' and
%   'xport' rows include values near zero where a relative band is meaningless,
%   'ratio' rows are already fractions of the PN peak, and 'order' rows are
%   booleans (book = 1 means "the book asserts this direction holds").
%   bandAbs() prints them as absolute for that reason; using bandPct() here
%   would silently mis-state every tolerance in this port.
%
%   THE TWO A.1 ROWS ARE WHY THIS PORT IS ON THE BOARD.  Optimal guidance is
%   the first ported listing that COMPENSATES for lag instead of pricing it,
%   and the book's own appendix measures how brittle that is in time-to-go: a
%   0.9 scale-factor error already loses to plain PN past 8 s of flight.
R = emptyRows();
if ~isfield(O, 'bench') || isempty(O.bench)
    error('mgs:zarchan:benchSuite:optimalMode', ...
        'adjointOptimal returned no bench rows.');
end
for i = 1:numel(O.bench)
    b = O.bench(i);
    src  = selfCite(b.source);
    band = bandAbs(b.band, b.tol, b.units);
    if b.skipped
        R(end+1) = mkrow('adjointOptimal','Listing 8.2 (+A.1)', b.what, ...
            sprintf('%.4g %s', b.book, b.units), 'skipped', '--', 'SKIP', ...
            false, true, true, src, band, b.why); %#ok<AGROW>
    elseif strcmpi(b.band, 'order')
        % Boolean rows: the book publishes a DIRECTION, not a value, so there
        % is nothing to subtract and the delta column says so.
        R(end+1) = mkrow('adjointOptimal','Listing 8.2 (+A.1)', b.what, ...
            'holds', holdStr(isfinite(b.ours) && b.ours == b.book), ...
            'ordering', vstr(b.ok), logical(b.ok), false, true, src, ...
            'exact', b.detail); %#ok<AGROW>
    else
        R(end+1) = mkrow('adjointOptimal','Listing 8.2 (+A.1)', b.what, ...
            sprintf('%.4g %s', b.book, b.units), ...
            sprintf('%.6g %s', b.ours, b.units), deltaStr(b.book, b.ours), ...
            vstr(b.ok), logical(b.ok), false, true, src, band, ...
            b.detail); %#ok<AGROW>
    end
end

if ~isfield(O, 'selfcheck') || ~isfield(O.selfcheck, 'ok')
    return
end
K = O.selfcheck;
S = STRUCTSRC();
R = addStructRow(R, 'adjointOptimal','Listing 8.2 (+A.1)', ...
    'structural: open loop vs t^2/2', K.openloop_rel, K.openloop_tol, ...
    K.openloop_ok, S, ...
    'XNP = 0 cuts every gain, and Heun is EXACT on the integrand that is left');
if K.c3_evaluated
    R = addStructRow(R, 'adjointOptimal','Listing 8.2 (+A.1)', ...
        'structural: C3 confined to x1dot', K.c3_rel, K.c3_tol, K.c3_ok, S, ...
        ['APN=0 and APN=1 must give BIT-IDENTICAL x2/x3/x4.  Catches C3 wired ' ...
         'into the wrong derivative, a fault that leaves both curves smooth']);
else
    R(end+1) = mkrow('adjointOptimal','Listing 8.2 (+A.1)', ...
        'structural: C3 confined to x1dot', '0 (bit-identical)', 'skipped', ...
        '--', 'SKIP', false, true, true, S, 'n/a', ...
        'needs both APN = 0 and APN = 1 in the run');
end
R = addStructRow(R, 'adjointOptimal','Listing 8.2 (+A.1)', ...
    'structural: A.1 inert at SF=1, BIAS=0', K.a1_rel, K.a1_tol, K.a1_ok, S, ...
    ['compared against a SECOND, un-extended transcription of Listing 8.2 kept ' ...
     'as an oracle: the only evidence this file still ports 8.2 and not only A.1']);
end


function R = adaptAutopilot(AP)
%ADAPTAUTOPILOT  Listing 23.3, the homing loop with a detailed flight-control
%   system.  ~59 rows.  Bands are RELATIVE and encode provenance: table 1%
%   (a three-figure table entry), text 2% (prose, or recomputed by the author
%   from already-rounded table values), approx 5% ("approximately 35 rad/s"),
%   graph 15% ("nearly 600 deg/s"), plus shape/bound rows with no tolerance.
%
%   INFO rows are NOT skips here and must not be counted as either.  The port
%   made two values INFO on purpose: the 25 g sizing statement (read off a
%   curve that diverges as tgo -> 0, so scoring it would score a grid choice
%   rather than the port) and "at sea level only a few deg/s of fin rate are
%   required", where the port computes 17.3 deg/s and DISAGREES WITH THE
%   ADJECTIVE instead of quietly widening "a few" to include 17.
%
%   THE ROW LOOP LIVES IN BENCHROWS() NOW.  It was here, and it was copied
%   verbatim by the five third-wave adapters, which is one copy per port of
%   the INFO/SKIP/value-skipped distinction -- five places for it to drift
%   apart while every one of them keeps printing a full-looking board.  The
%   logic is unchanged; only its address is.
if ~isfield(AP, 'bench') || isempty(AP.bench)
    error('mgs:zarchan:benchSuite:autopilotMode', ...
        ['adjointWithAutopilot returned no bench rows, so ''benchmark'' was ' ...
         'false; its R.pass is then false meaning "not checked", not "fine".']);
end
R = benchRows('adjointWithAutopilot', 'Listing 23.3', AP.bench);

if ~isfield(AP, 'selfcheck') || ~isfield(AP.selfcheck, 'ok')
    return
end
K = AP.selfcheck;
S = STRUCTSRC();
R = addStructRow(R, 'adjointWithAutopilot','Listing 23.3', ...
    'structural: radome gated by R', K.radome_zero_resid, K.radome_zero_tol, ...
    K.radome_zero_ok, S, ...
    'at R = 0 the two radome states must be equal EXACTLY; the tolerance is zero');
R(end+1) = mkrow('adjointWithAutopilot','Listing 23.3', ...
    'structural: radome path alive', '>= 2x', ...
    sprintf('x%.4g', K.radome_alive_ratio), '--', vstr(K.radome_alive_ok), ...
    logical(K.radome_alive_ok), false, true, S, ...
    sprintf('>= %.3g', K.radome_alive_tol), ...
    'a delete-experiment: the row above would also pass if R did nothing at all');
R(end+1) = mkrow('adjointWithAutopilot','Listing 23.3', ...
    'structural: measured t63 vs closed form', ...
    sprintf('%s s form', numList(K.lag_form)), ...
    sprintf('%s s meas', numList(K.lag_meas)), deltaStr(0, K.lag_rel), ...
    vstr(K.lag_ok), logical(K.lag_ok), false, true, S, ...
    sprintf('<= %.3g rel', K.lag_tol), ...
    ['THE LAG IS DERIVED, NOT FITTED -- and this is the only row that ' ...
     'exercises KR and wI at all']);
R = addStructRow(R, 'adjointWithAutopilot','Listing 23.3', ...
    'structural: unity closed-loop DC gain', K.dc_err, K.dc_tol, K.dc_ok, S, ...
    ['an algebraic IDENTITY, not a design goal: exercises the whole chain ' ...
     'K1 -> K3 -> KC -> KA -> KDC in one number, at 1e-9']);
end


% =============================================================================
%   THIRD WAVE ADAPTERS.  Listings 21.1, 21.2, 22.4, 23.1, 28.1.
%
%   All five carry a .source on EVERY row and use the same fifteen-field bench
%   schema as adjointWithAutopilot, so they share BENCHROWS() below instead of
%   holding five more copies of the same switch.  Their structural self-checks
%   share nothing -- those are per-port and are appended by hand under each
%   adapter, because a structural row that does not name the specific invariant
%   it tests is decoration.
%
%   WHY THE STRUCTURAL ROWS ARE NOT OPTIONAL HERE.  Three of these five ports
%   (airframeLinear, flightControlDelay, engage3d) do NOT throw on a failed
%   self-check when 'assert' is false -- which is exactly how this suite calls
%   them, because a throwing port takes its own board down with it.  If the
%   adapter did not surface the self-check, a damaged transcription could
%   reproduce every published number by accident and this board would print it
%   green.  threeLoopAutopilot and airframeSim DO throw on a structural failure
%   whatever 'assert' says; their rows below are therefore PASS-or-the-port-
%   errored, and are carried anyway so the residuals land on the board instead
%   of in a message nobody reads.
% =============================================================================

function R = adaptGeneric(key, listing, E)
%ADAPTGENERIC  The adapter every port from wave 4 onward uses.
%
%   ⭐⭐ WHY THIS EXISTS.  The first fifteen ports each got a hand-written
%   adapter.  That was fine at fifteen; the book has NINETY-EIGHT listings, and
%   ninety-eight bespoke adapters is ninety-eight places for the row schema to
%   drift while every board still prints full and green.
%
%   By wave 4 a convention had already emerged on its own: four of the five new
%   ports returned EXACTLY
%
%       what kind book book_str ours ours_str pct band tol ok skipped why
%       source units detail
%
%   so this function does what the bespoke adapters' CODE did (they were mostly
%   docstring): map those rows through benchRows, then turn each entry of
%   R.selfcheck.names/.oks into a structural row.
%
%   ⚠️ It is deliberately STRICT.  A port that returns a near-miss schema
%   (book_s instead of book_str, say) errors here rather than being quietly
%   accommodated -- one tolerated variant is how a convention stops being one.
%   monteCarloHoming was the single wave-4 deviant and was corrected, not
%   accommodated.
%
%   ⚠️ A port with genuinely unusual output still writes its own adapter. The
%   generic path is the default, not a requirement.
R = benchRows(key, listing, benchOrDie(E, key));
S = STRUCTSRC();

% ⚠️⚠️⚠️ 這一段以前是 `if ~isfield(...,'names'), return; end` —— **靜默**跳過。
%   上面那段 docstring 說本函式「deliberately STRICT，schema 對不上就報錯，
%   不默默遷就」。自檢這一支做的正好相反，而承諾與實作的落差沒有人看得見。
%
%   實測代價:shaping2d 與 shapingNonlinear 都**算完了**十條不變式(含四個
%   刪除實驗),但把結果掛在 K.bias_ok / K.nobias_ok 這種自訂欄位上，
%   沒有 .names/.oks ⇒ 兩個 port 各自貢獻 **0 列**結構檢查，而板面全綠。
%   一個「跑了刪除實驗」的移植，實際上一條都沒上板。
%
%   ⇒ 分成三種情況，而且只有第一種是安靜的:
%     ① 完全沒有 selfcheck 欄位  = 這個移植聲明它沒有自檢。安靜。
%        (五個早期移植屬於這類:adjointSingleLag / adjointSingleLagCh8 /
%         engage2d / engageLinear / weaveDesignCurve。沒覆蓋，但沒說謊。)
%     ② 有 selfcheck 但 schema 不合 = 有人算了檢查卻沒送到板上。**紅**。
%     ③ names 與 oks 長度不一致    = 對應關係已經錯了。**紅**。
if ~isfield(E, 'selfcheck')
    return
end
K = E.selfcheck;
if ~isfield(K, 'names') || ~isfield(K, 'oks')
    f = fieldnames(K);
    R = addFlagRow(R, key, listing, ...
        'structural: selfcheck reached the board', 'holds', 'LOST', false, S, ...
        sprintf(['⚠️ 這個移植回傳了 selfcheck，但沒有 .names/.oks ⇒ 它算出來的' ...
                 '檢查一條都沒上板。實際欄位:%s'], strjoin(f(:)', ', ')));
    return
end
if numel(K.names) ~= numel(K.oks)
    R = addFlagRow(R, key, listing, ...
        'structural: names and oks line up', 'holds', 'MISMATCH', false, S, ...
        sprintf('⚠️ names 有 %d 個、oks 有 %d 個 ⇒ 名字與結果的對應已經錯位。', ...
                numel(K.names), numel(K.oks)));
    return
end
for i = 1:numel(K.names)
    R = addFlagRow(R, key, listing, ...
        sprintf('structural: %s', K.names{i}), 'holds', holdStr(K.oks(i)), ...
        K.oks(i), S, '');
end
end


function R = benchRows(port, listing, B)
%BENCHROWS  The bench-row schema every self-citing port uses: what / kind /
%   book / book_str / ours / ours_str / pct / band / tol / ok / skipped / why /
%   source / units / detail.  Bands are RELATIVE in all six ports that reach
%   here, which is why bandPct and not bandAbs.
%
%   ONE COPY, SIX ADAPTERS.  This was adaptAutopilot's loop verbatim until the
%   third wave arrived carrying an identical schema; five more copies would
%   have been five more places for the INFO/SKIP handling to drift apart while
%   every one of them kept printing a full-looking board.
%
%   THE THREE UNSCORED KINDS ARE UNSCORED FOR THREE DIFFERENT REASONS AND MUST
%   STAY DISTINGUISHABLE:
%     INFO           the port computed a number and REFUSED to adjudicate it
%                    (the book's claim is an adjective, or the value is read
%                    off a diverging curve).  scored = false, skipped = false:
%                    it can neither pass nor fail, and it is not a check that
%                    failed to run.
%     SKIP           the check could not run here -- off-book parameters, a
%                    missing neighbour, a run too short to measure.  scored =
%                    true, skipped = true, so the tally counts a check that
%                    did not happen.
%     value+skipped  a value row whose computed number came back non-finite.
%                    Same treatment as SKIP, carrying the port's own 'why'.
%
%   An UNRECOGNISED kind falls into the value branch, which is what
%   adaptAutopilot did before this was factored out.  Left that way on purpose:
%   this function must not be the thing that decides a port is broken, and a
%   kind nobody knows prints as a value row carrying whatever the port put in
%   book_str/ours_str rather than vanishing off the board.
R = emptyRows();
for i = 1:numel(B)
    b = B(i);
    src = selfCite(b.source);
    switch lower(b.kind)
    case 'info'
        R(end+1) = mkrow(port, listing, b.what, b.book_str, b.ours_str, '--', ...
            'INFO', false, false, false, src, 'not scored', b.why); %#ok<AGROW>
    case 'skip'
        R(end+1) = mkrow(port, listing, b.what, b.book_str, '--', '--', ...
            'SKIP', false, true, true, src, 'not evaluable', b.why); %#ok<AGROW>
    case {'shape','bound'}
        R(end+1) = mkrow(port, listing, b.what, b.book_str, b.ours_str, ...
            upper(b.kind), vstr(b.ok), logical(b.ok), false, true, src, ...
            [b.band ', exact'], b.detail); %#ok<AGROW>
    otherwise
        if b.skipped
            R(end+1) = mkrow(port, listing, b.what, [b.book_str ' ' b.units], ...
                'skipped', '--', 'SKIP', false, true, true, src, ...
                bandPct(b.band, b.tol), b.why); %#ok<AGROW>
        else
            R(end+1) = mkrow(port, listing, b.what, [b.book_str ' ' b.units], ...
                [b.ours_str ' ' b.units], deltaStr(b.book, b.ours), ...
                vstr(b.ok), logical(b.ok), false, true, src, ...
                bandPct(b.band, b.tol), b.detail); %#ok<AGROW>
        end
    end
end
end


function B = benchOrDie(S, port)
%BENCHORDIE  Refuse an empty board.
%   Every third-wave port defaults 'benchmark' TRUE, so a bare call IS a
%   checked call for them -- unlike engage2d and weaveDesignCurve, which is why
%   the manifest passes the flag explicitly to everything.  What this guards is
%   the other direction: if the flag ever stops being honoured, R.bench comes
%   back empty, R.pass comes back FALSE MEANING "NOT CHECKED", and a suite that
%   shrugged at that would report zero failures for a port that adjudicated
%   nothing.
if ~isfield(S, 'bench') || isempty(S.bench)
    error('mgs:zarchan:benchSuite:noBench', ...
        ['%s returned no bench rows, so its ''benchmark'' flag was not ' ...
         'honoured.  Its R.pass is then false meaning "NOT CHECKED", which is ' ...
         'not the same thing as "fine".'], port);
end
B = S.bench;
end


function R = addFlagRow(R, port, listing, name, want, got, ok, src, detail)
%ADDFLAGROW  A structural row whose verdict is a BOOLEAN rather than a residual
%   under a tolerance -- "the clamp fires and releases", "the asymmetry is
%   present".  Same columns as addStructRow, so it tallies identically.
R(end+1) = mkrow(port, listing, name, want, got, '--', vstr(ok), ...
    logical(ok), false, true, src, 'holds / VIOLATED', detail);
end


function R = addNaRow(R, port, listing, name, why, src)
%ADDNAROW  A structural check that could not run at these parameters.
%   SKIP, never PASS.  "n/a" read as green is the precise failure this suite
%   exists to prevent, and airframeLinear counts its own n_applicable for the
%   same reason -- see the INFO row in its adapter.
R(end+1) = mkrow(port, listing, name, '0 (structural)', 'not evaluable', ...
    '--', 'SKIP', false, true, true, src, 'n/a', why);
end


function R = adaptAirframeSim(A)
%ADAPTAIRFRAMESIM  Listing 21.1, the NONLINEAR airframe of Fig. 21.3, run at
%   the book's two flight conditions (sea level and 50 kft, 5-deg fin,
%   3000 ft/s).  31 bench rows: 12 value, 3 bound, 10 shape, 5 INFO, 1 SKIP,
%   plus the structural block below.
%
%   ITS VALUE ROWS ARE LOOSE ON PURPOSE (graph 15%) and its SHAPE rows are the
%   sharp ones.  Ch.21's prose numbers were read off Figs. 21.4-21.7 to one
%   significant figure, and the OCR turned this chapter's minus signs into the
%   digit 2 -- "25 deg" is -5 deg, settled by the sentence two clauses earlier
%   that says "negative angle of attack".  A DIRECTION survives that intact
%   where a magnitude does not, so the sign of alpha, the wrong-way tail
%   effect and five orderings in altitude are where the teeth are.
R = benchRows('airframeSim', 'Listing 21.1', benchOrDie(A, 'airframeSim'));
if ~isfield(A, 'selfcheck') || ~isfield(A.selfcheck, 'ok')
    return
end
K = A.selfcheck;
S = STRUCTSRC();
R = addStructRow(R, 'airframeSim','Listing 21.1', ...
    'structural: CM(alphaTR) == 0', K.trim_resid, K.trim_tol, K.trim_ok, S, ...
    'the trim quadratic was solved, not merely written down');
R = addStructRow(R, 'airframeSim','Listing 21.1', ...
    'structural: dCM/da == y1 + 2*y2*aTR', K.jac_resid, K.jac_tol, K.jac_ok, S, ...
    'the only check that can see a mistyped 3, or a left-behind 1.5, in the derivative');
R = addStructRow(R, 'airframeSim','Listing 21.1', ...
    'structural: ODE settles on the trim root', K.alpha_resid, K.alpha_tol, ...
    K.alpha_ok, S, sprintf(['an ODE and a quadratic formula landing on one ' ...
    'number; the quadratic''s own DC offset on the ring is %.1e and the ' ...
    'settled estimator now fits it rather than eating it'], ...
    max(K.alpha_dc_pred)));
R = addStructRow(R, 'airframeSim','Listing 21.1', ...
    'structural: nL_ss == K1*delta (algebra)', K.k1_alg_resid, K.k_alg_tol, ...
    K.k1_alg_resid <= K.k_alg_tol, S, sprintf(['FLOORS at %.2e and cannot go ' ...
    'lower -- Zarchan''s 1845 is a rounded 57.3*32.2 = 1844.66'], ...
    abs(K.k_alg_expected)));
R = addFlagRow(R, 'airframeSim','Listing 21.1', ...
    'structural: 1845 once in K1, twice in K3', 'K1 rounds, K3 exact', ...
    sprintf('K1 %.1e / K3 %.1e', K.k1_alg_resid, K.k3_alg_resid), ...
    K.k_alg_asym_ok, S, ['the ASYMMETRY is the test: two equal residuals mean ' ...
    'one of the two 1845s has been dropped or duplicated']);
R = addStructRow(R, 'airframeSim','Listing 21.1', ...
    'structural: thetadot_ss == nL_ss*g/VM', K.chan_resid, K.chan_tol, ...
    K.chan_ok, S, ['ties the two output channels together; the DC offset ' ...
    'cancels here, so it is held 3x tighter than the row above it']);
R = addStructRow(R, 'airframeSim','Listing 21.1', ...
    'structural: Ta meas == Md/(Ma*Zd-Md*Za)', K.ta_resid, K.ta_tol, K.ta_ok, ...
    S, sprintf(['THE quantity this chapter was ported for: Ta = alpha/gammadot ' ...
    'measured off the nonlinear run, %s s, against the closed form %s s'], ...
    numList(K.ta_meas), numList(K.ta_form)));
if K.qscale_skipped
    R = addNaRow(R, 'airframeSim','Listing 21.1', ...
        'structural: Q-scaling across altitude', ...
        'needs two or more altitudes; this run has one', S);
else
    R = addStructRow(R, 'airframeSim','Listing 21.1', ...
        'structural: Q-scaling across altitude', K.qscale_resid, K.qscale_tol, ...
        K.qscale_ok, S, ['11 quantities, each at its own power of Q, in one ' ...
        'residual -- exercises the whole aero chain including the atmosphere branch']);
end
if ~K.dyn_meas_ok
    R = addNaRow(R, 'airframeSim','Listing 21.1', ...
        'structural: measured wn/zeta vs Jacobian', ...
        'the run did not produce enough extrema to fit a log decrement', S);
else
    R = addStructRow(R, 'airframeSim','Listing 21.1', ...
        'structural: measured wn vs Jacobian', K.wn_resid, K.wn_tol, ...
        K.wn_resid <= K.wn_tol, S, ...
        sprintf('measured %s vs %s rad/s', numList(K.wn_meas), numList(K.wn_jac)));
    R = addStructRow(R, 'airframeSim','Listing 21.1', ...
        'structural: measured zeta vs Jacobian', K.ze_resid, K.ze_tol, ...
        K.ze_resid <= K.ze_tol, S, ...
        sprintf('measured %s vs %s', numList(K.ze_meas), numList(K.ze_jac)));
end
R = addFlagRow(R, 'airframeSim','Listing 21.1', ...
    'structural: rings on the TANGENT, not on CMA', 'nearer the Jacobian', ...
    sprintf('secant gap wn %.1f%% / zeta %.1f%%', 100*K.wn_secant_gap, ...
    100*K.ze_secant_gap), K.wn_nearer && K.ze_nearer, S, ...
    ['Zarchan''s CMA is a SECANT through the origin (p.483 divides the alpha ' ...
     'terms by alpha, turning 1.5*Splan*a^2 into 1.5*Splan*a); this row is ' ...
     'what makes that a result and not a story']);
end


function R = adaptAirframeLinear(A)
%ADAPTAIRFRAMELINEAR  Listing 21.2, the LINEARISED airframe, plus Listing
%   22.1's acceleration trim -- which is where the only hard numbers ever
%   published for this airframe live (Tables 22.1/22.2: wAF, zAF, wZ, K1, Ta,
%   K3 at two flight conditions, twelve entries, all reproduced inside 1%).
%
%   TWELVE OF ITS SHAPE ROWS ARE SWEEPS ON OUR OWN GRID.  Figs. 21.12-21.17
%   became image links in the extraction, so the altitude x velocity grid the
%   book plotted is unrecoverable; the port sweeps alt [0 10k 20k 30k 40k 50k]
%   ft and VM [2000..4000] ft/s and prints that on every one of those rows.
%   What is checked there is the DIRECTION the prose states, which survived
%   extraction -- not a curve nobody has.
%
%   TWO ROWS READ Ma AND Md ON MAGNITUDE, and their detail says so: this
%   airframe is statically stable, so Ma is negative and "Ma gets smaller with
%   increasing altitude" gives the OPPOSITE verdict read as a signed number.
R = benchRows('airframeLinear', 'Listing 21.2 (+22.1)', ...
              benchOrDie(A, 'airframeLinear'));
if ~isfield(A, 'selfcheck') || ~isfield(A.selfcheck, 'ok')
    return
end
K = A.selfcheck;
S = STRUCTSRC();
R = addStructRow(R, 'airframeLinear','Listing 21.2', ...
    'structural: polynomial in descending powers', K.poly_resid, K.poly_tol, ...
    K.poly_ok, S, ['reversed coefficients give the RECIPROCAL roots, and a ' ...
    'reciprocal pole pair still prints like a lightly damped second-order system']);
R = addStructRow(R, 'airframeLinear','Listing 21.2', ...
    'structural: roots() returns wAF and zAF', K.damp_resid, K.damp_tol, ...
    K.damp_ok, S, 'damp() written out by hand: wn = |p|, zeta = -Re(p)/|p|');
R = addStructRow(R, 'airframeLinear','Listing 21.2', ...
    'structural: DC gain of nL/del == K1', K.dc_resid, K.dc_tol, K.dc_ok, S, ...
    'an identity of the transfer function, checked rather than assumed');
R = addStructRow(R, 'airframeLinear','Listing 21.2', ...
    'structural: resonant peak vs closed form', K.peak_resid, K.peak_tol, ...
    K.peak_ok, S, ['catches a flipped numerator sign: read as (1 + s^2/wz^2) ' ...
    'the peak comes out 15.2 instead of 36.1, and both look like plausible ' ...
    'resonances on a plot']);
R = addStructRow(R, 'airframeLinear','Listing 21.2', ...
    'structural: the peak sits at wAF', K.peakw_resid, K.peakw_tol, ...
    K.peakw_ok, S, '');
R = addStructRow(R, 'airframeLinear','Listing 21.2', ...
    'structural: CM == 0 at trim (nonlinear)', K.trim_resid, K.trim_tol, ...
    K.trim_ok, S, ['recomputed from the NONLINEAR moment, not from the ' ...
    'quadratic that produced the root']);
R = addStructRow(R, 'airframeLinear','Listing 21.2', ...
    'structural: K3 == 1845*K1/VM', K.k3_resid, K.k3_tol, K.k3_ok, S, ...
    'the listing''s own definition of K3, held at 1e-12');
R = addStructRow(R, 'airframeLinear','Listing 21.2', ...
    'structural: thd_ss == 57.3*32.2*nL_ss/VM', K.rate_resid, K.rate_tol, ...
    K.rate_ok, S, ['the PHYSICAL identity, and it is only good to 3e-5 because ' ...
    'the book''s 1845 is 1845.06 rounded.  Tightening this one to 1e-12 fails a ' ...
    'CORRECT port, which is why it carries a different tolerance from the row above']);
R = addStructRow(R, 'airframeLinear','Listing 21.2', ...
    'structural: fin trim is altitude-invariant', K.trimalt_resid, ...
    K.trimalt_tol, K.trimalt_ok, S, sprintf(['tolerance ZERO -- an identity, ' ...
    'not a tolerance: Q divides out of CM = 0, so alpha_trim is %.4f deg at ' ...
    'BOTH altitudes.  This is the fact behind Fig. 21.6 being scored on the ' ...
    'transient peak rather than on trim'], K.trimalt_deg));
R = addStructRow(R, 'airframeLinear','Listing 21.2', ...
    'structural: Ch.22 trim round-trips through Ch.21', K.rt_resid, K.rt_tol, ...
    K.rt_ok, S, 'the acceleration trim fed back through the fin-trim quadratic');
R = addStructRow(R, 'airframeLinear','Listing 21.2', ...
    'structural: Ch.22 trim delivers exactly 10 g', K.g_resid, K.g_tol, ...
    K.g_ok, S, 'the definition of that trim, and the precondition for Tables 22.1/22.2');
if K.lag_applies
    R = addStructRow(R, 'airframeLinear','Listing 21.2', ...
        'structural: envelope fit vs -1/Re(pole)', K.lag_rel, K.lag_tol, ...
        K.lag_ok, S, sprintf(['TWO entirely different computations of the same ' ...
        'lag: %.4f s fitted off the integration, %.4f s from roots()'], ...
        K.lag_meas, K.lag_form));
else
    R = addNaRow(R, 'airframeLinear','Listing 21.2', ...
        'structural: envelope fit vs -1/Re(pole)', K.lag_why, S);
end
if K.per_applies
    R = addStructRow(R, 'airframeLinear','Listing 21.2', ...
        'structural: measured period vs 2*pi/wd', K.per_rel, K.per_tol, ...
        K.per_ok, S, ['the 0.01 s recorder is 4% of a period at wAF = 25 rad/s, ' ...
        'so this cannot honestly be held tighter than 2%']);
else
    R = addNaRow(R, 'airframeLinear','Listing 21.2', ...
        'structural: measured period vs 2*pi/wd', ...
        'the run produced too few extrema to measure a period', S);
end
if K.ss_applies
    R = addStructRow(R, 'airframeLinear','Listing 21.2', ...
        'structural: the run settles on K1*del', K.ss_resid, K.ss_tol, ...
        K.ss_ok, S, ['the final value theorem taken off the record rather than ' ...
        'off the algebra that produced it']);
else
    R = addNaRow(R, 'airframeLinear','Listing 21.2', ...
        'structural: the run settles on K1*del', ...
        ['the run is shorter than two envelope time constants, so a window mean ' ...
         'is measuring the TRANSIENT -- scoring it here would red-light a ' ...
         'correct port at altitude'], S);
end
if K.otherroot_applies
    R = addFlagRow(R, 'airframeLinear','Listing 21.2', ...
        'structural: discarded trim root is unrealistic', '|alpha| > 90 deg', ...
        sprintf('%.2f deg', K.otherroot_deg), K.otherroot_ok, S, ...
        'p.487 calls it "the unrealistic root" and never prints its value');
else
    R = addNaRow(R, 'airframeLinear','Listing 21.2', ...
        'structural: discarded trim root is unrealistic', ...
        'only meaningful on the fin trim; this run is on the acceleration trim', S);
end
if K.trimnl_applies
    R = addStructRow(R, 'airframeLinear','Listing 21.2', ...
        'structural: 21.1 twin settles at the 21.2 trim', K.trimnl_resid, ...
        K.trimnl_tol, K.trimnl_ok, S, sprintf(['THE delete-experiment for the ' ...
        'trim quadratic: get it wrong and the linear model still runs, still ' ...
        'oscillates, still looks like a missile, and sits at the wrong angle of ' ...
        'attack.  Measured %.4f deg vs %.4f deg'], K.trimnl_meas, K.trimnl_form));
else
    R = addNaRow(R, 'airframeLinear','Listing 21.2', ...
        'structural: 21.1 twin settles at the 21.2 trim', ...
        'the nonlinear twin was not run, or this run is on the acceleration trim', S);
end
R(end+1) = mkrow('airframeLinear','Listing 21.2', ...
    'how much of the self-check applied', sprintf('%d of %d', K.n_total, ...
    K.n_total), sprintf('%d of %d', K.n_applicable, K.n_total), '--', 'INFO', ...
    false, false, false, S, 'not scored', ...
    ['carried because a mode that switches checks into n/a must not be able to ' ...
     'read green: five of these sixteen are MEASUREMENTS off the run and go n/a ' ...
     'on a run too short to make them']);
end


function R = adaptFlightControlDelay(D)
%ADAPTFLIGHTCONTROLDELAY  Listing 22.4, the rate-gyro flight-control system
%   with a PURE TRANSPORT DELAY in series with the autopilot gain.  This is
%   the port that measures a delay margin: 0.0147 s against the book's 0.015 s,
%   with the stated 55 deg of phase margin and 11.5 dB of gain margin both
%   reproduced by destabilising the same loop two different ways.
%
%   SEVEN OF ITS BOOK VALUES TRACE TO ONE SOURCE -- the author reading Fig.
%   22.16 -- so 55 deg, 11.5 dB, 64 and 150 rad/s and everything the prose
%   computes FROM them all carry the same 5% 'approx' band, including the ones
%   this port matches to a fraction of a per cent.  Tightening a band because
%   our number would survive it is the same error as widening one because it
%   would not, and the port fixed that rule before it had the results.
%
%   ('margins',false) would drop the two bisections -- about 3 s of its ~4 s --
%   and turn the delay-margin and gain-margin groups into SKIP rows.  The
%   manifest does NOT pass it: a 3 s saving is not worth a board that quietly
%   stops measuring the one quantity the listing exists to measure.
R = benchRows('flightControlDelay', 'Listing 22.4', ...
              benchOrDie(D, 'flightControlDelay'));
if ~isfield(D, 'selfcheck') || ~isfield(D.selfcheck, 'ok')
    return
end
K = D.selfcheck;
S = STRUCTSRC();
R = addStructRow(R, 'flightControlDelay','Listing 22.4', ...
    'structural: trim reproduces the 10 g command', K.trim_cn_resid, ...
    K.trim_cn_tol, K.trim_cn_ok, S, ['also the proof that ALFTR/DELTR are in ' ...
    'RADIANS: any other unit is off by a factor of 57.3, not by 5e-16']);
R = addStructRow(R, 'flightControlDelay','Listing 22.4', ...
    'structural: the trim moment is zero', K.trim_cm_resid, K.trim_cm_tol, ...
    K.trim_cm_ok, S, '');
if K.buffer_skipped
    R = addNaRow(R, 'flightControlDelay','Listing 22.4', ...
        'structural: delay buffer is exactly DINT steps', ...
        'the run under test is not the delayed one, so there is no buffer to replay', S);
else
    R = addFlagRow(R, 'flightControlDelay','Listing 22.4', ...
        'structural: delay buffer is exactly DINT steps', '0 violations', ...
        sprintf('%d of %d samples', K.buffer_violations, K.buffer_checked), ...
        K.buffer_ok, S, ['THE row that settles "DELC=Z(I+2-DINT)": the recorded ' ...
        'command is replayed against the recorded actuator input and BIT ' ...
        'equality is demanded at exactly DINT samples of offset, tolerance zero. ' ...
        'An off-by-one still rings and still diverges, just at the wrong margin']);
end
R = addStructRow(R, 'flightControlDelay','Listing 22.4', ...
    'structural: unity DC gain, delay removed', K.dc_err, K.dc_tol, K.dc_ok, ...
    S, 'XKDC = (1-KR*K3)/(K1*KR) exercised end to end in one number');
if K.delete_skipped
    R = addNaRow(R, 'flightControlDelay','Listing 22.4', ...
        'structural: DELETE EXPERIMENT, remove the delay', ...
        ['the run under test is not the delayed one -- comparing it against ' ...
         'itself gives a ratio of 1 and reports a FAILURE for a correct port'], S);
else
    R = addFlagRow(R, 'flightControlDelay','Listing 22.4', ...
        'structural: DELETE EXPERIMENT, remove the delay', ...
        sprintf('>= %.0e', K.delete_tol), sprintf('x%.3g', K.delete_ratio), ...
        K.delete_ok, S, sprintf(['last-quarter peak-to-peak %.4g g delayed ' ...
        'against %.3g g undelayed.  Without it, "the response grows" is equally ' ...
        'consistent with a broken integrator, a wrong sign on the rate feedback, ' ...
        'or a numerically unstable actuator'], K.delete_pp_delay, ...
        K.delete_pp_nodelay));
end
if K.lag_skipped
    R = addNaRow(R, 'flightControlDelay','Listing 22.4', ...
        'structural: a LAG in the same slot stays stable', ...
        'the lag probe did not run at these parameters', S);
else
    R = addFlagRow(R, 'flightControlDelay','Listing 22.4', ...
        'structural: a LAG in the same slot stays stable', ...
        'lag stable, delay not', sprintf('lag sigma %s / delay %+.3g', ...
        numList(K.lag_sigma), K.delay_sigma), K.lag_ok, S, ...
        ['THE measurement this project needs and Zarchan never makes: the same ' ...
         'time constant, in the same place in the loop, opposite verdicts.  Not ' ...
         'on the benchmark board because the book never contrasts the two, and ' ...
         'scoring it against a citation would mean inventing one']);
end
end


function R = adaptThreeLoop(T)
%ADAPTTHREELOOP  Listing 23.1, the three-loop autopilot step response at three
%   flight conditions.  34 scored value rows, worst -8.24% inside a 15% graph
%   band, everything else inside 1.3%.
%
%   ITS SHARPEST ROW EXISTS ONLY BECAUSE THE OUTPUT DECIMATION WAS PORTED.
%   Listing 23.1 carries its own sampler (S = S + H; if S >= .0099999 ...), so
%   Figs. 23.3 and 23.12 sit on a 0.01 s grid rather than the 1e-4 s
%   integration grid.  Read there, t63 is 0.3400 s at 0, 30 AND 50 kft -- the
%   published 0.34 s exactly, at every flight condition, which is what makes
%   "the same step response for the three different flight conditions"
%   literally true.  Read at full rate it is 0.3367 / 0.3351 / 0.3304 s and the
%   same claim holds only to 1.9%.
%
%   ONE OF ITS ROWS RUNS ANOTHER PORT.  The Ch.23 preamble is duplicated
%   byte-for-byte across Listings 23.1/23.2/23.3 and MATLAB cannot share a
%   local function between files, so threeLoopAutopilot RUNS
%   adjointWithAutopilot and compares 63 aero and gain quantities at 1e-12.
%   That row is inside its bench array (it carries its own .source) and it
%   reads SKIP, not PASS, if the neighbour is missing -- so check which, since
%   a green board with that row skipped does not prove the two Ch.23 ports
%   agree.  It is also the only row in that port that still bites off-book.
R = benchRows('threeLoopAutopilot', 'Listing 23.1', ...
              benchOrDie(T, 'threeLoopAutopilot'));
if ~isfield(T, 'selfcheck') || ~isfield(T.selfcheck, 'ok')
    return
end
K = T.selfcheck;
S = STRUCTSRC();
R = addStructRow(R, 'threeLoopAutopilot','Listing 23.1', ...
    'structural: unity closed-loop DC gain', K.dc_err, K.dc_tol, K.dc_ok, S, ...
    ['an IDENTITY, not a design goal: substituting the gain algorithm into the ' ...
     'closed-loop steady state collapses it to KDC/(1+KC), and the algorithm''s ' ...
     'own KDC is identically 1 + KC.  One number exercises K1 -> K3 -> KC -> ' ...
     'KA -> KDC']);
R = addStructRow(R, 'threeLoopAutopilot','Listing 23.1', ...
    'structural: t63 measured vs tau + 2*zeta/W', K.lag_rel, K.lag_tol, ...
    K.lag_ok, S, sprintf(['THE LAG IS DERIVED, NOT FITTED: %s s measured off ' ...
    'the run against %s s of closed form'], numList(K.lag_meas), ...
    numList(K.lag_form)));
R = addFlagRow(R, 'threeLoopAutopilot','Listing 23.1', ...
    'structural: the output sampler is a 0.01 s grid', 'n and dt both right', ...
    sprintf('n = %d/%d, dt err %.2e', K.grid_n, K.grid_n_want, K.grid_dt_err), ...
    K.grid_ok, S, ['the decimation is not cosmetic -- it is what makes the ' ...
    'published 0.34 s and the "same response at three altitudes" claim exact ' ...
    'rather than approximate']);
R = addFlagRow(R, 'threeLoopAutopilot','Listing 23.1', ...
    'structural: RHP zero present, opposes, grows', 'wrong-way dip, x>1 with alt', ...
    sprintf('%.4g / %.4g g, ratio x%.3g', K.ww(1), K.ww(2), K.ww_ratio), ...
    K.ww_ok, S, ['nL = K1*(E - EDD/wZ^2) and the second term IS the ' ...
    'right-half-plane zero: a transcription that dropped it still rises to the ' ...
    'right steady state and still passes the DC-gain identity, because the zero ' ...
    'contributes nothing at DC']);
R(end+1) = mkrow('threeLoopAutopilot','Listing 23.1', ...
    'where the 1 s run stops, as a fraction of ss', 'not published', ...
    sprintf('%s of steady state', numList(K.settled_frac)), '--', 'INFO', ...
    false, false, false, S, 'not scored', ...
    ['this is why the port scores "oscillatory" with a RINGING COUNT and not ' ...
     'with percent overshoot: the response never crosses its own steady state ' ...
     'inside TF = 1 s, so overshoot measures 0.00% even in the case the book ' ...
     'calls oscillatory']);
end


function R = adaptEngage3d(E)
%ADAPTENGAGE3D  Listing 28.1, the THREE-DIMENSIONAL engagement against a
%   spiralling target -- the only port on this board that flies our own law in
%   our own number of dimensions.  Its guidance kernel is ZEM = RTM + VTM*tgo
%   projected perpendicular to the line of sight, then N'*ZEMPER/tgo^2, which
%   is PPN in ZEM form.
%
%   THE HEADLINE IS A CROSS-CHAPTER COMPARISON AND ITS BAND SAYS SO.  Ch.28
%   asserts the 3-D miss "agrees with the single plane results of Fig. 20.7"
%   and quotes no digits, so the reference is Ch.20's closed form
%   32.2x/(1+x^2)^1.5 evaluated at x = wT = 3 -- and the port scores five Ch.20
%   provenance rows FIRST, to establish that the closed form is the right one
%   before leaning on it.  Band 'xchap' 15%, with a 4.4% floor set by the
%   terminal step quantum.
%
%   ITS STRUCTURAL BLOCK IS THE BIGGEST ON THE BOARD AND EARNS IT.  The port
%   resolved three OCR-damaged comparisons to '>' and then WIRED A DELETE
%   EXPERIMENT that discriminates that reading, rather than leaving the
%   resolution in a comment -- which is the standing lesson from
%   [[promised-guards-that-do-not-exist]].
R = benchRows('engage3d', 'Listing 28.1', benchOrDie(E, 'engage3d'));
if ~isfield(E, 'selfcheck') || ~isfield(E.selfcheck, 'names')
    return
end
K = E.selfcheck;
S = STRUCTSRC();
for i = 1:numel(K.names)
    R = addFlagRow(R, 'engage3d','Listing 28.1', ...
        sprintf('structural: %s', K.names{i}), 'holds', holdStr(K.oks(i)), ...
        K.oks(i), S, engage3dDetail(K, K.names{i}));
end
% NOT SCORED.  Both are findings ABOUT the listing rather than claims IN the
% book, which is exactly why they are INFO rows and not bound rows.
if isfield(K, 'clamp_alive_overshoot_frac')
    R(end+1) = mkrow('engage3d','Listing 28.1', ...
        'the clamp-before-corrector overshoot', 'no book claim exists', ...
        sprintf('%.2fx the limit', 1 + K.clamp_alive_overshoot_frac), '--', ...
        'INFO', false, false, false, S, 'not scored', ...
        ['the clamp runs inside the derivative loop and the corrector then ' ...
         'averages a CLAMPED predictor with an UNCLAMPED old value, so the ' ...
         'state ends each step OUTSIDE XNCLIM -- a "2 g limit" actually flies ' ...
         'about 5 g.  The same ordering trap as our own nz_clamp_after_floor. ' ...
         'MEASURED, NOT FIXED: fixing it changes the book''s answer, and the ' ...
         'book''s answer is what is being reproduced']);
end
if isfield(K, 'guard_takeover_s')
    R(end+1) = mkrow('engage3d','Listing 28.1', ...
        'where the .0001 guard takes the law over', 'never discussed', ...
        sprintf('last %.4g s of time to go', K.guard_takeover_s), '--', ...
        'INFO', false, false, false, S, 'not scored', ...
        ['BOT1+BOT2 cancels to fifth order out of terms of size 3, so below ' ...
         'x ~ 1e-3 the unguarded denominator is pure roundoff and can be ' ...
         'NEGATIVE.  With the guard, N'' -> 0 instead: the compensated weave ' ...
         'law does not saturate near intercept, it TURNS ITSELF OFF']);
end
end


function s = engage3dDetail(K, name)
%ENGAGE3DDETAIL  Put the NUMBER on engage3d's boolean structural rows.
%   Keyed on that port's own name strings.  An unrecognised name returns ''
%   rather than erroring: a renamed check must lose its detail, not take the
%   board down -- the same trade cite() makes when it routes a renamed row to
%   "** UNMAPPED **" instead of guessing.
s = '';
switch name
case 'initial VC = VT + VM'
    s = sprintf('err %.2e ft/s, tolerance %.0e', K.vc0_err, K.vc0_tol);
case 'PN run leaves XNP untouched'
    s = sprintf(['XNP in [%.6g, %.6g], tolerance EXACTLY 0.  A mis-transcribed ' ...
        'branch test would run the weave law instead, sweep XNP over four ' ...
        'decades, and the miss would still look like a number'], ...
        K.xnp_pn_min, K.xnp_pn_max);
case 'weave N'' -> 3 as x -> inf'
    s = sprintf(['N'' = %.5g at x = %g and %.5g at x = %g.  x = 100 is ' ...
        'deliberately NOT sampled: it is only 2%% converged and would fail a ' ...
        '2%% band on a CORRECT transcription'], K.xnp_inf(1), K.xnp_inf_x(1), ...
        K.xnp_inf(2), K.xnp_inf_x(2));
case 'unguarded denominator is roundoff at x=1e-3'
    s = sprintf('computed %.3e against the analytic 0.3*x^5 = %.3e', ...
        K.guard_bot_raw, K.guard_bot_true);
case 'guard changes XNP by >= 50% below x = 0.25'
    s = sprintf(['worst change %.1f%%; the guard takes over at x = %.4g, the ' ...
        'last %.4g s of time to go'], 100*K.guard_max_change, ...
        K.guard_takeover_x, K.guard_takeover_s);
case 'delete experiment: aT = 0 collapses the miss'
    s = sprintf(['%.3f ft against a %.3f ft threshold of one closing step.  ' ...
        'Without this row every miss-magnitude row above would also pass if the ' ...
        'miss were being produced by the geometry or by the integrator'], ...
        K.nomaneuver_miss, K.nomaneuver_tol);
case 'clamp inert at XNCLIM = 9999999999'
    s = sprintf('%d hits, %.4g of evaluations strictly inside, peak/limit %.2e', ...
        K.clamp_inert_hits, K.clamp_inert_frac, K.peak_am_over_limit);
case 'delete experiment: clamp fires AND releases at 2 g'
    s = sprintf(['%.1f%% fire / %.1f%% strictly inside.  THE discriminator for ' ...
        'the OCR reading: under the inverted ''<'' the inside fraction is 0 BY ' ...
        'CONSTRUCTION, because every strictly-inside sample is exactly what the ' ...
        'inverted test overwrites'], 100*K.clamp_alive_hits_frac, ...
        100*K.clamp_alive_inside_frac);
case 'missile speed drift under un-normalized accel'
    s = sprintf(['%.2f%% against a %.0f%% band -- the model genuinely does not ' ...
        'conserve speed, and the drift is dominated by the last few steps where ' ...
        'the command spikes to ~350 g'], 100*K.vm_drift, 100*K.vm_drift_tol);
case 'miss components really are sinusoidal in tF'
    s = sprintf(['worst r2 %.5f against %.2f.  Every amplitude and frequency row ' ...
        'above is meaningless if this one is red'], K.fit_r2, K.fit_r2_tol);
case 'amplitude stable across a 1 s window shift'
    s = sprintf(['%.2f%%.  The steady-state window is OURS -- the book says ' ...
        '"after an initial transient period" and never quantifies it -- so if ' ...
        'the answer moved when the window moved, the answer would be about the ' ...
        'window'], 100*K.window_shift);
end
end


% =============================================================================
%   CITATIONS.  Keyed on each port's own row name.  A rename lands in
%   "** UNMAPPED **" and is counted at the bottom of the board, because a
%   stale figure number printed with confidence is worse than a missing one.
% =============================================================================
function s = cite(port, key)
switch port
case 'engage2d'
    M = { ...
    'A|n_c(0)',              'Fig. 2.3 (also 2.7)'
    'A|n_c monotone decay',  'Fig. 2.3 + p.21 prose'
    'A|n_c at last log',     'p.21 prose'
    'A|flight time',         'Fig. 2.3 time axis'
    'A|miss distance',       'p.21 prose, Fig. 2.2'
    'B|n_c(0) / case A',     'Ch.2 PN definition p.16'
    'B|n_c(0) > case A',     'Fig. 2.3 caption'
    'B|n_c monotone decay',  'Fig. 2.3 + p.21 prose'
    'B|n_c(5.1s) < case A',  'p.21 prose'
    'B|miss distance',       'p.21 prose, Fig. 2.2'
    'B|n_c(0)',              'BRIEF ONLY - not in Ch.2'
    'C|n_c(0), coll. tri.',  'p.21-22 prose'
    'C|rises to its peak',   'p.22 prose, Fig. 2.5'
    'C|peak / target 3 g',   'p.23 prose, Fig. 2.5'
    'C|peak N''=3 > 4 > 5',  'Fig. 2.5 caption'
    'C|miss distance',       'p.22 prose, Fig. 2.4'
    'C|peak n_c',            'BRIEF ONLY - a Ch.3/6 number'};
case 'engageLinear'
    M = { ...
    'A: nc(0) matches closed form',         'Fig. 2.7 + Ch.2 closed form'
    'A: RK2 tracks exact solution',         'Ch.2 closed form (integrator)'
    'A: zero-lag miss is zero',             'Ch.2 miss defn; Ch.3 premise'
    'B: nc(tgo=0.5) matches closed form',   'Fig. 2.8 + closed form'
    'B: nc(tgo=0.5) approaches 6 g',        'Fig. 2.8; N''nT/(N''-2)'
    'B: RK2 tracks exact solution',         'Ch.2 closed form (integrator)'
    'B: nc monotonically increasing',       'Fig. 2.8 / Fig. 2.5 prose'
    'B: zero-lag miss is zero',             'Ch.2 miss definition'
    'C: trajectory independent of VC',      'Listing 2.2 data stmt (no VC)'
    'D1: linear ~ nonlinear (book TF)',     'Fig. 2.7'
    'D1: better at matched TF',             'Fig. 2.7'
    'D2: linear OVER-predicts terminal nc', 'Fig. 2.8 (direction only)'
    'D2: median terminal margin positive',  'Fig. 2.8 (direction only)'};
case 'adjointSingleLag'
    M = { ...
    'fig313',     'Fig. 3.13 / p.47 prose'
    'closedform', 'p.52-55, Fig. 3.16'
    'he',         'p.54-55, Figs 3.14/3.18'};
case 'adjointBinomial5'
    M = { ...
    'step  N''=3 @ tF/T=5',   'Fig. 6.6 / p.114'
    'ramp  N''=3 @ tF/T=5',   'Fig. 6.7 / p.116'
    'parab N''=3 @ tF/T=5',   'Fig. 6.8 / p.116'
    'opt.evasive N''=3',      'Fig. 6.9 / p.119 text'
    'step  N''=3 @ tF/T=2.5', 'Fig. 6.6 / p.115'
    'ramp  N''=4 @ tF/T=5',   'Fig. 6.7 / p.116'
    'parab N''=4 @ tF/T=5',   'Fig. 6.8 / p.116'
    'parab N''=5 @ tF/T=5',   'Fig. 6.8 / p.116'
    'opt.evasive N''=4',      'Fig. 6.10 / p.119 text'
    'opt.evasive N''=5',      'Fig. 6.11 / p.119 text'
    'sc_openloop',            'structural - NOT a book value'
    'sc_vc',                  'structural - NOT a book value'
    'sc_tau',                 'structural - NOT a book value'};
case 'weaveDesignCurve'
    M = { ...
    'N''=3  unlimited        x=2', 'Fig. 20.9/20.10 + text'
    'N''=3  unlimited        x=1', 'Fig. 20.10 + text'
    'N''=3  nLIM/nT = 3      x=1', 'Fig. 20.10 + text'
    'N''=3  nLIM/nT = 2      x=1', 'Fig. 20.10 + text'
    'N''=3  nLIM/nT = 1      x=1', 'Fig. 20.10 + text (85, erratum)'
    'N''=5  unlimited        x=1', 'Fig. 20.12 - no worked value'
    'N''=5  nLIM/nT = 3      x=1', 'Fig. 20.12 - no worked value'
    'peak',                        'Fig. 20.9 + text'
    'xpeak',                       'Fig. 20.9 text'
    'crossover',                   'Fig. 20.9 + following para'
    'coast1',                      'text after Fig. 20.11'
    'coast2',                      'text after Fig. 20.11'};
case {'covarianceHoming','accelMissAdjoint','adjointSingleLagCh8', ...
      'adjointOptimal','adjointWithAutopilot','airframeSim','airframeLinear', ...
      'flightControlDelay','threeLoopAutopilot','engage3d'}
    % NOT AN OVERSIGHT, AND DO NOT "FIX" IT BY ADDING A TABLE.  Every row of
    % these five ports already carries a .source naming the figure or page
    % AND the zarchan.txt line it came from, and their adapters read it
    % through selfCite().  A table here would be a second copy of a citation
    % that is already correct at the source -- precisely the drifting
    % duplicate this file's header warns about -- so reaching this branch is
    % a wiring mistake and is reported as one rather than silently answered.
    error('mgs:zarchan:benchSuite:citeSelf', ...
        ['port "%s" carries its own citation on every row.  Read it with ' ...
         'selfCite(row.source) -- benchRows() already does -- and do NOT add ' ...
         'a cite() table for it.'], port);
otherwise
    % A port with no citation table at all is a wiring error, not a data
    % condition: fix the table rather than shipping a board of blanks.
    error('mgs:zarchan:benchSuite:cite', ...
        'no citation table for port "%s".', port);
end
i = find(strcmp(M(:,1), key), 1);
if isempty(i)
    s = UNMAPPED();
else
    s = M{i,2};
end
end

function s = UNMAPPED()
s = '** UNMAPPED **';
end

function s = STRUCTSRC()
%STRUCTSRC  The exact source string every structural row carries.
%   ONE literal, because the run loop tests rows against it to tell a book
%   citation from a self-consistency check, and every adapter's local S now
%   calls this rather than repeating the string.  The three sc_* entries in
%   cite()'s adjointBinomial5 table are the only remaining copies and must
%   stay byte-identical, or the "NO BOOK VALUE ADJUDICATED" guard stops
%   recognising those rows as structural and quietly stops guarding.
s = 'structural - NOT a book value';
end


% =============================================================================
%   PLUMBING
% =============================================================================
function R = emptyRows()
R = struct('port',{}, 'listing',{}, 'name',{}, 'book',{}, 'computed',{}, ...
           'delta',{}, 'verdict',{}, 'ok',{}, 'skipped',{}, 'scored',{}, ...
           'source',{}, 'band',{}, 'detail',{});
end

function r = mkrow(port, listing, name, book, computed, delta, verdict, ok, ...
                   skipped, scored, source, band, detail)
%MKROW  The ONLY row constructor.  Field names and their ORDER must match
%   emptyRows exactly or the appends in the adapters throw -- which is the
%   behaviour we want: a struct array that quietly reordered fields would make
%   [rows.ok] mean something else.
r = struct('port',port, 'listing',listing, 'name',name, 'book',book, ...
           'computed',computed, 'delta',delta, 'verdict',verdict, ...
           'ok',logical(ok), 'skipped',logical(skipped), ...
           'scored',logical(scored), 'source',source, 'band',band, ...
           'detail',detail);
end

function [v, txt] = quiet(fh)
%QUIET  Run FH and capture what it prints.
%   engage2d's benchmark branch prints unconditionally -- that branch has no
%   render flag -- so five ports printing five full tables would bury the board
%   this file exists to produce.  Nothing is discarded: the text comes back in
%   S.ports(i).stdout and ('echo',true) prints it.
%   NOTE: if FH errors, whatever it printed before the error is lost with it.
%
%   FH IS USED INSIDE THE evalc STRING, where the Code Analyzer cannot see it
%   -- it reports "input argument might be unused" and offers to replace fh
%   with ~.  Taking that offer would leave a function that captures the output
%   of nothing and returns [] for every port, i.e. a board of empty rows.  The
%   type check below is a real guard (evalc on a non-handle fails with a
%   message about a string, not about the argument) and it also makes the use
%   visible so nobody is invited to "fix" the warning.
if ~isa(fh, 'function_handle')
    error('mgs:zarchan:benchSuite:quiet', ...
        'quiet() takes a function handle; got a %s.', class(fh));
end
v = [];
txt = evalc('v = fh();');
end

function s = deltaStr(book, comp)
%DELTASTR  Signed difference, with percent when the book value is nonzero.
%   A NaN on either side prints '--' rather than 'NaN': a NaN quietly compared
%   into a pass is the commonest way a board goes green while broken, and the
%   verdict column, not this one, is where that has to be caught.
if ~isfinite(book) || ~isfinite(comp)
    s = '--';
elseif book == 0
    s = sprintf('%+.3g', comp);
else
    s = sprintf('%+.3g/%+.1f%%', comp - book, 100*(comp/book - 1));
end
end

function s = vstr(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end

function s = holdStr(ok)
if ok, s = 'holds'; else, s = 'VIOLATED'; end
end

function s = boolStr(v)
if v, s = 'true'; else, s = 'false'; end
end

function s = numList(v)
%NUMLIST  A short vector printed as "a/b/c", for rows whose computed value is
%   one number per flight condition rather than a scalar.
s = strjoin(arrayfun(@(x) sprintf('%.4g', x), v(:).', ...
                     'UniformOutput', false), '/');
end

function s = selfCite(src)
%SELFCITE  Citation for the five second-wave ports, which carry their own on
%   every row.  See the header: cite() deliberately has no table for them.
%
%   An EMPTY source becomes "** UNMAPPED **" so the unmapped counter at the
%   bottom of the board covers this mechanism as well as cite()'s.  A blank
%   source column would otherwise read as a formatting quirk rather than as a
%   lost reference, which is the failure this whole convention exists to stop.
if isstring(src), src = char(src); end
if ~ischar(src) || isempty(strtrim(src))
    s = UNMAPPED();
else
    s = strtrim(regexprep(src, '\s+', ' '));
end
end

function s = bandPct(band, tol)
%BANDPCT  Band label for a port whose tolerance is RELATIVE (a fraction of the
%   book value): covarianceHoming, accelMissAdjoint, adjointWithAutopilot.
%   CHECK THE PORT BEFORE REUSING THIS.  adjointOptimal's bands are ABSOLUTE
%   and rendering them as percentages would mis-state every tolerance it has.
if isstring(band), band = char(band); end
if isempty(band), band = '?'; end
if ~isscalar(tol) || ~isfinite(tol)
    s = band;
elseif tol == 0
    s = sprintf('%s, exact', band);
else
    s = sprintf('%s %.4g%%', band, 100*tol);
end
end

function s = bandAbs(band, tol, units)
%BANDABS  Band label for a port whose tolerance is ABSOLUTE, in the row's own
%   units.  adaptAdjointOptimal says why that port is built that way.
if isstring(band), band = char(band); end
if isstring(units), units = char(units); end
if isempty(band), band = '?'; end
if ~isscalar(tol) || ~isfinite(tol)
    s = band;
elseif tol == 0
    s = sprintf('%s, exact', band);
else
    s = strtrim(sprintf('%s +/-%.4g %s', band, tol, units));
end
end

function R = addStructRow(R, port, listing, name, resid, tol, ok, src, detail)
%ADDSTRUCTROW  One structural-invariant row, of the "residual <= tol" shape.
%   The book column reads "0 (structural)" and the source column repeats that
%   it is not a book value.  Both, on every row, on purpose: a self-consistency
%   residual sitting under a column headed "book" is how a wiring check gets
%   quoted six months later as evidence that a port reproduces Zarchan.  It is
%   not that claim; it is the weaker one that the port agrees with itself.
if ~isscalar(tol) || ~isfinite(tol)
    bandTxt = 'no tolerance';
elseif tol == 0
    bandTxt = 'EXACTLY 0';
else
    bandTxt = sprintf('<= %.3g', tol);
end
R(end+1) = mkrow(port, listing, name, '0 (structural)', ...
    sprintf('%.4g', resid), deltaStr(0, resid), vstr(ok), logical(ok), ...
    false, true, src, bandTxt, detail);
end

function s = fitCite(src)
%FITCITE  DISPLAY ONLY.  The second-wave ports carry citations that name the
%   figure, the page AND the zarchan.txt line, which is longer than the board
%   is wide.  The FULL string stays in rows(j).source; only the print is
%   trimmed.  The line number sits at the FRONT of those strings, so trimming
%   never costs the part you need in order to go and look it up.  Every
%   first-wave citation is well under the limit, so nothing about how the
%   original five ports print has changed.
LIM = 66;
if numel(src) > LIM
    s = [src(1:LIM-3) '...'];
else
    s = src;
end
end

function s = oneLine(msg)
%ONELINE  Collapse a multi-line error message so it cannot break the table.
s = regexprep(msg, '\s+', ' ');
if numel(s) > 200
    s = [s(1:197) '...'];
end
end


% =============================================================================
%   RENDER
% =============================================================================
function s = render(rows, S, ports, X, opts)
W = 128;
L = {};
L{end+1} = 'ZARCHAN BOOK BENCHMARKS -- every audited port on one board';
L{end+1} = sprintf('  %s', S.units);
L{end+1} = '  row = what was checked | what the book publishes | what we compute | delta | verdict | where it came from';
L{end+1} = repmat('=', 1, W);

for i = 1:numel(ports)
    p = ports(i);
    mine = strcmp({rows.port}, p.key);
    r = rows(mine);
    nf = sum([r.scored] & ~[r.ok] & ~[r.skipped]);
    if ~p.ok
        state = 'PORT ERRORED';
    elseif nf == 0
        state = 'OK';
    else
        state = sprintf('%d FAILED', nf);
    end
    L{end+1} = ''; %#ok<AGROW>
    L{end+1} = sprintf('%s  --  %s  [%s]   %.2f s   %s', ...
        p.key, p.chapter, p.listing, p.elapsed_s, state); %#ok<AGROW>
    L{end+1} = sprintf('  entry: mgs.zarchan.%s', p.entry); %#ok<AGROW>
    L{end+1} = sprintf('  %-40s %-15s %-17s %-12s %-7s %s', ...
        'check', 'book', 'computed', 'delta', 'verdict', 'source'); %#ok<AGROW>
    L{end+1} = ['  ' repmat('-', 1, W-2)]; %#ok<AGROW>
    for j = 1:numel(r)
        L{end+1} = sprintf('  %-40s %-15s %-17s %-12s %-7s %s', ...
            r(j).name, r(j).book, r(j).computed, r(j).delta, r(j).verdict, ...
            fitCite(r(j).source)); %#ok<AGROW>
    end
    if ~p.ok
        L{end+1} = sprintf('  ERROR: %s', oneLine(p.err)); %#ok<AGROW>
    end
end

L{end+1} = '';
L{end+1} = repmat('=', 1, W);
L{end+1} = sprintf(['TOTAL: %d scored, %d PASS, %d FAIL   (+%d SKIP, %d INFO, ' ...
                    '%d REFUTED -- none of those three is scored)'], ...
    S.n_scored, S.n_pass, S.n_fail, S.n_skip, S.n_info, S.n_refuted);
if S.n_fail == 0
    L{end+1} = 'VERDICT: the ported listings reproduce the book.';
else
    L{end+1} = sprintf(['VERDICT: %d FAILED.  Do NOT widen a band to clear one -- several of ' ...
                        'these rows compare'], S.n_fail);
    L{end+1} = '         against closed-form solutions of the ODE being integrated, so red means';
    L{end+1} = '         the port is wrong, not that the band was tight.';
end
if S.n_skip > 0
    L{end+1} = sprintf(['NOTE: %d row(s) SKIPPED.  A skip is not a pass -- it is a check that ' ...
                        'did not run.'], S.n_skip);
end
if S.n_unmapped > 0
    un = unique({rows(strcmp({rows.source}, UNMAPPED())).name});
    L{end+1} = sprintf(['NOTE: %d row(s) have no citation.  A port renamed a check and the ' ...
                        'figure reference'], S.n_unmapped);
    L{end+1} = sprintf('      in benchSuite.m''s cite() table no longer reaches it: %s', ...
        strjoin(un, '; '));
end

% -- timing -----------------------------------------------------------------
%   MEASURED, not claimed.  The third wave roughly quintupled the number of
%   integrations on this board, so "is it still fast" is a question the board
%   answers about itself on every run instead of one the header asserts once.
L{end+1} = '';
if ~isempty(ports)
    el  = [ports.elapsed_s];
    tot = sum(el);
    [se, ord] = sort(el, 'descend');
    nsl  = min(3, numel(ord));
    slow = arrayfun(@(k) sprintf('%s %.1f s', ports(ord(k)).key, se(k)), ...
                    1:nsl, 'UniformOutput', false);
    L{end+1} = sprintf('WALL CLOCK: %.1f s over %d port(s).  Slowest: %s.', ...
        tot, numel(ports), strjoin(slow, ', '));
    % The subset suggestion is computed from what THIS run cost, not from a
    % guess in a comment, and the dropped ports are NAMED.  A fast board is a
    % board that covers less; quoting one without saying which listings are
    % missing is how a suite quietly stops meaning anything.
    drop = false(1, numel(el));
    acc  = 0;
    for k = 1:numel(ord)
        if acc >= 0.6*tot, break; end
        drop(ord(k)) = true;
        acc = acc + se(k);
    end
    keep = {ports(~drop).key};
    if tot > 20 && any(drop) && ~isempty(keep)
        q = cellfun(@(k) ['''' k ''''], keep, 'UniformOutput', false);
        L{end+1} = sprintf('  FAST SUBSET: mgs.zarchan.benchSuite(''only'', {%s})', ...
            strjoin(q, ', '));
        L{end+1} = sprintf(['  costs about %.1f s and DROPS %s.  Say which ports ' ...
            'you skipped whenever you quote that board.'], tot - acc, ...
            strjoin({ports(drop).key}, ', '));
    end
end

% -- exclusions -------------------------------------------------------------
%   THE COUNT PRINTS EVEN WHEN IT IS ZERO, and so does the word NONE.  "No
%   port was excluded" and "the exclusion block was forgotten" must not look
%   the same on the page.
L{end+1} = '';
if isempty(X)
    exNames = 'NONE';
else
    exNames = strjoin({X.key}, ', ');
end
L{end+1} = sprintf('EXCLUDED (failed or unverified audit): %d port(s) -- %s', ...
    numel(X), exNames);
if isempty(X)
    L{end+1} = sprintf(['  All %d audited ports passed audit and all %d of them are on the ' ...
                        'board above.  The'], S.audited_total, numel(ports));
    L{end+1} = '  count above prints even when it is zero so that "nothing was excluded" and "the';
    L{end+1} = '  exclusion block was forgotten" cannot look identical.  To exclude a port,';
    L{end+1} = '  add it to';
    L{end+1} = '  the X table in benchSuite.m rather than deleting it from P -- an absent port and';
    L{end+1} = '  a sound port look the same from outside.';
    L{end+1} = '  Wave 1 (Listings 2.1, 2.2, 3.1, 6.1, 20.3):    0 excluded.';
    L{end+1} = '  Wave 2 (Listings 5.2, 5.3, 8.1, 8.2, 23.3):    0 excluded.';
    L{end+1} = '  Wave 3 (Listings 21.1, 21.2, 22.4, 23.1, 28.1): 0 excluded -- the';
    L{end+1} = '  FAILED/UNVERIFIED set came back empty for all three waves, so there are no';
    L{end+1} = '  names to print here.  That is the audit result, not a missing paragraph.';
else
    for i = 1:numel(X)
        L{end+1} = sprintf('  %-20s %-13s %s', X(i).key, X(i).listing, X(i).why); %#ok<AGROW>
        L{end+1} = sprintf('  %-34s breaks: %s', '', X(i).breaks); %#ok<AGROW>
    end
    L{end+1} = '  These are NOT on the board above.  Nothing they compute has been verified';
    L{end+1} = '  against the book, and a number taken from one is a number with no provenance.';
end
if ~isempty(opts.only)
    L{end+1} = sprintf(['  SUBSET RUN: ''only'' was set, so this board covers %d of %d wired ' ...
                        'ports.  It is not the whole board.'], numel(ports), S.audited_total);
end

% -- standing disagreements with the commissioning brief --------------------
L{end+1} = '';
L{end+1} = 'BRIEFED VALUES THAT DID NOT SURVIVE (carried as REFUTED/INFO, scored by nobody):';
L{end+1} = '  engage2d  N''=5 initial accel, briefed ~18 g -> 15.21 g.  n_c(0) = N''*Vc*lamdot is';
L{end+1} = '            EXACTLY linear in N'', and the same brief accepts 12.17 g at N''=4.';
L{end+1} = '  engage2d  3-g target peak, briefed ~28 g -> 5.45 g.  Zero-lag PN gives';
L{end+1} = '            n_c(tF) = nT*N''/(N''-1) = 4.5 g; 28 g at N''=3 needs LAG, i.e. Ch.3/Ch.6.';
L{end+1} = '  weave     the "60 -> 70 at N''=5, Fig. 20.12" benchmark is really the N''=3 ladder';
L{end+1} = '            off Fig. 20.10.  N''=5 at x=1 is 87.5 / 114.0 and the book scores it nowhere.';
L{end+1} = '';
L{end+1} = 'VALUES THE SECOND-WAVE PORTS REFUSED TO SCORE (their reasons are in their headers,';
L{end+1} = 'and the ones that made the board are INFO rows above -- none of them is counted):';
L{end+1} = '  Ch.5   the book''s 15 g is a Fig. 5.6 reading with no stated time-to-go, and the';
L{end+1} = '         adjoint DIVERGES as tgo -> 0, so accelMissAdjoint refuses it outright and';
L{end+1} = '         covarianceHoming scores only a BRACKET (the 483 ft/s^2 crossing must occur,';
L{end+1} = '         and only in the last half second).  Do not turn either into a value row.';
L{end+1} = '  Ch.23  Fig. 23.18 DOES NOT REPRODUCE: the port gets a 25-ft radome swing of 0.046';
L{end+1} = '         against the book''s 0.06 at tau = 0.5 s, because the rms miss at R = 0 is';
L{end+1} = '         already sitting on the 25 ft line there.  It scores the robust half of that';
L{end+1} = '         page instead and reports the disagreement rather than widening a band.';
L{end+1} = '  Ch.23  "at sea level only a few deg/s of fin rate are required": the port computes';
L{end+1} = '         17.3 deg/s and says so as an INFO row.  The numeric half of the same';
L{end+1} = '         sentence ("nearly 600 deg/s" at 50 kft) reproduces at 552.';
L{end+1} = '  Ch.8   Zarchan publishes NO closed form and no number for the lagged APN miss.';
L{end+1} = '         adjointSingleLagCh8 DERIVED one, and every row that uses it says DERIVED --';
L{end+1} = '         in the port header, in the row''s own .source, and in the source column above.';
L{end+1} = '';
L{end+1} = 'AND WHAT THE THIRD WAVE REFUSED (same rule: an adjective is not a benchmark):';
L{end+1} = '  Ch.21  "a near perfect match" between the linear and nonlinear acceleration is the';
L{end+1} = '         ONLY thing Ch.21 says about that comparison -- no percentage, no rms, no';
L{end+1} = '         peak error anywhere in the chapter.  airframeLinear scores the two';
L{end+1} = '         ORDERINGS the book does assert (worse at 10 deg than 5, worse at 50 kft';
L{end+1} = '         than sea level: 16.9% > 6.6% and 11.6% > 6.6%) and emits the adjective as';
L{end+1} = '         INFO carrying 6.60% rms.  It also refuses the wAF 10-30 rad/s band: the';
L{end+1} = '         endpoints are ROUND, our 9.95 IS the book''s own "10.0" before rounding, so';
L{end+1} = '         containment would test the rounding and fail a correct port by 0.5%.';
L{end+1} = '  Ch.21  "at sea level only a few deg/s of fin rate are required": the port computes';
L{end+1} = '         17.3 deg/s and DISAGREES rather than widening "a few" to include 17.';
L{end+1} = '  Ch.22  the 11.5 dB gain margin is the SAME bisection as the KUNSTABLE row expressed';
L{end+1} = '         logarithmically, so flightControlDelay scores it once and carries the other';
L{end+1} = '         as INFO.  Counting both would make this board look denser than it is.';
L{end+1} = '  Ch.23  the words "overshoot", "settling" and "rise time" DO NOT OCCUR anywhere in';
L{end+1} = '         Ch.23.  The 63% rise time is the only step-response characteristic the book';
L{end+1} = '         publishes, so there is no overshoot benchmark to have forgotten.';
L{end+1} = '  Ch.28  "the miss is reduced to virtually zero" is an adjective AND unscoreable';
L{end+1} = '         here anyway: the measured 0.731 ft sits AT the 0.8 ft terminal step quantum,';
L{end+1} = '         so any tighter tolerance would be scoring the step size, not the guidance';
L{end+1} = '         law.  The ORDERING (weave/PN <= 0.15, measured 0.039) is scored instead.';

% -- scope ------------------------------------------------------------------
L{end+1} = '';
L{end+1} = 'SCOPE -- what a green board does and does not mean:';
L{end+1} = '  DOES: our integrator and PN kernel reproduce published answers, and the Ch.3/Ch.6';
L{end+1} = '        lag-to-miss machinery is wired correctly.';
L{end+1} = '  DOES NOT: model this vehicle.  No gravity, no drag, no thrust, no asymmetric nz';
L{end+1} = '        clamp, no LOS floor, and -- the one that matters -- no SATURATING lag.  Our';
L{end+1} = '        measured gamma-tracking gain is 0.063 against the model''s 1.385, with ~8 s of';
L{end+1} = '        lag saturating on 26 of 32 sorties; every listing here has a linear first-order T.';
L{end+1} = '  DOES NOT: cover our failure channel.  Ch.6''s adjoint carries step/ramp/parabolic';
L{end+1} = '        TARGET maneuvers only; our discriminant is release-side heading error |eps0|,';
L{end+1} = '        an initial-condition channel that appears here only in Ch.3''s MHE/HE curve.';
L{end+1} = '  Ch.20 is a weaving TARGET, not a weaving pursuer.  It does not predict our spread.';
L{end+1} = '  A miss distance off this board is NOT a sortie prediction.';
L{end+1} = '';
L{end+1} = 'WHAT THE SECOND WAVE ADDS TO THAT SCOPE, AND WHAT IT STILL DOES NOT:';
L{end+1} = '  ADDS: Ch.5 prices miss DISPERSION per error source in one run instead of a Monte';
L{end+1} = '        Carlo, and covarianceHoming''s x0 takes a release-side initial condition --';
L{end+1} = '        R.x0_from_heading_error(vm, sigma) is the |eps0| channel our own discriminant';
L{end+1} = '        lives in, and it is the first thing on this board that can price it.';
L{end+1} = '  ADDS: Ch.23 is the only listing here that DERIVES its lag rather than assuming one,';
L{end+1} = '        and holds tau_TOT within 1.9% while Ma changes 6.5x across 50 kft.  Ours is';
L{end+1} = '        held constant by nothing.  That is a structural difference, not a vague one.';
L{end+1} = '  DOES NOT: buy the lag back.  Ch.8 augmented PN feeds forward TARGET acceleration and';
L{end+1} = '        our bullseye is stationary (nT = 0 makes the whole APN term vanish), and the';
L{end+1} = '        optimal law of Listing 8.2 is beaten by plain PN past 8 s of flight on a 0.9';
L{end+1} = '        time-to-go scale error -- our dive predictor''s measured error is ~1.4-1.5.';
L{end+1} = '  DOES NOT: price arrival spread.  covarianceHoming proves Vc CANCELS out of its own';
L{end+1} = '        loop to ~1e-14 (a structural row above), so every sigma in it is LOS-normal';
L{end+1} = '        and there is no along-track channel for a 3 s arrival window to live in.';
L{end+1} = '';
L{end+1} = 'WHAT THE THIRD WAVE ADDS, AND THE ONE QUESTION IT SETTLES:';
L{end+1} = '  Ch.21-23 is a chain, and the chain is the point.  Listing 21.1 flies the bare';
L{end+1} = '  airframe, 21.2 linearises it, 22.4 closes a rate-gyro loop round it and 23.1 sets';
L{end+1} = '  that loop''s time constant from a design goal.  Every lag on this board below comes';
L{end+1} = '  out of GEOMETRY, MASS AND DYNAMIC PRESSURE -- not out of a curve fit:';
L{end+1} = '        1/wAF        0.040 s (sea level) .. 0.104 s (50 kft)   body dynamics';
L{end+1} = '        1/(zAF*wAF)  0.79 s   how long that transient is still ringing, 20x longer';
L{end+1} = '        Ta           0.56 s .. 3.78 s      FLIGHT-PATH lag, alpha/gammadot, ~1/Q';
L{end+1} = '        tau_TOT      0.34 s   the closed-loop answer, held constant by the autopilot';
L{end+1} = '  Our own ~8 s of gamma-tracking lag is 200x the airframe constant, so whatever it';
L{end+1} = '  is, it is NOT body dynamics.  It is the same ORDER as Ta, which is an altitude and';
L{end+1} = '  dynamic-pressure quantity rather than a gain-tuning one.';
L{end+1} = '  DOES NOT: hand us a number for our vehicle.  This is Mach 3 supersonic linear-';
L{end+1} = '        theory aero on a 1000 lb finned missile and ours is a subsonic jet;';
L{end+1} = '        airframeSim ERRORS below Mach 1 rather than returning the complex numbers';
L{end+1} = '        sqrt(M^2-1) would give.  What transfers is the METHOD -- Ta = alpha/gammadot';
L{end+1} = '        measured off a run and checked against Md/(Ma*Zd-Md*Za) -- not the value.';
L{end+1} = '  ADDS: the delay/lag distinction, measured.  A 0.0147 s pure DELAY destabilises the';
L{end+1} = '        Ch.22 loop; a first-order LAG of the same constant in the same slot leaves it';
L{end+1} = '        stable, and so does one at 10x and 100x.  Our 8 s was fitted as a tracking';
L{end+1} = '        gain plus a settling time, which cannot tell those two apart.';
L{end+1} = '  ADDS: engage3d is the only 3-D check of the PPN kernel we actually fly, and its';
L{end+1} = '        warning is bigger than its benchmark -- against a circularly evading target';
L{end+1} = '        the single-plane miss OSCILLATES with flight time while the 3-D total is';
L{end+1} = '        CONSTANT and equal to the single-plane PEAK.  Every miss and spread statistic';
L{end+1} = '        this project has published is computed in one plane, i.e. is one phase sample';
L{end+1} = '        of that circle.';
L{end+1} = '  DOES NOT: cover a stationary target.  Ch.28''s only error source is the target';
L{end+1} = '        maneuver (its own delete experiment proves it: aT = 0 collapses the miss to';
L{end+1} = '        one step quantum).  Our bullseye does not move, so the miss mechanism this';
L{end+1} = '        chapter prices is one we do not have.';
s = strjoin(L, newline);
end
