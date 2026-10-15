function R = kalmanLawAdjoint(opts)
%MGS.ZARCHAN.KALMANLAWADJOINT  Zarchan Listing 29.4 -- ADJOINT of the homing
%   loop with a three-state discrete Kalman filter and three guidance-law
%   options, plus its Listing 29.3 forward twin (the only thing that scores it).
%
%   R = mgs.zarchan.kalmanLawAdjoint()
%   R = mgs.zarchan.kalmanLawAdjoint('APN', 2)          optimal guidance
%   R = mgs.zarchan.kalmanLawAdjoint('forward', false)  adjoint only, UNSCORED
%   R = mgs.zarchan.kalmanLawAdjoint('assert', false)   warn instead of error
%
%   The book misspells its own listing title -- "LISTING 29.4 AJOINT OF HOMING
%   LOOP ..." (zarchan_decoded.txt:10980 = zarchan.txt:10793).  Spelled
%   correctly here; nothing else was "corrected" on the strength of that.
%
% ===========================================================================
%   UNITS ARE THE BOOK'S: FEET, ft/s, ft/s^2, s.  g = 32.2, so the listing's
%   XNT = 96.6 IS the 3-g target maneuver the prose names.  SIGNOISE is in
%   RADIANS; HEDEGF is in DEGREES and the listing converts with the book's
%   57.3, not pi/180.  Nothing is converted to SI -- convert AT THE CALL SITE.
% ===========================================================================
%
%   *** WHAT IT IS NOT. ***  A LINEARISED, PLANAR homing loop: constant closing
%   velocity, one first-order flight-control lag (TAP = 0.5 s), no gravity, no
%   drag, no acceleration limit, no LOS floor, and Kalman gains precomputed for
%   a FIXED 10 s flight.  The one disturbance is a step target maneuver.  Do not
%   read our spread problem out of it: the adjoint identity is a LINEARITY
%   identity and our dominant failure (gamma tracking) SATURATES.
%
% ---------------------------------------------------------------------------
%   WHY LISTING 29.3 IS ALSO IN THIS FILE
%
%   Chapter 29 publishes NO number for 29.4's output.  Figs. 29.9-29.11 became
%   image links (decoded:11022, :11052, :11054); only axis furniture survived,
%   and its tick ORDER is top-to-bottom in two figures and bottom-to-top in the
%   third, so no curve value is recoverable.  The chapter's ONE published claim
%   about 29.4 is a COMPARISON: "exact agreement between the adjoint and forward
%   simulations in all cases" (decoded:11007 + :11024 = raw:10820 + :10837).
%   Without the forward model this port would adjudicate nothing at all.
%
%   ⚠️⚠️ CORRECTED.  This header used to say the two book files "do not share
%   line numbering: 15120 vs 15307".  That is wrong, and so is the opposite
%   claim that later reviewers made about THIS file.  Measured:
%
%     * zarchan.txt and zarchan_decoded.txt ARE aligned.  Both hold 15120 LF
%       records and 15307 CR records, and the same sentence sits at the same
%       index in both.  A line number cited from one resolves in the other.
%     * The +187 is ONE FILE READ TWO WAYS.  grep / sed / the agent Read tool
%       split on LF and see 15120 lines; MATLAB readlines() splits on a bare CR
%       as well and sees 15308.  The gap grows through the file as the 187 bare
%       CRs accumulate: 0 at the front, +36 by Ch.7, +187 by Ch.29.
%     * THIS FILE'S CITATIONS ARE IN THE grep CONVENTION and are correct.
%       Verified by inspection: :10793 lands on "LISTING 29.4 AJOINT OF HOMING
%       LOOP ...", :2526 on "The effective navigation ratio for the optimal
%       guidance law is displayed in ...".  Converting them "to grep" would
%       move them onto an image link and a page number.  Do not convert them.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION (matlab/zarchan/_listings/listing_29_04.m; it does not
%   parse).  Seven repairs:
%
%     1  "while TP ,= (TF-1e-5)" -> "<=".  OCR renders "<" as ",".  TP only
%        increases, so ">=" never terminates.
%     2  "if S.(TS-.0001)" -> "if S > (TS-.0001)".  "." is the OCR of ">".
%        SHAPE-FORCED: S resets to 0 on every record, so "<" would record on
%        EVERY step and the gain index I would run to 1000 against a
%        100-element array -- an index error, not a wrong answer.
%     3  Listing 29.3's "if T.TSTART" -> "if T > TSTART": the maneuver STARTS
%        at TSTART and stays on, NOT stops.  Two reasons: decoded:10947 says
%        "the time to go at which the 3-g target maneuver BEGINS is made a
%        parameter", and the same passage describes the listing's own
%        TGOS = TF-TSTART as "varying between 9.9 and 0 s" over
%        TSTART = 0.1..10 -- which is time-to-go-at-ONSET only if onset is at
%        TSTART.
%     4  THE JOINED COMMENT.  Extraction produced
%           end; % Modification % ----------------------ICOUNT=round((TF/TS));
%        Taken literally the "%" swallows ICOUNT=round(TF/TS) and E/F/G are
%        never sized.  In the book these are separate lines: banner, dashed
%        rule, then live code.  Un-joined.  Same for "% ------for I=1:1:ICOUNT".
%     5  The bare "[TP I E(I) TS]" is an un-suppressed display statement, a
%        debugging leftover computing nothing.  Behind 'trace', default false.
%     6  "save datfil.txt output /ascii" is the book's recurring typo for
%        -ascii.  Nothing is written to disk; recorded and dropped.
%     7  Everything from "The nominal cases of Listing 29.3..." on is running
%        prose leaking into the transcription (it carries the one un-decoded
%        "APN 1/4 0" ligature).  Dropped, not "fixed" into code.
%
%   THE "% Modification" IS LOAD-BEARING AND IT DROPS A GAIN.  ICOUNT is
%   round(TF/TS) = 100, E/F/G are sized 100 and zeroed, ICOUNT is DECREMENTED
%   to 99, and the reversal writes E(99..1) = U(1..99).  So U(100) -- the gain
%   at tgo -> 0, where K1 -> 1 -- is DISCARDED and E(100) stays 0.  E(100) is
%   read once, at the last adjoint sample, where the gains feed Y-states never
%   used again.  Reproduced exactly; SELFCHECK 2 pins it against a "tidy-up".
%
%   Three quirks reproduced deliberately, each noted again at its site:
%     * 29.3's "else YDD = 0." drops the -XNL term too.  Harmless HERE and only
%       here: before onset the pseudo-measurement is identically zero, so the
%       filter states, XNC and XNL are all exactly 0.
%     * The optimal-gain denominator carries a "+.0001" the book's printed
%       formula lacks.  Not cosmetic -- BENCHMARK 2 measures it at -0.055% at
%       x = 1, and it grows as x -> 0.
%     * XLAM is formed with tgo+1e-5 and then multiplied at the sampler by an
%       RTM formed with tgo+1e-6.  The book's mismatch; it bites only on the
%       final sample.  Computed-but-never-read elsewhere: Y5OLD, Y8NEW, Y5NEW,
%       XLAMD, XLAMDH, XNEW, 29.3's inner-loop RTM.  Dropped, named here.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS -- every book value grepped; .source names the file AND the line.
%   Nothing was read off a figure.
%
%   1  XNT/g = 3.  Listing line 1 (decoded:10981) vs "3-g target maneuver"
%      (decoded:10947).  Exact.  SKIPPED, not failed, if XNT is off-book.
%   2  |C4| at x = tgo/T = 1.  Book prints C4 = N'(e^-x + x - 1)/x^2
%      (decoded:2576 = raw:2540) as a MAGNITUDE and carries the minus in the
%      zero-effort-miss bracket "[y + y'tgo + 0.5 nT tgo^2 - nL T^2(...)]"
%      (decoded:2556-2559 = raw:2520-2523); both listings fold that minus into
%      C4.  Band 0.2%, sized for the +.0001 guard alone (-0.055% here).  This
%      row catches a botched T^2/tgo^2 -> 1/x^2 substitution.
%   3  N' -> 3 for long time to go.  Prose, decoded:2562 = raw:2526; formula at
%      decoded:2561.  Scored at x = 1000, band 1%: the limit is approached like
%      3(1+2/x), so x = 1000 gives 3.006 (+0.20%) and x = 10 would not.
%   4-6  ADJOINT vs FORWARD, one row per guidance law (decoded:11007, :11024).
%      The book says "exact"; two independently discretised second-order
%      integrations of one loop are not bit-equal, so the band is a MECHANISM:
%
%          allow = max( 5% of the forward peak,  2*H*max|d(miss)/d(tgo)| )
%
%      The second term is maneuver-onset jitter -- the forward switches on at
%      the first step whose ACCUMULATED T exceeds TSTART, and accumulating 0.01
%      puts that boundary on either side of the nominal instant, so the two
%      schemes can differ by one step of onset time.  Both terms print on the
%      row.  The faults this exists to catch (unreversed gains, a dropped
%      adjoint path) move the curve by O(100%).  These are also the ONLY rows
%      valid at off-book parameters.
%   7  ORDERING: "the miss distance results get better as the guidance law
%      becomes more advanced" (decoded:11024), scored as peak|miss|
%      PN >= APN >= optimal.  In TENSION with the extracted axis ranges --
%      Fig. 29.9 runs 0..20 but Fig. 29.10 runs -20..10, a WIDER extent.  That
%      tension is reported in the INFO row and was NOT used to weaken this one.
%      If this row comes back red it is a finding about the book.
%
%   NOT SCORED: a SKIP row for the plotted curves themselves (image links), and
%   an INFO row carrying the three axis tick sequences beside our peaks.
%
% ---------------------------------------------------------------------------
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED.  THIS FILE HAS NEVER
%   BEEN EXECUTED -- MATLAB IS THE CALLER'S. ***
%     BENCHMARK 1: 96.6/32.2 = 3.0000, exact.
%     BENCHMARK 2: book 4.5252, ours 4.5227 (-0.055%), band 0.2%.
%     BENCHMARK 3: 3.0060 at x = 1000 (+0.20%), band 1%.
%     BENCHMARKS 4-6: the miss MAGNITUDES are NOT derivable here; the first run
%       establishes them.  Fig. 29.9's axis says the PN peak is under 20 ft and
%       Fig. 29.11's that the optimal peak is a few feet.  A residual of a few
%       tenths of a foot is what the onset-jitter mechanism predicts; a residual
%       comparable to the peak means the reversal or a path is wrong.
%     100 Kalman gains, 100 adjoint samples per case, K1 -> 1 as tgo -> 0.
%
% ---------------------------------------------------------------------------
%   OUTPUTS
%     R.gains      .U .V .W (forward order), .E .F .G (reversed, as used)
%     R.cases      1x3 struct: .APN .name .tp .miss .k1 and, when the forward
%                  ran, .fwd_tgo .fwd_miss .adj_at .resid .allow .peak .slope
%     R.tp R.miss  the caller's own APN case
%     R.bench      benchmark rows, schema shared with the other ports
%     R.selfcheck  .names/.oks (SAME LENGTH -- benchSuite needs BOTH or the
%                  invariants never reach the board), .all_ok, .first_bad,
%                  .detail, .notrun
%     R.pass       true only if at least one row was SCORED and all passed
%     R.verdict    '' when everything passed, else the one-line reason.  ALWAYS
%                  FORMED.  'assert' picks the reaction and nothing else: true
%                  errors, false WARNS and still returns R complete.
%     R.units      one line naming the units;  R.p  parameters actually used;
%     R.summary    the rendered text block
%
%   See also MGS.ZARCHAN.SHAPINGFILTERADJOINT, MGS.ZARCHAN.ADJOINTOPTIMAL,
%   MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 29.4 line 1, verbatim (decoded:10981).  TAP is set later in the
    % listing; Listing 29.3 calls the same 0.5 s constant TAU.
    opts.XNT      (1,1) double {mustBeFinite, mustBePositive} = 96.6   % ft/s^2 = 3 g
    opts.XNP      (1,1) double {mustBeFinite, mustBePositive} = 3.0
    opts.TAP      (1,1) double {mustBeFinite, mustBePositive} = 0.5    % s
    opts.TF       (1,1) double {mustBeFinite, mustBePositive} = 10.0   % s, FIXED
    opts.TS       (1,1) double {mustBeFinite, mustBePositive} = 0.1    % s, sampler
    opts.H        (1,1) double {mustBeFinite, mustBePositive} = 0.01   % s, RK2 step
    opts.VC       (1,1) double {mustBeFinite, mustBePositive} = 4000.0 % ft/s
    opts.VM       (1,1) double {mustBeFinite, mustBePositive} = 3000.0 % ft/s
    opts.HEDEGF   (1,1) double {mustBeFinite} = 20.0                   % deg
    opts.SIGNOISE (1,1) double {mustBeFinite, mustBePositive} = 0.001  % rad
    % 0 = PN, 1 = augmented PN, 2 = optimal.  This picks the case returned in
    % R.tp/R.miss; the benchmarks run all three regardless.
    opts.APN      (1,1) double {mustBeMember(opts.APN, [0 1 2])} = 0
    % Times-to-go at which the forward model starts the maneuver.  [] = 1:9 s.
    % The book sweeps 0.1:0.1:9.9 (100 runs); nine points is enough to
    % adjudicate agreement and costs a ninth as much.
    opts.fwd_tgo  double {mustBeReal, mustBeFinite} = []
    % THE ONLY SWITCHES.  Not nargin: with an arguments block declaring only
    % name-value options MATLAB reports nargin = 0 however many were passed.
    opts.forward   (1,1) logical = true
    opts.trace     (1,1) logical = false   % the listing's bare "[TP I E(I) TS]"
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

P = opts;
if abs(P.TS/P.H - round(P.TS/P.H)) > 1e-9 || round(P.TS/P.H) < 2
    error('mgs:zarchan:kalmanLawAdjoint:step', ...
        ['TS/H = %g.  The listing samples by accumulating H into S and ' ...
         'testing S > TS-1e-4, so a non-integer ratio makes the sampler ' ...
         'fire irregularly while the program still runs smoothly.'], P.TS/P.H);
end
if abs(P.TF/P.TS - round(P.TF/P.TS)) > 1e-9
    error('mgs:zarchan:kalmanLawAdjoint:grid', ...
        ['TF/TS = %g is not an integer.  ICOUNT = round(TF/TS) sizes the ' ...
         'reversed gain array, and a mismatch there indexes past its end.'], ...
        P.TF/P.TS);
end
if P.H >= P.TAP
    error('mgs:zarchan:kalmanLawAdjoint:stiff', ...
        ['H = %g s is not smaller than the flight-control time constant ' ...
         'TAP = %g s.  RK2 does not blow up here, it returns a smooth and ' ...
         'entirely wrong curve, so this is refused rather than warned about.'], ...
        P.H, P.TAP);
end

fwd_tgo = opts.fwd_tgo(:).';
if isempty(fwd_tgo), fwd_tgo = 1:floor(P.TF - 1); end
fwd_tgo = fwd_tgo(fwd_tgo > 0 & fwd_tgo < P.TF);
if opts.forward && isempty(fwd_tgo)
    error('mgs:zarchan:kalmanLawAdjoint:fwdtgo', ...
        ['no forward comparison points survive: fwd_tgo must lie in (0, TF) ' ...
         'and TF = %g s.  Running the adjoint alone and reporting a pass ' ...
         'would drop the only published claim about Listing 29.4.'], P.TF);
end

% =========================================================================
%   Riccati sweep and the "% Modification" reversal.  Listing 29.4,
%   decoded:10981-10984 (Riccati) and :10985-10986 (the reversal).
% =========================================================================
[U, V, W] = riccatiGains(P.TF, P.TS, P.VC, P.VM, P.XNT, P.HEDEGF, P.SIGNOISE);
ICOUNT = round(P.TF/P.TS);
E = zeros(1, ICOUNT);  F = E;  G = E;      % the listing's E(ICOUNT)=0 etc.
ICOUNT = ICOUNT - 1;                       % <<< the "% Modification"
for I = 1:ICOUNT
    REV = ICOUNT - I + 1;
    E(REV) = U(I);  F(REV) = V(I);  G(REV) = W(I);
end

% =========================================================================
%   The three guidance-law cases.
% =========================================================================
names = {'PN (APN=0)', 'augmented PN (APN=1)', 'optimal (APN=2)'};
C = struct('APN', {0,1,2}, 'name', names, 'tp', {[],[],[]}, 'miss', {[],[],[]}, ...
           'k1', {[],[],[]}, 'fwd_tgo', {[],[],[]}, 'fwd_miss', {[],[],[]}, ...
           'adj_at', {[],[],[]}, 'resid', {NaN,NaN,NaN}, 'allow', {NaN,NaN,NaN}, ...
           'peak', {NaN,NaN,NaN}, 'slope', {NaN,NaN,NaN});
fwdK = [];
for c = 1:3
    [C(c).tp, C(c).miss, C(c).k1] = adjointRun(C(c).APN, E, F, G, P.XNT, ...
        P.XNP, P.TAP, P.TF, P.TS, P.VC, P.H, opts.trace);
    if opts.forward
        m = zeros(size(fwd_tgo));
        for k = 1:numel(fwd_tgo)
            [m(k), Kf] = forwardRun(C(c).APN, P.TF - fwd_tgo(k), P.XNT, P.XNP, ...
                P.TAP, P.TF, P.TS, P.VC, P.VM, P.HEDEGF, P.SIGNOISE, P.H);
            % One capture is enough for SELFCHECK 4: the Riccati recursion is
            % driven only by TF/TS/VC/SIGNOISE, so it is bit-identical across
            % every APN and every TSTART.  The book says as much
            % (decoded:10947: solving for the gains each run "is not necessary
            % as the flight time is fixed").
            if c == 1 && k == 1, fwdK = Kf; end
        end
        C(c).fwd_tgo = fwd_tgo;  C(c).fwd_miss = m;
        [C(c).resid, C(c).allow, C(c).peak, C(c).slope, C(c).adj_at] = ...
            agreeMetric(C(c).tp, C(c).miss, fwd_tgo, m, P.H);
    end
end

R = struct();
R.gains = struct('U',U, 'V',V, 'W',W, 'E',E, 'F',F, 'G',G);
R.cases = C;
main    = find([C.APN] == P.APN, 1);
R.tp    = C(main).tp;
R.miss  = C(main).miss;
R.units = ['ft, ft/s, ft/s^2, s; g = 32.2 ft/s^2 (XNT = 96.6 = 3 g); ' ...
           'LOS noise in rad; heading error in deg via the book''s 57.3'];
R.p = struct('XNT',P.XNT, 'XNP',P.XNP, 'TAP',P.TAP, 'TF',P.TF, 'TS',P.TS, ...
             'H',P.H, 'VC',P.VC, 'VM',P.VM, 'HEDEGF',P.HEDEGF, ...
             'SIGNOISE',P.SIGNOISE, 'APN',P.APN, 'fwd_tgo',fwd_tgo, ...
             'forward',opts.forward, 'n_samples',numel(C(1).tp), ...
             'n_gains',numel(U), 'units',R.units);

R.selfcheck = structuralChecks(U, V, W, E, F, G, fwdK, C, P, opts.forward);

if opts.benchmark
    R.bench = benchmarks(C, P, opts.forward);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false(1,0);
    % NOT true.  Nothing was adjudicated, and "not checked" must never render
    % as "fine".
    R.pass  = false;
end

R.summary = render(R, C, P, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot,   drawCurves(C, P); end

msg = '';
if ~opts.benchmark
    msg = '''benchmark'' is false: nothing was compared. R.pass = NOT CHECKED.';
elseif ~any(ev)
    msg = ['not one book value could be evaluated -- with ''forward'' false ' ...
           'the only published claim about Listing 29.4 (agreement with 29.3) ' ...
           'cannot be tested, and an unvalidated port must not be handed back ' ...
           'silently.'];
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
        error('mgs:zarchan:kalmanLawAdjoint:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN.  R comes back complete so the caller can see WHICH
        % side moved; benchSuite runs every port with assert off for that.
        warning('mgs:zarchan:kalmanLawAdjoint:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [U, V, W] = riccatiGains(TF, TS, VC, VM, XNT, HEDEGF, SIGNOISE)
%RICCATIGAINS  Listing 29.4's first loop, decoded:10981-10984.
%   Every constant arrives as an ARGUMENT: MATLAB local functions cannot see
%   the main function's workspace.
TS2 = TS*TS;  TS3 = TS2*TS;  TS4 = TS3*TS;  TS5 = TS4*TS;
PHIN = XNT*XNT/TF;
RTM = VC*TF;  SIGPOS = RTM*SIGNOISE;  SIGN2 = SIGPOS^2;
P11 = SIGN2;  P12 = 0;  P13 = 0;
P22 = (VM*HEDEGF/57.3)^2;  P23 = 0;  P33 = XNT*XNT;

n = round(TF/TS);
U = zeros(1,n);  V = zeros(1,n);  W = zeros(1,n);
c = 0;
for T = TS:TS:TF
    TGO = TF - T + 0.000001;  RTM = VC*TGO;
    SIGPOS = RTM*SIGNOISE;  SIGN2 = SIGPOS^2;
    M11 = P11 + TS*P12 + .5*TS2*P13 + TS*(P12 + TS*P22 + .5*TS2*P23);
    M11 = M11 + .5*TS2*(P13 + TS*P23 + .5*TS2*P33) + TS5*PHIN/20;
    M12 = P12 + TS*P22 + .5*TS2*P23 + TS*(P13 + TS*P23 + .5*TS2*P33) + TS4*PHIN/8;
    M13 = P13 + TS*P23 + .5*TS2*P33 + PHIN*TS3/6;
    M22 = P22 + TS*P23 + TS*(P23 + TS*P33) + PHIN*TS3/3;
    M23 = P23 + TS*P33 + .5*TS2*PHIN;
    M33 = P33 + PHIN*TS;
    K1 = M11/(M11 + SIGN2);  K2 = M12/(M11 + SIGN2);  K3 = M13/(M11 + SIGN2);
    P11 = (1 - K1)*M11;  P12 = (1 - K1)*M12;  P13 = (1 - K1)*M13;
    P22 = -K2*M12 + M22; P23 = -K2*M13 + M23; P33 = -K3*M13 + M33;
    c = c + 1;  U(c) = K1;  V(c) = K2;  W(c) = K3;
end
U = U(1:c);  V = V(1:c);  W = W(1:c);
end


% =============================================================================
function [C1, C2, C3, C4] = controlGains(APN, XNP, TGO, TAP)
%CONTROLGAINS  The C1..C4 block shared by both listings.  In the ADJOINT the
%   argument is TP (adjoint time IS time to go); in the FORWARD it is TGO.
%
%   The "+.0001" in the optimal denominator is the LISTING's, not the book's
%   printed formula (decoded:2561).  BENCHMARK 2 measures what it costs.
%   C4 is NEGATIVE here.  The book prints C4 = N'(e^-x + x - 1)/x^2
%   (decoded:2576) as a magnitude and carries the minus in the zero-effort-miss
%   bracket at decoded:2556-2559.  Both listings fold it into C4.
if APN == 0
    C1 = XNP/TGO^2;  C2 = XNP/TGO;  C3 = 0;        C4 = 0;
elseif APN == 1
    C1 = XNP/TGO^2;  C2 = XNP/TGO;  C3 = .5*XNP;   C4 = 0;
else
    X = TGO/TAP;
    TOP  = 6*X*X*(exp(-X) - 1 + X);
    BOT1 = 2*X*X*X + 3 + 6*X - 6*X*X;
    BOT2 = -12*X*exp(-X) - 3*exp(-2*X);
    XNPP = TOP/(.0001 + BOT1 + BOT2);
    C1 = XNPP/TGO^2;  C2 = XNPP/TGO;  C3 = .5*XNPP;
    C4 = -XNPP*(exp(-X) + X - 1)/(X*X);
end
end


% =============================================================================
function [tp, miss, k1] = adjointRun(APN, E, F, G, XNT, XNP, TAP, TF, TS, VC, H, trace)
%ADJOINTRUN  Listing 29.4's second half, decoded:10987-11006.
%
%   Adjoint time TP is the TIME TO GO AT WHICH the step target maneuver starts,
%   for a FIXED TF flight -- not flight time.  That is forced by the Kalman
%   gains: they depend on TF, so TF cannot also be the sweep variable
%   (decoded:10973).
T = 0;  S = 0;  TP = T + 0.00001;
X1 = 0;  X2 = 0;  X3 = 1;  X4 = 0;  X5 = 0;  X6 = 0;   % <<< THE IMPULSE on X3
Y1OLD = 0;  Y2OLD = 0;  Y3OLD = 0;  Y4OLD = 0;  Y6OLD = 0;  Y7OLD = 0;
Y1NEW = 0;  Y7NEW = 0;
X1D = 0;  X2D = 0;  X3D = 0;  X4D = 0;  X5D = 0;  X6D = 0;
I = 1;

nmax = round(TF/TS) + 2;
tp = zeros(nmax,1);  miss = zeros(nmax,1);  k1 = zeros(nmax,1);
count = 0;

while TP <= (TF - 1e-5)
    X1OLD = X1;  X2OLD = X2;  X3OLD = X3;
    X4OLD = X4;  X5OLD = X5;  X6OLD = X6;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop: pass 1 at the current state, pass 2
    % after a full Euler step with TP advanced by H.  TP advances INSIDE this
    % loop, exactly once; move that line out and the scheme degrades to first
    % order while still running smoothly.
    while STEP <= 1
        if FLAG == 1
            X1 = X1 + H*X1D;  X2 = X2 + H*X2D;  X3 = X3 + H*X3D;
            X4 = X4 + H*X4D;  X5 = X5 + H*X5D;  X6 = X6 + H*X6D;
            TP = TP + H;  STEP = 2;
        end
        TGO = TP;
        X1D = X2;
        X2D = X3 + Y1NEW/(VC*TGO);
        X3D = Y1NEW/(VC*TGO*TGO);
        X4D = (X5 + Y7NEW + X6)/TAP;
        X5D = -X4D;
        X6D = -X2;
        FLAG = 1;
    end

    % Heun corrector, in the listing's own algebra.
    X1 = (X1OLD + X1)/2 + .5*H*X1D;
    X2 = (X2OLD + X2)/2 + .5*H*X2D;
    X3 = (X3OLD + X3)/2 + .5*H*X3D;
    X4 = (X4OLD + X4)/2 + .5*H*X4D;
    X5 = (X5OLD + X5)/2 + .5*H*X5D;
    X6 = (X6OLD + X6)/2 + .5*H*X6D;

    S = S + H;
    if S > (TS - .0001)
        S = 0;
        if I > numel(E)
            error('mgs:zarchan:kalmanLawAdjoint:index', ...
                ['the sampler fired %d times against a %d-element reversed ' ...
                 'gain array.  The listing relies on TF/TS samples exactly; ' ...
                 'this is refused rather than clamped, because a clamped ' ...
                 'gain would return a smooth wrong curve.'], I, numel(E));
        end
        K1 = E(I);  K2 = F(I);  K3 = G(I);
        if trace, disp([TP I E(I) TS]); end   % the listing's bare display line
        I = I + 1;

        [C1, C2, C3, C4] = controlGains(APN, XNP, TP, TAP);

        TEMP1 = X4 - Y6OLD;
        TEMP2 = C1*TEMP1 + Y2OLD;
        TEMP3 = C2*TEMP1 + Y3OLD;
        TEMP4 = C3*TEMP1 + Y4OLD;
        TEMP5 = K1*TEMP2 + K2*TEMP3 + K3*TEMP4;
        Y1NEW = Y1OLD + TEMP5*VC*TP;
        Y2NEW = TEMP2 - TEMP5;
        Y3NEW = TEMP3 + TS*Y2NEW;
        Y4NEW = TEMP4 + TS*TEMP3 + .5*TS*TS*Y2NEW;
        Y5    = -(TS*TEMP3 + .5*TS*TS*Y2NEW);
        Y7NEW = Y7OLD + C4*TEMP1 + Y5;
        Y6NEW = X4;
        XMNT  = XNT*X1;                       % <<< THE ANSWER, in feet
        Y1OLD = Y1NEW;  Y2OLD = Y2NEW;  Y3OLD = Y3NEW;
        Y4OLD = Y4NEW;  Y6OLD = Y6NEW;  Y7OLD = Y7NEW;

        count = count + 1;
        tp(count) = TP;  miss(count) = XMNT;  k1(count) = K1;
    end
end
tp = tp(1:count);  miss = miss(1:count);  k1 = k1(1:count);
end


% =============================================================================
function [Y, Kseq] = forwardRun(APN, TSTART, XNT, XNP, TAU, TF, TS, VC, VM, ...
                                HEDEGF, SIGNOISE, H)
%FORWARDRUN  Listing 29.3, decoded:10953-10971, for ONE maneuver start time.
%   Returns the miss Y at t = TF and the Kalman gain sequence it used (so the
%   standalone Riccati loop of 29.4 can be checked against it).
%
%   "if T > TSTART" -- the maneuver STARTS at TSTART; see repair 3 in the
%   header for the two reasons that direction is the right one.
TS2 = TS*TS;  TS3 = TS2*TS;  TS4 = TS3*TS;  TS5 = TS4*TS;
PHIN = XNT*XNT/TF;
Y = 0;  YD = 0;  T = 0;  S = 0;
RTM = VC*TF;  SIGPOS = RTM*SIGNOISE;  SIGN2 = SIGPOS^2;
P11 = SIGN2;  P12 = 0;  P13 = 0;
P22 = (VM*HEDEGF/57.3)^2;  P23 = 0;  P33 = XNT*XNT;
YH = 0;  YDH = 0;  XNTH = 0;  XNC = 0;  XNL = 0;
YDD = 0;  XNLD = 0;  XLAM = 0;
Kseq = zeros(round(TF/TS), 3);  nK = 0;

while T <= (TF - 1e-5)
    YOLD = Y;  YDOLD = YD;  XNLOLD = XNL;
    STEP = 1;  FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            Y = Y + H*YD;  YD = YD + H*YDD;  XNL = XNL + H*XNLD;
            T = T + H;  STEP = 2;
        end
        % The listing also forms RTM = VC*TGO and XLAMD = (RTM*YD+Y*VC)/RTM^2
        % here.  XLAMD is NEVER READ -- the guidance is driven by the filter's
        % YH/YDH, not by the raw LOS rate -- and this RTM is shadowed by the
        % sampler's own, which uses +.000001 where this one uses +.00001.  Both
        % dropped; the +1e-5 is KEPT on XLAM because XLAM IS carried into the
        % sampler and multiplied there by the +1e-6 RTM.  That mismatch is the
        % book's, it only bites on the final sample (tgo ~ 1e-5, where the
        % command has no time left to act), and "re-basing" XLAM would be a
        % silent change of model dressed up as a tidy-up.
        TGO = TF - T + .00001;
        XLAM = Y/(VC*TGO);
        XNLD = (XNC - XNL)/TAU;
        if T > TSTART
            YDD = XNT - XNL;
        else
            % The listing's own "else YDD = 0." -- it drops -XNL too.  Harmless
            % here and only here: before onset the pseudo-measurement is
            % identically zero, so YH, YDH, XNTH, XNC and XNL are all exactly 0.
            YDD = 0;
        end
        FLAG = 1;
    end
    Y   = .5*(YOLD + Y + H*YD);
    YD  = .5*(YDOLD + YD + H*YDD);
    XNL = .5*(XNLOLD + XNL + H*XNLD);

    S = S + H;
    if S >= (TS - 1e-5)
        S = 0;
        TGO = TF - T + .000001;  RTM = VC*TGO;
        SIGPOS = RTM*SIGNOISE;  SIGN2 = SIGPOS^2;
        M11 = P11 + TS*P12 + .5*TS2*P13 + TS*(P12 + TS*P22 + .5*TS2*P23);
        M11 = M11 + .5*TS2*(P13 + TS*P23 + .5*TS2*P33) + TS5*PHIN/20;
        M12 = P12 + TS*P22 + .5*TS2*P23 + TS*(P13 + TS*P23 + .5*TS2*P33) + TS4*PHIN/8;
        M13 = P13 + TS*P23 + .5*TS2*P33 + PHIN*TS3/6;
        M22 = P22 + TS*P23 + TS*(P23 + TS*P33) + PHIN*TS3/3;
        M23 = P23 + TS*P33 + .5*TS2*PHIN;
        M33 = P33 + PHIN*TS;
        K1 = M11/(M11 + SIGN2);  K2 = M12/(M11 + SIGN2);  K3 = M13/(M11 + SIGN2);
        P11 = (1 - K1)*M11;  P12 = (1 - K1)*M12;  P13 = (1 - K1)*M13;
        P22 = -K2*M12 + M22; P23 = -K2*M13 + M23; P33 = -K3*M13 + M33;
        nK = nK + 1;  Kseq(nK,:) = [K1 K2 K3];

        XLAMNOISE = 0;                         % noise-free run, as in the book
        YSTAR = RTM*(XLAM + XLAMNOISE);
        RES  = YSTAR - YH - TS*YDH - .5*TS*TS*(XNTH - XNL);
        YH   = K1*RES + YH + TS*YDH + .5*TS*TS*(XNTH - XNL);
        YDH  = K2*RES + YDH + TS*(XNTH - XNL);
        XNTH = K3*RES + XNTH;
        % The listing also forms XLAMDH here and, in the APN==2 branch, XNEW.
        % Neither is read by anything.  Dropped; named in the header.
        [C1, C2, C3, C4] = controlGains(APN, XNP, TGO, TAU);
        if APN == 0
            XNC = C1*YH + C2*YDH;
        elseif APN == 1
            XNC = C1*YH + C2*YDH + C3*XNTH;
        else
            XNC = C1*YH + C2*YDH + C3*XNTH + C4*XNL;
        end
    end
end
Kseq = Kseq(1:nK,:);
end


% =============================================================================
function [resid, allow, peakF, slope, adjAt] = agreeMetric(tp, miss, tgo, fmiss, H)
%AGREEMETRIC  The book's "exact agreement" claim, made measurable.
%
%   NEAREST SAMPLE, not interpolation: the adjoint grid is TS-spaced and the
%   comparison points sit on it, so interpolating would blur a real
%   disagreement into a smaller one.
%
%   The band is a MECHANISM, not a fitted number -- see BENCHMARK 4-6 in the
%   header.  slope is estimated from the forward curve's own finite differences,
%   so it scales with whatever case is being run.
adjAt = zeros(size(tgo));
for k = 1:numel(tgo)
    [~, j] = min(abs(tp - tgo(k)));
    adjAt(k) = miss(j);
end
resid = max(abs(adjAt - fmiss));
peakF = max(abs(fmiss));
if numel(tgo) >= 2
    slope = max(abs(diff(fmiss)./diff(tgo)));
else
    slope = 0;
end
allow = max(0.05*peakF, 2*H*slope);
end


% =============================================================================
function K = structuralChecks(U, V, W, E, F, G, fwdK, C, P, didForward)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.  .names and .oks are
%   what benchSuite puts on the board -- without BOTH, every one of these is
%   computed and then invisible.
names = {};  oks = [];  detail = {};  notrun = {};

% 1  Kalman gains are gains: K1 in (0,1], and K1 -> 1 as tgo -> 0 (the last
%    Riccati step has SIGN2 ~ 0 because RTM = VC*tgo -> 0).
ok1 = all(U > 0) && all(U <= 1 + 1e-12) && abs(U(end) - 1) < 1e-6;
names{end+1} = 'K1 in (0,1] and K1 -> 1 as tgo -> 0';
oks(end+1)   = ok1;
detail{end+1} = sprintf('min %.4g, max %.4g, last %.8f', min(U), max(U), U(end));

% 2  The "% Modification" reversal, exactly as transcribed: E(99..1) = U(1..99)
%    and E(100) = 0, i.e. the tgo -> 0 gain U(100) is DISCARDED.  This pins the
%    quirk so a later "tidy-up" cannot silently re-include it.
n  = numel(E);
ok2 = n == numel(U) && E(n) == 0 && F(n) == 0 && G(n) == 0 && ...
      isequal(E(1:n-1), fliplr(U(1:n-1))) && ...
      isequal(F(1:n-1), fliplr(V(1:n-1))) && ...
      isequal(G(1:n-1), fliplr(W(1:n-1)));
names{end+1} = 'gain reversal E(k)=U(n-k), E(n)=0 (U(n) discarded)';
oks(end+1)   = ok2;
detail{end+1} = sprintf('n = %d, E(n) = %g, U(n) = %.6f discarded', n, E(n), U(end));

% 3  Sampler arithmetic: exactly TF/TS adjoint samples in every case, so the
%    gain index never runs past the array.
want = round(P.TF/P.TS);
got  = arrayfun(@(c) numel(c.tp), C);
names{end+1} = sprintf('adjoint samples == TF/TS == %d in all three cases', want);
oks(end+1)   = all(got == want);
detail{end+1} = sprintf('samples %s', mat2str(got));

if didForward
    % 4  The standalone Riccati loop of 29.4 must reproduce the one EMBEDDED in
    %    29.3's sampler.  If it does not, "adjoint vs forward" is comparing two
    %    different filters and every agreement row below is meaningless.
    if isempty(fwdK) || size(fwdK,1) ~= numel(U)
        ok4 = false;
        d4  = sprintf('forward returned %d gain rows against %d', ...
                      size(fwdK,1), numel(U));
    else
        rel = max(max(abs(fwdK - [U(:) V(:) W(:)]) ./ ...
                      max(abs([U(:) V(:) W(:)]), realmin)));
        ok4 = rel <= 1e-6;
        d4  = sprintf('max relative gain difference %.3e', rel);
    end
    names{end+1} = '29.4 standalone Riccati == 29.3 embedded Riccati';
    oks(end+1)   = ok4;
    detail{end+1} = d4;

    % 5  DELETE EXPERIMENT.  "we must reverse all time varying gains ... to the
    %    rules of adjoints" (decoded:10973).  Run the adjoint with the gains NOT
    %    reversed and require the agreement to BREAK by a wide margin.  If it
    %    does not, the agreement rows are not testing the reversal at all and
    %    the whole file is decorative.
    [tpU, missU] = adjointRun(0, U, V, W, P.XNT, P.XNP, P.TAP, P.TF, P.TS, ...
                              P.VC, P.H, false);
    rU = agreeMetric(tpU, missU, C(1).fwd_tgo, C(1).fwd_miss, P.H);
    ok5 = rU >= 5*C(1).allow;
    names{end+1} = 'DELETE: unreversed Kalman gains break the agreement';
    oks(end+1)   = ok5;
    detail{end+1} = sprintf(['unreversed residual %.4g ft vs reversed %.4g ft ' ...
        '(band %.4g ft); need >= 5x band'], rU, C(1).resid, C(1).allow);
else
    notrun = {'29.4 standalone Riccati == 29.3 embedded Riccati', ...
              'DELETE: unreversed Kalman gains break the agreement'};
end

K = struct();
K.names  = names(:)';
K.oks    = logical(oks(:)');
K.detail = detail(:)';
K.notrun = notrun;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    j = find(~K.oks, 1);
    K.first_bad = sprintf('%s  [%s]', K.names{j}, K.detail{j});
end
end


% =============================================================================
function B = benchmarks(C, P, didForward)
%BENCHMARKS  Computed vs published.  Every book value was grepped and the line
%   is in .source.  Bands come from provenance and mechanism, never from what
%   we happen to compute.  Do NOT widen one to turn a row green.
B = emptyBench();
g = 32.2;

% ---- 1.  XNT = 96.6 ft/s^2 is the prose's 3-g maneuver --------------------
src1 = 'listing line 1 (zarchan_decoded.txt:10981) vs prose (:10947)';
if abs(P.XNT - 96.6) > 1e-9
    B = addSkip(B, 'XNT / g  (the "3-g target maneuver")', '3 g', ...
        sprintf(['this run used XNT = %g ft/s^2 = %.4g g.  The book value is ' ...
                 'the LISTING''S OWN 96.6; a different XNT is a different ' ...
                 'case, not a rescaled benchmark.'], P.XNT, P.XNT/g), src1, 'g');
else
    B = addVal(B, 'XNT / g  (the "3-g target maneuver")', 3, P.XNT/g, ...
        'exact', 1e-9, src1, 'g', 'g = 32.2 ft/s^2; 96.6/32.2');
end

% ---- 2.  the optimal-guidance gain C4 at x = tgo/T = 1 --------------------
x  = 1;
NPb = 6*x^2*(exp(-x) - 1 + x) / (2*x^3 + 3 + 6*x - 6*x^2 - 12*x*exp(-x) - 3*exp(-2*x));
c4b = NPb*(exp(-x) + x - 1)/x^2;                    % the book's printed form
[~,~,~,c4o] = controlGains(2, P.XNP, P.TAP*x, P.TAP);   % the listing's, at tgo = T
B = addVal(B, '|C4| at tgo/T = 1  (optimal guidance)', c4b, abs(c4o), ...
    'C4 formula', 0.002, ...
    ['C4 = N''(e^-x+x-1)/x^2, zarchan_decoded.txt:2576 (= zarchan.txt:2540); ' ...
     'sign from the ZEM bracket at :2556-2559 (= :2520-2523)'], '-', ...
    ['listing C4 is NEGATIVE (both listings fold the bracket''s minus into ' ...
     'C4); magnitude scored.  Band 0.2% covers the listing''s +.0001 ' ...
     'denominator guard, which the book''s formula does not have.']);

% ---- 3.  N' -> 3 for long time to go --------------------------------------
xL = 1000;
[~, c2L] = controlGains(2, P.XNP, P.TAP*xL, P.TAP);
B = addVal(B, 'optimal N'' at tgo/T = 1000', 3, c2L*(P.TAP*xL), ...
    'prose "approaching 3"', 0.01, ...
    'zarchan_decoded.txt:2562 (= zarchan.txt:2526), formula at :2561', '-', ...
    ['N'' recovered as C2*tgo.  The limit is approached like 3(1+2/x), so ' ...
     'x = 1000 gives 3.006 (+0.20%) -- the 1% band is sized for that.']);

% ---- 4-6.  adjoint vs forward, one row per guidance law -------------------
srcA = ['"exact agreement between the adjoint and forward simulations in all ' ...
        'cases", zarchan_decoded.txt:11007 + :11024 (= zarchan.txt:10820 + :10837)'];
for c = 1:3
    what = sprintf('adjoint == forward, %s', C(c).name);
    if ~didForward
        B = addSkip(B, what, 'exact agreement', ...
            ['''forward'' is false, so Listing 29.3 was not run.  This is the ' ...
             'ONLY published claim about Listing 29.4''s output; without it ' ...
             'nothing this port computes is adjudicated.'], srcA, 'ft');
        continue
    end
    B = addBound(B, what, ...
        sprintf('<= %.4g ft', C(c).allow), ...
        sprintf('%.4g ft (%.3g%% of peak %.4g ft)', C(c).resid, ...
                100*C(c).resid/max(C(c).peak, realmin), C(c).peak), ...
        C(c).resid <= C(c).allow, srcA, 'ft', ...
        sprintf(['band = max(5%% of peak = %.4g ft, 2*H*max|dmiss/dtgo| = ' ...
                 '%.4g ft); the second term is one integration step of ' ...
                 'maneuver-onset jitter'], 0.05*C(c).peak, 2*P.H*C(c).slope));
end

% ---- 7.  the ordering the prose claims ------------------------------------
if didForward
    pk = [C.peak];
    B = addShape(B, 'peak |miss| falls as the law advances: PN >= APN >= optimal', ...
        'PN >= APN >= optimal', ...
        sprintf('%.4g / %.4g / %.4g ft', pk(1), pk(2), pk(3)), ...
        pk(1) >= pk(2) && pk(2) >= pk(3), ...
        ['"the miss distance results get better as the guidance law becomes ' ...
         'more advanced", zarchan_decoded.txt:11024 (= zarchan.txt:10837)'], ...
        ['in tension with the extracted axis ranges: Fig. 29.9 runs 0..20 ' ...
         'but Fig. 29.10 runs -20..10.  See the INFO row; the tension was NOT ' ...
         'used to weaken this band.']);
else
    B = addSkip(B, 'peak |miss| falls as the law advances', 'PN >= APN >= optimal', ...
        '''forward'' is false; peaks come from the forward sweep.', ...
        'zarchan_decoded.txt:11024', 'ft');
end

% ---- 8.  the figures themselves -------------------------------------------
B = addSkip(B, 'the plotted miss-vs-tgo curves of Figs. 29.9-29.11', ...
    'curves only', ...
    ['the figures became image links (zarchan_decoded.txt:11022, :11052, ' ...
     ':11054); no curve value survived extraction, so no point on them is ' ...
     'scoreable.  Widening a band to absorb a figure reading is exactly the ' ...
     'failure this row refuses.'], ...
    'zarchan_decoded.txt:11022, :11052, :11054', 'ft');

% ---- 9.  the axis furniture, reported and NOT adjudicated -----------------
if didForward
    ours = sprintf('peaks %.4g / %.4g / %.4g ft', C(1).peak, C(2).peak, C(3).peak);
else
    ours = 'not computed (''forward'' false)';
end
B = addInfo(B, 'y-axis tick sequences as extracted, beside our peaks', ...
    'Fig 29.9: 20 15 10 5 0 | Fig 29.10: -20 -10 0 10 | Fig 29.11: 3 2 1 0 -1', ...
    ours, ...
    'zarchan_decoded.txt:11008-11012, :11025-11028, :11037-11041', ...
    ['axis furniture, not a curve reading: the tick ORDER is top-to-bottom in ' ...
     'Figs. 29.9/29.11 and bottom-to-top in Fig. 29.10, so these give an ' ...
     'order of magnitude and nothing that can be adjudicated.']);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want.  Schema is the
%   one benchSuite's shared benchRows() consumes.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.5g', book);
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
%ADDINFO  Reported, never scored.  An INFO row can neither pass nor fail.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'info';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'info';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = '-';  B(i).detail = '';
end

function B = addSkip(B, what, bookStr, why, source, units)
%ADDSKIP  A check that COULD NOT RUN.  Never a pass: counted as a check that
%   did not happen, which is a different thing from one that succeeded.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'skip';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = '--';
B(i).pct  = NaN;   B(i).band = 'skip';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end


% =============================================================================
function s = render(R, C, P, ev)
L = {};
L{end+1} = 'Zarchan Listing 29.4 -- ADJOINT OF THE HOMING LOOP WITH A';
L{end+1} = '   THREE-STATE KALMAN FILTER (the book prints "AJOINT")';
L{end+1} = '   + Listing 29.3, its forward twin, which is what scores it';
L{end+1} = '*** UNITS ARE FEET, ft/s, ft/s^2, s.  g = 32.2.  NOTHING HERE IS SI. ***';
L{end+1} = sprintf(['   XNT = %g ft/s^2 (%.3g g)   N'' = %g   TAP = %g s   ' ...
    'TF = %g s   TS = %g s   H = %g s'], P.XNT, P.XNT/32.2, P.XNP, P.TAP, ...
    P.TF, P.TS, P.H);
L{end+1} = sprintf('   VC = %g ft/s   VM = %g ft/s   HEDEGF = %g deg   SIGNOISE = %g rad', ...
    P.VC, P.VM, P.HEDEGF, P.SIGNOISE);
L{end+1} = sprintf('   %d Kalman gains; %d adjoint samples per case; last K1 = %.8f', ...
    numel(R.gains.U), numel(C(1).tp), R.gains.U(end));
L{end+1} = '';

L{end+1} = 'MISS vs TIME TO GO AT WHICH THE 3-g MANEUVER STARTS  [ft]';
if P.forward
    L{end+1} = sprintf('%8s %12s %12s %12s %12s %12s %12s', 'tgo [s]', ...
        'adj PN', 'fwd PN', 'adj APN', 'fwd APN', 'adj opt', 'fwd opt');
    for k = 1:numel(C(1).fwd_tgo)
        L{end+1} = sprintf('%8.2f %12.4f %12.4f %12.4f %12.4f %12.4f %12.4f', ...
            C(1).fwd_tgo(k), C(1).adj_at(k), C(1).fwd_miss(k), ...
            C(2).adj_at(k), C(2).fwd_miss(k), C(3).adj_at(k), C(3).fwd_miss(k)); %#ok<AGROW>
    end
    L{end+1} = '';
    for c = 1:3
        L{end+1} = sprintf(['   %-22s peak %9.4f ft   max|adj-fwd| %9.4f ft   ' ...
            'band %9.4f ft'], C(c).name, C(c).peak, C(c).resid, C(c).allow); %#ok<AGROW>
    end
else
    L{end+1} = '   (''forward'' is false -- Listing 29.3 was not run, so NOTHING';
    L{end+1} = '    this port computes has been adjudicated.)';
    stride = max(1, round(1/P.TS));
    idx = unique([1:stride:numel(C(1).tp), numel(C(1).tp)]);
    L{end+1} = sprintf('%8s %12s %12s %12s', 'tgo [s]', 'PN', 'APN', 'optimal');
    for k = idx
        L{end+1} = sprintf('%8.2f %12.4f %12.4f %12.4f', C(1).tp(k), ...
            C(1).miss(k), C(2).miss(k), C(3).miss(k)); %#ok<AGROW>
    end
end
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL (self-consistency, NOT book values)';
for i = 1:numel(K.names)
    if K.oks(i), tag = 'holds   '; else, tag = 'VIOLATED'; end
    L{end+1} = sprintf('   [%s] %-52s %s', tag, K.names{i}, K.detail{i}); %#ok<AGROW>
end
for i = 1:numel(K.notrun)
    L{end+1} = sprintf('   [not run] %s', K.notrun{i}); %#ok<AGROW>
end
L{end+1} = '';

L{end+1} = 'BENCHMARKS vs the book';
for i = 1:numel(R.bench)
    b = R.bench(i);
    switch lower(b.kind)
    case 'skip'
        L{end+1} = sprintf('   [SKIP] %-46s %s', b.what, b.why); %#ok<AGROW>
    case 'info'
        L{end+1} = sprintf('   [INFO] %-46s book %s | ours %s', b.what, ...
            b.book_str, b.ours_str); %#ok<AGROW>
    case {'shape','bound'}
        if b.ok, v = 'PASS'; else, v = 'FAIL'; end
        L{end+1} = sprintf('   [%s] %-46s book %s | ours %s', v, b.what, ...
            b.book_str, b.ours_str); %#ok<AGROW>
    otherwise
        if b.skipped
            L{end+1} = sprintf('   [SKIP] %-46s %s', b.what, b.why); %#ok<AGROW>
        else
            if b.ok, v = 'PASS'; else, v = 'FAIL'; end
            L{end+1} = sprintf('   [%s] %-46s book %s | ours %s | %+.3f%% (band %.3g%%)', ...
                v, b.what, b.book_str, b.ours_str, b.pct, 100*b.tol); %#ok<AGROW>
        end
    end
end
L{end+1} = sprintf('   -> %d of %d evaluated rows pass', ...
    sum([R.bench(ev).ok]), sum(ev));
s = strjoin(L, newline);
end


% =============================================================================
function drawCurves(C, P)
figure;
hold on; grid on;
for c = 1:3
    plot(C(c).tp, C(c).miss, 'LineWidth', 1.2, 'DisplayName', ...
        sprintf('adjoint, %s', C(c).name));
    if P.forward
        plot(C(c).fwd_tgo, C(c).fwd_miss, 'o', 'DisplayName', ...
            sprintf('forward, %s', C(c).name));
    end
end
hold off;
title('Miss for various tgo at which the 3-g maneuver starts (10 s flight)');
xlabel('Time to go at which maneuver occurs [s]');
ylabel('Miss [ft]');
legend('Location','best');
end
