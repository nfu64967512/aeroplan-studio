function R = operationalAreas3d(opts)
%MGS.ZARCHAN.OPERATIONALAREAS3D  Zarchan Listing 32.3 -- THREE-DIMENSIONAL
%   OPERATIONAL AREAS: from WHERE can an interceptor of limited burnout
%   velocity be launched and still reach a 10,000-km minimum-energy ICBM
%   somewhere on its trajectory?
%
%   R = mgs.zarchan.operationalAreas3d()
%   R = mgs.zarchan.operationalAreas3d('lat_deg', -40:2.5:40)   % the BOOK'S grid
%   R = mgs.zarchan.operationalAreas3d('alt_datum', "xtf", 'assert', false)
%
%   Book source (the DECODED file; line numbers shared with the raw one):
%   zarchan_decoded.txt:12331 LISTING 32.3 header, :12332-12348 the code,
%   :12325 the sweep and the gates in prose, :12349 Fig. 32.7 (6 km/s),
%   :12352 Fig. 32.8 (5 km/s) and the 6-km/s extents, :12390 Fig. 32.9
%   (4 km/s), :12172 the 7.19 km/s minimum-energy burnout velocity.
%   Transcription ported: matlab/zarchan/_listings/listing_32_03.m
%
% ===========================================================================
%   *** UNITS.  CHAPTER 32 REPORTS METRIC -- km, km/s, s, deg -- AND THAT IS
%   UNUSUAL FOR THIS BOOK. ***  Every number printed, scored or returned here
%   is metric.  The STATE underneath is still the book's feet: A = 2.0926e7 ft,
%   GM = 1.4077e16 ft^3/s^2, converted at the listing's own 3280 ft/km.
%   KEPLER1 works in km with its own hard-coded GMX = 398923 km^3/s^2
%   (:10486), 1.14e-6 above GM/3280^3 -- the mismatch keplerVsDirect scores.
%   It is the book's, it is kept, and SELFCHECK 4 measures its cost (metres).
%   The plot axes are NOT the model's geometry: ArrayXLONGM = XLONGMDEG*111,
%   while this Earth gives 111.34 km/deg, so every distance quoted is 0.3%
%   short of the arc flown.  Kept: the published extents are read off THAT axis.
% ===========================================================================
%
%   THE LISTING, AND THE FOUR THINGS THIS PORT DOES DIFFERENTLY.  Four nested
%   loops -- longitude x latitude x launch time x intercept time.  Per case: a
%   Lambert solve gives the target its 10,000-km arc velocity, KEPLER1
%   propagates the target to the intercept time TF, a second Lambert solve
%   gives the interceptor the velocity to be there at TF, and the case is kept
%   if VBOM < VBOLIM and VBOT < 7.5 km/s.  Nothing is integrated (:12324).
%     1  THE TARGET LAMBERT SOLVE IS LOOP-INVARIANT (TGOLAM = TFTOT - T, T = 0
%        always), so the listing re-solves one identical problem 272,118 times.
%        Hoisted; VBOT is a SCALAR and its gate is one scalar test.
%     2  THE TARGET'S KEPLER1 CALL DEPENDS ONLY ON TF.  266 (TLAUNCH,TF) pairs
%        use 32 distinct TF (all = TLAUNCH+60+50k, TLAUNCH a multiple of 100,
%        so TF = 10 mod 50).  Memoised: 32 calls, not 272,118.
%     3  THE MISSILE'S KEPLER1 CALL IS DEAD CODE -- XMF..ZMDF are computed and
%        never read; VBOM comes from LAMBERT3D's XMD,YMD,ZMD.  Dropped, and
%        SELFCHECK 4 runs it once to show what it would have shown.
%     4  VBOLIM ONLY FILTERS, so ONE sweep recording min VBOM per grid point
%        answers Figs. 32.7, 32.8 and 32.9 at once.  (The listing's own
%        VBOLIM = 5 is Fig. 32.8; the prose runs 6, 5 and 4 km/s.)
%   Net: one Lambert solve per (lon, lat, TLAUNCH, TF) and nothing else.
%   TWO ADDED GUARDS.  (a) LAMBERT3D divides by CROSSMAG = |r_m x r_t| and can
%   leave the real branch; an Inf or a COMPLEX velocity that isfinite() would
%   accept comes back NaN, is skipped and is COUNTED (expected count: 0).
%   (b) It cannot tell convergence from its ITERS > 100 break, so ITERS and the
%   residual are returned and SELFCHECK 1 scores them.
%
%   OCR RESOLUTIONS.  "if (VBOM,VBOLIM & VBOT,7.5)" -> "VBOM < VBOLIM &&
%   VBOT < 7.5": both directions are prose at :12325 ("less than VBOLIM ... a
%   success"; "not considered ... greater than 7.5"), strict "<" being the OCR
%   of ",".  "if ALTFKM,50." -> "< 50., break" (:12325, "intercepts are made
%   above 50-km altitude"); already resolved in the transcription.  Dropped as
%   prose leaking in: the transcription's lines 31 on (:12349 on).  Recorded
%   and dropped: "save datfil.txt output /ascii" (typo for -ascii); I = 1,
%   PI = 3.14159 and the pre-loop TLAUNCH = 300, all assigned and never read.
%
%   /!\ THE ALTITUDE DATUM IS A BOOK BUG, NOT AN OCR ARTEFACT.  The line reads
%   ALTFKM = (sqrt(XT^2 + YTF^2 + ZTF^2) - A)/3280 -- XT, the target's LAUNCH
%   x-coordinate, beside the propagated YTF and ZTF.  Listing 32.4 (:12421)
%   repeats it verbatim; Listing 32.5 (:12508) prints XTF.  An OCR that dropped
%   one "F" twice and kept it a third time is not credible, so this is the
%   book's own typo, copy-pasted once and fixed on the third copy.  As printed
%   it is not an altitude: at the target's impact point it returns
%   (sqrt(2)-1)*A = 2642 km, not 0.  THE TRANSCRIPTION IS WHAT IS PORTED, so
%   'alt_datum' defaults to "verbatim" and SELFCHECK 3 measures whether it
%   MATTERS.  On the nominal grid it does not: TF runs 360..1910 s and the true
%   altitude at those ends is 809.7 and 76.5 km, both above 50, so the gate
%   never fires under EITHER reading and both accept all 266 pairs.  It would
%   matter at a finer dTF, or for TLOFT < 0.
%
%   THE LATITUDE GRID.  The book sweeps -40:2.5:40 (33) -> 272,118 Lambert
%   solves; the default here is -40:10:40 (9), a SUBSET of the book's lattice,
%   for 74,214.  Every scored row is a DOWNRANGE extent and min VBOM rises
%   monotonically with |latitude| (at lon = 105 deg: 4.083, 4.100, 4.149,
%   4.340, 4.853 km/s for |lat| = 0, 2.5, 5, 10, 20), so every extent is
%   attained on the zero-latitude row, which is in the grid.  SELFCHECK 2
%   asserts that instead of assuming it.  Latitude DOES move the point counts,
%   so the counts are reported and only their ORDERING is scored.
%
%   THE BAND.  The published extents are readings off scatter plots whose plus
%   signs can only sit on a 5-deg longitude lattice, i.e. on multiples of
%   5*111 = 555 km.  That is the finest distinction Figs. 32.7-32.9 can
%   express, and it is the band.  It is not widened; a longitude grid COARSER
%   than the book's sends the extent rows to SKIP rather than scoring them
%   against a band the run itself inflated.
%
%   WHAT THIS PRINTS.  *** MEASURED.  This block used to be labelled "DERIVED,
%   NOT MEASURED ... this file has never been executed here", and that is no
%   longer true: it has been run, and every number below came back identical to
%   the derivation -- VBOT 7.193694, TFTOT 1938.0, DISTFKM 10000.0000, all three
%   extents and all three point counts.  The derivation was an INDEPENDENT
%   check -- a universal-variable (Bate-Mueller-White) Lambert solver and a
%   classical-element Kepler propagator sharing no code with the book's
%   gamma-iteration LAMBERT3D or with KEPLER1 -- which also agrees with the
%   already-ported Listing 32.1 (apogee 1331.8 km against impulsiveIcbm's
%   1331.84).  Two independent implementations and one execution now agree.
%     TFTOT = 252 + 2230 - 544 = 1938.0 s exactly (the 57.3 in XLONGFDEG
%       cancels, so DISTFKM = RDESKM = 10000 exactly)
%     VBOT = 7.193694 km/s (book 7.19); 266 pairs, 32 TF from 360 to 1910 s;
%       degenerate skips 0
%     min VBOM along the equator [km/s] at 0, 555, 1110, ... km downrange:
%       7.303 7.164 7.023 6.866 6.308 5.407 4.906 4.693 4.636 4.654 4.609
%       4.479 4.278 4.016 3.658 3.257 2.673 2.030 1.841 2.789 3.541 4.083
%       4.535 4.917 5.245 5.532 5.785 6.010 6.212 6.394 6.554
%     operational areas -- extent [km] and points on the 9-lat grid vs book:
%       6 km/s   2775 .. 14430   156 pts   book 3000 .. 14,500   :12352
%       5 km/s   3330 .. 12765    62 pts   book 3500 .. 12,500   :12352
%       4 km/s   7770 .. 11100    19 pts   book 7500 .. 10,500   :12390
%
%   /!\ ONE ROW IS RED, AND IS LEFT RED.  Five extents land 70-270 km from the
%   book, well inside one 555-km lattice step.  The sixth -- the 4-km/s UPPER
%   edge -- lands 600 km out, 1.08 steps, and fails.  Three things were measured
%   before leaving it red, and each kills one of the cheap explanations:
%
%     UNITS.  Ch.32 is metric while most of the book is ft with g = 32.2, so
%     this is the first thing to rule out and the most embarrassing to miss.
%     Both sides are km and the SAME km: the book's own axis conversion is
%     ArrayXLONGM = XLONGMDEG*111 (:12345) and P.KMPDEG is that same 111, so
%     the book's 10,500 and our 11,100 are the same unit by construction.
%     Independent confirmation on the same page: VBOT comes back 7.1937 against
%     the book's 7.19 (0.05%), and 150 deg gives 16,650 km against the book's
%     "more than 16,000 km" (:12325).  A ft/km slip anywhere would have moved
%     those by a factor of 3280, not by 6%.  It is not units.
%
%     THE LATTICE.  Re-run at 0.5 deg instead of 5 deg -- ten times finer -- and
%     the continuum crossing is at 11,544 km, i.e. refining moves the edge 444
%     km FURTHER FROM the book, not toward it.  Selfcheck 6 scores the property
%     that licenses the coarse grid: across the bracket the profile is monotone
%     and crosses the gate exactly once, so the coarse edge is the quantisation
%     of one real crossing rather than an aliased sample.  It is not the grid.
%
%     THE MARGIN.  min VBOM at the book's own 10,500 km is 2.75 km/s -- 31%
%     BELOW the 4.000 gate, not near it.  At our own edge, 11,100 km, it is
%     3.541 (12% below) and the next lattice point out, 11,655 km, is 4.083 and
%     IS excluded.  For the region to end where the book says, minVBOM at
%     10,545 km would have to jump from 2.79 to above 4.0.  Selfcheck 7 scores
%     that margin.  It is not a near-threshold coin flip.
%
%     A UNIFORM ERROR.  The last cheap explanation is that this whole page is
%     biased low.  It is not survivable: scaling VBOM by 1.40 -- the smallest
%     uniform factor that brings the 4-km/s upper edge onto the book (10,545 vs
%     10,500 km) -- moves the other five extents to 6660/11,655/8325/11,100/8880
%     against 3000/14,500/3500/12,500/7500 and turns ALL FIVE red.  There is no
%     uniform correction that reconciles the six; the disagreement is isolated
%     to one edge.
%
%   So the edge is where the arithmetic puts it, and either Fig. 32.9's last
%   plus sign was read short or the figure carries a setting the text does not
%   state.  Widening the band to 600 km would hide the only row on this page
%   that is saying something.
%
%   OUTPUTS.  R.minVBOM (nlat x nlon, least burnout velocity that works at each
%   site [km/s], Inf where nothing does), R.best_TF, R.best_TLAUNCH,
%   R.downrange_km, R.crossrange_km, R.area (one struct per VBOLIM: .VBOLIM
%   .mask .npts .dmin_km .dmax_km .dmin0_km .dmax0_km .sym_ok), R.target
%   (.TFTOT .DISTFKM .VBOT_kms .tf .alt_km_xtf .alt_km_verbatim .pairs .iters
%   .resid), R.bench, R.selfcheck (.names/.oks/.detail -- structural, NOT book
%   values), R.pass (true only if at least one row was SCORED and all passed),
%   R.verdict (ALWAYS formed; 'assert' picks error vs warn-and-return-anyway,
%   and benchSuite runs with assert off), R.p, R.units, R.summary.
%   Nothing random is drawn anywhere here, so there is no seed.
%
%   See also MGS.ZARCHAN.IMPULSIVEICBM (32.1), MGS.ZARCHAN.KINEMATICENGAGEMENTMC
%   (32.2), MGS.ZARCHAN.KEPLERVSDIRECT (28.3), MGS.ZARCHAN.BENCHSUITE.

arguments
    opts.RDESKM   (1,1) double {mustBeFinite, mustBePositive}    = 10000.  % km
    opts.TLOFT    (1,1) double {mustBeFinite}                    = 0.      % s
    opts.ALTMKMIC (1,1) double {mustBeFinite, mustBeNonnegative} = 0.      % km
    % The listing's own line 1 sets VBOLIM = 5 (that is Fig. 32.8).  The prose
    % runs 6, 5 and 4 km/s and all three are scored; see restructuring 4.
    opts.VBOLIM   (1,:) double {mustBeFinite, mustBePositive}    = [6 5 4] % km/s
    opts.VBOTLIM  (1,1) double {mustBeFinite, mustBePositive}    = 7.5     % km/s
    opts.ALTFKM_MIN (1,1) double {mustBeFinite}                  = 50.     % km
    opts.lon_deg  (1,:) double {mustBeFinite}                    = 0:5:150
    % [] means the lean default -40:10:40.  The book's own grid is -40:2.5:40
    % and costs about 4x; see the latitude-grid note in the header.
    opts.lat_deg  double {mustBeReal, mustBeFinite}              = []
    opts.tlaunch  (1,:) double {mustBeFinite, mustBeNonnegative} = 300:100:1600
    opts.dTF      (1,1) double {mustBeFinite, mustBePositive}    = 50.     % s
    opts.TF_lead  (1,1) double {mustBeFinite, mustBePositive}    = 60.     % s
    % "verbatim" is the transcription (XT, as printed in 32.3 and 32.4);
    % "xtf" is Listing 32.5's reading.  SELFCHECK 3 measures the difference.
    opts.alt_datum (1,1) string ...
        {mustBeMember(opts.alt_datum, ["verbatim","xtf"])} = "verbatim"
    opts.theta_min (1,1) double {mustBeFinite, mustBePositive}   = 1e-3    % rad
    % NOT nargin: with only name-value options declared, MATLAB reports
    % nargin = 0 however many were passed, so a mode switch on it never fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

lat = opts.lat_deg(:).';
if isempty(lat)
    lat = -40:10:40;                 % see the latitude-grid note in the header
end
if ~any(abs(lat) < 1e-12)
    error('mgs:zarchan:operationalAreas3d:lat0', ...
        ['the latitude grid must contain 0 deg.  Every scored row is a ' ...
         'DOWNRANGE extent, min VBOM rises with |latitude|, and SELFCHECK 2 ' ...
         'compares the whole-grid extent against the zero-latitude row.  ' ...
         'Without that row the rows would still print, against a region ' ...
         'that never touches the target trajectory.']);
end

if isempty(opts.VBOLIM)
    error('mgs:zarchan:operationalAreas3d:vbolim', ...
        ['VBOLIM is empty, so no operational area would be formed and every ' ...
         'extent row would report on a region that was never defined.  The ' ...
         'book''s three cases are 6, 5 and 4 km/s.']);
end

P = struct('A', 2.0926E7, 'GM', 1.4077E16, 'GMX', 398923., 'FTPKM', 3280., ...
    'DEG', 57.3, 'KMPDEG', 111., 'RDESKM', opts.RDESKM, 'TLOFT', opts.TLOFT, ...
    'ALTMKMIC', opts.ALTMKMIC, 'lon', opts.lon_deg, 'lat', lat, ...
    'tlaunch', opts.tlaunch, 'dTF', opts.dTF, 'lead', opts.TF_lead, ...
    'altmin', opts.ALTFKM_MIN, 'xtf', opts.alt_datum == "xtf", ...
    'theta_min', opts.theta_min);

S = sweepGrid(P);

R = struct();
R.minVBOM       = S.minVBOM;
R.best_TF       = S.bestTF;
R.best_TLAUNCH  = S.bestTL;
R.downrange_km  = P.lon * P.KMPDEG;
R.crossrange_km = P.lat * P.KMPDEG;
R.target = struct('TFTOT', S.TFTOT, 'DISTFKM', S.DISTFKM, 'VBOT_kms', S.VBOT, ...
    'tf', S.tf, 'alt_km_xtf', S.altX, 'alt_km_verbatim', S.altV, ...
    'pairs', [S.pairTL(:) S.pairTF(:)], 'iters', S.itT, 'resid', S.rsT);

% VBOLIM only filters -- see restructuring 4.  The VBOT gate is one scalar.
gateT = S.VBOT < opts.VBOTLIM;
i0    = find(abs(P.lat) < 1e-12, 1);
AR    = struct('VBOLIM', {}, 'mask', {}, 'npts', {}, 'dmin_km', {}, ...
               'dmax_km', {}, 'dmin0_km', {}, 'dmax0_km', {}, 'sym_ok', {});
for k = 1:numel(opts.VBOLIM)
    M = (S.minVBOM < opts.VBOLIM(k)) & gateT;
    AR(k) = struct('VBOLIM', opts.VBOLIM(k), 'mask', M, 'npts', nnz(M), ...
        'dmin_km',  extentOf(M,        R.downrange_km, @min), ...
        'dmax_km',  extentOf(M,        R.downrange_km, @max), ...
        'dmin0_km', extentOf(M(i0,:),  R.downrange_km, @min), ...
        'dmax0_km', extentOf(M(i0,:),  R.downrange_km, @max), ...
        'sym_ok', isequal(M, flipud(M)) && isequal(P.lat, -fliplr(P.lat)));
end
R.area = AR;

R.units = ['CHAPTER 32 REPORTS METRIC: km, km/s, s, deg.  The state underneath is ' ...
    'the book''s ft, ft/s (A = 2.0926e7 ft, GM = 1.4077e16 ft^3/s^2, 3280 ft/km); ' ...
    'KEPLER1 works in km with GMX = 398923 km^3/s^2; the plot axes use 111 km/deg'];
R.p = struct('RDESKM', opts.RDESKM, 'TLOFT', opts.TLOFT, 'VBOLIM', opts.VBOLIM, ...
    'VBOTLIM', opts.VBOTLIM, 'ALTFKM_MIN', opts.ALTFKM_MIN, ...
    'ALTMKMIC', opts.ALTMKMIC, 'lon_deg', P.lon, 'lat_deg', P.lat, ...
    'tlaunch', opts.tlaunch, 'dTF', opts.dTF, 'TF_lead', opts.TF_lead, ...
    'alt_datum', opts.alt_datum, 'theta_min', opts.theta_min, ...
    'n_pairs', numel(S.pairTF), 'n_lambert', S.nlam, 'n_noconv', S.nnoconv, ...
    'n_degenerate', S.ndegen, 'max_iters', S.maxit, 'A_ft', P.A, 'GM', P.GM, ...
    'GMX', P.GMX, 'FT_PER_KM', P.FTPKM, 'KM_PER_DEG', P.KMPDEG, ...
    'seed', 'none -- nothing random is drawn', 'units', R.units);

R.selfcheck = structuralChecks(R, S, P);

if opts.benchmark
    R.bench = benchmarks(R, S, P);
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
if opts.plot,   drawArea(R);              end

msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated.  The published ' ...
        'extents are for a 10,000-km minimum-energy target on the book''s ' ...
        '0:5:150 deg longitude grid at VBOLIM in {6,5,4} km/s; this run used ' ...
        'RDESKM = %g km, TLOFT = %g s, VBOLIM = %s.  An unvalidated port ' ...
        'must not be handed back silently.'], opts.RDESKM, opts.TLOFT, ...
        mat2str(opts.VBOLIM));
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
        error('mgs:zarchan:operationalAreas3d:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:operationalAreas3d:benchmark', ...
            '%s  (''assert'' is false, so R is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function S = sweepGrid(P)
%SWEEPGRID  Listing 32.3, zarchan.txt:12332-12347, with the four restructurings
%   described in the header.  Every constant arrives inside P: MATLAB local
%   functions cannot see the caller's workspace.

A = P.A;  GM = P.GM;  FTPKM = P.FTPKM;  DEG = P.DEG;

% ---- the listing's setup block, verbatim (:12332) ---------------------------
ALTM      = P.ALTMKMIC*FTPKM;
XLONGFDEG = DEG*P.RDESKM*FTPKM./A;   XLATFDEG = 0.;
XLONGTDEG = 0.;                      XLATTDEG = 0.;
XLONGF = XLONGFDEG/DEG;  XLATF = XLATFDEG/DEG;    % the 57.3 cancels exactly
XF = A*cos(XLATF)*cos(XLONGF);  YF = A*cos(XLATF)*sin(XLONGF);  ZF = A*sin(XLATF);
XLONGT = XLONGTDEG/DEG;  XLATT = XLATTDEG/DEG;
XT = A*cos(XLATT)*cos(XLONGT);  YT = A*cos(XLATT)*sin(XLONGT);  ZT = A*sin(XLATT);
DISTFKM = distance3dkm(XF, YF, ZF, XT, YT, ZT, A, FTPKM);
TFTOT   = 252. + .223*DISTFKM - (5.44E-6)*DISTFKM*DISTFKM;
TFTOT   = TFTOT + P.TLOFT;

% ---- restructuring 1: the target Lambert solve, ONCE ------------------------
[VRX, VRY, VRZ, itT, rsT] = lambert3d(XT, YT, ZT, TFTOT - 0., ...
                                      XF, YF, ZF, 0, GM, P.theta_min);
XTD = VRX;  YTD = VRY;  ZTD = VRZ;
VBOT = sqrt(XTD^2 + YTD^2 + ZTD^2)/FTPKM;

% ---- restructuring 2: KEPLER1 per DISTINCT TF -------------------------------
tfCell = cell(1, numel(P.tlaunch));
for k = 1:numel(P.tlaunch)
    tfCell{k} = (P.tlaunch(k) + P.lead):P.dTF:TFTOT;
end
tfAll = unique([tfCell{:}]);       % bit-identical values, so == is safe below
ntf   = numel(tfAll);
XTFv = zeros(1,ntf);  YTFv = XTFv;  ZTFv = XTFv;  altV = XTFv;  altX = XTFv;
X0   = [XT YT ZT XTD YTD ZTD]/FTPKM;
for k = 1:ntf
    X1 = kepler1(X0, 0., tfAll(k), P.GMX);
    XTFv(k) = X1(1)*FTPKM;  YTFv(k) = X1(2)*FTPKM;  ZTFv(k) = X1(3)*FTPKM;
    % BOTH readings of the altitude line -- see OCR resolution 3.
    altV(k) = (sqrt(XT^2      + YTFv(k)^2 + ZTFv(k)^2) - A)/FTPKM;  % as printed
    altX(k) = (sqrt(XTFv(k)^2 + YTFv(k)^2 + ZTFv(k)^2) - A)/FTPKM;  % Listing 32.5
end

[pairTL, pairTF, pairIx] = pairList(P, tfCell, tfAll, altV, altX, P.xtf);

% ---- the grid.  One Lambert solve per (lon, lat, TLAUNCH, TF). --------------
nlat = numel(P.lat);  nlon = numel(P.lon);  npair = numel(pairTF);
minV = inf(nlat, nlon);  bTF = nan(nlat, nlon);  bTL = nan(nlat, nlon);
bRS  = nan(nlat, nlon);
nlam = 0;  nnoconv = 0;  ndegen = 0;  maxit = 0;

for i = 1:nlat
    XLATM = P.lat(i)/DEG;
    for j = 1:nlon
        XLONGM = P.lon(j)/DEG;
        XM = (A+ALTM)*cos(XLATM)*cos(XLONGM);
        YM = (A+ALTM)*cos(XLATM)*sin(XLONGM);
        ZM = (A+ALTM)*sin(XLATM);
        for p = 1:npair
            q = pairIx(p);
            [vx, vy, vz, it, rs] = lambert3d(XM, YM, ZM, pairTF(p)-pairTL(p), ...
                XTFv(q), YTFv(q), ZTFv(q), 0, GM, P.theta_min);
            nlam  = nlam + 1;
            maxit = max(maxit, it);
            if ~isfinite(vx)
                ndegen = ndegen + 1;   % near-radial or off the real branch
                continue
            end
            if it > 100, nnoconv = nnoconv + 1; end
            VBOM = sqrt(vx^2 + vy^2 + vz^2)/FTPKM;
            if VBOM < minV(i,j)
                minV(i,j) = VBOM;    bTF(i,j) = pairTF(p);
                bTL(i,j)  = pairTL(p);  bRS(i,j) = rs;
            end
        end
    end
end

S = struct('TFTOT', TFTOT, 'DISTFKM', DISTFKM, 'VBOT', VBOT, 'itT', itT, ...
    'rsT', rsT, 'tf', tfAll, 'XTF', XTFv, 'YTF', YTFv, 'ZTF', ZTFv, ...
    'altV', altV, 'altX', altX, 'pairTL', pairTL, 'pairTF', pairTF, ...
    'pairIx', pairIx, 'minVBOM', minV, 'bestTF', bTF, 'bestTL', bTL, ...
    'bestResid', bRS, 'nlam', nlam, 'nnoconv', nnoconv, 'ndegen', ndegen, ...
    'maxit', maxit, 'XT', XT, 'YT', YT, 'ZT', ZT, 'ALTM', ALTM);
end


% =============================================================================
function [pairTL, pairTF, pairIx] = pairList(P, tfCell, tfAll, altV, altX, useXTF)
%PAIRLIST  The listing's inner "for TF = (TLAUNCH+60.):50:TFTOT" together with
%   its 50-km BREAK.  Returned as a flat list so that the grid loop carries no
%   control flow at all.  Called twice -- once per altitude datum -- by
%   SELFCHECK 3, which is why the datum is an ARGUMENT and not read from P.
if useXTF, alt = altX; else, alt = altV; end
pairTL = [];  pairTF = [];  pairIx = [];
for k = 1:numel(P.tlaunch)
    for tf = tfCell{k}
        q = find(tfAll == tf, 1);
        if alt(q) < P.altmin
            break                     % the listing's break: leaves the TF loop
        end
        pairTL(end+1) = P.tlaunch(k);  pairTF(end+1) = tf;  pairIx(end+1) = q; %#ok<AGROW>
    end
end
end


% =============================================================================
function [VRX, VRY, VRZ, ITERS, RESID, GAMMA] = lambert3d(XT, YT, ZT, TF, ...
                                              XF, YF, ZF, SWITCH1, GM, THMIN)
%LAMBERT3D  Zarchan Listing 28.2, zarchan.txt:10428.  The same transcription as
%   mgs.zarchan.impulsiveIcbm carries (its file header holds the nine OCR
%   resolutions and the delete experiment that earns the two bracket updates),
%   plus the guard described as (a) in this file's header: a near-radial
%   geometry makes CROSSMAG -> 0 and C2 = MAG1*SINA/CROSSMAG return Inf, and a
%   solve that leaves the real branch returns a COMPLEX velocity that
%   isfinite() would still accept.  Both come back NaN and are counted.
PI_ = 3.1415926535898;
HALFPI = PI_/2.;
RF0X = XF-XT;  RF0Y = YF-YT;  RF0Z = ZF-ZT;
R0DOTRF  = XT*XF + YT*YF + ZT*ZF;
R0DOTRF0 = XT*RF0X + YT*RF0Y + ZT*RF0Z;
R0MAG  = sqrt(XT^2+YT^2+ZT^2);
RFMAG  = sqrt(XF^2+YF^2+ZF^2);
RF0MAG = sqrt(RF0X^2+RF0Y^2+RF0Z^2);
RATIO = R0MAG/RFMAG;  GMDIVR0 = GM/R0MAG;
COS_T = R0DOTRF/(R0MAG*RFMAG);
COS_T = max(-1., min(1., COS_T));
THETA0 = acos(COS_T);

VRX = NaN;  VRY = NaN;  VRZ = NaN;  ITERS = 0;  RESID = NaN;  GAMMA = NaN;
if RF0MAG <= 0 || THETA0 < THMIN || (PI_ - THETA0) < THMIN
    return                                   % near-radial: no usable solution
end

VNUMER = GMDIVR0*(1.-COS_T);  T_MIN = 0.;
if SWITCH1 == 0
    G_MIN = HALFPI - acos(R0DOTRF0/(R0MAG*RF0MAG));  G_MAX = HALFPI;
    THETA = THETA0;
else
    G_MIN = -HALFPI;  G_MAX = -HALFPI + acos(R0DOTRF0/(R0MAG*RF0MAG));
    THETA = 2.*PI_ - THETA0;
end
SIN_T = sin(THETA);  COT_HALFT = 1./tan(THETA/2.);
GAMMA = (G_MAX+G_MIN)/2.;  GOLD = G_MIN;  TOLD = 0.;  T = 0.;  ITERS = 1;
RV0MAG = NaN;
% The listing also forms S, BL and BT here.  None of the three is ever read.
while abs(TF-T) > (0.00000001*TF)
    SIN_G = sin(GAMMA);  COS_G = cos(GAMMA);  TAN_G = SIN_G/COS_G;
    COS_TPLUSG = cos(THETA+GAMMA-2.*PI_);
    TERM1 = (RATIO*COS_G - COS_TPLUSG)*COS_G;
    RV0MAG = sqrt(VNUMER/TERM1);
    LAMBDA = RV0MAG*RV0MAG/GMDIVR0;
    if LAMBDA < 1.9999999                            % elliptic
        TERM0 = sqrt(2./LAMBDA-1.);
        TERM1 = (TAN_G*(1.-COS_T)+(1.-LAMBDA)*SIN_T)/((2.-LAMBDA)*RATIO);
        TERM2 = (COS_G+COS_G)/(LAMBDA*TERM0*TERM0*TERM0);
        TERM3 = atan2(TERM0, (COS_G*COT_HALFT-SIN_G));
        T = (R0MAG/(RV0MAG*COS_G))*(TERM1+TERM2*TERM3);
    elseif LAMBDA > 2.0000001                        % hyperbolic
        TERM0 = sqrt(1.-2./LAMBDA);
        TERM1 = (TAN_G*(1.-COS_T)+(1.-LAMBDA)*SIN_T)/((2.-LAMBDA)*RATIO);
        TERM2 = COS_G/(LAMBDA*TERM0*TERM0*TERM0);
        TERM3 = SIN_G - COS_G*COT_HALFT;
        TERM3 = log((TERM3-TERM0)/(TERM3+TERM0));
        T = (R0MAG/(RV0MAG*COS_G))*(TERM1-TERM2*TERM3);
    else                                             % parabolic
        TERM0 = COS_G*COT_HALFT;  TERM1 = TERM0 - SIN_G;
        TERM0 = (3*TERM0*TERM1+1.)/(TERM1*TERM1*TERM1);
        T = TERM0*(2.*R0MAG)/(3.*RV0MAG);
    end
    % Bracket update.  T rises with GAMMA, so a T that is too LONG lowers the
    % ceiling and a T that is too SHORT raises the floor.
    if T > TF && GAMMA < G_MAX,  G_MAX = GAMMA;              end
    if T < 0.  && GAMMA < G_MAX,  G_MAX = GAMMA;             end
    if T < TF && GAMMA > G_MIN,  G_MIN = GAMMA;  T_MIN = T;  end
    if T < 0.
        NEXT = (G_MIN+G_MAX)/2.;  GOLD = G_MIN;  TOLD = T_MIN;
    else
        NEXT = GAMMA + (TF-T)*(GAMMA-GOLD)/(T-TOLD);   % secant
        if NEXT >= G_MAX
            NEXT = (GAMMA+G_MAX)/2.;                   % ...clamped to bisection
        elseif NEXT <= G_MIN
            NEXT = (GAMMA+G_MIN)/2.;
        end
        GOLD = GAMMA;  TOLD = T;
    end
    GAMMA = NEXT;  ITERS = ITERS + 1;
    if ITERS > 100
        break
    end
end
RESID = abs(TF-T)/abs(TF);
if ~isreal(RV0MAG) || ~isfinite(RV0MAG)
    return                                   % off the real branch: guard (a)
end

% Unit vector at flight-path angle GAMMA above the local horizontal, built in
% the (r0, rf) plane as C1*r0 + C2*rf.  MAG1 and DOTMAG are the listing's own
% |r0| and r0.rf, already in hand above.  (The book's "GAMMA=GAMMA;" is a no-op
% and the missing semicolon after its second CROSSX is a display leak.)
if SWITCH1 == 0
    ANGLE = HALFPI - GAMMA;
else
    ANGLE = GAMMA - HALFPI;
end
SINA = sin(ANGLE);  COSA = cos(ANGLE);
MAG1   = R0MAG;
DOTMAG = R0DOTRF;
CROSSX = YT*ZF - ZT*YF;
CROSSY = ZT*XF - XT*ZF;
CROSSZ = XT*YF - YT*XF;
CROSSMAG = sqrt(CROSSX*CROSSX+CROSSY*CROSSY+CROSSZ*CROSSZ);
if CROSSMAG <= 0
    return                                   % collinear: guard (a)
end
C2 = MAG1*SINA/CROSSMAG;
C1 = COSA/MAG1 - DOTMAG*C2/(MAG1*MAG1);
VRX = RV0MAG*(C2*XF + C1*XT);
VRY = RV0MAG*(C2*YF + C1*YT);
VRZ = RV0MAG*(C2*ZF + C1*ZT);
end


% =============================================================================
function X1 = kepler1(X0, T0, T1, GMX)
%KEPLER1  Listing 28.3's subroutine, zarchan.txt:10486-10505.  Universal
%   variables with Laguerre iteration (n = 5), KN = 10 fixed passes, no
%   convergence test.  km and km/s in and out.  REX is a nondimensionalising
%   scale and cancels exactly; GMX = 398923 km^3/s^2 is the sole physical
%   constant, and it sits 1.14e-6 above GM/3280^3.  The same transcription as
%   mgs.zarchan.keplerVsDirect, which scores it against direct integration.
REX = 6380.;  TLIMIT = 1.E-10;  KN = 10;  MUQR = 1.;
DT = T1 - T0;
if abs(DT) < TLIMIT
    X1 = X0;      % RETURN added.  As printed this falls through, and the
    return        % fall-through divides by DT = 0.
end

TIME_FACTOR = sqrt(REX^3/GMX);
VEX = REX/TIME_FACTOR;
DT  = DT/TIME_FACTOR;
R0  = X0(1:3)/REX;
V0  = X0(4:6)/VEX;

R0MAG  = sqrt(R0(1)^2 + R0(2)^2 + R0(3)^2);
V0MAG  = sqrt(V0(1)^2 + V0(2)^2 + V0(3)^2);
D0     = R0(1)*V0(1) + R0(2)*V0(2) + R0(3)*V0(3);
SIGMA0 = D0/MUQR;
ALP0   = 2./R0MAG - V0MAG*V0MAG;      % reciprocal semi-major axis

X = ALP0*DT;
if ALP0 <= 0.
    X = .1*DT/R0MAG;
end

U1 = 0.;  U2 = 0.;  U3 = 0.;  DFX = 0.;
for K = 1:KN
    if ALP0 < 0.
        Y  = ALP0*X*X;   YQR = sqrt(-Y);
        CY = (1.-cosh(YQR))/Y;
        SY = (sinh(YQR) - YQR)/(YQR^3);
    elseif ALP0 == 0.
        Y = 0.;  CY = .5;  SY = 1./6.;
    else
        Y  = ALP0*X*X;   YQR = sqrt(Y);
        CY = (1.-cos(YQR))/Y;
        SY = (YQR - sin(YQR))/(YQR^3);
    end
    U1 = X*(1. - Y*SY);
    U2 = X*X*CY;
    U3 = X*X*X*SY;
    FX   = R0MAG*U1 + SIGMA0*U2 + U3 - DT*MUQR;
    DFX  = SIGMA0*U1 + (1. - ALP0*R0MAG)*U2 + R0MAG;
    DFX2 = SIGMA0*(1. - Y*CY) + (1. - ALP0*R0MAG)*U1;
    SDFX = DFX/(abs(DFX));
    DX2  = 16.*DFX*DFX - 20.*FX*DFX2;
    if DX2 > 0.
        DX = 5.*FX/(DFX + SDFX*sqrt(DX2));
    else
        DX = .5*X;
    end
    X = X - DX;
end

RMAG = DFX;
F  = 1. - U2/R0MAG;
G  = DT - U3/MUQR;
DF = -MUQR*U1/(RMAG*R0MAG);
DG = 1. - U2/RMAG;
X1 = zeros(1,6);
X1(1:3) = (F*R0  + G*V0 )*REX;
X1(4:6) = (DF*R0 + DG*V0)*VEX;
end


% =============================================================================
function DISTKM = distance3dkm(XT, YT, ZT, XF, YF, ZF, A, FTPKM)
%DISTANCE3DKM  Zarchan Listing 28.2, zarchan.txt:10423.  Great-circle arc at
%   the Earth radius A.  "if CBETA,=1." -> "<=": the else branch exists only to
%   catch a rounding overshoot that would turn acos() complex.
R  = sqrt(XT^2+YT^2+ZT^2);
RF = sqrt(XF^2+YF^2+ZF^2);
CBETA = (XT*XF+YT*YF+ZT*ZF)/(R*RF);
if CBETA <= 1.
    DISTKM = A*acos(CBETA)/FTPKM;
else
    DISTKM = (XF-XT)/FTPKM;
end
end


% =============================================================================
function d = extentOf(mask, dr, fn)
%EXTENTOF  min/max downrange over the columns holding at least one hit.
c = any(mask, 1);
if any(c), d = fn(dr(c)); else, d = NaN; end
end


% =============================================================================
function K = structuralChecks(R, S, P)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.
%   /!\ .names and .oks must BOTH exist and be the SAME LENGTH, or benchSuite
%   puts ZERO rows on the board and the board looks green for the wrong reason.
names = {};  oks = [];  detail = {};

% 1  LAMBERT3D met its own 1e-8 criterion at every argmin that DEFINES a point
%    of the widest operational area.  The book's routine cannot tell a
%    converged answer from its ITERS > 100 break, so without this row a
%    non-converged 101st iterate could be quietly supplying an extent.
%
%    ⚠️ THIS ROW IS GREEN WHILE ITS OWN DETAIL PRINTS "152 hit the ITERS>100
%    break".  That is correct and here is why, because it looks wrong:
%    the 152 are INNER-LOOP PROBES.  74,214 Lambert calls come from the
%    TF/TLAUNCH search run at every grid point; the 152 that ran out of
%    iterations are probes at flight times with no transfer, which is how the
%    search learns where the boundary is.  What the row scores is the 156
%    points that SURVIVE into the area (worst residual 9.49e-09), and none of
%    those came from a break.
%
%    ⭐⭐ WHY THERE IS NO NEAR-PARABOLIC (lambda) GUARD HERE, unlike the six
%    ports that transcribe Listing 13.2's lambertpz.  Those six were given one
%    after a measured failure: all six returned converged = true on a transfer
%    that needs lambda = 2.014 (hyperbolic) while the listing solves ellipses
%    only, the secant stalled at lambda = 1.999999919 where the closed form is
%    ill-conditioned, the residual test passed at 1e-8, and the answer missed
%    by 98,751 ft.  A small residual is NOT sufficient in that collar.
%
%    ⇒ So: is THIS chapter exposed?  Measured, no.  lambda = 2*(v/v_esc)^2
%    with v_esc = sqrt(2*GM/A) = 11.18 km/s, and the largest VBOM anywhere on
%    this grid is 7.31 km/s:
%
%        lambda_max = 2*(7.31/11.18)^2 = 0.855
%
%    Every geometry Ch.32 flies is deeply elliptical -- the collar at 1.999999
%    is more than a factor of two away in lambda, and the interceptor is
%    velocity-LIMITED, so it cannot be driven there.  The residual criterion is
%    sufficient here.  ⚠️ If anyone raises VBOLIM far enough to approach
%    11 km/s, that stops being true and this port needs the guard.
M1    = R.area(1).mask;
rs    = S.bestResid(M1);
worst = maxOr(rs);
names{end+1} = 'LAMBERT3D meets its own 1e-8 criterion at every scored grid point';
oks(end+1)   = isfinite(S.VBOT) && S.rsT <= 1e-8 && (isempty(rs) || worst <= 1e-8);
detail{end+1} = sprintf(['target solve resid %.2e in %d iters; worst resid over ' ...
    '%d area points %.2e; %d calls, %d hit the ITERS>100 break, %d degenerate ' ...
    'skips, max iters %d'], S.rsT, S.itT, numel(rs), worst, S.nlam, S.nnoconv, ...
    S.ndegen, S.maxit);

% 2  The downrange extent is attained on the ZERO-latitude row.  This is what
%    licenses the coarse default latitude grid: if it holds, no latitude
%    refinement can move a scored row.  If it ever fails, the extents become
%    grid-dependent and every extent row below is suspect.
e2 = true;  txt2 = '';
for k = 1:numel(R.area)
    a    = R.area(k);
    same = eqOrBothNaN(a.dmin_km, a.dmin0_km) && eqOrBothNaN(a.dmax_km, a.dmax0_km);
    e2   = e2 && same;
    txt2 = [txt2 sprintf('%g km/s: all-lat [%s], lat0 [%s]%s;  ', a.VBOLIM, ...
        kmpair(a.dmin_km, a.dmax_km), kmpair(a.dmin0_km, a.dmax0_km), ...
        tern(same, '', ' <-- DIFFER'))]; %#ok<AGROW>
end
names{end+1} = 'the downrange extent is attained on the zero-latitude row';
oks(end+1)   = e2;
detail{end+1} = strtrim(txt2);

% 3  The altitude datum -- OCR resolution 3.  Two facts, one row: the 50-km
%    gate does not fire under Listing 32.5's physical reading, and the book's
%    printed XT reading accepts exactly the same (TLAUNCH, TF) pairs.  If the
%    second half ever fails, the typo has started to matter and the extents
%    depend on which reading is in force.
tfCell = cell(1, numel(P.tlaunch));
for k = 1:numel(P.tlaunch)
    tfCell{k} = (P.tlaunch(k) + P.lead):P.dTF:S.TFTOT;
end
[tlV, tfV] = pairList(P, tfCell, S.tf, S.altV, S.altX, false);
[tlX, tfX] = pairList(P, tfCell, S.tf, S.altV, S.altX, true);
agree = isequal([tlV(:) tfV(:)], [tlX(:) tfX(:)]);
minX  = minOr(S.altX);
names{end+1} = 'the 50-km gate never fires, and both altitude datums accept the same pairs';
oks(end+1)   = agree && minX >= P.altmin;
detail{end+1} = sprintf(['min true altitude over the TF grid %.2f km (gate %g km); ' ...
    'verbatim-datum min %.2f km; pairs accepted %d (XT) vs %d (XTF), %s'], ...
    minX, P.altmin, minOr(S.altV), numel(tfV), numel(tfX), ...
    tern(agree, 'identical', 'DIFFERENT'));

% 4  The missile KEPLER1 propagation this port DROPPED as dead code (see
%    restructuring 3) is run once, at the easiest grid point, and required to
%    land on the predicted intercept point.  That is the end-to-end check on
%    the whole Lambert -> Kepler -> Lambert chain: a mis-wired chain misses by
%    thousands of km.  Band 5 km -- roughly three orders above the ~0.01 km
%    the GMX/GM mismatch predicts and three below the failure it is for.
[i4, j4]     = bestPoint(S.minVBOM);
[miss4, vb4] = rendezvousCheck(S, P, i4, j4);
names{end+1} = 'the dropped missile KEPLER1 propagation lands on the intercept point';
oks(end+1)   = isfinite(miss4) && miss4 <= 5.0;
detail{end+1} = sprintf(['at %g km downrange, %g km crossrange (VBOM %.4f km/s, ' ...
    'TLAUNCH %g s, TF %g s): |r_missile(TF) - r_target(TF)| = %.4f km'], ...
    P.lon(j4)*P.KMPDEG, P.lat(i4)*P.KMPDEG, vb4, S.bestTL(i4,j4), ...
    S.bestTF(i4,j4), miss4);

% 5  DELETE EXPERIMENT: remove the PREDICTION.  Aim the interceptor's Lambert
%    solve at the target's LAUNCH point instead of at KEPLER1's predicted
%    intercept point, at the same grid point, over the same pairs.  If the
%    prediction were not wired in, this would change nothing.
vbDel = deleteExperiment(S, P, i4, j4);
names{end+1} = 'DELETE the prediction (aim at the target LAUNCH point) -> min VBOM jumps';
oks(end+1)   = isfinite(vbDel) && vbDel >= 1.5*vb4;
detail{end+1} = sprintf(['min VBOM %.4f -> %.4f km/s (%+.0f%%); the deleted ' ...
    'variant is above every VBOLIM scored, so that point leaves the area'], ...
    vb4, vbDel, 100*(vbDel/vb4 - 1));

% 6  THE LATTICE QUANTISES THE EDGES; IT DOES NOT CREATE THEM.
%    Every extent on the board is a multiple of one 5-deg lattice step, so the
%    natural question about the one RED row is whether a finer grid would move
%    it.  The property that licenses the coarse grid is that each upper edge is
%    ONE MONOTONE CROSSING: refine the bracket between the last included and
%    the first excluded longitude ten times over, and min VBOM must rise
%    monotonically across it and cross the gate exactly once.  If it wiggled,
%    or crossed more than once, the coarse edge would be an aliased sample and
%    every extent row would be grid-dependent.
%
%    This is scored on the profile MINIMISED OVER LATITUDE, so it does not
%    inherit check 2's assumption about the zero-latitude row.
REF   = 10;
vAll  = min(R.minVBOM, [], 1);              % 1 x nlon
lonRef = [];  brack = nan(numel(R.area), 2);
for k = 1:numel(R.area)
    j = find(vAll < R.area(k).VBOLIM, 1, 'last');
    if ~isempty(j) && j < numel(vAll)
        brack(k,:) = [P.lon(j) P.lon(j+1)];
        lonRef = [lonRef, linspace(P.lon(j), P.lon(j+1), REF+1)]; %#ok<AGROW>
    end
end
bookLonDeg = 10500/P.KMPDEG;                % the failing row's own book value
lonRef = unique([lonRef, bookLonDeg]);
P2 = P;  P2.lon = lonRef;
S2 = sweepGrid(P2);
v2 = min(S2.minVBOM, [], 1);

K.refine_cross   = nan(1, numel(R.area));
K.refine_edge_km = nan(1, numel(R.area));
mono = true;  once = true;  evaluable = false;
txt6 = '';
for k = 1:numel(R.area)
    if ~isfinite(brack(k,1)), continue, end
    evaluable = true;
    w  = lonRef >= brack(k,1) - 1e-12 & lonRef <= brack(k,2) + 1e-12;
    vb = v2(w);  lb = lonRef(w);
    mono = mono && all(diff(vb) > 0);
    sgn  = (vb < R.area(k).VBOLIM);
    K.refine_cross(k) = sum(abs(diff(sgn)));         % sign changes in the bracket
    once = once && (K.refine_cross(k) == 1);
    ji   = find(sgn, 1, 'last');
    if ~isempty(ji), K.refine_edge_km(k) = lb(ji)*P.KMPDEG; end
    txt6 = [txt6 sprintf('%g km/s: %.0f -> %.0f km (%d crossing, %s);  ', ...
        R.area(k).VBOLIM, R.area(k).dmax_km, K.refine_edge_km(k), ...
        K.refine_cross(k), tern(all(diff(vb) > 0), 'monotone', 'NOT MONOTONE'))]; %#ok<AGROW>
end
names{end+1} = 'each upper edge is ONE monotone crossing (10x lattice refinement)';
oks(end+1)   = evaluable && mono && once;
% The string reports the MEASUREMENT and the REQUIREMENT, never the verdict.
% An earlier draft ended "...so each coarse edge is the quantisation of a
% single real crossing", which printed unchanged beside "NOT MONOTONE" under
% fault injection -- a conclusion hard-coded into an sprintf is exactly as
% unfalsifiable as a check that cannot fail.
detail{end+1} = sprintf(['coarse edge -> refined edge at %g x the lattice ' ...
    '(each bracket must be monotone with exactly 1 crossing): %s'], REF, txt6);

% 7  ...AND THE ONE RED ROW IS NOT A NEAR-THRESHOLD COIN FLIP.  The book puts
%    the 4-km/s upper edge at 10,500 km; this model puts it at 11,100 km, 600
%    km out and 45 km beyond the +/- one-lattice-step band.  A disagreement
%    that size is only worth calling a RESULT if the book's own edge point is
%    nowhere near being a boundary here, so that is measured directly: min VBOM
%    AT 10,500 km must sit at least 10% below the 4.000 km/s gate.  If it ever
%    creeps up to the gate this row goes red and the extent row becomes a
%    rounding argument instead of a finding.
%
%    Emitted only when 4 km/s is among the thresholds actually run -- a row
%    that quietly evaluates to "true" off the book case would be a row that
%    cannot fail.
q4 = find(abs([R.area.VBOLIM] - 4) < 1e-12, 1);
if ~isempty(q4)
    [~, jb] = min(abs(lonRef - bookLonDeg));
    K.book_edge_vbom = v2(jb);
    K.book_edge_frac = K.book_edge_vbom/4.0;
    names{end+1} = 'the book''s 4 km/s upper edge (10,500 km) is deep inside this model''s area';
    oks(end+1)   = isfinite(K.book_edge_vbom) && K.book_edge_frac <= 0.90;
    detail{end+1} = sprintf(['min VBOM at %.0f km downrange = %.4f km/s, ' ...
        '%.1f%% of the 4.000 gate (must be <= 90%%); our own edge at %.0f km ' ...
        'reads %.4f and the first excluded point %.0f km reads %.4f'], ...
        lonRef(jb)*P.KMPDEG, K.book_edge_vbom, 100*K.book_edge_frac, ...
        R.area(q4).dmax_km, vAll(find(vAll < 4, 1, 'last')), ...
        P.lon(find(vAll < 4, 1, 'last')+1)*P.KMPDEG, ...
        vAll(find(vAll < 4, 1, 'last')+1));
end

K.names = names;  K.oks = logical(oks);  K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    q = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{q}, detail{q});
end
end


% =============================================================================
function [miss_km, vb] = rendezvousCheck(S, P, i, j)
%RENDEZVOUSCHECK  SELFCHECK 4.  Re-solves the interceptor Lambert at one grid
%   point and then runs the listing's DROPPED "CALCULATE MISSILE STATES AT
%   DESIRED INTERCEPT TIME" block (:12342) exactly once.
A = P.A;  FTPKM = P.FTPKM;  DEG = P.DEG;
XLATM = P.lat(i)/DEG;  XLONGM = P.lon(j)/DEG;
XM = (A+S.ALTM)*cos(XLATM)*cos(XLONGM);
YM = (A+S.ALTM)*cos(XLATM)*sin(XLONGM);
ZM = (A+S.ALTM)*sin(XLATM);
TL = S.bestTL(i,j);  TFq = S.bestTF(i,j);
vb = S.minVBOM(i,j);
q  = [];
if isfinite(TFq), q = find(S.tf == TFq, 1); end
if isempty(q) || ~isfinite(vb)
    miss_km = NaN;  return
end
[XMD, YMD, ZMD] = lambert3d(XM, YM, ZM, TFq-TL, S.XTF(q), S.YTF(q), S.ZTF(q), ...
                            0, P.GM, P.theta_min);
X0 = [XM YM ZM XMD YMD ZMD]/FTPKM;
X1 = kepler1(X0, TL, TFq, P.GMX);
miss_km = sqrt((X1(1)-S.XTF(q)/FTPKM)^2 + (X1(2)-S.YTF(q)/FTPKM)^2 + ...
               (X1(3)-S.ZTF(q)/FTPKM)^2);
end


% =============================================================================
function vb = deleteExperiment(S, P, i, j)
%DELETEEXPERIMENT  SELFCHECK 5.  The same sweep over the same (TLAUNCH,TF)
%   pairs at one grid point, with KEPLER1's predicted intercept point replaced
%   by the target's LAUNCH point.  Nothing else changes.
A = P.A;  FTPKM = P.FTPKM;  DEG = P.DEG;
XLATM = P.lat(i)/DEG;  XLONGM = P.lon(j)/DEG;
XM = (A+S.ALTM)*cos(XLATM)*cos(XLONGM);
YM = (A+S.ALTM)*cos(XLATM)*sin(XLONGM);
ZM = (A+S.ALTM)*sin(XLATM);
vb = Inf;
for p = 1:numel(S.pairTF)
    [vx, vy, vz] = lambert3d(XM, YM, ZM, S.pairTF(p)-S.pairTL(p), ...
                             S.XT, S.YT, S.ZT, 0, P.GM, P.theta_min);
    if ~isfinite(vx), continue, end
    vb = min(vb, sqrt(vx^2+vy^2+vz^2)/FTPKM);
end
end


% =============================================================================
function B = benchmarks(R, S, P)
%BENCHMARKS  Every book value grepped from zarchan_decoded.txt; the line number
%   is in each row's .source.  Nothing here was read off a figure IMAGE -- the
%   extraction turned every figure into an image link.  The extents ARE prose
%   claims ABOUT figures, which is a different thing, and greppable.
B = emptyBench();

% ---- the band.  One 5-deg longitude lattice step = 555 km; see the header. --
BOOKSTEP = 5*P.KMPDEG;
runStep  = maxOr(diff(P.lon))*P.KMPDEG;
canScore = isfinite(runStep) && runStep <= BOOKSTEP + 1e-9;
bandStr  = sprintf('+/-%g km (one 5-deg lattice step)', BOOKSTEP);

% 1  The target itself.  The same 10,000-km minimum-energy arc as Listing 32.1,
%    so :12172's 7.19 km/s applies here, and it is the cheapest row on the
%    board: if the hoisted target solve were miswired every extent moves with
%    it, and this row says so in one number.
B = addVal(B, 'target burnout velocity VBOT (minimum energy, 10,000 km)', ...
    7.19, S.VBOT, '0.5%', 0.005, ...
    'Listing 32.1 discussion, p.837; zarchan.txt:12172', 'km/s', ...
    sprintf(['Lambert only, integrates nothing; TFTOT = %.1f s from tFME, ' ...
    'DISTFKM = %.4f km.  This is also the quantity the 7.5 km/s gate tests.'], ...
    S.TFTOT, S.DISTFKM));

% 2-7  The published operational-area extents.
bk = struct( ...
  'V',    {6, 6, 5, 5, 4, 4}, ...
  'side', {'min', 'max', 'min', 'max', 'min', 'max'}, ...
  'km',   {3000, 14500, 3500, 12500, 7500, 10500}, ...
  'src',  {'Fig. 32.7/32.8 discussion, zarchan.txt:12352 (:12349 states the same lower edge, "at zero latitude extends from 3000 km")', ...
           'Fig. 32.7/32.8 discussion, zarchan.txt:12352 (:12349 says only "more than 14,000 km"; the specific 14,500 is what is scored, both cited)', ...
           'Fig. 32.8 discussion, zarchan.txt:12352', ...
           'Fig. 32.8 discussion, zarchan.txt:12352', ...
           'Fig. 32.9 discussion, zarchan.txt:12390', ...
           'Fig. 32.9 discussion, zarchan.txt:12390'});
for k = 1:numel(bk)
    what = sprintf('operational area %s downrange, %g km/s interceptor', ...
        bk(k).side, bk(k).V);
    q = find(abs([R.area.VBOLIM] - bk(k).V) < 1e-12, 1);
    if isempty(q)
        B = addSkip(B, what, sprintf('%g', bk(k).km), ...
            sprintf('VBOLIM = %g km/s was not among the thresholds run', bk(k).V), ...
            bk(k).src, 'km');
    elseif ~canScore
        B = addSkip(B, what, sprintf('%g', bk(k).km), sprintf(['the longitude ' ...
            'grid steps %g km, coarser than the book''s %g km, so this run ' ...
            'cannot resolve the extent to the band the claim deserves.  ' ...
            'Scoring it would mean widening the band to match a grid the ' ...
            'caller chose.'], runStep, BOOKSTEP), bk(k).src, 'km');
    else
        if strcmp(bk(k).side, 'min'), ours = R.area(q).dmin_km;
        else,                         ours = R.area(q).dmax_km;
        end
        B = addVal(B, what, bk(k).km, ours, bandStr, BOOKSTEP/bk(k).km, ...
            bk(k).src, 'km', sprintf(['%d grid points inside; every extent ' ...
            'sits on the 5-deg lattice, so it can only be a multiple of %g km'], ...
            R.area(q).npts, BOOKSTEP));
    end
end

% 8  The listing's own degree-to-km conversion, checked at the far edge.
B = addBound(B, '150 deg of longitude is more than 16,000 km downrange', ...
    '> 16000', sprintf('%.0f', 150*P.KMPDEG), 150*P.KMPDEG > 16000, ...
    'zarchan.txt:12325', 'km', sprintf(['the listing''s ArrayXLONGM = ' ...
    'XLONGMDEG*111; the model Earth''s own arc is %.2f km/deg, so the plot ' ...
    'axis runs 0.3%% short of the sphere it was flown over'], ...
    P.A*pi/180/P.FTPKM));

% 9  ORDERING, from the prose about Figs. 32.8 and 32.9.
n = [R.area.npts];
B = addShape(B, 'the operational area shrinks as the interceptor gets slower', ...
    'the operational area shrinks considerably (6 -> 5 km/s) and reduces even more dramatically (5 -> 4 km/s)', ...
    sprintf('%s points at VBOLIM = %s km/s', strtrim(sprintf('%d ', n)), ...
        strtrim(sprintf('%g ', [R.area.VBOLIM]))), ...
    numel(n) >= 2 && all(diff(n) < 0), 'zarchan.txt:12352 and :12353', ...
    ['an ORDERING claim, not a count: no figure publishes a number of plus ' ...
     'signs, and the count depends on the latitude grid']);

% 10  SYMMETRY, from the prose about Fig. 32.7.
sym = all([R.area.sym_ok]);
Mv  = R.minVBOM;  Mf = flipud(Mv);
fin = isfinite(Mv) & isfinite(Mf);
res = maxOr(abs(Mv(fin) - Mf(fin)));
B = addShape(B, 'the operational area is symmetrical about the target trajectory', ...
    'the operational area is symmetrical about the target trajectory', ...
    sprintf('mask symmetric in latitude: %s (max |minVBOM(+lat) - minVBOM(-lat)| = %.2e km/s)', ...
        tern(sym, 'yes', 'NO'), res), ...
    sym, 'zarchan.txt:12349', ...
    ['the target flies the equator, so this is exact rather than approximate; ' ...
     'it fails only if the latitude grid is asymmetric or ZM lost its sign']);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER are benchSuite's adaptGeneric schema, which
%   is STRICT: a near-miss (book_s for book_str) errors rather than adapting.
%   Every row here is built through one of the four constructors below so the
%   order cannot drift.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.9g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.9g', ours);
B(i).band = band;  B(i).tol = tol;
B(i).units = units;  B(i).source = source;  B(i).detail = detail;
if isempty(ours) || ~isfinite(ours) || book == 0
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

function B = addSkip(B, what, bookStr, why, source, units)
%ADDSKIP  A check that COULD NOT RUN.  Never a pass.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'skip';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = '--';
B(i).pct  = NaN;   B(i).band = 'skip';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true;  B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end


% =============================================================================
function s = render(R, ev)
L = {};
L{end+1} = 'Zarchan Listing 32.3 -- THREE-DIMENSIONAL OPERATIONAL AREAS';
L{end+1} = sprintf('   %s', R.units);
L{end+1} = sprintf(['   RDESKM = %g km, TLOFT = %g s, TFTOT = %.1f s, ' ...
    'VBOT = %.6f km/s'], R.p.RDESKM, R.p.TLOFT, R.target.TFTOT, R.target.VBOT_kms);
L{end+1} = sprintf(['   grid %d lon x %d lat x %d (TLAUNCH,TF) pairs = %d ' ...
    'Lambert solves; alt_datum "%s"'], numel(R.p.lon_deg), numel(R.p.lat_deg), ...
    R.p.n_pairs, R.p.n_lambert, R.p.alt_datum);
L{end+1} = '';
L{end+1} = '   VBOLIM     points     downrange extent        at zero latitude';
L{end+1} = '   [km/s]                     [km]                    [km]';
for k = 1:numel(R.area)
    a = R.area(k);
    L{end+1} = sprintf('   %6g %10d   %20s %20s', a.VBOLIM, a.npts, ...
        kmpair(a.dmin_km, a.dmax_km), kmpair(a.dmin0_km, a.dmax0_km)); %#ok<AGROW>
end
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %s', vstr(K.oks(i)), K.names{i}); %#ok<AGROW>
    L{end+1} = sprintf('        %s', K.detail{i});                %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning';
    L{end+1} = '            NOT CHECKED, which is not the same thing as "fine".';
else
    L{end+1} = 'BENCHMARKS  (book values grepped from zarchan_decoded.txt; lines in .source)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        switch lower(b.kind)
            case 'skip'
                L{end+1} = sprintf('   SKIP %-50s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why);                     %#ok<AGROW>
                L{end+1} = sprintf('        [%s]', b.source);                %#ok<AGROW>
            case {'shape', 'bound'}
                L{end+1} = sprintf('   %-4s %-50s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s', b.book_str);          %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail);  %#ok<AGROW>
            otherwise
                L{end+1} = sprintf(['   %-4s %-50s book %s %s / ours %s %s ' ...
                    '(%+.3f%%, band %s)'], vstr(b.ok), b.what, b.book_str, ...
                    b.units, b.ours_str, b.units, b.pct, b.band);            %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail);  %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: IMPULSIVE launch on both sides, KEPLERIAN point-mass gravity,';
L{end+1} = '   a ROUND and NON-ROTATING Earth, no atmosphere, no drag, no guidance';
L{end+1} = '   loop and no miss distance.  This file answers "could it be reached",';
L{end+1} = '   not "would it be hit".';
s = strjoin(L, newline);
end

function s = vstr(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end

function s = kmpair(a, b)
if ~isfinite(a) || ~isfinite(b)
    s = '(empty)';
else
    s = sprintf('%g .. %g', a, b);
end
end

function s = tern(c, a, b)
if c, s = a; else, s = b; end
end

function y = maxOr(x)
if isempty(x), y = NaN; else, y = max(x(:)); end
end

function y = minOr(x)
if isempty(x), y = NaN; else, y = min(x(:)); end
end

function t = eqOrBothNaN(a, b)
t = (isnan(a) && isnan(b)) || a == b;
end

function [i, j] = bestPoint(minV)
%BESTPOINT  The grid point that needs the least burnout velocity.  Used by
%   SELFCHECKS 4 and 5 so that both run where a solution certainly exists.
[~, q] = min(minV(:));
[i, j] = ind2sub(size(minV), q);
end

function drawArea(R)
figure;  hold on;
mk = {'r+', 'b+', 'g+'};
for k = 1:numel(R.area)
    [ii, jj] = find(R.area(k).mask);
    plot(R.downrange_km(jj), R.crossrange_km(ii), mk{1+mod(k-1,3)}, ...
        'DisplayName', sprintf('%g km/s interceptor', R.area(k).VBOLIM));
end
plot([0 R.p.RDESKM], [0 0], 'k-', 'LineWidth', 2, 'DisplayName', 'ICBM trajectory');
grid on;  xlabel('Downrange (km)');  ylabel('Crossrange (km)');
title(sprintf('Operational area, %g km minimum-energy ICBM', R.p.RDESKM));
legend('Location', 'best');  hold off;
end
