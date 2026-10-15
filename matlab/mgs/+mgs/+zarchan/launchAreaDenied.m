function R = launchAreaDenied(opts)
%MGS.ZARCHAN.LAUNCHAREADENIED  Zarchan Listing 32.4 -- three-dimensional
%   LAUNCH AREA DENIED results for an impulsive interceptor against an
%   impulsive ICBM.
%
%   R = mgs.zarchan.launchAreaDenied()
%   R = mgs.zarchan.launchAreaDenied('grid','book')        the book's own steps
%   R = mgs.zarchan.launchAreaDenied('VBOLIM',6,'assert',false)
%
%   Port of zarchan.txt:12394-12429 (transcription listing_32_04.m), with the
%   LAMBERT3D, distance3dkm and KEPLER1 subroutines it calls but does not print
%   (Listings 28.2/28.3, zarchan.txt:10423, :10428, :10486).
%
% ===========================================================================
%   UNITS ARE THE LISTING'S.  Positions and velocities are FEET and ft/s
%   (A = 2.0926E7 ft, GM = 1.4077E16 ft^3/s^2); the listing divides by its own
%   rounded 3280 ft/km to report km and km/s, and KEPLER1 alone works in km.
%   Areas are km^2 ON THE LISTING'S OWN PLOT SCALE, which is degrees x 111.
% ===========================================================================
%
%   WHAT THE THREE CHAPTER-32 AREA LISTINGS ACTUALLY ARE
%
%   32.3, 32.4 and 32.5 are ONE success predicate
%       S(target launch, target aimpoint, interceptor site, TLAUNCH, TF)
%   swept over three different 2-D slices:  32.3 varies the INTERCEPTOR SITE
%   (operational area), 32.4 varies the TARGET LAUNCH SITE (launch area
%   denied), 32.5 varies the TARGET AIMPOINT (defended area).  The book says so
%   outright -- ":12392 The operational area code of Listing 32.3 can be
%   modified so that launch area denied results can be generated", ":12475 The
%   launch area denied code of Listing 32.4 can be modified so that defended
%   area results can be generated".  The three slices INTERSECT at one
%   configuration (target launch 0/0 deg, aimpoint 10,000 km downrange,
%   interceptor 60 deg/0 deg) and BENCHMARK 8 scores exactly that: the three
%   drivers must return the same VBOT to the last bit and the same accepted
%   (TLAUNCH, TF) set.  A single area number could not test this at all.
%
%   They are not textually identical, and this port carries all three deltas:
%   32.5 ends its TF loop at TFTOT-50 where 32.3/32.4 end at TFTOT; 32.3/32.5
%   BREAK out of the TF loop on a low intercept altitude where 32.4 merely
%   SKIPS that TF; and 32.3/32.4 build ALTFKM from XT -- the target's LAUNCH
%   x-coordinate -- where 32.5 uses XTF, the intercept-point x.  That last one
%   is a BOOK DEFECT, ported AS PRINTED ('alt_x','book'); SELF-CHECK 5 measures
%   it (2618 km of apparent altitude) and 'alt_x','xtf' takes 32.5's spelling.
%
%   -------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_32_04.m does not parse.  Its own header reports
%   2 ligature residues, 4 other non-ASCII and 3 lines of prose inside the
%   code: the figure axis labels at its lines 11-24 and the running text at
%   44-82, dropped, not "fixed" into code.  Two comparisons are OCR "," for "<":
%
%     1  "if ~(ALTFKM , 50.)" -> "if ~(ALTFKM < 50.)", i.e. proceed only when
%        the intercept is at or above 50 km.  Forced by :12325 ("checks are
%        performed to ensure that intercepts are made above 50-km altitude")
%        and by 32.3/32.5, which spell the same gate "if ALTFKM<50, break".
%        The other reading would demand intercepts BELOW 50 km and empty the map.
%     2  "if (VBOM , VBOLIM & VBOT,7.5)" -> both "<".  Forced by :12325 ("an
%        interceptor with an initial velocity that cannot exceed VBOLIM") and
%        :12392 ("ruled out if the required impulsive target velocity exceeds
%        7.5 km/s").
%
%   The minus-sign damage class appears twice in the surrounding prose, and
%   BENCHMARK 5 resolves it BY MEASUREMENT rather than by taste: ":12392 from
%   260 deg to 60 deg in latitude" is -60 to 60 (the listing prints
%   -60:2.5:60), and the depressed case is TLOFT = -200, printed "-200" at
%   :12473 and "2200" at :12558.
%
%   Dead code dropped, not repaired: I=1, PI=3.14159, S=0., T=0. (T is read
%   once, as TGOLAM=TFTOT-T, and never advanced -- SELF-CHECK 4 locks that),
%   the unused GM assignment (LAMBERT3D carries its own), and "save datfil.txt
%   output /ascii", the book's recurring typo for -ascii.  Nothing is written
%   to disk here.
%
%   TWO DELIBERATE DEVIATIONS.  (a) The listing hoists nothing: it solves both
%   Lambert legs for every (cell, TLAUNCH, TF) and tests the conjunction at the
%   end.  VBOT depends on the CELL ONLY (TGOLAM = TFTOT - T with T = 0), so
%   this port evaluates "VBOT < 7.5" once per cell and skips the inner loops
%   when it fails -- exact, the conjunction is unchanged, and it is what makes
%   the sweep affordable.  (b) LAMBERT3D gets a bail-out when TERM1 goes
%   non-positive: as printed it would either sqrt a negative and hand a COMPLEX
%   VBOM to "VBOM < VBOLIM" (MATLAB then compares real parts, silently) or
%   throw inside atan2.  Bail-outs are counted, and the cell is not denied.
%
%   -------------------------------------------------------------------------
%   COST.  The book's grid is 61 x 49 cells x 14 launch times x ~32 intercept
%   times and every innermost combination is a Lambert solve.  The DEFAULT is
%   coarser ('grid','coarse': 10 deg, 5 deg, 300 s, 100 s -> 31 x 25 cells)
%   because every area row is an ORDERING, and orderings do not move with
%   resolution: loft/min = 0.115 on the book grid against 0.115 coarse,
%   depr/min = 1.229 against 1.250.  'grid','book' costs about ten times more.
%
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED IN MATLAB *** -- MATLAB
%   is the caller's, not this author's.  From an independent double-precision
%   transcription of the same listing, so a first run has an expectation to be
%   compared against rather than merely admired:
%     reference cell (launch 0/0, aim 89.8136 deg, TLOFT=0): DISTFKM 10000.000
%       km, TFTOT 1938.000 s, VBOT 7.193694 km/s, 94 accepted (TLAUNCH,TF);
%       apogee 1331.7 km (XTF form) against 2945.2 km (the book's XT form)
%     coarse grid: denied 96/775 cells, 5.914e7 km^2; lofted 11 / 6.777e6;
%       depressed 120 / 7.393e7.  Book grid: 407/2989, 6.268e7; 47 / 7.239e6;
%       500 / 7.701e7.
%     delete the VBOM gate 96 -> 572; delete the VBOT gate 96 -> 191; delete
%       the 50-km altitude gate 96 -> 96 -- IT IS INERT ON THE MAP.  That is a
%       finding, not a spare check: the gate rejects samples (56, coarse) but
%       never the last surviving one for any cell.  Do not quote it as
%       evidence that anything here is validated.
%
%   OUTPUTS
%     R.lonT R.latT R.hit R.VBOT   the denied map and its grid [deg, km/s]
%     R.ndenied R.area_km2         the map's cell count and Riemann area
%     R.cases                      .min .loft .depr sweeps
%     R.deletes                    .no_vbom .no_vbot .no_altgate sweeps
%     R.ref                        reference-cell diagnostics
%     R.trio                       the 32.3/32.4/32.5 agreement evidence
%     R.bench R.selfcheck R.pass R.verdict R.summary R.p R.units
%
%   See also MGS.ZARCHAN.IMPULSIVEICBM (32.1), MGS.ZARCHAN.KINEMATICENGAGEMENTMC
%   (32.2), MGS.ZARCHAN.STRATEGICENGAGEMENT (28.4), MGS.ZARCHAN.BENCHSUITE.

arguments
    opts.VBOLIM    (1,1) double {mustBeFinite, mustBePositive} = 5.0     % km/s
    opts.VBOTLIM   (1,1) double {mustBeFinite, mustBePositive} = 7.5     % km/s
    opts.RDESKM    (1,1) double {mustBeFinite, mustBePositive} = 10000.  % km
    opts.TLOFT     (1,1) double {mustBeFinite}                 = 0.      % s
    opts.XLONGMDEG (1,1) double {mustBeFinite} = 60.
    opts.XLATMDEG  (1,1) double {mustBeFinite} = 0.
    opts.ALTMKMIC  (1,1) double {mustBeFinite, mustBeNonnegative} = 0.
    opts.ALTGATE   (1,1) double {mustBeFinite} = 50.                     % km
    opts.KMPDEG    (1,1) double {mustBeFinite, mustBePositive} = 111.    % :12325
    opts.grid  (1,:) char {mustBeMember(opts.grid,{'coarse','book'})} = 'coarse'
    % 'book' = XT, exactly as Listings 32.3/32.4 print it.  'xtf' = XTF, the
    % spelling Listing 32.5 uses.  SELF-CHECK 5 measures the difference.
    opts.alt_x (1,:) char {mustBeMember(opts.alt_x,{'book','xtf'})} = 'book'
    % true records only WHETHER a cell is denied (identical MAP, far cheaper);
    % false reproduces the listing's own "count" over every (TLAUNCH,TF).
    opts.first_hit_only (1,1) logical = true
    opts.loft_cases  (1,1) logical = true    % the TLOFT = 500 / -200 sweeps
    opts.experiments (1,1) logical = true    % the three delete experiments
    opts.benchmark   (1,1) logical = true
    opts.render      (1,1) logical = true
    opts.plot        (1,1) logical = false
    % NOT nargin: in a function whose arguments block declares only name-value
    % options MATLAB reports nargin = 0 however many were passed, so an
    % "if nargin == 0" here would be a mode that never fires.
    opts.assert      (1,1) logical = true
end

P = baseParams(opts);

% ---- the nominal sweep, Listing 32.4 as printed --------------------------
S = sweepLAD(P);

% ---- TLOFT = 500 and -200, the cases of :12430 and :12473 ----------------
Sl = [];  Sd = [];
if opts.loft_cases
    Pl = P;  Pl.TLOFT = P.TLOFT + 500.;   Sl = sweepLAD(Pl);
    Pd = P;  Pd.TLOFT = P.TLOFT - 200.;   Sd = sweepLAD(Pd);
end

% ---- the three delete experiments ----------------------------------------
Dm = [];  Dt = [];  Da = [];
if opts.experiments
    Pm = P;  Pm.VBOLIM  =  inf;  Dm = sweepLAD(Pm);   % delete the VBOM gate
    Pt = P;  Pt.VBOTLIM =  inf;  Dt = sweepLAD(Pt);   % delete the VBOT gate
    Pa = P;  Pa.ALTGATE = -inf;  Da = sweepLAD(Pa);   % delete the 50-km gate
end

% ---- the reference cell and the 32.3/32.4/32.5 agreement -----------------
ref  = evalRef(P, P.TLOFT, '32.4');
trio = trioCheck(P);

R = struct();
R.lonT = S.lonT;  R.latT = S.latT;  R.hit = S.hit;  R.VBOT = S.VBOT;
R.ndenied  = S.ndenied;
R.area_km2 = S.area_km2;
R.cases   = struct('min', S, 'loft', Sl, 'depr', Sd);
R.deletes = struct('no_vbom', Dm, 'no_vbot', Dt, 'no_altgate', Da);
R.ref  = ref;
R.trio = trio;
R.units = ['ft, ft/s internally (A = 2.0926E7 ft, GM = 1.4077E16 ft^3/s^2); ' ...
           'km and km/s via the listing''s own 3280 ft/km; areas km^2 at ' ...
           '111 km/deg'];
R.p = P;

R.selfcheck = structuralChecks(P, S, ref, trio, Dm, Dt, Da, opts.experiments);

if opts.benchmark
    R.bench = benchmarks(P, S, Sl, Sd, ref, trio, opts);
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
if opts.plot,   drawMap(R); end

% ---- adjudication.  The verdict is FORMED either way; 'assert' chooses only
%      the reaction to it, so that a failing run still hands back R with the
%      evidence attached.  benchSuite runs every port with assert off.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated: this run is off ' ...
        'the book''s nominal case (TLOFT = %g s, RDESKM = %g km, VBOLIM = ' ...
        '%g km/s, interceptor %g deg).  An unvalidated port must not be ' ...
        'handed back silently.'], P.TLOFT, P.RDESKM, P.VBOLIM, P.XLONGMDEG);
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
        error('mgs:zarchan:launchAreaDenied:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:launchAreaDenied:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', ...
            msg);
    end
end
end


% =============================================================================
function P = baseParams(opts)
%BASEPARAMS  Listing 32.4 line 1 and its two sweep headers, zarchan.txt:12394,
%   :12413-12414, plus the grid resolution this run will use.
P = struct();
P.A       = 2.0926E7;     % ft, the listing's Earth radius
P.GM      = 1.4077E16;    % ft^3/s^2
P.FTPKM   = 3280.;        % the listing's own rounding of 3280.84
P.DEGPRAD = 57.3;         % the listing's own rounding of 57.29578
P.RDESKM  = opts.RDESKM;
P.TLOFT   = opts.TLOFT;
P.VBOLIM  = opts.VBOLIM;
P.VBOTLIM = opts.VBOTLIM;
P.ALTGATE = opts.ALTGATE;
P.KMPDEG  = opts.KMPDEG;
P.XLONGMDEG = opts.XLONGMDEG;
P.XLATMDEG  = opts.XLATMDEG;
P.ALTM      = opts.ALTMKMIC*P.FTPKM;
% The aimpoint line, verbatim: XLONGFDEG = 57.3*RDESKM*3280./A.  The 57.3
% cancels on the way back out, so DISTFKM at the reference cell is RDESKM to
% the last bit -- BENCHMARK 2 scores that.
P.XLONGFDEG = P.DEGPRAD*P.RDESKM*P.FTPKM/P.A;
P.XLATFDEG  = 0.;
% The listing's grids, zarchan.txt:12413-12414.
P.lon0 = -100.;  P.lon1 = 200.;
P.lat0 =  -60.;  P.lat1 =  60.;
P.TLA0 =  300.;  P.TLA1 = 1600.;
switch opts.grid
    case 'book',  P.dlon =  5.;  P.dlat = 2.5;  P.dtla = 100.;  P.dtf =  50.;
    otherwise,    P.dlon = 10.;  P.dlat = 5.0;  P.dtla = 300.;  P.dtf = 100.;
end
P.grid = opts.grid;
% mod(dtla,dtf) == 0 is what lets the TF lattice be shared across TLAUNCH,
% which is what makes the KEPLER1 memo below EXACT rather than nearly exact.
if abs(mod(P.dtla, P.dtf)) > 1e-9
    error('mgs:zarchan:launchAreaDenied:lattice', ...
        'dtla = %g s must be a multiple of dtf = %g s.', P.dtla, P.dtf);
end
P.alt_x          = opts.alt_x;
P.alt_use_xtf    = strcmp(opts.alt_x, 'xtf');  % 32.5's spelling
P.gate_breaks    = false;                      % 32.4 SKIPS; 32.3/32.5 break
P.tf_backoff     = 0.;                         % 32.5 uses 50 s
P.first_hit_only = opts.first_hit_only;
end


% =============================================================================
function S = sweepLAD(P)
%SWEEPLAD  Listing 32.4's four nested loops, zarchan.txt:12413-12427.
%   Every constant arrives inside P.  MATLAB local functions cannot see the
%   main function's workspace, and a previous wave in this package lost a run
%   to exactly that.
A = P.A;
lonT = P.lon0:P.dlon:P.lon1;
latT = P.lat0:P.dlat:P.lat1;
nlo  = numel(lonT);  nla = numel(latT);
TLAs = P.TLA0:P.dtla:P.TLA1;

XF = A*cos(P.XLATFDEG/P.DEGPRAD)*cos(P.XLONGFDEG/P.DEGPRAD);
YF = A*cos(P.XLATFDEG/P.DEGPRAD)*sin(P.XLONGFDEG/P.DEGPRAD);
ZF = A*sin(P.XLATFDEG/P.DEGPRAD);
XM = (A+P.ALTM)*cos(P.XLATMDEG/P.DEGPRAD)*cos(P.XLONGMDEG/P.DEGPRAD);
YM = (A+P.ALTM)*cos(P.XLATMDEG/P.DEGPRAD)*sin(P.XLONGMDEG/P.DEGPRAD);
ZM = (A+P.ALTM)*sin(P.XLATMDEG/P.DEGPRAD);

S = struct();
S.lonT = lonT;  S.latT = latT;
S.hit     = false(nlo,nla);
S.VBOT    = nan(nlo,nla);
S.DISTFKM = nan(nlo,nla);
S.ncombo_hit = zeros(nlo,nla);
S.nlam = 0;  S.nkep = 0;  S.naltrej = 0;  S.nbail = 0;
S.ncombo = 0;  S.nvbot_cut = 0;

for i = 1:nlo
    for j = 1:nla
        XT = A*cos(latT(j)/P.DEGPRAD)*cos(lonT(i)/P.DEGPRAD);
        YT = A*cos(latT(j)/P.DEGPRAD)*sin(lonT(i)/P.DEGPRAD);
        ZT = A*sin(latT(j)/P.DEGPRAD);

        DISTFKM = distance3dkm(XF,YF,ZF, XT,YT,ZT, A, P.FTPKM);
        TFTOT   = 252. + .223*DISTFKM - (5.44E-6)*DISTFKM*DISTFKM + P.TLOFT;
        S.DISTFKM(i,j) = DISTFKM;

        % TGOLAM = TFTOT - T, and T is never advanced.  See SELF-CHECK 4.
        [XTD,YTD,ZTD,bail] = lambert3d(XT,YT,ZT, TFTOT, XF,YF,ZF, 0, P.GM);
        S.nlam = S.nlam + 1;  S.nbail = S.nbail + bail;
        VBOT = sqrt(XTD^2 + YTD^2 + ZTD^2)/P.FTPKM;
        S.VBOT(i,j) = VBOT;

        % The book's conjunction, hoisted: VBOT depends on the cell alone.
        if ~(VBOT < P.VBOTLIM)
            S.nvbot_cut = S.nvbot_cut + 1;
            continue
        end

        tfAll = (P.TLA0 + 60.):P.dtf:(TFTOT - P.tf_backoff);
        nTF   = numel(tfAll);
        if nTF == 0, continue; end
        XTFv = nan(1,nTF);  YTFv = XTFv;  ZTFv = XTFv;  ALTv = XTFv;
        kdone = false(1,nTF);

        hit = false;  nhit = 0;
        for a = 1:numel(TLAs)
            TLAUNCH = TLAs(a);
            k0 = find(tfAll >= TLAUNCH + 60. - 1e-9, 1);
            if isempty(k0), continue; end
            for k = k0:nTF
                S.ncombo = S.ncombo + 1;
                if ~kdone(k)
                    X1 = kepler1([XT YT ZT XTD YTD ZTD]/P.FTPKM, 0., tfAll(k));
                    XTFv(k) = X1(1)*P.FTPKM;
                    YTFv(k) = X1(2)*P.FTPKM;
                    ZTFv(k) = X1(3)*P.FTPKM;
                    % *** THE BOOK DEFECT.  32.3/32.4 print XT here -- the
                    % target's LAUNCH x-coordinate -- where 32.5 prints XTF.
                    if P.alt_use_xtf, XA = XTFv(k); else, XA = XT; end
                    ALTv(k) = (sqrt(XA^2 + YTFv(k)^2 + ZTFv(k)^2) - A)/P.FTPKM;
                    kdone(k) = true;  S.nkep = S.nkep + 1;
                end
                if ALTv(k) < P.ALTGATE
                    S.naltrej = S.naltrej + 1;
                    if P.gate_breaks, break; else, continue; end
                end
                [VRXM,VRYM,VRZM,bail] = lambert3d(XM,YM,ZM, ...
                    tfAll(k)-TLAUNCH, XTFv(k),YTFv(k),ZTFv(k), 0, P.GM);
                S.nlam = S.nlam + 1;  S.nbail = S.nbail + bail;
                VBOM = sqrt(VRXM^2 + VRYM^2 + VRZM^2)/P.FTPKM;
                if VBOM < P.VBOLIM && VBOT < P.VBOTLIM
                    hit = true;  nhit = nhit + 1;
                    if P.first_hit_only, break; end
                end
            end
            if hit && P.first_hit_only, break; end
        end
        S.hit(i,j) = hit;  S.ncombo_hit(i,j) = nhit;
    end
end

S.ndenied  = sum(S.hit(:));
S.area_km2 = S.ndenied * (P.dlon*P.KMPDEG) * (P.dlat*P.KMPDEG);
end


% =============================================================================
function E = evalRef(P, TLOFT, driver)
%EVALREF  ONE cell -- target launch 0/0 deg -- under one of the three drivers.
%   This is the configuration at which the 32.3, 32.4 and 32.5 slices
%   intersect.  ALWAYS run on the book's inner grid (100 s / 50 s) so that the
%   rows built on it do not move with the caller's 'grid' choice.
A = P.A;
switch driver
    case '32.3', backoff = 0.;   gbreak = true;   usextf = false;
    case '32.5', backoff = 50.;  gbreak = true;   usextf = true;
    otherwise,   backoff = 0.;   gbreak = false;  usextf = P.alt_use_xtf;
end
XF = A*cos(P.XLATFDEG/P.DEGPRAD)*cos(P.XLONGFDEG/P.DEGPRAD);
YF = A*cos(P.XLATFDEG/P.DEGPRAD)*sin(P.XLONGFDEG/P.DEGPRAD);
ZF = A*sin(P.XLATFDEG/P.DEGPRAD);
XM = (A+P.ALTM)*cos(P.XLATMDEG/P.DEGPRAD)*cos(P.XLONGMDEG/P.DEGPRAD);
YM = (A+P.ALTM)*cos(P.XLATMDEG/P.DEGPRAD)*sin(P.XLONGMDEG/P.DEGPRAD);
ZM = (A+P.ALTM)*sin(P.XLATMDEG/P.DEGPRAD);
XT = A;  YT = 0.;  ZT = 0.;              % target launch at 0 deg / 0 deg

E = struct();
E.driver  = driver;
E.TLOFT   = TLOFT;
E.DISTFKM = distance3dkm(XF,YF,ZF, XT,YT,ZT, A, P.FTPKM);
E.TFTOT   = 252. + .223*E.DISTFKM - (5.44E-6)*E.DISTFKM*E.DISTFKM + TLOFT;
[XTD,YTD,ZTD] = lambert3d(XT,YT,ZT, E.TFTOT, XF,YF,ZF, 0, P.GM);
E.VBOT = sqrt(XTD^2 + YTD^2 + ZTD^2)/P.FTPKM;

tfAll = (P.TLA0 + 60.):50.:(E.TFTOT - backoff);
nTF   = numel(tfAll);
E.tf  = tfAll;
E.alt_book = nan(1,nTF);  E.alt_xtf = nan(1,nTF);
XTFv = nan(1,nTF);  YTFv = XTFv;  ZTFv = XTFv;
for k = 1:nTF
    X1 = kepler1([XT YT ZT XTD YTD ZTD]/P.FTPKM, 0., tfAll(k));
    XTFv(k) = X1(1)*P.FTPKM;  YTFv(k) = X1(2)*P.FTPKM;  ZTFv(k) = X1(3)*P.FTPKM;
    E.alt_book(k) = (sqrt(XT^2      + YTFv(k)^2 + ZTFv(k)^2) - A)/P.FTPKM;
    E.alt_xtf(k)  = (sqrt(XTFv(k)^2 + YTFv(k)^2 + ZTFv(k)^2) - A)/P.FTPKM;
end
if usextf, ALT = E.alt_xtf; else, ALT = E.alt_book; end

acc = zeros(0,2);  E.naltrej = 0;
for TLAUNCH = P.TLA0:100.:P.TLA1
    k0 = find(tfAll >= TLAUNCH + 60. - 1e-9, 1);
    if isempty(k0), continue; end
    for k = k0:nTF
        if ALT(k) < P.ALTGATE
            E.naltrej = E.naltrej + 1;
            if gbreak, break; else, continue; end
        end
        [mx,my,mz] = lambert3d(XM,YM,ZM, tfAll(k)-TLAUNCH, ...
                               XTFv(k),YTFv(k),ZTFv(k), 0, P.GM);
        VBOM = sqrt(mx^2 + my^2 + mz^2)/P.FTPKM;
        if VBOM < P.VBOLIM && E.VBOT < P.VBOTLIM
            acc(end+1,:) = [TLAUNCH tfAll(k)];  %#ok<AGROW>
        end
    end
end
E.accept  = acc;
E.naccept = size(acc,1);
end


% =============================================================================
function T = trioCheck(P)
%TRIOCHECK  BENCHMARK 8.  Run all three drivers at the shared configuration.
%   VBOT is driver-independent by construction, so it must agree to ZERO ulp;
%   the accepted sets may differ only as the three code deltas allow (32.5
%   loses TF > TFTOT-50; 32.3's break can only REMOVE pairs that 32.4 keeps).
E3 = evalRef(P, P.TLOFT, '32.3');
E4 = evalRef(P, P.TLOFT, '32.4');
E5 = evalRef(P, P.TLOFT, '32.5');
T = struct();
T.n3 = E3.naccept;  T.n4 = E4.naccept;  T.n5 = E5.naccept;
T.vbot3 = E3.VBOT;  T.vbot4 = E4.VBOT;  T.vbot5 = E5.VBOT;
T.dvbot = max(abs([E3.VBOT E5.VBOT] - E4.VBOT));
T.vbot_identical = (E3.VBOT == E4.VBOT) && (E5.VBOT == E4.VBOT);
T.sub3 = isSubsetRows(E3.accept, E4.accept);
T.sub5 = isSubsetRows(E5.accept, E4.accept);
if isempty(E4.accept)
    pred5 = zeros(0,2);
else
    pred5 = E4.accept(E4.accept(:,2) <= E4.TFTOT - 50. + 1e-9, :);
end
T.five_explained = isSubsetRows(E5.accept, pred5) && isSubsetRows(pred5, E5.accept);
T.identical = T.vbot_identical && (T.n3 == T.n4) && (T.n5 == T.n4);
T.ok = T.vbot_identical && T.sub3 && T.sub5 && T.five_explained;
end

function tf = isSubsetRows(Asub, Aset)
tf = true;
for r = 1:size(Asub,1)
    if isempty(Aset) || ~any(all(abs(Aset - Asub(r,:)) < 1e-9, 2))
        tf = false;  return
    end
end
end


% =============================================================================
function DISTKM = distance3dkm(XT,YT,ZT, XF,YF,ZF, A, FTPKM)
%DISTANCE3DKM  zarchan.txt:10423.  Great-circle arc at the Earth radius A,
%   returned in km.  The else branch is the book's and is unreachable for
%   |CBETA| <= 1; kept so that nobody "restores" it later.  The clamp inside
%   acos is OURS: CBETA can come back as 1 + 2e-16 for a near-zero arc and
%   acos would then return a complex distance that plots perfectly happily.
R  = sqrt(XT^2 + YT^2 + ZT^2);
RF = sqrt(XF^2 + YF^2 + ZF^2);
CBETA = (XT*XF + YT*YF + ZT*ZF)/(R*RF);
if CBETA <= 1.
    DISTKM = A*acos(max(-1, min(1, CBETA)))/FTPKM;
else
    DISTKM = (XF - XT)/FTPKM;
end
end


% =============================================================================
function [VRX,VRY,VRZ,BAIL] = lambert3d(XT,YT,ZT, TF, XF,YF,ZF, SWITCH1, GM)
%LAMBERT3D  zarchan.txt:10428, the three-dimensional Lambert routine printed
%   under Listing 28.2.  Secant search on flight-path angle GAMMA inside a
%   bracket, then the in-plane velocity direction as C1*r0 + C2*rf.
%
%   The same routine MGS.ZARCHAN.STRATEGICENGAGEMENT and .IMPULSIVEICBM already
%   carry; the bracket-update comparisons were resolved there and CONFIRMED
%   against the published iterate sequence of Table 13.2 by
%   MGS.ZARCHAN.LAMBERTEFFICIENT, and are not re-derived here.  The book's dead
%   locals (BL, BT, DEG_PER_RAD, EARTH_RADIUS, FT_PER_KM, RAD_PER_SEC) are
%   dropped, not "fixed".  Two additions: GM is an argument, and BAIL flags the
%   non-real cases described in the file header.
PI = 3.1415926535898;  HALFPI = PI/2.;
BAIL = 0;  VRX = NaN;  VRY = NaN;  VRZ = NaN;
RF0X = XF-XT;  RF0Y = YF-YT;  RF0Z = ZF-ZT;
R0DOTRF  = XT*XF + YT*YF + ZT*ZF;
R0DOTRF0 = XT*RF0X + YT*RF0Y + ZT*RF0Z;
R0MAG  = sqrt(XT^2 + YT^2 + ZT^2);
RFMAG  = sqrt(XF^2 + YF^2 + ZF^2);
RF0MAG = sqrt(RF0X^2 + RF0Y^2 + RF0Z^2);
if R0MAG == 0 || RFMAG == 0 || RF0MAG == 0 || ~(TF > 0), BAIL = 1;  return; end
RATIO   = R0MAG/RFMAG;
GMDIVR0 = GM/R0MAG;
COS_T   = max(-1, min(1, R0DOTRF/(R0MAG*RFMAG)));
CB      = max(-1, min(1, R0DOTRF0/(R0MAG*RF0MAG)));
VNUMER  = GMDIVR0*(1. - COS_T);
T_MIN   = 0.;
if SWITCH1 == 0
    G_MIN = HALFPI - acos(CB);   G_MAX = HALFPI;              THETA = acos(COS_T);
else
    G_MIN = -HALFPI;             G_MAX = -HALFPI + acos(CB);  THETA = 2.*PI - acos(COS_T);
end
SIN_T = sin(THETA);  COT_HALFT = 1./tan(THETA/2.);
GAMMA = (G_MAX + G_MIN)/2.;  GOLD = G_MIN;  TOLD = 0.;  T = 0.;  ITERS = 1;
RV0MAG = NaN;  NEXT = GAMMA;   %#ok<NASGU> defensive; both set on every pass

while abs(TF - T) > (.00000001*TF)
    SIN_G = sin(GAMMA);  COS_G = cos(GAMMA);  TAN_G = SIN_G/COS_G;
    COS_TPLUSG = cos(THETA + GAMMA - 2.*PI);
    TERM1 = (RATIO*COS_G - COS_TPLUSG)*COS_G;
    if ~isfinite(TERM1) || TERM1 <= 0, BAIL = 1;  return; end
    RV0MAG = sqrt(VNUMER/TERM1);
    LAMBDA = RV0MAG*RV0MAG/GMDIVR0;
    if LAMBDA < 1.9999999
        TERM0 = sqrt(2./LAMBDA - 1.);
        TA = (TAN_G*(1.-COS_T) + (1.-LAMBDA)*SIN_T)/((2.-LAMBDA)*RATIO);
        TB = (COS_G + COS_G)/(LAMBDA*TERM0*TERM0*TERM0);
        TC = atan2(TERM0, (COS_G*COT_HALFT - SIN_G));
        T  = (R0MAG/(RV0MAG*COS_G))*(TA + TB*TC);
    elseif LAMBDA > 2.0000001
        TERM0 = sqrt(1. - 2./LAMBDA);
        TA = (TAN_G*(1.-COS_T) + (1.-LAMBDA)*SIN_T)/((2.-LAMBDA)*RATIO);
        TB = COS_G/(LAMBDA*TERM0*TERM0*TERM0);
        TC = SIN_G - COS_G*COT_HALFT;
        ARG = (TC - TERM0)/(TC + TERM0);
        if ~isfinite(ARG) || ARG <= 0, BAIL = 1;  return; end
        T  = (R0MAG/(RV0MAG*COS_G))*(TA - TB*log(ARG));
    else
        TERM0 = COS_G*COT_HALFT;  TA = TERM0 - SIN_G;
        TERM0 = (3*TERM0*TA + 1.)/(TA*TA*TA);
        T = TERM0*(2.*R0MAG)/(3.*RV0MAG);
    end
    if T > TF  && GAMMA < G_MAX, G_MAX = GAMMA; end
    if T < 0.  && GAMMA < G_MAX, G_MAX = GAMMA; end
    if T < TF  && GAMMA > G_MIN, G_MIN = GAMMA;  T_MIN = T; end
    if T < 0.
        NEXT = (G_MIN + G_MAX)/2.;  GOLD = G_MIN;  TOLD = T_MIN;
    else
        % Guard on the secant denominator.  The listing has none; a repeated
        % iterate would divide by zero and return Inf, which then compares
        % false against everything and looks like a legitimate exclusion.
        if T == TOLD, break; end
        NEXT = GAMMA + (TF - T)*(GAMMA - GOLD)/(T - TOLD);
        if NEXT >= G_MAX
            NEXT = (GAMMA + G_MAX)/2.;
        elseif NEXT <= G_MIN
            NEXT = (GAMMA + G_MIN)/2.;
        end
        GOLD = GAMMA;  TOLD = T;
    end
    GAMMA = NEXT;
    ITERS = ITERS + 1;
    if ITERS > 100, break; end
end

if SWITCH1 == 0, ANGLE = HALFPI - GAMMA;  else, ANGLE = GAMMA - HALFPI;  end
SINA = sin(ANGLE);  COSA = cos(ANGLE);
MAG1   = R0MAG;
DOTMAG = R0DOTRF;
CROSSX = YT*ZF - ZT*YF;
CROSSY = ZT*XF - XT*ZF;
CROSSZ = XT*YF - YT*XF;
CROSSMAG = sqrt(CROSSX^2 + CROSSY^2 + CROSSZ^2);
if ~isfinite(RV0MAG) || CROSSMAG == 0, BAIL = 1;  return; end
C2 = MAG1*SINA/CROSSMAG;
C1 = COSA/MAG1 - DOTMAG*C2/(MAG1*MAG1);
VRX = RV0MAG*(C2*XF + C1*XT);
VRY = RV0MAG*(C2*YF + C1*YT);
VRZ = RV0MAG*(C2*ZF + C1*ZT);
if ~all(isfinite([VRX VRY VRZ])) || ~isreal([VRX VRY VRZ])
    VRX = NaN;  VRY = NaN;  VRZ = NaN;  BAIL = 1;
end
end


% =============================================================================
function X1 = kepler1(X0, T0, T1)
%KEPLER1  zarchan.txt:10486, printed under Listing 28.3.  Laguerre-Conway
%   iteration (n = 5), KN = 10 fixed passes on the universal variable.
%   INPUTS AND OUTPUTS ARE km AND km/s, which is why Listing 32.4 divides by
%   3280 going in and multiplies by 3280 coming out.
%   *** Its GMX = 398923 km^3/s^2 and REX = 6380 km are NOT the Lambert
%   routine's GM and A converted (those give 398928 and 6379.88) -- a 1.3e-5
%   inconsistency the book carries and this port does not repair.
%   The degenerate-DT branch RETURNS; as printed it falls through and then
%   divides by DT = 0.  MGS.ZARCHAN.KEPLERVSDIRECT is the authority here.
GMX = 398923.;  REX = 6380.;  TLIMIT = 1.E-10;  KN = 10;  MUQR = 1.;
DT = T1 - T0;
if abs(DT) < TLIMIT, X1 = X0(1:6);  return; end
TIME_FACTOR = sqrt(REX^3/GMX);
VEX = REX/TIME_FACTOR;
DT  = DT/TIME_FACTOR;
R0 = X0(1:3)/REX;
V0 = X0(4:6)/VEX;
R0MAG = sqrt(R0(1)^2 + R0(2)^2 + R0(3)^2);
V0MAG = sqrt(V0(1)^2 + V0(2)^2 + V0(3)^2);
SIGMA0 = (R0(1)*V0(1) + R0(2)*V0(2) + R0(3)*V0(3))/MUQR;
ALP0 = 2./R0MAG - V0MAG*V0MAG;
X = ALP0*DT;
if ALP0 <= 0., X = .1*DT/R0MAG; end
U1 = 0.;  U2 = 0.;  U3 = 0.;  DFX = 1.;
for K = 1:KN
    if ALP0 < 0.
        Y = ALP0*X*X;  YQR = sqrt(-Y);
        CY = (1. - cosh(YQR))/Y;  SY = (sinh(YQR) - YQR)/(YQR^3);
    elseif ALP0 == 0.
        Y = 0.;  CY = .5;  SY = 1./6.;
    else
        Y = ALP0*X*X;
        if Y == 0.
            CY = .5;  SY = 1./6.;      % the X = 0 limit; as printed, 0/0
        else
            YQR = sqrt(Y);
            CY = (1. - cos(YQR))/Y;  SY = (YQR - sin(YQR))/(YQR^3);
        end
    end
    U1 = X*(1. - Y*SY);
    U2 = X*X*CY;
    U3 = X*X*X*SY;
    FX   = R0MAG*U1 + SIGMA0*U2 + U3 - DT*MUQR;
    DFX  = SIGMA0*U1 + (1. - ALP0*R0MAG)*U2 + R0MAG;
    DFX2 = SIGMA0*(1. - Y*CY) + (1. - ALP0*R0MAG)*U1;
    SDFX = DFX/abs(DFX);
    DX2  = 16.*DFX*DFX - 20.*FX*DFX2;
    if DX2 > 0.,  DX = 5.*FX/(DFX + SDFX*sqrt(DX2));  else,  DX = .5*X;  end
    X = X - DX;
end
RMAG = DFX;
F  = 1. - U2/R0MAG;
G  = DT - U3/MUQR;
DF = -MUQR*U1/(RMAG*R0MAG);
DG = 1. - U2/RMAG;
X1 = zeros(1,6);
for I = 1:3
    X1(I)   = (F*R0(I)  + G*V0(I))*REX;
    X1(I+3) = (DF*R0(I) + DG*V0(I))*VEX;
end
end


% =============================================================================
function K = structuralChecks(P, S, ref, trio, Dm, Dt, Da, expOn)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.
%   *** .names and .oks must BOTH exist and be the SAME LENGTH or benchSuite
%   puts ZERO rows on the board -- a port with twenty invariants and no .names
%   is a port that ran nothing anybody can see.
names = {};  oks = [];  detail = {};

% 1  DELETE THE VBOM GATE.  The only check that proves the interceptor's own
%    Lambert leg is wired to the accept test at all.
names{end+1} = 'DELETE the VBOM<VBOLIM gate -> denied area grows';
if expOn
    oks(end+1)    = Dm.ndenied > S.ndenied;
    detail{end+1} = sprintf('denied %d -> %d cells (x%.2f)', S.ndenied, ...
        Dm.ndenied, Dm.ndenied/max(S.ndenied,1));
else
    oks(end+1)    = false;
    detail{end+1} = 'NOT RUN (''experiments'',false) -- an unrun check is not a pass';
end

% 2  DELETE THE VBOT GATE (:12392, "ruled out if the required impulsive target
%    velocity exceeds 7.5 km/s").
names{end+1} = 'DELETE the VBOT<7.5 gate -> denied area grows';
if expOn
    oks(end+1)    = Dt.ndenied > S.ndenied;
    detail{end+1} = sprintf(['denied %d -> %d cells; %d of %d cells were cut ' ...
        'by VBOT alone'], S.ndenied, Dt.ndenied, S.nvbot_cut, numel(S.VBOT));
else
    oks(end+1)    = false;
    detail{end+1} = 'NOT RUN (''experiments'',false)';
end

% 3  THE 50-km ALTITUDE GATE IS EXERCISED.  It is MEASURED to be inert on the
%    map -- deleting it moves nothing -- so this row claims only that it fires
%    at all.  If it never fires, the resolution of "~(ALTFKM , 50.)" is
%    untestable here and must not be reported as validated.
names{end+1} = 'the 50-km altitude gate fires at least once (measured INERT on the map)';
if expOn
    oks(end+1)    = S.naltrej > 0;
    detail{end+1} = sprintf(['%d (TLAUNCH,TF) samples rejected; deleting the ' ...
        'gate moves denied %d -> %d cells'], S.naltrej, S.ndenied, Da.ndenied);
else
    oks(end+1)    = false;
    detail{end+1} = 'NOT RUN (''experiments'',false)';
end

% 4  TGOLAM = TFTOT - T with T never advanced.  A reader "tidying" that into
%    TFTOT - TF gets a different trajectory entirely, and it still runs.
[vx,vy,vz] = lambert3d(P.A, 0., 0., ref.TFTOT - 1000., ...
    P.A*cos(P.XLONGFDEG/P.DEGPRAD), P.A*sin(P.XLONGFDEG/P.DEGPRAD), 0., 0, P.GM);
vbot_tidied = sqrt(vx^2 + vy^2 + vz^2)/P.FTPKM;
names{end+1}  = 'TGOLAM = TFTOT - T with T == 0 (not TFTOT - TF)';
oks(end+1)    = isfinite(vbot_tidied) && abs(vbot_tidied/ref.VBOT - 1) > 0.01;
detail{end+1} = sprintf(['listing VBOT %.5f km/s vs the TFTOT-TF reading at ' ...
    'TF = 1000 s: %.5f km/s'], ref.VBOT, vbot_tidied);

% 5  THE BOOK DEFECT IS REAL: ALTFKM built from XT is not the intercept
%    altitude.  Measured, not argued.
dalt = max(abs(ref.alt_book - ref.alt_xtf));
names{end+1}  = 'ALTFKM(XT) differs from ALTFKM(XTF) -- the 32.3/32.4 defect is real';
oks(end+1)    = dalt > 1.0;
detail{end+1} = sprintf(['max |XT form - XTF form| = %.1f km over %d TF ' ...
    'samples; peaks %.1f km vs %.1f km'], dalt, numel(ref.tf), ...
    max(ref.alt_book), max(ref.alt_xtf));

% 6  the trio wiring: VBOT cannot depend on which driver asked for it.
names{end+1}  = '32.3/32.4/32.5 return the SAME VBOT to 0 ulp at the shared cell';
oks(end+1)    = trio.vbot_identical;
detail{end+1} = sprintf('max |dVBOT| = %.3e km/s; accepted pairs %d / %d / %d', ...
    trio.dvbot, trio.n3, trio.n4, trio.n5);

K = struct();
K.names = names;  K.oks = logical(oks);  K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    j = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{j}, detail{j});
end
end


% =============================================================================
function B = benchmarks(P, S, Sl, Sd, ref, trio, opts)
%BENCHMARKS  Nine rows.  Every book value was grepped from zarchan_decoded.txt
%   and its line is in .source.  Nothing was read off a figure: the extraction
%   turned every figure in Chapter 32 into an image link, so a "figure reading"
%   here would be an invention.
B = emptyBench();

% Off-book runs must SKIP, not silently rescale.  onTgt is what rows 1/2/5
% need (the target problem); onBook adds the interceptor and the two limits.
onTgt  = (P.TLOFT == 0.) && (P.RDESKM == 10000.);
onBook = onTgt && (P.VBOLIM == 5.) && (P.VBOTLIM == 7.5) && ...
         (P.XLONGMDEG == 60.) && (P.XLATMDEG == 0.) && (P.ALTGATE == 50.);
offTgt  = sprintf('off the book''s target case: TLOFT = %g s, RDESKM = %g km', ...
    P.TLOFT, P.RDESKM);
offBook = sprintf(['off the book''s nominal case: TLOFT %g s, RDESKM %g km, ' ...
    'VBOLIM %g km/s, VBOTLIM %g km/s, interceptor %g/%g deg, gate %g km'], ...
    P.TLOFT, P.RDESKM, P.VBOLIM, P.VBOTLIM, P.XLONGMDEG, P.XLATMDEG, P.ALTGATE);

% 1  the target's required burnout velocity at the reference cell.  THE ANCHOR:
%    same Lambert, same TFTOT formula, same 3280, so if this row is green the
%    kernel every other row rests on is the book's.  Band = 0.005 km/s, half
%    the last printed digit -- the same band MGS.ZARCHAN.KINEMATICENGAGEMENTMC
%    and .IMPULSIVEICBM use for the same number.
if onTgt
    B = addVal(B, 'target burnout velocity VBOT, minimum energy, reference cell', ...
        7.19, ref.VBOT, '0.005 km/s = half the last printed digit', 0.005/7.19, ...
        'Listing 32.1 discussion, zarchan.txt:12172', 'km/s', ...
        sprintf('target launch 0/0 deg, aim %.4f deg, TFTOT %.3f s', ...
            P.XLONGFDEG, ref.TFTOT));
else
    B = addSkip(B, 'target burnout velocity VBOT, minimum energy, reference cell', ...
        '7.19 km/s', offTgt, 'zarchan.txt:12172', 'km/s');
end

% 2  the minimum-energy time of flight, from the appendix formula printed at
%    :12157.  The extraction ATE both minus signs there ("252+ 0.223DRkm
%    5.44106DR2 km"); the listing has them.  This row locks the three
%    coefficients AND that the DR^2 term is SUBTRACTED.
if onTgt
    B = addVal(B, 'minimum-energy time of flight at DR = 10,000 km', ...
        1938.0, ref.TFTOT - P.TLOFT, 'exact arithmetic', 1e-9, ...
        'appendix formula zarchan.txt:12157; listing line zarchan.txt:12415', 's', ...
        sprintf(['DISTFKM at the reference cell = %.9f km -- the 57.3 in ' ...
        'XLONGFDEG cancels on the way back out'], ref.DISTFKM));
else
    B = addSkip(B, 'minimum-energy time of flight at DR = 10,000 km', '1938 s', ...
        offTgt, 'zarchan.txt:12157', 's');
end

% 3  the plot abscissa the prose quotes for the interceptor.
if P.XLONGMDEG == 60. && P.KMPDEG == 111.
    trueArc = P.A*(P.XLONGMDEG/P.DEGPRAD)/P.FTPKM;
    B = addVal(B, 'interceptor downrange label, 60-deg longitude', ...
        6660.0, P.XLONGMDEG*P.KMPDEG, 'exact arithmetic', 1e-12, ...
        'zarchan.txt:12430; Fig. 32.10-32.12 captions, :12445/:12466/:12495', 'km', ...
        sprintf(['ArrayXLONGT = XLONGTDEG*%g; the true equatorial arc for ' ...
        '60 deg is %.1f km, so 111 UNDERstates it by %.2f%% at the equator'], ...
        P.KMPDEG, trueArc, 100*(1 - P.XLONGMDEG*P.KMPDEG/trueArc)));
else
    B = addSkip(B, 'interceptor downrange label, 60-deg longitude', '6660 km', ...
        sprintf('needs XLONGMDEG = 60 and KMPDEG = 111; got %g and %g', ...
        P.XLONGMDEG, P.KMPDEG), 'zarchan.txt:12430', 'km');
end

% 4  SHAPE, from prose about the conversion itself, :12325: "1 deg corresponds
%    to 111 km (which is valid along the equator but overestimates distances as
%    the latitude increases)".  Two claims, one row.
kmdeg_true = P.A/(P.DEGPRAD*P.FTPKM);              % km per degree at the equator
r0  = P.KMPDEG/kmdeg_true;                         % ~1 : "valid along the equator"
r60 = P.KMPDEG/(kmdeg_true*cos(60./P.DEGPRAD));    % ~2 : "overestimates"
B = addShape(B, '111 km/deg is equator-valid and overestimates with latitude', ...
    'ratio ~ 1 at the equator, > 1.9 at 60 deg latitude, and increasing', ...
    sprintf('ratio %.4f at 0 deg, %.4f at 60 deg', r0, r60), ...
    abs(r0 - 1) <= 0.01 && r60 >= 1.9 && r60 > r0, 'zarchan.txt:12325', ...
    sprintf('true equatorial degree = %.3f km from A = %g ft', kmdeg_true, P.A));

% 5  SHAPE, and the minus-sign resolution.  The depressed case is TLOFT = -200,
%    printed "-200" at :12473 and "2200" at :12558 -- the same case, one of the
%    two lines damaged by the minus-to-2 mis-map.  This row DISCRIMINATES:
%    -200 gives 7.2297, which prints as the book's 7.23; +200 gives 7.2214,
%    which prints as 7.22.  Two-sided, hence a shape row.
if onTgt
    refD = evalRef(P, P.TLOFT - 200., '32.4');
    refU = evalRef(P, P.TLOFT + 200., '32.4');
    B = addShape(B, 'the depressed case is TLOFT = -200, not +200', ...
        'TLOFT=-200 prints as the book''s 7.23 km/s and TLOFT=+200 does not', ...
        sprintf('-200 -> %.4f km/s (prints %.2f), +200 -> %.4f km/s (prints %.2f)', ...
            refD.VBOT, refD.VBOT, refU.VBOT, refU.VBOT), ...
        abs(refD.VBOT - 7.23) < 0.005 && abs(refU.VBOT - 7.23) >= 0.005, ...
        'zarchan.txt:12172 (7.23), :12473 ("-200") against :12558 ("2200")', ...
        'the book contradicts itself on the sign; both lines cited, the -200 reading scored');
else
    B = addSkip(B, 'the depressed case is TLOFT = -200, not +200', ...
        '7.23 km/s at TLOFT = -200', offTgt, 'zarchan.txt:12473, :12558', 'km/s');
end

% 6-7  the two area orderings.  The thresholds QUANTIFY the book's adjectives
%      and are THIS PORT'S, not the book's; the measured ratios are in .detail
%      so that a reader can judge them.  They are orderings precisely so that
%      the grid resolution, which the caller chooses, cannot decide them.
if onBook && opts.loft_cases && S.area_km2 > 0
    rl = Sl.area_km2/S.area_km2;
    rd = Sd.area_km2/S.area_km2;
    B = addShape(B, 'lofted target (TLOFT=500): denied area shrinks considerably', ...
        'A_loft <= 0.5 * A_min', ...
        sprintf('A_loft/A_min = %.4f (%d vs %d cells)', rl, Sl.ndenied, S.ndenied), ...
        rl <= 0.5, 'zarchan.txt:12430, Fig. 32.11', ...
        sprintf('areas %.4g and %.4g km^2 on the ''%s'' grid', Sl.area_km2, ...
            S.area_km2, P.grid));
    % ":12473 roughly the same size" is said in explicit contrast to ":12430
    % shrinks considerably", so it is scored AS that contrast: the depressed
    % case must sit within a factor of two AND be at least three times closer
    % to parity on a log scale than the lofted case is.
    okd = rd > 0 && rl > 0 && rd >= 0.5 && rd <= 2.0 && ...
          abs(log(rd)) <= abs(log(rl))/3;
    B = addShape(B, 'depressed target (TLOFT=-200): roughly the same size as minimum energy', ...
        '|ln(A_dep/A_min)| <= |ln(A_loft/A_min)|/3 and A_dep/A_min in [0.5,2]', ...
        sprintf('A_dep/A_min = %.4f (%d vs %d cells); |ln| %.4f against budget %.4f', ...
            rd, Sd.ndenied, S.ndenied, abs(log(max(rd,realmin))), ...
            abs(log(max(rl,realmin)))/3), okd, 'zarchan.txt:12473, Fig. 32.12', ...
        'thresholds quantify the book''s adjectives and are this port''s, not the book''s');
else
    B = addSkip(B, 'lofted / depressed denied-area orderings', ...
        'A_loft << A_min ~ A_dep', ...
        ['''loft_cases'' is false, the nominal sweep denied nothing, or ' offBook], ...
        'zarchan.txt:12430, :12473', '-');
end

% 8  THE TRIO ROW -- the one the 32.3/32.4/32.5 relationship is worth more than
%    any single area number.  See the file header.
if onBook
    B = addShape(B, '32.3 / 32.4 / 32.5 are one kernel at the shared configuration', ...
        ['same VBOT to 0 ulp; 32.3 and 32.5 accept subsets of 32.4, and ' ...
         '32.5''s is exactly 32.4''s restricted to TF <= TFTOT-50'], ...
        sprintf(['dVBOT %.3e km/s; accepted pairs %d / %d / %d; subsets %d %d; ' ...
            '32.5 explained %d'], trio.dvbot, trio.n3, trio.n4, trio.n5, ...
            trio.sub3, trio.sub5, trio.five_explained), trio.ok, ...
        ['zarchan.txt:12392 and :12475 ("the code of Listing 32.x can be ' ...
         'modified..."); listings at :12328, :12393, :12476'], ...
        'shared configuration: target launch 0/0 deg, aim 10,000 km downrange, interceptor 60/0 deg');
else
    B = addSkip(B, '32.3 / 32.4 / 32.5 are one kernel at the shared configuration', ...
        'identical VBOT and accepted set', offBook, ...
        'zarchan.txt:12392, :12475', '-');
end

% 9  INFO.  The area itself has no published number anywhere in Chapter 32.
B = addInfo(B, 'launch area denied, nominal case', ...
    'no published number -- Figs. 32.10-32.12 only', ...
    sprintf('%.4g km^2 over %d of %d cells (''%s'' grid)', S.area_km2, ...
        S.ndenied, numel(S.hit), P.grid), 'zarchan.txt:12430', ...
    ['the book quotes only adjectives for the size of these regions, and ' ...
     'every figure in the extraction became an image link, so any "reading" ' ...
     'of Fig. 32.10 would be invented.  Rows 6-7 score the adjectives.']);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names AND ORDER are benchSuite's adaptGeneric schema.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.12g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.12g', ours);
B(i).band = band;  B(i).tol = tol;
B(i).units = units;  B(i).source = source;  B(i).detail = detail;
if isempty(ours) || ~isscalar(ours) || ~isfinite(ours) || book == 0
    B(i).pct = NaN;  B(i).ok = false;  B(i).skipped = true;
    B(i).why = 'the computed value is not a finite scalar (or the book value is zero)';
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

function B = addInfo(B, what, bookStr, oursStr, source, why)
%ADDINFO  Reported, never scored: an INFO row can neither pass nor fail.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'info';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'info';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = '-';  B(i).detail = '';
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
L{end+1} = 'Zarchan Listing 32.4 -- THREE-DIMENSIONAL LAUNCH AREA DENIED';
L{end+1} = sprintf('   %s', R.units);
L{end+1} = sprintf(['   interceptor %.1f/%.1f deg lat/long (VBOLIM %g km/s), ' ...
    'aim %.4f deg (%g km downrange), TLOFT %g s'], R.p.XLATMDEG, ...
    R.p.XLONGMDEG, R.p.VBOLIM, R.p.XLONGFDEG, R.p.RDESKM, R.p.TLOFT);
L{end+1} = sprintf(['   grid ''%s'': long %g:%g:%g, lat %g:%g:%g, TLAUNCH ' ...
    '%g:%g:%g, TF step %g s, ALTFKM from %s'], R.p.grid, R.p.lon0, R.p.dlon, ...
    R.p.lon1, R.p.lat0, R.p.dlat, R.p.lat1, R.p.TLA0, R.p.dtla, R.p.TLA1, ...
    R.p.dtf, R.p.alt_x);
L{end+1} = sprintf(['   DENIED %d of %d cells, area %.4g km^2 (Lambert %d, ' ...
    'Kepler %d, bail-outs %d, VBOT-cut cells %d)'], R.ndenied, numel(R.hit), ...
    R.area_km2, R.cases.min.nlam, R.cases.min.nkep, R.cases.min.nbail, ...
    R.cases.min.nvbot_cut);
L{end+1} = sprintf(['   reference cell: DISTFKM %.4f km, TFTOT %.3f s, VBOT ' ...
    '%.6f km/s, %d accepted (TLAUNCH,TF)'], R.ref.DISTFKM, R.ref.TFTOT, ...
    R.ref.VBOT, R.ref.naccept);
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-64s %s', vstr(K.oks(i)), K.names{i}, K.detail{i}); %#ok<AGROW>
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
                L{end+1} = sprintf('   SKIP %-58s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why);                     %#ok<AGROW>
            case 'info'
                L{end+1} = sprintf('   INFO %-58s %s', b.what, b.ours_str);  %#ok<AGROW>
                L{end+1} = sprintf('        %s  [%s]', b.why, b.source);     %#ok<AGROW>
            case 'shape'
                L{end+1} = sprintf('   %-4s %-58s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s  [%s]', b.book_str, b.source);      %#ok<AGROW>
            otherwise
                L{end+1} = sprintf(['   %-4s %-58s book %s %s / ours %s %s ' ...
                    '(%+.4f%%, band %s)'], vstr(b.ok), b.what, b.book_str, ...
                    b.units, b.ours_str, b.units, b.pct, b.band);            %#ok<AGROW>
                L{end+1} = sprintf('        [%s]  %s', b.source, b.detail);   %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped/info -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: the 50-km intercept-altitude gate is INERT on this map --';
L{end+1} = '   deleting it does not move a single cell.  It is reported, not';
L{end+1} = '   credited.  And ALTFKM is built from XT, which is the book''s own';
L{end+1} = '   defect (Listing 32.5 spells the same line with XTF).';
s = strjoin(L, newline);
end

function s = vstr(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end


% =============================================================================
function drawMap(R)
%DRAWMAP  The listing's own plot: plus signs at XLONGTDEG*111 vs XLATTDEG*111.
figure;
[LO, LA] = ndgrid(R.lonT, R.latT);
plot(LO(R.hit)*R.p.KMPDEG, LA(R.hit)*R.p.KMPDEG, 'r+');
grid on;
xlabel('Downrange (km)');
ylabel('Crossrange (km)');
title(sprintf('Launch area denied, %g km/s interceptor, TLOFT = %g s', ...
    R.p.VBOLIM, R.p.TLOFT));
end
