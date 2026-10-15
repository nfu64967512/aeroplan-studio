function R = defendedArea3d(opts)
%MGS.ZARCHAN.DEFENDEDAREA3D  Zarchan Listing 32.5 -- three-dimensional
%   DEFENDED AREA: which target IMPACT POINTS one fixed interceptor can defend.
%
%   R = mgs.zarchan.defendedArea3d()
%   R = mgs.zarchan.defendedArea3d('book_grid', true)   % the listing's own grid
%   R = mgs.zarchan.defendedArea3d('sweep', false)      % probe rows only
%
%   Third of the Chapter 32 trio.  32.3 fixes the target trajectory and sweeps
%   the INTERCEPTOR site; 32.4 sweeps the target LAUNCH site; 32.5 fixes the
%   target launch site (0/0) and the interceptor (equator, 60 deg lon =
%   6660 km downrange) and sweeps the target IMPACT POINT.  A plus sign is
%   plotted wherever some (TLAUNCH, TF) pair puts the interceptor's required
%   burnout speed below VBOLIM against a target whose own required burnout
%   speed is below 7.5 km/s.
%
%   UNITS ARE THE BOOK'S: feet and ft/s internally, with the book's own
%   A = 2.0926e7 ft, GM = 1.4077e16 ft^3/s^2 and 57.3 deg/rad (NOT 180/pi).
%   Speeds reported in km/s (/3280); plot coordinates in km via the listing's
%   111 km/deg, which the book says holds only at the equator (:12325).
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION (listing_32_05.m; book text zarchan.txt:12477-12518)
%
%     1  Lines 11-28 of the transcription are FIGURE TEXT leaked from the
%        facing page -- axis ticks, the split words "C ro / ss ra / ng e (k m)"
%        and the legend of Fig. 32.12, which belongs to Listing 32.4.  Dropped.
%     2  "if (VBOM,VBOLIM & VBOT,7.5)" is UNRESOLVED despite the file's header
%        claiming none: the repair pass rewrites a bare "if A , B" and this one
%        hides two comparisons inside one paren.  Resolved "VBOM < VBOLIM &&
%        VBOT < 7.5".  Both directions are stated in prose -- :12475 "Cases are
%        ruled out if the required target velocity exceeds 7.5 km/s" and, for
%        the sibling, :12327 "If the required interceptor velocity is less than
%        VBOLIM, the intercept is considered a success" -- and both are MEASURED
%        by BOUND rows 5 and 6: flip either and the matching row goes red.
%     3  "break end" -> "break; end" (relayout artifact).
%     4  RDESKM = 10000 is DEAD CODE here (32.3/32.4 build the aim point from
%        it; in 32.5 XLONGFDEG is the loop variable).  Kept only because the
%        PROBE rows need a 10,000-km aim point.  Also dead: I, PI, S, ALTM.
%     5  32.5 stops the TF loop at TFTOT-50 where the siblings stop at TFTOT,
%        and takes ALTFKM from XTF where 32.3 (:12337) and 32.4 (:12421) print
%        sqrt(XT^2+YTF^2+ZTF^2).  Neither sibling quirk is imported here.
%
%   THE MINUS SIGN, ADJUDICATED THREE WAYS.  :12558 prints the depressed case
%   as "TLOFT = 2200"; that is the book's minus mis-mapped to a 2, and it is
%   -200.  Two intact witnesses (:12473, :12172) plus BENCHMARK 3, which
%   settles it numerically: +2200 s of flight time would need far more than the
%   published 7.23 km/s.
%
%   THE ANTIPODE.  The listing's grid contains XLONGFDEG = 180 at XLATFDEG = 0.
%   A true antipode gives CROSSMAG = 0 in LAMBERT3D and non-finite velocities;
%   the book's 57.3 deg/rad puts that point 179.987 deg away, so the solve is
%   ill-conditioned rather than singular.  Where one does return non-finite,
%   "NaN < 7.5" is false and the listing silently drops the point.  Reproduced,
%   counted and reported -- never repaired into a pass.
%
%   COST.  The listing's grid is 37 x 49 longitudes/latitudes x 14 launch times
%   x ~20 intercept times, each inner pass a Lambert solve.  The default
%   SUBSAMPLES it (every 3rd longitude, 6th latitude, 4th launch time, 4th
%   intercept time); 'book_grid', true restores it and should take minutes --
%   AN ESTIMATE FROM THE ITERATION COUNT.  This file has never been executed,
%   so no runtime is claimed in it.  The one optimisation used cannot change
%   the answer: VBOT depends on neither TLAUNCH nor TF, so an aim point failing
%   the target-velocity gate is abandoned before inner loops that could have
%   recorded nothing anyway.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS -- book values grepped from zarchan_decoded.txt, lines in
%   .source.  Nothing is read off a figure.
%     1-3  PROBE, and the only 3-significant-figure values the chapter offers.
%          :12172 publishes the required target burnout speed for the
%          10,000-km trajectory: 7.19 km/s minimum energy, 7.33 lofted
%          (TLOFT = 500), 7.23 depressed (TLOFT = -200).  Those are Listing
%          32.1's numbers, but 32.5 reaches them down the SAME code path -- one
%          LAMBERT3D leg from the launch site to an aim point 10,000 km away --
%          so the probe puts the aim point there and reads VBOT.  Band 0.5%:
%          the three values are 1-2% apart, so 0.5% still separates them.
%     4    Interceptor downrange 6660 km (:12519; :12430 for the sibling).
%          Near-tautological alone, but it is the only number in the Fig. 32.13
%          paragraph and it pins the 111 km/deg convention.
%     5-6  BOUND: no accepted case exceeds VBOLIM or 7.5 km/s -- defect 2,
%          measured rather than argued.
%     7-8  SHAPE, from :12558.  "the size of the defended area region shrinks"
%          -> area(500) < area(0).  "roughly the same size" -> the ORDERING the
%          sentence asserts, |area(-200)/area(0) - 1| < |area(500)/area(0) - 1|;
%          "roughly" is an adjective, and giving it a percentage would be
%          inventing a book value.  AREA = DISTINCT AIM POINTS, because the
%          plus signs repeat for every (TLAUNCH, TF) that works.
%     9    INFO, NOT SCORED: extent of the defended set beside Fig. 32.13's
%          axis ticks (:12520-12529).  The ticks survived extraction but an
%          axis limit is MATLAB autoscale, not a measurement.
%    10    SKIP: ":12558 the difference ... is not as great as it was in the
%          previous section" needs Listing 32.4's areas, a different port's
%          output.  Not absorbed into a widened band.
%
%   WHAT THIS SHOULD PRINT.  *** DERIVED, NOT MEASURED. ***  The minimum-energy
%   probe is a 90-deg range angle, whose closed-form minimum-energy speed is
%   sqrt((GM/A)*2*sin45/(1+sin45)) = 23607 ft/s = 7.197 km/s, so BENCHMARK 1
%   should land within a few tenths of a percent of 7.19, and 2 and 3 should
%   both come out ABOVE it (a minimum is a minimum).  The lofted sweep should
%   defend fewer distinct aim points than the minimum-energy sweep, the
%   depressed sweep about as many.
%
%   OUTPUTS.  R.sweep (one entry per TLOFT: accepted records, .x_km/.y_km plus
%   signs, .area_deg distinct aim points, .area_nogate for the delete
%   experiment), R.probe, R.bench, R.selfcheck, R.pass, R.verdict, R.p,
%   R.summary, R.units.
%
%   See also MGS.ZARCHAN.IMPULSIVEICBM (Listing 32.1, where the probe values
%   are published), MGS.ZARCHAN.STRATEGICENGAGEMENT, MGS.ZARCHAN.BENCHSUITE.
arguments
    % Listing 32.5 line 1 and the loop headers, verbatim.
    opts.VBOLIM       (1,1) double {mustBeFinite, mustBePositive} = 5.0    % km/s
    opts.VBOTLIM      (1,1) double {mustBeFinite, mustBePositive} = 7.5    % km/s
    opts.XLONGMDEG    (1,1) double {mustBeFinite} = 60.0
    opts.XLATMDEG     (1,1) double {mustBeFinite} = 0.0
    opts.XLONGTDEGIC  (1,1) double {mustBeFinite} = 0.0
    opts.XLATTDEGIC   (1,1) double {mustBeFinite} = 0.0
    opts.ALTMKMIC     (1,1) double {mustBeFinite} = 0.0
    opts.ALTFKM_MIN   (1,1) double {mustBeFinite} = 50.0
    % The three published trajectory types.  -200 is the DEPRESSED case; the
    % book prints it as "2200" at :12558 and as -200 at :12473 and :12172.
    opts.TLOFT        double {mustBeReal, mustBeFinite} = [0, 500, -200]
    % Grid.  Strides subsample the listing's own vectors, so every point used
    % is a point the listing visits; book_grid overrides all four with 1.
    opts.book_grid      (1,1) logical = false
    opts.lon_stride     (1,1) double {mustBeInteger, mustBePositive} = 3
    opts.lat_stride     (1,1) double {mustBeInteger, mustBePositive} = 6
    opts.tlaunch_stride (1,1) double {mustBeInteger, mustBePositive} = 4
    opts.tf_stride      (1,1) double {mustBeInteger, mustBePositive} = 4
    % Aim-point range for the probe rows, in km along a great circle.
    opts.probe_rdeskm   (1,1) double {mustBeFinite, mustBePositive} = 10000.
    opts.roundtrip_n    (1,1) double {mustBeInteger, mustBePositive} = 25
    % THE ONLY SWITCHES.  Not nargin: with only name-value options declared,
    % MATLAB reports nargin = 0 however many were passed.
    opts.sweep      (1,1) logical = true
    opts.benchmark  (1,1) logical = true
    opts.render     (1,1) logical = true
    opts.plot       (1,1) logical = false
    opts.assert     (1,1) logical = true
end

A      = 2.0926E7;   % ft
FTPKM  = 3280.;      % ft/km, the book's
DEGRAD = 57.3;       % deg/rad, the book's
KMPDEG = 111.;       % km/deg, the listing's plotting convention

% ---- the listing's own grid vectors, then the stride ----------------------
lonAll = 20:5:200;  latAll = -60:2.5:60;  tlAll = 300:100:1600;
if opts.book_grid
    ls = 1;  as = 1;  ts = 1;  fs = 1;
else
    ls = opts.lon_stride;  as = opts.lat_stride;
    ts = opts.tlaunch_stride;  fs = opts.tf_stride;
end
lonList = lonAll(1:ls:end);
latList = latAll(1:as:end);
tlList  = tlAll(1:ts:end);
TFSTEP  = 50*fs;
if numel(lonList) < 2 || numel(latList) < 2
    error('mgs:zarchan:defendedArea3d:grid', ...
        ['the strides leave %d longitude(s) x %d latitude(s).  An "area" ' ...
         'measured on fewer than two of each is not an area, and the shape ' ...
         'rows would then be adjudicating a single point.'], ...
        numel(lonList), numel(latList));
end

% =========================================================================
%   PROBE.  One Lambert leg per trajectory type, to a 10,000-km aim point.
%   This is Listing 32.1's geometry reached through 32.5's own code path.
% =========================================================================
probeLonDeg = DEGRAD*opts.probe_rdeskm*FTPKM/A;
PR = struct('tloft', {}, 'tftot_s', {}, 'vbot_kms', {}, 'distfkm', {}, ...
            'vbot_xy_kms', {});
for k = 1:numel(opts.TLOFT)
    [vb, vbxy, tft, dk] = probeVbot(opts.TLOFT(k), probeLonDeg, ...
        opts.XLONGTDEGIC, opts.XLATTDEGIC, A, FTPKM, DEGRAD);
    PR(k).tloft = opts.TLOFT(k);  PR(k).tftot_s = tft;
    PR(k).vbot_kms = vb;  PR(k).distfkm = dk;  PR(k).vbot_xy_kms = vbxy;
end

% =========================================================================
%   THE SWEEP.  Transcribed from zarchan.txt:12500-12518.
% =========================================================================
SW = emptySweep();
if opts.sweep
    for k = 1:numel(opts.TLOFT)
        SW(k) = sweepOne(opts.TLOFT(k), lonList, latList, tlList, TFSTEP, ...
            opts.XLONGTDEGIC, opts.XLATTDEGIC, opts.XLONGMDEG, opts.XLATMDEG, ...
            opts.ALTMKMIC, opts.VBOLIM, opts.VBOTLIM, opts.ALTFKM_MIN, ...
            A, FTPKM, DEGRAD, KMPDEG, opts.roundtrip_n);
    end
end

R = struct();
R.units = ['internal ft, ft/s (A = 2.0926e7 ft, GM = 1.4077e16 ft^3/s^2, ' ...
           '57.3 deg/rad); speeds km/s (/3280); plot km (111 km/deg); time s'];
R.probe = PR;
R.sweep = SW;
R.p = struct('VBOLIM',opts.VBOLIM, 'VBOTLIM',opts.VBOTLIM, ...
             'XLONGMDEG',opts.XLONGMDEG, 'XLATMDEG',opts.XLATMDEG, ...
             'XLONGTDEG',opts.XLONGTDEGIC, 'XLATTDEG',opts.XLATTDEGIC, ...
             'TLOFT',opts.TLOFT, 'ALTFKM_MIN',opts.ALTFKM_MIN, ...
             'lon_deg',lonList, 'lat_deg',latList, 'tlaunch_s',tlList, ...
             'tf_step_s',TFSTEP, 'book_grid',opts.book_grid, ...
             'probe_lon_deg',probeLonDeg, 'interceptor_km',opts.XLONGMDEG*KMPDEG, ...
             'units',R.units);

R.selfcheck = structuralChecks(SW, opts, A);

if opts.benchmark
    R.bench = benchmarks(R, SW, PR, opts, KMPDEG);
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
if opts.plot,   drawArea(R); end

msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the ' ...
           'book.  R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = 'not one book value could be evaluated; an unvalidated port must not be handed back silently.';
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
        error('mgs:zarchan:defendedArea3d:benchmark', '%s\n\n%s', msg, R.summary);
    else
        % WARN AND RETURN, complete, so the caller can see which side moved.
        warning('mgs:zarchan:defendedArea3d:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function S = emptySweep()
% ⚠️ FIELD ORDER IS LOAD-BEARING.  sweepOne builds its S field by field and the
%   caller does SW(k) = S; MATLAB refuses a struct assignment whose fields are
%   the same but in a different order, so these two lists must stay in step.
S = struct('tloft', {}, 'lon_deg', {}, 'lat_deg', {}, 'tlaunch', {}, 'tf', {}, ...
           'vbom', {}, 'vbot', {}, 'altfkm', {}, 'x_km', {}, 'y_km', {}, ...
           'area_deg', {}, 'n_area', {}, 'n_records', {}, 'n_eval', {}, ...
           'n_aim', {}, 'n_aim_vbot_ok', {}, 'n_nonfinite', {}, ...
           'area_nogate', {}, 'fixed_dev_ft', {}, 'rt_target_ft', {}, ...
           'rt_missile_ft', {});
end


% =============================================================================
function [VBOT, VBOT_XY, TFTOT, DISTFKM] = probeVbot(TLOFT, lonFdeg, ...
                                    lonTdeg, latTdeg, A, FTPKM, DEGRAD)
%PROBEVBOT  The target Lambert leg alone, at an aim point on the equator
%   lonFdeg degrees away.  Listing 32.1 (zarchan.txt:12160-12162) computes
%   exactly this and the book publishes the answer at :12172.
%   VBOT_XY is 32.1's own two-component speed; VBOT is 32.5's three-component
%   one.  With XLATFDEG = XLATTDEG = 0 both z terms are identically zero, so
%   the two must agree -- which is what makes the comparison legitimate.
XLONGF = lonFdeg/DEGRAD;   XLATF = 0.;
XF = A*cos(XLATF)*cos(XLONGF);
YF = A*cos(XLATF)*sin(XLONGF);
ZF = A*sin(XLATF);
XLONGT = lonTdeg/DEGRAD;   XLATT = latTdeg/DEGRAD;
XT = A*cos(XLATT)*cos(XLONGT);
YT = A*cos(XLATT)*sin(XLONGT);
ZT = A*sin(XLATT);
DISTFKM = distance3dkm(XF, YF, ZF, XT, YT, ZT, A, FTPKM);
TFTOT = 252. + .223*DISTFKM - (5.44E-6)*DISTFKM*DISTFKM + TLOFT;
[VRX, VRY, VRZ] = lambert3d(XT, YT, ZT, TFTOT, XF, YF, ZF, 0);
VBOT    = sqrt(VRX^2 + VRY^2 + VRZ^2)/FTPKM;
VBOT_XY = sqrt(VRX^2 + VRY^2)/FTPKM;
end


% =============================================================================
function S = sweepOne(TLOFT, lonList, latList, tlList, TFSTEP, lonTdeg, ...
                      latTdeg, lonMdeg, latMdeg, ALTMKMIC, VBOLIM, VBOTLIM, ...
                      ALTFKM_MIN, A, FTPKM, DEGRAD, KMPDEG, rtN)
%SWEEPONE  The listing's four nested loops for ONE trajectory type.
%   Every constant arrives as an argument: MATLAB local functions cannot see
%   the main function's workspace.
%
%   Restructured only in ONE way, and it cannot change the recorded set: the
%   target's Lambert leg and its Kepler propagation depend on the aim point and
%   TF but NOT on TLAUNCH (TGOLAM = TFTOT - T with T identically 0), so they
%   are computed once per aim point and cached per TF.  The loop ORDER, the
%   break on ALTFKM and the acceptance test are the listing's.

ALTM   = ALTMKMIC*FTPKM;
% REFERENCE copies of the two points 32.5 holds fixed.  The loop below
% re-derives both on every aim point, exactly as the listing does, and
% structural check 1 measures them against these.  That is the check that
% notices an incomplete 32.4 -> 32.5 edit (32.4 sweeps the target site) or an
% incomplete 32.3 -> 32.5 one (32.3 sweeps the interceptor site).
XT0 = A*cos(latTdeg/DEGRAD)*cos(lonTdeg/DEGRAD);
YT0 = A*cos(latTdeg/DEGRAD)*sin(lonTdeg/DEGRAD);
ZT0 = A*sin(latTdeg/DEGRAD);
XM0 = (A + ALTM)*cos(latMdeg/DEGRAD)*cos(lonMdeg/DEGRAD);
YM0 = (A + ALTM)*cos(latMdeg/DEGRAD)*sin(lonMdeg/DEGRAD);
ZM0 = (A + ALTM)*sin(latMdeg/DEGRAD);

% cols: lon lat tlaunch tf vbom vbot altfkm XMD YMD ZMD XTF YTF ZTF
rec = zeros(1024, 13);   nrec = 0;
% cols: lon lat vbom   (every INNER pass that got as far as a missile solve)
cnd = zeros(4096, 3);    ncnd = 0;

nEval = 0;  nNonFinite = 0;  nAimOk = 0;  fixedDev = 0;
rtT = zeros(1, numel(lonList)*numel(latList));  nrtT = 0;

for lonF = lonList
    for latF = latList
        % The listing RE-DERIVES both fixed points inside the loop, the target
        % site from XLONGTDEGIC/XLATTDEGIC.  Reproduced, and measured.
        XLONGT = lonTdeg/DEGRAD;   XLATT = latTdeg/DEGRAD;
        XT = A*cos(XLATT)*cos(XLONGT);
        YT = A*cos(XLATT)*sin(XLONGT);
        ZT = A*sin(XLATT);
        XLONGM = lonMdeg/DEGRAD;   XLATM = latMdeg/DEGRAD;
        XM = (A + ALTM)*cos(XLATM)*cos(XLONGM);
        YM = (A + ALTM)*cos(XLATM)*sin(XLONGM);
        ZM = (A + ALTM)*sin(XLATM);
        fixedDev = max(fixedDev, max(abs([XT YT ZT XM YM ZM] - ...
                                         [XT0 YT0 ZT0 XM0 YM0 ZM0])));

        XLONGF = lonF/DEGRAD;  XLATF = latF/DEGRAD;
        XF = A*cos(XLATF)*cos(XLONGF);
        YF = A*cos(XLATF)*sin(XLONGF);
        ZF = A*sin(XLATF);

        DISTFKM = distance3dkm(XF, YF, ZF, XT, YT, ZT, A, FTPKM);
        TFTOT   = 252. + .223*DISTFKM - (5.44E-6)*DISTFKM*DISTFKM + TLOFT;

        [VRX, VRY, VRZ] = lambert3d(XT, YT, ZT, TFTOT - 0., XF, YF, ZF, 0);
        XTD = VRX;  YTD = VRY;  ZTD = VRZ;
        VBOT = sqrt(XTD^2 + YTD^2 + ZTD^2)/FTPKM;
        nEval = nEval + 1;
        if ~isfinite(VBOT), nNonFinite = nNonFinite + 1; end

        % VBOT is invariant over both inner loops, and "NaN < x" is false, so
        % failing here can record nothing.  Abandoning early is exactly
        % equivalent to running the loops out.
        if ~(VBOT < VBOTLIM), continue; end
        nAimOk = nAimOk + 1;

        % target round trip: Kepler-propagate the Lambert leg over TFTOT and
        % see whether it actually reaches the aim point.
        X1 = kepler1([XT YT ZT XTD YTD ZTD]/FTPKM, 0., TFTOT);
        nrtT = nrtT + 1;
        rtT(nrtT) = norm(X1(1:3)*FTPKM - [XF YF ZF]);

        kTF = zeros(1,0);  kP = zeros(3,0);  kA = zeros(1,0);
        for TLAUNCH = tlList
            for TF = (TLAUNCH + 60.):TFSTEP:(TFTOT - 50.)
                j = find(abs(kTF - TF) < 1e-9, 1);
                if isempty(j)
                    X1 = kepler1([XT YT ZT XTD YTD ZTD]/FTPKM, 0., TF);
                    p  = [X1(1); X1(2); X1(3)]*FTPKM;
                    kTF(end+1) = TF;              %#ok<AGROW>
                    kP(:,end+1) = p;              %#ok<AGROW>
                    kA(end+1) = (norm(p) - A)/FTPKM;  %#ok<AGROW>
                    j = numel(kTF);
                end
                XTF = kP(1,j);  YTF = kP(2,j);  ZTF = kP(3,j);
                ALTFKM = kA(j);
                if ALTFKM < ALTFKM_MIN
                    break
                end
                TGOLAMM = TF - TLAUNCH;
                [VRXM, VRYM, VRZM] = lambert3d(XM, YM, ZM, TGOLAMM, ...
                                               XTF, YTF, ZTF, 0);
                VBOM = sqrt(VRXM^2 + VRYM^2 + VRZM^2)/FTPKM;
                nEval = nEval + 1;
                if ~isfinite(VBOM), nNonFinite = nNonFinite + 1; end

                ncnd = ncnd + 1;
                if ncnd > size(cnd,1), cnd = [cnd; zeros(size(cnd))]; end %#ok<AGROW>
                cnd(ncnd,:) = [lonF, latF, VBOM];

                if (VBOM < VBOLIM && VBOT < VBOTLIM)
                    nrec = nrec + 1;
                    if nrec > size(rec,1), rec = [rec; zeros(size(rec))]; end %#ok<AGROW>
                    rec(nrec,:) = [lonF, latF, TLAUNCH, TF, VBOM, VBOT, ...
                                   ALTFKM, VRXM, VRYM, VRZM, XTF, YTF, ZTF];
                end
            end
        end
    end
end

rec = rec(1:nrec,:);  cnd = cnd(1:ncnd,:);

S.tloft    = TLOFT;
S.lon_deg  = rec(:,1).';   S.lat_deg = rec(:,2).';
S.tlaunch  = rec(:,3).';   S.tf      = rec(:,4).';
S.vbom     = rec(:,5).';   S.vbot    = rec(:,6).';
S.altfkm   = rec(:,7).';
S.x_km     = S.lon_deg*KMPDEG;   S.y_km = S.lat_deg*KMPDEG;
if nrec > 0
    S.area_deg = unique(rec(:,1:2), 'rows');
else
    S.area_deg = zeros(0,2);
end
S.n_area      = size(S.area_deg, 1);
S.n_records   = nrec;
S.n_eval      = nEval;
S.n_aim       = numel(lonList)*numel(latList);
S.n_aim_vbot_ok = nAimOk;
S.n_nonfinite = nNonFinite;
% DELETE EXPERIMENT: the same candidates with the interceptor-velocity gate
% removed.  The target-velocity gate is still on, so this is the area the
% interceptor gate is responsible for shrinking.
if ncnd > 0
    S.area_nogate = size(unique(cnd(:,1:2), 'rows'), 1);
else
    S.area_nogate = 0;
end
S.fixed_dev_ft = fixedDev;
S.rt_target_ft = rtT(1:nrtT);

% missile round trip on a strided sample of the accepted records.
if nrec > 0
    idx = unique(round(linspace(1, nrec, min(rtN, nrec))));
    rm  = zeros(1, numel(idx));
    for i = 1:numel(idx)
        r  = rec(idx(i),:);
        X1 = kepler1([XM0 YM0 ZM0 r(8) r(9) r(10)]/FTPKM, r(3), r(4));
        rm(i) = norm(X1(1:3)*FTPKM - r(11:13));
    end
    S.rt_missile_ft = rm;
else
    S.rt_missile_ft = zeros(1,0);
end
end


% =============================================================================
function DISTKM = distance3dkm(XT, YT, ZT, XF, YF, ZF, A, FTPKM)
%DISTANCE3DKM  zarchan.txt:10423, printed under Listing 28.2.  Great-circle arc
%   on a sphere of radius A, returned in KILOMETRES.  The clamp inside acos is
%   OURS: CBETA can come back as 1+2e-16 for a near-zero arc and acos would
%   then return a complex distance that still plots.  The else branch is the
%   book's own and is unreachable for |CBETA| <= 1; kept so nobody restores it.
Rr  = sqrt(XT^2 + YT^2 + ZT^2);
RFr = sqrt(XF^2 + YF^2 + ZF^2);
CBETA = (XT*XF + YT*YF + ZT*ZF)/(Rr*RFr);
if CBETA <= 1.
    BETA = acos(max(-1, min(1, CBETA)));
    DISTKM = A*BETA/FTPKM;
else
    DISTKM = (XF - XT)/FTPKM;
end
end


% =============================================================================
function [VRX, VRY, VRZ] = lambert3d(XT, YT, ZT, TF, XF, YF, ZF, SWITCH1)
%LAMBERT3D  zarchan.txt:10428, the routine Listing 32.5 calls but does not
%   print.  Secant search on the flight-path angle inside a bracket, then the
%   in-plane velocity direction as C1*r0 + C2*rf.  The bracket-update
%   comparisons are the 3-D twin of Listing 13.2's, resolved and then confirmed
%   against Table 13.2's published iterate sequence by
%   MGS.ZARCHAN.LAMBERTEFFICIENT; not re-derived here.  The book's dead locals
%   are dropped, not "fixed".
%
%   TWO GUARDS ARE OURS, and both matter on 32.5's grid, which the siblings'
%   geometries never reach:
%     * the clamp inside acos(COS_T).  |COS_T| <= 1 mathematically, but at the
%       near-antipodal aim points it rounds to -1-2e-16 and acos then returns a
%       COMPLEX angle that propagates all the way to a complex "speed" which
%       still compares, still plots, and never warns.
%     * the 100-iteration cap.  The book has no cap at all.  This one RETURNS
%       NaN rather than the last iterate: a non-convergent solve would
%       otherwise hand the caller a finite number for "VBOM < VBOLIM" to judge.
%       NaN makes that comparison false, which is exactly what the listing's
%       own antipodal cases do.
PI = 3.1415926535898;  GM = 1.4077E16;  HALFPI = PI/2.;
RF0X = XF-XT;  RF0Y = YF-YT;  RF0Z = ZF-ZT;
R0DOTRF  = XT*XF + YT*YF + ZT*ZF;
R0DOTRF0 = XT*RF0X + YT*RF0Y + ZT*RF0Z;
R0MAG  = sqrt(XT^2 + YT^2 + ZT^2);
RFMAG  = sqrt(XF^2 + YF^2 + ZF^2);
RF0MAG = sqrt(RF0X^2 + RF0Y^2 + RF0Z^2);
RATIO   = R0MAG/RFMAG;
GMDIVR0 = GM/R0MAG;
COS_T   = R0DOTRF/(R0MAG*RFMAG);
VNUMER  = GMDIVR0*(1. - COS_T);
T_MIN = 0.;
if SWITCH1 == 0
    G_MIN = HALFPI - acos(R0DOTRF0/(R0MAG*RF0MAG));  G_MAX = HALFPI;
    THETA = acos(max(-1, min(1, COS_T)));
else
    G_MIN = -HALFPI;  G_MAX = -HALFPI + acos(R0DOTRF0/(R0MAG*RF0MAG));
    THETA = 2.*PI - acos(max(-1, min(1, COS_T)));
end
SIN_T = sin(THETA);  COT_HALFT = 1./tan(THETA/2.);
GAMMA = (G_MAX + G_MIN)/2.;  GOLD = G_MIN;  TOLD = 0.;  T = 0.;  ITERS = 1;
RV0MAG = NaN;  NEXT = GAMMA;   % defensive; both are set on every pass below.
%   RV0MAG's NaN is load-bearing, not decoration: a degenerate geometry makes
%   the while condition false on entry, and NaN is then what reaches the
%   caller's gate instead of an uninitialised-variable error or a stale number.

while abs(TF - T) > (.00000001*TF)
    SIN_G = sin(GAMMA);  COS_G = cos(GAMMA);  TAN_G = SIN_G/COS_G;
    COS_TPLUSG = cos(THETA + GAMMA - 2.*PI);
    TERM1 = (RATIO*COS_G - COS_TPLUSG)*COS_G;
    RV0MAG = sqrt(VNUMER/TERM1);
    LAMBDA = RV0MAG*RV0MAG/GMDIVR0;
    if LAMBDA < 1.9999999
        TERM0 = sqrt(2./LAMBDA - 1.);
        TERM1 = (TAN_G*(1.-COS_T) + (1.-LAMBDA)*SIN_T)/((2.-LAMBDA)*RATIO);
        TERM2 = (COS_G + COS_G)/(LAMBDA*TERM0*TERM0*TERM0);
        TERM3 = atan2(TERM0, (COS_G*COT_HALFT - SIN_G));
        T = (R0MAG/(RV0MAG*COS_G))*(TERM1 + TERM2*TERM3);
    elseif LAMBDA > 2.0000001
        TERM0 = sqrt(1. - 2./LAMBDA);
        TERM1 = (TAN_G*(1.-COS_T) + (1.-LAMBDA)*SIN_T)/((2.-LAMBDA)*RATIO);
        TERM2 = COS_G/(LAMBDA*TERM0*TERM0*TERM0);
        TERM3 = SIN_G - COS_G*COT_HALFT;
        TERM3 = log((TERM3 - TERM0)/(TERM3 + TERM0));
        T = (R0MAG/(RV0MAG*COS_G))*(TERM1 - TERM2*TERM3);
    else
        TERM0 = COS_G*COT_HALFT;  TERM1 = TERM0 - SIN_G;
        TERM0 = (3*TERM0*TERM1 + 1.)/(TERM1*TERM1*TERM1);
        T = TERM0*(2.*R0MAG)/(3.*RV0MAG);
    end
    if ~isfinite(T), break; end
    if T > TF  && GAMMA < G_MAX, G_MAX = GAMMA; end
    if T < 0.  && GAMMA < G_MAX, G_MAX = GAMMA; end
    if T < TF  && GAMMA > G_MIN, G_MIN = GAMMA;  T_MIN = T; end
    if T < 0.
        NEXT = (G_MIN + G_MAX)/2.;  GOLD = G_MIN;  TOLD = T_MIN;
    else
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
    % The book has no iteration cap.  100 is OURS and it is a REFUSAL, not a
    % repair: a non-convergent solve leaves RV0MAG at whatever the last pass
    % produced, and the caller's "VBOM < VBOLIM" then decides on a number that
    % means nothing.  Returning NaN makes that comparison false, which is the
    % same thing the book's own antipodal cases do.
    if ITERS > 100
        VRX = NaN;  VRY = NaN;  VRZ = NaN;
        return
    end
end

if SWITCH1 == 0
    ANGLE = HALFPI - GAMMA;
else
    ANGLE = GAMMA - HALFPI;
end
SINA = sin(ANGLE);  COSA = cos(ANGLE);
V1X=XT; V1Y=YT; V1Z=ZT;  V2X=XF; V2Y=YF; V2Z=ZF;
MAG1   = sqrt(V1X*V1X + V1Y*V1Y + V1Z*V1Z);
DOTMAG = V1X*V2X + V1Y*V2Y + V1Z*V2Z;
CROSSX = V1Y*V2Z - V1Z*V2Y;
CROSSY = V1Z*V2X - V1X*V2Z;
CROSSZ = V1X*V2Y - V1Y*V2X;
CROSSMAG = sqrt(CROSSX*CROSSX + CROSSY*CROSSY + CROSSZ*CROSSZ);
C2 = MAG1*SINA/CROSSMAG;
C1 = COSA/MAG1 - DOTMAG*C2/(MAG1*MAG1);
VRX = RV0MAG*(C2*V2X + C1*V1X);
VRY = RV0MAG*(C2*V2Y + C1*V1Y);
VRZ = RV0MAG*(C2*V2Z + C1*V1Z);
end


% =============================================================================
function X1 = kepler1(X0, T0, T1)
%KEPLER1  zarchan.txt:10486, printed under Listing 28.3.  Laguerre-Conway
%   iteration on the universal variable, KN = 10 fixed passes.
%   INPUTS AND OUTPUTS ARE km AND km/s -- the listing divides by 3280 going in
%   and multiplies by 3280 coming out.
%   Book defect kept visible, not reproduced: the |DT| < TLIMIT branch fills
%   X1 = X0 and then FALLS THROUGH into the solver, which divides by DT = 0.
%   Returned here instead.  MGS.ZARCHAN.KEPLERVSDIRECT is the authority on this
%   routine; this copy exists because Listing 32.5 calls it.
GMX = 398923.;  REX = 6380.;  TLIMIT = 1.E-10;  KN = 10;  MUQR = 1.;
DT = T1 - T0;
if abs(DT) < TLIMIT
    X1 = X0(1:6);
    return
end
TIME_FACTOR = sqrt(REX^3/GMX);
VEX = REX/TIME_FACTOR;
DT  = DT/TIME_FACTOR;
R0 = X0(1:3)/REX;
V0 = X0(4:6)/VEX;
R0MAG = sqrt(R0(1)^2 + R0(2)^2 + R0(3)^2);
V0MAG = sqrt(V0(1)^2 + V0(2)^2 + V0(3)^2);
D0 = R0(1)*V0(1) + R0(2)*V0(2) + R0(3)*V0(3);
SIGMA0 = D0/MUQR;
ALP0 = 2./R0MAG - V0MAG*V0MAG;
X = ALP0*DT;
if ALP0 <= 0.
    X = .1*DT/R0MAG;
end
U1 = 0.; U2 = 0.; U3 = 0.; DFX = 1.;
for K = 1:KN
    if ALP0 < 0.
        Y = ALP0*X*X;  YQR = sqrt(-Y);
        CY = (1. - cosh(YQR))/Y;  SY = (sinh(YQR) - YQR)/(YQR^3);
    elseif ALP0 == 0.
        Y = 0.;  CY = .5;  SY = 1./6.;
    else
        Y = ALP0*X*X;  YQR = sqrt(Y);
        CY = (1. - cos(YQR))/Y;  SY = (YQR - sin(YQR))/(YQR^3);
    end
    U1 = X*(1. - Y*SY);
    U2 = X*X*CY;
    U3 = X*X*X*SY;
    FX   = R0MAG*U1 + SIGMA0*U2 + U3 - DT*MUQR;
    DFX  = SIGMA0*U1 + (1. - ALP0*R0MAG)*U2 + R0MAG;
    DFX2 = SIGMA0*(1. - Y*CY) + (1. - ALP0*R0MAG)*U1;
    SDFX = DFX/abs(DFX);
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
for I = 1:3
    X1(I)   = (F*R0(I)  + G*V0(I))*REX;
    X1(I+3) = (DF*R0(I) + DG*V0(I))*VEX;
end
end


% =============================================================================
function K = structuralChecks(SW, opts, A)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and never evidence that
%   this port reproduces Zarchan.  .names AND .oks must both exist and be the
%   SAME LENGTH or benchSuite puts ZERO rows on the board.
names = {};  oks = [];  detail = {};
tolFt = 1e-4*A;                       % ~2093 ft on a 2.09e7 ft radius

if isempty(SW)
    names{end+1} = 'sweep ran';
    oks(end+1)   = false;
    detail{end+1} = '''sweep'' is false -- an unrun check is not a pass';
    K.names = names;  K.oks = logical(oks);  K.detail = detail;
    K.all_ok = false;  K.first_bad = detail{1};
    return
end

% 1  the DEFENDED AREA method's defining invariant: target launch site and
%    interceptor site are FIXED and only the aim point moves.  In 32.4 the
%    target site is the loop variable and in 32.3 the interceptor site is, so
%    an incomplete edit of either sibling lands exactly here.
dev = max([SW.fixed_dev_ft]);
names{end+1} = 'launch site AND interceptor re-derive identically on every aim point';
oks(end+1)   = dev <= 1e-9;
detail{end+1} = sprintf(['max drift of any component of the target or ' ...
    'interceptor position over the sweep = %.3e ft'], dev);

% 2  every ACCEPTED case cleared the 50-km intercept-altitude floor.  The
%    listing enforces it with a break, so a break placed one loop out would
%    leave low intercepts in the record.
alt = [SW.altfkm];
if isempty(alt), amin = NaN; else, amin = min(alt); end
names{end+1} = 'every accepted intercept is above the ALTFKM floor';
oks(end+1)   = ~isempty(alt) && amin >= opts.ALTFKM_MIN;
detail{end+1} = sprintf('min accepted intercept altitude = %.2f km (floor %.0f), %d records', ...
    amin, opts.ALTFKM_MIN, numel(alt));

% 3  DELETE THE INTERCEPTOR-VELOCITY GATE.  With VBOLIM removed the defended
%    area must GROW.  This is the only check that proves the plotted region is
%    produced by the gate rather than by the grid or by the altitude break.
grew = true;  txt = '';
for k = 1:numel(SW)
    grew = grew && (SW(k).area_nogate > SW(k).n_area);
    txt = [txt sprintf('TLOFT %g: %d -> %d;  ', SW(k).tloft, SW(k).n_area, ...
                       SW(k).area_nogate)]; %#ok<AGROW>
end
names{end+1} = 'DELETE the VBOM < VBOLIM gate -> defended area grows on every trajectory';
oks(end+1)   = grew;
detail{end+1} = ['distinct aim points, gated -> ungated: ' txt];

% 4  the target's Lambert leg and KEPLER1 describe the same orbit: propagating
%    the Lambert velocity over TFTOT must arrive at the aim point.  Nothing in
%    the listing ever checks this, and a mismatched GM or a km/ft slip between
%    the two routines would leave the sweep running smoothly and wrong.
rt = [SW.rt_target_ft];
names{end+1} = 'LAMBERT3D target leg lands on the aim point when flown by KEPLER1';
[okT, dT] = q90ok(rt, tolFt);
oks(end+1) = okT;
detail{end+1} = sprintf('%s (tol %.0f ft, %d aim points)', dT, tolFt, numel(rt));

% 5  same for the missile leg, on a sample of accepted records.
rm = [SW.rt_missile_ft];
names{end+1} = 'LAMBERT3D missile leg lands on the predicted intercept point';
[okM, dM] = q90ok(rm, tolFt);
oks(end+1) = okM;
detail{end+1} = sprintf('%s (tol %.0f ft, %d sampled records)', dM, tolFt, numel(rm));

K.names = names;  K.oks = logical(oks);  K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    j = find(~K.oks, 1);
    K.first_bad = sprintf('%s (%s)', names{j}, detail{j});
end
end

function [ok, txt] = q90ok(v, tol)
%Q90OK  90th percentile by sort and index -- no Statistics toolbox.
v = v(isfinite(v));
if isempty(v)
    ok = false;  txt = 'no finite residuals -- an unrun check is not a pass';
    return
end
s = sort(v);
q = s(max(1, ceil(0.9*numel(s))));
ok = q <= tol;
txt = sprintf('q90 = %.2f ft, max = %.2f ft', q, max(s));
end


% =============================================================================
function B = benchmarks(R, SW, PR, opts, KMPDEG)
%BENCHMARKS  Ten rows.  Every band's provenance is in the header.
B = emptyBench();
SRC_TRAJ = 'Ch.32 prose on Fig. 32.1, zarchan.txt:12172';
SRC_FIG13 = 'Fig. 32.13 paragraph, zarchan.txt:12519';
SRC_GATE = 'Listing 32.5 acceptance test (zarchan.txt:12512) + prose (:12475)';
SRC_1458 = 'Fig. 32.14/32.15 paragraph, zarchan.txt:12558';

% 1-3  PROBE.  The only three-significant-figure values the chapter offers.
bookV = [7.19, 7.33, 7.23];
bookN = {'minimum energy (TLOFT = 0)', 'lofted (TLOFT = 500)', ...
         'depressed (TLOFT = -200)'};
bookT = [0, 500, -200];
for i = 1:3
    j = find([PR.tloft] == bookT(i), 1);
    what = sprintf('required target burnout speed, %s', bookN{i});
    if isempty(j)
        B = addSkip(B, what, sprintf('%.2f', bookV(i)), sprintf( ...
            'TLOFT = %g is not in this run''s ''TLOFT'' list', bookT(i)), ...
            SRC_TRAJ, 'km/s');
        continue
    end
    dtl = sprintf(['aim point %.4f deg east on the equator (= %.1f km by ' ...
        'the book''s own A and 57.3), TFTOT = %.1f s; 32.1''s two-component ' ...
        'VBOT is %.5f km/s, identical because both z terms are zero'], ...
        R.p.probe_lon_deg, PR(j).distfkm, PR(j).tftot_s, PR(j).vbot_xy_kms);
    if bookT(i) == -200
        dtl = [dtl sprintf(['.  THIS ROW ADJUDICATES THE MINUS SIGN: :12558 ' ...
            'prints "TLOFT = 2200", which would make TFTOT = %.1f s and ' ...
            'demand far more than 7.23 km/s'], PR(j).tftot_s + 2400)]; %#ok<AGROW>
    end
    B = addVal(B, what, bookV(i), PR(j).vbot_kms, '0.5%', 0.005, SRC_TRAJ, ...
               'km/s', dtl);
end

% 4  the interceptor's downrange distance, the one number in the Fig. 32.13
%    paragraph.  Pins the 111 km/deg convention the listing plots with.
B = addVal(B, 'interceptor downrange from target launch point', 6660., ...
    opts.XLONGMDEG*KMPDEG, '1e-9 rel', 1e-9, SRC_FIG13, 'km', ...
    ['XLONGMDEG*111 as the listing plots.  The great-circle distance by the ' ...
     'book''s own A and 57.3 deg/rad is 6680 km, so the 20 km gap is the ' ...
     '111 km/deg approximation the book declares at :12325, not an error.  ' ...
     'The listing plots XLONGFDEG*111 at :12513']);

% 5-6  BOUND: the acceptance test, measured.  These are what a flipped
%      comparison in defect 2 would light up.
if isempty(SW)
    B = addSkip(B, 'no accepted case exceeds VBOLIM', ...
        sprintf('%.1f km/s', opts.VBOLIM), '''sweep'' is false', SRC_GATE, 'km/s');
    B = addSkip(B, 'no accepted case exceeds the 7.5 km/s target-velocity cut', ...
        '7.5 km/s', '''sweep'' is false', SRC_GATE, 'km/s');
else
    vm = [SW.vbom];  vt = [SW.vbot];
    okM = ~isempty(vm) && all(vm < opts.VBOLIM);
    B = addBound(B, 'no accepted case exceeds VBOLIM', ...
        sprintf('every VBOM < %.1f', opts.VBOLIM), ...
        sprintf('max VBOM = %.4f over %d records', maxOr(vm), numel(vm)), ...
        okM, SRC_GATE, 'km/s', ...
        'flipping this comparison would admit every case the book rules out');
    okT = ~isempty(vt) && all(vt < opts.VBOTLIM);
    B = addBound(B, 'no accepted case exceeds the 7.5 km/s target-velocity cut', ...
        'every VBOT < 7.5', ...
        sprintf('max VBOT = %.4f over %d records', maxOr(vt), numel(vt)), ...
        okT, SRC_GATE, 'km/s', ...
        '"Cases are ruled out if the required target velocity exceeds 7.5 km/s"');
end

% 7-8  SHAPE, from the Fig. 32.14/32.15 sentence.
i0 = findLoft(SW, 0);  iL = findLoft(SW, 500);  iD = findLoft(SW, -200);
if isempty(i0) || isempty(iL)
    B = addSkip(B, 'lofted target shrinks the defended area', ...
        'area(TLOFT=500) < area(TLOFT=0)', ...
        'both TLOFT = 0 and TLOFT = 500 must be swept in one call', SRC_1458, '-');
else
    B = addShape(B, 'lofted target shrinks the defended area', ...
        'area(TLOFT=500) < area(TLOFT=0)', ...
        sprintf('%d < %d distinct aim points', SW(iL).n_area, SW(i0).n_area), ...
        SW(iL).n_area < SW(i0).n_area, SRC_1458, ...
        '"the size of the defended area region shrinks"');
end
if isempty(i0) || isempty(iL) || isempty(iD)
    B = addSkip(B, 'depressed target leaves the area roughly unchanged', ...
        '|area(-200)/area(0) - 1| < |area(500)/area(0) - 1|', ...
        'all three TLOFT values must be swept in one call', SRC_1458, '-');
elseif SW(i0).n_area == 0
    B = addSkip(B, 'depressed target leaves the area roughly unchanged', ...
        '|area(-200)/area(0) - 1| < |area(500)/area(0) - 1|', ...
        'the minimum-energy sweep defended nothing, so there is no ratio', ...
        SRC_1458, '-');
else
    rD = SW(iD).n_area/SW(i0).n_area;  rL = SW(iL).n_area/SW(i0).n_area;
    B = addShape(B, 'depressed target leaves the area roughly unchanged', ...
        '|area(-200)/area(0) - 1| < |area(500)/area(0) - 1|', ...
        sprintf('depressed x%.3f vs lofted x%.3f of the minimum-energy area', rD, rL), ...
        abs(rD - 1) < abs(rL - 1), SRC_1458, ...
        ['"roughly the same size" is an adjective, so it is scored as the ' ...
         'ORDERING the sentence asserts, not as an invented percentage']);
end

% 9  INFO: extent beside Fig. 32.13's axis ticks.  NOT SCORED.
iInfo = i0;  if isempty(iInfo) && ~isempty(SW), iInfo = 1; end
if isempty(iInfo) || isempty(SW(iInfo).x_km)
    B = addInfo(B, 'extent of the defended set vs Fig. 32.13''s axes', ...
        'x to 14000 km, y within +/-4000 km (axis ticks)', 'no records', ...
        'Fig. 32.13 axis tick text, zarchan.txt:12520-12529', ...
        'an axis limit is MATLAB autoscale, not a published measurement');
else
    B = addInfo(B, 'extent of the defended set vs Fig. 32.13''s axes', ...
        'x to 14000 km, y within +/-4000 km (axis ticks)', ...
        sprintf('TLOFT %g: x %.0f..%.0f km, |y| <= %.0f km', SW(iInfo).tloft, ...
                min(SW(iInfo).x_km), max(SW(iInfo).x_km), ...
                max(abs(SW(iInfo).y_km))), ...
        'Fig. 32.13 axis tick text, zarchan.txt:12520-12529', ...
        ['an axis limit is MATLAB autoscale over the plotted set, not a ' ...
         'published measurement, and a coarse grid can only shrink it']);
end

% 10  SKIP: the cross-section comparison.
B = addSkip(B, 'min-energy vs lofted gap is smaller here than in Listing 32.4', ...
    'qualitative: "not as great as it was in the previous section"', ...
    ['needs the LAUNCH AREA DENIED areas of Listing 32.4 (zarchan.txt:12394-' ...
     '12428), which are a different port''s output.  Computing 1 - area(500)/' ...
     'area(0) here and comparing it against a number this file never produced ' ...
     'would be a band widened to swallow a missing input'], SRC_1458, '-');
end

function j = findLoft(SW, v)
if isempty(SW), j = []; return; end
j = find(abs([SW.tloft] - v) < 1e-9, 1);
end

function y = maxOr(x)
if isempty(x), y = NaN; else, y = max(x); end
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
L{end+1} = 'Zarchan Listing 32.5 -- 3-D DEFENDED AREA (target impact point swept)';
L{end+1} = sprintf('   %s', R.units);
L{end+1} = sprintf(['   target launch %.1f/%.1f deg fixed, interceptor %.1f/%.1f deg ' ...
    '(= %.0f km downrange), VBOLIM = %.2f km/s, VBOT cut %.2f km/s'], ...
    R.p.XLATTDEG, R.p.XLONGTDEG, R.p.XLATMDEG, R.p.XLONGMDEG, ...
    R.p.interceptor_km, R.p.VBOLIM, R.p.VBOTLIM);
L{end+1} = sprintf(['   grid: %d longitudes %g..%g, %d latitudes %g..%g, ' ...
    '%d launch times %g..%g, TF step %g s%s'], numel(R.p.lon_deg), ...
    R.p.lon_deg(1), R.p.lon_deg(end), numel(R.p.lat_deg), R.p.lat_deg(1), ...
    R.p.lat_deg(end), numel(R.p.tlaunch_s), R.p.tlaunch_s(1), ...
    R.p.tlaunch_s(end), R.p.tf_step_s, bstr(R.p.book_grid));
L{end+1} = '';
L{end+1} = 'PROBE (Listing 32.1 geometry down 32.5''s own target-Lambert path)';
for i = 1:numel(R.probe)
    L{end+1} = sprintf('   TLOFT %+6g   TFTOT %8.1f s   D %8.1f km   VBOT %.4f km/s', ...
        R.probe(i).tloft, R.probe(i).tftot_s, R.probe(i).distfkm, ...
        R.probe(i).vbot_kms); %#ok<AGROW>
end
L{end+1} = '';
if isempty(R.sweep)
    L{end+1} = 'SWEEP  NOT RUN (''sweep'', false).';
else
    L{end+1} = 'SWEEP';
    for i = 1:numel(R.sweep)
        S = R.sweep(i);
        L{end+1} = sprintf(['   TLOFT %+6g   area %4d / %4d aim points   ' ...
            '%6d records   %7d solves   %d non-finite   ungated area %4d'], ...
            S.tloft, S.n_area, S.n_aim, S.n_records, S.n_eval, ...
            S.n_nonfinite, S.area_nogate); %#ok<AGROW>
    end
end
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
                L{end+1} = sprintf('   SKIP %-56s %s', b.what, b.book_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s', b.why); %#ok<AGROW>
            case 'info'
                L{end+1} = sprintf('   INFO %-56s %s', b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        %s  [%s]', b.why, b.source); %#ok<AGROW>
            case {'shape', 'bound'}
                L{end+1} = sprintf('   %-4s %-56s %s', vstr(b.ok), b.what, b.ours_str); %#ok<AGROW>
                L{end+1} = sprintf('        want: %s  [%s]', b.book_str, b.source); %#ok<AGROW>
            otherwise
                L{end+1} = sprintf(['   %-4s %-56s book %s %s / ours %s %s ' ...
                    '(%+.4f%%, band %s)'], vstr(b.ok), b.what, b.book_str, ...
                    b.units, b.ours_str, b.units, b.pct, b.band); %#ok<AGROW>
                L{end+1} = sprintf('        [%s]', b.source); %#ok<AGROW>
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('   %d scored, %d passed, %d skipped/info -> R.pass = %s', ...
        sum(ev), sum(ev & [R.bench.ok]), sum(~ev), vstr(R.pass));
end
L{end+1} = '';
L{end+1} = 'REMINDER: the default grid SUBSAMPLES the listing''s (''book_grid'', true';
L{end+1} = '   restores 37 x 49 x 14).  Areas are counts of DISTINCT AIM POINTS on';
L{end+1} = '   whatever grid was run, so they are comparable ACROSS TLOFT and not';
L{end+1} = '   against any absolute number -- the book publishes none.';
s = strjoin(L, newline);
end

function s = vstr(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end

function s = bstr(tf)
if tf, s = '  [book grid]'; else, s = '  [subsampled]'; end
end


% =============================================================================
function drawArea(R)
if isempty(R.sweep), return; end
figure;
for i = 1:numel(R.sweep)
    subplot(numel(R.sweep), 1, i);
    plot(R.sweep(i).x_km, R.sweep(i).y_km, 'r+'); grid on
    xlabel('Downrange (km)'); ylabel('Crossrange (km)');
    title(sprintf('Defended area, TLOFT = %g  (%d aim points)', ...
        R.sweep(i).tloft, R.sweep(i).n_area));
end
end
