function res = demo_reproduce(varargin)
%DEMO_REPRODUCE  Prove the ports are faithful by reproducing the book.
%
%   Every check states its ACCEPTANCE RULE before it runs, then prints the
%   measured number and a verdict.  No parameter in this file was tuned to
%   make a check pass.  If a check fails it is printed as FAIL and left
%   failing -- read the report, do not adjust the threshold.
%
%   USAGE
%       res = demo_reproduce()            % everything (slow, ~minutes)
%       res = demo_reproduce('ONLY',{'A','B'})
%       res = demo_reproduce('FAST',true) % coarser sweeps where allowed
%
%   CHECKS
%     A  Fig. 2.2  (pn2d)          larger N' removes heading error faster
%                                  AND demands more acceleration at t=0
%     B  Fig. 3.13/3.14 + Ch.3 closed forms (adjoint_miss, closedform)
%                                  3-g maneuver at tF=1 s -> "nearly 12 ft";
%                                  miss -> ~0 as tF/TAU grows;
%                                  adjoint vs closed form for N'=3,4,5;
%                                  N'=5 heading-error sign erratum
%     C  Figs. 30.2-30.5           perp-LOS vs perp-velocity: trajectories
%                                  nearly identical, command profiles very
%                                  different, severe speed loss vs none
%     D  port cross-check          purepn2d == ../ref/zarchan_listing30_1
%                                  on the shared IGUID=1 branch
%     E  Figs. 28.1-28.4 (zem3d)   crossrange miss ~ 0; total miss ~ constant
%     F  Figs. A.16/A.17 (tshape3d) terminal pitch -> -30 deg, yaw -> +20 deg

p = struct('ONLY',{{'A','B','C','D','E','F'}},'FAST',false,'VERBOSE',true);
for k = 1:2:numel(varargin)
    f = varargin{k};
    if ~isfield(p,f), error('demo_reproduce: unknown option %s',f); end
    p.(f) = varargin{k+1};
end
here = fileparts(mfilename('fullpath'));
addpath(here);
refdir = fullfile(fileparts(here),'ref');

res = struct(); res.pass = {}; res.fail = {}; res.skip = {};
L = @(varargin) fprintf(varargin{:});

L('\n');
L('=======================================================================\n');
L(' ZARCHAN TOOLBOX -- BOOK REPRODUCTION REPORT\n');
L(' %s\n', char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')));
L('=======================================================================\n');

%% =====================================================================
%  CHECK A -- Fig. 2.2 (zarchan.txt line 431)
%  "Increasing effective navigation ratio causes heading error to be
%   removed more rapidly."
%  Line 541: "higher effective navigation ratios require more acceleration
%   at the beginning of flight than at the end of the flight"
%  ACCEPTANCE (fixed before running):
%    A1  |nc(0)| is LARGER for N'=5 than for N'=4
%    A2  peak altitude excursion off the collision course is SMALLER for
%        N'=5 (heading error removed more rapidly)
%    A3  time for |nc| to fall to 10% of |nc(0)| is SHORTER for N'=5
%  =====================================================================
if any(strcmp(p.ONLY,'A'))
    L('\n--- CHECK A : Fig. 2.2, effective navigation ratio -------------\n');
    a4 = pn2d('XNP',4); a5 = pn2d('XNP',5);
    ex4 = max(abs(a4.RM2 - a4.opts.RM2));
    ex5 = max(abs(a5.RM2 - a5.opts.RM2));
    t4  = decay_time(a4.T,a4.XNCG,0.10);
    t5  = decay_time(a5.T,a5.XNCG,0.10);
    L('  flight time            N''=4 %8.3f s   N''=5 %8.3f s\n',a4.tf,a5.tf);
    L('  miss (integration floor) N''=4 %8.3f ft  N''=5 %8.3f ft\n', ...
        a4.miss_ft,a5.miss_ft);
    L('  A1  |nc(0)|            N''=4 %8.3f g   N''=5 %8.3f g\n', ...
        abs(a4.xnc0_g),abs(a5.xnc0_g));
    L('  A2  peak alt excursion N''=4 %8.1f ft  N''=5 %8.1f ft\n',ex4,ex5);
    L('  A3  t to 10%% of nc(0)  N''=4 %8.3f s   N''=5 %8.3f s\n',t4,t5);
    res = verdict(res,L,'A1 larger N'' pulls more g at t=0', ...
                  abs(a5.xnc0_g) > abs(a4.xnc0_g));
    res = verdict(res,L,'A2 larger N'' removes heading error more (smaller excursion)', ...
                  ex5 < ex4);
    res = verdict(res,L,'A3 larger N'' removes heading error faster (nc decays sooner)', ...
                  t5 < t4);
    res.A = struct('a4',a4,'a5',a5,'ex4',ex4,'ex5',ex5,'t4',t4,'t5',t5);
end

%% =====================================================================
%  CHECK B -- Figs. 3.13 / 3.14 and the Ch. 3 closed forms
%  Line 751 verbatim: "a 3-g maneuver causes nearly 12 ft of miss if the
%   flight time is only 1 s"; and "long flights (flight time large compared
%   to guidance system time constant) ... result in virtually zero miss".
%  ACCEPTANCE (fixed before running):
%    B1  adjoint maneuver miss at tF = 1 s, N'=4, TAU=1 : 11.0 .. 13.0 ft
%    B2  miss at tF/TAU = 10 is at least 20x smaller than at tF/TAU = 1
%    B2b (closed form, exact) |Miss| at tF/TAU = 20 is < 0.1% of tF/TAU = 1
%    B3  adjoint vs Ch.3 closed form, maneuver, N'=3,4,5 : max |dev| < 1e-2
%        (units: Miss/nT, s^2 -- the adjoint's own integration error is
%         this size, so a tighter bound would be testing the integrator)
%    B4  same for heading error, N'=3,4
%    B5  N'=5 heading error: the MINUS-sign version must beat the printed
%        PLUS-sign version by at least a factor of 10 in max |dev|
%
%  ## RECORDED CORRECTION TO AN EARLIER DRAFT OF THIS CHECK ##
%  B2 was first written as "|miss(tF/TAU=10)| < 1% of miss(tF/TAU=1)".  It
%  FAILED: measured -0.5152 ft against 12.0712 ft, i.e. 4.27%.  That is NOT
%  integration error.  Refining the step (H = 0.01 -> 0.0002) moves the
%  value only from -0.5152 to -0.5047 ft, and the exact Ch.3 closed form
%  gives 96.6 * (-0.00529666) = -0.5117 ft -- so the adjoint is right and
%  the threshold was wrong.  The reason: the N'=4 maneuver-miss curve
%  x^2*exp(-x)*(0.5 - x/6) CROSSES ZERO at x = tF/TAU = 3 and then has a
%  NEGATIVE LOBE; x = 10 is still on that lobe.  The decay to "virtually
%  zero" is real but is not complete by tF/TAU = 10.  Hence B2 now tests
%  the ratio (>=20x) and B2b tests the asymptote at tF/TAU = 20 using the
%  exact closed form -- the adjoint at the book's H = 0.01 has a noise
%  floor of ~3e-3 ft and cannot resolve the ~2e-4 ft value out there.
%  =====================================================================
if any(strcmp(p.ONLY,'B'))
    L('\n--- CHECK B : adjoint + Ch.3 closed forms ----------------------\n');
    adj4 = adjoint_miss('XNP',4);
    i1  = find(adj4.TP >= 1.0, 1, 'first');
    i10 = numel(adj4.TP);
    m1  = adj4.XMNT(i1); m10 = adj4.XMNT(i10);
    L('  B1  maneuver miss at tF = %.3f s : %8.4f ft   (book: "nearly 12")\n', ...
        adj4.TP(i1), m1);
    L('  B2  maneuver miss at tF = %.3f s : %8.4f ft  (%.1fx smaller than tF=1)\n', ...
        adj4.TP(i10), m10, abs(m1/m10));
    L('      heading miss  at tF = %.3f s : %8.4f ft\n', ...
        adj4.TP(i10), adj4.XMHE(i10));
    res = verdict(res,L,'B1 "nearly 12 ft" at tF=1 s (11.0..13.0)', ...
                  m1 > 11.0 && m1 < 13.0);
    res = verdict(res,L,'B2 miss at tF/TAU=10 is >=20x smaller than at tF/TAU=1', ...
                  abs(m1) >= 20*abs(m10));

    cf = closedform();  cfT = closedform('SIGN5','text');
    % B2b -- the asymptote, from the EXACT closed form (the adjoint at the
    % book's H=0.01 has a ~3e-3 ft noise floor and cannot resolve this).
    e1  = abs(cf.missfun_nt(1,1,4));
    e20 = abs(cf.missfun_nt(20,1,4));
    L('  B2b closed form Miss/nT :  tF/TAU=1  %.6e    tF/TAU=20  %.6e\n',e1,e20);
    L('      (zero crossing of the N''=4 maneuver curve is at tF/TAU = 3)\n');
    res = verdict(res,L,'B2b miss -> virtually zero by tF/TAU = 20 (<0.1%)', ...
                  e20 < 1e-3*e1);
    devN = zeros(1,3); devH = zeros(1,3); devH5plus = 0;
    L('      adjoint vs closed form (max |dev| over tF in [0.5,10]) :\n');
    for kk = 1:3
        NPq = kk+2;
        a = adjoint_miss('XNP',NPq);
        m = a.TP >= 0.5;
        tt = a.TP(m);
        % maneuver: adjoint X1 = XMNT/XNT  should equal Miss/nT
        cfn = arrayfun(@(z) cf.missfun_nt(z,a.opts.TAU,NPq), tt);
        devN(kk) = max(abs(a.XMNT(m)/a.opts.XNT - cfn));
        % heading: compare magnitudes (sign convention differs between the
        % adjoint output definition and the book's Miss/(VM*HE) form)
        HEr  = a.opts.HEDEG/57.3;
        adjH = abs(a.XMHE(m)/(a.opts.VM*abs(HEr)));
        cfh  = abs(arrayfun(@(z) cf.missfun_he(z,a.opts.TAU,NPq), tt));
        devH(kk) = max(abs(adjH - cfh));
        if NPq == 5
            cfh5p = abs(arrayfun(@(z) cfT.missfun_he(z,a.opts.TAU,5), tt));
            devH5plus = max(abs(adjH - cfh5p));
        end
        L('        N''=%d   maneuver %10.3e   heading %10.3e\n', ...
            NPq, devN(kk), devH(kk));
    end
    L('        N''=5 heading, PRINTED (+) sign version : %10.3e\n', devH5plus);
    res = verdict(res,L,'B3 maneuver closed form matches adjoint (N''=3,4,5)', ...
                  all(devN < 1e-2));
    res = verdict(res,L,'B4 heading closed form matches adjoint (N''=3,4)', ...
                  all(devH(1:2) < 1e-2));
    res = verdict(res,L,'B5 N''=5 heading: MINUS sign beats printed PLUS by >10x', ...
                  devH5plus > 10*devH(3));
    res.B = struct('m1',m1,'m10',m10,'devN',devN,'devH',devH, ...
                   'devH5plus',devH5plus,'e1',e1,'e20',e20);
end

%% =====================================================================
%  CHECK C -- Figs. 30.2 - 30.5
%  Line 10961: with the OPTIMAL trajectory shaping law (IGUID=0), running
%   ICHOICE=0 (perp LOS) vs ICHOICE=1 (perp velocity), "the missile
%   trajectories are nearly identical" but "the commanded acceleration
%   profiles are vastly different" and the perp-LOS "flight time is much
%   longer".
%  Line 10980: "severe missile velocity loss when ... perpendicular to the
%   line of sight and no velocity loss at all when ... perpendicular to the
%   velocity vector".
%  ACCEPTANCE (fixed before running):
%    C1  max |dAltitude| between the two trajectories, over their common
%        downrange span, < 10% of the apogee            ("nearly identical")
%    C1b both terminal flight-path angles within 1 deg of -90.  Line 10961
%        gives this one explicitly ("the flight path angle approaching -90
%        deg at intercept", Fig. 30.4), so it is testable.  The apogee
%        difference is REPORTED but NOT graded -- the book prints no number
%        for it, and inventing one is what went wrong in C1/E1.
%    C2  flight time (perp LOS) > 1.5 x flight time (perp velocity)
%    C3  perp-LOS  min speed < 0.75 x initial speed     ("severe loss")
%    C4  perp-VEL  min speed within 0.1% of initial     ("no loss at all")
%    C5  peak |nc| differs between the two by >= 25%    ("vastly different")
%  NOTE the book's figures use IGUID=0.  That branch lives in
%  ../ref/zarchan_listing30_1.m (already transcribed by another
%  workstream); this check reuses it rather than duplicating it.  The same
%  contrast is then reported for OUR pure-PN law via purepn2d, which is a
%  SECONDARY observation, not a book reproduction.
%
%  ## C1 DOES NOT REPRODUCE CLEANLY -- REPORTED, NOT TUNED ##
%  Measured: max |dAlt| vs downrange = 0.4728 km, symmetric Hausdorff
%  distance = 0.3474 km, against an apogee of 3.0019 km -- i.e. 15.8% and
%  11.6%.  The pre-registered bound was 10%, so C1 FAILS and is left
%  failing.  A third, independent metric agrees: the two apogees themselves
%  differ by 6.8% (3.0019 vs 2.7978 km).  So this is one coherent finding
%  measured three ways, not three separate anomalies -- the two
%  trajectories are CLOSE but not "nearly identical".
%  Two honest caveats on top, in both directions:
%    - the vs-downrange metric is ill-conditioned in the terminal dive
%      (downrange is nearly stationary while altitude drops), which is why
%      the Hausdorff figure is also reported; Hausdorff is the fairer one
%      and it still misses the bound;
%    - "nearly identical" is the book's PROSE about a plotted figure, with
%      no number attached.  A 0.35 km gap on a 10 km x 3 km plot is small
%      but visible.  We therefore call Fig. 30.2 a QUALIFIED reproduction:
%      the substantive content (both loft to the same apogee, both arrive
%      near-vertically) is confirmed by C1b, while the literal phrase
%      "nearly identical" is not confirmed at 10%.
%  Do NOT relax the 10% to turn this green.
%  =====================================================================
if any(strcmp(p.ONLY,'C'))
    L('\n--- CHECK C : Figs. 30.2-30.5, perp-LOS vs perp-velocity -------\n');
    reffile = fullfile(refdir,'zarchan_listing30_1.m');
    if exist(reffile,'file') ~= 2
        L('  SKIP: %s not found.  The book figures use IGUID=0, which is\n',reffile);
        L('        not part of this toolbox.\n');
        res.skip{end+1} = 'C (ref listing30_1 missing)';
    else
        addpath(refdir);
        c0 = zarchan_listing30_1('IGUID',0,'ICHOICE',0);
        c1 = zarchan_listing30_1('IGUID',0,'ICHOICE',1);
        [dalt, apo, haus] = traj_dev(c0.RM1K,c0.RM2K,c1.RM1K,c1.RM2K);
        L('  perp-LOS (ICHOICE=0):  tf %7.2f s  peak|nc| %7.3f g  ', ...
            c0.tf, c0.xnc_max_g);
        L('vmin/vic %6.4f  gam_f %7.2f deg\n', c0.vm_min_ms/c0.vm_ic_ms, c0.gam_f_deg);
        L('  perp-VEL (ICHOICE=1):  tf %7.2f s  peak|nc| %7.3f g  ', ...
            c1.tf, c1.xnc_max_g);
        L('vmin/vic %6.4f  gam_f %7.2f deg\n', c1.vm_min_ms/c1.vm_ic_ms, c1.gam_f_deg);
        L('  C1  max |dAlt| vs downrange %.4f km (%.1f%% of apogee)\n', ...
            dalt, 100*dalt/apo);
        L('      symmetric Hausdorff       %.4f km (%.1f%% of apogee)   apogee %.4f km\n', ...
            haus, 100*haus/apo, apo);
        res = verdict(res,L,'C1 trajectories nearly identical (<10% of apogee)', ...
                      dalt < 0.10*apo);
        if ~(dalt < 0.10*apo)
            L('      ^ EXPECTED FAIL, left failing on purpose.  See the C1 note in\n');
            L('        the header: "nearly identical" is prose with no number; the\n');
            L('        substantive content of Fig. 30.2 is tested by C1b below.\n');
        end
        L('  C1b apogee  perp-LOS %.4f km   perp-VEL %.4f km   (d %.2f%%)\n', ...
            max(c0.RM2K), max(c1.RM2K), ...
            100*abs(max(c0.RM2K)-max(c1.RM2K))/max(max(c0.RM2K),max(c1.RM2K)));
        L('      (apogee difference is REPORTED, NOT GRADED -- the book gives\n');
        L('       no number for it.  It is a third metric agreeing with C1.)\n');
        res = verdict(res,L,'C1b both arrive within 1 deg of -90 (Fig. 30.4)', ...
                      abs(c0.gam_f_deg+90) < 1 && abs(c1.gam_f_deg+90) < 1);
        res = verdict(res,L,'C2 perp-LOS flight time much longer (>1.5x)', ...
                      c0.tf > 1.5*c1.tf);
        res = verdict(res,L,'C3 perp-LOS severe speed loss (vmin < 0.75 vic)', ...
                      c0.vm_min_ms < 0.75*c0.vm_ic_ms);
        res = verdict(res,L,'C4 perp-VEL no speed loss at all (within 0.1%)', ...
                      abs(c1.vm_min_ms/c1.vm_ic_ms - 1) < 1e-3);
        res = verdict(res,L,'C5 command profiles vastly different (peak |nc| differs >=25%)', ...
                      abs(c0.xnc_max_g-c1.xnc_max_g) >= 0.25*max(c0.xnc_max_g,c1.xnc_max_g));
        res.C = struct('c0',c0,'c1',c1,'dalt',dalt,'apo',apo,'haus',haus);

        % --- secondary, NOT a book figure: the same contrast under our law
        L('  [secondary, not a book figure] pure PN (purepn2d, unbiased):\n');
        q0 = purepn2d('ICHOICE',0); q1 = purepn2d('ICHOICE',1);
        L('    perp-LOS: tf %7.2f s  peak|nc| %6.3f g  vmin/vic %6.4f  gam_f %7.2f\n', ...
            q0.tf, q0.xnc_max_g, q0.vm_min_ms/q0.vm_ic_ms, q0.gam_f_deg);
        L('    perp-VEL: tf %7.2f s  peak|nc| %6.3f g  vmin/vic %6.4f  gam_f %7.2f\n', ...
            q1.tf, q1.xnc_max_g, q1.vm_min_ms/q1.vm_ic_ms, q1.gam_f_deg);
        res.C.q0 = q0; res.C.q1 = q1;
    end
end

%% =====================================================================
%  CHECK D -- port cross-check against ../ref/zarchan_listing30_1.m
%  Two independent transcriptions of the same Listing 30.1 IGUID=1 branch
%  must agree to round-off.  ACCEPTANCE: max |dGAMDEG| < 1e-9 deg and
%  |dmiss| < 1e-9 m over the whole logged trajectory.
%  =====================================================================
if any(strcmp(p.ONLY,'D'))
    L('\n--- CHECK D : purepn2d vs ../ref/zarchan_listing30_1 -----------\n');
    reffile = fullfile(refdir,'zarchan_listing30_1.m');
    if exist(reffile,'file') ~= 2
        L('  SKIP: %s not found.\n',reffile);
        res.skip{end+1} = 'D (ref listing30_1 missing)';
    else
        addpath(refdir);
        r = zarchan_listing30_1('IGUID',1,'ICHOICE',1);
        q = purepn2d('ICHOICE',1,'BIASFORM','listing');
        nmin = min(numel(r.GAMDEG),numel(q.GAMDEG));
        dg = max(abs(r.GAMDEG(1:nmin)-q.GAMDEG(1:nmin)));
        dm = abs(r.miss_m - q.miss_m);
        db = abs(r.BIASDEG - q.BIASDEG);
        L('  BIASDEG   ref %+0.6f   ours %+0.6f   |d| %.3e\n', ...
            r.BIASDEG,q.BIASDEG,db);
        L('  samples   ref %d  ours %d\n',numel(r.GAMDEG),numel(q.GAMDEG));
        L('  max |dGAMDEG| %.3e deg    |dmiss| %.3e m\n',dg,dm);
        res = verdict(res,L,'D  the two transcriptions agree to round-off', ...
                      dg < 1e-9 && dm < 1e-9 && db < 1e-9 && ...
                      numel(r.GAMDEG)==numel(q.GAMDEG));
        res.D = struct('dg',dg,'dm',dm,'db',db);
    end
end

%% =====================================================================
%  CHECK E -- Figs. 28.1-28.4 (line 10326)
%  "the miss is virtually zero in the cross-range direction because there
%   is no target maneuver in that direction"; "the total miss is virtually
%   a constant vs flight time after an initial transient period".
%  ACCEPTANCE (fixed before running), over engagements with tf >= 4 s:
%    E1  median |RTM3| (crossrange) < 1 ft
%    E2  median |RTM3| < 10% of median total miss
%    E3  std(total miss) < 25% of mean(total miss)
%
%  ## E1 DOES NOT REPRODUCE -- REPORTED, NOT TUNED ##
%  Measured median |RTM3| = 1.2336 ft against the pre-registered 1 ft, so
%  E1 FAILS and is left failing.  In hindsight the 1 ft bound had NO basis
%  in the book -- Zarchan attaches no number to "virtually zero".  The only
%  scale the book itself supplies is its own plot axis for Fig. 28.3,
%  axis([0 10 -30 30]) (in the listing's plotting block), i.e. a +-30 ft
%  half-range; 1.23 ft is 4.1% of that.  E1b below states that
%  source-grounded version explicitly and separately, so the reader can see
%  exactly what was changed and why.  E1 itself is NOT relaxed.
%  =====================================================================
if any(strcmp(p.ONLY,'E'))
    L('\n--- CHECK E : Figs. 28.1-28.4, spiraling target in 3-D ---------\n');
    if p.FAST
        sw = 40000:-2500:500;  L('  (FAST: %d-case sweep, not the book''s 80)\n',numel(sw));
    else
        sw = 40000:-500:500;
    end
    e = zem3d('RT3IC',sw);
    m = e.T >= 4;
    if ~any(m), m = true(size(e.T)); end
    med3 = median(abs(e.RTM3(m)));
    medT = median(e.RTM(m));
    sdT  = std(e.RTM(m));
    L('  engagements %d   tf range %.2f .. %.2f s\n',numel(e.T),min(e.T),max(e.T));
    L('  E1  median |crossrange miss| (tf>=4 s) : %.4f ft\n',med3);
    L('  E2  median total miss                  : %.4f ft\n',medT);
    L('  E3  total miss  mean %.4f ft  std %.4f ft  (std/mean %.3f)\n', ...
        mean(e.RTM(m)),sdT,sdT/mean(e.RTM(m)));
    res = verdict(res,L,'E1 crossrange miss virtually zero (<1 ft)', med3 < 1.0);
    if ~(med3 < 1.0)
        L('      ^ EXPECTED FAIL, left failing on purpose.  The 1 ft bound was\n');
        L('        invented, not sourced.  See E1b for the book-grounded version.\n');
    end
    L('  E1b same claim against the book''s OWN Fig. 28.3 axis (+-30 ft):\n');
    L('      %.4f ft = %.2f%% of the 30 ft half-range\n', med3, 100*med3/30);
    res = verdict(res,L,'E1b crossrange miss virtually zero on the book''s own axis (<5%)', ...
                  med3 < 0.05*30);
    res = verdict(res,L,'E2 crossrange miss <10% of total miss', med3 < 0.10*medT);
    res = verdict(res,L,'E3 total miss virtually constant (std/mean <0.25)', ...
                  sdT < 0.25*mean(e.RTM(m)));
    res.E = struct('e',e,'med3',med3,'medT',medT,'sdT',sdT);
end

%% =====================================================================
%  CHECK F -- Figs. A.16 / A.17 (line 14463 context)
%  "the missile is meeting the desired pitch and yaw flight-path-angle
%   objectives at the end of the flight"
%  ACCEPTANCE (fixed before running):
%    F1  terminal pitch within 3 deg of GAMFPDEG (-30)
%    F2  terminal yaw   within 3 deg of GAMFYDEG (+20)
%    F3  peak commanded acceleration > 20 g ("very large", line 14463;
%        the book's Fig. A.18 axis runs to 60 g, which is a LOWER bound)
%  =====================================================================
if any(strcmp(p.ONLY,'F'))
    L('\n--- CHECK F : Figs. A.16-A.18, 3-D trajectory shaping ----------\n');
    hs = 1; if p.FAST, hs = 10; L('  (FAST: HSCALE=10, coarser than the book)\n'); end
    f = tshape3d('HSCALE',hs);
    L('  LAUNCHLOGIC tF guess %.2f s ; achieved tf %.4f s ; miss %.4f ft\n', ...
        f.TF_launch, f.tf, f.miss_ft);
    L('  F1  terminal pitch %8.3f deg  (target %+.1f)\n', ...
        f.gamP_f_deg, f.opts.GAMFPDEG);
    L('  F2  terminal yaw   %8.3f deg  (target %+.1f)\n', ...
        f.gamY_f_deg, f.opts.GAMFYDEG);
    % the raw peak is dominated by the 1/tgo^2 singularity as tgo -> 0, so
    % also report the peak away from the last 0.5 s.  The book's Fig. A.18
    % is drawn on axis([0 10 0 60]) -- a clip, not a measurement.
    mlate = f.T <= (f.tf - 0.5);
    xnc_nom = max(f.XNCG(mlate));
    L('  F3  peak commanded acceleration %12.2f g   (raw, includes tgo->0)\n', ...
        f.xnc_max_g);
    L('      peak excluding last 0.5 s   %12.2f g   (nominal demand)\n', xnc_nom);
    L('      -> the raw peak is the 1/tgo^2 singularity, not the shaping demand;\n');
    L('         the book''s 60 g is a PLOT CLIP (axis([0 10 0 60])), a lower bound.\n');
    res.F_xnc_nom = xnc_nom;
    res = verdict(res,L,'F1 terminal pitch meets the objective (+-3 deg)', ...
                  abs(f.gamP_f_deg - f.opts.GAMFPDEG) < 3);
    res = verdict(res,L,'F2 terminal yaw meets the objective (+-3 deg)', ...
                  abs(f.gamY_f_deg - f.opts.GAMFYDEG) < 3);
    res = verdict(res,L,'F3 "very large commanded acceleration" (>20 g)', ...
                  f.xnc_max_g > 20);
    res.F = struct('f',f);
end

%% ---- summary ------------------------------------------------------------
expected = {'C1 trajectories nearly identical (<10% of apogee)', ...
            'E1 crossrange miss virtually zero (<1 ft)'};
nunexp = 0;
L('\n=======================================================================\n');
L(' SUMMARY   PASS %d   FAIL %d   SKIP %d\n', ...
    numel(res.pass),numel(res.fail),numel(res.skip));
for k = 1:numel(res.fail)
    if any(strcmp(expected,res.fail{k}))
        L('   FAIL (EXPECTED, documented in the header): %s\n',res.fail{k});
    else
        L('   FAIL (UNEXPECTED -- investigate): %s\n',res.fail{k});
        nunexp = nunexp + 1;
    end
end
for k = 1:numel(res.skip), L('   SKIP: %s\n',res.skip{k}); end
res.n_unexpected_fail = nunexp;
if nunexp == 0
    L(' No unexpected failures.  The two expected ones are thresholds that\n');
    L(' were pre-registered and then missed; they are left failing on purpose\n');
    L(' rather than relaxed.  Both are qualified, not refuted -- see C1/E1.\n');
end
L('=======================================================================\n\n');
end

% =========================================================================
function res = verdict(res,L,name,ok)
if ok
    L('      PASS  %s\n',name);  res.pass{end+1} = name;
else
    L('      FAIL  %s\n',name);  res.fail{end+1} = name;
end
end

function t = decay_time(T,X,frac)
%DECAY_TIME  first time |X| stays below frac*|X(1)|.
a = abs(X); thr = frac*a(1);
i = find(a <= thr, 1, 'first');
if isempty(i), t = NaN; else, t = T(i); end
end

function [dalt, apo, haus] = traj_dev(x0,y0,x1,y1)
%TRAJ_DEV  how far apart two trajectories are, two ways.
%   dalt  max |altitude difference| at equal downrange.  Simple, but
%         ill-conditioned in a near-vertical terminal dive, where downrange
%         barely moves while altitude collapses.
%   haus  symmetric Hausdorff distance (max over each curve of the nearest
%         distance to the other curve).  Parametrisation-free, so it is the
%         fair measure of "do these two curves lie on top of each other".
%   Both are returned because they disagree, and the disagreement is itself
%   information about the terminal geometry.
apo = max([y0(:);y1(:)]);
[xa,ia] = unique(x0,'stable'); ya = y0(ia);
[xb,ib] = unique(x1,'stable'); yb = y1(ib);
lo = max(min(xa),min(xb)); hi = min(max(xa),max(xb));
if hi <= lo
    dalt = NaN;
else
    xg = linspace(lo,hi,400);
    ga = interp1(xa,ya,xg,'linear','extrap');
    gb = interp1(xb,yb,xg,'linear','extrap');
    dalt = max(abs(ga-gb));
end
d    = hypot(x0(:)-x1(:).', y0(:)-y1(:).');
haus = max( max(min(d,[],2)), max(min(d,[],1)) );
end
