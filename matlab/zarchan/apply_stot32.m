function A = apply_stot32(varargin)
%APPLY_STOT32  Zarchan's linear homing theory applied to our 32 measured flights.
%
%   A = apply_stot32()                 run everything, print the report
%   A = apply_stot32('NPERM',20000)    permutation count for the p-values
%   A = apply_stot32('WRITECSV',false) do not write the per-aircraft csv
%
%   READS  (read-only, never written)
%       ../data/truth32.csv            32 x 46 measured truth
%       ../data/traj/<TRIAL>_U<n>.csv  per-tick trajectories
%   USES
%       ./closedform.m                 Ch.2 profiles + Ch.3 lag-miss formulas
%   WRITES
%       ./apply_stot32_per_aircraft.csv   (this directory only)
%
%   ====================================================================
%   WHAT THIS FILE IS AND IS NOT
%   ====================================================================
%   It asks three questions Zarchan's LINEAR theory can be made to answer,
%   and reports the answers including the negative ones.
%
%     Q1  Ch.2 sizing.  nc(0) = VM*HE*N'/tF is the demand at RELEASE, and
%         line 540 verbatim says that is where the designer must size:
%           "From a system sizing point of view, the designer usually wants
%            to ensure that the acceleration capability of the missile is
%            adequate at the beginning of flight so that saturation can be
%            avoided."
%         Does demand/available at release predict our touchdown error?
%
%     Q2  Ch.19 lag criterion, line 5815 verbatim:
%           "...the ratio of the flight time tF ... to the guidance system
%            time constant T must now be greater than eight for there to be
%            negligible miss distance.  Recall that for the single time
%            constant system the number of guidance time constants had to be
%            greater than three for the miss distance to be negligible.  If
%            the number of guidance time constants is less than eight, it is
%            really a matter of luck as to how large or small the miss
%            distance will be."
%         Does tF/tau separate our failures from our successes?
%
%     Q3  Saturation budget.  Ch.2 line 539 verbatim: "the closed-form
%         solution for the miss distance y(tF) is zero.  In other words, as
%         long as the missile has sufficient acceleration capability, there
%         is no miss due to heading error!"  Ch.3 adds a single lag and gives
%         a non-zero miss.  Measured minus Ch.3-predicted is therefore the
%         part the linear theory does not own.
%
%   ---- THE DECOMPOSITION IN Q3 IS ONLY MEANINGFUL IF ITS PREMISES HOLD ----
%   Section 3 audits four premises BEFORE quoting the number, and says
%   plainly whether the decomposition failed.  See PREMISE AUDIT.
%
%   ====================================================================
%   OUR AUTHORITY MODEL  (transcribed from tot_node.py::_pn_cmd 2818-2884)
%   ====================================================================
%   LATERAL   roll = atan2(a_h, g*max(cos gam, 0.25))            [line 2831]
%             r_lim = radians(80)*abs(cos(theta_meas))           [line 2884]
%           => a_h_max = g*max(cos gam,0.25)*tan(radians(80)*|cos theta|)
%             traj/ has no pitch column, so theta is approximated by gamma
%             (alpha is a few degrees).  FLAGGED, not hidden; the resulting
%             median and steepest-decile numbers are printed so the
%             approximation can be checked against the briefed 2.03 / 1.49 g.
%   LONGITUDINAL  nz = cos gam + V*gdot/g, clipped to [pn_nz_min, 3.0]
%                 with pn_nz_min = -1.0 (contract_RRE1.json /values)
%           => a_down_max = (1 + cos gam)*g     (nz = -1)
%              a_up_max   = (3 - cos gam)*g     (nz = +3)
%
%   ====================================================================
%   NO STATISTICS TOOLBOX IS ASSUMED
%   ====================================================================
%   Spearman rho is computed here from tied ranks and its p-value by
%   permutation with a fixed seed, so every number is reproducible with
%   base MATLAB alone.

%% ---------------- options ----------------
p = struct('NPERM',20000,'WRITECSV',true,'NP',4.0,'NLAT',4.0,'NVERT',5.0, ...
           'PNLEAD',1.0,'NZMIN',-1.0,'NZMAX',3.0,'LOSFLOOR',1.0, ...
           'TAUMAX',30.0,'DIFFWIN',0.5,'CLOCK','sim','SEED',20260820);
for k = 1:2:numel(varargin)
    f = varargin{k};
    if ~isfield(p,f), error('apply_stot32: unknown option %s',f); end
    p.(f) = varargin{k+1};
end
G = 9.80665;
here = fileparts(mfilename('fullpath'));
root = fileparts(here);

fprintf('\n');
fprintf('================================================================\n');
fprintf(' apply_stot32 -- Zarchan linear theory vs 32 measured STOT flights\n');
fprintf('================================================================\n');

%% ---------------- load ----------------
T = readtable(fullfile(root,'data','truth32.csv'));
n = height(T);
fprintf('loaded truth32.csv : %d aircraft, %d columns\n', n, width(T));

switch lower(p.CLOCK)
    case 'sim',  tF = T.t_dive_sim_s;   clockname = 't_dive_sim_s (sim clock, freeze gaps removed)';
    case 'wall', tF = T.t_dive_wall_s;  clockname = 't_dive_wall_s (wall clock)';
    otherwise, error('CLOCK must be sim or wall');
end
fprintf('flight time tF     : %s\n', clockname);
fprintf('                     median %.2f s, range %.2f-%.2f s\n', ...
        median(tF), min(tF), max(tF));

err  = T.err_touch_m;          % CANONICAL touchdown error (AGL=0 interpolated)
zres = T.z_res_m;              % residual altitude at closest horizontal pass
main = err <= 100;             % main-population convention from truth32_README
fprintf('touchdown error    : median %.2f m, range %.2f-%.2f m\n', ...
        median(err), min(err), max(err));
fprintf('main population    : err<=100 m -> n=%d ; excluded: %s\n', ...
        sum(main), strjoin(cellstr(string(T.tag(~main))), ', '));

%% =====================================================================
%% SECTION 1 -- Ch.2 SIZING:  DEMAND AT RELEASE  vs  AUTHORITY AVAILABLE
%% =====================================================================
fprintf('\n');
fprintf('----------------------------------------------------------------\n');
fprintf(' 1. Ch.2  nc(0) = VM*HE*Np/tF   (Np=%.1f)   [lines 536-540]\n', p.NP);
fprintf('----------------------------------------------------------------\n');

r0   = T.r_rel_m;
z0   = T.z_rel_m;
V0   = T.V_rel_ms;
gam0 = deg2rad(T.gamma_rel_deg);
eps0 = deg2rad(T.eps_rel_deg);           % signed horizontal bearing offset
R0   = hypot(r0,z0);

% -------- heading error, in Zarchan's sense --------
% For a STATIONARY target the collision course is straight down the LOS, so
% HE is exactly the angle between the velocity vector and the LOS.  That is
% the one place our problem is CLEANER than the book's: no nT, no lead angle.
gam_los = -atan2(z0, max(r0,1e-9));      % LOS elevation, down-negative
HE_h = eps0;                             % horizontal channel (signed)
HE_v = gam0 - gam_los;                   % vertical channel, +ve = above LOS
% true 3-D angle between v and LOS, no small-angle assumption
cosHE = cos(gam0).*(r0.*cos(eps0) - z0.*tan(gam0))./R0;
HE_t  = acos(max(-1,min(1,cosHE)));

% -------- Ch.2 demand at t=0 --------
nc0_h = V0.*abs(HE_h)*p.NP./tF;
nc0_v = V0.*abs(HE_v)*p.NP./tF;
nc0_t = V0.*HE_t     *p.NP./tF;

% -------- our authority at release --------
a_h_max  = G*max(cos(gam0),0.25).*tan(deg2rad(80)*abs(cos(gam0)));
a_dn_max = (1 + cos(gam0))*G;            % nz = -1, pitch down
a_up_max = (3 - cos(gam0))*G;            % nz = +3, pull up
a_v_max  = a_dn_max;  a_v_max(HE_v < 0) = a_up_max(HE_v < 0);

rho_h = nc0_h ./ a_h_max;
rho_v = nc0_v ./ a_v_max;
rho_m = max(rho_h, rho_v);

fprintf('  heading error at release (3-D angle, velocity to LOS)\n');
fprintf('     total   median %6.2f deg   range %6.2f - %6.2f\n', ...
        rad2deg(median(HE_t)), rad2deg(min(HE_t)), rad2deg(max(HE_t)));
fprintf('     horiz   median %6.2f deg   range %6.2f - %6.2f   (|eps|)\n', ...
        rad2deg(median(abs(HE_h))), rad2deg(min(abs(HE_h))), rad2deg(max(abs(HE_h))));
fprintf('     vert    median %6.2f deg   range %6.2f - %6.2f   (gam - gam_LOS)\n', ...
        rad2deg(median(abs(HE_v))), rad2deg(min(abs(HE_v))), rad2deg(max(abs(HE_v))));
fprintf('     %d of %d released ABOVE the LOS (positive HE_v: must steepen)\n', ...
        sum(HE_v>0), n);
fprintf('  demand nc(0) [g]   lateral  median %.3f  max %.3f\n', ...
        median(nc0_h)/G, max(nc0_h)/G);
fprintf('                     vertical median %.3f  max %.3f\n', ...
        median(nc0_v)/G, max(nc0_v)/G);
fprintf('  authority    [g]   lateral  median %.3f  min %.3f\n', ...
        median(a_h_max)/G, min(a_h_max)/G);
fprintf('                     down     median %.3f  min %.3f   ((1+cos gam) g, nz=-1)\n', ...
        median(a_dn_max)/G, min(a_dn_max)/G);
fprintf('                     up       median %.3f  min %.3f   ((3-cos gam) g, nz=+3)\n', ...
        median(a_up_max)/G, min(a_up_max)/G);
fprintf('  RATIO demand/available   lateral  median %.4f  max %.4f\n', ...
        median(rho_h), max(rho_h));
fprintf('                           vertical median %.4f  max %.4f\n', ...
        median(rho_v), max(rho_v));
fprintf('                           worst-ch median %.4f  max %.4f\n', ...
        median(rho_m), max(rho_m));
fprintf('  aircraft with any channel ratio > 1 (Ch.2 predicts saturation at t=0): %d / %d\n', ...
        sum(rho_m>1), n);
fprintf('  worst channel is LATERAL on %d of %d aircraft (else vertical)\n', ...
        sum(rho_h>rho_v), n);

[ah_med_dive, ah_steep_dive, gam_med_dive] = deal(nan(n,1));

%% =====================================================================
%% SECTION 2 -- APPARENT GUIDANCE TIME CONSTANT, AND tF/tau
%% =====================================================================
fprintf('\n');
fprintf('----------------------------------------------------------------\n');
fprintf(' 2. apparent guidance time constant tau, and tF/tau  [line 5815]\n');
fprintf('----------------------------------------------------------------\n');
fprintf('  replaying tot_node.py::_pn_cmd 2836-2866 on the flown state, then\n');
fprintf('  fitting the ONE first-order lag Ch.3 assumes: tau*dgam/dt = gam_cmd - gam\n');

tau_all   = nan(n,1);   % fit over every dive sample
tau_nofl  = nan(n,1);   % fit scored only where the LOS floor is INACTIVE
rms_all   = nan(n,1);
rms_nofl  = nan(n,1);
lag_delay = nan(n,1);   % pure-delay cross-check
floorfrac = nan(n,1);
nzfrac    = nan(n,1);
nzlo      = nan(n,1);
nzhi      = nan(n,1);
nsamp     = nan(n,1);
% WHERE in the dive the demand actually bites.  Ch.2's heading-error profile
% is MONOTONICALLY DECREASING (peak at t=0, line 540: size for the beginning);
% Ch.2's target-maneuver profile is MONOTONICALLY INCREASING (lines 549-554).
% Our target does not maneuver, so the book says we are in the first case.
% These three columns test that directly.
clamp_tmed = nan(n,1);   % median normalised time of the nz-clamped samples
clamp_first= nan(n,1);   % clamp fraction inside the FIRST third of the dive
clamp_last = nan(n,1);   % clamp fraction inside the LAST  third of the dive
floor_first= nan(n,1);
floor_last = nan(n,1);

pn = struct('NVERT',p.NVERT,'PNLEAD',p.PNLEAD,'NZMIN',p.NZMIN, ...
            'NZMAX',p.NZMAX,'LOSFLOOR',p.LOSFLOOR,'G',G,'DIFFWIN',p.DIFFWIN);

for i = 1:n
    f = fullfile(root,'data','traj',[strrep(char(T.tag(i)),'-','_') '.csv']);
    if ~isfile(f), warning('missing %s',f); continue; end
    J = readtable(f);
    keep = J.post_touch == 0;
    t = J.t_dq(keep); r = J.r(keep); z = J.z(keep);
    gam = deg2rad(J.gamma_deg(keep)); gs = J.gs(keep);
    if numel(t) < 30, continue; end
    [gcmd, onfloor, lo, hi] = replay_pn(t,r,z,gam,gs,pn);
    nsamp(i)     = numel(t);
    floorfrac(i) = mean(onfloor);
    nzlo(i)      = mean(lo);  nzhi(i) = mean(hi);
    nzfrac(i)    = mean(lo|hi);
    tn_ = (t - t(1)) / max(t(end)-t(1), eps);      % normalised dive time 0..1
    cl = lo | hi;
    if any(cl), clamp_tmed(i) = median(tn_(cl)); end
    clamp_first(i) = mean(cl(tn_ <= 1/3));
    clamp_last(i)  = mean(cl(tn_ >= 2/3));
    floor_first(i) = mean(onfloor(tn_ <= 1/3));
    floor_last(i)  = mean(onfloor(tn_ >= 2/3));
    [tau_all(i),  rms_all(i)]  = fit_lag(t,gcmd,gam,true(size(t)),p.TAUMAX);
    m2 = ~onfloor;
    if sum(m2) >= 20
        [tau_nofl(i), rms_nofl(i)] = fit_lag(t,gcmd,gam,m2,p.TAUMAX);
    end
    lag_delay(i) = fit_delay(t,gcmd,gam,p.TAUMAX);
    ah = G*max(cos(gam),0.25).*tan(deg2rad(80)*abs(cos(gam)));
    ah_med_dive(i)  = median(ah)/G;
    gam_med_dive(i) = rad2deg(median(gam));
    q = quantile_local(gam, 0.10);            % steepest decile of gamma
    ah_steep_dive(i) = median(ah(gam<=q))/G;
end

fprintf('\n  REPLAY SELF-CHECKS (these validate the transcription, not the theory)\n');
dnz = nzfrac - T.nz_clip_frac;
fprintf('    nz clamp fraction, mine vs truth32.nz_clip_frac: max |diff| %.4f, median |diff| %.4f\n', ...
        max(abs(dnz)), median(abs(dnz)));
fprintf('    spearman(mine, truth32 column) = %+.3f\n', spear(nzfrac, T.nz_clip_frac, 4000));
fprintf('    ** FRAGILITY WARNING.  Two faithful transcriptions of the SAME equation, **\n');
fprintf('    ** agreeing to a median 0.6 percentage points, do NOT agree on the rank   **\n');
fprintf('    ** order.  Section 4 prints both against err: the truth32 column scores    **\n');
fprintf('    ** +0.935 and this replay +0.661 on the same 30 aircraft.  The clamp is a  **\n');
fprintf('    ** THRESHOLD CROSSING, so a per-sample difference far below the noise      **\n');
fprintf('    ** floor flips individual samples and reshuffles a fraction whose median   **\n');
fprintf('    ** is only 2.5 %%.  Do not quote +0.935 as if it were a stable constant.    **\n');
fprintf('    LOS floor active on %.1f %% of all PN samples pooled   [task brief: 43.1 %%]\n', ...
        100*sum(floorfrac.*nsamp)/sum(nsamp));
fprintf('    LOS floor per aircraft: median %.1f %%, range %.1f - %.1f %%\n', ...
        100*median(floorfrac), 100*min(floorfrac), 100*max(floorfrac));
fprintf('    lateral authority over the dive: median %.2f g, steepest decile %.2f g\n', ...
        median(ah_med_dive), median(ah_steep_dive));
fprintf('                                     [task brief: median 2.03 g, steepest 1.49 g]\n');
fprintf('    median gamma over the dive: %.1f deg\n', median(gam_med_dive));

fprintf('\n  WHERE IN THE FLIGHT DOES THE DEMAND ACTUALLY BITE?\n');
fprintf('    Ch.2 line 540 says size for the BEGINNING: the heading-error profile\n');
fprintf('    nc(t) = nc(0)*(1-t/tF)^(Np-2) is monotonically DECREASING, and with a\n');
fprintf('    non-maneuvering target that is the only profile the book offers us.\n');
fprintf('    measured: median normalised time of the nz-clamped samples = %.2f\n', ...
        median(clamp_tmed,'omitnan'));
fprintf('    nz clamp fraction   first third %.2f %%   last third %.2f %%\n', ...
        100*median(clamp_first), 100*median(clamp_last));
fprintf('                        aircraft with ZERO clamping in the first third: %d / %d\n', ...
        sum(clamp_first==0), n);
fprintf('    LOS floor fraction  first third %.2f %%   last third %.2f %%\n', ...
        100*median(floor_first), 100*median(floor_last));
if median(clamp_last) > median(clamp_first)
    fprintf('    => the demand peaks at the END.  Ch.2''s sizing advice points at the\n');
    fprintf('       WRONG END of our flight; the profile we actually fly is shaped like\n');
    fprintf('       the book''s TARGET-MANEUVER case (lines 549-554, monotonically\n');
    fprintf('       increasing) even though our target is stationary.\n');
else
    fprintf('    => the demand peaks at the START, as Ch.2 assumes.\n');
end

fprintf('\n  FITTED TIME CONSTANT\n');
fprintf('    tau, all samples          median %.2f s   range %.2f - %.2f s\n', ...
        median(tau_all,'omitnan'), min(tau_all), max(tau_all));
fprintf('    tau, floor-inactive only  median %.2f s   range %.2f - %.2f s   [brief: 1.50 s]\n', ...
        median(tau_nofl,'omitnan'), min(tau_nofl), max(tau_nofl));
fprintf('    pure-delay fit            median %.2f s   range %.2f - %.2f s\n', ...
        median(lag_delay,'omitnan'), min(lag_delay), max(lag_delay));
fprintf('    at the %.0f s search ceiling: %d aircraft (tau), %d (delay)\n', ...
        p.TAUMAX, sum(tau_all>=p.TAUMAX-1e-3), sum(lag_delay>=p.TAUMAX-1e-3));

ratio_all  = tF./tau_all;
ratio_nofl = tF./tau_nofl;
fprintf('\n    tF/tau, all samples       median %.1f   range %.1f - %.1f\n', ...
        median(ratio_all,'omitnan'), min(ratio_all), max(ratio_all));
fprintf('    tF/tau, floor-inactive    median %.1f   range %.1f - %.1f\n', ...
        median(ratio_nofl,'omitnan'), min(ratio_nofl), max(ratio_nofl));

fprintf('\n  CONFUSION MATRICES -- the book threshold used as a FAILURE PREDICTOR\n');
fprintf('    book rule: tF/T < 8 => "a matter of luck"; tF/T < 3 => miss not negligible\n');
CM = struct();
defs = {  'err > 10 m  (hit_10m == 0, the acceptance criterion)', err>10; ...
          'err > 30 m  (the four worst, hard fact #2)',           err>30 };
% The last three are NOT Zarchan's; they are here so the book's criterion is
% scored against something, instead of against nothing.
preds = { 'tF/tau_all  < 8',  ratio_all  < 8; ...
          'tF/tau_all  < 3',  ratio_all  < 3; ...
          'tF/tau_nofl < 8',  ratio_nofl < 8; ...
          'tF/tau_nofl < 3',  ratio_nofl < 3; ...
          '[ours] floor > 85%', floorfrac > 0.85; ...
          '[ours] nzclip > 10%', nzfrac  > 0.10; ...
          '[ours] Ch2 ratio>0.6', rho_m  > 0.60 };
for a = 1:size(defs,1)
    act = defs{a,2};
    fprintf('\n    ACTUAL FAILURE = %s   (%d of %d)\n', defs{a,1}, sum(act), n);
    fprintf('      %-20s  TP  FP  FN  TN | precision recall     MCC\n','predictor');
    for b = 1:size(preds,1)
        prd = preds{b,2};
        ok  = isfinite(prd) & isfinite(act);
        tp = sum(prd&act&ok); fp = sum(prd&~act&ok);
        fn = sum(~prd&act&ok); tn = sum(~prd&~act&ok);
        prec = tp/max(tp+fp,eps); rec = tp/max(tp+fn,eps);
        mcc  = (tp*tn - fp*fn)/max(sqrt((tp+fp)*(tp+fn)*(tn+fp)*(tn+fn)),eps);
        fprintf('      %-20s %3d %3d %3d %3d |    %6.3f  %6.3f  %+.3f\n', ...
                preds{b,1}, tp,fp,fn,tn, prec, rec, mcc);
        CM.(sprintf('a%d_b%d',a,b)) = [tp fp; fn tn];
    end
end

%% =====================================================================
%% SECTION 3 -- SATURATION BUDGET, AND WHETHER THE BUDGET IS EVEN VALID
%% =====================================================================
fprintf('\n');
fprintf('----------------------------------------------------------------\n');
fprintf(' 3. saturation budget:  measured miss  minus  linear-theory miss\n');
fprintf('----------------------------------------------------------------\n');

CF = closedform('XNP',p.NP);       % missfun_he(tf,T,Np) = Miss/(VM*HE), SI in -> m out
miss_ch3_all  = nan(n,1);
miss_ch3_nofl = nan(n,1);
for i = 1:n
    if isfinite(tau_all(i))
        miss_ch3_all(i)  = abs(V0(i)*HE_t(i)*CF.missfun_he(tF(i), tau_all(i),  p.NP));
    end
    if isfinite(tau_nofl(i))
        miss_ch3_nofl(i) = abs(V0(i)*HE_t(i)*CF.missfun_he(tF(i), tau_nofl(i), p.NP));
    end
end
res_ch3 = err - miss_ch3_all;

fprintf('  Ch.2 (no lag, no saturation)  predicted miss = 0 for all %d   [line 539 verbatim]\n', n);
fprintf('  Ch.3 (one lag T = fitted tau) predicted miss  median %.3e m   max %.3e m\n', ...
        median(miss_ch3_all,'omitnan'), max(miss_ch3_all));
fprintf('       using the floor-inactive tau             median %.3e m   max %.3e m\n', ...
        median(miss_ch3_nofl,'omitnan'), max(miss_ch3_nofl));
fprintf('  MEASURED miss                                 median %.3f  m   max %.3f  m\n', ...
        median(err), max(err));
fprintf('  fraction of the measured miss that the Ch.3 lag explains:\n');
fprintf('       median %.3e   max %.3e   (1.0 would mean the lag explains all of it)\n', ...
        median(miss_ch3_all./max(err,eps),'omitnan'), max(miss_ch3_all./max(err,eps)));
fprintf('  residual (measured - Ch.3): median %.3f m  =  %.4f %% of the measured miss\n', ...
        median(res_ch3,'omitnan'), 100*median(res_ch3./max(err,eps),'omitnan'));

fprintf('\n  ---- PREMISE AUDIT: is this decomposition allowed at all? ----\n');
smallang = rad2deg(HE_t);
linerr = abs(sin(HE_t)-HE_t)./max(HE_t,eps);
a1 = median(smallang) <= 10;
fprintf('   A1 SMALL ANGLES.   HE median %.1f deg (max %.1f).  |sin HE - HE|/HE:\n', ...
        median(smallang), max(smallang));
fprintf('                      median %.1f %%, max %.1f %%.  aircraft with HE > 15 deg: %d/%d\n', ...
        100*median(linerr), 100*max(linerr), sum(smallang>15), n);
fprintf('                      VERDICT: %s\n', tf2s(a1,'holds','VIOLATED'));
fprintf('   A2 EXACTLY ONE FIRST-ORDER LAG.  residual of the best single-lag fit:\n');
fprintf('                      median %.2f deg rms (floor-inactive %.2f deg rms)\n', ...
        rad2deg(median(rms_all,'omitnan')), rad2deg(median(rms_nofl,'omitnan')));
fprintf('                      our loop = alpha lowpass 1.75 s + pn_lead 1.0 s + the FC\n');
fprintf('                      attitude loop + 20 Hz command rate.  That is >= 3 lags, not 1.\n');
fprintf('                      VERDICT: VIOLATED by construction.\n');
a3 = max(nzfrac) < 0.01;
fprintf('   A3 NO SATURATION.  nz clamp fraction median %.2f %%, max %.2f %%; zero on %d/%d\n', ...
        100*median(nzfrac), 100*max(nzfrac), sum(nzfrac==0), n);
fprintf('                      LOS floor active on %.1f %% of pooled samples (NO book counterpart)\n', ...
        100*sum(floorfrac.*nsamp)/sum(nsamp));
fprintf('                      VERDICT: %s\n', tf2s(a3,'holds','VIOLATED'));
vratio = T.V_touch_ms./V0;
a4 = max(abs(vratio-1)) < 0.10;
fprintf('   A4 CONSTANT CLOSING VELOCITY.  V_touch/V_release median %.2f, range %.2f-%.2f\n', ...
        median(vratio), min(vratio), max(vratio));
fprintf('                      VERDICT: %s\n', tf2s(a4,'holds','VIOLATED'));

fprintf('\n   PREMISES VIOLATED: %d of 4.\n', double(~a1) + 1 + double(~a3) + double(~a4));

%% =====================================================================
%% SECTION 4 -- PREDICTIVE POWER.  ALL OF IT, INCLUDING THE NEGATIVES.
%% =====================================================================
fprintf('\n');
fprintf('----------------------------------------------------------------\n');
fprintf(' 4. predictive power (Spearman, tied ranks, permutation p, seed %d)\n', p.SEED);
fprintf('----------------------------------------------------------------\n');
rng(p.SEED,'twister');

cands = { ...
  'Ch2 demand/avail  worst channel', rho_m; ...
  'Ch2 demand/avail  vertical',      rho_v; ...
  'Ch2 demand/avail  lateral',       rho_h; ...
  'Ch2 demand nc(0)  total [g]',     nc0_t/G; ...
  'Ch2 demand nc(0)  vertical [g]',  nc0_v/G; ...
  'heading error HE  total [deg]',   rad2deg(HE_t); ...
  'heading error HE  vertical [deg]',rad2deg(HE_v); ...
  'heading error HE  |horizontal|',  rad2deg(abs(HE_h)); ...
  'Ch3 predicted miss [m]',          miss_ch3_all; ...
  'tF/tau all      [INVERSE exp]',   ratio_all; ...
  'tF/tau nofloor  [INVERSE exp]',   ratio_nofl; ...
  'tau fitted [s]',                  tau_all; ...
  'LOS-floor active fraction',       floorfrac; ...
  'nz clamp fraction (ours)',        nzfrac; ...
  'nz clamp frac (truth32 column)',  T.nz_clip_frac; ...
  'nz clamp LOW only',               nzlo; ...
  'nz clamp, last third of dive',    clamp_last; ...
  'residual altitude z_res [m]',     zres };

fprintf('\n  vs err_touch_m,   n=32 (all)        |  main population err<=100 m, n=%d\n', sum(main));
fprintf('  %-33s  rho       p    |  rho       p\n','predictor');
POW = struct('name',{},'rho32',{},'p32',{},'rho30',{},'p30',{});
for k = 1:size(cands,1)
    x = cands{k,2};
    [r32,pp32] = spear(x, err, p.NPERM);
    [r30,pp30] = spear(x(main), err(main), p.NPERM);
    fprintf('  %-33s %+.3f  %6.4f  | %+.3f  %6.4f\n', cands{k,1}, r32,pp32,r30,pp30);
    POW(end+1) = struct('name',cands{k,1},'rho32',r32,'p32',pp32,'rho30',r30,'p30',pp30); %#ok<AGROW>
end

fprintf('\n  DOES THE Ch.2 SIZING RULE PREDICT THE SATURATION IT EXISTS TO PREVENT?\n');
[rs1,ps1] = spear(rho_m, nzfrac, p.NPERM);
[rs2,ps2] = spear(rho_v, nzfrac, p.NPERM);
[rs3,ps3] = spear(rho_v, nzlo,  p.NPERM);
fprintf('    spearman(demand/avail worst-ch , nz clamp fraction) = %+.3f  p=%.4f\n', rs1, ps1);
fprintf('    spearman(demand/avail vertical , nz clamp fraction) = %+.3f  p=%.4f\n', rs2, ps2);
fprintf('    spearman(demand/avail vertical , nz clamp LOW only) = %+.3f  p=%.4f\n', rs3, ps3);

fprintf('\n  HEAD-TO-HEAD on the main population (n=%d): linear R^2 of err on one regressor\n', sum(main));
r2a = r2lin(rho_m(main),        err(main));
r2b = r2lin(miss_ch3_all(main), err(main));
r2c = r2lin(zres(main),         err(main));
r2d = r2lin(nzfrac(main),       err(main));
fprintf('    Zarchan Ch.2 demand/available   R^2 = %.3f\n', r2a);
fprintf('    Zarchan Ch.3 predicted miss     R^2 = %.3f\n', r2b);
fprintf('    residual altitude z_res         R^2 = %.3f   [memory: 0.906]\n', r2c);
fprintf('    nz clamp fraction               R^2 = %.3f\n', r2d);
bz = [ones(sum(main),1) zres(main)] \ err(main);
fprintf('    err = %.3f * z_res + %.2f\n', bz(2), bz(1));

% ---- LEVERAGE CHECK.  An R^2 of 0.93 next to a Spearman of ~0.2 means the
%      fit is being carried by a few extreme points, not by the bulk.  Say so.
fprintf('\n  LEVERAGE CHECK -- drop the four largest errors inside the main population\n');
em = err(main); zm = zres(main); rm = rho_m(main); cm = miss_ch3_all(main);
[~,o2] = sort(em,'descend'); drop = o2(1:4); kp = true(numel(em),1); kp(drop) = false;
tagm = string(T.tag(main));
fprintf('    dropped: %s\n', strjoin(cellstr(tagm(drop)), ', '));
fprintf('    z_res            R^2 %.3f -> %.3f   spearman %+.3f -> %+.3f\n', ...
        r2lin(zm,em), r2lin(zm(kp),em(kp)), ...
        spear(zm,em,2000), spear(zm(kp),em(kp),2000));
fprintf('    Ch.2 ratio       R^2 %.3f -> %.3f   spearman %+.3f -> %+.3f\n', ...
        r2lin(rm,em), r2lin(rm(kp),em(kp)), ...
        spear(rm,em,2000), spear(rm(kp),em(kp),2000));
fprintf('    Ch.3 pred miss   R^2 %.3f -> %.3f   spearman %+.3f -> %+.3f\n', ...
        r2lin(cm,em), r2lin(cm(kp),em(kp)), ...
        spear(cm,em,2000), spear(cm(kp),em(kp),2000));
fprintf('    (that drop selects on the OUTCOME, so it can only raise a suspicion.\n');
fprintf('     The split below selects on a PREDICTOR instead, and is the real test.)\n');

%% ---- PREDICTOR-SIDE SPLIT.  No outcome peeking. ------------------------
%  Hard fact #1 states the four failed aircraft sat on the LOS floor for
%  89.8-99.7 % of their PN samples.  Split on floor >= 0.85 -- a threshold
%  taken from that statement, not from the errors -- and score both theories
%  inside each half.  If Zarchan's Ch.2 sizing is a real predictor for the
%  aircraft that are ACTUALLY DOING PN, it has to show up here.
fprintf('\n  PREDICTOR-SIDE SPLIT: LOS floor active >= 85 %% of samples, or not\n');
hi_fl = floorfrac >= 0.85;
for s = 0:1
    m = main & (hi_fl == logical(s));
    lbl = tf2s(s==1, 'floor >= 85%  (PN mostly overridden)', 'floor <  85%  (PN mostly live)');
    fprintf('\n    %s   n=%d   err median %.2f m, max %.2f m\n', ...
            lbl, sum(m), median(err(m)), max(err(m)));
    if sum(m) < 4
        fprintf('      too few for a fit\n'); continue;
    end
    sub = { 'Zarchan Ch.2 demand/available', rho_m(m); ...
            'Zarchan Ch.3 predicted miss  ', miss_ch3_all(m); ...
            'residual altitude z_res      ', zres(m); ...
            'nz clamp fraction (replay)   ', nzfrac(m); ...
            'tF/tau (floor-inactive)      ', ratio_nofl(m) };
    fprintf('      %-30s   R^2     rho       p\n','predictor');
    for q = 1:size(sub,1)
        [rq,pq] = spear(sub{q,2}, err(m), 20000);
        fprintf('      %-30s %6.3f  %+.3f  %6.4f\n', ...
                sub{q,1}, r2lin(sub{q,2},err(m)), rq, pq);
    end
end

%% ---------------- assemble output ----------------
A = struct();
A.opts = p;
A.tF_clock = clockname;
A.per = table(string(T.tag), tF, err, zres, T.hit_10m, ...
    rad2deg(HE_t), rad2deg(HE_h), rad2deg(HE_v), V0, ...
    nc0_h/G, nc0_v/G, a_h_max/G, a_v_max/G, rho_h, rho_v, rho_m, ...
    tau_all, tau_nofl, lag_delay, ratio_all, ratio_nofl, ...
    floorfrac, nzfrac, T.nz_clip_frac, clamp_first, clamp_last, clamp_tmed, ...
    miss_ch3_all, res_ch3, ...
    'VariableNames', {'tag','tF_s','err_m','z_res_m','hit10', ...
      'HE_tot_deg','HE_h_deg','HE_v_deg','V_rel_ms', ...
      'nc0_h_g','nc0_v_g','a_h_max_g','a_v_max_g','rho_h','rho_v','rho_max', ...
      'tau_all_s','tau_nofloor_s','delay_s','tF_over_tau','tF_over_tau_nofl', ...
      'los_floor_frac','nz_frac_mine','nz_frac_truth', ...
      'nz_first_third','nz_last_third','nz_clamp_tmed','miss_ch3_m','resid_m'});
A.power = POW;
A.CM = CM;
A.r2 = struct('ch2_ratio',r2a,'ch3_miss',r2b,'z_res',r2c,'nz_frac',r2d);
A.split = struct('hi_floor',hi_fl, ...
    'ch2_R2_lo', r2lin(rho_m(main&~hi_fl),err(main&~hi_fl)), ...
    'ch2_R2_hi', r2lin(rho_m(main& hi_fl),err(main& hi_fl)), ...
    'zres_R2_lo',r2lin(zres(main&~hi_fl),err(main&~hi_fl)), ...
    'zres_R2_hi',r2lin(zres(main& hi_fl),err(main& hi_fl)));

if p.WRITECSV
    outf = fullfile(here,'apply_stot32_per_aircraft.csv');
    writetable(A.per, outf);
    fprintf('\n  wrote %s\n', outf);
end

[~,ord] = sort(err,'descend');
fprintf('\n  THE SIX WORST AIRCRAFT  [hard fact #2: four at err 37-46 m, tF/tau 15.1/7.6/5.5/4.7]\n');
fprintf('  %-10s %8s %8s %8s %9s %9s %8s %9s\n', ...
        'tag','err_m','z_res','tau_s','tF/tau','delay_s','floor%','nzclip%');
for k = 1:6
    i = ord(k);
    fprintf('  %-10s %8.2f %8.2f %8.2f %9.1f %9.2f %8.1f %9.2f\n', ...
        char(T.tag(i)), err(i), zres(i), tau_all(i), ratio_all(i), lag_delay(i), ...
        100*floorfrac(i), 100*nzfrac(i));
end
fprintf('\n================================================================\n\n');
end

%% =====================================================================
%% helpers
%% =====================================================================
function [gcmd, onfloor, lo, hi] = replay_pn(t,r,z,gam,gs,pn)
%REPLAY_PN  tot_node.py::_pn_cmd 2836-2866, longitudinal channel, verbatim.
%   sig_dot = (hdot*d - h*ddot)/R^2 ;  ddot = d(r)/dt
%   gdot    = -n_vert*sig_dot
%   nz      = cos gam + V*gdot/g , clipped to [NZMIN,NZMAX]
%   gdot    = (nz - cos gam)*g/V
%   gam_cmd = gam + gdot*pn_lead
%   floor   : gam_cmd = min(gam_cmd, -atan2(h,d)*LOSFLOOR)
d = r; h = z; vh = gs;
hdot = vh.*tan(gam);
V = hypot(vh,hdot);
ddot = cdiff(t, r, pn.DIFFWIN);          % d(r)/dt on the de-quantised clock
R2 = d.^2 + h.^2;
sig_dot = (hdot.*d - h.*ddot)./max(R2,1e-9);
gdot = -pn.NVERT*sig_dot;
nz_raw = cos(gam) + V.*gdot/pn.G;
lo = nz_raw < pn.NZMIN;
hi = nz_raw > pn.NZMAX;
nz = max(pn.NZMIN, min(pn.NZMAX, nz_raw));
gdot = (nz - cos(gam))*pn.G./max(V,1e-6);
gcmd = gam + gdot*pn.PNLEAD;
gam_los = -atan2(h, max(d,1e-3));
flo = gam_los*pn.LOSFLOOR;
onfloor = flo < gcmd;                     % the min() actually bites
gcmd = min(gcmd, flo);
end

function dy = cdiff(t,y,win)
%CDIFF  centred difference from the two window edges only, so it cannot
%   invent curvature.  t must be the DE-QUANTISED clock (t_dq): the raw t is
%   quantised to 0.1 s and about half the adjacent rows share a stamp.
m = numel(t); dy = zeros(m,1);
for k = 1:m
    a = find(t >= t(k)-win, 1, 'first');
    b = find(t <= t(k)+win, 1, 'last');
    if isempty(a), a = 1; end
    if isempty(b), b = m; end
    if b <= a, a = max(1,k-1); b = min(m,k+1); end
    dt = t(b)-t(a);
    if dt <= 0, dy(k) = 0; else, dy(k) = (y(b)-y(a))/dt; end
end
end

function [tau, e] = fit_lag(t,u,y,mask,taumax)
%FIT_LAG  best single first-order lag tau*dy/dt = u - y, exact ZOH stepping.
grid = logspace(log10(0.10), log10(taumax), 240);
e = inf; tau = NaN;
for k = 1:numel(grid)
    ek = lagres(t,u,y,grid(k),mask);
    if ek < e, e = ek; tau = grid(k); end
end
lo = max(0.05, tau/1.5); hi = min(taumax, tau*1.5);
gr = (sqrt(5)-1)/2;
c = hi - gr*(hi-lo); dd = lo + gr*(hi-lo);
fc = lagres(t,u,y,c,mask); fd = lagres(t,u,y,dd,mask);
for it = 1:60
    if fc < fd
        hi = dd; dd = c; fd = fc; c = hi - gr*(hi-lo); fc = lagres(t,u,y,c,mask);
    else
        lo = c;  c = dd; fc = fd; dd = lo + gr*(hi-lo); fd = lagres(t,u,y,dd,mask);
    end
    if hi-lo < 1e-4, break; end
end
tau = (lo+hi)/2; e = lagres(t,u,y,tau,mask);
end

function e = lagres(t,u,y,tau,mask)
m = numel(t); yh = zeros(m,1); yh(1) = y(1);
for k = 1:m-1
    dt = t(k+1)-t(k);
    if dt <= 0, yh(k+1) = yh(k); continue; end
    a = exp(-dt/tau);
    yh(k+1) = u(k) + (yh(k)-u(k))*a;
end
rr = yh - y; rr = rr(mask);
if isempty(rr), e = inf; else, e = sqrt(mean(rr.^2)); end
end

function L = fit_delay(t,u,y,lmax)
%FIT_DELAY  best pure time shift of u onto y, a cross-check on fit_lag.
%   t_dq is meant to be strictly increasing but is not always (the
%   de-quantisation only adds 0.05*k within a stamp group, so a group of
%   more than two can collide with the next stamp).  Deduplicate first --
%   interp1 refuses non-unique sample points.
[tu, iu] = unique(t(:), 'stable');
if numel(tu) < numel(t)
    [tu, ord] = sort(tu); iu = iu(ord);
else
    tu = t(:); iu = (1:numel(t))';
end
uu = u(iu);
grid = 0:0.05:lmax;
best = inf; L = NaN;
for k = 1:numel(grid)
    us = interp1(tu+grid(k), uu, t, 'linear', NaN);
    m = isfinite(us);
    if sum(m) < 20, continue; end
    e = sqrt(mean((us(m)-y(m)).^2));
    if e < best, best = e; L = grid(k); end
end
end

function [rho, pv] = spear(x,y,nperm)
x = x(:); y = y(:);
ok = isfinite(x) & isfinite(y);
x = x(ok); y = y(ok); m = numel(x);
if m < 4, rho = NaN; pv = NaN; return; end
rx = tiedrank_local(x); ry = tiedrank_local(y);
rho = pear(rx,ry);
cnt = 0;
for k = 1:nperm
    if abs(pear(rx(randperm(m)),ry)) >= abs(rho)-1e-12, cnt = cnt+1; end
end
pv = (cnt+1)/(nperm+1);
end

function c = pear(a,b)
a = a(:)-mean(a); b = b(:)-mean(b);
den = sqrt((a'*a)*(b'*b));
if den <= 0, c = NaN; else, c = (a'*b)/den; end
end

function r = tiedrank_local(x)
m = numel(x); [xs,idx] = sort(x(:)); r = zeros(m,1); r(idx) = 1:m;
i = 1;
while i <= m
    j = i;
    while j < m && xs(j+1) == xs(i), j = j+1; end
    if j > i, r(idx(i:j)) = mean(i:j); end
    i = j+1;
end
end

function v = r2lin(x,y)
ok = isfinite(x)&isfinite(y); x = x(ok); y = y(ok);
if numel(x) < 3, v = NaN; return; end
b = [ones(numel(x),1) x(:)] \ y(:);
yh = b(1) + b(2)*x(:);
ss = sum((y(:)-mean(y)).^2);
if ss <= 0, v = NaN; else, v = 1 - sum((y(:)-yh).^2)/ss; end
end

function q = quantile_local(x,pq)
xs = sort(x(:)); m = numel(xs);
if m == 0, q = NaN; return; end
h = (m-1)*pq + 1; lo = floor(h); hi = ceil(h);
q = xs(lo) + (h-lo)*(xs(hi)-xs(lo));
end

function s = tf2s(b,a,c)
if b, s = a; else, s = c; end
end
