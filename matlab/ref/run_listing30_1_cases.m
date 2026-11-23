function R = run_listing30_1_cases()
%RUN_LISTING30_1_CASES  Reproduce the book cases of Zarchan Listing 30.1 and
%   check them against the qualitative claims made in the surrounding prose
%   (Figs 30.2 - 30.5, pp. 783-784).
%
%   Claims under test
%     C1  the two ICHOICE trajectories are "nearly identical"
%     C2  the commanded acceleration profiles are "vastly different", with the
%         perpendicular-to-LOS maximum "much smaller" and its flight time
%         "much longer"
%     C3  perpendicular-to-LOS shows "severe missile velocity loss"
%         (Fig 30.5 y-axis 250 -> 0 m/s); perpendicular-to-V "no velocity loss
%         at all"
%     C4  both implementations drive the flight path angle to -90 deg
%
%   Run:  matlab -batch "cd matlab/ref; R=run_listing30_1_cases"

here = fileparts(mfilename('fullpath'));
addpath(here);
figdir = fullfile(here,'figs');
if ~exist(figdir,'dir'), mkdir(figdir); end

cases = { ...
  'A  OPT  perp-LOS   (book nominal, Fig 30.2/30.3 solid)', 0, 0, 'listing'; ...
  'B  OPT  perp-V     (Fig 30.2/30.3 dashed)',              0, 1, 'listing'; ...
  'C  BPN  perp-V     (listing bias, +5 deg/s)',            1, 1, 'listing'; ...
  'D  BPN  perp-V     (prose bias, erratum, -7 deg/s)',     1, 1, 'text';    ...
  'E  BPN  perp-LOS   (listing bias)',                      1, 0, 'listing'};

R = struct('name',{},'IGUID',{},'ICHOICE',{},'out',{});
for i = 1:size(cases,1)
    t0 = tic;
    o = zarchan_listing30_1('IGUID',cases{i,2},'ICHOICE',cases{i,3}, ...
                            'BIASFORM',cases{i,4});
    R(i).name    = cases{i,1};
    R(i).IGUID   = cases{i,2};
    R(i).ICHOICE = cases{i,3};
    R(i).out     = o;
    R(i).wall    = toc(t0);
end

fprintf('\n================ LISTING 30.1 -- CASE TABLE ================\n');
fprintf('%-52s %8s %9s %9s %9s %9s %9s %9s %10s\n', 'case', 'tf_s', ...
        'miss_m', 'gamF_deg', 'Vf_m/s', 'Vmin_m/s', 'apo_km', 'ncmax_g', 'X_perf');
for i = 1:numel(R)
    o = R(i).out;
    fprintf('%-52s %8.3f %9.3f %9.2f %9.1f %9.1f %9.3f %9.2f %10.3e\n', ...
        R(i).name, o.tf, o.miss_m, o.gam_f_deg, o.vm_f_ms, o.vm_min_ms, ...
        o.apogee_km, o.xnc_max_g, o.x_perf);
end
fprintf('BIAS (listing form) = %.4f deg/s ; BIAS (prose form) = %.4f deg/s\n', ...
        R(3).out.BIASDEG, R(4).out.BIASDEG);

A = R(1).out; B = R(2).out;

%% ---- C1  trajectory similarity, A vs B --------------------------------
xg = linspace(0, min(max(A.RM1K),max(B.RM1K)), 400);
ha = interp1(A.RM1K, A.RM2K, xg, 'linear');
hb = interp1(B.RM1K, B.RM2K, xg, 'linear');
d  = ha - hb;
fprintf('\n-- C1 trajectory similarity (altitude vs downrange, A vs B) --\n');
fprintf('   apogee   A = %.4f km   B = %.4f km   (diff %.4f km)\n', ...
        A.apogee_km, B.apogee_km, A.apogee_km-B.apogee_km);
fprintf('   |dh| max = %.1f m   rms = %.1f m   (%.2f%% of A apogee)\n', ...
        1000*max(abs(d)), 1000*rms_(d), 100*max(abs(d))/A.apogee_km);
% the terminal plunge is vertical, so altitude-vs-downrange is ill-conditioned
% there; judge similarity over the loft/cruise portion, downrange 0..9 km
m = xg <= 9;
fprintf('   over downrange 0-9 km: |dh| max = %.1f m   rms = %.1f m\n', ...
        1000*max(abs(d(m))), 1000*rms_(d(m)));
% "nearly identical" is a claim about how Fig 30.2 LOOKS.  Scored on apogee
% agreement and rms separation; the max is dominated by the single instant
% where the two plunges begin, so it is reported but not scored.
verdict('C1 nearly identical (apogee <10%, rms |dh| <10% of apogee)', ...
        abs(A.apogee_km-B.apogee_km)/A.apogee_km < 0.10 && ...
        rms_(d(m))/A.apogee_km < 0.10);
fprintf('       apogee disagreement %.2f%% ; rms |dh| %.2f%% of apogee ; max |dh| %.2f%%\n', ...
        100*abs(A.apogee_km-B.apogee_km)/A.apogee_km, ...
        100*rms_(d(m))/A.apogee_km, 100*max(abs(d(m)))/A.apogee_km);

%% ---- C2  acceleration profiles ----------------------------------------
fprintf('\n-- C2 commanded acceleration --\n');
fprintf('   perp-LOS  max|nc| = %6.2f g   tf = %7.3f s\n', A.xnc_max_g, A.tf);
fprintf('   perp-V    max|nc| = %6.2f g   tf = %7.3f s\n', B.xnc_max_g, B.tf);
fprintf('   ratio max|nc| (V/LOS) = %.2f   ;  tf ratio (LOS/V) = %.2f\n', ...
        B.xnc_max_g/A.xnc_max_g, A.tf/B.tf);
verdict('C2a perp-LOS peak nc is much smaller', B.xnc_max_g > 1.5*A.xnc_max_g);
verdict('C2b perp-LOS flight time is much longer', A.tf > 1.15*B.tf);

%% ---- C3  velocity ------------------------------------------------------
fprintf('\n-- C3 velocity (Fig 30.5) --\n');
fprintf('   perp-LOS  V: %.1f -> %.1f m/s  (min %.1f, loss %.1f%%)\n', ...
        A.vm_ic_ms, A.vm_f_ms, A.vm_min_ms, 100*(A.vm_ic_ms-A.vm_f_ms)/A.vm_ic_ms);
fprintf('   perp-V    V: %.1f -> %.1f m/s  (min %.1f, loss %.3f%%)\n', ...
        B.vm_ic_ms, B.vm_f_ms, B.vm_min_ms, 100*(B.vm_ic_ms-B.vm_f_ms)/B.vm_ic_ms);
verdict('C3a perp-LOS severe velocity loss (>50%)', ...
        (A.vm_ic_ms-A.vm_f_ms)/A.vm_ic_ms > 0.5);
verdict('C3b perp-V no velocity loss (<0.1%)', ...
        abs(B.vm_ic_ms-B.vm_f_ms)/B.vm_ic_ms < 1e-3);

%% ---- C4  terminal flight path angle ------------------------------------
fprintf('\n-- C4 terminal flight path angle --\n');
fprintf('   perp-LOS gamF = %+8.2f deg   perp-V gamF = %+8.2f deg\n', ...
        A.gam_f_deg, B.gam_f_deg);
verdict('C4 both approach -90 deg (within 10 deg)', ...
        abs(A.gam_f_deg+90) < 10 && abs(B.gam_f_deg+90) < 10);

%% ---- figures -----------------------------------------------------------
f = figure('Visible','off','Position',[100 100 1400 900]);
subplot(2,2,1); plot(A.RM1K,A.RM2K,'-',B.RM1K,B.RM2K,'--','LineWidth',1.3); grid on
title('Fig 30.2  Engagement Geometry'); xlabel('Downrange (km)'); ylabel('Altitude (km)')
legend('Perpendicular to LOS','Perpendicular to VM','Location','best')
subplot(2,2,2); plot(A.T,A.XNCG,'-',B.T,B.XNCG,'--','LineWidth',1.3); grid on
title('Fig 30.3  Commanded Acceleration'); xlabel('Time (s)'); ylabel('n_c (g)')
legend('Perpendicular to LOS','Perpendicular to VM','Location','best')
subplot(2,2,3); plot(A.T,A.GAMDEG,'-',B.T,B.GAMDEG,'--','LineWidth',1.3); grid on
title('Fig 30.4  Flight Path Angle'); xlabel('Time (s)'); ylabel('\gamma (deg)')
legend('Perpendicular to LOS','Perpendicular to VM','Location','best')
subplot(2,2,4); plot(A.T,A.VMM,'-',B.T,B.VMM,'--','LineWidth',1.3); grid on
ylim([0 260]);
title('Fig 30.5  Missile Velocity'); xlabel('Time (s)'); ylabel('V_M (m/s)')
legend('Perpendicular to LOS','Perpendicular to VM','Location','best')
saveas(f, fullfile(figdir,'listing30_1_opt_ichoice.png')); close(f);

f = figure('Visible','off','Position',[100 100 1400 450]);
subplot(1,3,1)
plot(B.RM1K,B.RM2K,'--',R(3).out.RM1K,R(3).out.RM2K,'-', ...
     R(4).out.RM1K,R(4).out.RM2K,':','LineWidth',1.3); grid on
title('OPT vs biased PN (perp-V)'); xlabel('Downrange (km)'); ylabel('Altitude (km)')
legend('OPT','BPN listing bias','BPN prose bias','Location','best')
subplot(1,3,2)
plot(B.T,B.GAMDEG,'--',R(3).out.T,R(3).out.GAMDEG,'-', ...
     R(4).out.T,R(4).out.GAMDEG,':','LineWidth',1.3); grid on
title('Flight path angle'); xlabel('Time (s)'); ylabel('\gamma (deg)')
subplot(1,3,3)
plot(B.T,B.XNCG,'--',R(3).out.T,R(3).out.XNCG,'-', ...
     R(4).out.T,R(4).out.XNCG,':','LineWidth',1.3); grid on
title('Commanded acceleration'); xlabel('Time (s)'); ylabel('n_c (g)')
saveas(f, fullfile(figdir,'listing30_1_bias_erratum.png')); close(f);

%% ---- step-size convergence ---------------------------------------------
%  This is the check that invalidated scripts/research/ditcg/sim3d.py, whose
%  terminal miss halved every time dt halved (0.020 -> 1.704 m, 0.010 -> 1.023,
%  0.005 -> 0.461): its miss was quantisation residue, not physics.  A usable
%  reference must NOT behave that way.
fprintf('\n-- step-size convergence (miss vs H multiplier) --\n');
fprintf('%10s %14s %14s %14s %14s\n','HSCALE','miss_A_m','miss_B_m','tf_A_s','tf_B_s');
for hs = [1 0.5 0.25]
    oa = zarchan_listing30_1('IGUID',0,'ICHOICE',0,'HSCALE',hs);
    ob = zarchan_listing30_1('IGUID',0,'ICHOICE',1,'HSCALE',hs);
    fprintf('%10.4g %14.6f %14.6f %14.5f %14.5f\n', hs, ...
            oa.miss_m, ob.miss_m, oa.tf, ob.tf);
end

fprintf('\nfigures -> %s\n', figdir);
end

function v = rms_(x), v = sqrt(mean(x(~isnan(x)).^2)); end

function verdict(label, ok)
if ok, tag = 'REPRODUCED'; else, tag = 'NOT REPRODUCED'; end
fprintf('   [%-14s] %s\n', tag, label);
end
