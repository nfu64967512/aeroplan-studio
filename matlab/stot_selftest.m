function R = stot_selftest(verbose)
%STOT_SELFTEST  The checks that have to pass before any of this is believable.
%
%   R = stot_selftest()
%
%   1 order      LOS floor is applied AFTER the nz clamp, so pn_nz_min is nearly
%                inert.  Unit case + whole-run statistics.
%   2 switch     with los_floor = 0 the same nz_min sweep must bite.  This is the
%                only mechanism behind negative control N1, so it is the single
%                most important test in the file.
%   3 primitive  air_density / drag_n / thrust_n agree with predictor.py to 1e-12
%   4 dtinv      halving the integration step must NOT halve the touchdown error
%                (this is the check sim3d.py fails)
%   5 sign       descending flight has gam < 0 and hdot < 0
%   6 fcclamp    roll_meas = 65 deg, pitch_cmd = -70 deg  ->  nav_pitch = -33.8
%   7 noloop     fc_clamp reads the PLANT attitude, not this tick's command
%
%   Nothing here is compared against the 32-flight truth set -- these are
%   internal-consistency checks only, so passing them says nothing about
%   fidelity.  Fidelity is stot_run32.

if nargin < 1, verbose = true; end
R = struct(); pass = true;
p = @(varargin) fprintf(varargin{:});

c   = cfg.params();
ics = stot_ic32('zero');
ic  = ics(strcmp({ics.tag},'RRE7-U1'));
if isempty(ic), ic = ics(1); end

% ========================================================================= 1
% Unit case chosen so that BOTH things are true at once: the nz clamp really
% bites (nz_raw = -4.6, well below nz_min) and the LOS floor is the binding
% branch.  The nz_min sweep therefore moves the PN branch by ~0.1 rad and moves
% the command that is actually sent by exactly zero.  That is the whole claim.
m = struct('n',1000,'e',0,'z',3000,'vn',-196,'ve',0,'vd',39.7,'pitch',deg2rad(-8));
gs0 = struct('alpha_lp',deg2rad(3),'has_alpha',true,'frozen',false, ...
             'roll_frz',0,'pitch_frz',0,'dive_T',20);
cA = withpn(c,'nz_min',-1.0);  cB = withpn(c,'nz_min',-3.0);
[~,~,dA] = stot_guidance(m, gs0, 0, cA.pn);
[~,~,dB] = stot_guidance(m, gs0, 0, cB.pn);
d_unit = abs(dA.gam_cmd - dB.gam_cmd);
d_pre  = abs(dA.gam_cmd_pn - dB.gam_cmd_pn);
R.t1_unit_dgam_rad = d_unit;
R.t1_unit_dgam_pre_rad = d_pre;
R.t1_unit_nz_raw = dA.nz_raw;
ok = (d_unit < 1e-12) && (d_pre > 1e-3) && dA.los_active && dA.nz_lo;
pass = pass && ok;
p('1 order/unit    nz_raw = %.2f (clamped), d(gam_cmd_pn) = %.4f rad, d(gam_cmd) = %.3e rad, los_active=%d  %s\n', ...
  dA.nz_raw, d_pre, d_unit, dA.los_active, tf(ok));

% Order discrimination: on a state where the LOS floor does NOT bind, the two
% possible orderings give measurably different commands.  Assert we implement
% the one tot_node.py:2848 -> 2866 implements (clamp first, floor last).
m2 = struct('n',300,'e',0,'z',200,'vn',-170,'ve',0,'vd',100,'pitch',deg2rad(-30));
[~,~,d2] = stot_guidance(m2, gs0, 0, cA.pn);
gam2  = d2.gam;  V2 = d2.V;
gdot_raw = (d2.nz_raw - cos(gam2))*c.pn.g/V2;      % before the clamp
gdot_cl  = (d2.nz     - cos(gam2))*c.pn.g/V2;      % after  the clamp
right = min(gam2 + gdot_cl *c.pn.lead, d2.gam_los);   % clamp -> lead -> floor
wrong = min(gam2 + gdot_raw*c.pn.lead, d2.gam_los);   % floor applied pre-clamp
R.t1_order_right = right; R.t1_order_wrong = wrong; R.t1_order_got = d2.gam_cmd;
ok = abs(d2.gam_cmd - right) < 1e-12 && abs(right - wrong) > 1e-3;
pass = pass && ok;
p('1 order/discrim got %.5f rad; right-order %.5f, wrong-order %.5f (differ by %.4f)  %s\n', ...
  d2.gam_cmd, right, wrong, abs(right-wrong), tf(ok));

% Whole-run statistics on a real release state.
oA = stot_sim(ic, cA);
oB = stot_sim(ic, cB);
nA = min(size(oA.GD,1), size(oB.GD,1));
dg = abs(oA.GD(1:nA,13) - oB.GD(1:nA,13));
R.t1_run_median_dgam_deg = rad2deg(median(dg));
R.t1_run_frac_nonzero    = mean(dg > 1e-12);
R.t1_run_max_dgam_deg    = rad2deg(max(dg));
ok = R.t1_run_median_dgam_deg == 0;
pass = pass && ok;
p('1 order/run     median d(gam_cmd) = %.6f deg, ticks that moved = %.2f %%, max %.3f deg  %s\n', ...
  R.t1_run_median_dgam_deg, 100*R.t1_run_frac_nonzero, R.t1_run_max_dgam_deg, tf(ok));

% ========================================================================= 2
cN = cfg.params('pn.los_floor',0.0);
oC = stot_sim(ic, withpn(cN,'nz_min',-1.0));
oD = stot_sim(ic, withpn(cN,'nz_min',0.05));   % the real N1 negative control
nC = min(size(oC.GD,1), size(oD.GD,1));
dg2 = abs(oC.GD(1:nC,13) - oD.GD(1:nC,13));
R.t2_median_dgam_deg = rad2deg(median(dg2));
R.t2_frac_nonzero    = mean(dg2 > 1e-12);
R.t2_err_nzmin_neg1  = oC.err_touch;
R.t2_err_nzmin_p005  = oD.err_touch;
ok = R.t2_median_dgam_deg > 1e-3;
pass = pass && ok;
p('2 switch        los_floor=0: median d(gam_cmd) = %.4f deg over %.1f %% of ticks  %s\n', ...
  R.t2_median_dgam_deg, 100*R.t2_frac_nonzero, tf(ok));
p('                err_touch  nz_min=-1.0 -> %.2f m ; nz_min=+0.05 -> %.2f m\n', ...
  oC.err_touch, oD.err_touch);

% ========================================================================= 3
b = c.b;
rho = @(h) 1.225*(1 - 2.25577e-5*max(h,0)).^4.2586;
dragn = @(v,h,g) deal_drag(v,h,g,b,rho);
thrn  = @(v,tm,vr) min(max(min(0.36+0.030*(vr-v),1),0), tm/100)*b.Tfull;
ref = [ rho(0.0)        - 1.225;
        rho(1000.0)     - 1.1115734721198134;
        rho(3053.7)     - 0.9039298659725135;
        rho(11000.0)    - 0.36363532911032304;
        dragn(210,3000,-0.5) - 135.4646068317207;
        dragn(150,0,0)       - 93.36141673684835;
        dragn(180,1500,-0.3) - 115.93684634073122;
        thrn(150,100,210) - 308.209;
        thrn(210,100,210) - 110.95524;
        thrn(205,100,210) - 157.18659 ];
R.t3_max_abs_err = max(abs(ref));
ok = R.t3_max_abs_err < 1e-9;
pass = pass && ok;
p('3 primitive     max |MATLAB - predictor.py| = %.3e  %s\n', R.t3_max_abs_err, tf(ok));

% ========================================================================= 4
hs = [0.005 0.0025 0.00125];
tg = zeros(size(hs)); er = zeros(size(hs));
for k = 1:numel(hs)
    o = stot_sim(ic, cfg.params('sim.h_max',hs(k),'sim.log',false));
    tg(k) = o.t_dive; er(k) = o.err_touch;
end
R.t4_h = hs; R.t4_t_dive = tg; R.t4_err = er;
R.t4_t_range = max(tg)-min(tg);
R.t4_err_range = max(er)-min(er);
ok = (R.t4_t_range < 0.1) && (R.t4_err_range < 0.01);
pass = pass && ok;
p('4 dtinv         h = %s\n', mat2str(hs));
p('                t_dive    = %s   range %.5f s\n', num2str(tg,'%12.5f'), R.t4_t_range);
p('                err_touch = %s   range %.5f m  %s\n', num2str(er,'%12.5f'), R.t4_err_range, tf(ok));

% ========================================================================= 5
o5 = stot_sim(ic, c);
mid = round(size(o5.TR,1)*0.6);
R.t5_gam_deg = rad2deg(o5.TR(mid,6));
R.t5_hdot    = o5.TR(mid,5)*sin(o5.TR(mid,6));
ok = R.t5_gam_deg < 0 && R.t5_hdot < 0;
pass = pass && ok;
p('5 sign          mid-dive gam = %+.2f deg, hdot = %+.2f m/s  %s\n', ...
  R.t5_gam_deg, R.t5_hdot, tf(ok));

% ========================================================================= 6
[~, nav_pitch] = stot_fcs_clamp(0, deg2rad(-70), 0.5, deg2rad(65), deg2rad(-20), ...
                                NaN, 1/50, c.fc);
R.t6_nav_pitch_deg = rad2deg(nav_pitch);
ok = abs(R.t6_nav_pitch_deg - (-80*abs(cosd(65)))) < 1e-9;
pass = pass && ok;
p('6 fcclamp       roll_meas 65 deg, pitch_cmd -70 deg -> nav_pitch = %.4f deg (want %.4f)  %s\n', ...
  R.t6_nav_pitch_deg, -80*abs(cosd(65)), tf(ok));

% ========================================================================= 7
roll_cmd = deg2rad(70); pitch_cmd = deg2rad(-70);
roll_meas = deg2rad(10); pitch_meas = deg2rad(-20);
[~, np_state] = stot_fcs_clamp(roll_cmd,pitch_cmd,0.5,roll_meas,pitch_meas,NaN,1/50,c.fc);
[~, np_cmdfb] = stot_fcs_clamp(roll_cmd,pitch_cmd,0.5,roll_cmd ,pitch_cmd ,NaN,1/50,c.fc);
R.t7_nav_pitch_state_deg = rad2deg(np_state);
R.t7_nav_pitch_cmdfb_deg = rad2deg(np_cmdfb);
ok = abs(np_state - np_cmdfb) > 1e-6;
pass = pass && ok;
p('7 noloop        state feedback %.3f deg vs command feedback %.3f deg  %s\n', ...
  R.t7_nav_pitch_state_deg, R.t7_nav_pitch_cmdfb_deg, tf(ok));

R.pass = pass;
p('\nSELFTEST %s\n', tf(pass));
end

% ---------------------------------------------------------------------------
function c = withpn(c, f, v)
c.pn.(f) = v;
end
function D = deal_drag(v,h,g,b,rho)
qs = 0.5*rho(h)*b.S*v*v;
cl = b.W*cos(g)/max(qs,1e-6);
D  = qs*(b.CD0 + cl*cl/b.piEAR);
end
function s = tf(ok)
if ok, s = 'PASS'; else, s = '**FAIL**'; end
end
