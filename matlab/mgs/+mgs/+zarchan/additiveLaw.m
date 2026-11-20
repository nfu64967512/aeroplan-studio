function [cmd, gs, dbg] = additiveLaw(m, gs, tnow, pn)
%MGS.ZARCHAN.ADDITIVELAW  Our law with Zarchan's ADDITIVE terminal-angle term.
%
%   Same signature as STOT_GUIDANCE.  Everything is identical to it except the
%   four lines that combine the PN demand with the terminal-angle demand.
%
% ---------------------------------------------------------------------------
%   WHY THIS EXISTS
%
%   tot_node combines them with a ONE-SIDED MINIMUM:
%
%       gam_cmd = min(gam_cmd_pn, gam_los * los_floor)        tot_node.py:2866
%
%   Zarchan never does that.  In all four places he imposes a terminal angle
%   it is an ADDITIVE, SIGNED term -- Ch. 24 (zarchan.txt:8109), Ch. 28,
%   Ch. 30 (10887) and Appendix A (14446).  The Ch. 30 form, verbatim from
%   line 10887 of the local text:
%
%       nc = 4*Vc*lamdot + 2*Vc*(lam - gamF)/tgo
%            \_ PN term _/   \_ terminal-angle term, ADDED ___/
%
%   The difference is not cosmetic, and it is TWO differences.
%
%   (1) A min() has ZERO GRADIENT on its active side: once the floor is the
%       smaller of the two, the guidance loop is OPEN with respect to the
%       sightline -- moving the target changes nothing.  An additive signed
%       error keeps pulling.  On the truth set the floor is active on 43.1 %
%       of PN samples overall and 89.8-99.7 % on the four aircraft that
%       missed, which is to say those four were barely doing PN at all.
%
%   (2) The min() sits AFTER the nz clamp, so the terminal-angle demand
%       bypasses [nz_min, nz_max] entirely -- which is why cmd_nz reaches
%       -18.28 g, and why a rate-space reading of that command produces the
%       spurious "0.063 tracking gain".  Below, the shaping term is added
%       BEFORE the clamp, so the TOTAL demand is clamped.  That is also
%       Zarchan's practice: he clamps the whole command, never the PN part
%       alone.  So this import closes the bypass as well as the loop.
%
%   *** THIS LAW HAS NEVER FLOWN. ***  There is no truth data for it, and none
%   of the 32-flight validation transfers.  It is a hypothesis with a good
%   pedigree, not a fix.  mgs.simSwarm marks any run using it uncalibrated.
%
% ---------------------------------------------------------------------------
%   PARITY, so the two copies cannot drift
%
%   With pn.shape_gain = 0 AND pn.los_floor = 0 this function must return
%   EXACTLY what stot_guidance returns -- same bits.  mgs.selftest asserts it.
%   If someone edits stot_guidance and forgets this file, that test goes red
%   instead of the two laws quietly diverging.
%
%   pn.shape_gain  Zarchan's coefficient on the terminal-angle term.  His
%                  value is 2.0, paired with N' = 4.  WE FLY n_vert = 5, and
%                  the pairing of those two gains is not something the book
%                  addresses -- his N' and his 2 come out of one optimal
%                  control problem together.  Treat 2.0 as a starting point
%                  that has to be swept, not as a value with authority.
%
%   See also STOT_GUIDANCE, MGS.ZARCHAN.CROSSCHECK, PUREPN2D.

GRAV = pn.g;

% ---- _geom (identical) -----------------------------------------------------
dn = -m.n;
de = -m.e;
d  = max(1e-3, hypot(dn, de));
h  = m.z;
R  = max(1e-3, hypot(d, h));
vn = m.vn;  ve = m.ve;  vd = m.vd;
vh   = hypot(vn, ve);
hdot = -vd;
V    = max(1e-3, hypot(vh, hdot));
gam  = atan2(hdot, max(vh, 1e-3));

% ---- lateral channel (identical) -------------------------------------------
lam_dot = (de*vn - dn*ve) / (d*d);
a_h     = pn.n_lat * vh * lam_dot;
if pn.itcg_on
    error('mgs:zarchan:itcg', ...
        ['itcg_on is not implemented here either.  A stub returning 0 would ' ...
         'look exactly like a working ITCG.']);
end
a_h = a_h + 0.0;
roll_cmd = atan2(a_h, GRAV * max(cos(gam), 0.25));

% ---- longitudinal: THE ONLY DIFFERENCE =====================================
ddot    = -(dn*vn + de*ve) / d;
sig_dot = (hdot*d - h*ddot) / (R*R);
t_go    = R / max(V, 1e-3);

gdot_pn = -pn.n_vert * sig_dot;                  % the PN term, unchanged

% ⚠️⚠️⚠️ 2026-08-22 CORRECTION.  THE FIRST VERSION OF THIS BLOCK WAS NOT
% ZARCHAN'S LAW, AND EVERY MEASUREMENT TAKEN THROUGH IT WAS OF SOMETHING ELSE.
%
%   Zarchan Ch.30, zarchan.txt:10942 verbatim, with GAMFDEG = -90 at :10934:
%       XNC = 4.*VC*XLAMD + 2.*VC*(XLAM - GAMF)/TGO
%   The repo's own correct copy: matlab/ref/zarchan_listing30_1.m:156.
%
%   XLAM  is the line-of-sight angle.
%   GAMF  is a DESIGNER-CHOSEN FINAL flight-path angle -- a CONSTANT SETPOINT,
%         the impact angle you are asking for.  Nominally -90 deg.
%
%   The first import wrote  (gam_los*los_floor - gam)  instead: it replaced the
%   constant setpoint GAMF with the CURRENT gamma, and scaled the reference by
%   los_floor.  Two consequences, both measured on all 32 release states:
%     - the shaping error came out NEGATIVE on 32/32 (median -22.58 deg) where
%       Zarchan's is POSITIVE on 32/32 (median +63.44 deg).  Opposite sign,
%       median magnitude ratio 2.8.  Zarchan's positive bias LOFTS FIRST so the
%       vehicle has altitude to dive out of -- that is the whole mechanism.
%     - with los_floor = 0 the reference collapsed to 0, making the term a
%       LEVEL-FLIGHT ATTRACTOR that fights the dive.  Measured: at shape_gain
%       3.0, 24 of 32 aircraft were still airborne at t_max with median error
%       722.7 m.
%   ⇒ it was an attractor on gamma with no impact-angle setpoint at all,
%     structurally the same object as pure PN, which already reaches gam = lam
%     against a stationary target.
%
% ⚠️ los_floor IS DELIBERATELY NOT USED HERE.  In the flown law it scales a CAP
%    that a min() takes; there is no min() in this file.  Reusing the name for a
%    reference scale is exactly the "same knob, two jobs" trap this project
%    keeps paying for, so setting it while this law is active is an ERROR.
if los_floor_of(pn) ~= 0.0
    error('mgs:additiveLaw:losFloor', ...
        ['pn.los_floor = %g, but this law has no floor: there is no min() in ' ...
         'additiveLaw.m.  In the flown law los_floor scales a CAP; an earlier ' ...
         'version of this file reused it as a scale on the terminal-angle ' ...
         'REFERENCE, which turned the term into a level-flight attractor at ' ...
         'los_floor = 0.  Set pn.los_floor = 0 and use pn.gam_f_deg to choose ' ...
         'the impact angle.'], los_floor_of(pn));
end
gam_los = -atan2(h, max(d, 1e-3));               % XLAM, the sightline angle
gam_f   = deg2rad(gam_f_deg_of(pn));             % GAMF, the DEMANDED impact angle
gdot_shape = 0.0;
if shape_gain_of(pn) ~= 0.0
    gdot_shape = shape_gain_of(pn) * (gam_los - gam_f) / max(t_go, 1e-3);
end
gdot_raw = gdot_pn + gdot_shape;                 % <<< ADDITIVE (10942)

% The clamp now sees the TOTAL demand, which is also what Zarchan does: he
% clamps the achieved state after the full command, never the PN part alone.
nz_raw  = cos(gam) + V*gdot_raw/GRAV;
nz      = max(pn.nz_min, min(pn.nz_max, nz_raw));
gdot    = (nz - cos(gam)) * GRAV / V;
gam_cmd = gam + gdot * pn.lead;

% No min().  If shape_gain = 0 (and los_floor = 0, enforced above) this block
% reduces exactly to the stot_guidance block -- what the parity test checks.
los_active = false;                              % kept so dbg stays comparable
% ===========================================================================

% ---- alpha estimate and attitude conversion (identical) --------------------
th_meas = m.pitch;
a_raw   = th_meas - gam;
if ~gs.has_alpha
    gs.alpha_lp = a_raw;
    gs.has_alpha = true;
end
k = min(1.0, pn.tick_dt / pn.alpha_tau);
gs.alpha_lp = gs.alpha_lp + k*(a_raw - gs.alpha_lp);
pitch_cmd = gam_cmd + gs.alpha_lp / max(cos(roll_cmd), 0.5);

% ---- node-side attitude clips (identical) ---------------------------------
if pn.dive_pitch_min_deg < 0.0
    pmin = deg2rad(pn.dive_pitch_min_deg);
else
    pmin = deg2rad(-80.0);
end
pmax = deg2rad(pn.dive_pitch_max_deg);
pitch_lo = (pitch_cmd < pmin);
pitch_hi = (pitch_cmd > pmax);
pitch_cmd = max(pmin, min(pmax, pitch_cmd));

r_lim = deg2rad(pn.node_roll_lim_deg) * abs(cos(th_meas));
roll_node_clip = (abs(roll_cmd) > r_lim);
roll_cmd = max(-r_lim, min(r_lim, roll_cmd));

% ---- throttle (identical) --------------------------------------------------
switch lower(pn.throttle_law)
case 'clock'
    if isnan(gs.dive_T)
        t_left = 0.0;
    else
        t_left = gs.dive_T - tnow;
    end
    late  = (t_left <= 1.0);
    if ~late
        t_rem = t_left;
    else
        t_rem = max(1.0, R/max(V,1e-3));
    end
    v_req = max(pn.v_req_floor, min(pn.vmax, R/t_rem));
    if late && pn.late_full
        v_req = pn.vmax;
    end
case 'vmax'
    late  = false;
    v_req = pn.vmax;
otherwise
    error('mgs:zarchan:law', 'unknown throttle_law "%s"', pn.throttle_law);
end
thr_unsat = pn.thr_trim + pn.thr_gain*(v_req - V);
thr = max(0.0, min(1.0, thr_unsat));
if pn.dive_full_thr
    thr = 1.0;
end

% ---- terminal freeze (identical) -------------------------------------------
freeze_now = (t_go <= pn.fz_tgo) || (R <= pn.fz_r);
roll_out = roll_cmd;  pitch_out = pitch_cmd;
if freeze_now
    if ~gs.frozen
        gs.roll_frz  = roll_cmd;
        gs.pitch_frz = pitch_cmd;
        gs.frozen    = true;
    end
    roll_out  = gs.roll_frz;
    pitch_out = gs.pitch_frz;
end

cmd = struct('roll', roll_out, 'pitch', pitch_out, 'thr', thr);

dbg = struct( ...
    'd',d, 'h',h, 'R',R, 'V',V, 'gam',gam, 'vh',vh, 'hdot',hdot, ...
    'lam_dot',lam_dot, 'sig_dot',sig_dot, ...
    'nz_raw',nz_raw, 'nz',nz, ...
    'nz_lo', nz_raw < pn.nz_min, ...
    'nz_hi', nz_raw > pn.nz_max, ...
    'gam_cmd_pn', gam + ((max(pn.nz_min, min(pn.nz_max, cos(gam)+V*gdot_pn/GRAV)) ...
                          - cos(gam))*GRAV/V) * pn.lead, ...
    'gam_los',gam_los, 'gam_cmd',gam_cmd, ...
    'los_active',los_active, ...
    'gdot_pn',gdot_pn, 'gdot_shape',gdot_shape, ...   % the split, for study
    'alpha_lp',gs.alpha_lp, ...
    'roll_cmd_raw',roll_cmd, ...
    'pitch_lo',pitch_lo, 'pitch_hi',pitch_hi, ...
    'roll_node_clip',roll_node_clip, 'r_lim',r_lim, ...
    'v_req',v_req, 'late',late, 'thr_unsat',thr_unsat, ...
    'thr_sat_lo', thr_unsat < 0.0, 'thr_sat_hi', thr_unsat > 1.0, ...
    't_go',t_go, 'frozen',freeze_now);
end

% -----------------------------------------------------------------------------
function v = gam_f_deg_of(pn)
%GAM_F_DEG_OF  Zarchan's GAMFDEG: the demanded terminal flight-path angle.
%   ⚠️ NOT the current gamma and NOT the sightline.  It is the impact angle you
%   are ASKING FOR, and it is what makes this a terminal-angle law rather than
%   an attractor.  Zarchan's own value is -90 (zarchan.txt:10934).
if isfield(pn, 'gam_f_deg') && ~isempty(pn.gam_f_deg)
    v = double(pn.gam_f_deg);
else
    v = -90.0;
end
end

function v = los_floor_of(pn)
if isfield(pn, 'los_floor'), v = pn.los_floor; else, v = 0.0; end
end

function v = shape_gain_of(pn)
%SHAPE_GAIN_OF  Zarchan's coefficient.  Absent means ZERO, not his 2.0:
%   silently adopting a gain nobody set is the failure mode this whole
%   project is organised against.
if isfield(pn, 'shape_gain'), v = pn.shape_gain; else, v = 0.0; end
end
