function [cmd, gs, dbg] = guidance(meas, gs, tnow, c)
%MGS.GUIDANCE  Guidance block -- phase logic around the validated PN law.
%
%   [cmd, gs, dbg] = mgs.guidance(meas, gs, tnow, c)
%   gs = mgs.guidance()                 seed a guidance state (FIRST output)
%
%   meas   output of mgs.seekerTracker
%   gs     guidance state; .phase drives everything
%   tnow   [s] node wall clock, same origin as gs.dive_T
%   c      mgs.params()
%
%   cmd    .roll .pitch [rad], .thr [0..1]
%   dbg    every saturation flag for the tick, plus .phase
%
% ---------------------------------------------------------------------------
%   THE PN LAW IS NOT IN THIS FILE.
%
%   Phase 'dive' calls stot_guidance, which is the line-by-line port of
%   tot_node.py::_pn_cmd checked against 32 flights.  This file only decides
%   WHICH phase is active and what happens in the phases that are not PN.
%
%   Corresponds to the Guidance subsystem of the MathWorks example, whose mode
%   logic is a Stateflow chart.  Same job, and the same reason it is separated
%   from the law: the mode logic is where the interesting failures live.
%
%   *** PHASES OTHER THAN 'dive' ARE UNCALIBRATED. ***  The truth set only ever
%   observed the vehicle from release onwards.  Anything this function does in
%   'hold' is a model of a leg nobody instrumented, and mgs.simSwarm marks the
%   run out.calibrated = false the moment a vehicle spends a tick in one.
%
%   Known and NOT modelled here, stated so it is not mistaken for absent:
%     - before PN engages the aircraft is on L1/TECS, not on this law.  On the
%       truth set mode=="dive" leads the PN opening by a median 8.1 s, and all
%       six observed failures were aircraft that were still CLIMBING when PN
%       took over.  The 'hold' phase below does not reproduce L1/TECS; it
%       holds a commanded attitude.  Pre-PN climb is therefore an INPUT here
%       (set ic.gam0_deg > 0), not an emergent behaviour.
%
%   See also STOT_GUIDANCE, MGS.SEEKERTRACKER, MGS.AUTOPILOT.

if nargin == 0
    % Seeding convention, shared by mgs.guidance / mgs.autopilot /
    % mgs.seekerTracker: called with no arguments, the FIRST output is the
    % seed state.  It is first so that `gs = mgs.guidance()` is right; a
    % seed hidden in output 2 is silently assignable to output 1.
    cmd = seed(); gs = cmd; dbg = struct(); return;
end
if isempty(gs), gs = seed(); end

switch gs.phase
    % ---------------------------------------------------------------- dive --
    case 'dive'
        switch law_of(c.pn)
            case 'node'
                % The flight law: line-by-line port of tot_node.py::_pn_cmd,
                % checked against 32 sorties.  This is the default and the
                % only one with evidence behind it.
                %
                % ⚠️⚠️ stot_guidance is pinned to commit d7562e2 and does NOT
                % implement the nz_clamp_after_floor branch (tot_node.py:3102),
                % which is newer.  Honouring the flag silently is impossible,
                % so setting it here is an ERROR rather than a no-op -- a knob
                % that reads as set and does nothing is the failure this
                % project is organised against.  Use mgs.lawCheck against a
                % real log to study that flag.
                if isfield(c.pn, 'nz_clamp_after_floor') && c.pn.nz_clamp_after_floor
                    error('mgs:guidance:nzClampAfterFloor', ...
                        ['c.pn.nz_clamp_after_floor = true, but ' ...
                         'stot_guidance.m does not implement that branch ' ...
                         '(it is newer than the commit the port is pinned ' ...
                         'to).  Simulating it would silently give you the ' ...
                         'OLD ordering.  Measured on NZC1: with the flag on, ' ...
                         'the old chain mispredicts cmd_nz by 22-39 g.  ' ...
                         'Either port the branch into stot_guidance.m and ' ...
                         're-establish the parity check, or study the flag ' ...
                         'with mgs.lawCheck against a real tot_*.csv.']);
                end
                [cmd, gs, dbg] = stot_guidance(meas, gs, tnow, c.pn);
            case 'zarchan_additive'
                % Zarchan's additive terminal-angle term instead of our
                % one-sided min().  NEVER FLOWN -- see the file header.
                [cmd, gs, dbg] = mgs.zarchan.additiveLaw(meas, gs, tnow, c.pn);
            otherwise
                error('mgs:guidance:law', ...
                    ['unknown c.pn.law "%s" (node | zarchan_additive).  ' ...
                     'Not defaulting to "node": a typo would silently give ' ...
                     'you the flight law when you asked for the other one.'], ...
                    law_of(c.pn));
        end
        dbg.phase = 'dive';

    % ---------------------------------------------------------------- hold --
    case 'hold'
        % Hold the ring.  UNCALIBRATED, and wrong in a direction we measured:
        % the commanded ring (95 m/s on 1300 m) was never flown -- the real
        % vehicles did 124-186 m/s on ~5867 m and cut straight through, which
        % is why release sigma came out at 125 deg and release R at 3140 m.
        % c.up.ring_honest = true models what was flown instead.
        if c.up.ring_honest
            rr = c.up.ring_r_meas;
        else
            rr = c.up.ring_r;
        end
        bank = atan2(meas.vh^2, max(rr, 1) * c.pn.g);
        bank = min(bank, deg2rad(c.pn.node_roll_lim_deg));
        cmd = struct('roll', sign_of(gs.ring_dir)*bank, ...
                     'pitch', gs.hold_pitch, 'thr', gs.hold_thr);
        dbg = struct('phase', 'hold', 'ring_r', rr, 'bank', bank, ...
                     'd', meas.d, 'h', meas.h, 'R', meas.R, 'V', ...
                     hypot(meas.vh, -meas.vd), 'gam', ...
                     atan2(-meas.vd, max(meas.vh,1e-3)), ...
                     'lam_dot', meas.lam_dot, 'sig_dot', meas.sig_dot, ...
                     't_go', NaN, 'frozen', false);

    % ---------------------------------------------------------------- done --
    case 'done'
        cmd = struct('roll', gs.roll_frz, 'pitch', gs.pitch_frz, 'thr', 0.0);
        dbg = struct('phase', 'done', 'd', meas.d, 'h', meas.h, ...
                     'R', meas.R, 't_go', NaN, 'frozen', true);

    otherwise
        error('mgs:guidance:phase', ...
            ['unknown phase "%s".  Allowed: hold | dive | done.'], gs.phase);
end
end

% -----------------------------------------------------------------------------
function gs = seed()
%SEED  A guidance state with every field stot_guidance expects, plus phase.
gs = struct( ...
    'phase',     'dive', ...
    'alpha_lp',  0.0, ...
    'has_alpha', false, ...
    'frozen',    false, ...
    'roll_frz',  0.0, ...
    'pitch_frz', 0.0, ...
    'dive_T',    NaN, ...
    'ring_dir',  +1, ...      % +1 = clockwise seen from above
    'hold_pitch', 0.0, ...
    'hold_thr',   0.36);      % pn.thr_trim
end

function s = sign_of(v)
if v < 0, s = -1; else, s = 1; end
end

function v = law_of(pn)
if isfield(pn, 'law') && ~isempty(pn.law), v = lower(char(pn.law)); else, v = 'node'; end
end
