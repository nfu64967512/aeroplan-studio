function [dx, aux] = airframe(x, u, c)
%MGS.AIRFRAME  Airframe block -- 3-DOF point mass + ArduPilot angle loop.
%
%   [dx, aux] = mgs.airframe(x, u, c)
%
%   A naming shim over stot_plant so the four blocks of this environment line
%   up one-for-one with the MathWorks example:
%
%       Seeker/Tracker  ->  mgs.seekerTracker
%       Guidance        ->  mgs.guidance      (-> stot_guidance)
%       Autopilot       ->  mgs.autopilot     (-> stot_fcs_clamp)
%       Airframe        ->  mgs.airframe      (-> stot_plant)
%
%   The airframe content differs from the example's in three ways that matter
%   when reading across:
%
%     - the example's airframe has a FIN ACTUATOR and 2-D aerodynamic lookup
%       tables over incidence and Mach, valid Mach 2-4 at 10-60 kft.  Ours is
%       a subsonic jet UAV at 0-3.1 km with an analytic CD0 + induced-drag
%       polar, CD0 = 0.01500 inverse-solved from 1617 flight samples.
%
%     - the example commands acceleration into a three-loop autopilot; ours
%       commands attitude into an angle loop (see mgs.autopilot).
%
%     - the example's atmosphere is the full ISA with a stratosphere branch.
%       Ours is the troposphere expression only, because 3.1 km is the ceiling
%       and predictor.py -- which the drag polar was fitted through -- uses
%       exactly that expression.  Do not fly this model above 11 km.
%
%   See also STOT_PLANT, MGS.AUTOPILOT.

il = c.il;
il.tconst_p = c.fc.PTCH2SRV_TCONST;
il.tconst_r = c.fc.RLL2SRV_TCONST;

env = struct('home_amsl', c.swarm.tgt_amsl);

if nargout > 1
    [dx, aux] = stot_plant(x, u, c.b, il, env);
else
    dx = stot_plant(x, u, c.b, il, env);
end
end
