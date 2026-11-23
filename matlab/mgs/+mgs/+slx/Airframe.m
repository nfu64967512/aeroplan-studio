classdef Airframe < matlab.System
%MGS.SLX.AIRFRAME  Simulink Airframe block: the DERIVATIVE, N vehicles at once.
%
%   in   X   N x 10   state
%   in   U   N x 3    [nav_roll nav_pitch thr_pc]
%   out  DX  N x 10   state derivative
%   out  AUX N x 6    [alpha_deg CL nz_body nz_max_aero eas gs]
%
%   THIS BLOCK DOES NOT INTEGRATE.  It outputs dx/dt and the model wires it
%   into a continuous Integrator, which is how the MathWorks example is built
%   (Aerodynamics & Equations of Motion feeding integrators) and which lets
%   Simulink's variable-step solver do the work my hand-rolled RK4 does in
%   mgs.simSwarm.
%
%   *** The two integrators are NOT identical, and results will differ in the
%   last digits. ***  mgs.simSwarm reproduces stot_sim bit for bit because it
%   copies its fixed-step RK4 and its secant touchdown search.  Simulink's
%   ode45 with zero-crossing detection is a different -- generally better --
%   numerical scheme, and it will not give the same last digits.  Use the
%   Simulink model to study STRUCTURE; use mgs.simSwarm for anything that has
%   to line up with the 32-flight truth set.  mgs.selftest checks the two
%   agree to a stated tolerance rather than pretending they are the same.
%
%   See also MGS.AIRFRAME, STOT_PLANT, MGS.SLX.BUILD.

properties (Nontunable)
    Params = struct()
    N (1,1) double {mustBePositive, mustBeInteger} = 2
end

properties (Access = private)
    c
end

methods (Access = protected)
    function setupImpl(obj)
        obj.c = local_params(obj.Params);
    end

    function [DX, AUX] = stepImpl(obj, X, U)
        DX  = zeros(obj.N, 10);
        AUX = zeros(obj.N, 6);
        for i = 1:obj.N
            [dx, ax] = mgs.airframe(X(i,:).', U(i,:).', obj.c);
            % Park a vehicle that has reached the datum.  Without this the
            % solver keeps flying it below ground, where it curves back up and
            % contaminates the swarm statistics -- a real simulator in this
            % project once scored second passes of drowned airframes as hits.
            % The crossing TIME is then recovered by interpolating the log.
            if X(i, 3) <= 0, dx(:) = 0; end
            DX(i, :) = dx(:).';
            AUX(i, :) = [rad2deg(ax.alpha_b), ax.CL, ax.nz_body, ...
                         ax.nz_max_aero, ax.eas, ax.gs];
        end
    end

    function n = getNumInputsImpl(~),  n = 2; end
    function n = getNumOutputsImpl(~), n = 2; end
    function varargout = getOutputSizeImpl(obj)
        varargout = {[obj.N 10], [obj.N 6]};
    end
    function varargout = getOutputDataTypeImpl(~)
        varargout = {'double', 'double'};
    end
    function varargout = isOutputComplexImpl(~),   varargout = {false, false}; end
    function varargout = isOutputFixedSizeImpl(~), varargout = {true, true};   end
end
end

function c = local_params(p)
if isempty(fieldnames(p)), c = mgs.params(); else, c = p; end
end
