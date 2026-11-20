classdef SeekerTracker < matlab.System
%MGS.SLX.SEEKERTRACKER  Simulink Seeker/Tracker block, N vehicles at once.
%
%   in   X     N x 10   plant state  [n e z V gam psi th q phi p]
%   out  MEAS  N x 10   [n e z vn ve vd pitch lam_dot sig_dot Vc]
%
%   Runs at the TELEMETRY rate (40 Hz), not the guidance rate, because that is
%   the real constraint: a node cannot see what MAVLink has not delivered.
%
%   The block is a thin wrapper over mgs.seekerTracker so that the Simulink
%   model and the MATLAB loop cannot diverge.  Set the block to INTERPRETED
%   EXECUTION -- the underlying code raises on bad configuration, which is the
%   behaviour we want and which does not code-generate.
%
%   See also MGS.SEEKERTRACKER, MGS.SLX.BUILD.

properties (Nontunable)
    %Params  mgs.params() struct
    Params = struct()
    %N  number of vehicles
    N (1,1) double {mustBePositive, mustBeInteger} = 2
    %SampleTime  telemetry period [s]
    SampleTime (1,1) double {mustBePositive} = 0.025
end

properties (Access = private)
    tk
    c
    tgt
end

methods (Access = protected)
    function setupImpl(obj)
        obj.c = local_params(obj.Params);
        obj.tgt = struct('n', 0.0, 'e', 0.0, 'z', 0.0);
        obj.tk = cell(1, obj.N);
        for i = 1:obj.N, obj.tk{i} = mgs.seekerTracker(); end
    end

    function resetImpl(obj)
        for i = 1:obj.N, obj.tk{i} = mgs.seekerTracker(); end
    end

    function MEAS = stepImpl(obj, X)
        MEAS = zeros(obj.N, 10);
        for i = 1:obj.N
            [m, obj.tk{i}] = mgs.seekerTracker(X(i, :).', obj.tgt, obj.tk{i}, obj.c);
            MEAS(i, :) = [m.n, m.e, m.z, m.vn, m.ve, m.vd, m.pitch, ...
                          m.lam_dot, m.sig_dot, m.Vc];
        end
    end

    function sts = getSampleTimeImpl(obj)
        sts = obj.createSampleTime('Type', 'Discrete', 'SampleTime', obj.SampleTime, ...
            'OffsetTime', 0);
    end

    function sz = getOutputSizeImpl(obj),      sz = [obj.N 10];  end
    function dt = getOutputDataTypeImpl(~),    dt = 'double';    end
    function tf = isOutputComplexImpl(~),      tf = false;       end
    function tf = isOutputFixedSizeImpl(~),    tf = true;        end
end
end

function c = local_params(p)
if isempty(fieldnames(p)), c = mgs.params(); else, c = p; end
end
