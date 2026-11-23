classdef Autopilot < matlab.System
%MGS.SLX.AUTOPILOT  Simulink Autopilot block, N vehicles at once.
%
%   in   CMD  N x 3    [roll pitch thr]  from the Guidance block
%   in   X    N x 10   plant state, for the MEASURED attitude the clamps use
%   out  U    N x 3    [nav_roll nav_pitch thr_pc]
%   out  FL   N x 6    [fc_roll_clip fc_pitch_lo fc_pitch_hi thr_clip
%                       thr_slew speed_refused]
%
%   Runs at the FLIGHT CONTROLLER rate (50 Hz = SCHED_LOOP_RATE).
%
%   Note the second input.  The firmware's pitch limit is scaled by
%   |cos(roll)| taken from the AHRS ESTIMATE at the top of the fast loop, not
%   from the command -- so this block feeds back the PLANT state, one step
%   delayed, and is not an algebraic loop.  Wiring the command in instead
%   changes the answer; stot_selftest has a check that catches exactly that.
%
%   See also MGS.AUTOPILOT, STOT_FCS_CLAMP.

properties (Nontunable)
    Params = struct()
    N (1,1) double {mustBePositive, mustBeInteger} = 2
    SampleTime (1,1) double {mustBePositive} = 0.02
end

properties (Access = private)
    ap
    c
end

methods (Access = protected)
    function setupImpl(obj)
        obj.c = local_params(obj.Params);
        obj.ap = cell(1, obj.N);
        for i = 1:obj.N
            a = mgs.autopilot();
            a.thr_pc_prev = NaN;     % bypass the slew limiter on tick 1
            obj.ap{i} = a;
        end
    end

    function resetImpl(obj)
        obj.setupImpl();
    end

    function [U, FL] = stepImpl(obj, CMD, X)
        U  = zeros(obj.N, 3);
        FL = zeros(obj.N, 6);
        for i = 1:obj.N
            cmd = struct('roll', CMD(i,1), 'pitch', CMD(i,2), 'thr', CMD(i,3));
            [u, obj.ap{i}, fl] = mgs.autopilot(cmd, X(i,:).', obj.ap{i}, ...
                                               obj.SampleTime, obj.c);
            U(i, :) = u(:).';
            FL(i, :) = [getf(fl, 'fc_roll_clip', 0), getf(fl, 'fc_pitch_lo', 0), ...
                        getf(fl, 'fc_pitch_hi', 0), getf(fl, 'thr_clip', 0), ...
                        getf(fl, 'thr_slew', 0), getf(fl, 'speed_refused', 0)];
        end
    end

    function sts = getSampleTimeImpl(obj)
        sts = obj.createSampleTime('Type', 'Discrete', 'SampleTime', obj.SampleTime, ...
            'OffsetTime', 0);
    end

    function n = getNumInputsImpl(~),  n = 2; end
    function n = getNumOutputsImpl(~), n = 2; end
    function varargout = getOutputSizeImpl(obj)
        varargout = {[obj.N 3], [obj.N 6]};
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

function v = getf(s, f, dflt)
if isstruct(s) && isfield(s, f), v = double(s.(f)); else, v = dflt; end
end
