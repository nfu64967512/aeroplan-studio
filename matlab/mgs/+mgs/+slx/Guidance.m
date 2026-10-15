classdef Guidance < matlab.System
%MGS.SLX.GUIDANCE  Simulink Guidance block, N vehicles at once.
%
%   in   MEAS  N x 10   from mgs.slx.SeekerTracker
%   out  CMD   N x 3    [roll pitch thr]   attitude target, thr in 0..1
%   out  DBG   N x 6    [nz gam_cmd los_active t_go frozen phase_code]
%
%   Runs at the GUIDANCE rate (20 Hz = TICK_PERIOD_S).  Calls mgs.guidance,
%   which calls stot_guidance -- the line-by-line port of tot_node.py::_pn_cmd.
%   The PN law exists once in this repository and this block does not contain
%   a second copy of it.
%
%   phase_code: 1 hold, 2 dive, 3 done.
%
%   See also MGS.GUIDANCE, STOT_GUIDANCE.

properties (Nontunable)
    Params = struct()
    N (1,1) double {mustBePositive, mustBeInteger} = 2
    SampleTime (1,1) double {mustBePositive} = 0.05
    %DiveT  scheduled touchdown per vehicle, [s] from t = 0.  NaN = free.
    DiveT = NaN
    %TRelease  release time per vehicle, [s].  Before it the vehicle holds.
    TRelease = 0
end

properties (Access = private)
    gs
    c
    t
end

methods (Access = protected)
    function setupImpl(obj)
        obj.c = local_params(obj.Params);
        obj.t = 0.0;
        obj.gs = cell(1, obj.N);
        dT = expand(obj.DiveT, obj.N);
        for i = 1:obj.N
            g = mgs.guidance();
            g.dive_T = dT(i);
            obj.gs{i} = g;
        end
    end

    function resetImpl(obj)
        obj.setupImpl();
    end

    function [CMD, DBG] = stepImpl(obj, MEAS)
        CMD = zeros(obj.N, 3);
        DBG = zeros(obj.N, 6);
        tRel = expand(obj.TRelease, obj.N);
        for i = 1:obj.N
            m = struct('n', MEAS(i,1), 'e', MEAS(i,2), 'z', MEAS(i,3), ...
                       'vn', MEAS(i,4), 've', MEAS(i,5), 'vd', MEAS(i,6), ...
                       'pitch', MEAS(i,7), 'lam_dot', MEAS(i,8), ...
                       'sig_dot', MEAS(i,9), 'Vc', MEAS(i,10), ...
                       'd', hypot(MEAS(i,1), MEAS(i,2)), 'h', MEAS(i,3), ...
                       'R', norm(MEAS(i,1:3)), 'vh', hypot(MEAS(i,4), MEAS(i,5)));
            if obj.t + 1e-9 < tRel(i)
                obj.gs{i}.phase = 'hold';
            elseif strcmp(obj.gs{i}.phase, 'hold')
                obj.gs{i}.phase = 'dive';
            end
            [cmd, obj.gs{i}, dbg] = mgs.guidance(m, obj.gs{i}, obj.t, obj.c);
            CMD(i, :) = [cmd.roll, cmd.pitch, cmd.thr];
            DBG(i, :) = [getf(dbg, 'nz', NaN), getf(dbg, 'gam_cmd', NaN), ...
                         double(getf(dbg, 'los_active', false)), ...
                         getf(dbg, 't_go', NaN), ...
                         double(getf(dbg, 'frozen', false)), ...
                         phase_code(obj.gs{i}.phase)];
        end
        obj.t = obj.t + obj.SampleTime;
    end

    function sts = getSampleTimeImpl(obj)
        sts = obj.createSampleTime('Type', 'Discrete', 'SampleTime', obj.SampleTime, ...
            'OffsetTime', 0);
    end

    function n = getNumOutputsImpl(~),      n = 2;                       end
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

function v = expand(v, n)
if isscalar(v), v = repmat(v, 1, n); end
if numel(v) < n
    error('mgs:slx:Guidance:size', ...
        ['a per-vehicle vector has %d entries for %d vehicles.  Not padding ' ...
         'it: a silently padded schedule is a schedule nobody set.'], ...
        numel(v), n);
end
end

function v = getf(s, f, dflt)
if isstruct(s) && isfield(s, f), v = double(s.(f)); else, v = dflt; end
end

function k = phase_code(p)
switch p
    case 'hold', k = 1;
    case 'dive', k = 2;
    case 'done', k = 3;
    otherwise,   k = 0;
end
end
