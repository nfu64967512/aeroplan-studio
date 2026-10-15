function S = sizing(out, opts)
%MGS.ZARCHAN.SIZING  Zarchan's Ch. 2 sizing criterion applied to an mgs run.
%
%   S = mgs.zarchan.sizing(out)              out = mgs.simSwarm(...)
%   S = mgs.zarchan.sizing(out, 'render', true)
%
%   The criterion is the ratio the book keeps returning to: how much normal
%   acceleration the guidance is ASKING for, against how much the airframe can
%   actually produce.  Zarchan's line (Ch. 2, zarchan.txt:539) is that with
%   sufficient acceleration capability there is no miss from heading error at
%   all -- so the interesting quantity is where that "sufficient" runs out.
%
%   S.req_g(i,:)     commanded normal acceleration, g, per logged tick
%   S.avail_g(i,:)   CLmax * qbar * S / (m g), the aerodynamic ceiling
%   S.ratio(i,:)     req / avail
%   S.ratio_max(i)   worst tick of the dive
%   S.ratio_p90(i)   90th percentile -- less hostage to one tick
%   S.binding(i)     fraction of ticks at or over the ceiling
%
%   *** WHAT THIS DOES AND DOES NOT TELL YOU. ***  Stratified by this ratio,
%   the measured 32-flight set splits into two populations that each regress
%   at R^2 0.71-0.73 -- the pooled 0.131 is the artefact of adding them
%   together.  So it ranks WHICH aircraft get into trouble.  It does not say
%   WHY: Zarchan's Ch. 3 lag theory, applied to the same set, has no
%   discriminating power at all (MCC = 0).  Use this to triage, not to
%   diagnose.
%
%   NOTE the ceiling here is AERODYNAMIC (CLmax * q * S / W), which moves with
%   density and V^2.  It is deliberately not the guidance clamp [nz_min,
%   nz_max]: a previous simulator in this project used the guidance limits as
%   the airframe's capability, which makes an aircraft that can always pull,
%   which is an aircraft that always hits.
%
%   See also MGS.SIMSWARM, MGS.ZARCHAN.CROSSCHECK.

arguments
    out
    opts.render (1,1) logical = false
end

c = out.c;
n = numel(out.veh);
S = struct('req_g', {cell(1,n)}, 'avail_g', {cell(1,n)}, 'ratio', {cell(1,n)}, ...
           'ratio_clamp', {cell(1,n)}, ...
           'ratio_max', nan(1,n), 'ratio_p90', nan(1,n), 'binding', nan(1,n), ...
           'clamp_p90', nan(1,n), 'clamp_bind', nan(1,n), 'tag', {cell(1,n)});

for i = 1:n
    T = out.veh(i).traj;
    if isempty(T), continue; end
    V   = T.V;
    z   = T.z;
    gam = deg2rad(T.gam_deg);

    % REQUIRED is the demand BEFORE the clamp.  Using the clamped nz would
    % measure what the vehicle was allowed to do, not what the guidance asked
    % for -- and the whole criterion is about the difference between those.
    if ismember('nz_raw', T.Properties.VariableNames)
        nz_req = T.nz_raw;
    else
        error('mgs:zarchan:sizing:nzraw', ...
            ['this trajectory has no nz_raw column, so the pre-clamp demand ' ...
             'is not available.  Re-run mgs.simSwarm; scoring the clamped nz ' ...
             'instead would silently measure the wrong quantity.']);
    end
    req_g = abs(nz_req - cos(gam));

    % TWO ceilings, because they are wildly different for this vehicle and
    % only one of them binds.
    %   aero  -- CLmax*qbar*S/W, what the airframe can make.  At 157-210 m/s
    %            this is tens of g and is nowhere near binding.
    %   clamp -- the guidance limits [nz_min, nz_max] = [-1, +3].  THIS is the
    %            one that bites, and it bites asymmetrically: the demand has
    %            no lower bound (measured cmd_nz down to -18.28 g, 29.3% of
    %            ticks below -2 g) while nz_min is -1.
    h_amsl = z + c.swarm.tgt_amsl;
    rho  = 1.225 * (1 - 2.25577e-5*max(h_amsl, 0)).^4.2586;
    qbar = 0.5*rho.*V.^2;
    avail_aero = c.b.CLmax .* qbar .* c.b.S ./ c.b.W;
    avail_clamp = zeros(size(req_g)) + max(abs(c.pn.nz_max), abs(c.pn.nz_min));

    ratio       = req_g ./ max(avail_aero, 1e-9);
    ratio_clamp = req_g ./ max(avail_clamp, 1e-9);

    S.req_g{i}       = req_g;
    S.avail_g{i}     = avail_aero;
    S.ratio{i}       = ratio;
    S.ratio_clamp{i} = ratio_clamp;
    S.tag{i}         = out.veh(i).tag;
    S.ratio_max(i)   = max(ratio);
    S.ratio_p90(i)   = prctile_safe(ratio, 90);
    S.binding(i)     = mean(ratio >= 1.0);
    S.clamp_p90(i)   = prctile_safe(ratio_clamp, 90);
    S.clamp_bind(i)  = mean(ratio_clamp >= 1.0);
end

S.summary = render(S, out);
if opts.render, fprintf('%s\n', S.summary); end
end

% -----------------------------------------------------------------------------
function s = render(S, out)
L = {};
L{end+1} = sprintf('Zarchan Ch.2 sizing -- %s', out.tag);
L{end+1} = sprintf('%-14s %11s %11s | %11s %11s | %8s', ...
    'vehicle', 'aero p90', 'aero bind', 'clamp p90', 'clamp bind', 'err m');
for i = 1:numel(S.ratio_max)
    L{end+1} = sprintf('%-14s %11.4f %10.1f%% | %11.3f %10.1f%% | %8.2f', ...
        S.tag{i}, S.ratio_p90(i), 100*S.binding(i), ...
        S.clamp_p90(i), 100*S.clamp_bind(i), out.veh(i).err_m); %#ok<AGROW>
end
L{end+1} = '  required = |nz_raw - cos(gam)|, the demand BEFORE the clamp.';
L{end+1} = '  aero  ceiling = CLmax*qbar*S/W, tens of g at these speeds.';
L{end+1} = '  clamp ceiling = the guidance limits [nz_min, nz_max].';
L{end+1} = '  Read the CLAMP column: on this vehicle the aerodynamic ceiling is';
L{end+1} = '  nowhere near binding, so an aero-only reading of Zarchan Ch.2 says';
L{end+1} = '  everything is fine while the guidance limits do all the biting.';
L{end+1} = '  Ranks who gets into trouble; says nothing about the mechanism.';
s = strjoin(L, newline);
end

function p = prctile_safe(v, q)
v = sort(v(~isnan(v)));
if isempty(v), p = NaN; return; end
if numel(v) == 1, p = v; return; end
idx = 1 + (q/100)*(numel(v)-1);
lo = floor(idx); hi = ceil(idx);
p = v(lo) + (idx-lo)*(v(hi)-v(lo));
end
