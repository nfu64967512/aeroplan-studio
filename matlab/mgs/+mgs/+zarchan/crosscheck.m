function X = crosscheck(out, opts)
%MGS.ZARCHAN.CROSSCHECK  The book as a live reference oracle for an mgs run.
%
%   X = mgs.zarchan.crosscheck(out)                 out = mgs.simSwarm(...)
%   X = mgs.zarchan.crosscheck(out, 'render', true)
%
%   Runs the SAME engagement through Zarchan's Listing 30.1 Pure PN
%   (../zarchan/purepn2d.m -- a line-by-line port that is known to reproduce
%   the book) and reports the gap against our full nonlinear model, together
%   with the clamp fractions that account for it.
%
%   THE GAP IS THE POINT.  These two are not supposed to agree.  purepn2d has
%   no gravity, no drag, no thrust, no lag, a SYMMETRIC acceleration limit,
%   and no LOS floor.  What the comparison buys is a known-correct ideal-loop
%   answer to sit next to ours, so that "our number looks odd" can be split
%   into "the geometry is odd" and "our clamps did it".
%
%   X.book(i)    purepn2d result: .miss_m .gam_f_deg .vm_f_ms .xnc_max_g
%   X.mgs(i)     ours:            .err_m  .gam_impact_deg .v_impact
%   X.clamps(i)  .los_frac .nz_frac -- how much of the dive was NOT PN
%   X.sizing     mgs.zarchan.sizing(out)
%
% ---------------------------------------------------------------------------
%   HOW THE ENGAGEMENT IS MAPPED, AND THE ONE APPROXIMATION IN IT
%
%   The book's Ch. 30 case launches from the ground at a target of altitude
%   RT2M.  Ours is the mirror image: the vehicle starts at z0 and the bullseye
%   is at sea level.  The two are the same problem translated vertically, so
%   RT2M is set to -z0 and the missile starts at zero.
%
%   The APPROXIMATION: purepn2d is planar and our engagement is not.  Release
%   heading offset eps0 puts velocity out of the vertical plane, and there it
%   loads the LATERAL channel (n_lat = 4.0), which the book's single-gain law
%   has no place for.  So the comparison is exact only at eps0 = 0 and is a
%   VERTICAL-CHANNEL comparison otherwise.  X.eps0_deg is reported for every
%   vehicle so the reader can see how far from planar each case is.
%
%   XNCLIMG is set from our nz_max, which is the UP limit.  Ours is asymmetric
%   ([nz_min, nz_max] = [-1, +3] on the RRE contract) and the book's is not;
%   the asymmetry is on the push-over side, which is the side that matters for
%   a steep dive, so this mapping FLATTERS us.  Treat X.book.miss_m as an
%   optimistic reference, not a target.
%
%   See also PUREPN2D, MGS.ZARCHAN.SIZING, MGS.SIMSWARM.

arguments
    out
    opts.render (1,1) logical = false
end

if isempty(which('purepn2d'))
    error('mgs:zarchan:crosscheck:missing', ...
        ['purepn2d not on the path.  It lives in matlab/zarchan/; add it:\n' ...
         '    addpath(fullfile(fileparts(which(''mgs_setup'')), ''..'', ''zarchan''))\n' ...
         'Refusing to substitute an inline PN of my own: the whole value of ' ...
         'that directory is that it is a checked transcription of the book.']);
end

c = out.c;
n = numel(out.veh);
X = struct();
X.tag = out.tag;
X.book = struct('miss_m', {}, 'gam_f_deg', {}, 'vm_f_ms', {}, 'xnc_max_g', {});
X.mgs  = struct('err_m', {}, 'gam_impact_deg', {}, 'v_impact', {});
X.clamps = struct('los_frac', {}, 'nz_frac', {});
X.eps0_deg = nan(1, n);
X.tags = cell(1, n);

for i = 1:n
    ic = out.scn.ic(i);
    X.tags{i} = out.veh(i).tag;
    X.eps0_deg(i) = ic.eps0_deg;

    b = purepn2d( ...
        'ICHOICE', 1, ...                       % command perp. velocity: our law
        'XNP',     c.pn.n_vert, ...             % our vertical gain
        'GAMDEG',  ic.gam0_deg, ...
        'GAMFDEG', -90, ...                     % unused with BIASFORM 'none'
        'XNCLIMG', c.pn.nz_max, ...             % see the header: flatters us
        'VMIC',    ic.V0, ...
        'RANGEM',  ic.r0, ...
        'RT2M',    -ic.z0, ...                  % target z0 BELOW the start
        'BIASFORM', 'none');

    X.book(i).miss_m    = b.miss_m;
    X.book(i).gam_f_deg = b.gam_f_deg;
    X.book(i).vm_f_ms   = b.vm_f_ms;
    X.book(i).xnc_max_g = b.xnc_max_g;

    X.mgs(i).err_m          = out.veh(i).err_m;
    X.mgs(i).gam_impact_deg = out.veh(i).gam_impact_deg;
    X.mgs(i).v_impact       = out.veh(i).v_impact;

    T = out.veh(i).traj;
    X.clamps(i).los_frac = mean(T.los_active, 'omitnan');
    X.clamps(i).nz_frac  = mean(T.nz <= c.pn.nz_min + 1e-9 | ...
                                T.nz >= c.pn.nz_max - 1e-9, 'omitnan');
end

X.sizing = mgs.zarchan.sizing(out);
X.summary = render(X, c);
if opts.render, fprintf('%s\n', X.summary); end
end

% -----------------------------------------------------------------------------
function s = render(X, c)
L = {};
L{end+1} = sprintf('Zarchan cross-check -- %s   (book: Listing 30.1 Pure PN, N''=%g)', ...
    X.tag, c.pn.n_vert);
L{end+1} = repmat('-', 1, 90);
L{end+1} = sprintf('%-12s %8s %8s | %9s %9s | %8s %8s %8s', ...
    'vehicle', 'eps0', 'book m', 'ours m', 'ours gam', 'LOS%', 'nz%', 'req/clamp');
for i = 1:numel(X.tags)
    L{end+1} = sprintf('%-12s %7.1f %8.2f | %9.2f %8.1f  | %7.1f %8.1f %8.3f', ...
        X.tags{i}, X.eps0_deg(i), X.book(i).miss_m, X.mgs(i).err_m, ...
        X.mgs(i).gam_impact_deg, 100*X.clamps(i).los_frac, ...
        100*X.clamps(i).nz_frac, X.sizing.clamp_p90(i)); %#ok<AGROW>
end
L{end+1} = repmat('-', 1, 90);
L{end+1} = 'book m   : Zarchan Listing 30.1, no gravity/drag/thrust/lag, SYMMETRIC limit';
L{end+1} = 'LOS%     : fraction of the dive the one-sided LOS floor was in charge.';
L{end+1} = '           On those ticks the PN loop is OPEN -- gradient is exactly zero.';
L{end+1} = 'nz%      : fraction of the dive the load-factor clamp was on a stop.';
L{end+1} = 'req/clamp: Ch.2 required/available, 90th pct, against the GUIDANCE clamp.';
L{end+1} = '           The aerodynamic ceiling is tens of g here and never binds.';
L{end+1} = '';
L{end+1} = 'A large LOS% is the reason a book comparison stops being meaningful:';
L{end+1} = 'the book has no such term, and an adjoint of a loop that is open half';
L{end+1} = 'the time is not an approximation of that loop, it is a different loop.';
s = strjoin(L, newline);
end
