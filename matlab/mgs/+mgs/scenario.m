function scn = scenario(c, opts)
%MGS.SCENARIO  Build one release state per vehicle.
%
%   scn = mgs.scenario(c)
%   scn = mgs.scenario(c, 'approach_deg', 270, 'tag', 'RRE-like')
%   scn = mgs.scenario(c, 'fromTruth', 'RRE8')     replay a flight's release
%
%   scn.tag          char
%   scn.tgt          .n .e .z  target in the tangent plane (always the origin)
%   scn.n            number of vehicles
%   scn.ic(i)        release state of vehicle i, the same fields stot_sim wants
%                    .tag .r0 .z0 .V0 .gam0_deg .eps0_deg .dive_T .phi0_deg
%                    plus .theta_deg (bearing of the vehicle FROM the target)
%   scn.T_arr        [s] common time over target, or [] for per-vehicle dive_T
%   scn.calibrated   false whenever any leg of this scenario is a PRIOR
%
% ---------------------------------------------------------------------------
%   GEOMETRY.  The bullseye is the origin of a local tangent plane; vehicle i
%   sits at bearing theta_i and range r0_i, and its velocity is offset from
%   the line of sight by eps0_i (positive = velocity right of the LOS).
%
%   Those two -- release RANGE and release HEADING -- are the pair that
%   actually sets dive duration: partial correlations +0.868 and +0.873.  The
%   defaults here are the measured operating point that produced 2.47 s of
%   spread (eps0 25-40 deg, r0 ~ 6760 m), not a round number.
%
%   RELEASE SIGMA IS NOT AN ACCEPTANCE SIGNAL.  Sigma measured AT the release
%   tick has no discriminating power (6.8 deg missed by 648 m; 23.9 deg hit by
%   0.36 m).  What predicts is the history BEFORE release.  If you find
%   yourself reading scn.ic(i).eps0_deg as evidence that a synchroniser
%   worked, stop -- that is a false green light this project has already paid
%   for once.
%
% ---------------------------------------------------------------------------
%   WHAT CHANGES ABOVE n = 2, STATED BECAUSE IT IS EASY TO MISS
%
%   The truth set is n = 2, and n = 2 is not "a swarm of two": it is one
%   leader plus one spare leader.  The programme target is 16 (8 x 2 groups),
%   and at that scale a solution that only works at n = 2 is not a solution.
%   Two specific things stop holding:
%
%     - The PN channel carries no time information: under pure PN against a
%       stationary target d(t_go)/dt = -1 to O(sigma^2), so a release-time
%       difference passes straight through.  (Guo et al., "Data-Driven
%       Method for Impact Time Control Based on PNG", Eq. (33) -- verified
%       verbatim 2026-08-22.  An earlier note here called it unsourced
%       because the PDF is not in the repo; it is in the STOT-plan
%       notebook.  It is also measured here: with no deadline commanded, a
%       release offset comes out AMPLIFIED to 176-219%.)  Our throttle
%       channel does correct it, but only inside a
%       saturation-bounded window -- see mgs.kpi for the measured numbers.
%       This function will happily build a scenario whose spread needs a
%       correction outside that window; mgs.kpi flags it, the builder does not.
%
%     - 3 s of spread is 8 aircraft arriving, not 16.  And per-vehicle
%       touchdown reliability comes first: at the last measurement it was
%       7/14, and spread computed over a population that half misses is not
%       a meaningful number.
%
%   See also MGS.SIMSWARM, MGS.KPI, MGS.PARAMS.

arguments
    c
    opts.approach_deg (1,1) double = 270    % bearing the fan opens towards
    opts.tag (1,:) char = 'mgs'
    opts.fromTruth (1,:) char = ''
    opts.dive_T (1,:) double = []
end

if ~isempty(opts.fromTruth)
    scn = from_truth(c, opts);
    return;
end

n  = c.swarm.n;
s  = c.scen;
scn = struct();
scn.tag = opts.tag;
scn.tgt = struct('n', 0.0, 'e', 0.0, 'z', 0.0);
scn.n   = n;
scn.T_arr = c.swarm.T_arr;
scn.calibrated = true;
scn.notes = {};

% ---- bearings across the fan ----------------------------------------------
half = s.half_angle_deg;
if n == 1
    theta = opts.approach_deg;
else
    switch lower(s.spread_mode)
        case 'even'
            theta = opts.approach_deg + linspace(-half, half, n);
        case 'mirror'
            % pair the vehicles about the centreline: 1 and 2 mirror, 3 and 4
            % mirror, and so on.  This is what a 2-ship actually flew.
            k = ceil((1:n)/2);
            sgn = 1 - 2*mod((1:n)+1, 2);
            theta = opts.approach_deg + sgn .* (half .* k / max(k));
        otherwise
            error('mgs:scenario:spread_mode', ...
                'unknown c.scen.spread_mode "%s" (even | mirror | fromTruth)', ...
                s.spread_mode);
    end
end

% ---- one release state per vehicle ----------------------------------------
if isempty(opts.dive_T)
    if isempty(c.swarm.T_arr)
        % No commanded arrival time: let every vehicle fly its own geometry.
        % The resulting spread is then pure geometry, which is the honest
        % baseline to compare a synchroniser against.
        diveT = nan(1, n);
    else
        diveT = repmat(c.swarm.T_arr, 1, n);
    end
else
    diveT = opts.dive_T(:).';
    if numel(diveT) == 1, diveT = repmat(diveT, 1, n); end
end

ic = struct('tag', {}, 'r0', {}, 'z0', {}, 'V0', {}, 'gam0_deg', {}, ...
            'eps0_deg', {}, 'dive_T', {}, 'phi0_deg', {}, 'theta_deg', {}, ...
            'th0_offset_deg', {}, 't_release', {});

for i = 1:n
    ic(i).tag       = sprintf('%s_U%d', scn.tag, i);
    ic(i).r0        = s.r0;
    ic(i).z0        = s.z0;
    ic(i).V0        = s.V0;
    ic(i).gam0_deg  = -abs(s.dive_angle_deg);
    ic(i).eps0_deg  = eps_for(i, n, s);
    ic(i).dive_T    = diveT(i);
    ic(i).phi0_deg  = 0.0;
    ic(i).theta_deg = theta(min(i, numel(theta)));
    ic(i).th0_offset_deg = 0.0;
    % Time staggering is a RELEASE-TIME offset, applied on the world clock.
    ic(i).t_release = (i-1) * c.swarm.stagger_s;
end
scn.ic = ic;

% ---- honesty bookkeeping ---------------------------------------------------
if ~strcmp(c.up.mode, 'ic')
    scn.calibrated = false;
    scn.notes{end+1} = sprintf(['upstream mode "%s" is uncalibrated; only ' ...
        '"ic" matches the 32-flight truth set'], c.up.mode);
end
if ~strcmp(c.trk.mode, 'analytic')
    scn.calibrated = false;
    scn.notes{end+1} = 'seeker/tracker mode is not the analytic LOS that flew';
end
if n > 2
    scn.notes{end+1} = sprintf(['n = %d; the truth set is n = 2.  The dive ' ...
        'chain per vehicle is still validated, the SWARM behaviour is not.'], n);
end
end

% -----------------------------------------------------------------------------
function e0 = eps_for(i, n, s)
%EPS_FOR  Release heading offset for vehicle i.
%   Sign alternates so the fan opens both ways; magnitude walks across the
%   measured 25-40 deg band, which is the operating point that produced 2.47 s
%   of spread.  A single value for everybody gives identical dive durations
%   and therefore zero geometric spread -- which is a legitimate baseline, so
%   set c.scen.eps0_deg and c.swarm.n = 1 if that is what you want.
if n == 1, e0 = s.eps0_deg; return; end
lo = abs(s.eps0_deg);
hi = lo + 15.0;                     % the measured band is 25-40 deg
mag = lo + (hi-lo) * (i-1)/(n-1);
sgn = 1 - 2*mod(i+1, 2);            % +,-,+,-,...
e0 = sgn * mag;
end

% -----------------------------------------------------------------------------
function scn = from_truth(c, opts)
%FROM_TRUTH  Take the release states of a real sortie out of truth32.csv.
here = fileparts(fileparts(mfilename('fullpath')));   % .../matlab/mgs
csv = fullfile(fileparts(here), 'data', 'truth32.csv');
if ~isfile(csv)
    error('mgs:scenario:truth', ...
        ['%s not found.  Regenerate it with matlab/export_truth32.py, or ' ...
         'drop the fromTruth option.'], csv);
end
T = readtable(csv);
if ~ismember('trial', T.Properties.VariableNames)
    error('mgs:scenario:truthcols', ...
        'truth32.csv has no "trial" column; columns are: %s', ...
        strjoin(T.Properties.VariableNames, ', '));
end
sel = strcmpi(string(T.trial), opts.fromTruth);
if ~any(sel)
    error('mgs:scenario:trial', 'no rows for trial "%s" in %s (have: %s)', ...
        opts.fromTruth, csv, strjoin(unique(cellstr(string(T.trial)))', ', '));
end
R = T(sel, :);

scn = struct();
scn.tag = opts.fromTruth;
scn.tgt = struct('n', 0.0, 'e', 0.0, 'z', 0.0);
scn.n   = height(R);
scn.T_arr = c.swarm.T_arr;
scn.calibrated = true;
scn.notes = {sprintf('release states replayed from %s', csv)};

% ⚠️⚠️ 2026-08-22 FIX.  This block asked for column names truth32.csv has
%    never had, so 'fromTruth' errored for every trial and the replay path was
%    dead.  The guard firing loudly was correct; the names were wrong.
%    The authority for this mapping is matlab/stot_ic32.m (what stot_run32
%    already uses), copied here rather than re-invented.
need = {'r_rel_m', 'z_rel_m', 'V_rel_ms', 'gamma_rel_deg', 'eps_rel_deg', 't_rem0_s'};
miss = need(~ismember(need, R.Properties.VariableNames));
if ~isempty(miss)
    error('mgs:scenario:truthcols', ...
        ['truth32.csv is missing %s.  Refusing to substitute defaults: a ' ...
         'replay that quietly used a default release state is not a replay.'], ...
        strjoin(miss, ', '));
end

% ⚠️⚠️ THE dive_T TRAP.  truth32.csv carries BOTH `dive_T_s` and `t_rem0_s`,
%    and the obvious-looking one is the wrong one:
%       dive_T_s   518.6 .. 669.7 s   ABSOLUTE wall clock of the sortie
%       t_rem0_s    21.4 ..  74.6 s   time remaining AT RELEASE, in SIM time
%    simSwarm's clock starts at 0 with c.sim.t_max = 200, so feeding dive_T_s
%    puts the deadline permanently in the past and the throttle law -- the only
%    impact-time channel there is -- inverts SILENTLY.  stot_ic32.m:33 says so
%    verbatim: "dive_T - now at release, in sim time".
if ismember('dive_T', R.Properties.VariableNames)
    error('mgs:scenario:diveTcolumn', ...
        ['truth32.csv now has a bare `dive_T` column.  Refusing to guess which ' ...
         'clock it is on: `dive_T_s` is an absolute wall clock (518-670 s) and ' ...
         'simSwarm starts at 0 with t_max = %g.  Use t_rem0_s explicitly.'], ...
        c.sim.t_max);
end

for i = 1:scn.n
    scn.ic(i).tag       = sprintf('%s_U%d', opts.fromTruth, i);
    scn.ic(i).r0        = R.r_rel_m(i);
    scn.ic(i).z0        = R.z_rel_m(i);
    scn.ic(i).V0        = R.V_rel_ms(i);
    scn.ic(i).gam0_deg  = R.gamma_rel_deg(i);
    scn.ic(i).eps0_deg  = R.eps_rel_deg(i);
    scn.ic(i).dive_T    = R.t_rem0_s(i);
    scn.ic(i).phi0_deg  = col_or(R, 'phi0_deg', i, 0.0);
    scn.ic(i).theta_deg = opts.approach_deg + (i-1)*10;
    scn.ic(i).th0_offset_deg = 0.0;
    scn.ic(i).t_release = 0.0;
end
end

function v = col_or(T, name, i, dflt)
if ismember(name, T.Properties.VariableNames), v = T.(name)(i); else, v = dflt; end
end
