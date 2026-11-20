function out = trialArtifacts(tag, opts)
%MGS.TRIALARTIFACTS  The standard visual set for one flown sortie.
%
%   mgs.trialArtifacts("RA1")
%   mgs.trialArtifacts("SB3", focus="all")
%   mgs.trialArtifacts("WV1", mp4=false)
%
%   Writes into runs/artifacts/<tag>/ :
%       <tag>_dash.png     full dashboard  (3D pose + gamma/theta, n_z, speed)
%       <tag>_engage.png   plain-background 3D  (the report main visual)
%       <tag>_engage.mp4   the same view, animated
%
% ---------------------------------------------------------------------------
%   ⚠️⚠️ WHY THIS FILE EXISTS AT ALL.
%
%   LAT1's and RB1's artifacts were produced from an ad-hoc command line that
%   was never written down, so the next seven sorties (RA1, RC1, RD1, RE21,
%   SB1-3) shipped with a CSV and no picture at all.  This repo has paid for
%   that exact shape before -- run_west.sh's own header records that it "only
%   existed in a temporary shell, the handover document referenced it, and
%   nobody had put it in the repo", taking phase_report.py / coupling.py /
%   cmd_speed_check.py with it.  A recipe that lives only in scrollback is not
%   a recipe.
%
%   ⚠️ The per-sortie CSVs under runs/artifacts/<tag>/ are ARCHIVED (the live
%   ones in fixwing_ws/runs/ are overwritten by the next sortie), so this can
%   be run retroactively.  That is the only reason the backfill was possible.
%
% ---------------------------------------------------------------------------
%   CHOOSING THE FREEZE FRAME
%
%   Not the last frame: by then every vehicle has hit, the HUD is all "--" and
%   the trails are empty.  Not the middle of the window either -- the window
%   ends at the LAST impact, so the midpoint can still be after the first
%   vehicle is gone.  The rule here is 60% of the way to the FIRST impact, so
%   every drawn vehicle is guaranteed to still be flying.
%
%   See also MGS.VIZ6DOF, MGS.REPOROOT.

arguments
    tag          (1,1) string
    opts.dir     (1,1) string  = ""      % override runs/artifacts/<tag>
    opts.focus                 = "dive"
    opts.at      (1,1) double  = NaN     % override the freeze frame
    opts.model   (1,1) string  = "cad"
    opts.mp4     (1,1) logical = true
    opts.speed   (1,1) double  = 4
end

root = mgs.repoRoot();
d = opts.dir;
if d == "", d = fullfile(root, 'runs', 'artifacts', char(tag)); end
csv = fullfile(char(d), sprintf('tot_%s.csv', tag));
if ~isfile(csv)
    error('mgs:trialArtifacts:noCsv', ...
        ['no flight log at %s.\n' ...
         '    ⚠️ this function reads the ARCHIVED per-sortie csv, not the live\n' ...
         '    one in fixwing_ws/runs/ (that one is overwritten every sortie).'], csv);
end

% ── freeze frame: 60% of the way to the FIRST impact ─────────────────────
A = mgs.viz6dof(csv, show=false, focus=opts.focus, model="glyph", export="none", at=NaN, probe=true);
t0 = min(arrayfun(@(v) v.t(1), A.veh));
% ⚠️⚠️ NOT min(v.t(end)).  apply_focus trims every vehicle to the SAME window,
%    so all the t arrays end at the same number and that expression silently
%    returns the window edge -- it looks like it works and it never picks the
%    first impact.  The logs also keep running after touchdown (the row stream
%    does not stop at impact), so "last sample" is not "last airborne" either.
%    Take the last sample each vehicle is still ABOVE the deck instead.
AIRBORNE_M = 50;
tE = inf;
for i = 1:numel(A.veh)
    k = find(A.veh(i).z > AIRBORNE_M, 1, 'last');
    if isempty(k), continue; end
    tE = min(tE, A.veh(i).t(k));
end
if ~isfinite(tE), tE = max(arrayfun(@(v) v.t(end), A.veh)); end
at = opts.at;
if ~isfinite(at), at = t0 + 0.60 * (tE - t0); end

out = struct('tag', tag, 'csv', csv, 'at', at, 'files', {{}});
% ⚠️ assigned after construction: struct() with a string array in braces
%    would build a struct ARRAY, not one struct with a string field.
out.dropped = A.dropped;

% ── headline number on the figure itself ─────────────────────────────────
%   ⚠️⚠️ This is the LOG-END basis: the last logged sample of each vehicle's
%   dive/impact segment.  It is NOT stot_score.py's basis, which interpolates
%   the point where the trajectory crosses AGL = 0 -- and that crossing is not
%   in these archived CSVs at all (they stop at 11-19 m AGL, because logging
%   ends when impact is detected).  The two agree to within 0.1 s on all nine
%   sorties checked (RA1 33.30/33.30, RC1 9.00/8.98, RD1 7.60/7.60,
%   RE21 3.00/3.08, SB1 5.60/5.55, SB2 9.20/9.35, RB1 1.70/1.74,
%   LAT1 16.80/16.80), but "agrees closely" is not "is the same quantity".
%   The basis is printed in the title so a screenshot cannot be quoted as the
%   scorer's number -- mixing two altitude bases silently is the exact failure
%   stot_score.py's own comments were written to stop.
%
%   No PASS/FAIL here either: the >=3 s criterion is defined over 6-8 aircraft
%   (memory reliability-before-spread), and these are n=2 sorties.
[spread_s, nveh] = logend_spread(csv);
ttl = sprintf('%s   n=%d', tag, nveh);
if isfinite(spread_s)
    ttl = sprintf('%s   touchdown spread %.2f s (log-end basis)', ttl, spread_s);
end
out.spread_s = spread_s;

png_dash = fullfile(char(d), sprintf('%s_dash.png', tag));
png_eng  = fullfile(char(d), sprintf('%s_engage.png', tag));
mp4_eng  = fullfile(char(d), sprintf('%s_engage.mp4', tag));

mgs.viz6dof(csv, show=false, focus=opts.focus, model=opts.model, style="dash", ...
            at=at, export="png", file=png_dash, title=ttl);
out.files{end+1} = png_dash;

mgs.viz6dof(csv, show=false, focus=opts.focus, model=opts.model, style="simple", ...
            at=at, export="png", file=png_eng, title=ttl);
out.files{end+1} = png_eng;

if opts.mp4
    mgs.viz6dof(csv, show=false, focus=opts.focus, model=opts.model, style="simple", ...
                export="mp4", file=mp4_eng, speed=opts.speed, title=ttl);
    out.files{end+1} = mp4_eng;
end

fprintf('%-6s  at %.1f s  spread %.2f s  ->  %d files\n', ...
        tag, at, spread_s, numel(out.files));
if ~isempty(out.dropped)
    fprintf(['  ⚠️ %s absent from the focus window -- the figure says so in its ' ...
             'title and ledger, but read it before quoting the picture.\n'], ...
            strjoin(cellstr(out.dropped), ', '));
end
end

% =============================================================================
function [spread, nveh] = logend_spread(csv)
%LOGEND_SPREAD  max-min of the last logged dive/impact sample time, per vehicle.
%   ⚠️ See the caller's comment: this is the LOG-END basis, not stot_score.py's
%      AGL=0 crossing.  Returns NaN when fewer than two vehicles ever dived --
%      a single-diver sortie has no spread, and reporting 0 would read as
%      "perfectly synchronised" when what actually happened is that one ship
%      never got there (SB3).
T = readtable(csv, 'TextType', 'string', 'VariableNamingRule', 'preserve');
need = ["t", "sysid", "mode"];
miss = need(~ismember(need, string(T.Properties.VariableNames)));
if ~isempty(miss)
    error('mgs:trialArtifacts:cols', 'csv is missing column(s): %s', strjoin(miss, ', '));
end
m = ismember(string(T.mode), ["dive", "impact"]);
nveh = numel(unique(T.sysid));
ids = unique(T.sysid(m));
if numel(ids) < 2, spread = NaN; return; end
ends = arrayfun(@(k) max(T.t(m & T.sysid == k)), ids);
spread = max(ends) - min(ends);
end
