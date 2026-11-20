function ok = selftest(opts)
%MGS.SELFTEST  Everything that would silently rot if nobody checked it.
%
%   mgs.selftest()                 run all checks
%   ok = mgs.selftest('slx', true) include the Simulink comparison (slow)
%
%   Each check exists because of a specific way this environment could stop
%   meaning what it claims to mean, and each one names that way.
%
%   See also MGS.SIMSWARM, MGS.PREFLIGHT.

arguments
    opts.slx (1,1) logical = false
    opts.verbose (1,1) logical = true
    % ⚠️ 預設開:沒人跑的基準套件會靜靜腐爛。慢(30-60 s)但值得。
    opts.zarchan (1,1) logical = true
end

R = {};

% =========================================================================
% 1  mgs.simSwarm at n = 1 IS stot_sim, to the bit.
%    If this drifts, every claim that the swarm environment inherits the
%    32-flight validation is void -- and it would drift silently, because
%    both sides would still produce plausible trajectories.
% =========================================================================
ic = struct('tag', 'PAR', 'r0', 6760, 'z0', 3100, 'V0', 157, ...
            'gam0_deg', -27, 'eps0_deg', 25, 'dive_T', 40, ...
            'phi0_deg', 0, 'th0_offset_deg', 0);
ref = stot_sim(ic, cfg.params());
c1 = mgs.params('swarm.n', 1);
scn = mgs.scenario(c1, 'tag', 'PAR');
scn.ic(1).dive_T = 40; scn.ic(1).eps0_deg = 25; scn.ic(1).theta_deg = 0;
o1 = mgs.simSwarm(scn, c1, 'verbose', false);
dt = abs(o1.veh(1).t_impact - ref.t_dive);
de = abs(o1.veh(1).err_m   - ref.err_touch);
dv = abs(o1.veh(1).v_impact - ref.V_touch);
R{end+1} = chk('simSwarm(n=1) == stot_sim', max([dt de dv]) == 0, ...
    sprintf('dt %.3g  derr %.3g  dV %.3g', dt, de, dv));

% =========================================================================
% 2  The Zarchan additive law reduces EXACTLY to the flight law when its
%    extra term is switched off.  Two copies of a 100-line guidance law
%    exist in this repository; this is the only thing stopping them from
%    drifting apart while both keep running.
% =========================================================================
pn = c1.pn;  pn.los_floor = 0.0;  pn.shape_gain = 0.0;
m = struct('n', 5000, 'e', -1200, 'z', 2600, 'vn', -130, 've', 40, ...
           'vd', 60, 'pitch', deg2rad(-22));
gsA = mgs.guidance();  gsA.dive_T = 40;
gsB = gsA;
[cA, ~, dA] = stot_guidance(m, gsA, 12.0, pn);
[cB, ~, dB] = mgs.zarchan.additiveLaw(m, gsB, 12.0, pn);
d1 = max(abs([cA.roll - cB.roll, cA.pitch - cB.pitch, cA.thr - cB.thr]));
d2 = max(abs([dA.nz - dB.nz, dA.gam_cmd - dB.gam_cmd, dA.t_go - dB.t_go]));
R{end+1} = chk('zarchan additive == node law when the term is off', ...
    max(d1, d2) == 0, sprintf('cmd %.3g  dbg %.3g', d1, d2));

% =========================================================================
% 3  The additive term actually DOES something when it is on.  A knob that
%    is wired up but inert reads exactly like a knob that works, and this
%    project has shipped one of those before (itcg_accel: 80,760 samples,
%    all zero, never carried a signal).
% =========================================================================
pnC = c1.pn;  pnC.los_floor = 0.0;  pnC.shape_gain = 2.0;
[cC, ~, dC] = mgs.zarchan.additiveLaw(m, gsB, 12.0, pnC);
moved = abs(cC.pitch - cB.pitch);
R{end+1} = chk('additive term is not inert', moved > 1e-6, ...
    sprintf('pitch moved %.4f deg with shape_gain 2.0', rad2deg(moved)));

% =========================================================================
% 4  The one-sided min() really is one-sided.  With the floor active, a
%    sweep of the PN gain must NOT move the command -- that zero gradient is
%    the whole reason the additive form is worth studying, so if this check
%    ever passes trivially the argument has lost its basis.
% =========================================================================
pnD = c1.pn; pnD.los_floor = 1.0;
% The floor bites when the sightline is STEEPER than anything the nz clamp
% lets PN ask for: close in horizontally, still high.  d = 800 m, h = 2600 m
% gives gam_los = -72.9 deg, while the clamp caps the PN demand near -32 deg.
mD = struct('n', 800, 'e', 0, 'z', 2600, 'vn', -130, 've', 0, ...
            'vd', 60, 'pitch', deg2rad(-22));
g1 = mgs.guidance(); g1.dive_T = 40;
[~, ~, e1] = stot_guidance(mD, g1, 12.0, pnD);
pnD2 = pnD; pnD2.n_vert = pnD.n_vert * 2;
g2 = mgs.guidance(); g2.dive_T = 40;
[~, ~, e2] = stot_guidance(mD, g2, 12.0, pnD2);
R{end+1} = chk('LOS floor opens the loop (min() has zero gradient)', ...
    e1.los_active && e2.los_active && abs(e1.gam_cmd - e2.gam_cmd) < 1e-12, ...
    sprintf('floor active %d/%d, gam_cmd moved %.3g rad on a 2x n_vert sweep', ...
            e1.los_active, e2.los_active, abs(e1.gam_cmd - e2.gam_cmd)));

% =========================================================================
% 5  A typo in a parameter override is an ERROR, not a no-op.
% =========================================================================
threw = false;
try
    mgs.params('swarm.nn', 8);
catch
    threw = true;
end
R{end+1} = chk('typo in an override raises', threw, ...
    'mgs.params(''swarm.nn'', 8) must not silently create a field');

% =========================================================================
% 6  The uncalibrated gamma deficit refuses to run unacknowledged.
% =========================================================================
threw = false;
try
    cg = mgs.params('swarm.n', 1, 'gam.deficit_on', true);
    mgs.simSwarm(mgs.scenario(cg), cg, 'verbose', false);
catch ME
    threw = strcmp(ME.identifier, 'mgs:autopilot:gam_uncalibrated');
end
R{end+1} = chk('uncalibrated gamma deficit is gated', threw, ...
    'deficit_on without acknowledge_uncalibrated must raise');

% =========================================================================
% 7  The contract table is not a hand copy: its sha1 matches its source.
% =========================================================================
try
    T = mgs.contractTable(true);
    R{end+1} = chk('contract table matches contract_table.py', true, ...
        sprintf('%d contracts, sha1 %s', T.meta.n_contracts, ...
                T.meta.source_sha1(1:12)));
catch ME
    R{end+1} = chk('contract table matches contract_table.py', false, ME.message);
end

% =========================================================================
% 8  A foam-era value is caught even when it sits inside the allowed band.
%    This is the check the range test cannot do.
% =========================================================================
rep = mgs.contractCheck(struct('dive_angle', 25.0), 'strict', false);
caught = any(strcmp({rep.violations.kind}, 'foam'));
R{end+1} = chk('foam value caught inside its own band', caught, ...
    'dive_angle = 25 (foam) sits inside the jet band and must still be flagged');

% =========================================================================
% 9a  THE SHADOW DIFF NULL CASE.  A stream generated by this very model must
%     produce NO divergence in any layer.  If it does, the frame conversion,
%     the resampling or the guidance replay is broken, and every attribution
%     the tool ever makes is worthless.  This check exists because the first
%     version failed it: the guidance replay ran at the recording rate, and
%     the alpha low-pass gain is tick_dt/alpha_tau, so any rate but 20 Hz is
%     a different filter.
% =========================================================================
sp = tempdir;
c1 = mgs.params();
try
    f0 = fullfile(sp, 'mgs_selftest_null.ndjson');
    mgs.synthStream(f0, c1, 'inject', 'none', 'hz', 1/c1.pn.tick_dt);
    D0 = mgs.shadowDiff(f0, c1, 'render', false);
    rmsAll = [D0.layer.rms];
    R{end+1} = chk('shadowDiff null case shows no divergence', ...
        strcmp(D0.attribution, 'none'), ...
        sprintf('attribution "%s"; layer rms %s', D0.attribution, ...
                mat2str(round(rmsAll, 3))));
catch ME
    R{end+1} = chk('shadowDiff null case shows no divergence', false, ME.message);
end

% =========================================================================
% 9b  ... and it attributes a KNOWN injection to the RIGHT layer.  A tool
%     whose whole job is attribution has to be shown to get a known answer
%     right; "it ran" is not evidence.
% =========================================================================
want = {'guidance', 'L1'; 'clamp', 'L2'; 'plant', 'L3'};
okAll = true;  detail = {};
for q = 1:size(want, 1)
    try
        fq = fullfile(sp, ['mgs_selftest_' want{q,1} '.ndjson']);
        mgs.synthStream(fq, c1, 'inject', want{q,1}, 'at_s', 15, 'deg', 6, ...
                        'hz', 1/c1.pn.tick_dt);
        Dq = mgs.shadowDiff(fq, c1, 'render', false);
        got = strtok(Dq.attribution);
        okAll = okAll && strcmp(got, want{q,2});
        detail{end+1} = sprintf('%s->%s', want{q,1}, got); %#ok<AGROW>
    catch ME
        okAll = false;
        detail{end+1} = sprintf('%s->ERROR %s', want{q,1}, ME.identifier); %#ok<AGROW>
    end
end
R{end+1} = chk('shadowDiff attributes a known injection', okAll, ...
    strjoin(detail, ', '));

% =========================================================================
% 9  Zarchan's terminal-angle term must be a SETPOINT, not an attractor.
%    ⚠️⚠️ This is the check that would have caught the 2026-08-22 defect.  The
%    first import wrote the shaping error as (sightline - CURRENT gamma) instead
%    of Zarchan's (sightline - DEMANDED gamF, zarchan.txt:10942, a constant).
%    It ran, it passed the parity test, it passed "the term is not inert", and
%    it was swept for a whole session -- while being an attractor with no
%    impact-angle demand at all, and carrying the OPPOSITE SIGN on 32 of 32
%    measured release states.
%    Two properties pin it, and both are structural rather than numerical:
%      (a) the term must not depend on the current gamma at all;
%      (b) at our release geometry it must be POSITIVE, i.e. it lofts first --
%          which is how Zarchan's own Listing 30.1 reaches gamF = -90 deg.
cz = mgs.params('swarm.n', 1);
cz.pn.law = 'zarchan_additive'; cz.pn.los_floor = 0.0; cz.pn.shape_gain = 2.0;
% 目標在原點;機體在南方 5 km、高 2500 m,向北飛。
mz = struct('n', -5000, 'e', 0, 'z', 2500, 'vn', 150, 've', 0, 'vd', 20, ...
            'pitch', 0.0);
gz = mgs.guidance();  gz.phase = 'dive';  gz.dive_T = 40;
% (a) 幾何與 |V| 完全相同、只有 gamma 不同 ——
%     ⚠️ 必須固定 |V|:項是 shape_gain*(lam-gamF)/t_go 而 t_go = R/V,
%     所以只改 vd 會經由 t_go 動到結果,那是**合法的**(Zarchan 的
%     TGO = RTM/VC 一樣依賴速度)。要隔離的是**誤差項**不是整個乘積。
Vfix = hypot(150, 20);
sh = zeros(1, 2);  vds = [20, 140];
for q = 1:2
    mq = mz;  mq.vd = vds(q);
    mq.vn = sqrt(max(Vfix^2 - vds(q)^2, 1));      % 同一個 |V|,不同 gamma
    [~, ~, dz] = mgs.guidance(mq, gz, 0, cz);
    sh(q) = dz.gdot_shape;
end
indep = abs(sh(1) - sh(2)) < 1e-12;
% (b) sign at the real release geometry
sgn_ok = sh(1) > 0;
R{end+1} = chk('zarchan term is a setpoint, not a gamma attractor', ...
    indep && sgn_ok, ...
    sprintf(['at fixed |V|, d(shape)/d(gamma) == 0 (%.6e vs %.6e) and the ' ...
             'term LOFTS (+%.4f rad/s at the release geometry)'], ...
             sh(1), sh(2), sh(1)));

% =========================================================================
% 10  An UNFLOWN law must report itself as uncalibrated.
%    ⚠️⚠️ This was PROMISED IN TWO HEADERS and not implemented until
%    2026-08-22: mgs.zarchan.additiveLaw says "mgs.simSwarm marks any run
%    using it uncalibrated" and mgs.guidance says the same about the 'hold'
%    leg, while simSwarm only ever checked gam.deficit_on and trk.mode.
%    A run on a law that has never flown reported calibrated = 1.  That is a
%    false green light inside the very mechanism built to prevent them.
cu = mgs.params('swarm.n', 2);
cu.pn.law = 'zarchan_additive';  cu.pn.los_floor = 0.0;  cu.pn.shape_gain = 2.0;
ou = mgs.simSwarm(mgs.scenario(cu), cu, 'verbose', false);
R{end+1} = chk('an unflown law reports itself uncalibrated', ...
    ~ou.calibrated && ~isempty(ou.notes), ...
    'c.pn.law ~= "node" has no truth data behind it; calibrated must go false and say why');

% =========================================================================
% 11  viz6dof refuses to draw an attitude it cannot support.
%    ⚠️⚠️ The visualisation is the one place where a fabricated number does
%    not look like a number -- it looks like a picture, and nobody re-derives
%    a picture.  So the check is not "does it run": it is "does the phi
%    reconstruction VETO ITSELF on data where it is known to be wrong".
%    On BASE2's dive it reconstructs a median 56 deg of bank against a
%    node-commanded median of 0.93 deg over 32 sorties; the guard must catch
%    that, zero the channel, and say so in the ledger.
try
    fcsv = fullfile(mgs.repoRoot(), 'runs', 'artifacts', 'BASE21', 'tot_BASE21.csv');
    if isfile(fcsv)
        Av = mgs.viz6dof(fcsv, 'at', 595, 'show', false);
        close(Av.fig);
        rr = find(strcmp(Av.prov(:,1), 'phi'), 1);
        vetoed = strcmp(Av.phi_kind, 'none') && strcmp(Av.prov{rr,2}, 'NONE');
        R{end+1} = chk('viz6dof vetoes an unsupportable attitude', vetoed, ...
            'BASE2 has no logged roll; the coordinated-turn reconstruction must be rejected, not drawn');
    else
        R{end+1} = skip('viz6dof vetoes an unsupportable attitude', ...
                        'runs/artifacts/BASE21 not in this checkout');
    end
catch ME
    R{end+1} = chk('viz6dof vetoes an unsupportable attitude', false, ME.message);
end

% ── Zarchan book benchmarks ──────────────────────────────────────────────
% ⭐⭐ The only checks in this file with an EXTERNAL oracle.  Everything above
%    compares mgs against mgs (or against a flight log we also produced); these
%    compare five ported listings against numbers Paul Zarchan published, which
%    is the one thing that can tell us the guidance kernel is right rather than
%    self-consistent.
%
% ⚠️ Two rows come back REFUTED by design -- the port disproves a value the
%    commissioning brief carried (the brief was assembled from a summary, not
%    from the book).  nfail counts only scored FAILs, so REFUTED and INFO rows
%    do not turn this red; that is deliberate.  Read the board with
%    mgs.zarchan.benchSuite() when this line goes red.
%
% ⚠️ ~30-60 s, dominated by weaveDesignCurve's 40-point sweep.  It is opt-out
%    (pass 'zarchan', false) rather than opt-in: a benchmark suite nobody runs
%    is a benchmark suite that silently rots.
if opts.zarchan
    try
        [zrows, znfail] = mgs.zarchan.benchSuite('render', false);
        R{end+1} = chk('zarchan book benchmarks', znfail == 0, ...
            sprintf('%d rows, %d scored FAIL, %d PASS', ...
                    numel(zrows), znfail, sum([zrows.ok])));
    catch ME
        R{end+1} = chk('zarchan book benchmarks', false, ME.message);
    end
else
    R{end+1} = skip('zarchan book benchmarks', "pass 'zarchan', true to run it");
end

% =========================================================================
% 12  Simulink and the MATLAB loop agree to a STATED tolerance.  They are
%    different integrators and will never be identical; the point is that
%    the difference is bounded and known, not assumed away.
% =========================================================================
if opts.slx
    try
        c2 = mgs.params('swarm.n', 2);
        s2 = mgs.scenario(c2);
        oM = mgs.simSwarm(s2, c2, 'verbose', false);
        mdl = mgs.slx.build(c2, s2, 'stopTime', 50);
        so = sim(mdl);
        X = so.mgs_log_X;  v = X.signals.values;  tt = X.time(:);
        dmax = 0;
        for i = 1:2
            z = squeeze(v(i,3,:));
            k = find(z <= 0, 1, 'first');
            a = (0 - z(k-1))/(z(k) - z(k-1));
            tc = tt(k-1) + a*(tt(k) - tt(k-1));
            dmax = max(dmax, abs(tc - oM.veh(i).t_impact));
        end
        R{end+1} = chk('Simulink vs MATLAB loop within 5 ms', dmax < 5e-3, ...
            sprintf('max touchdown-time difference %.4f s', dmax));
    catch ME
        R{end+1} = chk('Simulink vs MATLAB loop within 5 ms', false, ME.message);
    end
else
    R{end+1} = skip('Simulink vs MATLAB loop', 'pass ''slx'', true to run it');
end

% ---- report ----------------------------------------------------------------
ok = all(cellfun(@(r) r.ok || r.skipped, R));
if opts.verbose
    fprintf('mgs selftest\n%s\n', repmat('-', 1, 74));
    for k = 1:numel(R)
        r = R{k};
        if r.skipped,  mark = 'SKIP';
        elseif r.ok,   mark = 'PASS';
        else,          mark = 'FAIL';
        end
        fprintf('[%s] %-46s %s\n', mark, r.name, r.detail);
    end
    nf = sum(cellfun(@(r) ~r.ok && ~r.skipped, R));
    fprintf('%s\n%d checks, %d failed\n', repmat('-', 1, 74), numel(R), nf);
end
end

% =============================================================================
function r = chk(name, ok, detail)
r = struct('name', name, 'ok', logical(ok), 'skipped', false, 'detail', detail);
end

function r = skip(name, why)
r = struct('name', name, 'ok', false, 'skipped', true, 'detail', why);
end
