%DEMO_SHADOW  Where does the model part company with the vehicle?
%
%   Run:  mgs_setup; demo_shadow
%
%   Part 1 needs nothing but MATLAB -- it exercises the diff against synthetic
%   streams whose answer is known.  Part 2 needs the live WSL stack and is
%   skipped with a message when it is not there.

mgs_setup();
sp = tempdir;
c = mgs.params();

%% 1 -- the tool, checked against known answers ----------------------------
fprintf('\n========== 1. null case: a stream from this very model ==========\n');
fprintf('If this shows ANY divergence, the frame conversion, the resampling\n');
fprintf('or the guidance replay is broken and no later verdict is worth\n');
fprintf('anything.  It must come out "none".\n\n');
f0 = fullfile(sp, 'demo_shadow_null.ndjson');
mgs.synthStream(f0, c, 'inject', 'none', 'hz', 1/c.pn.tick_dt);
mgs.shadowDiff(f0, c, 'render', true);

fprintf('\n========== 2. known injections, one layer at a time =============\n');
for lay = {'guidance', 'clamp', 'plant'}
    Ln = lay{1};
    f = fullfile(sp, ['demo_shadow_' Ln '.ndjson']);
    mgs.synthStream(f, c, 'inject', Ln, 'at_s', 15, 'deg', 6, ...
                    'hz', 1/c.pn.tick_dt);
    D = mgs.shadowDiff(f, c, 'render', false);
    fprintf('  injected into %-9s -> attributed to %-24s', Ln, D.attribution);
    for k = 1:numel(D.layer)
        if D.layer(k).avail && isfinite(D.layer(k).t_first)
            fprintf(' [%s t+%.1f]', strtok(D.layer(k).name), D.layer(k).t_first);
        end
    end
    fprintf('\n');
end
fprintf(['\n  Note the order, not the silence: a clamp injection also moves L1,\n' ...
         '  because the law reads measured pitch for its alpha estimate.  It\n' ...
         '  moves it 0.1 s LATER, and that is what the attribution uses.\n']);

fprintf('\n========== 3. the rate requirement, measured ====================\n');
fprintf('%-10s %-12s\n', 'record Hz', 'L1 residue');
for hz = [10 20 50]
    f = fullfile(sp, sprintf('demo_shadow_hz%d.ndjson', hz));
    mgs.synthStream(f, c, 'inject', 'none', 'hz', hz);
    w = warning('off', 'mgs:shadowDiff:rate');
    D = mgs.shadowDiff(f, c, 'render', false);
    warning(w);
    fprintf('%-10d %-12.4f %s\n', hz, D.layer(1).rms, ...
        ternary(hz == 20, '<- the node ticks at 20 Hz', ...
                'a different alpha low-pass, so this residue is OURS'));
end

%% 2 -- against the live stack ---------------------------------------------
fprintf('\n========== 4. the live stack ====================================\n');
rep = mgs.preflight('verbose', false);
if ~rep.pass
    fprintf(['  preflight is not green, so there is nothing live to shadow.\n' ...
             '  Start the WSL stack and the bridge, then re-run:\n' ...
             '      source /opt/ros/humble/setup.bash\n' ...
             '      python3 matlab/mgs/bridge/mgs_bridge_node.py --state-hz 20\n']);
    return;
end
fprintf(['  preflight is green.  To capture a sortie and diff it:\n' ...
         '      mgs.recordStream(c, ''runs/shadow/RRX.ndjson'', ''seconds'', 900);\n' ...
         '      D = mgs.shadowDiff(''runs/shadow/RRX.ndjson'', c, ''freerun'', true);\n' ...
         '  Record BEFORE you fly -- the recorder only captures what arrives\n' ...
         '  while it is running, and a sortie cannot be replayed.\n']);

function v = ternary(tf, a, b)
if tf, v = a; else, v = b; end
end
