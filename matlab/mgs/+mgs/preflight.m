function rep = preflight(c, opts)
%MGS.PREFLIGHT  Check everything the interface depends on, and say what is wrong.
%
%   mgs.preflight()                  check with the default parameters
%   rep = mgs.preflight(c)           ... and get the result back
%   rep = mgs.preflight(c, 'strict', true)   abort on the first hard failure
%
%   Seven checks, in the order that a failure in one makes the next
%   uninterpretable:
%
%     1  the validated single-vehicle chain is on the path
%     2  contract_table.json exists and matches contract_table.py
%     3  WSL is installed and a distro is running
%     4  ROS 2 is present in that distro, and which distro/domain
%     5  WSL networking mode -- NAT or mirrored.  This decides whether the
%        'ros2' transport can work at all, and it is invisible otherwise.
%     6  the bridge port is listening
%     7  a full hello handshake against the bridge
%
%   Every failure prints the command that fixes it.  A check that cannot be
%   run is reported as 'skipped', never as 'pass' -- an unrun check that reads
%   green is worse than a red one.
%
%   See also MGS.BRIDGE, MGS.CONTRACTTABLE.

arguments
    c = mgs.params()
    opts.strict (1,1) logical = false
    opts.verbose (1,1) logical = true
end

R = {};

% ---- 1 ---------------------------------------------------------------------
need = {'stot_guidance', 'stot_fcs_clamp', 'stot_plant'};
miss = need(cellfun(@(f) isempty(which(f)), need));
R{end+1} = chk('validated chain on path', isempty(miss), ...
    ternary(isempty(miss), 'stot_guidance / stot_fcs_clamp / stot_plant found', ...
            sprintf('missing: %s', strjoin(miss, ', '))), ...
    'run mgs_setup from matlab/mgs');

% ---- 2 ---------------------------------------------------------------------
try
    T = mgs.contractTable(true);
    R{end+1} = chk('contract table fresh', true, ...
        sprintf('%d contracts, %d invariants, sha1 %s', ...
                T.meta.n_contracts, T.meta.n_invariants, T.meta.source_sha1(1:12)), '');
catch ME
    R{end+1} = chk('contract table fresh', false, ME.message, ...
        'python matlab/mgs/bridge/export_contract_table.py');
end

% ---- 3 ---------------------------------------------------------------------
% NOT `wsl -l -v`: it writes UTF-16 to a console handle and comes back as one
% byte when redirected, so the check would read as a failure on a perfectly
% healthy machine.  Probe through bash instead, which redirects properly.
% NOTE this STARTS the distro if it was stopped -- that is deliberate for a
% preflight, but it does mean the check is not purely passive.
[st, out] = runcmd(['wsl.exe -e bash -lc "echo WSL_OK; ' ...
                    'grep -m1 PRETTY_NAME /etc/os-release | cut -d= -f2"']);
wslUp = st == 0 && contains(clean(out), 'WSL_OK');
R{end+1} = chk('WSL distro reachable', wslUp, ...
    ternary(wslUp, strtrim(strrep(clean(out), 'WSL_OK', '')), clean(firstlines(out, 3))), ...
    'start the distro from a Windows terminal:  wsl -d Ubuntu-22.04');

% ---- 4 ---------------------------------------------------------------------
if wslUp
    [st4, out4] = runcmd('wsl.exe -e bash -lc "ls /opt/ros 2>/dev/null; echo DOMAIN=$ROS_DOMAIN_ID"');
    distro = strtrim(regexprep(clean(out4), 'DOMAIN=.*', ''));
    R{end+1} = chk('ROS 2 in WSL', st4 == 0 && ~isempty(distro), ...
        sprintf('/opt/ros -> "%s"', distro), ...
        'install ROS 2, or source /opt/ros/<distro>/setup.bash');
else
    R{end+1} = skip('ROS 2 in WSL', 'WSL is not running');
end

% ---- 5  networking mode ----------------------------------------------------
% This is the check that saves a session.  In NAT mode DDS discovery does not
% cross into Windows and NOTHING reports an error -- ROS Toolbox just receives
% nothing, forever.
if wslUp
    [~, ipOut] = runcmd('wsl.exe -e bash -lc "hostname -I"');
    wslIp = strtrim(clean(ipOut));
    cfgPath = fullfile(getenv('USERPROFILE'), '.wslconfig');
    mirrored = false;
    if isfile(cfgPath)
        txt = lower(fileread(cfgPath));
        mirrored = contains(txt, 'networkingmode') && contains(txt, 'mirrored');
    end
    if mirrored
        detail = sprintf('mirrored (.wslconfig), WSL IP %s', wslIp);
    else
        detail = sprintf(['NAT (no mirrored setting in %s), WSL IP %s -- ' ...
            'the "ros2" transport CANNOT see WSL topics in this mode'], ...
            cfgPath, wslIp);
    end
    wantRos2 = strcmpi(c.io.transport, 'ros2');
    R{end+1} = chk('WSL networking mode', mirrored || ~wantRos2, detail, ...
        sprintf(['either use c.io.transport = ''tcp'' (recommended here), or ' ...
                 'put\n        [wsl2]\n        networkingMode=mirrored\n' ...
                 '      in %s and run  wsl --shutdown'], cfgPath));
else
    R{end+1} = skip('WSL networking mode', 'WSL is not running');
end

% ---- 6  port ---------------------------------------------------------------
if strcmpi(c.io.transport, 'tcp')
    if wslUp
        [~, ssOut] = runcmd(sprintf( ...
            'wsl.exe -e bash -lc "ss -tln 2>/dev/null | grep -c :%d"', c.io.port));
        nListen = str2double(strtrim(clean(ssOut)));
        listening = ~isnan(nListen) && nListen > 0;
    else
        listening = false;
    end
    R{end+1} = chk(sprintf('bridge listening on :%d', c.io.port), listening, ...
        ternary(listening, 'a socket is listening inside WSL', ...
                'nothing is listening on that port inside WSL'), ...
        sprintf(['in WSL:\n        source /opt/ros/humble/setup.bash\n' ...
                 '        python3 matlab/mgs/bridge/mgs_bridge_node.py --port %d'], ...
                c.io.port));
else
    R{end+1} = skip(sprintf('bridge listening on :%d', c.io.port), ...
        sprintf('transport is "%s", not tcp', c.io.transport));
end

% ---- 7  handshake ----------------------------------------------------------
if strcmpi(c.io.transport, 'none')
    R{end+1} = skip('bridge handshake', 'transport is "none"');
else
    try
        b = mgs.Bridge(c);
        b.connect();
        a = b.ack;
        detail = sprintf('schema %d, ros %s, %d uav ns, target forwarding %s', ...
            getf(a, 'schema', -1), getf(a, 'ros_distro', '?'), ...
            getf(a, 'n_uav', 0), ...
            ternary(logical(getf(a, 'allow_target', false)), 'ENABLED', 'disabled'));
        b.close();
        R{end+1} = chk('bridge handshake', true, detail, '');
    catch ME
        R{end+1} = chk('bridge handshake', false, ME.message, ...
            'see the previous check');
    end
end

% ---- report ----------------------------------------------------------------
rep = struct('checks', {R});
rep.pass = all(cellfun(@(r) r.ok || r.skipped, R));
rep.hardFail = sum(cellfun(@(r) ~r.ok && ~r.skipped, R));
rep.summary = render(R, c);

if opts.verbose, fprintf('%s\n', rep.summary); end
if opts.strict && ~rep.pass
    error('mgs:preflight:failed', '%d preflight checks failed', rep.hardFail);
end
end

% =============================================================================
function r = chk(name, ok, detail, fix)
r = struct('name', name, 'ok', logical(ok), 'skipped', false, ...
           'detail', detail, 'fix', fix);
end

function r = skip(name, why)
r = struct('name', name, 'ok', false, 'skipped', true, 'detail', why, 'fix', '');
end

function s = render(R, c)
L = {};
L{end+1} = sprintf('mgs preflight  (transport %s, mode %s, %s:%d)', ...
    c.io.transport, c.io.mode, c.io.host, c.io.port);
L{end+1} = repmat('-', 1, 74);
for k = 1:numel(R)
    r = R{k};
    if r.skipped,   mark = 'SKIP';
    elseif r.ok,    mark = ' ok ';
    else,           mark = 'FAIL';
    end
    L{end+1} = sprintf('[%s] %-28s %s', mark, r.name, one_line(r.detail)); %#ok<AGROW>
    if ~r.ok && ~r.skipped && ~isempty(r.fix)
        L{end+1} = sprintf('       fix: %s', r.fix); %#ok<AGROW>
    end
end
s = strjoin(L, newline);
end

function [st, out] = runcmd(cmd)
[st, out] = system(cmd);
end

function s = clean(s)
% WSL writes UTF-16-ish output on some Windows builds; drop the NULs so the
% text is readable instead of appearing as one character.
s = char(s);
s(s == char(0)) = [];
end

function s = firstlines(s, n)
parts = strsplit(char(s), newline);
parts = parts(1:min(n, numel(parts)));
s = strjoin(parts, ' | ');
end

function s = one_line(s)
s = regexprep(char(s), '\s+', ' ');
if numel(s) > 88, s = [s(1:85) '...']; end
end

function v = ternary(tf, a, b)
if tf, v = a; else, v = b; end
end

function v = getf(s, f, dflt)
if isstruct(s) && isfield(s, f), v = s.(f); else, v = dflt; end
end
