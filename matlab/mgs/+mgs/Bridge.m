classdef Bridge < handle
%MGS.BRIDGE  The MATLAB end of the ROS 2 <-> MATLAB interface.
%
%   b = mgs.Bridge(c);            c = mgs.params()
%   b.connect();
%   b.poll();                     drain what has arrived
%   s = b.latest('status');       last /swarm/tot_status payload
%   b.sendShadow(snapshot);       publish the model's state to /mgs/shadow
%   v = b.contract();             read the LIVE effective values back
%   b.close();
%
% ---------------------------------------------------------------------------
%   ONE SCHEMA, TWO TRANSPORTS
%
%   c.io.transport = 'tcp'   (default)  MATLAB tcpclient  <->  NDJSON  <->
%                                       matlab/mgs/bridge/mgs_bridge_node.py
%                                       running inside WSL.
%
%   c.io.transport = 'ros2'             ROS Toolbox speaks DDS directly to the
%                                       WSL nodes, no bridge process.
%
%   'tcp' is the default because of how THIS machine is set up.  WSL2 is in
%   its default NAT networking mode, so DDS multicast discovery does not cross
%   into Windows -- ROS Toolbox comes up, finds nothing, and reports no error.
%   TCP works today because WSL2 forwards Windows loopback into the distro.
%
%   'ros2' needs mirrored networking:  %USERPROFILE%\.wslconfig
%       [wsl2]
%       networkingMode=mirrored
%   then `wsl --shutdown`.  mgs.preflight checks which mode you are in and
%   says so BEFORE you spend a session wondering why nothing arrives.
%
% ---------------------------------------------------------------------------
%   THREE MODES  (c.io.mode)
%
%   'shadow'   MATLAB subscribes and runs the same guidance in parallel with
%              the live stack.  Nothing MATLAB computes touches the aircraft.
%              This is the safe default and the one to use for "does the model
%              agree with the vehicle".
%
%   'sil'      MATLAB computes the guidance command and publishes it to
%              /mgs/cmd for the WSL side to consume.  Software in the loop.
%
%   'offline'  no link at all; mgs.simSwarm runs standalone.  Use this to
%              answer "if I set it like THIS, what happens" without touching
%              the live stack.
%
% ---------------------------------------------------------------------------
%   c.io.strict = true (default) MEANS WHAT IT SAYS.  A dropped link, a schema
%   mismatch or a stale stream raises.  It never falls back to the offline
%   model while still reporting live numbers -- that substitution is the exact
%   failure this project keeps paying for, and it is worse here than usual
%   because the offline model looks entirely healthy.
%
%   See also MGS.PREFLIGHT, MGS.SIMSWARM, MGS.PARAMS.

properties (Constant)
    SCHEMA = 1              % must match mgs_bridge_node.py SCHEMA
end

properties (SetAccess = private)
    c
    transport
    connected = false
    ack = struct()          % hello_ack from the bridge
    stats = struct('rx', 0, 'tx', 0, 'errors', 0, 'lastRx', NaN)
end

properties (Access = private)
    sock                    % tcpclient
    rxbuf = ''              % NDJSON reassembly
    store                   % containers.Map: type -> last payload
    node                    % ros2node        (transport 'ros2')
    pubs                    % containers.Map  (transport 'ros2')
    subs                    % containers.Map  (transport 'ros2')
    pending = {}            % decoded messages not yet consumed
end

methods
    function b = Bridge(c)
        if nargin < 1, c = mgs.params(); end
        b.c = c;
        b.transport = lower(c.io.transport);
        b.store = containers.Map('KeyType', 'char', 'ValueType', 'any');
    end

    % ---------------------------------------------------------------------
    function connect(b)
        switch b.transport
            case 'none'
                error('mgs:Bridge:none', ...
                    ['c.io.transport = "none".  There is nothing to ' ...
                     'connect to.  Set it to tcp or ros2, or run ' ...
                     'mgs.simSwarm directly for an offline study.']);
            case 'tcp',  b.connectTcp();
            case 'ros2', b.connectRos2();
            otherwise
                error('mgs:Bridge:transport', ...
                    'unknown c.io.transport "%s" (tcp | ros2 | none)', ...
                    b.transport);
        end
        b.connected = true;
    end

    % ---------------------------------------------------------------------
    function msgs = poll(b)
        %POLL  Drain everything that has arrived; return it and file the last
        %   of each type into the store.
        b.requireLink();
        switch b.transport
            case 'tcp',  msgs = b.pollTcp();
            case 'ros2', msgs = b.pollRos2();
        end
        for k = 1:numel(msgs)
            m = msgs{k};
            if isfield(m, 'type')
                b.store(m.type) = m;
            end
            if isfield(m, 'type') && strcmp(m.type, 'error')
                b.stats.errors = b.stats.errors + 1;
                warning('mgs:Bridge:remote', 'bridge reported: %s', ...
                    getf(m, 'what', '<no detail>'));
            end
        end
        if ~isempty(msgs), b.stats.lastRx = now_s(); end
        b.stats.rx = b.stats.rx + numel(msgs);
    end

    % ---------------------------------------------------------------------
    function s = latest(b, kind, maxAge)
        %LATEST  Most recent payload of a type, with an AGE CHECK.
        %   A stale message that still looks well-formed is the single hardest
        %   failure to see, so age is not optional: past maxAge this errors
        %   (strict) or warns.  Default 3 s.
        if nargin < 3, maxAge = 3.0; end
        if ~isKey(b.store, kind)
            s = [];
            return;
        end
        s = b.store(kind);
        if isfield(s, 't')
            age = now_s() - s.t;
            if age > maxAge
                msg = sprintf(['"%s" is %.1f s old (limit %.1f s).  The ' ...
                    'link is up but the data is not fresh.'], kind, age, maxAge);
                if b.c.io.strict
                    error('mgs:Bridge:stale', '%s', msg);
                else
                    warning('mgs:Bridge:stale', '%s', msg);
                end
            end
        end
    end

    % ---------------------------------------------------------------------
    function rtt = ping(b, timeout)
        %PING  Round-trip the link.  Returns the round-trip time in seconds,
        %   or errors -- never NaN, because a NaN round-trip reads as "slow"
        %   when it actually means "no answer at all".
        if nargin < 2, timeout = 3.0; end
        b.send(struct('type', 'ping', 't', now_s()));
        t0 = tic;
        m = b.waitFor('pong', timeout);
        if isempty(m)
            error('mgs:Bridge:ping', ...
                ['no pong within %g s.  The socket is open but the bridge ' ...
                 'is not answering -- check /tmp/mgs_bridge.log in WSL.'], ...
                timeout);
        end
        rtt = toc(t0);
    end

    function sendShadow(b, snap)
        %SENDSHADOW  Publish the model's swarm state to /mgs/shadow.
        m = snap;
        m.type = 'shadow';
        m.t = now_s();
        b.send(m);
    end

    function sendCmd(b, uavId, cmd)
        %SENDCMD  Publish one guidance command to /mgs/cmd.  SIL mode only.
        if ~strcmp(b.c.io.mode, 'sil')
            error('mgs:Bridge:mode', ...
                ['sendCmd is only valid with c.io.mode = "sil" (currently ' ...
                 '"%s").  In shadow mode the model must not command ' ...
                 'anything -- that separation is the whole point of the ' ...
                 'mode.'], b.c.io.mode);
        end
        b.send(struct('type', 'cmd', 't', now_s(), 'uav', uavId, ...
                      'roll', cmd.roll, 'pitch', cmd.pitch, 'thr', cmd.thr));
    end

    function ok = setTarget(b, lat, lon, amsl)
        %SETTARGET  Publish a bullseye to /swarm/tot_target.
        %
        %   *** THIS MAKES REAL AIRCRAFT FLY. ***  The bridge refuses it
        %   unless it was started with --allow-target, and says so rather than
        %   dropping it quietly.
        %
        %   amsl is REQUIRED and there is no default.  A sea target is at
        %   0 m AMSL; terrain libraries return garbage over water, and a
        %   strike payload without amsl once meant every maritime bullseye
        %   used a junk elevation.
        if nargin < 4
            error('mgs:Bridge:amsl', ...
                ['setTarget requires amsl.  Pass 0.0 for a sea-level target. ' ...
                 'There is deliberately no default: the missing-amsl case is ' ...
                 'a known, expensive failure.']);
        end
        b.send(struct('type', 'target', 't', now_s(), 'lat', lat, ...
                      'lon', lon, 'amsl', amsl));
        ok = true;
    end

    % ---------------------------------------------------------------------
    function [values, raw] = contract(b, timeout)
        %CONTRACT  Ask the live stack for its EFFECTIVE VALUES and return them.
        %
        %   values = b.contract()
        %
        %   The bridge runs scripts/wsl/contract_verify.py --snapshot, i.e.
        %   the same readback that gates a real sortie: ros2 param get off the
        %   running node, PARM read back off the flight controller.  Not the
        %   launch command, not the shell defaults -- what is actually in
        %   effect right now.
        %
        %   Feed the result straight to mgs.fromContract and the model runs on
        %   the same numbers the aircraft are running on.
        if nargin < 2, timeout = 130; end
        b.requireLink();
        b.send(struct('type', 'contract', 't', now_s(), 'snapshot', true));
        raw = b.waitFor('contract', timeout);
        if isempty(raw)
            error('mgs:Bridge:contract', ...
                ['no contract reply within %g s.  contract_verify.py can ' ...
                 'take a while (it reads every PARM back off the FC), but ' ...
                 'silence past that is a broken link, not a slow one.'], timeout);
        end
        values = getf(raw, 'values', struct());
        if ~getf(raw, 'ok', false)
            warning('mgs:Bridge:contractFailed', ...
                ['the live stack FAILED its own contract check ' ...
                 '(returncode %d).  The values below are what is really in ' ...
                 'effect, so the model can still be run on them -- but the ' ...
                 'aircraft are out of contract right now.\n%s'], ...
                getf(raw, 'returncode', -1), getf(raw, 'stdout', ''));
        end
    end

    % ---------------------------------------------------------------------
    function m = waitFor(b, kind, timeout)
        %WAITFOR  Block until a message of this type arrives, or time out.
        t0 = tic;  m = [];
        while toc(t0) < timeout
            msgs = b.poll();
            for k = 1:numel(msgs)
                if isfield(msgs{k}, 'type') && strcmp(msgs{k}.type, kind)
                    m = msgs{k};  return;
                end
            end
            pause(0.05);
        end
    end

    % ---------------------------------------------------------------------
    function hook = simHook(b)
        %SIMHOOK  A function handle for mgs.simSwarm's 'hook' option.
        %
        %   out = mgs.simSwarm(scn, c, 'hook', b.simHook(), 'realtime', true)
        %
        %   Streams every guidance tick of the model into /mgs/shadow, so the
        %   WSL side can display the simulated swarm live next to the real one.
        hook = @(t, snap) b.onSimTick(t, snap);
    end

    function onSimTick(b, t, snap)
        snap.t_sim = t;
        b.sendShadow(snap);
    end

    % ---------------------------------------------------------------------
    function close(b)
        try
            switch b.transport
                case 'tcp'
                    if ~isempty(b.sock), clear b.sock; b.sock = []; end
                case 'ros2'
                    b.pubs = []; b.subs = []; b.node = [];
            end
        catch
        end
        b.connected = false;
    end

    function delete(b)
        b.close();
    end
end

% =============================================================================
methods (Access = private)

    function requireLink(b)
        if ~b.connected
            error('mgs:Bridge:notConnected', 'call connect() first');
        end
    end

    function send(b, m)
        b.requireLink();
        switch b.transport
            case 'tcp'
                line = [jsonencode(m) newline];
                write(b.sock, uint8(line));
            case 'ros2'
                topic = b.topicFor(m.type);
                msg = ros2message(b.pubs(topic));
                msg.data = jsonencode(m);
                send(b.pubs(topic), msg);
        end
        b.stats.tx = b.stats.tx + 1;
    end

    % ---- TCP -------------------------------------------------------------
    function connectTcp(b)
        io = b.c.io;
        try
            b.sock = tcpclient(io.host, io.port, 'Timeout', io.timeout_s, ...
                               'ConnectTimeout', max(io.timeout_s, 5));
        catch ME
            error('mgs:Bridge:tcp', ...
                ['cannot reach the bridge at %s:%d (%s).\n' ...
                 'Start it inside WSL:\n' ...
                 '    source /opt/ros/humble/setup.bash\n' ...
                 '    python3 matlab/mgs/bridge/mgs_bridge_node.py --port %d\n' ...
                 'Then re-run mgs.preflight.'], ...
                io.host, io.port, ME.message, io.port);
        end
        b.connected = true;                 % so send() works during handshake
        b.send(struct('type', 'hello', 't', now_s(), 'schema', b.SCHEMA, ...
                      'peer', 'matlab', 'mode', b.c.io.mode));
        m = b.waitFor('hello_ack', max(b.c.io.timeout_s, 5));
        if isempty(m)
            b.connected = false;
            error('mgs:Bridge:handshake', ...
                ['connected to %s:%d but got no hello_ack.  Something is ' ...
                 'listening on that port -- it is not the mgs bridge.'], ...
                io.host, io.port);
        end
        b.ack = m;
    end

    function msgs = pollTcp(b)
        msgs = {};
        nb = b.sock.NumBytesAvailable;
        if nb > 0
            b.rxbuf = [b.rxbuf char(read(b.sock, nb, 'uint8'))];
        end
        while true
            idx = find(b.rxbuf == newline, 1, 'first');
            if isempty(idx), break; end
            line = strtrim(b.rxbuf(1:idx-1));
            b.rxbuf = b.rxbuf(idx+1:end);
            if isempty(line), continue; end
            try
                msgs{end+1} = jsondecode(line); %#ok<AGROW>
            catch ME
                b.stats.errors = b.stats.errors + 1;
                warning('mgs:Bridge:json', ...
                    'undecodable line from the bridge (%s): %s', ...
                    ME.message, trunc(line));
            end
        end
    end

    % ---- native ROS 2 -----------------------------------------------------
    function connectRos2(b)
        if isempty(ver('ros'))
            error('mgs:Bridge:noRosToolbox', ...
                ['c.io.transport = "ros2" needs ROS Toolbox, which is not ' ...
                 'installed.  Use the tcp transport instead.']);
        end
        io = b.c.io;
        b.node = ros2node('/mgs_matlab', io.domain_id);
        b.pubs = containers.Map('KeyType', 'char', 'ValueType', 'any');
        b.subs = containers.Map('KeyType', 'char', 'ValueType', 'any');
        b.pending = {};

        b.pubs('/mgs/shadow')        = ros2publisher(b.node, '/mgs/shadow', 'std_msgs/String');
        b.pubs('/mgs/cmd')           = ros2publisher(b.node, '/mgs/cmd', 'std_msgs/String');
        b.pubs('/swarm/tot_target')  = ros2publisher(b.node, '/swarm/tot_target', 'std_msgs/String');

        b.subs('/swarm/tot_status') = ros2subscriber(b.node, '/swarm/tot_status', ...
            'std_msgs/String', @(m) b.onRos2(m, 'status'));
        b.subs('/swarm/tot_gate') = ros2subscriber(b.node, '/swarm/tot_gate', ...
            'std_msgs/String', @(m) b.onRos2(m, 'gate'));

        % Discovery is not instant, and an empty topic list here is the
        % symptom of NAT-mode WSL, not of a stack that is down.  Say which.
        pause(1.0);
        tl = ros2('topic', 'list', 'DomainID', io.domain_id);
        if ~any(contains(string(tl), '/swarm/'))
            error('mgs:Bridge:discovery', ...
                ['ROS 2 discovery on domain %d found no /swarm/ topics.\n' ...
                 'If the WSL stack IS running, this is almost certainly WSL2 ' ...
                 'NAT networking: DDS multicast discovery does not cross it, ' ...
                 'and nothing reports an error -- you simply receive nothing ' ...
                 'forever.\nEither switch WSL to mirrored networking ' ...
                 '(%%USERPROFILE%%\\.wslconfig -> [wsl2] networkingMode=mirrored, ' ...
                 'then wsl --shutdown), or use c.io.transport = "tcp".\n' ...
                 'Run mgs.preflight for the specifics of this machine.'], ...
                io.domain_id);
        end
        b.ack = struct('type', 'hello_ack', 'schema', b.SCHEMA, ...
                       'transport', 'ros2', 'topics', {cellstr(string(tl))});
    end

    function onRos2(b, msg, kind)
        try
            data = jsondecode(msg.data);
        catch
            data = struct('x_raw', msg.data, 'x_parse_error', true);
        end
        b.pending{end+1} = struct('type', kind, 't', now_s(), 'data', data);
    end

    function msgs = pollRos2(b)
        msgs = b.pending;
        b.pending = {};
    end

    function topic = topicFor(~, kind)
        switch kind
            case 'shadow', topic = '/mgs/shadow';
            case 'cmd',    topic = '/mgs/cmd';
            case 'target', topic = '/swarm/tot_target';
            otherwise
                error('mgs:Bridge:topic', ...
                    ['message type "%s" has no ROS 2 topic.  It exists only ' ...
                     'on the tcp transport (the bridge process implements ' ...
                     'it); there is no equivalent to fall back to.'], kind);
        end
    end
end
end

% =============================================================================
function t = now_s()
%NOW_S  Unix seconds, matching the bridge's time.time().
t = posixtime(datetime('now', 'TimeZone', 'UTC'));
end

function v = getf(s, f, dflt)
if isstruct(s) && isfield(s, f), v = s.(f); else, v = dflt; end
end

function s = trunc(s)
if numel(s) > 200, s = [s(1:197) '...']; end
end
