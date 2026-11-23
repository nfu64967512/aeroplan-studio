function path = recordStream(c, path, opts)
%MGS.RECORDSTREAM  Capture the live WSL stream to a file for later comparison.
%
%   path = mgs.recordStream(c, 'runs/shadow/RRX.ndjson', 'seconds', 300)
%
%   Writes one JSON object per line, exactly as received.  Record first and
%   diff afterwards, rather than diffing live: a diff that only exists while
%   the sortie is flying cannot be re-run when you disagree with it, and this
%   project's expensive mistakes have all been things nobody could go back and
%   re-check.
%
%   The file also carries a header line ({"type":"_meta",...}) with the schema
%   version, the bridge's hello_ack and the parameter snapshot in force, so a
%   recording is self-describing.
%
%   Recording STOPS on any of: the time limit, all vehicles reporting done in
%   tot_status, or Ctrl-C.  It flushes as it goes, so a Ctrl-C keeps what was
%   captured.
%
%   See also MGS.SHADOWDIFF, MGS.PARSESTREAM, MGS.BRIDGE.

arguments
    c
    path (1,:) char
    opts.seconds (1,1) double = 600
    opts.verbose (1,1) logical = true
    opts.bridge = []
end

d = fileparts(path);
if ~isempty(d) && ~isfolder(d), mkdir(d); end

if isempty(opts.bridge)
    b = mgs.Bridge(c);
    b.connect();
    ownBridge = true;
else
    b = opts.bridge;
    ownBridge = false;
end

fid = fopen(path, 'w', 'n', 'UTF-8');
if fid < 0
    error('mgs:recordStream:open', 'cannot write %s', path);
end
closer = onCleanup(@() fclose(fid));

meta = struct('type', '_meta', 't', posixtime(datetime('now','TimeZone','UTC')), ...
              'schema', b.SCHEMA, 'ack', b.ack, ...
              'params', struct('n', c.swarm.n, 'los_floor', c.pn.los_floor, ...
                               'nz_min', c.pn.nz_min, 'pn_vmax', c.pn.vmax, ...
                               'law', c.pn.law));
fprintf(fid, '%s\n', jsonencode(meta));

t0 = tic;  nmsg = 0;  nstate = 0;  lastReport = 0;
if opts.verbose
    fprintf('recording to %s (limit %g s, Ctrl-C to stop early)\n', path, opts.seconds);
end

try
    while toc(t0) < opts.seconds
        msgs = b.poll();
        for k = 1:numel(msgs)
            fprintf(fid, '%s\n', jsonencode(msgs{k}));
            nmsg = nmsg + 1;
            if strcmp(getf(msgs{k}, 'type', ''), 'state')
                nstate = nstate + 1;
            end
        end
        if isempty(msgs), pause(0.02); end

        if opts.verbose && toc(t0) - lastReport > 5
            lastReport = toc(t0);
            fprintf('  %5.0f s  %6d msgs  %6d state\n', lastReport, nmsg, nstate);
        end

        st = [];
        if ~isempty(msgs)
            for k = numel(msgs):-1:1
                if strcmp(getf(msgs{k}, 'type', ''), 'status'), st = msgs{k}; break; end
            end
        end
        if ~isempty(st) && isfield(st, 'data') && isfield(st.data, 'phase') ...
                && any(strcmpi(st.data.phase, {'DONE'}))
            if opts.verbose, fprintf('  tot_status reports DONE, stopping\n'); end
            break;
        end
    end
catch ME
    if ~strcmp(ME.identifier, 'MATLAB:Ctrl_C_interrupt'), rethrow(ME); end
end

if ownBridge, b.close(); end

if opts.verbose
    fprintf('recorded %d messages (%d state) in %.0f s -> %s\n', ...
        nmsg, nstate, toc(t0), path);
end
if nstate == 0
    warning('mgs:recordStream:nostate', ...
        ['NOT ONE per-UAV state message was captured.  The recording has ' ...
         'tot_gate/tot_status at best, so mgs.shadowDiff will have nothing ' ...
         'to compare the model against.  Usual cause: MAVROS is not up, or ' ...
         'the bridge was started with --n 0.']);
end
end

function v = getf(s, f, dflt)
if isstruct(s) && isfield(s, f), v = s.(f); else, v = dflt; end
end
