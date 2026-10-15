function h = sha1file(path)
%MGS.UTIL.SHA1FILE  Lowercase hex SHA-1 of a file's bytes.
%
%   h = mgs.util.sha1file(path)
%
%   Used to prove contract_table.json was generated from the contract_table.py
%   sitting next to it.  Java is the primary route; the shell fallbacks exist
%   for -nojvm sessions.  If none of them work this ERRORS rather than
%   returning a sentinel: a hash comparison that silently always passes is
%   worse than no hash comparison at all.

if ~isfile(path)
    error('mgs:sha1file:missing', 'no such file: %s', path);
end

if usejava('jvm')
    fid = fopen(path, 'r');
    if fid < 0, error('mgs:sha1file:open', 'cannot open %s', path); end
    cleanup = onCleanup(@() fclose(fid));
    bytes = fread(fid, Inf, '*uint8');
    md = java.security.MessageDigest.getInstance('SHA-1');
    md.update(bytes);
    % dec2hex(...,2) not dec2hex(...): the 1-argument form sizes the field to
    % the widest byte in the array, so a digest whose bytes are all < 0x10
    % would come back half-length.
    h = lower(reshape(dec2hex(typecast(md.digest(), 'uint8'), 2).', 1, []));
    return;
end

if ispc
    [st, out] = system(sprintf('certutil -hashfile "%s" SHA1', path));
    if st == 0
        tok = regexp(out, '(?m)^\s*([0-9a-fA-F]{40})\s*$', 'tokens', 'once');
        if ~isempty(tok), h = lower(tok{1}); return; end
    end
else
    [st, out] = system(sprintf('sha1sum "%s"', path));
    if st == 0
        tok = regexp(out, '([0-9a-fA-F]{40})', 'tokens', 'once');
        if ~isempty(tok), h = lower(tok{1}); return; end
    end
end

error('mgs:sha1file:nohasher', ...
    ['could not hash %s: no JVM and no working command-line hasher.  ' ...
     'Refusing to return a placeholder -- the staleness check would then ' ...
     'pass unconditionally.'], path);
end
