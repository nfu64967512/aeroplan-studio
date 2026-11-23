function r = repoRoot()
%MGS.REPOROOT  Absolute path of the aeroplan-studio checkout.
%
%   r = mgs.repoRoot()
%
%   Derived from this file's own location, never from pwd -- a function that
%   silently resolves paths against the caller's working directory reads a
%   different file depending on where MATLAB happens to be sitting, and the
%   failure looks like "the data changed" rather than "I was in the wrong
%   folder".
%
%   Layout it depends on:  <root>/matlab/mgs/+mgs/repoRoot.m
%
%   See also MGS.CONTRACTTABLE, MGS.SELFTEST.

here = fileparts(mfilename('fullpath'));   % .../matlab/mgs/+mgs
r = fileparts(fileparts(fileparts(here))); % .../aeroplan-studio

% ⚠️ Verify rather than assume.  If someone moves this file the arithmetic
%    above still returns *a* path, and every caller would then look for data
%    in a directory that happens to exist and happens to be empty.
if ~isfolder(fullfile(r, 'scripts', 'wsl', 'sar_swarm_patches'))
    error('mgs:repoRoot:layout', ...
        ['resolved the repo root to "%s", but it has no ' ...
         'scripts/wsl/sar_swarm_patches -- this file has been moved out of ' ...
         '<root>/matlab/mgs/+mgs/ and the path arithmetic here needs updating.'], r);
end
end
