function root = mgs_setup()
%MGS_SETUP  Put the multi-missile guidance environment on the MATLAB path.
%
%   mgs_setup()          add matlab/ and matlab/mgs/ to the path
%   root = mgs_setup()   ... and return matlab/mgs/
%
%   matlab/ must come along because +mgs reuses the VALIDATED single-vehicle
%   chain that already lives there:
%       stot_guidance.m    line-by-line port of tot_node.py::_pn_cmd
%       stot_fcs_clamp.m   the ArduPlane clamp chain
%       stot_plant.m       3-DOF point mass + AP inner loop
%       +cfg/params.m      the effective-value snapshot
%   +mgs does NOT re-implement any of them.  If you find a second copy of the
%   PN law in this folder, it is a bug: two copies drift, and drift here is
%   exactly the silent failure this project keeps paying for.

here = fileparts(mfilename('fullpath'));      % .../matlab/mgs
parent = fileparts(here);                     % .../matlab
addpath(parent);
addpath(here);
addpath(fullfile(here, 'models'));
addpath(fullfile(here, 'demo'));
if nargout > 0, root = here; end

need = {'stot_guidance','stot_fcs_clamp','stot_plant'};
for k = 1:numel(need)
    if isempty(which(need{k}))
        error('mgs_setup:missing', ...
            ['%s not found on the path.  mgs reuses the validated chain in ' ...
             'matlab/ and refuses to run without it.'], need{k});
    end
end
if isempty(which('cfg.params'))
    error('mgs_setup:missing', 'package cfg (+cfg/params.m) not found under %s', parent);
end
