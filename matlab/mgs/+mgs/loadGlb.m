function M = loadGlb(path, opts)
%MGS.LOADGLB  Read a binary glTF 2.0 (.glb) mesh into patch faces/vertices.
%
%   M = mgs.loadGlb('ui/resources/barracuda500.glb')
%   M = mgs.loadGlb(path, faces=1500)          % decimate for animation
%   M = mgs.loadGlb(path, frame="body")        % remap axes to x-fwd y-right z-down
%
%   M.v        [Nv x 3]  vertices, metres, in the requested frame
%   M.f        [Nf x 3]  triangle faces, 1-based
%   M.bbox     struct with .min .max in the requested frame
%   M.n_tri_in number of triangles before decimation
%   M.notes    cellstr -- everything that was assumed or measured, verbatim
%
% ---------------------------------------------------------------------------
%   WHY A READER AND NOT A TOOLBOX CALL
%
%   Base MATLAB reads STL, not GLB, and every GLB->STL path adds an external
%   tool to a pipeline that currently has none (mgs.viz6dof resolves entirely
%   to toolbox/matlab).  A glTF 2.0 binary with no extensions is a 12-byte
%   header, a JSON chunk and a BIN chunk -- about eighty lines to decode, and
%   it keeps the render step dependency-free.
%
%   ⚠️ Deliberately NOT supported, because supporting them badly is worse than
%   refusing: Draco/meshopt compression, sparse accessors, byteStride
%   interleaving, node transforms, and non-TRIANGLES primitive modes.  Each
%   one ERRORS with what it found.  barracuda500.glb needs none of them
%   (measured: extensionsUsed absent, 3 primitives, mode 4, no byteStride,
%   one node with no transform).
%
% ---------------------------------------------------------------------------
%   ⚠️⚠️ THE AXIS MAPPING IS MEASURED, NOT ASSUMED -- AND ONE HALF OF IT IS
%   NOT OBSERVABLE.
%
%   glTF 2.0 defines +Y up and a right-handed frame.  This model was exported
%   with the FUSELAGE ALONG X, which is not the glTF "forward" convention, so
%   the mapping had to be established from the mesh itself:
%
%     measured cross-section of primitive 1 (141,867 vertices, 0.25 m bins):
%        X +1.85..+2.10   |Y|max 0.062  |Z|max 0.066   r95 0.076  <- tapers to a POINT
%        X -1.90..-1.65   |Y|max 0.451  |Z|max 0.397   r95 0.451  <- blunt, widest
%        X +0.10..+0.60                 |Z|max 1.162             <- the wings
%     plus primitive 2: a thin disc at X = +1.16..+1.19, Y mostly NEGATIVE
%        -> a ventral intake, which sits under the fuselage on this airframe
%
%   => +X IS THE NOSE.  Memory `barracuda500-model-facts` recorded this as
%      unresolved ("+X is nose or tail: the bounding box is symmetric, must be
%      confirmed visually in Gazebo").  It is resolved here from the geometry.
%
%   Body frame used by mgs.viz6dof is x forward, y right, z down, so:
%        bx = +gx        nose is +X (measured above)
%        bz = -gy        glTF +Y is up, body +Z is down
%        by = +gz        forced by right-handedness once bx and bz are fixed
%
%   ⚠️ WHICH PHYSICAL WING +gz POINTS AT IS NOT OBSERVABLE.  The airframe is
%   left-right symmetric, so no measurement on this mesh can distinguish port
%   from starboard.  Right-handedness fixes the arithmetic; it does not tell
%   you whether the CAD author drew the right wing at +Z.  For a symmetric
%   airframe the picture is identical either way, so this costs nothing here
%   -- but do NOT carry this mapping to anything that cares about handedness
%   (control-surface deflections, asymmetric stores) without checking.
%
% ---------------------------------------------------------------------------
%   ⚠️⚠️ THE CAD BODY IS NOT THE BODY THAT FLEW.  Memory
%   `barracuda500-model-facts`: every sortie to date was flown by ArduPilot
%   SIM_Plane's generic `-jet` (m = 22.0 kg, b = 1.88 m, S = 0.45 m^2), while
%   this CAD has a 2.325 m span and a 3.786 m length.  Drawing the CAD makes a
%   picture of an airframe whose aerodynamics were never simulated.  That is
%   fine for a report as long as it says so -- mgs.viz6dof puts it in the DOF
%   ledger -- and not fine anywhere it could be read as a validated model.
%
%   See also MGS.VIZ6DOF, REDUCEPATCH.

arguments
    path (1,1) string
    opts.faces (1,1) double = 0            % 0 = no decimation
    opts.frame (1,1) string {mustBeMember(opts.frame, ["gltf","body"])} = "body"
    opts.verbose (1,1) logical = false
end

f = char(path);
if ~isfile(f)
    error('mgs:loadGlb:missing', 'no such file: %s', f);
end
raw = read_all_bytes(f);

% ── header ───────────────────────────────────────────────────────────────
if numel(raw) < 12 || ~isequal(char(raw(1:4)'), 'glTF')
    error('mgs:loadGlb:magic', '%s does not start with the glTF magic', f);
end
ver  = typecast(raw(5:8),  'uint32');
tlen = typecast(raw(9:12), 'uint32');
if ver ~= 2
    error('mgs:loadGlb:version', 'glTF version %d; only 2 is supported', ver);
end
if tlen > numel(raw)
    error('mgs:loadGlb:truncated', ...
        'header declares %d bytes but the file has %d', tlen, numel(raw));
end

% ── chunks ───────────────────────────────────────────────────────────────
off = 12; jsonTxt = ''; binOff = [];
while off + 8 <= tlen
    clen  = double(typecast(raw(off+1:off+4), 'uint32'));
    ctype = char(raw(off+5:off+8)');
    body  = off + 8;
    switch ctype
        case 'JSON', jsonTxt = native2unicode(raw(body+1:body+clen)', 'UTF-8');
        case ['BIN' char(0)], binOff = body;
    end
    off = body + clen + mod(4 - mod(clen, 4), 4);
end
if isempty(jsonTxt), error('mgs:loadGlb:noJson', 'no JSON chunk in %s', f); end
if isempty(binOff),  error('mgs:loadGlb:noBin',  'no BIN chunk in %s', f);  end
J = jsondecode(jsonTxt);

% ── refuse what we cannot do correctly ───────────────────────────────────
if isfield(J, 'extensionsRequired') && ~isempty(J.extensionsRequired)
    error('mgs:loadGlb:extensions', ...
        ['%s requires glTF extensions %s. This reader decodes plain ' ...
         'accessors only -- decoding a compressed mesh incorrectly would ' ...
         'draw a plausible wrong shape, so it refuses instead.'], ...
        f, strjoin(cellstr(J.extensionsRequired), ', '));
end

meshes = as_cell(J.meshes);
notes = {};
V = zeros(0, 3); F = zeros(0, 3);
for mi = 1:numel(meshes)
    prims = as_cell(meshes{mi}.primitives);
    for pi = 1:numel(prims)
        pr = prims{pi};
        mode = 4;
        if isfield(pr, 'mode'), mode = pr.mode; end
        if mode ~= 4
            error('mgs:loadGlb:mode', ...
                'primitive %d of mesh %d uses mode %d; only 4 (TRIANGLES) is supported', ...
                pi, mi, mode);
        end
        if ~isfield(pr.attributes, 'POSITION')
            error('mgs:loadGlb:noPosition', 'primitive %d has no POSITION', pi);
        end
        P = read_accessor(J, raw, binOff, pr.attributes.POSITION, 'VEC3');
        if isfield(pr, 'indices') && ~isempty(pr.indices)
            I = read_accessor(J, raw, binOff, pr.indices, 'SCALAR');
            if mod(numel(I), 3) ~= 0
                error('mgs:loadGlb:indices', ...
                    'primitive %d has %d indices, not a multiple of 3', pi, numel(I));
            end
            Fp = reshape(I, 3, []).' + 1;                 % glTF is 0-based
        else
            Fp = reshape(1:size(P,1), 3, []).';
        end
        F = [F; Fp + size(V,1)];  V = [V; P];             %#ok<AGROW>
    end
end
n_tri_in = size(F, 1);
notes{end+1} = sprintf('%d vertices, %d triangles read from %d primitive(s)', ...
                       size(V,1), n_tri_in, numel(F)/n_tri_in*0 + count_prims(meshes));

% ── node transforms: refuse rather than ignore ───────────────────────────
if isfield(J, 'nodes')
    nd = as_cell(J.nodes);
    for k = 1:numel(nd)
        if isfield(nd{k}, 'matrix') || isfield(nd{k}, 'rotation') || ...
           isfield(nd{k}, 'scale')  || isfield(nd{k}, 'translation')
            error('mgs:loadGlb:nodeTransform', ...
                ['node %d carries a transform. This reader merges primitives in ' ...
                 'their own coordinates and would silently place them wrongly. ' ...
                 '(barracuda500.glb has none.)'], k);
        end
    end
end

% ── frame ────────────────────────────────────────────────────────────────
if opts.frame == "body"
    % [bx by bz] = [+gx +gz -gy]   -- see the header; +X measured as the nose
    V = [V(:,1), V(:,3), -V(:,2)];
    notes{end+1} = ['axes remapped glTF->body as [bx by bz] = [+gx +gz -gy]; ' ...
                    '+X measured as the NOSE from the cross-section profile'];
    notes{end+1} = ['which physical wing +gz points at is NOT observable on a ' ...
                    'symmetric airframe -- do not reuse this mapping where ' ...
                    'handedness matters'];
end

% ── decimate ─────────────────────────────────────────────────────────────
if opts.faces > 0 && n_tri_in > opts.faces
    p = reducepatch(struct('faces', F, 'vertices', V), opts.faces);
    V = p.vertices; F = p.faces;
    notes{end+1} = sprintf('decimated %d -> %d triangles for animation', ...
                           n_tri_in, size(F,1));
end

M = struct('v', V, 'f', F, 'n_tri_in', n_tri_in, ...
           'bbox', struct('min', min(V,[],1), 'max', max(V,[],1)), ...
           'notes', {notes}, 'path', f);

if opts.verbose
    fprintf('%s\n', f);
    fprintf('  bbox min [%7.3f %7.3f %7.3f]  max [%7.3f %7.3f %7.3f]  (m)\n', ...
            M.bbox.min, M.bbox.max);
    for k = 1:numel(notes), fprintf('  - %s\n', notes{k}); end
end
end

% =============================================================================
function b = read_all_bytes(f)
fid = fopen(f, 'r');
if fid < 0, error('mgs:loadGlb:open', 'cannot open %s', f); end
c = onCleanup(@() fclose(fid));
b = fread(fid, inf, '*uint8');
end

% -----------------------------------------------------------------------------
function A = read_accessor(J, raw, binOff, idx, wantType)
%READ_ACCESSOR  One glTF accessor -> [count x ncomp] double.
acc = elem(J.accessors, idx + 1);
if ~strcmp(acc.type, wantType)
    error('mgs:loadGlb:accType', 'accessor %d is %s, expected %s', ...
          idx, acc.type, wantType);
end
if isfield(acc, 'sparse')
    error('mgs:loadGlb:sparse', ...
        'accessor %d is sparse; this reader would drop the overrides', idx);
end
if ~isfield(acc, 'bufferView')
    error('mgs:loadGlb:noBufferView', ...
        'accessor %d has no bufferView (zero-filled); refusing to guess', idx);
end
bv = elem(J.bufferViews, acc.bufferView + 1);
if isfield(bv, 'byteStride') && ~isempty(bv.byteStride)
    error('mgs:loadGlb:stride', ...
        ['bufferView %d is interleaved (byteStride %d). This reader assumes ' ...
         'tight packing and would read garbage.'], acc.bufferView, bv.byteStride);
end
switch acc.componentType
    case 5126, cls = 'single'; w = 4;
    case 5125, cls = 'uint32'; w = 4;
    case 5123, cls = 'uint16'; w = 2;
    case 5121, cls = 'uint8';  w = 1;
    otherwise
        error('mgs:loadGlb:compType', ...
              'accessor %d componentType %d not supported', idx, acc.componentType);
end
ncomp = struct('SCALAR', 1, 'VEC2', 2, 'VEC3', 3, 'VEC4', 4).(acc.type);
n = double(acc.count) * ncomp;
off = binOff + getdef(bv, 'byteOffset', 0) + getdef(acc, 'byteOffset', 0);
need = off + n*w;
if need > numel(raw)
    error('mgs:loadGlb:range', ...
        'accessor %d wants bytes up to %d but the file has %d', idx, need, numel(raw));
end
A = double(typecast(raw(off+1 : off+n*w), cls));
if ncomp > 1, A = reshape(A, ncomp, []).'; end
end

% -----------------------------------------------------------------------------
function c = as_cell(x)
%AS_CELL  jsondecode gives a struct ARRAY for homogeneous lists and a cell for
%   heterogeneous ones.  Normalise so callers do not have to branch.
if iscell(x), c = x(:).'; return; end
c = arrayfun(@(k) x(k), 1:numel(x), 'UniformOutput', false);
end

function e = elem(x, k)
if iscell(x), e = x{k}; else, e = x(k); end
end

function v = getdef(s, f, d)
if isfield(s, f) && ~isempty(s.(f)), v = double(s.(f)); else, v = d; end
end

function n = count_prims(meshes)
n = 0;
for k = 1:numel(meshes), n = n + numel(as_cell(meshes{k}.primitives)); end
end
