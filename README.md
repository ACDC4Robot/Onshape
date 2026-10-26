# ACDC4Robot Onshape companion — beta foundation

A standalone Python CLI for **locally validating and preparing an exported URDF
bundle**. This companion does not import the Fusion Add-In or Autodesk APIs,
and does not change the existing Fusion release.

**Status:** `0.1.0b2` foundation. Automated tests use synthetic URDF bundles.
Live native Onshape export compatibility has not yet been established. No
Onshape API connector, embedded Onshape UI, app-store integration, or automatic
CAD-to-URDF export is included in this foundation. A successful offline report
does not establish simulator or CAD-frame correctness.

Onshape now has native assembly URDF export. Export the assembly as URDF with
**STL geometry**, then give this tool the resulting directory or ZIP. The tool
discovers the layout from URDF asset references, rather than expecting a
particular meshes folder. GLTF/GLB/OBJ are outside the first beta's scope.

## Install and run

Requires Python 3.12 or later. From this checkout:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install .
.venv/bin/acdc4robot-onshape validate /path/to/export.zip
.venv/bin/acdc4robot-onshape prepare /path/to/export.zip /path/to/prepared
```

On Windows, use `.venv\Scripts\python.exe` and the installed commands in
`.venv\Scripts`. Choose a new output directory whose parent already exists.
For directory input, source and output must be separate and non-nested. For ZIP
input, a new output directory beside the archive is supported.

If there are multiple URDF files, choose one explicitly:

```bash
acdc4robot-onshape prepare export.zip prepared \
  --urdf assembly/description/robot.urdf
```

A `package://robot/meshes/part.stl` reference needs an explicit package root:

```bash
acdc4robot-onshape prepare export prepared \
  --package robot=/path/to/robot-package
```

Relative paths resolve from the selected URDF and may use `../` while staying
inside the input bundle. Package paths must stay inside their mapped root.
Remote/file URIs, absolute asset paths, escaping symlinks, and unresolved assets
are rejected. No ROS environment, network lookup, or implicit package search
is used. Windows asset references must use forward slashes.

Optional source information is recorded as **user-supplied, unverified**
metadata:

```bash
acdc4robot-onshape prepare export prepared \
  --source-url https://cad.onshape.com/documents/DOCUMENT/v/VERSION/e/ASSEMBLY \
  --revision VERSION --configuration '{"length":"100 mm"}'
```

## Validation and preservation

The dependency-free core checks:

- unique nonempty link/joint names without whitespace, existing endpoints,
  exactly one root, a connected tree, no multiple parents or cycles;
- fixed, revolute, continuous, and prismatic joints; finite nonzero moving
  axes, numeric/origin vector lengths, and consistent provided limits;
- finite numeric fields, positive geometry dimensions/scales, and valid shape
  declarations;
- complete inertial blocks with positive mass, positive-definite tensors, and
  physical principal-moment triangle inequalities;
- legitimate geometry-free fixed/root frames without inertia, including
  explicitly zero mass/tensor fixed frames; explicitly zero-mass, visual-only
  markers attached by fixed joints are preserved with a warning; no mass is invented;
- valid mimic targets without mimic cycles;
- referenced, nonempty local STL and texture assets.

Missing revolute/prismatic limits are recorded as warnings and remain missing;
the tool does not invent effort, velocity, or travel values. Non-unit moving
axes also produce a warning and are preserved. A geometry-bearing link or a
moving link needs explicit physical inertial data in this first beta.

The original input files are preserved. A successful `prepare` creates:

- `robot.urdf`: the same names, graph, origins, axes, limits, inertias, materials,
  and primitive geometries, with asset references relocated and numeric
  attributes written as finite decimals without six-place rounding;
- `assets/<sha256>/<original-name>`: byte-identical referenced meshes/textures;
- `provenance/source-urdf.xml`: the original selected URDF bytes;
- `acdc4robot-onshape-report.json`: source/archive/asset/output hashes, tool
  version, validation warnings, and supplied source configuration.

XML formatting may change; comments are retained. Inputs with unexpanded Xacro,
XML namespaces, DTDs, or entities are rejected. The core does not inspect STL
solid topology or compare geometry and inertias against Onshape CAD; optional
collision processing has stronger solid-mesh checks.

Preparation validates before writing, stages a complete output beside the
requested destination, and publishes it only after success. Existing outputs
are rejected. Failed preparation cleans staging without publishing a partial
bundle. ZIP extraction rejects traversal, symlinks/special files, case-colliding
entries, and archives exceeding 10000 entries or 512 MiB uncompressed. URDF XML
is limited to 16 MiB.

## Optional shared tools

The package vendors the independent MIT utilities from ACDC4Robot/Fusion360;
source commit and hashes are recorded in `shared-source.json` and the installed
`shared/source.json`. See `THIRD_PARTY_NOTICES.md`. No Fusion module is loaded.

```bash
python -m pip install '.[collision]'
acdc4robot-convexify prepared/robot.urdf robot-convex --threshold 0.05 --seed 0
```

This creates another new bundle with separate convex collision pieces, metre
scale handling, original collision origins, and provenance. It supports closed
outward-wound STL solids and rejects invalid backend pieces rather than
repairing or dropping them. It does not invent collisions from visual-only
links. See the upstream
[collision guide](https://github.com/ACDC4Robot/Fusion360/blob/main/docs/COLLISION_MESHES.md).

The optional `isaac` extra exposes `acdc4robot-isaac-check` for the existing
isolated Isaac Sim 6.2 import smoke test. It requires NVIDIA packages and a
supported Python/platform environment. Pass `--allow-massless-fixed-frames`
for native exports with geometry-free fixed/root reference frames. The full
companion validator checks these exemptions first; physical links still require
valid inertials. Exempt frames are reported, and no fake inertias are added.
It checks USD asset conversion and does not
step a physics engine. Do not use `--require-decimal` as an Onshape acceptance
requirement; it is a serialization-specific diagnostic.

## Tests and release boundary

```bash
python -m unittest discover -s tests -v
python -m pip wheel --no-deps . -w dist
```

Tests cover nested bundle layouts, ZIP handling, explicit package roots,
asset containment, graph and physics failures, source integrity, preserved
transforms/materials/bytes, massless fixed frames, limit warnings, tiny decimal
precision, repeatable provenance, and atomic failure cleanup. The package can
be tested without an Onshape account, Fusion, NumPy, or a simulator.

Native Onshape 1.222 exports have now passed one-part and repeated-instance
revolute-joint intake checks. Broader release acceptance still requires native
STL exports from known
Onshape versions/configurations and compare joints, mass, CoM, full inertia
orientation, and geometry bounds with CAD. API acquisition and simulator
acceptance remain separate gates. MIT license; see `LICENSE`.

## Verified native export example (Onshape 1.222)

The original archive contains `assembly_1/urdf/assembly_1.urdf` and
`assembly_1/meshes/Part_1.stl`, referenced using `package://assembly_1/...`.
Extract the trusted Onshape archive into its own directory, then explicitly map
that package root (absolute paths are also accepted):

```bash
acdc4robot-onshape validate exported --package assembly_1=exported/assembly_1
acdc4robot-onshape prepare exported prepared --package assembly_1=exported/assembly_1
acdc4robot-isaac-check prepared/robot.urdf isaac-check --allow-massless-fixed-frames
```

The one-part and two-instance revolute fixtures passed isolated Isaac 6.2
conversion. The articulated export has five links (including reference frames),
four joints, and one STL reused by both solid instances. Native source angular
limits of -45/+60 degrees survived conversion within native-export precision.
These exports contain visual meshes but **no collision geometry**. Collision
creation, contact dynamics, motor gains, prismatic joints, and larger nested
robots remain outside this validation. The importer warns that this revolute
joint has no actuator stiffness/damping. No physics simulation was run.

`live_onshape_validation` in an automatically generated preparation report stays
false because offline input cannot prove its origin. The separately recorded
native test evidence describes what was actually verified.
