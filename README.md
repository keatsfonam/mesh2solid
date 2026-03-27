# `mesh2solid`

`mesh2solid` is a local mesh-to-solid assistant aimed at mechanical, mostly planar parts.
It is built as a reusable C++ core with a CLI first, and keeps clear seams for a future
Fusion 360 adapter and optional CGAL/Open CASCADE backends.

The current MVP reads STL and core mesh `3MF`. The project name is broader because we want
to keep expanding mesh inputs without renaming the core tool again later.

## Current MVP

- Reads ASCII and binary STL.
- Reads core mesh `3MF` packages with mesh objects, component references, build items, and unit scaling.
- Repairs basic mesh issues:
  - vertex welding
  - duplicate/degenerate face removal
  - tiny component pruning
  - triangle orientation normalization
- Detects planar regions using adjacency-driven region growing and plane fitting.
- Optionally regularizes detected planes with CGAL before constraint solving when that backend is available.
- Builds a constraint graph for coplanar, parallel, and perpendicular plane relations.
- Reconstructs planar shell faces from snapped boundary loops.
- Upgrades simple cylindrical solids and disconnected cylinder assemblies to true curved STEP surfaces instead of planar side facets.
- Falls back to a faceted mesh B-Rep path when the repaired mesh is closed but the analytic planar path cannot confidently emit a solid.
- Emits:
  - `cleaned_mesh.stl`
  - `report.json`
  - `regions.json`
  - `constraints.json`
  - `reconstruction_debug.json`
  - `reconstruction.step` when the planar shell validates as a closed solid

The current code ships with an internal fallback geometry path so it can build in a bare
workspace. `CMakeLists.txt` is already prepared to link CGAL and Open CASCADE later when
those libraries are available. In a bare build, plane regularization is reported as `none`
and the fallback solid path uses the internal faceted reconstruction flow.

## Build

Docker is the primary Linux development workflow going forward. It gives us a reproducible
environment with CGAL and Open CASCADE installed without polluting the host.

Build the Docker image and the full-stack binary:

```bash
make docker-build
```

Build the image and run the full-stack test suite:

```bash
make docker-test
```

Run the broader checked-in benchmark corpus against the host build:

```bash
make hard-bench
```

That enforces the `host-minimal` benchmark expectation profile.

Run that same corpus against the Docker full-stack build:

```bash
make docker-hard-bench
```

That enforces the stricter `docker-full` expectation profile, where every checked-in benchmark is
expected to reach a clean `solid_created` result with no recorded `failure_reasons`.

Open a shell inside the container:

```bash
make docker-shell
```

The Docker workflow uses an explicit full-stack CMake configure so it behaves consistently
inside the container even when the workspace starts clean. The matching `docker-full` preset
is still available in `CMakePresets.json` for manual local use.

For a lightweight host-only fallback build:

```bash
make
```

That produces:

```bash
build/mesh2solid
```

For the host minimal CMake path:

```bash
cmake --preset host-minimal
cmake --build --preset host-minimal
```

If you have CGAL and Open CASCADE installed on the host and want the full stack without Docker:

```bash
cmake --preset docker-full
cmake --build --preset docker-full
```

## Usage

```bash
build/mesh2solid analyze part.stl --out out --preset mechanical --solid-threshold 0.75
build/mesh2solid analyze part.3mf --out out --preset mechanical --solid-threshold 0.75
```

For the Docker full-stack binary after `make docker-build`:

```bash
build-cmake/docker-full/mesh2solid analyze part.stl --out out --preset mechanical --solid-threshold 0.75
build-cmake/docker-full/mesh2solid analyze part.3mf --out out --preset mechanical --solid-threshold 0.75
```

Each analysis run writes:

- `cleaned_mesh.stl`
- `report.json`
- `regions.json`
- `constraints.json`
- `reconstruction_debug.json`
- `reconstruction.step` when reconstruction reaches `solid_created`

## CI And Release

GitHub Actions now builds and tests the project on Linux and macOS for pushes, pull requests,
and manual runs.

Tagged releases publish prebuilt archives to GitHub Releases. To cut a release:

```bash
git tag v0.1.0
git push origin v0.1.0
```

That tag triggers the release workflow, which rebuilds the binary, reruns the test suite,
packages `mesh2solid`, and uploads platform archives to the matching GitHub release.

## Test Baseline

The repo includes checked-in planar fixtures, including a through-hole tube, and exact golden
outputs under `tests/golden/`. It also includes a more complex bridge example under
`examples/complex/bridge.stl` as a smoke regression for localized shell healing.

The test suite also generates a deterministic prismatic corpus at runtime, covering stepped
blocks, L-brackets, channels, disconnected box assemblies, cross-shaped solids, multi-item
3MF box assemblies, and prismatic blocks with analytic cylindrical through-bores, including
rotated off-axis, off-center, and varied aspect-ratio cases, plus blind cylindrical pockets,
blind counterbores, simple counterbores, cylindrical bosses/posts, and standoff-style tubular
bosses with concentric through-holes, plus obround through-slots with analytic half-cylinders.

Beyond that tight golden set, `examples/README.md` documents a broader public benchmark corpus
with upstream STL and 3MF models. Those smoke tests intentionally keep separate expectation
profiles for the lightweight host build and the Docker full-stack build, so we can expand
robustness without weakening the simpler baseline.

`examples/benchmark/` adds a broader public corpus with both current-success and current-hard
cases from the 3MF Consortium, CloudGripper Robot, and BCN3D Moveo repositories. Those smoke
tests lock in expected outcomes while we expand robustness without regressing the simpler
fixtures, and the Docker full-stack profile now requires every checked-in benchmark to reach
`solid_created`.

Refresh the golden baseline with:

```bash
python3 tests/update_goldens.py
```

or:

```bash
make golden
```

When using a non-default binary, both test helpers honor:

```bash
MESH2SOLID_BIN=/path/to/mesh2solid
MESH2SOLID_SKIP_BUILD=1
```

The manual benchmark runner uses the same environment variables:

```bash
MESH2SOLID_BIN=/path/to/mesh2solid MESH2SOLID_SKIP_BUILD=1 python3 benchmarks/run_examples.py
```

## Architecture

- `src/pipeline.cpp`
  - mesh ingest and repair
  - planar segmentation
  - optional plane regularization hook
  - constraint solving
  - reconstruction
  - faceted fallback selection
  - STEP and JSON emission
- `src/main.cpp`
  - CLI surface
- `include/mesh2solid/pipeline.h`
  - public types and pipeline entrypoints

## Fusion 360 path

This repo intentionally keeps Fusion 360 out of the geometry core. The expected phase-2
adapter will:

- invoke the core engine from a Fusion add-in or native module
- preview recognized planes and confidence
- import the reconstructed body into Fusion
- persist any final BRep body through Fusion APIs instead of relying on UI-only mesh conversion
