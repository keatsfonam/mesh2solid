# `mesh2solid`

`mesh2solid` is a local mesh-to-solid assistant aimed at mechanical, mostly planar parts.
It is built as a reusable C++ core with a CLI first, and keeps clear seams for a future
Fusion 360 adapter. The supported shipped binary is the Docker-built full-stack variant with
CGAL and Open CASCADE enabled.

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

The repo still keeps a lightweight internal fallback path so we can do fast smoke builds on
host machines without the full geometry stack. That host-minimal path is an internal
development aid now, not the supported release/runtime configuration.

## Build

Docker is the primary Linux development workflow going forward. It gives us a reproducible
environment with CGAL and Open CASCADE installed without polluting the host, and the
full-stack container build uses `clang++` so the large geometry translation unit compiles
reliably with the full dependency stack enabled.

Build the Docker image and the full-stack binary:

```bash
make docker-build
```

Build the image and run the full-stack test suite:

```bash
make docker-test
```

Run the broader checked-in benchmark corpus against the supported full-stack build:

```bash
make hard-bench
```

That enforces the `docker-full` expectation profile, where every checked-in benchmark is
expected to reach a clean `solid_created` result with no recorded `failure_reasons`.

`make docker-hard-bench` remains as an alias for the same command.

If you want the internal lightweight smoke benchmark pass instead:

```bash
make smoke-bench
```

That uses the legacy `host-minimal` expectation profile and is only meant for quick local
regression checks while iterating.

Open a shell inside the container:

```bash
make docker-shell
```

The Docker workflow uses an explicit full-stack CMake configure so it behaves consistently
inside the container even when the workspace starts clean. The matching `docker-full` preset
is still available in `CMakePresets.json` for manual local use.

For a lightweight host-only smoke build:

```bash
make
```

That produces:

```bash
build/mesh2solid
```

For the host minimal CMake smoke path:

```bash
cmake --preset host-minimal
cmake --build --preset host-minimal
```

If you have CGAL and Open CASCADE installed on the host and want to reproduce the full stack without Docker:

```bash
cmake --preset docker-full
cmake --build --preset docker-full
```

## Usage

Primary supported usage is the Docker full-stack binary after `make docker-build`:

```bash
build-cmake/docker-full/mesh2solid analyze part.stl --out out --preset mechanical --solid-threshold 0.75
build-cmake/docker-full/mesh2solid analyze part.3mf --out out --preset mechanical --solid-threshold 0.75
```

The host binary is still available for smoke/debug work:

```bash
build/mesh2solid analyze part.stl --out out --preset mechanical --solid-threshold 0.75
build/mesh2solid analyze part.3mf --out out --preset mechanical --solid-threshold 0.75
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

Tagged releases publish the Docker-built full-stack Linux archive to GitHub Releases. To cut a release:

```bash
git tag v0.1.0
git push origin v0.1.0
```

That tag triggers the release workflow, which rebuilds the full-stack binary, reruns the
full-stack test and benchmark gates, packages `mesh2solid`, and uploads the resulting archive
to the matching GitHub release.

## Test Baseline

The repo includes checked-in planar fixtures, including a through-hole tube, and exact golden
outputs under `tests/golden/`. It also includes a more complex bridge example under
`examples/complex/bridge.stl` as a smoke regression for localized shell healing.

The test suite also generates a deterministic prismatic corpus at runtime, covering stepped
blocks, L-brackets, channels, disconnected box assemblies, cross-shaped solids, multi-item
3MF box assemblies, and prismatic blocks with analytic cylindrical through-bores, including
rotated off-axis, off-center, repeated multi-bore mounting patterns, repeated multi-counterbore mounting patterns, repeated multi-countersink mounting patterns, and varied aspect-ratio cases, plus blind cylindrical pockets,
blind counterbores, blind countersinks/conical seats, simple counterbores, simple countersinks/conical seats, cylindrical bosses/posts, repeated multi-boss pads,
and standoff-style tubular
bosses with concentric through-holes, plus repeated multi-blind-bore, repeated multi-blind-counterbore, and repeated multi-blind-countersink
pocket patterns, obround through-slots with analytic half-cylinders, and benchmark-derived
planar bodies whose faceted circular through-holes are upgraded to analytic cylinders at STEP export,
including bodies with multiple distinct cap-face pairs in the same part, mixed through-bore plus
counterbore and countersink blocks, mixed blind cylindrical features on more complex planar supports, and mixed
planar bodies whose faceted extruded side-profile
bands are upgraded to smooth STEP faces, using exact cylinders when the recovered profile is circular
and B-spline faces otherwise, including cap-pair bands that are not aligned to the world Z axis.

Beyond that tight golden set, `examples/README.md` documents a broader public benchmark corpus
with upstream STL and 3MF models. The supported benchmark gate is now the Docker full-stack
profile; the host-minimal profile remains only as an internal smoke check while we simplify
and trim legacy fallback behavior.

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
- `docs/architecture-v2.md`
  - current checkpoint and target geometry-first rebuild plan
  - canonical planar / prismatic sweep / revolved / blend / fallback branches
  - incremental migration strategy away from feature-by-feature exporters

Current implementation checkpoint:
- the shared-cap prismatic through-feature family now routes through a canonical axial section model
- single bores, repeated bores, countersinks, counterbores, and mixed shared-cap through features share one analytic writer
- blind bores, blind counterbores, blind countersinks, and their repeated variants now share a canonical blind-section model and writer
- bosses, repeated bosses, and standoffs now share a canonical support-section model and writer
- smooth extruded cap-pair side bands now route through a canonical profile-sweep band writer, and mixed cylindrical bodies reuse the same loop-subsequence replacement primitive when analytic sweep edges are stitched back into planar caps
- obround slots now route through an explicit profile-sweep slot model and share the common synthetic profile-loop STEP helper layer
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
