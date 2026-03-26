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
- Builds a constraint graph for coplanar, parallel, and perpendicular plane relations.
- Reconstructs planar shell faces from snapped boundary loops.
- Emits:
  - `cleaned_mesh.stl`
  - `report.json`
  - `regions.json`
  - `constraints.json`
  - `reconstruction_debug.json`
  - `reconstruction.step` when the planar shell validates as a closed solid

The current code ships with an internal fallback geometry path so it can build in a bare
workspace. `CMakeLists.txt` is already prepared to link CGAL and Open CASCADE later when
those libraries are available.

## Build

With the toolchain available in this workspace:

```bash
make
```

If you have CMake and optional geometry libraries installed:

```bash
cmake -S . -B build-cmake
cmake --build build-cmake
```

## Usage

```bash
build/mesh2solid analyze part.stl --out out --preset mechanical --solid-threshold 0.75
build/mesh2solid analyze part.3mf --out out --preset mechanical --solid-threshold 0.75
```

## Test Baseline

The repo includes checked-in planar fixtures, including a through-hole tube, and exact golden
outputs under `tests/golden/`. It also includes a more complex bridge example under
`examples/complex/bridge.stl` as a smoke regression for localized shell healing.

Refresh the golden baseline with:

```bash
python3 tests/update_goldens.py
```

or:

```bash
make golden
```

## Architecture

- `src/pipeline.cpp`
  - mesh ingest and repair
  - planar segmentation
  - constraint solving
  - reconstruction
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
