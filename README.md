# `stl2solid`

`stl2solid` is a local STL-to-solid assistant aimed at mechanical, mostly planar parts.
It is built as a reusable C++ core with a CLI first, and keeps clear seams for a future
Fusion 360 adapter and optional CGAL/Open CASCADE backends.

## Current MVP

- Reads ASCII and binary STL.
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
build/stl2solid analyze part.stl --out out --preset mechanical --solid-threshold 0.75
```

## Test Baseline

The repo includes a checked-in cube fixture and exact golden outputs under `tests/golden/cube`.

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
- `include/stl2solid/pipeline.h`
  - public types and pipeline entrypoints

## Fusion 360 path

This repo intentionally keeps Fusion 360 out of the geometry core. The expected phase-2
adapter will:

- invoke the core engine from a Fusion add-in or native module
- preview recognized planes and confidence
- import the reconstructed body into Fusion
- persist any final BRep body through Fusion APIs instead of relying on UI-only mesh conversion
