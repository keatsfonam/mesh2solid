# Example Corpus

This directory holds checked-in benchmark meshes that exercise `mesh2solid`
beyond the tiny golden fixtures in `tests/fixtures/`.

## Complex

- `complex/bridge.stl`
  - existing in-repo regression for localized shell healing on a larger planar
    structure
  - current expected outcome: `solid_created`

## Public Benchmark Set

- `benchmark/3mf_samples/core_box.3mf`
  - source: 3MF Consortium `3mf-samples`
  - license: BSD-2-Clause
  - feature focus: basic core `3MF` ingest
  - current expected outcome: `solid_created`
- `benchmark/3mf_samples/core_cylinder.3mf`
  - source: 3MF Consortium `3mf-samples`
  - license: BSD-2-Clause
  - feature focus: curved faceted surface imported through `3MF`
  - current expected outcome: `solid_created`
- `benchmark/3mf_samples/core_multiple_cylinders.3mf`
  - source: 3MF Consortium `3mf-samples`
  - license: BSD-2-Clause
  - feature focus: `3MF` build items and multiple cylindrical bodies
  - current expected outcome: `solid_created`
- `benchmark/3mf_samples_hard/cube_gears.3mf`
  - source: 3MF Consortium `3mf-samples`
  - license: BSD-2-Clause
  - feature focus: multi-body gear assembly imported from `3MF`
  - current expected outcome:
    - `host-minimal`: `shell_only`
    - `docker-full`: `solid_created`
- `benchmark/3mf_samples_hard/heartgears.3mf`
  - source: 3MF Consortium `3mf-samples`
  - license: BSD-2-Clause
  - feature focus: dense single-body gear geometry with repeated teeth
  - current expected outcome: `solid_created`
- `benchmark/cloudgripper/xy_rail_mount.stl`
  - source: CloudGripper Robot
  - license: MIT
  - feature focus: mechanical mount with cutouts and mounting faces
  - current expected outcome: `solid_created`
- `benchmark/cloudgripper/arm_holder.stl`
  - source: CloudGripper Robot
  - license: MIT
  - feature focus: asymmetric holder geometry
  - current expected outcome: `solid_created`
- `benchmark/cloudgripper/xy_nema_bracket.stl`
  - source: CloudGripper Robot
  - license: MIT
  - feature focus: bracket topology with multiple openings and small details
  - current expected outcome: `solid_created`
- `benchmark/cloudgripper/arm_linear_pinion_gear.stl`
  - source: CloudGripper Robot
  - license: MIT
  - feature focus: repeated gear-tooth structure around a bore
  - current expected outcome: `solid_created`
- `benchmark/bcn3d_moveo/t4m1e.stl`
  - source: BCN3D Moveo
  - license: MIT
  - feature focus: more complex robotic-arm articulation geometry
  - current expected outcome: `solid_created`
- `benchmark/fdm_screws/fdm_nut_and_bolt.stl`
  - source: mechadense `scad-lib-FDMscrews`
  - license: LGPL-3.0
  - feature focus: printable threaded nut-and-bolt pair
  - current expected outcome: `solid_created`

## Manual Stress Cases

The repo also uses larger public mechanical meshes during local robustness work, but not all of
them are checked in. Some are intentionally left as manually downloaded stress cases so we can
avoid ballooning the repository or pulling in assets with unclear redistribution terms. Those are
good candidates for ad hoc Docker runs with `python3 benchmarks/run_examples.py` or one-off CLI
invocations when we are pushing on fallback robustness.

Repository sources:

- 3MF samples: <https://github.com/3MFConsortium/3mf-samples>
- CloudGripper Robot: <https://github.com/cloudgripper/cloudgripper-robot>
- BCN3D Moveo: <https://github.com/BCN3D/BCN3D-Moveo>
- scad-lib-FDMscrews: <https://github.com/mechadense/scad-lib-FDMscrews>
