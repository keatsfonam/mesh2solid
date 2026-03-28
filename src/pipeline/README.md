This directory holds private implementation fragments included by [`src/pipeline.cpp`](/Users/mkeats/Documents/New%20project/src/pipeline.cpp).

Why this exists:
- `pipeline.cpp` had grown past 5k lines and was becoming difficult to navigate.
- The current refactor keeps the implementation in a single translation unit, which minimizes behavior risk while splitting the code by concern.

Fragment layout:
- `io.inc`: XML, ZIP/3MF, STL, and mesh loading helpers
- `cgal.inc`: optional CGAL-backed plane regularization hooks
- `reconstruction.inc`: plane fitting, region constraint handling, loop recovery, shell reconstruction, and gap repair
- `fallback.inc`: faceted mesh fallback reconstruction for closed repaired meshes
- `output.inc`: STEP/STL emission and JSON/debug/report serialization orchestration
- `output_cylinder_solids.inc`: analytic cylinder-solid STEP writers
- `output_bores.inc`: single- and multi-bore prismatic STEP writers
- `output_blind_features.inc`: blind bore/counterbore/countersink STEP writers, including repeated blind-feature variants
- `output_supports.inc`: boss, multi-boss, and standoff STEP writers
- `output_counterbore.inc`: through-counterbore STEP writer
- `output_countersink.inc`: through-countersink STEP writer
- `output_step_loop.inc`: shared STEP loop, edge, and planar-face writing helpers for analytic export families
- `output_axial.inc`: shared-cap axial through-feature STEP writers
- `output_cylindrical.inc`: mixed cylindrical-feature STEP writers
- `output_obround.inc`: obround-slot STEP writer
- `output_fallback_step.inc`: generic planar STEP fallback writer used when no analytic export family matches
- `output_reporting.inc`: STL emission plus JSON/debug/report serialization helpers

This is an internal organization tool, not a public API boundary.
