This directory holds private implementation fragments included by [`src/pipeline.cpp`](/Users/mkeats/Documents/New%20project/src/pipeline.cpp).

Why this exists:
- `pipeline.cpp` had grown past 5k lines and was becoming difficult to navigate.
- The current refactor keeps the implementation in a single translation unit, which minimizes behavior risk while splitting the code by concern.

Fragment layout:
- `io.inc`: XML, ZIP/3MF, STL, and mesh loading helpers
- `reconstruction.inc`: plane fitting, region constraint handling, loop recovery, shell reconstruction, and gap repair
- `output.inc`: STEP/STL emission and JSON/debug/report serialization

This is an internal organization tool, not a public API boundary.
