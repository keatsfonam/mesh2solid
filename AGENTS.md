# AGENTS.md

Guidance for agents working in this repository.

## Project Intent

`mesh2solid` converts triangle meshes into cleaner planar solid representations for CAD-adjacent workflows.

Current priorities:
- preserve the existing strong baseline for simple mechanical geometry
- improve robustness on more complex mechanical parts without regressing current successes
- keep the core reusable and local-first
- treat Fusion 360 integration as a downstream consumer, not the geometry engine itself

This repo currently supports STL and core mesh `3MF` input.

## Working Style

- Make small, reversible changes.
- Prefer milestone commits over large mixed edits.
- Do not weaken existing successful cases to improve one hard model.
- When changing reconstruction behavior, assume regressions are easy to introduce and verify broadly.
- Keep explanations and commit messages concrete. Mention the geometry class or failure mode being addressed.

## Build And Test

Primary local commands:

```bash
make
make test
make golden
```

Primary full-stack Linux workflow:

```bash
make docker-test
make docker-shell
```

Equivalent direct test command:

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
```

Use `make golden` only when output changes are intentional. Golden diffs are part of the contract.

Prefer the Docker workflow when working on CGAL/Open CASCADE integration or anything that depends
on the full geometry stack. Keep the plain `make` path working as the lightweight fallback build.

If you change geometry logic, run at least:
- `make`
- `python3 -m unittest discover -s tests -p 'test_*.py'`

If you change output formatting or deterministic ordering, also refresh and review goldens.

## Repo Map

- `src/main.cpp`
  - CLI entrypoint
- `include/mesh2solid/pipeline.h`
  - public pipeline types and API
- `src/pipeline.cpp`
  - top-level pipeline orchestration for the internal fallback backend
- `src/pipeline/io.inc`
  - STL, `3MF`, ZIP/XML, and mesh ingest helpers
- `src/pipeline/reconstruction.inc`
  - segmentation, constraints, reconstruction, shell healing, and validation
- `src/pipeline/output.inc`
  - STEP/STL/JSON/debug emission
- `tests/fixtures/`
  - small deterministic geometry inputs
- `tests/golden/`
  - checked-in exact expected outputs
- `examples/complex/`
  - larger in-repo regression cases
- `examples/benchmark/`
  - broader public benchmark corpus
- `.github/workflows/ci.yml`
  - build/test CI
- `.github/workflows/release.yml`
  - tagged release packaging

Keep the split pipeline layout intact unless there is a strong reason to reorganize it further. Do not collapse everything back into one giant file.

## Geometry Change Rules

- Prefer preserving analytic planar faces over emitting triangulated topology.
- Treat `solid_created` as meaningful only when shell closure is real, not just cosmetically patched.
- Be cautious with aggressive snapping, simplification, and gap healing. Small tolerance changes can break basic fixtures.
- Favor deterministic ordering and stable IDs where possible. Tests depend on consistent JSON and STEP output.
- When adding a heuristic, document the failure mode it is meant to solve in code comments or commit text.

When working on hard cases:
- isolate the failure mode first
- add or update a focused regression
- then broaden verification back across the full suite

## Tests And Benchmarks

There are two important layers of protection:

1. Tight golden fixtures in `tests/fixtures/` and `tests/golden/`
2. Broader smoke benchmarks in `examples/benchmark/`

Use both.

Expected benchmark intent:
- simple fixtures should remain exact and deterministic
- benchmark cases should remain at least as successful as they are today
- if a benchmark expectation changes, update the rationale in `examples/README.md`

Do not silently update goldens after a geometry change. Review the diff and make sure it reflects intentional behavior.

## Output Expectations

Typical successful run outputs:
- `cleaned_mesh.stl`
- `report.json`
- `regions.json`
- `constraints.json`
- `reconstruction_debug.json`
- `reconstruction.step` when reconstruction validates as a closed solid

For STEP output, prefer true analytic planar B-rep entities over faceted exports whenever possible.

## Dependencies

The default build uses the internal fallback geometry path.

Optional future backends:
- CGAL
- Open CASCADE

Do not make the core build depend on those libraries unless the change is explicitly intended and reflected in build/docs.

`3MF` support depends on `zlib`.

## CI And Release

- CI runs through GitHub Actions in `.github/workflows/ci.yml`
- tagged releases are packaged by `.github/workflows/release.yml`

If you change build commands, binary names, or test entrypoints, update the workflows in the same change.

## Documentation Hygiene

Update docs when behavior changes:
- `README.md` for user-facing capability or usage changes
- `examples/README.md` for benchmark corpus additions or expectation changes
- `src/pipeline/README.md` for internal pipeline layout changes

## Safe Default Approach

If you are unsure how to proceed:
1. reproduce the issue with a specific fixture or benchmark
2. add a narrow regression if one is missing
3. make the smallest geometry change that addresses that failure mode
4. run the full test suite
5. review goldens before finishing
