# Benchmark Corpus

The automated tests generate representative mechanical fixtures at runtime, but the intended
benchmark corpus for manual evaluation should include:

- clean box and enclosure-like parts
- mildly noisy prismatic parts
- parts with small chamfers or fillets that should remain unresolved in v1
- intentionally bad meshes with holes, duplicate faces, and tiny disconnected fragments

The acceptance target for this MVP is deterministic planar recognition with clear diagnostics,
and solid export only when the reconstructed shell closes cleanly.

Run the checked-in public benchmark corpus with:

```bash
make hard-bench
```

Or run the same set against the Docker full-stack build with:

```bash
make docker-hard-bench
```

That benchmark runner treats the harder cases as minimum-outcome checks. Known difficult meshes
such as the new gear-heavy `3MF` samples are allowed to stay at `shell_only`, while the already
working mechanical parts are still required to reach `solid_created`.
