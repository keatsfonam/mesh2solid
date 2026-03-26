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

That uses the `host-minimal` expectation profile, which keeps the lightweight fallback baseline.

Or run the same set against the Docker full-stack build with:

```bash
make docker-hard-bench
```

That uses the `docker-full` expectation profile, which requires every checked-in benchmark,
including `cube_gears.3mf`, to reach a clean `solid_created` result with no reported
`failure_reasons`.

The benchmark runner treats harder cases as minimum-outcome checks per profile. The host-minimal
profile still allows `cube_gears.3mf` to stay at `shell_only`, while the already working
mechanical parts and `heartgears.3mf` are required to reach `solid_created`.
