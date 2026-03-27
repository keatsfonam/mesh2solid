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

That uses the supported `docker-full` expectation profile, which requires every checked-in
benchmark, including `cube_gears.3mf`, to reach a clean `solid_created` result with no
reported `failure_reasons`.

`make docker-hard-bench` remains as an alias for the same command.

For the lightweight internal smoke benchmark pass instead:

```bash
make smoke-bench
```

That uses the lightweight `host-minimal` expectation profile. It is kept only for fast local
smoke checks while iterating and still allows `cube_gears.3mf` to stay at `shell_only`.
