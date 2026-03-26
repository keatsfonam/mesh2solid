# Benchmark Corpus

The automated tests generate representative mechanical fixtures at runtime, but the intended
benchmark corpus for manual evaluation should include:

- clean box and enclosure-like parts
- mildly noisy prismatic parts
- parts with small chamfers or fillets that should remain unresolved in v1
- intentionally bad meshes with holes, duplicate faces, and tiny disconnected fragments

The acceptance target for this MVP is deterministic planar recognition with clear diagnostics,
and solid export only when the reconstructed shell closes cleanly.

