from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class BenchmarkCase:
    relative_path: str
    description: str
    min_regions: int
    host_minimum_outcome: str
    docker_full_minimum_outcome: str | None = None


BENCHMARK_CASES = [
    BenchmarkCase(
        "examples/benchmark/3mf_samples/core_box.3mf",
        "basic core 3MF ingest",
        6,
        "solid_created",
    ),
    BenchmarkCase(
        "examples/benchmark/3mf_samples/core_cylinder.3mf",
        "single curved body through 3MF",
        3,
        "solid_created",
    ),
    BenchmarkCase(
        "examples/benchmark/3mf_samples/core_multiple_cylinders.3mf",
        "multiple cylindrical build items",
        6,
        "solid_created",
    ),
    BenchmarkCase(
        "examples/benchmark/3mf_samples_hard/cube_gears.3mf",
        "multi-body gear assembly",
        0,
        "shell_only",
        docker_full_minimum_outcome="solid_created",
    ),
    BenchmarkCase(
        "examples/benchmark/3mf_samples_hard/heartgears.3mf",
        "dense single-body gear heart",
        0,
        "solid_created",
    ),
    BenchmarkCase(
        "examples/benchmark/cloudgripper/xy_rail_mount.stl",
        "mechanical mount with cutouts",
        6,
        "solid_created",
    ),
    BenchmarkCase(
        "examples/benchmark/cloudgripper/arm_holder.stl",
        "asymmetric holder geometry",
        2,
        "solid_created",
    ),
    BenchmarkCase(
        "examples/benchmark/cloudgripper/xy_nema_bracket.stl",
        "bracket with multiple openings",
        25,
        "solid_created",
    ),
    BenchmarkCase(
        "examples/benchmark/cloudgripper/arm_linear_pinion_gear.stl",
        "repeated gear teeth around a bore",
        25,
        "solid_created",
    ),
    BenchmarkCase(
        "examples/benchmark/bcn3d_moveo/t4m1e.stl",
        "robotic-arm articulation geometry",
        25,
        "solid_created",
    ),
    BenchmarkCase(
        "examples/benchmark/fdm_screws/fdm_nut_and_bolt.stl",
        "threaded FDM-style nut and bolt pair",
        0,
        "solid_created",
    ),
]


def minimum_outcome_for_profile(case: BenchmarkCase, profile: str) -> str:
    if profile == "docker-full" and case.docker_full_minimum_outcome is not None:
        return case.docker_full_minimum_outcome
    return case.host_minimum_outcome
