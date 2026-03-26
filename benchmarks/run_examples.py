#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import os
import pathlib
import shutil
import subprocess
import sys
from dataclasses import dataclass


ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_OUT_DIR = ROOT / "tmp" / "benchmark_runs"
OUTCOME_RANK = {
    "analysis_only": 0,
    "shell_only": 1,
    "solid_created": 2,
}


@dataclass(frozen=True)
class BenchmarkCase:
    relative_path: str
    minimum_outcome: str
    description: str

    @property
    def path(self) -> pathlib.Path:
        return ROOT / self.relative_path

    @property
    def stem(self) -> str:
        return self.path.stem


BENCHMARK_CASES = [
    BenchmarkCase(
        "examples/benchmark/3mf_samples/core_box.3mf",
        "solid_created",
        "basic core 3MF ingest",
    ),
    BenchmarkCase(
        "examples/benchmark/3mf_samples/core_cylinder.3mf",
        "solid_created",
        "single curved body through 3MF",
    ),
    BenchmarkCase(
        "examples/benchmark/3mf_samples/core_multiple_cylinders.3mf",
        "solid_created",
        "multiple cylindrical build items",
    ),
    BenchmarkCase(
        "examples/benchmark/3mf_samples_hard/cube_gears.3mf",
        "shell_only",
        "multi-body gear assembly",
    ),
    BenchmarkCase(
        "examples/benchmark/3mf_samples_hard/heartgears.3mf",
        "solid_created",
        "dense single-body gear heart",
    ),
    BenchmarkCase(
        "examples/benchmark/cloudgripper/xy_rail_mount.stl",
        "solid_created",
        "mechanical mount with cutouts",
    ),
    BenchmarkCase(
        "examples/benchmark/cloudgripper/arm_holder.stl",
        "solid_created",
        "asymmetric holder geometry",
    ),
    BenchmarkCase(
        "examples/benchmark/cloudgripper/xy_nema_bracket.stl",
        "solid_created",
        "bracket with multiple openings",
    ),
    BenchmarkCase(
        "examples/benchmark/cloudgripper/arm_linear_pinion_gear.stl",
        "solid_created",
        "repeated gear teeth around a bore",
    ),
    BenchmarkCase(
        "examples/benchmark/bcn3d_moveo/t4m1e.stl",
        "solid_created",
        "robotic-arm articulation geometry",
    ),
    BenchmarkCase(
        "examples/benchmark/fdm_screws/fdm_nut_and_bolt.stl",
        "solid_created",
        "threaded FDM-style nut and bolt pair",
    ),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the checked-in benchmark corpus.")
    parser.add_argument(
        "--bin",
        dest="binary",
        default=os.environ.get("MESH2SOLID_BIN", str(ROOT / "build" / "mesh2solid")),
        help="Path to the mesh2solid binary to execute.",
    )
    parser.add_argument(
        "--out",
        dest="output_dir",
        default=str(DEFAULT_OUT_DIR),
        help="Directory that will receive per-example outputs.",
    )
    parser.add_argument(
        "--filter",
        action="append",
        default=[],
        help="Substring filter for benchmark paths or descriptions. May be passed more than once.",
    )
    parser.add_argument(
        "--solid-threshold",
        type=float,
        default=0.60,
        help="Solid threshold to pass through to analyze.",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        default=os.environ.get("MESH2SOLID_SKIP_BUILD") == "1",
        help="Skip the default host build step.",
    )
    return parser.parse_args()


def ensure_binary(binary: pathlib.Path, skip_build: bool) -> None:
    if skip_build:
        return

    default_binary = ROOT / "build" / "mesh2solid"
    if binary != default_binary:
        if not binary.exists():
            raise FileNotFoundError(f"Configured binary does not exist: {binary}")
        return

    subprocess.run(["make"], cwd=ROOT, check=True)


def selected_cases(filters: list[str]) -> list[BenchmarkCase]:
    if not filters:
        return BENCHMARK_CASES

    lowered = [item.lower() for item in filters]
    cases = []
    for case in BENCHMARK_CASES:
        haystack = f"{case.relative_path} {case.description}".lower()
        if any(item in haystack for item in lowered):
            cases.append(case)
    return cases


def run_case(case: BenchmarkCase,
             binary: pathlib.Path,
             output_dir: pathlib.Path,
             solid_threshold: float) -> dict[str, object]:
    case_out = output_dir / case.stem
    if case_out.exists():
        shutil.rmtree(case_out)

    subprocess.run(
        [
            str(binary),
            "analyze",
            str(case.path),
            "--out",
            str(case_out),
            "--preset",
            "mechanical",
            "--solid-threshold",
            f"{solid_threshold:.2f}",
        ],
        cwd=ROOT,
        check=True,
        stdout=subprocess.DEVNULL,
    )

    report_path = case_out / "report.json"
    with report_path.open() as handle:
        report = json.load(handle)

    reconstruction = report["reconstruction"]
    actual_outcome = reconstruction["outcome"]
    minimum_ok = OUTCOME_RANK[actual_outcome] >= OUTCOME_RANK[case.minimum_outcome]
    return {
        "case": case,
        "report": report,
        "actual_outcome": actual_outcome,
        "minimum_ok": minimum_ok,
    }


def main() -> int:
    args = parse_args()
    binary = pathlib.Path(args.binary).resolve()
    out_dir = pathlib.Path(args.output_dir).resolve()
    cases = selected_cases(args.filter)

    if not cases:
        print("No benchmark cases matched the supplied filters.", file=sys.stderr)
        return 2

    ensure_binary(binary, args.skip_build)
    out_dir.mkdir(parents=True, exist_ok=True)

    results = []
    for case in cases:
        results.append(run_case(case, binary, out_dir, args.solid_threshold))

    failures = 0
    for result in results:
        case = result["case"]
        report = result["report"]
        reconstruction = report["reconstruction"]
        ok = result["minimum_ok"]
        status = "PASS" if ok else "FAIL"
        if not ok:
            failures += 1

        print(f"{status} {case.relative_path}")
        print(f"  minimum outcome: {case.minimum_outcome}")
        print(f"  actual outcome:  {result['actual_outcome']}")
        print(f"  method:          {reconstruction['method']}")
        print(f"  backend:         {report['backend']}")
        print(f"  description:     {case.description}")
        if reconstruction.get("failure_reasons"):
            print(f"  notes:           {', '.join(reconstruction['failure_reasons'])}")

    print()
    print(f"Benchmarks run: {len(results)}")
    print(f"Failures: {failures}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
