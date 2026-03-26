#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import os
import pathlib
import shutil
import subprocess
import sys

if __package__:
    from .corpus import BENCHMARK_CASES, BenchmarkCase, minimum_outcome_for_profile
else:
    from corpus import BENCHMARK_CASES, BenchmarkCase, minimum_outcome_for_profile


ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_OUT_DIR = ROOT / "tmp" / "benchmark_runs"
OUTCOME_RANK = {
    "analysis_only": 0,
    "shell_only": 1,
    "solid_created": 2,
}

def case_path(case: BenchmarkCase) -> pathlib.Path:
    return ROOT / case.relative_path


def case_stem(case: BenchmarkCase) -> str:
    return case_path(case).stem


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
    parser.add_argument(
        "--expectation-profile",
        default=os.environ.get("MESH2SOLID_EXPECTATION_PROFILE", "host-minimal"),
        choices=["host-minimal", "docker-full"],
        help="Benchmark expectation profile to enforce.",
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
             solid_threshold: float,
             expectation_profile: str) -> dict[str, object]:
    case_out = output_dir / case_stem(case)
    if case_out.exists():
        shutil.rmtree(case_out)

    subprocess.run(
        [
            str(binary),
            "analyze",
            str(case_path(case)),
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
    expected_minimum = minimum_outcome_for_profile(case, expectation_profile)
    minimum_ok = OUTCOME_RANK[actual_outcome] >= OUTCOME_RANK[expected_minimum]
    clean_success_ok = not (
        actual_outcome == "solid_created" and bool(reconstruction.get("failure_reasons"))
    )
    return {
        "case": case,
        "report": report,
        "actual_outcome": actual_outcome,
        "expected_minimum": expected_minimum,
        "minimum_ok": minimum_ok,
        "clean_success_ok": clean_success_ok,
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

    results = [
        run_case(case, binary, out_dir, args.solid_threshold, args.expectation_profile)
        for case in cases
    ]

    failures = 0
    for result in results:
        case = result["case"]
        report = result["report"]
        reconstruction = report["reconstruction"]
        ok = result["minimum_ok"] and result["clean_success_ok"]
        status = "PASS" if ok else "FAIL"
        if not ok:
            failures += 1

        print(f"{status} {case.relative_path}")
        print(f"  minimum outcome: {result['expected_minimum']}")
        print(f"  actual outcome:  {result['actual_outcome']}")
        print(f"  method:          {reconstruction['method']}")
        print(f"  backend:         {report['backend']}")
        print(f"  description:     {case.description}")
        if not result["clean_success_ok"]:
            print("  clean success:   no (solid_created reported failure_reasons)")
        if reconstruction.get("failure_reasons"):
            print(f"  notes:           {', '.join(reconstruction['failure_reasons'])}")

    print()
    print(f"Expectation profile: {args.expectation_profile}")
    print(f"Benchmarks run: {len(results)}")
    print(f"Failures: {failures}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
