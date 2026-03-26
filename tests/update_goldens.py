#!/usr/bin/env python3

from __future__ import annotations

import argparse
import pathlib
import shutil
import subprocess
import sys
import tempfile


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
BIN_PATH = REPO_ROOT / "build" / "stl2solid"
FIXTURES_DIR = REPO_ROOT / "tests" / "fixtures"
GOLDEN_ROOT = REPO_ROOT / "tests" / "golden"

FIXTURE_CONFIG = {
    "cube": {
        "input": pathlib.Path("tests/fixtures/cube.stl"),
        "preset": "mechanical",
        "solid_threshold": "0.60",
        "outputs": [
            "cleaned_mesh.stl",
            "report.json",
            "regions.json",
            "constraints.json",
            "reconstruction.step",
        ],
    },
    "rectangular_box": {
        "input": pathlib.Path("tests/fixtures/rectangular_box.stl"),
        "preset": "mechanical",
        "solid_threshold": "0.60",
        "outputs": [
            "cleaned_mesh.stl",
            "report.json",
            "regions.json",
            "constraints.json",
            "reconstruction.step",
        ],
    },
    "sloped_block": {
        "input": pathlib.Path("tests/fixtures/sloped_block.stl"),
        "preset": "mechanical",
        "solid_threshold": "0.60",
        "outputs": [
            "cleaned_mesh.stl",
            "report.json",
            "regions.json",
            "constraints.json",
            "reconstruction.step",
        ],
    },
    "triangular_prism": {
        "input": pathlib.Path("tests/fixtures/triangular_prism.stl"),
        "preset": "mechanical",
        "solid_threshold": "0.60",
        "outputs": [
            "cleaned_mesh.stl",
            "report.json",
            "regions.json",
            "constraints.json",
            "reconstruction.step",
        ],
    },
}


def run(cmd: list[str], *, cwd: pathlib.Path) -> None:
    completed = subprocess.run(
        cmd,
        cwd=cwd,
        text=True,
        capture_output=True,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"Command failed: {' '.join(cmd)}\n"
            f"STDOUT:\n{completed.stdout}\n"
            f"STDERR:\n{completed.stderr}"
        )


def ensure_binary(build: bool) -> None:
    if BIN_PATH.exists() and not build:
        return
    run(["make"], cwd=REPO_ROOT)


def update_fixture(name: str) -> None:
    config = FIXTURE_CONFIG[name]
    golden_dir = GOLDEN_ROOT / name
    golden_dir.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = pathlib.Path(tmp)
        out_dir = tmp_path / "out"
        run(
            [
                str(BIN_PATH),
                "analyze",
                str(config["input"]),
                "--out",
                str(out_dir),
                "--preset",
                str(config["preset"]),
                "--solid-threshold",
                str(config["solid_threshold"]),
            ],
            cwd=REPO_ROOT,
        )

        for output_name in config["outputs"]:
            source = out_dir / output_name
            if not source.exists():
                raise RuntimeError(f"Expected output was not produced for {name}: {output_name}")
            shutil.copyfile(source, golden_dir / output_name)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Refresh checked-in golden outputs.")
    parser.add_argument(
        "fixtures",
        nargs="*",
        default=sorted(FIXTURE_CONFIG.keys()),
        help="Fixture names to refresh",
    )
    parser.add_argument(
        "--no-build",
        action="store_true",
        help="Skip rebuilding the CLI before refreshing goldens",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    unknown = [fixture for fixture in args.fixtures if fixture not in FIXTURE_CONFIG]
    if unknown:
        raise RuntimeError(
            f"Unknown fixture(s): {', '.join(unknown)}. Known fixtures: {', '.join(sorted(FIXTURE_CONFIG))}"
        )
    ensure_binary(build=not args.no_build)
    for fixture_name in args.fixtures:
        update_fixture(fixture_name)
        print(f"Updated golden outputs for {fixture_name}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # pragma: no cover
        print(f"update_goldens failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
