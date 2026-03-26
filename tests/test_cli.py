import json
import pathlib
import subprocess
import tempfile
import unittest
import difflib


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
BIN_PATH = REPO_ROOT / "build" / "stl2solid"
FIXTURES_DIR = REPO_ROOT / "tests" / "fixtures"
GOLDEN_DIR = REPO_ROOT / "tests" / "golden" / "cube"


def cube_mesh(size=10.0):
    vertices = [
        (0.0, 0.0, 0.0),
        (size, 0.0, 0.0),
        (size, size, 0.0),
        (0.0, size, 0.0),
        (0.0, 0.0, size),
        (size, 0.0, size),
        (size, size, size),
        (0.0, size, size),
    ]
    faces = [
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (3, 6, 2),
        (3, 7, 6),
        (0, 4, 7),
        (0, 7, 3),
        (1, 2, 6),
        (1, 6, 5),
    ]
    return vertices, faces


def cube_with_repair_noise():
    vertices, faces = cube_mesh()
    fragment_base = len(vertices)
    vertices.extend(
        [
            (25.0, 25.0, 25.0),
            (25.2, 25.0, 25.0),
            (25.0, 25.2, 25.0),
        ]
    )
    faces.append((fragment_base, fragment_base + 1, fragment_base + 2))
    faces.append(faces[0])
    return vertices, faces


def open_cube_mesh():
    vertices, faces = cube_mesh()
    return vertices, faces[:-1]


def write_ascii_stl(path: pathlib.Path, vertices, faces):
    lines = ["solid fixture"]
    for a, b, c in faces:
        lines.append("  facet normal 0 0 0")
        lines.append("    outer loop")
        for vertex_index in (a, b, c):
            x, y, z = vertices[vertex_index]
            lines.append(f"      vertex {x} {y} {z}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append("endsolid fixture")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_cli(input_mesh: pathlib.Path, out_dir: pathlib.Path):
    try:
        cli_input = input_mesh.relative_to(REPO_ROOT)
    except ValueError:
        cli_input = input_mesh

    completed = subprocess.run(
        [
            str(BIN_PATH),
            "analyze",
            str(cli_input),
            "--out",
            str(out_dir),
            "--preset",
            "mechanical",
            "--solid-threshold",
            "0.60",
        ],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    if completed.returncode != 0:
        raise AssertionError(
            f"CLI failed with exit code {completed.returncode}\nSTDOUT:\n{completed.stdout}\nSTDERR:\n{completed.stderr}"
        )
    report = json.loads((out_dir / "report.json").read_text(encoding="utf-8"))
    regions = json.loads((out_dir / "regions.json").read_text(encoding="utf-8"))
    constraints = json.loads((out_dir / "constraints.json").read_text(encoding="utf-8"))
    return completed, report, regions, constraints


class CliIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        completed = subprocess.run(
            ["make"],
            cwd=REPO_ROOT,
            text=True,
            capture_output=True,
            check=False,
        )
        if completed.returncode != 0:
            raise AssertionError(
                f"Build failed with exit code {completed.returncode}\nSTDOUT:\n{completed.stdout}\nSTDERR:\n{completed.stderr}"
            )

    def assert_file_matches_golden(self, actual_path: pathlib.Path, golden_path: pathlib.Path):
        actual = actual_path.read_text(encoding="utf-8")
        expected = golden_path.read_text(encoding="utf-8")
        if actual != expected:
            diff = "".join(
                difflib.unified_diff(
                    expected.splitlines(keepends=True),
                    actual.splitlines(keepends=True),
                    fromfile=str(golden_path),
                    tofile=str(actual_path),
                )
            )
            self.fail(f"{actual_path.name} did not match golden output\n{diff}")

    def test_checked_in_cube_matches_golden_outputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            out_dir = tmp_path / "out"
            mesh_path = FIXTURES_DIR / "cube.stl"

            _, report, regions, constraints = run_cli(mesh_path, out_dir)

            self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
            self.assertGreaterEqual(report["regions"]["count"], 6)
            self.assertGreaterEqual(len(regions["regions"]), 6)
            self.assertIn("constraints", constraints)
            step_text = (out_dir / "reconstruction.step").read_text(encoding="utf-8")
            self.assertIn("ADVANCED_FACE", step_text)
            self.assertIn("MANIFOLD_SOLID_BREP", step_text)
            self.assertNotIn("FACETED_BREP", step_text)

            self.assert_file_matches_golden(out_dir / "cleaned_mesh.stl", GOLDEN_DIR / "cleaned_mesh.stl")
            self.assert_file_matches_golden(out_dir / "report.json", GOLDEN_DIR / "report.json")
            self.assert_file_matches_golden(out_dir / "regions.json", GOLDEN_DIR / "regions.json")
            self.assert_file_matches_golden(out_dir / "constraints.json", GOLDEN_DIR / "constraints.json")
            self.assert_file_matches_golden(out_dir / "reconstruction.step", GOLDEN_DIR / "reconstruction.step")

    def test_repair_path_removes_duplicate_faces_and_tiny_fragment(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            mesh_path = tmp_path / "repair_noise.stl"
            out_dir = tmp_path / "out"
            vertices, faces = cube_with_repair_noise()
            write_ascii_stl(mesh_path, vertices, faces)

            _, report, _, _ = run_cli(mesh_path, out_dir)

            self.assertGreaterEqual(report["repair"]["duplicate_triangles_removed"], 1)
            self.assertGreaterEqual(report["repair"]["tiny_components_removed"], 1)
            self.assertEqual(report["reconstruction"]["outcome"], "solid_created")

    def test_open_mesh_stays_non_solid_and_reports_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            mesh_path = tmp_path / "open_cube.stl"
            out_dir = tmp_path / "out"
            vertices, faces = open_cube_mesh()
            write_ascii_stl(mesh_path, vertices, faces)

            _, report, _, _ = run_cli(mesh_path, out_dir)

            self.assertNotEqual(report["reconstruction"]["outcome"], "solid_created")
            self.assertFalse((out_dir / "reconstruction.step").exists())
            combined_reasons = " ".join(report["reconstruction"]["failure_reasons"]).lower()
            self.assertIn("open", combined_reasons)


if __name__ == "__main__":
    unittest.main()
