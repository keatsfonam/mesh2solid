import json
import pathlib
import subprocess
import tempfile
import unittest
import difflib
import zipfile


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
BIN_PATH = REPO_ROOT / "build" / "mesh2solid"
FIXTURES_DIR = REPO_ROOT / "tests" / "fixtures"
GOLDEN_ROOT = REPO_ROOT / "tests" / "golden"
EXAMPLES_DIR = REPO_ROOT / "examples"

GOLDEN_CASES = {
    "cube": {"fixture": "cube.stl", "min_regions": 6, "step_tokens": []},
    "rectangular_box": {"fixture": "rectangular_box.stl", "min_regions": 6, "step_tokens": []},
    "sloped_block": {"fixture": "sloped_block.stl", "min_regions": 6, "step_tokens": []},
    "triangular_prism": {"fixture": "triangular_prism.stl", "min_regions": 5, "step_tokens": []},
    "rectangular_tube": {
        "fixture": "rectangular_tube.stl",
        "min_regions": 10,
        "step_tokens": ["FACE_BOUND"],
    },
}

BENCHMARK_CASES = {
    "3mf_core_box": {
        "path": EXAMPLES_DIR / "benchmark" / "3mf_samples" / "core_box.3mf",
        "expected_outcome": "solid_created",
        "min_regions": 6,
    },
    "3mf_core_cylinder": {
        "path": EXAMPLES_DIR / "benchmark" / "3mf_samples" / "core_cylinder.3mf",
        "expected_outcome": "solid_created",
        "min_regions": 3,
    },
    "3mf_core_multiple_cylinders": {
        "path": EXAMPLES_DIR / "benchmark" / "3mf_samples" / "core_multiple_cylinders.3mf",
        "expected_outcome": "solid_created",
        "min_regions": 6,
    },
    "cloudgripper_xy_rail_mount": {
        "path": EXAMPLES_DIR / "benchmark" / "cloudgripper" / "xy_rail_mount.stl",
        "expected_outcome": "solid_created",
        "min_regions": 6,
    },
    "cloudgripper_arm_holder": {
        "path": EXAMPLES_DIR / "benchmark" / "cloudgripper" / "arm_holder.stl",
        "expected_outcome": "solid_created",
        "min_regions": 2,
    },
    "cloudgripper_xy_nema_bracket": {
        "path": EXAMPLES_DIR / "benchmark" / "cloudgripper" / "xy_nema_bracket.stl",
        "expected_outcome": "solid_created",
        "min_regions": 25,
    },
    "cloudgripper_arm_linear_pinion_gear": {
        "path": EXAMPLES_DIR / "benchmark" / "cloudgripper" / "arm_linear_pinion_gear.stl",
        "expected_outcome": "solid_created",
        "min_regions": 25,
    },
    "bcn3d_moveo_t4m1e": {
        "path": EXAMPLES_DIR / "benchmark" / "bcn3d_moveo" / "t4m1e.stl",
        "expected_outcome": "solid_created",
        "min_regions": 25,
    },
}


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


def beveled_cube_mesh(size=10.0, bevel=3.0):
    vertices = [
        (0.0, 0.0, 0.0),
        (size, 0.0, 0.0),
        (size, size, 0.0),
        (0.0, size, 0.0),
        (0.0, 0.0, size),
        (size, 0.0, size),
        (size, size - bevel, size),
        (size, size, size - bevel),
        (size - bevel, size, size),
        (0.0, size, size),
    ]
    faces = [
        (0, 2, 1),
        (0, 3, 2),
        (0, 9, 3),
        (0, 4, 9),
        (0, 5, 4),
        (0, 1, 5),
        (1, 5, 6),
        (1, 6, 7),
        (1, 7, 2),
        (3, 2, 7),
        (3, 7, 8),
        (3, 8, 9),
        (4, 9, 8),
        (4, 8, 6),
        (4, 6, 5),
        (8, 6, 7),
    ]
    return vertices, faces


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


def write_basic_3mf(path: pathlib.Path, vertices, faces, *, unit="millimeter", build_transform=None):
    vertex_lines = [
        f'          <vertex x="{x}" y="{y}" z="{z}" />'
        for x, y, z in vertices
    ]
    triangle_lines = [
        f'          <triangle v1="{a}" v2="{b}" v3="{c}" />'
        for a, b, c in faces
    ]
    model_xml = "\n".join(
        [
            '<?xml version="1.0" encoding="UTF-8"?>',
            f'<model unit="{unit}" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">',
            "  <resources>",
            '    <object id="1" type="model">',
            "      <mesh>",
            "        <vertices>",
            *vertex_lines,
            "        </vertices>",
            "        <triangles>",
            *triangle_lines,
            "        </triangles>",
            "      </mesh>",
            "    </object>",
            '    <object id="2" type="model">',
            "      <components>",
            '        <component objectid="1" transform="1 0 0 0 1 0 0 0 1 0 0 0" />',
            "      </components>",
            "    </object>",
            "  </resources>",
            "  <build>",
            (
                f'    <item objectid="2" transform="{build_transform}" />'
                if build_transform is not None
                else '    <item objectid="2" />'
            ),
            "  </build>",
            "</model>",
            "",
        ]
    )
    content_types_xml = "\n".join(
        [
            '<?xml version="1.0" encoding="UTF-8"?>',
            '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">',
            '  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml" />',
            '  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml" />',
            "</Types>",
            "",
        ]
    )
    relationships_xml = "\n".join(
        [
            '<?xml version="1.0" encoding="UTF-8"?>',
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">',
            '  <Relationship Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" Target="/3D/3dmodel.model" />',
            "</Relationships>",
            "",
        ]
    )

    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("[Content_Types].xml", content_types_xml)
        archive.writestr("_rels/.rels", relationships_xml)
        archive.writestr("3D/3dmodel.model", model_xml)


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


def translated_mesh(vertices, dx=0.0, dy=0.0, dz=0.0):
    return [(x + dx, y + dy, z + dz) for x, y, z in vertices]


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

    def test_checked_in_fixtures_match_golden_outputs(self):
        for case_name, case in GOLDEN_CASES.items():
            with self.subTest(case=case_name):
                with tempfile.TemporaryDirectory() as tmp:
                    tmp_path = pathlib.Path(tmp)
                    out_dir = tmp_path / "out"
                    mesh_path = FIXTURES_DIR / case["fixture"]
                    golden_dir = GOLDEN_ROOT / case_name

                    _, report, regions, constraints = run_cli(mesh_path, out_dir)

                    self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
                    self.assertGreaterEqual(report["regions"]["count"], case["min_regions"])
                    self.assertGreaterEqual(len(regions["regions"]), case["min_regions"])
                    self.assertIn("constraints", constraints)
                    step_text = (out_dir / "reconstruction.step").read_text(encoding="utf-8")
                    self.assertIn("ADVANCED_FACE", step_text)
                    self.assertIn("MANIFOLD_SOLID_BREP", step_text)
                    self.assertNotIn("FACETED_BREP", step_text)
                    for token in case["step_tokens"]:
                        self.assertIn(token, step_text)

                    self.assert_file_matches_golden(out_dir / "cleaned_mesh.stl", golden_dir / "cleaned_mesh.stl")
                    self.assert_file_matches_golden(out_dir / "report.json", golden_dir / "report.json")
                    self.assert_file_matches_golden(out_dir / "regions.json", golden_dir / "regions.json")
                    self.assert_file_matches_golden(out_dir / "constraints.json", golden_dir / "constraints.json")
                    self.assert_file_matches_golden(out_dir / "reconstruction.step", golden_dir / "reconstruction.step")

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

    def test_tiny_beveled_corner_stays_solid(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            mesh_path = tmp_path / "beveled_cube.stl"
            out_dir = tmp_path / "out"
            vertices, faces = beveled_cube_mesh()
            write_ascii_stl(mesh_path, vertices, faces)

            _, report, _, _ = run_cli(mesh_path, out_dir)

            self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
            self.assertTrue((out_dir / "reconstruction.step").exists())
            self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
            self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)
            self.assertEqual(report["reconstruction"]["omitted_region_ids"], [])

    def test_generated_3mf_cube_matches_cube_baseline(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            mesh_path = tmp_path / "cube.3mf"
            out_dir = tmp_path / "out"
            vertices, faces = cube_mesh()
            write_basic_3mf(mesh_path, vertices, faces)

            _, report, regions, constraints = run_cli(mesh_path, out_dir)

            self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
            self.assertEqual(report["regions"]["count"], 6)
            self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
            self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)
            self.assertEqual(report["reconstruction"]["omitted_region_ids"], [])
            self.assertEqual(len(regions["regions"]), 6)
            self.assertIn("constraints", constraints)

            cube_golden = GOLDEN_ROOT / "cube"
            self.assert_file_matches_golden(out_dir / "cleaned_mesh.stl", cube_golden / "cleaned_mesh.stl")
            self.assert_file_matches_golden(out_dir / "regions.json", cube_golden / "regions.json")
            self.assert_file_matches_golden(out_dir / "constraints.json", cube_golden / "constraints.json")
            self.assert_file_matches_golden(out_dir / "reconstruction.step", cube_golden / "reconstruction.step")

    def test_generated_3mf_honors_units_and_build_transform(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            mesh_path = tmp_path / "cube_inches.3mf"
            stl_path = tmp_path / "expected_cube.stl"
            out_dir = tmp_path / "out"
            expected_out_dir = tmp_path / "expected_out"

            inch_cube_vertices, faces = cube_mesh(size=1.0)
            write_basic_3mf(
                mesh_path,
                inch_cube_vertices,
                faces,
                unit="inch",
                build_transform="1 0 0 0 1 0 0 0 1 12.7 0 0",
            )

            expected_vertices = translated_mesh(cube_mesh(size=25.4)[0], dx=12.7)
            write_ascii_stl(stl_path, expected_vertices, faces)

            _, report, _, _ = run_cli(mesh_path, out_dir)
            _, expected_report, _, _ = run_cli(stl_path, expected_out_dir)

            self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
            self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
            self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)
            self.assertEqual(report["regions"]["count"], expected_report["regions"]["count"])
            self.assert_file_matches_golden(out_dir / "cleaned_mesh.stl", expected_out_dir / "cleaned_mesh.stl")
            self.assert_file_matches_golden(out_dir / "regions.json", expected_out_dir / "regions.json")
            self.assert_file_matches_golden(out_dir / "constraints.json", expected_out_dir / "constraints.json")
            self.assert_file_matches_golden(out_dir / "reconstruction.step", expected_out_dir / "reconstruction.step")

    def test_complex_bridge_example_closes_to_solid(self):
        bridge_path = EXAMPLES_DIR / "complex" / "bridge.stl"
        self.assertTrue(bridge_path.exists(), f"Missing example fixture: {bridge_path}")

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            out_dir = tmp_path / "out"

            _, report, _, _ = run_cli(bridge_path, out_dir)

            self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
            self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
            self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)
            self.assertTrue((out_dir / "reconstruction.step").exists())
            self.assertEqual(report["reconstruction"]["shell_gap_score"], 0.0)

    def test_public_benchmark_corpus_matches_expected_outcomes(self):
        for case_name, case in BENCHMARK_CASES.items():
            with self.subTest(case=case_name):
                mesh_path = case["path"]
                self.assertTrue(mesh_path.exists(), f"Missing benchmark fixture: {mesh_path}")

                with tempfile.TemporaryDirectory() as tmp:
                    tmp_path = pathlib.Path(tmp)
                    out_dir = tmp_path / "out"

                    _, report, regions, constraints = run_cli(mesh_path, out_dir)

                    self.assertEqual(report["reconstruction"]["outcome"], case["expected_outcome"])
                    self.assertGreaterEqual(report["regions"]["count"], case["min_regions"])
                    self.assertGreaterEqual(len(regions["regions"]), case["min_regions"])
                    self.assertIn("constraints", constraints)

                    if case["expected_outcome"] == "solid_created":
                        self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
                        self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)
                        self.assertTrue((out_dir / "reconstruction.step").exists())
                    else:
                        self.assertFalse((out_dir / "reconstruction.step").exists())
                        self.assertTrue(report["reconstruction"]["failure_reasons"])
                        self.assertTrue(
                            report["reconstruction"]["open_edge_count"] > 0
                            or report["reconstruction"]["non_manifold_edge_count"] > 0
                        )
                        if "max_open_edges" in case:
                            self.assertLessEqual(
                                report["reconstruction"]["open_edge_count"],
                                case["max_open_edges"],
                            )
                        if "max_non_manifold_edges" in case:
                            self.assertLessEqual(
                                report["reconstruction"]["non_manifold_edge_count"],
                                case["max_non_manifold_edges"],
                            )


if __name__ == "__main__":
    unittest.main()
