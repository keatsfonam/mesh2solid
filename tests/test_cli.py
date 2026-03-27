import json
import os
import pathlib
import subprocess
import tempfile
import unittest
import difflib
import zipfile

from benchmarks.corpus import BENCHMARK_CASES as CORPUS_BENCHMARK_CASES
from benchmarks.corpus import minimum_outcome_for_profile


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_BIN_PATH = REPO_ROOT / "build" / "mesh2solid"


def configured_bin_path() -> pathlib.Path:
    configured = os.environ.get("MESH2SOLID_BIN")
    if not configured:
        return DEFAULT_BIN_PATH
    path = pathlib.Path(configured)
    return path if path.is_absolute() else REPO_ROOT / path


BIN_PATH = configured_bin_path()
SKIP_BUILD = os.environ.get("MESH2SOLID_SKIP_BUILD") == "1"
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

CORPUS_CASES_BY_PATH = {
    case.relative_path: case for case in CORPUS_BENCHMARK_CASES
}
TEST_BENCHMARK_PATHS = [
    "examples/benchmark/3mf_samples/core_box.3mf",
    "examples/benchmark/3mf_samples/core_cylinder.3mf",
    "examples/benchmark/3mf_samples/core_multiple_cylinders.3mf",
    "examples/benchmark/cloudgripper/xy_rail_mount.stl",
    "examples/benchmark/cloudgripper/arm_holder.stl",
    "examples/benchmark/cloudgripper/xy_nema_bracket.stl",
    "examples/benchmark/cloudgripper/arm_linear_pinion_gear.stl",
    "examples/benchmark/bcn3d_moveo/t4m1e.stl",
]
BENCHMARK_CASES = {
    pathlib.Path(relative_path).stem: {
        "path": REPO_ROOT / relative_path,
        "expected_outcome": minimum_outcome_for_profile(
            CORPUS_CASES_BY_PATH[relative_path], "host-minimal"
        ),
        "min_regions": CORPUS_CASES_BY_PATH[relative_path].min_regions,
    }
    for relative_path in TEST_BENCHMARK_PATHS
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


def voxel_mesh(cells, cell_size=10.0):
    occupied = set(cells)
    vertices = []
    vertex_ids = {}
    faces = []

    def vertex_id(point):
        if point not in vertex_ids:
            vertex_ids[point] = len(vertices)
            vertices.append(tuple(coordinate * cell_size for coordinate in point))
        return vertex_ids[point]

    face_definitions = [
        ((1, 0, 0), [(1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1)]),
        ((-1, 0, 0), [(0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0)]),
        ((0, 1, 0), [(0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0)]),
        ((0, -1, 0), [(0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1)]),
        ((0, 0, 1), [(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]),
        ((0, 0, -1), [(0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0)]),
    ]

    for cell_x, cell_y, cell_z in occupied:
        for neighbor_offset, face_corners in face_definitions:
            offset_x, offset_y, offset_z = neighbor_offset
            if (cell_x + offset_x, cell_y + offset_y, cell_z + offset_z) in occupied:
                continue
            quad = [
                vertex_id((cell_x + corner_x, cell_y + corner_y, cell_z + corner_z))
                for corner_x, corner_y, corner_z in face_corners
            ]
            faces.append((quad[0], quad[1], quad[2]))
            faces.append((quad[0], quad[2], quad[3]))

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


def write_multi_item_3mf(path: pathlib.Path, vertices, faces, *, build_transforms, unit="millimeter"):
    vertex_lines = [
        f'          <vertex x="{x}" y="{y}" z="{z}" />'
        for x, y, z in vertices
    ]
    triangle_lines = [
        f'          <triangle v1="{a}" v2="{b}" v3="{c}" />'
        for a, b, c in faces
    ]
    build_lines = [
        (
            f'    <item objectid="1" transform="{transform}" />'
            if transform is not None
            else '    <item objectid="1" />'
        )
        for transform in build_transforms
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
            "  </resources>",
            "  <build>",
            *build_lines,
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


PRISMATIC_VOXEL_CASES = {
    "l_bracket": {
        "cells": {
            (0, 0, 0),
            (1, 0, 0),
            (0, 1, 0),
            (0, 0, 1),
            (1, 0, 1),
            (0, 1, 1),
        },
        "expected_faces": 8,
    },
    "stepped_block": {
        "cells": {
            (0, 0, 0),
            (1, 0, 0),
            (0, 1, 0),
            (1, 1, 0),
            (0, 0, 1),
            (1, 0, 1),
        },
        "expected_faces": 8,
    },
    "u_channel": {
        "cells": {
            (0, 0, 0),
            (1, 0, 0),
            (2, 0, 0),
            (0, 1, 0),
            (2, 1, 0),
            (0, 0, 1),
            (1, 0, 1),
            (2, 0, 1),
            (0, 1, 1),
            (2, 1, 1),
        },
        "expected_faces": 10,
    },
    "double_box": {
        "cells": {
            (0, 0, 0),
            (1, 0, 0),
            (4, 0, 0),
            (5, 0, 0),
        },
        "expected_faces": 12,
    },
    "cross_block": {
        "cells": {
            (1, 0, 0),
            (0, 1, 0),
            (1, 1, 0),
            (2, 1, 0),
            (1, 2, 0),
            (1, 0, 1),
            (0, 1, 1),
            (1, 1, 1),
            (2, 1, 1),
            (1, 2, 1),
        },
        "expected_faces": 14,
    },
}


def run_cli(input_mesh: pathlib.Path, out_dir: pathlib.Path, solid_threshold: float = 0.60):
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
            f"{solid_threshold:.2f}",
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
        if SKIP_BUILD:
            if not BIN_PATH.exists():
                raise AssertionError(f"Configured mesh2solid binary does not exist: {BIN_PATH}")
            return

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

    def assert_report_matches_golden(self, actual_path: pathlib.Path, golden_path: pathlib.Path):
        actual = json.loads(actual_path.read_text(encoding="utf-8"))
        expected = json.loads(golden_path.read_text(encoding="utf-8"))

        self.assertTrue(actual["backend"])
        if "cgal" in actual["backend"]:
            self.assertEqual(actual["plane_regularization"]["method"], "cgal_regularize_planes")
        else:
            self.assertEqual(actual["plane_regularization"]["method"], "none")

        actual["backend"] = expected["backend"]
        actual["plane_regularization"]["method"] = expected["plane_regularization"]["method"]
        self.assertEqual(actual, expected)

    def assert_regions_semantically_match_golden(self, actual_path: pathlib.Path, golden_path: pathlib.Path):
        actual = json.loads(actual_path.read_text(encoding="utf-8"))
        expected = json.loads(golden_path.read_text(encoding="utf-8"))

        def summarize(region_doc):
            return sorted(
                (
                    region["triangle_count"],
                    region["vertex_count"],
                    round(region["area"], 8),
                    region["unresolved"],
                    round(region["fit"]["rms_error"], 8),
                    round(region["fit"]["max_error"], 8),
                    round(region["fit"]["confidence"], 8),
                    len(region["neighbors"]),
                )
                for region in region_doc["regions"]
            )

        self.assertEqual(summarize(actual), summarize(expected))

    def assert_constraints_semantically_match_golden(self, actual_path: pathlib.Path, golden_path: pathlib.Path):
        actual = json.loads(actual_path.read_text(encoding="utf-8"))
        expected = json.loads(golden_path.read_text(encoding="utf-8"))

        def summarize(graph_doc):
            return sorted(
                (
                    constraint["type"],
                    constraint["applied"],
                    round(constraint["score"], 8),
                    constraint["rationale"],
                )
                for constraint in graph_doc["constraints"]
            )

        self.assertEqual(summarize(actual), summarize(expected))

    def assert_step_semantically_matches_golden(self, actual_path: pathlib.Path, golden_path: pathlib.Path):
        actual = actual_path.read_text(encoding="utf-8")
        expected = golden_path.read_text(encoding="utf-8")

        tokens = [
            "ADVANCED_FACE",
            "FACE_BOUND",
            "FACE_OUTER_BOUND",
            "EDGE_CURVE",
            "EDGE_LOOP",
            "MANIFOLD_SOLID_BREP",
            "CLOSED_SHELL",
        ]
        for token in tokens:
            self.assertEqual(actual.count(token), expected.count(token), msg=token)

        self.assertIn("MANIFOLD_SOLID_BREP", actual)
        self.assertNotIn("FACETED_BREP", actual)

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
                    self.assert_report_matches_golden(out_dir / "report.json", golden_dir / "report.json")
                    if report["backend"] == "internal_fallback":
                        self.assert_file_matches_golden(out_dir / "regions.json", golden_dir / "regions.json")
                        self.assert_file_matches_golden(out_dir / "constraints.json", golden_dir / "constraints.json")
                    else:
                        self.assert_regions_semantically_match_golden(
                            out_dir / "regions.json", golden_dir / "regions.json"
                        )
                        self.assert_constraints_semantically_match_golden(
                            out_dir / "constraints.json", golden_dir / "constraints.json"
                        )
                    if report["backend"] == "internal_fallback":
                        self.assert_file_matches_golden(
                            out_dir / "reconstruction.step", golden_dir / "reconstruction.step"
                        )
                    else:
                        self.assert_step_semantically_matches_golden(
                            out_dir / "reconstruction.step", golden_dir / "reconstruction.step"
                        )

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

    def test_heartgears_example_closes_to_solid(self):
        heartgears_path = EXAMPLES_DIR / "benchmark" / "3mf_samples_hard" / "heartgears.3mf"
        self.assertTrue(heartgears_path.exists(), f"Missing benchmark fixture: {heartgears_path}")

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            out_dir = tmp_path / "out"

            _, report, _, _ = run_cli(heartgears_path, out_dir)

            self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
            self.assertEqual(report["reconstruction"]["method"], "faceted_mesh_fallback")
            self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
            self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)
            self.assertTrue((out_dir / "reconstruction.step").exists())

    def test_faceted_fallback_rescues_closed_mesh_when_analytic_confidence_is_gated(self):
        mesh_path = FIXTURES_DIR / "rectangular_tube.stl"

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            out_dir = tmp_path / "out"

            _, report, _, _ = run_cli(mesh_path, out_dir, solid_threshold=0.99)

            self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
            self.assertEqual(report["reconstruction"]["method"], "faceted_mesh_fallback")
            self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
            self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)
            self.assertTrue((out_dir / "reconstruction.step").exists())

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

    def test_simple_cylinder_exports_cylindrical_surface(self):
        mesh_path = EXAMPLES_DIR / "benchmark" / "3mf_samples" / "core_cylinder.3mf"
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            out_dir = tmp_path / "out"

            _, report, _, _ = run_cli(mesh_path, out_dir)

            step_text = (out_dir / "reconstruction.step").read_text(encoding="utf-8")
            self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
            self.assertIn("CYLINDRICAL_SURFACE", step_text)
            self.assertIn("CIRCLE(", step_text)

    def test_multiple_cylinders_export_cylindrical_surfaces(self):
        mesh_path = EXAMPLES_DIR / "benchmark" / "3mf_samples" / "core_multiple_cylinders.3mf"
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            out_dir = tmp_path / "out"

            _, report, _, _ = run_cli(mesh_path, out_dir)

            step_text = (out_dir / "reconstruction.step").read_text(encoding="utf-8")
            self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
            self.assertGreaterEqual(step_text.count("CYLINDRICAL_SURFACE"), 2)
            self.assertGreaterEqual(step_text.count("CIRCLE("), 4)
            self.assertLess(step_text.count("ADVANCED_FACE"), report["reconstruction"]["face_count"])

    def test_generated_prismatic_voxel_cases_stay_clean(self):
        for case_name, case in PRISMATIC_VOXEL_CASES.items():
            with self.subTest(case=case_name):
                with tempfile.TemporaryDirectory() as tmp:
                    tmp_path = pathlib.Path(tmp)
                    mesh_path = tmp_path / f"{case_name}.stl"
                    out_dir = tmp_path / "out"

                    vertices, faces = voxel_mesh(case["cells"])
                    write_ascii_stl(mesh_path, vertices, faces)
                    _, report, _, _ = run_cli(mesh_path, out_dir)

                    self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
                    self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
                    self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)

                    step_text = (out_dir / "reconstruction.step").read_text(encoding="utf-8")
                    self.assertNotIn("FACETED_BREP", step_text)
                    self.assertEqual(step_text.count("CYLINDRICAL_SURFACE"), 0)
                    self.assertEqual(
                        step_text.count("ADVANCED_FACE"), case["expected_faces"]
                    )
                    self.assertEqual(step_text.count("PLANE("), case["expected_faces"])

    def test_multi_item_3mf_prismatic_boxes_stay_clean(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            mesh_path = tmp_path / "prismatic_boxes.3mf"
            out_dir = tmp_path / "out"

            vertices, faces = cube_mesh(size=10.0)
            write_multi_item_3mf(
                mesh_path,
                vertices,
                faces,
                build_transforms=[
                    "1 0 0 0 1 0 0 0 1 0 0 0",
                    "1 0 0 0 1 0 0 0 1 40 0 0",
                ],
            )

            _, report, _, _ = run_cli(mesh_path, out_dir)

            self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
            self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
            self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)

            step_text = (out_dir / "reconstruction.step").read_text(encoding="utf-8")
            self.assertNotIn("FACETED_BREP", step_text)
            self.assertEqual(step_text.count("CYLINDRICAL_SURFACE"), 0)
            self.assertEqual(step_text.count("ADVANCED_FACE"), 12)
            self.assertEqual(step_text.count("PLANE("), 12)


if __name__ == "__main__":
    unittest.main()
