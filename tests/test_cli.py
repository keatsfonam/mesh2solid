import json
import math
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


def box_with_round_bore_mesh(
    half_size=20.0,
    radius=8.0,
    height=20.0,
    segments=16,
    center=(0.0, 0.0),
):
    vertices = []
    vertex_ids = {}
    faces = []
    center_x, center_y = center

    def vertex_id(point):
        key = tuple(round(value, 8) for value in point)
        if key not in vertex_ids:
            vertex_ids[key] = len(vertices)
            vertices.append(tuple(float(value) for value in point))
        return vertex_ids[key]

    def square_support(angle):
        cosine = math.cos(angle)
        sine = math.sin(angle)
        scale = max(abs(cosine), abs(sine))
        return half_size * cosine / scale, half_size * sine / scale

    outer_bottom = []
    outer_top = []
    inner_bottom = []
    inner_top = []
    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        outer_x, outer_y = square_support(angle)
        inner_x = center_x + radius * math.cos(angle)
        inner_y = center_y + radius * math.sin(angle)
        outer_bottom.append(vertex_id((outer_x, outer_y, 0.0)))
        outer_top.append(vertex_id((outer_x, outer_y, height)))
        inner_bottom.append(vertex_id((inner_x, inner_y, 0.0)))
        inner_top.append(vertex_id((inner_x, inner_y, height)))

    for index in range(segments):
        next_index = (index + 1) % segments

        faces.append((outer_top[index], outer_top[next_index], inner_top[next_index]))
        faces.append((outer_top[index], inner_top[next_index], inner_top[index]))

        faces.append((outer_bottom[index], inner_bottom[next_index], outer_bottom[next_index]))
        faces.append((outer_bottom[index], inner_bottom[index], inner_bottom[next_index]))

        faces.append((outer_bottom[index], outer_bottom[next_index], outer_top[next_index]))
        faces.append((outer_bottom[index], outer_top[next_index], outer_top[index]))

        faces.append((inner_bottom[index], inner_top[next_index], inner_bottom[next_index]))
        faces.append((inner_bottom[index], inner_top[index], inner_top[next_index]))

    return vertices, faces


def blind_bore_mesh(
    half_size=20.0,
    radius=8.0,
    height=20.0,
    pocket_depth=12.0,
    segments=24,
    center=(0.0, 0.0),
):
    vertices = []
    vertex_ids = {}
    faces = []
    center_x, center_y = center

    def vertex_id(point):
        key = tuple(round(value, 8) for value in point)
        if key not in vertex_ids:
            vertex_ids[key] = len(vertices)
            vertices.append(tuple(float(value) for value in point))
        return vertex_ids[key]

    def square_support(angle):
        cosine = math.cos(angle)
        sine = math.sin(angle)
        scale = max(abs(cosine), abs(sine))
        return half_size * cosine / scale, half_size * sine / scale

    floor_z = height - pocket_depth
    center_bottom = vertex_id((0.0, 0.0, 0.0))
    center_floor = vertex_id((center_x, center_y, floor_z))

    outer_bottom = []
    outer_top = []
    inner_top = []
    inner_floor = []
    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        outer_x, outer_y = square_support(angle)
        inner_x = center_x + radius * math.cos(angle)
        inner_y = center_y + radius * math.sin(angle)
        outer_bottom.append(vertex_id((outer_x, outer_y, 0.0)))
        outer_top.append(vertex_id((outer_x, outer_y, height)))
        inner_top.append(vertex_id((inner_x, inner_y, height)))
        inner_floor.append(vertex_id((inner_x, inner_y, floor_z)))

    for index in range(segments):
        next_index = (index + 1) % segments

        faces.append((outer_top[index], outer_top[next_index], inner_top[next_index]))
        faces.append((outer_top[index], inner_top[next_index], inner_top[index]))

        faces.append((center_bottom, outer_bottom[index], outer_bottom[next_index]))

        faces.append((outer_bottom[index], outer_bottom[next_index], outer_top[next_index]))
        faces.append((outer_bottom[index], outer_top[next_index], outer_top[index]))

        faces.append((inner_floor[index], inner_top[next_index], inner_floor[next_index]))
        faces.append((inner_floor[index], inner_top[index], inner_top[next_index]))

        faces.append((center_floor, inner_floor[next_index], inner_floor[index]))

    return vertices, faces


def counterbore_mesh(
    half_size=20.0,
    through_radius=4.0,
    counterbore_radius=8.0,
    height=20.0,
    counterbore_depth=6.0,
    segments=24,
    center=(0.0, 0.0),
):
    vertices = []
    vertex_ids = {}
    faces = []
    center_x, center_y = center

    def vertex_id(point):
        key = tuple(round(value, 8) for value in point)
        if key not in vertex_ids:
            vertex_ids[key] = len(vertices)
            vertices.append(tuple(float(value) for value in point))
        return vertex_ids[key]

    def square_support(angle):
        cosine = math.cos(angle)
        sine = math.sin(angle)
        scale = max(abs(cosine), abs(sine))
        return half_size * cosine / scale, half_size * sine / scale

    z_step = height - counterbore_depth
    outer_bottom = []
    outer_top = []
    counterbore_top = []
    counterbore_step = []
    through_step = []
    through_bottom = []
    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        outer_x, outer_y = square_support(angle)
        counterbore_x = center_x + counterbore_radius * math.cos(angle)
        counterbore_y = center_y + counterbore_radius * math.sin(angle)
        through_x = center_x + through_radius * math.cos(angle)
        through_y = center_y + through_radius * math.sin(angle)
        outer_bottom.append(vertex_id((outer_x, outer_y, 0.0)))
        outer_top.append(vertex_id((outer_x, outer_y, height)))
        counterbore_top.append(vertex_id((counterbore_x, counterbore_y, height)))
        counterbore_step.append(vertex_id((counterbore_x, counterbore_y, z_step)))
        through_step.append(vertex_id((through_x, through_y, z_step)))
        through_bottom.append(vertex_id((through_x, through_y, 0.0)))

    for index in range(segments):
        next_index = (index + 1) % segments

        faces.append((outer_top[index], outer_top[next_index], counterbore_top[next_index]))
        faces.append((outer_top[index], counterbore_top[next_index], counterbore_top[index]))

        faces.append((outer_bottom[index], outer_bottom[next_index], outer_top[next_index]))
        faces.append((outer_bottom[index], outer_top[next_index], outer_top[index]))

        faces.append(
            (counterbore_step[index], counterbore_top[next_index], counterbore_step[next_index])
        )
        faces.append(
            (counterbore_step[index], counterbore_top[index], counterbore_top[next_index])
        )

        faces.append((counterbore_step[index], counterbore_step[next_index], through_step[next_index]))
        faces.append((counterbore_step[index], through_step[next_index], through_step[index]))

        faces.append((through_bottom[index], through_step[next_index], through_bottom[next_index]))
        faces.append((through_bottom[index], through_step[index], through_step[next_index]))

        faces.append((outer_bottom[index], through_bottom[next_index], outer_bottom[next_index]))
        faces.append((outer_bottom[index], through_bottom[index], through_bottom[next_index]))

    return vertices, faces


def blind_counterbore_mesh(
    half_size=20.0,
    through_radius=4.0,
    counterbore_radius=8.0,
    height=20.0,
    counterbore_depth=6.0,
    pilot_depth=12.0,
    segments=24,
    center=(0.0, 0.0),
):
    vertices = []
    vertex_ids = {}
    faces = []
    center_x, center_y = center

    def vertex_id(point):
        key = tuple(round(value, 8) for value in point)
        if key not in vertex_ids:
            vertex_ids[key] = len(vertices)
            vertices.append(tuple(float(value) for value in point))
        return vertex_ids[key]

    def square_support(angle):
        cosine = math.cos(angle)
        sine = math.sin(angle)
        scale = max(abs(cosine), abs(sine))
        return half_size * cosine / scale, half_size * sine / scale

    z_step = height - counterbore_depth
    z_floor = height - pilot_depth
    center_bottom = vertex_id((0.0, 0.0, 0.0))
    center_floor = vertex_id((center_x, center_y, z_floor))

    outer_bottom = []
    outer_top = []
    counterbore_top = []
    counterbore_step = []
    pilot_step = []
    pilot_floor = []
    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        outer_x, outer_y = square_support(angle)
        counterbore_x = center_x + counterbore_radius * math.cos(angle)
        counterbore_y = center_y + counterbore_radius * math.sin(angle)
        pilot_x = center_x + through_radius * math.cos(angle)
        pilot_y = center_y + through_radius * math.sin(angle)
        outer_bottom.append(vertex_id((outer_x, outer_y, 0.0)))
        outer_top.append(vertex_id((outer_x, outer_y, height)))
        counterbore_top.append(vertex_id((counterbore_x, counterbore_y, height)))
        counterbore_step.append(vertex_id((counterbore_x, counterbore_y, z_step)))
        pilot_step.append(vertex_id((pilot_x, pilot_y, z_step)))
        pilot_floor.append(vertex_id((pilot_x, pilot_y, z_floor)))

    for index in range(segments):
        next_index = (index + 1) % segments

        faces.append((outer_top[index], outer_top[next_index], counterbore_top[next_index]))
        faces.append((outer_top[index], counterbore_top[next_index], counterbore_top[index]))

        faces.append((outer_bottom[index], outer_bottom[next_index], outer_top[next_index]))
        faces.append((outer_bottom[index], outer_top[next_index], outer_top[index]))

        faces.append(
            (counterbore_step[index], counterbore_top[next_index], counterbore_step[next_index])
        )
        faces.append(
            (counterbore_step[index], counterbore_top[index], counterbore_top[next_index])
        )

        faces.append((counterbore_step[index], counterbore_step[next_index], pilot_step[next_index]))
        faces.append((counterbore_step[index], pilot_step[next_index], pilot_step[index]))

        faces.append((pilot_floor[index], pilot_step[next_index], pilot_floor[next_index]))
        faces.append((pilot_floor[index], pilot_step[index], pilot_step[next_index]))

        faces.append((center_bottom, outer_bottom[index], outer_bottom[next_index]))
        faces.append((center_floor, pilot_floor[next_index], pilot_floor[index]))

    return vertices, faces


def boss_mesh(
    half_size=20.0,
    radius=8.0,
    height=16.0,
    boss_height=10.0,
    segments=24,
    center=(0.0, 0.0),
):
    vertices = []
    vertex_ids = {}
    faces = []
    center_x, center_y = center

    def vertex_id(point):
        key = tuple(round(value, 8) for value in point)
        if key not in vertex_ids:
            vertex_ids[key] = len(vertices)
            vertices.append(tuple(float(value) for value in point))
        return vertex_ids[key]

    def square_support(angle):
        cosine = math.cos(angle)
        sine = math.sin(angle)
        scale = max(abs(cosine), abs(sine))
        return half_size * cosine / scale, half_size * sine / scale

    center_bottom = vertex_id((0.0, 0.0, 0.0))
    center_top = vertex_id((center_x, center_y, height + boss_height))

    outer_bottom = []
    outer_top = []
    boss_base = []
    boss_top = []
    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        outer_x, outer_y = square_support(angle)
        boss_x = center_x + radius * math.cos(angle)
        boss_y = center_y + radius * math.sin(angle)
        outer_bottom.append(vertex_id((outer_x, outer_y, 0.0)))
        outer_top.append(vertex_id((outer_x, outer_y, height)))
        boss_base.append(vertex_id((boss_x, boss_y, height)))
        boss_top.append(vertex_id((boss_x, boss_y, height + boss_height)))

    for index in range(segments):
        next_index = (index + 1) % segments

        faces.append((outer_top[index], outer_top[next_index], boss_base[next_index]))
        faces.append((outer_top[index], boss_base[next_index], boss_base[index]))

        faces.append((center_bottom, outer_bottom[index], outer_bottom[next_index]))

        faces.append((outer_bottom[index], outer_bottom[next_index], outer_top[next_index]))
        faces.append((outer_bottom[index], outer_top[next_index], outer_top[index]))

        faces.append((boss_base[index], boss_top[next_index], boss_base[next_index]))
        faces.append((boss_base[index], boss_top[index], boss_top[next_index]))

        faces.append((center_top, boss_top[index], boss_top[next_index]))

    return vertices, faces


def standoff_mesh(
    half_size=20.0,
    inner_radius=4.0,
    outer_radius=8.0,
    base_height=16.0,
    boss_height=10.0,
    segments=24,
    center=(0.0, 0.0),
):
    vertices = []
    vertex_ids = {}
    faces = []
    center_x, center_y = center

    def vertex_id(point):
        key = tuple(round(value, 8) for value in point)
        if key not in vertex_ids:
            vertex_ids[key] = len(vertices)
            vertices.append(tuple(float(value) for value in point))
        return vertex_ids[key]

    def square_support(angle):
        cosine = math.cos(angle)
        sine = math.sin(angle)
        scale = max(abs(cosine), abs(sine))
        return half_size * cosine / scale, half_size * sine / scale

    outer_bottom = []
    outer_top = []
    hole_bottom = []
    hole_top = []
    boss_base = []
    boss_top = []
    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        outer_x, outer_y = square_support(angle)
        hole_x = center_x + inner_radius * math.cos(angle)
        hole_y = center_y + inner_radius * math.sin(angle)
        boss_x = center_x + outer_radius * math.cos(angle)
        boss_y = center_y + outer_radius * math.sin(angle)
        outer_bottom.append(vertex_id((outer_x, outer_y, 0.0)))
        outer_top.append(vertex_id((outer_x, outer_y, base_height)))
        hole_bottom.append(vertex_id((hole_x, hole_y, 0.0)))
        hole_top.append(vertex_id((hole_x, hole_y, base_height + boss_height)))
        boss_base.append(vertex_id((boss_x, boss_y, base_height)))
        boss_top.append(vertex_id((boss_x, boss_y, base_height + boss_height)))

    for index in range(segments):
        next_index = (index + 1) % segments

        faces.append((outer_bottom[index], hole_bottom[next_index], outer_bottom[next_index]))
        faces.append((outer_bottom[index], hole_bottom[index], hole_bottom[next_index]))

        faces.append((outer_bottom[index], outer_bottom[next_index], outer_top[next_index]))
        faces.append((outer_bottom[index], outer_top[next_index], outer_top[index]))

        faces.append((outer_top[index], outer_top[next_index], boss_base[next_index]))
        faces.append((outer_top[index], boss_base[next_index], boss_base[index]))

        faces.append((boss_base[index], boss_top[next_index], boss_base[next_index]))
        faces.append((boss_base[index], boss_top[index], boss_top[next_index]))

        faces.append((boss_top[index], boss_top[next_index], hole_top[next_index]))
        faces.append((boss_top[index], hole_top[next_index], hole_top[index]))

        faces.append((hole_bottom[index], hole_top[next_index], hole_bottom[next_index]))
        faces.append((hole_bottom[index], hole_top[index], hole_top[next_index]))

    return vertices, faces


def transform_mesh(vertices, *, rotation=None, translation=(0.0, 0.0, 0.0)):
    if rotation is None:
        rotation = (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )

    transformed = []
    translate_x, translate_y, translate_z = translation
    for x, y, z in vertices:
        transformed_x = (
            rotation[0][0] * x + rotation[0][1] * y + rotation[0][2] * z + translate_x
        )
        transformed_y = (
            rotation[1][0] * x + rotation[1][1] * y + rotation[1][2] * z + translate_y
        )
        transformed_z = (
            rotation[2][0] * x + rotation[2][1] * y + rotation[2][2] * z + translate_z
        )
        transformed.append((transformed_x, transformed_y, transformed_z))
    return transformed


def rotation_matrix_xyz(angle_x_degrees=0.0, angle_y_degrees=0.0, angle_z_degrees=0.0):
    angle_x = math.radians(angle_x_degrees)
    angle_y = math.radians(angle_y_degrees)
    angle_z = math.radians(angle_z_degrees)

    cos_x = math.cos(angle_x)
    sin_x = math.sin(angle_x)
    cos_y = math.cos(angle_y)
    sin_y = math.sin(angle_y)
    cos_z = math.cos(angle_z)
    sin_z = math.sin(angle_z)

    rotate_x = (
        (1.0, 0.0, 0.0),
        (0.0, cos_x, -sin_x),
        (0.0, sin_x, cos_x),
    )
    rotate_y = (
        (cos_y, 0.0, sin_y),
        (0.0, 1.0, 0.0),
        (-sin_y, 0.0, cos_y),
    )
    rotate_z = (
        (cos_z, -sin_z, 0.0),
        (sin_z, cos_z, 0.0),
        (0.0, 0.0, 1.0),
    )

    def multiply(left, right):
        return tuple(
            tuple(
                sum(left[row][index] * right[index][column] for index in range(3))
                for column in range(3)
            )
            for row in range(3)
        )

    return multiply(rotate_z, multiply(rotate_y, rotate_x))


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

    def test_prismatic_block_with_round_bore_exports_clean_cylinder(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            mesh_path = tmp_path / "box_with_bore.stl"
            out_dir = tmp_path / "out"

            vertices, faces = box_with_round_bore_mesh()
            write_ascii_stl(mesh_path, vertices, faces)
            _, report, _, _ = run_cli(mesh_path, out_dir)

            self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
            self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
            self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)

            step_text = (out_dir / "reconstruction.step").read_text(encoding="utf-8")
            self.assertNotIn("FACETED_BREP", step_text)
            self.assertEqual(step_text.count("CYLINDRICAL_SURFACE"), 1)
            self.assertGreaterEqual(step_text.count("CIRCLE("), 4)
            self.assertEqual(step_text.count("ADVANCED_FACE"), 7)
            self.assertEqual(step_text.count("PLANE("), 6)
            self.assertGreaterEqual(step_text.count("FACE_BOUND"), 3)

    def test_prismatic_block_with_blind_bore_exports_clean_cylinder(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            mesh_path = tmp_path / "blind_bore.stl"
            out_dir = tmp_path / "out"

            vertices, faces = blind_bore_mesh()
            write_ascii_stl(mesh_path, vertices, faces)
            _, report, _, _ = run_cli(mesh_path, out_dir)

            self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
            self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
            self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)

            step_text = (out_dir / "reconstruction.step").read_text(encoding="utf-8")
            self.assertNotIn("FACETED_BREP", step_text)
            self.assertEqual(step_text.count("CYLINDRICAL_SURFACE"), 1)
            self.assertGreaterEqual(step_text.count("CIRCLE("), 4)
            self.assertEqual(step_text.count("ADVANCED_FACE"), 8)
            self.assertEqual(step_text.count("PLANE("), 7)
            self.assertGreaterEqual(step_text.count("FACE_BOUND"), 2)

    def test_prismatic_block_with_counterbore_exports_clean_cylinders(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            mesh_path = tmp_path / "counterbore.stl"
            out_dir = tmp_path / "out"

            vertices, faces = counterbore_mesh()
            write_ascii_stl(mesh_path, vertices, faces)
            _, report, _, _ = run_cli(mesh_path, out_dir)

            self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
            self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
            self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)

            step_text = (out_dir / "reconstruction.step").read_text(encoding="utf-8")
            self.assertNotIn("FACETED_BREP", step_text)
            self.assertEqual(step_text.count("CYLINDRICAL_SURFACE"), 2)
            self.assertGreaterEqual(step_text.count("CIRCLE("), 8)
            self.assertEqual(step_text.count("ADVANCED_FACE"), 9)
            self.assertEqual(step_text.count("PLANE("), 7)
            self.assertGreaterEqual(step_text.count("FACE_BOUND"), 4)

    def test_prismatic_block_with_blind_counterbore_exports_clean_cylinders(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            mesh_path = tmp_path / "blind_counterbore.stl"
            out_dir = tmp_path / "out"

            vertices, faces = blind_counterbore_mesh()
            write_ascii_stl(mesh_path, vertices, faces)
            _, report, _, _ = run_cli(mesh_path, out_dir)

            self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
            self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
            self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)

            step_text = (out_dir / "reconstruction.step").read_text(encoding="utf-8")
            self.assertNotIn("FACETED_BREP", step_text)
            self.assertEqual(step_text.count("CYLINDRICAL_SURFACE"), 2)
            self.assertGreaterEqual(step_text.count("CIRCLE("), 8)
            self.assertEqual(step_text.count("ADVANCED_FACE"), 10)
            self.assertEqual(step_text.count("PLANE("), 8)
            self.assertGreaterEqual(step_text.count("FACE_BOUND"), 3)

    def test_prismatic_block_with_boss_exports_clean_cylinder(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            mesh_path = tmp_path / "boss.stl"
            out_dir = tmp_path / "out"

            vertices, faces = boss_mesh()
            write_ascii_stl(mesh_path, vertices, faces)
            _, report, _, _ = run_cli(mesh_path, out_dir)

            self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
            self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
            self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)

            step_text = (out_dir / "reconstruction.step").read_text(encoding="utf-8")
            self.assertNotIn("FACETED_BREP", step_text)
            self.assertEqual(step_text.count("CYLINDRICAL_SURFACE"), 1)
            self.assertGreaterEqual(step_text.count("CIRCLE("), 4)
            self.assertEqual(step_text.count("ADVANCED_FACE"), 8)
            self.assertEqual(step_text.count("PLANE("), 7)
            self.assertGreaterEqual(step_text.count("FACE_BOUND"), 2)

    def test_prismatic_block_with_standoff_exports_clean_cylinders(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            mesh_path = tmp_path / "standoff.stl"
            out_dir = tmp_path / "out"

            vertices, faces = standoff_mesh()
            write_ascii_stl(mesh_path, vertices, faces)
            _, report, _, _ = run_cli(mesh_path, out_dir)

            self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
            self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
            self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)

            step_text = (out_dir / "reconstruction.step").read_text(encoding="utf-8")
            self.assertNotIn("FACETED_BREP", step_text)
            self.assertEqual(step_text.count("CYLINDRICAL_SURFACE"), 2)
            self.assertGreaterEqual(step_text.count("CIRCLE("), 8)
            self.assertEqual(step_text.count("ADVANCED_FACE"), 9)
            self.assertEqual(step_text.count("PLANE("), 7)
            self.assertGreaterEqual(step_text.count("FACE_BOUND"), 4)

    def test_rotated_prismatic_standoff_stays_clean(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            mesh_path = tmp_path / "rotated_standoff.stl"
            out_dir = tmp_path / "out"

            vertices, faces = standoff_mesh()
            rotation = rotation_matrix_xyz(-18.0, 24.0, 37.0)
            transformed_vertices = transform_mesh(
                vertices, rotation=rotation, translation=(14.0, -11.0, 9.0)
            )
            write_ascii_stl(mesh_path, transformed_vertices, faces)
            _, report, _, _ = run_cli(mesh_path, out_dir)

            self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
            self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
            self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)

            step_text = (out_dir / "reconstruction.step").read_text(encoding="utf-8")
            self.assertNotIn("FACETED_BREP", step_text)
            self.assertEqual(step_text.count("CYLINDRICAL_SURFACE"), 2)
            self.assertGreaterEqual(step_text.count("CIRCLE("), 8)
            self.assertEqual(step_text.count("ADVANCED_FACE"), 9)
            self.assertEqual(step_text.count("PLANE("), 7)
            self.assertGreaterEqual(step_text.count("FACE_BOUND"), 4)

    def test_off_center_prismatic_cylindrical_features_stay_clean(self):
        cases = [
            {
                "name": "offset_bore",
                "mesh_fn": box_with_round_bore_mesh,
                "mesh_kwargs": {"center": (6.0, -4.0)},
                "expected_cylinders": 1,
                "expected_advanced_faces": 7,
                "expected_planes": 6,
            },
            {
                "name": "offset_blind_bore",
                "mesh_fn": blind_bore_mesh,
                "mesh_kwargs": {"center": (6.0, -4.0)},
                "expected_cylinders": 1,
                "expected_advanced_faces": 8,
                "expected_planes": 7,
            },
            {
                "name": "offset_counterbore",
                "mesh_fn": counterbore_mesh,
                "mesh_kwargs": {"center": (6.0, -4.0)},
                "expected_cylinders": 2,
                "expected_advanced_faces": 9,
                "expected_planes": 7,
            },
            {
                "name": "offset_blind_counterbore",
                "mesh_fn": blind_counterbore_mesh,
                "mesh_kwargs": {"center": (6.0, -4.0)},
                "expected_cylinders": 2,
                "expected_advanced_faces": 10,
                "expected_planes": 8,
            },
            {
                "name": "offset_boss",
                "mesh_fn": boss_mesh,
                "mesh_kwargs": {"center": (6.0, -4.0)},
                "expected_cylinders": 1,
                "expected_advanced_faces": 8,
                "expected_planes": 7,
            },
            {
                "name": "offset_standoff",
                "mesh_fn": standoff_mesh,
                "mesh_kwargs": {"center": (6.0, -4.0)},
                "expected_cylinders": 2,
                "expected_advanced_faces": 9,
                "expected_planes": 7,
            },
        ]

        for case in cases:
            with self.subTest(case=case["name"]):
                with tempfile.TemporaryDirectory() as tmp:
                    tmp_path = pathlib.Path(tmp)
                    mesh_path = tmp_path / f"{case['name']}.stl"
                    out_dir = tmp_path / "out"

                    vertices, faces = case["mesh_fn"](**case["mesh_kwargs"])
                    write_ascii_stl(mesh_path, vertices, faces)
                    _, report, _, _ = run_cli(mesh_path, out_dir)

                    self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
                    self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
                    self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)

                    step_text = (out_dir / "reconstruction.step").read_text(encoding="utf-8")
                    self.assertNotIn("FACETED_BREP", step_text)
                    self.assertEqual(
                        step_text.count("CYLINDRICAL_SURFACE"), case["expected_cylinders"]
                    )
                    self.assertEqual(
                        step_text.count("ADVANCED_FACE"), case["expected_advanced_faces"]
                    )
                    self.assertEqual(step_text.count("PLANE("), case["expected_planes"])

    def test_rotated_off_center_cylindrical_features_stay_clean(self):
        rotation = rotation_matrix_xyz(23.0, -31.0, 19.0)
        translation = (9.0, -12.0, 15.0)
        cases = [
            {
                "name": "rotated_offset_blind_bore",
                "mesh_fn": blind_bore_mesh,
                "mesh_kwargs": {"center": (5.5, -3.0)},
                "expected_cylinders": 1,
                "expected_advanced_faces": 8,
                "expected_planes": 7,
            },
            {
                "name": "rotated_offset_counterbore",
                "mesh_fn": counterbore_mesh,
                "mesh_kwargs": {"center": (5.5, -3.0)},
                "expected_cylinders": 2,
                "expected_advanced_faces": 9,
                "expected_planes": 7,
            },
            {
                "name": "rotated_offset_blind_counterbore",
                "mesh_fn": blind_counterbore_mesh,
                "mesh_kwargs": {"center": (5.5, -3.0)},
                "expected_cylinders": 2,
                "expected_advanced_faces": 10,
                "expected_planes": 8,
            },
            {
                "name": "rotated_offset_boss",
                "mesh_fn": boss_mesh,
                "mesh_kwargs": {"center": (5.5, -3.0)},
                "expected_cylinders": 1,
                "expected_advanced_faces": 8,
                "expected_planes": 7,
            },
            {
                "name": "rotated_offset_standoff",
                "mesh_fn": standoff_mesh,
                "mesh_kwargs": {"center": (5.5, -3.0)},
                "expected_cylinders": 2,
                "expected_advanced_faces": 9,
                "expected_planes": 7,
            },
        ]

        for case in cases:
            with self.subTest(case=case["name"]):
                with tempfile.TemporaryDirectory() as tmp:
                    tmp_path = pathlib.Path(tmp)
                    mesh_path = tmp_path / f"{case['name']}.stl"
                    out_dir = tmp_path / "out"

                    vertices, faces = case["mesh_fn"](**case["mesh_kwargs"])
                    transformed_vertices = transform_mesh(
                        vertices, rotation=rotation, translation=translation
                    )
                    write_ascii_stl(mesh_path, transformed_vertices, faces)
                    _, report, _, _ = run_cli(mesh_path, out_dir)

                    self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
                    self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
                    self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)

                    step_text = (out_dir / "reconstruction.step").read_text(encoding="utf-8")
                    self.assertNotIn("FACETED_BREP", step_text)
                    self.assertEqual(
                        step_text.count("CYLINDRICAL_SURFACE"), case["expected_cylinders"]
                    )
                    self.assertEqual(
                        step_text.count("ADVANCED_FACE"), case["expected_advanced_faces"]
                    )
                    self.assertEqual(step_text.count("PLANE("), case["expected_planes"])

    def test_rotated_prismatic_block_with_round_bore_stays_clean(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            mesh_path = tmp_path / "rotated_box_with_bore.stl"
            out_dir = tmp_path / "out"

            vertices, faces = box_with_round_bore_mesh()
            rotation = rotation_matrix_xyz(19.0, -27.0, 0.0)
            transformed_vertices = transform_mesh(
                vertices, rotation=rotation, translation=(13.5, -22.0, 8.0)
            )
            write_ascii_stl(mesh_path, transformed_vertices, faces)
            _, report, _, _ = run_cli(mesh_path, out_dir)

            self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
            self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
            self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)

            step_text = (out_dir / "reconstruction.step").read_text(encoding="utf-8")
            self.assertNotIn("FACETED_BREP", step_text)
            self.assertEqual(step_text.count("CYLINDRICAL_SURFACE"), 1)
            self.assertGreaterEqual(step_text.count("CIRCLE("), 4)
            self.assertEqual(step_text.count("ADVANCED_FACE"), 7)
            self.assertEqual(step_text.count("PLANE("), 6)
            self.assertGreaterEqual(step_text.count("FACE_BOUND"), 3)

    def test_prismatic_bore_variants_stay_clean(self):
        cases = [
            {
                "name": "thin_plate",
                "mesh_kwargs": {"half_size": 28.0, "radius": 4.5, "height": 8.0, "segments": 20},
                "rotation": rotation_matrix_xyz(0.0, 0.0, 0.0),
                "translation": (6.0, -9.0, 0.0),
                "expected_advanced_faces": 11,
                "expected_planes": 10,
            },
            {
                "name": "horizontal_bore",
                "mesh_kwargs": {"half_size": 22.0, "radius": 6.0, "height": 18.0, "segments": 18},
                "rotation": rotation_matrix_xyz(0.0, 90.0, 0.0),
                "translation": (-14.0, 5.0, 11.0),
                "expected_advanced_faces": 11,
                "expected_planes": 10,
            },
            {
                "name": "oblique_thick_block",
                "mesh_kwargs": {"half_size": 24.0, "radius": 9.5, "height": 32.0, "segments": 24},
                "rotation": rotation_matrix_xyz(23.0, -31.0, 11.0),
                "translation": (18.0, -12.0, 7.0),
                "expected_advanced_faces": 7,
                "expected_planes": 6,
            },
        ]

        for case in cases:
            with self.subTest(case=case["name"]):
                with tempfile.TemporaryDirectory() as tmp:
                    tmp_path = pathlib.Path(tmp)
                    mesh_path = tmp_path / f"{case['name']}.stl"
                    out_dir = tmp_path / "out"

                    vertices, faces = box_with_round_bore_mesh(**case["mesh_kwargs"])
                    transformed_vertices = transform_mesh(
                        vertices,
                        rotation=case["rotation"],
                        translation=case["translation"],
                    )
                    write_ascii_stl(mesh_path, transformed_vertices, faces)
                    _, report, _, _ = run_cli(mesh_path, out_dir)

                    self.assertEqual(report["reconstruction"]["outcome"], "solid_created")
                    self.assertEqual(report["reconstruction"]["open_edge_count"], 0)
                    self.assertEqual(report["reconstruction"]["non_manifold_edge_count"], 0)

                    step_text = (out_dir / "reconstruction.step").read_text(encoding="utf-8")
                    self.assertNotIn("FACETED_BREP", step_text)
                    self.assertEqual(step_text.count("CYLINDRICAL_SURFACE"), 1)
                    self.assertGreaterEqual(step_text.count("CIRCLE("), 4)
                    self.assertEqual(
                        step_text.count("ADVANCED_FACE"), case["expected_advanced_faces"]
                    )
                    self.assertEqual(step_text.count("PLANE("), case["expected_planes"])
                    self.assertGreaterEqual(step_text.count("FACE_BOUND"), 3)

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
