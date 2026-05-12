import math
import struct
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple

from config import (
    PPM,
    BLOCK_EFFECTIVE_DIAM_MM,
    BLOCK_CROSS_SECTION_SIDES,
    BLOCK_CORNER_RADIUS_MM,
    BLOCK_FLAT_RADIUS_MM,
)
from vec2 import Vec2


def _mm_to_px(mm: float) -> float:
    return (mm / 25.4) * PPM


def _mm_to_m(mm: float) -> float:
    return mm / 1000.0


@dataclass(frozen=True)
class TriballMeshData:
    source_path: Path | None
    collision_obj_path: Path
    centered_vertices_mm: List[Tuple[float, float, float]]
    centered_vertices_m: List[Tuple[float, float, float]]
    render_sample_vertices_px3d: List[Tuple[float, float, float]]
    triangles: List[Tuple[int, int, int]]
    render_footprint_px: List[Vec2]
    width_m: float
    depth_m: float
    height_m: float


COLLISION_SMOOTH_BLEND = 0.72
COLLISION_PROXY_SIDES = max(12, BLOCK_CROSS_SECTION_SIDES)
COLLISION_PROXY_Z_RINGS = (
    (-1.00, 0.72),
    (-0.52, 0.94),
    (0.00, 1.00),
    (0.52, 0.94),
    (1.00, 0.72),
)


def _convex_hull(points: Sequence[Vec2]) -> List[Vec2]:
    unique = sorted({(round(p.x, 6), round(p.y, 6)) for p in points})
    if len(unique) <= 1:
        return [Vec2(*p) for p in unique]

    def cross(o, a, b):
        return ((a[0] - o[0]) * (b[1] - o[1])) - ((a[1] - o[1]) * (b[0] - o[0]))

    lower = []
    for p in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper = []
    for p in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    hull = lower[:-1] + upper[:-1]
    return [Vec2(x, y) for x, y in hull]


def _parse_binary_stl(path: Path):
    raw = path.read_bytes()
    if len(raw) < 84:
        raise ValueError("STL file too small")

    tri_count = struct.unpack("<I", raw[80:84])[0]
    expected = 84 + (tri_count * 50)
    if expected != len(raw):
        raise ValueError("STL is not a valid binary STL")

    verts: List[Tuple[float, float, float]] = []
    triangles: List[Tuple[int, int, int]] = []
    index_map = {}
    off = 84

    for _ in range(tri_count):
        off += 12  # normal
        tri_idx = []
        for _ in range(3):
            x, y, z = struct.unpack("<fff", raw[off:off + 12])
            key = (round(x, 6), round(y, 6), round(z, 6))
            idx = index_map.get(key)
            if idx is None:
                idx = len(verts)
                verts.append((x, y, z))
                index_map[key] = idx
            tri_idx.append(idx)
            off += 12
        triangles.append((tri_idx[0], tri_idx[1], tri_idx[2]))
        off += 2  # attribute byte count

    return verts, triangles


def _build_fallback_mesh():
    corner_r_mm = BLOCK_CORNER_RADIUS_MM
    flat_r_mm = BLOCK_FLAT_RADIUS_MM
    height_mm = max(corner_r_mm * 1.68, 75.0)

    ring_bottom: List[Tuple[float, float, float]] = []
    ring_top: List[Tuple[float, float, float]] = []
    for i in range(BLOCK_CROSS_SECTION_SIDES):
        theta = (math.tau * i) / BLOCK_CROSS_SECTION_SIDES
        radius = corner_r_mm if (i % 2 == 0) else flat_r_mm
        x = math.cos(theta) * radius
        y = math.sin(theta) * radius
        ring_bottom.append((x, y, -height_mm * 0.5))
        ring_top.append((x, y, height_mm * 0.5))

    verts = ring_bottom + ring_top
    triangles: List[Tuple[int, int, int]] = []
    n = BLOCK_CROSS_SECTION_SIDES

    for i in range(1, n - 1):
        triangles.append((0, i, i + 1))
        triangles.append((n, n + i + 1, n + i))

    for i in range(n):
        a = i
        b = (i + 1) % n
        c = n + b
        d = n + a
        triangles.append((a, b, c))
        triangles.append((a, c, d))

    return verts, triangles


def _center_vertices(vertices: Sequence[Tuple[float, float, float]]):
    min_x = min(v[0] for v in vertices)
    min_y = min(v[1] for v in vertices)
    min_z = min(v[2] for v in vertices)
    max_x = max(v[0] for v in vertices)
    max_y = max(v[1] for v in vertices)
    max_z = max(v[2] for v in vertices)

    cx = 0.5 * (min_x + max_x)
    cy = 0.5 * (min_y + max_y)
    cz = 0.5 * (min_z + max_z)

    centered_mm = [(x - cx, y - cy, z - cz) for x, y, z in vertices]
    centered_m = [(_mm_to_m(x), _mm_to_m(y), _mm_to_m(z)) for x, y, z in centered_mm]
    width_m = _mm_to_m(max_x - min_x)
    depth_m = _mm_to_m(max_y - min_y)
    height_m = _mm_to_m(max_z - min_z)

    return centered_mm, centered_m, width_m, depth_m, height_m


def _write_obj(path: Path, vertices_m: Sequence[Tuple[float, float, float]], triangles: Sequence[Tuple[int, int, int]]):
    lines = ["o triball"]
    for x, y, z in vertices_m:
        lines.append(f"v {x:.8f} {y:.8f} {z:.8f}")
    for a, b, c in triangles:
        lines.append(f"f {a + 1} {b + 1} {c + 1}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _smoothed_collision_vertices(centered_mm: Sequence[Tuple[float, float, float]]):
    radii = [math.sqrt((x * x) + (y * y) + (z * z)) for x, y, z in centered_mm]
    avg_radius = sum(radii) / max(1, len(radii))
    smoothed = []
    for (x, y, z), r in zip(centered_mm, radii):
        if r < 1e-6:
            smoothed.append((x, y, z))
            continue
        unit = (x / r, y / r, z / r)
        sphere_pos = (unit[0] * avg_radius, unit[1] * avg_radius, unit[2] * avg_radius)
        blend = COLLISION_SMOOTH_BLEND
        smoothed.append((
            x + ((sphere_pos[0] - x) * blend),
            y + ((sphere_pos[1] - y) * blend),
            z + ((sphere_pos[2] - z) * blend),
        ))
    return smoothed


def _build_collision_proxy_mesh(width_m: float, depth_m: float, height_m: float):
    """Build a compact STL-derived collision proxy for Bullet.

    The visual/render data still comes from the full STL, but the dynamic
    collider must stay small because every triball can contact many others.
    """
    half_w_mm = width_m * 500.0
    half_d_mm = depth_m * 500.0
    half_h_mm = height_m * 500.0
    flat_ratio = BLOCK_FLAT_RADIUS_MM / max(1e-6, BLOCK_CORNER_RADIUS_MM)

    vertices: List[Tuple[float, float, float]] = []
    for z_norm, radius_scale in COLLISION_PROXY_Z_RINGS:
        z = z_norm * half_h_mm
        for i in range(COLLISION_PROXY_SIDES):
            theta = (math.tau * i) / COLLISION_PROXY_SIDES
            facet_ratio = 1.0 if (i % 2 == 0) else flat_ratio
            x = math.cos(theta) * half_w_mm * radius_scale * facet_ratio
            y = math.sin(theta) * half_d_mm * radius_scale * facet_ratio
            vertices.append((x, y, z))

    triangles: List[Tuple[int, int, int]] = []
    ring_count = len(COLLISION_PROXY_Z_RINGS)
    n = COLLISION_PROXY_SIDES
    for ring in range(ring_count - 1):
        start = ring * n
        next_start = (ring + 1) * n
        for i in range(n):
            a = start + i
            b = start + ((i + 1) % n)
            c = next_start + ((i + 1) % n)
            d = next_start + i
            triangles.append((a, b, c))
            triangles.append((a, c, d))

    bottom_center = len(vertices)
    vertices.append((0.0, 0.0, -half_h_mm))
    top_center = len(vertices)
    vertices.append((0.0, 0.0, half_h_mm))

    top_start = (ring_count - 1) * n
    for i in range(n):
        a = i
        b = (i + 1) % n
        triangles.append((bottom_center, b, a))

        ta = top_start + i
        tb = top_start + ((i + 1) % n)
        triangles.append((top_center, ta, tb))

    return vertices, triangles


def load_triball_mesh(asset_path: Path) -> TriballMeshData:
    if asset_path.exists():
        vertices, triangles = _parse_binary_stl(asset_path)
        source_path = asset_path
    else:
        vertices, triangles = _build_fallback_mesh()
        source_path = None

    centered_mm, centered_m, width_m, depth_m, height_m = _center_vertices(vertices)
    planar_diameter_mm = max(width_m, depth_m) * 1000.0
    geom_scale = 1.0
    if planar_diameter_mm > 1e-6:
        geom_scale = BLOCK_EFFECTIVE_DIAM_MM / planar_diameter_mm

    scaled_centered_mm = [
        (x * geom_scale, y * geom_scale, z * geom_scale)
        for x, y, z in centered_mm
    ]
    scaled_centered_m = [(_mm_to_m(x), _mm_to_m(y), _mm_to_m(z)) for x, y, z in scaled_centered_mm]
    width_m *= geom_scale
    depth_m *= geom_scale
    height_m *= geom_scale

    hull_points = _convex_hull([Vec2(_mm_to_px(x), _mm_to_px(y)) for x, y, _ in scaled_centered_mm])
    collision_mm, collision_triangles = _build_collision_proxy_mesh(width_m, depth_m, height_m)
    collision_m = [(_mm_to_m(x), _mm_to_m(y), _mm_to_m(z)) for x, y, z in collision_mm]
    # The renderer only needs enough points to hint roll/pitch;
    # keeping this set compact reduces per-frame hull work noticeably.
    sample_step = max(1, len(scaled_centered_mm) // 64)
    render_sample_vertices_px3d = [
        (_mm_to_px(x), _mm_to_px(y), _mm_to_px(z))
        for i, (x, y, z) in enumerate(scaled_centered_mm)
        if i % sample_step == 0
    ]

    obj_path = Path(tempfile.gettempdir()) / "vexsim_triball_collision.obj"
    _write_obj(obj_path, collision_m, collision_triangles)

    return TriballMeshData(
        source_path=source_path,
        collision_obj_path=obj_path,
        centered_vertices_mm=list(scaled_centered_mm),
        centered_vertices_m=list(scaled_centered_m),
        render_sample_vertices_px3d=render_sample_vertices_px3d,
        triangles=list(collision_triangles),
        render_footprint_px=hull_points,
        width_m=width_m,
        depth_m=depth_m,
        height_m=height_m,
    )
