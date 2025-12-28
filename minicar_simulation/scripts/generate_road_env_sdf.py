import numpy as np
import cv2
import math
from pathlib import Path


# scripts/generate_world.py の場所
SCRIPT_DIR = Path(__file__).resolve().parent

# パッケージルート（scripts の1個上）
PKG_ROOT = SCRIPT_DIR.parent

WORLDS_DIR = PKG_ROOT / "worlds"
MODELS_DIR = PKG_ROOT / "models"

def add_xy(p, v):
    return (p[0] + v[0], p[1] + v[1])

def cubic_bezier_points(P0, P1, P2, P3, num_points=300):
    P0 = np.array(P0, dtype=np.float32)
    P1 = np.array(P1, dtype=np.float32)
    P2 = np.array(P2, dtype=np.float32)
    P3 = np.array(P3, dtype=np.float32)

    t = np.linspace(0.0, 1.0, int(num_points), dtype=np.float32)[:, None]
    B = ((1 - t) ** 3) * P0 \
        + 3 * ((1 - t) ** 2) * t * P1 \
        + 3 * (1 - t) * (t ** 2) * P2 \
        + (t ** 3) * P3

    return [(float(x), float(y)) for x, y in B]

def line_points(P0, P1, num_points=200):
    P0 = np.array(P0, dtype=np.float32)
    P1 = np.array(P1, dtype=np.float32)
    t = np.linspace(0.0, 1.0, int(num_points), dtype=np.float32)[:, None]
    pts = (1 - t) * P0 + t * P1
    return [(float(x), float(y)) for x, y in pts]

def build_routes_from_pts_vec(pts, vec1, vec2, vec3, vec4, scale=0.01):
    b_left_outer = cubic_bezier_points(pts[0], add_xy(pts[0], vec1[0]), add_xy(pts[1], vec1[1]), pts[1], 300)
    b_left_inner = cubic_bezier_points(pts[0], add_xy(pts[0], vec2[0]), add_xy(pts[1], vec2[1]), pts[1], 300)
    b_right_outer = cubic_bezier_points(pts[2], add_xy(pts[2], vec3[0]), add_xy(pts[3], vec3[1]), pts[3], 300)
    b_right_inner = cubic_bezier_points(pts[2], add_xy(pts[2], vec4[0]), add_xy(pts[3], vec4[1]), pts[3], 300)

    l_top = line_points(pts[1], pts[2], 200)
    l_bottom = line_points(pts[3], pts[0], 200)

    outer_route_px = b_left_outer + l_top + b_right_outer + l_bottom
    inner_route_px = b_left_inner + l_top + b_right_inner + l_bottom

    # px -> m
    def s(p): return (p[0] * scale, p[1] * scale)
    routes_m = [
        [s(p) for p in outer_route_px],
        [s(p) for p in inner_route_px],
    ]
    return routes_m

def rasterize_centerlines(routes_xy_m, img_wh, meters_per_pixel, line_thickness_px=1):
    W, H = img_wh
    mask = np.zeros((H, W), dtype=np.uint8)

    def to_px(p_m):
        x = int(round(p_m[0] / meters_per_pixel))
        y = int(round(p_m[1] / meters_per_pixel))
        return (x, y)

    for r in routes_xy_m:
        if len(r) < 2:
            continue
        pts = np.array([to_px(p) for p in r], dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(mask, [pts], isClosed=False, color=255,
                      thickness=line_thickness_px, lineType=cv2.LINE_AA)

    _, mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
    return mask

def thicken_by_morphology_px(center_mask, road_width_px):
    """
    road_width_px : 道幅（ピクセル）
    """
    r_px = max(1, int(round(road_width_px / 2)))
    k = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (2 * r_px + 1, 2 * r_px + 1)
    )
    thick = cv2.dilate(center_mask, k, iterations=1)
    _, thick = cv2.threshold(thick, 127, 255, cv2.THRESH_BINARY)
    return thick

def mask_to_polygons(mask, approx_eps_px=2.0):
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if hierarchy is None or len(contours) == 0:
        return []

    hierarchy = hierarchy[0]
    polys = []
    used = set()

    def approx(cnt):
        return cv2.approxPolyDP(cnt, epsilon=float(approx_eps_px), closed=True)

    for i, h in enumerate(hierarchy):
        parent = h[3]
        if parent != -1:
            continue  # outer only
        if i in used:
            continue

        outer = approx(contours[i])
        outer = [(int(p[0][0]), int(p[0][1])) for p in outer]

        holes = []
        child = h[2]
        while child != -1:
            hole = approx(contours[child])
            hole = [(int(p[0][0]), int(p[0][1])) for p in hole]
            holes.append(hole)
            used.add(child)
            child = hierarchy[child][0]

        used.add(i)
        polys.append({"outer": outer, "holes": holes})

    return polys

def polygons_to_thick_wall_triangles(
    polys,
    wall_height_m,
    wall_thickness_px,
    meters_per_pixel,
    img_wh,          # ← ★ 追加
    z0=0.0,
):
    half = wall_thickness_px / 2.0

    def px_to_m(p):
        cx = img_wh[0] / 2.0
        cy = img_wh[1] / 2.0
        return (
            (p[0] - cx) * meters_per_pixel,
            (p[1] - cy) * meters_per_pixel
        )
    
    def ring_to_tris(ring_outer, ring_inner):
        tris = []
        n = len(ring_outer)

        for i in range(n):
            a0 = ring_outer[i]
            b0 = ring_outer[(i + 1) % n]
            a1 = ring_inner[i]
            b1 = ring_inner[(i + 1) % n]

            A0 = (*px_to_m(a0), z0)
            B0 = (*px_to_m(b0), z0)
            A1 = (*px_to_m(a1), z0)
            B1 = (*px_to_m(b1), z0)

            A0u = (A0[0], A0[1], z0 + wall_height_m)
            B0u = (B0[0], B0[1], z0 + wall_height_m)
            A1u = (A1[0], A1[1], z0 + wall_height_m)
            B1u = (B1[0], B1[1], z0 + wall_height_m)

            # 外側面
            tris.append((A0, B0, B0u))
            tris.append((A0, B0u, A0u))

            # 内側面
            tris.append((B1, A1, A1u))
            tris.append((B1, A1u, B1u))

            # 上面
            tris.append((A0u, B0u, B1u))
            tris.append((A0u, B1u, A1u))

        return tris

    triangles = []

    for p in polys:
        # 外周
        out_o = offset_polygon_px(p["outer"], +half)
        out_i = offset_polygon_px(p["outer"], -half)
        triangles.extend(ring_to_tris(out_o, out_i))

        # 穴（向きが逆）
        for h in p["holes"]:
            h_o = offset_polygon_px(h, -half)
            h_i = offset_polygon_px(h, +half)
            triangles.extend(ring_to_tris(h_o, h_i))

    return triangles


def make_triangles_double_sided(triangles):
    out = []
    for (a, b, c) in triangles:
        out.append((a, b, c))     # 元
        out.append((a, c, b))     # 裏
    return out


def routes_to_wall_mesh_triangles_px(
    routes_xy_m,
    road_width_px,
    wall_height_m,
    wall_thickness_px,     # ← 追加
    meters_per_pixel,
    img_wh,
    approx_eps_px=2.0,
    save_debug_dir=None,
):
    center = rasterize_centerlines(routes_xy_m, img_wh, meters_per_pixel)
    thick = thicken_by_morphology_px(center, road_width_px)
    polys = mask_to_polygons(thick, approx_eps_px=approx_eps_px)

    # ★ ここで呼ぶ ★
    triangles = polygons_to_thick_wall_triangles(
        polys,
        wall_height_m=wall_height_m,
        wall_thickness_px=wall_thickness_px,
        meters_per_pixel=meters_per_pixel,
        img_wh=img_wh,      # ← ★ 追加
    )
    triangles = make_triangles_double_sided(triangles)
    return triangles, polys

def offset_polygon_px(poly, offset_px):
    """
    poly: [(x_px, y_px), ...]  ※閉じていなくてOK
    offset_px: +外側, -内側
    """
    pts = np.array(poly, dtype=np.float32)
    n = len(pts)
    out = []

    for i in range(n):
        p_prev = pts[i - 1]
        p_curr = pts[i]
        p_next = pts[(i + 1) % n]

        # エッジ法線（前後）
        v1 = p_curr - p_prev
        v2 = p_next - p_curr

        def norm(v):
            L = math.hypot(v[0], v[1]) + 1e-9
            return np.array([-v[1] / L, v[0] / L], dtype=np.float32)

        n1 = norm(v1)
        n2 = norm(v2)

        # 角の2等分線方向
        n_avg = n1 + n2
        L = math.hypot(n_avg[0], n_avg[1])
        if L < 1e-6:
            n_avg = n1
        else:
            n_avg /= L

        out.append(p_curr + n_avg * offset_px)

    return [(float(p[0]), float(p[1])) for p in out]

def write_ascii_stl(path, triangles, solid_name="mesh"):
    def normal(tri):
        (ax, ay, az), (bx, by, bz), (cx, cy, cz) = tri
        ux, uy, uz = (bx-ax, by-ay, bz-az)
        vx, vy, vz = (cx-ax, cy-ay, cz-az)
        nx, ny, nz = (uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx)
        L = math.sqrt(nx*nx + ny*ny + nz*nz) + 1e-12
        return (nx/L, ny/L, nz/L)

    with open(path, "w", encoding="utf-8") as f:
        f.write(f"solid {solid_name}\n")
        for tri in triangles:
            nx, ny, nz = normal(tri)
            f.write(f"  facet normal {nx:.6e} {ny:.6e} {nz:.6e}\n")
            f.write("    outer loop\n")
            for (x, y, z) in tri:
                f.write(f"      vertex {x:.6e} {y:.6e} {z:.6e}\n")
            f.write("    endloop\n")
            f.write("  endfacet\n")
        f.write(f"endsolid {solid_name}\n")

def export_gazebo_model_from_triangles(
    triangles,
    model_name,
    mesh_filename="walls.stl",
    sdf_version="1.7",
):
    """
    triangles -> meshes/*.stl + model.sdf + model.config を生成
    """
    mesh_dir = MODELS_DIR / model_name / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    stl_path = mesh_dir / mesh_filename
    write_ascii_stl(str(stl_path), triangles, solid_name="walls")

    model_config = f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="{sdf_version}">model.sdf</sdf>
  <author><name>generated</name></author>
  <description>Generated from routes -> raster -> morphology -> contours -> extrusion.</description>
</model>
"""

    # uri は model://<dir名>/meshes/<file>
    model_sdf = f"""<?xml version="1.0"?>
<sdf version="{sdf_version}">
  <model name="{model_name}">
    <static>true</static>

    <link name="walls">
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://{model_name}/meshes/{mesh_filename}</uri>
          </mesh>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://{model_name}/meshes/{mesh_filename}</uri>
          </mesh>
        </geometry>

        <cast_shadows>false</cast_shadows>
        <lighting>false</lighting>

        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
          <specular>0 0 0 1</specular>
        </material>

      </visual>
    </link>
  </model>
</sdf>
"""
    (MODELS_DIR / model_name / "model.config").write_text(model_config, encoding="utf-8")
    (MODELS_DIR / model_name / "model.sdf").write_text(model_sdf, encoding="utf-8")

if __name__ == "__main__":
    # ---- あなたのデータ（そのまま） ----
    pts = [
        (320, 400), (320, 100),
        (480, 100), (480, 400)
    ]
    vec1 = [(-250, 0), (-250, 0)]
    vec2 = [(-100, 0), (-100, 0)]
    vec3 = [(250, 0), (250, 0)]
    vec4 = [(100, 0), (100, 0)]
    
    routes = build_routes_from_pts_vec(pts, vec1, vec2, vec3, vec4, scale=0.01)  # 1px=1cm
    
    triangles, polys = routes_to_wall_mesh_triangles_px(
        routes_xy_m=routes,
        road_width_px=60,          # 道幅（px）
        wall_height_m=0.3,         # 壁高さ（m）
        wall_thickness_px=5,      # ← ★ 壁厚み（px）を追加
        meters_per_pixel=0.01,     # 1px = 1cm
        img_wh=(800, 500),
        approx_eps_px=2.0,
        save_debug_dir="road_env_model/debug"
    )
    
    export_gazebo_model_from_triangles(
        triangles,
        model_name="road_env",
        mesh_filename="walls.stl"
    )
