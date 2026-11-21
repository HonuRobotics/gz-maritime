#!/usr/bin/env python3
"""
Build the shipwreck, modeled on the Monohansett, as profile-conforming parts.

Authored in Blender's own convention: bow toward -Y, up +Z, so the exported
glTF faces +Z with +Y up (profile Axes section). One object per part, one
part per .glb, named for the part (profile Scenes and nodes section).

Model frame, shared by every part (the datum, from the spec in
tools/maritime-workspace/notes/sonar_design/shipwreck.md):
    origin  on the seabed plane, under the keel, at the boiler's forward face
    z = 0   the seabed plane; anything below it is buried
    -Y      forward, toward the bow

Parts, one .glb each, split by acoustic class and by scored feature (spec decision 7):
    hull (wood)  mast (wood)  boiler, engine, debris (iron)  propeller, bell (bronze)
    name_plate (painted board)  coal

Writes, into the model directory given after --:
    meshes/<part>_visual.glb
    model.sdf                          visuals with acoustic values, primitive collisions
    ground_truth/features.yaml         named features, Gazebo model frame
and, as build artifacts in $SHIPWRECK_BUILD_DIR (default ~/maritime_ws/build/shipwreck): shipwreck.blend, and shipwreck_points.ply, surface
samples above the seabed in the Gazebo model frame. The meshes are the exact ground
truth; the point cloud is a convenience copy of them.

Gazebo model frame from Blender's: x = -y_b, y = x_b, z = z_b (bow at +X), which
is what the profile's visual pose (roll 90, yaw 90) does to the exported file.

    blender --background --factory-startup --python build_shipwreck.py -- models/shipwreck
"""
import math, os, random, struct, sys
import bpy, bmesh
from mathutils import Matrix, Vector

# --- Parameters: Monohansett at true size (NOAA), the rest authored ---------
L = 48.8            # length, m
B = 9.1             # beam, m
S_DATUM = 33.0      # datum (boiler forward face) as distance aft of the bow, m
BURY = 0.20         # how far the hull bottom sits below the seabed plane, m

SECTIONS = [        # (start, end) as distance aft of the bow, m; gaps are breaks
    (0.0, 13.0),    # bow section
    (15.0, 31.0),   # midship section, the burned-out hold
    (32.0, L),      # aft section, the most intact
]
STANDING_FROM = 38.0   # aft of this, the sides stand rather than lie splayed
FRAME_PITCH = 0.6      # floor spacing, m
FRAME_W, FRAME_H = 0.30, 0.30
KEELSON_W, KEELSON_H = 0.45, 0.55
SIDE_T = 0.25          # side planking thickness, m
SIDE_DEPTH = 2.8       # depth of a splayed side, bilge to sheer, m
SPLAY_DROP = 0.35      # how much lower the sheer lies than the bilge when splayed, m
BILGE_Z = 0.5
STAND_H = 2.2          # height of the standing aft sides, m
STAND_LEAN = math.radians(15)
DECK_Z, DECK_T = 2.0, 0.2
DECK_SPAN = (42.0, 48.0)
HATCH = (44.0, 46.0, 1.0)  # hatch from, to, half-width, m

# Machinery, stern gear and the added features (Blender frame)
BOILER_R, BOILER_LEN, BOILER_Z = 1.5, 4.0, 1.8     # forward face at y = 0, the datum
BOILER_LIST = math.radians(4)
ENGINE_Y = (4.5, 6.5)
SHAFT_Z = 1.3
PROP_Y, PROP_R = 16.3, 1.2
BELL_AT = (2.2, 10.0, DECK_Z + DECK_T)             # lying on its side on the aft deck
PLATE_AT, PLATE_SIZE = (-5.1, 11.0, 0.15), (0.05, 2.0, 0.3)
MAST_FROM, MAST_TO, MAST_R = (8.0, -22.5), (9.6, -7.6), 0.175
COAL_Y, COAL_HALF_W, COAL_H = (-16.0, -4.0), 3.3, 0.45
GT_SPACING = 0.05      # ground-truth point spacing, m
GT_SEED = 1907

out = os.path.abspath(sys.argv[sys.argv.index("--") + 1] if "--" in sys.argv else "models/shipwreck")
for d in ("meshes", "ground_truth"):
    os.makedirs(f"{out}/{d}", exist_ok=True)
# Build artifacts (the .blend, the optional point cloud) stay out of the model.
# Build artifacts (the .blend, the point cloud) go outside the source tree.
build = os.environ.get("SHIPWRECK_BUILD_DIR", os.path.expanduser("~/maritime_ws/build/shipwreck"))
os.makedirs(build, exist_ok=True)


def y_of(s):
    """Blender y of a station s metres aft of the bow."""
    return s - S_DATUM


def half_breadth(s):
    """Plan shape: fine bow over 10 m, parallel body, rounded stern over 6 m."""
    if s < 10.0:
        return B / 2 * math.sin(math.pi / 2 * max(s, 0.05) / 10.0)
    if s > L - 6.0:
        u = (s - (L - 6.0)) / 6.5
        return B / 2 * math.sqrt(max(1.0 - u * u, 0.15))
    return B / 2


def stations(a, b, step):
    n = max(1, round((b - a) / step))
    return [a + (b - a) * i / n for i in range(n + 1)]


def hexa(bm, c):
    """Add a closed hexahedron from 8 corners: c[0:4] one end, c[4:8] the other, same winding."""
    v = [bm.verts.new(p) for p in c]
    for f in ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
        bm.faces.new([v[i] for i in f])


def box(bm, lo, hi):
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    hexa(bm, [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1),
              (x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1)])


def side_profile(s, side, standing):
    """Four corners of the side planking's cross-section at station s, port (+1) or starboard (-1)."""
    b = half_breadth(s)
    if standing:
        h = STAND_H
        dx, dz = h * math.sin(STAND_LEAN), h * math.cos(STAND_LEAN)
        inner = [(b, -BURY), (b + dx, dz - BURY)]
        nx, nz = math.cos(STAND_LEAN) * SIDE_T, -math.sin(STAND_LEAN) * SIDE_T
    else:
        dz = -SPLAY_DROP
        dx = math.sqrt(SIDE_DEPTH ** 2 - dz ** 2)
        inner = [(b, BILGE_Z), (b + dx, BILGE_Z + dz)]
        ang = math.atan2(-dz, dx)
        nx, nz = -math.sin(ang) * SIDE_T, -math.cos(ang) * SIDE_T
    pts = [inner[0], inner[1], (inner[1][0] + nx, inner[1][1] + nz), (inner[0][0] + nx, inner[0][1] + nz)]
    return [(side * x, y_of(s), z) for x, z in pts]


def build_hull(bm):
    for a, b in SECTIONS:
        # keelson, on the centerline, the full section
        box(bm, (-KEELSON_W / 2, y_of(a + 0.3), -BURY), (KEELSON_W / 2, y_of(b - 0.3), KEELSON_H - BURY))
        # floors: transverse frames across the bottom
        for s in stations(a + 0.4, b - 0.4, FRAME_PITCH):
            hb = half_breadth(s) - 0.1
            box(bm, (-hb, y_of(s) - FRAME_W / 2, -BURY), (hb, y_of(s) + FRAME_W / 2, FRAME_H))
        # sides, as strips of hexahedra between stations; splayed forward, standing aft
        for lo, hi, standing in ((a, min(b, STANDING_FROM), False), (max(a, STANDING_FROM), b, True)):
            if hi - lo < 0.5:
                continue
            st = stations(lo, hi, 1.0)
            for side in (1, -1):
                for s0, s1 in zip(st, st[1:]):
                    p0, p1 = side_profile(s0, side, standing), side_profile(s1, side, standing)
                    if side < 0:   # mirror flips the winding; keep the faces outward
                        p0, p1 = p0[::-1], p1[::-1]
                    hexa(bm, p0 + p1)
    # stem post at the bow, raked forward to the bow, about 1.2 m proud
    hexa(bm, [(-0.2, y_of(0.6), -BURY), (0.2, y_of(0.6), -BURY), (0.2, y_of(2.1), -BURY), (-0.2, y_of(2.1), -BURY),
              (-0.2, y_of(0.0), 1.2), (0.2, y_of(0.0), 1.2), (0.2, y_of(1.0), 1.2), (-0.2, y_of(1.0), 1.2)])
    # stern post, about 2.6 m proud
    box(bm, (-0.2, y_of(L - 0.8), -BURY), (0.2, y_of(L - 0.2), 2.6))
    # aft deck remnant, burned through at the hatch
    d0, d1 = DECK_SPAN
    h0, h1, hw = HATCH
    w = min(half_breadth(d0), half_breadth(d1)) + STAND_H * math.sin(STAND_LEAN) * 0.85
    for lo, hi in (((-w, y_of(d0), DECK_Z), (w, y_of(h0), DECK_Z + DECK_T)),
                   ((-w, y_of(h1), DECK_Z), (w, y_of(d1), DECK_Z + DECK_T)),
                   ((-w, y_of(h0), DECK_Z), (-hw, y_of(h1), DECK_Z + DECK_T)),
                   ((hw, y_of(h0), DECK_Z), (w, y_of(h1), DECK_Z + DECK_T))):
        box(bm, lo, hi)


# --- Primitives with an orientation ----------------------------------------

def frame(center, axis=(0, 0, 1), roll=0.0):
    """Matrix placing local +Z along axis at center, then rolled about that axis."""
    q = Vector((0, 0, 1)).rotation_difference(Vector(axis).normalized())
    return Matrix.Translation(center) @ q.to_matrix().to_4x4() @ Matrix.Rotation(roll, 4, 'Z')


def obox(bm, size, m):
    """Add a box of the given size, centered on the local origin of matrix m."""
    bmesh.ops.create_cube(bm, size=1.0, matrix=m @ Matrix.Diagonal((*size, 1.0)))


def cyl(bm, r, length, m, segments=16):
    """Add a capped cylinder along local +Z of matrix m, centered on its origin."""
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=segments,
                          radius1=r, radius2=r, depth=length, matrix=m)


def lathe(bm, profile, m, segments=24):
    """Solid of revolution about local +Z from a closed (r, z) profile; no point on the axis."""
    rings = []
    for i in range(segments):
        a = 2 * math.pi * i / segments
        rings.append([bm.verts.new(m @ Vector((r * math.cos(a), r * math.sin(a), z))) for r, z in profile])
    n = len(profile)
    for i in range(segments):
        r0, r1 = rings[i], rings[(i + 1) % segments]
        for j in range(n):
            bm.faces.new((r0[j], r0[(j + 1) % n], r1[(j + 1) % n], r1[j]))


def strip(bm, points, size):
    """Add a row of boxes (chain links) along a polyline, alternating their roll."""
    k = 0
    for p0, p1 in zip(points, points[1:]):
        p0, p1 = Vector(p0), Vector(p1)
        n = max(1, round((p1 - p0).length / size[2]))
        for i in range(n):
            c = p0 + (p1 - p0) * ((i + 0.5) / n)
            obox(bm, size, frame(c, p1 - p0, roll=(math.pi / 2) * (k % 2)))
            k += 1


# --- The parts -------------------------------------------------------------

def build_boiler(bm):
    m = Matrix.Translation((0, BOILER_LEN / 2, BOILER_Z)) @ Matrix.Rotation(BOILER_LIST, 4, 'Y')
    cyl(bm, BOILER_R, BOILER_LEN, m @ frame((0, 0, 0), (0, 1, 0)), segments=24)
    cyl(bm, 0.45, 0.8, m @ frame((0, -BOILER_LEN / 2 + 0.8, BOILER_R + 0.3)))   # stack stub


def build_engine(bm):
    y0, y1 = ENGINE_Y
    box(bm, (-1.0, y0, 0.3), (1.0, y1, 2.3))
    for y in (y0 + 0.5, y1 - 0.5):                        # two cylinders, compound engine
        cyl(bm, 0.4, 0.9, frame((0, y, 2.75)))
    cyl(bm, 0.15, PROP_Y - 0.3 - y1, frame((0, (y1 + PROP_Y - 0.3) / 2, SHAFT_Z), (0, 1, 0)), 12)


def build_propeller(bm):
    hub = Vector((0, PROP_Y, SHAFT_Z))
    cyl(bm, 0.25, 0.6, frame(hub, (0, 1, 0)), 16)
    pitch = math.radians(35)
    for a in (0, 90, 180, 270):
        a = math.radians(a)
        radial = Vector((math.cos(a), 0, math.sin(a)))
        chord = Vector((-math.sin(a), 0, math.cos(a))) * math.cos(pitch) + Vector((0, 1, 0)) * math.sin(pitch)
        thick = radial.cross(chord).normalized() * 0.03
        secs = []
        for r, c in ((0.2, 0.5), (0.75, 0.8), (PROP_R, 0.35)):
            p = hub + radial * r
            h = chord * (c / 2)
            secs.append([p - h - thick, p + h - thick, p + h + thick, p - h + thick])
        for s0, s1 in zip(secs, secs[1:]):
            hexa(bm, s0 + s1)


def build_bell(bm):
    # outer then inner profile, (r, z), mouth at z = 0; lying on its side, mouth to port
    outer = [(0.25, 0.0), (0.23, 0.08), (0.19, 0.22), (0.17, 0.36), (0.12, 0.44), (0.04, 0.46)]
    inner = [(0.04, 0.42), (0.10, 0.40), (0.14, 0.34), (0.16, 0.22), (0.20, 0.08), (0.22, 0.0)]
    x, y, z = BELL_AT
    lathe(bm, outer + inner, frame((x, y, z + 0.25), (1, 0, 0)))


def build_name_plate(bm):
    x, y, z = PLATE_AT
    obox(bm, PLATE_SIZE, Matrix.Translation((x, y, z)) @ Matrix.Rotation(math.radians(-25), 4, 'Y'))


def build_mast(bm):
    (x0, y0), (x1, y1) = MAST_FROM, MAST_TO
    p0, p1 = Vector((x0, y0, MAST_R - 0.03)), Vector((x1, y1, MAST_R - 0.03))
    cyl(bm, MAST_R, (p1 - p0).length, frame((p0 + p1) / 2, p1 - p0), 12)


def build_debris(bm):
    # anchor, lying flat off the port bow: shank, crown arms, stock
    ax, ay = 5.5, -30.5
    box(bm, (ax - 0.1, ay - 1.0, 0.0), (ax + 0.1, ay + 1.0, 0.2))
    for s in (1, -1):
        obox(bm, (0.15, 0.9, 0.15), Matrix.Translation((ax + s * 0.35, ay - 0.85, 0.08)) @ Matrix.Rotation(s * math.radians(55), 4, 'Z'))
    obox(bm, (2.2, 0.15, 0.15), Matrix.Translation((ax, ay + 0.9, 0.2)))
    # windlass on the bow floors: drum between two cheeks
    wy = -29.0
    cyl(bm, 0.3, 1.6, frame((0, wy, 0.75), (1, 0, 0)), 16)
    for s in (1, -1):
        box(bm, (s * 0.9 - 0.08, wy - 0.4, 0.3), (s * 0.9 + 0.08, wy + 0.4, 1.2))
    # chain from the windlass over the port side to the anchor ring
    strip(bm, [(0.95, wy, 0.75), (2.6, wy - 0.2, 0.75), (5.4, wy - 0.4, 0.35), (ax, ay + 1.1, 0.12)], (0.12, 0.12, 0.25))
    # loose hull plates on the seabed off the starboard side of the hold
    for x, y, yaw, tilt in ((-8.6, -11.0, 20, 4), (-9.4, -7.5, -35, 8), (-8.2, -3.5, 70, 3)):
        obox(bm, (1.5, 1.0, 0.04), Matrix.Translation((x, y, 0.06)) @ Matrix.Rotation(math.radians(yaw), 4, 'Z')
             @ Matrix.Rotation(math.radians(tilt), 4, 'X'))


def build_coal(bm):
    """Add a low mound in the hold: a heightfield over a flat base, closed into a solid."""
    (y0, y1), w = COAL_Y, COAL_HALF_W
    nx, ny = 14, 30
    base, z0 = 0.2, FRAME_H

    def top(u, v):
        win = math.cos(math.pi / 2 * u) ** 2 * math.cos(math.pi / 2 * v) ** 2
        bumps = 0.8 + 0.2 * math.sin(7.1 * u + 2.3) * math.sin(5.3 * v + 0.7)
        return z0 + COAL_H * win * bumps

    T, Bt = [], []
    for i in range(nx + 1):
        u = -1 + 2 * i / nx
        T.append([]); Bt.append([])
        for j in range(ny + 1):
            v = -1 + 2 * j / ny
            x, y = u * w, y0 + (y1 - y0) * (j / ny)
            T[i].append(bm.verts.new((x, y, top(u, v))))
            Bt[i].append(bm.verts.new((x, y, base)))
    for i in range(nx):
        for j in range(ny):
            bm.faces.new((T[i][j], T[i + 1][j], T[i + 1][j + 1], T[i][j + 1]))
            bm.faces.new((Bt[i][j], Bt[i][j + 1], Bt[i + 1][j + 1], Bt[i + 1][j]))
    ring = [(i, 0) for i in range(nx)] + [(nx, j) for j in range(ny)] + \
           [(i, ny) for i in range(nx, 0, -1)] + [(0, j) for j in range(ny, 0, -1)]
    for (i0, j0), (i1, j1) in zip(ring, ring[1:] + ring[:1]):
        bm.faces.new((T[i0][j0], Bt[i0][j0], Bt[i1][j1], T[i1][j1]))


def material(name, rgb, roughness, metallic=0.0):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
    bsdf.inputs["Metallic"].default_value = metallic   # glTF and Blender both default to 1.0
    bsdf.inputs["Roughness"].default_value = roughness
    return mat


def make_part(name, builder, mat):
    me = bpy.data.meshes.new(f"{name}_mesh")
    bm = bmesh.new()
    builder(bm)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    me.materials.append(mat)
    # UVs: the profile's Geometry section requires TEXCOORD_0
    bpy.context.view_layer.objects.active = ob
    ob.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.uv.smart_project(island_margin=0.002)
    bpy.ops.object.mode_set(mode='OBJECT')
    ob.select_set(False)
    return ob


def export(ob):
    bpy.ops.object.select_all(action='DESELECT')
    ob.select_set(True)
    bpy.context.view_layer.objects.active = ob
    path = f"{out}/meshes/{ob.name}_visual.glb"
    bpy.ops.export_scene.gltf(
        filepath=path, export_format='GLB', export_yup=True,
        use_selection=True, export_apply=True, export_texcoords=True, export_normals=True,
        export_materials='EXPORT', export_cameras=False, export_lights=False,
        export_draco_mesh_compression_enable=False, export_animations=False,
        will_save_settings=True, export_copyright="Copyright 2026 Honu Robotics")
    return path


# --- Acoustic classes, from the spec's Acoustic properties table ------------
ACOUSTIC = {   # lambert dB, specular dB or None, specular width rad
    "wood":   (-25, None, None),
    "iron":   (-15, -5, 0.10),
    "bronze": (-15, -3, 0.07),
    "paint":  (-20, -5, 0.05),
    "coal":   (-22, None, None),
}
PARTS = [      # name, builder, acoustic class, base color, roughness
    ("hull",       build_hull,       "wood",   (0.28, 0.24, 0.19), 0.90),
    ("mast",       build_mast,       "wood",   (0.30, 0.25, 0.19), 0.90),
    ("boiler",     build_boiler,     "iron",   (0.33, 0.19, 0.11), 0.85),
    ("engine",     build_engine,     "iron",   (0.30, 0.18, 0.11), 0.85),
    ("debris",     build_debris,     "iron",   (0.31, 0.19, 0.12), 0.85),
    ("propeller",  build_propeller,  "bronze", (0.26, 0.30, 0.21), 0.70),
    ("bell",       build_bell,       "bronze", (0.27, 0.31, 0.20), 0.65),
    ("name_plate", build_name_plate, "paint",  (0.45, 0.42, 0.35), 0.80),
    ("coal",       build_coal,       "coal",   (0.14, 0.13, 0.12), 0.95),
]


def gz(p):
    """Blender model frame to Gazebo model frame."""
    return (-p[1], p[0], p[2])


def fmt(v):
    return [round(float(c), 3) for c in v]


def write_sdf(parts):
    vis = []
    for name, _, cls, _, _ in parts:
        lam, spec, width = ACOUSTIC[cls]
        ac = f"        <gz:acoustic_lambert>{lam}</gz:acoustic_lambert>\n"
        if spec is not None:
            ac += (f"        <gz:acoustic_specular>{spec}</gz:acoustic_specular>\n"
                   f"        <gz:acoustic_specular_width>{width}</gz:acoustic_specular_width>\n")
        vis.append(f"""      <visual name="{name}">
        <pose>0 0 0 1.5708 0 1.5708</pose>
        <geometry><mesh><uri>model://shipwreck/meshes/{name}_visual.glb</uri></mesh></geometry>
{ac}      </visual>""")
    bx, by, bz = gz((0, BOILER_LEN / 2, BOILER_Z))
    ex, ey, ez = gz((0, sum(ENGINE_Y) / 2, 1.55))
    a0, a1 = y_of(STANDING_FROM), y_of(L)
    sx = -(a0 + a1) / 2
    col = f"""      <collision name="aft_section_collision">
        <pose>{sx:.3f} 0 1.15 0 0 0</pose>
        <geometry><box><size>{a1 - a0:.3f} 10.2 2.6</size></box></geometry>
      </collision>
      <collision name="boiler_collision">
        <pose>{bx:.3f} {by:.3f} {bz:.3f} 0 1.5708 0</pose>
        <geometry><cylinder><radius>{BOILER_R}</radius><length>{BOILER_LEN}</length></cylinder></geometry>
      </collision>
      <collision name="engine_collision">
        <pose>{ex:.3f} {ey:.3f} {ez:.3f} 0 0 0</pose>
        <geometry><box><size>2 2 2.5</size></box></geometry>
      </collision>"""
    sdf = f"""<?xml version="1.0"?>
<!--
  Shipwreck, modeled on the Monohansett. Generated by build_shipwreck.py; edit
  the script, not this file.

  The model frame is the wreck's datum: on the seabed plane, on the keel
  centerline, at the boiler's forward face, +X toward the bow. Place the model
  with z at the seabed; a negative offset buries it. Every part's .glb shares
  the model frame, so each visual carries only the profile's glTF-to-Gazebo
  rotation. Acoustic values are illustrative (spec, Acoustic properties).
  The meshes are the exact ground truth geometry; named features are in
  ground_truth/features.yaml, in this same frame.
-->
<sdf version="1.12" xmlns:gz="http://gazebosim.org/schema">
  <model name="shipwreck">
    <static>true</static>
    <link name="wreck">
{chr(10).join(vis)}
{col}
    </link>
  </model>
</sdf>
"""
    open(f"{out}/model.sdf", "w").write(sdf)


def write_features():
    hx = (-(y_of(DECK_SPAN[0]) + y_of(DECK_SPAN[1])) / 2)
    feats = [
        ("bow_stem", "hull", gz((0, y_of(0.5), 0.6)), "stem post, about 1.2 m proud"),
        ("stern_post", "hull", gz((0, y_of(L - 0.5), 1.3)), "stern post, about 2.6 m proud"),
        ("hull_break_forward", "hull", gz((0, y_of(14.0), 0.0)), "gap between bow and midship sections, 2 m"),
        ("hull_break_aft", "hull", gz((0, y_of(31.5), 0.0)), "gap between midship and aft sections, 1 m"),
        ("aft_hatch", "hull", gz((0, y_of(sum(HATCH[:2]) / 2), DECK_Z + DECK_T)), "opening in the aft deck, 2 x 2 m"),
        ("boiler", "boiler", gz((0, BOILER_LEN / 2, BOILER_Z)), "3.0 m diameter x 4.0 m, axis fore and aft"),
        ("engine", "engine", gz((0, sum(ENGINE_Y) / 2, 1.55)), "2 x 2 x 2.9 m, two cylinders"),
        ("propeller", "propeller", gz((0, PROP_Y, SHAFT_Z)), "2.4 m diameter, four blades"),
        ("bell", "bell", gz((BELL_AT[0] + 0.23, BELL_AT[1], BELL_AT[2] + 0.25)), "0.5 m diameter, on its side on the aft deck"),
        ("name_plate", "name_plate", gz(PLATE_AT), "2.0 x 0.3 m board leaning on the starboard side"),
        ("mast", "mast", gz(((MAST_FROM[0] + MAST_TO[0]) / 2, (MAST_FROM[1] + MAST_TO[1]) / 2, MAST_R)), "15 m, fallen off the port side"),
        ("anchor", "debris", gz((5.5, -30.5, 0.1)), "off the port bow"),
        ("windlass", "debris", gz((0, -29.0, 0.75)), "on the bow floors"),
        ("coal_mound", "coal", gz((0, sum(COAL_Y) / 2, FRAME_H)), "low mound in the hold, about 0.45 m"),
    ]
    with open(f"{out}/ground_truth/features.yaml", "w") as f:
        f.write("# Named features of the shipwreck. Generated by build_shipwreck.py; edit the script.\n"
                "#\n"
                "# Frame: the shipwreck model frame, which is also every part's .glb origin.\n"
                "#   origin  on the seabed plane, on the keel centerline, at the boiler's forward face\n"
                "#   +X      toward the bow      +Y  to port      +Z  up\n"
                "#   z = 0   the seabed plane; geometry below it is buried\n"
                "# A feature's world position is the model's <pose> applied to its position here.\n"
                "# Positions are in meters. The meshes in meshes/ are the exact ground-truth geometry.\n"
                "frame: shipwreck model frame\nfeatures:\n")
        for name, part, pos, note in feats:
            f.write(f"  - name: {name}\n    part: {part}\n    position: {fmt(pos)}\n    note: \"{note}\"\n")
    return len(feats)


def write_ground_truth(objs):
    """Area-weighted random samples on every surface above the seabed plane, Gazebo frame."""
    rng = random.Random(GT_SEED)
    density = 1.0 / GT_SPACING ** 2
    pts = []
    for k, ob in enumerate(objs):
        bm = bmesh.new()
        bm.from_mesh(ob.data)
        bmesh.ops.triangulate(bm, faces=bm.faces)
        for f in bm.faces:
            a, b, c = (v.co.copy() for v in f.verts)
            if max(a.z, b.z, c.z) < 0.0:
                continue                        # wholly buried
            if f.normal.z < -0.7 and (a.z + b.z + c.z) / 3 < 0.4:
                continue                        # underside resting on the seabed
            n = f.calc_area() * density
            n = int(n) + (1 if rng.random() < n - int(n) else 0)
            for _ in range(n):
                u, v = rng.random(), rng.random()
                if u + v > 1:
                    u, v = 1 - u, 1 - v
                p = a + (b - a) * u + (c - a) * v
                if p.z >= 0.0:
                    pts.append((*gz(p), k))
        bm.free()
    path = f"{build}/shipwreck_points.ply"   # optional, not part of the model
    with open(path, "wb") as f:
        names = " ".join(ob.name for ob in objs)
        f.write((f"ply\nformat binary_little_endian 1.0\n"
                 f"comment shipwreck ground truth, Gazebo model frame, spacing {GT_SPACING} m, seed {GT_SEED}\n"
                 f"comment part index: {names}\n"
                 f"element vertex {len(pts)}\nproperty float x\nproperty float y\nproperty float z\n"
                 f"property uchar part\nend_header\n").encode())
        pack = struct.Struct("<fffB").pack
        f.write(b"".join(pack(*p) for p in pts))
    return len(pts)


bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.context.scene.unit_settings.scale_length = 1.0

objs = [make_part(name, fn, material(f"{name}_{cls}", rgb, rough)) for name, fn, cls, rgb, rough in PARTS]
total = 0
for ob in objs:
    path = export(ob)
    me = ob.data
    tris = sum(len(p.vertices) - 2 for p in me.polygons)
    total += tris
    lo = [min(v.co[i] for v in me.vertices) for i in range(3)]
    hi = [max(v.co[i] for v in me.vertices) for i in range(3)]
    print(f"=== {ob.name:<11} {tris:6d} tris  bbox x {lo[0]:7.2f}..{hi[0]:6.2f}  "
          f"y {lo[1]:7.2f}..{hi[1]:6.2f}  z {lo[2]:5.2f}..{hi[2]:5.2f}")
print(f"=== total {total} tris in {len(objs)} parts")
write_sdf(PARTS)
print(f"=== features {write_features()}, ground-truth points {write_ground_truth(objs)}")
bpy.context.preferences.filepaths.save_version = 0   # no .blend1 backup
bpy.ops.wm.save_as_mainfile(filepath=f"{build}/shipwreck.blend")
