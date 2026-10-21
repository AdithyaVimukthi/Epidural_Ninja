"""
Extract the right arm's needle tool (load-cell bracket, bar load cell, syringe
clamp, syringe) from the CAD of the whole robot,
new_setup&files/arm_with loadcell.step, into what make_needle_urdf.py needs:

    sim_files/udrf/needle_tool/right_needle_tool.stl   surface mesh, in the hand frame, metres
    sim_files/udrf/needle_tool/needle_tool_parts.json  per part: volume, centre of
                                                       mass and unit-density inertia
                                                       (hand frame), plus reference
                                                       points (syringe nozzle)

Run only when the CAD changes -- the outputs are kept in the repo, and the
URDF is made from them without CAD software. Needs gmsh (OpenCASCADE), which
is NOT in the project venv; e.g. into a scratch prefix:

    ../../.venv/bin/pip install --prefix /tmp/cad --no-deps --ignore-installed gmsh
    LD_LIBRARY_PATH=/tmp/cad/lib PYTHONPATH=/tmp/cad/lib/python3.10/site-packages \\
        ../../.venv/bin/python tools/extract_needle_tool.py

How the tool is found and placed (2026-10-12):
  * The STEP is the whole dual-arm robot at the zero pose (arms hanging down),
    in a frame aligned with the URDF's robot_base. Registering the URDF's own
    link7 / hand meshes against the CAD solids gives CAD -> URDF world =
    +30.0 mm x, -1.0 mm y, +1.2 mm z (no rotation) on the right arm.
  * The tool is the right-arm solids with no mirror twin on the left arm
    (which still carries the gripper): 8 parts, PARTS below.
  * Everything is then expressed in the URDF's openarm_right_hand_respondable
    frame (the old gripper's mount on link7), which the new tool link reuses.
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.normpath(os.path.join(HERE, "..", ".."))
STEP = os.path.join(PROJECT, "new_setup&files", "arm_with loadcell.step")
OUT_DIR = os.path.join(PROJECT, "sim_files", "udrf", "needle_tool")

# The tool's parts: (name, material, centre of mass in the CAD frame mm,
# volume cm3). Found by position and volume, not by solid number, which
# changes with shape healing and with any re-export of the CAD.
PARTS = [
    ("flange_bracket", "pla", (0.0, -165.4, 168.0), 40.84),
    ("syringe_clamp", "pla", (-0.2, -70.3, 145.9), 24.31),
    ("load_cell", "aluminium", (0.0, -113.1, 146.1), 9.72),
    ("gauge_cover_top", "epoxy", (0.0, -113.1, 153.2), 0.33),
    ("gauge_cover_bottom", "epoxy", (0.0, -113.1, 139.1), 0.33),
    ("clamp_bolt", "steel", (-7.3, -130.0, 146.4), 0.44),
    ("syringe_barrel", "polypropylene", (-0.3, -59.1, 161.4), 6.42),
    ("syringe_plunger", "polypropylene", (-0.3, -59.1, 177.8), 4.55),
]
CAD_TO_WORLD_MM = np.array([30.0, -1.0, 1.2])
# openarm_right_hand_respondable at the zero pose (pinocchio FK of
# sim_files/udrf/openarm_dual.urdf): origin, and its axes as world columns.
HAND_ORIGIN_MM = np.array([30.0, -153.5, 162.1])
HAND_R = np.array([[0.0, 0.0, 1.0],     # columns: hand x = world +z,
                   [0.0, -1.0, 0.0],    #          hand y = world -y,
                   [1.0, 0.0, 0.0]])    #          hand z = world +x


def to_hand(p_cad_m: np.ndarray) -> np.ndarray:
    world_mm = np.asarray(p_cad_m) * 1000 + CAD_TO_WORLD_MM
    return HAND_R.T @ (world_mm - HAND_ORIGIN_MM) / 1000


def main() -> None:
    import gmsh
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.option.setString("Geometry.OCCTargetUnit", "M")
    # Heal the imported shapes, or meshing trips over tiny gaps between edges.
    for opt in ("OCCFixDegenerated", "OCCFixSmallEdges", "OCCFixSmallFaces"):   # (OCCSewFaces turns solids into shells)
        gmsh.option.setNumber(f"Geometry.{opt}", 1)
    gmsh.model.occ.importShapes(STEP)
    gmsh.model.occ.synchronize()

    solids = [(t, gmsh.model.occ.getMass(3, t), np.array(gmsh.model.occ.getCenterOfMass(3, t)) * 1000)
              for _, t in gmsh.model.getEntities(3)]
    found = {}
    for name, material, com_mm, vol_cm3 in PARTS:
        match = [t for t, v, c in solids
                 if np.linalg.norm(c - com_mm) < 2.0 and abs(v * 1e6 - vol_cm3) < 0.05 * vol_cm3 + 0.01]
        if len(match) != 1:
            sys.exit(f"{name}: {len(match)} CAD solids match (want 1) -- has the CAD changed?")
        found[name] = (match[0], material)

    parts, cad_bounds = [], {}
    for name, (tag, material) in found.items():
        vol = gmsh.model.occ.getMass(3, tag)
        com = np.array(gmsh.model.occ.getCenterOfMass(3, tag))
        # OpenCASCADE: inertia of the solid at unit density about its centre of
        # mass, in the CAD axes (m^5).
        I_cad = np.array(gmsh.model.occ.getMatrixOfInertia(3, tag)).reshape(3, 3)
        bb = np.array(gmsh.model.getBoundingBox(3, tag))
        cad_bounds[name] = bb
        parts.append({
            "name": name, "material": material, "cad_solid": tag,
            "volume_m3": vol,
            "com_hand_m": to_hand(com).tolist(),
            # hand = R^T world, and the CAD axes are the world axes
            "inertia_unit_density_hand": (HAND_R.T @ I_cad @ HAND_R).tolist(),
        })

    # Syringe axis (barrel and plunger are centred on it) and its lowest point:
    # the nozzle end, where the needle hub goes on.
    b, pl = cad_bounds["syringe_barrel"], cad_bounds["syringe_plunger"]
    nozzle_cad = np.array([(pl[0] + pl[3]) / 2, (pl[1] + pl[4]) / 2, b[2]])
    lc = cad_bounds["load_cell"]
    refs = {
        "syringe_nozzle_end_hand_m": to_hand(nozzle_cad).tolist(),
        "syringe_axis_hand": [-1.0, 0.0, 0.0],   # pointing down the needle at the zero pose
        "load_cell_bottom_face_hand_x_m": float(to_hand(np.array([0, 0, lc[2]]))[0]),
        "cad_to_world_mm": CAD_TO_WORLD_MM.tolist(),
        "source": os.path.relpath(STEP, PROJECT),
    }

    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, "needle_tool_parts.json"), "w") as f:
        json.dump({"frame": "openarm_right_hand_respondable", "parts": parts, "refs": refs}, f, indent=1)
    for p in parts:
        print(f"  {p['name']:<20} {p['volume_m3'] * 1e6:7.2f} cm3  com (hand, mm) "
              f"{np.round(np.array(p['com_hand_m']) * 1000, 1)}")
    print(f"  syringe nozzle end (hand, mm): {np.round(np.array(refs['syringe_nozzle_end_hand_m']) * 1000, 1)}")

    # Visual mesh: each part on its own (one bad CAD face shouldn't lose the
    # rest), trying a few algorithms, then all triangles moved into the hand
    # frame and written as one binary STL.
    keep = {tag: name for name, (tag, _) in found.items()}
    gmsh.model.occ.remove([(3, t) for _, t in gmsh.model.getEntities(3) if t not in keep], recursive=True)
    gmsh.model.occ.synchronize()
    gmsh.option.setNumber("Mesh.MeshSizeMax", 0.004)
    gmsh.option.setNumber("Mesh.MeshSizeMin", 0.0005)
    gmsh.option.setNumber("Mesh.MeshOnlyVisible", 1)
    tris = []
    for tag, name in keep.items():
        faces = [t for _, t in gmsh.model.getBoundary([(3, tag)], oriented=False, combined=False)]
        got = None
        for algo in (6, 1, 5):
            gmsh.model.mesh.clear()
            gmsh.model.setVisibility(gmsh.model.getEntities(), 0, recursive=True)
            gmsh.model.setVisibility([(3, tag)], 1, recursive=True)
            gmsh.option.setNumber("Mesh.Algorithm", algo)
            try:
                gmsh.model.mesh.generate(2)
            except Exception:
                continue
            node_tags, coords, _ = gmsh.model.mesh.getNodes()
            xyz = dict(zip(node_tags, np.asarray(coords).reshape(-1, 3)))
            got = []
            for face in faces:
                for etype, _, enodes in zip(*gmsh.model.mesh.getElements(2, face)):
                    if etype == 2:     # 3-node triangles
                        got += [[xyz[n] for n in tri] for tri in np.asarray(enodes).reshape(-1, 3)]
            break
        if not got:
            print(f"  WARNING {name}: no algorithm could mesh it -- left out of the STL (mass properties kept)")
            continue
        tris += got
        print(f"  meshed {name}: {len(got)} triangles")
    gmsh.finalize()
    T = np.asarray(tris)                                  # (n, 3 vertices, xyz) in CAD metres
    T = np.einsum("ij,nkj->nki", HAND_R.T, T * 1000 + CAD_TO_WORLD_MM - HAND_ORIGIN_MM) / 1000
    stl = os.path.join(OUT_DIR, "right_needle_tool.stl")
    write_stl(stl, T)
    print(f"wrote {stl} ({len(T)} triangles) and needle_tool_parts.json ({len(parts)} parts)")


def write_stl(path: str, tris: np.ndarray) -> None:
    import struct
    with open(path, "wb") as f:
        f.write(b"OpenArm right needle tool, hand frame, metres".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            n = np.cross(b - a, c - a)
            n = n / (np.linalg.norm(n) or 1.0)
            f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))


if __name__ == "__main__":
    sys.exit(main())
