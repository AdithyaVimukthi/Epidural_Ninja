"""
Make the URDF with the needle tool on the right arm:

    sim_files/udrf/openarm_dual.urdf  (gripper on both arms, never changed)
      -> sim_files/udrf/openarm_dual_needle.urdf

Right arm only. The left arm keeps its gripper, as on the robot. What changes:

  removed   openarm_right_right/left_finger_respondable and their prismatic
            finger joints; openarm_right_hand_tcp_visual (the gripper's TCP)
  replaced  openarm_right_hand_respondable -> openarm_right_needle_tool, on
            the same mount (right_openarm_needle_tool_joint, the old hand
            joint's origin): load-cell bracket, bar load cell, syringe clamp,
            syringe, needle. Inertial from the CAD (tools/extract_needle_tool.py)
            and the materials below; visual / collision = the CAD mesh.
  added     openarm_right_needle_hub  where the needle goes on (syringe nozzle end)
            openarm_right_needle_tip  --needle-mm further along the needle
            Both with the tool link's axes: -X points down the needle, as the
            gripper TCP's did, so kinematics.TOOL_AXIS is unchanged. The
            kinematics pick openarm_{side}_needle_tip as the TCP whenever the
            URDF has it.

The needle axis is parallel to the wrist (link7) axis but ~93 mm to the side
of it -- the bar load cell carries the syringe off-axis.

    python3 tools/make_needle_urdf.py --needle-mm 38            # from src/
    python3 tools/make_needle_urdf.py --needle-mm 38 --tool-mass-g 112 --fluid-ml 2

--needle-mm   how far the needle point is below the syringe's nozzle end (the
              lowest point of the bare syringe), along the needle. With the
              needle fitted: (needle point to the bottom of the barrel's
              nozzle) -- equivalently, for a standard hub, roughly the
              needle's nominal length plus ~7 mm. MEASURE IT.
--tool-mass-g weigh the tool (everything below the link7 flange) and pass it:
              the part densities are scaled to match, keeping where the mass sits.
--fluid-ml    liquid in the syringe (water density), added along the barrel.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.normpath(os.path.join(HERE, "..", ".."))
UDRF = os.path.join(PROJECT, "sim_files", "udrf")
BASE_URDF = os.path.join(UDRF, "openarm_dual.urdf")
OUT_URDF = os.path.join(UDRF, "openarm_dual_needle.urdf")
TOOL_DIR = os.path.join(UDRF, "needle_tool")
PARTS_JSON = os.path.join(TOOL_DIR, "needle_tool_parts.json")
TOOL_STL = os.path.join(TOOL_DIR, "right_needle_tool.stl")

# kg/m^3. "pla" is a printed part's effective density (walls + infill), not
# solid PLA (1240) -- --tool-mass-g corrects all of these at once.
DENSITY = {"pla": 1000.0, "aluminium": 2700.0, "steel": 7850.0, "polypropylene": 900.0, "epoxy": 1200.0}
NEEDLE_MASS_KG = 0.002          # steel needle + plastic hub
SIDE = "right"


def fmt(v) -> str:
    return " ".join(f"{float(x):.6f}" for x in v)


def tool_inertia(parts, densities, scale, extras=()):
    """Total mass, centre of mass and inertia about it (tool link frame), of
    the CAD parts at densities x scale plus `extras` -- (mass, com, inertia
    about its own com) for what the CAD doesn't have (needle, liquid)."""
    items = [(densities[q["material"]] * scale * q["volume_m3"], np.array(q["com_hand_m"]),
              densities[q["material"]] * scale * np.array(q["inertia_unit_density_hand"])) for q in parts]
    items += list(extras)
    total = sum(m for m, _, _ in items)
    com = sum(m * c for m, c, _ in items) / total
    inertia = np.zeros((3, 3))
    for m, c, Ic in items:
        d = c - com
        inertia += Ic + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return total, com, inertia


def rod(mass, center, length, radius):
    """(mass, com, inertia) of a solid cylinder along the tool's x axis."""
    ax = 0.5 * mass * radius ** 2
    tr = mass * (3 * radius ** 2 + length ** 2) / 12
    return mass, np.asarray(center, dtype=float), np.diag([ax, tr, tr])


def el(parent, tag, **attrib):
    e = ET.SubElement(parent, tag)
    for k, v in attrib.items():
        e.set(k, v)
    return e


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--needle-mm", type=float, required=True,
                   help="needle point below the syringe nozzle end, along the needle (measure it)")
    p.add_argument("--tool-mass-g", type=float, default=None,
                   help="weighed mass of everything below the link7 flange; scales the densities")
    p.add_argument("--fluid-ml", type=float, default=0.0, help="liquid in the syringe")
    p.add_argument("--out", default=OUT_URDF)
    args = p.parse_args()

    with open(PARTS_JSON) as f:
        tool = json.load(f)
    parts, refs = tool["parts"], tool["refs"]
    hub = np.array(refs["syringe_nozzle_end_hand_m"])
    needle = args.needle_mm / 1000
    tip = hub + np.array([-needle, 0.0, 0.0])

    extras = [rod(NEEDLE_MASS_KG, hub + np.array([-needle / 2, 0.0, 0.0]), max(needle, 1e-3), 0.002)]
    if args.fluid_ml > 0:
        # a water column from just above the nozzle up the barrel (bore ~9.5 mm radius)
        r = 0.0095
        h = args.fluid_ml * 1e-6 / (math.pi * r * r)
        extras.append(rod(args.fluid_ml * 1e-3, hub + np.array([0.008 + h / 2, 0.0, 0.0]), h, r))
    scale = 1.0
    if args.tool_mass_g:
        cad = sum(DENSITY[q["material"]] * q["volume_m3"] for q in parts)
        # the weighed mass includes the needle and any liquid
        scale = (args.tool_mass_g / 1000 - NEEDLE_MASS_KG - args.fluid_ml * 1e-3) / cad
    mass, com, I = tool_inertia(parts, DENSITY, scale, extras)

    tree = ET.parse(BASE_URDF)
    robot = tree.getroot()
    links = {l.get("name"): l for l in robot.findall("link")}
    joints = {j.get("name"): j for j in robot.findall("joint")}
    hand = f"openarm_{SIDE}_hand_respondable"
    tool_link = f"openarm_{SIDE}_needle_tool"

    # Off: the fingers and the gripper TCP.
    for j in (f"openarm_{SIDE}_finger_joint1", f"openarm_{SIDE}_finger_joint2", f"openarm_{SIDE}_hand_tcp_joint"):
        robot.remove(joints[j])
    for l in (f"openarm_{SIDE}_right_finger_respondable", f"openarm_{SIDE}_left_finger_respondable",
              f"openarm_{SIDE}_hand_tcp_visual"):
        robot.remove(links[l])

    # The hand joint now carries the tool.
    hj = joints[f"{SIDE}_openarm_hand_joint"]
    hj.set("name", f"{SIDE}_openarm_needle_tool_joint")
    hj.find("child").set("link", tool_link)

    # The tool link replaces the hand link, in place.
    old = links[hand]
    idx = list(robot).index(old)
    robot.remove(old)
    L = ET.Element("link", {"name": tool_link})
    inert = el(L, "inertial")
    el(inert, "origin", rpy="0 0 0", xyz=fmt(com))
    el(inert, "mass", value=f"{mass:.6f}")
    el(inert, "inertia", ixx=f"{I[0, 0]:.4e}", ixy=f"{I[0, 1]:.4e}", ixz=f"{I[0, 2]:.4e}",
       iyy=f"{I[1, 1]:.4e}", iyz=f"{I[1, 2]:.4e}", izz=f"{I[2, 2]:.4e}")
    mesh_uri = "file://" + TOOL_STL
    for kind in ("collision", "visual"):
        e = el(L, kind, name=f"{tool_link}_{kind}")
        el(e, "origin", rpy="0 0 0", xyz="0 0 0")
        el(el(e, "geometry"), "mesh", filename=mesh_uri)
        if kind == "visual":
            el(el(e, "material", name=f"{tool_link}_material"), "color", rgba="0.85 0.85 0.80 1.0")
    # The plunger (its CAD face wouldn't mesh) and the needle, as cylinders
    # along the tool's x axis.
    plunger = next(q for q in parts if q["name"] == "syringe_plunger")
    for name, center, length, radius, rgba in (
            ("syringe_plunger", plunger["com_hand_m"], 0.102, 0.010, "0.95 0.95 0.95 0.6"),
            ("needle", hub + np.array([-needle / 2, 0, 0]), max(needle, 1e-4), 0.0004, "0.6 0.6 0.65 1.0")):
        e = el(L, "visual", name=f"{tool_link}_{name}")
        el(e, "origin", rpy=f"0 {math.pi / 2:.6f} 0", xyz=fmt(center))
        el(el(e, "geometry"), "cylinder", radius=f"{radius:.6f}", length=f"{length:.6f}")
        el(el(e, "material", name=f"{tool_link}_{name}_material"), "color", rgba=rgba)
    robot.insert(idx, L)

    # Frames: needle hub and tip, axes as the tool link (-X = down the needle).
    for name, xyz in (("needle_hub", hub), ("needle_tip", tip)):
        link = f"openarm_{SIDE}_{name}"
        j = ET.Element("joint", {"name": f"{link}_joint", "type": "fixed"})
        el(j, "parent", link=tool_link)
        el(j, "child", link=link)
        el(j, "origin", rpy="0 0 0", xyz=fmt(xyz))
        lk = ET.Element("link", {"name": link})
        e = el(lk, "visual", name=f"{link}_visual")
        el(e, "origin", rpy="0 0 0", xyz="0 0 0")
        el(el(e, "geometry"), "sphere", radius="0.0010")
        el(el(e, "material", name=f"{link}_material"), "color",
           rgba="1 0.2 0.2 1" if name == "needle_tip" else "0.2 0.6 1 1")
        idx += 1
        robot.insert(idx, j)
        idx += 1
        robot.insert(idx, lk)

    note = (f" Made by src/tools/make_needle_urdf.py from openarm_dual.urdf: right gripper replaced by the "
            f"needle tool. needle {args.needle_mm:g} mm below the syringe nozzle; tool mass {mass * 1000:.1f} g "
            f"({'weighed ' + format(args.tool_mass_g, 'g') + ' g' if args.tool_mass_g else 'estimated from the CAD and materials'})"
            f"{f', {args.fluid_ml:g} ml liquid' if args.fluid_ml else ''}. Regenerate rather than edit. ")
    robot.insert(0, ET.Comment(note))
    ET.indent(tree, space="    ")
    tree.write(args.out, xml_declaration=True, encoding="utf-8")

    old_mass = 0.35 + 2 * 0.036025453432771
    print(f"wrote {args.out}")
    print(f"  tool: {mass * 1000:.1f} g (the gripper it replaces: {old_mass * 1000:.0f} g), centre of mass "
          f"{np.round(com * 1000, 1)} mm in the tool frame"
          + (f"; densities scaled x{scale:.2f} to the weighed {args.tool_mass_g:g} g" if args.tool_mass_g else ""))
    print(f"  needle hub {np.round(hub * 1000, 1)} mm, tip {np.round(tip * 1000, 1)} mm (tool frame; "
          f"the old gripper TCP was at [-80, 0, 0])")


if __name__ == "__main__":
    main()
