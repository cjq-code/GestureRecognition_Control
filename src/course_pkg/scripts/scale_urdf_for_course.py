#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Uniform scale TurtleBot3 + OpenMANIPULATOR URDF for course demos (~1 m tall platform).
Reads URDF XML from stdin, writes scaled URDF to stdout.
Scale factor s applies to lengths, prismatic limits, diff-drive wheel params, laser range.
Mass ~ s^3, inertia ~ s^5 (uniform-density approximation).
"""
import sys
import xml.etree.ElementTree as ET


def _scale_xyz(attrib: dict, key: str, s: float) -> None:
    if key not in attrib:
        return
    parts = attrib[key].split()
    attrib[key] = " ".join(str(float(p) * s) for p in parts)


def _scale_float_attrib(attrib: dict, key: str, s: float, power: float = 1.0) -> None:
    if key not in attrib:
        return
    attrib[key] = str(float(attrib[key]) * (s ** power))


def scale_urdf_tree(root: ET.Element, s: float, control_period: float) -> None:

    for el in root.iter():
        tag = el.tag.split("}")[-1]  # strip namespace if any

        if tag == "origin":
            _scale_xyz(el.attrib, "xyz", s)
            # rpy unchanged (angles)

        elif tag == "box":
            _scale_xyz(el.attrib, "size", s)

        elif tag == "cylinder":
            _scale_float_attrib(el.attrib, "radius", s)
            _scale_float_attrib(el.attrib, "length", s)

        elif tag == "sphere":
            _scale_float_attrib(el.attrib, "radius", s)

        elif tag == "mesh":
            if "scale" in el.attrib:
                _scale_xyz(el.attrib, "scale", s)
            else:
                el.attrib["scale"] = f"{s} {s} {s}"

        elif tag == "mass":
            _scale_float_attrib(el.attrib, "value", s, power=3.0)

        elif tag == "inertia":
            for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"):
                _scale_float_attrib(el.attrib, k, s, power=5.0)

        elif tag in ("wheelSeparation", "wheelDiameter"):
            if el.text and el.text.strip():
                try:
                    el.text = str(float(el.text.strip()) * s)
                except ValueError:
                    pass

        elif tag == "hackBaseline" and el.text and el.text.strip():
            try:
                el.text = str(float(el.text.strip()) * s)
            except ValueError:
                pass

        elif tag == "controlPeriod":
            el.text = str(control_period)

    # Joint limits. Uniform scaling increases gravity torque demand; without
    # scaling effort limits, effort controllers saturate and the arm goes limp.
    for joint in root.iter():
        jtag = joint.tag.split("}")[-1]
        if jtag != "joint":
            continue
        lim = None
        for child in joint:
            if child.tag.split("}")[-1] == "limit":
                lim = child
                break
        if lim is None:
            continue

        joint_type = joint.get("type")
        if joint_type == "prismatic":
            _scale_float_attrib(lim.attrib, "lower", s)
            _scale_float_attrib(lim.attrib, "upper", s)
            _scale_float_attrib(lim.attrib, "effort", s, power=3.0)
        elif joint_type in ("revolute", "continuous"):
            _scale_float_attrib(lim.attrib, "effort", s, power=4.0)


def main():
    if len(sys.argv) < 2:
        print(
            "usage: scale_urdf_for_course.py SCALE [CONTROL_PERIOD] < in.urdf > out.urdf",
            file=sys.stderr,
        )
        sys.exit(1)
    s = float(sys.argv[1])
    control_period = float(sys.argv[2]) if len(sys.argv) >= 3 else 0.002
    if s <= 0:
        print("SCALE must be positive", file=sys.stderr)
        sys.exit(1)
    if control_period <= 0:
        print("CONTROL_PERIOD must be positive", file=sys.stderr)
        sys.exit(1)

    text = sys.stdin.read()
    root = ET.fromstring(text)
    scale_urdf_tree(root, s, control_period)
    out = ET.tostring(root, encoding="unicode")
    # ElementTree drops XML declaration; Gazebo is fine without it
    sys.stdout.write(out)


if __name__ == "__main__":
    main()
