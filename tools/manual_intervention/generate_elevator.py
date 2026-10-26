"""Generate a self-contained, textured two-floor manual elevator test world."""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def box(parent, name, xyz, size, color, texture=False):
    link = ET.SubElement(parent, "link", name=name)
    ET.SubElement(link, "pose").text = " ".join(map(str, xyz)) + " 0 0 0"
    for tag in ("collision", "visual"):
        part = ET.SubElement(link, tag, name=name + "_" + tag)
        geometry = ET.SubElement(part, "geometry")
        ET.SubElement(ET.SubElement(geometry, "box"), "size").text = " ".join(
            map(str, size)
        )
        if tag == "visual":
            mat = ET.SubElement(part, "material")
            ET.SubElement(mat, "ambient").text = color
            ET.SubElement(mat, "diffuse").text = color
            if texture:
                metal = ET.SubElement(ET.SubElement(mat, "pbr"), "metal")
                ET.SubElement(
                    metal, "albedo_map"
                ).text = "model://textures/STAIRS_Albedo.png"


def model(world, name, pose=(0, 0, 0)):
    m = ET.SubElement(world, "model", name=name)
    ET.SubElement(m, "static").text = "true"
    ET.SubElement(m, "pose").text = " ".join(map(str, pose)) + " 0 0 0"
    return m


def generate(path):
    root = ET.Element("sdf", version="1.9")
    w = ET.SubElement(root, "world", name="elevator")
    physics = ET.SubElement(w, "physics", name="1ms", type="ignored")
    ET.SubElement(physics, "max_step_size").text = ".001"
    ET.SubElement(physics, "real_time_factor").text = "1"
    for plugin, system in [
        ("physics", "Physics"),
        ("user-commands", "UserCommands"),
        ("scene-broadcaster", "SceneBroadcaster"),
        ("imu", "Imu"),
        ("sensors", "Sensors"),
    ]:
        p = ET.SubElement(
            w,
            "plugin",
            filename=f"ignition-gazebo-{plugin}-system",
            name=f"ignition::gazebo::systems::{system}",
        )
        if system == "Sensors":
            ET.SubElement(p, "render_engine").text = "ogre2"
    scene = ET.SubElement(w, "scene")
    ET.SubElement(scene, "ambient").text = "1 1 1 1"
    ET.SubElement(scene, "background").text = ".15 .2 .3 1"
    ET.SubElement(scene, "shadows").text = "false"
    light = ET.SubElement(w, "light", name="sun", type="directional")
    ET.SubElement(light, "pose").text = "0 0 12 0 0 0"
    ET.SubElement(light, "diffuse").text = "1 1 1 1"
    ET.SubElement(light, "direction").text = "-.3 .2 -1"
    ET.SubElement(light, "cast_shadows").text = "false"
    for f, color in [(0, ".25 .50 .85 1"), (1, ".85 .45 .20 1")]:
        z = f * 3.0
        m = model(w, f"floor_{f}")
        box(m, "floor", (-1, 0, z - 0.10), (8, 6, 0.20), ".7 .7 .7 1", True)
        box(m, "back", (-5.1, 0, z + 1.4), (0.2, 6.2, 2.8), color, True)
        for y in (-3.1, 3.1):
            box(m, "side_" + str(y), (-1, y, z + 1.4), (8.4, 0.2, 2.8), color, True)
        for y in (-2.0, 2.0):
            box(m, "entrance_" + str(y), (3, y, z + 1.4), (0.2, 2.2, 2.8), color, True)
        box(m, "lintel", (3, 0, z + 2.65), (0.2, 1.8, 0.3), color)
        # Different asymmetric landmarks on each floor provide visual anchors.
        for i in range(6):
            x = -4 + i * 1.05
            y = 2.5 if i % 2 else -2.5
            box(
                m,
                f"landmark_{i}",
                (x, y, z + 0.25 + 0.08 * i),
                (0.4, 0.4, 0.5 + 0.16 * i),
                color if i % 2 else ".8 .8 .2 1",
                True,
            )
        box(m, "ceiling", (-1, 0, z + 2.9), (8, 6, 0.15), ".6 .6 .6 1", True)
    cabin = model(w, "elevator_cabin", (4.5, 0, 0))
    box(cabin, "floor", (0, 0, -0.1), (3, 2, 0.2), ".5 .5 .5 1", True)
    box(cabin, "back", (1.6, 0, 1.3), (0.2, 2.2, 2.8), ".65 .65 .7 1", True)
    for y in (-1.1, 1.1):
        box(cabin, "wall_" + str(y), (0, y, 1.3), (3.2, 0.2, 2.8), ".65 .65 .7 1", True)
    box(cabin, "roof", (0, 0, 2.75), (3, 2, 0.15), ".8 .8 .8 1")
    for i in range(4):
        box(
            cabin,
            f"marker_{i}",
            (1.48, -0.7 + i * 0.45, 0.8 + i * 0.2),
            (0.03, 0.14, 0.14),
            ".1 .1 .1 1",
        )
    for side, direction in [("left", 1), ("right", -1)]:
        door = model(w, "cabin_" + side, (3.08, direction * 0.45, 0))
        box(door, "panel", (0, 0, 1.2), (0.07, 0.9, 2.4), ".6 .6 .65 1", True)
        for f in (0, 1):
            door = model(w, f"landing_{f}_{side}", (3, direction * 0.45, f * 3))
            box(door, "panel", (0, 0, 1.2), (0.07, 0.9, 2.4), ".4 .45 .55 1", True)
    ET.indent(root)
    Path(path).write_text(
        '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode") + "\n"
    )


if __name__ == "__main__":
    generate(
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[2] / "gazebo/worlds/elevator.sdf"
    )
