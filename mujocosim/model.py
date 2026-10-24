"""Compose a prepared scene, a robot builder and an optional sensor rig."""
from pathlib import Path
import os
import tempfile
import xml.etree.ElementTree as ET

from .config import PhysicsConfig, SensorConfig


def build_model_xml(scene_dir, robot, sensors: SensorConfig | None, physics: PhysicsConfig, *, return_text=False):
    scene_dir = Path(scene_dir).resolve()
    root = ET.Element("mujoco", model="mujocosim")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true", meshdir=".", texturedir=".")
    ET.SubElement(root, "option", timestep=str(physics.timestep), gravity="0 0 -9.81",
                  integrator="implicitfast", cone="elliptic", iterations="50")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth=str(sensors.width if sensors else 640), offheight=str(sensors.height if sensors else 480))
    ET.SubElement(visual, "map", znear="0.001", zfar="5")
    ET.SubElement(visual, "headlight", ambient="0.6 0.6 0.6", diffuse="0.6 0.6 0.6", specular="0 0 0")
    ET.SubElement(root, "statistic", extent="10", meansize="0.1")
    defaults = ET.SubElement(root, "default")
    ET.SubElement(defaults, "geom", solref="0.01 1", solimp="0.95 0.99 0.001", margin="0.001")
    scene = ET.parse(scene_dir / "scene.xml").getroot()
    root.append(scene.find("asset"))
    root.append(scene.find("worldbody"))
    body = robot.build(root)
    if sensors is not None:
        from .sensors.rig import attach_rig
        attach_rig(body, sensors)
    ET.indent(root)
    path = scene_dir / "world.xml"
    xml = ET.tostring(root, encoding="unicode", xml_declaration=True)
    # Runtime assembly is pure: concurrent worlds must not contend on a shared
    # export file (Windows can reject simultaneous atomic replacements).
    if return_text:
        return xml
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=scene_dir,
                                         prefix=".world-", suffix=".xml", delete=False) as f:
            temporary = Path(f.name)
            f.write(xml)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return path

