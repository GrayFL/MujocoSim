"""Configurable mounting of the demo lidar/RGBD rig on a robot body."""
import xml.etree.ElementTree as ET

def attach_rig(body, sensors):
    ET.SubElement(body, "geom", name="mast", type="cylinder", pos="-0.16 0 0.63", size="0.018 0.33",
                  mass="0.25", rgba="0.15 0.18 0.2 1")
    ET.SubElement(body, "geom", name="camera_mount", type="capsule", fromto="-0.16 0 0.97 0 0 0.97",
                  size="0.015", mass="0.04", rgba="0.15 0.18 0.2 1")
    ET.SubElement(body, "geom", name="lidar_housing", type="cylinder",
                  pos=f"0 0 {sensors.lidar_height-0.015}", size="0.055 0.025", mass="0.15", rgba="0.12 0.15 0.16 1")
    ET.SubElement(body, "site", name="lidar", pos=f"0 0 {sensors.lidar_height}", size="0.006", rgba="1 0.1 0.1 1")
    ET.SubElement(body, "geom", name="camera_housing", type="box",
                  pos=f"-0.025 0 {sensors.camera_height}", size="0.02 0.06 0.025", mass="0.12", rgba="0.1 0.1 0.1 1")
    ET.SubElement(body, "camera", name="rgbd", pos=f"0 0 {sensors.camera_height}",
                  xyaxes="0 -1 0 0 0 1", fovy=str(sensors.vertical_fov))
