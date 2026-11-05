import json
import xml.etree.ElementTree as ET
import numpy as np
import pytest

from mujocosim import SensorConfig
from mujocosim.robots import DifferentialDrive


def test_complete_scene_collision_covers_both_levels(tmp_path):
    trimesh = pytest.importorskip('trimesh')
    from mujocosim.scenes import prepare_scene
    scene = trimesh.Scene()
    for z in (-.1, 2.9):
        floor = trimesh.creation.box(extents=[4, 4, .2])
        floor.apply_translation([0, 0, z])
        scene.add_geometry(floor)
    wall = trimesh.creation.box(extents=[.1, 4, 6])
    wall.apply_translation([1.95, 0, 3])
    scene.add_geometry(wall)
    source = tmp_path/'scene.glb'
    source.write_bytes(scene.export(file_type='glb'))
    directory = prepare_scene(source, tmp_path/'cache', up_axis='z', pitch=.2,
                              floor_height=0., collision_scope='scene')
    metadata = json.loads((directory/'metadata.json').read_text())
    assert metadata['collision_z_range'][1] > 5
    collision = ET.parse(directory/'scene.xml').findall("worldbody/geom[@type='box']")
    assert any(float(geom.get('pos').split()[2]) > 3 for geom in collision)
    planar = prepare_scene(source, tmp_path/'cache', up_axis='z', pitch=.2, floor_height=0.)
    assert planar != directory
    assert json.loads((planar/'metadata.json').read_text())['collision_z_range'] == [0, 1.16]


def test_glb_texture_vertical_orientation_survives_mujoco_conversion(tmp_path):
    trimesh = pytest.importorskip('trimesh')
    pytest.importorskip('mujoco')
    from PIL import Image
    from mujocosim import Simulation
    from mujocosim.scenes import prepare_scene
    pixels = np.zeros((32, 32, 3), np.uint8)
    pixels[:16] = [255, 0, 0]
    pixels[16:] = [0, 0, 255]
    wall = trimesh.Trimesh(vertices=[[2, -1, 0], [2, 1, 0], [2, 1, 2], [2, -1, 2]],
                          faces=[[0, 2, 1], [0, 3, 2]], process=False)
    wall.visual = trimesh.visual.TextureVisuals(uv=[[0, 0], [1, 0], [1, 1], [0, 1]], image=Image.fromarray(pixels))
    floor = trimesh.creation.box(extents=[5, 5, .2])
    floor.apply_translation([0, 0, -.1])
    source = tmp_path/'color.glb'
    source.write_bytes(trimesh.Scene([floor, wall]).export(file_type='glb'))
    prepared = prepare_scene(source, tmp_path/'cache', up_axis='z', floor_height=0, pitch=.2)
    with Simulation(prepared, robot=DifferentialDrive(), spawn=[0, 0, 0],
                    sensors=SensorConfig(width=80, height=80, camera_height=1)) as simulation:
        rgb = simulation.rgbd().rgb
        assert rgb[20, 40, 0] > 200 and rgb[20, 40, 2] < 30
        assert rgb[60, 40, 2] > 200 and rgb[60, 40, 0] < 30
