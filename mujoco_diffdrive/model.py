from mujocosim.model import build_model_xml as build
from mujocosim.robots import DifferentialDrive
from mujocosim.config import PhysicsConfig

def build_model_xml(scene_dir, robot, sensors, **options):
    if not hasattr(robot, 'build'): robot = DifferentialDrive(robot)
    return build(scene_dir, robot, sensors, PhysicsConfig(), **options)
