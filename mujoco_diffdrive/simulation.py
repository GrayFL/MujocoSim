"""Legacy differential-drive convenience API, backed only by local code."""
from mujocosim.simulation import Simulation as Engine
from mujocosim.sensors import LaserScan, RGBDFrame
from mujocosim.math import transform
from mujocosim.config import SensorConfig
from mujocosim.robots import DifferentialDrive, DriveCommand

class Simulation(Engine):
    def __init__(self, scene_dir, *, robot=None, sensors=None, **options):
        if robot is None: robot = DifferentialDrive()
        elif not hasattr(robot, 'build'): robot = DifferentialDrive(robot)
        super().__init__(scene_dir, robot=robot, sensors=sensors or SensorConfig(), **options)

    @property
    def _base_qpos(self):
        return self.robot.base_qpos

    def set_velocity(self, linear, angular):
        return self.command(DriveCommand(linear, angular))
