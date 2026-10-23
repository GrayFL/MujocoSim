"""Engine and sensing settings; robot parameters live with their model."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class PhysicsConfig:
    timestep: float = .002
    settle_seconds: float = .5

    def __post_init__(self):
        if not math.isfinite(self.timestep) or self.timestep <= 0:
            raise ValueError('timestep must be positive')
        if not math.isfinite(self.settle_seconds) or self.settle_seconds < 0:
            raise ValueError('settle_seconds must be nonnegative')

@dataclass(frozen=True)
class SensorConfig:
    lidar_height: float = 0.4
    lidar_beams: int = 360
    lidar_min: float = 0.05
    lidar_max: float = 12.0
    camera_height: float = 1.0
    width: int = 640
    height: int = 480
    vertical_fov: float = 60.0
    depth_min: float = 0.05
    depth_max: float = 10.0

    def __post_init__(self):
        if self.lidar_beams < 4 or self.width < 1 or self.height < 1:
            raise ValueError("Invalid sensor resolution")
        if not 0 < self.vertical_fov < 180:
            raise ValueError("Camera vertical FOV must be in (0, 180) degrees")
        if not 0 < self.lidar_min < self.lidar_max or not 0 < self.depth_min < self.depth_max:
            raise ValueError("Sensor ranges must satisfy 0 < min < max")

