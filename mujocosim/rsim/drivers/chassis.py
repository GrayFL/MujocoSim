"""Differential-drive device recipe and its single complete port declaration."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
import hashlib
import json
import math
from pathlib import Path
from typing import ClassVar

from rsim.core.commands import CommandRejected, VelocityCommand
from rsim.runtime.sharing import PortSpec, SharedProvider
from mujocosim.config import PhysicsConfig, SensorConfig
from mujocosim.robots.differential_drive import DifferentialDrive, DifferentialDriveConfig


def validate_velocity(value):
    if not isinstance(value, VelocityCommand):
        raise CommandRejected('expected VelocityCommand')


def zero_velocity(value):
    return value == VelocityCommand()


def _resource_digest(path):
    """Explicit provider-side content identity; no scene or engine import."""
    path = Path(path).expanduser().resolve()
    def resources(item, seen):
        if item in seen:
            return
        seen.add(item)
        if item.is_dir():
            # Match prepared-scene inputs consumed by Simulation, excluding
            # logs, recordings and the optional generated world.xml.
            files = [item/name for name in ('scene.xml', 'metadata.json', 'navigation.npz')]
            files.extend(p for p in item.iterdir() if p.suffix.lower() in ('.obj', '.png'))
            for file in files:
                yield from resources(file, seen)
            return
        yield item
        if item.suffix.lower() not in ('.json', '.gltf'):
            return
        def sources(value):
            if isinstance(value, dict):
                for name, item in value.items():
                    if name in ('source', 'base_scene', 'uri') and isinstance(item, str):
                        if not item.startswith('data:'):
                            yield item
                    else:
                        yield from sources(item)
            elif isinstance(value, list):
                for item in value:
                    yield from sources(item)
        for relative in sources(json.loads(item.read_text(encoding='utf-8'))):
            yield from resources((item.parent/relative).resolve(), seen)
    digest = hashlib.sha256()
    for item in sorted(set(resources(path, set()))):
        digest.update(str(item).encode('utf-8'))
        with item.open('rb') as stream:
            for block in iter(lambda: stream.read(1024*1024), b''):
                digest.update(block)
    return digest.hexdigest()


@dataclass(frozen=True)
class MujocoChassisFactory:
    """Immutable recipe. Only ``open`` of the component loads the engine.

    ``device`` connects to an existing provider. Scene-free clients can construct
    :class:`mujocosim.rsim.devices.SimulationDevice` directly.
    """
    INTERFACE_VERSION: ClassVar[str] = 'chassis-ports-v1'
    FACTORY_VERSION: ClassVar[str] = 'mujocosim-chassis-v1'

    scene: str | Path | None = None
    scene_dir: str | Path | None = None
    scene_revision: str | None = None
    cache_root: str | Path | None = None
    up_axis: str = 'auto'
    floor_height: float | None = None
    pitch: float = .08
    rebuild: bool = False
    spawn: tuple[float, float, float] | None = None
    robot: DifferentialDrive = field(default_factory=DifferentialDrive)
    sensors: SensorConfig = field(default_factory=SensorConfig)
    physics: PhysicsConfig = field(default_factory=PhysicsConfig)
    lidar_hz: float = 10.
    camera_hz: float = 5.
    hz: float = 50.
    speedup: float = 1.
    motion_enabled: bool = False
    history: int = 32
    image_history: int = 8
    max_ttl: float = .5
    clock: str = 'simulation'
    reset_policy: str = 'native'

    def __post_init__(self):
        if (self.scene is None) == (self.scene_dir is None):
            raise ValueError('supply exactly one of scene or scene_dir')
        for name in ('scene', 'scene_dir', 'cache_root'):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, str(Path(value).expanduser().resolve()))
        if self.spawn is not None:
            spawn = tuple(float(v) for v in self.spawn)
            if len(spawn) != 3 or not all(math.isfinite(v) for v in spawn):
                raise ValueError('spawn must be finite [x, y, yaw_radians]')
            object.__setattr__(self, 'spawn', spawn)
        if (not isinstance(self.robot, DifferentialDrive)
                or not isinstance(self.robot.config, DifferentialDriveConfig)):
            raise TypeError('MujocoChassisFactory requires an immutable DifferentialDrive model')
        if not all(math.isfinite(v) and v > 0 for v in vars(self.robot.config).values()):
            raise ValueError('robot dimensions and limits must be finite and positive')
        if type(self.motion_enabled) is not bool or type(self.rebuild) is not bool:
            raise TypeError('motion_enabled and rebuild must be booleans')
        if self.scene_revision is not None and not isinstance(self.scene_revision, str):
            raise TypeError('scene_revision must be an immutable version string')
        if self.floor_height is not None and not math.isfinite(self.floor_height):
            raise ValueError('floor_height must be finite')
        if not isinstance(self.sensors, SensorConfig) or not isinstance(self.physics, PhysicsConfig):
            raise TypeError('sensors/physics must be immutable SensorConfig/PhysicsConfig')
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.hz, self.speedup, self.lidar_hz, self.camera_hz, self.pitch, self.max_ttl)):
            raise ValueError('rates, pitch and max_ttl must be finite and positive')
        substeps = 1/self.hz/self.physics.timestep
        if round(substeps) < 1 or not math.isclose(substeps, round(substeps), abs_tol=1e-8):
            raise ValueError('1/hz must be an integer multiple of the physics timestep')
        if not isinstance(self.clock, str) or not self.clock:
            raise ValueError('clock must be nonempty')
        if self.reset_policy not in ('native', 'legacy_continuous'):
            raise ValueError('reset_policy must be native or legacy_continuous')
        self.specs()

    @staticmethod
    def port_specs(clock='simulation', *, history=32, image_history=8, max_ttl=.5):
        """Fresh complete declarations used by component, provider and client."""
        ports = {name: PortSpec(clock=clock, history_capacity=history)
                 for name in ('pose', 'truth', 'odom', 'imu', 'state', 'scan', 'lidar_points')}
        ports.update({name: PortSpec(clock=clock, history_capacity=image_history)
                      for name in ('image', 'depth', 'camera_info', 'points')})
        ports['velocity'] = PortSpec(direction='sink', clock='host:monotonic',
                                     max_ttl=max_ttl, fallback=VelocityCommand())
        ports['reset_request'] = PortSpec(direction='sink', clock='host:monotonic',
                                          max_ttl=max_ttl, fallback=None)
        return ports

    def specs(self):
        return self.port_specs(self.clock, history=self.history, image_history=self.image_history,
                               max_ttl=self.max_ttl)

    def fingerprint(self):
        configuration = asdict(self)
        configuration['factory'] = self.FACTORY_VERSION
        configuration['interface'] = self.INTERFACE_VERSION
        configuration['scene_content'] = (self.scene_revision if self.scene_revision is not None
                                           else _resource_digest(self.scene or self.scene_dir))
        data = json.dumps(configuration, sort_keys=True, separators=(',', ':'), allow_nan=False)
        return hashlib.sha256(data.encode('utf-8')).hexdigest()

    def signature(self):
        return self.fingerprint()

    def build_backend(self):
        from mujocosim.rsim.components.backend import DifferentialDriveBackend
        options = {name: getattr(self, name) for name in
                   ('scene', 'scene_dir', 'up_axis', 'floor_height', 'pitch', 'rebuild',
                    'spawn', 'robot', 'sensors', 'physics', 'lidar_hz', 'camera_hz', 'motion_enabled')}
        if self.cache_root is not None:
            options['cache_root'] = self.cache_root
        return DifferentialDriveBackend(**options)

    def _build(self, *, legacy_access=False):
        from mujocosim.rsim.components.protocol import CommandSpec
        from mujocosim.rsim.components.source import LegacySimulationSource, SimulationSource
        cls = LegacySimulationSource if legacy_access else SimulationSource
        return cls(self.build_backend, ports=self.specs(),
                   commands={'velocity': CommandSpec(VelocityCommand(), zero_velocity, validate_velocity)},
                   hz=self.hz, speedup=self.speedup, motion_enabled=self.motion_enabled,
                   clock=self.clock, reset_policy=self.reset_policy, configuration=self.fingerprint())

    def build(self):
        return self._build()

    __call__ = build

    def shared(self, *, key, transport=None, options=None, log_path=None):
        return SharedProvider(factory=self.build, ports=self.specs(), key=key,
                              interface_version=self.INTERFACE_VERSION,
                              provider_version=self.fingerprint(), transport=transport,
                              options=options, log_path=log_path)

    def device(self, *, key, transport=None, options=None):
        from mujocosim.rsim.devices import SimulationDevice
        return SimulationDevice(key=key, clock=self.clock, history=self.history,
                                image_history=self.image_history, max_ttl=self.max_ttl,
                                transport=transport, options=options)

    def service(self, *, source=None, name='chassis', transport=None, control=None):
        """Optional RPC facade; pass ``source=shared`` to reuse its one World."""
        from rsim.drivers import Chassis
        source = self.build() if source is None else source
        return Chassis(name=name, pose=source.pose, velocity=source.velocity, state=source.state,
                       motion_enabled=self.motion_enabled, transport=transport, control=control)

    def with_scene(self, scene, **changes):
        return replace(self, scene=scene, scene_dir=None, scene_revision=None, **changes)
