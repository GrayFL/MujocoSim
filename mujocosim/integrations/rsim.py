"""Compatibility facade (retired in 0.4); use ``mujocosim.rsim`` for new code.

Old tooling retains continuous reset timestamps and direct inspection. Runtime
behavior is implemented only by the new local owner.
"""
from mujocosim.rsim.components.backend import DifferentialDriveBackend, readonly, matrix_metadata, xyz_cloud
from mujocosim.rsim.components.source import LegacySimulationSource as SimulationSource
from mujocosim.rsim.components.protocol import CommandSpec, Observation
from mujocosim.rsim.drivers.chassis import MujocoChassisFactory, validate_velocity, zero_velocity


def create_source(scene=None, *, scene_dir=None, hz=50., speedup=1., motion_enabled=False,
                  history=128, image_history=8, **options):
    """Old signature delegates to one factory; defaults preserve legacy resets."""
    options.setdefault('reset_policy', 'legacy_continuous')
    options = {name: value for name, value in options.items()
               if value is not None or name not in ('robot', 'sensors', 'physics')}
    factory = MujocoChassisFactory(scene=scene, scene_dir=scene_dir, hz=hz, speedup=speedup,
                 motion_enabled=motion_enabled, history=history, image_history=image_history, **options)
    return factory._build(legacy_access=True)
