"""Same pyrobo environment controller over real accelerated MuJoCo physics."""
import asyncio
from dataclasses import asdict
import json

import pytest

pytest.importorskip('pyrobo', reason='cross-library navigation requires the optional application extra')
from pyrobo import DWAConfig, Footprint
from pyrobo.apps.navigation_provider import navigate, room_map
from pyrobo.rsim import StackConfig
from rsim.devices import ChassisPortsDevice
from mujocosim.config import SensorConfig
from mujocosim.robots.differential_drive import DifferentialDrive, DifferentialDriveConfig
from mujocosim.rsim import MujocoChassisFactory


@pytest.mark.parametrize('rate', [1., 2., 10.])
def test_real_accelerated_provider_reaches_and_stops_with_environment_navigation(tmp_path, rate):
    scene = tmp_path/'room.json'
    scene.write_text(json.dumps({'format': 'mujocosim-scene-v1', 'room': {'size': [6, 6, 2.6]},
                                'spawn': [0, 0, 0], 'objects': []}))
    factory = MujocoChassisFactory(scene=scene, cache_root=tmp_path/'cache', motion_enabled=True,
        spawn=(0., 0., 0.), robot=DifferentialDrive(DifferentialDriveConfig(wheel_track=.4)),
        sensors=SensorConfig(width=80, height=60), scheduling='accelerated', speedup=rate)
    config = StackConfig(clock='simulation', scheduling='environment', control_period=.2,
        execution_period=.04, controller=DWAConfig(prediction_time=.6, integration_dt=.1,
            linear_samples=3, angular_samples=3), footprint=Footprint(.6, .6),
        command_speed_limit=.45, command_yaw_limit=.75)
    report = asyncio.run(navigate(ChassisPortsDevice.from_component(factory.build()), config=config,
                                  map_data=room_map(spacing=.3), execution='process', duration=20.))
    report['provider_fingerprint'] = factory.fingerprint()
    report['provider_configuration'] = asdict(factory)
    # Raw trajectory, source context, compute diagnostics and whole-job/window
    # timings remain inspectable. This correctness test makes no RTF guarantee.
    (tmp_path/'navigation.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    assert report['arrived'] and not report['collided']
    assert abs(report['samples'][-1]['speed']) < .02
    assert report['minimum_boundary_clearance_m'] > 1.
    assert report['clock'] == 'simulation' and report['scheduling'] == 'environment'
    assert report['execution'] == 'process' and report['navigation_compute_steps'] > 0
    assert all(sample['source_rate'] == rate for sample in report['samples'])
    assert len({(sample['session'], sample['episode']) for sample in report['samples']}) == 1
