"""Only this project's sensor/command mapping; no RSim framework regression."""
import asyncio
import time
import numpy as np
import pytest

from rsim import Runtime
from rsim.core import VelocityCommand, CommandRejected
from mujocosim.config import SensorConfig
from mujocosim.integrations.rsim import create_source
from test_simulation import make_room


def test_real_sensor_frames_odometry_reset_and_owner_thread_deadline(tmp_path):
    make_room(tmp_path)
    async def run():
        source = create_source(scene_dir=tmp_path, sensors=SensorConfig(width=80, height=60), motion_enabled=True)
        async with Runtime(source):
            rgb, depth, scan = await source.image.get(), await source.depth.get(), await source.scan.get()
            assert rgb.stamp_ns == depth.stamp_ns == scan.stamp_ns
            assert rgb.data.pixels.shape == (60, 80, 3)
            assert not rgb.data.pixels.flags.writeable
            assert not scan.data['ranges'].flags.writeable
            assert isinstance(rgb.metadata['world_from_base'], tuple)
            assert np.asarray(rgb.metadata['world_from_base']).shape == (4, 4)
            assert np.asarray((await source.camera_info.get()).data['K']).shape == (3, 3)
            assert np.isfinite(depth.data.pixels).all()
            assert (await source.imu.get()).data['linear_acceleration']['z'] == pytest.approx(9.81, abs=.5)
            await source.velocity.set(VelocityCommand(.2), ttl=.15)
            await asyncio.sleep(.1)
            assert (await source.odom.get()).data['pose']['pose']['position']['x'] > .003
            await source.inspect(lambda sim: time.sleep(.2))
            state = await source.state.get()
            state = await source.state.get(after=state.sequence, timeout=2)
            assert state.data['wheel_targets'] == [0., 0.]
            await source.reset()
            assert (await source.state.get()).stamp_ns > state.stamp_ns
            assert (await source.odom.get()).data['pose']['pose']['position']['x'] == 0
        assert source._executor is None
        assert source.simulation._renderer is None
    asyncio.run(run())


def test_disabled_commands_and_no_render_without_image_demand(tmp_path):
    make_room(tmp_path)
    async def run():
        source = create_source(scene_dir=tmp_path)
        async with Runtime(source.state):
            with pytest.raises(CommandRejected, match='motion_enabled'):
                await source.velocity.set(VelocityCommand(.1))
            assert source.simulation._renderer is None
            assert not source.image.frames
            assert not source.scan.frames
    asyncio.run(run())
