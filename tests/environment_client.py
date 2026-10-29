"""Run with a genuinely engine-free interpreter; used by check_environments.py."""
import argparse
import asyncio
import importlib.util
import json
from pathlib import Path
import sys

from rsim import Runtime
from rsim.transport.descriptor import TransportConfig
from mujocosim.rsim import MujocoChassisFactory, SimulationDevice


async def run(args):
    assert importlib.util.find_spec('mujoco') is None
    assert importlib.util.find_spec('pyrobo') is None
    local = MujocoChassisFactory(scene_dir='/not-needed-without-engine', scene_revision='test').build()
    try:
        async with Runtime(local.state):
            raise AssertionError('engine-free environment unexpectedly opened physics')
    except ModuleNotFoundError as error:
        assert 'mujocosim[engine]' in str(error)
        missing_engine_error = str(error)
    assert local._executor is None
    view = SimulationDevice(key='mujoco:' + args.name,
                            transport=TransportConfig('cyclonedds', args.domain))
    async with Runtime(view.pose, view.state, view.reset_request):
        initial = await view.state.get(timeout=10)
        await asyncio.sleep(.2)
        ack = await view.reset()
        assert ack['episode'] == initial.metadata['episode'] + 1
        current = await view.state.get(timeout=10)
        while current.metadata['episode'] != ack['episode']:
            current = await view.state.get(after=current.sequence, timeout=10)
        assert all(frame.metadata['episode'] == ack['episode'] for frame in view.state.frames)
        worker_pid = view.manifest['pid']
        cmdline = Path(f'/proc/{worker_pid}/cmdline')
        result = dict(executable=sys.executable, sys_path=sys.path, mujoco=None, pyrobo=None,
                      ack=ack, frame_episode=current.metadata['episode'],
                      missing_engine_error=missing_engine_error,
                      interface=view.interface_version, worker_pid=worker_pid,
                      worker_command=cmdline.read_bytes().decode().split('\0') if cmdline.exists() else None)
        print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True)
    parser.add_argument('--domain', type=int, required=True)
    asyncio.run(run(parser.parse_args()))
