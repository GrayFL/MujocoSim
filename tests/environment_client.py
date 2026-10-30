"""Run with a genuinely engine-free interpreter; used by check_environments.py."""
import argparse
import asyncio
import importlib.util
import json
from pathlib import Path
import sys
import tempfile

from rsim import Runtime
from rsim.core import CommandRejected, VelocityCommand
from rsim.transport.descriptor import TransportConfig
from mujocosim.rsim import MujocoChassisFactory, SimulationDevice


async def run(args):
    assert importlib.util.find_spec('mujoco') is None
    assert importlib.util.find_spec('pyrobo') is None
    # Explicit revisions no longer bypass the content digest. Supply a real,
    # tiny scene description before checking the independent engine error.
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)/'scene.json'
        path.write_text(json.dumps({'format': 'mujocosim-scene-v1', 'room': {'size': [6, 6, 3]}}))
        local = MujocoChassisFactory(scene=path, scene_revision='test').build()
        try:
            async with Runtime(local.state):
                raise AssertionError('engine-free environment unexpectedly opened physics')
        except ModuleNotFoundError as error:
            assert 'mujocosim[engine]' in str(error)
            missing_engine_error = str(error)
    assert local._executor is None
    view = SimulationDevice(key='mujoco:' + args.name,
                            transport=TransportConfig('cyclonedds', args.domain))
    async with Runtime(view.pose, view.state, view.velocity, view.reset_request,
                       view.advance_request, view.inspect_request, view.control_request):
        initial = await view.state.get(timeout=10)
        await asyncio.sleep(.2)
        status = await view.describe()
        controlled = None
        if status['scheduling'] == 'controlled':
            await asyncio.sleep(.03)
            assert (await view.synchronize())['stamp_ns'] == status['stamp_ns'] == 0
            accepted = await view.velocity.set(VelocityCommand(), ttl=.4)
            assert accepted['kind'] == 'CommandAccepted'
            request = (accepted['context'], accepted['command_id'], 5)
            advanced = await view.advance(*request)
            assert advanced['kind'] == 'PhysicsAdvanced' and advanced['actual_substeps'] == 5
            assert advanced['stamp_ns'] == 10_000_000
            await asyncio.sleep(.41)
            assert await view.advance(*request) == advanced
            try:
                await view.advance(accepted['context'], accepted['command_id'], 6)
                raise AssertionError('changed duplicate request was accepted')
            except CommandRejected:
                pass
            synchronized = await view.synchronize()
            assert synchronized['last_result'] == advanced and synchronized['physical_substeps'] == 5
            paused = await view.pause()
            resumed = await view.resume()
            assert resumed['context']['episode'] == paused['context']['episode']+1
            assert resumed['stamp_ns'] == paused['stamp_ns']
            controlled = dict(accepted=accepted, advanced=advanced, synchronized=synchronized,
                              paused=paused, resumed=resumed)
        previous = await view.describe()
        ack = await view.reset()
        assert ack['episode'] == previous['context']['episode'] + 1
        current = await view.state.get(timeout=10)
        while current.metadata['episode'] != ack['episode']:
            current = await view.state.get(after=current.sequence, timeout=10)
        assert all(frame.metadata['episode'] == ack['episode'] for frame in view.state.frames)
        worker_pid = view.manifest['pid']
        cmdline = Path(f'/proc/{worker_pid}/cmdline')
        result = dict(executable=sys.executable, sys_path=sys.path, mujoco=None, pyrobo=None,
                      ack=ack, frame_episode=current.metadata['episode'],
                      controlled=controlled, initial_status=status,
                      missing_engine_error=missing_engine_error,
                      interface=view.interface_version, worker_pid=worker_pid,
                      worker_command=cmdline.read_bytes().decode().split('\0') if cmdline.exists() else None)
        print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True)
    parser.add_argument('--domain', type=int, required=True)
    asyncio.run(run(parser.parse_args()))
