"""Run the real DDS service and motion client in separate processes.

This is an opt-in application check requiring DDS, OpenGL and a prepared scene.
It does not run RSim's framework test suites or change command timeouts.
"""
import argparse
import asyncio
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace
import uuid

from mujocosim.deployment import DEFAULT_SCENE, DEFAULT_CACHE


async def serve(args):
    from mujocosim.apps.service import run

    settings = SimpleNamespace(scene=args.scene, cache_dir=args.cache_dir,
        name=args.name, domain=args.domain, enable_motion=True, hz=50.,
        width=640, height=480, seconds=None,
        ready_file=str(args.output / 'ready.json'), log=str(args.output / 'provider.log'))
    service = asyncio.create_task(run(settings))

    async def stopped():
        while not (args.output / 'stop').exists():
            await asyncio.sleep(.05)

    stop = asyncio.create_task(stopped())
    try:
        done, _ = await asyncio.wait((service, stop), return_when=asyncio.FIRST_COMPLETED)
        if service in done:
            await service
    finally:
        service.cancel()
        stop.cancel()
        results = await asyncio.gather(service, stop, return_exceptions=True)
        for result in results:
            if isinstance(result, Exception):
                raise result


def validate_motion(result):
    start, moved, final = result['start'], result['moved'], result['final']
    forward = math.dist(start[:2], moved[:2])
    reverse = math.dist(moved[:2], final[:2])
    initial_yaw = math.atan2(moved[1]-start[1], moved[0]-start[0])
    turn = math.atan2(math.sin(result['turned_yaw_rad']-initial_yaw),
                      math.cos(result['turned_yaw_rad']-initial_yaw))
    assert .18 < forward < .22, result
    assert .08 < reverse < .12, result
    assert abs(turn-math.pi/6) < .04, result
    assert result['image_shape'] == [480, 640, 3], result
    assert result['depth_shape'] == [480, 640], result
    assert result['lidar_beams'] == 360 and result['shared_image'], result
    return dict(forward_m=forward, reverse_m=reverse, turn_deg=math.degrees(turn))


def check(args):
    root = Path(__file__).resolve().parents[1]
    args.scene = str(Path(args.scene).resolve())
    args.cache_dir = str(Path(args.cache_dir).resolve())
    output = args.output.resolve() / uuid.uuid4().hex[:8]
    output.mkdir(parents=True)
    base = [sys.executable, '-X', 'utf8', '-m']
    flags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0
    name = 'mujocosim_check_' + uuid.uuid4().hex[:12]
    results = []
    print(f'Checking {args.scene}; results: {output}', flush=True)
    with (output / 'service.log').open('w', encoding='utf-8') as log:
        server = subprocess.Popen(base + ['examples.check_dds_motion', '--serve',
            '--scene', args.scene, '--cache-dir', args.cache_dir, '--name', name,
            '--domain', str(args.domain), '--output', str(output)],
            cwd=root, stdout=log, stderr=subprocess.STDOUT, creationflags=flags)
        try:
            deadline = time.monotonic()+45
            ready_path = output / 'ready.json'
            while not ready_path.exists():
                if server.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError(f'service startup failed; see {output / "service.log"}')
                time.sleep(.1)
            ready = json.loads(ready_path.read_text(encoding='utf-8'))
            for index in range(args.rounds):
                destination = output / f'client_{index+1}'
                with (output / f'client_{index+1}.log').open('w', encoding='utf-8') as client_log:
                    subprocess.run(base + ['mujocosim.apps.motion', '--connect', name,
                        '--domain', str(args.domain), '--output', str(destination)],
                        cwd=root, stdout=client_log, stderr=subprocess.STDOUT,
                        timeout=70, check=True, creationflags=flags)
                result = json.loads((destination / 'motion_summary.json').read_text(encoding='utf-8'))
                assert result['physics_pid'] == ready['physics_pid']
                assert len({server.pid, result['physics_pid'], result['client_pid']}) == 3
                results.append(dict(round=index+1, client_pid=result['client_pid'],
                                    **validate_motion(result)))
                print(f'Round {index+1}/{args.rounds}: passed', flush=True)
            # Full diagnostics remain available on explicit request, outside
            # the movement loop; regular liveness checks do not collect them.
            from rsim.runtime.sharing import describe_shared
            from rsim.transport.descriptor import TransportConfig
            from mujocosim.integrations.transport import configure_transport
            manifest = asyncio.run(describe_shared('mujoco:' + name,
                transport=configure_transport(TransportConfig('cyclonedds', args.domain))))
            assert manifest['pid'] == ready['physics_pid'] and not manifest['port_errors']
            transport_stats = manifest['transport_stats']
        finally:
            # Cancelling service.run exits its Runtime normally, releasing the
            # source lease and stopping the supervised physics worker.
            (output / 'stop').write_text('stop\n', encoding='utf-8')
            try:
                server.wait(timeout=25)
            except subprocess.TimeoutExpired:
                server.terminate()
                server.wait(timeout=10)
        if server.returncode:
            raise RuntimeError(f'service cleanup failed; see {output / "service.log"}')
    report = dict(scene=args.scene, service_pid=server.pid, physics_pid=ready['physics_pid'],
                  service_exit=server.returncode, rounds=results, transport_stats=transport_stats)
    path = output / 'summary.json'
    path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(f'Passed: {path}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', default=str(DEFAULT_SCENE))
    parser.add_argument('--cache-dir', default=DEFAULT_CACHE)
    parser.add_argument('--output', type=Path, default=Path('assets/runs/dds_motion'))
    parser.add_argument('--domain', type=int, default=0)
    parser.add_argument('--rounds', type=int, default=1)
    parser.add_argument('--serve', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--name', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.rounds < 1:
        parser.error('--rounds must be positive')
    if args.serve:
        asyncio.run(serve(args))
    else:
        check(args)


if __name__ == '__main__':
    main()
