"""Serve MuJoCo physics in a shared worker and chassis control over Cyclone DDS."""
import argparse
import asyncio
import json
import os
from pathlib import Path

from rsim.runtime import Runtime
from rsim.transport.descriptor import TransportConfig
from mujocosim.rsim.drivers import MujocoChassisFactory
from mujocosim.integrations.transport import configure_transport
from mujocosim.config import SensorConfig
from mujocosim.deployment import DEFAULT_CACHE


async def run(args):
    transport = configure_transport(TransportConfig('cyclonedds', args.domain))
    factory = MujocoChassisFactory(args.scene, cache_root=args.cache_dir,
                    motion_enabled=args.enable_motion, hz=args.hz,
                    sensors=SensorConfig(width=args.width, height=args.height))
    shared = factory.shared(key='mujoco:' + args.name, transport=transport, log_path=args.log)
    service = factory.service(source=shared, name=args.name, transport=transport)
    async with Runtime(service) as runtime:
        await service.pose.get(timeout=30)
        status = dict(name=args.name, domain=args.domain, service_pid=os.getpid(),
                      physics_pid=shared.manifest['pid'], storage=str(shared.directory),
                      motion_enabled=args.enable_motion, hz=args.hz)
        if args.ready_file:
            path = Path(args.ready_file)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(status, indent=2), encoding='utf-8')
        print(json.dumps(status, indent=2), flush=True)
        if args.seconds is None:
            await runtime.wait()
        else:
            try:
                async with asyncio.timeout(args.seconds):
                    await runtime.wait()
            except TimeoutError:
                pass


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', required=True)
    parser.add_argument('--name', default='mujoco')
    parser.add_argument('--domain', type=int, default=0)
    parser.add_argument('--enable-motion', action='store_true')
    parser.add_argument('--cache-dir', default=DEFAULT_CACHE)
    parser.add_argument('--log', default='assets/runs/provider.log')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--hz', type=float, default=50.,
                        help='Simulation publication rate; physics timestep remains 0.002 s')
    parser.add_argument('--seconds', type=float, help='Optional service lifetime after ready')
    parser.add_argument('--ready-file', help='Write service and physics PIDs after startup')
    args = parser.parse_args(argv)
    if args.seconds is not None and (not __import__('math').isfinite(args.seconds) or args.seconds <= 0):
        parser.error('--seconds must be finite and positive')
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
