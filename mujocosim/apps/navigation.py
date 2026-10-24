"""Run a graphmap + pyrobo world, with a separate notebook/pygame DDS client."""
import argparse
import asyncio
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import time

from rsim.core import Component, Connect
from rsim.runtime import Runtime
from rsim.runtime.sharing import SharedProvider
from rsim.transport.descriptor import TransportConfig

from mujocosim.components.mapping import MappingConfig, PointCloudMapper
from mujocosim.components.navigation import NavigationConfig, PointCloudNavigation
from mujocosim.components.pixel_target import PixelTarget
from mujocosim.config import SensorConfig
from mujocosim.deployment import DEFAULT_SCENE, DEFAULT_CACHE, new_map_directory
from mujocosim.integrations.navigation import NavigationService
from mujocosim.integrations.rsim import create_source
from mujocosim.integrations.transport import configure_transport


class NavigationWorld(Component):
    def __init__(self, scene, *, name='navigation', transport=None, cache_dir=DEFAULT_CACHE,
                 motion_enabled=False, viewer=False, output=None, map_output=None, sensors=None,
                 mapping=None, navigation=None, pixel_target=None):
        mapping = mapping or MappingConfig()
        self.source = create_source(scene, cache_root=cache_dir, motion_enabled=motion_enabled,
                                    sensors=sensors or SensorConfig(), camera_hz=mapping.hz)
        self.mapper = PointCloudMapper(self.source, config=mapping)
        self.navigator = PointCloudNavigation(self.source, self.mapper,
                                              config=navigation, motion_enabled=motion_enabled)
        self.writer = Connect(self.navigator.command, self.source.velocity, ttl=.25, hz=50.)
        self.pixel_target = PixelTarget(self.source, self.mapper, self.navigator, config=pixel_target)
        self.service = NavigationService(name, self.navigator, targets=self.pixel_target, transport=transport)
        super().__init__(self.writer, self.service)
        self.viewer_enabled, self.output = viewer, output
        self.map_output = (Path(map_output).expanduser().resolve() if map_output else
                           new_map_directory('navigation') if output else None)
        self.viewer = None
        for name, signal in [('map', self.mapper.map), ('path', self.navigator.path),
                             ('state', self.navigator.state), ('pose', self.source.pose),
                             ('observation', self.pixel_target.observation), ('targets', self.pixel_target.targets),
                             ('topology', self.pixel_target.topology)]:
            self.expose(name, signal)

    async def open(self):
        if self.viewer_enabled:
            import mujoco.viewer
            from .demo import Keyboard
            self.keys = Keyboard()
            self.viewer = mujoco.viewer.launch_passive(self.source.simulation.model,
                self.source.simulation.data, key_callback=self.keys)
            with self.viewer.lock():
                self.viewer.opt.geomgroup[:] = [1, 1, 0, 0, 0, 0]
                self.viewer.cam.lookat[:] = [*self.source.simulation.pose2d[:2], .4]
                self.viewer.cam.distance, self.viewer.cam.azimuth, self.viewer.cam.elevation = 5., 135, -55
            self.last_key_command = (0., 0.)
            self.last_key_revision = 0
            self.task('viewer', self.sync_viewer, hz=20.)

    async def sync_viewer(self):
        with self.keys.lock:
            command, reset, quit_requested = self.keys.command, self.keys.reset, self.keys.quit
            revision = self.keys.command_revision
            self.keys.reset = False
        if not self.viewer.is_running() or quit_requested:
            self.navigator.cancel('viewer_closed')
            self.service.quit_requested = True
            self.navigator.quit_requested = True
            return
        if reset:
            self.navigator.cancel('simulation_reset')
            await self.source.reset()
        changed = revision != self.last_key_revision
        if any(command) and (changed or self.navigator.manual_owner == 'viewer'):
            self.navigator.manual(*command, owner='viewer', deadline=time.monotonic_ns()+250_000_000)
        elif changed and not any(command):
            self.navigator.manual(0, 0, owner='viewer', deadline=0, brake=True)
        self.last_key_command = command
        self.last_key_revision = revision
        await self.source.inspect(lambda sim: self.viewer.sync())

    async def close(self):
        self.navigator.cancel('world_closed')
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
        if self.map_output is not None:
            await self.mapper.save(self.map_output/'graphmap')
            await self.pixel_target.record()
            self.pixel_target.save(self.map_output/'topology.json')
            if self.output:
                output = Path(self.output)
                output.mkdir(parents=True, exist_ok=True)
                (output/'maps.json').write_text(json.dumps(dict(map_output=str(self.map_output),
                    graphmap=str(self.map_output/'graphmap'), topology=str(self.map_output/'topology.json')),
                    indent=2), encoding='utf-8')


async def run(args):
    transport = configure_transport(TransportConfig('cyclonedds', args.domain))
    output = Path(args.output).resolve()
    map_output = Path(args.map_output).expanduser().resolve() if args.map_output else new_map_directory('navigation')
    output.mkdir(parents=True, exist_ok=True)
    config = NavigationConfig(planner=args.planner, bounds=tuple(args.bounds) if args.bounds else None)
    world = NavigationWorld(args.scene, name=args.name, transport=transport,
        cache_dir=args.cache_dir, motion_enabled=args.enable_motion, viewer=not args.headless,
        output=str(output), map_output=map_output,
        sensors=SensorConfig(width=args.width, height=args.height), navigation=config)
    shared = SharedProvider(world, key='mujocosim-navigation:'+args.name,
        interface_version='mujocosim-navigation-v1',
        provider_version=str(('pixel-target-v1', str(Path(args.scene).resolve()), asdict(config), args.width, args.height,
                              args.enable_motion, args.headless, str(output), str(map_output))),
        transport=transport, log_path=str(output/'world.log'))
    # Export only requested ports. Map/path consumers in the notebook create
    # their own subscriptions; this lease keeps the physical world alive.
    async with Runtime(shared.state) as runtime:
        state = await shared.state.get(timeout=30.)
        while not state.data.get('map', {}).get('images'):
            state = await shared.state.get(after=state.sequence, timeout=15.)
        status = dict(name=args.name, domain=args.domain, launcher_pid=os.getpid(),
            physics_pid=shared.manifest['pid'], scene=str(Path(args.scene).resolve()),
            motion_enabled=args.enable_motion, planner=args.planner, output=str(output), map_output=str(map_output))
        (output/'ready.json').write_text(json.dumps(status, indent=2), encoding='utf-8')
        print(json.dumps(status, indent=2), flush=True)
        print('Ready. Start examples/navigation.ipynb or python -m mujocosim.apps.keyboard', flush=True)
        tasks = [asyncio.create_task(runtime.wait())]
        if args.keyboard:
            from .keyboard import run as keyboard_run
            tasks.append(asyncio.create_task(keyboard_run(args.name, args.domain)))

        async def lifetime():
            started = time.monotonic()
            while not (output/'stop').exists():
                if args.seconds is not None and time.monotonic()-started >= args.seconds:
                    return
                # The viewer updates the RPC service; the world state shares
                # this flag through the navigation component below.
                if shared.state.frames[-1].data.get('quit_requested'):
                    return
                await asyncio.sleep(.1)
        tasks.append(asyncio.create_task(lifetime()))
        try:
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                await task
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', default=DEFAULT_SCENE)
    parser.add_argument('--cache-dir', default=DEFAULT_CACHE)
    parser.add_argument('--name', default='navigation')
    parser.add_argument('--domain', type=int, default=0)
    parser.add_argument('--planner', choices=['rrt_connect', 'voronoi'], default='rrt_connect')
    parser.add_argument('--bounds', nargs=4, type=float, metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'))
    parser.add_argument('--enable-motion', action='store_true')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--keyboard', action='store_true', help='Open the existing RSim pygame controller')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--output', default='assets/runs/navigation')
    parser.add_argument('--map-output', help='Map data directory; defaults to <data-root>/maps/navigation/<run-id>')
    parser.add_argument('--seconds', type=float)
    args = parser.parse_args(argv)
    if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0):
        parser.error('--seconds must be finite and positive')
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
