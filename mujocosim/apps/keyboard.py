"""Reuse the RSim pygame dashboard to manually override point-cloud navigation."""
import argparse
import asyncio
import math
import time

from rsim.adapters.pygame_keyboard import PygameKeyboard
from rsim.components.vehicle import VehicleDynamics
from rsim.runtime import Runtime
from rsim.transport.descriptor import TransportConfig
from mujocosim.integrations.navigation import NavigationClient


async def run(name='navigation', domain=0,
              fonts='Inconsolata,Microsoft YaHei,Sarasa Mono SC,Noto Sans CJK SC,SimHei'):
    keys = PygameKeyboard(title='MujocoSim | Navigation and manual control', fonts=fonts, recover_stalls=True)
    client = NavigationClient(name, transport=TransportConfig('cyclonedds', domain))
    dynamics = VehicleDynamics()
    keys.present(connected=False, endpoint=name, dry_run=False)
    async with Runtime(keys, client) as runtime:
        previous, was_active, was_space = time.monotonic(), False, False

        async def control():
            nonlocal previous, was_active, was_space
            now = time.monotonic()
            dt, previous = now-previous, now
            if not keys.output.frames:
                return
            frame = keys.output.frames[-1]
            value = frame.data
            fresh = time.monotonic_ns()-frame.stamp_ns <= 200_000_000
            active = fresh and value['focused'] and not value['brake'] and not value.get('stalled')
            active = active and bool(value['keys']) and not value['quit']
            space = fresh and value.get('space', False)
            if active and dt <= dynamics.parameters.max_loop_gap:
                command = dynamics.step(value['keys'], dt)
                await client.manual(command.linear_x, command.angular_z,
                                    deadline_ns=frame.stamp_ns+250_000_000)
            else:
                dynamics.reset()
                if space and not was_space:
                    await client.manual(brake=True)
                elif was_active:
                    await client.manual(active=False)
            was_active, was_space = active, space
            state = client.last_state or {}
            pose = state.get('pose')
            if pose is not None:
                pose = [*pose[:2], math.degrees(pose[2])]
            keys.present(**dynamics.state(), connected=True, endpoint=f'{state.get("status", "connecting")} | {name}',
                         pose=pose, keys=value['keys'], brake=value['brake'], dry_run=not state.get('motion_enabled', False))

        keys.task('navigation-control', control, hz=30.)
        closed = asyncio.create_task(keys.quit_requested.wait())
        failed = asyncio.create_task(runtime.wait())
        try:
            done, _ = await asyncio.wait((closed, failed), return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                await task
        finally:
            closed.cancel()
            failed.cancel()
            await asyncio.gather(closed, failed, return_exceptions=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', default='navigation')
    parser.add_argument('--domain', type=int, default=0)
    parser.add_argument('--font', default='Inconsolata,Microsoft YaHei,Sarasa Mono SC,Noto Sans CJK SC,SimHei')
    args = parser.parse_args(argv)
    try:
        asyncio.run(run(args.name, args.domain, args.font))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
