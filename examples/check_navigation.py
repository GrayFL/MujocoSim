"""Opt-in real notebook kernel and pygame/DDS application acceptance check.

The scripted pygame check posts SDL events inside its own test window. It uses
the production renderer, IPC, control client and simulator; it does not inject
desktop input or automate other applications.
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
import uuid


def scripted_window(channel, title, fonts, render_hz):
    import pygame as pg
    from rsim.adapters.pygame_window import window_loop
    events = []

    def mark(name):
        events.append(dict(event=name, at_ns=time.monotonic_ns()))

    async def inject():
        while not pg.display.get_init() or pg.display.get_surface() is None:
            await asyncio.sleep(.05)
        await asyncio.sleep(2)
        pg.event.post(pg.event.Event(pg.WINDOWFOCUSGAINED))
        pg.event.post(pg.event.Event(pg.KEYDOWN, key=pg.K_w, repeat=False))
        mark('w_down')
        await asyncio.sleep(1.5)
        pg.event.post(pg.event.Event(pg.KEYUP, key=pg.K_w))
        mark('w_up')
        await asyncio.sleep(.5)
        pg.event.post(pg.event.Event(pg.KEYDOWN, key=pg.K_a, repeat=False))
        mark('a_down')
        await asyncio.sleep(.7)
        pg.event.post(pg.event.Event(pg.WINDOWFOCUSLOST))
        mark('focus_lost')
        await asyncio.sleep(.7)
        pg.event.post(pg.event.Event(pg.KEYUP, key=pg.K_a))
        pg.event.post(pg.event.Event(pg.WINDOWFOCUSGAINED))
        mark('focus_gained')
        pg.event.post(pg.event.Event(pg.KEYDOWN, key=pg.K_SPACE, repeat=False))
        mark('space_down')
        await asyncio.sleep(.2)
        pg.event.post(pg.event.Event(pg.KEYUP, key=pg.K_SPACE))
        await asyncio.sleep(.5)
        pg.image.save(pg.display.get_surface(), os.environ['MUJOCOSIM_PYGAME_SCREENSHOT'])
        mark('close')
        Path(os.environ['MUJOCOSIM_PYGAME_SCREENSHOT']).with_suffix('.events.json').write_text(
            json.dumps(dict(window_pid=os.getpid(), events=events), indent=2), encoding='utf-8')
        pg.event.post(pg.event.Event(pg.QUIT))

    async def run():
        injected = asyncio.create_task(inject())
        try:
            await window_loop(channel, title, fonts, render_hz)
            await injected
        finally:
            injected.cancel()
            channel.close()
    asyncio.run(run())


async def pygame_check(args):
    from rsim.adapters import pygame_window
    from rsim.runtime import Runtime
    from rsim.transport.descriptor import TransportConfig
    from mujocosim.apps.keyboard import run
    from mujocosim.integrations.navigation import NavigationClient
    pygame_window.run_window = scripted_window
    os.environ['MUJOCOSIM_PYGAME_SCREENSHOT'] = str(args.output/'pygame.png')
    client = NavigationClient(args.name, transport=TransportConfig('cyclonedds', args.domain))
    async with Runtime(client):
        state = await client.call('state')
        start = state['pose']
        # Turn in an already observed free footprint, rather than selecting a
        # new destination that might lie inside a furnished scene's obstacles.
        goal = [start[0], start[1], start[2]+math.pi]
        action = await client.go_to(*goal)
        window = asyncio.create_task(run(args.name, args.domain))
        states = []
        try:
            async with asyncio.timeout(25):
                while not window.done():
                    await asyncio.sleep(.1)
                    states.append(dict(client.last_state, observed_at_ns=time.monotonic_ns()))
                await window
            result = await client.call('result', action_id=action)
            await asyncio.sleep(.4)
            final = await client.call('state')
            report = dict(client_pid=os.getpid(), physics_pid=state['server_pid'], goal_result=result,
                          start=start, final=final, states=states,
                          input_method='SDL events posted inside the real pygame window process')
            (args.output/'pygame.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
            assert result['status'] == 'preempted', result
            assert any(s['status'] == 'manual' and abs(s['command'][0]) > .01 for s in states)
            assert any(s['status'] == 'manual' and abs(s['command'][1]) > .01 for s in states)
            assert final['command'] == [0., 0.] and final['goal'] is None, final
            events = json.loads((args.output/'pygame.events.json').read_text(encoding='utf-8'))
            times = {x['event']: x['at_ns'] for x in events['events']}
            for stop, resume in [('w_up', 'a_down'), ('focus_lost', 'focus_gained')]:
                stopped = [s for s in states if times[stop]+300_000_000 <= s['observed_at_ns'] < times[resume]]
                assert stopped and all(s['command'] == [0., 0.] for s in stopped), stop
            import psutil
            assert not psutil.pid_exists(events['window_pid']), 'pygame process did not exit'
            print('pygame: movement, preemption, focus loss, Space and close passed', flush=True)
        finally:
            window.cancel()
            await asyncio.gather(window, return_exceptions=True)
            await client.cancel()


def notebook_check(args):
    import nbformat
    from nbclient import NotebookClient
    from jupyter_client import KernelManager
    # Select the currently running interpreter without registering/changing a
    # user-wide kernel. The kernel is genuinely independent of the launcher.
    manager = KernelManager(kernel_name='python3')
    manager.kernel_spec.argv = [sys.executable, '-m', 'ipykernel_launcher', '-f', '{connection_file}']
    notebook = nbformat.read(Path(__file__).with_name(args.notebook+'.ipynb'), as_version=4)
    environment = dict(os.environ, MUJOCOSIM_NAV_NAME=args.name, MUJOCOSIM_NAV_DOMAIN=str(args.domain),
                       MUJOCOSIM_NOTEBOOK_OUTPUT=str(args.output/'notebook'),
                       MUJOCOSIM_GOAL_TIMEOUT=str(args.goal_timeout))
    if args.target:
        environment['MUJOCOSIM_NAV_TARGET'] = json.dumps(args.target)
    if args.pixels:
        environment['MUJOCOSIM_NAV_PIXELS'] = json.dumps(list(zip(args.pixels[::2], args.pixels[1::2])))
    count = len(args.pixels)//2 if args.pixels else 2
    client = NotebookClient(notebook, km=manager, timeout=int(args.goal_timeout*count+60),
        resources={'metadata': {'path': str(Path(__file__).resolve().parents[1])}})
    try:
        # Passing a KernelManager makes nbclient leave cleanup to its caller.
        client.execute(env=environment)
    finally:
        nbformat.write(notebook, args.output/(args.notebook+'.executed.ipynb'))
        if manager.has_kernel:
            manager.shutdown_kernel(now=True)
    result = json.loads((args.output/'notebook/result.json').read_text(encoding='utf-8'))
    assert result['notebook_pid'] != result['physics_pid']
    summary = {key: result[key] for key in ('notebook_pid', 'physics_pid', 'elapsed_seconds')}
    if 'results' in result:
        summary.update(statuses=[item['status'] for item in result['results']],
                       nodes=len(result['topology']['nodes']), edges=len(result['topology']['edges']))
    else:
        summary['result'] = result['result']
    print('notebook:', summary, flush=True)


def check(args):
    root = Path(__file__).resolve().parents[1]
    args.output = args.output.resolve()/uuid.uuid4().hex[:8]
    args.output.mkdir(parents=True)
    args.name = 'navcheck_'+uuid.uuid4().hex[:12]
    flags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0
    command = [sys.executable, '-X', 'utf8', '-m', 'mujocosim.apps.navigation',
        '--scene', str(Path(args.scene).resolve()), '--name', args.name, '--domain', str(args.domain),
        '--headless', '--enable-motion', '--planner', args.planner, '--output', str(args.output)]
    print('Acceptance output:', args.output, flush=True)
    with (args.output/'launcher.log').open('w', encoding='utf-8') as log:
        server = subprocess.Popen(command, cwd=root, stdout=log, stderr=subprocess.STDOUT, creationflags=flags)
        try:
            ready = args.output/'ready.json'
            deadline = time.monotonic()+45
            while not ready.exists():
                if server.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError(f'service startup failed: {args.output / "launcher.log"}')
                time.sleep(.1)
            notebook_check(args)
            if args.pygame:
                asyncio.run(pygame_check(args))
        finally:
            (args.output/'stop').write_text('stop\n', encoding='utf-8')
            try:
                server.wait(timeout=30)
            except subprocess.TimeoutExpired:
                server.terminate()
                server.wait(timeout=10)
        assert server.returncode == 0, 'see launcher.log/world.log'
    status = json.loads(ready.read_text(encoding='utf-8'))
    import psutil
    # The shared supervisor finishes export cleanup after the final lease ACK.
    deadline = time.monotonic()+15
    while psutil.pid_exists(status['physics_pid']) and time.monotonic() < deadline:
        time.sleep(.1)
    assert not psutil.pid_exists(status['physics_pid']), 'physics worker did not exit'
    status.update(passed=True, processes_cleaned=True, pygame=args.pygame)
    (args.output/'acceptance.json').write_text(json.dumps(status, indent=2), encoding='utf-8')


def main(argv=None):
    from mujocosim.deployment import DEFAULT_SCENE
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', default=DEFAULT_SCENE)
    parser.add_argument('--output', type=Path, default=Path('assets/runs/navigation_checks'))
    parser.add_argument('--domain', type=int, default=44)
    parser.add_argument('--planner', choices=['rrt_connect', 'voronoi'], default='rrt_connect')
    parser.add_argument('--target', nargs=3, type=float)
    parser.add_argument('--notebook', choices=('navigation', 'pixel_navigation'), default='navigation')
    parser.add_argument('--pixels', nargs='+', type=float, help='Pixel pairs u1 v1 u2 v2 ... for pixel_navigation')
    parser.add_argument('--goal-timeout', type=float, default=180.)
    parser.add_argument('--pygame', action='store_true')
    args = parser.parse_args(argv)
    if not math.isfinite(args.goal_timeout) or args.goal_timeout <= 0:
        parser.error('--goal-timeout must be finite and positive')
    if args.pixels and (len(args.pixels) % 2 or not all(math.isfinite(x) for x in args.pixels)):
        parser.error('--pixels requires finite u/v pairs')
    check(args)


if __name__ == '__main__':
    main()
