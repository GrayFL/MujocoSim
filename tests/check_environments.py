"""E02/E03/F07: explicit provider and extension-client interpreter acceptance."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid


def run(args):
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    name = 'env_' + uuid.uuid4().hex[:12]
    provider = Path(args.provider_python).absolute()
    client = Path(args.client_python).absolute()
    env = dict(os.environ, MUJOCO_GL='egl')
    env.pop('PYTHONPATH', None)
    for executable, absent in ((provider, 'pyrobo'), (client, 'mujoco')):
        subprocess.run([str(executable), '-c',
                        f'import importlib.util; assert importlib.util.find_spec({absent!r}) is None'],
                       cwd=output, env=env, check=True)
    with tempfile.TemporaryDirectory() as temporary:
        temporary = Path(temporary)
        scene = temporary/'room.scene.json'
        scene.write_text(json.dumps(dict(format='mujocosim-scene-v1',
                            room=dict(size=[6, 6, 2.6]), spawn=[0, 0, 0], objects=[])))
        ready = temporary/'ready.json'
        command = [str(provider), '-m', 'mujocosim.apps.service', '--scene', str(scene),
                   '--cache-dir', str(temporary/'cache'), '--name', name,
                   '--domain', str(args.domain), '--width', '80', '--height', '60',
                   '--scheduling', args.scheduling,
                   '--log', str(output/'worker.log'), '--ready-file', str(ready)]
        with (output/'provider.log').open('w') as stream:
            process = subprocess.Popen(command, cwd=output, env=env, stdout=stream, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 30
                while not ready.exists():
                    if process.poll() is not None:
                        raise RuntimeError(f'provider exited {process.returncode}; see provider.log')
                    if time.monotonic() >= deadline:
                        raise TimeoutError('provider did not become ready')
                    time.sleep(.05)
                result = subprocess.run([str(client), str(Path(__file__).with_name('environment_client.py')),
                                         '--name', name, '--domain', str(args.domain)],
                                        cwd=output, env=env, capture_output=True, text=True, timeout=30)
                (output/'client.log').write_text(result.stdout + result.stderr)
                result.check_returncode()
                evidence = json.loads(result.stdout)
                if evidence['worker_command'] is not None:
                    assert evidence['worker_command'][0] == str(provider)
                evidence['provider'] = json.loads(ready.read_text())
                evidence['provider_command'] = command
                (output/'result.json').write_text(json.dumps(evidence, indent=2))
                print(json.dumps(evidence, indent=2))
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                    raise RuntimeError('provider did not shut down after SIGINT')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--provider-python', required=True)
    parser.add_argument('--client-python', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--domain', type=int, default=62)
    parser.add_argument('--scheduling', choices=('realtime', 'accelerated', 'controlled'), default='realtime')
    run(parser.parse_args())
