"""Run with python -m examples.python_api --scene path/to/scene.glb."""
from mujocosim.apps.motion import exercise
from mujocosim.apps.motion import main as run
from mujocosim.deployment import DEFAULT_SCENE, DEFAULT_CACHE


def main(argv=None):
    return run(argv, default_scene=DEFAULT_SCENE, default_cache=DEFAULT_CACHE,
               default_output="assets/runs/python_api")


if __name__ == "__main__":
    main()
