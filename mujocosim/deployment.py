"""Local defaults, overridable through paths and MUJOCO_SCENE_ROOT."""
import os
from pathlib import Path
SCENE_ROOT = Path(os.environ.get('MUJOCO_SCENE_ROOT', 'D:/Storage/Datasets/mujoco'))
DEFAULT_SCENE = str(SCENE_ROOT / 'scenes/4ok3usBNeis_furnished.scene.json')
DEFAULT_CACHE = 'assets/cache'
DEFAULT_OUTPUT = 'assets/runs/latest'
