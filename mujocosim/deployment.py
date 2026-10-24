"""External simulation data and lightweight project output defaults."""
from datetime import datetime, timezone
import os
from pathlib import Path, PureWindowsPath
from uuid import uuid4

def data_root():
    default = 'D:/Storage/Datasets/mujoco' if os.name == 'nt' else Path.home() / 'Storage/Datasets/mujoco'
    value = os.environ.get('MUJOCO_SCENE_ROOT') or str(default)
    if os.name != 'nt' and PureWindowsPath(value).drive:
        raise ValueError('MUJOCO_SCENE_ROOT must use a Linux path, for example /mnt/d/Storage/Datasets/mujoco')
    return Path(value).expanduser().resolve()


DATA_ROOT = data_root()
DEFAULT_SCENE = str(DATA_ROOT / 'scenes/4ok3usBNeis_furnished.scene.json')
DEFAULT_CACHE = str(DATA_ROOT / 'cache')
MAP_ROOT = DATA_ROOT / 'maps'
DEFAULT_OUTPUT = 'assets/runs/latest'


def new_map_directory(application):
    """Choose an independent map run directory without creating it yet."""
    run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid4().hex[:8]
    return MAP_ROOT / application / run_id
