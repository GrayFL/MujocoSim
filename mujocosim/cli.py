"""Default demo entry; all application assembly lives in this project."""
from .deployment import DEFAULT_SCENE, DEFAULT_CACHE, DEFAULT_OUTPUT

def main(argv=None):
    from .apps.demo import main as run
    return run(argv, default_scene=DEFAULT_SCENE, default_cache=DEFAULT_CACHE, default_output=DEFAULT_OUTPUT)

def parser():
    from .apps.demo import parser as make_parser
    return make_parser(DEFAULT_SCENE, DEFAULT_CACHE, DEFAULT_OUTPUT)
