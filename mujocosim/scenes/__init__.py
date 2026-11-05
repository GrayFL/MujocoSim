from .prepare import prepare_scene, SceneBuilder

__all__ = ["prepare_scene", "SceneBuilder", "CACHE_VERSION", "rectangles", "write_mesh_scene"]


def __getattr__(name):
    if name in ('CACHE_VERSION', 'rectangles', 'write_mesh_scene'):
        from . import mesh
        return getattr(mesh, name)
    raise AttributeError(name)
