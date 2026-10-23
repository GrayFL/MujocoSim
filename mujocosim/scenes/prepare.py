"""Scene builders produce one prepared directory; the engine ignores provenance."""
from pathlib import Path
from typing import Protocol, runtime_checkable


@runtime_checkable
class SceneBuilder(Protocol):
    def prepare(self, cache_root, **options) -> Path: ...


def prepare_scene(source, cache_root="assets/cache", **options):
    if isinstance(source, SceneBuilder):
        return prepare_scene(source.prepare(cache_root, **options), cache_root)
    source = Path(source).expanduser().resolve()
    if source.is_dir():
        for name in ("scene.xml", "metadata.json", "navigation.npz"):
            if not (source / name).is_file():
                raise FileNotFoundError(source / name)
        return source
    if source.suffix.lower() == ".json":
        from .composition import prepare_composition
        return prepare_composition(source, cache_root, force=options.get("force", False))
    if source.suffix.lower() in (".glb", ".gltf"):
        from .mesh import prepare_mesh
        return prepare_mesh(source, cache_root, **options)
    raise ValueError("Expected GLB/GLTF, composition JSON, prepared directory, or SceneBuilder")
