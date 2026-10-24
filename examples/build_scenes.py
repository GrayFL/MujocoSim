"""Build scenes from recipes and optional templates in the external dataset."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil

from mujocosim.deployment import DATA_ROOT
from mujocosim.scenes import prepare_scene


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-root", type=Path, default=DATA_ROOT)
    parser.add_argument("--hm3d", type=Path, help="Copy this GLB to the dataset's hm3d/4ok3usBNeis.glb if missing")
    parser.add_argument("--cache-dir", type=Path, help="Defaults to <scene-root>/cache")
    parser.add_argument("--only", choices=["simple_indoor", "4ok3usBNeis_furnished"])
    parser.add_argument("--refresh-recipes", action="store_true", help="Replace recipes with <scene-root>/templates/*.scene.json")
    parser.add_argument("--rebuild", action="store_true")
    args = parser.parse_args(argv)
    scene_root = args.scene_root.expanduser().resolve()
    cache_dir = (args.cache_dir or scene_root / "cache").expanduser().resolve()
    templates = scene_root / "templates"
    destination = scene_root / "scenes"
    names = {p.name.removesuffix(".scene.json") for folder in (templates, destination)
             for p in folder.glob("*.scene.json")}
    selected = [args.only] if args.only else sorted(names)
    if not selected or any(name not in names for name in selected):
        parser.error(f"Missing scene recipes: place *.scene.json in {destination} or {templates}")
    if args.refresh_recipes:
        for name in selected:
            if not (templates / (name + ".scene.json")).is_file():
                parser.error(f"Missing template: {templates / (name + '.scene.json')}")
    for name in ("chair.glb", "table.glb"):
        if not (scene_root / "test" / name).is_file():
            parser.error(f"Missing object: {scene_root / 'test' / name}")
    hm3d = scene_root / "hm3d/4ok3usBNeis.glb"
    if "4ok3usBNeis_furnished" in selected and not hm3d.exists():
        if args.hm3d is None or not args.hm3d.is_file():
            parser.error(f"Place HM3D at {hm3d}, or pass --hm3d SOURCE.glb")
        hm3d.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(args.hm3d, hm3d)
    destination.mkdir(parents=True, exist_ok=True)
    index = cache_dir / "scene_index.json"
    built = json.loads(index.read_text(encoding="utf-8")) if args.only and index.exists() else {}
    for name in selected:
        template = templates / (name + ".scene.json")
        recipe = destination / template.name
        if not recipe.exists() or args.refresh_recipes:
            shutil.copy2(template, recipe)
        built[name] = dict(recipe=str(recipe), prepared=str(prepare_scene(recipe, cache_dir, force=args.rebuild).relative_to(cache_dir)))
    index.write_text(json.dumps(built, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(built, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
