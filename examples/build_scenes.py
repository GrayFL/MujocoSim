"""Install the portable recipes into a dataset root and build both scenes."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil

from mujocosim.deployment import SCENE_ROOT, DEFAULT_CACHE
from mujocosim.scenes import prepare_scene


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-root", type=Path, default=SCENE_ROOT)
    parser.add_argument("--hm3d", type=Path, help="Copy this GLB to SCENE_ROOT/hm3d/4ok3usBNeis.glb if missing")
    parser.add_argument("--cache-dir", default=DEFAULT_CACHE)
    parser.add_argument("--only", choices=["simple_indoor", "4ok3usBNeis_furnished"])
    parser.add_argument("--refresh-recipes", action="store_true", help="Replace installed recipes with repository templates")
    parser.add_argument("--rebuild", action="store_true")
    args = parser.parse_args(argv)
    scene_root = args.scene_root.expanduser().resolve()
    for name in ("chair.glb", "table.glb"):
        if not (scene_root / "test" / name).is_file():
            parser.error(f"Missing object: {scene_root / 'test' / name}")
    hm3d = scene_root / "hm3d/4ok3usBNeis.glb"
    if args.only != "simple_indoor" and not hm3d.exists():
        if args.hm3d is None or not args.hm3d.is_file():
            parser.error(f"Place HM3D at {hm3d}, or pass --hm3d SOURCE.glb")
        hm3d.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(args.hm3d, hm3d)
    templates = Path(__file__).resolve().parents[1] / "assets/templates"
    destination = scene_root / "scenes"
    destination.mkdir(parents=True, exist_ok=True)
    built = {}
    for template in sorted(templates.glob("*.scene.json")):
        name = template.name.removesuffix(".scene.json")
        if args.only and name != args.only:
            continue
        recipe = destination / template.name
        if not recipe.exists() or args.refresh_recipes:
            shutil.copy2(template, recipe)
        built[name] = dict(recipe=str(recipe), prepared=str(prepare_scene(recipe, args.cache_dir, force=args.rebuild).relative_to(Path(args.cache_dir).resolve())))
    index = Path(args.cache_dir).resolve() / "scene_index.json"
    index.write_text(json.dumps(built, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(built, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
