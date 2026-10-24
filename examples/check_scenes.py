"""Verify both built scenes and save RGBD, cutaway views and rigid-body checks."""
from __future__ import annotations
from mujocosim.robots import DifferentialDrive, DriveCommand

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from mujocosim import Simulation, SensorConfig
from mujocosim.deployment import DEFAULT_CACHE


def check(name, directory, output):
    output.mkdir(parents=True, exist_ok=True)
    with Simulation(directory, robot=DifferentialDrive(), sensors=SensorConfig(width=960, height=720)) as sim:
        objects = sim.metadata["objects"]
        initial = {item["name"]: sim.data.body(item["name"]).xpos.copy() for item in objects}
        sim.step(5)
        results = {}
        for item in objects:
            body = sim.model.body(item["name"])
            xyz = sim.data.body(item["name"]).xpos.copy()
            drift = float(np.linalg.norm(xyz-initial[item["name"]]))
            upright = float(sim.data.body(item["name"]).xmat.reshape(3, 3)[2, 2])
            results[item["name"]] = dict(mass=float(body.mass[0]), dimensions=item["extents"],
                                         scale=item["applied_scale"], position=xyz.tolist(),
                                         drift_5s=drift, upright_cosine=upright,
                                         triangles=item["visual_triangles"], collision_parts=item["collision_parts"])
            assert drift < .04 and upright > .98, (name, item["name"], results[item["name"]])
            assert abs(xyz[2]) < .025
        for item in objects:
            sim.reset()
            body = sim.model.body(item["name"])
            q = int(sim.model.joint(item["name"]+"_free").qposadr[0])
            sim.data.qpos[q+2] += .25
            sim.step(.1)
            height = float(sim.data.body(item["name"]).xpos[2])
            sim.step(1.5)
            rest_height = float(sim.data.body(item["name"]).xpos[2])
            assert .15 < height < .25 and abs(rest_height) < .03
            sim.reset()
            start = sim.data.body(item["name"]).xpos.copy()
            # Horizontal push at COM verifies a dynamic, frictional body.
            sim.data.xfrc_applied[body.id, 0] = float(body.mass[0])*12
            sim.step(.25)
            sim.data.xfrc_applied[:] = 0
            sim.step(.25)
            displacement = float(np.linalg.norm(sim.data.body(item["name"]).xpos[:2]-start[:2]))
            assert displacement > .02
            results[item["name"]].update(drop_height_after_0_1s=height,
                                          drop_rest_height=rest_height, push_distance=displacement)
        sim.reset()
        table_start = sim.data.body("table").xpos.copy()
        robot_root = sim.model.body("base_footprint").id
        table_root = sim.model.body("table").id
        contact_seen = False
        sim.command(DriveCommand(.35, 0))
        for _ in range(400):
            sim.step(.02)
            for contact in sim.data.contact:
                roots = sim.model.body_rootid[sim.model.geom_bodyid[[contact.geom1, contact.geom2]]]
                if set(roots) == {robot_root, table_root}:
                    contact_seen = True
        sim.command(DriveCommand(0, 0))
        table_push = float(np.linalg.norm(sim.data.body("table").xpos[:2]-table_start[:2]))
        assert contact_seen, f"{name}: robot never contacted the table"
        assert np.isfinite(sim.data.qpos).all()
        sim.reset()
        scan, frame = sim.save_snapshot(output)
        center = np.mean([item["position"] for item in objects], axis=0)
        with mujoco.Renderer(sim.model, width=960, height=720) as renderer:
            options = mujoco.MjvOption()
            options.geomgroup[:] = [1, 1, 0, 0, 0, 0]
            options.sitegroup[:] = 0
            camera = mujoco.MjvCamera()
            camera.lookat[:] = [center[0], center[1]-.6, .4]
            camera.distance = 5.8
            camera.azimuth = -75
            camera.elevation = -35
            if name == "simple_indoor":
                camera.azimuth = 105
                camera.elevation = -55
                camera.distance = 7
            renderer.update_scene(sim.data, camera=camera, scene_option=options)
            Image.fromarray(renderer.render()).save(output / "scene.png")
            camera.elevation = -90
            camera.azimuth = -90
            camera.distance = 7.5
            renderer.update_scene(sim.data, camera=camera, scene_option=options)
            Image.fromarray(renderer.render()).save(output / "top.png")
            camera.distance = 4
            camera.elevation = -25
            camera.azimuth = -65
            options.geomgroup[:] = [0, 1, 0, 1, 0, 0]
            renderer.update_scene(sim.data, camera=camera, scene_option=options)
            Image.fromarray(renderer.render()).save(output / "collision.png")
        report = dict(scene=name, prepared=str(directory), objects=results, spawn=sim.pose2d.tolist(),
                      robot_table_contact=contact_seen, robot_pushed_table_distance=table_push,
                      contacts=int(sim.data.ncon), lidar_valid=int(np.isfinite(scan.ranges).sum()),
                      depth_valid_fraction=float(np.isfinite(frame.depth).mean()))
        (output / "physics.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--index", type=Path, default=Path(DEFAULT_CACHE) / "scene_index.json")
    parser.add_argument("--output", type=Path, default=Path("assets/runs/scene_checks"))
    args = parser.parse_args(argv)
    for name, scene in json.loads(args.index.read_text(encoding="utf-8")).items():
        check(name, args.index.resolve().parent / scene["prepared"], args.output / name)


if __name__ == "__main__":
    main()
