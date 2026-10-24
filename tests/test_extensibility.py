"""Engine extension contracts, independent of the differential-drive preset."""
import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from mujocosim import Simulation
from mujocosim.scenes import prepare_scene
from test_simulation import make_room


class RoomBuilder:
    def prepare(self, cache_root, **options):
        directory = Path(cache_root) / 'custom-room'
        directory.mkdir(parents=True)
        make_room(directory)
        return directory


class Slider:
    """A different topology, command and joint layout; no wheels or freejoint."""
    clearance_radius = .15

    def build(self, root):
        body = ET.SubElement(root.find('worldbody'), 'body', name='slider', pos='0 0 .2')
        ET.SubElement(body, 'joint', name='rail', type='slide', axis='1 0 0', damping='1')
        ET.SubElement(body, 'geom', type='box', size='.1 .1 .1', mass='1')
        actuator = ET.SubElement(root, 'actuator')
        ET.SubElement(actuator, 'motor', name='force', joint='rail')
        return body

    def bind(self, model, data):
        class State:
            body_id = model.body('slider').id
            def reset(self, spawn): data.qpos[model.joint('rail').qposadr[0]] = spawn[0]
            def after_reset(self): pass
            def apply(self, force): data.ctrl[model.actuator('force').id] = float(force)
            def stop(self): data.ctrl[:] = 0
            def after_step(self): pass
            def telemetry(self): return {'displacement': float(data.qpos[0])}
        return State()


def test_custom_scene_and_robot_without_chassis_or_sensor_assumptions(tmp_path):
    directory = prepare_scene(RoomBuilder(), tmp_path)
    with Simulation(directory, robot=Slider(), spawn=[0, 0, 0]) as sim:
        assert sim.model.nq == sim.model.nu == 1
        assert sim.sensor_suite is None
        sim.command(1.)
        sim.step(.5)
        assert sim.robot.telemetry()['displacement'] > .05
        assert sim.world_from_base[0, 3] > .05
        sim.reset()
        assert sim.robot.telemetry()['displacement'] == pytest.approx(0)


def test_core_packages_do_not_depend_on_rsim():
    root = Path(__file__).resolve().parents[1] / 'mujocosim'
    for path in root.rglob('*.py'):
        if path.relative_to(root).parts[0] in ('integrations', 'apps', 'components'):
            continue
        for node in ast.walk(ast.parse(path.read_text(encoding='utf8'))):
            names = [x.name for x in node.names] if isinstance(node, ast.Import) else [node.module or ''] if isinstance(node, ast.ImportFrom) else []
            assert not any(name.split('.')[0] == 'rsim' for name in names), path
