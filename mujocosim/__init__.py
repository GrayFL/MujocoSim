"""Composable MuJoCo models, scenes and sensors. RSim is an optional integration."""
from importlib import import_module
from .config import PhysicsConfig, SensorConfig

__all__ = ['Simulation', 'PhysicsConfig', 'SensorConfig', 'prepare_scene']

def __getattr__(name):
    modules = {'Simulation':'simulation', 'prepare_scene':'scenes'}
    if name not in modules: raise AttributeError(name)
    return getattr(import_module(f'{__name__}.{modules[name]}'), name)
