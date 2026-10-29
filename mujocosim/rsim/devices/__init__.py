"""Connection-only views; no engine import or implicit provider startup."""
from .simulation import SimulationDevice

__all__ = ['SimulationDevice']
