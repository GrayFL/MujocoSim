"""Small NumPy coordinate helpers, independent of application protocols."""
import numpy as np

def numbers(values):
    return " ".join(f"{float(x):.8g}" for x in values)

def transform(position, rotation):
    result = np.eye(4)
    result[:3, :3] = np.asarray(rotation).reshape(3, 3)
    result[:3, 3] = position
    return result
