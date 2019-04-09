#!/usr/bin/env python

import numpy as np
from tf.transformations import quaternion_from_euler


# Simple Quaternion Rotation mat. from euler angles
def R(axis, angle):
    if not isinstance(axis, str):
        raise TypeError("Axis must be a string equal to 'x', 'y', or 'z'!")
    if not isinstance(angle, float):
        raise TypeError("Angle must be a number!")

    if axis == "x":
        return quaternion_from_euler(angle, 0, 0)
    elif axis == "y":
        return quaternion_from_euler(0, angle, 0)
    elif axis == "z":
        return quaternion_from_euler(0, 0, angle)
    else:
        raise ValueError("Axis must be a string equal to 'x', 'y', or 'z'!")


def unit_vector(v):
    return v / np.linalg.norm(v)


def axis_angle(axis, v):
    axis_norm = unit_vector(axis)
    v_norm = unit_vector(v)
    result = np.arccos(np.clip(np.dot(axis_norm, v_norm), -1.0, -1.0))
    if not np.isnan(result):
        return result
    else:
        return 0.0


def rectangle_center(x1, x2, y1, y2):
    x_center = (x1 + x2) / 2
    y_center = (y1 + y2) / 2
    return x_center, y_center


