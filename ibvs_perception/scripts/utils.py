#!/usr/bin/env python
"""Utilities for transform, lin. alg., and vector math.

May include stuff I personally use a lot as well like signal handlers.
"""

import numpy as np
import sys
import tf.transformations

X_AXIS = np.array([1, 0, 0])
Y_AXIS = np.array([0, 1, 0])
Z_AXIS = np.array([0, 0, 1])


def R(axis, angle):
    """Returns a (numpy) quaternion matrix for a euler rotation about an axis"""
    if not isinstance(axis, str):
        raise TypeError("Axis must be a string equal to 'x', 'y', or 'z'!")
    if not isinstance(angle, float):
        raise TypeError("Angle must be a number!")

    axis = axis.lower()  # Just do it before hand pls...
    if axis == "x":
        return tf.transformations.quaternion_from_euler(angle, 0, 0)
    elif axis == "y":
        return tf.transformations.quaternion_from_euler(0, angle, 0)
    elif axis == "z":
        return tf.transformations.quaternion_from_euler(0, 0, angle)
    else:
        raise ValueError("Axis must be a string equal to 'x', 'y', or 'z'!")


def unit_vector(v):
    """Does this need docs? Yes, please use a numpy array if possible."""
    return v / np.linalg.norm(v)


def axis_angle(axis, v):
    """Safely compute the angle between a vector and its axis."""
    axis_norm = unit_vector(axis)
    v_norm = unit_vector(v)
    result = np.arccos(np.clip(np.dot(axis_norm, v_norm), -1.0, -1.0))
    if not np.isnan(result):
        return result
    else:
        return 0.0


def rectangle_center(x1, x2, y1, y2, x_max=640, y_max=480):
    """Compute the center of a rectangle."""
    x_center = (x1 + x2) / 2
    y_center = (y1 + y2) / 2
    if x_center > x_max:
        return x_max, y_center
    if y_center > y_max:
        return x_center, y_max

    return x_center, y_center


def project_pixel_to_3d(u, v, c, f):
    """Use camera intrinsics to reconstruct 2D point into 3D unit vector."""
    # We want only lists or tuples
    is_list_or_tuple = isinstance(c, (list, tuple)) and (isinstance(f, (list, tuple)))
    is_correct_length = len(c) == 2 and len(f) == 2
    if not is_list_or_tuple:
        raise TypeError("Args c and f must be of type list or tuple!")
    if not is_correct_length:
        raise ValueError("Args c and f must be of length 2! (x, y)")

    x = (u - c[0]) / f[0]
    y = (v - c[1]) / f[1]
    norm = np.sqrt(x * x + y * y + 1)
    x /= norm
    y /= norm
    z = 1.0 / norm
    return np.array([x, y, z])


def signal_handler(sig, frame):
    """EXECUTE ORDER 66 !!! >:c"""
    print("Ctrl-c captured!")
    sys.exit(0)

