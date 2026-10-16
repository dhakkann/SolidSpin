"""
Geometry and Coordinate System utilities for SolidWorks Weapon Balancer.
Ported from Fusion 360 Asymmetric Weapon Balancer.
Units: Coordinates in meters, converted to mm for offset readouts.
"""

import math

M_TO_MM = 1000.0


def _v(p):
    """Ensure tuple format (x, y, z)."""
    if hasattr(p, 'x') and hasattr(p, 'y') and hasattr(p, 'z'):
        return (p.x, p.y, p.z)
    return (p[0], p[1], p[2])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    )


def _unit(a):
    length = math.sqrt(_dot(a, a))
    if length == 0.0:
        raise RuntimeError("Vector has zero length.")
    return (a[0] / length, a[1] / length, a[2] / length)


def _axis_name(v, fallback):
    """'X', 'Y' or 'Z' if v lies along a principal axis, else fallback."""
    i = max(range(3), key=lambda k: abs(v[k]))
    sign = 1.0 if v[i] >= 0.0 else -1.0
    v = (v[0] * sign, v[1] * sign, v[2] * sign)
    return v, ("XYZ"[i] if abs(v[i]) > 0.9999 else fallback)


class SpinAxis:
    """Spin axis in model space (meters) with a 2D orthonormal basis for the plane normal to it.

    The two in-plane directions follow the model axes where possible so offsets
    read intuitively (e.g. spinning about Z gives in-plane X and Y).
    """

    def __init__(self, point, direction, label="Spin Axis"):
        self.p = _v(point)
        self.n = _unit(_v(direction))

        # Model axis most perpendicular to n
        k = min(range(3), key=lambda i: abs(self.n[i]))
        ref = [0.0, 0.0, 0.0]
        ref[k] = 1.0

        # Project ref onto plane perpendicular to n
        proj = _sub(ref, tuple(self.n[i] * self.n[k] for i in range(3)))
        e1 = _unit(proj)
        self.e1, name1 = _axis_name(e1, "U")
        self.e2, name2 = _axis_name(_cross(self.n, self.e1), "V")

        # Standardize order: X-Y, X-Z, Y-Z
        if name1 in "XYZ" and name2 in "XYZ" and name1 > name2:
            self.e1, self.e2, name1, name2 = self.e2, self.e1, name2, name1

        self.names = (name1, name2)
        self.n_name = _axis_name(self.n, "axis")[1]
        self.label = label

    def offset_mm(self, com):
        """In-plane offset of Center of Mass from the axis, (a, b) in mm."""
        d = _sub(com, self.p)
        return (_dot(d, self.e1) * M_TO_MM, _dot(d, self.e2) * M_TO_MM)

    def along_mm(self, com):
        """Position along the spin axis in mm (does not affect balance)."""
        return _dot(_sub(com, self.p), self.n) * M_TO_MM

    def describe(self, ab, digits=4):
        return f"{self.names[0]} {ab[0]:+.{digits}f}   {self.names[1]} {ab[1]:+.{digits}f} mm"
