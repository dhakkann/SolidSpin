"""
Combat Robotics CAD Balancer & Kinetic Optimizer.
Centered strictly on:
1. Spin Axis Inertia (Izz)
2. Bite h
3. Tooth & Gullet Shape
"""

from .model import WeaponGeometry
from .optimizer import balance_for_bite, pareto_bite_vs_inertia

__all__ = ["WeaponGeometry", "balance_for_bite", "pareto_bite_vs_inertia"]
