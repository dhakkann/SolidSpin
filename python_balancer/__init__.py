"""
Python-native CAD Balancer & Kinetic Optimizer for Combat Robotics.
Powered by build123d and pure-Python numerical solvers.
"""

from .model import WeaponGeometry
from .optimizer import balance_weapon, pareto_frontier

__all__ = ["WeaponGeometry", "balance_weapon", "pareto_frontier"]
