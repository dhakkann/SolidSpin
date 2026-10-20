"""
Unit tests for the standalone python_balancer package.
"""

from python_balancer.model import WeaponGeometry
from python_balancer.optimizer import balance_weapon, pareto_frontier


def test_weapon_geometry_build():
    geom = WeaponGeometry()
    part = geom.build()
    assert part is not None
    assert part.volume > 0

    mp = geom.mass_properties()
    assert mp["mass_g"] > 0
    assert mp["izz_kg_m2"] > 0
    assert abs(mp["com_x"]) < 1e-4  # Symmetric across X
    assert mp["com_offset_mm"] > 1.0  # Initially unbalanced


def test_balance_weapon():
    geom = WeaponGeometry()
    balanced, report = balance_weapon(geom, free_params=["r_top", "angle_top"])
    assert report["converged"]
    assert report["final_offset_mm"] < 1e-4
    assert report["elapsed_s"] < 5.0  # Must be fast


def test_pareto_frontier_small():
    geom = WeaponGeometry()
    frontier = pareto_frontier(geom, sweep_param="r_top", sweep_values=[55.0, 65.0])
    assert len(frontier) == 2
    assert all(pt["balanced"] for pt in frontier)
