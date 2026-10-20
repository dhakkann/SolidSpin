"""
Unit tests for the standalone python_balancer package.
"""

from python_balancer.model import WeaponGeometry
from python_balancer.optimizer import balance_for_bite, pareto_bite_vs_inertia


def test_weapon_geometry_pillars():
    geom = WeaponGeometry()
    part = geom.build()
    assert part is not None
    assert part.volume > 0

    # Test Bite h
    assert geom.bite_h == 18.0  # 63.0 - 45.0
    assert geom.gullet_depth == 38.0  # 63.0 - 25.0
    assert geom.tooth_tip_chord > 0

    mp = geom.mass_properties(rpm=8000.0)
    assert mp["izz_kg_m2"] > 0
    assert mp["kinetic_energy_j"] > 0
    assert mp["bite_h_physical_mm"] == 18.0


def test_balance_for_bite():
    geom = WeaponGeometry()
    balanced, report = balance_for_bite(geom, target_bite_h=20.0)
    assert report["converged"]
    assert report["com_offset_mm"] < 1e-4
    assert balanced.bite_h == 20.0
    assert report["elapsed_ms"] < 3000.0  # Sub-second to couple seconds


def test_pareto_bite_vs_inertia():
    geom = WeaponGeometry()
    frontier = pareto_bite_vs_inertia(geom, bite_values=[15.0, 20.0])
    assert len(frontier) == 2
    assert all(pt["balanced"] for pt in frontier)
    assert all(pt["izz_kg_m2"] > 0 for pt in frontier)
