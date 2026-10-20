"""
Unit tests for the standalone python_balancer package.
"""

from python_balancer.model import WeaponGeometry
from python_balancer.optimizer import balance_for_bite, pareto_bite_vs_inertia


def test_weapon_geometry_option_b():
    geom = WeaponGeometry(dia_bore=22.0)
    part = geom.build()
    assert part is not None
    assert part.volume > 0

    # Test Option B hub specs (Max R = 63 mm)
    assert geom.r_tooth == 63.0
    assert geom.dia_bore == 22.0
    assert (geom.dia_relief - geom.dia_bore) / 2.0 == 14.0  # 14 mm solid wall
    assert abs(geom.bite_h - 18.0) < 1e-4  # 63.0 - 45.0
    assert geom.gullet_depth == 38.0  # 63.0 - 25.0

    # Test bearing spacer creation
    spacer = geom.build_bearing_spacer(bearing_width=7.0, shaft_dia=8.0)
    assert spacer is not None
    assert spacer.volume > 0


def test_balance_for_bite_with_mass_target():
    geom = WeaponGeometry(dia_bore=22.0)
    balanced, report = balance_for_bite(geom, target_bite_h=18.0, target_mass_g=1500.0)
    assert report["converged"]
    assert report["com_offset_mm"] < 1e-4
    assert abs(report["mass_g"] - 1500.0) < 1.0  # Exactly 1.5 kg
    assert balanced.thickness > 30.0  # Sized for 1.5 kg at R=63mm


def test_pareto_bite_vs_inertia():
    geom = WeaponGeometry(dia_bore=22.0)
    frontier = pareto_bite_vs_inertia(geom, bite_values=[16.0, 20.0], target_mass_g=1500.0)
    assert len(frontier) == 2
    assert all(pt["balanced"] for pt in frontier)
    assert all(pt["izz_kg_m2"] > 0 for pt in frontier)
    assert all(abs(pt["mass_g"] - 1500.0) < 1.0 for pt in frontier)
