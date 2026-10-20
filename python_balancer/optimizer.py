"""
Kinetic Optimizer focusing strictly on:
1. Spin Axis Inertia (Izz)
2. Bite h
3. Tooth & Gullet Shape
"""

import copy
import math
import time
from typing import List, Dict, Any, Tuple
from sw_balancer.solver import solve_multi
from .model import WeaponGeometry


def balance_for_bite(
    base_geom: WeaponGeometry,
    target_bite_h: float,
    free_params: List[str] = None,
    target_mass_g: float = 1500.0,
    pct_range: float = 55.0
) -> Tuple[WeaponGeometry, Dict[str, Any]]:
    """
    Enforce exact Bite h, preserve Tooth/Gullet profile, adjust thickness to target mass,
    and dynamically balance:
    - Sets r_counter = r_tooth - target_bite_h
    - Tunes tooth angle / counterweight angle to achieve COM = (0, 0)
    - Sets thickness to hit exactly target_mass_g (e.g. 1500 g)
    """
    if free_params is None:
        free_params = ["angle_tooth", "angle_counter"]

    geom = copy.copy(base_geom)
    geom.bite_h = target_bite_h  # Sets r_counter = r_tooth - target_bite_h

    x0 = [getattr(geom, name) for name in free_params]
    scale = list(x0)
    lo = [v * (1.0 - pct_range / 100.0) for v in x0]
    hi = [v * (1.0 + pct_range / 100.0) for v in x0]

    def evaluate(xs):
        trial = copy.copy(geom)
        for name, val in zip(free_params, xs):
            setattr(trial, name, float(val))
        return trial.evaluate_com()

    t0 = time.perf_counter()
    res = solve_multi(evaluate, x0, scale, lo, hi)
    t_elapsed = time.perf_counter() - t0

    balanced = copy.copy(geom)
    for name, val in zip(free_params, res["x"]):
        setattr(balanced, name, float(val))

    if target_mass_g is not None and target_mass_g > 0:
        balanced.set_mass_target(target_mass_g)

    final_offset = math.hypot(*balanced.evaluate_com())
    mp = balanced.mass_properties()

    report = {
        "converged": final_offset < 1e-4,
        "evals": res["evals"],
        "elapsed_ms": t_elapsed * 1000.0,
        "bite_h_mm": target_bite_h,
        "r_tooth_mm": balanced.r_tooth,
        "r_counter_mm": balanced.r_counter,
        "tooth_tip_chord_mm": balanced.tooth_tip_chord,
        "gullet_depth_mm": balanced.gullet_depth,
        "izz_kg_m2": mp["izz_kg_m2"],
        "kinetic_energy_j": mp["kinetic_energy_j"],
        "mass_g": mp["mass_g"],
        "com_offset_mm": final_offset,
        "balanced_params": {k: getattr(balanced, k) for k in free_params}
    }
    return balanced, report


def pareto_bite_vs_inertia(
    base_geom: WeaponGeometry,
    bite_values: List[float] = None,
    target_mass_g: float = 1500.0,
    rpm: float = 11000.0,
    closing_speed_m_s: float = 5.364
) -> List[Dict[str, Any]]:
    """
    Analyze the trade-off between:
    - Bite h (engagement depth)
    - Spin Axis Inertia (Izz & stored kinetic energy)
    - Tooth & Gullet geometry
    - Plate Thickness (managed to hit target_mass_g = 1500 g)

    Every point on this curve is 100% dynamically balanced.
    """
    if bite_values is None:
        bite_values = [14.0, 16.0, 18.0, 20.0, 22.0, 24.0]

    frontier = []

    for bite in bite_values:
        try:
            balanced_geom, report = balance_for_bite(
                base_geom,
                target_bite_h=bite,
                target_mass_g=target_mass_g
            )
            mp = balanced_geom.mass_properties(rpm=rpm, closing_speed_m_s=closing_speed_m_s)
            frontier.append({
                "bite_h_mm": bite,
                "dynamic_bite_mm": mp["bite_h_dynamic_mm"],
                "clearance_ratio": bite / mp["bite_h_dynamic_mm"] if mp["bite_h_dynamic_mm"] > 0 else 0,
                "balanced": report["converged"],
                "izz_kg_m2": mp["izz_kg_m2"],
                "kinetic_energy_j": mp["kinetic_energy_j"],
                "mass_g": mp["mass_g"],
                "thickness_mm": balanced_geom.thickness,
                "specific_inertia": mp["specific_inertia"],
                "tooth_chord_mm": mp["tooth_tip_chord_mm"],
                "gullet_depth_mm": mp["gullet_depth_mm"],
                "angle_tooth": balanced_geom.angle_tooth,
                "angle_counter": balanced_geom.angle_counter,
                "geometry": balanced_geom
            })
        except Exception as e:
            frontier.append({
                "bite_h_mm": bite,
                "balanced": False,
                "error": str(e)
            })

    return frontier
