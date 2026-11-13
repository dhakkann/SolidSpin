"""
Kinetic Optimizer:
Uses a 1-DOF primary solve on `angle_counter` (preserving `angle_tooth` as a fixed design choice),
with an automatic fallback to joint `["angle_tooth", "angle_counter"]` for extreme bite depths
where counterweight geometry reaches its physical limit.
"""

import copy
import math
import time
from typing import List, Dict, Any, Tuple
from sw_balancer.solver import solve_multi
from .model import WeaponGeometry


def _solve_com(
    base_geom: WeaponGeometry,
    target_bite_h: float,
    free_params: List[str],
    target_mass_g: float = 1500.0,
    target_energy_j: float = 2000.0,
    pct_range: float = 65.0
) -> Tuple[WeaponGeometry, Dict[str, Any]]:
    geom = copy.copy(base_geom)
    geom.bite_h = target_bite_h

    x0 = [getattr(geom, name) for name in free_params]
    scale = list(x0)
    lo = [max(10.0, v * (1.0 - pct_range / 100.0)) for v in x0]
    hi = [min(240.0, v * (1.0 + pct_range / 100.0)) for v in x0]

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
    mp = balanced.mass_properties(target_energy_j=target_energy_j)

    report = {
        "converged": final_offset < 1e-3,
        "evals": res["evals"],
        "elapsed_ms": t_elapsed * 1000.0,
        "bite_h_mm": target_bite_h,
        "r_tooth_mm": balanced.r_tooth,
        "r_counter_mm": balanced.r_counter,
        "tooth_tip_chord_mm": balanced.tooth_tip_chord,
        "min_neck_width_mm": balanced.min_neck_width_mm,
        "actual_fillet_r_mm": balanced.actual_fillet_r,
        "gullet_depth_mm": balanced.gullet_depth,
        "izz_kg_m2": mp["izz_kg_m2"],
        "kinetic_energy_j": mp["kinetic_energy_j"],
        "required_rpm": mp["required_rpm"],
        "mass_g": mp["mass_g"],
        "com_offset_mm": final_offset,
        "balanced_params": {k: getattr(balanced, k) for k in free_params}
    }
    return balanced, report


def balance_for_bite(
    base_geom: WeaponGeometry,
    target_bite_h: float,
    free_params: List[str] = None,
    target_mass_g: float = 1500.0,
    target_energy_j: float = 2000.0,
    pct_range: float = 65.0
) -> Tuple[WeaponGeometry, Dict[str, Any]]:
    """
    Enforce exact physical Bite step h.
    First attempts 1-DOF solve on angle_counter (preserving angle_tooth).
    If angle_counter hits physical limit, falls back to joint [angle_tooth, angle_counter].
    """
    if free_params is None:
        try_geom, try_rep = _solve_com(
            base_geom, target_bite_h, ["angle_counter"], target_mass_g, target_energy_j, pct_range
        )
        if try_rep["converged"]:
            return try_geom, try_rep
        # Fallback to joint optimization if angle_counter alone cannot overcome large tooth mass
        return _solve_com(
            base_geom, target_bite_h, ["angle_tooth", "angle_counter"], target_mass_g, target_energy_j, pct_range
        )
    return _solve_com(
        base_geom, target_bite_h, free_params, target_mass_g, target_energy_j, pct_range
    )


def balance_for_speed_bite(
    base_geom: WeaponGeometry,
    target_mass_g: float = 1500.0,
    target_energy_j: float = 2000.0,
    closing_speed_m_s: float = 5.364,
    max_iters: int = 10,
    tol_mm: float = 0.05
) -> Tuple[WeaponGeometry, Dict[str, Any]]:
    """
    Finds the exact fixed-point equilibrium where physical bite step h
    equals the dynamic speed bite limit at target energy: h = dynamic_bite(h).
    """
    h_curr = 20.0
    balanced = None
    rep = None
    for _ in range(max_iters):
        balanced, rep = balance_for_bite(
            base_geom,
            target_bite_h=h_curr,
            target_mass_g=target_mass_g,
            target_energy_j=target_energy_j
        )
        mp = balanced.mass_properties(target_energy_j=target_energy_j, closing_speed_m_s=closing_speed_m_s)
        h_dyn = mp["bite_h_dynamic_mm"]
        if abs(h_dyn - h_curr) < tol_mm:
            break
        h_curr = 0.5 * (h_curr + h_dyn)

    rep["fixed_point_bite_mm"] = balanced.bite_h
    return balanced, rep


def pareto_bite_vs_inertia(
    base_geom: WeaponGeometry,
    bite_values: List[float] = None,
    target_mass_g: float = 1500.0,
    target_energy_j: float = 2000.0,
    closing_speed_m_s: float = 5.364
) -> List[Dict[str, Any]]:
    if bite_values is None:
        bite_values = [10.0, 12.0, 14.0, 16.0, 18.0]

    frontier = []
    for bite in bite_values:
        try:
            balanced_geom, report = balance_for_bite(
                base_geom,
                target_bite_h=bite,
                target_mass_g=target_mass_g,
                target_energy_j=target_energy_j
            )
            mp = balanced_geom.mass_properties(
                target_energy_j=target_energy_j,
                closing_speed_m_s=closing_speed_m_s
            )
            frontier.append({
                "bite_h_mm": bite,
                "sim_avg_bite_mm": mp["sim_avg_bite_mm"],
                "bite_angle_max_deg": mp["bite_angle_max_deg"],
                "balanced": report["converged"],
                "izz_kg_m2": mp["izz_kg_m2"],
                "mass_g": mp["mass_g"],
                "m_eff_kg": mp["m_eff_kg"],
                "required_rpm": mp["required_rpm"],
                "v_tip_mph": mp["v_tip_mph"],
                "thickness_mm": balanced_geom.thickness,
                "min_neck_width_mm": balanced_geom.min_neck_width_mm,
                "actual_fillet_r_mm": balanced_geom.actual_fillet_r,
                "angle_counter": balanced_geom.angle_counter,
                "yaw_lift_deg_s": mp["yaw_lift_deg_s"],
                "geometry": balanced_geom
            })
        except Exception as e:
            frontier.append({
                "bite_h_mm": bite,
                "balanced": False,
                "error": str(e)
            })

    return frontier
