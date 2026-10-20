"""
Numerical Balancing & Multi-Objective Pareto Optimizer.
Uses Gauss-Newton + Broyden solver directly on parametric build123d geometry.
"""

import copy
import math
import time
from typing import List, Dict, Any, Tuple
from sw_balancer.solver import solve_multi, COM_TOL_MM
from .model import WeaponGeometry


def balance_weapon(
    base_geom: WeaponGeometry,
    free_params: List[str] = None,
    pct_range: float = 30.0,
    tol_mm: float = 1e-4
) -> Tuple[WeaponGeometry, Dict[str, Any]]:
    """
    Solve for exact dynamic balance (COM offset -> 0.0000 mm).

    :param base_geom: Initial WeaponGeometry instance
    :param free_params: List of attribute names to tune, e.g. ['r_top', 'angle_top']
    :param pct_range: Max allowable percentage variation (±pct)
    :param tol_mm: Convergence tolerance for COM offset in mm
    :return: (balanced_geometry, solver_report)
    """
    if free_params is None:
        free_params = ["r_top", "angle_top"]

    x0 = [getattr(base_geom, name) for name in free_params]
    scale = list(x0)
    lo = [v * (1.0 - pct_range / 100.0) for v in x0]
    hi = [v * (1.0 + pct_range / 100.0) for v in x0]

    def evaluate(xs):
        geom = copy.copy(base_geom)
        for name, val in zip(free_params, xs):
            setattr(geom, name, float(val))
        return geom.evaluate_com()

    t_start = time.perf_counter()
    res = solve_multi(evaluate, x0, scale, lo, hi)
    t_elapsed = time.perf_counter() - t_start

    # Create balanced geometry instance
    balanced_geom = copy.copy(base_geom)
    for name, val in zip(free_params, res["x"]):
        setattr(balanced_geom, name, float(val))

    initial_com = base_geom.evaluate_com()
    final_com = balanced_geom.evaluate_com()
    initial_offset = math.hypot(*initial_com)
    final_offset = math.hypot(*final_com)

    report = {
        "converged": final_offset <= tol_mm,
        "evals": res["evals"],
        "elapsed_s": t_elapsed,
        "free_params": free_params,
        "initial_values": {k: getattr(base_geom, k) for k in free_params},
        "balanced_values": {k: getattr(balanced_geom, k) for k in free_params},
        "initial_offset_mm": initial_offset,
        "final_offset_mm": final_offset,
        "mass_props": balanced_geom.mass_properties()
    }
    return balanced_geom, report


def pareto_frontier(
    base_geom: WeaponGeometry,
    sweep_param: str = "r_top",
    sweep_values: List[float] = None,
    balance_params: List[str] = None,
    rpm: float = 8000.0
) -> List[Dict[str, Any]]:
    """
    Explore the Pareto trade-off curve:
    Vary one primary design requirement (e.g. Tooth Radius / Reach `r_top`),
    and for each step, dynamically balance the weapon using `balance_params`.

    Computes:
    - Mass (g) vs Kinetic Energy (Joules)
    - Moment of Inertia Izz vs Weight
    - Hit Energy per gram (J/g) efficiency
    """
    if sweep_values is None:
        sweep_values = [55.0, 58.0, 60.0, 63.0, 66.0, 68.0, 70.0]

    if balance_params is None:
        balance_params = ["angle_top", "r_bot"]

    frontier = []

    for val in sweep_values:
        trial_geom = copy.copy(base_geom)
        setattr(trial_geom, sweep_param, val)

        try:
            balanced_geom, report = balance_weapon(trial_geom, free_params=balance_params)
            mp = balanced_geom.mass_properties(rpm=rpm)
            frontier.append({
                sweep_param: val,
                "balanced": report["converged"],
                "mass_g": mp["mass_g"],
                "izz_kg_m2": mp["izz_kg_m2"],
                "energy_j": mp["kinetic_energy_j"],
                "energy_per_g": mp["kinetic_energy_j"] / mp["mass_g"] if mp["mass_g"] > 0 else 0,
                "com_offset_mm": mp["com_offset_mm"],
                "tuning": report["balanced_values"],
                "geometry": balanced_geom
            })
        except Exception as e:
            frontier.append({
                sweep_param: val,
                "balanced": False,
                "error": str(e)
            })

    return frontier
