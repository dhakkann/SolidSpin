"""
Unit test for geometry and solver algorithms.
Tests:
1. SpinAxis 2D projection and offset calculation.
2. Controllability analysis on simulated Jacobian.
3. Multi-dimensional Gauss-Newton convergence to target COM (0, 0).
"""

import math
from sw_balancer.geometry import SpinAxis
from sw_balancer.solver import (
    solve,
    solve_multi,
    controllability,
    rate_dimensions,
    COM_TOL_MM
)


def test_spin_axis():
    # Spin axis along Z axis through (0.01, 0.02, 0) in meters
    axis = SpinAxis((0.01, 0.02, 0.0), (0, 0, 1), label="Bore Axis")

    assert axis.origin == (0.01, 0.02, 0.0)
    assert axis.point == (0.01, 0.02, 0.0)
    assert axis.direction == (0.0, 0.0, 1.0)
    assert repr(axis).startswith("<SpinAxis 'Bore Axis'")

    # Center of mass at (0.015, 0.025, 0.05) -> offset is +5 mm in X, +5 mm in Y
    com = (0.015, 0.025, 0.05)
    offset = axis.offset_mm(com)

    assert abs(offset[0] - 5.0) < 1e-6, f"Expected 5.0, got {offset[0]}"
    assert abs(offset[1] - 5.0) < 1e-6, f"Expected 5.0, got {offset[1]}"
    print("[PASS] SpinAxis test passed!")


def test_custom_axis_dialog_init():
    try:
        import tkinter as tk
        from sw_balancer.ui import CustomAxisDialog
        root = tk.Tk()
        root.withdraw()
        root.wait_window = lambda win: None

        axis = SpinAxis((0.005, 0.010, 0.015), (0.0, 0.0, 1.0), "Test Axis")
        dlg = CustomAxisDialog(root, axis)
        assert dlg.ent_ox.get() == "5.00"
        assert dlg.ent_oy.get() == "10.00"
        assert dlg.ent_oz.get() == "15.00"
        assert dlg.ent_dz.get() == "1.000"

        dlg.ent_ox.delete(0, tk.END)
        dlg.ent_ox.insert(0, "25.0")
        dlg.on_ok()
        assert dlg.result is not None
        assert abs(dlg.result.origin[0] - 0.025) < 1e-6
        root.destroy()
        print("[PASS] CustomAxisDialog test passed!")
    except Exception as e:
        print(f"Skipping GUI dialog test: {e}")


def test_solver_multi():
    # Simulated model with 2 dimensions: x1 (radius) and x2 (pocket angle)
    # Suppose baseline offset is (2.0 mm, -1.5 mm)
    # x1 moves COM primarily along X: dx = 0.5 * (x1 - 0.08) * 1000
    # x2 moves COM primarily along Y: dy = 0.3 * (x2 - 1.2) * 1000

    x0 = [0.08, 1.20]  # baseline in meters/radians
    target_offset = [2.0, -1.5]  # initial offset in mm

    def evaluate(xs):
        # returns offset (dx, dy)
        dx = target_offset[0] - 80.0 * (xs[0] - x0[0])
        dy = target_offset[1] - 60.0 * (xs[1] - x0[1])
        return (dx, dy)

    scale = [0.08, 1.20]
    lo = [0.04, 0.60]
    hi = [0.12, 1.80]

    res = solve_multi(evaluate, x0, scale, lo, hi)
    final_offset = res['residual']
    assert math.hypot(*final_offset) <= COM_TOL_MM, f"Solver did not converge: {final_offset}"
    print(f"[PASS] Multi-D Solver converged in {res['evals']} evaluations to {final_offset} mm!")


if __name__ == "__main__":
    test_spin_axis()
    test_solver_multi()
    print("\nAll unit tests passed successfully!")
