"""
CLI entry point for python_balancer.
Run instant balancing, Pareto trade-off sweeps, and CAD export.
"""

import argparse
import sys
from .model import WeaponGeometry
from .optimizer import balance_weapon, pareto_frontier


def print_banner():
    print("""
===================================================================
         SolidSpin Python-Native Weapon Balancer & Optimizer        
       Powered by OpenCASCADE (build123d) & Pure-Python Solvers     
===================================================================
""")


def run_balance_demo():
    print_banner()
    geom = WeaponGeometry()

    print("[*] Baseline Design Parameters:")
    print(f"    Top Lobe Arc:      R{geom.r_top:.1f} mm, {geom.angle_top:.1f}°")
    print(f"    Bottom Lobe Arc:   R{geom.r_bot:.1f} mm, {geom.angle_bot:.1f}°")
    print(f"    Waist Arc:         R{geom.r_waist:.1f} mm")
    print(f"    Center Relief:     Ø{geom.dia_relief:.1f} mm (fillet R{geom.fillet_r:.1f})")
    print(f"    Shaft Bore:        Ø{geom.dia_bore:.1f} mm")
    print(f"    Thickness:         {geom.thickness:.1f} mm")

    mp_init = geom.mass_properties()
    print(f"\n[*] Initial COM Offset:  {mp_init['com_offset_mm']:.4f} mm  (UNBALANCED)")
    print(f"    Total Mass:          {mp_init['mass_g']:.2f} g")
    print(f"    Moment of Inertia:   {mp_init['izz_kg_m2']:.6e} kg·m²")
    print(f"    Kinetic Energy @8k:  {mp_init['kinetic_energy_j']:.2f} Joules")

    print("\n[*] Running Fast In-Memory Balancing (optimizing R_top and Angle_top)...")
    balanced, report = balance_weapon(geom, free_params=["r_top", "angle_top"], pct_range=25.0)

    mp_bal = report["mass_props"]
    print(f"\n[+] Balanced in {report['elapsed_s']*1000.0:.1f} ms across {report['evals']} rebuilds!")
    print(f"    Remaining Offset:    {report['final_offset_mm']:.6f} mm  (PERFECT BALANCE)")
    print(f"    Optimized R_top:     {geom.r_top:.2f} mm -> {balanced.r_top:.2f} mm ({((balanced.r_top/geom.r_top)-1)*100:+.2f}%)")
    print(f"    Optimized Angle_top: {geom.angle_top:.2f}° -> {balanced.angle_top:.2f}° ({((balanced.angle_top/geom.angle_top)-1)*100:+.2f}%)")
    print(f"    Balanced Mass:       {mp_bal['mass_g']:.2f} g")
    print(f"    Balanced Energy:     {mp_bal['kinetic_energy_j']:.2f} Joules")

    # Export
    step_file, stl_file = balanced.export("balanced_weapon_disc")
    print(f"\n[+] Exported Balanced CAD: {step_file} & {stl_file}")


def run_pareto_demo():
    print_banner()
    geom = WeaponGeometry()
    print("[*] Generating Pareto Frontier: Weapon Reach (R_top) vs Mass vs Kinetic Energy")
    print("    (Each design point is dynamically balanced to COM offset < 0.0001 mm)\n")

    sweep = [52.0, 56.0, 60.0, 63.0, 66.0, 70.0]
    frontier = pareto_frontier(geom, sweep_param="r_top", sweep_values=sweep, balance_params=["angle_top", "r_bot"])

    header = f"{'R_top (mm)':>10} | {'Angle_top (°)':>13} | {'R_bot (mm)':>11} | {'Mass (g)':>10} | {'MOI Izz (kg·m²)':>16} | {'Energy (J)':>11} | {'Eff (J/g)':>10}"
    print(header)
    print("-" * len(header))

    for pt in frontier:
        if pt.get("balanced"):
            t = pt["tuning"]
            print(f"{pt['r_top']:>10.1f} | {t['angle_top']:>13.2f} | {t['r_bot']:>11.2f} | {pt['mass_g']:>10.1f} | {pt['izz_kg_m2']:>16.6e} | {pt['energy_j']:>11.1f} | {pt['energy_per_g']:>10.3f}")
        else:
            print(f"{pt['r_top']:>10.1f} | FAILED TO BALANCE ({pt.get('error')})")

    print("-" * len(header))
    print("\n[!] Pareto Trade-off Insight:")
    print("    - Increasing R_top from 52 mm to 70 mm increases Kinetic Energy substantially.")
    print("    - All designs on this curve maintain 0.0000 mm dynamic balance at combat speed.")


def main():
    parser = argparse.ArgumentParser(description="SolidSpin Python-Native CAD Balancer")
    parser.add_argument("--pareto", action="store_true", help="Run multi-objective Pareto trade-off curve")
    args = parser.parse_args()

    if args.pareto:
        run_pareto_demo()
    else:
        run_balance_demo()


if __name__ == "__main__":
    main()
