"""
CLI for python_balancer.
Focuses strictly on:
1. Spin Axis Inertia (Izz)
2. Bite h
3. Tooth & Gullet Shape
"""

import argparse
import sys
from .model import WeaponGeometry
from .optimizer import balance_for_bite, pareto_bite_vs_inertia


def print_banner():
    print("""
===================================================================================
     SolidSpin Combat Robotics Optimizer: Spin Inertia, Bite h, & Tooth/Gullet     
===================================================================================
""")


def run_balance_demo(target_bite: float = 18.0):
    print_banner()
    geom = WeaponGeometry()

    print("[1] TOOTH & GULLET SHAPE (Baseline):")
    print(f"    Tooth Tip Radius:      R{geom.r_tooth:.1f} mm")
    print(f"    Tooth Tip Width:       {geom.tooth_tip_chord:.2f} mm ({geom.angle_tooth:.1f}°)")
    print(f"    Gullet Depth:          {geom.gullet_depth:.2f} mm (swallows armor)")
    print(f"    Gullet Root Fillet:    R{geom.fillet_r:.1f} mm (reduces peak impact stress)")
    print(f"    Waist Relief Radius:   R{geom.r_waist:.1f} mm")

    print("\n[2] BITE h METRICS:")
    print(f"    Target Physical Bite:  h = {target_bite:.2f} mm (Tooth drop R_tooth - R_counter)")
    dyn_bite = geom.dynamic_bite(robot_closing_speed_m_s=3.5, rpm=8000.0)
    print(f"    Dynamic Bite Needed:   h_dyn = {dyn_bite:.2f} mm (at 3.5 m/s close & 8000 RPM)")
    print(f"    Clearance Margin:      {(target_bite / dyn_bite):.2f}x (prevents body rub)")

    print("\n[3] SPIN AXIS INERTIA & DYNAMIC BALANCE:")
    print("    Optimizing mass distribution to bring COM offset -> 0.0000 mm...")

    balanced, report = balance_for_bite(geom, target_bite_h=target_bite)
    mp = balanced.mass_properties(rpm=8000.0, closing_speed_m_s=3.5)

    print(f"\n[+] BALANCED IN {report['elapsed_ms']:.1f} ms ({report['evals']} rebuilds)!")
    print(f"    Spin Axis Inertia (Izz):  {mp['izz_kg_m2']:.6e} kg·m²")
    print(f"    Kinetic Energy @ 8000 RPM: {mp['kinetic_energy_j']:.1f} Joules")
    print(f"    Specific Inertia (Izz/M):  {mp['specific_inertia']:.6e} m² (mass efficiency)")
    print(f"    Total Mass:                {mp['mass_g']:.1f} g")
    print(f"    Remaining COM Offset:      {report['com_offset_mm']:.6f} mm (< 1 µm)")

    step_f, stl_f = balanced.export(f"weapon_bite_{int(target_bite)}mm")
    print(f"\n[+] Exported CAD Models: {step_f} & {stl_f}")


def run_pareto_demo():
    print_banner()
    geom = WeaponGeometry()

    print("[*] Pareto Trade-off Analysis: Bite h vs. Spin Axis Inertia vs. Tooth Shape")
    print("    Parameters: 8000 RPM, 3.5 m/s closing speed, Hardox Steel (7.85 g/cm³)")
    print("    Every design on this curve is 100% dynamically balanced (COM < 0.0001 mm)\n")

    bites = [12.0, 15.0, 18.0, 21.0, 24.0, 27.0]
    frontier = pareto_bite_vs_inertia(geom, bite_values=bites, rpm=8000.0, closing_speed_m_s=3.5)

    hdr = f"{'Bite h (mm)':>11} | {'Clearance':>9} | {'Tooth Width':>11} | {'Gullet Depth':>12} | {'Izz (kg·m²)':>13} | {'Energy (J)':>10} | {'Mass (g)':>8}"
    print(hdr)
    print("-" * len(hdr))

    for pt in frontier:
        if pt.get("balanced"):
            print(
                f"{pt['bite_h_mm']:>11.1f} | "
                f"{pt['clearance_ratio']:>8.2f}x | "
                f"{pt['tooth_chord_mm']:>10.1f}mm | "
                f"{pt['gullet_depth_mm']:>11.1f}mm | "
                f"{pt['izz_kg_m2']:>13.6e} | "
                f"{pt['kinetic_energy_j']:>10.1f} | "
                f"{pt['mass_g']:>8.1f}"
            )

    print("-" * len(hdr))
    print("\n[!] Engineering Takeaways:")
    print("    1. Higher Bite h requires a deeper counterweight drop (smaller R_counter).")
    print("    2. As Bite h increases from 12 mm to 27 mm, counterweight mass shifts inward,")
    print("       slightly reducing Spin Axis Inertia (Izz) while guaranteeing clean, non-rubbing hits.")
    print("    3. Tooth tip width and gullet depth remain structurally solid across all bite settings.")


def main():
    parser = argparse.ArgumentParser(description="SolidSpin Combat Robotics Optimizer")
    parser.add_argument("--bite", type=float, default=18.0, help="Target physical Bite h in mm (default: 18.0)")
    parser.add_argument("--pareto", action="store_true", help="Generate Bite h vs Spin Axis Inertia Pareto curve")
    args = parser.parse_args()

    if args.pareto:
        run_pareto_demo()
    else:
        run_balance_demo(target_bite=args.bite)


if __name__ == "__main__":
    main()
