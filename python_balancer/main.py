"""
CLI for SolidSpin Combat Robotics Optimizer.
Supports parametric energy targets, 350 mm track width gyro calculation,
2D bite simulation, dynamic speed-bite matching, and dead-axle shaft/bearing safety checks.
"""

import argparse
import os
import shutil
from datetime import datetime
from .model import WeaponGeometry
from .optimizer import balance_for_bite, balance_for_speed_bite, pareto_bite_vs_inertia
from .visualizer import plot_weapon_profile


def print_banner():
    print("""
===================================================================================
     SolidSpin Combat Robotics Optimizer: Inertia, Bite Sim & Structural Checks    
===================================================================================
""")


def run_balance_demo(
    radius: float = 72.0,
    target_bite: float = 18.0,
    target_mass: float = 1500.0,
    target_energy: float = 2000.0,
    robot_mass: float = 8.0,
    track_width: float = 350.0,
    match_speed_bite: bool = False,
    export_cad: bool = False,
    image_path: str = None
):
    print_banner()
    r_waist = max(120.0, radius * 2.08)
    geom = WeaponGeometry(r_tooth=radius, r_counter=radius - target_bite, r_waist=r_waist)

    if match_speed_bite:
        print(f"[*] Solving fixed-point equilibrium where bite step h = dynamic_bite(h)...")
        print(f"    Target Mass = {target_mass:.0f} g, Target E = {target_energy:.0f} J, Robot = {robot_mass:.1f} kg")
        balanced, report = balance_for_speed_bite(
            geom,
            target_mass_g=target_mass,
            target_energy_j=target_energy
        )
        target_bite = balanced.bite_h
    else:
        print(f"[*] Balancing R{geom.r_tooth:.1f} mm profile for h = {target_bite:.1f} mm, Mass = {target_mass:.0f} g, Target E = {target_energy:.0f} J...")
        balanced, report = balance_for_bite(
            geom,
            target_bite_h=target_bite,
            target_mass_g=target_mass,
            target_energy_j=target_energy
        )

    mp = balanced.mass_properties(
        target_energy_j=target_energy,
        closing_speed_m_s=5.364,
        robot_mass_kg=robot_mass,
        track_width_mm=track_width
    )

    rebuilds = report.get('evals', 1)
    elapsed = report.get('elapsed_ms', 0.0)
    print(f"\n[+] CONVERGED IN {elapsed:.1f} ms ({rebuilds} rebuilds) | COM Offset = {report['com_offset_mm']:.6f} mm")
    print(f"    Solved Counterweight Arc:  {balanced.angle_counter:.2f}° (Tooth Arc Fixed @ {balanced.angle_tooth:.1f}°)")
    print(f"    Solved Plate Thickness:    {balanced.thickness:.2f} mm (Mass = {mp['mass_g']:.1f} g)")

    print("\n[1] TOOTH, GULLET & STRUCTURAL NECK:")
    print(f"    Tooth Tip Radius / Width:  R{balanced.r_tooth:.1f} mm  |  Chord = {balanced.tooth_tip_chord:.2f} mm")
    print(f"    True Signed Rake Angle:    {mp['rake_angle_deg']:+.2f}°  |  Included Wedge = {mp['wedge_angle_deg']:.2f}°")
    print(f"    Min Stem Neck Width:       {mp['min_neck_width_mm']:.2f} mm")
    print(f"    Gullet Root Fillet:        R{mp['actual_fillet_r_mm']:.1f} mm achieved (R{mp['requested_fillet_r_mm']:.1f} mm requested)")
    print(f"    Centrifugal Neck Stress:   {mp['centrifugal_stress_mpa']:.1f} MPa (vs ~950 MPa EN24 Yield)")

    print("\n[2] KINEMATIC BITE SIMULATION (at 5.364 m/s / 12 mph closing speed):")
    print(f"    Uncapped Dynamic Bite:     {mp['bite_h_dynamic_mm']:.1f} mm")
    print(f"    Physical Step Limit (h):   {mp['bite_h_physical_mm']:.1f} mm (Geometry-limited)")
    print(f"    Simulated Actual Bite:     Max = {mp['sim_max_bite_mm']:.1f} mm  |  Avg = {mp['sim_avg_bite_mm']:.1f} mm")
    print(f"    Effective Contact Angle:   {mp['bite_angle_max_deg']:.1f}° at max bite ({mp['bite_angle_avg_deg']:.1f}° avg)")

    print(f"\n[3] ENERGY, SPEED & GYRO STABILITY ({robot_mass:.1f} kg Robot, {track_width:.0f} mm Track):")
    print(f"    Spin Axis Inertia (Izz):   {mp['izz_kg_m2']:.6e} kg·m²  |  Effective Tip Mass = {mp['m_eff_kg']:.2f} kg")
    print(f"    Required Speed for {target_energy:.0f} J: {mp['required_rpm']:.0f} RPM  |  Tip Speed = {mp['v_tip_m_s']:.1f} m/s ({mp['v_tip_mph']:.1f} mph)")
    print(f"    Angular Momentum (H):      {mp['h_gyro_nms']:.2f} N·m·s")
    print(f"    Wheel-Lift Yaw Threshold:  {mp['yaw_lift_deg_s']:.1f}°/s ({mp['yaw_lift_rev_s']:.2f} rev/s)")

    print("\n[4] 16mm DEAD AXLE & NEEDLE BEARING (2x HK1616) SHOCK CHECK:")
    print(f"    Est. Peak Recoil Force:    {mp['f_impact_peak_kn']:.1f} kN (35% energy transfer over 15 mm)")
    print(f"    16mm EN24 Shaft Yield:     {mp['f_shaft_yield_kn']:.1f} kN (Safety Factor = {mp['shaft_safety_factor']:.2f}x)")
    print(f"    2x HK1616 Static C0 (48.4 kN): Safety Factor = {mp['bearing_c0_safety_factor']:.2f}x")

    save_path = image_path
    if not save_path or save_path == "weapon_profile.png":
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_path = f"weapon_profile_{int(target_energy)}J_R{int(balanced.r_tooth)}mm_h{balanced.bite_h:.1f}mm_{timestamp}.png"

    img_file = plot_weapon_profile(balanced, output_path=save_path, metrics=mp)
    print(f"\n[+] Saved unique 2D profile drawing: {img_file}")

    for sym_name in ["weapon_profile_latest.png", "weapon_profile.png"]:
        try:
            shutil.copyfile(img_file, sym_name)
        except Exception:
            pass

    artifact_dir = r"C:\Users\holme\.gemini\antigravity\brain\d071d536-1d43-4889-8d28-c3bf3d20d78a"
    if os.path.isdir(artifact_dir):
        try:
            shutil.copyfile(img_file, os.path.join(artifact_dir, os.path.basename(img_file)))
            shutil.copyfile(img_file, os.path.join(artifact_dir, "weapon_profile.png"))
        except Exception:
            pass

    if export_cad:
        exports = balanced.export(f"weapon_{int(target_energy)}J_R{int(balanced.r_tooth)}mm_h{int(target_bite)}mm")
        print(f"[+] Exported CAD files: {', '.join(exports)}")


def run_pareto_demo(target_energy: float = 2000.0):
    print_banner()
    geom = WeaponGeometry()
    bites = [10.0, 12.0, 14.0, 16.0, 18.0]
    frontier = pareto_bite_vs_inertia(geom, bite_values=bites, target_mass_g=1500.0, target_energy_j=target_energy)

    hdr = f"{'Step h':>7} | {'Avg Bite':>8} | {'Bite Ang':>8} | {'Thick':>7} | {'Izz (kg·m²)':>12} | {'Req RPM':>8} | {'Tip MPH':>7} | {'Lift °/s':>8}"
    print(hdr)
    print("-" * len(hdr))
    for pt in frontier:
        if pt.get("balanced"):
            print(
                f"{pt['bite_h_mm']:>5.1f}mm | "
                f"{pt['sim_avg_bite_mm']:>6.1f}mm | "
                f"{pt['bite_angle_max_deg']:>7.1f}° | "
                f"{pt['thickness_mm']:>5.1f}mm | "
                f"{pt['izz_kg_m2']:>12.5e} | "
                f"{pt['required_rpm']:>8.0f} | "
                f"{pt['v_tip_mph']:>7.1f} | "
                f"{pt['yaw_lift_deg_s']:>7.1f}°"
            )


def main():
    parser = argparse.ArgumentParser(description="SolidSpin Combat Robotics Optimizer")
    parser.add_argument("--radius", "-r", type=float, default=72.0, help="Outer tooth radius in mm (default: 72.0)")
    parser.add_argument("--bite", type=float, default=18.0, help="Physical Bite step h in mm (default: 18.0)")
    parser.add_argument("--speed-bite", action="store_true", help="Auto-calculate bite step matching the dynamic speed limit at target energy")
    parser.add_argument("--energy", type=float, default=2000.0, help="Target kinetic energy in Joules (default: 2000)")
    parser.add_argument("--mass", type=float, default=1500.0, help="Target weapon mass in grams (default: 1500)")
    parser.add_argument("--robot-mass", type=float, default=8.0, help="Total robot mass in kg (default: 8.0)")
    parser.add_argument("--track", type=float, default=350.0, help="Drive wheel track width in mm (default: 350.0)")
    parser.add_argument("--pareto", action="store_true", help="Run Pareto sweep across bite steps")
    parser.add_argument("--export", action="store_true", help="Export STEP and STL files")
    parser.add_argument("--image", type=str, default=None, help="Output path for 2D schematic (defaults to unique timestamped run filename)")
    args = parser.parse_args()

    if args.pareto:
        run_pareto_demo(target_energy=args.energy)
    else:
        run_balance_demo(
            radius=args.radius,
            target_bite=args.bite,
            target_mass=args.mass,
            target_energy=args.energy,
            robot_mass=args.robot_mass,
            track_width=args.track,
            match_speed_bite=args.speed_bite,
            export_cad=args.export,
            image_path=args.image
        )


if __name__ == "__main__":
    main()
