"""
High-Resolution 2D Technical Profile Visualizer for Combat Weapon Disc.
Generates technical drawings with exact CAD boundary representations,
dimension callouts, kinematic bite results, gyro stability, and dead-axle structural checks.
"""

import math
from typing import Dict, Any, Optional
import matplotlib
matplotlib.use("Agg")  # Non-interactive backend
import matplotlib.pyplot as plt
import numpy as np
from build123d import (
    BuildSketch, BuildLine, ThreePointArc, Circle, Mode,
    make_face, fillet
)
from .model import WeaponGeometry


def extract_profile_coordinates(geom: WeaponGeometry):
    """
    Extract exact boundary wire coordinates from OpenCASCADE sketch.
    Respects actual fillet radius achieved in geom.actual_fillet_r.
    Returns:
        (xs_outer, ys_outer), (xs_bore, ys_bore)
    """
    ang_top_half = np.radians(geom.angle_tooth / 2.0)
    theta_tr = np.pi / 2.0 - ang_top_half
    theta_tl = np.pi / 2.0 + ang_top_half
    p_tr = (geom.r_tooth * np.cos(theta_tr), geom.r_tooth * np.sin(theta_tr))
    p_tl = (geom.r_tooth * np.cos(theta_tl), geom.r_tooth * np.sin(theta_tl))
    p_top_mid = (0.0, geom.r_tooth)

    ang_bot_half = np.radians(geom.angle_counter / 2.0)
    theta_br = -np.pi / 2.0 + ang_bot_half
    theta_bl = -np.pi / 2.0 - ang_bot_half
    p_br = (geom.r_counter * np.cos(theta_br), geom.r_counter * np.sin(theta_br))
    p_bl = (geom.r_counter * np.cos(theta_bl), geom.r_counter * np.sin(theta_bl))
    p_bot_mid = (0.0, -geom.r_counter)

    p_waist_r_mid = geom._calculate_waist_arc_midpoint(p_tr, p_br, geom.r_waist, is_right=True)
    p_waist_l_mid = (-p_waist_r_mid[0], p_waist_r_mid[1])
    r_relief = geom.dia_relief / 2.0

    # Use the actual fillet achieved during the 3D build, or step down cleanly
    r_target = getattr(geom, "actual_fillet_r", 0.0)
    if r_target <= 0.0:
        max_geom_fillet = min(geom.fillet_r, 0.45 * geom.min_neck_width_mm)
        candidate_radii = [max_geom_fillet, max_geom_fillet * 0.75, max_geom_fillet * 0.5, 4.0, 0.0]
    else:
        candidate_radii = [r_target, 0.0]

    for r_try in candidate_radii:
        try:
            with BuildSketch() as sketch:
                with BuildLine():
                    ThreePointArc(p_tr, p_top_mid, p_tl)
                    ThreePointArc(p_tl, p_waist_l_mid, p_bl)
                    ThreePointArc(p_bl, p_bot_mid, p_br)
                    ThreePointArc(p_br, p_waist_r_mid, p_tr)
                make_face()
                Circle(radius=r_relief, mode=Mode.ADD)

                if r_try > 0.5:
                    verts = [v for v in sketch.vertices() if abs(np.hypot(v.X, v.Y) - r_relief) < 4.5]
                    if verts:
                        fillet(verts, radius=r_try)

                Circle(radius=geom.dia_bore / 2.0, mode=Mode.SUBTRACT)

            face = sketch.sketch.faces()[0]
            outer_wire = face.outer_wire()
            inner_wires = face.inner_wires()

            pts_outer = [outer_wire.position_at(t) for t in np.linspace(0, 1, 600)]
            xs_outer = [p.X for p in pts_outer]
            ys_outer = [p.Y for p in pts_outer]

            bore_wire = inner_wires[0]
            pts_bore = [bore_wire.position_at(t) for t in np.linspace(0, 1, 200)]
            xs_bore = [p.X for p in pts_bore]
            ys_bore = [p.Y for p in pts_bore]

            return (xs_outer, ys_outer), (xs_bore, ys_bore)
        except Exception:
            continue

    raise RuntimeError("Failed to extract valid 2D wire geometry from OpenCASCADE.")


def plot_weapon_profile(
    geom: WeaponGeometry,
    output_path: str = "weapon_profile.png",
    metrics: Optional[Dict[str, Any]] = None,
    dpi: int = 200
):
    """
    Generate a technical 2D drawing of the weapon profile with dimension callouts,
    straight 22mm needle bearing bore, center of mass marker, and performance metrics.
    """
    if metrics is None:
        metrics = geom.mass_properties(target_energy_j=2000.0, closing_speed_m_s=5.364)

    (xs_outer, ys_outer), (xs_bore, ys_bore) = extract_profile_coordinates(geom)

    fig, ax = plt.subplots(figsize=(13, 11), facecolor='#0e131b')
    ax.set_facecolor('#131a24')

    # 1. Background grid & centerline axes
    ax.set_xlim(-110, 110)
    ax.set_ylim(-105, 105)
    ax.set_aspect('equal')
    ax.grid(True, color='#1e293b', linestyle='--', linewidth=0.7, alpha=0.7)

    ax.axhline(0, color='#38bdf8', linestyle=':', linewidth=0.8, alpha=0.4)
    ax.axvline(0, color='#38bdf8', linestyle=':', linewidth=0.8, alpha=0.4)

    # 2. Reference circles (Hub core Ø50, Tooth sweep Ø144)
    circle_hub = plt.Circle((0, 0), geom.dia_relief / 2.0, color='#38bdf8', fill=False,
                            linestyle='--', linewidth=1.0, alpha=0.35)
    circle_tip = plt.Circle((0, 0), geom.r_tooth, color='#f59e0b', fill=False,
                            linestyle=':', linewidth=1.0, alpha=0.4)
    ax.add_patch(circle_hub)
    ax.add_patch(circle_tip)

    # 3. Fill weapon body & bore
    ax.fill(xs_outer, ys_outer, color='#1e293b', alpha=0.95, edgecolor='#38bdf8', linewidth=2.4)
    ax.fill(xs_bore, ys_bore, color='#131a24', edgecolor='#f43f5e', linewidth=2.0)

    # 4. Center of Mass (COM) Crosshair at (0, 0)
    ax.scatter([0], [0], color='#22c55e', s=90, zorder=10, marker='o', edgecolors='white', linewidth=1.2)
    ax.plot([-3, 3], [0, 0], color='#22c55e', linewidth=1.5, zorder=11)
    ax.plot([0, 0], [-3, 3], color='#22c55e', linewidth=1.5, zorder=11)
    ax.text(2, -4, f"COM ({metrics.get('com_offset_mm', 0.0):.4f} mm)", color='#22c55e', fontsize=9, fontweight='bold',
            bbox=dict(boxstyle='square,pad=0.2', facecolor='#0e131b', edgecolor='#22c55e', alpha=0.85))

    # 5. Bore annotation (Ø22 mm straight through-bore for HK1612 needle bearings)
    r_bore = geom.dia_bore / 2.0
    ax.annotate(
        f"Dead-Axle Bore: Ø{geom.dia_bore:.1f} mm\n(Fits 2x HK1616 on 16mm Shaft)",
        xy=(-r_bore * np.cos(np.pi / 4), r_bore * np.sin(np.pi / 4)),
        xytext=(-50, 32),
        arrowprops=dict(facecolor='#f43f5e', edgecolor='#f43f5e', shrink=0.08, width=1.5, headwidth=6),
        color='#f43f5e', fontsize=9.5, fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.3', facecolor='#1e1b2e', edgecolor='#f43f5e', alpha=0.9)
    )

    # 6. Tooth Tip Radius & Chord Width Callout
    ang_top_half = np.radians(geom.angle_tooth / 2.0)
    theta_tr = np.pi / 2.0 - ang_top_half
    theta_tl = np.pi / 2.0 + ang_top_half
    p_tr = (geom.r_tooth * np.cos(theta_tr), geom.r_tooth * np.sin(theta_tr))
    p_tl = (geom.r_tooth * np.cos(theta_tl), geom.r_tooth * np.sin(theta_tl))

    ax.plot([p_tl[0], p_tr[0]], [p_tl[1] + 4, p_tr[1] + 4], color='#f59e0b', linestyle='-', linewidth=1.5)
    ax.text(0, p_tr[1] + 7, f"Tooth Chord: {geom.tooth_tip_chord:.1f} mm\n(R{geom.r_tooth:.1f} mm / Ø{2*geom.r_tooth:.0f} mm sweep)",
            color='#f59e0b', fontsize=9.5, fontweight='bold', ha='center',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#1e1a12', edgecolor='#f59e0b', alpha=0.9))

    # 7. Minimum Neck Width Dimension
    y_neck = 26.0
    w_half = geom.min_neck_width_mm / 2.0
    ax.plot([-w_half, w_half], [y_neck, y_neck], color='#38bdf8', linestyle='-', linewidth=1.8)
    ax.plot([-w_half, -w_half], [y_neck - 2, y_neck + 2], color='#38bdf8', linewidth=1.8)
    ax.plot([w_half, w_half], [y_neck - 2, y_neck + 2], color='#38bdf8', linewidth=1.8)
    centrifugal_str = metrics.get('centrifugal_stress_mpa', 0.0)
    ax.text(0, y_neck + 2.5, f"Min Stem Neck: {geom.min_neck_width_mm:.1f} mm (σ_cent = {centrifugal_str:.0f} MPa)",
            color='#38bdf8', fontsize=9, fontweight='bold', ha='center',
            bbox=dict(boxstyle='square,pad=0.2', facecolor='#0e131b', edgecolor='#38bdf8', alpha=0.85))

    # 8. Bite Clearance Step & Kinematic Simulation Callout
    sim_avg_bite = metrics.get('sim_avg_bite_mm', 0.0)
    bite_angle = metrics.get('bite_angle_max_deg', 0.0)
    ax.plot([56, 74], [-55, -55], color='#a855f7', linestyle=':', linewidth=1.2)
    ax.annotate(
        f"Bite Step h = {geom.bite_h:.1f} mm\nSim Avg Bite = {sim_avg_bite:.1f} mm\nImpact Angle = {bite_angle:.1f}°",
        xy=(p_tr[0], p_tr[1]),
        xytext=(55, 45),
        arrowprops=dict(facecolor='#a855f7', edgecolor='#a855f7', shrink=0.08, width=1.5, headwidth=6),
        color='#c084fc', fontsize=9.5, fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.3', facecolor='#1e142e', edgecolor='#a855f7', alpha=0.9)
    )

    # 9. Rake & Wedge Angle Callout (dynamic sign handling)
    rake_val = geom.rake_angle_deg
    rake_desc = "Positive (Hook)" if rake_val > 0 else "Negative (Compressive)"
    ax.text(
        42, 6,
        f"Rake Angle: {rake_val:+.2f}° ({rake_desc})\nWedge Angle: {geom.wedge_angle_deg:.2f}°",
        color='#34d399', fontsize=9, fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.3', facecolor='#0d2319', edgecolor='#34d399', alpha=0.9)
    )

    # 10. Root Fillet Callout (shows actual radius achieved)
    actual_r = metrics.get('actual_fillet_r_mm', getattr(geom, 'actual_fillet_r', geom.fillet_r))
    ax.annotate(
        f"Root Fillet\nR{actual_r:.1f} mm",
        xy=(20.0, 18.0),
        xytext=(48, -18),
        arrowprops=dict(facecolor='#38bdf8', edgecolor='#38bdf8', shrink=0.08, width=1.2, headwidth=5),
        color='#7dd3fc', fontsize=9,
        bbox=dict(boxstyle='round,pad=0.2', facecolor='#0f172a', edgecolor='#38bdf8', alpha=0.85)
    )

    # 11. Counterweight Lobe Callout
    ax.text(
        0, -geom.r_counter - 10,
        f"Counterweight Lobe (R{geom.r_counter:.1f} mm, {geom.angle_counter:.1f}°)",
        color='#94a3b8', fontsize=9, ha='center',
        bbox=dict(boxstyle='square,pad=0.2', facecolor='#0e131b', edgecolor='#475569', alpha=0.85)
    )

    # 12. Dynamic Engineering Specifications Header
    e_target = metrics.get('kinetic_energy_j', 2000.0)
    rpm_val = metrics.get('required_rpm', 11700.0)
    v_mph = metrics.get('v_tip_mph', 197.3)
    h_gyro = metrics.get('h_gyro_nms', 3.27)
    yaw_lift = metrics.get('yaw_lift_rev_s', 0.61)

    title_box = (
        f"COMBAT ROBOTICS WEAPON PROFILE ({e_target:.0f} J TARGET)\n"
        f"Mass: {metrics.get('mass_g', 1500):.1f} g | Thickness: {geom.thickness:.2f} mm | Bore: Ø{geom.dia_bore:.1f} mm\n"
        f"I_zz: {metrics.get('izz_kg_m2', 0):.4e} kg·m² | Gyro H: {h_gyro:.2f} N·m·s (350mm Track Wheel-Lift: {yaw_lift:.2f} rev/s)\n"
        f"Speed: {rpm_val:.0f} RPM | Tip Speed: {v_mph:.1f} mph | Sim Avg Bite: {sim_avg_bite:.1f} mm"
    )
    plt.title(title_box, color='white', fontsize=10.5, fontweight='bold', pad=14,
              bbox=dict(boxstyle='round,pad=0.5', facecolor='#1e293b', edgecolor='#38bdf8', alpha=0.95))

    # 13. Dynamic Dead Axle & Needle Bearing Structural Verification Banner
    f_peak = metrics.get('f_impact_peak_kn', 0.0)
    sf_shaft = metrics.get('shaft_safety_factor', 0.0)
    sf_bearing = metrics.get('bearing_c0_safety_factor', 0.0)
    status_text = (
        f"[DEAD AXLE SHOCK CHECK] Peak Recoil: {f_peak:.1f} kN | "
        f"16mm EN24 Shaft SF: {sf_shaft:.2f}x | "
        f"2x HK1616 C0 (48.4 kN) SF: {sf_bearing:.2f}x"
    )
    status_color = '#22c55e' if (sf_shaft >= 1.0 and sf_bearing >= 1.0) else '#f43f5e'
    ax.text(0, -98, status_text,
            color=status_color, fontsize=9, fontweight='bold', ha='center',
            bbox=dict(boxstyle='square,pad=0.3', facecolor='#111827', edgecolor=status_color, alpha=0.9))

    plt.tight_layout()
    plt.savefig(output_path, dpi=dpi, facecolor=fig.get_facecolor(), edgecolor='none')
    plt.close()
    return output_path
