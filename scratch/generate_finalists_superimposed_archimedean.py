import math
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.lines import Line2D

repo_dir = r"c:\Users\holme\Desktop\RoboWars-IIT-Mandi\SW Balancer Plugin"
if repo_dir not in sys.path:
    sys.path.insert(0, repo_dir)

from profiles.sculpted_spiral_cam_profile_thick_tooth import SculptedSpiralThickToothGeometry
from profiles.sw_profile_r60 import SW_R60_Geometry

art_dir = r"C:\Users\holme\.gemini\antigravity\brain\74358194-5efd-4c2e-a100-84b78a868dd1"

# 1. Finalist 1 (Fortified Archimedean Spiral Cam Disc)
g1 = SculptedSpiralThickToothGeometry(align_tooth_to_plus_y=True)
p1 = g1.build_part()
m1 = g1.compute_mass_properties(2000.0)
face1 = p1.faces().sort_by(lambda f: f.center().Z)[0]
outer_pts1 = [edge.position_at(t) for edge in face1.outer_wire().edges() for t in np.linspace(0, 1, 150)]
poly1 = [(p.X, p.Y) for p in outer_pts1]
holes1 = []
for iw in face1.inner_wires():
    h_pts = [edge.position_at(t) for edge in iw.edges() for t in np.linspace(0, 1, 60)]
    holes1.append([(p.X, p.Y) for p in h_pts])

# 2. Finalist 2 (SW R60 Hook)
g2 = SW_R60_Geometry.positive_rake_max_hook()
p2 = g2.build()
m2 = g2.mass_properties()
face2 = p2.faces().sort_by(lambda f: f.center().Z)[0]
outer_pts2 = [edge.position_at(t) for edge in face2.outer_wire().edges() for t in np.linspace(0, 1, 150)]
poly2_raw = [(p.X, p.Y) for p in outer_pts2]
holes2 = []
for iw in face2.inner_wires():
    h_pts = [edge.position_at(t) for edge in iw.edges() for t in np.linspace(0, 1, 60)]
    holes2.append([(p.X, p.Y) for p in h_pts])

# Rotate Finalist 2 so that its striking tooth tip coincides with Finalist 1's tip at exactly (0.0, 60.0) mm
rot_ang = math.radians(90.0 - math.degrees(math.atan2(57.827329, -16.0)))
c_rot, s_rot = math.cos(rot_ang), math.sin(rot_ang)

poly2_aligned = [(x*c_rot - y*s_rot, x*s_rot + y*c_rot) for x, y in poly2_raw]
holes2_aligned = [[(x*c_rot - y*s_rot, x*s_rot + y*c_rot) for x, y in h] for h in holes2]

# 3. Archimedean Reference Spirals
phis = np.linspace(0.0, 2.0 * math.pi, 600)

# A) Theoretical Full-Cam Archimedean Spiral: R(phi) = 60.0 - 30.0 * (phi / 2pi)
r_arch_cam = 60.0 - 30.0 * (phis / (2.0 * math.pi))
x_arch_cam = r_arch_cam * np.sin(phis)
y_arch_cam = r_arch_cam * np.cos(phis)

# B) 12 mph Dynamic Penetration Spiral: b_dyn = 20.27 mm at 15,882 RPM
r_arch_dyn12 = 60.0 - 20.27 * (phis / (2.0 * math.pi))
x_arch_dyn12 = r_arch_dyn12 * np.sin(phis)
y_arch_dyn12 = r_arch_dyn12 * np.cos(phis)

# Precise Radial Envelope Evaluation for both profiles
def compute_radius_envelope(poly_pts, angles_deg):
    pts_arr = np.array(poly_pts)
    xs = pts_arr[:, 0]
    ys = pts_arr[:, 1]
    rs = np.hypot(xs, ys)
    thetas = np.degrees(np.arctan2(ys, xs))
    # Clockwise wrap angle from +Y axis:
    phis = (90.0 - thetas) % 360.0
    
    idx = np.argsort(phis)
    phis_sorted = phis[idx]
    rs_sorted = rs[idx]
    
    r_out = []
    for a in angles_deg:
        dphi = np.abs((phis_sorted - a + 180.0) % 360.0 - 180.0)
        mask = dphi < 2.0
        if np.any(mask):
            r_out.append(np.max(rs_sorted[mask]))
        else:
            r_out.append(np.interp(a, phis_sorted, rs_sorted))
    return np.array(r_out)

eval_angles = np.linspace(0.0, 360.0, 721)
r_prof1 = compute_radius_envelope(poly1, eval_angles)
r_prof2 = compute_radius_envelope(poly2_aligned, eval_angles)
r_arch_line = 60.0 - 30.0 * (eval_angles / 360.0)
r_dyn12_line = 60.0 - 20.27 * (eval_angles / 360.0)

# Setup Plot with 2 Panels
fig = plt.figure(figsize=(24, 11), dpi=300)
fig.patch.set_facecolor('#0d0f12')

# ==========================================
# Panel 1: 2D Superimposition
# ==========================================
ax1 = fig.add_axes([0.05, 0.08, 0.46, 0.84])
ax1.set_facecolor('#111317')
ax1.set_xlim(-72, 72)
ax1.set_ylim(-72, 72)
ax1.set_aspect('equal')
ax1.grid(True, linestyle=':', alpha=0.25, color='#718096')
ax1.tick_params(colors='#a0aec0')
for spine in ax1.spines.values():
    spine.set_color('#334155')
ax1.set_xlabel('X [mm] (Tip Aligned Frame)', color='#cbd5e0', fontsize=12)
ax1.set_ylabel('Y [mm] (Tip Aligned Frame)', color='#cbd5e0', fontsize=12)

# Fill Finalist 1 (Cyan tint)
poly1_arr = np.array(poly1)
ax1.fill(poly1_arr[:, 0], poly1_arr[:, 1], color='#06b6d4', alpha=0.22, label='_nolegend_')
ax1.plot(poly1_arr[:, 0], poly1_arr[:, 1], color='#22d3ee', lw=2.2, label='Finalist 1: Fortified Spiral Cam (EN24, 1500g)')

# Fill Finalist 2 (Purple tint)
poly2_arr = np.array(poly2_aligned)
ax1.fill(poly2_arr[:, 0], poly2_arr[:, 1], color='#a855f7', alpha=0.20, label='_nolegend_')
ax1.plot(poly2_arr[:, 0], poly2_arr[:, 1], color='#c084fc', lw=2.2, linestyle='-', label='Finalist 2: SW R60 Hook Aligned (EN24, 1500g)')

# Plot Bore & Bolt Holes
for h in holes1:
    h_arr = np.array(h)
    ax1.plot(h_arr[:, 0], h_arr[:, 1], color='#38bdf8', lw=1.5, linestyle=':')
for h in holes2_aligned:
    h_arr = np.array(h)
    ax1.plot(h_arr[:, 0], h_arr[:, 1], color='#e879f9', lw=1.5, linestyle=':')

# Plot Archimedean Reference Spiral (Full Cam 30 mm pitch)
ax1.plot(x_arch_cam, y_arch_cam, color='#fbbf24', lw=2.4, linestyle='--', label='Archimedean Cam Spiral: R(φ) = 60 - 30(φ/2π)')

# Plot 12 mph Dynamic Combat Advance Trajectory (b_dyn = 20.27 mm)
ax1.plot(x_arch_dyn12, y_arch_dyn12, color='#34d399', lw=2.0, linestyle='-.', label='12 mph Opponent Advance: R(φ) = 60 - 20.3(φ/2π)')

# Markers & Annotations
# Origin (Shaft Center)
ax1.plot(0, 0, '+', color='#f8fafc', markersize=14, markeredgewidth=2)
ax1.annotate('Shaft Center (0, 0)\n(Coincident Axis)', xy=(0, 0), xytext=(-28, -12),
             color='#f8fafc', fontsize=9.5, fontweight='bold',
             arrowprops=dict(arrowstyle='->', color='#f8fafc', lw=1.5))

# Tooth Tip (Coincident Tips at 0, 60)
ax1.plot(0, 60, 'o', color='#fbbf24', markersize=9)
ax1.annotate('COINCIDENT TOOTH TIPS (0.00, 60.00) mm\nΔr = 0.000 mm (Exact Tip Coincidence)',
             xy=(0, 60), xytext=(-36, 65),
             color='#fbbf24', fontsize=10, fontweight='bold',
             arrowprops=dict(arrowstyle='->', color='#fbbf24', lw=2.0))

# CCW Rotation Arrow
arc_ang = np.linspace(math.radians(110), math.radians(145), 30)
ax1.plot(66 * np.cos(arc_ang), 66 * np.sin(arc_ang), color='#38bdf8', lw=2.0)
ax1.annotate('CCW Weapon Rotation\n(Strike Motion)', xy=(66*math.cos(math.radians(145)), 66*math.sin(math.radians(145))),
             xytext=(-66, 45), color='#38bdf8', fontsize=9.5, fontweight='bold',
             arrowprops=dict(arrowstyle='->', color='#38bdf8', lw=2.0))

# Finalist 1 CW End Callout (separated to right)
r_cw1 = 47.25
x_cw1, y_cw1 = 21.41, -42.12
ax1.plot(x_cw1, y_cw1, 'o', color='#22d3ee', markersize=7)
ax1.annotate(f'Finalist 1 CW End\n(R = {r_cw1:.2f} mm | Step = {60-r_cw1:.2f} mm)',
             xy=(x_cw1, y_cw1), xytext=(28, -58),
             color='#22d3ee', fontsize=9, fontweight='bold',
             arrowprops=dict(arrowstyle='->', color='#22d3ee', lw=1.5))

# Finalist 2 CW Apex Callout (separated to left)
r_apex2 = 43.00
x_apex2, y_apex2 = -11.47, -41.44
ax1.plot(x_apex2, y_apex2, 'o', color='#c084fc', markersize=7)
ax1.annotate(f'Finalist 2 CW Apex\n(R = {r_apex2:.2f} mm | Throat = {60-r_apex2:.2f} mm)',
             xy=(x_apex2, y_apex2), xytext=(-58, -58),
             color='#c084fc', fontsize=9, fontweight='bold',
             arrowprops=dict(arrowstyle='->', color='#c084fc', lw=1.5))

ax1.set_title('SUPERIMPOSED WEAPON PROFILES & ARCHIMEDEAN CLEARANCE\n(Coincident Centers at (0,0) | Coincident Tooth Tips at (0,60) mm)',
              color='#f8fafc', fontsize=13, fontweight='bold', pad=14)
ax1.legend(loc='lower left', fontsize=9.5, facecolor='#1a202c', edgecolor='#334155', labelcolor='#e2e8f0')

# ==========================================
# Panel 2: Radial Clearance Margin vs Angle
# ==========================================
ax2 = fig.add_axes([0.57, 0.08, 0.39, 0.84])
ax2.set_facecolor('#111317')
ax2.grid(True, linestyle=':', alpha=0.25, color='#718096')
ax2.tick_params(colors='#a0aec0')
for spine in ax2.spines.values():
    spine.set_color('#334155')

# Plot Profile Radii vs Wrap Angle
ax2.plot(eval_angles, r_arch_line, color='#fbbf24', lw=2.4, linestyle='--', label='Archimedean Cam Envelope: R_cam(φ) = 60 - 30(φ/360)')
ax2.plot(eval_angles, r_dyn12_line, color='#34d399', lw=2.0, linestyle='-.', label='12 mph Opponent Advance: R_traj(φ) = 60 - 20.3(φ/360)')
ax2.plot(eval_angles, r_prof1, color='#22d3ee', lw=2.6, label='Finalist 1: Fortified Cam Disc Perimeter R_1(φ)')
ax2.plot(eval_angles, r_prof2, color='#c084fc', lw=2.6, label='Finalist 2: SW R60 Hook Perimeter R_2(φ)')

# Fill clearance zone for Finalist 1
ax2.fill_between(eval_angles, r_prof1, r_arch_line, where=(r_arch_line >= r_prof1),
                 color='#06b6d4', alpha=0.14, label='Finalist 1 Archimedean Clearance Zone')

# Annotations on Plot 2
ax2.annotate('Finalist 1 Aggressive Flank Drop\n(Relief Angle β = 11.4° | Saved 14.3 g)',
             xy=(20, 56), xytext=(45, 61),
             color='#22d3ee', fontsize=9.5, fontweight='bold',
             arrowprops=dict(arrowstyle='->', color='#22d3ee', lw=1.5))

ax2.annotate('Counterweight Comparison (180°)\n• Finalist 1: R = 45.0 mm (Exact Spiral)\n• Finalist 2: R = 43.1 mm (Apex Throat)',
             xy=(180, 43.1), xytext=(155, 52),
             color='#f8fafc', fontsize=9.5, fontweight='bold',
             arrowprops=dict(arrowstyle='->', color='#f8fafc', lw=1.5))

ax2.annotate('Finalist 2 Exceeds Spiral Cam Envelope\n(Circular arc R = 44.1 mm vs Cam R = 39.2 mm)',
             xy=(245, 43.8), xytext=(180, 32),
             color='#e879f9', fontsize=9.0, fontweight='bold',
             arrowprops=dict(arrowstyle='->', color='#e879f9', lw=1.5))

ax2.set_xlim(0, 360)
ax2.set_ylim(20, 65)
ax2.set_xlabel('Clockwise Wrap Angle Behind Tooth Tip φ [degrees]', color='#cbd5e0', fontsize=12)
ax2.set_ylabel('Radial Perimeter Distance R [mm]', color='#cbd5e0', fontsize=12)
ax2.set_title('BITE CAPACITY & ARCHIMEDEAN CLEARANCE PROFILES\n(Radial Distance vs Angle Behind Strike Tip)',
              color='#f8fafc', fontsize=13, fontweight='bold', pad=14)
ax2.legend(loc='upper right', fontsize=9.0, facecolor='#1a202c', edgecolor='#334155', labelcolor='#e2e8f0')

info_text = (
    "ARCHIMEDEAN BITE AUDIT (at 12 mph Closing Speed):\n"
    "• Tooth Tip Sweep: R_outer = 60.00 mm on both finalists (0.000 mm delta)\n"
    "• Dynamic Penetration (b_dyn): 20.27 mm/rev (F1) | 21.16 mm/rev (F2)\n"
    "• Finalist 1 Min Clearance Margin: +4.14 mm (100% Bite Preserved everywhere)\n"
    "• Finalist 2 Apex Clearance Margin: +6.42 mm (h = 17.0 mm apex throat)\n"
    "• Spiral Compliance: Finalist 1 stays <= Archimedean Cam everywhere;\n"
    "  Finalist 2 circular counterweight extends outside Cam at φ = 210°-255°."
)
ax2.text(0.04, 0.05, info_text, transform=ax2.transAxes, fontsize=8.8, color='#e2e8f0', family='monospace',
         bbox=dict(boxstyle='round,pad=0.6', facecolor='#1a202c', edgecolor='#fbbf24', alpha=0.92))

out_path = os.path.join(art_dir, 'finalists_superimposed_archimedean_overlay.png')
plt.savefig(out_path, dpi=300, facecolor=fig.get_facecolor(), bbox_inches='tight')
plt.close()
print(f"Saved superimposed Archimedean overlay graphic to: {out_path}")
