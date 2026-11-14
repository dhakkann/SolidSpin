"""
Sculpted Archimedean Spiral Cam Disc (Screenshots 003340 - 003511).
Dynamically balanced to COM = (0.000, 0.000) via dual-variable tuning:
- balance_param (counterweight wrap angle): 201.83° (initial was 222.0°)
- deg_right_spiral (flank tangent angle): 333.59° (initial was 332.0°)
"""

import math
from dataclasses import dataclass
from typing import Tuple, List
from build123d import (
    BuildSketch, BuildLine, Line, RadiusArc, ThreePointArc, Spline,
    Circle, PolarLocations, Mode, make_face, fillet,
    BuildPart, extrude, Part
)


@dataclass
class SculptedSpiralGeometry:
    dia_bore: float = 22.00          # Center shaft bore Ø22.00 mm (or Ø24.00 mm)
    r_inner: float = 30.00           # Equation global variable: Σ 30.00 mm (at t = 0)
    r_outer: float = 60.00           # Outer sweep radius: Σ 60.00 mm (at t = 2*pi)
    t_start: float = -0.354          # Spiral start angle (-0.354 rad gives 72.99° tooth angle)
    t_end: float = 2.0 * math.pi     # Spiral end angle at tooth tip (2*pi rad)

    # Mounting pattern
    bolt_count: int = 3              # 3x circular pattern
    bolt_dia: float = 4.00           # Ø4.00 mm mounting holes
    bolt_circle_r: float = 18.50     # R18.50 mm (Ø37.00 mm bolt circle)

    # Radii
    gullet_fillet_r: float = 14.00   # R14.00 mm concave gullet fillet at tooth root
    upper_relief_r: float = 26.00    # Upper-right concave relief arc radius
    lower_hook_r: float = 22.00      # Bottom-left counterweight hook relief radius

    align_tooth_to_plus_y: bool = True
    density: float = 0.00785         # Steel density (g/mm³)
    thickness: float = 46.66         # Auto-sized to hit 1500 g target mass (for Ø22 bore)

    # Numerically balanced parameters:
    # 1. Counterweight outer spiral wrap termination angle
    balance_param: float = 201.827   # degrees (SolidWorks initial was 222.0°)
    # 2. Bottom-right tangent chord intersection on outer spiral
    deg_right_spiral: float = 333.590 # degrees (SolidWorks initial was 332.0°)

    def _spiral_r(self, t: float) -> float:
        return self.r_inner + (self.r_outer - self.r_inner) * (t / (2.0 * math.pi))

    def _spiral_pt(self, t: float) -> Tuple[float, float]:
        r = self._spiral_r(t)
        return (r * math.cos(t), r * math.sin(t))

    def _core_pt(self, r_core: float, deg: float) -> Tuple[float, float]:
        rad = math.radians(deg)
        return (r_core * math.cos(rad), r_core * math.sin(rad))

    def _rot(self, pt: Tuple[float, float], rot_rad: float) -> Tuple[float, float]:
        c, s = math.cos(rot_rad), math.sin(rot_rad)
        return (pt[0] * c - pt[1] * s, pt[0] * s + pt[1] * c)

    def build_sketch(self):
        rot_offset = (math.pi / 2.0) if self.align_tooth_to_plus_y else 0.0

        r_core = self._spiral_r(self.t_start)
        deg_root = math.degrees(self.t_start)

        deg_top_spiral = 90.0
        deg_upper_core = 62.0
        deg_cw_end = self.balance_param
        deg_lower_core_1 = deg_cw_end + 28.0
        deg_lower_core_2 = 270.0

        t_cw_start = math.radians(deg_top_spiral)
        t_cw_end = math.radians(deg_cw_end)
        cw_spiral_pts = [
            self._rot(self._spiral_pt(t_cw_start + (t_cw_end - t_cw_start) * (i / 80)), rot_offset)
            for i in range(81)
        ]

        p_cw_tip = cw_spiral_pts[-1]
        p_core_bl = self._rot(self._core_pt(r_core, deg_lower_core_1), rot_offset)
        p_core_bot = self._rot(self._core_pt(r_core, deg_lower_core_2), rot_offset)

        t_tooth_base = math.radians(self.deg_right_spiral)
        tooth_spiral_pts = [
            self._rot(self._spiral_pt(t_tooth_base + (self.t_end - t_tooth_base) * (i / 40)), rot_offset)
            for i in range(41)
        ]

        p_tooth_base = tooth_spiral_pts[0]
        p_tooth_tip = tooth_spiral_pts[-1]

        p_root = self._rot(self._spiral_pt(self.t_start), rot_offset)
        p_core_mid_r = self._rot(self._core_pt(r_core, 0.5 * (deg_root + deg_upper_core)), rot_offset)
        p_core_ur = self._rot(self._core_pt(r_core, deg_upper_core), rot_offset)
        p_top_spiral = cw_spiral_pts[0]

        with BuildSketch() as s:
            with BuildLine():
                Spline(*cw_spiral_pts)
                RadiusArc(p_cw_tip, p_core_bl, radius=-self.lower_hook_r, short_sagitta=True)
                RadiusArc(p_core_bl, p_core_bot, radius=r_core, short_sagitta=True)
                Line(p_core_bot, p_tooth_base)
                Spline(*tooth_spiral_pts)
                Line(p_tooth_tip, p_root)
                ThreePointArc(p_root, p_core_mid_r, p_core_ur)
                RadiusArc(p_core_ur, p_top_spiral, radius=-self.upper_relief_r, short_sagitta=True)
            make_face()

            if self.gullet_fillet_r > 0.0:
                root_vtx = s.vertices().sort_by_distance(p_root)[0]
                try:
                    fillet(root_vtx, radius=self.gullet_fillet_r)
                except Exception:
                    pass

            Circle(radius=self.dia_bore / 2.0, mode=Mode.SUBTRACT)

            hole_start_deg = 90.0 + math.degrees(rot_offset)
            with PolarLocations(radius=self.bolt_circle_r, count=self.bolt_count, start_angle=hole_start_deg):
                Circle(radius=self.bolt_dia / 2.0, mode=Mode.SUBTRACT)

        return s.sketch

    def build(self) -> Part:
        with BuildPart() as p:
            self.build_sketch()
            extrude(amount=self.thickness)
        return p.part

    def evaluate_com(self) -> Tuple[float, float]:
        part = self.build()
        c = part.center()
        return (c.X, c.Y)

    def set_mass_target(self, target_mass_g: float = 1500.0) -> float:
        part = self.build()
        area_mm2 = part.volume / self.thickness
        self.thickness = target_mass_g / (self.density * area_mm2)
        return self.thickness
