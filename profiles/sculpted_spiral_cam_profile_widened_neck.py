"""
Sculpted Archimedean Spiral Cam Disc (Screenshots 003340 - 003511).
Reinforced Widened Neck Profile — dynamically balanced to COM = (0.000, 0.000).

Geometry Updates:
- Reinforced Front Neck: R_neck_front = 35.00 mm (baseline was 28.31 mm, +24% wider neck)
- Front Bore Wall: 24.0 mm (+39% thicker web between bore and tooth root)
- Front Bolt Web: 14.5 mm (+86% thicker web between M4 bolt hole and tooth root)
- Dynamically balanced tuning variables:
  * balance_param (CW wrap angle): 225.522° (baseline SW drawing was ~222.0°)
  * deg_right_spiral (flank tangent cut): 336.463° (baseline SW drawing was 332.0°)
- Required plate thickness for 1500 g mass target: 42.30 mm (slimmer by 5.5 mm)
- Speed for 2000 J kinetic energy: 15,524 RPM (tip speed 97.5 m/s / 351 km/h)
"""

import math
from dataclasses import dataclass
from typing import Tuple, List, Dict, Any
from build123d import (
    BuildSketch, BuildLine, Line, ThreePointArc, Spline,
    Circle, PolarLocations, Mode, make_face, fillet,
    extrude, Part
)
from scipy.optimize import root


@dataclass
class SculptedSpiralGeometry:
    dia_bore: float = 22.00          # Center shaft bore Ø22.00 mm (or Ø24.00 mm)
    r_inner: float = 30.00           # Spiral inner radius: 30.00 mm (at t = 0)
    r_outer: float = 60.00           # Outer sweep radius: 60.00 mm (at t = 2*pi)
    t_start: float = -0.354          # Spiral start angle (-0.354 rad gives 72.99° tooth angle)
    t_end: float = 2.0 * math.pi     # Spiral end angle at tooth tip (2*pi rad)

    # 3x Ø4.00 mm Mounting Bolt Pattern (Screenshot 003511)
    bolt_count: int = 3              # 3x circular pattern
    bolt_dia: float = 4.00           # Ø4.00 mm mounting holes
    bolt_circle_r: float = 18.50     # R18.50 mm (Ø37.00 mm bolt circle)

    # Gullet & relief sculpting radii (Screenshots 003407 & 003340)
    gullet_fillet_r: float = 15.00   # Concave gullet root fillet at tooth base
    upper_relief_r: float = 26.00    # Upper-right inward-curving relief arc radius
    lower_hook_r: float = 22.00      # Bottom-left backward-curving hook arc radius

    # Front Neck & Hub Boss Radius (Reinforced)
    r_neck_front: float = 35.00      # Front throat / core boss radius (mm)

    align_tooth_to_plus_y: bool = False  # False = native SolidWorks frame; True = combat spinner (+Y)
    density: float = 0.00785         # Steel density (g/mm³) - EN24 / 4340
    thickness: float = 42.30         # Auto-sized to hit 1500 g target mass (for Ø22 bore)

    # Numerically balanced parameters:
    balance_param: float = 225.522   # CW outer spiral wrap termination angle (deg)
    deg_right_spiral: float = 336.463 # Bottom-right tangent chord intersection angle (deg)

    def _spiral_r(self, t: float) -> float:
        """Archimedean spiral radius R(t) = R_inner + (R_outer - R_inner) * (t / 2*pi)."""
        return self.r_inner + (self.r_outer - self.r_inner) * (t / (2.0 * math.pi))

    def _spiral_pt(self, t: float) -> Tuple[float, float]:
        """Evaluate (x, y) on the Archimedean spiral in native SolidWorks frame."""
        r = self._spiral_r(t)
        return (r * math.cos(t), r * math.sin(t))

    def _core_pt(self, r_core: float, deg: float) -> Tuple[float, float]:
        """Evaluate (x, y) on a core circle of radius r_core."""
        rad = math.radians(deg)
        return (r_core * math.cos(rad), r_core * math.sin(rad))

    def _rot(self, pt: Tuple[float, float], rot_rad: float) -> Tuple[float, float]:
        """Rotate 2D point counter-clockwise around origin (0, 0)."""
        c, s = math.cos(rot_rad), math.sin(rot_rad)
        return (pt[0] * c - pt[1] * s, pt[0] * s + pt[1] * c)

    @staticmethod
    def _arc_midpoint(
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        radius: float,
        concave_inward_to_origin: bool
    ) -> Tuple[float, float]:
        """
        Computes the exact midpoint of a circular arc of given `radius` between p1 and p2.
        Eliminates all OpenCASCADE RadiusArc sign/sagitta ambiguity.
        """
        mx, my = 0.5 * (p1[0] + p2[0]), 0.5 * (p1[1] + p2[1])
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        chord = math.hypot(dx, dy)
        r = max(abs(radius), 0.5 * chord + 1e-4)

        h = math.sqrt(max(0.0, r * r - (0.5 * chord) ** 2))
        sagitta = r - h

        nx, ny = -dy / chord, dx / chord
        mid_a = (mx + sagitta * nx, my + sagitta * ny)
        mid_b = (mx - sagitta * nx, my - sagitta * ny)

        dist_a = math.hypot(mid_a[0], mid_a[1])
        dist_b = math.hypot(mid_b[0], mid_b[1])
        if concave_inward_to_origin:
            return mid_a if dist_a < dist_b else mid_b
        else:
            return mid_a if dist_a > dist_b else mid_b

    def build_sketch(self):
        """
        Builds the reinforced SolidWorks profile with widened front neck:
        - Sharp top spiral tip at 90° curving inward along R26 relief arc to core circle (58°).
        - Outward-bulging reinforced center-right neck arc (R35 mm) from 58° to root at -20.28°.
        - R15.00 concave gullet fillet ONLY at the tooth root joining the flat striking face.
        - Sharp bottom-left counterweight hook at ~225.5° curving backwards/inward (R22) to core.
        - Smooth S-curve bottom transition flat chord.
        """
        rot_offset = (math.pi / 2.0) if self.align_tooth_to_plus_y else 0.0

        r_core_base = self._spiral_r(self.t_start) # 28.31 mm for CW hook/bottom
        r_neck = self.r_neck_front                 # 35.00 mm for reinforced front neck
        deg_root = math.degrees(self.t_start)      # -20.28°

        deg_top_spiral = 90.0
        deg_upper_core = 58.0
        deg_cw_end = self.balance_param
        deg_lower_core_1 = deg_cw_end + 28.0
        deg_lower_core_2 = 270.0
        deg_right_spiral = self.deg_right_spiral

        # 1. Counterweight Outer Spiral
        t_cw_start = math.radians(deg_top_spiral)
        t_cw_end = math.radians(deg_cw_end)
        cw_spiral_pts: List[Tuple[float, float]] = [
            self._rot(
                self._spiral_pt(t_cw_start + (t_cw_end - t_cw_start) * (i / 80)),
                rot_offset
            )
            for i in range(81)
        ]
        p_top_spiral = cw_spiral_pts[0]
        p_cw_hook_tip = cw_spiral_pts[-1]

        # 2. Bottom-Left Hook Relief Arc (curves BACKWARDS / concave inward)
        p_core_bl = self._rot(self._core_pt(r_core_base, deg_lower_core_1), rot_offset)
        p_hook_mid = self._arc_midpoint(
            p_cw_hook_tip, p_core_bl, radius=self.lower_hook_r, concave_inward_to_origin=True
        )

        # 3. Bottom Core Circle Arc
        p_core_bot = self._rot(self._core_pt(r_core_base, deg_lower_core_2), rot_offset)
        p_core_bot_mid = self._rot(
            self._core_pt(r_core_base, 0.5 * (deg_lower_core_1 + deg_lower_core_2)),
            rot_offset
        )

        # 4. Outer Tooth Spiral Flank
        t_tooth_base = math.radians(deg_right_spiral)
        tooth_spiral_pts: List[Tuple[float, float]] = [
            self._rot(
                self._spiral_pt(t_tooth_base + (self.t_end - t_tooth_base) * (i / 40)),
                rot_offset
            )
            for i in range(41)
        ]
        p_tooth_base = tooth_spiral_pts[0]
        p_tooth_tip = tooth_spiral_pts[-1]

        # 5. Reinforced Front Neck & Core Boss (R35 mm from root -20.28° around right bolt hole to +58°)
        p_root = self._rot(self._core_pt(r_neck, deg_root), rot_offset)
        p_core_right_mid = self._rot(
            self._core_pt(r_neck, 0.5 * (deg_root + deg_upper_core)),
            rot_offset
        )
        p_core_ur = self._rot(self._core_pt(r_neck, deg_upper_core), rot_offset)

        # 6. Upper-Right Concave Relief Arc
        p_upper_relief_mid = self._arc_midpoint(
            p_core_ur, p_top_spiral, radius=self.upper_relief_r, concave_inward_to_origin=True
        )

        with BuildSketch() as s:
            with BuildLine():
                Spline(*cw_spiral_pts)
                ThreePointArc(p_cw_hook_tip, p_hook_mid, p_core_bl)
                ThreePointArc(p_core_bl, p_core_bot_mid, p_core_bot)
                Line(p_core_bot, p_tooth_base)
                Spline(*tooth_spiral_pts)
                Line(p_tooth_tip, p_root)
                ThreePointArc(p_root, p_core_right_mid, p_core_ur)
                ThreePointArc(p_core_ur, p_upper_relief_mid, p_top_spiral)

            make_face()

            # ONLY fillet the interior tooth gullet root (R15.00 mm)
            if self.gullet_fillet_r > 0.0:
                root_vtx = s.vertices().sort_by_distance(p_root)[0]
                fillet(root_vtx, radius=self.gullet_fillet_r)

            # Central shaft bore
            Circle(radius=self.dia_bore / 2.0, mode=Mode.SUBTRACT)

            # 3x Ø4.00 mm mounting holes
            hole_start_deg = 90.0 + math.degrees(rot_offset)
            with PolarLocations(
                radius=self.bolt_circle_r,
                count=self.bolt_count,
                start_angle=hole_start_deg
            ):
                Circle(radius=self.bolt_dia / 2.0, mode=Mode.SUBTRACT)

        return s.sketch

    def build(self) -> Part:
        sk = self.build_sketch()
        return extrude(sk, amount=self.thickness)

    def evaluate_com(self) -> Tuple[float, float]:
        part = self.build()
        c = part.center()
        return (c.X, c.Y)

    def set_mass_target(self, target_mass_g: float = 1500.0) -> float:
        part = self.build()
        area_mm2 = part.volume / self.thickness
        self.thickness = target_mass_g / (self.density * area_mm2)
        return self.thickness

    def balance(self, target_tol: float = 1e-4) -> Tuple[float, float]:
        """Numerically balance the disc by adjusting balance_param and deg_right_spiral."""
        def obj(x):
            self.balance_param = x[0]
            self.deg_right_spiral = x[1]
            try:
                c = self.evaluate_com()
                return [c[0], c[1]]
            except Exception:
                return [999.0, 999.0]

        res = root(obj, [self.balance_param, self.deg_right_spiral], method='hybr', tol=target_tol)
        if res.success:
            self.balance_param = float(res.x[0])
            self.deg_right_spiral = float(res.x[1])
        return self.evaluate_com()
