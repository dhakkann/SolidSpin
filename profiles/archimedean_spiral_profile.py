"""
Archetype B: Dynamically Balanced Archimedean Spiral Cam Disc.
Generated and numerically balanced to COM = (0.000, 0.000).
"""

import math
from dataclasses import dataclass
from typing import Tuple
from build123d import (
    BuildSketch, BuildLine, Spline, Line, Circle, Polygon,
    Mode, make_face, fillet, BuildPart, extrude, Part
)


@dataclass
class ArchimedeanSpiralGeometry:
    dia_bore: float = 22.00          # Central shaft bore Ø22.00 mm
    r_inner: float = 24.00           # Spiral inner radius (t = 0)
    r_outer: float = 60.00           # Tooth tip outer radius (t = 2*pi)
    t_start: float = -0.30           # Spiral start angle in radians
    t_end: float = 2.0 * math.pi     # Spiral end angle (2*pi radians)

    # Numerically balanced counterweight flat chord parameters:
    # Solved to drive COM -> (0.0000, 0.0000) mm
    cut_dist: float = 22.950         # mm (offset from origin; SolidWorks initial was 28.00 mm)
    cut_angle_deg: float = 7.696     # degrees (angle relative to tooth face; initial was 20.00°)

    gullet_fillet_r: float = 10.00   # R10.00 mm stress-relief fillet
    spiral_samples: int = 200

    density: float = 0.00785         # g/mm³ (7.85 g/cm³ steel)
    thickness: float = 53.92         # mm (auto-sized to hit 1500 g target mass)

    def _spiral_point(self, t: float) -> Tuple[float, float]:
        pitch_per_rad = (self.r_outer - self.r_inner) / (2.0 * math.pi)
        r = self.r_inner + pitch_per_rad * t
        return (r * math.cos(t), r * math.sin(t))

    def build_sketch(self):
        rot_offset = math.pi / 2.0  # Align tooth tip toward +Y axis

        def _rot(pt):
            c, s = math.cos(rot_offset), math.sin(rot_offset)
            return (pt[0]*c - pt[1]*s, pt[0]*s + pt[1]*c)

        pts = [_rot(self._spiral_point(self.t_start + (self.t_end - self.t_start) * (i / self.spiral_samples)))
               for i in range(self.spiral_samples + 1)]
        p_root = pts[0]
        p_tip = pts[-1]

        p_root_sw = self._spiral_point(self.t_start)
        p_tip_sw = self._spiral_point(self.t_end)
        tooth_angle_sw = math.atan2(p_tip_sw[1] - p_root_sw[1], p_tip_sw[0] - p_root_sw[0])

        cut_line_angle_native = tooth_angle_sw - math.radians(self.cut_angle_deg)
        u_dir_native = (math.cos(cut_line_angle_native), math.sin(cut_line_angle_native))
        n_native = (-u_dir_native[1], u_dir_native[0])
        if n_native[1] > 0:
            n_native = (-n_native[0], -n_native[1])

        u_dir = _rot(u_dir_native)
        n_out = _rot(n_native)

        box_half_width = 200.0
        box_depth = 200.0

        c0 = (n_out[0] * self.cut_dist - u_dir[0] * box_half_width,
              n_out[1] * self.cut_dist - u_dir[1] * box_half_width)
        c1 = (n_out[0] * self.cut_dist + u_dir[0] * box_half_width,
              n_out[1] * self.cut_dist + u_dir[1] * box_half_width)
        c2 = (c1[0] + n_out[0] * box_depth, c1[1] + n_out[1] * box_depth)
        c3 = (c0[0] + n_out[0] * box_depth, c0[1] + n_out[1] * box_depth)

        with BuildSketch() as s:
            with BuildLine():
                Spline(*pts)
                Line(p_tip, p_root)
            make_face()
            if self.gullet_fillet_r > 0:
                v = s.vertices().sort_by_distance(p_root)[0]
                fillet(v, radius=self.gullet_fillet_r)
            Polygon(c0, c1, c2, c3, align=None, mode=Mode.SUBTRACT)
            Circle(radius=self.dia_bore / 2.0, mode=Mode.SUBTRACT)

        return s.sketch

    def build(self) -> Part:
        with BuildPart() as bp:
            self.build_sketch()
            extrude(amount=self.thickness)
        return bp.part

    def evaluate_com(self) -> Tuple[float, float]:
        part = self.build()
        c = part.center()
        return (c.X, c.Y)

    def set_mass_target(self, target_mass_g: float = 1500.0) -> float:
        part = self.build()
        area_mm2 = part.volume / self.thickness
        self.thickness = target_mass_g / (self.density * area_mm2)
        return self.thickness
