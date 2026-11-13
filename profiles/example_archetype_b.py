"""
Archetype B: Custom 2D Sketch Geometry
Use this format if your SolidWorks sketch has custom geometry (chisel teeth, multiple teeth, asymmetric lobes, or lightening pockets).
"""

import math
from dataclasses import dataclass
from typing import Tuple
from build123d import (
    BuildSketch, BuildLine, Line, ThreePointArc, Circle, Mode,
    make_face, fillet, BuildPart, extrude, Part
)


@dataclass
class CustomWeaponGeometry:
    # 1. Extracted Fixed Dimensions from SolidWorks
    r_outer: float = 72.0        # Max outer tip radius (mm)
    dia_bore: float = 22.0       # Center bore Ø22.0 mm at (0, 0)
    density: float = 0.00785     # Steel density (g/mm³)
    thickness: float = 30.60     # Auto-sized to hit target mass (1500 g)

    # 2. Free Balancing Parameter (Tuned by the solver to drive COM -> 0, 0)
    counter_weight_angle: float = 85.0

    def build_sketch(self):
        """Build exact 2D planar face from SolidWorks sketch entities."""
        # Top tooth tip points
        p_tip_r = (25.0, 67.5)
        p_tip_mid = (0.0, self.r_outer)
        p_tip_l = (-25.0, 67.5)

        # Counterweight lobe points (dependent on counter_weight_angle)
        ang_rad = math.radians(self.counter_weight_angle / 2.0)
        r_c = 60.0
        p_bot_r = (r_c * math.sin(ang_rad), -r_c * math.cos(ang_rad))
        p_bot_l = (-r_c * math.sin(ang_rad), -r_c * math.cos(ang_rad))
        p_bot_mid = (0.0, -r_c)

        with BuildSketch() as s:
            with BuildLine():
                ThreePointArc(p_tip_r, p_tip_mid, p_tip_l)
                ThreePointArc(p_tip_l, (-42.0, 0.0), p_bot_l)
                ThreePointArc(p_bot_l, p_bot_mid, p_bot_r)
                ThreePointArc(p_bot_r, (42.0, 0.0), p_tip_r)
            make_face()

            # Central hub reinforcement
            Circle(radius=25.0, mode=Mode.ADD)

            # Central shaft bore Ø22 mm at origin (0, 0)
            Circle(radius=self.dia_bore / 2.0, mode=Mode.SUBTRACT)

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
