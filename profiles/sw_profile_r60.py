"""
SolidWorks Extracted Profile: R60mm / R40mm Single-Tooth Disc
Extracted from SolidWorks sketch (media_1794608585655_71ff813e.png).
"""

import math
from dataclasses import dataclass
from typing import Tuple, Dict, Any
from build123d import (
    BuildSketch, BuildLine, RadiusArc, Circle, Mode,
    make_face, BuildPart, extrude, Part
)


@dataclass
class SW_R60_Geometry:
    dia_bore: float = 22.00       # Central shaft bore Ø22.00 mm
    dia_hub: float = 50.00        # Central hub core Ø50.00 mm (R = 25.0 mm)
    r_tooth: float = 60.00        # Outer tooth radius R60.00 mm
    r_waist: float = 61.52        # Concave waist radius R61.52 mm
    r_transition: float = 21.09   # Transition fillet radius R21.09 mm

    x_tooth: float = 14.34        # Tooth tip half-width (14.34 mm)
    x_waist_hub: float = 18.70    # Waist meets hub at x = 18.70 mm
    x_trans_hub: float = 24.84    # Transition meets hub at x = 24.84 mm
    x_counter: float = 39.75      # Counterweight half-span at x = 39.75 mm

    # Free balancing parameter (Counterweight radius: 40.0 mm orig, 43.06 mm balanced)
    r_counter: float = 43.062

    density: float = 0.00785      # g/mm³ (7.85 g/cm³ steel)
    thickness: float = 46.71      # mm (auto-sized to hit 1500 g target mass)

    def build_sketch(self):
        r_hub = self.dia_hub / 2.0
        r_c = self.r_counter

        y_tooth = math.sqrt(self.r_tooth**2 - self.x_tooth**2)
        y_waist_hub = math.sqrt(r_hub**2 - self.x_waist_hub**2)
        y_trans_hub = math.sqrt(r_hub**2 - self.x_trans_hub**2)
        y_counter = -math.sqrt(max(0.01, r_c**2 - self.x_counter**2))

        p1_r = (self.x_tooth, y_tooth)
        p2_r = (self.x_waist_hub, y_waist_hub)
        p3_r = (self.x_trans_hub, y_trans_hub)
        p4_r = (self.x_counter, y_counter)

        p4_l = (-self.x_counter, y_counter)
        p3_l = (-self.x_trans_hub, y_trans_hub)
        p2_l = (-self.x_waist_hub, y_waist_hub)
        p1_l = (-self.x_tooth, y_tooth)

        with BuildSketch() as s:
            with BuildLine():
                RadiusArc(p1_l, p1_r, radius=self.r_tooth)
                RadiusArc(p1_r, p2_r, radius=-self.r_waist)
                RadiusArc(p2_r, p3_r, radius=r_hub)
                RadiusArc(p3_r, p4_r, radius=-self.r_transition)
                RadiusArc(p4_r, p4_l, radius=r_c)
                RadiusArc(p4_l, p3_l, radius=-self.r_transition)
                RadiusArc(p3_l, p2_l, radius=r_hub)
                RadiusArc(p2_l, p1_l, radius=-self.r_waist)
            make_face()
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
