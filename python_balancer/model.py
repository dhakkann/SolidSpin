"""
Parametric Weapon Disc Geometry Definition using build123d.

Defines the asymmetric single/dual-tooth weapon disc with:
- Top lobe arc (R_top, Angle_top)
- Bottom lobe arc (R_bot, Angle_bot)
- Inward waist arcs (R_waist)
- Central relief boss (Dia_relief)
- Transition fillets (R_fillet)
- Shaft bore (Dia_bore)
- Extrusion thickness (Thickness)
"""

import math
from dataclasses import dataclass, field
from typing import Dict, Any, Tuple
from build123d import (
    BuildPart, BuildSketch, BuildLine,
    ThreePointArc, Circle, Mode,
    make_face, fillet, extrude,
    export_step, export_stl, Part
)


@dataclass
class WeaponGeometry:
    r_top: float = 63.0         # mm (top lobe radius)
    angle_top: float = 40.0     # deg (top lobe angle)
    r_bot: float = 45.0         # mm (bottom lobe radius)
    angle_bot: float = 100.0    # deg (bottom lobe angle)
    r_waist: float = 80.0       # mm (connecting waist arc radius)
    dia_relief: float = 50.0    # mm (center boss diameter)
    fillet_r: float = 15.0      # mm (transition fillet radius)
    dia_bore: float = 12.0      # mm (center shaft bore)
    thickness: float = 15.0     # mm (plate thickness)
    density: float = 0.00785    # g/mm³ (7.85 g/cm³ for Hardox / Alloy Steel)

    def _calculate_waist_arc_midpoint(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        r_waist: float,
        is_right: bool = True
    ) -> Tuple[float, float]:
        """Compute the midpoint on the inward-curving circular waist arc."""
        p1x, p1y = p1
        p2x, p2y = p2

        mx = (p1x + p2x) / 2.0
        my = (p1y + p2y) / 2.0

        chord = math.hypot(p2x - p1x, p2y - p1y)
        if chord >= 2.0 * r_waist:
            raise ValueError(f"Chord distance {chord:.2f} mm exceeds arc diameter {2*r_waist:.2f} mm")

        h = math.sqrt(r_waist * r_waist - (chord / 2.0) ** 2)
        ux = (p2x - p1x) / chord
        uy = (p2y - p1y) / chord

        # Normal perpendicular to chord
        nx, ny = -uy, ux
        if is_right and nx < 0:
            nx, ny = -nx, -ny
        elif not is_right and nx > 0:
            nx, ny = -nx, -ny

        # Circle center (pointing outside)
        cx = mx + h * nx
        cy = my + h * ny

        # Arc midpoint (bows inward toward the origin)
        return (cx - r_waist * nx, cy - r_waist * ny)

    def build(self) -> Part:
        """Construct the 3D Part object via build123d."""
        # Top arc endpoints
        ang_top_half = math.radians(self.angle_top / 2.0)
        theta_tr = math.pi / 2.0 - ang_top_half
        theta_tl = math.pi / 2.0 + ang_top_half
        p_tr = (self.r_top * math.cos(theta_tr), self.r_top * math.sin(theta_tr))
        p_tl = (self.r_top * math.cos(theta_tl), self.r_top * math.sin(theta_tl))
        p_top_mid = (0.0, self.r_top)

        # Bottom arc endpoints
        ang_bot_half = math.radians(self.angle_bot / 2.0)
        theta_br = -math.pi / 2.0 + ang_bot_half
        theta_bl = -math.pi / 2.0 - ang_bot_half
        p_br = (self.r_bot * math.cos(theta_br), self.r_bot * math.sin(theta_br))
        p_bl = (self.r_bot * math.cos(theta_bl), self.r_bot * math.sin(theta_bl))
        p_bot_mid = (0.0, -self.r_bot)

        # Waist midpoints
        p_waist_r_mid = self._calculate_waist_arc_midpoint(p_tr, p_br, self.r_waist, is_right=True)
        p_waist_l_mid = (-p_waist_r_mid[0], p_waist_r_mid[1])

        r_relief = self.dia_relief / 2.0

        with BuildPart() as wp:
            with BuildSketch() as sketch:
                # 1. Main outer lobe wire
                with BuildLine():
                    ThreePointArc(p_tr, p_top_mid, p_tl)
                    ThreePointArc(p_tl, p_waist_l_mid, p_bl)
                    ThreePointArc(p_bl, p_bot_mid, p_br)
                    ThreePointArc(p_br, p_waist_r_mid, p_tr)
                make_face()

                # 2. Add central relief disk
                Circle(radius=r_relief, mode=Mode.ADD)

                # 3. Fillet the 4 transition vertices where relief circle meets waist
                junction_vertices = [
                    v for v in sketch.vertices()
                    if abs(math.hypot(v.X, v.Y) - r_relief) < 3.0
                ]
                if junction_vertices and self.fillet_r > 0.0:
                    try:
                        fillet(junction_vertices, radius=self.fillet_r)
                    except Exception:
                        pass

                # 4. Central bore cutout
                Circle(radius=self.dia_bore / 2.0, mode=Mode.SUBTRACT)

            # 5. Extrude into 3D solid
            extrude(amount=self.thickness)

        return wp.part

    def evaluate_com(self) -> Tuple[float, float]:
        """Return (offset_x, offset_y) of Center of Mass from origin in mm."""
        part = self.build()
        c = part.center()
        return (c.X, c.Y)

    def mass_properties(self, rpm: float = 8000.0) -> Dict[str, Any]:
        """Compute full physical and inertial mass properties."""
        part = self.build()
        vol = part.volume  # mm³
        mass_g = vol * self.density  # g
        mass_kg = mass_g / 1000.0
        com = part.center()  # mm

        # Matrix of inertia from OpenCASCADE is volume inertia in mm⁵
        # Multiply by density (g/mm³) -> g·mm² -> convert to kg·m² (1 kg·m² = 1e9 g·mm²)
        mat = part.matrix_of_inertia
        scale_to_kg_m2 = (self.density * 1e-9)
        ixx = mat[0][0] * scale_to_kg_m2
        iyy = mat[1][1] * scale_to_kg_m2
        izz = mat[2][2] * scale_to_kg_m2

        # Kinetic energy about Z-axis at target RPM: E = 0.5 * I * omega²
        omega_rad_s = (rpm * 2.0 * math.pi) / 60.0
        energy_joules = 0.5 * izz * (omega_rad_s ** 2)

        return {
            "volume_mm3": vol,
            "mass_g": mass_g,
            "mass_kg": mass_kg,
            "com_x": com.X,
            "com_y": com.Y,
            "com_z": com.Z,
            "com_offset_mm": math.hypot(com.X, com.Y),
            "ixx_kg_m2": ixx,
            "iyy_kg_m2": iyy,
            "izz_kg_m2": izz,
            "rpm": rpm,
            "kinetic_energy_j": energy_joules
        }

    def export(self, prefix: str = "weapon_disc"):
        """Export STEP and STL formats."""
        part = self.build()
        step_path = f"{prefix}.step"
        stl_path = f"{prefix}.stl"
        export_step(part, step_path)
        export_stl(part, stl_path)
        return step_path, stl_path
