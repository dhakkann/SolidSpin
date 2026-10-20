"""
Parametric Combat Robotics Weapon Disc Model.
Formulated strictly around three core engineering pillars:
1. Spin Axis Inertia (Izz, kinetic energy, spin-up dynamics)
2. Bite h (tooth engagement clearance vs closing speed)
3. Tooth & Gullet Shape (tip width, gullet depth, stress relief radius)
"""

import math
from dataclasses import dataclass
from typing import Dict, Any, Tuple
from build123d import (
    BuildPart, BuildSketch, BuildLine,
    ThreePointArc, Circle, Mode,
    make_face, fillet, extrude,
    export_step, export_stl, Part
)


@dataclass
class WeaponGeometry:
    """
    Parametric weapon disc defined by:
    - Tooth & Gullet Shape: r_tooth, angle_tooth, r_waist, dia_relief, fillet_r
    - Bite h: governed by tooth radial drop relative to counterweight (r_counter)
    - Spin Axis Inertia (Izz): governed by overall mass distribution about shaft bore
    - Hub Option B: Straight through-bore for dual pressed bearings and floating spacer sleeve
    """
    # 1. Tooth & Gullet Geometry (2 kJ / <250 mph baseline)
    r_tooth: float = 97.0         # mm (outer impact tooth radius, <= 250 mph @ 11k RPM)
    angle_tooth: float = 38.0     # deg (tooth tip engagement arc)
    r_waist: float = 120.0        # mm (gullet transition arc radius)
    dia_relief: float = 50.0      # mm (central hub / gullet root diameter)
    fillet_r: float = 18.0        # mm (gullet root stress-relief fillet radius)

    # 2. Bite Clearance & Counterweight (12 mph single-tooth bite clearance)
    r_counter: float = 67.7       # mm (counterweight lobe radius, sets tooth step h = 29.3 mm)
    angle_counter: float = 100.0  # deg (counterweight arc angle)

    # 3. Spin Axis & Shaft Bore (Option B: Straight Through-Bore)
    dia_bore: float = 22.0        # mm (straight through-bore for 22mm OD bearings e.g. 608)
    thickness: float = 23.16      # mm (sets mass to exactly 1.500 kg in Hardox/Steel)
    density: float = 0.00785      # g/mm³ (7.85 g/cm³ for Hardox 450 / 500)

    # --------------------------------------------------------------------------
    # PILLAR 1: BITE h METRICS
    # --------------------------------------------------------------------------
    @property
    def bite_h(self) -> float:
        """
        Physical Bite Height / Radial Tooth Clearance:
        h = R_tooth - R_counter
        The radial step that allows the tooth to bite into the opponent before
        the counterweight lobe or hub makes contact.
        """
        return self.r_tooth - self.r_counter

    @bite_h.setter
    def bite_h(self, target_h: float):
        """Set counterweight radius to achieve the target physical bite clearance."""
        self.r_counter = self.r_tooth - target_h

    def dynamic_bite(self, robot_closing_speed_m_s: float, rpm: float) -> float:
        """
        Theoretical forward penetration per revolution (RioBotz equation):
        h_dyn = (v_robot * 60) / (rpm * n_teeth)
        """
        revs_per_sec = rpm / 60.0
        # Single tooth disc (n_teeth = 1)
        return (robot_closing_speed_m_s * 1000.0) / revs_per_sec

    # --------------------------------------------------------------------------
    # PILLAR 2: TOOTH & GULLET SHAPE METRICS
    # --------------------------------------------------------------------------
    @property
    def gullet_depth(self) -> float:
        """
        Radial depth of the gullet pocket from tooth tip to central hub:
        d_gullet = R_tooth - (Dia_relief / 2)
        Must be deep enough to swallow the target robot's armor/chassis corner.
        """
        return self.r_tooth - (self.dia_relief / 2.0)

    @property
    def tooth_tip_chord(self) -> float:
        """Linear width of the impact tooth tip."""
        half_ang = math.radians(self.angle_tooth / 2.0)
        return 2.0 * self.r_tooth * math.sin(half_ang)

    # --------------------------------------------------------------------------
    # GEOMETRIC CONSTRUCTION
    # --------------------------------------------------------------------------
    def _calculate_waist_arc_midpoint(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        r_waist: float,
        is_right: bool = True
    ) -> Tuple[float, float]:
        """Compute the midpoint on the concave gullet waist arc."""
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

        nx, ny = -uy, ux
        if is_right and nx < 0:
            nx, ny = -nx, -ny
        elif not is_right and nx > 0:
            nx, ny = -nx, -ny

        cx = mx + h * nx
        cy = my + h * ny
        return (cx - r_waist * nx, cy - r_waist * ny)

    def build(self) -> Part:
        """Construct the exact 3D Part object via build123d."""
        # Tooth arc endpoints (symmetric about +Y)
        ang_top_half = math.radians(self.angle_tooth / 2.0)
        theta_tr = math.pi / 2.0 - ang_top_half
        theta_tl = math.pi / 2.0 + ang_top_half
        p_tr = (self.r_tooth * math.cos(theta_tr), self.r_tooth * math.sin(theta_tr))
        p_tl = (self.r_tooth * math.cos(theta_tl), self.r_tooth * math.sin(theta_tl))
        p_top_mid = (0.0, self.r_tooth)

        # Counterweight arc endpoints (symmetric about -Y)
        ang_bot_half = math.radians(self.angle_counter / 2.0)
        theta_br = -math.pi / 2.0 + ang_bot_half
        theta_bl = -math.pi / 2.0 - ang_bot_half
        p_br = (self.r_counter * math.cos(theta_br), self.r_counter * math.sin(theta_br))
        p_bl = (self.r_counter * math.cos(theta_bl), self.r_counter * math.sin(theta_bl))
        p_bot_mid = (0.0, -self.r_counter)

        # Gullet waist midpoints
        p_waist_r_mid = self._calculate_waist_arc_midpoint(p_tr, p_br, self.r_waist, is_right=True)
        p_waist_l_mid = (-p_waist_r_mid[0], p_waist_r_mid[1])

        r_relief = self.dia_relief / 2.0

        with BuildPart() as wp:
            with BuildSketch() as sketch:
                with BuildLine():
                    ThreePointArc(p_tr, p_top_mid, p_tl)
                    ThreePointArc(p_tl, p_waist_l_mid, p_bl)
                    ThreePointArc(p_bl, p_bot_mid, p_br)
                    ThreePointArc(p_br, p_waist_r_mid, p_tr)
                make_face()

                # Central gullet root hub
                Circle(radius=r_relief, mode=Mode.ADD)

                # Fillet gullet root transition vertices
                junction_vertices = [
                    v for v in sketch.vertices()
                    if abs(math.hypot(v.X, v.Y) - r_relief) < 3.0
                ]
                if junction_vertices and self.fillet_r > 0.0:
                    try:
                        fillet(junction_vertices, radius=self.fillet_r)
                    except Exception:
                        pass

                # Spin axis shaft bore
                Circle(radius=self.dia_bore / 2.0, mode=Mode.SUBTRACT)

            extrude(amount=self.thickness)

        return wp.part

    def evaluate_com(self) -> Tuple[float, float]:
        """Return (com_x, com_y) offset from spin axis in mm."""
        part = self.build()
        c = part.center()
        return (c.X, c.Y)

    # --------------------------------------------------------------------------
    # PILLAR 3: SPIN AXIS INERTIA (Izz) METRICS
    # --------------------------------------------------------------------------
    def mass_properties(self, rpm: float = 8000.0, closing_speed_m_s: float = 3.5) -> Dict[str, Any]:
        """Compute metrics focusing strictly on Inertia, Bite, and Tooth/Gullet shape."""
        part = self.build()
        vol = part.volume
        mass_g = vol * self.density
        mass_kg = mass_g / 1000.0
        com = part.center()

        # Matrix of inertia from OpenCASCADE is volume inertia in mm⁵
        # Scale to kg·m²: multiply by (density in g/mm³ * 1e-9)
        mat = part.matrix_of_inertia
        scale_to_kg_m2 = (self.density * 1e-9)
        izz = mat[2][2] * scale_to_kg_m2  # Spin Axis Inertia

        omega = (rpm * 2.0 * math.pi) / 60.0
        energy_j = 0.5 * izz * (omega ** 2)
        h_dyn = self.dynamic_bite(closing_speed_m_s, rpm)

        return {
            # Pillar 1: Spin Axis Inertia
            "izz_kg_m2": izz,
            "kinetic_energy_j": energy_j,
            "mass_g": mass_g,
            "mass_kg": mass_kg,
            "rpm": rpm,
            "specific_inertia": (izz / mass_kg) if mass_kg > 0 else 0,  # Inertial efficiency (m²)

            # Pillar 2: Bite h
            "bite_h_physical_mm": self.bite_h,
            "bite_h_dynamic_mm": h_dyn,
            "bite_clearance_ratio": self.bite_h / h_dyn if h_dyn > 0 else 0,

            # Pillar 3: Tooth & Gullet Shape
            "r_tooth_mm": self.r_tooth,
            "tooth_tip_chord_mm": self.tooth_tip_chord,
            "gullet_depth_mm": self.gullet_depth,
            "fillet_r_mm": self.fillet_r,
            "r_counter_mm": self.r_counter,
            "r_waist_mm": self.r_waist,

            # Balance State
            "com_offset_mm": math.hypot(com.X, com.Y),
            "is_balanced": math.hypot(com.X, com.Y) < 1e-4
        }

    def set_mass_target(self, target_mass_g: float = 1500.0) -> float:
        """Adjust thickness to hit the exact target mass (e.g. 1500 g)."""
        temp_part = self.build()
        area_mm2 = temp_part.volume / self.thickness
        self.thickness = target_mass_g / (self.density * area_mm2)
        return self.thickness

    def build_bearing_spacer(
        self,
        bearing_width: float = 7.0,
        shaft_dia: float = 8.0,
        spacer_wall: float = 2.5
    ) -> Part:
        """
        Construct matching internal floating bearing spacer sleeve for Option B.
        Length spans between the two pressed bearings across total plate thickness.
        """
        spacer_len = self.thickness - (2.0 * bearing_width)
        if spacer_len <= 1.0:
            raise ValueError(f"Plate thickness ({self.thickness:.1f} mm) too thin for dual {bearing_width} mm bearings")

        inner_r = (shaft_dia + 0.3) / 2.0  # slight clearance for shaft bolt
        outer_r = inner_r + spacer_wall    # contacts bearing inner race only

        with BuildPart() as sp:
            with BuildSketch():
                Circle(radius=outer_r)
                Circle(radius=inner_r, mode=Mode.SUBTRACT)
            extrude(amount=spacer_len)

        return sp.part

    def export(self, prefix: str = "weapon_disc", include_spacer: bool = True):
        """Export STEP and STL files for the disc and Option B bearing spacer."""
        part = self.build()
        step_path = f"{prefix}.step"
        stl_path = f"{prefix}.stl"
        export_step(part, step_path)
        export_stl(part, stl_path)

        exports = [step_path, stl_path]
        if include_spacer and self.thickness > 15.0:
            try:
                spacer = self.build_bearing_spacer()
                sp_step = f"{prefix}_bearing_spacer.step"
                sp_stl = f"{prefix}_bearing_spacer.stl"
                export_step(spacer, sp_step)
                export_stl(spacer, sp_stl)
                exports.extend([sp_step, sp_stl])
            except Exception:
                pass

        return tuple(exports)
