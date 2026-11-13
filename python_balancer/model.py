"""
Parametric Combat Robotics Weapon Disc Model.
Formulated around:
1. Spin Axis Inertia (Izz, target kinetic energy, gyro stability)
2. True Geometric & Dynamic Bite (including waist arc simulation)
3. Tooth, Gullet, Neck Stress & Dead-Axle Bearing/Shaft Mechanics
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
    # 1. Tooth & Gullet Geometry (Max R = 72.0 mm / Ø144 mm envelope)
    r_tooth: float = 72.0         # mm (outer impact tooth radius)
    angle_tooth: float = 40.0     # deg (tooth tip engagement arc)
    r_waist: float = 150.0        # mm (gullet transition arc radius)
    dia_relief: float = 50.0      # mm (central hub / gullet root diameter)
    fillet_r: float = 18.0        # mm (requested gullet root fillet radius)

    # 2. Bite Clearance & Counterweight
    r_counter: float = 60.0       # mm (counterweight lobe radius, step h = 12.0 mm)
    angle_counter: float = 83.8   # deg (counterweight arc angle - solved by balancer)

    # 3. Spin Axis & Central Bore (Ø22.0 mm for HK1612/HK1614 needle bearings)
    dia_bore: float = 22.0        # mm (straight through-bore)
    thickness: float = 30.61      # mm (adjusted to hit target mass)
    density: float = 0.00785      # g/mm³ (7.85 g/cm³ for EN24 / 4340 steel)

    # Track actual fillet achieved during OpenCASCADE build
    actual_fillet_r: float = field(default=0.0, init=False)

    # --------------------------------------------------------------------------
    # PILLAR 1: BITE METRICS & 2D KINEMATIC SIMULATION
    # --------------------------------------------------------------------------
    @property
    def bite_h(self) -> float:
        """Radial step between primary tooth radius and counterweight radius (mm)."""
        return self.r_tooth - self.r_counter

    @bite_h.setter
    def bite_h(self, target_h: float):
        self.r_counter = self.r_tooth - target_h

    def dynamic_bite(self, robot_closing_speed_m_s: float, rpm: float) -> float:
        """Theoretical un-capped advance per revolution (mm) for n=1 tooth."""
        if rpm <= 0:
            return 0.0
        revs_per_sec = rpm / 60.0
        return (robot_closing_speed_m_s * 1000.0) / revs_per_sec

    def _profile_radius_at_angle(self, theta_rad: float) -> float:
        """
        Analytical polar radius R(theta) of the outer profile (excluding bore)
        for fast kinematic bite simulation. Angle theta_rad is measured from +X.
        """
        x = math.cos(theta_rad)
        y = math.sin(theta_rad)
        phi = math.atan2(y, abs(x))  # in [-pi/2, +pi/2]

        ang_top_half = math.radians(self.angle_tooth / 2.0)
        ang_bot_half = math.radians(self.angle_counter / 2.0)

        if phi >= (math.pi / 2.0 - ang_top_half):
            return self.r_tooth
        if phi <= (-math.pi / 2.0 + ang_bot_half):
            return self.r_counter

        # Waist arc intersection along ray at angle phi
        theta_tr = math.pi / 2.0 - ang_top_half
        p_tr = (self.r_tooth * math.cos(theta_tr), self.r_tooth * math.sin(theta_tr))
        theta_br = -math.pi / 2.0 + ang_bot_half
        p_br = (self.r_counter * math.cos(theta_br), self.r_counter * math.sin(theta_br))

        mx = (p_tr[0] + p_br[0]) / 2.0
        my = (p_tr[1] + p_br[1]) / 2.0
        chord = math.hypot(p_br[0] - p_tr[0], p_br[1] - p_tr[1])
        if chord >= 2.0 * self.r_waist:
            return max(self.dia_relief / 2.0, self.r_counter)

        h = math.sqrt(self.r_waist ** 2 - (chord / 2.0) ** 2)
        ux = (p_br[0] - p_tr[0]) / chord
        uy = (p_br[1] - p_tr[1]) / chord
        nx, ny = -uy, ux
        if nx < 0:
            nx, ny = -nx, -ny
        cx = mx + h * nx
        cy = my + h * ny

        # Ray from origin: P(r) = (r*cos(phi), r*sin(phi))
        # |P(r) - C|^2 = r_waist^2  =>  r^2 - 2*r*(cx*cos + cy*sin) + (cx^2 + cy^2 - r_waist^2) = 0
        b_half = cx * math.cos(phi) + cy * math.sin(phi)
        c_val = cx ** 2 + cy ** 2 - self.r_waist ** 2
        disc = b_half ** 2 - c_val
        r_ray = self.dia_relief / 2.0
        if disc >= 0:
            r_candidate = b_half - math.sqrt(disc)
            if r_candidate > 0:
                r_ray = max(r_ray, r_candidate)
        return min(self.r_tooth, r_ray)

    def simulate_real_bite(self, closing_speed_m_s: float, rpm: float, n_samples: int = 180) -> Dict[str, float]:
        """
        Simulate opponent wall advancing radially inward at v_close while weapon spins at RPM.
        Computes actual max bite, average bite, and effective contact angle (deg) accounting for
        both the counterweight lobe and the R_waist gullet slope.
        """
        h_dyn = self.dynamic_bite(closing_speed_m_s, rpm)
        if h_dyn <= 0:
            return {"max_bite_mm": 0.0, "avg_bite_mm": 0.0, "bite_angle_max_deg": 0.0, "bite_angle_avg_deg": 0.0}

        ang_tooth = self.angle_tooth
        ang_top_half = math.radians(ang_tooth / 2.0)
        theta_lead = math.pi / 2.0 - ang_top_half

        max_possible_bite = h_dyn
        for i in range(1, n_samples):
            d_alpha_deg = 360.0 * (i / n_samples)
            if d_alpha_deg < ang_tooth or d_alpha_deg > 355.0:
                continue

            theta_check = theta_lead + math.radians(d_alpha_deg)
            r_body = self._profile_radius_at_angle(theta_check)
            allowed_b = (self.r_tooth - r_body) + h_dyn * (1.0 - d_alpha_deg / 360.0)
            if allowed_b < max_possible_bite:
                max_possible_bite = allowed_b

        max_bite = max(0.0, min(self.bite_h, max_possible_bite))
        frac = max_bite / h_dyn if h_dyn > 0 else 1.0
        avg_bite = max_bite * (1.0 - 0.5 * frac)

        cos_max = max(-1.0, min(1.0, 1.0 - max_bite / self.r_tooth))
        cos_avg = max(-1.0, min(1.0, 1.0 - avg_bite / self.r_tooth))

        return {
            "max_bite_mm": max_bite,
            "avg_bite_mm": avg_bite,
            "bite_angle_max_deg": math.degrees(math.acos(cos_max)),
            "bite_angle_avg_deg": math.degrees(math.acos(cos_avg)),
        }

    # --------------------------------------------------------------------------
    # PILLAR 2: TOOTH, GULLET, RAKE & NECK STRESS METRICS
    # --------------------------------------------------------------------------
    @property
    def gullet_depth(self) -> float:
        return self.r_tooth - (self.dia_relief / 2.0)

    @property
    def tooth_tip_chord(self) -> float:
        half_ang = math.radians(self.angle_tooth / 2.0)
        return 2.0 * self.r_tooth * math.sin(half_ang)

    @property
    def min_neck_width_mm(self) -> float:
        ang_top_half = math.radians(self.angle_tooth / 2.0)
        theta_tr = math.pi / 2.0 - ang_top_half
        p_tr = (self.r_tooth * math.cos(theta_tr), self.r_tooth * math.sin(theta_tr))

        ang_bot_half = math.radians(self.angle_counter / 2.0)
        theta_br = -math.pi / 2.0 + ang_bot_half
        p_br = (self.r_counter * math.cos(theta_br), self.r_counter * math.sin(theta_br))

        mx = (p_tr[0] + p_br[0]) / 2.0
        my = (p_tr[1] + p_br[1]) / 2.0
        chord = math.hypot(p_br[0] - p_tr[0], p_br[1] - p_tr[1])
        if chord >= 2.0 * self.r_waist:
            return 2.0 * p_tr[0]

        h = math.sqrt(self.r_waist ** 2 - (chord / 2.0) ** 2)
        ux = (p_br[0] - p_tr[0]) / chord
        uy = (p_br[1] - p_tr[1]) / chord
        nx, ny = -uy, ux
        if nx < 0:
            nx, ny = -nx, -ny

        cx_arc = mx + h * nx
        cy_arc = my + h * ny
        r_relief = self.dia_relief / 2.0

        if r_relief <= cy_arc <= p_tr[1]:
            return max(self.dia_relief, 2.0 * (cx_arc - self.r_waist))
        elif cy_arc < r_relief:
            disc = self.r_waist ** 2 - (r_relief - cy_arc) ** 2
            if disc >= 0:
                return max(self.dia_relief, 2.0 * (cx_arc - math.sqrt(disc)))
        return 2.0 * p_tr[0]

    @property
    def rake_angle_deg(self) -> float:
        """
        True signed Combat Tooth Rake Angle (gamma) in degrees at p_tr.
        - Negative rake (< 0): Face slopes backward into the disc body.
        - Positive rake (> 0): Face hooks forward toward the direction of rotation.
        """
        half_ang = math.radians(self.angle_tooth / 2.0)
        theta_tr = math.pi / 2.0 - half_ang
        p_tr = (self.r_tooth * math.cos(theta_tr), self.r_tooth * math.sin(theta_tr))

        ang_bot_half = math.radians(self.angle_counter / 2.0)
        theta_br = -math.pi / 2.0 + ang_bot_half
        p_br = (self.r_counter * math.cos(theta_br), self.r_counter * math.sin(theta_br))

        mx = (p_tr[0] + p_br[0]) / 2.0
        my = (p_tr[1] + p_br[1]) / 2.0
        chord = math.hypot(p_br[0] - p_tr[0], p_br[1] - p_tr[1])
        if chord >= 2.0 * self.r_waist:
            return 0.0

        h = math.sqrt(self.r_waist ** 2 - (chord / 2.0) ** 2)
        ux = (p_br[0] - p_tr[0]) / chord
        uy = (p_br[1] - p_tr[1]) / chord
        nx, ny = -uy, ux
        if nx < 0:
            nx, ny = -nx, -ny

        cx_arc = mx + h * nx
        cy_arc = my + h * ny

        rx = p_tr[0] - cx_arc
        ry = p_tr[1] - cy_arc
        r_norm = math.hypot(rx, ry)
        unx, uny = rx / r_norm, ry / r_norm
        t_face_x, t_face_y = uny, -unx
        if t_face_y > 0:
            t_face_x, t_face_y = -t_face_x, -t_face_y

        inward_r_x = -p_tr[0] / self.r_tooth
        inward_r_y = -p_tr[1] / self.r_tooth

        v_dir_x = p_tr[1] / self.r_tooth
        v_dir_y = -p_tr[0] / self.r_tooth

        dot_face_r = t_face_x * inward_r_x + t_face_y * inward_r_y
        ang_deg = math.degrees(math.acos(max(-1.0, min(1.0, dot_face_r))))

        dot_v = t_face_x * v_dir_x + t_face_y * v_dir_y
        return -ang_deg if dot_v > 0 else ang_deg

    @property
    def wedge_angle_deg(self) -> float:
        """Included tooth tip angle between leading face and outer tip land."""
        return 90.0 - self.rake_angle_deg

    # --------------------------------------------------------------------------
    # GEOMETRIC CONSTRUCTION (WITH ROBUST FILLET FALLBACK)
    # --------------------------------------------------------------------------
    def _calculate_waist_arc_midpoint(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        r_waist: float,
        is_right: bool = True
    ) -> Tuple[float, float]:
        p1x, p1y = p1
        p2x, p2y = p2
        mx = (p1x + p2x) / 2.0
        my = (p1y + p2y) / 2.0
        chord = math.hypot(p2x - p1x, p2y - p1y)
        if chord >= 2.0 * r_waist:
            raise ValueError(f"Chord {chord:.2f} mm exceeds waist diameter {2*r_waist:.2f} mm")

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
        ang_top_half = math.radians(self.angle_tooth / 2.0)
        theta_tr = math.pi / 2.0 - ang_top_half
        theta_tl = math.pi / 2.0 + ang_top_half
        p_tr = (self.r_tooth * math.cos(theta_tr), self.r_tooth * math.sin(theta_tr))
        p_tl = (self.r_tooth * math.cos(theta_tl), self.r_tooth * math.sin(theta_tl))
        p_top_mid = (0.0, self.r_tooth)

        ang_bot_half = math.radians(self.angle_counter / 2.0)
        theta_br = -math.pi / 2.0 + ang_bot_half
        theta_bl = -math.pi / 2.0 - ang_bot_half
        p_br = (self.r_counter * math.cos(theta_br), self.r_counter * math.sin(theta_br))
        p_bl = (self.r_counter * math.cos(theta_bl), self.r_counter * math.sin(theta_bl))
        p_bot_mid = (0.0, -self.r_counter)

        p_waist_r_mid = self._calculate_waist_arc_midpoint(p_tr, p_br, self.r_waist, is_right=True)
        p_waist_l_mid = (-p_waist_r_mid[0], p_waist_r_mid[1])
        r_relief = self.dia_relief / 2.0

        max_geom_fillet = min(self.fillet_r, 0.45 * self.min_neck_width_mm)
        candidate_radii = [
            max_geom_fillet,
            max_geom_fillet * 0.75,
            max_geom_fillet * 0.5,
            4.0,
            0.0
        ]

        self.actual_fillet_r = 0.0
        for r_try in candidate_radii:
            try:
                with BuildPart() as wp:
                    with BuildSketch() as sketch:
                        with BuildLine():
                            ThreePointArc(p_tr, p_top_mid, p_tl)
                            ThreePointArc(p_tl, p_waist_l_mid, p_bl)
                            ThreePointArc(p_bl, p_bot_mid, p_br)
                            ThreePointArc(p_br, p_waist_r_mid, p_tr)
                        make_face()
                        Circle(radius=r_relief, mode=Mode.ADD)

                        junction_vertices = []
                        if r_try > 0.5:
                            junction_vertices = [
                                v for v in sketch.vertices()
                                if abs(math.hypot(v.X, v.Y) - r_relief) < 4.5
                            ]
                            if junction_vertices:
                                fillet(junction_vertices, radius=r_try)

                        Circle(radius=self.dia_bore / 2.0, mode=Mode.SUBTRACT)
                    extrude(amount=self.thickness)

                if wp.part.volume > 0:
                    self.actual_fillet_r = r_try if junction_vertices else 0.0
                    return wp.part
            except Exception:
                continue

        raise RuntimeError("Failed to construct valid OpenCASCADE geometry even with zero fillet.")

    def evaluate_com(self) -> Tuple[float, float]:
        part = self.build()
        c = part.center()
        return (c.X, c.Y)

    # --------------------------------------------------------------------------
    # PILLAR 3: INERTIA, ENERGY, GYRO & STRUCTURAL CHECKS
    # --------------------------------------------------------------------------
    def mass_properties(
        self,
        target_energy_j: float = 2000.0,
        closing_speed_m_s: float = 5.364,
        robot_mass_kg: float = 8.0,
        track_width_mm: float = 350.0,
        shaft_dia_mm: float = 16.0,
        shaft_span_mm: float = 55.0,
        shaft_yield_mpa: float = 950.0,
        bearing_c0_n: float = 48400.0  # 2x HK1616 static rating (24.2 kN each = 48.4 kN total)
    ) -> Dict[str, Any]:
        part = self.build()
        vol = part.volume
        mass_g = vol * self.density
        mass_kg = mass_g / 1000.0
        com = part.center()

        mat = part.matrix_of_inertia
        izz = mat[2][2] * (self.density * 1e-9)  # kg·m²

        omega = math.sqrt(2.0 * target_energy_j / izz) if izz > 0 else 0.0
        rpm = (omega * 60.0) / (2.0 * math.pi)
        v_tip_m_s = omega * (self.r_tooth / 1000.0)
        v_tip_mph = v_tip_m_s / 0.44704

        r_tooth_m = self.r_tooth / 1000.0
        m_eff_kg = izz / (r_tooth_m ** 2) if r_tooth_m > 0 else 0.0

        h_dyn = self.dynamic_bite(closing_speed_m_s, rpm)
        bite_sim = self.simulate_real_bite(closing_speed_m_s, rpm)

        h_gyro = izz * omega
        grav_restoring_nm = robot_mass_kg * 9.81 * (track_width_mm / 2000.0)
        yaw_lift_rad_s = grav_restoring_nm / h_gyro if h_gyro > 0 else 0.0
        yaw_lift_deg_s = math.degrees(yaw_lift_rad_s)
        yaw_lift_rev_s = yaw_lift_rad_s / (2.0 * math.pi)

        f_centrifugal_n = (omega ** 2) * (0.5 * mass_kg) * (0.42 * r_tooth_m)
        hub_net_width_mm = max(1.0, self.min_neck_width_mm - self.dia_bore)
        neck_area_mm2 = hub_net_width_mm * self.thickness
        centrifugal_stress_mpa = f_centrifugal_n / neck_area_mm2

        impact_work_j = 0.35 * target_energy_j
        deform_m = 0.015
        f_impact_peak_n = impact_work_j / deform_m
        f_shaft_yield_n = (math.pi * (shaft_dia_mm ** 3) * shaft_yield_mpa) / (2.0 * shaft_span_mm)

        return {
            "izz_kg_m2": izz,
            "m_eff_kg": m_eff_kg,
            "kinetic_energy_j": target_energy_j,
            "required_rpm": rpm,
            "v_tip_m_s": v_tip_m_s,
            "v_tip_mph": v_tip_mph,
            "mass_g": mass_g,
            "mass_kg": mass_kg,
            "specific_inertia": (izz / mass_kg) if mass_kg > 0 else 0.0,

            # Bite Metrics
            "bite_h_physical_mm": self.bite_h,
            "bite_h_dynamic_mm": h_dyn,
            "sim_max_bite_mm": bite_sim["max_bite_mm"],
            "sim_avg_bite_mm": bite_sim["avg_bite_mm"],
            "bite_angle_max_deg": bite_sim["bite_angle_max_deg"],
            "bite_angle_avg_deg": bite_sim["bite_angle_avg_deg"],
            "bite_clearance_ratio": self.bite_h / h_dyn if h_dyn > 0 else 0.0,

            # Geometry & Stress Metrics
            "r_tooth_mm": self.r_tooth,
            "tooth_tip_chord_mm": self.tooth_tip_chord,
            "min_neck_width_mm": self.min_neck_width_mm,
            "rake_angle_deg": self.rake_angle_deg,
            "wedge_angle_deg": self.wedge_angle_deg,
            "gullet_depth_mm": self.gullet_depth,
            "requested_fillet_r_mm": self.fillet_r,
            "actual_fillet_r_mm": self.actual_fillet_r,
            "r_counter_mm": self.r_counter,
            "r_waist_mm": self.r_waist,
            "centrifugal_stress_mpa": centrifugal_stress_mpa,

            # Gyro & Shaft/Bearing Mechanics
            "h_gyro_nms": h_gyro,
            "grav_restoring_nm": grav_restoring_nm,
            "yaw_lift_deg_s": yaw_lift_deg_s,
            "yaw_lift_rev_s": yaw_lift_rev_s,
            "f_impact_peak_kn": f_impact_peak_n / 1000.0,
            "f_shaft_yield_kn": f_shaft_yield_n / 1000.0,
            "shaft_safety_factor": f_shaft_yield_n / f_impact_peak_n if f_impact_peak_n > 0 else 0.0,
            "bearing_c0_safety_factor": bearing_c0_n / f_impact_peak_n if f_impact_peak_n > 0 else 0.0,

            # Balance State
            "com_offset_mm": math.hypot(com.X, com.Y),
            "is_balanced": math.hypot(com.X, com.Y) < 1e-3
        }

    def set_mass_target(self, target_mass_g: float = 1500.0) -> float:
        temp_part = self.build()
        area_mm2 = temp_part.volume / self.thickness
        self.thickness = target_mass_g / (self.density * area_mm2)
        return self.thickness

    def export(self, prefix: str = "weapon_disc"):
        part = self.build()
        step_path = f"{prefix}.step"
        stl_path = f"{prefix}.stl"
        export_step(part, step_path)
        export_stl(part, stl_path)
        return (step_path, stl_path)
