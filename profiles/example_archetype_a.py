"""
Archetype A: Standard Parametric Disc
Use this format if your SolidWorks sketch has a tooth arc, counterweight arc, waist arcs, and central hub/bore.
"""

profile_data = {
    "profile_name": "Variant_R72_Standard",
    "archetype": "parametric_disc",
    # Extracted dimensions from SolidWorks:
    "r_tooth": 72.0,        # Outer tooth radius (mm)
    "angle_tooth": 40.0,    # Tooth tip angle (deg)
    "r_waist": 150.0,       # Concave gullet waist radius (mm)
    "dia_relief": 50.0,     # Central hub core diameter (mm)
    "fillet_r": 18.0,       # Gullet root fillet radius (mm)
    "r_counter": 60.0,      # Counterweight radius (mm) - sets bite step h = 12.0 mm
    "angle_counter": 83.8,  # Initial counterweight angle (deg) - free balance parameter
    "dia_bore": 22.0,       # Straight through-bore (mm)
    "free_balancing_param": "angle_counter"
}
