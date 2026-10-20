# SolidSpin Python Optimizer: Spin Axis Inertia, Bite h, & Tooth/Gullet Shape

A specialized CAD optimizer for combat robotics spinners, designed specifically around the **three physical drivers of weapon performance**:

1. **Spin Axis Inertia ($I_{zz}$)** — Kinetic Energy & Spin-Up Dynamics
2. **Bite ($h$)** — Engagement Depth & Anti-Rub Clearance
3. **Tooth and Gullet Shape** — Impact Delivery & Stress Distribution

---

## The Three Pillars Described

```
                             ▲ +Y (Spin Axis = Z)
                             │
                      ┌──────┴──────┐   ← Tooth Tip Arc (R_tooth = 63 mm)
                     /   TOOTH TIP   \    Chord Width & Impact Face
                    /                 \
                   │    GULLET DEPTH   │ ← Gullet (swallows opponent armor)
      Waist Arc    │   d_g = 38.0 mm   │   Waist Arc (R_waist = 80 mm)
   (R_waist = 80)   \                 /
                     \   ROOT FILLET /    R15 Fillet (reduces peak impact stress)
                      └───┐  Ø50  ┌───┘
                          │ (HUB) │       Bite Clearance Drop:
                          └───┬───┘       h = R_tooth - R_counter = 18.0 mm
                             /│\
                            / │ \
                           /  │  \
                          └───┴───┘       ← Counterweight Lobe (R_counter = 45 mm)
                             ▼ -Y
```

### 1. Spin Axis Inertia ($I_{zz}$)
- **Kinetic Energy**: The weapon's hitting energy is $E_k = \frac{1}{2} I_{zz} \omega^2$.
- **Mass Placement**: Centrifugal inertia scales with radius squared ($r^2$). Material placed at the outer tooth radius ($R_{tooth}$) contributes drastically more energy per gram than material near the center bore.
- **Specific Inertia ($I_{zz} / M$)**: Measures inertial efficiency ($m^2$). The optimizer maximizes stored kinetic energy for a given robot weight budget while maintaining $COM = (0,0)$.

### 2. Bite ($h$)
- **Theoretical Forward Bite**: At robot closing speed $v_{robot}$ and rotational speed $\omega$, the forward advance per revolution is:
  $$h_{dyn} = \frac{v_{robot} \cdot 60}{\text{RPM} \cdot N_{teeth}}$$
  For a single-tooth disc ($N=1$) charging at $3.5\text{ m/s}$ at $8000\text{ RPM}$, $h_{dyn} = 26.25\text{ mm}$.
- **Physical Clearance ($h = R_{tooth} - R_{counter}$)**: If the radial drop from the tooth tip to the counterweight is smaller than $h_{dyn}$, the counterweight lobe slaps against the opponent before the tooth can engage deeply, leading to "weapon skating" and glancing hits.
- **The Optimizer's Role**: Enforces the required physical clearance $h$ while automatically adjusting the lobe angles so the disc stays dynamically balanced.

### 3. Tooth and Gullet Shape
- **Tooth Tip**: Defined by tip radius $R_{tooth}$ and chord width. Must be wide enough to withstand shear fracture upon slamming into Hardox/AR500 plate, but narrow enough to concentrate contact pressure.
- **Gullet Throat & Depth**: The recess in front of the impact face ($d_g = R_{tooth} - R_{hub}$). It provides the physical pocket needed to swallow the opponent's chassis corner during the bite window.
- **Root Transition Fillet ($R_{fillet}$)**: Sharp internal corners at the gullet root create extreme notch stress concentrations during shock loading. A generous $R15\text{ mm}$ fillet smoothly blends the gullet waist into the hub, diffusing impact forces across the core.

---

## Running the Optimizer

### 1. Balance for a Specific Bite $h$
Enforces a target bite (e.g. $h = 18\text{ mm}$), keeps the tooth profile intact, and balances in $< 1\text{ second}$:

```bash
python run_py_balancer.py --bite 18.0
```

### 2. Pareto Curve: Bite $h$ vs. Spin Axis Inertia ($I_{zz}$)
Evaluates the physical trade-off between deeper bite clearance and stored kinetic energy:

```bash
python run_py_balancer.py --pareto
```
