# SolidSpin — Asymmetric Weapon Balancer for SolidWorks

**SolidSpin** is a high-performance SolidWorks automation plugin and desktop utility engineered to balance asymmetric kinetic spinner weapons for combat robots (discs, single-tooth bars, asymmetric drums, eggbeaters).

Based on the numerical optimization concepts of the ARCHMASTER Fusion 360 balancer by Soso Chkhortolia, rebuilt and optimized specifically for **SolidWorks 2020+ on Windows**.

---

## Key Features

- **⚡ Instant Connection (< 0.01s):** Directly attaches to active SolidWorks documents.
- **🔍 Automatic Sketch Dimension Discovery:** Automatically scans absorbed and unabsorbed sketches via property-first COM traversal (`GetFirstDisplayDimension` / `GetNextDisplayDimension`), enumerating driving dimensions (e.g., `OuterRadius`, `ToothAngle`, `Bore`, `ReliefDia`) with live values and angular units.
- **🖥️ Modern 2-Column Desktop GUI:** Controls, dimension management, and quick actions on the left; full-height, dedicated real-time solve logs and telemetry on the right.
- **🎯 Targeted Dimension Management:** Auto-scan dimensions, add custom dimensions by name, pick directly from CAD with multi-selection support, and remove with one click.
- **💾 Persistent Part Memory:** Remembers configured dimensions for each part file automatically (`saved_dimensions.json`).
- **📐 Default Z-Axis Spin:** Defaults to Z-Axis through the CAD origin $(0,0,0)$ with instant switching to X/Y or custom coordinates.
- **🔬 Sensitivity Analysis & Controllability:** Probes in-plane mass movement ($mm/\%$) and evaluates SVD controllability to prevent singular balancing attempts.
- **⚙️ Numerical Optimization:** Minimum-norm damped Gauss-Newton with Broyden rank-1 updates (multi-D) and Illinois bracket root-finding (1D).
- **🛡️ Safe & Reversible:** Supports one-click **Undo** to restore original dimension values instantly.

---

## Requirements

- **SolidWorks 2020** or newer (running on Windows).
- **Python 3.10+** (64-bit).
- Dependencies: `pywin32` (installed via `pip install -r requirements.txt`).

---

## How to Run

1. Open your weapon part in **SolidWorks**.
2. Run **SolidSpin**:
   - **Option 1**: Double-click `run_balancer.bat`
   - **Option 2**: In PowerShell or terminal:
     ```powershell
     python run_balancer.py
     ```
3. In the SolidSpin window:
   - Click **Connect / Refresh Model** (connects instantly).
   - Enter your tuning dimension names (e.g. `OuterRadius@Sketch1`, `ToothAngle`) or click dimensions in SolidWorks and click **Pick From CAD Selection**.
   - Click **Re-Analyse Sensitivity** to inspect dimension authority.
   - Click **⚡ Balance Weapon** to bring the Center of Mass onto the spin axis!

---

## Common Dimension Patterns Reference

SolidSpin lets you balance weapons by tweaking any sketch or feature dimensions. Common dimensions builders balance with include:

| Dimension Pattern | Typical Geometry | Notes |
|---|---|---|
| `OuterRadius@Sketch1` | Weapon disc outer radius / lobe | Primary radial balance control |
| `ToothAngle@Sketch1` or `ToothAngle` | Weapon tooth impact angle | Angular mass distribution (deg) |
| `ToothWidth@Sketch1` | Tooth base / tip width | Fine kinetic weight trimming |
| `ToothDepth@Sketch1` | Impact bite depth | High authority on COM radius |
| `D1@Boss-Extrude1` | Weapon plate thickness | Feature extrusion depth |
| `D1@Fillet1` | Stress-relief fillet radius | Weight relief trimming |
| `Bore@Sketch1` | Center bearing bore | Center datum reference |
| `InnerRadius@Sketch1` | Counterweight pocket radius | Pocket mass removal |
| `D1@Sketch1`, `D2@Sketch1` | Default SolidWorks sketch dimensions | Generic driving dimensions |

You can quickly select any of these from the **Quick Templates** dropdown in the UI!

---

## Project Structure

```
SW Balancer Plugin/
├── run_balancer.py        # Top-level runner script
├── run_balancer.bat       # Windows double-click launcher
├── requirements.txt       # Dependencies (pywin32)
├── LICENSE                # MIT License
├── README.md              # Project documentation
├── tests/
│   ├── test_balancer_math.py # Standalone mathematical solver unit tests
│   └── test_sw_bridge.py     # COM bridge property-first dimension discovery tests
└── sw_balancer/
    ├── geometry.py        # SpinAxis 3D coordinate math & projections
    ├── solver.py          # Gauss-Newton & root-finding optimization algorithms
    ├── sw_bridge.py       # SolidWorks COM automation bridge (win32com)
    ├── ui.py              # 2-Column Tkinter desktop GUI
    ├── saved_dimensions.json # Dimension persistence per CAD part
    └── main.py            # Package entry point
```

---

## License

MIT License — free to use, modify, and distribute for combat robotics and CAD design.
