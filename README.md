# SolidSpin — Asymmetric Weapon Balancer for SolidWorks

**SolidSpin** is a high-performance SolidWorks automation plugin and desktop utility engineered to balance asymmetric kinetic spinner weapons for combat robots (discs, single-tooth bars, asymmetric drums, eggbeaters).

Based on the numerical optimization concepts of the ARCHMASTER Fusion 360 balancer by Soso Chkhortolia, rebuilt and optimized specifically for **SolidWorks 2020+ on Windows**.

---

## Key Features

- **⚡ Instant Connection (< 0.01s):** Directly attaches to active SolidWorks documents.
- **🔍 Pure Automatic Dimension Discovery:** Automatically scans absorbed and unabsorbed sketches via property-first COM traversal (`GetFirstDisplayDimension` / `GetNextDisplayDimension`), enumerating all driving sketch dimensions with live values and angular unit detection (degrees / mm).
- **🖥️ Modern 2-Column Desktop GUI:** Controls and dimension management on the left; full-height, dedicated real-time solve logs and telemetry on the right.
- **🎯 Checkbox-Driven Tuning:** Simply check or uncheck which dimensions SolidSpin is permitted to adjust — zero manual dimension typing or name-guessing required.
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

1. Open your part in **SolidWorks**.
2. Run **SolidSpin**:
   - **Option 1**: Double-click `run_balancer.bat`
   - **Option 2**: In PowerShell or terminal:
     ```powershell
     python run_balancer.py
     ```
3. In the SolidSpin window:
   - Click **Connect / Refresh Model** (connects instantly and auto-populates all driving sketch dimensions).
   - Select which dimensions to adjust using the checkbuttons.
   - Click **Re-Analyse Sensitivity** to inspect each dimension's balancing authority ($mm/\%$).
   - Click **⚡ Balance Weapon** to bring the Center of Mass onto the spin axis!

---

## Automatic Dimension Discovery

SolidSpin automatically inspects the active SolidWorks model's feature tree:
- Traverses all features and their absorbed sub-sketches (e.g. sketches absorbed under Boss-Extrusions, Cuts, Revolves).
- Traverses all top-level unabsorbed sketches.
- Automatically discovers all driving display dimensions, extracts live values, and detects whether dimensions are linear (mm) or angular (deg).
- Driven/reference dimensions (`swDimensionDriven = 1`) are automatically ignored to protect model integrity.

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
    └── main.py            # Package entry point
```

---

## License

MIT License — free to use, modify, and distribute for combat robotics and CAD design.
