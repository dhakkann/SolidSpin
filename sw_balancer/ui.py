"""
Tkinter User Interface for SolidWorks Asymmetric Weapon Balancer.
Provides:
- 2-Column Desktop layout (Controls on Left, Live Logs on Right)
- Instant SolidWorks connection (0.01s, zero slow feature tree traversal)
- Default Z-Axis through Origin spin axis with quick X/Y/Custom switches
- Targeted Dimension Management (+ Add Dimension, Pick From CAD, Delete ✕)
- Persistent dimension memory per CAD part
- Dimension sensitivity analysis & real-time controllability check
- Responsive optimization solver with Cancel and Undo support
"""

import json
import math
import os
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext

from .geometry import SpinAxis, _unit, M_TO_MM
from .solver import (
    COM_TOL_MM,
    SolveError,
    Cancelled,
    probe_sensitivity,
    controllability,
    rate_dimensions,
    solve,
    solve_multi,
    control_lines,
    control_blocked,
)
from .sw_bridge import (
    SolidWorksConnection,
    SuspendGraphicsContext,
    get_all_solid_bodies,
    get_mass_properties,
    get_sketch_dimensions,
    find_dimension_by_name,
    get_dimensions_from_selection,
    rebuild_model,
    make_evaluator,
    safe_call,
)

SETTINGS_FILE = os.path.join(os.path.dirname(__file__), "saved_dimensions.json")


def load_saved_dim_names(part_title):
    """Load previously saved dimension names for a specific part."""
    try:
        if os.path.exists(SETTINGS_FILE):
            with open(SETTINGS_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
                return data.get(part_title, [])
    except Exception:
        pass
    return []


def save_dim_names(part_title, dim_names):
    """Persist dimension names for a specific part so they reload across sessions."""
    try:
        data = {}
        if os.path.exists(SETTINGS_FILE):
            try:
                with open(SETTINGS_FILE, "r", encoding="utf-8") as f:
                    data = json.load(f)
            except Exception:
                data = {}
        data[part_title] = dim_names
        with open(SETTINGS_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
    except Exception:
        pass


class CustomAxisDialog(tk.Toplevel):
    """Dialog allowing user to enter coordinates and direction for a custom spin axis."""

    def __init__(self, parent, current_axis=None):
        super().__init__(parent)
        self.title("Specify Custom Spin Axis")
        self.geometry("420x340")
        self.resizable(False, False)
        self.configure(bg="#f4f5f7")
        self.result = None

        init_origin = (0.0, 0.0, 0.0)
        init_dir = (0.0, 0.0, 1.0)
        if current_axis is not None:
            init_origin = (
                current_axis.origin[0] * M_TO_MM,
                current_axis.origin[1] * M_TO_MM,
                current_axis.origin[2] * M_TO_MM
            )
            init_dir = current_axis.direction

        pad = {"padx": 10, "pady": 4}

        frm_orig = ttk.LabelFrame(self, text="Axis Point / Origin (mm)", padding=8)
        frm_orig.pack(fill=tk.X, padx=12, pady=6)

        ttk.Label(frm_orig, text="X:").grid(row=0, column=0, **pad)
        self.ent_ox = ttk.Entry(frm_orig, width=10)
        self.ent_ox.insert(0, f"{init_origin[0]:.2f}")
        self.ent_ox.grid(row=0, column=1, **pad)

        ttk.Label(frm_orig, text="Y:").grid(row=0, column=2, **pad)
        self.ent_oy = ttk.Entry(frm_orig, width=10)
        self.ent_oy.insert(0, f"{init_origin[1]:.2f}")
        self.ent_oy.grid(row=0, column=3, **pad)

        ttk.Label(frm_orig, text="Z:").grid(row=0, column=4, **pad)
        self.ent_oz = ttk.Entry(frm_orig, width=10)
        self.ent_oz.insert(0, f"{init_origin[2]:.2f}")
        self.ent_oz.grid(row=0, column=5, **pad)

        frm_dir = ttk.LabelFrame(self, text="Axis Direction Vector (unit vector)", padding=8)
        frm_dir.pack(fill=tk.X, padx=12, pady=6)

        ttk.Label(frm_dir, text="dX:").grid(row=0, column=0, **pad)
        self.ent_dx = ttk.Entry(frm_dir, width=10)
        self.ent_dx.insert(0, f"{init_dir[0]:.3f}")
        self.ent_dx.grid(row=0, column=1, **pad)

        ttk.Label(frm_dir, text="dY:").grid(row=0, column=2, **pad)
        self.ent_dy = ttk.Entry(frm_dir, width=10)
        self.ent_dy.insert(0, f"{init_dir[1]:.3f}")
        self.ent_dy.grid(row=0, column=3, **pad)

        ttk.Label(frm_dir, text="dZ:").grid(row=0, column=4, **pad)
        self.ent_dz = ttk.Entry(frm_dir, width=10)
        self.ent_dz.insert(0, f"{init_dir[2]:.3f}")
        self.ent_dz.grid(row=0, column=5, **pad)

        btn_box = ttk.Frame(self)
        btn_box.pack(pady=12)

        ttk.Button(btn_box, text="Set Spin Axis", style="Primary.TButton", command=self.on_ok).pack(side=tk.LEFT, padx=6)
        ttk.Button(btn_box, text="Cancel", command=self.destroy).pack(side=tk.LEFT, padx=6)

        self.transient(parent)
        self.grab_set()
        parent.wait_window(self)

    def on_ok(self):
        try:
            ox = float(self.ent_ox.get())
            oy = float(self.ent_oy.get())
            oz = float(self.ent_oz.get())
            dx = float(self.ent_dx.get())
            dy = float(self.ent_dy.get())
            dz = float(self.ent_dz.get())
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid numeric values for all coordinates.")
            return

        mag = math.sqrt(dx * dx + dy * dy + dz * dz)
        if mag < 1e-9:
            messagebox.showerror("Invalid Vector", "Direction vector cannot have zero length.")
            return

        dir_unit = (dx / mag, dy / mag, dz / mag)
        origin_m = (ox / M_TO_MM, oy / M_TO_MM, oz / M_TO_MM)
        label = f"Custom: Pt ({ox:.1f}, {oy:.1f}, {oz:.1f}) mm, Dir ({dir_unit[0]:.2f}, {dir_unit[1]:.2f}, {dir_unit[2]:.2f})"
        self.result = SpinAxis(origin_m, dir_unit, label)
        self.destroy()


class BalancerApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("SolidSpin — SolidWorks Asymmetric Weapon Balancer")
        self.geometry("1120x700")
        self.minsize(980, 580)

        self.sw_conn = SolidWorksConnection()
        self.spin_axis = SpinAxis((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), "Z-Axis through Origin (0,0,0)")
        self.solid_bodies = []
        self.dimensions = []
        self.analysis_cache = None
        self.undo_state = None  # {dim_name: (dim_item, original_value)}
        self.cancel_requested = False

        self._init_style()
        self._build_ui()

    def _init_style(self):
        self.style = ttk.Style(self)
        self.style.theme_use("clam")
        self.configure(bg="#f4f5f7")

        self.style.configure(".", background="#f4f5f7", font=("Segoe UI", 9))
        self.style.configure("TLabel", background="#f4f5f7")
        self.style.configure("TLabelframe", background="#f4f5f7")
        self.style.configure("TLabelframe.Label", background="#f4f5f7", font=("Segoe UI", 10, "bold"), foreground="#2d3748")
        self.style.configure("Header.TLabel", font=("Segoe UI", 13, "bold"), foreground="#1a365d")
        self.style.configure("Subheader.TLabel", font=("Segoe UI", 9, "italic"), foreground="#4a5568")
        self.style.configure("Action.TButton", font=("Segoe UI", 9, "bold"), padding=5)
        self.style.configure("Primary.TButton", font=("Segoe UI", 10, "bold"), padding=6)
        self.style.configure("White.TFrame", background="#ffffff")

    def _build_ui(self):
        # Top banner
        top_frame = ttk.Frame(self, padding=(14, 10, 14, 4))
        top_frame.pack(fill=tk.X)

        title_lbl = ttk.Label(top_frame, text="SolidSpin — Weapon Balancer for SolidWorks", style="Header.TLabel")
        title_lbl.pack(anchor=tk.W)
        subtitle_lbl = ttk.Label(
            top_frame,
            text="Adjusts selected sketch dimensions until the weapon's centre of mass sits exactly on its spin axis.",
            style="Subheader.TLabel"
        )
        subtitle_lbl.pack(anchor=tk.W)

        # Main 2-Column Container
        main_container = ttk.Frame(self, padding=(12, 4, 12, 10))
        main_container.pack(fill=tk.BOTH, expand=True)

        main_container.columnconfigure(0, weight=6)  # Left column (Controls & Setup)
        main_container.columnconfigure(1, weight=5)  # Right column (Live Logs & Report)
        main_container.rowconfigure(0, weight=1)

        left_col = ttk.Frame(main_container)
        left_col.grid(row=0, column=0, sticky="nsew", padx=(0, 6))

        right_col = ttk.Frame(main_container)
        right_col.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        # =====================================================================
        # LEFT COLUMN: 1. Model & Spin Axis
        # =====================================================================
        conn_frame = ttk.LabelFrame(left_col, text="1. Model & Spin Axis", padding=10)
        conn_frame.pack(fill=tk.X, pady=(0, 6))

        c_row1 = ttk.Frame(conn_frame)
        c_row1.pack(fill=tk.X)
        self.btn_connect = ttk.Button(c_row1, text="Connect / Refresh Model", command=self.on_refresh_model)
        self.btn_connect.pack(side=tk.LEFT, padx=(0, 10))
        self.lbl_model_status = ttk.Label(c_row1, text="Not connected. Click to attach.", foreground="#718096")
        self.lbl_model_status.pack(side=tk.LEFT)

        self.lbl_com_status = ttk.Label(conn_frame, text="Mass & COM: Connect model first.", foreground="#2d3748")
        self.lbl_com_status.pack(anchor=tk.W, pady=(6, 4))

        c_row2 = ttk.Frame(conn_frame)
        c_row2.pack(fill=tk.X, pady=(4, 0))

        ttk.Label(c_row2, text="Spin Axis:", font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(0, 6))

        self.axis_options = [
            "Z-Axis through Origin (0,0,0)  [Default]",
            "Y-Axis through Origin (0,0,0)",
            "X-Axis through Origin (0,0,0)",
            "Custom Coordinates (Point & Vector)...",
        ]
        self.var_axis_choice = tk.StringVar(value=self.axis_options[0])
        self.combo_axis = ttk.Combobox(
            c_row2,
            textvariable=self.var_axis_choice,
            values=self.axis_options,
            state="readonly",
            width=36
        )
        self.combo_axis.pack(side=tk.LEFT, padx=(0, 8))
        self.combo_axis.bind("<<ComboboxSelected>>", self.on_axis_combobox_selected)

        self.lbl_axis_status = ttk.Label(
            c_row2,
            text=f"Active: {self.spin_axis.label}",
            foreground="#22543d",
            font=("Segoe UI", 9, "bold")
        )
        self.lbl_axis_status.pack(side=tk.LEFT)

        # =====================================================================
        # LEFT COLUMN: 2. Balancing Dimensions
        # =====================================================================
        dims_frame = ttk.LabelFrame(left_col, text="2. Balancing Dimensions", padding=10)
        dims_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 6))

        # Quick-add bar with Quick Templates
        add_bar = ttk.Frame(dims_frame)
        add_bar.pack(fill=tk.X, pady=(0, 6))

        ttk.Label(add_bar, text="Name:", font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(0, 4))
        self.entry_custom_dim = ttk.Entry(add_bar, width=18)
        self.entry_custom_dim.pack(side=tk.LEFT, padx=(0, 4))
        self.entry_custom_dim.bind("<Return>", lambda e: self.on_add_custom_dim())

        self.btn_add_dim = ttk.Button(add_bar, text="+ Add", command=self.on_add_custom_dim)
        self.btn_add_dim.pack(side=tk.LEFT, padx=(0, 4))

        self.btn_pick_cad_dim = ttk.Button(add_bar, text="Pick CAD", command=self.on_pick_cad_dim)
        self.btn_pick_cad_dim.pack(side=tk.LEFT, padx=(0, 4))

        self.btn_auto_discover = ttk.Button(add_bar, text="🔍 Auto-Scan", command=self.on_auto_discover_dimensions)
        self.btn_auto_discover.pack(side=tk.LEFT, padx=(0, 6))

        # Common dimension templates from before
        self.dim_templates = [
            "Quick Templates...",
            "OuterRadius@Sketch1",
            "ToothAngle@Sketch1",
            "ToothAngle",
            "ToothWidth@Sketch1",
            "ToothDepth@Sketch1",
            "D1@Boss-Extrude1",
            "D1@Fillet1",
            "Bore@Sketch1",
            "InnerRadius@Sketch1",
            "D1@Sketch1",
            "D2@Sketch1"
        ]
        self.var_template = tk.StringVar(value=self.dim_templates[0])
        self.combo_template = ttk.Combobox(
            add_bar,
            textvariable=self.var_template,
            values=self.dim_templates,
            state="readonly",
            width=18
        )
        self.combo_template.pack(side=tk.LEFT)
        self.combo_template.bind("<<ComboboxSelected>>", self.on_template_selected)

        # Dimension List Box with Scrollbar
        list_container = ttk.Frame(dims_frame)
        list_container.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(list_container, bg="#ffffff", highlightthickness=1, highlightbackground="#cbd5e0")
        self.scrollbar = ttk.Scrollbar(list_container, orient=tk.VERTICAL, command=self.canvas.yview)
        self.dim_list_frame = ttk.Frame(self.canvas, style="White.TFrame")

        self.dim_list_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        )
        self.canvas_window = self.canvas.create_window((0, 0), window=self.dim_list_frame, anchor="nw")
        self.canvas.configure(xscrollcommand=None, yscrollcommand=self.scrollbar.set)
        self.canvas.bind("<Configure>", lambda e: self.canvas.itemconfig(self.canvas_window, width=e.width))

        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.dim_vars = []  # list of (DimensionItem, IntVar)

        # Dimension Toolbar (Re-Analyse, Select Usable, Clear, Bounds)
        d_toolbar = ttk.Frame(dims_frame)
        d_toolbar.pack(fill=tk.X, pady=(6, 4))

        self.btn_analyse = ttk.Button(d_toolbar, text="Re-Analyse Sensitivity", command=self.on_analyse_dimensions)
        self.btn_analyse.pack(side=tk.LEFT, padx=(0, 6))

        self.btn_select_all = ttk.Button(d_toolbar, text="Select Usable", command=self.on_select_all_usable)
        self.btn_select_all.pack(side=tk.LEFT, padx=(0, 6))

        self.btn_clear_ticks = ttk.Button(d_toolbar, text="Uncheck All", command=self.on_clear_all_ticks)
        self.btn_clear_ticks.pack(side=tk.LEFT, padx=(0, 6))

        self.btn_clear_list = ttk.Button(d_toolbar, text="Clear List", command=self.on_clear_dimensions_list)
        self.btn_clear_list.pack(side=tk.LEFT, padx=(0, 8))

        ttk.Label(d_toolbar, text="Max ±%:").pack(side=tk.LEFT, padx=(6, 2))
        self.var_pct = tk.IntVar(value=45)
        self.spin_pct = ttk.Spinbox(d_toolbar, from_=1, to=95, increment=5, textvariable=self.var_pct, width=4)
        self.spin_pct.pack(side=tk.LEFT)
        self.var_pct.trace_add("write", lambda *args: self.update_controllability_check())

        # Controllability verdict banner
        self.lbl_verdict = ttk.Label(
            dims_frame,
            text="Add dimensions above to check controllability.",
            wraplength=480,
            justify=tk.LEFT,
            font=("Segoe UI", 9, "bold"),
            foreground="#2b6cb0"
        )
        self.lbl_verdict.pack(fill=tk.X, pady=(4, 0))

        # =====================================================================
        # LEFT COLUMN: 3. Optimization Actions
        # =====================================================================
        action_frame = ttk.LabelFrame(left_col, text="3. Optimization Actions", padding=10)
        action_frame.pack(fill=tk.X)

        btn_row = ttk.Frame(action_frame)
        btn_row.pack(fill=tk.X, pady=(0, 6))

        self.btn_balance = ttk.Button(btn_row, text="⚡ Balance Weapon", style="Primary.TButton", command=self.on_balance)
        self.btn_balance.pack(side=tk.LEFT, padx=(0, 8))

        self.btn_undo = ttk.Button(btn_row, text="Undo Last Balance", style="Action.TButton", state=tk.DISABLED, command=self.on_undo)
        self.btn_undo.pack(side=tk.LEFT, padx=(0, 8))

        self.btn_cancel = ttk.Button(btn_row, text="Cancel", state=tk.DISABLED, command=self.on_cancel)
        self.btn_cancel.pack(side=tk.LEFT)

        self.progress_bar = ttk.Progressbar(action_frame, mode="determinate")
        self.progress_bar.pack(fill=tk.X)

        # =====================================================================
        # RIGHT COLUMN: Solve Report & Real-Time Logs
        # =====================================================================
        log_frame = ttk.LabelFrame(right_col, text="Solve Report & Real-Time Logs", padding=8)
        log_frame.pack(fill=tk.BOTH, expand=True)

        self.txt_log = scrolledtext.ScrolledText(log_frame, font=("Consolas", 9), bg="#1a202c", fg="#e2e8f0")
        self.txt_log.pack(fill=tk.BOTH, expand=True)

        log_toolbar = ttk.Frame(right_col)
        log_toolbar.pack(fill=tk.X, pady=(6, 0))

        self.btn_clear_log = ttk.Button(log_toolbar, text="Clear Log", command=self.on_clear_log)
        self.btn_clear_log.pack(side=tk.LEFT, padx=(0, 6))

        self.btn_copy_log = ttk.Button(log_toolbar, text="Copy Log", command=self.on_copy_log)
        self.btn_copy_log.pack(side=tk.LEFT)

        # Initial dimension empty state
        self._render_dimension_list()

    def log(self, text):
        self.txt_log.insert(tk.END, text + "\n")
        self.txt_log.see(tk.END)

    def on_clear_log(self):
        self.txt_log.delete("1.0", tk.END)

    def on_copy_log(self):
        self.clipboard_clear()
        self.clipboard_append(self.txt_log.get("1.0", tk.END))
        messagebox.showinfo("Copied", "Logs copied to clipboard.")

    def set_axis(self, axis):
        if not axis:
            return
        self.spin_axis = axis
        self.lbl_axis_status.config(text=f"Active: {axis.label}", foreground="#22543d")
        self.log(f"Spin axis set to: {axis.label}")
        if self.sw_conn.sw_model:
            try:
                mass_g, com = get_mass_properties(self.sw_conn.sw_model, self.solid_bodies)
                self.update_mass_and_com_labels(mass_g, com)
            except Exception:
                pass
        if self.analysis_cache:
            self.update_controllability_check()

    def on_template_selected(self, event=None):
        choice = self.var_template.get()
        if choice and choice != "Quick Templates...":
            self.entry_custom_dim.delete(0, tk.END)
            self.entry_custom_dim.insert(0, choice)
            self.var_template.set("Quick Templates...")

    def on_axis_combobox_selected(self, event=None):
        choice = self.var_axis_choice.get()
        if "Z-Axis" in choice:
            self.set_axis(SpinAxis((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), "Z-Axis through Origin (0,0,0)"))
        elif "Y-Axis" in choice:
            self.set_axis(SpinAxis((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), "Y-Axis through Origin (0,0,0)"))
        elif "X-Axis" in choice:
            self.set_axis(SpinAxis((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), "X-Axis through Origin (0,0,0)"))
        elif "Custom" in choice:
            dlg = CustomAxisDialog(self, self.spin_axis)
            if dlg.result:
                self.set_axis(dlg.result)
                self.var_axis_choice.set(dlg.result.label)
            else:
                if self.spin_axis:
                    self.var_axis_choice.set(self.spin_axis.label)

    def _save_active_dimensions(self):
        title = safe_call(self.sw_conn.sw_model, "GetTitle") if self.sw_conn.sw_model else None
        if title:
            dim_names = [d.full_name or d.name for d in self.dimensions]
            save_dim_names(title, dim_names)

    def _load_saved_dimensions_for_model(self, sw_model):
        title = safe_call(sw_model, "GetTitle")
        if not title:
            return
        saved = load_saved_dim_names(title)
        for name in saved:
            if not any(d.name == name or d.full_name == name for d in self.dimensions):
                dim_item = find_dimension_by_name(sw_model, name)
                if dim_item:
                    self.dimensions.append(dim_item)

    def on_auto_discover_dimensions(self, silent=False):
        """Auto-discover all driving sketch dimensions from active SolidWorks model."""
        if not self.sw_conn.sw_model:
            if not silent:
                messagebox.showinfo("Connect First", "Please click 'Connect / Refresh Model' first.")
            return

        sw_model = self.sw_conn.sw_model
        try:
            discovered = get_sketch_dimensions(sw_model)
            if not discovered:
                # Fallback to saved JSON cache if auto-enumeration found nothing
                self._load_saved_dimensions_for_model(sw_model)
                if not silent:
                    if self.dimensions:
                        messagebox.showinfo("Loaded Saved", f"Loaded {len(self.dimensions)} previously saved dimensions.")
                    else:
                        messagebox.showwarning(
                            "No Dimensions Found",
                            "Could not automatically detect sketch dimensions.\n"
                            "Please use '+ Add' to specify dimension names (e.g. OuterRadius@Sketch1) or 'Pick CAD'."
                        )
                return

            added_cnt = 0
            for d in discovered:
                if not any(existing.name == d.name or existing.full_name == d.full_name for existing in self.dimensions):
                    self.dimensions.append(d)
                    added_cnt += 1

            self._save_active_dimensions()
            self._render_dimension_list()
            self.log(f"Auto-discovered {len(discovered)} sketch dimensions ({added_cnt} new added):")
            for d in discovered:
                self.log(f"  • {d.name} [{d.sketch_name}] = {d.to_display_value():.3f} {d.display_unit}")
            if not silent:
                messagebox.showinfo("Auto-Discovery Complete", f"Successfully discovered {len(discovered)} sketch dimensions.")
        except Exception as e:
            if not silent:
                messagebox.showerror("Discovery Error", f"Error scanning sketch dimensions: {e}")
            self.log(f"Dimension discovery error: {e}")

    def on_refresh_model(self):
        """Connect to active SolidWorks part and auto-discover sketch dimensions."""
        try:
            sw_model = self.sw_conn.refresh_model()
            title = safe_call(sw_model, "GetTitle") or "SolidWorks Document"
            self.lbl_model_status.config(text=f"Connected: {title}", foreground="#22543d")
            self.log(f"\nConnected to SolidWorks document: {title}")

            # Fetch bodies & compute initial mass
            self.solid_bodies = get_all_solid_bodies(sw_model)
            mass_g, com = get_mass_properties(sw_model, self.solid_bodies)
            body_count_str = f"{len(self.solid_bodies)} bodies" if self.solid_bodies else "model bodies"
            self.log(f"Model mass: {mass_g:.2f} g ({body_count_str})")
            self.update_mass_and_com_labels(mass_g, com)

            # Auto-discover driving sketch dimensions if dimension list is currently empty
            if not self.dimensions:
                self.on_auto_discover_dimensions(silent=True)

            # Re-read live values of all active dimensions
            for d in self.dimensions:
                d.get_current_value()

            self._render_dimension_list()

            if self.dimensions:
                self.log(f"Loaded {len(self.dimensions)} active dimensions:")
                for d in self.dimensions:
                    self.log(f"  • {d.name} = {d.to_display_value():.3f} {d.display_unit}")
            else:
                self.log("Ready. Add dimensions to balance using the field above or 'Pick From CAD Selection'.")

        except Exception as e:
            import traceback
            self.log(f"SolidWorks connection error:\n{traceback.format_exc()}")
            messagebox.showerror("SolidWorks Error", str(e))
            self.lbl_model_status.config(text="Error connecting to SolidWorks.", foreground="#c53030")

    def on_add_custom_dim(self):
        if not self.sw_conn.sw_model:
            messagebox.showinfo("Connect First", "Please click 'Connect / Refresh Model' first.")
            return

        query = self.entry_custom_dim.get().strip()
        if not query:
            return

        # Check if already in list
        if any(d.name == query or d.full_name == query for d in self.dimensions):
            messagebox.showinfo("Already Added", f"Dimension '{query}' is already in the list.")
            return

        dim_item = find_dimension_by_name(self.sw_conn.sw_model, query)
        if dim_item:
            self.dimensions.append(dim_item)
            self._save_active_dimensions()
            self._render_dimension_list()
            self.log(f"Added dimension: {dim_item.name} = {dim_item.to_display_value():.3f} {dim_item.display_unit}")
            self.entry_custom_dim.delete(0, tk.END)
        else:
            messagebox.showwarning(
                "Dimension Not Found",
                f"Could not find dimension '{query}' in SolidWorks.\n\n"
                f"Tips:\n"
                f"• Specify the full name, e.g.: OuterRadius@Sketch1 or D1@Boss-Extrude1\n"
                f"• Or just the parameter name: OuterRadius or ToothAngle\n"
                f"• Ensure the sketch or feature name matches your Feature Tree."
            )

    def on_pick_cad_dim(self):
        if not self.sw_conn.sw_model:
            messagebox.showinfo("Connect First", "Please click 'Connect / Refresh Model' first.")
            return

        dims = get_dimensions_from_selection(self.sw_conn.sw_model)
        if not dims:
            messagebox.showinfo(
                "No Dimension Selected",
                "Please click on a dimension in SolidWorks first (so it highlights blue), then click 'Pick From CAD Selection'."
            )
            return

        added_cnt = 0
        for dim_item in dims:
            if not any(d.name == dim_item.name or d.full_name == dim_item.full_name for d in self.dimensions):
                self.dimensions.append(dim_item)
                added_cnt += 1
                self.log(f"Added dimension from CAD: {dim_item.name} = {dim_item.to_display_value():.3f} {dim_item.display_unit}")

        if added_cnt > 0:
            self._save_active_dimensions()
            self._render_dimension_list()

    def on_remove_dimension(self, idx):
        if 0 <= idx < len(self.dimensions):
            removed = self.dimensions.pop(idx)
            self.log(f"Removed dimension: {removed.name}")
            self._save_active_dimensions()
            self._render_dimension_list()
            if self.analysis_cache:
                self.analysis_cache = None
            self.update_controllability_check()

    def on_clear_dimensions_list(self):
        if not self.dimensions:
            return
        if messagebox.askyesno("Clear Dimensions", "Remove all dimensions from the list?"):
            self.dimensions.clear()
            self._save_active_dimensions()
            self._render_dimension_list()
            self.analysis_cache = None
            self.lbl_verdict.config(text="Add dimensions to adjust.", foreground="#4a5568")

    def update_mass_and_com_labels(self, mass_g, com):
        text = f"Mass: {mass_g:.2f} g ({len(self.solid_bodies)} solid bodies)"
        if self.spin_axis:
            ab = self.spin_axis.offset_mm(com)
            r = math.hypot(*ab)
            imbalance = mass_g * r
            text += (
                f"  |  COM Offset: {self.spin_axis.describe(ab)} "
                f"(total {r:.4f} mm, Imbalance: {imbalance:.1f} g·mm)"
            )
        self.lbl_com_status.config(text=text)

    def _render_dimension_list(self, rates=None):
        for widget in self.dim_list_frame.winfo_children():
            widget.destroy()

        self.dim_vars = []

        if not self.dimensions:
            lbl_empty = ttk.Label(
                self.dim_list_frame,
                text="No dimensions added yet.\n\n"
                     "• Enter dimension name above (e.g. OuterRadius@Sketch1) and click '+ Add'\n"
                     "• Or click a dimension in SolidWorks and click 'Pick From CAD Selection'",
                justify=tk.CENTER,
                foreground="#718096",
                font=("Segoe UI", 9, "italic")
            )
            lbl_empty.pack(pady=28, fill=tk.BOTH, expand=True)
            self.lbl_verdict.config(text="Add at least one dimension to balance.", foreground="#718096")
            return

        for i, dim in enumerate(self.dimensions):
            val_display = f"{dim.to_display_value(dim.get_current_value()):.3f} {dim.display_unit}"
            label_text = f"{dim.name}  [{dim.sketch_name}]  =  {val_display}"

            if rates and i < len(rates):
                rt = rates[i]
                if rt['kind'] == 'dead':
                    label_text += "  —  [breaks model]"
                elif rt['kind'] == 'none':
                    label_text += "  —  [no effect]"
                else:
                    label_text += f"  —  {rt['effect']:.3f} mm/%"
                    if rt['share'] >= 0.20:
                        label_text += f", alone fixes {rt['share']*100:.0f}% at {rt['needed']:+.1f}%"
                        if abs(rt['needed']) > self.var_pct.get():
                            label_text += " (over limit)"

            row_frame = ttk.Frame(self.dim_list_frame, style="White.TFrame")
            row_frame.pack(fill=tk.X, padx=4, pady=2)

            var = tk.IntVar(value=1)
            cb = tk.Checkbutton(
                row_frame,
                text=label_text,
                variable=var,
                anchor="w",
                bg="#ffffff",
                activebackground="#e2e8f0",
                font=("Consolas" if rates else "Segoe UI", 9),
                command=self.update_controllability_check
            )
            cb.pack(side=tk.LEFT, fill=tk.X, expand=True)

            btn_del = tk.Button(
                row_frame,
                text="✕",
                fg="#e53e3e",
                bg="#ffffff",
                activeforeground="#c53030",
                activebackground="#fed7d7",
                relief=tk.FLAT,
                bd=0,
                cursor="hand2",
                font=("Segoe UI", 9, "bold"),
                command=lambda idx=i: self.on_remove_dimension(idx)
            )
            btn_del.pack(side=tk.RIGHT, padx=4)

            self.dim_vars.append((dim, var))

    def on_analyse_dimensions(self):
        if not self.sw_conn.sw_model or not self.spin_axis:
            messagebox.showinfo("Selection Required", "Please ensure SolidWorks is connected.")
            return

        pct = self.var_pct.get()
        params = self.dimensions
        if not params:
            messagebox.showinfo("No Dimensions", "Please add at least one dimension to analyze.")
            return

        self.log("\n--- Starting Dimension Sensitivity Analysis ---")
        self.progress_bar.config(maximum=len(params) + 1, value=0)
        self.btn_cancel.config(state=tk.NORMAL)
        self.cancel_requested = False
        self.update()

        # Baseline values
        x0 = [d.get_current_value() for d in params]
        scale = [abs(v) if abs(v) > 1e-12 else 0.001 for v in x0]
        lo = [v - v * (pct / 100.0) for v in x0]
        hi = [v + v * (pct / 100.0) for v in x0]

        evaluator = make_evaluator(self.sw_conn.sw_model, params, self.solid_bodies, self.spin_axis)

        def do_analysis():
            try:
                with SuspendGraphicsContext(self.sw_conn.sw_model):
                    r0 = evaluator(x0)
                    self.progress_bar.step(1)
                    self.update()

                    def counted_eval(x):
                        if self.cancel_requested:
                            raise Cancelled()
                        res = evaluator(x)
                        self.progress_bar.step(1)
                        self.update()
                        return res

                    jac, dead = probe_sensitivity(counted_eval, x0, scale, lo, hi, r0)
                    # Restore baseline values
                    for d, val in zip(params, x0):
                        d.set_value(val)
                    rebuild_model(self.sw_conn.sw_model)

                per_pct = [(jac[0][i] / 100.0, jac[1][i] / 100.0) for i in range(len(params))]
                self.analysis_cache = {
                    'x0': x0,
                    'r0': r0,
                    'jac': jac,
                    'dead': set(dead),
                    'per_pct': per_pct
                }
                rates = rate_dimensions(per_pct, r0, pct, dead)
                self.btn_cancel.config(state=tk.DISABLED)
                self._post_analysis_update(rates)
            except Cancelled:
                for d, val in zip(params, x0):
                    d.set_value(val)
                rebuild_model(self.sw_conn.sw_model)
                self.btn_cancel.config(state=tk.DISABLED)
                self.progress_bar.config(value=0)
                self.log("Sensitivity analysis cancelled.")
            except Exception as exc:
                err_text = str(exc)
                for d, val in zip(params, x0):
                    d.set_value(val)
                rebuild_model(self.sw_conn.sw_model)
                self.btn_cancel.config(state=tk.DISABLED)
                self._on_analysis_error(err_text)

        self.after(20, do_analysis)

    def _on_analysis_error(self, err_text):
        self.progress_bar.config(value=0)
        self.log(f"\nSensitivity Analysis error: {err_text}")
        messagebox.showerror("Sensitivity Analysis Error", f"Sensitivity analysis encountered an error:\n\n{err_text}")

    def _post_analysis_update(self, rates):
        self.progress_bar.config(value=0)
        self._render_dimension_list(rates)
        usable = [i for i, r in enumerate(rates) if r['kind'] == 'ok']
        self.log(f"Analysis complete: {len(usable)} of {len(rates)} dimensions move the Centre of Mass.")
        self.update_controllability_check()

    def update_controllability_check(self):
        if not self.analysis_cache or not self.spin_axis:
            return

        pct = self.var_pct.get()
        ticked_indices = [i for i, (_, var) in enumerate(self.dim_vars) if var.get() == 1]
        if not ticked_indices:
            self.lbl_verdict.config(text="Tick dimensions to adjust.", foreground="#4a5568")
            return

        jac = [[self.analysis_cache['jac'][p][i] for i in ticked_indices] for p in range(2)]
        dead = [k for k, i in enumerate(ticked_indices) if i in self.analysis_cache['dead']]
        c = controllability(jac, self.analysis_cache['r0'], pct, dead)
        c['dead'] = dead

        names = [self.dimensions[i].name for i in ticked_indices]
        lines = control_lines(c, names, self.spin_axis.names, pct, show_effects=False)

        verdict_text = " | ".join(lines)
        if c['verdict'] == 'both':
            color = "#22543d"  # green
        elif c['verdict'] == 'weak':
            color = "#d69e2e"  # yellow/amber
        else:
            color = "#c53030"  # red

        self.lbl_verdict.config(text=verdict_text, foreground=color)

    def on_select_all_usable(self):
        if not self.analysis_cache:
            return
        pct = self.var_pct.get()
        rates = rate_dimensions(self.analysis_cache['per_pct'], self.analysis_cache['r0'], pct, self.analysis_cache['dead'])
        for i, (_, var) in enumerate(self.dim_vars):
            if rates[i]['kind'] == 'ok' and rates[i]['share'] >= 0.15:
                var.set(1)
        self.update_controllability_check()

    def on_clear_all_ticks(self):
        for _, var in self.dim_vars:
            var.set(0)
        self.update_controllability_check()

    def on_cancel(self):
        self.cancel_requested = True
        self.log("Cancel requested...")

    def on_balance(self):
        if not self.sw_conn.sw_model or not self.spin_axis:
            messagebox.showinfo("Setup Incomplete", "Connect SolidWorks first.")
            return

        ticked = [(dim, i) for i, (dim, var) in enumerate(self.dim_vars) if var.get() == 1]
        if not ticked:
            messagebox.showinfo("No Selection", "Please tick at least one dimension to adjust.")
            return

        free_dims = [t[0] for t in ticked]
        free_indices = [t[1] for t in ticked]
        pct = self.var_pct.get()

        # Save Undo state
        self.undo_state = {d.name: (d, d.get_current_value()) for d in free_dims}
        self.btn_undo.config(state=tk.NORMAL)

        # Check controllability
        if self.analysis_cache:
            jac = [[self.analysis_cache['jac'][p][i] for i in free_indices] for p in range(2)]
            dead = [k for k, i in enumerate(free_indices) if i in self.analysis_cache['dead']]
            ctrl = controllability(jac, self.analysis_cache['r0'], pct, dead)
            if control_blocked(ctrl):
                if not messagebox.askyesno(
                    "Balance Incomplete Warning",
                    "The selected dimensions move the COM along only one line or have zero effect.\n"
                    "Balance as far as possible anyway?"
                ):
                    return

        # Prepare bounds & solver
        x0 = [d.get_current_value() for d in free_dims]
        scale = [abs(v) if abs(v) > 1e-12 else 0.001 for v in x0]
        lo = [v - v * (pct / 100.0) for v in x0]
        hi = [v + v * (pct / 100.0) for v in x0]

        evaluator = make_evaluator(self.sw_conn.sw_model, free_dims, self.solid_bodies, self.spin_axis)

        self.btn_balance.config(state=tk.DISABLED)
        self.btn_cancel.config(state=tk.NORMAL)
        self.cancel_requested = False
        self.progress_bar.config(maximum=50, value=0)

        self.log(f"\n================ Starting Solve ================")
        self.log(f"Optimizing {len(free_dims)} dimensions with ±{pct}% bounds...")
        self.update()

        def do_solve():
            def tick_fn(count):
                if self.cancel_requested:
                    raise Cancelled()
                self.progress_bar.config(value=min(count, 50))
                self.update()

            try:
                with SuspendGraphicsContext(self.sw_conn.sw_model):
                    if len(free_dims) == 1:
                        res = solve(
                            lambda v: evaluator([v]),
                            lo[0], hi[0], x0[0],
                            fmt=lambda v: f"{free_dims[0].to_display_value(v):.3f} {free_dims[0].display_unit}",
                            tick=tick_fn
                        )
                        sol_x = [res['root']]
                        evals = res['evals']
                        stop_reason = ""
                    else:
                        res = solve_multi(evaluator, x0, scale, lo, hi, tick=tick_fn)
                        sol_x = res['x']
                        evals = res['evals']
                        stop_reason = res['stop_reason']

                    # Lock in solution
                    evaluator(sol_x)

                self._on_solve_finished(free_dims, x0, sol_x, evals, stop_reason)

            except Cancelled:
                self._on_solve_cancelled(free_dims, x0)
            except Exception as exc:
                self._on_solve_error(free_dims, x0, str(exc))

        self.after(50, do_solve)

    def _on_solve_finished(self, free_dims, x0, sol_x, evals, stop_reason):
        self.btn_balance.config(state=tk.NORMAL)
        self.btn_cancel.config(state=tk.DISABLED)
        self.progress_bar.config(value=0)

        # Final measurement
        mass_g, com = get_mass_properties(self.sw_conn.sw_model, self.solid_bodies)
        ab = self.spin_axis.offset_mm(com)
        r = math.hypot(*ab)
        balanced = max(abs(ab[0]), abs(ab[1])) <= COM_TOL_MM and r <= COM_TOL_MM

        status_msg = "SUCCESS: Weapon is BALANCED!" if balanced else "FINISHED: Improved balance."
        self.log(f"\n{status_msg}")
        self.log(f"Remaining COM Offset: {self.spin_axis.describe(ab, 5)} (total {r:.5f} mm)")
        self.log(f"Remaining Imbalance: {mass_g * r:.3f} g·mm  |  New Mass: {mass_g:.2f} g")
        self.log(f"Total Rebuilds: {evals}")
        if stop_reason:
            self.log(f"Note: {stop_reason}")

        self.log("\nChanged Dimensions:")
        for dim, v_old, v_new in zip(free_dims, x0, sol_x):
            old_disp = dim.to_display_value(v_old)
            new_disp = dim.to_display_value(v_new)
            unit = dim.display_unit
            pct_chg = (v_new - v_old) / v_old * 100.0 if v_old != 0 else 0
            self.log(f"  {dim.name}: {old_disp:.3f} {unit} -> {new_disp:.3f} {unit} ({pct_chg:+.2f}%)")

        self.update_mass_and_com_labels(mass_g, com)
        self._render_dimension_list()
        messagebox.showinfo(
            "Balancing Complete",
            f"{status_msg}\n\nRemaining offset: {r:.4f} mm\nImbalance: {mass_g * r:.2f} g·mm\nRebuilds: {evals}"
        )

    def _on_solve_cancelled(self, free_dims, x0):
        self.btn_balance.config(state=tk.NORMAL)
        self.btn_cancel.config(state=tk.DISABLED)
        self.progress_bar.config(value=0)
        for dim, v0 in zip(free_dims, x0):
            dim.set_value(v0)
        rebuild_model(self.sw_conn.sw_model)
        self._render_dimension_list()
        self.log("\nSolve was cancelled by user. Dimensions restored.")
        messagebox.showinfo("Cancelled", "Solve was cancelled. Dimensions restored to baseline values.")

    def _on_solve_error(self, free_dims, x0, err_text):
        self.btn_balance.config(state=tk.NORMAL)
        self.btn_cancel.config(state=tk.DISABLED)
        self.progress_bar.config(value=0)
        for dim, v0 in zip(free_dims, x0):
            dim.set_value(v0)
        rebuild_model(self.sw_conn.sw_model)
        self._render_dimension_list()
        self.log(f"\nSolve Error: {err_text}")
        messagebox.showerror("Solve Failed", f"Optimization failed:\n{err_text}\n\nDimensions were restored.")

    def on_undo(self):
        if not self.undo_state or not self.sw_conn.sw_model:
            return

        for name, (dim, val) in self.undo_state.items():
            dim.set_value(val)

        rebuild_model(self.sw_conn.sw_model)
        self.undo_state = None
        self.btn_undo.config(state=tk.DISABLED)
        self.log("\nUndo applied: Original dimension values restored.")

        mass_g, com = get_mass_properties(self.sw_conn.sw_model, self.solid_bodies)
        self.update_mass_and_com_labels(mass_g, com)
        self._render_dimension_list()
        messagebox.showinfo("Undo Successful", "All adjusted dimensions have been restored to their previous values.")


def main():
    app = BalancerApp()
    app.mainloop()
