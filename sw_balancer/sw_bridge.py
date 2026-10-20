"""
SolidWorks COM Bridge.
Automates SolidWorks 2020+ via Windows COM (win32com.client).
Handles:
- Connecting to SolidWorks instance
- Document inspection (Parts & Assemblies)
- Mass properties and Center of Mass extraction (meters, kg -> mm, grams)
- Dimension enumeration, inspection (driving vs driven), and modification
- Model rebuild and feature error checking
- Spin axis extraction from cylindrical faces, circular edges, and reference axes
"""

import math
import pythoncom
import win32com.client
from .geometry import SpinAxis, _unit, _sub, M_TO_MM
from .solver import RebuildFailed

# SolidWorks Selection & Document Type Enums
swDocPART = 1
swDocASSEMBLY = 2
swDocDRAWING = 3

swSelFACES = 2
swSelEDGES = 1
swSelAXES = 12
swSelDATUMAXES = 48
swSelSOLIDBODIES = 78
swSelSKETCHSEGS = 10

swDimensionDriving = 2
swDimensionDriven = 1
swThisConfiguration = 1


def safe_getattr(obj, attr_name, default=None):
    """Safely get an attribute on a COM or Python object.

    Catches both AttributeError and pywintypes.com_error (e.g. DISP_E_MEMBERNOTFOUND).
    """
    if obj is None:
        return default
    try:
        return getattr(obj, attr_name)
    except Exception:
        return default


def com_prop_or_call(obj, attr_name, *args, **kwargs):
    """Property-first COM accessor falling back to method invocation.

    In win32com late-binding (IDispatch), 0-argument COM members (FirstFeature,
    GetNextFeature, GetFirstSubFeature, GetNextSubFeature, GetFirstDisplayDimension,
    SystemValue, FullName, etc.) MUST be accessed as properties without parentheses ();
    calling them as methods invokes DISPID_VALUE, which raises DISP_E_MEMBERNOTFOUND (-2147352573).
    Members with arguments (e.g. GetDimension2(0), GetNextDisplayDimension(dd))
    are called directly with arguments.
    """
    if obj is None:
        return None

    # If arguments are passed, it must be a method invocation
    if args or kwargs:
        try:
            fn = getattr(obj, attr_name)
            if callable(fn):
                return fn(*args, **kwargs)
        except Exception:
            pass
        return None

    # Zero arguments: Property-first access
    try:
        val = getattr(obj, attr_name)
        # In win32com.client, CDispatch instances have __call__ (callable is True),
        # but they are COM objects (properties), NOT methods. Calling them raises DISP_E_MEMBERNOTFOUND!
        if type(val).__name__ == "CDispatch" or not callable(val):
            return val
        # If win32com returned a callable wrapper (e.g. gencache bound method), try calling it
        try:
            return val()
        except Exception:
            return val
    except Exception:
        # Fallback to method call if property access failed
        try:
            fn = getattr(obj, attr_name)
            if callable(fn):
                return fn()
        except Exception:
            pass
    return None


def safe_call(obj, attr_name, *args, **kwargs):
    """Safely get a property or invoke a method on a COM object.

    Handles early-bound attributes, late-bound dispatch methods, and catches
    pywintypes.com_error (DISP_E_MEMBERNOTFOUND, etc.) so missing members return None
    cleanly instead of crashing.
    """
    return com_prop_or_call(obj, attr_name, *args, **kwargs)


def as_dynamic(obj):
    """Ensure COM object is wrapped with dynamic dispatch so all interface methods are accessible."""
    if obj is None:
        return None
    try:
        import win32com.client.dynamic
        return win32com.client.dynamic.Dispatch(obj)
    except Exception:
        return obj


def get_com_array(obj, attr_name, *args):
    """Retrieve an array/tuple from a COM object whether it is a property or method.

    In late-binding (dynamic dispatch), win32com exposes COM properties returning arrays
    as callable _Method objects that MUST be invoked with () to return the tuple.
    In early-binding, win32com exposes them as direct tuples.
    This helper guarantees we get the underlying tuple or list of numbers in both cases.
    """
    if obj is None:
        return None

    val = None
    try:
        val = getattr(obj, attr_name)
    except Exception:
        pass

    # If it is callable (e.g. late-bound _Method or function), invoke it!
    if callable(val):
        try:
            val = val(*args)
        except Exception:
            pass

    # If still not a tuple/list, try with 'Get' prefix (e.g. GetCylinderParams)
    if not isinstance(val, (tuple, list)) and not attr_name.startswith("Get"):
        try:
            get_fn = getattr(obj, "Get" + attr_name)
            if callable(get_fn):
                val = get_fn(*args)
        except Exception:
            pass

    if isinstance(val, (tuple, list)):
        return val
    return None


class SolidWorksConnection:
    """Manages the COM connection to SolidWorks."""

    def __init__(self):
        self.sw_app = None
        self.sw_model = None

    def connect(self):
        """Connect to active SolidWorks instance or launch one."""
        try:
            # Connect to existing running instance in ROT
            self.sw_app = win32com.client.GetActiveObject("SldWorks.Application")
        except Exception:
            try:
                # If not registered in ROT, dispatch
                self.sw_app = win32com.client.Dispatch("SldWorks.Application")
                self.sw_app.Visible = True
            except Exception as e:
                raise RuntimeError(
                    f"Could not connect to SolidWorks.\n"
                    f"Please make sure SolidWorks is running with a part open.\n\nDetails: {e}"
                )
        return self.sw_app

    def refresh_model(self):
        """Refresh reference to the active document."""
        if not self.sw_app:
            self.connect()

        self.sw_model = safe_getattr(self.sw_app, "ActiveDoc")
        if not self.sw_model:
            raise RuntimeError(
                "No active document found in SolidWorks.\n\n"
                "Please make sure SolidWorks is open and you have a Part (.SLDPRT) or Assembly (.SLDASM) open and visible."
            )

        doc_type = safe_call(self.sw_model, "GetType")
        if doc_type not in (swDocPART, swDocASSEMBLY):
            raise RuntimeError(f"The active document must be a Part or an Assembly (detected document type: {doc_type}).")

        return self.sw_model


class SuspendGraphicsContext:
    """Context manager for optimization passes."""

    def __init__(self, sw_model):
        self.sw_model = sw_model

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            safe_call(self.sw_model, "GraphicsRedraw2")
        except Exception:
            pass


def get_all_solid_bodies(sw_model):
    """Retrieve all solid bodies from the active part/assembly."""
    doc_type = safe_call(sw_model, "GetType")
    bodies = []

    if doc_type == swDocPART:
        part = sw_model
        # Try direct call (0 = swSolidBody, False = all bodies including hidden)
        b = safe_call(part, "GetBodies2", 0, False)

        # If still not found, traverse SolidBodyFolder in the feature tree
        if not b:
            feat = safe_call(sw_model, "FirstFeature")
            while feat is not None:
                if safe_call(feat, "GetTypeName2") == "SolidBodyFolder":
                    body_folder = safe_call(feat, "GetSpecificFeature2")
                    if body_folder:
                        b = safe_call(body_folder, "GetBodies")
                    break
                feat = safe_call(feat, "GetNextFeature")

        if b:
            bodies = list(b) if isinstance(b, (list, tuple)) else [b]

    elif doc_type == swDocASSEMBLY:
        cfg_mgr = safe_getattr(sw_model, "ConfigurationManager")
        if cfg_mgr:
            active_cfg = safe_getattr(cfg_mgr, "ActiveConfiguration")
            if active_cfg:
                root_comp = safe_call(active_cfg, "GetRootComponent3", True)
                if root_comp:
                    children = safe_call(root_comp, "GetChildren")
                    if children:
                        for comp in children:
                            b = safe_call(comp, "GetBodies3", 0)  # 0 = swSolidBody
                            if b:
                                bodies.extend(list(b) if isinstance(b, (list, tuple)) else [b])

    return bodies


def get_mass_properties(sw_model, bodies=None):
    """Compute mass (in grams) and center of mass (in meters).

    Uses multiple redundant SolidWorks APIs to guarantee reliability:
    1. ModelDocExtension.CreateMassProperty2 / CreateMassProperty
    2. IModelDoc2.GetMassProperties2(1) / GetMassProperties
    3. Individual body-level GetMassProperties sum
    """
    if not sw_model:
        raise RuntimeError("No active SolidWorks document provided.")

    errors = []

    # Strategy 1: Try ModelDocExtension.CreateMassProperty2 or CreateMassProperty
    ext = safe_getattr(sw_model, "Extension")
    if ext is not None:
        for method_name in ("CreateMassProperty2", "CreateMassProperty"):
            try:
                mass_prop = safe_call(ext, method_name)
                if mass_prop is not None:
                    safe_call(mass_prop, "put_UseSystemUnits", True)
                    try:
                        setattr(mass_prop, "UseSystemUnits", True)
                    except Exception:
                        pass

                    # Only assign SelectedItems if bodies is provided AND has > 1 body in a multi-body part.
                    # For a single-body part, leaving SelectedItems unset evaluates the entire model cleanly.
                    if bodies and len(bodies) > 1:
                        try:
                            mass_prop.SelectedItems = bodies
                        except Exception as e_sel:
                            errors.append(f"SelectedItems: {e_sel}")

                    safe_call(mass_prop, "Recalculate")

                    mass_kg = safe_getattr(mass_prop, "Mass")
                    if mass_kg is None:
                        mass_kg = safe_call(mass_prop, "GetMass")

                    com = safe_getattr(mass_prop, "CenterOfMass")
                    if com is None:
                        com = safe_call(mass_prop, "GetCenterOfMass")

                    if com is not None and mass_kg is not None and float(mass_kg) > 0.0:
                        return float(mass_kg) * 1000.0, (float(com[0]), float(com[1]), float(com[2]))
            except Exception as e1:
                errors.append(f"{method_name}: {e1}")

    # Strategy 2: Direct ModelDoc2.GetMassProperties2 / GetMassProperties
    # Returns 12-element array: [COM_X, COM_Y, COM_Z, Volume, Area, Mass, ...]
    for method_name in ("GetMassProperties2", "GetMassProperties"):
        try:
            props = safe_call(sw_model, method_name, 1)
            if props is None and method_name == "GetMassProperties":
                props = safe_call(sw_model, method_name)

            if props is not None and len(props) >= 6:
                com_x, com_y, com_z = float(props[0]), float(props[1]), float(props[2])
                mass_kg = float(props[5])
                if mass_kg > 0.0:
                    return mass_kg * 1000.0, (com_x, com_y, com_z)
        except Exception as e2:
            errors.append(f"{method_name}: {e2}")

    # Strategy 3: Individual body GetMassProperties sum (if bodies were provided)
    if bodies and len(bodies) > 0:
        total_mass = 0.0
        acc = [0.0, 0.0, 0.0]
        for b in bodies:
            try:
                p = safe_call(b, "GetMassProperties", 1)
                if p and len(p) >= 6:
                    m = float(p[5])
                    if m > 0.0:
                        total_mass += m
                        acc[0] += m * float(p[0])
                        acc[1] += m * float(p[1])
                        acc[2] += m * float(p[2])
            except Exception as e3:
                errors.append(f"body.GetMassProperties: {e3}")
        if total_mass > 0.0:
            return total_mass * 1000.0, (acc[0] / total_mass, acc[1] / total_mass, acc[2] / total_mass)

    err_details = f"\nDetails: {'; '.join(errors)}" if errors else ""
    raise RuntimeError(
        f"Could not calculate mass properties from SolidWorks.{err_details}\n"
        "Please ensure your CAD part has 3D geometry with material/density assigned."
    )


def _get_ref_axis_params(ref_axis_or_feat):
    """Retrieve the 6-element start/end array from a reference axis feature or specific feature."""
    if ref_axis_or_feat is None:
        return None

    # Try direct methods on the object
    for name in ("GetRefAxisParams", "RefAxisParams"):
        p = get_com_array(ref_axis_or_feat, name)
        if p and len(p) >= 6:
            return p

    # Try GetSpecificFeature2 / GetSpecificFeature if it is an IFeature
    specific = safe_call(ref_axis_or_feat, "GetSpecificFeature2") or safe_call(ref_axis_or_feat, "GetSpecificFeature")
    if specific:
        for name in ("GetRefAxisParams", "RefAxisParams"):
            p = get_com_array(specific, name)
            if p and len(p) >= 6:
                return p

    # Try GetDefinition() -> IRefAxisFeatureData
    defn = safe_call(ref_axis_or_feat, "GetDefinition")
    if defn:
        for name in ("GetRefAxisParams", "RefAxisParams"):
            p = get_com_array(defn, name)
            if p and len(p) >= 6:
                return p

    return None


def _extract_from_face(selected_obj):
    """Try extracting spin axis from a cylindrical, revolved, or conical face."""
    surf = safe_call(selected_obj, "GetSurface")
    if surf is None:
        surf = safe_getattr(selected_obj, "Surface")

    if surf is None:
        return None

    # 1. Cylinder
    p = get_com_array(surf, "CylinderParams")
    if p and len(p) >= 7:
        origin = (float(p[0]), float(p[1]), float(p[2]))
        direction = (float(p[3]), float(p[4]), float(p[5]))
        radius_mm = float(p[6]) * M_TO_MM
        return SpinAxis(origin, direction, f"Cylindrical Face Ø{radius_mm * 2:.3f} mm")

    # 2. Revolved Surface
    p_rev = get_com_array(surf, "RevolutionParams")
    if p_rev and len(p_rev) >= 6:
        origin = (float(p_rev[0]), float(p_rev[1]), float(p_rev[2]))
        direction = (float(p_rev[3]), float(p_rev[4]), float(p_rev[5]))
        return SpinAxis(origin, direction, "Revolved Surface Axis")

    # 3. Conical Surface
    p_cone = get_com_array(surf, "ConeParams")
    if p_cone and len(p_cone) >= 6:
        origin = (float(p_cone[0]), float(p_cone[1]), float(p_cone[2]))
        direction = (float(p_cone[3]), float(p_cone[4]), float(p_cone[5]))
        return SpinAxis(origin, direction, "Conical Surface Axis")

    return None


def _extract_from_edge(selected_obj):
    """Try extracting spin axis from a circular edge."""
    curve = safe_call(selected_obj, "GetCurve")
    if curve is None:
        curve = safe_getattr(selected_obj, "Curve")

    if curve is None:
        return None

    p = get_com_array(curve, "CircleParams")
    if p and len(p) >= 7:
        origin = (float(p[0]), float(p[1]), float(p[2]))
        direction = (float(p[3]), float(p[4]), float(p[5]))
        radius_mm = float(p[6]) * M_TO_MM
        return SpinAxis(origin, direction, f"Circular Edge Ø{radius_mm * 2:.3f} mm")

    return None


def _extract_from_sketch_arc(selected_obj):
    """Try extracting spin axis from a sketch circle or arc."""
    arc = safe_call(selected_obj, "GetSpecificSegment")
    if arc is None:
        arc = selected_obj

    is_circle = safe_call(arc, "IsCircle")
    if is_circle in (1, True):
        pt = get_com_array(arc, "GetCenterPoint2") or get_com_array(arc, "GetCenterPoint")
        norm = get_com_array(arc, "GetNormalVector")
        if pt and norm and len(pt) >= 3 and len(norm) >= 3:
            origin = (float(pt[0]), float(pt[1]), float(pt[2]))
            direction = (float(norm[0]), float(norm[1]), float(norm[2]))
            return SpinAxis(origin, direction, "Sketch Circle")
    return None


def extract_spin_axis(sw_model):
    """Extract spin axis from the current SolidWorks selection.

    Supports:
    - Cylindrical Face (bore hole)
    - Revolved Face
    - Conical Face
    - Circular Edge
    - Reference Axis Feature (selected on screen or in Feature Tree)
    - Temporary Axis
    - Sketch Circle / Arc
    """
    sel_mgr = safe_getattr(sw_model, "SelectionManager")
    if not sel_mgr:
        raise RuntimeError("Could not access SolidWorks SelectionManager.")

    count = safe_call(sel_mgr, "GetSelectedObjectCount2", -1)
    if count is None:
        count = safe_call(sel_mgr, "GetSelectedObjectCount")
    if not count:
        raise RuntimeError(
            "Nothing is selected in SolidWorks for the Spin Axis.\n\n"
            "Please click on a cylindrical bore face, circular edge, or reference axis in SolidWorks, then try again."
        )

    debug_items = []

    for i in range(1, count + 1):
        obj_type = safe_call(sel_mgr, "GetSelectedObjectType3", i, -1)
        selected_obj = safe_call(sel_mgr, "GetSelectedObject6", i, -1)
        if selected_obj is None:
            selected_obj = safe_call(sel_mgr, "GetSelectedObject2", i)

        if selected_obj is None:
            continue

        type_name = safe_call(selected_obj, "GetTypeName2") or ""
        debug_items.append(f"Selection {i}: type={obj_type}, type_name='{type_name}'")

        # 1. Reference Axis (swSelDATUMAXES = 5, swSelAXES = 12, swSelBODYFEATURES = 22, etc.)
        if obj_type in (5, 12, 22, 39, 48) or "RefAxis" in type_name or "Axis" in type_name:
            ref_axis = safe_call(selected_obj, "GetSpecificFeature2")
            if ref_axis is None:
                ref_axis = safe_call(selected_obj, "GetSpecificFeature")
            if ref_axis is None:
                ref_axis = selected_obj

            p_axis = _get_ref_axis_params(ref_axis)
            if p_axis is None and ref_axis != selected_obj:
                p_axis = _get_ref_axis_params(selected_obj)

            if p_axis is not None and len(p_axis) >= 6:
                start = (float(p_axis[0]), float(p_axis[1]), float(p_axis[2]))
                end = (float(p_axis[3]), float(p_axis[4]), float(p_axis[5]))
                direction = _unit(_sub(end, start))
                feat_name = safe_getattr(selected_obj, "Name") or "Axis"
                return SpinAxis(start, direction, f"Reference Axis '{feat_name}'")

        # 2. Face (Cylindrical, Revolved, Conical)
        face_axis = _extract_from_face(selected_obj)
        if face_axis is not None:
            return face_axis

        # 3. Edge (Circular)
        edge_axis = _extract_from_edge(selected_obj)
        if edge_axis is not None:
            return edge_axis

        # 4. Sketch Circle / Arc
        sketch_axis = _extract_from_sketch_arc(selected_obj)
        if sketch_axis is not None:
            return sketch_axis

    details = "\n".join(debug_items) if debug_items else "No recognizable objects."
    raise RuntimeError(
        f"Selected entity is not recognized as a cylindrical bore, circular edge, or reference axis.\n\n"
        f"SolidWorks selection details:\n{details}\n\n"
        f"Tip: Click directly on an inner cylindrical bore face (or a circular edge around the bore) in SolidWorks, then click 'Pick Spin Axis'."
    )


class DimensionItem:
    """Represents a driving sketch or feature dimension in SolidWorks."""

    def __init__(self, name, full_name, sketch_name, value_m, dim_obj, is_angular=False):
        self.name = name                 # e.g., "D1@Sketch1"
        self.full_name = full_name
        self.sketch_name = sketch_name   # e.g., "Sketch1"
        self.value_m = float(value_m)    # baseline value in meters (or radians if angular)
        self.dim_obj = dim_obj           # COM IDimension object
        self.is_angular = bool(is_angular)

    @property
    def display_unit(self):
        return "deg" if self.is_angular else "mm"

    def to_display_value(self, val_si=None):
        """Convert SI units (meters or radians) to display units (mm or degrees)."""
        if val_si is None:
            val_si = self.value_m
        if self.is_angular:
            return math.degrees(val_si)
        return val_si * M_TO_MM

    def from_display_value(self, val_disp):
        """Convert display units (mm or degrees) to SI units (meters or radians)."""
        if self.is_angular:
            return math.radians(val_disp)
        return val_disp / M_TO_MM

    def get_current_value(self):
        val = safe_getattr(self.dim_obj, "SystemValue")
        if val is not None:
            v = float(val[0] if isinstance(val, (tuple, list)) else val)
            self.value_m = v
            return v
        val = safe_call(self.dim_obj, "GetSystemValue2", "")
        if val is not None:
            v = float(val[0] if isinstance(val, (tuple, list)) else val)
            self.value_m = v
            return v
        val = safe_call(self.dim_obj, "GetSystemValue3", 1, "")
        if val is not None:
            v = float(val[0] if isinstance(val, (tuple, list)) else val)
            self.value_m = v
            return v
        return float(self.value_m)

    def set_value(self, new_val_m):
        val = float(new_val_m)
        self.value_m = val

        # 1. Try direct SystemValue property
        try:
            self.dim_obj.SystemValue = val
            return True
        except Exception:
            pass

        # 2. Try SetSystemValue3
        for cfg_opt in (1, 2):
            for cfg_name in ("", None):
                try:
                    res = safe_call(self.dim_obj, "SetSystemValue3", val, cfg_opt, cfg_name)
                    if res == 0 or res is True:
                        return True
                except Exception:
                    pass

        # 3. Try SetSystemValue2
        try:
            res = safe_call(self.dim_obj, "SetSystemValue2", val, "")
            if res == 0 or res is True:
                return True
        except Exception:
            pass

        return False


def _process_dimension(dim, feat, feat_name, dimensions, visited_dim_names):
    """Inspect and register a dimension if it is driving and readable."""
    if dim is None:
        return

    raw_name = com_prop_or_call(dim, "GetNameForSelection") or com_prop_or_call(dim, "FullName") or com_prop_or_call(dim, "Name")
    if not raw_name:
        raw_name = "Dimension"

    raw_str = str(raw_name)
    parts = raw_str.split("@")
    if len(parts) >= 2:
        # Canonicalize e.g. "D1@Fillet1@Document.Part" -> "D1@Fillet1"
        canonical = f"{parts[0]}@{parts[1]}"
    elif feat_name:
        canonical = f"{raw_str}@{feat_name}"
    else:
        canonical = raw_str

    # Avoid duplicate additions
    if canonical in visited_dim_names or raw_str in visited_dim_names:
        return

    driven_state = com_prop_or_call(dim, "DrivenState")
    if driven_state is None:
        driven_state = com_prop_or_call(dim, "GetDrivenState")

    # swDimensionDriven = 1. Skip driven/reference dimensions
    if driven_state == swDimensionDriven:
        return

    val = com_prop_or_call(dim, "SystemValue")
    if val is None:
        val = com_prop_or_call(dim, "GetSystemValue2", "")
    if val is None:
        val = com_prop_or_call(dim, "GetSystemValue3", swThisConfiguration, None)
    if val is None:
        val = com_prop_or_call(dim, "Value")

    if val is not None:
        val_m = val[0] if isinstance(val, (tuple, list)) else val
        try:
            val_m = float(val_m)
        except (ValueError, TypeError):
            return

        dim_type = com_prop_or_call(dim, "GetType")
        is_angular = (dim_type == 1) or any(k in str(canonical).lower() for k in ["angle", "deg", "sweep"])

        full_name = com_prop_or_call(dim, "FullName") or canonical
        visited_dim_names.add(raw_name)
        visited_dim_names.add(raw_str)
        visited_dim_names.add(canonical)
        visited_dim_names.add(full_name)
        dimensions.append(
            DimensionItem(
                name=canonical,
                full_name=full_name,
                sketch_name=feat_name,
                value_m=val_m,
                dim_obj=dim,
                is_angular=is_angular
            )
        )


SKIP_FEATURE_TYPES = {
    "originprofilefeature", "refplane", "refaxis", "coordsys",
    "materialfolder", "favoritematerialfeature", "solidbodyfolder",
    "surfacebodyfolder", "annotationfolder", "selectionsetfolder",
    "commentsfolder", "historyfolder", "sensorfolder"
}


def _extract_dimensions_from_feature(feat, feat_name, dimensions, visited_dim_names, sw_model=None):
    """Walk all DisplayDimensions and direct parameters on any feature (Fillets, Chamfers, Sketches, Extrusions)."""
    if feat is None:
        return

    # 1. Walk DisplayDimension linked list (works on sketches, fillets, extrusions, revolves)
    dd = com_prop_or_call(feat, "GetFirstDisplayDimension")
    while dd is not None:
        dim = com_prop_or_call(dd, "GetDimension2", 0)
        if dim is None:
            dim = com_prop_or_call(dd, "GetDimension")

        if dim is not None:
            _process_dimension(dim, feat, feat_name, dimensions, visited_dim_names)

        dd = com_prop_or_call(feat, "GetNextDisplayDimension", dd)

    # 2. Check direct feature parameters (e.g. D1@Fillet1, Radius, D1@Chamfer1)
    # Ensures fillet radii and feature parameters are captured even if display dimension visibility is toggled off
    for pname in ("D1", "D2", "Radius", "FilletRadius", "Distance", "Angle"):
        dim = None
        try:
            dim = com_prop_or_call(feat, "Parameter", pname)
        except Exception:
            pass
        if dim is None and sw_model is not None:
            try:
                dim = safe_call(sw_model, "Parameter", f"{pname}@{feat_name}")
            except Exception:
                pass
        if dim is not None:
            _process_dimension(dim, feat, feat_name, dimensions, visited_dim_names)


def get_sketch_dimensions(sw_model):
    """Enumerate all driving dimensions (sketches, fillets, extrusions, chamfers) from active SolidWorks model."""
    dimensions = []
    visited_dim_names = set()

    if not sw_model:
        return dimensions

    # Enable dimension display visibility so display dimensions are loaded
    try:
        sw_model.ShowFeatureDimensions()
    except Exception:
        pass
    try:
        sw_model.SetUserPreferenceToggle(32, True)   # swDisplayFeatureDimensions
        sw_model.SetUserPreferenceToggle(616, True)  # swHideShowSketchDimensions
    except Exception:
        pass

    feat = com_prop_or_call(sw_model, "FirstFeature")
    while feat is not None:
        fname = com_prop_or_call(feat, "Name") or "Feature"
        ftype = (com_prop_or_call(feat, "GetTypeName2") or "").lower()

        if ftype not in SKIP_FEATURE_TYPES:
            # 1. Extract dimensions from the feature itself (Fillets, Chamfers, Extrusions, Top-level Sketches)
            _extract_dimensions_from_feature(feat, fname, dimensions, visited_dim_names, sw_model)

            # 2. Extract dimensions from any sub-features (sketches absorbed under Extrude, Cut, Boss, Revolve, etc.)
            sub = com_prop_or_call(feat, "GetFirstSubFeature")
            while sub is not None:
                sname = com_prop_or_call(sub, "Name") or "Sketch"
                stype = (com_prop_or_call(sub, "GetTypeName2") or "").lower()
                if stype not in SKIP_FEATURE_TYPES:
                    _extract_dimensions_from_feature(sub, sname, dimensions, visited_dim_names, sw_model)
                sub = com_prop_or_call(sub, "GetNextSubFeature")

        feat = com_prop_or_call(feat, "GetNextFeature")

    # 3. Check Equation Manager for any global / equation-driven dimensions
    try:
        eq_mgr = com_prop_or_call(sw_model, "GetEquationMgr")
        if eq_mgr:
            eq_count = com_prop_or_call(eq_mgr, "GetCount") or 0
            for i in range(eq_count):
                eq_str = com_prop_or_call(eq_mgr, "Equation", i) or ""
                if "=" in eq_str:
                    lhs = eq_str.split("=")[0].strip().strip('"')
                    if "@" in lhs and lhs not in visited_dim_names:
                        dim = safe_call(sw_model, "Parameter", lhs)
                        if dim is not None:
                            _process_dimension(dim, None, lhs.split("@")[-1], dimensions, visited_dim_names)
    except Exception:
        pass

    # 4. Check active selections in SelectionManager (if user clicked a feature or dimension)
    try:
        sel_mgr = com_prop_or_call(sw_model, "SelectionManager")
        if sel_mgr:
            sel_count = com_prop_or_call(sel_mgr, "GetSelectedObjectCount2", -1) or 0
            for i in range(1, sel_count + 1):
                sel_obj = com_prop_or_call(sel_mgr, "GetSelectedObject6", i, -1)
                if sel_obj:
                    dim = com_prop_or_call(sel_obj, "GetDimension2", 0) or com_prop_or_call(sel_obj, "GetDimension")
                    if dim is not None:
                        _process_dimension(dim, sel_obj, "Selected", dimensions, visited_dim_names)
                    else:
                        s_feat = com_prop_or_call(sel_obj, "GetSpecificFeature2") or sel_obj
                        s_fname = com_prop_or_call(s_feat, "Name") or "Selected"
                        _extract_dimensions_from_feature(s_feat, s_fname, dimensions, visited_dim_names, sw_model)
    except Exception:
        pass

    return dimensions


def find_dimension_by_name(sw_model, query_name):
    """Attempt to locate a dimension in the model by name or pattern."""
    if not sw_model or not query_name:
        return None

    clean_name = query_name.strip().strip('"')
    if not clean_name:
        return None

    # 1. Direct model.Parameter lookup
    dim = safe_call(sw_model, "Parameter", clean_name)
    if dim is not None:
        val = safe_call(dim, "GetSystemValue2", "")
        if val is not None:
            full_name = safe_call(dim, "GetNameForSelection") or clean_name
            sketch_name = clean_name.split("@")[-1] if "@" in clean_name else "Model"
            dim_type = safe_call(dim, "GetType")
            is_angular = (dim_type == 1) or any(k in clean_name.lower() for k in ["angle", "deg", "sweep"])
            return DimensionItem(
                name=clean_name,
                full_name=full_name,
                sketch_name=sketch_name,
                value_m=float(val),
                dim_obj=dim,
                is_angular=is_angular
            )

    # 2. If no '@' in name, try appending feature names from the model
    if "@" not in clean_name:
        feat_count = safe_call(sw_model, "GetFeatureCount") or 0
        for pos in range(feat_count):
            feat = safe_call(sw_model, "FeatureByPositionReverse", pos)
            if feat:
                fname = safe_getattr(feat, "Name")
                if fname:
                    pat = f"{clean_name}@{fname}"
                    dim = safe_call(sw_model, "Parameter", pat)
                    if dim is not None:
                        val = safe_call(dim, "GetSystemValue2", "")
                        if val is not None:
                            full_name = safe_call(dim, "GetNameForSelection") or pat
                            dim_type = safe_call(dim, "GetType")
                            is_angular = (dim_type == 1) or any(k in pat.lower() for k in ["angle", "deg", "sweep"])
                            return DimensionItem(
                                name=pat,
                                full_name=full_name,
                                sketch_name=fname,
                                value_m=float(val),
                                dim_obj=dim,
                                is_angular=is_angular
                            )
    return None


def get_dimensions_from_selection(sw_model):
    """Extract all dimensions currently selected in SolidWorks."""
    if not sw_model:
        return []
    sel_mgr = safe_getattr(sw_model, "SelectionManager")
    if not sel_mgr:
        return []
    sel_count = safe_call(sel_mgr, "GetSelectedObjectCount2", -1) or 0
    results = []
    seen = set()

    for i in range(1, sel_count + 1):
        sel_obj = safe_call(sel_mgr, "GetSelectedObject6", i, -1)
        if not sel_obj:
            continue
        dim = safe_call(sel_obj, "GetDimension")
        if dim is None:
            dim = safe_call(sel_obj, "GetDimension2", 0)
        if dim is None and hasattr(sel_obj, "Parameter"):
            dim = sel_obj
        if dim is not None:
            name = safe_call(dim, "GetNameForSelection") or safe_getattr(dim, "FullName") or safe_getattr(dim, "Name")
            val = safe_call(dim, "GetSystemValue2", "")
            if name and val is not None and name not in seen:
                seen.add(name)
                dim_type = safe_call(dim, "GetType")
                is_angular = (dim_type == 1) or any(k in name.lower() for k in ["angle", "deg", "sweep"])
                results.append(
                    DimensionItem(
                        name=name,
                        full_name=name,
                        sketch_name=name.split("@")[-1] if "@" in name else "Selected",
                        value_m=float(val),
                        dim_obj=dim,
                        is_angular=is_angular
                    )
                )
    return results


def get_dimension_from_selection(sw_model):
    """Extract a single dimension from selection (backwards compatibility)."""
    dims = get_dimensions_from_selection(sw_model)
    return dims[0] if dims else None


def count_feature_errors(sw_model):
    """Count features in error or warning state."""
    count = 0
    feat_count = safe_call(sw_model, "GetFeatureCount") or 0
    if feat_count > 0:
        for pos in range(feat_count):
            feat = safe_call(sw_model, "FeatureByPositionReverse", pos)
            if feat:
                try:
                    status = safe_call(feat, "GetErrorCode2")
                    if status is not None and status != 0:
                        count += 1
                except Exception:
                    pass
    return count


def rebuild_model(sw_model):
    """Trigger a smart rebuild."""
    if not sw_model:
        return False
    try:
        res = safe_call(sw_model, "EditRebuild3")
        if res is True or res == 1:
            return True
        res_force = safe_call(sw_model, "ForceRebuild3", False)
        if res_force is True or res_force == 1:
            return True
    except Exception:
        pass
    return True


def make_evaluator(sw_model, dimensions, bodies, spin_axis):
    """Create the evaluation closure evaluate(xs) -> (offset_a, offset_b) for the solver."""
    baseline_errors = count_feature_errors(sw_model)

    def evaluate(xs):
        for dim, val in zip(dimensions, xs):
            dim.set_value(val)

        rebuild_model(sw_model)

        err_cnt = count_feature_errors(sw_model)
        if err_cnt > baseline_errors:
            raise RebuildFailed(f"Feature rebuild generated new errors ({err_cnt} vs baseline {baseline_errors}).")

        try:
            # If single body or whole part, passing bodies=None avoids stale IBody2 COM pointers after rebuild
            eval_bodies = bodies if (bodies and len(bodies) > 1) else None
            _, com = get_mass_properties(sw_model, eval_bodies)
        except Exception as e:
            raise RebuildFailed(f"Could not compute mass properties: {e}")

        return spin_axis.offset_mm(com)

    return evaluate
