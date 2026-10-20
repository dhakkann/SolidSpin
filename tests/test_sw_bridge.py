"""
Unit tests for SolidWorks COM bridge utilities and dimension enumeration.
Tests:
1. com_prop_or_call helper on mock properties and methods.
2. get_sketch_dimensions feature tree traversal on mock SolidWorks model hierarchy.
"""

from sw_balancer.sw_bridge import com_prop_or_call, get_sketch_dimensions, DimensionItem


class MockDimension:
    def __init__(self, name, full_name, val_m, is_driven=False, dim_type=0):
        self._name = name
        self._full_name = full_name
        self._val_m = val_m
        self._is_driven = is_driven
        self._dim_type = dim_type

    @property
    def Name(self):
        return self._name

    @property
    def FullName(self):
        return self._full_name

    @property
    def DrivenState(self):
        return 1 if self._is_driven else 2  # 1 = driven, 2 = driving

    @property
    def SystemValue(self):
        return self._val_m

    def GetType(self):
        return self._dim_type


class MockDisplayDimension:
    def __init__(self, dim):
        self._dim = dim

    def GetDimension2(self, index):
        return self._dim


class MockSketchFeature:
    def __init__(self, name, dims):
        self._name = name
        self._type = "ProfileFeature"
        self._dds = [MockDisplayDimension(d) for d in dims]

    @property
    def Name(self):
        return self._name

    def GetTypeName2(self):
        return self._type

    @property
    def GetFirstDisplayDimension(self):
        return self._dds[0] if self._dds else None

    def GetNextDisplayDimension(self, cur_dd):
        idx = self._dds.index(cur_dd)
        if idx + 1 < len(self._dds):
            return self._dds[idx + 1]
        return None


class MockExtrudeFeature:
    def __init__(self, name, sub_sketches):
        self._name = name
        self._type = "Extrusion"
        self._subs = sub_sketches

    @property
    def Name(self):
        return self._name

    def GetTypeName2(self):
        return self._type

    @property
    def GetFirstSubFeature(self):
        return self._subs[0] if self._subs else None

    def GetNextSubFeature(self):
        return None


class MockModel:
    def __init__(self, features):
        self._features = features

    def ShowFeatureDimensions(self):
        pass

    def SetUserPreferenceToggle(self, toggle, val):
        pass

    @property
    def FirstFeature(self):
        return self._features[0] if self._features else None

    def GetNextFeature(self):
        return None

    def GetEquationMgr(self):
        return None

    @property
    def SelectionManager(self):
        return None


def test_com_prop_or_call():
    dim = MockDimension("Radius1", "Radius1@Sketch1", 0.085)
    # Test property access
    assert com_prop_or_call(dim, "Name") == "Radius1"
    assert com_prop_or_call(dim, "SystemValue") == 0.085
    # Test method invocation with args
    dd = MockDisplayDimension(dim)
    assert com_prop_or_call(dd, "GetDimension2", 0) == dim
    # Test method invocation with 0 args
    assert com_prop_or_call(dim, "GetType") == 0


def test_get_sketch_dimensions_traversal():
    # Setup mock hierarchy of a CAD part with sketches and extrusions
    dims = [
        MockDimension("BoreDia", "BoreDia@Sketch1", 0.012),
        MockDimension("AngleParam", "AngleParam@Sketch1", 0.785, dim_type=1),
        MockDimension("Radius1", "Radius1@Sketch1", 0.090),
        MockDimension("ReferenceDim", "Ref@Sketch1", 0.050, is_driven=True),  # Should be skipped!
    ]
    sketch = MockSketchFeature("Sketch1", dims)
    extrude = MockExtrudeFeature("Boss-Extrude1", [sketch])
    model = MockModel([extrude])

    discovered = get_sketch_dimensions(model)

    # 3 driving dimensions should be found, ReferenceDim skipped
    assert len(discovered) == 3
    names = [d.name for d in discovered]
    assert "BoreDia@Sketch1" in names
    assert "AngleParam@Sketch1" in names
    assert "Radius1@Sketch1" in names
    assert not any("Ref" in n for n in names)

    # Verify AngleParam is marked as angular
    angle_dim = next(d for d in discovered if "AngleParam" in d.name)
    assert angle_dim.is_angular is True
    assert angle_dim.display_unit == "deg"
