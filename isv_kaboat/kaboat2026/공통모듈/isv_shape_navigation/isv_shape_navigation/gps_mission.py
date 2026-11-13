"""Legacy GPS-to-visual-docking API used by the existing course mission."""
from importlib import import_module as _import_module
import sys as _sys
_sys.modules[__name__] = _import_module("isv_shape_navigation.gps_docking_legacy")
