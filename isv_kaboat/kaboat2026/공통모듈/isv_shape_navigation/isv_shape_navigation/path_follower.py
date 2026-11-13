"""Compatibility import; implementation: 주행코드/GPS호핑/path_follower.py."""
from importlib import import_module as _import_module
import sys as _sys
_sys.modules[__name__] = _import_module("kaboat_gps_hopping.path_follower")
