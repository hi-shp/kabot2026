"""Compatibility import; implementation: 주행코드/장애물회피/route_mission.py."""
from importlib import import_module as _import_module
import sys as _sys
_sys.modules[__name__] = _import_module("kaboat_avoidance.route_mission")
