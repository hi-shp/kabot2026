"""Compatibility import for the V3 avoidance module."""
from importlib import import_module as _import_module
import sys as _sys

_sys.modules[__name__] = _import_module("kaboat_avoidance.avoidance_v3")
