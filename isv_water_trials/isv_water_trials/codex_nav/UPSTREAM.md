# CODEX planner source

Source: https://github.com/hi-shp/data_science/tree/codex

Revision: `8f8faf8983921b8e2479f259543f82c83dc74faa`.
Original Git blob identifiers are recorded in `UPSTREAM.json`.
Copied modules retain their algorithms; imports are package-qualified.

`outdoor_planner.py` subclasses `SamplingNavigator`. It replaces force allocation,
rollout integration and hull clearance with the frozen empirical PWM model and
the 1.1 m x 0.5 m vessel envelope. The original simulated dynamics remain in the
source dependency set but are not used for the real-boat candidate rollout.
Default reference-objective trajectory control is used; ETA/passage/forward-policy
experimental variants are not enabled. No simulator truth or precomputed future
sensor frames are passed to the adapter.
