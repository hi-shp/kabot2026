"""Batch dispatch for the unchanged per-knot yaw slew limit."""
import numpy as np

try:
    from numba import njit
except ImportError:
    njit = None


def slew_yaw_reference(sequences, previous, step):
    prior = (np.full(len(sequences), previous) if np.ndim(previous) == 0
             else np.asarray(previous))
    for tick in range(sequences.shape[1]):
        sequences[:, tick, 1] = np.clip(sequences[:, tick, 1],
                                      prior-step, prior+step)
        prior = sequences[:, tick, 1]


def _slew_yaw(sequences, previous, step):
    for candidate in range(len(sequences)):
        prior = previous[0] if len(previous) == 1 else previous[candidate]
        for tick in range(sequences.shape[1]):
            value = np.minimum(np.maximum(sequences[candidate, tick, 1],
                                          prior-step), prior+step)
            sequences[candidate, tick, 1] = value
            prior = value


compiled_slew_yaw = njit(cache=True)(_slew_yaw) if njit is not None else None


def slew_yaw(sequences, previous, step):
    if compiled_slew_yaw is not None and sequences.dtype == np.float64:
        compiled_slew_yaw(sequences, np.asarray(previous, dtype=np.float64).reshape(-1), step)
    else:
        slew_yaw_reference(sequences, previous, step)


def warmup_command_arrays():
    slew_yaw(np.zeros((1, 1, 2)), 0., .1)
