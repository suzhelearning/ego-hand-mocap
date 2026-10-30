"""Nearest timestamp pairing on an unchanged left-camera timeline."""

import numpy as np


def pair_timestamps(left_timestamp_ns, right_timestamp_ns, *, max_skew_ms=5.0):
    """Return nearest indices, signed right-minus-left skew and acceptance mask.

    Integer nanoseconds avoid precision loss. Ties choose the earlier right
    frame. Reuse of a nearest right frame is explicit via right_reused; there
    is no synthetic interpolation, offset estimation or drift correction.
    """
    arrays = []
    for values in (left_timestamp_ns, right_timestamp_ns):
        a = np.asarray(values)
        if a.ndim != 1 or (a.size and a.dtype.kind not in "iu"):
            raise ValueError("Timestamps must be 1D integer nanoseconds")
        a = a.astype(np.int64)
        if np.any(a < 0) or np.any(a[1:] <= a[:-1]):
            raise ValueError("Timestamps must be nonnegative and strictly increasing")
        arrays.append(a)
    if not np.isfinite(max_skew_ms) or max_skew_ms < 0:
        raise ValueError("max_skew_ms must be finite and nonnegative")
    left, right = arrays
    index = np.full(len(left), -1, dtype=np.int64)
    skew = np.full(len(left), np.nan)
    reused = np.zeros(len(left), bool)
    if len(right):
        hi = np.clip(np.searchsorted(right, left), 0, len(right)-1)
        lo = np.maximum(hi-1, 0)
        index = np.where(abs(right[lo]-left) <= abs(right[hi]-left), lo, hi)
        skew = (right[index]-left).astype(np.float64) / 1e6
        _, inverse, counts = np.unique(index, return_inverse=True, return_counts=True)
        reused = counts[inverse] > 1
    return {"right_index": index, "skew_ms": skew,
            "valid": np.isfinite(skew) & (abs(skew) <= max_skew_ms), "right_reused": reused}
