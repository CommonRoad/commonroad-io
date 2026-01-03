import numpy as np


def round_and_hash(*args: float) -> int:
    # Hash stable representation: round floats to avoid tiny numerical differences
    rounded = tuple(np.round(args, 9))
    return hash(rounded)
