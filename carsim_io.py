"""Utilities for parsing CarSim export arrays.

CarSim export channels are configurable. This file keeps parsing explicit:
pass the export variable names in the same order used by the CarSim run.
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, List, Mapping, Sequence


DEFAULT_EXPORT_NAMES = [
    "Vx"
]


def normalize_export_names(names: Iterable[str]) -> List[str]:
    """Normalize CarSim export names for case-insensitive lookup."""

    return [name.strip() for name in names]


def export_array_to_dict(
    export_vars: Sequence[float],
    export_names: Sequence[str],
) -> Dict[str, float]:
    """Map an export array to ``{name: value}``.

    ``export_names`` must match the order configured in the CarSim run.
    """

    if len(export_vars) != len(export_names):
        raise ValueError(
            f"export length mismatch: got {len(export_vars)} values, "
            f"but {len(export_names)} names were provided"
        )
    return {name: float(value) for name, value in zip(export_names, export_vars)}


def _first(data: Mapping[str, float], *names: str, default: float = 0.0) -> float:
    lower_map = {key.lower(): value for key, value in data.items()}
    for name in names:
        if name.lower() in lower_map:
            return float(lower_map[name.lower()])
    return float(default)


def parse_observation(
    export_vars: Sequence[float],
    export_names: Sequence[str] = DEFAULT_EXPORT_NAMES,
) -> Dict[str, float]:
    """Convert raw CarSim exports into a small longitudinal observation.

    Supported common exports:
    - ``Vx``: ego speed, usually km/h in CarSim outputs

    If your run uses different export variables, pass a matching
    ``export_names`` list and extend this parser as needed.
    """

    raw = export_array_to_dict(export_vars, export_names)

    vx_kph = _first(raw, "Vx", default=0.0)


    obs = {
        "vx_kph": vx_kph,
        "vx": vx_kph / 3.6
    }

    # Keep raw channels for logging/debugging.
    for name, value in raw.items():
        obs[f"raw_{name}"] = value

    return obs


def clip01(value: float) -> float:
    """Clamp a scalar to [0, 1]."""

    return max(0.0, min(1.0, float(value)))
