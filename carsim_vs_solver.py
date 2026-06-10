"""Minimal Python wrapper for the CarSim/VehicleSim solver DLL.

This module intentionally keeps only the APIs needed for a simple
Python-in-the-loop demo:

- read the solver DLL path from a ``simfile.sim``
- initialize a CarSim run
- exchange import/export arrays step by step
- terminate the run cleanly
"""

from __future__ import annotations

import ctypes
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


class CarSimSolver:
    """Small ctypes wrapper around the CarSim solver DLL."""

    def __init__(self) -> None:
        self.dll_handle = None

    @staticmethod
    def _char_pointer(value: str) -> ctypes.c_char_p:
        return ctypes.c_char_p(value.encode("utf-8"))

    @staticmethod
    def _parameter_value(line: str) -> Optional[str]:
        parts = line.strip().split(maxsplit=1)
        if len(parts) < 2:
            return None
        return parts[1].strip()

    def get_dll_path(self, sim_file: str) -> str:
        """Return the solver DLL path declared in ``simfile.sim``.

        The simplest and most robust setup is to keep ``DLLFILE`` in the
        simfile. If it is missing, this method tries the standard CarSim
        Windows solver location using ``PROGDIR`` and ``PRODUCT_ID``.
        """

        sim_path = Path(sim_file)
        if not sim_path.exists():
            raise FileNotFoundError(f"simfile not found: {sim_file}")

        dll_path = None
        prog_dir = None
        product_id = "CarSim"
        product_ver = None
        vehicle_code = None
        lib_key = "SOFILE" if sys.platform == "linux" else "DLLFILE"

        with sim_path.open("r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                stripped = line.lstrip()
                if stripped.startswith(lib_key):
                    dll_path = self._parameter_value(line)
                elif stripped.startswith("PROGDIR"):
                    prog_dir = self._parameter_value(line)
                elif stripped.startswith("PRODUCT_ID"):
                    product_id = self._parameter_value(line) or product_id
                elif stripped.startswith("PRODUCT_VER"):
                    product_ver = self._parameter_value(line)
                elif stripped.startswith("VEHICLE_CODE"):
                    vehicle_code = self._parameter_value(line)

        if dll_path:
            return dll_path

        if not prog_dir:
            raise RuntimeError("DLLFILE is missing and PROGDIR was not found in simfile")

        if sys.platform == "linux":
            if product_id == "TruckSim":
                library_name = f"libtrucksim.so.{product_ver}"
            elif product_id == "BikeSim":
                library_name = f"libbikesim.so.{product_ver}"
            else:
                library_name = f"libcarsim.so.{product_ver}"
            return os.path.join(prog_dir, library_name)

        suffix = "_64" if ctypes.sizeof(ctypes.c_void_p) == 8 else "_32"
        if product_id == "TruckSim":
            library_name = f"trucksim{suffix}.dll"
        elif product_id == "BikeSim":
            library_name = f"bikesim{suffix}.dll"
        elif vehicle_code and "tire" in vehicle_code.lower():
            library_name = f"tire{suffix}.dll"
        else:
            library_name = f"carsim{suffix}.dll"
        return os.path.join(prog_dir, "Programs", "Solvers", library_name)

    def load(self, sim_file: str) -> str:
        """Load the solver DLL and validate required functions."""

        dll_path = self.get_dll_path(sim_file)
        self.dll_handle = ctypes.CDLL(dll_path)

        required = [
            "vs_read_configuration",
            "vs_integrate_io",
            "vs_copy_export_vars",
            "vs_terminate_run",
            "vs_error_occurred",
            "vs_get_error_message",
        ]
        missing = [name for name in required if not hasattr(self.dll_handle, name)]
        if missing:
            raise RuntimeError(f"solver DLL is missing required APIs: {missing}")

        return dll_path

    def read_configuration(self, sim_file: str) -> Dict[str, float]:
        """Read run configuration and initialize the solver run."""

        if self.dll_handle is None:
            raise RuntimeError("call load(sim_file) before read_configuration()")

        if sys.platform == "linux":
            ref_n_import = ctypes.c_long()
            ref_n_export = ctypes.c_longlong()
        else:
            ref_n_import = ctypes.c_int32()
            ref_n_export = ctypes.c_int64()

        ref_t_start = ctypes.c_double()
        ref_t_stop = ctypes.c_double()
        ref_t_step = ctypes.c_double()

        self.dll_handle.vs_read_configuration(
            self._char_pointer(sim_file),
            ctypes.byref(ref_n_import),
            ctypes.byref(ref_n_export),
            ctypes.byref(ref_t_start),
            ctypes.byref(ref_t_stop),
            ctypes.byref(ref_t_step),
        )

        return {
            "n_import": int(ref_n_import.value),
            "n_export": int(ref_n_export.value),
            "t_start": float(ref_t_start.value),
            "t_stop": float(ref_t_stop.value),
            "t_step": float(ref_t_step.value),
        }

    def copy_export_vars(self, n_export: int) -> List[float]:
        """Return the current CarSim export array."""

        if self.dll_handle is None:
            raise RuntimeError("solver DLL is not loaded")

        export_array = (ctypes.c_double * n_export)()
        self.dll_handle.vs_copy_export_vars(
            ctypes.cast(export_array, ctypes.POINTER(ctypes.c_double))
        )
        return [float(export_array[i]) for i in range(n_export)]

    def integrate_io(
        self,
        t_current: float,
        import_vars: Sequence[float],
        export_vars: Sequence[float],
    ) -> Tuple[int, List[float]]:
        """Advance one solver step using Python import variables."""

        if self.dll_handle is None:
            raise RuntimeError("solver DLL is not loaded")

        import_array = (ctypes.c_double * len(import_vars))(*import_vars)
        export_array = (ctypes.c_double * len(export_vars))(*export_vars)

        code = self.dll_handle.vs_integrate_io(
            ctypes.c_double(t_current),
            ctypes.byref(import_array),
            ctypes.byref(export_array),
        )

        return int(code), [float(export_array[i]) for i in range(len(export_vars))]

    def terminate_run(self, t_current: float) -> None:
        """Terminate the current CarSim run."""

        if self.dll_handle is not None:
            self.dll_handle.vs_terminate_run(ctypes.c_double(t_current))

    def get_error_message(self) -> str:
        """Return the current solver error message, if available."""

        if self.dll_handle is None:
            return ""
        message_ptr = self.dll_handle.vs_get_error_message()
        if not message_ptr:
            return ""
        return ctypes.c_char_p(message_ptr).value.decode("ascii", errors="ignore")
