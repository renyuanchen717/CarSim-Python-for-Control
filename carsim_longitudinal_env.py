"""A minimal throttle/brake-only CarSim environment.

This environment assumes the CarSim run has exactly two import variables:

1. throttle
2. brake

No steering input is sent by this wrapper. Configure the CarSim run so that
its import ports match this order.
"""

from __future__ import annotations

from typing import Dict, Optional, Sequence, Tuple

from carsim_io import DEFAULT_EXPORT_NAMES, clip01, parse_observation
from carsim_vs_solver import CarSimSolver


class CarSimLongitudinalEnv:
    """Tiny CarSim environment exposing only throttle and brake."""

    def __init__(
        self,
        sim_file: str,
        export_names: Sequence[str] = DEFAULT_EXPORT_NAMES,
        dt_control: float = 0.05,
    ) -> None:
        self.sim_file = sim_file
        self.export_names = list(export_names)
        self.dt_control = float(dt_control)
        self.solver = CarSimSolver()
        self.dll_path = self.solver.load(sim_file)

        self.config: Optional[Dict[str, float]] = None
        self.t = 0.0
        self.t_step = 0.0
        self.t_stop = 0.0
        self.steps_per_control = 1
        self.export_vars = []
        self.done = True

    def reset(self) -> Dict[str, float]:
        """Start a new CarSim run and return the initial observation."""

        if not self.done:
            self.close()

        self.config = self.solver.read_configuration(self.sim_file)
        n_import = int(self.config["n_import"])
        n_export = int(self.config["n_export"])

        if n_import != 2:
            self.solver.terminate_run(float(self.config["t_start"]))
            raise RuntimeError(
                "This demo expects exactly 2 CarSim imports: "
                "[throttle, brake]. Your simfile reports "
                f"n_import={n_import}."
            )
        if n_export != len(self.export_names):
            self.solver.terminate_run(float(self.config["t_start"]))
            raise RuntimeError(
                "Export name count does not match CarSim n_export: "
                f"{len(self.export_names)} names for {n_export} exports."
            )

        self.t = float(self.config["t_start"])
        self.t_stop = float(self.config["t_stop"])
        self.t_step = float(self.config["t_step"])
        self.steps_per_control = max(1, round(self.dt_control / self.t_step))
        self.export_vars = self.solver.copy_export_vars(n_export)
        self.done = False

        return parse_observation(self.export_vars, self.export_names)

    def step(self, throttle: float, brake: float) -> Tuple[Dict[str, float], bool, Dict[str, float]]:
        """Advance one Python control step.

        Parameters
        ----------
        throttle, brake:
            Scalars in [0, 1]. Values outside the range are clipped.
        """

        if self.done:
            raise RuntimeError("Call reset() before step(), or reset after done=True.")

        throttle = clip01(throttle)
        brake = clip01(brake)
        import_vars = [throttle, brake]

        return_code = 0
        for _ in range(self.steps_per_control):
            return_code, self.export_vars = self.solver.integrate_io(
                self.t,
                import_vars,
                self.export_vars,
            )
            self.t += self.t_step
            if return_code != 0:
                break
            if self.t_stop > 0.0 and self.t >= self.t_stop:
                break

        obs = parse_observation(self.export_vars, self.export_names)
        reached_end = return_code != 0 or (self.t_stop > 0.0 and self.t >= self.t_stop)

        info = {
            "time": self.t,
            "return_code": float(return_code),
            "throttle": throttle,
            "brake": brake,
        }

        if reached_end:
            self.solver.terminate_run(self.t)
            self.done = True

        return obs, self.done, info

    def close(self) -> None:
        """Terminate the current CarSim run if needed."""

        if not self.done:
            self.solver.terminate_run(self.t)
        self.done = True
