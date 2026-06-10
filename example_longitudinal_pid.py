"""Run a minimal throttle/brake CarSim demo.

Example:
    python example_longitudinal_pid.py ^
        --sim-file ./simfile.sim ^
        --target-speed 25 ^
        --duration 10

The CarSim run must expose exactly two imports in this order:
    [throttle, brake]
"""

from __future__ import annotations

import argparse
import csv
from collections import deque
from pathlib import Path
from typing import Tuple

from carsim_longitudinal_env import CarSimLongitudinalEnv


class SpeedPID:
    """Small longitudinal PID that outputs throttle/brake in [0, 1]."""

    def __init__(self, kp: float = 0.25, ki: float = 0.02, kd: float = 0.02, dt: float = 0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.errors = deque(maxlen=20)

    def step(self, target_speed: float, current_speed: float) -> Tuple[float, float]:
        error = target_speed - current_speed
        self.errors.append(error)

        if len(self.errors) >= 2:
            derivative = (self.errors[-1] - self.errors[-2]) / self.dt
            integral = sum(self.errors) * self.dt
        else:
            derivative = 0.0
            integral = 0.0

        command = self.kp * error + self.ki * integral + self.kd * derivative
        command = max(-1.0, min(1.0, command))

        if command >= 0.0:
            return command, 0.0
        return 0.0, -command


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Minimal CarSim throttle/brake PID demo")
    parser.add_argument("--sim-file", default="./simfile.sim", help="Path to CarSim simfile.sim")
    parser.add_argument(
        "--export-names",
        default="Vx",
        help="Comma-separated CarSim export names in the exact configured order",
    )
    parser.add_argument("--target-speed", type=float, default=25.0, help="Target ego speed in m/s")
    parser.add_argument("--duration", type=float, default=10.0, help="Demo duration in seconds")
    parser.add_argument("--dt-control", type=float, default=0.05, help="Python control period in seconds")
    parser.add_argument("--log-csv", default="longitudinal_pid_log.csv", help="Output CSV path")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    export_names = [name.strip() for name in args.export_names.split(",") if name.strip()]

    env = CarSimLongitudinalEnv(
        sim_file=args.sim_file,
        export_names=export_names,
        dt_control=args.dt_control,
    )
    controller = SpeedPID(dt=args.dt_control)

    log_path = Path(args.log_csv)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    obs = env.reset()
    max_steps = int(round(args.duration / args.dt_control))

    with log_path.open("w", newline="", encoding="utf-8") as f:
        fieldnames = [
            "time",
            "vx",
            "target_speed",
            "speed_error",
            "throttle",
            "brake",
            "return_code",
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for _ in range(max_steps):
            throttle, brake = controller.step(args.target_speed, obs["vx"])
            obs, done, info = env.step(throttle, brake)

            writer.writerow(
                {
                    "time": info["time"],
                    "vx": obs["vx"],
                    "target_speed": args.target_speed,
                    "speed_error": args.target_speed - obs["vx"],
                    "throttle": info["throttle"],
                    "brake": info["brake"],
                    "return_code": info["return_code"],
                }
            )

            if done:
                break

    env.close()
    print(f"Demo finished. Log written to: {log_path.resolve()}")


if __name__ == "__main__":
    main()
