import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt


def load_telemetry(path: Path):
    with path.open("r", encoding="utf-8") as f:
        payload = json.load(f)
    data = payload.get("data", [])
    if not data:
        raise ValueError(f"No telemetry data found in {path}")
    return data


def series(data, key):
    return [row.get(key) for row in data]


def main():
    parser = argparse.ArgumentParser(description="Visualize VehicleTelemetry.json on a time axis")
    parser.add_argument(
        "--file",
        default=str(Path("Assets") / "VehicleTelemetry.json"),
        help="Path to VehicleTelemetry.json (default: Assets/VehicleTelemetry.json)",
    )
    parser.add_argument(
        "--show-wheels",
        action="store_true",
        help="Plot individual wheel speeds in a separate figure",
    )
    args = parser.parse_args()

    path = Path(args.file)
    if not path.exists():
        raise FileNotFoundError(path)

    data = load_telemetry(path)
    t = series(data, "time")

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("Vehicle Telemetry")

    axes[0].plot(t, series(data, "carSpeed"), label="carSpeed (km/h)")
    axes[0].plot(t, series(data, "wheelSpeed"), label="wheelSpeed (km/h)", alpha=0.7)
    axes[0].set_ylabel("Speed")
    axes[0].legend(loc="upper right")

    axes[1].plot(t, series(data, "rpm"), label="engineRPM")
    axes[1].set_ylabel("RPM")
    axes[1].legend(loc="upper right")

    axes[2].plot(t, series(data, "torque"), label="wheelTorque")
    axes[2].plot(t, series(data, "engineTorque"), label="engineTorque", alpha=0.7)
    axes[2].set_ylabel("Torque")
    axes[2].legend(loc="upper right")

    axes[3].plot(t, series(data, "slip"), label="wheelSlip")
    axes[3].plot(t, series(data, "steering"), label="steeringAngle", alpha=0.7)
    axes[3].set_ylabel("Slip/Steer")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(loc="upper right")

    fig.tight_layout()

    if args.show_wheels:
        fig2, ax2 = plt.subplots(1, 1, figsize=(12, 4))
        ax2.plot(t, series(data, "wheelSpeedFL"), label="FL")
        ax2.plot(t, series(data, "wheelSpeedFR"), label="FR")
        ax2.plot(t, series(data, "wheelSpeedRL"), label="RL")
        ax2.plot(t, series(data, "wheelSpeedRR"), label="RR")
        ax2.set_title("Wheel Speeds")
        ax2.set_ylabel("km/h")
        ax2.set_xlabel("Time (s)")
        ax2.legend(loc="upper right")
        fig2.tight_layout()

    plt.show()


if __name__ == "__main__":
    main()
