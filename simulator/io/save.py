from datetime import datetime
from config.config import SimulationConfig
import numpy as np
import json
import os


def create_run_directory(label: str | None = None) -> str:

    base_path = "results"
    os.makedirs(base_path, exist_ok=True)

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    if label is not None:
        timestamp = label + "__" + timestamp

    path = os.path.join(base_path, f"{timestamp}")
    os.makedirs(path)

    return path


def save_single_run(config: SimulationConfig, data: dict[int, np.ndarray], label: str | None = None) -> str:

    dir_path = create_run_directory(label)

    with open(os.path.join(dir_path, "config.json"), "w") as file:
        json.dump(config.to_dict(), file, indent=4)

    with open(os.path.join(dir_path, "metadata.json"), "w") as file:
        metadata = {
            "runs": len(data),
        }
        json.dump(metadata, file, indent=4)

    for run, com_data in data.items():
        np.save(os.path.join(dir_path, f"com_{run}.npy"), com_data)

    return dir_path


def save_1d_sweep(base_config: SimulationConfig, results_dict: dict[str, dict[int, np.ndarray]], target_parameter: str, label: str | None = None) -> str:

    dir_path = create_run_directory(label)

    with open(os.path.join(dir_path, "base_config.json"), "w") as file:
        json.dump(base_config.to_dict(), file, indent=4)

    with open(os.path.join(dir_path, "sweep_metadata.json"), "w") as file:
        metadata = {
            "target_parameter": target_parameter,
            "values": list(results_dict.keys()),
            "runs": len(next(iter(results_dict.values())))
        }
        json.dump(metadata, file, indent=4)

    for value, run_dict in results_dict.items():
        for run, data in run_dict.items():
            np.save(os.path.join(dir_path, f"{target_parameter}_{value}_{run}.npy"), data)

    return dir_path
