from datetime import datetime
import numpy as np
import json
import os


def create_run_directory(label=None):

    base_path = "results"
    os.makedirs(base_path, exist_ok=True)

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    if label is not None:
        timestamp = label + "__" + timestamp

    path = os.path.join(base_path, f"{timestamp}")
    os.makedirs(path)

    return path


def save_single_run(config, data, label=None):

    dir_path = create_run_directory(label)

    with open(os.path.join(dir_path, "config.json"), "w") as file:
        json.dump(config.to_dict(), file, indent=4)

    np.save(os.path.join(dir_path, "com.npy"), data)

    return dir_path