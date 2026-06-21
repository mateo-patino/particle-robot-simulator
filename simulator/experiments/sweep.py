from config.config import SimulationConfig
from simulator.simulation.engine import Simulation
from simulator.io.save import create_run_directory
from simulator.experiments.single_run import run_fast_save
from copy import deepcopy
from typing import Any
import json
import os
import numpy as np
import logging

logger = logging.getLogger(__name__)


"""
This function is similar to run_1d_sweep except that it creates and saves a .npy file immediately after
every simulation run is complete by calling run_fast_save. This function does not create the final dict
data structure that run_1d_sweep does because it calls np.save immediately after MuJoCo terminates.

"""
def run_1d_sweep_fs(config: SimulationConfig, target_parameter: str, values: list[Any] | None = None, 
                    num_runs: int = 1, label: str | None = None, existing_runs: int = 0, argv: list[str] | None = None):
    
    if values is None or len(values) == 0:
        raise ValueError("A non-empty set of values to sweep must be provided.")

    if not hasattr(config, target_parameter):
        raise ValueError(f"Cannot sweep \"{target_parameter}\": not a valid configuration parameter.")

    logger.info("Sweeping \"%s\" over the values %s", target_parameter, values)

    # Create directory for results where all data files will be saved
    dir_path = create_run_directory(label)

    for i, value in enumerate(values, start=1):

        logger.info("----------------------------------------------------------------")
        logger.info("Running %s = %s (%d out of %d)", target_parameter, value, i, len(values))

        for run in range(1, num_runs + 1):

            new_config = deepcopy(config)
            setattr(new_config, target_parameter, value)

            # Ensure variability across runs by shifting the seed by +1
            if config.seed is not None:
                new_config.seed = config.seed + existing_runs + run - 1

            logger.info("Run %d started", run)
            out_path = os.path.join(dir_path, f"{target_parameter}_{value}_{existing_runs + run}.npy")
            run_fast_save(new_config, out_path) # Run and fast save
            logger.info("Run %d complete", run)

    # run_fast_save does not save metadata files, so do it manually
    with open(os.path.join(dir_path, "sweep_metadata.json"), "w") as file:
        cmd = " ".join(argv).strip() if argv is not None else "argv is None"
        metadata = {
            "target_parameter": target_parameter,
            "values": values,
            "runs": num_runs,
            "existing_runs": existing_runs,
            "cmd": cmd
        }
        json.dump(metadata, file, indent=4)

    with open(os.path.join(dir_path, "base_config.json"), "w") as file:
        json.dump(config.to_dict(), file, indent=4)

    logger.info("Sweeping complete")

    return dir_path

"""

This function performs a parameter sweep on 'target_parameter' over 'values' and returns a dict 
data structure with the results.

Parameters: configuration, parameter to sweep, values to sweep over.
Returns: A 2D Python dictionary where the first dimension of keys corresponds to the value
(as a string) in the parameter space, and the second dimension of keys corresponds to the run number.

Note that this function does NOT save data to .npy files after simulation finishes (i.e. does not do a fast save)

"""

def run_1d_sweep(config: SimulationConfig, target_parameter: str, values: list[Any] | None = None, 
                 num_runs: int = 1, existing_runs: int = 0) -> dict[str, dict[int, np.ndarray]]:

    if values is None or len(values) == 0:
        raise ValueError("A non-empty set of values to sweep must be provided.")

    if not hasattr(config, target_parameter):
        raise ValueError(f"Cannot sweep \"{target_parameter}\": not a valid configuration parameter.")

    logger.info("Sweeping \"%s\" over the values %s", target_parameter, values)

    sweep_results: dict[str, dict[int, np.ndarray]] = {}
    for i, value in enumerate(values, start=1):

        logger.info("----------------------------------------------------------------")
        logger.info("Running %s = %s (%d out of %d)", target_parameter, value, i, len(values))

        results: dict[int, np.ndarray] = {}
        for run in range(1, num_runs + 1):

            new_config = deepcopy(config)
            setattr(new_config, target_parameter, value)

            # Ensure variability across runs by shifting the seed by +1
            if config.seed is not None:
                new_config.seed = config.seed + existing_runs + run - 1

            logger.info("Run %d started", run)
            sim = Simulation(new_config)
            results[run] = sim.run()
            logger.info("Run %d complete", run)

        sweep_results[str(value)] = results

    logger.info("Sweeping complete")

    return sweep_results
