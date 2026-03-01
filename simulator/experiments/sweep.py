import logging
from typing import Any

from simulator.simulation.engine import Simulation
from config.config import SimulationConfig
from copy import deepcopy
import numpy as np

logger = logging.getLogger(__name__)

"""

Parameters: configuration, parameter to sweep, values to sweep over.
Returns: A 2D Python dictionary where the first dimension of keys corresponds to the value
(as a string) in the parameter space, and the second dimension of keys corresponds to the run number.

"""

def run_1d_sweep(config: SimulationConfig, target_parameter: str, values: list[Any] | None = None, num_runs: int = 1) -> dict[str, dict[int, np.ndarray]]:

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
                new_config.seed = config.seed + run - 1

            logger.info("Run %d started", run)
            sim = Simulation(new_config)
            results[run] = sim.run()
            logger.info("Run %d complete", run)

        sweep_results[str(value)] = results

    logger.info("Sweeping complete")

    return sweep_results
