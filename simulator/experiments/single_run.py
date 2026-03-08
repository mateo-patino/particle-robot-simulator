import logging

from simulator.simulation.engine import Simulation
from config.config import SimulationConfig
from copy import deepcopy
import numpy as np

logger = logging.getLogger(__name__)

"""
Parameters: a number of runs and a configuration object
Returns: a dictionary of the results of the simulation where the keys represent the run numbers
"""

def run_single(config: SimulationConfig, num_runs: int) -> dict[int, np.ndarray]:
    results: dict[int, np.ndarray] = {}
    for run in range(1, num_runs + 1):

        # Offset seed by one each run if seed is set
        if config.seed is not None:
            new_config = deepcopy(config)
            new_config.seed = config.seed + run - 1
            sim = Simulation(new_config)
        else:
            sim = Simulation(config)

        logger.info("Run %d started", run)
        results[run] = sim.run()
        logger.info("Run %d complete", run)

    return results
