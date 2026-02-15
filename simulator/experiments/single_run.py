from simulator.simulation.engine import Simulation
from copy import deepcopy

"""
Parameters: a number of runs and a configuration object
Returns: a dictionary of the results of the simulation
"""

def run_single(config, num_runs):
    results = {}
    for run in range(1, num_runs + 1):

        # Offset seed by one each run if seed is set
        if config.seed is not None:
            new_config = deepcopy(config)
            new_config.seed = config.seed + run - 1
            sim = Simulation(new_config)
        else:
            sim = Simulation(config)

        results[run] = sim.run()

    return results