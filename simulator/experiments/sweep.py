from simulator.simulation.engine import Simulation
from copy import deepcopy

"""

Parameters: configuration, parameter to sweep, values to sweep over.
Returns: A 2D Python dictionary where the first dimension of keys corresponds to the value
(as a string) in the parameter space, and the second dimension of keys corresponds to the run number.

"""

def run_1d_sweep(config, target_parameter, values=None, num_runs=1):

    if values is None or len(values) == 0:
        raise ValueError("A non-empty set of values to sweep must be provided.")
    
    if not hasattr(config, target_parameter):
        raise ValueError(f"Cannot sweep \"{target_parameter}\": not a valid configuration parameter.")
    
    print(f"\nSweeping \"{target_parameter}\" over the values {values}")

    sweep_results = {}
    for i, value in enumerate(values, start=1):

        print("\n----------------------------------------------------------------")
        print(f"\nRunning {target_parameter} = {value} ({i} out of {len(values)})")

        results = {}
        for run in range(1, num_runs + 1):

            new_config = deepcopy(config)
            setattr(new_config, target_parameter, value)

            # Ensure variability across runs by shifting the seed by +1
            if config.seed is not None:
                new_config.seed = config.seed + run - 1

            print(f"\nRun {run} started.........")
            sim = Simulation(new_config)
            results[run] = sim.run()
            print(f"\nRun {run} complete")

        sweep_results[str(value)] = results

    print("\nSweeping complete")

    return sweep_results
