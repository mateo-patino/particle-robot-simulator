from simulator.simulation.engine import Simulation
from copy import deepcopy

"""

Parameters: configuration, parameter to sweep, values to sweep over.
Returns: A Python dictionary where they keys correspond to each value in the sweeping space
and the value corresponds to the c.o.m array returned by the simulation.

"""

def run_1d_sweep(config, target_parameter, values=None):
    if values is None or len(values) == 0:
        raise ValueError(" A non-empty set of values to sweep must be provided.")
    
    if not hasattr(config, target_parameter):
        raise ValueError(f"{target_parameter} is not a valid configuration parameter.")
    
    print(f"\nSweeping \"{target_parameter}\" over the values {values}")

    results = dict()
    for i, value in enumerate(values, start=1):
        print("\n----------------------------------------------------------------")
        print(f"\nRunning {target_parameter} = {value} ({i} out of {len(values)})")
        new_config = deepcopy(config)
        setattr(new_config, target_parameter, value)

        sim = Simulation(new_config)
        results[str(value)] = sim.run()

    print("\nSweeping complete")

    return results
