from simulator.simulation.engine import Simulation

"""
Parameters: a number of runs and a configuration object
Returns: a dictionary of the results of the simulation
"""

def run_single(config, num_runs):
    results = {}
    for run in range(1, num_runs + 1):
        sim = Simulation(config)
        results[run] = sim.run()
    return results