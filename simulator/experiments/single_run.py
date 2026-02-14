from simulator.simulation.engine import Simulation

def run_single(config):
    sim = Simulation(config)
    return sim.run()