import numpy as np
from config.config import SimulationConfig
from simulator.simulation.engine import Simulation

def test_initialize_directions_length(small_config):
    sim = Simulation(small_config)
    rng = np.random.default_rng(42)
    dirs = sim.initialize_directions(rng)
    assert len(dirs) == small_config.size


def test_initialize_directions_values(small_config):
    sim = Simulation(small_config)
    rng = np.random.default_rng(42)
    dirs = sim.initialize_directions(rng)
    assert set(dirs).issubset({-1, 1})


def test_initialize_directions_balanced(small_config):
    sim = Simulation(small_config)
    rng = np.random.default_rng(42)
    dirs = sim.initialize_directions(rng)
    n_pos = np.sum(dirs == 1)
    n_neg = np.sum(dirs == -1)
    assert n_pos == small_config.size // 2
    assert n_neg == small_config.size - small_config.size // 2


def test_initialize_directions_deterministic(small_config):
    sim = Simulation(small_config)
    rng1 = np.random.default_rng(42)
    rng2 = np.random.default_rng(42)
    dirs1 = sim.initialize_directions(rng1)
    dirs2 = sim.initialize_directions(rng2)
    np.testing.assert_array_equal(dirs1, dirs2)


def test_run_calls_stop_condition(small_config):
    small_config.check_stop_every = 0.1
    sim = Simulation(small_config)
    called = False
    def stop_if(config, model, data):
        nonlocal called
        called = True
        return False
    sim.run(stop_if=stop_if)
    assert called


def test_run_stop_condition_reduces_samples(small_config):
    small_config.check_stop_every = 0.1
    sim = Simulation(small_config)
    full_positions = sim.run()
    def stop_if(config, model, data):
        return True
    stopped_positions = sim.run(stop_if=stop_if)
    assert len(stopped_positions) < len(full_positions)

