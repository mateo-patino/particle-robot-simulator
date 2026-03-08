import numpy as np
from config.config import SimulationConfig
from simulator.simulation.engine import Simulation
from simulator.simulation.engine_parallel import Simulation as SimulationParallel


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


def test_parallel_initialize_directions_matches(small_config):
    rng1 = np.random.default_rng(42)
    rng2 = np.random.default_rng(42)
    dirs_orig = Simulation(small_config).initialize_directions(rng1)
    dirs_par = SimulationParallel(small_config).initialize_directions(rng2)
    np.testing.assert_array_equal(dirs_orig, dirs_par)
