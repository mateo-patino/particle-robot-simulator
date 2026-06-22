import numpy as np
import pytest
from config.config import SimulationConfig
from simulator.simulation import engine 


@pytest.mark.slow
def test_trajectory_shape():
    config = SimulationConfig(seed=42, size=4, sim_duration=0.5)
    traj = engine.Simulation(config).run()
    assert traj.ndim == 2
    assert traj.shape[1] == 2
    assert traj.shape[0] > 0


@pytest.mark.slow
def test_seed_reproducibility():
    config = SimulationConfig(seed=42, size=4, sim_duration=0.5)
    traj1 = engine.Simulation(config).run()
    traj2 = engine.Simulation(config).run()
    np.testing.assert_array_equal(traj1, traj2)


@pytest.mark.slow
def test_different_seeds_produce_different_results():
    config1 = SimulationConfig(seed=42, size=4, sim_duration=0.5)
    config2 = SimulationConfig(seed=99, size=4, sim_duration=0.5)
    traj1 = engine.Simulation(config1).run()
    traj2 = engine.Simulation(config2).run()
    assert not np.allclose(traj1, traj2)


@pytest.mark.slow
def test_trajectory_no_nan_or_inf():
    config = SimulationConfig(seed=42, size=4, sim_duration=0.5)
    traj = engine.Simulation(config).run()
    assert not np.any(np.isnan(traj)), "Trajectory contains NaN values"
    assert not np.any(np.isinf(traj)), "Trajectory contains Inf values"


@pytest.mark.slow
def test_trajectory_sliced_to_actual_samples():
    """COM_POSITION should be sliced to actual samples_taken, no uninitialized rows."""
    config = SimulationConfig(seed=42, size=4, sim_duration=0.01, timestep=3e-05, record_com_every=7)
    traj = engine.Simulation(config).run()
    assert not np.any(np.isnan(traj))
    assert not np.any(np.isinf(traj))
    # All values should be reasonable (particle positions near origin)
    assert np.all(np.abs(traj) < 10)


@pytest.mark.slow
def test_cylinder_geom_type_runs():
    config = SimulationConfig(seed=42, size=4, sim_duration=0.1, geom_type="cylinder")
    traj = engine.Simulation(config).run()
    assert traj.ndim == 2
    assert traj.shape[1] == 2
    assert traj.shape[0] > 0
