import pytest
from config.config import SimulationConfig


@pytest.fixture
def default_config():
    return SimulationConfig()


@pytest.fixture
def small_config():
    return SimulationConfig(seed=42, size=4, sim_duration=0.5)
