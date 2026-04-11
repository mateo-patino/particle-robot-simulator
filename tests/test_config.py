import json
import pytest
from math import pi
from config.config import SimulationConfig


def test_default_construction():
    config = SimulationConfig()
    assert config.size == 64
    assert config.geom_type == "sphere"
    assert config.solver == "Newton"
    assert config.seed is None
    assert config.gui is False
    assert config.record is False
    assert config.video_path is None


def test_from_json_roundtrip(tmp_path):
    original = SimulationConfig(size=16, sim_duration=5.0, seed=123)
    path = tmp_path / "config.json"
    path.write_text(json.dumps(original.to_dict()))

    loaded = SimulationConfig.from_json(str(path))
    assert loaded.size == 16
    assert loaded.sim_duration == 5.0
    assert loaded.seed == 123


def test_to_dict():
    config = SimulationConfig(size=32)
    d = config.to_dict()
    assert isinstance(d, dict)
    assert d["size"] == 32
    assert "geom_type" in d


def test_invalid_geom_type():
    with pytest.raises(ValueError, match="not a valid particle geom type"):
        SimulationConfig(geom_type="box")


def test_invalid_solver():
    with pytest.raises(ValueError, match="not a valid solver"):
        SimulationConfig(solver="Euler")


def test_invalid_tau_negative():
    with pytest.raises(ValueError, match="tau"):
        SimulationConfig(tau=-0.1)


def test_invalid_tau_too_large():
    with pytest.raises(ValueError, match="tau"):
        SimulationConfig(tau=pi)


def test_target_direction_coerced_to_tuple():
    config = SimulationConfig(target_direction=[1, 1])
    assert isinstance(config.target_direction, tuple)
    assert config.target_direction == (1, 1)


def test_valid_geom_types():
    for geom in ("sphere", "cylinder"):
        config = SimulationConfig(geom_type=geom)
        assert config.geom_type == geom


def test_valid_solvers():
    for solver in ("PGS", "Newton", "CG"):
        config = SimulationConfig(solver=solver)
        assert config.solver == solver


def test_from_json_minimal(tmp_path):
    """Loading a JSON with only a few fields should use defaults for the rest."""
    path = tmp_path / "minimal.json"
    path.write_text(json.dumps({"size": 16}))
    config = SimulationConfig.from_json(str(path))
    assert config.size == 16
    assert config.sim_duration == 15  # default
    assert config.record is False  # default for new field


def test_from_json_example_config():
    """The shipped example_config.json should load without error."""
    config = SimulationConfig.from_json("example_config.json")
    assert config.size == 64


def test_from_json_speed_vs_frequency_config():
    """The speed_vs_frequency config should load after fixing the stale 'headless' key."""
    config = SimulationConfig.from_json("config/paper_experiments/speed_vs_frequency_sphere.json")
    assert config.gui is False


def test_from_json_default_config():
    """The default.json config should load without error."""
    config = SimulationConfig.from_json("config/default.json")
    assert config.size == 100
