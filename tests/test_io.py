import json
import numpy as np
import pytest
from config.config import SimulationConfig
from simulator.io.save import create_run_directory, save_single_run, save_1d_sweep
import os


def test_create_run_directory(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    path = create_run_directory()
    assert os.path.isdir(path)
    assert "results" in path


def test_create_run_directory_with_label(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    path = create_run_directory(label="test_label")
    assert os.path.isdir(path)
    assert "test_label" in path


def test_save_single_run(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    config = SimulationConfig(size=4)
    data = {
        1: np.zeros((10, 2)),
        2: np.ones((10, 2)),
    }
    path = save_single_run(config, data, label="test")
    assert os.path.isdir(path)
    assert os.path.exists(os.path.join(path, "config.json"))
    assert os.path.exists(os.path.join(path, "metadata.json"))
    assert os.path.exists(os.path.join(path, "com_1.npy"))
    assert os.path.exists(os.path.join(path, "com_2.npy"))

    with open(os.path.join(path, "metadata.json")) as f:
        meta = json.load(f)
    assert meta["runs"] == 2


def test_save_1d_sweep(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    config = SimulationConfig(size=4)
    results = {
        "10": {1: np.zeros((5, 2))},
        "20": {1: np.ones((5, 2))},
    }
    path = save_1d_sweep(config, results, "size", label="sweep_test")
    assert os.path.isdir(path)
    assert os.path.exists(os.path.join(path, "base_config.json"))
    assert os.path.exists(os.path.join(path, "sweep_metadata.json"))
    assert os.path.exists(os.path.join(path, "size_10_1.npy"))
    assert os.path.exists(os.path.join(path, "size_20_1.npy"))
