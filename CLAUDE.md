# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MuJoCo-based simulation framework for vibrating particle robots with gradient-based control. Developed at the Creative Machines Laboratory, Columbia University. Python 3.10+ required (uses `int | None` union syntax).

## Commands

```bash
# Setup
python -m venv venv && source venv/bin/activate
pip install -r requirements.txt

# Single run
python run.py --config example_config.json

# Multiple runs
python run.py --config example_config.json --runs 20

# Parameter sweep
python run.py --config example_config.json --sweep size --sweep_values 100 200 300 --runs 15

# With label and GUI
python run.py --config example_config.json --runs 5 --label "test" --gui

# Skip saving results
python run.py --config example_config.json --no_save

# Record simulation to video
python run.py --config example_config.json --record --no_save

# Control log verbosity
python run.py --config example_config.json --no_save --log_level DEBUG

# Run tests
python -m pytest tests/ -v

# Run tests excluding slow integration tests
python -m pytest tests/ -v -m "not slow"

# Benchmark (correctness + speed comparison of both engines)
python benchmark.py
```

Dependencies: numpy, mujoco, imageio[ffmpeg] (video export), pytest (testing).

## Architecture

**Entry point**: `run.py` — CLI argument parsing, dispatches to single-run or sweep mode.

**config/config.py**: `SimulationConfig` dataclass with all simulation parameters. Loaded from JSON via `SimulationConfig.from_json(path)`. All values in SI units except `record_com_every`/`render_every` (simulation steps) and `run_control_every`/`log_sim_time_every` (simulation seconds). Validates `geom_type` (sphere/cylinder), `solver` (PGS/Newton/CG), and `tau` range on init.

**simulator/simulation/engine.py**: `Simulation` class — the core physics loop. Builds MuJoCo model from XML, runs the gradient-based control algorithm (partitions particles by dot product with target direction, assigns high/low frequencies), applies oscillating forces, and records center-of-mass trajectory. Force model: `F = 4π²Rm(freq²)cos(2πfreq*t + phase)`.

**simulator/model/xml_builder.py**: Generates MuJoCo XML using `xml.etree.ElementTree`. `create_xml()` builds the full model with particles arranged in a grid, floor plane, camera, tendons, and equality constraints. `Particle` class represents individual particle geometry.

**simulator/model/chain.py**: `add_chain_elements()` generates hinged chain structure that encloses the particle assembly using ElementTree. Chain tightness controlled by `tau` parameter.

**simulator/experiments/single_run.py**: `run_single(config, num_runs)` — runs N independent simulations, incrementing seed by 1 per run for reproducibility.

**simulator/experiments/sweep.py**: `run_1d_sweep(config, param, values, num_runs)` — sweeps one parameter across values, running multiple repetitions per value.

**simulator/io/save.py**: Saves results to timestamped directories under `results/`. COM trajectories stored as `.npy` files. Config and metadata saved as JSON.

## Data Flow

1. JSON config → `SimulationConfig` dataclass
2. Config → `create_xml()` → MuJoCo XML string → `MjModel`/`MjData`
3. Simulation loop runs control algorithm + physics stepping
4. Output: COM position arrays (shape: `[samples, 2]`) as `.npy` files in `results/`
