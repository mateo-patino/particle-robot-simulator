# 🤖 Vibrating Particle Robot Simulator

A MuJoCo-based simulation framework for studying vibrating particle robots with gradient-based control.

Developed at the ***Creative Machines Laboratory, Columbia University***.

---

# 📦 Installation

### 1️⃣ Clone the repository

```bash
git clone https://github.com/mateo-patino/particle-robot-simulator.git
cd particle-robot-simulator
```

### 2️⃣ Create a virtual environment
Though this is not necessary, it is highly recommended to use a virtual environment to avoid compatibility issues between MuJoCo and other libraries.

```bash
python -m venv venv
source venv/bin/activate # Mac/Linux
venv\Scripts\activate # Windows
```

### 3️⃣ Install dependencies

```bash
pip install -r requirements.txt
```
All set!

# ⚙️ Running Simulations
There are **two** kinds of simulations you can perform: **single-run** and **one-dimensional parameter sweeps**. 

A single-run performs one full simulation with constant simulation parameters, and it is useful for quick exploration and testing. A one-dimensional parameter sweep performs multiple simulations with a specific parameter value modified on each one.

## Single-run mode
To perform a single-run, use

```bash
python run.py --config example_config.json 
```
The command above runs one simulation using the parameter values specified in `example_config.json `. `--config` is the only required command-line argument of `run.py`, and it must be the path to a `JSON` file that contains the custom simulation parameters you wish to use. See here (TODO) for all simulation parameter values available.

### Multiple Runs
If you wish to run a simulation multiple times, you can pass a value into `--runs`
```bash
python run.py --config example_config.json --runs 20
```
The command above will run a simulation with the parameter values from `example_config.json` 20 times.

### Results
The only form of data that the simulation records is the position of the center of mass of the particle robot during the simulation. When the simulation ends, a `results` folder will be created and the center of mass data will be saved there.

## Parameter sweep




