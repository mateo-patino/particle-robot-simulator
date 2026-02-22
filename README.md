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

## Single-run Mode
To perform a single-run, use

```bash
python run.py --config example_config.json 
```
The command above runs one simulation using the parameter values specified in `example_config.json `. 

`--config` is the only required command-line argument of `run.py`, and it must be the path to a `JSON` file that contains the custom simulation parameters you wish to use. See here (TODO) for all simulation parameter values available.

### Multiple Runs
If you wish to run a simulation multiple times, you can pass a value into `--runs`
```bash
python run.py --config example_config.json --runs 20
```
The command above will run a simulation with the parameter values from `example_config.json` 20 times.

### Graphics
If you wish to open a window to visualize the simulation, you can enable the `--gui` flag (***NOTE: still in development***)
```bash
python run.py --config example_config.json --runs 20 --gui
```

### Results
The only form of data that the simulation records is the **position of the center of mass** of the particle robot during the simulation. When the simulation(s) end(s), a `results` folder will be created. Inside of it, a **timestamped** folder will contain the center-of-mass data alongside metadata of the simulation.

```bash
tree results/2026-01-01_00-00-00
results/2026-01-01_00-00-00
├── com_1.npy
├── config.json
└── metadata.json
```
### Labels
You can add a **label** to the subfolder in which results are saved via `--label`. The label will be placed before the timestamp in the subfolder name.
```bash
python run.py --config example_config.json --runs 20 --label "example"
```
The command above will run 20 simulations and save the results at `./results/example__2026-01-01_00-00-00`. The data will be saved in files `com_1.npy` through `com_20.npy`.

## Parameter Sweep Mode
To perform a parameter sweep, you must pass values for the `--sweep` and `--sweep_values` arguments.
```bash
python run.py --config example_config.json --sweep size --sweep_values 100 200 300 400 500 --runs 15 
```
The command above will sweep the simulation's size parameter (i.e. the total number of vribrating particles) over the values 100, 200, 300, 400, and 500. It will perform 15 runs **for each** value in `--sweep_values`.

### Sweep Results
The center-of-mass data collected in each simulation is saved to timestamped folders inside a `results` folder. The name of each data file will be prefixed by the **name of parameter swept** and the specific **value** of the parameter in that run, followed by the **run number**.

For example, the following command sweeps the high-frequency parameter (i.e the frequency at which the high-energy particles vibrate) across the values 8, 9, and 10 (Hz). 
```bash
python run.py --config example_config.json --sweep high_freq --sweep_values 8 9 10 --runs 2 --label "frequency-sweep"
```
The results of this set of simulations will be saved to a timestamped subfolder of th
```
tree results/frequency-sweep__2026-01-01_00-00-00
results/frequency-sweep__2026-01-01_00-00-00
├── high_freq_8_1.npy
├── high_freq_8_2.npy
├── high_freq_9_1.npy
├── high_freq_9_2.npy
├── high_freq_10_1.npy
├── high_freq_10_2.npy
├── base_config.json
└── sweep_metadata.json
```
