"""
Docstring for config
"""

from dataclasses import dataclass
import json

@dataclass
class SimulationConfig:

    # Particle parameters
    particle_radius: float = 0.019
    cylinder_halflen: float = 0.019
    particle_mass: float = 0.26
    rotor_mass: float = 0.056
    geom_type: str = "sphere"
    particle_solref: list = [0.003, 1]
    particle_solimp: list = [0.95, 0.95, 0.005]
    particle_clearance: float = 1e-05

    # Gradient parameters
    high_freq: float = 10
    low_freq: float = 3
    phase_noise_std: float = 0.05

    # Simulation parameters
    size: int = 64
    sim_duration: float = 15
    timestep: float = 7e-05
    solver: str = "Newton"
    iterations: int = 100
    tolerance: float = 1e-12

    # Chain parameters
    links_per_side: int = 5
    tau: float = 0.7
    link_radius: float = 0.019
    link_mass: float = 0.0155
    link_solref: list = [0.003, 1]
    link_solimp: list = [0.9, 0.9, 0.005]

    # Control parameters
    target_direction: tuple = (1, 0)
    run_control_every: float = 0.05

    @classmethod
    def from_json(cls, path):
        with open(path) as f:
            data = json.load(f)
        return cls(**data)
