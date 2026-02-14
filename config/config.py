"""

Configuration parameters for the simulation. 
Every variable is in S.I. units. "refresh_every," "record_com_every" are in units of simulation steps. 
"run_control_every" and "log_sim_time" are in simulation seconds.

"""

from dataclasses import dataclass
from math import pi
import json

@dataclass
class SimulationConfig:

    seed: int | None = None

    # Graphics parameters
    headless: bool = False
    render_every: int = 100
    plane_length_factor: float = 10

    # Data collection parameters
    record_com_every: int = 25
    log_sim_time_every: float = 5

    # Particle parameters
    particle_radius: float = 0.019
    cylinder_halflen: float = 0.019
    particle_mass: float = 0.26
    rotor_mass: float = 0.056
    geom_type: str = "sphere"
    particle_solref: tuple = (0.003, 1)
    particle_solimp: tuple = (0.95, 0.95, 0.005)
    particle_clearance: float = 1e-05

    # Gradient parameters
    high_freq: float = 10
    low_freq: float = 3
    phase_noise_std: float = 0.05
    noise_phase_every: float = 0.1

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
    link_solref: tuple = (0.003, 1)
    link_solimp: tuple = (0.9, 0.9, 0.005)

    # Control parameters
    target_direction: tuple = (1, 0)
    run_control_every: float = 0.05

    @classmethod
    def from_json(cls, path):
        with open(path) as f:
            data = json.load(f)
        return cls(**data)
    
    def to_dict(self):
        return dict(self.__dict__)
    
    def __post_init__(self):
        if self.geom_type not in ["sphere", "cylinder"]:
            raise ValueError(f"'{self.geom_type}' is not a valid particle geom type. Use 'sphere' or 'cylinder'.")
        
        if self.solver not in ["PGS", "Newton", "CG"]:
            raise ValueError(f"'{self.solver}' is not a valid solver in MuJoCo. Use 'PGS' or 'Newton', or 'CG'.")
        
        if self.tau < 0 or self.tau > pi / 4:
            raise ValueError("Chain tightness ratio (tau) must be positive and less than pi / 4 (0.785 approx.).")

