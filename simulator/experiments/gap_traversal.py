"""
Functions for the compliance experiments (gap traversals).

"""


from config.config import SimulationConfig
from simulator.model.chain import link_length
from simulator.simulation.engine import Simulation
from simulator.io.save import create_run_directory
from simulator.experiments.single_run import run_fast_save
import json
import os
import xml.etree.ElementTree as ET
import numpy as np

"""

Returns the initial side length of the VPR. The side length is measured from and to
the outermost edge of the chain.

"""
def get_side_length(config: SimulationConfig) -> float:
    return config.links_per_side*link_length(config) + 2*config.link_radius

"""

Creates an XML description of a funnel-shaped gap. The size of the gap
is gamma * vpr_side_length, where vpr_side_length is expected to be
the original side length of the VPR. The walls are 45 degrees away from 
the x-axis.

The gap's position (gap_pos) is assumed to be in world coordinates.

"""
def create_funnel_gap_xml(vpr_side_length: float, gamma: float, gap_pos: float) -> str:
    
    gap_width = vpr_side_length * gamma

    root = ET.Element("environment")
  
    # Start and end coordinates of walls
    wall_length = vpr_side_length / (2 * np.sin(np.pi / 4))
    start_x = -1 * np.abs(gap_pos)
    end_x = gap_pos - (vpr_side_length / 2)

    start_y = gap_width / 2 
    end_y = start_y + (vpr_side_length / 2)

    # Top wall
    top_wall = ET.SubElement(root, "body", name="top_wall")
    ET.SubElement(top_wall, "geom", type="capsule", fromto=f"{start_x} {start_y} 0.05 {end_x} {end_y} 0.05",
            size="0.01", rgba="0.8 0.2 0.2 1")

    # Bottom wall
    bottom_wall = ET.SubElement(root, "body", name="bottom_wall")
    ET.SubElement(bottom_wall, "geom", type="capsule", fromto=f"{start_x} {-1*start_y} 0.05 {end_x} {-1*end_y} 0.05",
                  size="0.01", rgba="0.8 0.2 0.2 1")

    ET.indent(root)

    return ET.tostring(root, encoding="unicode")


"""

Sets the env_path config variable of 'config' and runs it 'num_runs' times. 'env_path' must be the
path to an XML file describing a gap of ratio 'gamma'.

"""
def run_gap_traversal_fs(config: SimulationConfig, env_path: str, gamma: float, num_runs: int = 1, existing_runs: int = 0, 
                         label: str | None = None, argv: list[str] | None = None) -> str:

    if env_path is not None:
        if os.path.exists(env_path):
            config.env_path = env_path
        else:        
           raise FileNotFoundError(f"Environment path {config.env_path} was not found.")
    
    out_dir = create_run_directory(label)

    for run in range(1, num_runs + 1):
        out_path = os.path.join(out_dir, f"size{config.size}_gamma{gamma}_{existing_runs + run}.npy")
        logger.info("Run %d started", run)
        run_fast_save(config, out_path)
        logger.info("Run %d complete", run)

    # run_fast_save does not save metadata, so do it manually
    with open(os.join.path(out_dir, "metadata.json"), "w") as file:
        cmd = " ".join(argv).strip() if argv is not None else "argv is None"
        metadata = {
            "env_path": env_path,
            "gamma": gamma,
            "runs": num_runs,
            "existing_runs": existing_runs,
            "label": label,
            "cmd": cmd
        }
        json.dump(metadata, file, indent=4)
 
    with open(os.path.join(out_dir, "config.json"), "w") as file:
        json.dump(config.to_dict(), file, indent=4)

    return dir_path

