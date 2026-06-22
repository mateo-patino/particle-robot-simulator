"""
Functions for the compliance experiments (gap traversals).

"""


from config.config import SimulationConfig
from simulator.simulation.engine import StopCondition
from simulator.model.chain import link_length, chain_offset_from_origin
from simulator.io.save import create_run_directory
from simulator.experiments.single_run import run_fast_save
from copy import deepcopy
import mujoco
import json
import os
import xml.etree.ElementTree as ET
import numpy as np
import logging

logger = logging.getLogger(__name__)



"""

Returns True if all Particles in the VPR have crossed the gap. "Crossing the gap" is defined as 
all Particles having an x-coordinate that is greater in magnitude than the x-coordinate of the gap.
The gap is always located a non-zero distance from the origin, so we say the VPR crossed the gap 
as soon as all Particles have gone past this distance.

Note that his function only works for gaps generated parallel to the x axis. Gaps generated
with different orientations (e.g. for VPRs where the heading isn't (1,0)) are unsuitable for
this function.

"""
def has_crossed_gap(config: SimulationConfig, model: mujoco.MjModel, data: mujoco.MjData):
    
    gap_x = get_gap_position(config)[0]

    geom_id = np.empty(config.size, dtype=np.int32)
    for i in range(config.size):
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{config.geom_type}{i}")
        if gid == -1:
            raise RuntimeError(f"Geom '{config.geom_type}{i}' not found in model")
        geom_id[i] = gid

    return np.all(np.abs(data.geom_xpos[geom_id, 0]) > np.abs(gap_x))


"""

Returns the initial side length of the VPR. The side length is measured from and to
the outermost edge of the chain.

"""
def get_side_length(config: SimulationConfig) -> float:
    return config.links_per_side*link_length(config) + 2*config.link_radius 


"""

Opens the file in config.env_path and returns the (x, y, z) location of the gap. This location
represents the geometric center of the gap (i.e. the midpoint between the narrowest end of both 
walls). 

Note that this function assumes the walls are defined using the fromto attribute, and that
the starting coordinates determine the location of the gap (e.g. the narrow end of each wall corresponds
to the starting endpoint in the fromto attribute).

"""

def get_gap_position(config: SimulationConfig) -> tuple[float, float, float]:

    if config.env_path is None: raise ValueError("SimulationConfig does not have env_path.")

    if not os.path.exists(config.env_path): raise FileNotFoundError(f"{config.env_path} does not exist.")

    root = ET.parse(config.env_path).getroot()
    if (len(root) != 2): raise ValueError(f"{config.env_path} is invalid. It must have only two elements representing the two walls of the gap.")

    top_wall, bottom_wall = root

    # Check if the wall geoms were wrapped inside of body tags
    if top_wall.tag == "body":
        top_wall = top_wall.find("geom")
        if top_wall is None: raise ValueError(f"{config.env_path} is invalid. Top wall does not contain a geom.")
    
    if bottom_wall.tag == "body":
        bottom_wall = bottom_wall.find("geom")
        if bottom_wall is None: raise ValueError(f"{config.env_path} is invalid. Bottom wall does not contain a geom.")
    
    top_fromto = top_wall.get("fromto")
    bot_fromto = bottom_wall.get("fromto")

    if top_fromto is None or bot_fromto is None:
        raise ValueError(f"{config.env_path} is invalid. Gap walls must define a fromto attribute. If using <body> tags, both walls must be wrapped in <body> blocks")

    top_x, top_y, top_z = [float(x) for x in top_fromto.split()[:3]]
    bot_x, bot_y, bot_z = [float(x) for x in top_fromto.split()[:3]] 

    gap_x = top_x if np.isclose(top_x, bot_x, rtol=0, atol=1e-12) else None
    if gap_x is None:  raise ValueError(f"{config.env_path} is invalid. The walls have different start x-coordinates.")

    gap_y = bot_y + (top_y - bot_y) / 2 # Midpoint between both walls

    gap_z = top_z if np.isclose(top_z, bot_z, rtol=0, atol=1e-12) else None
    if gap_z is None: raise ValueError(f"{config.env_path} is invalid. The walls have different start z-coordinates.")

    return gap_x, gap_y, gap_z

"""

Creates an XML description of a funnel-shaped gap. The size of the gap is 
gamma times the VPR's original side length. The gap is centered at y-midpoint
of the VPR. The walls stretch the VPR's vertical half-length.

The gap's position (gap_pos) is assumed to be in world coordinates.

TODO: this function assumes locomotion only occurs along the x axis. If the target
heading were set to a direction that isn't parallel to the x axis, the gap would
be incorrectly generated along x. A rotationally-invariant gap builder that takes
config.target_heading into account 

"""
def create_funnel_gap_xml(config: SimulationConfig, gamma: float) -> str:
    
    vpr_side_length = get_side_length(config)
    gap_width = vpr_side_length * gamma
    chain_offset = chain_offset_from_origin(link_length(config), config) 

    root = ET.Element("environment")
  
    # Start and end coordinates of walls
    sign = -1 if config.geom_type == "sphere" else 1
    gap_offset = 0 if config.geom_type == "sphere" else vpr_side_length
    gap_pos = vpr_side_length / 2
    start_x = sign * (gap_offset + np.abs(gap_pos))
    end_x = 0.25 * start_x
    
    top_start_y = (vpr_side_length / 2) + (gap_width / 2) - (np.abs(chain_offset) + config.link_radius) 
    top_end_y = top_start_y + (vpr_side_length / 2)

    bottom_start_y = (vpr_side_length / 2) - (gap_width / 2) - (np.abs(chain_offset) + config.link_radius) 
    bottom_end_y = bottom_start_y - (vpr_side_length / 2)

    height = config.link_radius

    # Top wall
    top_wall = ET.SubElement(root, "body", name="top_wall")
    ET.SubElement(top_wall, "geom", type="capsule", fromto=f"{start_x} {top_start_y} {height} {end_x} {top_end_y} {height}",
            size=f"{height}", rgba="0.8 0.2 0.2 1")

    # Bottom wall
    bottom_wall = ET.SubElement(root, "body", name="bottom_wall")
    ET.SubElement(bottom_wall, "geom", type="capsule", fromto=f"{start_x} {bottom_start_y} {height} {end_x} {bottom_end_y} {height}",
                  size=f"{height}", rgba="0.8 0.2 0.2 1")

    ET.indent(root)

    return ET.tostring(root, encoding="unicode")


"""

Simulates a VPR crossing a gap across multiple gamma values. The simulation stops early whenever
all Particles in the VPR have crossed the gap (see has_crossed_gap).

"""
def run_gap_traversal_fs(config: SimulationConfig, gammas: list[float], num_runs: int = 1, existing_runs: int = 0, 
                         stop_if: StopCondition | None = has_crossed_gap, label: str | None = None, argv: list[str] | None = None) -> str:

    out_dir = create_run_directory(label)
    env_dir_path = "config/env/"
    for gamma in gammas:
        # Runs with same gamma value can reuse the gap XML file
        env_path = create_funnel_gap_file(env_dir_path, config, gamma)
        
        # Run
        for run in range(1, num_runs + 1):
            new_config = deepcopy(config)
            results_path = os.path.join(out_dir, f"size{new_config.size}_gamma{gamma}_{existing_runs + run}.npy")
            
            # Set env_path and seed if any
            new_config.env_path = env_path
            if config.seed is not None:
                new_config.seed = config.seed + existing_runs + run - 1

            logger.info("Run %d started", run)
            run_fast_save(new_config, results_path, stop_if=stop_if)
            logger.info("Run %d complete", run)

    # run_fast_save does not save metadata, so do it manually
    with open(os.path.join(out_dir, "metadata.json"), "w") as file:
        cmd = " ".join(argv).strip() if argv is not None else "argv is None"
        metadata = {
            "env_dir_path": env_dir_path,
            "gammas": " ".join([str(g) for g in gammas]),
            "runs": num_runs,
            "existing_runs": existing_runs,
            "stop_if_callback": stop_if.__qualname__ if stop_if is not None else "None",
            "label": label,
            "cmd": cmd
        }
        json.dump(metadata, file, indent=4)
 
    with open(os.path.join(out_dir, "base_config.json"), "w") as file:
        json.dump(config.to_dict(), file, indent=4)

    return out_dir


"""

Creates a .xml file containing a description of a funnel gap. It returns the path to the .xml file.

"""
def create_funnel_gap_file(dir_path: str, config: SimulationConfig, gamma: float):
    os.makedirs(dir_path, exist_ok=True)
    file_path = os.path.join(dir_path, f"N{config.size}_gamma{gamma}.xml")

    with open(file_path, "w") as file:
        xml_str = create_funnel_gap_xml(config, gamma)
        file.write(xml_str)

    return file_path


