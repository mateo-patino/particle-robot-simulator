"""

This script creates an XML file that describes a zigzag corridor made up of triangles.

Each triangle is parametrized by width and height, which can be passed into this program
as command line arguments in meters. The width of the corridor is parametrized by the 
gap-to-length ratio 'gamma' and the size of the VPR.


For intuition on what appropiate dimensions might be for the triangles, a 64-Particle VPR 
in its initial square form has a side length of ???.

"""

from pathlib import Path
from config.config import SimulationConfig
from simulator.experiments.gap_traversal import get_side_length
from simulator.model.chain import chain_offset_from_origin, link_length
import argparse
import xml.etree.ElementTree as ET
import numpy as np


def create_leg(parent: ET.Element, from_: tuple[float, float, float], 
               to: tuple[float, float, float], size: float):
    ET.SubElement(parent, 
                  "geom",
                  type="capsule",
                  fromto=f"{from_[0]} {from_[1]} {from_[2]} {to[0]} {to[1]} {to[2]}",
                  size=f"{size}",
                  rgba="0.8 0.2 0.2 1"
    )
    


def create_triangle(parent: ET.Element, width: float, height: float, from_: tuple[float, float, float]) :
    hw = width / 2
    # First leg (i.e. the one closest to the origin)
    leg1_tox = from_[0] + hw
    leg1_toy = from_[1] + height
    radius = from_[2] # should be equal to from_[2]

    create_leg(parent, 
               from_=from_, 
               to=(leg1_tox, leg1_toy, radius),
               size=radius
    )

    # Second leg
    create_leg(parent, 
               from_=(leg1_tox, leg1_toy, radius),
               to=(leg1_tox+hw, leg1_toy-height, radius),
               size=radius
    )



def create_zigzag_corridor_xml(width: float, height: float, count: int, gamma: float, config: SimulationConfig) -> str:

    vpr_side_length = get_side_length(config)
    gap_width = gamma * vpr_side_length
    corridor_x_offset = 1.1*vpr_side_length
    chain_offset = chain_offset_from_origin(link_length(config), config)

    root = ET.Element("environment")

    for i in range(count):
        # Top and bottom triangles share x and z coordinates
        from_x = corridor_x_offset + i*width
        from_z = config.link_radius

        # Top triangle
        top_triangle_body = ET.SubElement(root, "body", name=f"top_triangle{i}")
        top_from_y = (vpr_side_length / 2) + (gap_width / 2) - (np.abs(chain_offset) + config.link_radius)
        create_triangle(top_triangle_body, width, height, from_=(from_x, top_from_y, from_z))

        # Bottom tirangle
        bot_triangle_body = ET.SubElement(root, "body", name=f"bottom_triangle{i}")
        bot_from_y = (vpr_side_length / 2) - (gap_width / 2) - (np.abs(chain_offset) + config.link_radius)
        create_triangle(bot_triangle_body, width, height, from_=(from_x, bot_from_y, from_z))
    
    ET.indent(root)

    return ET.tostring(root, encoding="unicode")


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Create an XML of zigzag corridor")

    parser.add_argument(
        "--dir-path",
        type=str,
        default="config/env",
        help="Directory where XML file will be saved"

    )
    parser.add_argument(
        "--config",
        required=True,
        type=str,
        help="Path to simulation config"
    )

    parser.add_argument(
        "--width",
        type=float,
        default=0.35,
        help="Triangle width"
    )

    parser.add_argument(
        "--height",
        type=float,
        default=0.35,
        help="Triangle height"
    )

    parser.add_argument(
        "--count",
        type=int,
        default=5,
        help="How many triangles to generate"
    )


    parser.add_argument(
        "--gamma",
        type=float,
        default=1.0,
        help="Gap-to-length ratio"
    )
    
    args = parser.parse_args()
    
    dir_path = Path(args.dir_path)
    config = SimulationConfig.from_json(args.config)
    width = args.width
    height = args.height
    count = args.count
    gamma = args.gamma

    dir_path.mkdir(parents=True, exist_ok=True)

    file_path = dir_path / f"zigzag_N{config.size}_gamma{gamma}.xml"

    with open(file_path, "w") as out_file:
        xml_str = create_zigzag_corridor_xml(width, height, count, gamma, config)
        out_file.write(xml_str)
    
    print(f"{file_path}")
    
