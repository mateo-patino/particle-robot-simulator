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
import argparse
import os
import xml.etree.ElementTree as ET

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Create an XML of zigzag corridor")

    parser.add_argument(
        "--dir-path",
        type=str,
        default="config/env"
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
    gamma = args.gamma

    dir_path.mkdir(parents=True, exist_ok=True)

    file_path = dir_path / f"zigzag_N{config.size}_gamma{gamma}.xml"


    
