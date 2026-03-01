"""
Builds MuJoCo XML model strings for particle robot simulations
using xml.etree.ElementTree for structured, validated output.
"""

from simulator.model.chain import add_chain_elements
from config.config import SimulationConfig
import math
import xml.etree.ElementTree as ET


class Particle:

    def __init__(self, config: SimulationConfig) -> None:

        self.initial_x: float = 0
        self.initial_y: float = 0
        self.initial_z: float = 0

        if config.geom_type == "sphere":
            self.initial_z = config.particle_radius
        elif config.geom_type == "cylinder":
            self.initial_z = config.cylinder_halflen

    def set_initial_xy(self, x: float, y: float) -> None:
        self.initial_x = x
        self.initial_y = y

    def get_position(self) -> tuple[float, float, float]:
        return self.initial_x, self.initial_y, self.initial_z


def create_xml(config: SimulationConfig) -> str:

    particles = []
    for i in range(config.size):
        particles.append(Particle(config))
    arrange_initial_xy(particles, config)

    plane_length = config.plane_length_factor * math.sqrt(config.size) * 2 * config.particle_radius

    root = ET.Element("mujoco", model=f"{config.size}-particle robot")

    # Visual
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "headlight", ambient="0.5 0.5 0.5", diffuse="0.6 0.6 0.6", specular="0.4 0.4 0.4")

    # Option
    ET.SubElement(root, "option",
                  gravity="0 0 -9.81",
                  timestep=str(config.timestep),
                  solver=config.solver,
                  iterations=str(config.iterations),
                  tolerance=str(config.tolerance))

    # Default
    default = ET.SubElement(root, "default")
    link_default = ET.SubElement(default, "default", **{"class": "link"})
    ET.SubElement(link_default, "geom",
                  type="capsule",
                  size=str(config.link_radius),
                  mass=str(config.link_mass),
                  rgba="0.2 0.8 0.2 1",
                  solref=f"{config.link_solref[0]} {config.link_solref[1]}",
                  solimp=f"{config.link_solimp[0]} {config.link_solimp[1]} {config.link_solimp[2]}")

    # Asset
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", name="grid", type="2d", builtin="checker",
                  width="512", height="512", rgb1="0.8 0.8 0.8", rgb2="0.9 0.9 0.9")
    ET.SubElement(asset, "material", name="gridmat", texture="grid",
                  texrepeat="20 20", texuniform="true")

    # Worldbody
    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(worldbody, "camera", name="top", pos="0 0 1", quat="0 1 0 0", mode="fixed")
    ET.SubElement(worldbody, "geom", name="floor", type="plane",
                  size=f"{plane_length} {plane_length} 0.1",
                  material="gridmat", rgba="1 1 1 1")

    # Particle bodies
    add_particle_bodies(worldbody, particles, config)

    # Chain
    add_chain_elements(worldbody, config)

    # Tendon
    tendon = ET.SubElement(root, "tendon")
    spatial = ET.SubElement(tendon, "spatial", name="loop_closer", rgba="1 0 0 1")
    ET.SubElement(spatial, "site", site="start_anchor")
    ET.SubElement(spatial, "site", site="end_anchor")

    # Equality
    equality = ET.SubElement(root, "equality")
    ET.SubElement(equality, "tendon", tendon1="loop_closer", polycoef="0 1", solref="0.0001 1")

    ET.indent(root)
    return ET.tostring(root, encoding="unicode")


def arrange_initial_xy(particles: list[Particle], config: SimulationConfig) -> list[tuple[float, float]]:

    dist = 2 * config.particle_radius + config.particle_clearance
    side_length = int(math.sqrt(config.size))
    coordinates: list[tuple[float, float]] = []
    count = 0

    for i in range(side_length):
        for j in range(side_length):
            x = i * dist
            y = j * dist
            particles[count].set_initial_xy(x, y)
            coordinates.append((x, y))
            count += 1

    if len(coordinates) != len(particles):
        missing = len(particles) - len(coordinates)
        x = side_length * dist
        for i in range(missing):
            y = i * dist
            if i < side_length:
                particles[count].set_initial_xy(x, y)
                coordinates.append((x, y))
                count += 1
            else:
                y = side_length * dist
                for j in range(side_length):
                    if count < len(particles):
                        x = j * dist
                        particles[count].set_initial_xy(x, y)
                        coordinates.append((x, y))
                        count += 1
                    else:
                        break
                break
    return coordinates


def add_particle_bodies(parent: ET.Element, particles: list[Particle], config: SimulationConfig) -> None:

    solref_str = f"{config.particle_solref[0]} {config.particle_solref[1]}"
    solimp_str = f"{config.particle_solimp[0]} {config.particle_solimp[1]} {config.particle_solimp[2]}"

    for i, particle in enumerate(particles):
        x, y, z = particle.get_position()

        body = ET.SubElement(parent, "body", name=f"particle{i}", pos=f"{x} {y} {z}")
        ET.SubElement(body, "freejoint")

        geom_attrs: dict[str, str] = {
            "mass": str(config.particle_mass),
            "rgba": "",
            "solref": solref_str,
            "solimp": solimp_str,
        }

        if config.geom_type == "sphere":
            geom_attrs["name"] = f"sphere{i}"
            geom_attrs["type"] = "sphere"
            geom_attrs["size"] = str(config.particle_radius)
        elif config.geom_type == "cylinder":
            geom_attrs["name"] = f"cylinder{i}"
            geom_attrs["type"] = "cylinder"
            geom_attrs["size"] = f"{config.particle_radius} {config.cylinder_halflen}"

        ET.SubElement(body, "geom", **geom_attrs)
