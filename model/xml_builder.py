"""
Docstring for xml_builder
"""

from chain import create_chain_xml
import math


class Particle:

    def __init__(self, config):

        self.initial_x = 0
        self.initial_y = 0
        self.initial_z = 0

        if config.geom_type == "sphere":
            self.initial_z = config.particle_radius
        elif config.geom_type == "cylinder":
            self.initial_z = config.cylinder_halflen


    def set_initial_xy(self, x, y):
        self.initial_x = x
        self.initial_y = y

    def get_position(self):
        return self.initial_x, self.initial_y, self.initial_z


def create_xml(config):
    
    particles = []
    for i in range(config.size):
        particles.append(Particle(config))
    arrange_initial_xy(particles, config)

    plane_length = config.plane_length_factor * math.sqrt(config.size) * 2 * config.particle_radius
    xml_string = f"""

        <mujoco model="{config.size}-particle robot">

                <visual>
                    <headlight ambient="0.5 0.5 0.5" diffuse="0.6 0.6 0.6" specular="0.4 0.4 0.4"/>
                </visual>

                <option gravity="0 0 -9.81" timestep="{config.timestep}" solver="{config.solver}" iterations="{config.iterations}" tolerance="{config.tolerance}"/>

                <default>
                    <default class="link">
                        <geom type="capsule" size="{config.link_radius}" mass="{config.link_mass}" rgba="0.2 0.8 0.2 1" solref="{config.link_solref[0]} {config.link_solref[1]}" solimp="{config.link_solimp[0]} {config.link_solimp[1]} {config.link_solimp[2]}"/>
                    </default>
                </default>

                <asset>
                    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1="0.8 0.8 0.8" rgb2="0.9 0.9 0.9" />
                    <material name="gridmat" texture="grid" texrepeat="20 20" texuniform="true" />
                </asset>

                <worldbody>
                    <camera name="top" pos="0 0 1" quat="0 1 0 0" mode="fixed"/>
                    <geom name="floor" type="plane" size="{plane_length} {plane_length} 0.1" material="gridmat" rgba="1 1 1 1"/>

                    {particle_xml_bodies(particles, config)}

                    {create_chain_xml(config)}

                </worldbody>

                <tendon>
                    <spatial name="loop_closer" rgba="1 0 0 1">
                        <site site="start_anchor"/>
                        <site site="end_anchor"/>
                    </spatial>
                </tendon>

                <equality>
                    <tendon tendon1="loop_closer" polycoef="0 1" solref="0.0001 1"/>
                </equality>

            </mujoco>

    """
    return xml_string


def arrange_initial_xy(particles, config):

    dist = 2 * config.particle_radius + config.particle_clearance
    side_length = int(math.sqrt(config.size))
    coordinates = []
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


def particle_xml_bodies(particles, config):

    particles_bodies = ""
    for i, particle in enumerate(particles):
        x, y, z = particle.get_position()
        geom_string = ""

        if config.geom_type == "sphere":
            geom_string = f"""<geom name="sphere{i}" type="sphere" size="{config.particle_radius}" mass="{config.particle_mass}" rgba="" solref="{config.particle_solref[0]} {config.particle_solref[1]}" solimp="{config.particle_solimp[0]} {config.particle_solimp[1]} {config.particle_solimp[2]}"/>"""
        elif config.geom_type == "cylinder":
            geom_string = f"""<geom name="cylinder{i}" type="cylinder" size="{config.particle_radius} {config.cylinder_halflen}" mass="{config.particle_mass}" rgba="" solref="{config.particle_solref[0]} {config.particle_solref[1]}" solimp="{config.particle_solimp[0]} {config.particle_solimp[1]} {config.particle_solimp[2]}"/>"""
        
        particles_bodies += f"""
                <body name="particle{i}" pos="{x} {y} {z}">
                    <freejoint/>
                    {geom_string}
                </body>
            """
    return particles_bodies
