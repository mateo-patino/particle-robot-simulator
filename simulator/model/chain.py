"""
Docstring for chain
"""

import math


def create_chain_xml(config):
    L = link_length(config)
    chain_offset = chain_offset_from_origin(L, config)
    capsule_bodies = horizontal_links(config.links_per_side, L, 1, 1)

    chain = f"""
        <body name="capsule1" pos="{chain_offset} {chain_offset} {config.link_radius}">
                <freejoint/>
                <geom class="link" fromto="0 0 0 {L} 0 0"/> 
                <site name="start_anchor" size="0.001" pos="0 0 0"/>

                {capsule_bodies}

        </body>
    """
    return chain


def link_length(config):
    particle_area = config.size * math.pi * pow(config.particle_radius, 2)
    return (1 / config.links_per_side) * math.sqrt(particle_area / config.tau) + (2 * config.link_radius / config.links_per_side)


def chain_offset_from_origin(L, config):
    particles_per_side = math.ceil(math.sqrt(config.size))
    rest_separation = 2 * config.particle_radius + config.particle_clearance
    return ((particles_per_side - 1) * rest_separation - config.links_per_side * L) / 2


def horizontal_links(n, L, count, total):
    if count == n:
        return vertical_links(n, L, 0, total)

    if count == 0:
        return f"""
                <body name="capsule{total + 1}" pos="0 {-1 * L} 0">
                    <joint type="hinge" axis="0 0 1" pos="0 0 0"/>
                    <geom class="link" fromto="0 0 0 {L} 0 0"/>

                    {horizontal_links(n, L, count + 1, total + 1)}

                </body>
            """
    else:
        return f"""
            <body name="capsule{total + 1}" pos="{L} 0 0">
                <joint type="hinge" axis="0 0 1" pos="0 0 0"/>
                <geom class="link" fromto="0 0 0 {L} 0 0"/>

                {horizontal_links(n, L, count + 1, total + 1)}

            </body>
        """
    

def vertical_links(n, L, count, total):
    if count == n and L < 0:
        return f"""
            <site name="end_anchor" size="0.001" pos="0 {L} 0"/>
        """
    elif count == n and L > 0:
        return horizontal_links(n, -L, 0, total)

    if count == 0:
        return f"""
                <body name="capsule{total + 1}" pos="{L} 0 0">
                    <joint type="hinge" axis="0 0 1" pos="0 0 0"/>
                    <geom class="link" fromto="0 0 0 0 {L} 0"/>

                    {vertical_links(n, L, count + 1, total + 1)}

                </body>
            """
    else:
        return f"""
                <body name="capsule{total + 1}" pos="0 {L} 0">
                    <joint type="hinge" axis="0 0 1" pos="0 0 0"/>
                    <geom class="link" fromto="0 0 0 0 {L} 0"/>

                    {vertical_links(n, L, count + 1, total + 1)}

                </body>
            """
