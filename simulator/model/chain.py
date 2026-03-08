"""
Builds the hinged chain structure that encloses the particle assembly
using xml.etree.ElementTree.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from config.config import SimulationConfig


def add_chain_elements(parent: ET.Element, config: SimulationConfig) -> None:
    L = link_length(config)
    chain_offset = chain_offset_from_origin(L, config)

    # Root chain body
    capsule1 = ET.SubElement(parent, "body", name="capsule1",
                             pos=f"{chain_offset} {chain_offset} {config.link_radius}")
    ET.SubElement(capsule1, "freejoint")
    ET.SubElement(capsule1, "geom", **{"class": "link"}, fromto=f"0 0 0 {L} 0 0")
    ET.SubElement(capsule1, "site", name="start_anchor", size="0.001", pos="0 0 0")

    # Build recursive chain
    _horizontal_links(capsule1, config.links_per_side, L, 1, 1)


def link_length(config: SimulationConfig) -> float:
    particle_area = config.size * math.pi * pow(config.particle_radius, 2)
    return (1 / config.links_per_side) * math.sqrt(particle_area / config.tau) + (2 * config.link_radius / config.links_per_side)


def chain_offset_from_origin(L: float, config: SimulationConfig) -> float:
    particles_per_side = math.ceil(math.sqrt(config.size))
    rest_separation = 2 * config.particle_radius + config.particle_clearance
    return ((particles_per_side - 1) * rest_separation - config.links_per_side * L) / 2


def _horizontal_links(parent: ET.Element, n: int, L: float, count: int, total: int) -> None:
    if count == n:
        _vertical_links(parent, n, L, 0, total)
        return

    if count == 0:
        child = ET.SubElement(parent, "body", name=f"capsule{total + 1}",
                              pos=f"0 {-1 * L} 0")
        ET.SubElement(child, "joint", type="hinge", axis="0 0 1", pos="0 0 0")
        ET.SubElement(child, "geom", **{"class": "link"}, fromto=f"0 0 0 {L} 0 0")
        _horizontal_links(child, n, L, count + 1, total + 1)
    else:
        child = ET.SubElement(parent, "body", name=f"capsule{total + 1}",
                              pos=f"{L} 0 0")
        ET.SubElement(child, "joint", type="hinge", axis="0 0 1", pos="0 0 0")
        ET.SubElement(child, "geom", **{"class": "link"}, fromto=f"0 0 0 {L} 0 0")
        _horizontal_links(child, n, L, count + 1, total + 1)


def _vertical_links(parent: ET.Element, n: int, L: float, count: int, total: int) -> None:
    if count == n and L < 0:
        ET.SubElement(parent, "site", name="end_anchor", size="0.001", pos=f"0 {L} 0")
        return
    elif count == n and L > 0:
        _horizontal_links(parent, n, -L, 0, total)
        return

    if count == 0:
        child = ET.SubElement(parent, "body", name=f"capsule{total + 1}",
                              pos=f"{L} 0 0")
        ET.SubElement(child, "joint", type="hinge", axis="0 0 1", pos="0 0 0")
        ET.SubElement(child, "geom", **{"class": "link"}, fromto=f"0 0 0 0 {L} 0")
        _vertical_links(child, n, L, count + 1, total + 1)
    else:
        child = ET.SubElement(parent, "body", name=f"capsule{total + 1}",
                              pos=f"0 {L} 0")
        ET.SubElement(child, "joint", type="hinge", axis="0 0 1", pos="0 0 0")
        ET.SubElement(child, "geom", **{"class": "link"}, fromto=f"0 0 0 0 {L} 0")
        _vertical_links(child, n, L, count + 1, total + 1)
