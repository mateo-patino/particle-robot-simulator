import mujoco
import pytest
from config.config import SimulationConfig
from simulator.model.xml_builder import create_xml, arrange_initial_xy, Particle


def test_xml_produces_valid_mujoco_model(small_config):
    xml = create_xml(small_config)
    model = mujoco.MjModel.from_xml_string(xml)
    assert model.ngeom > 0


def test_sphere_geom_count():
    config = SimulationConfig(size=9, geom_type="sphere")
    xml = create_xml(config)
    model = mujoco.MjModel.from_xml_string(xml)

    count = 0
    for i in range(9):
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"sphere{i}")
        if gid != -1:
            count += 1
    assert count == 9


def test_cylinder_geom_count():
    config = SimulationConfig(size=9, geom_type="cylinder")
    xml = create_xml(config)
    model = mujoco.MjModel.from_xml_string(xml)

    count = 0
    for i in range(9):
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"cylinder{i}")
        if gid != -1:
            count += 1
    assert count == 9


def test_arrange_perfect_square():
    config = SimulationConfig(size=9)
    particles = [Particle(config) for _ in range(9)]
    coords = arrange_initial_xy(particles, config)
    assert len(coords) == 9


def test_arrange_non_square():
    config = SimulationConfig(size=10)
    particles = [Particle(config) for _ in range(10)]
    coords = arrange_initial_xy(particles, config)
    assert len(coords) == 10


def test_xml_contains_camera_and_floor():
    config = SimulationConfig(size=4)
    xml = create_xml(config)
    model = mujoco.MjModel.from_xml_string(xml)
    floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    assert floor_id != -1
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "top")
    assert cam_id != -1


@pytest.mark.parametrize("size", [4, 16, 25, 64])
def test_various_sizes_produce_valid_xml(size):
    config = SimulationConfig(size=size)
    xml = create_xml(config)
    model = mujoco.MjModel.from_xml_string(xml)
    assert model.ngeom > size  # particles + chain geoms + floor
