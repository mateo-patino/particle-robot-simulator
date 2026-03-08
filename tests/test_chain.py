from config.config import SimulationConfig
from simulator.model.chain import link_length, chain_offset_from_origin


def test_link_length_positive(default_config):
    L = link_length(default_config)
    assert isinstance(L, float)
    assert L > 0


def test_link_length_scales_with_size():
    small = SimulationConfig(size=4)
    large = SimulationConfig(size=64)
    assert link_length(large) > link_length(small)


def test_chain_offset_is_float(default_config):
    L = link_length(default_config)
    offset = chain_offset_from_origin(L, default_config)
    assert isinstance(offset, float)


def test_link_length_scales_with_links_per_side():
    few = SimulationConfig(links_per_side=3)
    many = SimulationConfig(links_per_side=10)
    assert link_length(few) > link_length(many)
