from .xml_builder import create_xml
from .chain import create_chain_xml
from config.config import SimulationConfig

c = SimulationConfig()
print(create_chain_xml(c))
