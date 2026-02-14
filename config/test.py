from config import SimulationConfig

c = SimulationConfig.from_json("default.json")

print(c.rotor_mass)
print(c.to_dict())