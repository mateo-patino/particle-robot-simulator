"""

Author(s): Mateo Patino, Hod Lipson
Creative Machines Laboratory, Columbia University

This script implements a class to encapsulate useful properties about the particles used
in the simulations. Not all of the attributes are used in the codebase.

"""

class Particle:

    def __init__(self, geomType, halfLen, radius, mass, rotorMass, solref=[], solimp=[]):
        self.geomType = geomType
        self.halfLen = halfLen
        self.radius = radius
        self.mass = mass
        self.rotorMass = rotorMass
        self.solref = solref
        self.solimp = solimp
        self.color = "0 0 0 0"
        self.initialPosition = "0 0 0"
        self.energyType = "N/A"

    
    def __str__(self):
        return f"{self.energyType}"
    

    def setPositionCoordinates(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z
    
    def getPosition(self):
        return self.x, self.y, self.z
    
    