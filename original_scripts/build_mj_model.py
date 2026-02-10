"""

Author(s): Mateo Patino, Hod Lipson
Creative Machines Laboratory, Columbia University

This script builds the primary XML string for the MuJoCo model. It calls build_chain.py to build the 
XML string for the chain.

"""

from Particle import Particle
from math import sqrt
import build_chain
import lowsizeparams as params


# returns the XML for two box geoms
def insertBoxObstacles():

    halfWidth = params.obstacle["width"] / 2
    halfDepth = params.obstacle["depth"] / 2
    halfHeight = params.obstacle["height"] / 2
    x = params.obstacle["xpos"]
    y = params.obstacle["ypos"]
    z = params.obstacle["zpos"]

    return f""" 

        <geom type="box" size="{halfWidth} {halfDepth} {halfHeight}" pos="{x} {y} {z}" rgba="0 0 0 1"/>
        <geom type="box" size="{halfWidth} {halfDepth} {halfHeight}" pos="{x} {y + params.obstacle["separation"]} {z}" rgba="0 0 0 1"/>

    """


# returns the initial height at which a Particle should be set
def initialHeight(particle):
    if particle.geomType == "cylinder":
        print("Yes, returned cylinder halflen")
        return particle.halfLen
    return particle.radius


# generate the XML string for a simulation with cylindrical particles and a chain made up of capsules
def _constructCylinderXMLWithChain_(particles, algorithm):

    particleBodies = ""

    for i, p in enumerate(particles):

        x, y, z = p.getPosition()
        solref = p.solref
        solimp = p.solimp

        particleBodies += f"""

            <body name="particle{i}" pos="{x} {y} {z}">
                <freejoint/>
                <geom name="cylinder{i}" type="cylinder" size="{p.radius} {p.halfLen}" mass="{p.mass}" rgba="" solref="{solref[0]} {solref[1]}" solimp="{solimp[0]} {solimp[1]} {solimp[2]}"/>
            </body>

        """

    planeLen = params.planeLengthFactor * len(particles) * 2 * params.particle["radius"]
    cap = params.capsule
    csolref = cap["solref"]
    csolimp = cap["solimp"]

    xml = f"""

        <mujoco model="{params.N}-particle robot">

            <size nstack="500000000"/>

            <visual>
                <headlight ambient="0.5 0.5 0.5" diffuse="0.6 0.6 0.6" specular="0.4 0.4 0.4"/>
            </visual>
            
            <option gravity="0 0 -9.81" timestep="{algorithm["timestep"]}" solver="{algorithm["solver"]}" iterations="{algorithm["iterations"]}" tolerance="{algorithm["tolerance"]}"/>

            <default>
                <default class="link">
                    <geom type="capsule" size="{cap["radius"]}" mass="{cap["mass"]}" rgba="0.2 0.8 0.2 1" solref="{csolref[0]} {csolref[1]}" solimp="{csolimp[0]} {csolimp[1]} {csolimp[2]}"/>
                </default>
            </default>

            <asset>
                <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1="0.8 0.8 0.8" rgb2="0.9 0.9 0.9" />
                <material name="gridmat" texture="grid" texrepeat="20 20" texuniform="true" />
            </asset>

            <worldbody>
                <camera name="top" pos="0 0 1" quat="0 1 0 0" mode="fixed"/>
                <geom name="floor" type="plane" size="{planeLen} {planeLen} 0.1" material="gridmat" rgba="1 1 1 1"/>
            
                {particleBodies}

                {build_chain._createChainXML_(params.chain["linksPerSide"], params.chain["tau"])}

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

    return xml


# generate the XML string for a simulation with SPHERICAL particles and a chain made up of capsules
def _constructSphereXMLWithChain_(particles, algorithm):

    particleBodies = ""

    for i, p in enumerate(particles):

        x, y, z = p.getPosition()
        solref = p.solref
        solimp = p.solimp

        particleBodies += f"""

            <body name="particle{i}" pos="{x} {y} {z}">
                <freejoint/>
                <geom name="sphere{i}" type="sphere" size="{p.radius}" mass="{p.mass}" rgba="" solref="{solref[0]} {solref[1]}" solimp="{solimp[0]} {solimp[1]} {solimp[2]}"/>
            </body>

        """

    planeLen = params.planeLengthFactor * len(particles) * 2 * params.particle["radius"]
    cap = params.capsule
    csolref = cap["solref"]
    csolimp = cap["solimp"]

    # add two box geoms as obstacles if requested
    obstacle = insertBoxObstacles() if params.insertObstacles else ""

    xml = f"""

        <mujoco model="{params.N}-particle robot">

            <size nstack="500000000"/>

            <visual>
                <headlight ambient="0.5 0.5 0.5" diffuse="0.6 0.6 0.6" specular="0.4 0.4 0.4"/>
            </visual>
            
            <option gravity="0 0 -9.81" timestep="{algorithm["timestep"]}" solver="{algorithm["solver"]}" iterations="{algorithm["iterations"]}" tolerance="{algorithm["tolerance"]}"/>

            <default>
                <default class="link">
                    <geom type="capsule" size="{cap["radius"]}" mass="{cap["mass"]}" rgba="0.2 0.8 0.2 1" solref="{csolref[0]} {csolref[1]}" solimp="{csolimp[0]} {csolimp[1]} {csolimp[2]}"/>
                </default>
            </default>

            <asset>
                <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1="0.8 0.8 0.8" rgb2="0.9 0.9 0.9" />
                <material name="gridmat" texture="grid" texrepeat="20 20" texuniform="true" />
            </asset>

            <worldbody>
                <camera name="top" pos="0 0 1" quat="0 1 0 0" mode="fixed"/>
                <geom name="floor" type="plane" size="{planeLen} {planeLen} 0.1" material="gridmat" rgba="1 1 1 1"/>

                {obstacle}
            
                {particleBodies}

                {build_chain._createChainXML_(params.chain["linksPerSide"], params.chain["tau"])}

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

    return xml



""" Given an array of Particles, the algorithm below modifies their position relative to the origin in order to
    create a square arrangement of particles. It returns an array of the new positions of all particles"""
def arrangeInitialPosition(particles, clearance):

    # separation between the centroids of each particle
    dist = 2 * params.particle["radius"] + clearance

    sideLength = int(sqrt(len(particles)))
    coordinates = []

    # start at x = 0 and y = 1 and calculate all coordinates starting from the bottom left corner
    count = 0
    for i in range(sideLength):
        for j in range(sideLength):
            x = i * dist
            y = j * dist
            particles[count].setPositionCoordinates(x=x, y=y, z=initialHeight(particles[count]))
            coordinates.append((x, y))
            count += 1
    
    # complete the rest of the arrangement
    if len(coordinates) != len(particles):
        missing = len(particles) - len(coordinates)

        # add a column to the right by holding x constant
        x = sideLength * dist
        for i in range(missing):
            y = i * dist
            if i < sideLength:
                particles[count].setPositionCoordinates(x=x, y=y, z=initialHeight(particles[count]))
                coordinates.append((x, y))
                count += 1
            else:
                # add a row to the top by now holding y constant
                y = sideLength * dist
                for j in range(sideLength):
                    if count < len(particles): # stop once we've added all the points
                        x = j * dist
                        particles[count].setPositionCoordinates(x=x, y=y, z=initialHeight(particles[count]))
                        coordinates.append((x, y))
                        count += 1
                    else:
                        break
                break
    return coordinates


# returns the XML string for the MuJoCo model
def xml(N):

    # generate particles
    particles = []
    for i in range(N):
        particles.append(Particle(**params.particle)) 
    
    # arrange the particles position
    arrangeInitialPosition(particles, params.clearance)

    # build the XML model string and return it
    if params.particle["geomType"] == "cylinder":
        return _constructCylinderXMLWithChain_(particles, params.algorithm)
    
    return _constructSphereXMLWithChain_(particles, params.algorithm)
    
