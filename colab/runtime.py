"""

Author(s): Mateo Patino, Hod Lipson
Creative Machines Laboratory, Columbia University

This script is identical to the runtime environment in Google Colab that I am using to run simulations and 
collect data on SPEED VS. SIZE for the paper. This script is self-contained and has very simple scopes to imitate exactly the
control flow and code in Google Colab and ensure both produce the same results. 

Maintaining the integrity of this code is critical because this script is the main source of experimental speed-vs-size data
for this project alongside Colab.

"""


import math
import mujoco
import numpy as np
import time
import os

"""PARAMETERS"""
# PLANE LENGTH FACTOR
planeLengthFactor = 4

# CYLINDER/SPHERICAL PARAMETERS
radius = 0.019
halfLen = radius
mass = 0.26
solref = [0.003, 1] # [r, d] = [how quickly the constraint error is corrected, damping; how inelastic the collision is] [0.003, 1]
solimp = [0.95, 0.95, 0.005] # [m, h, w] = [rate at which stiffness grows with penetration, min. impendance when constraint is barely violated, width of the interval around the contact margin where the transition in stiffness occurs] [0.95, 0.95, 0.005]
clearance = 0.00001
rotorMass = 4 * 0.014
geomType = "sphere" # 'sphere' or 'cylinder'

# GRADIENT PARAMETERS
N = 100 #14
highFreq = 10 # 15
lowFreq = 3 #20
phase = 2 * np.pi
forceNoiseSTD = 0.01 #0.1
phaseNoiseSTD = 0.05 #0.05

# ALGORITHMS PARAMETERS
SIM_DURATION = 20
timestep = 7e-05
solver = "Newton"
iterations = 100
tolerance = 1e-12

# CAPSULE PARAMETERS
capsuleRadius = radius
capsuleMass = 0.0155
offsetFromOrigin = 0.033
cap_solref = [0.003, 1] # [r, d] = [how quickly the constraint error is corrected, damping; how inelastic the collision is] [0.003, 1]
cap_solimp = [0.9, 0.9, 0.005] # [m, h, w] = [rate at which stiffness grows with penetration, min. impendance when constraint is barely violated, width of the interval around the contact margin where the transition in stiffness occurs] [0.9, 0.9, 0.005]

# CHAIN PARAMETERS
linksPerSide = 5 #5
tau = 0.7 # tightness ratio 0 < tau <= pi/4 (higher = tighter, lower = looser). Bounded above by pi/4 (~0.785); beyond that interpentration occurs

# CONTROL ALGORITHM
targetDirection = np.array([1, 0])
RUN_ALG_EVERY = 0.05 # 0.01
margin = 2

# buffers to report in the parameters file
linkLengthValue = None
runNumber = None

""" PARTICLE CLASS """
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
    
    
""" MJ MODEL CONSTRUCTOR """
def initializeDirections(N):
    half = N // 2
    signs = [1] * half + [-1] * (N - half)
    np.random.shuffle(signs)
    return np.array(signs)


def linkLength(tau, particleArea, linksPerSide, capsuleRadius):
    L = (1 / linksPerSide) * math.sqrt(particleArea / tau) + (2 * capsuleRadius / linksPerSide)
    global linkLengthValue
    linkLengthValue = L
    return L


def chainOffsetFromOrigin(particlesPerSide, restSeparation, linksPerSide, L):
    return ((particlesPerSide - 1) * restSeparation - linksPerSide * L) / 2


def initialHeight(particle):
    if particle.geomType == "cylinder":
        return particle.halfLen
    return particle.radius


def createChainXML(linksPerSide, tau):
    particleArea = N * np.pi * pow(radius, 2)
    L = linkLength(tau, particleArea, linksPerSide, capsuleRadius)

    capsuleBodies = horizontalLinks(linksPerSide, L, 1, 1)

    particlesPerSide = math.ceil(math.sqrt(N))
    restSeparation = 2 * radius + clearance
    offset = chainOffsetFromOrigin(particlesPerSide, restSeparation, linksPerSide, L)

    chain = f"""
        <body name="capsule1" pos="{offset} {offset} {capsuleRadius}">
                <freejoint/>
                <geom class="link" fromto="0 0 0 {L} 0 0"/> 
                <site name="start_anchor" size="0.001" pos="0 0 0"/>

                {capsuleBodies}

            </body>
    """

    return chain


def horizontalLinks(n, L, count, total):
    if count == n:
        return verticalLinks(n, L, 0, total)

    if count == 0:
        return f"""
                <body name="capsule{total + 1}" pos="0 {-1 * L} 0">
                    <joint type="hinge" axis="0 0 1" pos="0 0 0"/>
                    <geom class="link" fromto="0 0 0 {L} 0 0"/>

                    {horizontalLinks(n, L, count + 1, total + 1)}

                </body>
            """
    else:
        return f"""
            <body name="capsule{total + 1}" pos="{L} 0 0">
                <joint type="hinge" axis="0 0 1" pos="0 0 0"/>
                <geom class="link" fromto="0 0 0 {L} 0 0"/>

                {horizontalLinks(n, L, count + 1, total + 1)}

            </body>
        """


def verticalLinks(n, L, count, total):
    if count == n and L < 0:
        return f"""
            <site name="end_anchor" size="0.001" pos="0 {L} 0"/>
        """
    elif count == n and L > 0:
        return horizontalLinks(n, -L, 0, total)

    if count == 0:
        return f"""
                <body name="capsule{total + 1}" pos="{L} 0 0">
                    <joint type="hinge" axis="0 0 1" pos="0 0 0"/>
                    <geom class="link" fromto="0 0 0 0 {L} 0"/>

                    {verticalLinks(n, L, count + 1, total + 1)}

                </body>
            """
    else:
        return f"""
                <body name="capsule{total + 1}" pos="0 {L} 0">
                    <joint type="hinge" axis="0 0 1" pos="0 0 0"/>
                    <geom class="link" fromto="0 0 0 0 {L} 0"/>

                    {verticalLinks(n, L, count + 1, total + 1)}

                </body>
            """


def constructSphereXMLWithChain(particles):

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

    planeLen = planeLengthFactor * len(particles) * 2 * radius

    xml = f"""

        <mujoco model="{N}-particle robot">

                <size nstack="500000000"/>

                <visual>
                    <headlight ambient="0.5 0.5 0.5" diffuse="0.6 0.6 0.6" specular="0.4 0.4 0.4"/>
                </visual>

                <option gravity="0 0 -9.81" timestep="{timestep}" solver="{solver}" iterations="{iterations}" tolerance="{tolerance}"/>

                <default>
                    <default class="link">
                        <geom type="capsule" size="{capsuleRadius}" mass="{capsuleMass}" rgba="0.2 0.8 0.2 1" solref="{cap_solref[0]} {cap_solref[1]}" solimp="{cap_solimp[0]} {cap_solimp[1]} {cap_solimp[2]}"/>
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

                    {createChainXML(linksPerSide, tau)}

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


def constructCylinderXMLWithChain(particles):

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

    planeLen = planeLengthFactor * len(particles) * 2 * radius

    xml = f"""

        <mujoco model="{N}-particle robot">

                <size nstack="500000000"/>

                <visual>
                    <headlight ambient="0.5 0.5 0.5" diffuse="0.6 0.6 0.6" specular="0.4 0.4 0.4"/>
                </visual>

                <option gravity="0 0 -9.81" timestep="{timestep}" solver="{solver}" iterations="{iterations}" tolerance="{tolerance}"/>

                <default>
                    <default class="link">
                        <geom type="capsule" size="{capsuleRadius}" mass="{capsuleMass}" rgba="0.2 0.8 0.2 1" solref="{cap_solref[0]} {cap_solref[1]}" solimp="{cap_solimp[0]} {cap_solimp[1]} {cap_solimp[2]}"/>
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

                    {createChainXML(linksPerSide, tau)}

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


def arrangeInitialPosition(particles, clearance):

    dist = 2 * radius + clearance
    sideLength = int(math.sqrt(len(particles)))
    coordinates = []
    count = 0

    for i in range(sideLength):
        for j in range(sideLength):
            x = i * dist
            y = j * dist
            particles[count].setPositionCoordinates(x=x, y=y, z=initialHeight(particles[count]))
            coordinates.append((x, y))
            count += 1

    if len(coordinates) != len(particles):
        missing = len(particles) - len(coordinates)
        x = sideLength * dist
        for i in range(missing):
            y = i * dist
            if i < sideLength:
                particles[count].setPositionCoordinates(x=x, y=y, z=initialHeight(particles[count]))
                coordinates.append((x, y))
                count += 1
            else:
                y = sideLength * dist
                for j in range(sideLength):
                    if count < len(particles):
                        x = j * dist
                        particles[count].setPositionCoordinates(x=x, y=y, z=initialHeight(particles[count]))
                        coordinates.append((x, y))
                        count += 1
                    else:
                        break
                break
    return coordinates


def createXML(N):
    particles = []
    for i in range(N):
        particles.append(Particle(geomType, halfLen, radius, mass, rotorMass, solref, solimp))
    arrangeInitialPosition(particles, clearance)
    if geomType == "cylinder":
        return constructCylinderXMLWithChain(particles)
    return constructSphereXMLWithChain(particles)


""" SIMULATION """
def simulation(N):

    # RATES
    LOG_EVERY = 5 # simulation seconds
    NOISE_PHASE_EVERY = 0.1 # simulation seconds
    LAST_ALG_RUN = 0
    LAST_PHASE_NOISE = 0
    LAST_LOG = 0

    # MuJoCo objects
    xml = createXML(N)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    # INITIALIZE FREQUENCIES, COLORS, PHASES, SIGNS, AND GEOM IDs
    freq = np.array([np.random.choice([lowFreq, highFreq]) for i in range(N)])
    color = np.array([True for i in range(N)]) # True for red False for blue
    phases = np.random.uniform(0, 2 * np.pi, size=N)
    geomID = np.empty(N, dtype=np.int32)
    sign = initializeDirections(N)
    R = radius
    m = rotorMass
    steps_done = 0

    for i in range(N):
      id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{geomType}{i}")
      geomID[i] = id
      if id != -1:
          if color[i]:
              model.geom_rgba[id, :] = [1.0, 0.2, 0.2, 1.0]
          else:
              model.geom_rgba[id, :] = [0.2, 0.4, 1.0, 1.0]


    # DATA BUFFERS AND COLLECTION PARAMETERS
    RECORD_COM_EVERY = 25 # steps
    EXPECTED_SAMPLE_SIZE = int((SIM_DURATION / timestep) / RECORD_COM_EVERY)
    POSITION = np.empty((EXPECTED_SAMPLE_SIZE, 2), dtype=np.float64)
    currentPositions = np.empty((N, 2), dtype=np.float64)
    s = 0

    # SIMULATION LOOP
    WALLstart = time.perf_counter()
    while (data.time < SIM_DURATION and s < EXPECTED_SAMPLE_SIZE):

        # record positions
        if (steps_done % RECORD_COM_EVERY == 0):
            for i in range(N):
                currentPositions[i, :] = data.geom_xpos[geomID[i]][:2]
            POSITION[s, :] = np.mean(currentPositions, axis=0)
            s += 1

        # control algorithm
        if (data.time - LAST_ALG_RUN > RUN_ALG_EVERY):

            # find the position of all particles at the current step
            for i in range(N):
                currentPositions[i, :] = data.geom_xpos[geomID[i]][:2]
            com = np.mean(currentPositions, axis=0)

            # determine colors and frequencies
            for i in range(N):
                if np.dot(targetDirection, currentPositions[i] - com) > 0:
                    freq[i] = lowFreq
                    color[i] = False
                else:
                    freq[i] = highFreq
                    color[i] = True

            # update colors
            for i in range(N):
                if color[i]:
                    model.geom_rgba[geomID[i], :] = [1.0, 0.1, 0.1, 1.0]
                else:
                    model.geom_rgba[geomID[i], :] = [0.1, 0.1, 1.0, 1.0]

            LAST_ALG_RUN = data.time

            # add noise to the phases periodically
            if (data.time - LAST_PHASE_NOISE > NOISE_PHASE_EVERY):
                phases = phases + np.random.normal(0, phaseNoiseSTD * phases, size=N)
                LAST_PHASE_NOISE = data.time

        # update forces
        fx = 4 * (np.pi**2) * R * m * (freq**2) * np.cos(sign * 2 * np.pi * freq * data.time + phases)
        fy = 4 * (np.pi**2) * R * m * (freq**2) * np.sin(sign * 2 * np.pi * freq * data.time + phases)
        for i in range(N):
            data.xfrc_applied[i + 1, :3] = [fx[i], fy[i], 0]

        mujoco.mj_step(model, data)
        steps_done += 1

        # log to terminal
        if (data.time - LAST_LOG > LOG_EVERY):
            print(f"Simulated time (s): {data.time:.8f}")
            print(f"RTF = {(data.time / (time.perf_counter() - WALLstart)):.8f}")
            LAST_LOG = data.time

    del data
    del model

    return POSITION


""" SAVE SIMULATION PARAMETERS """
def saveParametersToFile(path):
    with open(path, "w") as file:
        file.write(f"N = {N} particles\n")
        file.write(f"Links per side = {linksPerSide}\n")
        file.write(f"Link length = {(linkLengthValue * 100)} (cm)\n")
        file.write(f"Chain tightness ratio = {tau}\n")
        file.write(f"High frequency = {highFreq} (Hz)\n")
        file.write(f"Low frequency = {lowFreq} (Hz)\n")
        file.write(f"Simulation duration = {SIM_DURATION} (s)\n")
        file.write(f"Timestep = {timestep} (s)\n")
        file.write(f"geomType = {geomType}\n")
        file.write(f"Run {runNumber}\n")


""" START CODE """
RAW_DATA_DIR = f"data/{geomType}/"
PARAMETERS_DIR = f"parameters/{geomType}/"
os.makedirs(RAW_DATA_DIR, exist_ok=True)
os.makedirs(PARAMETERS_DIR, exist_ok=True)

sizes = [600]
runsPerSize = 3
existingRuns = 10

line = "\n------------------------------------------\n"

print(line)
print("Running sizes", sizes)
print("Runs per size", runsPerSize)
print(line)

for i, N in enumerate(sizes):
    print(f"Simulating N = {N}")
    for runNumber in range(1, runsPerSize + 1):
        print(f"\nRun {runNumber + existingRuns} started")
        POSITION = simulation(N)
        print(f"Run {runNumber + existingRuns} complete")
        np.save(RAW_DATA_DIR + f"{N}p_{runNumber + existingRuns}.npy", POSITION)
        saveParametersToFile(PARAMETERS_DIR + f"{N}p_{runNumber + existingRuns}.txt")
        del POSITION
    print(line)

