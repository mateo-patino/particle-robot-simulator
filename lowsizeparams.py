"""

Author(s): Mateo Patino, Hod Lipson
Creative Machines Laboratory, Columbia University

This script contains all the parameters than the simulation needs to run. Many hours have been 
poured into optimizing this parameters so the simulation is stable, fast, and physically correct.

"""

import numpy as np

"""PLANE LENGTH"""
planeLengthFactor = 4

""" BASIC PARAMETERS OF A CYLINDRICAL/SPHERICAL PARTICLE """
radius = 0.019
halfLen = radius
mass = 0.26
solref = [0.003, 1] # [r, d] = [how quickly the constraint error is corrected, damping; how inelastic the collision is] [0.003, 1]
solimp = [0.95, 0.95, 0.005] # [m, h, w] = [rate at which stiffness grows with penetration, min. impendance when constraint is barely violated, width of the interval around the contact margin where the transition in stiffness occurs] [0.95, 0.95, 0.005]
clearance = 0.00001 # a small distance used to separate particles in the initial set up of the simulation to avoid interpenetration
rotorMass = 4 * 0.014
geomType = "sphere" # "sphere" or "cylinder"

particle = {
        "halfLen": halfLen,
        "radius": radius,
        "mass": mass,
        "solref": solref,
        "solimp": solimp,
        "rotorMass": rotorMass,
        "geomType": geomType,
    }

""" PARAMETERS OF THE HIGH-LOW ENERGY GRADIENT. """
N = 100
highFreq = 10 # Hz
lowFreq = 0 # Hz
phase = 2 * np.pi
forceNoiseSTD = 0.01 #0.01
phaseNoiseSTD = 0.05 #0.1

gradient = {
    "N": N,
    "highFreq": highFreq,
    "lowFreq": lowFreq,
    "phase": phase,
    "noiseSTD": forceNoiseSTD,
    "phaseNoiseSTD": phaseNoiseSTD,
}

""" PARAMETERS OF THE ALGORITHM """
SIM_DURATION = 20
timestep = 7e-05 # 7e-05
solver = "Newton"
iterations = 100
tolerance = 1e-12
STEPS_PER_RENDER = 500
algorithm = {
    "timestep": timestep,
    "solver": solver,
    "iterations": iterations,
    "tolerance": tolerance,
    "STEPS_PER_RENDER": STEPS_PER_RENDER,
}


"""PARAMETERS OF THE CAPSULES"""
capsuleRadius = radius #radius
capsuleMass = 0.0155
cap_solref = [0.003, 1] # [r, d] = [how quickly the constraint error is corrected, damping; how inelastic the collision is] [0.003, 1]
cap_solimp = [0.9, 0.9, 0.005] # [m, h, w] = [rate at which stiffness grows with penetration, min. impendance when constraint is barely violated, width of the interval around the contact margin where the transition in stiffness occurs] [0.9, 0.9, 0.005]
capsule = {
    "radius": capsuleRadius,
    "mass": capsuleMass,
    "solref": cap_solref,
    "solimp": cap_solimp,
}

"""CHAIN PARAMETERS. This script assumes that linksPerSide is constant/arbitrary."""
linksPerSide = 5 #5
tau = 0.6 # tightness ratio 0 < tau <= pi/4 (higher = tighter, lower = looser). Mathematically bounded above by pi/4 (~0.785); beyond that interpentration occurs
chain = {
    "linksPerSide": linksPerSide,
    "tau": tau,
}


"""CONTROL ALGORITHM"""
targetDirection = np.array([1, 0])
RUN_EVERY = 0.05 # 0.01
margin = 2
control = {
    "targetDirection": targetDirection,
    "RUN_EVERY": RUN_EVERY,
    "margin": margin,
}