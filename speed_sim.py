"""

Author(s): Mateo Patino, Hod Lipson
Creative Machines Laboratory, Columbia University

This script implements a simulation using MuJoCo's passive viewer. This script is otherwise identical to
the runtime.py script in the "colab" folder, which does NOT use the passive viewer. I use this script to
test the behavior of the particle robot and determine ideal parameters to use in real experiments.

"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import build_mj_model
from lowsizeparams import gradient, control, algorithm, particle, SIM_DURATION
import os


def netDisplacement(start, end):
    return np.linalg.norm(end - start)


def noise(gauss_std, value, size=1):
    n = np.random.normal(0, gauss_std * value, size=size)
    if size == 1:
        return n[0]
    return n


def intializeDirections(N):
    half = N // 2
    signs = [1] * half + [-1] * (N - half)
    np.random.shuffle(signs)
    return np.array(signs)


def launch():

    """
    particles[0:low] have low energy, particles[low:low+high] have high energy
    """

    # performance parameters
    STEPS_PER_RENDER = algorithm["STEPS_PER_RENDER"]
    LOG_EVERY = 2
    NOISE_PHASE_EVERY = 0.1

    # core parameters
    N = gradient["N"]
    lowFreq = gradient["lowFreq"]
    highFreq = gradient["highFreq"]
    phaseNoiseSTD = gradient["phaseNoiseSTD"]


    # initialize simulation
    xml = build_mj_model.xml(N)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)


    # run the visualization loop
    with mujoco.viewer.launch_passive(model, data) as viewer:

        # CAMERA
        viewer.cam.lookat[:] = [0, 0, 0]  # Look at the origin
        viewer.cam.distance = 2.0         # 1 meter away
        viewer.cam.elevation = -90        # Look straight down
        viewer.cam.azimuth = 90

        # DIRECTION AND LOGS
        dir = control["targetDirection"]
        RUN_ALG_EVERY = control["RUN_EVERY"] # check every milisecond how velocity vector is doing against target
        LAST_ALG_RUN = 0
        LAST_PHASE_NOISE = 0
        LAST_LOG = 0

        # INITIALIZE FREQUENCIES, COLORS, PHASES, SIGNS, AND GEOM IDs
        freq = np.array([np.random.choice([lowFreq, highFreq]) for i in range(N)])
        color = np.array([True for i in range(N)]) # True for red False for blue
        phases = np.random.uniform(0, 2 * np.pi, size=N)
        geomID = np.empty(N, dtype=np.int32)
        geomType = particle["geomType"]
        sign = intializeDirections(N)
        R = particle["radius"]
        m = particle["rotorMass"]
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
        EXPECTED_SAMPLE_SIZE = int(SIM_DURATION / (RECORD_COM_EVERY * algorithm["timestep"]))
        POSITION = np.empty((EXPECTED_SAMPLE_SIZE, 2), dtype=np.float64)
        currentPositions = np.empty((N, 2), dtype=np.float64)
        s = 0

        # report sample size
        print(f"\nNumber of COM samples {EXPECTED_SAMPLE_SIZE}")

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
                    if np.dot(dir, currentPositions[i] - com) > 0:
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
                phases = phases + noise(phaseNoiseSTD, phases, size=N)
                LAST_PHASE_NOISE = data.time

            # update forces
            fx = 4 * (np.pi**2) * R * m * (freq**2) * np.cos(sign * 2 * np.pi * freq * data.time + phases)
            fy = 4 * (np.pi**2) * R * m * (freq**2) * np.sin(sign * 2 * np.pi * freq * data.time + phases)
            for i in range(N):
                data.xfrc_applied[i + 1, :3] = [fx[i], fy[i], 0]
            
            mujoco.mj_step(model, data)
            steps_done += 1

            # sync the viewer
            if (steps_done % STEPS_PER_RENDER == 0):
                viewer.sync()
            
            # log to terminal
            WALLnow = time.perf_counter()
            if (WALLnow - LAST_LOG > LOG_EVERY):
                print(f"\nWall-time elapsed (s): {(WALLnow - WALLstart):.8f}")
                print(f"Simulated time (s): {data.time:.8f}")
                print(f"RTF = {(data.time / (WALLnow - WALLstart)):.8f}")
                LAST_LOG = WALLnow

    # ensure the viewer thread is killed before the new simulation begins. Two concurrent viewers leads to crashes
    viewer = None
    time.sleep(0.2)

    # SUMMARY
    netD = netDisplacement(POSITION[0], POSITION[-1])
    print(f"\nSteps completed: {steps_done}")
    print(f"Time of simulation (s): {data.time:.8f}\n")
    print(f"Net displacement: {(100*netD):.8f} cm")
    print(f"Average speed: {(100 * netD / data.time):.8f} cm/s \n")

    return POSITION
    

if __name__ == "__main__":

    # Record the position data of the particles as a numpy binary file.
    dataPath = f"data/{particle["geomType"]}/"
    os.makedirs(dataPath, exist_ok=True)
    runs = 5
    existingRuns = 0
    for r in range(1, runs + 1):
        POSITION = launch()
        np.save(dataPath + f"{gradient["N"]}p_{r + existingRuns}.npy", POSITION)
        del POSITION