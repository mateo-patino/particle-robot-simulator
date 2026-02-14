from simulator.model.xml_builder import create_xml
import numpy as np
import mujoco
import time

class Simulation:

    def __init__(self, config):
        self.config = config

    def initialize_directions(self):
        half = self.config.size // 2
        signs = [1] * half + [-1] * (self.config.size - half)
        np.random.shuffle(signs)
        return np.array(signs)
    
    def run(self):

        config = self.config
        
        if config.seed is not None:
            np.random.seed(config.seed)

        # Unpack config
        LOG_EVERY = config.log_sim_time_every # simulation seconds
        NOISE_PHASE_EVERY = config.noise_phase_every # simulation seconds
        RUN_ALG_EVERY = config.run_control_every # simulation seconds
        RECORD_COM_EVERY = config.record_com_every # steps
        SIM_DURATION = config.sim_duration
        size = config.size
        low_freq = config.low_freq
        high_freq = config.high_freq
        phase_noise_std = config.phase_noise_std
        geom_type = config.geom_type
        timestep = config.timestep
        target_direction = np.array(config.target_direction)
        R = config.particle_radius
        m = config.rotor_mass

        # MuJoCo objects
        xml = create_xml(config)
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        # Initialize frequencies, colors, phases, and geom IDs
        freq = np.array([np.random.choice([low_freq, high_freq]) for i in range(size)])
        color = np.array([True for i in range(size)]) # True for red False for blue
        phases = np.random.uniform(0, 2 * np.pi, size=size)
        geomID = np.empty(size, dtype=np.int32)
        sign = self.initialize_directions()

        for i in range(size):
            id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{geom_type}{i}")
            geomID[i] = id
            if id != -1:
                if color[i]:
                    # CHECK: does the else block every execute?
                    model.geom_rgba[id, :] = [1.0, 0.2, 0.2, 1.0]
                else:
                    model.geom_rgba[id, :] = [0.2, 0.4, 1.0, 1.0]


        # Data buffers
        EXPECTED_SAMPLE_SIZE = int((SIM_DURATION / timestep) / RECORD_COM_EVERY)
        COM_POSITION = np.empty((EXPECTED_SAMPLE_SIZE, 2), dtype=np.float64)
        current_positions = np.empty((size, 2), dtype=np.float64)
        samples_taken = 0
        print(f"\nNumber of COM samples to be collected: {EXPECTED_SAMPLE_SIZE}")

        # Main simulation loop
        LAST_LOG = 0
        LAST_ALG_RUN = 0
        LAST_PHASE_NOISE = 0
        steps_done = 0
        wall_start = time.perf_counter()
        while (data.time < SIM_DURATION and samples_taken < EXPECTED_SAMPLE_SIZE):

            # Record positions
            if (steps_done % RECORD_COM_EVERY == 0):
                for i in range(size):
                    current_positions[i, :] = data.geom_xpos[geomID[i]][:2]
                COM_POSITION[samples_taken, :] = np.mean(current_positions, axis=0)
                samples_taken += 1

            # Control algorithm
            if (data.time - LAST_ALG_RUN > RUN_ALG_EVERY):

                # Find the position of all particles at the current step
                for i in range(size):
                    current_positions[i, :] = data.geom_xpos[geomID[i]][:2]
                com = np.mean(current_positions, axis=0)

                # Determine colors and frequencies
                for i in range(size):
                    if np.dot(target_direction, current_positions[i] - com) > 0:
                        freq[i] = low_freq
                        color[i] = False
                    else:
                        freq[i] = high_freq
                        color[i] = True

                # CHECK: is this loop necessary? Could it be merged with the loop above?
                for i in range(size):
                    if color[i]:
                        model.geom_rgba[geomID[i], :] = [1.0, 0.1, 0.1, 1.0]
                    else:
                        model.geom_rgba[geomID[i], :] = [0.1, 0.1, 1.0, 1.0]

                LAST_ALG_RUN = data.time

                # Add noise to the phases periodically. CHECK: originally, this block was inside the control algorithm block. Should it be moved outside?
                if (data.time - LAST_PHASE_NOISE > NOISE_PHASE_EVERY):
                    phases = phases + np.random.normal(0, phase_noise_std * phases, size=size)
                    LAST_PHASE_NOISE = data.time

            # Update forces
            fx = 4 * (np.pi**2) * R * m * (freq**2) * np.cos(sign * 2 * np.pi * freq * data.time + phases)
            fy = 4 * (np.pi**2) * R * m * (freq**2) * np.sin(sign * 2 * np.pi * freq * data.time + phases)
            for i in range(size):
                data.xfrc_applied[i + 1, :3] = [fx[i], fy[i], 0]

            mujoco.mj_step(model, data)
            steps_done += 1

            # Log to terminal
            if (data.time - LAST_LOG > LOG_EVERY):
                real_time_elapsed = time.perf_counter() - wall_start
                print(f"\nReal time elapsed (s): {(real_time_elapsed):.8f}")
                print(f"Simulated time (s): {data.time:.8f}")
                print(f"Real-Time Factor: {(data.time / (real_time_elapsed)):.8f}")
                print(f"Steps completed: {steps_done}")
                LAST_LOG = data.time

        del data
        del model

        return COM_POSITION
