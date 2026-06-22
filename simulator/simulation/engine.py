import logging

from simulator.model.xml_builder import create_xml
from config.config import SimulationConfig
import numpy as np
import mujoco
import time

logger = logging.getLogger(__name__)

class Simulation:

    def __init__(self, config: SimulationConfig) -> None:
        self.config = config

    def initialize_directions(self, rng: np.random.Generator) -> np.ndarray:
        half = self.config.size // 2
        signs = [1] * half + [-1] * (self.config.size - half)
        rng.shuffle(signs)
        return np.array(signs)

    def run(self) -> np.ndarray:

        config = self.config

        rng = np.random.default_rng(config.seed)

        # Unpack config
        LOG_EVERY = config.log_sim_time_every # simulation seconds
        NOISE_PHASE_EVERY = config.noise_phase_every # simulation seconds
        RUN_ALG_EVERY = config.run_control_every # simulation seconds
        RECORD_COM_EVERY = config.record_com_every # steps
        RENDER_EVERY = config.render_every # steps
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

        # GUI and video setup
        viewer = None
        renderer = None
        frames: list[np.ndarray] = []

        if config.gui:
            from mujoco.viewer import launch_passive
            viewer = launch_passive(model, data)
        if config.record:
            renderer = mujoco.Renderer(model, height=480, width=640)

        # Initialize frequencies, colors, phases, and geom IDs
        freq = rng.choice([low_freq, high_freq], size=size)
        color = np.ones(size, dtype=bool) # True for red False for blue
        phases = rng.uniform(0, 2 * np.pi, size=size)
        geomID = np.empty(size, dtype=np.int32)
        sign = self.initialize_directions(rng)

        for i in range(size):
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{geom_type}{i}")
            if gid == -1:
                raise RuntimeError(f"Geom '{geom_type}{i}' not found in model")
            geomID[i] = gid

        # Vectorized RGBA init
        model.geom_rgba[geomID[color]] = [1.0, 0.2, 0.2, 1.0]
        model.geom_rgba[geomID[~color]] = [0.2, 0.4, 1.0, 1.0]

        # Pre-compute body IDs for force application
        body_ids = np.arange(1, size + 1)

        # Data buffers
        EXPECTED_SAMPLE_SIZE = int((SIM_DURATION / timestep) / RECORD_COM_EVERY)
        COM_POSITION = np.empty((EXPECTED_SAMPLE_SIZE, 2), dtype=np.float64)
        current_positions = np.empty((size, 2), dtype=np.float64)
        samples_taken = 0
        logger.info("Number of COM samples to be collected: %d", EXPECTED_SAMPLE_SIZE)

        # Main simulation loop
        LAST_LOG = 0
        LAST_ALG_RUN = 0
        LAST_PHASE_NOISE = 0
        steps_done = 0
        wall_start = time.perf_counter()
        while (data.time < SIM_DURATION and samples_taken < EXPECTED_SAMPLE_SIZE):\

            # TODO: implement callback for early stoppage

            # Record positions
            if (steps_done % RECORD_COM_EVERY == 0):
                current_positions[:] = data.geom_xpos[geomID, :2]
                COM_POSITION[samples_taken, :] = np.mean(current_positions, axis=0)
                samples_taken += 1

            # Control algorithm
            if (data.time - LAST_ALG_RUN > RUN_ALG_EVERY):

                # Find the position of all particles at the current step
                current_positions[:] = data.geom_xpos[geomID, :2]
                com = np.mean(current_positions, axis=0)

                # Determine colors and frequencies
                dots = (current_positions - com) @ target_direction
                mask = dots > 0
                freq[mask] = low_freq
                freq[~mask] = high_freq
                color[mask] = False
                color[~mask] = True

                # Update RGBA
                model.geom_rgba[geomID[color]] = [1.0, 0.1, 0.1, 1.0]
                model.geom_rgba[geomID[~color]] = [0.1, 0.1, 1.0, 1.0]

                LAST_ALG_RUN = data.time

                # Add noise to the phases periodically.
                if (data.time - LAST_PHASE_NOISE > NOISE_PHASE_EVERY):
                    phases = phases + rng.normal(0, phase_noise_std * phases, size=size)
                    LAST_PHASE_NOISE = data.time

            # Update forces
            fx = 4 * (np.pi**2) * R * m * (freq**2) * np.cos(sign * 2 * np.pi * freq * data.time + phases)
            fy = 4 * (np.pi**2) * R * m * (freq**2) * np.sin(sign * 2 * np.pi * freq * data.time + phases)
            data.xfrc_applied[body_ids, 0] = fx
            data.xfrc_applied[body_ids, 1] = fy
            data.xfrc_applied[body_ids, 2] = 0.0

            mujoco.mj_step(model, data)
            steps_done += 1

            # Render
            if steps_done % RENDER_EVERY == 0:
                if viewer is not None:
                    viewer.sync()
                    if not viewer.is_running():
                        break
                if renderer is not None:
                    renderer.update_scene(data, camera="top")
                    frames.append(renderer.render().copy())

            # Log to terminal
            if (data.time - LAST_LOG > LOG_EVERY):
                real_time_elapsed = time.perf_counter() - wall_start
                logger.debug("Real time elapsed (s): %.8f", real_time_elapsed)
                logger.debug("Simulated time (s): %.8f", data.time)
                logger.debug("Real-Time Factor: %.8f", data.time / real_time_elapsed)
                logger.debug("Steps completed: %d", steps_done)
                LAST_LOG = data.time

        # Cleanup viewer and save video
        if viewer is not None:
            viewer.close()
        if renderer is not None and frames:
            import imageio.v3 as iio
            video_path = config.video_path or "simulation.mp4"
            fps = max(1, min(int(1.0 / (timestep * RENDER_EVERY)), 60))
            iio.imwrite(video_path, np.stack(frames), fps=fps)
            logger.info("Video saved to %s", video_path)
        if renderer is not None:
            renderer.close()

        del data
        del model

        return COM_POSITION[:samples_taken]
