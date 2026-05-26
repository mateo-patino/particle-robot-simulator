"""
This is the entry point for the simulator. If you wish to interact with the simulator,
you will most likely want to call this script and pass command-line arguments to it.

Only one command-line argument is required: --config. This is the path to a .json file
containing the configurations parameters needed to run a simulation. You must always
provide this argument.

"""

import logging

from config.config import SimulationConfig
from simulator.experiments.sweep import run_1d_sweep
from simulator.experiments.single_run import run_single
from simulator.io.save import save_single_run, save_1d_sweep
from typing import get_type_hints
from sys import argv
import argparse

logger = logging.getLogger(__name__)

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Entry point for the simulator")

    parser.add_argument(
        "--config",
        required=True,
        type=str,
        help="Path to the .json configuration file for the simulation"
    )

    parser.add_argument(
        "--gui",
        action="store_true",
        help="Enable graphic visualization"
    )

    parser.add_argument(
        "--record",
        action="store_true",
        help="Record simulation to video file"
    )

    parser.add_argument(
        "--runs",
        type=int,
        default=1,
        help="Number of times to repeat the simulation"
    )

    parser.add_argument(
        "--existing-runs",
        type=int,
        default=0,
        help="Start naming result files at existing_runs + 1"
    )

    parser.add_argument(
        "--sweep",
        default=None,
        type=str,
        help="Conduct a parameter sweep on"
    )

    parser.add_argument(
        "--sweep-values",
        nargs='+',
        help="A list of values to sweep"
    )

    parser.add_argument(
        "--label",
        type=str,
        help="Insert a label in the path to the results directory"
    )

    parser.add_argument(
        "--no-save",
        default=False,
        action="store_true",
        help="Disable automatic saving of simulation results"
    )

    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING"],
        help="Set logging verbosity (default: INFO)"
    )

    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    # Load sim configuration from provided JSON file
    config = SimulationConfig.from_json(args.config)

    # Override config parameters
    if args.gui:
        config.gui = True
    if args.record:
        config.record = True

    # Run simulation in desired mode (sweep or not sweep)
    if args.sweep is not None:

        logger.info("Running in SWEEP mode...")

        target_parameter = args.sweep

        if args.sweep_values is None or len(args.sweep_values) == 0:
            raise ValueError("You must provide a list of space-separated values for the sweep via --sweep_values")

        if not hasattr(config, target_parameter):
            raise ValueError(f"Cannot sweep \"{target_parameter}\": not a valid configuration parameter.")

        attr_type = get_type_hints(config)[target_parameter]
        values = [attr_type(v) for v in args.sweep_values]

        # Run 1-dimensional sweep
        sweep_results = run_1d_sweep(config, target_parameter, values, num_runs=args.runs)

        # Save results
        if not args.no_save:
            results_path = save_1d_sweep(config, sweep_results, target_parameter, argv=argv, label=args.label, 
                                         existing_runs=args.existing_runs)
            logger.info("Results saved to \"%s\"", results_path)

    else:

        logger.info("Running in SINGLE mode...")

        # Run simulation
        single_run_results = run_single(config, args.runs)

        # Save results
        if not args.no_save:
            results_path = save_single_run(config, single_run_results, argv=argv, label=args.label, existing_runs=args.existing_runs)
            logger.info("Results saved to directory \"%s\"", results_path)
