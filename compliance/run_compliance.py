"""

This is the script used to perform the compliance experiments.

"""

import logging

from config.config import SimulationConfig
from simulator.experiments.gap_traversal import run_gap_traversal_fs
from sys import argv
import argparse
import logging

logger = logging.getLogger(__name__)

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Entry point to simulator for compliance experiments")

    
    parser.add_argument(
        "--config",
        required=True,
        type=str,
        help="Path to the .json configuration file for the simulation"
    )

    parser.add_argument(
        "--gammas",
        required=True,
        nargs='+',
        help="List of gamma ratios"
    )

    parser.add_argument(
        "--gui",
        action="store_true",
        help="Enable graphic visualization"
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
        "--label",
        type=str,
        help="Insert a label in the path to the results directory"
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

    config = SimulationConfig.from_json(args.config)

    if args.gui:
        config.gui = True

    # Perform desired number of runs for each gamma value
    gammas = [float(g) for g in args.gammas]
    results_path = run_gap_traversal_fs(config, gammas, num_runs=args.runs, existing_runs=args.existing_runs, label=args.label,
                                        argv=argv)
    logger.info(f"Results saved to {results_path}")

