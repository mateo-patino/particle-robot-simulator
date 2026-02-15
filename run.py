from config.config import SimulationConfig
from simulator.experiments.sweep import run_1d_sweep
import argparse

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
        help="Disable graphic visualization"
    )

    parser.add_argument(
        "--runs",
        type=int,
        default=1,
        help="Number of times to repeat the simulation"
    )

    parser.add_argument(
        "--sweep",
        default=None,
        type=str,
        help="Conduct a parameter sweep on"
    )

    parser.add_argument(
        "--sweep_values",
        nargs='+',
        help="A list of values to sweep"
    )

    parser.add_argument(
        "--label",
        type=str,
        help="Insert a label in the path to the results directory"
    )

    parser.add_argument(
        "--no_save",
        default=False,
        action="store_true",
        help="Disable automatic saving of simulation results"
    )

    args = parser.parse_args()

    config = SimulationConfig.from_json(args.config)

    # Override config parameters
    if args.gui:
        config.gui = True

    # Run simulation in desired mode (sweep or not sweep)
    if args.sweep is not None:
        target_parameter = args.sweep

        if len(args.sweep_values) == 0:
            raise ValueError("You must provide a list of space-separated values for the sweep via --sweep_values")
        
        attr_type = type(getattr(config, target_parameter))
        values = [attr_type(v) for v in args.sweep_values]
    
        # Run 1-dimensional sweep
        sweep_results = run_1d_sweep(config, target_parameter, values, num_runs=args.runs)

        # TODO: 1) Save for sweep mode 2) Execute single-run mode



    
    

    

        





