"""

Author(s): Mateo Patino, Hod Lipson
Creative Machines Laboratory, Columbia University

This script plots median speed vs. size (N) for the speed-size experiments. It must be run from the 
root directory of the speed_frequency and colab directories.

"""

from speed_frequency.speed_vs_frequency import averageSpeed
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":

    sizes = [100, 200, 300, 400, 500, 600]
    runsPerSize = 15
    geomType = "sphere"

    DATA_PATH = f"colab/data/{geomType}/"
    PARAMS_PATH = f"colab/parameters/{geomType}/"

    medianSpeeds = []
    lowerErr = []
    upperErr = []
    for s in sizes:
        speeds = []
        for i in range(1, runsPerSize + 1):
            speeds.append(averageSpeed(DATA_PATH + f"{s}p_{i}.npy", PARAMS_PATH + f"{s}p_{i}.txt"))
        median = np.median(speeds)
        medianSpeeds.append(median)
        lowerErr.append(median - np.percentile(speeds, 25))
        upperErr.append(np.percentile(speeds, 75) - median)
    
    # LaTeX rendering
    plt.rcParams["text.usetex"] = True
    plt.rcParams["font.family"] = "serif"
    plt.rcParams["text.latex.preamble"] = r"\usepackage{amsmath}"
    plt.rcParams['xtick.labelsize'] = 12
    plt.rcParams['ytick.labelsize'] = 13

    plt.scatter(sizes, medianSpeeds, color="black", s=25, zorder=3)
    plt.errorbar(sizes, medianSpeeds, yerr=[lowerErr, upperErr], color="black",
                    fmt="o", capsize=5, markersize=6, elinewidth=1.5)
    plt.xlabel(r"$\text{Number of particles}$", fontsize=12)
    plt.ylabel(r"$\text{Speed (cm/s)}$", fontsize=12)
    plt.title(r"$\text{Median speed " + f"({geomType}) " + " vs. robot size}$", fontsize=14)
    plt.grid("both")
    plt.show()
