"""

Author(s): Mateo Patino, Hod Lipson
Creative Machines Laboratory, Columbia University

This script plots median speed vs. size (N) for the speed-size experiments. It must be run from the 
root directory of the speed_frequency and colab directories.

"""

from speed_frequency.speed_vs_frequency import averageSpeed, normalizationFactor
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":

    sizes = [100, 200, 300, 400, 500, 600]
    runsPerSize = 15
    geomType = "sphere"
    NORMALIZE = True

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
    
    medianSpeeds = np.array(medianSpeeds)
    lowerErr = np.array(lowerErr)
    upperErr = np.array(upperErr)

    # normalize if requested
    if NORMALIZE:
        factors = []
        for s in sizes:
            factors.append(normalizationFactor(PARAMS_PATH + f"{s}p_1.txt")) # use the first run
        factors = np.array(factors)
        mask = factors != 0
        medianSpeeds = medianSpeeds[mask] / factors[mask]
        lowerErr = lowerErr[mask]
        upperErr = upperErr[mask]

    
    # LaTeX rendering
    plt.rcParams["text.usetex"] = True
    plt.rcParams["font.family"] = "serif"
    plt.rcParams["text.latex.preamble"] = r"\usepackage{amsmath}"
    plt.rcParams['xtick.labelsize'] = 12
    plt.rcParams['ytick.labelsize'] = 13

    plt.scatter(sizes, medianSpeeds, color="black", s=25, zorder=3)
    if not NORMALIZE:
        plt.errorbar(sizes, medianSpeeds, yerr=[lowerErr, upperErr], color="black",
                    fmt="o", capsize=5, markersize=6, elinewidth=1.5)
    plt.xlabel(r"$\text{Number of particles}$", fontsize=12)
    if NORMALIZE:
        plt.title(r"$\text{Normalized median speed " + f"({geomType}) " + r" vs. robot size for } \Delta f = 7 \text{ Hz} $", fontsize=14)
        plt.ylabel(r"$\text{Normalized speed} \left(\frac{v}{d \Delta f} \right)$", fontsize=12)
    else:
        plt.title(r"$\text{Median speed " + f"({geomType}) " + r" vs. robot size for } \Delta f = 7 Hz $", fontsize=14)
        plt.ylabel(r"$\text{Speed (cm/s)}$", fontsize=12)
    plt.grid("both")
    plt.show()
