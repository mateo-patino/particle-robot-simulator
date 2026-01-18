"""

Author(s): Mateo Patino, Hod Lipson
Creative Machines Laboratory, Columbia University

This script reads the position data from the .npy files collected in the speed vs. frequency 
experiments and plots the average velocity of each gradient configuration. Note the use of the
word "velocity" and not "speed." As opposed to the speed_vs_frequency.py file, which plots the 
speed (net displacement over time), this script computes the average velocity vector among all
the runs made for a specific gradient configuration and plots the magnitude of this average vector.
This accounts for gradients that yield a high speed but low directional stability, as the net 
displacement vector for these simulations will point in random directions that will cancel out 
when added. Hence, only gradients that produce a large speed AND are directionally stable will
be highlighted by the graph.


"""

import matplotlib.pyplot as plt
import numpy as np
from speed_vs_frequency import getDuration


# returns a numpy array of the velocity vector given paths to position data and parameter files
def velocity(dataPath, paramsPath):
    POSITION = np.load(dataPath)
    duration = getDuration(paramsPath)
    return (POSITION[-1] - POSITION[0]) / duration


if __name__ == "__main__":

    geomType = "cylinder"
    runsPerGradient = 30
    gradients = [(10, 10), (9, 10), (8, 10), (7, 10), (6, 10), (5, 10),
                 (4, 10), (3, 10), (2, 10), (1, 10), (0, 10)]

    DATA_PATH = f"data/{geomType}/"
    PARAMETERS_PATH = f"parameters/{geomType}/"

    averageMagnitude = np.empty(len(gradients), dtype=np.float64)
    frequencyDifferences = np.empty(len(gradients), dtype=np.float64)
    lowerErrors = np.empty(len(gradients), dtype=np.float64)
    upperErrors = np.empty(len(gradients), dtype=np.float64)

    for i, (low, high) in enumerate(gradients):
        frequencyDifferences[i] = high - low
        velocities = np.empty((runsPerGradient, 2), dtype=np.float64)
        for r in range(1, runsPerGradient + 1):
            velocities[r-1] = velocity(DATA_PATH + f"{low}l_{high}h_{r}.npy", PARAMETERS_PATH + f"{low}l_{high}h_{r}.txt")

        # add up the velocity vectors and divided by the size to find the average velocity vector
        avgVelocity = np.sum(velocities, axis=0) / len(velocities)
        averageMagnitude[i] = np.linalg.norm(avgVelocity)

        # compute errors by looking at the magnitudes of the vectors in the velocity array
        magnitudes = np.linalg.norm(velocities, axis=1)
        medMag = np.median(magnitudes)

        # note that this error bars are based on the magnitudes array
        lowerErrors[i] = medMag - np.percentile(magnitudes, 25)
        upperErrors[i] = np.percentile(magnitudes, 75) - medMag


    # LaTeX rendering
    plt.rcParams["text.usetex"] = True
    plt.rcParams["font.family"] = "serif"
    plt.rcParams["text.latex.preamble"] = r"\usepackage{amsmath}"
    plt.rcParams['xtick.labelsize'] = 12
    plt.rcParams['ytick.labelsize'] = 13

    plt.scatter(frequencyDifferences, averageMagnitude, color="black", s=25, zorder=2)
    plt.errorbar(frequencyDifferences, averageMagnitude, yerr=[lowerErrors, upperErrors], color="black",
                    fmt="o", capsize=5, markersize=6, elinewidth=1.5)
    plt.xlabel(r"$\text{Frequency gradient } \omega_{\text{high}} - \omega_{\text{low}} \text{ (Hz)}$", fontsize=12)
    title = r"$\text{Median speed " + f"({geomType}) " + r"vs. frequency gradient for } N = 100 $"
    plt.ylabel(r"$\text{Speed (cm/s)}$", fontsize=12)
    plt.title(title, fontsize=14)
    plt.grid("both")
    plt.show()