"""

Author(s): Mateo Patino, Hod Lipson
Creative Machines Laboratory, Columbia University

This script reads the .npy data files collected from the speed vs. frequency experiments and plots
the median speed for each gradient against its frequency difference (high freq. - low freq.).

"""

import matplotlib.pyplot as plt
import numpy as np


""" returns a value d*f, where d is the diameter of the robot and f is the frequency gradient. It takes a 
path to a .txt parameters file and reads the frequencies, link length, and links per side"""
def normalizationFactor(paramsPath):
    # assumes file structure as built by the saveParametersToFile() function in runtime.py
    with open(paramsPath) as file:
        file.readline() # read off N
        linksPerSide = int(file.readline().split(" ")[4])
        L = float(file.readline().split(" ")[3])

        file.readline() # read off tau
        highFreq = float(file.readline().split(" ")[3])
        lowFreq = float(file.readline().split(" ")[3])
        
        return linksPerSide * L * (highFreq - lowFreq)


# returns the average speed (cm/s) of a .npy file from the given data and parameter path
def averageSpeed(dataPath, paramsPath):

    # read the simulation duration from the parameters .txt file
    with open(paramsPath) as paramsFile:
        line = paramsFile.readline().split(" ")
        while (len(line) > 0 and line[0] != "Simulation"):
            line = paramsFile.readline().split(" ")

        # the line must be formatted "Simulation duration = X (s)" so that line[3] is duration
        duration = float(line[3])

    # compute net displacement
    POSITION = np.load(dataPath)
    D = np.linalg.norm(POSITION[-1] - POSITION[0])

    return 100 * D / duration

if __name__ == "__main__":
    # intervals to plot and number of runs to read
    runsPerGradient = 30
    gradients = [(0, 10), (1, 10), (2, 10), (3, 10), (4, 10), (5, 10),
                (6, 10), (7, 10), (8, 10), (9, 10), (10, 10)]
    gradients.reverse() # start from low gradients to high gradients
    NORMALIZE = True # normalize speed by d * (high - low)
    geomType = "cylinder"

    # buffers and paths
    medianSpeeds = []
    lowerErrors = []
    upperErrors = []
    frequencyDifference = []
    DATA_PATH = f"data/{geomType}/"
    PARAMETER_PATH = f"parameters/{geomType}/"

    for i, (low, high) in enumerate(gradients):
        frequencyDifference.append(high - low)
        speeds = []
        for r in range(1, runsPerGradient + 1):
            speeds.append(averageSpeed(DATA_PATH + f"{low}l_{high}h_{r}.npy",
                                    PARAMETER_PATH + f"{low}l_{high}h_{r}.txt"))
        median = np.median(speeds)
        medianSpeeds.append(median)
        lowerErrors.append(median - np.percentile(speeds, 25))
        upperErrors.append(np.percentile(speeds, 75) - median)

    frequencyDifference = np.array(frequencyDifference)
    medianSpeeds = np.array(medianSpeeds)
    lowerErrors = np.array(lowerErrors)
    upperErrors = np.array(upperErrors)

    # LaTeX rendering
    plt.rcParams["text.usetex"] = True
    plt.rcParams["font.family"] = "serif"
    plt.rcParams["text.latex.preamble"] = r"\usepackage{amsmath}"
    plt.rcParams['xtick.labelsize'] = 12
    plt.rcParams['ytick.labelsize'] = 13

    # normalize if needed
    if NORMALIZE:
        factors = []
        for (low, high) in gradients:
            # for the same gradient, all runs have identical settings, so use the 1st run
            factors.append(normalizationFactor(PARAMETER_PATH + f"{low}l_{high}h_1.txt"))
        factors = np.array(factors)
        mask = factors != 0
        medianSpeeds = medianSpeeds[mask] / factors[mask]
        frequencyDifference = frequencyDifference[mask]
        lowerErrors = lowerErrors[mask]
        upperErrors = upperErrors[mask]

    # plot
    plt.scatter(frequencyDifference, medianSpeeds, color="black", s=25, zorder=3)
    if not NORMALIZE:
        plt.errorbar(frequencyDifference, medianSpeeds, yerr=[lowerErrors, upperErrors], color="black",
                    fmt="o", capsize=5, markersize=6, elinewidth=1.5)
    plt.xlabel(r"$\text{Frequency gradient } \omega_{\text{high}} - \omega_{\text{low}} \text{ (Hz)}$", fontsize=12)
    if NORMALIZE:
        title = r"$\text{Normalized median speed " + f"({geomType}) " + r"vs. frequency gradient for } N = 100 $"
        plt.ylabel(r"$\text{Normalized speed} \left(\frac{v}{d\Delta f}\right)$", fontsize=12)
        plt.title(title, fontsize=14)
    else:
        title = r"$\text{Median speed " + f"({geomType}) " + r"vs. frequency gradient for } N = 100 $"
        plt.ylabel(r"$\text{Speed (cm/s)}$", fontsize=12)
        plt.title(title, fontsize=14)
    plt.grid()
    plt.show()
