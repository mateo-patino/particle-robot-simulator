"""

Author(s): Mateo Patino, Hod Lipson
Creative Machines Laboratory, Columbia University

This script reads the .npy data files collected from the speed vs. frequency experiments and plots
the median speed for each gradient against its frequency difference (high freq. - low freq.).

"""

import matplotlib.pyplot as plt
import numpy as np


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
        

# intervals to plot and number of runs to read
runsPerGradient = 20
gradients = [(0, 10), (1, 10), (2, 10), (3, 10), (4, 10), (5, 10),
             (6, 10), (7, 10), (8, 10), (9, 10), (10, 10)]
gradients.reverse()

# buffers and paths
medianSpeeds = []
frequencyDifference = []
DATA_PATH = "data/sphere/"
PARAMETER_PATH = "parameters/sphere/"

for i, (low, high) in enumerate(gradients):
    frequencyDifference.append(high - low)
    speeds = []
    for r in range(1, runsPerGradient + 1):
        speeds.append(averageSpeed(DATA_PATH + f"{low}l_{high}h_{r}.npy",
                                   PARAMETER_PATH + f"{low}l_{high}h_{r}.txt"))
    medianSpeeds.append(np.median(speeds))

# plot
plt.plot(frequencyDifference, medianSpeeds, color="tab:red", zorder=1)
plt.xlabel("Frequency gradient (high - low)")
plt.ylabel("Speed (cm/s)")
plt.title("Median speed vs. frequency gradient for N = 100 robot")
plt.grid()
plt.show()
