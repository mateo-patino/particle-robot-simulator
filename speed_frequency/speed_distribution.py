"""

Author(s): Mateo Patino, Hod Lipson
Creative Machines Laboratory, Columbia University

This script reads all the runs for one or more gradient configurations and plots the distribution of speeds for those runs.
The user must provide a number of rows and columns to display the correct number of axes.

"""

import matplotlib.pyplot as plt
import numpy as np
from speed_vs_frequency import averageSpeed


def plotHistogram(ax, data, label):
    ax.hist(data, bins="auto", density=True, color="gray", edgecolor="black", label=label, zorder=2)
    ax.legend()
    ax.set_xlabel("Speed (cm/s)", fontsize=10)
    ax.set_ylabel("Probability density")
    ax.grid("both")

gradients = [(0, 10), (5, 10),
             (7, 10), (10, 10)]
runsPerGradient = 20

DATA_PATH = "data/sphere/"
PARAMETER_PATH = "parameters/sphere/"

rows = 2
columns = 2
distributions = []

fig, axes = plt.subplots(rows, columns)
for i, (low, high) in enumerate(gradients):
    speeds = []
    for r in range(1, runsPerGradient + 1):
        speeds.append(averageSpeed(DATA_PATH + f"{low}l_{high}h_{r}.npy", 
                                   PARAMETER_PATH + f"{low}l_{high}h_{r}.txt"))
    distributions.append(speeds)

count = 0
for r in range(rows):
    for c in range(columns):
        plotHistogram(axes[r][c], distributions[count], f"{gradients[count]}")
        count += 1

plt.show()
