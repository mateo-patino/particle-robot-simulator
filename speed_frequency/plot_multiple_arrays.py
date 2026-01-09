"""

Author(s): Mateo Patino, Hod Lipson
Creative Machines Laboratory, Columbia University

I use this script for plotting multiple numpy arrays collected from the simulations. This script 
uses a grid with user-defined rows and columns to plot many arrays in multiple axes.

"""

import matplotlib.pyplot as plt
import numpy as np

# makes a  x and y
def linePlot(ax, x, y, title, xlabel, ylabel):
    ax.plot(x, y, color="tab:red", zorder=1)
    ax.scatter(x[:1], y[:1], color="black", marker="d", label="Start")
    ax.scatter(x[len(x)-1:], y[len(y)-1:], color="black", marker="x", label="End")
    ax.legend()
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid("both")
    ax.axis("equal")


if __name__ == "__main__":

    files = ["1l_10h_1.npy", "1l_10h_2.npy", "1l_10h_3.npy",
             "5l_10h_1.npy", "5l_10h_2.npy", "5l_10h_3.npy",
             "10l_10h_1.npy", "10l_10h_2.npy", "10l_10h_3.npy",]
    rows = 3
    columns = 3
    fig, axes = plt.subplots(rows, columns)

    fileCount = 0
    for r in range(rows):
        for c in range(columns):
            file = files[fileCount]
            fileCount += 1
            POSITION = np.load("data/" + file)
            x = POSITION[:, 0]
            y = POSITION[:, 1]
            linePlot(axes[r][c], x, y, title=f"{file}", xlabel="x (m)", ylabel="y (m)")

    plt.show()

    