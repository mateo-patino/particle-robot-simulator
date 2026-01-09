"""

Author(s): Mateo Patino, Hod Lipson
Creative Machines Laboratory, Columbia University

I use this script for plotting a "POSITION" array from a numpy binary file; useful for doing a quick
sanity check on a simulation.

"""

import matplotlib.pyplot as plt
import numpy as np

# makes a  x and y
def linePlot(x, y, title, xlabel, ylabel):
    plt.plot(x, y, color="tab:red", zorder=1)
    plt.scatter(x[:1], y[:1], color="black", marker="d", label="Start")
    plt.scatter(x[len(x)-1:], y[len(y)-1:], color="black", marker="x", label="End")
    plt.legend()
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid("both")
    plt.axis("equal")
    plt.show()


if __name__ == "__main__":

    file = "1l_10h_1.npy"
    geomType = "sphere"
    POSITION = np.load(f"data/{geomType}/" + file)

    x = POSITION[:, 0]
    y = POSITION[:, 1]

    linePlot(x, y, title=f"{file}", xlabel="x (m)", ylabel="y (m)")