from math import sqrt
import matplotlib.pyplot as plt
import numpy as np
from sys import argv

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

    files = ["36p_27.npy", "36p_28.npy", "36p_29.npy", 
             "36p_30.npy", "36p_31.npy", "36p_32.npy",
             "36p_33.npy", "36p_34.npy", "36p_35.npy",]
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

    