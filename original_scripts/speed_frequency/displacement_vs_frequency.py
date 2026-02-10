"""

Author(s): Mateo Patino, Hod Lipson
Creative Machines Laboratory, Columbia University

This script reads the position data from the .npy files collected in the speed vs. frequency 
experiments and plots the net displacement vectors for each run for each gradient. This plot is useful
for sensing the directional stability of a gradient configuration, as it shows the direction of motion for
all runs for a fixed gradient.

"""

import numpy as np
import matplotlib.pyplot as plt


# returns the net displacement vector for a specific data path
def netDisplacement(dataPath):
    POSITION = np.load(dataPath)
    return POSITION[-1] - POSITION[0]


if __name__ == "__main__":

    geomType = "cylinder"
    runsPerGradient = 30
    gradients = [(10, 10), (9, 10), (8, 10), (7, 10), (6, 10), (5, 10),
                 (4, 10), (3, 10), (2, 10), (1, 10), (0, 10)]
    DATA_PATH = f"data/{geomType}/"

    scaleBy = 5
    quiverParams = {
        "color": "black",
        "angles": "xy",
        "scale_units": "xy",
        "scale": 1,
        "width": 0.004,
        "alpha": 0.7,
        "zorder": 2,
    }
    
    displacements = []
    
    for i, (low, high) in enumerate(gradients):
        x = high - low
        for r in range(1, runsPerGradient):
            disp = netDisplacement(DATA_PATH + f"{low}l_{high}h_{r}.npy")
            plt.quiver(x, 0, scaleBy * disp[0], scaleBy * disp[1], **quiverParams)

    plt.title(f"Inidividual net displacement vectors ({geomType}) vs. frequency gradient ({scaleBy}x scale)")
    plt.xlabel(r"$\text{x (m), } \text{gradient } \omega_\text{high} - \omega_\text{low}$", fontsize=12)
    plt.ylabel(r"$\text{y (m)}$")
    plt.grid("both")
    plt.axis("equal")
    plt.show()    
