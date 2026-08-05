from simulator.experiments.gap_traversal import get_gap_position
from pathlib import Path
from config.config import SimulationConfig
from pathlib import Path
import statsmodels.api as sm
import os
import matplotlib.pyplot as plt
import numpy as np
import json


""" 

Computes the delta-method standard error of a scalar quantity. 'gradient' is the gradient
of the quantity with respect to the regression coefficients and 'covariance' is the 
covariance matrix of the regression coefficients.

"""

def delta_standard_error(gradient: np.ndarray, covariance: np.ndarray) -> float:
    return float(np.sqrt(gradient @ covariance @ gradient))

"""

Saves statistics from a fitted binomial logistic regression to a JSON file.

"""

def save_regression_metadata(result, size: int) -> None:

    out_dir = Path("vector_images/compliance_regression")
    out_dir.mkdir(parents=True, exist_ok=True)

    beta0, beta1 = result.params

    gamma50 = -beta0 / beta1
    gamma10 = (np.log(0.1 / 0.9) - beta0) / beta1
    gamma90 = (np.log(0.9 / 0.1) - beta0) / beta1

    cov = result.cov_params()

    grad50 = np.array([-1 / beta1, beta0 / beta1**2])
    gamma50_se = delta_standard_error(grad50, cov)

    c10 = np.log(0.1 / 0.9)
    grad10 = np.array([-1 / beta1, -(c10 - beta0) / beta1**2])
    gamma10_se = delta_standard_error(grad10, cov)

    c90 = np.log(0.9 / 0.1)
    grad90 = np.array([-1 / beta1, -(c90 - beta0) / beta1**2])
    gamma90_se = delta_standard_error(grad90, cov)

    ci = result.conf_int()

    metadata = {
        "beta0": float(beta0),
        "beta1": float(beta1),

        "beta0_95_confidence_interval": [
            float(ci[0, 0]),
            float(ci[0, 1])
        ],

        "beta1_95_confidence_interval": [
            float(ci[1, 0]),
            float(ci[1, 1])
        ],

        "gamma10": float(gamma10),
        "gamma50": float(gamma50),
        "gamma90": float(gamma90),
        
        "gamma10_se": float(gamma10_se),
        "gamma50_se": float(gamma50_se),
        "gamma90_se": float(gamma90_se),

        "transition_width": float(gamma90 - gamma10),
        
        "equation": f"P(pass) = 1 / (1 + exp(-({beta0:.6f} + {beta1:.6f}*gamma)))",
        "log_likelihood": float(result.llf),
        "deviance": float(result.deviance),
        "pearson_chi2": float(result.pearson_chi2),
        "aic": float(result.aic),
        "bic": float(result.bic)
    }

    with open(out_dir / f"size{size}_regression.json", "w") as file:
        json.dump(metadata, file, indent=4)


"""

Returns True if the last recorded COM position in 'positions' is further away from the origin

than the gap's position (i.e. the VPRs COM crossed the gap).

Note: the simulation loop checks that ALL Particles have crossed the gap's position, while this
function checks that the COM crossed the gap. 

"""

def crossed_gap(positions: np.ndarray, size: int, gamma: float, env_dir_path: str = "config/env/"):
    
    env_path = os.path.join(env_dir_path, f"N{size}_gamma{gamma}.xml")
    gap_x, gap_y, gap_z = get_gap_position(SimulationConfig(env_path=env_path))

    return np.abs(positions[-1, 0]) > np.abs(gap_x)


"""

Returns the rate of successful traversal of a VPR of size 'size' through a gap with ratio 'gamma'
over 'num_runs' simulations. It searches for the .npy data files in 'dir_path'.

"""
def get_pass_rate(size: int, gamma: float, num_runs: int, dir_path: str = "results/compliance/"):
    
    passes = 0
    for run in range(1, num_runs + 1):
        file_name = f"size{size}_gamma{gamma}_{run}.npy"
        positions = np.load(os.path.join(dir_path, file_name))

        if crossed_gap(positions, size, gamma):
            passes += 1

    return passes / num_runs

        
"""

Returns the number of successful traversals of a VPR of size 'size' through a gap with ratio 'gamma'
over 'num_runs' simulations.

"""
def get_pass_count(size: int, gamma: float, num_runs: int = 1, dir_path: str = "results/compliance"):

    passes = 0
    for run in range(1, num_runs + 1):
        file_name = f"size{size}_gamma{gamma}_{run}.npy"
        positions = np.load(os.path.join(dir_path, file_name))

        if (crossed_gap(positions, size, gamma)):
            passes += 1

    return passes


if __name__ == "__main__":

    # Parameters
    FIT_CURVE = True 
    OUTPUT_FILE = Path("/Users/mateopatinohasbon/particle-robot-simulator/vector_images/compliance.eps")
    sizes = [16, 64, 100]
    gamma_values = {
            16: [1.0, 0.9, 0.88, 0.86, 0.84, 0.82, 0.8, 0.75, 0.7],
            64: [1.0, 0.9, 0.8, 0.7, 0.65, 0.625, 0.6, 0.575, 0.55, 0.5],
            100: [1.0, 0.9, 0.8, 0.7, 0.65, 0.625, 0.6, 0.575, 0.55, 0.5],
    }
    runs = {
            16: {"1.0": 10, "0.9": 10, "0.88": 20, "0.86": 20, "0.84": 20, "0.82": 20, "0.8": 20, "0.75": 10, "0.7": 10},
            64: {"1.0": 10, "0.9": 10, "0.8": 10, "0.7": 20, "0.65": 20, "0.625": 20, "0.6": 20, "0.575": 20, "0.55": 10, "0.5": 10},
            100: {"1.0": 10, "0.9": 10, "0.8": 20, "0.7": 20, "0.65": 20, "0.625": 20, "0.6": 20, "0.575": 20, "0.55": 10, "0.5": 10},
    } 
    fig, ax = plt.subplots(figsize=(7, 5))
    colors = {
        16: "tab:blue",
        64: "tab:red",
        100: "tab:green"
    }
    markers = {
        16: "s",
        64: "o",
        100: "^"
    }

    for size in sizes:
        gammas = gamma_values[size]

        # Probabilities
        probs = [get_pass_rate(size, gamma, runs[size][str(gamma)]) for gamma in gammas]
        ax.scatter(gammas, probs, s=50, c=colors[size], marker=markers[size], label=f"N={size}", zorder=4)

        # Fit curve
        if FIT_CURVE:
            gammas = np.array(gammas)
            passes = np.array([get_pass_count(size, gamma, runs[size][str(gamma)]) for gamma in gammas])
            trials = np.array([runs[size][str(gamma)] for gamma in gammas])

            endog = np.column_stack([passes, trials - passes])
            X = sm.add_constant(gammas)
            model = sm.GLM(endog, X, family=sm.families.Binomial())
            result = model.fit()
            save_regression_metadata(result, size)

            gamma_fit = np.linspace(min(gammas), max(gammas), 500)
            X_fit = sm.add_constant(gamma_fit)
            probs_fit = result.predict(X_fit) 
            
            ax.plot(gamma_fit, probs_fit, color=colors[size], linewidth=2, alpha=0.35, zorder=2)



    ax.set_ylabel("Successful traversal probability")
    ax.set_xlabel("Gap-to-length ratio, d/D")
    ax.set_xticks([0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1])

    ax.grid(alpha=0.3)
    ax.legend()

    plt.tight_layout()

    fig.savefig(OUTPUT_FILE, format="eps", bbox_inches="tight")
    fig.savefig(OUTPUT_FILE.with_suffix(".pdf"), bbox_inches="tight")

    
    print(f"Saved figure to:\n{OUTPUT_FILE}")

    plt.show()

