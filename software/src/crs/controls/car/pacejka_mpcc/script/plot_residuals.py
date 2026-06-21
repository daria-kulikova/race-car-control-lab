"""
Plot model residuals from MPCC data log.

Shows that the Pacejka model error is systematic (not just noise),
which motivates learning a GP correction.

Usage:
    python3 plot_residuals.py --data /code/src/data/mpcc_residuals.csv
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--data", default="/code/src/data/mpcc_residuals.csv")
parser.add_argument("--out", default=str(Path(__file__).parent / "residuals.png"))
parser.add_argument("--vx-min", type=float, default=0.5)
args = parser.parse_args()

data = np.loadtxt(args.data, delimiter=",", skiprows=1)
mask = data[:, 0] >= args.vx_min
data = data[mask]

vx    = data[:, 0]
vy    = data[:, 1]
omega = data[:, 2]
eps_vx    = data[:, 5]
eps_vy    = data[:, 6]
eps_omega = data[:, 7]

fig, axes = plt.subplots(2, 3, figsize=(14, 8))
fig.suptitle("Pacejka model residuals (actual − predicted)", fontsize=13)

NAMES = ["eps_vx  [m/s²]", "eps_vy  [m/s²]", "eps_omega  [rad/s²]"]
RESIDS = [eps_vx, eps_vy, eps_omega]
COLORS = ["steelblue", "darkorange", "forestgreen"]

# Row 0: residual vs vx (shows state-dependent structure)
for i, (name, res, col) in enumerate(zip(NAMES, RESIDS, COLORS)):
    ax = axes[0, i]
    ax.scatter(vx, res, s=4, alpha=0.4, color=col)
    ax.axhline(0, color="black", linewidth=0.8, linestyle="--")
    ax.axhline(np.mean(res), color="red", linewidth=1.2, linestyle="-", label=f"mean={np.mean(res):.4f}")
    ax.set_xlabel("vx [m/s]")
    ax.set_ylabel(name)
    ax.set_title(f"{name} vs vx")
    ax.legend(fontsize=8)

# Row 1: histograms
for i, (name, res, col) in enumerate(zip(NAMES, RESIDS, COLORS)):
    ax = axes[1, i]
    ax.hist(res, bins=60, color=col, alpha=0.7, edgecolor="none")
    ax.axvline(0, color="black", linewidth=0.8, linestyle="--")
    ax.axvline(np.mean(res), color="red", linewidth=1.2, label=f"mean={np.mean(res):.4f}\nstd={np.std(res):.4f}")
    ax.set_xlabel(name)
    ax.set_ylabel("count")
    ax.set_title(f"Distribution of {name}")
    ax.legend(fontsize=8)

plt.tight_layout()
plt.savefig(args.out, dpi=150)
print(f"Saved → {args.out}")
plt.show()
