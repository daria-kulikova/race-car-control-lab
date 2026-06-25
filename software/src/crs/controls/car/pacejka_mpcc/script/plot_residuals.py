"""
Plot model residuals from MPCC data log.

Saves one figure per feature (vx, vy, omega, delta, T) showing how each
residual depends on that feature, plus a histogram figure.

Usage:
    python3 /code/src/crs/controls/car/pacejka_mpcc/script/plot_residuals.py
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--data", default="/code/src/data/mpcc_residuals.csv")
parser.add_argument("--out-dir", default=str(Path(__file__).parent))
parser.add_argument("--vx-min", type=float, default=0.5)
parser.add_argument("--lf",    type=float, default=0.04448988, help="front axle distance [m]")
parser.add_argument("--lr",    type=float, default=0.04866242, help="rear axle distance [m]")
args = parser.parse_args()

out_dir = Path(args.out_dir)

data = np.loadtxt(args.data, delimiter=",", skiprows=1)
data = data[data[:, 0] >= args.vx_min]

vx    = data[:, 0]
vy    = data[:, 1]
omega = data[:, 2]
delta = data[:, 3]
T     = data[:, 4]
eps_vx    = data[:, 5]
eps_vy    = data[:, 6]
eps_omega = data[:, 7]

vx_safe = np.maximum(vx, 0.1)
beta    = np.arctan2(vy, vx_safe)
alpha_f = delta - np.arctan2(vy + args.lf * omega, vx_safe)
alpha_r = -np.arctan2(vy - args.lr * omega, vx_safe)

FEATURES = [
    ("vx",      vx,      "vx [m/s]"),
    ("vy",      vy,      "vy [m/s]"),
    ("omega",   omega,   "omega [rad/s]"),
    ("delta",   delta,   "delta [rad]"),
    ("T",       T,       "T [–]"),
    ("beta",    beta,    "beta [rad]"),
    ("alpha_f", alpha_f, "alpha_f [rad]"),
    ("alpha_r", alpha_r, "alpha_r [rad]"),
]
RESIDUALS = [
    ("eps_vx",    eps_vx,    "eps_vx [m/s²]",    "steelblue"),
    ("eps_vy",    eps_vy,    "eps_vy [m/s²]",    "darkorange"),
    ("eps_omega", eps_omega, "eps_omega [rad/s²]","forestgreen"),
]

# ── One figure per feature ────────────────────────────────────────────────────
for feat_key, feat_vals, feat_label in FEATURES:
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    fig.suptitle(f"Residuals vs {feat_label}", fontsize=12)

    for ax, (res_key, res_vals, res_label, col) in zip(axes, RESIDUALS):
        sc = ax.scatter(feat_vals, res_vals, s=5, alpha=0.35,
                        c=np.abs(omega), cmap="plasma")
        plt.colorbar(sc, ax=ax, label="|omega| [rad/s]")
        ax.axhline(0, color="black", linewidth=0.8, linestyle="--")
        ax.axhline(np.mean(res_vals), color="red", linewidth=1.0,
                   label=f"mean={np.mean(res_vals):.5f}")
        ax.set_xlabel(feat_label)
        ax.set_ylabel(res_label)
        ax.set_title(f"{res_label} vs {feat_label}")
        ax.legend(fontsize=8)

    plt.tight_layout()
    path = out_dir / f"residuals_vs_{feat_key}.png"
    plt.savefig(path, dpi=150)
    print(f"Saved → {path}")
    plt.close()

# ── Histograms ────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(1, 3, figsize=(14, 4))
fig.suptitle("Distribution of residuals", fontsize=12)

for ax, (res_key, res_vals, res_label, col) in zip(axes, RESIDUALS):
    ax.hist(res_vals, bins=80, color=col, alpha=0.75, edgecolor="none")
    ax.axvline(0, color="black", linewidth=0.8, linestyle="--")
    ax.axvline(np.mean(res_vals), color="red", linewidth=1.2,
               label=f"mean={np.mean(res_vals):.5f}\nstd={np.std(res_vals):.5f}")
    ax.set_xlabel(res_label)
    ax.set_ylabel("count")
    ax.set_title(f"Distribution of {res_label}")
    ax.legend(fontsize=8)

plt.tight_layout()
path = out_dir / "residuals_histograms.png"
plt.savefig(path, dpi=150)
print(f"Saved → {path}")
plt.close()

print("\nDone.")
