"""
Compare tracking quality (contouring error eC) between two runs:
one with GP correction and one without.

Usage:
    python3 plot_tracking.py \
        --with-gp    /code/src/data/tracking_gp.csv \
        --without-gp /code/src/data/tracking_no_gp.csv
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--with-gp",    required=True, help="tracking CSV from run WITH GP")
parser.add_argument("--without-gp", required=True, help="tracking CSV from run WITHOUT GP")
parser.add_argument("--out", default=str(Path(__file__).parent / "tracking_comparison.png"))
args = parser.parse_args()


def load(path):
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    t      = data[:, 0] - data[0, 0]   # relative time
    eC     = data[:, 1]
    eL     = data[:, 2]
    pos_x  = data[:, 3]
    pos_y  = data[:, 4]
    ref_x  = data[:, 5]
    ref_y  = data[:, 6]
    return t, eC, eL, pos_x, pos_y, ref_x, ref_y


t_gp,    eC_gp,    eL_gp,    px_gp,    py_gp,    rx_gp,    ry_gp    = load(args.with_gp)
t_nogp,  eC_nogp,  eL_nogp,  px_nogp,  py_nogp,  rx_nogp,  ry_nogp  = load(args.without_gp)

fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle("Tracking quality: with GP vs without GP", fontsize=13)

# ── eC over time ──────────────────────────────────────────────────────────────
ax = axes[0, 0]
ax.plot(t_nogp, eC_nogp, color="tomato",     alpha=0.7, linewidth=0.8, label="without GP")
ax.plot(t_gp,   eC_gp,   color="steelblue",  alpha=0.7, linewidth=0.8, label="with GP")
ax.axhline(0, color="black", linewidth=0.6, linestyle="--")
ax.set_xlabel("time [s]")
ax.set_ylabel("eC [m]")
ax.set_title("Contouring error over time")
ax.legend()

# ── |eC| histogram ────────────────────────────────────────────────────────────
ax = axes[0, 1]
bins = np.linspace(0, max(np.abs(eC_gp).max(), np.abs(eC_nogp).max()), 60)
ax.hist(np.abs(eC_nogp), bins=bins, alpha=0.6, color="tomato",    label=f"without GP  mean={np.abs(eC_nogp).mean():.4f} m")
ax.hist(np.abs(eC_gp),   bins=bins, alpha=0.6, color="steelblue", label=f"with GP     mean={np.abs(eC_gp).mean():.4f} m")
ax.set_xlabel("|eC| [m]")
ax.set_ylabel("count")
ax.set_title("Distribution of |contouring error|")
ax.legend()

# ── eL over time ──────────────────────────────────────────────────────────────
ax = axes[1, 0]
ax.plot(t_nogp, eL_nogp, color="tomato",    alpha=0.7, linewidth=0.8, label="without GP")
ax.plot(t_gp,   eL_gp,   color="steelblue", alpha=0.7, linewidth=0.8, label="with GP")
ax.axhline(0, color="black", linewidth=0.6, linestyle="--")
ax.set_xlabel("time [s]")
ax.set_ylabel("eL [m]")
ax.set_title("Lag error over time")
ax.legend()

# ── Trajectory on track ───────────────────────────────────────────────────────
ax = axes[1, 1]
ax.plot(rx_gp, ry_gp, "k--", linewidth=1.0, alpha=0.5, label="reference")
ax.plot(px_nogp, py_nogp, color="tomato",    alpha=0.6, linewidth=0.8, label="without GP")
ax.plot(px_gp,   py_gp,   color="steelblue", alpha=0.6, linewidth=0.8, label="with GP")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_title("Trajectory on track")
ax.legend()
ax.set_aspect("equal")

# ── Summary stats ─────────────────────────────────────────────────────────────
print("\n── Summary ───────────────────────────────────────────")
print(f"{'Metric':<30} {'Without GP':>12} {'With GP':>12} {'Change':>10}")
metrics = [
    ("mean |eC| [m]",    np.abs(eC_nogp).mean(),   np.abs(eC_gp).mean()),
    ("std  |eC| [m]",    np.abs(eC_nogp).std(),    np.abs(eC_gp).std()),
    ("95th pct |eC| [m]", np.percentile(np.abs(eC_nogp), 95), np.percentile(np.abs(eC_gp), 95)),
    ("mean |eL| [m]",    np.abs(eL_nogp).mean(),   np.abs(eL_gp).mean()),
]
for name, v_nogp, v_gp in metrics:
    change = (v_gp - v_nogp) / v_nogp * 100
    print(f"{name:<30} {v_nogp:>12.5f} {v_gp:>12.5f} {change:>+9.1f}%")

plt.tight_layout()
plt.savefig(args.out, dpi=150)
print(f"\nSaved → {args.out}")
plt.show()
