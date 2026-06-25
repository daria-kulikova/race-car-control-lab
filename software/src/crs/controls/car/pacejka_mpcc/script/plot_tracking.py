"""
Compare tracking quality (contouring error eC) between two runs:
one with GP correction and one without.

Usage:
    python3 /code/src/crs/controls/car/pacejka_mpcc/script/plot_tracking.py \
        --with-gp    /code/src/crs/controls/car/pacejka_mpcc/script/data/tracking_cm1_0555_3.csv \
        --without-gp /code/src/crs/controls/car/pacejka_mpcc/script/data/tracking_cm1_0555_2.csv \
        --residuals-gp /code/src/crs/controls/car/pacejka_mpcc/script/data/mpcc_residuals_cm1_0555_3.csv \
        --residuals-no-gp /code/src/crs/controls/car/pacejka_mpcc/script/data/mpcc_residuals_cm1_0555_2.csv
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--with-gp",        required=True, help="tracking CSV from run WITH GP")
parser.add_argument("--without-gp",     required=True, help="tracking CSV from run WITHOUT GP")
parser.add_argument("--residuals-gp",   default=None,  help="residuals CSV from run WITH GP (for eps_omega map)")
parser.add_argument("--residuals-no-gp",default=None,  help="residuals CSV from run WITHOUT GP")
parser.add_argument("--out", default=str(Path(__file__).parent / "tracking_comparison.png"))
args = parser.parse_args()


def load_residuals(path):
    """Load residuals CSV: vx,vy,omega,delta,T,eps_vx,eps_vy,eps_omega,pos_x,pos_y,theta"""
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    return data[:, 8], data[:, 9], data[:, 7]   # pos_x, pos_y, eps_omega


def load(path):
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    t      = data[:, 0] - data[0, 0]   # relative time
    eC     = data[:, 1]
    eL     = data[:, 2]
    pos_x  = data[:, 3]
    pos_y  = data[:, 4]
    ref_x  = data[:, 5]
    ref_y  = data[:, 6]
    lap    = data[:, 7].astype(int) if data.shape[1] > 7 else np.zeros(len(t), dtype=int)
    return t, eC, eL, pos_x, pos_y, ref_x, ref_y, lap


def mean_lap_time(t, lap):
    """Return mean lap time in seconds, or nan if fewer than 3 complete laps."""
    lap_starts = np.where(np.diff(lap) > 0)[0] + 1  # indices where lap counter increments
    if len(lap_starts) < 3:
        return float("nan")
    lap_times = np.diff(t[lap_starts])[1:]  # skip first lap (car may start mid-track)
    return lap_times.mean()


t_gp,    eC_gp,    eL_gp,    px_gp,    py_gp,    rx_gp,    ry_gp,    lap_gp    = load(args.with_gp)
t_nogp,  eC_nogp,  eL_nogp,  px_nogp,  py_nogp,  rx_nogp,  ry_nogp,  lap_nogp  = load(args.without_gp)

has_residuals = args.residuals_gp is not None and args.residuals_no_gp is not None
n_rows = 4 if has_residuals else 3
fig, axes = plt.subplots(n_rows, 2, figsize=(14, 5 * n_rows))
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

# ── |eL| histogram ────────────────────────────────────────────────────────────
ax = axes[1, 1]
bins = np.linspace(0, max(np.abs(eL_gp).max(), np.abs(eL_nogp).max()), 60)
ax.hist(np.abs(eL_nogp), bins=bins, alpha=0.6, color="tomato",    label=f"without GP  mean={np.abs(eL_nogp).mean():.4f} m")
ax.hist(np.abs(eL_gp),   bins=bins, alpha=0.6, color="steelblue", label=f"with GP     mean={np.abs(eL_gp).mean():.4f} m")
ax.set_xlabel("|eL| [m]")
ax.set_ylabel("count")
ax.set_title("Distribution of |lag error|")
ax.legend()

# ── Trajectory on track ───────────────────────────────────────────────────────
ax = axes[2, 0]
ax.plot(rx_gp, ry_gp, "k--", linewidth=1.0, alpha=0.5, label="reference")
ax.plot(px_nogp, py_nogp, color="tomato",    alpha=0.6, linewidth=0.8, label="without GP")
ax.plot(px_gp,   py_gp,   color="steelblue", alpha=0.6, linewidth=0.8, label="with GP")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_title("Trajectory on track")
ax.legend()
ax.set_aspect("equal")
axes[2, 1].axis("off")

# ── Summary stats ─────────────────────────────────────────────────────────────
print("\n── Summary ───────────────────────────────────────────")
print(f"{'Metric':<30} {'Without GP':>12} {'With GP':>12} {'Change':>10}")
lt_nogp = mean_lap_time(t_nogp, lap_nogp)
lt_gp   = mean_lap_time(t_gp,   lap_gp)
metrics = [
    ("mean |eC| [m]",    np.abs(eC_nogp).mean(),   np.abs(eC_gp).mean()),
    ("std  |eC| [m]",    np.abs(eC_nogp).std(),    np.abs(eC_gp).std()),
    ("95th pct |eC| [m]", np.percentile(np.abs(eC_nogp), 95), np.percentile(np.abs(eC_gp), 95)),
    ("mean |eL| [m]",     np.abs(eL_nogp).mean(),              np.abs(eL_gp).mean()),
    ("std  |eL| [m]",     np.abs(eL_nogp).std(),               np.abs(eL_gp).std()),
    ("95th pct |eL| [m]", np.percentile(np.abs(eL_nogp), 95),  np.percentile(np.abs(eL_gp), 95)),
    ("mean lap time [s]",  lt_nogp,                             lt_gp),
]
for name, v_nogp, v_gp in metrics:
    if np.isnan(v_nogp) or np.isnan(v_gp):
        print(f"{name:<30} {'N/A':>12} {'N/A':>12} {'N/A':>10}")
    else:
        change = (v_gp - v_nogp) / v_nogp * 100
        print(f"{name:<30} {v_nogp:>12.5f} {v_gp:>12.5f} {change:>+9.1f}%")

if has_residuals:
    rx_gp,   ry_gp,   eps_omega_gp    = load_residuals(args.residuals_gp)
    rx_nogp, ry_nogp, eps_omega_nogp  = load_residuals(args.residuals_no_gp)

    vmax = max(np.abs(eps_omega_gp).max(), np.abs(eps_omega_nogp).max())

    ax = axes[3, 0]
    sc = ax.scatter(rx_nogp, ry_nogp, c=eps_omega_nogp, cmap="RdBu_r",
                    vmin=-vmax, vmax=vmax, s=8, alpha=0.7)
    plt.colorbar(sc, ax=ax, label="eps_omega [m/s]")
    ax.set_title("eps_omega on track — WITHOUT GP")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_aspect("equal")

    ax = axes[3, 1]
    sc = ax.scatter(rx_gp, ry_gp, c=eps_omega_gp, cmap="RdBu_r",
                    vmin=-vmax, vmax=vmax, s=8, alpha=0.7)
    plt.colorbar(sc, ax=ax, label="eps_omega [m/s]")
    ax.set_title("eps_omega on track — WITH GP")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_aspect("equal")

plt.tight_layout()
plt.savefig(args.out, dpi=150)
print(f"\nSaved → {args.out}")
plt.show()
