"""
Compare tracking quality between two runs.

Modes:
  baseline  compare dataset 0 vs the last found (default)
  last      compare second-to-last vs last
  none      specify datasets explicitly via --indices or --file-a/--file-b

Usage examples:
    python3 plot_tracking.py
    python3 plot_tracking.py --mode last
    python3 plot_tracking.py --mode none --indices 1 4
    python3 plot_tracking.py --mode none --file-a tracking_other_0.csv --file-b tracking_other_3.csv
"""

import argparse
import re
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

DATA_DIR       = Path("/code/src/crs/controls/car/pacejka_mpcc/script/data")
PLOTS_DIR      = Path("/code/src/crs/controls/car/pacejka_mpcc/script/plots")
TRACKING_BASE  = "tracking_cm1_0555"
RESIDUALS_BASE = "mpcc_residuals_cm1_0555"

parser = argparse.ArgumentParser()
parser.add_argument("--mode", choices=["baseline", "last", "none"], default="baseline",
                    help="how to select the two datasets")
parser.add_argument("--indices", nargs=2, type=int, metavar=("A", "B"),
                    help="[none mode] indices appended to default base names")
parser.add_argument("--file-a", default=None,
                    help="[none mode] explicit filename for dataset A (inside DATA_DIR)")
parser.add_argument("--file-b", default=None,
                    help="[none mode] explicit filename for dataset B (inside DATA_DIR)")
parser.add_argument("--out", default=None,
                    help="output filename (default: tracking_comparison_A_B.png in PLOTS_DIR)")
args = parser.parse_args()


def discover_indices(base: str):
    i, found = 0, []
    while (DATA_DIR / f"{base}_{i}.csv").exists():
        found.append(i)
        i += 1
    return found


def tracking_path(name: str) -> Path:
    return DATA_DIR / name


def residuals_name(tracking_name: str) -> str:
    return tracking_name.replace("tracking_", "mpcc_residuals_", 1)


# Resolve which two files to compare
if args.mode == "baseline":
    indices = discover_indices(TRACKING_BASE)
    if len(indices) < 2:
        raise FileNotFoundError(f"Need at least 2 {TRACKING_BASE}_*.csv files in {DATA_DIR}")
    idx_a, idx_b = indices[0], indices[-1]
    name_a = f"{TRACKING_BASE}_{idx_a}.csv"
    name_b = f"{TRACKING_BASE}_{idx_b}.csv"

elif args.mode == "last":
    indices = discover_indices(TRACKING_BASE)
    if len(indices) < 2:
        raise FileNotFoundError(f"Need at least 2 {TRACKING_BASE}_*.csv files in {DATA_DIR}")
    idx_a, idx_b = indices[-2], indices[-1]
    name_a = f"{TRACKING_BASE}_{idx_a}.csv"
    name_b = f"{TRACKING_BASE}_{idx_b}.csv"

else:  # none
    if args.indices:
        idx_a, idx_b = args.indices
        name_a = f"{TRACKING_BASE}_{idx_a}.csv"
        name_b = f"{TRACKING_BASE}_{idx_b}.csv"
    elif args.file_a and args.file_b:
        idx_a, idx_b = "A", "B"
        name_a, name_b = args.file_a, args.file_b
    else:
        parser.error("--mode none requires either --indices A B or both --file-a and --file-b")

path_a = DATA_DIR / name_a
path_b = DATA_DIR / name_b
res_path_a = DATA_DIR / residuals_name(name_a)
res_path_b = DATA_DIR / residuals_name(name_b)
has_residuals = res_path_a.exists() and res_path_b.exists()

label_a = f"dataset {idx_a}"
label_b = f"dataset {idx_b}"
out_path = args.out if args.out else str(PLOTS_DIR / f"tracking_comparison_{idx_a}_{idx_b}.png")
print(f"Comparing:  A = {name_a}  vs  B = {name_b}")
if has_residuals:
    print(f"Residuals:  {res_path_a.name}  vs  {res_path_b.name}")


def load_residuals(path):
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    return data[:, 8], data[:, 9], data[:, 7]   # pos_x, pos_y, eps_omega


def load(path):
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    t     = data[:, 0] - data[0, 0]
    eC    = data[:, 1]
    eL    = data[:, 2]
    pos_x = data[:, 3]
    pos_y = data[:, 4]
    ref_x = data[:, 5]
    ref_y = data[:, 6]
    lap   = data[:, 7].astype(int) if data.shape[1] > 7 else np.zeros(len(t), dtype=int)
    return t, eC, eL, pos_x, pos_y, ref_x, ref_y, lap


def mean_lap_time(t, lap):
    lap_starts = np.where(np.diff(lap) > 0)[0] + 1
    if len(lap_starts) < 3:
        return float("nan")
    return np.diff(t[lap_starts])[1:].mean()


t_a, eC_a, eL_a, px_a, py_a, rx_a, ry_a, lap_a = load(path_a)
t_b, eC_b, eL_b, px_b, py_b, rx_b, ry_b, lap_b = load(path_b)

n_rows = 4 if has_residuals else 3
fig, axes = plt.subplots(n_rows, 2, figsize=(14, 5 * n_rows))
fig.suptitle(f"Tracking quality: {label_a} vs {label_b}", fontsize=13)

# ── eC over time ──────────────────────────────────────────────────────────────
ax = axes[0, 0]
ax.plot(t_a, eC_a, color="tomato",    alpha=0.7, linewidth=0.8, label=label_a)
ax.plot(t_b, eC_b, color="steelblue", alpha=0.7, linewidth=0.8, label=label_b)
ax.axhline(0, color="black", linewidth=0.6, linestyle="--")
ax.set_xlabel("time [s]")
ax.set_ylabel("eC [m]")
ax.set_title("Contouring error over time")
ax.legend()

# ── |eC| histogram ────────────────────────────────────────────────────────────
ax = axes[0, 1]
bins = np.linspace(0, max(np.abs(eC_a).max(), np.abs(eC_b).max()), 60)
ax.hist(np.abs(eC_a), bins=bins, alpha=0.6, color="tomato",    label=f"{label_a}  mean={np.abs(eC_a).mean():.4f} m")
ax.hist(np.abs(eC_b), bins=bins, alpha=0.6, color="steelblue", label=f"{label_b}  mean={np.abs(eC_b).mean():.4f} m")
ax.set_xlabel("|eC| [m]")
ax.set_ylabel("count")
ax.set_title("Distribution of |contouring error|")
ax.legend()

# ── eL over time ──────────────────────────────────────────────────────────────
ax = axes[1, 0]
ax.plot(t_a, eL_a, color="tomato",    alpha=0.7, linewidth=0.8, label=label_a)
ax.plot(t_b, eL_b, color="steelblue", alpha=0.7, linewidth=0.8, label=label_b)
ax.axhline(0, color="black", linewidth=0.6, linestyle="--")
ax.set_xlabel("time [s]")
ax.set_ylabel("eL [m]")
ax.set_title("Lag error over time")
ax.legend()

# ── |eL| histogram ────────────────────────────────────────────────────────────
ax = axes[1, 1]
bins = np.linspace(0, max(np.abs(eL_a).max(), np.abs(eL_b).max()), 60)
ax.hist(np.abs(eL_a), bins=bins, alpha=0.6, color="tomato",    label=f"{label_a}  mean={np.abs(eL_a).mean():.4f} m")
ax.hist(np.abs(eL_b), bins=bins, alpha=0.6, color="steelblue", label=f"{label_b}  mean={np.abs(eL_b).mean():.4f} m")
ax.set_xlabel("|eL| [m]")
ax.set_ylabel("count")
ax.set_title("Distribution of |lag error|")
ax.legend()

# ── Trajectory on track ───────────────────────────────────────────────────────
ax = axes[2, 0]
ax.plot(rx_b, ry_b, "k--", linewidth=1.0, alpha=0.5, label="reference")
ax.plot(px_a, py_a, color="tomato",    alpha=0.6, linewidth=0.8, label=label_a)
ax.plot(px_b, py_b, color="steelblue", alpha=0.6, linewidth=0.8, label=label_b)
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_title("Trajectory on track")
ax.legend()
ax.set_aspect("equal")
axes[2, 1].axis("off")

# ── Summary stats ─────────────────────────────────────────────────────────────
print(f"\n{'Metric':<30} {label_a:>12} {label_b:>12} {'Change':>10}")
print("-" * 68)
lt_a = mean_lap_time(t_a, lap_a)
lt_b = mean_lap_time(t_b, lap_b)
metrics = [
    ("mean |eC| [m]",      np.abs(eC_a).mean(),               np.abs(eC_b).mean()),
    ("std  |eC| [m]",      np.abs(eC_a).std(),                np.abs(eC_b).std()),
    ("95th pct |eC| [m]",  np.percentile(np.abs(eC_a), 95),   np.percentile(np.abs(eC_b), 95)),
    ("mean |eL| [m]",      np.abs(eL_a).mean(),               np.abs(eL_b).mean()),
    ("std  |eL| [m]",      np.abs(eL_a).std(),                np.abs(eL_b).std()),
    ("95th pct |eL| [m]",  np.percentile(np.abs(eL_a), 95),   np.percentile(np.abs(eL_b), 95)),
    ("mean lap time [s]",  lt_a,                              lt_b),
]
for name, va, vb in metrics:
    if np.isnan(va) or np.isnan(vb):
        print(f"{name:<30} {'N/A':>12} {'N/A':>12} {'N/A':>10}")
    else:
        change = (vb - va) / va * 100
        print(f"{name:<30} {va:>12.5f} {vb:>12.5f} {change:>+9.1f}%")

if has_residuals:
    rx_a, ry_a, eps_omega_a = load_residuals(res_path_a)
    rx_b, ry_b, eps_omega_b = load_residuals(res_path_b)

    vmax = max(np.abs(eps_omega_a).max(), np.abs(eps_omega_b).max())

    ax = axes[3, 0]
    sc = ax.scatter(rx_a, ry_a, c=eps_omega_a, cmap="RdBu_r",
                    vmin=-vmax, vmax=vmax, s=8, alpha=0.7)
    plt.colorbar(sc, ax=ax, label="eps_omega [m/s]")
    ax.set_title(f"eps_omega on track — {label_a}")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_aspect("equal")

    ax = axes[3, 1]
    sc = ax.scatter(rx_b, ry_b, c=eps_omega_b, cmap="RdBu_r",
                    vmin=-vmax, vmax=vmax, s=8, alpha=0.7)
    plt.colorbar(sc, ax=ax, label="eps_omega [m/s]")
    ax.set_title(f"eps_omega on track — {label_b}")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_aspect("equal")

PLOTS_DIR.mkdir(parents=True, exist_ok=True)
plt.tight_layout()
plt.savefig(out_path, dpi=150)
print(f"\nSaved -> {out_path}")
plt.show()
