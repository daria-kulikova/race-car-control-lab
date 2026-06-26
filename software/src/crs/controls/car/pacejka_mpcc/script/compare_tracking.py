"""
Compare tracking metrics across multiple datasets without plots.

Discovers all files matching <base>_0.csv, <base>_1.csv, ... automatically.

Two comparison modes (--mode):
  baseline    : each dataset vs dataset 0  -> pairs 0-1, 0-2, 0-3, ...
  consecutive : each dataset vs previous   -> pairs 1-2, 2-3, 3-4, ... (0 ignored)

Output: human-readable text file, one section per pair.

Usage:
    python3 compare_tracking.py --data tracking_cm1_0555_0.csv --mode baseline
    python3 compare_tracking.py --data tracking_cm1_0555_0.csv --mode consecutive --out results.txt
"""

import argparse
import math
import numpy as np
from pathlib import Path

parser = argparse.ArgumentParser()
DATA_DIR    = Path("/code/src/crs/controls/car/pacejka_mpcc/script/data")
RESULTS_DIR = Path("/code/src/crs/controls/car/pacejka_mpcc/script/results")

parser.add_argument("--data", default="mpcc_log_cm1_0555_0.csv",
                    help="Filename of the _0.csv log file inside DATA_DIR")
parser.add_argument("--mode", choices=["baseline", "consecutive"], default="baseline",
                    help="baseline: all vs dataset 0; consecutive: i vs i-1 (from pair 1-2 onward)")
parser.add_argument("--out", default=None,
                    help="Output filename inside RESULTS_DIR (default: compare_<mode>.txt)")
args = parser.parse_args()

data_path = DATA_DIR / args.data
if not data_path.name.endswith("_0.csv"):
    raise ValueError("--data must be a _0.csv filename, e.g. tracking_cm1_0555_0.csv")

base = str(data_path)[: -len("_0.csv")]

# Discover consecutive files starting from 0
files = {}
i = 0
while True:
    p = Path(f"{base}_{i}.csv")
    if p.exists():
        files[i] = p
        i += 1
    else:
        break

if len(files) < 2:
    raise FileNotFoundError(f"Need at least 2 files matching {base}_*.csv, found {len(files)}")

print(f"Found {len(files)} datasets: {sorted(files)}")


def load(path: Path):
    # columns: t,vx,vy,omega,delta,T,eps_vx,eps_vy,eps_omega,eC,eL,pos_x,pos_y,ref_x,ref_y,lap,theta
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    t   = data[:, 0] - data[0, 0]
    eC  = data[:, 9]
    eL  = data[:, 10]
    lap = data[:, 15].astype(int)
    return t, eC, eL, lap


def mean_lap_time(t, lap):
    lap = lap.astype(int)
    diff = np.diff(lap)
    lap_start_idx = np.where(diff > 0)[0] + 1
    reset_idx     = np.where(diff < 0)[0] + 1

    boundaries = [0] + list(reset_idx) + [len(lap)]
    intervals = []
    for lo, hi in zip(boundaries[:-1], boundaries[1:]):
        grp = lap_start_idx[(lap_start_idx >= lo) & (lap_start_idx < hi)]
        for i in range(1, len(grp) - 1):
            intervals.append(t[grp[i + 1]] - t[grp[i]])

    return float(np.mean(intervals)) if intervals else float("nan")


def compute_metrics(t, eC, eL, lap):
    return {
        "mean |eC| [m]":     float(np.abs(eC).mean()),
        "std  |eC| [m]":     float(np.abs(eC).std()),
        "95th |eC| [m]":     float(np.percentile(np.abs(eC), 95)),
        "mean |eL| [m]":     float(np.abs(eL).mean()),
        "std  |eL| [m]":     float(np.abs(eL).std()),
        "95th |eL| [m]":     float(np.percentile(np.abs(eL), 95)),
        "mean lap time [s]": mean_lap_time(t, lap),
    }


# Determine pairs
if args.mode == "baseline":
    pairs = [(0, j) for j in sorted(files) if j > 0]
else:
    idx = sorted(k for k in files if k >= 1)
    pairs = [(idx[k], idx[k + 1]) for k in range(len(idx) - 1)]

# Load only the datasets we need
needed = {i for pair in pairs for i in pair}
cache = {}
for idx in sorted(needed):
    t, eC, eL, lap = load(files[idx])
    cache[idx] = compute_metrics(t, eC, eL, lap)
    print(f"  loaded dataset {idx}: {len(eC)} points")

# Layout constants
W_METRIC = 22
W_VAL = 12
W_CHG = 11
ROW_W = W_METRIC + 2 * W_VAL + W_CHG + 8
HEAVY = "=" * ROW_W
LIGHT = "-" * ROW_W


def fmt_val(v):
    return f"{v:.5f}" if not math.isnan(v) else "N/A"


def fmt_change(va, vb):
    if math.isnan(va) or math.isnan(vb) or va == 0.0:
        return "N/A"
    pct = (vb - va) / abs(va) * 100
    arrow = " ^" if pct > 0 else " v"
    sign = "+" if pct >= 0 else ""
    return f"{sign}{pct:.1f}%{arrow}"


lines = []
lines.append(f"Tracking comparison  |  mode: {args.mode}  |  base: {Path(base).name}")
lines.append("")

col_hdr = f"  {'Metric':<{W_METRIC}}  {'Dataset A':>{W_VAL}}  {'Dataset B':>{W_VAL}}  {'Change':>{W_CHG}}"

for a, b in pairs:
    ma, mb = cache[a], cache[b]
    lines.append(HEAVY)
    lines.append(f"  Pair {a} -> {b}   (A = dataset {a},  B = dataset {b})")
    lines.append(HEAVY)
    lines.append(col_hdr)
    lines.append("  " + LIGHT)
    for metric, va in ma.items():
        vb = mb[metric]
        change = fmt_change(va, vb)
        lines.append(
            f"  {metric:<{W_METRIC}}  {fmt_val(va):>{W_VAL}}  {fmt_val(vb):>{W_VAL}}  {change:>{W_CHG}}"
        )
    lines.append("")

out_name = args.out if args.out else f"compare_{args.mode}.txt"
out_path = RESULTS_DIR / out_name
out_path.parent.mkdir(parents=True, exist_ok=True)
out_path.write_text("\n".join(lines) + "\n")
print(f"\nSaved -> {out_path}  ({len(pairs)} pairs)")
