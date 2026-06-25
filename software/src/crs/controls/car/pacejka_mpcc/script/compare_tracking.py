"""
Compare tracking metrics across multiple datasets without plots.

Discovers all files matching <base>_0.csv, <base>_1.csv, ... automatically.

Two comparison modes (--mode):
  baseline    : each dataset vs dataset 0  → pairs 0-1, 0-2, 0-3, ...
  consecutive : each dataset vs previous   → pairs 1-2, 2-3, 3-4, ... (0 ignored)

Output: CSV with one row per (pair, metric).

Usage:
    python3 src/crs/controls/car/pacejka_mpcc/script/compare_tracking.py --data src/crs/controls/car/pacejka_mpcc/script/data/tracking_cm1_0555_0.csv --mode baseline --out results.csv
    python3 compare_tracking.py --data tracking_0.csv --mode consecutive
"""

import argparse
import math
import numpy as np
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--data", required=True,
                    help="Path to the _0.csv tracking file")
parser.add_argument("--mode", choices=["baseline", "consecutive"], default="baseline",
                    help="baseline: all vs dataset 0; consecutive: i vs i-1 (from pair 1-2 onward)")
parser.add_argument("--out", default=None,
                    help="Output CSV path (default: <base>_comparison_<mode>.csv)")
args = parser.parse_args()

data_path = Path(args.data)
if not data_path.name.endswith("_0.csv"):
    raise ValueError("--data must point to the _0.csv file, e.g. tracking_0.csv")

base = str(data_path)[: -len("_0.csv")]

# Discover all available files
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
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    t   = data[:, 0] - data[0, 0]
    eC  = data[:, 1]
    eL  = data[:, 2]
    lap = data[:, 7].astype(int) if data.shape[1] > 7 else np.zeros(len(t), dtype=int)
    return t, eC, eL, lap


def mean_lap_time(t, lap):
    lap_starts = np.where(np.diff(lap) > 0)[0] + 1
    if len(lap_starts) < 3:
        return float("nan")
    return np.diff(t[lap_starts])[1:].mean()


def compute_metrics(t, eC, eL, lap):
    return {
        "mean |eC| [m]":      float(np.abs(eC).mean()),
        "std |eC| [m]":       float(np.abs(eC).std()),
        "95th pct |eC| [m]":  float(np.percentile(np.abs(eC), 95)),
        "mean |eL| [m]":      float(np.abs(eL).mean()),
        "std |eL| [m]":       float(np.abs(eL).std()),
        "95th pct |eL| [m]":  float(np.percentile(np.abs(eL), 95)),
        "mean lap time [s]":  mean_lap_time(t, lap),
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

# Build output rows
header = "pair,metric,dataset_a,dataset_b,change_pct"
rows = [header]

for a, b in pairs:
    ma, mb = cache[a], cache[b]
    for metric in ma:
        va, vb = ma[metric], mb[metric]
        if math.isnan(va) or math.isnan(vb):
            change_str = "N/A"
        elif va == 0.0:
            change_str = "N/A"
        else:
            change_str = f"{(vb - va) / abs(va) * 100:.2f}"
        va_str = f"{va:.6f}" if not math.isnan(va) else "N/A"
        vb_str = f"{vb:.6f}" if not math.isnan(vb) else "N/A"
        rows.append(f"{a}-{b},{metric},{va_str},{vb_str},{change_str}")

out_path = Path(args.out) if args.out else Path(f"{base}_comparison_{args.mode}.csv")
out_path.write_text("\n".join(rows) + "\n")
print(f"\nSaved → {out_path}  ({len(pairs)} pairs, {len(rows) - 1} rows)")
