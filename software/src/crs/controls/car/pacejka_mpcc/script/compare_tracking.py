"""
Compare tracking metrics across multiple datasets without plots.

Discovers all files matching <base>_0.csv, <base>_1.csv, ... automatically.

Three comparison modes (--mode):
  baseline    : each dataset vs dataset 0  -> pairs 0-1, 0-2, 0-3, ...
  consecutive : each dataset vs previous   -> pairs 1-2, 2-3, 3-4, ... (0 ignored)
  none        : compare exactly two files specified with --file-a and --file-b

Output: human-readable text file, one section per pair.

Usage:
    python3 compare_tracking.py --data tracking_cm1_0555_0.csv --mode baseline
    python3 compare_tracking.py --data tracking_cm1_0555_0.csv --mode consecutive --out results.txt
    python3 compare_tracking.py --mode none --file-a data/run_a.csv --file-b data/run_b.csv --out comparison.txt
"""

import argparse
import math
import numpy as np
from pathlib import Path

parser = argparse.ArgumentParser()
DATA_DIR    = Path("/code/src/crs/controls/car/pacejka_mpcc/script/data")
RESULTS_DIR = Path("/code/src/crs/controls/car/pacejka_mpcc/script/results")

parser.add_argument("--data", default="mpcc_log_cm1_0555_0.csv",
                    help="Filename of the _0.csv log file inside DATA_DIR (baseline/consecutive modes)")
parser.add_argument("--mode", choices=["baseline", "consecutive", "none"], default="baseline",
                    help="baseline: all vs dataset 0; consecutive: i vs i-1; none: two explicit files")
parser.add_argument("--file-a", default=None,
                    help="[none mode] path to file A (absolute, or relative to DATA_DIR)")
parser.add_argument("--file-b", default=None,
                    help="[none mode] path to file B (absolute, or relative to DATA_DIR)")
parser.add_argument("--out", default=None,
                    help="Output filename inside RESULTS_DIR (default: compare_<mode>.txt)")
args = parser.parse_args()


def resolve(p: str) -> Path:
    """Accept absolute path or filename relative to DATA_DIR."""
    path = Path(p)
    return path if path.is_absolute() else DATA_DIR / path


# ── Pair mode: skip auto-discovery entirely ───────────────────────────────────
if args.mode == "none":
    if not args.file_a or not args.file_b:
        parser.error("--mode none requires both --file-a and --file-b")
    path_a = resolve(args.file_a)
    path_b = resolve(args.file_b)
    for p in (path_a, path_b):
        if not p.exists():
            raise FileNotFoundError(p)
    label_a = path_a.stem
    label_b = path_b.stem
    print(f"None mode: A = {path_a.name}  vs  B = {path_b.name}")

else:
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
    t         = data[:, 0] - data[0, 0]
    eps_vx    = data[:, 6]
    eps_vy    = data[:, 7]
    eps_omega = data[:, 8]
    eC        = data[:, 9]
    eL        = data[:, 10]
    lap       = data[:, 15].astype(int)
    return t, eC, eL, lap, eps_vx, eps_vy, eps_omega


def mean_lap_time(t, lap):
    lap = lap.astype(int)
    diff = np.diff(lap)
    lap_start_idx = np.where(diff > 0)[0] + 1
    reset_idx     = np.where(diff < 0)[0] + 1

    boundaries = [0] + list(reset_idx) + [len(lap)]
    intervals = []
    for lo, hi in zip(boundaries[:-1], boundaries[1:]):
        grp = lap_start_idx[(lap_start_idx >= lo) & (lap_start_idx < hi)]
        for i in range(0, len(grp) - 1):
            intervals.append(t[grp[i + 1]] - t[grp[i]])

    return float(np.mean(intervals)) if intervals else float("nan")


def compute_metrics(t, eC, eL, lap, eps_vx, eps_vy, eps_omega):
    m = {
        "mean |eC| [m]":     float(np.abs(eC).mean()),
        "std  |eC| [m]":     float(np.abs(eC).std()),
        "95th |eC| [m]":     float(np.percentile(np.abs(eC), 95)),
        "mean |eL| [m]":     float(np.abs(eL).mean()),
        "std  |eL| [m]":     float(np.abs(eL).std()),
        "95th |eL| [m]":     float(np.percentile(np.abs(eL), 95)),
        "mean lap time [s]": mean_lap_time(t, lap),
    }
    for name, arr in [("eps_vx", eps_vx), ("eps_vy", eps_vy), ("eps_omega", eps_omega)]:
        m[f"mean |{name}|"] = float(np.abs(arr).mean())
        m[f"std  |{name}|"] = float(np.abs(arr).std())
        m[f"95th |{name}|"] = float(np.percentile(np.abs(arr), 95))
    return m


# Determine pairs and load data
if args.mode == "none":
    t_a, eC_a, eL_a, lap_a, evx_a, evy_a, eom_a = load(path_a)
    t_b, eC_b, eL_b, lap_b, evx_b, evy_b, eom_b = load(path_b)
    cache = {
        "A": compute_metrics(t_a, eC_a, eL_a, lap_a, evx_a, evy_a, eom_a),
        "B": compute_metrics(t_b, eC_b, eL_b, lap_b, evx_b, evy_b, eom_b),
    }
    pairs = [("A", "B")]
    print(f"  loaded A: {len(eC_a)} points")
    print(f"  loaded B: {len(eC_b)} points")
else:
    if args.mode == "baseline":
        pairs = [(0, j) for j in sorted(files) if j > 0]
    else:
        idx = sorted(k for k in files if k >= 1)
        pairs = [(idx[k], idx[k + 1]) for k in range(len(idx) - 1)]

    needed = {i for pair in pairs for i in pair}
    cache = {}
    for idx in sorted(needed):
        t, eC, eL, lap, eps_vx, eps_vy, eps_omega = load(files[idx])
        cache[idx] = compute_metrics(t, eC, eL, lap, eps_vx, eps_vy, eps_omega)
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


if args.mode == "none":
    header_base = f"{path_a.stem} vs {path_b.stem}"
else:
    header_base = Path(base).name

lines = []
lines.append(f"Tracking comparison  |  mode: {args.mode}  |  base: {header_base}")
lines.append("")

col_hdr = f"  {'Metric':<{W_METRIC}}  {'Dataset A':>{W_VAL}}  {'Dataset B':>{W_VAL}}  {'Change':>{W_CHG}}"

EPS_GROUPS = [
    {"mean |eps_vx|", "std  |eps_vx|", "95th |eps_vx|"},
    {"mean |eps_vy|", "std  |eps_vy|", "95th |eps_vy|"},
    {"mean |eps_omega|", "std  |eps_omega|", "95th |eps_omega|"},
]

def eps_group_end(metric):
    """Return True if this metric is the last in its eps group."""
    for g in EPS_GROUPS:
        if metric in g:
            return metric.startswith("95th")
    return False

for a, b in pairs:
    ma, mb = cache[a], cache[b]
    lines.append(HEAVY)
    if args.mode == "none":
        lines.append(f"  {path_a.name}  vs  {path_b.name}")
    else:
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
        if eps_group_end(metric):
            lines.append("")
    lines.append("")

out_name = args.out if args.out else f"compare_{args.mode}.txt"
out_path = RESULTS_DIR / out_name
out_path.parent.mkdir(parents=True, exist_ok=True)
out_path.write_text("\n".join(lines) + "\n")
print(f"\nSaved -> {out_path}  ({len(pairs)} pairs)")
