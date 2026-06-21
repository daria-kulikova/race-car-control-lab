"""
Convert mpcc_residuals.csv to the format expected by gp_training.ipynb.

The notebook expects:
  x_data.csv — 12 columns: [x, y, yaw, vx, vy, omega, T, delta, theta, dT, ddelta, dtheta]
                but only uses indices 3-7 = [vx, vy, omega, T, delta]
  y_data.csv — 3 columns: [eps_vx, eps_vy, eps_omega]

Usage:
    python3 prepare_notebook_data.py
"""

import argparse
import numpy as np
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--data", default="/code/src/data/mpcc_residuals.csv")
parser.add_argument("--out-dir", default="/code/src/ros4crs/rospy_controllers/data")
parser.add_argument("--vx-min", type=float, default=0.5)
parser.add_argument("--max-points", type=int, default=2000)
parser.add_argument("--seed", type=int, default=42)
args = parser.parse_args()

out_dir = Path(args.out_dir)
out_dir.mkdir(parents=True, exist_ok=True)

# Load our CSV: [vx, vy, omega, delta, T, eps_vx, eps_vy, eps_omega]
data = np.loadtxt(args.data, delimiter=",", skiprows=1)
data = data[data[:, 0] >= args.vx_min]

if len(data) > args.max_points:
    rng = np.random.default_rng(args.seed)
    idx = rng.choice(len(data), args.max_points, replace=False)
    data = data[idx]

print(f"Points after filtering: {len(data)}")

# Build x_data with 12 columns — zeros everywhere except indices 3-7
# Notebook input_selection = [0,0,0,1,1,1,1,1,0,0,0,0]
# Maps to: [x, y, yaw, vx, vy, omega, T, delta, theta, dT, ddelta, dtheta]
x_data = np.zeros((len(data), 12))
x_data[:, 3] = data[:, 0]  # vx
x_data[:, 4] = data[:, 1]  # vy
x_data[:, 5] = data[:, 2]  # omega
x_data[:, 6] = data[:, 4]  # T    (note: our col 4, notebook col 6)
x_data[:, 7] = data[:, 3]  # delta (note: our col 3, notebook col 7)

# y_data: [eps_vx, eps_vy, eps_omega]
y_data = data[:, 5:8]

x_path = out_dir / "x_data.csv"
y_path = out_dir / "y_data.csv"
np.savetxt(x_path, x_data, delimiter=",")
np.savetxt(y_path, y_data, delimiter=",")

print(f"Saved x_data.csv → {x_path}  shape={x_data.shape}")
print(f"Saved y_data.csv → {y_path}  shape={y_data.shape}")
print("\nNow open gp_training.ipynb and change WORKDIR to:")
print(f'  WORKDIR = "/code/src/ros4crs/rospy_controllers"')
