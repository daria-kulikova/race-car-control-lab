"""
Train a GP residual model from MPCC data log.

Input CSV (from C++ controller data_log_path):
    vx, vy, omega, delta, T, eps_vx, eps_vy, eps_omega

Output (saved alongside this script):
    gp_model.npz  — everything needed for C++ inference

Usage:
    python3 train_gp.py --data /path/to/mpcc_residuals.csv
"""

import argparse
import numpy as np
from pathlib import Path

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel
from sklearn.preprocessing import StandardScaler


# ── CLI ───────────────────────────────────────────────────────────────────────

parser = argparse.ArgumentParser()
parser.add_argument("--data", default="/code/src/data/mpcc_residuals.csv",
                    help="Path to mpcc_residuals.csv")
parser.add_argument("--out",  default=str(Path(__file__).parent / "gp_model.npz"),
                    help="Output .npz path (default: gp_model.npz next to this script)")
parser.add_argument("--max-points", type=int, default=2000,
                    help="Max training points (subsample if more)")
parser.add_argument("--vx-min", type=float, default=0.5,
                    help="Ignore rows where vx < this (unstable low-speed dynamics)")
args = parser.parse_args()


# ── Load data ─────────────────────────────────────────────────────────────────

print(f"Loading {args.data} ...")
data = np.loadtxt(args.data, delimiter=",", skiprows=1)
print(f"  raw rows: {len(data)}")

# Filter low speed
mask = data[:, 0] >= args.vx_min          # vx >= vx_min
data = data[mask]
print(f"  after vx >= {args.vx_min} filter: {len(data)}")

# Subsample if too many points (GP is O(n³))
if len(data) > args.max_points:
    idx = np.random.choice(len(data), args.max_points, replace=False)
    data = data[idx]
    print(f"  subsampled to {args.max_points}")

# Features: [vx, vy, omega, delta, T]
X = data[:, :5]
# Targets:  [eps_vx, eps_vy, eps_omega]
Y = data[:, 5:]

print(f"  X shape: {X.shape},  Y shape: {Y.shape}")
print(f"  residual std:  eps_vx={Y[:,0].std():.4f}  "
      f"eps_vy={Y[:,1].std():.4f}  eps_omega={Y[:,2].std():.4f}")


# ── Scale features ────────────────────────────────────────────────────────────

scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)


# ── Train GP ──────────────────────────────────────────────────────────────────
# One GP per output dimension (sklearn's multi-output GP is just 3 independent GPs)

print("Training GP ...")
kernel = 1.0 * RBF(length_scale=np.ones(5)) + WhiteKernel(noise_level=1e-3)
gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=5, normalize_y=True)
gp.fit(X_scaled, Y)

print(f"  learned kernel: {gp.kernel_}")

# Decompose kernel components
k_rbf    = gp.kernel_.k1          # RBF part  (k1 * k2 = ConstantKernel * RBF → combined as 1.0*RBF)
k_noise  = gp.kernel_.k2          # WhiteKernel

# Extract length scales (after fitting, RBF may be wrapped in ConstantKernel)
# Handle both sklearn kernel structures
try:
    length_scale = k_rbf.k2.length_scale          # ConstantKernel * RBF
    amplitude    = np.sqrt(k_rbf.k1.constant_value)
except AttributeError:
    length_scale = k_rbf.length_scale              # plain RBF (if amplitude=1)
    amplitude    = 1.0

noise_level = k_noise.noise_level

print(f"  amplitude:     {amplitude:.4f}")
print(f"  length_scales: {length_scale}")
print(f"  noise_level:   {noise_level:.6f}")


# ── Precompute alpha = K^{-1} y  (needed for fast inference) ─────────────────

alpha = gp.alpha_        # shape (n_train, n_outputs) — sklearn stores this
X_train = gp.X_train_   # scaled training inputs

# y normalisation params (needed to unnormalise predictions in C++)
y_mean = gp._y_train_mean   # (n_outputs,)
y_std  = getattr(gp, "_y_train_std", np.ones_like(y_mean))  # (n_outputs,)

print(f"  alpha shape: {alpha.shape}")
print(f"  y_mean:  {y_mean}")
print(f"  y_std:   {y_std}")


# ── Save ──────────────────────────────────────────────────────────────────────

np.savez(args.out,
         X_train      = X_train,          # (n, 5) scaled
         alpha        = alpha,            # (n, 3)
         length_scale = length_scale,     # (5,)
         amplitude    = np.array([amplitude]),
         noise_level  = np.array([noise_level]),
         scaler_mean  = scaler.mean_,     # (5,) for standardisation in C++
         scaler_std   = scaler.scale_,    # (5,)
         y_mean       = y_mean,           # (3,) output mean  (for unnormalising)
         y_std        = y_std,            # (3,) output std
)

print(f"\nSaved → {args.out}")
print("Columns in alpha: [eps_vx, eps_vy, eps_omega]")


# ── Export binary for C++ inference ──────────────────────────────────────────

bin_path = str(args.out).replace(".npz", ".bin")
n_train, n_features = X_train.shape
n_outputs = alpha.shape[1]

with open(bin_path, "wb") as f:
    np.array([n_train, n_features, n_outputs], dtype=np.int32).tofile(f)
    X_train.astype(np.float64).tofile(f)
    alpha.astype(np.float64).tofile(f)
    length_scale.astype(np.float64).tofile(f)
    np.array([amplitude], dtype=np.float64).tofile(f)
    scaler.mean_.astype(np.float64).tofile(f)
    scaler.scale_.astype(np.float64).tofile(f)
    y_mean.astype(np.float64).tofile(f)
    y_std.astype(np.float64).tofile(f)

print(f"Saved → {bin_path}  (for C++ inference)")


# ── Quick sanity check ────────────────────────────────────────────────────────

y_pred, y_std = gp.predict(X_scaled[:10], return_std=True)
print("\nSanity check (first 10 points):")
for i, name in enumerate(["eps_vx", "eps_vy", "eps_omega"]):
    print(f"  pred  {name}: {y_pred[:, i]}")
    print(f"  actual {name}: {Y[:10, i]}")
