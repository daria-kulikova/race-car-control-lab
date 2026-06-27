"""
Train a GP residual model from MPCC combined log files.

Loads n-datasets files (*_0.csv, *_1.csv, ...) with dataset i having weight i+1.
Test data is loaded from *_test_{i}.csv files (same convention as train_mlp_2.py).

Usage:
    python3 train_gp.py
    python3 train_gp.py --n-datasets 3 --features vx vy omega delta T
    python3 train_gp.py --max-points 3000 --vx-min 1.0
"""

import argparse
import numpy as np
from pathlib import Path

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel
from sklearn.preprocessing import StandardScaler


parser = argparse.ArgumentParser()
parser.add_argument("--data",         default="/code/src/crs/controls/car/pacejka_mpcc/script/data/mlp_final/residuals_0.csv")
parser.add_argument("--out",          default=str(Path(__file__).parent / "models/mlp_final/gp_model.npz"))
parser.add_argument("--max-points",   type=int,   default=1500,
                    help="max training points (GP is O(n³))")
parser.add_argument("--vx-min",       type=float, default=1.0)
parser.add_argument("--iqr-k",        type=float, default=3.0,
                    help="IQR multiplier for outlier removal on eps_ targets")
parser.add_argument("--n-datasets",   type=int,   default=1,
                    help="number of datasets (*_0.csv, *_1.csv, ...); dataset i gets weight i+1")
parser.add_argument("--seed",         type=int,   default=42)
parser.add_argument("--lf",           type=float, default=0.052, help="front axle distance [m]")
parser.add_argument("--lr",           type=float, default=0.038, help="rear axle distance [m]")
parser.add_argument("--features",     type=str,   nargs="+",
                    default=["vx", "vy", "omega", "delta", "T"],
                    choices=["vx", "vy", "omega", "delta", "T", "beta", "alpha_f", "alpha_r"],
                    help="feature subset to use for training")
args = parser.parse_args()
args.features = sorted(args.features)

rng = np.random.default_rng(args.seed)


def slip_angles(vx, vy, omega, delta):
    vx_safe = np.maximum(vx, 0.1)
    beta    = np.arctan2(vy, vx_safe)
    alpha_f = delta - np.arctan2(vy + args.lf * omega, vx_safe)
    alpha_r = -np.arctan2(vy - args.lr * omega, vx_safe)
    return beta, alpha_f, alpha_r


def build_features(data):
    # columns: t,vx,vy,omega,delta,T,eps_vx,eps_vy,eps_omega,eC,eL,pos_x,pos_y,ref_x,ref_y,lap,theta
    raw = data[:, 1:6]   # [vx, vy, omega, delta, T]
    beta, alpha_f, alpha_r = slip_angles(raw[:, 0], raw[:, 1], raw[:, 2], raw[:, 3])
    ALL = {
        "vx": raw[:, 0], "vy": raw[:, 1], "omega": raw[:, 2],
        "delta": raw[:, 3], "T": raw[:, 4],
        "beta": beta, "alpha_f": alpha_f, "alpha_r": alpha_r,
    }
    X = np.column_stack([ALL[f] for f in args.features])
    Y = data[:, 6:9]   # [eps_vx, eps_vy, eps_omega]
    return X, Y


# ── Load training data ────────────────────────────────────────────────────────
base = args.data[:-len("0.csv")]
chunks, weight_chunks = [], []
for i in range(args.n_datasets):
    path = base + f"{i}.csv"
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    chunks.append(d)
    weight_chunks.append(np.full(len(d), float(i + 1)))
    print(f"  loaded {path}: {len(d)} rows, weight={i + 1}")
data    = np.vstack(chunks)
weights = np.concatenate(weight_chunks)
print(f"  total: {len(data)} rows")

mask_vx = data[:, 1] >= args.vx_min
data    = data[mask_vx]
weights = weights[mask_vx]
print(f"  {len(data)} points after vx >= {args.vx_min} filter")

# mask = np.ones(len(data), dtype=bool)
# for c in [6, 7, 8]:
#     q25, q75 = np.percentile(data[:, c], [25, 75])
#     iqr = q75 - q25
#     mask &= (data[:, c] >= q25 - args.iqr_k * iqr) & (data[:, c] <= q75 + args.iqr_k * iqr)
# data    = data[mask]
# weights = weights[mask]
# print(f"  {mask.sum()} points after outlier filter (k={args.iqr_k}, removed {(~mask).sum()})")

X_all, Y_all = build_features(data)
print(f"  features ({len(args.features)}): {args.features}")

# ── Load test data from *_test_{i}.csv ───────────────────────────────────────
test_base   = base + "test_"
test_chunks = []
for i in range(args.n_datasets):
    p = test_base + f"{i}.csv"
    if Path(p).exists():
        test_chunks.append(np.loadtxt(p, delimiter=",", skiprows=1))
        print(f"  loaded test {p}: {len(test_chunks[-1])} rows")

if test_chunks:
    test_data = np.vstack(test_chunks)
    test_data = test_data[test_data[:, 1] >= args.vx_min]
    X_test, Y_test = build_features(test_data)
    print(f"  test total: {len(X_test)} points")
else:
    print("  no test files found — falling back to 20% random split")
    n_test   = int(len(data) * 0.2)
    test_idx = rng.choice(len(data), n_test, replace=False)
    tr_idx   = np.setdiff1d(np.arange(len(data)), test_idx)
    X_test,  Y_test  = X_all[test_idx], Y_all[test_idx]
    X_all,   Y_all   = X_all[tr_idx],   Y_all[tr_idx]
    weights          = weights[tr_idx]

# ── Weighted subsample for GP ─────────────────────────────────────────────────
w_norm = weights / weights.sum()
if len(X_all) > args.max_points:
    gp_idx = rng.choice(len(X_all), args.max_points, replace=False, p=w_norm)
    X_train, Y_train = X_all[gp_idx], Y_all[gp_idx]
    print(f"  subsampled to {args.max_points} (weighted by dataset recency)")
else:
    X_train, Y_train = X_all, Y_all

print(f"  GP train: {len(X_train)}  test: {len(X_test)}")
print(f"  residual std:  eps_vx={Y_train[:,0].std():.4f}  "
      f"eps_vy={Y_train[:,1].std():.4f}  eps_omega={Y_train[:,2].std():.4f}")

# ── Scale features ────────────────────────────────────────────────────────────
scaler   = StandardScaler()
X_train_sc = scaler.fit_transform(X_train)
X_test_sc  = scaler.transform(X_test)

# ── Train GP ──────────────────────────────────────────────────────────────────
n_feat = X_train.shape[1]
print(f"\nTraining GP ({n_feat} features, {len(X_train)} points) ...")
kernel = 1.0 * RBF(length_scale=np.ones(n_feat), length_scale_bounds=(1e-2, 100)) + WhiteKernel(noise_level=0.1, noise_level_bounds=(1e-2, 1e1))
gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=5, normalize_y=True, random_state=args.seed)
gp.fit(X_train_sc, Y_train)
print(f"  learned kernel: {gp.kernel_}")

# ── Extract kernel parameters ─────────────────────────────────────────────────
k_rbf   = gp.kernel_.k1
k_noise = gp.kernel_.k2
try:
    length_scale = k_rbf.k2.length_scale
    amplitude    = np.sqrt(k_rbf.k1.constant_value)
except AttributeError:
    length_scale = k_rbf.length_scale
    amplitude    = 1.0
noise_level = k_noise.noise_level

print(f"  amplitude:     {amplitude:.4f}")
print(f"  length_scales: {length_scale}")
print(f"  noise_level:   {noise_level:.6f}")

alpha   = gp.alpha_
X_stored = gp.X_train_
y_mean  = gp._y_train_mean
y_std   = getattr(gp, "_y_train_std", np.ones_like(y_mean))

# ── RMSE on test set ──────────────────────────────────────────────────────────
Y_pred = gp.predict(X_test_sc)
print("\n── RMSE on test set ──────────────────────────────────────────────────────")
for i, name in enumerate(["eps_vx", "eps_vy", "eps_omega"]):
    rmse_base = np.sqrt(np.mean(Y_test[:, i] ** 2))
    rmse_gp   = np.sqrt(np.mean((Y_test[:, i] - Y_pred[:, i]) ** 2))
    print(f"  {name}: no-model={rmse_base:.6f}  GP={rmse_gp:.6f}  "
          f"improv={100*(rmse_base-rmse_gp)/rmse_base:.1f}%")

# ── Save .npz ─────────────────────────────────────────────────────────────────
stem     = args.out[:-len(".npz")]
npz_path = f"{stem}_{args.n_datasets - 1}.npz"
Path(npz_path).parent.mkdir(parents=True, exist_ok=True)
np.savez(npz_path,
         X_train      = X_stored,
         alpha        = alpha,
         length_scale = np.atleast_1d(length_scale),
         amplitude    = np.array([amplitude]),
         noise_level  = np.array([noise_level]),
         scaler_mean  = scaler.mean_,
         scaler_std   = scaler.scale_,
         y_mean       = y_mean,
         y_std        = y_std,
)
print(f"\nSaved → {npz_path}")

# ── Save .bin for C++ inference ───────────────────────────────────────────────
bin_path   = str(npz_path).replace(".npz", ".bin")
n_train, n_features = X_stored.shape
n_outputs  = alpha.shape[1]

with open(bin_path, "wb") as f:
    np.array([n_train, n_features, n_outputs], dtype=np.int32).tofile(f)
    X_stored.astype(np.float64).tofile(f)
    alpha.astype(np.float64).tofile(f)
    np.atleast_1d(length_scale).astype(np.float64).tofile(f)
    np.array([amplitude], dtype=np.float64).tofile(f)
    scaler.mean_.astype(np.float64).tofile(f)
    scaler.scale_.astype(np.float64).tofile(f)
    y_mean.astype(np.float64).tofile(f)
    y_std.astype(np.float64).tofile(f)

print(f"Saved → {bin_path}  (for C++ inference)")
print(f"Columns in alpha: [eps_vx, eps_vy, eps_omega]")
