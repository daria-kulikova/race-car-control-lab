"""
Train MLP residual model and save weights for:
  1. CasADi integration in pacejka_model.py (via .npz)
  2. C++ lag compensation inference (via .bin)

Usage:
    python3 train_mlp.py
    python3 train_mlp.py --data /code/src/data/combined_data.csv
"""

import argparse
import numpy as np
from pathlib import Path
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler

parser = argparse.ArgumentParser()
parser.add_argument("--data", default="/code/src/crs/controls/car/pacejka_mpcc/script/data/mpcc_residuals_cm1_0555_0.csv")
parser.add_argument("--out",  default=str(Path(__file__).parent / "mlp_model.npz"))
parser.add_argument("--vx-min",    type=float, default=0.5)
parser.add_argument("--iqr-k",     type=float, default=7.0, help="IQR multiplier for outlier removal (larger = keep more)")
parser.add_argument("--hidden",    type=int,   nargs="+", default=[64, 64, 32])
parser.add_argument("--n-datasets", type=int,   default=1,
                    help="Number of datasets to combine (*_0.csv, *_1.csv, ...); dataset i gets weight i+1")
parser.add_argument("--alpha-reg", type=float, default=0.01)
parser.add_argument("--seed",      type=int,   default=42)
parser.add_argument("--lf",        type=float, default=0.052, help="front axle distance [m]")
parser.add_argument("--lr",        type=float, default=0.038, help="rear axle distance [m]")
parser.add_argument("--features",      type=str,   nargs="+",
                    default=["vx", "vy", "omega", "delta", "T", "beta", "alpha_f", "alpha_r"],
                    choices=["vx", "vy", "omega", "delta", "T", "beta", "alpha_f", "alpha_r"],
                    help="feature subset to use for training")
parser.add_argument("--test-fraction", type=float, default=0.0, help="fraction held out for test RMSE (0 = disabled)")
args = parser.parse_args()


def slip_angles(vx, vy, omega, delta, lf, lr):
    vx_safe = np.maximum(vx, 0.1)
    beta  = np.arctan2(vy, vx_safe)
    alpha_f = delta - np.arctan2(vy + lf * omega, vx_safe)
    alpha_r = -np.arctan2(vy - lr * omega, vx_safe)
    return beta, alpha_f, alpha_r


# ── Load ──────────────────────────────────────────────────────────────────────
base = args.data[:-len("0.csv")]  # strip trailing "0.csv" → e.g. "/path/to/data_"
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

# ── Outlier filtering on eps_ targets (cols 5,6,7) ────────────────────────────
# eps_cols = [5, 6, 7]
# mask = np.ones(len(data), dtype=bool)
# for c in eps_cols:
#     q25, q75 = np.percentile(data[:, c], [25, 75])
#     iqr = q75 - q25
#     mask &= (data[:, c] >= q25 - args.iqr_k * iqr) & (data[:, c] <= q75 + args.iqr_k * iqr)
# print(f"  {mask.sum()} points after outlier filter (removed {(~mask).sum()}, k={args.iqr_k})")
# data    = data[mask]
# weights = weights[mask]

raw = data[:, 1:6]   # [vx, vy, omega, delta, T]
beta, alpha_f, alpha_r = slip_angles(raw[:, 0], raw[:, 1], raw[:, 2], raw[:, 3],
                                     args.lf, args.lr)
ALL_FEATURES = {
    "vx": raw[:, 0], "vy": raw[:, 1], "omega": raw[:, 2],
    "delta": raw[:, 3], "T": raw[:, 4],
    "beta": beta, "alpha_f": alpha_f, "alpha_r": alpha_r,
}
X = np.column_stack([ALL_FEATURES[f] for f in args.features])
Y = data[:, 6:9]                                     # [eps_vx, eps_vy, eps_omega]
print(f"  features ({len(args.features)}): {args.features}")

# ── Optional test split (held out before scaling) ─────────────────────────────
rng = np.random.default_rng(args.seed)
if args.test_fraction > 0.0:
    n_test    = int(len(X) * args.test_fraction)
    test_idx  = rng.choice(len(X), n_test, replace=False)
    train_idx = np.setdiff1d(np.arange(len(X)), test_idx)
    X_test_raw, Y_test_raw = X[test_idx], Y[test_idx]
    X, Y, weights = X[train_idx], Y[train_idx], weights[train_idx]
    print(f"  test set: {n_test} points  train: {len(X)} points")
else:
    X_test_raw = Y_test_raw = None

# ── Scale (fit only on train) ─────────────────────────────────────────────────
x_scaler = StandardScaler()
X_sc = x_scaler.fit_transform(X)

y_scaler = StandardScaler()
Y_sc = y_scaler.fit_transform(Y)

# ── Train ─────────────────────────────────────────────────────────────────────
print(f"Training MLP {args.hidden} ...")
mlp = MLPRegressor(hidden_layer_sizes=tuple(args.hidden), activation='tanh',
                   solver='adam', max_iter=5000, early_stopping=True,
                   alpha=args.alpha_reg, random_state=args.seed)
X_sc_w = np.repeat(X_sc, weights.astype(int), axis=0)
Y_sc_w = np.repeat(Y_sc, weights.astype(int), axis=0)
mlp.fit(X_sc_w, Y_sc_w)
print(f"  converged in {mlp.n_iter_} iterations")

# Sanity: RMSE on training data
Y_pred_sc = mlp.predict(X_sc)
Y_pred = y_scaler.inverse_transform(Y_pred_sc)
rmse_base = np.sqrt(np.mean(Y ** 2, axis=0))
rmse_mlp  = np.sqrt(np.mean((Y - Y_pred) ** 2, axis=0))
print("  [train]")
for name, rb, rm in zip(["eps_vx", "eps_vy", "eps_omega"], rmse_base, rmse_mlp):
    print(f"    {name}: no-model={rb:.6f}  mlp={rm:.6f}  improv={100*(rb-rm)/rb:.1f}%")

if X_test_raw is not None:
    X_test_sc   = x_scaler.transform(X_test_raw)
    Y_test_pred = y_scaler.inverse_transform(mlp.predict(X_test_sc))
    rmse_base_t = np.sqrt(np.mean(Y_test_raw ** 2, axis=0))
    rmse_mlp_t  = np.sqrt(np.mean((Y_test_raw - Y_test_pred) ** 2, axis=0))
    print("  [test]")
    for name, rb, rm in zip(["eps_vx", "eps_vy", "eps_omega"], rmse_base_t, rmse_mlp_t):
        print(f"    {name}: no-model={rb:.6f}  mlp={rm:.6f}  improv={100*(rb-rm)/rb:.1f}%")

# ── Save .npz (for pacejka_model.py) ─────────────────────────────────────────
# sklearn coefs_[i] shape: (n_in_i, n_out_i) → transpose to get W @ x + b convention
# Keys: W0/b0, W1/b1, ..., W{n}/b{n}  where n = len(hidden)  (last = output layer)
layers = mlp.coefs_
biases = mlp.intercepts_

n_in  = X.shape[1]   # 5
n_out = Y.shape[1]   # 3
sizes = [n_in] + args.hidden + [n_out]  # e.g. [5, 64, 64, 32, 3]

npz_dict = dict(
    x_mean=x_scaler.mean_,  x_std=x_scaler.scale_,
    y_mean=y_scaler.mean_,  y_std=y_scaler.scale_,
    hidden=np.array(args.hidden, dtype=np.int32),
)
for i, (W, b) in enumerate(zip(layers, biases)):
    npz_dict[f"W{i}"] = W.T   # (out, in) convention
    npz_dict[f"b{i}"] = b

stem = args.out[:-len(".npz")]
npz_path = f"{stem}_{args.n_datasets - 1}.npz"
np.savez(npz_path, **npz_dict)
print(f"Saved → {npz_path}  (arch {sizes})")

# ── Save .bin (for C++ inference) ─────────────────────────────────────────────
# Layout (generic, any depth):
#   int32[1]:          n_layers  (= len(hidden) + 1, i.e. number of weight matrices)
#   int32[n_layers+1]: sizes     [n_in, h1, ..., hk, n_out]
#   float64[n_in]:     x_mean
#   float64[n_in]:     x_std
#   float64[n_out]:    y_mean
#   float64[n_out]:    y_std
#   for each layer i in 0..n_layers-1:
#     float64[sizes[i+1] * sizes[i]]:  W[i]  (row major, W@x convention)
#     float64[sizes[i+1]]:             b[i]
n_layers = len(layers)

bin_path = str(npz_path).replace(".npz", ".bin")
with open(bin_path, "wb") as f:
    np.array([n_layers], dtype=np.int32).tofile(f)
    np.array(sizes, dtype=np.int32).tofile(f)
    x_scaler.mean_.astype(np.float64).tofile(f)
    x_scaler.scale_.astype(np.float64).tofile(f)
    y_scaler.mean_.astype(np.float64).tofile(f)
    y_scaler.scale_.astype(np.float64).tofile(f)
    for W, b in zip(layers, biases):
        W.T.astype(np.float64).tofile(f)   # row major, W@x convention
        b.astype(np.float64).tofile(f)

print(f"Saved → {bin_path}  (for C++ lag compensation)")
