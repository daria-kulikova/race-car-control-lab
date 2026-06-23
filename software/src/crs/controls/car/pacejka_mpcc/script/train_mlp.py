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
parser.add_argument("--data", default="/code/src/crs/controls/car/pacejka_mpcc/script/data/combined_data.csv")
parser.add_argument("--out",  default=str(Path(__file__).parent / "mlp_model.npz"))
parser.add_argument("--vx-min",    type=float, default=0.5)
parser.add_argument("--iqr-k",     type=float, default=3.0, help="IQR multiplier for outlier removal (larger = keep more)")
parser.add_argument("--hidden",    type=int,   nargs="+", default=[64, 64, 32])
parser.add_argument("--alpha-reg", type=float, default=0.01)
parser.add_argument("--seed",      type=int,   default=42)
args = parser.parse_args()

# ── Load ──────────────────────────────────────────────────────────────────────
print(f"Loading {args.data} ...")
data = np.loadtxt(args.data, delimiter=",", skiprows=1)
data = data[data[:, 0] >= args.vx_min]
print(f"  {len(data)} points after vx >= {args.vx_min} filter")

# ── Outlier filtering on eps_ targets (cols 5,6,7) ────────────────────────────
eps_cols = [5, 6, 7]
mask = np.ones(len(data), dtype=bool)
for c in eps_cols:
    q25, q75 = np.percentile(data[:, c], [25, 75])
    iqr = q75 - q25
    mask &= (data[:, c] >= q25 - args.iqr_k * iqr) & (data[:, c] <= q75 + args.iqr_k * iqr)
print(f"  {mask.sum()} points after outlier filter (removed {(~mask).sum()}, k={args.iqr_k})")
data = data[mask]

X = data[:, :5]   # [vx, vy, omega, delta, T]
Y = data[:, 5:8]  # [eps_vx, eps_vy, eps_omega]

# ── Scale ─────────────────────────────────────────────────────────────────────
x_scaler = StandardScaler()
X_sc = x_scaler.fit_transform(X)

y_scaler = StandardScaler()
Y_sc = y_scaler.fit_transform(Y)

# ── Train ─────────────────────────────────────────────────────────────────────
print(f"Training MLP {args.hidden} ...")
mlp = MLPRegressor(hidden_layer_sizes=tuple(args.hidden), activation='tanh',
                   solver='adam', max_iter=5000, early_stopping=True,
                   alpha=args.alpha_reg, random_state=args.seed)
mlp.fit(X_sc, Y_sc)
print(f"  converged in {mlp.n_iter_} iterations")

# Sanity: RMSE on training data
Y_pred_sc = mlp.predict(X_sc)
Y_pred = y_scaler.inverse_transform(Y_pred_sc)
rmse_base = np.sqrt(np.mean(Y ** 2, axis=0))
rmse_mlp  = np.sqrt(np.mean((Y - Y_pred) ** 2, axis=0))
for name, rb, rm in zip(["eps_vx", "eps_vy", "eps_omega"], rmse_base, rmse_mlp):
    print(f"  {name}: no-model={rb:.6f}  mlp={rm:.6f}  improv={100*(rb-rm)/rb:.1f}%")

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

npz_path = args.out
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
