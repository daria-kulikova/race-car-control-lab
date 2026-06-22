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
parser.add_argument("--hidden",    type=int,   nargs="+", default=[64, 64])
parser.add_argument("--alpha-reg", type=float, default=0.01)
parser.add_argument("--seed",      type=int,   default=42)
args = parser.parse_args()

# ── Load ──────────────────────────────────────────────────────────────────────
print(f"Loading {args.data} ...")
data = np.loadtxt(args.data, delimiter=",", skiprows=1)
data = data[data[:, 0] >= args.vx_min]
print(f"  {len(data)} points after vx >= {args.vx_min} filter")

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
                   solver='lbfgs', max_iter=2000,
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
# sklearn coefs_[i] shape: (n_in, n_out)  → forward pass: X @ W + b
# We want column-vector convention: W @ x + b  → transpose weight matrices
layers = mlp.coefs_
biases = mlp.intercepts_

npz_path = args.out
np.savez(npz_path,
         W1=layers[0].T,    b1=biases[0],    # (h1, n_in),  (h1,)
         W2=layers[1].T,    b2=biases[1],    # (h2, h1),    (h2,)
         Wout=layers[2].T,  bout=biases[2],  # (n_out, h2), (n_out,)
         x_mean=x_scaler.mean_,  x_std=x_scaler.scale_,
         y_mean=y_scaler.mean_,  y_std=y_scaler.scale_,
         hidden=np.array(args.hidden, dtype=np.int32))
print(f"Saved → {npz_path}")

# ── Save .bin (for C++ inference) ─────────────────────────────────────────────
# Layout:
#   int32[4]: n_in, h1, h2, n_out
#   float64[n_in]: x_mean
#   float64[n_in]: x_std
#   float64[n_out]: y_mean
#   float64[n_out]: y_std
#   float64[h1 * n_in]: W1  (row major)
#   float64[h1]:        b1
#   float64[h2 * h1]:   W2  (row major)
#   float64[h2]:        b2
#   float64[n_out * h2]: Wout (row major)
#   float64[n_out]:      bout
n_in   = X.shape[1]        # 5
h1, h2 = args.hidden       # 64, 64
n_out  = Y.shape[1]        # 3

bin_path = str(npz_path).replace(".npz", ".bin")
with open(bin_path, "wb") as f:
    np.array([n_in, h1, h2, n_out], dtype=np.int32).tofile(f)
    x_scaler.mean_.astype(np.float64).tofile(f)
    x_scaler.scale_.astype(np.float64).tofile(f)
    y_scaler.mean_.astype(np.float64).tofile(f)
    y_scaler.scale_.astype(np.float64).tofile(f)
    layers[0].T.astype(np.float64).tofile(f)   # W1 row major
    biases[0].astype(np.float64).tofile(f)
    layers[1].T.astype(np.float64).tofile(f)   # W2 row major
    biases[1].astype(np.float64).tofile(f)
    layers[2].T.astype(np.float64).tofile(f)   # Wout row major
    biases[2].astype(np.float64).tofile(f)

print(f"Saved → {bin_path}  (for C++ lag compensation)")
