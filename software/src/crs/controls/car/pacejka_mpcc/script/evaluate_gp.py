"""
Evaluate GP residual model: predictions vs actuals, RMSE with/without GP.

Splits data 80/20, trains GP on 80%, evaluates on held-out 20%.
This gives an honest estimate of GP generalisation quality.

Usage:
    python3 /code/src/crs/controls/car/pacejka_mpcc/script/evaluate_gp.py
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel
from sklearn.preprocessing import StandardScaler

parser = argparse.ArgumentParser()
parser.add_argument("--data", default="/code/src/data/mpcc_residuals.csv")
parser.add_argument("--out", default=str(Path(__file__).parent / "gp_evaluation.png"))
parser.add_argument("--vx-min", type=float, default=0.5)
parser.add_argument("--max-points", type=int, default=2000)
parser.add_argument("--test-fraction", type=float, default=0.2)
parser.add_argument("--seed", type=int, default=42)
args = parser.parse_args()

# ── Load & filter ─────────────────────────────────────────────────────────────
data = np.loadtxt(args.data, delimiter=",", skiprows=1)
data = data[data[:, 0] >= args.vx_min]

if len(data) > args.max_points:
    rng = np.random.default_rng(args.seed)
    idx = rng.choice(len(data), args.max_points, replace=False)
    data = data[idx]

X = data[:, :5]   # [vx, vy, omega, delta, T]
Y = data[:, 5:]   # [eps_vx, eps_vy, eps_omega]

# ── Train/test split ──────────────────────────────────────────────────────────
rng = np.random.default_rng(args.seed)
n_test = int(len(data) * args.test_fraction)
test_idx = rng.choice(len(data), n_test, replace=False)
train_idx = np.setdiff1d(np.arange(len(data)), test_idx)

X_train, Y_train = X[train_idx], Y[train_idx]
X_test,  Y_test  = X[test_idx],  Y[test_idx]

print(f"Train: {len(X_train)}  Test: {len(X_test)}")

# ── Train GP ──────────────────────────────────────────────────────────────────
scaler = StandardScaler()
X_train_sc = scaler.fit_transform(X_train)
X_test_sc  = scaler.transform(X_test)

kernel = 1.0 * RBF(length_scale=np.ones(5)) + WhiteKernel(noise_level=1e-3)
gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=5, normalize_y=True)
gp.fit(X_train_sc, Y_train)
print(f"Kernel: {gp.kernel_}")

# ── Predict on test set ───────────────────────────────────────────────────────
Y_pred = gp.predict(X_test_sc)

# ── RMSE: baseline = predict 0 (= Pacejka with no correction) vs GP ──────────
# Without GP the controller assumes residual = 0, so its prediction error = std(eps).
# GP is useful if its RMSE < std(eps), i.e. it explains some of the variance.

OUTPUT_NAMES = ["eps_vx", "eps_vy", "eps_omega"]

print("\n── RMSE comparison (baseline = predict 0, i.e. no GP correction) ────────")
print(f"{'Output':<12} {'No GP (=0)':>12} {'With GP':>12} {'Improvement':>14}")
for i, name in enumerate(OUTPUT_NAMES):
    rmse_base = np.sqrt(np.mean(Y_test[:, i] ** 2))                        # predict 0
    rmse_gp   = np.sqrt(np.mean((Y_test[:, i] - Y_pred[:, i]) ** 2))
    improvement = (rmse_base - rmse_gp) / rmse_base * 100
    print(f"{name:<12} {rmse_base:>12.6f} {rmse_gp:>12.6f} {improvement:>13.1f}%")

# ── Plots ─────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(1, 3, figsize=(14, 5))
fig.suptitle("GP prediction vs actual residual (held-out 20% test set)", fontsize=12)

COLORS = ["steelblue", "darkorange", "forestgreen"]

for i, (name, col) in enumerate(zip(OUTPUT_NAMES, COLORS)):
    ax = axes[i]
    actual = Y_test[:, i]
    pred   = Y_pred[:, i]

    lim = max(np.abs(actual).max(), np.abs(pred).max()) * 1.1
    ax.scatter(actual, pred, s=8, alpha=0.4, color=col)
    ax.plot([-lim, lim], [-lim, lim], "k--", linewidth=1, label="perfect")
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_xlabel(f"actual {name}")
    ax.set_ylabel(f"predicted {name}")

    rmse_base = np.sqrt(np.mean(actual ** 2))
    rmse_gp   = np.sqrt(np.mean((actual - pred) ** 2))
    ax.set_title(f"{name}\nRMSE no-GP={rmse_base:.5f}  GP={rmse_gp:.5f}")
    ax.legend(fontsize=8)
    ax.set_aspect("equal")

plt.tight_layout()
plt.savefig(args.out, dpi=150)
print(f"\nSaved → {args.out}")
plt.show()
