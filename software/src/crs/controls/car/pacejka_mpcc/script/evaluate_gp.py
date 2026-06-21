"""
Evaluate and compare GP residual models: sklearn vs gpytorch.

Splits data 80/20, trains both GPs on 80%, evaluates on held-out 20%.

Usage:
    python3 evaluate_gp.py
    python3 evaluate_gp.py --no-gpytorch   # skip gpytorch if not installed
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
parser.add_argument("--no-gpytorch", action="store_true", help="skip gpytorch comparison")
args = parser.parse_args()

OUTPUT_NAMES = ["eps_vx", "eps_vy", "eps_omega"]
COLORS = ["steelblue", "darkorange", "forestgreen"]

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

# ── sklearn GP ────────────────────────────────────────────────────────────────
print("\nTraining sklearn GP ...")
scaler = StandardScaler()
X_train_sc = scaler.fit_transform(X_train)
X_test_sc  = scaler.transform(X_test)

kernel = 1.0 * RBF(length_scale=np.ones(5)) + WhiteKernel(noise_level=1e-3)
gp_sklearn = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=5, normalize_y=True)
gp_sklearn.fit(X_train_sc, Y_train)
Y_pred_sklearn = gp_sklearn.predict(X_test_sc)
print(f"  Kernel: {gp_sklearn.kernel_}")

# ── gpytorch GP ───────────────────────────────────────────────────────────────
Y_pred_gpytorch = None
if not args.no_gpytorch:
    try:
        import torch
        import gpytorch

        class SingleOutputGP(gpytorch.models.ExactGP):
            def __init__(self, train_x, train_y, likelihood):
                super().__init__(train_x, train_y, likelihood)
                self.mean_module  = gpytorch.means.ZeroMean()
                self.covar_module = gpytorch.kernels.ScaleKernel(
                    gpytorch.kernels.RBFKernel(ard_num_dims=5)
                )
            def forward(self, x):
                return gpytorch.distributions.MultivariateNormal(
                    self.mean_module(x), self.covar_module(x)
                )

        print("\nTraining gpytorch GP (3 independent models) ...")
        tx = torch.tensor(X_train_sc, dtype=torch.float32)
        test_x = torch.tensor(X_test_sc, dtype=torch.float32)

        preds_list = []
        for out_i, name in enumerate(OUTPUT_NAMES):
            ty_i = torch.tensor(Y_train[:, out_i], dtype=torch.float32)
            likelihood_i = gpytorch.likelihoods.GaussianLikelihood()
            model_i = SingleOutputGP(tx, ty_i, likelihood_i)

            model_i.train(); likelihood_i.train()
            optimizer = torch.optim.Adam(model_i.parameters(), lr=0.1)
            mll = gpytorch.mlls.ExactMarginalLogLikelihood(likelihood_i, model_i)

            for it in range(200):
                optimizer.zero_grad()
                loss = -mll(model_i(tx), ty_i)
                loss.backward()
                optimizer.step()

            model_i.eval(); likelihood_i.eval()
            with torch.no_grad():
                pred = likelihood_i(model_i(test_x)).mean.numpy()
            preds_list.append(pred)
            print(f"  {name} done")

        Y_pred_gpytorch = np.column_stack(preds_list)
        print("  gpytorch GP done.")

    except ImportError:
        print("  gpytorch not installed — skipping. Run: pip install gpytorch")

# ── RMSE comparison ───────────────────────────────────────────────────────────
print("\n── RMSE comparison (baseline = predict 0 = no GP correction) ────────────")
header = f"{'Output':<12} {'No GP':>10} {'sklearn GP':>12} {'Improv':>8}"
if Y_pred_gpytorch is not None:
    header += f" {'gpytorch GP':>13} {'Improv':>8}"
print(header)

for i, name in enumerate(OUTPUT_NAMES):
    rmse_base = np.sqrt(np.mean(Y_test[:, i] ** 2))
    rmse_sk   = np.sqrt(np.mean((Y_test[:, i] - Y_pred_sklearn[:, i]) ** 2))
    imp_sk    = (rmse_base - rmse_sk) / rmse_base * 100
    row = f"{name:<12} {rmse_base:>10.6f} {rmse_sk:>12.6f} {imp_sk:>7.1f}%"
    if Y_pred_gpytorch is not None:
        rmse_gpt = np.sqrt(np.mean((Y_test[:, i] - Y_pred_gpytorch[:, i]) ** 2))
        imp_gpt  = (rmse_base - rmse_gpt) / rmse_base * 100
        row += f" {rmse_gpt:>13.6f} {imp_gpt:>7.1f}%"
    print(row)

# ── Plots ─────────────────────────────────────────────────────────────────────
n_models = 2 if Y_pred_gpytorch is not None else 1
fig, axes = plt.subplots(n_models, 3, figsize=(14, 5 * n_models))
if n_models == 1:
    axes = axes[np.newaxis, :]

fig.suptitle("GP prediction vs actual residual (held-out 20% test set)", fontsize=12)

for col_i, (name, color) in enumerate(zip(OUTPUT_NAMES, COLORS)):
    for row_i, (label, Y_pred) in enumerate(
        [("sklearn GP", Y_pred_sklearn)]
        + ([("gpytorch GP", Y_pred_gpytorch)] if Y_pred_gpytorch is not None else [])
    ):
        ax = axes[row_i, col_i]
        actual = Y_test[:, col_i]
        pred   = Y_pred[:, col_i]

        lim = max(np.abs(actual).max(), np.abs(pred).max()) * 1.1
        ax.scatter(actual, pred, s=8, alpha=0.4, color=color)
        ax.plot([-lim, lim], [-lim, lim], "k--", linewidth=1, label="perfect")
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim)
        ax.set_xlabel(f"actual {name}")
        ax.set_ylabel(f"predicted {name}")
        rmse_base = np.sqrt(np.mean(actual ** 2))
        rmse      = np.sqrt(np.mean((actual - pred) ** 2))
        ax.set_title(f"{label} — {name}\nno-GP={rmse_base:.5f}  {label}={rmse:.5f}")
        ax.legend(fontsize=8)
        ax.set_aspect("equal")

plt.tight_layout()
plt.savefig(args.out, dpi=150)
print(f"\nSaved → {args.out}")
plt.show()
