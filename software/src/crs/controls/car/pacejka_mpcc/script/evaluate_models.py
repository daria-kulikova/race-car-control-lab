"""
Evaluate and compare GP residual models: sklearn vs gpytorch exact vs gpytorch inducing.

Loads n-datasets files (*_0.csv, *_1.csv, ...) with dataset i having weight i+1.
Test data is loaded from *_test_{i}.csv files (same convention as train_mlp_2.py).

Usage:
    python3 evaluate_gp.py
    python3 evaluate_gp.py --n-datasets 3 --features vx vy omega delta T
    python3 evaluate_gp.py --inducing-gp --n-inducing-points 200
    python3 evaluate_gp.py --no-gp --no-gpytorch   # MLP only
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler

parser = argparse.ArgumentParser()
parser.add_argument("--data",              default="/code/src/crs/controls/car/pacejka_mpcc/script/data/mlp_final/residuals_0.csv")
parser.add_argument("--out",               default=str(Path(__file__).parent / "plots/model_evaluation.png"))
parser.add_argument("--vx-min",            type=float, default=1.0)
parser.add_argument("--iqr-k",             type=float, default=3.0)
parser.add_argument("--gp-max-points",     type=int,   default=1500,
                    help="max training points for sklearn/exact gpytorch GP")
parser.add_argument("--n-datasets",        type=int,   default=1,
                    help="number of datasets (*_0.csv, ...); dataset i gets weight i+1")
parser.add_argument("--seed",              type=int,   default=42)
parser.add_argument("--no-gp",             action="store_true", help="skip sklearn GP")
parser.add_argument("--no-mlp",            action="store_true", help="skip MLP")
parser.add_argument("--no-gpytorch",       action="store_true", help="skip gpytorch GP")
parser.add_argument("--inducing-gp",       action="store_true",
                    help="use inducing point GP instead of exact gpytorch GP")
parser.add_argument("--n-inducing-points", type=int,   default=500,
                    help="number of inducing points (--inducing-gp only)")
parser.add_argument("--lf",                type=float, default=0.052)
parser.add_argument("--lr",                type=float, default=0.038)
parser.add_argument("--features",          type=str,   nargs="+",
                    default=["vx", "vy", "omega", "delta", "T", "beta", "alpha_f", "alpha_r"],
                    choices=["vx", "vy", "omega", "delta", "T", "beta", "alpha_f", "alpha_r"])
args = parser.parse_args()
args.features = sorted(args.features)

OUTPUT_NAMES = ["eps_vx", "eps_vy", "eps_omega"]
COLORS = ["steelblue", "darkorange", "forestgreen"]
rng = np.random.default_rng(args.seed)


def slip_angles(vx, vy, omega, delta):
    vx_safe = np.maximum(vx, 0.1)
    beta    = np.arctan2(vy, vx_safe)
    alpha_f = delta - np.arctan2(vy + args.lf * omega, vx_safe)
    alpha_r = -np.arctan2(vy - args.lr * omega, vx_safe)
    return beta, alpha_f, alpha_r


def build_features(data):
    # columns: t,vx,vy,omega,delta,T,eps_vx,eps_vy,eps_omega,eC,eL,pos_x,pos_y,ref_x,ref_y,lap,theta
    raw = data[:, 1:6]
    beta, alpha_f, alpha_r = slip_angles(raw[:, 0], raw[:, 1], raw[:, 2], raw[:, 3])
    ALL = {
        "vx": raw[:, 0], "vy": raw[:, 1], "omega": raw[:, 2],
        "delta": raw[:, 3], "T": raw[:, 4],
        "beta": beta, "alpha_f": alpha_f, "alpha_r": alpha_r,
    }
    X = np.column_stack([ALL[f] for f in args.features])
    Y = data[:, 6:9]
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
# print(f"  {mask.sum()} points after outlier filter (removed {(~mask).sum()})")

X_train, Y_train = build_features(data)
print(f"  features ({len(args.features)}): {args.features}")

# ── Load test data ────────────────────────────────────────────────────────────
test_base, test_chunks = base + "test_", []
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
    print("  no test files — falling back to 20% random split")
    n_test   = int(len(data) * 0.2)
    test_idx = rng.choice(len(data), n_test, replace=False)
    tr_idx   = np.setdiff1d(np.arange(len(data)), test_idx)
    X_test,  Y_test  = X_train[test_idx],  Y_train[test_idx]
    X_train, Y_train = X_train[tr_idx],    Y_train[tr_idx]
    weights          = weights[tr_idx]

# ── Weighted subsample for sklearn / exact gpytorch GP ────────────────────────
w_norm = weights / weights.sum()
if len(X_train) > args.gp_max_points:
    gp_idx = rng.choice(len(X_train), args.gp_max_points, replace=False, p=w_norm)
    X_train_gp, Y_train_gp = X_train[gp_idx], Y_train[gp_idx]
else:
    X_train_gp, Y_train_gp = X_train, Y_train

print(f"\nTrain total: {len(X_train)}  GP subsample: {len(X_train_gp)}  Test: {len(X_test)}")

# ── Scaler ────────────────────────────────────────────────────────────────────
scaler = StandardScaler()
scaler.fit(X_train_gp)
X_train_gp_sc = scaler.transform(X_train_gp)
X_train_sc    = scaler.transform(X_train)
X_test_sc     = scaler.transform(X_test)

# ── sklearn GP ────────────────────────────────────────────────────────────────
Y_pred_sklearn = None
if not args.no_gp:
    print("\nTraining sklearn GP ...")
    kernel = 1.0 * RBF(length_scale=np.ones(X_train.shape[1]), length_scale_bounds=(1e-2, 100)) + WhiteKernel(noise_level=0.1, noise_level_bounds=(1e-2, 1e1))
    gp_sklearn = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=5, normalize_y=True, random_state=args.seed)
    gp_sklearn.fit(X_train_gp_sc, Y_train_gp)
    Y_pred_sklearn = gp_sklearn.predict(X_test_sc)
    k = gp_sklearn.kernel_
    try:
        amplitude   = np.sqrt(k.k1.k1.constant_value)
        ls          = k.k1.k2.length_scale
    except AttributeError:
        amplitude   = 1.0
        ls          = k.k1.length_scale
    noise = k.k2.noise_level
    print(f"  amplitude:    {amplitude:.4f}")
    print(f"  length_scale: {dict(zip(args.features, np.round(ls, 4)))}")
    print(f"  noise_level:  {noise:.6f}")

# ── MLP (sklearn, weighted by repeating samples) ──────────────────────────────
Y_pred_mlp = None
if not args.no_mlp:
    print("\nTraining MLP ...")
    y_scaler   = StandardScaler()
    Y_train_sc = y_scaler.fit_transform(Y_train)
    int_w  = weights.astype(int)
    X_tr_w = np.repeat(X_train_sc, int_w, axis=0)
    Y_tr_w = np.repeat(Y_train_sc, int_w, axis=0)
    mlp = MLPRegressor(hidden_layer_sizes=(64, 64, 32), activation="tanh",
                       solver="adam", max_iter=5000, early_stopping=True,
                       alpha=0.01, random_state=args.seed)
    mlp.fit(X_tr_w, Y_tr_w)
    Y_pred_mlp = y_scaler.inverse_transform(mlp.predict(X_test_sc))
    print(f"  hidden: {mlp.hidden_layer_sizes}  iters: {mlp.n_iter_}")

# ── gpytorch GP (exact or inducing) ───────────────────────────────────────────
Y_pred_gpytorch = None
gp_label        = None
if not args.no_gpytorch:
    try:
        import torch
        import gpytorch
        torch.manual_seed(args.seed)

        if not args.inducing_gp:
            # ── Exact GP on subsampled data ────────────────────────────────────
            class ExactGP(gpytorch.models.ExactGP):
                def __init__(self, train_x, train_y, likelihood):
                    super().__init__(train_x, train_y, likelihood)
                    self.mean_module  = gpytorch.means.ZeroMean()
                    self.covar_module = gpytorch.kernels.ScaleKernel(
                        gpytorch.kernels.RBFKernel(ard_num_dims=train_x.shape[1])
                    )
                def forward(self, x):
                    return gpytorch.distributions.MultivariateNormal(
                        self.mean_module(x), self.covar_module(x)
                    )

            print(f"\nTraining gpytorch Exact GP (3 models, {len(X_train_gp_sc)} pts) ...")
            tx     = torch.tensor(X_train_gp_sc, dtype=torch.float32)
            test_x = torch.tensor(X_test_sc,     dtype=torch.float32)

            preds_list = []
            for out_i, name in enumerate(OUTPUT_NAMES):
                y_mean = Y_train_gp[:, out_i].mean()
                y_std  = Y_train_gp[:, out_i].std() + 1e-8
                ty_i   = torch.tensor((Y_train_gp[:, out_i] - y_mean) / y_std, dtype=torch.float32)

                lik_i   = gpytorch.likelihoods.GaussianLikelihood()
                model_i = ExactGP(tx, ty_i, lik_i)
                model_i.train(); lik_i.train()
                opt = torch.optim.Adam(model_i.parameters(), lr=0.1)
                mll = gpytorch.mlls.ExactMarginalLogLikelihood(lik_i, model_i)
                for _ in range(1000):
                    opt.zero_grad()
                    (-mll(model_i(tx), ty_i)).backward()
                    opt.step()

                with torch.no_grad():
                    amp = model_i.covar_module.outputscale.item() ** 0.5
                    ls  = model_i.covar_module.base_kernel.lengthscale.squeeze().numpy()
                    nz  = lik_i.noise.item()
                print(f"  {name}  amplitude={amp:.4f}  noise={nz:.6f}")
                print(f"    length_scale: {dict(zip(args.features, np.round(ls, 4)))}")

                model_i.eval(); lik_i.eval()
                with torch.no_grad(), gpytorch.settings.cholesky_jitter(1e-3):
                    pred_sc = lik_i(model_i(test_x)).mean.numpy()
                preds_list.append(pred_sc * y_std + y_mean)

            Y_pred_gpytorch = np.column_stack(preds_list)
            gp_label = "gpytorch ExactGP"

        else:
            # ── Inducing Point GP on all training data ─────────────────────────
            class InducingGP(gpytorch.models.ApproximateGP):
                def __init__(self, inducing_points):
                    var_dist  = gpytorch.variational.CholeskyVariationalDistribution(
                        inducing_points.size(0)
                    )
                    var_strat = gpytorch.variational.VariationalStrategy(
                        self, inducing_points, var_dist, learn_inducing_locations=True
                    )
                    super().__init__(var_strat)
                    self.mean_module  = gpytorch.means.ZeroMean()
                    self.covar_module = gpytorch.kernels.ScaleKernel(
                        gpytorch.kernels.RBFKernel(ard_num_dims=inducing_points.size(-1))
                    )
                def forward(self, x):
                    return gpytorch.distributions.MultivariateNormal(
                        self.mean_module(x), self.covar_module(x)
                    )

            m = args.n_inducing_points
            print(f"\nTraining gpytorch Inducing GP (3 models, {len(X_train_sc)} pts, {m} inducing) ...")

            # Init inducing points with k-means
            try:
                from scipy.cluster.vq import kmeans
                X_w = (X_train_sc - X_train_sc.mean(0)) / (X_train_sc.std(0) + 1e-8)
                ind_np, _ = kmeans(X_w, m, iter=20)
                ind_np = ind_np * (X_train_sc.std(0) + 1e-8) + X_train_sc.mean(0)
            except Exception:
                ind_np = X_train_sc[rng.choice(len(X_train_sc), m, replace=False)]

            tx     = torch.tensor(X_train_sc, dtype=torch.float32)
            test_x = torch.tensor(X_test_sc,  dtype=torch.float32)
            ind_t  = torch.tensor(ind_np,      dtype=torch.float32)

            preds_list = []
            for out_i, name in enumerate(OUTPUT_NAMES):
                y_mean = Y_train[:, out_i].mean()
                y_std  = Y_train[:, out_i].std() + 1e-8
                ty_i   = torch.tensor((Y_train[:, out_i] - y_mean) / y_std, dtype=torch.float32)

                lik_i   = gpytorch.likelihoods.GaussianLikelihood()
                model_i = InducingGP(ind_t.clone())
                model_i.train(); lik_i.train()
                opt = torch.optim.Adam(
                    list(model_i.parameters()) + list(lik_i.parameters()), lr=0.05
                )
                mll = gpytorch.mlls.VariationalELBO(lik_i, model_i, num_data=len(ty_i))
                for it in range(500):
                    opt.zero_grad()
                    (-mll(model_i(tx), ty_i)).backward()
                    opt.step()
                    if (it + 1) % 100 == 0:
                        with torch.no_grad():
                            l = -mll(model_i(tx), ty_i).item()
                        print(f"    {name} iter {it+1}/500  loss={l:.4f}")

                with torch.no_grad():
                    amp = model_i.covar_module.outputscale.item() ** 0.5
                    ls  = model_i.covar_module.base_kernel.lengthscale.squeeze().numpy()
                    nz  = lik_i.noise.item()
                print(f"  {name}  amplitude={amp:.4f}  noise={nz:.6f}")
                print(f"    length_scale: {dict(zip(args.features, np.round(ls, 4)))}")

                model_i.eval(); lik_i.eval()
                with torch.no_grad(), gpytorch.settings.fast_pred_var():
                    pred_sc = lik_i(model_i(test_x)).mean.numpy()
                preds_list.append(pred_sc * y_std + y_mean)

            Y_pred_gpytorch = np.column_stack(preds_list)
            gp_label = f"gpytorch InducingGP ({m})"

        print(f"  {gp_label} done.")

    except ImportError:
        print("  gpytorch not installed — skipping. Run: pip install gpytorch")

# ── RMSE comparison ───────────────────────────────────────────────────────────
print("\n── RMSE comparison (baseline = predict 0) ────────────────────────────────")
active = []
if Y_pred_sklearn  is not None: active.append(("sklearn GP",  Y_pred_sklearn))
if Y_pred_mlp      is not None: active.append(("MLP",         Y_pred_mlp))
if Y_pred_gpytorch is not None: active.append((gp_label,      Y_pred_gpytorch))

header = f"{'Output':<12} {'No model':>10}" + "".join(
    f" {lbl:>14} {'Improv':>8}" for lbl, _ in active
)
print(header)
for i, name in enumerate(OUTPUT_NAMES):
    rmse_base = np.sqrt(np.mean(Y_test[:, i] ** 2))
    row = f"{name:<12} {rmse_base:>10.6f}"
    for lbl, Y_pred in active:
        rmse = np.sqrt(np.mean((Y_test[:, i] - Y_pred[:, i]) ** 2))
        imp  = (rmse_base - rmse) / rmse_base * 100
        row += f" {rmse:>14.6f} {imp:>+7.1f}%"
    print(row)

# ── Plots ─────────────────────────────────────────────────────────────────────
all_models = active

n_models = len(all_models)
fig, axes = plt.subplots(n_models, 3, figsize=(14, 5 * n_models))
if n_models == 1:
    axes = axes[np.newaxis, :]

src = "test file" if test_chunks else "random 20% split"
fig.suptitle(f"Model comparison: predicted vs actual residual ({src})", fontsize=12)

for col_i, (name, color) in enumerate(zip(OUTPUT_NAMES, COLORS)):
    for row_i, (label, Y_pred) in enumerate(all_models):
        ax     = axes[row_i, col_i]
        actual = Y_test[:, col_i]
        pred   = Y_pred[:, col_i]
        lim = float(np.percentile(np.abs(np.concatenate([actual, pred])), 99)) * 1.1
        ax.scatter(actual, pred, s=8, alpha=0.4, color=color)
        ax.plot([-lim, lim], [-lim, lim], "k--", linewidth=1, label="perfect")
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim)
        ax.set_xlabel(f"actual {name}")
        ax.set_ylabel(f"predicted {name}")
        rmse_base = np.sqrt(np.mean(actual ** 2))
        rmse      = np.sqrt(np.mean((actual - pred) ** 2))
        ax.set_title(f"{label} — {name}\nno-model={rmse_base:.5f}  {label}={rmse:.5f}")
        ax.legend(fontsize=8)
        ax.set_aspect("equal")

plt.tight_layout()
plt.savefig(args.out, dpi=150)
print(f"\nSaved → {args.out}")
plt.show()
