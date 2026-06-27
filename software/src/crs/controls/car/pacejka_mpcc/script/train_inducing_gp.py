"""
Train an inducing-point GP residual model from MPCC combined log files.
Saves .npz (for inspection) and .bin (for C++ controller).

Three separate models are trained, one per output [eps_vx, eps_vy, eps_omega].

Usage:
    python3 train_inducing_gp.py
    python3 train_inducing_gp.py --n-datasets 3 --features omega alpha_f alpha_r vx
    python3 train_inducing_gp.py --n-inducing-points 200 --n-iter 1000
"""

import argparse
import numpy as np
import torch
import gpytorch
from pathlib import Path
from sklearn.preprocessing import StandardScaler

try:
    from sklearn.cluster import KMeans as _KMeans
    HAS_SKLEARN_KMEANS = True
except ImportError:
    HAS_SKLEARN_KMEANS = False

# Canonical feature order — sorted() of all feature names.
# Capital 'T' (ASCII 84) sorts before lowercase letters (ASCII 97+).
ALL_FEATURES  = ["T", "alpha_f", "alpha_r", "beta", "delta", "omega", "vx", "vy"]
FEATURE_TO_IDX = {f: i for i, f in enumerate(ALL_FEATURES)}

OUTPUT_NAMES = ["eps_vx", "eps_vy", "eps_omega"]

parser = argparse.ArgumentParser()
parser.add_argument("--data",              default="/code/src/crs/controls/car/pacejka_mpcc/script/data/gp_final/residuals_0.csv")
parser.add_argument("--out",               default=str(Path(__file__).parent / "models/gp_final/inducing_gp_model.npz"))
parser.add_argument("--n-inducing-points", type=int,   default=300)
parser.add_argument("--n-iter",            type=int,   default=1000)
parser.add_argument("--step-size",         type=float, default=0.05, help="Adam learning rate")
parser.add_argument("--vx-min",            type=float, default=1.0)
parser.add_argument("--n-datasets",        type=int,   default=1)
parser.add_argument("--seed",              type=int,   default=42)
parser.add_argument("--lf",               type=float, default=0.052, help="front axle distance [m]")
parser.add_argument("--lr",               type=float, default=0.038, help="rear axle distance [m]")
parser.add_argument("--features",         type=str,   nargs="+",
                    default=["vx", "vy", "omega", "delta", "T", "alpha_f", "alpha_r"],
                    choices=ALL_FEATURES)
args = parser.parse_args()
args.features = sorted(args.features)

torch.manual_seed(args.seed)
rng = np.random.default_rng(args.seed)

feature_indices = np.array([FEATURE_TO_IDX[f] for f in args.features], dtype=np.uint8)


def slip_angles(vx, vy, omega, delta):
    vx_safe = np.maximum(vx, 0.1)
    beta    = np.arctan2(vy, vx_safe)
    alpha_f = delta - np.arctan2(vy + args.lf * omega, vx_safe)
    alpha_r = -np.arctan2(vy - args.lr * omega, vx_safe)
    return beta, alpha_f, alpha_r


def build_features(data):
    # columns: t,vx,vy,omega,delta,T,eps_vx,eps_vy,eps_omega,...
    raw = data[:, 1:6]
    beta, alpha_f, alpha_r = slip_angles(raw[:, 0], raw[:, 1], raw[:, 2], raw[:, 3])
    ALL = {
        "T": raw[:, 4], "alpha_f": alpha_f, "alpha_r": alpha_r, "beta": beta,
        "delta": raw[:, 3], "omega": raw[:, 2], "vx": raw[:, 0], "vy": raw[:, 1],
    }
    X = np.column_stack([ALL[f] for f in args.features])
    Y = data[:, 6:9]
    return X, Y


# ── Load training data ─────────────────────────────────────────────────────────
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

mask_vx = data[:, 1] >= args.vx_min
data    = data[mask_vx]
weights = weights[mask_vx]
print(f"  {len(data)} points after vx >= {args.vx_min} filter")

X_all, Y_all = build_features(data)
print(f"  features ({len(args.features)}): {args.features}")
print(f"  feature_indices: {feature_indices.tolist()}")

# ── Load test data ─────────────────────────────────────────────────────────────
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
    X_test,  Y_test  = X_all[test_idx], Y_all[test_idx]
    X_all,   Y_all   = X_all[tr_idx],   Y_all[tr_idx]
    weights          = weights[tr_idx]

print(f"\nTrain: {len(X_all)}  Test: {len(X_test)}  Inducing: {args.n_inducing_points}")

# ── Scale features ─────────────────────────────────────────────────────────────
scaler    = StandardScaler()
X_sc      = scaler.fit_transform(X_all)
X_test_sc = scaler.transform(X_test)

# ── Inducing point initialization (k-means on training data) ───────────────────
m = args.n_inducing_points
if HAS_SKLEARN_KMEANS:
    try:
        km = _KMeans(n_clusters=m, random_state=args.seed, n_init=1, max_iter=100)
        km.fit(X_sc)
        ind_np = km.cluster_centers_
        print("  inducing init: k-means (sklearn, seeded)")
    except Exception:
        ind_np = X_sc[rng.choice(len(X_sc), m, replace=False)]
        print("  inducing init: random (k-means failed)")
else:
    ind_np = X_sc[rng.choice(len(X_sc), m, replace=False)]
    print("  inducing init: random (sklearn unavailable)")

# ── gpytorch model definition ──────────────────────────────────────────────────
class InducingGP(gpytorch.models.ApproximateGP):
    def __init__(self, inducing_points):
        var_dist  = gpytorch.variational.CholeskyVariationalDistribution(inducing_points.size(0))
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


tx     = torch.tensor(X_sc,      dtype=torch.float32)
test_x = torch.tensor(X_test_sc, dtype=torch.float32)
ind_t  = torch.tensor(ind_np,    dtype=torch.float32)

# ── Train one model per output ─────────────────────────────────────────────────
Z_list, alpha_ind_list, amplitude_list, ls_list = [], [], [], []
y_mean_list, y_std_list = [], []

print(f"\nTraining {len(OUTPUT_NAMES)} inducing GPs ({len(X_all)} pts, {m} inducing) ...")

for out_i, name in enumerate(OUTPUT_NAMES):
    y_mean = float(Y_all[:, out_i].mean())
    y_std  = float(Y_all[:, out_i].std()) + 1e-8
    ty_i   = torch.tensor((Y_all[:, out_i] - y_mean) / y_std, dtype=torch.float32)

    lik_i   = gpytorch.likelihoods.GaussianLikelihood()
    model_i = InducingGP(ind_t.clone())
    model_i.train(); lik_i.train()

    opt = torch.optim.Adam(
        list(model_i.parameters()) + list(lik_i.parameters()), lr=args.step_size
    )
    warmup_iters = args.n_iter // 10
    def lr_lambda(it):
        if it < warmup_iters:
            return it / warmup_iters
        t = (it - warmup_iters) / (args.n_iter - warmup_iters)
        return 0.01 + 0.99 * 0.5 * (1 + torch.cos(torch.tensor(t * 3.14159)).item())
    scheduler = torch.optim.lr_scheduler.LambdaLR(opt, lr_lambda)
    mll = gpytorch.mlls.VariationalELBO(lik_i, model_i, num_data=len(ty_i))

    for it in range(args.n_iter):
        opt.zero_grad()
        (-mll(model_i(tx), ty_i)).backward()
        opt.step()
        scheduler.step()
        if (it + 1) % 100 == 0:
            with torch.no_grad():
                l = -mll(model_i(tx), ty_i).item()
            lr_now = scheduler.get_last_lr()[0]
            print(f"  {name}  iter {it + 1}/{args.n_iter}  loss={l:.4f}  lr={lr_now:.5f}")

    # Extract parameters for C++ inference
    model_i.eval(); lik_i.eval()
    with torch.no_grad():
        Z_o   = model_i.variational_strategy.inducing_points.detach()       # m × d
        m_u_o = model_i.variational_strategy._variational_distribution.variational_mean.detach()  # m
        K_ZZ      = model_i.covar_module(Z_o).evaluate()
        K_ZZ_diag = K_ZZ.diagonal().mean().item()
        jitter    = max(K_ZZ_diag * 1e-4, 1e-7)
        L         = torch.linalg.cholesky(K_ZZ + torch.eye(m) * jitter)
        # VariationalStrategy is whitened: variational_mean is m_tilde in whitened space.
        # Predictive mean = K(x*,Z) @ L^{-T} @ m_tilde, so alpha_ind = L^{-T} @ m_tilde.
        alpha_ind = torch.linalg.solve_triangular(L.T, m_u_o.unsqueeze(-1), upper=True).squeeze(-1).numpy()
        alpha_norm = float(np.linalg.norm(alpha_ind))
        print(f"    K_ZZ diag~{K_ZZ_diag:.4f}  ||alpha_ind||={alpha_norm:.3f}")
        if alpha_norm > 1e2:
            print(f"    WARNING: alpha_ind norm={alpha_norm:.1f} large — use residual_scale<=0.3 on first test")

        amp = float(model_i.covar_module.outputscale.item() ** 0.5)
        ls  = model_i.covar_module.base_kernel.lengthscale.squeeze().numpy()

        pred_sc = lik_i(model_i(test_x)).mean.numpy()
        pred    = pred_sc * y_std + y_mean
        rmse_base = np.sqrt(np.mean(Y_test[:, out_i] ** 2))
        rmse      = np.sqrt(np.mean((Y_test[:, out_i] - pred) ** 2))

    print(f"  {name}  amplitude={amp:.4f}  noise={lik_i.noise.item():.6f}")
    print(f"    length_scale: {dict(zip(args.features, np.round(ls, 4)))}")
    print(f"    RMSE: no-model={rmse_base:.6f}  GP={rmse:.6f}  improv={100*(rmse_base-rmse)/rmse_base:.1f}%")

    Z_list.append(Z_o.numpy())
    alpha_ind_list.append(alpha_ind)
    amplitude_list.append(amp)
    ls_list.append(ls)
    y_mean_list.append(y_mean)
    y_std_list.append(y_std)

# ── Save .npz ──────────────────────────────────────────────────────────────────
stem     = args.out[:-len(".npz")]
npz_path = f"{stem}_{args.n_datasets - 1}.npz"
Path(npz_path).parent.mkdir(parents=True, exist_ok=True)
np.savez(npz_path,
         features        = np.array(args.features),
         feature_indices = feature_indices,
         lf              = np.array([args.lf]),
         lr              = np.array([args.lr]),
         Z               = np.stack(Z_list),             # (3, m, d)
         alpha_ind       = np.stack(alpha_ind_list),     # (3, m)
         amplitude       = np.array(amplitude_list),     # (3,)
         length_scale    = np.stack(ls_list),            # (3, d)
         scaler_mean     = scaler.mean_,
         scaler_std      = scaler.scale_,
         y_mean          = np.array(y_mean_list),
         y_std           = np.array(y_std_list))
print(f"\nSaved → {npz_path}")

# ── Save .bin for C++ inference ────────────────────────────────────────────────
# Format:
#   int32[3]       : n_inducing, n_features, n_outputs
#   uint8[d]       : feature_indices  (into ALL_FEATURES order)
#   float64[2]     : lf, lr
#   for o in 0..2:
#     float64[m*d] : Z_o  (row-major)
#     float64[m]   : alpha_ind_o
#     float64      : amplitude_o
#     float64[d]   : length_scale_o
#   float64[d]     : scaler_mean
#   float64[d]     : scaler_std
#   float64[3]     : y_mean
#   float64[3]     : y_std
bin_path  = npz_path.replace(".npz", ".bin")
d = len(args.features)

with open(bin_path, "wb") as f:
    np.array([m, d, 3], dtype=np.int32).tofile(f)
    feature_indices.tofile(f)
    np.array([args.lf, args.lr], dtype=np.float64).tofile(f)
    for o in range(3):
        Z_list[o].astype(np.float64).tofile(f)
        alpha_ind_list[o].astype(np.float64).tofile(f)
        np.array([amplitude_list[o]], dtype=np.float64).tofile(f)
        ls_list[o].astype(np.float64).tofile(f)
    scaler.mean_.astype(np.float64).tofile(f)
    scaler.scale_.astype(np.float64).tofile(f)
    np.array(y_mean_list, dtype=np.float64).tofile(f)
    np.array(y_std_list,  dtype=np.float64).tofile(f)

print(f"Saved → {bin_path}  (for C++ inference)")
print(f"  n_inducing={m}  n_features={d}  features={args.features}")
