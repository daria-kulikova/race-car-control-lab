"""
Train MLP residual model using PyTorch with SmoothL1 (Huber) loss.

Saves weights in the same .npz / .bin format as before,
so C++ inference and CasADi embedding are unchanged.

Usage:
    python3 train_mlp.py
    python3 train_mlp.py --n-datasets 3 --epochs 3000
"""

import argparse
import numpy as np
from pathlib import Path
from sklearn.preprocessing import StandardScaler
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset, WeightedRandomSampler

parser = argparse.ArgumentParser()
parser.add_argument("--data",          default="/code/src/crs/controls/car/pacejka_mpcc/script/data/mpcc_residuals_cm1_0555_0.csv")
parser.add_argument("--out",           default=str(Path(__file__).parent / "mlp_model.npz"))
parser.add_argument("--vx-min",        type=float, default=0.5)
parser.add_argument("--iqr-k",         type=float, default=7.0,   help="IQR multiplier for outlier removal")
parser.add_argument("--hidden",        type=int,   nargs="+",     default=[64, 64, 32])
parser.add_argument("--n-datasets",    type=int,   default=1,     help="number of datasets (*_0.csv, *_1.csv, ...); dataset i gets weight i+1")
parser.add_argument("--lf",            type=float, default=0.052, help="front axle distance [m]")
parser.add_argument("--lr",            type=float, default=0.038, help="rear axle distance [m]")
parser.add_argument("--learning-rate", type=float, default=1e-3)
parser.add_argument("--epochs",        type=int,   default=2000)
parser.add_argument("--batch-size",    type=int,   default=256)
parser.add_argument("--patience",      type=int,   default=50,   help="early stopping patience in epochs")
parser.add_argument("--huber-beta",    type=float, default=1.0,  help="SmoothL1 transition point (in scaled space)")
parser.add_argument("--weight-decay",  type=float, default=1e-4)
parser.add_argument("--seed",          type=int,   default=42)
parser.add_argument("--features",      type=str,   nargs="+",
                    default=["vx", "vy", "omega", "delta", "T", "beta", "alpha_f", "alpha_r"],
                    choices=["vx", "vy", "omega", "delta", "T", "beta", "alpha_f", "alpha_r"],
                    help="feature subset to use for training")
args = parser.parse_args()

torch.manual_seed(args.seed)
np.random.seed(args.seed)


def slip_angles(vx, vy, omega, delta, lf, lr):
    vx_safe = np.maximum(vx, 0.1)
    beta    = np.arctan2(vy, vx_safe)
    alpha_f = delta - np.arctan2(vy + lf * omega, vx_safe)
    alpha_r = -np.arctan2(vy - lr * omega, vx_safe)
    return beta, alpha_f, alpha_r


# ── Load ──────────────────────────────────────────────────────────────────────
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

# ── Outlier filtering ─────────────────────────────────────────────────────────
# mask = np.ones(len(data), dtype=bool)
# for c in [5, 6, 7]:
#     q25, q75 = np.percentile(data[:, c], [25, 75])
#     iqr = q75 - q25
#     mask &= (data[:, c] >= q25 - args.iqr_k * iqr) & (data[:, c] <= q75 + args.iqr_k * iqr)
# print(f"  {mask.sum()} points after outlier filter (removed {(~mask).sum()}, k={args.iqr_k})")
# data    = data[mask]
# weights = weights[mask]

# ── Features ──────────────────────────────────────────────────────────────────
raw = data[:, 1:6]   # [vx, vy, omega, delta, T]
beta, alpha_f, alpha_r = slip_angles(raw[:, 0], raw[:, 1], raw[:, 2], raw[:, 3], args.lf, args.lr)
ALL_FEATURES = {
    "vx": raw[:, 0], "vy": raw[:, 1], "omega": raw[:, 2],
    "delta": raw[:, 3], "T": raw[:, 4],
    "beta": beta, "alpha_f": alpha_f, "alpha_r": alpha_r,
}
X = np.column_stack([ALL_FEATURES[f] for f in args.features])
Y = data[:, 6:9]                                     # [eps_vx, eps_vy, eps_omega]
print(f"  features ({len(args.features)}): {args.features}")

# ── Load test data from *_test_{i}.csv files ──────────────────────────────────
test_base = base + "test_"
test_chunks = []
for i in range(args.n_datasets):
    p = test_base + f"{i}.csv"
    if Path(p).exists():
        test_chunks.append(np.loadtxt(p, delimiter=",", skiprows=1))
        print(f"  loaded test {p}: {len(test_chunks[-1])} rows")
if test_chunks:
    test_data = np.vstack(test_chunks)
    test_data = test_data[test_data[:, 1] >= args.vx_min]
    raw_t = test_data[:, 1:6]
    b_t, af_t, ar_t = slip_angles(raw_t[:, 0], raw_t[:, 1], raw_t[:, 2], raw_t[:, 3], args.lf, args.lr)
    ALL_T = {"vx": raw_t[:, 0], "vy": raw_t[:, 1], "omega": raw_t[:, 2],
             "delta": raw_t[:, 3], "T": raw_t[:, 4],
             "beta": b_t, "alpha_f": af_t, "alpha_r": ar_t}
    X_test_raw = np.column_stack([ALL_T[f] for f in args.features])
    Y_test_raw = test_data[:, 6:9]
    print(f"  test total: {len(X_test_raw)} points")
else:
    X_test_raw = Y_test_raw = None
    print("  no test files found — skipping test evaluation")

rng = np.random.default_rng(args.seed)

# ── Scale (fit only on train+val) ─────────────────────────────────────────────
x_scaler = StandardScaler()
X_sc = x_scaler.fit_transform(X)

y_scaler = StandardScaler()
Y_sc = y_scaler.fit_transform(Y)

# ── Train / val split ─────────────────────────────────────────────────────────
val_idx = rng.choice(len(X_sc), int(len(X_sc) * 0.1), replace=False)
tr_idx  = np.setdiff1d(np.arange(len(X_sc)), val_idx)

X_tr, Y_tr, w_tr = X_sc[tr_idx], Y_sc[tr_idx], weights[tr_idx]
X_val_t = torch.tensor(X_sc[val_idx], dtype=torch.float32)
Y_val_t = torch.tensor(Y_sc[val_idx], dtype=torch.float32)

sampler = WeightedRandomSampler(
    weights=torch.tensor(w_tr / w_tr.sum(), dtype=torch.float32),
    num_samples=len(w_tr),
    replacement=True,
)
loader = DataLoader(
    TensorDataset(torch.tensor(X_tr, dtype=torch.float32),
                  torch.tensor(Y_tr, dtype=torch.float32)),
    batch_size=args.batch_size,
    sampler=sampler,
)

# ── Model ─────────────────────────────────────────────────────────────────────
n_in, n_out = X.shape[1], Y.shape[1]
sizes = [n_in] + args.hidden + [n_out]

layer_list = []
for i in range(len(sizes) - 1):
    layer_list.append(nn.Linear(sizes[i], sizes[i + 1]))
    if i < len(sizes) - 2:
        layer_list.append(nn.Tanh())
model = nn.Sequential(*layer_list)

optimizer = torch.optim.Adam(model.parameters(),
                             lr=args.learning_rate,
                             weight_decay=args.weight_decay)
scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=args.epochs)
criterion = nn.SmoothL1Loss(beta=args.huber_beta)

# ── Train ─────────────────────────────────────────────────────────────────────
print(f"Training PyTorch MLP {sizes}  SmoothL1(beta={args.huber_beta}) ...")
best_val, best_state, patience_cnt = float("inf"), None, 0

for epoch in range(args.epochs):
    model.train()
    for xb, yb in loader:
        optimizer.zero_grad()
        criterion(model(xb), yb).backward()
        optimizer.step()
    scheduler.step()

    model.eval()
    with torch.no_grad():
        val_loss = criterion(model(X_val_t), Y_val_t).item()

    if val_loss < best_val:
        best_val  = val_loss
        best_state = {k: v.clone() for k, v in model.state_dict().items()}
        patience_cnt = 0
    else:
        patience_cnt += 1

    if (epoch + 1) % 100 == 0:
        print(f"  epoch {epoch + 1:4d}  val={val_loss:.6f}  best={best_val:.6f}  patience={patience_cnt}/{args.patience}")

    if patience_cnt >= args.patience:
        print(f"  early stopping at epoch {epoch + 1}")
        break

model.load_state_dict(best_state)
print(f"  best val loss: {best_val:.6f}")

# ── RMSE sanity check ─────────────────────────────────────────────────────────
model.eval()
with torch.no_grad():
    Y_pred_sc = model(torch.tensor(X_sc, dtype=torch.float32)).numpy()
Y_pred = y_scaler.inverse_transform(Y_pred_sc)
rmse_base = np.sqrt(np.mean(Y ** 2, axis=0))
rmse_mlp  = np.sqrt(np.mean((Y - Y_pred) ** 2, axis=0))
print("  [train]")
for name, rb, rm in zip(["eps_vx", "eps_vy", "eps_omega"], rmse_base, rmse_mlp):
    print(f"    {name}: no-model={rb:.6f}  mlp={rm:.6f}  improv={100*(rb-rm)/rb:.1f}%")

if X_test_raw is not None:
    X_test_sc = x_scaler.transform(X_test_raw)
    with torch.no_grad():
        Y_test_pred_sc = model(torch.tensor(X_test_sc, dtype=torch.float32)).numpy()
    Y_test_pred = y_scaler.inverse_transform(Y_test_pred_sc)
    rmse_base_t = np.sqrt(np.mean(Y_test_raw ** 2, axis=0))
    rmse_mlp_t  = np.sqrt(np.mean((Y_test_raw - Y_test_pred) ** 2, axis=0))
    print("  [test]")
    for name, rb, rm in zip(["eps_vx", "eps_vy", "eps_omega"], rmse_base_t, rmse_mlp_t):
        print(f"    {name}: no-model={rb:.6f}  mlp={rm:.6f}  improv={100*(rb-rm)/rb:.1f}%")

# ── Extract linear layer weights ──────────────────────────────────────────────
# PyTorch Linear.weight shape: (n_out, n_in) — already row-major W@x+b convention
linear_layers = [(m.weight.detach().numpy(), m.bias.detach().numpy())
                 for m in model if isinstance(m, nn.Linear)]

# ── Save .npz (for CasADi / pacejka_model.py) ────────────────────────────────
npz_dict = dict(
    x_mean=x_scaler.mean_,  x_std=x_scaler.scale_,
    y_mean=y_scaler.mean_,  y_std=y_scaler.scale_,
    hidden=np.array(args.hidden, dtype=np.int32),
)
for i, (W, b) in enumerate(linear_layers):
    npz_dict[f"W{i}"] = W   # (n_out, n_in) — no transpose needed unlike sklearn
    npz_dict[f"b{i}"] = b

stem     = args.out[:-len(".npz")]
npz_path = f"{stem}_{args.n_datasets - 1}.npz"
np.savez(npz_path, **npz_dict)
print(f"Saved → {npz_path}  (arch {sizes})")

# ── Save .bin (for C++ inference) ─────────────────────────────────────────────
bin_path = str(npz_path).replace(".npz", ".bin")
with open(bin_path, "wb") as f:
    np.array([len(linear_layers)], dtype=np.int32).tofile(f)
    np.array(sizes, dtype=np.int32).tofile(f)
    x_scaler.mean_.astype(np.float64).tofile(f)
    x_scaler.scale_.astype(np.float64).tofile(f)
    y_scaler.mean_.astype(np.float64).tofile(f)
    y_scaler.scale_.astype(np.float64).tofile(f)
    for W, b in linear_layers:
        W.astype(np.float64).tofile(f)
        b.astype(np.float64).tofile(f)

print(f"Saved → {bin_path}  (for C++ lag compensation)")
