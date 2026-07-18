"""
Compare predictions of an already-trained MLP and InducingGP residual model on a
test dataset, without retraining anything.

Loads the exact .bin artifacts consumed by the C++ controller (MLPResidual /
InducingGPResidual), so the metrics reflect precisely what would run on the car.

Usage:
    python3 compare_saved_models.py \
        --mlp-bin models/mlp_final/mlp_model_0.bin \
        --inducing-gp-bin models/gp_final/inducing_gp_model_0.bin \
        --test-data data/mlp_final/residuals_test_0.csv \
        --out results/model_comparison.csv
"""

import argparse
import struct
import numpy as np

OUTPUT_NAMES = ["eps_vx", "eps_vy", "eps_omega"]
# ALL_FEATURES order used by InducingGPResidual's feature_indices
ALL_FEATURES = ["T", "alpha_f", "alpha_r", "beta", "delta", "omega", "vx", "vy"]

parser = argparse.ArgumentParser()
parser.add_argument("--mlp-bin", required=True, help="path to mlp_model_*.bin")
parser.add_argument("--inducing-gp-bin", required=True, help="path to inducing_gp_model_*.bin")
parser.add_argument("--test-data", required=True,
                    help="CSV with columns t,vx,vy,omega,delta,T,eps_vx,eps_vy,eps_omega,...")
parser.add_argument("--out", default="model_comparison.csv", help="where to save the metrics table")
parser.add_argument("--vx-min", type=float, default=0.5, help="drop rows with vx below this")
args = parser.parse_args()


def slip_angles(vx, vy, omega, delta, lf, lr):
    vx_safe = np.maximum(vx, 0.1)
    beta    = np.arctan2(vy, vx_safe)
    alpha_f = delta - np.arctan2(vy + lf * omega, vx_safe)
    alpha_r = -np.arctan2(vy - lr * omega, vx_safe)
    return beta, alpha_f, alpha_r


class MLPModel:
    def __init__(self, path):
        with open(path, "rb") as f:
            (n_layers,) = struct.unpack("<i", f.read(4))
            sizes = np.frombuffer(f.read(4 * (n_layers + 1)), dtype=np.int32)
            n_in, n_out = int(sizes[0]), int(sizes[-1])
            self.x_mean = np.frombuffer(f.read(8 * n_in), dtype=np.float64).copy()
            self.x_std  = np.frombuffer(f.read(8 * n_in), dtype=np.float64).copy()
            self.y_mean = np.frombuffer(f.read(8 * n_out), dtype=np.float64).copy()
            self.y_std  = np.frombuffer(f.read(8 * n_out), dtype=np.float64).copy()
            self.W, self.b = [], []
            for i in range(n_layers):
                n_r, n_c = int(sizes[i + 1]), int(sizes[i])
                W = np.frombuffer(f.read(8 * n_r * n_c), dtype=np.float64).reshape(n_r, n_c).copy()
                b = np.frombuffer(f.read(8 * n_r), dtype=np.float64).copy()
                self.W.append(W)
                self.b.append(b)
        self.n_in = n_in
        self.n_layers = n_layers
        print(f"[MLP] loaded {path}  n_in={n_in}  arch={list(sizes)}")

    def predict(self, X8):
        # X8 columns: [vx, vy, omega, delta, T, beta, alpha_f, alpha_r]
        X = X8 if self.n_in == 8 else X8[:, :5]
        act = (X - self.x_mean) / self.x_std
        for i in range(self.n_layers):
            pre = act @ self.W[i].T + self.b[i]
            act = np.tanh(pre) if i < self.n_layers - 1 else pre
        return act * self.y_std + self.y_mean


class InducingGPModel:
    def __init__(self, path):
        with open(path, "rb") as f:
            m, d, n_out = struct.unpack("<iii", f.read(12))
            feat_idx = np.frombuffer(f.read(d), dtype=np.uint8).copy()
            lf, lr = struct.unpack("<dd", f.read(16))
            self.Z, self.alpha_ind, self.amp, self.ls, self.L_ZZ_inv, self.LS_T_LZZinv = [], [], [], [], [], []
            for _ in range(n_out):
                Z = np.frombuffer(f.read(8 * m * d), dtype=np.float64).reshape(m, d).copy()
                alpha = np.frombuffer(f.read(8 * m), dtype=np.float64).copy()
                (amp,) = struct.unpack("<d", f.read(8))
                ls = np.frombuffer(f.read(8 * d), dtype=np.float64).copy()
                L_ZZ_inv = np.frombuffer(f.read(8 * m * m), dtype=np.float64).reshape(m, m).copy()
                LS_T_LZZinv = np.frombuffer(f.read(8 * m * m), dtype=np.float64).reshape(m, m).copy()
                self.Z.append(Z); self.alpha_ind.append(alpha); self.amp.append(amp)
                self.ls.append(ls); self.L_ZZ_inv.append(L_ZZ_inv); self.LS_T_LZZinv.append(LS_T_LZZinv)
            self.scaler_mean = np.frombuffer(f.read(8 * d), dtype=np.float64).copy()
            self.scaler_std  = np.frombuffer(f.read(8 * d), dtype=np.float64).copy()
            self.y_mean = np.frombuffer(f.read(8 * n_out), dtype=np.float64).copy()
            self.y_std  = np.frombuffer(f.read(8 * n_out), dtype=np.float64).copy()
        self.m, self.d, self.n_out = m, d, n_out
        self.lf, self.lr = lf, lr
        self.feat_idx = feat_idx
        print(f"[InducingGP] loaded {path}  m={m}  d={d}  features={[ALL_FEATURES[i] for i in feat_idx]}")

    def predict(self, X_all, use_uncertainty_scaling=True):
        # X_all columns follow ALL_FEATURES order: [T, alpha_f, alpha_r, beta, delta, omega, vx, vy]
        X = X_all[:, self.feat_idx]
        x_sc = (X - self.scaler_mean) / self.scaler_std
        out = np.zeros((X.shape[0], self.n_out))
        scales = np.zeros((X.shape[0], self.n_out))
        for o in range(self.n_out):
            diff = x_sc[:, None, :] - self.Z[o][None, :, :]
            dist2 = np.sum(diff ** 2 / (self.ls[o] ** 2)[None, None, :], axis=2)
            amp2 = self.amp[o] ** 2
            k = amp2 * np.exp(-0.5 * dist2)                       # (n, m)
            mean = k @ self.alpha_ind[o]                            # (n,)
            k_v = k @ self.L_ZZ_inv[o].T
            lsk = k @ self.LS_T_LZZinv[o].T
            sigma2 = np.clip(amp2 - np.sum(k_v ** 2, axis=1) + np.sum(lsk ** 2, axis=1), 0, amp2)
            scale = np.clip(1.0 - sigma2 / amp2, 0.0, None)
            scales[:, o] = scale
            out[:, o] = mean * (scale if use_uncertainty_scaling else 1.0)
        return out * self.y_std + self.y_mean, scales


def rmse(y_true, y_pred):
    return np.sqrt(np.mean((y_true - y_pred) ** 2, axis=0))


def mae(y_true, y_pred):
    return np.mean(np.abs(y_true - y_pred), axis=0)


# ── Load test data ──────────────────────────────────────────────────────────
data = np.loadtxt(args.test_data, delimiter=",", skiprows=1)
data = data[data[:, 1] >= args.vx_min]
print(f"[data] {args.test_data}: {len(data)} rows after vx >= {args.vx_min} filter")

vx, vy, omega, delta, T = data[:, 1], data[:, 2], data[:, 3], data[:, 4], data[:, 5]
Y_true = data[:, 6:9]  # [eps_vx, eps_vy, eps_omega]

mlp = MLPModel(args.mlp_bin)
gp  = InducingGPModel(args.inducing_gp_bin)

beta, alpha_f, alpha_r = slip_angles(vx, vy, omega, delta, gp.lf, gp.lr)
X8 = np.column_stack([vx, vy, omega, delta, T, beta, alpha_f, alpha_r])
X_all = np.column_stack([T, alpha_f, alpha_r, beta, delta, omega, vx, vy])  # ALL_FEATURES order

Y_mlp = mlp.predict(X8)
Y_gp, gp_scales = gp.predict(X_all, use_uncertainty_scaling=True)

rmse_base = rmse(Y_true, np.zeros_like(Y_true))
rmse_mlp  = rmse(Y_true, Y_mlp)
rmse_gp   = rmse(Y_true, Y_gp)
mae_base  = mae(Y_true, np.zeros_like(Y_true))
mae_mlp   = mae(Y_true, Y_mlp)
mae_gp    = mae(Y_true, Y_gp)

rows = ["output,no_model_rmse,mlp_rmse,mlp_improv_pct,inducing_gp_rmse,inducing_gp_improv_pct,"
        "no_model_mae,mlp_mae,inducing_gp_mae,inducing_gp_mean_confidence"]
print(f"\n{'output':<10}{'no-model RMSE':>15}{'MLP RMSE':>12}{'improv%':>10}"
      f"{'InducingGP RMSE':>17}{'improv%':>10}{'mean conf':>11}")
for i, name in enumerate(OUTPUT_NAMES):
    improv_mlp = 100 * (rmse_base[i] - rmse_mlp[i]) / rmse_base[i]
    improv_gp  = 100 * (rmse_base[i] - rmse_gp[i]) / rmse_base[i]
    mean_conf  = gp_scales[:, i].mean()
    print(f"{name:<10}{rmse_base[i]:>15.5f}{rmse_mlp[i]:>12.5f}{improv_mlp:>10.1f}"
          f"{rmse_gp[i]:>17.5f}{improv_gp:>10.1f}{mean_conf:>11.3f}")
    rows.append(f"{name},{rmse_base[i]:.6f},{rmse_mlp[i]:.6f},{improv_mlp:.2f},"
                f"{rmse_gp[i]:.6f},{improv_gp:.2f},"
                f"{mae_base[i]:.6f},{mae_mlp[i]:.6f},{mae_gp[i]:.6f},{mean_conf:.4f}")

with open(args.out, "w") as f:
    f.write("\n".join(rows) + "\n")
print(f"\nSaved metrics -> {args.out}")
