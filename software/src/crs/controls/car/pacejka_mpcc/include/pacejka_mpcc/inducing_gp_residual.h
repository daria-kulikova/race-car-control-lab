#pragma once

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace crs_controls::pacejka_mpcc
{

/**
 * Inducing-point GP residual model for Pacejka dynamics.
 *
 * Three separate GP models (one per output: eps_vx, eps_vy, eps_omega).
 * Predictions are scaled by posterior confidence: uncertain regions get
 * less correction, falling back to the nominal model.
 *
 * Produced by train_inducing_gp.py.  Binary (.bin) layout:
 *
 *   int32[3]        n_inducing, n_features, n_outputs
 *   uint8[d]        feature_indices — indices into ALL_FEATURES[]
 *   float64[2]      lf, lr
 *   for o in 0..2:
 *     float64[m*d]  Z_o            (inducing points, row-major)
 *     float64[m]    alpha_ind_o    (= L_ZZ^{-T} @ m_tilde, whitened)
 *     float64       amplitude_o
 *     float64[d]    length_scale_o
 *     float64[m*m]  L_ZZ_inv_o    (L_ZZ^{-1}, row-major)
 *     float64[m*m]  LS_T_LZZinv_o (L_S^T @ L_ZZ^{-1}, row-major)
 *   float64[d]      scaler_mean
 *   float64[d]      scaler_std
 *   float64[3]      y_mean
 *   float64[3]      y_std
 *
 * ALL_FEATURES: 0=T 1=alpha_f 2=alpha_r 3=beta 4=delta 5=omega 6=vx 7=vy
 *
 * Inference:
 *   k[i]   = amp^2 * exp(-0.5 * sum_j (x[j]-Z[i,j])^2 / ls[j]^2)
 *   k_v[i] = sum_j L_ZZ_inv[i,j] * k[j]
 *   Lsk[i] = sum_j LS_T_LZZinv[i,j] * k[j]
 *   sigma2 = clamp(amp^2 - dot(k_v,k_v) + dot(Lsk,Lsk), 0, amp^2)
 *   scale  = max(0, 1 - sigma2/amp^2)
 *   pred_o = dot(k, alpha_ind) * scale * y_std + y_mean
 */
class InducingGPResidual
{
public:
  static constexpr int N_ALL_FEATURES = 8;
  static constexpr int N_OUTPUTS      = 3;

  struct Residual
  {
    double d_vx    = 0.0;
    double d_vy    = 0.0;
    double d_omega = 0.0;
  };

  InducingGPResidual() = default;

  bool load(const std::string& path)
  {
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open())
    {
      std::cout << "[InducingGP] could not open " << path << std::endl;
      return false;
    }

    int32_t header[3];
    f.read(reinterpret_cast<char*>(header), sizeof(header));
    n_inducing_ = header[0];
    n_features_ = header[1];
    if (header[2] != N_OUTPUTS)
    {
      std::cout << "[InducingGP] unexpected n_outputs=" << header[2] << std::endl;
      return false;
    }

    feature_indices_.resize(n_features_);
    f.read(reinterpret_cast<char*>(feature_indices_.data()), n_features_ * sizeof(uint8_t));

    f.read(reinterpret_cast<char*>(&lf_), sizeof(double));
    f.read(reinterpret_cast<char*>(&lr_), sizeof(double));

    for (int o = 0; o < N_OUTPUTS; ++o)
    {
      Z_[o].resize(n_inducing_ * n_features_);
      f.read(reinterpret_cast<char*>(Z_[o].data()), n_inducing_ * n_features_ * sizeof(double));

      alpha_ind_[o].resize(n_inducing_);
      f.read(reinterpret_cast<char*>(alpha_ind_[o].data()), n_inducing_ * sizeof(double));

      f.read(reinterpret_cast<char*>(&amplitude_[o]), sizeof(double));

      length_scale_[o].resize(n_features_);
      f.read(reinterpret_cast<char*>(length_scale_[o].data()), n_features_ * sizeof(double));

      L_ZZ_inv_[o].resize(n_inducing_ * n_inducing_);
      f.read(reinterpret_cast<char*>(L_ZZ_inv_[o].data()), n_inducing_ * n_inducing_ * sizeof(double));

      LS_T_LZZinv_[o].resize(n_inducing_ * n_inducing_);
      f.read(reinterpret_cast<char*>(LS_T_LZZinv_[o].data()), n_inducing_ * n_inducing_ * sizeof(double));
    }

    scaler_mean_.resize(n_features_);
    f.read(reinterpret_cast<char*>(scaler_mean_.data()), n_features_ * sizeof(double));

    scaler_std_.resize(n_features_);
    f.read(reinterpret_cast<char*>(scaler_std_.data()), n_features_ * sizeof(double));

    f.read(reinterpret_cast<char*>(y_mean_.data()), N_OUTPUTS * sizeof(double));
    f.read(reinterpret_cast<char*>(y_std_.data()),  N_OUTPUTS * sizeof(double));

    loaded_ = true;
    std::cout << "[InducingGP] loaded " << path
              << "  (m=" << n_inducing_ << "  d=" << n_features_ << ")" << std::endl;
    return true;
  }

  bool isLoaded() const { return loaded_; }

  Residual predict(double vx, double vy, double omega, double delta, double T,
                   bool use_uncertainty_scaling = true) const
  {
    if (!loaded_)
      return {};

    // Compute all 8 candidate features
    // ALL_FEATURES order: T=0, alpha_f=1, alpha_r=2, beta=3, delta=4, omega=5, vx=6, vy=7
    const double vx_safe = std::max(vx, 0.1);
    const double beta    = std::atan2(vy, vx_safe);
    const double alpha_f = delta - std::atan2(vy + lf_ * omega, vx_safe);
    const double alpha_r = -std::atan2(vy - lr_ * omega, vx_safe);
    const double all_raw[N_ALL_FEATURES] = { T, alpha_f, alpha_r, beta, delta, omega, vx, vy };

    // Select and scale the chosen features
    double x_sc[N_ALL_FEATURES] = {};
    for (int j = 0; j < n_features_; ++j)
      x_sc[j] = (all_raw[feature_indices_[j]] - scaler_mean_[j]) / scaler_std_[j];

    // Per-output kernel inference with confidence scaling
    double out[N_OUTPUTS] = {};
    for (int o = 0; o < N_OUTPUTS; ++o)
    {
      const double amp2 = amplitude_[o] * amplitude_[o];

      // Compute kernel vector k (m-vector)
      std::vector<double> k(n_inducing_);
      for (int i = 0; i < n_inducing_; ++i)
      {
        double dist2 = 0.0;
        const double* row = &Z_[o][i * n_features_];
        for (int j = 0; j < n_features_; ++j)
        {
          const double diff = x_sc[j] - row[j];
          const double l    = length_scale_[o][j];
          dist2 += diff * diff / (l * l);
        }
        k[i] = amp2 * std::exp(-0.5 * dist2);
      }

      // Mean: dot(k, alpha_ind)
      double mean = 0.0;
      for (int i = 0; i < n_inducing_; ++i)
        mean += k[i] * alpha_ind_[o][i];

      // Posterior variance: sigma2 = amp2 - ||k_v||^2 + ||Lsk||^2
      // k_v = L_ZZ_inv @ k,  Lsk = LS_T_LZZinv @ k
      double kv_sq = 0.0, lsk_sq = 0.0;
      for (int i = 0; i < n_inducing_; ++i)
      {
        double kv_i = 0.0, lsk_i = 0.0;
        const double* lzz_row = &L_ZZ_inv_[o][i * n_inducing_];
        const double* lst_row = &LS_T_LZZinv_[o][i * n_inducing_];
        for (int j = 0; j < n_inducing_; ++j)
        {
          kv_i  += lzz_row[j] * k[j];
          lsk_i += lst_row[j] * k[j];
        }
        kv_sq  += kv_i  * kv_i;
        lsk_sq += lsk_i * lsk_i;
      }
      double scale = 1.0;
      if (use_uncertainty_scaling)
      {
        const double sigma2 = std::max(0.0, std::min(amp2, amp2 - kv_sq + lsk_sq));
        scale = std::max(0.0, 1.0 - sigma2 / amp2);
      }

      out[o] = mean * scale;
    }

    return {
      out[0] * y_std_[0] + y_mean_[0],
      out[1] * y_std_[1] + y_mean_[1],
      out[2] * y_std_[2] + y_mean_[2],
    };
  }

private:
  bool   loaded_      = false;
  int    n_inducing_  = 0;
  int    n_features_  = 0;
  double lf_          = 0.052;
  double lr_          = 0.038;

  std::vector<uint8_t>                       feature_indices_;
  std::array<std::vector<double>, N_OUTPUTS> Z_;              // [o]: m×d row-major
  std::array<std::vector<double>, N_OUTPUTS> alpha_ind_;      // [o]: m
  std::array<double,              N_OUTPUTS> amplitude_{};
  std::array<std::vector<double>, N_OUTPUTS> length_scale_;
  std::array<std::vector<double>, N_OUTPUTS> L_ZZ_inv_;       // [o]: m×m row-major
  std::array<std::vector<double>, N_OUTPUTS> LS_T_LZZinv_;    // [o]: m×m row-major

  std::vector<double>             scaler_mean_;
  std::vector<double>             scaler_std_;
  std::array<double, N_OUTPUTS>   y_mean_{};
  std::array<double, N_OUTPUTS>   y_std_{};
};

}  // namespace crs_controls::pacejka_mpcc
