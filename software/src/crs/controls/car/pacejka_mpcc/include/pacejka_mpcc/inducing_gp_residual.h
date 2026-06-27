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
 * Three separate GP models (one per output: eps_vx, eps_vy, eps_omega),
 * each with its own learnable inducing points Z_o, variational alpha
 * alpha_ind_o = K(Z,Z)^{-1} @ m_u, amplitude, and ARD length-scales.
 *
 * Produced by train_inducing_gp.py.  Binary (.bin) layout:
 *
 *   int32[3]      n_inducing, n_features, n_outputs
 *   uint8[d]      feature_indices — indices into ALL_FEATURES[]
 *   float64[2]    lf, lr
 *   for o in 0..2:
 *     float64[m*d]  Z_o           (inducing points, row-major)
 *     float64[m]    alpha_ind_o   (precomputed K(Z,Z)^{-1} @ m_u)
 *     float64       amplitude_o
 *     float64[d]    length_scale_o
 *   float64[d]    scaler_mean
 *   float64[d]    scaler_std
 *   float64[3]    y_mean
 *   float64[3]    y_std
 *
 * ALL_FEATURES (canonical sorted order, matches Python sorted()):
 *   0=T  1=alpha_f  2=alpha_r  3=beta  4=delta  5=omega  6=vx  7=vy
 *
 * Inference for output o at scaled input x_sc:
 *   k[i]   = amp_o^2 * exp(-0.5 * sum_j (x_sc[j] - Z_o[i,j])^2 / ls_o[j]^2)
 *   pred_o = dot(k, alpha_ind_o) * y_std_o + y_mean_o
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

  Residual predict(double vx, double vy, double omega, double delta, double T) const
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

    // Per-output kernel inference
    double out[N_OUTPUTS] = {};
    for (int o = 0; o < N_OUTPUTS; ++o)
    {
      const double amp2 = amplitude_[o] * amplitude_[o];
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
        out[o] += amp2 * std::exp(-0.5 * dist2) * alpha_ind_[o][i];
      }
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
  std::array<std::vector<double>, N_OUTPUTS> Z_;            // [o]: m×d row-major
  std::array<std::vector<double>, N_OUTPUTS> alpha_ind_;    // [o]: m
  std::array<double,              N_OUTPUTS> amplitude_{};
  std::array<std::vector<double>, N_OUTPUTS> length_scale_;

  std::vector<double>             scaler_mean_;
  std::vector<double>             scaler_std_;
  std::array<double, N_OUTPUTS>   y_mean_{};
  std::array<double, N_OUTPUTS>   y_std_{};
};

}  // namespace crs_controls::pacejka_mpcc
