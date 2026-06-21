#pragma once

#include <array>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace crs_controls::pacejka_mpcc
{

/**
 * Zero-order GP residual model for the Pacejka dynamics.
 *
 * Predicts [d_vx, d_vy, d_omega] — the error between the Pacejka model
 * and the real car — given features [vx, vy, omega, delta, T].
 *
 * Model file (gp_model.bin) is produced by train_gp.py.
 *
 * Inference:
 *   x_scaled = (x - scaler_mean) / scaler_std
 *   k[i]     = amplitude^2 * exp(-0.5 * sum_j((x_scaled[j] - X_train[i,j])^2 / l[j]^2))
 *   pred     = k @ alpha * y_std + y_mean
 */
class GPResidual
{
public:
  static constexpr int N_FEATURES = 5;  // [vx, vy, omega, delta, T]
  static constexpr int N_OUTPUTS  = 3;  // [d_vx, d_vy, d_omega]

  struct Residual
  {
    double d_vx   = 0.0;
    double d_vy   = 0.0;
    double d_omega = 0.0;
  };

  GPResidual() = default;

  bool load(const std::string& path)
  {
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open())
    {
      std::cout << "[GPResidual] could not open " << path << std::endl;
      return false;
    }

    // Header: n_train, n_features, n_outputs
    int32_t header[3];
    f.read(reinterpret_cast<char*>(header), sizeof(header));
    n_train_ = header[0];

    if (header[1] != N_FEATURES || header[2] != N_OUTPUTS)
    {
      std::cout << "[GPResidual] unexpected dimensions in " << path << std::endl;
      return false;
    }

    // X_train: (n_train, N_FEATURES)
    X_train_.resize(n_train_ * N_FEATURES);
    f.read(reinterpret_cast<char*>(X_train_.data()), n_train_ * N_FEATURES * sizeof(double));

    // alpha: (n_train, N_OUTPUTS)
    alpha_.resize(n_train_ * N_OUTPUTS);
    f.read(reinterpret_cast<char*>(alpha_.data()), n_train_ * N_OUTPUTS * sizeof(double));

    f.read(reinterpret_cast<char*>(length_scale_.data()), N_FEATURES * sizeof(double));
    f.read(reinterpret_cast<char*>(&amplitude_),          sizeof(double));
    f.read(reinterpret_cast<char*>(scaler_mean_.data()),  N_FEATURES * sizeof(double));
    f.read(reinterpret_cast<char*>(scaler_std_.data()),   N_FEATURES * sizeof(double));
    f.read(reinterpret_cast<char*>(y_mean_.data()),       N_OUTPUTS  * sizeof(double));
    f.read(reinterpret_cast<char*>(y_std_.data()),        N_OUTPUTS  * sizeof(double));

    loaded_ = true;
    std::cout << "[GPResidual] loaded " << path
              << "  (n_train=" << n_train_ << ")" << std::endl;
    return true;
  }

  bool isLoaded() const { return loaded_; }

  Residual predict(double vx, double vy, double omega, double delta, double T) const
  {
    if (!loaded_)
      return {};

    // Scale input
    const double x_raw[N_FEATURES] = { vx, vy, omega, delta, T };
    double x[N_FEATURES];
    for (int j = 0; j < N_FEATURES; ++j)
      x[j] = (x_raw[j] - scaler_mean_[j]) / scaler_std_[j];

    // Kernel vector k[i] = amplitude^2 * exp(-0.5 * sum_j (x[j]-X[i,j])^2 / l[j]^2)
    double amp2 = amplitude_ * amplitude_;
    double out[N_OUTPUTS] = { 0.0, 0.0, 0.0 };

    for (int i = 0; i < n_train_; ++i)
    {
      double dist2 = 0.0;
      const double* row = &X_train_[i * N_FEATURES];
      for (int j = 0; j < N_FEATURES; ++j)
      {
        double diff = x[j] - row[j];
        double l    = length_scale_[j];
        dist2 += diff * diff / (l * l);
      }
      double k_i = amp2 * std::exp(-0.5 * dist2);

      for (int o = 0; o < N_OUTPUTS; ++o)
        out[o] += k_i * alpha_[i * N_OUTPUTS + o];
    }

    // Unnormalise: pred = out * y_std + y_mean
    return {
      out[0] * y_std_[0] + y_mean_[0],
      out[1] * y_std_[1] + y_mean_[1],
      out[2] * y_std_[2] + y_mean_[2],
    };
  }

private:
  bool loaded_ = false;
  int  n_train_ = 0;

  std::vector<double> X_train_;  // (n_train, N_FEATURES) row-major
  std::vector<double> alpha_;    // (n_train, N_OUTPUTS)  row-major

  std::array<double, N_FEATURES> length_scale_{};
  double amplitude_ = 1.0;
  std::array<double, N_FEATURES> scaler_mean_{};
  std::array<double, N_FEATURES> scaler_std_{};
  std::array<double, N_OUTPUTS>  y_mean_{};
  std::array<double, N_OUTPUTS>  y_std_{};
};

}  // namespace crs_controls::pacejka_mpcc
