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
 * MLP residual model for Pacejka dynamics (lag compensation).
 *
 * Architecture: tanh(W2 @ tanh(W1 @ x + b1) + b2) → Wout @ h2 + bout
 * Predicts velocity residuals [eps_vx, eps_vy, eps_omega] in m/s.
 * Divide by sample_period to get acceleration corrections for the ODE.
 *
 * Model file (mlp_model.bin) produced by train_mlp.py.
 *
 * Binary layout:
 *   int32[4]:          n_in, h1, h2, n_out
 *   float64[n_in]:     x_mean
 *   float64[n_in]:     x_std
 *   float64[n_out]:    y_mean
 *   float64[n_out]:    y_std
 *   float64[h1*n_in]:  W1  (row major)
 *   float64[h1]:       b1
 *   float64[h2*h1]:    W2  (row major)
 *   float64[h2]:       b2
 *   float64[n_out*h2]: Wout (row major)
 *   float64[n_out]:    bout
 */
class MLPResidual
{
public:
  static constexpr int N_FEATURES = 5;  // [vx, vy, omega, delta, T]
  static constexpr int N_OUTPUTS  = 3;  // [eps_vx, eps_vy, eps_omega]

  struct Residual
  {
    double d_vx    = 0.0;
    double d_vy    = 0.0;
    double d_omega = 0.0;
  };

  MLPResidual() = default;

  bool load(const std::string& path)
  {
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open())
    {
      std::cout << "[MLPResidual] could not open " << path << std::endl;
      return false;
    }

    int32_t header[4];
    f.read(reinterpret_cast<char*>(header), sizeof(header));
    int n_in = header[0], h1 = header[1], h2 = header[2], n_out = header[3];

    if (n_in != N_FEATURES || n_out != N_OUTPUTS)
    {
      std::cout << "[MLPResidual] unexpected dimensions: n_in=" << n_in
                << " n_out=" << n_out << std::endl;
      return false;
    }
    h1_ = h1; h2_ = h2;

    auto read_vec = [&](std::vector<double>& v, int n) {
      v.resize(n);
      f.read(reinterpret_cast<char*>(v.data()), n * sizeof(double));
    };

    read_vec(x_mean_, n_in);
    read_vec(x_std_,  n_in);
    read_vec(y_mean_, n_out);
    read_vec(y_std_,  n_out);
    read_vec(W1_,     h1 * n_in);
    read_vec(b1_,     h1);
    read_vec(W2_,     h2 * h1);
    read_vec(b2_,     h2);
    read_vec(Wout_,   n_out * h2);
    read_vec(bout_,   n_out);

    loaded_ = true;
    std::cout << "[MLPResidual] loaded " << path
              << "  (arch=" << n_in << "-" << h1 << "-" << h2 << "-" << n_out << ")" << std::endl;
    return true;
  }

  bool isLoaded() const { return loaded_; }

  Residual predict(double vx, double vy, double omega, double delta, double T) const
  {
    if (!loaded_)
      return {};

    // Normalize input
    const double x_raw[N_FEATURES] = { vx, vy, omega, delta, T };
    double x_norm[N_FEATURES];
    for (int j = 0; j < N_FEATURES; ++j)
      x_norm[j] = (x_raw[j] - x_mean_[j]) / x_std_[j];

    // Layer 1: h1 = tanh(W1 @ x_norm + b1)
    std::vector<double> h1(h1_);
    for (int i = 0; i < h1_; ++i)
    {
      double s = b1_[i];
      for (int j = 0; j < N_FEATURES; ++j)
        s += W1_[i * N_FEATURES + j] * x_norm[j];
      h1[i] = std::tanh(s);
    }

    // Layer 2: h2 = tanh(W2 @ h1 + b2)
    std::vector<double> h2(h2_);
    for (int i = 0; i < h2_; ++i)
    {
      double s = b2_[i];
      for (int j = 0; j < h1_; ++j)
        s += W2_[i * h1_ + j] * h1[j];
      h2[i] = std::tanh(s);
    }

    // Output: out_norm = Wout @ h2 + bout
    double out_norm[N_OUTPUTS];
    for (int i = 0; i < N_OUTPUTS; ++i)
    {
      double s = bout_[i];
      for (int j = 0; j < h2_; ++j)
        s += Wout_[i * h2_ + j] * h2[j];
      out_norm[i] = s;
    }

    // Unnormalize: result = out_norm * y_std + y_mean
    return {
      out_norm[0] * y_std_[0] + y_mean_[0],
      out_norm[1] * y_std_[1] + y_mean_[1],
      out_norm[2] * y_std_[2] + y_mean_[2],
    };
  }

private:
  bool loaded_ = false;
  int  h1_ = 0, h2_ = 0;

  std::vector<double> x_mean_, x_std_;
  std::vector<double> y_mean_, y_std_;
  std::vector<double> W1_, b1_;
  std::vector<double> W2_, b2_;
  std::vector<double> Wout_, bout_;
};

}  // namespace crs_controls::pacejka_mpcc
