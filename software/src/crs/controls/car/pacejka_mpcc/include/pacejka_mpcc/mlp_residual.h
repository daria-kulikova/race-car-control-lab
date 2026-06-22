#pragma once

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
 * Generic depth: supports any number of hidden layers.
 * Architecture: tanh hidden layers → linear output layer.
 * Predicts velocity residuals [eps_vx, eps_vy, eps_omega] in m/s.
 * Divide by sample_period to get acceleration corrections for the ODE.
 *
 * Binary format (produced by train_mlp.py):
 *   int32[1]:          n_layers  (= n_hidden + 1)
 *   int32[n_layers+1]: sizes     [n_in, h1, ..., hk, n_out]
 *   float64[n_in]:     x_mean
 *   float64[n_in]:     x_std
 *   float64[n_out]:    y_mean
 *   float64[n_out]:    y_std
 *   for each layer i:
 *     float64[sizes[i+1]*sizes[i]]:  W[i]  row-major  (W @ x + b convention)
 *     float64[sizes[i+1]]:           b[i]
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

    int32_t n_layers;
    f.read(reinterpret_cast<char*>(&n_layers), sizeof(int32_t));

    sizes_.resize(n_layers + 1);
    f.read(reinterpret_cast<char*>(sizes_.data()), (n_layers + 1) * sizeof(int32_t));

    if (sizes_.front() != N_FEATURES || sizes_.back() != N_OUTPUTS)
    {
      std::cout << "[MLPResidual] unexpected dimensions: n_in=" << sizes_.front()
                << " n_out=" << sizes_.back() << std::endl;
      return false;
    }

    auto read_vec = [&](std::vector<double>& v, int n) {
      v.resize(n);
      f.read(reinterpret_cast<char*>(v.data()), n * sizeof(double));
    };

    read_vec(x_mean_, N_FEATURES);
    read_vec(x_std_,  N_FEATURES);
    read_vec(y_mean_, N_OUTPUTS);
    read_vec(y_std_,  N_OUTPUTS);

    W_.resize(n_layers);
    b_.resize(n_layers);
    for (int i = 0; i < n_layers; ++i)
    {
      read_vec(W_[i], sizes_[i + 1] * sizes_[i]);
      read_vec(b_[i], sizes_[i + 1]);
    }

    loaded_ = true;
    std::cout << "[MLPResidual] loaded " << path << "  (arch:";
    for (int s : sizes_) std::cout << " " << s;
    std::cout << ")" << std::endl;
    return true;
  }

  bool isLoaded() const { return loaded_; }

  Residual predict(double vx, double vy, double omega, double delta, double T) const
  {
    if (!loaded_)
      return {};

    // Normalize input
    std::vector<double> act = { vx, vy, omega, delta, T };
    for (int j = 0; j < N_FEATURES; ++j)
      act[j] = (act[j] - x_mean_[j]) / x_std_[j];

    // Forward pass: tanh for all layers except the last (linear output)
    const int n_layers = static_cast<int>(W_.size());
    for (int i = 0; i < n_layers; ++i)
    {
      const int out_size = sizes_[i + 1];
      const int in_size  = sizes_[i];
      std::vector<double> next(out_size);
      for (int r = 0; r < out_size; ++r)
      {
        double s = b_[i][r];
        for (int c = 0; c < in_size; ++c)
          s += W_[i][r * in_size + c] * act[c];
        next[r] = (i < n_layers - 1) ? std::tanh(s) : s;  // last layer: linear
      }
      act = std::move(next);
    }

    // Unnormalize output
    return {
      act[0] * y_std_[0] + y_mean_[0],
      act[1] * y_std_[1] + y_mean_[1],
      act[2] * y_std_[2] + y_mean_[2],
    };
  }

private:
  bool loaded_ = false;

  std::vector<int32_t>              sizes_;    // [n_in, h1, ..., hk, n_out]
  std::vector<double>               x_mean_, x_std_;
  std::vector<double>               y_mean_, y_std_;
  std::vector<std::vector<double>>  W_, b_;   // one entry per layer
};

}  // namespace crs_controls::pacejka_mpcc
