#ifndef IMU_BIAS_IMU_BIAS_CONTINUOUS_H
#define IMU_BIAS_IMU_BIAS_CONTINUOUS_H

// #include <pacejka_model/pacejka_car_input.h>
#include "imu_bias_state.h"
#include "imu_bias_input.h"
#include <algorithm>
#include <dynamic_models/continuous_dynamic_model.h>
#include <iostream>
#include <iostream>

namespace crs_models
{
namespace imu_bias
{

class ContinuousImuBias : public ContinuousDynamicModel<imu_bias_state, imu_bias_input>
{
public:
  ContinuousImuBias();  // Constructor

  /**
   * @brief applies bias model for a given control input
   *
   */
  imu_bias_state applyModel(const imu_bias_state state, const imu_bias_input control_input);

  /**
   * @brief Get the Numerical Jacobian for a given state and control input (evaluate symbolic jacobian)
   *
   * @param state current state
   * @param control_input control input
   * @param A the state jacobian df/dx
   * @param B the input jacobian df/du
   */
  void getNumericalJacobian(const imu_bias_state& state, const imu_bias_input& control_input, StateMatrix& A,
                            InputMatrix& B);

  /**
   * @brief Returns the casadi function x_dot = f(x,u)
   *
   * @param state the state x
   * @param control_input  the input u
   * @return
   */
  std::vector<casadi::MX> getContinuousDynamics(const std::vector<casadi::MX> state,
                                                const std::vector<casadi::MX> control_input);
};

}  // namespace imu_bias

}  // namespace crs_models
#endif
