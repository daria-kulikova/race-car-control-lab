#include "imu_bias/imu_bias_continuous.h"
#include <dynamic_models/utils/data_conversion.h>

#include <casadi/casadi.hpp>
namespace crs_models
{
namespace imu_bias
{
ContinuousImuBias::ContinuousImuBias()  // Constructor
{
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("bias_ax"), casadi::MX::sym("bias_ay"),
                                       casadi::MX::sym("bias_dyaw") };
  std::vector<casadi::MX> input_mx = {};

  std::vector<casadi::MX> state_dot_mx = getContinuousDynamics(state_mx, input_mx);
  // append control inputs to state vector (casadi functions only take one input)

  dynamics_f_ = casadi::Function("f_bias_cont", state_mx, state_dot_mx);

  jacobian_fn_ = getSymbolicJacobian();
}

/**
 * @brief evaluated state_dot (= f(x,u)) at current state and control input
 *
 * @param state
 * @param input
 * @return imu_bias_state returns state_dot evaluated at current state and control input
 */
imu_bias_state ContinuousImuBias::applyModel(const imu_bias_state state, const imu_bias_input input)
{
  imu_bias_state state_dot_numerical;
  // Convert casadi symbols (mx) to casadi function to be evaluated at current state & control input
  dynamics_f_(commons::convertToConstVector(state), commons::convertToVector(state_dot_numerical));
  return state_dot_numerical;
};

/**
 * @brief Evaluate the Symbolic Jacobian at a given state and control input. The Jacobians are saved in A, B
 *
 * @param state
 * @param control_input
 * @param A empty matrix to fill as Jacobian df/dx
 * @param B empty matrix to fill as Jacobian df/du
 */
void ContinuousImuBias::getNumericalJacobian(const imu_bias_state& state, const imu_bias_input& control_input,
                                             StateMatrix& A, InputMatrix& B)
{
  auto state_and_input = commons::convertToConstVector(state);

  // Prepare inputs for jacobian function

  // the jacobian function expects 3 + 2 + 3 arguments, since it can deal with implicit representation.
  // Our function is: f(bias_yaw,bias_ax,bias_ay) -> (bias_yaw_d,bias_ax_d,bias_ay_d)
  // jacobian_fn() returns:
  // f(bias_yaw,bias_ax,bias_ay,bias_yaw_d,bias_ax_d,bias_ay_d) -> (dbias_yaw_d/dbias_yaw, dbias_yaw_d/dbias_ax,
  // dbias_yaw_d/dbias_ay ...) Lets just add zeros at the end since we don't have any implicit dependencies

  auto& state_and_input_and_implicit = state_and_input;
  imu_bias_state unused_implicit_inputs;
  auto unused_implicit_inputs_vec = commons::convertToConstVector(unused_implicit_inputs);
  state_and_input_and_implicit.insert(state_and_input.end(), unused_implicit_inputs_vec.begin(),
                                      unused_implicit_inputs_vec.end());  // Adds unused_implicit_inputs at end of
                                                                          // fnc_input

  getNumericalJacobianInternal(state_and_input_and_implicit, A, B);
}

/**
 * @brief Get the analytical state equations f(state, input) = state_dot
 *
 * @param state
 * @param control_input
 * @return std::vector<casadi::MX> returns the analytical state equations f(state, input) = state_dot
 */
std::vector<casadi::MX> ContinuousImuBias::getContinuousDynamics(const std::vector<casadi::MX> state,
                                                                 const std::vector<casadi::MX> control_input)
{
  using namespace casadi;

  std::vector<casadi::MX> state_dot = { 0, 0, 0 };  // Constant IMU bias

  return state_dot;
}

}  // namespace imu_bias
}  // namespace crs_models
