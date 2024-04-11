#ifndef DYNAMIC_MODELS_CONTINUOUS_DYNAMIC_MODEL_H
#define DYNAMIC_MODELS_CONTINUOUS_DYNAMIC_MODEL_H
#include "utils/data_conversion.h"
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <iostream>

namespace crs_models
{
template <typename StateType, typename InputType>
class ContinuousDynamicModel
{
public:
  typedef Eigen::Matrix<double, StateType::NX, StateType::NX> StateMatrix;
  typedef Eigen::Matrix<double, StateType::NX, InputType::NU> InputMatrix;

  /** Definition of often used constants */
  static constexpr double GRAVITY = 9.81;

  /** State and input dimensions of the Model*/
  static const int NX = StateType::NX;
  static const int NU = InputType::NU;

  /**
   * @brief Evaluates the model dynamic function at the given state and input
   *
   * @param state
   * @param control_input
   * @return StateType
   */
  virtual StateType applyModel(const StateType state, const InputType control_input) = 0;

  /**
   * @brief Get the Numerical Jacobian for a given state and control input
   *
   * @param state current state
   * @param control_input control input
   * @param A the state jacobian df/dx
   * @param B the input jacobian df/du
   */
  virtual void getNumericalJacobian(const StateType& state, const InputType& control_input,
                                    Eigen::Matrix<double, StateType::NX, StateType::NX>& A,
                                    Eigen::Matrix<double, StateType::NX, InputType::NU>& B) = 0;

  /**
   * @brief Get the Symbolic Jacobian as casadi function.
   *
   * Note, this functions requires #States + #Inputs + #States inputs and gives #States x (#Inputs + #States) outputs.
   *
   * Example for a state of dimension 6 (x,y,yaw, vx, vy, dyaw) and input of dimension 2 (u0, u1):
   *        jacobian(x,y,yaw,vx,vy,dyaw,u0,u1, x_d,y_d,yaw_d,vx_d,vy_d,dyaw_d) -> (dx_d/dx, dx_d/dy, dx_d/dyaw ...)
   *        Where x_d, y_d, yaw_d, ... is the output of f(x,y,yaw,vx,vy,dyaw,u0,u1) and can be ignored if the function
   *        is defined explicitely
   *
   * @return casadi::Function
   */
  casadi::Function getSymbolicJacobian()
  {
    return dynamics_f_.jac();
  }

  /**
   * @brief Returns the casadi function x_dot = f(x,u)
   *
   * @param state the state x
   * @param control_input  the input u
   * @return vector with dyanmics defined as casadi::MX vector
   */
  virtual std::vector<casadi::MX> getContinuousDynamics(const std::vector<casadi::MX> state,
                                                        const std::vector<casadi::MX> control_input) = 0;

  void setQ(const Eigen::Matrix<double, StateType::NX, StateType::NX>& Q)
  {
    Q_ = Q;
  }

  const Eigen::Matrix<double, StateType::NX, StateType::NX> getQ()
  {
    return Q_;
  }

protected:
  Eigen::Matrix<double, StateType::NX, StateType::NX> Q_;
  casadi::Function dynamics_f_;  // x_dot = f(x,u)
  casadi::Function jacobian_fn_;

  /**
   * @brief Returns nummerical Jacobian
   *
   * @param jacobian_inputs
   * @param A the state jacobian df/dx
   * @param B the input jacobian df/du
   */

  void getNumericalJacobianInternal(std::vector<const double*> jacobian_inputs,
                                    Eigen::Matrix<double, StateType::NX, StateType::NX>& A,
                                    Eigen::Matrix<double, StateType::NX, InputType::NU>& B)
  {
    Eigen::Matrix<double, StateType::NX, StateType::NX + InputType::NU> full_jacobian =
        Eigen::Matrix<double, StateType::NX, StateType::NX + InputType::NU>::Zero();  // Matrix that the results
                                                                                      // will be written into

    // Call jacobian
    jacobian_fn_(jacobian_inputs,
                 commons::convertToVector<StateType::NX, StateType::NX + InputType::NU>(full_jacobian));

    A = full_jacobian.block(0, 0, StateType::NX, StateType::NX);              // df/dx
    B = full_jacobian.block(0, StateType::NX, StateType::NX, InputType::NU);  // df/du
  }
};
}  // namespace crs_models
#endif
