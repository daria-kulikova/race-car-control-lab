#pragma once

#include <array>

#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_pacejka_model.h"

#include "pacejka_model/pacejka_params.h"
#include "control_commons/mpc_solver.h"

#include "pacejka_mpcc/solvers/pacejka_mpcc_structures.h"

namespace crs_controls::pacejka_mpcc::solvers::acados_solver
{
struct AcadosPacejkaMpccSolverDims
{
  static constexpr int NX = PACEJKA_MODEL_NX;
  static constexpr int NU = PACEJKA_MODEL_NU;
  static constexpr int N = PACEJKA_MODEL_N;
  static constexpr int NP = PACEJKA_MODEL_NP;
};

class AcadosPacejkaMpccSolver
  : public crs_controls::control_commons::MpcSolver<
        AcadosPacejkaMpccSolver, AcadosPacejkaMpccSolverDims, crs_models::pacejka_model::pacejka_params,
        crs_controls::pacejka_mpcc::solvers::TrackingCosts, crs_controls::pacejka_mpcc::solvers::TrajectoryTrackPoint>
{
public:
  // TODO(@naefjo): This is kinda ugly. Can we get these dimensions from the template list?
  static constexpr int NX = AcadosPacejkaMpccSolverDims::NX;
  static constexpr int NU = AcadosPacejkaMpccSolverDims::NU;
  static constexpr int N = AcadosPacejkaMpccSolverDims::N;
  static constexpr int NP = AcadosPacejkaMpccSolverDims::NP;
  using vars = crs_controls::pacejka_mpcc::solvers::vars;
  using inputs = crs_controls::pacejka_mpcc::solvers::inputs;

  AcadosPacejkaMpccSolver();

  /**
   * @brief Sets the Initial State Constraint. The provided array must have the same length as the state dimension
   *
   * @param constraint
   */
  void setInitialState(AcadosPacejkaMpccSolver::StateArray& initial_state);

  /**
   * @brief Sets an initial guess for the state at stage "stage" of the solver.
   * The provided array must have the same length as the state dimension
   *
   * @param constraint
   */
  void setStateInitialGuess(int stage, AcadosPacejkaMpccSolver::StateArray& initial_guess);

  /**
   * @brief Sets an initial guess for the input at stage "stage" of the solver.
   * The provided array must have the same length as the input dimension
   *
   * @param constraint
   */
  void setInputInitialGuess(int stage, AcadosPacejkaMpccSolver::InputArray& initial_guess);

  /**
   * @brief Updates the internal params for stage "stage"
   *
   * @param stage which stage to update the parameter [0,HorizonLength)
   * @param model_dynamics  the model dynamics
   * @param costs  the cost parameters
   * @param tracking_point tracking point
   */
  void updateParams(int stage, AcadosPacejkaMpccSolver::MpcStageParameters& params);

  /**
   * @brief Set the GP residual [d_vx, d_vy, d_omega] to apply as a constant
   * disturbance over the entire horizon. Call once per control step before solve().
   */
  void setGPResidual(double d_vx, double d_vy, double d_omega)
  {
    gp_d_vx_.fill(d_vx);
    gp_d_vy_.fill(d_vy);
    gp_d_omega_.fill(d_omega);
  }

  /**
   * @brief Set the GP residual [d_vx, d_vy, d_omega] for a single stage of the
   * horizon, overriding the broadcast value set by the constant-residual overload
   * above for that stage. Used when mpcc_pacejka_config::residual_per_stage is
   * enabled to evaluate the residual model at each stage's own predicted state.
   *
   * @param stage which stage to set the residual for [0, N]
   */
  void setGPResidual(int stage, double d_vx, double d_vy, double d_omega)
  {
    gp_d_vx_[stage]    = d_vx;
    gp_d_vy_[stage]    = d_vy;
    gp_d_omega_[stage] = d_omega;
  }
  /**
   * @brief Solves the optimization problems and stores the solution in x and u.
   *
   * @param x State array or point with size N*StateDimenstion
   * @param u Input array or point with size N*Inputdimension
   * @return const int, return code. If no error occurred, return code is zero
   */
  MpcReturnType solve();

  double getSamplePeriod();

private:
  // Sized N+1: updateParams() is called for stages [0, N] inclusive (see
  // control_commons::MpcSolver::solve_problem, which updates the terminal
  // stage N separately after the [0, N) loop).
  std::array<double, N + 1> gp_d_vx_{};
  std::array<double, N + 1> gp_d_vy_{};
  std::array<double, N + 1> gp_d_omega_{};

  std::unique_ptr<pacejka_model_solver_capsule> acados_ocp_capsule_;
  std::unique_ptr<ocp_nlp_config> nlp_config_;
  std::unique_ptr<ocp_nlp_dims> nlp_dims_;
  std::unique_ptr<ocp_nlp_in> nlp_in_;
  std::unique_ptr<ocp_nlp_out> nlp_out_;
  std::unique_ptr<ocp_nlp_solver> nlp_solver_;

  /**
   * @brief Get the Last Solution and stores it in x and u
   *
   * @param x states, size state dimension x horizon length
   * @param u input, size state dimension x horizon length
   */
  AcadosPacejkaMpccSolver::MpcSolution getSolution();
};

}  // namespace crs_controls::pacejka_mpcc::solvers::acados_solver
