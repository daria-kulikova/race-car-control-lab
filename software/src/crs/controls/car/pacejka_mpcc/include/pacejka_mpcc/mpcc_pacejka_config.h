#pragma once

#include <string>

namespace crs_controls::pacejka_mpcc
{
struct mpcc_pacejka_config
{
  /**
   * @brief contouring cost
   *
   */
  double Q1;
  /**
   * @brief lag cost
   *
   */
  double Q2;
  /**
   * @brief dtorque cost
   *
   */
  double R1;
  /**
   * @brief dsteer cost
   *
   */
  double R2;
  /**
   * @brief  darclength cost
   *
   */
  double R3;
  /**
   * @brief arclength cost
   *
   */
  double q;
  /**
   * @brief State will be propagated by this amount before calculating control input
   *
   */
  double lag_compensation_time;

  /**
   * @brief Solver integration timestep [s]. Must match Ts in solver_acados.yaml.
   *        Used to convert GP velocity residuals (m/s) to acceleration corrections (m/s²).
   */
  double sample_period = 0.02;
  /**
   * @brief Number of iterations to warmstart solver
   *
   */
  double warmstart_iterations = 1000;
  /**
   * @brief Type of solver that should be used. e.g. ACADOS
   *
   */
  std::string solver_type;

  /**
   * @brief How many iterations of SQP should be run
   */
  unsigned int max_sqp_iterations = 1;

  /**
   * @brief Path for combined data+tracking CSV. Empty string disables logging.
   *        CSV columns: t,vx,vy,omega,delta,T,eps_vx,eps_vy,eps_omega,eC,eL,pos_x,pos_y,ref_x,ref_y,lap,theta
   */
  std::string log_path = "";

  /**
   * @brief Path to the GP residual model binary (gp_model.bin from train_gp.py).
   *        Empty string disables GP correction.
   */
  std::string gp_model_path = "";

  /**
   * @brief Path to the MLP residual model binary (mlp_model.bin from train_mlp.py).
   *        Empty string disables MLP correction. Takes priority over GP if both are set.
   */
  std::string mlp_model_path = "";

  /**
   * @brief Scaling factor in [0, 1] applied to all residual corrections (GP and MLP).
   *        Applied to both lag compensation and the zero-order solver correction.
   *        Use to tune aggressiveness; set to 0 to disable correction without removing model path.
   */
  double residual_scale = 1.0;

  /**
   * @brief Maximum number of rows to write to the combined log.
   *        0 = unlimited.
   */
  unsigned int log_max_points = 0;
};
}  // namespace crs_controls::pacejka_mpcc
