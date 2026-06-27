#include "ros_crs_utils/parameter_io.h"

#include "pacejka_mpcc/mpcc_pacejka_config.h"

namespace parameter_io
{
template <>
crs_controls::pacejka_mpcc::mpcc_pacejka_config
getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>(const ros::NodeHandle& nh)
{
  crs_controls::pacejka_mpcc::mpcc_pacejka_config params;

  if (!nh.getParam("Q1", params.Q1))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>: did not load Q1");
  if (!nh.getParam("Q2", params.Q2))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>: did not load Q2");
  if (!nh.getParam("R1", params.R1))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>: did not load R1");
  if (!nh.getParam("R2", params.R2))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>: did not load R2");
  if (!nh.getParam("R3", params.R3))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>: did not load R3");
  if (!nh.getParam("q", params.q))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>: did not load q");
  if (!nh.getParam("lag_compensation_time", params.lag_compensation_time))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>: did not load lag_compensation_time");
  if (!nh.getParam("solver_type", params.solver_type))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>: did not load solver_type");

  int max_sqp_iter = 1;
  if (!nh.getParam("max_sqp_iterations", max_sqp_iter))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>: did not load max_sqp_iterations");
  params.max_sqp_iterations = static_cast<unsigned int>(max_sqp_iter);

  nh.getParam("warmstart_iterations", params.warmstart_iterations);  // Optional Param. No warning raised if not set.
  nh.getParam("log_path", params.log_path);                        // Optional. Empty = logging disabled.
  nh.getParam("gp_model_path", params.gp_model_path);                        // Optional. Empty = disabled.
  nh.getParam("inducing_gp_model_path", params.inducing_gp_model_path);    // Optional. Empty = disabled.
  nh.getParam("inducing_gp_uncertainty_scaling", params.inducing_gp_uncertainty_scaling);  // Optional. Default = true.
  nh.getParam("mlp_model_path", params.mlp_model_path);                    // Optional. Empty = disabled.
  nh.getParam("residual_scale", params.residual_scale);            // Optional. Default = 1.0.
  int log_max_points = 0;
  nh.getParam("log_max_points", log_max_points);                  // Optional. 0 = unlimited.
  params.log_max_points = static_cast<unsigned int>(std::max(0, log_max_points));
  nh.getParam("sample_period", params.sample_period);             // Must match Ts in solver_acados.yaml.

  return params;
}
}  // namespace parameter_io
