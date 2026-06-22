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
  nh.getParam("data_log_path", params.data_log_path);               // Optional. Empty = logging disabled.
  nh.getParam("gp_model_path", params.gp_model_path);              // Optional. Empty = GP disabled.
  nh.getParam("mlp_model_path", params.mlp_model_path);            // Optional. Empty = MLP disabled.
  nh.getParam("tracking_log_path", params.tracking_log_path);     // Optional. Empty = tracking log disabled.
  nh.getParam("sample_period", params.sample_period);             // Must match Ts in solver_acados.yaml.

  return params;
}
}  // namespace parameter_io
