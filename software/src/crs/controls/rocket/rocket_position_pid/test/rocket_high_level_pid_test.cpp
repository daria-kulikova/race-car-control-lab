#include <gtest/gtest.h>
#include <Eigen/Core>
#include <iostream>
#include <random>

#include <rocket_position_pid/rocket_controller_specializations.h>

#define EQUALITY_TOLERANCE 1e-10

TEST(rocketControllerTests, testControllerInterface)
{
  crs_models::rocket_6_dof_model::rocket_6_dof_params dummy_params =
      crs_models::rocket_6_dof_model::rocket_6_dof_params();

  crs_controls::rocket_controller_config<crs_controls::RocketHighLevelPidController,
                                         crs_controls::RocketAttitudeController, crs_controls::Rocket6DofAllocation>
      dummy_config = { crs_controls::rocket_high_level_pid_controller_config(),
                       crs_controls::rocket_attitude_controller_config() };

  crs_controls::Rocket6DofPidController controller = crs_controls::Rocket6DofPidController(
      dummy_config, std::make_shared<crs_models::rocket_6_dof_model::rocket_6_dof_params>(dummy_params), nullptr);

  auto return_configs = controller.getConfig();
  EXPECT_TRUE(return_configs.high_level_controller_config.position_p_gain.isApprox(
      dummy_config.high_level_controller_config.position_p_gain, EQUALITY_TOLERANCE));
  EXPECT_TRUE(return_configs.high_level_controller_config.velocity_p_gain.isApprox(
      dummy_config.high_level_controller_config.velocity_p_gain, EQUALITY_TOLERANCE));
  EXPECT_TRUE(return_configs.high_level_controller_config.velocity_i_gain.isApprox(
      dummy_config.high_level_controller_config.velocity_i_gain, EQUALITY_TOLERANCE));
  EXPECT_TRUE(return_configs.high_level_controller_config.velocity_d_gain.isApprox(
      dummy_config.high_level_controller_config.velocity_d_gain, EQUALITY_TOLERANCE));
  EXPECT_NEAR(return_configs.high_level_controller_config.max_angle,
              dummy_config.high_level_controller_config.max_angle, EQUALITY_TOLERANCE);
  EXPECT_NEAR(return_configs.high_level_controller_config.loop_rate,
              dummy_config.high_level_controller_config.loop_rate, EQUALITY_TOLERANCE);
  EXPECT_TRUE(return_configs.low_level_controller_config.attitude_p_gain.isApprox(
      dummy_config.low_level_controller_config.attitude_p_gain, EQUALITY_TOLERANCE));
  EXPECT_TRUE(return_configs.low_level_controller_config.rate_p_gain.isApprox(
      dummy_config.low_level_controller_config.rate_p_gain, EQUALITY_TOLERANCE));
  EXPECT_TRUE(return_configs.low_level_controller_config.rate_i_gain.isApprox(
      dummy_config.low_level_controller_config.rate_i_gain, EQUALITY_TOLERANCE));
  EXPECT_TRUE(return_configs.low_level_controller_config.rate_d_gain.isApprox(
      dummy_config.low_level_controller_config.rate_d_gain, EQUALITY_TOLERANCE));
  EXPECT_NEAR(return_configs.low_level_controller_config.loop_rate, dummy_config.low_level_controller_config.loop_rate,
              EQUALITY_TOLERANCE);
}

int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}
