#ifndef SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_PACEJKA_ESTIMATOR_PACEJKA_ESTIMATOR
#define SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_PACEJKA_ESTIMATOR_PACEJKA_ESTIMATOR

#include "ros_estimators/state_estimator_ros.h"

#include <ros/ros.h>
#include <crs_msgs/car_state_cart.h>
#include <crs_msgs/car_input.h>

#include "ros_estimators/data_converter.h"
#include <estimators/base_estimator.h>
#include <estimators/model_based_estimator.h>
#include <ros_crs_utils/state_message_conversion.h>

namespace ros_estimators
{
/** Dummy struct for empty model */
struct empty_model
{
};
template <typename StateType, typename InputType, typename ModelType = empty_model>
class RosCarEstimator : public RosStateEstimator
{
public:
  // Construction.
  RosCarEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                  const std::vector<std::string> measurement_keys,
                  std::shared_ptr<crs_estimators::BaseEstimator<StateType>> base_estimator)
    : base_estimator(base_estimator), nh_(nh), nh_private_(nh_private)
  {
    // Load config
    nh_private.getParam("max_callback_rate", max_measurement_rate_);
    if (!nh_private.getParam("world_frame", vicon_converter.world_frame))
      ROS_WARN_STREAM("Did not load parameter for world frame. Defaulting to: " << vicon_converter.world_frame);
    if (!nh_private.getParam("track_frame", vicon_converter.track_frame))
      ROS_WARN_STREAM("Did not load parameter for track frame. Defaulting to: " << vicon_converter.track_frame);
    if (!nh_private.getParam("update_track_transform", vicon_converter.update_track_transform))
      ROS_WARN_STREAM("Did not load parameter for update_track_transform. Defaulting to: "
                      << vicon_converter.update_track_transform);

    // Setup ros publishers
    state_estimate_pub_ = nh_private_.advertise<crs_msgs::car_state_cart>("best_state", 10);

    // Register callbacks for keys
    for (const auto& key : measurement_keys)
    {
      if (key == "vicon")
        measurement_subs_.push_back(nh_.subscribe(key, 1, &RosCarEstimator::viconMeasurementCallback, this));
      else if (key == "imu")
        measurement_subs_.push_back(nh_.subscribe(key, 1, &RosCarEstimator::imuMeasurementCallback, this));
      else
      {
        ROS_WARN_STREAM("Masurement key "
                        << key << " has no known ROS conversion registered in estimator. Key will be dropped!");
      }
    }

    assert(!measurement_subs_.empty() && "No Measurement topic provided for Estimator. Aborting!");
  };

  RosCarEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                  const std::vector<std::string> measurement_keys,
                  std::shared_ptr<crs_estimators::ModelBasedEstimator<StateType, InputType>> model_based_estimator)
    : RosCarEstimator(nh, nh_private, measurement_keys,
                      std::static_pointer_cast<crs_estimators::BaseEstimator<StateType>>(model_based_estimator))
  {
    // Regiser control input subscription if estimator is model based
    control_input_sub_ = nh_.subscribe("control_input", 1, &RosCarEstimator::controlInputCallback, this);
  };

  void controlInputCallback(const crs_msgs::car_input::ConstPtr input_msg)
  {
    last_input_ = *input_msg;
    std::dynamic_pointer_cast<crs_estimators::ModelBasedEstimator<StateType, InputType>>(base_estimator)
        ->controlInputCallback(message_conversion::convertToCrsInput<crs_msgs::car_input, InputType>(*input_msg),
                               input_msg->header.stamp.toSec());
  };

  void viconMeasurementCallback(const geometry_msgs::TransformStamped::ConstPtr msg)
  {
    // Rate Limit calls. TODO, one rate limit for each msg type?
    if (msg->header.stamp.toSec() - last_measurement_ts_ < 1 / max_measurement_rate_)
      return;
    last_measurement_ts_ = msg->header.stamp.toSec();

    base_estimator->measurementCallback(vicon_converter.parseData2D(msg));
  };

  void imuMeasurementCallback(const sensor_msgs::Imu::ConstPtr msg)
  {
    // Rate Limit calls. TODO, one rate limit for each msg type?
    if (msg->header.stamp.toSec() - last_measurement_ts_ < 1 / max_measurement_rate_)
      return;
    last_measurement_ts_ = msg->header.stamp.toSec();

    base_estimator->measurementCallback(parseImuData2D(msg));
  };

  void publishState() override;

  // Ugly. TODO, use getter and setter or similar
  std::shared_ptr<ModelType> model;

private:
  double max_measurement_rate_ = 50;  // Hz
  double last_measurement_ts_ = 0;
  // Node handles.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // The following needs to be initialized by child class
  // Publisher
  ros::Publisher state_estimate_pub_;
  // Subscriptions
  ros::Subscriber control_input_sub_;
  std::vector<ros::Subscriber> measurement_subs_;

  ViconConverter vicon_converter;
  std::shared_ptr<crs_estimators::BaseEstimator<StateType>> base_estimator;

  crs_msgs::car_input last_input_;
};
}  // namespace ros_estimators

#endif /* SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_PACEJKA_ESTIMATOR_PACEJKA_ESTIMATOR */
