#ifndef ESTIMATORS_BASE_ESTIMATOR_H
#define ESTIMATORS_BASE_ESTIMATOR_H

#include <sensor_models/sensor_measurement.h>
#include <memory>

namespace crs_estimators
{
template <typename StateType>
class BaseEstimator
{
public:
  /**
   * @brief Construct a new Base Estimator object
   *
   */
  BaseEstimator(){};

  virtual StateType getStateEstimate(const double timestamp)
  {
    // Default implementation, ignore timestamp
    return getStateEstimate();
  }

  /**
   * Returns the currently best state estimate
   */
  virtual StateType getStateEstimate() = 0;

  /**
   * @brief Get the last valid timestamp (i.e. timestamp for which the state estimate is valid)
   *
   * @return double
   */
  virtual double getLastValidTs() const = 0;

  /**
   * @brief Function that gets called when new measurements from sensor models arrive
   *
   */
  virtual void measurementCallback(const crs_sensor_models::measurement measurement) = 0;

  /* @brief Resets the state estimate to the given state
   *
   * @param state The state to which the state estimate should be reset
   */
  virtual void resetStateEstimate(const StateType state) = 0;

  /**
   * @brief Returns diagnostic data of the estimator. This is a
   * map of string to vector of floats. The string is the name of the
   * diagnostic data and the vector of floats is the data itself.
   *
   * @return std::map<std::string, std::vector<float>>
   */
  virtual std::vector<std::pair<std::string, std::vector<float>>> getDiagnosticData()
  {
    auto data = std::move(dignostic_data_);  // Copy the data to return
    dignostic_data_ = {};                    // Clear the data
    return data;
  }

protected:
  std::vector<std::pair<std::string, std::vector<float>>> dignostic_data_;
};

}  // namespace crs_estimators
#endif /* ESTIMATORS_BASE_ESTIMATOR_H */
