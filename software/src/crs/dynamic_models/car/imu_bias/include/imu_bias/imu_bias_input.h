#ifndef IMU_BIAS_INPUT_H
#define IMU_BIAS_INPUT_H

#include <iostream>

namespace crs_models
{
namespace imu_bias
{

struct imu_bias_input
{
  static constexpr int NU = 0;

  imu_bias_input(){};  // default constructor
};

inline imu_bias_input operator+(const imu_bias_input& a, const imu_bias_input& b)
{
  imu_bias_input added_struct;
  return added_struct;
}

inline imu_bias_input operator*(double b, const imu_bias_input& a)
{
  imu_bias_input product_struct;
  return product_struct;
}

inline void operator+=(imu_bias_input& a, const imu_bias_input& b)
{
}

inline bool operator==(const imu_bias_input& a, const imu_bias_input& b)
{
  return true;
}

inline std::ostream& operator<<(std::ostream& os,
                                const imu_bias_input& input)  // how to print struct (input) to output stream (os)
{
  os << "imu_bias_input:" << std::endl;                     // Type of struct (input)
  os << " This model does not have an input" << std::endl;  // Type of struct (input)
  return os;
}

}  // namespace imu_bias
}  // namespace crs_models
#endif
