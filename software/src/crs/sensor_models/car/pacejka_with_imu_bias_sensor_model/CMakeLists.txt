cmake_minimum_required(VERSION 3.0.2)
project(pacejka_with_imu_bias_sensor_model)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
find_package(casadi)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

#############
# Libraries #
#############
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

cs_add_library(${PROJECT_NAME}
        src/vicon_with_imu_bias_sensor_model.cpp
        src/imu_with_bias_sensor_model.cpp
        )

target_link_libraries(${PROJECT_NAME})

##########
# Tests #
##########

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(imu_bias_sensor_model_test test/imu_bias_sensor_model_test.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/)
  target_link_libraries(imu_bias_sensor_model_test ${PROJECT_NAME} ${catkin_LIBRARIES} casadi stdc++fs)

  catkin_add_gtest(vicon_with_imu_bias_sensor_model_test test/vicon_with_imu_bias_sensor_model_test.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/)
  target_link_libraries(vicon_with_imu_bias_sensor_model_test ${PROJECT_NAME} ${catkin_LIBRARIES} casadi stdc++fs)
endif()

##########
# Export #
##########

cs_install()
cs_export()
