cmake_minimum_required(VERSION 3.0.2)
project(ros_estimators)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
find_package(casadi)

# TODO: CHECK THIS BEFORE MERGING @SABODMER!!!!!!
find_package(acados)
catkin_simple()
catkin_package()

#############
# Libraries #
#############


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
# TODO: CHECK THIS BEFORE MERGING @SABODMER!!!!!!

  # Link against acados for MPC generation
  $ENV{ACADOS_SOURCE_DIR}/include
  $ENV{ACADOS_SOURCE_DIR}/include/acados
  $ENV{ACADOS_SOURCE_DIR}/include/blasfeo/include
  $ENV{ACADOS_SOURCE_DIR}/include/hpipm/include
)


find_library(ACADOS_LIB
  NAMES acados
        hpipm
        blasfeo
  PATHS $ENV{ACADOS_SOURCE_DIR}/lib
)

# file(GLOB_RECURSE CPP_SRC_FILES RELATIVE ${CMAKE_SOURCE_DIR}
# # /code/src/crs/estimators/mhe_solvers/acados/acados_pacejka_mhe_solver/src/c_generated_code/**.o
# # /code/src/crs/estimators/mhe_solvers/acados/acados_pacejka_mhe_solver/src/c_generated_code/**.c
# # /code/src/crs/estimators/mhe_solvers/acados/acados_pacejka_mhe_solver/src/c_generated_code/**.h
# # /code/src/crs/estimators/mhe_solvers/acados/acados_pacejka_mhe_solver/src/c_generated_code/**.cpp
# )

message("Generated code: ${CPP_SRC_FILES}")

message("Generated code: ${CPP_SRC_FILES}")
if (${mh_estimator_FOUND})
  add_definitions(-Dmh_estimator_FOUND)
else()
  message("Moving Horizon Estimator not found. Building without Moving Horizon Estimator support")
endif()


if (${kalman_estimator_FOUND})
  add_definitions(-Dkalman_estimator_FOUND)
else()
  message("Kalman Estimator not found. Building without Kalman Estimator support")
endif()

# ================== Pacejka Model ==================
if (${pacejka_model_FOUND})
  add_definitions(-Dpacejka_model_FOUND)
  list(APPEND CPP_SRC_FILES src/car_estimator/pacejka_estimator.cpp)
  list(APPEND CPP_SRC_FILES src/component_registry/pacejka/resolve_sensor_models.cpp)
  list(APPEND CPP_SRC_FILES src/component_registry/pacejka/resolve_visualizers.cpp)
else()
  message("Pacejka Model not found. Building without Pacejka model support")
endif()

if (${pacejka_model_FOUND} AND ${kalman_estimator_FOUND})
  add_definitions(-Dpacejka_model_FOUND -Dkalman_estimator_FOUND)
  list(APPEND CPP_SRC_FILES src/component_registry/pacejka/resolve_ekf.cpp)
  list(APPEND CPP_SRC_FILES src/component_registry/pacejka/resolve_sqrt_ekf.cpp)
else()
  message("Pacejka model or Kalman Filter not found. Building without Pacejka Kalman support")
endif()

if (${pacejka_model_FOUND} AND ${mh_estimator_FOUND})
  add_definitions(-Dpacejka_model_FOUND -Dmh_estimator_FOUND)
  list(APPEND CPP_SRC_FILES src/component_registry/pacejka/resolve_mhe.cpp)
else()
  message("Pacejka model or MHE not found. Building without Pacejka Kalman support")
endif()

if (${pacejka_model_FOUND} AND ${lowpass_estimator_FOUND})
  add_definitions(-Dlowpass_estimator_FOUND)
  list(APPEND CPP_SRC_FILES src/component_registry/pacejka/resolve_lowpass.cpp)
else()
  message("Lowpass Estimator or Lowpass Filter not found not found. Building without Pacejka Lowpass Estimator")
endif()

# ================== Kinematic Model ==================
if (${kinematic_model_FOUND})
  add_definitions(-Dkinematic_model_FOUND)
  list(APPEND CPP_SRC_FILES src/car_estimator/kinematic_estimator.cpp)
  list(APPEND CPP_SRC_FILES src/component_registry/kinematic/resolve_sensor_models.cpp)
  list(APPEND CPP_SRC_FILES src/component_registry/kinematic/resolve_visualizers.cpp)
else()
  message("Kinematic Model not found. Building without Kinematic model support")
endif()

if (${kinematic_model_FOUND} AND ${kalman_estimator_FOUND})
  add_definitions(-Dpacejka_model_FOUND -Dkalman_estimator_FOUND)
  list(APPEND CPP_SRC_FILES src/component_registry/kinematic/resolve_ekf.cpp)
  list(APPEND CPP_SRC_FILES src/component_registry/kinematic/resolve_sqrt_ekf.cpp)
else()
  message("Kinematic model or Kalman Filter not found. Building without Kinematic Kalman support")
endif()

# ================== Stacked Model ==================
if (${stacked_model_FOUND} AND ${imu_bias_FOUND} AND ${pacejka_model_FOUND})
  add_definitions(-Dstacked_model_FOUND -Dimu_bias_FOUND)
  list(APPEND CPP_SRC_FILES src/car_estimator/pacejka_with_imu_bias_estimator.cpp)
else()
  message("Stacked model and IMU model not found. Building without stacked model and IMU model support")
endif()


cs_add_library(${PROJECT_NAME}
        ${CPP_SRC_FILES}
        src/data_converter.cpp
        src/component_registry/component_registry.cpp
        )


target_link_libraries(${PROJECT_NAME} casadi ${OPTIONAL_LIBS})

###############
# Executables #
###############
cs_add_executable(ros_estimation_node
  app/ros_estimation_node.cpp
        )
target_link_libraries(ros_estimation_node ${PROJECT_NAME} acados casadi stdc++fs 	${ACADOS_LIB})

##########
# Tests #
##########

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  if (${kalman_estimator_FOUND})
    add_rostest_gtest(test_node test/pacejka_kalman_test.launch test/pacejka_kalman_test.cpp)
    target_link_libraries(test_node ${catkin_LIBRARIES}  ${PROJECT_NAME} casadi stdc++fs)
  endif()

endif()


##########
# Export #
##########

cs_install()
cs_export(LIBRARIES ${PROJECT_NAME} ${ACADOS_LIB} casadi)
