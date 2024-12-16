cmake_minimum_required(VERSION 3.0.2)
project(ros_controllers)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
find_package(casadi)

catkin_simple()

## Optional dependency handling

if(${pacejka_model_FOUND})
   add_definitions(-Dpacejka_model_FOUND)
else()
  message("Pacejka Model not found. Building without Pacejka model support")
endif()

if(${kinematic_model_FOUND})
   add_definitions(-Dkinematic_model_FOUND)
else()
  message("Kinematic Model not found. Building without Kinematic model support")
endif()

if(${pid_controller_FOUND})
add_definitions(-Dpid_controller_FOUND)
else()
message("PID Controller not found. Building without PID Controller support")
endif()

if(${mpc_controller_FOUND})
add_definitions(-Dmpc_controller_FOUND)
else()
message("MPC Controller not found. Building without MPC Controller support")
endif()

if(${ff_fb_controller_FOUND})
add_definitions(-Dff_fb_controller_FOUND)
else()
message("FF FB Controller not found. Building without FF FB Controller support")
endif()

if(${rocket_6_dof_model_FOUND})
   add_definitions(-Drocket_6_dof_model_FOUND)
else()
  message("Rocket 6 Dof Model not found. Building without Rocket model support")
endif()

if(${rocket_position_pid_FOUND})
add_definitions(-Drocket_position_pid_FOUND)
else()
message("Rocket PID Controller not found. Building without Rocket Controller support")
endif()

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/dynamic_config.cpp
        src/component_registry.cpp
        src/trajectory/dynamic_trajectory_updater.cpp
        src/ros_joystick_controller.cpp
)

###########
# Tests #
##########


###############
# Executables #
###############
cs_add_executable(controller_node
  app/controller_node.cpp
)
target_link_libraries(controller_node ${PROJECT_NAME} casadi)

# make sure configure headers are built before any node using them
add_dependencies(controller_node ${PROJECT_NAME}_gencfg)

##########
# Export #
##########

cs_install()
cs_export()
