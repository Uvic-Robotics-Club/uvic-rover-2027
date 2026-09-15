cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(rover_arm)

# Enable strict warnings for C++
if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-W -Wall -Wextra)
endif()

#=====================================
# Dependencies
#=====================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

#=====================================
# Include directories
#=====================================

include_directories(include)

#=====================================
# HAL library
#=====================================

# Build the HAL backends as a shared library so the control node can link to it
add_library(arm_hal SHARED
  src/sim_backend.cpp
  src/can_backend.cpp
  src/joint_limits.cpp
)

# Link the HAL library against ROS2 and XML dependencies
ament_target_dependencies(arm_hal
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(arm_hal
    tinyxml2
)

#=====================================
# arm_control_node executable
#=====================================

add_executable(arm_control_node
  src/arm_control_node.cpp
)

# Link the control node against our HAL library and ROS2 dependencies
target_link_libraries(arm_control_node
  arm_hal
)

ament_target_dependencies(arm_control_node
  rclcpp
  sensor_msgs
  std_msgs
)

#=====================================
# Install rules
#=====================================

# Install the executable so ros2 run can find it
install (TARGETS
  arm_control_node
  arm_hal
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files so other packages can use the HAL interface
install(DIRECTORY include/
  DESTINATION include
)

# Install resource directories
foreach(dir config launch meshes urdf)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

# Install RViz config
install(FILES urdf.rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/joint_cmd_cli.py
  DESTINATION lib/${PROJECT_NAME}
)

#=====================================
# Testing
#=====================================
if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # sim_backend tests
  ament_add_gtest(test_sim_backend
    test/test_sim_backend.cpp
  )
  target_link_libraries(test_sim_backend
    arm_hal
  )
  ament_target_dependencies(test_sim_backend
    rclcpp
    sensor_msgs
    std_msgs
  )

  # can_backend tests
  ament_add_gtest(test_can_backend
    test/test_can_backend.cpp
  )
  target_link_libraries(test_can_backend
    arm_hal
  )
  ament_target_dependencies(test_can_backend
    rclcpp
    sensor_msgs
    std_msgs
  )
endif()

ament_package()
