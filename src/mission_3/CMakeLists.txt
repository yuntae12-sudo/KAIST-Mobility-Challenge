cmake_minimum_required(VERSION 3.5)
project(mission_3)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_index_cpp REQUIRED)


# =============================================================================
# Mission 3: Motor Check
# =============================================================================
add_executable(check_motor
  src/check_motor/check_motor.cpp
  src/check_motor/Global/Global.cpp
  src/check_motor/Utils/Utils.cpp
  src/check_motor/Planning/Planning.cpp
  src/check_motor/Control/Control.cpp
  src/check_motor/Mission/Mission.cpp
)
target_include_directories(check_motor PRIVATE src/check_motor)
ament_target_dependencies(check_motor
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  ament_index_cpp
)
install(TARGETS check_motor
  DESTINATION lib/${PROJECT_NAME})


# =============================================================================
# Mission 3: CAV control
# =============================================================================
add_executable(control_cav_mission_3
  src/cav/control_cav_mission_3.cpp
  src/cav/Global/Global.cpp
  src/cav/Utils/Utils.cpp
  src/cav/Planning/Planning.cpp
  src/cav/Control/Control.cpp
  src/cav/Mission/Mission.cpp
)
target_include_directories(control_cav_mission_3 PRIVATE src/cav)
ament_target_dependencies(control_cav_mission_3
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  ament_index_cpp
)
install(TARGETS control_cav_mission_3
  DESTINATION lib/${PROJECT_NAME})


# =============================================================================
# Mission 3: Control Tower
# =============================================================================
add_executable(control_tower_mission_3
  src/tower/control_tower_mission_3.cpp
  src/tower/Global/Global.cpp
  src/tower/Utils/Utils.cpp
  src/tower/Planning/Planning.cpp
  src/tower/Tower/Tower.cpp
  src/tower/Visualizer/Visualizer.cpp
)
target_include_directories(control_tower_mission_3 PRIVATE src/tower)
ament_target_dependencies(control_tower_mission_3
  rclcpp
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2
  ament_index_cpp
)
install(TARGETS control_tower_mission_3
  DESTINATION lib/${PROJECT_NAME})



# =============================================================================
# Mission 3: Control rotary
# =============================================================================
add_executable(control_rotary_mission_3
  src/rotary/control_rotary_mission_3.cpp
  src/rotary/Global/Global.cpp
  src/rotary/Utils/Utils.cpp
  src/rotary/Planning/Planning.cpp
  src/rotary/Rotary/Rotary.cpp
  src/rotary/Visualizer/Visualizer.cpp
)
target_include_directories(control_rotary_mission_3 PRIVATE src/rotary)
ament_target_dependencies(control_rotary_mission_3 rclcpp std_msgs geometry_msgs visualization_msgs)
install(TARGETS control_rotary_mission_3 DESTINATION lib/${PROJECT_NAME})


# =============================================================================
# Install CSV files to share directory
# =============================================================================
install(DIRECTORY src/global_path/
  DESTINATION share/${PROJECT_NAME}/global_path
  FILES_MATCHING PATTERN "*.csv")

# Install launch files
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
    FILES_MATCHING PATTERN "*.py"
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
