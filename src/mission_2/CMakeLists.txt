cmake_minimum_required(VERSION 3.5)
project(mission_2)

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
# Mission 2: CAV control
# =============================================================================
add_executable(control_cav_mission_2
  src/control_cav_mission_2.cpp
  src/Global/Global.cpp
  src/Utils/Utils.cpp
  src/Planning/Planning.cpp
  src/Control/Control.cpp
  src/Mission/Mission.cpp
)
target_include_directories(control_cav_mission_2 PRIVATE src)
ament_target_dependencies(control_cav_mission_2
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  ament_index_cpp
)
install(TARGETS control_cav_mission_2
  DESTINATION lib/${PROJECT_NAME})

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
