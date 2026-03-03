cmake_minimum_required(VERSION 3.8)
project(dual_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ───────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)

# ── Executable ─────────────────────────────────────────────────────────────────
add_executable(mtc_handover src/mtc_handover.cpp)
ament_target_dependencies(mtc_handover
  rclcpp
  moveit_task_constructor_core
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometry_msgs
  tf2_eigen
  tf2_geometry_msgs
)

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS mtc_handover
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz    DESTINATION share/${PROJECT_NAME})

ament_package()