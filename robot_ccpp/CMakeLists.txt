cmake_minimum_required(VERSION 3.8)
project(robot_ccpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies
  "std_msgs"
  "rclcpp"
  "tf2_ros"
  "nav2_costmap_2d"
  "geometry_msgs"
  "nav_msgs"
)

set(path_planning "path_planning_node")
add_executable(${path_planning} src/PathPlanningNode.cpp src/CleaningPathPlanner.cpp)
target_link_libraries(${path_planning} ${OpenCV_LIBS})
ament_target_dependencies(${path_planning} ${dependencies})

set(next_goal "next_goal_node")
add_executable(${next_goal} src/next_goal.cpp)
target_link_libraries(${next_goal} ${OpenCV_LIBS})
ament_target_dependencies(${next_goal} ${dependencies})
################################################################################
# Install
################################################################################
# 屏蔽生成的c++可执行文件，用python文件去运行
install(TARGETS ${path_planning} ${next_goal}
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include/
)

install (PROGRAMS
        scripts/CleaningPathPlanner.py
        scripts/next_goal.py
        scripts/PathPlanningNode.py
        scripts/simulate_tf2_pub.py
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch maps params rviz2
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
