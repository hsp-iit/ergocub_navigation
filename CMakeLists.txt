cmake_minimum_required(VERSION 3.8)
project(ergocub_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
FIND_PACKAGE(YARP COMPONENTS os sig dev REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_eigen)

find_package(tf2_sensor_msgs REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)
add_subdirectory(interfaces)


add_executable(planner_trigger_server src/navigation/walking_planning/planner_trigger_server.cpp)
ament_target_dependencies(planner_trigger_server
  "YARP"
  "rclcpp"
  "std_srvs"
  "rclcpp_action"
)

add_executable(scan_filter src/perception/laser_scan_filter/scan_filter.cpp)
ament_target_dependencies(scan_filter
  "std_msgs"
  "tf2_ros"
  "tf2_msgs"
  "rclcpp"
  "rclcpp_lifecycle"
  "laser_geometry"
  "pcl_conversions"
  "geometry_msgs"
  "sensor_msgs"
  "tf2_sensor_msgs"
  "tf2_geometry_msgs"
)
target_link_libraries(scan_filter ${PCL_LIBRARIES})

add_executable(footsteps_viewer src/visualization/footsteps_viewer/main.cpp 
                                src/visualization/footsteps_viewer/footsteps_viewer_yarp.cpp 
                                src/visualization/footsteps_viewer/footsteps_viewer_ros.cpp)
ament_target_dependencies(footsteps_viewer
  "YARP"
  "rclcpp"
  "nav_msgs"
  "tf2_ros"
  "tf2_msgs"
  "tf2"
  "nav2_util"
  "nav_2d_msgs"
  "visualization_msgs"
)

#add_executable(base_projector src/visualization/base_projector/base_projector.cpp)
#ament_target_dependencies(base_projector
#  "std_msgs"
#  "tf2_ros"
#  "tf2_msgs"
#  "YARP"
#  "rclcpp"
#  "tf2_geometry_msgs"
#)

add_executable(odom_node src/odom/odom_node.cpp)
ament_target_dependencies(odom_node
  "YARP"
  "rclcpp"
  "rclcpp_lifecycle"
  "tf2_ros"
  "tf2_msgs"
  "tf2"
  "tf2_kdl"
  "nav_msgs"
  "tf2_geometry_msgs"
)

add_executable(pointcloud_filter src/perception/pointcloud_filter/pointcloud_filter.cpp)
ament_target_dependencies(pointcloud_filter
  "std_msgs"
  "tf2_ros"
  "tf2_msgs"
  "rclcpp"
  "pcl_conversions"
  "geometry_msgs"
  "sensor_msgs"
  "rclcpp_lifecycle"
  "tf2_ros"
  "pcl_ros"
)
target_link_libraries(pointcloud_filter ${PCL_LIBRARIES})

add_executable(path_converter src/navigation/walking_planning/path_converter.cpp)
ament_target_dependencies(path_converter
  "YARP"
  "rclcpp"
  "nav_msgs"
  "tf2_ros"
  "tf2_msgs"
  "tf2"
  "nav2_util"
  "rclcpp_lifecycle"
)

add_executable(walking_phase_detector src/perception/walking_phase_detector/walking_phase_detector.cpp src/perception/walking_phase_detector/neck_motor_control.cpp) 
ament_target_dependencies(walking_phase_detector
  "YARP"
  "rclcpp"
  "nav_msgs"
  "tf2_ros"
  "tf2_msgs"
  "tf2"
  "sensor_msgs"
  "rclcpp_lifecycle"
  "tf2_geometry_msgs"
)

add_executable(human_pose_goal_generator src/navigation/human_pose_goal_generator/main.cpp 
                                        src/navigation/human_pose_goal_generator/goal_generator.cpp
                                        src/navigation/human_pose_goal_generator/yarp_communication_wrapper.cpp)
ament_target_dependencies(human_pose_goal_generator
  "YARP"
  "rclcpp"
  "rclcpp_lifecycle"
  "nav_msgs"
  "tf2_ros"
  "tf2_msgs"
  "tf2"
  "nav2_util"
  "nav_2d_msgs"
  "visualization_msgs"
  "nav2_costmap_2d"
  "rclcpp_action"
)

add_executable(plane_detector src/perception/plane_detector/plane_detector.cpp)
ament_target_dependencies(plane_detector
  "rclcpp"
  "rclcpp_lifecycle"
  "tf2_ros"
  "tf2_msgs"
  "tf2"
  "tf2_sensor_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "pcl_ros"
  "tf2_eigen"
)
target_link_libraries(plane_detector ${PCL_LIBRARIES})

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav2_msgs
  nav_msgs
  nav2_util
  nav2_core
  tf2
  YARP
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  odom_node
  planner_trigger_server
  scan_filter
  footsteps_viewer
  pointcloud_filter
  walking_phase_detector
  path_converter
  human_pose_goal_generator
  plane_detector
  DESTINATION lib/${PROJECT_NAME})

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
# install the maps directory
install(DIRECTORY
  config/maps
  DESTINATION share/${PROJECT_NAME}/
)
# install the param directory
install(DIRECTORY
  config/param
  DESTINATION share/${PROJECT_NAME}/
)
# install the RVIZ directory
install(DIRECTORY
  config/rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)


ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()