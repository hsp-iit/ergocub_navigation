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
FIND_PACKAGE(YARP COMPONENTS os sig REQUIRED)
find_package(PCL 1.2 REQUIRED)

find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_kdl REQUIRED)

find_package(iDynTree 4.3.0 REQUIRED)
find_package(UnicyclePlanner 0.5.2 REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(/opt/ros/humble/include/tf2_geometry_msgs)
include_directories(/opt/ros/humble/include/tf2)
include_directories(/opt/ros/humble/include/tf2_ros)
include_directories(/opt/ros/humble/include/laser_geometry)
include_directories(/opt/ros/humble/include/pcl_conversions)
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#add_executable(scan_filter_compensated src/scan_filtering_ros2_compensated.cpp)
#ament_target_dependencies(scan_filter_compensated
#  "std_msgs"
#  "tf2_ros"
#  "tf2_msgs"
#  "rclcpp"
#  "laser_geometry"
#  "pcl_conversions"
#  "geometry_msgs"
#  "sensor_msgs"
#)
#target_link_libraries(scan_filter_compensated ${PCL_LIBRARIES})

#add_executable(chest_projection src/chest_projection.cpp)
#ament_target_dependencies(chest_projection
#  "std_msgs"
#  "tf2_ros"
#  "tf2_msgs"
#  "YARP"
#  "rclcpp"
#)

#add_executable(footsteps_viewer src/footsteps_viewer.cpp)
#ament_target_dependencies(footsteps_viewer
#  "YARP"
#  "rclcpp"
#  "nav_msgs"
#  "tf2_ros"
#  "tf2_msgs"
#  "tf2"
#  "nav2_util"
#  "nav_2d_msgs"
#  "visualization_msgs"
#)

#add_executable(virtual_unicycle_publisher src/virtual_unicycle_publisher.cpp)
#ament_target_dependencies(virtual_unicycle_publisher
#  "YARP"
#  "rclcpp"
#  "tf2_ros"
#  "tf2_msgs"
#  "tf2"
#  "tf2_kdl"
#)

add_executable(planner_trigger_server src/planner_trigger_server.cpp)
ament_target_dependencies(planner_trigger_server
  "YARP"
  "rclcpp"
  "std_srvs"
)

#add_executable(interpolation_walking_controller_talker src/interpolation_walking_controller_talker.cpp)
#ament_target_dependencies(interpolation_walking_controller_talker
#  "YARP"
#  "rclcpp"
#  "nav_msgs"
#  "tf2_ros"
#  "tf2_msgs"
#  "tf2"
#  "nav2_util"
#  "nav_2d_msgs"
#)

add_executable(scan_filter src/ScanFilter.cpp)
ament_target_dependencies(scan_filter
  "std_msgs"
  "tf2_ros"
  "tf2_msgs"
  "rclcpp"
  "laser_geometry"
  "pcl_conversions"
  "geometry_msgs"
  "sensor_msgs"
)
target_link_libraries(scan_filter ${PCL_LIBRARIES})

add_executable(footsteps_viewer_ src/FootstepsViewer/main.cpp src/FootstepsViewer/FootstepsViewerYarp.cpp src/FootstepsViewer/FootstepsViewerRos.cpp)
ament_target_dependencies(footsteps_viewer_
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

add_executable(base_projector src/BaseProjector.cpp)
ament_target_dependencies(base_projector
  "std_msgs"
  "tf2_ros"
  "tf2_msgs"
  "YARP"
  "rclcpp"
)

add_executable(odom_node src/OdomNode.cpp)
ament_target_dependencies(odom_node
  "YARP"
  "rclcpp"
  "tf2_ros"
  "tf2_msgs"
  "tf2"
  "tf2_kdl"
)

add_executable(path_converter src/PathConverter.cpp)
ament_target_dependencies(path_converter
  "YARP"
  "rclcpp"
  "nav_msgs"
  "tf2_ros"
  "tf2_msgs"
  "tf2"
  "nav2_util"
  "nav_2d_msgs"
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
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
#  scan_filter_compensated
#  chest_projection
#  footsteps_viewer
#  virtual_unicycle_publisher
  odom_node
  planner_trigger_server
# interpolation_walking_controller_talker
  scan_filter
  footsteps_viewer_
  base_projector
  path_converter
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

ament_package()