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

find_package(tf2_sensor_msgs REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

#include_directories(/opt/ros/humble/include/tf2_geometry_msgs)
#include_directories(/opt/ros/humble/include/tf2)
#include_directories(/opt/ros/humble/include/tf2_ros)
#include_directories(/opt/ros/humble/include/laser_geometry)
#include_directories(/opt/ros/humble/include/pcl_conversions)
#include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(/opt/ros/iron/include/tf2_geometry_msgs)
include_directories(/opt/ros/iron/include/tf2)
include_directories(/opt/ros/iron/include/tf2_ros)
include_directories(/opt/ros/iron/include/laser_geometry)
include_directories(/opt/ros/iron/include/pcl_conversions)
include_directories(include)
add_subdirectory(interfaces)


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
  "rclcpp_action"
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
  "rclcpp_lifecycle"
  "laser_geometry"
  "pcl_conversions"
  "geometry_msgs"
  "sensor_msgs"
  "tf2_sensor_msgs"
  "tf2_geometry_msgs"
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
  "tf2_geometry_msgs"
)

add_executable(odom_node src/OdomNode.cpp)
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

add_executable(pointcloud_filter src/PointcloudFilter.cpp)
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

add_executable(path_converter_v2 src/PathConverter_v2.cpp)
ament_target_dependencies(path_converter_v2
  "YARP"
  "rclcpp"
  "nav_msgs"
  "tf2_ros"
  "tf2_msgs"
  "tf2"
  "nav2_util"
  "rclcpp_lifecycle"
)

add_executable(phase_detector src/PhaseDetector.cpp src/MotorControl.cpp) 
ament_target_dependencies(phase_detector
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

add_executable(human_pose_goal_generator src/HumanPoseGoalGenerator/main.cpp src/HumanPoseGoalGenerator/GoalGenerator.cpp 
                src/HumanPoseGoalGenerator/CommunicationWrapper.cpp)
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

add_library(ergocub_local_human_avoidance_controller SHARED
        src/LocalHumanAvoidance.cpp src/ControlInterface.cpp)

# prevent pluginlib from using boost
target_compile_definitions(ergocub_local_human_avoidance_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(ergocub_local_human_avoidance_controller
  ${dependencies}
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
  footsteps_viewer_
  base_projector
  path_converter
  pointcloud_filter
  phase_detector
  path_converter_v2
  human_pose_goal_generator
  ergocub_local_human_avoidance_controller
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

install(FILES ergocub_local_human_avoidance_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(ergocub_local_human_avoidance_controller)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core ergocub_local_human_avoidance_controller.xml)

ament_package()