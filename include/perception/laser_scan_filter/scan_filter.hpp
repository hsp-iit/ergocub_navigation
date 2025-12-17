/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SCAN_FILTER__HPP
#define SCAN_FILTER__HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "laser_geometry/laser_geometry.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/filters/project_inliers.h"

#include <mutex>

class ScanFilter : public rclcpp_lifecycle::LifecycleNode
{
private:
    // Parameter List
    std::string m_referece_frame = "geometric_unicycle";    // reference frame to which all the computations are performed
    std::string m_scan_topic = "/scan_local";               // topic name of the lidar scan
    std::string m_pub_topic = "/compensated_pc2";           // name of the topic where the filtered pointcloud will be published
    std::string m_imu_topic = "/head_imu";                  // name of the imu topic used to filter out high speed measurements
    laser_geometry::LaserProjection m_projector;            // object that converts a 2D lidar in a 3D cartesian pointcloud

    float m_filter_z_low = 0.2;             // minimum height to accept laser readings, from the reference frame, in meters
    float m_filter_z_high = 2.5;            // maximum height to accept laser readings, from the reference frame, in meters
    double m_close_threshold = 0.5;         // exlude points with distance lower than this value, used only when the robot is on the crane
    double m_imuVel_x_threshold = 0.4;      // speed threshold on the x-component red by the imu from which exclude laser readings
    double m_imuVel_y_threshold = 0.4;      // speed threshold on the y-component red by the imu from which exclude laser readings
    double m_ms_wait = 400.0;               // milliseconds to wait before accepting new laser readings, after detecting a velocity higher than a velocity threshold
    std::chrono::system_clock::time_point m_last_vibration_detection;   // latest time point at which a high vibration has been detected

    bool m_robot_on_crane = true;   // flag that states whether the robot is on the crane or not, so if to exclude closer points from scans

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_in;

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_raw_scan_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    geometry_msgs::msg::Vector3 m_imu_angular_velocity;
    std::mutex m_imu_mutex;     // mutex used for imu readings

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstPtr& scan_in);

    void imuCallback(const sensor_msgs::msg::Imu::ConstPtr& imu_msg);

public:
    ScanFilter(const rclcpp::NodeOptions & options);

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
    CallbackReturn on_error(const rclcpp_lifecycle::State & state);
};  // End of class ScanFilter node

#endif
