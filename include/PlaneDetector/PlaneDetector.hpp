/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SCAN_FILTER__HPP
#define SCAN_FILTER__HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

class PlaneDetector : public rclcpp_lifecycle::LifecycleNode
{
private:
    // Parameter List
    std::string m_head_frame = "realsense";    // reference frame to which all the computations are performed
    std::string m_contact_frame = "r_sole";             // reference frame of the foot in contact with the ground
    std::string m_pc_topic = "/camera/depth/color/points";               // topic name of the lidar scan
    std::string m_pub_topic = "/adjusted_depth_pc";           // name of the topic where the filtered pointcloud will be published
    std::string m_right_foot_topic = "/right_foot_heel_ft";
    std::string m_left_foot_topic = "/left_foot_heel_ft";
    std::string m_frame_name = "compensated_realsense_frame";
    double m_wrench_threshold = 100.0;
    bool m_right_foot_contact, m_left_foot_contact;
    std::string m_realsense_frame = "realsense";
    std::vector<std::tuple<double, double>> m_tf_vec;
    bool m_debug_publish = false;
    geometry_msgs::msg::TransformStamped m_avg_tf;
    int m_sample_size = 5;                  // how many timess collect RANSAC readings before averaging roll and pitch angles
    float m_filter_z_low = 0.2;             // minimum height to accept laser readings, from the reference frame, in meters
    float m_filter_z_high = 2.5;            // maximum height to accept laser readings, from the reference frame, in meters

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_in;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_static_tf_broadcaster;

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_pub;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_plane_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pc_sub;
    // Unfortunately, message filters aren't available for lifecycle nodes
    //rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr m_right_foot_sub, m_left_foot_sub;

    void pc_callback(const sensor_msgs::msg::PointCloud2::ConstPtr& pc_in);
    //void right_foot_callback(const geometry_msgs::msg::WrenchStamped::ConstPtr &msg);
    //void left_foot_callback(const geometry_msgs::msg::WrenchStamped::ConstPtr &msg);
    geometry_msgs::msg::TransformStamped average_pitch(std::vector<std::tuple<double, double>> tf_vec_in, builtin_interfaces::msg::Time stmp);
    std::vector<geometry_msgs::msg::TransformStamped> median_filter(std::vector<geometry_msgs::msg::TransformStamped> tf_vec_in);

public:
    PlaneDetector(const rclcpp::NodeOptions & options);

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
    CallbackReturn on_error(const rclcpp_lifecycle::State & state);
};  // End of class PlaneDetector node

#endif
