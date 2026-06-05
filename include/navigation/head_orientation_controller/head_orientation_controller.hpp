/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef ERGOCUB_NAVIGATION__HEAD_ORIENTATION_CONTROLLER_HPP
#define ERGOCUB_NAVIGATION__HEAD_ORIENTATION_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>

#include <atomic>
#include <mutex>
#include <memory>
#include <string>

class HeadOrientationController : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit HeadOrientationController(const rclcpp::NodeOptions & options);
    ~HeadOrientationController();

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
    CallbackReturn on_error(const rclcpp_lifecycle::State & state);

private:
    // Parameters
    std::string m_plan_topic;
    std::string m_robot_base_frame;
    std::string m_map_frame;
    double m_lookahead_size{2.0};
    double m_update_rate_hz{10.0};
    double m_pitch_angle_deg{15.0};
    double m_max_yaw_deg{60.0};
    double m_max_pitch_deg{30.0};
    double m_min_pitch_deg{-10.0};
    std::string m_head_rpc_server;
    std::string m_head_rpc_client;

    // ROS2
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_plan_sub;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr m_nav_status_sub;
    rclcpp::TimerBase::SharedPtr m_timer;

    // TF2 — created in constructor to warm up cache before on_configure
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

    // YARP — plain Port (one-way write); the head controller uses a BufferedPort that never replies
    yarp::os::Port m_rpc_client;
    bool m_yarp_connected{false};

    // State
    nav_msgs::msg::Path::SharedPtr m_latest_plan;
    std::mutex m_plan_mutex;
    rclcpp::Time m_last_plan_time{0, 0, RCL_ROS_TIME};
    double m_plan_timeout_sec{5.0};       // safety fallback
    std::atomic<bool> m_navigation_active{false};  // primary: set by action status

    // Callbacks
    void planCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void navStatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
    void timerCallback();

    // YARP helpers
    bool sendOrientationMatrix(double r11, double r12, double r13,
                               double r21, double r22, double r23,
                               double r31, double r32, double r33);
    bool sendGoHome();
    bool sendStop();
    void closeYarp();
};

#endif  // ERGOCUB_NAVIGATION__HEAD_ORIENTATION_CONTROLLER_HPP
