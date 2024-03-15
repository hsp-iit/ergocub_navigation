/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "GoalGenerator.hpp"

using std::placeholders::_1;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
GoalGenerator::GoalGenerator(const rclcpp::NodeOptions & options): rclcpp_lifecycle::LifecycleNode("goal_generator_node", "", options)
{
    declare_parameter("costmap_topic",
    rclcpp::ParameterValue(std::string("global_costmap/costmap_raw")));
    declare_parameter("robot_reference_frame",
    rclcpp::ParameterValue(std::string("geometric_unicycle")));
    declare_parameter("pose_source_frame",
    rclcpp::ParameterValue(std::string("realsense")));
    declare_parameter("goal_offset",
    rclcpp::ParameterValue(double(0.3)));
    declare_parameter("human_pose_port",
    rclcpp::ParameterValue(std::string("/BT/human_pose:o")));

    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
}

CallbackReturn GoalGenerator::on_configure(const rclcpp_lifecycle::State &)
{
    m_costmap_topic_name = this->get_parameter("costmap_topic").as_string();
    m_reference_frame = this->get_parameter("robot_reference_frame").as_string();
    m_pose_source_frame = this->get_parameter("pose_source_frame").as_string();
    m_goal_offset = this -> get_parameter("goal_offset").as_double();
    m_remote_yarp_port_name = this->get_parameter("human_pose_port").as_string();

    RCLCPP_INFO(this->get_logger(), "Configuring with: costmap_topic: %s ", m_costmap_topic_name.c_str());

    m_costmap_sub = std::make_unique<nav2_costmap_2d::CostmapSubscriber> (this, m_costmap_topic_name);
    m_goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(m_goal_topic_name, 10);

    return CallbackReturn::SUCCESS;
}

CallbackReturn GoalGenerator::on_activate(const rclcpp_lifecycle::State &)
{
    m_goal_pub->on_activate();
    RCLCPP_INFO(get_logger(), "Activating");
    return CallbackReturn::SUCCESS;
}
CallbackReturn GoalGenerator::on_deactivate(const rclcpp_lifecycle::State &)
{
    m_goal_pub->on_deactivate();
    RCLCPP_INFO(get_logger(), "Deactivating");
    return CallbackReturn::SUCCESS;
}

CallbackReturn GoalGenerator::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning up");
    return CallbackReturn::SUCCESS;
}

CallbackReturn GoalGenerator::on_shutdown(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Shutting down");
    return CallbackReturn::SUCCESS;
}
CallbackReturn GoalGenerator::on_error(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "On error");
    return CallbackReturn::SUCCESS;
}

GoalGenerator::~GoalGenerator()
{
}