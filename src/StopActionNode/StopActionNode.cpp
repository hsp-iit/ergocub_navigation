/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "StopActionNode/StopActionNode.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "yarp/os/Network.h"
#include "yarp/os/LogStream.h"

using std::placeholders::_1;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
StopActionNode::StopActionNode(const rclcpp::NodeOptions & options): rclcpp_lifecycle::LifecycleNode("stop_action_node", options)
{
    //declare_parameter("stop_action_port",
    //rclcpp::ParameterValue(std::string("/BT/stop_action:o")));
}

CallbackReturn StopActionNode::on_configure(const rclcpp_lifecycle::State &)
{
    //Action Client
    m_callback_group = create_callback_group(
                            rclcpp::CallbackGroupType::MutuallyExclusive,
                            false);
    m_callback_group_executor.add_callback_group(m_callback_group, get_node_base_interface());
    m_nav_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
                            get_node_base_interface(),
                            get_node_graph_interface(),
                            get_node_logging_interface(),
                            get_node_waitables_interface(),
                            "navigate_to_pose", m_callback_group);

    /*m_navigation_feedback_sub = this->create_subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
                "navigate_to_pose/_action/feedback",
                rclcpp::SystemDefaultsQoS(),
                // Write lambda function to what to do with the feedback
                [this](const nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::SharedPtr msg) {   
                    RCLCPP_INFO( this->get_logger(), "Remaining distance from action feedback: %f", msg->feedback.distance_remaining);
                    {
                        //mutex maybe
                    if (navigation_goal_handle_!=nullptr)
                    {
                        RCLCPP_INFO(client_node_->get_logger(), "FEEDBACK status: %i", navigation_goal_handle_->get_status());
                    }
                    }
                });
    m_navigation_result_sub = this->create_subscription<action_msgs::msg::GoalStatusArray>(
                "navigate_to_pose/_action/status",
                rclcpp::SystemDefaultsQoS(),
                [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {   
                    RCLCPP_INFO( this->get_logger(), "GOAL STATUS: %i", msg->status_list.back().status);
                    yarp::os::Bottle data;
                    data.addInt32(msg->status_list.back().status);
                    m_nav_status_port.write(data);

                });*/
    client_node_ = std::make_shared<rclcpp::Node>("nav_action_client_node_stop_action");

    return CallbackReturn::SUCCESS;
}

CallbackReturn StopActionNode::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating");
    return CallbackReturn::SUCCESS;
}
CallbackReturn StopActionNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating");
    return CallbackReturn::SUCCESS;
}

CallbackReturn StopActionNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    m_nav_client.reset();
    RCLCPP_INFO(get_logger(), "Cleaning up");
    return CallbackReturn::SUCCESS;
}

CallbackReturn StopActionNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Shutting down");
    return CallbackReturn::SUCCESS;
}
CallbackReturn StopActionNode::on_error(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "On error");
    return CallbackReturn::SUCCESS;
}

StopActionNode::~StopActionNode()
{
}

bool StopActionNode::cancelGoal(){
    if (m_nav_client->action_server_is_ready())
    {
        m_nav_client->async_cancel_all_goals();
        return true;
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Navigation Client is not ready");
        return false;
    }
    
    return true;
};