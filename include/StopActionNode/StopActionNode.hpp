/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_util/service_client.hpp"

#include "yarp/os/Bottle.h"
#include "yarp/os/Port.h"

#include <memory>

using namespace std::chrono_literals;

class StopActionNode : public rclcpp_lifecycle::LifecycleNode
{
private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr m_nav_client;
    rclcpp::CallbackGroup::SharedPtr m_callback_group;
    rclcpp::executors::SingleThreadedExecutor m_callback_group_executor;
    std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> m_future_goal_handle;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_goal_handle_;
    
    // The (non-spinning) client node used to invoke the action client
    rclcpp::Node::SharedPtr client_node_;

    rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr m_navigation_feedback_sub;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr m_navigation_result_sub;

    // PARAMETERS
    std::string m_remote_yarp_port_name;
    std::string m_remote_bt_port_name;

public:
    StopActionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~StopActionNode();

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
    CallbackReturn on_error(const rclcpp_lifecycle::State & state);

    bool cancelGoal();
};
