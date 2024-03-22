/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_util/service_client.hpp"

#include "yarp/os/Bottle.h"
#include "yarp/os/Port.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include <memory>

using namespace std::chrono_literals;

class GoalGenerator : public rclcpp_lifecycle::LifecycleNode
{
private:
    std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> m_costmap_sub;
    std::string m_costmap_topic_name;

    std::string m_goal_topic_name;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_goal_pub;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_goal_markers_pub;

    //nav2_util::ServiceClient<nav2_msgs::action::NavigateToPose_Feedback> m_navigation_client;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr m_nav_client;
    rclcpp::CallbackGroup::SharedPtr m_callback_group;
    rclcpp::executors::SingleThreadedExecutor m_callback_group_executor;
    std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> m_future_goal_handle;
    nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_goal_handle_;
    // The (non-spinning) client node used to invoke the action client
    rclcpp::Node::SharedPtr client_node_;
    std::chrono::milliseconds  m_server_timeout = 100ms;

    rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr m_navigation_feedback_sub;
    yarp::os::Port m_nav_status_port;

    // PARAMETERS
    std::string m_goal_markers_topic_name = "/human_pose_goal_gen/goal";
    std::string m_pose_source_frame;
    std::string m_reference_frame;
    double m_goal_offset;
    std::string m_remote_yarp_port_name;
    double m_tf_tol;
    std::string m_map_frame;

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;

    void poseCallback();

public:
    GoalGenerator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~GoalGenerator();

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
    CallbackReturn on_error(const rclcpp_lifecycle::State & state);

    bool publishMarkers(const double& x, const double& y, const double& z, geometry_msgs::msg::Pose goal);
    bool publishGoal(const yarp::os::Bottle& data);
};
