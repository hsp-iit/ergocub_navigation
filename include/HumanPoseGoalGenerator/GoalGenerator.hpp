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

#include "yarp/os/Bottle.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include <memory>

class GoalGenerator : public rclcpp_lifecycle::LifecycleNode
{
private:
    std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> m_costmap_sub;
    std::string m_costmap_topic_name;

    std::string m_goal_topic_name;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_goal_pub;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_goal_markers_pub;

    // PARAMETERS
    std::string m_goal_markers_topic_name = "/human_pose_goal_gen/goal";
    std::string m_pose_source_frame;
    std::string m_reference_frame;
    double m_goal_offset;
    std::string m_remote_yarp_port_name;
    double m_tf_tol;
    std::string m_map_frame;

    // YARP port

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

    bool publishMarkers(const double& x, const double& y, const double& z, geometry_msgs::msg::PoseStamped goal);
    bool publishGoal(const yarp::os::Bottle& data);
};
