/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "HumanPoseGoalGenerator/GoalGenerator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point.hpp"

using std::placeholders::_1;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
GoalGenerator::GoalGenerator(const rclcpp::NodeOptions & options): rclcpp_lifecycle::LifecycleNode("goal_generator_node", options)
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
    declare_parameter("goal_markers_topic_name",
    rclcpp::ParameterValue(std::string("/human_pose_goal_gen/goal")));
    declare_parameter("transform_tolerance",
    rclcpp::ParameterValue(double(0.3)));
    declare_parameter("map_frame",
    rclcpp::ParameterValue(std::string("map")));

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
    m_goal_markers_topic_name = this->get_parameter("goal_markers_topic_name").as_string();
    m_tf_tol = this -> get_parameter("transform_tolerance").as_double();
    m_map_frame = this->get_parameter("map_frame").as_string();

    RCLCPP_INFO(this->get_logger(), "Configuring with: costmap_topic: %s ", m_costmap_topic_name.c_str());

    m_costmap_sub = std::make_unique<nav2_costmap_2d::CostmapSubscriber> (this->weak_from_this(), m_costmap_topic_name);
    m_goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(m_goal_topic_name, 10);
    m_goal_markers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(m_goal_markers_topic_name, 10);

    return CallbackReturn::SUCCESS;
}

CallbackReturn GoalGenerator::on_activate(const rclcpp_lifecycle::State &)
{
    m_goal_pub->on_activate();
    m_goal_markers_pub->on_activate();
    RCLCPP_INFO(get_logger(), "Activating");
    return CallbackReturn::SUCCESS;
}
CallbackReturn GoalGenerator::on_deactivate(const rclcpp_lifecycle::State &)
{
    m_goal_pub->on_deactivate();
    m_goal_markers_pub->on_deactivate();
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

bool GoalGenerator::publishMarkers(const double& x, const double& y, const double& z, geometry_msgs::msg::PoseStamped goal){

        visualization_msgs::msg::MarkerArray marker_array;
        //Clear the previous markers
        visualization_msgs::msg::Marker clear_msg;
        clear_msg.id = 0;
        clear_msg.ns = "my_namespace";
        clear_msg.action = visualization_msgs::msg::Marker::DELETEALL;

        marker_array.markers.push_back(clear_msg);
        m_goal_markers_pub->publish(marker_array);

        marker_array.markers.clear();
        builtin_interfaces::msg::Time timestamp = now();
        visualization_msgs::msg::Marker tmp_marker_msg;
        tmp_marker_msg.header.frame_id = m_pose_source_frame;
        tmp_marker_msg.id = 0;
        tmp_marker_msg.ns = "my_namespace";
        tmp_marker_msg.header.stamp = timestamp;
        tmp_marker_msg.scale.x = 0.05;
        tmp_marker_msg.scale.y = 0.05;
        tmp_marker_msg.scale.z = 0.05;
        // Color
        tmp_marker_msg.color.r = 0.0;
        tmp_marker_msg.color.g = 0.0;
        tmp_marker_msg.color.b = 1.0;
        tmp_marker_msg.color.a = 1.0;
        tmp_marker_msg.type = visualization_msgs::msg::Marker::ARROW;
        tmp_marker_msg.pose.position.x = x;
        tmp_marker_msg.pose.position.y = y;
        tmp_marker_msg.pose.position.z = z;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        tmp_marker_msg.pose.orientation = tf2::toMsg(q);
        tmp_marker_msg.frame_locked = true;
        tmp_marker_msg.action = visualization_msgs::msg::Marker::ADD;
        //Populate the marker with atleast one mesh point
        geometry_msgs::msg::Point cube_center;
        cube_center.x = 0.0;
        cube_center.y = 0.0;
        cube_center.z = 0.0;
        tmp_marker_msg.points.push_back(cube_center);
        cube_center.x = 0.1;
        cube_center.y = 0.0;
        cube_center.z = 0.0;
        tmp_marker_msg.points.push_back(cube_center);
        //save marker in the array
        marker_array.markers.push_back(tmp_marker_msg);

        // GOAL MARKER
        tmp_marker_msg.id = 1;
        tmp_marker_msg.pose.position.x = goal.pose.position.x;
        tmp_marker_msg.pose.position.y = goal.pose.position.y;
        tmp_marker_msg.pose.position.z = goal.pose.position.z;
        tmp_marker_msg.pose.orientation = goal.pose.orientation;
        tmp_marker_msg.frame_locked = true;
        tmp_marker_msg.action = visualization_msgs::msg::Marker::ADD;
        //Populate the marker with atleast one mesh point
        cube_center.x = 0.0;
        cube_center.y = 0.0;
        cube_center.z = 0.0;
        tmp_marker_msg.points.push_back(cube_center);
        cube_center.x = 0.1;
        cube_center.y = 0.0;
        cube_center.z = 0.0;
        tmp_marker_msg.points.push_back(cube_center);
        //save marker in the array
        marker_array.markers.push_back(tmp_marker_msg);

        m_goal_markers_pub->publish(marker_array);

        return true;
};

bool GoalGenerator::publishGoal(const yarp::os::Bottle& data){
    geometry_msgs::msg::Point realsense_point;
    realsense_point.x = data.get(0).asFloat64();
    realsense_point.y = data.get(1).asFloat64();
    realsense_point.z = data.get(2).asFloat64();
    
    if (m_tf_buffer->canTransform(m_reference_frame, m_pose_source_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(m_tf_tol)))
    {
        auto tf = m_tf_buffer->lookupTransform(m_reference_frame, m_pose_source_frame, rclcpp::Time(0));
        geometry_msgs::msg::Point transformed_point;
        tf2::doTransform(realsense_point, transformed_point, tf);
        geometry_msgs::msg::Pose odom_goal;
        double angle = std::atan2(transformed_point.x, transformed_point.y);
        odom_goal.position.x = transformed_point.x - m_goal_offset * std::cos(angle);
        odom_goal.position.x = transformed_point.y - m_goal_offset * std::sin(angle);
        odom_goal.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, angle);

        geometry_msgs::msg::PoseStamped map_goal;
        map_goal.header.frame_id = m_map_frame;
        map_goal.header.stamp = now();
        auto reference_map_tf = m_tf_buffer->lookupTransform(m_map_frame, m_reference_frame, rclcpp::Time(0));
        tf2::doTransform(odom_goal, map_goal.pose, reference_map_tf);
        // TODO check on costmap
        // m_goal_pub->publish(map_goal);    //Do I need to call the action or is enough to publish a goal?
        this->publishMarkers(realsense_point.x, realsense_point.y, realsense_point.z, map_goal);
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Cannot transform: %s to %s", m_pose_source_frame.c_str(), m_reference_frame.c_str());
        return false;
    }
    
    
    // TODO publish Goal
    
    return true;
};