/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "navigation/human_pose_goal_generator/ros_node.hpp"

NodeRos::NodeRos() : rclcpp::Node("human_pose_goal_gen_node")
{   
    m_goalMarkersPub = this->create_publisher<visualization_msgs::msg::MarkerArray>(m_goalMarkersTopicName, 10);
};

bool NodeRos::publishMarkers(const yarp::os::Bottle& data){
        //std::cout << "publishMarkers" << std::endl;
        //std::cout << data.size() << std::endl;
        auto x = data.get(0).asFloat64();
        auto y = data.get(1).asFloat64();
        auto z = data.get(2).asFloat64();

        visualization_msgs::msg::MarkerArray marker_array;
        //Clear the previous markers
        visualization_msgs::msg::Marker clear_msg;
        clear_msg.id = 0;
        clear_msg.ns = "my_namespace";
        clear_msg.action = visualization_msgs::msg::Marker::DELETEALL;
        
        marker_array.markers.push_back(clear_msg);
        m_goalMarkersPub->publish(marker_array);

        marker_array.markers.clear();
        builtin_interfaces::msg::Time timestamp = now();
        visualization_msgs::msg::Marker tmp_marker_msg;
        tmp_marker_msg.header.frame_id = "realsense";
        tmp_marker_msg.id = 0;
        tmp_marker_msg.ns = "my_namespace";
        tmp_marker_msg.header.stamp = timestamp;
        tmp_marker_msg.scale.x = 0.05;
        tmp_marker_msg.scale.y = 0.05;
        tmp_marker_msg.scale.z = 0.05;
        // Color
        tmp_marker_msg.color.r = 0.0;
        tmp_marker_msg.color.g = 1.0;
        tmp_marker_msg.color.b = 0.0;
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
        
        RCLCPP_INFO(this->get_logger(), "Publishing");
        //std::cout << "Left Publish" << std::endl;
        m_goalMarkersPub->publish(marker_array);

        std::cout << "Exiting Pub" << std::endl;
};

bool NodeRos::publishGoal(const yarp::os::Bottle& data){
    auto x = data.get(0).asFloat64();
    auto y = data.get(1).asFloat64();
    auto z = data.get(2).asFloat64();
    // TODO publish Goal
};