/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef NODE_ROS__HPP
#define NODE_ROS__HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_2d_msgs/msg/pose2_d_stamped.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "yarp/os/Bottle.h"

#include <memory>
#include <chrono>

class NodeRos : public rclcpp::Node
{
private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_goalMarkersPub;
    const std::string m_goalMarkersTopicName = "/human_pose_goal_gen/goal";

    std::shared_ptr<tf2_ros::TransformListener> m_tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;
    
public:
    NodeRos();

    bool publishMarkers(const yarp::os::Bottle& data);
    bool publishGoal(const yarp::os::Bottle& data);
};  //End of class NodeRos

#endif
