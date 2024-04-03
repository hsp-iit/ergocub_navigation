/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef BASE_PROJECTOR__HPP
#define BASE_PROJECTOR__HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

#include <chrono>
#include <thread>
#include <mutex>

class BaseProjector : public rclcpp::Node
{
private:
    /* const */
    const std::string m_reader_port_name = "/chest_projector/wrench_reader:i";
    const std::string m_writer_port_name = "/feetWrenches";
    const double m_loopFreq = 100.0;
    const std::string m_chest_link = "chest";
    double m_sensor_treshold;
    bool m_ok = false;
    /* msgs */
    geometry_msgs::msg::TransformStamped m_TF;
    geometry_msgs::msg::TransformStamped m_projection_TF;
    /* TF objects*/
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_pub;
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer_in;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_{nullptr};
    /* YARP ports*/
    yarp::os::BufferedPort<yarp::os::Bottle> m_wrench_reader_port;
    /* ground contact var */
    std::string m_foot_link = "r_sole";
    //timer for loop
    rclcpp::TimerBase::SharedPtr m_timer_;
    std::mutex m_mutex;
    void timer_callback();
    bool get_TF(const std::string &target_link, const std::string &source_link);

    //virtual unicycle base position variables
    geometry_msgs::msg::TransformStamped m_initial_tf_right;
    geometry_msgs::msg::TransformStamped m_initial_tf_left;
    geometry_msgs::msg::TransformStamped m_virtual_unicycle_base_tf;
public:
    BaseProjector();
    ~BaseProjector();
};

#endif
