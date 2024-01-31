/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "yarp/os/BufferedPort.h"
#include "yarp/sig/Vector.h"
#include "yarp/os/Network.h"

#include "PhaseDetector/MotorControl.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include <memory>
#include <chrono>
#include <mutex>

using namespace std::literals::chrono_literals;

enum FootState
{
    inContact = 0,
    leavingContact,
    apexZone,
    approachingContact
};

class PhaseDetector : public rclcpp_lifecycle::LifecycleNode
{
private:
    std::string m_leftFoot_topic = "/left_foot_heel_tiptoe_ft";
    std::string m_rightFoot_topic = "/right_foot_heel_tiptoe_ft";
    std::string m_imu_topic = "/head_imu";
    std::string m_referenceFrame_right = "r_sole";
    std::string m_referenceFrame_left = "l_sole";
    double m_wrench_threshold = 80.0;
    double m_imu_threshold_y = 0.3;
    double m_tf_height_threshold_m = 0.02;    //height setpoint 0.05 m
    //yarp::os::BufferedPort<yarp::sig::VectorOf<double>> m_port;

    FootState m_rightFootState, m_leftFootState;

    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;
    std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr m_rightFoot_sub;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr m_leftFoot_sub;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr m_debug_pub;

    void leftFootCallback(const geometry_msgs::msg::WrenchStamped::ConstPtr &msg);
    void rightFootCallback(const geometry_msgs::msg::WrenchStamped::ConstPtr &msg);
    void imuCallback(const sensor_msgs::msg::Imu::ConstPtr &msg);

    //Neck controller
    void gazeCallback(bool directionLeft);   //Main loop
    bool gazePattern(bool directionLeft);   //true for positive rotations = left
    double m_joint_limit_deg;   //swing around 0 +- this limit
    double m_joint_increment = 5.0;    //increase the joint sepoint by a constant quantity
    std::vector<std::string> m_joint_name;
    rclcpp::Duration m_time_increment = 200ms;
    double m_joint_state;
    bool m_startup;
    std::string m_out_port_name = "/neck_controller/setpoints:o";
    std::vector<std::string> m_in_port_name;
    MotorControl m_jointInterface;

    //Debug only
    int m_counter_rightSteps, m_counter_leftSteps;
    std::chrono::_V2::high_resolution_clock::time_point m_last_impact_time_left, m_approaching_time_left;
    std::chrono::_V2::high_resolution_clock::time_point m_last_impact_time_right, m_approaching_time_right;
    std::chrono::_V2::high_resolution_clock::time_point m_vibration_time;

public:
    PhaseDetector(const rclcpp::NodeOptions & options);

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
    CallbackReturn on_error(const rclcpp_lifecycle::State & state);
    ~PhaseDetector();
};
