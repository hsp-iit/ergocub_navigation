#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
//#include "yarp/os/BufferedPort.h"
//#include "yarp/sig/Vector.h"
//#include "yarp/os/Network.h"

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

class PhaseDetector : public rclcpp::Node
{
private:
    const std::string m_leftFoot_topic = "/left_foot_heel_tiptoe_ft";
    const std::string m_rightFoot_topic = "/right_foot_heel_tiptoe_ft";
    const std::string m_imu_topic = "/head_imu";
    const std::string m_referenceFrame_right = "r_sole";
    const std::string m_referenceFrame_left = "l_sole";
    const double m_wrench_threshold = 80.0;
    const double m_imu_threshold_y = 0.3;
    const double m_tf_height_threshold_m = 0.02;    //height setpoint 0.05 m

    FootState m_rightFootState, m_leftFootState;

    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;
    std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr m_rightFoot_sub;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr m_leftFoot_sub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_debug_pub;

    void leftFootCallback(const geometry_msgs::msg::WrenchStamped::ConstPtr &msg);
    void rightFootCallback(const geometry_msgs::msg::WrenchStamped::ConstPtr &msg);
    void imuCallback(const sensor_msgs::msg::Imu::ConstPtr &msg);

    //Neck controller
    void gazeCallback(bool directionLeft);   //Main loop
    bool gazePattern(bool directionLeft);   //true for positive rotations = left
    const double m_joint_limit_deg = 20.0;   //swing around 0 +- this limit
    const std::string m_joint_name = "neck_yaw";
    const double m_joint_increment = 5.0;    //increase the joint sepoint by a constant quantity
    const rclcpp::Duration m_time_increment = 200ms;
    double m_joint_state;
    bool m_startup;
    const std::string m_out_port_name = "/neck_controller/setpoints:o";
    const std::string m_in_port_name = "/tmp/tmp:i";
    //yarp::os::BufferedPort<yarp::sig::VectorOf<double>> m_port;

    //Debug only
    int m_counter_rightSteps, m_counter_leftSteps;
    std::chrono::_V2::high_resolution_clock::time_point m_last_impact_time_left, m_approaching_time_left;
    std::chrono::_V2::high_resolution_clock::time_point m_last_impact_time_right, m_approaching_time_right;
    std::chrono::_V2::high_resolution_clock::time_point m_vibration_time;

public:
    PhaseDetector();
};
