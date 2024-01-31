#include "yarp/os/Bottle.h"
#include "yarp/os/Port.h"
#include "yarp/os/BufferedPort.h"
#include "yarp/os/Network.h"
#include "yarp/sig/Vector.h"
#include "yarp/os/ConnectionWriter.h"
#include "yarp/os/Portable.h"
#include "yarp/os/Contact.h"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include <thread>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class OdomNode : public rclcpp_lifecycle::LifecycleNode
{
private:
    //Parameters
    bool m_ekf_enabled;             // flag if the extended kalman filter is being externally used
    std::string m_in_port_name;     // yarp port name of this module
    std::string m_out_port_name;    // yarp port name of the walking-controller
    std::string m_odom_topic_name;  // name of the odom topic
    std::string m_vel_topic;        // name of hte odom vel topic estimated by the walking-controller
    double m_loopFreq;              // frequency of the node, used by the wall timer
    double m_nominalWidth;          // nominal width of the robot feet
    std::string m_odom_frame_name;  // frame name to give to the odom
    double m_delta_x = 0.1;         // offset used by the walking-controller for the reference of the virtual unicycle
    bool m_expose_ulterior_frames;  // flag to whether publish uterior frames for debug purposes, about internal measurements used in the walking controller

    std::array<double, 36UL> m_pose_cov_matrix = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                        0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.001};    // covariance matrix of the odometry, X, Y and Theta values only
    yarp::os::BufferedPort<yarp::os::Bottle> port;

    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_in;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr m_control_pub;
public:
    OdomNode(const rclcpp::NodeOptions & options);

    void PublishOdom();

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
    CallbackReturn on_error(const rclcpp_lifecycle::State & state);
};  //End of class OdomNode