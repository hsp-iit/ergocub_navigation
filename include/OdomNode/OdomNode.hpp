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

#include <thread>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class OdomNode : public rclcpp::Node
{
private:
    const std::string port_name = "/virtual_unicycle_publisher/unicycle_states:i";
    const std::string m_odom_topic_name = "/odom";
    const std::array<double, 36UL> m_pose_cov_matrix = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                        0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.001};
    yarp::os::BufferedPort<yarp::os::Bottle> port;
    //yarp::os::Contact portContact{port_name, "shmem" };
    const double m_loopFreq = 100.0;
    const double m_nominalWidth = 0.2;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_in;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> m_odom_pub;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> m_control_pub;
public:
    OdomNode();

    void PublishOdom();
};  //End of class VirtualUnicyclePub