#include "yarp/os/Bottle.h"
#include "yarp/os/BufferedPort.h"
#include "yarp/os/Network.h"
#include "yarp/sig/Vector.h"
#include "yarp/os/ConnectionWriter.h"
#include "yarp/os/Portable.h"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <list>
#include <vector>
#include <mutex>

//This class subscribes to the /path topic and will pass it to the walking-controller on a yarp port
class PathConverter_v2 : public rclcpp_lifecycle::LifecycleNode
{
private:
    double m_zero_speed_threshold;
    std::string m_topic_name = "/plan";
    std::string m_state_topic = "/is_goal_reached/goal_state";
    std::string m_outPortName = "/path_converter/path:o";
    std::string m_inPortName = "/walking-coordinator/goal:i";
    std::string m_reference_frame = "geometric_unicycle";  //virtual_unicycle_base
    yarp::os::BufferedPort<yarp::sig::VectorOf<double>> m_port;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_setpoint_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_state_sub;

    //Shifting path
    double m_shift_enabled;
    double m_shift = 0.15;
    bool m_shiftLeft;
    bool m_shiftFlag = false;
    yarp::os::BufferedPort<yarp::sig::VectorOf<double>> m_shift_port;
    std::string m_shift_portName = "/path_converter/shift_command:i";
    std::string m_shift_portConnectionName = "/TODO/shift_command:o";

    int msg_counter = 0;
    int m_max_msg_counter = 4;
    bool msg_num_reached = false;

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::mutex m_mutex;
    bool m_goalReached = false;

    //Each time I have a new path we transform the path and pass it to the walking controller
    void msg_callback(const nav_msgs::msg::Path::ConstPtr& msg_in);

    void state_callback(const std_msgs::msg::Bool::ConstPtr& in);

    nav_msgs::msg::Path transformPlan(const nav_msgs::msg::Path::ConstPtr& path, 
                                      geometry_msgs::msg::TransformStamped & t_tf, 
                                      bool t_prune = true);

    nav_msgs::msg::Path shiftPlan(const nav_msgs::msg::Path &path,
                                    bool directionLeft);

public:
    PathConverter_v2(const rclcpp::NodeOptions & options);

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
    CallbackReturn on_error(const rclcpp_lifecycle::State & state);
};