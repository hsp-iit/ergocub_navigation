/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PATH_CONVERTER__V2__HPP
#define PATH_CONVERTER__V2__HPP

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
    std::string m_topic_name = "/plan";                         // topic name where the global plan is being published
    std::string m_state_topic = "/is_goal_reached";  // topic name of the state of the navigation
    std::string m_outPortName = "/path_converter/path:o";       // yarp port name of this module that connects with the walking controller
    std::string m_inPortName = "/walking-coordinator/goal:i";   // yarp port name of the walking controller
    std::string m_reference_frame = "geometric_unicycle";       // reference frame
    yarp::os::BufferedPort<yarp::sig::VectorOf<double>> m_port;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_setpoint_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_state_sub;

    //Shifting path
    double m_shift_enabled;     // flag if the possibility to shift the path of a fixed amount is enabled
    double m_shift = 0.15;      // amount of meter to shift to the left or right
    bool m_shiftLeft;           // if true shift to the left, to the right otherwise
    bool m_shiftFlag = false;   // bool if the shift has been actuated
    //yarp::os::BufferedPort<yarp::sig::VectorOf<double>> m_shift_port;
    yarp::os::BufferedPort<yarp::os::Bottle> m_shift_port;
    std::string m_shift_portName = "/path_converter/shift_command:i";
    std::string m_shift_portConnectionName = "/TODO/shift_command:o";

    int msg_counter = 0;
    int m_max_msg_counter = 6;
    bool msg_num_reached = false;

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::mutex m_mutex;             // mutex between the state callback and the path convertion
    bool m_goalReached = false;     // goal reached state red from a topic

    //Each time I have a new path we transform the path and pass it to the walking controller
    void msg_callback(const nav_msgs::msg::Path::ConstPtr& msg_in);

    void state_callback(const std_msgs::msg::Bool::ConstPtr& in);
    /**
     * @brief Transform the plan applying a given transform.
     * @param path Path to be transformed
     * @param t_tf The transform to be applied to the path
     * @param t_prune Flag to whether remove the negative part of the path, laying behind the reference frame
     * @return The transformed path
     */
    nav_msgs::msg::Path transformPlan(const nav_msgs::msg::Path::ConstPtr& path, 
                                      geometry_msgs::msg::TransformStamped & t_tf, 
                                      bool t_prune = false);

    /**
     * @brief Shift the path by a fixed amount to the left or right with rispect to the initial pose.
     * @param path Path to be shifted
     * @param directionLeft if true, shift to the left, if false, shift to the right
     * @return The shifted path
     */
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

#endif
