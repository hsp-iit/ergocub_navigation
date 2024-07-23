/**
 *
 * @Author : Vignesh Sushrutha Raghavan
 * @date   : June 2024
 * @Brief  : Header for the LocalHumanAvoidancePlugin
 **/

#ifndef ERGOCUB_LOCAL_HUMAN_AVOIDANCE_HPP_
#define ERGOCUB_LOCAL_HUMAN_AVOIDANCE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include "ControlInterface.h"

namespace ergocub_local_human_avoidance
{
    class HumanAvoidanceController : public nav2_core::Controller
    {
    public:
        /**
         * Constructor and Destructor for the controller Plugin
         **/

        HumanAvoidanceController() = default;
        ~HumanAvoidanceController() = default;

        /**
         * Configure function to be called by the navigator
         **/
        void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                       std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
                       const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

        /**
         * Cleanup,activation and deactivation functions for the navigator
         **/
        void cleanup() override;
        void activate() override;
        void deactivate() override;
        void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

        /**
         * Function to compute velocity commands
         **/

        geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                                                                 const geometry_msgs::msg::Twist &velocity,
                                                                 nav2_core::GoalChecker *goal_checker) override;

        /**
         * Function to set global plan to the local variable
         **/
        void setPlan(const nav_msgs::msg::Path &path) override;

    protected:
        // nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped &pose);

        /*bool transformPose(
            const std::shared_ptr<tf2_ros::Buffer> tf,
            const std::string frame,
            const geometry_msgs::msg::PoseStamped &in_pose,
            geometry_msgs::msg::PoseStamped &out_pose,
            const rclcpp::Duration &transform_tolerance) const;*/

        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::string plugin_name_;
        std::string human_left_frame_;
        std::string human_right_frame_;
        std::string human_tf_base_frame_;
        std::string nav_shift_port_name_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        rclcpp::Logger logger_{rclcpp::get_logger("HumanAvoidanceController")};
        rclcpp::Clock::SharedPtr clock_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;

        double desired_linear_vel_;
        double lookahead_dist_;
        double max_angular_vel_;
        double object_size_;
        double safe_dist_to_human_;
        double current_human_horizontal_dist_;
        double human_dist_threshold_;
        bool nav_shift_enabled_;
        rclcpp::Duration transform_tolerance_{0, 0};

        nav_msgs::msg::Path global_plan_;

        double obj_max_translation_;
        double obj_max_rotation_;
        double obj_translation_slope_;
        double obj_orientation_slope_;

        // yarp stuff
        yarp::os::Network yarp_;
        yarp::os::Port bimannual_port_;
        yarp::os::BufferedPort<yarp::os::Bottle> nav_shift_port_;
        ControlInterface bimanual_client_;
        bool obj_pose_action_executed_;
        bool reset_executed_;
    };

} // end of namespace

#endif // ERGOCUB_LOCAL_HUMAN_AVOIDANCE_HPP_