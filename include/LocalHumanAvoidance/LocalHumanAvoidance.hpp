/**
 *
 * @Author : Vignesh Sushrutha Raghavan
 * @date   : June 2024
 * @Brief  : Header for the ergocub_local_human_avoidance::HumanAvoidanceController that plans object pose changes and path shift to avoid nearby humans.
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
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RpcClient.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <std_srvs/srv/trigger.hpp>

#include "ControlInterface.h"
namespace ergocub_local_human_avoidance
{
    class HumanAvoidanceController : public nav2_core::Controller // class declaration based on nav2_tutorials custom controller plugin
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
         * Plugin functions that is called to give velocity commands but in this case used to also determine object pose changes and nav shifts.
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
        std::string human_left_frame_;                               // Name of frame attached on the left extreme of the detected human
        std::string human_right_frame_;                              // Name of frame attached on the right extreme of the detected human
        std::string human_tf_base_frame_;                            // Name of frame attached on the pelvis of the detected human
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_; // Variable to store local command
        rclcpp::Logger logger_{rclcpp::get_logger("HumanAvoidanceController")};
        rclcpp::Clock::SharedPtr clock_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_; // publisher to publish path
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr path_trigger_client_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  // To broadcast any transform if needed

        double max_local_path_dist_;                 // maximum distance of allowed for global path point w.r.t current pose to be considered a local path point.
        double desired_linear_vel_;                  // velocity commands.
        double lookahead_dist_;                      // Distance to look ahead of current pose.
        double max_angular_vel_;                     // max allowed rotation speed.
        double object_size_;                         // size of the object held by the robot, currently unused.
        double safe_dist_to_human_;                  // Safe horizontal distance required between the robot and detected human.
        double current_human_horizontal_dist_;       // Variable to store the current horizontal (y-axis) distance of the human extreme w.r.t robot.
        double executed_translation_;                // Variable to record the last executed object translation.
        double executed_rotation_;                   // Variable to record the last executed object rotation.
        double human_dist_threshold_;                // Threshold at which the robot should start making changes to the object pose if the human is close.
        double horizontal_dist_modifier_;            // Typically a multiplier to horizontal dist from camera to account for inaccuracies.
        bool nav_shift_enabled_;                     // Bool to inform if nav shift is enabled.
        bool obj_pose_action_executed_;              // Bool to check if object pose change has already started.
        bool reset_executed_;                        // Bool to check if the object pose was reset recently.
        rclcpp::Duration transform_tolerance_{0, 0}; // Transform duration difference tolerence, currently unused.

        nav_msgs::msg::Path global_plan_;

        double obj_max_translation_;   // Param to store max allowed object pose tranlsation.
        double obj_max_rotation_;      // Param to store max allowed object pose rotation.
        double obj_translation_slope_; // Param used to determine translation of object as (obj_translation_slope x required separation from human)
        double obj_orientation_slope_; // Param used to determine rotation of object as (obj_orientation_slope x required separation from human)

        // yarp stuff
        yarp::os::Network yarp_;        // Yarp network declaration to allow for sending messages to navigation and bimanual ports.
        yarp::os::Port bimanual_port_; // Port to the Bimanual Server to allow for held object pose change
        yarp::os::RpcClient human_extremes_port_; //Port to connect to the perception module to receive human pose and extremes data.
        yarp::os::BufferedPort<yarp::os::Bottle> nav_shift_port_;         // Port to path converter to shift planned path
        yarp::os::BufferedPort<yarp::os::Bottle> direct_human_data_port_; // Port to directly read human data
        ControlInterface bimanual_client_;                                // Thrift interface for peforming grasp actions that cause object pose change
        std::string nav_shift_port_name_;  // Name of the port to send path shift command
        std::string bimanual_server_name_; // Name of the bimanual server.
    };

} // end of namespace

#endif // ERGOCUB_LOCAL_HUMAN_AVOIDANCE_HPP_