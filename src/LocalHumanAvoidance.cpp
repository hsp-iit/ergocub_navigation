/**
 *
 * @Author: Vignesh Sushrutha Raghavan
 * @Date  : June 2024
 * @Brief : Full declaration of the human avoidance plugin
 *
 */
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "LocalHumanAvoidance/LocalHumanAvoidance.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;

namespace ergocub_local_human_avoidance
{
  void HumanAvoidanceController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                                           std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
                                           const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    //Setup Ros Params and variables. Based on Pure Pursuit Controller from nav2_tutorials.
    node_ = parent;

    auto node = node_.lock();

    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    declare_parameter_if_not_declared(node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.2));
    declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.4));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(node, plugin_name_ + ".object_size", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".safe_dist_to_human", rclcpp::ParameterValue(0.30));
    declare_parameter_if_not_declared(node, plugin_name_ + ".human_left_frame", rclcpp::ParameterValue("human_left_frame"));
    declare_parameter_if_not_declared(node, plugin_name_ + ".human_right_frame", rclcpp::ParameterValue("human_right_frame"));
    declare_parameter_if_not_declared(node, plugin_name_ + ".human_tf_base_frame", rclcpp::ParameterValue("head_laser_frame"));
    declare_parameter_if_not_declared(node, plugin_name_ + ".object_max_translation", rclcpp::ParameterValue(0.2));
    declare_parameter_if_not_declared(node, plugin_name_ + ".object_max_rotation", rclcpp::ParameterValue(0.45));
    declare_parameter_if_not_declared(node, plugin_name_ + ".object_translation_slope", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".object_orientation_slope", rclcpp::ParameterValue(3.0));

    node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
    node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
    node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
    double transform_tolerance;
    node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
    node->get_parameter(plugin_name_ + ".object_size", object_size_);
    node->get_parameter(plugin_name_ + ".safe_dist_to_human", safe_dist_to_human_);
    node->get_parameter(plugin_name_ + ".human_left_frame", human_left_frame_);
    node->get_parameter(plugin_name_ + ".human_right_frame", human_right_frame_);
    node->get_parameter(plugin_name_ + ".human_tf_base_frame", human_tf_base_frame_);
    node->get_parameter(plugin_name_ + ".object_max_translation", obj_max_translation_);
    node->get_parameter(plugin_name_ + ".object_max_rotation", obj_max_rotation_);
    node->get_parameter(plugin_name_ + ".object_translation_slope", obj_translation_slope_);
    node->get_parameter(plugin_name_ + ".object_orientation_slope", obj_orientation_slope_);

    global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);

    //Setup Yarp Ports for connection to Bimanual Module and Nav Shift
    bimannual_port_.open("bimanual_nav_client");
    while (!yarp_.connect("bimanual_nav_client", "bimanual_server"))
    {
        std::cout << "Error! Could not connect to bimanual server\n";
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    bimanual_client_.yarp().attachAsClient(bimannual_port_);


  }

  void HumanAvoidanceController::cleanup()
  {
    RCLCPP_INFO(
        logger_,
        "Cleaning up controller: %s of type local_human_avoidance",
        plugin_name_.c_str());
    global_pub_.reset();
  }

  void HumanAvoidanceController::activate()
  {
    RCLCPP_INFO(
        logger_,
        "Activating controller: %s of type local_human_avaoidance\"  %s",
        plugin_name_.c_str());
    global_pub_->on_activate();
  }

  void HumanAvoidanceController::deactivate()
  {
    RCLCPP_INFO(
        logger_,
        "Dectivating controller: %s of type pure_pursuit_controller::PurePursuitController\"  %s",
        plugin_name_.c_str(), plugin_name_.c_str());
    global_pub_->on_deactivate();
  }

  geometry_msgs::msg::TwistStamped HumanAvoidanceController::computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &velocity,
      nav2_core::GoalChecker *goal_checker)
  {
    geometry_msgs::msg::TwistStamped cmd_vel;
    geometry_msgs::msg::TransformStamped left_transform;
    try
    {
      left_transform = tf_->lookupTransform(
          human_tf_base_frame_, human_left_frame_,
          clock_->now(), rclcpp::Duration::from_seconds(0.02));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_INFO(
          logger_, "Could not transform %s to %s: %s",
          human_tf_base_frame_.c_str(), human_left_frame_.c_str(), ex.what());
      return cmd_vel;
    }

    geometry_msgs::msg::TransformStamped right_transform;
    try
    {
      right_transform = tf_->lookupTransform(
          human_tf_base_frame_, human_right_frame_,
          clock_->now(), rclcpp::Duration::from_seconds(0.02));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_INFO(
          logger_, "Could not transform %s to %s: %s",
          human_tf_base_frame_.c_str(), human_right_frame_.c_str(), ex.what());
      return cmd_vel;
    }

    RCLCPP_INFO(
        logger_,
        "Detected Humans Close By");
    double dist_left = pow(left_transform.transform.translation.x, 2) + pow(left_transform.transform.translation.y, 2);
    double dist_right = pow(right_transform.transform.translation.x, 2) + pow(right_transform.transform.translation.y, 2);

    double horizontal_min = (dist_left < dist_right) ? left_transform.transform.translation.y : right_transform.transform.translation.y;
    double total_min = (dist_left < dist_right) ? dist_left : dist_right;
    if (total_min < 3.0)
    {
      // std::cout<"Got here \n =======================================\n";
      if(std::fabs(horizontal_min)-safe_dist_to_human_>0)
      {

        std::ostringstream bimanual_msg;
        double req_obj_translation = obj_translation_slope_* (horizontal_min -((horizontal_min<0)?1.0:-1.0)*safe_dist_to_human_);
        double req_obj_orientation = obj_orientation_slope_* (horizontal_min -((horizontal_min<0)?1.0:-1.0)*safe_dist_to_human_);

        bimanual_msg<<"0.0, "<<req_obj_translation<<", 0.0, "<<req_obj_orientation;
        bimanual_client_.perform_grasp_action(bimanual_msg.str());


      }
      
      else
      {
        bimanual_client_.perform_grasp_action("reset");
      }

      return cmd_vel;
    }
    else
    {
      return cmd_vel;
    }
  }

  void HumanAvoidanceController::setPlan(const nav_msgs::msg::Path &path)
  {
    global_pub_->publish(path);
    global_plan_ = path;
  }

  void HumanAvoidanceController::setSpeedLimit(const double &speed_limit, const bool &percentage)
  {
    (void)speed_limit;
    (void)percentage;
  }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ergocub_local_human_avoidance::HumanAvoidanceController, nav2_core::Controller)