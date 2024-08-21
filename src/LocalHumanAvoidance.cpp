/**
 *
 * @Author: Vignesh Sushrutha Raghavan
 * @Date  : June 2024
 * @Brief : Full declaration of the human avoidance plugin
 *
 */
#include <chrono>
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "LocalHumanAvoidance/LocalHumanAvoidance.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;
using namespace std::chrono_literals;

namespace ergocub_local_human_avoidance
{
  void HumanAvoidanceController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                                           std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
                                           const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    // Setup Ros Params and variables. Based on Pure Pursuit Controller from nav2_tutorials.
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
    declare_parameter_if_not_declared(node, plugin_name_ + ".bimanual_manipulation_server", rclcpp::ParameterValue("/Components/Manipulation"));
    declare_parameter_if_not_declared(node, plugin_name_ + ".nav_shift_port_name", rclcpp::ParameterValue("/path_converter/shift_command:i"));

    declare_parameter_if_not_declared(node, plugin_name_ + ".object_max_translation", rclcpp::ParameterValue(0.2));
    declare_parameter_if_not_declared(node, plugin_name_ + ".object_max_rotation", rclcpp::ParameterValue(0.45));
    declare_parameter_if_not_declared(node, plugin_name_ + ".object_translation_slope", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".object_orientation_slope", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".human_distance_threshold", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".horizontal_dist_modifier", rclcpp::ParameterValue(2.0));

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
    node->get_parameter(plugin_name_ + ".bimanual_manipulation_server", bimanual_server_name_);

    node->get_parameter(plugin_name_ + ".nav_shift_port_name", nav_shift_port_name_);
    node->get_parameter(plugin_name_ + ".object_max_translation", obj_max_translation_);
    node->get_parameter(plugin_name_ + ".object_max_rotation", obj_max_rotation_);
    node->get_parameter(plugin_name_ + ".object_translation_slope", obj_translation_slope_);
    node->get_parameter(plugin_name_ + ".object_orientation_slope", obj_orientation_slope_);
    node->get_parameter(plugin_name_ + ".human_distance_threshold", human_dist_threshold_);
    node->get_parameter(plugin_name_ + ".horizontal_dist_modifier", horizontal_dist_modifier_);

    human_extremes_client_ = node->create_client<ergocub_navigation::srv::GetHumanExtremes>("human_pose_service");
    global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);

    // Setup Yarp Ports for connection to Bimanual Module and Nav Shift
    bimannual_port_.open("/bimanual_nav_client");
    nav_shift_port_.open("/nav_shift_client");
    human_extremes_port_.open("/eCubperception/rpc:o");
    // Wait until bimanual server is available and connected to.
    while (!yarp_.connect("/bimanual_nav_client", bimanual_server_name_))
    {
      std::cout << "Error! Could not connect to bimanual server\n";
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    while (!yarp_.connect("/eCubperception/rpc:o", "/eCubperception/rpc:o"))
    {
      std::cout << "Error! Could not connect to human pose server\n";
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    bimanual_client_.yarp().attachAsClient(bimannual_port_);
    human_data_client_.yarp().attachAsClient(human_extremes_port_);

    // Setup variables to sequence bimanual actions
    current_human_horizontal_dist_ = 100.0;
    obj_pose_action_executed_ = false;
    reset_executed_ = false;

    // Connect to Nav shift. Not in a while loop so that it can be used when the robot is stationary for testing purposes.
    yarp::os::Network::connect("/nav_shift_client", nav_shift_port_name_);
    if (!yarp::os::Network::isConnected("/nav_shift_client", nav_shift_port_name_))
    {
      std::cout << "Error! Could not connect to Nav Shift Port\n";
      nav_shift_enabled_ = false;
    }
    else
      nav_shift_enabled_ = true;
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
        "Activating controller: %s of type local_human_avaoidance",
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

    (void)velocity;
    (void)goal_checker;
    //(void)pose;

    // Check Once again for connection to nav shift in path converter.
    if (!yarp::os::Network::isConnected("/nav_shift_client", nav_shift_port_name_))
    {
      std::cout << "Error! Could not connect to Nav Shift Port\n";
      nav_shift_enabled_ = false;
    }
    else
      nav_shift_enabled_ = true;

    // Fake message to be published for now.
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.twist.linear.x = 0.1;

    // Get transforms to the left and right frames attached to the detected human.
    /*geometry_msgs::msg::TransformStamped left_transform;
    try
    {
      left_transform = tf_->lookupTransform(
          human_tf_base_frame_, human_left_frame_,
          tf2::TimePointZero);
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
          tf2::TimePointZero);
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
        "Detected Humans Close By");*/
    /*bool received_response = false;
    auto request = std::make_shared<ergocub_navigation::srv::GetHumanExtremes::Request>();
    request->request = true;
    auto result = human_extremes_client_->async_send_request(request);
    if (result.wait_for(1s) == std::future_status::ready)
    {
      received_response = true;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");
      return cmd_vel;
    }*/

    auto human_position = human_data_client_.get_human_position();
    auto human_extremes = human_data_client_.get_human_occupancy();

    if (human_position.data()[2] != -1)
    {
      geometry_msgs::msg::TransformStamped left_transform, right_transform;
      left_transform.transform.translation.x = human_position.data()[2];
      left_transform.transform.translation.y = -human_position.data()[0] - human_extremes.data()[0];
      right_transform.transform.translation.x = human_position.data()[2];
      right_transform.transform.translation.y = -human_position.data()[0] - human_extremes.data()[1];

      // Determine the closest frame to the robot and get the horizontal ditance of the human frames w.r.t to the robot.
      double dist_left = pow(left_transform.transform.translation.x, 2) + pow(left_transform.transform.translation.y, 2);
      double dist_right = pow(right_transform.transform.translation.x, 2) + pow(right_transform.transform.translation.y, 2);

      double horizontal_min = (dist_left < dist_right) ? left_transform.transform.translation.y : right_transform.transform.translation.y;
      double total_min = (dist_left < dist_right) ? dist_left : dist_right;
      RCLCPP_INFO(
          logger_,
          "Distances %f, %f, %f, %f", total_min, horizontal_min, dist_left, dist_right);
      // double diff = rclcpp::Time(result.get()->left_transform.header.stamp.sec, result.get()->left_transform.header.stamp.nanosec).seconds() - clock_->now().seconds();

      /* Current logic : Named SimpleAvoidance checks if human detected is within a distance threshold and if the received transform timeis within
       * 1.5 seconds of now. Also check if the human is one side of the robot cmpletely and  not directly in front of the robot. If the human is
       * is directly in front of the robot don't do anything. A stop command will be sent soon.
       */

      if (pow(total_min, 0.5) < human_dist_threshold_ && (left_transform.transform.translation.y * right_transform.transform.translation.y) >= 0)
      {
        RCLCPP_INFO(
            logger_,
            "Human is really close in front");
        horizontal_min *= horizontal_dist_modifier_;
        if (std::fabs(horizontal_min) - (safe_dist_to_human_) < 0)
        {
          RCLCPP_INFO(
              logger_,
              "Human is really close on the side %f, %f", safe_dist_to_human_, horizontal_min);
          // Calculate object pose changes.
          std::ostringstream bimanual_msg;
          double req_obj_translation = obj_translation_slope_ * (horizontal_min + ((horizontal_min < 0) ? 1.0 : -1.0) * (safe_dist_to_human_));
          double req_obj_orientation = obj_orientation_slope_ * (horizontal_min + ((horizontal_min < 0) ? 1.0 : -1.0) * (safe_dist_to_human_));
          // Check if calculated object pose changes are withing the limit.
          req_obj_translation = (req_obj_translation < -obj_max_translation_) ? -obj_max_translation_ : req_obj_translation;
          req_obj_translation = (req_obj_translation > obj_max_translation_) ? obj_max_translation_ : req_obj_translation;

          req_obj_orientation = (req_obj_orientation < -obj_max_rotation_) ? -obj_max_rotation_ : req_obj_orientation;
          req_obj_orientation = (req_obj_orientation > obj_max_rotation_) ? obj_max_rotation_ : req_obj_orientation;
          // Create the bimanual message to send to Bimanual Server.
          bimanual_msg << "0.0, " << req_obj_translation << ", 0.0, " << req_obj_orientation;
          if (std::fabs(req_obj_orientation) >= 0.04 || std::fabs(req_obj_translation) >= 0.01)
          {
            yarp::os::Bottle bimanual_bottle;
            yarp::os::Bottle shift_bottle = nav_shift_port_.prepare();
            shift_bottle.clear();
            bimanual_bottle.clear();
            bimanual_bottle.addString(bimanual_msg.str());
            if (!obj_pose_action_executed_)
            {
              // If object pose change is being executed for the first time, setup sequencing variables.
              current_human_horizontal_dist_ = horizontal_min;
              obj_pose_action_executed_ = true;
              bimanual_client_.perform_grasp_action(bimanual_msg.str().c_str());
              reset_executed_ = false;
              RCLCPP_INFO(
                  logger_,
                  "Sending To Bimanual %s", bimanual_msg.str().c_str());
              if (nav_shift_enabled_) // calculate nav shift if needed.
              {
                if (std::fabs(horizontal_min) < 0.10)
                {
                  int shift_required = ((horizontal_min < 0) ? 1 : 0);
                  shift_bottle.addInt32(1);
                  shift_bottle.addInt32(shift_required);
                  nav_shift_port_.write();
                  RCLCPP_INFO(
                      logger_,
                      "Sending Nav Shift %d", shift_required);
                }
              }
            }
            else
            {
              /*Send another bimanual message only if deteted human horizontal dists has changed w.r.t to the robot.
               * Otherise skip to avoid sending too many messages to bimanual
               */
              if (std::fabs(current_human_horizontal_dist_) - std::fabs(horizontal_min) > 0.01)
              {
                current_human_horizontal_dist_ = horizontal_min;
                bimanual_client_.perform_grasp_action(bimanual_msg.str().c_str());
                reset_executed_ = false;
                RCLCPP_INFO(
                    logger_,
                    "Sending To Bimanual %s", bimanual_msg.str().c_str());
                if (nav_shift_enabled_)
                {
                  if (std::fabs(horizontal_min) < 0.10)
                  {
                    int shift_required = ((horizontal_min < 0) ? 1 : 0);
                    shift_bottle.addInt32(1);
                    shift_bottle.addInt32(shift_required);
                    nav_shift_port_.write();
                    RCLCPP_INFO(
                        logger_,
                        "Sending Nav Shift %d", shift_required);
                  }
                }
              }
            }
          }
        }
        else
        {
          if (obj_pose_action_executed_)
          { // If human is at a safe distance reset object pose.
            if (!reset_executed_)
            {
              reset_executed_ = true;
              bimanual_client_.perform_grasp_action("reset");
              obj_pose_action_executed_ = false;
              current_human_horizontal_dist_ = 100.0;
            }
          }
        }

        return cmd_vel;
      }
      else
      { // If no human is detected or if human is too far away execute reset accordingly.

        if (!reset_executed_)
        {
          reset_executed_ = true;
          bimanual_client_.perform_grasp_action("reset");
        }
        obj_pose_action_executed_ = false;
        current_human_horizontal_dist_ = 100.0;

        return cmd_vel;
      }
    }
    else
      return cmd_vel;
  }

  void HumanAvoidanceController::setPlan(const nav_msgs::msg::Path &path) // Plugin Functions. Unmodified for now.
  {
    global_pub_->publish(path);
    global_plan_ = path;
  }

  void HumanAvoidanceController::setSpeedLimit(const double &speed_limit, const bool &percentage) // Plugin Functions Unmodified for now.
  {
    (void)speed_limit;
    (void)percentage;
  }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ergocub_local_human_avoidance::HumanAvoidanceController, nav2_core::Controller)