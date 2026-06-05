/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "navigation/head_orientation_controller/head_orientation_controller.hpp"

#include <algorithm>
#include <cmath>

using std::placeholders::_1;

HeadOrientationController::HeadOrientationController(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("head_orientation_controller_node", options)
{
    declare_parameter("plan_topic",        "/plan");
    declare_parameter("robot_base_frame",  "geometric_unicycle");
    declare_parameter("map_frame",         "map");
    declare_parameter("lookahead_size",    2.0);
    declare_parameter("update_rate_hz",    10.0);
    declare_parameter("pitch_angle_deg",   15.0);
    declare_parameter("max_yaw_deg",       45.0);
    declare_parameter("max_pitch_deg",     10.0);
    declare_parameter("min_pitch_deg",     -30.0);
    declare_parameter("head_rpc_server",   "/mc-ergocub-head-controller/rpc:i");
    declare_parameter("head_rpc_client",   "/head_orientation_controller/rpc:o");
    declare_parameter("plan_timeout_sec",  5.0);

    m_tf_buffer   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
}

HeadOrientationController::~HeadOrientationController()
{
    closeYarp();
}

HeadOrientationController::CallbackReturn
HeadOrientationController::on_configure(const rclcpp_lifecycle::State &)
{
    m_plan_topic        = get_parameter("plan_topic").as_string();
    m_robot_base_frame  = get_parameter("robot_base_frame").as_string();
    m_map_frame         = get_parameter("map_frame").as_string();
    m_lookahead_size    = get_parameter("lookahead_size").as_double();
    m_update_rate_hz    = get_parameter("update_rate_hz").as_double();
    m_pitch_angle_deg   = get_parameter("pitch_angle_deg").as_double();
    m_max_yaw_deg       = get_parameter("max_yaw_deg").as_double();
    m_max_pitch_deg     = get_parameter("max_pitch_deg").as_double();
    m_min_pitch_deg     = get_parameter("min_pitch_deg").as_double();
    m_head_rpc_server   = get_parameter("head_rpc_server").as_string();
    m_head_rpc_client   = get_parameter("head_rpc_client").as_string();
    m_plan_timeout_sec  = get_parameter("plan_timeout_sec").as_double();

    RCLCPP_INFO(get_logger(),
        "Configuring — plan: %s | base: %s | map: %s | lookahead: %.2f m | "
        "rate: %.1f Hz | pitch: %.1f° | yaw_max: %.1f°",
        m_plan_topic.c_str(), m_robot_base_frame.c_str(), m_map_frame.c_str(),
        m_lookahead_size, m_update_rate_hz, m_pitch_angle_deg, m_max_yaw_deg);

    m_plan_sub = create_subscription<nav_msgs::msg::Path>(
        m_plan_topic, 10, std::bind(&HeadOrientationController::planCallback, this, _1));

    m_nav_status_sub = create_subscription<action_msgs::msg::GoalStatusArray>(
        "navigate_to_pose/_action/status",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&HeadOrientationController::navStatusCallback, this, _1));

    if (!m_rpc_client.open(m_head_rpc_client))
    {
        RCLCPP_ERROR(get_logger(), "Cannot open YARP RPC client port: %s", m_head_rpc_client.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        return CallbackReturn::FAILURE;
    }

    if (!yarp::os::Network::connect(m_head_rpc_client, m_head_rpc_server))
    {
        RCLCPP_ERROR(get_logger(), "Cannot connect YARP %s -> %s (is mc-ergocub-head-controller running?)",
            m_head_rpc_client.c_str(), m_head_rpc_server.c_str());
        m_rpc_client.close();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        return CallbackReturn::FAILURE;
    }

    m_yarp_connected = true;
    RCLCPP_INFO(get_logger(), "YARP RPC connected: %s -> %s",
        m_head_rpc_client.c_str(), m_head_rpc_server.c_str());

    return CallbackReturn::SUCCESS;
}

HeadOrientationController::CallbackReturn
HeadOrientationController::on_activate(const rclcpp_lifecycle::State &)
{
    auto period = std::chrono::duration<double>(1.0 / m_update_rate_hz);
    m_timer = create_wall_timer(period,
        std::bind(&HeadOrientationController::timerCallback, this));

    RCLCPP_INFO(get_logger(), "Activated — gaze tracking at %.1f Hz", m_update_rate_hz);
    return CallbackReturn::SUCCESS;
}

HeadOrientationController::CallbackReturn
HeadOrientationController::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating");
    if (m_timer) { m_timer->cancel(); m_timer.reset(); }
    if (m_yarp_connected) { sendGoHome(); }
    return CallbackReturn::SUCCESS;
}

HeadOrientationController::CallbackReturn
HeadOrientationController::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning up");
    m_plan_sub.reset();
    m_nav_status_sub.reset();
    m_navigation_active = false;
    {
        std::lock_guard<std::mutex> lock(m_plan_mutex);
        m_latest_plan.reset();
    }
    closeYarp();
    return CallbackReturn::SUCCESS;
}

HeadOrientationController::CallbackReturn
HeadOrientationController::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Shutting down");
    if (m_timer) { m_timer->cancel(); m_timer.reset(); }
    closeYarp();
    return CallbackReturn::SUCCESS;
}

HeadOrientationController::CallbackReturn
HeadOrientationController::on_error(const rclcpp_lifecycle::State &)
{
    RCLCPP_FATAL(get_logger(), "Error state — closing YARP");
    closeYarp();
    return CallbackReturn::SUCCESS;
}

// Plan callback — cache under mutex
void HeadOrientationController::planCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_plan_mutex);
    m_latest_plan = msg;
    m_last_plan_time = this->now();
    if (!msg->poses.empty()) {
        m_navigation_active = true;
    }
}

// Nav2 action status callback for returning head home
void HeadOrientationController::navStatusCallback(
    const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
    if (msg->status_list.empty()) { return; }

    const auto status = msg->status_list.back().status;
    switch (status)
    {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
        // Triggers for all STATUS_SUCCEEDED STATUS_CANCELED STATUS_ABORTED
        if (m_navigation_active)
        {
            // To avoid multiple triggers
            m_navigation_active = false;
            RCLCPP_INFO(get_logger(),
                "Navigation ended (status %d): returning head home", status);
            sendGoHome();
        }
        break;
    default:    //else
        m_navigation_active = true;
        break;
    }
}

// Timer callback: core algorithm
void HeadOrientationController::timerCallback()
{
    // 1. Grab the latest plan under the mutex, then release
    nav_msgs::msg::Path::SharedPtr plan;
    {
        std::lock_guard<std::mutex> lock(m_plan_mutex);
        plan = m_latest_plan;
    }

    if (!plan || plan->poses.empty())
    {
        sendGoHome();
        return;
    }

    // Primary check: action status callback cleared this flag when goal ended.
    if (!m_navigation_active)
    {
        return;
    }

    // Safety fallback: if the plan stops being published (e.g. Nav2 crashed)
    // but the action status never fired, return home after the timeout.
    const double age = (this->now() - m_last_plan_time).seconds();
    if (age > m_plan_timeout_sec)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Plan stale (%.1f s) with no action status update — returning home", age);
        m_navigation_active = false;
        sendGoHome();
        return;
    }

    // Guard: plan must be in the expected map frame
    if (!plan->header.frame_id.empty() && plan->header.frame_id != m_map_frame)
    {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
            "Plan frame '%s' != expected '%s' — skipping",
            plan->header.frame_id.c_str(), m_map_frame.c_str());
        return;
    }

    // 2. TF lookup: map -> robot base frame
    geometry_msgs::msg::TransformStamped tf_map_to_robot;
    try
    {
        tf_map_to_robot = m_tf_buffer->lookupTransform(
            m_robot_base_frame, m_map_frame, rclcpp::Time(0));
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "TF lookup failed (%s) — keeping current head pose", ex.what());
        return;
    }

    // 3. Find lookahead point: first path point that exits the lookahead square
    //    Uses L∞ norm: max(|x|, |y|) > half_size
    const double half_size = m_lookahead_size / 2.0;
    geometry_msgs::msg::Point lookahead_pt;
    bool found_exit = false;

    for (const auto & stamped_pose : plan->poses)
    {
        geometry_msgs::msg::PoseStamped pt_robot;
        tf2::doTransform(stamped_pose, pt_robot, tf_map_to_robot);

        const double x = pt_robot.pose.position.x;
        const double y = pt_robot.pose.position.y;

        if (std::max(std::abs(x), std::abs(y)) > half_size)
        {
            lookahead_pt = pt_robot.pose.position;
            found_exit = true;
            break;
        }
    }

    // Fallback: entire path inside the square → use the goal (last point)
    if (!found_exit)
    {
        geometry_msgs::msg::PoseStamped last_robot;
        tf2::doTransform(plan->poses.back(), last_robot, tf_map_to_robot);
        lookahead_pt = last_robot.pose.position;
    }

    // 4. Compute yaw, clamp to [-max_yaw, +max_yaw]
    double yaw_rad = std::atan2(lookahead_pt.y, lookahead_pt.x);
    const double max_yaw_rad = m_max_yaw_deg * M_PI / 180.0;
    yaw_rad = std::clamp(yaw_rad, -max_yaw_rad, max_yaw_rad);

    // 5. Compute pitch (positive = downward), clamp
    double pitch_rad = m_pitch_angle_deg * M_PI / 180.0;
    const double max_pitch_rad = m_max_pitch_deg * M_PI / 180.0;
    const double min_pitch_rad = m_min_pitch_deg * M_PI / 180.0;
    pitch_rad = std::clamp(pitch_rad, min_pitch_rad, max_pitch_rad);

    // 6. Build R = Rz(yaw) × Ry(−pitch)
    //    Columns are head_x, head_y, head_z expressed in robot root frame.
    //    Identity = head looking forward (+x), identity = home position.
    //
    //    r11=cψ·cθ   r12=−sψ   r13=cψ·sθ
    //    r21=sψ·cθ   r22= cψ   r23=sψ·sθ
    //    r31=  −sθ   r32=  0   r33=  cθ
    const double cy = std::cos(yaw_rad),   sy = std::sin(yaw_rad);
    const double cp = std::cos(pitch_rad), sp = std::sin(pitch_rad);

    const double r11 = cy * cp,  r12 = -sy,  r13 = cy * sp;
    const double r21 = sy * cp,  r22 =  cy,  r23 = sy * sp;
    const double r31 =     -sp,  r32 =   0,  r33 =      cp;

    // 7. Send to head controller
    sendOrientationMatrix(r11, r12, r13,
                          r21, r22, r23,
                          r31, r32, r33);
}

// YARP helpers
bool HeadOrientationController::sendOrientationMatrix(
    double r11, double r12, double r13,
    double r21, double r22, double r23,
    double r31, double r32, double r33)
{
    if (!m_yarp_connected) { return false; }

    yarp::os::Bottle cmd;
    cmd.addString("setOrientationFlat");
    cmd.addFloat64(r11); cmd.addFloat64(r12); cmd.addFloat64(r13);
    cmd.addFloat64(r21); cmd.addFloat64(r22); cmd.addFloat64(r23);
    cmd.addFloat64(r31); cmd.addFloat64(r32); cmd.addFloat64(r33);

    const bool ok = m_rpc_client.write(cmd);
    if (!ok)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "YARP write failed for setOrientationFlat");
    }
    return ok;
}

// Home the head looking straight (0, 0, 0)
bool HeadOrientationController::sendGoHome()
{
    if (!m_yarp_connected) { return false; }

    yarp::os::Bottle cmd;
    cmd.addString("goHome");
    const bool ok = m_rpc_client.write(cmd);
    if (!ok)
    {
        RCLCPP_WARN(get_logger(), "YARP write failed for goHome");
    }
    return ok;
}

bool HeadOrientationController::sendStop()
{
    if (!m_yarp_connected) { return false; }

    yarp::os::Bottle cmd;
    cmd.addString("stop");
    const bool ok = m_rpc_client.write(cmd);
    if (!ok)
    {
        RCLCPP_WARN(get_logger(), "YARP write failed for stop");
    }
    return ok;
}

void HeadOrientationController::closeYarp()
{
    if (m_yarp_connected)
    {
        sendStop();
        m_rpc_client.close();
        m_yarp_connected = false;
    }
}


int main(int argc, char ** argv)
{
    // Init middlewares
    rclcpp::init(argc, argv);
    yarp::os::Network yarp;

    if (rclcpp::ok())
    {
        rclcpp::executors::SingleThreadedExecutor executor;
        rclcpp::NodeOptions options;
        auto node = std::make_shared<HeadOrientationController>(options);
        executor.add_node(node->get_node_base_interface());
        executor.spin();
    }

    RCLCPP_INFO(rclcpp::get_logger("head_orientation_controller"), "Shutting down");
    rclcpp::shutdown();
    return 0;
}
